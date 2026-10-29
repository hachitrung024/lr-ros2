"""Offline look-ahead on the host, independent of the causal telemetry cursor."""

from bisect import bisect_left, bisect_right
import math

from .path_protocol import MAX_HORIZON_NS, MAX_POINTS, PathPoint, Snapshot, fragments
from .records import finite


def quaternion(roll, pitch, yaw):
    """Convert MAVLink NED/FRD attitude to ROS ENU/FLU XYZW."""
    roll, pitch, yaw = roll / 2., -pitch / 2., (math.pi / 2. - yaw) / 2.
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def slerp(left, right, ratio):
    dot = sum(a * b for a, b in zip(left, right))
    if dot < 0:
        right, dot = tuple(-v for v in right), -dot
    dot = max(-1., min(1., dot))
    if dot > .9995:
        values = tuple(a + ratio * (b - a) for a, b in zip(left, right))
    else:
        angle = math.acos(dot)
        a, b = math.sin((1. - ratio) * angle), math.sin(ratio * angle)
        values = tuple(a * x + b * y for x, y in zip(left, right))
    norm = math.sqrt(sum(v * v for v in values))
    return tuple(v / norm for v in values)


def ecef(lla):
    lat, lon = map(math.radians, lla[:2])
    altitude = lla[2]
    radius = 6378137. / math.sqrt(1. - 6.69437999014e-3 * math.sin(lat) ** 2)
    return ((radius + altitude) * math.cos(lat) * math.cos(lon),
            (radius + altitude) * math.cos(lat) * math.sin(lon),
            (radius * (1. - 6.69437999014e-3) + altitude) * math.sin(lat))


class FuturePath:
    """Select bounded future GPS/attitude evidence without advancing telemetry."""

    def __init__(self, recording, *, radius_m=15., step_m=.2, rate_hz=5.):
        if not all(math.isfinite(v) and v > 0 for v in (radius_m, step_m, rate_hz)) or rate_hz > 5.:
            raise ValueError('Invalid future path radius, step or rate')
        self.radius, self.step = radius_m, step_m
        self.period_ns = round(1e9 / rate_hz)
        self.next_stamp = 0
        self.sequence = 0
        self.gps, self.attitude = [], []
        for record in recording.records:
            v = record.values
            if record.kind == 'gps' and all(finite(v.get(k)) for k in ('lat', 'lon', 'alt')):
                if not (int(v.get('fix_type') or 0) >= 3 and 0 < abs(v['lat']) <= 90
                        and 0 < abs(v['lon']) <= 180
                        and -2147483.648 <= v['alt'] <= 2147483.647):
                    continue
                lla = tuple(float(v[k]) for k in ('lat', 'lon', 'alt'))
                self.gps.append((record.stamp_us * 1000, lla, v.get('vel')))
            elif record.kind == 'attitude' and all(
                    finite(v.get(k)) for k in ('roll', 'pitch', 'yaw')):
                self.attitude.append((record.stamp_us * 1000,
                                      quaternion(v['roll'], v['pitch'], v['yaw'])))
        if len(self.gps) < 2 or len(self.attitude) < 2:
            raise ValueError('Future path requires two valid GPS and attitude samples')
        self.gps_stamps = [row[0] for row in self.gps]
        self.attitude_stamps = [row[0] for row in self.attitude]
        origin = self.gps[0][1]
        self.origin_ecef = ecef(origin)
        self.lat, self.lon = map(math.radians, origin[:2])
        self.positions = [self.enu(row[1]) for row in self.gps]

    def enu(self, lla):
        dx, dy, dz = (v - o for v, o in zip(ecef(lla), self.origin_ecef))
        return (-math.sin(self.lon) * dx + math.cos(self.lon) * dy,
                -math.sin(self.lat) * math.cos(self.lon) * dx
                - math.sin(self.lat) * math.sin(self.lon) * dy + math.cos(self.lat) * dz)

    @staticmethod
    def bracket(stamps, stamp, *, max_gap_ns=None):
        right = bisect_left(stamps, stamp)
        if right < len(stamps) and stamps[right] == stamp:
            return right, right, 0.
        if right == 0 or right == len(stamps):
            edge = min(right, len(stamps) - 1)
            return (edge, edge, 0.) if abs(stamps[edge] - stamp) <= 1_000_000_000 else None
        left = right - 1
        span = stamps[right] - stamps[left]
        if max_gap_ns is not None and span > max_gap_ns:
            return None
        return left, right, (stamp - stamps[left]) / span

    def orientation(self, stamp):
        bracket = self.bracket(self.attitude_stamps, stamp)
        if bracket is None:
            return None
        left, right, ratio = bracket
        return slerp(self.attitude[left][1], self.attitude[right][1], ratio)

    @staticmethod
    def point(stamp, lla, orientation, live):
        return PathPoint(live.boot(stamp), round(lla[0] * 1e7), round(lla[1] * 1e7),
                         round(lla[2] * 1000), orientation)

    def select(self, stamp, live):
        bracket = self.bracket(self.gps_stamps, stamp, max_gap_ns=1_500_000_000)
        orientation = self.orientation(stamp)
        if bracket is None or orientation is None:
            return ()
        left, right, ratio = bracket
        lla = tuple(a + ratio * (b - a) for a, b in zip(self.gps[left][1], self.gps[right][1]))
        current = self.enu(lla)
        points = [self.point(stamp, lla, orientation, live)]
        selected, plausible, plausible_stamp, distance = current, current, stamp, 0.
        for index in range(bisect_right(self.gps_stamps, stamp), len(self.gps)):
            future, lla, speed = self.gps[index]
            if future - stamp > MAX_HORIZON_NS or (
                    self.gps_stamps[index] - self.gps_stamps[index - 1] > 1_500_000_000):
                break
            position = self.positions[index]
            elapsed = (future - plausible_stamp) / 1e9
            if math.dist(position, plausible) / elapsed > 5. or (finite(speed) and speed > 5.):
                continue
            plausible, plausible_stamp = position, future
            if finite(speed) and speed <= .1:
                continue
            step = math.dist(position, selected)
            boundary = math.dist(position, current) > self.radius or distance + step > 2 * self.radius
            if step < self.step and not boundary:
                continue
            orientation = self.orientation(future)
            if orientation is None:
                break
            points.append(self.point(future, lla, orientation, live))
            selected, distance = position, distance + step
            if boundary or len(points) >= MAX_POINTS:
                break
        return tuple(points) if len(points) >= 2 else ()

    def prepare(self, stamp_ns, live):
        if stamp_ns < self.next_stamp:
            return None
        self.next_stamp = stamp_ns + self.period_ns
        snapshot = Snapshot(live.session_id.hex, self.sequence, live.boot(stamp_ns),
                            self.select(stamp_ns, live))
        self.sequence += 1
        return fragments(snapshot)
