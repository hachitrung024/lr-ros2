"""Causal, bounded GPS/attitude extrapolation; no database access."""

from bisect import bisect_right
from dataclasses import dataclass
import numpy as np

from lr_future_path.mavlink import PoseSample
from lr_future_path.trajectory import normalize_quaternion, quaternion_multiply


@dataclass(frozen=True)
class Gps:
    stamp_ns: int
    position: np.ndarray
    valid: bool = True


@dataclass(frozen=True)
class Attitude:
    stamp_ns: int
    orientation: np.ndarray
    angular_velocity: np.ndarray


class Estimator:
    def __init__(self, gps_age=0.75, gps_gap=1.5, max_speed=5.0, attitude_age=0.25):
        limits = (gps_age, gps_gap, max_speed, attitude_age)
        if not all(np.isfinite(v) and v > 0 for v in limits):
            raise ValueError("Estimator limits must be finite and positive")
        self.gps_age, self.gps_gap, self.max_speed, self.attitude_age = limits
        self.gps, self.attitude = [], []

    def reset(self):
        self.gps.clear()
        self.attitude.clear()

    @staticmethod
    def _insert(buffer, item):
        index = bisect_right([x.stamp_ns for x in buffer], item.stamp_ns)
        if index and buffer[index - 1].stamp_ns == item.stamp_ns:
            return False
        buffer.insert(index, item)
        cutoff = buffer[-1].stamp_ns - 10_000_000_000
        while buffer and (buffer[0].stamp_ns < cutoff or len(buffer) > 1000):
            buffer.pop(0)
        return True

    def add_gps(self, sample):
        return self._insert(self.gps, sample)

    def add_attitude(self, sample):
        return self._insert(self.attitude, sample)

    def estimate(self, stamp_ns):
        gi = bisect_right([x.stamp_ns for x in self.gps], stamp_ns)
        ai = bisect_right([x.stamp_ns for x in self.attitude], stamp_ns)
        if gi < 2 or ai < 1:
            return None, "waiting_samples", None
        old, gps = self.gps[gi - 2 : gi]
        att = self.attitude[ai - 1]
        ga = (stamp_ns - gps.stamp_ns) / 1e9
        gap = (gps.stamp_ns - old.stamp_ns) / 1e9
        aa = (stamp_ns - att.stamp_ns) / 1e9
        if not gps.valid or not old.valid:
            return None, "gps_no_fix", None
        if ga > self.gps_age or gap > self.gps_gap:
            return None, "gps_stale", None
        if aa > self.attitude_age:
            return None, "attitude_stale", None
        velocity = (gps.position - old.position) / gap
        if not np.all(np.isfinite(velocity)) or np.linalg.norm(velocity) > self.max_speed:
            return None, "gps_speed_rejected", None
        q, omega = att.orientation, att.angular_velocity
        if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 0.5:
            return None, "invalid_attitude", None
        if not np.all(np.isfinite(omega)):
            return None, "invalid_angular_velocity", None
        rate = np.linalg.norm(omega)
        dq = np.array([0.0, 0.0, 0.0, 1.0])
        if rate > 1e-12:
            angle = rate * aa
            dq = np.r_[omega / rate * np.sin(angle / 2), np.cos(angle / 2)]
        result = PoseSample(
            stamp_ns,
            gps.position + velocity * ga,
            normalize_quaternion(quaternion_multiply(q, dq)),
        )
        evidence = {
            "gps_stamps": [old.stamp_ns, gps.stamp_ns],
            "gps_positions": [old.position.tolist(), gps.position.tolist()],
            "attitude_stamp": att.stamp_ns,
            "attitude": q.tolist(),
            "angular_velocity": omega.tolist(),
        }
        return result, "valid", evidence
