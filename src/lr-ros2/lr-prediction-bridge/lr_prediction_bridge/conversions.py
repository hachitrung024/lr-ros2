"""Stateful converters independent of ROS executors and publishers."""

from collections import deque
import math

from safety_perception_msgs.msg import (
    GeometryArray,
    GeometryStep,
    Point2D,
    RoverState,
    TrackedObject,
    TrackedObjectArray,
    Trajectory,
    TrajectoryStep,
)

from .detection3d_conversion import convert_detection3d_array
from .helpers import finite_difference, subsample_indices, yaw_from_quaternion


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def require_frame(header, expected):
    if header.frame_id.strip() != expected:
        raise ValueError(f"frame {header.frame_id!r} does not match {expected!r}")


class TrajectoryConverter:
    """Sample each accepted Path without inventing per-step timestamps."""

    def __init__(
        self,
        *,
        frame='map',
        horizon_steps=20,
        path_stride=1,
        output_dt_sec=0.25,
        min_distance_from_start_m=1.0,
        minimum_cycle_period_sec=0.25,
    ):
        if not 1 <= horizon_steps <= 20 or path_stride < 1:
            raise ValueError('horizon_steps must be 1..20 and path_stride positive')
        if not frame or not all(
            math.isfinite(v) and v >= 0
            for v in (output_dt_sec, min_distance_from_start_m, minimum_cycle_period_sec)
        ):
            raise ValueError('frame and trajectory sampling parameters are invalid')
        self.frame = frame
        self.horizon = horizon_steps
        self.stride = path_stride
        self.dt_ns = round(output_dt_sec * 1e9)
        self.min_distance = min_distance_from_start_m
        self.period_ns = round(minimum_cycle_period_sec * 1e9)
        self.trajectory_id = 0
        self.last_stamp = None

    def reset(self):
        # IDs must not be reused when an SVO is rewound.
        self.last_stamp = None

    def convert(self, path):
        require_frame(path.header, self.frame)
        now = stamp_ns(path.header.stamp)
        if not path.poses or now <= 0:
            return None
        if self.last_stamp is not None and 0 <= now - self.last_stamp < self.period_ns:
            return None
        origin = path.poses[0].pose.position
        start = next(
            (
                i
                for i, p in enumerate(path.poses)
                if self.min_distance <= 0
                or len(path.poses) == 1
                or math.hypot(p.pose.position.x - origin.x, p.pose.position.y - origin.y)
                >= self.min_distance
            ),
            None,
        )
        if start is None:
            return None
        poses = path.poses[start:]
        indices = self._indices(poses)
        out = Trajectory(header=path.header)
        for index in indices:
            pose = poses[index]
            if pose.header.frame_id and pose.header.frame_id != self.frame:
                raise ValueError('Path contains a pose in another frame')
            p, q = pose.pose.position, pose.pose.orientation
            norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
            if not math.isfinite(norm) or norm <= 1e-12:
                continue
            yaw = yaw_from_quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)
            if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
                continue
            out.steps.append(TrajectoryStep(step_id=len(out.steps), x=p.x, y=p.y, yaw=yaw))
        if not out.steps:
            return None
        self.trajectory_id += 1
        out.trajectory_id = self.trajectory_id
        self.last_stamp = now
        return out

    def _indices(self, poses):
        stamps = [stamp_ns(p.header.stamp) for p in poses]
        if (
            self.dt_ns > 0
            and len(stamps) > 1
            and stamps[-1] > stamps[0]
            and all(a <= b for a, b in zip(stamps, stamps[1:]))
        ):
            result = [0]
            for index, stamp in enumerate(stamps[1:], 1):
                if len(result) >= self.horizon:
                    break
                if stamp >= stamps[result[-1]] + self.dt_ns:
                    result.append(index)
            if result[-1] != len(poses) - 1 and len(result) < self.horizon:
                result.append(len(poses) - 1)
            return result
        return subsample_indices(len(poses), horizon_steps=self.horizon, stride=self.stride)


class StateConverter:
    """Preserve the three-pose finite-difference acceleration model."""

    def __init__(self, *, frame='map', min_dt_sec=1e-3, max_dt_sec=1.0):
        if not (0 < min_dt_sec <= max_dt_sec < math.inf):
            raise ValueError('invalid state sample dt bounds')
        self.frame, self.min_dt, self.max_dt = frame, min_dt_sec, max_dt_sec
        self.history = deque(maxlen=3)

    def reset(self):
        self.history.clear()

    def convert(self, message):
        require_frame(message.header, self.frame)
        p, q = message.pose.position, message.pose.orientation
        stamp = stamp_ns(message.header.stamp)
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            raise ValueError('non-finite pose')
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w <= 1e-12:
            raise ValueError('zero quaternion')
        if self.history and stamp <= self.history[-1][0]:
            self.reset()
        self.history.append((stamp, (p.x, p.y, p.z)))
        out = RoverState(header=message.header, pose=message.pose, pose_valid=True)
        velocity = None
        if len(self.history) >= 2:
            previous, latest = self.history[-2], self.history[-1]
            dt = (latest[0] - previous[0]) * 1e-9
            if self.min_dt <= dt <= self.max_dt:
                velocity = finite_difference(previous[1], latest[1], dt)
                out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = velocity
                out.twist_valid = True
        if len(self.history) == 3 and velocity is not None:
            first, middle, latest = self.history
            dt_first = (middle[0] - first[0]) * 1e-9
            dt_latest = (latest[0] - middle[0]) * 1e-9
            if self.min_dt <= dt_first <= self.max_dt:
                previous_velocity = finite_difference(first[1], middle[1], dt_first)
                acceleration = finite_difference(
                    previous_velocity, velocity, (dt_first + dt_latest) * 0.5
                )
                out.acceleration.linear.x, out.acceleration.linear.y, out.acceleration.linear.z = (
                    acceleration
                )
                out.acceleration_valid = True
        return out


def geometry_from_trajectory(
    trajectory, sampler, grid_header, *, allow_flat=False, flat_confidence=0.25
):
    """Keep sensor and trajectory timestamps separate in the existing message."""
    out = GeometryArray()
    out.header = grid_header if grid_header is not None else trajectory.header
    out.source_trajectory_id = trajectory.trajectory_id
    out.source_trajectory_stamp = trajectory.header.stamp
    for step in trajectory.steps:
        sample = None if sampler is None else sampler.sample(step.x, step.y)
        geom = GeometryStep(step_id=step.step_id)
        if sample is None or not sample.valid or sample.normal_xyz is None:
            if not allow_flat:
                continue
            geom.plane_id = f'flat-fallback-{step.step_id}'
            geom.normal.z = 1.0
            geom.confidence = float(flat_confidence)
            geom.confidence_valid = True
        else:
            geom.plane_id = sample.plane_id
            geom.normal.x, geom.normal.y, geom.normal.z = sample.normal_xyz
            geom.confidence_valid = sample.confidence is not None
            geom.confidence = float(sample.confidence or 0.0)
        out.steps.append(geom)
    return out


def objects_from_detections(message, frame, **transform):
    converted, stats = convert_detection3d_array(
        message.detections, stamp_ns(message.header.stamp), **transform
    )
    out = TrackedObjectArray()
    out.header.stamp = message.header.stamp
    out.header.frame_id = frame
    for item in converted:
        obj = TrackedObject(
            track_id=item.track_id,
            class_name=item.class_name,
            confidence=item.confidence,
            confidence_valid=item.confidence_valid,
            velocity_valid=False,
        )
        obj.footprint_polygon_xy = [Point2D(x=x, y=y) for x, y in item.footprint_polygon_xy]
        out.objects.append(obj)
    return out, stats
