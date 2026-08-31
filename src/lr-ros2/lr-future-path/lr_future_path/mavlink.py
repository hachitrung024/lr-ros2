"""Read and interpolate the rover's offline MAVLink SQLite trajectory."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sqlite3
from typing import Iterable
from urllib.parse import quote

import numpy as np

from .trajectory import (
    normalize_quaternion,
    quaternion_multiply,
    rotate_vector,
)


NANOSECONDS_PER_SECOND = 1_000_000_000
MICROSECONDS_TO_NANOSECONDS = 1_000
WGS84_A_M = 6_378_137.0
WGS84_E2 = 6.69437999014e-3

_REQUIRED_COLUMNS = {
    "gps": {
        "t_wall_epoch_us",
        "lat",
        "lon",
        "alt",
        "fix_type",
    },
    "attitude": {
        "t_wall_epoch_us",
        "roll",
        "pitch",
        "yaw",
    },
}


class MavlinkDataError(ValueError):
    """Indicate that a MAVLink database cannot provide a valid trajectory."""


@dataclass(frozen=True)
class SessionSummary:
    """Minimal time coverage used to match an SVO to a MAVLink session."""

    path: Path
    start_ns: int
    end_ns: int
    gps_start_ns: int
    gps_end_ns: int


@dataclass(frozen=True)
class PoseSample:
    """A timestamped pose represented in a local ROS ENU frame."""

    stamp_ns: int
    position: np.ndarray
    orientation_xyzw: np.ndarray


@dataclass(frozen=True)
class MavlinkTrajectory:
    """Filtered GPS positions and MAVLink attitudes in one local ENU frame."""

    path: Path
    gps_stamps_ns: np.ndarray
    gps_positions: np.ndarray
    attitude_stamps_ns: np.ndarray
    attitude_orientations_xyzw: np.ndarray

    def __post_init__(self) -> None:
        """Validate trajectory shapes and strict timestamp ordering."""
        gps_count = len(self.gps_stamps_ns)
        attitude_count = len(self.attitude_stamps_ns)
        if gps_count < 2:
            raise MavlinkDataError(
                "MAVLink trajectory needs at least two GPS samples"
            )
        if attitude_count < 2:
            raise MavlinkDataError(
                "MAVLink trajectory needs at least two attitude samples"
            )
        if self.gps_positions.shape != (gps_count, 3):
            raise MavlinkDataError("GPS positions must have shape (N, 3)")
        if self.attitude_orientations_xyzw.shape != (attitude_count, 4):
            raise MavlinkDataError(
                "attitude orientations must have shape (N, 4)"
            )
        if np.any(np.diff(self.gps_stamps_ns) <= 0):
            raise MavlinkDataError(
                "GPS timestamps must be strictly increasing"
            )
        if np.any(np.diff(self.attitude_stamps_ns) <= 0):
            raise MavlinkDataError(
                "attitude timestamps must be strictly increasing"
            )

    def pose_at(
        self,
        stamp_ns: int,
        *,
        max_gps_gap_ns: int,
        edge_tolerance_ns: int = NANOSECONDS_PER_SECOND,
    ) -> PoseSample | None:
        """Interpolate a pose, rejecting missing or stale GPS intervals."""
        position = _interpolate_position(
            self.gps_stamps_ns,
            self.gps_positions,
            stamp_ns,
            max_gap_ns=max_gps_gap_ns,
            edge_tolerance_ns=edge_tolerance_ns,
        )
        if position is None:
            return None
        orientation = _interpolate_orientation(
            self.attitude_stamps_ns,
            self.attitude_orientations_xyzw,
            stamp_ns,
            edge_tolerance_ns=edge_tolerance_ns,
        )
        if orientation is None:
            return None
        return PoseSample(
            stamp_ns=int(stamp_ns),
            position=position,
            orientation_xyzw=orientation,
        )

    def future_samples(
        self,
        stamp_ns: int,
        *,
        radius_m: float,
        step_m: float,
        max_gps_gap_ns: int,
        max_points: int,
        edge_tolerance_ns: int = NANOSECONDS_PER_SECOND,
    ) -> list[PoseSample]:
        """Sample a future GPS path beginning at the exact current pose."""
        if radius_m <= 0.0 or step_m <= 0.0:
            raise ValueError("radius_m and step_m must be positive")
        if max_gps_gap_ns <= 0:
            raise ValueError("max_gps_gap_ns must be positive")
        if max_points < 1:
            raise ValueError("max_points must be at least one")

        current = self.pose_at(
            stamp_ns,
            max_gps_gap_ns=max_gps_gap_ns,
            edge_tolerance_ns=edge_tolerance_ns,
        )
        if current is None:
            return []

        selected = [current]
        if max_points == 1:
            return selected
        first_future = int(np.searchsorted(
            self.gps_stamps_ns, stamp_ns, side="right"
        ))
        last_selected_position = current.position
        previous_position = current.position
        distance_since_sample = 0.0
        last_valid: PoseSample | None = None

        for index in range(first_future, len(self.gps_stamps_ns)):
            if index > 0 and (
                int(
                    self.gps_stamps_ns[index]
                    - self.gps_stamps_ns[index - 1]
                )
                > max_gps_gap_ns
            ):
                break
            sample_stamp = int(self.gps_stamps_ns[index])
            orientation = _interpolate_orientation(
                self.attitude_stamps_ns,
                self.attitude_orientations_xyzw,
                sample_stamp,
                edge_tolerance_ns=edge_tolerance_ns,
            )
            if orientation is None:
                break
            position = self.gps_positions[index]
            radial_distance = float(
                np.linalg.norm(position[:2] - current.position[:2])
            )
            if radial_distance > radius_m:
                break
            distance_since_sample += float(
                np.linalg.norm(position[:2] - previous_position[:2])
            )
            previous_position = position
            last_valid = PoseSample(sample_stamp, position, orientation)
            if distance_since_sample >= step_m:
                selected.append(last_valid)
                last_selected_position = position
                distance_since_sample = 0.0
                if len(selected) >= max_points:
                    return selected

        if (
            last_valid is not None
            and len(selected) < max_points
            and not np.array_equal(last_valid.position, last_selected_position)
        ):
            selected.append(last_valid)
        return selected


def _readonly_connection(path: Path) -> sqlite3.Connection:
    """Open SQLite without creating journal or shared-memory sidecars."""
    resolved = path.expanduser().resolve()
    encoded = quote(str(resolved), safe="/")
    return sqlite3.connect(
        f"file:{encoded}?mode=ro&immutable=1",
        uri=True,
    )


def _table_columns(
    connection: sqlite3.Connection, table: str
) -> set[str]:
    escaped = table.replace('"', '""')
    return {
        str(row[1])
        for row in connection.execute(f'PRAGMA table_info("{escaped}")')
    }


def inspect_session(path: str | Path) -> SessionSummary:
    """Validate a custom MAVLink database and return its time coverage."""
    database = Path(path).expanduser().resolve()
    if not database.is_file():
        raise MavlinkDataError(f"MAVLink database does not exist: {database}")
    try:
        with _readonly_connection(database) as connection:
            for table, required in _REQUIRED_COLUMNS.items():
                missing = required - _table_columns(connection, table)
                if missing:
                    raise MavlinkDataError(
                        f"{database}: table {table} is missing columns "
                        f"{sorted(missing)}"
                    )
            gps_range = connection.execute(
                """
                SELECT MIN(t_wall_epoch_us), MAX(t_wall_epoch_us)
                FROM gps
                WHERE lat IS NOT NULL AND lon IS NOT NULL
                  AND alt IS NOT NULL AND lat != 0 AND lon != 0
                  AND fix_type >= 3
                """
            ).fetchone()
            attitude_range = connection.execute(
                """
                SELECT MIN(t_wall_epoch_us), MAX(t_wall_epoch_us)
                FROM attitude
                WHERE roll IS NOT NULL AND pitch IS NOT NULL
                  AND yaw IS NOT NULL
                """
            ).fetchone()
    except sqlite3.Error as exception:
        raise MavlinkDataError(
            f"Cannot read MAVLink database {database}: {exception}"
        ) from exception
    if (
        gps_range is None
        or attitude_range is None
        or None in gps_range
        or None in attitude_range
    ):
        raise MavlinkDataError(
            f"MAVLink database has no valid GPS/attitude coverage: {database}"
        )
    gps_start_ns, gps_end_ns = (
        int(value) * MICROSECONDS_TO_NANOSECONDS for value in gps_range
    )
    attitude_start_ns, attitude_end_ns = (
        int(value) * MICROSECONDS_TO_NANOSECONDS
        for value in attitude_range
    )
    return SessionSummary(
        path=database,
        start_ns=min(gps_start_ns, attitude_start_ns),
        end_ns=max(gps_end_ns, attitude_end_ns),
        gps_start_ns=gps_start_ns,
        gps_end_ns=gps_end_ns,
    )


def discover_sessions(directory: str | Path) -> list[SessionSummary]:
    """Find and validate every MAVLink session database below a directory."""
    root = Path(directory).expanduser().resolve()
    if not root.is_dir():
        raise MavlinkDataError(f"MAVLink directory does not exist: {root}")
    summaries = []
    failures = []
    for path in sorted(root.rglob("session_mavlink.db")):
        try:
            summaries.append(inspect_session(path))
        except MavlinkDataError as exception:
            failures.append(str(exception))
    if not summaries:
        detail = f" ({'; '.join(failures)})" if failures else ""
        raise MavlinkDataError(
            f"No valid session_mavlink.db found below {root}{detail}"
        )
    return summaries


def select_session(
    clock_stamp_ns: int,
    *,
    directory: str | Path = "mavlink",
    explicit_path: str | Path = "",
    tolerance_ns: int = 60 * NANOSECONDS_PER_SECOND,
) -> SessionSummary:
    """Select the unique session whose start and coverage match SVO time."""
    if tolerance_ns < 0:
        raise ValueError("tolerance_ns must not be negative")
    if str(explicit_path).strip():
        candidates = [inspect_session(explicit_path)]
    else:
        candidates = discover_sessions(directory)
    matching = [
        summary
        for summary in candidates
        if summary.start_ns - tolerance_ns
        <= clock_stamp_ns
        <= summary.end_ns + tolerance_ns
        and abs(clock_stamp_ns - summary.start_ns) <= tolerance_ns
    ]
    if not matching:
        locations = ", ".join(str(item.path) for item in candidates)
        raise MavlinkDataError(
            "No MAVLink session matches SVO clock "
            f"{clock_stamp_ns / 1e9:.9f}; checked: {locations}"
        )
    if len(matching) > 1:
        locations = ", ".join(str(item.path) for item in matching)
        raise MavlinkDataError(
            "Multiple MAVLink sessions match SVO clock "
            f"{clock_stamp_ns / 1e9:.9f}: {locations}"
        )
    return matching[0]


def _strict_rows(rows: Iterable[tuple]) -> list[tuple]:
    """Drop duplicate/non-increasing wall timestamps deterministically."""
    output = []
    previous = None
    for row in rows:
        stamp = int(row[0])
        if previous is not None and stamp <= previous:
            continue
        output.append(row)
        previous = stamp
    return output


def load_trajectory(path: str | Path) -> MavlinkTrajectory:
    """Load valid GPS and attitude rows from one MAVLink database."""
    summary = inspect_session(path)
    try:
        with _readonly_connection(summary.path) as connection:
            gps_rows = _strict_rows(connection.execute(
                """
                SELECT t_wall_epoch_us, lat, lon, alt
                FROM gps
                WHERE lat IS NOT NULL AND lon IS NOT NULL
                  AND alt IS NOT NULL AND lat != 0 AND lon != 0
                  AND fix_type >= 3
                ORDER BY t_wall_epoch_us
                """
            ))
            attitude_rows = _strict_rows(connection.execute(
                """
                SELECT t_wall_epoch_us, roll, pitch, yaw
                FROM attitude
                WHERE roll IS NOT NULL AND pitch IS NOT NULL
                  AND yaw IS NOT NULL
                ORDER BY t_wall_epoch_us
                """
            ))
    except sqlite3.Error as exception:
        raise MavlinkDataError(
            f"Cannot load MAVLink database {summary.path}: {exception}"
        ) from exception
    if len(gps_rows) < 2 or len(attitude_rows) < 2:
        raise MavlinkDataError(
            f"MAVLink database has too few valid samples: {summary.path}"
        )

    gps_stamps = np.asarray(
        [int(row[0]) * MICROSECONDS_TO_NANOSECONDS for row in gps_rows],
        dtype=np.int64,
    )
    geodetic = np.asarray(
        [[float(row[1]), float(row[2]), float(row[3])] for row in gps_rows],
        dtype=np.float64,
    )
    gps_positions = geodetic_to_enu(geodetic, geodetic[0])
    attitude_stamps = np.asarray(
        [int(row[0]) * MICROSECONDS_TO_NANOSECONDS for row in attitude_rows],
        dtype=np.int64,
    )
    attitude_orientations = np.asarray(
        [
            mavlink_attitude_to_ros_quaternion(
                float(row[1]), float(row[2]), float(row[3])
            )
            for row in attitude_rows
        ],
        dtype=np.float64,
    )
    return MavlinkTrajectory(
        path=summary.path,
        gps_stamps_ns=gps_stamps,
        gps_positions=gps_positions,
        attitude_stamps_ns=attitude_stamps,
        attitude_orientations_xyzw=attitude_orientations,
    )


def geodetic_to_ecef(geodetic: np.ndarray) -> np.ndarray:
    """Convert WGS84 latitude/longitude/altitude rows to ECEF metres."""
    values = np.asarray(geodetic, dtype=np.float64)
    single = values.ndim == 1
    values = np.atleast_2d(values)
    latitude = np.radians(values[:, 0])
    longitude = np.radians(values[:, 1])
    altitude = values[:, 2]
    sin_latitude = np.sin(latitude)
    prime_vertical = WGS84_A_M / np.sqrt(
        1.0 - WGS84_E2 * sin_latitude * sin_latitude
    )
    x = (prime_vertical + altitude) * np.cos(latitude) * np.cos(longitude)
    y = (prime_vertical + altitude) * np.cos(latitude) * np.sin(longitude)
    z = (
        prime_vertical * (1.0 - WGS84_E2) + altitude
    ) * sin_latitude
    result = np.column_stack((x, y, z))
    return result[0] if single else result


def geodetic_to_enu(
    geodetic: np.ndarray, reference_geodetic: np.ndarray
) -> np.ndarray:
    """Convert WGS84 rows to local east/north/up coordinates."""
    values = np.asarray(geodetic, dtype=np.float64)
    single = values.ndim == 1
    values = np.atleast_2d(values)
    reference = np.asarray(reference_geodetic, dtype=np.float64)
    ecef = geodetic_to_ecef(values)
    reference_ecef = geodetic_to_ecef(reference)
    latitude = math.radians(float(reference[0]))
    longitude = math.radians(float(reference[1]))
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)
    rotation = np.array(
        [
            [-sin_longitude, cos_longitude, 0.0],
            [
                -sin_latitude * cos_longitude,
                -sin_latitude * sin_longitude,
                cos_latitude,
            ],
            [
                cos_latitude * cos_longitude,
                cos_latitude * sin_longitude,
                sin_latitude,
            ],
        ],
        dtype=np.float64,
    )
    result = (rotation @ (ecef - reference_ecef).T).T
    return result[0] if single else result


def _rotation_x(angle: float) -> np.ndarray:
    sine, cosine = math.sin(angle), math.cos(angle)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]]
    )


def _rotation_y(angle: float) -> np.ndarray:
    sine, cosine = math.sin(angle), math.cos(angle)
    return np.array(
        [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]
    )


def _rotation_z(angle: float) -> np.ndarray:
    sine, cosine = math.sin(angle), math.cos(angle)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
    )


def quaternion_from_rotation_matrix(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper 3D rotation matrix to a ROS XYZW quaternion."""
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("rotation matrix must have shape (3, 3)")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
            ) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
            ) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = math.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
            ) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    return normalize_quaternion(quaternion)


def mavlink_attitude_to_ros_quaternion(
    roll: float, pitch: float, yaw: float
) -> np.ndarray:
    """Convert MAVLink NED/FRD attitude RPY to ROS ENU/FLU orientation."""
    ned_from_frd = (
        _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)
    )
    enu_from_ned = np.array(
        [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]
    )
    frd_from_flu = np.diag([1.0, -1.0, -1.0])
    return quaternion_from_rotation_matrix(
        enu_from_ned @ ned_from_frd @ frd_from_flu
    )


def ros_rpy_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> np.ndarray:
    """Convert ROS fixed-axis RPY to an XYZW quaternion."""
    return quaternion_from_rotation_matrix(
        _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)
    )


def slerp(left: np.ndarray, right: np.ndarray, ratio: float) -> np.ndarray:
    """Spherically interpolate two XYZW quaternions along the short arc."""
    if not 0.0 <= ratio <= 1.0:
        raise ValueError("slerp ratio must be within [0, 1]")
    first = normalize_quaternion(left)
    second = normalize_quaternion(right)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(first + ratio * (second - first))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return normalize_quaternion(
        math.sin((1.0 - ratio) * angle) / sine * first
        + math.sin(ratio * angle) / sine * second
    )


def _bracket(
    stamps_ns: np.ndarray,
    stamp_ns: int,
    *,
    edge_tolerance_ns: int,
) -> tuple[int, int, float] | None:
    """Return interpolation indices and ratio, including close edge holds."""
    insertion = int(np.searchsorted(stamps_ns, stamp_ns, side="left"))
    if insertion < len(stamps_ns) and int(stamps_ns[insertion]) == stamp_ns:
        return insertion, insertion, 0.0
    if insertion == 0:
        if int(stamps_ns[0]) - stamp_ns <= edge_tolerance_ns:
            return 0, 0, 0.0
        return None
    if insertion == len(stamps_ns):
        if stamp_ns - int(stamps_ns[-1]) <= edge_tolerance_ns:
            last = len(stamps_ns) - 1
            return last, last, 0.0
        return None
    left = insertion - 1
    right = insertion
    span = int(stamps_ns[right] - stamps_ns[left])
    ratio = (stamp_ns - int(stamps_ns[left])) / span
    return left, right, float(ratio)


def _interpolate_position(
    stamps_ns: np.ndarray,
    positions: np.ndarray,
    stamp_ns: int,
    *,
    max_gap_ns: int,
    edge_tolerance_ns: int,
) -> np.ndarray | None:
    bracket = _bracket(
        stamps_ns, stamp_ns, edge_tolerance_ns=edge_tolerance_ns
    )
    if bracket is None:
        return None
    left, right, ratio = bracket
    if left != right and int(stamps_ns[right] - stamps_ns[left]) > max_gap_ns:
        return None
    return np.asarray(
        positions[left] + ratio * (positions[right] - positions[left]),
        dtype=np.float64,
    )


def _interpolate_orientation(
    stamps_ns: np.ndarray,
    orientations: np.ndarray,
    stamp_ns: int,
    *,
    edge_tolerance_ns: int,
) -> np.ndarray | None:
    bracket = _bracket(
        stamps_ns, stamp_ns, edge_tolerance_ns=edge_tolerance_ns
    )
    if bracket is None:
        return None
    left, right, ratio = bracket
    if left == right:
        return normalize_quaternion(orientations[left])
    return slerp(orientations[left], orientations[right], ratio)


def apply_body_to_camera(
    sample: PoseSample, body_to_camera: np.ndarray
) -> PoseSample:
    """Compose a MAV body pose with XYZ/RPY camera mounting extrinsics."""
    extrinsic = np.asarray(body_to_camera, dtype=np.float64)
    if extrinsic.shape != (6,) or not np.all(np.isfinite(extrinsic)):
        raise ValueError("body_to_camera must contain six finite values")
    camera_orientation = quaternion_multiply(
        sample.orientation_xyzw,
        ros_rpy_to_quaternion(*extrinsic[3:]),
    )
    camera_position = sample.position + rotate_vector(
        sample.orientation_xyzw, extrinsic[:3]
    )
    return PoseSample(
        stamp_ns=sample.stamp_ns,
        position=camera_position,
        orientation_xyzw=camera_orientation,
    )
