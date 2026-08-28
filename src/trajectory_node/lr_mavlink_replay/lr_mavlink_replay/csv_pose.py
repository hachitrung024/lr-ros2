# Copyright 2026 Landfill Rover Maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Parse and interpolate MAVLink GPS and attitude CSV recordings."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

NED_TO_ENU = np.asarray(
    [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
    dtype=np.float64,
)
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


@dataclass(frozen=True)
class PoseSample:
    """A rover-base pose in the local ENU world at one timestamp."""

    timestamp_s: float
    position_enu_m: np.ndarray
    quaternion_enu_flu_xyzw: np.ndarray


@dataclass(frozen=True)
class PositionSample:
    """A GPS-derived local ENU position at one timestamp."""

    timestamp_s: float
    position_enu_m: np.ndarray


def _read_rows(path: Path) -> tuple[list[dict[str, str]], set[str]]:
    if not path.is_file():
        raise FileNotFoundError(f'CSV file not found: {path}')
    with path.open('r', newline='', encoding='utf-8-sig') as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or ())
        rows = list(reader)
    if not fields:
        raise ValueError(f'CSV has no header: {path}')
    return rows, fields


def _validate_fields(
    path: Path,
    fields: set[str],
    required: set[str],
) -> None:
    missing = sorted(required - fields)
    if missing:
        raise ValueError(f'{path} is missing columns: {", ".join(missing)}')
    if 'timestamp_unix_s' not in fields and 't_wall_epoch_us' not in fields:
        raise ValueError(
            f'{path} requires timestamp_unix_s or t_wall_epoch_us'
        )


def _timestamp_s(row: dict[str, str]) -> float:
    value = row.get('timestamp_unix_s', '').strip()
    if value:
        return float(value)
    return float(row['t_wall_epoch_us']) * 1e-6


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """Convert WGS84 latitude, longitude and altitude to ECEF metres."""
    lat, lon = np.deg2rad([lat_deg, lon_deg])
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    radius = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return np.asarray([
        (radius + alt_m) * cos_lat * np.cos(lon),
        (radius + alt_m) * cos_lat * np.sin(lon),
        (radius * (1.0 - WGS84_E2) + alt_m) * sin_lat,
    ])


def ecef_to_enu_matrix(lat_deg: float, lon_deg: float) -> np.ndarray:
    """Return the ECEF-to-local-ENU rotation at a geodetic origin."""
    lat, lon = np.deg2rad([lat_deg, lon_deg])
    return np.asarray([
        [-np.sin(lon), np.cos(lon), 0.0],
        [
            -np.sin(lat) * np.cos(lon),
            -np.sin(lat) * np.sin(lon),
            np.cos(lat),
        ],
        [
            np.cos(lat) * np.cos(lon),
            np.cos(lat) * np.sin(lon),
            np.sin(lat),
        ],
    ])


def _deduplicate_last(times: np.ndarray, values: np.ndarray):
    """Sort samples and retain the final row at duplicate timestamps."""
    order = np.argsort(times, kind='stable')
    times = times[order]
    values = values[order]
    keep = np.r_[np.diff(times) > 0.0, True]
    return times[keep], values[keep]


def load_gps(
    path: Path,
    *,
    origin_samples: int = 20,
    minimum_fix_type: int = 3,
):
    """Load valid GPS fixes and convert them to a local ENU trajectory."""
    rows, fields = _read_rows(path)
    _validate_fields(path, fields, {'lat', 'lon', 'alt', 'fix_type'})
    samples = []
    for row in rows:
        try:
            sample = (
                _timestamp_s(row),
                float(row['lat']),
                float(row['lon']),
                float(row['alt']),
                int(float(row['fix_type'])),
            )
        except (KeyError, TypeError, ValueError):
            continue
        if (
            np.isfinite(sample[:4]).all()
            and -90.0 <= sample[1] <= 90.0
            and -180.0 <= sample[2] <= 180.0
            and sample[4] >= minimum_fix_type
        ):
            samples.append(sample)
    if len(samples) < 2:
        raise ValueError(f'{path} has fewer than two valid GPS fixes')

    samples_array = np.asarray(samples, dtype=np.float64)
    times, geodetic = _deduplicate_last(
        samples_array[:, 0], samples_array[:, 1:4]
    )
    if len(times) < 2:
        raise ValueError(f'{path} has fewer than two unique GPS timestamps')

    count = min(20, int(origin_samples), len(geodetic))
    if count <= 0:
        raise ValueError('origin_samples must be greater than zero')
    origin = geodetic[:count].mean(axis=0)
    origin_ecef = geodetic_to_ecef(*origin)
    ecef_to_enu = ecef_to_enu_matrix(origin[0], origin[1])
    positions = np.asarray([
        ecef_to_enu @ (geodetic_to_ecef(*value) - origin_ecef)
        for value in geodetic
    ])
    origin_info = {
        'latitude_deg': float(origin[0]),
        'longitude_deg': float(origin[1]),
        'altitude_m': float(origin[2]),
        'samples_used': count,
    }
    return times, positions, origin_info


def _euler_xyz_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build the standard MAVLink body-to-NED rotation from RPY radians."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rotation_x = np.asarray([
        [1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]
    ])
    rotation_y = np.asarray([
        [cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]
    ])
    rotation_z = np.asarray([
        [cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]
    ])
    return rotation_z @ rotation_y @ rotation_x


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper 3x3 rotation matrix to a normalized XYZW quaternion."""
    matrix = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = np.sqrt(
                1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
            ) * 2.0
            quaternion = np.asarray([
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ])
        elif index == 1:
            scale = np.sqrt(
                1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
            ) * 2.0
            quaternion = np.asarray([
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ])
        else:
            scale = np.sqrt(
                1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
            ) * 2.0
            quaternion = np.asarray([
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])
    return quaternion / np.linalg.norm(quaternion)


def load_attitude(path: Path):
    """Load MAVLink NED/FRD RPY samples as ENU/FLU quaternions."""
    rows, fields = _read_rows(path)
    _validate_fields(path, fields, {'roll', 'pitch', 'yaw'})
    samples = []
    for row in rows:
        try:
            sample = (
                _timestamp_s(row),
                float(row['roll']),
                float(row['pitch']),
                float(row['yaw']),
            )
        except (KeyError, TypeError, ValueError):
            continue
        if np.isfinite(sample).all():
            samples.append(sample)
    if len(samples) < 2:
        raise ValueError(f'{path} has fewer than two valid attitude samples')

    samples_array = np.asarray(samples, dtype=np.float64)
    times, rpy = _deduplicate_last(
        samples_array[:, 0], samples_array[:, 1:4]
    )
    if len(times) < 2:
        raise ValueError(f'{path} has fewer than two unique attitude timestamps')
    quaternions = np.asarray([
        matrix_to_quaternion_xyzw(
            NED_TO_ENU @ _euler_xyz_matrix(*value) @ FLU_TO_FRD
        )
        for value in rpy
    ])
    return times, quaternions


def bracket_gap(times: np.ndarray, query: float) -> float:
    """Return the gap surrounding a query, treating exact endpoints as valid."""
    right = int(np.searchsorted(times, query, side='left'))
    if right < len(times) and np.isclose(
        query, times[right], rtol=0.0, atol=1e-9
    ):
        return 0.0
    if right == 0 or right == len(times):
        return float('inf')
    return float(times[right] - times[right - 1])


def slerp_xyzw(first: np.ndarray, second: np.ndarray, amount: float) -> np.ndarray:
    """Shortest-path spherical interpolation between XYZW quaternions."""
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    first = first / np.linalg.norm(first)
    second = second / np.linalg.norm(second)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        result = first + float(amount) * (second - first)
        return result / np.linalg.norm(result)
    angle = np.arccos(dot)
    sin_angle = np.sin(angle)
    first_weight = np.sin((1.0 - amount) * angle) / sin_angle
    second_weight = np.sin(amount * angle) / sin_angle
    result = first_weight * first + second_weight * second
    return result / np.linalg.norm(result)


class MavlinkPoseLog:
    """Validated GPS/attitude logs that can be sampled at image timestamps."""

    def __init__(
        self,
        gps_path: Path,
        attitude_path: Path,
        *,
        origin_samples: int = 20,
        minimum_fix_type: int = 3,
        maximum_gps_gap_s: float = 2.0,
        maximum_attitude_gap_s: float = 0.5,
    ) -> None:
        if maximum_gps_gap_s <= 0.0 or maximum_attitude_gap_s <= 0.0:
            raise ValueError('maximum interpolation gaps must be positive')
        self.gps_time, self.gps_position, self.origin = load_gps(
            Path(gps_path),
            origin_samples=origin_samples,
            minimum_fix_type=minimum_fix_type,
        )
        self.attitude_time, self.attitude_quaternion = load_attitude(
            Path(attitude_path)
        )
        self.maximum_gps_gap_s = float(maximum_gps_gap_s)
        self.maximum_attitude_gap_s = float(maximum_attitude_gap_s)

    def gps_unavailable_reason(self, timestamp_s: float) -> Optional[str]:
        """Return why an exact-time GPS position cannot be interpolated."""
        timestamp_s = float(timestamp_s)
        if not np.isfinite(timestamp_s):
            return 'invalid_timestamp'
        if (
            timestamp_s < self.gps_time[0]
            or timestamp_s > self.gps_time[-1]
        ):
            return 'outside_log'
        if bracket_gap(self.gps_time, timestamp_s) > self.maximum_gps_gap_s:
            return 'gps_gap'
        return None

    def unavailable_reason(self, timestamp_s: float) -> Optional[str]:
        """Return why an exact-time pose cannot be interpolated."""
        gps_reason = self.gps_unavailable_reason(timestamp_s)
        if gps_reason is not None:
            return gps_reason
        timestamp_s = float(timestamp_s)
        if (
            timestamp_s < self.attitude_time[0]
            or timestamp_s > self.attitude_time[-1]
        ):
            return 'outside_log'
        if (
            bracket_gap(self.attitude_time, timestamp_s)
            > self.maximum_attitude_gap_s
        ):
            return 'attitude_gap'
        return None

    def position_at(self, timestamp_s: float) -> Optional[np.ndarray]:
        """Interpolate a GPS-derived ENU position at one timestamp."""
        timestamp_s = float(timestamp_s)
        if self.gps_unavailable_reason(timestamp_s) is not None:
            return None
        return np.asarray([
            np.interp(timestamp_s, self.gps_time, self.gps_position[:, axis])
            for axis in range(3)
        ])

    def sample_at(self, timestamp_s: float) -> Optional[PoseSample]:
        """Interpolate base position and attitude at an image timestamp."""
        timestamp_s = float(timestamp_s)
        if self.unavailable_reason(timestamp_s) is not None:
            return None
        position = self.position_at(timestamp_s)
        right = int(np.searchsorted(
            self.attitude_time, timestamp_s, side='right'
        ))
        if right == 0:
            quaternion = self.attitude_quaternion[0].copy()
        elif right >= len(self.attitude_time):
            quaternion = self.attitude_quaternion[-1].copy()
        else:
            left = right - 1
            duration = self.attitude_time[right] - self.attitude_time[left]
            amount = (timestamp_s - self.attitude_time[left]) / duration
            quaternion = slerp_xyzw(
                self.attitude_quaternion[left],
                self.attitude_quaternion[right],
                float(amount),
            )
        return PoseSample(timestamp_s, position, quaternion)

    def sample_future_within_radius(
        self,
        timestamp_s: float,
        radius_m: float,
        step_m: float,
    ) -> list[PositionSample]:
        """Sample the future GPS polyline inside an XY circle around the rover."""
        timestamp_s = float(timestamp_s)
        radius_m = float(radius_m)
        step_m = float(step_m)
        if not np.isfinite(radius_m) or not np.isfinite(step_m):
            raise ValueError('future path radius and step must be finite')
        if radius_m <= 0.0:
            raise ValueError('future path radius must be positive')
        if step_m <= 0.0:
            raise ValueError('future path step must be positive')
        if radius_m / step_m > 10000:
            raise ValueError('future path must contain at most 10001 poses')

        current_position = self.position_at(timestamp_s)
        if current_position is None:
            return []

        vertices = [PositionSample(timestamp_s, current_position)]
        previous_time = timestamp_s
        previous_position = current_position
        first_future_index = int(np.searchsorted(
            self.gps_time, timestamp_s, side='right'
        ))
        for index in range(first_future_index, len(self.gps_time)):
            candidate_time = float(self.gps_time[index])
            candidate_position = self.gps_position[index]
            if candidate_time - previous_time > self.maximum_gps_gap_s:
                break

            radial_distance = float(np.linalg.norm(
                candidate_position[:2] - current_position[:2]
            ))
            if radial_distance <= radius_m:
                vertices.append(PositionSample(
                    candidate_time, candidate_position.copy()
                ))
                previous_time = candidate_time
                previous_position = candidate_position
                continue

            segment_xy = candidate_position[:2] - previous_position[:2]
            offset_xy = previous_position[:2] - current_position[:2]
            quadratic_a = float(np.dot(segment_xy, segment_xy))
            quadratic_b = 2.0 * float(np.dot(offset_xy, segment_xy))
            quadratic_c = float(np.dot(offset_xy, offset_xy) - radius_m ** 2)
            discriminant = max(
                0.0,
                quadratic_b ** 2 - 4.0 * quadratic_a * quadratic_c,
            )
            roots = (
                (-quadratic_b - np.sqrt(discriminant))
                / (2.0 * quadratic_a),
                (-quadratic_b + np.sqrt(discriminant))
                / (2.0 * quadratic_a),
            )
            fractions = [value for value in roots if 0.0 <= value <= 1.0]
            if fractions:
                fraction = max(fractions)
                boundary_position = previous_position + fraction * (
                    candidate_position - previous_position
                )
                boundary_time = previous_time + fraction * (
                    candidate_time - previous_time
                )
                if fraction > 1e-12:
                    vertices.append(PositionSample(
                        boundary_time, boundary_position
                    ))
            break

        samples = [vertices[0]]
        next_distance = step_m
        travelled_distance = 0.0
        for start, end in zip(vertices, vertices[1:]):
            segment_distance = float(np.linalg.norm(
                end.position_enu_m[:2] - start.position_enu_m[:2]
            ))
            if segment_distance <= 1e-12:
                continue
            segment_end_distance = travelled_distance + segment_distance
            while next_distance <= segment_end_distance + 1e-9:
                if len(samples) >= 10001:
                    return samples
                amount = min(
                    1.0,
                    (next_distance - travelled_distance) / segment_distance,
                )
                samples.append(PositionSample(
                    start.timestamp_s + amount * (
                        end.timestamp_s - start.timestamp_s
                    ),
                    start.position_enu_m + amount * (
                        end.position_enu_m - start.position_enu_m
                    ),
                ))
                next_distance += step_m
            travelled_distance = segment_end_distance

        final_vertex = vertices[-1]
        if (
            len(samples) < 10001
            and not np.allclose(
                samples[-1].position_enu_m,
                final_vertex.position_enu_m,
                rtol=0.0,
                atol=1e-9,
            )
        ):
            samples.append(final_vertex)
        return samples


def load_pose_log(
    gps_path: str,
    attitude_path: str,
    **kwargs,
) -> MavlinkPoseLog:
    """Shared launch/runtime entry point for validating and loading a CSV pair."""
    return MavlinkPoseLog(Path(gps_path), Path(attitude_path), **kwargs)
