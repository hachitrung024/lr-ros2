"""ROS-independent constant-velocity tracking for measured 3D boxes."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .box_geometry import (
    BoxMeasurement, TrackedBox, _nlerp_quaternion, _require_positive,
    _require_unit_interval,
)


@dataclass
class _Track:
    """Mutable state for one constant-velocity 3D box track."""

    track_id: int
    state: np.ndarray
    covariance: np.ndarray
    size: np.ndarray
    orientation: np.ndarray
    footprint_local_xy: np.ndarray | None
    z_limits_local: np.ndarray | None
    last_stamp_sec: float
    hits: int = 1
    missed: int = 0


class BoxTracker:
    """Associate fitted boxes and filter their centers with a 3D Kalman filter."""

    def __init__(
        self,
        *,
        association_distance_m: float,
        max_missed_frames: int,
        min_confirmations: int,
        measurement_stddev_m: float,
        acceleration_stddev_mps2: float,
        size_smoothing: float,
        orientation_smoothing: float,
    ) -> None:
        _require_positive("association_distance_m", association_distance_m)
        _require_positive("measurement_stddev_m", measurement_stddev_m)
        _require_positive("acceleration_stddev_mps2", acceleration_stddev_mps2)
        if max_missed_frames < 0:
            raise ValueError("max_missed_frames must not be negative")
        if min_confirmations < 1:
            raise ValueError("min_confirmations must be at least one")
        _require_unit_interval("size_smoothing", size_smoothing)
        _require_unit_interval("orientation_smoothing", orientation_smoothing)

        self._association_distance_m = float(association_distance_m)
        self._max_missed_frames = int(max_missed_frames)
        self._min_confirmations = int(min_confirmations)
        self._measurement_variance = float(measurement_stddev_m) ** 2
        self._acceleration_variance = float(acceleration_stddev_mps2) ** 2
        self._size_smoothing = float(size_smoothing)
        self._orientation_smoothing = float(orientation_smoothing)
        self._tracks: list[_Track] = []
        self._next_track_id = 1

    def reset(self) -> None:
        """Discard state from a previous, now-invalid timeline."""
        self._tracks.clear()
        self._next_track_id = 1

    def update(
        self,
        measurements: list[BoxMeasurement],
        stamp_sec: float,
    ) -> list[TrackedBox]:
        """Update tracks from current-frame boxes and return confirmed boxes."""
        if not math.isfinite(stamp_sec):
            raise ValueError("stamp_sec must be finite")
        for track in self._tracks:
            _predict_track(track, stamp_sec, self._acceleration_variance)

        matches = _associate_tracks(
            self._tracks,
            measurements,
            self._association_distance_m,
        )
        matched_track_indices = set()
        matched_measurement_indices = set()
        updated_tracks: list[_Track] = []
        for track_index, measurement_index in matches:
            track = self._tracks[track_index]
            measurement = measurements[measurement_index]
            _update_track(
                track,
                measurement,
                self._measurement_variance,
                self._size_smoothing,
                self._orientation_smoothing,
            )
            matched_track_indices.add(track_index)
            matched_measurement_indices.add(measurement_index)
            updated_tracks.append(track)

        for index, track in enumerate(self._tracks):
            if index not in matched_track_indices:
                track.missed += 1

        for index, measurement in enumerate(measurements):
            if index in matched_measurement_indices:
                continue
            track = _new_track(self._next_track_id, measurement, stamp_sec)
            self._next_track_id += 1
            self._tracks.append(track)
            updated_tracks.append(track)

        self._tracks = [track for track in self._tracks if track.missed <= self._max_missed_frames]
        return [
            TrackedBox(
                track_id=track.track_id,
                center=track.state[:3].copy(),
                size=track.size.copy(),
                orientation=track.orientation.copy(),
                footprint_xy=(
                    None
                    if track.footprint_local_xy is None
                    else track.footprint_local_xy + track.state[:2]
                ),
                minimum_z=(
                    None
                    if track.z_limits_local is None
                    else float(track.state[2] + track.z_limits_local[0])
                ),
                maximum_z=(
                    None
                    if track.z_limits_local is None
                    else float(track.state[2] + track.z_limits_local[1])
                ),
            )
            for track in updated_tracks
            if track.hits >= self._min_confirmations
        ]


def _associate_tracks(
    tracks: list[_Track],
    measurements: list[BoxMeasurement],
    association_distance_m: float,
) -> list[tuple[int, int]]:
    """Greedily associate nearby predicted tracks with current measurements."""
    candidates = []
    for track_index, track in enumerate(tracks):
        for measurement_index, measurement in enumerate(measurements):
            distance = float(np.linalg.norm(track.state[:3] - measurement.center))
            adaptive_gate = max(
                association_distance_m,
                0.5 * float(np.linalg.norm(track.size + measurement.size)),
            )
            if distance <= adaptive_gate:
                candidates.append((distance, track_index, measurement_index))
    candidates.sort()
    matched_tracks = set()
    matched_measurements = set()
    matches = []
    for _, track_index, measurement_index in candidates:
        if track_index in matched_tracks or measurement_index in matched_measurements:
            continue
        matched_tracks.add(track_index)
        matched_measurements.add(measurement_index)
        matches.append((track_index, measurement_index))
    return matches


def _measurement_z_limits_local(measurement: BoxMeasurement) -> np.ndarray | None:
    """Express measured prism limits relative to the tracked center Z."""
    if measurement.minimum_z is None and measurement.maximum_z is None:
        half_height = float(measurement.size[2]) * 0.5
        return np.asarray([-half_height, half_height], dtype=np.float64)
    if measurement.minimum_z is None or measurement.maximum_z is None:
        return None
    return np.asarray(
        [
            float(measurement.minimum_z) - measurement.center[2],
            float(measurement.maximum_z) - measurement.center[2],
        ],
        dtype=np.float64,
    )


def _new_track(
    track_id: int,
    measurement: BoxMeasurement,
    stamp_sec: float,
) -> _Track:
    """Create a track with a zero initial velocity."""
    return _Track(
        track_id=track_id,
        state=np.concatenate((measurement.center, np.zeros(3))),
        covariance=np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0]),
        size=measurement.size.copy(),
        orientation=measurement.orientation.copy(),
        footprint_local_xy=(
            None
            if measurement.footprint_xy is None
            else measurement.footprint_xy.copy() - measurement.center[:2]
        ),
        z_limits_local=_measurement_z_limits_local(measurement),
        last_stamp_sec=stamp_sec,
    )


def _predict_track(
    track: _Track,
    stamp_sec: float,
    acceleration_variance: float,
) -> None:
    """Propagate a constant-velocity track to the supplied timestamp."""
    dt = min(max(stamp_sec - track.last_stamp_sec, 0.0), 1.0)
    transition = np.eye(6)
    transition[:3, 3:] = np.eye(3) * dt
    process = np.zeros((6, 6))
    process[:3, :3] = np.eye(3) * 0.25 * dt**4 * acceleration_variance
    process[:3, 3:] = np.eye(3) * 0.5 * dt**3 * acceleration_variance
    process[3:, :3] = process[:3, 3:]
    process[3:, 3:] = np.eye(3) * dt**2 * acceleration_variance
    track.state = transition @ track.state
    track.covariance = transition @ track.covariance @ transition.T + process
    track.last_stamp_sec = stamp_sec


def _update_track(
    track: _Track,
    measurement: BoxMeasurement,
    measurement_variance: float,
    size_smoothing: float,
    orientation_smoothing: float,
) -> None:
    """Correct one track from a current OBB measurement."""
    innovation = measurement.center - track.state[:3]
    residual_covariance = track.covariance[:3, :3] + (np.eye(3) * measurement_variance)
    gain = track.covariance[:, :3] @ np.linalg.inv(residual_covariance)
    track.state = track.state + gain @ innovation
    observation = np.zeros((3, 6))
    observation[:, :3] = np.eye(3)
    track.covariance = (np.eye(6) - gain @ observation) @ track.covariance
    track.size = (1.0 - size_smoothing) * track.size + size_smoothing * measurement.size
    track.orientation = _nlerp_quaternion(
        track.orientation,
        measurement.orientation,
        orientation_smoothing,
    )
    if measurement.footprint_xy is not None:
        track.footprint_local_xy = measurement.footprint_xy.copy() - measurement.center[:2]
    measured_z_limits = _measurement_z_limits_local(measurement)
    if measured_z_limits is not None:
        if track.z_limits_local is None:
            track.z_limits_local = measured_z_limits
        else:
            track.z_limits_local = (
                (1.0 - size_smoothing) * track.z_limits_local
                + size_smoothing * measured_z_limits
            )
    track.hits += 1
    track.missed = 0
