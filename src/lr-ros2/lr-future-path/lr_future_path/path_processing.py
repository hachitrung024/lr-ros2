"""Spatial resampling and bounded smoothing of recorded future paths."""

import math

import numpy as np


def prepare_path(stamps, positions, orientations, *, radius_m, step_m,
                 max_points=256, smoothing_window_m=0.8, max_deviation_m=0.2):
    """Clip at the first radius exit and smooth uniformly spaced spatial samples."""
    stamps = np.asarray(stamps, dtype=np.int64)
    positions = np.asarray(positions, dtype=np.float64).copy()
    orientations = np.asarray(orientations, dtype=np.float64).copy()
    if not len(stamps):
        return stamps, positions, orientations
    if (positions.shape != (len(stamps), 3) or orientations.shape != (len(stamps), 4)
            or not np.isfinite(positions).all() or not np.isfinite(orientations).all()):
        raise ValueError("path contains malformed or non-finite poses")
    norms = np.linalg.norm(orientations, axis=1)
    if np.any(norms < 1e-12) or np.any(np.diff(stamps) < 0):
        raise ValueError("path has invalid orientations or timestamp order")
    if (not all(math.isfinite(v) for v in
                (radius_m, step_m, smoothing_window_m, max_deviation_m))
            or min(radius_m, step_m) <= 0 or min(smoothing_window_m, max_deviation_m) < 0
            or not 1 <= max_points <= 256):
        raise ValueError("invalid spatial path processing parameters")
    orientations /= norms[:, None]
    center = positions[0, :2].copy()
    outside = np.flatnonzero(np.linalg.norm(positions[:, :2] - center, axis=1) > radius_m)
    if len(outside):
        end = int(outside[0])
        a = positions[end - 1, :2] - center
        direction = positions[end, :2] - positions[end - 1, :2]
        aa = float(direction @ direction)
        bb = float(a @ direction)
        fraction = (-bb + math.sqrt(max(0., bb * bb - aa * (a @ a - radius_m ** 2)))) / aa
        fraction = float(np.clip(fraction, 0., 1.))
        positions[end] = positions[end - 1] + fraction * (positions[end] - positions[end - 1])
        orientations[end] = interpolate_quaternion(
            orientations[end - 1], orientations[end], fraction,
        )
        stamps = stamps.copy()
        stamps[end] = stamps[end - 1] + round(fraction * int(stamps[end] - stamps[end - 1]))
        stamps, positions, orientations = (
            stamps[:end + 1], positions[:end + 1], orientations[:end + 1],
        )
    distances = np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1)
    keep = np.r_[True, distances > 1e-8]
    stamps, positions, orientations = stamps[keep], positions[keep], orientations[keep]
    if len(stamps) < 2:
        return stamps, positions, orientations
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1))]
    # A route can loop indefinitely inside the circle. Bound display complexity
    # spatially as well, without making slow motion or stops shorten the path.
    length = min(arc[-1], 2. * radius_m)
    if max_points == 1:
        targets = np.array([0.])
    else:
        spacing = max(step_m, length / (max_points - 1))
        targets = np.r_[np.arange(0., length, spacing)[:max_points - 1], length]
    count = len(targets)
    right = np.clip(np.searchsorted(arc, targets, side="right"), 1, len(arc) - 1)
    fraction = (targets - arc[right - 1]) / (arc[right] - arc[right - 1])
    xyz = positions[right - 1] + fraction[:, None] * (positions[right] - positions[right - 1])
    times = stamps[right - 1] + np.rint(
        fraction * (stamps[right] - stamps[right - 1])
    ).astype(np.int64)
    quaternions = np.array([
        interpolate_quaternion(orientations[i - 1], orientations[i], f)
        for i, f in zip(right, fraction)
    ])
    if smoothing_window_m > 0 and max_deviation_m > 0 and count > 2:
        endpoints = xyz[[0, -1]].copy()
        delta = targets[None, :] - targets[:, None]
        weights = np.maximum(0., 1. - np.abs(delta) / (smoothing_window_m / 2.))
        s0 = weights.sum(axis=1)
        s1 = (weights * delta).sum(axis=1)
        s2 = (weights * delta ** 2).sum(axis=1)
        denominator = s0 * s2 - s1 ** 2
        valid = denominator > 1e-12
        smoothed = xyz.copy()
        smoothed[valid] = (
            s2[:, None] * (weights @ xyz) - s1[:, None] * ((weights * delta) @ xyz)
        )[valid] / denominator[valid, None]
        displacement = smoothed - xyz
        scale = np.minimum(1., max_deviation_m / np.maximum(
            np.linalg.norm(displacement, axis=1), 1e-12,
        ))
        xyz += displacement * scale[:, None]
        xyz[0], xyz[-1] = endpoints
        # Local linear regression can overshoot a bend near the circle boundary.
        radial = xyz[:, :2] - center
        xyz[:, :2] = center + radial * np.minimum(
            1., radius_m / np.maximum(np.linalg.norm(radial, axis=1), 1e-12),
        )[:, None]
    return times, xyz, quaternions


class StablePathProcessor:
    """
    Retain map landmarks within the smoothing bound as the view advances.

    Source pose timestamps identify route locations, including at crossings.
    Interior landmarks stay fixed while the current pose and radius boundary
    move. A corrected route, rewind or missing overlap starts a fresh set.
    """

    def __init__(self):
        """Start without any landmarks from an earlier view."""
        self.reset()

    def reset(self):
        """Discard landmarks on a frame change, expiration or clock rewind."""
        self._anchors = None
        self._start_stamp = None

    def process(self, stamps, positions, orientations, **parameters):
        """Smooth new route sections and reuse validated overlapping landmarks."""
        prepared = prepare_path(stamps, positions, orientations, **parameters)
        times, xyz, quaternions = prepared
        if len(times) < 3:
            # PathOutput may hold the previous display across a short input
            # gap; keep its landmarks until that hold expires or input recovers.
            return prepared
        if np.any(np.diff(stamps) <= 0):
            self.reset()
            return prepared
        if self._start_stamp is not None and times[0] < self._start_stamp:
            self.reset()
        self._start_stamp = times[0]
        retained = None
        if self._anchors is not None:
            old_times, old_xyz, old_quaternions = self._anchors
            keep = (old_times > times[0]) & (old_times < times[-1])
            keep &= np.linalg.norm(old_xyz[:, :2] - xyz[0, :2], axis=1) <= parameters['radius_m']
            old_times, old_xyz, old_quaternions = (
                old_times[keep], old_xyz[keep], old_quaternions[keep],
            )
            if len(old_times):
                # Validate against raw source geometry, so retention cannot
                # accumulate drift or hide a real correction of the route.
                raw_times = np.asarray(stamps, dtype=np.int64)
                raw_xyz = np.asarray(positions, dtype=float)
                right = np.clip(np.searchsorted(raw_times, old_times), 1, len(raw_times) - 1)
                fraction = (old_times - raw_times[right - 1]) / (
                    raw_times[right] - raw_times[right - 1]
                )
                reference = raw_xyz[right - 1] + fraction[:, None] * (
                    raw_xyz[right] - raw_xyz[right - 1]
                )
                tolerance = parameters.get('max_deviation_m', .2) + 1e-6
                if np.all(np.linalg.norm(old_xyz - reference, axis=1) <= tolerance):
                    # Keep map locations stable, but retain current source
                    # attitude for yaw/physics when tracking corrects rotation.
                    raw_quaternions = np.asarray(orientations, dtype=float)
                    raw_quaternions = raw_quaternions / np.linalg.norm(
                        raw_quaternions, axis=1,
                    )[:, None]
                    old_quaternions = np.array([
                        interpolate_quaternion(raw_quaternions[i - 1], raw_quaternions[i], f)
                        for i, f in zip(right, fraction)
                    ])
                    retained = (old_times, old_xyz, old_quaternions)
        if retained is None:
            self._anchors = tuple(values[1:-1].copy() for values in prepared)
            return prepared

        old_times, old_xyz, old_quaternions = retained
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1))]
        # Append only newly visible stations, without redistributing the old
        # ones when the radius intersection or the route length changes.
        last_arc = float(np.interp(int(old_times[-1] - times[0]), times - times[0], arc))
        additions = []
        for index in range(1, len(times) - 1):
            if (times[index] > old_times[-1]
                    and arc[index] - last_arc >= parameters['step_m'] - 1e-6):
                additions.append(index)
                last_arc = arc[index]
        capacity = parameters.get('max_points', 256) - 2
        self._anchors = tuple(
            np.concatenate((old, new[additions]))[:capacity]
            for old, new in zip(retained, prepared)
        )
        return tuple(
            np.concatenate((new[:1], anchors, new[-1:]))
            for anchors, new in zip(self._anchors, prepared)
        )


def interpolate_quaternion(first, second, fraction):
    """Interpolate normalized attitudes on the shortest quaternion hemisphere."""
    first, second = np.asarray(first), np.asarray(second)
    if first @ second < 0:
        second = -second
    result = (1. - fraction) * first + fraction * second
    return result / np.linalg.norm(result)
