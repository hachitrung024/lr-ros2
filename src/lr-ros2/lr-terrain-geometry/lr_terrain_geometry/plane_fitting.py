"""Deterministic RANSAC and robust plane fitting, independent of map lifetime."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from .terrain_types import EffectiveCellThresholds, LocalTerrainCell, TerrainGeometryConfig


class PlaneFitter:
    """Fit cell planes with a reproducible random stream."""

    def __init__(self, config: TerrainGeometryConfig):
        """Keep the validated terrain configuration and initialize RANSAC."""
        self.config = config
        self.reset()

    def reset(self):
        """Restore the random stream for a new replay timeline."""
        self._rng = np.random.default_rng(self.config.random_seed)

    def fit(
        self,
        fit_points_world: np.ndarray,
        central_points_world: np.ndarray,
        cell: LocalTerrainCell,
        thresholds: EffectiveCellThresholds,
    ) -> Optional[dict]:
        sample = np.asarray(fit_points_world, dtype=np.float64)
        central_sample = np.asarray(
            central_points_world,
            dtype=np.float64,
        )
        sample_count = int(central_sample.shape[0])
        if sample.shape[0] < 3 or sample_count < 3:
            return None

        prior_plane = cell.plane or cell.candidate_plane
        prior_normal = None
        if prior_plane is not None:
            prior_normal = np.asarray(
                prior_plane["normal_world"],
                dtype=np.float64,
            )
            prior_normal /= max(float(np.linalg.norm(prior_normal)), 1e-9)

        iteration_count = self.config.ransac_iterations
        point_count = sample.shape[0]
        hypothesis_indices = self._rng.integers(
            0,
            point_count,
            size=(iteration_count, 3),
        )
        duplicate_rows = (
            (hypothesis_indices[:, 0] == hypothesis_indices[:, 1])
            | (hypothesis_indices[:, 0] == hypothesis_indices[:, 2])
            | (hypothesis_indices[:, 1] == hypothesis_indices[:, 2])
        )
        while np.any(duplicate_rows):
            hypothesis_indices[duplicate_rows] = self._rng.integers(
                0,
                point_count,
                size=(int(np.count_nonzero(duplicate_rows)), 3),
            )
            duplicate_rows = (
                (hypothesis_indices[:, 0] == hypothesis_indices[:, 1])
                | (hypothesis_indices[:, 0] == hypothesis_indices[:, 2])
                | (hypothesis_indices[:, 1] == hypothesis_indices[:, 2])
            )

        p0 = sample[hypothesis_indices[:, 0]]
        p1 = sample[hypothesis_indices[:, 1]]
        p2 = sample[hypothesis_indices[:, 2]]
        normals = np.cross(p1 - p0, p2 - p0)
        normal_lengths = np.linalg.norm(normals, axis=1)
        valid_hypotheses = normal_lengths >= 1e-6
        normals[valid_hypotheses] /= normal_lengths[
            valid_hypotheses,
            None,
        ]
        normals[~valid_hypotheses] = 0.0
        flip_mask = normals[:, 2] < 0.0
        normals[flip_mask] *= -1.0
        offsets = -np.einsum("ij,ij->i", normals, p0)

        hypothesis_distances = np.abs(
            central_sample @ normals.T + offsets[None, :]
        )
        hypothesis_inliers = (
            hypothesis_distances < self.config.distance_threshold
        )
        hypothesis_counts = np.count_nonzero(
            hypothesis_inliers,
            axis=0,
        )
        squared_error = np.sum(
            np.square(hypothesis_distances)
            * hypothesis_inliers,
            axis=0,
        )
        hypothesis_rmse = np.sqrt(
            squared_error / np.maximum(hypothesis_counts, 1)
        )
        if prior_normal is None:
            prior_deltas = np.zeros(iteration_count, dtype=np.float64)
        else:
            prior_deltas = np.arccos(np.clip(
                normals @ prior_normal,
                -1.0,
                1.0,
            ))
        slopes = np.degrees(np.arccos(np.clip(
            np.abs(normals[:, 2]),
            -1.0,
            1.0,
        )))

        def select_hypothesis(mask: np.ndarray) -> Optional[int]:
            candidates = np.flatnonzero(
                mask & (hypothesis_counts > 0)
            )
            if candidates.size == 0:
                return None
            order = np.lexsort((
                prior_deltas[candidates],
                hypothesis_rmse[candidates],
                -hypothesis_counts[candidates],
            ))
            return int(candidates[order[0]])

        best_index = select_hypothesis(
            valid_hypotheses
            & (slopes <= self.config.max_plane_slope_deg)
        )
        if best_index is None:
            # Preserve rejected-slope diagnostics when no acceptable
            # hypothesis exists, without allowing a steep dominant object to
            # hide a valid terrain hypothesis when one is available.
            best_index = select_hypothesis(valid_hypotheses)
        if best_index is None:
            return None
        best_normal = normals[best_index]
        best_d = float(offsets[best_index])

        distances = np.abs(sample @ best_normal + best_d)
        inliers = sample[distances < self.config.distance_threshold]
        if inliers.shape[0] < 3:
            return None

        center, normal, d = self._robust_refine_plane(
            inliers,
            best_normal,
        )
        distances = np.abs(sample @ normal + d)
        inlier_mask = distances < self.config.distance_threshold
        refined_inliers = sample[inlier_mask]
        if refined_inliers.shape[0] >= 3:
            center, normal, d = self._robust_refine_plane(
                refined_inliers,
                normal,
            )
        if refined_inliers.shape[0] < 3:
            return None

        if normal[2] < 0.0:
            normal = -normal
            d = -d
        central_distances = np.abs(central_sample @ normal + d)
        central_inlier_mask = (
            central_distances < self.config.distance_threshold
        )
        central_inliers = central_sample[central_inlier_mask]
        if central_inliers.shape[0] < 3:
            return None
        inlier_count = int(central_inliers.shape[0])
        inlier_ratio = inlier_count / float(sample_count)
        minor_stddev, major_stddev = self._principal_stddev_xy(
            central_inliers
        )
        slope_deg = self.slope_from_world_normal(normal)
        rmse = float(np.sqrt(np.mean(np.square(
            central_distances[central_inlier_mask]
        ))))

        rejection_reasons = []
        if (
            inlier_count < thresholds.min_inliers
            or inlier_ratio < self.config.min_cell_inlier_ratio
        ):
            rejection_reasons.append("inliers")
        if minor_stddev < thresholds.min_principal_stddev:
            rejection_reasons.append("coverage")
        if rmse > self.config.max_cell_rmse:
            rejection_reasons.append("rmse")
        if slope_deg > self.config.max_plane_slope_deg:
            rejection_reasons.append("slope")

        grid_x, grid_y = self.grid_center_xy(cell.key)
        if abs(float(normal[2])) >= 1e-6:
            grid_z = float(
                center[2]
                - (
                    normal[0] * (grid_x - center[0])
                    + normal[1] * (grid_y - center[1])
                ) / normal[2]
            )
        else:
            grid_z = float(np.median(refined_inliers[:, 2]))
        grid_center = np.asarray(
            [grid_x, grid_y, grid_z],
            dtype=np.float64,
        )
        footprint = self.plane_footprint(
            cell.key,
            grid_center,
            normal,
        )
        half_size = self.config.cell_size_m * 0.5
        rejection_reason = (
            ", ".join(rejection_reasons)
            if rejection_reasons
            else None
        )

        return {
            "center_world": grid_center.astype(np.float32),
            "normal_world": normal.astype(np.float32),
            "slope_deg": slope_deg,
            "inlier_count": inlier_count,
            "sample_count": sample_count,
            "inlier_ratio": inlier_ratio,
            "rmse": rmse,
            "inlier_principal_stddev_minor": minor_stddev,
            "inlier_principal_stddev_major": major_stddev,
            "extents": (half_size, half_size),
            "rejection_reason": rejection_reason,
            "column_key": cell.key,
            "footprint_world": footprint,
            "footprint_local": None,
            "grid_center_world": grid_center.astype(np.float32),
            "observation_count": cell.observation_count,
            "last_seen": cell.last_point_seen,
            "show_label": rejection_reason is not None,
        }

    def grid_center_xy(
        self,
        key: tuple[int, int],
    ) -> tuple[float, float]:
        size = self.config.cell_size_m
        return (
            (key[0] + 0.5) * size,
            (key[1] + 0.5) * size,
        )

    def horizontal_footprint(
        self,
        key: tuple[int, int],
        height: float,
    ) -> np.ndarray:
        size = self.config.cell_size_m
        x0 = key[0] * size
        x1 = x0 + size
        y0 = key[1] * size
        y1 = y0 + size
        return np.asarray(
            [
                [x0, y0, height],
                [x1, y0, height],
                [x1, y1, height],
                [x0, y1, height],
            ],
            dtype=np.float32,
        )

    def plane_footprint(
        self,
        key: tuple[int, int],
        center: np.ndarray,
        normal: np.ndarray,
    ) -> np.ndarray:
        if abs(float(normal[2])) < 1e-6:
            return self.horizontal_footprint(key, float(center[2]))
        size = self.config.cell_size_m
        x0 = key[0] * size
        x1 = x0 + size
        y0 = key[1] * size
        y1 = y0 + size
        corners = []
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            z = center[2] - (
                normal[0] * (x - center[0])
                + normal[1] * (y - center[1])
            ) / normal[2]
            corners.append((x, y, z))
        return np.asarray(corners, dtype=np.float32)

    def _refine_plane(
        self,
        points: np.ndarray,
        initial_normal: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        center = points.mean(axis=0)
        _, _, vh = np.linalg.svd(points - center, full_matrices=False)
        normal = vh[-1]
        normal /= np.linalg.norm(normal)
        if np.dot(normal, initial_normal) < 0.0:
            normal = -normal
        d = -float(np.dot(normal, center))
        return center, normal, d

    def _robust_refine_plane(
        self,
        points: np.ndarray,
        initial_normal: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        center, normal, d = self._refine_plane(points, initial_normal)
        huber_delta = self.config.distance_threshold
        for _ in range(self.config.robust_refinement_iterations):
            residuals = np.abs(points @ normal + d)
            weights = np.ones(points.shape[0], dtype=np.float64)
            outside = residuals > huber_delta
            weights[outside] = huber_delta / np.maximum(
                residuals[outside],
                1e-12,
            )
            weight_sum = float(np.sum(weights))
            if weight_sum <= 1e-9:
                break
            center = np.sum(
                points * weights[:, None],
                axis=0,
            ) / weight_sum
            centered = points - center
            covariance = (
                (centered * weights[:, None]).T @ centered
            ) / weight_sum
            _, eigenvectors = np.linalg.eigh(covariance)
            updated_normal = eigenvectors[:, 0]
            updated_normal /= max(
                float(np.linalg.norm(updated_normal)),
                1e-9,
            )
            if float(np.dot(updated_normal, normal)) < 0.0:
                updated_normal = -updated_normal
            normal = updated_normal
            d = -float(np.dot(normal, center))
        return center, normal, d

    def _principal_stddev_xy(
        self,
        points_world: np.ndarray,
    ) -> tuple[float, float]:
        xy = np.asarray(points_world[:, [0, 1]], dtype=np.float64)
        centered = xy - np.mean(xy, axis=0)
        covariance = centered.T @ centered / xy.shape[0]
        eigenvalues = np.maximum(np.linalg.eigvalsh(covariance), 0.0)
        deviations = np.sqrt(eigenvalues)
        return float(deviations[0]), float(deviations[1])

    def slope_from_world_normal(self, normal_world: np.ndarray) -> float:
        normal = normal_world / np.linalg.norm(normal_world)
        cosine = abs(float(normal[2]))
        return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))
