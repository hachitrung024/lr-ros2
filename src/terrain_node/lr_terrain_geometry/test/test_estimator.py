import math
import unittest

import numpy as np

from lr_terrain_geometry import (
    TerrainGeometryConfig,
    TerrainGeometryEstimator,
)


def pose_matrix(x=0.0, y=0.0, z=0.0, yaw_deg=0.0):
    yaw = math.radians(yaw_deg)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    pose = np.identity(4, dtype=np.float64)
    pose[:3, :3] = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    pose[:3, 3] = [x, y, z]
    return pose


def update_terrain(estimator, points, pose, timestamp, valid, **kwargs):
    assert not kwargs
    transform = np.asarray(pose, dtype=np.float64) if valid else None
    return estimator.update(points, transform, timestamp)


def camera_points_from_world(points_world, pose):
    points_world = np.asarray(points_world, dtype=np.float64)
    return (points_world - pose[:3, 3]) @ pose[:3, :3]


def plane_points(
    *,
    x_range=(0.05, 0.95),
    y_range=(-0.95, -0.05),
    x_count=10,
    y_count=10,
    height=0.0,
    slope_x=0.0,
    noise=0.0,
    seed=4,
):
    x = np.linspace(*x_range, x_count, dtype=np.float64)
    y = np.linspace(*y_range, y_count, dtype=np.float64)
    xx, yy = np.meshgrid(x, y)
    zz = height + slope_x * xx
    if noise > 0.0:
        zz += np.random.default_rng(seed).normal(0.0, noise, zz.shape)
    return np.column_stack((xx.reshape(-1), yy.reshape(-1), zz.reshape(-1)))


def narrow_strip_points():
    x = np.linspace(0.05, 0.95, 30, dtype=np.float64)
    y = np.linspace(-0.51, -0.49, 3, dtype=np.float64)
    xx, yy = np.meshgrid(x, y)
    return np.column_stack((
        xx.reshape(-1),
        yy.reshape(-1),
        np.zeros(xx.size, dtype=np.float64),
    ))


def two_cell_points():
    return np.vstack((
        plane_points(),
        plane_points(x_range=(1.05, 1.95)),
    ))


class TerrainGeometryEstimatorTests(unittest.TestCase):

    def make_map(self, **overrides):
        values = {
            "radius_m": 50.0,
            "cell_size_m": 1.0,
            "ttl_seconds": 0.0,
            "retention_hysteresis_m": 0.0,
            "pose_jump_threshold_m": 100.0,
            "pose_jump_rotation_deg": 180.0,
            "ransac_iterations": 60,
            "min_distance": 0.0,
            "max_distance": 100.0,
            "min_forward_m": None,
            "max_forward_m": None,
            "min_lateral_m": None,
            "max_lateral_m": None,
            "min_height_m": -100.0,
            "max_height_m": 100.0,
            "min_cell_points": 20,
            "min_cell_inliers": 10,
            "min_cell_principal_stddev": 0.05,
            "max_ingest_points_per_frame": 10000,
            "max_points_per_cell_per_frame": 1000,
            "max_points_per_cell": 1500,
            "point_window_seconds": 10.0,
            "min_accumulation_frames": 3,
            "fit_every_frames": 1,
            "max_cells_fit_per_cycle": 100,
        }
        values.update(overrides)
        return TerrainGeometryEstimator(TerrainGeometryConfig(**values))

    def test_same_world_cell_accumulates_across_camera_poses(self):
        terrain_map = self.make_map(fit_every_frames=100)
        points_world = plane_points()
        first_pose = pose_matrix()
        second_pose = pose_matrix(x=0.2, y=0.1, yaw_deg=37.0)

        update_terrain(
            terrain_map,
            camera_points_from_world(points_world, first_pose),
            first_pose,
            1.0,
            True,
        )
        update_terrain(
            terrain_map,
            camera_points_from_world(points_world, second_pose),
            second_pose,
            2.0,
            True,
        )

        self.assertEqual(terrain_map.cell_count, 1)
        self.assertIn((0, -1), terrain_map._cells)
        self.assertEqual(terrain_map._cells[(0, -1)].point_count, 200)

    def test_fit_waits_for_minimum_frames_and_fit_cadence(self):
        terrain_map = self.make_map(fit_every_frames=5)
        points = plane_points(noise=0.002)

        for frame in range(1, 10):
            update = update_terrain(
                terrain_map,
                points,
                pose_matrix(),
                float(frame),
                True,
            )
            self.assertEqual(update.accepted_planes, ())
            self.assertEqual(update.fit_cycle, frame % 5 == 0)

        update = update_terrain(
            terrain_map,
            points,
            pose_matrix(),
            10.0,
            True,
        )

        self.assertTrue(update.fit_cycle)
        self.assertEqual(len(update.accepted_planes), 1)
        self.assertLess(update.accepted_planes[0]["rmse"], 0.01)
        self.assertLess(update.accepted_planes[0]["slope_deg"], 1.0)

    def test_global_per_frame_cell_and_total_caps_are_enforced(self):
        terrain_map = self.make_map(
            fit_every_frames=100,
            max_ingest_points_per_frame=50,
            max_points_per_cell_per_frame=20,
            max_points_per_cell=30,
            point_window_seconds=1.0,
        )
        points = plane_points()

        update_terrain(terrain_map, points, pose_matrix(), 0.0, True)
        self.assertEqual(terrain_map.point_count, 20)
        update_terrain(terrain_map, points, pose_matrix(), 0.5, True)
        self.assertEqual(terrain_map.point_count, 30)
        update_terrain(terrain_map, points, pose_matrix(), 2.0, True)
        self.assertEqual(terrain_map.point_count, 20)

    def test_world_voxel_downsampling_removes_frame_duplicates(self):
        terrain_map = self.make_map(
            fit_every_frames=100,
            voxel_size_m=0.05,
        )
        points = np.asarray([
            [0.101, 0.001, 0.101],
            [0.119, 0.019, 0.119],
            [0.181, 0.001, 0.181],
        ])

        update_terrain(terrain_map, points, pose_matrix(), 0.0, True)

        self.assertEqual(terrain_map.cell_count, 1)
        self.assertEqual(terrain_map.point_count, 2)

    def test_dominant_surface_wins_in_mixed_cell(self):
        terrain_map = self.make_map(
            confirmation_good_fits=1,
            min_accumulation_frames=1,
            voxel_size_m=0.001,
        )
        lower = plane_points(x_count=10, y_count=6, height=0.0)
        upper = plane_points(x_count=8, y_count=5, height=0.2)

        update = update_terrain(
            terrain_map,
            np.vstack((lower, upper)),
            pose_matrix(),
            0.0,
            True,
        )

        self.assertEqual(len(update.accepted_planes), 1)
        self.assertLess(abs(update.accepted_planes[0]["center_world"][2]), 0.02)
        self.assertGreater(update.accepted_planes[0]["inlier_ratio"], 0.55)

    def test_neighbor_halo_cannot_replace_central_coverage(self):
        terrain_map = self.make_map(
            confirmation_good_fits=1,
            min_accumulation_frames=1,
            voxel_size_m=0.001,
            max_neighbor_fit_points=300,
            neighbor_halo_m=0.1,
        )
        central = plane_points(
            x_range=(0.91, 0.99),
            y_range=(-0.95, -0.05),
            x_count=8,
            y_count=12,
        )
        neighbor = plane_points(
            x_range=(1.01, 1.09),
            y_range=(-0.95, -0.05),
            x_count=8,
            y_count=12,
        )

        update = update_terrain(
            terrain_map,
            np.vstack((central, neighbor)),
            pose_matrix(),
            0.0,
            True,
        )

        central_cell = terrain_map._cells[(0, -1)]
        central_points, fit_points = terrain_map._fit_points_for_cell(
            central_cell
        )
        self.assertGreater(fit_points.shape[0], central_points.shape[0])
        rejected = [
            plane
            for plane in update.rejected_planes
            if plane["column_key"] == (0, -1)
        ]
        self.assertEqual(len(rejected), 1)
        self.assertIn("coverage", rejected[0]["rejection_reason"])

    def test_surface_switch_requires_two_consistent_good_fits(self):
        terrain_map = self.make_map(point_window_seconds=0.25)
        lower = plane_points(height=0.0)
        upper = plane_points(height=0.2)

        for timestamp in (0.0, 0.05, 0.1, 0.15, 0.2, 0.25):
            confirmed = update_terrain(
                terrain_map,
                lower,
                pose_matrix(),
                timestamp,
                True,
            )
        self.assertAlmostEqual(
            confirmed.accepted_planes[0]["center_world"][2],
            0.0,
            places=3,
        )

        for timestamp in (1.0, 1.05, 1.1):
            candidate = update_terrain(
                terrain_map,
                upper,
                pose_matrix(),
                timestamp,
                True,
            )
        self.assertAlmostEqual(
            candidate.accepted_planes[0]["center_world"][2],
            0.0,
            places=3,
        )

        for timestamp in (1.15, 1.2, 1.25):
            switched = update_terrain(
                terrain_map,
                upper,
                pose_matrix(),
                timestamp,
                True,
            )
        self.assertAlmostEqual(
            switched.accepted_planes[0]["center_world"][2],
            0.2,
            places=3,
        )

    def test_noisy_plane_stays_stable_after_confirmation(self):
        terrain_map = self.make_map(
            point_window_seconds=2.0,
            plane_ema_alpha=0.25,
        )
        heights = []
        slopes = []
        for frame in range(18):
            update = update_terrain(
                terrain_map,
                plane_points(noise=0.01, seed=frame),
                pose_matrix(),
                frame * 0.05,
                True,
            )
            if update.accepted_planes:
                heights.append(float(
                    update.accepted_planes[0]["center_world"][2]
                ))
                slopes.append(float(update.accepted_planes[0]["slope_deg"]))

        self.assertGreater(len(heights), 3)
        self.assertLess(np.ptp(heights), 0.02)
        self.assertLess(np.ptp(slopes), 1.0)

    def test_two_failed_refits_remove_previous_plane(self):
        terrain_map = self.make_map(point_window_seconds=0.25)
        good_points = plane_points()

        for timestamp in (0.0, 0.05, 0.1, 0.15, 0.2, 0.25):
            good_update = update_terrain(
                terrain_map,
                good_points,
                pose_matrix(),
                timestamp,
                True,
            )
        self.assertEqual(len(good_update.accepted_planes), 1)

        bad_points = narrow_strip_points()
        for timestamp in (1.0, 1.1, 1.2):
            first_bad_update = update_terrain(
                terrain_map,
                bad_points,
                pose_matrix(),
                timestamp,
                True,
            )
        self.assertEqual(len(first_bad_update.accepted_planes), 1)
        self.assertEqual(len(first_bad_update.rejected_planes), 1)

        for timestamp in (1.3, 1.4, 1.5):
            second_bad_update = update_terrain(
                terrain_map,
                bad_points,
                pose_matrix(),
                timestamp,
                True,
            )

        self.assertEqual(second_bad_update.accepted_planes, ())
        self.assertEqual(len(second_bad_update.rejected_planes), 1)
        self.assertIn(
            "coverage",
            second_bad_update.rejected_planes[0]["rejection_reason"],
        )

    def test_slope_rejection_is_reported_in_world_frame(self):
        terrain_map = self.make_map()
        steep_points = plane_points(slope_x=2.0)

        for timestamp in (0.0, 0.1, 0.2):
            update = update_terrain(
                terrain_map,
                steep_points,
                pose_matrix(),
                timestamp,
                True,
            )

        self.assertEqual(update.accepted_planes, ())
        self.assertEqual(len(update.rejected_planes), 1)
        self.assertIn("slope", update.rejected_planes[0]["rejection_reason"])
        self.assertEqual(
            update.rejected_planes[0]["footprint_world"].shape,
            (4, 3),
        )

    def test_rmse_acceptance_threshold_is_configurable(self):
        terrain_map = self.make_map(
            confirmation_good_fits=1,
            min_accumulation_frames=1,
            max_cell_rmse=0.001,
            voxel_size_m=0.001,
        )

        update = update_terrain(
            terrain_map,
            plane_points(noise=0.01),
            pose_matrix(),
            0.0,
            True,
        )

        self.assertEqual(update.accepted_planes, ())
        self.assertEqual(len(update.rejected_planes), 1)
        self.assertIn("rmse", update.rejected_planes[0]["rejection_reason"])

    def test_scheduler_does_not_starve_second_eligible_cell(self):
        terrain_map = self.make_map(
            max_cells_fit_per_cycle=1,
            confirmation_good_fits=1,
        )
        points = two_cell_points()

        for timestamp in (0.0, 0.1, 0.2):
            update = update_terrain(
                terrain_map,
                points,
                pose_matrix(),
                timestamp,
                True,
            )
        self.assertEqual(len(update.accepted_planes), 1)
        self.assertEqual(update.accepted_planes[0]["column_key"], (0, -1))

        update = update_terrain(
            terrain_map,
            points,
            pose_matrix(),
            0.3,
            True,
        )
        self.assertEqual(len(update.accepted_planes), 2)

    def test_sampling_is_deterministic_for_same_seed(self):
        first = self.make_map(
            fit_every_frames=100,
            max_ingest_points_per_frame=50,
            max_points_per_cell_per_frame=20,
            random_seed=11,
        )
        second = self.make_map(
            fit_every_frames=100,
            max_ingest_points_per_frame=50,
            max_points_per_cell_per_frame=20,
            random_seed=11,
        )
        points = plane_points()

        update_terrain(first, points, pose_matrix(), 1.0, True)
        update_terrain(second, points, pose_matrix(), 1.0, True)

        np.testing.assert_array_equal(
            first._cell_points(first._cells[(0, -1)]),
            second._cell_points(second._cells[(0, -1)]),
        )

    def test_tracking_loss_freezes_ingestion_but_ttl_prunes(self):
        terrain_map = self.make_map(
            ttl_seconds=0.5,
            point_window_seconds=0.2,
            confirmation_good_fits=1,
        )
        points = plane_points()
        for timestamp in (0.0, 0.1, 0.2):
            update = update_terrain(
                terrain_map,
                points,
                pose_matrix(),
                timestamp,
                True,
            )
        self.assertEqual(len(update.accepted_planes), 1)

        frozen = update_terrain(
            terrain_map,
            points,
            pose_matrix(),
            0.3,
            False,
        )
        self.assertEqual(len(frozen.accepted_planes), 1)
        expired = update_terrain(
            terrain_map,
            points,
            pose_matrix(),
            0.71,
            False,
        )
        self.assertEqual(expired.accepted_planes, ())
        self.assertEqual(terrain_map.cell_count, 0)

    def test_pose_jump_clears_points_and_plane_state(self):
        terrain_map = self.make_map(
            fit_every_frames=100,
            pose_jump_threshold_m=2.0,
        )
        update_terrain(terrain_map, plane_points(), pose_matrix(), 1.0, True)

        jumped_pose = pose_matrix(x=3.0)
        update = update_terrain(
            terrain_map,
            np.empty((0, 3)),
            jumped_pose,
            2.0,
            True,
        )

        self.assertEqual(terrain_map.cell_count, 0)
        self.assertEqual(update.reset_reason, "pose discontinuity")

    def test_timestamp_rollback_clears_state_before_new_ingestion(self):
        terrain_map = self.make_map(fit_every_frames=100)
        update_terrain(terrain_map, plane_points(), pose_matrix(), 10.0, True)

        update = update_terrain(
            terrain_map,
            np.empty((0, 3)),
            pose_matrix(),
            9.0,
            True,
        )

        self.assertEqual(terrain_map.cell_count, 0)
        self.assertEqual(update.reset_reason, "timestamp moved backwards")

    def test_explicit_reset_restores_frame_and_rng_state(self):
        terrain_map = self.make_map(fit_every_frames=100)
        points = plane_points()
        update_terrain(terrain_map, points, pose_matrix(), 10.0, True)
        first_sample = terrain_map._cell_points(
            terrain_map._cells[(0, -1)]
        ).copy()

        terrain_map.reset("SVO seek")
        update_terrain(terrain_map, points, pose_matrix(), 1.0, True)
        second_sample = terrain_map._cell_points(
            terrain_map._cells[(0, -1)]
        )

        np.testing.assert_array_equal(second_sample, first_sample)
        self.assertEqual(terrain_map.last_reset_reason, "SVO seek")

    def test_snapshot_radius_and_retention_hysteresis_are_distinct(self):
        terrain_map = self.make_map(
            radius_m=1.0,
            retention_hysteresis_m=1.0,
            fit_every_frames=100,
        )
        update_terrain(terrain_map, plane_points(), pose_matrix(), 1.0, True)
        self.assertEqual(terrain_map.cell_count, 1)

        update_terrain(
            terrain_map,
            np.empty((0, 3)),
            pose_matrix(x=1.6),
            2.0,
            True,
        )
        self.assertEqual(terrain_map.debug_snapshot(), [])
        self.assertEqual(terrain_map.cell_count, 1)

        update_terrain(
            terrain_map,
            np.empty((0, 3)),
            pose_matrix(x=3.0),
            3.0,
            True,
        )
        self.assertEqual(terrain_map.cell_count, 0)


class TerrainGeometryConfigTests(unittest.TestCase):

    def test_accumulation_profile_defaults(self):
        config = TerrainGeometryConfig()

        self.assertEqual(config.max_ingest_points_per_frame, 12000)
        self.assertEqual(config.max_points_per_cell_per_frame, 200)
        self.assertEqual(config.max_points_per_cell, 1500)
        self.assertEqual(config.point_window_seconds, 10.0)
        self.assertEqual(config.voxel_size_m, 0.05)
        self.assertEqual(config.min_accumulation_frames, 3)
        self.assertEqual(config.fit_every_frames, 5)
        self.assertEqual(config.max_cells_fit_per_cycle, 100)
        self.assertEqual(config.max_central_fit_points, 1200)
        self.assertEqual(config.max_neighbor_fit_points, 300)
        self.assertEqual(config.neighbor_halo_m, 0.1)
        self.assertEqual(config.robust_refinement_iterations, 3)
        self.assertEqual(config.min_cell_points, 80)
        self.assertEqual(config.distance_threshold, 0.035)
        self.assertEqual(config.min_cell_inliers, 10)
        self.assertEqual(config.min_cell_inlier_ratio, 0.15)
        self.assertEqual(config.min_cell_principal_stddev, 0.1)
        self.assertEqual(config.max_cell_rmse, 0.035)
        self.assertEqual(config.max_plane_slope_deg, 55.0)
        self.assertEqual(config.confirmation_good_fits, 2)
        self.assertEqual(config.rejection_bad_fits, 2)
        self.assertEqual(config.plane_ema_alpha, 0.25)

    def test_cell_thresholds_scale_by_area_and_length(self):
        reference = TerrainGeometryEstimator()._effective_cell_thresholds()
        half = TerrainGeometryEstimator(
            TerrainGeometryConfig(cell_size_m=0.5)
        )._effective_cell_thresholds()
        double = TerrainGeometryEstimator(
            TerrainGeometryConfig(cell_size_m=2.0)
        )._effective_cell_thresholds()

        self.assertEqual(reference.min_points, 80)
        self.assertEqual(reference.min_inliers, 10)
        self.assertAlmostEqual(reference.min_principal_stddev, 0.1)
        self.assertEqual(reference.max_cells_fit, 100)
        self.assertEqual(half.min_points, 20)
        self.assertEqual(half.min_inliers, 3)
        self.assertAlmostEqual(half.min_principal_stddev, 0.05)
        self.assertEqual(half.max_cells_fit, 400)
        self.assertEqual(double.min_points, 320)
        self.assertEqual(double.min_inliers, 40)
        self.assertAlmostEqual(double.min_principal_stddev, 0.2)
        self.assertEqual(double.max_cells_fit, 25)

    def test_invalid_grid_and_accumulation_parameters_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "radius_m"):
            TerrainGeometryEstimator(TerrainGeometryConfig(radius_m=0.0))
        with self.assertRaisesRegex(ValueError, "cell_size_m"):
            TerrainGeometryEstimator(TerrainGeometryConfig(cell_size_m=0.0))
        with self.assertRaisesRegex(ValueError, "ttl_seconds"):
            TerrainGeometryEstimator(TerrainGeometryConfig(ttl_seconds=-1.0))
        with self.assertRaisesRegex(ValueError, "point_window_seconds"):
            TerrainGeometryEstimator(
                TerrainGeometryConfig(point_window_seconds=0.0)
            )
        with self.assertRaisesRegex(ValueError, "voxel_size_m"):
            TerrainGeometryEstimator(TerrainGeometryConfig(voxel_size_m=0.0))
        with self.assertRaisesRegex(ValueError, "fit_every_frames"):
            TerrainGeometryEstimator(TerrainGeometryConfig(fit_every_frames=0))
        with self.assertRaisesRegex(ValueError, "plane_ema_alpha"):
            TerrainGeometryEstimator(TerrainGeometryConfig(plane_ema_alpha=0.0))
        with self.assertRaisesRegex(ValueError, "max_cell_rmse"):
            TerrainGeometryEstimator(TerrainGeometryConfig(max_cell_rmse=0.0))
        with self.assertRaisesRegex(ValueError, "finite"):
            TerrainGeometryEstimator(
                TerrainGeometryConfig(radius_m=float("nan"))
            )

    def test_disabled_map_returns_empty_update(self):
        terrain_map = TerrainGeometryEstimator(
            TerrainGeometryConfig(enabled=False)
        )

        update = update_terrain(
            terrain_map,
            plane_points(),
            pose_matrix(),
            1.0,
            True,
        )

        self.assertEqual(update.accepted_planes, ())
        self.assertEqual(update.rejected_planes, ())
        self.assertEqual(update.debug_cells, ())
        self.assertEqual(terrain_map.cell_count, 0)


if __name__ == "__main__":
    unittest.main()
