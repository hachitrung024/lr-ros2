"""Check projection, grouping and missing instances across depth sampling."""

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from lr_segmentation.depth_sampling import instance_point_clouds


def test_grouped_projection_preserves_spatial_sampling_and_missing_labels():
    labels = np.zeros((4, 8), dtype=np.uint16)
    labels[:, :4] = 7
    labels[:, 4:] = 9
    labels[1, 1] = 11  # An instance skipped by stride must remain unobservable.
    depth = np.where(labels == 7, 2.0, 3.0)
    depth[0, 4] = np.nan
    info = CameraInfo(k=[2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0])
    clouds = instance_point_clouds(
        labels, depth, info, minimum_depth_m=0.2, maximum_depth_m=20.0,
        sampling_stride=2, maximum_points=2, depth_mad_scale=8.0,
        minimum_depth_band_m=0.15,
    )
    np.testing.assert_array_equal(clouds[7], [[0, 0, 2], [2, 2, 2]])
    np.testing.assert_array_equal(clouds[9], [[9, 0, 3], [9, 3, 3]])
    assert clouds[11].shape == (0, 3)


@pytest.mark.parametrize("invalid_depth", [np.nan, np.inf, 0.0, 21.0])
def test_sparse_uint16_ids_with_invalid_depth_remain_unobservable(invalid_depth):
    labels = np.zeros((6, 8), dtype=np.uint16)
    labels[0, 0] = 65535
    labels[1, 2] = 32768  # Missed by stride, but still an observed mask ID.
    labels = labels[:, ::2]  # Decoded/cropped views need not be contiguous.
    info = CameraInfo(k=[2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0])
    clouds = instance_point_clouds(
        labels, np.full(labels.shape, invalid_depth), info,
        minimum_depth_m=0.2, maximum_depth_m=20.0, sampling_stride=2,
        maximum_points=10, depth_mad_scale=8.0, minimum_depth_band_m=0.15,
    )
    assert list(clouds) == [32768, 65535]
    assert all(cloud.shape == (0, 3) for cloud in clouds.values())


def test_background_only_mask_has_no_instances():
    labels = np.zeros((4, 8), dtype=np.uint16)
    info = CameraInfo(k=[2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0])
    assert instance_point_clouds(
        labels, np.ones(labels.shape), info,
        minimum_depth_m=0.2, maximum_depth_m=20.0, sampling_stride=2,
        maximum_points=10, depth_mad_scale=8.0, minimum_depth_band_m=0.15,
    ) == {}
