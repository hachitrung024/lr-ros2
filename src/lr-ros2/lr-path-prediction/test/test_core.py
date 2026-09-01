import math

import numpy as np
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from vision_msgs.msg import Detection3D, Detection3DArray

from lr_path_prediction.core import (
    GridMapSampler,
    obstacles_from_message,
    predict_steps,
)


def _layer(values):
    values = np.asarray(values, dtype=np.float32)
    rows, columns = values.shape
    message = Float32MultiArray()
    message.layout.dim = [
        MultiArrayDimension(
            label="column_index",
            size=columns,
            stride=rows * columns,
        ),
        MultiArrayDimension(
            label="row_index",
            size=rows,
            stride=rows,
        ),
    ]
    message.data = values.reshape(-1, order="F").tolist()
    return message


def _terrain_grid(slope=25.0):
    message = GridMap()
    message.header.frame_id = "map"
    message.info.resolution = 1.0
    message.info.length_x = 3.0
    message.info.length_y = 3.0
    message.info.pose.orientation.w = 1.0
    message.layers = [
        "elevation",
        "slope_deg",
        "normal_x",
        "normal_y",
        "normal_z",
        "state",
    ]
    center_only = np.full((3, 3), np.nan, dtype=np.float32)
    elevation = center_only.copy()
    elevation[1, 1] = 0.4
    slopes = center_only.copy()
    slopes[1, 1] = slope
    normal_x = center_only.copy()
    normal_x[1, 1] = 0.0
    normal_y = center_only.copy()
    normal_y[1, 1] = math.sin(math.radians(slope))
    normal_z = center_only.copy()
    normal_z[1, 1] = math.cos(math.radians(slope))
    state = center_only.copy()
    state[1, 1] = 1.0
    message.data = [
        _layer(values)
        for values in (
            elevation,
            slopes,
            normal_x,
            normal_y,
            normal_z,
            state,
        )
    ]
    return message


def _path(count=22):
    message = Path()
    message.header.frame_id = "map"
    message.header.stamp.sec = 10
    for index in range(count):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp.sec = 10
        pose.header.stamp.nanosec = index * 100_000_000
        pose.pose.position.x = float(index)
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def test_grid_map_sampler_reads_slope_elevation_and_normal():
    sampler = GridMapSampler(_terrain_grid(slope=25.0))

    sample = sampler.sample(0.0, 0.0)

    assert sample.valid
    assert sample.elevation_m == np.float32(0.4)
    assert sample.slope_deg == 25.0
    np.testing.assert_allclose(
        sample.normal_xyz,
        [0.0, math.sin(math.radians(25.0)), math.cos(math.radians(25.0))],
        atol=1e-6,
    )
    assert not sampler.sample(5.0, 0.0).valid


def test_prediction_uses_exactly_20_future_poses_and_excludes_current():
    predictions = predict_steps(
        _path(),
        terrain=None,
        obstacles=[],
        object_data_available=True,
        step_count=20,
        path_stride=1,
        rover_radius_m=0.5,
        collision_margin_m=0.1,
    )

    assert len(predictions) == 20
    assert predictions[0].source_pose_index == 1
    assert predictions[-1].source_pose_index == 20
    assert predictions[0].distance_from_start_m == 1.0
    assert predictions[-1].time_from_start_sec == 2.0


def test_oriented_box_collision_includes_rover_radius_and_margin():
    detections = Detection3DArray()
    detection = Detection3D()
    detection.id = "rock-7"
    detection.bbox.center.position.x = 2.0
    detection.bbox.size.x = 1.0
    detection.bbox.size.y = 1.0
    detection.bbox.size.z = 1.0
    detection.bbox.center.orientation.w = 1.0
    detections.detections.append(detection)

    predictions = predict_steps(
        _path(count=4),
        terrain=None,
        obstacles=obstacles_from_message(detections),
        object_data_available=True,
        step_count=3,
        path_stride=1,
        rover_radius_m=0.4,
        collision_margin_m=0.1,
    )

    assert predictions[0].object_collision
    assert predictions[0].object_ids == ("rock-7",)
    assert predictions[1].object_collision
    assert predictions[2].object_collision


def test_missing_object_message_is_unknown_not_clear():
    prediction = predict_steps(
        _path(count=2),
        terrain=None,
        obstacles=[],
        object_data_available=False,
        step_count=1,
        path_stride=1,
        rover_radius_m=0.5,
        collision_margin_m=0.1,
    )[0]

    assert not prediction.object_data_available
    assert not prediction.object_collision
    assert math.isinf(prediction.nearest_object_clearance_m)
