import math

import numpy as np
import pytest
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose

from lr_terrain_geometry.conversions import (
    instance_mask_image_to_labels,
    object_boxes_to_detection_array,
    object_boxes_to_markers,
)
from lr_terrain_geometry.object_filter import (
    ObjectFilterConfig,
    eligible_detections,
    filter_points_in_detection_boxes,
    filter_points_in_instance_masks,
)


def make_detection(
    *,
    class_id="waste",
    score=0.9,
    center=(50.0, 50.0),
    size=(30.0, 30.0),
    detection_id="frame:0",
):
    detection = Detection2D()
    detection.id = detection_id
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = class_id
    result.hypothesis.score = score
    detection.results = [result]
    detection.bbox.center.position.x = center[0]
    detection.bbox.center.position.y = center[1]
    detection.bbox.size_x = size[0]
    detection.bbox.size_y = size[1]
    return detection


def projection_matrix():
    return np.asarray([
        [100.0, 0.0, 50.0, 0.0],
        [0.0, 100.0, 50.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ])


def image_plane_grid(depth, count):
    image_coordinate = np.linspace(42.0, 58.0, count)
    uu, vv = np.meshgrid(image_coordinate, image_coordinate)
    x = (uu.reshape(-1) - 50.0) * depth / 100.0
    y = (vv.reshape(-1) - 50.0) * depth / 100.0
    z = np.full(x.shape, depth)
    return np.column_stack((x, y, z))


def test_filter_removes_nearest_supported_depth_cluster_and_keeps_background():
    object_points = image_plane_grid(3.0, 6)
    background_points = image_plane_grid(8.0, 8)
    unsupported_near_points = image_plane_grid(1.0, 2)
    outside_points = np.asarray([
        [3.0, 0.0, 3.0],
        [-3.0, 0.0, 3.0],
        [math.nan, 0.0, 2.0],
    ])
    points = np.vstack((
        object_points,
        background_points,
        unsupported_near_points,
        outside_points,
    ))
    config = ObjectFilterConfig(
        minimum_cluster_points=10,
        depth_tolerance_m=0.15,
        box_padding_m=0.05,
    )

    filtered, boxes, removed = filter_points_in_detection_boxes(
        points,
        [make_detection()],
        np.identity(4),
        projection_matrix(),
        config,
    )

    assert len(boxes) == 1
    assert boxes[0].class_id == "waste"
    assert boxes[0].detection_id == "frame:0"
    assert removed == object_points.shape[0]
    assert filtered.shape[0] == points.shape[0] - removed
    assert np.count_nonzero(np.isclose(filtered[:, 2], 8.0)) == (
        background_points.shape[0]
    )
    assert np.count_nonzero(np.isclose(filtered[:, 2], 1.0)) == (
        unsupported_near_points.shape[0]
    )
    assert np.isnan(filtered).any()


def test_filter_applies_confidence_and_class_gates():
    config = ObjectFilterConfig(
        minimum_confidence=0.5,
        classes=("waste",),
        minimum_cluster_points=3,
    )
    selected = eligible_detections(
        [
            make_detection(class_id="waste", score=0.8),
            make_detection(class_id="person", score=0.9),
            make_detection(class_id="waste", score=0.2),
        ],
        config,
    )

    assert len(selected) == 1
    assert selected[0].results[0].hypothesis.class_id == "waste"


def test_instance_mask_filter_fits_yaw_obb_and_rejects_far_depth():
    yaw = math.radians(35.0)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotation = np.asarray([[cosine, -sine], [sine, cosine]])
    along = np.linspace(-0.30, 0.30, 10)
    across = np.linspace(-0.06, 0.06, 4)
    local_x, local_y = np.meshgrid(along, across)
    local_xy = np.column_stack((local_x.reshape(-1), local_y.reshape(-1)))
    object_xy = local_xy @ rotation.T
    object_points = np.column_stack((
        object_xy,
        np.full(object_xy.shape[0], 3.0),
    ))
    background_points = object_points * (8.0 / 3.0)
    points = np.vstack((object_points, background_points))

    labels = np.zeros((100, 100), dtype=np.uint16)
    projected_u = np.rint(100.0 * object_xy[:, 0] / 3.0 + 50.0).astype(int)
    projected_v = np.rint(100.0 * object_xy[:, 1] / 3.0 + 50.0).astype(int)
    labels[projected_v, projected_u] = 1
    config = ObjectFilterConfig(
        minimum_cluster_points=10,
        depth_tolerance_m=0.15,
        box_padding_m=0.02,
    )

    filtered, boxes, removed = filter_points_in_instance_masks(
        points,
        [make_detection()],
        labels,
        np.identity(4),
        projection_matrix(),
        config,
    )

    assert removed == object_points.shape[0]
    assert filtered.shape == background_points.shape
    assert np.allclose(filtered, background_points)
    assert len(boxes) == 1
    assert boxes[0].yaw == pytest.approx(yaw, abs=math.radians(2.0))
    message = object_boxes_to_detection_array(boxes, Header())
    assert message.detections[0].bbox.center.orientation.z == pytest.approx(
        math.sin(0.5 * yaw),
        abs=0.02,
    )


def test_instance_mask_decoder_supports_padded_rows():
    expected = np.asarray([[0, 1, 2], [3, 4, 5]], dtype="<u2")
    rows = np.zeros((2, 8), dtype=np.uint8)
    rows[:, :6] = expected.view(np.uint8).reshape(2, 6)
    message = Image()
    message.height = 2
    message.width = 3
    message.encoding = "mono16"
    message.step = 8
    message.data = rows.tobytes()

    actual = instance_mask_image_to_labels(message)

    assert actual.dtype == np.uint16
    assert np.array_equal(actual, expected)


def test_object_box_serialization_preserves_cloud_frame_and_dimensions():
    points = image_plane_grid(3.0, 4)
    config = ObjectFilterConfig(
        minimum_cluster_points=3,
        box_padding_m=0.05,
    )
    _, boxes, _ = filter_points_in_detection_boxes(
        points,
        [make_detection()],
        np.identity(4),
        projection_matrix(),
        config,
    )
    header = Header(frame_id="cloud_frame")

    message = object_boxes_to_detection_array(boxes, header)

    assert message.header.frame_id == "cloud_frame"
    assert len(message.detections) == 1
    detection = message.detections[0]
    assert detection.id == "frame:0"
    assert detection.results[0].hypothesis.class_id == "waste"
    assert detection.bbox.center.orientation.w == 1.0
    assert detection.bbox.size.x > 0.0
    assert detection.bbox.size.y > 0.0
    assert detection.bbox.size.z == pytest.approx(0.10)


def test_object_box_markers_contain_wireframe_label_and_delete_all():
    points = image_plane_grid(3.0, 4)
    config = ObjectFilterConfig(
        minimum_cluster_points=3,
        box_padding_m=0.05,
    )
    _, boxes, _ = filter_points_in_detection_boxes(
        points,
        [make_detection(score=0.91)],
        np.identity(4),
        projection_matrix(),
        config,
    )
    header = Header(frame_id="cloud_frame")

    output = object_boxes_to_markers(boxes, header)

    assert len(output.markers) == 3
    assert output.markers[0].action == Marker.DELETEALL
    outline = output.markers[1]
    assert outline.header.frame_id == "cloud_frame"
    assert outline.ns == "object_boxes"
    assert outline.type == Marker.LINE_LIST
    assert len(outline.points) == 24
    assert outline.lifetime.sec == 1
    label = output.markers[2]
    assert label.ns == "object_labels"
    assert label.type == Marker.TEXT_VIEW_FACING
    assert label.text == "waste 91%"


@pytest.mark.parametrize(
    "overrides",
    [
        {"minimum_confidence": -0.1},
        {"minimum_cluster_points": 2},
        {"depth_bin_size_m": 0.0},
        {"lower_percentile": 80.0, "upper_percentile": 20.0},
        {"minimum_orientation_ratio": 0.9},
    ],
)
def test_object_filter_config_rejects_invalid_values(overrides):
    with pytest.raises(ValueError):
        ObjectFilterConfig(**overrides).validate()
