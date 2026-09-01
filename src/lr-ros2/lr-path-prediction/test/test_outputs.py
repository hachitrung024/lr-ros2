import math

from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from lr_path_prediction.core import StepPrediction, TerrainSample
from lr_path_prediction.outputs import (
    predictions_to_diagnostics,
    predictions_to_markers,
)


def _prediction(
    *,
    slope=10.0,
    collision=False,
    step_index=1,
    distance=0.2,
    x_value=1.0,
):
    return StepPrediction(
        step_index=step_index,
        source_pose_index=step_index,
        position_xyz=(x_value, 2.0, 3.0),
        distance_from_start_m=distance,
        time_from_start_sec=0.25,
        terrain=TerrainSample(
            valid=True,
            elevation_m=0.3,
            slope_deg=slope,
            normal_xyz=(0.0, 0.0, 1.0),
        ),
        object_data_available=True,
        object_collision=collision,
        object_ids=("5",) if collision else (),
        nearest_object_clearance_m=-0.1 if collision else math.inf,
    )


def test_diagnostics_report_slope_normal_and_object_collision():
    message = predictions_to_diagnostics(
        [_prediction(slope=35.0, collision=True)],
        Header(frame_id="map"),
        slope_warning_deg=20.0,
        slope_critical_deg=30.0,
    )

    assert len(message.status) == 1
    assert message.status[0].level == DiagnosticStatus.ERROR
    values = {item.key: item.value for item in message.status[0].values}
    assert values["slope_deg"] == "35.000000"
    assert values["normal_z"] == "1.000000"
    assert values["object_collision"] == "true"
    assert values["object_ids"] == "5"


def test_markers_include_points_slope_label_and_normal():
    message = predictions_to_markers(
        [_prediction()],
        Header(frame_id="map"),
        slope_warning_deg=20.0,
        slope_critical_deg=30.0,
        normal_length_m=0.8,
        marker_z_offset_m=0.1,
        label_height_m=0.45,
    )

    assert message.markers[0].action == Marker.DELETEALL
    namespaces = {marker.ns for marker in message.markers[1:]}
    assert namespaces == {
        "prediction_path",
        "prediction_steps",
        "prediction_labels",
        "terrain_normals",
    }
    label = next(
        marker for marker in message.markers
        if marker.ns == "prediction_labels"
    )
    assert label.text == "10.0 deg"
    assert label.color.r == 0.55
    assert label.color.g == 0.55
    assert label.color.b == 0.55
    normal = next(
        marker for marker in message.markers
        if marker.ns == "terrain_normals"
    )
    assert normal.points[0].z == 0.4
    assert math.isclose(normal.points[1].z, 1.2)


def test_collision_warning_is_a_large_triangle_with_separate_symbol():
    message = predictions_to_markers(
        [_prediction(collision=True)],
        Header(frame_id="map"),
        slope_warning_deg=20.0,
        slope_critical_deg=30.0,
        normal_length_m=0.8,
        marker_z_offset_m=0.1,
        label_height_m=0.45,
    )

    slope_label = next(
        marker for marker in message.markers
        if marker.ns == "prediction_labels"
    )
    assert slope_label.text == "10.0 deg"
    symbol = next(
        marker for marker in message.markers
        if marker.ns == "collision_warning_symbol"
    )
    outline = next(
        marker for marker in message.markers
        if marker.ns == "collision_warning_outline"
    )
    assert symbol.text == "!"
    assert symbol.scale.z == 0.60
    assert symbol.pose.position.x == 1.0
    assert outline.type == Marker.LINE_STRIP
    assert outline.scale.x == 0.08
    assert len(outline.points) == 4
    assert outline.points[0].z > outline.points[1].z


def test_only_nearest_collision_is_highlighted():
    far = _prediction(
        collision=True,
        step_index=5,
        distance=5.0,
        x_value=5.0,
    )
    near = _prediction(
        collision=True,
        step_index=2,
        distance=2.0,
        x_value=2.0,
    )

    message = predictions_to_markers(
        [far, near],
        Header(frame_id="map"),
        slope_warning_deg=20.0,
        slope_critical_deg=30.0,
        normal_length_m=0.8,
        marker_z_offset_m=0.1,
        label_height_m=0.45,
    )

    symbols = [
        marker for marker in message.markers
        if marker.ns == "collision_warning_symbol"
    ]
    assert len(symbols) == 1
    assert symbols[0].pose.position.x == 2.0
    points = next(
        marker for marker in message.markers
        if marker.ns == "prediction_steps"
    )
    assert points.colors[0].r == 0.55
    assert points.colors[1].r == 1.0
    assert points.colors[1].b == 0.55
