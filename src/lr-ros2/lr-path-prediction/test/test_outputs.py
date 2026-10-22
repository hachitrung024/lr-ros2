import math

from diagnostic_msgs.msg import DiagnosticStatus
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from lr_path_prediction.presentation import StepPrediction, TerrainSample
from lr_path_prediction.outputs import (
    predictions_to_diagnostics,
    predictions_to_markers,
    predictions_to_reference_path,
)


def _prediction(
    *,
    slope=10.0,
    collision=False,
    step_index=1,
    distance=0.2,
    x_value=1.0,
    terrain_valid=True,
):
    return StepPrediction(
        step_index=step_index,
        source_pose_index=step_index,
        position_xyz=(x_value, 2.0, 3.0),
        distance_from_start_m=distance,
        time_from_start_sec=0.25,
        terrain=TerrainSample(
            valid=terrain_valid,
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
        "prediction_steps",
        "prediction_labels",
        "terrain_normals",
    }
    label = next(marker for marker in message.markers if marker.ns == "prediction_labels")
    assert label.text == "10.0 deg"
    assert label.color.r == 0.55
    assert label.color.g == 0.55
    assert label.color.b == 0.55
    normal = next(marker for marker in message.markers if marker.ns == "terrain_normals")
    assert normal.points[0].z == 3.1
    assert math.isclose(normal.points[1].z, 3.9)


def test_reference_path_uses_exact_marker_positions_and_yaw():
    prediction = _prediction()
    header = Header(frame_id="map")
    markers = predictions_to_markers(
        [prediction],
        header,
        slope_warning_deg=20.0,
        slope_critical_deg=30.0,
        normal_length_m=0.8,
        marker_z_offset_m=0.1,
        label_height_m=0.45,
    )
    path = predictions_to_reference_path(
        [prediction], header, marker_z_offset_m=0.1
    )

    marker_points = next(
        marker for marker in markers.markers if marker.ns == "prediction_steps"
    ).points
    assert len(path.poses) == len(marker_points) == 1
    assert path.poses[0].pose.position == marker_points[0]
    assert path.poses[0].pose.orientation.w == 1.0


def test_unknown_terrain_preserves_that_step_and_the_remaining_horizon():
    predictions = [
        _prediction(step_index=1, x_value=1.0),
        _prediction(step_index=2, x_value=2.0, terrain_valid=False),
        _prediction(step_index=3, x_value=3.0),
    ]
    header = Header(frame_id="map")

    markers = predictions_to_markers(
        predictions,
        header,
        slope_warning_deg=20.0,
        slope_critical_deg=30.0,
        normal_length_m=0.8,
        marker_z_offset_m=0.1,
        label_height_m=0.45,
    )
    path = predictions_to_reference_path(
        predictions, header, marker_z_offset_m=0.1
    )
    diagnostics = predictions_to_diagnostics(
        predictions,
        header,
        slope_warning_deg=20.0,
        slope_critical_deg=30.0,
    )

    points = next(
        marker for marker in markers.markers if marker.ns == "prediction_steps"
    )
    assert [point.x for point in points.points] == [1.0, 2.0, 3.0]
    assert [pose.pose.position.x for pose in path.poses] == [1.0, 2.0, 3.0]
    assert len(diagnostics.status) == 3
    assert diagnostics.status[1].message == "TERRAIN UNKNOWN"


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

    slope_label = next(marker for marker in message.markers if marker.ns == "prediction_labels")
    assert slope_label.text == "10.0 deg"
    symbol = next(marker for marker in message.markers if marker.ns == "collision_warning_symbol")
    outline = next(
        marker for marker in message.markers if marker.ns == "collision_warning_outline"
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

    symbols = [marker for marker in message.markers if marker.ns == "collision_warning_symbol"]
    assert len(symbols) == 1
    assert symbols[0].pose.position.x == 2.0
    points = next(marker for marker in message.markers if marker.ns == "prediction_steps")
    assert points.colors[0].r == 0.55
    assert points.colors[1].r == 1.0
    assert points.colors[1].b == 0.55


def test_rollover_policy_boundaries_and_valid_dynamic_evidence():
    from dataclasses import replace
    from lr_path_prediction.outputs import rollover_risk

    base = _prediction()
    for margin, expected in ((1.0, DiagnosticStatus.OK),
                             (0.20, DiagnosticStatus.WARN),
                             (0.0, DiagnosticStatus.ERROR),
                             (-0.1, DiagnosticStatus.ERROR)):
        item = replace(base, normalized_static_stability_margin=margin)
        assert rollover_risk(item) == (expected, "SSM", margin)
    item = replace(base, normalized_static_stability_margin=0.8,
                   stability_moment_valid=True, normalized_minimum_stability_moment=0.1,
                   zmp_valid=False, normalized_zmp_margin=-1.0)
    assert rollover_risk(item) == (DiagnosticStatus.WARN, "MOMENT", 0.1)
    item = replace(item, zmp_valid=True, normalized_zmp_margin=-0.1)
    assert rollover_risk(item) == (DiagnosticStatus.ERROR, "ZMP", -0.1)
    assert rollover_risk(base)[0] is None
    assert rollover_risk(replace(item, terrain=TerrainSample(valid=False)))[0] is None
    assert rollover_risk(replace(base, normalized_static_stability_margin=math.inf))[0] is None
    custom = replace(base, normalized_static_stability_margin=0.3)
    assert rollover_risk(custom, 0.4, 0.1)[0] == DiagnosticStatus.WARN


def test_rollover_markers_and_diagnostics_preserve_collision_warning():
    from dataclasses import replace

    item = replace(_prediction(collision=True), normalized_static_stability_margin=-0.1)
    diagnostics = predictions_to_diagnostics(
        [item], Header(frame_id="map"), slope_warning_deg=20.0, slope_critical_deg=30.0,
    )
    assert diagnostics.status[0].level == DiagnosticStatus.ERROR
    assert "ROLLOVER CRITICAL" in diagnostics.status[0].message
    assert "OBJECT COLLISION" in diagnostics.status[0].message
    markers = predictions_to_markers(
        [item], Header(frame_id="map"), slope_warning_deg=20.0, slope_critical_deg=30.0,
        normal_length_m=0.8, marker_z_offset_m=0.0, label_height_m=0.45,
    ).markers
    assert any(m.ns == "rollover_warning_label" and "SSM: -0.10" in m.text for m in markers)
    assert any(m.ns == "collision_warning_symbol" for m in markers)
    points = next(m for m in markers if m.ns == "prediction_steps")
    assert points.colors[0].b == 0.55  # Collision keeps its magenta point.
    # Recovery deletes old warning markers before drawing the new cycle.
    recovered = predictions_to_markers(
        [replace(item, object_collision=False, normalized_static_stability_margin=1.0)],
        Header(frame_id="map"), slope_warning_deg=20.0, slope_critical_deg=30.0,
        normal_length_m=0.8, marker_z_offset_m=0.0, label_height_m=0.45,
    ).markers
    assert recovered[0].action == Marker.DELETEALL
    assert not any(m.ns.startswith("rollover_warning") for m in recovered)


def test_rollover_unknown_and_invalid_policy():
    import pytest
    from lr_path_prediction.outputs import rollover_risk

    for warning, critical in ((0.2, 0.2), (0.1, 0.2), (math.nan, 0.0)):
        with pytest.raises(ValueError):
            rollover_risk(_prediction(), warning, critical)
    diagnostics = predictions_to_diagnostics(
        [_prediction()], Header(), slope_warning_deg=20.0, slope_critical_deg=30.0,
    )
    assert "ROLLOVER UNKNOWN" in diagnostics.status[0].message
    assert diagnostics.status[0].level == DiagnosticStatus.WARN


def test_hazard_signs_have_distinct_labels_colors_and_separate_vertical_lanes():
    from dataclasses import replace

    near = replace(_prediction(collision=True), normalized_static_stability_margin=0.1)
    far = replace(near, step_index=2, distance_from_start_m=4.0)
    markers = predictions_to_markers(
        [far, near], Header(frame_id="map"),
        slope_warning_deg=20.0, slope_critical_deg=30.0,
        normal_length_m=0.8, marker_z_offset_m=0.0, label_height_m=0.45,
    ).markers
    by_ns = {m.ns: m for m in markers}
    assert by_ns["collision_warning_label"].text == "COLLISION"
    assert "ROLLOVER WARNING" in by_ns["rollover_warning_label"].text
    for kind in ("collision", "rollover"):
        assert by_ns[f"{kind}_warning_symbol"].text == "!"
        assert by_ns[f"{kind}_warning_fill"].type == Marker.TRIANGLE_LIST
        assert len([m for m in markers if m.ns == f"{kind}_warning_symbol"]) == 1
        assert by_ns[f"{kind}_warning_symbol"].pose.position.x == near.position_xyz[0]
    collision_top = max(p.z for p in by_ns["collision_warning_outline"].points)
    rollover_bottom = min(p.z for p in by_ns["rollover_warning_outline"].points)
    assert rollover_bottom > collision_top + 0.5
    assert by_ns["collision_warning_outline"].color.b == 0.55
    assert by_ns["rollover_warning_outline"].color.g == 0.65
