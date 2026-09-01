"""Diagnostic and RViz outputs for future-step predictions."""

from __future__ import annotations

import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from .core import StepPrediction


def predictions_to_diagnostics(
    predictions: list[StepPrediction],
    header: Header,
    *,
    slope_warning_deg: float,
    slope_critical_deg: float,
) -> DiagnosticArray:
    """Serialize every predicted step as one standard diagnostic status."""
    message = DiagnosticArray()
    message.header = header
    for prediction in predictions:
        level, labels = _risk(
            prediction,
            slope_warning_deg,
            slope_critical_deg,
        )
        status = DiagnosticStatus()
        status.level = level
        status.name = (
            f"lr_path_prediction/step_{prediction.step_index:02d}"
        )
        status.hardware_id = "landfill_rover"
        status.message = ", ".join(labels)
        normal = prediction.terrain.normal_xyz
        status.values = [
            _value("step_index", prediction.step_index),
            _value("source_pose_index", prediction.source_pose_index),
            _value("x_m", prediction.position_xyz[0]),
            _value("y_m", prediction.position_xyz[1]),
            _value("path_z_m", prediction.position_xyz[2]),
            _value("distance_from_start_m", prediction.distance_from_start_m),
            _value("time_from_start_sec", prediction.time_from_start_sec),
            _value("terrain_available", prediction.terrain.valid),
            _value("terrain_elevation_m", prediction.terrain.elevation_m),
            _value("slope_deg", prediction.terrain.slope_deg),
            _value("normal_x", normal[0] if normal else math.nan),
            _value("normal_y", normal[1] if normal else math.nan),
            _value("normal_z", normal[2] if normal else math.nan),
            _value(
                "object_data_available",
                prediction.object_data_available,
            ),
            _value("object_collision", prediction.object_collision),
            _value("object_ids", ",".join(prediction.object_ids)),
            _value(
                "nearest_object_clearance_m",
                prediction.nearest_object_clearance_m,
            ),
        ]
        message.status.append(status)
    return message


def predictions_to_markers(
    predictions: list[StepPrediction],
    header: Header,
    *,
    slope_warning_deg: float,
    slope_critical_deg: float,
    normal_length_m: float,
    marker_z_offset_m: float,
    label_height_m: float,
) -> MarkerArray:
    """Build path points, warnings, labels, and terrain-normal arrows."""
    output = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    output.markers.append(clear)
    if not predictions:
        return output

    line = _base_marker(header, "prediction_path", 0, Marker.LINE_STRIP)
    line.scale.x = 0.06
    points = _base_marker(header, "prediction_steps", 0, Marker.SPHERE_LIST)
    points.scale.x = 0.22
    points.scale.y = 0.22
    points.scale.z = 0.22

    for prediction in predictions:
        color = _risk_color(
            prediction,
            slope_warning_deg,
            slope_critical_deg,
        )
        position = _display_position(prediction, marker_z_offset_m)
        line.points.append(position)
        line.colors.append(color)
        points.points.append(position)
        points.colors.append(color)

        label_text = _label_text(prediction)
        if label_text:
            label = _base_marker(
                header,
                "prediction_labels",
                prediction.step_index,
                Marker.TEXT_VIEW_FACING,
            )
            label.pose.position.x = position.x
            label.pose.position.y = position.y
            label.pose.position.z = position.z + label_height_m
            label.scale.z = 0.20
            label.color = color
            label.text = label_text
            output.markers.append(label)

        if prediction.terrain.normal_xyz is not None:
            normal = prediction.terrain.normal_xyz
            arrow = _base_marker(
                header,
                "terrain_normals",
                prediction.step_index,
                Marker.ARROW,
            )
            arrow.points = [
                position,
                Point(
                    x=position.x + normal[0] * normal_length_m,
                    y=position.y + normal[1] * normal_length_m,
                    z=position.z + normal[2] * normal_length_m,
                ),
            ]
            arrow.scale.x = 0.035
            arrow.scale.y = 0.09
            arrow.scale.z = 0.12
            arrow.color = ColorRGBA(r=0.1, g=0.75, b=1.0, a=1.0)
            output.markers.append(arrow)

    output.markers.insert(1, line)
    output.markers.insert(2, points)
    return output


def _risk(
    prediction: StepPrediction,
    warning_deg: float,
    critical_deg: float,
) -> tuple[int, list[str]]:
    labels = []
    level = DiagnosticStatus.OK
    if prediction.object_collision:
        labels.append("OBJECT COLLISION")
        level = DiagnosticStatus.ERROR
    elif not prediction.object_data_available:
        labels.append("OBJECT DATA UNAVAILABLE")
        level = max(level, DiagnosticStatus.WARN)

    if not prediction.terrain.valid:
        labels.append("TERRAIN UNKNOWN")
        level = max(level, DiagnosticStatus.WARN)
    elif prediction.terrain.slope_deg >= critical_deg:
        labels.append("CRITICAL SLOPE")
        level = DiagnosticStatus.ERROR
    elif prediction.terrain.slope_deg >= warning_deg:
        labels.append("SLOPE WARNING")
        level = max(level, DiagnosticStatus.WARN)
    if not labels:
        labels.append("CLEAR")
    return level, labels


def _risk_color(
    prediction: StepPrediction,
    warning_deg: float,
    critical_deg: float,
) -> ColorRGBA:
    if prediction.object_collision:
        return ColorRGBA(r=1.0, g=0.0, b=0.55, a=1.0)
    if (
        prediction.terrain.valid
        and prediction.terrain.slope_deg >= critical_deg
    ):
        return ColorRGBA(r=1.0, g=0.05, b=0.0, a=1.0)
    if (
        prediction.terrain.valid
        and prediction.terrain.slope_deg >= warning_deg
    ):
        return ColorRGBA(r=1.0, g=0.65, b=0.0, a=1.0)
    return ColorRGBA(r=0.55, g=0.55, b=0.55, a=1.0)


def _label_text(prediction: StepPrediction) -> str:
    parts = []
    if prediction.terrain.valid:
        parts.append(f"{prediction.terrain.slope_deg:.1f} deg")
    if prediction.object_collision:
        parts.append("!")
    return " ".join(parts)


def _display_position(
    prediction: StepPrediction,
    marker_z_offset_m: float,
) -> Point:
    z_value = (
        prediction.terrain.elevation_m
        if prediction.terrain.valid
        else prediction.position_xyz[2]
    )
    return Point(
        x=prediction.position_xyz[0],
        y=prediction.position_xyz[1],
        z=z_value + marker_z_offset_m,
    )


def _base_marker(
    header: Header,
    namespace: str,
    marker_id: int,
    marker_type: int,
) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _value(key: str, value) -> KeyValue:
    if isinstance(value, bool):
        rendered = str(value).lower()
    elif isinstance(value, float):
        rendered = f"{value:.6f}"
    else:
        rendered = str(value)
    return KeyValue(key=key, value=rendered)
