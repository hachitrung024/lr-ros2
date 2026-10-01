"""ROS messages and RViz rendering for tracked boxes and footprints."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point, Pose
from safety_perception_msgs.msg import Point2D, TrackedObject, TrackedObjectArray
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from .box_geometry import TrackedBox, _signed_polygon_area


def _detection_array(
    header: Header,
    boxes: list[TrackedBox],
) -> Detection3DArray:
    """Convert tracked OBBs into the standard ROS vision_msgs output."""
    array = Detection3DArray()
    array.header = header
    for box in boxes:
        detection = Detection3D()
        detection.header = header
        detection.id = str(box.track_id)
        detection.bbox.center = _pose(box.center, box.orientation)
        detection.bbox.size.x = float(box.size[0])
        detection.bbox.size.y = float(box.size[1])
        detection.bbox.size.z = float(box.size[2])
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "unknown"
        hypothesis.hypothesis.score = 1.0
        hypothesis.pose.pose = detection.bbox.center
        detection.results.append(hypothesis)
        array.detections.append(detection)
    return array


def _tracked_object_array(
    header: Header,
    boxes: list[TrackedBox],
) -> TrackedObjectArray:
    """Convert mask-derived footprints into the canonical collision input."""
    array = TrackedObjectArray()
    array.header = header
    for box in boxes:
        if box.footprint_xy is None or box.footprint_xy.shape[0] < 3:
            continue
        tracked_object = TrackedObject()
        tracked_object.track_id = box.track_id
        tracked_object.class_name = "unknown"
        tracked_object.confidence = 1.0
        tracked_object.confidence_valid = True
        tracked_object.velocity_valid = False
        tracked_object.footprint_polygon_xy = [
            Point2D(x=float(point[0]), y=float(point[1])) for point in box.footprint_xy
        ]
        array.objects.append(tracked_object)
    return array


def _marker_array(
    header: Header,
    boxes: list[TrackedBox],
    *,
    line_width_m: float,
    fill_alpha: float,
    label_scale_m: float,
    footprint_z_offset_m: float,
    prism_fill_alpha: float,
    show_boxes: bool,
) -> MarkerArray:
    """Render extruded footprints, optional boxes, and persistent-ID labels."""
    array = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    array.markers.append(clear)
    for box in boxes:
        red, green, blue = _track_color(box.track_id)
        minimum_z, maximum_z = _tracked_box_z_limits(box)
        if show_boxes:
            cube = Marker()
            cube.header = header
            cube.ns = "segmentation_box_fills"
            cube.id = box.track_id * 3
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose = _pose(box.center, box.orientation)
            cube.scale.x = float(box.size[0])
            cube.scale.y = float(box.size[1])
            cube.scale.z = float(box.size[2])
            cube.color.r = red
            cube.color.g = green
            cube.color.b = blue
            cube.color.a = fill_alpha
            array.markers.append(cube)

            frame = Marker()
            frame.header = header
            frame.ns = "segmentation_box_frames"
            frame.id = box.track_id * 3 + 1
            frame.type = Marker.LINE_LIST
            frame.action = Marker.ADD
            frame.pose = _pose(box.center, box.orientation)
            frame.scale.x = line_width_m
            frame.color.r = red
            frame.color.g = green
            frame.color.b = blue
            frame.color.a = 1.0
            frame.points = _box_wireframe_points(box.size)
            array.markers.append(frame)

        if box.footprint_xy is not None and box.footprint_xy.shape[0] >= 3:
            bottom_z = minimum_z + footprint_z_offset_m
            top_z = maximum_z + footprint_z_offset_m

            prism = Marker()
            prism.header = header
            prism.ns = "segmentation_footprint_prisms"
            prism.id = box.track_id
            prism.type = Marker.TRIANGLE_LIST
            prism.action = Marker.ADD
            prism.pose.orientation.w = 1.0
            prism.scale.x = prism.scale.y = prism.scale.z = 1.0
            prism.color.r = red
            prism.color.g = green
            prism.color.b = blue
            prism.color.a = prism_fill_alpha
            prism.points = _prism_surface_points(box.footprint_xy, bottom_z, top_z)
            array.markers.append(prism)

            edges = Marker()
            edges.header = header
            edges.ns = "segmentation_footprint_prism_edges"
            edges.id = box.track_id
            edges.type = Marker.LINE_LIST
            edges.action = Marker.ADD
            edges.pose.orientation.w = 1.0
            edges.scale.x = line_width_m
            edges.color.r = red
            edges.color.g = green
            edges.color.b = blue
            edges.color.a = 1.0
            edges.points = _prism_edge_points(box.footprint_xy, bottom_z, top_z)
            array.markers.append(edges)

        label = Marker()
        label.header = header
        label.ns = "segmentation_box_ids"
        label.id = box.track_id * 3 + 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = float(box.center[0])
        label.pose.position.y = float(box.center[1])
        label.pose.position.z = maximum_z + 0.15
        label.pose.orientation.w = 1.0
        label.scale.z = label_scale_m
        label.color.r = red
        label.color.g = green
        label.color.b = blue
        label.color.a = 1.0
        label.text = f"ID {box.track_id}"
        array.markers.append(label)
    return array


def _tracked_box_z_limits(box: TrackedBox) -> tuple[float, float]:
    """Return ordered global-Z limits for an extruded footprint."""
    if box.minimum_z is None or box.maximum_z is None:
        half_height = float(box.size[2]) * 0.5
        return float(box.center[2] - half_height), float(box.center[2] + half_height)
    return min(box.minimum_z, box.maximum_z), max(box.minimum_z, box.maximum_z)


def _prism_surface_points(
    polygon: np.ndarray,
    bottom_z: float,
    top_z: float,
) -> list[Point]:
    """Triangulate the caps and side walls of a vertical polygon prism."""
    points = []
    for first, second, third in _triangulate_polygon_xy(polygon):
        points.extend(
            [
                _point_at_z(polygon[third], bottom_z),
                _point_at_z(polygon[second], bottom_z),
                _point_at_z(polygon[first], bottom_z),
                _point_at_z(polygon[first], top_z),
                _point_at_z(polygon[second], top_z),
                _point_at_z(polygon[third], top_z),
            ]
        )
    for index in range(polygon.shape[0]):
        following = (index + 1) % polygon.shape[0]
        points.extend(
            [
                _point_at_z(polygon[index], bottom_z),
                _point_at_z(polygon[following], bottom_z),
                _point_at_z(polygon[following], top_z),
                _point_at_z(polygon[index], bottom_z),
                _point_at_z(polygon[following], top_z),
                _point_at_z(polygon[index], top_z),
            ]
        )
    return points


def _prism_edge_points(
    polygon: np.ndarray,
    bottom_z: float,
    top_z: float,
) -> list[Point]:
    """Return bottom, top, and vertical edges of a polygon prism."""
    points = []
    for index in range(polygon.shape[0]):
        following = (index + 1) % polygon.shape[0]
        points.extend(
            [
                _point_at_z(polygon[index], bottom_z),
                _point_at_z(polygon[following], bottom_z),
                _point_at_z(polygon[index], top_z),
                _point_at_z(polygon[following], top_z),
                _point_at_z(polygon[index], bottom_z),
                _point_at_z(polygon[index], top_z),
            ]
        )
    return points


def _triangulate_polygon_xy(polygon: np.ndarray) -> list[tuple[int, int, int]]:
    """Triangulate a simple polygon using deterministic ear clipping."""
    vertices = list(range(polygon.shape[0]))
    if _signed_polygon_area(polygon) < 0.0:
        vertices.reverse()
    triangles = []
    while len(vertices) > 3:
        ear_found = False
        for position, current in enumerate(vertices):
            previous = vertices[position - 1]
            following = vertices[(position + 1) % len(vertices)]
            first = polygon[previous]
            second = polygon[current]
            third = polygon[following]
            if _triangle_cross(first, second, third) <= 1e-10:
                continue
            if any(
                _point_in_triangle(polygon[index], first, second, third)
                for index in vertices
                if index not in (previous, current, following)
            ):
                continue
            triangles.append((previous, current, following))
            del vertices[position]
            ear_found = True
            break
        if not ear_found:
            return []
    if len(vertices) == 3:
        triangles.append(tuple(vertices))
    return triangles


def _triangle_cross(first, second, third) -> float:
    """Return twice the signed area of a triangle."""
    return float(
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def _point_in_triangle(point, first, second, third) -> bool:
    """Return whether a point lies in or on a counter-clockwise triangle."""
    epsilon = 1e-10
    return (
        _triangle_cross(first, second, point) >= -epsilon
        and _triangle_cross(second, third, point) >= -epsilon
        and _triangle_cross(third, first, point) >= -epsilon
    )


def _point_at_z(point_xy, z: float) -> Point:
    """Create a visualization point at one XY polygon vertex and height."""
    return Point(x=float(point_xy[0]), y=float(point_xy[1]), z=float(z))


def _box_wireframe_points(size: np.ndarray) -> list[Point]:
    """Return the twelve local-space edges of an oriented box."""
    half_x, half_y, half_z = (float(value) * 0.5 for value in size)
    corners = (
        (-half_x, -half_y, -half_z),
        (half_x, -half_y, -half_z),
        (half_x, half_y, -half_z),
        (-half_x, half_y, -half_z),
        (-half_x, -half_y, half_z),
        (half_x, -half_y, half_z),
        (half_x, half_y, half_z),
        (-half_x, half_y, half_z),
    )
    edge_indices = (
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    )
    return [
        Point(x=corners[index][0], y=corners[index][1], z=corners[index][2])
        for edge in edge_indices
        for index in edge
    ]


def _track_color(track_id: int) -> tuple[float, float, float]:
    """Generate a stable, high-contrast RGB color for a tracker ID."""
    return (
        0.35 + 0.65 * ((track_id * 53) % 255) / 255.0,
        0.35 + 0.65 * ((track_id * 97) % 255) / 255.0,
        0.35 + 0.65 * ((track_id * 193) % 255) / 255.0,
    )


def _pose(center: np.ndarray, orientation: np.ndarray) -> Pose:
    """Create a geometry pose from numeric center and quaternion values."""
    pose = Pose()
    pose.position.x = float(center[0])
    pose.position.y = float(center[1])
    pose.position.z = float(center[2])
    pose.orientation.x = float(orientation[0])
    pose.orientation.y = float(orientation[1])
    pose.orientation.z = float(orientation[2])
    pose.orientation.w = float(orientation[3])
    return pose
