"""Tests for canonical raw-evidence presentation semantics."""

from types import SimpleNamespace
from copy import deepcopy

import pytest
import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from safety_perception_msgs.msg import (
    GeometryArray,
    GeometryStep,
    PredictionOutput,
    Trajectory,
    TrajectoryStep,
    TrajectoryPath,
)
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from lr_path_prediction.canonical_visualizer_node import (
    CanonicalPredictionVisualizerNode,
    intersecting_collision_objects,
)


def test_exclamation_requires_actual_footprint_intersection():
    """A positive-clearance safety-margin candidate must remain gray."""
    intersecting = SimpleNamespace(min_distance_m=0.0)
    touching_with_roundoff = SimpleNamespace(min_distance_m=1e-10)
    margin_only = SimpleNamespace(min_distance_m=0.05)

    result = intersecting_collision_objects(
        [
            margin_only,
            intersecting,
            touching_with_roundoff,
        ]
    )

    assert result == [intersecting, touching_with_roundoff]


class Publisher:
    def __init__(self, subscribers=1):
        self.messages = []
        self.subscribers = subscribers

    def get_subscription_count(self):
        return self.subscribers

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def visualizer():
    rclpy.init()
    node = CanonicalPredictionVisualizerNode()
    node._markers_publisher = Publisher()
    node._steps_publisher = Publisher()
    node._reference_path_publisher = Publisher()
    node._now_ns = lambda: 11_000_000_000
    yield node
    node.destroy_node()
    rclpy.shutdown()


def sample_inputs():
    header = Header(frame_id='map', stamp=Time(sec=10))
    trajectory = Trajectory(
        header=header, trajectory_id=7, steps=[TrajectoryStep(step_id=0, x=1.0)]
    )
    geometry = GeometryArray(
        header=header,
        source_trajectory_id=7,
        source_trajectory_stamp=header.stamp,
        steps=[GeometryStep(step_id=0, plane_id='p')],
    )
    geometry.steps[0].normal.z = 1.0
    geometry.steps[0].elevation_m = 0.4
    geometry.steps[0].elevation_valid = True
    output_header = Header(frame_id='map', stamp=Time(sec=11))
    output = PredictionOutput(header=output_header, source_trajectory_id=7)
    return trajectory, geometry, output


def source_path(trajectory):
    snapshot = TrajectoryPath(
        header=trajectory.header, trajectory_id=trajectory.trajectory_id,
        step_ids=[s.step_id for s in trajectory.steps], source_pose_indices=[2],
    )
    for x, y, z in [(0., 0., 2.), (.5, .2, 2.1), (1., 0., 2.2), (1.5, -.3, 2.3)]:
        pose = PoseStamped(header=trajectory.header)
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        pose.pose.orientation.w = 1.
        snapshot.poses.append(pose)
    return snapshot


def feed_cycle(node, trajectory, geometry, output):
    node._on_trajectory(trajectory)
    node._on_geometry(geometry)
    node._on_prediction(output)
    node._on_path_snapshot(source_path(trajectory))


def test_output_before_inputs_is_joined_and_clock_jump_clears_warning(visualizer):
    trajectory, geometry, output = sample_inputs()
    visualizer._on_prediction(output)
    assert not visualizer._markers_publisher.messages
    visualizer._on_trajectory(trajectory)
    visualizer._on_geometry(geometry)
    assert not visualizer._markers_publisher.messages
    visualizer._on_path_snapshot(source_path(trajectory))
    assert len(visualizer._markers_publisher.messages) == 1
    reference = visualizer._reference_path_publisher.messages[-1]
    assert reference.header.stamp.sec == 10
    assert len(reference.poses) == 4
    assert reference.poses[0].pose.position.z == pytest.approx(2.0)
    points = next(m for m in visualizer._markers_publisher.messages[-1].markers
                  if m.ns == 'prediction_steps')
    assert points.points[0] == reference.poses[2].pose.position
    assert not any(m.ns == 'prediction_path'
                   for m in visualizer._markers_publisher.messages[-1].markers)
    visualizer._on_time_jump(None)
    assert visualizer._markers_publisher.messages[-1].markers[0].action == Marker.DELETEALL
    assert not visualizer._reference_path_publisher.messages[-1].poses
    assert not visualizer._predictions


def test_late_marker_subscriber_gets_latest_prediction_while_paused(visualizer):
    visualizer._markers_publisher.subscribers = 0
    trajectory, geometry, output = sample_inputs()
    feed_cycle(visualizer, trajectory, geometry, output)
    assert not visualizer._markers_publisher.messages
    visualizer._markers_publisher.subscribers = 1
    visualizer._flush_pending()
    assert len(visualizer._markers_publisher.messages) == 1


def test_unavailable_evidence_clears_warning_without_rendering_cached_result(visualizer):
    trajectory, geometry, output = sample_inputs()
    feed_cycle(visualizer, trajectory, geometry, output)
    diagnostic = DiagnosticArray(
        header=output.header,
        status=[
            DiagnosticStatus(
                name='prediction_node',
                level=DiagnosticStatus.WARN,
                message='stale tracked objects',
            )
        ],
    )
    visualizer._on_status(diagnostic)
    assert visualizer._markers_publisher.messages[-1].markers[0].action == Marker.DELETEALL
    count = len(visualizer._markers_publisher.messages)
    visualizer._flush_pending()
    assert len(visualizer._markers_publisher.messages) == count
    assert len(visualizer._reference_path_publisher.messages[-1].poses) == 4
    assert all(m.ns != 'collision_warning_symbol'
               for m in visualizer._markers_publisher.messages[-1].markers)
    assert any(m.ns == 'terrain_normals'
               for m in visualizer._markers_publisher.messages[-1].markers)


def test_periodic_ok_diagnostic_does_not_clear_prediction(visualizer):
    """ROS byte-valued OK must not be mistaken for a nonzero numeric severity."""
    trajectory, geometry, output = sample_inputs()
    feed_cycle(visualizer, trajectory, geometry, output)
    count = len(visualizer._markers_publisher.messages)
    visualizer._on_status(DiagnosticArray(
        header=output.header,
        status=[DiagnosticStatus(name='prediction_node', level=DiagnosticStatus.OK)],
    ))
    visualizer._flush_pending()
    assert visualizer._prediction_ready
    assert visualizer._display_key == (7, True)
    assert len(visualizer._markers_publisher.messages) == count


def test_waiting_next_cycle_does_not_invalidate_fresh_completed_cycle(visualizer):
    trajectory, geometry, output = sample_inputs()
    feed_cycle(visualizer, trajectory, geometry, output)
    count = len(visualizer._markers_publisher.messages)
    diagnostic = DiagnosticArray(header=output.header, status=[DiagnosticStatus(
        name='prediction_node', level=DiagnosticStatus.WARN,
        message='missing tracked objects batch',
        values=[KeyValue(key='trajectory_id', value='8')],
    )])
    visualizer._on_status(diagnostic)
    assert visualizer._display_key == (7, True)
    assert len(visualizer._markers_publisher.messages) == count
    diagnostic.status[0].values[0].value = '7'
    visualizer._on_status(diagnostic)
    assert visualizer._display_key == (7, False)
    # A real result arriving later in the same paused ROS tick can recover.
    visualizer._on_prediction(output)
    assert visualizer._display_key == (7, True)


def test_geometry_arrival_and_expiry_are_independent_of_object_prediction(visualizer):
    trajectory, geometry, _ = sample_inputs()
    visualizer._max_path_age = 5.
    visualizer._on_path_snapshot(source_path(trajectory))
    visualizer._on_trajectory(trajectory)
    assert not any(m.ns == 'terrain_normals'
                   for m in visualizer._markers_publisher.messages[-1].markers)
    visualizer._on_geometry(geometry)
    assert visualizer._display_key == (7, False)
    assert any(m.ns == 'terrain_normals'
               for m in visualizer._markers_publisher.messages[-1].markers)
    count = len(visualizer._markers_publisher.messages)
    visualizer._flush_pending()  # Pausing must not age ROS-stamped terrain.
    assert len(visualizer._markers_publisher.messages) == count
    visualizer._now_ns = lambda: 12_100_000_000
    visualizer._flush_pending()
    assert visualizer._reference_path_publisher.messages[-1].poses
    assert not any(m.ns == 'terrain_normals'
                   for m in visualizer._markers_publisher.messages[-1].markers)


def test_async_next_path_keeps_terrain_until_its_geometry_arrives(visualizer):
    trajectory, geometry, _ = sample_inputs()
    visualizer._on_path_snapshot(source_path(trajectory))
    visualizer._on_trajectory(trajectory)
    visualizer._on_geometry(geometry)
    count = len(visualizer._markers_publisher.messages)
    newer = deepcopy(trajectory)
    newer.trajectory_id = 8
    newer.header.stamp.nanosec = 500_000_000
    visualizer._on_path_snapshot(source_path(newer))
    visualizer._on_trajectory(newer)
    assert visualizer._display_key == (7, False)
    assert len(visualizer._markers_publisher.messages) == count
    next_geometry = deepcopy(geometry)
    next_geometry.source_trajectory_id = 8
    next_geometry.source_trajectory_stamp = newer.header.stamp
    visualizer._on_geometry(next_geometry)
    assert visualizer._display_key == (8, False)
    assert any(m.ns == 'terrain_normals'
               for m in visualizer._markers_publisher.messages[-1].markers)


def test_missing_terrain_keeps_dense_path_and_gray_points(visualizer):
    trajectory, geometry, output = sample_inputs()
    geometry.steps = []
    feed_cycle(visualizer, trajectory, geometry, output)
    assert len(visualizer._reference_path_publisher.messages[-1].poses) == 4
    points = next(m for m in visualizer._markers_publisher.messages[-1].markers
                  if m.ns == 'prediction_steps')
    assert points.points[0].z == 2.2
    assert points.colors[0].r == points.colors[0].g == points.colors[0].b


def test_new_path_does_not_mix_with_older_prediction_or_move_display_backward(visualizer):
    trajectory, geometry, output = sample_inputs()
    feed_cycle(visualizer, trajectory, geometry, output)
    newer = deepcopy(trajectory)
    newer.trajectory_id = 8
    newer.header.stamp.nanosec = 500_000_000
    snapshot = source_path(newer)
    snapshot.poses[0].pose.position.z = 9.
    visualizer._on_path_snapshot(snapshot)
    visualizer._on_trajectory(newer)
    # Continue showing the complete fresh cycle, including its original dense path.
    shown = visualizer._reference_path_publisher.messages[-1]
    assert shown.header.stamp == trajectory.header.stamp
    newest_output = deepcopy(output)
    newest_output.source_trajectory_id = 9  # Its inputs have not arrived yet.
    visualizer._on_prediction(newest_output)
    assert visualizer._displayed_cycle == 7
    newer_geometry = deepcopy(geometry)
    newer_geometry.source_trajectory_id = 8
    newer_geometry.source_trajectory_stamp = newer.header.stamp
    newer_output = deepcopy(output)
    newer_output.source_trajectory_id = 8
    visualizer._on_geometry(newer_geometry)
    visualizer._on_prediction(newer_output)
    assert visualizer._displayed_cycle == 8
    assert visualizer._reference_path_publisher.messages[-1].poses[0].pose.position.z == 9.
    visualizer._on_prediction(output)
    assert visualizer._displayed_cycle == 8


def test_expiry_preserves_source_path_then_clears_and_pause_does_not_age(visualizer):
    feed_cycle(visualizer, *sample_inputs())
    count = len(visualizer._markers_publisher.messages)
    visualizer._flush_pending()
    assert len(visualizer._markers_publisher.messages) == count
    visualizer._now_ns = lambda: 11_100_000_000
    visualizer._flush_pending()
    assert visualizer._display_key == (7, False)
    assert visualizer._reference_path_publisher.messages[-1].poses
    visualizer._now_ns = lambda: 12_000_000_000
    visualizer._flush_pending()
    assert not visualizer._reference_path_publisher.messages[-1].poses


@pytest.mark.parametrize('case', ['xy', 'timestamp', 'index', 'duplicate', 'frame'])
def test_mismatched_source_mapping_is_not_rendered(visualizer, case):
    trajectory, geometry, output = sample_inputs()
    snapshot = source_path(trajectory)
    if case == 'xy':
        snapshot.poses[2].pose.position.x = 100.
    elif case == 'timestamp':
        snapshot.header = deepcopy(snapshot.header)
        snapshot.header.stamp.nanosec = 1
    elif case == 'index':
        snapshot.source_pose_indices = [200]
    elif case == 'duplicate':
        snapshot.step_ids = [0, 0]
        snapshot.source_pose_indices = [1, 2]
    else:
        snapshot.header = deepcopy(snapshot.header)
        snapshot.header.frame_id = 'odom'
    visualizer._on_prediction(output)
    visualizer._on_geometry(geometry)
    visualizer._on_trajectory(trajectory)
    visualizer._on_path_snapshot(snapshot)
    assert not visualizer._reference_path_publisher.messages
