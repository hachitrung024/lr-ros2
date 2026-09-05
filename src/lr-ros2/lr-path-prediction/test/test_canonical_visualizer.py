"""Tests for canonical raw-evidence presentation semantics."""

from types import SimpleNamespace

import pytest
import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from safety_perception_msgs.msg import (
    GeometryArray,
    GeometryStep,
    PredictionOutput,
    Trajectory,
    TrajectoryStep,
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
    output = PredictionOutput(header=header, source_trajectory_id=7)
    return trajectory, geometry, output


def test_output_before_inputs_is_joined_and_clock_jump_clears_warning(visualizer):
    trajectory, geometry, output = sample_inputs()
    visualizer._on_prediction(output)
    assert not visualizer._markers_publisher.messages
    visualizer._on_trajectory(trajectory)
    visualizer._on_geometry(geometry)
    assert len(visualizer._markers_publisher.messages) == 1
    visualizer._on_time_jump(None)
    assert visualizer._markers_publisher.messages[-1].markers[0].action == Marker.DELETEALL
    assert not visualizer._predictions


def test_late_marker_subscriber_gets_latest_prediction_while_paused(visualizer):
    visualizer._markers_publisher.subscribers = 0
    trajectory, geometry, output = sample_inputs()
    visualizer._on_trajectory(trajectory)
    visualizer._on_geometry(geometry)
    visualizer._on_prediction(output)
    assert not visualizer._markers_publisher.messages
    visualizer._markers_publisher.subscribers = 1
    visualizer._flush_pending()
    assert len(visualizer._markers_publisher.messages) == 1


def test_unavailable_evidence_clears_warning_without_rendering_cached_result(visualizer):
    trajectory, geometry, output = sample_inputs()
    visualizer._on_trajectory(trajectory)
    visualizer._on_geometry(geometry)
    visualizer._on_prediction(output)
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
