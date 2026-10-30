"""Exercise callback ordering and rewind with real ROS message contracts."""

from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.parameter import Parameter
from safety_perception_msgs.msg import (
    GeometryArray,
    GeometryStep,
    RoverState,
    TrackedObjectArray,
    Trajectory,
    TrajectoryStep,
)
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from prediction_ros.prediction_node import PredictionNode


def header(sec):
    return Header(frame_id='map', stamp=Time(sec=sec))


@pytest.fixture
def node():
    rclpy.init()
    config = Path(__file__).resolve().parents[2] / 'prediction-core/config/rover.reference.yaml'
    wrapper = PredictionNode(
        parameter_overrides=[
            Parameter('config_path', value=str(config)),
            Parameter('prediction_profile', value='dynamic'),
            Parameter('use_sim_time', value=True),
        ]
    )
    wrapper.outputs = []
    wrapper._publisher = SimpleNamespace(publish=wrapper.outputs.append)
    yield wrapper
    wrapper.destroy()
    rclpy.shutdown()


def inputs(stamp=10, cycle=1):
    trajectory = Trajectory(
        header=header(stamp), trajectory_id=cycle, steps=[TrajectoryStep(step_id=0, x=1.0)]
    )
    geometry = GeometryArray(
        header=header(stamp - 1),
        source_trajectory_id=cycle,
        source_trajectory_stamp=header(stamp).stamp,
        steps=[GeometryStep(step_id=0, plane_id='p')],
    )
    geometry.steps[0].normal.z = 1.0
    return trajectory, geometry


def test_geometry_can_arrive_before_trajectory_and_state_can_arrive_after_it(node):
    trajectory, geometry = inputs()
    node._objects_callback(TrackedObjectArray(header=header(10)))
    node._geometry_callback(geometry)
    node._trajectory_callback(trajectory)
    assert not node.outputs
    # A future state must neither unlock this cycle nor hide a later-arriving valid sample.
    node._state_callback(RoverState(header=header(11), acceleration_valid=True))
    assert not node.outputs
    node._state_callback(RoverState(header=header(10), acceleration_valid=True))
    assert len(node.outputs) == 1
    node._geometry_callback(geometry)
    node._objects_callback(TrackedObjectArray(header=header(10)))
    assert len(node.outputs) == 1


def test_rewind_discards_cached_empty_objects_and_acceleration(node):
    trajectory, geometry = inputs()
    node._objects_callback(TrackedObjectArray(header=header(10)))
    node._state_callback(RoverState(header=header(10), acceleration_valid=True))
    node._geometry_callback(geometry)
    node._trajectory_callback(trajectory)
    assert len(node.outputs) == 1
    node._on_time_jump(None)
    trajectory, geometry = inputs(5, 2)
    node._trajectory_callback(trajectory)
    node._geometry_callback(geometry)
    assert len(node.outputs) == 1
    assert 'missing' in node._waiting
    node._objects_callback(TrackedObjectArray(header=header(5)))
    node._state_callback(RoverState(header=header(5), acceleration_valid=True))
    assert len(node.outputs) == 2


def test_diagnostics_identify_the_waiting_cycle(node):
    messages = []
    node._diagnostics = SimpleNamespace(publish=messages.append)
    trajectory, _ = inputs(cycle=42)
    node._trajectory_callback(trajectory)
    node._publish_diagnostics()
    values = {v.key: v.value for v in messages[-1].status[0].values}
    assert values['trajectory_id'] == '42'


def test_diagnostics_allow_processing_latency_within_live_path_freshness(node):
    from diagnostic_msgs.msg import DiagnosticStatus
    from rclpy.time import Time as ClockTime
    node._node.get_clock().set_ros_time_override(ClockTime(seconds=10.7))
    trajectory, geometry = inputs()
    node._objects_callback(TrackedObjectArray(header=header(10)))
    node._state_callback(RoverState(header=header(10), acceleration_valid=True))
    node._geometry_callback(geometry)
    node._trajectory_callback(trajectory)
    assert node.outputs
    messages = []
    node._diagnostics = SimpleNamespace(publish=messages.append)
    node._publish_diagnostics()
    assert messages[-1].status[0].level == DiagnosticStatus.OK
    node._node.get_clock().set_ros_time_override(ClockTime(seconds=11.2))
    node._publish_diagnostics()
    assert messages[-1].status[0].level == DiagnosticStatus.WARN


def test_source_changes_during_physics_do_not_publish_old_result(node):
    import json
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from rclpy.time import Time as ClockTime
    from std_msgs.msg import String
    from lr_mavros_bridge.prediction_guard import PredictionGuard

    node._node.get_clock().set_ros_time_override(ClockTime(seconds=10))
    guard = PredictionGuard(node._node, node._on_time_jump)
    node._source_guard = guard
    def generation(value, start):
        guard.source.on_state(String(data=json.dumps({
            'runtime_id': 'runtime', 'generation': value, 'ready': True, 'start_ns': start})))
    generation(1, 1)
    guard.on_path(RosPath(header=header(10), poses=[PoseStamped(), PoseStamped()]))
    trajectory, geometry = inputs()
    node._objects_callback(TrackedObjectArray(header=header(10)))
    node._state_callback(RoverState(header=header(10), acceleration_valid=True))
    node._trajectory_callback(trajectory)
    compute = node.runtime.on_geometry
    def changed_during_computation(*args, **kwargs):
        result = compute(*args, **kwargs)
        generation(2, 10_000_000_001)
        return result
    node.runtime.on_geometry = changed_during_computation
    node._geometry_callback(geometry)
    assert not node.outputs
    guard.poll()
    assert node._active_trajectory is None and not node._geometries
    node._objects_callback(TrackedObjectArray(header=header(10)))
    assert not node.outputs
