"""Contract checks for the consolidated bridge and its compatibility nodes."""

import math
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped, TransformStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.parameter import Parameter
from safety_perception_msgs.msg import (
    Point2D,
    TrackedObject,
    TrackedObjectArray,
    Trajectory,
    TrajectoryStep,
)
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

from lr_prediction_bridge.conversions import (
    StateConverter,
    TrajectoryConverter,
    geometry_from_trajectory,
)
from lr_prediction_bridge.prediction_bridge_node import PredictionBridgeNode
from lr_prediction_bridge.geometry_adapter_node import GeometryAdapterNode
from lr_prediction_bridge.trajectory_adapter_node import TrajectoryAdapterNode
from lr_prediction_bridge.rover_state_adapter_node import RoverStateAdapterNode
from lr_prediction_bridge.tracked_objects_adapter_node import TrackedObjectsAdapterNode
from lr_terrain_geometry.grid_map_sampling import GridMapSampler


def grid(stamp=9, state=1.0):
    msg = GridMap()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = stamp
    msg.info.resolution = 1.0
    msg.info.length_x = msg.info.length_y = 1.0
    msg.info.pose.orientation.w = 1.0
    msg.layers = ['elevation', 'slope_deg', 'normal_x', 'normal_y', 'normal_z', 'state']
    for value in (0.4, 25.0, 0.0, math.sin(math.radians(25)), math.cos(math.radians(25)), state):
        layer = Float32MultiArray(data=[value])
        layer.layout.dim = [
            MultiArrayDimension(size=1, stride=1),
            MultiArrayDimension(size=1, stride=1),
        ]
        msg.data.append(layer)
    return msg


def pose(second, x=0.0, frame='map'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp.sec = second
    msg.pose.orientation.w = 1.0
    msg.pose.position.x = float(x)
    return msg


def path(second=10):
    msg = Path(header=pose(second).header)
    msg.poses = [pose(second + i, i) for i in range(22)]
    return msg


def test_sampling_throttle_and_monotonic_ids_after_seek():
    converter = TrajectoryConverter(output_dt_sec=0.0)
    out = converter.convert(path())
    assert len(out.steps) == 20
    assert out.steps[0].x == 1.0
    assert converter.convert(path()) is None
    converter.reset()
    assert converter.convert(path(5)).trajectory_id == out.trajectory_id + 1
    bad = path()
    bad.header.frame_id = 'odom'
    with pytest.raises(ValueError, match='frame'):
        converter.convert(bad)


def test_state_acceleration_missing_zero_gaps_and_rewind():
    converter = StateConverter()
    first = converter.convert(pose(1, 0.5))
    assert not first.acceleration_valid
    converter.convert(pose(2, 2.0))
    out = converter.convert(pose(3, 4.5))
    assert out.acceleration_valid
    assert out.acceleration.linear.x == pytest.approx(1.0)
    assert not converter.convert(pose(6, 5.0)).acceleration_valid
    assert not converter.convert(pose(1, 0.5)).acceleration_valid
    with pytest.raises(ValueError, match='frame'):
        converter.convert(pose(2, frame='odom'))


def test_geometry_preserves_sensor_stamp_and_unknown_cells():
    terrain = grid()
    traj = Trajectory(
        header=pose(10).header,
        trajectory_id=42,
        steps=[TrajectoryStep(step_id=0, x=0.0, y=0.0), TrajectoryStep(step_id=1, x=5.0, y=0.0)],
    )
    out = geometry_from_trajectory(traj, GridMapSampler(terrain), terrain.header)
    assert out.header.stamp.sec == 9
    assert out.source_trajectory_stamp.sec == 10
    assert out.source_trajectory_id == 42
    assert len(out.steps) == 1
    assert out.steps[0].elevation_valid
    assert out.steps[0].elevation_m == pytest.approx(0.4)
    terrain = grid(state=0.0)
    assert not geometry_from_trajectory(traj, GridMapSampler(terrain), terrain.header).steps


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.parametrize('profile', ['static', 'dynamic'])
def test_one_node_roles_and_geometry_use_past_grid(ros, profile):
    node = PredictionBridgeNode(
        parameter_overrides=[Parameter('prediction_profile', value=profile)]
    )
    try:
        subscriptions = {s.topic_name for s in node.subscriptions}
        assert '/trajectory' not in subscriptions
        assert ('/lr/mavlink/pose' in subscriptions) == (profile == 'dynamic')
        node._on_grid(grid(9))
        node._on_grid(grid(11))
        collected = []
        node._geometry_pub = SimpleNamespace(publish=collected.append)
        traj = Trajectory(
            header=pose(10).header,
            trajectory_id=42,
            steps=[TrajectoryStep(step_id=0, x=0.0, y=0.0)],
        )
        node._on_trajectory(traj)
        assert collected[-1].header.stamp.sec == 9
        bad = grid(10)
        bad.header.frame_id = 'odom'
        node._on_grid(bad)
        assert len(node._grids) == 2
        node._on_time_jump(None)
        assert not node._grids
        assert node._active_trajectory is None
        assert node._trajectory_converter.trajectory_id == 0
    finally:
        node.destroy_node()


@pytest.mark.parametrize(
    'kind',
    [TrajectoryAdapterNode, GeometryAdapterNode, RoverStateAdapterNode, TrackedObjectsAdapterNode],
)
def test_compatibility_executables_share_bridge_implementation(ros, kind):
    node = kind()
    try:
        assert isinstance(node, PredictionBridgeNode)
        assert len(node._roles) == 1
    finally:
        node.destroy_node()


def detections(frame='camera'):
    message = Detection3DArray(header=pose(10, frame=frame).header)
    box = Detection3D(id='42')
    box.bbox.center.orientation.w = 1.0
    box.bbox.size.x = box.bbox.size.y = box.bbox.size.z = 2.0
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = 'rock'
    hypothesis.hypothesis.score = 0.9
    box.results = [hypothesis]
    message.detections = [box]
    return message


def tracked_objects(frame='map'):
    message = TrackedObjectArray(header=pose(10, frame=frame).header)
    tracked_object = TrackedObject(track_id=42, class_name='rock')
    tracked_object.footprint_polygon_xy = [
        Point2D(x=-1.0, y=-1.0),
        Point2D(x=1.0, y=-1.0),
        Point2D(x=1.0, y=1.0),
        Point2D(x=-1.0, y=1.0),
    ]
    message.objects = [tracked_object]
    return message


def test_canonical_polygon_input_is_validated_and_republished(ros):
    node = TrackedObjectsAdapterNode(
        parameter_overrides=[Parameter('input_type', value='tracked_objects')]
    )
    published = []
    node._objects_pub = SimpleNamespace(publish=published.append)
    try:
        message = tracked_objects()
        node._on_tracked_objects(message)
        assert published == [message]
        assert node._tf_buffer is None

        invalid = tracked_objects()
        invalid.objects[0].footprint_polygon_xy = [Point2D(x=0.0, y=0.0)] * 3
        node._on_tracked_objects(invalid)
        assert published == [message]

        self_intersecting = tracked_objects()
        self_intersecting.objects[0].footprint_polygon_xy = [
            Point2D(x=0.0, y=0.0),
            Point2D(x=3.0, y=3.0),
            Point2D(x=0.0, y=2.0),
            Point2D(x=2.0, y=0.0),
        ]
        node._on_tracked_objects(self_intersecting)
        assert published == [message]

        wrong_frame = tracked_objects(frame='odom')
        node._on_tracked_objects(wrong_frame)
        assert published == [message]
    finally:
        node.destroy_node()


def test_missing_tf_retries_without_emitting_empty_then_preserves_track(ros, monkeypatch):
    node = TrackedObjectsAdapterNode()
    published = []
    node._objects_pub = SimpleNamespace(publish=published.append)
    monkeypatch.setattr('lr_prediction_bridge.prediction_bridge_node.time.monotonic', lambda: 1.0)
    try:
        node._on_detections(detections())
        assert len(node._pending_detections) == 1
        assert not published
        transform = TransformStamped(header=pose(10).header, child_frame_id='camera')
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = 3.0
        node._tf_buffer.set_transform(transform, 'test')
        node._retry_detections()
        assert not node._pending_detections
        assert len(published) == 1
        assert published[0].header.frame_id == 'map'
        assert published[0].header.stamp.sec == 10
        obj = published[0].objects[0]
        assert obj.track_id == 42
        assert not obj.velocity_valid
        assert obj.footprint_polygon_xy[0].x == pytest.approx(2.0)
    finally:
        node.destroy_node()


def test_tf_timeout_malformed_batch_and_seek_do_not_invent_empty_observations(ros, monkeypatch):
    node = TrackedObjectsAdapterNode()
    published = []
    node._objects_pub = SimpleNamespace(publish=published.append)
    now = [1.0]
    monkeypatch.setattr(
        'lr_prediction_bridge.prediction_bridge_node.time.monotonic', lambda: now[0]
    )
    try:
        node._on_detections(detections())
        now[0] += 1.0
        node._retry_detections()
        assert not node._pending_detections
        invalid = detections('map')
        invalid.detections[0].bbox.size.x = float('nan')
        node._on_detections(invalid)
        assert not published
        node._on_detections(detections())
        node._on_time_jump(None)
        assert not node._pending_detections
        empty = Detection3DArray(header=pose(10).header)
        node._on_detections(empty)
        assert len(published) == 1
        assert not published[0].objects
        assert published[0].header.stamp.sec == 10
    finally:
        node.destroy_node()
