import time
from threading import Event, Thread

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from lr_terrain_geometry.node import TerrainGeometryNode


def spin_until(executor, predicate, timeout=4.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


@pytest.fixture
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_node_processes_cloud_with_tf_and_reset_service(ros_context):
    overrides = [
        Parameter("input.point_cloud_topic", value="/test/cloud"),
        Parameter("input.trajectory_topic", value="/test/terrain_path"),
        Parameter("corridor.stale_after_sec", value=10.0),
        Parameter("terrain.radius_m", value=5.0),
        Parameter("terrain.ttl_seconds", value=0.0),
        Parameter("terrain.min_distance", value=0.0),
        Parameter("terrain.max_distance", value=100.0),
        Parameter("terrain.min_forward_m", value=0.0),
        Parameter("terrain.max_forward_m", value=100.0),
        Parameter("terrain.min_lateral_m", value=-100.0),
        Parameter("terrain.max_lateral_m", value=100.0),
        Parameter("terrain.min_cell_points", value=20),
        Parameter("terrain.min_cell_inliers", value=10),
        Parameter("terrain.min_cell_principal_stddev", value=0.05),
        Parameter("terrain.min_accumulation_frames", value=1),
        Parameter("terrain.fit_every_frames", value=1),
        Parameter("terrain.confirmation_good_fits", value=1),
        Parameter("terrain.ransac_iterations", value=60),
        Parameter("terrain.voxel_size_m", value=0.01),
    ]
    terrain_node = TerrainGeometryNode(
        parameter_overrides=overrides,
        node_name="terrain_geometry_test",
        namespace="/terrain_test",
    )
    driver = rclpy.create_node("terrain_geometry_test_driver")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(terrain_node)
    executor.add_node(driver)

    broadcaster = StaticTransformBroadcaster(driver)
    transform = TransformStamped()
    transform.header.stamp = driver.get_clock().now().to_msg()
    transform.header.frame_id = "map"
    transform.child_frame_id = "camera"
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)

    output_qos = QoSProfile(depth=1)
    output_qos.reliability = ReliabilityPolicy.RELIABLE
    output_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    maps = []
    heatmaps = []
    driver.create_subscription(
        GridMap,
        "/terrain_test/terrain_geometry_test/grid_map",
        maps.append,
        output_qos,
    )
    driver.create_subscription(
        Image,
        "/terrain_test/terrain_geometry_test/heatmap",
        heatmaps.append,
        output_qos,
    )
    cloud_publisher = driver.create_publisher(
        type(point_cloud2.create_cloud_xyz32(Header(), [])),
        "/test/cloud",
        10,
    )
    path_publisher = driver.create_publisher(Path, "/test/terrain_path", 10)

    try:
        assert spin_until(
            executor,
            lambda: cloud_publisher.get_subscription_count() > 0,
        )
        x = np.linspace(0.05, 0.95, 10)
        y = np.linspace(-0.95, -0.05, 10)
        xx, yy = np.meshgrid(x, y)
        points = np.column_stack((
            xx.reshape(-1),
            yy.reshape(-1),
            np.zeros(xx.size),
        ))
        header = Header()
        header.stamp = driver.get_clock().now().to_msg()
        header.frame_id = "camera"
        cloud_publisher.publish(
            point_cloud2.create_cloud_xyz32(header, points.tolist())
        )

        assert spin_until(executor, lambda: maps and terrain_node._processed_count > 0)
        assert all(np.isnan(value) for value in maps[-1].data[4].data)
        assert terrain_node._estimator.cell_count == 0
        assert spin_until(executor, lambda: path_publisher.get_subscription_count() > 0)
        initial_path = path_message(0, [(0., -.5), (4., -.5)])
        initial_path.header.stamp = driver.get_clock().now().to_msg()
        path_publisher.publish(initial_path)
        assert spin_until(executor, lambda: len(terrain_node._paths) == 1)
        header.stamp = driver.get_clock().now().to_msg()
        cloud_publisher.publish(point_cloud2.create_cloud_xyz32(header, points.tolist()))

        assert spin_until(
            executor,
            lambda: maps
            and any(value == 1.0 for value in maps[-1].data[4].data),
        )
        assert spin_until(executor, lambda: bool(heatmaps))
        assert heatmaps[-1].encoding == "rgb8"
        assert heatmaps[-1].height == 264
        assert heatmaps[-1].width == 264

        assert spin_until(executor, lambda: path_publisher.get_subscription_count() > 0)
        path = path_message(0, [(0., 3.), (4., 3.)])
        path.header.stamp = driver.get_clock().now().to_msg()
        path_publisher.publish(path)
        assert spin_until(executor, lambda: len(terrain_node._paths) == 2)
        header.stamp = driver.get_clock().now().to_msg()
        cloud_publisher.publish(point_cloud2.create_cloud_xyz32(header, points.tolist()))
        assert spin_until(
            executor,
            lambda: terrain_node._corridor_status == "active"
            and all(np.isnan(value) for value in maps[-1].data[4].data),
        )
        assert terrain_node._estimator.last_corridor_points == 0

        # An empty path must hide cached terrain and stop ingesting new points.
        path.poses = []
        path.header.stamp = driver.get_clock().now().to_msg()
        path_publisher.publish(path)
        assert spin_until(executor, lambda: len(terrain_node._paths) == 3)
        processed = terrain_node._processed_count
        header.stamp = driver.get_clock().now().to_msg()
        cloud_publisher.publish(point_cloud2.create_cloud_xyz32(header, points.tolist()))
        assert spin_until(executor, lambda: terrain_node._processed_count > processed)
        assert all(np.isnan(value) for value in maps[-1].data[4].data)
        assert terrain_node._estimator.last_corridor_points == 0

        initial_path.header.stamp = driver.get_clock().now().to_msg()
        path_publisher.publish(initial_path)
        assert spin_until(executor, lambda: len(terrain_node._paths) == 4)
        header.stamp = driver.get_clock().now().to_msg()
        cloud_publisher.publish(point_cloud2.create_cloud_xyz32(header, points.tolist()))
        assert spin_until(executor, lambda: any(value == 1.0 for value in maps[-1].data[4].data))
        assert terrain_node._estimator.last_corridor_points == len(points)

        reset_client = driver.create_client(
            Trigger,
            "/terrain_test/terrain_geometry_test/reset",
        )
        assert reset_client.wait_for_service(timeout_sec=2.0)
        future = reset_client.call_async(Trigger.Request())
        assert spin_until(executor, future.done)
        assert future.result().success
        assert not terrain_node._paths
        assert spin_until(
            executor,
            lambda: maps
            and all(np.isnan(value) for value in maps[-1].data[4].data),
        )
    finally:
        executor.remove_node(terrain_node)
        executor.remove_node(driver)
        terrain_node.destroy_node()
        driver.destroy_node()
        executor.shutdown()


def path_message(stamp, points, frame="map"):
    message = Path(header=Header(frame_id=frame))
    message.header.stamp.sec = stamp
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        message.poses.append(pose)
    return message


def test_corridor_freshness_buffer_and_rewind(ros_context):
    node = TerrainGeometryNode()
    try:
        assert node._corridor_for_cloud(10.) is None
        node._on_path(path_message(10, [(0, 0), (4, 0)]))
        first = node._corridor_for_cloud(10.)
        assert first is not None
        node._on_path(path_message(12, [(0, 0), (0, 4)]))
        assert node._corridor_for_cloud(10.5) is first  # Late cloud uses older buffered path.
        assert node._corridor_for_cloud(11.5) is None  # Older path stale; new path too far ahead.
        assert node._corridor_status == "stale trajectory"
        assert node._corridor_for_cloud(11.9) is not None  # Within future tolerance.
        assert node._corridor_for_cloud(12.) is not None
        assert node._corridor_for_cloud(12.) is not None  # A paused clock does not age the path.
        assert node._corridor_for_cloud(1.) is None
        assert not node._paths
        node._on_path(path_message(1, [(0, 0)]))
        assert node._corridor_for_cloud(1.) is not None
        node._on_time_jump(None)
        assert not node._paths
    finally:
        node.destroy_node()


@pytest.mark.parametrize("case", ["empty", "frame", "mixed", "nan", "oversized"])
def test_invalid_path_replaces_previous_corridor(ros_context, case):
    node = TerrainGeometryNode()
    try:
        node._on_path(path_message(10, [(0, 0), (4, 0)]))
        assert node._corridor_for_cloud(10.) is not None
        message = path_message(10, [(0, 0)])
        if case == "empty":
            message.poses = []
        elif case == "frame":
            message.header.frame_id = "odom"
        elif case == "mixed":
            message.poses[0].header.frame_id = "odom"
        elif case == "nan":
            message.poses[0].pose.position.z = float("nan")
        else:
            message.poses = [message.poses[0]] * 257
        node._on_path(message)
        assert node._corridor_for_cloud(10.) is None
        assert node._corridor_status != "active"
    finally:
        node.destroy_node()


def test_disabled_corridor_has_no_subscription(ros_context):
    node = TerrainGeometryNode(parameter_overrides=[Parameter("corridor.enabled", value=False)])
    try:
        assert node._path_subscription is None
        assert node._corridor_for_cloud(1.) is None
        assert node._corridor_status == "disabled"
    finally:
        node.destroy_node()


def test_path_subscription_progresses_while_cloud_processing_holds_state(ros_context):
    """Slow fitting/rendering cannot starve path input and create false stale corridors."""
    node = TerrainGeometryNode(parameter_overrides=[
        Parameter('input.point_cloud_topic', value='/test/busy_cloud'),
        Parameter('input.trajectory_topic', value='/test/busy_path'),
    ])
    driver = rclpy.create_node('busy_terrain_driver')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.add_node(driver)
    entered, release = Event(), Event()

    def process(*_args):
        entered.set()
        release.wait(3.)

    node._process_point_cloud = process
    cloud = point_cloud2.create_cloud_xyz32(Header(frame_id='camera'), [])
    clouds = driver.create_publisher(type(cloud), '/test/busy_cloud', 10)
    paths = driver.create_publisher(Path, '/test/busy_path', 10)
    worker = None
    try:
        assert spin_until(executor, lambda: clouds.get_subscription_count() > 0
                          and paths.get_subscription_count() > 0)
        worker = Thread(target=executor.spin)
        worker.start()
        clouds.publish(cloud)
        assert entered.wait(2.)
        paths.publish(path_message(10, [(0., 0.), (4., 0.)]))
        deadline = time.monotonic() + 1.
        while not node._paths and time.monotonic() < deadline:
            time.sleep(.01)
        assert node._paths, 'Path input was blocked behind the heavy cloud callback'
        assert not release.is_set()
    finally:
        release.set()
        executor.shutdown()
        if worker is not None:
            worker.join(timeout=3.)
        node.destroy_node()
        driver.destroy_node()
