import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from grid_map_msgs.msg import GridMap
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

        assert spin_until(
            executor,
            lambda: maps
            and any(value == 1.0 for value in maps[-1].data[4].data),
        )
        assert spin_until(executor, lambda: bool(heatmaps))
        assert heatmaps[-1].encoding == "rgb8"
        assert heatmaps[-1].height == 264
        assert heatmaps[-1].width == 264

        reset_client = driver.create_client(
            Trigger,
            "/terrain_test/terrain_geometry_test/reset",
        )
        assert reset_client.wait_for_service(timeout_sec=2.0)
        future = reset_client.call_async(Trigger.Request())
        assert spin_until(executor, future.done)
        assert future.result().success
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
