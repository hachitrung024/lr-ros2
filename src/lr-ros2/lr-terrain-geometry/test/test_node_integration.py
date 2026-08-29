import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from grid_map_msgs.msg import GridMap
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

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
    terrain_node = TerrainGeometryNode(parameter_overrides=overrides)
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
        "/terrain_geometry/grid_map",
        maps.append,
        output_qos,
    )
    driver.create_subscription(
        Image,
        "/terrain_geometry/heatmap",
        heatmaps.append,
        output_qos,
    )
    cloud_publisher = driver.create_publisher(
        type(point_cloud2.create_cloud_xyz32(Header(), [])),
        "/test/cloud",
        10,
    )

    try:
        assert spin_until(executor, lambda: cloud_publisher.get_subscription_count() > 0)
        x = np.linspace(0.05, 0.95, 10)
        y = np.linspace(-0.95, -0.05, 10)
        xx, yy = np.meshgrid(x, y)
        points = np.column_stack((xx.reshape(-1), yy.reshape(-1), np.zeros(xx.size)))
        header = Header()
        header.stamp = driver.get_clock().now().to_msg()
        header.frame_id = "camera"
        cloud_publisher.publish(
            point_cloud2.create_cloud_xyz32(header, points.tolist())
        )

        assert spin_until(
            executor,
            lambda: maps and any(value == 1.0 for value in maps[-1].data[4].data),
        )
        assert spin_until(executor, lambda: bool(heatmaps))
        assert heatmaps[-1].encoding == "rgb8"
        assert heatmaps[-1].height == 264
        assert heatmaps[-1].width == 264

        reset_client = driver.create_client(Trigger, "/terrain_geometry/reset")
        assert reset_client.wait_for_service(timeout_sec=2.0)
        future = reset_client.call_async(Trigger.Request())
        assert spin_until(executor, future.done)
        assert future.result().success
        assert spin_until(
            executor,
            lambda: maps and all(
                np.isnan(value) for value in maps[-1].data[4].data
            ),
        )
    finally:
        executor.remove_node(terrain_node)
        executor.remove_node(driver)
        terrain_node.destroy_node()
        driver.destroy_node()
        executor.shutdown()


def test_node_synchronizes_detection_and_removes_object_points(ros_context):
    overrides = [
        Parameter("input.point_cloud_topic", value="/filter_test/cloud"),
        Parameter("object_filter.enabled", value=True),
        Parameter(
            "object_filter.detections_topic",
            value="/filter_test/detections",
        ),
        Parameter(
            "object_filter.instance_masks_topic",
            value="/filter_test/instance_masks",
        ),
        Parameter(
            "object_filter.camera_info_topic",
            value="/filter_test/camera_info",
        ),
        Parameter("object_filter.minimum_cluster_points", value=10),
        Parameter("object_filter.depth_tolerance_m", value=0.15),
        Parameter("terrain.enabled", value=False),
    ]
    terrain_node = TerrainGeometryNode(parameter_overrides=overrides)
    driver = rclpy.create_node("terrain_object_filter_test_driver")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(terrain_node)
    executor.add_node(driver)

    broadcaster = StaticTransformBroadcaster(driver)
    map_to_cloud = TransformStamped()
    map_to_cloud.header.stamp = driver.get_clock().now().to_msg()
    map_to_cloud.header.frame_id = "map"
    map_to_cloud.child_frame_id = "cloud_frame"
    map_to_cloud.transform.rotation.w = 1.0
    cloud_to_image = TransformStamped()
    cloud_to_image.header.stamp = map_to_cloud.header.stamp
    cloud_to_image.header.frame_id = "cloud_frame"
    cloud_to_image.child_frame_id = "image_optical_frame"
    cloud_to_image.transform.rotation.w = 1.0
    broadcaster.sendTransform([map_to_cloud, cloud_to_image])

    sensor_qos = QoSProfile(depth=1)
    sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    sensor_qos.durability = DurabilityPolicy.VOLATILE
    boxes = []
    object_markers = []
    driver.create_subscription(
        Detection3DArray,
        "/terrain_geometry/object_boxes_3d",
        boxes.append,
        sensor_qos,
    )
    driver.create_subscription(
        MarkerArray,
        "/terrain_geometry/object_box_markers",
        object_markers.append,
        sensor_qos,
    )
    cloud_publisher = driver.create_publisher(
        type(point_cloud2.create_cloud_xyz32(Header(), [])),
        "/filter_test/cloud",
        sensor_qos,
    )
    detection_publisher = driver.create_publisher(
        Detection2DArray,
        "/filter_test/detections",
        sensor_qos,
    )
    instance_mask_publisher = driver.create_publisher(
        Image,
        "/filter_test/instance_masks",
        sensor_qos,
    )
    camera_info_publisher = driver.create_publisher(
        CameraInfo,
        "/filter_test/camera_info",
        sensor_qos,
    )

    try:
        assert spin_until(
            executor,
            lambda: (
                cloud_publisher.get_subscription_count() > 0
                and detection_publisher.get_subscription_count() > 0
                and instance_mask_publisher.get_subscription_count() > 0
                and camera_info_publisher.get_subscription_count() > 0
            ),
        )

        camera_info = CameraInfo()
        camera_info.header.frame_id = "image_optical_frame"
        camera_info.width = 100
        camera_info.height = 100
        camera_info.p = [
            100.0, 0.0, 50.0, 0.0,
            0.0, 100.0, 50.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        camera_info_publisher.publish(camera_info)

        image_coordinates = np.linspace(42.0, 58.0, 6)
        uu, vv = np.meshgrid(image_coordinates, image_coordinates)
        object_points = np.column_stack((
            (uu.reshape(-1) - 50.0) * 3.0 / 100.0,
            (vv.reshape(-1) - 50.0) * 3.0 / 100.0,
            np.full(uu.size, 3.0),
        ))
        background_points = np.column_stack((
            (uu.reshape(-1) - 50.0) * 8.0 / 100.0,
            (vv.reshape(-1) - 50.0) * 8.0 / 100.0,
            np.full(uu.size, 8.0),
        ))
        stamp = driver.get_clock().now().to_msg()
        cloud_header = Header(stamp=stamp, frame_id="cloud_frame")
        cloud_publisher.publish(point_cloud2.create_cloud_xyz32(
            cloud_header,
            np.vstack((object_points, background_points)).tolist(),
        ))

        detection = Detection2D()
        detection.id = "test:0"
        detection.header = Header(
            stamp=stamp,
            frame_id="image_optical_frame",
        )
        detection.bbox.center.position.x = 50.0
        detection.bbox.center.position.y = 50.0
        detection.bbox.size_x = 30.0
        detection.bbox.size_y = 30.0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "waste"
        hypothesis.hypothesis.score = 0.9
        detection.results = [hypothesis]
        detection_array = Detection2DArray()
        detection_array.header = detection.header
        detection_array.detections = [detection]
        detection_publisher.publish(detection_array)

        instance_labels = np.zeros((100, 100), dtype="<u2")
        instance_labels[35:66, 35:66] = 1
        instance_mask = Image()
        instance_mask.header = detection.header
        instance_mask.height = 100
        instance_mask.width = 100
        instance_mask.encoding = "mono16"
        instance_mask.step = 200
        instance_mask.data = instance_labels.tobytes()
        instance_mask_publisher.publish(instance_mask)

        assert spin_until(
            executor,
            lambda: (
                terrain_node._processed_count == 1
                and bool(boxes)
                and bool(object_markers)
            ),
        )
        assert terrain_node._synchronized_pair_count == 1
        assert terrain_node._instance_mask_count == 1
        assert terrain_node._last_removed_point_count == object_points.shape[0]
        assert len(boxes[-1].detections) == 1
        assert boxes[-1].header.frame_id == "cloud_frame"
        assert boxes[-1].detections[0].id == "test:0"
        assert len(object_markers[-1].markers) == 3
        assert object_markers[-1].markers[1].ns == "object_boxes"
    finally:
        executor.remove_node(terrain_node)
        executor.remove_node(driver)
        terrain_node.destroy_node()
        driver.destroy_node()
        executor.shutdown()
