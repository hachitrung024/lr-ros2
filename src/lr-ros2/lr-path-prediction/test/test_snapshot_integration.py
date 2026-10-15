"""Exercise bridge-to-display snapshot transport over real ROS subscriptions."""

import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from safety_perception_msgs.msg import (
    GeometryArray, GeometryStep, PredictionOutput, TrajectoryPath,
)
from visualization_msgs.msg import MarkerArray

from lr_prediction_bridge.prediction_bridge_node import PredictionBridgeNode
from lr_path_prediction.canonical_visualizer_node import CanonicalPredictionVisualizerNode


def test_source_mapping_and_terrain_height_do_not_change_displayed_xyz():
    """Dense source bends/Z survive DDS while risk points retain exact source indices."""
    rclpy.init()
    bridge = PredictionBridgeNode(role='trajectory', parameter_overrides=[
        Parameter('input_topic', value='/test/alignment/source'),
        Parameter('spatial_step_m', value=.2),
    ])
    visualizer = CanonicalPredictionVisualizerNode(parameter_overrides=[
        Parameter('visualization.max_prediction_age_sec', value=10.),
        Parameter('visualization.max_path_age_sec', value=10.),
    ])
    driver = rclpy.create_node('alignment_test_driver')
    executor = SingleThreadedExecutor()
    for node in (bridge, visualizer, driver):
        executor.add_node(node)
    snapshots, paths, markers = [], [], []
    driver.create_subscription(TrajectoryPath, '/trajectory/path', snapshots.append, 10)
    driver.create_subscription(Path, '/lr/path_prediction/reference_path', paths.append, 10)
    driver.create_subscription(MarkerArray, '/lr/path_prediction/markers', markers.append, 10)
    source = driver.create_publisher(Path, '/test/alignment/source', 10)
    geometry_pub = driver.create_publisher(GeometryArray, '/geometry', 10)
    prediction_pub = driver.create_publisher(PredictionOutput, '/predict_output', 10)

    def wait_until(condition):
        deadline = time.monotonic() + 4.
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if condition():
                return True
        return False

    try:
        assert wait_until(lambda: source.get_subscription_count() > 0
                          and bridge._path_snapshot_pub.get_subscription_count() >= 2)
        path = Path()
        path.header.stamp = driver.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for i in range(40):
            pose = PoseStamped(header=path.header)
            pose.pose.position.x = i * .2
            pose.pose.position.y = math.sin(i * .15)
            pose.pose.position.z = 2. + i * .01
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        source.publish(path)
        assert wait_until(lambda: snapshots and paths and markers)
        snapshot = snapshots[-1]
        geometry = GeometryArray(
            header=path.header, source_trajectory_id=snapshot.trajectory_id,
            source_trajectory_stamp=path.header.stamp,
        )
        # Alternate known and unknown terrain. The unknown steps must stay visible.
        for step in list(snapshot.step_ids)[::2]:
            cell = GeometryStep(step_id=step, elevation_m=-3., elevation_valid=True)
            cell.normal.z = 1.
            geometry.steps.append(cell)
        geometry_pub.publish(geometry)
        prediction_pub.publish(PredictionOutput(
            header=path.header, source_trajectory_id=snapshot.trajectory_id,
        ))
        assert wait_until(lambda: any(m.ns == 'prediction_labels' for m in markers[-1].markers))
        assert paths[-1].poses == path.poses
        points = next(m for m in markers[-1].markers if m.ns == 'prediction_steps')
        assert points.header == paths[-1].header
        assert points.points == [path.poses[i].pose.position for i in snapshot.source_pose_indices]
        assert len(points.points) == 20
        assert not any(m.ns == 'prediction_path' for m in markers[-1].markers)
    finally:
        executor.shutdown()
        for node in (bridge, visualizer, driver):
            node.destroy_node()
        rclpy.shutdown()
