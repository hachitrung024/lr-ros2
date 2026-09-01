"""Typed upstream publisher demonstrating static and dynamic PredictionRuntime profiles."""

from __future__ import annotations


def main() -> None:  # pragma: no cover - requires ROS-generated messages
    import rclpy
    from rclpy.node import Node
    from safety_perception_msgs.msg import (
        ExternalWrench,
        ExternalWrenchArray,
        GeometryArray,
        GeometryStep,
        RoverState,
        TrackedObjectArray,
        Trajectory,
        TrajectoryStep,
    )

    class MockUpstreamNode(Node):
        def __init__(self) -> None:
            super().__init__("mock_upstream_node")
            self.declare_parameter("demo_mode", "static")
            # static | dynamic | dynamic_zero | dynamic_wrench
            self.demo_mode = str(self.get_parameter("demo_mode").value).strip().lower()
            self.trajectory_pub = self.create_publisher(Trajectory, "/trajectory", 10)
            self.objects_pub = self.create_publisher(TrackedObjectArray, "/tracked_objects", 10)
            self.geometry_pub = self.create_publisher(GeometryArray, "/geometry", 10)
            self.state_pub = self.create_publisher(RoverState, "/rover/state", 10)
            self.wrench_pub = self.create_publisher(ExternalWrenchArray, "/external_wrenches", 10)
            self.timer = self.create_timer(0.2, self._tick)
            self.phase = 0

        def _tick(self) -> None:
            now = self.get_clock().now().to_msg()
            if self.phase == 0:
                self._publish_trajectory_objects_geometry(now)
                self.phase = 1
                self.get_logger().info(
                    f"[{self.demo_mode}] published trajectory + empty objects + geometry"
                )
                if self.demo_mode == "static":
                    self.phase = 99
                return

            if self.phase == 1 and self.demo_mode.startswith("dynamic"):
                accel = (0.0, 0.0, 0.0) if self.demo_mode == "dynamic_zero" else (0.0, 1.5, 0.0)
                self._publish_state(now, acceleration_xyz=accel, acceleration_valid=True)
                self.phase = 2
                self.get_logger().info(
                    f"[{self.demo_mode}] published RoverState accel={accel} valid=true"
                )
                if self.demo_mode != "dynamic_wrench":
                    self.phase = 99
                return

            if self.phase == 2 and self.demo_mode == "dynamic_wrench":
                self._publish_wrench(now)
                self.phase = 99
                self.get_logger().info("[dynamic_wrench] published ExternalWrenchArray")

        def _publish_trajectory_objects_geometry(self, now) -> None:
            trajectory = Trajectory()
            trajectory.header.stamp = now
            trajectory.header.frame_id = "map"
            trajectory.trajectory_id = 1
            for step_id in range(3):
                step = TrajectoryStep()
                step.step_id, step.x, step.y, step.yaw = step_id, float(step_id), 0.0, 0.0
                trajectory.steps.append(step)

            objects = TrackedObjectArray()
            objects.header.stamp, objects.header.frame_id = now, "map"

            geometry = GeometryArray()
            geometry.header.stamp, geometry.header.frame_id = now, "map"
            geometry.source_trajectory_id = 1
            geometry.source_trajectory_stamp = now
            for step_id in range(3):
                step = GeometryStep()
                step.step_id, step.plane_id = step_id, f"flat-{step_id}"
                step.normal.z = 1.0
                geometry.steps.append(step)

            self.trajectory_pub.publish(trajectory)
            self.objects_pub.publish(objects)
            self.geometry_pub.publish(geometry)

        def _publish_state(self, now, *, acceleration_xyz, acceleration_valid: bool) -> None:
            state = RoverState()
            state.header.stamp, state.header.frame_id = now, "map"
            state.pose_valid = False
            state.twist_valid = False
            state.acceleration_valid = acceleration_valid
            if acceleration_valid:
                state.acceleration.linear.x = float(acceleration_xyz[0])
                state.acceleration.linear.y = float(acceleration_xyz[1])
                state.acceleration.linear.z = float(acceleration_xyz[2])
            self.state_pub.publish(state)

        def _publish_wrench(self, now) -> None:
            array = ExternalWrenchArray()
            array.header.stamp, array.header.frame_id = now, "map"
            item = ExternalWrench()
            item.header.stamp, item.header.frame_id = now, "map"
            item.source = "boom"
            item.wrench.force.y = 200.0
            item.application_point_valid = True
            item.application_point.x = 0.0
            item.application_point.y = 0.0
            item.application_point.z = 0.8
            item.confidence_valid = True
            item.confidence = 1.0
            array.wrenches.append(item)
            self.wrench_pub.publish(array)

    rclpy.init()
    node = MockUpstreamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
