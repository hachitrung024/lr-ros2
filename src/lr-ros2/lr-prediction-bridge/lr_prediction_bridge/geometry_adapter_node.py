#!/usr/bin/env python3
"""Build /geometry from /trajectory + terrain GridMap normal layers."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from safety_perception_msgs.msg import GeometryArray, GeometryStep, Trajectory


class GeometryAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("geometry_adapter_node")
        self.declare_parameter("trajectory_topic", "/trajectory")
        self.declare_parameter("grid_map_topic", "/terrain_geometry/grid_map")
        self.declare_parameter("output_topic", "/geometry")
        self.declare_parameter("expected_frame_id", "map")
        self.declare_parameter("force_frame_id_map", True)
        self.declare_parameter("allow_flat_fallback", False)
        self.declare_parameter("flat_fallback_confidence", 0.25)

        self._expected_frame = str(self.get_parameter("expected_frame_id").value)
        self._force_map = bool(self.get_parameter("force_frame_id_map").value)
        self._allow_flat = bool(self.get_parameter("allow_flat_fallback").value)
        self._flat_conf = float(self.get_parameter("flat_fallback_confidence").value)

        self._grid: Optional[GridMap] = None
        self._layers: dict[str, np.ndarray] = {}

        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        grid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(
            GeometryArray, self.get_parameter("output_topic").value, reliable
        )
        self.create_subscription(
            GridMap,
            self.get_parameter("grid_map_topic").value,
            self._on_grid,
            grid_qos,
        )
        self.create_subscription(
            Trajectory,
            self.get_parameter("trajectory_topic").value,
            self._on_trajectory,
            reliable,
        )
        self.get_logger().info(
            "geometry adapter: /trajectory + GridMap normals → /geometry "
            f"(allow_flat_fallback={self._allow_flat})"
        )

    def _on_grid(self, msg: GridMap) -> None:
        self._grid = msg
        self._layers = {}
        if not msg.data or len(msg.data[0].layout.dim) < 2:
            return
        columns = int(msg.data[0].layout.dim[0].size)
        rows = int(msg.data[0].layout.dim[1].size)
        for name, data in zip(msg.layers, msg.data):
            arr = np.asarray(data.data, dtype=np.float32)
            if arr.size != rows * columns:
                continue
            self._layers[name] = arr.reshape((rows, columns), order="F")

    def _on_trajectory(self, traj: Trajectory) -> None:
        if not traj.steps:
            self.get_logger().warn("ignoring empty trajectory")
            return

        frame_id = traj.header.frame_id.strip() or self._expected_frame
        if self._force_map:
            frame_id = "map"

        out = GeometryArray()
        out.header.stamp = traj.header.stamp
        out.header.frame_id = frame_id
        out.source_trajectory_id = int(traj.trajectory_id)
        out.source_trajectory_stamp = traj.header.stamp
        steps: list[GeometryStep] = []

        missing = 0
        for step in traj.steps:
            normal = self._lookup_normal(step.x, step.y)
            geom = GeometryStep()
            geom.step_id = int(step.step_id)
            if normal is None:
                missing += 1
                if not self._allow_flat:
                    # Partial terrain coverage is normal for a forward camera.
                    # PredictionCore accepts partial geometry when configured
                    # with require_full_geometry_coverage=false.
                    continue
                geom.plane_id = f"flat-fallback-{step.step_id}"
                geom.normal.x = 0.0
                geom.normal.y = 0.0
                geom.normal.z = 1.0
                geom.confidence = self._flat_conf
                geom.confidence_valid = True
            else:
                nx, ny, nz, conf, conf_valid, plane_id = normal
                geom.plane_id = plane_id
                geom.normal.x = nx
                geom.normal.y = ny
                geom.normal.z = nz
                geom.confidence = conf
                geom.confidence_valid = conf_valid
            steps.append(geom)

        out.steps = steps
        if missing:
            action = "used flat fallback" if self._allow_flat else "omitted"
            self.get_logger().debug(
                f"geometry {action} {missing}/{len(traj.steps)} steps"
            )
        if not steps:
            self.get_logger().debug(
                "no valid terrain normals overlap the current trajectory"
            )
            return
        self._pub.publish(out)

    def _lookup_normal(
        self, x: float, y: float
    ) -> Optional[tuple[float, float, float, float, bool, str]]:
        if self._grid is None:
            return None
        for layer in ("normal_x", "normal_y", "normal_z"):
            if layer not in self._layers:
                return None

        info = self._grid.info
        resolution = float(info.resolution)
        if resolution <= 0.0:
            return None
        rows, cols = self._layers["normal_x"].shape
        pose = info.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        dx = float(x) - float(pose.position.x)
        dy = float(y) - float(pose.position.y)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        logical_row = int(
            math.floor((rows * resolution * 0.5 - local_x) / resolution)
        )
        logical_col = int(
            math.floor((cols * resolution * 0.5 - local_y) / resolution)
        )
        if not (0 <= logical_row < rows and 0 <= logical_col < cols):
            return None
        row = (logical_row + int(self._grid.outer_start_index)) % rows
        col = (logical_col + int(self._grid.inner_start_index)) % cols

        nx = float(self._layers["normal_x"][row, col])
        ny = float(self._layers["normal_y"][row, col])
        nz = float(self._layers["normal_z"][row, col])
        if not all(math.isfinite(v) for v in (nx, ny, nz)):
            return None
        norm = math.sqrt(nx * nx + ny * ny + nz * nz)
        if norm <= 1e-12:
            return None
        nx, ny, nz = nx / norm, ny / norm, nz / norm
        if nz < 0.0:
            nx, ny, nz = -nx, -ny, -nz

        conf = 1.0
        conf_valid = True
        if "inlier_ratio" in self._layers:
            ratio = float(self._layers["inlier_ratio"][row, col])
            if math.isfinite(ratio):
                conf = max(0.0, min(1.0, ratio))
        if "state" in self._layers:
            state = float(self._layers["state"][row, col])
            if math.isfinite(state) and state < 0.5:
                conf_valid = False

        return nx, ny, nz, conf, conf_valid, f"terrain-r{row}-c{col}"


def main(argv: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=argv)
    node = GeometryAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
