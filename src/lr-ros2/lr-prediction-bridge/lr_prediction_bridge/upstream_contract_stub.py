#!/usr/bin/env python3
"""Dev stub that mimics mavlink Path + pose topics for adapter smoke tests.

Does NOT invent tracked objects. Publishes:
  /lr/mavlink/trajectory_future (nav_msgs/Path)
  /lr/mavlink/position_enu
  /lr/mavlink/attitude_enu
and a minimal GridMap with upright normals for geometry adapter.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PointStamped, PoseStamped, QuaternionStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class UpstreamContractStub(Node):
    def __init__(self) -> None:
        super().__init__("upstream_contract_stub")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("path_frames", 60)
        self.declare_parameter("svo_fps", 15.0)
        self.declare_parameter("step_m", 0.25)

        qos = QoSProfile(
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
        self._path_pub = self.create_publisher(Path, "/lr/mavlink/trajectory_future", qos)
        self._pos_pub = self.create_publisher(PointStamped, "/lr/mavlink/position_enu", qos)
        self._att_pub = self.create_publisher(
            QuaternionStamped, "/lr/mavlink/attitude_enu", qos
        )
        self._grid_pub = self.create_publisher(
            GridMap, "/terrain_geometry/grid_map", grid_qos
        )

        self._tick = 0
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._on_timer)
        self.get_logger().warn(
            "upstream_contract_stub running — synthetic Path/pose/GridMap only"
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now().to_msg()
        n = int(self.get_parameter("path_frames").value)
        fps = float(self.get_parameter("svo_fps").value)
        step_m = float(self.get_parameter("step_m").value)
        base_t = float(now.sec) + float(now.nanosec) * 1e-9
        x0 = 0.05 * self._tick
        y0 = 0.0

        path = Path()
        path.header.stamp = now
        path.header.frame_id = "map"
        for i in range(n + 1):
            pose = PoseStamped()
            t = base_t + i / max(fps, 1e-6)
            sec = int(t)
            pose.header.stamp.sec = sec
            pose.header.stamp.nanosec = int(round((t - sec) * 1e9))
            pose.header.frame_id = "map"
            pose.pose.position.x = x0 + i * step_m
            pose.pose.position.y = y0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_pub.publish(path)

        pos = PointStamped()
        pos.header = path.header
        pos.point.x = x0
        pos.point.y = y0
        pos.point.z = 0.0
        self._pos_pub.publish(pos)

        att = QuaternionStamped()
        att.header = path.header
        att.quaternion.w = 1.0
        self._att_pub.publish(att)

        # 41x41 map (resolution 1 m) covering a ~20 m future path.
        size = 41
        resolution = 1.0
        center_key_x = math.floor(x0 / resolution)
        center_key_y = math.floor(y0 / resolution)
        grid = GridMap()
        grid.header = path.header
        grid.info.resolution = resolution
        grid.info.length_x = size * resolution
        grid.info.length_y = size * resolution
        # Match lr_terrain_geometry pose convention: (center_key + 0.5) * resolution
        grid.info.pose.position.x = (center_key_x + 0.5) * resolution
        grid.info.pose.position.y = (center_key_y + 0.5) * resolution
        grid.info.pose.orientation.w = 1.0
        layers = [
            "elevation",
            "slope_deg",
            "rmse_m",
            "inlier_ratio",
            "state",
            "color",
            "normal_x",
            "normal_y",
            "normal_z",
        ]
        grid.layers = layers
        grid.basic_layers = ["elevation"]
        n_cells = size * size
        values = {
            "elevation": [0.0] * n_cells,
            "slope_deg": [0.0] * n_cells,
            "rmse_m": [0.01] * n_cells,
            "inlier_ratio": [1.0] * n_cells,
            "state": [1.0] * n_cells,
            "color": [0.0] * n_cells,
            "normal_x": [0.0] * n_cells,
            "normal_y": [0.0] * n_cells,
            "normal_z": [1.0] * n_cells,
        }
        grid.data = []
        for name in layers:
            values_2d = np.asarray(values[name], dtype=np.float32).reshape(
                (size, size), order="C"
            )
            # Match lr_terrain_geometry Fortran/column-major GridMap storage.
            flat = values_2d.reshape(-1, order="F").tolist()
            arr = Float32MultiArray()
            arr.layout.dim = [
                MultiArrayDimension(label="column_index", size=size, stride=size * size),
                MultiArrayDimension(label="row_index", size=size, stride=size),
            ]
            arr.data = flat
            grid.data.append(arr)
        self._grid_pub.publish(grid)
        self._tick += 1


def main(argv: list[str] | None = None) -> None:
    import rclpy

    rclpy.init(args=argv)
    node = UpstreamContractStub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
