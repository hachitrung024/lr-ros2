"""Project synchronized 2D instance labels and registered depth into 3D."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


_CLOUD_DTYPE = np.dtype([
    ("x", "<f4"),
    ("y", "<f4"),
    ("z", "<f4"),
    ("rgb", "<f4"),
    ("instance_id", "<u4"),
])


def _stamp_nanoseconds(message) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


def instance_mask_message_to_labels(message: Image) -> np.ndarray:
    """Decode a mono16 instance-label image while respecting row padding."""
    if message.encoding.lower() not in {"mono16", "16uc1"}:
        raise ValueError("instance mask encoding must be mono16 or 16UC1")
    height = int(message.height)
    width = int(message.width)
    if height <= 0 or width <= 0:
        raise ValueError("instance mask dimensions must be positive")
    packed_width = width * 2
    if int(message.step) < packed_width:
        raise ValueError("instance mask step is smaller than its packed row")
    required = int(message.step) * height
    if len(message.data) < required:
        raise ValueError("instance mask data is shorter than step * height")

    rows = np.frombuffer(
        message.data,
        dtype=np.uint8,
        count=required,
    ).reshape(height, int(message.step))
    packed = np.ascontiguousarray(rows[:, :packed_width])
    byte_order = ">u2" if bool(message.is_bigendian) else "<u2"
    return packed.view(byte_order).reshape(height, width).astype(
        np.uint16,
        copy=False,
    )


def depth_message_to_meters(message: Image) -> np.ndarray:
    """Decode registered float-metre or uint16-millimetre depth."""
    encoding = message.encoding.lower()
    if encoding == "32fc1":
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        scale = 1.0
    elif encoding in {"16uc1", "mono16"}:
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        scale = 0.001
    else:
        raise ValueError(
            f"unsupported depth encoding '{message.encoding}'"
        )

    height = int(message.height)
    width = int(message.width)
    if height <= 0 or width <= 0:
        raise ValueError("depth dimensions must be positive")
    packed_width = width * dtype.itemsize
    if int(message.step) < packed_width:
        raise ValueError("depth step is smaller than its packed row")
    required = int(message.step) * height
    if len(message.data) < required:
        raise ValueError("depth data is shorter than step * height")

    rows = np.frombuffer(
        message.data,
        dtype=np.uint8,
        count=required,
    ).reshape(height, int(message.step))
    packed = np.ascontiguousarray(rows[:, :packed_width])
    depth = packed.view(dtype).reshape(height, width)
    return np.asarray(depth, dtype=np.float32) * scale


def _camera_intrinsics(camera_info: CameraInfo) -> tuple[float, ...]:
    projection = np.asarray(camera_info.p, dtype=np.float64).reshape(3, 4)
    if projection[0, 0] > 0.0 and projection[1, 1] > 0.0:
        values = (
            projection[0, 0],
            projection[1, 1],
            projection[0, 2],
            projection[1, 2],
        )
    else:
        intrinsic = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        values = (
            intrinsic[0, 0],
            intrinsic[1, 1],
            intrinsic[0, 2],
            intrinsic[1, 2],
        )
    if not np.isfinite(values).all() or values[0] <= 0.0 or values[1] <= 0.0:
        raise ValueError("CameraInfo is uncalibrated or malformed")
    return tuple(float(value) for value in values)


def project_mask_to_cloud(
    labels: np.ndarray,
    depth_m: np.ndarray,
    camera_info: CameraInfo,
    header,
    *,
    minimum_depth_m: float,
    maximum_depth_m: float,
    sampling_stride: int,
    maximum_points: int,
) -> PointCloud2:
    """Create a colored PointCloud2 containing only labeled pixels."""
    label_values = np.asarray(labels)
    depth_values = np.asarray(depth_m, dtype=np.float32)
    if label_values.ndim != 2 or label_values.dtype != np.uint16:
        raise ValueError("labels must be uint16 with shape (H, W)")
    if depth_values.shape != label_values.shape:
        raise ValueError("instance mask and registered depth sizes differ")
    if (
        camera_info.width not in {0, label_values.shape[1]}
        or camera_info.height not in {0, label_values.shape[0]}
    ):
        raise ValueError("CameraInfo dimensions differ from mask and depth")

    fx, fy, cx, cy = _camera_intrinsics(camera_info)
    sampled_labels = label_values[::sampling_stride, ::sampling_stride]
    sampled_depth = depth_values[::sampling_stride, ::sampling_stride]
    valid = (
        (sampled_labels > 0)
        & np.isfinite(sampled_depth)
        & (sampled_depth >= minimum_depth_m)
        & (sampled_depth <= maximum_depth_m)
    )
    rows, columns = np.nonzero(valid)
    if rows.size > maximum_points:
        selected = np.linspace(
            0,
            rows.size - 1,
            maximum_points,
            dtype=np.int64,
        )
        rows = rows[selected]
        columns = columns[selected]

    pixel_v = rows.astype(np.float32) * sampling_stride
    pixel_u = columns.astype(np.float32) * sampling_stride
    z = sampled_depth[rows, columns].astype(np.float32, copy=False)
    instance_ids = sampled_labels[rows, columns].astype(
        np.uint32,
        copy=False,
    )

    values = np.empty(rows.size, dtype=_CLOUD_DTYPE)
    values["x"] = (pixel_u - cx) * z / fx
    values["y"] = (pixel_v - cy) * z / fy
    values["z"] = z
    red = 50 + (instance_ids * 97) % 206
    green = 50 + (instance_ids * 57) % 206
    blue = 50 + (instance_ids * 23) % 206
    packed_rgb = (
        (red.astype(np.uint32) << 16)
        | (green.astype(np.uint32) << 8)
        | blue.astype(np.uint32)
    )
    values["rgb"] = packed_rgb.astype("<u4", copy=False).view("<f4")
    values["instance_id"] = instance_ids

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = values.size
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(
            name="instance_id",
            offset=16,
            datatype=PointField.UINT32,
            count=1,
        ),
    ]
    cloud.is_bigendian = False
    cloud.point_step = _CLOUD_DTYPE.itemsize
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = values.tobytes()
    cloud.is_dense = True
    return cloud


class MaskProjector3DNode(Node):
    """Synchronize mask/depth and publish a labeled 3D point cloud."""

    def __init__(
        self,
        *,
        parameter_overrides=None,
        node_name="mask_projector_3d",
        namespace="",
    ) -> None:
        super().__init__(
            node_name,
            namespace=namespace,
            parameter_overrides=parameter_overrides,
        )
        self._mask_topic = self.declare_parameter(
            "input.mask_topic",
            "/segmentation/instance_mask",
        ).value
        self._depth_topic = self.declare_parameter(
            "input.depth_topic",
            "/zed/zed_node/depth/depth_registered",
        ).value
        self._camera_info_topic = self.declare_parameter(
            "input.camera_info_topic",
            "/zed/zed_node/rgb/color/rect/camera_info",
        ).value
        self._cloud_topic = self.declare_parameter(
            "output.cloud_topic",
            "/segmentation/mask_cloud",
        ).value
        self._sync_tolerance_ns = int(
            float(self.declare_parameter("sync_tolerance_sec", 0.05).value)
            * 1_000_000_000
        )
        self._minimum_depth_m = float(
            self.declare_parameter("minimum_depth_m", 0.2).value
        )
        self._maximum_depth_m = float(
            self.declare_parameter("maximum_depth_m", 20.0).value
        )
        self._sampling_stride = self.declare_parameter(
            "sampling_stride",
            1,
        ).value
        self._maximum_points = self.declare_parameter(
            "maximum_points",
            200000,
        ).value
        self._validate_parameters()

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        self._cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self._cloud_topic),
            sensor_qos,
        )
        self._mask_subscription = self.create_subscription(
            Image,
            str(self._mask_topic),
            self._on_mask,
            sensor_qos,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            str(self._depth_topic),
            self._on_depth,
            sensor_qos,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            str(self._camera_info_topic),
            self._on_camera_info,
            sensor_qos,
        )
        self._pending_mask = None
        self._pending_depth = None
        self._camera_info = None
        self.get_logger().info(
            f"mask={self._mask_topic}; depth={self._depth_topic}; "
            f"cloud={self._cloud_topic}"
        )

    def _validate_parameters(self) -> None:
        for name, value in (
            ("input.mask_topic", self._mask_topic),
            ("input.depth_topic", self._depth_topic),
            ("input.camera_info_topic", self._camera_info_topic),
            ("output.cloud_topic", self._cloud_topic),
        ):
            if not str(value).strip():
                raise ValueError(f"{name} must not be empty")
        if self._sync_tolerance_ns < 0:
            raise ValueError("sync_tolerance_sec cannot be negative")
        if (
            not math.isfinite(self._minimum_depth_m)
            or self._minimum_depth_m < 0.0
            or not math.isfinite(self._maximum_depth_m)
            or self._maximum_depth_m <= self._minimum_depth_m
        ):
            raise ValueError("depth range is invalid")
        for name, value in (
            ("sampling_stride", self._sampling_stride),
            ("maximum_points", self._maximum_points),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")

    def _on_mask(self, message: Image) -> None:
        self._pending_mask = message
        self._try_project()

    def _on_depth(self, message: Image) -> None:
        self._pending_depth = message
        self._try_project()

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._camera_info = message
        self._try_project()

    def _try_project(self) -> None:
        if (
            self._pending_mask is None
            or self._pending_depth is None
            or self._camera_info is None
        ):
            return
        mask_stamp = _stamp_nanoseconds(self._pending_mask)
        depth_stamp = _stamp_nanoseconds(self._pending_depth)
        difference = mask_stamp - depth_stamp
        if abs(difference) > self._sync_tolerance_ns:
            if difference < 0:
                self._pending_mask = None
            else:
                self._pending_depth = None
            return

        mask_message = self._pending_mask
        depth_message = self._pending_depth
        self._pending_mask = None
        self._pending_depth = None
        try:
            labels = instance_mask_message_to_labels(mask_message)
            depth_m = depth_message_to_meters(depth_message)
            cloud = project_mask_to_cloud(
                labels,
                depth_m,
                self._camera_info,
                depth_message.header,
                minimum_depth_m=self._minimum_depth_m,
                maximum_depth_m=self._maximum_depth_m,
                sampling_stride=self._sampling_stride,
                maximum_points=self._maximum_points,
            )
        except (TypeError, ValueError) as error:
            self.get_logger().error(f"3D mask projection failed: {error}")
            return
        self._cloud_publisher.publish(cloud)


def main(args=None) -> None:
    """Run the 3D mask projector node."""
    rclpy.init(args=args)
    node = None
    try:
        node = MaskProjector3DNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
