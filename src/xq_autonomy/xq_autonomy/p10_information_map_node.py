"""Build P10 InformationMap messages from FAST-LIO registered scans."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from xq_sim_interfaces.msg import InformationMap

from .p10_active_perception_node import _cloud_xyz
from .surfel_map import TemporalVoxelSurfelMap


def _xyz_cloud(header, points: np.ndarray) -> PointCloud2:
    xyz = np.asarray(points, dtype="<f4").reshape((-1, 3))
    message = PointCloud2()
    message.header = header
    message.height = 1
    message.width = len(xyz)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(xyz)
    message.data = xyz.tobytes()
    message.is_dense = True
    return message


class P10InformationMapNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p10_information_map")
        defaults = {
            "input_topic": "/cloud_registered",
            "voxel_size_m": 0.40,
            "minimum_points": 8,
            "confidence_observations": 4,
            "minimum_geometry_quality": 0.10,
            "stale_after_s": 30.0,
            "maximum_voxels": 6000,
            "maximum_input_points": 30000,
            "publish_period_s": 0.50,
            "minimum_valid_surfels": 12,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._map = TemporalVoxelSurfelMap(
            voxel_size_m=float(self.get_parameter("voxel_size_m").value),
            minimum_points=int(self.get_parameter("minimum_points").value),
            confidence_observations=int(self.get_parameter("confidence_observations").value),
            minimum_geometry_quality=float(self.get_parameter("minimum_geometry_quality").value),
            stale_after_s=float(self.get_parameter("stale_after_s").value),
            maximum_voxels=int(self.get_parameter("maximum_voxels").value),
        )
        self._last_header = None
        self._received_scans = 0
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.map_publisher = self.create_publisher(InformationMap, "/integrity/information_map", qos)
        self.cloud_publisher = self.create_publisher(PointCloud2, "/xq/p5/cloud_map", qos)
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_topic").value), self._cloud_cb, qos
        )
        self.create_timer(float(self.get_parameter("publish_period_s").value), self._publish)
        self.get_logger().info(
            "P10 temporal surfel map ready: FAST-LIO registered cloud only; no Ground Truth"
        )

    def _cloud_cb(self, message: PointCloud2) -> None:
        points = _cloud_xyz(message)
        maximum = int(self.get_parameter("maximum_input_points").value)
        if len(points) > maximum:
            stride = int(math.ceil(len(points) / maximum))
            points = points[::stride]
        stamp_s = float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)
        try:
            self._map.update(points, stamp_s)
        except ValueError as error:
            self.get_logger().warning(f"Rejected registered scan: {error}")
            return
        self._last_header = message.header
        self._received_scans += 1

    def _publish(self) -> None:
        if self._last_header is None:
            return
        snapshot = self._map.snapshot()
        output = InformationMap()
        output.header = self._last_header
        output.positions = [
            Point(x=float(value[0]), y=float(value[1]), z=float(value[2]))
            for value in snapshot.positions
        ]
        output.normals = [
            Vector3(x=float(value[0]), y=float(value[1]), z=float(value[2]))
            for value in snapshot.normals
        ]
        output.static_confidence = snapshot.static_confidence.tolist()
        output.geometry_quality = snapshot.geometry_quality.tolist()
        output.last_seen_s = snapshot.last_seen_s.tolist()
        output.valid = len(snapshot.positions) >= int(
            self.get_parameter("minimum_valid_surfels").value
        )
        output.source = "/cloud_registered:temporal_voxel_surfel_v1"
        output.ground_truth_used = False
        self.map_publisher.publish(output)
        self.cloud_publisher.publish(_xyz_cloud(output.header, snapshot.positions))
        if self._received_scans % 20 == 0:
            self.get_logger().info(
                f"P10 InformationMap surfels={len(snapshot.positions)} "
                f"voxels={self._map.voxel_count} valid={output.valid}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P10InformationMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
