"""Deterministic wide-room / narrow-passage P9 Gate stimulus."""

from __future__ import annotations

import struct
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import DirectionalIntegrity


WIDE_TRAJECTORY_ID = 9001
NARROW_TRAJECTORY_ID = 9002
FIXED_INTEGRITY_COVARIANCE = np.diag((1.6e-5, 1.6e-5, 1.6e-5))


def _wall_points(center_y: float, half_width: float) -> np.ndarray:
    x_values = np.linspace(-3.5, 3.5, 141)
    z_values = np.linspace(0.35, 2.45, 8)
    points = []
    for y in (center_y - half_width, center_y + half_width):
        for z in z_values:
            points.extend((x, y, z) for x in x_values)
    return np.asarray(points, dtype=np.float32)


def _point_cloud(points: np.ndarray, stamp: Time) -> PointCloud2:
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = "xq_lio_map"
    message.height = 1
    message.width = len(points)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.is_dense = True
    message.data = b"".join(struct.pack("<fff", *point) for point in points)
    return message


class P9GateScenarioNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p9_gate_scenario")
        self.cloud_publisher = self.create_publisher(PointCloud2, "/xq/p5/cloud_map", 20)
        self.trajectory_publisher = self.create_publisher(
            Bspline, "/planning/candidate_bspline", 20
        )
        self.integrity_publisher = self.create_publisher(
            DirectionalIntegrity, "/integrity/directional", 20
        )
        self.scenario_publisher = self.create_publisher(String, "/xq/p9/scenario", 20)
        self._start = time.monotonic()
        self._published_ids: set[int] = set()
        self.create_timer(0.10, self._tick)

    @staticmethod
    def _trajectory(stamp: Time, trajectory_id: int, center_y: float) -> Bspline:
        message = Bspline()
        message.order = 1
        message.start_time = stamp
        message.traj_id = trajectory_id
        message.pos_pts = [
            Point(x=-2.5, y=center_y, z=1.20),
            Point(x=2.5, y=center_y, z=1.20),
        ]
        message.knots = [0.0, 0.0, 6.0, 6.0]
        message.yaw_pts = [0.0, 0.0]
        message.yaw_dt = 6.0
        return message

    @staticmethod
    def _integrity(stamp: Time) -> DirectionalIntegrity:
        message = DirectionalIntegrity()
        message.header.stamp = stamp
        message.header.frame_id = "xq_lio_map"
        covariance = FIXED_INTEGRITY_COVARIANCE
        message.lio_position_covariance = covariance.reshape(-1).tolist()
        message.integrity_covariance = covariance.reshape(-1).tolist()
        message.information_matrix = np.eye(3).reshape(-1).tolist()
        message.geometry_eigenvalues = [1.0, 1.0, 1.0]
        message.weak_direction_map = [1.0, 0.0, 0.0]
        message.lambda_min = 1.0
        message.condition_number = 1.0
        message.protection_level_axes = (3.0 * np.sqrt(np.diag(covariance))).tolist()
        message.weak_direction_protection_level = 0.012
        message.kappa = 1.0
        message.k_alpha = 3.0
        message.eta = 0.02
        message.epsilon = 0.01
        message.effective_points = 2256
        return message

    def _tick(self) -> None:
        elapsed = time.monotonic() - self._start
        if elapsed < 1.0:
            return
        if elapsed < 4.0:
            name, trajectory_id, center_y, half_width = "wide_room", WIDE_TRAJECTORY_ID, 4.0, 1.5
        else:
            name, trajectory_id, center_y, half_width = "narrow_passage", NARROW_TRAJECTORY_ID, -4.0, 0.6
        stamp = self.get_clock().now().to_msg()
        self.integrity_publisher.publish(self._integrity(stamp))
        label = String()
        label.data = name
        self.scenario_publisher.publish(label)
        if trajectory_id not in self._published_ids:
            self.trajectory_publisher.publish(self._trajectory(stamp, trajectory_id, center_y))
            self._published_ids.add(trajectory_id)
        self.cloud_publisher.publish(_point_cloud(_wall_points(center_y, half_width), stamp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P9GateScenarioNode()
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
