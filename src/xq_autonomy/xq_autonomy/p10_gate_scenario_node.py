"""Deterministic ROS-contract stimulus for the P10 selector (not the formal flight Gate)."""

from __future__ import annotations

import struct
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import DirectionalIntegrity, InformationMap


BASELINE_ID = 10001


def _cloud(points: np.ndarray, stamp) -> PointCloud2:
    output = PointCloud2()
    output.header.stamp = stamp
    output.header.frame_id = "xq_lio_map"
    output.height = 1
    output.width = len(points)
    output.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    output.point_step = 12
    output.row_step = output.point_step * output.width
    output.is_dense = True
    output.data = b"".join(struct.pack("<fff", *point) for point in points.astype(np.float32))
    return output


class P10GateScenarioNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p10_gate_scenario")
        self.baseline_publisher = self.create_publisher(
            Bspline, "/planning/p10/baseline_bspline", 20
        )
        self.cloud_publisher = self.create_publisher(PointCloud2, "/xq/p5/cloud_map", 20)
        self.map_publisher = self.create_publisher(
            InformationMap, "/integrity/information_map", 20
        )
        self.integrity_publisher = self.create_publisher(
            DirectionalIntegrity, "/integrity/directional", 20
        )
        self.started = time.monotonic()
        self.create_timer(0.10, self._tick)

        line = np.linspace(-3.0, 3.5, 66)
        side = np.vstack((
            np.column_stack((line, np.full_like(line, 3.0), np.full_like(line, 1.2))),
            np.column_stack((line, np.full_like(line, -3.0), np.full_like(line, 1.2))),
        ))
        end_y = np.linspace(-3.0, 3.0, 61)
        end = np.column_stack((np.full_like(end_y, 3.5), end_y, np.full_like(end_y, 1.2)))
        feature_x = np.linspace(-1.5, 1.5, 25)
        self.features = np.column_stack((feature_x, np.full_like(feature_x, 1.9), np.full_like(feature_x, 1.2)))
        self.obstacles = np.vstack((side, end, self.features))

    @staticmethod
    def _baseline(stamp) -> Bspline:
        message = Bspline()
        message.order = 1
        message.traj_id = BASELINE_ID
        message.start_time = stamp
        message.pos_pts = [Point(x=-2.0, y=0.0, z=1.2), Point(x=2.0, y=0.0, z=1.2)]
        message.knots = [0.0, 0.0, 6.0, 6.0]
        message.yaw_pts = [0.0, 0.0]
        message.yaw_dt = 6.0
        return message

    @staticmethod
    def _integrity(stamp) -> DirectionalIntegrity:
        covariance = np.diag((2.56e-4, 1.0e-6, 1.0e-6))
        message = DirectionalIntegrity()
        message.header.stamp = stamp
        message.header.frame_id = "xq_lio_map"
        message.lio_position_covariance = covariance.reshape(-1).tolist()
        message.integrity_covariance = covariance.reshape(-1).tolist()
        message.information_matrix = np.diag((1.0, 20.0, 20.0)).reshape(-1).tolist()
        message.geometry_eigenvalues = [1.0, 20.0, 20.0]
        message.weak_direction_map = [1.0, 0.0, 0.0]
        message.lambda_min = 1.0
        message.condition_number = 20.0
        message.protection_level_axes = [0.048, 0.003, 0.003]
        message.weak_direction_protection_level = 0.048
        message.kappa = 1.0
        message.k_alpha = 3.0
        message.eta = 0.02
        message.epsilon = 0.01
        message.effective_points = 1200
        return message

    def _tick(self) -> None:
        if time.monotonic() - self.started < 0.5:
            return
        stamp = self.get_clock().now().to_msg()
        stamp_s = float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)
        self.cloud_publisher.publish(_cloud(self.obstacles, stamp))
        information = InformationMap()
        information.header.stamp = stamp
        information.header.frame_id = "xq_lio_map"
        information.positions = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.features]
        information.normals = [Vector3(x=1.0, y=0.0, z=0.0) for _ in self.features]
        information.static_confidence = [1.0] * len(self.features)
        information.geometry_quality = [1.0] * len(self.features)
        information.last_seen_s = [stamp_s] * len(self.features)
        information.valid = True
        information.source = "/xq/p5/cloud_map:local_surfel_extraction"
        information.ground_truth_used = False
        self.map_publisher.publish(information)
        self.integrity_publisher.publish(self._integrity(stamp))
        # Keep the latched stimulus reproducible even though this publisher
        # intentionally uses ordinary volatile QoS.  The selector de-duplicates
        # by trajectory ID, so late DDS discovery cannot lose the only baseline.
        self.baseline_publisher.publish(self._baseline(stamp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P10GateScenarioNode()
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
