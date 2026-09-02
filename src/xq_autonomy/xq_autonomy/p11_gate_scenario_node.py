"""Deterministic P11 ROS-contract stimulus; no Ground Truth is used."""

from __future__ import annotations

import struct
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import DirectionalIntegrity, ExplorationCandidateSet, InformationMap


IDS = (20261101, 20261102, 20261103, 20261104)
NAMES = (
    "high_information_direct",
    "geometry_rich_right",
    "collision_violation",
    "return_energy_violation",
)


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


class P11GateScenarioNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p11_gate_scenario")
        reliable = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.candidate_publisher = self.create_publisher(
            Bspline, "/planning/p11/frontier_candidates", reliable
        )
        self.metadata_publisher = self.create_publisher(
            ExplorationCandidateSet, "/planning/p11/frontier_candidate_set", latched
        )
        self.cloud_publisher = self.create_publisher(PointCloud2, "/xq/p5/cloud_map", reliable)
        self.map_publisher = self.create_publisher(
            InformationMap, "/integrity/information_map", reliable
        )
        self.integrity_publisher = self.create_publisher(
            DirectionalIntegrity, "/integrity/directional", reliable
        )
        self.started = time.monotonic()
        self.create_timer(0.10, self._tick)

        x = np.linspace(-2.5, 3.0, 80)
        walls = np.vstack(
            (
                np.column_stack((x, np.full_like(x, 3.0), np.full_like(x, 1.2))),
                np.column_stack((x, np.full_like(x, -3.0), np.full_like(x, 1.2))),
            )
        )
        # The narrow direct route has little clearance; the right-hand viewpoint
        # creates clearance and observes the feature surface.  This is a frozen
        # Frontier output contract, not a map-quality benchmark.
        panel_x = np.linspace(-0.55, 0.55, 32)
        panel = np.column_stack(
            (panel_x, np.full_like(panel_x, 0.64), np.full_like(panel_x, 1.2))
        )
        feature_x = np.linspace(-1.7, 1.7, 48)
        self.features = np.column_stack(
            (feature_x, np.full_like(feature_x, -2.15), np.full_like(feature_x, 1.2))
        )
        self.obstacles = np.vstack((walls, panel))

    def _trajectory(self, trajectory_id: int, lateral: float, stamp) -> Bspline:
        phase = np.linspace(0.0, 1.0, 17)
        positions = np.column_stack(
            (-2.0 + 4.0 * phase, lateral * np.sin(np.pi * phase), np.full_like(phase, 1.2))
        )
        duration = 8.0
        message = Bspline()
        message.order = 1
        message.traj_id = trajectory_id
        message.start_time = stamp
        message.pos_pts = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in positions]
        message.knots = [0.0, 0.0, *[duration * i / 16.0 for i in range(1, 16)], duration, duration]
        message.yaw_pts = [0.0] * len(positions)
        message.yaw_dt = duration / 16.0
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
        candidates = (
            self._trajectory(IDS[0], 0.0, stamp),
            self._trajectory(IDS[1], -1.2, stamp),
            self._trajectory(IDS[2], -1.2, stamp),
            self._trajectory(IDS[3], -1.2, stamp),
        )
        for candidate in candidates:
            self.candidate_publisher.publish(candidate)
        metadata = ExplorationCandidateSet()
        metadata.header.stamp = stamp
        metadata.header.frame_id = "xq_lio_map"
        metadata.batch_id = 202611
        metadata.trajectory_ids = list(IDS)
        metadata.candidate_names = list(NAMES)
        metadata.frontier_ids = ["frontier_main"] * len(IDS)
        metadata.information_gains = [1.0, 0.75, 1.0, 1.0]
        metadata.travel_times_s = [8.0, 8.0, 1.0, 1.0]
        metadata.energy_costs = [4.0, 4.2, 0.1, 19.0]
        metadata.return_energy_costs = [4.0, 4.0, 0.1, 4.0]
        metadata.collision_probabilities = [0.001, 0.001, 0.20, 0.001]
        metadata.ground_truth_used = False
        self.metadata_publisher.publish(metadata)
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
        information.source = "p11_contract:frozen_frontier_map"
        information.ground_truth_used = False
        self.map_publisher.publish(information)
        self.integrity_publisher.publish(self._integrity(stamp))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P11GateScenarioNode()
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
