#!/usr/bin/env python3
"""Report the exact static-map clearance of the current P11 candidates."""

from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from traj_utils.msg import Bspline
from xq_autonomy.alert_limit import sample_bspline
from xq_autonomy.p10_active_perception_node import _cloud_xyz
from xq_sim_interfaces.msg import IntegrityExplorationDecision


class CandidateClearanceProbe(Node):
    def __init__(self) -> None:
        super().__init__("xq_live_candidate_clearance_probe")
        qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cloud: np.ndarray | None = None
        self.candidates: dict[int, Bspline] = {}
        self.decision: IntegrityExplorationDecision | None = None
        self.create_subscription(
            PointCloud2,
            "/mapping/p12/static_voxels",
            lambda message: setattr(self, "cloud", _cloud_xyz(message)),
            qos,
        )
        self.create_subscription(
            Bspline,
            "/planning/p11/frontier_candidates",
            lambda message: self.candidates.__setitem__(int(message.traj_id), message),
            volatile_qos,
        )
        self.create_subscription(
            IntegrityExplorationDecision,
            "/integrity/exploration_decision",
            lambda message: setattr(self, "decision", message),
            qos,
        )

    def ready(self) -> bool:
        return bool(
            self.cloud is not None
            and self.decision is not None
            and all(int(value) in self.candidates for value in self.decision.trajectory_ids)
        )

    def report(self) -> dict[str, object]:
        assert self.cloud is not None and self.decision is not None
        records = []
        for index, trajectory_id in enumerate(self.decision.trajectory_ids):
            message = self.candidates[int(trajectory_id)]
            samples = sample_bspline(
                np.asarray([(point.x, point.y, point.z) for point in message.pos_pts]),
                np.asarray(message.knots),
                int(message.order),
                0.10,
            )
            deltas = samples[:, None, :] - self.cloud[None, :, :]
            distance2 = np.einsum("ijk,ijk->ij", deltas, deltas)
            flat = int(np.argmin(distance2))
            sample_index, obstacle_index = np.unravel_index(flat, distance2.shape)
            records.append(
                {
                    "name": str(self.decision.candidate_names[index]),
                    "trajectory_id": int(trajectory_id),
                    "predicted_minimum_margin_m": float(
                        self.decision.predicted_minimum_margins[index]
                    ),
                    "minimum_static_clearance_m": math.sqrt(
                        max(float(distance2[sample_index, obstacle_index]), 0.0)
                    ),
                    "critical_sample_xyz": samples[sample_index].tolist(),
                    "nearest_static_voxel_xyz": self.cloud[obstacle_index].tolist(),
                }
            )
        return {
            "batch_id": int(self.decision.batch_id),
            "static_voxel_count": len(self.cloud),
            "selected_name": str(self.decision.selected_name),
            "reason": str(self.decision.reason),
            "candidates": records,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=8.0)
    args = parser.parse_args()
    rclpy.init()
    node = CandidateClearanceProbe()
    deadline = time.monotonic() + args.timeout_s
    try:
        while time.monotonic() < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.20)
        if not node.ready():
            raise RuntimeError("timed out waiting for latched P11 candidate data")
        print(json.dumps(node.report(), indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
