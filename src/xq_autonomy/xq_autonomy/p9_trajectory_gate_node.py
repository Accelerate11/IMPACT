"""Transport-level hard gate: only ACCEPTed candidate B-splines pass downstream."""

from __future__ import annotations

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import IntegrityMargin


class P9TrajectoryGateNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p9_trajectory_gate")
        self._candidates: dict[int, Bspline] = {}
        self._decided: set[int] = set()
        self.trajectory_publisher = self.create_publisher(Bspline, "/planning/bspline", 20)
        self.certified_publisher = self.create_publisher(Bool, "/planning/trajectory_certified", 20)
        self.debug_publisher = self.create_publisher(String, "/planning/certification_debug", 10)
        self.create_subscription(Bspline, "/planning/candidate_bspline", self._candidate_cb, 20)
        self.create_subscription(IntegrityMargin, "/integrity/margin", self._margin_cb, 20)
        self.get_logger().info("P9 transport gate ready: REJECT means no downstream B-spline")

    def _candidate_cb(self, message: Bspline) -> None:
        trajectory_id = int(message.traj_id)
        self._candidates[trajectory_id] = message
        # Bound memory without changing a decision already made for an ID.
        if len(self._candidates) > 100:
            for old_id in sorted(self._candidates)[:-50]:
                self._candidates.pop(old_id, None)

    def _margin_cb(self, message: IntegrityMargin) -> None:
        trajectory_id = int(message.trajectory_id)
        if trajectory_id in self._decided or trajectory_id not in self._candidates:
            return
        accepted = bool(message.valid and message.hard_constraint and message.accepted)
        decision = Bool()
        decision.data = accepted
        self.certified_publisher.publish(decision)
        if accepted:
            self.trajectory_publisher.publish(self._candidates[trajectory_id])
        debug = String()
        debug.data = json.dumps(
            {
                "phase": "P9_TRAJECTORY_GATE",
                "trajectory_id": trajectory_id,
                "decision": "ACCEPT" if accepted else "REJECT",
                "downstream_published": accepted,
                "hard_constraint": True,
                "weighted_cost": False,
                "reason": message.decision_reason,
            },
            separators=(",", ":"),
        )
        self.debug_publisher.publish(debug)
        # Missing/stale inputs remain fail-closed but may be retried when a
        # later valid synchronized profile arrives.  A valid ACCEPT/REJECT is
        # final for this candidate trajectory ID.
        if message.valid and message.hard_constraint:
            self._decided.add(trajectory_id)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P9TrajectoryGateNode()
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
