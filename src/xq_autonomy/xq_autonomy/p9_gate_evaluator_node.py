"""Independent P9 wide-room / narrow-passage acceptance Gate."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import DirectionalIntegrity, IntegrityMargin

from .p9_gate_scenario_node import NARROW_TRAJECTORY_ID, WIDE_TRAJECTORY_ID


class P9GateEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p9_gate_evaluator")
        self.declare_parameter("result_file", "")
        self.margins: dict[int, list[IntegrityMargin]] = {
            WIDE_TRAJECTORY_ID: [],
            NARROW_TRAJECTORY_ID: [],
        }
        self.certified_ids: set[int] = set()
        self.reference_covariance: np.ndarray | None = None
        self.maximum_covariance_difference = 0.0
        self.maximum_margin_equation_error = 0.0
        self.maximum_decision_error = 0
        self.create_subscription(IntegrityMargin, "/integrity/margin", self._margin_cb, 50)
        self.create_subscription(
            DirectionalIntegrity, "/integrity/directional", self._integrity_cb, 50
        )
        self.create_subscription(Bspline, "/planning/bspline", self._certified_cb, 20)
        self.create_timer(0.25, self._write)

    def _integrity_cb(self, message: DirectionalIntegrity) -> None:
        covariance = np.asarray(message.integrity_covariance, dtype=float).reshape(3, 3)
        if self.reference_covariance is None:
            self.reference_covariance = covariance
        else:
            self.maximum_covariance_difference = max(
                self.maximum_covariance_difference,
                float(np.max(np.abs(covariance - self.reference_covariance))),
            )

    def _margin_cb(self, message: IntegrityMargin) -> None:
        trajectory_id = int(message.trajectory_id)
        if trajectory_id not in self.margins or not message.valid:
            return
        self.margins[trajectory_id].append(message)
        self.maximum_margin_equation_error = max(
            self.maximum_margin_equation_error,
            abs(float(message.minimum_margin) - (float(message.alert_limit) - float(message.protection_level))),
        )
        expected_accept = message.minimum_margin >= message.margin_reserve
        self.maximum_decision_error = max(
            self.maximum_decision_error, int(bool(message.accepted) != bool(expected_accept))
        )

    def _certified_cb(self, message: Bspline) -> None:
        self.certified_ids.add(int(message.traj_id))

    @staticmethod
    def _minimum(messages: list[IntegrityMargin], field: str):
        return min((float(getattr(message, field)) for message in messages), default=None)

    @staticmethod
    def _maximum(messages: list[IntegrityMargin], field: str):
        return max((float(getattr(message, field)) for message in messages), default=None)

    def _snapshot(self) -> dict:
        wide = self.margins[WIDE_TRAJECTORY_ID]
        narrow = self.margins[NARROW_TRAJECTORY_ID]
        wide_accept = bool(wide) and all(message.accepted for message in wide)
        narrow_reject = bool(narrow) and all(not message.accepted for message in narrow)
        wide_clearance = self._minimum(wide, "alert_limit")
        narrow_clearance = self._maximum(narrow, "alert_limit")
        checks = {
            "wide_room_profile_received": len(wide) >= 3,
            "narrow_passage_profile_received": len(narrow) >= 3,
            "identical_localization_covariance": self.maximum_covariance_difference <= 1.0e-15,
            "wide_room_accept": wide_accept,
            "narrow_passage_reject": narrow_reject,
            "wide_room_transport_passed": WIDE_TRAJECTORY_ID in self.certified_ids,
            "narrow_passage_transport_blocked": NARROW_TRAJECTORY_ID not in self.certified_ids,
            "margin_equation_exact": self.maximum_margin_equation_error <= 1.0e-12,
            "hard_decision_exact": self.maximum_decision_error == 0,
            "geometric_ordering_intuitive": (
                wide_clearance is not None
                and narrow_clearance is not None
                and wide_clearance > narrow_clearance + 0.50
            ),
            "hard_constraint_not_weighted_cost": all(
                message.hard_constraint for message in wide + narrow
            ),
        }
        ready = len(wide) >= 3 and len(narrow) >= 3
        return {
            "schema_version": 1,
            "gate": "P9_INTEGRITY_MARGIN_HARD_CONSTRAINT",
            "status": "PASS" if ready and all(checks.values()) else "IN_PROGRESS",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "checks": checks,
            "metrics": {
                "wide_messages": len(wide),
                "narrow_messages": len(narrow),
                "wide_minimum_margin_m": self._minimum(wide, "minimum_margin"),
                "narrow_maximum_margin_m": self._maximum(narrow, "minimum_margin"),
                "wide_minimum_alert_limit_m": wide_clearance,
                "narrow_maximum_alert_limit_m": narrow_clearance,
                "wide_protection_level_m": self._maximum(wide, "protection_level"),
                "narrow_protection_level_m": self._maximum(narrow, "protection_level"),
                "maximum_covariance_difference": self.maximum_covariance_difference,
                "maximum_margin_equation_error_m": self.maximum_margin_equation_error,
                "certified_trajectory_ids": sorted(self.certified_ids),
            },
            "data_contract": {
                "same_covariance": True,
                "ground_truth_subscribed": False,
                "calibration_source": "P7 train-only frozen k95",
                "decision_rule": "minimum_margin >= margin_reserve",
                "weighted_cost": False,
            },
        }

    def _write(self) -> None:
        path = Path(str(self.get_parameter("result_file").value))
        if str(path) in ("", "."):
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(self._snapshot(), indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P9GateEvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
