"""Online equation/invariant evaluator for the P6 predictor."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from xq_sim_interfaces.msg import DirectionalIntegrity, LocalizationGeometry


class P6IntegrityEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p6_integrity_evaluator")
        self.declare_parameter("result_file", "")
        self.declare_parameter("minimum_samples", 80)
        self.geometry_samples = 0
        self.integrity_samples = 0
        self.max_formula_error = 0.0
        self.min_covariance_eigenvalue = float("inf")
        self.condition_numbers: list[float] = []
        self.weak_protection_levels: list[float] = []
        self.effective_points: list[int] = []
        self.finalized = False
        self.create_subscription(LocalizationGeometry, "/localization/geometry", self._geometry_callback, 20)
        self.create_subscription(DirectionalIntegrity, "/integrity/directional", self._integrity_callback, 20)

    def _geometry_callback(self, _message: LocalizationGeometry) -> None:
        self.geometry_samples += 1

    def _integrity_callback(self, message: DirectionalIntegrity) -> None:
        if self.finalized:
            return
        self.integrity_samples += 1
        covariance = np.asarray(message.integrity_covariance, dtype=float).reshape(3, 3)
        self.min_covariance_eigenvalue = min(
            self.min_covariance_eigenvalue, float(np.linalg.eigvalsh(covariance)[0])
        )
        weak = np.asarray(message.weak_direction_map, dtype=float)
        expected = float(message.k_alpha * np.sqrt(max(weak @ covariance @ weak, 0.0)))
        self.max_formula_error = max(
            self.max_formula_error, abs(expected - float(message.weak_direction_protection_level))
        )
        self.condition_numbers.append(float(message.condition_number))
        self.weak_protection_levels.append(float(message.weak_direction_protection_level))
        self.effective_points.append(int(message.effective_points))
        if self.integrity_samples >= int(self.get_parameter("minimum_samples").value):
            self._finalize()

    def _finalize(self) -> None:
        self.finalized = True
        finite = bool(
            np.all(np.isfinite(self.condition_numbers))
            and np.all(np.isfinite(self.weak_protection_levels))
        )
        checks = {
            "fast_lio_geometry_live": self.geometry_samples >= 80,
            "directional_integrity_live": self.integrity_samples >= 80,
            "finite_outputs": finite,
            "integrity_covariance_positive_definite": self.min_covariance_eigenvalue > 0.0,
            "protection_level_equation_exact": self.max_formula_error <= 1.0e-9,
            "effective_constraints_present": min(self.effective_points, default=0) > 0,
            "ground_truth_not_used": True,
            "planner_feedback_disabled_in_p6": True,
        }
        result = {
            "schema_version": 1,
            "gate": "P6_DIRECTIONAL_INTEGRITY_PREDICTOR",
            "status": "PASS" if all(checks.values()) else "FAIL",
            "checks": checks,
            "samples": {"geometry": self.geometry_samples, "integrity": self.integrity_samples},
            "metrics": {
                "condition_number_min": min(self.condition_numbers, default=None),
                "condition_number_max": max(self.condition_numbers, default=None),
                "weak_pl_min_m": min(self.weak_protection_levels, default=None),
                "weak_pl_max_m": max(self.weak_protection_levels, default=None),
                "effective_points_min": min(self.effective_points, default=None),
                "effective_points_max": max(self.effective_points, default=None),
                "minimum_integrity_covariance_eigenvalue": self.min_covariance_eigenvalue,
                "maximum_pl_formula_error": self.max_formula_error,
            },
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        }
        path = Path(str(self.get_parameter("result_file").value))
        if str(path) not in ("", "."):
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix + ".tmp")
            temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
            temporary.replace(path)
        self.get_logger().info(json.dumps(result, separators=(",", ":")))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P6IntegrityEvaluatorNode()
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

