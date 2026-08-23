"""P9 trajectory-wide Integrity Margin certifier (hard constraint)."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from xq_sim_interfaces.msg import AlertLimitProfile, DirectionalIntegrity, IntegrityMargin

from .integrity_margin import certify_trajectory


class P9IntegrityMarginNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p9_integrity_margin")
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("calibration_sha256", "")
        self.declare_parameter("margin_reserve_m", 0.10)
        self.declare_parameter("maximum_integrity_age_s", 0.50)
        self._integrity: DirectionalIntegrity | None = None
        self._k_alpha, self._calibration_sha256 = self._load_calibration()
        self.publisher = self.create_publisher(IntegrityMargin, "/integrity/margin", 20)
        self.debug_publisher = self.create_publisher(String, "/integrity/margin_debug", 10)
        self.create_subscription(
            DirectionalIntegrity, "/integrity/directional", self._integrity_cb, 20
        )
        self.create_subscription(
            AlertLimitProfile, "/integrity/alert_limit_profile", self._alert_cb, 20
        )
        self.get_logger().info(
            "P9 hard certifier ready: max train-only P7 k95; no Ground Truth; no weighted cost"
        )

    def _load_calibration(self) -> tuple[float, str]:
        path = Path(str(self.get_parameter("calibration_file").value))
        if not path.is_file():
            raise ValueError(f"P7 calibration file missing: {path}")
        payload = path.read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        expected = str(self.get_parameter("calibration_sha256").value).strip()
        if expected and digest != expected:
            raise ValueError("P7 calibration SHA-256 mismatch")
        calibration = json.loads(payload)
        if not calibration.get("train_only") or calibration.get("test_data_used", True):
            raise ValueError("P9 only accepts a train-only frozen P7 calibration")
        directional = calibration.get("directional", {})
        factors = [float(value["k95"]) for value in directional.values()]
        if not factors or not np.isfinite(factors).all() or min(factors) <= 0.0:
            raise ValueError("P7 calibration has no valid k95 factors")
        # P7 calibrated named directions, while P9 obstacle normals are arbitrary.
        # The maximum frozen train-only k95 is the conservative scalar k_alpha
        # required by PL(a)=k_alpha*sqrt(a^T P_int a).
        return max(factors), digest

    @staticmethod
    def _stamp_s(message) -> float:
        return float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)

    def _integrity_cb(self, message: DirectionalIntegrity) -> None:
        self._integrity = message

    def _publish_invalid(self, alert: AlertLimitProfile, reason: str) -> None:
        output = IntegrityMargin()
        output.header = alert.header
        output.trajectory_id = alert.trajectory_id
        output.trajectory_sample_count = alert.trajectory_sample_count
        output.margin_reserve = float(self.get_parameter("margin_reserve_m").value)
        output.k_alpha = self._k_alpha
        output.accepted = False
        output.valid = False
        output.hard_constraint = True
        output.decision_reason = reason
        output.calibration_sha256 = self._calibration_sha256
        self.publisher.publish(output)

    def _alert_cb(self, alert: AlertLimitProfile) -> None:
        if self._integrity is None:
            self._publish_invalid(alert, "REJECT_NO_INTEGRITY_INPUT")
            return
        age = abs(self._stamp_s(alert) - self._stamp_s(self._integrity))
        if age > float(self.get_parameter("maximum_integrity_age_s").value):
            self._publish_invalid(alert, "REJECT_STALE_INTEGRITY_INPUT")
            return
        count = int(alert.trajectory_sample_count)
        if (
            not alert.valid
            or len(alert.trajectory_sample_points) != count
            or len(alert.sample_nearest_obstacle_points) != count
            or len(alert.sample_obstacle_directions_map) != 3 * count
            or len(alert.sample_alert_limits) != count
        ):
            self._publish_invalid(alert, "REJECT_INVALID_ALERT_LIMIT_PROFILE")
            return
        try:
            result = certify_trajectory(
                np.asarray(alert.sample_alert_limits, dtype=float),
                np.asarray(alert.sample_obstacle_directions_map, dtype=float).reshape(-1, 3),
                np.asarray(self._integrity.integrity_covariance, dtype=float).reshape(3, 3),
                k_alpha=self._k_alpha,
                margin_reserve=float(self.get_parameter("margin_reserve_m").value),
            )
        except ValueError as error:
            self._publish_invalid(alert, f"REJECT_INVALID_MATH_INPUT:{error}")
            return
        index = result.critical_index
        output = IntegrityMargin()
        output.header = alert.header
        output.trajectory_id = alert.trajectory_id
        output.trajectory_sample_count = count
        output.critical_sample_index = index
        output.critical_sample_point = alert.trajectory_sample_points[index]
        output.nearest_obstacle_point = alert.sample_nearest_obstacle_points[index]
        direction = np.asarray(alert.sample_obstacle_directions_map, dtype=float).reshape(-1, 3)[index]
        output.obstacle_direction_map = direction.tolist()
        output.alert_limit = float(alert.sample_alert_limits[index])
        output.protection_level = float(result.protection_levels[index])
        output.minimum_margin = result.minimum_margin
        output.margin_reserve = float(self.get_parameter("margin_reserve_m").value)
        output.k_alpha = self._k_alpha
        output.sample_protection_levels = result.protection_levels.tolist()
        output.sample_margins = result.margins.tolist()
        output.accepted = result.accepted
        output.valid = True
        output.hard_constraint = True
        output.decision_reason = "ACCEPT_MARGIN_RESERVED" if result.accepted else "REJECT_MARGIN_VIOLATION"
        output.calibration_sha256 = self._calibration_sha256
        self.publisher.publish(output)
        debug = String()
        debug.data = json.dumps(
            {
                "phase": "P9_INTEGRITY_MARGIN",
                "trajectory_id": int(output.trajectory_id),
                "decision": "ACCEPT" if output.accepted else "REJECT",
                "minimum_margin_m": output.minimum_margin,
                "margin_reserve_m": output.margin_reserve,
                "alert_limit_m": output.alert_limit,
                "protection_level_m": output.protection_level,
                "k_alpha": output.k_alpha,
                "hard_constraint": True,
                "weighted_cost": False,
                "ground_truth_subscribed": False,
            },
            separators=(",", ":"),
        )
        self.debug_publisher.publish(debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P9IntegrityMarginNode()
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
