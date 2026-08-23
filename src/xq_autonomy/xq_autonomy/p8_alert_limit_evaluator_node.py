"""Independent equation/geometry Gate for P8 Alert Limit outputs."""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from xq_sim_interfaces.msg import AlertLimit


class P8AlertLimitEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("xq_p8_alert_limit_evaluator")
        self.declare_parameter("result_file", "")
        self.declare_parameter("minimum_messages", 100)
        self.declare_parameter("minimum_trajectory_ids", 3)
        self.messages = 0
        self.trajectory_ids: set[int] = set()
        self.clearances: list[float] = []
        self.alert_limits: list[float] = []
        self.latency_reserves: list[float] = []
        self.negative_alert_limits = 0
        self.invalid = 0
        self.max_direction_norm_error = 0.0
        self.max_distance_error = 0.0
        self.max_equation_error = 0.0
        self.maximum_samples = 0
        self.maximum_obstacles = 0
        self.source_ok = True
        self.static_ok = True
        self.create_subscription(AlertLimit, "/integrity/alert_limit", self._callback, 50)
        self.create_timer(0.5, self._write)

    def _callback(self, message: AlertLimit) -> None:
        self.messages += 1
        self.trajectory_ids.add(int(message.trajectory_id))
        self.maximum_samples = max(self.maximum_samples, int(message.trajectory_sample_count))
        self.maximum_obstacles = max(self.maximum_obstacles, int(message.obstacle_point_count))
        self.source_ok &= message.obstacle_source == "/xq/p5/cloud_map"
        self.static_ok &= bool(message.static_obstacles_only) and abs(message.dynamic_reserve) <= 1.0e-12
        values = np.array(
            (
                message.geometric_clearance,
                message.body_radius,
                message.base_reserve,
                message.tracking_reserve,
                message.dynamic_reserve,
                message.latency_reserve,
                message.speed,
                message.latency_p99,
                message.maximum_acceleration,
                message.alert_limit,
                *message.obstacle_direction_map,
            ),
            dtype=float,
        )
        if not message.valid or not np.isfinite(values).all():
            self.invalid += 1
            return
        sample = np.array(
            (message.critical_sample_point.x, message.critical_sample_point.y, message.critical_sample_point.z)
        )
        obstacle = np.array(
            (message.nearest_obstacle_point.x, message.nearest_obstacle_point.y, message.nearest_obstacle_point.z)
        )
        direction = np.asarray(message.obstacle_direction_map, dtype=float)
        self.max_direction_norm_error = max(
            self.max_direction_norm_error, abs(float(np.linalg.norm(direction)) - 1.0)
        )
        self.max_distance_error = max(
            self.max_distance_error,
            abs(float(np.linalg.norm(obstacle - sample)) - message.geometric_clearance),
        )
        expected_latency = message.speed * message.latency_p99 + 0.5 * message.maximum_acceleration * message.latency_p99**2
        expected_alert = (
            message.geometric_clearance
            - message.body_radius
            - message.base_reserve
            - message.tracking_reserve
            - message.dynamic_reserve
            - expected_latency
        )
        self.max_equation_error = max(
            self.max_equation_error,
            abs(message.latency_reserve - expected_latency),
            abs(message.alert_limit - expected_alert),
        )
        self.clearances.append(float(message.geometric_clearance))
        self.alert_limits.append(float(message.alert_limit))
        self.latency_reserves.append(float(message.latency_reserve))
        self.negative_alert_limits += int(message.alert_limit < 0.0)

    def _snapshot(self) -> dict:
        minimum_messages = int(self.get_parameter("minimum_messages").value)
        minimum_ids = int(self.get_parameter("minimum_trajectory_ids").value)
        clearance_range = max(self.clearances) - min(self.clearances) if self.clearances else 0.0
        checks = {
            "minimum_messages": self.messages >= minimum_messages,
            "multiple_ego_trajectories": len(self.trajectory_ids) >= minimum_ids,
            "all_outputs_valid": self.invalid == 0,
            "trajectory_samples_present": self.maximum_samples >= 2,
            "obstacle_points_present": self.maximum_obstacles >= 10,
            "static_obstacle_v1": self.static_ok,
            "mapped_cloud_source": self.source_ok,
            "direction_unit_norm": self.max_direction_norm_error <= 1.0e-6,
            "nearest_point_geometry": self.max_distance_error <= 1.0e-6,
            "alert_limit_equation": self.max_equation_error <= 1.0e-9,
            "environmental_clearance_varies": clearance_range >= 0.05,
        }
        status = "PASS" if all(checks.values()) else "IN_PROGRESS"
        return {
            "schema_version": 1,
            "gate": "P8_STATIC_OBSTACLE_ALERT_LIMIT",
            "status": status,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "checks": checks,
            "metrics": {
                "messages": self.messages,
                "trajectory_ids": len(self.trajectory_ids),
                "maximum_trajectory_samples": self.maximum_samples,
                "maximum_obstacle_points": self.maximum_obstacles,
                "geometric_clearance_min_m": min(self.clearances) if self.clearances else None,
                "geometric_clearance_max_m": max(self.clearances) if self.clearances else None,
                "alert_limit_min_m": min(self.alert_limits) if self.alert_limits else None,
                "alert_limit_max_m": max(self.alert_limits) if self.alert_limits else None,
                "alert_limit_mean_m": float(np.mean(self.alert_limits)) if self.alert_limits else None,
                "latency_reserve_max_m": max(self.latency_reserves) if self.latency_reserves else None,
                "negative_alert_limit_messages": self.negative_alert_limits,
                "maximum_direction_norm_error": self.max_direction_norm_error,
                "maximum_nearest_point_distance_error_m": self.max_distance_error,
                "maximum_equation_error_m": self.max_equation_error,
            },
            "data_contract": {
                "trajectory_source": "/planning/bspline",
                "obstacle_source": "/xq/p5/cloud_map",
                "ground_truth_subscribed": False,
                "planner_feedback_enabled": False,
                "dynamic_obstacles_enabled": False,
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
    node = P8AlertLimitEvaluator()
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
