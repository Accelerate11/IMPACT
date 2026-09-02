"""P6 Directional Integrity Predictor fed by FAST-LIO's real constraints."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from xq_sim_interfaces.msg import DirectionalIntegrity, LocalizationGeometry

from .integrity import TemporalInformationMemory, compute_directional_integrity


class P6DirectionalIntegrityNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p6_directional_integrity")
        defaults = {
            "eta": 0.02,
            "epsilon": 0.01,
            "k_alpha": 3.0,
            "a_d": 0.40,
            "a_nnis": 0.20,
            "a_timing": 0.20,
            "a_residual": 0.20,
            "nnis_term": 0.0,
            "timing_jitter_term": 0.0,
            "residual_dynamics_term": 0.0,
            "degeneracy_condition_reference": 100.0,
            "degeneracy_cap": 3.0,
            "covariance_floor": 1.0e-9,
            "maximum_odom_age_s": 0.25,
            # Disabled by default so all existing P6/P7 maps retain their
            # instantaneous constraint contract. Complex composition enables
            # the bounded memory explicitly through its launch profile.
            "information_memory_horizon_s": 0.0,
            "information_memory_max_frames": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._odom: Odometry | None = None
        self._published = 0
        self._information_memory = TemporalInformationMemory(
            horizon_s=self._parameter("information_memory_horizon_s"),
            maximum_equivalent_frames=self._parameter(
                "information_memory_max_frames"
            ),
        )
        self.publisher = self.create_publisher(DirectionalIntegrity, "/integrity/directional", 20)
        self.debug_publisher = self.create_publisher(String, "/integrity/debug", 10)
        self.create_subscription(Odometry, "/localization/odom", self._odom_callback, 20)
        self.create_subscription(LocalizationGeometry, "/localization/geometry", self._geometry_callback, 20)
        self.get_logger().info("P6 predictor ready: no Ground Truth subscription; planner feedback disabled")

    @staticmethod
    def _position_covariance(message: Odometry) -> np.ndarray:
        covariance = np.asarray(message.pose.covariance, dtype=float).reshape(6, 6)
        return covariance[:3, :3]

    def _odom_callback(self, message: Odometry) -> None:
        self._odom = message

    def _parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _geometry_callback(self, geometry: LocalizationGeometry) -> None:
        if self._odom is None:
            return
        geometry_stamp = geometry.header.stamp.sec + geometry.header.stamp.nanosec * 1.0e-9
        odom_stamp = self._odom.header.stamp.sec + self._odom.header.stamp.nanosec * 1.0e-9
        if abs(geometry_stamp - odom_stamp) > self._parameter("maximum_odom_age_s"):
            return
        observed_information = self._information_memory.update(
            np.asarray(geometry.information_matrix, dtype=float).reshape(3, 3),
            geometry_stamp,
        )
        result = compute_directional_integrity(
            observed_information,
            self._position_covariance(self._odom),
            eta=self._parameter("eta"),
            epsilon=self._parameter("epsilon"),
            k_alpha=self._parameter("k_alpha"),
            a_d=self._parameter("a_d"),
            a_nnis=self._parameter("a_nnis"),
            a_timing=self._parameter("a_timing"),
            a_residual=self._parameter("a_residual"),
            nnis_term=self._parameter("nnis_term"),
            timing_jitter_term=self._parameter("timing_jitter_term"),
            residual_dynamics_term=self._parameter("residual_dynamics_term"),
            degeneracy_condition_reference=self._parameter("degeneracy_condition_reference"),
            degeneracy_cap=self._parameter("degeneracy_cap"),
            covariance_floor=self._parameter("covariance_floor"),
        )
        message = DirectionalIntegrity()
        message.header = geometry.header
        message.lio_position_covariance = self._position_covariance(self._odom).reshape(-1).tolist()
        message.integrity_covariance = result.integrity_covariance.reshape(-1).tolist()
        message.information_matrix = observed_information.reshape(-1).tolist()
        message.geometry_eigenvalues = result.geometry_eigenvalues.tolist()
        message.weak_direction_map = result.weak_direction.tolist()
        message.lambda_min = result.lambda_min
        message.condition_number = result.condition_number
        message.protection_level_axes = result.protection_level_axes.tolist()
        message.weak_direction_protection_level = result.weak_direction_protection_level
        message.kappa = result.kappa
        message.k_alpha = self._parameter("k_alpha")
        message.eta = self._parameter("eta")
        message.epsilon = self._parameter("epsilon")
        message.degeneracy_term = result.degeneracy_term
        message.nnis_term = self._parameter("nnis_term")
        message.timing_jitter_term = self._parameter("timing_jitter_term")
        message.residual_dynamics_term = self._parameter("residual_dynamics_term")
        message.effective_points = geometry.effective_points
        self.publisher.publish(message)
        self._published += 1
        if self._published % 10 == 0:
            debug = String()
            debug.data = json.dumps(
                {
                    "phase": "P6_DIRECTIONAL_INTEGRITY",
                    "lambda_min": result.lambda_min,
                    "condition_number": result.condition_number,
                    "weak_direction_map": result.weak_direction.tolist(),
                    "pl_axes_m": result.protection_level_axes.tolist(),
                    "pl_weak_m": result.weak_direction_protection_level,
                    "kappa": result.kappa,
                    "effective_points": int(geometry.effective_points),
                    "information_memory_horizon_s": self._parameter(
                        "information_memory_horizon_s"
                    ),
                    "information_memory_equivalent_frames": (
                        self._information_memory.equivalent_frames
                    ),
                    "ground_truth_subscribed": False,
                    "planner_feedback_enabled": False,
                },
                separators=(",", ":"),
            )
            self.debug_publisher.publish(debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P6DirectionalIntegrityNode()
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
