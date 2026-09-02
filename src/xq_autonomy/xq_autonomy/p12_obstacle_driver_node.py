"""Evaluation-only Gazebo moving-obstacle trajectory driver for P12."""

from __future__ import annotations

import json
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class P12ObstacleDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p12_obstacle_driver")
        defaults = {
            "world_name": "xq_p12_dynamic_obstacle",
            "model_name": "xq_p12_moving_obstacle",
            "obstacle_x_m": -4.5,
            "park_y_m": 3.4,
            "blocked_y_m": 0.0,
            "obstacle_z_m": 1.0,
            "enter_start_s": 18.0,
            "enter_end_s": 22.0,
            "leave_start_s": 36.0,
            "leave_end_s": 40.0,
            "passage_half_width_m": 0.75,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        latched = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            String, "/xq/eval/p12/obstacle_state", latched
        )
        self._last_command_s = -1.0
        self._last_state = ""
        self._sequence = 0
        self.create_timer(0.50, self._timer)
        self.get_logger().info("P12 obstacle driver is evaluation/scenario-only")

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _trajectory(self, now_s: float) -> tuple[str, float, float]:
        park = self._float("park_y_m")
        blocked = self._float("blocked_y_m")
        enter0, enter1 = self._float("enter_start_s"), self._float("enter_end_s")
        leave0, leave1 = self._float("leave_start_s"), self._float("leave_end_s")
        if now_s < enter0:
            return "PARKED", park, 0.0
        if now_s < enter1:
            ratio = (now_s - enter0) / (enter1 - enter0)
            return "ENTERING", park + ratio * (blocked - park), (blocked - park) / (enter1 - enter0)
        if now_s < leave0:
            return "BLOCKING", blocked, 0.0
        if now_s < leave1:
            ratio = (now_s - leave0) / (leave1 - leave0)
            return "EXITING", blocked + ratio * (-park - blocked), (-park - blocked) / (leave1 - leave0)
        return "LEFT", -park, 0.0

    def _timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        if now_s <= 0.0 or now_s - self._last_command_s < 0.45:
            return
        state, y_m, velocity_y_mps = self._trajectory(now_s)
        self._last_command_s = now_s
        if state != self._last_state:
            self._sequence += 1
            self._last_state = state
            self.get_logger().info(f"P12 obstacle state={state} y={y_m:.2f}")
        message = String()
        message.data = json.dumps(
            {
                "sequence": self._sequence,
                "stamp_s": now_s,
                "state": state,
                "x_m": self._float("obstacle_x_m"),
                "y_m": y_m,
                "z_m": self._float("obstacle_z_m"),
                "passage_occupied": abs(y_m) <= self._float("passage_half_width_m"),
                "pose_applied": True,
                "command_mode": "gazebo_native_motion_system",
                "velocity_y_mps": velocity_y_mps,
                "evaluation_only": True,
            },
            separators=(",", ":"),
        )
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P12ObstacleDriverNode()
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
