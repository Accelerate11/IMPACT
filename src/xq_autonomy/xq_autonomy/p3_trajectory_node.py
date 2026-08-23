"""Deterministic, open-loop P3 localization benchmark trajectories."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class P3TrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p3_trajectory")
        self.declare_parameter("scenario", "structured_room")
        self.declare_parameter("cmd_topic", "/xq/p3/cmd_vel")
        self.declare_parameter("trajectory_variant", "baseline")
        self.scenario = str(self.get_parameter("scenario").value)
        self.variant = str(self.get_parameter("trajectory_variant").value)
        if self.scenario not in ("structured_room", "long_corridor"):
            raise ValueError(f"unsupported P3 scenario: {self.scenario}")
        if self.variant not in ("baseline", "train", "test", "validation"):
            raise ValueError(f"unsupported trajectory variant: {self.variant}")
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter("cmd_topic").value), 10
        )
        self.start_time_s: float | None = None
        self.last_phase = ""
        self.create_timer(0.05, self._timer_cb)

    def _elapsed(self) -> float | None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return None
        if self.start_time_s is None:
            self.start_time_s = now
            self.get_logger().info(f"P3 trajectory clock started: {self.scenario}")
        return now - self.start_time_s

    @staticmethod
    def _smoothstep(value: float) -> float:
        clipped = min(1.0, max(0.0, value))
        return clipped * clipped * (3.0 - 2.0 * clipped)

    @classmethod
    def _trapezoid(
        cls, elapsed: float, start: float, end: float, ramp_s: float, peak: float
    ) -> float:
        """C1-continuous command that avoids non-physical IMU impulses."""
        if elapsed < start or elapsed >= end:
            return 0.0
        if elapsed < start + ramp_s:
            return peak * cls._smoothstep((elapsed - start) / ramp_s)
        if elapsed >= end - ramp_s:
            return peak * cls._smoothstep((end - elapsed) / ramp_s)
        return peak

    def _command(self, elapsed: float) -> tuple[str, float, float, float]:
        if self.variant == "validation":
            if self.scenario == "structured_room":
                if elapsed < 8.0:
                    return "validation_imu_initialization", 0.0, 0.0, 0.0
                if elapsed < 18.0:
                    return "validation_north", 0.0, self._trapezoid(elapsed, 8.0, 18.0, 2.0, 0.22), 0.0
                if elapsed < 30.0:
                    return (
                        "validation_diagonal",
                        self._trapezoid(elapsed, 18.0, 30.0, 2.0, 0.20),
                        self._trapezoid(elapsed, 18.0, 30.0, 2.0, 0.12),
                        0.0,
                    )
                if elapsed < 34.0:
                    return "validation_yaw_positive", 0.0, 0.0, self._trapezoid(elapsed, 30.0, 34.0, 1.0, 0.14)
                if elapsed < 38.0:
                    return "validation_yaw_negative", 0.0, 0.0, self._trapezoid(elapsed, 34.0, 38.0, 1.0, -0.14)
                if elapsed < 56.0:
                    return (
                        "validation_diagonal_return",
                        self._trapezoid(elapsed, 38.0, 56.0, 3.0, -0.17),
                        self._trapezoid(elapsed, 38.0, 56.0, 3.0, -0.13),
                        0.0,
                    )
                if elapsed < 64.0:
                    return "validation_east_finish", self._trapezoid(elapsed, 56.0, 64.0, 2.0, 0.15), 0.0, 0.0
            else:
                if elapsed < 8.0:
                    return "validation_imu_initialization", 0.0, 0.0, 0.0
                if elapsed < 36.0:
                    return "validation_corridor_out", self._trapezoid(elapsed, 8.0, 36.0, 3.0, 0.30), 0.0, 0.0
                if elapsed < 40.0:
                    return "validation_yaw_positive", 0.0, 0.0, self._trapezoid(elapsed, 36.0, 40.0, 1.0, 0.14)
                if elapsed < 44.0:
                    return "validation_yaw_negative", 0.0, 0.0, self._trapezoid(elapsed, 40.0, 44.0, 1.0, -0.14)
                if elapsed < 64.0:
                    return "validation_corridor_return", self._trapezoid(elapsed, 44.0, 64.0, 3.0, -0.26), 0.0, 0.0
            return "validation_complete_hold", 0.0, 0.0, 0.0
        if self.variant == "test":
            if self.scenario == "structured_room":
                if elapsed < 8.0:
                    return "test_imu_initialization", 0.0, 0.0, 0.0
                if elapsed < 20.0:
                    return (
                        "test_diagonal_out",
                        self._trapezoid(elapsed, 8.0, 20.0, 2.0, 0.28),
                        self._trapezoid(elapsed, 8.0, 20.0, 2.0, 0.20),
                        0.0,
                    )
                if elapsed < 26.0:
                    return "test_yaw_positive", 0.0, 0.0, self._trapezoid(elapsed, 20.0, 26.0, 1.5, 0.16)
                if elapsed < 40.0:
                    return (
                        "test_diagonal_return",
                        self._trapezoid(elapsed, 26.0, 40.0, 2.5, -0.24),
                        self._trapezoid(elapsed, 26.0, 40.0, 2.5, -0.17),
                        0.0,
                    )
                if elapsed < 46.0:
                    return "test_yaw_negative", 0.0, 0.0, self._trapezoid(elapsed, 40.0, 46.0, 1.5, -0.16)
                if elapsed < 58.0:
                    return "test_east", self._trapezoid(elapsed, 46.0, 58.0, 2.0, 0.22), 0.0, 0.0
            else:
                if elapsed < 8.0:
                    return "test_imu_initialization", 0.0, 0.0, 0.0
                if elapsed < 38.0:
                    return "test_corridor_out", self._trapezoid(elapsed, 8.0, 38.0, 3.0, 0.32), 0.0, 0.0
                if elapsed < 44.0:
                    return "test_yaw_positive", 0.0, 0.0, self._trapezoid(elapsed, 38.0, 44.0, 1.5, 0.16)
                if elapsed < 64.0:
                    return "test_corridor_return", self._trapezoid(elapsed, 44.0, 64.0, 3.0, -0.28), 0.0, 0.0
            return "test_complete_hold", 0.0, 0.0, 0.0
        if self.scenario == "structured_room":
            if elapsed < 8.0:
                return "imu_initialization", 0.0, 0.0, 0.0
            if elapsed < 18.0:
                return "east", self._trapezoid(elapsed, 8.0, 18.0, 2.0, 0.40), 0.0, 0.0
            if elapsed < 23.0:
                return "settle_east", 0.0, 0.0, 0.0
            if elapsed < 28.0:
                return "north", 0.0, self._trapezoid(elapsed, 23.0, 28.0, 2.0, 0.40), 0.0
            if elapsed < 33.0:
                return "settle_north", 0.0, 0.0, 0.0
            if elapsed < 43.0:
                return "west", self._trapezoid(elapsed, 33.0, 43.0, 2.0, -0.40), 0.0, 0.0
            if elapsed < 48.0:
                return "settle_west", 0.0, 0.0, 0.0
            if elapsed < 53.0:
                return "south", 0.0, self._trapezoid(elapsed, 48.0, 53.0, 2.0, -0.40), 0.0
            if elapsed < 57.0:
                return "yaw_positive", 0.0, 0.0, self._trapezoid(elapsed, 53.0, 57.0, 1.0, 0.20)
            if elapsed < 61.0:
                return "yaw_negative", 0.0, 0.0, self._trapezoid(elapsed, 57.0, 61.0, 1.0, -0.20)
        else:
            if elapsed < 8.0:
                return "imu_initialization", 0.0, 0.0, 0.0
            if elapsed < 48.0:
                return "corridor_outbound", self._trapezoid(elapsed, 8.0, 48.0, 2.0, 0.40), 0.0, 0.0
            if elapsed < 52.0:
                return "yaw_positive", 0.0, 0.0, self._trapezoid(elapsed, 48.0, 52.0, 1.0, 0.20)
            if elapsed < 56.0:
                return "yaw_negative", 0.0, 0.0, self._trapezoid(elapsed, 52.0, 56.0, 1.0, -0.20)
            if elapsed < 68.0:
                return "corridor_return", self._trapezoid(elapsed, 56.0, 68.0, 2.0, -0.40), 0.0, 0.0
        return "complete_hold", 0.0, 0.0, 0.0

    def _timer_cb(self) -> None:
        elapsed = self._elapsed()
        if elapsed is None:
            return
        phase, vx, vy, wz = self._command(elapsed)
        message = Twist()
        message.linear.x = vx
        message.linear.y = vy
        message.angular.z = wz
        self.publisher.publish(message)
        if phase != self.last_phase:
            self.get_logger().info(
                f"P3 trajectory phase={phase} t={elapsed:.2f}s "
                f"v=({vx:.2f},{vy:.2f}) yaw_rate={wz:.2f}"
            )
            self.last_phase = phase


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P3TrajectoryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            stop = Twist()
            for _ in range(3):
                node.publisher.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
