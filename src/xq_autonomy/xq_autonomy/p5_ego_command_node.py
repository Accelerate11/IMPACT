"""Safety-gated EGO PositionCommand -> MAVROS Guided position adapter."""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class P5EgoCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p5_ego_command")
        self.declare_parameter("maximum_tracking_error_m", 2.5)
        self.enabled = False
        self.have_odom = False
        self.current = (0.0, 0.0, 0.0)
        self.received = 0
        self.forwarded = 0
        self.rejected = 0
        self.last_command_wall = 0.0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(
            PoseStamped, "/uav1/mavros/setpoint_position/local", 30
        )
        self.status_pub = self.create_publisher(String, "/xq/p5/ego_adapter/status", 10)
        self.create_subscription(PositionCommand, "/position_cmd", self._command_cb, 50)
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, qos)
        self.create_subscription(Bool, "/xq/p5/exploration/enable", self._enable_cb, 10)
        self.create_timer(1.0, self._status_cb)
        self.get_logger().info("P5 EGO command adapter is safety-gated until takeoff completes")

    def _enable_cb(self, message: Bool) -> None:
        self.enabled = bool(message.data)

    def _odom_cb(self, message: Odometry) -> None:
        p = message.pose.pose.position
        values = (float(p.x), float(p.y), float(p.z))
        if all(math.isfinite(value) for value in values):
            self.current = values
            self.have_odom = True

    def _command_cb(self, source: PositionCommand) -> None:
        self.received += 1
        values = (
            float(source.position.x),
            float(source.position.y),
            float(source.position.z),
            float(source.yaw),
        )
        if not self.enabled or not self.have_odom:
            return
        distance = math.sqrt(sum((values[i] - self.current[i]) ** 2 for i in range(3)))
        if (
            not all(math.isfinite(value) for value in values)
            or distance > float(self.get_parameter("maximum_tracking_error_m").value)
            or not 0.35 <= values[2] <= 3.4
        ):
            self.rejected += 1
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position = source.position
        message.pose.orientation.z = math.sin(values[3] / 2.0)
        message.pose.orientation.w = math.cos(values[3] / 2.0)
        self.publisher.publish(message)
        self.forwarded += 1
        self.last_command_wall = time.monotonic()

    def _status_cb(self) -> None:
        age = time.monotonic() - self.last_command_wall if self.last_command_wall else None
        message = String()
        message.data = json.dumps(
            {
                "schema_version": 1,
                "enabled": self.enabled,
                "healthy": self.enabled and self.forwarded > 0 and age is not None and age < 0.5,
                "received": self.received,
                "forwarded": self.forwarded,
                "rejected": self.rejected,
                "last_forward_age_s": age,
                "ground_truth_subscribed": False,
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P5EgoCommandNode()
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
