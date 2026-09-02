"""RViz overlay for P13 latency, AL, margin and applied speed envelope."""

from __future__ import annotations

import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class P13ReplayVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p13_replay_visualizer")
        self.trace: dict[str, object] = {}
        self.position_x = -12.0
        self.publisher = self.create_publisher(Marker, "/xq/replay/p13_latency", 20)
        self.create_subscription(String, "/integrity/p13/latency_trace", self._trace_cb, 100)
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, 30)
        self.create_timer(0.20, self._publish)

    def _trace_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if payload.get("ground_truth_used") is False:
            self.trace = payload

    def _odom_cb(self, message: Odometry) -> None:
        self.position_x = float(message.pose.pose.position.x)

    def _base(self, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "xq_lio_map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "p13_latency_safety"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _publish(self) -> None:
        if not self.trace:
            return
        profile = str(self.trace.get("profile", ""))
        high = profile.startswith("high")
        color = (1.0, 0.25, 0.08) if high else (0.12, 0.95, 0.30)
        text = self._base(0, Marker.TEXT_VIEW_FACING)
        text.pose.position.x = self.position_x
        text.pose.position.y = -1.80
        text.pose.position.z = 2.55
        text.scale.z = 0.30
        text.color.r, text.color.g, text.color.b, text.color.a = (*color, 1.0)
        text.text = (
            f"P13 {profile} | p99={float(self.trace['p99_ms']):.1f} ms\n"
            f"AL(raw)={float(self.trace['unmitigated_alert_limit_m']):.3f} m  "
            f"AL(safe)={float(self.trace['alert_limit_m']):.3f} m\n"
            f"M={float(self.trace['integrity_margin_m']):.3f} m  "
            f"v<= {float(self.trace['speed_limit_mps']):.3f} m/s"
        )
        self.publisher.publish(text)

        speed = max(0.0, float(self.trace["speed_limit_mps"]))
        bar = self._base(1, Marker.CUBE)
        bar.pose.position.x = self.position_x
        bar.pose.position.y = -1.80
        bar.pose.position.z = 2.18
        bar.scale.x = max(0.02, 2.0 * speed)
        bar.scale.y = 0.12
        bar.scale.z = 0.12
        bar.color.r, bar.color.g, bar.color.b, bar.color.a = (*color, 0.9)
        self.publisher.publish(bar)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P13ReplayVisualizerNode()
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
