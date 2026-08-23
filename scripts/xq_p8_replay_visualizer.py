#!/usr/bin/env python3
"""Render P8 critical trajectory sample, nearest obstacle and Alert Limit."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker
from xq_sim_interfaces.msg import AlertLimit


class P8ReplayVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("xq_p8_replay_visualizer")
        self.publisher = self.create_publisher(Marker, "/xq/replay/p6_integrity", 20)
        self.create_subscription(AlertLimit, "/integrity/alert_limit", self._callback, 20)

    @staticmethod
    def _base(message: AlertLimit, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header = message.header
        marker.header.frame_id = "xq_lio_map"
        marker.ns = "xq_p8_alert_limit"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _callback(self, message: AlertLimit) -> None:
        critical = message.critical_sample_point
        obstacle = message.nearest_obstacle_point
        positive = message.alert_limit >= 0.0

        line = self._base(message, 0, Marker.ARROW)
        line.points = [
            Point(x=critical.x, y=critical.y, z=critical.z),
            Point(x=obstacle.x, y=obstacle.y, z=obstacle.z),
        ]
        line.scale.x, line.scale.y, line.scale.z = 0.045, 0.09, 0.12
        line.color.r = 0.15 if positive else 1.0
        line.color.g = 1.0 if positive else 0.12
        line.color.b, line.color.a = 0.25, 1.0
        self.publisher.publish(line)

        sample = self._base(message, 1, Marker.SPHERE)
        sample.pose.position = critical
        sample.scale.x = sample.scale.y = sample.scale.z = 0.20
        sample.color.r, sample.color.g, sample.color.b, sample.color.a = 0.1, 0.65, 1.0, 0.95
        self.publisher.publish(sample)

        nearest = self._base(message, 2, Marker.SPHERE)
        nearest.pose.position = obstacle
        nearest.scale.x = nearest.scale.y = nearest.scale.z = 0.16
        nearest.color.r, nearest.color.g, nearest.color.b, nearest.color.a = 1.0, 0.55, 0.05, 1.0
        self.publisher.publish(nearest)

        text = self._base(message, 3, Marker.TEXT_VIEW_FACING)
        text.pose.position.x = critical.x
        text.pose.position.y = critical.y
        text.pose.position.z = critical.z + 0.55
        text.scale.z = 0.30
        text.color.r = 0.25 if positive else 1.0
        text.color.g = 1.0 if positive else 0.2
        text.color.b, text.color.a = 0.25, 1.0
        state = "BUDGET AVAILABLE" if positive else "NO ERROR BUDGET"
        text.text = (
            f"P8 {state}  AL={message.alert_limit:.3f} m  "
            f"d={message.geometric_clearance:.3f} m  rlat={message.latency_reserve:.3f} m"
        )
        self.publisher.publish(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P8ReplayVisualizer()
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
