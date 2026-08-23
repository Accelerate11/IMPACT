#!/usr/bin/env python3
"""Render P6 directional protection levels as RViz markers."""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker
from xq_sim_interfaces.msg import DirectionalIntegrity


class P6ReplayVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("xq_p6_replay_visualizer")
        self.declare_parameter("visual_scale", 20.0)
        self.declare_parameter("calibration_file", "")
        self.position: tuple[float, float, float] | None = None
        calibration_file = str(self.get_parameter("calibration_file").value)
        self.calibrated_k95: dict[str, float] | None = None
        if calibration_file:
            artifact = json.loads(open(calibration_file, encoding="utf-8").read())
            self.calibrated_k95 = {
                name: float(artifact["directional"][name]["k95"])
                for name in ("x", "y", "z", "weak")
            }
            self.get_logger().info(f"P7 frozen calibration loaded: {calibration_file}")
        self.publisher = self.create_publisher(Marker, "/xq/replay/p6_integrity", 20)
        self.create_subscription(Odometry, "/localization/odom", self._odom_callback, 20)
        self.create_subscription(
            DirectionalIntegrity, "/integrity/directional", self._integrity_callback, 20
        )

    def _odom_callback(self, message: Odometry) -> None:
        point = message.pose.pose.position
        self.position = (float(point.x), float(point.y), float(point.z))

    def _base(self, message: DirectionalIntegrity, marker_id: int, kind: int) -> Marker:
        marker = Marker()
        marker.header = message.header
        marker.header.frame_id = "xq_lio_map"
        marker.ns = "xq_p6_directional_integrity"
        marker.id = marker_id
        marker.type = kind
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _integrity_callback(self, message: DirectionalIntegrity) -> None:
        if self.position is None:
            return
        x, y, z = self.position
        scale = float(self.get_parameter("visual_scale").value)
        base_k = max(float(message.k_alpha), 1.0e-12)
        axis_pl = [float(value) for value in message.protection_level_axes]
        weak_pl = float(message.weak_direction_protection_level)
        if self.calibrated_k95 is not None:
            axis_pl = [
                axis_pl[index] * self.calibrated_k95[name] / base_k
                for index, name in enumerate(("x", "y", "z"))
            ]
            weak_pl = weak_pl * self.calibrated_k95["weak"] / base_k
        colors = ((1.0, 0.25, 0.20), (0.20, 1.0, 0.35), (0.20, 0.55, 1.0))
        for axis in range(3):
            marker = self._base(message, axis, Marker.ARROW)
            marker.points = [Point(x=x, y=y, z=z), Point(x=x, y=y, z=z)]
            marker.points[1].x += scale * axis_pl[axis] * (axis == 0)
            marker.points[1].y += scale * axis_pl[axis] * (axis == 1)
            marker.points[1].z += scale * axis_pl[axis] * (axis == 2)
            marker.scale.x, marker.scale.y, marker.scale.z = 0.055, 0.11, 0.15
            marker.color.r, marker.color.g, marker.color.b = colors[axis]
            marker.color.a = 0.95
            self.publisher.publish(marker)

        weak = self._base(message, 3, Marker.ARROW)
        weak.points = [Point(x=x, y=y, z=z), Point(x=x, y=y, z=z)]
        weak_length = scale * weak_pl
        weak.points[1].x += weak_length * float(message.weak_direction_map[0])
        weak.points[1].y += weak_length * float(message.weak_direction_map[1])
        weak.points[1].z += weak_length * float(message.weak_direction_map[2])
        weak.scale.x, weak.scale.y, weak.scale.z = 0.09, 0.16, 0.20
        weak.color.r, weak.color.g, weak.color.b, weak.color.a = 1.0, 0.0, 0.85, 1.0
        self.publisher.publish(weak)

        envelope = self._base(message, 4, Marker.SPHERE)
        envelope.pose.position.x, envelope.pose.position.y, envelope.pose.position.z = x, y, z
        envelope.scale.x = 2.0 * scale * axis_pl[0]
        envelope.scale.y = 2.0 * scale * axis_pl[1]
        envelope.scale.z = 2.0 * scale * axis_pl[2]
        envelope.color.r, envelope.color.g, envelope.color.b, envelope.color.a = 0.8, 0.15, 1.0, 0.16
        self.publisher.publish(envelope)

        text = self._base(message, 5, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = x, y, z + 1.3
        text.scale.z = 0.34
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
        phase = "P7 PL95" if self.calibrated_k95 is not None else "P6"
        text.text = f"{phase}  PLweak={weak_pl:.3f} m  cond={message.condition_number:.2f}  N={message.effective_points}"
        self.publisher.publish(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P6ReplayVisualizer()
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
