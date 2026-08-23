"""Persistent RViz markers for P9 wide ACCEPT / narrow REJECT evidence."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker
from xq_sim_interfaces.msg import IntegrityMargin

from .alert_limit import sample_bspline


class P9ReplayVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p9_replay_visualizer")
        self.publisher = self.create_publisher(Marker, "/xq/replay/p6_integrity", 100)
        self.create_subscription(
            Bspline, "/planning/candidate_bspline", self._trajectory_cb, 20
        )
        self.create_subscription(IntegrityMargin, "/integrity/margin", self._margin_cb, 50)
        self.create_timer(1.0, self._publish_environment)

    def _base(self, namespace: str, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "xq_lio_map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _publish_environment(self) -> None:
        definitions = (
            (0, 0.0, 2.5, 8.0, 0.12, 0.25, 0.85, 0.38),
            (1, 0.0, 5.5, 8.0, 0.12, 0.25, 0.85, 0.38),
            (2, -4.0, 4.0, 0.12, 3.0, 0.25, 0.85, 0.38),
            (3, 4.0, 4.0, 0.12, 3.0, 0.25, 0.85, 0.38),
            (4, 0.0, -4.6, 8.0, 0.12, 0.95, 0.18, 0.12),
            (5, 0.0, -3.4, 8.0, 0.12, 0.95, 0.18, 0.12),
            (6, -4.0, -4.0, 0.12, 1.2, 0.95, 0.18, 0.12),
            (7, 4.0, -4.0, 0.12, 1.2, 0.95, 0.18, 0.12),
        )
        for marker_id, x, y, sx, sy, red, green, blue in definitions:
            wall = self._base("xq_p9_environment", marker_id, Marker.CUBE)
            wall.pose.position.x, wall.pose.position.y, wall.pose.position.z = x, y, 1.2
            wall.scale.x, wall.scale.y, wall.scale.z = sx, sy, 2.4
            wall.color.r, wall.color.g, wall.color.b, wall.color.a = red, green, blue, 0.42
            self.publisher.publish(wall)

    def _trajectory_cb(self, message: Bspline) -> None:
        try:
            import numpy as np
            samples = sample_bspline(
                np.asarray([(p.x, p.y, p.z) for p in message.pos_pts]),
                np.asarray(message.knots), int(message.order), 0.10,
            )
        except ValueError:
            return
        accepted = int(message.traj_id) == 9001
        line = self._base("xq_p9_integrity_margin", int(message.traj_id) * 10, Marker.LINE_STRIP)
        line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in samples]
        line.scale.x = 0.10
        line.color.r = 0.05 if accepted else 1.0
        line.color.g = 1.0 if accepted else 0.08
        line.color.b, line.color.a = 0.18, 1.0
        self.publisher.publish(line)

    def _margin_cb(self, message: IntegrityMargin) -> None:
        if not message.valid:
            return
        accepted = bool(message.accepted)
        base_id = int(message.trajectory_id) * 10
        arrow = self._base("xq_p9_integrity_margin", base_id + 1, Marker.ARROW)
        arrow.points = [message.critical_sample_point, message.nearest_obstacle_point]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.05, 0.10, 0.14
        arrow.color.r = 0.05 if accepted else 1.0
        arrow.color.g = 1.0 if accepted else 0.08
        arrow.color.b, arrow.color.a = 0.18, 1.0
        self.publisher.publish(arrow)

        envelope = self._base("xq_p9_integrity_margin", base_id + 2, Marker.SPHERE)
        envelope.pose.position = message.critical_sample_point
        diameter = max(0.05, 2.0 * float(message.protection_level))
        envelope.scale.x = envelope.scale.y = envelope.scale.z = diameter
        envelope.color.r = 0.12 if accepted else 1.0
        envelope.color.g = 0.65 if accepted else 0.12
        envelope.color.b, envelope.color.a = 1.0, 0.34
        self.publisher.publish(envelope)

        text = self._base("xq_p9_integrity_margin", base_id + 3, Marker.TEXT_VIEW_FACING)
        text.pose.position = message.critical_sample_point
        text.pose.position.z += 0.70
        text.scale.z = 0.30
        text.color.r = 0.05 if accepted else 1.0
        text.color.g = 1.0 if accepted else 0.10
        text.color.b, text.color.a = 0.18, 1.0
        decision = "ACCEPT" if accepted else "REJECT"
        scenario = "WIDE ROOM" if int(message.trajectory_id) == 9001 else "NARROW PASSAGE"
        text.text = (
            f"P9 {scenario}: {decision}\n"
            f"AL={message.alert_limit:.3f}  PL={message.protection_level:.3f}  "
            f"Mmin={message.minimum_margin:.3f}  reserve={message.margin_reserve:.3f} m"
        )
        self.publisher.publish(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P9ReplayVisualizerNode()
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
