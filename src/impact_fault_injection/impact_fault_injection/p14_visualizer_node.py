"""RViz markers for active P14 fault and resilient-autonomy state."""

from __future__ import annotations

import json

from geometry_msgs.msg import Point
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


COLORS = {
    "NORMAL": (0.10, 0.90, 0.25), "CAUTIOUS": (1.0, 0.72, 0.05),
    "RECOVERY": (0.10, 0.65, 1.0), "BRAKE": (1.0, 0.20, 0.05),
    "HOVER": (0.90, 0.10, 0.85), "RETURN": (0.45, 0.35, 1.0),
    "LAND": (1.0, 1.0, 1.0), "MANUAL": (0.70, 0.70, 0.70),
}


class P14VisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("impact_p14_visualizer")
        self.status: dict[str, object] = {}
        self.position = (-12.0, 0.0, 1.0)
        self.publisher = self.create_publisher(MarkerArray, "/impact/p14/markers", 20)
        self.create_subscription(String, "/impact/p14/safety_status", self._status_cb, 100)
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, 30)
        self.create_timer(0.10, self._publish)

    def _status_cb(self, message: String) -> None:
        try:
            self.status = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def _odom_cb(self, message: Odometry) -> None:
        p = message.pose.pose.position
        self.position = (float(p.x), float(p.y), float(p.z))

    def _base(self, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "xq_lio_map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "impact_p14"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _publish(self) -> None:
        if not self.status:
            return
        mode = str(self.status.get("mode", "NORMAL"))
        color = COLORS.get(mode, (1.0, 1.0, 1.0))
        faults = self.status.get("active_fault_ids", [])
        x, y, z = self.position
        output = MarkerArray()

        halo = self._base(0, Marker.SPHERE)
        halo.pose.position = Point(x=x, y=y, z=z)
        halo.scale.x = halo.scale.y = halo.scale.z = 0.85
        halo.color.r, halo.color.g, halo.color.b, halo.color.a = (*color, 0.32)
        output.markers.append(halo)

        text = self._base(1, Marker.TEXT_VIEW_FACING)
        text.pose.position = Point(x=x, y=y - 1.4, z=z + 1.25)
        text.scale.z = 0.32
        text.color.r, text.color.g, text.color.b, text.color.a = (*color, 1.0)
        text.text = (
            f"P14 {mode} | {self.status.get('reason', '')}\n"
            f"faults: {', '.join(faults) if faults else 'none'}\n"
            f"geometry_only={self.status.get('geometry_only', False)}  "
            f"essential_only={self.status.get('essential_only', False)}"
        )
        output.markers.append(text)

        modes = ["NORMAL", "CAUTIOUS", "RECOVERY", "BRAKE", "HOVER", "RETURN", "LAND"]
        for index, value in enumerate(modes):
            cell = self._base(10 + index, Marker.CUBE)
            cell.pose.position = Point(x=-10.8 + index * 0.72, y=-3.25, z=0.18)
            cell.scale.x, cell.scale.y, cell.scale.z = 0.62, 0.32, 0.12
            rgb = COLORS[value]
            cell.color.r, cell.color.g, cell.color.b = rgb
            cell.color.a = 1.0 if value == mode else 0.22
            output.markers.append(cell)
            label = self._base(30 + index, Marker.TEXT_VIEW_FACING)
            label.pose.position = Point(x=-10.8 + index * 0.72, y=-3.25, z=0.48)
            label.scale.z = 0.15
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = value
            output.markers.append(label)
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P14VisualizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
