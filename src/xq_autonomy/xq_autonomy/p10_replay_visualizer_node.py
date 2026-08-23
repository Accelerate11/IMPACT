"""Persistent RViz markers for P10 candidates, surfels and hard selection."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker
from xq_sim_interfaces.msg import ActivePerceptionDecision, InformationMap

from .alert_limit import sample_bspline


class P10ReplayVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p10_replay_visualizer")
        self.publisher = self.create_publisher(Marker, "/xq/replay/p6_integrity", 200)
        self.candidates: dict[int, Bspline] = {}
        self.decision: ActivePerceptionDecision | None = None
        self.information_map: InformationMap | None = None
        self.create_subscription(
            Bspline, "/planning/active_perception_candidates", self._candidate_cb, 50
        )
        self.create_subscription(
            ActivePerceptionDecision,
            "/integrity/active_perception_decision",
            self._decision_cb,
            20,
        )
        self.create_subscription(
            InformationMap, "/integrity/information_map", self._map_cb, 20
        )
        self.create_timer(0.5, self._publish)

    def _candidate_cb(self, message: Bspline) -> None:
        self.candidates[int(message.traj_id)] = message

    def _decision_cb(self, message: ActivePerceptionDecision) -> None:
        self.decision = message

    def _map_cb(self, message: InformationMap) -> None:
        self.information_map = message

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

    def _publish_candidates(self) -> None:
        if self.decision is None:
            return
        by_id = {
            int(trajectory_id): index
            for index, trajectory_id in enumerate(self.decision.candidate_trajectory_ids)
        }
        for trajectory_id, message in self.candidates.items():
            index = by_id.get(trajectory_id)
            if index is None:
                continue
            try:
                samples = sample_bspline(
                    np.asarray([(p.x, p.y, p.z) for p in message.pos_pts]),
                    np.asarray(message.knots),
                    int(message.order),
                    0.10,
                )
            except ValueError:
                continue
            selected = trajectory_id == int(self.decision.selected_trajectory_id)
            feasible = bool(self.decision.feasible[index])
            line = self._base("xq_p10_candidates", trajectory_id, Marker.LINE_STRIP)
            line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in samples]
            line.scale.x = 0.12 if selected else 0.035
            if selected:
                line.color.r, line.color.g, line.color.b = 0.05, 1.0, 0.18
            elif feasible:
                line.color.r, line.color.g, line.color.b = 0.10, 0.72, 1.0
            else:
                line.color.r, line.color.g, line.color.b = 1.0, 0.12, 0.08
            line.color.a = 1.0 if selected else 0.55
            self.publisher.publish(line)

        text = self._base("xq_p10_decision", 1, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = 3.5, 0.0, 2.2
        text.scale.z = 0.34
        text.color.r, text.color.g, text.color.b, text.color.a = 0.15, 1.0, 0.25, 1.0
        selected_index = int(self.decision.selected_index)
        selected_margin = (
            float(self.decision.predicted_minimum_margins[selected_index])
            if 0 <= selected_index < len(self.decision.predicted_minimum_margins)
            else float("nan")
        )
        baseline_margin = (
            float(self.decision.predicted_minimum_margins[0])
            if self.decision.predicted_minimum_margins else float("nan")
        )
        text.text = (
            f"P10 HARD RECOVERY: {self.decision.selected_name}\n"
            f"baseline Mmin={baseline_margin:.3f} m -> selected Mmin={selected_margin:.3f} m"
        )
        self.publisher.publish(text)

    def _publish_normals(self) -> None:
        message = self.information_map
        if message is None or not message.valid:
            return
        count = min(len(message.positions), len(message.normals), 120)
        if count == 0:
            return
        stride = max(1, len(message.positions) // count)
        lines = self._base("xq_p10_surfel_normals", 0, Marker.LINE_LIST)
        lines.scale.x = 0.018
        lines.color.r, lines.color.g, lines.color.b, lines.color.a = 0.20, 0.62, 1.0, 0.80
        for index in range(0, len(message.positions), stride):
            if len(lines.points) >= 2 * count:
                break
            position = message.positions[index]
            normal = message.normals[index]
            lines.points.append(position)
            lines.points.append(
                Point(
                    x=position.x + 0.25 * normal.x,
                    y=position.y + 0.25 * normal.y,
                    z=position.z + 0.25 * normal.z,
                )
            )
        self.publisher.publish(lines)

    def _publish(self) -> None:
        self._publish_candidates()
        self._publish_normals()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P10ReplayVisualizerNode()
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
