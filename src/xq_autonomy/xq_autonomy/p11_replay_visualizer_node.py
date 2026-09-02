"""Persistent RViz markers for P11 utility and integrity-constrained choices."""

from __future__ import annotations

import json

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker
from xq_sim_interfaces.msg import InformationMap, IntegrityExplorationDecision

from .alert_limit import sample_bspline


class P11ReplayVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p11_replay_visualizer")
        self.declare_parameter("show_complex_demo_overlay", False)
        self.declare_parameter("complex_scenario_name", "xq_complex_warehouse")
        self.publisher = self.create_publisher(Marker, "/xq/replay/p6_integrity", 200)
        self.candidates: dict[int, Bspline] = {}
        self.decision: IntegrityExplorationDecision | None = None
        self.decisions: dict[int, IntegrityExplorationDecision] = {}
        self.information_map: InformationMap | None = None
        self.flight_status: dict[str, object] = {}
        self.map_status: dict[str, object] = {}
        self.create_subscription(
            Bspline, "/planning/p11/frontier_candidates", self._candidate_cb, 50
        )
        self.create_subscription(
            IntegrityExplorationDecision,
            "/integrity/exploration_decision",
            self._decision_cb,
            20,
        )
        self.create_subscription(InformationMap, "/integrity/information_map", self._map_cb, 20)
        self.create_subscription(String, "/xq/p12/flight_status", self._flight_status_cb, 20)
        self.create_subscription(String, "/mapping/p12/status", self._map_status_cb, 20)
        self.create_timer(0.5, self._publish)

    def _candidate_cb(self, message: Bspline) -> None:
        self.candidates[int(message.traj_id)] = message

    def _decision_cb(self, message: IntegrityExplorationDecision) -> None:
        self.decision = message
        self.decisions[int(message.batch_id)] = message

    def _map_cb(self, message: InformationMap) -> None:
        self.information_map = message

    def _flight_status_cb(self, message: String) -> None:
        try:
            self.flight_status = json.loads(message.data)
        except json.JSONDecodeError:
            return

    def _map_status_cb(self, message: String) -> None:
        try:
            self.map_status = json.loads(message.data)
        except json.JSONDecodeError:
            return

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
        if not self.decisions:
            return
        for decision in self.decisions.values():
            by_id = {
                int(value): index for index, value in enumerate(decision.trajectory_ids)
            }
            for trajectory_id in decision.trajectory_ids:
                message = self.candidates.get(int(trajectory_id))
                index = by_id.get(int(trajectory_id))
                if message is None or index is None:
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
                selected = int(trajectory_id) == int(decision.selected_trajectory_id)
                unconstrained = int(trajectory_id) == int(
                    decision.unconstrained_selected_trajectory_id
                )
                line = self._base(
                    "xq_p11_frontier_candidates", int(trajectory_id), Marker.LINE_STRIP
                )
                line.points = [
                    Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in samples
                ]
                line.scale.x = 0.12 if selected else 0.075 if unconstrained else 0.035
                if selected:
                    line.color.r, line.color.g, line.color.b = 0.05, 1.0, 0.18
                elif unconstrained:
                    line.color.r, line.color.g, line.color.b = 1.0, 0.08, 0.05
                elif decision.feasible[index]:
                    line.color.r, line.color.g, line.color.b = 0.10, 0.72, 1.0
                else:
                    line.color.r, line.color.g, line.color.b = 1.0, 0.55, 0.05
                line.color.a = 1.0
                self.publisher.publish(line)

        interventions = [
            item
            for item in self.decisions.values()
            if item.unconstrained_selected_name != item.selected_name
        ]
        decision = interventions[0] if interventions else self.decision
        if decision is None:
            return
        direct = int(decision.unconstrained_selected_index)
        selected = int(decision.selected_index)
        if not (0 <= direct < len(decision.candidate_names) and 0 <= selected < len(decision.candidate_names)):
            return
        text = self._base("xq_p11_decision", 1, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = 4.0, -0.2, 2.4
        text.scale.z = 0.31
        text.color.r, text.color.g, text.color.b, text.color.a = 0.95, 0.98, 1.0, 1.0
        text.text = (
            f"P11 ROLLING HARD SELECTION ({len(self.decisions)} batches)\n"
            f"RED utility-only: {decision.unconstrained_selected_name}  "
            f"U={decision.utilities[direct]:.3f}  Mmin={decision.predicted_minimum_margins[direct]:.3f} m\n"
            f"GREEN selected: {decision.selected_name}  "
            f"U={decision.utilities[selected]:.3f}  Mmin={decision.predicted_minimum_margins[selected]:.3f} m"
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
        lines = self._base("xq_p11_online_surfel_normals", 0, Marker.LINE_LIST)
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

    def _publish_complex_demo_overlay(self) -> None:
        if not bool(self.get_parameter("show_complex_demo_overlay").value):
            return

        # Gazebo starts at world x=-12 m while FAST-LIO starts at x=0 m.
        # These labels therefore use mission-relative positions in xq_lio_map.
        scenario = str(self.get_parameter("complex_scenario_name").value)
        bidirectional = scenario == "xq_complex_bidirectional_warehouse"
        spatial = scenario == "xq_complex_3d_warehouse"
        compositional = scenario == "xq_complex_compositional_warehouse"
        zones = (
            (10, 3.75, "A  HARD GATE -> RIGHT", (1.0, 0.22, 0.08)),
            (11, 7.50, "B  DYNAMIC CROSSING", (1.0, 0.67, 0.05)),
            (
                12,
                11.0,
                "C  HARD GATE -> LEFT"
                if bidirectional or spatial or compositional
                else "C  HARD GATE -> RIGHT",
                (0.10, 0.35, 1.0)
                if bidirectional or spatial or compositional
                else (1.0, 0.22, 0.08),
            ),
            (
                13,
                20.0 if spatial or compositional else 18.2,
                "D  COMBINED 3D -> UP+RIGHT"
                if compositional
                else "D  3D HARD GATE -> UP"
                if spatial
                else "D  X-NORMAL BARCODE",
                (0.92, 0.18, 0.88)
                if compositional
                else (0.72, 0.22, 1.0)
                if spatial
                else (0.10, 0.78, 1.0),
            ),
            (14, 22.5, "E  RESTORE DIRECT", (0.10, 0.95, 0.35))
            if spatial or compositional
            else None,
        )
        for zone in zones:
            if zone is None:
                continue
            marker_id, position_x, label, color = zone
            text = self._base("xq_complex_demo_zones", marker_id, Marker.TEXT_VIEW_FACING)
            text.pose.position.x = position_x
            text.pose.position.y = 2.75
            text.pose.position.z = 2.15
            text.scale.z = 0.30
            text.color.r, text.color.g, text.color.b, text.color.a = (*color, 1.0)
            text.text = label
            self.publisher.publish(text)

        goal = self._base("xq_complex_demo_goal", 20, Marker.CYLINDER)
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = 24.0, 0.0, 0.04
        goal.scale.x, goal.scale.y, goal.scale.z = 1.0, 1.0, 0.08
        goal.color.r, goal.color.g, goal.color.b, goal.color.a = 0.10, 1.0, 0.30, 0.80
        self.publisher.publish(goal)
        goal_text = self._base("xq_complex_demo_goal", 21, Marker.TEXT_VIEW_FACING)
        goal_text.pose.position.x, goal_text.pose.position.y, goal_text.pose.position.z = (
            24.0,
            0.0,
            1.5,
        )
        goal_text.scale.z = 0.36
        goal_text.color.r, goal_text.color.g, goal_text.color.b, goal_text.color.a = (
            0.10,
            1.0,
            0.30,
            1.0,
        )
        goal_text.text = "24 m MISSION GOAL"
        self.publisher.publish(goal_text)

        interventions = sum(
            item.unconstrained_selected_name != item.selected_name
            for item in self.decisions.values()
        )
        intervention_decisions = [
            item
            for item in self.decisions.values()
            if item.unconstrained_selected_name != item.selected_name
        ]
        advantage = "waiting for comparable candidates"
        if intervention_decisions:
            comparison = intervention_decisions[-1]
            baseline_index = int(comparison.unconstrained_selected_index)
            selected_index = int(comparison.selected_index)
            if (
                0 <= baseline_index < len(comparison.predicted_minimum_margins)
                and 0 <= selected_index < len(comparison.predicted_minimum_margins)
            ):
                advantage = (
                    f"baseline {comparison.unconstrained_selected_name}: "
                    f"M={comparison.predicted_minimum_margins[baseline_index]:+.3f} m  ->  "
                    f"IMPACT {comparison.selected_name}: "
                    f"M={comparison.predicted_minimum_margins[selected_index]:+.3f} m"
                )
        flight = self.flight_status
        mapping = self.map_status
        current_x = float(flight.get("current_position_x_m") or 0.0)
        segments = int(flight.get("segments_completed") or 0)
        replans = int(flight.get("replan_event_count") or 0)
        phase = str(flight.get("phase") or "INITIALIZING")
        blocked = bool(mapping.get("forward_path_blocked", False))
        dynamic_state = "BLOCKED - SAFE BRAKE" if blocked else "CLEAR / TTL VERIFIED"
        board = self._base("xq_complex_demo_status", 30, Marker.TEXT_VIEW_FACING)
        board.pose.position.x = min(max(current_x + 1.0, 2.0), 22.0)
        board.pose.position.y = 3.65
        board.pose.position.z = 2.85
        board.scale.z = 0.34
        board.color.r, board.color.g, board.color.b, board.color.a = 0.95, 0.98, 1.0, 1.0
        board.text = (
            "FULL NORMAL AUTONOMY - NO FAULT INJECTION\n"
            f"scenario={scenario}\n"
            f"phase={phase} | segments={segments}/4 | integrity interventions={interventions}\n"
            f"dynamic map={dynamic_state} | certified replans={replans}\n"
            f"{advantage}"
        )
        self.publisher.publish(board)

    def _publish(self) -> None:
        self._publish_candidates()
        self._publish_normals()
        self._publish_complex_demo_overlay()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P11ReplayVisualizerNode()
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
