"""Evaluate the P11 online ROS selection/transport contract."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import IntegrityExplorationDecision


class P11GateEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p11_gate_evaluator")
        self.declare_parameter("result_file", "")
        self.decision: IntegrityExplorationDecision | None = None
        self.selected_ids: list[int] = []
        self.unconstrained_ids: list[int] = []
        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            IntegrityExplorationDecision,
            "/integrity/exploration_decision",
            self._decision_cb,
            latched,
        )
        self.create_subscription(Bspline, "/planning/p11/selected_bspline", self._selected_cb, latched)
        self.create_subscription(
            Bspline, "/planning/p11/unconstrained_bspline", self._unconstrained_cb, latched
        )
        self.create_timer(0.20, self._write)

    def _decision_cb(self, message: IntegrityExplorationDecision) -> None:
        self.decision = message

    def _selected_cb(self, message: Bspline) -> None:
        self.selected_ids.append(int(message.traj_id))

    def _unconstrained_cb(self, message: Bspline) -> None:
        self.unconstrained_ids.append(int(message.traj_id))

    def _snapshot(self) -> dict:
        if self.decision is None:
            return {"schema_version": 1, "gate": "P11_ROS_CONTRACT", "status": "IN_PROGRESS"}
        message = self.decision
        names = list(message.candidate_names)
        groups = (
            message.trajectory_ids,
            message.information_gains,
            message.travel_times_s,
            message.energy_costs,
            message.collision_probabilities,
            message.utilities,
            message.predicted_minimum_margins,
            message.integrity_feasible,
            message.collision_feasible,
            message.energy_feasible,
            message.feasible,
        )
        aligned = bool(names) and all(len(group) == len(names) for group in groups)
        index = {name: names.index(name) for name in names}
        required = {
            "high_information_direct",
            "geometry_rich_right",
            "collision_violation",
            "return_energy_violation",
        }
        have_required = required.issubset(index)
        direct = index.get("high_information_direct", -1)
        safe = index.get("geometry_rich_right", -1)
        collision = index.get("collision_violation", -1)
        energy = index.get("return_energy_violation", -1)
        checks = {
            "valid_hard_decision": bool(message.valid and message.hard_constraint),
            "margin_absent_from_utility": not message.margin_in_utility,
            "profiles_aligned": aligned,
            "required_candidates_present": have_required,
            "unconstrained_prefers_high_information": message.unconstrained_selected_name
            == "high_information_direct",
            "integrity_selects_geometry_rich": message.selected_name == "geometry_rich_right",
            "direct_utility_higher": have_required and message.utilities[direct] > message.utilities[safe],
            "direct_integrity_rejected": have_required and not message.integrity_feasible[direct],
            "safe_all_hard_feasible": have_required and message.feasible[safe],
            "collision_hard_rejected": have_required and not message.collision_feasible[collision],
            "energy_hard_rejected": have_required and not message.energy_feasible[energy],
            "selected_trajectory_published": int(message.selected_trajectory_id) in self.selected_ids,
            "unconstrained_trajectory_published": int(message.unconstrained_selected_trajectory_id)
            in self.unconstrained_ids,
            "ground_truth_not_required": True,
        }
        return {
            "schema_version": 1,
            "gate": "P11_ROS_CONTRACT",
            "status": "PASS" if all(checks.values()) else "FAIL",
            "scope": "algorithm_and_transport_contract_only_not_formal_gazebo_flight_gate",
            "candidate_names": names,
            "utilities": list(message.utilities),
            "predicted_minimum_margins": list(message.predicted_minimum_margins),
            "integrity_feasible": list(message.integrity_feasible),
            "collision_feasible": list(message.collision_feasible),
            "energy_feasible": list(message.energy_feasible),
            "feasible": list(message.feasible),
            "unconstrained_selected_name": message.unconstrained_selected_name,
            "selected_name": message.selected_name,
            "reason": message.reason,
            "checks": checks,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        }

    def _write(self) -> None:
        path = Path(str(self.get_parameter("result_file").value))
        if str(path) in ("", "."):
            return
        result = self._snapshot()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P11GateEvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._write()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
