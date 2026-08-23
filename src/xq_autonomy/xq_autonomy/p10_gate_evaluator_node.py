"""P10 ROS-contract evaluator; formal long-corridor flight metrics come later."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from traj_utils.msg import Bspline
from xq_sim_interfaces.msg import ActivePerceptionDecision


class P10GateEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p10_gate_evaluator")
        self.declare_parameter("result_file", "")
        self.decision: ActivePerceptionDecision | None = None
        self.selected_ids: list[int] = []
        decision_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ActivePerceptionDecision, "/integrity/active_perception_decision",
            self._decision_cb, decision_qos,
        )
        self.create_subscription(
            Bspline, "/planning/active_perception_bspline", self._selected_cb, decision_qos
        )
        self.create_timer(0.20, self._write)

    def _decision_cb(self, message: ActivePerceptionDecision) -> None:
        self.decision = message

    def _selected_cb(self, message: Bspline) -> None:
        self.selected_ids.append(int(message.traj_id))

    def _snapshot(self) -> dict:
        if self.decision is None:
            return {"schema_version": 1, "gate": "P10_ROS_CONTRACT", "status": "IN_PROGRESS"}
        message = self.decision
        names = list(message.candidate_names)
        margins = list(message.predicted_minimum_margins)
        costs = list(message.costs)
        feasible = list(message.feasible)
        selected_index = int(message.selected_index)
        aligned = len(names) > 0 and len({len(names), len(margins), len(costs), len(feasible)}) == 1
        feasible_costs = [cost for cost, is_feasible in zip(costs, feasible) if is_feasible]
        checks = {
            "valid_hard_decision": bool(message.valid and message.hard_constraint),
            "profiles_aligned": aligned,
            "baseline_present": "baseline" in names,
            "baseline_insufficient": bool(message.baseline_insufficient),
            "recovery_found": bool(message.recovery_found),
            "selected_left_lateral": message.selected_name == "left_lateral",
            "selected_index_valid": aligned and 0 <= selected_index < len(names),
            "selected_is_feasible": aligned and 0 <= selected_index < len(feasible) and feasible[selected_index],
            "selected_has_minimum_feasible_cost": (
                aligned and 0 <= selected_index < len(costs) and bool(feasible_costs)
                and abs(costs[selected_index] - min(feasible_costs)) <= 1.0e-12
            ),
            "selected_trajectory_published": int(message.selected_trajectory_id) in self.selected_ids,
            "ground_truth_not_required": True,
        }
        status = "PASS" if all(checks.values()) else "FAIL"
        return {
            "schema_version": 1,
            "gate": "P10_ROS_CONTRACT",
            "status": status,
            "scope": "algorithm_and_transport_contract_only_not_formal_long_corridor_flight_gate",
            "candidate_names": names,
            "predicted_minimum_margins": margins,
            "costs": costs,
            "information_traces": list(message.information_traces),
            "feasible": feasible,
            "selected_name": message.selected_name,
            "selected_trajectory_id": int(message.selected_trajectory_id),
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
    node = P10GateEvaluatorNode()
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
