from __future__ import annotations

import json
import hashlib
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

from xq_sim_interfaces.msg import (
    FaultEvent,
    HealthStatus,
    LocalizationQuality,
    ReplanEvent,
)

from .metrics import (
    append_state_transition,
    evaluate_fault_response,
    fault_evidence_summary,
    frequency_metrics,
    replan_metrics,
    trajectory_metrics,
)


def _stamp_s(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9


class XqMetricsNode(Node):
    """Collect Gazebo SIL evidence while keeping truth under /xq/eval only."""

    def __init__(self) -> None:
        super().__init__("xq_metrics_node")
        self.declare_parameter("run_dir", "/home/accelerate/xuanqiong_x1_sim_ws/runs/latest")
        self.declare_parameter("scenario", "xq_indoor_office")
        self.declare_parameter("seed", 20260820)
        self.declare_parameter("spec_sha256", "UNSET")
        self.declare_parameter("world_sha256", "UNSET")
        self.declare_parameter("configuration_manifest_path", "")
        self.declare_parameter("indoor_ate_limit_m", 0.30)
        self.declare_parameter("map_rate_limit_hz", 10.0)
        self.declare_parameter("replan_limit_s", 2.0)
        self.declare_parameter("write_period_s", 2.0)

        self.run_dir = Path(str(self.get_parameter("run_dir").value)).expanduser()
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.truth: List[Tuple[float, float, float]] = []
        self.estimate: List[Tuple[float, float, float]] = []
        self.odom_stamps: List[float] = []
        self.map_stamps: List[float] = []
        self.raw_lidar_stamps: List[float] = []
        self.raw_imu_stamps: List[float] = []
        self.localization_quality_stamps: List[float] = []
        self.replan_latency: List[float] = []
        self.replan_accepted: List[bool] = []
        self.replan_events: List[Dict[str, object]] = []
        self.states: List[Dict[str, object]] = []
        self.health: List[Dict[str, object]] = []
        self.faults: List[Dict[str, object]] = []
        self.network_stats: List[Dict[str, object]] = []
        self.map_resolution_m: float | None = None
        self._final_report_written = False
        self.configuration_evidence = self._load_configuration_evidence()
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.create_subscription(
            Odometry,
            "/xq/eval/agent_01/ground_truth",
            self._on_truth,
            100,
        )
        self.create_subscription(
            Odometry,
            "/xq/agent_01/localization/odom",
            self._on_estimate,
            100,
        )
        self.create_subscription(
            PointCloud2,
            "/xq/agent_01/sensors/lidar/points",
            self._on_raw_lidar,
            lidar_qos,
        )
        self.create_subscription(
            Imu,
            "/xq/agent_01/sensors/imu",
            self._on_raw_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LocalizationQuality,
            "/xq/agent_01/localization/quality",
            self._on_localization_quality,
            100,
        )
        self.create_subscription(OccupancyGrid, "/xq/agent_01/map/nav", self._on_map, 20)
        self.create_subscription(ReplanEvent, "/xq/agent_01/planning/events", self._on_replan, 50)
        self.create_subscription(String, "/xq/agent_01/autonomy/state", self._on_state, 50)
        self.create_subscription(HealthStatus, "/xq/agent_01/health", self._on_health, 100)
        self.create_subscription(FaultEvent, "/xq/test/fault_event", self._on_fault, 50)
        self.create_subscription(
            String,
            "/xq/agent_01/network/stats",
            self._on_network_stats,
            50,
        )
        self.create_timer(float(self.get_parameter("write_period_s").value), self.write_snapshot)
        self.get_logger().info(f"Metrics evidence directory: {self.run_dir}")

    def _load_configuration_evidence(self) -> Dict[str, object]:
        configured = str(self.get_parameter("configuration_manifest_path").value).strip()
        if not configured:
            return {
                "status": "UNAVAILABLE",
                "reason": "No configuration_manifest_path was supplied (direct launch).",
            }
        manifest_path = Path(configured).expanduser()
        try:
            raw = manifest_path.read_bytes()
            data = json.loads(raw.decode("utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            return {
                "status": "INVALID",
                "manifest_path": str(manifest_path),
                "reason": str(exc),
            }
        if data.get("contract") != "xq_runtime_configuration_evidence":
            return {
                "status": "INVALID",
                "manifest_path": str(manifest_path),
                "reason": "Unexpected configuration evidence contract.",
            }
        verification = []
        evidence_root = manifest_path.parent.resolve()
        for item in data.get("files", []):
            relative = Path(str(item.get("snapshot_path", "")))
            snapshot = (evidence_root / relative).resolve()
            try:
                snapshot.relative_to(evidence_root)
                actual_hash = hashlib.sha256(snapshot.read_bytes()).hexdigest()
                matches = actual_hash == item.get("sha256")
                error = None
            except (OSError, ValueError) as exc:
                actual_hash = None
                matches = False
                error = str(exc)
            verification.append(
                {
                    "logical_name": item.get("logical_name"),
                    "snapshot_path": str(relative),
                    "expected_sha256": item.get("sha256"),
                    "actual_sha256": actual_hash,
                    "matches": matches,
                    "error": error,
                }
            )
        all_verified = bool(verification) and all(
            bool(item["matches"]) for item in verification
        )
        return {
            "status": "CAPTURED" if all_verified else "INVALID",
            "manifest_path": str(manifest_path),
            "manifest_sha256": hashlib.sha256(raw).hexdigest(),
            "snapshot_hash_verification": {
                "status": "PASS" if all_verified else "FAIL",
                "files": verification,
            },
            "manifest": data,
        }

    def _planned_fault_events(self) -> List[Dict[str, object]]:
        if self.configuration_evidence.get("status") != "CAPTURED":
            return []
        manifest = self.configuration_evidence.get("manifest", {})
        if not manifest.get("fault_injection_enabled", False):
            return []
        manifest_path = Path(
            str(self.configuration_evidence.get("manifest_path", ""))
        )
        for item in manifest.get("files", []):
            if item.get("logical_name") != "fault_schedule":
                continue
            schedule_path = manifest_path.parent / str(item.get("snapshot_path", ""))
            try:
                schedule = json.loads(schedule_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                return []
            events = schedule.get("events", [])
            return [dict(event) for event in events if isinstance(event, dict)]
        return []

    @staticmethod
    def _append_monotonic_stamp(stamps: List[float], stamp_s: float) -> None:
        """Keep only positive, strictly increasing simulation timestamps."""
        if stamp_s > 0.0 and (not stamps or stamp_s > stamps[-1]):
            stamps.append(stamp_s)

    def _on_raw_lidar(self, message: PointCloud2) -> None:
        self._append_monotonic_stamp(self.raw_lidar_stamps, _stamp_s(message))

    def _on_raw_imu(self, message: Imu) -> None:
        self._append_monotonic_stamp(self.raw_imu_stamps, _stamp_s(message))

    def _on_localization_quality(self, message: LocalizationQuality) -> None:
        self._append_monotonic_stamp(
            self.localization_quality_stamps,
            _stamp_s(message),
        )

    def _on_truth(self, message: Odometry) -> None:
        self.truth.append(
            (
                _stamp_s(message),
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            )
        )

    def _on_estimate(self, message: Odometry) -> None:
        stamp = _stamp_s(message)
        self.estimate.append(
            (
                stamp,
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
            )
        )
        self._append_monotonic_stamp(self.odom_stamps, stamp)

    def _on_map(self, message: OccupancyGrid) -> None:
        stamp = _stamp_s(message)
        self._append_monotonic_stamp(self.map_stamps, stamp)
        self.map_resolution_m = float(message.info.resolution)

    def _on_replan(self, message: ReplanEvent) -> None:
        self.replan_latency.append(float(message.latency_ms) * 1.0e-3)
        self.replan_accepted.append(bool(message.accepted))
        self.replan_events.append(
            {
                "stamp_s": _stamp_s(message),
                "seq": int(message.seq),
                "trigger_reason": str(message.trigger_reason),
                "latency_s": float(message.latency_ms) * 1.0e-3,
                "accepted": bool(message.accepted),
                "brake_fallback": bool(message.brake_fallback),
                "outcome": str(message.outcome),
            }
        )

    def _on_state(self, message: String) -> None:
        append_state_transition(
            self.states,
            self.get_clock().now().nanoseconds * 1.0e-9,
            message.data,
        )

    def _on_health(self, message: HealthStatus) -> None:
        self.health.append(
            {
                "stamp_s": _stamp_s(message),
                "module": message.module_id,
                "state": int(message.state),
                "age_ms": float(message.age_ms),
                "quality": float(message.quality),
                "error_code": message.error_code,
            }
        )
        if len(self.health) > 10000:
            self.health = self.health[-10000:]

    def _on_fault(self, message: FaultEvent) -> None:
        stamp_s = _stamp_s(message)
        if stamp_s <= 0.0:
            stamp_s = self.get_clock().now().nanoseconds * 1.0e-9
        self.faults.append(
            {
                "stamp_s": stamp_s,
                "fault_id": str(message.fault_id),
                "target_module": str(message.target_module),
                "action": str(message.action),
                "severity": float(message.severity),
                "duration_s": float(message.duration_s),
                "seed": int(message.seed),
            }
        )

    def _on_network_stats(self, message: String) -> None:
        """Capture project-relay JSON without assuming a relay implementation."""
        stamp_s = self.get_clock().now().nanoseconds * 1.0e-9
        try:
            decoded = json.loads(message.data)
            payload: Dict[str, object] = (
                decoded if isinstance(decoded, dict) else {"value": decoded}
            )
            record: Dict[str, object] = {
                "stamp_s": stamp_s,
                "parse_status": "JSON_OK",
                "payload": payload,
            }
        except json.JSONDecodeError as exc:
            record = {
                "stamp_s": stamp_s,
                "parse_status": "INVALID_JSON",
                "error": str(exc),
                "raw": message.data[:2048],
            }
        self.network_stats.append(record)
        if len(self.network_stats) > 10000:
            self.network_stats = self.network_stats[-10000:]

    def _trajectory_report(self) -> Dict[str, object]:
        if len(self.truth) < 2 or len(self.estimate) < 2:
            return {"status": "INSUFFICIENT_EVIDENCE", "samples": 0}
        truth = np.asarray(self.truth, dtype=float)
        estimate = np.asarray(self.estimate, dtype=float)
        order_truth = np.argsort(truth[:, 0])
        order_est = np.argsort(estimate[:, 0])
        truth = truth[order_truth]
        estimate = estimate[order_est]
        valid = (estimate[:, 0] >= truth[0, 0]) & (estimate[:, 0] <= truth[-1, 0])
        estimate = estimate[valid]
        if estimate.shape[0] < 10:
            return {"status": "INSUFFICIENT_EVIDENCE", "samples": int(estimate.shape[0])}
        truth_interp = np.column_stack(
            [
                np.interp(estimate[:, 0], truth[:, 0], truth[:, 1]),
                np.interp(estimate[:, 0], truth[:, 0], truth[:, 2]),
            ]
        )
        result = trajectory_metrics(estimate[:, 1:3], truth_interp)
        limit = float(self.get_parameter("indoor_ate_limit_m").value)
        result["threshold_m"] = limit
        result["status"] = "SIMULATED_PASS" if result["ate_rms_m"] <= limit else "SIMULATED_FAIL"
        return result

    def _frequency_report(self, stamps: List[float], threshold: float) -> Dict[str, object]:
        if len(stamps) < 3:
            return {"status": "INSUFFICIENT_EVIDENCE", "samples": len(stamps)}
        result = frequency_metrics(stamps)
        result["threshold_hz"] = threshold
        result["status"] = (
            "SIMULATED_PASS" if result["mean_hz"] >= threshold and result["worst_window_hz"] >= threshold else "SIMULATED_FAIL"
        )
        return result

    def _replan_report(self) -> Dict[str, object]:
        threshold = float(self.get_parameter("replan_limit_s").value)
        reason_counts: Dict[str, int] = {}
        for event in self.replan_events:
            reason = str(event.get("trigger_reason", "unknown"))
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
        proxy_summary: Dict[str, object]
        if self.replan_latency:
            proxy_summary = replan_metrics(
                self.replan_latency,
                self.replan_accepted,
            )
            proxy_summary["measurement_status"] = "OBSERVED_PROXY_ONLY"
        else:
            proxy_summary = {
                "events": 0,
                "measurement_status": "INSUFFICIENT_EVIDENCE",
            }
        obstacle_events = [
            event
            for event in self.replan_events
            if event.get("trigger_reason") == "new_obstacle"
        ]
        if obstacle_events:
            obstacle_latency = [float(event["latency_s"]) for event in obstacle_events]
            obstacle_accepted = [bool(event["accepted"]) for event in obstacle_events]
            obstacle_proxy: Dict[str, object] = replan_metrics(
                obstacle_latency,
                obstacle_accepted,
            )
            obstacle_proxy.update(
                {
                    "measurement_status": "OBSERVED_PROXY_ONLY",
                    "threshold_s": threshold,
                    "within_2s_proxy_observation": bool(
                        obstacle_proxy["max_s"] <= threshold
                        and obstacle_proxy["success_rate"] == 1.0
                    ),
                    "formal_requirement_credit": False,
                }
            )
        else:
            obstacle_proxy = {
                "events": 0,
                "measurement_status": "INSUFFICIENT_EVIDENCE",
                "threshold_s": threshold,
                "formal_requirement_credit": False,
            }
        return {
            "status": "UNVERIFIED",
            "formal_requirement": "R6 dynamic-obstacle-triggered replanning <= 2 s",
            "reason": (
                "Periodic/no_path proxy planning is excluded from R6 evidence; "
                "the run does not exercise the production EGO optimizer and a "
                "formal outdoor dynamic-obstacle protocol."
            ),
            "events": len(self.replan_events),
            "trigger_reason_counts": reason_counts,
            "all_proxy_events": proxy_summary,
            "obstacle_trigger_proxy": obstacle_proxy,
        }

    def _latest_observed_sim_time(self) -> float:
        candidates = [0.0]
        for stamps in (
            self.odom_stamps,
            self.map_stamps,
            self.raw_lidar_stamps,
            self.raw_imu_stamps,
            self.localization_quality_stamps,
        ):
            if stamps:
                candidates.append(float(stamps[-1]))
        for records, key in (
            (self.states, "sim_time_s"),
            (self.health, "stamp_s"),
            (self.replan_events, "stamp_s"),
            (self.network_stats, "stamp_s"),
        ):
            if records:
                candidates.append(float(records[-1].get(key, 0.0)))
        return max(candidates)

    @staticmethod
    def _state_at(states: List[Dict[str, object]], stamp_s: float) -> str | None:
        state = None
        for item in states:
            if float(item.get("sim_time_s", 0.0)) > stamp_s:
                break
            state = str(item.get("state", "UNKNOWN"))
        return state

    @staticmethod
    def _stream_window_evidence(
        stamps: List[float],
        start_s: float,
        end_s: float,
        observation_end_s: float,
        continuity_gap_limit_s: float = 0.50,
    ) -> Dict[str, object]:
        window = [stamp for stamp in stamps if start_s <= stamp <= end_s]
        nearest_before = next(
            (stamp for stamp in reversed(stamps) if stamp <= start_s),
            None,
        )
        nearest_after = next((stamp for stamp in stamps if stamp >= end_s), None)
        anchors = [start_s, *window, end_s]
        max_silence_s = max(
            (right - left for left, right in zip(anchors, anchors[1:])),
            default=max(0.0, end_s - start_s),
        )
        window_complete = observation_end_s >= end_s
        if not window_complete:
            continuity_status = "INCOMPLETE_OBSERVATION_WINDOW"
        elif window and max_silence_s <= continuity_gap_limit_s:
            continuity_status = "CONTINUOUS"
        else:
            continuity_status = "GAP_OBSERVED"
        result: Dict[str, object] = {
            "status": continuity_status,
            "window_complete": window_complete,
            "samples_in_window": len(window),
            "nearest_sample_at_or_before_start_s": nearest_before,
            "nearest_sample_at_or_after_end_s": nearest_after,
            "max_silence_within_window_s": float(max_silence_s),
            "continuity_gap_limit_s": continuity_gap_limit_s,
            "continued_through_fault_window": continuity_status == "CONTINUOUS",
        }
        if len(window) >= 3:
            result["window_frequency"] = frequency_metrics(window)
        return result

    def _fault_response_evidence(self) -> List[Dict[str, object]]:
        observation_end_s = self._latest_observed_sim_time()
        responses: List[Dict[str, object]] = []
        health_names = {0: "OK", 1: "WARN", 2: "FAIL"}
        map_gap_limit = max(
            0.50,
            3.0 / max(0.1, float(self.get_parameter("map_rate_limit_hz").value)),
        )
        for fault in self.faults:
            start_s = float(fault["stamp_s"])
            duration_s = max(0.0, float(fault["duration_s"]))
            end_s = start_s + duration_s
            state_transitions = [
                dict(item)
                for item in self.states
                if start_s <= float(item.get("sim_time_s", 0.0)) <= end_s
            ]
            target = str(fault["target_module"])
            health_window = [
                dict(item)
                for item in self.health
                if item.get("module") == target
                and start_s <= float(item.get("stamp_s", 0.0)) <= end_s
            ]
            critical_modules = ("fcu", "lidar", "localization", "planner")
            critical_health_window = [
                dict(item)
                for item in self.health
                if item.get("module") in critical_modules
                and start_s <= float(item.get("stamp_s", 0.0)) <= end_s
            ]
            non_ok = [item for item in health_window if int(item.get("state", 0)) != 0]
            first_non_ok_latency = (
                float(non_ok[0]["stamp_s"]) - start_s if non_ok else None
            )
            network_window = [
                dict(item)
                for item in self.network_stats
                if start_s <= float(item.get("stamp_s", 0.0)) <= end_s
            ]
            network_payloads = [
                item.get("payload", {})
                for item in network_window
                if item.get("parse_status") == "JSON_OK"
                and isinstance(item.get("payload"), dict)
            ]
            network_fault_ids = list(
                dict.fromkeys(
                    str(payload.get("fault_id", ""))
                    for payload in network_payloads
                    if payload.get("fault_id")
                )
            )
            matching_network_payloads = [
                payload
                for payload in network_payloads
                if str(payload.get("fault_id", "")) == str(fault["fault_id"])
            ]
            active_seen = any(
                bool(payload.get("active")) for payload in matching_network_payloads
            )
            reported_drop_rates = [
                float(payload["observed_drop_rate"])
                for payload in matching_network_payloads
                if isinstance(payload.get("observed_drop_rate"), (int, float))
            ]
            sent_counters = [
                int(payload["fault_window_sent"])
                for payload in matching_network_payloads
                if isinstance(payload.get("fault_window_sent"), (int, float))
            ]
            dropped_counters = [
                int(payload["fault_window_dropped"])
                for payload in matching_network_payloads
                if isinstance(payload.get("fault_window_dropped"), (int, float))
            ]
            sent_delta = (
                max(sent_counters) - min(sent_counters) if sent_counters else None
            )
            dropped_delta = (
                max(dropped_counters) - min(dropped_counters)
                if dropped_counters
                else None
            )
            counter_delta_drop_rate = (
                float(dropped_delta) / float(sent_delta)
                if sent_delta is not None
                and dropped_delta is not None
                and sent_delta > 0
                else None
            )
            target_health = {
                "samples_in_window": len(health_window),
                "states_observed": list(
                    dict.fromkeys(
                        health_names.get(int(item.get("state", -1)), "UNKNOWN")
                        for item in health_window
                    )
                ),
                "worst_state": health_names.get(
                    max((int(item.get("state", -1)) for item in health_window), default=-1),
                    "NO_SAMPLE",
                ),
                "first_non_ok_latency_s": first_non_ok_latency,
            }
            critical_states_by_module = {
                module: list(
                    dict.fromkeys(
                        health_names.get(int(item.get("state", -1)), "UNKNOWN")
                        for item in critical_health_window
                        if item.get("module") == module
                    )
                )
                for module in critical_modules
            }
            response: Dict[str, object] = {
                "fault": dict(fault),
                "window": {
                    "start_sim_time_s": start_s,
                    "end_sim_time_s": end_s,
                    "duration_s": duration_s,
                    "observation_end_sim_time_s": observation_end_s,
                    "complete": observation_end_s >= end_s,
                },
                "autonomy_state_response": {
                    "state_at_start": self._state_at(self.states, start_s),
                    "transitions_in_window": state_transitions,
                    "state_at_end": self._state_at(self.states, end_s),
                },
                "target_health_response": target_health,
                "critical_health_response": {
                    "samples_in_window": len(critical_health_window),
                    "states_by_module": critical_states_by_module,
                    "fail_modules": [
                        module
                        for module, states in critical_states_by_module.items()
                        if "FAIL" in states
                    ],
                },
                "network_stats_response": {
                    "records_in_window": len(network_window),
                    "json_records_in_window": len(network_payloads),
                    "first": network_window[0] if network_window else None,
                    "last": network_window[-1] if network_window else None,
                    "fault_ids_seen": network_fault_ids,
                    "matching_fault_id_seen": bool(matching_network_payloads),
                    "active_seen": active_seen,
                    "observed_fault_window_drop_rate": (
                        reported_drop_rates[-1] if reported_drop_rates else None
                    ),
                    "counter_delta_drop_rate": counter_delta_drop_rate,
                    "fault_window_sent_delta": sent_delta,
                    "fault_window_dropped_delta": dropped_delta,
                },
                "core_stream_continuity": {
                    "proxy_odom": self._stream_window_evidence(
                        self.odom_stamps,
                        start_s,
                        end_s,
                        observation_end_s,
                    ),
                    "map_nav": self._stream_window_evidence(
                        self.map_stamps,
                        start_s,
                        end_s,
                        observation_end_s,
                        map_gap_limit,
                    ),
                },
                "verdict_scope": (
                    "Simulation proxy evidence only: this does not establish "
                    "formal R7 hardware or flight robustness."
                ),
            }
            response.update(evaluate_fault_response(response))
            responses.append(response)
        return responses

    def build_report(self) -> Dict[str, object]:
        map_threshold = float(self.get_parameter("map_rate_limit_hz").value)
        trajectory = self._trajectory_report()
        odom_frequency = self._frequency_report(self.odom_stamps, 10.0)
        odom_frequency.update(
            {
                "topic": "/xq/agent_01/localization/odom",
                "clock_basis": "message_header_sim_time",
                "evidence_role": "daf_lio_proxy_2d_output_cadence",
                "formal_localization_algorithm_credit": False,
                "note": (
                    "This is a proxy odometry publisher cadence, not raw sensor "
                    "frequency and not production FAST-LIO2 throughput."
                ),
            }
        )
        map_frequency = self._frequency_report(self.map_stamps, map_threshold)
        map_frequency.update(
            {
                "topic": "/xq/agent_01/map/nav",
                "clock_basis": "message_header_sim_time",
                "evidence_role": "td_semmap_2d_proxy_output_cadence",
            }
        )
        raw_lidar_frequency = self._frequency_report(self.raw_lidar_stamps, 10.0)
        raw_lidar_frequency.update(
            {
                "topic": "/xq/agent_01/sensors/lidar/points",
                "clock_basis": "message_header_sim_time",
                "evidence_role": "raw_gazebo_lidar_via_xq_bridge",
                "configured_nominal_hz": 10.0,
            }
        )
        raw_imu_frequency = self._frequency_report(self.raw_imu_stamps, 10.0)
        raw_imu_frequency.update(
            {
                "topic": "/xq/agent_01/sensors/imu",
                "clock_basis": "message_header_sim_time",
                "evidence_role": "raw_gazebo_imu_via_xq_bridge",
                "configured_nominal_hz": 200.0,
                "threshold_note": "R4 >=10 Hz gate; nominal 200 Hz is reported, not claimed as a formal requirement.",
            }
        )
        quality_frequency = self._frequency_report(
            self.localization_quality_stamps,
            10.0,
        )
        quality_frequency.update(
            {
                "topic": "/xq/agent_01/localization/quality",
                "clock_basis": "message_header_sim_time",
                "evidence_role": "daf_lio_proxy_2d_quality_output_cadence",
                "formal_localization_algorithm_credit": False,
            }
        )
        gated_metrics = {
            "trajectory": trajectory.get("status", "MISSING"),
            "raw_lidar_frequency": raw_lidar_frequency.get("status", "MISSING"),
            "raw_imu_frequency": raw_imu_frequency.get("status", "MISSING"),
            "localization_quality_frequency": quality_frequency.get("status", "MISSING"),
            "proxy_odom_frequency": odom_frequency.get("status", "MISSING"),
            "map_frequency": map_frequency.get("status", "MISSING"),
        }
        gate_checks = [
            {"metric": name, "status": status}
            for name, status in gated_metrics.items()
        ]
        gate_status = (
            "SIMULATED_PASS"
            if gate_checks
            and all(item["status"] == "SIMULATED_PASS" for item in gate_checks)
            else "SIMULATED_FAIL"
            if any(item["status"] == "SIMULATED_FAIL" for item in gate_checks)
            else "INSUFFICIENT_EVIDENCE"
        )
        fault_summary = fault_evidence_summary(self.faults)
        planned_faults = self._planned_fault_events()
        planned_fault_ids = [
            str(item.get("fault_id"))
            for item in planned_faults
            if item.get("fault_id")
        ]
        observed_fault_ids = list(fault_summary["fault_ids_observed"])
        faults_enabled = bool(
            self.configuration_evidence.get("manifest", {}).get(
                "fault_injection_enabled",
                False,
            )
        )
        missing_fault_ids = [
            item for item in planned_fault_ids if item not in observed_fault_ids
        ]
        unexpected_fault_ids = [
            item for item in observed_fault_ids if item not in planned_fault_ids
        ]
        schedule_coverage_status = (
            "UNVERIFIED_CONFIGURATION"
            if self.configuration_evidence.get("status") != "CAPTURED"
            else "NOT_ENABLED"
            if not faults_enabled
            else "COMPLETE"
            if planned_fault_ids and not missing_fault_ids and not unexpected_fault_ids
            else "INCOMPLETE_EVIDENCE"
        )
        fault_responses = self._fault_response_evidence()
        response_status_counts: Dict[str, int] = {}
        for response in fault_responses:
            status = str(response.get("status", "INSUFFICIENT_EVIDENCE"))
            response_status_counts[status] = response_status_counts.get(status, 0) + 1
        response_acceptance_status = (
            "NOT_ENABLED"
            if not faults_enabled
            else "INSUFFICIENT_EVIDENCE"
            if not fault_responses
            else "SIMULATED_FAIL"
            if response_status_counts.get("SIMULATED_FAIL", 0) > 0
            else "INSUFFICIENT_EVIDENCE"
            if response_status_counts.get("INSUFFICIENT_EVIDENCE", 0) > 0
            else "SIMULATED_PASS"
        )
        fault_summary.update(
            {
                "responses": fault_responses,
                "response_acceptance": {
                    "status": response_acceptance_status,
                    "status_counts": response_status_counts,
                    "scope": (
                        "Gazebo SIL/proxy acceptance only; no formal hardware or "
                        "flight acceptance credit."
                    ),
                },
                "schedule_coverage": {
                    "status": schedule_coverage_status,
                    "planned_count": len(planned_fault_ids),
                    "planned_fault_ids": planned_fault_ids,
                    "observed_fault_ids": observed_fault_ids,
                    "missing_fault_ids": missing_fault_ids,
                    "unexpected_fault_ids": unexpected_fault_ids,
                },
                "scope": (
                    "Simulation-time state/health/stream evidence only; no formal "
                    "hardware or flight-robustness credit."
                ),
            }
        )
        return {
            "schema_version": 3,
            "validation_layer": "GAZEBO_SIL_PROXY",
            "scenario": str(self.get_parameter("scenario").value),
            "seed": int(self.get_parameter("seed").value),
            "spec_sha256": str(self.get_parameter("spec_sha256").value),
            "world_sha256": str(self.get_parameter("world_sha256").value),
            "truth_topic_contract_declared_eval_only": True,
            "runtime_graph_audit": {
                "status": "UNVERIFIED",
                "reason": (
                    "This report records the source/launch topic contract; it does "
                    "not contain a captured runtime ROS graph subscription audit."
                ),
            },
            "configuration_evidence": self.configuration_evidence,
            "trajectory": trajectory,
            "trajectory_scope": {
                "scenario_class": "indoor_simulation_only",
                "formal_r1_credit": False,
                "formal_r2_outdoor_credit": False,
            },
            "sensor_frequencies": {
                "raw_lidar": raw_lidar_frequency,
                "raw_imu": raw_imu_frequency,
                "localization_quality": quality_frequency,
            },
            "odom_frequency": odom_frequency,
            "map_frequency": map_frequency,
            "replanning": self._replan_report(),
            "algorithm_gate": {
                "status": gate_status,
                "checks": gate_checks,
                "scope": (
                    "Gazebo/proxy regression gate only. R2 outdoor ATE and formal "
                    "R6 are intentionally excluded."
                ),
            },
            "formal_requirement_verdicts": {
                "R2_outdoor_ate": {
                    "status": "UNVERIFIED",
                    "reason": "This world and trajectory are indoor Gazebo SIL; no outdoor protocol was run.",
                },
                "R6_dynamic_obstacle_replanning": {
                    "status": "UNVERIFIED",
                    "reason": (
                        "Periodic/no_path proxy events are not obstacle-response evidence, "
                        "and the production EGO planner/outdoor protocol were not run."
                    ),
                },
            },
            "map_resolution": {
                "configured_m": self.map_resolution_m,
                "official_output_requirement_m": 0.05,
                "status": "UNVERIFIED",
                "reason": "A configured voxel/cell size does not prove real 5 cm map accuracy.",
            },
            "power": {
                "status": "UNVERIFIED",
                "reason": "Gazebo cannot prove the Atlas <=30 W requirement; use an input power analyzer.",
            },
            "fast_lio2": {
                "status": "UNVERIFIED",
                "reason": "This run uses daf_lio_proxy_2d, not the production FAST-LIO2 ESIKF.",
            },
            "ego_planner": {
                "status": "UNVERIFIED",
                "reason": "This run uses r2_ego_proxy_2d, not the production EGO B-spline optimizer.",
            },
            "state_samples": len(self.states),
            "health_samples": len(self.health),
            "fault_events": fault_summary,
            "network_evidence": {
                "topic": "/xq/agent_01/network/stats",
                "message_contract": "std_msgs/String containing JSON",
                "records": len(self.network_stats),
                "latest": self.network_stats[-1] if self.network_stats else None,
                "status": "OBSERVED" if self.network_stats else "INSUFFICIENT_EVIDENCE",
            },
        }

    def write_snapshot(self) -> None:
        """Write compact, overwrite-safe progress evidence only."""
        self.write_report(include_timelines=False)

    def write_report(self, include_timelines: bool = False) -> None:
        report = self.build_report()
        metrics_path = self.run_dir / "metrics.json"
        metrics_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        if include_timelines:
            (self.run_dir / "state_timeline.json").write_text(
                json.dumps(self.states, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            (self.run_dir / "health_samples.json").write_text(
                json.dumps(self.health, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            (self.run_dir / "fault_events.json").write_text(
                json.dumps(self.faults, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            (self.run_dir / "fault_response_evidence.json").write_text(
                json.dumps(
                    report["fault_events"]["responses"],
                    ensure_ascii=False,
                    indent=2,
                ),
                encoding="utf-8",
            )
            (self.run_dir / "network_stats.json").write_text(
                json.dumps(self.network_stats, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            (self.run_dir / "replan_events.json").write_text(
                json.dumps(self.replan_events, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            self._final_report_written = True
        fault_summary = report["fault_events"]
        fault_ids = ", ".join(fault_summary["fault_ids_observed"]) or "无"
        configuration = report["configuration_evidence"]
        configuration_manifest = configuration.get("manifest", {})
        configuration_files = configuration_manifest.get("files", [])
        lines = [
            "# 玄穹-X1 Gazebo SIL 验证报告",
            "",
            f"- 验证层级：`{report['validation_layer']}`",
            f"- 场景：`{report['scenario']}`",
            f"- 随机种子：`{report['seed']}`",
            f"- TXT SHA-256：`{report['spec_sha256']}`",
            f"- World SHA-256：`{report['world_sha256']}`",
            "- 真值订阅契约/源码审计：算法路径声明的真值订阅数为 0，算法栈源码不订阅 `/xq/eval/**`；运行时 ROS graph 审计为 `UNVERIFIED`。",
            f"- 配置证据：`{configuration.get('status')}`，manifest SHA-256=`{configuration.get('manifest_sha256', 'N/A')}`",
            "",
            "## 实际运行配置快照",
            "",
        ]
        if configuration_files:
            lines.extend(
                f"- `{item.get('logical_name')}`：`{item.get('sha256')}`，快照 `{item.get('snapshot_path')}`，active={item.get('active_for_this_run')}"
                for item in configuration_files
            )
        else:
            lines.append("- 未捕获配置清单；本次直接启动不具备可复现配置证据。")
        lines.extend(
            [
            "",
            "## 自动指标",
            "",
            f"- 室内仿真代理 ATE：`{report['trajectory'].get('status')}`，RMS={report['trajectory'].get('ate_rms_m', 'N/A')} m；不授予正式 R1/R2 结论",
            f"- Raw LiDAR（仿真时间）：`{report['sensor_frequencies']['raw_lidar'].get('status')}`，mean={report['sensor_frequencies']['raw_lidar'].get('mean_hz', 'N/A')} Hz",
            f"- Raw IMU（仿真时间）：`{report['sensor_frequencies']['raw_imu'].get('status')}`，mean={report['sensor_frequencies']['raw_imu'].get('mean_hz', 'N/A')} Hz",
            f"- LocalizationQuality（仿真时间）：`{report['sensor_frequencies']['localization_quality'].get('status')}`，mean={report['sensor_frequencies']['localization_quality'].get('mean_hz', 'N/A')} Hz",
            f"- 代理 Odom 输出频率：`{report['odom_frequency'].get('status')}`，mean={report['odom_frequency'].get('mean_hz', 'N/A')} Hz；不是 raw 传感器频率或正式 FAST-LIO2 吞吐",
            f"- 地图频率：`{report['map_frequency'].get('status')}`，mean={report['map_frequency'].get('mean_hz', 'N/A')} Hz",
            f"- R6 正式障碍响应重规划：`{report['replanning'].get('status')}`；代理 events={report['replanning'].get('events', 0)}，周期/no_path 事件不计作 R6 证据",
            f"- Gazebo/代理算法门槛：`{report['algorithm_gate'].get('status')}`（明确排除正式 R2 与 R6）",
            f"- 项目级网络 relay 统计：`{report['network_evidence'].get('status')}`，records={report['network_evidence'].get('records', 0)}",
            f"- 已观测故障事件：`{fault_summary['observed_count']}`，fault_id={fault_ids}",
            f"- 故障计划覆盖：`{fault_summary['schedule_coverage'].get('status')}`，missing={fault_summary['schedule_coverage'].get('missing_fault_ids')}，unexpected={fault_summary['schedule_coverage'].get('unexpected_fault_ids')}",
            f"- F1–F8 代理响应验收：`{fault_summary['response_acceptance'].get('status')}`，counts={fault_summary['response_acceptance'].get('status_counts')}；不授予正式硬件/实飞验收结论",
            "",
            "## 故障窗响应证据",
            "",
            ]
        )
        responses = fault_summary.get("responses", [])
        if responses:
            for response in responses:
                fault = response["fault"]
                state = response["autonomy_state_response"]
                streams = response["core_stream_continuity"]
                transition_names = [
                    str(item.get("state"))
                    for item in state.get("transitions_in_window", [])
                ]
                failed_checks = [
                    str(item.get("name"))
                    for item in response.get("expected_checks", [])
                    if item.get("status") != "SIMULATED_PASS"
                ]
                lines.append(
                    f"- `{fault.get('fault_id')}` ({fault.get('target_module')}): "
                    f"`{response.get('status', 'INSUFFICIENT_EVIDENCE')}`，"
                    f"state {state.get('state_at_start')} → {state.get('state_at_end')}，"
                    f"窗内转换={transition_names or ['无']}；"
                    f"proxy odom={streams['proxy_odom'].get('status')}，"
                    f"map={streams['map_nav'].get('status')}；"
                    f"未通过检查={failed_checks or ['无']}。"
                )
        else:
            lines.append("- 本次未启用或未观测故障；无故障响应结论。")
        lines.extend(
            [
            "",
            "## 不可由本仿真证明",
            "",
            "- Atlas 30 W 功耗、温升与 4 GB 满载实时性：`UNVERIFIED`",
            "- 真实 FAST-LIO2 / EGO-Planner 性能：`UNVERIFIED`",
            "- 真实 5 cm 三维地图质量、正式室内 ATE 与 R2 室外 ATE：`UNVERIFIED`",
            "- 正式 R6 动态障碍响应（生产 EGO + 室外协议）：`UNVERIFIED`",
            "- 多机结果只能表述为 SITL/HIL，不得表述为多机实飞。",
            ]
        )
        (self.run_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def destroy_node(self):
        try:
            if not self._final_report_written:
                self.write_report(include_timelines=True)
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = XqMetricsNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
