#!/usr/bin/env python3
"""Evaluate one ArduPilot interface SIL trial, not statistical integrity safety."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile


REQUIRED_THRESHOLDS = (
    "minimum_progress_m",
    "maximum_ate_rms_m",
    "maximum_position_error_m",
    "maximum_watchdog_stop_delay_s",
)
REQUIRED_BAG_TOPICS = (
    "/xq/eval/p4/ground_truth",
    "/impact/research/command_provenance",
    "/uav1/mavros/setpoint_velocity/cmd_vel",
    "/impact/ap/mission_status",
)
REQUIRED_MISSION_CHECKS = (
    "gps_disabled", "external_nav", "takeoff_and_hover", "research_decision",
    "executed_progress", "executor_forwarded", "execution_disabled_acknowledged",
    "landed_and_disarmed",
)


def _finite_number(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _counter(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _reject_constant(value: str) -> None:
    raise ValueError(f"nonstandard JSON numeric constant: {value}")


def parse_topic_endpoints(graph: str) -> list[dict[str, str]]:
    """Parse ros2 topic info -v endpoint blocks, rejecting incomplete output."""
    nodes = list(re.finditer(r"^\s*Node name:\s*([^\r\n]+)", graph, re.MULTILINE))
    if not nodes:
        raise ValueError("topic graph contains no Node name endpoint blocks")
    if re.search(r"^\s*Endpoint type:", graph[:nodes[0].start()], re.MULTILINE):
        raise ValueError("topic graph contains an endpoint without a node")
    endpoints: list[dict[str, str]] = []
    for index, node in enumerate(nodes):
        block_end = nodes[index + 1].start() if index + 1 < len(nodes) else len(graph)
        block = graph[node.end():block_end]
        roles = re.findall(r"^\s*Endpoint type:\s*(\S+)\s*$", block, re.MULTILINE)
        if len(roles) != 1 or roles[0] not in {"PUBLISHER", "SUBSCRIPTION"}:
            raise ValueError(f"missing, duplicate, or invalid Endpoint type for {node.group(1).strip()}")
        endpoints.append({
            "node_name": node.group(1).strip().lstrip("/"),
            "endpoint_type": roles[0],
        })
    for label, role in (("Publisher", "PUBLISHER"), ("Subscription", "SUBSCRIPTION")):
        counts = re.findall(rf"^\s*{label} count:\s*(\d+)\s*$", graph, re.MULTILINE)
        if counts and (len(counts) != 1 or int(counts[0]) != sum(
            endpoint["endpoint_type"] == role for endpoint in endpoints
        )):
            raise ValueError(f"{label} count does not match the parsed endpoint blocks")
    return endpoints


def evaluate_run(run_dir: Path, replay_test: bool = False) -> dict[str, object]:
    run_dir = run_dir.resolve()
    errors: dict[str, str] = {}
    hashes: dict[str, str] = {}

    def read(relative: str, *, as_json: bool = True):
        try:
            content = (run_dir / relative).read_bytes()
            hashes[relative] = hashlib.sha256(content).hexdigest()
            text = content.decode("utf-8")
            if not as_json:
                return text
            value = json.loads(text, parse_constant=_reject_constant)
            if not isinstance(value, dict):
                raise ValueError("expected a JSON object")
            return value
        except (OSError, UnicodeError, ValueError) as exc:
            errors[relative] = str(exc)
            return {} if as_json else ""

    mission = read("mission-result.json")
    truth = read("evaluation-result.json")
    config = read("configuration/research_ap_gate.json")
    readiness = read("readiness.json")
    graph = read("ground-truth-topic-graph.txt", as_json=False)
    legacy_graph = read("unused-legacy-topic-graph.txt", as_json=False)
    command_graph = read("ap-command-topic-graph.txt", as_json=False)
    velocity_frame = read("mavros-velocity-frame.txt", as_json=False)
    active_audit = read("active-node-log-audit.json")
    bag_audit = read("bag-audit.json")
    thresholds = {
        name: config.get(name) if _finite_number(config.get(name)) else None
        for name in REQUIRED_THRESHOLDS
    }
    valid_thresholds = all(value is not None and value > 0.0 for value in thresholds.values())
    mission_checks = mission.get("checks")
    executor = mission.get("executor")
    executor = executor if isinstance(executor, dict) else {}

    def bounded_metric(name: str, threshold: str, *, minimum: bool = False) -> bool:
        value = truth.get(name)
        bound = thresholds[threshold]
        if not _finite_number(value) or value < 0.0 or bound is None or bound <= 0.0:
            return False
        return value >= bound if minimum else value <= bound

    checks = {
        "required_inputs_readable": not errors,
        "required_thresholds_valid": valid_thresholds,
        "mission_pass": mission.get("status") == "PASS",
        "mission_checks_all_true": (
            isinstance(mission_checks, dict) and bool(mission_checks)
            and all(mission_checks.get(name) is True for name in REQUIRED_MISSION_CHECKS)
            and all(value is True for value in mission_checks.values())
        ),
        "truth_complete": truth.get("status") == "COMPLETE",
        "truth_progress": bounded_metric("research_truth_progress_m", "minimum_progress_m", minimum=True),
        "truth_ate_rms": bounded_metric("ate_rms_m", "maximum_ate_rms_m"),
        "truth_maximum_position_error": bounded_metric("ate_max_m", "maximum_position_error_m"),
        "readiness_ready": readiness.get("status") == "READY",
        "disabled_setpoints_absent": (
            _counter(executor.get("disabled_publish_count")) and executor["disabled_publish_count"] == 0
        ),
        "executor_accepted_commands": (
            _counter(executor.get("accepted_commands")) and executor["accepted_commands"] > 0
        ),
        "executor_published_commands": (
            _counter(executor.get("published_commands")) and executor["published_commands"] > 0
        ),
    }
    def parse_graph(relative: str, content: str) -> tuple[list[dict[str, str]], bool]:
        try:
            return parse_topic_endpoints(content), True
        except ValueError as exc:
            errors[relative] = str(exc)
            return [], False

    endpoints, graph_valid = parse_graph("ground-truth-topic-graph.txt", graph)
    checks.update({
        "truth_graph_parseable": graph_valid,
        "truth_publishers_allowed": graph_valid and all(
            item["node_name"] == "xq_gz_bridge_node"
            for item in endpoints if item["endpoint_type"] == "PUBLISHER"
        ),
        "truth_subscribers_allowed": graph_valid and all(
            item["node_name"] in {"impact_ap_research_evaluator", "rosbag2_recorder"}
            for item in endpoints if item["endpoint_type"] == "SUBSCRIPTION"
        ),
        "truth_bridge_publisher_present": any(
            item == {"node_name": "xq_gz_bridge_node", "endpoint_type": "PUBLISHER"}
            for item in endpoints
        ),
        "truth_evaluator_subscriber_present": any(
            item == {"node_name": "impact_ap_research_evaluator", "endpoint_type": "SUBSCRIPTION"}
            for item in endpoints
        ),
    })
    legacy_endpoints, legacy_valid = parse_graph("unused-legacy-topic-graph.txt", legacy_graph)
    command_endpoints, command_valid = parse_graph("ap-command-topic-graph.txt", command_graph)
    legacy_subscriber_counts = re.findall(
        r"^\s*Subscription count:\s*(\d+)\s*$", legacy_graph, re.MULTILINE,
    )
    bag_topics = bag_audit.get("topics")
    checks.update({
        "legacy_graph_parseable": legacy_valid,
        "legacy_has_no_subscribers": (
            legacy_valid and legacy_subscriber_counts == ["0"]
            and not any(item["endpoint_type"] == "SUBSCRIPTION" for item in legacy_endpoints)
        ),
        "legacy_only_p13_publisher": legacy_valid and [
            item["node_name"] for item in legacy_endpoints if item["endpoint_type"] == "PUBLISHER"
        ] == ["xq_p13_flight_controller"],
        "ap_command_graph_parseable": command_valid,
        "ap_execution_is_only_command_publisher": command_valid and [
            item["node_name"] for item in command_endpoints if item["endpoint_type"] == "PUBLISHER"
        ] == ["impact_ap_execution"],
        "mavros_velocity_frame_local_ned": bool(re.search(
            r"^\s*String value is:\s*LOCAL_NED\s*$", velocity_frame, re.MULTILINE,
        )),
        "active_node_log_audit_pass": active_audit.get("status") == "PASS",
        "active_processes_alive": active_audit.get("processes_alive") is True,
        "bag_audit_pass": bag_audit.get("status") == "PASS",
        "bag_contains_messages": (
            _counter(bag_audit.get("message_count")) and bag_audit["message_count"] > 0
        ),
        "bag_required_topics_recorded": isinstance(bag_topics, dict) and all(
            _counter(bag_topics.get(topic)) and bag_topics[topic] > 0
            for topic in REQUIRED_BAG_TOPICS
        ),
    })
    if replay_test:
        delay = executor.get("freeze_first_zero_delay_s")
        limit = thresholds["maximum_watchdog_stop_delay_s"]
        checks.update({
            "freeze_test_started": executor.get("freeze_test_started") is True,
            "freeze_test_completed": executor.get("freeze_test_completed") is True,
            "replay_rejected": _counter(executor.get("replay_rejections")) and executor["replay_rejections"] > 0,
            "freeze_zero_published": _counter(executor.get("freeze_zero_count")) and executor["freeze_zero_count"] > 0,
            "watchdog_stop_delay_bounded": (
                _finite_number(delay) and delay >= 0.0
                and limit is not None and limit > 0.0 and delay <= limit
            ),
        })
    failed = [name for name, passed in checks.items() if not passed]
    return {
        "schema_version": 1,
        "gate": "R1_IMPACT_ARDUPILOT_INTERFACE_SIL",
        "status": "PASS" if not failed else "FAIL",
        "replay_test": replay_test,
        "run_dir": str(run_dir),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "proof_scope": (
            "Single-trial ArduPilot interface SIL gate; not statistical navigation "
            "integrity, real-flight validation, or a safety guarantee."
        ),
        "checks": checks,
        "failed_checks": failed,
        "input_errors": errors,
        "thresholds": thresholds,
        "metrics": {
            name: truth.get(name) if _finite_number(truth.get(name)) else None
            for name in ("research_truth_progress_m", "ate_rms_m", "ate_max_m")
        },
        "executor_metrics": {
            name: executor.get(name) if _finite_number(executor.get(name)) else None
            for name in (
                "disabled_publish_count", "accepted_commands", "published_commands",
                "replay_rejections", "freeze_zero_count", "freeze_first_zero_delay_s",
            )
        },
        "ground_truth_endpoints": endpoints,
        "legacy_endpoints": legacy_endpoints,
        "ap_command_endpoints": command_endpoints,
        "bag_evidence": {
            "message_count": bag_audit.get("message_count") if _counter(bag_audit.get("message_count")) else None,
            "required_topic_message_counts": {
                topic: bag_topics.get(topic) if isinstance(bag_topics, dict) and _counter(bag_topics.get(topic)) else None
                for topic in REQUIRED_BAG_TOPICS
            },
        },
        "input_sha256": hashes,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--replay-test", action="store_true")
    args = parser.parse_args(argv)
    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        print(f"FAIL: trial directory does not exist: {run_dir}", file=sys.stderr)
        return 2
    result = evaluate_run(run_dir, args.replay_test)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=run_dir, prefix=".gate-result-", suffix=".tmp", delete=False,
        ) as stream:
            temporary = Path(stream.name)
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, run_dir / "gate-result.json")
    except (OSError, ValueError) as exc:
        print(f"FAIL: cannot write gate-result.json: {exc}", file=sys.stderr)
        return 2
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    summary = f"{result['status']}: {result['gate']} ({len(result['checks'])} checks)"
    if result["failed_checks"]:
        summary += "; failed=" + ",".join(result["failed_checks"])
    print(summary)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
