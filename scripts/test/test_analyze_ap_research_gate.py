"""Trial-level checks must fail closed and preserve their input evidence."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "analyze_ap_research_gate.py"


def endpoint(name: str, role: str) -> str:
    return f"Node name: {name}\nNode namespace: /\nEndpoint type: {role}\n"


class APResearchGateTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.run_dir = Path(self.temporary.name)
        (self.run_dir / "configuration").mkdir()
        self.config = {
            "minimum_progress_m": 1.65, "maximum_ate_rms_m": 0.1,
            "maximum_position_error_m": 0.3, "maximum_watchdog_stop_delay_s": 0.55,
        }
        self.mission = {
            "status": "PASS", "checks": {name: True for name in (
                "gps_disabled", "external_nav", "takeoff_and_hover", "research_decision",
                "executed_progress", "executor_forwarded", "execution_disabled_acknowledged",
                "landed_and_disarmed",
            )},
            "executor": {
                "disabled_publish_count": 0, "accepted_commands": 50, "published_commands": 100,
                "freeze_test_started": True, "freeze_test_completed": True,
                "replay_rejections": 5, "freeze_zero_count": 3, "freeze_first_zero_delay_s": 0.4,
            },
        }
        self.truth = {
            "status": "COMPLETE", "research_truth_progress_m": 1.8,
            "ate_rms_m": 0.04, "ate_max_m": 0.15,
        }
        self.write("configuration/research_ap_gate.json", self.config)
        self.write("mission-result.json", self.mission)
        self.write("evaluation-result.json", self.truth)
        self.write("readiness.json", {"status": "READY"})
        self.graph = (
            "Type: nav_msgs/msg/Odometry\nPublisher count: 1\n"
            + endpoint("xq_gz_bridge_node", "PUBLISHER")
            + "Subscription count: 2\n"
            + endpoint("impact_ap_research_evaluator", "SUBSCRIPTION")
            + endpoint("rosbag2_recorder", "SUBSCRIPTION")
        )
        self.write_graph(self.graph)
        self.legacy_graph = (
            "Publisher count: 1\n" + endpoint("xq_p13_flight_controller", "PUBLISHER")
            + "Subscription count: 0\n"
        )
        (self.run_dir / "unused-legacy-topic-graph.txt").write_text(self.legacy_graph, encoding="utf-8")
        self.command_graph = (
            "Publisher count: 1\n" + endpoint("impact_ap_execution", "PUBLISHER")
            + "Subscription count: 2\n" + endpoint("setpoint_velocity", "SUBSCRIPTION")
            + endpoint("rosbag2_recorder", "SUBSCRIPTION")
        )
        (self.run_dir / "ap-command-topic-graph.txt").write_text(self.command_graph, encoding="utf-8")
        (self.run_dir / "mavros-velocity-frame.txt").write_text("String value is: LOCAL_NED\n", encoding="utf-8")
        self.active_audit = {"status": "PASS", "processes_alive": True}
        self.write("active-node-log-audit.json", self.active_audit)
        self.bag_audit = {
            "status": "PASS", "message_count": 1000,
            "topics": {
                "/xq/eval/p4/ground_truth": 400,
                "/impact/research/command_provenance": 200,
                "/uav1/mavros/setpoint_velocity/cmd_vel": 300,
                "/impact/ap/mission_status": 100,
            },
        }
        self.write("bag-audit.json", self.bag_audit)

    def write(self, relative, data):
        (self.run_dir / relative).write_text(json.dumps(data), encoding="utf-8")

    def write_graph(self, text):
        (self.run_dir / "ground-truth-topic-graph.txt").write_text(text, encoding="utf-8")

    def analyze(self, *, replay=False):
        args = [sys.executable, "-B", str(SCRIPT), "--run-dir", str(self.run_dir)]
        if replay:
            args.append("--replay-test")
        completed = subprocess.run(args, capture_output=True, text=True, check=False)
        result = json.loads((self.run_dir / "gate-result.json").read_text(encoding="utf-8"))
        return completed, result

    def test_normal_and_replay_pass_without_altering_source_evidence(self):
        before = {path: path.read_bytes() for path in self.run_dir.rglob("*") if path.is_file()}
        for replay in (False, True):
            with self.subTest(replay=replay):
                process, result = self.analyze(replay=replay)
                self.assertEqual(process.returncode, 0, process.stderr)
                self.assertEqual(result["status"], "PASS")
                self.assertTrue(all(result["checks"].values()))
                self.assertIn("not statistical", result["proof_scope"])
        for path, content in before.items():
            self.assertEqual(path.read_bytes(), content)

    def test_truth_failure_is_persisted_even_when_mission_reports_pass(self):
        self.truth["research_truth_progress_m"] = 1.0
        self.write("evaluation-result.json", self.truth)
        process, result = self.analyze()
        self.assertNotEqual(process.returncode, 0)
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("truth_progress", result["failed_checks"])

    def test_missing_handoff_check_cannot_pass(self):
        del self.mission["checks"]["execution_disabled_acknowledged"]
        self.write("mission-result.json", self.mission)
        process, result = self.analyze()
        self.assertNotEqual(process.returncode, 0)
        self.assertIn("mission_checks_all_true", result["failed_checks"])

    def test_replay_requires_rejection_zero_and_bounded_observed_delay(self):
        for field, invalid in (
            ("replay_rejections", 0), ("freeze_zero_count", 0),
            ("freeze_first_zero_delay_s", 0.551), ("freeze_test_completed", False),
        ):
            with self.subTest(field=field):
                original = self.mission["executor"][field]
                self.mission["executor"][field] = invalid
                self.write("mission-result.json", self.mission)
                process, result = self.analyze(replay=True)
                self.assertNotEqual(process.returncode, 0)
                self.assertEqual(result["status"], "FAIL")
                self.mission["executor"][field] = original

    def test_graph_uses_endpoint_roles_and_rejects_algorithm_subscriber(self):
        bad_graphs = (
            self.graph.replace("Subscription count: 2", "Subscription count: 3")
            + endpoint("xq_fast_lio", "SUBSCRIPTION"),
            self.graph.replace(
                endpoint("impact_ap_research_evaluator", "SUBSCRIPTION"),
                endpoint("impact_ap_research_evaluator", "PUBLISHER"),
            ),
            self.graph.replace("Endpoint type: SUBSCRIPTION", "", 1),
        )
        for graph in bad_graphs:
            with self.subTest(graph=graph):
                self.write_graph(graph)
                process, result = self.analyze()
                self.assertNotEqual(process.returncode, 0)
                self.assertEqual(result["status"], "FAIL")

    def test_missing_threshold_and_non_boolean_mission_check_fail_closed(self):
        del self.config["maximum_watchdog_stop_delay_s"]
        self.write("configuration/research_ap_gate.json", self.config)
        self.mission["checks"]["landed"] = 1
        self.write("mission-result.json", self.mission)
        process, result = self.analyze()
        self.assertNotEqual(process.returncode, 0)
        self.assertIn("required_thresholds_valid", result["failed_checks"])
        self.assertIn("mission_checks_all_true", result["failed_checks"])

    def test_missing_input_and_nonfinite_error_produce_failure_artifact(self):
        (self.run_dir / "readiness.json").unlink()
        self.truth["ate_rms_m"] = float("nan")
        self.write("evaluation-result.json", self.truth)
        process, result = self.analyze()
        self.assertNotEqual(process.returncode, 0)
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("readiness.json", result["input_errors"])
        self.assertIn("evaluation-result.json", result["input_errors"])

    def test_legacy_subscription_and_second_ap_publisher_are_rejected(self):
        (self.run_dir / "unused-legacy-topic-graph.txt").write_text(
            self.legacy_graph.replace("Subscription count: 0", "Subscription count: 1")
            + endpoint("direct_gazebo_bridge", "SUBSCRIPTION"), encoding="utf-8",
        )
        (self.run_dir / "ap-command-topic-graph.txt").write_text(
            self.command_graph.replace("Publisher count: 1", "Publisher count: 2")
            + endpoint("unexpected_velocity_source", "PUBLISHER"), encoding="utf-8",
        )
        process, result = self.analyze()
        self.assertNotEqual(process.returncode, 0)
        self.assertIn("legacy_has_no_subscribers", result["failed_checks"])
        self.assertIn("ap_execution_is_only_command_publisher", result["failed_checks"])

    def test_wrong_mavros_frame_or_failed_active_audit_cannot_pass(self):
        (self.run_dir / "mavros-velocity-frame.txt").write_text("String value is: LOCAL_NED_BODY\n", encoding="utf-8")
        self.active_audit["processes_alive"] = False
        self.write("active-node-log-audit.json", self.active_audit)
        process, result = self.analyze()
        self.assertNotEqual(process.returncode, 0)
        self.assertIn("mavros_velocity_frame_local_ned", result["failed_checks"])
        self.assertIn("active_processes_alive", result["failed_checks"])

    def test_bag_requires_positive_count_for_each_required_topic(self):
        for topic in self.bag_audit["topics"]:
            with self.subTest(topic=topic):
                original = self.bag_audit["topics"][topic]
                self.bag_audit["topics"][topic] = 0
                self.write("bag-audit.json", self.bag_audit)
                process, result = self.analyze()
                self.assertNotEqual(process.returncode, 0)
                self.assertIn("bag_required_topics_recorded", result["failed_checks"])
                self.bag_audit["topics"][topic] = original


if __name__ == "__main__":
    unittest.main()
