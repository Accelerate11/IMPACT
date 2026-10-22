"""CLI acceptance must reject contradictory, incomplete and malformed R2 evidence."""

from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
ANALYZER = ROOT / "scripts/analyze_multisensor_contract.py"
FAULT_MATRIX = ROOT / "scripts/run_multisensor_fault_matrix.py"
VALIDATE = runpy.run_path(str(ROOT / "src/xq_autonomy/xq_autonomy/multisensor_contract.py"))["validate_contract_config"]


class MultisensorAcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.run = Path(self.temporary.name)
        (self.run / "configuration").mkdir()
        self.config = json.loads((ROOT / "config/research_multisensor_contract.json").read_text(encoding="utf-8"))
        self.protocol = json.loads((ROOT / "config/research_multisensor_gate.json").read_text(encoding="utf-8"))
        self.write("configuration/research_multisensor_contract.json", self.config)
        self.write("configuration/research_multisensor_gate.json", self.protocol)
        self.write("gate-result.json", {"status": "PASS"})
        start, end = 10_000_000_000, 10_066_666_667
        flow = {
            "valid": True, "dt_s": (end - start) * 1.0e-9,
            "acquisition_start_ns": start, "acquisition_end_ns": end,
            "source_image_stamps_ns": [start, end], "frame_id": "impact_down_optical",
            "compensated_integrated_flow_rad": [0.01, -0.002], "quality": 200,
            "raw_integrated_flow_rad": [0.011, -0.001], "track_count": 25,
            "metric_velocity_valid": False, "metric_velocity_mps": None,
            "scale_source": "unavailable", "ground_truth_used": False,
            "dependencies": ["down_camera", "livox_imu"],
        }
        streams = {}
        for name, spec in self.config["streams"].items():
            rate = max(spec["nominal_rate_hz"], self.protocol["minimum_rates_hz"][name])
            span = self.protocol["minimum_source_span_s"] + 10.0
            count = max(self.protocol["minimum_counts"][name] + 1, int(rate * span) + 1)
            streams[name] = {
                "count": count, "rate_hz": (count - 1) / span,
                "first_stamp_ns": start, "last_stamp_ns": start + int(span * 1.0e9),
                "age_p99_s": 0.01, "age_min_s": 0.0, "duplicate_or_backward": 0,
            }
        valid_count = self.protocol["minimum_valid_flow_samples"] + 100
        self.measured = {
            "schema_version": 1, "config_validation": VALIDATE(self.config),
            "hardware_calibration_validated": False, "metric_flow_fusion_ready": False,
            "ground_truth_subscribed": False, "publishes_navigation_or_control": False,
            "errors": {}, "streams": streams,
            "flow": {
                "valid_count": valid_count, "invalid_count": 1,
                "gyro_bad_count": 0, "gyro_verified_count": valid_count, "gyro_pending": 0,
                "maximum_compensated_norm_rad": self.protocol["minimum_compensated_motion_rad"] + 0.01,
            },
            "pixel_stats": {
                name: {"std": 30.0, "distinct_values": 128}
                for name in ("front_camera", "down_camera")
            },
            "camera_info_stamp_match": {
                name: {"image_count": 500, "info_count": 500, "matched": 500, "missing_info": 0}
                for name in ("front_camera", "down_camera")
            },
            "tf_checks": {
                name: {"valid": True} for name in ("lidar", "imu", "front_camera", "down_camera")
            },
            "source_examples": {
                "down_camera": {"stamp_ns": start, "source_now_ns": start + 10_000_000, "receipt_monotonic_s": 100.0},
                "gyro_window": {"start_s": start * 1e-9, "end_s": end * 1e-9,
                                "stamps_s": [9.995 + i * 0.005 for i in range(17)]},
                "valid_flow": flow,
            },
        }
        self.write("multisensor-result.json", self.measured)
        self.bag = {"status": "PASS", "message_count": 5000, "topics": {
            topic: 500 for topic in (
                "/impact/r2/front/image_raw", "/impact/r2/front/camera_info",
                "/impact/r2/down/image_raw", "/impact/r2/down/camera_info", "/impact/r2/flow/measurement",
                "/xq/eval/p4/ground_truth", "/impact/research/command_provenance",
                "/uav1/mavros/setpoint_velocity/cmd_vel", "/impact/ap/mission_status",
            )
        }}
        self.write("bag-audit.json", self.bag)
        process, matrix = self.cli(FAULT_MATRIX, "multisensor-fault-matrix.json")
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertEqual(matrix["status"], "PASS")

    def write(self, name, value):
        (self.run / name).write_text(json.dumps(value), encoding="utf-8")

    def cli(self, script, artifact):
        target = self.run / artifact
        target.unlink(missing_ok=True)
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(ROOT / "src/xq_autonomy") + os.pathsep + environment.get("PYTHONPATH", "")
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        process = subprocess.run(
            [sys.executable, "-B", str(script), "--run-dir", str(self.run)],
            env=environment, cwd=ROOT, capture_output=True, text=True, check=False,
        )
        self.assertTrue(target.is_file(), f"CLI must preserve a failure artifact, not crash:\n{process.stderr}")
        return process, json.loads(target.read_text(encoding="utf-8"))

    def assert_analyzer_fails(self):
        process, result = self.cli(ANALYZER, "multisensor-gate-result.json")
        self.assertNotEqual(process.returncode, 0, f"contradictory/incomplete evidence passed: {result}")
        self.assertEqual(result["status"], "FAIL")

    def test_valid_frozen_fixture_passes_both_clis_and_preserves_inputs(self):
        before = {path: path.read_bytes() for path in self.run.rglob("*") if path.is_file()}
        process, result = self.cli(ANALYZER, "multisensor-gate-result.json")
        self.assertEqual(process.returncode, 0, process.stderr)
        self.assertEqual(result["status"], "PASS")
        self.assertTrue(all(result["checks"].values()))
        self.assertFalse(result["metric_flow_fusion_ready"])
        self.assertEqual(result["hardware_readiness"], "NOT_READY")
        for path, content in before.items():
            self.assertEqual(path.read_bytes(), content)

    def test_missing_monitor_or_bag_evidence_fails_closed(self):
        for name in ("multisensor-result.json", "bag-audit.json"):
            with self.subTest(name=name):
                path = self.run / name
                original = path.read_bytes()
                path.unlink()
                self.assert_analyzer_fails()
                path.write_bytes(original)

    def test_false_hardware_metric_or_independence_claim_is_rejected(self):
        mutations = (
            ("hardware_calibration_validated",),
            ("metric_flow_fusion_ready",),
            ("config_validation", "independent_fusion_ready"),
        )
        for keys in mutations:
            with self.subTest(keys=keys):
                measured = deepcopy(self.measured)
                parent = measured
                for key in keys[:-1]:
                    parent = parent[key]
                parent[keys[-1]] = True
                self.write("multisensor-result.json", measured)
                self.assert_analyzer_fails()

    def test_frozen_config_cannot_contradict_cached_monitor_validation(self):
        for field in ("hardware_calibration_validated", "assume_independent_fusion"):
            with self.subTest(field=field):
                config = deepcopy(self.config)
                config[field] = True
                self.write("configuration/research_multisensor_contract.json", config)
                self.assert_analyzer_fails()

    def test_broken_tf_or_inadequate_gyro_coverage_is_rejected(self):
        for mutation in ("tf", "gyro"):
            with self.subTest(mutation=mutation):
                measured = deepcopy(self.measured)
                if mutation == "tf":
                    measured["tf_checks"]["front_camera"]["valid"] = False
                else:
                    measured["flow"]["gyro_bad_count"] = 1
                    measured["flow"]["gyro_verified_count"] = 0
                self.write("multisensor-result.json", measured)
                self.assert_analyzer_fails()

    def test_missing_or_failed_frozen_matrix_case_cannot_be_hidden_by_pass_status(self):
        original = json.loads((self.run / "multisensor-fault-matrix.json").read_text(encoding="utf-8"))
        for mutation in ("missing", "failed"):
            with self.subTest(mutation=mutation):
                matrix = deepcopy(original)
                case = self.protocol["fault_cases"][-1]
                if mutation == "missing":
                    del matrix["cases"][case]
                else:
                    matrix["cases"][case]["passed"] = False
                self.write("multisensor-fault-matrix.json", matrix)
                self.assert_analyzer_fails()

    def test_failed_bag_audit_or_zero_total_count_is_rejected(self):
        for field, value in (("status", "FAIL"), ("message_count", 0)):
            with self.subTest(field=field):
                bag = deepcopy(self.bag)
                bag[field] = value
                self.write("bag-audit.json", bag)
                self.assert_analyzer_fails()

    def test_future_source_age_is_not_a_small_good_latency(self):
        measured = deepcopy(self.measured)
        measured["streams"]["front_camera"]["age_p99_s"] = -1.0
        measured["streams"]["front_camera"]["age_min_s"] = -1.0
        self.write("multisensor-result.json", measured)
        self.assert_analyzer_fails()

    def test_malformed_monitor_container_still_writes_failure_result(self):
        measured = deepcopy(self.measured)
        measured["config_validation"] = []
        self.write("multisensor-result.json", measured)
        self.assert_analyzer_fails()

    def test_fault_matrix_requires_all_frozen_cases(self):
        protocol = deepcopy(self.protocol)
        protocol["fault_cases"] = protocol["fault_cases"][:-1]
        self.write("configuration/research_multisensor_gate.json", protocol)
        process, result = self.cli(FAULT_MATRIX, "multisensor-fault-matrix.json")
        self.assertNotEqual(process.returncode, 0)
        self.assertEqual(result["status"], "FAIL")

    def test_fault_rejection_must_start_from_valid_gyro_and_flow_examples(self):
        for mutation in ("gyro", "flow"):
            with self.subTest(mutation=mutation):
                measured = deepcopy(self.measured)
                if mutation == "gyro":
                    measured["source_examples"]["gyro_window"]["stamps_s"] = [9.995, 10.075]
                else:
                    measured["source_examples"]["valid_flow"]["dt_s"] = 0.0
                self.write("multisensor-result.json", measured)
                process, result = self.cli(FAULT_MATRIX, "multisensor-fault-matrix.json")
                self.assertNotEqual(process.returncode, 0, "fault rejection was credited for an already invalid baseline")
                self.assertEqual(result["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
