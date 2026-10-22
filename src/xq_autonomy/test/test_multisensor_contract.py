"""Contract tests need no ROS runtime or numerical-library installation."""

from copy import deepcopy
import json
from pathlib import Path
import unittest

from xq_autonomy.multisensor_contract import (
    StreamClockGuard, check_camera_model, check_gyro_coverage,
    check_independent_fusion, check_rigid_transform, flow_metric_velocity,
    map_acquisition_time, validate_contract_config,
)


CONFIG_PATH = Path(__file__).resolve().parents[3] / "config/research_multisensor_contract.json"


class MultisensorContractTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

    def test_simulation_contract_passes_but_hardware_and_independence_do_not(self):
        result = validate_contract_config(self.config)
        self.assertEqual(result["status"], "PASS", result["errors"])
        self.assertEqual(result["hardware_readiness"]["status"], "NOT_READY")
        self.assertFalse(result["independent_fusion_ready"])
        self.assertEqual(len(result["provenance_audit"]["shared_leaf_dependencies"]), 3)
        hardware = deepcopy(self.config)
        hardware["profile"] = "hardware"
        hardware["streams"]["lidar"]["timestamp_semantics"] = "scan_start"
        self.assertEqual(validate_contract_config(hardware)["status"], "NOT_READY")
        self.config["assume_independent_fusion"] = True
        self.assertEqual(validate_contract_config(self.config)["status"], "NOT_READY")

    def test_simulated_lidar_snapshot_cannot_claim_real_scan_timing(self):
        lidar = self.config["streams"]["lidar"]
        self.assertEqual(lidar["timestamp_semantics"], "snapshot_instant")
        self.assertFalse(lidar["per_point_timing_available"])
        self.assertFalse(lidar["real_mid360_scan_pattern"])
        lidar["timestamp_semantics"] = "scan_start"
        self.assertEqual(validate_contract_config(self.config)["status"], "FAIL")
        lidar["timestamp_semantics"] = "snapshot_instant"
        lidar["per_point_timing_available"] = True
        self.assertEqual(validate_contract_config(self.config)["status"], "FAIL")

    def test_transform_rejects_reflection_frame_inverse_and_numeric_inverse(self):
        transform = deepcopy(self.config["streams"]["front_camera"]["body_from_sensor"])
        kwargs = {"body_frame": "xq_base_link", "sensor_frame": "impact_front_optical"}
        self.assertEqual(check_rigid_transform(transform, **kwargs)["status"], "PASS")
        reference = deepcopy(transform["rotation"])
        transform["rotation"] = [list(row) for row in zip(*reference)]
        self.assertEqual(check_rigid_transform(transform, expected_rotation=reference, **kwargs)["status"], "FAIL")
        transform["parent_frame"], transform["child_frame"] = transform["child_frame"], transform["parent_frame"]
        self.assertEqual(check_rigid_transform(transform, **kwargs)["status"], "FAIL")
        transform = deepcopy(self.config["streams"]["front_camera"]["body_from_sensor"])
        transform["rotation"] = [[1, 0, 0], [0, 1, 0], [0, 0, -1]]
        self.assertEqual(check_rigid_transform(transform, **kwargs)["status"], "FAIL")

    def test_only_validated_acquisition_clock_mapping_is_accepted(self):
        mappings = self.config["clock_mappings"]
        self.assertEqual(map_acquisition_time(2.0, clock_domain="simulation", clock_mappings=mappings), 2.0)
        for stamp, domain in ((0.0, "simulation"), (1.0, "unknown")):
            with self.assertRaises(ValueError):
                map_acquisition_time(stamp, clock_domain=domain, clock_mappings=mappings)
        mappings["simulation"]["acquisition_not_receipt"] = False
        with self.assertRaises(ValueError):
            map_acquisition_time(2.0, clock_domain="simulation", clock_mappings=mappings)

    def test_duplicate_does_not_refresh_wall_freshness_and_rollback_latches(self):
        guard = StreamClockGuard(0.25)
        self.assertEqual(guard.observe(0.0, receipt_monotonic_s=10, now_source_s=0, now_monotonic_s=10)["status"], "WAITING")
        self.assertTrue(guard.observe(1.0, receipt_monotonic_s=10, now_source_s=1, now_monotonic_s=10)["accepted"])
        self.assertFalse(guard.observe(1.0, receipt_monotonic_s=10.2, now_source_s=1, now_monotonic_s=10.2)["accepted"])
        self.assertEqual(guard.status(now_source_s=1, now_monotonic_s=10.3)["status"], "STALE")
        self.assertEqual(guard.status(now_source_s=0.9, now_monotonic_s=10.4)["status"], "FAIL")
        self.assertEqual(guard.status(now_source_s=2, now_monotonic_s=10.5)["status"], "FAIL")

    def test_invalid_camera_model_or_receipt_semantics_fail_config(self):
        camera = self.config["streams"]["front_camera"]
        self.assertEqual(check_camera_model(camera["intrinsics"], axis_convention="ros_optical")["status"], "PASS")
        camera["intrinsics"]["fx"] = 0.0
        camera["timestamp_source"] = "receipt"
        self.assertEqual(validate_contract_config(self.config)["status"], "FAIL")
        self.assertEqual(check_camera_model(camera["intrinsics"], axis_convention="body_flu")["status"], "FAIL")

    def test_gyro_window_requires_brackets_and_no_gap(self):
        result = check_gyro_coverage(1.0, 1.04, [0.995, 1.01, 1.025, 1.04], max_gap_s=0.02)
        self.assertTrue(result["covered"])
        self.assertTrue(result["valid"])
        self.assertFalse(check_gyro_coverage(1.0, 1.04, [1.01, 1.025, 1.04], 0.02)["covered"])
        self.assertFalse(check_gyro_coverage(1.0, 1.04, [0.995, 1.04], 0.02)["covered"])
        self.assertFalse(check_gyro_coverage(1.0, 1.04, [], 0.02)["valid"])

    def test_lidar_scaled_flow_must_include_lidar_in_all_derived_dependencies(self):
        flow = self.config["streams"]["flow"]
        flow["range_source"] = "lidar"
        flow["metric_velocity_available"] = True
        self.assertEqual(validate_contract_config(self.config)["status"], "FAIL")
        flow["leaf_dependencies"].append("lidar")
        self.assertEqual(validate_contract_config(self.config)["status"], "FAIL")
        self.config["provenance"]["flow"]["leaf_dependencies"].append("lidar")
        self.assertEqual(validate_contract_config(self.config)["status"], "PASS")

    def test_unknown_range_cannot_become_metric_velocity(self):
        for distance in (None, -1.0, 0.0, float("nan")):
            result = flow_metric_velocity(0.01, -0.02, integration_time_s=0.1, distance_m=distance, quality=200)
            self.assertEqual(result["status"], "FAIL")
            self.assertFalse(result["metric_velocity_available"])
        angular_only = flow_metric_velocity(0.01, -0.02, integration_time_s=0.1, distance_m=None, quality=200, metric_requested=False)
        self.assertEqual(angular_only["status"], "PASS")
        self.assertFalse(angular_only["metric_velocity_available"])

    def test_internal_optical_sign_and_no_double_gyro_subtraction(self):
        result = flow_metric_velocity(0.01, -0.02, integration_time_s=0.1, distance_m=2.0, quality=200)
        self.assertAlmostEqual(result["metric_velocity_optical_mps"][0], -0.2)
        self.assertAlmostEqual(result["metric_velocity_optical_mps"][1], 0.4)
        self.assertEqual(result["calibration_status"], "NOT_CALIBRATED")
        self.assertFalse(result["uncertainty_known"])
        double = flow_metric_velocity(0.01, -0.02, integration_time_s=0.1, distance_m=2.0, quality=200, gyro_integral_xy=(0.0, 0.0))
        self.assertEqual(double["status"], "FAIL")

    def test_shared_dependencies_and_repeated_input_do_not_prove_independence(self):
        shared = check_independent_fusion(self.config["provenance"], assume_independent=True)
        self.assertFalse(shared["streaming_allowed"])
        self.assertFalse(shared["independent_fusion_ready"])
        self.assertTrue(shared["shared_leaf_dependencies"])
        self.assertFalse(check_independent_fusion({"a": ["camera"], "b": ["lidar"]})["independent_fusion_ready"])
        self.assertEqual(check_independent_fusion({"a": ["imu", "imu"], "b": ["camera"]})["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
