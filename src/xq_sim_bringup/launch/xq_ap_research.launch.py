"""Staged AP research stack: base before takeoff, mapping/planning after hover."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    config = json.loads(Path(value("config_file")).read_text())
    autonomy = Path(get_package_share_directory("xq_autonomy"))
    bridge = Path(get_package_share_directory("xq_gz_bridge"))
    lio = Path(get_package_share_directory("xq_fast_lio"))
    sim = {"use_sim_time": True}
    mode = value("stage")
    if mode == "base":
        transforms = []
        for child, xyz in (("livox_frame", (0.04, 0.0, 0.12)), ("livox_imu", (0.0, 0.0, 0.0))):
            # FAST-LIO owns xq_lio_map -> livox_imu. The coincident base must
            # be its child; making the IMU a base child gives it two parents.
            parent_frame, child_frame = (("livox_imu", "xq_base_link")
                                         if child == "livox_imu" else ("xq_base_link", child))
            transforms.append(Node(
                package="tf2_ros", executable="static_transform_publisher",
                name="impact_ap_base_to_" + child,
                arguments=["--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
                           "--roll", "0", "--pitch", "0", "--yaw", "0",
                           "--frame-id", parent_frame, "--child-frame-id", child_frame],
                parameters=[sim], output="screen"))
        execution = {
            "max_speed_mps": config["maximum_speed_mps"],
            "max_acceleration_mps2": config["executor_max_acceleration_mps2"],
            **{key: config[key] for key in (
                "max_command_age_s", "max_source_odom_age_s", "max_external_nav_status_age_s",
                "max_future_skew_s", "map_to_fcu_enu_yaw_rad", "maximum_frame_yaw_error_rad",
                "freeze_duration_s")},
            "freeze_after_moving_s": config["freeze_after_moving_s"] if value("replay_test") == "true" else 0.0,
        }
        return transforms + [
            Node(package="xq_gz_bridge", executable="xq_gz_bridge_node", name="xq_gz_bridge_node",
                 parameters=[str(bridge / "config/p4_external_nav.yaml"), {
                     "gz_ground_truth_topic": "/world/" + value("world_name") + "/pose/info",
                 }], output="screen"),
            Node(package="xq_fast_lio", executable="fastlio_mapping", name="xq_fast_lio",
                 parameters=[str(lio / "config/xq_p4.yaml"), {
                     **sim, "integrity_geometry.enable": True, "publish.scan_publish_en": True}], output="screen"),
            Node(package="xq_autonomy", executable="xq_p4_external_nav", name="xq_p4_external_nav",
                 parameters=[{"use_sim_time": False}], output="screen"),
            Node(package="xq_autonomy", executable="impact_ap_execution", name="impact_ap_execution",
                 parameters=[{**sim, **execution}], output="screen"),
            Node(package="xq_autonomy", executable="impact_ap_research_evaluator", name="impact_ap_research_evaluator",
                 parameters=[{**sim, "result_file": value("evaluation_result_file")}], output="screen"),
            Node(package="xq_autonomy", executable="impact_ap_research_mission", name="impact_ap_research_mission",
                 parameters=[{
                     "use_sim_time": False, "result_file": value("mission_result_file"),
                     "mission_timeout_s": config["mission_timeout_s"],
                     "research_timeout_s": config["research_timeout_s"],
                     "takeoff_altitude_m": config["takeoff_altitude_m"], "hover_duration_s": 3.0,
                     "mission_distance_m": config["mission_distance_m"],
                     "minimum_progress_m": config["minimum_progress_m"],
                     "require_freeze_test": value("replay_test") == "true",
                 }], output="screen"),
        ]
    if mode == "mapping":
        return [
            Node(package="xq_autonomy", executable="xq_p6_directional_integrity", name="xq_p6_directional_integrity",
                 parameters=[str(autonomy / "config/p6_integrity.yaml"), {
                     **sim, "information_memory_horizon_s": 0.0}], output="screen"),
            Node(package="xq_autonomy", executable="xq_p10_information_map", name="xq_p10_information_map",
                 parameters=[sim], output="screen"),
            Node(package="xq_autonomy", executable="xq_p12_dynamic_map", name="xq_p12_dynamic_map",
                 parameters=[{
                     **sim, "variant": "integrity_constrained",
                     "mission_distance_m": config["mission_distance_m"],
                     "static_warmup_s": config["static_warmup_s"],
                     "dynamic_path_query_mode": "active_trajectory",
                     "minimum_dynamic_cluster_points": 5, "dynamic_cluster_radius_m": 0.45,
                 }], output="screen"),
        ]
    if mode == "planning":
        calibration = Path(value("calibration_file"))
        calibration_sha = hashlib.sha256(calibration.read_bytes()).hexdigest()
        return [
            Node(package="xq_autonomy", executable="xq_p11_integrity_exploration", name="xq_p11_integrity_exploration",
                 remappings=[("/xq/p5/cloud_map", "/mapping/p12/static_voxels"),
                             ("/xq/p11/flight_status", "/xq/p12/flight_status")],
                 parameters=[{
                     **sim, "calibration_file": str(calibration), "calibration_sha256": calibration_sha,
                     "visibility_radius_m": 2.8, "information_scale": 2500.0,
                     "minimum_prediction_variance_m2": 0.00001, "margin_reserve_m": 0.10,
                     "collision_probability_limit": 0.01, "energy_remaining": 32.0,
                     "maximum_acceleration_mps2": 0.8,
                 }], output="screen"),
            Node(package="xq_autonomy", executable="xq_p13_flight_controller", name="xq_p13_flight_controller",
                 # P4 bridge also has its direct Gazebo velocity topic disabled.
                 remappings=[("/xq/p3/cmd_vel", "/impact/unused_legacy_velocity")],
                 parameters=[{
                     **sim, "variant": "integrity_constrained", "initialization_s": 0.0,
                     "mission_distance_m": config["mission_distance_m"],
                     "trajectory_distance_m": config["mission_distance_m"],
                     "trajectory_duration_s": config["trajectory_duration_s"],
                     "maximum_speed_mps": config["maximum_speed_mps"], "maximum_acceleration_mps2": 0.8,
                     **{key: config[key] for key in (
                         "candidate_generation_mode", "candidate_metric_source", "lattice_lateral_levels",
                         "lattice_vertical_levels", "lateral_offset_m", "runtime_integrity_guard_mode",
                         "runtime_integrity_margin_m", "planner_delay_ms")},
                     "runtime_integrity_calibration_file": str(calibration),
                     "enable_vertical_candidate": False, "enable_diagonal_vertical_candidates": False,
                     "dynamic_path_query_mode": "active_trajectory", "minimum_dynamic_cluster_points": 5,
                     "dynamic_cluster_radius_m": 0.45, "latency_profile": "r1_measured_no_injection",
                     "integrity_recovery_speed_mps": 0.0, "integrity_recovery_max_offset_m": 0.0,
                 }], output="screen"),
        ]
    raise ValueError(f"Unsupported AP research stage: {mode}")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("stage"), DeclareLaunchArgument("config_file"),
        DeclareLaunchArgument("mission_result_file", default_value=""),
        DeclareLaunchArgument("evaluation_result_file", default_value=""),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("replay_test", default_value="false"),
        DeclareLaunchArgument("world_name", default_value="xq_p4_external_nav"),
        OpaqueFunction(function=_nodes),
    ])
