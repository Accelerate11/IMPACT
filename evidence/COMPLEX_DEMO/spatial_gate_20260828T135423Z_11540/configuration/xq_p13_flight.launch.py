"""P13 two-profile latency-aware safety flight in the unchanged P12 world."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _gazebo(context):
    command = ["gz", "sim", "-r", "-s"]
    if LaunchConfiguration("headless_rendering").perform(context).strip().lower() in {
        "1", "true", "yes", "on",
    }:
        command.append("--headless-rendering")
    command += ["-v", "2"]
    record_path = LaunchConfiguration("gz_record_path").perform(context).strip()
    if record_path:
        command += ["--record-path", record_path, "--record-period", "0.02"]
    command.append(LaunchConfiguration("world_file").perform(context))
    return [ExecuteProcess(cmd=command, name="xq_p13_gazebo", output="screen")]


def f(name: str):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def i(name: str):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def b(name: str):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description() -> LaunchDescription:
    assets = Path(get_package_share_directory("xq_gz_assets"))
    bridge = Path(get_package_share_directory("xq_gz_bridge"))
    bridge_prefix = Path(get_package_prefix("xq_gz_bridge"))
    fast_lio = Path(get_package_share_directory("xq_fast_lio"))
    autonomy = Path(get_package_share_directory("xq_autonomy"))
    resource_path = os.pathsep.join((str(assets / "models"), str(assets / "worlds")))
    arguments = [
        DeclareLaunchArgument(
            "world_file",
            default_value=str(assets / "worlds" / "xq_p12_dynamic_obstacle.sdf"),
        ),
        DeclareLaunchArgument("headless_rendering", default_value="true"),
        DeclareLaunchArgument("calibration_file"),
        DeclareLaunchArgument("calibration_sha256", default_value=""),
        DeclareLaunchArgument("thresholds_file"),
        DeclareLaunchArgument("p12_thresholds_file"),
        DeclareLaunchArgument("p12_result_file"),
        DeclareLaunchArgument("p13_result_file"),
        DeclareLaunchArgument("flight_variant", default_value="integrity_constrained"),
        DeclareLaunchArgument("enable_p11_evaluator", default_value="false"),
        DeclareLaunchArgument("p11_result_file", default_value=""),
        DeclareLaunchArgument("gz_record_path", default_value=""),
        DeclareLaunchArgument("latency_profile", default_value="low_50ms"),
        DeclareLaunchArgument("planner_delay_ms", default_value="50.0"),
        DeclareLaunchArgument("voxel_size_m", default_value="0.25"),
        DeclareLaunchArgument("dynamic_ttl_s", default_value="3.0"),
        DeclareLaunchArgument("dynamic_occupied_threshold", default_value="0.35"),
        DeclareLaunchArgument("dynamic_clear_threshold", default_value="0.08"),
        DeclareLaunchArgument("static_confirmation_hits", default_value="6"),
        DeclareLaunchArgument("free_confirmation_rays", default_value="3"),
        DeclareLaunchArgument("path_clearance_radius_m", default_value="0.70"),
        DeclareLaunchArgument("planning_lookahead_m", default_value="4.0"),
        DeclareLaunchArgument("clear_confirmation_s", default_value="1.0"),
        DeclareLaunchArgument("mission_distance_m", default_value="24.0"),
        DeclareLaunchArgument("lateral_offset_m", default_value="0.60"),
        DeclareLaunchArgument(
            "lateral_candidate_shape", default_value="return_to_center"
        ),
        DeclareLaunchArgument("enable_vertical_candidate", default_value="false"),
        DeclareLaunchArgument("vertical_offset_m", default_value="0.70"),
        DeclareLaunchArgument("geometric_clearance_m", default_value="0.82"),
        DeclareLaunchArgument("fixed_buffer_m", default_value="0.58"),
        DeclareLaunchArgument("protection_level_m", default_value="0.10"),
        DeclareLaunchArgument("required_margin_m", default_value="0.06"),
        DeclareLaunchArgument("maximum_speed_mps", default_value="0.42"),
        DeclareLaunchArgument("maximum_acceleration_mps2", default_value="0.8"),
        DeclareLaunchArgument("rejected_candidate_retry_s", default_value="2.0"),
        DeclareLaunchArgument("maximum_candidate_retries", default_value="6"),
        DeclareLaunchArgument("integrity_recovery_speed_mps", default_value="0.08"),
        DeclareLaunchArgument("integrity_recovery_max_offset_m", default_value="0.35"),
        DeclareLaunchArgument("integrity_recovery_half_period_s", default_value="3.0"),
        DeclareLaunchArgument("obstacle_x_m", default_value="-4.5"),
        DeclareLaunchArgument("obstacle_park_y_m", default_value="3.4"),
        DeclareLaunchArgument("obstacle_blocked_y_m", default_value="0.0"),
        DeclareLaunchArgument("obstacle_z_m", default_value="1.0"),
        DeclareLaunchArgument("obstacle_enter_start_s", default_value="18.0"),
        DeclareLaunchArgument("obstacle_enter_end_s", default_value="22.0"),
        DeclareLaunchArgument("obstacle_leave_start_s", default_value="36.0"),
        DeclareLaunchArgument("obstacle_leave_end_s", default_value="40.0"),
    ]
    common_environment = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            os.pathsep.join(
                value
                for value in (
                    str(bridge_prefix / "lib"),
                    os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                )
                if value
            ),
        ),
        SetEnvironmentVariable("SDF_PATH", str(assets / "models")),
    ]
    nodes = [
        OpaqueFunction(function=_gazebo),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="xq_p13_base_to_livox",
            arguments=[
                "--x", "0.04", "--y", "0.0", "--z", "0.135",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "xq_base_link", "--child-frame-id", "livox_frame",
            ],
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="xq_p13_base_to_livox_imu",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.015",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "xq_base_link", "--child-frame-id", "livox_imu",
            ],
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="xq_gz_bridge",
            executable="xq_gz_bridge_node",
            name="xq_gz_bridge_node",
            parameters=[str(bridge / "config" / "p3_fast_lio.yaml")],
        ),
        Node(
            package="xq_fast_lio",
            executable="fastlio_mapping",
            name="xq_fast_lio",
            parameters=[
                str(fast_lio / "config" / "xq_p3.yaml"),
                {
                    "use_sim_time": True,
                    "integrity_geometry.enable": True,
                    "publish.scan_publish_en": True,
                },
            ],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p6_directional_integrity",
            name="xq_p6_directional_integrity",
            parameters=[str(autonomy / "config" / "p6_integrity.yaml")],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p10_information_map",
            name="xq_p10_information_map",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p11_integrity_exploration",
            name="xq_p11_integrity_exploration",
            remappings=[
                ("/xq/p5/cloud_map", "/mapping/p12/static_voxels"),
                ("/xq/p11/flight_status", "/xq/p12/flight_status"),
            ],
            parameters=[{
                "use_sim_time": True,
                "calibration_file": LaunchConfiguration("calibration_file"),
                "calibration_sha256": LaunchConfiguration("calibration_sha256"),
                "visibility_radius_m": 2.8,
                "information_scale": 2500.0,
                "minimum_prediction_variance_m2": 0.00001,
                "margin_reserve_m": 0.10,
                "collision_probability_limit": 0.01,
                "energy_remaining": 32.0,
                "information_weight": 1.0,
                "travel_time_weight": 0.01,
                "energy_weight": 0.005,
            }],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p12_dynamic_map",
            name="xq_p12_dynamic_map",
            parameters=[{
                "use_sim_time": True,
                "voxel_size_m": f("voxel_size_m"),
                "dynamic_ttl_s": f("dynamic_ttl_s"),
                "dynamic_occupied_threshold": f("dynamic_occupied_threshold"),
                "dynamic_clear_threshold": f("dynamic_clear_threshold"),
                "static_confirmation_hits": i("static_confirmation_hits"),
                "free_confirmation_rays": i("free_confirmation_rays"),
                "path_clearance_radius_m": f("path_clearance_radius_m"),
                "planning_lookahead_m": f("planning_lookahead_m"),
                "mission_distance_m": f("mission_distance_m"),
                "lateral_offset_m": f("lateral_offset_m"),
                "lateral_candidate_shape": LaunchConfiguration(
                    "lateral_candidate_shape"
                ),
            }],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p12_obstacle_driver",
            name="xq_p12_obstacle_driver",
            parameters=[{
                "use_sim_time": True,
                "obstacle_x_m": f("obstacle_x_m"),
                "park_y_m": f("obstacle_park_y_m"),
                "blocked_y_m": f("obstacle_blocked_y_m"),
                "obstacle_z_m": f("obstacle_z_m"),
                "enter_start_s": f("obstacle_enter_start_s"),
                "enter_end_s": f("obstacle_enter_end_s"),
                "leave_start_s": f("obstacle_leave_start_s"),
                "leave_end_s": f("obstacle_leave_end_s"),
            }],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p13_flight_controller",
            name="xq_p13_flight_controller",
            parameters=[{
                "use_sim_time": True,
                "variant": LaunchConfiguration("flight_variant"),
                "direct_information_gain": 1.0,
                "safe_information_gain": 0.75,
                "mission_distance_m": f("mission_distance_m"),
                "lateral_offset_m": f("lateral_offset_m"),
                "lateral_candidate_shape": LaunchConfiguration(
                    "lateral_candidate_shape"
                ),
                "enable_vertical_candidate": b("enable_vertical_candidate"),
                "vertical_offset_m": f("vertical_offset_m"),
                "path_clearance_radius_m": f("path_clearance_radius_m"),
                "planning_lookahead_m": f("planning_lookahead_m"),
                "clear_confirmation_s": f("clear_confirmation_s"),
                "latency_profile": LaunchConfiguration("latency_profile"),
                "planner_delay_ms": f("planner_delay_ms"),
                "geometric_clearance_m": f("geometric_clearance_m"),
                "fixed_buffer_m": f("fixed_buffer_m"),
                "protection_level_m": f("protection_level_m"),
                "required_margin_m": f("required_margin_m"),
                "maximum_speed_mps": f("maximum_speed_mps"),
                "maximum_acceleration_mps2": f("maximum_acceleration_mps2"),
                "rejected_candidate_retry_s": f("rejected_candidate_retry_s"),
                "maximum_candidate_retries": i("maximum_candidate_retries"),
                "integrity_recovery_speed_mps": f("integrity_recovery_speed_mps"),
                "integrity_recovery_max_offset_m": f("integrity_recovery_max_offset_m"),
                "integrity_recovery_half_period_s": f("integrity_recovery_half_period_s"),
            }],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p11_flight_evaluator",
            name="xq_complex_p11_flight_evaluator",
            condition=IfCondition(LaunchConfiguration("enable_p11_evaluator")),
            remappings=[
                ("/xq/p5/cloud_map", "/mapping/p12/static_voxels"),
                ("/xq/p11/flight_status", "/xq/p12/flight_status"),
            ],
            parameters=[{
                "use_sim_time": True,
                "variant": LaunchConfiguration("flight_variant"),
                "result_file": LaunchConfiguration("p11_result_file"),
                "calibration_file": LaunchConfiguration("calibration_file"),
                "mission_distance_m": f("mission_distance_m"),
            }],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p12_flight_evaluator",
            name="xq_p12_flight_evaluator",
            parameters=[{
                "use_sim_time": True,
                "result_file": LaunchConfiguration("p12_result_file"),
                "thresholds_file": LaunchConfiguration("p12_thresholds_file"),
            }],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p13_flight_evaluator",
            name="xq_p13_flight_evaluator",
            parameters=[{
                "use_sim_time": True,
                "result_file": LaunchConfiguration("p13_result_file"),
                "thresholds_file": LaunchConfiguration("thresholds_file"),
                "expected_planner_delay_ms": f("planner_delay_ms"),
                "latency_profile": LaunchConfiguration("latency_profile"),
            }],
        ),
    ]
    return LaunchDescription(arguments + common_environment + nodes)
