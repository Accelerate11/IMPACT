"""P14 deterministic fault matrix on the accepted P13/P12 Gazebo stack."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _gazebo(context):
    command = ["gz", "sim", "-r", "-s", "--headless-rendering", "-v", "2"]
    record_path = LaunchConfiguration("gz_record_path").perform(context).strip()
    if record_path:
        command += ["--record-path", record_path, "--record-period", "0.02"]
    command.append(LaunchConfiguration("world_file").perform(context))
    return [ExecuteProcess(cmd=command, name="xq_p14_gazebo", output="screen")]


def f(name: str):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def i(name: str):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description() -> LaunchDescription:
    assets = Path(get_package_share_directory("xq_gz_assets"))
    bridge = Path(get_package_share_directory("xq_gz_bridge"))
    bridge_prefix = Path(get_package_prefix("xq_gz_bridge"))
    fast_lio = Path(get_package_share_directory("xq_fast_lio"))
    autonomy = Path(get_package_share_directory("xq_autonomy"))
    resource_path = os.pathsep.join((str(assets / "models"), str(assets / "worlds")))
    matrix_only = IfCondition(PythonExpression(["'", LaunchConfiguration("trial"), "' == 'matrix'"]))
    arguments = [
        DeclareLaunchArgument("world_file", default_value=str(assets / "worlds" / "xq_p12_dynamic_obstacle.sdf")),
        DeclareLaunchArgument("trial", default_value="matrix"),
        DeclareLaunchArgument("schedule_file"),
        DeclareLaunchArgument("thresholds_file"),
        DeclareLaunchArgument("calibration_file"),
        DeclareLaunchArgument("calibration_sha256", default_value=""),
        DeclareLaunchArgument("p12_thresholds_file"),
        DeclareLaunchArgument("p12_result_file", default_value="/tmp/p14-p12-retention.json"),
        DeclareLaunchArgument("p13_thresholds_file"),
        DeclareLaunchArgument("p13_result_file", default_value="/tmp/p14-p13-retention.json"),
        DeclareLaunchArgument("p14_result_file"),
        DeclareLaunchArgument("gz_record_path", default_value=""),
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
        DeclareLaunchArgument("geometric_clearance_m", default_value="0.82"),
        DeclareLaunchArgument("fixed_buffer_m", default_value="0.58"),
        DeclareLaunchArgument("protection_level_m", default_value="0.10"),
        DeclareLaunchArgument("required_margin_m", default_value="0.06"),
        DeclareLaunchArgument("maximum_speed_mps", default_value="0.42"),
        DeclareLaunchArgument("maximum_acceleration_mps2", default_value="0.8"),
    ]
    environment = [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", os.pathsep.join(
            value for value in (str(bridge_prefix / "lib"), os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")) if value
        )),
        SetEnvironmentVariable("SDF_PATH", str(assets / "models")),
    ]
    common = {"use_sim_time": True}
    nodes = [
        OpaqueFunction(function=_gazebo),
        Node(package="tf2_ros", executable="static_transform_publisher", name="xq_p14_base_to_livox",
             arguments=["--x", "0.04", "--y", "0.0", "--z", "0.135", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "xq_base_link", "--child-frame-id", "livox_frame"], parameters=[common]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="xq_p14_base_to_livox_imu",
             arguments=["--x", "0.0", "--y", "0.0", "--z", "0.015", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "xq_base_link", "--child-frame-id", "livox_imu"], parameters=[common]),
        Node(package="xq_gz_bridge", executable="xq_gz_bridge_node", name="xq_gz_bridge_node",
             parameters=[str(bridge / "config" / "p3_fast_lio.yaml")],
             remappings=[("/livox/lidar", "/impact/raw/lidar"), ("/livox/imu", "/impact/raw/imu")]),
        Node(package="impact_fault_injection", executable="impact_sensor_proxy", name="impact_sensor_proxy", parameters=[common]),
        Node(package="impact_fault_injection", executable="impact_fault_injector", name="impact_fault_injector",
             parameters=[{"use_sim_time": True, "schedule_file": LaunchConfiguration("schedule_file")}]),
        Node(package="xq_fast_lio", executable="fastlio_mapping", name="xq_fast_lio",
             parameters=[str(fast_lio / "config" / "xq_p3.yaml"), {"use_sim_time": True, "integrity_geometry.enable": True, "publish.scan_publish_en": True}],
             remappings=[("/localization/odom", "/impact/raw/odom")]),
        Node(package="xq_autonomy", executable="xq_p6_directional_integrity", name="xq_p6_directional_integrity",
             parameters=[str(autonomy / "config" / "p6_integrity.yaml")]),
        Node(package="xq_autonomy", executable="xq_p10_information_map", name="xq_p10_information_map", parameters=[common]),
        Node(package="xq_autonomy", executable="xq_p11_integrity_exploration", name="xq_p11_integrity_exploration",
             remappings=[("/xq/p5/cloud_map", "/mapping/p12/static_voxels")],
             parameters=[{"use_sim_time": True, "calibration_file": LaunchConfiguration("calibration_file"),
                          "calibration_sha256": LaunchConfiguration("calibration_sha256"), "visibility_radius_m": 2.8,
                          "information_scale": 2500.0, "minimum_prediction_variance_m2": 0.00001,
                          "margin_reserve_m": 0.10, "collision_probability_limit": 0.01,
                          "energy_remaining": 32.0, "information_weight": 1.0,
                          "travel_time_weight": 0.01, "energy_weight": 0.005}]),
        Node(package="xq_autonomy", executable="xq_p12_dynamic_map", name="xq_p12_dynamic_map",
             parameters=[{"use_sim_time": True, "voxel_size_m": f("voxel_size_m"), "dynamic_ttl_s": f("dynamic_ttl_s"),
                          "dynamic_occupied_threshold": f("dynamic_occupied_threshold"), "dynamic_clear_threshold": f("dynamic_clear_threshold"),
                          "static_confirmation_hits": i("static_confirmation_hits"), "free_confirmation_rays": i("free_confirmation_rays"),
                          "path_clearance_radius_m": f("path_clearance_radius_m"), "planning_lookahead_m": f("planning_lookahead_m"),
                          "mission_distance_m": f("mission_distance_m")}]),
        Node(package="xq_autonomy", executable="xq_p12_obstacle_driver", name="xq_p12_obstacle_driver",
             parameters=[{"use_sim_time": True, "enter_start_s": 24.0, "enter_end_s": 28.0,
                          "leave_start_s": 44.0, "leave_end_s": 48.0}]),
        Node(package="impact_fault_injection", executable="impact_p14_controller", name="impact_p14_controller",
             parameters=[{"use_sim_time": True, "p14_trial": LaunchConfiguration("trial"), "variant": "integrity_constrained",
                          "direct_information_gain": 1.0, "safe_information_gain": 0.75,
                          "mission_distance_m": f("mission_distance_m"), "path_clearance_radius_m": f("path_clearance_radius_m"),
                          "planning_lookahead_m": f("planning_lookahead_m"), "clear_confirmation_s": f("clear_confirmation_s"),
                          "latency_profile": "p14_low_50ms", "planner_delay_ms": f("planner_delay_ms"),
                          "geometric_clearance_m": f("geometric_clearance_m"), "fixed_buffer_m": f("fixed_buffer_m"),
                          "protection_level_m": f("protection_level_m"), "required_margin_m": f("required_margin_m"),
                          "maximum_speed_mps": f("maximum_speed_mps"), "maximum_acceleration_mps2": f("maximum_acceleration_mps2"),
                          "rejected_candidate_retry_s": 1.0, "maximum_candidate_retries": 50}]),
        Node(package="impact_fault_injection", executable="impact_p14_evaluator", name="impact_p14_evaluator",
             parameters=[{"use_sim_time": True, "trial": LaunchConfiguration("trial"),
                          "result_file": LaunchConfiguration("p14_result_file"), "thresholds_file": LaunchConfiguration("thresholds_file")}]),
        Node(package="xq_autonomy", executable="xq_p12_flight_evaluator", name="xq_p14_p12_retention_evaluator",
             condition=matrix_only, parameters=[{"use_sim_time": True, "result_file": LaunchConfiguration("p12_result_file"),
                                                  "thresholds_file": LaunchConfiguration("p12_thresholds_file")}]),
        Node(package="xq_autonomy", executable="xq_p13_flight_evaluator", name="xq_p14_p13_retention_evaluator",
             condition=matrix_only, parameters=[{"use_sim_time": True, "result_file": LaunchConfiguration("p13_result_file"),
                                                  "thresholds_file": LaunchConfiguration("p13_thresholds_file"),
                                                  "expected_planner_delay_ms": f("planner_delay_ms"), "latency_profile": "p14_low_50ms"}]),
    ]
    return LaunchDescription(arguments + environment + nodes)
