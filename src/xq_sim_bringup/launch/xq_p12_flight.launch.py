"""P12 LiDAR dynamic-map, replan, TTL-reopen, full-corridor Gazebo Gate."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _gazebo(context):
    command = ["gz", "sim", "-r", "-s", "--headless-rendering", "-v", "2"]
    record_path = LaunchConfiguration("gz_record_path").perform(context).strip()
    if record_path:
        command += ["--record-path", record_path, "--record-period", "0.02"]
    command.append(LaunchConfiguration("world_file").perform(context))
    return [ExecuteProcess(cmd=command, name="xq_p12_gazebo", output="screen")]


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
    arguments = [
        DeclareLaunchArgument("world_file", default_value=str(assets / "worlds" / "xq_p12_dynamic_obstacle.sdf")),
        DeclareLaunchArgument("calibration_file"),
        DeclareLaunchArgument("calibration_sha256", default_value=""),
        DeclareLaunchArgument("thresholds_file"),
        DeclareLaunchArgument("result_file"),
        DeclareLaunchArgument("gz_record_path", default_value=""),
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
    ]
    return LaunchDescription(
        arguments
        + [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable(
                "GZ_SIM_SYSTEM_PLUGIN_PATH",
                os.pathsep.join(
                    value for value in (
                        str(bridge_prefix / "lib"),
                        os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
                    ) if value
                ),
            ),
            SetEnvironmentVariable("SDF_PATH", str(assets / "models")),
            OpaqueFunction(function=_gazebo),
            Node(
                package="tf2_ros", executable="static_transform_publisher",
                name="xq_p12_base_to_livox",
                arguments=["--x", "0.04", "--y", "0.0", "--z", "0.135", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "xq_base_link", "--child-frame-id", "livox_frame"],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros", executable="static_transform_publisher",
                name="xq_p12_base_to_livox_imu",
                arguments=["--x", "0.0", "--y", "0.0", "--z", "0.015", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "xq_base_link", "--child-frame-id", "livox_imu"],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="xq_gz_bridge", executable="xq_gz_bridge_node",
                name="xq_gz_bridge_node", parameters=[str(bridge / "config" / "p3_fast_lio.yaml")],
            ),
            Node(
                package="xq_fast_lio", executable="fastlio_mapping", name="xq_fast_lio",
                parameters=[str(fast_lio / "config" / "xq_p3.yaml"), {"use_sim_time": True, "integrity_geometry.enable": True, "publish.scan_publish_en": True}],
            ),
            Node(
                package="xq_autonomy", executable="xq_p6_directional_integrity",
                name="xq_p6_directional_integrity", parameters=[str(autonomy / "config" / "p6_integrity.yaml")],
            ),
            Node(
                package="xq_autonomy", executable="xq_p10_information_map",
                name="xq_p10_information_map", parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="xq_autonomy", executable="xq_p11_integrity_exploration",
                name="xq_p11_integrity_exploration",
                remappings=[("/xq/p5/cloud_map", "/mapping/p12/static_voxels")],
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
                package="xq_autonomy", executable="xq_p12_dynamic_map",
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
                }],
            ),
            Node(
                package="xq_autonomy", executable="xq_p12_obstacle_driver",
                name="xq_p12_obstacle_driver", parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="xq_autonomy", executable="xq_p12_flight_controller",
                name="xq_p12_flight_controller",
                parameters=[{
                    "use_sim_time": True,
                    "variant": "integrity_constrained",
                    "direct_information_gain": 1.0,
                    "safe_information_gain": 0.75,
                    "mission_distance_m": f("mission_distance_m"),
                    "path_clearance_radius_m": f("path_clearance_radius_m"),
                    "planning_lookahead_m": f("planning_lookahead_m"),
                    "clear_confirmation_s": f("clear_confirmation_s"),
                }],
            ),
            Node(
                package="xq_autonomy", executable="xq_p12_flight_evaluator",
                name="xq_p12_flight_evaluator",
                parameters=[{
                    "use_sim_time": True,
                    "result_file": LaunchConfiguration("result_file"),
                    "thresholds_file": LaunchConfiguration("thresholds_file"),
                }],
            ),
        ]
    )
