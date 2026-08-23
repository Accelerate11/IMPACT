"""P10 long-corridor flight arm using Gazebo, FAST-LIO and online surfels."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
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
    return [ExecuteProcess(cmd=command, name="xq_p10_gazebo", output="screen")]


def generate_launch_description() -> LaunchDescription:
    assets = Path(get_package_share_directory("xq_gz_assets"))
    bridge = Path(get_package_share_directory("xq_gz_bridge"))
    fast_lio = Path(get_package_share_directory("xq_fast_lio"))
    autonomy = Path(get_package_share_directory("xq_autonomy"))
    resource_path = os.pathsep.join((str(assets / "models"), str(assets / "worlds")))
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file"),
            DeclareLaunchArgument("variant"),
            DeclareLaunchArgument("calibration_file"),
            DeclareLaunchArgument("calibration_sha256", default_value=""),
            DeclareLaunchArgument("result_file"),
            DeclareLaunchArgument("gz_record_path", default_value=""),
            DeclareLaunchArgument("visibility_radius_m", default_value="2.8"),
            DeclareLaunchArgument("information_scale", default_value="2500.0"),
            DeclareLaunchArgument("minimum_prediction_variance_m2", default_value="0.00001"),
            DeclareLaunchArgument("margin_reserve_m", default_value="0.10"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("SDF_PATH", str(assets / "models")),
            OpaqueFunction(function=_gazebo),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_p10_base_to_livox",
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
                name="xq_p10_base_to_livox_imu",
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
                executable="xq_p10_active_perception",
                name="xq_p10_active_perception",
                parameters=[
                    {
                        "use_sim_time": True,
                        "calibration_file": LaunchConfiguration("calibration_file"),
                        "calibration_sha256": LaunchConfiguration("calibration_sha256"),
                        "visibility_radius_m": ParameterValue(
                            LaunchConfiguration("visibility_radius_m"), value_type=float
                        ),
                        "information_scale": ParameterValue(
                            LaunchConfiguration("information_scale"), value_type=float
                        ),
                        "minimum_prediction_variance_m2": ParameterValue(
                            LaunchConfiguration("minimum_prediction_variance_m2"), value_type=float
                        ),
                        "margin_reserve_m": ParameterValue(
                            LaunchConfiguration("margin_reserve_m"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p10_flight_controller",
                name="xq_p10_flight_controller",
                parameters=[{"use_sim_time": True, "variant": LaunchConfiguration("variant")}],
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p10_flight_evaluator",
                name="xq_p10_flight_evaluator",
                parameters=[
                    {
                        "use_sim_time": True,
                        "variant": LaunchConfiguration("variant"),
                        "result_file": LaunchConfiguration("result_file"),
                        "calibration_file": LaunchConfiguration("calibration_file"),
                        "margin_reserve_m": ParameterValue(
                            LaunchConfiguration("margin_reserve_m"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
