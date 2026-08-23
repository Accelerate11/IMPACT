"""Isolated IMPACT P3 FAST-LIO2 baseline and evaluation launch."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    assets = Path(get_package_share_directory("xq_gz_assets"))
    bridge = Path(get_package_share_directory("xq_gz_bridge"))
    fast_lio = Path(get_package_share_directory("xq_fast_lio"))
    resource_path = os.pathsep.join((str(assets / "models"), str(assets / "worlds")))

    gazebo = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", "-s", "--headless-rendering", "-v", "2",
            LaunchConfiguration("world_file"),
        ],
        name="xq_p3_gazebo",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file"),
            DeclareLaunchArgument(
                "bridge_config", default_value=str(bridge / "config" / "p3_fast_lio.yaml")
            ),
            DeclareLaunchArgument(
                "fast_lio_config", default_value=str(fast_lio / "config" / "xq_p3.yaml")
            ),
            DeclareLaunchArgument("scenario", default_value="structured_room"),
            DeclareLaunchArgument("result_file"),
            DeclareLaunchArgument("minimum_duration_s", default_value="65.0"),
            DeclareLaunchArgument("integrity_geometry_enable", default_value="false"),
            DeclareLaunchArgument("scan_publish_enable", default_value="false"),
            DeclareLaunchArgument("trajectory_variant", default_value="baseline"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("SDF_PATH", str(assets / "models")),
            LogInfo(msg=["P3 evaluation result: ", LaunchConfiguration("result_file")]),
            gazebo,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gazebo,
                    on_exit=[EmitEvent(event=Shutdown(reason="P3 Gazebo exited"))],
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_p3_base_to_livox",
                arguments=[
                    "--x", "0.04", "--y", "0.0", "--z", "0.135",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "xq_base_link", "--child-frame-id", "livox_frame",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_p3_base_to_livox_imu",
                arguments=[
                    "--x", "0.0", "--y", "0.0", "--z", "0.015",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "xq_base_link", "--child-frame-id", "livox_imu",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="xq_gz_bridge",
                executable="xq_gz_bridge_node",
                name="xq_gz_bridge_node",
                parameters=[LaunchConfiguration("bridge_config")],
                output="screen",
            ),
            Node(
                package="xq_fast_lio",
                executable="fastlio_mapping",
                name="xq_fast_lio",
                parameters=[
                    LaunchConfiguration("fast_lio_config"),
                    {
                        "use_sim_time": True,
                        "integrity_geometry.enable": LaunchConfiguration("integrity_geometry_enable"),
                        "publish.scan_publish_en": ParameterValue(
                            LaunchConfiguration("scan_publish_enable"), value_type=bool
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p3_trajectory",
                name="xq_p3_trajectory",
                parameters=[
                    {
                        "use_sim_time": True,
                        "scenario": LaunchConfiguration("scenario"),
                        "trajectory_variant": LaunchConfiguration("trajectory_variant"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p3_evaluator",
                name="xq_p3_evaluator",
                parameters=[
                    {
                        "use_sim_time": True,
                        "scenario": LaunchConfiguration("scenario"),
                        "result_file": LaunchConfiguration("result_file"),
                        "minimum_duration_s": ParameterValue(
                            LaunchConfiguration("minimum_duration_s"), value_type=float
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
