"""Launch the isolated IMPACT P2 Mid-360-like sensor contract."""

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
    world = assets / "worlds" / "xq_indoor_office.sdf"
    bridge_config = bridge / "config" / "p2_mid360.yaml"
    resource_path = os.pathsep.join((str(assets / "models"), str(assets / "worlds")))

    gazebo = ExecuteProcess(
        cmd=[
            "gz", "sim", "-r", "-s", "--headless-rendering", "-v", "2",
            LaunchConfiguration("world_file"),
        ],
        name="xq_p2_gazebo",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(world)),
            DeclareLaunchArgument("bridge_config", default_value=str(bridge_config)),
            DeclareLaunchArgument("result_file"),
            DeclareLaunchArgument("minimum_duration_s", default_value="600.0"),
            DeclareLaunchArgument("lidar_x", default_value="0.04"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.135"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("imu_x", default_value="0.0"),
            DeclareLaunchArgument("imu_y", default_value="0.0"),
            DeclareLaunchArgument("imu_z", default_value="0.015"),
            DeclareLaunchArgument("imu_roll", default_value="0.0"),
            DeclareLaunchArgument("imu_pitch", default_value="0.0"),
            DeclareLaunchArgument("imu_yaw", default_value="0.0"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("SDF_PATH", str(assets / "models")),
            LogInfo(msg=["P2 sensor validation result: ", LaunchConfiguration("result_file")]),
            gazebo,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gazebo,
                    on_exit=[EmitEvent(event=Shutdown(reason="P2 Gazebo exited"))],
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_p2_base_to_livox",
                arguments=[
                    "--x", LaunchConfiguration("lidar_x"),
                    "--y", LaunchConfiguration("lidar_y"),
                    "--z", LaunchConfiguration("lidar_z"),
                    "--roll", LaunchConfiguration("lidar_roll"),
                    "--pitch", LaunchConfiguration("lidar_pitch"),
                    "--yaw", LaunchConfiguration("lidar_yaw"),
                    "--frame-id", "xq_base_link",
                    "--child-frame-id", "livox_frame",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_p2_base_to_livox_imu",
                arguments=[
                    "--x", LaunchConfiguration("imu_x"),
                    "--y", LaunchConfiguration("imu_y"),
                    "--z", LaunchConfiguration("imu_z"),
                    "--roll", LaunchConfiguration("imu_roll"),
                    "--pitch", LaunchConfiguration("imu_pitch"),
                    "--yaw", LaunchConfiguration("imu_yaw"),
                    "--frame-id", "xq_base_link",
                    "--child-frame-id", "livox_imu",
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
                package="xq_autonomy",
                executable="xq_p2_sensor_validator",
                name="xq_p2_sensor_validator",
                parameters=[
                    {
                        "use_sim_time": False,
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
