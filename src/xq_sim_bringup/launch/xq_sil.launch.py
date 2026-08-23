"""Launch the project-isolated Xuanqiong-X1 Gazebo SIL stack."""

from __future__ import annotations

import os
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _unique_defaults() -> tuple[str, str]:
    """Return collision-resistant defaults for direct (non-scripted) launches."""
    nonce = time.time_ns() ^ os.getpid()
    # Stay inside the Linux-safe DDS domain range.  run_smoke.sh additionally
    # scans /proc and selects an unused value before invoking this launch file.
    ros_domain_id = str(32 + nonce % 70)
    gz_partition = f"xq_sil_{os.getpid()}_{nonce:x}"
    return ros_domain_id, gz_partition


def generate_launch_description() -> LaunchDescription:
    assets_share = Path(get_package_share_directory("xq_gz_assets"))
    bridge_share = Path(get_package_share_directory("xq_gz_bridge"))
    autonomy_share = Path(get_package_share_directory("xq_autonomy"))

    default_world = assets_share / "worlds" / "xq_indoor_office.sdf"
    default_bridge_config = bridge_share / "config" / "bridge.yaml"
    default_stack_config = autonomy_share / "config" / "stack.yaml"
    default_fault_schedule = autonomy_share / "config" / "fault_schedule.json"
    default_run_dir = Path.home() / "xuanqiong_x1_sim_ws" / "runs" / "latest"
    isolated_resource_path = os.pathsep.join(
        (str(assets_share / "models"), str(assets_share / "worlds"))
    )

    generated_domain, generated_partition = _unique_defaults()
    inherited_domain = os.environ.get("ROS_DOMAIN_ID", "")
    try:
        inherited_domain_value = int(inherited_domain)
    except ValueError:
        inherited_domain_value = -1
    default_domain = (
        inherited_domain
        if 32 <= inherited_domain_value <= 101
        else generated_domain
    )
    default_partition = os.environ.get("GZ_PARTITION", generated_partition)

    world_file = LaunchConfiguration("world_file")
    bridge_config = LaunchConfiguration("bridge_config")
    stack_config = LaunchConfiguration("stack_config")
    fault_schedule = LaunchConfiguration("fault_schedule")
    run_dir = LaunchConfiguration("run_dir")
    seed = LaunchConfiguration("seed")
    headless = LaunchConfiguration("headless")
    start_gazebo = LaunchConfiguration("start_gazebo")

    gz_headless = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            "-s",
            "--headless-rendering",
            "-v",
            LaunchConfiguration("gz_verbosity"),
            world_file,
        ],
        name="xq_gazebo_headless",
        output="screen",
        condition=IfCondition(headless),
    )
    gz_gui = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            "-v",
            LaunchConfiguration("gz_verbosity"),
            world_file,
        ],
        name="xq_gazebo_gui",
        output="screen",
        condition=UnlessCondition(headless),
    )

    shutdown_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=process,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="XQ Gazebo process exited")
                    )
                ],
            ),
            condition=IfCondition(LaunchConfiguration("shutdown_on_gazebo_exit")),
        )
        for process in (gz_headless, gz_gui)
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument(
                "bridge_config", default_value=str(default_bridge_config)
            ),
            DeclareLaunchArgument("stack_config", default_value=str(default_stack_config)),
            DeclareLaunchArgument(
                "fault_schedule", default_value=str(default_fault_schedule)
            ),
            DeclareLaunchArgument("run_dir", default_value=str(default_run_dir)),
            DeclareLaunchArgument("scenario", default_value="xq_indoor_office"),
            DeclareLaunchArgument("seed", default_value="20260820"),
            DeclareLaunchArgument("spec_sha256", default_value="UNSET"),
            DeclareLaunchArgument("world_sha256", default_value="UNSET"),
            DeclareLaunchArgument(
                "configuration_manifest_path", default_value=""
            ),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("inject_faults", default_value="false"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_network_relay", default_value="true"),
            DeclareLaunchArgument("start_stack", default_value="true"),
            DeclareLaunchArgument("start_metrics", default_value="true"),
            DeclareLaunchArgument("shutdown_on_gazebo_exit", default_value="true"),
            DeclareLaunchArgument("gz_verbosity", default_value="2"),
            DeclareLaunchArgument("ros_domain_id", default_value=default_domain),
            DeclareLaunchArgument("gz_partition", default_value=default_partition),
            # Never inherit model paths from another Gazebo project.  The world is
            # self-contained and may resolve models only from xq_gz_assets.
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", value=isolated_resource_path
            ),
            SetEnvironmentVariable("SDF_PATH", value=str(assets_share / "models")),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", value=LaunchConfiguration("ros_domain_id")
            ),
            SetEnvironmentVariable(
                "GZ_PARTITION", value=LaunchConfiguration("gz_partition")
            ),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", value="1"),
            LogInfo(
                msg=[
                    "XQ isolated SIL: ROS_DOMAIN_ID=",
                    LaunchConfiguration("ros_domain_id"),
                    ", GZ_PARTITION=",
                    LaunchConfiguration("gz_partition"),
                ]
            ),
            GroupAction(
                actions=[gz_headless, gz_gui, *shutdown_handlers],
                condition=IfCondition(start_gazebo),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_base_to_mid360_tf",
                arguments=[
                    "--x", "0.04", "--y", "0.0", "--z", "0.135",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "xq_base_link",
                    "--child-frame-id", "xq_mid360_link",
                ],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("start_bridge")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_base_to_imu_tf",
                arguments=[
                    "--x", "0.0", "--y", "0.0", "--z", "0.015",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "xq_base_link",
                    "--child-frame-id", "xq_imu_link",
                ],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("start_bridge")),
            ),
            Node(
                package="xq_gz_bridge",
                executable="xq_gz_bridge_node",
                name="xq_gz_bridge_node",
                output="screen",
                parameters=[bridge_config, {"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("start_bridge")),
            ),
            Node(
                package="xq_autonomy",
                executable="xq_network_relay",
                name="xq_network_relay",
                output="screen",
                parameters=[{"use_sim_time": True, "seed": seed}],
                condition=IfCondition(LaunchConfiguration("start_network_relay")),
            ),
            Node(
                package="xq_autonomy",
                executable="xq_stack_node",
                name="xq_stack_node",
                output="screen",
                parameters=[stack_config, {"use_sim_time": True, "seed": seed}],
                condition=IfCondition(LaunchConfiguration("start_stack")),
            ),
            Node(
                package="xq_autonomy",
                executable="xq_metrics_node",
                name="xq_metrics_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "run_dir": run_dir,
                        "scenario": LaunchConfiguration("scenario"),
                        "seed": seed,
                        "spec_sha256": LaunchConfiguration("spec_sha256"),
                        "world_sha256": LaunchConfiguration("world_sha256"),
                        "configuration_manifest_path": LaunchConfiguration(
                            "configuration_manifest_path"
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("start_metrics")),
            ),
            Node(
                package="xq_autonomy",
                executable="xq_fault_injector",
                name="xq_fault_injector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "schedule_file": fault_schedule,
                        "seed": seed,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("inject_faults")),
            ),
        ]
    )
