"""ROS-side FAST-LIO -> MAVROS ExternalNav stack for IMPACT Gate P4."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bridge = Path(get_package_share_directory("xq_gz_bridge"))
    fast_lio = Path(get_package_share_directory("xq_fast_lio"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bridge_config", default_value=str(bridge / "config" / "p4_external_nav.yaml")
            ),
            DeclareLaunchArgument(
                "fast_lio_config", default_value=str(fast_lio / "config" / "xq_p4.yaml")
            ),
            DeclareLaunchArgument("evaluation_result_file"),
            DeclareLaunchArgument("minimum_duration_s", default_value="70.0"),
            LogInfo(msg=["P4 evaluation result: ", LaunchConfiguration("evaluation_result_file")]),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_p4_base_to_livox",
                arguments=[
                    "--x", "0.04", "--y", "0.0", "--z", "0.12",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                    "--frame-id", "xq_base_link", "--child-frame-id", "livox_frame",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xq_p4_base_to_livox_imu",
                arguments=[
                    "--x", "0.0", "--y", "0.0", "--z", "0.0",
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
                parameters=[LaunchConfiguration("fast_lio_config"), {"use_sim_time": True}],
                output="screen",
            ),
            # Deliberately wall-timed at the MAVROS boundary.  Source stamps are
            # still used for velocity differentiation inside the adapter.
            Node(
                package="xq_autonomy",
                executable="xq_p4_external_nav",
                name="xq_p4_external_nav",
                parameters=[{"use_sim_time": False}],
                output="screen",
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p3_evaluator",
                name="xq_p4_evaluator",
                parameters=[
                    {
                        "use_sim_time": True,
                        "scenario": "structured_room",
                        "gate_name": "P4_FAST_LIO_IN_FLIGHT_QUALITY",
                        "ground_truth_consumer": "xq_p4_evaluator_only",
                        "ground_truth_topic": "/xq/eval/p4/ground_truth",
                        "result_file": LaunchConfiguration("evaluation_result_file"),
                        "minimum_duration_s": ParameterValue(
                            LaunchConfiguration("minimum_duration_s"), value_type=float
                        ),
                    }
                ],
                output="screen",
            ),
        ]
    )
