"""P10 online ROS contract Gate before the formal three-arm flight benchmark."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("result_file"),
        DeclareLaunchArgument("calibration_file"),
        DeclareLaunchArgument("calibration_sha256", default_value=""),
        Node(package="xq_autonomy", executable="xq_p10_gate_scenario"),
        Node(
            package="xq_autonomy",
            executable="xq_p10_active_perception",
            parameters=[{
                "calibration_file": LaunchConfiguration("calibration_file"),
                "calibration_sha256": LaunchConfiguration("calibration_sha256"),
                "visibility_radius_m": 1.70,
                "information_scale": 2500.0,
                "margin_reserve_m": 0.10,
            }],
        ),
        Node(
            package="xq_autonomy",
            executable="xq_p10_gate_evaluator",
            parameters=[{"result_file": LaunchConfiguration("result_file")}],
        ),
    ])
