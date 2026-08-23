"""Deterministic P9 hard-certification Gate under identical covariance."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("result_file"),
            DeclareLaunchArgument("calibration_file"),
            DeclareLaunchArgument("calibration_sha256", default_value=""),
            Node(package="xq_autonomy", executable="xq_p9_gate_scenario"),
            Node(
                package="xq_autonomy",
                executable="xq_p8_alert_limit",
                remappings=[("/planning/bspline", "/planning/candidate_bspline")],
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p9_integrity_margin",
                parameters=[{
                    "calibration_file": LaunchConfiguration("calibration_file"),
                    "calibration_sha256": LaunchConfiguration("calibration_sha256"),
                    "margin_reserve_m": 0.10,
                }],
            ),
            Node(package="xq_autonomy", executable="xq_p9_trajectory_gate"),
            Node(
                package="xq_autonomy",
                executable="xq_p9_gate_evaluator",
                parameters=[{"result_file": LaunchConfiguration("result_file")}],
            ),
        ]
    )
