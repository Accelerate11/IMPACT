"""Replay/live P8 static-obstacle Alert Limit Gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("result_file"),
            Node(
                package="xq_autonomy",
                executable="xq_p8_alert_limit",
                name="xq_p8_alert_limit",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p8_alert_limit_evaluator",
                name="xq_p8_alert_limit_evaluator",
                parameters=[
                    {
                        "use_sim_time": True,
                        "result_file": LaunchConfiguration("result_file"),
                    }
                ],
            ),
        ]
    )
