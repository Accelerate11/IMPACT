"""Live P5 autonomy stack plus non-controlling P8 static Alert Limit."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory("xq_sim_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("evaluation_result_file"),
            DeclareLaunchArgument("alert_limit_result_file"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bringup / "launch" / "xq_p5_baseline.launch.py")
                ),
                launch_arguments={
                    "evaluation_result_file": LaunchConfiguration("evaluation_result_file"),
                }.items(),
            ),
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
                        "result_file": LaunchConfiguration("alert_limit_result_file"),
                    }
                ],
            ),
        ]
    )
