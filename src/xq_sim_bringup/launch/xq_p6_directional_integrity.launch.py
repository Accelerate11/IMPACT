"""P5 baseline plus the non-controlling P6 Directional Integrity Predictor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory("xq_sim_bringup"))
    autonomy = Path(get_package_share_directory("xq_autonomy"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("evaluation_result_file"),
            DeclareLaunchArgument("integrity_result_file"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup / "launch" / "xq_p5_baseline.launch.py")),
                launch_arguments={
                    "evaluation_result_file": LaunchConfiguration("evaluation_result_file"),
                    "integrity_geometry_enable": "true",
                }.items(),
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p6_directional_integrity",
                name="xq_p6_directional_integrity",
                parameters=[str(autonomy / "config" / "p6_integrity.yaml")],
            ),
            Node(
                package="xq_autonomy",
                executable="xq_p6_integrity_evaluator",
                name="xq_p6_integrity_evaluator",
                parameters=[
                    {
                        "use_sim_time": True,
                        "result_file": LaunchConfiguration("integrity_result_file"),
                        "minimum_samples": 80,
                    }
                ],
            ),
        ]
    )

