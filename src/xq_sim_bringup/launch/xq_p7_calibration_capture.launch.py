"""P3 localization benchmark plus P6 predictor and P7 train/test collector."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory("xq_sim_bringup"))
    autonomy = Path(get_package_share_directory("xq_autonomy"))
    arguments = [
        DeclareLaunchArgument("world_file"),
        DeclareLaunchArgument("scenario"),
        DeclareLaunchArgument("split"),
        DeclareLaunchArgument("trajectory_variant"),
        DeclareLaunchArgument("p3_result_file"),
        DeclareLaunchArgument("p7_result_file"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("minimum_duration_s", default_value="65.0"),
    ]
    return LaunchDescription(
        arguments
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup / "launch" / "xq_p3_fast_lio.launch.py")),
                launch_arguments={
                    "world_file": LaunchConfiguration("world_file"),
                    "scenario": LaunchConfiguration("scenario"),
                    "result_file": LaunchConfiguration("p3_result_file"),
                    "minimum_duration_s": LaunchConfiguration("minimum_duration_s"),
                    "integrity_geometry_enable": "true",
                    "trajectory_variant": LaunchConfiguration("trajectory_variant"),
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
                executable="xq_p7_calibration_collector",
                name="xq_p7_calibration_collector",
                parameters=[
                    {
                        "use_sim_time": True,
                        "scenario": LaunchConfiguration("scenario"),
                        "split": LaunchConfiguration("split"),
                        "trajectory_variant": LaunchConfiguration("trajectory_variant"),
                        "result_file": LaunchConfiguration("p7_result_file"),
                        "calibration_file": LaunchConfiguration("calibration_file"),
                        "minimum_duration_s": ParameterValue(
                            LaunchConfiguration("minimum_duration_s"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )

