"""Publish one deterministic fault, matching the P14 command-line contract."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("fault"),
        DeclareLaunchArgument("start_time", default_value="30.0"),
        DeclareLaunchArgument("duration", default_value="1.0"),
        DeclareLaunchArgument("severity", default_value="1.0"),
        DeclareLaunchArgument("seed", default_value="20260828"),
        Node(
            package="impact_fault_injection",
            executable="impact_fault_injector",
            name="impact_single_fault_injector",
            parameters=[{
                "use_sim_time": True,
                "fault": LaunchConfiguration("fault"),
                "start_time_s": ParameterValue(LaunchConfiguration("start_time"), value_type=float),
                "duration_s": ParameterValue(LaunchConfiguration("duration"), value_type=float),
                "severity": ParameterValue(LaunchConfiguration("severity"), value_type=float),
                "seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
            }],
        ),
    ])
