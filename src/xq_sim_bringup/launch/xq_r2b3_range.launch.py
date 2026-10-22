"""R2B3 rendered-range acquisition alongside the unchanged R2 shadow sensors.

The reference range is an evaluator-only channel. This launch adds no range or
flow measurement to the estimator and sends neither to ArduPilot.
"""
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration('config_file')
    result = LaunchConfiguration('result_file')
    contract = PythonLaunchDescriptionSource(
        get_package_share_directory('xq_sim_bringup') + '/launch/xq_r2_contract.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('config_file'),
        DeclareLaunchArgument('result_file'),
        IncludeLaunchDescription(
            contract,
            launch_arguments={
                'config_file': config,
                'result_file': result,
                'calibration_context': 'r2b3_range',
            }.items()),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='impact_r2b3_tf_down_range',
            arguments=[
                '--x', '0', '--y', '0', '--z', '-0.03',
                '--roll', '0', '--pitch', str(math.pi / 2), '--yaw', '0',
                '--frame-id', 'xq_base_link', '--child-frame-id', 'impact_down_range',
            ],
            parameters=[{'use_sim_time': True}], output='screen'),
        Node(
            package='xq_gz_bridge', executable='impact_r2_range_bridge',
            name='impact_r2_range_bridge',
            parameters=[{'use_sim_time': True}], output='screen'),
    ])
