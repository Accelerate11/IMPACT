"""R2 shadow sensors: rendered images -> gyro-compensated angular flow only."""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config, result = LaunchConfiguration('config_file'), LaunchConfiguration('result_file')
    sim = {'use_sim_time': True}
    transforms = []
    for name, xyz, rpy in (
        ('impact_front_optical', (0.12, 0., 0.03), (-math.pi/2, 0., -math.pi/2)),
        ('impact_down_optical', (0., 0., -0.03), (math.pi, 0., -math.pi/2)),
    ):
        transforms.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='impact_r2_tf_' + name,
            arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                       '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                       '--frame-id', 'xq_base_link', '--child-frame-id', name],
            parameters=[sim], output='screen'))
    return LaunchDescription([
        DeclareLaunchArgument('config_file'), DeclareLaunchArgument('result_file'),
        DeclareLaunchArgument('calibration_context', default_value='r2_base'),
        *transforms,
        Node(package='xq_gz_bridge', executable='impact_r2_camera_bridge',
             name='impact_r2_camera_bridge',
             parameters=[{**sim, 'calibration_context': LaunchConfiguration('calibration_context')}],
             output='screen'),
        Node(package='xq_autonomy', executable='impact_downward_flow',
             name='impact_downward_flow', parameters=[sim], output='screen'),
        Node(package='xq_autonomy', executable='impact_multisensor_monitor',
             name='impact_multisensor_monitor',
             parameters=[{**sim, 'config_file': config, 'result_file': result}], output='screen'),
    ])
