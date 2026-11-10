"""Opt-in bridge from a rigid ZLIO sensor frame to the navigation base."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'use_sim_time': 'false',
        'sensor_frame': 'lio_base_sensor',
        'source_child_frame': 'imu',
        'odom_frame': 'odom',
        'base_frame': 'base_footprint',
        'input_topic': '/odometry',
        'output_topic': '/odom',
        'frontend_frame_topic': '/frontend/frame',
        'frontend_session_topic': '/frontend/session',
        'deskewed_cloud_topic': '/cloud_deskewed',
    }
    params = {name: LaunchConfiguration(name) for name in defaults if name != 'use_sim_time'}
    params['use_sim_time'] = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        DeclareLaunchArgument('publish_deskewed_cloud', default_value='false', choices=['true', 'false']),
        Node(package='xxu_bringup', executable='zlio_base_odometry.py',
             name='zlio_base_odometry', parameters=[params, {
                 'publish_deskewed_cloud': ParameterValue(LaunchConfiguration('publish_deskewed_cloud'), value_type=bool),
             }], output='screen'),
    ])
