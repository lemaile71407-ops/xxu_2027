"""Opt-in bridge from a rigid ZLIO sensor frame to the navigation base."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from xxu_bringup.interfaces import interfaces


def generate_launch_description():
    contract = interfaces()
    defaults = {
        'use_sim_time': 'false',
        'sensor_frame': 'lio_base_sensor',
        'source_child_frame': 'imu',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'input_topic': contract.topic('localization.frontend_odometry'),
        'output_topic': contract.topic('localization.odometry'),
        'tf_output_topic': '/tf',
        'tf_publish_rate': '30.0',
    }
    params = {name: LaunchConfiguration(name) for name in defaults
              if name not in ('use_sim_time', 'tf_output_topic', 'tf_publish_rate')}
    params['use_sim_time'] = False
    params['tf_publish_rate'] = ParameterValue(LaunchConfiguration('tf_publish_rate'), value_type=float)
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        Node(package='xxu_bringup', executable='zlio_base_odometry',
             name='zlio_base_odometry', parameters=[params], output='screen',
             remappings=[('/tf', LaunchConfiguration('tf_output_topic'))]),
    ])
