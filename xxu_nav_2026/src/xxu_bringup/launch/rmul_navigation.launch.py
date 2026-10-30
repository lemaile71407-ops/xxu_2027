"""Supplied RMUL field and fixed folded EGA vehicle using existing navigation."""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from xxu_bringup.robot_profiles import RobotGeometryFile


def generate_launch_description():
    share = FindPackageShare('xxu_bringup')
    defaults = {
        'field_model': PathJoinSubstitution([share, 'models', 'rmul_2027', 'model.sdf']),
        'map': PathJoinSubstitution([share, 'models', 'rmul_2027', 'navigation', 'rmul_2027.yaml']),
        'spawn_x': '2.0', 'spawn_y': '2.0', 'spawn_yaw': '0.0',
        'gazebo_gui': 'true', 'rviz': 'true',
        'start_navigation': 'true', 'enable_localization': 'true',
        'enable_lio': 'true', 'lidar_mode': 'rgl', 'rgl_plugin_dir': '',
        'robot_model': 'ega_folded',
        'robot_geometry_file': RobotGeometryFile(LaunchConfiguration('robot_model'), share),
    }
    return LaunchDescription([
        DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()
    ] + [
        LogInfo(msg='RMUL user field and folded EGA vehicle; CAD mass with corrected lidar inertias, uncalibrated contact/actuator model'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            share, 'launch', 'rm_sim_navigation.launch.py'])),
            launch_arguments={key: LaunchConfiguration(key) for key in defaults}.items()),
    ])
