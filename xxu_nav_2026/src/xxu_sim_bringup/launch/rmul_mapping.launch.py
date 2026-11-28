"""Run RMUL simulation, Z-Localization mapping, and mapless Nav2."""

import math
from pathlib import Path
from tempfile import TemporaryDirectory

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml

from xxu_bringup.backend_config import load_backend_params, merge_mapping
from xxu_bringup.interfaces import interfaces
from xxu_bringup.robot_profiles import RobotGeometryFile


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    if arg('robot_model') != 'ega_folded':
        raise ValueError('RMUL mapping requires robot_model:=ega_folded')

    auto_waypoints = arg('auto_waypoints') == 'true'
    required_files = ['frontend_config', 'backend_config', 'nav2_params']
    if auto_waypoints:
        required_files.append('waypoints_file')
    for name in required_files:
        if not Path(arg(name)).is_file():
            raise ValueError(f'{name} must be an existing file')

    try:
        map_voxel_resolution = float(arg('map_voxel_resolution'))
    except ValueError as error:
        raise ValueError('map_voxel_resolution must be a finite positive number') from error
    if not math.isfinite(map_voxel_resolution) or map_voxel_resolution <= 0.0:
        raise ValueError('map_voxel_resolution must be a finite positive number')

    backend = merge_mapping(load_backend_params(arg('backend_config')), {
        'prior_map': {'enabled': False},
        'uwb': {'enabled': False},
        'startup_hint': {'enabled': False},
        'loop': {'enabled': True},
        'trajectory': {
            'save_directory': arg('save_directory'),
            'save_global_pcd': True,
            'map_voxel_resolution': map_voxel_resolution,
        },
        'visualization': {'enabled': True, 'publish_runtime_map': True, 'publish_paths': True},
        'output': {'tf': {'publish_predictions': True}},
    })
    temporary = TemporaryDirectory(prefix='xxu_rmul_mapping_')
    config = Path(temporary.name) / 'backend.yaml'
    config.write_text(yaml.safe_dump(backend, sort_keys=False), encoding='utf-8')

    def include(package, launch_file, arguments):
        path = Path(get_package_share_directory(package)) / 'launch' / launch_file
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=arguments.items())

    contract = interfaces()
    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temporary.cleanup()),
        ])),
        include('xxu_sim_bringup', 'rm_sim_gazebo.launch.py', {
            **{name: arg(name) for name in (
                'field_model', 'spawn_x', 'spawn_y', 'spawn_yaw', 'gazebo_gui', 'lidar_mode',
                'rgl_plugin_dir', 'robot_model', 'imu_sampling', 'clock_publish_rate',
                'scan_min_height', 'scan_max_height',
            )},
            'require_localization': 'false',
            'rgl_point_timing': 'uniform',
        }),
        include('bringup', 'bringup.launch.py', {
            'mode': 'mapping',
            'prior_map': '',
            'frontend_config': arg('frontend_config'),
            'backend_config': str(config),
            'use_sim_time': 'true',
            'input_type': 'pointcloud2',
            'lidar_topic': contract.topic('simulation.lidar_compensated'),
            'imu_topic': contract.topic('simulation.imu_compensated'),
            'imu_acceleration_scale': '1.0',
            'point_time_field': 'timestamp',
            'point_time_scale': '1e-9',
            'point_time_absolute': 'true',
            'publish_tf': 'false',
            'rviz': 'false',
            'deskewed_cloud_frame': contract.frame('lidar_compensated'),
            'backend_tf_topic': '/sentry/tf/localization',
        }),
        include('xxu_bringup', 'zlio_base_adapter.launch.py', {
            'use_sim_time': 'true',
            'tf_output_topic': '/sentry/tf/odometry',
        }),
        include('xxu_bringup', 'navigation.launch.py', {
            'use_map_server': 'false',
            'enable_localization': 'false',
            'wait_for_tf_only': 'true',
            'localization_mode': 'external',
            'use_sim_time': 'true',
            'rviz': arg('rviz'),
            'params_file': arg('nav2_params'),
            'robot_geometry_file': arg('robot_geometry_file'),
            'planner_tolerance': arg('planner_tolerance'),
        }),
    ]
    if auto_waypoints:
        actions.append(Node(
            package='xxu_bringup',
            executable='waypoint_mapping_runner.py',
            name='waypoint_mapping_runner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'waypoints_file': ParameterValue(arg('waypoints_file'), value_type=str),
                'auto_save_map': arg('auto_save_map') == 'true',
            }],
        ))
    return actions


def generate_launch_description():
    sim_share = FindPackageShare('xxu_sim_bringup')
    bringup_share = FindPackageShare('xxu_bringup')
    defaults = {
        'field_model': PathJoinSubstitution([sim_share, 'models', 'rmul_2027', 'model.sdf']),
        'spawn_x': '2.0',
        'spawn_y': '2.0',
        'spawn_yaw': '0.0',
        'gazebo_gui': 'true',
        'rviz': 'true',
        'lidar_mode': 'rgl',
        'rgl_plugin_dir': '',
        'clock_publish_rate': '200.0',
        'robot_model': 'ega_folded',
        'robot_geometry_file': RobotGeometryFile(LaunchConfiguration('robot_model'), sim_share),
        'scan_min_height': '0.10',
        'scan_max_height': '',
        'frontend_config': '',
        'backend_config': '',
        'save_directory': '/tmp/xxu_rmul_mapping_output',
        'map_voxel_resolution': '0.05',
        'nav2_params': PathJoinSubstitution([bringup_share, 'config', 'nav2_mapping.yaml']),
        'waypoints_file': PathJoinSubstitution([bringup_share, 'waypoints', 'rmul_2027_mapping_loop.csv']),
        'planner_tolerance': '0.35',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        DeclareLaunchArgument('imu_sampling', default_value='interval_mean',
                              choices=['direct', 'interval_mean']),
        DeclareLaunchArgument('auto_waypoints', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('auto_save_map', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
