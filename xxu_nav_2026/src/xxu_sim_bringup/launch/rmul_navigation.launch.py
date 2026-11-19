"""Supplied RMUL field and fixed folded EGA vehicle using existing navigation."""


from launch import LaunchDescription
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from xxu_bringup.robot_profiles import RobotGeometryFile
from xxu_bringup.interfaces import interfaces


def external_setup(context):
    """Compose existing simulation/localization/navigation entries, explicitly."""
    lc = lambda name: LaunchConfiguration(name).perform(context)
    if lc('enable_localization') != 'true':
        raise ValueError('zlio_tdt mode requires enable_localization:=true')
    for name in ('frontend_config', 'backend_config', 'external_map', 'external_nav2_params'):
        if not lc(name) or not Path(lc(name)).is_file():
            raise ValueError(f'zlio_tdt mode requires an existing {name}')
    if not lc('prior_map') or not Path(lc('prior_map')).is_dir():
        raise ValueError('zlio_tdt mode requires an existing prior_map directory')
    if lc('robot_model') != 'ega_folded':
        raise ValueError('zlio_tdt mode currently supports the fixed-sensor ega_folded profile')
    bringup_share = get_package_share_directory('xxu_bringup')
    sim_share = get_package_share_directory('xxu_sim_bringup')
    contract = interfaces()

    def include(path, args, condition=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                       launch_arguments=args.items(), condition=condition)

    return [
        LogInfo(msg='ZLIO/TDT: fixed sensor, no AMCL/GICP or spin; Nav2 waits for trusted localization'),
        include(Path(sim_share) / 'launch/rm_sim_gazebo.launch.py', {
            **{key: lc(key) for key in ('field_model', 'spawn_x', 'spawn_y', 'spawn_yaw',
                                       'gazebo_gui', 'lidar_mode', 'rgl_plugin_dir', 'robot_model', 'imu_sampling',
                                       'dynamic_test_vehicle', 'clock_publish_rate')},
            'rgl_point_timing': 'uniform', 'require_localization': 'true',
        }),
        include(Path(get_package_share_directory('bringup')) / 'launch/bringup.launch.py', {
            **{key: lc(key) for key in ('frontend_config', 'backend_config', 'prior_map')},
            'mode': 'localization', 'use_sim_time': 'true', 'input_type': 'pointcloud2',
            'lidar_topic': contract.topic('simulation.lidar_compensated'),
            'imu_topic': contract.topic('simulation.imu_compensated'),
            'imu_acceleration_scale': '1.0', 'point_time_field': 'timestamp',
            'point_time_scale': '1e-9', 'point_time_absolute': 'true', 'publish_tf': 'false',
            'rviz': 'false',
            'deskewed_cloud_frame': contract.frame('lidar_compensated'),
            'backend_tf_topic': '/sentry/tf/localization',
        }),
        include(Path(bringup_share) / 'launch/zlio_base_adapter.launch.py', {
            'use_sim_time': 'true',
            'tf_output_topic': '/sentry/tf/odometry',
        }),
        include(Path(bringup_share) / 'launch/navigation.launch.py', {
            'idle_scan': lc('idle_scan'),
            'enable_localization': 'true', 'localization_mode': 'external',
            'use_sim_time': 'true', 'rviz': lc('rviz'),
            'map': lc('external_map'), 'params_file': lc('external_nav2_params'),
            'robot_geometry_file': lc('robot_geometry_file'), 'planner_tolerance': '0.12',
        }, condition=IfCondition(LaunchConfiguration('start_navigation'))),
    ]


def generate_launch_description():
    sim_share = FindPackageShare('xxu_sim_bringup')
    defaults = {
        'field_model': PathJoinSubstitution([sim_share, 'models', 'rmul_2027', 'model.sdf']),
        'spawn_x': '2.0', 'spawn_y': '2.0', 'spawn_yaw': '0.0',
        'gazebo_gui': 'true', 'rviz': 'true',
        'start_navigation': 'true', 'enable_localization': 'true', 'dynamic_test_vehicle': 'false',
        'idle_scan': 'false',
        'clock_publish_rate': '200.0',
        'lidar_mode': 'rgl', 'rgl_plugin_dir': '',
        'robot_model': 'ega_folded',
        'robot_geometry_file': RobotGeometryFile(LaunchConfiguration('robot_model'), sim_share),
    }
    return LaunchDescription([
        DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()
    ] + [
        DeclareLaunchArgument('imu_sampling', default_value='direct', choices=['direct', 'interval_mean']),
        DeclareLaunchArgument('localization_backend', default_value='zlio_tdt', choices=['zlio_tdt']),
        *(DeclareLaunchArgument(name, default_value='') for name in
          ('frontend_config', 'backend_config', 'prior_map', 'external_map', 'external_nav2_params')),
        LogInfo(msg='RMUL user field and folded EGA vehicle; CAD mass with corrected lidar inertias, uncalibrated contact/actuator model'),
        OpaqueFunction(function=external_setup),
    ])
