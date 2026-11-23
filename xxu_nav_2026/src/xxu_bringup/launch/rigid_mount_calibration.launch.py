"""Capture rigid extrinsics from raw LiDAR, IMU and physical-IMU ZLIO odometry."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


DEFAULTS = {
    'driver': 'false', 'driver_config': '', 'start_zlio': 'true', 'frontend_config': '',
    'lidar_input_type': 'pointcloud2', 'lidar_topic': '/livox/lidar', 'imu_topic': '/livox/imu',
    # livox_ros_driver2 PointCloud2 copies absolute epoch ns into timestamp (pub_handler.cpp:388, lddc.cpp:250).
    'point_time_field': 'timestamp', 'point_time_scale': '1e-9', 'point_time_absolute': 'true',
    'odom_topic': '', 'odom_frame': 'odom', 'odom_child_frame': 'imu',
    'allow_unisolated_odom_topic': 'false',
    'imu_acceleration_scale': '0.0',
    'zlio_imu_acceleration_scale': '9.81', 'base_height_above_ground': '0.0',
    'sensor_frame_mode': 'base_aligned', 'base_frame': 'base_link',
    'sensor_frame': 'lio_base_sensor', 'lidar_frame': 'livox_frame', 'imu_source_frame': 'livox_frame',
    'imu_from_lidar_xyz': '-0.011 -0.02329 0.04412', 'imu_from_lidar_rpy': '0 0 0',
    'straight_direction': '1', 'stationary_capture_sec': '3.0',
    'output_mount_file': '/tmp/rigid_mount_calibrated.yaml',
    'output_backend_body_file': '/tmp/backend_body_calibrated.yaml',
    'output_report_file': '/tmp/rigid_mount_calibration_report.yaml',
    'confirm': 'false', 'use_sim_time': 'false',
    'manual_yaw_deg': '', 'manual_tx': '', 'manual_ty': '', 'manual_tz': '',
}
BOOL_ARGS = ('driver', 'start_zlio', 'confirm', 'use_sim_time', 'point_time_absolute',
             'allow_unisolated_odom_topic')


def setup(context):
    args = {name: LaunchConfiguration(name).perform(context) for name in DEFAULTS}
    pointcloud2 = args['lidar_input_type'] == 'pointcloud2'
    start_zlio = args['start_zlio'] == 'true'
    odom_topic = args['odom_topic'] or '/calibration/lidar_odometry'
    allow_unisolated = args['allow_unisolated_odom_topic'] == 'true'
    if not allow_unisolated and odom_topic in ('/lidar_odometry', '/odometry', '/frontend/odometry', '/odom'):
        raise ValueError(
            'odom_topic points to a shared production topic that may publish compensated '
            'virtual-sensor poses; use /calibration/lidar_odometry or set '
            'allow_unisolated_odom_topic:=true only for raw-input ZLIO')
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=str)
        for name in (
            'lidar_input_type', 'lidar_topic', 'imu_topic', 'sensor_frame_mode',
            'base_frame', 'sensor_frame', 'lidar_frame', 'imu_source_frame',
            'odom_frame', 'odom_child_frame',
            'output_mount_file', 'output_backend_body_file', 'output_report_file',
            'manual_yaw_deg', 'manual_tx', 'manual_ty', 'manual_tz',
        )
    }
    parameters['odom_topic'] = odom_topic
    for name in ('imu_acceleration_scale', 'base_height_above_ground', 'stationary_capture_sec'):
        parameters[name] = float(args[name])
    for name in ('imu_from_lidar_xyz', 'imu_from_lidar_rpy'):
        parameters[name] = [float(value) for value in args[name].split()]
    parameters.update(straight_direction=int(args['straight_direction']),
                      allow_unisolated_odom_topic=allow_unisolated,
                      confirm=args['confirm'] == 'true', use_sim_time=args['use_sim_time'] == 'true')

    actions = []
    if args['driver'] == 'true':
        if not Path(args['driver_config']).is_file():
            raise ValueError('driver:=true requires an existing driver_config')
        actions.append(Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen', parameters=[{
                'xfer_format': 0 if pointcloud2 else 1, 'multi_topic': 0, 'data_src': 0,
                'output_data_type': 0, 'publish_freq': 10.0, 'frame_id': args['lidar_frame'],
                'user_config_path': args['driver_config'], 'cmdline_input_bd_code': 'livox0000000001',
            }], remappings=[('/livox/lidar', args['lidar_topic']), ('/livox/imu', args['imu_topic'])]))

    if start_zlio:
        share = Path(get_package_share_directory('frontend'))
        scale = (args['imu_acceleration_scale'] if parameters['imu_acceleration_scale'] != 0.0
                 else args['zlio_imu_acceleration_scale'])
        frontend = {
            'config': args['frontend_config'] or str(share / 'config/lio.yaml'),
            'publish_tf': 'false', 'input_type': args['lidar_input_type'],
            'lidar_topic': args['lidar_topic'], 'imu_topic': args['imu_topic'],
            'use_sim_time': args['use_sim_time'], 'imu_acceleration_scale': scale,
        }
        if pointcloud2:
            frontend.update(point_time_field=args['point_time_field'],
                            point_time_scale=args['point_time_scale'],
                            point_time_absolute=args['point_time_absolute'])
        source = PythonLaunchDescriptionSource(str(share / 'launch/lio.launch.py'))
        frontend_remaps = {
            '/lidar_odometry': odom_topic,
            '/odometry': '/calibration/odometry',
            '/frontend/frame': '/calibration/frontend/frame',
            '/frontend/odometry': '/calibration/frontend/odometry',
            '/frontend/session': '/calibration/frontend/session',
            '/frontend/lidar_diagnostics': '/calibration/frontend/lidar_diagnostics',
            '/cloud_deskewed': '/calibration/cloud_deskewed',
            '/localization_start': '/calibration/localization_start',
            '/zlio/reset_internal': '/calibration/zlio/reset_internal',
            '/stop': '/calibration/stop',
            '__node': 'calibration_zlio',
        }
        actions.append(GroupAction([
            *(SetRemap(src=src, dst=dst) for src, dst in frontend_remaps.items()),
            IncludeLaunchDescription(source, launch_arguments=frontend.items()),
        ]))

    actions.append(Node(package='xxu_bringup', executable='rigid_mount_calibration_node',
                        name='rigid_mount_calibration', parameters=[parameters], output='screen'))
    return actions


def generate_launch_description():
    choices = {name: ['true', 'false'] for name in BOOL_ARGS}
    choices.update(lidar_input_type=['pointcloud2', 'livox'],
                   sensor_frame_mode=['base_aligned', 'imu_aligned'])
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value, choices=choices.get(name))
          for name, value in DEFAULTS.items()),
        OpaqueFunction(function=setup),
    ])
