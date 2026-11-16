"""Rigid real-sensor localization, C++ bridges and deskewed navigation scan.

Requires measured mount extrinsics, a matching backend/map configuration and
explicit driver configuration. Does not start chassis hardware or send goals.
"""
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from xxu_bringup.sensor_launch import load_rigid_mount, rigid_sensor_actions


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    mount = load_rigid_mount(arg('mount_file'))
    if mount['frames']['base'] != 'base_link':
        raise ValueError('Sentry integration requires measured extrinsics relative to base_link')
    for name in ('frontend_config', 'backend_config'):
        if not Path(arg(name)).is_file():
            raise ValueError(f'{name} must be an existing configuration')
    frontend = yaml.safe_load(Path(arg('frontend_config')).read_text())
    # Both inputs have already been transformed into the same rigid IMU-origin
    # frame. Apply no second lidar/IMU extrinsic inside the estimator.
    frontend.setdefault('sensor', {})['body_from_lidar'] = {
        'rotation_xyzw': [0., 0., 0., 1.], 'translation': [0., 0., 0.]}
    temporary = tempfile.TemporaryDirectory(prefix='xxu_sentry_')
    config = Path(temporary.name) / 'lio.yaml'
    config.write_text(yaml.safe_dump(frontend, sort_keys=False))
    simulated = arg('use_sim_time') == 'true'
    frames, topics = mount['frames'], mount['topics']
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda context: temporary.cleanup())]))]
    actions += rigid_sensor_actions(mount, use_sim_time=simulated,
                                    publish_mount_tf=arg('publish_mount_tf') == 'true')
    actions += [
        ComposableNodeContainer(
            name='sentry_bridge_container', namespace='', package='rclcpp_components',
            executable='component_container_mt', output='screen',
            composable_node_descriptions=[
                ComposableNode(package='xxu_bringup', plugin='xxu_bringup::ZlioBaseOdometry',
                    name='zlio_base_odometry', parameters=[{
                        'use_sim_time': simulated, 'base_frame': 'base_link',
                        'sensor_frame': frames['sensor'], 'source_child_frame': 'imu',
                        'tf_publish_rate': 30.0,
                        'extrinsics_source': 'parameters',
                        'base_from_sensor_xyz': mount['base_to_sensor']['xyz'],
                        'base_from_sensor_rpy': mount['base_to_sensor']['rpy']}],
                    remappings=[('/tf', '/sentry/tf/odometry')]),
                ComposableNode(package='xxu_bringup', plugin='xxu_bringup::CmdVelWatchdog',
                    name='cmd_vel_watchdog', parameters=[{
                        'use_sim_time': simulated, 'publish_rate': 100.,
                        'require_localization': True,
                        'require_joint_states': arg('require_joint_states') == 'true',
                        'required_tf_pairs': ['odom->base_link'],
                        'tf_source_frame': frames['lidar']}]),
            ]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('bringup')) / 'launch/bringup.launch.py')),
            launch_arguments={
                'frontend_config': str(config), 'backend_config': arg('backend_config'),
                'prior_map': arg('prior_map'), 'mode': arg('mode'),
                'use_sim_time': arg('use_sim_time'), 'driver': 'false',
                'input_type': 'pointcloud2', 'lidar_topic': topics['points_out'],
                'imu_topic': topics['imu_out'], 'imu_acceleration_scale': '1.0',
                'point_time_field': 'timestamp', 'point_time_scale': '1e-9',
                'point_time_absolute': 'true', 'publish_tf': 'false',
                'deskewed_cloud_frame': frames['sensor'],
                'backend_tf_topic': '/sentry/tf/localization', 'rviz': 'false',
            }.items()),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan', output='screen', parameters=[{
                'use_sim_time': simulated, 'target_frame': 'base_link',
                'scan_qos_reliability': 'best_effort', 'scan_time': .1,
                'min_height': float(arg('scan_min_height')), 'max_height': float(arg('scan_max_height'))}],
            remappings=[('cloud_in', '/cloud_deskewed'), ('scan', '/scan')]),
    ]
    if arg('driver') == 'true':
        if not Path(arg('driver_config')).is_file():
            raise ValueError('driver:=true requires an existing driver_config')
        actions.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen', parameters=[{
                'xfer_format': 0, 'multi_topic': 0, 'data_src': 0, 'output_data_type': 0,
                'publish_freq': 10., 'frame_id': frames['lidar'],
                'user_config_path': arg('driver_config'), 'cmdline_input_bd_code': 'livox0000000001'}],
            remappings=[('/livox/lidar', topics['points_in']), ('/livox/imu', topics['imu_in'])]))
    return actions


def generate_launch_description():
    defaults = {
        'mount_file': '', 'frontend_config': '', 'backend_config': '', 'prior_map': '',
        'mode': 'localization', 'driver': 'false', 'driver_config': '',
        'use_sim_time': 'false', 'publish_mount_tf': 'true', 'require_joint_states': 'true',
        'scan_min_height': '0.05', 'scan_max_height': '0.5',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        SetEnvironmentVariable(name='OMP_NUM_THREADS', value='2'),
        OpaqueFunction(function=setup),
    ])
