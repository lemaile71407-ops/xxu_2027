"""Rigid real-sensor localization, C++ bridges and deskewed navigation scan.

Requires measured mount extrinsics, a matching backend/map configuration and
explicit driver configuration. Does not start chassis hardware or send goals.
"""
import math
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
from xxu_bringup.interfaces import interfaces


def _launch_bool(value, name):
    normalized = str(value).strip().lower()
    if normalized not in ('true', 'false', '1', '0', 'yes', 'no', 'on', 'off'):
        raise ValueError(f'{name} must be a boolean launch value, got {value!r}')
    return normalized in ('true', '1', 'yes', 'on')


def _finite_float(value, name):
    try:
        parsed = float(value)
    except ValueError as error:
        raise ValueError(f'{name} must be a finite number') from error
    if not math.isfinite(parsed):
        raise ValueError(f'{name} must be a finite number')
    return parsed


def _merge_mapping(base, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge_mapping(base[key], value)
        else:
            base[key] = value
    return base


def _load_backend_params(path, ancestors=()):
    resolved = Path(path).resolve()
    if resolved in ancestors:
        raise ValueError(f'backend_config inheritance cycle: {resolved}')
    params = yaml.safe_load(resolved.read_text(encoding='utf-8')) or {}
    if not isinstance(params, dict):
        raise ValueError(f'backend_config must be a mapping: {resolved}')
    parent = params.pop('extends', None)
    if parent:
        parent_path = resolved.parent / parent
        if not parent_path.is_file():
            raise ValueError(f'backend_config parent must be an existing file: {parent_path}')
        params = _merge_mapping(_load_backend_params(parent_path, ancestors=(*ancestors, resolved)), params)
    return params


def setup(context):
    contract = interfaces()
    arg = lambda name: LaunchConfiguration(name).perform(context)
    mount = load_rigid_mount(arg('mount_file'))
    if mount['frames']['base'] != 'base_link':
        raise ValueError('Sentry integration requires measured extrinsics relative to base_link')
    for name in ('frontend_config', 'backend_config'):
        if not Path(arg(name)).is_file():
            raise ValueError(f'{name} must be an existing configuration')
    scan_min_height = _finite_float(arg('scan_min_height'), 'scan_min_height')
    scan_max_height = _finite_float(arg('scan_max_height'), 'scan_max_height')
    if not scan_min_height <= scan_max_height:
        raise ValueError('scan_min_height must be less than or equal to scan_max_height')
    try:
        scan_self_filter_radius = _finite_float(arg('scan_self_filter_radius'), 'scan_self_filter_radius')
    except ValueError as error:
        raise ValueError('scan_self_filter_radius must be a finite non-negative number') from error
    if scan_self_filter_radius < 0.0:
        raise ValueError('scan_self_filter_radius must be a finite non-negative number')
    simulated = _launch_bool(arg('use_sim_time'), 'use_sim_time')
    simulated_arg = 'true' if simulated else 'false'
    publish_mount_tf = _launch_bool(arg('publish_mount_tf'), 'publish_mount_tf')
    require_joint_states = _launch_bool(arg('require_joint_states'), 'require_joint_states')
    driver_enabled = _launch_bool(arg('driver'), 'driver')
    sensor_actions = rigid_sensor_actions(
        mount, use_sim_time=simulated, publish_mount_tf=publish_mount_tf,
        self_filter_geometry_file=arg('self_filter_geometry_file'))
    frontend = yaml.safe_load(Path(arg('frontend_config')).read_text())
    # Both inputs have already been transformed into the same rigid IMU-origin
    # frame. Apply no second lidar/IMU extrinsic inside the estimator.
    frontend.setdefault('sensor', {})['body_from_lidar'] = {
        'rotation_xyzw': [0., 0., 0., 1.], 'translation': [0., 0., 0.]}
    temporary = tempfile.TemporaryDirectory(prefix='xxu_sentry_')
    config = Path(temporary.name) / 'lio.yaml'
    config.write_text(yaml.safe_dump(frontend, sort_keys=False))
    frames, topics = mount['frames'], mount['topics']
    mode = arg('mode')
    backend_params = _load_backend_params(arg('backend_config')) if mode == 'config' else {}
    localization_enabled = mode == 'localization' or (
        mode == 'config' and bool(backend_params.get('prior_map', {}).get('enabled', False)))
    watchdog_setting = arg('enable_watchdog')
    watchdog_enabled = watchdog_setting == 'true' or (
        watchdog_setting == 'auto' and localization_enabled)
    backend_tf_topic = '/sentry/tf/localization' if watchdog_enabled else '/tf'
    bridge_components = [
        ComposableNode(
            package='xxu_bringup', plugin='xxu_bringup::ZlioBaseOdometry',
            name='zlio_base_odometry', parameters=[{
                'use_sim_time': False, 'base_frame': 'base_link',
                'input_topic': contract.topic('localization.frontend_odometry'),
                'output_topic': contract.topic('localization.odometry'),
                'sensor_frame': frames['sensor'], 'source_child_frame': 'imu',
                'tf_publish_rate': 30.0,
                'extrinsics_source': 'parameters',
                'base_from_sensor_xyz': mount['base_to_sensor']['xyz'],
                'base_from_sensor_rpy': mount['base_to_sensor']['rpy']}],
            remappings=[('/tf', '/sentry/tf/odometry')] if watchdog_enabled else [],
            extra_arguments=[{'use_intra_process_comms': True}]),
    ]
    if watchdog_enabled:
        bridge_components.append(ComposableNode(
            package='xxu_bringup', plugin='xxu_bringup::CmdVelWatchdog',
            name='cmd_vel_watchdog', parameters=[{
                'use_sim_time': simulated, 'publish_rate': 100.,
                'require_localization': localization_enabled,
                'require_joint_states': require_joint_states,
                'required_tf_pairs': ['odom->base_link'],
                'tf_source_frame': frames['lidar']}],
            extra_arguments=[{'use_intra_process_comms': True}]))
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=[
        OpaqueFunction(function=lambda context: temporary.cleanup())]))]
    actions += sensor_actions
    actions += [
        ComposableNodeContainer(
            name='sentry_bridge_container', namespace='', package='xxu_bringup',
            executable='component_container_events', output='screen',
            composable_node_descriptions=bridge_components),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('bringup')) / 'launch/bringup.launch.py')),
            launch_arguments={
                'frontend_config': str(config), 'backend_config': arg('backend_config'),
                'prior_map': arg('prior_map'), 'mode': mode,
                'use_sim_time': simulated_arg, 'driver': 'false',
                'input_type': 'pointcloud2', 'lidar_topic': topics['points_out'],
                'imu_topic': topics['imu_out'], 'imu_acceleration_scale': '1.0',
                'point_time_field': 'timestamp', 'point_time_scale': '1e-9',
                'point_time_absolute': 'true', 'publish_tf': 'false',
                'deskewed_cloud_frame': frames['sensor'],
                'backend_tf_topic': backend_tf_topic, 'rviz': 'false',
            }.items()),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan', output='screen', parameters=[{
                'use_sim_time': False, 'target_frame': 'base_link',
                'scan_qos_reliability': 'best_effort', 'scan_time': .1,
                'min_height': scan_min_height, 'max_height': scan_max_height,
                'robot_self_filter_radius': scan_self_filter_radius}],
            remappings=[('cloud_in', '/cloud_deskewed'), ('scan', '/scan')]),
    ]
    if driver_enabled:
        if not Path(arg('driver_config')).is_file():
            raise ValueError('driver:=true requires an existing driver_config')
        actions.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen', parameters=[{
                'xfer_format': 0, 'multi_topic': 0, 'data_src': 0, 'output_data_type': 0,
                'publish_freq': 10., 'frame_id': frames['lidar'],
                'imu_frame_id': frames['imu_source'],
                'user_config_path': arg('driver_config'), 'cmdline_input_bd_code': 'livox0000000001'}],
            remappings=[('/livox/lidar', topics['points_in']), ('/livox/imu', topics['imu_in'])]))
    return actions


def generate_launch_description():
    defaults = {
        'mount_file': '', 'frontend_config': '', 'backend_config': '', 'prior_map': '',
        'driver': 'false', 'driver_config': '',
        'use_sim_time': 'false', 'publish_mount_tf': 'true', 'require_joint_states': 'false',
        'self_filter_geometry_file': '', 'scan_self_filter_radius': '0.0',
        'scan_min_height': '0.05', 'scan_max_height': '0.5',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        DeclareLaunchArgument('mode', default_value='localization',
                              choices=['config', 'mapping', 'localization']),
        DeclareLaunchArgument('enable_watchdog', default_value='auto',
                              choices=['auto', 'true', 'false']),
        SetEnvironmentVariable(name='OMP_NUM_THREADS', value='2'),
        OpaqueFunction(function=setup),
    ])
