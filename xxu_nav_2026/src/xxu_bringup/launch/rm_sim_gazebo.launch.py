"""Use pinned rm_sim_26 geometry with the existing XXU estimation/safety chain."""


import os
import tempfile
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from xxu_bringup.interfaces import interfaces
from xxu_bringup.robot_profiles import chassis_parameters, robot_profile
from xxu_bringup.sensor_launch import sensor_preprocessing, small_point_lio
from xxu_bringup.velocity_launch import velocity_chain


def prepare_assets(share, directory, lidar_mode='gpu_lidar', field_model='', robot_model='upstream',
                   rgl_point_timing='off'):
    """Adapt copies only; preserve the pinned upstream sources."""
    contract = interfaces()
    world = ET.parse(os.path.join(share, 'worlds', 'rmuc_2025_world.world'))
    for plugin in list(world.getroot().find('world').findall('plugin')):
        if lidar_mode == 'gpu_lidar' and 'RGL' in plugin.get('filename', ''):
            world.getroot().find('world').remove(plugin)
    if field_model:
        scene = world.getroot().find('world')
        field = ET.parse(field_model).getroot().find('model')
        if field is None or field.findtext('static') != 'true':
            raise ValueError('field_model must contain one static SDF model')
        # Resolve local mesh paths before embedding the field in a temporary world.
        for uri in field.findall('.//mesh/uri'):
            if not os.path.isabs(uri.text) and '://' not in uri.text:
                uri.text = os.path.abspath(os.path.join(os.path.dirname(field_model), uri.text))
        old_field = scene.find("model[@name='rmuc_2025']")
        scene.remove(old_field)
        scene.append(field)
        camera = scene.find("gui/plugin[@filename='MinimalScene']/camera_pose")
        if camera is not None:
            camera.text = '6.3 4.3 13 0 1.57 1.57'
    world_path = os.path.join(directory, 'rmuc_2025.world')
    world.write(world_path)

    profile = robot_profile(robot_model, get_package_share_directory('xxu_bringup'))
    if profile.geometry:
        if lidar_mode != 'rgl':
            raise ValueError('CAD vehicles currently require lidar_mode:=rgl')
        folder = os.path.dirname(profile.geometry_file)
        model = ET.parse(os.path.join(folder, 'model.sdf'))
        if rgl_point_timing == 'uniform':
            plugins = model.findall(".//sensor/plugin[@name='rgl::RGLServerPluginInstance']")
            if len(plugins) != 1:
                raise ValueError('expected exactly one RGL sensor plugin')
            ET.SubElement(plugins[0], 'timed_scan_uniform').text = 'true'
        for uri in model.findall('.//mesh/uri'):
            uri.text = os.path.abspath(os.path.join(folder, uri.text))
        model_path = os.path.join(directory, f'{profile.name}.sdf')
        model.write(model_path)
        with open(os.path.join(folder, 'robot.urdf')) as handle:
            robot = handle.read()
        return world_path, model_path, robot

    model = ET.parse(os.path.join(share, 'models', 'sentry', 'model.sdf'))
    link = model.getroot().find("model/link[@name='livox_lidar']")
    rgl_sensor = link.find("sensor[@name='RGLLidar']")
    if lidar_mode == 'rgl':
        plugin = rgl_sensor.find('plugin')
        plugin.find('topic').text = contract.topic('simulation.rm_lidar')
        plugin.find('range/min').text = '0.1'
        plugin.find('range/max').text = '40'
        if rgl_point_timing == 'uniform':
            ET.SubElement(plugin, 'timed_scan_uniform').text = 'true'
        plugin.remove(plugin.find('pattern_preset_path'))
        # Named presets carry the 40-frame alternation contract. The path
        # loader would cast the complete four-second file in every frame.
        ET.SubElement(plugin, 'pattern_preset').text = 'Livox Mid360'
        # The Livox casing is part of the parent gimbal visual. RGL ignores
        # geometry in the sensor's own link; attaching it to the empty IMU
        # link instead makes every ray stop on that casing below min range.
        # Keep the same physical sensor pose and ROS frame/extrinsics.
        link.remove(rgl_sensor)
        model.getroot().find("model/link[@name='gimbal_link']").append(rgl_sensor)
        sensor_pose = link.findtext('pose').split()
        ET.SubElement(rgl_sensor, 'pose').text = ' '.join(sensor_pose)
    else:
        link.remove(rgl_sensor)
    # One gpu_lidar frame is simultaneous. The shaper supplies identical point
    # times, never invented scan-time offsets, to the existing Livox adapter.
        link.append(ET.fromstring('''<sensor name="livox_lidar" type="gpu_lidar">
      <always_on>true</always_on><update_rate>10</update_rate>
      <topic>/rm_sim/lidar</topic><gz_frame_id>livox_lidar</gz_frame_id>
      <lidar><scan>
        <horizontal><samples>1875</samples><resolution>1</resolution>
          <min_angle>0</min_angle><max_angle>6.2831852</max_angle></horizontal>
        <vertical><samples>32</samples><resolution>1</resolution>
          <min_angle>-0.12217304764</min_angle><max_angle>0.90757121104</max_angle></vertical>
      </scan><range><min>0.1</min><max>40</max><resolution>0.01</resolution></range>
      <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
      </lidar></sensor>'''))
    model_path = os.path.join(directory, 'sentry.sdf')
    pose_plugin = model.getroot().find("model/plugin[@name='gz::sim::systems::PosePublisher']")
    ET.SubElement(pose_plugin, 'publish_model_pose').text = 'true'
    ET.SubElement(pose_plugin, 'use_pose_vector_msg').text = 'true'
    ET.SubElement(pose_plugin, 'update_frequency').text = '100'
    model.write(model_path)
    urdf = ET.parse(os.path.join(share, 'models', 'sentry', 'robot.urdf'))
    root = urdf.getroot()
    ET.SubElement(root, 'link', name=contract.frame('ground_base'))
    joint = ET.SubElement(root, 'joint', name='footprint_joint', type='fixed')
    ET.SubElement(joint, 'parent', link=contract.frame('ground_base'))
    ET.SubElement(joint, 'child', link=contract.frame('navigation_base'))
    ET.SubElement(joint, 'origin', xyz='0 0 0.096', rpy='0 0 0')
    ET.SubElement(root, 'link', name=contract.frame('lidar_compensated'))
    joint = ET.SubElement(root, 'joint', name='lio_sensor_joint', type='fixed')
    ET.SubElement(joint, 'parent', link=contract.frame('navigation_base'))
    ET.SubElement(joint, 'child', link=contract.frame('lidar_compensated'))
    ET.SubElement(joint, 'origin', xyz='0 0 0.0845', rpy='0 0 0')
    return world_path, model_path, ET.tostring(root, encoding='unicode')


def launch_setup(context):
    contract = interfaces()
    if LaunchConfiguration('gimbal_mode').perform(context) != 'hold':
        raise ValueError('rm_sim_26 currently supports gimbal_mode:=hold')
    share = get_package_share_directory('rm_sim_26')
    bringup = get_package_share_directory('xxu_bringup')
    profile = robot_profile(LaunchConfiguration('robot_model').perform(context), bringup)
    custom_robot = bool(profile.geometry)
    geometry = profile.geometry
    lidar_mode = LaunchConfiguration('lidar_mode').perform(context)
    rgl_point_timing = LaunchConfiguration('rgl_point_timing').perform(context)
    enable_zlio_timed = LaunchConfiguration('enable_zlio_timed').perform(context) == 'true'
    if rgl_point_timing == 'uniform' and lidar_mode != 'rgl':
        raise ValueError('rgl_point_timing:=uniform requires lidar_mode:=rgl')
    if enable_zlio_timed and (rgl_point_timing != 'uniform' or not custom_robot or
                              LaunchConfiguration('enable_lio').perform(context) != 'false'):
        raise ValueError('enable_zlio_timed requires a fixed-sensor CAD robot, '
                         'rgl_point_timing:=uniform and enable_lio:=false')
    rgl_actions = []
    if lidar_mode == 'rgl':
        selected = LaunchConfiguration('rgl_plugin_dir').perform(context)
        candidates = [selected] if selected else [os.path.join(
            os.path.dirname(get_package_prefix('rm_sim_26')),
            'rgl_plugin', 'RGLServerPlugin')]
        if not selected:
            candidates.extend(filter(None, os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '').split(':')))
        required = ('libRobotecGPULidar.so', 'libRGLServerPluginInstance.so',
                    'libRGLServerPluginManager.so')
        plugin_dir = next((path for path in candidates if all(
            os.path.isfile(os.path.join(path, name)) for name in required)), None)
        if plugin_dir is None:
            raise RuntimeError('RGL libraries not found. Run tools/navigation/build_rgl.sh '
                               'or pass rgl_plugin_dir:=<RGLServerPlugin directory>; '
                               'lidar_mode:=gpu_lidar selects the fallback explicitly.')
        pattern_dir = os.path.join(share, 'plugin')
        if not os.path.isfile(os.path.join(pattern_dir, 'LivoxMid360.mat3x4f')):
            raise RuntimeError('Pinned LivoxMid360 pattern file is missing')
        rgl_actions = [
            SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', ':'.join(filter(None, [
                plugin_dir, os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')]))),
            SetEnvironmentVariable('RGL_PATTERNS_DIR', pattern_dir),
        ]
    temporary = tempfile.TemporaryDirectory(prefix='xxu_rm_sim_')
    world, model, robot = prepare_assets(
        share, temporary.name, lidar_mode, LaunchConfiguration('field_model').perform(context),
        LaunchConfiguration('robot_model').perform(context), rgl_point_timing)
    lc = LaunchConfiguration
    sim = {'use_sim_time': True}
    zlio_config = None
    if enable_zlio_timed:
        # Both preprocessing outputs are already in the same virtual sensor frame.
        source_config = os.path.join(get_package_share_directory('frontend'), 'config', 'lio.yaml')
        with open(source_config, encoding='utf-8') as handle:
            settings = yaml.safe_load(handle)
        settings['sensor']['body_from_lidar'] = {
            'rotation_xyzw': [0, 0, 0, 1], 'translation': [0, 0, 0]}
        zlio_config = os.path.join(temporary.name, 'zlio_virtual_sensor.yaml')
        with open(zlio_config, 'w', encoding='utf-8') as handle:
            yaml.safe_dump(settings, handle)

    def node(package, executable, name, parameters=None, **kwargs):
        return Node(package=package, executable=executable, name=name,
                    output='screen', parameters=[sim, parameters or {}], **kwargs)

    def bridge(name, arguments, remappings=None):
        return node('ros_gz_bridge', 'parameter_bridge', name,
                    arguments=arguments, remappings=remappings or [])

    actions = [
        LogInfo(msg=f'rm_sim_26 lidar_mode={lidar_mode}; robot={profile.name}; '
                    f'rgl_point_timing={rgl_point_timing} (uniform is an assumed scan model)'),
        *rgl_actions,
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temporary.cleanup())])),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(filter(None, [
            os.path.join(share, 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ('-r ' if lc('gazebo_gui').perform(context) == 'true'
                                         else '-r -s --headless-rendering ') + world,
                              'on_exit_shutdown': 'true'}.items()),
        node('robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
             {'robot_description': robot, 'publish_frequency': 100.0}),
        TimerAction(period=3.0, actions=[node('ros_gz_sim', 'create', 'spawn_sentry',
            arguments=['-file', model, '-name', 'sentry_bot',
                       '-x', lc('spawn_x'), '-y', lc('spawn_y'), '-z', '0.3',
                       '-Y', lc('spawn_yaw')])]),
        bridge('rm_clock_bridge', [contract.topic('system.clock') + '@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
        bridge('rm_sensor_bridge', [
            contract.topic('simulation.rm_lidar') + '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            contract.topic('radar.imu') + '@sensor_msgs/msg/Imu[gz.msgs.IMU',
            contract.topic('gimbal.joint_states') + '@sensor_msgs/msg/JointState[gz.msgs.Model']),
        # Diagnostics only. Ground truth never supplies odom or navigation TF.
        bridge('rm_truth_bridge', [
            contract.topic('simulation.model_pose') + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            remappings=[(contract.topic('simulation.model_pose'), contract.topic('simulation.truth'))]),
        bridge('rm_command_bridge', [
            contract.topic('simulation.wheel_command', wheel=wheel) + '@std_msgs/msg/Float64]gz.msgs.Double'
            for wheel in ([item['name'] for item in geometry['wheels']] if custom_robot else
                          ('wheel_fl', 'wheel_rl', 'wheel_rr', 'wheel_fr'))
        ] + ([] if custom_robot else [contract.topic(f'simulation.gimbal_{axis}') + '@std_msgs/msg/Float64]gz.msgs.Double'
             for axis in ('big_yaw', 'small_yaw', 'pitch')])),
        *([] if custom_robot else [node('rm_sim_26', 'omni_drive.py', 'omni_drive')]),
        *([node('xxu_chassis_controller', 'chassis_controller', 'chassis_controller',
                 chassis_parameters(profile))] if custom_robot else
           [node('xxu_bringup', 'rm_sim_adapter.py', 'rm_sim_adapter')]),
        node('xxu_livox_sim', 'livox_pointcloud_shaper', 'rm_lidar_shaper', {
            'target_frame': contract.frame('rm_lidar'), 'scan_period': 0.0, 'max_points': 12000,
            'input_time_offset_field': 'offset_time' if rgl_point_timing == 'uniform' else ''},
            remappings=[('points_in', contract.topic('simulation.rm_lidar')),
                        ('points_out', contract.topic('simulation.lidar_shaped'))]),
        *sensor_preprocessing('rm_sim', use_sim_time=True, profile=profile,
                              pointcloud_overrides={'use_point_timestamps': False}
                              if custom_robot and rgl_point_timing == 'uniform' else None),
        small_point_lio(
            'rm_sim', use_sim_time=True, profile=profile,
            config_file=os.path.join(bringup, 'config', 'rm_sim_lio.yaml'),
            condition=IfCondition(lc('enable_lio'))),
        *([node('frontend', 'zlio_node', 'zlio_timed_candidate', {
            'config_file': zlio_config,
            'lidar_topic': contract.topic('simulation.lidar_compensated'),
            'imu_topic': contract.topic('simulation.imu_compensated'),
            'input_type': 'pointcloud2', 'point_time_field': 'timestamp',
            'point_time_scale': 1e-9, 'point_time_absolute': True,
            'imu_acceleration_scale': 1.0, 'publish_tf': False})] if enable_zlio_timed else []),
        node('pointcloud_to_laserscan', 'pointcloud_to_laserscan_node', 'pointcloud_to_laserscan', {
            'target_frame': contract.frame('ground_base'), 'transform_tolerance': 0.05,
            'min_height': ParameterValue(lc('scan_min_height'), value_type=float),
            'max_height': geometry['collision_height'] + .02 if custom_robot else 0.50,
            'angle_min': -3.1415926, 'angle_max': 3.1415926,
            'angle_increment': 0.0087266, 'scan_time': 0.1, 'range_min': 0.3,
            'range_max': 40.0,
            'robot_self_filter_radius': geometry['robot_radius'] if custom_robot else ParameterValue(lc('scan_self_filter_radius'), value_type=float),
            'use_inf': True, 'scan_qos_reliability': 'reliable',
            'publish_processed_cloud': True, 'processed_cloud_frame': contract.frame('ground_base'),
            'processed_cloud_project_to_2d': True, 'processed_cloud_z_value': 0.0},
            remappings=[('cloud_in', contract.topic('localization.deskewed_cloud')), ('scan', contract.topic('localization.scan')),
                        ('processed_cloud', contract.topic('localization.navigation_cloud'))]),
        *velocity_chain(
            use_sim_time=True, use_fake_frame=lc('use_fake_frame'),
            cmd_vel_topic=lc('cmd_vel_out_topic'),
            spin_speed=ParameterValue(lc('gyro_spin_rate'), value_type=float),
            velocity_window=ParameterValue(lc('nav_velocity_smoothing_window'), value_type=float),
            output_topic=contract.topic('simulation.safe_velocity'),
            # The sensor is fixed: check its TF chain exists, but require a
            # fresh stream only for the dynamic odometry edge.
            watchdog_parameters={
                **({'required_tf_pairs': [f"{contract.frame('odom')}->{contract.frame('ground_base')}"],
                    'tf_source_frame': contract.frame('rm_lidar')} if custom_robot else {}),
                'require_localization': ParameterValue(lc('require_localization'), value_type=bool),
                'localization_state_topic': lc('localization_state_topic'),
                'localization_pose_topic': lc('localization_pose_topic'),
                'localization_timeout': ParameterValue(lc('localization_timeout'), value_type=float),
                'localization_recovery_hold': ParameterValue(lc('localization_recovery_hold'), value_type=float),
            },
        ),
    ]
    return actions


def generate_launch_description():
    contract = interfaces()
    defaults = {
        'gazebo_gui': 'true', 'enable_lio': 'true', 'use_fake_frame': 'true',
        'gimbal_mode': 'hold',
        'spawn_x': '3.4', 'spawn_y': '9.5', 'spawn_yaw': '0.0',
        'scan_min_height': '0.10', 'scan_self_filter_radius': '0.42',
        'gyro_spin_rate': '1.5', 'nav_velocity_smoothing_window': '0.3',
        'cmd_vel_out_topic': contract.topic('navigation.collision_output'),
        'rgl_plugin_dir': '', 'field_model': '',
        'robot_model': 'upstream',
    }
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [
        DeclareLaunchArgument('lidar_mode', default_value='rgl',
                              choices=['rgl', 'gpu_lidar']),
        DeclareLaunchArgument('rgl_point_timing', default_value='off', choices=['off', 'uniform']),
        DeclareLaunchArgument('enable_zlio_timed', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('require_localization', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('localization_state_topic', default_value='/backend_node/localization_state'),
        DeclareLaunchArgument('localization_pose_topic', default_value='/localization/odometry'),
        DeclareLaunchArgument('localization_timeout', default_value='0.5'),
        DeclareLaunchArgument('localization_recovery_hold', default_value='1.0'),
        OpaqueFunction(function=launch_setup)])
