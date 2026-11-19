"""Use pinned rm_sim_26 geometry with the existing XXU estimation/safety chain."""


import os
import math
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from xxu_bringup.interfaces import interfaces
from xxu_bringup.robot_profiles import robot_profile
from xxu_bringup.sensor_launch import navigation_static_actions, sensor_preprocessing
from xxu_bringup.velocity_launch import velocity_chain
from xxu_sim_bringup.sim_profiles import chassis_parameters


def prepare_assets(share, directory, lidar_mode='gpu_lidar', field_model='', robot_model='upstream',
                   rgl_point_timing='off', imu_sampling='direct'):
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

    profile = robot_profile(robot_model, get_package_share_directory('xxu_sim_bringup'))
    if profile.geometry:
        if lidar_mode != 'rgl':
            raise ValueError('CAD vehicles currently require lidar_mode:=rgl')
        folder = os.path.dirname(profile.geometry_file)
        model = ET.parse(os.path.join(folder, 'model.sdf'))
        if imu_sampling == 'interval_mean':
            if robot_model != 'ega_folded':
                raise ValueError('interval_mean is validated only for the fixed EGA simulation IMU')
            sensors = model.findall(".//sensor[@type='imu']")
            if len(sensors) != 1 or float(sensors[0].findtext('update_rate')) != 100.0:
                raise ValueError('interval_mean requires exactly one original 100 Hz IMU')
            sensors[0].find('update_rate').text = '1000'
            for noise in sensors[0].findall('.//noise/stddev'):
                noise.text = str(float(noise.text) * math.sqrt(10.0))
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

    if imu_sampling != 'direct':
        raise ValueError('interval_mean requires the EGA fixed sensor model')
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
    sim_share = get_package_share_directory('xxu_sim_bringup')
    profile = robot_profile(LaunchConfiguration('robot_model').perform(context), sim_share)
    custom_robot = bool(profile.geometry)
    geometry = profile.geometry
    lidar_mode = LaunchConfiguration('lidar_mode').perform(context)
    rgl_point_timing = LaunchConfiguration('rgl_point_timing').perform(context)
    if rgl_point_timing == 'uniform' and lidar_mode != 'rgl':
        raise ValueError('rgl_point_timing:=uniform requires lidar_mode:=rgl')
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
        LaunchConfiguration('robot_model').perform(context), rgl_point_timing,
        LaunchConfiguration('imu_sampling').perform(context))
    lc = LaunchConfiguration
    sim = {'use_sim_time': True}
    def node(package, executable, name, parameters=None, **kwargs):
        return Node(package=package, executable=executable, name=name,
                    output='screen', parameters=[sim, parameters or {}], **kwargs)

    def bridge(name, arguments, remappings=None):
        return node('ros_gz_bridge', 'parameter_bridge', name,
                    arguments=arguments, remappings=remappings or [])

    second_actions = []
    if lc('dynamic_test_vehicle').perform(context) == 'true':
        if profile.name != 'ega_folded':
            raise ValueError('dynamic_test_vehicle requires robot_model:=ega_folded')
        second = ET.parse(model)
        for link in second.findall('.//link'):
            for sensor in list(link.findall('sensor')):
                link.remove(sensor)
        for plugin in list(second.getroot().find('model').findall('plugin')):
            if plugin.get('name') != 'gz::sim::systems::JointController':
                second.getroot().find('model').remove(plugin)
            else:
                topic = plugin.find('topic')
                topic.text = '/dynamic_test' + topic.text
        second_path = os.path.join(temporary.name, 'dynamic_test_vehicle.sdf')
        second.write(second_path)
        wheels = [item['name'] for item in geometry['wheels']]
        params = chassis_parameters(profile)
        params.update({'cmd_vel_topic': '/dynamic_test/cmd_vel',
                       'wheel_command_topics': [f'/dynamic_test/{wheel}/cmd_vel' for wheel in wheels]})
        x = 12.6 - float(lc('spawn_x').perform(context))
        y = 8.6 - float(lc('spawn_y').perform(context))
        yaw = float(lc('spawn_yaw').perform(context)) + math.pi
        second_actions = [
            LogInfo(msg=f'Dynamic test vehicle spawn: ({x:.3f}, {y:.3f}), yaw={yaw:.3f}'),
            TimerAction(period=3.0, actions=[node('ros_gz_sim', 'create', 'spawn_dynamic_test_vehicle',
                arguments=['-file', second_path, '-name', 'dynamic_test_vehicle',
                           '-x', str(x), '-y', str(y), '-z', '0.3', '-Y', str(yaw)])]),
            bridge('dynamic_test_wheel_bridge', [
                f'/dynamic_test/{wheel}/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double'
                for wheel in wheels]),
            node('xxu_chassis_controller', 'chassis_controller', 'dynamic_test_chassis_controller', params),
        ]

    actions = [
        LogInfo(msg=f'rm_sim_26 lidar_mode={lidar_mode}; robot={profile.name}; '
                    f'rgl_point_timing={rgl_point_timing} (uniform is an assumed scan model)'),
        *rgl_actions,
        *second_actions,
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temporary.cleanup())])),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(filter(None, [
            os.path.join(share, 'models'), os.path.join(sim_share, 'models'), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ('-r ' if lc('gazebo_gui').perform(context) == 'true'
                                         else '-r -s --headless-rendering ') + world,
                              'on_exit_shutdown': 'true'}.items()),
        *navigation_static_actions(robot, contract.frame('navigation_base'),
            [contract.frame('lidar_compensated')] +
            ([contract.frame('rm_lidar'), 'livox_lidar_imu'] if custom_robot else []),
            use_sim_time=True),
        TimerAction(period=3.0, actions=[node('ros_gz_sim', 'create', 'spawn_sentry',
            arguments=['-file', model, '-name', 'sentry_bot',
                       '-x', lc('spawn_x'), '-y', lc('spawn_y'), '-z', '0.3',
                       '-Y', lc('spawn_yaw')])]),
        node('ros_gz_bridge', 'parameter_bridge', 'rm_clock_bridge', {
            'use_sim_time': False, 'bridge_names': ['clock'],
            'bridges.clock.ros_topic_name': '/sentry/simulation/clock_raw',
            'bridges.clock.gz_topic_name': contract.topic('system.clock'),
            'bridges.clock.ros_type_name': 'rosgraph_msgs/msg/Clock',
            'bridges.clock.gz_type_name': 'gz.msgs.Clock',
            'bridges.clock.direction': 'GZ_TO_ROS', 'bridges.clock.qos_profile': 'CLOCK',
        }),
        node('xxu_sim_bringup', 'clock_rate_limiter', 'rm_clock_limiter', {
            'use_sim_time': False,
            'publish_rate': ParameterValue(lc('clock_publish_rate'), value_type=float),
        }),
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
        *([] if custom_robot else [node('xxu_sim_bringup', 'omni_drive', 'omni_drive')]),
        *([node('xxu_chassis_controller', 'chassis_controller', 'chassis_controller',
                 chassis_parameters(profile))] if custom_robot else
           [node('xxu_sim_bringup', 'rm_sim_adapter', 'rm_sim_adapter')]),
        node('xxu_livox_sim', 'livox_pointcloud_shaper', 'rm_lidar_shaper', {
            'use_sim_time': False,  # Point times come from the input cloud, not /clock.
            'target_frame': contract.frame('rm_lidar'), 'scan_period': 0.0, 'max_points': 12000,
            'input_time_offset_field': 'offset_time' if rgl_point_timing == 'uniform' else ''},
            remappings=[('points_in', contract.topic('simulation.rm_lidar')),
                        ('points_out', contract.topic('simulation.lidar_shaped'))]),
        *sensor_preprocessing('rm_sim', use_sim_time=True, profile=profile,
                              imu_overrides={'imu_interval_samples': 10 if LaunchConfiguration('imu_sampling').perform(context) == 'interval_mean' else 1},
                              pointcloud_overrides={'use_point_timestamps': False}
                              if custom_robot and rgl_point_timing == 'uniform' else None),
        node('pointcloud_to_laserscan', 'pointcloud_to_laserscan_node', 'pointcloud_to_laserscan', {
            'use_sim_time': False, 'target_frame': contract.frame('navigation_base'), 'transform_tolerance': 0.05,
            'min_height': float(lc('scan_min_height').perform(context)) - geometry['base_height']
                          if custom_robot else ParameterValue(lc('scan_min_height'), value_type=float),
            'max_height': geometry['collision_height'] + .02 - geometry['base_height'] if custom_robot else 0.50,
            'angle_min': -3.1415926, 'angle_max': 3.1415926,
            'angle_increment': 0.0087266, 'scan_time': 0.1, 'range_min': 0.3,
            'range_max': 40.0,
            'robot_self_filter_radius': geometry['robot_radius'] if custom_robot else ParameterValue(lc('scan_self_filter_radius'), value_type=float),
            'use_inf': True, 'scan_qos_reliability': 'best_effort',
            'publish_processed_cloud': False, 'processed_cloud_frame': contract.frame('navigation_base'),
            'processed_cloud_project_to_2d': True, 'processed_cloud_z_value': 0.0},
            remappings=[('cloud_in', contract.topic('localization.deskewed_cloud')), ('scan', contract.topic('localization.scan')),
                        ('processed_cloud', contract.topic('localization.navigation_cloud'))]),
        *velocity_chain(
            use_sim_time=True,
            cmd_vel_topic=lc('cmd_vel_out_topic'),
            output_topic=contract.topic('simulation.safe_velocity'),
            # The sensor is fixed: check its TF chain exists, but require a
            # fresh stream only for the dynamic odometry edge.
            watchdog_parameters={
                'required_tf_pairs': [f"{contract.frame('odom')}->{contract.frame('navigation_base')}"],
                'tf_source_frame': contract.frame('lidar_compensated'),
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
        'gazebo_gui': 'true',
        'gimbal_mode': 'hold',
        'spawn_x': '3.4', 'spawn_y': '9.5', 'spawn_yaw': '0.0',
        'scan_min_height': '0.10', 'scan_self_filter_radius': '0.42',
        'cmd_vel_out_topic': contract.topic('navigation.collision_output'),
        'rgl_plugin_dir': '', 'field_model': '',
        'robot_model': 'upstream', 'dynamic_test_vehicle': 'false',
        'clock_publish_rate': '200.0',
    }
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value)
                              for key, value in defaults.items()] + [
        SetEnvironmentVariable(name='OMP_NUM_THREADS', value='2'),
        DeclareLaunchArgument('lidar_mode', default_value='rgl',
                              choices=['rgl', 'gpu_lidar']),
        DeclareLaunchArgument('imu_sampling', default_value='direct', choices=['direct', 'interval_mean']),
        DeclareLaunchArgument('rgl_point_timing', default_value='off', choices=['off', 'uniform']),
        DeclareLaunchArgument('require_localization', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('localization_state_topic', default_value='/backend_node/localization_state'),
        DeclareLaunchArgument('localization_pose_topic', default_value='/localization/odometry'),
        DeclareLaunchArgument('localization_timeout', default_value='0.5'),
        DeclareLaunchArgument('localization_recovery_hold', default_value='1.0'),
        OpaqueFunction(function=launch_setup)])
