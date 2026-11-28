#!/usr/bin/env python3
"""Rigid mount regressions; source ROS Jazzy, no hardware or simulation started."""
import importlib.util
import math
from pathlib import Path
import struct
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.events import Shutdown
import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
SHARE = WORKSPACE / 'src/xxu_bringup'
sys.path.insert(0, str(SHARE))
from xxu_bringup.sensor_launch import load_rigid_mount, rigid_sensor_actions, navigation_static_actions


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


entry = module('rigid_entry', SHARE / 'launch/rigid_sensor.launch.py')
sentry = module('sentry_entry', SHARE / 'launch/sentry_localization.launch.py')


def launch_context(launch_module, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in launch_module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def node_parameters(actions, executable):
    return next(action['parameters'][0] for action in actions
                if isinstance(action, dict) and action['executable'] == executable)


def measured():
    data = yaml.safe_load((SHARE / 'config/rigid_mount.pending.yaml').read_text())
    data.update(calibration_confirmed=True, imu_acceleration_scale=9.81,
                sensor_from_imu_rpy=[0., 0., math.pi / 2])
    data['base_to_sensor'] = {'xyz': [.2, -.1, .3], 'rpy': [0., 0., math.pi / 2]}
    data['sensor_to_lidar'] = {'xyz': [.01, .02, -.03], 'rpy': [0., 0., math.pi / 2]}
    return data


class MountTest(unittest.TestCase):
    def load(self, data):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'mount.yaml'
            path.write_text(yaml.safe_dump(data))
            return load_rigid_mount(path)

    def test_pending_template_is_not_a_zero_calibration(self):
        with self.assertRaisesRegex(ValueError, 'pending'):
            load_rigid_mount(SHARE / 'config/rigid_mount.pending.yaml')
        data = measured()
        data['base_to_sensor']['xyz'][0] = None
        with self.assertRaisesRegex(ValueError, 'base_to_sensor.xyz'):
            self.load(data)

    def test_invalid_transforms_frames_units_and_topic_loops_are_rejected(self):
        cases = []
        for value in (float('nan'), float('inf'), True, '0.1'):
            data = measured()
            data['sensor_to_lidar']['rpy'][0] = value
            cases.append(data)
        for value in (0, -1, None, float('inf')):
            data = measured()
            data['imu_acceleration_scale'] = value
            cases.append(data)
        data = measured()
        data['frames']['sensor'] = data['frames']['base']
        cases.append(data)
        data = measured()
        data['frames']['base'] = '/base_link'
        cases.append(data)
        data = measured()
        data['topics']['points_out'] = data['topics']['points_in']
        cases.append(data)
        data = measured()
        data['mounting'] = 'gimbal'
        cases.append(data)
        for data in cases:
            with self.subTest(data=data), self.assertRaises(ValueError):
                self.load(data)

    def test_two_chassis_do_not_share_frames_or_extrinsics(self):
        first = self.load(measured())
        second = measured()
        second['frames'].update(base='other_base', sensor='other_imu_origin', lidar='other_lidar')
        second['base_to_sensor']['xyz'] = [-.4, .3, .8]
        second = self.load(second)
        with patch('xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
            a = rigid_sensor_actions(first, use_sim_time=False)
            b = rigid_sensor_actions(second, use_sim_time=False)
        self.assertIn('base_link', a[0]['arguments'])
        self.assertIn('other_base', b[0]['arguments'])
        self.assertIn('-0.4', b[0]['arguments'])
        self.assertEqual(b[2]['parameters'][0]['target_frame'], 'other_imu_origin')
        self.assertEqual(b[3]['parameters'][0]['sensor_rpy'], second['sensor_from_imu_rpy'])
        self.assertFalse(b[2]['parameters'][0]['use_joint_interpolation'])
        self.assertEqual(b[3]['parameters'][0]['sensor_offset'], [0., 0., 0.])
        self.assertEqual(first['base_to_sensor']['xyz'], [.2, -.1, .3])

    def test_direct_mount_preserves_rotated_lever_arm(self):
        with patch('xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
            actions = rigid_sensor_actions(self.load(measured()), use_sim_time=False)
        args = actions[1]['arguments']
        params = dict(zip(args[::2], args[1::2]))
        self.assertEqual(params['--frame-id'], 'base_link')
        self.assertEqual(params['--child-frame-id'], 'livox_frame')
        np.testing.assert_allclose([float(params['--' + k]) for k in ('x', 'y', 'z')],
                                   [.18, -.09, .27], atol=1e-12)
        self.assertAlmostEqual(abs(float(params['--yaw'])), math.pi)

    def test_fixed_urdf_chain_is_flat_and_moving_joint_is_rejected(self):
        urdf = '''<robot name="probe"><link name="base_link"/><link name="housing"/>
          <link name="laser"/><joint name="mount" type="fixed"><parent link="base_link"/>
          <child link="housing"/><origin xyz=".2 -.1 .3" rpy="0 0 1.5707963267948966"/></joint>
          <joint name="laser_joint" type="fixed"><parent link="housing"/><child link="laser"/>
          <origin xyz=".01 .02 -.03"/></joint></robot>'''
        with patch('xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
            action, = navigation_static_actions(urdf, 'base_link', ['laser'], use_sim_time=False)
        params = dict(zip(action['arguments'][::2], action['arguments'][1::2]))
        self.assertEqual(params['--frame-id'], 'base_link')
        np.testing.assert_allclose([float(params['--' + k]) for k in ('x', 'y', 'z')],
                                   [.18, -.09, .27], atol=1e-12)
        with self.assertRaisesRegex(ValueError, 'Moving mount'):
            navigation_static_actions(urdf.replace('name="mount" type="fixed"',
                'name="mount" type="continuous"'), 'base_link', ['laser'], use_sim_time=False)
        with self.assertRaisesRegex(ValueError, 'No unique fixed mount'):
            navigation_static_actions(urdf, 'base_link', ['missing'], use_sim_time=False)

    def test_robot_model_can_own_static_tf_without_duplicate_publishers(self):
        with patch('xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
            actions = rigid_sensor_actions(self.load(measured()), use_sim_time=True,
                                           publish_mount_tf=False)
        self.assertEqual(len(actions), 2)
        self.assertTrue(all(a['package'] == 'xxu_bringup' for a in actions))
        self.assertTrue(all(a['parameters'][0]['use_sim_time'] is False for a in actions))

    def test_rigid_sensor_actions_forward_optional_self_filter_geometry(self):
        mount = self.load(measured())
        with tempfile.TemporaryDirectory() as folder, patch(
                'xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
            geometry = Path(folder) / 'geometry.yaml'
            with self.assertRaisesRegex(ValueError, 'self_filter_geometry_file'):
                rigid_sensor_actions(mount, use_sim_time=False, self_filter_geometry_file=geometry)
            with self.assertRaisesRegex(ValueError, 'self_filter_geometry_file'):
                rigid_sensor_actions(mount, use_sim_time=False, self_filter_geometry_file=folder)
            geometry.write_text(yaml.safe_dump({'self_filter': {'frame': mount['frames']['sensor']}}))
            actions = rigid_sensor_actions(
                mount, use_sim_time=False, self_filter_geometry_file=geometry)
            params = node_parameters(actions, 'pointcloud_frame_republisher')
            self.assertEqual(params['self_filter_geometry_file'], str(geometry))
            self.assertEqual(params['target_frame'], mount['frames']['sensor'])
            actions = rigid_sensor_actions(mount, use_sim_time=False)
            self.assertNotIn('self_filter_geometry_file',
                             node_parameters(actions, 'pointcloud_frame_republisher'))

    def test_entry_passes_selected_base_and_sensor_to_existing_adapter(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'other.yaml'
            data = measured()
            data['frames']['base'] = 'other_base'
            path.write_text(yaml.safe_dump(data))
            with patch.object(entry, 'get_package_share_directory', return_value=str(SHARE)):
                context = launch_context(entry, mount_file=str(path))
                actions = entry.setup(context)
            args = dict(actions[-1].launch_arguments)
            self.assertEqual(args['base_frame'], 'other_base')
            self.assertEqual(args['sensor_frame'], data['frames']['sensor'])
            context.launch_configurations['start_base_adapter'] = 'false'
            self.assertEqual(len(entry.setup(context)), 4)

    def test_rigid_entry_forwards_self_filter_geometry(self):
        with tempfile.TemporaryDirectory() as folder, patch(
                'xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
            mount = measured()
            mount_path = Path(folder) / 'mount.yaml'
            mount_path.write_text(yaml.safe_dump(mount))
            geometry = Path(folder) / 'geometry.yaml'
            geometry.write_text(yaml.safe_dump({'self_filter': {'frame': mount['frames']['sensor']}}))
            context = launch_context(entry, mount_file=str(mount_path), start_base_adapter='false',
                                     self_filter_geometry_file=str(geometry))
            actions = entry.setup(context)
            params = node_parameters(actions, 'pointcloud_frame_republisher')
            self.assertEqual(params['self_filter_geometry_file'], str(geometry))

    def test_sentry_entry_forwards_self_filter_geometry_and_scan_radius(self):
        with tempfile.TemporaryDirectory() as folder, patch(
                'xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw), patch.object(
                sentry, 'Node', side_effect=lambda **kw: kw):
            mount = measured()
            mount_path = Path(folder) / 'mount.yaml'
            mount_path.write_text(yaml.safe_dump(mount))
            geometry = Path(folder) / 'geometry.yaml'
            geometry.write_text(yaml.safe_dump({'self_filter': {'frame': mount['frames']['sensor']}}))
            config = Path(folder) / 'lio.yaml'
            config.write_text('{}')
            context = launch_context(sentry, mount_file=str(mount_path),
                                     frontend_config=str(config), backend_config=str(config))
            for overrides, expected_radius in (({}, 0.0), ({
                    'self_filter_geometry_file': str(geometry), 'scan_self_filter_radius': '0.42',
                    'scan_min_height': '0.2', 'scan_max_height': '0.2',
            }, 0.42)):
                with self.subTest(radius=expected_radius):
                    context.launch_configurations.update(overrides)
                    actions = sentry.setup(context)
                    for action in actions[0].event_handler.handle(Shutdown(), context):
                        action.execute(context)
                    cloud_params = node_parameters(actions, 'pointcloud_frame_republisher')
                    self.assertEqual(cloud_params.get('self_filter_geometry_file', ''),
                                     overrides.get('self_filter_geometry_file', ''))
                    scan_params = node_parameters(actions, 'pointcloud_to_laserscan_node')
                    self.assertIsInstance(scan_params['robot_self_filter_radius'], float)
                    self.assertEqual(scan_params['robot_self_filter_radius'], expected_radius)
                    if 'scan_min_height' in overrides:
                        self.assertEqual(scan_params['min_height'], 0.2)
                        self.assertEqual(scan_params['max_height'], 0.2)

    def sentry_actions(self, **overrides):
        mount = measured()
        mount['frames'].update(lidar='sentry_lidar', imu_source='sentry_imu')
        with tempfile.TemporaryDirectory() as folder:
            mount_path = Path(folder) / 'mount.yaml'
            mount_path.write_text(yaml.safe_dump(mount))
            config = Path(folder) / 'lio.yaml'
            config.write_text('{}')
            driver_config = Path(folder) / 'driver.json'
            driver_config.write_text('{}')
            context = launch_context(
                sentry, mount_file=str(mount_path), frontend_config=str(config),
                backend_config=str(config), driver_config=str(driver_config), **overrides)
            with patch.object(sentry, 'get_package_share_directory', return_value=str(SHARE)), \
                    patch.object(sentry, 'Node', side_effect=lambda **kw: kw), \
                    patch.object(sentry, 'ComposableNode', side_effect=lambda **kw: kw), \
                    patch.object(sentry, 'ComposableNodeContainer', side_effect=lambda **kw: kw):
                actions = sentry.setup(context)
            for action in actions[0].event_handler.handle(Shutdown(), context):
                action.execute(context)
        return actions, mount['frames']

    def sentry_bridge_components(self, actions):
        return next(action['composable_node_descriptions'] for action in actions
                    if isinstance(action, dict) and action['name'] == 'sentry_bridge_container')

    def test_sentry_watchdog_joint_states_default_and_override(self):
        for overrides, required in (({}, False), ({'require_joint_states': 'true'}, True)):
            with self.subTest(overrides=overrides):
                actions, _ = self.sentry_actions(**overrides)
                watchdog = next(node for node in self.sentry_bridge_components(actions)
                                if node['name'] == 'cmd_vel_watchdog')
                self.assertIs(watchdog['parameters'][0]['require_joint_states'], required)

    def test_sentry_watchdog_follows_mode_and_explicit_override(self):
        for mode, setting, enabled, require_loc in (
                ('localization', 'auto', True, True),
                ('mapping', 'auto', False, False),
                ('localization', 'false', False, True),
                ('mapping', 'true', True, False)):
            with self.subTest(mode=mode, enable_watchdog=setting):
                actions, _ = self.sentry_actions(mode=mode, enable_watchdog=setting)
                components = self.sentry_bridge_components(actions)
                expected = ['zlio_base_odometry']
                if enabled:
                    expected.append('cmd_vel_watchdog')
                self.assertEqual([node['name'] for node in components], expected)
                odometry = components[0]
                if enabled:
                    self.assertIs(components[1]['parameters'][0]['require_localization'], require_loc)
                bringup, = (action for action in actions
                            if isinstance(action, IncludeLaunchDescription))
                launch_arguments = dict(bringup.launch_arguments)
                if enabled:
                    self.assertIn(('/tf', '/sentry/tf/odometry'), odometry.get('remappings', []))
                    self.assertEqual(launch_arguments['backend_tf_topic'], '/sentry/tf/localization')
                else:
                    odometry_remappings = dict(odometry.get('remappings', []))
                    self.assertEqual(odometry_remappings.get('/tf', '/tf'), '/tf')
                    self.assertEqual(launch_arguments['backend_tf_topic'], '/tf')

    def test_sentry_config_mode_resolves_inherited_prior_map_and_normalizes_sim_time(self):
        mount = measured()
        with tempfile.TemporaryDirectory() as folder:
            mount_path = Path(folder) / 'mount.yaml'
            mount_path.write_text(yaml.safe_dump(mount))
            frontend_cfg = Path(folder) / 'lio.yaml'
            frontend_cfg.write_text('{}')
            base_backend = Path(folder) / 'base.yaml'
            base_backend.write_text(yaml.safe_dump({'prior_map': {'enabled': True}}))
            child_loc = Path(folder) / 'child_loc.yaml'
            child_loc.write_text(yaml.safe_dump({'extends': 'base.yaml'}))
            child_map = Path(folder) / 'child_map.yaml'
            child_map.write_text(yaml.safe_dump({'extends': 'base.yaml', 'prior_map': {'enabled': False}}))

            for backend_path, expected_watchdog in ((child_loc, True), (child_map, False)):
                with self.subTest(backend=backend_path.name):
                    context = launch_context(
                        sentry, mount_file=str(mount_path), frontend_config=str(frontend_cfg),
                        backend_config=str(backend_path), mode='config', use_sim_time='True')
                    with patch.object(sentry, 'get_package_share_directory', return_value=str(SHARE)), \
                            patch.object(sentry, 'Node', side_effect=lambda **kw: kw), \
                            patch.object(sentry, 'ComposableNode', side_effect=lambda **kw: kw), \
                            patch.object(sentry, 'ComposableNodeContainer', side_effect=lambda **kw: kw):
                        actions = sentry.setup(context)
                    for action in actions[0].event_handler.handle(Shutdown(), context):
                        action.execute(context)
                    components = self.sentry_bridge_components(actions)
                    bringup, = (action for action in actions
                                if isinstance(action, IncludeLaunchDescription))
                    launch_arguments = dict(bringup.launch_arguments)
                    self.assertEqual(launch_arguments['use_sim_time'], 'true')
                    if expected_watchdog:
                        self.assertEqual([node['name'] for node in components],
                                         ['zlio_base_odometry', 'cmd_vel_watchdog'])
                        self.assertIs(components[1]['parameters'][0]['use_sim_time'], True)
                        self.assertIs(components[1]['parameters'][0]['require_localization'], True)
                    else:
                        self.assertEqual([node['name'] for node in components], ['zlio_base_odometry'])

    def test_rigid_mount_rejects_reserved_navigation_frames(self):
        for frame in ('map', 'odom', 'base_footprint'):
            for key in ('base', 'sensor', 'lidar', 'imu_source'):
                data = measured()
                data['frames'][key] = frame
                with self.subTest(key=key, frame=frame), self.assertRaisesRegex(ValueError, 'reserved'):
                    self.load(data)

    def test_sentry_driver_uses_distinct_lidar_and_imu_frames(self):
        actions, frames = self.sentry_actions(driver='true')
        params = node_parameters(actions, 'livox_ros_driver2_node')
        self.assertEqual(params['frame_id'], frames['lidar'])
        self.assertEqual(params['imu_frame_id'], frames['imu_source'])

    def test_sentry_entry_rejects_invalid_scan_filters(self):
        with tempfile.TemporaryDirectory() as folder:
            mount_path = Path(folder) / 'mount.yaml'
            mount_path.write_text(yaml.safe_dump(measured()))
            config = Path(folder) / 'lio.yaml'
            config.write_text('{}')
            context = launch_context(sentry, mount_file=str(mount_path),
                                     frontend_config=str(config), backend_config=str(config))
            for radius in ('-0.01', 'nan', 'inf', '-inf', 'invalid'):
                with self.subTest(radius=radius):
                    context.launch_configurations['scan_self_filter_radius'] = radius
                    with self.assertRaisesRegex(ValueError, 'scan_self_filter_radius'):
                        sentry.setup(context)
            context.launch_configurations.update(
                scan_self_filter_radius='0.0', scan_min_height='0.6', scan_max_height='0.5')
            with self.assertRaisesRegex(ValueError, 'scan_min_height.*scan_max_height'):
                sentry.setup(context)
            for bad_height in ('nan', 'inf', '-inf', 'invalid'):
                with self.subTest(scan_min_height=bad_height):
                    context.launch_configurations.update(
                        scan_min_height=bad_height, scan_max_height='0.5')
                    with self.assertRaisesRegex(ValueError, 'scan_min_height'):
                        sentry.setup(context)

    def test_sentry_overhead_clearance_is_opt_in_and_validated(self):
        actions_default, _ = self.sentry_actions()
        names_default = [node['name'] for node in self.sentry_bridge_components(actions_default)]
        self.assertNotIn('overhead_clearance', names_default)

        actions_enabled, frames = self.sentry_actions(
            enable_overhead_clearance='true',
            overhead_base_height='0.175',
            overhead_min_height='0.22',
            overhead_max_height='0.55',
            overhead_detection_radius='1.4',
            overhead_release_hold_sec='2.0',
            overhead_release_travel_distance='0.9',
        )
        overhead = next(
            node for node in self.sentry_bridge_components(actions_enabled)
            if node['name'] == 'overhead_clearance'
        )
        self.assertEqual(overhead['plugin'], 'xxu_bringup::OverheadClearanceNode')
        params = overhead['parameters'][0]
        self.assertEqual(params['extrinsics_source'], 'parameters')
        self.assertEqual(params['base_frame'], 'base_link')
        self.assertEqual(params['sensor_frame'], frames['sensor'])
        self.assertEqual(params['cloud_topic'], '/cloud_deskewed')
        self.assertEqual(params['odom_topic'], '/odom')
        self.assertAlmostEqual(params['base_height_m'], 0.175)
        self.assertAlmostEqual(params['max_overhead_height_m'], 0.55)
        self.assertAlmostEqual(params['release_travel_distance_m'], 0.9)

        with tempfile.TemporaryDirectory() as folder:
            mount_path = Path(folder) / 'mount.yaml'
            mount_path.write_text(yaml.safe_dump(measured()))
            config = Path(folder) / 'lio.yaml'
            config.write_text('{}')
            context = launch_context(
                sentry, mount_file=str(mount_path),
                frontend_config=str(config), backend_config=str(config),
                enable_overhead_clearance='true',
                overhead_min_height='0.6', overhead_max_height='0.5',
            )
            with self.assertRaisesRegex(ValueError, 'overhead_max_height'):
                sentry.setup(context)

    def test_cpp_sensor_and_mount_runtime(self):
        import subprocess
        subprocess.run([sys.executable, str(Path(__file__).with_name('test_cpp_bridge_pipeline.py')),
                        'CppPipelineTest.test_padded_cloud_and_fixed_imu_remain_cpp',
                        'CppPipelineTest.test_odometry_inverse_mount_and_zero_tf_subscription'], check=True)


if __name__ == '__main__':
    unittest.main()
