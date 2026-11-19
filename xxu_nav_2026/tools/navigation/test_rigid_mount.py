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

    def test_entry_passes_selected_base_and_sensor_to_existing_adapter(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'other.yaml'
            data = measured()
            data['frames']['base'] = 'other_base'
            path.write_text(yaml.safe_dump(data))
            context = LaunchContext()
            context.launch_configurations.update(
                mount_file=str(path), use_sim_time='false', publish_mount_tf='true',
                start_base_adapter='true', source_child_frame='imu', odom_frame='odom',
                input_topic='/odometry', output_topic='/odom', publish_deskewed_cloud='false')
            with patch.object(entry, 'get_package_share_directory', return_value=str(SHARE)):
                actions = entry.setup(context)
            args = dict(actions[-1].launch_arguments)
            self.assertEqual(args['base_frame'], 'other_base')
            self.assertEqual(args['sensor_frame'], data['frames']['sensor'])
            context.launch_configurations['start_base_adapter'] = 'false'
            self.assertEqual(len(entry.setup(context)), 4)

    def test_cpp_sensor_and_mount_runtime(self):
        import subprocess
        subprocess.run([sys.executable, str(Path(__file__).with_name('test_cpp_bridge_pipeline.py')),
                        'CppPipelineTest.test_padded_cloud_and_fixed_imu_remain_cpp',
                        'CppPipelineTest.test_odometry_inverse_mount_and_zero_tf_subscription'], check=True)


if __name__ == '__main__':
    unittest.main()
