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
from xxu_bringup.sensor_launch import load_rigid_mount, rigid_sensor_actions


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


cloud = module('rigid_cloud', SHARE / 'scripts/pointcloud_frame_republisher.py')
imu = module('rigid_imu', SHARE / 'scripts/gimbal_imu_compensator.py')
adapter = module('rigid_adapter', SHARE / 'scripts/zlio_base_odometry.py')
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
        self.assertIn('base_footprint', a[0]['arguments'])
        self.assertIn('other_base', b[0]['arguments'])
        self.assertIn('-0.4', b[0]['arguments'])
        self.assertEqual(b[2]['parameters'][0]['target_frame'], 'other_imu_origin')
        self.assertEqual(b[3]['parameters'][0]['sensor_rpy'], second['sensor_from_imu_rpy'])
        self.assertFalse(b[2]['parameters'][0]['use_joint_interpolation'])
        self.assertEqual(b[3]['parameters'][0]['sensor_offset'], [0., 0., 0.])
        self.assertEqual(first['base_to_sensor']['xyz'], [.2, -.1, .3])

    def test_robot_model_can_own_static_tf_without_duplicate_publishers(self):
        with patch('xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
            actions = rigid_sensor_actions(self.load(measured()), use_sim_time=False,
                                           publish_mount_tf=False)
        self.assertEqual(len(actions), 2)
        self.assertTrue(all(a['package'] == 'xxu_bringup' for a in actions))

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

    def test_cloud_rotates_translates_and_preserves_point_times(self):
        message = PointCloud2(height=1, width=1, point_step=24, row_step=24)
        message.header.frame_id = 'livox_frame'
        message.header.stamp.sec = 12
        message.fields = [PointField(name=n, offset=o, datatype=t, count=1)
                          for n, o, t in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
                                          ('timestamp', 16, 8)]]
        message.data = struct.pack('<fff4xd', 1., 2., 3., 1234567890123.)
        transform = TransformStamped()
        transform.header.frame_id = 'lio_base_sensor'
        transform.transform.translation.x = .01
        transform.transform.translation.y = .02
        transform.transform.translation.z = -.03
        transform.transform.rotation.z = math.sin(math.pi / 4)
        transform.transform.rotation.w = math.cos(math.pi / 4)
        output = cloud.PointCloudFrameRepublisher._transform_cloud(message, transform)
        np.testing.assert_allclose(struct.unpack_from('<fff', output.data), [-1.99, 1.02, 2.97], atol=1e-6)
        self.assertEqual(bytes(output.data[12:]), bytes(message.data[12:]))
        self.assertEqual(output.header.stamp, message.header.stamp)
        self.assertEqual(output.header.frame_id, 'lio_base_sensor')
        self.assertEqual(message.header.frame_id, 'livox_frame')

    def test_base_adapter_removes_measured_mount_rotation_and_lever_arm(self):
        # Base at origin, yaw zero; sensor mounted at (.2,-.1,.3), yaw +90deg.
        message = Odometry()
        message.pose.pose.position.x = .2
        message.pose.pose.position.y = -.1
        message.pose.pose.position.z = .3
        message.pose.pose.orientation.z = math.sin(math.pi / 4)
        message.pose.pose.orientation.w = math.cos(math.pi / 4)
        # Base v=(1,0,0), omega=(0,0,1): sensor v in its axes=(.2,-1.1,0).
        message.twist.twist.linear.x = .2
        message.twist.twist.linear.y = -1.1
        message.twist.twist.angular.z = 1.
        inverse = TransformStamped().transform
        inverse.translation.x, inverse.translation.y, inverse.translation.z = .1, .2, -.3
        inverse.rotation.z = -math.sin(math.pi / 4)
        inverse.rotation.w = math.cos(math.pi / 4)
        result = adapter.convert(message, inverse, 'other_base')
        np.testing.assert_allclose(adapter.vector(result.pose.pose.position), [0., 0., 0.], atol=1e-12)
        # Returned twist is expressed in the selected base frame.
        np.testing.assert_allclose(adapter.vector(result.twist.twist.linear), [1., 0., 0.], atol=1e-12)
        self.assertEqual(result.child_frame_id, 'other_base')


class FixedImuTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=194)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_interval_input_preserves_samples_queued_during_callback_stall(self):
        topic = '/rigid_mount_regression/imu_burst'
        node = imu.GimbalImuCompensator(parameter_overrides=[
            Parameter('fixed_sensor', value=True),
            Parameter('imu_interval_samples', value=10),
            Parameter('expected_source_frame', value='imu')],
            cli_args=['--ros-args', '-r', f'imu_in:={topic}'])
        writer = rclpy.create_node('imu_burst_regression_writer')
        publisher = writer.create_publisher(
            Imu, topic, QoSProfile(depth=4000, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.publisher = Mock()
        try:
            deadline = time.monotonic() + 5
            while not publisher.get_subscription_count() and time.monotonic() < deadline:
                rclpy.spin_once(writer, timeout_sec=.01)
            self.assertGreater(publisher.get_subscription_count(), 0)
            # Model a blocked executor while 200 ms of real stamped samples arrive.
            # Never interpolate, alter timestamps, or relax the averaging rule.
            for i in range(200):
                message = Imu()
                message.header.frame_id = 'imu'
                message.header.stamp.sec = 10
                message.header.stamp.nanosec = i * 1000000
                message.angular_velocity.x = float(i)
                publisher.publish(message)
            deadline = time.monotonic() + 3
            while node.publisher.publish.call_count < 20 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
            outputs = [call.args[0] for call in node.publisher.publish.call_args_list]
            self.assertEqual(len(outputs), 20)
            self.assertEqual([m.header.stamp.nanosec for m in outputs],
                             [4500000 + i * 10000000 for i in range(20)])
            self.assertEqual([m.angular_velocity.x for m in outputs],
                             [4.5 + i * 10 for i in range(20)])
        finally:
            node.destroy_node()
            writer.destroy_node()

    def test_interval_mean_midpoint_covariance_and_gap_reset(self):
        node = imu.GimbalImuCompensator(parameter_overrides=[
            Parameter('fixed_sensor', value=True),
            Parameter('imu_interval_samples', value=10),
            Parameter('expected_source_frame', value='imu')])
        try:
            node._imu_callback = Mock()

            def feed(start, count, frame='imu'):
                for i in range(count):
                    message = Imu()
                    message.header.frame_id = frame
                    message.header.stamp.sec = 10
                    message.header.stamp.nanosec = (start + i) * 1000000
                    message.angular_velocity.x = float(i)
                    message.linear_acceleration.z = float(2 * i)
                    message.angular_velocity_covariance = [10.] * 9
                    message.linear_acceleration_covariance = [-1.] + [0.] * 8
                    node._input_imu_callback(message)

            feed(0, 9)
            node._imu_callback.assert_not_called()
            feed(20, 9)  # A gap must discard the first incomplete interval.
            node._imu_callback.assert_not_called()
            feed(29, 1)
            output = node._imu_callback.call_args.args[0]
            self.assertEqual(node._imu_callback.call_count, 1)
            self.assertEqual(output.header.stamp.nanosec, 24500000)
            self.assertAlmostEqual(output.angular_velocity.x, 3.6)
            self.assertAlmostEqual(output.linear_acceleration.z, 7.2)
            self.assertEqual(list(output.angular_velocity_covariance), [1.] * 9)
            self.assertEqual(output.linear_acceleration_covariance[0], -1.)
            feed(30, 5)
            feed(35, 1, 'wrong')
            self.assertEqual(node._interval_rows, [])
            # Existing validator receives the wrong frame and rejects it.
            self.assertEqual(node._imu_callback.call_args.args[0].header.frame_id, 'wrong')
        finally:
            node.destroy_node()

    def test_default_imu_input_is_forwarded_without_resampling(self):
        node = imu.GimbalImuCompensator()
        try:
            node._imu_callback = Mock()
            message = Imu()
            node._input_imu_callback(message)
            node._imu_callback.assert_called_once_with(message)
        finally:
            node.destroy_node()

    def test_fixed_imu_needs_no_joint_state_and_rotates_vectors_covariance(self):
        mount = measured()
        with patch('xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
            params = rigid_sensor_actions(mount, use_sim_time=False)[-1]['parameters'][0]
        node = imu.GimbalImuCompensator(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        try:
            self.assertFalse(any(s.topic_name.endswith('/joint_states') for s in node.subscriptions))
            node.publisher = Mock()
            message = Imu()
            message.header.frame_id = mount['frames']['imu_source']
            message.header.stamp.sec = 10
            message.angular_velocity.x = 1.
            message.linear_acceleration.x = 1.
            message.angular_velocity_covariance = [1., 0., 0., 0., 2., 0., 0., 0., 3.]
            message.linear_acceleration_covariance = list(message.angular_velocity_covariance)
            node._imu_callback(message)
            output = node.publisher.publish.call_args.args[0]
            self.assertAlmostEqual(output.angular_velocity.y, 1.)
            self.assertAlmostEqual(output.angular_velocity.x, 0.)
            self.assertAlmostEqual(output.linear_acceleration.y, 9.81)
            np.testing.assert_allclose(output.angular_velocity_covariance, [2., 0., 0., 0., 1., 0., 0., 0., 3.], atol=1e-12)
            self.assertAlmostEqual(output.linear_acceleration_covariance[0], 2 * 9.81 ** 2)
            self.assertEqual(output.header.stamp, message.header.stamp)
            self.assertEqual(output.header.frame_id, mount['frames']['sensor'])
            node.publisher.reset_mock()
            message.header.frame_id = 'wrong_imu'
            node._imu_callback(message)
            node.publisher.publish.assert_not_called()
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
