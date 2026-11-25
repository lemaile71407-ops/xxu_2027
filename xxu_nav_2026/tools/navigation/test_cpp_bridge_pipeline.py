#!/usr/bin/env python3
"""Isolated synthetic C++ pipeline tests. Never opens real hardware or sends goals."""
import json
import math
import os
from copy import deepcopy
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest
import yaml

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from composition_interfaces.srv import ListNodes


class CppHarness(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.domain = int(os.environ.get('XXU_TEST_ROS_DOMAIN_ID', '183'))
        rclpy.init(domain_id=cls.domain)

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.node = rclpy.create_node('cpp_pipeline_probe')
        self.processes = []
        self.logs = tempfile.TemporaryDirectory(prefix='xxu_cpp_probe_')

    def tearDown(self):
        errors = []
        for process, log in reversed(self.processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                code = process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                code = process.wait()
            log.flush()
            if code not in (0, -signal.SIGINT):
                errors.append(Path(log.name).read_text())
            log.close()
        self.node.destroy_node()
        self.logs.cleanup()
        if errors:
            self.fail('\n'.join(errors))

    def start(self, executable, params=None, remaps=(), package='xxu_bringup'):
        path = Path(get_package_prefix(package)) / 'lib' / package / executable
        self.assertTrue(path.is_file(), str(path))
        args = [str(path), '--ros-args']
        for name, value in (params or {}).items():
            args += ['-p', f'{name}:={json.dumps(value)}']
        for name, target in remaps:
            args += ['-r', f'{name}:={target}']
        log = open(Path(self.logs.name) / f'{len(self.processes)}.log', 'w+')
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain), ROS_LOCALHOST_ONLY='1')
        process = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT)
        self.processes.append((process, log))
        return process

    def spin(self, duration, publish=None):
        deadline = time.monotonic() + duration
        next_publish = 0.
        while time.monotonic() < deadline:
            if publish and time.monotonic() >= next_publish:
                publish()
                next_publish = time.monotonic() + .02
            rclpy.spin_once(self.node, timeout_sec=.002)
        for process, log in self.processes:
            if process.poll() is not None:
                log.flush()
                self.fail(Path(log.name).read_text())

    def received(self, kind, topic, qos=qos_profile_sensor_data):
        messages = []
        self.node.create_subscription(kind, topic, messages.append, qos)
        return messages

    def static(self, parent, child, xyz=(0., 0., 0.), yaw=0.):
        pub = self.node.create_publisher(TFMessage, '/tf_static',
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        pub.publish(TFMessage(transforms=[transform]))
        return pub

    def assert_no_global_tf_subscribers(self):
        self.spin(.1)
        self.assertEqual(self.node.get_subscriptions_info_by_topic('/tf'), [])

class CppPipelineTest(CppHarness):
    def test_clock_decimation_preserves_stamps_and_drops_rewinds(self):
        self.start('clock_rate_limiter', {'publish_rate': 200.,
            'input_topic': '/probe/clock_raw', 'output_topic': '/probe/clock'},
            package='xxu_sim_bringup')
        received = self.received(Clock, '/probe/clock')
        raw = self.node.create_publisher(Clock, '/probe/clock_raw', 10)
        self.spin(.8)
        self.assertEqual(self.node.get_subscriptions_info_by_topic('/clock'), [])
        stamps = lambda: [m.clock.sec * 10**9 + m.clock.nanosec for m in received]
        sent = set()

        def feed(value):
            m = Clock(); m.clock.sec, m.clock.nanosec = divmod(value, 10**9)
            sent.add(value); raw.publish(m); self.spin(.01)

        # Slow wall-time delivery deliberately differs from the source's 1 ms steps.
        # Clock decimation must depend on source time, not on the executor's clock.
        for index in range(31):
            feed(100 * 10**9 + index * 10**6)
        self.assertEqual(stamps(), [100 * 10**9 + index * 5 * 10**6 for index in range(7)])
        count = len(received)
        for _ in range(5):
            feed(100 * 10**9 + 30 * 10**6)
        self.spin(.05)
        self.assertEqual(len(received), count, 'paused source must not replay ticks')
        feed(0)
        feed(5 * 10**6)
        feed(100 * 10**9 + 29 * 10**6)
        self.assertEqual(len(received), count, 'backward samples must not reset the input watermark')
        feed(100 * 10**9 + 34 * 10**6)
        self.assertEqual(len(received), count, 'backward samples must not change the 5 ms phase')
        feed(100 * 10**9 + 35 * 10**6)
        self.assertEqual(stamps()[-1], 100 * 10**9 + 35 * 10**6)
        count = len(received)
        feed(108 * 10**9)
        self.assertEqual(len(received), count + 1, 'large jumps must not replay missed slots')
        self.spin(.05)
        self.assertEqual(len(received), count + 1)
        self.assertTrue(set(stamps()) <= sent)
        self.assertEqual(stamps(), sorted(set(stamps())))

    def test_tf_rate_with_31hz_and_200hz_input_and_frozen_source(self):
        self.start('zlio_base_odometry', {
            'extrinsics_source': 'parameters', 'base_from_sensor_xyz': [0., 0., 0.],
            'base_from_sensor_rpy': [0., 0., 0.], 'tf_publish_rate': 30.,
            'input_topic': '/probe/raw', 'output_topic': '/probe/base'},
            [('/tf', '/probe/tf')])
        received = self.received(TFMessage, '/probe/tf')
        raw = self.node.create_publisher(Odometry, '/probe/raw', qos_profile_sensor_data)
        message = Odometry(); message.header.frame_id = 'odom'; message.child_frame_id = 'imu'
        message.pose.pose.orientation.w = 1.
        self.spin(.8)
        results = []
        for hz in (31., 200.):
            received.clear()
            started = time.monotonic(); deadline = started + 2.; due = started
            while time.monotonic() < deadline:
                if time.monotonic() >= due:
                    message.header.stamp = self.node.get_clock().now().to_msg()
                    raw.publish(message); due += 1. / hz
                rclpy.spin_once(self.node, timeout_sec=.001)
            rate = len(received) / (time.monotonic() - started)
            results.append({'input_hz': hz, 'tf_hz': rate})
        print(json.dumps({'tf_rate_probe': results}), flush=True)
        for result in results:
            self.assertGreaterEqual(result['tf_hz'], 20.)
            self.assertLessEqual(result['tf_hz'], 32.)
        self.spin(.1)
        count = len(received)
        self.spin(.2, lambda: raw.publish(message))
        self.assertEqual(len(received), count, 'a repeated source stamp must not renew TF')
        self.spin(.2)
        self.assertEqual(len(received), count, 'stopped input must not be replayed by timer')
        self.assertTrue(all(t.header.frame_id == 'odom' and t.child_frame_id == 'base_link'
                            for m in received for t in m.transforms))

    def test_public_tf_contains_only_core_edges_and_caps_200hz_sources(self):
        self.check_public_tf_relay(50.)

    def test_public_tf_keeps_20hz_and_flushes_one_tail(self):
        self.check_public_tf_relay(20.)

    def check_public_tf_relay(self, configured_rate):
        self.start('cmd_vel_watchdog', {
            'input_topic': '/probe/cmd', 'output_topic': '/probe/safe',
            'require_odom': False, 'require_scan': False, 'require_joint_states': False,
            'require_tf': False, 'tf_publish_rate': configured_rate})
        received = []

        def observe(message):
            now = time.monotonic()
            received.extend((now, t) for t in message.transforms)

        self.node.create_subscription(TFMessage, '/tf', observe, qos_profile_sensor_data)
        source = self.node.create_publisher(TFMessage, '/sentry/tf/odometry', qos_profile_sensor_data)
        expected = {('map', 'odom'), ('odom', 'base_link')}
        edges = [('map', 'odom'), ('odom', 'base_link'),
                 ('base_link', 'wheel_fl'), ('base_link', 'base_footprint')]
        sent = {}
        delay_limit = 1. / configured_rate + .01
        self.spin(.8)

        def publish():
            stamp = self.node.get_clock().now().to_msg()
            transforms = []
            for parent, child in edges:
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = parent, child
                transform.header.stamp = stamp
                transform.transform.rotation.w = 1.
                transforms.append(transform)
            sent_at = time.monotonic()
            source.publish(TFMessage(transforms=transforms))
            sent[(stamp.sec, stamp.nanosec)] = sent_at
            return stamp, sent_at

        started = due = time.monotonic()
        while time.monotonic() - started < 3.:
            if time.monotonic() >= due:
                publish()
                due += .005
            rclpy.spin_once(self.node, timeout_sec=.001)
        duration = time.monotonic() - started
        counts = {}
        for _, transform in received:
            edge = (transform.header.frame_id, transform.child_frame_id)
            counts[edge] = counts.get(edge, 0) + 1
        self.assertEqual(set(counts), expected)
        rates = {str(edge): count / duration for edge, count in counts.items()}
        print(json.dumps({'configured_tf_rate': configured_rate, 'public_tf_rates': rates}), flush=True)
        for rate in rates.values():
            self.assertGreaterEqual(rate, configured_rate - .5)
            self.assertLessEqual(rate, configured_rate + 1.)
        stopped_at = len(received)
        self.spin(.15)
        max_delay = max(when - sent[(t.header.stamp.sec, t.header.stamp.nanosec)] for when, t in received)
        self.assertLess(max_delay, delay_limit)
        for edge in expected:
            self.assertLessEqual(sum((t.header.frame_id, t.child_frame_id) == edge
                                     for _, t in received[stopped_at:]), 1)
        received.clear()

        first_stamp, _ = publish()
        self.spin(.004)
        _, queued_at = publish()
        self.spin(.004)
        tail_stamp, _ = publish()
        self.spin(.12)
        tail_delays = []
        for edge in expected:
            frames = [(when, t.header.stamp) for when, t in received
                      if (t.header.frame_id, t.child_frame_id) == edge]
            self.assertEqual([stamp for _, stamp in frames], [first_stamp, tail_stamp])
            tail_delays.append(frames[-1][0] - queued_at)
        self.assertLess(max(tail_delays), delay_limit)
        print(json.dumps({'configured_tf_rate': configured_rate, 'max_tf_delay_ms': max_delay * 1e3,
                          'tail_delay_ms': max(tail_delays) * 1e3}), flush=True)
        self.spin(.15)
        self.assertEqual(len(received), 2 * len(expected))

    def test_odometry_accepts_only_static_mount_then_releases_subscription(self):
        self.start('zlio_base_odometry', {'input_topic': '/probe/raw', 'output_topic': '/probe/base'},
            [('/tf', '/sentry/tf/odometry')])
        output = self.received(Odometry, '/probe/base')
        raw = self.node.create_publisher(Odometry, '/probe/raw', qos_profile_sensor_data)
        dynamic = self.node.create_publisher(TFMessage, '/tf', 10)
        message = Odometry(); message.header.frame_id = 'odom'; message.child_frame_id = 'imu'
        message.pose.pose.orientation.w = 1.
        transform = TransformStamped(); transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'lio_base_sensor'; transform.transform.rotation.w = 1.
        transform.transform.translation.x = 9.
        def feed():
            message.header.stamp = self.node.get_clock().now().to_msg(); raw.publish(message)
            transform.header.stamp = message.header.stamp; dynamic.publish(TFMessage(transforms=[transform]))
        self.spin(.9, feed)
        self.assertEqual(output, [], 'a dynamic transform must never be cached as rigid extrinsics')
        static = self.static('base_link', 'lio_base_sensor', (.2, 0., 0.))
        self.spin(.3, feed)
        self.assertTrue(output)
        self.assertAlmostEqual(output[-1].pose.pose.position.x, -.2)
        for info in self.node.get_subscriptions_info_by_topic('/tf_static'):
            self.assertNotEqual(info.node_name, 'zlio_base_odometry')
        self.assert_no_global_tf_subscribers()

    def test_real_entry_loads_two_cpp_components_without_hardware(self):
        workspace = Path(__file__).resolve().parents[2]
        mount = yaml.safe_load((workspace / 'src/xxu_bringup/config/rigid_mount.pending.yaml').read_text())
        mount.update(calibration_confirmed=True, imu_acceleration_scale=1., sensor_from_imu_rpy=[0., 0., 0.])
        for key in ('base_to_sensor', 'sensor_to_lidar'):
            mount[key] = {'xyz': [0., 0., .1], 'rpy': [0., 0., 0.]}
        path = Path(self.logs.name) / 'mount.yaml'; path.write_text(yaml.safe_dump(mount))
        log = open(Path(self.logs.name) / 'composition.log', 'w+')
        process = subprocess.Popen(['ros2', 'launch', 'xxu_bringup', 'sentry_localization.launch.py',
            f'mount_file:={path}', f'frontend_config:={workspace}/src/z-localization/config/lio.yaml',
            f'backend_config:={workspace}/src/z-localization/config/backend.smoke.yaml', 'mode:=mapping',
            'enable_watchdog:=true', 'driver:=false', 'use_sim_time:=true'],
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
            env=dict(os.environ, ROS_DOMAIN_ID=str(self.domain), ROS_LOCALHOST_ONLY='1'))
        self.processes.append((process, log))
        statics = self.received(TFMessage, '/tf_static', QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        client = self.node.create_client(ListNodes, '/sentry_bridge_container/_container/list_nodes')
        self.assertTrue(client.wait_for_service(timeout_sec=10))
        expected = {'/zlio_base_odometry', '/cmd_vel_watchdog'}
        deadline = time.monotonic() + 10
        found = set()
        while found != expected and time.monotonic() < deadline:
            future = client.call_async(ListNodes.Request())
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=.01)
            if future.done(): found = set(future.result().full_node_names)
            if found != expected: self.spin(.05)
        self.assertEqual(found, expected)
        self.spin(.4)
        edges = {(t.header.frame_id, t.child_frame_id) for m in statics for t in m.transforms}
        self.assertEqual(edges, {('base_link', mount['frames']['sensor']),
                                 ('base_link', mount['frames']['lidar'])})
        clock_subscribers = {info.node_name for info in self.node.get_subscriptions_info_by_topic('/clock')}
        self.assertIn('cmd_vel_watchdog', clock_subscribers)
        self.assertTrue(clock_subscribers.isdisjoint({
            'zlio_base_odometry', 'pointcloud_to_laserscan',
            f"mount_{mount['frames']['sensor']}", f"mount_{mount['frames']['lidar']}",
        }))
        self.assertFalse(any('robot_state_publisher' == n for n, _ in self.node.get_node_names_and_namespaces()))
        self.assert_no_global_tf_subscribers()

    def test_adapter_throughput_at_200hz_imu_and_10hz_cloud(self):
        """Conversion/scan throughput only; the estimator and real LiDAR are excluded."""
        self.start('pointcloud_frame_republisher', {'target_frame': 'base_link',
            'expected_source_frame': 'probe_lidar'}, [('points_in', '/probe/points'), ('points_out', '/probe/cloud')])
        self.start('gimbal_imu_compensator', {'fixed_sensor': True, 'sensor_offset': [0., 0., 0.],
            'sensor_rpy': [0., 0., 0.]}, [('imu_in', '/probe/imu'), ('imu_out', '/probe/imu_out')])
        self.start('pointcloud_to_laserscan_node', {'target_frame': 'base_link', 'min_height': -.5,
            'max_height': 5.}, [('cloud_in', '/probe/cloud'), ('scan', '/probe/scan')], package='pointcloud_to_laserscan')
        static = self.static('base_link', 'probe_lidar')
        received_imu = self.received(Imu, '/probe/imu_out',
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT))
        received_cloud = self.received(PointCloud2, '/probe/cloud')
        received_scan = self.received(LaserScan, '/probe/scan')
        imus = self.node.create_publisher(Imu, '/probe/imu', qos_profile_sensor_data)
        clouds = self.node.create_publisher(PointCloud2, '/probe/points', qos_profile_sensor_data)
        self.spin(.8)
        cloud = PointCloud2(height=1, width=12000, point_step=16, row_step=12000*16)
        cloud.header.frame_id = 'probe_lidar'
        cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1) for i,n in enumerate('xyz')]
        cloud.data = struct.pack('<ffff', 1., 2., .2, 0.) * cloud.width
        imu = Imu(); imu.linear_acceleration.z = 9.81
        begin = time.monotonic(); end = begin + 3.; next_imu = next_cloud = begin
        sent_imu = sent_cloud = 0
        while time.monotonic() < end:
            now = time.monotonic(); stamp = self.node.get_clock().now().to_msg()
            if now >= next_imu:
                imu.header.stamp = stamp; imus.publish(imu); next_imu += .005; sent_imu += 1
            if now >= next_cloud:
                cloud.header.stamp = stamp; clouds.publish(cloud); next_cloud += .1; sent_cloud += 1
            rclpy.spin_once(self.node, timeout_sec=.001)
        self.spin(.15)
        rates = {'scope': 'synthetic adapter/scan only, no estimator or hardware',
            'seconds': 3., 'sent_imu': sent_imu, 'received_imu': len(received_imu),
            'sent_cloud': sent_cloud, 'received_cloud': len(received_cloud), 'received_scan': len(received_scan)}
        print(json.dumps(rates, sort_keys=True), flush=True)
        self.assertGreaterEqual(sent_imu, 570)
        self.assertGreaterEqual(len(received_imu), .9 * sent_imu)
        self.assertGreaterEqual(len(received_cloud), .9 * sent_cloud)
        self.assertGreaterEqual(len(received_scan), .9 * sent_cloud)

    def test_interval_mean_retains_burst_samples_and_rejects_gaps(self):
        self.start('gimbal_imu_compensator', {'fixed_sensor': True, 'imu_interval_samples': 10,
            'sensor_rpy': [0., 0., 0.], 'sensor_offset': [0., 0., 0.], 'expected_source_frame': 'imu'},
            [('imu_in', '/probe/burst'), ('imu_out', '/probe/mean')])
        qos = QoSProfile(depth=4000, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub = self.node.create_publisher(Imu, '/probe/burst', qos)
        received = self.received(Imu, '/probe/mean', qos)
        self.spin(.8)
        self.assertGreater(pub.get_subscription_count(), 0)
        for i in range(200):
            message = Imu(); message.header.frame_id = 'imu'; message.header.stamp.sec = 1
            message.header.stamp.nanosec = i * 1000000; message.angular_velocity.x = float(i)
            message.angular_velocity_covariance = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            pub.publish(message)
        self.spin(.4)
        self.assertEqual(len(received), 20)
        self.assertAlmostEqual(received[0].angular_velocity.x, 4.5)
        self.assertEqual(received[0].header.stamp.nanosec, 4500000)
        self.assertAlmostEqual(received[0].angular_velocity_covariance[0], .1)
        # A missing millisecond clears a partial interval; no samples are fabricated.
        for i in (300, 301, 302, 304, 305, 306, 307, 308, 309, 310):
            message = Imu(); message.header.frame_id = 'imu'; message.header.stamp.sec = 1
            message.header.stamp.nanosec = i * 1000000; pub.publish(message)
        self.spin(.2); self.assertEqual(len(received), 20)
        for i in (311, 312, 313):
            message.header.stamp.nanosec = i * 1000000; pub.publish(message)
        self.spin(.2); self.assertEqual(len(received), 21)

    def test_dynamic_gimbal_imu_lever_arm_compensation(self):
        self.start('gimbal_imu_compensator', {'fixed_sensor': False,
            'sensor_rpy': [0., 0., 0.], 'sensor_offset': [1., 0., 0.]},
            [('imu_in', '/probe/moving_imu'), ('imu_out', '/probe/base_imu'), ('joint_states', '/probe/joints')])
        received = self.received(Imu, '/probe/base_imu')
        imus = self.node.create_publisher(Imu, '/probe/moving_imu', qos_profile_sensor_data)
        joints = self.node.create_publisher(JointState, '/probe/joints', qos_profile_sensor_data)
        self.spin(.8)
        joint = JointState(); joint.name = ['gimbal_joint']; joint.position = [0.]; joint.velocity = [1.]
        joint.header.stamp.sec = 1; joints.publish(joint)
        joint.header.stamp.nanosec = 20000000; joints.publish(joint)
        self.spin(.05)
        imu = Imu(); imu.header.stamp.sec = 1; imu.header.stamp.nanosec = 10000000
        imu.angular_velocity.z = 2.; imu.linear_acceleration.x = -4.; imu.linear_acceleration.z = 9.81
        imus.publish(imu); self.spin(.1)
        self.assertEqual(len(received), 1)
        self.assertAlmostEqual(received[-1].angular_velocity.z, 1.)
        self.assertAlmostEqual(received[-1].linear_acceleration.x, 0.)
        self.assertAlmostEqual(received[-1].linear_acceleration.z, 9.81)
        imu.header.stamp.nanosec = 100000000; imus.publish(imu); self.spin(.05)
        self.assertEqual(len(received), 1, 'no extrapolation past the joint history')

    def test_odometry_inverse_mount_and_zero_tf_subscription(self):
        self.start('zlio_base_odometry', {
            'extrinsics_source': 'parameters', 'base_from_sensor_xyz': [.2, -.1, .3],
            'base_from_sensor_rpy': [0., 0., math.pi / 2],
            'input_topic': '/probe/raw', 'output_topic': '/probe/base'}, [('/tf', '/sentry/tf/odometry')])
        received = self.received(Odometry, '/probe/base')
        publisher = self.node.create_publisher(Odometry, '/probe/raw', qos_profile_sensor_data)
        message = Odometry()
        message.header.frame_id = 'odom'; message.child_frame_id = 'imu'
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = .2, -.1, .3
        message.pose.pose.orientation.z = message.pose.pose.orientation.w = math.sqrt(.5)
        message.twist.twist.linear.x, message.twist.twist.linear.y = .2, -1.1
        message.twist.twist.angular.z = 1.
        def feed():
            message.header.stamp = self.node.get_clock().now().to_msg(); publisher.publish(message)
        self.spin(1.2, feed)
        self.assertTrue(received)
        output = received[-1]
        self.assertEqual(output.child_frame_id, 'base_link')
        for axis in ('x', 'y', 'z'):
            self.assertAlmostEqual(getattr(output.pose.pose.position, axis), 0., places=10)
        self.assertAlmostEqual(output.twist.twist.linear.x, 1., places=10)
        self.assertAlmostEqual(output.twist.twist.linear.y, 0., places=10)
        count = len(received); message.child_frame_id = 'wrong'
        self.spin(.15, feed); self.assertEqual(len(received), count)
        clock_subscribers = {info.node_name for info in self.node.get_subscriptions_info_by_topic('/clock')}
        self.assertNotIn('zlio_base_odometry', clock_subscribers)
        self.assert_no_global_tf_subscribers()

    def test_watchdog_source_stamp_and_paused_clock_fail_closed(self):
        self.start('cmd_vel_watchdog', {
            'input_topic': '/probe/cmd', 'output_topic': '/probe/safe',
            'require_odom': False, 'require_scan': False, 'require_joint_states': False,
            'require_tf': False, 'use_sim_time': True, 'publish_rate': 100.})
        output = self.received(TwistStamped, '/probe/safe')
        pub = self.node.create_publisher(TwistStamped, '/probe/cmd', qos_profile_sensor_data)
        clock = self.node.create_publisher(Clock, '/clock', 10)
        now = Clock(); now.clock.sec = 100
        self.spin(1., lambda: clock.publish(now))
        command = TwistStamped(); command.header.stamp.sec = 100; command.twist.linear.x = 1.
        pub.publish(command); self.spin(.08)
        self.assertTrue(any(m.twist.linear.x == 1. for m in output[-5:]))
        now.clock.nanosec = 200000000; clock.publish(now); self.spin(.05)
        self.assertEqual(output[-1].twist.linear.x, 1.)
        now.clock.nanosec = 310000000; clock.publish(now); self.spin(.05)
        self.assertEqual(output[-1].twist.linear.x, 0.)
        now.clock.sec = 101; now.clock.nanosec = 0; clock.publish(now); self.spin(.03)
        command.header.stamp.sec = 101; command.header.stamp.nanosec = 20_000_000
        pub.publish(command); self.spin(.07)
        self.assertEqual(output[-1].twist.linear.x, 1., 'small cross-process clock lead should be tolerated')
        command.header.frame_id = 'map'; command.header.stamp.nanosec = 0
        pub.publish(command); self.spin(.05)
        self.assertEqual(output[-1].twist.linear.x, 0., 'wrong command frame_id must close gate')
        command.header.frame_id = 'base_link'; pub.publish(command); self.spin(.05)
        self.assertEqual(output[-1].twist.linear.x, 1.)
        self.spin(.35)
        self.assertEqual(output[-1].twist.linear.x, 0.)
        command.twist.linear.x = float('nan'); pub.publish(command); self.spin(.05)
        self.assertEqual(output[-1].twist.linear.x, 0.)
        self.assert_no_global_tf_subscribers()

    def test_watchdog_drops_reordered_dynamic_tf(self):
        self.start('cmd_vel_watchdog', {
            'input_topic': '/probe/cmd', 'output_topic': '/probe/safe', 'require_odom': False,
            'require_scan': False, 'require_joint_states': False, 'use_sim_time': True,
            'publish_rate': 100., 'tf_timeout': .5,
        })
        safe = self.received(TwistStamped, '/probe/safe')
        relayed = self.received(TFMessage, '/tf')
        command_pub = self.node.create_publisher(TwistStamped, '/probe/cmd', qos_profile_sensor_data)
        source = self.node.create_publisher(TFMessage, '/sentry/tf/odometry', qos_profile_sensor_data)
        clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        clock = Clock()
        clock.clock.sec = 100
        self.spin(.8, lambda: clock_pub.publish(clock))

        def send(sec, nanosec):
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
            transform.header.stamp.sec, transform.header.stamp.nanosec = sec, nanosec
            transform.transform.rotation.w = 1.
            source.publish(TFMessage(transforms=[transform]))

        def command():
            message = TwistStamped()
            message.header.stamp = clock.clock
            message.twist.linear.x = .25
            command_pub.publish(message)

        send(100, 0)
        self.spin(.06, command)
        self.assertEqual(safe[-1].twist.linear.x, .25)
        send(99, 250_000_000)
        self.spin(.04, command)
        self.assertEqual(safe[-1].twist.linear.x, .25)

        def advance():
            clock.clock.nanosec += 20_000_000
            clock_pub.publish(clock)
            send(clock.clock.sec, clock.clock.nanosec)
            command()

        self.spin(.68, advance)
        self.assertEqual(safe[-1].twist.linear.x, .25)
        stamps = [Time.from_msg(t.header.stamp).nanoseconds for msg in relayed for t in msg.transforms]
        self.assertTrue(all(stamp >= 100_000_000_000 for stamp in stamps))
        self.assertGreater(stamps[-1], 100_500_000_000)

        def stale_tf_and_command():
            send(99, 250_000_000)
            command()

        self.spin(.56, stale_tf_and_command)
        self.assertEqual(safe[-1].twist.linear.x, 0., 'reordered TF must not refresh the gate')
        self.assertTrue(all(t.header.stamp.sec >= 100 for msg in relayed for t in msg.transforms))

        # A real ROS /clock rewind (e.g. bag loop or sim restart) resets dynamic TF state cleanly.
        clock.clock.sec = 10
        clock.clock.nanosec = 0
        self.spin(.06, lambda: clock_pub.publish(clock))

        def post_rewind():
            clock.clock.nanosec += 20_000_000
            clock_pub.publish(clock)
            send(clock.clock.sec, clock.clock.nanosec)
            command()

        self.spin(.15, post_rewind)
        self.assertEqual(safe[-1].twist.linear.x, .25, 'gate must recover after ROS clock rewind')

        # Repeated identical TF stamps must not keep the gate open indefinitely.
        frozen_sec, frozen_nsec = clock.clock.sec, clock.clock.nanosec

        def frozen_tf():
            clock.clock.nanosec += 20_000_000
            clock_pub.publish(clock)
            send(frozen_sec, frozen_nsec)
            command()

        self.spin(.56, frozen_tf)
        self.assertEqual(safe[-1].twist.linear.x, 0., 'frozen TF timestamp must time out the gate')

    def test_watchdog_tf_stream_loss_and_localization_recovery(self):
        self.start('cmd_vel_watchdog', {
            'input_topic': '/probe/cmd', 'output_topic': '/probe/safe',
            'odom_topic': '/probe/odom', 'scan_topic': '/probe/scan',
            'require_joint_states': False, 'require_localization': True,
            'required_tf_pairs': ['odom->base_link'], 'tf_source_frame': 'probe_sensor',
            'localization_state_topic': '/probe/state', 'localization_pose_topic': '/probe/pose',
            'publish_rate': 100.})
        output = self.received(TwistStamped, '/probe/safe')
        ready_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ready = self.received(Bool, '/cmd_vel_watchdog/localization_ready', ready_qos)
        tf_ready = self.received(Bool, '/cmd_vel_watchdog/tf_ready', ready_qos)
        pubs = {name: self.node.create_publisher(kind, '/probe/' + name, qos_profile_sensor_data)
                for name, kind in [('cmd', TwistStamped), ('odom', Odometry), ('scan', LaserScan), ('pose', Odometry)]}
        state = self.node.create_publisher(String, '/probe/state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        odom_tf = self.node.create_publisher(TFMessage, '/sentry/tf/odometry', 10)
        map_tf = self.node.create_publisher(TFMessage, '/sentry/tf/localization', 10)
        static = self.static('base_link', 'probe_sensor')
        trusted, stream, commands, generation = True, True, True, 2
        def feed():
            stamp = self.node.get_clock().now().to_msg()
            ns = stamp.sec * 1000000000 + stamp.nanosec
            odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'; odom.pose.pose.orientation.w = 1.
            pubs['odom'].publish(odom)
            odom.header.frame_id = 'map'; pubs['pose'].publish(odom)
            scan = LaserScan(); scan.header.stamp = stamp; pubs['scan'].publish(scan)
            label = 'NORMAL' if trusted else 'SUSPECT'
            state.publish(String(data=f'status={label};pose_trusted={str(trusted).lower()};tf_frozen={str(not trusted).lower()};epoch=0;generation={generation};pose_revision=1;stamp_ns={ns}'))
            tf = TransformStamped(); tf.header.stamp = stamp; tf.header.frame_id = 'map'; tf.child_frame_id = 'odom'; tf.transform.rotation.w = 1.
            map_tf.publish(TFMessage(transforms=[tf]))
            tf.header.frame_id = 'odom'; tf.child_frame_id = 'base_link'
            if stream: odom_tf.publish(TFMessage(transforms=[tf]))
            if commands:
                cmd = TwistStamped(); cmd.header.stamp = stamp; cmd.twist.linear.x = 1.; pubs['cmd'].publish(cmd)
        self.spin(2.3, feed)
        self.assertTrue(any(m.twist.linear.x == 1. for m in output[-10:]))
        ready.clear(); tf_ready.clear()
        self.spin(.6, feed)
        for heartbeat in (ready, tf_ready):
            self.assertTrue(3 <= len(heartbeat) <= 5, f'Unexpected readiness rate: {len(heartbeat)} in 0.6 s')
            self.assertTrue(all(message.data for message in heartbeat))
        stream = False; self.spin(.65, feed); self.assertEqual(output[-1].twist.linear.x, 0.)
        stream = True; self.spin(.12, feed); self.assertEqual(output[-1].twist.linear.x, 1.)
        trusted = False; generation = 3; self.spin(.12, feed); self.assertEqual(output[-1].twist.linear.x, 0.)
        # A new healthy generation starts a new hold. Do not reuse buffered commands.
        trusted = True; generation = 4
        self.spin(.8, feed); self.assertEqual(output[-1].twist.linear.x, 0.)
        commands = False; self.spin(.35, feed); self.assertEqual(output[-1].twist.linear.x, 0.)
        commands = True; self.spin(.12, feed); self.assertEqual(output[-1].twist.linear.x, 1.)
        self.assert_no_global_tf_subscribers()

    def test_watchdog_generic_keepalive_freshness_and_timeouts(self):
        self.start('cmd_vel_watchdog', {
            'input_topic': '/probe/cmd', 'output_topic': '/probe/safe',
            'odom_topic': '/probe/odom', 'scan_topic': '/probe/scan',
            'joint_states_topic': '/probe/joint_states', 'require_tf': False,
            'publish_rate': 100., 'odom_timeout': .2, 'scan_timeout': .2, 'joint_states_timeout': .2})
        output = self.received(TwistStamped, '/probe/safe')
        pubs = {
            'cmd': self.node.create_publisher(TwistStamped, '/probe/cmd', qos_profile_sensor_data),
            'odom': self.node.create_publisher(Odometry, '/probe/odom', qos_profile_sensor_data),
            'scan': self.node.create_publisher(LaserScan, '/probe/scan', qos_profile_sensor_data),
            'joints': self.node.create_publisher(JointState, '/probe/joint_states', qos_profile_sensor_data),
        }
        self.spin(.6)
        expected_types = {
            '/probe/odom': 'nav_msgs/msg/Odometry',
            '/probe/scan': 'sensor_msgs/msg/LaserScan',
            '/probe/joint_states': 'sensor_msgs/msg/JointState',
        }
        for topic, expected_type in expected_types.items():
            infos = [i for i in self.node.get_subscriptions_info_by_topic(topic) if i.node_name == 'cmd_vel_watchdog']
            self.assertEqual(len(infos), 1, topic)
            self.assertEqual(infos[0].topic_type, expected_type)
        active = {'odom': True, 'scan': True, 'joints': False}

        def feed():
            stamp = self.node.get_clock().now().to_msg()
            cmd = TwistStamped(); cmd.header.stamp = stamp; cmd.twist.linear.x = 1.
            pubs['cmd'].publish(cmd)
            if active['odom']:
                odom = Odometry(); odom.header.stamp = stamp; odom.pose.pose.orientation.w = 1.
                pubs['odom'].publish(odom)
            if active['scan']:
                scan = LaserScan(); scan.header.stamp = stamp; scan.ranges = [1.5, 2.0, 2.5]
                pubs['scan'].publish(scan)
            if active['joints']:
                joints = JointState(); joints.header.stamp = stamp
                joints.name = ['gimbal_joint', 'wheel_fl', 'wheel_fr']
                joints.position = [0.1, 0.2, 0.3]; joints.velocity = [0.0, 1.0, 1.0]
                pubs['joints'].publish(joints)

        self.spin(.25, feed)
        self.assertEqual(output[-1].twist.linear.x, 0.)
        active['joints'] = True
        self.spin(.12, feed)
        self.assertEqual(output[-1].twist.linear.x, 1.)
        for stream in ('joints', 'scan', 'odom'):
            active[stream] = False
            self.spin(.3, feed)
            self.assertEqual(output[-1].twist.linear.x, 0., f'{stream} timeout must close gate')
            active[stream] = True
            self.spin(.12, feed)
            self.assertEqual(output[-1].twist.linear.x, 1., f'{stream} recovery must reopen gate')

    def test_timed_self_filter_waits_for_unsorted_cloud_coverage(self):
        geometry = Path(self.logs.name) / 'self_filter.yaml'
        planes = [[sign if component == axis else 0. for component in range(3)] + [-.25]
                  for axis in range(3) for sign in (-1., 1.)]
        geometry.write_text(yaml.safe_dump({'self_filter': {
            'frame': 'base_link', 'tolerance': 0., 'hull_planes': [planes], 'wheels': [],
        }}))
        self.start('pointcloud_frame_republisher', {
            'target_frame': 'base_link', 'expected_source_frame': 'probe_lidar',
            'use_point_timestamps': True, 'use_joint_interpolation': True,
            'sensor_offset': [0., 0., 0.], 'sensor_rpy': [0., 0., 0.],
            'pending_timeout': .7, 'pending_queue_size': 1, 'self_filter_geometry_file': str(geometry),
        }, [('points_in', '/probe/raw'), ('points_out', '/probe/filtered'), ('joint_states', '/probe/joints')])
        received = self.received(PointCloud2, '/probe/filtered')
        cloud_pub = self.node.create_publisher(PointCloud2, '/probe/raw', qos_profile_sensor_data)
        joints = self.node.create_publisher(JointState, '/probe/joints', qos_profile_sensor_data)
        self.spin(.8)

        def joint(nanosec, position):
            sample = JointState(name=['gimbal_joint'], position=[position])
            sample.header.stamp.sec = 10
            sample.header.stamp.nanosec = nanosec
            joints.publish(sample)

        joint(0, 0.)
        joint(20_000_000, 0.)
        self.spin(.05)
        cloud = PointCloud2(width=2, height=2, point_step=32, row_step=72, is_bigendian=True)
        cloud.header.frame_id = 'probe_lidar'
        cloud.header.stamp.sec = 10
        cloud.fields = [PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
                        for index, name in enumerate('xyz')]
        cloud.fields.append(PointField(name='timestamp', offset=16, datatype=PointField.FLOAT64, count=1))
        payload = bytearray([0x55] * (cloud.height * cloud.row_step))
        offsets = (0, 32, 72, 104)
        for offset, xyz, stamp in zip(offsets, ((0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (2., 0., 0.)),
                                      (10.010e9, 10.030e9, 10.030e9, 10.010e9)):
            struct.pack_into('>fff', payload, offset, *xyz)
            struct.pack_into('>d', payload, offset + 16, stamp)
        cloud.data = bytes(payload)

        ordered = deepcopy(cloud)
        ordered.header.stamp.nanosec = 1
        ordered_payload = bytearray(payload)
        for index, offset in enumerate(offsets, 1):
            struct.pack_into('>d', ordered_payload, offset + 16, 10e9 + index * 1e6)
        ordered.data = bytes(ordered_payload)
        cloud_pub.publish(ordered)
        self.spin(.12)
        self.assertEqual(len(received), 1)
        self.assertEqual(received[0].header.stamp, ordered.header.stamp)
        self.assertEqual(received[0].width, 3)
        for offset, expected in zip((0, 32, 64), ((1., 0., 0.), (0., 1., 0.), (2., 0., 0.))):
            self.assertEqual(struct.unpack_from('>fff', received[0].data, offset), expected)
        received.clear()

        cloud_pub.publish(cloud)
        self.spin(.12)
        self.assertFalse(received, 'an uncovered interior point must defer the entire cloud')

        bad_time = deepcopy(cloud)
        bad_payload = bytearray(payload)
        struct.pack_into('>d', bad_payload, offsets[1] + 16, math.nan)
        bad_time.data = bytes(bad_payload)
        cloud_pub.publish(bad_time)
        bad_layout = deepcopy(cloud)
        bad_layout.data = bytes(payload[:-1])
        cloud_pub.publish(bad_layout)
        self.spin(.08)
        self.assertFalse(received, 'invalid input must not evict or release a valid pending frame')

        joint(40_000_000, math.pi)
        self.spin(.2)
        self.assertEqual(len(received), 1)
        output = received[0]
        self.assertEqual(output.header.frame_id, 'base_link')
        self.assertEqual(output.header.stamp, cloud.header.stamp)
        self.assertEqual((output.width, output.height, output.row_step), (3, 1, 96))
        self.assertTrue(output.is_bigendian)
        for offset, original, xyz in zip((0, 32, 64), offsets[1:], ((0., 1., 0.), (-1., 0., 0.), (2., 0., 0.))):
            for actual, expected in zip(struct.unpack_from('>fff', output.data, offset), xyz):
                self.assertAlmostEqual(actual, expected, places=5)
            self.assertEqual(bytes(output.data[offset + 12:offset + 32]), bytes(payload[original + 12:original + 32]))

    def test_joint_coverage_wait_avoids_reprocessing_large_clouds(self):
        process = self.start('pointcloud_frame_republisher', {
            'target_frame': 'base_link', 'expected_source_frame': 'probe_lidar',
            'use_point_timestamps': True, 'use_joint_interpolation': True, 'pending_timeout': .7,
        }, [('points_in', '/probe/raw'), ('points_out', '/probe/compensated'), ('joint_states', '/probe/joints')])
        clouds = self.node.create_publisher(PointCloud2, '/probe/raw', qos_profile_sensor_data)
        joints = self.node.create_publisher(JointState, '/probe/joints', qos_profile_sensor_data)
        received = self.received(PointCloud2, '/probe/compensated')
        self.spin(.8)

        def joint(sec, nanosec):
            sample = JointState(name=['gimbal_joint'], position=[.2])
            sample.header.stamp.sec, sample.header.stamp.nanosec = sec, nanosec
            joints.publish(sample)

        process_stat = Path(f'/proc/{process.pid}/stat')
        ticks_per_second = os.sysconf('SC_CLK_TCK')

        def cpu_ticks():
            fields = process_stat.read_text().rsplit(') ', 1)[1].split()
            return int(fields[11]) + int(fields[12])  # User and system CPU ticks.

        cloud = PointCloud2(width=20_000, height=1, point_step=24, row_step=480_000)
        cloud.header.frame_id = 'probe_lidar'
        cloud.fields = [PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
                        for index, name in enumerate('xyz')]
        cloud.fields.append(PointField(name='timestamp', offset=16, datatype=PointField.FLOAT64, count=1))
        payload = bytearray(struct.pack('<fff4xd', 1., .1, .2, 0.) * cloud.width)
        wait_ticks = 0
        for sec in range(10, 13):
            cloud.header.stamp.sec = sec
            for index in range(cloud.width):
                struct.pack_into('<d', payload, index * cloud.point_step + 16, sec * 1e9 + 5e6 + index * 100)
            struct.pack_into('<d', payload, (cloud.width - 2) * cloud.point_step + 16, sec * 1e9 + 30e6)
            cloud.data = bytes(payload)
            joint(sec, 0)
            joint(sec, 20_000_000)
            self.spin(.04)
            before = cpu_ticks()
            clouds.publish(cloud)
            self.spin(.23)
            wait_ticks += cpu_ticks() - before
            self.assertFalse(received)
            joint(sec, 40_000_000)
            self.spin(.2)
            self.assertEqual([frame.header.stamp.sec for frame in received], [sec])
            self.assertEqual(received[0].width, cloud.width)
            received.clear()

        wait_cpu_ms = wait_ticks * 1_000. / ticks_per_second
        print(json.dumps({'cloud_wait_probe': {'frames': 3, 'points': cloud.width,
                          'total_wait_seconds': .69, 'cpu_ms': wait_cpu_ms}}), flush=True)
        # Allow DDS reception and clock tick rounding while catching repeated full transforms.
        self.assertLessEqual(wait_cpu_ms, 35.)

    def test_padded_cloud_and_fixed_imu_remain_cpp(self):
        self.start('pointcloud_frame_republisher', {'target_frame': 'probe_sensor',
            'expected_source_frame': 'probe_lidar'}, [('points_in', '/probe/points'), ('points_out', '/probe/transformed')])
        self.start('gimbal_imu_compensator', {'fixed_sensor': True, 'target_frame': 'probe_sensor',
            'sensor_offset': [0., 0., 0.], 'sensor_rpy': [0., 0., math.pi / 2],
            'input_acceleration_scale': 9.81}, [('imu_in', '/probe/imu'), ('imu_out', '/probe/imu_out')])
        self.start('pointcloud_to_laserscan_node', {'target_frame': 'base_link', 'min_height': -.5,
            'max_height': 5., 'range_min': .01, 'publish_processed_cloud': True,
            'processed_cloud_project_to_2d': True},
            [('cloud_in', '/probe/transformed'), ('scan', '/probe/scan'), ('processed_cloud', '/probe/processed')],
            package='pointcloud_to_laserscan')
        static1 = self.static('probe_sensor', 'probe_lidar', (.01, .02, -.03), math.pi / 2)
        static2 = self.static('base_link', 'probe_sensor')
        cloud_out = self.received(PointCloud2, '/probe/transformed')
        imu_out = self.received(Imu, '/probe/imu_out')
        scans = self.received(LaserScan, '/probe/scan')
        clouds = self.node.create_publisher(PointCloud2, '/probe/points', qos_profile_sensor_data)
        imus = self.node.create_publisher(Imu, '/probe/imu', qos_profile_sensor_data)
        cloud = PointCloud2(); cloud.header.frame_id = 'probe_lidar'; cloud.width = 1; cloud.height = 2
        cloud.point_step = 32; cloud.row_step = 40
        cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1) for i,n in enumerate('xyz')]
        payload = bytearray([0x55] * 80)
        struct.pack_into('<fff', payload, 0, 1., 2., 3.); struct.pack_into('<fff', payload, 40, 1., 2., 3.)
        cloud.data = bytes(payload)
        imu = Imu(); imu.angular_velocity.x = 1.; imu.linear_acceleration.x = 1.
        imu.angular_velocity_covariance = [1., 0., 0., 0., 2., 0., 0., 0., 3.]
        sent_stamps = set()
        def feed():
            stamp = self.node.get_clock().now().to_msg(); cloud.header.stamp = stamp; imu.header.stamp = stamp
            sent_stamps.add((stamp.sec, stamp.nanosec))
            clouds.publish(cloud); imus.publish(imu)
        self.spin(1.5, feed)
        self.assertTrue(cloud_out and imu_out and scans)
        for message in [*cloud_out, *imu_out]:
            self.assertIn((message.header.stamp.sec, message.header.stamp.nanosec), sent_stamps)
        clock_subscribers = {info.node_name for info in self.node.get_subscriptions_info_by_topic('/clock')}
        self.assertTrue(clock_subscribers.isdisjoint({
            'pointcloud_frame_republisher', 'gimbal_imu_compensator', 'pointcloud_to_laserscan',
        }))
        for offset in (0, 40):
            actual = struct.unpack_from('<fff', cloud_out[-1].data, offset)
            for a, b in zip(actual, (-1.99, 1.02, 2.97)): self.assertAlmostEqual(a, b, places=5)
            self.assertEqual(bytes(cloud_out[-1].data[offset+12:offset+40]), bytes(payload[offset+12:offset+40]))
        self.assertEqual(scans[-1].header.frame_id, 'base_link')
        ranges_without_cloud_subscriber = list(scans[-1].ranges)
        self.assertAlmostEqual(min(ranges_without_cloud_subscriber), math.hypot(-1.99, 1.02), places=5)
        self.assertAlmostEqual(imu_out[-1].angular_velocity.y, 1., places=10)
        self.assertAlmostEqual(imu_out[-1].linear_acceleration.y, 9.81, places=10)
        self.assertEqual(imu_out[-1].orientation_covariance[0], -1.)
        self.assert_no_global_tf_subscribers()
        for info in self.node.get_subscriptions_info_by_topic('/tf_static'):
            self.assertNotIn(info.node_name, ('pointcloud_frame_republisher', 'pointcloud_to_laserscan'))
        processed = self.received(PointCloud2, '/probe/processed')
        scans.clear()
        self.spin(.7, feed)
        self.assertTrue(processed and scans)
        self.assertEqual(list(scans[-1].ranges), ranges_without_cloud_subscriber)
        projected = processed[-1]
        self.assertEqual(projected.header.frame_id, 'base_link')
        self.assertEqual((projected.width, projected.height, projected.row_step), (2, 1, 64))
        for offset in (0, 32):
            for actual, expected in zip(struct.unpack_from('<fff', projected.data, offset), (-1.99, 1.02, 0.)):
                self.assertAlmostEqual(actual, expected, places=5)
            self.assertEqual(bytes(projected.data[offset+12:offset+32]), bytes(payload[12:32]))

    def test_self_filter_rejects_invalid_geometry_and_normalizes_wheel_axis(self):
        binary = Path(get_package_prefix('xxu_bringup')) / 'lib' / 'xxu_bringup' / 'pointcloud_frame_republisher'
        for name, spec in (
            ('empty_hull', {'self_filter': {'frame': 'base_link', 'tolerance': 0.0, 'hull_planes': [[]]}}),
            ('missing_tol', {'self_filter': {'frame': 'base_link', 'hull_planes': []}}),
            ('zero_axis', {
                'self_filter': {'frame': 'base_link', 'tolerance': 0.0, 'hull_planes': []},
                'wheels': [{'center': [0., 0., 0.], 'axis': [0., 0., 0.], 'radius': 0.1, 'width': 0.1}],
            }),
        ):
            with self.subTest(case=name):
                path = Path(self.logs.name) / f'{name}.yaml'
                path.write_text(yaml.safe_dump(spec))
                result = subprocess.run(
                    [str(binary), '--ros-args', '-p', 'target_frame:=base_link',
                     '-p', f'self_filter_geometry_file:={path}'],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=5,
                    env=dict(os.environ, ROS_DOMAIN_ID=str(self.domain), ROS_LOCALHOST_ONLY='1'))
                self.assertNotEqual(result.returncode, 0, result.stdout)

        # Non-unit wheel axis [0, 0, 0.5] must normalize to [0, 0, 1] and keep a point at z=0.08 (> width/2=0.05).
        valid_path = Path(self.logs.name) / 'non_unit_wheel.yaml'
        valid_path.write_text(yaml.safe_dump({
            'self_filter': {'frame': 'base_link', 'tolerance': 0.0, 'hull_planes': []},
            'wheels': [{'center': [0., 0., 0.], 'axis': [0., 0., 0.5], 'radius': 0.1, 'width': 0.1}],
        }))
        self.start('pointcloud_frame_republisher', {
            'target_frame': 'base_link', 'expected_source_frame': 'base_link',
            'self_filter_geometry_file': str(valid_path),
        }, [('points_in', '/probe/wheel_in'), ('points_out', '/probe/wheel_out')])
        cloud_out = self.received(PointCloud2, '/probe/wheel_out')
        clouds = self.node.create_publisher(PointCloud2, '/probe/wheel_in', qos_profile_sensor_data)
        cloud = PointCloud2()
        cloud.header.frame_id = 'base_link'
        cloud.height = 1
        cloud.width = 2
        cloud.point_step = 12
        cloud.row_step = 24
        cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        # Point 0 inside cylinder (z=0.02, r=0.02), Point 1 outside width (z=0.08, r=0.02)
        cloud.data = struct.pack('<ffffff', 0.02, 0.0, 0.02, 0.02, 0.0, 0.08)
        def feed():
            cloud.header.stamp = self.node.get_clock().now().to_msg()
            clouds.publish(cloud)
        self.spin(0.8, feed)
        self.assertTrue(cloud_out)
        self.assertEqual(cloud_out[-1].width, 1)
        kept = struct.unpack_from('<fff', cloud_out[-1].data, 0)
        self.assertAlmostEqual(kept[2], 0.08, places=5)


if __name__ == '__main__':
    unittest.main()
