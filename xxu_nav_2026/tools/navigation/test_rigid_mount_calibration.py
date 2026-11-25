#!/usr/bin/env python3
"""Exercise the installed C++ calibration toolchain with synthetic inputs and a read-only bag."""
import array
from contextlib import contextmanager
import importlib.util
from itertools import groupby
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Quaternion, Vector3
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_srvs.srv import Trigger
import yaml

from xxu_bringup.sensor_launch import load_rigid_mount


WORKSPACE = Path(__file__).resolve().parents[2]
LEVER = np.array([0.22, -0.11, 0.34])
IMU_FROM_LIDAR = np.array([-0.011, -0.02329, 0.04412])
RPY = np.deg2rad([12., -8., 135.])
SPIN_CENTER = np.array([3., 0., 0.])
EPOCH_NS = 1_700_000_000_000_000_000
IMU = '/livox/imu'
LIDAR = '/livox/lidar'
ODOM = '/calibration/lidar_odometry'


def euler_matrix(roll, pitch, yaw):
    a, b, c = np.cos((roll, pitch, yaw))
    x, y, z = np.sin((roll, pitch, yaw))
    return np.array([[c*b, c*y*x-z*a, c*y*a+z*x],
                     [z*b, z*y*x+c*a, z*y*a-c*x],
                     [-y, b*x, b*a]])


BASE_FROM_IMU = euler_matrix(*RPY)


def quaternion(roll, pitch, yaw):
    a, b, c = np.cos(np.array([roll, pitch, yaw]) / 2)
    x, y, z = np.sin(np.array([roll, pitch, yaw]) / 2)
    return Quaternion(x=float(x*b*c-a*y*z), y=float(a*y*c+x*b*z),
                      z=float(a*b*z-x*y*c), w=float(a*b*c+x*y*z))


def stamp(message, timestamp, frame):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(timestamp, 10**9)
    message.header.frame_id = frame
    return message


def imu_message(timestamp, yaw_rate=0., scale=9.81, accel_x=0.):
    message = stamp(Imu(), timestamp, 'livox_frame')
    raw_accel = (BASE_FROM_IMU.T @ [accel_x, 0., 9.81] / scale).tolist()
    message.linear_acceleration = Vector3(**dict(zip('xyz', raw_accel)))
    message.angular_velocity = Vector3(**dict(zip('xyz', (BASE_FROM_IMU[2] * yaw_rate).tolist())))
    return message


def odom_message(timestamp, position, yaw, *, physical=True):
    message = stamp(Odometry(), timestamp, 'odom')
    message.child_frame_id = 'imu'
    imu_position = position + euler_matrix(0., 0., yaw) @ LEVER
    p = message.pose.pose.position
    p.x, p.y, p.z = map(float, imu_position)
    message.pose.pose.orientation = (
        quaternion(RPY[0], RPY[1], RPY[2]+yaw) if physical else quaternion(0., 0., yaw))
    return message


def make_cloud(timestamp, *, clutter, floor_z_offset=0.):
    rng = np.random.default_rng(673)
    xy = rng.uniform(-2.8, 2.8, (460, 2))
    floor = np.column_stack((xy, floor_z_offset + rng.normal(0., .0004, len(xy))))
    if clutter:
        # The negative-Z chassis ROI layer is denser than the floor.
        chassis_xy = rng.uniform([.62, -.43], [1.2, .43], (900, 2))
        chassis = np.column_stack((chassis_xy, .2 + rng.normal(0., .001, len(chassis_xy))))
        ceiling = np.column_stack((xy, 2.0 + rng.normal(0., .001, len(xy))))
        floor = np.concatenate((floor, chassis, ceiling))
    raw_points = np.asarray((floor - LEVER) @ BASE_FROM_IMU - IMU_FROM_LIDAR, dtype='<f4')
    cloud = stamp(PointCloud2(), timestamp, 'livox_frame')
    cloud.width, cloud.height = len(raw_points), 1
    cloud.point_step, cloud.row_step = 12, len(raw_points) * 12
    cloud.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                    for index, name in enumerate('xyz')]
    cloud.is_dense = True
    cloud.data = array.array('B', raw_points.tobytes())
    return cloud


def write_bag(path, storage='mcap', stationary_only=False, *, scale=9.81, odom_lag=0.,
              fastwrite=False, unordered=False, odom_child_frame='imu', straight_speed=.75,
              odom_stride=8, straight_odom_jitter=0., lidar_lag=0., moving_lidar_z_offset=0.,
              reception_window=None, stale_stationary_lidar=False, split=False,
              odom_topic=ODOM, physical_odom=True, stationary_imu_outage=False,
              spin_odom_dropout=False):
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=str(path), storage_id=storage,
                               storage_preset_profile='fastwrite' if fastwrite else ''),
                ConverterOptions('cdr', 'cdr'))
    types = [(IMU, 'sensor_msgs/msg/Imu'), (LIDAR, 'sensor_msgs/msg/PointCloud2')]
    if not stationary_only:
        types.append((odom_topic, 'nav_msgs/msg/Odometry'))
    for name, msg_type in types:
        writer.create_topic(TopicMetadata(id=0, name=name, type=msg_type,
                                          serialization_format='cdr'))
    cloud = make_cloud(EPOCH_NS, clutter=True,
                       floor_z_offset=.20 if stale_stationary_lidar or stationary_imu_outage else 0.)
    moving_cloud = make_cloud(EPOCH_NS, clutter=True, floor_z_offset=moving_lidar_z_offset)
    records = []
    lag_ns = round(odom_lag * 10**9)
    lidar_lag_ns = round(lidar_lag * 10**9)
    jitter_ns = round(straight_odom_jitter * 10**9)
    stationary_start = 3 if stale_stationary_lidar else 0
    straight_start = stationary_start + 3
    spin_start = straight_start + math.ceil(3 / straight_speed)
    spin_duration = 8 if spin_odom_dropout else 6
    last_tick = (straight_start if stationary_only else spin_start + spin_duration) * 200
    for tick in range(last_tick + 1):
        t = tick * .005
        timestamp = EPOCH_NS + tick * 5_000_000
        yaw_rate = math.tau / 6 if t >= spin_start else 0.
        if t >= stationary_start and (not stationary_imu_outage or tick <= 100 or tick >= 500):
            accel_x = 2. if odom_lag and straight_start <= t < straight_start + .25 else 0.
            records.append((IMU, serialize_message(
                imu_message(timestamp, yaw_rate, scale, accel_x)), timestamp))
        cloud_due = tick % 20 == 0
        if stationary_imu_outage and t <= straight_start:
            cloud_due = tick in (200, 220, 240, 260)
        if cloud_due:
            if t > straight_start or not stale_stationary_lidar:
                scan = stamp(moving_cloud if t > straight_start else cloud, timestamp, 'livox_frame')
                records.append((LIDAR, serialize_message(scan), timestamp + lidar_lag_ns))
            elif stationary_start <= t <= stationary_start + 2.5:
                scan = stamp(cloud, timestamp - stationary_start * 10**9, 'livox_frame')
                records.append((LIDAR, serialize_message(scan), timestamp))
        if not stationary_only and tick % odom_stride == 0:
            position = np.array([np.clip((t - straight_start) * straight_speed, 0., 3.), 0., 0.])
            yaw = max(0., t - spin_start) * yaw_rate
            if spin_odom_dropout and tick >= spin_start * 200:
                spin_tick = tick - spin_start * 200
                # Two 10-degree fragments ending at 1.2 s and restarting at 5.0 s.
                if spin_tick <= 240:
                    yaw = math.radians(10. * spin_tick / 240)
                elif 1000 <= spin_tick <= 1240:
                    yaw = math.radians(179. + 10. * (spin_tick - 1000) / 240)
                else:
                    continue
            odom = odom_message(timestamp, position, yaw, physical=physical_odom)
            odom.child_frame_id = odom_child_frame
            delay_ns = jitter_ns if straight_start <= t < spin_start and tick // odom_stride % 2 else 0
            records.append((odom_topic, serialize_message(odom), timestamp + lag_ns + delay_ns))
    if reception_window is not None:
        start, end = (EPOCH_NS + round(t * 10**9) for t in reception_window)
        records = [record for record in records if start <= record[2] <= end]
    records.sort(key=lambda record: record[2])
    if unordered:
        # Reverse each half-second block in file order, with one duplicate per stream.
        for topic, sec in ((IMU, 1.), (LIDAR, 1.), (odom_topic, 4.)):
            original = next(record for record in records
                            if record[0] == topic and record[2] == EPOCH_NS + sec * 10**9)
            records.append(original)
        records.sort(key=lambda record: record[2])
        for _, block in groupby(records, key=lambda record: (record[2] - EPOCH_NS) // 500_000_000):
            for record in reversed(list(block)):
                writer.write(*record)
    elif split:
        overlap_start = EPOCH_NS + 2_400_000_000
        first_file_tail = EPOCH_NS + 2_600_000_000
        for record in records:
            if record[2] < overlap_start or record[2] == first_file_tail:
                writer.write(*record)
        writer.split_bagfile()
        for record in records:
            if record[2] >= overlap_start and record[2] != first_file_tail:
                writer.write(*record)
    else:
        for record in records:
            writer.write(*record)
    writer.close()


class CalibrationLaunchTest(unittest.TestCase):
    def test_driver_forwards_distinct_lidar_and_imu_frames(self):
        launch_file = (Path(__file__).resolve().parents[2] /
                       'src/xxu_bringup/launch/rigid_mount_calibration.launch.py')
        spec = importlib.util.spec_from_file_location('calibration_launch', launch_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with tempfile.TemporaryDirectory() as folder:
            driver_config = Path(folder) / 'driver.json'
            driver_config.write_text('{}')
            context = LaunchContext()
            context.launch_configurations.update(
                driver='true', driver_config=str(driver_config), start_zlio='false',
                lidar_frame='calibration_lidar', imu_source_frame='calibration_imu',
            )
            actions = module.generate_launch_description().entities
            for action in actions:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            setup_action = next(action for action in actions if isinstance(action, OpaqueFunction))
            with patch.object(module, 'Node', side_effect=lambda **options: options):
                nodes = setup_action.execute(context)

        drivers = [node for node in nodes if node['executable'] == 'livox_ros_driver2_node']
        self.assertEqual(len(drivers), 1)
        parameters = drivers[0]['parameters'][0]
        self.assertEqual(parameters['frame_id'], 'calibration_lidar')
        self.assertEqual(parameters['imu_frame_id'], 'calibration_imu')


class CalibrationIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture_root = tempfile.TemporaryDirectory(prefix='xxu-calibration-fixtures-')
        cls.addClassCleanup(cls.fixture_root.cleanup)
        cls.bag = Path(cls.fixture_root.name) / 'synthetic_mcap'
        write_bag(cls.bag)
        cls.stationary_bag = Path(cls.fixture_root.name) / 'stationary_sqlite'
        write_bag(cls.stationary_bag, 'sqlite3', stationary_only=True)
        cls.bin_dir = Path(get_package_prefix('xxu_bringup')) / 'lib' / 'xxu_bringup'
        cls.domain = int(os.environ.get('ROS_DOMAIN_ID', '197'))
        if cls.domain < 190:
            raise ValueError('use a test ROS_DOMAIN_ID of at least 190')
        cls.env = dict(os.environ, ROS_DOMAIN_ID=str(cls.domain), ROS_LOCALHOST_ONLY='1')

    def setUp(self):
        root = tempfile.TemporaryDirectory(prefix='xxu-calibration-test-')
        self.addCleanup(root.cleanup)
        self.root = Path(root.name)
        self.mount_path = self.root / 'rigid_mount.yaml'
        self.backend_path = self.root / 'backend_body.yaml'
        self.report_path = self.root / 'report.yaml'
        self.env = dict(self.env, ROS_LOG_DIR=str(self.root / 'roslogs'))

    def cli(self, *args, expected=0):
        result = subprocess.run([str(self.bin_dir / 'rigid_mount_calibrate'), *map(str, args)],
                                env=self.env, text=True, capture_output=True, timeout=60)
        if expected is not None:
            self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        return result

    def output_args(self):
        return ('--output-mount', self.mount_path, '--output-backend-body', self.backend_path,
                '--output-report', self.report_path)

    def assert_mount_accuracy(self, mount_path, scale=9.81):
        mount = load_rigid_mount(mount_path)
        r_base_sensor = euler_matrix(*mount['base_to_sensor']['rpy'])
        r_base_imu = r_base_sensor @ euler_matrix(*mount['sensor_from_imu_rpy'])
        angle = math.degrees(math.acos(np.clip((np.trace(r_base_imu.T @ BASE_FROM_IMU)-1) / 2, -1, 1)))
        distance = np.linalg.norm(np.array(mount['base_to_sensor']['xyz']) - LEVER)
        self.assertLess(angle, .5)
        self.assertLess(distance, .005)
        self.assertEqual(mount['imu_acceleration_scale'], scale)
        return mount, angle, distance

    def test_launch_isolates_frontend_outputs_and_resolves_odom_input(self):
        launch_file = (Path(get_package_share_directory('xxu_bringup')) /
                       'launch/rigid_mount_calibration.launch.py')
        remaps = {
            '/odometry': '/calibration/odometry',
            '/frontend/frame': '/calibration/frontend/frame',
            '/frontend/odometry': '/calibration/frontend/odometry',
            '/frontend/session': '/calibration/frontend/session',
            '/cloud_deskewed': '/calibration/cloud_deskewed',
        }

        def launch_setup(context):
            description = PythonLaunchDescriptionSource(str(launch_file))
            for action in description.get_launch_description(context).entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
                if isinstance(action, OpaqueFunction):
                    return action.execute(context)
            self.fail('calibration launch is missing its setup action')

        for start_zlio, override, allow_unisolated, expected in (
            ('true', '', False, ODOM),
            ('false', '', False, ODOM),
            ('true', '/manual/calibration_odom', False, '/manual/calibration_odom'),
            ('false', '/manual/calibration_odom', False, '/manual/calibration_odom'),
            ('false', '/lidar_odometry', False, None),
            ('false', '/odometry', False, None),
            ('false', '/lidar_odometry', True, '/lidar_odometry'),
            ('true', '/odometry', True, '/odometry'),
        ):
            with self.subTest(start_zlio=start_zlio, override=override,
                              allow_unisolated=allow_unisolated):
                context = LaunchContext()
                context.launch_configurations['start_zlio'] = start_zlio
                if override:
                    context.launch_configurations['odom_topic'] = override
                if allow_unisolated:
                    context.launch_configurations['allow_unisolated_odom_topic'] = 'true'
                time_args = {
                    'point_time_field': 'offset_time' if override else 'timestamp',
                    'point_time_scale': '1e-6' if override else '1e-9',
                    'point_time_absolute': 'false' if override else 'true',
                }
                frame_args = {'odom_frame': 'raw_odom', 'odom_child_frame': 'raw_imu'}
                if override:
                    context.launch_configurations.update(time_args | frame_args)
                if expected is None:
                    with self.assertRaisesRegex(ValueError, 'shared production topic'):
                        launch_setup(context)
                    continue
                actions = launch_setup(context)
                if start_zlio == 'true':
                    group, node = actions
                    self.assertIsInstance(group, GroupAction)
                    includes = 0
                    for child in group.get_sub_entities():
                        if isinstance(child, IncludeLaunchDescription):
                            includes += 1
                            forwarded = {
                                perform_substitutions(context, normalize_to_list_of_substitutions(name)):
                                perform_substitutions(context, normalize_to_list_of_substitutions(value))
                                for name, value in child.launch_arguments
                            }
                            self.assertEqual({name: forwarded.get(name) for name in time_args}, time_args)
                            resolved = dict(context.launch_configurations.get('ros_remaps', []))
                            for src, dst in {**remaps, '/lidar_odometry': expected}.items():
                                self.assertEqual(resolved.get(src), dst, resolved)
                        else:
                            child.execute(context)
                    self.assertEqual(includes, 1)
                    self.assertNotIn('ros_remaps', context.launch_configurations)
                else:
                    node, = actions
                self.assertIsInstance(node, Node)
                with patch.object(node, '_create_params_file_from_dict',
                                  return_value=str(launch_file)) as create_params:
                    node._perform_substitutions(context)
                parameters = create_params.call_args.args[0]
                self.assertEqual(parameters['odom_topic'], expected)
                self.assertIs(parameters['allow_unisolated_odom_topic'], allow_unisolated)
                expected_frames = frame_args if override else {
                    'odom_frame': 'odom', 'odom_child_frame': 'imu'}
                self.assertEqual({name: parameters.get(name) for name in expected_frames}, expected_frames)

    def test_unisolated_production_odom_topic_and_compensated_odometry_are_rejected(self):
        topic = '/lidar_odometry'
        raw_bag = self.root / 'raw_production_topic'
        compensated_bag = self.root / 'compensated_production_topic'
        write_bag(raw_bag, odom_topic=topic)
        write_bag(compensated_bag, odom_topic=topic, physical_odom=False)
        options = ('--odom-topic', topic, '--auto-segment', '--confirm')

        result = self.cli('--bag', raw_bag, *options, *self.output_args(), expected=1)
        self.assertIn('Calibration unconfirmed', result.stdout)
        self.assertIn('allow_unisolated_odom_topic', result.stderr)
        self.assertFalse(self.backend_path.exists())
        self.assertFalse(yaml.safe_load(self.mount_path.read_text())['calibration_confirmed'])
        report = yaml.safe_load(self.report_path.read_text())
        self.assertEqual(report['odom_topic'], topic)
        self.assertIs(report['allow_unisolated_odom_topic'], False)

        opt_in = '--allow-unisolated-odom-topic'
        result = self.cli('--bag', compensated_bag, opt_in, *options, *self.output_args(), expected=1)
        self.assertIn('Calibration unconfirmed', result.stdout)
        self.assertFalse(self.backend_path.exists())
        self.assertFalse(yaml.safe_load(self.mount_path.read_text())['calibration_confirmed'])
        report = yaml.safe_load(self.report_path.read_text())
        self.assertIs(report['allow_unisolated_odom_topic'], True)
        reason = ('odometry orientation disagrees with stationary gravity leveling '
                  '(check raw vs compensated odometry)')
        for stage in ('straight', 'spin'):
            self.assertIn(reason, report[stage]['reasons'])
            self.assertIn(f'{stage}: {reason}', report['reasons'])

        result = self.cli('--bag', raw_bag, opt_in, *options, *self.output_args())
        self.assertIn('Calibration confirmed', result.stdout)
        self.assertTrue(self.backend_path.is_file())
        self.assert_mount_accuracy(self.mount_path)

    def test_offline_mcap_calibrates_and_verifies_confirmed_mount(self):
        report_path = self.root / 'verification.yaml'
        for mode in ('base_aligned', 'imu_aligned'):
            with self.subTest(mode=mode):
                self.cli('--bag', self.bag, '--auto-segment', '--confirm',
                         '--sensor-frame-mode', mode, '--odom-frame', 'odom',
                         '--odom-child-frame', 'imu', *self.output_args())
                mount, angle, distance = self.assert_mount_accuracy(self.mount_path)
                report = yaml.safe_load(self.report_path.read_text())
                self.assertTrue(report['calibration_confirmed'])
                self.assertTrue(all(report['windows_sec'][stage]
                                    for stage in ('stationary', 'straight', 'spin')), report)
                self.assertEqual(len(report['refinement']['iterations']), 3)
                body = yaml.safe_load(self.backend_path.read_text())['common']['body']
                self.assertEqual(body['frame'], 'base_link')
                np.testing.assert_allclose(list(body['offset'].values()), -LEVER, atol=.005)
                self.cli('--verify-mount', self.mount_path, '--bag', self.bag, '--auto-segment',
                         '--output-report', report_path)
                verified = yaml.safe_load(report_path.read_text())
                self.assertTrue(verified['verification_passed'], verified['reasons'])
                print(f'MCAP {mode}: rotation error {angle:.6f} deg, translation error '
                      f'{distance * 1000:.4f} mm, verification passed', flush=True)

                mount['base_to_sensor']['xyz'][0] += .12
                tampered_path = self.root / 'tampered.yaml'
                tampered_path.write_text(yaml.safe_dump(mount))
                self.cli('--verify-mount', tampered_path, '--bag', self.bag, '--auto-segment',
                         '--output-report', report_path, expected=1)
                verified = yaml.safe_load(report_path.read_text())
                self.assertFalse(verified['verification_passed'])
                self.assertGreater(verified['spin']['residual_rms_m'], .025)

    def test_cli_rejects_base_footprint_odometry_child(self):
        bag = self.root / 'base_footprint_mcap'
        write_bag(bag, odom_child_frame='base_footprint')
        result = self.cli('--bag', bag, '--auto-segment', '--confirm',
                          *self.output_args(), expected=1)
        self.assertIn('Calibration unconfirmed', result.stdout)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertFalse(report['calibration_confirmed'])
        self.assertEqual(report['observed_frames']['odometry_child'], ['base_footprint'])
        self.assertIn('observed odometry child_frame_id does not match configured odom_child_frame',
                      report['reasons'])

    def test_cli_does_not_confirm_when_output_write_fails(self):
        for flag, path, other_output in (
            ('--output-mount', self.mount_path, ('--output-backend-body', self.backend_path)),
            ('--output-backend-body', self.backend_path, ('--output-mount', self.mount_path)),
        ):
            with self.subTest(flag=flag):
                missing_path = self.root / 'nonexistent_dir' / path.name
                result = self.cli('--bag', self.bag, '--auto-segment', '--confirm',
                                  flag, missing_path, *other_output,
                                  '--output-report', self.report_path, expected=1)
                self.assertIn('Calibration unconfirmed', result.stdout)
                self.assertNotIn('Calibration confirmed', result.stdout + result.stderr)
                report = yaml.safe_load(self.report_path.read_text())
                self.assertFalse(report['calibration_confirmed'])
                self.assertTrue(any(str(missing_path) in reason for reason in report['reasons']),
                                report['reasons'])
                if flag == '--output-backend-body':
                    self.assertFalse(yaml.safe_load(self.mount_path.read_text())['calibration_confirmed'])
                else:
                    self.assertFalse(self.backend_path.exists())

    def test_report_write_failure_leaves_no_new_mount_or_backend(self):
        missing_report = self.root / 'nonexistent_dir' / 'report.yaml'
        result = self.cli('--bag', self.bag, '--auto-segment', '--confirm',
                          '--output-mount', self.mount_path, '--output-backend-body', self.backend_path,
                          '--output-report', missing_report, expected=1)
        self.assertIn('Calibration unconfirmed', result.stdout)
        self.assertIn(str(missing_report), result.stderr)
        self.assertFalse(self.backend_path.exists())
        self.assertFalse(self.mount_path.exists())
        self.assertEqual(list(self.root.glob('*.yaml.*')), [])

    def test_auto_segment_uses_acquisition_time_for_jittered_odometry(self):
        # At 100 ms, the longer reception interval implies 0.1125 m/s. At 40 ms,
        # the delay also reverses the reception order of adjacent samples.
        for odom_stride in (20, 8):
            with self.subTest(odom_period_ms=odom_stride * 5):
                bag = self.root / f'jittered_mcap_{odom_stride}'
                write_bag(bag, straight_speed=.18, odom_stride=odom_stride, straight_odom_jitter=.06)
                modes = {'auto': ('--bag', bag, '--auto-segment')}
                if odom_stride == 8:
                    modes['windows'] = ('--bag', bag, '--stationary-window', '0:3',
                                        '--straight-window', '3:20', '--spin-window', '20:26')
                    straight_bag = self.root / 'jittered_straight'
                    spin_bag = self.root / 'jittered_spin'
                    for stage_bag, window in ((straight_bag, (3, 20)), (spin_bag, (20, 26))):
                        write_bag(stage_bag, straight_speed=.18, odom_stride=odom_stride,
                                  straight_odom_jitter=.06, reception_window=window)
                    modes['separate bags'] = ('--stationary-bag', self.stationary_bag,
                                              '--straight-bag', straight_bag, '--spin-bag', spin_bag)
                for mode, args in modes.items():
                    with self.subTest(mode=mode):
                        result = self.cli(*args, '--confirm', *self.output_args())
                        self.assertIn('Calibration confirmed', result.stdout)
                        _, angle, distance = self.assert_mount_accuracy(self.mount_path)
                        report = yaml.safe_load(self.report_path.read_text())
                        self.assertTrue(report['calibration_confirmed'])
                        self.assertGreater(report['straight']['distance_m'], 2.95)
                        self.assertEqual(report['rejected_messages']['odometry'], 0)
                        print(f'Jittered MCAP ({odom_stride * 5} ms odometry, {mode}): '
                              f'rotation error {angle:.6f} deg, '
                              f'translation error {distance * 1000:.4f} mm', flush=True)

    def test_auto_segment_uses_acquisition_time_for_delayed_lidar(self):
        for lag, offset in ((3.5, 0.), (1., .6)):
            with self.subTest(lidar_lag=lag, moving_lidar_z_offset=offset):
                bag = self.root / f'delayed_lidar_{lag}'
                write_bag(bag, lidar_lag=lag, moving_lidar_z_offset=offset)
                result = self.cli('--bag', bag, '--auto-segment', '--confirm', *self.output_args())
                self.assertIn('Calibration confirmed', result.stdout)
                self.assert_mount_accuracy(self.mount_path)
                solved = yaml.safe_load(self.report_path.read_text())
                self.assertTrue(solved['calibration_confirmed'])
                result = self.cli('--verify-mount', self.mount_path, '--bag', bag, '--auto-segment',
                                  '--output-report', self.report_path)
                self.assertIn('Verification passed', result.stdout)
                verified = yaml.safe_load(self.report_path.read_text())
                self.assertTrue(verified['verification_passed'], verified['reasons'])
                for report in (solved, verified):
                    self.assertEqual(report['captured_messages']['lidar'], 31)
                    self.assertEqual(report['rejected_messages']['lidar'], 0)
                    self.assertEqual(report['capture_intervals_header_sec']['lidar'],
                                     [EPOCH_NS / 10**9, EPOCH_NS / 10**9 + 3.])
                    np.testing.assert_allclose(report['windows_sec']['stationary'], [0., 3. + lag])

    def test_millig_with_delayed_odometry_auto_segments_and_verifies(self):
        scale = .00981
        bag = self.root / 'millig_mcap'
        write_bag(bag, scale=scale, odom_lag=.25)
        self.cli('--bag', bag, '--auto-segment', '--imu-scale', scale, '--confirm',
                 *self.output_args())
        self.assert_mount_accuracy(self.mount_path, scale)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertTrue(report['calibration_confirmed'])
        start, end = report['windows_sec']['stationary']
        self.assertAlmostEqual(start, 0.)
        self.assertLess(end, 3.)
        self.assertGreaterEqual(end - start, 2.)
        self.assertAlmostEqual(report['windows_sec']['straight'][0], 3.25)
        self.assertAlmostEqual(report['windows_sec']['spin'][0], 7.25)
        verified = self.cli('--verify-mount', self.mount_path, '--bag', bag, '--auto-segment',
                            '--output-report', self.report_path)
        self.assertIn('Verification passed', verified.stdout)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertTrue(report['verification_passed'], report['reasons'])
        self.assertEqual(report['thresholds']['imu_scale_requested'], scale)

    def test_fastwrite_mcap_sorts_out_of_order_records_and_counts_duplicates(self):
        bag = self.root / 'unordered_mcap'
        write_bag(bag, fastwrite=True, unordered=True)
        self.cli('--bag', self.bag, '--auto-segment', '--confirm', *self.output_args())
        reference = yaml.safe_load(self.report_path.read_text())
        selected = reference['windows_sec']
        manual = []
        for stage in ('stationary', 'straight', 'spin'):
            start, end = selected[stage]
            manual.extend((f'--{stage}-window', f'{start}:{end}'))
        for mode in (manual, ['--auto-segment']):
            with self.subTest(mode=mode):
                self.cli('--bag', bag, *mode, '--confirm', *self.output_args())
                self.assert_mount_accuracy(self.mount_path)
                report = yaml.safe_load(self.report_path.read_text())
                self.assertEqual(report['captured_messages'], reference['captured_messages'])
                self.assertEqual(report['capture_intervals_header_sec'],
                                 reference['capture_intervals_header_sec'])
                self.assertEqual(report['rejected_messages'], {'imu': 1, 'lidar': 1, 'odometry': 1})
                self.assertEqual(report['windows_sec'], selected)

    def test_explicit_window_rejects_stale_lidar_outside_stationary_imu_interval(self):
        bag = self.root / 'stale_stationary_lidar'
        write_bag(bag, stale_stationary_lidar=True)
        windows = ('--stationary-window', '3:6', '--straight-window', '6:10',
                   '--spin-window', '10:16')
        result = self.cli('--bag', bag, *windows, '--confirm', *self.output_args(), expected=1)
        self.assertIn('Calibration unconfirmed', result.stdout)
        self.assertFalse(self.backend_path.exists())
        report = yaml.safe_load(self.report_path.read_text())
        self.assertFalse(report['stationary']['passed'])
        self.assertEqual(report['stationary']['clouds'], 0)
        mount = yaml.safe_load(self.mount_path.read_text())
        self.assertFalse(mount['calibration_confirmed'])
        self.assertIsNone(mount['base_to_sensor']['xyz'][2])

        confirmed_mount = self.root / 'confirmed_mount.yaml'
        self.cli('--bag', self.bag, '--auto-segment', '--confirm',
                 '--output-mount', confirmed_mount)
        fresh_backend = self.root / 'fresh_backend.yaml'
        result = self.cli('--verify-mount', confirmed_mount, '--bag', bag, *windows,
                          '--confirm', '--output-backend-body', fresh_backend,
                          '--output-report', self.report_path, expected=1)
        self.assertIn('Verification failed', result.stdout)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertFalse(report['verification_passed'])
        self.assertEqual(report['stationary']['clouds'], 0)
        self.assertFalse(fresh_backend.exists())

    def test_stationary_imu_outage_is_rejected_in_solve_and_verify(self):
        bag = self.root / 'stationary_imu_outage'
        write_bag(bag, stationary_imu_outage=True)
        window = ('--stationary-window', '0:3')
        reason = 'stationary IMU stream contains a sample gap'
        result = self.cli('--bag', bag, *window, '--yaw-deg', '135',
                          '--base-to-sensor-x', '.22', '--base-to-sensor-y', '-.11',
                          '--confirm', *self.output_args(), expected=1)
        self.assertIn('Calibration unconfirmed', result.stdout)
        self.assertFalse(self.backend_path.exists())
        report = yaml.safe_load(self.report_path.read_text())
        self.assertIs(report['stationary']['passed'], False)
        self.assertIn(reason, report['stationary']['reasons'])
        self.assertAlmostEqual(report['stationary']['max_imu_gap_sec'], 2.)
        self.assertIsNone(report['base_from_imu']['xyz_m'][2])
        mount = yaml.safe_load(self.mount_path.read_text())
        self.assertFalse(mount['calibration_confirmed'])
        self.assertIsNone(mount['base_to_sensor']['xyz'][2])

        confirmed_mount = self.root / 'confirmed_mount.yaml'
        self.cli('--bag', self.bag, '--auto-segment', '--confirm',
                 '--output-mount', confirmed_mount)
        fresh_backend = self.root / 'fresh_backend.yaml'
        result = self.cli('--verify-mount', confirmed_mount, '--bag', bag, *window,
                          '--straight-window', '3:7', '--spin-window', '7:13',
                          '--confirm', '--output-backend-body', fresh_backend,
                          '--output-report', self.report_path, expected=1)
        self.assertIn('Verification failed', result.stdout)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertIs(report['verification_passed'], False)
        self.assertIs(report['stationary']['passed'], False)
        self.assertIn(reason, report['stationary']['reasons'])
        self.assertAlmostEqual(report['stationary']['max_imu_gap_sec'], 2.)
        self.assertFalse(fresh_backend.exists())

    def test_motion_odometry_dropout_is_rejected_in_solve_and_verify(self):
        bag = self.root / 'motion_odometry_dropout'
        write_bag(bag, spin_odom_dropout=True)
        windows = ('--stationary-window', '0:3', '--straight-window', '3:7',
                   '--spin-window', '7:15')
        reason = 'spin odometry stream contains a sample gap'
        result = self.cli('--bag', bag, *windows, '--confirm', *self.output_args(), expected=1)
        self.assertIn('Calibration unconfirmed', result.stdout)
        self.assertFalse(self.backend_path.exists())
        report = yaml.safe_load(self.report_path.read_text())
        self.assertIs(report['calibration_confirmed'], False)
        self.assertEqual(report['base_from_imu']['xyz_m'][:2], [None, None])
        self.assertEqual(yaml.safe_load(self.mount_path.read_text())['base_to_sensor']['xyz'][:2],
                         [None, None])
        self.assertIn(reason, report['spin']['reasons'])
        self.assertGreater(report['spin']['max_sample_gap_sec'], .5)

        confirmed_mount = self.root / 'confirmed_mount.yaml'
        self.cli('--bag', self.bag, '--auto-segment', '--confirm',
                 '--output-mount', confirmed_mount)
        fresh_backend = self.root / 'fresh_backend.yaml'
        result = self.cli('--verify-mount', confirmed_mount, '--bag', bag, *windows,
                          '--confirm', '--output-backend-body', fresh_backend,
                          '--output-report', self.report_path, expected=1)
        self.assertIn('Verification failed', result.stdout)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertIs(report['verification_passed'], False)
        self.assertIn(reason, report['spin']['reasons'])
        self.assertGreater(report['spin']['max_sample_gap_sec'], .5)
        self.assertFalse(fresh_backend.exists())

    def test_split_mcap_with_overlapping_files_keeps_all_in_window_samples(self):
        split = self.root / 'split_mcap'
        reference = self.root / 'unsplit_mcap'
        write_bag(split, split=True)
        write_bag(reference)
        self.assertEqual(len(list(split.glob('*.mcap'))), 2)
        stationary = ('--stationary-window', '0:2.5', '--min-stationary-duration', '2.45')
        windows = (*stationary, '--straight-window', '3:7', '--spin-window', '7:13')
        self.cli('--bag', reference, *windows, '--confirm', *self.output_args())
        expected = yaml.safe_load(self.report_path.read_text())
        self.assertEqual(expected['captured_messages']['imu'], 501)
        result = self.cli('--bag', split, *windows, '--confirm', *self.output_args())
        self.assertIn('Calibration confirmed', result.stdout)
        self.assert_mount_accuracy(self.mount_path)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertEqual(report['captured_messages'], expected['captured_messages'])
        self.assertEqual(report['rejected_messages'], {'imu': 0, 'lidar': 0, 'odometry': 0})

        # With only a stationary window, the end of file 0 lies past the capture end.
        self.cli('--bag', split, *stationary, '--output-report', self.report_path)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertTrue(report['stationary']['passed'], report['stationary'])
        self.assertEqual(report['captured_messages']['imu'], 501)
        self.assertEqual(report['rejected_messages'], {'imu': 0, 'lidar': 0, 'odometry': 0})

        result = self.cli('--bag', split, '--auto-segment', '--confirm', *self.output_args())
        self.assertIn('Calibration confirmed', result.stdout)
        self.assert_mount_accuracy(self.mount_path)

    @contextmanager
    def process(self, args, label):
        log_path = self.root / f'{label}.log'
        with log_path.open('w') as log:
            process = subprocess.Popen(list(map(str, args)), env=self.env, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                yield
                self.assertIsNone(process.poll(), log_path.read_text())
            except Exception as exc:
                exc.add_note(f'{label} output:\n{log_path.read_text()}')
                raise
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)

    def wait_for(self, condition, label, timeout=8.):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.02)
        self.assertTrue(condition(), label)

    def call_service(self, service_type, name, request):
        client = self.node.create_client(service_type, f'/rigid_mount_calibration/{name}')
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=8.), name)
            future = client.call_async(request)
            self.wait_for(future.done, name)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def trigger(self, name, success=True):
        response = self.call_service(Trigger, name, Trigger.Request())
        self.assertEqual(response.success, success, response.message)
        return response.message

    def step(self, duration=.006):
        deadline = time.monotonic() + duration
        while (remaining := deadline - time.monotonic()) > 0:
            rclpy.spin_once(self.node, timeout_sec=min(remaining, .004))

    def test_online_services_export_into_production_closed_loop(self):
        with patch.dict(os.environ, self.env):
            rclpy.init(domain_id=self.domain)
            self.addCleanup(rclpy.try_shutdown)
            self.node = rclpy.create_node('rigid_calibration_integration_test')
        self.addCleanup(self.node.destroy_node)
        imu_pub = self.node.create_publisher(Imu, IMU, qos_profile_sensor_data)
        lidar_pub = self.node.create_publisher(PointCloud2, LIDAR, qos_profile_sensor_data)
        odom_pub = self.node.create_publisher(Odometry, ODOM, qos_profile_sensor_data)
        backend_path = self.root / 'backend' / self.backend_path.name
        args = ['ros2', 'launch', 'xxu_bringup', 'rigid_mount_calibration.launch.py',
                'start_zlio:=false', 'confirm:=false', f'output_mount_file:={self.mount_path}',
                f'output_backend_body_file:={backend_path}',
                f'output_report_file:={self.report_path}']

        def assert_confirmation(value):
            response = self.call_service(GetParameters, 'get_parameters',
                                         GetParameters.Request(names=['confirm']))
            self.assertEqual(response.values[0].bool_value, value)

        def request_confirmation():
            request = SetParameters.Request(parameters=[Parameter('confirm', value=True).to_parameter_msg()])
            response = self.call_service(SetParameters, 'set_parameters', request)
            self.assertTrue(response.results[0].successful, response.results[0].reason)
            assert_confirmation(True)

        def assert_unconfirmed_outputs():
            for path in (self.mount_path, self.report_path):
                self.assertFalse(yaml.safe_load(path.read_text())['calibration_confirmed'], path)

        def publish_spin():
            for tick in range(121):
                t = EPOCH_NS + 9_000_000_000 + tick * 40_000_000
                odom_pub.publish(odom_message(t, SPIN_CENTER, tick * math.tau / 120))
                self.step()

        with self.process(args, 'calibration'):
            self.wait_for(lambda: all(p.get_subscription_count() for p in (imu_pub, lidar_pub, odom_pub)),
                          'calibration input subscriptions')
            self.assertEqual(self.node.get_subscriptions_info_by_topic('/tf'), [])
            self.trigger('start_stationary')
            cloud = make_cloud(EPOCH_NS, clutter=True)
            for tick in range(301):
                timestamp = EPOCH_NS + tick * 10_000_000
                imu_pub.publish(imu_message(timestamp))
                if tick % 10 == 0:
                    lidar_pub.publish(stamp(cloud, timestamp, 'livox_frame'))
                self.step()
            self.trigger('stop_stationary')
            self.assertTrue(self.trigger('solve_and_export').startswith('calibration unconfirmed'))
            assert_unconfirmed_outputs()
            self.assertFalse(backend_path.exists())
            self.trigger('start_straight')
            for tick in range(101):
                t = EPOCH_NS + 4_000_000_000 + tick * 40_000_000
                odom_pub.publish(odom_message(t, np.array([tick * .03, 0., 0.]), 0.))
                self.step()
            self.trigger('stop_straight')
            self.trigger('start_spin')
            publish_spin()
            self.trigger('stop_spin')
            self.assertEqual(self.trigger('solve_and_export'), 'calibration unconfirmed')
            assert_unconfirmed_outputs()
            self.assertFalse(backend_path.exists())
            request_confirmation()
            response = self.trigger('solve_and_export', success=False)
            self.assertTrue(response.startswith('calibration unconfirmed'), response)
            self.assertIn(str(backend_path), response)
            assert_unconfirmed_outputs()
            self.assertFalse(backend_path.exists())
            backend_path.parent.mkdir()
            self.assertEqual(self.trigger('solve_and_export'), 'calibration confirmed')
            mount, _, _ = self.assert_mount_accuracy(self.mount_path)
            self.assertTrue(yaml.safe_load(self.report_path.read_text())['calibration_confirmed'])
            confirmed_mount_text = self.mount_path.read_text()
            confirmed_backend_text = backend_path.read_text()
            body = yaml.safe_load(confirmed_backend_text)['common']['body']
            self.assertEqual(body['frame'], 'base_link')
            np.testing.assert_allclose(list(body['offset'].values()), -LEVER, atol=.005)

            assert_confirmation(True)
            self.trigger('start_spin')
            assert_confirmation(False)
            publish_spin()
            request_confirmation()
            self.trigger('stop_spin')
            assert_confirmation(False)
            self.assertEqual(self.trigger('solve_and_export'), 'calibration unconfirmed')
            assert_unconfirmed_outputs()
            self.assertEqual(backend_path.read_text(), confirmed_backend_text)

            request_confirmation()
            self.trigger('reset')
            assert_confirmation(False)
            self.assertTrue(self.trigger('solve_and_export').startswith('calibration unconfirmed'))
            assert_unconfirmed_outputs()
            self.assertEqual(backend_path.read_text(), confirmed_backend_text)

        confirmed_mount_path = self.root / 'confirmed_mount.yaml'
        confirmed_mount_path.write_text(confirmed_mount_text)
        compensated_imu, compensated_cloud, odometry = [], [], []
        compensated_imu_topic = '/livox/imu_lio_compensated'
        compensated_lidar_topic = '/livox/lidar_compensated'
        subscriptions = [
            self.node.create_subscription(Imu, compensated_imu_topic,
                                          compensated_imu.append, qos_profile_sensor_data),
            self.node.create_subscription(PointCloud2, compensated_lidar_topic,
                                          compensated_cloud.append, qos_profile_sensor_data),
            self.node.create_subscription(Odometry, '/odom', odometry.append, qos_profile_sensor_data),
        ]
        raw_odom_pub = self.node.create_publisher(Odometry, '/odometry', qos_profile_sensor_data)
        args = ['ros2', 'launch', 'xxu_bringup', 'rigid_sensor.launch.py',
                f'mount_file:={confirmed_mount_path}', 'use_sim_time:=false', 'publish_mount_tf:=true',
                'start_base_adapter:=true', 'source_child_frame:=imu']
        with self.process(args, 'rigid_sensor'):
            self.wait_for(lambda: all(p.get_subscription_count()
                                      for p in (imu_pub, lidar_pub, raw_odom_pub)),
                          'production input subscriptions')
            cloud = make_cloud(EPOCH_NS, clutter=False)

            def publish_probe(tick, yaw, cloud_due):
                timestamp = EPOCH_NS + 16_000_000_000 + tick * 40_000_000
                imu_pub.publish(imu_message(timestamp))
                if cloud_due:
                    lidar_pub.publish(stamp(cloud, timestamp, 'livox_frame'))
                # The production frontend reports poses in the compensated virtual axes.
                raw_odom_pub.publish(odom_message(timestamp, SPIN_CENTER, yaw, physical=False))
                self.step(.018)

            # DDS discovery of the static transforms is asynchronous. Check the
            # whole pipeline is ready before collecting the full-turn trajectory.
            for warmup_tick in range(100):
                publish_probe(warmup_tick, 0., cloud_due=warmup_tick % 4 == 0)
                if compensated_imu and compensated_cloud and odometry:
                    break
            self.assertTrue(compensated_imu and compensated_cloud and odometry,
                            f'production warmup counts: {len(compensated_imu)}, '
                            f'{len(compensated_cloud)}, {len(odometry)}')
            compensated_imu.clear()
            compensated_cloud.clear()
            odometry.clear()
            for tick in range(121):
                publish_probe(warmup_tick + 1 + tick, tick * math.tau / 120, cloud_due=tick % 8 == 0)
            self.assertGreaterEqual(len(compensated_imu), 50)
            self.assertGreaterEqual(len(compensated_cloud), 3)
            self.assertGreaterEqual(len(odometry), 60)
            angles = np.unwrap([2 * math.atan2(message.pose.pose.orientation.z,
                                               message.pose.pose.orientation.w)
                                for message in odometry])
            self.assertGreater(np.ptp(angles), math.radians(330))
            gravity = compensated_imu[-1].linear_acceleration
            gravity_error = np.linalg.norm(np.array([gravity.x, gravity.y, gravity.z]) - [0, 0, 9.81])
            self.assertLess(gravity_error, .15)
            cloud = compensated_cloud[-1]
            self.assertEqual(cloud.header.frame_id, mount['frames']['sensor'])
            # base_aligned defines identity axes. Fit the floor in C++ from the captured messages.
            self.assertEqual(mount['base_to_sensor']['rpy'], [0., 0., 0.])
            compensated_bag = self.root / 'compensated_probe_bag'
            writer = SequentialWriter()
            writer.open(StorageOptions(uri=str(compensated_bag), storage_id='mcap'),
                        ConverterOptions('cdr', 'cdr'))
            records = []
            for topic, msg_type, messages in (
                    (compensated_imu_topic, 'sensor_msgs/msg/Imu', compensated_imu),
                    (compensated_lidar_topic, 'sensor_msgs/msg/PointCloud2', compensated_cloud)):
                writer.create_topic(TopicMetadata(id=0, name=topic, type=msg_type,
                                                  serialization_format='cdr'))
                for message in messages:
                    header_stamp = message.header.stamp
                    timestamp = header_stamp.sec * 10**9 + header_stamp.nanosec
                    records.append((timestamp, topic, message))
            for timestamp, topic, message in sorted(records, key=lambda record: record[0]):
                writer.write(topic, serialize_message(message), timestamp)
            writer.close()

            probe_report = self.root / 'compensated_probe_report.yaml'
            self.cli('--bag', compensated_bag, '--stationary-window', '0:30',
                     '--imu-topic', compensated_imu_topic, '--lidar-topic', compensated_lidar_topic,
                     '--imu-source-frame', 'lio_base_sensor', '--lidar-frame', 'lio_base_sensor',
                     '--sensor-frame', 'compensated_probe_sensor',
                     '--imu-from-lidar-xyz', '0', '0', '0', '--imu-from-lidar-rpy', '0', '0', '0',
                     '--min-imu-samples', '50', '--min-ground-clouds', '3',
                     '--output-report', probe_report)
            probe = yaml.safe_load(probe_report.read_text())
            self.assertTrue(probe['stationary']['passed'], probe['stationary']['reasons'])
            floor = probe['stationary']['floor']
            self.assertGreaterEqual(floor['inliers'], 200)
            floor_error = abs(floor['solved_tz_m'] - mount['base_to_sensor']['xyz'][2])
            self.assertLess(floor_error, .005)
            self.assertLess(floor['rms_m'], .005)
            self.assertEqual({message.child_frame_id for message in odometry}, {'base_link'})
            xy = np.array([(message.pose.pose.position.x, message.pose.pose.position.y)
                           for message in odometry])
            center_rms = float(np.sqrt(np.mean(np.sum((xy - SPIN_CENTER[:2])**2, axis=1))))
            self.assertLess(center_rms, .005)
            print(f'Production C++: {len(odometry)} odometry outputs, gravity error '
                  f'{gravity_error:.6f} m/s², floor height error {floor_error * 1000:.4f} mm, '
                  f'spin center RMS {center_rms * 1000:.4f} mm', flush=True)
        for sub in subscriptions:
            self.node.destroy_subscription(sub)

    def test_stationary_only_sqlite_mount_is_rejected_and_confirm_exits_nonzero(self):
        base_args = ('--stationary-bag', self.stationary_bag, *self.output_args())
        self.cli(*base_args)
        self.assertFalse(self.backend_path.exists())
        partial = yaml.safe_load(self.mount_path.read_text())
        self.assertFalse(partial['calibration_confirmed'])
        self.assertEqual(partial['base_to_sensor']['xyz'][:2], [None, None])
        self.assertAlmostEqual(partial['base_to_sensor']['xyz'][2], LEVER[2], delta=.005)
        self.assertEqual(partial['sensor_from_imu_rpy'], [None, None, None])
        self.assertIn('[null, null,', self.mount_path.read_text())
        with self.assertRaisesRegex(ValueError, 'pending'):
            load_rigid_mount(self.mount_path)
        self.cli(*base_args, '--confirm', expected=1)
        self.assertFalse(self.backend_path.exists())
        self.assertFalse(yaml.safe_load(self.report_path.read_text())['calibration_confirmed'])
        self.cli(*base_args, '--imu-topic', '/missing_imu_input', expected=1)
        failure_report = yaml.safe_load(self.report_path.read_text())
        self.assertFalse(failure_report['calibration_confirmed'])
        self.assertIn('bag missing topic /missing_imu_input', failure_report['input_errors'][0])

    def test_stationary_only_verification_cannot_confirm_wrong_yaw_mount(self):
        self.cli('--bag', self.bag, '--auto-segment', '--confirm', *self.output_args())
        original_backend = self.backend_path.read_text()
        mount = yaml.safe_load(self.mount_path.read_text())
        mount['sensor_from_imu_rpy'][2] += math.pi / 2
        wrong_mount = self.root / 'wrong_yaw.yaml'
        wrong_mount.write_text(yaml.safe_dump(mount))
        result = self.cli('--verify-mount', wrong_mount, '--stationary-bag', self.stationary_bag,
                          '--confirm', *self.output_args(), expected=1)
        self.assertIn('Verification failed', result.stdout)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertTrue(report['stationary']['passed'])
        self.assertFalse(report['verification_passed'])
        self.assertFalse(report['calibration_confirmed'])
        self.assertFalse(yaml.safe_load(self.mount_path.read_text())['calibration_confirmed'])
        self.assertCountEqual(report['reasons'], [
            'straight motion verification was not executed',
            'spin motion verification was not executed',
        ])
        self.assertEqual(self.backend_path.read_text(), original_backend)
        fresh_backend = self.root / 'fresh_backend.yaml'
        result = self.cli('--verify-mount', wrong_mount, '--stationary-bag', self.stationary_bag,
                          '--confirm', '--output-backend-body', fresh_backend,
                          '--output-report', self.report_path, expected=1)
        self.assertIn('Verification failed', result.stdout)
        self.assertFalse(fresh_backend.exists())

    def test_real_custommsg_bag_read_only(self):
        real_bag = WORKSPACE / 'bags/mid360s_corridor_test_20261115_1900'
        if not (real_bag / 'metadata.yaml').is_file():
            self.skipTest('real bag is unavailable')
        result = self.cli('--bag', real_bag, '--stationary-window', '0:3',
                          '--output-report', self.report_path, expected=None)
        if 'rebuild with livox_ros_driver2 available' in result.stderr:
            self.skipTest('xxu_bringup compiled without Livox CustomMsg support')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = yaml.safe_load(self.report_path.read_text())
        self.assertEqual(report['imu_acceleration_scale'], 9.81)
        self.assertFalse(report['calibration_confirmed'])
        self.assertGreaterEqual(report['captured_messages']['lidar'], 3)
        self.assertGreaterEqual(report['captured_messages']['imu'], 100)
        print(f"Read-only CustomMsg bag: {report['captured_messages']}, IMU scale 9.81, "
              f"stationary passed={report['stationary']['passed']}", flush=True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
