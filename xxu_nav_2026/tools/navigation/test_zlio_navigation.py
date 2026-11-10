#!/usr/bin/env python3
"""Candidate cloud contract and localization-gate regressions; source ROS first."""
import copy
import importlib.util
from pathlib import Path
import struct
import unittest
from unittest.mock import Mock
from types import SimpleNamespace

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointField
from localization_interfaces.msg import FrontendFrame, FrontendSession

ROOT = Path(__file__).resolve().parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


watchdog = load('candidate_watchdog', ROOT / 'src/fake_vel_transform/scripts/cmd_vel_watchdog.py')
adapter = load('candidate_adapter', ROOT / 'src/xxu_bringup/scripts/zlio_base_odometry.py')


def pose(stamp):
    m = Odometry()
    m.header.frame_id = 'map'
    m.child_frame_id = 'imu'
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 1000000000)
    m.pose.pose.orientation.w = 1.
    return m


def state(stamp, status='NORMAL', epoch=0, generation=2, revision=1):
    trusted = status in ('NORMAL', 'OBS_DEGRADED')
    return (f'status={status};pose_trusted={str(trusted).lower()};tf_frozen={str(not trusted).lower()}'
            f';epoch={epoch};generation={generation};pose_revision={revision};stamp_ns={stamp}')


class LocalizationHealthTest(unittest.TestCase):
    def setUp(self):
        self.guard = watchdog.LocalizationHealth(.5, .2)

    def feed(self, t, **kwargs):
        stamp = 100000000000 + round(t*1e9)
        self.guard.observe_state(state(stamp, **kwargs), t)
        self.guard.observe_pose(pose(stamp), t)
        return self.guard.check(t, stamp)[0]

    def test_startup_hold_fault_and_recovery(self):
        self.assertFalse(self.guard.check(0, 100000000000)[0])
        self.assertFalse(self.feed(0))
        self.assertFalse(self.feed(.1))
        self.assertTrue(self.feed(.21))
        self.assertFalse(self.feed(.3, status='SUSPECT', generation=3))
        self.assertFalse(self.feed(.4, generation=4))
        self.assertFalse(self.feed(.5, generation=4))
        self.assertTrue(self.feed(.61, generation=4))
        self.assertFalse(self.feed(.7, status='LOST', generation=5))

    def test_repeat_heartbeats_do_not_renew_progress_with_paused_ros_clock(self):
        self.feed(0)
        for t in (.2, .4, .6):
            self.guard.observe_state(state(100000000000), t)
            self.guard.observe_pose(pose(100000000000), t)
        self.assertFalse(self.guard.check(.6, 100000000000)[0])

    def test_zero_stamp_notices_cannot_renew_old_state(self):
        self.feed(0)
        self.guard.observe_state(state(0, revision=2), .6)
        self.guard.observe_pose(pose(100600000000), .6)
        self.assertFalse(self.guard.check(.6, 100600000000)[0])

    def test_state_and_pose_source_ages_are_checked(self):
        self.feed(0)
        self.assertFalse(self.guard.check(.1, 102000000000)[0])
        self.guard = watchdog.LocalizationHealth(.5, 0)
        self.guard.observe_state(state(110000000000), 1)
        self.guard.observe_pose(pose(110000000000), 1)
        self.assertFalse(self.guard.check(1, 100000000000)[0])

    def test_identity_change_requires_new_pose_and_hold(self):
        self.feed(0)
        self.assertTrue(self.feed(.21))
        self.guard.observe_state(state(100300000000, epoch=1, generation=4), .3)
        self.assertFalse(self.guard.check(.3, 100300000000)[0])
        self.assertFalse(self.feed(.31, epoch=1, generation=4))
        self.assertTrue(self.feed(.52, epoch=1, generation=4))
        self.assertFalse(self.feed(.6, epoch=0, generation=2))

    def test_malformed_state_and_nonfinite_pose_close(self):
        for text in ('status=NORMAL', state(100000000000)+';epoch=0', state(-1),
                     state(100000000000).replace('NORMAL', 'UNKNOWN')):
            with self.subTest(text=text):
                self.feed(0)
                self.guard.observe_state(text, .1)
                self.assertFalse(self.guard.check(.1, 100000000000)[0])
        self.feed(.2)
        m = pose(100300000000)
        m.pose.pose.position.x = float('nan')
        self.guard.observe_pose(m, .3)
        self.assertFalse(self.guard.check(.3, 100300000000)[0])


class CommandRecoveryTest(unittest.TestCase):
    def test_command_cached_during_fault_or_hold_cannot_release(self):
        from geometry_msgs.msg import TwistStamped
        clock = SimpleNamespace(nanoseconds=100000000000)
        guard = watchdog.LocalizationHealth(.5, 0)
        node = SimpleNamespace(require_localization=True, localization_ready=False,
                               localization_health=guard, last_cmd=TwistStamped(),
                               last_cmd_receive_time=0., timeout=.3,
                               require_odom=False, require_scan=False, require_joint_states=False, require_tf=False,
                               get_clock=lambda: SimpleNamespace(now=lambda: clock))
        node._fresh = watchdog.CmdVelWatchdog._fresh

        def refresh(t, status='NORMAL', generation=2):
            clock.nanoseconds = 100000000000+round(t*1e9)
            guard.observe_state(state(clock.nanoseconds, status=status, generation=generation), t)
            guard.observe_pose(pose(clock.nanoseconds), t)
            node.last_cmd.header.stamp.sec, node.last_cmd.header.stamp.nanosec = divmod(clock.nanoseconds, 1000000000)
            node.last_cmd_receive_time = t

        refresh(0)
        self.assertFalse(watchdog.CmdVelWatchdog._health(node, 0)[0])
        self.assertIsNone(node.last_cmd_receive_time)
        refresh(.1)
        self.assertTrue(watchdog.CmdVelWatchdog._health(node, .1)[0])
        refresh(.2, 'SUSPECT', 3)
        self.assertFalse(watchdog.CmdVelWatchdog._health(node, .2)[0])
        self.assertIsNone(node.last_cmd_receive_time)
        refresh(.3, generation=4)
        self.assertFalse(watchdog.CmdVelWatchdog._health(node, .3)[0])
        self.assertIsNone(node.last_cmd_receive_time)
        refresh(.31, generation=4)
        self.assertTrue(watchdog.CmdVelWatchdog._health(node, .31)[0])


class FrameCloudTest(unittest.TestCase):
    def setUp(self):
        self.bridge = adapter.FrameCloudBridge('lio_base_sensor', 'imu', 'odom', .5)
        self.session = FrontendSession(instance_id=7, epoch=0)
        self.frame = FrontendFrame(instance_id=7, epoch=0, sequence=1, lidar_updated=True)
        self.frame.odometry = pose(100000000000)
        self.frame.odometry.header.frame_id = 'odom'
        c = self.frame.cloud
        c.header.stamp = copy.deepcopy(self.frame.odometry.header.stamp)
        c.header.frame_id = 'imu'
        c.width, c.height, c.point_step, c.row_step = 2, 1, 16, 32
        c.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1) for i,n in enumerate(('x','y','z','intensity'))]
        c.data = struct.pack('<8f', 1,2,3,10, 4,5,6,20)

    def test_payload_fields_and_stamp_are_preserved(self):
        self.bridge.session(self.session)
        out = self.bridge.convert(self.frame, 100010000000)
        self.assertEqual(out.data, self.frame.cloud.data)
        self.assertEqual(out.fields, self.frame.cloud.fields)
        self.assertEqual(out.header.stamp, self.frame.cloud.header.stamp)
        self.assertEqual(out.header.frame_id, 'lio_base_sensor')
        self.assertEqual(self.frame.cloud.header.frame_id, 'imu')
        with self.assertRaises(ValueError): self.bridge.convert(self.frame, 100010000000)

    def test_requires_healthy_matching_session(self):
        with self.assertRaises(ValueError): self.bridge.convert(self.frame, 100010000000)
        self.bridge.session(self.session)
        self.session.reset_required = True
        self.bridge.session(self.session)
        with self.assertRaises(ValueError): self.bridge.convert(self.frame, 100010000000)
        self.session.reset_required = False
        self.session.epoch = 2
        self.bridge.session(self.session)
        with self.assertRaises(ValueError): self.bridge.convert(self.frame, 100010000000)

    def test_malformed_stale_and_nonfinite_clouds_are_rejected(self):
        self.bridge.session(self.session)
        variants = []
        m = copy.deepcopy(self.frame); m.cloud.header.frame_id = 'odom'; variants.append(m)
        m = copy.deepcopy(self.frame); m.cloud.row_step = 31; variants.append(m)
        m = copy.deepcopy(self.frame); m.cloud.fields[0].offset = 16; variants.append(m)
        m = copy.deepcopy(self.frame); m.cloud.data = struct.pack('<8f', float('nan'),2,3,10, 4,5,6,20); variants.append(m)
        m = copy.deepcopy(self.frame); m.lidar_updated = False; variants.append(m)
        for m in variants:
            with self.assertRaises(ValueError): self.bridge.convert(m, 100010000000)
        with self.assertRaises(ValueError): self.bridge.convert(self.frame, 101000000000)
        with self.assertRaises(ValueError): self.bridge.convert(self.frame, 99000000000)


class ExternalStartupTest(unittest.TestCase):
    def probe(self, trusted, transform=True):
        return SimpleNamespace(
            request_sent=False, client=Mock(), tf_buffer=Mock(),
            localization_health=Mock(), global_frame='map', odom_frame='odom',
            get_logger=Mock(), get_clock=Mock(), startup_complete=Mock())

    def test_tf_alone_cannot_activate_external_navigation(self):
        startup = load('external_startup', ROOT / 'src/xxu_bringup/scripts/nav2_navigation_startup.py')
        node = self.probe(False)
        node.client.service_is_ready.return_value = True
        node.tf_buffer.can_transform.return_value = True
        node.localization_health.check.return_value = (False, ['untrusted'])
        startup.NavigationStartup.try_start(node)
        node.client.call_async.assert_not_called()
        self.assertFalse(node.request_sent)
        node.localization_health.check.return_value = (True, [])
        startup.NavigationStartup.try_start(node)
        node.client.call_async.assert_called_once()
        self.assertTrue(node.request_sent)

    def test_trusted_state_still_requires_transform(self):
        startup = load('external_startup_tf', ROOT / 'src/xxu_bringup/scripts/nav2_navigation_startup.py')
        node = self.probe(True)
        node.client.service_is_ready.return_value = True
        node.localization_health.check.return_value = (True, [])
        node.tf_buffer.can_transform.return_value = False
        startup.NavigationStartup.try_start(node)
        node.client.call_async.assert_not_called()


if __name__ == '__main__': unittest.main()
