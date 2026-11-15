#!/usr/bin/env python3
"""Idle survey ownership and safety regressions; source ROS first."""
import importlib.util
from pathlib import Path
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
from action_msgs.msg import GoalStatus, GoalStatusArray

FILE = Path(__file__).resolve().parents[2] / 'src/xxu_bringup/scripts/nav2_navigation_startup.py'
spec = importlib.util.spec_from_file_location('idle_scan_startup', FILE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class IdleScanTests(unittest.TestCase):
    def setUp(self):
        self.scan = module.IdleScan.__new__(module.IdleScan)
        s = self.scan
        s.node = Mock()
        s.node.localization_health.check.return_value = (True, [])
        s.node.get_clock().now().nanoseconds = 1000000000
        s.node.count_publishers.return_value = 1
        s.clients = {'bt_navigator': Mock(), 'behavior_server': Mock()}
        s.states = {name: (3, time.monotonic()) for name in s.clients}
        s.status = {}
        s.goal = None
        s.goal_id = b'own'
        s.pending = None
        s.canceling = False
        s.idle_since = time.monotonic() - 3.
        s.next_attempt = 0.
        s.spin = Mock()
        s.spin.server_is_ready.return_value = True

    def test_another_navigation_goal_cancels_only_owned_spin(self):
        s = self.scan
        s.goal = Mock()
        s.status = {'navigate_to_pose': [(b'new', 2)]}
        s.tick()
        s.goal.cancel_goal_async.assert_called_once()
        s.spin.send_goal_async.assert_not_called()
        s.tick()
        s.goal.cancel_goal_async.assert_called_once()

    def test_pending_acceptance_is_canceled_if_navigation_started(self):
        s = self.scan
        s.pending = Mock()
        s.status = {'follow_path': [(b'path', 1)]}
        goal = Mock(accepted=True)
        s.accepted(Mock(result=Mock(return_value=goal)))
        goal.cancel_goal_async.assert_called_once()

    def test_own_action_status_does_not_interrupt_own_spin(self):
        s = self.scan
        s.status = {'spin': [(b'own', 2)]}
        s.goal = Mock()
        self.assertFalse(s.busy())
        s.tick()
        s.goal.cancel_goal_async.assert_not_called()

    def test_external_spin_is_busy(self):
        s = self.scan
        s.status = {'spin': [(b'external', 2)]}
        s.tick()
        s.spin.send_goal_async.assert_not_called()

    def test_untrusted_localization_cancels(self):
        s = self.scan
        s.node.localization_health.check.return_value = (False, ['LOST'])
        s.goal = Mock()
        s.tick()
        s.goal.cancel_goal_async.assert_called_once()
        s.spin.send_goal_async.assert_not_called()

    def test_stale_lifecycle_or_absent_navigator_prevents_spin(self):
        s = self.scan
        s.states['bt_navigator'] = (3, time.monotonic() - 2.)
        self.assertFalse(s.ready())
        s.states['bt_navigator'] = (3, time.monotonic())
        s.node.count_publishers.return_value = 0
        self.assertFalse(s.ready())

    def test_idle_spin_uses_action_and_one_circle(self):
        s = self.scan
        s.tick()
        s.spin.send_goal_async.assert_called_once()
        goal = s.spin.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(goal.target_yaw, 6.2831853, places=5)
        self.assertEqual(goal.time_allowance.sec, 90)
        self.assertEqual(len(s.goal_id), 16)

    def test_rejected_goal_backs_off(self):
        s = self.scan
        s.accepted(Mock(result=Mock(return_value=Mock(accepted=False))))
        self.assertGreater(s.next_attempt, time.monotonic() + 4.)
        self.assertIsNone(s.goal)

    def test_canceling_navigation_is_still_busy_but_terminal_is_not(self):
        s = self.scan
        s.status = {'navigate_to_pose': [(b'nav', 3)]}
        self.assertTrue(s.busy())
        for status in (4, 5, 6):
            s.status['navigate_to_pose'] = [(b'nav', status)]
            self.assertFalse(s.busy())


if __name__ == '__main__':
    unittest.main()
