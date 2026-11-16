#!/usr/bin/env python3
"""C++ localization gate and installed pipeline regressions; source ROS first."""
from pathlib import Path
import subprocess
import unittest
import importlib.util
from types import SimpleNamespace
from unittest.mock import Mock, patch
from std_msgs.msg import Bool
from test_cpp_bridge_pipeline import CppPipelineTest


class CppContractsTest(unittest.TestCase):
    def test_conversion_covariance_and_localization_state_contracts(self):
        folder = Path(__file__).resolve().parents[2] / 'build/xxu_bringup'
        subprocess.run(['ctest', '--test-dir', str(folder), '-R', '^sentry_bridge_test$',
                        '--output-on-failure'], check=True)


class StartupReadinessTest(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[2] / 'src/xxu_bringup/scripts/nav2_navigation_startup.py'
        spec = importlib.util.spec_from_file_location('startup_cpp_readiness', path)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def test_stale_or_false_readiness_never_renews_itself(self):
        ready = self.module.BridgeReadiness(.5)
        with patch.object(self.module.time, 'monotonic', return_value=10.):
            ready.observe(Bool(data=True))
        self.assertTrue(ready.check(10.1, 0)[0])
        self.assertFalse(ready.check(10.6, 0)[0])
        with patch.object(self.module.time, 'monotonic', return_value=10.7):
            ready.observe(Bool(data=False))
        self.assertFalse(ready.check(10.7, 0)[0])

    def test_navigation_cannot_start_without_cpp_tf_readiness(self):
        node = SimpleNamespace(request_sent=False, client=Mock(), readiness=Mock(),
            get_logger=Mock(), get_clock=Mock(), startup_complete=Mock())
        node.client.service_is_ready.return_value = True
        node.readiness.check.return_value = (False, ['TF unavailable'])
        self.module.NavigationStartup.try_start(node)
        node.client.call_async.assert_not_called()
        node.readiness.check.return_value = (True, [])
        self.module.NavigationStartup.try_start(node)
        node.client.call_async.assert_called_once()


if __name__ == '__main__':
    unittest.main()
