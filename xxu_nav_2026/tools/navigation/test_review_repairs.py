#!/usr/bin/env python3
"""Regression checks for freshness, process ownership and model contracts.

Source ROS Jazzy first. By default no ROS nodes or simulations are started.
Set XXU_REVIEW_CHASSIS_EXECUTABLE to a rebuilt chassis binary to also run
the isolated node check (domain 193, private topic names, no robot drivers).
"""

import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped


WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / 'src/xxu_sim_bringup'))
from xxu_sim_bringup.simulation_processes import Process, select_trees


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


interfaces = load('review_interfaces', Path(__file__).with_name('check_interfaces.py'))
navigation = load('review_navigation', Path(__file__).with_name('check_ega_navigation.py'))


class FreshnessTest(unittest.TestCase):
    def test_installed_cpp_gate_source_stamp_and_steady_timeout(self):
        subprocess.run([sys.executable, str(Path(__file__).with_name('test_cpp_bridge_pipeline.py')),
                        'CppPipelineTest.test_watchdog_source_stamp_and_paused_clock_fail_closed'], check=True)


class ProcessOwnershipTest(unittest.TestCase):
    def test_only_owned_simulation_and_mapping_trees_are_selected(self):
        def process(pid, parent, package, launch, cwd):
            return Process(pid, parent, str(pid), ('ros2', 'launch', package, launch), Path(cwd))
        processes = {
            1: process(1, 0, 'xxu_sim_bringup', 'rmul_navigation.launch.py', '/work'),
            2: Process(2, 1, '2', ('chassis_controller',), Path('/other')),
            3: process(3, 0, 'xxu_sim_bringup', 'rmul_navigation.launch.py', '/other'),
            4: process(4, 0, 'xxu_hardware', 'chassis_lower_bridge.launch.py', '/work'),
            5: process(5, 0, 'other_robot', 'simulation.launch.py', '/work'),
            6: process(6, 0, 'xxu_slam_toolbox', 'mapping.launch.py', '/work/src'),
            7: Process(7, 4, '7', ('chassis_lower_bridge.py',), Path('/work')),
        }
        self.assertEqual(select_trees(processes, Path('/work')), ({1, 6}, {1, 2, 6}))
        self.assertEqual(select_trees(processes, Path('/work'), caller=2), ({6}, {6}))


class ContractTest(unittest.TestCase):
    def test_model_endpoint_changes_are_rejected(self):
        path = WORKSPACE / 'src/xxu_bringup/config/robot_interfaces.yaml'
        self.assertEqual(interfaces.check(WORKSPACE)[2], [])
        for key in ['wheel_command', 'rm_lidar']:
            with self.subTest(key=key), tempfile.TemporaryDirectory() as folder:
                data = yaml.safe_load(path.read_text())
                data['topics']['simulation'][key]['name'] = '/wrong/{wheel}' if key == 'wheel_command' else '/wrong'
                candidate = Path(folder) / 'contract.yaml'
                candidate.write_text(yaml.safe_dump(data))
                errors = interfaces.check(WORKSPACE, candidate)[2]
                self.assertTrue(any('ega_folded' in error for error in errors), errors)

    def test_interface_module_does_not_import_launch_or_nav2(self):
        source = WORKSPACE / 'src/xxu_bringup'
        code = (f'import sys; sys.path.insert(0, {str(source)!r}); '
                'import xxu_bringup.interfaces; '
                'assert not any(x.startswith(("launch", "nav2_common")) for x in sys.modules)')
        subprocess.run([sys.executable, '-I', '-c', code], check=True)

    def test_map_path_module_does_not_import_launch_or_nav2(self):
        source = WORKSPACE / 'src/xxu_bringup'
        code = (f'import sys; sys.path.insert(0, {str(source)!r}); '
                'import xxu_bringup.map_paths; '
                'assert not any(x.startswith(("launch", "nav2_common")) for x in sys.modules)')
        subprocess.run([sys.executable, '-I', '-c', code], check=True)


class ParkingEvidenceTest(unittest.TestCase):
    def history(self):
        return {key: [(99.3 + .2 * i, 9.3 + .2 * i, list(values)) for i in range(4)]
                for key, values in {'truth': [2., 3., math.pi - .001], 'odom': [0., 0., 0.],
                                    'safe': [0., 0., 0.], 'wheels': [0.]}.items()}

    def test_zero_commands_do_not_prove_body_has_stopped(self):
        history = self.history()
        self.assertTrue(navigation.parking_evidence(history, 100., 10.)['passed'])
        history['odom'][-1][2][0] = .1
        self.assertFalse(navigation.parking_evidence(history, 100., 10.)['passed'])

    def test_drift_and_return_to_origin_are_detected(self):
        history = self.history()
        history['truth'][1][2][0] += .02
        self.assertFalse(navigation.parking_evidence(history, 100., 10.)['passed'])

    def test_stale_and_missing_observations_fail_closed(self):
        for key in self.history():
            history = self.history()
            history[key] = []
            self.assertFalse(navigation.parking_evidence(history, 100., 10.)['passed'])
        self.assertFalse(navigation.parking_evidence(self.history(), 101., 11.)['passed'])


@unittest.skipUnless(os.environ.get('XXU_REVIEW_CHASSIS_EXECUTABLE'), 'rebuilt chassis executable not selected')
class ChassisInputTest(unittest.TestCase):
    def test_invalid_stamps_stop_without_crashing_and_valid_commands_recover(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Float64MultiArray

        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '193', 'ROS_LOCALHOST_ONLY': '1'}):
            context = Context()
            rclpy.init(context=context)
            node = rclpy.create_node('review_chassis_observer', context=context, enable_rosout=False,
                                     start_parameter_services=False)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            messages = []
            publisher = node.create_publisher(TwistStamped, '/review_repair/cmd_vel', 10)
            subscription = node.create_subscription(Float64MultiArray, '/review_repair/wheels',
                                                    lambda message: messages.append(list(message.data)), 10)
            def wait_for(predicate, timeout=5.):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.02)
                    if predicate():
                        return True
                return False

            try:
                for fresh in ['false', 'true']:
                    with self.subTest(require_fresh_stamp=fresh), tempfile.TemporaryFile() as log:
                        proc = subprocess.Popen([
                            os.environ['XXU_REVIEW_CHASSIS_EXECUTABLE'], '--ros-args',
                            '-p', 'cmd_vel_topic:=/review_repair/cmd_vel',
                            '-p', 'wheel_command_topic:=/review_repair/wheels',
                            '-p', 'require_fresh_stamp:=' + fresh], stdout=log, stderr=subprocess.STDOUT)
                        try:
                            self.assertTrue(wait_for(lambda: publisher.get_subscription_count() > 0))
                            def send(sec=None, nanosec=None):
                                message = TwistStamped()
                                message.header.frame_id = 'base_link'
                                message.header.stamp = node.get_clock().now().to_msg()
                                if sec is not None:
                                    message.header.stamp.sec = sec
                                if nanosec is not None:
                                    message.header.stamp.nanosec = nanosec
                                message.twist.linear.x = 1.
                                publisher.publish(message)
                            for sec, nanosec in [(-1, 0), (0, 1_000_000_000)]:
                                messages.clear()
                                send()
                                self.assertTrue(wait_for(lambda: any(max(map(abs, row)) > 1 for row in messages)))
                                messages.clear()
                                send(sec, nanosec)
                                self.assertTrue(wait_for(lambda: any(max(map(abs, row)) == 0 for row in messages)))
                                self.assertIsNone(proc.poll())
                            messages.clear()
                            send()
                            self.assertTrue(wait_for(lambda: any(max(map(abs, row)) > 1 for row in messages)))
                            messages.clear()
                            self.assertTrue(wait_for(lambda: any(max(map(abs, row)) == 0 for row in messages), 1.))
                        finally:
                            proc.terminate()
                            try:
                                proc.wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                proc.kill()
                                proc.wait()
                            wait_for(lambda: publisher.get_subscription_count() == 0, 2.)
            finally:
                node.destroy_subscription(subscription)
                executor.shutdown()
                node.destroy_node()
                context.try_shutdown()


if __name__ == '__main__':
    unittest.main()
