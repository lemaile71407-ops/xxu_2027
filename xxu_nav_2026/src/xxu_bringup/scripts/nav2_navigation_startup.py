#!/usr/bin/env python3
"""Start the Nav2 navigation lifecycle only after localization is usable."""

import importlib.util
from pathlib import Path
import signal
import time

from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool

try:
    from xxu_bringup.idle_scan import IdleScan
except ModuleNotFoundError as exc:
    if exc.name not in ('xxu_bringup', 'xxu_bringup.idle_scan'):
        raise
    _IDLE_SCAN_PATH = Path(__file__).resolve().parents[1] / 'xxu_bringup' / 'idle_scan.py'
    _IDLE_SCAN_SPEC = importlib.util.spec_from_file_location('xxu_bringup.idle_scan', _IDLE_SCAN_PATH)
    _IDLE_SCAN_MODULE = importlib.util.module_from_spec(_IDLE_SCAN_SPEC)
    _IDLE_SCAN_SPEC.loader.exec_module(_IDLE_SCAN_MODULE)
    IdleScan = _IDLE_SCAN_MODULE.IdleScan


class BridgeReadiness:
    """Low-rate readiness from the C++ gate; no sensor or TF deserialization."""

    def __init__(self, timeout):
        self.timeout = timeout
        self.received = None
        self.ready = False

    def observe(self, message):
        self.ready = message.data
        self.received = time.monotonic()

    def check(self, now, _ros_now_ns):
        fresh = self.received is not None and 0 <= now - self.received <= self.timeout
        return (
            (True, [])
            if self.ready and fresh
            else (False, ['C++ localization/TF readiness unavailable or expired'])
        )


class NavigationStartup(Node):
    """Gate activation on TF and, in external mode, existing localization health."""

    def __init__(self):
        super().__init__('nav2_navigation_startup')
        self.declare_parameter('manager_service', '/lifecycle_manager_navigation/manage_nodes')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('check_period', 0.5)
        self.declare_parameter('require_localization', False)
        self.declare_parameter('localization_state_topic', '/backend_node/localization_state')
        self.declare_parameter('localization_pose_topic', '/localization/odometry')
        self.declare_parameter('localization_timeout', 0.5)
        self.declare_parameter('localization_recovery_hold', 1.0)
        self.declare_parameter('idle_scan', False)
        self.declare_parameter('idle_scan_only', False)
        self.readiness = BridgeReadiness(
            float(self.get_parameter('localization_timeout').value)
        )
        require_localization = self.get_parameter('require_localization').value
        self.localization_health = self.readiness if require_localization else None
        self.declare_parameter(
            'readiness_topic',
            '/cmd_vel_watchdog/localization_ready'
            if require_localization
            else '/cmd_vel_watchdog/tf_ready',
        )
        self.ready_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter('readiness_topic').value),
            self.readiness.observe,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.manager_service = self.get_parameter('manager_service').value
        self.client = self.create_client(ManageLifecycleNodes, self.manager_service)
        self.request_sent = False
        self.timer = self.create_timer(
            float(self.get_parameter('check_period').value), self.try_start
        )
        if self.get_parameter('idle_scan_only').value:
            self.timer.cancel()
        self.idle_scan = IdleScan(self) if self.get_parameter('idle_scan').value else None

    def try_start(self):
        if self.request_sent:
            return
        if not self.client.service_is_ready():
            self.get_logger().info(
                f'Waiting for navigation lifecycle manager at {self.manager_service}',
                throttle_duration_sec=5.0,
            )
            return
        ready, reasons = self.readiness.check(
            time.monotonic(), self.get_clock().now().nanoseconds
        )
        if not ready:
            self.get_logger().info(
                'Waiting for C++ localization/TF readiness: ' + '; '.join(reasons),
                throttle_duration_sec=5.0,
            )
            return
        self.request_sent = True
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self.client.call_async(request)
        future.add_done_callback(self.startup_complete)
        self.get_logger().info('Localization is ready; starting Nav2 navigation lifecycle')

    def startup_complete(self, future):
        try:
            response = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Navigation lifecycle startup failed: {exc}')
            self.request_sent = False
            return

        if response.success:
            self.get_logger().info('Nav2 navigation lifecycle is active')
            self.timer.cancel()
        else:
            self.get_logger().error('Navigation lifecycle manager rejected startup; retrying')
            self.request_sent = False


def main():
    # Keep the ROS context alive long enough to cancel our owned Spin on exit.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = NavigationStartup()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node.idle_scan is not None and rclpy.ok():
                node.idle_scan.stop()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
