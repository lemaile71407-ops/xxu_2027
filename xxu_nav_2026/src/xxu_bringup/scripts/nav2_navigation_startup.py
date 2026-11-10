#!/usr/bin/env python3
"""Start the Nav2 navigation lifecycle only after localization is usable."""

import importlib.util
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class NavigationStartup(Node):
    """Gate activation on TF and, in external mode, existing localization health."""

    def __init__(self):
        super().__init__("nav2_navigation_startup")
        self.declare_parameter("manager_service", "/lifecycle_manager_navigation/manage_nodes")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("check_period", 0.5)
        self.declare_parameter('require_localization', False)
        self.declare_parameter('localization_state_topic', '/backend_node/localization_state')
        self.declare_parameter('localization_pose_topic', '/localization/odometry')
        self.declare_parameter('localization_timeout', 0.5)
        self.declare_parameter('localization_recovery_hold', 1.0)
        self.localization_health = None
        if self.get_parameter('require_localization').value:
            # Reuse the final gate's source-time/trust contract without copying it.
            path = Path(get_package_prefix('fake_vel_transform')) / 'lib/fake_vel_transform/cmd_vel_watchdog.py'
            spec = importlib.util.spec_from_file_location('navigation_localization_health', path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.localization_health = module.LocalizationHealth(
                float(self.get_parameter('localization_timeout').value),
                float(self.get_parameter('localization_recovery_hold').value))
            self.state_subscription = self.create_subscription(
                String, str(self.get_parameter('localization_state_topic').value),
                lambda m: self.localization_health.observe_state(m.data, time.monotonic()), 100)
            self.pose_subscription = self.create_subscription(
                Odometry, str(self.get_parameter('localization_pose_topic').value),
                lambda m: self.localization_health.observe_pose(m, time.monotonic()), 100)

        self.manager_service = self.get_parameter("manager_service").value
        self.global_frame = self.get_parameter("global_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = self.create_client(ManageLifecycleNodes, self.manager_service)
        self.request_sent = False
        self.timer = self.create_timer(
            float(self.get_parameter("check_period").value), self.try_start
        )

    def try_start(self):
        if self.request_sent:
            return
        if not self.client.service_is_ready():
            self.get_logger().info(
                f"Waiting for navigation lifecycle manager at {self.manager_service}",
                throttle_duration_sec=5.0,
            )
            return
        if self.localization_health is not None:
            ready, reasons = self.localization_health.check(
                time.monotonic(), self.get_clock().now().nanoseconds)
            if not ready:
                self.get_logger().info('Waiting for trusted external localization: ' + '; '.join(reasons),
                                       throttle_duration_sec=5.0)
                return
        if not self.tf_buffer.can_transform(
            self.global_frame, self.odom_frame, Time(), timeout=Duration(seconds=0.0)
        ):
            self.get_logger().info(
                f"Waiting for localization transform {self.global_frame} -> {self.odom_frame}",
                throttle_duration_sec=5.0,
            )
            return

        self.request_sent = True
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self.client.call_async(request)
        future.add_done_callback(self.startup_complete)
        self.get_logger().info("Localization is ready; starting Nav2 navigation lifecycle")

    def startup_complete(self, future):
        try:
            response = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Navigation lifecycle startup failed: {exc}")
            self.request_sent = False
            return

        if response.success:
            self.get_logger().info("Nav2 navigation lifecycle is active")
            self.timer.cancel()
        else:
            self.get_logger().error("Navigation lifecycle manager rejected startup; retrying")
            self.request_sent = False


def main():
    rclpy.init()
    node = NavigationStartup()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
