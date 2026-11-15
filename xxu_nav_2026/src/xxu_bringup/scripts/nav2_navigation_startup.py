#!/usr/bin/env python3
"""Start the Nav2 navigation lifecycle only after localization is usable."""

import importlib.util
import math
from pathlib import Path
import signal
import time
import uuid

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.action import Spin
from action_msgs.msg import GoalStatusArray
from unique_identifier_msgs.msg import UUID
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class IdleScan:
    """Opt-in idle survey through Nav2 Spin; never publish velocity directly."""

    ACTIONS = ('navigate_to_pose', 'navigate_through_poses', 'follow_waypoints',
               'follow_path', 'spin', 'backup', 'wait', 'drive_on_heading',
               'assisted_teleop')
    BUSY = {1, 2, 3}  # accepted, executing, canceling

    def __init__(self, node):
        self.node = node
        self.status = {}
        self.states = {}
        self.polls = {}
        self.pending = None
        self.goal = None
        self.goal_id = None
        self.canceling = False
        self.next_attempt = time.monotonic() + 3.
        self.idle_since = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [node.create_subscription(
            GoalStatusArray, f'/{name}/_action/status',
            lambda m, name=name: self.observe(name, m), qos) for name in self.ACTIONS]
        self.clients = {name: node.create_client(GetState, f'/{name}/get_state')
                        for name in ('bt_navigator', 'behavior_server')}
        self.spin = ActionClient(node, Spin, '/spin')
        self.timer = node.create_timer(.1, self.tick)
        self.poll_timer = node.create_timer(.5, self.poll_states)

    def observe(self, name, msg):
        self.status[name] = [(bytes(s.goal_info.goal_id.uuid), s.status) for s in msg.status_list]
        if self.busy():
            self.idle_since = None
            self.block_reason = 'another navigation or behavior action is active'
            self.cancel()

    def busy(self):
        return any(status in self.BUSY and not (name == 'spin' and uid == self.goal_id)
                   for name, entries in self.status.items() for uid, status in entries)

    def poll_states(self):
        for name, client in self.clients.items():
            if name in self.polls or not client.service_is_ready():
                continue
            future = client.call_async(GetState.Request())
            self.polls[name] = future
            future.add_done_callback(lambda f, name=name: self.state_result(name, f))

    def state_result(self, name, future):
        self.polls.pop(name, None)
        try:
            self.states[name] = (future.result().current_state.id, time.monotonic())
        except Exception:
            self.states.pop(name, None)

    def ready(self):
        now = time.monotonic()
        if any(self.states.get(name, (0, 0))[0] != 3 or
               now - self.states.get(name, (0, 0))[1] > 1.5 for name in self.clients):
            self.block_reason = 'navigator/behavior lifecycle is inactive or stale'
            return False
        # An absent action publisher must not be interpreted as an idle navigator.
        if any(self.node.count_publishers(f'/{name}/_action/status') == 0
               for name in ('navigate_to_pose', 'navigate_through_poses', 'follow_path')):
            self.block_reason = 'navigation action server is unavailable'
            return False
        health = self.node.localization_health
        if health is None:
            self.block_reason = 'external localization health is required'
            return False
        ready, reasons = health.check(now, self.node.get_clock().now().nanoseconds)
        self.block_reason = '; '.join(reasons)
        return ready

    def cancel(self):
        if self.goal is not None and not self.canceling:
            self.canceling = True
            self.goal.cancel_goal_async()
            self.node.get_logger().info('Canceling idle survey: ' +
                                        getattr(self, 'block_reason', 'stop requested'))

    def tick(self):
        now = time.monotonic()
        if self.busy() or not self.ready():
            self.idle_since = None
            if self.busy():
                self.block_reason = 'another navigation or behavior action is active'
            self.cancel()
            return
        if self.idle_since is None:
            self.idle_since = now
        if (self.goal is not None or self.pending is not None or now < self.next_attempt
                or now - self.idle_since < 2. or not self.spin.server_is_ready()):
            return
        request = Spin.Goal()
        request.target_yaw = 2. * math.pi
        request.time_allowance.sec = 90
        # Assign the ID before sending so our own accepted status is never a rival.
        uid = UUID(uuid=list(uuid.uuid4().bytes))
        self.goal_id = bytes(uid.uuid)
        self.pending = self.spin.send_goal_async(request, goal_uuid=uid)
        self.pending.add_done_callback(self.accepted)

    def accepted(self, future):
        self.pending = None
        try:
            goal = future.result()
        except Exception as exc:
            self.node.get_logger().warn(f'Idle survey request failed: {exc}')
            self.next_attempt = time.monotonic() + 5.
            return
        if not goal.accepted:
            self.next_attempt = time.monotonic() + 5.
            return
        self.goal = goal
        self.canceling = False
        goal.get_result_async().add_done_callback(self.finished)
        self.node.get_logger().info('Idle survey: starting collision-checked one-turn Spin')
        # A navigation goal or loss of localization may arrive while acceptance is pending.
        if self.busy() or not self.ready():
            self.cancel()

    def finished(self, future):
        try:
            result = future.result()
            self.node.get_logger().info(f'Idle survey ended: status={result.status}, error={result.result.error_code}')
            delay = .5 if result.status == 4 else 5.
        except Exception as exc:
            self.node.get_logger().warn(f'Idle survey result failed: {exc}')
            delay = 5.
        self.goal = None
        self.canceling = False
        self.next_attempt = time.monotonic() + delay

    def stop(self):
        self.timer.cancel()
        self.poll_timer.cancel()
        self.block_reason = 'idle survey supervisor is stopping'
        self.cancel()
        deadline = time.monotonic() + 2.
        while (self.pending is not None or self.goal is not None) and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.05)
            self.cancel()


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
        self.declare_parameter('idle_scan', False)
        self.declare_parameter('idle_scan_only', False)
        self.localization_health = None
        if self.get_parameter('require_localization').value:
            # Reuse the final gate's source-time/trust contract without copying it.
            path = Path(get_package_prefix('xxu_bringup')) / 'lib/xxu_bringup/cmd_vel_watchdog.py'
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
        if self.get_parameter('idle_scan_only').value:
            self.timer.cancel()
        self.idle_scan = IdleScan(self) if self.get_parameter('idle_scan').value else None

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
    # Keep the ROS context alive long enough to cancel our owned Spin on exit.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = NavigationStartup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.idle_scan is not None and rclpy.ok():
            node.idle_scan.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
