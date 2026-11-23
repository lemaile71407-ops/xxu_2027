"""Opt-in idle survey supervisor for Nav2 Spin."""

import math
import time
import uuid

from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import Spin
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from unique_identifier_msgs.msg import UUID


class IdleScan:
    """Opt-in idle survey through Nav2 Spin; never publish velocity directly."""

    ACTIONS = (
        'navigate_to_pose',
        'navigate_through_poses',
        'follow_waypoints',
        'follow_path',
        'spin',
        'backup',
        'wait',
        'drive_on_heading',
        'assisted_teleop',
    )
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
        self.next_attempt = time.monotonic() + 3.0
        self.idle_since = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            node.create_subscription(
                GoalStatusArray,
                f'/{name}/_action/status',
                lambda m, name=name: self.observe(name, m),
                qos,
            )
            for name in self.ACTIONS
        ]
        self.clients = {
            name: node.create_client(GetState, f'/{name}/get_state')
            for name in ('bt_navigator', 'behavior_server')
        }
        self.spin = ActionClient(node, Spin, '/spin')
        self.timer = node.create_timer(0.1, self.tick)
        self.poll_timer = node.create_timer(0.5, self.poll_states)

    def observe(self, name, msg):
        self.status[name] = [
            (bytes(s.goal_info.goal_id.uuid), s.status) for s in msg.status_list
        ]
        if self.busy():
            self.idle_since = None
            self.block_reason = 'another navigation or behavior action is active'
            self.cancel()

    def busy(self):
        return any(
            status in self.BUSY and not (name == 'spin' and uid == self.goal_id)
            for name, entries in self.status.items()
            for uid, status in entries
        )

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
        if any(
            self.states.get(name, (0, 0))[0] != 3
            or now - self.states.get(name, (0, 0))[1] > 1.5
            for name in self.clients
        ):
            self.block_reason = 'navigator/behavior lifecycle is inactive or stale'
            return False
        # An absent action publisher must not be interpreted as an idle navigator.
        if any(
            self.node.count_publishers(f'/{name}/_action/status') == 0
            for name in ('navigate_to_pose', 'navigate_through_poses', 'follow_path')
        ):
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
            self.node.get_logger().info(
                'Canceling idle survey: '
                + getattr(self, 'block_reason', 'stop requested')
            )

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
        if (
            self.goal is not None
            or self.pending is not None
            or now < self.next_attempt
            or now - self.idle_since < 2.0
            or not self.spin.server_is_ready()
        ):
            return
        request = Spin.Goal()
        request.target_yaw = 2.0 * math.pi
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
            self.next_attempt = time.monotonic() + 5.0
            return
        if not goal.accepted:
            self.next_attempt = time.monotonic() + 5.0
            return
        self.goal = goal
        self.canceling = False
        goal.get_result_async().add_done_callback(self.finished)
        self.node.get_logger().info(
            'Idle survey: starting collision-checked one-turn Spin'
        )
        # A navigation goal or loss of localization may arrive while acceptance is pending.
        if self.busy() or not self.ready():
            self.cancel()

    def finished(self, future):
        try:
            result = future.result()
            self.node.get_logger().info(
                f'Idle survey ended: status={result.status}, error={result.result.error_code}'
            )
            delay = 0.5 if result.status == 4 else 5.0
        except Exception as exc:
            self.node.get_logger().warn(f'Idle survey result failed: {exc}')
            delay = 5.0
        self.goal = None
        self.canceling = False
        self.next_attempt = time.monotonic() + delay

    def stop(self):
        self.timer.cancel()
        self.poll_timer.cancel()
        self.block_reason = 'idle survey supervisor is stopping'
        self.cancel()
        deadline = time.monotonic() + 2.0
        while (
            self.pending is not None or self.goal is not None
        ) and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.cancel()
