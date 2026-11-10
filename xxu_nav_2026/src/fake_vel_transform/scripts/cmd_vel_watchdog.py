#!/usr/bin/env python3
"""Gate the final velocity command and fail closed on unhealthy inputs.

The node is deliberately the last ROS node before the chassis controller.  A
command is forwarded only while the command source, odometry, scan, gimbal
joint state, and the configured TF edges are fresh.  Freshness uses a steady
clock so a paused or missing simulation clock cannot keep a stale command
alive.
"""

import copy
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def _finite_twist(message: TwistStamped) -> bool:
    values = (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    )
    return all(math.isfinite(value) for value in values)


def _frame_name(frame: str) -> str:
    return str(frame).strip().lstrip("/")


class LocalizationHealth:
    """Check backend trust, steady-clock source progress and ROS source age.

    Backend epochs are local counters, independent of frontend generations.
    Zero-stamp transition notices cannot renew source freshness.
    """

    def __init__(self, timeout=.5, recovery_hold=1.):
        if not math.isfinite(timeout) or timeout <= 0 or not math.isfinite(recovery_hold) or recovery_hold < 0:
            raise ValueError('localization timeout must be positive and recovery hold nonnegative')
        self.timeout, self.recovery_hold = timeout, recovery_hold
        self.identity, self.revision = None, -1
        self.state_stamp = self.pose_stamp = 0
        self.state_received = self.state_progress = None
        self.pose_received = self.pose_progress = None
        self.trusted = False
        self.stable_since = None

    def close(self):
        self.trusted = False
        self.state_progress = self.pose_progress = self.pose_received = None
        self.stable_since = None

    def observe_state(self, text, received):
        try:
            parts = [part.split('=', 1) for part in text.split(';')]
            if any(len(part) != 2 for part in parts) or len({part[0] for part in parts}) != len(parts):
                raise ValueError('malformed state fields')
            fields = dict(parts)
            epoch, generation, revision, stamp = (int(fields[k]) for k in ('epoch', 'generation', 'pose_revision', 'stamp_ns'))
            if min(epoch, generation, revision, stamp) < 0:
                raise ValueError('negative state identity or stamp')
            identity = (epoch, generation)
            if self.identity is not None and (identity < self.identity or (identity == self.identity and revision < self.revision)):
                raise ValueError('regressed state version')
            if identity != self.identity:
                self.close()
                self.state_stamp = self.pose_stamp = 0
                self.identity = identity
            self.revision = revision
            self.state_received = received
            trusted = (fields['status'] in ('NORMAL', 'OBS_DEGRADED')
                       and fields['pose_trusted'] == 'true' and fields['tf_frozen'] == 'false')
            if not trusted:
                self.close()
                return
            if stamp and stamp < self.state_stamp:
                raise ValueError('regressed state source time')
            if stamp > self.state_stamp:
                self.state_stamp, self.state_progress = stamp, received
            self.trusted = True
        except (ValueError, KeyError, TypeError):
            self.close()

    def observe_pose(self, message, received):
        stamp = message.header.stamp
        pose = message.pose.pose
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if (message.header.frame_id != 'map' or not message.child_frame_id
                or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or stamp_ns <= 0
                or not all(math.isfinite(v) for v in values)
                or sum(v*v for v in values[3:]) < 1.e-12 or stamp_ns < self.pose_stamp):
            self.pose_received = self.pose_progress = self.stable_since = None
            return
        self.pose_received = received
        if stamp_ns > self.pose_stamp:
            self.pose_stamp, self.pose_progress = stamp_ns, received

    def check(self, now, ros_now_ns):
        reasons = []
        if not self.trusted:
            reasons.append('localization untrusted or malformed')
        for name, received, progress, stamp in (
                ('state', self.state_received, self.state_progress, self.state_stamp),
                ('pose', self.pose_received, self.pose_progress, self.pose_stamp)):
            if (received is None or progress is None or not 0 <= now-received <= self.timeout
                    or not 0 <= now-progress <= self.timeout):
                reasons.append(f'localization {name} receipt/source-progress timeout')
            # Allow 50 ms of ROS clock delivery skew, never an unbounded future stamp.
            if stamp <= 0 or not -50_000_000 <= ros_now_ns-stamp <= self.timeout * 1.e9:
                reasons.append(f'localization {name} source timestamp expired or invalid')
        if reasons:
            self.stable_since = None
            return False, reasons
        if self.stable_since is None:
            self.stable_since = now
        if now-self.stable_since < self.recovery_hold:
            return False, ['localization recovery hold']
        return True, []


class CmdVelWatchdog(Node):
    """Republish safe TwistStamped commands, otherwise publish zero."""

    def __init__(self):
        super().__init__("cmd_vel_watchdog")

        self.declare_parameter("input_topic", "/cmd_vel_collision")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/cmd_vel_watchdog/healthy")
        self.declare_parameter("timeout", 0.3)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("output_frame_id", "base_link")

        self.declare_parameter("require_odom", True)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_timeout", 0.5)
        self.declare_parameter("require_scan", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("require_joint_states", True)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("joint_states_timeout", 0.5)

        self.declare_parameter("require_tf", True)
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("tf_target_frame", "odom")
        self.declare_parameter("tf_source_frame", "base_link")
        self.declare_parameter("tf_timeout", 0.5)
        self.declare_parameter("require_localization", False)
        self.declare_parameter("localization_state_topic", "/backend_node/localization_state")
        self.declare_parameter("localization_pose_topic", "/localization/odometry")
        self.declare_parameter("localization_timeout", 0.5)
        self.declare_parameter("localization_recovery_hold", 1.0)
        self.declare_parameter(
            "required_tf_pairs", ["odom->base_footprint", "base_link->gimbal_link"]
        )

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.timeout = max(0.0, float(self.get_parameter("timeout").value))
        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            publish_rate = 20.0
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)

        self.require_odom = bool(self.get_parameter("require_odom").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_timeout = max(0.0, float(self.get_parameter("odom_timeout").value))
        self.require_scan = bool(self.get_parameter("require_scan").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.scan_timeout = max(0.0, float(self.get_parameter("scan_timeout").value))
        self.require_joint_states = bool(
            self.get_parameter("require_joint_states").value
        )
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.joint_states_timeout = max(
            0.0, float(self.get_parameter("joint_states_timeout").value)
        )

        self.require_tf = bool(self.get_parameter("require_tf").value)
        self.tf_topic = str(self.get_parameter("tf_topic").value)
        self.tf_target_frame = _frame_name(self.get_parameter("tf_target_frame").value)
        self.tf_source_frame = _frame_name(self.get_parameter("tf_source_frame").value)
        self.tf_timeout = max(0.0, float(self.get_parameter("tf_timeout").value))
        raw_tf_pairs = self.get_parameter("required_tf_pairs").value
        self.required_tf_pairs = self._parse_tf_pairs(raw_tf_pairs)

        self.last_cmd = TwistStamped()
        self.last_cmd_receive_time = None
        self.last_health_receive_time = {
            "odom": None,
            "scan": None,
            "joint_states": None,
        }
        self.last_tf_pair_receive_time = {
            pair: None for pair in self.required_tf_pairs
        }
        self._last_health_state = None
        self.require_localization = bool(self.get_parameter('require_localization').value)
        self.localization_ready = False
        if self.require_localization:
            self.localization_health = LocalizationHealth(
                float(self.get_parameter('localization_timeout').value),
                float(self.get_parameter('localization_recovery_hold').value))
            self.localization_state_subscription = self.create_subscription(
                String, str(self.get_parameter('localization_state_topic').value),
                lambda msg: self.localization_health.observe_state(msg.data, time.monotonic()), 100)
            self.localization_pose_subscription = self.create_subscription(
                Odometry, str(self.get_parameter('localization_pose_topic').value),
                lambda msg: self.localization_health.observe_pose(msg, time.monotonic()), 100)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.publisher = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.status_publisher = self.create_publisher(Bool, self.status_topic, 10)
        self.subscription = self.create_subscription(
            TwistStamped,
            self.input_topic,
            self.cmd_callback,
            qos_profile_sensor_data,
        )

        if self.require_odom:
            self.odom_subscription = self.create_subscription(
                Odometry,
                self.odom_topic,
                lambda _msg: self._health_callback("odom"),
                qos_profile_sensor_data,
            )
        if self.require_scan:
            self.scan_subscription = self.create_subscription(
                LaserScan,
                self.scan_topic,
                lambda _msg: self._health_callback("scan"),
                qos_profile_sensor_data,
            )
        if self.require_joint_states:
            self.joint_subscription = self.create_subscription(
                JointState,
                self.joint_states_topic,
                lambda _msg: self._health_callback("joint_states"),
                qos_profile_sensor_data,
            )

        self.tf_buffer = Buffer()
        # Keep TF subscriptions progressing independently of the safety timer
        # and health callbacks.  This is important when a high-rate sensor
        # callback temporarily occupies the node's default executor.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        if self.require_tf and self.tf_topic:
            self.tf_subscription = self.create_subscription(
                TFMessage,
                self.tf_topic,
                self._tf_callback,
                qos_profile_sensor_data,
            )

        # A wall timer keeps the safety output alive even when /clock is
        # paused.  Message timestamps still use the node clock, so the same
        # executable works with either real or simulation time.
        self.timer = self.create_timer(
            1.0 / publish_rate, self.publish_cmd, clock=self.steady_clock
        )

        self.get_logger().info(
            f"Velocity safety gate: {self.input_topic} -> {self.output_topic}; "
            f"command_timeout={self.timeout:.3f}s, "
            f"health odom={self.require_odom}, scan={self.require_scan}, "
            f"joint_states={self.require_joint_states}, tf={self.require_tf}"
        )

    def close_tf_listener(self):
        """Stop the dedicated TF executor before destroying the ROS node."""
        listener = self.tf_listener
        if listener is None:
            return
        try:
            listener.unregister()
        finally:
            executor = getattr(listener, "executor", None)
            thread = getattr(listener, "dedicated_listener_thread", None)
            if executor is not None:
                executor.shutdown()
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
            self.tf_listener = None

    @staticmethod
    def _parse_tf_pairs(raw_pairs):
        if isinstance(raw_pairs, str):
            raw_pairs = [raw_pairs]
        pairs = []
        for raw_pair in raw_pairs or []:
            text = str(raw_pair)
            if "->" not in text:
                continue
            parent, child = text.split("->", 1)
            parent = _frame_name(parent)
            child = _frame_name(child)
            if parent and child:
                pairs.append((parent, child))
        return tuple(dict.fromkeys(pairs))

    def _health_callback(self, name):
        self.last_health_receive_time[name] = time.monotonic()

    def _tf_callback(self, message):
        received = time.monotonic()
        for transform in message.transforms:
            pair = (
                _frame_name(transform.header.frame_id),
                _frame_name(transform.child_frame_id),
            )
            if pair in self.last_tf_pair_receive_time:
                self.last_tf_pair_receive_time[pair] = received

    def cmd_callback(self, message):
        if not _finite_twist(message):
            self.get_logger().error("Rejecting non-finite TwistStamped command")
            self.last_cmd_receive_time = None
            return
        self.last_cmd = message
        self.last_cmd_receive_time = time.monotonic()

    @staticmethod
    def _fresh(received, timeout, now):
        if received is None:
            return False
        age = now - received
        return 0.0 <= age <= timeout

    def _health(self, now):
        reasons = []
        if self.require_localization:
            ready, localization_reasons = self.localization_health.check(now, self.get_clock().now().nanoseconds)
            # Discard buffered commands while closed, and on the opening edge:
            # release requires a new command after stable localization returns.
            if not ready or not self.localization_ready:
                self.last_cmd_receive_time = None
            self.localization_ready = ready
            reasons.extend(localization_reasons)
        if not self._fresh(self.last_cmd_receive_time, self.timeout, now):
            reasons.append("command timeout")
        stamp = self.last_cmd.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        if (stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000
                or not 0 <= age_ns <= self.timeout * 1_000_000_000):
            reasons.append("command source timestamp expired or invalid")
        if self.require_odom and not self._fresh(
            self.last_health_receive_time["odom"], self.odom_timeout, now
        ):
            reasons.append("odometry timeout")
        if self.require_scan and not self._fresh(
            self.last_health_receive_time["scan"], self.scan_timeout, now
        ):
            reasons.append("scan timeout")
        if self.require_joint_states and not self._fresh(
            self.last_health_receive_time["joint_states"], self.joint_states_timeout, now
        ):
            reasons.append("joint_states timeout")

        if self.require_tf:
            if not self.tf_buffer.can_transform(
                self.tf_target_frame,
                self.tf_source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            ):
                reasons.append(
                    f"missing TF {self.tf_target_frame}->{self.tf_source_frame}"
                )
            for parent, child in self.required_tf_pairs:
                if not self._fresh(
                    self.last_tf_pair_receive_time[(parent, child)], self.tf_timeout, now
                ):
                    reasons.append(f"TF stream timeout {parent}->{child}")

        return len(reasons) == 0, reasons

    def publish_cmd(self):
        now_wall = time.monotonic()
        healthy, reasons = self._health(now_wall)
        cmd = TwistStamped()
        if healthy:
            # Never overwrite the source stamp used by the next health check.
            cmd = copy.deepcopy(self.last_cmd)
        if not cmd.header.frame_id and self.output_frame_id:
            cmd.header.frame_id = self.output_frame_id
        cmd.header.stamp = self.get_clock().now().to_msg()
        if not healthy:
            cmd.twist = TwistStamped().twist

        self.publisher.publish(cmd)
        status = Bool()
        status.data = healthy
        self.status_publisher.publish(status)

        if healthy != self._last_health_state:
            self._last_health_state = healthy
            if healthy:
                self.get_logger().info("Velocity safety gate opened")
            else:
                self.get_logger().warning(
                    "Velocity safety gate closed; publishing zero: "
                    + ", ".join(reasons)
                )


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SIGINT can arrive while DDS is tearing down a subscription.  Keep
        # shutdown fail-safe and quiet; an interrupted cleanup must not turn
        # the watchdog into a noisy, non-zero child process.
        try:
            node.close_tf_listener()
        except KeyboardInterrupt:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
