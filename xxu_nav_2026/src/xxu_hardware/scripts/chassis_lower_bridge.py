#!/usr/bin/env python3
"""ROS 2 serial/USB CDC bridge for the STM32 chassis lower controller."""

import json
import math
import threading
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
import serial
from std_msgs.msg import String

from xxu_hardware.chassis_protocol import (
    ChassisBridgeStateMachine,
    ChassisStatus,
    DEFAULT_ACK_TIMEOUT_S,
    DEFAULT_COMMAND_RATE_HZ,
    DEFAULT_STATUS_TIMEOUT_S,
    FrameStreamParser,
    WheelOdometryIntegrator,
    quantize_milli,
)


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _default_serial_factory(port: str, baudrate: int) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
        write_timeout=0.05,
        rtscts=False,
        dsrdtr=False,
        xonxoff=False,
    )


class ChassisLowerBridge(Node):
    """Bridge /cmd_vel to STM32 0x10 frames and decode 0x20 status/wheel feedback."""

    def __init__(self, *, serial_factory=None):
        super().__init__("chassis_lower_bridge")
        self._serial_factory = serial_factory or _default_serial_factory

        self.port = str(self.declare_parameter("port", "/dev/ttyACM0").value)
        self.baudrate = int(self.declare_parameter("baudrate", 115200).value)
        self.cmd_vel_topic = str(
            self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        )
        self.expected_frame_id = str(
            self.declare_parameter("expected_frame_id", "base_link").value
        )
        self.accept_empty_frame_id = bool(
            self.declare_parameter("accept_empty_frame_id", True).value
        )
        self.send_rate = float(
            self.declare_parameter("send_rate", DEFAULT_COMMAND_RATE_HZ).value
        )
        self.cmd_timeout = float(self.declare_parameter("cmd_timeout", 0.3).value)
        self.require_fresh_stamp = bool(
            self.declare_parameter("require_fresh_stamp", True).value
        )
        self.status_timeout = float(
            self.declare_parameter("status_timeout", DEFAULT_STATUS_TIMEOUT_S).value
        )
        self.ack_timeout = float(
            self.declare_parameter("ack_timeout", DEFAULT_ACK_TIMEOUT_S).value
        )
        self.require_run_enable = bool(
            self.declare_parameter("require_run_enable", False).value
        )
        self.reconnect_interval = float(
            self.declare_parameter("reconnect_interval", 0.5).value
        )
        self.status_topic = str(
            self.declare_parameter("status_topic", "/chassis/status").value
        )
        self.publish_wheel_vel = bool(
            self.declare_parameter("publish_wheel_vel", True).value
        )
        self.wheel_vel_topic = str(
            self.declare_parameter("wheel_vel_topic", "/chassis/wheel_vel").value
        )
        self.publish_wheel_odom = bool(
            self.declare_parameter("publish_wheel_odom", True).value
        )
        self.wheel_odom_topic = str(
            self.declare_parameter("wheel_odom_topic", "/wheel_odom").value
        )
        self.wheel_odom_frame = str(
            self.declare_parameter("wheel_odom_frame", "wheel_odom").value
        )
        self.base_frame = str(
            self.declare_parameter("base_frame", "base_footprint").value
        )

        self._validate_parameters()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._serial = None
        self._connected = False
        self._next_reconnect_mono = 0.0

        self._parser = FrameStreamParser()
        self._wheel_odom = WheelOdometryIntegrator()
        self._sm = ChassisBridgeStateMachine(
            status_timeout_s=self.status_timeout,
            ack_timeout_s=self.ack_timeout,
            require_run_enable=self.require_run_enable,
        )

        self._has_cmd = False
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0
        self._cmd_stamp_ns = 0
        self._cmd_receive_mono = 0.0
        self._last_tx_meta = None
        self._last_logged_state = None
        self._last_pub_stamp_ns = 0

        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, self.status_topic, status_qos)
        self.wheel_vel_pub = (
            self.create_publisher(TwistStamped, self.wheel_vel_topic, 20)
            if self.publish_wheel_vel
            else None
        )
        self.wheel_odom_pub = (
            self.create_publisher(Odometry, self.wheel_odom_topic, 20)
            if self.publish_wheel_odom
            else None
        )

        self.cmd_sub = self.create_subscription(
            TwistStamped,
            self.cmd_vel_topic,
            self.cmd_callback,
            qos_profile_sensor_data,
        )

        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0 / self.send_rate, self._on_send_timer, clock=self._steady_clock
        )
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="chassis_serial_rx", daemon=True
        )
        self._reader_thread.start()

        self.get_logger().info(
            f"Chassis lower bridge started: port={self.port}, baudrate={self.baudrate}, "
            f"cmd_vel={self.cmd_vel_topic}, send_rate={self.send_rate:.1f}Hz"
        )

    def _validate_parameters(self) -> None:
        if not self.port:
            raise ValueError("port must be nonempty")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if not math.isfinite(self.send_rate) or self.send_rate <= 0.0:
            raise ValueError("send_rate must be finite and positive")
        if not math.isfinite(self.cmd_timeout) or self.cmd_timeout < 0.0:
            raise ValueError("cmd_timeout must be finite and nonnegative")
        if not math.isfinite(self.status_timeout) or self.status_timeout <= 0.0:
            raise ValueError("status_timeout must be finite and positive")
        if not math.isfinite(self.ack_timeout) or self.ack_timeout < 0.0:
            raise ValueError("ack_timeout must be finite and nonnegative")
        if not math.isfinite(self.reconnect_interval) or self.reconnect_interval <= 0.0:
            raise ValueError("reconnect_interval must be finite and positive")
        if not self.wheel_odom_frame or not self.base_frame:
            raise ValueError("wheel_odom_frame and base_frame must be nonempty")

    def cmd_callback(self, msg: TwistStamped) -> None:
        if not self.accept_empty_frame_id and not msg.header.frame_id:
            self.get_logger().warn(
                "Rejecting TwistStamped with empty frame_id", throttle_duration_sec=2.0
            )
            return
        if (
            msg.header.frame_id
            and self.expected_frame_id
            and msg.header.frame_id != self.expected_frame_id
        ):
            self.get_logger().warn(
                f"Rejecting TwistStamped in frame '{msg.header.frame_id}'; "
                f"expected '{self.expected_frame_id}'",
                throttle_duration_sec=2.0,
            )
            return

        stamp = msg.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            self.get_logger().error(
                "Rejecting TwistStamped with invalid timestamp",
                throttle_duration_sec=2.0,
            )
            with self._lock:
                self._has_cmd = False
            return

        twist = msg.twist
        values = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        if not all(math.isfinite(v) for v in values):
            self.get_logger().error(
                "Rejecting non-finite TwistStamped command",
                throttle_duration_sec=2.0,
            )
            with self._lock:
                self._has_cmd = False
            return

        try:
            quantize_milli(twist.linear.x, "vx")
            quantize_milli(twist.linear.y, "vy")
            quantize_milli(twist.angular.z, "wz")
        except ValueError as exc:
            self.get_logger().error(
                f"Rejecting out-of-range TwistStamped command: {exc}",
                throttle_duration_sec=2.0,
            )
            with self._lock:
                self._has_cmd = False
            return

        now_mono = time.monotonic()
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        with self._lock:
            self._cmd_vx = float(twist.linear.x)
            self._cmd_vy = float(twist.linear.y)
            self._cmd_wz = float(twist.angular.z)
            self._cmd_stamp_ns = stamp_ns
            self._cmd_receive_mono = now_mono
            self._has_cmd = True

    def _next_pub_stamp(self):
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            if now_ns <= self._last_pub_stamp_ns:
                now_ns = self._last_pub_stamp_ns + 1
            self._last_pub_stamp_ns = now_ns
        return Time(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)

    def _open_serial_if_due(self, now_mono: float | None = None):
        with self._lock:
            if self._serial is not None:
                return self._serial
            check_mono = time.monotonic() if now_mono is None else now_mono
            if check_mono < self._next_reconnect_mono:
                return None

        try:
            ser = self._serial_factory(self.port, self.baudrate)
        except Exception as exc:
            self.get_logger().warn(
                f"Failed to open serial port {self.port}: {exc}",
                throttle_duration_sec=2.0,
            )
            with self._lock:
                mono = time.monotonic()
                self._connected = False
                self._next_reconnect_mono = mono + self.reconnect_interval
            return None

        with self._lock:
            mono = time.monotonic()
            self._serial = ser
            self._connected = True
            self._parser.clear()
            self._wheel_odom.reset()
            self._sm.on_link_reset(mono, "serial_connected")
        self.get_logger().info(f"Connected to STM32 chassis port {self.port}")
        return ser

    def _close_serial(
        self, reason: str, now_mono: float | None = None, *, expected_ser=None
    ) -> None:
        ser = None
        was_connected = False
        with self._lock:
            if expected_ser is not None and self._serial is not expected_ser:
                return
            mono = time.monotonic() if now_mono is None else now_mono
            ser = self._serial
            self._serial = None
            was_connected = self._connected
            self._connected = False
            self._last_tx_meta = None
            self._next_reconnect_mono = mono + self.reconnect_interval
            self._parser.clear()
            self._wheel_odom.reset()
            self._sm.on_link_reset(mono, reason)
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if was_connected:
            self.get_logger().warn(f"Serial port {self.port} closed ({reason})")

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                ser = self._serial
            if ser is None:
                self._stop_event.wait(0.02)
                continue

            try:
                chunk = ser.read(256)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warn(
                        f"Serial read failed on {self.port}: {exc}",
                        throttle_duration_sec=2.0,
                    )
                    self._close_serial("serial_read_error", expected_ser=ser)
                continue

            if not chunk:
                continue

            outputs = []
            with self._lock:
                # Sample monotonic clock under _lock so status timestamps stay ordered with send timer checks.
                now_mono = time.monotonic()
                decoded = self._parser.feed(chunk)
                for status in decoded:
                    self._sm.on_status(status, now_mono)
                    odom_step = self._wheel_odom.update(status)
                    outputs.append((status, odom_step))

            for status, odom_step in outputs:
                self._publish_feedback(status, odom_step)

    def _publish_feedback(
        self,
        status: ChassisStatus,
        odom_step: tuple[float, float, float, float, float, float, float] | None,
    ) -> None:
        if not status.wheel_valid or odom_step is None:
            return
        x, y, yaw, vx, vy, wz, _dt = odom_step
        stamp = self._next_pub_stamp()

        if self.wheel_vel_pub is not None:
            vel_msg = TwistStamped()
            vel_msg.header.stamp = stamp
            vel_msg.header.frame_id = self.base_frame
            vel_msg.twist.linear.x = vx
            vel_msg.twist.linear.y = vy
            vel_msg.twist.angular.z = wz
            self.wheel_vel_pub.publish(vel_msg)

        if self.wheel_odom_pub is not None:
            qx, qy, qz, qw = _yaw_to_quaternion(yaw)
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = self.wheel_odom_frame
            odom_msg.child_frame_id = self.base_frame
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.angular.z = wz
            self.wheel_odom_pub.publish(odom_msg)

    def _publish_status_json(self) -> None:
        with self._lock:
            now_mono = time.monotonic()
            st = self._sm.last_status
            st_time = self._sm.last_status_time
            counters = self._parser.counters
            connected = self._connected
            state = self._sm.state
            stop_reason = self._sm.stop_reason
            payload = {
                "connected": connected,
                "port": self.port,
                "state": state,
                "stop_reason": stop_reason,
                "tx": dict(self._last_tx_meta) if self._last_tx_meta is not None else None,
                "rx_age_s": None if st_time is None else round(now_mono - st_time, 4),
                "chassis": None
                if st is None
                else {
                    "seq": st.seq,
                    "last_seq": st.last_seq,
                    "status_raw": st.status,
                    "ready": st.ready,
                    "enabled": st.enabled,
                    "estop": st.estop,
                    "fault": st.fault,
                    "comm_timeout": st.comm_timeout,
                    "wheel_valid": st.wheel_valid,
                    "sample_time_ms": st.sample_time_ms,
                },
                "parser": {
                    "frames_ok": counters.frames_ok,
                    "bad_header": counters.bad_header,
                    "bad_version": counters.bad_version,
                    "bad_type": counters.bad_type,
                    "bad_length": counters.bad_length,
                    "bad_crc": counters.bad_crc,
                },
            }
            state_key = (connected, state, stop_reason)

        if state_key != self._last_logged_state:
            self._last_logged_state = state_key
            if state == self._sm.STATE_RUNNING:
                self.get_logger().info("Chassis bridge unlocked: RUNNING (mode=1 allowed)")
            else:
                self.get_logger().info(
                    f"Chassis bridge holding stop frames: state={state}, "
                    f"reason={stop_reason}, connected={connected}"
                )

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.status_pub.publish(msg)

    def _on_send_timer(self) -> None:
        ser = self._open_serial_if_due()
        now_ros_ns = self.get_clock().now().nanoseconds

        frame = None
        with self._lock:
            if ser is not None and self._serial is ser:
                now_mono = time.monotonic()
                cmd_fresh = self._has_cmd and (
                    0.0 <= now_mono - self._cmd_receive_mono <= self.cmd_timeout
                )
                if cmd_fresh and self.require_fresh_stamp:
                    stamp_age_ns = now_ros_ns - self._cmd_stamp_ns
                    cmd_fresh = 0 <= stamp_age_ns <= int(self.cmd_timeout * 1e9)
                vx, vy, wz = self._cmd_vx, self._cmd_vy, self._cmd_wz
                frame, meta = self._sm.build_next_command(
                    now_mono, vx, vy, wz, cmd_valid=cmd_fresh
                )
                self._last_tx_meta = meta
            else:
                self._last_tx_meta = None

        if frame is not None and ser is not None:
            try:
                written = ser.write(frame)
                if written is not None and written != len(frame):
                    raise OSError(f"short serial write {written}/{len(frame)}")
            except Exception as exc:
                self.get_logger().warn(
                    f"Serial write failed on {self.port}: {exc}",
                    throttle_duration_sec=2.0,
                )
                self._close_serial("serial_write_error", expected_ser=ser)

        self._publish_status_json()

    def close(self) -> None:
        """Send a best-effort stop frame, stop reader thread, and close serial port."""
        self._stop_event.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        with self._lock:
            ser = self._serial
            stop_frame = self._sm.build_shutdown_stop_frame()
        if ser is not None:
            try:
                ser.write(stop_frame)
                ser.flush()
            except Exception:
                pass
        self._close_serial("node_shutdown")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisLowerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
