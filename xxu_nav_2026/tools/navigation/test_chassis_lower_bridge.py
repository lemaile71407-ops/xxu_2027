#!/usr/bin/env python3
"""Unit and Linux PTY ROS 2 loopback tests for the STM32 chassis lower bridge."""

import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import struct
import sys
import threading
import time
import unittest
from unittest.mock import patch

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TwistStamped
from launch import LaunchContext
from nav_msgs.msg import Odometry
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

WORKSPACE = Path(__file__).resolve().parents[2]
SHARE = WORKSPACE / "src/xxu_hardware"
sys.path.insert(0, str(SHARE))

from xxu_hardware.chassis_protocol import (
    ChassisBridgeStateMachine,
    ChassisStatus,
    FRAME_TYPE_CMD,
    FRAME_TYPE_STATUS,
    FrameStreamParser,
    HEADER_FORMAT,
    HEADER_LEN,
    MODE_RUN,
    MODE_STOP,
    PROTOCOL_VERSION,
    SOF,
    STATUS_ENABLED,
    STATUS_ESTOP,
    STATUS_FAULT,
    STATUS_READY,
    STATUS_TIMEOUT,
    STATUS_WHEEL_VALID,
    WheelOdometryIntegrator,
    build_frame,
    crc16_modbus,
    encode_velocity_command,
    quantize_milli,
)


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


launch_mod = _load_module(
    "chassis_bridge_launch", SHARE / "launch/chassis_lower_bridge.launch.py"
)


def _make_status_frame(
    seq: int,
    status: int,
    last_seq: int,
    vx_mm: int = 0,
    vy_mm: int = 0,
    wz_mrad: int = 0,
    sample_time_ms: int = 0,
) -> bytes:
    payload = struct.pack(
        "<HHhhhI", status, last_seq, vx_mm, vy_mm, wz_mrad, sample_time_ms
    )
    return build_frame(FRAME_TYPE_STATUS, seq, payload)


class ProtocolCodecAndParserTest(unittest.TestCase):
    def test_matches_protocol_doc_hex_samples(self):
        stop_hex = "AA 55 02 10 01 00 07 00 00 00 00 00 00 00 70 DA"
        run_hex = "AA 55 02 10 02 00 07 01 E8 03 0C FE C8 00 F6 5D"
        self.assertEqual(
            encode_velocity_command(1, MODE_STOP, 0.0, 0.0, 0.0),
            bytes.fromhex(stop_hex),
        )
        self.assertEqual(
            encode_velocity_command(2, MODE_RUN, 1.0, -0.5, 0.2),
            bytes.fromhex(run_hex),
        )

        fb_valid_hex = (
            "AA 55 02 20 01 00 0E 23 00 02 00 E8 03 0C FE C8 00 40 E2 01 00 55 28"
        )
        fb_invalid_hex = (
            "AA 55 02 20 02 00 0E 03 00 02 00 00 00 00 00 00 00 00 00 00 00 AE C0"
        )
        parser = FrameStreamParser()
        decoded = parser.feed(bytes.fromhex(fb_valid_hex) + bytes.fromhex(fb_invalid_hex))
        self.assertEqual(len(decoded), 2)

        s1, s2 = decoded
        self.assertEqual((s1.seq, s1.last_seq, s1.sample_time_ms), (1, 2, 123456))
        self.assertTrue(s1.ready and s1.enabled and s1.wheel_valid)
        self.assertFalse(s1.estop or s1.fault or s1.comm_timeout)
        self.assertAlmostEqual(s1.vx_measured, 1.0)
        self.assertAlmostEqual(s1.vy_measured, -0.5)
        self.assertAlmostEqual(s1.wz_measured, 0.2)

        self.assertEqual((s2.seq, s2.last_seq, s2.sample_time_ms), (2, 2, 0))
        self.assertTrue(s2.ready and s2.enabled)
        self.assertFalse(s2.wheel_valid)
        self.assertIsNone(s2.vx_measured)
        self.assertIsNone(s2.vy_measured)
        self.assertIsNone(s2.wz_measured)

    def test_quantize_and_command_validation(self):
        self.assertEqual(quantize_milli(0.0005), 1)
        self.assertEqual(quantize_milli(-0.0005), -1)
        self.assertEqual(quantize_milli(-0.0), 0)
        for bad in (float("nan"), float("inf"), float("-inf"), True, 32.768, -32.769):
            with self.assertRaises(ValueError):
                quantize_milli(bad)
        with self.assertRaises(ValueError):
            encode_velocity_command(1, MODE_STOP, 0.1, 0.0, 0.0)
        with self.assertRaises(ValueError):
            encode_velocity_command(1, 2, 0.0, 0.0, 0.0)

    def test_stream_parser_resync_fragmentation_and_error_counters(self):
        good = _make_status_frame(
            10, STATUS_READY | STATUS_ENABLED | STATUS_WHEEL_VALID, 9, 500, -250, 100, 42
        )
        # Corrupted frame whose first payload bytes contain a real SOF of the good frame.
        bad_crc_prefix = bytearray(_make_status_frame(1, STATUS_READY, 1))
        bad_crc_prefix[-1] ^= 0xFF
        bad_ver = bytearray(_make_status_frame(2, STATUS_READY, 2))
        bad_ver[2] = 0x01
        bad_len = bytearray(build_frame(FRAME_TYPE_STATUS, 3, b"\x00" * 5))
        bad_type = bytearray(build_frame(FRAME_TYPE_CMD, 4, b"\x00" * 7))

        # Place valid frame immediately after an incomplete false SOF prefix.
        stream = (
            b"\x00\xAA\x11"
            + bytes(bad_ver[:4])
            + bytes(bad_len[:7])
            + bytes(bad_type[:7])
            + bytes(bad_crc_prefix)
            + b"\xAA\x55\x02\x20"  # false header immediately followed by real frame
            + good
        )
        parser = FrameStreamParser()
        got = []
        for idx in range(len(stream)):
            got.extend(parser.feed(stream[idx : idx + 1]))
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].seq, 10)
        self.assertEqual(got[0].last_seq, 9)
        self.assertEqual(parser.counters.frames_ok, 1)
        self.assertGreater(parser.counters.bad_crc, 0)
        self.assertGreater(parser.counters.bad_version, 0)
        self.assertGreater(parser.counters.bad_length, 0)
        self.assertGreater(parser.counters.bad_type, 0)


class StateMachineTest(unittest.TestCase):
    def test_startup_handshake_avoids_stale_last_seq_collision(self):
        sm = ChassisBridgeStateMachine()
        # Even with valid non-zero velocity, startup must send mode=0 first.
        _, m1 = sm.build_next_command(0.00, 0.5, 0.0, 0.0, cmd_valid=True)
        self.assertEqual((m1["seq"], m1["mode"], m1["state"]), (1, MODE_STOP, "WAIT_STOP_ACK"))

        # Suppose STM32 already held last_seq=1 from a previous session before receiving new seq=1.
        sm.on_status(
            ChassisStatus(1, STATUS_READY | STATUS_ENABLED, 1, 0, 0, 0, 0), now=0.01
        )
        self.assertEqual(sm.state, "WAIT_STOP_ACK")

        # Next stop frame seq=2 must be acknowledged to unlock RUNNING.
        _, m2 = sm.build_next_command(0.02, 0.5, 0.0, 0.0, cmd_valid=True)
        self.assertEqual((m2["seq"], m2["mode"]), (2, MODE_STOP))
        sm.on_status(
            ChassisStatus(2, STATUS_READY | STATUS_ENABLED, 2, 0, 0, 0, 0), now=0.03
        )
        self.assertEqual(sm.state, "RUNNING")

        _, m3 = sm.build_next_command(0.04, 0.5, 0.0, 0.0, cmd_valid=True)
        self.assertEqual(m3["mode"], MODE_RUN)
        # Zero or invalid upstream command sends mode=0 even when RUNNING.
        _, m4 = sm.build_next_command(0.06, 0.0, 0.0, 0.0, cmd_valid=True)
        self.assertEqual(m4["mode"], MODE_STOP)
        _, m5 = sm.build_next_command(0.08, 0.5, 0.0, 0.0, cmd_valid=False)
        self.assertEqual(m5["mode"], MODE_STOP)

    def test_estop_requires_stop_frame_sent_after_clear(self):
        sm = ChassisBridgeStateMachine()
        sm.on_status(ChassisStatus(1, STATUS_READY, 0, 0, 0, 0, 0), now=0.00)
        _, m1 = sm.build_next_command(0.02, 0.5, 0.0, 0.0, cmd_valid=True)
        sm.on_status(ChassisStatus(2, STATUS_READY, m1["seq"], 0, 0, 0, 0), now=0.03)
        self.assertEqual(sm.state, "RUNNING")

        # E-stop asserts -> immediately falls back to WAIT_STOP_ACK.
        sm.on_status(ChassisStatus(3, STATUS_READY | STATUS_ESTOP, m1["seq"], 0, 0, 0, 0), now=0.04)
        self.assertEqual(sm.state, "WAIT_STOP_ACK")

        # Stop frame sent WHILE E-stop is still active.
        _, during_estop = sm.build_next_command(0.06, 0.5, 0.0, 0.0, cmd_valid=True)
        self.assertEqual(during_estop["mode"], MODE_STOP)

        # E-stop clears while last_seq still echoes the frame sent during E-stop -> MUST NOT unlock!
        sm.on_status(ChassisStatus(4, STATUS_READY, during_estop["seq"], 0, 0, 0, 0), now=0.07)
        self.assertEqual(sm.state, "WAIT_STOP_ACK")

        # Stop frame sent AFTER E-stop cleared unlocks RUNNING.
        _, after_clear = sm.build_next_command(0.08, 0.5, 0.0, 0.0, cmd_valid=True)
        self.assertEqual(after_clear["mode"], MODE_STOP)
        sm.on_status(ChassisStatus(5, STATUS_READY, after_clear["seq"], 0, 0, 0, 0), now=0.09)
        self.assertEqual(sm.state, "RUNNING")

    def test_comm_timeout_and_one_way_stall_recover_without_deadlock(self):
        sm = ChassisBridgeStateMachine(status_timeout_s=0.15, ack_timeout_s=0.25)
        sm.on_status(ChassisStatus(1, STATUS_READY, 0, 0, 0, 0, 0), now=0.00)
        _, m1 = sm.build_next_command(0.02, 0.0, 0.0, 0.0, cmd_valid=True)
        sm.on_status(ChassisStatus(2, STATUS_READY, m1["seq"], 0, 0, 0, 0), now=0.03)
        self.assertEqual(sm.state, "RUNNING")

        # Case 1: STM32 reports bit4=1 while 0x20 still arrives -> host drops to WAIT_STOP_ACK and recovers.
        sm.on_status(
            ChassisStatus(3, STATUS_READY | STATUS_TIMEOUT, m1["seq"], 0, 0, 0, 0),
            now=0.05,
        )
        self.assertEqual(sm.state, "WAIT_STOP_ACK")
        _, m_stop = sm.build_next_command(0.06, 0.5, 0.0, 0.0, cmd_valid=True)
        self.assertEqual(m_stop["mode"], MODE_STOP)
        sm.on_status(ChassisStatus(4, STATUS_READY, m_stop["seq"], 0, 0, 0, 0), now=0.07)
        self.assertEqual(sm.state, "RUNNING")

        # Case 2: Silent stall (0x20 keeps coming with frozen last_seq while mode=1 is sent).
        t = 0.10
        for step in range(15):
            _, m_run = sm.build_next_command(t, 0.5, 0.0, 0.0, cmd_valid=True)
            if t - 0.07 <= 0.25:
                self.assertEqual(m_run["mode"], MODE_RUN)
            sm.on_status(ChassisStatus(5 + step, STATUS_READY, m_stop["seq"], 0, 0, 0, 0), now=t + 0.01)
            t += 0.02
        self.assertEqual(sm.state, "WAIT_STOP_ACK")
        self.assertEqual(sm.stop_reason, "cmd_ack_stall")

    def test_status_rx_timeout_and_fast_reboot_detection(self):
        sm = ChassisBridgeStateMachine(status_timeout_s=0.15)
        sm.on_status(ChassisStatus(1, STATUS_READY, 0, 0, 0, 0, 0), now=0.00)
        _, m1 = sm.build_next_command(0.02, 0.0, 0.0, 0.0, cmd_valid=True)
        sm.on_status(ChassisStatus(2, STATUS_READY, m1["seq"], 0, 0, 0, 0), now=0.03)
        self.assertEqual(sm.state, "RUNNING")

        # Fast reboot: STM32 resets last_seq to 0 within <150ms.
        sm.on_status(ChassisStatus(1, STATUS_READY, 0, 0, 0, 0, 0), now=0.05)
        self.assertEqual(sm.state, "WAIT_STOP_ACK")
        self.assertEqual(sm.stop_reason, "stm32_reboot_detected")

        _, m2 = sm.build_next_command(0.06, 0.3, 0.0, 0.0, cmd_valid=True)
        sm.on_status(ChassisStatus(2, STATUS_READY, m2["seq"], 0, 0, 0, 0), now=0.07)
        self.assertEqual(sm.state, "RUNNING")

        # 150ms without 0x20 triggers host timeout -> sends mode=0.
        _, m3 = sm.build_next_command(0.25, 0.3, 0.0, 0.0, cmd_valid=True)
        self.assertEqual(m3["mode"], MODE_STOP)
        self.assertEqual(sm.state, "WAIT_STOP_ACK")
        self.assertEqual(sm.stop_reason, "status_rx_timeout")

    def test_wheel_odom_integrator_ignores_invalid_and_duplicate_and_wraps(self):
        integ = WheelOdometryIntegrator()
        self.assertIsNone(integ.update(ChassisStatus(1, STATUS_READY, 1, 500, 0, 0, 100)))
        step1 = integ.update(
            ChassisStatus(2, STATUS_READY | STATUS_WHEEL_VALID, 1, 1000, 0, 0, 0xFFFFFFF0)
        )
        self.assertIsNotNone(step1)
        # Repeated sample_time_ms must be ignored.
        self.assertIsNone(
            integ.update(
                ChassisStatus(
                    3, STATUS_READY | STATUS_WHEEL_VALID, 1, 1000, 0, 0, 0xFFFFFFF0
                )
            )
        )
        # Wraparound across 2^32 by +36ms -> integrates +0.036m in x.
        step2 = integ.update(
            ChassisStatus(4, STATUS_READY | STATUS_WHEEL_VALID, 1, 1000, 0, 0, 0x00000014)
        )
        self.assertIsNotNone(step2)
        self.assertAlmostEqual(step2[0], 0.036, places=6)
        self.assertAlmostEqual(step2[6], 0.036, places=6)

    def test_seq_rollover_around_65535_does_not_trigger_false_reboot(self):
        sm = ChassisBridgeStateMachine()
        sm._seq = 65533
        sm.on_status(ChassisStatus(1, STATUS_READY, 0, 0, 0, 0, 0), now=0.00)
        _, stop_meta = sm.build_next_command(0.02, 0.0, 0.0, 0.0, cmd_valid=True)
        sm.on_status(ChassisStatus(2, STATUS_READY, stop_meta["seq"], 0, 0, 0, 0), now=0.03)
        self.assertEqual(sm.state, "RUNNING")

        t = 0.04
        for step in range(45):
            _, meta = sm.build_next_command(t, 0.2, 0.0, 0.0, cmd_valid=True)
            self.assertEqual(meta["mode"], MODE_RUN)
            # Echo seq=0 once rollover reaches 0 and hold briefly across multiple frames.
            echo_seq = 0 if 2 <= step < 8 else meta["seq"]
            sm.on_status(ChassisStatus(3 + step, STATUS_READY, echo_seq, 0, 0, 0, 0), now=t + 0.01)
            self.assertEqual(sm.state, "RUNNING")
            t += 0.02


class FakeSTM32Firmware:
    """Simulated STM32 implementing protocol v0.2 over a Linux PTY fd."""

    def __init__(self, master_fd: int):
        self.fd = master_fd
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.buf = bytearray()
        self.waiting_for_stop = True
        self.ready = True
        self.enabled = True
        self.estop = False
        self.fault = False
        self.comm_timeout = False
        self.wheel_valid = False
        self.last_seq = 0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_valid_cmd_time = time.monotonic()
        self.tx_seq = 0
        self.sample_time_ms = 100
        self.drop_rx = False
        self.received_cmds: list[tuple[int, int, int, int, int]] = []
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def close(self):
        self.stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)

    def _run(self):
        os.set_blocking(self.fd, False)
        next_tx = time.monotonic()
        while not self.stop_event.is_set():
            try:
                chunk = os.read(self.fd, 512)
                if chunk:
                    with self.lock:
                        drop_rx = self.drop_rx
                    if drop_rx:
                        self.buf.clear()
                    else:
                        self._on_rx(chunk)
            except BlockingIOError:
                pass
            except OSError:
                break

            now = time.monotonic()
            with self.lock:
                if (now - self.last_valid_cmd_time) > 0.15 and not self.comm_timeout:
                    self.comm_timeout = True
                    self.waiting_for_stop = True
                    self.vx = self.vy = self.wz = 0.0
                if now >= next_tx:
                    next_tx = now + 0.02
                    self._send_status()
            time.sleep(0.005)

    def _on_rx(self, chunk: bytes):
        self.buf.extend(chunk)
        min_len = HEADER_LEN + 2
        while len(self.buf) >= min_len:
            idx = self.buf.find(SOF)
            if idx == -1:
                self.buf = self.buf[-1:] if self.buf[-1:] == SOF[:1] else bytearray()
                return
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < min_len:
                return
            _, ver, ftype, seq, length = struct.unpack_from(HEADER_FORMAT, self.buf, 0)
            if ver != PROTOCOL_VERSION or ftype != FRAME_TYPE_CMD or length != 7:
                del self.buf[:1]
                continue
            total = HEADER_LEN + length + 2
            if len(self.buf) < total:
                return
            frame = bytes(self.buf[:total])
            got_crc = struct.unpack_from("<H", frame, HEADER_LEN + length)[0]
            if crc16_modbus(frame[:-2]) != got_crc:
                del self.buf[:1]
                continue
            del self.buf[:total]
            mode, q_vx, q_vy, q_wz = struct.unpack_from("<Bhhh", frame, HEADER_LEN)
            if mode not in (MODE_STOP, MODE_RUN):
                continue
            if mode == MODE_STOP and (q_vx or q_vy or q_wz):
                continue
            now = time.monotonic()
            with self.lock:
                self.received_cmds.append((seq, mode, q_vx, q_vy, q_wz))
                if self.estop or self.fault or not self.ready:
                    self.vx = self.vy = self.wz = 0.0
                    continue
                if self.waiting_for_stop:
                    if mode == MODE_STOP:
                        self.waiting_for_stop = False
                        self.comm_timeout = False
                        self.last_seq = seq
                        self.last_valid_cmd_time = now
                        self.vx = self.vy = self.wz = 0.0
                    continue
                self.last_seq = seq
                self.last_valid_cmd_time = now
                if mode == MODE_STOP:
                    self.vx = self.vy = self.wz = 0.0
                else:
                    self.vx = q_vx * 0.001
                    self.vy = q_vy * 0.001
                    self.wz = q_wz * 0.001

    def _send_status(self):
        self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        bits = 0
        if self.ready:
            bits |= STATUS_READY
        if self.enabled:
            bits |= STATUS_ENABLED
        if self.estop:
            bits |= STATUS_ESTOP
        if self.fault:
            bits |= STATUS_FAULT
        if self.comm_timeout:
            bits |= STATUS_TIMEOUT
        if self.wheel_valid:
            bits |= STATUS_WHEEL_VALID
            self.sample_time_ms = (self.sample_time_ms + 20) & 0xFFFFFFFF
            vx_mm = quantize_milli(self.vx)
            vy_mm = quantize_milli(self.vy)
            wz_mrad = quantize_milli(self.wz)
            sample = self.sample_time_ms
        else:
            vx_mm = vy_mm = wz_mrad = sample = 0
        frame = _make_status_frame(
            self.tx_seq, bits, self.last_seq, vx_mm, vy_mm, wz_mrad, sample
        )
        try:
            os.write(self.fd, frame)
        except OSError:
            pass


from test_cpp_bridge_pipeline import CppHarness


class RosPtyLoopbackTest(CppHarness):
    def test_launch_selects_cpp_component_executable(self):
        captured = []
        with patch.object(launch_mod, 'Node', side_effect=lambda **kw: captured.append(kw) or kw):
            launch_mod.generate_launch_description()
        self.assertEqual(captured[0]['executable'], 'chassis_lower_bridge')

    def test_end_to_end_cpp_pty_loopback(self):
        master, slave = pty.openpty()
        firmware = FakeSTM32Firmware(master)
        firmware.start()
        try:
            process = self.start('chassis_lower_bridge', {
                'port': os.ttyname(slave), 'cmd_vel_topic': '/probe/chassis_cmd',
                'status_topic': '/probe/chassis_status',
                'wheel_vel_topic': '/probe/wheel_vel', 'wheel_odom_topic': '/probe/wheel_odom'},
                package='xxu_hardware')
            statuses = self.received(String, '/probe/chassis_status')
            odoms = self.received(Odometry, '/probe/wheel_odom')
            pub = self.node.create_publisher(TwistStamped, '/probe/chassis_cmd', 10)
            def drive():
                command = TwistStamped(); command.header.frame_id = 'base_link'
                command.header.stamp = self.node.get_clock().now().to_msg()
                command.twist.linear.x = .4; command.twist.linear.y = -.2; command.twist.angular.z = .1
                pub.publish(command)
            self.spin(1.3, drive)
            self.assertTrue(statuses)
            self.assertEqual(json.loads(statuses[-1].data)['state'], 'RUNNING')
            self.assertTrue(any(mode == MODE_RUN and vx == 400 and vy == -200 and wz == 100
                                for _,mode,vx,vy,wz in firmware.received_cmds))
            self.assertEqual(odoms, [])
            with firmware.lock: firmware.wheel_valid = True
            self.spin(.3, drive)
            self.assertGreater(len(odoms), 3)
            self.assertGreater(odoms[-1].pose.pose.position.x, 0.)
            self.assertEqual(odoms[-1].child_frame_id, 'base_link')
            stamps = [m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec for m in odoms]
            self.assertEqual(stamps, sorted(set(stamps)))
            with firmware.lock:
                firmware.estop = True
                firmware.waiting_for_stop = True
            self.spin(.12, drive)
            self.assertEqual(json.loads(statuses[-1].data)['state'], 'WAIT_STOP_ACK')
            self.assertTrue(all(mode == MODE_STOP for _,mode,*_ in firmware.received_cmds[-3:]))
            with firmware.lock: firmware.estop = False
            self.spin(.3, drive)
            self.assertEqual(json.loads(statuses[-1].data)['state'], 'RUNNING')
            self.spin(.4)
            self.assertTrue(all(mode == MODE_STOP for _,mode,*_ in firmware.received_cmds[-3:]))
            with firmware.lock: firmware.drop_rx = True
            self.spin(.4, drive)
            self.assertEqual(json.loads(statuses[-1].data)['state'], 'WAIT_STOP_ACK')
            with firmware.lock: firmware.drop_rx = False
            self.spin(.4, drive)
            self.assertEqual(json.loads(statuses[-1].data)['state'], 'RUNNING')
            # Shutdown must emit an explicit stop even after ROS context shutdown.
            process.send_signal(__import__('signal').SIGINT)
            process.wait(timeout=5)
            time.sleep(.05)
            self.assertEqual(firmware.received_cmds[-1][1], MODE_STOP)
        finally:
            firmware.close(); os.close(master); os.close(slave)


if __name__ == '__main__':
    unittest.main()
