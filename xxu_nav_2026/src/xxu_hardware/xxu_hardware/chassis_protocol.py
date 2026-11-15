"""STM32 chassis binary protocol codec and stop/recovery state machine."""

from collections import OrderedDict, deque
from dataclasses import dataclass
import math
import struct

SOF = b"\xAA\x55"
PROTOCOL_VERSION = 0x02
FRAME_TYPE_CMD = 0x10
FRAME_TYPE_STATUS = 0x20

HEADER_FORMAT = "<2sBBHB"
HEADER_LEN = struct.calcsize(HEADER_FORMAT)
CMD_PAYLOAD_FORMAT = "<Bhhh"
CMD_PAYLOAD_LEN = struct.calcsize(CMD_PAYLOAD_FORMAT)
STATUS_PAYLOAD_FORMAT = "<HHhhhI"
STATUS_PAYLOAD_LEN = struct.calcsize(STATUS_PAYLOAD_FORMAT)
CRC_FORMAT = "<H"
CRC_LEN = struct.calcsize(CRC_FORMAT)
MAX_PAYLOAD_LEN = 64

MODE_STOP = 0
MODE_RUN = 1

STATUS_READY = 1 << 0
STATUS_ENABLED = 1 << 1
STATUS_ESTOP = 1 << 2
STATUS_FAULT = 1 << 3
STATUS_TIMEOUT = 1 << 4
STATUS_WHEEL_VALID = 1 << 5
STATUS_KNOWN_MASK = (
    STATUS_READY
    | STATUS_ENABLED
    | STATUS_ESTOP
    | STATUS_FAULT
    | STATUS_TIMEOUT
    | STATUS_WHEEL_VALID
)

INT16_MIN = -32768
INT16_MAX = 32767
DEFAULT_COMMAND_RATE_HZ = 50.0
DEFAULT_STATUS_TIMEOUT_S = 0.15
DEFAULT_ACK_TIMEOUT_S = 0.25


def crc16_modbus(data: bytes) -> int:
    """Compute CRC16/MODBUS (init 0xFFFF, reflected polynomial 0xA001)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def quantize_milli(value: float, name: str = "value") -> int:
    """Convert meters/s or rad/s to int16 milli-units using round-half-away-from-zero."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    scaled = number * 1000.0
    if scaled == 0.0:
        quantized = 0
    else:
        quantized = int(math.copysign(math.floor(abs(scaled) + 0.5), scaled))
    if not INT16_MIN <= quantized <= INT16_MAX:
        raise ValueError(f"{name} quantized value {quantized} exceeds int16 range")
    return quantized


def build_frame(frame_type: int, seq: int, payload: bytes) -> bytes:
    """Wrap a payload with protocol header and little-endian CRC16/MODBUS."""
    if not 0 <= frame_type <= 0xFF:
        raise ValueError("frame_type out of uint8 range")
    if not 0 <= seq <= 0xFFFF:
        raise ValueError("seq out of uint16 range")
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError("payload exceeds maximum length")
    header = struct.pack(
        HEADER_FORMAT, SOF, PROTOCOL_VERSION, frame_type, seq, len(payload)
    )
    body = header + payload
    crc = struct.pack(CRC_FORMAT, crc16_modbus(body))
    return body + crc


def encode_velocity_command(
    seq: int, mode: int, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0
) -> bytes:
    """Encode a type=0x10 chassis velocity command frame."""
    if mode not in (MODE_STOP, MODE_RUN):
        raise ValueError(f"invalid command mode: {mode}")
    q_vx = quantize_milli(vx, "vx")
    q_vy = quantize_milli(vy, "vy")
    q_wz = quantize_milli(wz, "wz")
    if mode == MODE_STOP and (q_vx != 0 or q_vy != 0 or q_wz != 0):
        raise ValueError("mode=0 requires zero vx, vy, and wz")
    payload = struct.pack(CMD_PAYLOAD_FORMAT, mode, q_vx, q_vy, q_wz)
    return build_frame(FRAME_TYPE_CMD, seq, payload)


@dataclass(frozen=True)
class ChassisStatus:
    """Decoded type=0x20 chassis status and optional wheel velocity feedback."""

    seq: int
    status: int
    last_seq: int
    vx_raw: int
    vy_raw: int
    wz_raw: int
    sample_time_ms: int

    @property
    def ready(self) -> bool:
        return bool(self.status & STATUS_READY)

    @property
    def enabled(self) -> bool:
        return bool(self.status & STATUS_ENABLED)

    @property
    def estop(self) -> bool:
        return bool(self.status & STATUS_ESTOP)

    @property
    def fault(self) -> bool:
        return bool(self.status & STATUS_FAULT)

    @property
    def comm_timeout(self) -> bool:
        return bool(self.status & STATUS_TIMEOUT)

    @property
    def wheel_valid(self) -> bool:
        return bool(self.status & STATUS_WHEEL_VALID)

    @property
    def reserved_bits(self) -> int:
        return self.status & (~STATUS_KNOWN_MASK & 0xFFFF)

    @property
    def vx_measured(self) -> float | None:
        return self.vx_raw * 0.001 if self.wheel_valid else None

    @property
    def vy_measured(self) -> float | None:
        return self.vy_raw * 0.001 if self.wheel_valid else None

    @property
    def wz_measured(self) -> float | None:
        return self.wz_raw * 0.001 if self.wheel_valid else None

    def is_healthy_to_run(self, require_run_enable: bool = False) -> bool:
        """Return whether chassis state allows mode=1 motion commands."""
        return (
            self.ready
            and not self.estop
            and not self.fault
            and not self.comm_timeout
            and (not require_run_enable or self.enabled)
        )

    def requires_post_reset_stop_ack(self, require_run_enable: bool = False) -> bool:
        """Return whether hardware safety state requires a new stop frame after clear."""
        return (
            not self.ready
            or self.estop
            or self.fault
            or (require_run_enable and not self.enabled)
        )


@dataclass
class ParserCounters:
    """Diagnostic counters for streaming binary frame decoding."""

    frames_ok: int = 0
    bad_header: int = 0
    bad_version: int = 0
    bad_type: int = 0
    bad_length: int = 0
    bad_crc: int = 0


class FrameStreamParser:
    """Incremental byte-stream parser for STM32 -> host type=0x20 frames."""

    def __init__(self):
        self._buf = bytearray()
        self.counters = ParserCounters()

    def clear(self) -> None:
        self._buf.clear()

    def feed(self, chunk: bytes) -> list[ChassisStatus]:
        """Consume raw serial bytes and return all valid decoded status frames."""
        if chunk:
            self._buf.extend(chunk)
        frames = []
        min_frame_len = HEADER_LEN + CRC_LEN
        while len(self._buf) >= min_frame_len:
            sof_idx = self._buf.find(SOF)
            if sof_idx == -1:
                keep = 1 if self._buf[-1] == SOF[0] else 0
                drop = len(self._buf) - keep
                if drop > 0:
                    self.counters.bad_header += drop
                    del self._buf[:drop]
                break
            if sof_idx > 0:
                self.counters.bad_header += sof_idx
                del self._buf[:sof_idx]
            if len(self._buf) < min_frame_len:
                break

            _, version, frame_type, seq, length = struct.unpack_from(
                HEADER_FORMAT, self._buf, 0
            )
            if version != PROTOCOL_VERSION:
                self.counters.bad_version += 1
                del self._buf[:1]
                continue
            if frame_type != FRAME_TYPE_STATUS:
                self.counters.bad_type += 1
                del self._buf[:1]
                continue
            if length > MAX_PAYLOAD_LEN or length != STATUS_PAYLOAD_LEN:
                self.counters.bad_length += 1
                del self._buf[:1]
                continue

            frame_len = HEADER_LEN + length + CRC_LEN
            if len(self._buf) < frame_len:
                break

            frame_bytes = bytes(self._buf[:frame_len])
            got_crc = struct.unpack_from(CRC_FORMAT, frame_bytes, HEADER_LEN + length)[0]
            if crc16_modbus(frame_bytes[:-CRC_LEN]) != got_crc:
                self.counters.bad_crc += 1
                # Advance by 1 byte so a real SOF inside corrupted bytes is not skipped.
                del self._buf[:1]
                continue

            del self._buf[:frame_len]
            status, last_seq, vx_raw, vy_raw, wz_raw, sample_time_ms = (
                struct.unpack_from(STATUS_PAYLOAD_FORMAT, frame_bytes, HEADER_LEN)
            )
            self.counters.frames_ok += 1
            frames.append(
                ChassisStatus(
                    seq=seq,
                    status=status,
                    last_seq=last_seq,
                    vx_raw=vx_raw,
                    vy_raw=vy_raw,
                    wz_raw=wz_raw,
                    sample_time_ms=sample_time_ms,
                )
            )
        return frames


class WheelOdometryIntegrator:
    """Dead-reckon planar pose solely from valid, newly sampled wheel velocities."""

    def __init__(self, max_dt_s: float = 0.5):
        if not math.isfinite(max_dt_s) or max_dt_s <= 0.0:
            raise ValueError("max_dt_s must be finite and positive")
        self.max_dt_s = float(max_dt_s)
        self.reset()

    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._last_sample_time_ms: int | None = None

    def update(
        self, status: ChassisStatus
    ) -> tuple[float, float, float, float, float, float, float] | None:
        """Update planar pose if bit5=1 and sample_time_ms advanced; else return None."""
        if not status.wheel_valid:
            return None
        sample_ms = status.sample_time_ms
        if self._last_sample_time_ms is None:
            self._last_sample_time_ms = sample_ms
            return (
                self.x,
                self.y,
                self.yaw,
                status.vx_raw * 0.001,
                status.vy_raw * 0.001,
                status.wz_raw * 0.001,
                0.0,
            )
        if sample_ms == self._last_sample_time_ms:
            return None

        delta_ms = (sample_ms - self._last_sample_time_ms) & 0xFFFFFFFF
        self._last_sample_time_ms = sample_ms
        dt = delta_ms * 0.001
        vx = status.vx_raw * 0.001
        vy = status.vy_raw * 0.001
        wz = status.wz_raw * 0.001
        if delta_ms == 0 or delta_ms > 0x80000000 or dt > self.max_dt_s:
            # Skip integration across wraparound glitches, STM32 reboot, or long gaps.
            return (self.x, self.y, self.yaw, vx, vy, wz, 0.0)

        mid_yaw = self.yaw + 0.5 * wz * dt
        cos_yaw = math.cos(mid_yaw)
        sin_yaw = math.sin(mid_yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        return (self.x, self.y, self.yaw, vx, vy, wz, dt)


class ChassisBridgeStateMachine:
    """Host-side stop/recovery and velocity command framing state machine."""

    STATE_WAIT_STOP_ACK = "WAIT_STOP_ACK"
    STATE_RUNNING = "RUNNING"

    def __init__(
        self,
        *,
        status_timeout_s: float = DEFAULT_STATUS_TIMEOUT_S,
        ack_timeout_s: float = DEFAULT_ACK_TIMEOUT_S,
        require_run_enable: bool = False,
    ):
        if not math.isfinite(status_timeout_s) or status_timeout_s <= 0.0:
            raise ValueError("status_timeout_s must be finite and positive")
        if not math.isfinite(ack_timeout_s) or ack_timeout_s < 0.0:
            raise ValueError("ack_timeout_s must be finite and nonnegative")
        self.status_timeout_s = float(status_timeout_s)
        self.ack_timeout_s = float(ack_timeout_s)
        self.require_run_enable = bool(require_run_enable)

        self._seq = 1
        self._order_counter = 0
        self._sent_cmds: OrderedDict[int, tuple[int, float]] = OrderedDict()
        self._pending_stop_seqs: OrderedDict[int, tuple[int, float]] = OrderedDict()
        self._recent_sent_seqs: deque[int] = deque(maxlen=128)

        self.state = self.STATE_WAIT_STOP_ACK
        self.stop_reason = "startup"
        self._last_status: ChassisStatus | None = None
        self._last_status_time: float | None = None
        self._baseline_last_seq: int | None = None
        self._saw_status_in_wait = False
        self._saw_nonzero_last_seq = False
        self._last_acked_order_id = 0
        self._last_ack_rx_time: float | None = None

    @property
    def last_status(self) -> ChassisStatus | None:
        return self._last_status

    @property
    def last_status_time(self) -> float | None:
        return self._last_status_time

    def _enter_wait_stop(self, reason: str, now: float) -> None:
        self.state = self.STATE_WAIT_STOP_ACK
        self.stop_reason = reason
        self._pending_stop_seqs.clear()
        self._last_ack_rx_time = None
        status_fresh = (
            self._last_status is not None
            and self._last_status_time is not None
            and 0.0 <= (now - self._last_status_time) <= self.status_timeout_s
        )
        if status_fresh:
            self._baseline_last_seq = self._last_status.last_seq
            self._saw_status_in_wait = True
        else:
            self._baseline_last_seq = None
            self._saw_status_in_wait = False

    def on_link_reset(self, now: float, reason: str = "link_reset") -> None:
        """Reset link state on serial open/close/disconnect and require stop ack."""
        self._last_status = None
        self._last_status_time = None
        self._saw_nonzero_last_seq = False
        self._sent_cmds.clear()
        self._enter_wait_stop(reason, now)

    @staticmethod
    def _unhealthy_reason(status: ChassisStatus, require_run_enable: bool) -> str:
        reasons = []
        if not status.ready:
            reasons.append("not_ready")
        if status.estop:
            reasons.append("estop")
        if status.fault:
            reasons.append("driver_fault")
        if status.comm_timeout:
            reasons.append("stm32_comm_timeout")
        if require_run_enable and not status.enabled:
            reasons.append("run_disabled")
        return "+".join(reasons) if reasons else "unhealthy"

    def _detect_reboot(
        self, prev_status: ChassisStatus | None, status: ChassisStatus
    ) -> bool:
        if self._saw_nonzero_last_seq and status.last_seq == 0:
            if 0 not in self._recent_sent_seqs:
                return True
        if (
            prev_status is not None
            and prev_status.wheel_valid
            and status.wheel_valid
        ):
            delta_ms = (status.sample_time_ms - prev_status.sample_time_ms) & 0xFFFFFFFF
            if delta_ms > 0x80000000:
                return True
        return False

    def on_status(self, status: ChassisStatus, now: float) -> None:
        """Process a newly decoded 0x20 status frame at monotonic time `now`."""
        prev_status = self._last_status
        rebooted = (
            self.state == self.STATE_RUNNING
            and self._detect_reboot(prev_status, status)
        )
        self._last_status = status
        self._last_status_time = now
        if status.last_seq != 0:
            self._saw_nonzero_last_seq = True

        acked = self._sent_cmds.get(status.last_seq)
        if acked is not None and acked[0] > self._last_acked_order_id:
            self._last_acked_order_id = acked[0]
            self._last_ack_rx_time = now

        if self.state == self.STATE_RUNNING:
            if rebooted:
                self._enter_wait_stop("stm32_reboot_detected", now)
                return
            if not status.is_healthy_to_run(self.require_run_enable):
                self._enter_wait_stop(
                    self._unhealthy_reason(status, self.require_run_enable), now
                )
            return

        if not self._saw_status_in_wait:
            self._saw_status_in_wait = True
            self._baseline_last_seq = status.last_seq

        if status.requires_post_reset_stop_ack(self.require_run_enable):
            # Hardware fault/E-stop/not-ready requires a new stop frame after clearing.
            self._pending_stop_seqs.clear()
            self._baseline_last_seq = status.last_seq
            self.stop_reason = self._unhealthy_reason(status, self.require_run_enable)
            return

        if status.comm_timeout:
            # Stop frames sent now are what clear bit4; keep pending_stop_seqs alive.
            self.stop_reason = "stm32_comm_timeout"
            return

        stop_entry = self._pending_stop_seqs.get(status.last_seq)
        if stop_entry is not None and (
            self._baseline_last_seq is None
            or status.last_seq != self._baseline_last_seq
        ):
            self.state = self.STATE_RUNNING
            self.stop_reason = ""
            self._pending_stop_seqs.clear()
            self._last_acked_order_id = max(self._last_acked_order_id, stop_entry[0])
            self._last_ack_rx_time = now

    def check_timeouts(self, now: float) -> None:
        """Enforce host-side 150 ms status timeout and command-ack stall protection."""
        status_timed_out = (
            self._last_status_time is None
            or (now - self._last_status_time) < 0.0
            or (now - self._last_status_time) > self.status_timeout_s
        )
        if self.state == self.STATE_RUNNING and status_timed_out:
            self._enter_wait_stop("status_rx_timeout", now)
            return

        if (
            self.state == self.STATE_RUNNING
            and self.ack_timeout_s > 0.0
            and self._last_ack_rx_time is not None
            and (now - self._last_ack_rx_time) > self.ack_timeout_s
        ):
            self._enter_wait_stop("cmd_ack_stall", now)

    def build_shutdown_stop_frame(self) -> bytes:
        """Encode an immediate zero-speed stop frame on node shutdown."""
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        return encode_velocity_command(seq, MODE_STOP, 0.0, 0.0, 0.0)

    def build_next_command(
        self,
        now: float,
        vx: float = 0.0,
        vy: float = 0.0,
        wz: float = 0.0,
        *,
        cmd_valid: bool = False,
    ) -> tuple[bytes, dict]:
        """Build the next 50 Hz type=0x10 frame according to protocol and safety state."""
        self.check_timeouts(now)

        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        self._order_counter += 1
        order_id = self._order_counter
        self._recent_sent_seqs.append(seq)

        mode = MODE_STOP
        out_vx = 0.0
        out_vy = 0.0
        out_wz = 0.0
        invalid_reason = ""

        if self.state == self.STATE_RUNNING and cmd_valid:
            try:
                q_vx = quantize_milli(vx, "vx")
                q_vy = quantize_milli(vy, "vy")
                q_wz = quantize_milli(wz, "wz")
                if q_vx != 0 or q_vy != 0 or q_wz != 0:
                    mode = MODE_RUN
                    out_vx = float(vx)
                    out_vy = float(vy)
                    out_wz = float(wz)
            except ValueError as exc:
                invalid_reason = str(exc)

        frame = encode_velocity_command(seq, mode, out_vx, out_vy, out_wz)
        self._sent_cmds[seq] = (order_id, now)
        while len(self._sent_cmds) > 128:
            self._sent_cmds.popitem(last=False)

        if self.state == self.STATE_WAIT_STOP_ACK and mode == MODE_STOP:
            can_unlock_after_ack = (
                self._last_status is not None
                and not self._last_status.requires_post_reset_stop_ack(
                    self.require_run_enable
                )
            )
            if can_unlock_after_ack:
                self._pending_stop_seqs[seq] = (order_id, now)
                while len(self._pending_stop_seqs) > 128:
                    self._pending_stop_seqs.popitem(last=False)

        meta = {
            "seq": seq,
            "mode": mode,
            "state": self.state,
            "stop_reason": self.stop_reason,
            "invalid_reason": invalid_reason,
        }
        return frame, meta
