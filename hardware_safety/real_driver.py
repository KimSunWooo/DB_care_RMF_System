"""Real Pinky command gate. The default transport never opens /dev/ttyAMA4.

Torque and goal velocity stay off unless a caller passes an explicit operator
stop approval. Bus watchdog is recorded and is not used as the cmd_vel timeout.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from hardware_safety.session import OdomSession, SafetyConfig, load_config

MODEL_XL330_M288 = 1200
REAL_PORT = "/dev/ttyAMA4"
SERVO_IDS = (1, 2)
COMMAND_TIMEOUT_SEC = 0.4
TRIAL_MAX_LINEAR_M_S = 0.03
TRIAL_MAX_ANGULAR_RAD_S = 0.2
TRIAL_MAX_SECONDS = 5.0
TRIAL_MAX_DISTANCE_M = 0.10
VELOCITY_SETTLED_RAW = 5


@dataclass(frozen=True)
class ServoSample:
    servo_id: int
    model: int
    firmware: int
    torque: int
    hardware_error: int
    bus_watchdog: int
    position: int
    velocity: int
    operating_mode: int = -1


class RealTransport:
    """Test double. open() does not touch a serial device."""

    def __init__(self, samples: list[ServoSample], *, stop_ok: bool = True, motion_enabled: bool = False) -> None:
        self.samples = list(samples)
        self.stop_ok = stop_ok
        self.motion_enabled = motion_enabled
        self.port_open = False
        self.velocity_writes: list[tuple[float, float]] = []
        self.stop_writes: list[tuple[float, float]] = []
        self.torque_writes: list[int] = []
        self.watchdog_writes: list[int] = []
        self.fail_read = False

    def open(self) -> None:
        self.port_open = True

    def close(self) -> None:
        self.port_open = False

    def read_samples(self) -> list[ServoSample]:
        if self.fail_read:
            raise ConnectionError("dynamixel read failed")
        return list(self.samples)

    def write_goal_velocity(self, rpm_left: float, rpm_right: float) -> bool:
        if not self.motion_enabled:
            return False
        self.velocity_writes.append((rpm_left, rpm_right))
        return True

    def write_stop(self) -> bool:
        self.stop_writes.append((0.0, 0.0))
        return self.stop_ok

    def write_torque(self, enabled: int) -> bool:
        self.torque_writes.append(int(enabled))
        return self.motion_enabled

    def write_bus_watchdog(self, value: int) -> None:
        self.watchdog_writes.append(int(value))
        raise RuntimeError("refusing a bus watchdog write")


def wheel_rpm(vx: float, wz: float, config: SafetyConfig) -> tuple[float, float]:
    """Match pinky_bringup: the right wheel command is inverted."""
    left = vx - wz * config.wheel_separation_m / 2.0
    right = vx + wz * config.wheel_separation_m / 2.0
    scale = 60.0 / (2.0 * math.pi * config.wheel_radius_m)
    return left * scale, -right * scale


@dataclass
class RealPinkyDriver:
    transport: RealTransport
    config: SafetyConfig
    operator_stop_approved: bool = False
    power_cut_observed: bool = False
    state: str = "idle"
    reason: str = ""
    torque_enabled: bool = False
    publishes_wheel_joints: bool = False
    bus_watchdog_supported: bool = False
    bus_watchdog_is_command_watchdog: bool = False
    stop_write_acked: bool = False
    velocity_settled: bool = False
    trial_started_at: float | None = None
    trial_deadline: float | None = None
    trial_origin: tuple[float, float] | None = None
    trial_distance_m: float = 0.0
    last_command_at: float | None = None
    commanded_vx: float = 0.0
    commanded_wz: float = 0.0
    samples: list[ServoSample] = field(default_factory=list)
    odom: OdomSession | None = None

    def __post_init__(self) -> None:
        self.odom = OdomSession("ODOM_MOCK", self.config.mock_domain_id, self.config)
        refused = self.odom.arm({"encoder_joints"})
        if refused:
            raise RuntimeError(refused)

    def identify(self) -> str | None:
        self.transport.open()
        try:
            self.samples = self.transport.read_samples()
        except Exception as exc:
            self._latch(f"identify read failed: {exc}")
            return self.reason
        ids = tuple(sample.servo_id for sample in self.samples)
        if ids != SERVO_IDS:
            self._latch(f"servo ids are {ids}")
            return self.reason
        for sample in self.samples:
            if sample.model != MODEL_XL330_M288:
                self._latch(f"id {sample.servo_id} model {sample.model} is not {MODEL_XL330_M288}")
                return self.reason
            if sample.hardware_error != 0:
                self._latch(f"id {sample.servo_id} hardware error {sample.hardware_error}")
                return self.reason
            if sample.torque != 0:
                self._latch(f"id {sample.servo_id} torque is already enabled")
                return self.reason
        self.bus_watchdog_supported = True
        self.bus_watchdog_is_command_watchdog = False
        self.state = "identified"
        self.reason = ""
        self._ingest(self.samples, 0.0)
        return None

    def rebase_clock(self, now: float) -> None:
        """Move the seeded feedback stamp onto the caller's clock. No pose change."""
        if self.odom is not None and self.odom.last_feedback_at is not None:
            self.odom.last_feedback_at = now

    def arm(self, other_joint_publishers: set[str]) -> str | None:
        if self.state != "identified":
            self.reason = "identify has not passed"
            return self.reason
        if not self.operator_stop_approved:
            self.reason = "operator has not confirmed an independent stop"
            return self.reason
        if not self.power_cut_observed:
            self.reason = "power cut has not been observed"
            return self.reason
        if any(sample.operating_mode != 1 for sample in self.samples):
            self.reason = "operating mode is not velocity mode 1"
            return self.reason
        self.publishes_wheel_joints = "joint_state_publisher" not in other_joint_publishers
        self.state = "armed"
        self.reason = ""
        return None

    def use_joint_topic(self, other_joint_publishers: set[str]) -> bool:
        """Publish wheel joints only when this process is the only source."""
        self.publishes_wheel_joints = "joint_state_publisher" not in other_joint_publishers
        return self.publishes_wheel_joints

    def enable_torque(self) -> str | None:
        if self.state != "armed" or not self.operator_stop_approved or not self.power_cut_observed:
            self.reason = "torque stays off without arm, operator approval, and an observed power cut"
            return self.reason
        if not self.transport.write_torque(1):
            self.torque_enabled = False
            self.reason = "torque enable was refused"
            return self.reason
        self.torque_enabled = True
        return None

    def start_trial(self, now: float, distance_m: float, seconds: float) -> str | None:
        if self.state != "armed" or not self.torque_enabled:
            self.reason = "trial stays closed until torque is explicitly enabled"
            return self.reason
        if distance_m <= 0 or distance_m > TRIAL_MAX_DISTANCE_M or seconds <= 0 or seconds > TRIAL_MAX_SECONDS:
            self.reason = "trial distance or time is outside the short-move cap"
            return self.reason
        self.trial_started_at = now
        self.trial_deadline = now + seconds
        self.trial_origin = (self.odom.x, self.odom.y)
        self.trial_distance_m = distance_m
        self.state = "trial"
        self.reason = ""
        return None

    def push_command(self, vx: float, wz: float, now: float) -> bool:
        if self.state != "trial":
            self.reason = "cmd_vel is refused outside an approved trial"
            return False
        if not math.isfinite(vx) or not math.isfinite(wz):
            return self._stop(now, "cmd_vel is not finite")
        if abs(vx) > TRIAL_MAX_LINEAR_M_S or abs(wz) > TRIAL_MAX_ANGULAR_RAD_S:
            return self._stop(now, "cmd_vel exceeds the trial speed cap")
        if abs(vx) > self.config.max_linear_m_s or abs(wz) > self.config.max_angular_rad_s:
            return self._stop(now, "cmd_vel exceeds the software speed cap")
        rpm = wheel_rpm(vx, wz, self.config)
        if not self.transport.write_goal_velocity(*rpm):
            return self._stop(now, "goal velocity write failed")
        self.commanded_vx = vx
        self.commanded_wz = wz
        self.last_command_at = now
        self.reason = ""
        return True

    def poll_bus(self, now: float) -> bool:
        try:
            samples = self.transport.read_samples()
        except Exception as exc:
            self._stop(now, f"feedback read failed: {exc}")
            return False
        return self.push_feedback(samples, now)

    def push_feedback(self, samples: list[ServoSample], now: float) -> bool:
        if self.state in {"fault_latched", "idle"}:
            return False
        self.samples = samples
        if any(sample.hardware_error != 0 for sample in samples):
            self._stop(now, "hardware error status is set")
            return False
        accepted = self._ingest(samples, now)
        if not accepted:
            self._stop(now, self.odom.last_reject or "encoder sample rejected")
            return False
        self.velocity_settled = all(abs(sample.velocity) <= VELOCITY_SETTLED_RAW for sample in samples)
        if self.state == "trial" and self._trial_expired(now):
            self._stop(now, "trial distance or time is exhausted")
            return False
        return True

    def poll(self, now: float) -> None:
        if self.state != "trial" or self.last_command_at is None:
            return
        if now - self.last_command_at > COMMAND_TIMEOUT_SEC or (self.trial_deadline is not None and now > self.trial_deadline):
            self._stop(now, "ros command watchdog")

    def recover(self) -> bool:
        if self.state != "stopped" or not self.stop_write_acked:
            self.reason = "recovery needs a stop write acknowledgement"
            return False
        self.torque_enabled = False
        self.odom = OdomSession("ODOM_MOCK", self.config.mock_domain_id, self.config)
        refused = self.odom.arm({"encoder_joints"})
        if refused:
            self._latch(refused)
            return False
        if self.samples:
            self._ingest(self.samples, 0.0)
        self.state = "identified"
        self.reason = ""
        self.last_command_at = None
        self.commanded_vx = 0.0
        self.commanded_wz = 0.0
        return True

    def joint_state(self) -> dict | None:
        if not self.publishes_wheel_joints or len(self.samples) < 2:
            return None
        scale = 2.0 * math.pi / self.config.ticks_per_rev
        by_id = {sample.servo_id: sample for sample in self.samples}
        return {
            "name": ["l_wheel_joint", "r_wheel_joint"],
            "position": [by_id[1].position * scale, by_id[2].position * scale],
        }

    def tf(self) -> dict:
        return self.odom.tf()

    def _ingest(self, samples: list[ServoSample], now: float) -> bool:
        by_id = {sample.servo_id: sample for sample in samples}
        if SERVO_IDS[0] not in by_id or SERVO_IDS[1] not in by_id:
            return False
        return self.odom.push_raw(by_id[1].position, by_id[2].position, now)

    def _trial_expired(self, now: float) -> bool:
        if self.trial_deadline is not None and now > self.trial_deadline:
            return True
        if self.trial_origin is None:
            return False
        moved = math.hypot(self.odom.x - self.trial_origin[0], self.odom.y - self.trial_origin[1])
        return moved >= self.trial_distance_m

    def _stop(self, now: float, reason: str) -> bool:
        self.reason = reason
        self.commanded_vx = 0.0
        self.commanded_wz = 0.0
        self.last_command_at = None
        self.stop_write_acked = self.transport.write_stop()
        if self.torque_enabled:
            self.transport.write_torque(0)
        self.torque_enabled = False
        self.state = "stopped" if self.stop_write_acked else "fault_latched"
        self.odom.measured_vx = 0.0
        self.odom.measured_wz = 0.0
        return False

    def _latch(self, reason: str) -> None:
        self.reason = reason
        self.torque_enabled = False
        self.state = "fault_latched"
        self.transport.close()


def assess_samples(samples: list[ServoSample], config: SafetyConfig | None = None) -> RealPinkyDriver:
    driver = RealPinkyDriver(RealTransport(samples), config or _default_config())
    driver.identify()
    return driver


def _default_config() -> SafetyConfig:
    from pathlib import Path
    return load_config(Path(__file__).resolve().parents[1])
