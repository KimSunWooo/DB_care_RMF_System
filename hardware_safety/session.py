"""Diff-drive encoder math and the command gate around it.

The Pinky driver is not imported. Nothing here opens a serial port or enables torque.
commanded velocity 0 is not a physical stop. The Dynamixel model and bus watchdog
register stay unknown.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

MODES = {"SENSOR_ONLY", "ODOM_MOCK", "HARDWARE"}
TICK_MOD = 2**32
BLOCKED = {
    "refused",
    "init_failed",
    "hardware_blocked",
    "sensor_only",
    "encoder_fault",
    "STOP_REQUIRED",
    "FAULT_LATCHED",
    "RECOVERY_REQUIRED",
}


class SafetyError(Exception):
    pass


class FakeTransport:
    """Injected stop-write result. It has no serial port."""

    def __init__(self, accept_stop: bool = True) -> None:
        self.accept_stop = accept_stop
        self.writes: list[tuple[float, float]] = []
        self.port_opened = False

    def write_stop(self) -> bool:
        self.writes.append((0.0, 0.0))
        return self.accept_stop

    def write_velocity(self, _rpm_left: float, _rpm_right: float) -> bool:
        self.port_opened = False
        return False


@dataclass(frozen=True)
class SafetyConfig:
    mock_domain_id: int
    occupied_domains: tuple[int, ...]
    command_timeout_sec: float
    feedback_timeout_sec: float
    max_linear_m_s: float
    max_angular_rad_s: float
    max_linear_accel_m_s2: float
    max_angular_accel_rad_s2: float
    encoder_margin: float
    nominal_period_sec: float
    max_feedback_dt_sec: float
    max_clock_step_sec: float
    ticks_per_rev: int
    right_raw_sign: int
    wheel_radius_m: float
    wheel_radius_source: str
    wheel_separation_m: float
    wheel_separation_source: str
    urdf_joints: tuple[str, ...]
    driver_joints: tuple[str, ...]
    bus_watchdog_register: int | None


def load_config(root: Path) -> SafetyConfig:
    raw = yaml.safe_load((root / "config" / "hardware" / "odom_safety.yaml").read_text(encoding="utf-8"))
    if raw.get("bus_watchdog_register") is not None:
        raise SafetyError("refusing a guessed Dynamixel bus watchdog register")
    sign = int(raw["right_raw_sign"])
    if sign not in {-1, 1}:
        raise SafetyError("right_raw_sign must be -1 or 1")
    return SafetyConfig(
        mock_domain_id=int(raw["mock_domain_id"]),
        occupied_domains=tuple(int(item) for item in raw["occupied_domains"]),
        command_timeout_sec=float(raw["command_timeout_sec"]),
        feedback_timeout_sec=float(raw["feedback_timeout_sec"]),
        max_linear_m_s=float(raw["max_linear_m_s"]),
        max_angular_rad_s=float(raw["max_angular_rad_s"]),
        max_linear_accel_m_s2=float(raw["max_linear_accel_m_s2"]),
        max_angular_accel_rad_s2=float(raw["max_angular_accel_rad_s2"]),
        encoder_margin=float(raw["encoder_margin"]),
        nominal_period_sec=float(raw["nominal_period_sec"]),
        max_feedback_dt_sec=float(raw["max_feedback_dt_sec"]),
        max_clock_step_sec=float(raw["max_clock_step_sec"]),
        ticks_per_rev=int(raw["ticks_per_rev"]),
        right_raw_sign=sign,
        wheel_radius_m=float(raw["wheel_radius_m"]),
        wheel_radius_source=str(raw["wheel_radius_source"]),
        wheel_separation_m=float(raw["wheel_separation_m"]),
        wheel_separation_source=str(raw["wheel_separation_source"]),
        urdf_joints=tuple(raw["urdf_joints"]),
        driver_joints=tuple(raw["driver_joints"]),
        bus_watchdog_register=None,
    )


def signed_delta(previous: int, current: int) -> int:
    """Shortest step on the 32-bit position register."""
    delta = (int(current) - int(previous)) % TICK_MOD
    if delta >= TICK_MOD // 2:
        delta -= TICK_MOD
    return delta


def max_wheel_ticks(config: SafetyConfig, dt: float) -> int:
    """Ticks one wheel may move at the software speed cap over dt."""
    if dt <= 0 or dt > config.max_feedback_dt_sec:
        return 0
    wheel_speed = config.max_linear_m_s + config.max_angular_rad_s * config.wheel_separation_m / 2.0
    circumference = 2.0 * math.pi * config.wheel_radius_m
    ticks = config.ticks_per_rev * (wheel_speed * dt) / circumference
    return max(1, math.ceil(ticks * config.encoder_margin))


def joint_source_block(mode: str, sources: set[str]) -> str | None:
    if mode not in MODES:
        return f"unknown hardware mode: {mode}"
    if "encoder_joints" in sources and "joint_state_publisher" in sources:
        return "encoder joint_states and joint_state_publisher would publish the same topic"
    if mode == "ODOM_MOCK" and sources != {"encoder_joints"}:
        return "ODOM_MOCK publishes joint_states only from the encoder model"
    if mode == "SENSOR_ONLY" and "encoder_joints" in sources:
        return "SENSOR_ONLY does not start the motor or encoder publisher"
    if mode == "HARDWARE" and sources:
        return "HARDWARE mode does not start a publisher or the motor driver"
    return None


def motor_launch_block(mode: str) -> str:
    if mode not in MODES:
        return f"unknown hardware mode: {mode}"
    return "motor serial port and torque stay disabled in every mode"


def mock_domain_block(domain: int | None, config: SafetyConfig) -> str | None:
    if domain is None or domain in config.occupied_domains:
        return f"refusing mock odom on occupied ROS domain {domain}"
    if domain != config.mock_domain_id:
        return f"mock odom is only allowed on ROS domain {config.mock_domain_id}"
    return None


def framed_outputs(config: SafetyConfig, robot_id: str = "pinky1") -> dict:
    """Names for a namespaced robot description.

    upload_robot passes namespace pinky1/ into the xacro, so joint names are
    pinky1/l_wheel_joint. frame_prefix does the same for link frames. The joint
    name is not prefixed a second time.
    """
    prefix = f"{robot_id}/"
    return {
        "odom_topic": f"/{robot_id}/odom",
        "joint_topic": f"/{robot_id}/joint_states",
        "tf_topic": "/tf",
        "parent": f"{prefix}odom",
        "child": f"{prefix}base_footprint",
        "joints": [f"{prefix}{name}" for name in config.urdf_joints],
    }


@dataclass
class OdomSession:
    mode: str
    domain: int
    config: SafetyConfig
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    measured_vx: float = 0.0
    measured_wz: float = 0.0
    commanded_vx: float = 0.0
    commanded_wz: float = 0.0
    state: str = "idle"
    wheel_left: int = 0
    wheel_right: int = 0
    last_wheels: tuple[int, int] | None = None
    last_raw: tuple[int, int] | None = None
    last_command_at: float | None = None
    last_feedback_at: float | None = None
    last_reject: str = ""
    motor_port_open: bool = False
    torque_enabled: bool = False
    rpm_sent: tuple[float, float] | None = None
    physical_stop_confirmed: bool = False
    bus_watchdog_verified: bool = False
    stop_write_acked: bool = False
    init_failed: bool = False
    transitions: list[str] = field(default_factory=list)
    transport: FakeTransport | None = None

    def __post_init__(self) -> None:
        if self.transport is None:
            self.transport = FakeTransport()

    def arm(self, sources: set[str]) -> str | None:
        reason = joint_source_block(self.mode, sources)
        if reason:
            self.state = "refused"
            self.last_reject = reason
            return reason
        if self.mode == "SENSOR_ONLY":
            self.state = "sensor_only"
            return None
        if self.mode == "HARDWARE":
            self.state = "hardware_blocked"
            return None
        reason = mock_domain_block(self.domain, self.config)
        if reason:
            self.state = "refused"
            self.last_reject = reason
            return reason
        self.state = "ready"
        return None

    def fail_init(self, reason: str) -> None:
        self._drop_command()
        self.state = "FAULT_LATCHED"
        self.init_failed = True
        self.last_reject = reason
        self.motor_port_open = False
        self.torque_enabled = False
        self.rpm_sent = None
        self.physical_stop_confirmed = False
        self.transitions.append("FAULT_LATCHED")

    def push_twist(self, message: object, now: float) -> bool:
        if not isinstance(message, (tuple, list)) or len(message) != 2:
            self._fail_safe("twist is not a pair of finite velocities")
            return False
        return self.push_command(message[0], message[1], now)

    def push_command(self, vx: float, wz: float, now: float) -> bool:
        if self.motor_port_open or self.torque_enabled:
            self._fail_safe("motor port is open")
            return False
        if self.mode != "ODOM_MOCK" or self.state in BLOCKED:
            self.last_reject = "this mode does not accept cmd_vel"
            self.rpm_sent = None
            return False
        if not math.isfinite(now):
            self._fail_safe("command time is not finite")
            return False
        if self.last_command_at is not None and now < self.last_command_at:
            self._fail_safe("command time moved backwards")
            return False
        if self.last_command_at is not None and now - self.last_command_at > self.config.max_clock_step_sec:
            self._fail_safe("command time jumped")
            return False
        if isinstance(vx, bool) or isinstance(wz, bool) or not isinstance(vx, (int, float)) or not isinstance(wz, (int, float)):
            self._fail_safe("twist type is not numeric")
            return False
        if not math.isfinite(vx) or not math.isfinite(wz):
            self._fail_safe("cmd_vel is not finite")
            return False
        if abs(vx) > self.config.max_linear_m_s or abs(wz) > self.config.max_angular_rad_s:
            self._fail_safe("cmd_vel exceeds the software speed limit")
            return False
        if self.last_command_at is not None:
            dt = now - self.last_command_at
            if dt > 0:
                if abs(vx - self.commanded_vx) / dt > self.config.max_linear_accel_m_s2:
                    self._fail_safe("cmd_vel exceeds the software acceleration limit")
                    return False
                if abs(wz - self.commanded_wz) / dt > self.config.max_angular_accel_rad_s2:
                    self._fail_safe("cmd_vel exceeds the software yaw acceleration limit")
                    return False
        self.commanded_vx = float(vx)
        self.commanded_wz = float(wz)
        self.last_command_at = now
        self.rpm_sent = None
        self.last_reject = ""
        return True

    def request_recovery(self) -> bool:
        if self.init_failed or self.state == "FAULT_LATCHED":
            self.last_reject = "latched fault needs a successful stop write before recovery"
            return False
        if self.state != "RECOVERY_REQUIRED":
            return False
        self._drop_command()
        self.last_wheels = None
        self.last_raw = None
        self.last_feedback_at = None
        self.last_command_at = None
        self.measured_vx = 0.0
        self.measured_wz = 0.0
        self.state = "ready"
        self.transitions.append("ready")
        return True

    def poll(self, now: float) -> None:
        if self.state in BLOCKED or self.state in {"idle", "refused"}:
            return
        if not math.isfinite(now):
            self._fail_safe("clock is not finite")
            return
        command_late = (
            self.last_command_at is not None
            and now - self.last_command_at > self.config.command_timeout_sec
        )
        feedback_late = (
            self.last_feedback_at is not None
            and now - self.last_feedback_at > self.config.feedback_timeout_sec
        )
        jumped = (
            (self.last_command_at is not None and now - self.last_command_at > self.config.max_clock_step_sec)
            or (self.last_feedback_at is not None and now - self.last_feedback_at > self.config.max_clock_step_sec)
        )
        backwards = (
            (self.last_command_at is not None and now < self.last_command_at)
            or (self.last_feedback_at is not None and now < self.last_feedback_at)
        )
        if backwards or jumped or command_late or feedback_late:
            self._fail_safe("watchdog or clock fault")

    def push_wheels(self, left_ticks: int | None, right_ticks: int | None, now: float) -> bool:
        if self.state in BLOCKED:
            return False
        if not math.isfinite(now):
            self._fail_safe("feedback time is not finite")
            return False
        if left_ticks is None or right_ticks is None:
            self._fail_safe("one encoder sample is missing")
            return False
        try:
            left = int(left_ticks)
            right = int(right_ticks)
        except (TypeError, ValueError):
            self._fail_safe("encoder sample is not an integer")
            return False
        if self.last_feedback_at is not None and now < self.last_feedback_at:
            self._fail_safe("feedback time moved backwards")
            return False
        if self.last_wheels is None:
            self.last_wheels = (left, right)
            self.wheel_left = left
            self.wheel_right = right
            self.last_feedback_at = now
            self.state = "stopped"
            return True
        dt = now - self.last_feedback_at
        if dt <= 0 or dt > self.config.max_feedback_dt_sec:
            self._fail_safe("feedback sample interval is not usable")
            return False
        delta_left = signed_delta(self.last_wheels[0], left)
        delta_right = signed_delta(self.last_wheels[1], right)
        allowed = max_wheel_ticks(self.config, dt)
        if abs(delta_left) > allowed or abs(delta_right) > allowed:
            self._fail_safe("encoder step exceeds the speed limit for this sample interval")
            return False
        self._integrate(delta_left, delta_right, dt)
        self.last_wheels = (left, right)
        self.wheel_left = left
        self.wheel_right = right
        self.last_feedback_at = now
        return True

    def push_raw(self, left_raw: int | None, right_raw: int | None, now: float) -> bool:
        """Apply the driver sign, then the same wheel integrator."""
        if left_raw is None or right_raw is None:
            return self.push_wheels(None, None, now)
        if self.last_raw is None:
            self.last_raw = (int(left_raw), int(right_raw))
            return self.push_wheels(0, 0, now)
        delta_left = signed_delta(self.last_raw[0], int(left_raw))
        delta_right = self.config.right_raw_sign * signed_delta(self.last_raw[1], int(right_raw))
        base = self.last_wheels or (0, 0)
        accepted = self.push_wheels(base[0] + delta_left, base[1] + delta_right, now)
        if accepted:
            self.last_raw = (int(left_raw), int(right_raw))
        return accepted

    def _drop_command(self) -> None:
        self.commanded_vx = 0.0
        self.commanded_wz = 0.0
        self.rpm_sent = None

    def _fail_safe(self, reason: str) -> None:
        self._drop_command()
        self.measured_vx = 0.0
        self.measured_wz = 0.0
        self.last_reject = reason
        self.physical_stop_confirmed = False
        self.transitions.append("STOP_REQUIRED")
        self.state = "STOP_REQUIRED"
        acked = bool(self.transport and self.transport.write_stop())
        self.stop_write_acked = acked
        self.state = "RECOVERY_REQUIRED" if acked else "FAULT_LATCHED"
        self.transitions.append(self.state)

    def _integrate(self, delta_left: int, delta_right: int, dt: float) -> None:
        circumference = 2.0 * math.pi * self.config.wheel_radius_m
        dist_left = delta_left / self.config.ticks_per_rev * circumference
        dist_right = delta_right / self.config.ticks_per_rev * circumference
        distance = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.config.wheel_separation_m
        self.theta += delta_theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        self.measured_vx = distance / dt if dt > 0 else 0.0
        self.measured_wz = delta_theta / dt if dt > 0 else 0.0
        self.state = "stopped" if delta_left == 0 and delta_right == 0 else "moving"

    def joint_state(self) -> dict:
        scale = 2.0 * math.pi / self.config.ticks_per_rev
        return {
            "name": list(self.config.urdf_joints),
            "position": [self.wheel_left * scale, self.wheel_right * scale],
        }

    def tf(self) -> dict:
        return {
            "parent": "odom",
            "child": "base_footprint",
            "x": self.x,
            "y": self.y,
            "yaw": self.theta,
        }

    @property
    def publishes_messages(self) -> bool:
        return False
