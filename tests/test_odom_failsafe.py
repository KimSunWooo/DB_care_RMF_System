"""Fail-safe transitions and the domain-90 message plan. No serial port."""
from pathlib import Path

import pytest

from hardware_safety.read_only import ReadOnlyPlan
from hardware_safety.session import (
    FakeTransport,
    OdomSession,
    SafetyError,
    framed_outputs,
    load_config,
    max_wheel_ticks,
    mock_domain_block,
)

ROOT = Path(__file__).resolve().parents[1]


def _session(transport: FakeTransport | None = None) -> OdomSession:
    session = OdomSession("ODOM_MOCK", 90, load_config(ROOT), transport=transport)
    assert session.arm({"encoder_joints"}) is None
    return session


def test_nominal_period_rejects_a_2048_tick_jump():
    session = _session()
    period = session.config.nominal_period_sec
    allowed = max_wheel_ticks(session.config, period)
    assert allowed < 2048
    assert session.push_wheels(0, 0, 0.0)
    assert session.push_wheels(100, 100, period)
    assert session.x > 0.0
    spike = _session()
    assert spike.push_wheels(0, 0, 0.0)
    assert spike.push_wheels(2048, 0, period) is False
    assert spike.x == 0.0
    assert spike.state == "RECOVERY_REQUIRED"
    assert spike.commanded_vx == 0.0


def test_invalid_twist_and_time_jump_drop_the_old_speed():
    session = _session()
    assert session.push_command(0.1, 0.0, 1.0)
    assert session.push_twist("not-a-twist", 1.1) is False
    assert session.commanded_vx == 0.0
    assert session.state == "RECOVERY_REQUIRED"
    assert session.request_recovery()
    assert session.commanded_vx == 0.0
    assert session.push_command(0.05, 0.0, 2.0)
    assert session.push_command(0.05, 0.0, 1.0) is False
    assert session.commanded_vx == 0.0
    assert "STOP_REQUIRED" in session.transitions


def test_feedback_gap_and_reconnect_do_not_replay_motion():
    session = _session()
    assert session.push_command(0.1, 0.0, 0.0)
    assert session.push_wheels(0, 0, 0.0)
    assert session.push_wheels(40, 40, 0.05)
    held = (session.x, session.y, session.theta)
    session.poll(0.7)
    assert session.state == "RECOVERY_REQUIRED"
    assert session.measured_vx == 0.0
    assert (session.x, session.y, session.theta) == held
    assert session.physical_stop_confirmed is False
    assert session.request_recovery()
    assert session.commanded_vx == 0.0
    assert session.push_wheels(0, 0, 1.0)
    assert session.x == held[0]


def test_stop_write_failure_latches_until_a_new_command_is_not_replayed():
    session = _session(FakeTransport(accept_stop=False))
    assert session.push_command(0.1, 0.0, 1.0)
    assert session.push_command(float("inf"), 0.0, 1.1) is False
    assert session.state == "FAULT_LATCHED"
    assert session.stop_write_acked is False
    assert session.commanded_vx == 0.0
    assert session.physical_stop_confirmed is False
    assert session.motor_port_open is False
    assert session.transport.port_opened is False
    assert session.request_recovery() is False
    assert session.push_command(0.1, 0.0, 2.0) is False
    assert session.commanded_vx == 0.0


def test_frames_are_prefixed_once_and_other_domains_fail_closed():
    config = load_config(ROOT)
    frames = framed_outputs(config, "pinky1")
    assert frames["odom_topic"] == "/pinky1/odom"
    assert frames["joint_topic"] == "/pinky1/joint_states"
    assert frames["parent"] == "pinky1/odom"
    assert frames["child"] == "pinky1/base_footprint"
    assert frames["joints"] == ["pinky1/l_wheel_joint", "pinky1/r_wheel_joint"]
    for domain in (None, 0, 42, 53):
        assert mock_domain_block(domain, config)
    plan = ReadOnlyPlan()
    assert plan.port_opened is False
    assert plan.writes == []
    assert plan.model_name is None
    assert plan.bus_watchdog_supported is None
    with pytest.raises(SafetyError):
        plan.execute()
