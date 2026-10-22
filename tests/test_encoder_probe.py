"""Fake Dynamixel reads and the scripted mock path. No serial port and no ROS."""
from pathlib import Path

import pytest

from hardware_safety.fake_bus import FakeDynamixelBus, run_fake_identification, simulated_pair
from hardware_safety.mock_path import replay, wheel_samples
from hardware_safety.read_only import ReadOnlyPlan
from hardware_safety.session import OdomSession, SafetyError, load_config

ROOT = Path(__file__).resolve().parents[1]


def test_scripted_path_drives_forward_and_then_turns():
    session = OdomSession("ODOM_MOCK", 90, load_config(ROOT))
    assert session.arm({"encoder_joints"}) is None
    replay(session)
    assert session.x > 0.02
    assert abs(session.theta) > 0.2
    assert session.motor_port_open is False
    assert len(wheel_samples()) == 51


def test_fake_read_does_not_open_a_device_or_write():
    bus = FakeDynamixelBus(simulated_pair(1060, 1200))
    report = run_fake_identification(bus)
    assert report["real_port_opened"] is False
    assert report["port"] == "fake://dynamixel"
    assert report["model_confirmed"] is False
    assert report["writes"] == []
    assert [item["id"] for item in report["servos"]] == [1, 2]
    assert report["servos"][0]["model_number"] == 1060
    assert report["servos"][0]["source"] == "fake_bus"
    with pytest.raises(SafetyError, match="real serial"):
        bus.open("/dev/ttyAMA4")
    with pytest.raises(SafetyError, match="torque_enable"):
        bus.write(1, 64, bytes([1]))
    assert bus.writes
    plan = ReadOnlyPlan()
    with pytest.raises(SafetyError):
        plan.execute()
    assert plan.port_opened is False


def test_enabled_torque_or_a_short_read_aborts_without_a_write():
    live = FakeDynamixelBus(simulated_pair(1060, 1060, torque_a=1))
    with pytest.raises(SafetyError, match="torque"):
        run_fake_identification(live)
    assert live.writes == []
    broken = simulated_pair(1060, 1060)
    broken[2][0] = bytes([1])
    short = FakeDynamixelBus(broken)
    with pytest.raises(SafetyError, match="unexpected"):
        run_fake_identification(short)
    assert short.writes == []


def test_rviz_config_points_at_the_mock_frames_only():
    text = (ROOT / "sim" / "hardware" / "mock_odom.rviz").read_text(encoding="utf-8")
    assert "Fixed Frame: pinky1/odom" in text
    assert "/pinky1/odom" in text
    assert "ttyAMA" not in text
    assert "domain" not in text.lower()
    assert "/scan" not in text
