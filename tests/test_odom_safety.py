"""Mock encoder odometry. Does not import rclpy and does not publish."""
from pathlib import Path

from hardware_safety.session import OdomSession, load_config, motor_launch_block

ROOT = Path(__file__).resolve().parents[1]


def _session(mode: str, domain: int = 90) -> OdomSession:
    session = OdomSession(mode, domain, load_config(ROOT))
    return session


def test_equal_wheel_ticks_drive_forward_and_opposite_ticks_turn():
    session = _session("ODOM_MOCK")
    assert session.arm({"encoder_joints"}) is None
    assert session.push_wheels(0, 0, 0.0)
    assert session.push_wheels(100, 100, 0.1)
    assert session.x > 0.0
    assert abs(session.y) < 1e-9
    assert abs(session.theta) < 1e-9
    assert session.state == "moving"
    turned = _session("ODOM_MOCK")
    assert turned.arm({"encoder_joints"}) is None
    assert turned.push_wheels(0, 0, 0.0)
    assert turned.push_wheels(100, -100, 0.1)
    assert abs(turned.x) < 1e-9
    assert abs(turned.theta) > 0.0


def test_zero_ticks_stay_stopped_and_bad_samples_do_not_move_the_pose():
    session = _session("ODOM_MOCK")
    assert session.arm({"encoder_joints"}) is None
    assert session.push_wheels(0, 0, 0.0)
    assert session.push_wheels(0, 0, 0.1)
    assert session.measured_vx == 0.0
    assert session.state == "stopped"
    held = (session.x, session.y, session.theta)
    assert session.push_wheels(None, 10, 0.2) is False
    assert session.state == "RECOVERY_REQUIRED"
    assert (session.x, session.y, session.theta) == held
    assert session.push_wheels(50, 50, 0.3) is False
    assert (session.x, session.y, session.theta) == held

    glitch = _session("ODOM_MOCK")
    assert glitch.arm({"encoder_joints"}) is None
    assert glitch.push_wheels(0, 0, 0.0)
    assert glitch.push_wheels(100_000, 0, 0.1) is False
    assert glitch.state == "RECOVERY_REQUIRED"
    assert glitch.x == 0.0


def test_encoder_wrap_is_a_small_step():
    session = _session("ODOM_MOCK")
    assert session.arm({"encoder_joints"}) is None
    start = 2**31 - 5
    assert session.push_wheels(start, start, 0.0)
    assert session.push_wheels(-2**31 + 5, -2**31 + 5, 0.1)
    assert session.state == "moving"
    assert 0.0 < session.x < 0.01


def test_timeout_enters_stop_and_abnormal_commands_are_rejected():
    session = _session("ODOM_MOCK")
    assert session.arm({"encoder_joints"}) is None
    assert session.push_command(0.1, 0.0, 1.0)
    session.poll(1.6)
    assert session.state == "RECOVERY_REQUIRED"
    assert "STOP_REQUIRED" in session.transitions
    assert session.commanded_vx == 0.0
    assert session.stop_write_acked is True
    assert session.physical_stop_confirmed is False
    assert session.bus_watchdog_verified is False
    assert session.rpm_sent is None
    fresh = _session("ODOM_MOCK")
    assert fresh.arm({"encoder_joints"}) is None
    assert fresh.push_command(0.1, 0.0, 1.0)
    assert fresh.push_command(5.0, 0.0, 1.2) is False
    assert fresh.commanded_vx == 0.0
    assert fresh.state == "RECOVERY_REQUIRED"
    assert fresh.push_command(float("nan"), 0.0, 1.3) is False
    assert fresh.push_command(0.2, 0.0, 1.05) is False
    assert fresh.commanded_vx == 0.0
    assert fresh.rpm_sent is None


def test_joint_names_match_the_urdf_and_publishers_do_not_overlap():
    session = _session("ODOM_MOCK")
    assert session.arm({"encoder_joints"}) is None
    assert session.push_wheels(0, 0, 0.0)
    assert session.joint_state()["name"] == ["l_wheel_joint", "r_wheel_joint"]
    assert session.tf()["parent"] == "odom"
    assert session.tf()["child"] == "base_footprint"
    duplicate = _session("ODOM_MOCK")
    assert duplicate.arm({"encoder_joints", "joint_state_publisher"})
    assert duplicate.state == "refused"
    sensor = _session("SENSOR_ONLY")
    assert sensor.arm({"joint_state_publisher"}) is None
    assert sensor.push_command(0.1, 0.0, 0.0) is False
    assert sensor.push_wheels(10, 10, 0.1) is False
    hardware = _session("HARDWARE", domain=53)
    assert hardware.arm(set()) is None
    assert hardware.state == "hardware_blocked"
    assert hardware.push_command(0.1, 0.0, 0.0) is False
    assert hardware.motor_port_open is False
    assert hardware.torque_enabled is False
    assert motor_launch_block("HARDWARE")
    assert motor_launch_block("SENSOR_ONLY")
    assert motor_launch_block("ODOM_MOCK")


def test_mock_odom_refuses_the_pinky_rmf_and_gazebo_domains():
    config_session = _session("ODOM_MOCK", domain=53)
    assert config_session.arm({"encoder_joints"})
    for domain in (0, 42, 24, 25):
        blocked = _session("ODOM_MOCK", domain=domain)
        assert blocked.arm({"encoder_joints"})
        assert blocked.publishes_messages is False
    allowed = _session("ODOM_MOCK", domain=90)
    assert allowed.arm({"encoder_joints"}) is None
    assert allowed.publishes_messages is False


def test_raw_right_encoder_uses_the_driver_sign_and_init_failure_holds_pose():
    session = _session("ODOM_MOCK")
    assert session.arm({"encoder_joints"}) is None
    assert session.push_raw(0, 0, 0.0)
    assert session.push_raw(100, -100, 0.1)
    assert session.x > 0.0
    assert abs(session.theta) < 1e-9
    failed = _session("ODOM_MOCK")
    assert failed.arm({"encoder_joints"}) is None
    failed.fail_init("serial port was not opened")
    assert failed.push_wheels(0, 0, 0.0) is False
    assert failed.x == 0.0
    assert failed.motor_port_open is False
    assert failed.torque_enabled is False
    assert failed.config.driver_joints == ("left_wheel_joint", "right_wheel_joint")
    assert failed.config.urdf_joints == ("l_wheel_joint", "r_wheel_joint")
    assert failed.config.bus_watchdog_register is None
