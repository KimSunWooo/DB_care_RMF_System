"""Real Pinky gate. Uses an in-memory transport and does not open a serial port."""
from pathlib import Path

from fleet_adapter.real_gate import real_task_block
from hardware_safety.real_driver import (
    MODEL_XL330_M288,
    RealPinkyDriver,
    RealTransport,
    ServoSample,
)
from hardware_safety.session import load_config

ROOT = Path(__file__).resolve().parents[1]


def _sample(servo_id: int, position: int, **overrides) -> ServoSample:
    values = dict(
        servo_id=servo_id,
        model=MODEL_XL330_M288,
        firmware=52,
        torque=0,
        hardware_error=0,
        bus_watchdog=0,
        position=position,
        velocity=1,
    )
    values.update(overrides)
    return ServoSample(**values)


def _pair(left: int = 2950, right: int = 2373, **overrides) -> list[ServoSample]:
    return [_sample(1, left, **overrides), _sample(2, right, **overrides)]


def _driver(samples: list[ServoSample], **transport) -> RealPinkyDriver:
    return RealPinkyDriver(RealTransport(samples, **transport), load_config(ROOT))


def test_observed_xl330_identity_stays_disarmed_and_stopped():
    driver = _driver(_pair())
    assert driver.identify() is None
    assert driver.state == "identified"
    assert driver.torque_enabled is False
    assert driver.transport.torque_writes == []
    assert driver.transport.velocity_writes == []
    assert driver.transport.watchdog_writes == []
    assert driver.bus_watchdog_is_command_watchdog is False
    assert driver.push_feedback(_pair(), 0.05)
    assert driver.odom.state == "stopped"
    assert driver.odom.x == 0.0
    assert driver.odom.measured_vx == 0.0
    assert driver.arm(set()) == "operator has not confirmed an independent stop"
    assert driver.enable_torque()
    assert driver.transport.torque_writes == []


def test_joint_names_match_urdf_and_skip_the_existing_publisher():
    driver = _driver(_pair())
    driver.operator_stop_approved = True
    assert driver.identify() is None
    assert driver.arm({"joint_state_publisher"}) is None
    assert driver.publishes_wheel_joints is False
    assert driver.joint_state() is None
    alone = _driver(_pair())
    alone.operator_stop_approved = True
    assert alone.identify() is None
    assert alone.arm(set()) is None
    state = alone.joint_state()
    assert state["name"] == ["l_wheel_joint", "r_wheel_joint"]
    assert state["position"][0] > 0
    assert alone.tf()["parent"] == "odom"
    assert alone.tf()["child"] == "base_footprint"


def test_right_encoder_sign_and_wrap_follow_the_existing_integrator():
    driver = _driver(_pair(0, 0))
    assert driver.identify() is None
    assert driver.push_feedback(_pair(0, 100), 0.05)
    assert driver.odom.x != 0.0 or driver.odom.theta != 0.0
    wrapped = _driver(_pair(2**32 - 5, 0))
    assert wrapped.identify() is None
    assert wrapped.push_feedback(_pair(5, 0), 0.05)
    assert wrapped.odom.state == "moving"
    assert abs(wrapped.odom.x) < 0.01


def test_bad_model_torque_and_encoder_step_do_not_enable_motion():
    wrong = _driver(_pair(model=1190))
    assert wrong.identify()
    assert wrong.state == "fault_latched"
    assert wrong.transport.velocity_writes == []
    live_torque = _driver(_pair(torque=1))
    assert "torque" in live_torque.identify()
    assert live_torque.transport.torque_writes == []
    jump = _driver(_pair())
    assert jump.identify() is None
    assert jump.push_feedback(_pair(left=2950 + 100000), 0.05) is False
    assert jump.transport.stop_writes
    assert jump.push_command(0.03, 0.0, 0.2) is False


def test_ros_watchdog_stops_and_latches_when_the_stop_write_fails():
    samples = _pair()
    driver = _driver(samples, motion_enabled=True)
    driver.operator_stop_approved = True
    assert driver.identify() is None
    assert driver.arm(set()) is None
    assert driver.enable_torque() is None
    assert driver.start_trial(0.0, 0.05, 5.0) is None
    assert driver.push_command(0.03, 0.0, 0.0)
    assert driver.transport.velocity_writes[0][0] > 0
    assert driver.transport.velocity_writes[0][1] < 0
    driver.poll(0.5)
    assert driver.reason == "ros command watchdog"
    assert driver.transport.stop_writes == [(0.0, 0.0)]
    assert driver.push_command(0.03, 0.0, 0.6) is False
    assert driver.recover()
    assert driver.state == "identified"
    assert driver.start_trial(1.0, 0.05, 5.0) is not None

    failed = _driver(samples, motion_enabled=True, stop_ok=False)
    failed.operator_stop_approved = True
    assert failed.identify() is None
    assert failed.arm(set()) is None
    assert failed.enable_torque() is None
    assert failed.start_trial(0.0, 0.05, 5.0) is None
    assert failed.push_command(0.03, 0.0, 0.0)
    failed.poll(0.5)
    assert failed.state == "fault_latched"
    assert failed.recover() is False
    assert failed.push_command(0.03, 0.0, 1.0) is False


def test_feedback_loss_does_not_restart_the_trial():
    driver = _driver(_pair(), motion_enabled=True)
    driver.operator_stop_approved = True
    assert driver.identify() is None
    assert driver.arm(set()) is None
    assert driver.enable_torque() is None
    assert driver.start_trial(0.0, 0.05, 5.0) is None
    driver.transport.fail_read = True
    assert driver.poll_bus(0.1) is False
    assert driver.transport.stop_writes
    assert driver.state == "stopped"
    assert driver.start_trial(0.2, 0.05, 5.0) is not None


def test_speed_cap_and_unapproved_trial_send_nothing():
    driver = _driver(_pair(), motion_enabled=True)
    driver.operator_stop_approved = True
    assert driver.identify() is None
    assert driver.arm(set()) is None
    assert driver.enable_torque() is None
    assert "cap" in driver.start_trial(0.0, 1.0, 5.0)
    assert driver.start_trial(0.0, 0.05, 5.0) is None
    assert driver.push_command(0.2, 0.0, 0.0) is False
    assert driver.transport.velocity_writes == []


def test_real_fleet_gate_stays_shut_for_gazebo_and_existing_domains():
    blocked = dict(
        robot_id="real_pinky1",
        fleet_name="real_pinky1",
        rmf_domain=91,
        nav_domain=53,
        commands_enabled=False,
        operator_stop_approved=False,
        safety_ready=False,
        nav2_ready=False,
        graph_aligned=False,
        goal_is_short_trial=False,
    )
    assert real_task_block(**blocked)
    assert real_task_block(**{**blocked, "robot_id": "pinky1", "fleet_name": "gazebo_pinky1"})
    assert "77" in real_task_block(**{**blocked, "rmf_domain": 77, "commands_enabled": True})
    ready = dict(blocked, commands_enabled=True, operator_stop_approved=True, safety_ready=True,
                 nav2_ready=True, graph_aligned=True, goal_is_short_trial=True)
    assert real_task_block(**ready) is None
    assert real_task_block(**{**ready, "nav_domain": 42})
