"""Simulation fleet session. Does not start ROS or publish a goal."""
from pathlib import Path

import pytest

from fleet_adapter.config import load_fleet_spec
from fleet_adapter.sim_gateway import (
    GatewayRefused,
    SimGateway,
    TaskRecord,
    readonly_argv_allowed,
    serve_line,
    write_json_line,
)
from fleet_adapter.live_nav import ABORTED, CANCELED, SUCCEEDED, GoalTracker
from fleet_adapter.sim_graph import dispatch_block, load_sim_session
from fleet_adapter.tf_frames import frame_collisions, frame_ids
from robot_profile.calibration import CHOSEN_CORRECTION, calibration_status
from robot_profile.profiles import load_robot
from robot_profile.safety import hardware_goal_block_reason

ROOT = Path(__file__).resolve().parents[1]
READY = dict(
    rmf_domain=77,
    nav_domain=42,
    backend="gazebo",
    robot_id="pinky1",
    fleet_name="gazebo_pinky1",
    action_ready=True,
    pose_fresh=True,
    clock_ok=True,
    hardware_devices=False,
)


def test_operational_fleet_stays_locked_and_sim_fleet_is_separate():
    operational = load_fleet_spec(ROOT / "config" / "rmf" / "care_move_fleet.yaml", ROOT)
    assert operational.fleet_name == "care_move"
    assert operational.allow_motion is False
    assert operational.simulation_confirmed is False
    assert operational.graph_alignment == "placeholder"
    session = load_sim_session(ROOT)
    assert session.fleet_name == "gazebo_pinky1"
    assert session.allow_motion is False
    assert session.rmf_domain_id == 77
    assert session.nav_domain_id == 42
    assert dispatch_block(session, **READY) is None
    assert dispatch_block(session, **{**READY, "action_ready": False})


def test_hardware_backend_stays_blocked_when_the_gazebo_session_is_open():
    session = load_sim_session(ROOT)
    hardware = load_robot(ROOT, "pinky1", "hardware")
    assert dispatch_block(session, **{**READY, "backend": "hardware"})
    assert hardware_goal_block_reason(
        hardware,
        motion_approved=True,
        estop_confirmed=True,
        scan_ok=True,
        odom_ok=True,
        localization_ok=True,
    )
    gateway = SimGateway()
    with pytest.raises(GatewayRefused):
        gateway.refuse_hardware()
    assert gateway.hardware_commands == 1


def test_namespaces_and_tf_frames_do_not_collide_for_gazebo():
    gazebo = [
        (robot_id, "gazebo", 42) for robot_id in ("pinky1", "pinky2", "pinky3")
    ]
    assert frame_collisions(gazebo) == []
    assert frame_ids("pinky1", "gazebo")[0] == "pinky1/odom"
    hardware_same_domain = [
        ("pinky1", "hardware", 77),
        ("pinky2", "hardware", 77),
    ]
    assert frame_collisions(hardware_same_domain)
    split = [("pinky1", "hardware", 81), ("pinky2", "hardware", 82)]
    assert frame_collisions(split) == []
    assert frame_ids("pinky1", "hardware") == (
        "odom", "base_footprint", "base_link", "rplidar_link",
    )


def test_graph_goal_cancel_failure_and_bad_waypoint():
    session = load_sim_session(ROOT)
    gateway = SimGateway()
    gateway.connect(dispatch_block(session, **READY))
    gateway.register("pinky1")
    gateway.publish_pose(0.0, 0.0, 0.0, 10.0)
    task = TaskRecord()
    task.dispatch(session, gateway, "GZ_EAST", "task-a", 10.5, **READY)
    assert task.rmf_status == "underway"
    assert gateway.goal is not None
    assert (gateway.goal.x, gateway.goal.y) == (1.2, 0.0)
    gateway.succeed()
    task.observe(gateway)
    assert task.rmf_status == "completed"

    cancel = TaskRecord()
    gateway2 = SimGateway()
    gateway2.connect(None)
    gateway2.publish_pose(1.2, 0.0, 0.0, 20.0)
    cancel.dispatch(session, gateway2, "GZ_SPAWN", "task-b", 20.5, **READY)
    gateway2.cancel()
    cancel.observe(gateway2)
    assert cancel.rmf_status == "canceled"

    lost = TaskRecord()
    gateway3 = SimGateway()
    gateway3.connect(None)
    gateway3.publish_pose(0.0, 0.0, 0.0, 30.0)
    lost.dispatch(session, gateway3, "GZ_EAST", "task-c", 30.5, **READY)
    gateway3.drop()
    lost.observe(gateway3)
    assert lost.rmf_status == "failed"
    again = TaskRecord()
    again.dispatch(session, gateway3, "GZ_EAST", "task-d", 31.0, **READY)
    assert again.rmf_status == "rejected"
    assert again.goal_sent is False

    bad = TaskRecord()
    fresh = SimGateway()
    fresh.connect(None)
    fresh.publish_pose(0.0, 0.0, 0.0, 40.0)
    bad.dispatch(session, fresh, "N_ROOM102", "task-e", 40.5, **READY)
    assert bad.rmf_status == "rejected"
    assert fresh.goal is None
    assert calibration_status()["wheel_radius_m"]["chosen"] is CHOSEN_CORRECTION


def test_readonly_inspection_cannot_publish_or_launch():
    assert readonly_argv_allowed(["ros2", "node", "list"])
    assert readonly_argv_allowed(["ros2", "topic", "list", "-t"])
    assert readonly_argv_allowed(["ros2", "action", "list", "-t"])
    assert readonly_argv_allowed(["ros2", "topic", "echo", "--once", "/pinky1/odom"])
    assert not readonly_argv_allowed(["ros2", "topic", "pub", "/pinky1/cmd_vel", "geometry_msgs/msg/Twist"])
    assert not readonly_argv_allowed(["ros2", "action", "send_goal", "/pinky1/navigate_to_pose"])
    assert not readonly_argv_allowed(["ros2", "launch", "pinky_bringup", "bringup_robot.launch.xml"])
    assert not readonly_argv_allowed(["ros2", "topic", "echo", "/cmd_vel"])


def test_gateway_socket_refuses_a_hardware_hello():
    from fleet_adapter.sim_graph import GraphError

    server, port = serve_line(None)

    def handler():
        conn, _addr = server.accept()
        with conn:
            raw = b""
            while b"\n" not in raw:
                raw += conn.recv(1024)
            message = __import__("json").loads(raw)
            if message.get("backend") != "gazebo":
                write_json_line(conn, {"op": "refuse", "reason": "backend"})
            else:
                write_json_line(conn, {"op": "ready"})

    import threading
    thread = threading.Thread(target=handler)
    thread.start()
    import socket
    client = socket.create_connection(("127.0.0.1", port), timeout=2)
    client.sendall(b'{"op":"hello","backend":"hardware"}\n')
    reply = client.recv(256)
    client.close()
    thread.join(timeout=2)
    server.close()
    assert b"refuse" in reply
    with pytest.raises(GraphError):
        load_sim_session(ROOT).nav_pose("floor_7")


def test_live_goal_tracker_ignores_a_stale_result():
    tracker = GoalTracker()
    first = tracker.begin()
    assert tracker.accept(first, "goal-a")
    assert tracker.request_cancel(first)
    assert tracker.observe_status(first, CANCELED) == "canceled"
    second = tracker.begin()
    assert tracker.accept(second, "goal-b")
    assert tracker.observe_status(first, SUCCEEDED) == "stale"
    assert tracker.phase == "accepted"
    assert tracker.observe_status(second, ABORTED) == "aborted"
    assert tracker.begin() is not None
