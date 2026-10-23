from fleet_adapter.physical_dry_run import DRY_RUN, DryRunGateway, assess, classify_joints, decide_registration


def test_joint_publisher_zeros_are_not_encoder_values():
    assert classify_joints(
        ["l_wheel_joint", "r_wheel_joint", "caster_rotate_joint", "caster_wheel_joint"],
        [0.0, 0.0, 0.0, 0.0],
    ) == "joint_state_publisher_zeros"
    assert classify_joints(["left_wheel_joint", "right_wheel_joint"], [0.2, -0.2]) == "bringup_joint_names"


def test_current_pinky_observation_is_not_registered_and_has_no_pose():
    decision = assess({
        "mode": DRY_RUN,
        "domain": 24,
        "rmw": "rmw_fastrtps_cpp",
        "scan_fresh": True,
        "scan_frame": "rplidar_link",
        "joint_names": ["l_wheel_joint", "r_wheel_joint"],
        "joint_positions": [0.0, 0.0],
        "odom_topic": False,
        "nav2_action": False,
        "map_topic": False,
        "localization_valid": False,
        "cmd_vel_publishers": 0,
    })
    assert decision["register_with_rmf"] is False
    assert decision["motion_allowed"] is False
    assert decision["pose"] is None
    assert decision["robot_id"] == "real_pinky1"
    assert decision["fleet_name"] == "physical_pinky"
    assert "gazebo" not in decision["fleet_name"]


def test_go_and_repeat_do_not_increment_motion_commands():
    gateway = DryRunGateway({"domain": 24, "rmw": "rmw_fastrtps_cpp", "mode": DRY_RUN})
    first = gateway.handle({"op": "go", "correlation_id": "same-request"})
    second = gateway.handle({"op": "go", "correlation_id": "same-request"})
    torque = gateway.handle({"op": "torque"})
    assert first["refused"] is True
    assert second["duplicate"] is True
    assert torque["motion_commands"] == 0
    assert gateway.motion_commands == 0
    assert gateway.refused == 3


def test_fleet_adapter_decision_does_not_register():
    report = {
        "motion_commands": 0,
        "observation": {"scan_fresh": True, "scan_frame": "rplidar_link", "joint_source": "joint_state_publisher_zeros",
                        "odom_topic": False, "nav2_action": False, "map_topic": False, "cmd_vel_publishers": 0},
        "decision": {"reasons": ["navigate_to_pose action server is absent"], "register_with_rmf": False},
    }
    decision = decide_registration(report)
    assert decision["registered"] is False
    assert decision["register_with_rmf"] is False
    assert decision["motion_commands"] == 0


def test_measured_segment_is_five_centimetres_along_heading():
    from fleet_adapter.physical_motion import MoveController, TrialSession, measured_segment
    segment = measured_segment(1.0, 2.0, 0.0)
    assert abs(segment["P_TEST"][0] - 1.05) < 1e-9
    assert abs(segment["P_TEST"][1] - 2.0) < 1e-9
    assert measured_segment(0, 0, 0, distance=0.04) is None
    pose = {"x": 1.0, "y": 2.0, "yaw": 0.0, "speed": 0.0, "fresh": True}
    move = MoveController()
    accepted = move.go(pose, segment["P_TEST"], "once")
    assert accepted["ok"] is True
    assert move.go(pose, segment["P_TEST"], "once")["duplicate"] is True
    short = MoveController()
    assert short.go({"x": 0, "y": 0, "yaw": 0, "speed": 0, "fresh": True}, [0.04, 0, 0], "short")["ok"] is False
    far = MoveController()
    assert far.go(pose, [1.0 + 1.0, 2.0, 0.0], "far")["ok"] is False
    moving = move.tick({**pose, "x": 1.02, "speed": 0.04, "fresh": True})
    assert moving[0] > 0
    assert moving[0] <= 0.04
    assert move.phase == "moving"
    move.tick({**pose, "x": 1.046, "speed": 0.04, "fresh": True})
    assert move.phase == "stopping"
    move.tick({**pose, "x": 1.046, "speed": 0.0, "fresh": True})
    assert move.phase == "stopping"
    move.tick({**pose, "x": 1.046, "speed": 0.0, "fresh": True})
    move.tick({**pose, "x": 1.046, "speed": 0.0, "fresh": True})
    assert move.phase == "succeeded"
    assert move.traveled >= 0.04
    stale = MoveController()
    stale.go(pose, segment["P_TEST"], "stale")
    assert stale.tick({"fresh": False, "yaw": 0.0}) == (0.0, 0.0)
    assert stale.phase == "failed"
    session = TrialSession()
    opened = session.prepare(0.2, 0.0, 0.0, "idle", True, 0.0)
    assert opened["ok"] is True
    assert abs(opened["segment"]["P_TEST"][0] - 0.25) < 1e-9
    assert session.prepare(0.2, 0.0, 0.0, "idle", True, 0.0)["ok"] is False


def test_physical_robot_is_rejected_until_odom_waypoints_exist(monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    monkeypatch.setattr(main, "MODE", "rmf")
    monkeypatch.setattr(main, "read_physical_gateway", lambda: {
        "reachable": True, "mode": "PHYSICAL_DRY_RUN", "waypoints": [], "pose": None, "pose_source": None})
    response = TestClient(main.app).post(
        "/api/tasks",
        headers={"Authorization": "Bearer operator-token"},
        json={"robot_id": "real_pinky1", "places": ["P_TEST"]},
    )
    assert response.status_code == 409
    main.MODE = "mock"


def test_five_centimetre_lane_keeps_a_real_footprint():
    from fleet_adapter.physical_motion import FOOTPRINT_M, button_block, lane_is_navigable, may_finish, measured_segment, trial_graph
    assert lane_is_navigable(0.05) is None
    assert lane_is_navigable(0.005) is not None
    assert FOOTPRINT_M >= 0.15
    segment = measured_segment(0.05, 0.0, 0.0)
    fleet, nav = trial_graph(segment)
    assert "physical_pinky" in fleet and "real_pinky1" in fleet
    assert "footprint: 0.005" not in fleet
    assert f"footprint: {FOOTPRINT_M}" in fleet
    assert "GZ_" not in fleet and "GZ_" not in nav
    goal = {"phase": "succeeded", "target": segment["P_TEST"], "stopped_samples": 3, "traveled": 0.046, "lateral": 0.0}
    assert may_finish(goal, [0.05, 0.0, 0.0], True, 0.0) is False
    assert may_finish(goal, segment["P_TEST"], True, 0.0) is True
    assert may_finish({**goal, "stopped_samples": 1}, segment["P_TEST"], True, 0.0) is False
    ready_dry = button_block({"reachable": True, "mode": "PHYSICAL_DRY_RUN", "motor_ready": True, "odom_fresh": True, "pose": [0, 0, 0], "odom_velocity": 0, "goal_phase": "idle"})
    assert ready_dry is None
    ready = button_block({"reachable": True, "mode": "PHYSICAL_TEST", "motor_ready": True, "odom_fresh": True, "pose": [0, 0, 0], "odom_velocity": 0, "goal_phase": "idle"})
    assert ready is None
    assert button_block({"reachable": True, "mode": "FAULT", "motor_ready": True, "odom_fresh": True, "pose": [0, 0, 0], "odom_velocity": 0, "goal_phase": "idle"})


def test_connected_pinkies_share_one_drive_domain():
    from fleet_adapter.physical_motion import DRIVE_DOMAIN_ID, RobotSelection, button_block, describe_robots
    robots = describe_robots(
        {"/odom": 1, "/pinky1/odom": 1, "/pinky2/odom": 1},
        {"/cmd_vel": 1, "/pinky2/cmd_vel": 1},
        {},
    )
    assert DRIVE_DOMAIN_ID == "24"
    assert [item["id"] for item in robots] == ["real_pinky1", "real_pinky2", "real_pinky3"]
    assert robots[0]["odom_topic"] == "/odom" and robots[0]["cmd_vel_topic"] == "/cmd_vel"
    assert robots[0]["available"] and robots[1]["available"] and not robots[2]["available"]
    namespaced = describe_robots({"/pinky1/odom": 1}, {"/pinky1/cmd_vel": 1}, {})
    assert namespaced[0]["odom_topic"] == "/pinky1/odom"
    choice = RobotSelection()
    assert choice.choose("real_pinky3", robots, "idle", False, False)["ok"] is False
    assert choice.choose("real_pinky2", robots, "moving", False, False)["ok"] is False
    assert choice.choose("real_pinky2", robots, "idle", False, False)["robot_id"] == "real_pinky2"
    blocked = button_block({"reachable": True, "mode": "PHYSICAL_TEST", "domain": 42, "motor_ready": True,
                             "odom_fresh": True, "pose": [0, 0, 0], "odom_velocity": 0, "goal_phase": "idle"})
    assert blocked


def test_half_turn_stays_in_place_until_180_degrees():
    import math
    from fleet_adapter.physical_motion import (
        FOOTPRINT_M, MoveController, TrialSession, may_finish, trial_graph,
    )
    session = TrialSession()
    opened = session.prepare(0.2, 0.1, 0.0, "idle", True, 0.0, kind="rotate_180", robot_id="real_pinky2")
    assert opened["ok"] is True
    fleet, nav = trial_graph(opened["segment"])
    assert "real_pinky2" in fleet and "real_pinky1" not in fleet
    assert "rotate_180" in fleet and "P_TEST" not in nav
    assert f"footprint: {FOOTPRINT_M}" in fleet
    move = MoveController()
    pose = {"x": 0.2, "y": 0.1, "yaw": 0.0, "fresh": True, "speed": 0.0}
    assert move.go_rotate(pose, math.pi / 2, "quarter")["ok"] is False
    assert move.go_rotate(pose, opened["segment"]["P_TURN"][2], "spin")["ok"] is True
    yaw = 0.0
    for _ in range(40):
        yaw += 0.1
        linear, angular = move.tick({"x": 0.2, "y": 0.1, "yaw": yaw, "speed": 0.0, "angular": 0.2, "fresh": True})
        assert linear == 0.0 and abs(angular) <= 0.3
        if move.phase == "stopping":
            break
    assert move.phase == "stopping"
    for _ in range(3):
        move.tick({"x": 0.2, "y": 0.1, "yaw": yaw, "speed": 0.0, "angular": 0.0, "fresh": True})
    assert move.phase == "succeeded"
    assert may_finish(move.snapshot(), [0.2, 0.1, yaw], True, 0.0, 0.0) is True
    drift = MoveController()
    drift.go_rotate({"x": 0.0, "y": 0.0, "yaw": 0.0, "fresh": True}, math.pi, "drift")
    drift.tick({"x": 0.04, "y": 0.0, "yaw": 0.2, "speed": 0.0, "angular": 0.1, "fresh": True})
    assert drift.fail_reason == "rotation drifted more than 0.03 m"


def test_task_submit_timeout_stays_separate_from_status_timeout():
    from app import main
    from app.rmf_client import RmfClient
    assert main.RMF_TASK_SUBMIT_TIMEOUT == 12.0 or main.RMF_TASK_SUBMIT_TIMEOUT > main.RMF_API_TIMEOUT
    client = RmfClient("http://rmf.test", timeout=5, submit_timeout=12)
    assert client.timeout == 5
    assert client.submit_timeout == 12
