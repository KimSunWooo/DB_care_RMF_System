"""Gazebo and hardware profile contract."""
from pathlib import Path

import pytest
import yaml

from fleet_adapter.nav2_link import Nav2Link
from robot_profile.goals import GoalRejected, fleet_goal_command
from robot_profile.maps import MapError, load_map, require_navigation_map
from robot_profile.modes import ModeError, mapping_output_path, master_replace_allowed, update_block_reason
from robot_profile.preflight import hardware_launch_block, static_checks
from robot_profile.profiles import ProfileError, load_robot
from robot_profile.safety import goal_block_reason, observe_robot_state

ROOT = Path(__file__).resolve().parents[1]
OPEN_FLEET = dict(
    graph_alignment="surveyed",
    allow_motion=True,
    simulation_confirmed=True,
    action_available=True,
    pose_fresh=True,
)


def test_backend_profiles_stay_separate():
    gazebo = load_robot(ROOT, "pinky1", "gazebo")
    hardware = load_robot(ROOT, "pinky1", "hardware")
    assert gazebo.backend.ros_domain_id == 42
    assert gazebo.backend.use_sim_time is True
    assert hardware.backend.ros_domain_id is None
    assert hardware.backend.use_sim_time is False
    assert hardware.backend.hardware_motion_approved is False
    assert gazebo.backend.map_id == "gazebo_test_room"
    assert hardware.backend.map_id is None


def test_namespace_topics_for_each_robot():
    for robot_id in ("pinky1", "pinky2", "pinky3"):
        profile = load_robot(ROOT, robot_id, "hardware")
        assert profile.namespace == robot_id
        assert profile.cmd_vel == f"/{robot_id}/cmd_vel"
        assert profile.odom == f"/{robot_id}/odom"
        assert profile.scan == f"/{robot_id}/scan"
        assert profile.navigate_to_pose == f"/{robot_id}/navigate_to_pose"
    assert load_robot(ROOT, "pinky2", "gazebo").enabled is False
    assert load_robot(ROOT, "pinky3", "gazebo").enabled is False


def test_topic_contract_matches_verified_gazebo_types():
    profile = load_robot(ROOT, "pinky1", "gazebo")
    assert profile.backend.cmd_vel_type == "geometry_msgs/msg/Twist"
    assert profile.backend.odom_frame == "pinky1/odom"
    assert profile.backend.base_frame == "pinky1/base_footprint"
    assert profile.backend.laser_frame == "pinky1/rplidar_link"
    hardware = load_robot(ROOT, "pinky1", "hardware")
    assert hardware.backend.cmd_vel_type == "geometry_msgs/msg/Twist"
    assert hardware.backend.odom_frame == "odom"
    assert hardware.backend.base_frame == "base_footprint"
    assert hardware.backend.laser_frame == "rplidar_link"
    assert hardware.backend.lidar_serial_port == "/dev/ttyAMA0"
    assert hardware.backend.motor_serial_port == "/dev/ttyAMA4"


def test_gazebo_map_loads_and_missing_floors_do_not():
    room = require_navigation_map(ROOT, "gazebo_test_room")
    assert room.resolution == 0.05
    assert room.origin == (-4.0, -4.0, 0.0)
    assert room.image_path.is_file()
    for floor in ("floor_1", "floor_7"):
        record = load_map(ROOT, floor)
        assert record.verification == "missing"
        with pytest.raises(MapError, match="do not invent"):
            require_navigation_map(ROOT, floor)


def test_unverified_and_broken_maps_are_rejected(tmp_path: Path):
    lab = load_map(ROOT, "pinky_lab")
    assert lab.image_path.is_file()
    assert lab.usable_for_navigation is False
    with pytest.raises(MapError, match="not verified"):
        require_navigation_map(ROOT, "pinky_lab")
    broken = tmp_path / "broken.yaml"
    broken.write_text("image: missing.pgm\nresolution: 0.05\n", encoding="utf-8")
    from robot_profile.maps import _parse_map_yaml
    with pytest.raises(MapError, match="origin"):
        _parse_map_yaml(broken)


def test_modes_do_not_replace_the_master(tmp_path: Path):
    assert master_replace_allowed("navigation") is False
    assert master_replace_allowed("mapping") is False
    assert master_replace_allowed("map_update") is False
    master = tmp_path / "map.yaml"
    master.write_text("x", encoding="utf-8")
    with pytest.raises(ModeError, match="master"):
        mapping_output_path(ROOT, "map.yaml", [master])
    staged = mapping_output_path(ROOT, "candidate.yaml", [master])
    assert staged.parent == ROOT / "maps" / "staging"
    current = load_map(ROOT, "gazebo_test_room")
    candidate = load_map(ROOT, "pinky_my_map")
    assert update_block_reason(current, candidate)
    assert update_block_reason(load_map(ROOT, "floor_1"), current)


def test_nav2_action_and_fleet_goal_translation():
    profile = load_robot(ROOT, "pinky1", "gazebo")
    link = Nav2Link(profile.robot_id, profile.namespace)
    with pytest.raises(GoalRejected, match="unsurveyed"):
        fleet_goal_command(profile, link, "N_EV7", "placeholder")
    command = fleet_goal_command(profile, link, "N_EV7", "surveyed")
    assert command["action"] == "/pinky1/navigate_to_pose"
    assert "x" not in command


def test_hardware_motion_stays_blocked_and_disconnect_is_visible():
    hardware = load_robot(ROOT, "pinky1", "hardware")
    gazebo = load_robot(ROOT, "pinky1", "gazebo")
    assert goal_block_reason(gazebo, graph_alignment="placeholder", allow_motion=False,
                             simulation_confirmed=False, action_available=True, pose_fresh=True)
    assert goal_block_reason(hardware, **OPEN_FLEET)
    assert hardware_launch_block(hardware)
    link = Nav2Link("pinky1", "pinky1")
    link.set_action_available(True)
    link.request_goal(1.0, 0.0, 0.0)
    link.mark_accepted()
    link.mark_disconnected()
    assert link.action_available is False
    assert link.goal.phase == "failed"
    assert observe_robot_state(
        connected=False, navigating=False, arrived=False, navigation_failed=False,
        localization_ok=True, estop=False, battery_low=False,
    ) == "disconnected"
    assert observe_robot_state(
        connected=True, navigating=False, arrived=False, navigation_failed=False,
        localization_ok=False, estop=False, battery_low=False,
    ) == "localization_lost"


def test_static_preflight_reports_hardware_gaps():
    gazebo = static_checks(ROOT, load_robot(ROOT, "pinky1", "gazebo"), "navigation")
    assert all(item.ok for item in gazebo)
    hardware = static_checks(ROOT, load_robot(ROOT, "pinky1", "hardware"), "navigation")
    by_name = {item.name: item for item in hardware}
    assert by_name["ros_domain_id"].ok is False
    assert by_name["navigation_map"].ok is False
    assert by_name["hardware_motion"].ok is True
    mapping = static_checks(ROOT, load_robot(ROOT, "pinky1", "hardware"), "mapping")
    mapping_by_name = {item.name: item for item in mapping}
    assert mapping_by_name["mapping_launch"].ok is True
    assert mapping_by_name["master_overwrite_blocked"].ok is True


def test_profile_file_cannot_approve_hardware_motion(tmp_path: Path, monkeypatch):
    source = yaml.safe_load((ROOT / "config" / "robots" / "pinky_profiles.yaml").read_text())
    source["backends"]["hardware"]["hardware_motion_approved"] = True
    config = tmp_path / "config" / "robots"
    config.mkdir(parents=True)
    (config / "pinky_profiles.yaml").write_text(yaml.safe_dump(source), encoding="utf-8")
    with pytest.raises(ProfileError, match="cannot be stored"):
        load_robot(tmp_path, "pinky1", "hardware")
    with pytest.raises(ProfileError, match="unknown robot"):
        load_robot(ROOT, "pinky9", "gazebo")
    with pytest.raises(ModeError, match="unknown"):
        master_replace_allowed("slam")
