from pathlib import Path

import pytest

from fleet_adapter.config import FleetConfigError, load_fleet_spec, read_graph_names
from fleet_adapter.nav2_link import Nav2Busy, Nav2Link, Nav2Unavailable
from fleet_adapter.safety import navigation_block_reason

ROOT = Path(__file__).resolve().parents[1]
FLEET = ROOT / "config" / "rmf" / "care_move_fleet.yaml"
GRAPH = ROOT / "config" / "rmf" / "care_move_placeholder.nav.yaml"


def test_only_pinky1_is_active_and_motion_stays_closed():
    spec = load_fleet_spec(FLEET, ROOT)
    assert spec.fleet_name == "care_move"
    assert [robot.robot_id for robot in spec.active_robots] == ["pinky1"]
    assert spec.active_robots[0].ros_namespace == "pinky1"
    assert spec.active_robots[0].navigate_action == "navigate_to_pose"
    disabled = [robot.robot_id for robot in spec.robots if not robot.enabled]
    assert disabled == ["pinky2", "pinky3"]
    assert spec.server_uri == "ws://127.0.0.1:8000/_internal"
    names = read_graph_names(GRAPH)
    assert names[:2] == ["N_DOCK1", "N_ROOM102"]
    assert navigation_block_reason(
        graph_alignment=spec.graph_alignment,
        allow_motion=spec.allow_motion,
        simulation_confirmed=spec.simulation_confirmed,
        action_available=True,
        pose_fresh=True,
    )


def test_motion_gate_requires_surveyed_graph_and_live_nav():
    assert navigation_block_reason(
        graph_alignment="surveyed", allow_motion=True, simulation_confirmed=True,
        action_available=False, pose_fresh=True,
    )
    assert navigation_block_reason(
        graph_alignment="surveyed", allow_motion=True, simulation_confirmed=False,
        action_available=True, pose_fresh=True,
    )
    assert navigation_block_reason(
        graph_alignment="surveyed", allow_motion=True, simulation_confirmed=True,
        action_available=True, pose_fresh=False,
    ) is not None
    assert navigation_block_reason(
        graph_alignment="surveyed", allow_motion=True, simulation_confirmed=True,
        action_available=True, pose_fresh=True,
    ) is None


def test_nav2_link_rejects_missing_server_and_tracks_goal_lifecycle():
    link = Nav2Link("pinky1", "pinky1")
    assert link.action_path == "/pinky1/navigate_to_pose"
    with pytest.raises(Nav2Unavailable):
        link.request_goal(1, 2, 0)
    link.set_action_available(True)
    goal = link.request_goal(5, 0, 0)
    assert goal.phase == "pending"
    with pytest.raises(Nav2Busy):
        link.request_goal(0, 0, 0)
    link.mark_rejected()
    assert link.goal.phase == "rejected"
    link.request_goal(5, 0, 0)
    link.mark_accepted()
    link.observe_feedback(1, 0, 0, 10)
    assert link.pose_fresh(11)
    assert not link.pose_fresh(13)
    link.mark_result(False)
    assert link.goal.phase == "failed"


def test_cancel_timeout_disconnect_and_reconnect():
    link = Nav2Link("pinky1", "pinky1", goal_timeout_sec=30)
    link.set_action_available(True)
    link.request_goal(1, 0, 0)
    link.mark_accepted()
    link.request_cancel()
    link.mark_canceled()
    assert link.goal.phase == "canceled"
    link.request_goal(1, 0, 0)
    link.mark_accepted()
    link.mark_goal_timeout(50, 10)
    assert link.goal.phase == "timed_out"
    link.mark_disconnected()
    assert link.disconnects == 1
    assert not link.action_available
    link.set_action_available(True)
    assert link.action_available


def test_enabled_robot_without_charger_is_rejected(tmp_path: Path):
    text = FLEET.read_text(encoding="utf-8").replace("      charger: N_DOCK1\n", "")
    path = tmp_path / "fleet.yaml"
    path.write_text(text, encoding="utf-8")
    graph = tmp_path / "graph.yaml"
    graph.write_text(GRAPH.read_text(encoding="utf-8"), encoding="utf-8")
    broken = text.replace("config/rmf/care_move_placeholder.nav.yaml", str(graph))
    path.write_text(broken, encoding="utf-8")
    with pytest.raises(FleetConfigError):
        load_fleet_spec(path, tmp_path)
