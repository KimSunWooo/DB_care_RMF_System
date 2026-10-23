import importlib.util
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from fleet_adapter.control_lease import LeaseAuthority, bind_enforced_authority
from fleet_adapter.mock_control_gateway import FORBIDDEN_PORTS, MockControlGateway, request, serve

ROOT = Path(__file__).resolve().parents[1]
ROBOT = "mock_pinky1"
ENV_B = "rmf-b-domain91"
ENV_A = "rmf-a-domain77"


@pytest.fixture()
def stack(tmp_path):
    authority = LeaseAuthority("test-lease-secret", tmp_path / "generation")
    gateway = MockControlGateway(authority)
    server = serve(gateway, 17491)
    try:
        yield gateway, authority
    finally:
        server.close()


def _issue(authority, environment, session, ttl=30.0, now=None):
    return {
        "op": "issue_lease",
        "issuer_secret": authority.issuer_secret,
        "environment_id": environment,
        "robot_id": ROBOT,
        "session_id": session,
        "ttl_s": ttl,
        "now": now,
    }


def _go(lease, environment, session, robot=ROBOT, now=None):
    return {
        "op": "go", "lease": lease, "environment_id": environment, "robot_id": robot,
        "session_id": session, "now": now,
    }


def test_mock_protocol_rejects_direct_and_stale_commands(stack):
    gateway, authority = stack
    missing = request(17491, {"op": "arm", "robot_id": ROBOT, "environment_id": ENV_B, "session_id": "s"})
    assert missing["ok"] is False and gateway.motion_commands == 0

    forged = request(17491, _issue(authority, ENV_B, "s-b") | {"issuer_secret": "not-the-issuer"})
    assert forged["ok"] is False

    first = request(17491, _issue(authority, ENV_A, "session-a"))
    second = request(17491, _issue(authority, ENV_B, "session-b"))
    assert second["control_generation"] > first["control_generation"]

    armed = request(17491, _go(second["lease"], ENV_B, "session-b") | {"op": "arm"})
    assert armed["ok"] is True and armed["motion_commands"] == 0
    moved = request(17491, _go(second["lease"], ENV_B, "session-b"))
    assert moved["ok"] is True and moved["motion_commands"] == 1

    old_environment = request(17491, _go(first["lease"], ENV_A, "session-a"))
    wrong_environment = request(17491, _go(second["lease"], ENV_A, "session-b"))
    old_generation = request(17491, _go(first["lease"], ENV_B, "session-b"))
    other_robot = request(17491, _go(second["lease"], ENV_B, "session-b", robot="mock_pinky2"))
    old_session = request(17491, _go(second["lease"], ENV_B, "session-a"))
    tampered = request(17491, _go(second["lease"][:-4] + "abcd", ENV_B, "session-b"))
    assert old_environment["ok"] is False
    assert wrong_environment["reason"] == "environment가 다릅니다"
    assert old_generation["ok"] is False
    assert other_robot["ok"] is False
    assert old_session["ok"] is False
    assert tampered["reason"] == "lease가 없거나 변조되었습니다"
    assert request(17491, {"op": "poll"})["motion_commands"] == 1


def test_revoke_during_go_does_not_emit_motion(stack):
    gateway, authority = stack
    issued = request(17491, _issue(authority, ENV_B, "race"))
    assert request(17491, _go(issued["lease"], ENV_B, "race") | {"op": "arm"})["ok"] is True

    def revoke_after_check():
        authority.revoke(ROBOT)

    gateway.after_check = revoke_after_check
    raced = request(17491, _go(issued["lease"], ENV_B, "race"))
    gateway.after_check = None
    assert raced["ok"] is False
    assert raced["motion_commands"] == 0
    assert request(17491, {"op": "poll"})["goal_phase"] == "idle"


def test_expiry_stops_motion_without_another_command(stack):
    _gateway, authority = stack
    issued = request(17491, _issue(authority, ENV_B, "short", ttl=0.2))
    assert request(17491, _go(issued["lease"], ENV_B, "short") | {"op": "arm"})["ok"] is True
    started = time.monotonic()
    assert request(17491, _go(issued["lease"], ENV_B, "short"))["motion_commands"] == 1
    status = {}
    for _ in range(20):
        status = request(17491, {"op": "poll"})
        if status["goal_phase"] == "idle" and status["fault"] == "lease expired":
            break
        time.sleep(0.05)
    elapsed = time.monotonic() - started
    assert status["goal_phase"] == "idle"
    assert status["motion_commands"] == 1
    assert elapsed < 1.5
    expired = request(17491, _go(issued["lease"], ENV_B, "short"))
    assert expired["ok"] is False and expired["motion_commands"] == 1


def test_restart_invalidates_the_old_boot(tmp_path):
    path = tmp_path / "generation"
    first_authority = LeaseAuthority("test-lease-secret", path)
    first = MockControlGateway(first_authority)
    issued = first.handle(_issue(first_authority, ENV_B, "boot"))
    assert first.handle(_go(issued["lease"], ENV_B, "boot") | {"op": "arm"})["ok"] is True
    assert first.handle(_go(issued["lease"], ENV_B, "boot"))["motion_commands"] == 1
    second_authority = LeaseAuthority("test-lease-secret", path)
    second = MockControlGateway(second_authority)
    assert second_authority.boot_id != first_authority.boot_id
    assert second_authority.generation == first_authority.generation
    replay = second.handle(_go(issued["lease"], ENV_B, "boot"))
    assert replay["ok"] is False and second.motion_commands == 0
    assert second.status()["mode"] == "PHYSICAL_DRY_RUN"


def test_stop_is_not_a_public_motion_command(stack):
    gateway, authority = stack
    issued = request(17491, _issue(authority, ENV_B, "stop"))
    request(17491, _go(issued["lease"], ENV_B, "stop") | {"op": "arm"})
    request(17491, _go(issued["lease"], ENV_B, "stop"))
    stranger = request(17491, {"op": "stop", "robot_id": ROBOT})
    assert stranger["ok"] is False and stranger["motion_commands"] == 1
    stopped = request(17491, {"op": "stop", "robot_id": ROBOT, "emergency_stop": authority.emergency_stop})
    assert stopped["ok"] is True and stopped["motion_commands"] == 1
    assert request(17491, {"op": "poll"})["goal_phase"] == "idle"


def test_cutover_keeps_one_generation_and_does_not_revive_the_old_lease(stack):
    _gateway, authority = stack
    old = request(17491, _issue(authority, ENV_A, "old"))
    # A failed precheck does not ask the gateway for a new lease and does not move.
    assert request(17491, {"op": "go", "environment_id": ENV_B, "robot_id": ROBOT, "session_id": "new"})["ok"] is False
    assert request(17491, {"op": "poll"})["motion_commands"] == 0
    new = request(17491, _issue(authority, ENV_B, "new"))
    assert request(17491, _go(old["lease"], ENV_A, "old"))["ok"] is False
    assert request(17491, _go(new["lease"], ENV_B, "new") | {"op": "arm"})["ok"] is True
    assert request(17491, _go(new["lease"], ENV_B, "new"))["motion_commands"] == 1
    rolled = request(17491, _issue(authority, ENV_A, "rollback"))
    assert request(17491, _go(new["lease"], ENV_B, "new"))["ok"] is False
    assert request(17491, _go(old["lease"], ENV_A, "old"))["ok"] is False
    assert request(17491, _go(rolled["lease"], ENV_A, "rollback") | {"op": "arm"})["ok"] is True


def test_controller_process_exit_stops_the_goal_and_does_not_replay(tmp_path):
    authority = LeaseAuthority("test-lease-secret", tmp_path / "generation")
    gateway = MockControlGateway(authority)
    server = serve(gateway, 17491)
    child = subprocess.Popen(
        [sys.executable, "-c", _HOLDER],
        env={**os.environ, "PYTHONPATH": str(ROOT), "PYTHONNOUSERSITE": "1", "ISSUER_SECRET": authority.issuer_secret},
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        status = {}
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            status = request(17491, {"op": "poll"})
            if status["motion_commands"] == 1 and status["goal_phase"] == "moving":
                break
            time.sleep(0.05)
        assert status["goal_phase"] == "moving"
        child.kill()
        child.wait(timeout=2)
        stopped = {}
        started = time.monotonic()
        for _ in range(30):
            stopped = request(17491, {"op": "poll"})
            if stopped["goal_phase"] == "idle":
                break
            time.sleep(0.05)
        assert stopped["goal_phase"] == "idle"
        assert stopped["fault"] == "lease expired"
        assert stopped["motion_commands"] == 1
        assert time.monotonic() - started < 2.0
        replay = request(17491, _go(authority.issue(ENV_B, ROBOT, "session-held", ttl_s=1.0)["lease"], ENV_B, "session-held"))
        # A new lease is not an automatic replay of the old goal: GO still has to be sent,
        # and the killed hold does not leave a second command behind.
        assert replay["motion_commands"] == 1
        assert request(17491, {"op": "poll"})["motion_commands"] == 1
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=2)
        server.close()


_HOLDER = """
import os, time
from fleet_adapter.mock_control_gateway import request
secret = os.environ["ISSUER_SECRET"]
issued = request(17491, {"op":"issue_lease","issuer_secret":secret,"environment_id":"rmf-b-domain91","robot_id":"mock_pinky1","session_id":"session-held","ttl_s":1.0})
lease = issued["lease"]
arm = {"op":"arm","lease":lease,"environment_id":"rmf-b-domain91","robot_id":"mock_pinky1","session_id":"session-held"}
go = dict(arm, op="go")
request(17491, arm)
request(17491, go)
time.sleep(30)
"""


def test_watchdog_invalidates_a_goal_without_another_request(stack):
    _gateway, authority = stack
    issued = request(17491, _issue(authority, ENV_B, "watch", ttl=0.25))
    assert request(17491, _go(issued["lease"], ENV_B, "watch") | {"op": "arm"})["ok"] is True
    assert request(17491, _go(issued["lease"], ENV_B, "watch"))["motion_commands"] == 1
    time.sleep(0.7)
    before_poll = time.time()
    status = request(17491, {"op": "poll"})
    watch = status["lease_watch"]
    assert status["goal_phase"] == "idle"
    assert status["mode"] == "PHYSICAL_DRY_RUN"
    assert status["motion_commands"] == 1
    assert watch["last_motion_command_at"] < watch["goal_invalidated_at"] < before_poll
    assert watch["lease_expired_at"] == watch["stop_requested_at"] == watch["goal_invalidated_at"]
    late = request(17491, _go(issued["lease"], ENV_B, "watch"))
    assert late["ok"] is False and late["motion_commands"] == 1


def test_eight_adapter_loss_points_do_not_replay_motion(stack):
    gateway, authority = stack
    # 1. Submitted, adapter gone before accept.
    assert request(17491, {"op": "go", "environment_id": ENV_B, "robot_id": ROBOT, "session_id": "s1"})["motion_commands"] == 0
    # 2. Accepted lease, adapter gone before ARM.
    accepted = request(17491, _issue(authority, ENV_B, "s2", ttl=30))
    request(17491, {"op": "revoke", "issuer_secret": authority.issuer_secret, "robot_id": ROBOT})
    assert request(17491, _go(accepted["lease"], ENV_B, "s2"))["motion_commands"] == 0
    # 3. ARM accepted, adapter gone before GO.
    armed = request(17491, _issue(authority, ENV_B, "s3"))
    assert request(17491, _go(armed["lease"], ENV_B, "s3") | {"op": "arm"})["ok"] is True
    request(17491, {"op": "revoke", "issuer_secret": authority.issuer_secret, "robot_id": ROBOT})
    assert request(17491, _go(armed["lease"], ENV_B, "s3"))["motion_commands"] == 0
    # 4. GO accepted, then the lease is revoked before another command.
    moving = request(17491, _issue(authority, ENV_B, "s4", ttl=0.2))
    request(17491, _go(moving["lease"], ENV_B, "s4") | {"op": "arm"})
    assert request(17491, _go(moving["lease"], ENV_B, "s4"))["motion_commands"] == 1
    # 5. Renewal stops while the goal is active. The watchdog, not a new GO, clears it.
    time.sleep(0.6)
    expired = request(17491, {"op": "poll"})
    assert expired["goal_phase"] == "idle" and expired["motion_commands"] == 1
    # 6. A late GO from the old session does not count as arrival.
    assert request(17491, _go(moving["lease"], ENV_B, "s4"))["ok"] is False
    # 7. A renewed replacement session can arm, and the old token still cannot move.
    renewed = request(17491, _issue(authority, ENV_B, "s7"))
    assert request(17491, _go(moving["lease"], ENV_B, "s4") | {"op": "arm"})["ok"] is False
    assert request(17491, _go(renewed["lease"], ENV_B, "s7") | {"op": "arm"})["ok"] is True
    # 8. Completing the old session does not add motion, and a new GO is a separate command.
    assert request(17491, _go(renewed["lease"], ENV_B, "s7"))["motion_commands"] == 2
    assert gateway.motion_commands == 2


def test_expired_physical_goal_is_canceled_without_a_new_command(monkeypatch):
    monkeypatch.setenv("GATEWAY_LEASE_ENFORCED", "1")
    spec = importlib.util.spec_from_file_location(
        "physical_gateway_expiry", ROOT / "scripts" / "physical_pinky_gateway.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    authority = LeaseAuthority("test-lease-secret")
    issued = authority.issue(ENV_B, "real_pinky1", "physical-session", ttl_s=1, now=100.0)
    gateway = module.TestGateway(module.RobotSelection())
    gateway.controller.phase = "moving"
    canceled = module.release_if_lease_expired(gateway, authority, "real_pinky1", now=issued["expires_at"] + 1)
    assert canceled["ok"] is True
    assert gateway.controller.phase == "canceled"
    assert gateway.controller.motion_commands == 0
    monkeypatch.delenv("GATEWAY_LEASE_ENFORCED", raising=False)


def test_production_port_and_unenforced_gateway_class(monkeypatch):
    with pytest.raises(RuntimeError):
        serve(MockControlGateway(LeaseAuthority("x")), 17453)
    assert 17453 in FORBIDDEN_PORTS
    spec = importlib.util.spec_from_file_location("physical_gateway_under_test", ROOT / "scripts" / "physical_pinky_gateway.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("GATEWAY_LEASE_ENFORCED", "1")
    bind_enforced_authority(None)
    gateway = module.TestGateway(module.RobotSelection())
    refused = gateway.handle({"op": "go", "robot_id": "real_pinky1"}, {"odom_pose": [0, 0, 0], "odom_fresh": True})
    assert refused["ok"] is False
    assert refused["motion_commands"] == 0
    monkeypatch.delenv("GATEWAY_LEASE_ENFORCED", raising=False)
    bind_enforced_authority(None)
