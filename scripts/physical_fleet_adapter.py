"""Physical fleet adapter. Dry-run exits before importing rmf_adapter.

Registration runs only when PHYSICAL_FLEET_REGISTER=1 and the gateway reports
a stopped, fresh encoder segment on its own ROS domain. Motion starts only
from the RMF navigate callback.

TaskManager queues are process memory. This process does not restore a booking
left behind by an older adapter, and it does not mark that booking complete.
"""
from __future__ import annotations

import json
import os
import socket
import sys
import threading
import time
from datetime import timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fleet_adapter.physical_dry_run import decide_registration
from fleet_adapter.physical_motion import (
    MAP_NAME, MERGE_LANE_M, MERGE_WAYPOINT_M, PHYSICAL_ROBOT_IDS, ROTATE_ACTION, ROTATE_TEST,
    may_finish, trial_graph, unrecoverable_goal,
)

HOST = "127.0.0.1"
PORT = 17453


def _with_lease(payload: dict) -> dict:
    """Attach a lease only when this process was started with one. The running adapter has none."""
    lease = os.environ.get("CARE_MOVE_GATEWAY_LEASE", "")
    if not lease:
        return payload
    payload["lease"] = lease
    payload["session_id"] = os.environ.get("CARE_MOVE_GATEWAY_SESSION", "")
    payload["environment_id"] = os.environ.get("CARE_MOVE_GATEWAY_ENVIRONMENT", "")
    return payload


def read_gateway(payload: dict | None = None, timeout: float = 2) -> dict:
    with socket.create_connection((HOST, PORT), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall((json.dumps(payload or {"op": "poll"}) + "\n").encode("utf-8"))
        data = b""
        while b"\n" not in data:
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
    body = json.loads(data.split(b"\n", 1)[0])
    if not isinstance(body, dict):
        raise ConnectionError("physical gateway reply is not an object")
    return body


def _value(item):
    return item() if callable(item) else item


def main() -> int:
    if os.environ.get("PHYSICAL_FLEET_REGISTER") != "1":
        try:
            report = read_gateway()
        except OSError as exc:
            print(json.dumps({"started": False, "registered": False, "reason": f"physical gateway is down: {exc}"}))
            return 4
        print(json.dumps(decide_registration(report), ensure_ascii=False))
        return 0
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        print(json.dumps({"started": False, "registered": False, "reason": "physical fleet adapter uses RMF domain 77"}))
        return 2
    if Path("/dev/ttyAMA0").exists() or Path("/dev/ttyAMA4").exists():
        print(json.dumps({"started": False, "registered": False, "reason": "refusing to register on a host with a Pinky serial device"}))
        return 3
    try:
        report = read_gateway()
    except OSError as exc:
        print(json.dumps({"started": False, "registered": False, "reason": f"physical gateway is down: {exc}"}))
        return 4
    if report.get("mode") != "PHYSICAL_TEST" or not report.get("register_with_rmf"):
        print(json.dumps({"started": False, "registered": False, "reason": "gateway is not ready to register",
                          "mode": report.get("mode"), "reasons": (report.get("decision") or {}).get("reasons")}))
        return 4
    segment = report.get("segment")
    if not isinstance(segment, dict):
        print(json.dumps({"started": False, "registered": False, "reason": "no measured odom segment"}))
        return 4
    robot_id = str(segment.get("robot_id") or report.get("robot_id") or "")
    if robot_id not in PHYSICAL_ROBOT_IDS:
        print(json.dumps({"started": False, "registered": False, "reason": "prepared robot is not a physical Pinky"}))
        return 4
    pose = report.get("pose")
    if not pose or len(pose) < 3:
        print(json.dumps({"started": False, "registered": False, "reason": "encoder pose is missing"}))
        return 4
    fleet_text, nav_text = trial_graph(segment)
    out = Path("/tmp/care-move-e2e/physical_pinky")
    out.mkdir(parents=True, exist_ok=True)
    (out / "fleet.yaml").write_text(fleet_text, encoding="utf-8")
    (out / "nav.yaml").write_text(nav_text, encoding="utf-8")
    import rmf_adapter
    import rmf_adapter.easy_full_control as easy

    server_uri = os.environ.get("SIM_RMF_SERVER_URI") or None
    rmf_adapter.init_rclcpp()
    configuration = easy.FleetConfiguration.from_config_files(str(out / "fleet.yaml"), str(out / "nav.yaml"), server_uri)
    if configuration is None:
        print(json.dumps({"started": False, "registered": False, "reason": "fleet configuration rejected"}))
        return 5
    configuration.server_uri = server_uri
    adapter = rmf_adapter.Adapter.make("physical_pinky_fleet_adapter", wait_time=timedelta(seconds=8))
    if adapter is None:
        print(json.dumps({"started": False, "registered": False, "reason": "traffic schedule is not available on domain 77"}))
        return 6
    adapter.start()
    fleet = adapter.add_easy_fleet(configuration)
    current = {"execution": None, "activity": None, "generation": None}
    guard = threading.Lock()

    def navigate(destination, execution) -> None:
        def work() -> None:
            try:
                position = list(_value(destination.position))
                name = str(_value(destination.name) or "")
                rmf_map = str(_value(destination.map) or "")
                ident = execution.identifier
                activity = ident() if callable(ident) else ident
                if rmf_map != MAP_NAME or name != "P_TEST" or segment.get("kind") == ROTATE_TEST:
                    print(json.dumps({"event": "navigate_refused", "reason": "destination is outside the measured forward test",
                                      "map": rmf_map, "name": name}), flush=True)
                    cancel_unrecoverable("destination is outside the measured forward test")
                    return
                reply = read_gateway(_with_lease({
                    "op": "go", "waypoint": name, "target": position, "robot_id": robot_id,
                    "correlation_id": str(activity),
                }), timeout=5)
            except Exception as exc:
                print(json.dumps({"event": "navigate_error", "reason": str(exc)}), flush=True)
                return
            print(json.dumps({"event": "navigate", "reply_ok": reply.get("ok"), "goal": reply.get("goal"),
                              "reason": reply.get("reason")}), flush=True)
            if not reply.get("ok"):
                reason = str(reply.get("reason") or "")
                if unrecoverable_goal(reason, bool(reply.get("duplicate"))):
                    cancel_unrecoverable(reason)
                return
            with guard:
                current["execution"] = execution
                current["activity"] = activity
                current["generation"] = (reply.get("goal") or {}).get("generation")
        threading.Thread(target=work, daemon=True).start()

    def stop(activity) -> None:
        requested = _value(activity)
        with guard:
            active = current.get("activity")
        if active is not None and requested != active:
            print(json.dumps({"event": "stop_refused", "reason": "activity does not match the active goal"}), flush=True)
            return
        try:
            reply = read_gateway({"op": "cancel"}, timeout=5)
        except Exception as exc:
            reply = {"ok": False, "reason": str(exc)}
        print(json.dumps({"event": "stop", "reply": reply}), flush=True)
        with guard:
            current["execution"] = None
            current["activity"] = None

    def action_executor(category, _description, execution) -> None:
        def work() -> None:
            if str(category) != ROTATE_ACTION or segment.get("kind") != ROTATE_TEST:
                print(json.dumps({"event": "action_refused", "category": str(category)}), flush=True)
                return
            try:
                ident = execution.identifier
                activity = ident() if callable(ident) else ident
                reply = read_gateway(_with_lease({
                    "op": "go", "waypoint": "P_TURN", "robot_id": robot_id,
                    "correlation_id": str(activity),
                }), timeout=5)
            except Exception as exc:
                print(json.dumps({"event": "action_error", "reason": str(exc)}), flush=True)
                return
            print(json.dumps({"event": "rotate", "reply_ok": reply.get("ok"), "goal": reply.get("goal"),
                              "reason": reply.get("reason")}), flush=True)
            if not reply.get("ok"):
                reason = str(reply.get("reason") or "")
                if unrecoverable_goal(reason, bool(reply.get("duplicate"))):
                    cancel_unrecoverable(reason)
                return
            with guard:
                current["execution"] = execution
                current["activity"] = activity
                current["generation"] = (reply.get("goal") or {}).get("generation")
        threading.Thread(target=work, daemon=True).start()

    refused_tasks: set[str] = set()

    def cancel_unrecoverable(reason: str) -> None:
        """Ask RMF to cancel a goal that can never move. Do not report it finished."""
        try:
            task_id = str(handle.more().current_task_id() or "")
        except Exception as exc:
            print(json.dumps({"event": "goal_unrecoverable", "reason": reason, "error": str(exc)}), flush=True)
            return
        if not task_id or task_id in refused_tasks:
            print(json.dumps({"event": "goal_unrecoverable", "reason": reason, "task_id": task_id,
                              "cancel": "not repeated"}), flush=True)
            return
        refused_tasks.add(task_id)

        def done(ok: bool) -> None:
            print(json.dumps({"event": "rmf_cancel_requested", "task_id": task_id, "ok": bool(ok),
                              "reason": reason}), flush=True)

        try:
            handle.more().cancel_task(task_id, ["care-move-unrecoverable-goal"], done)
        except Exception as exc:
            print(json.dumps({"event": "rmf_cancel_error", "task_id": task_id, "reason": str(exc)}), flush=True)

    callbacks = easy.RobotCallbacks(navigate, stop, action_executor)
    state = easy.RobotState(MAP_NAME, [float(pose[0]), float(pose[1]), float(pose[2])], 0.95)
    robot_config = easy.RobotConfiguration(["P_START"])
    handle = fleet.add_robot(robot_id, state, robot_config, callbacks)
    if handle is not None:
        handle.set_max_merge_waypoint_distance(MERGE_WAYPOINT_M)
        handle.set_max_merge_lane_distance(MERGE_LANE_M)
        handle.set_min_lane_length(1e-8)
    print(json.dumps({
        "started": handle is not None,
        "registered": False,
        "robots_added": 1 if handle is not None else 0,
        "fleet": "physical_pinky",
        "robot": robot_id,
        "map": MAP_NAME,
        "merge_waypoint_m": MERGE_WAYPOINT_M,
        "merge_lane_m": MERGE_LANE_M,
        "rmf_domain": 77,
        "note": "robots_added is local; confirm the fleet with the RMF API",
    }), flush=True)
    if handle is None:
        adapter.stop()
        return 7
    while True:
        time.sleep(0.2)
        try:
            poll = read_gateway(timeout=2)
        except Exception as exc:
            print(json.dumps({"event": "gateway_lost", "reason": str(exc)}), flush=True)
            time.sleep(1.0)
            continue
        live = poll.get("pose")
        with guard:
            execution = current["execution"]
            generation = current["generation"]
        activity = None
        if execution is not None:
            try:
                if execution.okay():
                    activity = current.get("activity")
                else:
                    with guard:
                        if current["execution"] is execution:
                            current["execution"] = None
                            current["activity"] = None
                    execution = None
            except Exception as exc:
                print(json.dumps({"event": "activity_error", "reason": str(exc)}), flush=True)
                execution = None
        if live and len(live) >= 3:
            try:
                handle.update(easy.RobotState(MAP_NAME, [float(live[0]), float(live[1]), float(live[2])], 0.95), activity)
            except Exception as exc:
                print(json.dumps({"event": "update_error", "reason": str(exc)}), flush=True)
        goal = poll.get("goal") or {}
        if execution is None or goal.get("generation") != generation:
            continue
        if may_finish(goal, live, bool(poll.get("odom_fresh")), float(poll.get("odom_velocity") or 0.0),
                      float(poll.get("odom_angular") or 0.0)):
            try:
                if execution.okay():
                    execution.finished()
                    print(json.dumps({"event": "navigation_finished", "generation": generation, "pose": live,
                                      "motion_commands": poll.get("motion_commands")}), flush=True)
            except Exception as exc:
                print(json.dumps({"event": "finish_error", "reason": str(exc)}), flush=True)
            with guard:
                if current["execution"] is execution:
                    current["execution"] = None
                    current["activity"] = None
        elif goal.get("phase") == "succeeded":
            print(json.dumps({"event": "finish_withheld", "reason": "gateway succeeded without a stopped measured pose",
                              "pose": live}), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
