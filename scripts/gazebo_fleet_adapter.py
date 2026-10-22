"""Isolated RMF fleet adapter for Gazebo pinky1.

Runs only on ROS domain 77. Navigation is forwarded to the domain 42 gateway.
Does not publish cmd_vel and does not open a motor port.
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

from fleet_adapter.sim_graph import load_sim_session

HOST = "127.0.0.1"
PORT = 17442
MAP = "gazebo_test_room"


def gateway_call(payload: dict, timeout: float = 25.0) -> dict:
    with socket.create_connection((HOST, PORT), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        data = b""
        while b"\n" not in data:
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
    if not data:
        raise ConnectionError("nav gateway closed")
    body = json.loads(data.split(b"\n", 1)[0])
    if not isinstance(body, dict):
        raise ConnectionError("nav gateway reply is not an object")
    return body


def _vec3(x: float, y: float, yaw: float):
    return [float(x), float(y), float(yaw)]


def main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") != "77":
        print(json.dumps({"started": False, "reason": "adapter refuses every domain except 77"}))
        return 2
    if Path("/dev/ttyAMA0").exists() or Path("/dev/ttyAMA4").exists():
        print(json.dumps({"started": False, "reason": "refusing to start while a Pinky serial device exists"}))
        return 3
    session = load_sim_session(ROOT)
    try:
        health = gateway_call({"op": "health"}, timeout=5)
    except OSError as exc:
        print(json.dumps({"started": False, "reason": f"nav gateway is down: {exc}"}))
        return 4
    if not health.get("action_ready") or not health.get("pose_fresh") or not health.get("clock_ok"):
        print(json.dumps({"started": False, "reason": "gazebo Nav2 is not ready", "health": health}))
        return 4
    import rmf_adapter
    import rmf_adapter.easy_full_control as easy

    server_uri = os.environ.get("SIM_RMF_SERVER_URI") or None
    rmf_adapter.init_rclcpp()
    configuration = easy.FleetConfiguration.from_config_files(
        str(ROOT / "config" / "rmf" / "gazebo_test_room" / "fleet.yaml"),
        str(ROOT / "config" / "rmf" / "gazebo_test_room" / "nav.yaml"),
        server_uri,
    )
    if configuration is None:
        print(json.dumps({"started": False, "reason": "fleet configuration rejected"}))
        return 5
    configuration.server_uri = server_uri
    adapter = rmf_adapter.Adapter.make("gazebo_pinky1_fleet_adapter", wait_time=timedelta(seconds=8))
    if adapter is None:
        print(json.dumps({"started": False, "reason": "traffic schedule is not available on domain 77", "robots_added": 0}))
        return 6
    adapter.start()
    fleet = adapter.add_easy_fleet(configuration)
    current = {"execution": None, "generation": None}
    guard = threading.Lock()

    def navigate(destination, execution) -> None:
        def _value(item):
            return item() if callable(item) else item

        def work() -> None:
            try:
                position = list(_value(destination.position))
                name = str(_value(destination.name) or "")
                if _value(destination.map) != MAP:
                    print(json.dumps({"event": "navigate_refused", "reason": "map is not gazebo_test_room", "map": str(_value(destination.map))}), flush=True)
                    return
                reply = gateway_call({
                    "op": "go",
                    "waypoint": name,
                    "x": position[0],
                    "y": position[1],
                    "yaw": position[2] if len(position) > 2 else 0.0,
                })
            except Exception as exc:
                print(json.dumps({"event": "navigate_error", "reason": str(exc)}), flush=True)
                return
            print(json.dumps({"event": "navigate", "reply_ok": reply.get("ok"), "waypoint": reply.get("waypoint"), "goal": reply.get("goal")}), flush=True)
            if not reply.get("ok"):
                return
            ident = execution.identifier
            activity = ident() if callable(ident) else ident
            with guard:
                current["execution"] = execution
                current["activity"] = activity
                current["generation"] = (reply.get("goal") or {}).get("generation")
        threading.Thread(target=work, daemon=True).start()

    def stop(_activity) -> None:
        try:
            reply = gateway_call({"op": "cancel"}, timeout=15)
        except Exception as exc:
            reply = {"ok": False, "reason": str(exc)}
        print(json.dumps({"event": "stop", "reply": reply}), flush=True)

    def action_executor(category, _description, _execution) -> None:
        print(json.dumps({"event": "action_refused", "category": str(category)}), flush=True)

    callbacks = easy.RobotCallbacks(navigate, stop, action_executor)
    pose = health["pose"]
    state = easy.RobotState(MAP, _vec3(pose[0], pose[1], pose[2]), 0.95)
    robot_config = easy.RobotConfiguration(["GZ_SPAWN"])
    handle = fleet.add_robot(session.robot_id, state, robot_config, callbacks)
    if handle is not None:
        handle.set_max_merge_waypoint_distance(0.3)
        handle.set_max_merge_lane_distance(0.5)
        handle.set_min_lane_length(1e-8)
    print(json.dumps({
        "started": handle is not None,
        "robots_added": 1 if handle is not None else 0,
        "fleet": session.fleet_name,
        "robot": session.robot_id,
        "server_uri_sent": bool(server_uri),
        "rmf_domain": 77,
        "nav_domain": 42,
    }), flush=True)
    if handle is None:
        adapter.stop()
        return 7
    while True:
        time.sleep(0.3)
        try:
            poll = gateway_call({"op": "poll"}, timeout=5)
        except Exception as exc:
            print(json.dumps({"event": "gateway_lost", "reason": str(exc)}), flush=True)
            time.sleep(1.0)
            continue
        pose = poll.get("pose")
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
        if pose and len(pose) == 3:
            try:
                handle.update(easy.RobotState(MAP, _vec3(pose[0], pose[1], pose[2]), 0.95), activity)
            except Exception as exc:
                print(json.dumps({"event": "update_error", "reason": str(exc)}), flush=True)
        goal = poll.get("goal") or {}
        if execution is not None and goal.get("generation") == generation and goal.get("phase") == "succeeded":
            try:
                if execution.okay():
                    execution.finished()
                    print(json.dumps({"event": "navigation_finished", "generation": generation, "pose": pose}), flush=True)
            except Exception as exc:
                print(json.dumps({"event": "finish_error", "reason": str(exc)}), flush=True)
            with guard:
                if current["execution"] is execution:
                    current["execution"] = None


if __name__ == "__main__":
    raise SystemExit(main())
