"""Physical Pinky gateway on the existing ROS domain 24, TCP 127.0.0.1:17453.

Every connected Pinky shares that domain. Selection uses the robot namespace.
PHYSICAL_DRY_RUN subscribes only. PHYSICAL_TEST may publish the selected
robot's cmd_vel for one prepared 0.05 m move or one in-place 180 degree turn.
"""
from __future__ import annotations

import json
import math
import os
import queue
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fleet_adapter.gateway_mode import DRY_RUN as MODE_DRY_RUN, FAULT, PHYSICAL_TEST, STOPPING, ModeTable
from fleet_adapter.physical_dry_run import DRY_RUN
from fleet_adapter.physical_motion import (
    DRIVE_DOMAIN_ID, FORWARD_TEST, PHYSICAL_ROBOT_IDS, ROTATE_TEST, SESSION_PROTOCOL_VERSION,
    STOP_ANGULAR_RPS, SUPPORTED_TEST_KINDS,
    MoveController, RobotSelection, TrialSession, describe_robots, yaw_from_quat,
)

ODOM_TOPICS = ("/odom", "/pinky1/odom", "/pinky2/odom", "/pinky3/odom")
CMD_TOPICS = ("/cmd_vel", "/pinky1/cmd_vel", "/pinky2/cmd_vel", "/pinky3/cmd_vel")

TEST = "PHYSICAL_TEST"

HOST = "127.0.0.1"
PORT = 17453


def _freeze_identity() -> dict:
    """Identity of the code this process imported. Later edits on disk do not change it."""
    version = "unknown"
    try:
        version = subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
            text=True, stderr=subprocess.DEVNULL,
        ).strip() or "unknown"
        porcelain = subprocess.check_output(
            ["git", "-C", str(ROOT), "status", "--porcelain", "--",
             "scripts/physical_pinky_gateway.py", "fleet_adapter/physical_motion.py",
             "fleet_adapter/gateway_mode.py"],
            text=True, stderr=subprocess.DEVNULL,
        )
        if porcelain.strip():
            version += "-dirty"
    except (OSError, subprocess.SubprocessError):
        version = "unknown"
    return {
        "gateway_version": version,
        "session_protocol_version": SESSION_PROTOCOL_VERSION,
        "supported_test_kinds": list(SUPPORTED_TEST_KINDS),
    }


GATEWAY_IDENTITY = _freeze_identity()


class RosObservation:
    def __init__(self, mode: str) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState, LaserScan

        self.mode = mode
        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("physical_pinky_gateway")
        self.scan = None
        self.scan_at = 0.0
        self.joints = None
        self.odom: dict[str, tuple] = {}
        self.publishers: dict[str, object] = {}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(LaserScan, "/scan", self._scan, qos)
        self.node.create_subscription(JointState, "/joint_states", self._joints, qos)
        from nav_msgs.msg import Odometry
        for topic in ODOM_TOPICS:
            self.node.create_subscription(Odometry, topic, lambda message, name=topic: self._odom(name, message), qos)
        from geometry_msgs.msg import Twist
        self.Twist = Twist
        if mode == TEST:
            self.ensure_publisher("/cmd_vel")

    def _scan(self, message) -> None:
        self.scan = message
        self.scan_at = time.monotonic()

    def _joints(self, message) -> None:
        self.joints = message

    def _odom(self, topic: str, message) -> None:
        self.odom[topic] = (message, time.monotonic())

    def ensure_publisher(self, topic: str) -> None:
        if topic in self.publishers:
            return
        self.publishers[topic] = self.node.create_publisher(self.Twist, topic, 10)

    def release_publisher(self, topic: str) -> None:
        publisher = self.publishers.pop(topic, None)
        if publisher is not None:
            self.node.destroy_publisher(publisher)

    def publish(self, topic: str, linear: float, angular: float) -> None:
        publisher = self.publishers.get(topic)
        if publisher is None:
            return
        message = self.Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        publisher.publish(message)

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def _count(self, topic: str, publishers: bool) -> int:
        try:
            info = self.node.get_publishers_info_by_topic(topic) if publishers else self.node.get_subscriptions_info_by_topic(topic)
            return len(info)
        except Exception:
            return 0

    def snapshot(self, selected_id: str) -> dict:
        names = [name for name, _types in self.node.get_topic_names_and_types()]
        scan = self.scan
        finite = []
        if scan is not None:
            finite = [value for value in scan.ranges if math.isfinite(value)]
        joints = self.joints
        now = time.monotonic()
        odom_publishers = {topic: self._count(topic, True) for topic in ODOM_TOPICS}
        cmd_subscribers = {topic: self._count(topic, False) for topic in CMD_TOPICS}
        samples: dict[str, dict] = {}
        for topic, (message, seen) in self.odom.items():
            rotation = message.pose.pose.orientation
            stamp = message.header.stamp
            samples[topic] = {
                "fresh": now - seen < 0.3,
                "pose": [
                    float(message.pose.pose.position.x),
                    float(message.pose.pose.position.y),
                    yaw_from_quat(rotation.x, rotation.y, rotation.z, rotation.w),
                ],
                "speed": float(message.twist.twist.linear.x),
                "angular": float(message.twist.twist.angular.z),
                "stamp": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            }
        robots = describe_robots(odom_publishers, cmd_subscribers, samples)
        selected = next((item for item in robots if item["id"] == selected_id), None)
        sample = samples.get(selected["odom_topic"]) if selected and selected.get("available") else None
        fresh = bool(sample and sample.get("fresh"))
        age = now - self.scan_at if scan is not None else None
        cmd_topic = selected["cmd_vel_topic"] if selected else "/cmd_vel"
        return {
            "domain": int(os.environ.get("ROS_DOMAIN_ID") or "0"),
            "drive_domain": int(DRIVE_DOMAIN_ID),
            "rmw": os.environ.get("RMW_IMPLEMENTATION"),
            "scan_fresh": age is not None and age < 2.0 and len(finite) >= 50,
            "scan_frame": scan.header.frame_id if scan is not None else None,
            "scan_points": len(finite),
            "joint_names": list(joints.name) if joints is not None else [],
            "joint_positions": [float(item) for item in joints.position] if joints is not None else [],
            "odom_topic": bool(selected and selected.get("available")),
            "nav2_action": any(name.endswith("/navigate_to_pose/_action/status") for name in names),
            "map_topic": "/map" in names,
            "localization_valid": False,
            "cmd_vel_publishers": self._count(cmd_topic, True),
            "cmd_vel_subscribers": cmd_subscribers.get(cmd_topic, 0),
            "cmd_vel_topic": cmd_topic,
            "odom_fresh": fresh,
            "odom_pose": sample["pose"] if fresh and sample else None,
            "odom_speed": sample["speed"] if fresh and sample else 0.0,
            "odom_angular": sample["angular"] if fresh and sample else 0.0,
            "odom_stamp": sample["stamp"] if fresh and sample else None,
            "robots": robots,
            "selected_robot_id": selected_id,
            "mode": self.mode,
        }


def serve(commands: queue.Queue, mode: str) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((HOST, PORT))
    except OSError as exc:
        print(json.dumps({"ready": False, "reason": "127.0.0.1:17453 is already used by the running gateway",
                          "errno": exc.errno}), flush=True)
        os._exit(98)
    server.listen(4)
    server.settimeout(0.2)
    print(json.dumps({"ready": True, "host": HOST, "port": PORT, "mode": mode}), flush=True)
    while True:
        try:
            conn, _addr = server.accept()
        except socket.timeout:
            continue
        threading.Thread(target=_client, args=(commands, conn), daemon=True).start()


def _client(commands: queue.Queue, conn: socket.socket) -> None:
    box = {"event": threading.Event(), "reply": None}
    try:
        data = b""
        conn.settimeout(5)
        while b"\n" not in data:
            chunk = conn.recv(65536)
            if not chunk:
                break
            data += chunk
        payload = json.loads(data.split(b"\n", 1)[0]) if data else {}
        if not isinstance(payload, dict):
            box["reply"] = {"ok": False, "reason": "payload must be an object", "motion_commands": 0}
        else:
            commands.put((payload, box))
            box["event"].wait(5)
        reply = box["reply"] or {"ok": False, "reason": "gateway timed out", "motion_commands": 0}
        conn.sendall((json.dumps(reply) + "\n").encode("utf-8"))
    except Exception as exc:
        try:
            conn.sendall((json.dumps({"ok": False, "reason": str(exc), "motion_commands": 0}) + "\n").encode("utf-8"))
        except OSError:
            pass
    finally:
        conn.close()


class TestGateway:
    """One prepared test for the selected Pinky. Nonzero cmd_vel exists only while moving."""

    def __init__(self, selection: RobotSelection, initial_mode: str = DRY_RUN) -> None:
        self.controllers = {robot_id: MoveController() for robot_id in PHYSICAL_ROBOT_IDS}
        self.sessions = {robot_id: TrialSession() for robot_id in PHYSICAL_ROBOT_IDS}
        self.modes = ModeTable(PHYSICAL_TEST if initial_mode == TEST else MODE_DRY_RUN)
        self.selection = selection

    @property
    def controller(self) -> MoveController:
        return self.controllers[self.selection.robot_id]

    @property
    def session(self) -> TrialSession:
        return self.sessions[self.selection.robot_id]

    def handle(self, payload: dict, observation: dict) -> dict:
        op = str(payload.get("op") or "")
        pose = _pose(observation)
        robot_id = self.selection.robot_id
        if op == "arm":
            blocked = _lease_block(payload, op)
            if blocked:
                blocked["motion_commands"] = self.controller.motion_commands
                return blocked
            requested = str(payload.get("robot_id") or robot_id)
            if requested != robot_id:
                return {"ok": False, "reason": "select that Pinky before arming it", "motion_commands": self.controller.motion_commands}
            robot = _robot(observation, robot_id)
            result = self.modes.arm(
                robot_id, robot, self.controller.phase,
                self.session.open and not self.session.submitted,
                str(payload.get("kind") or FORWARD_TEST),
            )
            if result.get("ok") and not result.get("already"):
                result["ensure_cmd_vel"] = robot.get("cmd_vel_topic") or "/cmd_vel"
            result["motion_commands"] = self.controller.motion_commands
            result["robot_id"] = robot_id
            return result
        if op == "disarm":
            requested = str(payload.get("robot_id") or robot_id)
            robot = _robot(observation, requested)
            controller = self.controllers.get(requested) or self.controller
            speed = float(robot.get("speed") or 0.0)
            angular = float(robot.get("angular") or 0.0)
            if requested == robot_id and pose is not None:
                speed = float(pose.get("speed") or speed)
                angular = float(pose.get("angular") or angular)
            result = self.modes.disarm(requested, speed, angular, controller.phase)
            if result.get("mode") == MODE_DRY_RUN:
                result["release_cmd_vel"] = robot.get("cmd_vel_topic") or "/cmd_vel"
            result["motion_commands"] = controller.motion_commands
            return result
        if op == "select":
            result = self.selection.choose(
                str(payload.get("robot_id") or ""),
                observation.get("robots") or [],
                self.controller.phase,
                self.session.open,
                self.session.submitted,
            )
            result["motion_commands"] = self.controller.motion_commands
            return result
        if op == "prepare":
            blocked = _lease_block(payload, op)
            if blocked:
                blocked["motion_commands"] = self.controller.motion_commands
                return blocked
            if self.modes.get(robot_id) != PHYSICAL_TEST:
                return {"ok": False, "reason": "robot is not in PHYSICAL_TEST", "motion_commands": self.controller.motion_commands}
            result = self.session.prepare(
                pose["x"] if pose else 0.0,
                pose["y"] if pose else 0.0,
                pose["yaw"] if pose else 0.0,
                self.controller.phase,
                bool(pose and pose["fresh"]),
                float(pose["speed"]) if pose else 0.0,
                kind=str(payload.get("kind") or FORWARD_TEST),
                robot_id=self.selection.robot_id,
                angular=float(pose["angular"]) if pose else 0.0,
            )
            result["goal"] = self.controller.snapshot()
            result["motion_commands"] = self.controller.motion_commands
            return result
        if op in {"health", "poll"}:
            return _status(observation, self.controller, self.session, self.selection, self.modes.get(robot_id), self.modes.snapshot())
        if op == "go":
            blocked = _lease_block(payload, op)
            if blocked:
                blocked["motion_commands"] = self.controller.motion_commands
                return blocked
            if self.modes.get(robot_id) != PHYSICAL_TEST:
                return {"ok": False, "refused": True, "motion_commands": self.controller.motion_commands,
                        "reason": "robot is not in PHYSICAL_TEST"}
            if not self.session.open or self.session.segment is None:
                return {"ok": False, "refused": True, "motion_commands": self.controller.motion_commands,
                        "reason": "no prepared test session"}
            segment = self.session.segment
            if str(payload.get("robot_id") or segment.get("robot_id")) != segment.get("robot_id"):
                return {"ok": False, "refused": True, "motion_commands": self.controller.motion_commands,
                        "reason": "goal robot does not match the prepared Pinky"}
            if segment.get("kind") == ROTATE_TEST:
                if str(payload.get("waypoint") or "") != "P_TURN":
                    return {"ok": False, "refused": True, "motion_commands": self.controller.motion_commands,
                            "reason": "rotation goal is not the locked turn"}
                if pose is None:
                    return {"ok": False, "refused": True, "motion_commands": self.controller.motion_commands,
                            "reason": "no fresh odometry target"}
                result = self.controller.go_rotate(pose, float(segment["P_TURN"][2]), str(payload.get("correlation_id") or ""))
            else:
                target = _target(payload, segment)
                if pose is None or target is None:
                    return {"ok": False, "refused": True, "motion_commands": self.controller.motion_commands,
                            "reason": "no fresh odometry target"}
                raw = payload.get("target")
                if isinstance(raw, list) and len(raw) >= 2:
                    if math.hypot(float(raw[0]) - target[0], float(raw[1]) - target[1]) > 0.005:
                        return {"ok": False, "refused": True, "motion_commands": self.controller.motion_commands,
                                "reason": "rmf coordinate does not match the measured odom target"}
                result = self.controller.go(pose, target, str(payload.get("correlation_id") or ""))
            if result.get("ok"):
                generation = (result.get("goal") or {}).get("generation")
                self.session.mark_submitted(generation if isinstance(generation, int) else None)
            result["goal"] = self.controller.snapshot()
            result["session_id"] = self.session.session_id
            return result
        if op == "abandon":
            if self.controller.phase in {"moving", "stopping"}:
                return {"ok": False, "reason": "cannot abandon an active goal",
                        "motion_commands": self.controller.motion_commands}
            self.session.abandon()
            return {"ok": True, "motion_commands": self.controller.motion_commands, "session_open": False}
        if op in {"cancel", "stop"}:
            result = self.controller.cancel()
            result["goal"] = self.controller.snapshot()
            return result
        return {"ok": False, "motion_commands": self.controller.motion_commands, "reason": "unknown op"}

    def commands(self, observation: dict) -> list[tuple[str, float, float]]:
        """Publish only for robots that are in a test. Other Pinkies get no command."""
        robots = {item.get("id"): item for item in observation.get("robots") or []}
        output = []
        for robot_id, controller in self.controllers.items():
            mode = self.modes.get(robot_id)
            if mode not in {PHYSICAL_TEST, STOPPING, FAULT}:
                continue
            robot = robots.get(robot_id) or {}
            topic = str(robot.get("cmd_vel_topic") or "/cmd_vel")
            if mode in {STOPPING, FAULT}:
                output.append((topic, 0.0, 0.0))
                continue
            if robot_id == self.selection.robot_id:
                pose = _pose(observation) or {"fresh": False}
            else:
                pose = _robot_pose(robot)
            linear, angular = controller.tick(pose)
            output.append((topic, linear, angular))
        return output


def _bind_lease_authority():
    if os.environ.get("GATEWAY_LEASE_ENFORCED") != "1":
        return None
    from fleet_adapter.control_lease import LeaseAuthority, bind_enforced_authority
    secret_path = Path(os.environ.get("CARE_MOVE_GATEWAY_LEASE_SECRET", ""))
    if not secret_path.is_file():
        return None
    authority = LeaseAuthority(secret_path.read_text(encoding="utf-8").strip(), secret_path.with_suffix(".generation"))
    bind_enforced_authority(authority)
    return authority


def release_if_lease_expired(gateway: TestGateway, authority, robot_id: str, now: float | None = None) -> dict | None:
    """Cancel an active physical goal when its lease is gone. This does not issue a new motion command."""
    if os.environ.get("GATEWAY_LEASE_ENFORCED") != "1" or authority is None:
        return None
    if authority.current_valid(robot_id, now):
        return None
    if gateway.controller.phase not in {"moving", "stopping"}:
        return None
    return gateway.handle({"op": "cancel", "robot_id": robot_id}, {})


def _lease_block(payload: dict, op: str) -> dict | None:
    from fleet_adapter.control_lease import lease_refusal_if_enforced
    return lease_refusal_if_enforced(payload, op)


def _robot(observation: dict, robot_id: str) -> dict:
    for item in observation.get("robots") or []:
        if item.get("id") == robot_id:
            return item
    return {"id": robot_id, "available": False}


def _robot_pose(robot: dict) -> dict:
    raw = robot.get("pose")
    if not raw:
        return {"fresh": False, "x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 0.0, "angular": 0.0}
    return {"x": raw[0], "y": raw[1], "yaw": raw[2] if len(raw) > 2 else 0.0,
            "speed": robot.get("speed") or 0.0, "angular": robot.get("angular") or 0.0,
            "fresh": bool(robot.get("odom_fresh"))}


def _pose(observation: dict) -> dict | None:
    raw = observation.get("odom_pose")
    if not raw:
        return None
    return {"x": raw[0], "y": raw[1], "yaw": raw[2], "speed": observation.get("odom_speed") or 0.0,
            "angular": observation.get("odom_angular") or 0.0,
            "fresh": bool(observation.get("odom_fresh")), "stamp": observation.get("odom_stamp")}


def _target(payload: dict, segment: dict | None) -> list[float] | None:
    name = str(payload.get("waypoint") or "")
    if segment and name in {"P_START", "P_TEST"}:
        return list(segment[name])
    raw = payload.get("target")
    if isinstance(raw, list) and len(raw) >= 2:
        return [float(item) for item in raw[:3]]
    return None


def _status(observation: dict, controller: MoveController, session: TrialSession, selection: RobotSelection,
            robot_mode: str = TEST, robot_modes: dict | None = None) -> dict:
    goal = controller.snapshot()
    fresh = bool(observation.get("odom_fresh"))
    stopped = abs(float(observation.get("odom_speed") or 0.0)) <= 0.01 and abs(float(observation.get("odom_angular") or 0.0)) <= STOP_ANGULAR_RPS
    motor = int(observation.get("cmd_vel_subscribers") or 0) >= 1
    segment = session.segment if session.open else None
    robot_id = selection.robot_id
    kind = (segment or {}).get("kind")
    ready = (robot_mode == PHYSICAL_TEST and fresh and segment is not None and stopped and motor and session.open
             and not session.submitted and controller.phase not in {"moving", "stopping"}
             and segment.get("robot_id") == robot_id)
    robots = []
    for item in observation.get("robots") or []:
        copied = dict(item)
        copied["gateway_mode"] = (robot_modes or {}).get(item.get("id"), MODE_DRY_RUN)
        robots.append(copied)
    reasons = []
    if not fresh:
        reasons.append("fresh encoder odometry is required before a physical task")
    if not motor:
        reasons.append("cmd_vel has no subscriber")
    if not stopped:
        reasons.append("the robot is still moving")
    if segment is None:
        reasons.append("no prepared test session")
    return {
        "ok": True,
        "gateway_version": GATEWAY_IDENTITY["gateway_version"],
        "session_protocol_version": GATEWAY_IDENTITY["session_protocol_version"],
        "supported_test_kinds": GATEWAY_IDENTITY["supported_test_kinds"],
        "robot_id": robot_id,
        "selected_robot_id": robot_id,
        "robots": robots,
        "robot_modes": robot_modes or {},
        "test_kind": kind,
        "turned_rad": goal.get("turned"),
        "odom_angular": observation.get("odom_angular") if fresh else None,
        "cmd_vel_topic": observation.get("cmd_vel_topic"),
        "mode": robot_mode,
        "connected": True,
        "reachable": True,
        "motor_ready": motor,
        "odom_fresh": fresh,
        "odom_pose": observation.get("odom_pose") if fresh else None,
        "odom_velocity": observation.get("odom_speed") if fresh else None,
        "odom_stamp": observation.get("odom_stamp") if fresh else None,
        "domain": observation.get("domain"),
        "motion_commands": controller.motion_commands,
        "lease_enforced": os.environ.get("GATEWAY_LEASE_ENFORCED") == "1",
        "pose": observation.get("odom_pose") if fresh else None,
        "pose_source": "odom" if fresh else None,
        "goal": goal,
        "goal_id": goal["correlation_id"],
        "goal_generation": goal["generation"],
        "goal_phase": goal["phase"],
        "initial_pose": goal["initial_pose"],
        "target_pose": goal["target_pose"],
        "current_pose": observation.get("odom_pose") if fresh else None,
        "distance_traveled": goal["traveled"],
        "remaining_distance": goal["remaining"],
        "current_velocity": observation.get("odom_speed") if fresh else None,
        "odom_timestamp": observation.get("odom_stamp") if fresh else None,
        "last_error": goal["fail_reason"],
        "session_id": session.session_id if session.open else "",
        "session_open": session.open,
        "session_submitted": session.submitted,
        "session_goal_generation": session.goal_generation,
        "target_distance_m": 0.05 if kind != ROTATE_TEST else 0.0,
        "target_yaw_rad": (segment or {}).get("yaw_delta_rad"),
        "waypoints": (["P_START", "P_TURN"] if kind == ROTATE_TEST else ["P_START", "P_TEST"]) if segment else [],
        "segment": segment,
        "register_with_rmf": ready,
        "decision": {
            "register_with_rmf": ready,
            "motion_allowed": False,
            "robot_id": robot_id,
            "fleet_name": "physical_pinky",
            "pose": observation.get("odom_pose") if fresh else None,
            "reasons": reasons,
        },
    }


def main() -> int:
    domain = os.environ.get("ROS_DOMAIN_ID")
    if domain != DRIVE_DOMAIN_ID:
        print(json.dumps({"ready": False, "reason": "physical tests use the existing Pinky ROS domain 24"}))
        return 2
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        print(json.dumps({"ready": False, "reason": "physical gateway requires rmw_fastrtps_cpp"}))
        return 2
    mode = os.environ.get("PHYSICAL_PINKY_MODE", DRY_RUN)
    if mode not in {DRY_RUN, TEST}:
        print(json.dumps({"ready": False, "reason": "PHYSICAL_ARMED is refused in this process"}))
        return 3
    if mode == TEST and os.environ.get("GATEWAY_LEASE_ENFORCED") != "1":
        print(json.dumps({"ready": False, "reason": "PHYSICAL_TEST requires lease enforcement"}))
        return 3
    authority = _bind_lease_authority()
    if os.environ.get("GATEWAY_LEASE_ENFORCED") == "1" and authority is None:
        print(json.dumps({"ready": False, "reason": "lease enforcement is fail-closed without a local secret file"}))
        return 3
    commands: queue.Queue = queue.Queue()
    ros = RosObservation(mode)
    selection = RobotSelection()
    test = TestGateway(selection, mode)
    threading.Thread(target=serve, args=(commands, mode), daemon=True).start()
    while True:
        ros.spin_once()
        observation = ros.snapshot(selection.robot_id)
        release_if_lease_expired(test, authority, selection.robot_id)
        for topic, linear, angular in test.commands(observation):
            ros.publish(topic, linear, angular)
        try:
            payload, box = commands.get_nowait()
        except queue.Empty:
            continue
        box["reply"] = test.handle(payload, observation)
        reply = box["reply"] or {}
        if reply.get("ensure_cmd_vel"):
            ros.ensure_publisher(str(reply["ensure_cmd_vel"]))
        if reply.get("release_cmd_vel"):
            ros.release_publisher(str(reply["release_cmd_vel"]))
        box["event"].set()


if __name__ == "__main__":
    raise SystemExit(main())
