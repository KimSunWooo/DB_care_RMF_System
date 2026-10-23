"""Short encoder move for one physical Pinky. No Nav2 and no Gazebo coordinates."""
from __future__ import annotations

import math
import time
import uuid

MAX_LINEAR_MPS = 0.04
TARGET_DISTANCE_M = 0.05
MAX_DISTANCE_M = 0.06
GOAL_TOLERANCE_M = 0.01
MIN_TRAVEL_M = 0.04
MAX_LATERAL_M = 0.02
MAX_HEADING_ERROR_RAD = 0.35
STOP_SPEED_MPS = 0.01
STOP_SAMPLES = 3
MOVE_TIMEOUT_SEC = 15.0
TARGET_YAW_RAD = math.pi
YAW_TOLERANCE_RAD = 0.087
MIN_YAW_RAD = TARGET_YAW_RAD - YAW_TOLERANCE_RAD
MAX_YAW_RAD = TARGET_YAW_RAD + 0.17
MAX_SPIN_TRANSLATION_M = 0.03
MAX_ANGULAR_RPS = 0.3
STOP_ANGULAR_RPS = 0.05
SPIN_TIMEOUT_SEC = 20.0
FORWARD_TEST = "forward_5cm"
ROTATE_TEST = "rotate_180"
SESSION_PROTOCOL_VERSION = 2
SUPPORTED_TEST_KINDS = (FORWARD_TEST, ROTATE_TEST)
ROTATE_ACTION = "rotate_180"
DRIVE_DOMAIN_ID = "24"
FOOTPRINT_M = 0.18
VICINITY_M = 0.25
WAYPOINTS = ("P_START", "P_TEST")
MAP_NAME = "physical_odom_trial"
MERGE_WAYPOINT_M = 0.005
MERGE_LANE_M = 0.01
PHYSICAL_ROBOT_IDS = ("real_pinky1", "real_pinky2", "real_pinky3")
_ROBOT_SLOTS = (
    {"id": "real_pinky1", "label": "Pinky #1", "namespace": ""},
    {"id": "real_pinky2", "label": "Pinky #2", "namespace": "pinky2"},
    {"id": "real_pinky3", "label": "Pinky #3", "namespace": "pinky3"},
)


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def ang_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def hypot(x: float, y: float) -> float:
    return math.hypot(x, y)


_UNRECOVERABLE_GOALS = {
    "robot is not in PHYSICAL_TEST",
    "no prepared test session",
    "goal robot does not match the prepared Pinky",
    "destination is outside the measured forward test",
    "rmf coordinate does not match the measured odom target",
    "rotation goal is not the locked turn",
    "no fresh odometry target",
    "target is not the 0.05 m test",
    "target is not along the current heading",
    "target is not a 180 degree turn",
}


def unrecoverable_goal(reason: str, duplicate: bool = False) -> bool:
    """A refused goal that can never start. An active or duplicate goal is left alone."""
    if duplicate or reason in {"a physical goal is already active", "duplicate physical goal"}:
        return False
    return reason in _UNRECOVERABLE_GOALS


def gateway_protocol_block(status: dict) -> str | None:
    """None when this process loaded the session protocol the API expects."""
    if status.get("session_protocol_version") != SESSION_PROTOCOL_VERSION:
        return "실행 중인 Gateway의 세션 프로토콜이 이 서버와 맞지 않습니다."
    kinds = status.get("supported_test_kinds")
    if not isinstance(kinds, list) or not set(SUPPORTED_TEST_KINDS).issubset(set(kinds)):
        return "Gateway가 이 서버의 5cm·180도 시험을 지원하지 않습니다."
    return None


def button_block(status: dict) -> str | None:
    """Why a physical test button must stay disabled. None means the selected robot is ready."""
    if not status.get("reachable"):
        return "Physical Gateway가 연결되지 않았습니다."
    if "domain" in status and status.get("domain") not in (None, "", DRIVE_DOMAIN_ID, 24):
        return "주행 도메인이 기존 Pinky ROS domain 24가 아닙니다."
    mode = status.get("mode")
    if mode in {"FAULT"}:
        return "Gateway가 FAULT입니다. 정지가 확인되지 않았습니다."
    if mode in {"TRANSITIONING", "STOPPING"}:
        return "모드를 전환하는 중입니다."
    if mode not in {"PHYSICAL_TEST", "PHYSICAL_DRY_RUN", "DRY_RUN"}:
        return "Gateway 모드를 확인하지 못했습니다."
    if status.get("selected_available") is False:
        return "선택한 Pinky가 이 ROS domain에 없습니다."
    if status.get("motor_ready") is not True:
        return "Motor가 ready가 아닙니다."
    if not status.get("odom_fresh") or not status.get("pose"):
        return "Odometry가 fresh가 아닙니다."
    if abs(float(status.get("odom_velocity") or 0.0)) > STOP_SPEED_MPS:
        return "로봇이 정지하지 않았습니다."
    if abs(float(status.get("odom_angular") or 0.0)) > STOP_ANGULAR_RPS:
        return "로봇이 회전 중입니다."
    if status.get("goal_phase") in {"moving", "stopping"}:
        return "진행 중인 Goal이 있습니다."
    if status.get("session_open") and not status.get("session_submitted"):
        return "이미 준비된 테스트 session이 있어 다시 제출하지 않습니다."
    return None


def _slot_topics(slot: dict, odom_publishers: dict[str, int]) -> tuple[str, str]:
    """Pinky #1 keeps the un-namespaced bringup topics when that publisher is present."""
    if slot["id"] == "real_pinky1":
        if int(odom_publishers.get("/odom") or 0) >= 1:
            return "/odom", "/cmd_vel"
        if int(odom_publishers.get("/pinky1/odom") or 0) >= 1:
            return "/pinky1/odom", "/pinky1/cmd_vel"
        return "/odom", "/cmd_vel"
    odom = f"/{slot['namespace']}/odom"
    return odom, f"/{slot['namespace']}/cmd_vel"


def describe_robots(odom_publishers: dict[str, int], cmd_subscribers: dict[str, int],
                    samples: dict[str, dict] | None = None) -> list[dict]:
    """Pinkies visible on the one existing drive domain. Namespaces distinguish them."""
    samples = samples or {}
    robots = []
    for slot in _ROBOT_SLOTS:
        odom_topic, cmd_topic = _slot_topics(slot, odom_publishers)
        available = int(odom_publishers.get(odom_topic) or 0) >= 1
        sample = samples.get(odom_topic) or {}
        robots.append({
            "id": slot["id"],
            "label": slot["label"],
            "namespace": slot["namespace"],
            "odom_topic": odom_topic,
            "cmd_vel_topic": cmd_topic,
            "available": available,
            "motor_ready": available and int(cmd_subscribers.get(cmd_topic) or 0) >= 1,
            "odom_fresh": bool(available and sample.get("fresh")),
            "pose": list(sample["pose"]) if available and sample.get("fresh") and sample.get("pose") else None,
            "speed": float(sample.get("speed") or 0.0) if available and sample.get("fresh") else 0.0,
            "angular": float(sample.get("angular") or 0.0) if available and sample.get("fresh") else 0.0,
        })
    return robots


class RobotSelection:
    """One selected Pinky. A test never starts on a robot that is not visible."""

    def __init__(self) -> None:
        self.robot_id = "real_pinky1"

    def choose(self, robot_id: str, robots: list[dict], phase: str, session_open: bool,
               session_submitted: bool) -> dict:
        if robot_id not in PHYSICAL_ROBOT_IDS:
            return {"ok": False, "reason": "알 수 없는 Pinky입니다.", "robot_id": self.robot_id}
        visible = {item["id"]: item for item in robots}
        if not visible.get(robot_id, {}).get("available"):
            return {"ok": False, "reason": "그 Pinky는 이 ROS domain에서 보이지 않습니다.", "robot_id": self.robot_id}
        if phase in {"moving", "stopping"}:
            return {"ok": False, "reason": "주행 중에는 Pinky를 바꾸지 않습니다.", "robot_id": self.robot_id}
        if session_open and not session_submitted:
            return {"ok": False, "reason": "준비된 테스트가 있어 Pinky를 바꾸지 않습니다.", "robot_id": self.robot_id}
        self.robot_id = robot_id
        return {"ok": True, "robot_id": self.robot_id}


def lane_is_navigable(distance: float) -> str | None:
    """None when a fleet adapter can treat the lane as a real move."""
    if distance <= MERGE_WAYPOINT_M:
        return "distance is inside the waypoint merge radius"
    if distance <= MERGE_LANE_M:
        return "distance is inside the lane merge radius"
    return None


def may_finish(goal: dict, pose: list[float] | None, fresh: bool, speed: float, angular: float = 0.0) -> bool:
    """True only after the locked test is reached and several fresh stops agree."""
    if goal.get("phase") != "succeeded" or goal.get("canceled") or not fresh or not pose or len(pose) < 2:
        return False
    if int(goal.get("stopped_samples") or 0) < STOP_SAMPLES:
        return False
    if abs(speed) > STOP_SPEED_MPS:
        return False
    if goal.get("kind") == ROTATE_TEST:
        if len(pose) < 3 or abs(angular) > STOP_ANGULAR_RPS:
            return False
        if float(goal.get("translation") or 0.0) > MAX_SPIN_TRANSLATION_M:
            return False
        if float(goal.get("turned") or 0.0) < MIN_YAW_RAD:
            return False
        target = goal.get("target") or [None, None, None]
        if target[2] is None:
            return False
        return abs(ang_diff(float(pose[2]), float(target[2]))) <= YAW_TOLERANCE_RAD
    if float(goal.get("traveled") or 0.0) < MIN_TRAVEL_M:
        return False
    if float(goal.get("lateral") or 0.0) > MAX_LATERAL_M:
        return False
    target = goal.get("target") or [None, None]
    if target[0] is None:
        return False
    return hypot(float(pose[0]) - float(target[0]), float(pose[1]) - float(target[1])) <= GOAL_TOLERANCE_M


def track_error(start: list[float] | tuple, target: list[float], pose: list[float]) -> dict:
    dx = float(target[0]) - float(start[0])
    dy = float(target[1]) - float(start[1])
    length = hypot(dx, dy) or 1.0
    px = float(pose[0]) - float(start[0])
    py = float(pose[1]) - float(start[1])
    return {
        "traveled": hypot(px, py),
        "remaining": hypot(float(pose[0]) - float(target[0]), float(pose[1]) - float(target[1])),
        "lateral": abs(px * dy - py * dx) / length,
        "along": (px * dx + py * dy) / length,
    }


def trial_graph(segment: dict) -> tuple[str, str]:
    """Fleet and nav YAML for one measured test on the selected Pinky."""
    kind = str(segment.get("kind") or FORWARD_TEST)
    robot_id = str(segment.get("robot_id") or "real_pinky1")
    if robot_id not in PHYSICAL_ROBOT_IDS:
        raise ValueError("unknown physical robot")
    start = segment["P_START"]
    actions = ""
    angular_limits = "[0.2, 0.3]"
    if kind == FORWARD_TEST:
        reason = lane_is_navigable(float(segment["distance_m"]))
        if reason:
            raise ValueError(reason)
        test = segment["P_TEST"]
        nav_body = (
            "    vertices:\n"
            f"      - [{start[0]}, {start[1]}, {{name: P_START, is_charger: true, is_parking_spot: true}}]\n"
            f"      - [{test[0]}, {test[1]}, {{name: P_TEST}}]\n"
            "    lanes:\n"
            "      - [0, 1, {speed_limit: 0.04}]\n"
            "      - [1, 0, {speed_limit: 0.04}]\n"
        )
    elif kind == ROTATE_TEST:
        actions = '\n  actions: ["rotate_180"]'
        angular_limits = "[0.3, 0.3]"
        nav_body = (
            "    vertices:\n"
            f"      - [{start[0]}, {start[1]}, {{name: P_START, is_charger: true, is_parking_spot: true}}]\n"
            "    lanes: []\n"
        )
    else:
        raise ValueError("unknown physical test")
    fleet = f"""rmf_fleet:
  name: physical_pinky
  limits:
    linear: [0.04, 0.05]
    angular: {angular_limits}
  profile:
    footprint: {FOOTPRINT_M}
    vicinity: {VICINITY_M}
  reversible: true
  battery_system:
    voltage: 12.0
    capacity: 10.0
    charging_current: 2.0
  mechanical_system:
    mass: 5.0
    moment_of_inertia: 0.2
    friction_coefficient: 0.3
  ambient_system:
    power: 10.0
  tool_system:
    power: 0.0
  recharge_threshold: 0.1
  recharge_soc: 1.0
  publish_fleet_state: 2.0
  account_for_battery_drain: false
  retreat_to_charger_interval: 600.0
  max_delay: 15.0
  finishing_request: nothing
  task_capabilities:
    loop: true
    delivery: false{actions}
  robots:
    {robot_id}:
      charger: P_START
      responsive_wait: false
"""
    nav = f"levels:\n  {MAP_NAME}:\n{nav_body}"
    return fleet, nav


def measured_segment(x: float, y: float, yaw: float, distance: float = TARGET_DISTANCE_M) -> dict | None:
    """Two points in the live odom frame. None unless the distance is the 5cm trial."""
    if abs(distance - TARGET_DISTANCE_M) > 1e-6:
        return None
    return {
        "frame": "odom",
        "kind": FORWARD_TEST,
        "P_START": [x, y, yaw],
        "P_TEST": [x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw],
        "distance_m": distance,
    }


def rotation_segment(x: float, y: float, yaw: float, robot_id: str) -> dict:
    """In-place CCW half turn. The graph point does not move."""
    target_yaw = math.atan2(math.sin(yaw + TARGET_YAW_RAD), math.cos(yaw + TARGET_YAW_RAD))
    return {
        "frame": "odom",
        "kind": ROTATE_TEST,
        "robot_id": robot_id,
        "P_START": [x, y, yaw],
        "P_TURN": [x, y, target_yaw],
        "yaw_delta_rad": TARGET_YAW_RAD,
        "distance_m": 0.0,
    }


class TrialSession:
    """One forward or rotation test. A second submit of the same session is refused."""

    def __init__(self) -> None:
        self.session_id = ""
        self.segment = None
        self.submitted = False
        self.goal_generation = None
        self.open = False

    def prepare(self, x: float, y: float, yaw: float, phase: str, fresh: bool, speed: float,
                kind: str = FORWARD_TEST, robot_id: str = "real_pinky1", angular: float = 0.0) -> dict:
        if phase in {"moving", "stopping"}:
            return {"ok": False, "reason": "a physical goal is already active"}
        if self.open and not self.submitted:
            return {"ok": False, "reason": "a test session is already prepared"}
        if self.open and self.submitted and phase not in {"succeeded", "failed", "canceled"}:
            return {"ok": False, "reason": "the current test session was already submitted"}
        if robot_id not in PHYSICAL_ROBOT_IDS:
            return {"ok": False, "reason": "unknown physical robot"}
        if kind not in {FORWARD_TEST, ROTATE_TEST}:
            return {"ok": False, "reason": "unknown physical test"}
        if not fresh:
            return {"ok": False, "reason": "odometry is not fresh"}
        if abs(speed) > STOP_SPEED_MPS or abs(angular) > STOP_ANGULAR_RPS:
            return {"ok": False, "reason": "the robot is still moving"}
        if kind == ROTATE_TEST:
            segment = rotation_segment(x, y, yaw, robot_id)
        else:
            segment = measured_segment(x, y, yaw, TARGET_DISTANCE_M)
            if segment is None:
                return {"ok": False, "reason": "5cm segment was rejected"}
            segment["robot_id"] = robot_id
            nav_reason = lane_is_navigable(float(segment["distance_m"]))
            if nav_reason:
                return {"ok": False, "reason": nav_reason}
        self.session_id = uuid.uuid4().hex
        self.segment = segment
        self.submitted = False
        self.goal_generation = None
        self.open = True
        return {"ok": True, "session_id": self.session_id, "segment": segment, "robot_id": robot_id, "kind": kind}

    def mark_submitted(self, generation: int | None = None) -> None:
        self.submitted = True
        self.goal_generation = generation

    def abandon(self) -> None:
        self.open = False
        self.submitted = False
        self.goal_generation = None


class MoveController:
    def __init__(self) -> None:
        self.phase = "idle"
        self.generation = 0
        self.correlation = ""
        self.start = None
        self.target = None
        self.motion_commands = 0
        self.fail_reason = ""
        self.stopped_samples = 0
        self.lateral = 0.0
        self.traveled = 0.0
        self.remaining = None
        self.started_at = 0.0
        self.kind = ""
        self.turned = 0.0
        self.translation = 0.0
        self._last_yaw: float | None = None
        self._seen: set[str] = set()

    def snapshot(self) -> dict:
        return {
            "phase": self.phase,
            "generation": self.generation,
            "correlation_id": self.correlation,
            "motion_commands": self.motion_commands,
            "fail_reason": self.fail_reason,
            "target": self.target,
            "initial_pose": list(self.start) if self.start else None,
            "target_pose": self.target,
            "traveled": self.traveled,
            "remaining": self.remaining,
            "lateral": self.lateral,
            "stopped_samples": self.stopped_samples,
            "canceled": self.phase == "canceled",
            "kind": self.kind,
            "turned": self.turned,
            "translation": self.translation,
        }

    def go(self, pose: dict, target: list[float], correlation: str) -> dict:
        if self.phase in {"moving", "stopping"}:
            return self._refuse("a physical goal is already active", duplicate=bool(correlation and correlation in self._seen))
        if correlation and correlation in self._seen:
            return self._refuse("duplicate physical goal", duplicate=True)
        if not pose.get("fresh"):
            return self._refuse("odometry is not fresh")
        if len(target) < 2:
            return self._refuse("target is incomplete")
        here = (float(pose["x"]), float(pose["y"]), float(pose["yaw"]))
        goal = (float(target[0]), float(target[1]))
        distance = hypot(goal[0] - here[0], goal[1] - here[1])
        if abs(distance - TARGET_DISTANCE_M) > 0.005:
            return self._refuse("target is not the 0.05 m test")
        bearing = math.atan2(goal[1] - here[1], goal[0] - here[0])
        if abs(ang_diff(bearing, here[2])) > MAX_HEADING_ERROR_RAD:
            return self._refuse("target is not along the current heading")
        self.generation += 1
        self.phase = "moving"
        self.kind = FORWARD_TEST
        self.correlation = correlation
        self.start = here
        self.target = [goal[0], goal[1], float(target[2]) if len(target) > 2 else here[2]]
        self.turned = 0.0
        self.translation = 0.0
        self._last_yaw = here[2]
        self.fail_reason = ""
        self.stopped_samples = 0
        self.traveled = 0.0
        self.remaining = distance
        self.lateral = 0.0
        self.started_at = time.monotonic()
        if correlation:
            self._seen.add(correlation)
        return {"ok": True, "accepted": True, "generation": self.generation, "motion_commands": self.motion_commands}

    def go_rotate(self, pose: dict, target_yaw: float, correlation: str) -> dict:
        if self.phase in {"moving", "stopping"}:
            return self._refuse("a physical goal is already active", duplicate=bool(correlation and correlation in self._seen))
        if correlation and correlation in self._seen:
            return self._refuse("duplicate physical goal", duplicate=True)
        if not pose.get("fresh"):
            return self._refuse("odometry is not fresh")
        here = (float(pose["x"]), float(pose["y"]), float(pose["yaw"]))
        expected = math.atan2(math.sin(here[2] + TARGET_YAW_RAD), math.cos(here[2] + TARGET_YAW_RAD))
        if abs(ang_diff(float(target_yaw), expected)) > 0.05:
            return self._refuse("target is not a 180 degree turn")
        self.generation += 1
        self.phase = "moving"
        self.kind = ROTATE_TEST
        self.correlation = correlation
        self.start = here
        self.target = [here[0], here[1], float(target_yaw)]
        self.fail_reason = ""
        self.stopped_samples = 0
        self.traveled = 0.0
        self.remaining = TARGET_YAW_RAD
        self.lateral = 0.0
        self.turned = 0.0
        self.translation = 0.0
        self._last_yaw = here[2]
        self.started_at = time.monotonic()
        if correlation:
            self._seen.add(correlation)
        return {"ok": True, "accepted": True, "generation": self.generation, "motion_commands": self.motion_commands}

    def tick(self, pose: dict) -> tuple[float, float]:
        if self.phase not in {"moving", "stopping"}:
            return 0.0, 0.0
        if self.kind == ROTATE_TEST:
            return self._tick_rotate(pose)
        if not pose.get("fresh"):
            self.phase = "failed"
            self.fail_reason = "odometry stopped during the move"
            return 0.0, 0.0
        start = self.start or (pose["x"], pose["y"], pose["yaw"])
        target = self.target or [pose["x"], pose["y"], pose["yaw"]]
        measured = track_error(start, target, [pose["x"], pose["y"], pose["yaw"]])
        self.traveled = measured["traveled"]
        self.remaining = measured["remaining"]
        self.lateral = measured["lateral"]
        error = measured["remaining"]
        speed = abs(float(pose.get("speed") or 0.0))
        yaw_error = abs(ang_diff(float(pose["yaw"]), float(start[2])))
        if self.lateral > MAX_LATERAL_M:
            self.fail_reason = "lateral error left the forward test"
        elif yaw_error > MAX_HEADING_ERROR_RAD:
            self.fail_reason = "heading changed during the forward test"
        elif self.traveled > MAX_DISTANCE_M:
            self.fail_reason = "travel exceeded 0.06 m"
        elif time.monotonic() - self.started_at > MOVE_TIMEOUT_SEC:
            self.fail_reason = "movement timed out"
        if self.phase == "moving" and not self.fail_reason:
            if error <= GOAL_TOLERANCE_M and self.traveled >= MIN_TRAVEL_M:
                self.phase = "stopping"
                return 0.0, 0.0
            self.motion_commands += 1
            return min(MAX_LINEAR_MPS, max(error, 0.0)), 0.0
        self.phase = "stopping"
        if speed <= STOP_SPEED_MPS:
            self.stopped_samples += 1
        else:
            self.stopped_samples = 0
            return 0.0, 0.0
        if self.stopped_samples < STOP_SAMPLES:
            return 0.0, 0.0
        if not self.fail_reason and error <= GOAL_TOLERANCE_M and self.traveled >= MIN_TRAVEL_M:
            self.phase = "succeeded"
            self.fail_reason = ""
        else:
            self.phase = "failed"
            if not self.fail_reason:
                self.fail_reason = "stopped before the measured target"
        return 0.0, 0.0

    def _tick_rotate(self, pose: dict) -> tuple[float, float]:
        if not pose.get("fresh"):
            self.phase = "failed"
            self.fail_reason = "odometry stopped during the move"
            return 0.0, 0.0
        start = self.start or (pose["x"], pose["y"], pose["yaw"])
        yaw = float(pose["yaw"])
        if self._last_yaw is None:
            self._last_yaw = float(start[2])
        self.turned += ang_diff(yaw, self._last_yaw)
        self._last_yaw = yaw
        self.translation = hypot(float(pose["x"]) - float(start[0]), float(pose["y"]) - float(start[1]))
        self.traveled = self.translation
        remaining = TARGET_YAW_RAD - self.turned
        self.remaining = remaining
        speed = abs(float(pose.get("speed") or 0.0))
        angular = abs(float(pose.get("angular") or 0.0))
        if self.translation > MAX_SPIN_TRANSLATION_M:
            self.fail_reason = "rotation drifted more than 0.03 m"
        elif self.turned < -0.15:
            self.fail_reason = "rotation reversed"
        elif self.turned > MAX_YAW_RAD:
            self.fail_reason = "rotation exceeded 180 degrees"
        elif time.monotonic() - self.started_at > SPIN_TIMEOUT_SEC:
            self.fail_reason = "rotation timed out"
        if self.phase == "moving" and not self.fail_reason:
            if abs(remaining) <= YAW_TOLERANCE_RAD and self.turned >= MIN_YAW_RAD:
                self.phase = "stopping"
                return 0.0, 0.0
            self.motion_commands += 1
            return 0.0, min(MAX_ANGULAR_RPS, max(remaining, 0.0))
        self.phase = "stopping"
        if speed <= STOP_SPEED_MPS and angular <= STOP_ANGULAR_RPS:
            self.stopped_samples += 1
        else:
            self.stopped_samples = 0
            return 0.0, 0.0
        if self.stopped_samples < STOP_SAMPLES:
            return 0.0, 0.0
        if (not self.fail_reason and abs(remaining) <= YAW_TOLERANCE_RAD and self.turned >= MIN_YAW_RAD
                and self.translation <= MAX_SPIN_TRANSLATION_M):
            self.phase = "succeeded"
            self.fail_reason = ""
        else:
            self.phase = "failed"
            if not self.fail_reason:
                self.fail_reason = "stopped before 180 degrees"
        return 0.0, 0.0

    def cancel(self) -> dict:
        if self.phase in {"moving", "stopping"}:
            self.phase = "canceled"
            self.fail_reason = "cancel requested"
        return {"ok": True, "phase": self.phase, "motion_commands": self.motion_commands}

    def _refuse(self, reason: str, duplicate: bool = False) -> dict:
        return {
            "ok": False,
            "refused": True,
            "duplicate": duplicate,
            "reason": reason,
            "motion_commands": self.motion_commands,
            "generation": self.generation,
        }
