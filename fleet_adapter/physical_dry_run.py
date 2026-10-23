"""Read-only decisions for the physical Pinky. This module publishes nothing."""
from __future__ import annotations

DRY_RUN = "PHYSICAL_DRY_RUN"
ARMED = "PHYSICAL_ARMED"
MOTION_OPS = {"go", "cancel", "cmd_vel", "torque", "navigate"}


def classify_joints(names: list[str] | None, positions: list[float] | None) -> str:
    """Separate the URDF joint publisher from the encoder publisher. Neither is motion."""
    if not names:
        return "absent"
    if names[:2] == ["left_wheel_joint", "right_wheel_joint"]:
        return "bringup_joint_names"
    if "l_wheel_joint" in names and "r_wheel_joint" in names:
        values = positions or []
        if values and all(abs(float(item)) < 1e-9 for item in values):
            return "joint_state_publisher_zeros"
        return "urdf_joint_names"
    return "unknown"


def assess(observation: dict) -> dict:
    """Decide whether RMF may register this robot. Never invents a pose."""
    reasons: list[str] = []
    if int(observation.get("domain") or -1) != 24:
        reasons.append("physical observation must stay on ROS domain 24")
    if observation.get("rmw") != "rmw_fastrtps_cpp":
        reasons.append("physical RMW is not rmw_fastrtps_cpp")
    scan_ok = bool(observation.get("scan_fresh")) and observation.get("scan_frame") == "rplidar_link"
    if not scan_ok:
        reasons.append("lidar scan is missing or stale")
    joint_source = classify_joints(observation.get("joint_names"), observation.get("joint_positions"))
    if joint_source != "bringup_joint_names":
        reasons.append("wheel joints are not the encoder publisher")
    if not observation.get("odom_topic"):
        reasons.append("hardware odometry topic is absent")
    if not observation.get("nav2_action"):
        reasons.append("navigate_to_pose action server is absent")
    if not observation.get("map_topic"):
        reasons.append("no physical map topic")
    if not observation.get("localization_valid"):
        reasons.append("localization is not valid")
    if observation.get("cmd_vel_publishers"):
        reasons.append("a velocity publisher is already present")
    mode = observation.get("mode") or DRY_RUN
    if mode != DRY_RUN:
        reasons.append("only PHYSICAL_DRY_RUN may run without a separate motion approval")
    return {
        "mode": DRY_RUN,
        "fleet_name": "physical_pinky",
        "robot_id": "real_pinky1",
        "register_with_rmf": False,
        "motion_allowed": False,
        "pose": None,
        "joint_source": joint_source,
        "reasons": reasons,
    }


def decide_registration(report: dict) -> dict:
    """Use the gateway decision. This does not contact RMF."""
    decision = report.get("decision") if isinstance(report.get("decision"), dict) else {}
    return {
        "started": True,
        "registered": False,
        "register_with_rmf": False,
        "motion_commands": report.get("motion_commands", 0),
        "reasons": decision.get("reasons"),
        "robot_id": decision.get("robot_id") or "real_pinky1",
        "fleet_name": "physical_pinky",
        "note": "dry-run does not call Adapter.make and does not copy a Gazebo pose",
    }


class DryRunGateway:
    """Command handler whose motion counter stays at zero."""

    def __init__(self, observation: dict | None = None):
        self.observation = observation or {}
        self.motion_commands = 0
        self.refused = 0
        self._seen: set[str] = set()

    def handle(self, payload: dict) -> dict:
        op = str(payload.get("op") or "")
        correlation = str(payload.get("correlation_id") or "")
        if op in MOTION_OPS:
            self.refused += 1
            duplicate = bool(correlation and correlation in self._seen)
            if correlation:
                self._seen.add(correlation)
            return {
                "ok": False,
                "refused": True,
                "duplicate": duplicate,
                "motion_commands": self.motion_commands,
                "reason": "PHYSICAL_DRY_RUN blocks motor, cmd_vel, and Nav2 goals",
            }
        if op not in {"health", "poll"}:
            return {"ok": False, "motion_commands": self.motion_commands, "reason": "unknown op"}
        decision = assess(self.observation)
        return {
            "ok": True,
            "mode": DRY_RUN,
            "robot_id": "real_pinky1",
            "reachable": True,
            "connected": True,
            "motor_ready": int(self.observation.get("cmd_vel_subscribers") or 0) >= 1,
            "odom_fresh": bool(self.observation.get("odom_fresh")),
            "odom_velocity": self.observation.get("odom_speed"),
            "pose": self.observation.get("odom_pose") if self.observation.get("odom_fresh") else None,
            "pose_source": "odom" if self.observation.get("odom_fresh") else None,
            "goal_phase": "idle",
            "motion_commands": self.motion_commands,
            "refused_ops": self.refused,
            "observation": {
                "domain": self.observation.get("domain"),
                "scan_fresh": self.observation.get("scan_fresh"),
                "scan_frame": self.observation.get("scan_frame"),
                "scan_points": self.observation.get("scan_points"),
                "joint_source": decision["joint_source"],
                "odom_topic": bool(self.observation.get("odom_topic")),
                "odom_fresh": bool(self.observation.get("odom_fresh")),
                "odom_pose": self.observation.get("odom_pose"),
                "odom_speed": self.observation.get("odom_speed"),
                "nav2_action": bool(self.observation.get("nav2_action")),
                "map_topic": bool(self.observation.get("map_topic")),
                "cmd_vel_publishers": self.observation.get("cmd_vel_publishers") or 0,
                "cmd_vel_subscribers": self.observation.get("cmd_vel_subscribers") or 0,
            },
            "decision": decision,
        }
