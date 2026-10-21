"""Print hardware readiness. Does not publish cmd_vel or start a driver."""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_profile.preflight import read_only_gap, static_checks  # noqa: E402
from robot_profile.profiles import ProfileError, load_robot  # noqa: E402
from robot_profile.safety import goal_block_reason  # noqa: E402


def main() -> int:
    robot_id = os.environ.get("ROBOT_ID", "pinky1")
    backend = os.environ.get("ROBOT_BACKEND", "hardware")
    mode = os.environ.get("ROBOT_MODE", "navigation")
    try:
        profile = load_robot(ROOT, robot_id, backend)
    except ProfileError as exc:
        print(json.dumps({"ok": False, "reason": str(exc)}, ensure_ascii=False))
        return 2
    checks = static_checks(ROOT, profile, mode)
    blocked = goal_block_reason(
        profile,
        graph_alignment="placeholder",
        allow_motion=False,
        simulation_confirmed=False,
        action_available=True,
        pose_fresh=True,
        motion_approved=os.environ.get("CARE_MOVE_HARDWARE_MOTION_APPROVAL") == "yes",
    )
    report = {
        "robot_id": profile.robot_id,
        "backend": profile.backend.name,
        "namespace": profile.namespace,
        "cmd_vel": profile.cmd_vel,
        "odom": profile.odom,
        "scan": profile.scan,
        "navigate_to_pose": profile.navigate_to_pose,
        "odom_frame": profile.backend.odom_frame,
        "base_frame": profile.backend.base_frame,
        "laser_frame": profile.backend.laser_frame,
        "ros_domain_id": profile.backend.ros_domain_id,
        "goal_blocked": blocked,
        "checks": [{"name": item.name, "ok": item.ok, "detail": item.detail} for item in checks],
        "unread_without_robot": read_only_gap(profile),
        "motion_sent": False,
    }
    print(json.dumps(report, ensure_ascii=False))
    if blocked is None and backend == "hardware":
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
