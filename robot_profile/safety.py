"""Goal gates. Hardware motion stays blocked. The fleet graph gate is unchanged."""
from __future__ import annotations

from fleet_adapter.safety import navigation_block_reason
from robot_profile.profiles import RobotProfile


def hardware_goal_block_reason(profile: RobotProfile, *, motion_approved: bool,
                               estop_confirmed: bool, scan_ok: bool, odom_ok: bool,
                               localization_ok: bool) -> str | None:
    if profile.backend.name != "hardware":
        return None
    if motion_approved or profile.backend.hardware_motion_approved:
        return "hardware motion approval is not accepted from this process"
    if not estop_confirmed:
        return "an independent motor stop has not been confirmed"
    if not scan_ok or not odom_ok:
        return "lidar or odometry was not observed"
    if not localization_ok:
        return "localization is not ready"
    return "hardware goals stay blocked until a separate approved test"


def goal_block_reason(profile: RobotProfile, *, graph_alignment: str, allow_motion: bool,
                      simulation_confirmed: bool, action_available: bool, pose_fresh: bool,
                      motion_approved: bool = False, estop_confirmed: bool = False,
                      scan_ok: bool = False, odom_ok: bool = False,
                      localization_ok: bool = False) -> str | None:
    fleet_reason = navigation_block_reason(
        graph_alignment=graph_alignment,
        allow_motion=allow_motion,
        simulation_confirmed=simulation_confirmed,
        action_available=action_available,
        pose_fresh=pose_fresh,
    )
    if fleet_reason:
        return fleet_reason
    return hardware_goal_block_reason(
        profile,
        motion_approved=motion_approved,
        estop_confirmed=estop_confirmed,
        scan_ok=scan_ok,
        odom_ok=odom_ok,
        localization_ok=localization_ok,
    )


def observe_robot_state(*, connected: bool, navigating: bool, arrived: bool,
                        navigation_failed: bool, localization_ok: bool,
                        estop: bool, battery_low: bool) -> str:
    """Map observed facts to one state. Missing facts stay disconnected."""
    if not connected:
        return "disconnected"
    if estop:
        return "emergency_stop"
    if battery_low:
        return "battery_low"
    if not localization_ok:
        return "localization_lost"
    if navigation_failed:
        return "navigation_failed"
    if navigating:
        return "navigating"
    if arrived:
        return "arrived"
    return "idle"
