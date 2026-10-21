"""Static checks for a robot profile. This module does not publish cmd_vel."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from robot_profile.maps import MapError, require_navigation_map
from robot_profile.modes import assert_mode
from robot_profile.profiles import CMD_VEL_TYPE, HARDWARE_FORBIDDEN_DOMAINS, RobotProfile


@dataclass(frozen=True)
class Check:
    name: str
    ok: bool
    detail: str


def static_checks(root, profile: RobotProfile, mode: str) -> list[Check]:
    assert_mode(mode)
    backend = profile.backend
    checks = [
        Check("robot_id", profile.robot_id.startswith("pinky"), profile.robot_id),
        Check("namespace", profile.namespace == profile.robot_id, profile.namespace),
        Check("cmd_vel_type", backend.cmd_vel_type == CMD_VEL_TYPE, backend.cmd_vel_type),
        Check("navigate_action", profile.navigate_to_pose.endswith("/navigate_to_pose"), profile.navigate_to_pose),
        Check("bringup_launch", bool(backend.bringup_launch), backend.bringup_launch),
        Check("nav_launch", bool(backend.nav_launch), backend.nav_launch),
    ]
    if backend.name == "gazebo":
        checks.append(Check(
            "ros_domain_id",
            backend.ros_domain_id == 42,
            str(backend.ros_domain_id),
        ))
        checks.append(Check("use_sim_time", backend.use_sim_time, str(backend.use_sim_time)))
        checks.append(Check(
            "frames",
            backend.base_frame == f"{profile.namespace}/base_footprint",
            backend.base_frame,
        ))
    else:
        domain_ok = backend.ros_domain_id is not None and backend.ros_domain_id not in HARDWARE_FORBIDDEN_DOMAINS
        checks.append(Check(
            "ros_domain_id",
            domain_ok,
            "unset" if backend.ros_domain_id is None else str(backend.ros_domain_id),
        ))
        checks.append(Check("use_sim_time", not backend.use_sim_time, str(backend.use_sim_time)))
        checks.append(Check("frames", backend.base_frame == "base_footprint", backend.base_frame))
        checks.append(Check(
            "serial_ports",
            backend.lidar_serial_port == "/dev/ttyAMA0" and backend.motor_serial_port == "/dev/ttyAMA4",
            f"{backend.lidar_serial_port} {backend.motor_serial_port}",
        ))
        lidar_node = Path(backend.lidar_serial_port or "")
        motor_node = Path(backend.motor_serial_port or "")
        present = lidar_node.exists() and motor_node.exists()
        checks.append(Check(
            "serial_device_present",
            present,
            "present" if present else "serial device nodes are absent on this host",
        ))
        checks.append(Check(
            "hardware_motion",
            not backend.hardware_motion_approved,
            "blocked",
        ))
    if mode == "navigation":
        try:
            record = require_navigation_map(root, backend.map_id)
            checks.append(Check("navigation_map", True, record.map_id))
        except MapError as exc:
            checks.append(Check("navigation_map", False, str(exc)))
    elif mode == "mapping":
        checks.append(Check(
            "mapping_launch",
            bool(backend.mapping_launch),
            backend.mapping_launch or "hardware mapping launch is not selected",
        ))
        checks.append(Check(
            "master_overwrite_blocked",
            True,
            "mapping cannot replace a catalog master",
        ))
    else:
        checks.append(Check("map_update", True, "comparison only; promotion is manual"))
    return checks


def hardware_launch_block(profile: RobotProfile) -> str | None:
    if profile.backend.name != "hardware":
        return "this launch is only for the hardware backend"
    if profile.backend.ros_domain_id is None:
        return "hardware ROS_DOMAIN_ID is unset; refusing to start the driver"
    if profile.backend.ros_domain_id in HARDWARE_FORBIDDEN_DOMAINS:
        return "hardware domain collides with RMF or Gazebo"
    if profile.backend.hardware_motion_approved:
        return "refusing a profile that marks hardware motion approved"
    return None


def read_only_gap(profile: RobotProfile) -> list[str]:
    """Facts that cannot be checked without the physical robot."""
    if profile.backend.name != "hardware":
        return []
    return [
        "no live /cmd_vel subscription check",
        "no live lidar sample",
        "no live odometry sample",
        "no live battery sample",
        "no physical e-stop test",
    ]
