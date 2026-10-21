"""Hardware driver launch gate.

The Pinky driver is included only after the domain check passes. The shipped
profile leaves the hardware domain unset, so this file refuses to start the
motors. It never publishes a goal.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robot_profile.preflight import hardware_launch_block  # noqa: E402
from robot_profile.profiles import HARDWARE_FORBIDDEN_DOMAINS, load_robot  # noqa: E402


def generate_launch_description() -> LaunchDescription:
    raw_domain = os.environ.get("ROS_DOMAIN_ID")
    if raw_domain in {None, ""} or int(raw_domain) in HARDWARE_FORBIDDEN_DOMAINS:
        raise RuntimeError("refusing hardware bringup on a RMF, Gazebo, or alias ROS domain")
    profile = load_robot(ROOT, "pinky1", "hardware")
    reason = hardware_launch_block(profile)
    if reason:
        raise RuntimeError(reason)
    if int(raw_domain) != profile.backend.ros_domain_id:
        raise RuntimeError("ROS_DOMAIN_ID does not match the hardware profile")
    pinky_share = os.environ.get("PINKY_BRINGUP_SHARE")
    if not pinky_share:
        raise RuntimeError("PINKY_BRINGUP_SHARE is unset; not guessing a driver path")
    launch_file = str(Path(pinky_share) / "launch" / "bringup_robot.launch.xml")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pinky1"),
        GroupAction([
            PushRosNamespace(LaunchConfiguration("namespace")),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(launch_file),
                launch_arguments={"use_sim_time": "False"}.items(),
            ),
        ]),
    ])
