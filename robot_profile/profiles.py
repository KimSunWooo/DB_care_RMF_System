"""Load gazebo and hardware profiles. Does not start ROS or a robot."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

BACKENDS = {"gazebo", "hardware"}
ROBOT_IDS = ("pinky1", "pinky2", "pinky3")
CMD_VEL_TYPE = "geometry_msgs/msg/Twist"
# 0 is the running RMF domain. 42 is the Gazebo exercise domain.
# 24 and 25 are the local Pinky shell aliases.
HARDWARE_FORBIDDEN_DOMAINS = {0, 24, 25, 42}


class ProfileError(Exception):
    pass


@dataclass(frozen=True)
class BackendProfile:
    name: str
    ros_domain_id: int | None
    use_sim_time: bool
    cmd_vel_type: str
    odom_frame: str
    base_frame: str
    laser_frame: str
    map_id: str | None
    bringup_launch: str
    nav_launch: str
    mapping_launch: str | None
    lidar_serial_port: str | None
    motor_serial_port: str | None
    motor_baud: int | None
    motor_ids: tuple[int, ...]
    wheel_radius_m: float | None
    wheel_separation_m: float | None
    battery_percent_topic: str
    battery_voltage_topic: str
    battery_low_voltage: float | None
    hardware_motion_approved: bool


@dataclass(frozen=True)
class RobotProfile:
    robot_id: str
    enabled: bool
    namespace: str
    backend: BackendProfile

    def topic(self, relative: str) -> str:
        return f"/{self.namespace}/{relative.strip('/')}"

    @property
    def cmd_vel(self) -> str:
        return self.topic("cmd_vel")

    @property
    def odom(self) -> str:
        return self.topic("odom")

    @property
    def scan(self) -> str:
        return self.topic("scan")

    @property
    def navigate_to_pose(self) -> str:
        return self.topic("navigate_to_pose")

    @property
    def battery_percent(self) -> str:
        return self.topic(self.backend.battery_percent_topic)

    @property
    def battery_voltage(self) -> str:
        return self.topic(self.backend.battery_voltage_topic)


def profiles_path(root: Path) -> Path:
    return root / "config" / "robots" / "pinky_profiles.yaml"


def _frame(template: str, namespace: str, prefix_from_namespace: bool) -> str:
    prefix = f"{namespace}/" if prefix_from_namespace else ""
    return template.format(prefix=prefix)


def load_robot(root: Path, robot_id: str, backend: str) -> RobotProfile:
    if robot_id not in ROBOT_IDS:
        raise ProfileError(f"unknown robot id: {robot_id}")
    if backend not in BACKENDS:
        raise ProfileError(f"unknown backend: {backend}")
    path = profiles_path(root)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    robots = raw.get("robots") or {}
    backends = raw.get("backends") or {}
    robot = robots.get(robot_id)
    item = backends.get(backend)
    if not isinstance(robot, dict) or not isinstance(item, dict):
        raise ProfileError("profile is missing the robot or backend")
    prefix = bool(item.get("frame_prefix_from_namespace"))
    approved = bool(item.get("hardware_motion_approved"))
    if backend != "hardware" and approved:
        raise ProfileError("hardware_motion_approved belongs on the hardware backend")
    if backend == "hardware" and approved:
        raise ProfileError("hardware motion approval cannot be stored in the profile file")
    domain = item.get("ros_domain_id")
    domain_id = None if domain is None else int(domain)
    if backend == "hardware" and domain_id in HARDWARE_FORBIDDEN_DOMAINS:
        raise ProfileError(f"hardware ROS_DOMAIN_ID {domain_id} collides with RMF or Gazebo")
    motors = tuple(int(value) for value in (item.get("motor_ids") or []))
    return RobotProfile(
        robot_id=robot_id,
        enabled=bool(robot.get("enabled")),
        namespace=robot_id,
        backend=BackendProfile(
            name=backend,
            ros_domain_id=domain_id,
            use_sim_time=bool(item.get("use_sim_time")),
            cmd_vel_type=str(item.get("cmd_vel_type") or ""),
            odom_frame=_frame(str(item.get("odom_frame") or ""), robot_id, prefix),
            base_frame=_frame(str(item.get("base_frame") or ""), robot_id, prefix),
            laser_frame=_frame(str(item.get("laser_frame") or ""), robot_id, prefix),
            map_id=item.get("map_id") or None,
            bringup_launch=str(item.get("bringup_launch") or ""),
            nav_launch=str(item.get("nav_launch") or ""),
            mapping_launch=item.get("mapping_launch") or None,
            lidar_serial_port=item.get("lidar_serial_port") or None,
            motor_serial_port=item.get("motor_serial_port") or None,
            motor_baud=None if item.get("motor_baud") is None else int(item["motor_baud"]),
            motor_ids=motors,
            wheel_radius_m=None if item.get("wheel_radius_m") is None else float(item["wheel_radius_m"]),
            wheel_separation_m=None if item.get("wheel_separation_m") is None else float(item["wheel_separation_m"]),
            battery_percent_topic=str(item.get("battery_percent_topic") or "battery/percent"),
            battery_voltage_topic=str(item.get("battery_voltage_topic") or "battery/voltage"),
            battery_low_voltage=None if item.get("battery_low_voltage") is None else float(item["battery_low_voltage"]),
            hardware_motion_approved=False,
        ),
    )
