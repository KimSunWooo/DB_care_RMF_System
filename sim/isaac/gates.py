"""Launch gates for the virtual pinky1. These checks do not start Isaac Sim."""
from __future__ import annotations

SIM_DOMAIN_ID = 42
# 0: running RMF API and traffic schedule.
# 24: ~/.bashrc jazzy alias. 25: pinky patrol workspace.
RESERVED_DOMAIN_IDS = {0, 24, 25}
MIN_AVAILABLE_BYTES = 8 * 1024 ** 3
EXPECTED_CMD_VEL_TYPE = "geometry_msgs/msg/Twist"
WHEEL_RADIUS_M = 0.028
# URDF joint origins are y=±0.04055. The Gazebo plugin separately sets 0.0961.
URDF_WHEEL_SEPARATION_M = 0.0811
GAZEBO_PLUGIN_WHEEL_SEPARATION_M = 0.0961


def launch_block_reason(*, domain_id: int | None, available_bytes: int,
                        isaac_installed: bool, hardware_nodes_on_domain: bool) -> str | None:
    if not isaac_installed:
        return "Isaac Sim is not installed"
    if domain_id is None:
        return "ROS_DOMAIN_ID is unset"
    if domain_id in RESERVED_DOMAIN_IDS:
        return f"ROS_DOMAIN_ID {domain_id} is reserved for RMF or the Pinky alias"
    if domain_id != SIM_DOMAIN_ID:
        return f"ROS_DOMAIN_ID must be {SIM_DOMAIN_ID} for this virtual pinky1"
    if hardware_nodes_on_domain:
        return "another robot node is already visible on the simulation domain"
    if available_bytes < MIN_AVAILABLE_BYTES:
        return "available RAM is below 8 GiB; refusing to start Isaac Sim"
    return None
