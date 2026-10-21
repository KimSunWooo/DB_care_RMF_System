"""Launch gates for the virtual pinky1 Gazebo process.

These checks do not start Gazebo and do not touch the RMF domain.
"""
from __future__ import annotations

SIM_DOMAIN_ID = 42
# 0: running RMF API and traffic schedule.
# 24: ~/.bashrc jazzy alias. 25: pinky patrol workspace.
RESERVED_DOMAIN_IDS = {0, 24, 25}
EXPECTED_CMD_VEL_TYPE = "geometry_msgs/msg/Twist"
PINKY1_NAMESPACE = "pinky1"


def gazebo_block_reason(*, domain_id: int | None, gz_installed: bool,
                         hardware_nodes_on_domain: bool) -> str | None:
    if not gz_installed:
        return "Gazebo Sim is not installed"
    if domain_id is None:
        return "ROS_DOMAIN_ID is unset"
    if domain_id in RESERVED_DOMAIN_IDS:
        return f"ROS_DOMAIN_ID {domain_id} is reserved for RMF or the Pinky alias"
    if domain_id != SIM_DOMAIN_ID:
        return f"ROS_DOMAIN_ID must be {SIM_DOMAIN_ID} for this virtual pinky1"
    if hardware_nodes_on_domain:
        return "another robot node is already visible on the simulation domain"
    return None
