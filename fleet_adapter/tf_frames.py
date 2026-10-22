"""TF frame names. Namespace remaps do not change frame ids by themselves."""
from __future__ import annotations


def frame_ids(robot_id: str, backend: str) -> tuple[str, ...]:
    prefix = f"{robot_id}/" if backend == "gazebo" else ""
    return (
        f"{prefix}odom",
        f"{prefix}base_footprint",
        f"{prefix}base_link",
        f"{prefix}rplidar_link",
    )


def frame_collisions(robots: list[tuple[str, str, int]]) -> list[str]:
    """robots are (robot_id, backend, ros_domain_id)."""
    seen: dict[tuple[int, str], str] = {}
    hits: list[str] = []
    for robot_id, backend, domain in robots:
        for frame in frame_ids(robot_id, backend):
            key = (domain, frame)
            other = seen.get(key)
            if other is not None:
                hits.append(f"{other} and {robot_id} share {frame} on domain {domain}")
            else:
                seen[key] = robot_id
    return hits
