"""Adjust a namespaced Pinky URDF before Gazebo spawn.

Link names in the URDF stay unprefixed. The Gazebo sensor reference in
pinky_gz.urdf.xacro uses the namespace prefix, so a namespaced description
does not attach gpu_lidar. Camera and IMU sensors are removed here so the
headless render budget stays on the lidar.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET

DROP_SENSOR_TYPES = {"camera", "imu"}


def prepare_spawn_urdf(urdf: str, namespace: str) -> str:
    prefix = namespace.strip("/")
    prefix = f"{prefix}/" if prefix else ""
    root = ET.fromstring(urdf)
    link_names = {link.get("name") for link in root.findall("link")}
    for gazebo in list(root.findall("gazebo")):
        sensors = gazebo.findall("sensor")
        if any(sensor.get("type") in DROP_SENSOR_TYPES for sensor in sensors):
            root.remove(gazebo)
            continue
        reference = gazebo.get("reference")
        if not reference or reference in link_names or not prefix or not reference.startswith(prefix):
            continue
        stripped = reference[len(prefix):]
        if stripped in link_names:
            gazebo.set("reference", stripped)
    return ET.tostring(root, encoding="unicode")
