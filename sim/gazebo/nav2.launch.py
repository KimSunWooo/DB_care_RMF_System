"""Nav2 for the virtual pinky1 already running in Gazebo on domain 42.

Does not start Gazebo and does not send a goal. TF is relayed onto /pinky1/tf
because this stack is namespaced and the simulator publishes global /tf.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace, SetParameter

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PARAMS = ROOT / "sim" / "nav2" / "pinky1_gazebo.yaml"
MAP_YAML = ROOT / "sim" / "nav2" / "map" / "pinky1_room.yaml"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]
LIFECYCLE = [
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
]


def generate_launch_description() -> LaunchDescription:
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        raise RuntimeError(
            f"refusing Nav2 launch on ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')!r}; "
            "source sim/gazebo/env.sh"
        )
    params = str(PARAMS)
    common = {"use_sim_time": True}
    nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params, common, {"yaml_filename": str(MAP_YAML)}],
            remappings=REMAPS,
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params, common],
            remappings=REMAPS,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params, common],
            remappings=REMAPS,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params, common],
            remappings=REMAPS,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params, common],
            remappings=REMAPS,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params, common],
            remappings=REMAPS,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_pinky1",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": LIFECYCLE,
                "bond_timeout": 0.0,
            }],
        ),
    ]
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        ExecuteProcess(
            cmd=[sys.executable, str(HERE / "tf_relay.py")],
            output="screen",
        ),
        GroupAction([
            PushRosNamespace("pinky1"),
            SetParameter("use_sim_time", True),
            *nodes,
        ]),
    ])
