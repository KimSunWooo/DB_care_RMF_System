"""Headless Gazebo Harmonic bringup for one virtual pinky1.

Uses the existing Pinky URDF and diff-drive plugin. The spawn description
reattaches gpu_lidar to the unprefixed link and drops camera/IMU sensors.
Does not start Nav2 or the fleet adapter, and does not publish cmd_vel.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEFAULT_WORLD = HERE / "worlds" / "pinky1_empty.sdf"
DEFAULT_BRIDGE = HERE / "pinky1_bridge.yaml"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.gazebo.description_filter import prepare_spawn_urdf  # noqa: E402


def _gazebo_server(context):
    world = LaunchConfiguration("world").perform(context)
    gz_sim_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            "gz_args": f"-r -s --headless-rendering -v3 {world}",
            "on_exit_shutdown": "true",
        }.items(),
    )]


def _spawn(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    robot_name = LaunchConfiguration("robot_name").perform(context)
    xacro_file = os.path.join(
        get_package_share_directory("pinky_description"), "urdf", "robot.urdf.xacro"
    )
    completed = subprocess.run(
        [
            "xacro", xacro_file,
            f"namespace:={namespace}/",
            "is_sim:=True",
            "cam_tilt_deg:=0",
            "screen_tilt_deg:=-25",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    urdf = prepare_spawn_urdf(completed.stdout, namespace)
    out_dir = Path("/tmp/care_move_gz")
    out_dir.mkdir(parents=True, exist_ok=True)
    urdf_path = out_dir / "pinky1.urdf"
    urdf_path.write_text(urdf, encoding="utf-8")
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_pinky1",
        output="screen",
        arguments=[
            "-world", "pinky1_empty",
            "-name", robot_name,
            "-file", str(urdf_path),
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.1",
        ],
        parameters=[{"use_sim_time": True}],
    )
    return [TimerAction(period=8.0, actions=[spawn])]


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    bridge_params = LaunchConfiguration("bridge_params")

    domain_raw = os.environ.get("ROS_DOMAIN_ID", "")
    if domain_raw != "42":
        raise RuntimeError(
            f"refusing Gazebo launch on ROS_DOMAIN_ID={domain_raw!r}; "
            "source sim/gazebo/env.sh so the virtual pinky1 stays on domain 42"
        )

    upload_robot = os.path.join(
        get_package_share_directory("pinky_description"), "launch", "upload_robot.launch.py"
    )

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="pinky1"),
        DeclareLaunchArgument("robot_name", default_value="pinky1"),
        DeclareLaunchArgument("world", default_value=str(DEFAULT_WORLD)),
        DeclareLaunchArgument("bridge_params", default_value=str(DEFAULT_BRIDGE)),
        OpaqueFunction(function=_gazebo_server),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upload_robot),
            launch_arguments={
                "namespace": namespace,
                "is_sim": "True",
                "cam_tilt_deg": "0",
            }.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="pinky1_bridge",
            output="screen",
            arguments=["--ros-args", "-p", ["config_file:=", bridge_params]],
            parameters=[{"use_sim_time": True}],
        ),
        OpaqueFunction(function=_spawn),
    ])
