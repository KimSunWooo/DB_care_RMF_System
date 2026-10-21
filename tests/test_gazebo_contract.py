from pathlib import Path

import yaml

from fleet_adapter.config import load_fleet_spec
from sim.gazebo.description_filter import prepare_spawn_urdf
from sim.gazebo.gates import SIM_DOMAIN_ID, gazebo_block_reason
from sim.gazebo.room_geometry import (
    GOAL_XY,
    SPAWN_XY,
    point_occupied,
    world_sdf,
)
from sim.isaac.gates import launch_block_reason

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "sim" / "gazebo" / "pinky1_bridge.yaml"


def _bridge_rows() -> list[dict]:
    rows = yaml.safe_load(BRIDGE.read_text(encoding="utf-8"))
    assert isinstance(rows, list)
    return rows


def test_gazebo_bridge_keeps_twist_and_pinky1_topics():
    by_ros = {row["ros_topic_name"]: row for row in _bridge_rows()}
    assert by_ros["/clock"]["ros_type_name"] == "rosgraph_msgs/msg/Clock"
    assert by_ros["/clock"]["direction"] == "GZ_TO_ROS"
    assert by_ros["/tf"]["ros_type_name"] == "tf2_msgs/msg/TFMessage"
    assert by_ros["/pinky1/cmd_vel"]["ros_type_name"] == "geometry_msgs/msg/Twist"
    assert by_ros["/pinky1/cmd_vel"]["direction"] == "ROS_TO_GZ"
    assert by_ros["/pinky1/odom"]["ros_type_name"] == "nav_msgs/msg/Odometry"
    assert by_ros["/pinky1/odom"]["direction"] == "GZ_TO_ROS"
    assert by_ros["/pinky1/scan"]["ros_type_name"] == "sensor_msgs/msg/LaserScan"
    assert by_ros["/pinky1/scan"]["qos_profile"] == "SENSOR_DATA"


def test_spawn_filter_keeps_lidar_and_drops_camera():
    urdf = """<robot name="pinky">
      <link name="rplidar_link"/>
      <link name="front_camera_link"/>
      <link name="l_wheel"/>
      <gazebo reference="pinky1/rplidar_link">
        <sensor name="pinky1/gpu_lidar" type="gpu_lidar"/>
      </gazebo>
      <gazebo reference="pinky1/front_camera_link">
        <sensor name="pinky1/camera" type="camera"/>
      </gazebo>
      <gazebo reference="pinky1/l_wheel"/>
      <gazebo reference="pinky1/l_wheel_joint"/>
    </robot>"""
    fixed = prepare_spawn_urdf(urdf, "pinky1")
    assert 'reference="rplidar_link"' in fixed
    assert "gpu_lidar" in fixed
    assert 'type="camera"' not in fixed
    assert 'reference="l_wheel"' in fixed
    assert 'reference="pinky1/l_wheel_joint"' in fixed


def test_room_map_matches_spawn_goal_and_world():
    assert point_occupied(*SPAWN_XY) is False
    assert point_occupied(*GOAL_XY) is False
    assert point_occupied(2.0, 1.2) is True
    assert point_occupied(0.0, 3.6) is True
    sdf = (ROOT / "sim" / "gazebo" / "worlds" / "pinky1_empty.sdf").read_text(encoding="utf-8")
    assert sdf == world_sdf()
    assert "gz-sim-sensors-system" in sdf
    params = (ROOT / "sim" / "nav2" / "pinky1_gazebo.yaml").read_text(encoding="utf-8")
    assert "pinky1/base_footprint" in params
    assert "pinky1/odom" in params
    assert "enable_stamped_cmd_vel: false" in params


def test_gazebo_launch_rejects_reserved_domains_without_ram_gate():
    assert gazebo_block_reason(
        domain_id=None, gz_installed=True, hardware_nodes_on_domain=False,
    )
    for domain in (0, 24, 25):
        assert gazebo_block_reason(
            domain_id=domain, gz_installed=True, hardware_nodes_on_domain=False,
        )
    assert gazebo_block_reason(
        domain_id=SIM_DOMAIN_ID, gz_installed=True, hardware_nodes_on_domain=True,
    )
    assert gazebo_block_reason(
        domain_id=SIM_DOMAIN_ID, gz_installed=True, hardware_nodes_on_domain=False,
    ) is None


def test_isaac_gate_and_fleet_lock_stay_unchanged():
    spec = load_fleet_spec(ROOT / "config" / "rmf" / "care_move_fleet.yaml", ROOT)
    assert spec.allow_motion is False
    assert spec.simulation_confirmed is False
    assert spec.graph_alignment == "placeholder"
    assert launch_block_reason(
        domain_id=SIM_DOMAIN_ID, available_bytes=1024,
        isaac_installed=True, hardware_nodes_on_domain=False,
    )
