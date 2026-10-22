"""Domain gate and the domain-90 publisher. The motor port stays closed."""
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_publisher_exits_before_creating_a_node_on_real_domains():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "53"
    env.pop("PYTHONPATH", None)
    result = subprocess.run(
        [str(ROOT / ".venv" / "bin" / "python"), "scripts/odom_mock_ros_check.py"],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 2
    assert '"node_created": false' in result.stdout
    assert '"domain": 53' in result.stdout


def test_domain_90_publishes_odom_tf_and_joints():
    script = f'''
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=90
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="{ROOT}:${{PYTHONPATH:-}}"
python3 scripts/odom_mock_ros_check.py
'''
    result = subprocess.run(
        ["bash", "-lc", script],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert '"passed": true' in result.stdout
    assert '"tf_frames": true' in result.stdout
    assert '"stamp_match": true' in result.stdout
    assert '"port_opened": false' in result.stdout
    assert '"physical_stop_confirmed": false' in result.stdout
    assert "pinky1/l_wheel_joint" in result.stdout


def test_two_processes_exchange_odom_on_domain_90():
    script = f'''
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=90
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="{ROOT}:${{PYTHONPATH:-}}"
python3 scripts/odom_mock_ipc_check.py
'''
    result = subprocess.run(
        ["bash", "-lc", script],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert '"passed": true' in result.stdout
    assert '"reconnected": true' in result.stdout
    assert '"pose_match": true' in result.stdout
    assert '"port_opened": false' in result.stdout
