import os

from fleet_adapter.config import load_fleet_spec
from sim.isaac.gates import SIM_DOMAIN_ID, launch_block_reason
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_existing_fleet_flags_stay_closed():
    spec = load_fleet_spec(ROOT / "config" / "rmf" / "care_move_fleet.yaml", ROOT)
    assert spec.graph_alignment == "placeholder"
    assert spec.allow_motion is False
    assert spec.simulation_confirmed is False


def test_sim_launch_rejects_reserved_domains_and_low_memory():
    assert launch_block_reason(
        domain_id=None, available_bytes=20 * 1024 ** 3,
        isaac_installed=True, hardware_nodes_on_domain=False,
    )
    for domain in (0, 24, 25):
        assert launch_block_reason(
            domain_id=domain, available_bytes=20 * 1024 ** 3,
            isaac_installed=True, hardware_nodes_on_domain=False,
        )
    assert launch_block_reason(
        domain_id=SIM_DOMAIN_ID, available_bytes=1024,
        isaac_installed=True, hardware_nodes_on_domain=False,
    )
    assert launch_block_reason(
        domain_id=SIM_DOMAIN_ID, available_bytes=20 * 1024 ** 3,
        isaac_installed=True, hardware_nodes_on_domain=True,
    )
    assert launch_block_reason(
        domain_id=SIM_DOMAIN_ID, available_bytes=20 * 1024 ** 3,
        isaac_installed=True, hardware_nodes_on_domain=False,
    ) is None


def test_bringup_module_does_not_import_isaac(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    import sim.isaac.pinky1_bringup as bringup
    assert "isaacsim" not in os.sys.modules
    assert bringup.URDF.name == "robot.urdf.xacro"
