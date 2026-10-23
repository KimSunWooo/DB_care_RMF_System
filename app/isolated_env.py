"""Refuse an RMF development stack that can see the running Care-Move system."""
from __future__ import annotations

PRODUCTION_DOMAINS = frozenset({0, 24, 42, 77})
PRODUCTION_API_PORTS = frozenset({8000, 8011})
PRODUCTION_GATEWAY_PORTS = frozenset({17453, 17442})
PRODUCTION_DATABASES = frozenset({
    "/run/rmf_api_server.db",
    "data/task_queue.sqlite",
})


def isolation_problems(settings: dict) -> list[str]:
    problems = []
    domain = settings.get("ros_domain")
    if domain in PRODUCTION_DOMAINS:
        problems.append(f"ROS domain {domain} is a running Care-Move domain")
    port = settings.get("api_port")
    if port in PRODUCTION_API_PORTS:
        problems.append(f"API port {port} is already used by Care-Move or RMF")
    gateway = settings.get("gateway_port")
    if gateway in PRODUCTION_GATEWAY_PORTS:
        problems.append(f"gateway port {gateway} is a live gateway")
    database = str(settings.get("database") or "")
    if database in PRODUCTION_DATABASES or database.endswith("task_queue.sqlite") or database.endswith("rmf_api_server.db"):
        problems.append("database path is an operating RMF or Care-Move store")
    if settings.get("cmd_vel") in {"/cmd_vel", "/pinky2/cmd_vel", "/pinky3/cmd_vel"}:
        problems.append("cmd_vel is a physical Pinky topic")
    peers = settings.get("peers") or []
    if settings.get("rmw") == "rmw_fastrtps_cpp" and domain == 24:
        problems.append("Fast DDS on domain 24 is the physical Pinky graph")
    if any(peer not in {"127.0.0.1", "localhost"} for peer in peers):
        problems.append("DDS peers leave the loopback")
    return problems


ISOLATED_STACK = {
    "ros_domain": 91,
    "rmw": "rmw_cyclonedds_cpp",
    "peers": ["127.0.0.1"],
    "api_port": 8001,
    "gateway_port": 17491,
    "database": "/tmp/care-move-isolated/rmf.sqlite",
    "cmd_vel": None,
    "fleet": "isolated_mock",
}
