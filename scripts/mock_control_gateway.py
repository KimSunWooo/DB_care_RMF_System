"""Lease-enforcing mock gateway. Binds 127.0.0.1:17491 and never a motor or port 17453."""
from __future__ import annotations

import os
import secrets
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fleet_adapter.control_lease import LeaseAuthority
from fleet_adapter.mock_control_gateway import MockControlGateway, serve

PORT = 17491


def main() -> None:
    secret_path = Path(os.environ.get("CARE_MOVE_MOCK_LEASE_SECRET", "/tmp/care-move-mock-gw/lease-secret"))
    secret_path.parent.mkdir(parents=True, exist_ok=True)
    if not secret_path.is_file():
        secret_path.write_text(secrets.token_hex(32), encoding="utf-8")
        secret_path.chmod(0o600)
    authority = LeaseAuthority(secret_path.read_text(encoding="utf-8").strip(), secret_path.with_suffix(".generation"))
    gateway = MockControlGateway(authority)
    serve(gateway, PORT)
    print(f"mock control gateway 127.0.0.1:{PORT} lease_enforced boot={authority.boot_id}", flush=True)
    try:
        while True:
            time.sleep(0.2)
            gateway.expire_motion()
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
