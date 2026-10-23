"""HTTP client for an already running RMF-Web API server.

Reads the URL from the caller. This module does not choose a port and does not
open ROS, Nav2, or elevator connections.
"""
from __future__ import annotations

import base64
import json
import logging
import time
from datetime import datetime, timezone
from typing import Any
from urllib.parse import quote

import httpx

logger = logging.getLogger("care_move.task")
if not logger.handlers:
    _handler = logging.StreamHandler()
    _handler.setFormatter(logging.Formatter("%(message)s"))
    logger.addHandler(_handler)
    logger.setLevel(logging.INFO)
    logger.propagate = False

from app.rmf_token import TokenUnavailable


class RmfClientError(Exception):
    def __init__(self, kind: str, detail: str, status_code: int | None = None):
        super().__init__(detail)
        self.kind = kind
        self.detail = detail
        self.status_code = status_code


def navigation_waypoints(building_map: dict) -> tuple[list[str], list[str]]:
    """Waypoint names from RMF building-map nav graphs. Not UI concept coordinates."""
    names: set[str] = set()
    levels: list[str] = []
    for level in building_map.get("levels") or []:
        if not isinstance(level, dict):
            continue
        if level.get("name"):
            levels.append(str(level["name"]))
        for graph in level.get("nav_graphs") or []:
            if not isinstance(graph, dict):
                continue
            for vertex in graph.get("vertices") or []:
                if isinstance(vertex, dict) and vertex.get("name"):
                    names.add(str(vertex["name"]))
    return sorted(names), levels


def normalize_token(token: str) -> str:
    """Strip a repeated Bearer prefix so the header is sent once."""
    value = (token or "").strip()
    while value.lower().startswith("bearer "):
        value = value[7:].strip()
    return value


def jwt_expiry(token: str) -> dict:
    """Read exp for display. This does not decide whether the server accepted the token."""
    result = {"expires_at": None, "expired": None}
    parts = token.split(".")
    if len(parts) < 2:
        return result
    payload = parts[1]
    try:
        data = json.loads(base64.urlsafe_b64decode(payload + "=" * (-len(payload) % 4)))
    except (ValueError, json.JSONDecodeError):
        return result
    exp = data.get("exp") if isinstance(data, dict) else None
    if not isinstance(exp, (int, float)):
        return result
    result["expires_at"] = datetime.fromtimestamp(float(exp), timezone.utc).isoformat()
    result["expired"] = time.time() >= float(exp)
    return result


def booking_ids_for_label(states: Any, label: str) -> list[str]:
    """Booking ids whose labels contain this Care-Move request label. Order is preserved."""
    if not isinstance(states, list) or not label:
        return []
    found: list[str] = []
    for state in states:
        if not isinstance(state, dict):
            continue
        booking = state.get("booking")
        if not isinstance(booking, dict):
            continue
        labels = booking.get("labels") or []
        booking_id = booking.get("id")
        if label in labels and isinstance(booking_id, str) and booking_id:
            found.append(booking_id)
    return found


def state_for_label(states: Any, label: str) -> dict | None:
    """The single task state for a label. None when missing or ambiguous."""
    if not isinstance(states, list):
        return None
    matches = []
    for state in states:
        if not isinstance(state, dict):
            continue
        booking = state.get("booking")
        if isinstance(booking, dict) and label in (booking.get("labels") or []):
            matches.append(state)
    if len(matches) != 1:
        return None
    return matches[0]


def log_task_event(event: str, **fields: Any) -> None:
    """One JSON line. Callers must not pass tokens, secrets, or Authorization headers."""
    payload = {"timestamp": datetime.now(timezone.utc).isoformat(), "event": event}
    payload.update(fields)
    logger.info("%s", json.dumps(payload, ensure_ascii=False, default=str))


def _error_text(payload: Any) -> str:
    if isinstance(payload, dict):
        if payload.get("errors"):
            parts = []
            for item in payload["errors"]:
                if isinstance(item, dict):
                    parts.append(str(item.get("detail") or item.get("category") or item))
                else:
                    parts.append(str(item))
            if parts:
                return "; ".join(parts)
        if payload.get("detail"):
            return str(payload["detail"])[:1000]
    return str(payload)[:1000]


class RmfClient:
    def __init__(self, base_url: str, token: str = "", timeout: float = 5.0,
                 transport: httpx.BaseTransport | None = None, provider=None,
                 submit_timeout: float = 12.0):
        self.base_url = base_url.rstrip("/") if base_url else ""
        self.token = normalize_token(token)
        self.timeout = timeout
        self.submit_timeout = submit_timeout
        self._transport = transport
        self.provider = provider

    async def _authorization(self) -> tuple[dict[str, str], int | None]:
        if self.provider is not None:
            try:
                token = normalize_token(await self.provider.get_token())
            except TokenUnavailable as ex:
                raise RmfClientError("token_unavailable", str(ex)) from ex
            if not token:
                raise RmfClientError("token_unavailable", "RMF token provider returned an empty token")
            return {"Authorization": f"Bearer {token}"}, self.provider.generation
        if not self.token:
            return {}, None
        return {"Authorization": f"Bearer {self.token}"}, None

    async def request(self, method: str, path: str, payload: dict | None = None,
                      _auth_retry: bool = False, timeout: float | None = None) -> Any:
        if not self.base_url:
            raise RmfClientError("not_configured", "RMF_API_URL is not set")
        headers, generation = await self._authorization()
        limit = self.timeout if timeout is None else timeout
        # A single float in httpx applies to connect, read, write, and pool together.
        phases = httpx.Timeout(connect=min(2.0, limit), read=limit, write=min(5.0, limit), pool=min(5.0, limit))
        try:
            async with httpx.AsyncClient(base_url=self.base_url, timeout=phases,
                                          transport=self._transport, headers=headers) as client:
                response = await client.request(method, path, json=payload)
        except httpx.TimeoutException as ex:
            raise RmfClientError("timeout", f"RMF API timed out: {type(ex).__name__}") from ex
        except httpx.ConnectError as ex:
            raise RmfClientError("unreachable", f"RMF API unreachable: {ex}") from ex
        except httpx.RequestError as ex:
            raise RmfClientError("unreachable", f"RMF API request failed: {ex}") from ex
        body: Any
        try:
            body = response.json()
        except ValueError:
            body = {"raw": response.text[:1000]}
        if response.status_code == 401:
            if (not _auth_retry and method.upper() in {"GET", "HEAD"}
                    and self.provider is not None and generation is not None
                    and getattr(self.provider, "status", lambda: {})().get("auth_mode") == "local_jwt"):
                try:
                    await self.provider.refresh_after_unauthorized(generation)
                except TokenUnavailable as ex:
                    raise RmfClientError("auth", str(ex), 401) from ex
                return await self.request(method, path, payload, _auth_retry=True)
            raise RmfClientError("auth", _error_text(body) or "RMF API authentication failed", 401)
        if response.status_code == 403:
            raise RmfClientError("forbidden", _error_text(body) or "RMF API forbidden", 403)
        if response.status_code == 404:
            raise RmfClientError("not_found", _error_text(body) or "RMF API returned 404", 404)
        if isinstance(body, dict) and body.get("success") is False:
            raise RmfClientError("rejected", _error_text(body) or "RMF rejected the request",
                                 response.status_code)
        if response.status_code >= 500:
            raise RmfClientError("server", _error_text(body) or "RMF API server error",
                                 response.status_code)
        if response.status_code >= 400:
            raise RmfClientError("client", _error_text(body) or "RMF API rejected the HTTP request",
                                 response.status_code)
        return body

    async def get_json(self, path: str) -> Any:
        return await self.request("GET", path)

    async def post_json(self, path: str, payload: dict, timeout: float | None = None) -> Any:
        return await self.request("POST", path, payload, timeout=timeout)

    async def recover_submission(self, correlation_id: str) -> dict | None:
        """One GET by requester label. Does not POST again."""
        label = f"care-move-request={correlation_id}"
        requester = quote(f"care-move:{correlation_id}", safe="")
        try:
            states = await self.get_json(f"/tasks?requester={requester}")
        except RmfClientError as ex:
            log_task_event("RMF_TASK_BOOKING_OBSERVED", correlation_id=correlation_id,
                           error_kind=ex.kind, match_count=0, task_status="SUBMISSION_UNKNOWN")
            return None
        state = state_for_label(states, label)
        if state is None:
            log_task_event("RMF_TASK_BOOKING_OBSERVED", correlation_id=correlation_id,
                           match_count=len(booking_ids_for_label(states, label)),
                           task_status="SUBMISSION_UNKNOWN")
            return None
        booking = state.get("booking") if isinstance(state, dict) else None
        booking_id = booking.get("id") if isinstance(booking, dict) else None
        log_task_event("RMF_TASK_BOOKING_OBSERVED", correlation_id=correlation_id,
                       rmf_booking_id=booking_id, task_status=state.get("status"),
                       match_count=1)
        return {"success": True, "state": state}

    async def cancel_task(self, task_id: str) -> Any:
        return await self.post_json("/tasks/cancel_task", {
            "type": "cancel_task_request",
            "task_id": task_id,
            "labels": ["care-move-dashboard"],
        })

    async def probe(self, fleet_name: str, expected_robots: list[str]) -> dict:
        """GET-only contract and fleet check. Does not submit tasks."""
        observed: dict[str, Any] = {
            "status": "not_checked",
            "detail": "",
            "reachable": False,
            "authenticated": False,
            "api_url": self.base_url or None,
            "fleet_name": fleet_name,
            "fleet_registered": False,
            "robots_on_fleet": [],
            "missing_robots": list(expected_robots),
            "graph_waypoints": None,
            "graph_levels": [],
            "endpoints": {},
            "http_status": None,
            "last_error_kind": None,
            "fleet_state": None,
            "state_transport": "http_poll",
            "api_connected": False,
            "auth_state": "unknown",
            "token_configured": bool(self.token),
            "token_expires_at": None,
            "token_expired_locally": None,
            "fleet_count": None,
            "fleet_names": [],
            "fleet_data": "unknown",
            "robot_data": "unknown",
            "robot_count": None,
            "map_data": "unknown",
            "map_http_status": None,
            "map_detail": "",
            "operation_ready": False,
            "motion_verified": False,
            "data_fresh": False,
        }
        if self.provider is None:
            expiry = jwt_expiry(self.token) if self.token else {"expires_at": None, "expired": None}
            observed["token_expires_at"] = expiry["expires_at"]
            observed["token_expired_locally"] = expiry["expired"]
            observed["token_configured"] = bool(self.token)
        if not self.base_url:
            observed["status"] = "not_configured"
            observed["detail"] = "RMF_API_URL이 설정되지 않았습니다."
            observed["last_error_kind"] = "not_configured"
            return observed
        try:
            spec = await self.get_json("/openapi.json")
        except RmfClientError as ex:
            observed.update(status=ex.kind, detail=ex.detail, http_status=ex.status_code,
                            last_error_kind=ex.kind, reachable=False)
            return observed
        if not isinstance(spec, dict):
            observed.update(status="invalid_response", detail="OpenAPI 문서가 객체가 아닙니다.",
                            reachable=True, api_connected=True, last_error_kind="invalid_response")
            return observed
        observed["reachable"] = True
        observed["api_connected"] = True
        paths = spec.get("paths") if isinstance(spec.get("paths"), dict) else {}
        observed["endpoints"] = {
            "fleets": "/fleets" in paths,
            "fleet_state": "/fleets/{name}/state" in paths,
            "dispatch_task": "/tasks/dispatch_task" in paths,
            "robot_task": "/tasks/robot_task" in paths,
            "cancel_task": "/tasks/cancel_task" in paths,
            "task_state": "/tasks/{task_id}/state" in paths,
            "tasks": "/tasks" in paths,
            "socket_io": "/socket.io" in paths,
            "lifts": "/lifts" in paths,
            "building_map": "/building_map" in paths,
        }
        required = ("dispatch_task", "robot_task", "fleets", "task_state", "cancel_task")
        if not all(observed["endpoints"][name] for name in required):
            observed.update(status="contract_mismatch",
                            detail="실행 중인 RMF API에 필요한 작업/Fleet 경로가 없습니다.",
                            last_error_kind="contract_mismatch")
            return observed
        if self.provider is not None:
            try:
                await self.provider.get_token()
            except (TokenUnavailable, RmfClientError) as ex:
                message = getattr(ex, "detail", None) or str(ex)
                observed.update(status="token_unavailable", auth_state="missing_token", authenticated=False,
                                data_fresh=False, last_error_kind="token_unavailable", token_configured=False,
                                detail=message)
                return observed
            meta = self.provider.status()
            observed["token_configured"] = bool(meta.get("token_ready"))
            observed["token_expires_at"] = meta.get("expires_at")
            observed["token_expired_locally"] = False
        elif not self.token:
            observed.update(status="token_missing", auth_state="missing_token", authenticated=False,
                            data_fresh=False, last_error_kind="token_missing",
                            detail="RMF_API_TOKEN이 없습니다. static 모드에서는 실행용 .env에 토큰이 필요합니다.")
            return observed
        try:
            fleets = await self.get_json("/fleets")
        except RmfClientError as ex:
            status, detail = _auth_failure(ex, observed["token_expired_locally"])
            observed.update(status=status, detail=detail, http_status=ex.status_code,
                            last_error_kind=ex.kind, authenticated=False, data_fresh=False,
                            auth_state="unauthorized" if status == "auth_failed" else (
                                "forbidden" if status == "forbidden" else "unknown"))
            return observed
        if not isinstance(fleets, list):
            observed.update(status="invalid_response", authenticated=True, auth_state="ok",
                            detail="GET /fleets 응답이 배열이 아닙니다.",
                            last_error_kind="invalid_response", data_fresh=False)
            return observed
        observed["authenticated"] = True
        observed["auth_state"] = "ok"
        observed["data_fresh"] = True
        observed["http_status"] = 200
        names_all = [item.get("name") for item in fleets if isinstance(item, dict) and item.get("name")]
        observed["fleet_count"] = len(fleets)
        observed["fleet_names"] = names_all
        observed["fleet_data"] = "empty" if len(fleets) == 0 else "present"
        fleet = next((item for item in fleets if isinstance(item, dict) and item.get("name") == fleet_name), None)
        observed["fleet_registered"] = fleet is not None
        remote_robots = (fleet or {}).get("robots") if isinstance(fleet, dict) else None
        names = sorted(remote_robots) if isinstance(remote_robots, dict) else []
        observed["robots_on_fleet"] = names
        observed["robot_count"] = len(names)
        observed["robot_data"] = "present" if names else "none"
        observed["missing_robots"] = [robot for robot in expected_robots if robot not in names]
        observed["fleet_state"] = fleet if isinstance(fleet, dict) else None
        await self._read_map(observed)
        observed["status"], observed["detail"] = _fleet_status(observed, fleet_name)
        observed["operation_ready"] = observed["status"] == "ready"
        return observed

    async def _read_map(self, observed: dict) -> None:
        if not observed["endpoints"].get("building_map"):
            observed["map_data"] = "path_missing"
            observed["map_detail"] = "실행 중 OpenAPI에 GET /building_map 경로가 없습니다."
            return
        try:
            building_map = await self.get_json("/building_map")
        except RmfClientError as ex:
            observed["map_http_status"] = ex.status_code
            if ex.status_code == 404:
                observed["map_data"] = "missing"
                observed["map_detail"] = (
                    "GET /building_map 은 OpenAPI에 있고 응답은 404입니다. "
                    "경로 오류가 아니라 지도 데이터 부재입니다. " + (ex.detail or "")
                ).strip()
                return
            observed["map_data"] = "error"
            observed["map_detail"] = ex.detail
            return
        if not isinstance(building_map, dict):
            observed["map_data"] = "error"
            observed["map_detail"] = "building_map 응답 형식이 올바르지 않습니다."
            return
        waypoints, levels = navigation_waypoints(building_map)
        observed["graph_waypoints"] = waypoints
        observed["graph_levels"] = levels
        observed["map_http_status"] = 200
        observed["map_data"] = "present" if waypoints else "empty"
        observed["map_detail"] = "navigation graph waypoint를 수신했습니다." if waypoints else "지도 응답에 waypoint가 없습니다."


def _auth_failure(ex: RmfClientError, expired_locally: bool | None) -> tuple[str, str]:
    if ex.status_code == 403 or ex.kind == "forbidden":
        return "forbidden", "RMF API가 403을 반환했습니다. 권한 부족입니다."
    if ex.status_code == 401 or ex.kind == "auth":
        detail = "RMF API가 401을 반환했습니다. 인증 실패이며 토큰 만료일 수 있습니다."
        if expired_locally is True:
            detail += " 로컬 JWT exp는 이미 지났습니다."
        elif expired_locally is False:
            detail += " 로컬 JWT exp는 아직 남아 있습니다. 성공 여부는 서버 응답을 기준으로 합니다."
        return "auth_failed", detail
    return ex.kind, ex.detail


def _fleet_status(observed: dict, fleet_name: str) -> tuple[str, str]:
    if observed.get("fleet_data") == "empty":
        return "fleet_empty", "인증에는 성공했습니다. GET /fleets 는 빈 목록입니다. Fleet 데이터가 없습니다."
    if not observed.get("fleet_registered"):
        found = ", ".join(observed.get("fleet_names") or []) or "없음"
        return "fleet_missing", f"인증에는 성공했습니다. Fleet '{fleet_name}'은 없습니다. 수신한 Fleet: {found}"
    if observed.get("robot_data") != "present":
        return "robots_waiting", f"Fleet '{fleet_name}'은 있으나 로봇 상태가 없습니다. 로봇 데이터를 기다리는 중입니다."
    if observed.get("map_data") == "missing":
        return "map_unavailable", observed.get("map_detail") or "지도 데이터가 없습니다."
    if observed.get("map_data") == "path_missing":
        return "map_path_missing", observed.get("map_detail") or "지도 경로가 명세에 없습니다."
    if observed.get("map_data") in {"error", "unknown"}:
        return "map_error", observed.get("map_detail") or "지도를 읽지 못했습니다."
    if observed.get("map_data") == "empty" or not observed.get("graph_waypoints"):
        return "graph_empty", "지도 응답에 navigation graph waypoint가 없습니다."
    detail = f"인증에 성공했고 Fleet '{fleet_name}' 로봇 상태와 지도를 수신했습니다."
    if observed.get("missing_robots"):
        detail += " 설정에는 있으나 Fleet 상태에 없는 로봇: " + ", ".join(observed["missing_robots"])
    detail += " 인증·데이터 수신이며, 로봇 주행 완료는 확인하지 않았습니다."
    return "ready", detail
