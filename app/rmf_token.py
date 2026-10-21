"""Issue and rotate Open-RMF API JWTs inside this process.

One provider is shared by the process. Each process keeps its own cache.
Signing material never leaves this module's return values except as a Bearer
token on server-side HTTP requests.
"""
from __future__ import annotations

import asyncio
import base64
import contextlib
import hashlib
import hmac
import json
import logging
import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone

logger = logging.getLogger("care_move.rmf_token")
ALLOWED_ALGORITHMS = {"HS256"}


class TokenConfigError(Exception):
    pass


class TokenUnavailable(Exception):
    pass


@dataclass
class Clock:
    def wall(self) -> float:
        return time.time()

    def mono(self) -> float:
        return time.monotonic()


def _b64(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).rstrip(b"=").decode("ascii")


def sign_hs256(payload: dict, secret: str) -> str:
    header = {"alg": "HS256", "typ": "JWT"}
    signing_input = ".".join((
        _b64(json.dumps(header, separators=(",", ":")).encode("utf-8")),
        _b64(json.dumps(payload, separators=(",", ":")).encode("utf-8")),
    ))
    signature = hmac.new(secret.encode("utf-8"), signing_input.encode("ascii"), hashlib.sha256).digest()
    return signing_input + "." + _b64(signature)


def _iso(timestamp: float | None) -> str | None:
    if timestamp is None:
        return None
    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat()


def _positive(name: str, value: float) -> float:
    if value <= 0:
        raise TokenConfigError(f"{name} must be greater than 0")
    return value


class RmfTokenProvider:
    def __init__(self, *, secret: str, issuer: str, audience: str, username: str,
                 algorithm: str = "HS256", ttl_seconds: float = 3600,
                 rotate_interval_seconds: float = 2700, expiry_skew_seconds: float = 60,
                 retry_interval_seconds: float = 30, clock: Clock | None = None,
                 sleeper=None, signer=None):
        if algorithm not in ALLOWED_ALGORITHMS:
            raise TokenConfigError(f"RMF_JWT_ALGORITHM must be one of {sorted(ALLOWED_ALGORITHMS)}")
        if not secret:
            raise TokenConfigError("RMF_JWT_SECRET is required for local_jwt")
        if not issuer or not audience or not username:
            raise TokenConfigError("RMF_JWT_ISSUER, RMF_JWT_AUDIENCE, and RMF_JWT_USERNAME are required")
        self.ttl_seconds = _positive("RMF_JWT_TTL_SECONDS", float(ttl_seconds))
        self.rotate_interval_seconds = _positive("RMF_JWT_ROTATE_INTERVAL_SECONDS", float(rotate_interval_seconds))
        if expiry_skew_seconds < 0:
            raise TokenConfigError("RMF_JWT_EXPIRY_SKEW_SECONDS must be 0 or greater")
        self.expiry_skew_seconds = float(expiry_skew_seconds)
        if self.rotate_interval_seconds >= self.ttl_seconds - self.expiry_skew_seconds:
            raise TokenConfigError("rotation interval must be less than ttl minus expiry skew")
        self.secret = secret
        self.issuer = issuer
        self.audience = audience
        self.username = username
        self.algorithm = algorithm
        self.retry_interval_seconds = _positive("token refresh retry", float(retry_interval_seconds))
        self._clock = clock or Clock()
        self._sleeper = sleeper or asyncio.sleep
        self._signer = signer
        self._lock = asyncio.Lock()
        self._token: str | None = None
        self.generation = 0
        self._issued_mono: float | None = None
        self._issued_wall: float | None = None
        self._expires_wall: float | None = None
        self._last_result: str | None = None
        self._last_error: str | None = None
        self._last_reason: str | None = None
        self._retry_not_before = 0.0
        self._task: asyncio.Task | None = None
        self._closed = False

    def _remaining(self) -> float | None:
        if self._expires_wall is None:
            return None
        return self._expires_wall - self._clock.wall()

    def _within_skew_or_expired(self) -> bool:
        remaining = self._remaining()
        return remaining is None or remaining <= self.expiry_skew_seconds

    def _rotation_due(self) -> bool:
        if self._issued_mono is None:
            return True
        return self._clock.mono() - self._issued_mono >= self.rotate_interval_seconds

    def _needs_refresh(self) -> bool:
        return self._token is None or self._rotation_due() or self._within_skew_or_expired()

    def _safe_error(self, ex: Exception) -> str:
        text = str(ex)
        if self.secret and self.secret in text:
            text = text.replace(self.secret, "[redacted]")
        return text[:300] or ex.__class__.__name__

    def _sign(self) -> tuple[str, float, float]:
        issued = self._clock.wall()
        expires = issued + self.ttl_seconds
        payload = {
            "iss": self.issuer,
            "aud": self.audience,
            "sub": self.username,
            "preferred_username": self.username,
            "iat": int(issued),
            "exp": int(expires),
        }
        if self._signer is not None:
            return self._signer(payload), issued, expires
        return sign_hs256(payload, self.secret), issued, expires

    def _issue_locked(self, reason: str, *, allow_stale: bool) -> str:
        previous = self._token
        try:
            token, issued, expires = self._sign()
        except Exception as ex:
            self._last_result = "failed"
            self._last_error = self._safe_error(ex)
            self._last_reason = reason
            self._retry_not_before = self._clock.mono() + self.retry_interval_seconds
            logger.info("rmf jwt issue failed generation=%s reason=%s error=%s",
                        self.generation, reason, self._last_error)
            if allow_stale and previous and not self._within_skew_or_expired():
                return previous
            raise TokenUnavailable(self._last_error) from ex
        self._token = token
        self.generation += 1
        self._issued_mono = self._clock.mono()
        self._issued_wall = issued
        self._expires_wall = expires
        self._last_result = "ok"
        self._last_error = None
        self._last_reason = reason
        self._retry_not_before = 0.0
        logger.info("rmf jwt issued generation=%s reason=%s expires_at=%s",
                    self.generation, reason, _iso(expires))
        return token

    async def get_token(self) -> str:
        if not self._needs_refresh():
            return self._token or ""
        return await self.refresh_if_needed()

    async def refresh_if_needed(self) -> str:
        async with self._lock:
            if not self._needs_refresh():
                return self._token or ""
            if self._token is None and self._clock.mono() < self._retry_not_before:
                raise TokenUnavailable(self._last_error or "token refresh is backing off")
            if (self._token and not self._within_skew_or_expired()
                    and self._last_result == "failed" and self._clock.mono() < self._retry_not_before):
                return self._token
            reason = "startup" if self._token is None else (
                "expiry_skew" if self._within_skew_or_expired() else "rotate")
            return self._issue_locked(reason, allow_stale=True)

    async def refresh_after_unauthorized(self, rejected_generation: int) -> str:
        async with self._lock:
            if (self.generation != rejected_generation and self._token
                    and not self._within_skew_or_expired()):
                self._last_reason = "reuse_after_peer_refresh"
                return self._token
            return self._issue_locked("unauthorized", allow_stale=False)

    def _sleep_seconds(self) -> float:
        if self._token is None or self._last_result == "failed":
            return self.retry_interval_seconds
        remaining_rotate = self.rotate_interval_seconds - (self._clock.mono() - (self._issued_mono or self._clock.mono()))
        remaining_skew = (self._remaining() or 0) - self.expiry_skew_seconds
        return max(0.0, min(remaining_rotate, remaining_skew))

    async def _loop(self) -> None:
        while not self._closed:
            try:
                await self.refresh_if_needed()
            except asyncio.CancelledError:
                raise
            except Exception:
                logger.info("rmf jwt background refresh failed generation=%s error=%s",
                            self.generation, self._last_error)
            if self._closed:
                return
            await self._sleeper(self._sleep_seconds())

    async def start(self) -> None:
        if self._task and not self._task.done():
            return
        self._closed = False
        self._task = asyncio.create_task(self._loop(), name="rmf-token-refresh")

    async def close(self) -> None:
        self._closed = True
        task = self._task
        self._task = None
        if task:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    def status(self) -> dict:
        next_rotation = None
        if self._issued_mono is not None:
            next_rotation = self._clock.wall() + max(0.0, self.rotate_interval_seconds - (self._clock.mono() - self._issued_mono))
        return {
            "auth_mode": "local_jwt",
            "token_ready": bool(self._token) and not self._within_skew_or_expired(),
            "issued_at": _iso(self._issued_wall),
            "expires_at": _iso(self._expires_wall),
            "next_rotation_at": _iso(next_rotation),
            "generation": self.generation,
            "last_refresh_result": self._last_result,
            "last_refresh_reason": self._last_reason,
            "last_error": self._last_error,
            "algorithm": self.algorithm,
            "ttl_seconds": self.ttl_seconds,
            "rotate_interval_seconds": self.rotate_interval_seconds,
            "expiry_skew_seconds": self.expiry_skew_seconds,
        }


class StaticTokenProvider:
    """Uses RMF_API_TOKEN and never mints a replacement."""

    def __init__(self, token: str):
        self._token = token.strip()
        self.generation = 1 if self._token else 0

    async def get_token(self) -> str:
        if not self._token:
            raise TokenUnavailable("RMF_API_TOKEN is not set")
        return self._token

    async def refresh_if_needed(self) -> str:
        return await self.get_token()

    async def refresh_after_unauthorized(self, rejected_generation: int) -> str:
        raise TokenUnavailable("static mode does not mint a replacement token")

    async def start(self) -> None:
        return None

    async def close(self) -> None:
        return None

    def status(self) -> dict:
        return {
            "auth_mode": "static",
            "token_ready": bool(self._token),
            "issued_at": None,
            "expires_at": None,
            "next_rotation_at": None,
            "generation": self.generation,
            "last_refresh_result": None,
            "last_refresh_reason": None,
            "last_error": None if self._token else "RMF_API_TOKEN is not set",
            "algorithm": None,
            "ttl_seconds": None,
            "rotate_interval_seconds": None,
            "expiry_skew_seconds": None,
        }


class UnavailableTokenProvider:
    def __init__(self, message: str):
        self._message = message
        self.generation = 0

    async def get_token(self) -> str:
        raise TokenUnavailable(self._message)

    async def refresh_if_needed(self) -> str:
        raise TokenUnavailable(self._message)

    async def refresh_after_unauthorized(self, rejected_generation: int) -> str:
        raise TokenUnavailable(self._message)

    async def start(self) -> None:
        return None

    async def close(self) -> None:
        return None

    def status(self) -> dict:
        return {
            "auth_mode": "local_jwt",
            "token_ready": False,
            "issued_at": None,
            "expires_at": None,
            "next_rotation_at": None,
            "generation": 0,
            "last_refresh_result": "config_error",
            "last_refresh_reason": None,
            "last_error": self._message,
            "algorithm": None,
            "ttl_seconds": None,
            "rotate_interval_seconds": None,
            "expiry_skew_seconds": None,
        }


def build_token_provider(environ: dict | None = None):
    """static keeps RMF_API_TOKEN. local_jwt never falls back to that token."""
    env = os.environ if environ is None else environ
    mode = env.get("RMF_AUTH_MODE", "static").strip().lower() or "static"
    if mode == "static":
        return StaticTokenProvider(env.get("RMF_API_TOKEN", ""))
    if mode != "local_jwt":
        return UnavailableTokenProvider("RMF_AUTH_MODE must be static or local_jwt")
    try:
        return RmfTokenProvider(
            secret=env.get("RMF_JWT_SECRET", ""),
            issuer=env.get("RMF_JWT_ISSUER", ""),
            audience=env.get("RMF_JWT_AUDIENCE", ""),
            username=env.get("RMF_JWT_USERNAME", ""),
            algorithm=env.get("RMF_JWT_ALGORITHM", "HS256").strip() or "HS256",
            ttl_seconds=float(env.get("RMF_JWT_TTL_SECONDS", "3600")),
            rotate_interval_seconds=float(env.get("RMF_JWT_ROTATE_INTERVAL_SECONDS", "2700")),
            expiry_skew_seconds=float(env.get("RMF_JWT_EXPIRY_SKEW_SECONDS", "60")),
            retry_interval_seconds=float(env.get("RMF_AUTH_RETRY_SEC", "30")),
        )
    except (TokenConfigError, ValueError) as ex:
        return UnavailableTokenProvider(str(ex))
