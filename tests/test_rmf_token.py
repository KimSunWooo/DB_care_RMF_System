import asyncio

import httpx
import pytest

from app.rmf_client import RmfClient, RmfClientError
from app.rmf_token import (
    Clock,
    RmfTokenProvider,
    StaticTokenProvider,
    TokenConfigError,
    TokenUnavailable,
    build_token_provider,
)


class FakeClock(Clock):
    def __init__(self):
        self.wall_now = 1_700_000_000.0
        self.mono_now = 1_000.0

    def wall(self) -> float:
        return self.wall_now

    def mono(self) -> float:
        return self.mono_now

    def advance(self, seconds: float) -> None:
        self.wall_now += seconds
        self.mono_now += seconds


def provider(clock: FakeClock, **overrides) -> RmfTokenProvider:
    values = dict(secret="test-secret", issuer="stub", audience="rmf_api_server", username="admin",
                  ttl_seconds=3600, rotate_interval_seconds=2700, expiry_skew_seconds=60,
                  retry_interval_seconds=30, clock=clock)
    values.update(overrides)
    return RmfTokenProvider(**values)


def test_config_rules_reject_bad_intervals():
    clock = FakeClock()
    with pytest.raises(TokenConfigError):
        provider(clock, rotate_interval_seconds=3600)
    with pytest.raises(TokenConfigError):
        provider(clock, secret="")
    with pytest.raises(TokenConfigError):
        provider(clock, algorithm="none")
    with pytest.raises(TokenConfigError):
        provider(clock, expiry_skew_seconds=-1)


def test_first_issue_then_reuse_until_rotation():
    clock = FakeClock()
    item = provider(clock)
    first = asyncio.run(item.get_token())
    second = asyncio.run(item.get_token())
    assert first == second
    assert item.generation == 1
    clock.advance(2699)
    assert asyncio.run(item.get_token()) == first
    clock.advance(2)
    rotated = asyncio.run(item.get_token())
    assert rotated != first
    assert item.generation == 2
    assert item.status()["last_refresh_reason"] == "rotate"
    assert "test-secret" not in str(item.status())


def test_skew_and_large_time_jump_refresh():
    clock = FakeClock()
    item = provider(clock)
    original = asyncio.run(item.get_token())
    clock.advance(3600 - 60)
    refreshed = asyncio.run(item.get_token())
    assert refreshed != original
    assert item.status()["last_refresh_reason"] == "expiry_skew"
    current = asyncio.run(item.get_token())
    clock.advance(10_000)
    jumped = asyncio.run(item.get_token())
    assert jumped != current
    assert item.generation == 3


def test_concurrent_callers_issue_once():
    clock = FakeClock()
    item = provider(clock)

    async def run():
        results = await asyncio.gather(*[item.get_token() for _ in range(12)])
        return results

    results = asyncio.run(run())
    assert len(set(results)) == 1
    assert item.generation == 1


def test_old_unauthorized_generation_does_not_issue_again():
    clock = FakeClock()
    item = provider(clock)
    asyncio.run(item.get_token())
    clock.advance(2700)
    asyncio.run(item.refresh_if_needed())
    assert item.generation == 2
    before = item.generation
    asyncio.run(item.refresh_after_unauthorized(1))
    assert item.generation == before


def test_failed_rotation_keeps_valid_token_and_missing_token_errors():
    clock = FakeClock()
    calls = {"n": 0}

    def signer(payload):
        calls["n"] += 1
        if calls["n"] > 1:
            raise RuntimeError("sign failed")
        return f"token-{calls['n']}"

    item = provider(clock, signer=signer)
    first = asyncio.run(item.get_token())
    clock.advance(2700)
    kept = asyncio.run(item.refresh_if_needed())
    assert kept == first
    assert item.status()["last_refresh_result"] == "failed"
    assert item.generation == 1
    clock.advance(item.retry_interval_seconds)
    asyncio.run(item.refresh_if_needed())
    assert item.generation == 1

    def boom(_payload):
        raise RuntimeError("no token")

    broken = provider(FakeClock(), signer=boom)
    with pytest.raises(TokenUnavailable):
        asyncio.run(broken.get_token())
    with pytest.raises(TokenUnavailable):
        asyncio.run(broken.get_token())


def test_background_rotation_and_single_task():
    clock = FakeClock()
    item = provider(clock, ttl_seconds=90, rotate_interval_seconds=30, expiry_skew_seconds=10,
                    retry_interval_seconds=5)
    sleeps = []

    async def sleeper(seconds):
        sleeps.append(seconds)
        if len(sleeps) >= 2:
            item._closed = True
            return
        clock.advance(seconds)

    item._sleeper = sleeper

    async def run():
        await item.start()
        await item.start()
        await item._task
        await item.close()

    asyncio.run(run())
    assert item.generation >= 2
    assert sleeps[0] == pytest.approx(30)
    assert item._task is None


def test_static_mode_and_local_config_error_do_not_swap():
    static = build_token_provider({"RMF_AUTH_MODE": "static", "RMF_API_TOKEN": "manual-token"})
    assert asyncio.run(static.get_token()) == "manual-token"
    assert static.status()["auth_mode"] == "static"
    with pytest.raises(TokenUnavailable):
        asyncio.run(static.refresh_after_unauthorized(1))
    broken = build_token_provider({"RMF_AUTH_MODE": "local_jwt"})
    assert broken.status()["auth_mode"] == "local_jwt"
    assert broken.status()["token_ready"] is False
    with pytest.raises(TokenUnavailable):
        asyncio.run(broken.get_token())


def _client(handler, provider) -> RmfClient:
    return RmfClient("http://rmf.test", provider=provider, transport=httpx.MockTransport(handler))


class CountingProvider:
    def __init__(self):
        self.generation = 1
        self.tokens = {1: "old-token", 2: "new-token"}
        self.refreshes = 0

    async def get_token(self):
        return self.tokens[self.generation]

    def status(self):
        return {"auth_mode": "local_jwt", "token_ready": True, "expires_at": None}

    async def refresh_after_unauthorized(self, rejected_generation: int):
        self.refreshes += 1
        if rejected_generation == self.generation:
            self.generation = 2
        return self.tokens[self.generation]


def test_get_unauthorized_retries_once_with_new_token():
    provider_item = CountingProvider()
    seen = []

    def handler(request: httpx.Request) -> httpx.Response:
        seen.append(request.headers.get("authorization"))
        if request.headers.get("authorization") == "Bearer old-token":
            return httpx.Response(401, json={"detail": "Not authenticated"})
        return httpx.Response(200, json=[])

    body = asyncio.run(_client(handler, provider_item).get_json("/fleets"))
    assert body == []
    assert provider_item.refreshes == 1
    assert seen == ["Bearer old-token", "Bearer new-token"]


def test_second_unauthorized_does_not_refresh_again():
    provider_item = CountingProvider()

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(401, json={"detail": "Not authenticated"})

    with pytest.raises(RmfClientError) as caught:
        asyncio.run(_client(handler, provider_item).get_json("/fleets"))
    assert caught.value.status_code == 401
    assert provider_item.refreshes == 1


@pytest.mark.parametrize("status", [403, 500])
def test_forbidden_and_server_error_do_not_refresh(status):
    provider_item = CountingProvider()

    def handler(request: httpx.Request) -> httpx.Response:
        return httpx.Response(status, json={"detail": "no"})

    with pytest.raises(RmfClientError):
        asyncio.run(_client(handler, provider_item).get_json("/fleets"))
    assert provider_item.refreshes == 0


def test_timeout_and_post_unauthorized_do_not_refresh():
    provider_item = CountingProvider()

    def timeout(request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectTimeout("timed out")

    with pytest.raises(RmfClientError) as timed:
        asyncio.run(_client(timeout, provider_item).get_json("/fleets"))
    assert timed.value.kind == "timeout"
    assert provider_item.refreshes == 0

    def denied(request: httpx.Request) -> httpx.Response:
        return httpx.Response(401, json={"detail": "Not authenticated"})

    with pytest.raises(RmfClientError):
        asyncio.run(_client(denied, provider_item).post_json("/tasks/dispatch_task", {"type": "dispatch_task_request"}))
    assert provider_item.refreshes == 0


def test_static_provider_does_not_retry_unauthorized():
    static = StaticTokenProvider("manual-token")
    calls = {"n": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        calls["n"] += 1
        return httpx.Response(401, json={"detail": "Not authenticated"})

    with pytest.raises(RmfClientError):
        asyncio.run(_client(handler, static).get_json("/fleets"))
    assert calls["n"] == 1
