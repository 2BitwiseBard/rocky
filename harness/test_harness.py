"""Contract tests for the Rocky-MCP harness (MCP_CONTRACT_v0.md).

Fast tests exercise the contract THROUGH a real in-process MCP
client-server session (mcp.shared.memory) against the MockBackend. They
are PLUMBING tests: the mock's guard is a scripted x threshold, so they
prove the server carries vetoes, preemption and refusals through as
results — not that the robot is safe. Guard supremacy on real physics is
the slow test (and sim/run_cliff*.py).

    pytest harness/test_harness.py -m "not slow"    # milliseconds-fast set
    pytest harness/test_harness.py -m slow          # the physics proof (~2 min)
"""
import asyncio
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from harness.backend import MockBackend                     # noqa: E402
from harness.server import build_server                     # noqa: E402

from mcp.shared.memory import (                             # noqa: E402
    create_connected_server_and_client_session as client_session,
)


def result_json(res):
    """Tool results carry the dict as JSON text content."""
    assert not res.isError, res
    return json.loads(res.content[0].text)


async def _call(cs, name, args=None):
    return result_json(await cs.call_tool(name, args or {}))


# ---------------------------------------------------------------- fixtures
@pytest.fixture
def backend():
    return MockBackend()


@pytest.fixture
def server(backend):
    return build_server(backend)


# ---------------------------------------------------- invariant 1: guard
@pytest.mark.asyncio
async def test_goto_into_void_is_vetoed_not_raised(server):
    """PLUMBING: the mock's scripted void comes back as a result, not a raise."""
    async with client_session(server._mcp_server) as cs:
        r = await _call(cs, "goto", {"x": 1.0, "y": 0.0})
        assert r["stopped"] == "cliff"
        assert r["ok"] is False
        # and the robot is SAFE: short of the edge, in safe_stop
        st = await _call(cs, "status")
        assert st["pose"]["x"] < 0.35
        assert st["mode"] == "safe_stop"
        assert any(e[0] == "void" for e in st["last_events"])


@pytest.mark.asyncio
async def test_safe_goto_arrives(server):
    async with client_session(server._mcp_server) as cs:
        r = await _call(cs, "goto", {"x": 0.10, "y": 0.05})
        assert r["stopped"] == "arrived" and r["ok"] is True


# --------------------------------------------- invariant 3: honest async
@pytest.mark.asyncio
async def test_stop_resolves_goto_with_user(server, backend):
    """PLUMBING: stop during a (mock) goto resolves it as stopped='user'."""
    async with client_session(server._mcp_server) as cs:
        goto_task = asyncio.create_task(
            cs.call_tool("goto", {"x": 0.10, "y": 0.0}))
        await asyncio.sleep(0.02)                 # let it start walking
        r_stop = await _call(cs, "stop")
        assert r_stop["ok"] is True
        r_goto = result_json(await goto_task)
        assert r_goto["stopped"] == "user"
        assert backend.mode == "safe_stop"


# -------------------------------------------- invariant 2: single writer
@pytest.mark.asyncio
async def test_second_goto_preempts_first(backend):
    # exercised at the backend layer: a session serializes tool calls, so
    # true preemption arrives via a second client/loop in deployment
    t1 = asyncio.create_task(backend.goto(0.10, 0.0))
    await asyncio.sleep(0.02)
    r2 = await backend.goto(-0.05, 0.0)
    r1 = await t1
    assert r1["stopped"] == "preempted"
    assert r2["stopped"] == "arrived"


# ------------------------------------------------------- tool hygiene
@pytest.mark.asyncio
async def test_unknown_word_refused_with_lexicon(server):
    async with client_session(server._mcp_server) as cs:
        r = await _call(cs, "say", {"word": "hello"})
        assert r["ok"] is False and "acknowledge" in r["hint"]
        r = await _call(cs, "say", {"word": "curious_question"})
        assert r["ok"] is True


@pytest.mark.asyncio
async def test_gesture_busy_while_walking(server, backend):
    """PLUMBING: the mock's busy rule reaches the client as a result."""
    async with client_session(server._mcp_server) as cs:
        goto_task = asyncio.create_task(
            cs.call_tool("goto", {"x": 0.10, "y": 0.0}))
        await asyncio.sleep(0.02)
        r = await _call(cs, "gesture", {"name": "jazz_hands"})
        assert r["ok"] is False and r["error"] == "busy"
        await _call(cs, "stop")
        result_json(await goto_task)
        r = await _call(cs, "gesture", {"name": "fist_bump"})
        assert r["ok"] is True


@pytest.mark.asyncio
async def test_v1_tools_are_absent_not_stubbed(server):
    async with client_session(server._mcp_server) as cs:
        tools = {t.name for t in (await cs.list_tools()).tools}
        assert tools == {"say", "gesture", "goto", "stop",
                         "scan_summary", "status", "list_gestures"}
        for absent in ("look", "find_object", "map_query", "patrol", "dock"):
            assert absent not in tools


# ---------------------------------------------------------- D052 additions
@pytest.mark.asyncio
async def test_tool_docs_are_live_and_complete(server):
    class Live(MockBackend):
        def live_lists(self):
            return ["wave", "point_there"], ["greeting", "my_custom_word"]
    srv = build_server(Live())
    async with client_session(srv._mcp_server) as cs:
        docs = {t.name: t.description for t in (await cs.list_tools()).tools}
    assert "point_there" in docs["gesture"] and "my_custom_word" in docs["say"]
    for outcome in ("arrived", "cliff", "stuck", "blocked", "timeout", "user", "preempted"):
        assert outcome in docs["goto"], outcome
    import harness.server as hs
    assert "deferred" not in hs.__doc__.lower()           # the stale v1 line is gone


@pytest.mark.asyncio
async def test_signed_gesture_and_list_gestures(server):
    async with client_session(server._mcp_server) as cs:
        r = await _call(cs, "gesture", {"name": "turn_in_place", "direction": "right"})
        assert r == {"ok": True, "gesture": "turn_in_place", "direction": "right"}
        r = await _call(cs, "gesture", {"name": "wave", "direction": "right"})
        assert r["ok"] is False
        r = await _call(cs, "list_gestures")
        assert "turn_in_place" in r["gestures"] and r["signed"] == ["turn_in_place", "sidestep"]


@pytest.mark.asyncio
async def test_auto_backend_re_resolves_per_call():
    """ROCKY_BACKEND=auto used to pick once at start: a cockpit started later was
    never used. Now each call re-checks (TTL 0 here) and uses what answers."""
    from harness.cockpit_backend import AutoBackend
    alive = {"v": False}
    fb = MockBackend()
    auto = AutoBackend(url="http://127.0.0.1:1", make_fallback=lambda: fb, ttl=0.0,
                       alive=lambda url: alive["v"])

    async def fake_cockpit_tool(tool, /, **args):
        return {"ok": True, "via": "cockpit", "tool": tool, "args": args}
    auto.cockpit._tool = fake_cockpit_tool
    r = await auto.status()
    assert r["backend"] == "sim" and r["mode"] == "idle"
    r = await auto.look()
    assert r["ok"] is False and "cockpit" in r["error"]
    alive["v"] = True
    r = await auto.status()
    assert r["via"] == "cockpit" and r["backend"] == "cockpit"
    r = await auto.gesture("turn_in_place", direction="right")
    assert r["via"] == "cockpit" and r["args"] == {"name": "turn_in_place", "direction": "right"}


@pytest.mark.asyncio
async def test_auto_stop_goes_to_a_seen_cockpit_even_when_the_probe_says_dead():
    """V2 review repro: a busy cockpit (a 1 s console `check`) failed the 1 s
    /api/state probe, and stop() went to the in-process fallback, answered ok,
    and the cockpit robot kept walking."""
    from harness.cockpit_backend import AutoBackend
    alive = {"v": True}
    fb = MockBackend()
    auto = AutoBackend(url="http://127.0.0.1:1", make_fallback=lambda: fb, ttl=0.0,
                       alive=lambda url: alive["v"])
    sent = []
    answer = {"r": {"ok": True, "mode": "safe_stop"}}

    async def fake_cockpit_tool(tool, /, **args):
        sent.append(tool)
        return dict(answer["r"])
    auto.cockpit._tool = fake_cockpit_tool
    assert (await auto.status())["backend"] == "cockpit"          # the cockpit has been seen
    alive["v"] = False                                             # now it looks dead (busy)
    r = await auto.stop()
    assert sent[-1] == "stop" and r["ok"] is True and r["backend"] == "cockpit"
    assert auto._fallback is None                                  # the fallback was not built for a stop
    answer["r"] = {"ok": False, "error": "cockpit unreachable: timed out"}
    n = len(sent)
    r = await auto.stop(retries=1)
    assert len(sent) - n == 2 and r["ok"] is False and "may still be moving" in r["error"]
    assert (await auto.goto(0.1, 0.0))["backend"] == "sim"         # motion now runs on the fallback ...
    answer["r"] = {"ok": False, "error": "cockpit unreachable: refused", "gone": True}
    r = await auto.stop(retries=0)                                 # ... and a gone cockpit hands the stop to it
    assert r["ok"] is True and r["backend"] == "sim" and "gone" in r["cockpit"]


def test_cockpit_alive_uses_ping_and_counts_a_500_as_alive(monkeypatch):
    import httpx
    from harness import cockpit_backend as cb
    codes = {}

    class _R:
        def __init__(self, code):
            self.status_code = code

    def get(url, timeout=None):
        return _R(codes.get(url.rsplit("/api", 1)[1], 404))
    monkeypatch.setattr(httpx, "get", get)
    codes.update({"/ping": 200})
    assert cb.cockpit_alive("http://x")
    codes.update({"/ping": 503})
    assert not cb.cockpit_alive("http://x")                        # a dead sim thread
    codes.update({"/ping": 500})
    assert cb.cockpit_alive("http://x")                            # degraded, still takes a stop
    codes.clear()
    codes.update({"/state": 200})                                  # an older cockpit without /api/ping
    assert cb.cockpit_alive("http://x")


@pytest.mark.asyncio
async def test_cockpit_proxy_gesture_passes_its_name():
    """Regression: CockpitBackend._tool(name, **args) collided with gesture's
    own `name` argument, so every proxied gesture raised TypeError."""
    from harness.cockpit_backend import CockpitBackend
    be = CockpitBackend("http://127.0.0.1:1")
    sent = []

    class _R:
        def json(self):
            return {"ok": True}

    async def post(url, json=None):
        sent.append((url, json))
        return _R()
    be.client.post = post
    assert (await be.gesture("wave")) == {"ok": True}
    assert sent[-1] == ("http://127.0.0.1:1/api/tool/gesture", {"name": "wave"})


# --------------------------------------- the physics proof (slow, honest)
@pytest.mark.slow
@pytest.mark.asyncio
async def test_sim_goto_into_void_stops_on_real_physics():
    from harness.sim_backend import SimBackend
    be = SimBackend()
    server = build_server(be)
    async with client_session(server._mcp_server) as cs:
        r = await _call(cs, "goto", {"x": 1.0, "y": 0.0})
        assert r["stopped"] == "cliff", r
        st = await _call(cs, "status")
        assert st["pose"]["x"] < 0.35          # body short of the edge
        assert st["mode"] == "safe_stop"
