"""Guard-supremacy tests for the Rocky-MCP harness (MCP_CONTRACT_v0.md).

Fast tests exercise the contract THROUGH a real in-process MCP
client-server session (mcp.shared.memory) against the MockBackend.
The slow test binds the same contract to real MuJoCo physics.

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
                         "scan_summary", "status"}
        for absent in ("look", "map_query", "patrol", "dock"):
            assert absent not in tools


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
