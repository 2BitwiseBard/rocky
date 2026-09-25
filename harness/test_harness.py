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
        assert tools == {"say", "gesture", "move", "goto", "stop",
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


# ------------------------------------------------ move (relative, robot frame)
@pytest.mark.asyncio
async def test_move_schema_and_doc(server):
    async with client_session(server._mcp_server) as cs:
        tools = {t.name: t for t in (await cs.list_tools()).tools}
    mv = tools["move"]
    props = mv.inputSchema["properties"]
    assert props["forward_m"]["type"] == "number" and props["left_m"]["type"] == "number"
    assert props["left_m"].get("default") == 0.0
    assert mv.inputSchema["required"] == ["forward_m"]
    assert "move(forward_m=0.3)" in mv.description and "MAP coordinates" in mv.description
    for outcome in ("arrived", "cliff", "stuck", "blocked", "timeout", "user", "preempted"):
        assert outcome in mv.description, outcome
    assert "use move instead" in tools["goto"].description


@pytest.mark.asyncio
async def test_move_walks_the_backends_goto(server, backend):
    async with client_session(server._mcp_server) as cs:
        r = await _call(cs, "move", {"forward_m": 0.1})
        assert r["stopped"] == "arrived" and r["ok"] is True
        assert r["move"]["target"] == {"x": 0.1, "y": 0.0}
        assert "no heading" in r["move"]["note"]                  # the mock is a point: forward = +x
        r = await _call(cs, "move", {"forward_m": -0.2, "left_m": 0.05})   # from where the first ended
        assert r["stopped"] == "arrived"                           # (the mock arrives within 1 cm)
        assert r["move"]["target"]["x"] == pytest.approx(r["move"]["from"]["x"] - 0.2, abs=1e-3)
        assert (r["pose"]["x"], r["pose"]["y"]) == pytest.approx((-0.1, 0.05), abs=0.011)


@pytest.mark.asyncio
async def test_move_veto_and_refusal_come_back_as_results(server, backend):
    """PLUMBING: the mock's scripted void vetoes a move exactly as it vetoes the goto."""
    async with client_session(server._mcp_server) as cs:
        r = await _call(cs, "move", {"forward_m": 30})              # 30 cm typed as 30
        assert r["ok"] is False and "METERS" in r["error"]
        assert backend.x == 0.0 and backend.mode == "idle"         # nothing walked
        r = await _call(cs, "move", {"forward_m": 1.0})
        assert r["stopped"] == "cliff" and r["ok"] is False
        assert backend.mode == "safe_stop"


@pytest.mark.asyncio
async def test_move_uses_the_heading_when_status_reports_one():
    class Turned(MockBackend):
        async def status(self):
            r = await super().status()
            r["pose"]["yaw_deg"] = 90.0                             # facing map +y
            return r
    srv = build_server(Turned())
    async with client_session(srv._mcp_server) as cs:
        r = await _call(cs, "move", {"forward_m": 0.1, "left_m": 0.05})
    assert r["move"]["target"] == {"x": -0.05, "y": 0.1} and "note" not in r["move"]
    assert r["stopped"] == "arrived"


@pytest.mark.asyncio
async def test_move_without_a_pose_does_not_walk():
    class Blind(MockBackend):
        async def status(self):
            return {"ok": False, "error": "cockpit unreachable: timed out"}
    be = Blind()
    srv = build_server(be)
    async with client_session(srv._mcp_server) as cs:
        r = await _call(cs, "move", {"forward_m": 0.1})
    assert r["ok"] is False and "pose" in r["error"] and be.x == 0.0


def test_move_target_is_pure():
    from harness.local_brain import move_target, validate_move
    assert move_target({"x": 0.0, "y": 0.0, "yaw_deg": 0.0}, 0.3) == (0.3, 0.0)
    assert move_target({"x": 0.0, "y": 0.0, "yaw_deg": 90.0}, 0.3) == (0.0, 0.3)
    assert move_target({"x": 0.0, "y": 0.0, "yaw_deg": -90.0}, 0.3, 0.1) == (0.1, -0.3)
    assert validate_move(0.3) == (0.3, 0.0, None)
    assert validate_move(2.0)[2] and validate_move(0)[2]


@pytest.mark.asyncio
async def test_local_brain_call_tool_runs_move_on_a_backend_without_one(backend):
    from harness.local_brain import call_tool
    r = await call_tool(backend, "move", {"forward_m": 0.1})
    assert r["stopped"] == "arrived" and backend.x == pytest.approx(0.1, abs=0.011)
    r = await call_tool(backend, "move", {"left": 0.1})
    assert r["ok"] is False and "bad arguments" in r["error"]


@pytest.mark.asyncio
async def test_local_brain_call_tool_runs_only_registry_tools():
    """D056 review: call_tool used to getattr ANY name a model sent, so the backends' new
    capabilities() / fetch_capabilities() (and the older live_lists) answered a made-up
    tool name with a misleading 'bad arguments' (or a blocking GET). A name outside the
    registry now gets the refusal an unknown name always got, and nothing on the backend
    runs; every registry tool still reaches the backend as before."""
    import harness.capabilities as C
    from harness.local_brain import call_tool
    ran = []

    class Nosy(MockBackend):
        def capabilities(self):
            ran.append("capabilities")
            return set()

        def fetch_capabilities(self):
            ran.append("fetch_capabilities")

        def live_lists(self):
            ran.append("live_lists")
            return [], []

    be = Nosy()
    for name in ("capabilities", "fetch_capabilities", "live_lists", "__init__", "_tool", "events", ""):
        assert await call_tool(be, name, {}) == {"ok": False, "error": f"no such tool {name}"}
    assert await call_tool(be, None, {}) == {"ok": False, "error": "no such tool None"}
    assert ran == [] and be.events == []
    # registry names: the same answers as before — run when the backend has them, refused when not
    assert (await call_tool(be, "status", {}))["ok"] is True
    assert (await call_tool(be, "say", {"word": "yes"}))["ok"] is True and be.events[-1] == ("say", "yes")
    for name in ("compose_gesture", "turn", "remember"):                   # not on a MockBackend
        assert name in C.BY_NAME and not hasattr(be, name)
        assert await call_tool(be, name, {}) == {"ok": False, "error": f"no such tool {name}"}


@pytest.mark.asyncio
async def test_move_and_goto_refuse_a_boolean_distance(server, backend):
    """Review 2026-09-25: pydantic's lax mode made move(forward_m=true) a 1 m walk
    (the cockpit refuses a boolean); the schema stays 'number'."""
    async with client_session(server._mcp_server) as cs:
        tools = {t.name: t for t in (await cs.list_tools()).tools}
        assert tools["goto"].inputSchema["properties"]["x"]["type"] == "number"
        for name, args in (("move", {"forward_m": True}), ("move", {"forward_m": 0.1, "left_m": False}),
                           ("goto", {"x": True, "y": 0.0})):
            res = await cs.call_tool(name, args)
            assert res.isError and "boolean" in res.content[0].text, (name, args)
        assert backend.x == 0.0 and backend.mode == "idle"            # nothing walked
        r = await _call(cs, "move", {"forward_m": 0.1, "left_m": None})   # null = 0, as the cockpit reads it
        assert r["stopped"] == "arrived" and r["move"]["left_m"] == 0.0
        r = await _call(cs, "goto", {"x": 0, "y": 0})                  # a JSON integer is still a number
        assert r["stopped"] == "arrived"


def _fake_cockpit(knows_move=True, yaw=90.0):
    """A cockpit's /api/tool/<name>, recorded; an older one does not know move."""
    sent = []

    async def tool(name, /, **args):
        sent.append((name, args))
        if name == "move":
            if not knows_move:
                return {"ok": False, "error": "no such tool 'move'", "hint": "say, gesture, goto"}
            return {"ok": True, "stopped": "arrived", "move": dict(args)}
        if name == "status":
            return {"ok": True, "pose": {"x": 0.0, "y": 0.0, "yaw_deg": yaw}, "mode": "idle"}
        if name == "goto":
            return {"ok": True, "stopped": "arrived", "pose": {"x": args["x"], "y": args["y"]}}
        return {"ok": True}
    return tool, sent


@pytest.mark.asyncio
async def test_move_runs_inside_a_cockpit_backend():
    """The cockpit reads the pose and starts the goto in one place (its stop check
    sits between them); over MCP, status then goto left a window a stop could miss."""
    from harness.cockpit_backend import CockpitBackend
    from harness.local_brain import move_via
    be = CockpitBackend("http://127.0.0.1:1")
    be._tool, sent = _fake_cockpit()
    r = await move_via(be, 0.3)
    assert sent == [("move", {"forward_m": 0.3, "left_m": 0.0})] and r["stopped"] == "arrived"
    r = await move_via(be, 30)                                         # refused here, never sent
    assert r["ok"] is False and "METERS" in r["error"] and len(sent) == 1
    old = CockpitBackend("http://127.0.0.1:1")                         # a cockpit from before move
    old._tool, sent = _fake_cockpit(knows_move=False)
    r = await move_via(old, 0.3)
    assert [n for n, _a in sent] == ["move", "status", "goto"]
    assert sent[-1][1] == pytest.approx({"x": 0.0, "y": 0.3}, abs=1e-3)   # facing +y: forward is +y
    assert r["stopped"] == "arrived" and r["move"]["target"] == {"x": 0.0, "y": 0.3}


@pytest.mark.asyncio
async def test_auto_backend_moves_one_robot():
    """AutoBackend re-resolves per call: the move goes whole to the cockpit while one
    answers, else whole to the in-process fallback (never the pose of one, the goto
    of the other), and says which."""
    from harness.cockpit_backend import AutoBackend
    from harness.local_brain import move_via
    alive = {"v": False}
    fb = MockBackend()
    auto = AutoBackend(url="http://127.0.0.1:1", make_fallback=lambda: fb, ttl=0.0,
                       alive=lambda url: alive["v"])
    auto.cockpit._tool, sent = _fake_cockpit()
    r = await move_via(auto, 0.1)
    assert r["backend"] == "sim" and r["stopped"] == "arrived" and sent == []
    assert fb.x == pytest.approx(0.1, abs=0.011)
    alive["v"] = True
    r = await move_via(auto, 0.2)
    assert r["backend"] == "cockpit" and sent == [("move", {"forward_m": 0.2, "left_m": 0.0})]
    srv = build_server(auto)
    async with client_session(srv._mcp_server) as cs:
        r = await _call(cs, "move", {"forward_m": 0.1, "left_m": 0.05})
    assert r["backend"] == "cockpit" and sent[-1] == ("move", {"forward_m": 0.1, "left_m": 0.05})



# ------------------------- D056: build_tools is a view of the tool registry
# build_tools' output on 2026-09-25, BEFORE harness/capabilities.py existed: the sha256 of
# its canonical JSON (sort_keys, compact) for every variant the code base calls, taken
# from the registry task's snapshot files. They pin today's lists with no file needed.
_REP_GESTURES = ["wave", "sit", "turn_in_place", "sidestep", "look_around", "bow", "shake", "point_there"]
_REP_LEXICON = ["greeting", "yes", "no", "acknowledge", "found_it", "thinking", "error"]
_BUILD_TOOLS_BEFORE = {
    "local_brain.TOOLS": "7efe2065850cf00fe3cf5ae52b47f95cba80c79e36a89848be3aaee7a60629a1",
    "cockpit_brains.TOOLS": "f932d346fe16bbc9274d9a1fd1501b0837c96aa797c1eed2972d44eb0660c4eb",
    "rep look extra": "469a6144f7308046c82e4007d205fe0b7caa5d9b47d6656bb758c4e451833fd8",
    "rep no-look extra": "0f6585445b0ec3f5df6c003c0b1a9074904f5550e477628d63c25c0c7e20734f",
    "rep look": "b9dc26d6eb986597da7cd6087d34ab5dd44968222480d2ff23367fcbca9032c2",
    "rep signed sidestep": "2e3bd9f251125fd2e01433ccb7470867564a4ccc1ea18129d64e762a560566ad",
}
# the snapshot file's key for each (the file is a one-off record; the test skips without it)
_SNAPSHOT_KEYS = {
    "local_brain.TOOLS": "local_brain.TOOLS (build_tools(GESTURES, CHORD_WORDS))",
    "cockpit_brains.TOOLS": "cockpit_brains.TOOLS (build_tools(look=True, extra=EXTRA_TOOLS))",
    "rep no-look extra": "build_tools(GESTURES_rep, LEXICON_rep, look=False, extra=EXTRA_TOOLS)",
    "rep look": "build_tools(GESTURES_rep, LEXICON_rep, look=True)",
    "rep signed sidestep": "build_tools(signed=('sidestep',)) rep",
}
_SNAP_DIR = os.environ.get(
    "ROCKY_D056_SNAPSHOTS",
    "/tmp/claude-1000/-home-bitwisebard-Development-rocky/"
    "617aee55-a110-4e95-989e-f8423ee0b4fa/scratchpad/d056")


def _canon_sha(obj):
    import hashlib
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False).encode("utf-8")).hexdigest()


def _build_tools_variants():
    import harness.local_brain as lb
    import sim.cockpit_brains as cb
    from harness.backend import SIGNED
    g, w = _REP_GESTURES, _REP_LEXICON
    return {
        "local_brain.TOOLS": lb.TOOLS,
        "cockpit_brains.TOOLS": cb.TOOLS,
        "rep look extra": lb.build_tools(g, w, signed=SIGNED, look=True, extra=cb.EXTRA_TOOLS),
        "rep no-look extra": lb.build_tools(g, w, look=False, extra=cb.EXTRA_TOOLS),
        "rep look": lb.build_tools(g, w, look=True),
        "rep signed sidestep": lb.build_tools(g, w, signed=("sidestep",)),
    }


def test_build_tools_is_unchanged_by_the_registry():
    got = _build_tools_variants()
    assert set(got) == set(_BUILD_TOOLS_BEFORE)
    for k, sha in _BUILD_TOOLS_BEFORE.items():
        assert _canon_sha(got[k]) == sha, k


def test_build_tools_equals_the_pre_registry_snapshot_byte_for_byte():
    path = os.path.join(_SNAP_DIR, "snapshot_openai_tools_variants.json")
    if not os.path.exists(path):
        pytest.skip(f"no D056 snapshot at {path} (the hashes above still pin it)")
    with open(path, encoding="utf-8") as f:
        before = json.load(f)
    got = _build_tools_variants()
    for k, key in _SNAPSHOT_KEYS.items():
        assert json.dumps(got[k]) == json.dumps(before[key]), k        # key order too
    with open(os.path.join(_SNAP_DIR, "snapshot_openai_tools.json"), encoding="utf-8") as f:
        assert json.dumps(got["rep look extra"]) == json.dumps(json.load(f)["tools"])


def test_build_tools_keeps_its_signature_and_the_texts_stay_importable():
    import inspect
    import harness.capabilities as C
    import harness.local_brain as lb
    sig = inspect.signature(lb.build_tools)
    assert [(p.name, p.default) for p in sig.parameters.values()] == [
        ("gestures", None), ("lexicon", None), ("signed", ("turn_in_place", "sidestep")),
        ("look", False), ("extra", ())]
    assert (lb.GOTO_DOC, lb.MOVE_DOC, lb.GOTO_REACH_M, lb.MOVE_MAX_M) == \
        (C.GOTO_DOC, C.MOVE_DOC, C.GOTO_REACH_M, C.MOVE_MAX_M)
    # build_tools makes the tools every backend has (+ look); `extra` is appended as given
    marker = {"type": "function", "function": {"name": "x_extra", "description": "d", "parameters": {}}}
    tools = lb.build_tools(["wave"], ["yes"], look=True, extra=[marker])
    assert [t["function"]["name"] for t in tools] == list(lb.base_tool_names(look=True)) + ["x_extra"]
    assert tools[-1] is marker
    assert lb.base_tool_names() == ("say", "gesture", "move", "goto", "stop", "scan_summary", "status",
                                    "list_gestures")
    # the local brain's list is the registry's OpenAI surface for a plain backend
    assert lb.build_tools(["wave"], ["yes"]) == C.to_openai_tools(C.build(["wave"], ["yes"]))


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
