"""D056: the MCP server's tool list comes from the registry and follows the robot.

build_server registers harness.capabilities.to_mcp_specs(...) for the backend's
capabilities; LiveTools re-reads the backend's capabilities snapshot on a stale
tools/list and a watcher sends notifications/tools/list_changed when the list moved.
The cockpit here is FakeCockpit: its GET routes served in-process through a
monkeypatched httpx.get (no socket is opened; nothing ever reaches :8765).

F3: the D057 place tools come and go with the cockpit's place recognition — FakeCockpit's
`recognize` is the cockpit's switch (its snapshot then lists `places`, as sim/cockpit.py's
_caps_current does), and the next tools/list (or watcher tick) follows it.

    MUJOCO_GL=egl .venv/bin/python -m pytest harness/test_server_live.py -q
"""
import json
import os
import sys

import anyio
import httpx
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import harness.capabilities as C                                               # noqa: E402
from harness.backend import CHORD_WORDS, GESTURES, SIGNED, MockBackend           # noqa: E402
from harness.cockpit_backend import AutoBackend, CockpitBackend                  # noqa: E402
from harness.server import LiveTools, build_server, runnable                     # noqa: E402
from harness.test_capabilities import (ADDED_D057, GOLDEN, GOLDEN_D057, REP_GESTURES,  # noqa: E402
                                       REP_LEXICON, _rows_from_mcp, _rows_from_specs, _sha, rep_caps)

from mcp import types                                                            # noqa: E402
from mcp.shared.memory import (                                                  # noqa: E402
    create_connected_server_and_client_session as client_session,
)

URL = "http://fake-cockpit.invalid"


class _R:
    def __init__(self, code, body):
        self.status_code = code
        self._body = body

    def json(self):
        if self._body is None:
            raise json.JSONDecodeError("Expecting value", "Not Found", 0)
        return self._body


class FakeCockpit:
    """A cockpit's GET routes, in-process. /api/capabilities answers what a D056 cockpit
    answers (harness.capabilities.build of its lists, every capability on); `modern=False`
    is a cockpit from before D056 (404 there, the two list routes instead). `recognize`:
    the cockpit's place recognition (D057) — on, its snapshot lists `places` and the four
    place tools, exactly as sim/cockpit.py's _caps_current builds it."""

    def __init__(self, gestures=REP_GESTURES, lexicon=REP_LEXICON, modern=True):
        self.gestures, self.lexicon = list(gestures), list(lexicon)
        self.envelope = None                 # None: the registry's default envelope
        self.modern = modern
        self.recognize = False               # D057 place recognition (POST /api/awareness {recognize})
        self.down = False                    # connection refused
        self.status = 200                    # /api/capabilities status while up
        self.garbage = False                 # /api/capabilities answers something that is not a snapshot
        self.lists_status = 200              # the two pre-D056 list routes
        self.gets = []
        self.foreign = []                    # any other URL (must stay empty)

    def snapshot(self):
        return C.build(self.gestures, self.lexicon, list(SIGNED), has_eye=True, has_memory=True,
                       is_cockpit=True, envelope=self.envelope, has_places=self.recognize)

    def get(self, url, timeout=None):
        if not url.startswith(URL):
            self.foreign.append(url)
            raise httpx.ConnectError(f"test: {url} is not the fake cockpit")
        path = url[len(URL):]
        self.gets.append(path)
        if self.down:
            raise httpx.ConnectError("test: connection refused")
        if path == "/api/ping":
            return _R(200, {"ok": True})
        if path == "/api/capabilities" and self.modern:
            if self.garbage:
                return _R(200, ["not", "a", "snapshot"])
            return _R(self.status, self.snapshot() if self.status == 200 else {"error": "boom"})
        if path == "/api/gesture/list":
            return _R(self.lists_status, {"all": list(self.gestures)} if self.lists_status == 200 else None)
        if path == "/api/chord/list":
            ok = self.lists_status == 200
            return _R(self.lists_status, {"lexicon": list(self.lexicon)} if ok else None)
        return _R(404, None)

    def n(self, path="/api/capabilities"):
        return sum(1 for p in self.gets if p == path)


async def _no_post(name, /, **args):
    """CockpitBackend._tool stand-in: the fake has no POST routes (and nothing may leave the process)."""
    return {"ok": True, "tool": name, "args": args}


@pytest.fixture
def fake(monkeypatch):
    fc = FakeCockpit()
    monkeypatch.setattr(httpx, "get", fc.get)
    yield fc
    assert fc.foreign == [], fc.foreign


class FakeSession:
    def __init__(self, fail=False):
        self.sent = 0
        self.fail = fail

    async def send_tool_list_changed(self):
        if self.fail:
            raise anyio.ClosedResourceError()
        self.sent += 1


def _dump(tools):
    return [t.model_dump(mode="json", exclude_none=True, by_alias=True) for t in tools]


async def _list(cs):
    return {t.name: t for t in (await cs.list_tools()).tools}


def _stale(srv):
    srv.live.fetched_at = -1e9            # the next tools/list re-reads (instead of sleeping ttl_s)


# ---------------------------------------------------------------- the list = the registry = today
async def test_cockpit_tool_list_equals_the_snapshot(fake):
    srv = build_server(CockpitBackend(URL))
    async with client_session(srv._mcp_server) as cs:
        tools = (await cs.list_tools()).tools
        caps = cs.get_server_capabilities()
    rows = _dump(tools)
    # names, order, descriptions, input schemas, annotations: today's full-backend snapshot
    assert [t.name for t in tools] == [s["name"] for s in C.to_mcp_specs(rep_caps())]
    assert _rows_from_mcp(rows) == _rows_from_specs(C.to_mcp_specs(rep_caps()))
    assert _sha(_rows_from_mcp(rows)) == GOLDEN["mcp_rep"]
    assert all("outputSchema" not in r for r in rows)          # unstructured dict results, as before
    # the server's snapshot IS the cockpit's (same lists, same capabilities -> same version)
    assert srv.live.version == fake.snapshot()["version"]
    assert caps.tools.listChanged is True                      # a cockpit's list can move: declared
    assert srv.instructions == C.MCP_INSTRUCTIONS


async def test_auto_backend_lists_the_cockpits_tools(fake):
    auto = AutoBackend(url=URL, make_fallback=MockBackend)
    srv = build_server(auto)
    names = [t.name for t in await srv.list_tools()]
    # all 15, registry order (F3: the place tools only while the cockpit reports recognition on)
    assert names == [t.name for t in C.REGISTRY if "mcp" in t.surfaces and "places" not in t.requires]
    assert srv.live.version == fake.snapshot()["version"]
    # no cockpit at all: still the 15 (its look / memory tools say they need the cockpit)
    fake.down = True
    srv2 = build_server(AutoBackend(url=URL, make_fallback=MockBackend))
    rows = _dump(await srv2.list_tools())
    assert _rows_from_mcp(rows) == _rows_from_specs(C.to_mcp_specs(C.fallback_caps(has_places=False)))  # canon


async def test_a_backend_without_an_eye_lists_no_look_or_find_object(fake):
    names = lambda srv: [t.name for t in srv._tool_manager.list_tools()]          # noqa: E731
    base = ["say", "gesture", "move", "goto", "stop", "scan_summary", "status", "list_gestures"]
    assert names(build_server(MockBackend())) == base

    class MemOnly(MockBackend):
        async def remember(self, note):
            return {"ok": True}

        async def where_is(self, name):
            return {"ok": True}

        async def recall(self, query="", k=5):
            return {"ok": True}

        async def go_back_to(self, name):
            return {"ok": True}

        async def forget(self, name):
            return {"ok": True}
    assert MemOnly().capabilities() == {"memory"}
    assert names(build_server(MemOnly())) == base + ["remember", "where_is", "recall", "go_back_to", "forget"]

    class BlindCockpit(CockpitBackend):                  # a cockpit that says it has no eye
        def capabilities(self):
            return {"cockpit", "memory"}
    got = names(build_server(BlindCockpit(URL)))
    assert "look" not in got and "find_object" not in got and "where_is" in got

    class LookOnly(MockBackend):                         # as before D056: find_object only where it exists
        async def look(self):
            return {"ok": True}
    assert names(build_server(LookOnly())) == base + ["look"]


# ---------------------------------------------------------------- live: a new gesture, a new word
async def test_a_new_gesture_reaches_the_next_list_tools(fake):
    srv = build_server(CockpitBackend(URL))
    v0 = srv.live.version
    async with client_session(srv._mcp_server) as cs:
        before = await _list(cs)
        assert "happy_dance" not in before["gesture"].description
        n = fake.n()
        await cs.list_tools()                                    # within ttl_s: no re-read
        assert fake.n() == n
        fake.gestures.append("happy_dance")                      # saved in the studio: a new version
        fake.lexicon.append("my_word")
        _stale(srv)
        after = await _list(cs)
        assert fake.n() == n + 1
    assert "happy_dance" in after["gesture"].description and "my_word" in after["say"].description
    assert srv.live.version != v0 and srv.live.version == fake.snapshot()["version"]
    # the kept snapshot's gesture enum (the local brains' schema) has it too; the MCP gesture
    # schema has no enum, as before D056 (a plain string, refused by the backend if unknown)
    fn = {t["function"]["name"]: t["function"] for t in C.to_openai_tools(srv.live.caps)}
    assert "happy_dance" in fn["gesture"]["parameters"]["properties"]["name"]["enum"]
    assert "my_word" in fn["say"]["parameters"]["properties"]["word"]["enum"]
    assert "enum" not in after["gesture"].inputSchema["properties"]["name"]
    # only the two texts that list them changed; every other tool is the same as before
    for name, t in after.items():
        if name not in ("gesture", "say"):
            assert t.model_dump() == before[name].model_dump(), name
    assert list(after) == list(before)                           # registry order kept after re-registration


async def test_an_older_cockpit_is_followed_through_the_list_routes(fake):
    fake.modern = False
    fake.gestures = ["wave", "point_there"]
    srv = build_server(CockpitBackend(URL))
    assert "/api/gesture/list" in fake.gets and "/api/chord/list" in fake.gets
    tools = {t.name: t for t in await srv.list_tools()}
    assert "point_there" in tools["gesture"].description
    fake.gestures.append("bow_low")
    _stale(srv)
    tools = {t.name: t for t in await srv.list_tools()}
    assert "bow_low" in tools["gesture"].description


# ---------------------------------------------------------------- list_changed
async def test_list_changed_is_sent_once_per_version_change(fake):
    srv = build_server(CockpitBackend(URL))
    live = srv.live
    live.session = s = FakeSession()
    assert await live.poll_once() is False and s.sent == 0     # nothing moved
    fake.gestures.append("happy_dance")
    assert await live.poll_once() is True and s.sent == 1
    assert await live.poll_once() is False and s.sent == 1     # same version: no second notification
    v = live.version
    fake.envelope = dict(C.default_envelope(), speed_m_s=0.03)  # a gait change: new version, same MCP texts
    assert await live.poll_once() is False and s.sent == 1
    assert live.version != v
    fake.lexicon.append("my_word")
    assert await live.poll_once() is True and s.sent == 2
    assert live.notified == 2
    # a session that cannot take it: logged, never raised; the list is still current
    live.session = FakeSession(fail=True)
    fake.gestures.append("bow_low")
    assert await live.poll_once() is False
    assert "bow_low" in {t.name: t for t in await srv.list_tools()}["gesture"].description


async def test_a_real_client_gets_one_notification_and_the_new_list(fake):
    be = CockpitBackend(URL)
    be._tool = _no_post
    srv = build_server(be)
    live = srv.live
    live.poll_s = 0.05
    got = []

    async def on_message(msg):
        if (isinstance(msg, types.ServerNotification)
                and isinstance(msg.root, types.ToolListChangedNotification)):
            got.append(msg.root.method)

    async with client_session(srv._mcp_server, message_handler=on_message) as cs:
        assert not live.watching
        await cs.list_tools()                                    # the first request starts the watcher
        assert live.watching
        await anyio.sleep(0.2)
        assert got == []                                         # polling, nothing moved
        fake.gestures.append("happy_dance")
        with anyio.fail_after(5):
            while not got:
                await anyio.sleep(0.02)
        await anyio.sleep(0.3)                                   # several more polls: still one
        assert got == ["notifications/tools/list_changed"]
        tools = await _list(cs)
        assert "happy_dance" in tools["gesture"].description
        r = await cs.call_tool("gesture", {"name": "happy_dance"})   # the backend decides whether it knows it
        assert json.loads(r.content[0].text) == {"ok": True, "tool": "gesture",
                                                 "args": {"name": "happy_dance"}}
    assert live._tg is None and not live.watching                # the watcher ended with the server run


# ---------------------------------------------------------------- failures keep the last list
async def test_a_failed_fetch_keeps_the_last_list(fake):
    fake.gestures = ["wave", "point_there"]
    be = CockpitBackend(URL)
    srv = build_server(be)
    live = srv.live
    live.session = s = FakeSession()
    v = live.version
    desc = lambda: {t.name: t for t in srv._tool_manager.list_tools()}["gesture"].description   # noqa: E731
    assert "point_there" in desc()
    fake.lists_status = 500                                        # the list routes fail too
    for broken in ("down", "500", "garbage"):
        fake.down, fake.garbage = broken == "down", broken == "garbage"
        fake.status = 500 if broken == "500" else 200
        _stale(srv)
        tools = {t.name: t for t in await srv.list_tools()}
        assert "point_there" in tools["gesture"].description, broken      # the last list, not the canon
        assert await live.poll_once() is False and s.sent == 0, broken
        assert live.version == v and live.source_ok is False, broken
    # /api/capabilities failing while the list routes answer: the lists are followed, as before D056
    fake.lists_status = 200
    fake.gestures.append("bow_low")
    assert await live.poll_once() is True and s.sent == 1 and "bow_low" in desc()
    assert live.source_ok is True
    v = live.version
    # a source that raises (not just fails) changes nothing
    fake.down = fake.garbage = False
    fake.status = 200

    def boom(*a, **k):
        raise RuntimeError("test: broken source")
    be.fetch_capabilities = boom
    assert await live.poll_once() is False and live.version == v
    del be.fetch_capabilities                                     # the real one again
    fake.gestures.append("kneel")                                 # back, and moved: followed again
    assert await live.poll_once() is True and s.sent == 2 and "kneel" in desc()
    assert fake.n("/api/capabilities") > 0


async def test_the_watcher_never_takes_the_server_down(fake):
    be = CockpitBackend(URL)
    be._tool = _no_post
    srv = build_server(be)
    srv.live.poll_s = 0.02
    calls = {"n": 0}

    def boom(*a, **k):
        calls["n"] += 1
        raise RuntimeError("test: broken source")
    async with client_session(srv._mcp_server) as cs:
        await cs.list_tools()
        be.fetch_capabilities = boom
        await anyio.sleep(0.2)
        assert calls["n"] >= 3                                    # it kept polling through the failures
        _stale(srv)
        assert "stop" in await _list(cs)                          # the server still answers
        r = await cs.call_tool("status", {})
        assert not r.isError


# ---------------------------------------------------------------- static backends, the switch
async def test_mock_is_static_and_live_can_be_switched_off(fake, monkeypatch):
    srv = build_server(MockBackend())
    assert not srv.live.live and srv.live.fetches == 0
    async with client_session(srv._mcp_server) as cs:
        await cs.list_tools()
        assert cs.get_server_capabilities().tools.listChanged is False     # as before D056
    assert srv.live.fetches == 0 and not srv.live.watching
    monkeypatch.setenv("ROCKY_MCP_LIVE", "0")
    srv = build_server(CockpitBackend(URL))
    assert srv.live.fetches == 1 and not srv.live.live                    # read once at start, as before
    fake.gestures.append("happy_dance")
    _stale(srv)
    async with client_session(srv._mcp_server) as cs:
        tools = await _list(cs)
        assert cs.get_server_capabilities().tools.listChanged is False
    assert "happy_dance" not in tools["gesture"].description and srv.live.fetches == 1


# ---------------------------------------------------------------- forwarding + capabilities
async def test_calls_forward_to_the_backend_as_before(fake):
    seen = []

    class Rec(MockBackend):
        async def look(self):
            seen.append(("look",))
            return {"ok": True}

        async def find_object(self, name, max_steps=None):
            seen.append(("find_object", name, max_steps))
            return {"ok": True}

        async def where_is(self, name):
            return {"ok": True}

        async def recall(self, query="", k=5):
            seen.append(("recall", query, k))
            return {"ok": True}

    srv = build_server(Rec())
    async with client_session(srv._mcp_server) as cs:
        await cs.call_tool("find_object", {"name": "ball"})
        await cs.call_tool("recall", {})
        await cs.call_tool("look", {})
    assert seen == [("find_object", "ball", 6), ("recall", "", 5), ("look",)]   # MCP defaults, positional

    class ToolOnly:                                   # a cockpit-like proxy with no look method of its own
        def __init__(self):
            self.sent = []

        def capabilities(self):
            return {"cockpit", "eye"}

        async def _tool(self, name, /, **args):
            self.sent.append((name, args))
            return {"ok": True, "via": name}

        async def status(self):
            return {"ok": True, "pose": {"x": 0.0, "y": 0.0}}

        async def goto(self, x, y):
            return {"ok": True}

    be = ToolOnly()
    assert runnable(be, next(s for s in C.to_mcp_specs(C.fallback_caps()) if s["name"] == "look"))
    srv = build_server(be)
    async with client_session(srv._mcp_server) as cs:
        tools = await _list(cs)
        r = await cs.call_tool("look", {})
        r2 = await cs.call_tool("find_object", {"name": "box", "max_steps": 3})
    assert "look" in tools and "where_is" not in tools
    assert json.loads(r.content[0].text) == {"ok": True, "via": "look"}
    assert be.sent == [("look", {}), ("find_object", {"name": "box", "max_steps": 3})]
    assert json.loads(r2.content[0].text)["via"] == "find_object"


def test_every_backend_says_what_it_has():
    from harness.sim_backend import SimBackend
    assert MockBackend().capabilities() == set()
    assert SimBackend.__new__(SimBackend).capabilities() == set()           # no MuJoCo needed to ask
    assert CockpitBackend("http://127.0.0.1:9").capabilities() == {"cockpit", "eye", "memory"}
    auto = AutoBackend(url="http://127.0.0.1:9", alive=lambda u: False, make_fallback=MockBackend)
    assert auto.capabilities() == {"cockpit", "eye", "memory"}               # any call may land on a cockpit
    assert auto.fetch_capabilities() is None                                 # no cockpit: nothing fetched
    # F3: `places` only from what the cockpit reported (its last snapshot's capabilities list)
    ck = CockpitBackend("http://127.0.0.1:9")
    for snap, has in ((None, False), ({"capabilities": ["cockpit", "eye", "memory"]}, False),
                      ({"capabilities": ["cockpit", "eye", "memory", "places"]}, True),
                      ({"capabilities": "places"}, False), (["places"], False), ({"gestures": []}, False)):
        ck.last_capabilities = auto.cockpit.last_capabilities = snap
        want = {"cockpit", "eye", "memory"} | ({"places"} if has else set())
        assert ck.capabilities() == want and auto.capabilities() == want, snap
    for be in (MockBackend(), CockpitBackend("http://127.0.0.1:9")):
        flags = C.backend_flags(be)
        caps = be.capabilities()
        assert flags == {"has_eye": "eye" in caps, "has_memory": "memory" in caps,
                         "is_cockpit": "cockpit" in caps}


def test_live_tools_defaults():
    lt = LiveTools(MockBackend())
    assert (lt.ttl_s, lt.poll_s) == (2.0, 3.0)
    assert lt.gestures == GESTURES and lt.lexicon == CHORD_WORDS and lt.signed == SIGNED


# ---------------------------------------------------------------- F3: the place tools
def _names(srv):
    return [t.name for t in srv._tool_manager.list_tools()]


D056_MCP = [t.name for t in C.REGISTRY if "mcp" in t.surfaces and "places" not in t.requires]


async def test_cockpit_tool_list_with_recognition_on_equals_the_snapshot(fake):
    """The GOLDEN mcp pin with the four place tools ADDED: a full-capability cockpit that reports
    `places` lists the D056 tools, unchanged, + where_am_i, name_place, places, forget_place."""
    fake.recognize = True
    srv = build_server(CockpitBackend(URL))
    async with client_session(srv._mcp_server) as cs:
        tools = (await cs.list_tools()).tools
    rows = _dump(tools)
    assert [t.name for t in tools] == D056_MCP + list(ADDED_D057)             # registry order
    assert _rows_from_mcp(rows) == _rows_from_specs(C.to_mcp_specs(rep_caps(has_places=True)))
    assert _sha(_rows_from_mcp(rows)) == GOLDEN_D057["mcp_rep"]
    assert _sha(_rows_from_mcp([r for r in rows if r["name"] not in ADDED_D057])) == GOLDEN["mcp_rep"]
    assert srv.live.version == fake.snapshot()["version"]
    assert "places" in srv.live.caps["capabilities"]
    by = {t.name: t for t in tools}
    assert by["name_place"].inputSchema["required"] == ["name"]
    assert by["forget_place"].annotations.destructiveHint is True
    assert by["where_am_i"].annotations.readOnlyHint is True


async def test_switching_recognition_on_and_off_moves_the_list(fake):
    """The cockpit's switch (POST /api/awareness {recognize}) is a new capabilities version: the next
    stale tools/list re-reads it and adds the four tools; the watcher tells the client; off removes them."""
    be = CockpitBackend(URL)
    be._tool = _no_post                                  # nothing leaves the process
    srv = build_server(be)
    live = srv.live
    async with client_session(srv._mcp_server) as cs:
        before = await _list(cs)
        assert list(before) == D056_MCP                                    # recognition off (the default)
        fake.recognize = True
        await cs.list_tools()                                              # within ttl_s: not re-read yet
        assert not set(ADDED_D057) & set(_names(srv))
        _stale(srv)
        after = await _list(cs)
        assert list(after) == D056_MCP + list(ADDED_D057)
        for name in D056_MCP:                                              # nothing else moved
            assert after[name].model_dump() == before[name].model_dump(), name
        r = await cs.call_tool("where_am_i", {})                           # registered and callable
        assert json.loads(r.content[0].text) == {"ok": True, "tool": "where_am_i", "args": {}}
    live.session = s = FakeSession()                   # (the client's session ended with its server run)
    assert await live.poll_once() is False and s.sent == 0                 # already current: no notification
    fake.recognize = False
    assert await live.poll_once() is True and s.sent == 1                  # the watcher: off -> list_changed
    assert _names(srv) == D056_MCP
    fake.recognize = True
    assert await live.poll_once() is True and s.sent == 2 and _names(srv) == D056_MCP + list(ADDED_D057)


async def test_a_real_client_is_told_when_recognition_comes_on(fake):
    be = CockpitBackend(URL)
    sent = []

    async def rec(name, /, **args):
        sent.append((name, args))
        return {"ok": True, "tool": name, "args": args}
    be._tool = rec
    srv = build_server(be)
    srv.live.poll_s = 0.05
    got = []

    async def on_message(msg):
        if (isinstance(msg, types.ServerNotification)
                and isinstance(msg.root, types.ToolListChangedNotification)):
            got.append(msg.root.method)

    async with client_session(srv._mcp_server, message_handler=on_message) as cs:
        assert not set(ADDED_D057) & set(await _list(cs))                 # starts the watcher
        fake.recognize = True
        with anyio.fail_after(5):
            while not got:
                await anyio.sleep(0.02)
        tools = await _list(cs)
        assert set(ADDED_D057) <= set(tools)
        r = await cs.call_tool("name_place", {"name": "basement"})
        assert json.loads(r.content[0].text)["args"] == {"name": "basement", "new": False, "rename": False}
    assert got == ["notifications/tools/list_changed"]
    assert sent == [("name_place", {"name": "basement", "new": False, "rename": False})]


async def test_place_calls_forward_to_the_cockpit_with_the_executors_arguments(fake):
    fake.recognize = True
    be = CockpitBackend(URL)
    sent = []

    async def rec(name, /, **args):
        sent.append((name, args))
        return {"ok": True, "via": name}
    be._tool = rec
    srv = build_server(be)
    async with client_session(srv._mcp_server) as cs:
        for name, args in (("where_am_i", {}), ("places", {}),
                           ("name_place", {"name": "master bedroom"}),
                           ("name_place", {"name": "basement", "new": True}),
                           ("name_place", {"name": "study", "rename": True, "new": None}),   # null = false
                           ("forget_place", {"name": "all"})):
            r = await cs.call_tool(name, args)
            assert json.loads(r.content[0].text) == {"ok": True, "via": name}, (name, args)
        n = len(sent)
        # the cockpit refuses anything but a boolean for new / rename (Brains.name_place): so does MCP
        for bad in ({"name": "x", "new": "yes"}, {"name": "x", "new": 1}, {"name": "x", "rename": "true"}):
            r = await cs.call_tool("name_place", bad)
            assert r.isError and "true or false" in r.content[0].text, bad
        r = await cs.call_tool("forget_place", {})                          # name is required
        assert r.isError
        assert len(sent) == n                                               # nothing reached the cockpit
    assert sent == [("where_am_i", {}), ("places", {}),
                    ("name_place", {"name": "master bedroom", "new": False, "rename": False}),
                    ("name_place", {"name": "basement", "new": True, "rename": False}),
                    ("name_place", {"name": "study", "new": False, "rename": True}),
                    ("forget_place", {"name": "all"})]


async def test_an_older_cockpit_has_no_place_tools(fake):
    fake.recognize = True                         # whatever it would say: a pre-D056 cockpit cannot report it
    fake.modern = False
    srv = build_server(CockpitBackend(URL))
    assert _names(srv) == D056_MCP and "/api/gesture/list" in fake.gets
    # a modern cockpit with recognition on, then an older one in its place (404): the tools go
    fake.modern = True
    be = CockpitBackend(URL)
    srv = build_server(be)
    assert set(ADDED_D057) <= set(_names(srv)) and be.capabilities() >= {"places"}
    fake.modern = False
    _stale(srv)
    await srv.list_tools()
    assert _names(srv) == D056_MCP and be.last_capabilities is None and "places" not in be.capabilities()
    # a D056 cockpit (a snapshot, but no `places` in it) is the D056 list
    fake.modern = True
    fake.recognize = False
    _stale(srv)
    await srv.list_tools()
    assert _names(srv) == D056_MCP


async def test_a_failed_fetch_keeps_the_place_tools_it_had(fake):
    """As every list (D056): a cockpit that stops answering does not take the tools away, and a
    5xx on /api/capabilities (the list routes still answering) keeps the last reported flag."""
    fake.recognize = True
    srv = build_server(CockpitBackend(URL))
    want = D056_MCP + list(ADDED_D057)
    assert _names(srv) == want
    fake.down = True
    _stale(srv)
    await srv.list_tools()
    assert _names(srv) == want
    fake.down, fake.status = False, 500
    _stale(srv)
    await srv.list_tools()
    assert _names(srv) == want and srv.live.source_ok is True


async def test_auto_backend_follows_recognition_and_answers_without_a_cockpit(fake):
    alive = {"v": True}
    auto = AutoBackend(url=URL, make_fallback=MockBackend, ttl=0.0, alive=lambda u: alive["v"])
    sent = []

    async def rec(name, /, **args):
        sent.append((name, args))
        return {"ok": True, "via": name}
    auto.cockpit._tool = rec
    srv = build_server(auto)
    assert _names(srv) == D056_MCP                                         # recognition off
    fake.recognize = True
    assert await srv.live.poll_once() is False                             # no session yet: nobody to notify
    assert _names(srv) == D056_MCP + list(ADDED_D057)
    async with client_session(srv._mcp_server) as cs:
        r = await cs.call_tool("forget_place", {"name": "here"})
        assert json.loads(r.content[0].text) == {"ok": True, "via": "forget_place"}
        alive["v"] = False                                                 # the cockpit is gone
        _stale(srv)
        assert set(ADDED_D057) <= set(await _list(cs))                     # the last list is kept ...
        r = await cs.call_tool("where_am_i", {})
        out = json.loads(r.content[0].text)                                # ... and the sim says it has none
        assert out["ok"] is False and "cockpit" in out["error"] and "place" in out["error"]
    assert sent == [("forget_place", {"name": "here"})]


async def test_mock_and_sim_never_offer_place_tools(fake):
    """No places without a cockpit that reports them — not even on a backend that has the methods."""
    from harness.sim_backend import SimBackend

    class Placey(MockBackend):
        async def where_am_i(self):
            return {"ok": True}

        async def name_place(self, name, new=False, rename=False):
            return {"ok": True}

        async def places(self):
            return {"ok": True}

        async def forget_place(self, name):
            return {"ok": True}
    assert Placey().capabilities() == set() and "places" not in SimBackend.__new__(SimBackend).capabilities()
    for be in (MockBackend(), Placey()):
        assert not set(ADDED_D057) & set(_names(build_server(be)))
