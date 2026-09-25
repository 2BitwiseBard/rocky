"""Situational awareness + the memory routes on a REAL headless CockpitSim
(MuJoCo physics, rendering off, righter off). No model server: every model
call is wired to fail loudly, so a test that passes proves the awareness loop
never called one.

What is pinned: the tick composes a situation line from what the sim has
(pose, guards, a lidar summary, memory, heat, bus); a new obstacle within
0.5 m while the robot stands still says curious_question ONCE (30 s rate
limit), the robot's own motion does not count, a void latch says alarm_help;
reactions off = silent; a tick that raises turns awareness off instead of
killing the sim; /api/memory and /api/awareness answer and validate; the
state feed carries 'situation' and 'memory_objects'; a world switch switches
the memory; the MCP server offers the memory tools only where a cockpit is."""
import asyncio
import os
import sys
import threading
import warnings

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
for sub in ("sim", "gait", "perception"):
    sys.path.insert(0, os.path.join(ROOT, sub))
sys.path.insert(0, ROOT)

pytest.importorskip("starlette")
from starlette.testclient import TestClient                     # noqa: E402


def _no_model(*_a, **_k):
    raise AssertionError("the awareness loop must not call a model")


def _make_sim(world="flat"):
    import cockpit
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        sim = cockpit.CockpitSim(world)
    sim.do("righter off")
    sim.render_every = 10 ** 9
    sim.brains.conf_path = None
    sim.brains._complete_fn = _no_model
    return sim


def _settle(sim, n=300):
    for _ in range(n):
        sim.step()
    assert sim.idle_reason() is None, sim.idle_reason()


def _lid(**near):
    import cockpit
    sectors = {n: None for n in cockpit.SECTORS}
    sectors.update({k.replace("_", "-"): v for k, v in near.items()})
    rs = [(r, n) for n, r in sectors.items() if r is not None]
    r, n = min(rs) if rs else (None, None)
    return dict(sectors=sectors, nearest_m=r, nearest_dir=n, plane_m=0.178)


@pytest.fixture(scope="module")
def still_sim():
    sim = _make_sim()
    sim.awareness["interval_s"] = 0.0          # the step loop must not tick on its own here
    _settle(sim)
    return sim


def _reacts(sim):
    return [e for e in sim.events if e[0] == "react"]


def test_tick_composes_a_situation_and_reacts_once_to_a_new_obstacle(still_sim):
    sim = still_sim
    sim.set_awareness(interval_s=20, reactions=True, curious=False)
    sim.memory.remember({"kind": "find", "text": "ball", "objects": [
        {"name": "ball", "x": 0.5, "y": 0.5, "confidence": 0.9}]})
    sim._aware_tick(now=1000.0, lid=_lid())                              # baseline, composes
    s = sim.situation
    assert s["text"].startswith("at (") and "world 'flat'" in s["text"] and "reflex NORMAL, idle" in s["text"]
    assert "nothing within 1.5 m" in s["text"] and "ball 0.7 m ahead-left" in s["text"]
    # nominal clauses are left out (review 2026-09-24): no bus without servos, no heat when cool,
    # no 'no guard latched'; the bus is still in the fields for the UI
    assert "no servos connected" not in s["text"] and "thermal" not in s["text"] and "guard" not in s["text"]
    assert s["bus"].startswith("bus: no servos") and "eye: no description yet" in s["text"]
    assert len(s["text"]) < 400
    assert _reacts(sim) == []
    said0 = sim.said[0]
    sim._aware_tick(now=1001.1, lid=_lid(ahead=0.40))                    # something new, 0.4 m ahead
    r = _reacts(sim)
    assert len(r) == 1 and r[0][1].startswith("curious_question: something new 0.40 m ahead")
    assert sim.said == (said0 + 1, "curious_question")
    assert sim.memory.last(5, kind="said") and sim.memory.last(5, kind="scan")
    sim._aware_tick(now=1002.2, lid=_lid(ahead=0.40, left=0.30))         # another one, within 30 s
    assert len(_reacts(sim)) == 1 and any("held back" in line for _t, line in sim.console)
    assert sim.brains.last_look is None                                  # no look was asked of anyone


def test_a_guard_latch_says_alarm_help_and_is_remembered(still_sim):
    sim = still_sim
    sim.set_awareness(reactions=True)
    n = len(_reacts(sim))
    sim.note("void", "VOID at 20 deg (test) — backing off, then safe-stop")
    assert sim.memory.last(1, kind="guard")[0]["text"].startswith("void: VOID at 20 deg")
    sim._aw["last_react"] = -1e9
    sim._aware_tick(now=2000.0, lid=_lid())
    r = _reacts(sim)
    assert len(r) == n + 1 and r[-1][1].startswith("alarm_help: void: VOID at 20 deg")
    sim.refusal = None


def test_own_motion_and_reactions_off_stay_silent(still_sim):
    sim = still_sim
    sim.set_awareness(reactions=True)
    sim._aw.update(last_react=-1e9, check_next=0.0)
    sim._aware_tick(now=3000.0, lid=_lid())
    sim._aw["prev_pose"] = {"x": 5.0, "y": 5.0, "yaw_deg": 90.0}          # "it just walked here"
    n = len(_reacts(sim))
    sim._aware_tick(now=3001.1, lid=_lid(ahead=0.3))
    assert len(_reacts(sim)) == n                                         # its own motion: not news
    sim.set_awareness(reactions=False)
    sim._aware_tick(now=3002.2, lid=_lid())
    sim._aware_tick(now=3003.3, lid=_lid(right=0.3))
    assert len(_reacts(sim)) == n


def test_a_failing_tick_turns_awareness_off_not_the_sim(still_sim):
    sim = still_sim
    sim.set_awareness(interval_s=20, reactions=True)
    real = sim._lidar_summary

    def boom():
        raise RuntimeError("lidar on fire")
    sim._lidar_summary = boom
    try:
        for _ in range(3):
            sim._aw["check_next"] = 0.0
            sim.step()                                                    # must not raise
        assert sim.awareness["interval_s"] == 0.0 and sim.awareness["reactions"] is False
        assert any("awareness: OFF" in line for _t, line in sim.console)
    finally:
        sim._lidar_summary = real
    with pytest.raises(ValueError):
        sim.set_awareness(interval_s=-1)
    with pytest.raises(ValueError):
        sim.set_awareness(curious="yes")
    assert sim.set_awareness(interval_s=2)["interval_s"] == 5.0           # clamped


def test_the_real_lidar_summary_sees_the_obstacle_course_wall():
    sim = _make_sim("obstacle course")
    _settle(sim, 200)
    lid = sim._lidar_summary()
    assert lid["nearest_dir"] == "ahead" and 1.0 < lid["nearest_m"] < 1.3    # the wall at x = 1.2
    assert lid["sectors"]["behind"] is None


# ------------------------------------------------------------------ over HTTP
@pytest.fixture(scope="module")
def client():
    import cockpit
    sim = _make_sim()
    sim.speed = 4.0
    threading.Thread(target=sim.run_forever, daemon=True).start()
    c = TestClient(cockpit.make_app(sim, extra_hosts=("testserver",)))
    yield sim, c
    sim.alive = False


def test_memory_routes(client):
    sim, c = client
    j = c.get("/api/memory").json()
    assert j["ok"] and j["world"] == "flat" and j["objects"] == [] and "situation" in j
    assert j["stats"]["path"] is None                                     # RAM only under test
    r = c.post("/api/memory", json={"action": "remember", "name": "charger", "x": 0.8, "y": -0.2}).json()
    assert r["ok"] and r["object"]["name"] == "charger" and r["objects"][0]["x"] == 0.8
    r = c.post("/api/memory", json={"action": "remember", "note": "the door is behind the sofa"}).json()
    assert r["ok"] and r["object"] is None
    r = c.post("/api/memory", json={"action": "where_is", "name": "charger"}).json()
    assert r["known"] and r["confidence"] == 0.9
    r = c.post("/api/tool/where_is", json={"name": "charger"}).json()      # the MCP proxy's path
    assert r["known"] and r["source"] == "user"
    r = c.post("/api/memory", json={"action": "recall", "query": "door"}).json()
    assert r["found"] == 1
    assert c.post("/api/memory", json={"action": "bogus"}).status_code == 400
    r = c.post("/api/memory", json={"action": "remember", "name": "x", "x": "nan", "y": 0}).json()
    assert r["ok"] is False
    j = c.get("/api/memory?n=5").json()
    assert len(j["observations"]) == 2 and j["observations"][0]["text"] == "the door is behind the sofa"


def test_state_carries_situation_and_memory_objects(client):
    sim, c = client
    assert c.post("/api/awareness", json={"interval_s": -3}).status_code == 400
    r = c.post("/api/awareness", json={"interval_s": 15, "reactions": False, "refresh": True}).json()
    assert r["ok"] and r["awareness"]["interval_s"] == 15 and r["awareness"]["reactions"] is False
    assert r["situation"]["text"].startswith("at (")
    s = c.get("/api/state").json()
    assert s["situation"]["text"] and s["situation"]["age_s"] is not None
    assert set(s["situation"]["lidar"]) == {"nearest_m", "nearest_dir", "sectors"}
    assert s["awareness"]["interval_s"] == 15
    names = {o["name"] for o in s["memory_objects"]}
    assert "charger" in names                                             # pinned by test_memory_routes (file order)
    ch = next(o for o in s["memory_objects"] if o["name"] == "charger")
    assert {"x", "y", "confidence", "source", "age_s", "stale"} <= set(ch)
    st = c.post("/api/tool/status", json={}).json()
    assert st["situation"].startswith("at (")                            # MCP status carries the line
    r = c.post("/api/memory", json={"action": "clear"}).json()
    assert r["ok"] and r["objects"] == []
    assert c.get("/api/awareness").json()["awareness"]["interval_s"] == 15


def test_a_world_switch_switches_the_memory(client):
    sim, c = client
    c.post("/api/memory", json={"action": "remember", "name": "ball", "x": 0.3, "y": 0.0})
    c.post("/api/world", json={"preset": "obstacle course"})
    # a fresh world remembers only where the robot spawned ('go back to the start')
    assert sim.memory.world == "obstacle course"
    assert [(o["name"], o["source"], o.get("anchor")) for o in sim.memory.objects()] == [("start", "spawn", "robot")]
    c.post("/api/world", json={"preset": "flat"})
    assert sim.memory.world == "flat"
    # RAM only: a world you left is not kept without a directory (persistence is main()'s)
    assert sim.memory.where_is("ball") is None


def test_mcp_server_offers_memory_tools_only_with_a_cockpit():
    import harness.server as hs
    from harness.backend import MockBackend
    from harness.cockpit_backend import AutoBackend
    names = lambda srv: {t.name for t in asyncio.run(srv.list_tools())}      # noqa: E731
    assert not ({"where_is", "go_back_to", "remember"} & names(hs.build_server(MockBackend())))
    auto = AutoBackend(url="http://127.0.0.1:9", alive=lambda u: False, make_fallback=MockBackend)
    assert {"remember", "where_is", "recall", "go_back_to", "forget"} <= names(hs.build_server(auto))
    r = asyncio.run(auto.where_is("ball"))                                # no cockpit: says so
    assert r["ok"] is False and "no scene memory" in r["error"]
    r = asyncio.run(auto.go_back_to("ball"))
    assert r["ok"] is False and r["backend"] == "sim"


# ------------------------------------------------------------ review 2026-09-24 fixes
def test_the_situation_line_stays_short_with_a_full_memory_and_a_long_look():
    import time as _t
    sim = _make_sim()
    sim.awareness["interval_s"] = 0.0
    _settle(sim, 100)
    for i, (name, x, y) in enumerate((("ball", 0.5, 0.5), ("box", 1.0, -0.4), ("red chair", -0.8, 0.2),
                                       ("charger", 0.2, -1.1))):
        sim.memory.remember({"kind": "find", "text": name, "objects": [
            {"name": name, "x": x, "y": y, "confidence": 0.9}]})
    sim.memory.remember({"kind": "user", "text": "the kitchen is through the door on the left of the sofa, "
                                                 "past the rug and the two plant pots"})
    sim.brains.last_look = {"text": "A long description " * 12, "model": "lfm2.5-vl", "t": _t.time(), "pose": None}
    s = sim.compose_situation(lid=_lid(ahead=1.18))
    assert len(s["text"]) < 400, (len(s["text"]), s["text"])
    assert "lidar: nearest 1.18 m ahead; clear elsewhere" in s["text"]
    assert "+1 more" in s["text"] and "(0.50, 0.50)" not in s["text"]     # 3 objects, no map coordinates
    assert s["text"].index("lidar") < s["text"].index("remembered") < s["text"].index("eye (")
    sim.brains.last_find = {"name": "ball", "found": True, "detail": "found", "t": _t.time() + 1, "model": "x"}
    assert "eye: last used by find_object (ball found" in sim.compose_situation(lid=_lid())["text"]
    many = _lid(ahead=0.4, ahead_left=0.5, left=0.6, behind=0.7, right=0.9)
    w = sim._lidar_words(many)
    assert "(+2 more within 1.5 m)" in w and "clear: behind-left, behind-right, ahead-right" in w


def test_bus_rate_events_are_routine_and_bus_guards_are_rate_limited():
    sim = _make_sim()
    n0 = len(sim.memory.last(500, kind="guard"))
    for _ in range(5):
        sim._hw_event("rate", "goal step clamped")
        sim._hw_event("reopen", "port reopened")
        sim._hw_event("nan", "non-finite targets for legs [1]")
    g = sim.memory.last(500, kind="guard")
    assert len(g) - n0 == 1 and g[0]["text"].startswith("bus nan")
    assert sum(1 for e in sim.events if e[0] == "hw:nan") == 5                # the events feed keeps them all


def test_memory_key_separates_custom_worlds_and_keeps_preset_names():
    import cockpit
    from world_builder import PRESETS
    assert cockpit.memory_key("room", dict(PRESETS["room"])) == "room"
    a = cockpit.memory_key("custom", {"base": "flat", "objects": [{"kind": "ball", "pos": [1, 0]}]})
    b = cockpit.memory_key("custom", {"base": "flat", "objects": [{"kind": "ball", "pos": [2, 0]}]})
    assert a.startswith("custom-") and b.startswith("custom-") and a != b
    assert cockpit.memory_key("room", {"base": "flat"}).startswith("room-")   # an edited preset is not the preset


def test_bad_json_is_a_400_and_a_reset_makes_sightings_earlier_session(client):
    sim, c = client
    for route in ("/api/memory", "/api/awareness"):
        r = c.post(route, content=b"not json", headers={"content-type": "application/json"})
        assert r.status_code == 400 and r.json()["error"] == "body must be JSON", route
    sim.memory.remember({"kind": "find", "text": "ball", "objects": [
        {"name": "ball", "x": 0.5, "y": 0.3, "confidence": 0.95}]})
    assert not c.post("/api/tool/where_is", json={"name": "ball"}).json()["stale"]
    assert "reset" in c.post("/api/reset", json={}).json()["reply"]
    w = c.post("/api/tool/where_is", json={"name": "ball"}).json()
    assert w["earlier_session"] and w["stale"] and "before the last reset" in w["detail"]
    r = c.post("/api/tool/go_back_to", json={"name": "ball"}).json()
    assert r["ok"] is False and "before the last reset" in r["error"]
    st = c.post("/api/tool/where_is", json={"name": "start"}).json()
    assert st["known"] and st["source"] == "spawn" and not st["stale"]
    r = c.post("/api/memory", json={"action": "forget", "name": "that"})
    assert r.json()["ok"] is False and sim.memory.where_is("ball") is not None
