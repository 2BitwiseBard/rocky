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
the memory; the MCP server offers the memory tools only where a cockpit is.

D057 place recognition (the end of this file; a fake eye and a fake embedder,
the lidar and the physics real): OFF by default and then nothing changes; on,
a first world is a new place, another world a new one, the first again is
known with its memory bound back; a change is declared only when two looks
agree (a new object, a missing one) and a single look declares nothing; an
unsure place takes one turn and name_place settles it; the tick recognises
by itself after a world switch; the four tools through /api/tool and
Brains.tool; talk mode's place lines only while it is on; a model turn reads
the place first and can name it. Review fixes: a failed look declares
nothing and erases nothing; a wrong 'known' corrected by the owner (by name,
or 'this is completely new') leaves the wrong place as it was and moves the
records; 'this is completely new' stores one place per visit; an edit keeps
the bound place; no tool or memory record names the world while recognition
is on; the situation line stays under 400 chars with a place clause; 'this X
has a new Y' really checks with two looks. The change check by the EYE
(2026-09-25): per name a box question (a fake box answerer), True / False /
None; a planted chair is 'added' only when both looks box it, a removed ball
'missing' only when both looks see its spot and say no, an object out of view
is never asked and never missing, an eye that errs or answers prose declares
nothing; a new place is stored from two looks (two samples) and the situation
line says it is looking again; where_am_i and /api/state carry the signals.
Review round 2 (2026-09-25): a thing the eye boxes but can never place (a cup
on a table) is kept by name and is not 'new here' on every visit (a new one is
declared once; one placed later is stored, not declared); both looks stand on
one spot, so a ball hidden behind a box the eye drew, or behind a nearer
lidar return, is unobservable in both and never erased; a two-look answer
carries each look's own parts (per_look) and the senses as a list. The fake
eye's boxes cover the whole object (its top edge from the object's height)."""
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


# ================================================================ D057: place recognition
# The cockpit recognises the place from what the robot senses (sim/place_memory.py): a lidar
# sweep + a look + its embedding. Here the eye is FAKE (a description written from the world
# spec and the robot's pose: what is within the eye's +-43 deg and 2 m, 'on the left' /
# 'straight ahead' / 'on the right', its distance) and so is the embedder (a bag of words
# hashed into 256 dims): no vision model, no llama-swap. The lidar and the physics are real.
# The change check asks the eye per name (Brains.detect, FIND_PROMPT, method bbox): the fake
# answers with a box drawn by the inverse of the cockpit's camera model (_fake_find), so the
# box -> floor geometry the cockpit applies lands on the object. An object's `label` (a key
# world_builder ignores) is what the fake eye calls it ('chair' for a box).
import hashlib                                                   # noqa: E402
import json                                                      # noqa: E402
import math                                                      # noqa: E402
import re                                                        # noqa: E402
import time                                                      # noqa: E402

_BASE_WORDS = {"room": "A small room with pale walls all around and two grey pillars.",
               "flat": "An open grey floor stretching far away, nothing standing on it."}


def _label(o):
    return o.get("label") or o["kind"]


def _describe(sim):
    spec, pose = sim.world_spec, sim.pose()
    parts = [spec.get("look") or _BASE_WORDS.get(spec.get("base", "flat"), "A floor.")]
    for o in spec.get("objects") or []:
        dx, dy = o["pos"][0] - pose["x"], o["pos"][1] - pose["y"]
        d = math.hypot(dx, dy)
        rel = (math.degrees(math.atan2(dy, dx)) - pose["yaw_deg"] + 180.0) % 360.0 - 180.0
        if abs(rel) > 43.0 or d > 2.0:
            continue
        side = "on the left" if rel > 15 else "on the right" if rel < -15 else "straight ahead"
        parts.append(f"A {_label(o)} {side}, {d:.1f} m away.")
    return " ".join(parts)


def _floor_to_uv(fwd, left, z=0.0):
    """A point in the CAMERA's frame (fwd ahead of it, left of it, z above the floor) -> the
    image point (u right, v down, 0..1): for z = 0 the inverse of cockpit_brains.pixel_to_floor
    (same pinhole, pitch and height), written out here independently of floor_to_pixel."""
    import cockpit_brains as cb
    f = (cb.EYE_H_PX / 2) / math.tan(math.radians(cb.EYE_FOVY_DEG / 2))
    p, h = math.radians(cb.EYE_PITCH_DEG), cb.EYE_HEIGHT_M - z
    depth = fwd * math.cos(p) + h * math.sin(p)
    right, up = -left, fwd * math.sin(p) - h * math.cos(p)
    return 0.5 + (right / depth) * f / cb.EYE_W_PX, 0.5 - (up / depth) * f / cb.EYE_H_PX


def _height(o):
    """An object spec's height (world_builder: a box's size is full x, y, z; a ball's radius)."""
    if o.get("radius_m") is not None:
        return 2.0 * float(o["radius_m"])
    if o.get("size"):
        return float(o["size"][2])
    return float(o.get("height_m") or 0.05)


def _fake_find(sim, name):
    """The fake eye's answer to FIND_PROMPT for `name`: the nearest spec object it calls that
    (its label or kind) whose centre is within +-43 deg and 0.2-2 m of the CAMERA -> a box whose
    bottom edge is the object's near edge on the floor (so sighting_to_map adds half the width
    back and lands on its centre) and whose top edge is the object's top at its far side (a box
    drawn around the whole thing, as a real model draws one: what lies inside it is behind it),
    else seen false."""
    import cockpit_brains as cb
    pose = sim.pose()
    yaw = math.radians(pose["yaw_deg"])
    cx, cy = pose["x"] + cb.EYE_FWD_M * math.cos(yaw), pose["y"] + cb.EYE_FWD_M * math.sin(yaw)
    best = None
    for o in sim.world_spec.get("objects") or []:
        if _label(o) != name:
            continue
        dx, dy = o["pos"][0] - cx, o["pos"][1] - cy
        fwd, left = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
        d = math.hypot(fwd, left)
        if abs(math.degrees(math.atan2(left, fwd))) > 43.0 or not 0.2 <= d <= 2.0:
            continue
        if best is None or d < best[0]:
            half = float(o.get("radius_m") or max(o.get("size") or [0.1]) / 2.0)
            best = (d, fwd, left, half, _height(o))
    if best is None:
        return json.dumps({"seen": False, "what": "no " + name})
    d, fwd, left, half, hgt = best
    nf, nl = fwd * (d - half) / d, left * (d - half) / d           # the near edge, toward the camera
    ux, uy = -nl / (d - half) * half, nf / (d - half) * half        # half the width, sideways
    (u1, v1), (u2, v2) = _floor_to_uv(nf + ux, nl + uy), _floor_to_uv(nf - ux, nl - uy)
    x1, x2 = sorted((min(max(u1, 0.0), 1.0), min(max(u2, 0.0), 1.0)))
    y2 = min(max((v1 + v2) / 2, 0.0), 1.0)
    far = (d + half) / d
    top = min(_floor_to_uv(fwd * far, left * far, hgt)[1], _floor_to_uv(nf, nl, hgt)[1])
    y1 = min(max(top, 0.0), y2 - 0.01)
    box = [round(x1 * 1000), round(y1 * 1000), round(x2 * 1000), round(y2 * 1000)]
    return json.dumps({"seen": True, "bbox": box, "confidence": 0.9, "what": name})


_FIND_RE = re.compile(r"^Is there an? (.+?) in this image\?")


def _fake_embed(text):
    v = [0.0] * 256
    for w in re.findall(r"[a-z]+", str(text).lower()):
        v[int(hashlib.md5(w.encode()).hexdigest(), 16) % 256] += 1.0
    return v


def _prompt(msgs):
    c = msgs[-1]["content"]
    return c[0]["text"] if isinstance(c, list) else str(c)


def _wire_fake_eye(sim, find=None):
    """The fake eye: a description (_describe) for a look, a box (_fake_find) for a find
    question; find(name, n) -> an answer overrides the box for the n-th question (1-based)."""
    b = sim.brains
    b._eye_b64 = lambda: "ZmFrZQ=="                  # the renderer is off under test: any frame will do
    sim.fake_finds = []                              # every find question, in order: (name, pose)

    async def acatalog():
        return {}

    async def acall(model, msgs, tools, max_tokens):
        if tools:
            raise AssertionError("no chat model in the place tests unless a test scripts one")
        m = _FIND_RE.match(_prompt(msgs))
        if m:
            sim.fake_finds.append((m.group(1), sim.pose()))
            ans = find(m.group(1), len(sim.fake_finds)) if find else None
            if isinstance(ans, Exception):
                raise ans
            return {"content": ans if ans is not None else _fake_find(sim, m.group(1))}
        return {"content": _describe(sim)}
    b._acatalog = acatalog
    b.chain = lambda role, first=None, cat=None: (["fake-vl"], [])
    b._acall = acall
    sim.embed_fn = _fake_embed


@pytest.fixture
def place_client(tmp_path):
    """A fresh headless cockpit with the fake eye, persistence in tmp_path (as main() does with
    ROCKY_MEMORY_DIR) and ONE event loop for every request (the tick schedules recognitions on it)."""
    import cockpit
    sim = _make_sim()
    _wire_fake_eye(sim)
    sim.memory.set_directory(str(tmp_path))
    sim.place_memory.set_directory(str(tmp_path))
    sim.speed = 4.0
    threading.Thread(target=sim.run_forever, daemon=True).start()
    with TestClient(cockpit.make_app(sim, extra_hosts=("testserver",))) as c:
        yield sim, c, tmp_path
    sim.alive = False


def _recognise(c, **kw):
    r = c.post("/api/place", json=dict(action="recognize", **kw)).json()
    assert r.get("ok"), r
    return r


def _room_with(*objects):
    return {"base": "room", "objects": [dict(o) for o in objects]}


def test_recognition_off_changes_nothing(place_client):
    sim, c, _ = place_client
    assert sim.awareness["recognize"] is False and sim.awareness_settings()["recognize"] is False
    s = c.get("/api/state").json()
    assert "place" not in s and s["awareness"]["recognize"] is False
    assert c.post("/api/world", json={"preset": "room"}).json()["ok"]
    assert sim.memory.world == "room" and sim.memory is sim._mem_main          # keyed by the world, as before
    r = c.post("/api/awareness", json={"refresh": True}).json()
    assert "place" not in r and r["situation"]["text"].startswith("at (") and "world 'room'" in r["situation"]["text"]
    w = c.post("/api/tool/where_am_i", json={}).json()
    assert w["ok"] is False and w["error"] == "place recognition is off" and "recognize" in w["hint"]
    assert c.post("/api/tool/name_place", json={"name": "attic"}).json()["ok"] is False
    assert c.post("/api/place", json={"action": "recognize"}).json()["ok"] is False
    p = c.post("/api/tool/places", json={}).json()
    assert p["ok"] and p["count"] == 0 and p["recognize"] is False and p["current"] is None
    assert c.post("/api/awareness", json={"recognize": "yes"}).status_code == 400
    with pytest.raises(ValueError):
        sim.set_awareness(recognize=1)
    time.sleep(0.5)                                                           # a tick would have run by now
    assert sim.place_memory.stats()["places"] == 0 and [e for e in sim.events if e[0] == "place"] == []
    m = c.get("/api/memory").json()
    assert m["places"] == [] and "place" not in m
    assert c.post("/api/place", json={"action": "bogus"}).status_code == 400


def test_new_new_known_and_a_new_object_after_two_looks(place_client):
    sim, c, tmp = place_client
    r = c.post("/api/awareness", json={"recognize": True, "reactions": True}).json()
    assert r["awareness"]["recognize"] is True and r["place"]["verdict"] == "recognising"
    assert sim.memory.world == "place-pending" and sim.memory is not sim._mem_main  # provisional, RAM only
    # 1. a first world: nothing to compare with -> a new place, the memory bound to it
    assert c.post("/api/world", json={"preset": "room"}).json()["ok"]
    a = _recognise(c)
    assert (a["verdict"], a["name"], a["text"]) == ("new", "new place #1", "place: NEW (no places yet)")
    assert a["memory_key"] == a["place_id"] == sim.memory.world and a["place_id"].startswith("place-")
    assert ("react", "curious_question: a new place: stored as new place #1 (the first place)") in sim.events
    st = c.get("/api/state").json()["place"]
    assert {"verdict", "name", "confidence", "place_id", "text", "status"} <= set(st) and st["verdict"] == "new"
    assert c.post("/api/memory", json={"action": "remember", "name": "charger", "x": 0.5, "y": 0.2}).json()["ok"]
    # 2. another world: new again, its own memory
    assert c.post("/api/world", json={"preset": "obstacle course"}).json()["ok"]
    b = _recognise(c)
    assert b["verdict"] == "new" and b["name"] == "new place #2" and b["place_id"] != a["place_id"]
    assert b["nearest_known"]["place_id"] == a["place_id"] and b["confidence"] < 0.5
    assert sim.memory.world == b["place_id"] and sim.memory.where_is("charger") is None
    # 3. back to the first: known, and the memory is the first place's again (from its file)
    assert c.post("/api/world", json={"preset": "room"}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["place_id"] == a["place_id"] and k["confidence"] >= 0.75
    assert k["second"]["place_id"] == b["place_id"] and k["looks"] == 1 and k["changes"]["added"] == []
    assert sim.memory.world == a["place_id"]
    w = c.post("/api/tool/where_is", json={"name": "charger"}).json()
    assert w["known"] and w["earlier_session"]                               # remembered here, before this spawn
    assert k["text"].startswith("place: new place #1 (") and "new here" not in k["text"]
    # 4. the known room gets a ball: the change is declared only after the second look agrees
    sim._aw["last_react"] = -1e9                                              # the 30 s reaction limit
    n0 = len(sim.cmd_log)
    assert c.post("/api/world/add", json={"object": {"kind": "ball", "pos": [0.6, 0.0], "radius_m": 0.05}}).json()["ok"]
    d = _recognise(c)
    assert d["verdict"] == "known" and d["place_id"] == a["place_id"] and d["looks"] == 2
    assert [o["name"] for o in d["changes"]["added"]] == ["ball"] and d["changes"]["missing"] == []
    assert any(line.startswith("tool turn") for _t, line in sim.cmd_log[n0:])      # the one 30 deg turn
    assert d["text"].startswith("place: new place #1 (") and d["text"].endswith("— new here: ball")
    assert len(d["text"]) <= 90
    assert any(e[0] == "react" and e[1].startswith("curious_question: new here: ball") for e in sim.events)
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]   # stored: confirmed
    # the situation line starts with the verdict and no longer names the world
    sit = c.post("/api/awareness", json={"refresh": True}).json()["situation"]
    assert sit["text"].startswith(d["text"] + ". at (") and "world '" not in sit["text"]
    assert sit["place"]["verdict"] == "known"
    # persistence: <dir>/places.json and one scene-memory file per place
    sim.flush_memories()
    places = json.load(open(tmp / "places.json"))["places"]
    assert sorted(p["id"] for p in places) == sorted([a["place_id"], b["place_id"]])
    assert (tmp / f"{a['place_id']}.json").exists() and not (tmp / "place-pending.json").exists()
    mem = c.get("/api/memory").json()
    assert {p["id"] for p in mem["places"]} == {a["place_id"], b["place_id"]} and mem["place"]["verdict"] == "known"


def test_missing_is_declared_after_two_looks_and_one_look_declares_nothing(place_client):
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    ball_ahead = {"kind": "ball", "pos": [0.8, 0.0], "radius_m": 0.05}
    assert c.post("/api/world", json={"name": "w1", "spec": _room_with(ball_ahead)}).json()["ok"]
    a = _recognise(c)
    assert a["verdict"] == "new" and [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]
    # the ball is gone: the first look misses it where it could see it, the second (30 deg left,
    # the spot still in view) agrees -> declared, and the stored snapshot loses it
    assert c.post("/api/world", json={"name": "w2", "spec": _room_with()}).json()["ok"]
    m = _recognise(c)
    assert m["verdict"] == "known" and m["looks"] == 2 and [o["name"] for o in m["changes"]["missing"]] == ["ball"]
    assert [e["seen"] for e in m["eye_checks"]] == [{"ball": False}, {"ball": False}]   # both looks asked the eye
    assert m["text"].endswith("— missing: ball") and sim.place_memory.get(a["place_id"])["objects"] == []
    # a ball on the right: after the turn its spot is out of the eye's view, so only one look saw
    # it -> pending, not declared, not stored
    ball_right = {"kind": "ball", "pos": [0.5, -0.35], "radius_m": 0.05}
    assert c.post("/api/world", json={"name": "w3", "spec": _room_with(ball_right)}).json()["ok"]
    p = _recognise(c)
    assert p["verdict"] == "known" and p["looks"] == 2
    assert p["changes"] == {"added": [], "missing": [], "moved": []}
    assert [(o["name"], o["change"]) for o in p["pending"]] == [("ball", "added")]
    assert "not declared" in p["detail"] and "new here" not in p["text"]
    assert sim.place_memory.get(a["place_id"])["objects"] == []


def test_an_unsure_place_takes_one_turn_and_name_place_settles_it(place_client):
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    # a world the lidar sees: an empty sweep is no sense at all (place_memory SCAN_INFO_MIN), and
    # the look alone never says known (SINGLE_EVIDENCE)
    c.post("/api/world", json={"preset": "room"})
    first = _recognise(c)
    assert first["verdict"] == "new"
    # "this is completely new" on the visit that stored it: still ONE place (review 2026-09-25)
    k = c.post("/api/place", json={"action": "name", "name": "attic", "new": True}).json()
    assert k["ok"] and k["action"] == "renamed" and k["place_id"] == first["place_id"]
    assert len(sim.place_memory.places()) == 1
    # the robot knows it again after a reset; the operator says it is a DIFFERENT place that
    # senses the same: stored under the name from the same senses, the known one left as it was
    assert "reset" in c.post("/api/reset", json={}).json()["reply"]
    again = _recognise(c)
    assert again["verdict"] == "known" and again["place_id"] == first["place_id"]
    before = sim.place_memory.get(first["place_id"])
    r = c.post("/api/place", json={"action": "name", "name": "loft", "new": True}).json()
    assert r["ok"] and r["action"] == "stored" and r["name"] == "loft" and r["place_id"] != first["place_id"]
    assert sim.memory.world == r["place_id"] and r["text"] == "place: loft (1.00)"
    assert r["took_back"]["how"] == "restored" and r["took_back"]["name"] == "attic"
    assert sim.place_memory.get(first["place_id"])["visits"] == before["visits"] - 1   # the visit taken back
    # two places that sense alike: after a reset the robot cannot tell -> one turn, still unsure,
    # no place claimed (the memory stays provisional)
    n0 = len(sim.cmd_log)
    assert "reset" in c.post("/api/reset", json={}).json()["reply"]
    u = _recognise(c)
    assert u["verdict"] == "ambiguous" and u["looks"] == 2 and u["place_id"] is None
    assert sum(1 for _t, line in sim.cmd_log[n0:] if line.startswith("tool turn")) == 1
    assert u["text"].startswith("place: ") and "?" in u["text"] and " or " in u["text"]
    assert sim.memory.world == "place-pending"
    # the operator settles it
    s = c.post("/api/tool/name_place", json={"name": "loft"}).json()
    assert s["ok"] and s["action"] == "settled" and s["place_id"] == r["place_id"]
    assert sim.memory.world == r["place_id"]
    st = c.get("/api/state").json()["place"]
    assert (st["verdict"], st["name"], st["confidence"], st["by"]) == ("known", "loft", 1.0, "operator")


def test_the_tick_recognises_after_a_world_switch_without_being_asked(place_client):
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    c.post("/api/world", json={"preset": "room"})
    got = {}

    def done():
        p = c.get("/api/state").json().get("place") or {}
        got.update(p)
        return p.get("verdict") not in (None, "recognising")
    t_end = time.monotonic() + 30.0
    while time.monotonic() < t_end and not done():
        time.sleep(0.1)
    assert got.get("verdict") == "new" and got.get("name") == "new place #1", got
    assert any(e[0] == "place" for e in sim.events)


def test_the_place_tools_through_api_tool_and_brains_tool(place_client):
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    c.post("/api/world", json={"preset": "room"})
    a = _recognise(c)
    via_api = c.post("/api/tool/where_am_i", json={}).json()
    via_brains = c.portal.call(sim.brains.tool, "where_am_i", {})
    for w in (via_api, via_brains):
        assert w["ok"] and w["verdict"] == "new" and w["place_id"] == a["place_id"]
        assert w["place"]["id"] == a["place_id"] and w["text"] == "place: NEW (no places yet)"
    n = c.post("/api/tool/name_place", json={"name": "basement"}).json()
    assert n["ok"] and n["action"] == "renamed" and n["previous_name"] == "new place #1"
    assert c.portal.call(sim.brains.tool, "name_place", {"name": "kitchen"})["previous_name"] == "basement"
    assert c.post("/api/tool/name_place", json={"name": "  "}).json()["ok"] is False
    assert c.post("/api/tool/name_place", json={"name": "x", "new": "yes"}).json()["ok"] is False
    ps = c.post("/api/tool/places", json={}).json()
    assert ps["count"] == 1 and ps["current"] == "kitchen" and ps["places"][0]["visits"] == 1
    assert c.portal.call(sim.brains.tool, "places", {})["current_id"] == a["place_id"]
    # forget: a pronoun forgets nothing; 'all' by voice needs the wake word; 'here' forgets this one
    assert c.post("/api/tool/forget_place", json={"name": "that"}).json()["ok"] is False
    v = c.portal.call(sim.brains.tool, "forget_place", {"name": "all"}, True)            # voice=True
    assert v["ok"] is False and v["error"] == "voice_unconfirmed"
    f = c.post("/api/tool/forget_place", json={"name": "here"}).json()
    assert f["ok"] and f["places"] == 1 and f["was_here"] is True
    assert sim.memory.world == "place-pending" and sim.place_memory.stats()["places"] == 0
    w = c.post("/api/tool/where_am_i", json={}).json()
    assert w["verdict"] == "unknown" and w["text"].startswith("place: unknown (forgotten")
    time.sleep(0.6)                                          # forgotten: not looked for again this spawn
    assert sim.place_memory.stats()["places"] == 0
    assert c.post("/api/tool/forget_place", json={"name": "nowhere"}).json()["ok"] is False
    # recognition off again: the world's memory, no place in the state feed
    r = c.post("/api/awareness", json={"recognize": False}).json()
    assert r["awareness"]["recognize"] is False and "place" not in c.get("/api/state").json()
    assert sim.memory.world == "room" and sim.memory is sim._mem_main


def test_talk_mode_hears_place_lines_only_with_recognition_on(place_client):
    sim, c, _ = place_client
    line = "hey, I'm in the basement at home"
    off = c.post("/api/chat", json={"text": line, "mode": "talk"}).json()
    assert all(t["tool"] != "name_place" for t in off["trace"])            # off: intent as before
    c.post("/api/awareness", json={"recognize": True})
    c.post("/api/world", json={"preset": "room"})
    _recognise(c)
    on = c.post("/api/chat", json={"text": line, "mode": "talk"}).json()
    assert [t["tool"] for t in on["trace"]] == ["name_place"] and on["trace"][0]["args"] == {"name": "basement"}
    assert "basement" in on["reply"] and sim.place_memory.find("basement")
    w = c.post("/api/chat", json={"text": "Pebble, where am I?", "mode": "talk"}).json()
    assert [t["tool"] for t in w["trace"]] == ["where_am_i"] and "basement" in w["reply"]
    new = c.post("/api/chat", json={"text": "hey, this is completely new", "mode": "talk"}).json()
    # the robot stored this place on this very visit: "completely new" keeps it — one place, not two
    assert new["trace"][0]["args"] == {"name": "", "new": True} and new["trace"][0]["result"]["action"] == "kept"
    assert "one place, not two" in new["reply"] and len(sim.place_memory.places()) == 1
    ch = c.post("/api/chat", json={"text": "hey, this master bedroom has a new chair", "mode": "talk"}).json()
    assert ch["trace"][0]["args"] == {"name": "master bedroom"} and ch["trace"][1]["tool"] == "place_check"
    # it really checked (no promise): stored on this visit, so there is nothing earlier to compare with
    assert "No change check ran" in ch["reply"] and "first stored on this visit" in ch["reply"]
    assert "I will check" not in ch["reply"]
    rn = c.post("/api/chat", json={"text": "call this place the study", "mode": "talk"}).json()
    assert rn["trace"][0]["args"] == {"name": "study", "rename": True} and rn["trace"][0]["result"]["action"] == "renamed"


def test_a_model_turn_reads_the_place_first_and_can_name_it(place_client):
    sim, c, _ = place_client
    b = sim.brains
    c.post("/api/awareness", json={"recognize": True})
    c.post("/api/world", json={"preset": "room"})
    _recognise(c)
    seen = []

    async def acall(model, msgs, tools, max_tokens):
        if not tools:                                                     # the eye (look)
            return {"content": _describe(sim)}
        seen.append((json.loads(json.dumps(msgs)), [t["function"]["name"] for t in tools]))
        if len(seen) == 1:
            return {"content": "", "tool_calls": [{"id": "c1", "name": "name_place",
                                                   "arguments": json.dumps({"name": "basement"})}]}
        return {"content": "Noted: the basement.", "tool_calls": []}
    b._acall = acall
    r = c.post("/api/chat", json={"text": "I'm in the basement", "mode": "local"}).json()
    assert r["reply"] == "Noted: the basement." and r["trace"][0]["tool"] == "name_place"
    assert r["trace"][0]["result"]["ok"] and r["trace"][0]["result"]["name"] == "basement"
    msgs, tools = seen[0]
    assert msgs[-1]["content"].startswith("Situation: place: NEW (no places yet)")
    assert "Place recognition is on" in msgs[0]["content"]
    assert {"where_am_i", "name_place", "places", "forget_place"} <= set(tools)
    assert c.get("/api/state").json()["place"]["name"] == "basement"


def test_place_helpers_are_pure():
    import cockpit
    see = cockpit.eye_visible({"x": 0.0, "y": 0.0, "yaw_deg": 0.0})
    assert see(1.0, 0.0) and see(1.0, 0.8) and not see(1.0, 1.2)            # the +-43 deg cone
    assert not see(-1.0, 0.0) and not see(0.2, 0.0) and not see(2.5, 0.0)   # behind, too near, too far
    turned = cockpit.eye_visible({"x": 0.0, "y": 0.0, "yaw_deg": 90.0})
    assert turned(0.0, 1.0) and not turned(1.0, 0.0)
    r1 = {"verdict": "ambiguous", "place_id": "place-aaaaaa", "name": "a", "confidence": 0.7,
          "ranking": [{"place_id": "place-aaaaaa", "name": "a", "confidence": 0.7},
                      {"place_id": "place-bbbbbb", "name": "b", "confidence": 0.66}]}
    r2 = {"verdict": "known", "place_id": "place-aaaaaa", "name": "a", "confidence": 0.9,
          "ranking": [{"place_id": "place-aaaaaa", "name": "a", "confidence": 0.9},
                      {"place_id": "place-bbbbbb", "name": "b", "confidence": 0.6}]}
    k = cockpit.combine_recognitions(r1, r2)
    assert (k["verdict"], k["place_id"], k["confidence"]) == ("known", "place-aaaaaa", 0.8)
    assert k["second"]["confidence"] == 0.63 and len(k["per_look"]) == 2
    r3 = dict(r2, ranking=[{"place_id": "place-bbbbbb", "name": "b", "confidence": 0.69},
                           {"place_id": "place-aaaaaa", "name": "a", "confidence": 0.68}])
    assert cockpit.combine_recognitions(r1, r3)["verdict"] == "ambiguous"
    stored = [{"name": "ball", "x": 0.8, "y": 0.0, "confidence": 0.2}, {"name": "box", "x": 1.0, "y": 0.5}]
    new = cockpit.apply_confirmed(stored, {"added": [{"name": "chair", "x": 0.7, "y": 0.1, "confidence": 0.2}],
                                           "missing": [{"name": "ball", "x": 0.8, "y": 0.0}],
                                           "moved": [{"name": "box", "from": [1.0, 0.5], "to": [1.5, 0.5], "m": 0.5}],
                                           "pending": [{"name": "cup", "x": 0.3, "y": 0.0, "change": "added"}]})
    assert sorted((o["name"], o["x"], o["y"]) for o in new) == [("box", 1.5, 0.5), ("chair", 0.7, 0.1)]
    # an addition the eye boxed twice but never placed is declared, not stored
    assert cockpit.apply_confirmed([], {"added": [{"name": "lamp"}]}) == []


def test_the_eye_check_helpers_are_pure():
    """The seen map (True | False | None per name), the placed sightings, the candidates a
    description proposes, the eye's check through place_memory.checked_objects, a new place's
    first objects, and the signal / looking-again words."""
    import cockpit
    here = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    stored = [{"name": "ball", "x": 0.8, "y": 0.0}, {"name": "cup", "x": -1.0, "y": 0.0},
              {"name": "wall", "x": 1.0, "y": 1.0}]
    spots = cockpit.place_spots(stored)
    assert spots == {"ball": [(0.8, 0.0)], "cup": [(-1.0, 0.0)]}                 # a fixture is never asked about
    assert cockpit.place_candidates("A red ball on the left, a chair ahead. No box. Pale walls.",
                                    exclude=["ball"]) == ["chair"]              # not the stored, the negated, the wall
    look = {"pose": here, "objects": [], "eye": {
        "ball": {"seen": False, "asked": True},                                  # asked, in view: a real 'no'
        "cup": {"seen": False, "asked": True},                                   # asked, but its spot is behind
        "chair": {"seen": True, "asked": True, "x": 0.7, "y": 0.1, "confidence": 0.9},
        "box": {"seen": True, "asked": True, "x": None, "y": None, "confidence": 0.8, "why": "boxed, not placed"},
        "toy": {"seen": None, "asked": True, "why": "the eye did not answer (HTTP 503)"}}}
    assert cockpit.seen_map(look, spots) == {"ball": False, "cup": None, "chair": True, "box": True, "toy": None}
    assert cockpit.found_map(look) == {"chair": {"x": 0.7, "y": 0.1, "confidence": 0.9}}
    chk = cockpit.CockpitSim._place_eye_check(stored, look)
    assert [o["name"] for o in chk["missing"]] == ["ball"]                       # in view, the eye said no
    assert sorted((o["name"], o.get("x")) for o in chk["added"]) == [("box", None), ("chair", 0.7)]
    assert {o["name"] for o in chk["unobservable"]} == {"cup", "wall"}           # out of view / never asked
    # a failed look: every name None — nothing missing, nothing added, whatever its answers say
    failed = dict(look, objects=None)
    assert set(cockpit.seen_map(failed, spots).values()) == {None} and cockpit.found_map(failed) == {}
    fchk = cockpit.CockpitSim._place_eye_check(stored, failed)
    assert fchk["missing"] == [] and fchk["added"] == []
    # a new place's first objects: a sighting another look contradicts (its spot in that look's
    # view, the eye said no) is left out; one the other look could not see is kept, once
    l1 = {"pose": here, "objects": [], "eye": {"chair": {"seen": True, "x": 0.7, "y": 0.1, "confidence": 0.9},
                                              "ball": {"seen": True, "x": 0.9, "y": 0.1, "confidence": 0.9}}}
    l2 = {"pose": dict(here, yaw_deg=30.0), "objects": [],
          "eye": {"chair": {"seen": True, "x": 0.72, "y": 0.1, "confidence": 0.8}, "ball": {"seen": False}}}
    assert [(o["name"], o["x"]) for o in cockpit.eye_objects([l1, l2])] == [("chair", 0.7)]
    l2["pose"] = dict(here, yaw_deg=90.0)                                        # the ball's spot out of l2's view
    assert sorted(o["name"] for o in cockpit.eye_objects([l1, l2])) == ["ball", "chair"]
    assert cockpit.eye_objects([dict(l1, objects=None)]) == []                    # a failed look places nothing
    # the words
    assert cockpit.signals_text({"signals": ["scan", "desc"]}) == "scan + description"
    assert cockpit.signals_text({"parts": {"scan": 0.9, "desc": None, "radio": None}}) == "scan only"
    assert cockpit.signals_text({"signals": ["desc"]}) == "description only"
    assert cockpit.signals_text({"by": "operator"}) == "the operator's word"
    assert cockpit.looking_again_text({"verdict": "new", "place_id": None}, "new") == \
        "place: NEW? looking again (no places yet)"
    amb = {"verdict": "ambiguous", "place_id": "place-aaaaaa", "name": "den", "confidence": 0.7,
           "second": {"place_id": "place-bbbbbb", "name": "study", "confidence": 0.66}}
    assert cockpit.looking_again_text(amb, "ambiguous") == "place: den? looking again (0.70, or study 0.66)"
    assert len(cockpit.looking_again_text(dict(amb, name="x" * 200), "2 changes")) <= 90


def test_the_occlusion_record_and_per_look_helpers_are_pure():
    """Review fixes (2026-09-25, round 2): floor_to_pixel inverts pixel_to_floor; a spot inside
    a box the eye drew (behind its foot) or behind a nearer lidar return is hidden, so a 'no' there
    is no answer; the names a place's notes keep as boxed-never-placed; a seen-before addition
    is placed, never declared; per_look keeps each look's parts."""
    import cockpit
    import cockpit_brains as cb
    for u, v in ((0.5, 0.9), (0.1, 0.7), (0.93, 0.6)):
        d, b = cb.pixel_to_floor(u, v)
        uu, vv = cb.floor_to_pixel(d, b)
        assert abs(uu - u) < 1e-9 and abs(vv - v) < 1e-9
        assert (uu, vv) == pytest.approx(_floor_to_uv(d * math.cos(math.radians(-b)), d * math.sin(math.radians(-b))))
    assert cb.floor_to_pixel(0.5, 180.0) is None                                 # behind the camera
    here = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    u1, v1 = cockpit.eye_pixel(here, 0.6, 0.0)
    u2, v2 = cockpit.eye_pixel(here, 1.2, 0.0)
    assert abs(u1 - 0.5) < 1e-9 and v2 < v1 and cockpit.eye_pixel(here, 0.6, 0.3)[0] < 0.5   # left = left
    # a box the eye drew around a thing standing 0.45 m ahead: its foot there, its top above the ball's spot
    foot = cockpit.eye_pixel(here, 0.45, 0.0)[1]
    ball_v = cockpit.eye_pixel(here, 0.8, 0.0)[1]
    box = [0.4, ball_v - 0.1, 0.6, foot]
    look = {"pose": here, "objects": [], "eye": {
        "box": {"seen": True, "asked": True, "x": 0.5, "y": 0.0, "confidence": 0.9, "bbox": box},
        "ball": {"seen": False, "asked": True}}}
    hid = cockpit.look_hidden(look)
    assert hid(0.8, 0.0) == "hidden behind the box"                              # behind its foot, inside its box
    assert hid(0.3, 0.0) is None and hid(0.8, 0.5) is None                       # in front of it / beside it
    assert cockpit.view_why(look, 0.8, 0.0) == "hidden behind the box" and cockpit.view_why(look, -1, 0) == "out of view"
    stored = [{"name": "ball", "x": 0.8, "y": 0.0}]
    spots = cockpit.place_spots(stored)
    assert cockpit.seen_map(look, spots)["ball"] is None                         # a 'no' it could not see: no answer
    assert cockpit.seen_map(dict(look, eye={"ball": {"seen": False}}), spots)["ball"] is False   # nothing in front
    chk = cockpit.CockpitSim._place_eye_check(stored, look)
    assert chk["missing"] == [] and [(o["name"], o["why"]) for o in chk["unobservable"]] == \
        [("ball", "said no, but hidden behind the box")]
    assert cockpit.eye_brief(look, spots)["why"] == {"ball": "said no, but hidden behind the box"}
    # a lidar return 0.4 m out on the ball's bearing (from the torso): something tall in the way
    lid = {"pose": here, "objects": [], "eye": {"ball": {"seen": False}}}
    assert cockpit.look_hidden(dict(lid, sweep=([0.0, 1.0], [0.4, 3.0])))(0.8, 0.0) == \
        "hidden (a lidar return in front)"
    assert cockpit.look_hidden(dict(lid, sweep=([0.0], [0.9])))(0.8, 0.0) is None          # behind the ball
    assert cockpit.look_hidden(dict(lid, sweep=([math.radians(10.0)], [0.4])))(0.8, 0.0) is None   # beside
    assert not cockpit.look_view(dict(lid, sweep=([0.0], [0.4])))(0.8, 0.0) and cockpit.look_view(lid)(0.8, 0.0)
    # the boxed-never-placed record: a place note, read back by name
    note = cockpit.PLACE_UNPLACED_NOTE
    q = {"notes": [{"text": "the charger is behind the door"}, {"text": note + "cup, lamp"},
                   {"text": note + "cup, red bottle"}]}
    assert cockpit.unplaced_names(q) == ["cup", "lamp", "red bottle"] and cockpit.unplaced_names(None) == []
    l1 = {"pose": here, "objects": [], "eye": {"cup": {"seen": True, "x": None, "y": None},
                                              "chair": {"seen": True, "x": 0.7, "y": 0.1},
                                              "lamp": {"seen": True, "x": None, "y": None}}}
    l2 = {"pose": here, "objects": [], "eye": {"lamp": {"seen": True, "x": 1.2, "y": 0.4}}}
    failed = {"pose": here, "objects": None, "eye": {"toy": {"seen": True, "x": None, "y": None}}}
    assert cockpit.boxed_unplaced([l1, l2, failed]) == ["cup"]                   # placed once = placed; failed = nothing
    # a name boxed here before but never placed: unplaced again -> present; placed now -> added, marked
    seen = {"pose": here, "objects": [], "eye": {"cup": {"seen": True, "x": None, "y": None, "confidence": 0.9},
                                                "red bottle": {"seen": True, "x": 0.6, "y": 0.1, "confidence": 0.9},
                                                "chair": {"seen": True, "x": 0.7, "y": -0.1, "confidence": 0.9}}}
    chk = cockpit.CockpitSim._place_eye_check([], seen, seen_before=["cup", "bottle"])
    assert [(o["name"], o["why"]) for o in chk["present"]] == [("cup", "boxed here before, never placed")]
    assert sorted((o["name"], bool(o.get("seen_before"))) for o in chk["added"]) == \
        [("chair", False), ("red bottle", True)]
    c = cockpit.split_seen_before({"added": [{"name": "bottle", "x": 0.6, "y": 0.1, "seen_before": True},
                                             {"name": "chair", "x": 0.7, "y": -0.1}],
                                   "missing": [], "moved": [],
                                   "pending": [{"name": "bottle", "change": "added", "seen_before": True},
                                               {"name": "toy", "change": "added"}]})
    assert [o["name"] for o in c["placed"]] == ["bottle"] and [o["name"] for o in c["added"]] == ["chair"]
    assert [o["name"] for o in c["pending"]] == ["toy"]
    # per_look keeps each look's parts, evidence, signals, runner-up and ranking (a replay needs them)
    pa, pb = {"scan": 1.0, "desc": 0.42, "radio": None}, {"scan": 0.97, "desc": 0.9, "radio": None}
    r1 = {"verdict": "ambiguous", "place_id": "place-aaaaaa", "name": "a", "confidence": 0.7, "parts": pa,
          "evidence": 1.0, "signals": ["scan", "desc"],
          "second": {"place_id": "place-bbbbbb", "name": "b", "confidence": 0.66, "parts": pb},
          "ranking": [{"place_id": "place-aaaaaa", "name": "a", "confidence": 0.7, "parts": pa},
                      {"place_id": "place-bbbbbb", "name": "b", "confidence": 0.66, "parts": pb}]}
    r2 = dict(r1, confidence=0.9, parts=pb, signals=None, verdict="known",
              ranking=[{"place_id": "place-aaaaaa", "name": "a", "confidence": 0.9, "parts": pb}])
    k = cockpit.combine_recognitions(r1, r2)
    assert [x["parts"] for x in k["per_look"]] == [pa, pb] and [x["confidence"] for x in k["per_look"]] == [0.7, 0.9]
    assert [x["signals"] for x in k["per_look"]] == [["scan", "desc"]] * 2       # a missing list: from the parts
    assert k["per_look"][0]["second"]["parts"] == pb and k["per_look"][0]["ranking"][1]["parts"] == pb
    assert k["per_look"][0]["evidence"] == 1.0
    assert cockpit.signal_list(k) == ["scan", "desc"] and cockpit.signal_list({"parts": pa}) == ["scan", "desc"]
    assert cockpit.signal_list({"by": "operator", "signals": ["scan"]}) == []


def test_the_embed_helper_answers_a_vector_or_none():
    """cockpit_brains.embed: llama-swap's OpenAI embeddings API (model 'embedding'), None on any
    failure. Faked here: a client object, and a port nothing listens on (no llama-swap traffic)."""
    import cockpit_brains as cb
    from types import SimpleNamespace
    seen = []

    class Client:
        class embeddings:                                    # noqa: N801 — the SDK's attribute name
            @staticmethod
            def create(model, input):                        # noqa: A002 — the SDK's argument name
                seen.append((model, input))
                return SimpleNamespace(data=[SimpleNamespace(embedding=[0.5, 0.25, 0.0])])
    assert cb.embed("a small room", client=Client()) == [0.5, 0.25, 0.0]
    assert seen == [("embedding", "a small room")]
    assert cb.embed("   ", client=Client()) is None and len(seen) == 1            # no text: no call

    class Broken:
        class embeddings:                                    # noqa: N801
            @staticmethod
            def create(model, input):                        # noqa: A002
                return SimpleNamespace(data=[SimpleNamespace(embedding=[float("nan")])])
    assert cb.embed("x", client=Broken()) is None
    t0 = time.monotonic()
    assert cb.embed("x", base_url="http://127.0.0.1:9/v1", api_key="k", timeout_s=2.0) is None
    assert time.monotonic() - t0 < 5.0
    assert (cb.EMBED_MODEL, cb.EMBED_TIMEOUT_S) == ("embedding", 5.0)


# ---------------------------------------------------------------- D057 review fixes (2026-09-25)
_BALL = {"kind": "ball", "pos": [0.8, 0.0], "radius_m": 0.05}
# four low boxes (under the puck's scan plane: the lidar sees the same room) that the eye names:
# the fake descriptions are ~0.76 cosine alike -> the room with them is 'known den' at ~0.84,
# a WRONG known the owner corrects
_BOXES = [{"kind": "box", "pos": p, "size": [0.1, 0.1, 0.05]}
          for p in ([0.52, 0.30], [0.61, -0.35], [1.5, 0.05], [1.09, 0.51])]
# ... and the guest room's walls LOOK different to the eye (a spec key only the fake eye reads):
# with the bench-measured DESC_COS_FLOOR 0.25 (was 0.55) the room above with only its boxes
# added reads 0.92 den / 1.0 guest — a twin the look alone can no longer split by MARGIN, as
# place_memory says (twins need the eye's presence answers). With this wording: guest -> den
# 0.84 (the wrong known), den <-> guest 0.87 once both are stored (known, margin 0.13).
_GUEST_LOOK = ("A guest bedroom with striped wallpaper, a woollen rug, long curtains, framed pictures "
               "and a reading lamp.")


def _failing_eye(sim):
    import cockpit_brains as cb

    async def acall(model, msgs, tools, max_tokens):
        raise cb.ModelFailure("HTTP 503: model loading")
    sim.brains._acall = acall


def test_failed_looks_declare_nothing_and_erase_nothing(place_client):
    """Review blocker: a look that failed was a look that saw nothing — two of them declared the
    ball missing and erased it from the place."""
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    assert c.post("/api/world", json={"name": "w1", "spec": _room_with(_BALL)}).json()["ok"]
    a = _recognise(c)
    assert a["verdict"] == "new" and [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]
    # a revisit with the eye failing: the lidar alone never says 'known' (place_memory), so no
    # place is claimed and nothing is compared
    _failing_eye(sim)
    assert c.post("/api/world", json={"name": "w1 again", "spec": _room_with(_BALL)}).json()["ok"]
    u = _recognise(c)
    assert u["verdict"] == "ambiguous" and "changes" not in u and "missing" not in u["text"]
    assert "scan only: the look failed" in u["detail"] and "503" in u["look_error"]
    assert "the look failed" in u["note"] and "lidar alone" in u["note"] and u["text"].endswith("[scan only]")
    assert u["signals"] == "scan only" and "[signals: scan only; confidence " in u["detail"]
    assert c.get("/api/state").json()["place"]["signals"] == "scan only"
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]
    # known again with a working eye; then an EDIT (out of the eye's view) with the eye failing:
    # the robot stays in its place, and a failed look is not a look that saw no ball
    _wire_fake_eye(sim)
    assert c.post("/api/world", json={"name": "w1 once more", "spec": _room_with(_BALL)}).json()["ok"]
    assert _recognise(c)["verdict"] == "known"
    _failing_eye(sim)
    n0 = len(sim.cmd_log)
    assert c.post("/api/world/add", json={"object": {"kind": "ball", "pos": [-0.9, 0.0], "radius_m": 0.05}}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["place_id"] == a["place_id"] and k["kept"] == "an edit" and k["looks"] == 1
    assert "changes" not in k and "missing" not in k["text"]                  # nothing compared, nothing declared
    assert not any(line.startswith("tool turn") for _t, line in sim.cmd_log[n0:])   # no second look for nothing
    assert "scan only: the look failed" in k["detail"] and "no change check" in k["note"]
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]   # the snapshot stays
    sit = c.post("/api/awareness", json={"refresh": True}).json()["situation"]["text"]
    assert "missing" not in sit
    # the same through a place check (the eye still failing)
    chk = c.portal.call(sim.place_check, "test")
    assert chk["ok"] and "changes" not in chk and chk["place_id"] == a["place_id"]
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]


def _guest_world():
    return dict(_room_with(_BALL, *_BOXES), look=_GUEST_LOOK)


def _den_and_a_wrong_known(c, sim):
    """room + ball -> new, named 'den'; the same walls with four low boxes -> KNOWN den (wrong),
    the boxes confirmed by two looks and written into den. -> (den id, den before, the answer)."""
    assert c.post("/api/world", json={"name": "den world", "spec": _room_with(_BALL)}).json()["ok"]
    d = _recognise(c)
    assert d["verdict"] == "new"
    assert c.post("/api/tool/name_place", json={"name": "den"}).json()["action"] == "renamed"
    before = sim.place_memory.get(d["place_id"])
    assert c.post("/api/world", json={"name": "guest world", "spec": _guest_world()}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["place_id"] == d["place_id"] and 0.75 <= k["confidence"] < 0.92, k
    assert k["looks"] == 2 and "box" in {o["name"] for o in k["changes"]["added"]}
    polluted = sim.place_memory.get(d["place_id"])
    assert polluted["visits"] == before["visits"] + 1 and "box" in {o["name"] for o in polluted["objects"]}
    assert sim.memory.world == d["place_id"]
    return d["place_id"], before, k


def _same_record(a, b):
    import numpy as np
    return (a["visits"] == b["visits"] and a["name"] == b["name"] and a["objects"] == b["objects"]
            and a["desc_texts"] == b["desc_texts"] and len(a["scan_sigs"]) == len(b["scan_sigs"])
            and all(np.array_equal(x, y) for x, y in zip(a["scan_sigs"], b["scan_sigs"])))


def test_a_wrong_known_corrected_as_new_is_taken_back(place_client):
    """Review: name_place(new=true) after a wrong 'known' left the other room's sample, objects
    and memory in the wrong place; the next visit was then a tie."""
    sim, c, tmp = place_client
    c.post("/api/awareness", json={"recognize": True})
    den, before, _k = _den_and_a_wrong_known(c, sim)
    r = c.post("/api/tool/name_place", json={"name": "guest room", "new": True}).json()
    assert r["ok"] and r["action"] == "stored" and r["place_id"] != den
    assert r["took_back"] == {"place_id": den, "name": "den", "how": "restored",
                              "records_moved": r["took_back"]["records_moved"]}
    assert r["took_back"]["records_moved"] >= 1 and "left as it was" in r["detail"]
    assert _same_record(sim.place_memory.get(den), before)                   # den exactly as before this visit
    assert sim.memory.world == r["place_id"]
    sim.flush_memories()
    time.sleep(0.2)
    assert "box" not in (tmp / f"{den}.json").read_text()                   # den's memory: no guest-room boxes
    assert "box" in (tmp / f"{r['place_id']}.json").read_text()             # moved to where they were seen
    # the next visit to the guest room: known guest room (den ~0.84 behind), not a tie
    assert c.post("/api/world", json={"name": "guest world", "spec": _guest_world()}).json()["ok"]
    g = _recognise(c)
    assert g["verdict"] == "known" and g["place_id"] == r["place_id"], g
    assert g["second"]["place_id"] == den and g["confidence"] - g["second"]["confidence"] >= 0.08


def test_the_owner_naming_another_place_after_a_wrong_known_settles_there(place_client):
    """Review: 'hey, I'm in the basement at home' after a wrong 'known den' renamed den to
    basement — den was gone and the two places merged by name for good."""
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    assert c.post("/api/world", json={"preset": "flat"}).json()["ok"]
    b = _recognise(c)
    assert b["verdict"] == "new"
    assert c.post("/api/chat", json={"text": "hey, I'm in the basement at home", "mode": "talk"}).json()
    den, before, _k = _den_and_a_wrong_known(c, sim)
    r = c.post("/api/chat", json={"text": "hey, I'm in the basement at home", "mode": "talk"}).json()
    res = r["trace"][0]["result"]
    assert res["action"] == "settled" and res["place_id"] == b["place_id"] and res["took_back"]["how"] == "restored"
    assert "I had taken it for 'den'" in r["reply"]
    assert sorted(p["name"] for p in sim.place_memory.places()) == ["basement", "den"]
    assert _same_record(sim.place_memory.get(den), before) and sim.memory.world == b["place_id"]


def test_the_owner_giving_a_new_name_after_a_wrong_known_stores_a_new_place(place_client):
    """Review: "I'm in the workshop" after a wrong 'known den' renamed den to workshop. Now a new
    place; den keeps its name and record; rename=true is how a place gets a new name."""
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    den, before, _k = _den_and_a_wrong_known(c, sim)
    w = c.post("/api/chat", json={"text": "hey, I'm in the workshop", "mode": "talk"}).json()["trace"][0]["result"]
    assert w["action"] == "stored" and w["name"] == "workshop" and w["place_id"] != den
    assert "rename=true" in w["detail"] and w["took_back"]["how"] == "restored"
    assert sorted(p["name"] for p in sim.place_memory.places()) == ["den", "workshop"]
    assert _same_record(sim.place_memory.get(den), before)
    # back in the den: known den again (the workshop's walls are the same, its look is not)
    assert c.post("/api/world", json={"name": "den world 2", "spec": _room_with(_BALL)}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["place_id"] == den, k
    # an explicit rename renames the recognised place; new and rename together are refused
    rn = c.post("/api/tool/name_place", json={"name": "study", "rename": True}).json()
    assert rn["action"] == "renamed" and rn["previous_name"] == "den" and rn["place_id"] == den
    assert c.post("/api/tool/name_place", json={"name": "x", "new": True, "rename": True}).json()["ok"] is False
    assert c.post("/api/tool/name_place", json={"name": "x", "rename": "yes"}).json()["ok"] is False


def test_this_is_completely_new_stores_one_place_per_visit(place_client):
    """Review: 'this is completely new' right after the robot stored the place made a duplicate,
    and every later visit was ambiguous between the two."""
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    assert c.post("/api/world", json={"preset": "room"}).json()["ok"]
    a = _recognise(c)
    assert a["verdict"] == "new"
    for _ in range(2):                                                         # twice: still one place
        r = c.post("/api/chat", json={"text": "hey, this is completely new", "mode": "talk"}).json()
        assert r["trace"][0]["result"]["action"] == "kept" and r["trace"][0]["result"]["place_id"] == a["place_id"]
    assert len(sim.place_memory.places()) == 1
    assert "reset" in c.post("/api/reset", json={}).json()["reply"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["place_id"] == a["place_id"] and k["confidence"] >= 0.75


def test_an_edit_keeps_the_bound_place(place_client):
    """Review: an edit (the robot was not moved) could enroll a new place and move the memory
    away (3 tall boxes -> 'new', the charger forgotten)."""
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    assert c.post("/api/world", json={"preset": "room"}).json()["ok"]
    a = _recognise(c)
    assert a["verdict"] == "new"
    assert c.post("/api/memory", json={"action": "remember", "name": "charger", "x": 0.5, "y": 0.2}).json()["ok"]
    for p in ([0.9, 0.35], [1.0, -0.3], [0.7, 0.6]):
        assert c.post("/api/world/add", json={"object": {"kind": "box", "pos": p, "size": [0.2, 0.2, 0.4]}}).json()["ok"]
    e = _recognise(c)
    assert e["verdict"] == "known" and e["place_id"] == a["place_id"] and e["kept"] == "an edit", e
    assert "the robot was not moved" in e["note"]
    assert len(sim.place_memory.places()) == 1 and sim.memory.world == a["place_id"]
    assert c.post("/api/tool/where_is", json={"name": "charger"}).json()["known"]


def test_no_tool_or_memory_record_names_the_world_while_recognising(place_client):
    sim, c, tmp = place_client
    c.post("/api/awareness", json={"recognize": True})
    assert c.post("/api/world", json={"name": "grandmas-attic", "spec": _room_with(_BALL)}).json()["ok"]
    _recognise(c)
    st = c.post("/api/tool/status", json={}).json()
    sc = c.post("/api/tool/scan_summary", json={}).json()
    assert "world" not in st and "world" not in sc and st["place"].startswith("place: ")
    rc = c.post("/api/tool/recall", json={"query": "world", "k": 20}).json()
    assert "grandmas" not in json.dumps(rc) and "grandmas" not in json.dumps(sim.memory.last(500))
    sim.flush_memories()
    time.sleep(0.2)
    for f in tmp.glob("*.json"):
        assert "grandmas" not in f.read_text(), f
    # the operator's feed keeps the name; recognition off, the tools have it again
    assert c.get("/api/state").json()["world"] == "grandmas-attic"
    c.post("/api/awareness", json={"recognize": False})
    assert c.post("/api/tool/status", json={}).json()["world"] == "grandmas-attic"


def test_the_situation_line_stays_short_with_a_place_clause():
    """Review: with recognition on the place clause (up to 90 chars) pushed the full-memory line to
    446 chars; the memory then names 2 objects and the eye quote is shorter."""
    import time as _t
    import place_memory as pm
    sim = _make_sim()
    sim.awareness["interval_s"] = 0.0
    _settle(sim, 100)
    sim.set_awareness(recognize=True)
    for name, x, y in (("ball", 0.5, 0.5), ("box", 1.0, -0.4), ("red chair", -0.8, 0.2), ("charger", 0.2, -1.1)):
        sim.memory.remember({"kind": "find", "text": name, "objects": [
            {"name": name, "x": x, "y": y, "confidence": 0.9}]})
    sim.memory.remember({"kind": "user", "text": "the kitchen is through the door on the left of the sofa, "
                                                 "past the rug and the two plant pots"})
    sim.brains.last_look = {"text": "A long description " * 12, "model": "lfm2.5-vl", "t": _t.time(), "pose": None}
    clause = ("place: master bedroom (0.88) — new here: chair, lamp; missing: rug; moved: box"
              + "x" * pm.VERDICT_MAX)[:pm.VERDICT_MAX]
    sim._place_text = lambda: clause
    s = sim.compose_situation(lid=_lid(ahead=1.18))
    assert len(clause) == 90 and s["text"].startswith(clause + ". at (")
    assert len(s["text"]) < 400, (len(s["text"]), s["text"])
    assert "lidar: nearest 1.18 m ahead; clear elsewhere" in s["text"] and "world '" not in s["text"]
    assert "+2 more" in s["text"]                                              # 2 objects named, not 3


def test_this_room_has_a_new_thing_checks_with_two_looks(place_client):
    """Review: talk mode said 'I will check for the new chair with two looks' and looked at nothing.
    Now it checks the named place: a change only when two looks agree, with the confidence."""
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    assert c.post("/api/world", json={"name": "br", "spec": _room_with(_BALL)}).json()["ok"]
    a = _recognise(c)
    assert c.post("/api/tool/name_place", json={"name": "master bedroom"}).json()["action"] == "renamed"
    # the next visit's own recognition runs with a failed eye (the lidar alone: unsure, nothing
    # compared); then the eye is back and the owner says where the robot is and what is new
    _failing_eye(sim)
    # the unsure recognition turns the robot ~30 deg left for its second look; the chair stands
    # where the check's two looks (from ~30 and ~60 deg) both see it
    chair = {"kind": "box", "pos": [0.55, 0.3], "size": [0.2, 0.2, 0.25]}
    assert c.post("/api/world", json={"name": "br 2", "spec": _room_with(_BALL, chair)}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "ambiguous" and "changes" not in k
    _wire_fake_eye(sim)
    n0 = len(sim.cmd_log)
    r = c.post("/api/chat", json={"text": "hey, this master bedroom has a new chair", "mode": "talk"}).json()
    assert [t["tool"] for t in r["trace"]] == ["name_place", "place_check"]
    assert r["trace"][0]["result"]["action"] == "settled" and r["trace"][0]["result"]["place_id"] == a["place_id"]
    chk = r["trace"][1]["result"]
    assert chk["kept"] == "a change check" and chk["looks"] == 2
    assert [o["name"] for o in chk["changes"]["added"]] == ["box"]           # the eye calls it a box
    assert "Two looks agree (place " in r["reply"] and "new here: box" in r["reply"]
    assert sum(1 for _t, line in sim.cmd_log[n0:] if line.startswith("tool turn")) == 1
    assert "box" in {o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]}
    assert len(sim.place_memory.places()) == 1


# ---------------------------------------------------------------- the change check by the EYE (2026-09-25)
# The prose only proposes questions; the eye's box answers them (Brains.detect, method bbox — what
# /api/look {find, method: "bbox"} asks): per name True (boxed) / False (asked, not there) / None
# (not asked, no answer, or its remembered spot out of view). A change is declared only when the
# second look, from 30 deg further, agrees; a new place is stored from two looks.
_CHAIR = {"kind": "box", "label": "chair", "pos": [0.6, 0.15], "size": [0.2, 0.2, 0.12]}   # low: lidar-blind
_TOY = {"kind": "box", "label": "toy", "pos": [0.75, 0.05], "size": [0.1, 0.1, 0.05]}


def _finds_since(sim, k):
    return [n for n, _p in sim.fake_finds[k:]]


def test_a_new_place_is_stored_from_two_looks_and_says_it_looks_again(place_client):
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    texts = []
    orig = sim.compose_situation

    def spy(*a, **k):
        r = orig(*a, **k)
        texts.append(r["text"])
        return r
    sim.compose_situation = spy
    n0 = len(sim.cmd_log)
    assert c.post("/api/world", json={"name": "w1", "spec": _room_with(_BALL)}).json()["ok"]
    a = _recognise(c)
    assert (a["verdict"], a["looks"]) == ("new", 2)
    assert sum(1 for _t, line in sim.cmd_log[n0:] if line.startswith("tool turn")) == 1
    # the situation line said so before the turn
    assert any(t.startswith("place: NEW? looking again (no places yet). at (") for t in texts), texts
    assert ("place", "place: NEW? looking again (no places yet)") in sim.events
    # two samples: the arrival (the ball straight ahead) and 30 deg further (on the right)
    q = sim.place_memory.get(a["place_id"])
    assert len(q["desc_texts"]) == 2
    assert "ball straight ahead" in q["desc_texts"][0] and "ball on the right" in q["desc_texts"][1]
    p = sim.place_memory.places()[0]
    assert p["samples"]["desc"] == 2 and p["samples"]["scan"] >= 1 and p["visits"] == 1
    # its first object is where the eye's box put it (both looks agree), not the prose's guess
    (ball,) = q["objects"]
    assert ball["name"] == "ball" and abs(ball["x"] - 0.8) < 0.05 and abs(ball["y"]) < 0.05
    assert [e["seen"] for e in a["eye_checks"]] == [{"ball": True}, {"ball": True}]
    assert a["signals"] == "no places to compare with" and "[signals: no places" in a["detail"]
    # a place that is NEW by a wide margin still gets its second look before it is stored
    texts.clear()
    assert c.post("/api/world", json={"preset": "obstacle course"}).json()["ok"]
    b = _recognise(c)
    assert (b["verdict"], b["looks"]) == ("new", 2) and b["confidence"] < 0.5
    assert any(t.startswith("place: NEW? looking again (best new place #1 ") for t in texts), texts
    assert len(sim.place_memory.get(b["place_id"])["desc_texts"]) == 2
    assert b["signals"] == "scan + description"


def test_a_planted_chair_is_added_only_when_two_looks_agree(place_client):
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True, "reactions": True})
    assert c.post("/api/world", json={"name": "w1", "spec": _room_with(_BALL)}).json()["ok"]
    a = _recognise(c)
    assert a["verdict"] == "new"
    # the chair planted where both looks see it: the eye boxes it twice -> declared, stored at its box
    sim._aw["last_react"] = -1e9
    k0 = len(sim.fake_finds)
    assert c.post("/api/world", json={"name": "w2", "spec": _room_with(_BALL, _CHAIR)}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["place_id"] == a["place_id"] and k["looks"] == 2, k
    assert [o["name"] for o in k["changes"]["added"]] == ["chair"] and k["pending"] == []
    assert k["changes"]["missing"] == [] and k["text"].endswith("— new here: chair")
    chair = k["changes"]["added"][0]
    assert abs(chair["x"] - 0.6) < 0.05 and abs(chair["y"] - 0.15) < 0.05
    assert sorted(_finds_since(sim, k0)) == ["ball", "ball", "chair", "chair"]   # the same questions, twice
    assert [e["seen"] for e in k["eye_checks"]] == [{"ball": True, "chair": True}] * 2
    assert "[signals: scan + description; confidence " in k["detail"] and k["signals"] == "scan + description"
    st = c.get("/api/state").json()["place"]
    assert st["signals"] == "scan + description" and st["confidence"] == k["confidence"]
    stored = {o["name"]: o for o in sim.place_memory.get(a["place_id"])["objects"]}
    assert set(stored) == {"ball", "chair"} and abs(stored["chair"]["x"] - 0.6) < 0.05
    assert any(e[0] == "react" and e[1].startswith("curious_question: new here: chair") for e in sim.events)
    # a toy the eye boxes once and then, from 30 deg further, says is not there: the looks
    # disagree -> pending, not declared, not stored
    asked = {"toy": 0}

    def flaky(name, _n):
        if name != "toy":
            return None
        asked["toy"] += 1
        return None if asked["toy"] == 1 else json.dumps({"seen": False, "what": "a bare floor"})
    _wire_fake_eye(sim, find=flaky)
    assert c.post("/api/world", json={"name": "w3", "spec": _room_with(_BALL, _CHAIR, _TOY)}).json()["ok"]
    t = _recognise(c)
    assert t["verdict"] == "known" and t["looks"] == 2 and asked["toy"] == 2, t
    assert t["changes"] == {"added": [], "missing": [], "moved": []}
    assert [(o["name"], o["change"]) for o in t["pending"]] == [("toy", "added")]
    assert "not declared" in t["detail"] and "new here" not in t["text"]
    assert "toy" not in {o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]}


def test_a_removed_ball_is_missing_only_when_both_looks_see_its_spot_and_say_no(place_client):
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    # a ball ahead-right: in the eye's view from the spawn, out of it after the 30 deg left turn
    ball_r = {"kind": "ball", "pos": [0.5, -0.33], "radius_m": 0.05}
    assert c.post("/api/world", json={"name": "w1", "spec": _room_with(ball_r)}).json()["ok"]
    a = _recognise(c)
    assert a["verdict"] == "new" and [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]
    # the ball gone: the first look says no where it could see it, the second cannot see the spot
    # (unobservable: not even asked) -> pending, never declared, still remembered
    assert c.post("/api/world", json={"name": "w2", "spec": _room_with()}).json()["ok"]
    m = _recognise(c)
    assert m["verdict"] == "known" and m["looks"] == 2, m
    assert m["changes"] == {"added": [], "missing": [], "moved": []}
    assert [(o["name"], o["change"]) for o in m["pending"]] == [("ball", "missing")]
    assert [e["seen"] for e in m["eye_checks"]] == [{"ball": False}, {"ball": None}]
    assert m["eye_checks"][1]["why"] == {"ball": "out of view"}
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]
    # the ball there after all, but the eye misses it once: the second look boxes it -> not missing
    first = {"n": 0}

    def miss_once(name, _n):
        if name == "ball":
            first["n"] += 1
            if first["n"] == 1:
                return json.dumps({"seen": False, "what": "a floor"})
        return None
    # an open floor with a wall behind the robot: the lidar sees it (an empty sweep is no sense, so
    # a lidar-empty world is never 'known'), the eye does not
    open_floor = {"base": "flat", "objects": [{"kind": "ball", "pos": [0.8, 0.0], "radius_m": 0.05},
                                              {"kind": "wall", "pos": [-0.8, 0.0], "len_m": 1.2, "yaw_deg": 90}]}
    assert c.post("/api/world", json={"name": "w3", "spec": open_floor}).json()["ok"]
    b = _recognise(c)
    assert b["verdict"] == "new", b                                   # another place, its ball ahead
    pid = b["place_id"]
    _wire_fake_eye(sim, find=miss_once)
    assert c.post("/api/world", json={"name": "w4", "spec": open_floor}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["place_id"] == pid and k["looks"] == 2, k
    assert k["changes"]["missing"] == [] and [(o["name"], o["change"]) for o in k["pending"]] == [("ball", "missing")]
    assert [e["seen"]["ball"] for e in k["eye_checks"]] == [False, True]
    assert "ball" in {o["name"] for o in sim.place_memory.get(pid)["objects"]}


def test_an_object_out_of_view_is_unobservable_and_never_missing(place_client):
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    assert c.post("/api/world", json={"name": "w1", "spec": _room_with(_BALL)}).json()["ok"]
    a = _recognise(c)
    assert c.post("/api/world", json={"name": "w1 again", "spec": _room_with(_BALL)}).json()["ok"]
    assert _recognise(c)["place_id"] == a["place_id"]                      # known: stored on an earlier visit
    r = c.portal.call(sim.brains.tool, "turn", {"deg": 90})                   # the ball now behind the eye's right
    assert r["ok"], r
    # the ball taken away while the robot faces the other way (an edit keeps the pose)
    k0, n0 = len(sim.fake_finds), len(sim.cmd_log)
    assert c.post("/api/world/add", json={"clear": True}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["kept"] == "an edit" and k["looks"] == 1, k
    assert k["changes"] == {"added": [], "missing": [], "moved": []} and k["pending"] == []
    assert k["eye_checks"][0]["seen"] == {"ball": None} and k["eye_checks"][0]["why"] == {"ball": "out of view"}
    assert "ball" not in _finds_since(sim, k0)                                # never asked: it could not be seen
    assert not any(line.startswith("tool turn") for _t, line in sim.cmd_log[n0:])
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]


@pytest.mark.parametrize("bad", ["error", "garbage"])
def test_an_eye_that_does_not_answer_declares_nothing(place_client, bad):
    """The description works, but every box question fails (the model errs) or comes back as
    prose: no answer is 'not there' — nothing missing, nothing pending, no second look."""
    import cockpit_brains as cb
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    assert c.post("/api/world", json={"name": "w1", "spec": _room_with(_BALL)}).json()["ok"]
    a = _recognise(c)
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]
    _wire_fake_eye(sim, find=lambda name, n: cb.ModelFailure("HTTP 503: model loading") if bad == "error"
                   else "I think there might be something there.")
    n0 = len(sim.cmd_log)
    assert c.post("/api/world", json={"name": "w2", "spec": _room_with()}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["looks"] == 1, k
    assert k["changes"] == {"added": [], "missing": [], "moved": []} and k["pending"] == []
    assert k["eye_checks"][0]["seen"] == {"ball": None}
    why = k["eye_checks"][0]["why"]["ball"]
    assert ("did not answer" in why and "503" in why) if bad == "error" else "did not parse" in why
    assert not any(line.startswith("tool turn") for _t, line in sim.cmd_log[n0:])
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]


# Review round 2 (2026-09-25): a thing the eye boxes but can never place is kept by name (not new on
# every visit); both looks stand on one spot, so what one look cannot see past hides the spot from
# the other too (never 'missing'); a two-look answer carries each look's own parts (per_look).
_CUP_ON_TABLE = "A small room with pale walls all around and two grey pillars. A cup stands on a table."


def _above_horizon_box(u0=0.45, u1=0.55):
    """A box whose bottom edge is above the horizon (a cup on a table): boxed, never placed."""
    import cockpit_brains as cb
    f = (cb.EYE_H_PX / 2) / math.tan(math.radians(cb.EYE_FOVY_DEG / 2))
    v_h = 0.5 - math.tan(math.radians(cb.EYE_PITCH_DEG)) * f / cb.EYE_H_PX
    return [round(u0 * 1000), round((v_h - 0.15) * 1000), round(u1 * 1000), round((v_h - 0.05) * 1000)]


def _on_table(*names):
    box = _above_horizon_box()

    def find(name, _n):
        if name in names:
            return json.dumps({"seen": True, "bbox": box, "confidence": 0.9, "what": f"a {name} on a table"})
        return None
    return find


def _unplaced_notes(sim, pid):
    import cockpit
    return [n for n in sim.place_memory.get(pid)["notes"] if n["text"].startswith(cockpit.PLACE_UNPLACED_NOTE)]


def test_a_thing_the_eye_boxes_but_never_places_is_not_new_on_every_visit(place_client):
    import cockpit
    sim, c, _ = place_client
    _wire_fake_eye(sim, find=_on_table("cup"))
    spec = dict(_room_with(_BALL), look=_CUP_ON_TABLE)
    c.post("/api/awareness", json={"recognize": True, "reactions": True})
    assert c.post("/api/world", json={"name": "kitchen", "spec": spec}).json()["ok"]
    a = _recognise(c)
    assert a["verdict"] == "new" and a["looks"] == 2
    q = sim.place_memory.get(a["place_id"])
    assert [o["name"] for o in q["objects"]] == ["ball"]                   # no floor spot: not stored ...
    assert cockpit.unplaced_names(q) == ["cup"] and a["boxed_never_placed"] == ["cup"]   # ... kept by name
    assert "boxed, never placed" in a["note"]
    # the same room, unchanged, twice: the cup is boxed again and counts as present — no change,
    # no second look, no reaction (it used to be 'new here: cup' on every visit)
    for k in range(2):
        sim._aw["last_react"] = -1e9
        n0 = len(sim.cmd_log)
        assert c.post("/api/world", json={"name": f"kitchen again {k}", "spec": spec}).json()["ok"]
        r = _recognise(c)
        assert r["verdict"] == "known" and r["place_id"] == a["place_id"], r
        assert r["changes"] == {"added": [], "missing": [], "moved": []} and r["pending"] == [], r
        assert r["looks"] == 1 and "new here" not in r["text"]
        assert r["eye_checks"][0]["seen"]["cup"] is True                  # asked, and boxed
        assert not any(line.startswith("tool turn") for _t, line in sim.cmd_log[n0:])
    assert not any(e[0] == "react" and "new here" in e[1] for e in sim.events)
    assert len(_unplaced_notes(sim, a["place_id"])) == 1                   # one note, not one per visit
    # a bottle put on the table: really new, and just as unplaceable -> declared once (two looks
    # agree), kept by name, and not new on the next visit
    _wire_fake_eye(sim, find=_on_table("cup", "bottle"))
    spec2 = dict(spec, look=_CUP_ON_TABLE + " A bottle stands beside it.")
    sim._aw["last_react"] = -1e9
    assert c.post("/api/world", json={"name": "kitchen + bottle", "spec": spec2}).json()["ok"]
    b = _recognise(c)
    assert b["verdict"] == "known" and b["looks"] == 2, b
    assert [o["name"] for o in b["changes"]["added"]] == ["bottle"] and b["text"].endswith("— new here: bottle")
    assert b["changes"]["added"][0].get("x") is None                       # declared, never placed
    assert any(e[0] == "react" and e[1].startswith("curious_question: new here: bottle") for e in sim.events)
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]
    assert b["boxed_never_placed"] == ["cup", "bottle"]
    sim._aw["last_react"] = -1e9
    assert c.post("/api/world", json={"name": "kitchen + bottle again", "spec": spec2}).json()["ok"]
    b2 = _recognise(c)
    assert b2["verdict"] == "known" and b2["looks"] == 1 and b2["changes"]["added"] == [], b2
    assert "new here" not in b2["text"]
    assert sum(1 for e in sim.events if e[0] == "react" and "new here" in e[1]) == 1
    # the cup comes down to the floor where both looks box it: placed now — stored at its box's
    # spot, never declared new (it was here before)
    cup_floor = {"kind": "box", "label": "cup", "pos": [0.7, 0.12], "size": [0.06, 0.06, 0.08]}
    _wire_fake_eye(sim, find=_on_table("bottle"))
    sim._aw["last_react"] = -1e9
    assert c.post("/api/world", json={"name": "kitchen, cup down", "spec": dict(spec2, objects=[
        dict(_BALL), cup_floor])}).json()["ok"]
    p = _recognise(c)
    assert p["verdict"] == "known" and p["looks"] == 2, p
    assert p["changes"] == {"added": [], "missing": [], "moved": []} and "new here" not in p["text"], p
    assert [o["name"] for o in p["now_placed"]] == ["cup"] and "placed now" in p["detail"]
    stored = {o["name"]: o for o in sim.place_memory.get(a["place_id"])["objects"]}
    assert set(stored) == {"ball", "cup"} and abs(stored["cup"]["x"] - 0.7) < 0.05
    assert sum(1 for e in sim.events if e[0] == "react" and "new here" in e[1]) == 1


def test_a_thing_hidden_from_both_looks_is_never_erased(place_client):
    """Both looks of a change check stand on one spot (the second turns 30 deg in place), so a
    ball hidden behind a new low box is hidden from both: two 'not there' answers prove nothing.
    Hidden = inside a box the eye drew (behind its foot), or behind a nearer lidar return."""
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True, "reactions": True})
    assert c.post("/api/world", json={"name": "w1", "spec": _room_with(_BALL)}).json()["ok"]
    a = _recognise(c)
    assert [o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]] == ["ball"]

    def ball_unseen(name, _n):
        return json.dumps({"seen": False, "what": "a box, no ball"}) if name == "ball" else None
    _wire_fake_eye(sim, find=ball_unseen)
    # 1. a low (12 cm: lidar-blind) wide box between the robot and the ball: the eye boxes it (the
    #    description names it) and the ball's spot lies inside that box in both looks
    screen = {"kind": "box", "pos": [0.45, 0.0], "size": [0.12, 0.4, 0.12]}
    sim._aw["last_react"] = -1e9
    assert c.post("/api/world", json={"name": "w2", "spec": _room_with(_BALL, screen)}).json()["ok"]
    k = _recognise(c)
    assert k["verdict"] == "known" and k["place_id"] == a["place_id"] and k["looks"] == 2, k
    assert k["changes"]["missing"] == [] and [o["name"] for o in k["changes"]["added"]] == ["box"], k
    assert [e["seen"]["ball"] for e in k["eye_checks"]] == [None, None]
    assert [e["why"]["ball"] for e in k["eye_checks"]] == ["said no, but hidden behind the box"] * 2
    assert "missing" not in k["text"] and k["text"].endswith("— new here: box")
    assert sorted(o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]) == ["ball", "box"]
    # 2. a tall narrow crate (0.3 m: the lidar sees it; not a thing the eye is asked about) put in
    #    front of the ball by an edit (the robot stays; a change check of this place): the lidar
    #    return in front of the ball's spot hides it — no answer, nothing missing, no second look.
    #    (With nothing in front, two 'no's are a missing ball: test_missing_is_declared_...)
    assert c.post("/api/world", json={"name": "w1 again", "spec": _room_with(_BALL)}).json()["ok"]
    _wire_fake_eye(sim)
    assert _recognise(c)["place_id"] == a["place_id"]
    _wire_fake_eye(sim, find=ball_unseen)
    crate = {"kind": "box", "label": "crate", "pos": [0.45, 0.0], "size": [0.1, 0.1, 0.3]}
    n0 = len(sim.cmd_log)
    assert c.post("/api/world/add", json={"object": crate}).json()["ok"]
    e = _recognise(c)
    assert e["verdict"] == "known" and e["kept"] == "an edit" and e["looks"] == 1, e
    assert e["changes"] == {"added": [], "missing": [], "moved": []} and e["pending"] == []
    assert e["eye_checks"][0]["seen"]["ball"] is None
    assert e["eye_checks"][0]["why"]["ball"] == "said no, but hidden (a lidar return in front)"
    assert not any(line.startswith("tool turn") for _t, line in sim.cmd_log[n0:])
    assert "ball" in {o["name"] for o in sim.place_memory.get(a["place_id"])["objects"]}


def test_a_two_look_answer_carries_each_looks_parts(place_client):
    """where_am_i after an ambiguous first look: per_look holds BOTH looks' own recognitions
    (confidence, parts, evidence, signals, runner-up, ranking) — the top-level parts are one
    look's only — and signal_list is the senses as a list."""
    sim, c, _ = place_client
    c.post("/api/awareness", json={"recognize": True})
    c.post("/api/world", json={"preset": "room"})
    first = _recognise(c)
    assert first["verdict"] == "new" and len(first["per_look"]) == 2           # two looks, no place to compare
    assert c.post("/api/place", json={"action": "name", "name": "den"}).json()["ok"]
    assert "reset" in c.post("/api/reset", json={}).json()["reply"]
    assert _recognise(c)["verdict"] == "known"
    assert c.post("/api/place", json={"action": "name", "name": "study", "new": True}).json()["ok"]
    assert "reset" in c.post("/api/reset", json={}).json()["reply"]
    u = _recognise(c)                                                           # two places that sense alike
    assert u["verdict"] == "ambiguous" and u["looks"] == 2, u
    assert u["signal_list"] == ["scan", "desc"] and u["signals"] == "scan + description"
    assert len(u["per_look"]) == 2
    for lk in u["per_look"]:
        assert lk["signals"] == ["scan", "desc"] and lk["evidence"] is not None
        assert lk["parts"]["scan"] is not None and lk["parts"]["desc"] is not None
        assert lk["second"]["place_id"] and lk["second"]["parts"]["scan"] is not None
        assert {e["name"] for e in lk["ranking"]} == {"den", "study"}
    assert u["confidence"] == pytest.approx(sum(x["confidence"] for x in u["per_look"]) / 2, abs=0.002)
    w = c.post("/api/tool/where_am_i", json={}).json()
    assert w["per_look"] == u["per_look"] and w["signal_list"] == ["scan", "desc"]
