"""Scene memory (sim/scene_memory.py) and the brains' memory tools — pure
Python, a fake clock, a fake eye and a fake goto: no MuJoCo, no model server.

What is pinned: sightings of one name within 0.25 m merge (a confidence-
weighted mean) and farther apart stay two; recall scores name match over
recency over closeness, and a specific query that matches nothing returns
nothing; the JSON file round-trips (and a corrupt one is moved aside, never
lost); the cap prunes the oldest LOW-value observations first; where_is ages
with the clock; find_object records the box geometry's map position;
go_back_to walks ordinary gotos to the standoff, refuses the unknown and the
vague, and goes ONTO a place pinned with 'X is here'; the talk brain routes
'where is the ball' / 'go back to the box' / 'remember that ...'; a chat turn
carries the cockpit's situation line in the user turn."""
import asyncio
import json
import math
import os
import sys
import time
from collections import deque

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
for sub in ("sim", "gait"):
    sys.path.insert(0, os.path.join(ROOT, sub))
sys.path.insert(0, ROOT)

import scene_memory as sm                                              # noqa: E402
import cockpit_brains as cb                                            # noqa: E402


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def mem(**kw):
    kw.setdefault("clock", Clock())
    return sm.SceneMemory("flat", **kw)


def sight(m, name, x, y, conf=0.9, kind="find", **extra):
    return m.remember({"kind": kind, "text": f"{name} seen", "pose": {"x": 0, "y": 0, "yaw_deg": 0},
                       "objects": [dict(name=name, x=x, y=y, confidence=conf, **extra)]})


# ------------------------------------------------------------------ names
def test_names_fold_articles_plurals_and_synonyms():
    assert sm.norm_name("The Red Balls") == "red ball"
    assert sm.norm_name("a cube") == "box" and sm.norm_name("boxes") == "box"
    assert sm.norm_name("the stairs") == "stairs" and sm.norm_name("steps") == "stairs"
    assert sm.same_thing("ball", "orange ball") and not sm.same_thing("red ball", "blue ball")
    assert not sm.same_thing("ball", "box") and not sm.same_thing("", "ball")


# ------------------------------------------------------------------ merging
def test_sightings_within_025_m_merge_farther_stay_two():
    m = mem()
    sight(m, "ball", 0.50, 0.50, conf=0.9)
    sight(m, "ball", 0.60, 0.50, conf=0.9)                 # 0.10 m away: the same ball
    objs = m.objects("ball")
    assert len(objs) == 1 and objs[0]["seen"] == 2
    assert abs(objs[0]["x"] - 0.55) < 1e-6                 # equal confidence: the mean
    sight(m, "box", 0.55, 0.50)                            # another name at the same spot: its own entry
    sight(m, "orange ball", 0.52, 0.48)                    # a more specific name merges and wins
    assert [o["name"] for o in m.objects("ball")] == ["orange ball"]
    sight(m, "ball", 1.50, 0.0)                            # 1 m away: a second ball
    assert len(m.objects("ball")) == 2 and len(m.objects()) == 3
    assert m.where_is("ball")["x"] == 1.5                  # where_is: the most recent


def test_merge_is_confidence_weighted_and_capped_per_name():
    m = mem()
    sight(m, "ball", 0.0, 0.0, conf=0.9)
    sight(m, "ball", 0.2, 0.0, conf=0.1)                   # a vague sighting barely moves it
    assert m.where_is("ball")["x"] < 0.03
    for i in range(sm.MAX_PER_NAME + 3):                   # many boxes, far apart
        sight(m, "box", -3.0 + 0.8 * i, 0.0)
    assert len(m.objects("box")) == sm.MAX_PER_NAME
    assert abs(min(o["x"] for o in m.objects("box")) - (-3.0 + 0.8 * 3)) < 1e-9   # the oldest went


def test_bad_observations_are_refused_or_dropped():
    m = mem()
    with pytest.raises(ValueError):
        m.remember({"kind": "gossip", "text": "x"})
    with pytest.raises(ValueError):
        m.remember("not a dict")
    r = m.remember({"kind": "find", "text": "t", "objects": [
        {"name": "ball", "x": float("nan"), "y": 0}, {"name": "", "x": 0, "y": 0},
        {"name": "ball", "x": 1e9, "y": 0}, "junk", {"name": "ball", "x": 0.1, "y": 0.2, "confidence": 7}]})
    assert len(r["objects"]) == 1 and r["objects"][0]["confidence"] == 1.0


# ------------------------------------------------------------------ recall
def test_recall_scores_name_then_recency_then_distance():
    clock = Clock()
    m = mem(clock=clock)
    sight(m, "ball", 3.0, 3.0)                             # far and old
    clock.t += 300
    m.remember({"kind": "look", "text": "mostly floor, a wall far ahead"})
    clock.t += 300
    sight(m, "ball", 0.2, 0.0)                             # near and new
    hits = m.recall("ball", k=5, pose={"x": 0, "y": 0, "yaw_deg": 0})
    assert [h["kind"] for h in hits] == ["find", "find"]   # the wall look does not match
    assert hits[0]["objects"][0]["x"] == 0.2 and hits[0]["score"] > hits[1]["score"]
    assert m.recall("giraffe") == []                       # specific + no match: nothing, not noise
    latest = m.recall("", k=1)
    assert latest[0]["objects"][0]["x"] == 0.2             # generic: the latest
    m.remember({"kind": "user", "text": "the charger is by the door"})
    assert m.recall("where is the charger")[0]["kind"] == "user"


def test_recall_prefers_the_closer_of_two_equal_matches():
    clock = Clock()
    m = mem(clock=clock)
    sight(m, "box", 2.0, 0.0)
    sight(m, "box", 0.3, 0.0)
    clock.t += 1
    near = m.recall("box", k=2, pose={"x": 0.0, "y": 0.0, "yaw_deg": 0})
    far = m.recall("box", k=2, pose={"x": 2.0, "y": 0.0, "yaw_deg": 0})
    assert near[0]["objects"][0]["x"] == 0.3 and far[0]["objects"][0]["x"] == 2.0


# ------------------------------------------------------------------ where_is + age
def test_where_is_ages_with_the_clock_and_forget():
    clock = Clock()
    m = mem(clock=clock)
    assert m.where_is("ball") is None and m.where_is("") is None
    sight(m, "ball", 0.5, 0.5, conf=0.8)
    clock.t += 125
    w = m.where_is("the ball")
    assert w["x"] == 0.5 and w["age_s"] == 125.0 and w["confidence"] == 0.8 and w["source"] == "find"
    assert sm.fmt_age(w["age_s"]) == "2 min ago"
    m.remember({"kind": "user", "text": "the ball rolls away a lot"})
    assert m.forget("ball") == 3                           # the object, its sighting, the note
    assert m.where_is("ball") is None and m.recall("ball") == []
    sight(m, "box", 1, 1)
    assert m.forget("all") == 2 and m.objects() == [] and m.last() == []


# ------------------------------------------------------------------ pruning
def test_cap_prunes_the_oldest_low_value_first():
    clock = Clock()
    m = mem(clock=clock, cap=10)
    for i in range(4):
        clock.t += 1
        m.remember({"kind": "user", "text": f"note {i}"})
    for i in range(20):
        clock.t += 1
        m.remember({"kind": "said", "text": f"chord {i}"})
    kinds = [o["kind"] for o in m.last(100)]
    assert len(kinds) == 10 and kinds.count("user") == 4   # the operator's notes survive
    said = [o["text"] for o in m.last(100, kind="said")]
    assert said[0] == "chord 19" and "chord 0" not in said # the newest chords stay, the oldest went


# ------------------------------------------------------------------ persistence
def test_persistence_round_trip_and_world_switch(tmp_path):
    clock = Clock()
    m = sm.SceneMemory("obstacle course", directory=str(tmp_path), clock=clock)
    sight(m, "ball", 0.5, 0.5, size_m=0.1)
    m.remember({"kind": "user", "text": "the charger is by the door"})
    m.flush()
    path = tmp_path / "obstacle_course.json"
    d = json.loads(path.read_text())
    assert d["version"] == 1 and d["world"] == "obstacle course" and len(d["observations"]) == 2
    m2 = sm.SceneMemory("obstacle course", directory=str(tmp_path), clock=clock)
    assert m2.where_is("ball")["size_m"] == 0.1 and m2.recall("charger")[0]["kind"] == "user"
    assert m2.where_is("ball")["earlier_session"] is True     # loaded from a file: an earlier session
    m2.set_world("flat")                                   # another world: empty, the old one kept on disk
    assert m2.objects() == [] and path.exists()
    m2.set_world("obstacle course")
    assert m2.where_is("ball") is not None
    m2.set_world("custom", carry=True)                     # an EDIT of the world: carried over
    assert m2.where_is("ball") is not None
    m2.flush()
    assert (tmp_path / "custom.json").exists()


def test_debounced_save_happens_off_thread(tmp_path):
    m = sm.SceneMemory("flat", directory=str(tmp_path))
    sight(m, "ball", 1, 1)
    t_end = time.monotonic() + 5
    while not (tmp_path / "flat.json").exists() and time.monotonic() < t_end:
        time.sleep(0.05)
    assert json.loads((tmp_path / "flat.json").read_text())["objects"][0]["name"] == "ball"


def test_a_corrupt_file_is_moved_aside_not_lost(tmp_path):
    (tmp_path / "flat.json").write_text("{ this is not json")
    logs = []
    m = sm.SceneMemory("flat", directory=str(tmp_path), log=logs.append)
    assert m.objects() == [] and any("did not load" in x for x in logs)
    assert [p.name for p in tmp_path.iterdir() if ".corrupt-" in p.name]


def test_map_and_summary():
    clock = Clock()
    m = mem(clock=clock)
    assert "nothing remembered" in m.summary()
    sight(m, "ball", 0.5, 0.5)
    m.remember({"kind": "look", "text": "x", "objects": [{"name": "wall", "x": 1.2, "y": 0, "confidence": 0.2}]})
    clock.t += 700
    mp = m.to_map()
    assert {o["name"] for o in mp} == {"ball", "wall"} and all(o["stale"] for o in mp)
    assert set(mp[0]) >= {"name", "x", "y", "confidence", "source", "age_s", "seen", "stale"}
    s = m.summary({"x": 0, "y": 0, "yaw_deg": 0})
    assert "ball 0.7 m ahead-left at (0.50, 0.50)" in s and "vague" in s   # the look-placed wall


# ------------------------------------------------------------------ text -> objects
def test_objects_from_a_look_description_are_vague_and_need_a_distance():
    pose = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    objs = sm.objects_from_description(
        "There is an orange ball on the left, about 0.5 m away. A grey box far ahead. No stairs in view. "
        "A wall somewhere.", pose)
    by = {o["name"]: o for o in objs}
    assert set(by) == {"ball", "box"}                      # stairs negated; the wall has no direction
    assert by["ball"]["confidence"] == 0.2 and by["ball"]["y"] > 0.2   # left = +y
    assert abs(by["box"]["x"] - 1.2) < 1e-6 and by["box"]["y"] == 0.0
    assert sm.objects_from_description("mostly floor", pose) == []


def test_parse_note_pins_here_or_coordinates():
    pose = {"x": 0.3, "y": -0.2, "yaw_deg": 10}
    o = sm.parse_note("the charger is here", pose)
    assert o["name"] == "charger" and (o["x"], o["y"]) == (0.3, -0.2) and o["anchor"] == "robot"
    o = sm.parse_note("The red chair is at (1.0, 0.25)", pose)
    assert o["name"] == "red chair" and (o["x"], o["y"]) == (1.0, 0.25) and "anchor" not in o
    assert sm.parse_note("the charger is by the door", pose) is None


# ============================================================ the brains' tools
class MemSim:
    """A fake cockpit: pose, a scripted eye, gotos that arrive (or a veto),
    and a real SceneMemory (RAM)."""

    def __init__(self, goto_result=None):
        self.frames = {"eye": (1, b"\xff\xd8fake")}
        self.events, self.logs, self.cmd_log = deque(), [], []
        self.gesture_busy = False
        self.mode, self.speed = "idle", 1.0
        self.p = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
        self.gotos, self.turns = [], []
        self.goto_result = goto_result
        self.memory = sm.SceneMemory("flat")
        self.gesture_names = ["wave"]
        self.lexicon = ["greeting", "confused"]
        self.situation_calls = 0

    def log(self, m):
        self.logs.append(m)

    def note_cmd(self, line):
        self.cmd_log.append((0.0, line))

    async def call(self, fn):
        return fn()

    def pose(self):
        return dict(self.p)

    def start_gesture(self, fn, total, name=None):
        self.turns.append(name)
        self.p["yaw_deg"] += 30.0 if "left" in name else -30.0
        return None

    async def tool_goto(self, x, y):
        self.gotos.append((x, y))
        if self.goto_result:
            return dict(self.goto_result, pose=self.pose())
        self.p.update(x=x, y=y)
        return {"ok": True, "stopped": "arrived", "pose": self.pose()}

    async def tool_stop(self):
        return {"ok": True}

    async def tool_status(self):
        return {"ok": True, "pose": self.pose()}

    async def situation_now(self):
        self.situation_calls += 1
        return {"text": f"at ({self.p['x']:.2f}, {self.p['y']:.2f}); lidar: nearest 0.62 m ahead-left"}


def brains(sim, answers=('{"seen": false}',)):
    q = list(answers)
    seen = []

    def complete(model, messages, tools, max_tokens):
        seen.append([dict(m) for m in messages])        # a copy: the loop appends to the list later
        if callable(q[0]):
            return q[0](model, messages)
        return {"content": q.pop(0) if len(q) > 1 else q[0], "tool_calls": []}
    b = cb.Brains(sim, conf_path=None, complete=complete, catalog=[])
    b._cat = (float("inf"), None, False)
    b.frame_wait_s = 0.0
    b.seen_messages = seen
    return b


def run(c):
    return asyncio.run(c)


# LFM2.5-VL's real box for the ball at map (0.7, 0.2) with the robot at the origin
# (test_find_object.py): near edge 0.58 m from the camera, true centre 0.63 m
BOX_FAR = '{"seen": true, "bbox": [275, 443, 375, 563], "confidence": 0.9, "what": "a yellow sphere"}'
NEAR = '{"seen": true, "bearing_deg": 5, "distance_m": 0.2, "confidence": 0.9, "what": "ball"}'


def test_find_object_remembers_the_box_geometry_in_the_map_frame():
    sim = MemSim()
    b = brains(sim, [BOX_FAR, NEAR])
    r = run(b.find_object("ball"))
    assert r["found"] and r["remembered"]["name"] == "ball"
    first = sim.memory.last(10, kind="find")[-1]            # oldest find record: the first sighting
    ob = first["objects"][0]
    # the first sighting's estimate of the ball centre vs the truth (0.7, 0.2)
    assert math.hypot(ob["x"] - 0.7, ob["y"] - 0.2) < 0.06, ob
    assert 0.05 < ob["size_m"] < 0.12 and ob["source"] == "find"
    assert "map_xy" in r["steps"][0]
    assert any("find ball: found" in o["text"] for o in sim.memory.last(10))


def test_find_estimate_sightings_stay_below_go_back_to():
    sim = MemSim()
    b = brains(sim, ['{"seen": true, "bearing_deg": 0, "distance_m": 0.8, "confidence": 0.95}', NEAR])
    run(b.find_object("ball", method="estimate"))
    w = sim.memory.where_is("ball")
    assert w["source"] == "find-estimate" and w["confidence"] <= 0.3
    assert run(b.tool("go_back_to", {"name": "ball"}))["ok"] is False


def test_where_is_and_go_back_to_walk_ordinary_gotos_to_the_standoff():
    sim = MemSim()
    b = brains(sim)
    sim.memory.remember({"kind": "find", "text": "ball", "objects": [
        {"name": "ball", "x": 2.0, "y": 0.0, "confidence": 0.9, "size_m": 0.1}]})
    w = run(b.tool("where_is", {"name": "the ball"}))
    assert w["known"] and w["x"] == 2.0 and w["direction"] == "ahead" and "2.00 m ahead" in w["detail"]
    r = run(b.tool("go_back_to", {"name": "ball"}))
    assert r["ok"] and r["arrived"] and len(sim.gotos) == 2           # 1.2 m, then the rest
    standoff = cb.GO_BACK_STANDOFF_M + 0.05
    assert abs(sim.p["x"] - (2.0 - standoff)) < 1e-3 and sim.p["y"] == 0.0
    assert all(line.startswith("tool goto") for _t, line in sim.cmd_log)   # replays repeat the gotos


def test_go_back_to_refuses_the_unknown_and_the_vague_and_reports_a_veto():
    sim = MemSim(goto_result={"ok": False, "stopped": "cliff", "detail": "VOID at 0 deg"})
    b = brains(sim)
    r = run(b.tool("go_back_to", {"name": "ball"}))
    assert r["ok"] is False and r["error"] == "not remembered" and sim.gotos == []
    sim.memory.remember({"kind": "look", "text": "a ball on the left, near", "objects": [
        {"name": "ball", "x": 0.5, "y": 0.3, "confidence": 0.2, "source": "look"}]})
    r = run(b.tool("go_back_to", {"name": "ball"}))
    assert r["ok"] is False and "vague" in r["error"] and sim.gotos == []
    sim.memory.remember({"kind": "find", "text": "box", "objects": [{"name": "box", "x": 1.0, "y": 0, "confidence": 0.9}]})
    r = run(b.tool("go_back_to", {"name": "box"}))
    assert r["ok"] is False and r["stopped"] == "cliff" and len(sim.gotos) == 1
    assert run(b.tool("go_back_to", {"name": "box"}, voice=True))["error"] == "voice_unconfirmed"


def test_remember_here_pins_the_pose_and_go_back_goes_onto_it():
    sim = MemSim()
    b = brains(sim)
    sim.p.update(x=0.4, y=-0.3)
    r = run(b.tool("remember", {"note": "the charger is here"}))
    assert r["ok"] and r["object"]["anchor"] == "robot"
    r = run(b.tool("remember", {"note": "the door is behind the sofa"}))
    assert r["ok"] and r["object"] is None and "note" in r
    sim.p.update(x=0.0, y=0.0)
    r = run(b.tool("go_back_to", {"name": "charger"}))
    assert r["ok"] and (sim.p["x"], sim.p["y"]) == (0.4, -0.3)       # onto the spot, no standoff
    rc = run(b.tool("recall", {"query": "door"}))
    assert rc["found"] == 1 and "sofa" in rc["entries"][0]["text"]
    assert run(b.tool("where_is", {"name": "door"}))["known"] is False
    assert run(b.tool("forget", {"name": "charger"}))["records"] >= 2
    assert run(b.tool("where_is", {"name": "charger"}))["known"] is False


def test_memory_tools_without_a_memory_say_so():
    class NoMem(MemSim):
        memory = None

        def __init__(self):
            super().__init__()
            del self.memory
    b = brains(NoMem())
    for tool, args in (("where_is", {"name": "ball"}), ("go_back_to", {"name": "ball"}),
                       ("remember", {"note": "x"}), ("recall", {}), ("forget", {"name": "all"})):
        r = run(b.tool(tool, args))
        assert r["ok"] is False and "no scene memory" in r["error"], tool
    assert "Situation" not in b.system_for()


def test_talk_mode_routes_memory_phrasings():
    sim = MemSim()
    b = brains(sim)
    sim.memory.remember({"kind": "find", "text": "box", "objects": [{"name": "box", "x": 1.0, "y": 0.5, "confidence": 0.9}]})
    r = run(b.chat("where is the box", mode="talk"))
    assert r["trace"][0]["tool"] == "where_is" and "(1.00, 0.50)" in r["reply"]
    r = run(b.chat("pebble, go back to the box", mode="talk", source="voice"))
    assert r["trace"][0]["tool"] == "go_back_to" and r["trace"][0]["result"]["arrived"]
    r = run(b.chat("go back to the box", mode="talk", source="voice"))       # no wake word: no walking
    assert r["motion_blocked"] and len(sim.gotos) == 1
    r = run(b.chat("remember that the charger is by the door", mode="talk"))
    assert r["trace"][0]["tool"] == "remember" and "as a note" in r["reply"]
    r = run(b.chat("what do you remember about the charger", mode="talk"))
    assert "by the door" in r["reply"]
    r = run(b.chat("forget everything", mode="talk"))
    assert "forgot all" in r["reply"] and sim.memory.objects() == []


def test_a_chat_turn_carries_the_situation_in_the_user_turn_only():
    sim = MemSim()
    b = brains(sim, [lambda model, messages: {"content": "ok", "tool_calls": []}])
    run(b.chat("what's around you?", mode="local"))
    run(b.chat("and now?", mode="local"))
    last = b.seen_messages[-1]
    assert last[-1]["content"].startswith("Situation: at (0.00, 0.00)") and last[-1]["content"].endswith("and now?")
    assert last[1]["content"] == "what's around you?"                # history: the operator's words only
    assert "Situation" in last[0]["content"] and "where_is" in last[0]["content"]   # MEMORY_NOTE
    assert sim.situation_calls == 2
    assert "remember" in {t["function"]["name"] for t in b.tools_for()}


def test_look_is_remembered_and_places_named_objects_vaguely():
    sim = MemSim()
    b = brains(sim, ["An orange ball on the left, close to the robot. Open floor ahead."])
    r = run(b.look())
    assert r["ok"] and r["placed"] == ["ball"] and b.last_look["text"].startswith("An orange ball")
    w = sim.memory.where_is("ball")
    assert w["source"] == "look" and w["confidence"] == 0.2
    run(b.look(prompt="Does the floor end ahead? yes or no"))       # a custom question is not a scene
    assert len(sim.memory.last(10, kind="look")) == 1


# ============================================================ review 2026-09-24 fixes
def test_a_hand_edited_file_loads_normalised_and_a_bad_seq_is_moved_aside(tmp_path):
    (tmp_path / "flat.json").write_text(json.dumps({
        "objects": [{"name": "ball", "x": 1, "y": 1},                          # no t / n / w / confidence
                    {"name": "box", "x": 0.5, "y": 0, "confidence": "high", "seq": "x"},
                    {"name": "far", "x": 1e9, "y": 0}],                         # off the floor: dropped
        "observations": [{"kind": "look", "text": "no objects key"},
                         {"kind": "find", "objects": "not a list", "t": "soon"}]}))
    clock = Clock()
    m = sm.SceneMemory("flat", directory=str(tmp_path), clock=clock)
    assert {o["name"] for o in m.to_map()} == {"ball", "box"}
    w = m.where_is("ball")
    assert w["confidence"] == 0.2 and w["seen"] == 1          # no saved confidence: vague, never a goal
    assert "ball" in m.summary({"x": 0, "y": 0, "yaw_deg": 0}) and m.recall("") and m.recall("objects")
    (tmp_path / "flat.json").write_text(json.dumps({"objects": 5}))   # the wrong shape: moved aside
    logs = []
    m2 = sm.SceneMemory("flat", directory=str(tmp_path), log=logs.append)
    assert m2.objects() == [] and any("did not load" in x for x in logs)


def test_forget_that_forgets_nothing_only_all_wipes_and_a_wipe_keeps_a_bak(tmp_path):
    m = sm.SceneMemory("flat", directory=str(tmp_path))
    sight(m, "ball", 0.5, 0.5)
    m.remember({"kind": "user", "text": "the charger is here", "pose": {"x": 0, "y": 0},
                "objects": [{"name": "charger", "x": 0, "y": 0, "confidence": 0.9, "source": "user"}]})
    for word in ("that", "the", "those", "my stuff", "it", "", "this one"):
        assert m.forget(word) == 0 and len(m.objects()) == 2, word
        assert sm.forget_scope(word)[0] == "unclear"
    assert sm.forget_scope("everything") == ("all", None) and sm.forget_scope("the ball") == ("one", "ball")
    m.flush()
    assert m.forget("all") == 4 and m.objects() == []
    for _ in range(100):
        if (tmp_path / "flat.json.bak").exists():
            break
        time.sleep(0.01)
    bak = json.loads((tmp_path / "flat.json.bak").read_text())
    assert {o["name"] for o in bak["objects"]} == {"ball", "charger"}


def test_a_vague_sighting_neither_moves_nor_hides_a_confident_find():
    m = mem()
    sight(m, "ball", 0.5, 0.5, conf=0.95)
    sight(m, "ball", 0.6, 0.45, conf=0.2, kind="look", source="look")         # merges: counted only
    w = m.where_is("ball")
    assert (w["x"], w["y"], w["confidence"], w["source"], w["seen"]) == (0.5, 0.5, 0.95, "find", 2)
    sight(m, "ball", 0.866, -0.5, conf=0.2, kind="look", source="look")       # a separate, newer vague one
    w = m.where_is("ball")
    assert (w["x"], w["confidence"]) == (0.5, 0.95) and w["also_seen"]["source"] == "look"
    sight(m, "ball", 0.55, 0.5, conf=0.6)                                     # comparable: moves, weighted
    w = m.where_is("ball")
    assert 0.5 < w["x"] < 0.55 and 0.6 < w["confidence"] < 0.95 and w["source"] == "find"


def test_stale_is_flagged_everywhere_and_a_new_epoch_makes_old_sightings_earlier_session():
    clock = Clock()
    m = mem(clock=clock)
    sight(m, "ball", 0.5, 0.5)
    m.remember({"kind": "user", "text": "home is here", "objects": [
        {"name": "home", "x": 0, "y": 0, "confidence": 0.9, "source": "user", "anchor": "robot"}]})
    clock.t += 900
    assert m.where_is("ball")["stale"] and not m.where_is("home")["stale"]    # a place never goes stale
    assert "STALE" in m.summary({"x": 0, "y": 0, "yaw_deg": 0})
    assert "stale" in m.summary({"x": 0, "y": 0, "yaw_deg": 0}, short=True)
    m2 = mem(clock=Clock())
    sight(m2, "ball", 0.5, 0.5)
    m2.clock.t += 5
    m2.new_epoch("reset", spawn={"x": 0.0, "y": 0.0})
    w = m2.where_is("ball")
    assert w["earlier_session"] and w["stale"]
    st = m2.where_is("start")
    assert st["source"] == "spawn" and st["anchor"] == "robot" and not st["stale"]
    assert "start" not in m2.summary({"x": 0, "y": 0, "yaw_deg": 0}, short=True)


def test_positions_are_bounded_by_the_floor_and_filenames_never_collide():
    m = mem()
    sight(m, "ball", 50, 50)
    sight(m, "ball", 6.5, 0)
    assert m.objects() == []
    assert sm.SceneMemory.filename("room") == "room.json"
    assert sm.SceneMemory.filename("obstacle course") == "obstacle_course.json"   # the existing files
    assert sm.SceneMemory.filename("a/b") != sm.SceneMemory.filename("a_b")
    assert len(sm.SceneMemory.filename("x" * 200)) < 90


def test_a_world_switch_reads_a_save_still_in_flight(tmp_path):
    m = sm.SceneMemory("flat", directory=str(tmp_path))
    sight(m, "ball", 0.5, 0.5)
    m.set_world("room")                  # flat's save runs off-thread ...
    m.set_world("flat")                  # ... and coming straight back must still see it
    assert m.where_is("ball") is not None
    m.flush()


def test_parse_note_pins_spots_by_other_phrasings():
    pose = {"x": 0.3, "y": -0.2, "yaw_deg": 10}
    for note, name in (("this spot as home", "home"), ("call this spot home", "home"),
                       ("mark here as the dock", "dock"), ("this is the kitchen", "kitchen"),
                       ("this spot is my charging dock", "charging dock")):
        o = sm.parse_note(note, pose)
        assert o and o["name"] == name and o["anchor"] == "robot" and (o["x"], o["y"]) == (0.3, -0.2), note
    assert sm.parse_note("this is it", pose) is None
    assert sm.parse_note("this is a note about the thing", pose) is None


# the ball ~3 m out: its box bottom at v = 0.36, where a pixel is ~0.1 m
BOX_HORIZON = '{"seen": true, "bbox": [480, 340, 520, 360], "confidence": 0.95, "what": "a ball"}'


def test_a_far_find_sighting_is_remembered_as_vague():
    sim = MemSim()
    b = brains(sim, [BOX_HORIZON, NEAR])
    run(b.find_object("ball"))
    first = sim.memory.last(10, kind="find")[-1]["objects"][0]
    assert first["confidence"] <= cb.FIND_FAR_CONF < sm.GO_MIN_CONF
    assert first["x"] > 3.0 and first["source"] == "find"                  # a 3 m box: a hint, not a goal
    r = run(b.tool("go_back_to", {"name": "ball"}))                        # (NEAR is a typed estimate: vague too)
    assert r["ok"] is False and "vague" in r["error"]
    assert b.last_find["name"] == "ball" and b.last_find["found"]


def test_memory_tools_refuse_off_floor_pins_forget_that_and_voice_wipes():
    sim = MemSim()
    b = brains(sim)
    r = run(b.tool("remember", {"note": "the ball is at (50, 50)"}))
    assert r["ok"] is False and "off the floor" in r["error"]
    r = run(b.mem_remember(name="ball", x=150, y=0))
    assert r["ok"] is False and sim.memory.objects() == []
    run(b.tool("remember", {"note": "the charger is here"}))
    r = run(b.tool("forget", {"name": "that"}))
    assert r["ok"] is False and "forget what" in r["error"] and sim.memory.where_is("charger")
    r = run(b.tool("forget", {"name": "all"}, voice=True))
    assert r["error"] == "voice_unconfirmed" and sim.memory.where_is("charger")
    assert run(b.tool("forget", {"name": "all"}))["records"] >= 2


def test_go_back_to_refuses_a_sighting_from_before_the_reset_and_flags_a_stale_one():
    sim = MemSim()
    sim.memory = sm.SceneMemory("flat", clock=Clock())
    b = brains(sim)
    sight(sim.memory, "ball", 1.0, 0.0)
    sim.memory.clock.t += 900                                            # 15 min: stale, same session
    w = run(b.tool("where_is", {"name": "ball"}))
    assert w["stale"] and "STALE" in w["detail"]
    r = run(b.tool("go_back_to", {"name": "ball"}))
    assert r["ok"] and r["stale"] and "STALE" in r["detail"]
    sim.memory.new_epoch("reset", spawn={"x": 0.0, "y": 0.0})           # the world respawned its bodies
    r = run(b.tool("go_back_to", {"name": "ball"}))
    assert r["ok"] is False and "before the last reset" in r["error"]
    n = len(sim.gotos)
    r = run(b.tool("go_back_to", {"name": "start"}))                     # the spawn pin: onto it
    assert r["ok"] and len(sim.gotos) == n + 1 and sim.gotos[-1] == (0.0, 0.0)
