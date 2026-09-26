"""place_bench — the case table, the bench rooms (added presets only), the
opaque world names, reading the place payload, the scoring (confusion
matrix, change detection, the name-leak flag), the Markdown, the fence's
rules (and a live dry-run fence on loopback), the owner-port refusal and a
--help smoke test. No cockpit, no GPU, no llama-swap.

THE NAME-SWAP PROOF (test_*_name_swap_*): the bench's own trial loop
(run_trial) drives two fake cockpits that follow sim/cockpit.py's D057
protocol (recognition after every spawn; its own second look after a 30 deg
turn when unsure or new, or when a look shows a change; two looks combined
and a visit committed by the cockpit's own pure functions,
combine_recognitions and apply_confirmed; name_place / forget_place /
where_am_i / POST /api/place). One recognises places from what the robot
senses, BOTH senses: a real MuJoCo lidar sweep of the staged world at the
robot's pose (world_builder + sim_lidar, CPU only) and a fake eye's
description of what is in its view (test_awareness's fake eye: the base's
words, then each object's kind, side and distance), embedded as a hashed bag
of words, fed to the real place_memory.PlaceMemory; the change check asks
that eye about each name (place_memory.checked_objects: True / False / None
per name) and two looks must agree. The other cheats: it knows a place by
the world's NAME. The bench scores the first right on both swapped-name
cases and flags the second as a name leak on both — so a cockpit whose
recognition leaned on the world name could not pass this bench.
What that proves is the bench's staging and scoring and place_memory on real
sweeps — NOT that sim/cockpit.py holds together: the fake eye is perfect and
deterministic (the same view, the same words: cos 1.0, where the real run
measured the look embedding separating nothing), boxes and places everything
in its view (the cockpit's "boxed, not placed" path never occurs) and never
contradicts its own words. The cockpit itself is test_awareness's. One
lidar-empty case runs through the fake too (flat, stairs, rubble field: never
"known" as one another)."""
import copy
import json
import math
import os
import subprocess
import sys

import httpx
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for sub in ("sim", "gait"):
    sys.path.insert(0, os.path.join(ROOT, sub))
sys.path.insert(0, ROOT)

import place_bench as pb                                               # noqa: E402
import place_memory as pm                                              # noqa: E402
import vision_bench as vb                                              # noqa: E402
from world_builder import PRESETS                                      # noqa: E402

HAVE_MJ = os.path.exists(os.path.join(ROOT, "sim", "pebble.xml"))
try:
    import mujoco                                                      # noqa: F401
except ImportError:                                                    # pragma: no cover
    HAVE_MJ = False
needs_mujoco = pytest.mark.skipif(not HAVE_MJ, reason="needs mujoco and sim/pebble.xml")

# the nine presets as they were before the bench's rooms were added (byte-identical specs)
ORIGINAL_PRESETS = {
    "flat": {"base": "flat"},
    "room": {"base": "room"},
    "cliff": {"base": "cliff"},
    "obstacle course": {"base": "flat", "objects": [
        {"kind": "box", "pos": [0.7, 0.15], "size": [0.15, 0.15, 0.06]},
        {"kind": "wall", "pos": [1.2, 0.1], "len_m": 0.8, "yaw_deg": 90},
        {"kind": "ramp", "pos": [0.4, -0.6], "len_m": 0.45, "rise_m": 0.05, "width_m": 0.5,
         "landing_m": 0.2, "yaw_deg": 0},
        {"kind": "ball", "pos": [0.5, 0.5], "radius_m": 0.05}]},
    "rubble field": {"base": "flat", "objects": [
        {"kind": "rubble", "pos": [0.9, 0.0], "radius_m": 0.5, "n": 30, "size_m": 0.03}]},
    "rough terrain": {"base": "flat", "terrain": {"kind": "rough", "amp_m": 0.02, "size_m": 1.6,
                                                   "pos": [1.3, 0.0], "seed": 1}},
    "stairs": {"base": "flat", "objects": [
        {"kind": "stairs", "pos": [0.6, 0.0], "steps": 4, "rise_m": 0.015, "run_m": 0.15}]},
    "slope 8 deg": {"base": "flat", "gravity_tilt_deg": 8.0, "gravity_tilt_dir_deg": 0.0},
    "icy floor": {"base": "flat", "friction": 0.35},
}
BENCH_PRESETS = ["room a", "room b", "room c", "room d", "room a + chair", "room b + chair",
                 "room c + chair", "room c - ball"]
CASE = {c["id"]: c for c in pb.case_table()}


# ------------------------------------------------------------------ the presets
def test_existing_presets_are_untouched_and_first():
    keys = list(PRESETS)
    assert keys[:len(ORIGINAL_PRESETS)] == list(ORIGINAL_PRESETS)
    for k, spec in ORIGINAL_PRESETS.items():
        assert json.dumps(PRESETS[k], sort_keys=True) == json.dumps(spec, sort_keys=True), k
    assert keys[len(ORIGINAL_PRESETS):] == BENCH_PRESETS


def test_changed_rooms_differ_by_exactly_one_object():
    def objs(name):
        return [json.dumps(o, sort_keys=True) for o in PRESETS[name].get("objects") or []]
    for room in ("a", "b", "c"):
        base, chair = objs(f"room {room}"), objs(f"room {room} + chair")
        assert chair[:-1] == base and len(chair) == len(base) + 1
        added = json.loads(chair[-1])
        assert added["kind"] == "box" and added["size"][2] >= 0.2        # a chair the lidar sees too
        assert PRESETS[f"room {room} + chair"]["base"] == PRESETS[f"room {room}"]["base"]
    gone = [o for o in PRESETS["room c"]["objects"] if json.dumps(o, sort_keys=True) not in objs("room c - ball")]
    assert [o["kind"] for o in gone] == ["ball"]
    # deep copies: editing one preset's object never edits another's
    PRESETS["room c - ball"]["objects"][0]["pos"][0] += 0.0
    assert PRESETS["room c"]["objects"][0] is not PRESETS["room c - ball"]["objects"][0]


@pytest.mark.parametrize("preset,kind", [("room a + chair", "box"), ("room b + chair", "box"),
                                         ("room c + chair", "box"), ("room c", "ball")])
def test_the_changed_thing_is_in_view_at_spawn_and_after_the_second_look(preset, kind):
    obj = PRESETS[preset]["objects"][-1] if kind == "box" else \
        [o for o in PRESETS[preset]["objects"] if o["kind"] == "ball"][0]
    assert obj["kind"] == kind
    for yaw in (0.0, pb.SECOND_LOOK_DEG):
        t = vb.truth({"x": 0.0, "y": 0.0, "yaw_deg": yaw}, obj)
        assert vb.in_view(t), (preset, yaw, t)
        assert t["near_m"] < 2.0


# ------------------------------------------------------------------ the case table
def test_case_table_is_the_three_verdicts_plus_the_name_swap():
    ids = [c["id"] for c in pb.CASES]
    assert len(ids) == len(set(ids))
    assert CASE["a same pose"]["expect"] == "known" and CASE["a same pose"]["preset"] == "room a"
    assert CASE["d never seen"]["expect"] == "new" and "d" not in pb.ENROL_ROOMS
    mv = CASE["a moved + turned"]["move"]
    assert math.hypot(mv["x"], mv["y"]) >= 0.2 and abs(mv["yaw_deg"]) >= 45
    assert CASE["b + chair"]["changes"] == {"added": [PRESETS["room b + chair"]["objects"][-1]], "missing": []}
    ball = [o for o in PRESETS["room c"]["objects"] if o["kind"] == "ball"]
    assert CASE["c - ball"]["changes"] == {"added": [], "missing": ball}
    for cid in ("a same pose", "b named as a", "a named as b", "a moved + turned", "d never seen"):
        assert CASE[cid]["changes"] == {"added": [], "missing": []}, cid
    # the swap: each borrows the OTHER room's enrolment world name
    assert (CASE["b named as a"]["room"], CASE["b named as a"]["world_name_of"]) == ("b", "a")
    assert (CASE["a named as b"]["room"], CASE["a named as b"]["world_name_of"]) == ("a", "b")
    # every enrolled room's changed visit is its last visit (a confirmed change updates the place)
    for i, c in enumerate(pb.CASES):
        if CASE[c["id"]]["changes"]["added"] or CASE[c["id"]]["changes"]["missing"]:
            assert all(later["room"] != c["room"] for later in pb.CASES[i + 1:]), c["id"]


def test_case_table_refuses_nonsense():
    with pytest.raises(ValueError):
        pb.case_table([dict(id="x", room="a", preset="no such world", expect="known")])
    with pytest.raises(ValueError):
        pb.case_table([dict(id="x", room="d", preset="room d", expect="known")])   # d is never enrolled
    with pytest.raises(ValueError):
        pb.case_table([dict(id="x", room="a", preset="room a", expect="maybe")])
    with pytest.raises(ValueError):
        pb.case_table([dict(id="x", room="a", preset="room a", expect="known", world_name_of="d")])


def test_world_names_and_specs_are_fresh_every_visit():
    names = {pb.world_name() for _ in range(50)}
    assert len(names) == 50 and all(pb.WORLD_NAME_RE.match(n) for n in names)
    for n in names:
        assert not any(lab in n.split("-")[1:] for lab in pb.ROOMS)      # nothing but hex after pb-
    a, b = pb.staged_spec("room a"), pb.staged_spec("room a")
    assert a["visit"] != b["visit"] and a["base"] == "room"
    s = pb.staged_spec("room c")
    s["objects"][0]["pos"][0] = 99.0
    assert PRESETS["room c"]["objects"][0]["pos"][0] != 99.0                # a deep copy


# ------------------------------------------------------------------ reading the place payload
def _rec(verdict="known", pid="place-aaaaaa", name="den", conf=0.91, **kw):
    r = {"verdict": verdict, "place_id": pid, "name": name, "confidence": conf,
         "second": {"place_id": "place-bbbbbb", "name": "workshop", "confidence": 0.42}}
    r.update(kw)
    return r


def test_parse_place_shapes():
    now = 1000.0
    p = pb.parse_place(dict(_rec(), t=now), now=now)
    assert p["verdict"] == "known" and p["name"] == "den" and p["confidence"] == 0.91
    assert p["second"]["name"] == "workshop" and p["changes"] is None and p["t"] == now
    # nested, with changes beside it
    p = pb.parse_place({"ok": True, "place": _rec(), "changes": {"added": [{"name": "Cube"}], "missing": [],
                                                                  "moved": [], "pending": [{"name": "ball"}]}})
    assert p["changes"] == {"added": ["box"], "missing": [], "moved": []} and p["pending"] == ["ball"]
    # {confirmed: {...}, pending: [...]}
    p = pb.parse_place(_rec(diff={"confirmed": {"missing": ["balls"]}, "pending": ["chair"]}))
    assert p["changes"]["missing"] == ["ball"] and p["pending"] == ["chair"]
    # top-level lists
    p = pb.parse_place(_rec(added=[{"name": "box", "x": 1, "y": 0}]))
    assert p["changes"]["added"] == ["box"]
    # age_s instead of t
    p = pb.parse_place(_rec(age_s=4.0), now=now)
    assert p["t"] == pytest.approx(996.0)
    assert pb.parse_place({"ok": False, "error": "no such tool 'where_am_i'"}) is None
    assert pb.parse_place(None) is None and pb.parse_place("place: den (0.9)") is None


def test_parse_place_keeps_the_diagnosis_fields():
    """What the next run needs to be replayed without guessing: parts, evidence, signals
    (the answer's, else the non-None parts), each look of a two-look recognition, and the
    change check's per-name presence answers (eye) — at the top, beside a nested verdict,
    or under changes."""
    # the shape of the 2026-09-25 run's where_am_i answers: parts + evidence, no signals
    real = {"ok": True, "verdict": "known", "name": "den", "place_id": "place-02e08d", "confidence": 0.875,
            "parts": {"desc": 0.697, "radio": None, "scan": 1.0}, "evidence": 1.0, "looks": 1,
            "second": {"place_id": "place-97ded5", "name": "workshop", "confidence": 0.525}}
    p = pb.parse_place(real)
    assert p["parts"] == {"scan": 1.0, "desc": 0.697, "radio": None} and p["evidence"] == 1.0
    assert p["signals"] == ["scan", "desc"] and p["per_look"] == [] and p["eye"] == []
    assert p["second"]["parts"] is None
    # a lone scan says so
    assert pb.parse_place(dict(real, parts={"scan": 0.9, "desc": None, "radio": None}))["signals"] == ["scan"]
    assert pb.parse_place(dict(real, parts=None))["signals"] is None
    # the cockpit's two-look answer: per_look with each look's parts, the eye's answers per look
    two = dict(real, looks=2, signals=["scan", "desc"],
               second={"place_id": "place-97ded5", "name": "workshop", "confidence": 0.62,
                       "parts": {"scan": 0.526, "desc": 0.8}},
               per_look=[{"verdict": "ambiguous", "place_id": "place-02e08d", "confidence": 0.74,
                          "parts": {"scan": 1.0, "desc": 0.38, "radio": None}, "signals": ["scan", "desc"]},
                         {"verdict": "known", "place_id": "place-02e08d", "confidence": 0.8,
                          "parts": {"scan": 1.0, "desc": 0.6}}],
               eye=[{"pose": {"x": 0, "y": 0, "yaw_deg": 0}, "seen": {"Box": True, "ball": None},
                     "placed": {"box": {"x": 0.65, "y": 0.15, "confidence": 0.9}}, "asked": 1,
                     "why": {"ball": "out of view"}},
                    {"seen": {"box": {"seen": True, "x": 0.66}, "ball": {"seen": False}}}])
    p = pb.parse_place(two)
    assert p["second"]["parts"] == {"scan": 0.526, "desc": 0.8, "radio": None}
    assert [x["confidence"] for x in p["per_look"]] == [0.74, 0.8]
    assert p["per_look"][1]["signals"] == ["scan", "desc"] and p["per_look"][0]["parts"]["desc"] == 0.38
    assert [e["seen"] for e in p["eye"]] == [{"box": True, "ball": None}, {"box": True, "ball": False}]
    assert p["eye"][0]["why"] == {"ball": "out of view"} and p["eye"][0]["placed"]["box"]["x"] == 0.65
    assert pb.eye_answers(p, "box") == [True, True] and pb.eye_answers(p, "ball") == [None, False]
    assert pb.eye_answers(p, "chair") == [None, None] and pb.eye_answers(pb.parse_place(real), "box") == []
    # a bare presence map (one look), under changes, beside a nested verdict
    p = pb.parse_place({"place": real, "changes": {"added": [], "missing": [{"name": "ball"}], "moved": [],
                                                   "checked": {"ball": False, "cubes": True, "junk": 3}}})
    assert p["changes"]["missing"] == ["ball"] and p["eye"] == [{"seen": {"ball": False, "box": True}}]
    assert pb.eye_looks({"presence": [{"ball": None}, "nonsense"]}) == [{"seen": {"ball": None}}]
    assert pb.eye_looks({"eye": {}}) == [] and pb.eye_looks(None) == []


def test_changes_score_says_what_the_eye_answered():
    exp = CASE["c - ball"]["changes"]
    parsed = dict(P("known", IDS["c"], looks=2),
                  eye=[{"seen": {"ball": False}}, {"seen": {"red ball": None}}])
    s = pb.score_changes(exp, parsed, _visible)
    assert s["misses"] == ["missing ball"] and s["eye"] == {"missing ball": [False, None]}
    assert pb.score_changes(exp, P("known", IDS["c"]), _visible)["eye"] == {"missing ball": []}
    assert pb.score_changes({"added": [], "missing": []}, parsed, _visible)["eye"] == {}


def test_confidence_rows_and_table():
    rows = pb.confidence_rows(_runs())
    assert len(rows) == 2 * 3 + 7 + 4 + 4                   # enrolments, then every READ (4 relooks)
    first = [r for r in rows if r["case"] == "enrol a"][0]
    assert first["expect"] == "new" and first["phase"] == "arrival"
    kn = [r for r in rows if r["case"] == "b named as a" and r["trial"] == 2 and r["phase"] == "arrival"][0]
    assert kn["observed"] == "known-wrong" and kn["verdict"] == "known" and kn["confidence"] == 0.8
    rl = [r for r in rows if r["phase"] == "relook"]
    assert len(rl) == 4 and all(r["observed"] == r["verdict"] for r in rl)
    parsed = pb.parse_place({"verdict": "known", "name": "den", "place_id": IDS["a"], "confidence": 0.86,
                             "parts": {"scan": 1.0, "desc": 0.66}, "evidence": 1.0,
                             "second": {"place_id": IDS["b"], "name": "workshop", "confidence": 0.59},
                             "per_look": [{"confidence": 0.74}, {"confidence": 0.86}], "looks": 2})
    row = pb.confidence_rows([{"trial": 3, "tests": [_row("a same pose", parsed, trial=3)]}])[0]
    assert (row["scan"], row["desc"], row["lead"], row["per_look"]) == (1.0, 0.66, 0.27, [0.74, 0.86])
    assert row["signals"] == ["scan", "desc"] and row["second"] == "workshop"
    t = pb.render_confidence_table([row]).splitlines()
    assert t[0].split()[:5] == ["case", "tr", "phase", "observed", "conf"]
    assert "a same pose" in t[2] and "scan+desc" in t[2] and "workshop 0.59 / looks 0.74, 0.86" in t[2]
    assert pb.render_confidence_table([]).count("\n") == 1


def test_changes_from_the_verdict_text_tail():
    rec = {"verdict": "known", "place_id": "place-aaaaaa", "name": "den", "confidence": 0.88}
    diff = {"added": [{"name": "box"}, {"name": "chair"}], "missing": [{"name": "ball"}], "moved": []}
    text = pm.verdict_text(rec, diff)
    assert pb.changes_from_text(text) == {"added": ["box", "chair"], "missing": ["ball"], "moved": []}
    p = pb.parse_place({"verdict": "known", "name": "den", "confidence": 0.88, "text": text})
    assert p["changes"]["missing"] == ["ball"]
    assert pb.changes_from_text("place: den (0.91)") is None
    assert pb.changes_from_text("place: den (0.88) — new here: box +2; missing: 3") == \
        {"added": ["box"], "missing": [], "moved": []}


# ------------------------------------------------------------------ scoring
IDS = {"a": "place-aaaaaa", "b": "place-bbbbbb", "c": "place-cccccc"}
NAMES = dict(pb.PLACE_NAMES)


def P(verdict, pid=None, name=None, conf=0.8, changes=None, pending=(), looks=1):
    return {"verdict": verdict, "place_id": pid, "name": name, "confidence": conf, "second": {},
            "changes": changes, "pending": list(pending), "looks": looks, "by": "robot", "status": "done",
            "text": None, "t": None}


def test_parse_place_reads_the_cockpits_answer():
    # the shape sim/cockpit.py _place_answer returns (where_am_i, POST /api/place recognize)
    ans = {"ok": True, "recognize": True, "verdict": "known", "name": "playroom", "place_id": "place-cccccc",
           "confidence": 0.91, "text": "place: playroom (0.91) — missing: ball", "status": "done", "looks": 2,
           "by": "robot", "second": {"place_id": "place-aaaaaa", "name": "den", "confidence": 0.2},
           "age_s": 1.5, "changes": {"added": [], "missing": [{"name": "ball", "x": 0.7, "y": 0.1}], "moved": []},
           "pending": [{"name": "box", "x": 0.9, "y": -0.3, "change": "missing"}]}
    p = pb.parse_place(ans, now=100.0)
    assert (p["verdict"], p["name"], p["looks"], p["by"]) == ("known", "playroom", 2, "robot")
    assert p["changes"] == {"added": [], "missing": ["ball"], "moved": []} and p["pending"] == ["box"]
    assert p["t"] == pytest.approx(98.5)
    # the state feed's brief while it still looks: no verdict to score
    brief = pb.parse_place({"verdict": "recognising", "status": "running", "looks": 0})
    assert brief["verdict"] == "recognising"
    assert pb.observe("known", "a", brief, IDS, NAMES) == ("error", None)


def test_observe_and_correct():
    assert pb.observe("known", "a", P("known", IDS["a"]), IDS, NAMES) == ("known", "a")
    assert pb.observe("known", "a", P("known", None, "Den"), IDS, NAMES) == ("known", "a")      # by name
    assert pb.observe("known", "a", P("known", IDS["b"]), IDS, NAMES) == ("known-wrong", "b")
    assert pb.observe("known", "a", P("known", "place-zzzzzz"), IDS, NAMES) == ("known-wrong", None)
    assert pb.observe("new", "d", P("known", IDS["c"]), IDS, NAMES) == ("known-wrong", "c")
    assert pb.observe("new", "d", P("new", "place-dddddd", "new place #1", conf=0.2), IDS, NAMES) == ("new", None)
    assert pb.observe("known", "a", P("ambiguous", None, "den"), IDS, NAMES) == ("ambiguous", "a")
    assert pb.observe("known", "a", None, IDS, NAMES) == ("error", None)
    assert pb.observe("known", "a", P("maybe"), IDS, NAMES) == ("error", None)
    assert pb.is_correct("known", "known") and pb.is_correct("new", "new")
    assert not pb.is_correct("known", "ambiguous") and not pb.is_correct("new", "known-wrong")
    # a relook in a never-seen room: new, or the unnamed place its first recognition stored
    assert pb.relook_ok("new", "d", P("new"), IDS, NAMES)
    assert pb.relook_ok("new", "d", P("known", "place-dddddd", "new place #1"), IDS, NAMES)
    assert not pb.relook_ok("new", "d", P("known", IDS["a"]), IDS, NAMES)
    assert pb.relook_ok("known", "b", P("known", IDS["b"]), IDS, NAMES)


def _visible(obj):
    return True


def test_change_scoring_hits_misses_false_alarms_and_the_two_look_rule():
    exp = CASE["b + chair"]["changes"]
    two = P("known", IDS["b"], changes={"added": ["cube"], "missing": ["wall"], "moved": []}, pending=["ball"],
            looks=2)
    s = pb.score_changes(exp, two, _visible)
    assert s["hits"] == ["added box"] and s["misses"] == [] and s["false_alarms"] == ["missing wall"]
    assert s["single_glance"] == [] and s["pending"] == ["ball"] and s["looks"] == 2
    # declared after ONE look: the two-look rule broken
    one = P("known", IDS["b"], changes={"added": ["box"], "missing": [], "moved": []}, looks=1)
    assert pb.score_changes(exp, one, _visible)["single_glance"] == ["added box"]
    assert pb.score_changes(exp, dict(one, looks=None), _visible)["single_glance"] == []   # cannot tell
    # not reported = a miss; a spot the eye could not see = not counted either way
    s = pb.score_changes(CASE["c - ball"]["changes"], P("known", IDS["c"], looks=2), _visible)
    assert s["misses"] == ["missing ball"] and s["expected"] == ["missing ball"]
    s = pb.score_changes(CASE["c - ball"]["changes"], P("known", IDS["c"]), lambda o: False)
    assert s["unobservable"] == ["missing ball"] and s["expected"] == [] and s["misses"] == []
    s = pb.score_changes(CASE["c - ball"]["changes"], None, lambda o: 1 / 0)     # a broken check claims nothing
    assert s["unobservable"] == ["missing ball"]
    # the same kind of thing reported as MOVED is not the expected "added"
    s = pb.score_changes(exp, P("known", IDS["b"], changes={"added": [], "missing": [], "moved": ["box"]}, looks=2),
                         _visible)
    assert s["misses"] == ["added box"] and s["false_alarms"] == ["moved box"]


def test_eye_sees_matches_the_cockpits_view():
    home = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    assert pb.eye_sees(home, {"kind": "ball", "pos": [1.0, 0.0], "radius_m": 0.05})
    assert not pb.eye_sees(home, {"kind": "ball", "pos": [2.5, 0.0], "radius_m": 0.05})       # beyond 2 m
    assert not pb.eye_sees(home, {"kind": "ball", "pos": [0.2, 0.0], "radius_m": 0.05})       # under the frame
    assert not pb.eye_sees(home, {"kind": "ball", "pos": [0.5, 0.8], "radius_m": 0.05})       # 60 deg left
    assert pb.eye_sees({"x": 0.0, "y": 0.0, "yaw_deg": 60.0}, {"kind": "ball", "pos": [0.5, 0.8], "radius_m": 0.05})


def test_score_visit_flags_a_name_leak_only_on_the_borrowed_room():
    case = CASE["b named as a"]
    leak = pb.score_visit(case, P("known", IDS["a"]), IDS, NAMES, _visible)
    assert leak["observed"] == "known-wrong" and leak["label"] == "a" and leak["name_leak"]
    other = pb.score_visit(case, P("known", IDS["c"]), IDS, NAMES, _visible)
    assert not other["name_leak"] and not other["correct"]
    ok = pb.score_visit(case, P("known", IDS["b"], conf=0.9), IDS, NAMES, _visible, relook=P("known", IDS["b"]))
    assert ok["correct"] and ok["confidence"] == 0.9 and not ok["name_leak"]
    assert ok["relook"] == {"observed": "known", "label": "b", "confidence": 0.8, "agrees": True}
    plain = pb.score_visit(CASE["a same pose"], P("known", IDS["b"]), IDS, NAMES, _visible)
    assert not plain["name_leak"] and plain["relook"] is None            # no borrowed name: a plain mistake


def _row(cid, parsed, trial=1, wall=10.0, read_s=1.0, relook=None):
    case = CASE[cid]
    reads = [{"phase": "arrival", "wall_s": read_s, "agree": True, "parsed": parsed}]
    if relook is not None:
        reads.append({"phase": "relook", "wall_s": read_s + 0.5, "agree": True, "parsed": relook})
    return {"trial": trial, "case": cid, "room": case["room"], "expect": case["expect"],
            "world_name_of": case.get("world_name_of"),
            "changes_expected": {k: [pb.kind_name(o) for o in v] for k, v in case["changes"].items()},
            "reads": reads, "score": pb.score_visit(case, parsed, IDS, NAMES, _visible, relook=relook),
            "wall_s": wall}


def _runs():
    kn = lambda lab, conf=0.9, **kw: P("known", IDS[lab], NAMES[lab], conf, **kw)   # noqa: E731
    t1 = [_row("a same pose", kn("a", 0.95)),
          _row("b named as a", kn("b", 0.88)),
          _row("a named as b", kn("a", 0.93)),
          _row("a moved + turned", P("ambiguous", None, "den", 0.7, looks=2)),
          _row("d never seen", P("new", "place-dddddd", "new place #1", 0.2)),
          _row("b + chair", kn("b", 0.85, changes={"added": ["box"], "missing": [], "moved": []}, looks=2)),
          _row("c - ball", kn("c", 0.8, pending=["ball"], looks=2))]
    t2 = [_row("a same pose", kn("a", 0.97), trial=2, relook=kn("a")),
          _row("b named as a", kn("a", 0.8), trial=2, relook=kn("a")),                  # a name leak
          _row("d never seen", kn("b", 0.8), trial=2, relook=kn("b")),                  # a false recognition
          _row("c - ball", kn("c", 0.82, changes={"added": ["wall"], "missing": ["ball"], "moved": []}, looks=2),
               trial=2, relook=kn("c"))]
    enrol = [{"room": lab, "reads": [{"phase": "arrival", "wall_s": 1.0,
                                      "parsed": P("new" if lab != "b" else "ambiguous")}],
              "name_place": {"ok": True}, "wall_s": 12.0} for lab in "abc"]
    return [{"trial": 1, "enrol": enrol, "tests": t1, "wall_s": 100.0},
            {"trial": 2, "enrol": enrol, "tests": t2, "wall_s": 80.0}]


def test_summary_confusion_matrix_and_changes():
    s = pb.summarize(_runs())
    assert s["visits"] == 11 and s["correct"] == "8/11"
    m = s["confusion"]
    assert m["known"] == {"known": 7, "known-wrong": 1, "new": 0, "ambiguous": 1, "unknown": 0, "error": 0}
    assert m["new"] == {"known": 0, "known-wrong": 1, "new": 1, "ambiguous": 0, "unknown": 0, "error": 0}
    assert sum(sum(r.values()) for r in m.values()) == s["visits"]
    c = s["cases"]
    assert c["a same pose"]["correct"] == "2/2" and c["a same pose"]["confidence"]["median"] == 0.96
    assert c["b named as a"]["correct"] == "1/2" and c["b named as a"]["name_leak"] == 1
    assert c["d never seen"]["verdicts"] == {"new": 1, "known-wrong b": 1}
    assert c["a moved + turned"]["verdicts"] == {"ambiguous (a?)": 1} and c["a moved + turned"]["second_looks"] == "1/1"
    assert c["c - ball"]["second_looks"] == "2/2" and c["a same pose"]["second_looks"] == "0/2"
    assert s["name_swap"] == {"correct": "2/3", "name_leak": 1}
    ch = s["changes"]
    assert ch["hits"] == 2 and ch["expected"] == 3 and ch["misses"] == [("missing ball", 1)]
    assert ch["false_alarms"] == 1 and ch["false_alarm_kinds"] == {"added wall": 1}
    assert ch["single_glance_visits"] == 0
    assert ch["false_alarm_visits_unchanged"] == "0/8"      # the one false alarm was on a changed visit
    assert s["second_looks"] == "4/11"
    assert s["enrol"] == {"first_new": "4/6", "first_not_known": "6/6", "named": "6/6"}
    assert s["timings"]["arrival_med_s"] == 1.0 and s["timings"]["trial_med_s"] == 90.0
    assert s["relook_agrees"] == "2/4"                      # trial 2: b named as a (says a), d (says b)
    assert c["a same pose"]["relook_agrees"] == "1/1" and c["a named as b"]["relook_agrees"] is None


def _result(runs=None, **kw):
    r = {"date": "2026-09-25 18:30", "cockpit": "http://127.0.0.1:8795", "fenced": True, "dry_run": False,
         "vision_model": "lfm2.5-vl", "embed": True, "speed": 2.0, "trials_done": 2, "recognize_via": "api",
         "fence": {"counts": {"embeddings": 20, "chat": 20}}, "runs": _runs() if runs is None else runs}
    r.update(kw)
    r["summary"] = pb.summarize(r["runs"])
    return r


def _cells(line):
    return [c.strip() for c in line.strip().strip("|").split("|")]


def test_markdown_tables():
    md = pb.render_markdown(_result()).splitlines()
    assert md[0].startswith("**Place bench** — 2026-09-25 18:30") and "`lfm2.5-vl`" in md[0]
    assert "embeddings on (20 through the fence)" in md[0] and "(own, fenced)" in md[0] and "DRY RUN" not in md[0]
    i = md.index(pb.MD_CASE_HEADER)
    assert _cells(md[i]) == ["case", "expected", "verdicts", "correct", "confidence median (min–max)",
                             "second look taken", "changes (two looks agree)", "arrival → verdict median (s)"]
    assert md[i + 1] == "|" + "---|" * 8
    rows = [_cells(x) for x in md[i + 2:i + 2 + len(pb.CASES)]]
    assert [r[0] for r in rows] == [c["id"] for c in pb.CASES]
    assert all(len(r) == 8 for r in rows)
    by = {r[0]: r for r in rows}
    assert by["b named as a"][1] == "known b (world name of a)"
    assert by["b named as a"][2] == "known b ×1, known-wrong a ×1"
    assert by["a same pose"][3] == "2/2" and by["a same pose"][4] == "0.96 (0.95–0.97)"
    assert by["b + chair"][5] == "1/1" and by["b + chair"][6] == "1/1 found"
    assert by["c - ball"][6].startswith("1/2 found; 1 false alarm")
    assert by["d never seen"][1] == "new" and by["a same pose"][7] == "1.0"
    j = md.index(pb.MD_MATRIX_HEADER)
    assert _cells(md[j + 2]) == ["known", "7", "1", "0", "1", "0", "0"]
    assert _cells(md[j + 3]) == ["new", "0", "1", "1", "0", "0", "0"]
    tail = "\n".join(md[j + 4:])
    assert "NAME LEAK suspected in 1" in tail and "Verdicts correct 8/11" in tail
    assert "second look in 4/11 visits" in tail and "Relook from 30 deg further left agrees 2/4" in tail
    dry = pb.render_markdown(_result(dry_run=True, interrupted="SIGTERM"))
    assert dry.startswith("**DRY RUN — plumbing only, not a measurement**") and "INTERRUPTED (SIGTERM)" in dry
    empty = pb.render_markdown(_result(runs=[]))
    assert pb.MD_MATRIX_HEADER in empty                     # renders with nothing measured
    assert "known-wrong" in pb.render_table(_result())


# ------------------------------------------------------------------ the fence
def test_fence_rules():
    allowed = {"lfm2.5-vl", "embedding"}
    assert pb.place_fence_allows("/v1/chat/completions", "lfm2.5-vl", allowed)
    assert not pb.place_fence_allows("/v1/chat/completions", "gemma-4-26b-a4b", allowed)   # no fallback loads
    assert not pb.place_fence_allows("/v1/chat/completions", "embedding", allowed)
    assert pb.place_fence_allows("/v1/embeddings", "embedding", allowed)
    assert not pb.place_fence_allows("/v1/embeddings", "embedding", allowed, embed=False)
    assert pb.place_fence_allows("/upstream/lfm2.5-vl/props", None, allowed)
    assert not pb.place_fence_allows("/upstream/qwen3.6-35b-a3b/props", None, allowed)
    assert not pb.place_fence_allows("/v1/completions", "lfm2.5-vl", allowed)
    v = pb.hashed_embedding("a grey wall ahead")
    assert len(v) == pb.EMBED_DIM and abs(sum(x * x for x in v) - 1.0) < 1e-9
    assert v == pb.hashed_embedding("A grey WALL ahead!") and v != pb.hashed_embedding("a ball")
    code, body = pb.dry_answer("/v1/chat/completions", {"model": "lfm2.5-vl"})
    assert code == 200 and body["choices"][0]["message"]["content"] == pb.DRY_LOOK
    code, body = pb.dry_answer("/v1/embeddings", {"model": "embedding", "input": ["a", "b"]})
    assert [d["index"] for d in body["data"]] == [0, 1]
    assert pb.dry_answer("/v1/models", {}) is None


def test_dry_run_fence_answers_on_loopback_and_never_forwards(monkeypatch):
    def boom(*a, **k):
        raise AssertionError("the dry-run fence reached llama-swap")
    monkeypatch.setattr(httpx, "request", boom)
    for embed in (True, False):
        f = pb.PlaceFence({"lfm2.5-vl", "embedding"}, embed=embed, dry_run=True)
        try:
            base = f.base_url
            ids = {m["id"] for m in httpx.get(base + "/models", timeout=5).json()["data"]}
            assert ids == {"lfm2.5-vl", "embedding"}
            r = httpx.post(base + "/chat/completions", json={"model": "lfm2.5-vl", "messages": []}, timeout=5)
            assert r.status_code == 200 and r.json()["choices"][0]["message"]["content"] == pb.DRY_LOOK
            r = httpx.post(base + "/chat/completions", json={"model": "gemma-4-26b-a4b"}, timeout=5)
            assert r.status_code == 503 and "only" in r.json()["error"]["message"]
            r = httpx.post(base + "/embeddings", json={"model": "embedding", "input": "a wall"}, timeout=5)
            if embed:
                assert r.status_code == 200 and len(r.json()["data"][0]["embedding"]) == pb.EMBED_DIM
            else:
                assert r.status_code == 503 and "--no-embed" in r.json()["error"]["message"]
            assert f.counts["chat"] == 1 and f.counts["chat_refused"] == 1
            assert f.counts["embeddings" if embed else "embeddings_refused"] == 1
        finally:
            f.close()


def test_own_cockpit_env_is_scratch_only(tmp_path, monkeypatch):
    src = tmp_path / "cockpit.py"
    src.write_text('d = os.environ.get("ROCKY_PLACE_DIR") or os.environ.get("ROCKY_MEMORY_DIR")\n'
                   'x = os.environ.get("ROCKY_PLACES_FILE")  # ROCKY_PLACE_DIR again\n')
    assert pb.place_env_names(str(src)) == ["ROCKY_PLACES_FILE", "ROCKY_PLACE_DIR"]
    assert pb.place_env_names(str(tmp_path / "missing.py")) == []

    class F:
        base_url = "http://127.0.0.1:1/v1"
    monkeypatch.setattr(pb, "place_env_names", lambda: ["ROCKY_PLACE_DIR"])
    own = pb.OwnCockpit(8799, F(), str(tmp_path), print)
    env = own.env()
    assert env["ROCKY_LLM_BASE_URL"] == F.base_url
    assert env["ROCKY_MEMORY_DIR"].startswith(str(tmp_path)) and env["ROCKY_PLACE_DIR"] == env["ROCKY_MEMORY_DIR"]
    assert env["ROCKY_COCKPIT_CONF"].startswith(str(tmp_path))
    assert not any(os.path.expanduser("~/.config") in v for v in env.values())
    own.starts = 1
    assert own.env()["ROCKY_MEMORY_DIR"] != env["ROCKY_MEMORY_DIR"]     # a fresh memory per start


# ------------------------------------------------------------------ the owner's cockpit
def test_main_refuses_the_owner_cockpit_before_any_network(monkeypatch):
    def boom(*a, **k):
        raise AssertionError("network touched")
    for name in ("listed_models", "running_models", "unload", "PlaceFence", "OwnCockpit", "BenchCockpit"):
        monkeypatch.setattr(pb, name, boom)
    for argv in (["--port", "8765"], ["--url", "http://127.0.0.1:8765"], ["--url", "http://localhost:8765/"]):
        with pytest.raises(SystemExit) as e:
            pb.main(argv)
        assert "8765" in str(e.value)
    for url in ("https://ai-hub.tail54f481.ts.net:9445", "http://100.101.102.103:8795", "http://192.168.1.20:8795"):
        with pytest.raises(SystemExit) as e:
            pb.main(["--url", url])
        assert "refusing" in str(e.value)
    for argv, why in ((["--trials", "0"], "at least 1"), (["--cases", "nope"], "unknown case"),
                      (["--vision-model", "gpt-oss-20b"], "quarantined")):
        with pytest.raises(SystemExit) as e:
            pb.main(argv)
        assert why in str(e.value)


def test_help_smoke():
    # MUJOCO_GL=disable: --help needs no renderer (see test_brain_bench.test_help_smoke)
    r = subprocess.run([sys.executable, os.path.join(ROOT, "sim", "place_bench.py"), "--help"],
                       capture_output=True, text=True, timeout=120, cwd=ROOT,
                       env=dict(os.environ, MUJOCO_GL="disable"))
    assert r.returncode == 0, r.stderr
    for flag in ("--url", "--port", "--vision-model", "--embed", "--no-embed", "--trials", "--cases",
                 "--relook", "--speed", "--dry-run", "--keep-loaded", "--out"):
        assert flag in r.stdout


# ------------------------------------------------------------------ the name-swap proof
_MODELS = {}
_STAND = {}


def _sweep(spec, pose):
    """A real MuJoCo lidar sweep of the staged world with the robot standing at pose."""
    import mujoco
    import rocky_model as rm
    import world_builder as wb
    from pebble_gait import WaveGait, leg_ik, body_to_leg
    from sim_lidar import scan
    key = json.dumps({k: v for k, v in spec.items() if k != "visit"}, sort_keys=True)
    if key not in _MODELS:
        _MODELS[key] = wb.build({k: v for k, v in spec.items() if k != "visit"})
    m, z0 = _MODELS[key]
    if "q0" not in _STAND:
        g = WaveGait()
        _STAND.update(q0=np.array([leg_ik(body_to_leg(i, g.p_nom[i])) for i in range(5)]).flatten(),
                      z=rm.spawn_z_m(g.h))
    d = mujoco.MjData(m)
    jadr = [m.joint(f"{n}{i}").qposadr[0] for i in range(5) for n in ("yaw", "hip", "knee")]
    yaw = math.radians(pose["yaw_deg"])
    d.qpos[0:3] = [pose["x"], pose["y"], _STAND["z"] + z0]
    d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    d.qpos[jadr] = _STAND["q0"]
    mujoco.mj_forward(m, d)
    angles, ranges, _ = scan(m, d, m.body("torso").id)
    return pm.scan_signature(angles, ranges, yaw)


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


_BASE_WORDS = {"room": "A small room with pale walls all around and two grey pillars.",
               "flat": "An open grey floor stretching far away, nothing standing on it."}


def _in_view(pose, x, y):
    return pb.eye_sees(pose, {"kind": "ball", "pos": [x, y], "radius_m": 0.0})


def _describe(spec, pose):
    """The fake eye's words for a frame: the base, then every object in the eye's view
    (pb.eye_sees), its side and distance — the way test_awareness's fake eye writes them."""
    parts = [_BASE_WORDS.get(spec.get("base", "flat"), "A floor.")]
    for o in spec.get("objects") or []:
        if not _in_view(pose, *o["pos"]):
            continue
        dx, dy = o["pos"][0] - pose["x"], o["pos"][1] - pose["y"]
        rel = _wrap(math.degrees(math.atan2(dy, dx)) - pose["yaw_deg"])
        side = "on the left" if rel > 15 else "on the right" if rel < -15 else "straight ahead"
        parts.append(f"A {o['kind']} {side}, {math.hypot(dx, dy):.1f} m away.")
    return " ".join(parts)


def _cockpit():
    """sim/cockpit.py, for its PURE place helpers (combine_recognitions, apply_confirmed,
    PLACE_FIXTURES): the fake combines two looks and commits a visit by the cockpit's own
    rules, not by a copy of them. Imported lazily: only the MuJoCo tests drive the fake."""
    import cockpit
    return cockpit


def _is_fixture(name):
    n = pm.norm_name(name)
    return bool(n) and n.split()[-1] in _cockpit().PLACE_FIXTURES


def _merge_objects(looks):
    """A new place's first objects: the perfect eye's placed sightings of both looks, each
    once, the room's fixtures (walls) left out — what cockpit.eye_objects stores when no look
    contradicts another (a perfect eye never does)."""
    out = []
    for lk in looks:
        for o in lk["objects"]:
            if _is_fixture(o["name"]):
                continue
            if not any(p["name"] == o["name"] and abs(p["x"] - o["x"]) + abs(p["y"] - o["y"]) < 1e-6
                       for p in out):
                out.append(dict(o))
    return out


class FakeCockpit:
    """The cockpit's HTTP surface as the bench uses it, following sim/cockpit.py's D057
    protocol: recognition after every spawn while it is on (POST /api/place recognize
    runs it); an ambiguous or new first look takes a second look after a 30 deg turn,
    and an ambiguous one is settled by cockpit.combine_recognitions (the MEAN of the two
    looks, the verdict re-derived); a known place is checked for changes by the eye (a
    per-name presence answer per look -> checked_objects with the look's own words as
    `mentioned`), a first look that shows a change takes the second look, and a change
    is declared only when both looks agree (confirm_diff); the visit is committed as the
    cockpit commits it: samples + a visit, the object snapshot rewritten only by a
    CONFIRMED change (cockpit.apply_confirmed) — an unchanged look rewrites nothing —
    and a known visit's second look added as a sample (add_samples). 'new' stores a
    place ('new place #k') from BOTH looks (PlaceMemory.enroll_samples); name_place
    renames the bound place, or (new=true, or none bound) stores one from this stay's
    looks. by="senses": a place is recognised from a real MuJoCo lidar sweep AND the
    fake eye's description embedded (both senses: a lone sweep is capped below "known").
    by="name": a place IS its world's name — the cheat the bench must catch.
    WHAT IT DOES NOT PROVE: the fake eye is perfect and deterministic — the same view
    gives the same words (cos 1.0, where the real run measured the look embedding
    separating nothing), it boxes AND places every object in its view (the cockpit's
    "boxed, not placed" path never occurs here), and it never contradicts its own words.
    So the green name-swap and change tests prove the bench's staging and scoring and
    place_memory on real sweeps, not that sim/cockpit.py holds together (that is
    test_awareness's, through the real cockpit with a fake eye). The answer carries
    parts, evidence, signals, per_look (each look's own recognition, with its parts —
    more than the cockpit reports today) and eye_checks (the per-name presence answers
    of each look of the change check, the cockpit's key)."""

    url = "http://127.0.0.1:8796"

    def __init__(self, by="senses"):
        self.by = by
        self.mem = pm.PlaceMemory(directory=None)
        self.name = self.spec = None
        self.pose = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
        self.on = False
        self._blank()
        self.calls = []
        self.aw = {"reactions": True, "curious": True}
        self.roles = {"brain": "tool-model", "vision": "vision-model"}
        self.speed = 1.0

    def alive(self):
        return True

    def _blank(self):
        self.seq, self.result_seq, self.answer, self.bound, self.samples = 0, -1, None, None, []

    def _vis(self, pose):
        return lambda x, y: pb.eye_sees(pose, {"kind": "ball", "pos": [x, y], "radius_m": 0.0})

    def _look(self):
        pose = dict(self.pose)
        vis = self._vis(pose)
        objs = [{"name": o["kind"], "x": o["pos"][0], "y": o["pos"][1], "confidence": 0.9}
                for o in self.spec.get("objects") or [] if vis(*o["pos"])]
        text = _describe(self.spec, pose)
        return {"sig": _sweep(self.spec, pose), "pose": pose, "objects": objs, "vis": vis, "text": text,
                "emb": pb.hashed_embedding(text)}

    def _rec(self, look):
        if self.by == "name":
            pid = self.by_name().get(self.name)
            if pid:
                return {"verdict": "known", "place_id": pid, "name": self.mem.get(pid)["name"], "confidence": 0.99,
                        "second": {}}
            return {"verdict": "new", "place_id": None, "name": None, "confidence": 0.0, "second": {}}
        return self.mem.recognize(scan_sig=look["sig"], desc_emb=look["emb"])

    @staticmethod
    def _eye(look, stored):
        """The perfect eye's per-name presence answers for one look: every stored name and
        every kind in view (what its words mention), the room's fixtures never asked (as the
        cockpit) -> True (an object of that kind is in view) / False."""
        names = {pm.norm_name(o["name"]) for o in stored} | {pm.norm_name(o["name"]) for o in look["objects"]}
        return {n: any(pm.same_thing(n, o["name"]) for o in look["objects"])
                for n in sorted(names) if not _is_fixture(n)}

    def _check(self, look, stored):
        """One look's change check by the eye -> (checked_objects, eye_brief-like record)."""
        seen = self._eye(look, stored)
        d = pm.checked_objects(stored, seen, visible=look["vis"], mentioned=look["text"])
        return d, {"pose": dict(look["pose"]), "seen": seen}

    def by_name(self):
        return getattr(self, "_by_name", {})

    def _turn(self, deg):
        self.pose["yaw_deg"] = _wrap(self.pose["yaw_deg"] + float(deg))

    def _second_look(self, looks):
        self._turn(pb.SECOND_LOOK_DEG)
        looks.append(self._look())
        return looks[-1]

    def _recognize(self):
        ck = _cockpit()
        l1 = self._look()
        rec, looks = self._rec(l1), [l1]
        per_look = [rec]
        if rec["verdict"] in ("ambiguous", "new") and self.by == "senses":
            r2 = self._rec(self._second_look(looks))     # the cockpit's second look, 30 deg further
            per_look.append(r2)
            if rec["verdict"] == "ambiguous":
                rec = ck.combine_recognitions(rec, r2)  # the mean of the two looks, re-derived
        confirmed, eye = None, []
        if rec["verdict"] == "known":
            pid = rec["place_id"]
            stored = [o for o in (self.mem.get(pid) or {}).get("objects") or [] if not _is_fixture(o["name"])]
            checks = [self._check(lk, stored) for lk in looks]
            if len(looks) == 1 and any(checks[0][0][k] for k in ("added", "missing")):
                checks.append(self._check(self._second_look(looks), stored))
            eye = [c[1] for c in checks]
            if len(checks) >= 2:
                confirmed = pm.confirm_diff(checks[0][0], checks[1][0], ck.PLACE_MATCH_M)
            else:                                        # one look: nothing is declared
                confirmed = {"added": [], "missing": [], "moved": [],
                             "pending": [dict(o, change=k) for k in ("added", "missing")
                                         for o in checks[0][0][k]]}
            objs = None                                  # no confirmed change: the snapshot stays
            if any(confirmed[k] for k in ("added", "missing", "moved")):
                objs = ck.apply_confirmed(stored, confirmed)
            self.mem.visit(pid, scan_sig=l1["sig"], desc_emb=l1["emb"], desc_text=l1["text"], objects=objs)
            if len(looks) > 1:                           # the second look is another view of this place
                self.mem.add_samples(pid, looks[1:])
            self.bound = pid
        elif rec["verdict"] == "new":
            k = 1 + sum(1 for p in self.mem.places() if str(p["name"]).startswith("new place #"))
            self.bound = self.mem.enroll_samples(looks, objects=_merge_objects(looks), name=f"new place #{k}")
            self._by_name = dict(self.by_name(), **{self.name: self.bound})
        else:
            self.bound = None
        self.samples = looks
        self.result_seq = self.seq
        q = self.mem.get(self.bound) if self.bound else None
        self.answer = {"ok": True, "recognize": True, "verdict": rec["verdict"],
                       "name": (q or {}).get("name") if q else rec.get("name"), "place_id": self.bound,
                       "confidence": rec.get("confidence"), "looks": len(looks), "by": "robot",
                       "status": "done", "second": rec.get("second") or {}, "age_s": 0.0,
                       "text": pm.verdict_text(rec, confirmed), "parts": rec.get("parts"),
                       "evidence": rec.get("evidence"), "signals": rec.get("signals")}
        if len(per_look) > 1:
            keys = ("verdict", "place_id", "name", "confidence", "parts", "signals", "evidence")
            self.answer["per_look"] = [{k: r.get(k) for k in keys} for r in per_look]
        if eye:
            self.answer["eye_checks"] = eye
        if confirmed is not None:
            self.answer["changes"] = {k: confirmed[k] for k in ("added", "missing", "moved")}
            self.answer["pending"] = confirmed["pending"]
        return copy.deepcopy(self.answer)

    def _place_answer(self):
        if not self.on:
            return {"ok": False, "recognize": False, "error": "place recognition is off"}
        if self.result_seq != self.seq:
            return {"ok": True, "recognize": True, "verdict": "recognising", "status": "due", "looks": 0}
        return copy.deepcopy(self.answer)

    def _name_place(self, name, new):
        if new or self.bound is None:
            pid = self.mem.enroll_samples(self.samples, objects=_merge_objects(self.samples), name=name)
            action = "stored"
            self.bound = pid
            self._by_name = dict(self.by_name(), **{self.name: pid})
        else:
            pid, action = self.bound, "renamed"
            self.mem.name_place(pid, name)
        self.answer = dict(self.answer or {}, verdict="known", name=name, place_id=pid, confidence=1.0,
                           by="operator")
        return {"ok": True, "place_id": pid, "name": name, "action": action}

    def post(self, path, body=None, timeout=None):
        body = body or {}
        self.calls.append(("POST", path, json.loads(json.dumps(body))))
        if path == "/api/world":
            self.name, self.spec = body["name"], body["spec"]
            self.pose = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
            self.seq += 1                                # a spawn: recognition due
            return {"ok": True, "name": self.name}
        if path == "/api/awareness":
            if "recognize" in body and bool(body["recognize"]) != self.on:
                self.on = bool(body["recognize"])
                self._blank()
                self.seq = 1 if self.on else 0           # switched on: a recognition due where it stands
            self.aw.update({k: body[k] for k in ("reactions", "curious") if k in body})
            return {"ok": True, "awareness": dict(self.aw, recognize=self.on)}
        if path == "/api/brain":
            if body.get("vision_model"):
                self.roles["vision"] = body["vision_model"]
            self.roles.update(body.get("roles") or {})
            return {"ok": True, "roles": dict(self.roles)}
        if path == "/api/speed":
            self.speed = float(body["speed"])
            return {"speed": self.speed}
        if path == "/api/place" and body.get("action") == "recognize":
            if not self.on:
                return self._place_answer()
            if self.result_seq == self.seq and not body.get("force"):
                return self._place_answer()
            return self._recognize()
        if path == "/api/tool/where_am_i":
            return self._place_answer()
        if path == "/api/tool/name_place":
            return self._name_place(body["name"], bool(body.get("new")))
        if path == "/api/tool/forget_place":
            return {"ok": True, "places": self.mem.forget("all")}
        if path == "/api/tool/status":
            return {"ok": True, "pose": dict(self.pose)}
        if path == "/api/tool/goto":
            self.pose.update(x=float(body["x"]), y=float(body["y"]))
            return {"ok": True, "pose": dict(self.pose)}
        if path == "/api/tool/turn":
            self._turn(body["deg"])
            return {"ok": True, "turned_deg": body["deg"]}
        return {"ok": False, "error": f"fake: no {path}"}

    def get(self, path):
        self.calls.append(("GET", path, None))
        if path == "/api/state":
            a = self._place_answer() if self.on else None
            return {"place": None if a is None else {k: a.get(k) for k in ("verdict", "name", "place_id",
                                                                             "confidence", "status", "looks")},
                    "pose": dict(self.pose), "speed": self.speed}
        if path == "/api/awareness":
            return {"ok": True, "awareness": dict(self.aw, recognize=self.on)}
        if path == "/api/models":
            return {"roles": dict(self.roles), "brain": {"mode": "talk"}}
        if path == "/api/capabilities":
            return {"tools": [{"name": n} for n in ("where_am_i", "name_place", "places", "forget_place")]}
        return {"ok": False}


def _cfg(relook=False):
    return type("Cfg", (), dict(settle_s=0.0, motion_settle_s=0.0, timeout=5.0, first_timeout=5.0,
                                relook=relook, relook_deg=pb.SECOND_LOOK_DEG))


def _trial(by, relook=False):
    ck = FakeCockpit(by)
    assert pb.awareness_on(ck)
    run = pb.run_trial(ck, 1, pb.case_table(), _cfg(relook), lambda s: None)
    return ck, run


@needs_mujoco
def test_bench_rooms_are_distinct_to_the_lidar():
    home = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    sig = {lab: _sweep(PRESETS[p], home) for lab, p in pb.ROOMS.items()}
    for lab in "abcd":
        assert pm.scan_info(sig[lab]) >= 0.5, lab                 # every bench room gives the lidar something
    sim = lambda p, q: pm.scan_similarity(sig[p], sig[q])        # noqa: E731
    assert sim("a", "b") < pm.KNOWN_T                             # same walls, different furniture: not a twin
    for p, q in (("a", "c"), ("b", "c"), ("a", "d"), ("b", "d"), ("c", "d")):
        assert sim(p, q) < 0.1, (p, q)
    for room in "abc":                                            # a chair does not make a room a stranger
        assert pm.scan_similarity(_sweep(PRESETS[f"room {room} + chair"], home), sig[room]) >= pm.KNOWN_T
    assert pm.scan_similarity(_sweep(PRESETS["room c - ball"], home), sig["c"]) > 0.99   # the ball is below the plane


@needs_mujoco
def test_senses_recogniser_passes_the_name_swap_and_the_changes():
    ck, run = _trial("senses")
    rows = {r["case"]: r for r in run["tests"]}
    for cid in ("b named as a", "a named as b"):
        s = rows[cid]["score"]
        assert s["correct"] and not s["name_leak"], (cid, s)
        assert rows[cid]["stage"]["world_name"] == run["world_names"][CASE[cid]["world_name_of"]]
    assert rows["a same pose"]["score"]["correct"] and rows["a same pose"]["score"]["looks"] == 1
    assert rows["d never seen"]["score"]["observed"] == "new"
    b, c = rows["b + chair"]["score"], rows["c - ball"]["score"]
    assert b["correct"] and b["changes"]["hits"] == ["added box"] and not b["changes"]["false_alarms"]
    assert c["correct"] and c["changes"]["hits"] == ["missing ball"] and not c["changes"]["false_alarms"]
    assert b["looks"] == c["looks"] == 2 and not b["changes"]["single_glance"] and not c["changes"]["single_glance"]
    # what the eye answered about each expected change, per look, lands in the score
    assert b["changes"]["eye"] == {"added box": [True, True]}
    assert c["changes"]["eye"] == {"missing ball": [False, False]}
    # every READ records the senses it rested on: both, with their parts and the evidence
    for r in run["tests"] + run["enrol"][1:]:                 # the first enrolment has nothing to compare
        p = r["reads"][0]["parsed"]
        assert p["signals"] == ["scan", "desc"] and p["evidence"] == pytest.approx(1.0), (r.get("case"), p)
        assert None not in (p["parts"]["scan"], p["parts"]["desc"]) and p["parts"]["radio"] is None
    table = pb.render_confidence_table(pb.confidence_rows([run]))
    assert "b + chair" in table and "scan+desc" in table
    # ONE sense: the same room by its lidar sweep alone is capped at SINGLE_EVIDENCE — "ambiguous",
    # never "known" (the reason the fake must feed the description too) ...
    home = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    sweep_b = _sweep(PRESETS["room b"], home)
    lone = ck.mem.recognize(scan_sig=sweep_b)
    assert lone["verdict"] == "ambiguous" and lone["name"] == pb.PLACE_NAMES["b"]
    assert lone["signals"] == ["scan"]
    assert lone["parts"]["scan"] == pytest.approx(1.0, abs=1e-3)
    assert lone["evidence"] == pytest.approx(pm.SINGLE_EVIDENCE)
    capped = 0.5 + (lone["parts"]["scan"] - 0.5) * pm.SINGLE_EVIDENCE
    assert lone["confidence"] == pytest.approx(capped, abs=1e-3)
    assert lone["confidence"] <= 0.5 + 0.5 * pm.SINGLE_EVIDENCE < pm.KNOWN_T
    assert pm.verdict_text(lone).endswith("[scan only]")
    # ... and with the description of the same view it is known
    words = _describe(PRESETS["room b"], home)
    both = ck.mem.recognize(scan_sig=sweep_b, desc_emb=pb.hashed_embedding(words))
    assert both["verdict"] == "known" and both["name"] == pb.PLACE_NAMES["b"]
    assert both["signals"] == ["scan", "desc"]
    # the moved case: recognition off for the load and the walk, back on at the moved pose
    mv = rows["a moved + turned"]
    assert mv["recognize_off_for_move"] and mv["move"]["recognize_back_on"]
    assert mv["reads"][0]["pose"] == {"x": 0.25, "y": -0.15, "yaw_deg": 90.0}
    assert all(r["reads"][0]["arrival_s"] is not None for r in run["tests"] + run["enrol"])
    s = pb.summarize([run])
    assert s["name_swap"] == {"correct": "2/2", "name_leak": 0}
    assert s["changes"]["single_glance_visits"] == 0 and s["enrol"]["named"] == "3/3"
    # what the bench sent: opaque world names (only the swap cases reuse one), fresh spec tokens,
    # and no room label or preset name anywhere a recogniser could read it
    worlds = [b for m, p, b in ck.calls if p == "/api/world"]
    assert len(worlds) == len(pb.ENROL_ROOMS) + len(pb.CASES)
    assert all(pb.WORLD_NAME_RE.match(w["name"]) for w in worlds)
    assert len({w["spec"]["visit"] for w in worlds}) == len(worlds)
    assert len({w["name"] for w in worlds}) == len(worlds) - 2
    blob = json.dumps([b for _m, _p, b in ck.calls])
    for preset in PRESETS:
        if preset.startswith("room "):
            assert f'"{preset}"' not in blob, preset
    named = [b for m, p, b in ck.calls if p == "/api/tool/name_place"]
    assert [b["name"] for b in named] == [pb.PLACE_NAMES[lab] for lab in pb.ENROL_ROOMS]
    # the owner's words follow the robot's verdict: a plain name after "new", new=true otherwise
    firsts = [r["reads"][0]["parsed"]["verdict"] for r in run["enrol"]]
    assert [bool(b.get("new")) for b in named] == [v != "new" for v in firsts]
    # the bench never turned for a READ: every turn is the move's (the cockpit's own looks turn it)
    turns = [b for m, p, b in ck.calls if p == "/api/tool/turn"]
    assert turns == [{"deg": 90.0}]


@needs_mujoco
def test_lidar_empty_worlds_are_never_known_as_another_through_the_fake():
    """Review 2026-09-25: an empty sweep counted as a second sense, so flat, then stairs,
    then the rubble field were ONE place ('known' 0.89, 0.95) whose objects each world
    overwrote. Two empty sweeps are not compared now: the look alone decides, capped below
    'known' — each world is 'ambiguous' [look only] after its second look, nothing is stored
    into the first place, and even the flat itself is only 'ambiguous' (the owner names it).
    Real MuJoCo sweeps: seven of the nine original presets give the puck nothing."""
    home = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    for preset in ("flat", "stairs", "rubble field", "rough terrain", "slope 8 deg", "icy floor", "cliff"):
        assert pm.scan_info(_sweep(PRESETS[preset], home)) == 0.0, preset
    ck = FakeCockpit("senses")
    ck.post("/api/awareness", {"recognize": True})
    ck.post("/api/world", {"name": "w-1", "spec": copy.deepcopy(PRESETS["flat"])})
    a = ck.post("/api/place", {"action": "recognize"})
    assert (a["verdict"], a["looks"]) == ("new", 2)
    before = ck.mem.get(a["place_id"])
    for k, preset in enumerate(("stairs", "rubble field", "flat")):
        ck.post("/api/world", {"name": f"w-{k + 2}", "spec": copy.deepcopy(PRESETS[preset])})
        r = ck.post("/api/place", {"action": "recognize"})
        assert r["verdict"] == "ambiguous" and r["looks"] == 2 and r["place_id"] is None, (preset, r)
        assert r["signals"] == ["desc"] and r["parts"]["scan"] is None and r["text"].endswith("[look only]")
        assert r["confidence"] <= 0.5 + 0.5 * pm.SINGLE_EVIDENCE + 1e-9 and "changes" not in r
        assert [x["signals"] for x in r["per_look"]] == [["desc"], ["desc"]]
        after = ck.mem.get(a["place_id"])                  # nothing written into the first place
        assert (after["visits"], after["objects"], after["desc_texts"]) == \
            (before["visits"], before["objects"], before["desc_texts"]), preset
    assert len(ck.mem.places()) == 1
    # with the look the same words as the stored one, the flat reads the one-sense cap exactly
    assert r["confidence"] == pytest.approx(0.725, abs=1e-3)


@needs_mujoco
def test_a_name_based_recogniser_is_caught_by_the_name_swap():
    _ck, run = _trial("name")
    rows = {r["case"]: r for r in run["tests"]}
    for cid in ("b named as a", "a named as b"):
        s = rows[cid]["score"]
        assert s["name_leak"] and not s["correct"] and s["label"] == CASE[cid]["world_name_of"], (cid, s)
    assert not rows["a same pose"]["score"]["correct"]         # a fresh opaque name reads as new to it
    s = pb.summarize([run])
    assert s["name_swap"] == {"correct": "0/2", "name_leak": 2}
    assert "NAME LEAK suspected in 2" in pb.render_markdown(_result(runs=[run]))


@needs_mujoco
def test_relook_turns_and_forces_a_second_recognition():
    ck, run = _trial("senses", relook=True)
    forced = [b for m, p, b in ck.calls if p == "/api/place" and b.get("force")]
    assert len(forced) == len(pb.ENROL_ROOMS) + len(pb.CASES)
    for r in run["tests"]:
        assert [rd["phase"] for rd in r["reads"]] == ["arrival", "relook"]
        assert r["score"]["relook"] is not None
    assert all(r["relook_known_self"] for r in run["enrol"])
    assert pb.summarize([run])["relook_agrees"] is not None


@needs_mujoco
def test_main_against_a_url_cockpit_writes_the_json_and_puts_the_cockpit_back(tmp_path, monkeypatch, capsys):
    fake = FakeCockpit("senses")
    fake.mem.enroll(scan_sig=_sweep(PRESETS["room d"], {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}), name="old place")
    monkeypatch.setattr(pb, "BenchCockpit", lambda url: fake)
    out = tmp_path / "pb.json"
    res = pb.main(["--url", fake.url, "--dry-run", "--trials", "1", "--settle", "0", "--out", str(out)])
    d = json.loads(out.read_text())
    assert d["trials_done"] == 1 and d["summary"]["visits"] == len(pb.CASES) and d["recognize_via"] == "api"
    assert d["summary"]["name_swap"]["name_leak"] == 0
    assert "old place" not in json.dumps(d["runs"])          # the url cockpit's places were wiped first
    assert d["markdown"].startswith("**DRY RUN") and "Markdown for the place-recognition docs" in capsys.readouterr().out
    # put back: roles, speed, and recognition off again (it was off when the bench came)
    assert fake.roles["vision"] == "vision-model" and fake.speed == 1.0
    assert fake.on is False and fake.aw == {"reactions": True, "curious": True}
    assert "restored" in d["url_restored"] and res["cockpit"] == fake.url
