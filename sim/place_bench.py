"""Place bench — does Pebble know which room it is in, from what it senses alone?

    python sim/place_bench.py                              # own cockpit on :8795, 3 trials
    python sim/place_bench.py --trials 1 --no-embed        # lidar + look, no description embeddings
    python sim/place_bench.py --dry-run --trials 1         # plumbing only: no GPU (canned look, hashed embeddings)
    python sim/place_bench.py --url http://127.0.0.1:8796  # a cockpit you started (loopback, NOT :8765)

The owner's three sentences as a bench: "hey, I'm in the basement at home"
(KNOWN), "hey, this is completely new" (NEW), "hey, this master bedroom has
a new chair" (KNOWN WITH A CHANGE). Every verdict is the cockpit's own
place recognition (D057: sim/cockpit.py runs sim/place_memory.py on a lidar
sweep, a look and the look's embedding) — never the world's name, which a
real robot never has.

THE COCKPIT'S PROTOCOL (what is measured). With recognition on (POST
/api/awareness {recognize: true}, or cockpit.py --recognize), every spawn
(a world load) makes the cockpit, once the robot stands still, take one
sweep + one look and recognise; an ambiguous verdict, or a known place whose
look shows a change, makes it turn 30 deg left (the `turn` tool) and look
again: a change is declared only when both looks agree (confirm_diff). The
bench never looks or turns for it; it only stages, waits and reads.

EACH TRIAL starts from an empty place memory (the bench's own cockpit is
restarted with a fresh scratch memory directory; a --url cockpit gets
forget_place {name: all}, which keeps its places.json.bak), then:

  ENROL  rooms a, b, c (world_builder presets "room a" / "room b" / "room c",
         see the comment above them): load the world, READ, then name the
         place the way a truthful owner would: name_place {name} when the
         robot said new (it stored the place: this renames it), name_place
         {name, new: true} when it said known or ambiguous ("no, this is a
         different place").
  TEST   the case table (CASES): load the world, READ. A case with a `move`
         switches recognition OFF before the load, walks there (goto, then
         turn to the heading) and switches it back ON, so the only
         recognition is the one at the moved pose.
  --relook  after a READ: turn 30 deg left and force a fresh recognition
         (POST /api/place {action: recognize, force: true}); "relook agrees"
         = the same place again (a never-seen room: new, or the unnamed place
         its first recognition stored). Off by default: it adds samples to
         the places, which the next visits then benefit from.

READ = POST /api/place {action: "recognize"} (waits for this spawn's
recognition: a running one, or one run now; retried while it answers "not
recognised yet"), then POST /api/tool/where_am_i {} and GET /api/state's
"place". The verdict scored is where_am_i's (the recognize answer's, then the
state's, when it has none); the robot's pose is read before and after (the
cockpit's own second look turns it), so the bench knows what the eye saw.

THE WORLD NAME NEVER REACHES THE RECOGNITION PATH. Every world load is
POST /api/world {name: "pb-<8 hex>", spec: <preset> + {"visit": <8 hex>}}:
an opaque name, and a spec whose hash differs on every visit, so nothing
keyed by the name or the spec (memory_key) can carry a room across visits;
only the geometry the robot senses is shared. Two cases load a room under
the world name ANOTHER room was enrolled with ("b named as a", "a named as
b"): a recogniser that used the name answers the other room, which the
summary flags as name_leak.

SCORING (pure: score_visit, summarize).
  verdict   observed as: known (the right place) | known-wrong (a place, but
            not this one: for a never-seen room any known) | new | ambiguous
            | unknown | error (no readable verdict). correct = known for a
            known case, new for the never-seen room. A confusion matrix
            expected x observed over every test visit.
  changes   expected changes come from the spec difference between the case's
            preset and its room's enrolled preset (b + chair -> added box,
            c - ball -> missing ball), counted only when the changed spot was
            in the eye's view (86 deg, 0.15-2 m) at the robot's pose before
            AND after the READ (else "unobservable"). hit = the READ's
            confirmed changes name it (scene_memory.same_thing); every other
            confirmed change is a false alarm; a confirmed change from a READ
            that took one look is "single glance" (the two-look rule broken).
  looks     how many looks the cockpit took (1, or 2 after its own turn).
  confidence  per case: median (min-max). timings: arrival -> verdict, from
            the world load (or recognition switched back on after a move) to
            the answer: settling + sweep + look + embedding [+ turn + look],
            at the bench's physics speed; per visit, per trial.

Prints the verdict table and a per-case CONFIDENCE TABLE (every READ: its
confidence, its parts scan / desc / radio, the evidence, the senses compared,
the looks, the runner-up and the lead over it, each look's own confidence
when the cockpit reports per_look), writes --out (sim/out/place_bench.json:
every READ with the raw answers and `parsed` — parts, evidence, signals,
per_look and `eye`, the change check's per-name presence answers per look
(the first of EYE_KEYS in the answer: {name: true | false | null} or the
cockpit's eye_brief) — each expected change's eye answers in its score, the
confidence rows, the summary, the Markdown, the exact API calls made) and
prints a Markdown block for the place-recognition docs. The per-read
record is what the place_memory thresholds are derived from (see its
THRESHOLDS): a two-look recognition is only replayable when per_look
carries each look's parts.

THE FENCE. The bench's own cockpit talks to llama-swap through a loopback
proxy (brain_bench's, widened) that lists and forwards ONLY the vision model
and the CPU "embedding" model (the cockpit's Brains.embed uses the same
ROCKY_LLM_BASE_URL): the vision fallback chain (lfm2.5-vl -> gemma-4-12b ->
gemma-4-26b-a4b) would otherwise load a 21 GB model whenever the tested one
fails. --no-embed makes the fence refuse /v1/embeddings, so recognition runs
on the lidar sweep alone (the look still runs: it feeds the changes); the
fence's counts in the JSON show what went through. --dry-run: the fence
answers every look with one canned description and every embedding with a
hashed bag of words — nothing reaches llama-swap, nothing loads, and the
result is marked DRY RUN (plumbing, not a measurement). A --url cockpit is
not fenced (its fallback chain applies).

STOPPING. Ctrl-C, SIGTERM and SIGHUP stop the bench's cockpit and the fence,
remove the scratch directory, unload a vision model the bench loaded (unless
--keep-loaded or it is the resident lfm2.5-vl) and write the partial JSON.

Honesty: MuJoCo rooms are flat-shaded boxes on a checker floor and the lidar
is a perfect ray cast with the sim's pose as a perfect odometry, so every
number here is an upper bound for the real camera and puck. Each world load
respawns the robot at exactly the origin facing +x, the pose it enrolled
from, so every case but the moved one revisits from the enrolment pose; the
sim's map frame is the world frame (a real robot's restarts every session).
The lidar sees only what stands above its ~0.18 m scan plane. The vision
model describes the same frame nearly the same way every time, so trials
are repeats, not independent draws. Every bench room has walls (the lidar
gets a return in at least half its bins), so a LIDAR-EMPTY world is outside
what this bench measures: there two empty sweeps are not compared, the look
alone decides and place_memory never says "known" (review 2026-09-25).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import secrets
import shutil
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from collections import Counter

import httpx

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (HERE, ROOT, os.path.join(ROOT, "gait")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import vision_bench as vb                                                 # noqa: E402
from vision_bench import (Cockpit, parent_death_hook, listed_models, running_models,   # noqa: E402
                          unload, OWNER_PORT)
from brain_bench import (refuse_url, port_busy, Fence, BenchStopped, _child_cockpits,   # noqa: E402
                         _install_signals, _restore_signals, target_settings, restore_target)
from world_builder import PRESETS                                         # noqa: E402
from scene_memory import same_thing, norm_name                            # noqa: E402
from cockpit_brains import QUARANTINED, DEFAULT_BASE                      # noqa: E402

DEFAULT_PORT = 8795
OUT = os.path.join(HERE, "out", "place_bench.json")
DEFAULT_VISION = "lfm2.5-vl"
RESIDENT = frozenset({"lfm2.5-vl"})             # llama-swap's resident vision model: never unloaded here
EMBED_MODEL = "embedding"                       # llama-swap's CPU Qwen3-Embedding-0.6B (always warm)
EMBED_DIM = 1024
COCKPIT_PY = os.path.join(HERE, "cockpit.py")
VIEW_NEAR_M, VIEW_MAX_M = 0.15, 2.0             # the cockpit's eye_visible (sim/cockpit.py VIEW_*)

# ------------------------------------------------------------------ the rooms and the cases
ROOMS = {"a": "room a", "b": "room b", "c": "room c", "d": "room d"}   # label -> world_builder preset
PLACE_NAMES = {"a": "den", "b": "workshop", "c": "playroom"}          # what the owner calls them
ENROL_ROOMS = ("a", "b", "c")
SECOND_LOOK_DEG = 30.0      # the cockpit's own second-look turn (sim/cockpit.py PLACE_TURN_DEG, + = left)
MOVED = {"x": 0.25, "y": -0.15, "yaw_deg": 90.0}    # 0.29 m from the enrolment spot, a quarter turn
VERDICTS = ("known", "new", "ambiguous", "unknown")
OBSERVED = ("known", "known-wrong", "new", "ambiguous", "unknown", "error")
WORLD_NAME_RE = re.compile(r"^pb-[0-9a-f]{8}$")

# id: the row's name. room: which room it is (a/b/c enrolled, d never). preset: the world
# loaded. expect: known | new. world_name_of: load it under the world name THAT room was
# enrolled with (the name-swap proof). move: walk + turn there (recognition off) first.
# Order matters: each room's changed visit is its last one (a confirmed change updates the
# place's objects, and a later plain visit would then rightly report the change undone).
CASES = [
    dict(id="a same pose", room="a", preset="room a", expect="known"),
    dict(id="b named as a", room="b", preset="room b", expect="known", world_name_of="a"),
    dict(id="a named as b", room="a", preset="room a", expect="known", world_name_of="b"),
    dict(id="a moved + turned", room="a", preset="room a", expect="known", move=dict(MOVED)),
    dict(id="d never seen", room="d", preset="room d", expect="new"),
    dict(id="b + chair", room="b", preset="room b + chair", expect="known"),
    dict(id="c - ball", room="c", preset="room c - ball", expect="known"),
]


def _canon(o):
    return json.dumps(o, sort_keys=True)


def kind_name(obj):
    """A world object's plain name, as a look would say it (scene_memory normalised)."""
    return norm_name(str((obj or {}).get("kind") or "thing"))


def expected_changes(case, presets=None):
    """{"added": [obj...], "missing": [obj...]}: the spec difference between the
    case's preset and its room's enrolled preset (multiset, exact objects)."""
    presets = presets or PRESETS
    base = [_canon(o) for o in (presets[ROOMS[case["room"]]].get("objects") or [])]
    now = [_canon(o) for o in (presets[case["preset"]].get("objects") or [])]
    b, n = Counter(base), Counter(now)
    return {"added": [json.loads(s) for s in (n - b).elements()],
            "missing": [json.loads(s) for s in (b - n).elements()]}


def case_table(cases=None, presets=None):
    """CASES with every default filled in and the expected changes attached."""
    presets = presets or PRESETS
    out = []
    for c in (CASES if cases is None else cases):
        c = dict(c)
        c.setdefault("world_name_of", None)
        c.setdefault("move", None)
        if c["room"] not in ROOMS or c["preset"] not in presets:
            raise ValueError(f"case {c['id']!r}: unknown room or preset")
        if c["expect"] not in ("known", "new"):
            raise ValueError(f"case {c['id']!r}: expect must be known or new")
        if c["expect"] == "known" and c["room"] not in ENROL_ROOMS:
            raise ValueError(f"case {c['id']!r}: room {c['room']} is never enrolled, it cannot be known")
        if c["world_name_of"] is not None and c["world_name_of"] not in ENROL_ROOMS:
            raise ValueError(f"case {c['id']!r}: world_name_of must be an enrolled room")
        c["changes"] = (expected_changes(c, presets) if c["room"] in ENROL_ROOMS
                        else {"added": [], "missing": []})
        out.append(c)
    return out


def world_name():
    """An opaque, fresh world name: it says nothing about the room."""
    return "pb-" + secrets.token_hex(4)


def staged_spec(preset, token=None, presets=None):
    """A deep copy of the preset with a per-visit token, so the spec's hash (the
    scene memory's key for a non-preset name) is fresh on every visit too.
    world_builder ignores the extra key."""
    spec = json.loads(json.dumps((presets or PRESETS)[preset]))
    spec["visit"] = token or secrets.token_hex(4)
    return spec


def eye_sees(pose, obj):
    """Could a look from `pose` see the object's spot? The eye's view as the
    cockpit judges it (eye_visible: 86 deg wide, 0.15-2 m from the eye), with
    vision_bench's 2 deg margin at the edges."""
    t = vb.truth(pose, obj)
    return vb.in_view(t) and VIEW_NEAR_M <= t["centre_m"] <= VIEW_MAX_M


# ------------------------------------------------------------------ pure: reading the place payload
def _num(v):
    if isinstance(v, bool) or v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


_NEST = ("place", "recognition", "result", "rec")
_CHANGE_KEYS = ("added", "missing", "moved")
_TEXT_CAT = {"new here": "added", "added": "added", "missing": "missing", "moved": "moved"}


def _item_names(items):
    out = []
    for o in items if isinstance(items, (list, tuple)) else []:
        n = o.get("name") if isinstance(o, dict) else o
        n = norm_name(n)
        if n:
            out.append(n)
    return out


def _changes_from(d):
    """A change dict (added / missing / moved lists, maybe pending, maybe nested
    under confirmed) -> ({added, missing, moved: [names]}, [pending names])."""
    if not isinstance(d, dict):
        return None, []
    src = d.get("confirmed") if isinstance(d.get("confirmed"), dict) else d
    if not any(k in src for k in _CHANGE_KEYS):
        return None, _item_names(d.get("pending"))
    ch = {k: _item_names(src.get(k)) for k in _CHANGE_KEYS}
    return ch, _item_names(d.get("pending") if "pending" in d else src.get("pending"))


def changes_from_text(text):
    """verdict_text's tail ('— new here: box, chair; missing: ball') -> {added,
    missing, moved: [names]} or None. Counts-only lists ('missing: 2') and
    '+N' tails give what they name."""
    s = str(text or "")
    m = re.search(r"\s[—-]\s(.+)$", s)
    if not m:
        return None
    out = {k: [] for k in _CHANGE_KEYS}
    hit = False
    for part in m.group(1).split(";"):
        k, _, v = part.partition(":")
        key = _TEXT_CAT.get(k.strip().lower())
        if key is None:
            continue
        hit = True
        for n in v.split(","):
            n = re.sub(r"\s*\+\d+$", "", n.strip().rstrip("…"))
            if n and not n.isdigit():
                out[key].append(norm_name(n))
    return out if hit else None


EYE_KEYS = ("eye", "eye_checks", "eye_check", "checked", "presence")
_SIGNALS = ("scan", "desc", "radio")


def _seen_value(v):
    """One presence answer -> True | False | None (a dict answers by its 'seen')."""
    if isinstance(v, dict):
        v = v.get("seen")
    return v if v is True or v is False else None


def _one_eye(d):
    """One look's eye record -> {"seen": {name: True | False | None}, pose?, placed?,
    asked?, why?} or None. Takes the cockpit's eye_brief ({pose, seen, placed, asked,
    why}) or a bare presence map ({name: bool | None | {seen, ...}})."""
    if not isinstance(d, dict):
        return None
    if isinstance(d.get("seen"), dict):
        out = {"seen": {norm_name(k): _seen_value(v) for k, v in d["seen"].items() if norm_name(k)}}
        out.update({k: d[k] for k in ("pose", "placed", "asked", "why") if k in d})
        return out
    seen = {norm_name(k): _seen_value(v) for k, v in d.items()
            if norm_name(k) and (v is None or isinstance(v, (bool, dict)))}
    return {"seen": seen} if seen else None


def eye_looks(obj):
    """The change check's per-name PRESENCE answers in a place answer -> one entry per
    look [{"seen": {name: True | False | None}, pose?, placed?, why?}] ([] = none
    reported). Read from the first of EYE_KEYS at the top, or under changes / diff: a
    list (per look) or one dict (one look), each an eye_brief or a bare presence map."""
    if not isinstance(obj, dict):
        return []
    for src in (obj, obj.get("changes"), obj.get("diff")):
        if not isinstance(src, dict):
            continue
        for k in EYE_KEYS:
            v = src.get(k)
            if isinstance(v, list):
                looks = [e for e in (_one_eye(x) for x in v) if e is not None]
                if looks:
                    return looks
            elif isinstance(v, dict):
                e = _one_eye(v)
                if e is not None:
                    return [e]
    return []


def _parts(p):
    """{scan, desc, radio} with numbers or None, or None when the answer has no parts."""
    if not isinstance(p, dict):
        return None
    return {k: _num(p.get(k)) for k in _SIGNALS}


def _signals_of(obj, parts):
    s = obj.get("signals")
    if isinstance(s, (list, tuple)):
        return [str(x) for x in s]
    return [k for k in _SIGNALS if (parts or {}).get(k) is not None] if parts is not None else None


def _per_look(v):
    out = []
    for r in v if isinstance(v, list) else []:
        if isinstance(r, dict):
            parts = _parts(r.get("parts"))
            out.append({"verdict": r.get("verdict"), "place_id": r.get("place_id"), "name": r.get("name"),
                        "confidence": _num(r.get("confidence")), "parts": parts,
                        "signals": _signals_of(r, parts), "evidence": _num(r.get("evidence"))})
    return out


def parse_place(obj, now=None):
    """A where_am_i / POST /api/place recognize answer or /api/state's "place" ->
    {verdict, place_id, name, confidence, second {place_id, name, confidence, parts},
    changes {added, missing, moved: [names]} | None, pending [names], looks, by,
    status, text, t, parts {scan, desc, radio} | None, evidence, signals (the answer's,
    else the non-None parts), per_look [{verdict, place_id, name, confidence, parts,
    signals, evidence}] (each look of a two-look recognition, when reported), eye
    (eye_looks: the change check's per-name presence answers, per look), look_error,
    note} or None (nothing that carries a verdict). Accepts the verdict at the top or
    nested under place / recognition / result; changes under changes / diff / change
    (a {added, missing, moved, pending} dict, or {confirmed: {...}, pending}), as
    top-level added / missing / moved lists, or in the text's tail (verdict_text).
    t: 't' / 'time' / 'at', else now - age_s."""
    if not isinstance(obj, dict):
        return None
    if "verdict" not in obj:
        for k in _NEST:
            p = parse_place(obj.get(k), now) if isinstance(obj.get(k), dict) else None
            if p is not None:
                if p["changes"] is None:                 # changes reported beside the nested verdict
                    outer = parse_place(dict(obj, verdict=p["verdict"], **{k: None}), now)
                    if outer and outer["changes"] is not None:
                        p["changes"], p["pending"] = outer["changes"], outer["pending"] or p["pending"]
                for key, fn in (("eye", eye_looks), ("per_look", lambda o: _per_look(o.get("per_look")))):
                    if not p[key]:                       # reported beside the nested verdict
                        p[key] = fn(obj)
                return p
        return None
    verdict = str(obj.get("verdict") or "").strip().lower()
    sec = obj.get("second") if isinstance(obj.get("second"), dict) else {}
    changes, pending = None, []
    for k in ("changes", "diff", "change"):
        if isinstance(obj.get(k), dict):
            changes, pending = _changes_from(obj[k])
            if changes is not None or pending:
                break
    if changes is None and any(isinstance(obj.get(k), list) for k in _CHANGE_KEYS):
        changes = {k: _item_names(obj.get(k)) for k in _CHANGE_KEYS}
    if not pending and isinstance(obj.get("pending"), list):
        pending = _item_names(obj["pending"])
    text = obj.get("text") if isinstance(obj.get("text"), str) else None
    if changes is None and text:
        changes = changes_from_text(text)
    t = None
    for k in ("t", "time", "at"):
        t = _num(obj.get(k))
        if t is not None:
            break
    if t is None and _num(obj.get("age_s")) is not None:
        t = (time.time() if now is None else now) - _num(obj.get("age_s"))
    looks = _num(obj.get("looks"))
    parts = _parts(obj.get("parts"))
    return {"verdict": verdict, "place_id": obj.get("place_id"), "name": obj.get("name"),
            "confidence": _num(obj.get("confidence")),
            "second": {"place_id": sec.get("place_id"), "name": sec.get("name"),
                       "confidence": _num(sec.get("confidence")), "parts": _parts(sec.get("parts"))},
            "changes": changes, "pending": pending, "looks": None if looks is None else int(looks),
            "by": obj.get("by"), "status": obj.get("status"), "text": text, "t": t,
            "parts": parts, "evidence": _num(obj.get("evidence")), "signals": _signals_of(obj, parts),
            "per_look": _per_look(obj.get("per_look")), "eye": eye_looks(obj),
            "look_error": obj.get("look_error"), "note": obj.get("note")}


def label_of(parsed, ids, names):
    """Which enrolled room (a / b / c) a reading points at: by place id first,
    then by the name the bench gave it; None for anything else."""
    if not parsed:
        return None
    pid, nm = parsed.get("place_id"), parsed.get("name")
    for lab, i in (ids or {}).items():
        if pid and i and pid == i:
            return lab
    key = str(nm or "").strip().lower()
    for lab, n in (names or {}).items():
        if key and key == str(n).strip().lower():
            return lab
    return None


def observe(expect, room, parsed, ids, names):
    """(observed class, label) for one reading of a visit to `room`."""
    if not parsed or parsed.get("verdict") not in VERDICTS:
        return "error", None
    v = parsed["verdict"]
    if v == "known":
        lab = label_of(parsed, ids, names)
        return ("known" if expect == "known" and lab == room else "known-wrong"), lab
    return v, (label_of(parsed, ids, names) if v == "ambiguous" else None)


def is_correct(expect, observed):
    return observed == ("known" if expect == "known" else "new")


def relook_ok(expect, room, parsed2, ids, names):
    """Does a forced second recognition (--relook) agree with the room? Known: the
    same place again. Never seen: new, or known as a place that is none of the
    enrolled ones (the unnamed place its first recognition stored)."""
    obs, lab = observe(expect, room, parsed2, ids, names)
    if expect == "known":
        return obs == "known"
    return obs == "new" or ((parsed2 or {}).get("verdict") == "known" and lab is None)


def _all_changes(parsed):
    ch = (parsed or {}).get("changes") or {}
    return [(k, n) for k in _CHANGE_KEYS for n in ch.get(k) or []]


def eye_answers(parsed, name):
    """What the eye answered about `name` in each look of a READ (its per-name presence
    answers, same_thing): [True | False | None per look] — None = not asked or no usable
    answer; [] when the READ reported no presence answers at all."""
    out = []
    for lk in (parsed or {}).get("eye") or []:
        hits = [v for k, v in (lk.get("seen") or {}).items() if same_thing(k, name)]
        out.append(True if True in hits else False if False in hits else None)
    return out


def score_changes(expected, parsed, observable=None):
    """expected {"added": [obj], "missing": [obj]} (world objects); parsed: the READ
    (its confirmed changes, pending, looks and eye answers); observable(obj) -> was its
    spot in the eye's view before and after the READ (None = always). -> {expected,
    hits, misses, unobservable, false_alarms, single_glance, pending, looks} (lists of
    "added box"-style strings) + eye {tag: [the eye's answer per look]} for every
    expected change (a miss the eye answered True for is the cockpit's; one it
    answered None for was never asked)."""
    reported = [[k, n, False] for k, n in _all_changes(parsed)]
    looks = (parsed or {}).get("looks")
    out = {"expected": [], "hits": [], "misses": [], "unobservable": [], "false_alarms": [],
           # a confirmed change from a READ that took ONE look: the two-look rule broken
           "single_glance": ([f"{k} {n}" for k, n, _ in reported] if looks is not None and looks < 2 else []),
           "pending": list((parsed or {}).get("pending") or []), "looks": looks, "eye": {}}
    for kind in ("added", "missing"):
        for obj in expected.get(kind) or []:
            tag = f"{kind} {kind_name(obj)}"
            out["eye"][tag] = eye_answers(parsed, kind_name(obj))
            ok = True
            if observable is not None:
                try:
                    ok = bool(observable(obj))
                except Exception:                        # noqa: BLE001 — unknown = not claimed
                    ok = False
            if not ok:
                out["unobservable"].append(tag)
                continue
            out["expected"].append(tag)
            for r in reported:
                if not r[2] and r[0] == kind and same_thing(kind_name(obj), r[1]):
                    r[2] = True
                    out["hits"].append(tag)
                    break
            else:
                out["misses"].append(tag)
    out["false_alarms"] = [f"{k} {n}" for k, n, used in reported if not used]
    return out


def score_visit(case, parsed, ids, names, observable=None, relook=None):
    """One test visit -> the scored fields (pure). relook: the --relook READ or None."""
    obs, lab = observe(case["expect"], case["room"], parsed, ids, names)
    swapped = case.get("world_name_of")
    out = {"observed": obs, "label": lab, "correct": is_correct(case["expect"], obs),
           "confidence": (parsed or {}).get("confidence"), "looks": (parsed or {}).get("looks"),
           # the READ named exactly the room whose world name this visit borrowed
           "name_leak": bool(swapped) and obs == "known-wrong" and lab == swapped,
           "changes": score_changes(case["changes"], parsed, observable),
           "relook": None}
    if relook is not None:
        o2, l2 = observe(case["expect"], case["room"], relook, ids, names)
        out["relook"] = {"observed": o2, "label": l2, "confidence": (relook or {}).get("confidence"),
                         "agrees": relook_ok(case["expect"], case["room"], relook, ids, names)}
    return out


# ------------------------------------------------------------------ pure: summary
def _stats(xs, nd=2):
    xs = [x for x in xs if x is not None]
    if not xs:
        return {"median": None, "min": None, "max": None, "n": 0}
    return {"median": round(statistics.median(xs), nd), "min": round(min(xs), nd),
            "max": round(max(xs), nd), "n": len(xs)}


def _p95(xs):
    xs = sorted(x for x in xs if x is not None)
    return round(xs[max(0, math.ceil(0.95 * len(xs)) - 1)], 2) if xs else None


def _frac(k, n):
    return f"{k}/{n}"


def verdict_tag(obs, lab):
    """'known a', 'known-wrong b', 'known-wrong (unnamed)', 'ambiguous (a?)', 'new', 'error'."""
    if obs in ("known", "known-wrong"):
        return f"{obs} {lab or '(unnamed)'}"
    if obs == "ambiguous" and lab:
        return f"ambiguous ({lab}?)"
    return obs


def summarize(runs):
    """runs: [{"trial", "enrol": [visit rows], "tests": [visit rows], "wall_s"}] ->
    the summary (confusion matrix, per case, changes, looks, name swap, enrolment,
    relook, timings)."""
    tests = [r for run in runs for r in run.get("tests") or []]
    enrols = [r for run in runs for r in run.get("enrol") or []]
    matrix = {e: {o: 0 for o in OBSERVED} for e in ("known", "new")}
    per_case, order = {}, []
    def arrival_s(rd):
        return rd.get("arrival_s") if rd.get("arrival_s") is not None else rd.get("wall_s")
    reads = [arrival_s(rd) for r in tests + enrols for rd in r.get("reads") or [] if rd.get("phase") == "arrival"]
    for r in tests:
        s = r.get("score") or {}
        obs = s.get("observed", "error")
        matrix[r["expect"]][obs] += 1
        pc = per_case.get(r["case"])
        if pc is None:
            order.append(r["case"])
            pc = per_case[r["case"]] = {"expect": r["expect"], "room": r["room"], "n": 0, "correct": 0,
                                        "verdicts": Counter(), "conf": [], "wall": [], "reads": [],
                                        "two_looks": 0, "hits": 0, "expected": 0, "false_alarms": 0,
                                        "unobservable": 0, "single_glance": 0, "name_leak": 0,
                                        "relooks": 0, "relook_agrees": 0,
                                        "swapped_from": r.get("world_name_of")}
        pc["n"] += 1
        pc["correct"] += bool(s.get("correct"))
        pc["verdicts"][verdict_tag(obs, s.get("label"))] += 1
        pc["conf"].append(s.get("confidence"))
        pc["wall"].append(r.get("wall_s"))
        pc["reads"] += [arrival_s(rd) for rd in r.get("reads") or [] if rd.get("phase") == "arrival"]
        pc["two_looks"] += (s.get("looks") or 0) >= 2
        ch = s.get("changes") or {}
        pc["hits"] += len(ch.get("hits") or [])
        pc["expected"] += len(ch.get("expected") or [])
        pc["false_alarms"] += len(ch.get("false_alarms") or [])
        pc["unobservable"] += len(ch.get("unobservable") or [])
        pc["single_glance"] += bool(ch.get("single_glance"))
        pc["name_leak"] += bool(s.get("name_leak"))
        if s.get("relook") is not None:
            pc["relooks"] += 1
            pc["relook_agrees"] += bool(s["relook"].get("agrees"))
    cases = {}
    for cid in order:
        pc = per_case[cid]
        cases[cid] = {"expect": pc["expect"], "room": pc["room"], "swapped_from": pc["swapped_from"],
                      "n": pc["n"], "correct": _frac(pc["correct"], pc["n"]),
                      "verdicts": dict(pc["verdicts"]), "confidence": _stats(pc["conf"]),
                      "second_looks": _frac(pc["two_looks"], pc["n"]),
                      "changes": {"hits": pc["hits"], "expected": pc["expected"],
                                  "false_alarms": pc["false_alarms"], "unobservable": pc["unobservable"],
                                  "single_glance": pc["single_glance"]},
                      "relook_agrees": _frac(pc["relook_agrees"], pc["relooks"]) if pc["relooks"] else None,
                      "name_leak": pc["name_leak"], "wall_s": _stats(pc["wall"], 1),
                      "arrival_s": _stats(pc["reads"])}
    n = len(tests)
    correct = sum(bool((r.get("score") or {}).get("correct")) for r in tests)
    swap = [r for r in tests if r.get("world_name_of")]
    unchanged = [r for r in tests if not any((r.get("changes_expected") or {}).get(k) for k in ("added", "missing"))]
    ch_all = [(r.get("score") or {}).get("changes") or {} for r in tests]
    agree = [rd.get("agree") for r in tests + enrols for rd in r.get("reads") or [] if rd.get("agree") is not None]
    relooks = [r["score"]["relook"] for r in tests if (r.get("score") or {}).get("relook") is not None]

    def enrol_first(r):
        return ((r.get("reads") or [{}])[0].get("parsed") or {}).get("verdict")
    return {
        "visits": n,
        "correct": _frac(correct, n),
        "confusion": matrix,
        "cases": cases,
        "name_swap": {"correct": _frac(sum(bool(r["score"].get("correct")) for r in swap), len(swap)),
                      "name_leak": sum(bool(r["score"].get("name_leak")) for r in swap)},
        "changes": {
            "hits": sum(len(c.get("hits") or []) for c in ch_all),
            "expected": sum(len(c.get("expected") or []) for c in ch_all),
            "misses": sorted(Counter(m for c in ch_all for m in c.get("misses") or []).items()),
            "unobservable": sum(len(c.get("unobservable") or []) for c in ch_all),
            "false_alarms": sum(len(c.get("false_alarms") or []) for c in ch_all),
            "false_alarm_kinds": dict(Counter(f for c in ch_all for f in c.get("false_alarms") or [])),
            "false_alarm_visits_unchanged": _frac(
                sum(bool((r.get("score") or {}).get("changes", {}).get("false_alarms")) for r in unchanged),
                len(unchanged)),
            "single_glance_visits": sum(bool(c.get("single_glance")) for c in ch_all),
        },
        "second_looks": _frac(sum((((r.get("score") or {}).get("looks")) or 0) >= 2 for r in tests), n),
        "relook_agrees": _frac(sum(bool(x.get("agrees")) for x in relooks), len(relooks)) if relooks else None,
        "state_agrees_with_where_am_i": _frac(sum(bool(a) for a in agree), len(agree)),
        "enrol": {
            "first_new": _frac(sum(enrol_first(r) == "new" for r in enrols), len(enrols)),
            "first_not_known": _frac(sum(enrol_first(r) in ("new", "ambiguous") for r in enrols), len(enrols)),
            "named": _frac(sum(bool((r.get("name_place") or {}).get("ok")) for r in enrols), len(enrols)),
        },
        "timings": {"arrival_med_s": _stats(reads)["median"], "arrival_p95_s": _p95(reads),
                    "visit_med_s": _stats([r.get("wall_s") for r in tests], 1)["median"],
                    "enrol_visit_med_s": _stats([r.get("wall_s") for r in enrols], 1)["median"],
                    "trial_med_s": _stats([run.get("wall_s") for run in runs], 1)["median"]},
    }


# ------------------------------------------------------------------ pure: rendering
def _fmt(v, nd=2, dash="—"):
    if v is None:
        return dash
    return f"{v:.{nd}f}" if isinstance(v, float) else str(v)


def _conf_cell(st):
    if not st or st.get("median") is None:
        return "—"
    if st["n"] == 1 or st["min"] == st["max"]:
        return _fmt(st["median"])
    return f"{_fmt(st['median'])} ({_fmt(st['min'])}–{_fmt(st['max'])})"


def _verdicts_cell(v):
    return ", ".join(f"{k} ×{n}" for k, n in sorted(v.items(), key=lambda kv: (-kv[1], kv[0]))) or "—"


def _expect_cell(c):
    e = f"known {c['room']}" if c["expect"] == "known" else "new"
    if c.get("swapped_from"):
        e += f" (world name of {c['swapped_from']})"
    return e


def _changes_cell(c):
    ch = c.get("changes") or {}
    parts = []
    if ch.get("expected") or ch.get("unobservable"):
        parts.append(f"{ch.get('hits', 0)}/{ch.get('expected', 0)} found")
        if ch.get("unobservable"):
            parts.append(f"{ch['unobservable']} out of view")
    if ch.get("false_alarms"):
        parts.append(f"{ch['false_alarms']} false alarm{'s' if ch['false_alarms'] != 1 else ''}")
    if ch.get("single_glance"):
        parts.append(f"{ch['single_glance']} after one glance")
    return "; ".join(parts) or "none (as expected)"


MD_CASE_HEADER = ("| case | expected | verdicts | correct | confidence median (min–max) | "
                  "second look taken | changes (two looks agree) | arrival → verdict median (s) |")
MD_MATRIX_HEADER = "| expected \\ observed | " + " | ".join(OBSERVED) + " |"


def render_markdown(result):
    s = result.get("summary") or {}
    fence = result.get("fence") or {}
    fc = fence.get("counts") or {}
    head = (f"**Place bench** — {result.get('date', '?')}: cockpit {result.get('cockpit') or '—'}"
            f"{' (own, fenced)' if result.get('fenced') else ''}, vision `{result.get('vision_model')}`, "
            f"description embeddings {'on' if result.get('embed') else 'off'}"
            + (f" ({fc.get('embeddings', 0)} through the fence)" if result.get("fenced") and result.get("embed")
               else "")
            + f", {result.get('trials_done', 0)} trial(s) of {len(s.get('cases') or {})} cases, "
            f"speed {result.get('speed')}x, recognition switched on via {result.get('recognize_via') or '?'}"
            f"{', --relook' if result.get('relook') else ''}.")
    if result.get("dry_run"):
        head = ("**DRY RUN — plumbing only, not a measurement** (the fence answered every look with one "
                "canned description and every embedding with a hashed bag of words). " + head)
    if result.get("interrupted"):
        head += f" INTERRUPTED ({result['interrupted']}): partial."
    lines = [head, "", MD_CASE_HEADER, "|" + "---|" * 8]
    for cid, c in (s.get("cases") or {}).items():
        lines.append(f"| {cid} | {_expect_cell(c)} | {_verdicts_cell(c['verdicts'])} | {c['correct']} | "
                     f"{_conf_cell(c['confidence'])} | {c.get('second_looks', '—')} | {_changes_cell(c)} | "
                     f"{_fmt((c.get('arrival_s') or {}).get('median'), 1)} |")
    lines += ["", MD_MATRIX_HEADER, "|" + "---|" * (len(OBSERVED) + 1)]
    for e in ("known", "new"):
        row = (s.get("confusion") or {}).get(e) or {}
        lines.append(f"| {e} | " + " | ".join(str(row.get(o, 0)) for o in OBSERVED) + " |")
    ch = s.get("changes") or {}
    en = s.get("enrol") or {}
    sw = s.get("name_swap") or {}
    tm = s.get("timings") or {}
    lines += ["",
              f"Verdicts correct {s.get('correct', '—')}. Changes: {ch.get('hits', 0)}/{ch.get('expected', 0)} "
              f"expected changes declared (two looks agreeing; {ch.get('unobservable', 0)} out of the eye's view, "
              f"not counted); {ch.get('false_alarms', 0)} false alarm(s), in "
              f"{ch.get('false_alarm_visits_unchanged', '—')} visits with nothing changed; "
              f"{ch.get('single_glance_visits', 0)} visit(s) declared a change after one glance. The cockpit took "
              f"its second look in {s.get('second_looks', '—')} visits. "
              f"Name swap (the world name of another room): {sw.get('correct', '—')} right"
              f"{', NAME LEAK suspected in ' + str(sw['name_leak']) if sw.get('name_leak') else ''}. "
              f"Enrolment: first recognition new {en.get('first_new', '—')} "
              f"(not known {en.get('first_not_known', '—')}), named {en.get('named', '—')}."
              + (f" Relook from 30 deg further left agrees {s['relook_agrees']}." if s.get("relook_agrees") else "")
              + f" Arrival to verdict {_fmt(tm.get('arrival_med_s'), 1)} s median / "
              f"{_fmt(tm.get('arrival_p95_s'), 1)} p95 at {result.get('speed')}x physics (from the world load: "
              f"settling, sweep, look, embedding, and the turn + look when it takes a second); a visit "
              f"{_fmt(tm.get('visit_med_s'), 1)} s, a trial {_fmt(tm.get('trial_med_s'), 0)} s.",
              "",
              "(known = the right place; known-wrong = a place, but not this one. Clean MuJoCo rooms, a perfect "
              "ray-cast lidar and the sim's pose as odometry: an upper bound for the real camera and puck. Every "
              "world load respawns the robot at the pose it enrolled from, except the moved case. Every bench room "
              "has walls: a lidar-empty world is not measured here, and place_memory never calls one 'known'.)"]
    return "\n".join(lines)


def render_table(result):
    s = result.get("summary") or {}
    hdr = (f"{'case':20s} {'expected':26s} {'correct':>7s} {'conf med':>8s} {'2 looks':>7s} "
           f"{'changes':34s} verdicts")
    out = [hdr, "-" * len(hdr)]
    for cid, c in (s.get("cases") or {}).items():
        out.append(f"{cid:20s} {_expect_cell(c):26s} {c['correct']:>7s} "
                   f"{_fmt((c.get('confidence') or {}).get('median')):>8s} {c.get('second_looks', '—'):>7s} "
                   f"{_changes_cell(c)[:34]:34s} {_verdicts_cell(c['verdicts'])}")
    m = s.get("confusion") or {}
    out.append("")
    out.append(f"{'expected/observed':18s} " + " ".join(f"{o:>11s}" for o in OBSERVED))
    for e in ("known", "new"):
        out.append(f"{e:18s} " + " ".join(f"{(m.get(e) or {}).get(o, 0):>11d}" for o in OBSERVED))
    return "\n".join(out)


def confidence_rows(runs):
    """One row per READ (enrolments first, then the cases in run order), for the
    per-case confidence table and the JSON: {trial, case, phase, expect, observed,
    verdict, confidence, scan, desc, radio, evidence, signals, looks, second,
    second_conf, lead (confidence - second_conf), per_look [the looks' confidences]}.
    A test visit's arrival READ carries the scored `observed`; an enrolment's truth
    is "new" (the room is seen for the first time)."""
    out = []
    for run in runs or []:
        for key in ("enrol", "tests"):
            for r in run.get(key) or []:
                case = r.get("case") or f"enrol {r.get('room')}"
                for rd in r.get("reads") or []:
                    p = rd.get("parsed") or {}
                    parts = p.get("parts") or {}
                    sec = p.get("second") or {}
                    c, c2 = _num(p.get("confidence")), _num(sec.get("confidence"))
                    scored = key == "tests" and rd.get("phase") == "arrival"
                    row = {"trial": run.get("trial"), "case": case, "phase": rd.get("phase"),
                           "expect": r.get("expect") or "new",
                           "observed": (r.get("score") or {}).get("observed") if scored else p.get("verdict"),
                           "verdict": p.get("verdict"), "confidence": c}
                    row.update({k: parts.get(k) for k in ("scan", "desc", "radio")})
                    row.update({k: p.get(k) for k in ("evidence", "signals", "looks")})
                    has2 = c is not None and c2 is not None and bool(sec.get("place_id"))
                    row.update(second=sec.get("name") or sec.get("place_id"), second_conf=c2,
                               lead=round(c - c2, 3) if has2 else None,
                               per_look=[x.get("confidence") for x in p.get("per_look") or []] or None)
                    out.append(row)
    return out


def render_confidence_table(rows):
    """The per-case confidence table (printed after the verdict table): every READ's
    confidence, its parts, the senses compared, the runner-up and the lead over it —
    grouped by case, trials in order."""
    hdr = (f"{'case':18s} {'tr':>2s} {'phase':7s} {'observed':12s} {'conf':>5s} {'scan':>5s} {'desc':>5s} "
           f"{'evid':>5s} {'signals':12s} {'looks':>5s} {'lead':>6s}  runner-up / per look")
    out = [hdr, "-" * len(hdr)]
    order = []
    for r in rows or []:
        if r["case"] not in order:
            order.append(r["case"])
    for case in order:
        for r in sorted((x for x in rows if x["case"] == case),
                        key=lambda x: (x["trial"] or 0, x["phase"] or "")):
            sig = "+".join(r["signals"]) if isinstance(r["signals"], list) and r["signals"] else "—"
            tail = f"{r['second'] or '—'} {_fmt(r['second_conf'])}" if r["second"] else "—"
            if r["per_look"]:
                tail += " / looks " + ", ".join(_fmt(x) for x in r["per_look"])
            nums = " ".join(f"{_fmt(r[k]):>5s}" for k in ("confidence", "scan", "desc", "evidence"))
            out.append(f"{case[:18]:18s} {_fmt(r['trial']):>2s} {str(r['phase'] or '—')[:7]:7s} "
                       f"{str(r['observed'] or '—')[:12]:12s} {nums} {sig[:12]:12s} {_fmt(r['looks']):>5s} "
                       f"{_fmt(r['lead']):>6s}  {tail}")
    return "\n".join(out)


# ------------------------------------------------------------------ pure: the fence's rules
DRY_LOOK = ("A grey wall stands ahead, about 1.5 m away. A box is on the left, about 0.8 m away, and "
            "the open floor is on the right.")


def place_fence_allows(path, model, allowed, embed=True):
    """May the fence forward this request? Chat completions and /upstream/<m>/ for
    the allowed models; /v1/embeddings for the embedding model unless embed is off."""
    p = str(path or "").split("?")[0]
    allowed = set(allowed or ())
    if p == "/v1/chat/completions":
        return model in allowed and model != EMBED_MODEL
    if p == "/v1/embeddings":
        return bool(embed) and model in allowed
    m = re.match(r"^/upstream/([^/]+)/", p)
    return bool(m) and m.group(1) in allowed


def hashed_embedding(text, dim=EMBED_DIM):
    """A deterministic unit vector for --dry-run: a signed hashed bag of words."""
    v = [0.0] * dim
    for w in re.findall(r"[a-z]+", str(text or "").lower()):
        h = int.from_bytes(hashlib.sha256(w.encode()).digest()[:8], "big")
        v[h % dim] += 1.0 if (h >> 32) & 1 else -1.0
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return [x / n for x in v]


def dry_answer(path, body):
    """(status, JSON dict) the --dry-run fence answers with, or None to refuse."""
    p = str(path or "").split("?")[0]
    body = body if isinstance(body, dict) else {}
    if p == "/v1/chat/completions":
        return 200, {"id": "place-bench-dry", "object": "chat.completion", "model": body.get("model"),
                     "choices": [{"index": 0, "finish_reason": "stop",
                                  "message": {"role": "assistant", "content": DRY_LOOK}}],
                     "usage": {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0}}
    if p == "/v1/embeddings":
        inp = body.get("input")
        texts = inp if isinstance(inp, list) else [inp]
        return 200, {"object": "list", "model": body.get("model"),
                     "data": [{"object": "embedding", "index": i, "embedding": hashed_embedding(t)}
                              for i, t in enumerate(texts)]}
    return None


# ------------------------------------------------------------------ I/O: the fence
class PlaceFence(Fence):
    """brain_bench's fence, widened to a SET of models (the vision model + the
    embedding model) and /v1/embeddings, with counters and a --dry-run mode."""

    def __init__(self, allowed, embed=True, dry_run=False, upstream=DEFAULT_BASE):
        self.allowed_set = set(allowed)
        self.embed = bool(embed)
        self.dry_run = bool(dry_run)
        self.counts = Counter()
        super().__init__(upstream)

    def _send_json(self, h, code, obj):
        h.send(code, json.dumps(obj).encode())

    def _get(self, h):
        path = h.path.split("?")[0]
        try:
            if path == "/v1/models":
                if self.dry_run:
                    data = {"object": "list", "data": [
                        {"id": m, "object": "model",
                         "name": m + (" (vision, mmproj)" if m != EMBED_MODEL else "")}
                        for m in sorted(self.allowed_set)]}
                else:
                    data = self._up("GET", "/v1/models", timeout=10.0).json()
                    data["data"] = [e for e in data.get("data") or []
                                    if isinstance(e, dict) and e.get("id") in self.allowed_set]
                return self._send_json(h, 200, data)
            if place_fence_allows(path, None, self.allowed_set, self.embed):
                if self.dry_run:
                    return self._send_json(h, 200, {"modalities": {"vision": True}})
                r = self._up("GET", h.path, timeout=10.0)
                return h.send(r.status_code, r.content, r.headers.get("content-type", "application/json"))
        except Exception as e:                           # noqa: BLE001
            return self._send_json(h, 502, {"error": {"message": f"place_bench fence: {e}"}})
        self._send_json(h, 404, {"error": {"message": f"place_bench fence: {path} not forwarded"}})

    def _post(self, h):
        raw = h.rfile.read(int(h.headers.get("Content-Length") or 0))
        try:
            body = json.loads(raw or b"{}")
        except ValueError:
            body = {}
        model = body.get("model") if isinstance(body, dict) else None
        path = h.path.split("?")[0]
        kind = "embeddings" if path == "/v1/embeddings" else "chat" if path == "/v1/chat/completions" else "other"
        if not place_fence_allows(path, model, self.allowed_set, self.embed):
            self.counts[f"{kind}_refused"] += 1
            self.refused.append(model)
            why = ("place_bench fence: embeddings are off (--no-embed)" if kind == "embeddings" and not self.embed
                   else f"place_bench fence: only {sorted(self.allowed_set)} may answer here (asked {model})")
            return self._send_json(h, 503, {"error": {"type": "fenced", "message": why}})
        self.counts[kind] += 1
        if self.dry_run:
            ans = dry_answer(path, body)
            if ans is not None:
                return self._send_json(h, *ans)
        try:
            r = self._up("POST", path, raw)
        except httpx.TimeoutException:
            return self._send_json(h, 504, {"error": {"message": "place_bench fence: llama-swap timed out"}})
        except Exception as e:                           # noqa: BLE001
            return self._send_json(h, 502, {"error": {"message": f"place_bench fence: {e}"}})
        h.send(r.status_code, r.content, r.headers.get("content-type", "application/json"))


# ------------------------------------------------------------------ I/O: the cockpit
class BenchCockpit(Cockpit):
    """vision_bench's Cockpit that records every call it makes (method, path,
    body keys, one example body) and never raises on a non-JSON answer."""

    def __init__(self, url, timeout=180.0):
        super().__init__(url, timeout=timeout)
        self.calls = {}

    def _note(self, method, path, body):
        k = f"{method} {path}"
        e = self.calls.setdefault(k, {"n": 0, "keys": [], "example": body})
        e["n"] += 1
        for key in (body or {}):
            if key not in e["keys"]:
                e["keys"].append(key)

    @staticmethod
    def _json(r):
        try:
            return r.json()
        except ValueError:
            return {"ok": False, "error": f"HTTP {r.status_code}: {r.text[:200]}"}

    def post(self, path, body=None, timeout=None):
        self._note("POST", path, body or {})
        kw = {"timeout": timeout} if timeout else {}
        return self._json(self.c.post(self.url + path, json=body or {}, **kw))

    def get(self, path):
        self._note("GET", path, None)
        return self._json(self.c.get(self.url + path))


def place_env_names(src_path=COCKPIT_PY):
    """The ROCKY_*PLACE* environment variables sim/cockpit.py reads (a place
    memory directory the bench must point at its scratch copy, whatever it is
    called), from the source; [] when the file cannot be read."""
    try:
        with open(src_path) as f:
            src = f.read()
    except OSError:
        return []
    return sorted(set(re.findall(r"\b(ROCKY_[A-Z0-9_]*PLACE[A-Z0-9_]*)\b", src)))


class OwnCockpit:
    """The bench's own cockpit process: fenced, a scratch conf and a scratch
    memory directory per start (a fresh, empty place memory each trial)."""

    def __init__(self, port, fence, tmp, log, extra_args=()):
        self.port, self.fence, self.tmp, self.log = port, fence, tmp, log
        self.extra_args = list(extra_args)
        self.proc, self.ck, self.starts = None, None, 0
        self.memory_dir = None

    def env(self):
        mem = os.path.join(self.tmp, f"memory-{self.starts}")
        out = {"ROCKY_LLM_BASE_URL": self.fence.base_url,
               "ROCKY_COCKPIT_CONF": os.path.join(self.tmp, "cockpit.json"),   # never ~/.config
               "ROCKY_MEMORY_DIR": mem}
        for k in place_env_names():                   # a separate place directory, if the cockpit has one
            out[k] = mem
        return out

    def start(self):
        if port_busy(self.port):
            raise SystemExit(f"something already answers on 127.0.0.1:{self.port} (a cockpit left behind by an "
                             "earlier run?) — the bench would drive THAT one, unfenced. Stop it, or pick --port")
        self.starts += 1
        over = self.env()
        self.memory_dir = over["ROCKY_MEMORY_DIR"]
        os.makedirs(self.memory_dir, exist_ok=True)
        env = dict(os.environ, **over)
        env["MUJOCO_GL"] = os.environ.get("MUJOCO_GL") or "egl"
        log = open(os.path.join(self.tmp, f"cockpit_{self.port}_{self.starts}.log"), "w")
        argv = [sys.executable, COCKPIT_PY, "--port", str(self.port), "--world", "flat"] + self.extra_args
        try:
            self.proc = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                         preexec_fn=parent_death_hook())   # a SIGKILLed bench takes it along
            ck = BenchCockpit(f"http://127.0.0.1:{self.port}")
            t_end = time.monotonic() + 90
            while True:
                if self.proc.poll() is not None:
                    log.flush()
                    tail = open(log.name).read()[-600:]
                    raise SystemExit(f"the bench cockpit exited ({self.proc.returncode}); {log.name}:\n{tail}")
                if ck.alive():
                    break
                if time.monotonic() > t_end:
                    raise SystemExit(f"the bench cockpit did not come up in 90 s; see {log.name}")
                time.sleep(0.5)
        except BaseException:
            self.stop()
            for pid in _child_cockpits(self.port):       # interrupted mid-start: no orphan
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    pass
            raise
        if self.ck is None:
            self.ck = ck
        else:                                            # callers hold self.ck: same URL, a fresh client
            old, self.ck.c = self.ck.c, ck.c
            old.close()
        return self.ck

    def stop(self):
        p, self.proc = self.proc, None
        if p is None:
            return
        p.terminate()
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            p.kill()
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                pass

    def restart(self, why, extra_args=None):
        self.log(f"  a fresh bench cockpit ({why})")
        self.stop()
        if extra_args is not None:
            self.extra_args = list(extra_args)
        t_end = time.monotonic() + 10
        while port_busy(self.port) and time.monotonic() < t_end:
            time.sleep(0.25)
        return self.start()


def _brief(r, n=300):
    """A response, trimmed for the JSON (long strings cut, deep structures kept)."""
    if isinstance(r, dict):
        return {k: _brief(v, n) for k, v in r.items()}
    if isinstance(r, list):
        return [_brief(v, n) for v in r[:40]]
    if isinstance(r, str) and len(r) > n:
        return r[:n] + "…"
    return r


def set_recognize(ck, on):
    """POST /api/awareness {recognize: on} -> did the cockpit confirm it?"""
    r = ck.post("/api/awareness", {"recognize": bool(on)})
    a = (r or {}).get("awareness") if isinstance(r, dict) else None
    return isinstance(a, dict) and a.get("recognize") is bool(on)


def awareness_on(ck):
    """Reactions and the curious look off (nothing but the bench and the cockpit's
    own recognition moves or looks), then recognition on. -> confirmed?"""
    ck.post("/api/awareness", {"reactions": False, "curious": False})
    return set_recognize(ck, True)


def recognize_state(ck):
    """GET /api/awareness -> awareness.recognize (None when it does not say)."""
    try:
        return ((ck.get("/api/awareness") or {}).get("awareness") or {}).get("recognize")
    except Exception:                                   # noqa: BLE001
        return None


def tool_names(ck):
    """The tool names the cockpit's capability snapshot lists (empty on failure)."""
    try:
        caps = ck.get("/api/capabilities") or {}
    except Exception:                                   # noqa: BLE001
        return set()
    return {t.get("name") for t in caps.get("tools") or [] if isinstance(t, dict)}


def wipe_places(ck):
    """Empty a cockpit's place memory: POST /api/tool/forget_place {"name": "all"}
    (the cockpit writes places.json.bak first). -> True when it said ok."""
    try:
        r = ck.post("/api/tool/forget_place", {"name": "all"})
    except Exception:                                   # noqa: BLE001
        return False
    return bool(isinstance(r, dict) and r.get("ok"))


def pose(ck):
    try:
        p = (ck.post("/api/tool/status") or {}).get("pose")
    except Exception:                                   # noqa: BLE001
        return None
    return {k: round(float(p[k]), 3) for k in ("x", "y", "yaw_deg")} if isinstance(p, dict) else None


def _wrap(a):
    return (float(a) + 180.0) % 360.0 - 180.0


def stage(ck, preset, name, settle_s):
    """Load a room under an opaque name: a spawn (the robot at the origin facing
    +x) — with recognition on, the cockpit recognises once it stands still."""
    token = secrets.token_hex(4)
    spec = staged_spec(preset, token)
    r = ck.post("/api/world", {"name": name, "spec": spec})
    time.sleep(settle_s)
    return {"world_name": name, "visit": token, "ok": bool((r or {}).get("ok"))}


def turn(ck, deg, timeout=120.0, settle_s=0.5):
    r = ck.post("/api/tool/turn", {"deg": round(float(deg), 1)}, timeout=timeout)
    time.sleep(settle_s)
    return {"deg": round(float(deg), 1), "result": _brief(r, 120)}


def move_to(ck, target, timeout=180.0, settle_s=0.5):
    """goto (x, y), then turn to the heading. -> what happened and the end pose."""
    out = {"target": dict(target)}
    r = ck.post("/api/tool/goto", {"x": float(target["x"]), "y": float(target["y"])}, timeout=timeout)
    out["goto"] = _brief(r, 120)
    time.sleep(settle_s)
    p = pose(ck)
    if p is not None and target.get("yaw_deg") is not None:
        d = _wrap(float(target["yaw_deg"]) - p["yaw_deg"])
        if abs(d) >= 5.0:
            out["turn"] = turn(ck, d, timeout=timeout, settle_s=settle_s)
            p = pose(ck)
    out["pose"] = p
    if p is not None:
        out["err_m"] = round(math.hypot(p["x"] - target["x"], p["y"] - target["y"]), 3)
        if target.get("yaw_deg") is not None:
            out["yaw_err_deg"] = round(_wrap(p["yaw_deg"] - float(target["yaw_deg"])), 1)
    return out


def _same_reading(a, b):
    if not a or not b:
        return None
    return a.get("verdict") == b.get("verdict") and (a.get("place_id") or a.get("name")) == \
        (b.get("place_id") or b.get("name"))


def read_place(ck, phase, since, timeout, force=False, poll_s=1.0, t_ref=None):
    """One READ: the pose; POST /api/place {action: recognize, force} (waits for this
    spawn's recognition; asked again while it answers without a verdict — the robot
    not standing still yet, a spawn meanwhile — until `timeout`); POST
    /api/tool/where_am_i {}; GET /api/state's place; the pose again. The verdict
    scored: where_am_i's when it has one no older than `since` (a time.time()),
    else the recognize answer's, else the state's. wall_s = the recognize call(s);
    arrival_s = from t_ref (a time.monotonic(): the world load, or recognition
    switched back on) to the answer — the cockpit starts on its own after a spawn,
    so the call alone can be ~0 s."""
    p0 = pose(ck)
    t0 = time.monotonic()
    deadline = t0 + timeout
    attempts, rr, pr = 0, None, None
    while True:
        attempts += 1
        try:
            rr = ck.post("/api/place", {"action": "recognize", "force": bool(force)},
                         timeout=max(5.0, deadline - time.monotonic()))
        except httpx.TimeoutException:
            rr = {"ok": False, "error": f"recognize: no answer within {timeout:.0f} s"}
        pr = parse_place(rr)
        if isinstance(rr, dict) and rr.get("ok") and pr and pr["verdict"] in VERDICTS:
            break
        if (isinstance(rr, dict) and rr.get("recognize") is False) or time.monotonic() > deadline:
            break                                       # recognition off, or out of time
        time.sleep(poll_s)
    t_ans = time.monotonic()
    wall = round(t_ans - t0, 2)
    try:
        w = ck.post("/api/tool/where_am_i", {}, timeout=60.0)
    except httpx.TimeoutException:
        w = {"ok": False, "error": "where_am_i: no answer within 60 s"}
    pw = parse_place(w)
    st = ck.get("/api/state")
    place = (st or {}).get("place") if isinstance(st, dict) else None
    ps = parse_place(place)
    p1 = pose(ck)
    stale = bool(pw and pw.get("t") is not None and pw["t"] < since - 1.0)
    if pw and pw["verdict"] in VERDICTS and not stale:
        source, parsed = "where_am_i", pw
    elif pr and pr["verdict"] in VERDICTS:
        source, parsed = "recognize", pr
    elif ps and ps["verdict"] in VERDICTS:
        source, parsed = "state", ps
    else:
        source, parsed = None, None
    return {"phase": phase, "pose": p0, "pose_after": p1, "wall_s": wall,
            "arrival_s": None if t_ref is None else round(t_ans - t_ref, 2), "attempts": attempts,
            "source": source, "stale_where_am_i": stale, "recognize": _brief(rr), "where_am_i": _brief(w),
            "state_place": _brief(place), "parsed": parsed, "state_parsed": ps, "agree": _same_reading(pw, ps)}


def observable_fn(poses):
    """obj -> in the eye's view (eye_sees) at every pose given."""
    def fn(obj):
        ps = [p for p in poses if p]
        if not ps:
            return False
        return all(eye_sees(p, obj) for p in ps)
    return fn


def name_place(ck, label, new):
    body = {"name": PLACE_NAMES[label]}
    if new:
        body["new"] = True
    r = ck.post("/api/tool/name_place", body)
    return r if isinstance(r, dict) else {"ok": False, "error": "no answer"}


def place_id_of(r):
    """The place id a name_place answer carries (place_id / id, or under place), or None."""
    if not isinstance(r, dict):
        return None
    for d in (r, r.get("place") if isinstance(r.get("place"), dict) else {}):
        for k in ("place_id", "id"):
            if isinstance(d.get(k), str) and d[k]:
                return d[k]
    return None


def _relook_tag(case, rl):
    """The log's relook clause: a never-seen room's relook that finds the place its own
    first recognition stored reads 'known itself (unnamed)', not 'known-wrong'."""
    if case["expect"] == "new" and rl["agrees"] and rl["observed"] == "known-wrong":
        tag = "known itself (unnamed)"
    else:
        tag = verdict_tag(rl["observed"], rl["label"])
    return tag + (" (agrees)" if rl["agrees"] else " (DISAGREES)")


def run_trial(ck, trial, cases, cfg, log):
    """One trial on a cockpit with recognition on and an empty place memory: enrol
    a, b, c, then every case. cfg: settle_s (after a world load), motion_settle_s
    (after a goto / turn), timeout, first_timeout, relook (bool), relook_deg."""
    t_trial = time.monotonic()
    ms = getattr(cfg, "motion_settle_s", 0.5)
    ids, names, enrol_names = {}, {}, {}
    enrol, tests = [], []
    first_read = [True]

    def rd(phase, since, force=False, t_ref=None):
        to = cfg.first_timeout if first_read[0] else cfg.timeout
        first_read[0] = False
        return read_place(ck, phase, since, to, force=force, t_ref=t_ref)

    def relook(since):
        if not getattr(cfg, "relook", False):
            return None, None
        tn = turn(ck, getattr(cfg, "relook_deg", SECOND_LOOK_DEG), settle_s=ms)
        return tn, rd("relook", since, force=True, t_ref=time.monotonic())

    for lab in ENROL_ROOMS:
        t0 = time.monotonic()
        since = time.time()
        wname = world_name()
        enrol_names[lab] = wname
        st = stage(ck, ROOMS[lab], wname, cfg.settle_s)
        r1 = rd("arrival", since, t_ref=t0)
        p1 = r1.get("parsed") or {}
        # a truthful owner: "this is the den" after "new" (it stored the place: this names it);
        # "no, this is a different place: the den" after known / ambiguous
        nm = name_place(ck, lab, new=p1.get("verdict") != "new")
        names[lab] = PLACE_NAMES[lab]
        if place_id_of(nm):
            ids[lab] = place_id_of(nm)
        tn, r2 = relook(since)
        row = {"trial": trial, "room": lab, "preset": ROOMS[lab], "stage": st,
               "reads": [r for r in (r1, r2) if r is not None], "name_place": _brief(nm), "relook_turn": tn,
               "wall_s": round(time.monotonic() - t0, 1)}
        if r2 is not None:
            row["relook_known_self"] = relook_ok("known", lab, r2.get("parsed"), ids, names)
        enrol.append(row)
        log(f"  enrol {lab} ({ROOMS[lab]}, world {wname}): {p1.get('verdict')} {_fmt(p1.get('confidence'))} "
            f"[{p1.get('looks')} look(s)], named {PLACE_NAMES[lab]!r}{' as a new place' if p1.get('verdict') != 'new' else ''}"
            f" ({(nm.get('action') or 'ok') if nm.get('ok') else 'FAILED: ' + str(nm.get('error'))[:60]})"
            f"{'' if r2 is None else ', relook ' + str((r2.get('parsed') or {}).get('verdict'))} [{row['wall_s']} s]")
    for case in cases:
        t0 = time.monotonic()
        since = time.time()
        wname = enrol_names[case["world_name_of"]] if case.get("world_name_of") else world_name()
        mv, rec_off = None, None
        if case.get("move"):
            # recognition off for the load and the walk: the only recognition is at the moved pose
            rec_off = set_recognize(ck, False)
            st = stage(ck, case["preset"], wname, cfg.settle_s)
            mv = move_to(ck, case["move"], settle_s=ms)
            t_on = time.monotonic()
            mv["recognize_back_on"] = set_recognize(ck, True)
            since = time.time()
        else:
            t_on = t0
            st = stage(ck, case["preset"], wname, cfg.settle_s)
        r1 = rd("arrival", since, t_ref=t_on)
        tn, r2 = relook(since)
        score = score_visit(case, r1.get("parsed"), ids, names,
                            observable=observable_fn([r1.get("pose"), r1.get("pose_after")]),
                            relook=None if r2 is None else r2.get("parsed"))
        row = {"trial": trial, "case": case["id"], "room": case["room"], "preset": case["preset"],
               "expect": case["expect"], "world_name_of": case.get("world_name_of"), "stage": st,
               "recognize_off_for_move": rec_off, "move": mv, "relook_turn": tn,
               "reads": [r for r in (r1, r2) if r is not None],
               "changes_expected": {k: [kind_name(o) for o in v] for k, v in case["changes"].items()},
               "score": score, "wall_s": round(time.monotonic() - t0, 1)}
        tests.append(row)
        ch = score["changes"]
        rl = score.get("relook")
        log(f"  {case['id']:18s} {verdict_tag(score['observed'], score['label']):24s} "
            f"{'OK ' if score['correct'] else 'NO '} conf {_fmt(score['confidence'])} [{score['looks']} look(s)]; "
            f"changes hit {ch['hits'] or '-'} miss {ch['misses'] or '-'} false {ch['false_alarms'] or '-'}"
            f"{' ONE-GLANCE ' + str(ch['single_glance']) if ch['single_glance'] else ''}"
            f"{' NAME LEAK' if score['name_leak'] else ''}"
            f"{'' if rl is None else '; relook ' + _relook_tag(case, rl)}"
            f" [{row['wall_s']} s]")
    return {"trial": trial, "ids": dict(ids), "world_names": enrol_names, "enrol": enrol, "tests": tests,
            "wall_s": round(time.monotonic() - t_trial, 1)}


# ------------------------------------------------------------------ main
def write_result(result, path, lines):
    result["summary"] = summarize(result.get("runs") or [])
    result["confidence_rows"] = confidence_rows(result.get("runs") or [])
    md = render_markdown(result)
    result["markdown"] = md
    result["log"] = list(lines)
    with open(path, "w") as f:
        json.dump(result, f, indent=1, default=str)
    return md


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="The protocol, the cases, the scoring, the fence and the honesty notes: "
                                        "see the module docstring (sim/place_bench.py).")
    ap.add_argument("--url", help=f"a running cockpit on this machine's loopback (never :{OWNER_PORT}); "
                                  "its places are wiped (forget_place all) before each trial; "
                                  "default: start one on --port")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help="the bench's own cockpit (default %(default)s)")
    ap.add_argument("--vision-model", default=DEFAULT_VISION, help="the cockpit's vision role (default %(default)s)")
    ap.add_argument("--embed", action=argparse.BooleanOptionalAction, default=True,
                    help="description embeddings on (default) / off (--no-embed: the fence refuses them)")
    ap.add_argument("--trials", type=int, default=3, help="enrol + every case, N times (default %(default)s)")
    ap.add_argument("--cases", help="comma-separated case ids (default: all): " + ", ".join(c["id"] for c in CASES))
    ap.add_argument("--relook", action="store_true",
                    help="after each recognition: turn 30 deg left and force another (does it agree?)")
    ap.add_argument("--speed", type=float, default=2.0, help="physics speed (wall time only; default %(default)s)")
    ap.add_argument("--settle", type=float, default=0.5, help="s after a world load before asking (the cockpit "
                                                              "waits for a standstill itself)")
    ap.add_argument("--read-timeout", type=float, default=120.0, help="one recognition, s")
    ap.add_argument("--load-timeout", type=float, default=180.0, help="the first recognition (a cold load), s")
    ap.add_argument("--dry-run", action="store_true",
                    help="no GPU: the fence answers the looks and embeddings itself (plumbing, not a measurement)")
    ap.add_argument("--keep-loaded", action="store_true", help="leave a vision model the bench loaded loaded")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args(argv)

    if args.url:
        bad = refuse_url(args.url)
        if bad:
            raise SystemExit(bad)
    elif args.port == OWNER_PORT:
        raise SystemExit(f"refusing --port {OWNER_PORT}: that is the owner's live cockpit")
    if args.trials < 1:
        raise SystemExit("--trials must be at least 1")
    ids = [c["id"] for c in CASES]
    want = [x.strip() for x in (args.cases or "").split(",") if x.strip()] or ids
    unknown = [x for x in want if x not in ids]
    if unknown:
        raise SystemExit(f"unknown case(s) {unknown}; have: {', '.join(ids)}")
    cases = case_table([c for c in CASES if c["id"] in want])
    vm = args.vision_model
    if vm in QUARANTINED:
        raise SystemExit(f"{vm} is quarantined (GPU faults) — never used")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)

    lines = []

    def log(s):
        print(s, flush=True)
        lines.append(s)

    before = set()
    if not args.dry_run:
        listed = listed_models()
        if not listed:
            raise SystemExit("llama-swap does not answer GET /v1/models on :8080 — use --dry-run for plumbing")
        if vm not in listed:
            raise SystemExit(f"{vm} is not listed by llama-swap")
        if args.embed and EMBED_MODEL not in listed:
            log(f"note: llama-swap does not list {EMBED_MODEL!r}: the cockpit's embeddings will fail")
        before = running_models()
    cfg = argparse.Namespace(settle_s=args.settle, motion_settle_s=0.5, timeout=args.read_timeout,
                             first_timeout=args.load_timeout, relook=args.relook, relook_deg=SECOND_LOOK_DEG)
    result = {"date": time.strftime("%Y-%m-%d %H:%M"), "bench": "place_bench", "cockpit": None,
              "fenced": False, "dry_run": args.dry_run, "vision_model": vm, "embed": args.embed,
              "relook": args.relook, "speed": args.speed, "trials": args.trials, "trials_done": 0,
              "recognize_via": None,
              "rooms": {lab: {"preset": ROOMS[lab], "name": PLACE_NAMES.get(lab), "enrolled": lab in ENROL_ROOMS,
                              "spec": PRESETS[ROOMS[lab]]} for lab in ROOMS},
              "cases": [{k: v for k, v in c.items() if k != "changes"} |
                        {"changes": {k: [kind_name(o) for o in v] for k, v in c["changes"].items()}}
                        for c in cases],
              "runs": []}
    fence, own, tmp, ck, interrupted = None, None, None, None, None
    saved, saved_aw = None, None
    old_signals = _install_signals()
    try:
        if args.url:
            ck = BenchCockpit(args.url)
            if not ck.alive():
                raise SystemExit(f"no cockpit answers at {args.url}")
            # its roles persist in its conf file (by default the owner's ~/.config/rocky/cockpit.json)
            saved = target_settings(ck)
            if saved is None:
                raise SystemExit(f"could not read the roles of the cockpit at {args.url} (GET /api/models) — "
                                 "refusing: the bench's vision role would be saved to its conf file and could not "
                                 "be put back")
            try:
                saved_aw = (ck.get("/api/awareness") or {}).get("awareness")
            except Exception:                           # noqa: BLE001
                saved_aw = None
            log(f"cockpit {args.url}: NOT fenced (its vision fallback chain applies); its places are wiped before "
                "each trial (places.json.bak keeps the old set); its roles, speed and awareness are put back at "
                "the end")
        else:
            fence = PlaceFence({vm, EMBED_MODEL}, embed=args.embed, dry_run=args.dry_run)
            tmp = tempfile.mkdtemp(prefix="place_bench_")
            own = OwnCockpit(args.port, fence, tmp, log)
            ck = own.start()
            log(f"cockpit {ck.url} (own, fenced through {fence.base_url}"
                f"{', DRY RUN: the fence answers the looks and embeddings' if args.dry_run else ''})")
        result.update(cockpit=ck.url, fenced=fence is not None)

        def setup(first):
            """Speed, recognition on, the vision role. Recognition: the API first; the bench's
            own cockpit falls back to a restart with --recognize (a cockpit that does not know
            the flag exits, and start() says so with its log)."""
            ck.post("/api/speed", {"speed": args.speed})
            r = ck.post("/api/brain", {"vision_model": vm})
            if first and not (r or {}).get("ok"):
                log(f"note: POST /api/brain vision_model={vm}: {(r or {}).get('errors') or r}")
            ok = awareness_on(ck)
            if first:
                if ok:
                    result["recognize_via"] = "api"
                elif own is not None:
                    own.restart("POST /api/awareness {recognize: true} was not confirmed: trying the "
                                "--recognize flag", extra_args=["--recognize"])
                    ck.post("/api/speed", {"speed": args.speed})
                    ck.post("/api/brain", {"vision_model": vm})
                    ok = awareness_on(ck)
                    if not ok and recognize_state(ck) is not True:
                        raise SystemExit("place recognition would not switch on (neither POST /api/awareness "
                                         "{recognize: true} nor --recognize): is D057 in this cockpit?")
                    result["recognize_via"] = "flag"
                else:
                    raise SystemExit(f"{args.url} did not confirm POST /api/awareness {{recognize: true}}")
                tools = tool_names(ck)
                result["cockpit_tools"] = sorted(t for t in tools if t)
                missing = [t for t in ("where_am_i", "name_place", "forget_place") if tools and t not in tools]
                if missing:
                    raise SystemExit(f"the cockpit's capability list has no {missing}: is D057 in this cockpit?")

        for trial in range(1, args.trials + 1):
            if trial > 1 and own is not None:
                own.restart(f"trial {trial}: an empty place memory")
            setup(trial == 1 or own is not None)
            if own is None and not wipe_places(ck):
                raise SystemExit("the --url cockpit's places could not be wiped (POST /api/tool/forget_place "
                                 "{name: all})")
            log(f"trial {trial}/{args.trials}")
            run = run_trial(ck, trial, cases, cfg, log)
            result["runs"].append(run)
            result["trials_done"] = trial
    except BenchStopped as e:
        interrupted = e.signame
        raise
    except KeyboardInterrupt:
        interrupted = "SIGINT"
        raise
    finally:
        _restore_signals({s: signal.SIG_IGN for s in (old_signals or {})})   # a second kill: cleanup still runs
        try:
            if saved is not None and ck is not None:
                result["url_restored"] = restore_target(ck, saved, log)
                if isinstance(saved_aw, dict):
                    body = {k: saved_aw[k] for k in ("recognize", "reactions", "curious")
                            if isinstance(saved_aw.get(k), bool)}
                    try:
                        ck.post("/api/awareness", body)
                        result["url_restored"] += f"; awareness {body} restored"
                    except Exception as e:              # noqa: BLE001
                        result["url_restored"] += f"; awareness NOT restored ({type(e).__name__})"
            if ck is not None:
                result["api_calls"] = ck.calls
            if own is not None:
                own.stop()
                result["cockpit_starts"] = own.starts
            if fence is not None:
                fence.close()
                result["fence"] = {"allowed": sorted(fence.allowed_set), "embed": fence.embed,
                                   "dry_run": fence.dry_run, "counts": dict(fence.counts),
                                   "refused": [m for m in fence.refused if m][:20]}
            if tmp:
                shutil.rmtree(tmp, ignore_errors=True)
            if not args.dry_run and not args.keep_loaded and vm not in before and vm not in RESIDENT:
                if vm in running_models():
                    result["unloaded"] = {vm: unload(vm)}
            if interrupted and result["runs"]:
                result["interrupted"] = interrupted
                write_result(result, args.out, lines)
                print(f"\ninterrupted ({interrupted}): partial results in "
                      f"{os.path.relpath(os.path.abspath(args.out), ROOT)}", flush=True)
        finally:
            _restore_signals(old_signals)

    result["summary"] = summarize(result["runs"])
    log("")
    log(render_table(result))
    log("")
    log(render_confidence_table(confidence_rows(result["runs"])))
    md = write_result(result, args.out, lines)
    print("\nMarkdown for the place-recognition docs:\n")
    print(md)
    print(f"\nwrote {os.path.relpath(os.path.abspath(args.out), ROOT)}")
    return result


if __name__ == "__main__":
    main()
