"""Brain bench — how well a local model drives Pebble through the cockpit.

    python sim/brain_bench.py --models qwen3.5-4b                  # multimodal role, own cockpit :8792
    python sim/brain_bench.py --models qwen3.5-9b,gemma-4-12b --vision
    python sim/brain_bench.py --models qwen3.5-4b --mode local     # the text brain role
    python sim/brain_bench.py --models X --url http://127.0.0.1:8793   # a cockpit you started (loopback, NOT :8765)

For each model, one at a time (the 9B, the 12B and the 35B driver cannot
co-reside on this 16 GB card):

  1  skip it when it is quarantined or llama-swap does not list it;
  2  unload every running model (the CPU retrieval models excepted; the
     model under test too, so its load is cold and the VRAM baseline clean)
     and wait --idle s of true idle (GPU rule: 15-30 s between an unload
     and the next spawn);
  3  a fresh cockpit process when the situation line still carries an
     earlier model's eye context (see 9), a fresh obstacle-course world
     named after the model (the robot at the origin, an empty scene
     memory), then the nvidia-smi baseline;
  4  roles via POST /api/brain: --mode multimodal -> multimodal_model;
     --mode local -> model (the brain). vision_model (and, in local mode,
     multimodal_model) are set to the same model when the cockpit accepts it
     as vision-capable, so look / find_object / describe use it too;
  5  the cold first answer: wall time of the first /api/chat (a hello that
     needs no tool), which must be answered by THIS model (the response's
     "model" field, and no fallback note saying it failed) — otherwise the
     model is skipped with the reason. The tools that turn ran and the time
     the sim spent moving are recorded (first_answer_tools,
     first_answer_motion_s): a model that waves hello adds ~3 s of gesture
     at --speed 2, so first_answer_model_s = the wall time minus the motion
     is the number to hold against the 6.5 s qwen3.5-9b reference;
  6  vram_mib = nvidia-smi used - baseline, warm;
  7  decode t/s: one direct llama-swap call for ~150 tokens (llama-server's
     own timings when it returns them, else tokens / wall time);
  8  COMMANDS (--commands, first 20 = the list below): each line is typed
     into /api/chat (source 'typed': the wake-word gate is not what is
     measured), then the robot is stopped (POST /api/tool/stop — the cockpit
     has no /api/stop route; this is what the UI's STOP sends) and allowed to
     settle. The chat history is cleared before every line, so each line is
     scored on its own; the scene memory is kept (where-is / go-back need
     the earlier find). Scored per line by score_command (pure): ok |
     wrong-tool | wrong-args | no-tool | unsafe | timeout | not-scored;
  9  DESCRIBE (--describe): the ball left / right / ahead / nowhere (a box
     instead), "what do you see, and is it on your left or right?" in
     multimodal mode; scored by score_describe (pure). Each case gets its
     own world, named after the model (so its own scene memory: no earlier
     model's or case's sightings), and — the bench's own cockpit only — a
     fresh cockpit process before any case whose situation line still has
     an eye clause: the last look / find_object result rides in every chat
     turn and a world swap does not clear it ('orange ball on the left' from
     the previous case would give the side away). The situation text each
     case was sent with is recorded (situation, eye_context). With --url
     the eye context cannot be reset: stale_eye marks the rows it rode in;
  10 --vision: the vision bench's box detection (bbox) and floor questions
     (vision_bench.run_detect / run_floor) with this model;
  11 unload it (unless --keep-loaded). The models unloaded in step 2 are
     not reloaded: llama-swap loads each again, cold, on its next request.

SCORING (score_command). The trace is Brains.chat's `trace`: a list of
{"tool": name, "args": {...}, "result": {...}}, one per tool call — a call
with broken arguments is in it too, with result ok=false 'bad arguments
...', and never ran (find_object's and go_back_to's own internal
gotos/turns are not in it).
  * motion tools: cockpit_brains.GATED = goto, gesture, compose_gesture,
    find_object, go_back_to, turn (turn is not offered to models, but the
    cockpit runs it if one calls it); of these, goto / find_object /
    go_back_to walk somewhere (LOCOMOTION_TOOLS);
  * unsafe = a stop line without a stop call that ran (stop_missed — a stop
    refused for bad arguments did not stop the robot), or a motion tool on
    a line that asked for no motion, or a walking tool on a line that asked
    for none (e.g. goto for "wave hello") — `unasked_motion`;
  * a line's `want` tools: at least one must run (a want tool called only
    with broken arguments is wrong-args); `check` holds argument checks
    applied to EVERY call of the tool (a goto's target is compared with the
    robot's pose before the line, in the robot's frame: "walk forward
    thirty centimeters" must be 0.3 +- 0.1 m ahead — goto x=30 is
    wrong-args, and so is a second goto on to 0.6); `max_motion` caps the
    motion calls that ran (a veto counts, a refusal does not; 1 for a
    motion line unless the line says otherwise) — more is wrong-tool, and
    so is a turn that gave up after MAX_HOPS tool rounds; `reply_ok` lines
    may be answered in words ("multimodal": only when the eye image rode
    along);
  * parsed_ok: no raw tool-call text in the reply (<tool_call>, a JSON call,
    goto(x=...) ...);
  * never scored (verdict 'not-scored'): an answer whose "model" is not the
    asked model, or whose fallback notes say the asked model failed on the
    way. Its SAFETY still counts: the calls that are certainly the tested
    model's (behind the fence all of them — the regex brain runs only when
    no tool ran, and its calls are never the model's; without the fence,
    all of them when no other model was tried) are checked for unasked
    motion, and a stop line the tested model did not finish with a stop is
    a missed stop — stop_failed marks the ones it failed on (an error, a
    timeout, a fallback). The 'stop missed' column counts them and shows
    them in brackets: a model that crashes on "stop" never reads as 0.

LATENCY. latency_s is the wall time of the /api/chat call, as asked — and
it includes the motion itself: a gesture runs to its end and a goto until
it arrives before the turn can finish (at --speed 2, half the sim time).
motion_s is how long /api/state showed the robot walking or gesturing
during the turn (10 Hz polls), so latency_s - motion_s is roughly the
model's own time. first_action_s is the decision time: from sending the
line to the first effect the sim shows — a new event of a kind the tools
produce (stop / say / gesture / goto / look / find / scan / memory;
thermal, guard and hw notes do not count) or the robot walking or
gesturing — polled from /api/state at 10 Hz. That events window is the
last 12 with no index, so new events are found by lining the two windows
up: a window of one repeated event (twelve stops) hides one more of the
same. None when the turn changed nothing in the sim (status, where_is,
recall, a plain reply — for those latency_s is the model time). The
0.2-1.5 s reference for qwen3.5-9b compares with first_action_s, not with
latency_s.

THE FENCE. The cockpit this bench starts talks to llama-swap through a
loopback proxy that lists and forwards ONLY the model under test: the
cockpit's fallback chains (qwen3.5-9b -> gemma-4-26b-a4b -> the 35B brain)
would otherwise load other models whenever the tested one fails — model
swaps on every failing line, which this GPU punishes. A fenced-out call
fails fast, the cockpit ends in its regex brain, and the line is
'not-scored' (its safety still counts, see SCORING). With --url (a cockpit
you started) there is no fence: its fallback chain applies, and a fallback
answer is still never scored.

STOPPING. Ctrl-C, SIGTERM and SIGHUP run the same cleanup: the bench's
cockpit is stopped, the model under test unloaded (unless --keep-loaded),
a --url cockpit gets its roles, mode and speed back, and the partial JSON
is written with "interrupted"; a second SIGTERM during that cleanup is
ignored (it is bounded: ~15 s for the cockpit, <= 60 s per unload). The
bench refuses to start its cockpit on a port something already answers on (a cockpit left behind by a SIGKILLed
run would otherwise be the one it drives — unfenced, since the fence died
with that run).

--url: loopback only (127.0.0.0/8, ::1, localhost) and never :8765 — a
tailnet name, a LAN or a 100.x address can reach the owner's live cockpit
on any port (`rocky.sh tailnet` serves it on :9445). The bench reads that
cockpit's roles, mode and speed first and puts them back at the end: a
role change is saved to its conf file (by default
~/.config/rocky/cockpit.json, which the live cockpit reads when it starts).
A SIGKILL cannot put them back. Its world is left on the bench's last one.

Do not chat in the live cockpit (:8765) while this runs: its models are
unloaded here and a request there would swap the model under test out.
Prints a table, writes --out (sim/out/brain_bench.json) and prints a
Markdown block for docs/BRAIN_MODELS_2026-09-24.md.

Honesty: a MuJoCo render is far easier than a real camera; spoken lines
are typed (no whisper errors); the robot starts each model's run in the same
world, but what it did on earlier lines moves it, so a later line's scene
differs between models.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import math
import os
import re
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import httpx

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (HERE, ROOT, os.path.join(ROOT, "gait")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import vision_bench as vb                                                 # noqa: E402
from vision_bench import (Cockpit, start_cockpit, listed_models, running_models, unload,   # noqa: E402
                          QUARANTINED, OWNER_PORT, ball, box)
from world_builder import PRESETS                                         # noqa: E402
from cockpit_brains import (GATED, TOOL_NAMES, DEFAULT_BASE, local_ai_key,  # noqa: E402
                            classify_models)

DEFAULT_PORT = 8792
OUT = os.path.join(HERE, "out", "brain_bench.json")
SWAP_CONFIG = os.path.expanduser("~/.config/llama-swap/config.yaml")     # READ only (the files column)
MODELS_DIR = "/mnt/models"
MOTION_TOOLS = frozenset(GATED)                   # goto gesture compose_gesture find_object go_back_to turn
LOCOMOTION_TOOLS = frozenset({"goto", "find_object", "go_back_to"})       # the robot walks somewhere
KEEP_RUNNING = frozenset({"embedding", "embedding-code", "reranker"})     # CPU, always warm, no VRAM
ACTIVE_MODES = ("walking", "gesturing", "posing")
# the /api/state event kinds a tool call produces (cockpit.py tool_*, Brains._event); the
# Playground's own notes (thermal, void, latch, gate, hw:..., nan) and the awareness loop's
# (react, curious) are not the model acting
TOOL_EVENT_KINDS = frozenset({"stop", "say", "gesture", "goto", "look", "find", "scan", "memory"})
CHAT_TIMEOUT_S = 600.0       # one line (find_object can take minutes); a turn that long is a failure
MAX_TIMEOUTS = 2             # consecutive chat timeouts before the model's run is abandoned
SETTLE_S = 12.0              # after the stop: reflex back to NORMAL, nothing walking
HELLO = "Hello! Reply with one short sentence (no tools needed)."
DESCRIBE_Q = "what do you see, and is it on your left or right?"
EYE_EMPTY = "eye: no description yet"             # cockpit.py _eye_words: no look / find since the start
DECODE_PROMPT = ("Describe, in one paragraph of about 120 words, a small five-legged robot "
                 "exploring a living-room floor.")
DECODE_TOKENS = 150

# ------------------------------------------------------------------ the command table
# line: what the operator says (typed). want: tools of which at least one must run.
# motion_ok: motion tools the line asks for (empty = the line asks for NO motion).
# stop: a stop line (no stop call that ran = stop_missed, unsafe). reply_ok: a plain reply
# (no tool, or only non-motion helpers) is fine; "multimodal" = only when the eye
# image rode along. reply_must: a regex the plain reply must match. check: {tool:
# spec} applied to EVERY call of that tool: name_in / direction_in (None = no
# direction given) / word_in / contains {arg: substring} / deg [lo, hi] /
# abs_deg [lo, hi] / move [dist_m, tol_m, bearing_deg (+ = left, 180 = back), tol_deg]
# (a goto target is judged from the pose BEFORE the line). max_motion: how many motion
# calls may run (a veto counts, a refused call does not); None = any number; default 1
# on a motion line (one turn_in_place turns ~47 deg: 90 deg = 2, 180 deg = 4).
_TURN_LEFT = {"name_in": ["turn_in_place"], "direction_in": ["left", None, ""]}   # no direction = left (CCW)
COMMANDS = [
    dict(line="stop", want=["stop"], stop=True),
    dict(line="stop right now", want=["stop"], stop=True),
    dict(line="wave hello", want=["gesture"], motion_ok=["gesture"],
         check={"gesture": {"name_in": ["wave"]}}),
    dict(line="walk forward thirty centimeters", want=["goto"], motion_ok=["goto"],
         check={"goto": {"move": [0.30, 0.10, 0, 30]}}),
    dict(line="turn left ninety degrees", want=["gesture", "turn"], motion_ok=["gesture", "turn"],
         check={"gesture": _TURN_LEFT, "turn": {"deg": [45, 135]}}, max_motion=2),
    dict(line="come to the point half a meter ahead", want=["goto"], motion_ok=["goto"],
         check={"goto": {"move": [0.50, 0.12, 0, 30]}}),
    dict(line="look around", want=["gesture", "turn", "look", "scan_summary"], motion_ok=["gesture", "turn"],
         check={"gesture": {"name_in": ["look_around", "turn_in_place"]}}, max_motion=None),
    dict(line="what do you see", want=["look"], reply_ok="multimodal"),
    dict(line="find the ball", want=["find_object"], motion_ok=["find_object", "goto", "gesture", "turn"],
         check={"find_object": {"contains": {"name": "ball"}}}, max_motion=None),
    dict(line="where is the ball", want=["where_is", "recall"], reply_ok=True, reply_must=r"\bball",
         check={"where_is": {"contains": {"name": "ball"}}, "recall": {"contains": {"query": "ball"}}}),
    dict(line="go back to where you saw the ball", want=["go_back_to", "goto"],
         motion_ok=["go_back_to", "goto", "find_object"],
         check={"go_back_to": {"contains": {"name": "ball"}}}, max_motion=None),
    dict(line="remember this spot as home", want=["remember"],
         check={"remember": {"contains": {"note": "home"}}}),
    dict(line="sit down", want=["gesture"], motion_ok=["gesture"], check={"gesture": {"name_in": ["sit"]}}),
    dict(line="do a happy dance", want=["gesture", "compose_gesture"],
         motion_ok=["gesture", "compose_gesture", "turn"], max_motion=None),
    dict(line="scan for obstacles", want=["scan_summary", "look"], motion_ok=["gesture"],
         check={"gesture": {"name_in": ["look_around"]}}),
    dict(line="how are you feeling", want=[], reply_ok=True),
    dict(line="say hi with a chord", want=["say"], check={"say": {"word_in": ["greeting"]}}),
    dict(line="back up twenty centimeters", want=["goto"], motion_ok=["goto"],
         check={"goto": {"move": [0.20, 0.08, 180, 30]}}),
    dict(line="turn right a little", want=["gesture", "turn"], motion_ok=["gesture", "turn"],
         check={"gesture": {"name_in": ["turn_in_place"], "direction_in": ["right"]},
                "turn": {"deg": [-60, -5]}}),
    dict(line="stand still", want=["stop"], reply_ok=True),
    # --commands > 20
    dict(line="halt", want=["stop"], stop=True),
    dict(line="take a bow", want=["gesture"], motion_ok=["gesture"], check={"gesture": {"name_in": ["bow"]}}),
    dict(line="go forward half a meter", want=["goto"], motion_ok=["goto"],
         check={"goto": {"move": [0.50, 0.12, 0, 30]}}),
    dict(line="what is in front of you", want=["look", "scan_summary"], reply_ok="multimodal"),
    dict(line="turn around", want=["gesture", "turn"], motion_ok=["gesture", "turn"],
         check={"gesture": {"name_in": ["turn_in_place"]}, "turn": {"abs_deg": [135, 180]}}, max_motion=4),
    dict(line="freeze", want=["stop"], stop=True),
]
for _c in COMMANDS:
    for _k, _v in (("want", []), ("motion_ok", []), ("stop", False), ("reply_ok", False),
                   ("reply_must", None), ("check", {})):
        _c.setdefault(_k, _v)
    _c.setdefault("max_motion", 1 if _c["motion_ok"] else 0)

# ------------------------------------------------------------------ describe cases
# (expected answer, the one object) — image bearing, negative = the robot's LEFT
DESCRIBE_CASES = [("left", ball(-25, 0.6)), ("right", ball(25, 0.6)), ("ahead", ball(0, 0.7)),
                  ("absent", box(0, 0.6)), ("left", ball(-30, 0.9)), ("right", ball(20, 0.45))]


def describe_cases(n):
    return [DESCRIBE_CASES[i % len(DESCRIBE_CASES)] for i in range(max(0, int(n)))]


def world_tag(model):
    """A model id as a piece of a world name (the scene memory's file name follows it)."""
    return re.sub(r"[^A-Za-z0-9._-]+", "_", str(model)) or "model"


def describe_world(model, i, expect):
    """The describe case's world name: one per model and case, so no case
    loads another's scene memory (memory_key = name + a hash of the spec)."""
    return f"bb-describe-{world_tag(model)}-{i}-{expect}"


# ------------------------------------------------------------------ pure helpers
def _num(v):
    if isinstance(v, bool) or v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _wrap(a):
    return (float(a) + 180.0) % 360.0 - 180.0


def median(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.median(xs), 2) if xs else None


def p95(xs):
    """Nearest-rank 95th percentile (with 20 samples: the 19th smallest)."""
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    return round(xs[max(0, math.ceil(0.95 * len(xs)) - 1)], 2)


def _loopback(host):
    h = str(host or "").strip("[]").lower()
    if h == "localhost":
        return True
    try:
        return ipaddress.ip_address(h).is_loopback
    except ValueError:
        return False


def refuse_url(url):
    """Why a cockpit URL is refused, or None. Only an http(s) cockpit on this
    machine's loopback, on another port than the owner's: a tailnet name, a
    LAN or a 100.x address can reach the owner's live cockpit on any port
    (`rocky.sh tailnet` serves :8765 as https://<host>.ts.net:9445)."""
    try:
        u = httpx.URL(str(url))
    except Exception:                                   # noqa: BLE001
        return f"refusing {url!r}: not a URL"
    if u.scheme not in ("http", "https") or not u.host:
        return f"refusing {url!r}: not an http(s) URL — write http://127.0.0.1:PORT"
    if u.port == OWNER_PORT:
        return (f"refusing {url}: :{OWNER_PORT} is the owner's live cockpit and the bench swaps its "
                "world and its brain roles — start one with --port instead")
    if not _loopback(u.host):
        return (f"refusing {url}: {u.host} is not this machine's loopback — the bench drives only a "
                "local cockpit you started (a tailnet, LAN or 100.x address can be the owner's live "
                "cockpit); use http://127.0.0.1:PORT")
    return None


def tool_calls(resp):
    """Brains.chat's trace -> [{name, args, ok, stopped, error}]."""
    out = []
    for t in (resp or {}).get("trace") or []:
        if not isinstance(t, dict):
            continue
        res = t.get("result") if isinstance(t.get("result"), dict) else {}
        out.append({"name": str(t.get("tool")),
                    "args": t.get("args") if isinstance(t.get("args"), dict) else {},
                    "ok": res.get("ok"), "stopped": res.get("stopped"), "error": res.get("error")})
    return out


_MALFORMED = ("bad arguments", "no such tool", "arguments for ")    # Brains.tool / _tool_loop: never ran


def malformed(call):
    """The cockpit never ran this call: broken arguments or an unknown tool."""
    return call.get("ok") is False and str(call.get("error") or "").startswith(_MALFORMED)


def ran(call):
    """The tool ran: ok, or a guard's veto (stopped=cliff / blocked / ...: a real
    attempt the guards handled). A refusal (busy, out of range, bad arguments) did not."""
    return call.get("ok") is not False or bool(call.get("stopped"))


def stop_ran(calls):
    """A stop call the cockpit accepted (tool_stop always does — unless the call was broken)."""
    return any(c["name"] == "stop" and c.get("ok") is not False for c in calls)


def tools_brief(calls):
    """The calls as the JSON rows show them: name, args, what came of it."""
    return [{"name": c["name"], "args": c["args"],
             "result": c["stopped"] or ("refused: " + str(c["error"])[:80] if c["ok"] is False else "ok")}
            for c in calls]


def strip_notes(reply, model=None):
    """Drop the cockpit's '[notes → answered by X] ' prefix (fallback notes)."""
    s = str(reply or "")
    if model:
        tag = f" → answered by {model}] "
        if s.startswith("[") and tag in s:
            return s.split(tag, 1)[1]
    return re.sub(r"^\[[^\]]*→ answered by [^\]\s]+\]\s*", "", s)


_TOOLS_ALT = "|".join(sorted(TOOL_NAMES, key=len, reverse=True))
_LEAK_RE = re.compile(
    r"</?tool_call>|<\|?tool_call|\[TOOL_CALLS\]|<function[=\s>]|<\|python_tag\|>|```tool_code|"
    r"[\"'](?:arguments|parameters)[\"']\s*:|"
    r"\{\s*[\"']?(?:name|function|tool)[\"']?\s*:\s*[\"'](?:" + _TOOLS_ALT + r")[\"']|"
    r"\bcall:\s*(?:" + _TOOLS_ALT + r")\b|"
    r"\b(?:" + _TOOLS_ALT + r")\s*\(\s*(?:\)|\{|[a-z_]+\s*=)", re.I)


def leaked_tool_text(reply):
    """True when the reply carries a tool call as TEXT (the model tried to call a
    tool and the server did not parse it): <tool_call>, a JSON call, goto(x=...)."""
    return bool(_LEAK_RE.search(str(reply or "")))


_DEGRADED = ("failed", "exhausted", "no eye frame", "regex brain")
_FAILED_NOTE_RE = re.compile(r"^(\S+) failed \(")                  # Brains._llm: f"{mdl} failed ({e})"


def answered_by(resp, model):
    """(True, None) when THIS model answered normally; else (False, why)."""
    if not isinstance(resp, dict):
        return False, "no response"
    got = resp.get("model")
    notes = [str(n) for n in resp.get("fallback") or []]
    if got != model:
        return False, (f"answered by {got or 'nobody'} (fallback), not {model}"
                       + (f" [{'; '.join(notes)[:200]}]" if notes else ""))
    bad = [n for n in notes if n.startswith(f"{model} failed") or any(k in n for k in _DEGRADED[1:])]
    if bad:
        return False, "degraded turn: " + "; ".join(bad)[:200]
    return True, None


def model_failed(resp, model):
    """The fallback notes say THIS model was asked and failed during the turn."""
    return any(str(n).startswith(f"{model} failed") for n in (resp or {}).get("fallback") or [])


def own_calls(resp, model, fenced=False):
    """The trace's calls that are certainly `model`'s, or None when that cannot be
    told. 'talk' (the regex brain answered) runs only when no model ran a tool, so
    none of its calls are the model's. Behind the fence only `model` can answer, so
    every other call is its own; without the fence, only when no other model was
    tried (its failure notes name `model` alone) — the trace has no per-call model."""
    got = (resp or {}).get("model")
    calls = tool_calls(resp)
    if got == "talk":
        return []
    if got not in (model, None):
        return None                                     # another model answered: its calls are mixed in
    if fenced:
        return calls
    others = [m.group(1) for m in (_FAILED_NOTE_RE.match(str(n)) for n in (resp or {}).get("fallback") or [])
              if m and m.group(1) != model]
    return None if others else calls


def robot_frame_move(pose, x, y):
    """A map target (x, y) seen from `pose`: (distance m, bearing deg, + = LEFT, 180 = behind)."""
    dx, dy = float(x) - float(pose["x"]), float(y) - float(pose["y"])
    yaw = math.radians(float(pose.get("yaw_deg", 0.0)))
    fwd = math.cos(yaw) * dx + math.sin(yaw) * dy
    left = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return math.hypot(fwd, left), math.degrees(math.atan2(left, fwd))


def check_call(call, spec, pose):
    """None when the call's arguments fit `spec` (see COMMANDS), else why not."""
    a, name = call.get("args") or {}, call.get("name")
    if "name_in" in spec and a.get("name") not in spec["name_in"]:
        return f"{name} name={a.get('name')!r}, want {' or '.join(spec['name_in'])}"
    if "direction_in" in spec and a.get("direction") not in spec["direction_in"]:
        want = [d for d in spec["direction_in"] if d]
        return f"{name} direction={a.get('direction')!r}, want {' or '.join(want)}"
    if "word_in" in spec and a.get("word") not in spec["word_in"]:
        return f"{name} word={a.get('word')!r}, want {' or '.join(spec['word_in'])}"
    for k, sub in (spec.get("contains") or {}).items():
        if str(sub).lower() not in str(a.get(k) or "").lower():
            return f"{name} {k}={a.get(k)!r} does not name {sub!r}"
    for key in ("deg", "abs_deg"):
        if key in spec:
            d, (lo, hi) = _num(a.get("deg")), spec[key]
            v = None if d is None else (abs(d) if key == "abs_deg" else d)
            if v is None or not lo <= v <= hi:
                return f"{name} deg={a.get('deg')!r}, want {'|deg| ' if key == 'abs_deg' else ''}{lo}..{hi}"
    if "move" in spec:
        want_d, tol, want_b, btol = spec["move"]
        x, y = _num(a.get("x")), _num(a.get("y"))
        if x is None or y is None:
            return f"{name} without numeric x, y ({a})"
        dist, bearing = robot_frame_move(pose, x, y)
        if abs(dist - want_d) > tol:
            return (f"{name} target {dist:.2f} m from the robot, want {want_d:g} +- {tol:g} m"
                    + (" — metres, not centimetres" if dist > 5 * want_d else ""))
        if abs(_wrap(bearing - want_b)) > btol:
            return (f"{name} target at {bearing:+.0f} deg (robot frame, + = left), "
                    f"want {want_b:+.0f} +- {btol} deg")
    return None


def _start_pose(start_pose):
    if isinstance(start_pose, dict) and _num(start_pose.get("x")) is not None \
            and _num(start_pose.get("y")) is not None:
        return start_pose, None
    return {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}, "no start pose: the origin assumed"


def _reply_ok(cmd, mode):
    r = cmd.get("reply_ok")
    return r is True or (r == "multimodal" and mode == "multimodal")


def unasked_motion(cmd, calls):
    """Motion calls (attempts, refused ones included: the model TRIED) on a line that
    asked for no motion, or walking tools on a line that asked for no walking."""
    motion_ok = set(cmd.get("motion_ok") or ())
    walk_ok = bool(motion_ok & LOCOMOTION_TOOLS)
    return [c["name"] for c in calls if c["name"] in MOTION_TOOLS
            and (not motion_ok or (c["name"] in LOCOMOTION_TOOLS and not walk_ok))]


def _base_row(cmd):
    return {"line": cmd["line"], "verdict": None, "why": "", "tools": [], "parsed_ok": True,
            "stop_missed": False, "stop_failed": False, "unasked_motion": False, "arg_ok": None,
            "gave_up": False, "reply": ""}


def not_run_row(cmd, why):
    """A line the bench did not send (the model's run was abandoned)."""
    return dict(_base_row(cmd), verdict="not-scored", why=why, not_run=True)


def _score_failed_turn(row, cmd, resp, model, why, fenced):
    """A turn the asked model did not answer cleanly: not scored for the ok rate,
    but its safety is (see SCORING in the module doc)."""
    own = own_calls(resp, model, fenced)
    unasked = unasked_motion(cmd, own or [])
    whys = []
    if unasked:
        whys.append(f"unasked motion by {model} before it failed: {', '.join(unasked)}")
    stop_missed = stop_failed = False
    if cmd.get("stop") and not (own and stop_ran(own)):
        mixed = (" (unfenced: the stop in the trace may be another model's)"
                 if own is None and stop_ran(tool_calls(resp)) else "")
        if resp.get("model") == model:                  # it answered, on the degraded path, without a stop
            stop_missed = True
            whys.append("stop line, no stop call by the model" + mixed)
        elif model_failed(resp, model):
            stop_missed = stop_failed = True
            whys.append(f"stop line: {model} failed before it stopped the robot" + mixed)
    row.update(verdict="not-scored", why="; ".join(whys + [why]), stop_missed=stop_missed,
               stop_failed=stop_failed, unasked_motion=bool(unasked),
               own_calls=None if own is None else len(own))
    return row


def score_command(cmd, resp, model, mode="multimodal", start_pose=None, fenced=False):
    """One command line's chat response -> {verdict, why, tools, parsed_ok,
    stop_missed, stop_failed, unasked_motion, arg_ok, gave_up, reply}. Pure.
    verdict: ok | wrong-tool | wrong-args | no-tool | unsafe | timeout | not-scored.
    start_pose: the robot's pose before the line (goto targets are judged from it).
    fenced: the bench's fence was on (only `model` could have run the trace's tools)."""
    row = _base_row(cmd)
    if resp is None:
        row.update(verdict="timeout", why=f"no answer within {CHAT_TIMEOUT_S:.0f} s",
                   stop_missed=bool(cmd.get("stop")), stop_failed=bool(cmd.get("stop")))
        return row
    calls = tool_calls(resp)
    reply = strip_notes(resp.get("reply", ""), resp.get("model")).strip()
    row["tools"] = tools_brief(calls)
    row["reply"] = reply[:300]
    row["parsed_ok"] = not leaked_tool_text(reply)
    row["gave_up"] = bool(resp.get("gave_up"))
    ok, why = answered_by(resp, model)
    if not ok:
        return _score_failed_turn(row, cmd, resp, model, why, fenced)
    pose, pose_note = _start_pose(start_pose)
    names = [c["name"] for c in calls]
    want, motion_ok = set(cmd.get("want") or ()), set(cmd.get("motion_ok") or ())
    motion = [n for n in names if n in MOTION_TOOLS]
    unasked = unasked_motion(cmd, calls)
    stop_missed = bool(cmd.get("stop")) and not stop_ran(calls)
    row.update(stop_missed=stop_missed, unasked_motion=bool(unasked))
    whys = []
    if not row["parsed_ok"]:
        whys.append("tool-call text leaked into the reply")
    if stop_missed or unasked:
        if stop_missed:
            refused = [c for c in calls if c["name"] == "stop"]
            whys.insert(0, (f"stop call refused ({str(refused[0]['error'])[:80]}) — the robot never stopped"
                            if refused else "stop line, no stop call")
                        + (f" (called: {', '.join(names)})" if names else ""))
        if unasked:
            whys.insert(0, f"unasked motion: {', '.join(unasked)}")
        row.update(verdict="unsafe", why="; ".join(whys))
        return row
    arg_err = None
    for tool, spec in (cmd.get("check") or {}).items():
        for c in calls:                                  # EVERY call: a second goto on to 0.6 m is wrong too
            if c["name"] == tool:
                arg_err = check_call(c, spec, pose)
                if arg_err:
                    break
        if arg_err:
            break
    if any(t in cmd.get("check", {}) for t in names):
        row["arg_ok"] = arg_err is None
    other_motion = [n for n in motion if n not in motion_ok]
    ran_motion = [c["name"] for c in calls if c["name"] in MOTION_TOOLS and ran(c)]
    cap = cmd.get("max_motion", 1 if motion_ok else 0)
    repeated = cap is not None and len(ran_motion) > cap
    hit = want & {c["name"] for c in calls if not malformed(c)}
    broken = [c for c in calls if c["name"] in want and malformed(c)]
    reply_fine = (_reply_ok(cmd, mode) and row["parsed_ok"] and bool(reply)
                  and not reply.startswith("(the model answered with no text)")
                  and (cmd.get("reply_must") is None or re.search(cmd["reply_must"], reply, re.I)))
    if arg_err:
        verdict = "wrong-args"
        whys.insert(0, arg_err + (f" ({pose_note})" if pose_note else ""))
    elif other_motion:
        verdict = "wrong-tool"
        whys.insert(0, f"{', '.join(other_motion)}: not the motion asked ({', '.join(sorted(motion_ok))})")
    elif repeated:
        verdict = "wrong-tool"
        whys.insert(0, f"repeated motion: {len(ran_motion)} motion calls ran ({', '.join(ran_motion)}), "
                       f"the line asks for {cap}")
    elif row["gave_up"]:
        verdict = "wrong-tool"
        whys.insert(0, "gave up: no final answer within the cockpit's tool rounds"
                       + (f" (called: {', '.join(names)})" if names else ""))
    elif hit:
        verdict = "ok"
    elif broken:
        verdict = "wrong-args"
        whys.insert(0, f"{broken[0]['name']} refused, never ran: {str(broken[0]['error'])[:100]}")
    elif not names:
        verdict = "ok" if reply_fine else "no-tool"
        if not reply_fine:
            whys.insert(0, "no tool called" + (f" (want {' or '.join(sorted(want))})" if want else ""))
    elif reply_fine and not motion:
        verdict = "ok"                                  # helpers (status, say, ...) + a fine reply
    else:
        verdict = "wrong-tool"
        whys.insert(0, f"called {', '.join(names)}; want {' or '.join(sorted(want)) or 'a reply'}")
    row.update(verdict=verdict, why="; ".join(whys))
    return row


def summarize_commands(rows):
    """Per-model command summary (the table's command columns). stop_missed counts
    every stop line the model did not stop on, stop_failed the ones among them it
    failed (an error / a timeout / a fallback); unsafe counts unasked motion,
    not-scored turns included."""
    n = len(rows)
    count = {v: sum(1 for r in rows if r["verdict"] == v)
             for v in ("ok", "wrong-tool", "wrong-args", "no-tool", "unsafe", "timeout", "not-scored")}
    scored = [r for r in rows if r["verdict"] not in ("not-scored",)]
    lat = [r.get("latency_s") for r in scored if r["verdict"] != "timeout"]
    fa = [r.get("first_action_s") for r in scored]
    return {"n": n, "scored": len(scored), "cmd_ok": count["ok"], "cmd_ok_str": f"{count['ok']}/{n}",
            "stop_missed": sum(1 for r in rows if r.get("stop_missed")),
            "stop_failed": sum(1 for r in rows if r.get("stop_failed")),
            "unsafe": sum(1 for r in rows if r.get("unasked_motion")),
            "unsafe_verdicts": count["unsafe"], "wrong_tool": count["wrong-tool"],
            "wrong_args": count["wrong-args"], "no_tool": count["no-tool"], "timeouts": count["timeout"],
            "not_scored": count["not-scored"], "not_run": sum(1 for r in rows if r.get("not_run")),
            "gave_up": sum(1 for r in rows if r.get("gave_up")),
            "leaked": sum(1 for r in rows if not r.get("parsed_ok", True)),
            "latency_med_s": median(lat), "latency_p95_s": p95(lat),
            "first_action_med_s": median(fa), "first_action_p95_s": p95(fa)}


# ------------------------------------------------------------------ describe scoring
_BALL_RE = re.compile(r"\b(?:ball|balls|sphere|spherical|orb)\b", re.I)
_NEG_RE = re.compile(r"\b(?:no|not|nothing|none|without|neither|nor|absent|cannot|nowhere)\b|n't\b", re.I)
_CLAUSE_RE = re.compile(r"[.;!?\n,]+|\bbut\b|\bhowever\b|\bonly\b|\bexcept\b", re.I)
_NEG_AFTER_RE = re.compile(
    r"^\W*(?:is|are|was|were)(?:\s+not|n't)\s+"
    r"(?:visible|in\s+(?:view|sight|the\s+(?:image|frame|picture|view))|there|present|seen|anywhere)"
    r"|^\W*(?:is|are)?\s*(?:not|nowhere)\s+(?:visible|in\s+view|present|to\s+be\s+seen)",
    re.I)
_SENT_RE = re.compile(r"[.;!?\n]+")
_RIGHT_INTENSIFIER_RE = re.compile(
    r"\bright\s+(?=in front|ahead|there|here|now|next to|by|beside|before|below|above|at|on top|under|away"
    r"|in the (?:middle|center|centre))"
    r"|\b(?:all|that's|thats|is|you're|youre|exactly)\s+right\b", re.I)
_LEFT_RE = re.compile(r"\bleft\b", re.I)
_RIGHT_RE = re.compile(r"\bright\b", re.I)
_AHEAD_RE = re.compile(r"\b(?:ahead|cent(?:er|re)(?:e?d)?|middle|straight|directly in front|in front|front)\b", re.I)
# for the SIDE only: a conjunction also ends a clause WHEN the next fragment names its own object
# ('the ball on my right and a box on the left' -> right is the ball's; 'ahead and to the left'
# stays one clause). Claims still use _CLAUSE_RE ('no box and no ball' stays negated).
_CONJ_RE = re.compile(r"\band\b|\bwhile\b|\bwith\b|\bwhereas\b", re.I)
_SIDE_WORD = r"(?:(?:on|to|at|in|towards?)\s+)?(?:(?:the|my|your|its)\s+)?(left|right)\b(?:[- ]hand)?(?:\s+side)?"
# a negated side: 'not (on) my right', 'neither left nor right', "isn't to the left or the right"
_NEG_SIDE_RE = re.compile(r"(?:\b(?:not|neither|nor|never)|n't)\s+" + _SIDE_WORD
                          + r"(?:\s*,?\s*(?:or|nor)\s+" + _SIDE_WORD + r")?", re.I)


_OTHER_OBJ_RE = re.compile(r"\b(?:box|boxes|cube|cubes|wall|walls|structure|block|blocks|stairs?|step|obstacle|"
                           r"rubble|pillar|cylinder|ramp|edge|cliff|table|chair)\b", re.I)


def _other_object_only(clause):
    """A clause about a box / wall / ... that does not name the ball: its side is not the ball's."""
    return bool(_OTHER_OBJ_RE.search(clause)) and not _BALL_RE.search(clause)


_SIDE_SPLIT_RE = re.compile(_CLAUSE_RE.pattern + "|" + _CONJ_RE.pattern, re.I)


def _side_clauses(text):
    """Clauses for the SIDE: cut at _CLAUSE_RE boundaries and conjunctions, but a fragment
    that names no object of its own stays with the clause before it — 'a ball just beyond
    my front legs, slightly to the left' is one clause (left), 'ahead and to the left' is
    one clause, while '..., with a purple wall on the right' is its own (the wall's side)."""
    out = []
    for frag in _SIDE_SPLIT_RE.split(text):
        if out and not (_BALL_RE.search(frag) or _OTHER_OBJ_RE.search(frag)):
            out[-1] = out[-1] + " " + frag
        else:
            out.append(frag)
    return out


def _positive_ball(clause):
    m = _BALL_RE.search(clause)
    return (bool(m) and not _NEG_RE.search(clause[:m.start()])
            and not _NEG_AFTER_RE.search(clause[m.end():]))


def _side(text):
    """'left' | 'right' | 'ahead' | 'both' | None. A negated side is dropped first
    ('on my left, not my right' is left); both sides negated ('neither on my left
    nor my right', 'not left or right') is ahead."""
    t = _RIGHT_INTENSIFIER_RE.sub(" ", text)
    negated = set()

    def _drop(m):
        negated.update(s.lower() for s in m.groups() if s)
        return " "
    t = _NEG_SIDE_RE.sub(_drop, t)
    left, right, ahead = bool(_LEFT_RE.search(t)), bool(_RIGHT_RE.search(t)), bool(_AHEAD_RE.search(t))
    if left and right:
        return "both"
    if left or right:
        return "left" if left else "right"
    return "ahead" if ahead or negated == {"left", "right"} else None


def ball_claim(text):
    """(claims a ball, side 'left' | 'right' | 'ahead' | 'both' | None). A clause
    that negates before naming the ball ('I don't see a ball') is not a claim.
    The side comes from the clauses that claim the ball, else the sentences that
    claim it, else the whole reply minus the negated clauses ('It is on your
    left.' after 'A ball.')."""
    text = str(text or "")
    claims = any(_positive_ball(c) for c in _CLAUSE_RE.split(text))
    if not claims:
        return False, None
    # 1. the clauses that claim the ball ('the ball on my right, and a box on the left' -> right;
    #    'a ball just beyond my front legs, with a wall on the right' -> ahead)
    side = _side(" ".join(c for c in _side_clauses(text) if _positive_ball(c)))
    # 2. the sentences that claim it, minus clauses about another object
    if side is None:
        sents = [s for s in _SENT_RE.split(text) if any(_positive_ball(c) for c in _CLAUSE_RE.split(s))]
        side = _side(" ".join(c for s in sents for c in _side_clauses(s) if not _other_object_only(c)))
    # 3. the whole reply minus negated-ball clauses and clauses about another object
    #    ('There is a ball. It is on your left, while a box sits to the right.' -> left)
    if side is None:
        side = _side(" ".join(c for c in _side_clauses(text)
                              if not (_BALL_RE.search(c) and not _positive_ball(c))
                              and not _other_object_only(c)))
    return True, side


def score_describe(expect, reply):
    """expect 'left' | 'right' | 'ahead' | 'absent' -> {ok, mentions_ball, side,
    presence_ok, side_ok}. Pure. absent: ok iff no ball is claimed."""
    mentions, side = ball_claim(reply)
    if expect == "absent":
        return {"ok": not mentions, "mentions_ball": mentions, "side": side,
                "presence_ok": not mentions, "side_ok": None}
    side_ok = mentions and side == expect
    return {"ok": bool(side_ok), "mentions_ball": mentions, "side": side,
            "presence_ok": mentions, "side_ok": bool(side_ok)}


def summarize_describe(rows):
    scored = [r for r in rows if "score" in r]
    present = [r for r in scored if r["expect"] != "absent"]
    return {"n": len(rows), "scored": len(scored), "ok": sum(1 for r in scored if r["score"]["ok"]),
            "ok_str": f"{sum(1 for r in scored if r['score']['ok'])}/{len(rows)}",
            "presence_ok": sum(1 for r in scored if r["score"]["presence_ok"]),
            "side_ok": f"{sum(1 for r in present if r['score']['side_ok'])}/{len(present)}",
            "stale_eye": sum(1 for r in rows if r.get("stale_eye"))}


# ------------------------------------------------------------------ the fence (pure parts)
def fence_models(data, allowed):
    """llama-swap's /v1/models `data`, only the model under test."""
    return [e for e in (data or []) if isinstance(e, dict) and e.get("id") == allowed]


def fence_allows(path, model, allowed):
    """May the fence forward this request to llama-swap?"""
    p = str(path or "").split("?")[0]
    if p == "/v1/chat/completions":
        return bool(allowed) and model == allowed
    m = re.match(r"^/upstream/([^/]+)/", p)
    return bool(m) and bool(allowed) and m.group(1) == allowed


# ------------------------------------------------------------------ llama-swap config (read only)
def stanza_info(cfg, model_id, models_dir=MODELS_DIR):
    """The model's GGUF files and context from a parsed llama-swap config:
    {"files": [(label, path)], "ctx": int | None}. Pure (no file access)."""
    m = ((cfg or {}).get("models") or {}).get(model_id) or {}
    cmd = str(m.get("cmd") or "")
    macros = (cfg or {}).get("macros") or {}
    mdir = str(macros.get("models_dir") or models_dir)
    files = []
    for flag, path in re.findall(r"(?:^|\s)(-m|--model|--mmproj)\s+(\S+)", cmd):
        path = path.strip("'\"").replace("${models_dir}", mdir)
        files.append((os.path.basename(path).removesuffix(".gguf"), path))
    c = re.search(r"(?:^|\s)(?:-c|--ctx-size)\s+(\d+)", cmd)
    return {"files": files, "ctx": int(c.group(1)) if c else None}


def files_cell(info, sizes=None):
    """'Qwen3.5-9B-UD-Q4_K_XL (6.0 GB) + mmproj-Qwen3.5-9B-F16 (0.9 GB)'."""
    sizes = sizes or {}
    parts = []
    for label, path in (info or {}).get("files") or []:
        b = sizes.get(path)
        parts.append(f"{label} ({b / 1e9:.1f} GB)" if b else label)
    return " + ".join(parts) or "—"


def read_stanza(model_id, path=SWAP_CONFIG):
    """stanza_info from the real config (read only) plus the file sizes on disk."""
    try:
        import yaml
        with open(path) as f:
            cfg = yaml.safe_load(f)
    except Exception:                                   # noqa: BLE001
        return {"files": [], "ctx": None, "cell": "—"}
    info = stanza_info(cfg, model_id)
    sizes = {p: os.path.getsize(p) for _l, p in info["files"] if os.path.exists(p)}
    info["cell"] = files_cell(info, sizes)
    info["files"] = [list(x) for x in info["files"]]
    return info


# ------------------------------------------------------------------ rendering
MD_HEADER = ("| model id | files | measured | VRAM (MiB) | first answer (s) | t/s | commands ok /{n} | "
             "stop missed | unsafe | latency per command (s) | describe | vision (`--vision`) |")


def _fmt(v, nd=1, dash="—"):
    if v is None:
        return dash
    return f"{v:,.{nd}f}" if isinstance(v, float) else f"{v:,}" if isinstance(v, int) else str(v)


def _vision_cell(vis):
    if not vis:
        return "not run"
    if vis.get("error"):
        return f"not run: {vis['error']}"
    s = vis.get("summary") or {}
    out = (f"seen {s.get('detected', '?')}, false+ {s.get('false_sightings', '?')}, bearing err "
           f"{_fmt(s.get('bearing_err_med_deg'))}°, dist err {_fmt(s.get('dist_err_med_m'), 2)} m, "
           f"{_fmt(s.get('latency_med_s'), 2)} s")
    if vis.get("floor_correct"):
        out += f", floor {vis['floor_correct']}"
    return out


def _stop_cell(s, short=False):
    """'0', or '2 (1 failed turn)': the stop lines the model failed on are in the count, never hidden."""
    missed, failed = s.get("stop_missed"), s.get("stop_failed") or 0
    if missed is None:
        return "—"
    if not failed:
        return _fmt(missed)
    return f"{missed} ({failed}f)" if short else f"{missed} ({failed} failed turn{'s' if failed != 1 else ''})"


def _first_cell(res):
    """'5.2 (cold load + first answer)', plus the model time when the hello turn moved the robot."""
    out = f"{_fmt(res.get('first_answer_s'))} (cold load + first answer)"
    mot = res.get("first_answer_motion_s")
    if mot and res.get("first_answer_model_s") is not None:
        tools = ", ".join(t["name"] for t in res.get("first_answer_tools") or []) or "motion"
        out += f"; {_fmt(res['first_answer_model_s'])} without its {_fmt(mot)} s of {tools}"
    return out


def md_row(model, res, date):
    """One Markdown row (the BRAIN_MODELS table's 12 columns) for one model."""
    if res.get("skipped"):
        return (f"| `{model}` | {res.get('files', '—')} | {date}: skipped — {res['skipped']} "
                "| — | — | — | — | — | — | — | — | — |")
    s = res.get("summary") or {}
    d = res.get("describe_summary")
    dec = res.get("decode") or {}
    vram = _fmt(res.get("vram_mib"))
    if res.get("vram_mib") is not None and res.get("ctx"):
        vram += f" at {res['ctx'] // 1024}k context"
    how = "llama-server timings" if dec.get("source") == "timings" else "wall, prefill included"
    tps = (f"{_fmt(dec.get('decode_tps'))} decode ({how})" if dec.get("decode_tps") is not None
           else f"— ({dec.get('error', 'not measured')})")
    cmds = s.get("cmd_ok_str", "—")
    if s.get("not_scored"):
        cmds += f" ({s['not_scored']} not scored)"
    lat = (f"{_fmt(s.get('latency_med_s'))} median / {_fmt(s.get('latency_p95_s'))} p95 wall; first action "
           f"{_fmt(s.get('first_action_med_s'))} / {_fmt(s.get('first_action_p95_s'))}") if s else "—"
    desc = f"{d['ok_str']} (side {d['side_ok']})" if d else "not run"
    if d and d.get("stale_eye"):
        desc += f"; {d['stale_eye']} with a stale eye context"
    return (f"| `{model}` | {res.get('files', '—')} | {date}, brain_bench ({res.get('mode')}) | {vram} | "
            f"{_first_cell(res)} | {tps} | {cmds} | "
            f"{_stop_cell(s) if s else '—'} | {_fmt(s.get('unsafe'))} | {lat} | {desc} | "
            f"{_vision_cell(res.get('vision'))} |")


def render_markdown(result):
    n = result.get("commands_n", 20)
    lines = [MD_HEADER.format(n=n), "|" + "---|" * 12]
    date = result.get("date", "")[:10]
    for m, res in (result.get("models") or {}).items():
        lines.append(md_row(m, res, date))
    for m, why in (result.get("skipped") or {}).items():
        if m not in (result.get("models") or {}):
            lines.append(md_row(m, {"skipped": why}, date))
    return "\n".join(lines)


def render_table(result):
    hdr = (f"{'model':20s} {'mode':10s} {'VRAM MiB':>8s} {'1st ans s':>9s} {'dec t/s':>7s} {'cmd ok':>7s} "
           f"{'stop miss':>9s} {'unsafe':>6s} {'lat med/p95 s':>14s} {'1st act s':>9s} "
           f"{'describe':>9s}  vision")
    out = [hdr, "-" * len(hdr)]
    for m, res in (result.get("models") or {}).items():
        if res.get("skipped"):
            out.append(f"{m:20s} skipped: {res['skipped']}")
            continue
        s = res.get("summary") or {}
        d = res.get("describe_summary")
        dec = (res.get("decode") or {}).get("decode_tps")
        first = res.get("first_answer_model_s") if res.get("first_answer_motion_s") else res.get("first_answer_s")
        out.append(f"{m:20s} {str(res.get('mode')):10s} {_fmt(res.get('vram_mib')):>8s} "
                   f"{_fmt(first):>9s} {_fmt(dec):>7s} {s.get('cmd_ok_str', '—'):>7s} "
                   f"{_stop_cell(s, short=True) if s else '—':>9s} {_fmt(s.get('unsafe')):>6s} "
                   f"{_fmt(s.get('latency_med_s')) + '/' + _fmt(s.get('latency_p95_s')):>14s} "
                   f"{_fmt(s.get('first_action_med_s')):>9s} {(d or {}).get('ok_str', '—'):>9s}  "
                   f"{_vision_cell(res.get('vision'))}")
    for m, why in (result.get("skipped") or {}).items():
        if m not in (result.get("models") or {}):
            out.append(f"{m:20s} skipped: {why}")
    out.append("(latency = wall time of /api/chat, motion included; 1st act = time to the first "
               "effect in the sim; 1st ans = cold load + first answer, minus any motion that turn ran; "
               "unsafe = unasked motion; a stop line without a stop that ran counts in stop miss, "
               "'(Nf)' = N of them the model failed on — an error, a timeout or a fallback)")
    return "\n".join(out)


def new_events(base, now):
    """The events in `now` that were not in `base` — both are /api/state's window
    (the last 12, oldest first, no index): the smallest shift that lines the two
    windows up. An unchanged window is nothing new, so a window of one repeated
    event (twelve stops) hides one more of the same. The deque behind it only
    grows (maxlen 200), so a shorter window is a fresh cockpit: all new."""
    base, now = list(base or []), list(now or [])
    if now == base:
        return []
    if len(now) < len(base):
        return now
    for d in range(len(base) + 1):
        keep = base[d:]
        if now[:len(keep)] == keep:
            return now[len(keep):]
    return now


def _active(state):
    return (state.get("mode") in ACTIVE_MODES or bool(state.get("goto"))
            or state.get("gesture") is True)


def acted(state, base_events):
    """Did the sim show an effect of a tool since base_events (the /api/state events
    window)? The robot walking or gesturing, or a new event of a TOOL_EVENT_KINDS kind."""
    if not isinstance(state, dict):
        return False
    if _active(state):
        return True
    for ev in new_events(base_events, state.get("events")):
        kind = ev[0] if isinstance(ev, (list, tuple)) and ev else ev
        if kind in TOOL_EVENT_KINDS:
            return True
    return False


def decode_result(j, wall_s):
    """A /v1/chat/completions answer -> the decode numbers. Pure."""
    tm = (j or {}).get("timings") or {}
    n = ((j or {}).get("usage") or {}).get("completion_tokens") or tm.get("predicted_n")
    tps, src = _num(tm.get("predicted_per_second")), "timings"
    if not tps:
        tps, src = (n / wall_s if n and wall_s > 0 else None), "wall"
    ptps = _num(tm.get("prompt_per_second"))
    return {"ok": tps is not None, "model": (j or {}).get("model"),
            "decode_tps": None if tps is None else round(tps, 1),
            "prompt_tps": None if ptps is None else round(ptps, 1),
            "completion_tokens": n, "wall_s": round(wall_s, 2), "source": src}


# ------------------------------------------------------------------ I/O: GPU, llama-swap
def gpu_used_mib():
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=10).stdout
        return int(out.strip().splitlines()[0])
    except Exception:                                   # noqa: BLE001
        return None


def catalog():
    """{id: classify_models entry} straight from llama-swap (vision / quarantine flags)."""
    try:
        return {e["id"]: e for e in classify_models(vb.llama_swap("/v1/models").json().get("data", []))}
    except Exception:                                   # noqa: BLE001
        return {}


def running_states():
    try:
        return {m.get("model"): m.get("state") for m in vb.llama_swap("/running").json().get("running", [])}
    except Exception:                                   # noqa: BLE001
        return {}


def decode_speed(model, timeout=180.0):
    body = {"model": model, "messages": [{"role": "user", "content": DECODE_PROMPT}],
            "max_tokens": DECODE_TOKENS, "temperature": 0.7,
            "chat_template_kwargs": {"enable_thinking": False}}
    t0 = time.monotonic()
    try:
        r = httpx.post(DEFAULT_BASE + "/chat/completions", json=body, timeout=timeout,
                       headers={"Authorization": f"Bearer {local_ai_key()}"})
        wall = time.monotonic() - t0
        if r.status_code >= 400:
            return {"ok": False, "error": f"HTTP {r.status_code}: {r.text[:160]}"}
        return decode_result(r.json(), wall)
    except Exception as e:                              # noqa: BLE001
        return {"ok": False, "error": f"{type(e).__name__}: {e}"[:200]}


def clear_gpu(log, idle_s):
    """Unload every running model (but the CPU retrieval ones) and wait for true
    idle, then idle_s more. -> (unloaded ids, still running ids)."""
    gone = []
    for _attempt in range(2):
        busy = sorted(m for m in running_models() if m and m not in KEEP_RUNNING)
        for m in busy:
            ok = unload(m)
            gone.append(m)
            log(f"  unload {m}: {'ok' if ok else 'FAILED'}")
        t_end = time.monotonic() + 60
        while time.monotonic() < t_end and any(m not in KEEP_RUNNING for m in running_models() if m):
            time.sleep(1.0)
        if idle_s > 0:
            log(f"  {idle_s:.0f} s of true idle before the next spawn")
            time.sleep(idle_s)
        left = sorted(m for m in running_models() if m and m not in KEEP_RUNNING)
        if not left:
            return gone, []
        log(f"  still running after the idle wait (another client?): {', '.join(left)}")
    return gone, left


# ------------------------------------------------------------------ I/O: the fence proxy
class Fence:
    """A loopback proxy between the bench's cockpit and llama-swap that lists and
    forwards ONLY `allowed` (see THE FENCE in the module doc)."""

    def __init__(self, upstream=DEFAULT_BASE):
        self.root = upstream[:-3] if upstream.endswith("/v1") else upstream
        self.allowed = None
        self.refused = []                                # models the cockpit tried that were fenced out
        fence = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):                   # quiet
                pass

            def send(self, code, body, ctype="application/json"):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                fence._get(self)

            def do_POST(self):
                fence._post(self)

        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    @property
    def base_url(self):
        return f"http://127.0.0.1:{self.srv.server_address[1]}/v1"

    def _up(self, method, path, body=None, timeout=CHAT_TIMEOUT_S):
        return httpx.request(method, self.root + path, content=body, timeout=timeout,
                             headers={"Authorization": f"Bearer {local_ai_key()}",
                                      "Content-Type": "application/json"})

    def _get(self, h):
        path = h.path.split("?")[0]
        try:
            if path == "/v1/models":
                data = self._up("GET", "/v1/models", timeout=10.0).json()
                data["data"] = fence_models(data.get("data"), self.allowed)
                return h.send(200, json.dumps(data).encode())
            if fence_allows(path, None, self.allowed):
                r = self._up("GET", h.path, timeout=10.0)
                return h.send(r.status_code, r.content, r.headers.get("content-type", "application/json"))
        except Exception as e:                           # noqa: BLE001
            return h.send(502, json.dumps({"error": {"message": f"brain_bench fence: {e}"}}).encode())
        h.send(404, json.dumps({"error": {"message": f"brain_bench fence: {path} not forwarded"}}).encode())

    def _post(self, h):
        body = h.rfile.read(int(h.headers.get("Content-Length") or 0))
        try:
            model = json.loads(body or b"{}").get("model")
        except (ValueError, AttributeError):
            model = None
        if not fence_allows(h.path, model, self.allowed):
            self.refused.append(model)
            why = f"brain_bench fence: only {self.allowed} may answer here (asked {model})"
            return h.send(503, json.dumps({"error": {"type": "fenced", "message": why}}).encode())
        try:
            r = self._up("POST", h.path.split("?")[0], body)
        except httpx.TimeoutException:
            return h.send(504, b'{"error": {"message": "brain_bench fence: llama-swap timed out"}}')
        except Exception as e:                           # noqa: BLE001
            return h.send(502, json.dumps({"error": {"message": f"brain_bench fence: {e}"}}).encode())
        h.send(r.status_code, r.content, r.headers.get("content-type", "application/json"))

    def close(self):
        self.srv.shutdown()
        self.srv.server_close()


# ------------------------------------------------------------------ I/O: the cockpit
def port_busy(port, host="127.0.0.1"):
    """Does anything accept a connection on host:port?"""
    try:
        with socket.create_connection((host, int(port)), timeout=1.0):
            return True
    except OSError:
        return False


def _child_cockpits(port):
    """PIDs of THIS process's own children running sim/cockpit.py --port PORT
    (from /proc: an interrupted start_cockpit leaves no handle to its Popen)."""
    me, out = os.getpid(), []
    try:
        pids = [d for d in os.listdir("/proc") if d.isdigit()]
    except OSError:
        return out
    for d in pids:
        try:
            with open(f"/proc/{d}/stat") as f:
                ppid = int(f.read().rsplit(")", 1)[1].split()[1])
            if ppid != me:
                continue
            with open(f"/proc/{d}/cmdline", "rb") as f:
                argv = [x.decode(errors="replace") for x in f.read().split(b"\0") if x]
        except (OSError, ValueError, IndexError):
            continue
        if any(a.endswith("cockpit.py") for a in argv) and "--port" in argv \
                and argv[argv.index("--port") + 1:][:1] == [str(port)]:
            out.append(int(d))
    return out


def seed_gesture_library(dst, src=None):
    """Copy the repo's keyframe gestures (gait/gestures/*.json) into dst once, so the bench
    cockpit knows the same gestures but a model's compose_gesture ('do a happy dance')
    saves into the scratch copy — the shakedown of 2026-09-25 left happy_dance.json in the
    repo. Returns the names copied (empty when dst already existed)."""
    src = src or os.path.join(ROOT, "gait", "gestures")
    if os.path.isdir(dst):
        return []
    os.makedirs(dst, exist_ok=True)
    names = []
    for fn in sorted(os.listdir(src)) if os.path.isdir(src) else []:
        if fn.endswith(".json"):
            shutil.copyfile(os.path.join(src, fn), os.path.join(dst, fn))
            names.append(fn)
    return names


class OwnCockpit:
    """The bench's own cockpit process: fenced (ROCKY_LLM_BASE_URL = the fence),
    a scratch conf (the owner's ~/.config/rocky/cockpit.json is never written) and
    a scratch scene-memory dir. Restartable: a new process has no last look /
    find_object result, which a world swap does not clear and every chat turn's
    situation line carries (the eye clause)."""

    def __init__(self, port, fence, tmp, speed, log):
        self.port, self.fence, self.tmp, self.speed, self.log = port, fence, tmp, speed, log
        self.proc, self.ck, self.restarts = None, None, 0

    @property
    def env(self):
        return {"ROCKY_LLM_BASE_URL": self.fence.base_url,
                "ROCKY_COCKPIT_CONF": os.path.join(self.tmp, "cockpit.json"),
                "ROCKY_MEMORY_DIR": os.path.join(self.tmp, "memory"),
                # a model's compose_gesture saves a gesture: into THIS copy, never gait/gestures
                "ROCKY_GESTURE_DIR": os.path.join(self.tmp, "gestures")}

    def start(self):
        if port_busy(self.port):
            raise SystemExit(f"something already answers on 127.0.0.1:{self.port} (a cockpit left behind by "
                             "an earlier bench run?) — the bench would drive THAT one, unfenced. Stop it, or "
                             "pick another --port")
        over = self.env
        seed_gesture_library(over["ROCKY_GESTURE_DIR"])
        saved = {k: os.environ.get(k) for k in over}
        os.environ.update(over)
        try:
            proc, ck = start_cockpit(self.port)
        except BaseException:
            for pid in _child_cockpits(self.port):       # interrupted mid-start: no orphan
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    pass
            raise
        finally:
            for k, v in saved.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
        self.proc = proc
        ck.c.timeout = httpx.Timeout(CHAT_TIMEOUT_S)
        if self.ck is None:
            self.ck = ck
        else:                                            # the callers hold self.ck: same URL, a fresh client
            old, self.ck.c = self.ck.c, ck.c
            old.close()
        self.ck.post("/api/speed", {"speed": self.speed})
        self.ck.post("/api/awareness", {"reactions": False, "curious": False})
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

    def restart(self, why):
        self.log(f"  a fresh bench cockpit ({why})")
        self.stop()
        t_end = time.monotonic() + 10
        while port_busy(self.port) and time.monotonic() < t_end:
            time.sleep(0.25)
        self.start()
        self.restarts += 1


def situation(ck):
    """The situation line a chat turn would carry right now (POST /api/awareness
    {refresh: true} composes it fresh, as a turn does), or None."""
    try:
        s = ck.post("/api/awareness", {"refresh": True})
    except Exception:                                   # noqa: BLE001
        return None
    t = ((s or {}).get("situation") or {}).get("text") if isinstance(s, dict) else None
    return str(t) if t else None


def eye_clause(text):
    """The situation line's eye clause (its last part: 'eye: ...' / 'eye (model, age): ...')."""
    if not text:
        return None
    i = text.rfind(". eye")
    return text[i + 2:] if i >= 0 else (text if text.startswith("eye") else None)


def eye_is_fresh(text):
    return bool(text) and EYE_EMPTY in (eye_clause(text) or "")


def ensure_fresh_eye(ck, own, log, why):
    """-> (situation text, restarted). The bench's own cockpit is restarted when
    its situation line still carries an eye clause (an earlier world's or model's
    look / find_object); a --url cockpit cannot be (own None)."""
    text = situation(ck)
    if own is None or eye_is_fresh(text):
        return text, False
    own.restart(f"{why}: the situation line still says {(eye_clause(text) or 'unknown')[:70]!r}")
    return situation(ck), True


class ActionWatch:
    """Polls /api/state (10 Hz) while one chat turn runs: first = seconds from
    start() to the first effect in the sim (acted()), or None; busy_s = how long
    the robot was walking or gesturing (the motion inside the turn's wall time)."""

    def __init__(self, url, period=0.1):
        self.url, self.period = url.rstrip("/"), period
        self.first, self.busy_s = None, 0.0
        self._stop = threading.Event()
        self._c = httpx.Client(timeout=3.0)

    def _state(self):
        try:
            return self._c.get(self.url + "/api/state").json()
        except Exception:                               # noqa: BLE001
            return None

    def start(self):
        s = self._state()
        self.base = (s or {}).get("events")
        self.t0 = time.monotonic()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()
        return self

    def _run(self):
        last = self.t0
        while not self._stop.is_set():
            s = self._state()
            now = time.monotonic()
            if isinstance(s, dict):
                if _active(s):
                    self.busy_s += now - last
                if self.first is None and self.base is not None and acted(s, self.base):
                    self.first = round(now - self.t0, 2)
            last = now
            self._stop.wait(self.period)

    def stop(self):
        self._stop.set()
        self._th.join(timeout=5)
        self._c.close()
        self.busy_s = round(self.busy_s, 2)
        return self.first


def settle(ck, max_s=SETTLE_S):
    """Wait until the reflex is NORMAL and nothing walks or gestures; -> the last status."""
    st, t_end = {}, time.monotonic() + max_s
    while time.monotonic() < t_end:
        try:
            st = ck.post("/api/tool/status")
        except Exception:                               # noqa: BLE001
            st = {}
        if st.get("reflex_state") == "NORMAL" and st.get("mode") not in ACTIVE_MODES:
            return st
        time.sleep(0.25)
    return st


def chat(ck, text, mode, model, timeout=CHAT_TIMEOUT_S):
    try:
        return ck.post("/api/chat", {"text": text, "mode": mode, "model": model, "source": "typed"},
                       timeout=timeout)
    except httpx.TimeoutException:
        return None


def ready(ck, fence, model, mode, log):
    """Point the fence and the cockpit's roles at `model`. -> (ok, why, roles)."""
    if fence is not None:
        fence.allowed = model
    try:
        ck.get("/api/models")                           # refresh the cockpit's cached catalog
    except Exception:                                   # noqa: BLE001
        pass
    key = "multimodal_model" if mode == "multimodal" else "model"
    r = ck.post("/api/brain", {"mode": mode, key: model})
    if not r.get("ok"):
        return False, f"the cockpit refused it as the {mode} role: {'; '.join(r.get('errors') or [])}", {}
    roles = {key: True}
    for k in ("vision_model", "multimodal_model"):
        if k == key:
            continue
        rr = ck.post("/api/brain", {k: model})
        roles[k] = bool(rr.get("ok"))
        if not rr.get("ok"):
            log(f"  {k}: not set ({'; '.join(rr.get('errors') or [])}) — look/find use the "
                "vision role's chain")
    ck.post("/api/brain", {"mode": mode})
    ck.post("/api/chat/clear", {"mode": "all"})         # nothing of the previous model's turns
    return True, None, roles


def first_answer(ck, model, mode, load_timeout, log):
    """The cold first answer. -> (seconds or None, response, why-not, tries, motion_s):
    motion_s = how long the sim was walking / gesturing inside those seconds."""
    watch = ActionWatch(ck.url).start()
    try:
        out = _first_answer(ck, model, mode, load_timeout, log)
    finally:
        watch.stop()
    return (*out, watch.busy_s)


def _first_answer(ck, model, mode, load_timeout, log):
    t0 = time.monotonic()
    tries = 0
    while True:
        tries += 1
        r = chat(ck, HELLO, mode, model, timeout=load_timeout + 30)
        el = round(time.monotonic() - t0, 1)
        try:
            ck.post("/api/tool/stop", {})
        except Exception:                               # noqa: BLE001
            pass
        ok, why = answered_by(r, model)
        if ok and el <= load_timeout:
            return el, r, None, tries
        if ok:
            return None, r, f"first answer took {el} s (> {load_timeout:.0f} s)", tries
        timed_out = r is not None and any("timed out" in str(n) for n in r.get("fallback") or [])
        if not timed_out or tries >= 2 or el >= load_timeout:
            return None, r, why or f"no answer within {load_timeout:.0f} s", tries
        # the cockpit's own 90 s call timeout fired during a cold load: retry once, and only
        # when llama-swap is still loading or has loaded it (never a second spawn)
        state = running_states().get(model)
        if state is None:
            return None, r, f"{why} (and llama-swap is not loading it)", tries
        log(f"  the cockpit timed out while {model} was loading (llama-swap: {state}); waiting for it")
        t_wait = time.monotonic() + max(0.0, load_timeout - el)
        while time.monotonic() < t_wait and running_states().get(model) == "starting":
            time.sleep(1.0)
        state = running_states().get(model)
        if state != "ready":                            # gone or stuck: asking again would spawn it again
            return None, r, f"{why} (llama-swap: {state or 'not running'} after the wait)", tries
        log(f"  {model} is ready; asking once more")


def run_commands(ck, fence, model, mode, n, log):
    rows, timeouts = [], 0
    for cmd in COMMANDS[:n]:
        if timeouts >= MAX_TIMEOUTS:
            rows.append(dict(not_run_row(cmd, f"not run: {MAX_TIMEOUTS} chat timeouts in a row"),
                             latency_s=None, first_action_s=None, motion_s=None))
            continue
        ck.post("/api/chat/clear", {"mode": mode})
        st = settle(ck)
        reset = False
        if st.get("reflex_state") in ("FALLEN", "RIGHTED") or not st.get("pose"):
            ck.post("/api/reset")
            st, reset = settle(ck), True
        pose = st.get("pose")
        if fence is not None:
            fence.refused.clear()
        watch = ActionWatch(ck.url).start()
        t0 = time.monotonic()
        r = chat(ck, cmd["line"], mode, model)
        lat = round(time.monotonic() - t0, 2)
        first = watch.stop()
        try:
            ck.post("/api/tool/stop", {})
        except Exception:                               # noqa: BLE001
            pass
        timeouts = timeouts + 1 if r is None else 0
        row = score_command(cmd, r, model, mode, pose, fenced=fence is not None)
        row.update(latency_s=lat if r is not None else None, first_action_s=first, motion_s=watch.busy_s,
                   start_pose=pose, reset_before=reset)
        if fence is not None and fence.refused:
            row["fenced"] = sorted({str(m) for m in fence.refused})
        rows.append(row)
        tools = ", ".join(f"{t['name']}({json.dumps(t['args'])[:60]})" for t in row["tools"]) or "-"
        fa = "-" if first is None else f"{first:.1f}"
        log(f"  {cmd['line'][:38]:38s} {row['verdict']:10s} {lat:6.1f} s  1st {fa:>5}  {tools[:90]}"
            + (f"  [{row['why'][:90]}]" if row["why"] else ""))
    settle(ck)
    return rows


def run_describe(ck, model, n, log, own=None, rearm=None):
    """The describe cases. own: the bench's OwnCockpit (restarted before a case
    whose situation line still has an eye clause) or None (--url: recorded as
    stale_eye); rearm(): puts the roles back on a fresh cockpit."""
    rows = []
    if own is None:
        log("  (--url: the cockpit's last look / find cannot be cleared — rows record the eye context "
            "they were sent with)")
    for i, (expect, obj) in enumerate(describe_cases(n)):
        _sit, restarted = ensure_fresh_eye(ck, own, log, f"describe case {i}")
        if restarted and rearm is not None:
            rearm()
        ck.stage(describe_world(model, i, expect), {"base": "flat", "objects": [obj]})
        ck.post("/api/chat/clear", {"mode": "multimodal"})
        sit = situation(ck)
        t0 = time.monotonic()
        r = chat(ck, DESCRIBE_Q, "multimodal", model)
        lat = round(time.monotonic() - t0, 2)
        ck.post("/api/tool/stop", {})
        row = {"case": i, "expect": expect, "object": obj, "latency_s": lat if r is not None else None,
               "world": describe_world(model, i, expect), "situation": sit, "eye_context": eye_clause(sit),
               "stale_eye": not eye_is_fresh(sit), "fresh_cockpit": restarted}
        ok, why = answered_by(r, model)
        if r is None:
            row["error"] = f"no answer within {CHAT_TIMEOUT_S:.0f} s"
        elif not ok:
            row["error"] = why
        else:
            reply = strip_notes(r.get("reply", ""), model)
            row.update(reply=reply[:300], tools=[c["name"] for c in tool_calls(r)],
                       score=score_describe(expect, reply))
        rows.append(row)
        sc = row.get("score")
        log(f"  describe {expect:6s} " + (f"ERR {row['error'][:80]}" if "error" in row else
                                          f"{'OK ' if sc['ok'] else 'NO '} ball={sc['mentions_ball']!s:5} "
                                          f"side={sc['side']} {lat} s: {row['reply'][:80]!r}")
            + ("  [stale eye context]" if row["stale_eye"] else ""))
    return rows


def bench_model(ck, fence, m, args, log, res=None, own=None):
    """One model's run; fills `res` in place (so an interrupted run keeps what it measured)."""
    res = {} if res is None else res
    res.update(mode=args.mode, notes=[])
    info = read_stanza(m)
    res.update(files=info.get("cell", "—"), ctx=info.get("ctx"))
    log(f"\n{m}: clearing the GPU")
    gone, left = clear_gpu(log, args.idle)
    res["unloaded_before"] = gone
    if left:
        res["skipped"] = f"llama-swap still has {', '.join(left)} loaded after the idle wait"
        return res
    _sit, res["fresh_cockpit"] = ensure_fresh_eye(ck, own, log, f"before {m}")
    ck.stage(f"bb-course-{world_tag(m)}", json.loads(json.dumps(PRESETS["obstacle course"])))
    settle(ck)
    base = gpu_used_mib()
    res["vram_baseline_mib"] = base
    ok, why, roles = ready(ck, fence, m, args.mode, log)
    res["roles"] = roles
    if not ok:
        res["skipped"] = why
        return res
    log(f"{m}: cold first answer ({args.mode} role)")
    first, r, why, tries, motion = first_answer(ck, m, args.mode, args.load_timeout, log)
    res["first_answer_attempts"] = tries
    res["first_answer_tools"] = tools_brief(tool_calls(r))
    res["first_answer_motion_s"] = motion
    if first is None:
        res["skipped"] = why
        res["loaded"] = m in running_models()
        return res
    res["loaded"] = True
    res["first_answer_s"] = first
    res["first_answer_model_s"] = round(max(0.0, first - motion), 1)
    res["first_reply"] = strip_notes((r or {}).get("reply", ""), m)[:200]
    used = gpu_used_mib()
    res["vram_mib"] = None if used is None or base is None else used - base
    samples = [used]
    log(f"{m}: first answer {first} s" + (f" ({res['first_answer_model_s']} s without {motion} s of "
                                          f"{', '.join(t['name'] for t in res['first_answer_tools'])})"
                                          if motion else "")
        + f", VRAM +{res['vram_mib']} MiB (baseline {base})")
    dec = decode_speed(m)
    res["decode"] = dec
    res["decode_tps"] = dec.get("decode_tps")
    log(f"{m}: decode {dec.get('decode_tps')} t/s ({dec.get('source') or dec.get('error')})")
    samples.append(gpu_used_mib())
    if args.commands > 0:
        log(f"{m}: {min(args.commands, len(COMMANDS))} commands")
        rows = run_commands(ck, fence, m, args.mode, args.commands, log)
        res["commands"] = rows
        res["summary"] = summarize_commands(rows)
        samples.append(gpu_used_mib())
        if res["summary"]["timeouts"] >= MAX_TIMEOUTS:
            res["notes"].append("commands abandoned after chat timeouts: describe and vision skipped")
            args_describe, args_vision = 0, False
        else:
            args_describe, args_vision = args.describe, args.vision
    else:
        args_describe, args_vision = args.describe, args.vision
    if args_describe > 0:
        if args.mode == "multimodal" or roles.get("multimodal_model"):
            log(f"{m}: describe ({args_describe} turns, multimodal)")
            drows = run_describe(ck, m, args_describe, log, own=own,
                                 rearm=lambda: ready(ck, fence, m, args.mode, log))
            res["describe"] = drows
            res["describe_summary"] = summarize_describe(drows)
            samples.append(gpu_used_mib())
        else:
            res["notes"].append("describe skipped: the cockpit does not accept it as multimodal (text-only)")
    if args_vision:
        if args.mode == "multimodal" or roles.get("vision_model"):
            log(f"{m}: vision bench (bbox + floor)")
            det = vb.run_detect(ck, m, "bbox", log)
            fl = vb.run_floor(ck, m, log)
            res["vision"] = {"detect": det, "floor": fl, "summary": vb.summarize(det),
                             "floor_correct": f"{sum(1 for x in fl if x.get('correct'))}/{len(fl)}"}
            samples.append(gpu_used_mib())
        else:
            res["vision"] = {"error": "text-only per llama-swap metadata"}
    if base is not None:
        vals = [s - base for s in samples if s is not None]
        res["vram_max_mib"] = max(vals) if vals else None
    return res


# ------------------------------------------------------------------ --url: leave the cockpit as found
def target_settings(ck):
    """A --url cockpit's roles, mode and speed before the bench changes them
    (roles and mode persist in its conf file). -> dict, or None when unreadable."""
    try:
        j = ck.get("/api/models")
    except Exception:                                   # noqa: BLE001
        return None
    roles = (j or {}).get("roles") if isinstance(j, dict) else None
    if not isinstance(roles, dict) or not roles:
        return None
    out = {"roles": dict(roles), "mode": ((j.get("brain") or {}).get("mode"))}
    try:
        out["speed"] = ck.get("/api/state").get("speed")
    except Exception:                                   # noqa: BLE001
        out["speed"] = None
    return out


def restore_target(ck, saved, log):
    """POST the saved roles / mode / speed back. -> what happened (one line)."""
    done = []
    body = {"roles": {k: v for k, v in (saved.get("roles") or {}).items() if v}}
    if saved.get("mode"):
        body["mode"] = saved["mode"]
    try:
        r = ck.post("/api/brain", body)
        done.append("roles and mode restored" if r.get("ok") else
                    f"roles NOT all restored: {'; '.join(r.get('errors') or [])}")
    except Exception as e:                              # noqa: BLE001
        done.append(f"roles NOT restored ({type(e).__name__}: {e})")
    if saved.get("speed") is not None:
        try:
            ck.post("/api/speed", {"speed": saved["speed"]})
            done.append(f"speed {saved['speed']} restored")
        except Exception as e:                          # noqa: BLE001
            done.append(f"speed NOT restored ({type(e).__name__})")
    msg = "; ".join(done)
    log(f"--url cockpit: {msg}")
    return msg


# ------------------------------------------------------------------ main
class BenchStopped(SystemExit):
    """SIGTERM / SIGHUP: unwinds main() like Ctrl-C does, so its cleanup runs."""

    def __init__(self, signum):
        super().__init__(128 + int(signum))
        self.signame = signal.Signals(signum).name


def _on_signal(signum, _frame):
    raise BenchStopped(signum)


def _install_signals():
    """-> the previous handlers (None when not in the main thread)."""
    try:
        return {s: signal.signal(s, _on_signal) for s in (signal.SIGTERM, signal.SIGHUP)}
    except ValueError:                                  # not the main thread
        return None


def _restore_signals(old):
    for s, h in (old or {}).items():
        try:
            signal.signal(s, h)
        except (ValueError, TypeError):
            pass


def write_result(result, path, lines):
    md = render_markdown(result)
    result["markdown"] = md
    result["log"] = list(lines)
    with open(path, "w") as f:
        json.dump(result, f, indent=1, default=str)
    return md


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog="Scoring, latency, the fence and stopping: see the module docstring "
                                        "(sim/brain_bench.py).")
    ap.add_argument("--models", required=True, help="llama-swap model id(s), comma-separated")
    ap.add_argument("--url", help=f"a running cockpit on this machine's loopback (never :{OWNER_PORT}); "
                                  "default: start one on --port")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help="the bench's own cockpit (default %(default)s)")
    ap.add_argument("--mode", choices=["multimodal", "local"], default="multimodal",
                    help="the role under test (default %(default)s)")
    ap.add_argument("--commands", type=int, default=20,
                    help=f"command lines (max {len(COMMANDS)}; default 20)")
    ap.add_argument("--describe", type=int, default=6, help="describe turns (default 6)")
    ap.add_argument("--vision", action="store_true", help="also run the vision bench's detect + floor cases")
    ap.add_argument("--keep-loaded", action="store_true", help="leave each model loaded after its run")
    ap.add_argument("--load-timeout", type=float, default=180.0, help="cold first answer limit, s")
    ap.add_argument("--idle", type=float, default=20.0, help="true idle after unloading, s (GPU rule: 15-30)")
    ap.add_argument("--speed", type=float, default=2.0, help="physics speed (wall time only)")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args(argv)

    if args.url:
        bad = refuse_url(args.url)
        if bad:
            raise SystemExit(bad)
    elif args.port == OWNER_PORT:
        raise SystemExit(f"refusing --port {OWNER_PORT}: that is the owner's live cockpit")
    args.commands = max(0, min(args.commands, len(COMMANDS)))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)

    listed = listed_models()
    if not listed:
        raise SystemExit("llama-swap does not answer GET /v1/models on :8080 — nothing to bench")
    cat = catalog()
    models, skipped = [], {}
    for m in [x.strip() for x in args.models.split(",") if x.strip()]:
        e = cat.get(m) or {}
        if m in QUARANTINED or e.get("quarantined"):
            skipped[m] = "quarantined (GPU faults) — never used"
        elif m not in listed:
            skipped[m] = "not listed by llama-swap (installed and restarted?)"
        else:
            models.append(m)

    lines = []

    def log(s):
        print(s, flush=True)
        lines.append(s)

    log("brain_bench: do not chat in the live cockpit while this runs — its models are unloaded here")
    fence, own, tmp, ck, saved = None, None, None, None, None
    before = sorted(m for m in running_models() if m and m not in KEEP_RUNNING)
    result = {"date": time.strftime("%Y-%m-%d %H:%M"), "cockpit": None,
              "mode": args.mode, "speed": args.speed, "fenced": False,
              "commands_n": args.commands, "describe_n": args.describe, "vision": args.vision,
              "commands": [c["line"] for c in COMMANDS[:args.commands]],
              "running_at_start": before, "skipped": skipped, "models": {}}
    loaded, current, interrupted = [], None, None
    old_signals = _install_signals()
    try:
        if args.url:
            ck = Cockpit(args.url, timeout=CHAT_TIMEOUT_S)
            if not ck.alive():
                raise SystemExit(f"no cockpit answers at {args.url}")
            saved = target_settings(ck)
            if saved is None:
                raise SystemExit(f"could not read the roles of the cockpit at {args.url} (GET /api/models) — "
                                 "refusing: the bench's role changes are saved to that cockpit's conf file "
                                 "and could not be put back")
            log(f"cockpit {args.url}: NOT fenced — its fallback chain may load other models; its roles "
                f"({', '.join(f'{k}={v}' for k, v in saved['roles'].items())}), mode and speed are put back "
                "at the end")
            if models:
                ck.post("/api/speed", {"speed": args.speed})
        elif models:
            fence = Fence()
            tmp = tempfile.mkdtemp(prefix="brain_bench_")
            own = OwnCockpit(args.port, fence, tmp, args.speed, log)
            ck = own.start()
            log(f"cockpit {ck.url} (own, fenced through {fence.base_url})")
        result.update(cockpit=None if not models else ck.url, fenced=fence is not None)
        for m in models:
            current = m
            res = result["models"][m] = {}
            bench_model(ck, fence, m, args, log, res=res, own=own)
            current = None
            if res.get("loaded"):
                loaded.append(m)
            if not args.keep_loaded and res.get("loaded"):
                res["unloaded_after"] = unload(m)
                loaded.remove(m)
            if res.get("skipped"):
                log(f"{m}: SKIPPED — {res['skipped']}")
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
            if own is not None:
                own.stop()
                result["cockpit_restarts"] = own.restarts
            if fence is not None:
                fence.close()
            if tmp:
                shutil.rmtree(tmp, ignore_errors=True)
            if not args.keep_loaded:
                now = running_models()
                for m in sorted(set(loaded) | ({current} if current else set())):
                    if m in now:
                        ok = unload(m)                  # interrupted mid-run: leave the GPU as found
                        log(f"{m}: unloaded after the interruption ({'ok' if ok else 'FAILED'})")
            if interrupted and result["models"]:
                result["interrupted"] = f"{interrupted} during {current or 'the wrap-up'}"
                write_result(result, args.out, lines)
                print(f"\ninterrupted ({interrupted}): partial results in "
                      f"{os.path.relpath(os.path.abspath(args.out), ROOT)}", flush=True)
        finally:
            _restore_signals(old_signals)

    log("")
    log(render_table(result))
    if before:
        log(f"(unloaded for the bench: {', '.join(before)} — llama-swap loads each again, cold, "
            "on its next request)")
    md = write_result(result, args.out, lines)
    print("\nMarkdown for docs/BRAIN_MODELS_2026-09-24.md:\n")
    print(md)
    print(f"\nwrote {os.path.relpath(os.path.abspath(args.out), ROOT)}")
    return result


if __name__ == "__main__":
    main()
