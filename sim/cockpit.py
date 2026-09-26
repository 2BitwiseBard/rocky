"""Pebble COCKPIT (D049) — the browser playground: live cameras, teleop,
console, a chat with a switchable brain (regex / local LLM / Claude), the
robot's eye + a vision model, a world editor (obstacles, terrain, friction,
slopes) and the RL panel (checkpoint table, righter hot-swap, tuning),
all on ONE continuously running MuJoCo sim.

    ./rocky.sh cockpit                # http://127.0.0.1:8765
    MUJOCO_GL=egl python sim/cockpit.py --port 8765 --world "obstacle course"

Architecture: one sim thread owns physics (the Playground step loop with a
goto controller + cliff detector folded in, real-time paced, offscreen
rendering of a chase camera and the eye camera to JPEG). Everything else
talks to it through a job queue (`call`), so the HTTP layer, the brains
and the MCP proxy (harness/cockpit_backend.py — `./rocky.sh chat` drives
THIS sim while the cockpit is up) never touch MjData from another thread.
Guard supremacy is unchanged: goto runs the reflex supervisor and the
cliff detector exactly as harness/sim_backend.py does; a veto comes back
as stopped='cliff'.

Honesty: same MJCF, same guessed servo gains and friction as every other
sim entry point (see playground.py's box). The vision tool sends the eye
camera's JPEG to a local vision model; its words are the model's, not a
sensor's.

D052 (this file): the sim thread can no longer die silently (a step that
raises limps the bridge, fails every waiting request and exits non-zero;
/api/state carries a heartbeat); the server refuses a non-loopback --host
unless --unsafe-lan (nothing here is authenticated) and every POST must be
same-origin JSON (voice: multipart); goto shares the Playground's always-on
void guard, reads the lidar at 8 Hz for what is in its way (a +-45 deg
detour, then a sidestep, then stopped='blocked' with bearing + range) and
calls a 3 s no-progress run 'stuck'; the gesture studio checks every spec
with pebble_feasibility before it moves or saves anything, solves reaches
with the whole-body pose solver and turns a recorded pose stream into
keyframes; the residual walker is fed the observation its checkpoint was
trained on (rl_common contract), not a hand-built one.

Memory + awareness (2026-09-24): sim.memory is a SceneMemory per world
(sim/scene_memory.py; persisted by main() to sim/out/memory/<key>.json —
memory_key: a preset by name, anything else by name + a hash of its spec);
guards, looks and finds record into it; a reset / world load starts a new
memory epoch (earlier sightings turn stale, 'start' is pinned at the spawn). _aware_tick (sim thread, 1 Hz wall
clock, never fatal) composes sim.situation while idle (--awareness-s),
reacts to a guard latch / a new obstacle with a chord (reactions, once per
30 s) and, only with --curious, schedules one look a minute. Routes:
/api/memory, /api/awareness; the state feed carries situation +
memory_objects.

Hygiene (2026-09-25): a world load / edit / reset forgets the eye's last
description and find_object's last report, the composed situation, the
awareness tick's previous scan and the eye FRAME itself (_forget_scene; live
cameras re-render on the next loop turn, paused or not), so neither the next
situation line nor the next look describes a world that is gone; the state feed's `event_seq` counts
every event ever appended (EventLog), so a client sees a new event even
when the 12-event window looks the same.

Capabilities (D056): GET /api/capabilities is what this cockpit can do right
now, from the one tool registry (harness/capabilities.py): the live gesture
and chord-word lists, every tool it runs with its schema and whether a spoken
line needs the wake word for it, the live gait envelope and the robot, plus a
`version` (12 hex of a sha256 over all of it). Built on demand and cached by
its inputs; saving or deleting a gesture or a chord word, or a gait change,
makes a new version. The state feed (/api/state, /api/events) carries it as
`caps_version`, with `caps_seq` counting the changes since start, and the
console logs each change. It is NOT an entry on the events feed: the `status`
tool hands every model that feed's last five entries, and a tool the models
already call must not answer differently because a list or the gait moved.

Places (D057, 2026-09-25; OFF by default: awareness `recognize`, POST
/api/awareness {recognize: true}, or --recognize). A world's name is ground
truth a real robot never has, so with recognition on the scene memory is
keyed by the place the robot RECOGNISES (sim/place_memory.py), not by the
world. After a spawn (a world load, a reset; an edit keeps the robot where it
is) the memory is a provisional RAM-only one (PLACE_PENDING_KEY); once the
robot stands still (RECOG_SETTLE_S) it takes a lidar sweep (scan_signature,
the sim's pose standing in for odometry — a perfect one) and a look (the
vision role, Brains.look), embeds the description (sim.embed_fn, default
Brains.embed: llama-swap's CPU 'embedding' model) and asks
PlaceMemory.recognize. 'ambiguous' -> a second look after a 30 deg
turn_in_place (the `turn` tool: one gesture, every guard) and the two are
combined; 'new' -> the same second look first (the situation line says it is
looking again), then a place is stored ("new place #k") from BOTH looks as
samples (PlaceMemory.enroll_samples) and the memory bound to it; 'known' -> a
visit, the memory bound to 'place-<id>' (what was sensed since the spawn is
carried over), and the place is checked for changes BY THE EYE, not by the
prose: every remembered object name and every thing the description mentions
(LOOK_NOUNS, not a fixture) is one question to the vision model — the box
question find_object and /api/look {find, method: "bbox"} ask
(Brains.detect) — answered True (boxed; the box's floor geometry places it,
sighting_to_map) / False (not there) / None (no answer, or its remembered
spot out of the eye's 86 deg x 2 m view: unobservable), fed to
place_memory.checked_objects. Any change takes the second look, which asks the
same questions again, and only what BOTH looks agree on (confirm_diff) is
declared or stored — one glance never declares a change, and a failed look
or an unanswered question is never 'not there'. The situation line then
starts with verdict_text (<= 90 chars), the state feed carries `place` (with
`signals`: 'scan + description' | 'scan only' | ...), /api/memory `places`;
a new place or a confirmed new object says curious_question (reactions,
rate-limited as ever). Tools:
where_am_i, name_place, places, forget_place; routes GET/POST /api/place.
Recognition off = everything as before (memory keyed by world, no `place`,
and the models' tool list is the D056 one: the place tools need the
registry's `places` capability).
Review fixes (2026-09-25): a look that FAILED has no objects (None, not []):
it is never read as "nothing is there", so no change check runs on it and
the answer says the verdict rests on the lidar. An edit (the robot was not
moved) and place_check (talk mode's "this X has a new Y") are change checks
of the bound place only — never a new place, never a rebind. name_place is
a correction when the name is another place's or the recognised place's
own name differs (rename=true renames instead), and "this is completely
new" stores ONE place per visit; a correction takes back what the visit
wrote into the wrongly recognised place (_place_take_back: its record from
before the visit, its scene memory from the bind) and moves the records to
the right one. While recognition is on, no memory record, status or
scan_summary result names the world.
Review fixes, round 2 (2026-09-25): both looks of a change check stand on
one spot (the second turns in place), so a remembered spot hidden behind a
box the eye drew in that look, or behind a nearer lidar return, is
unobservable, never missing (look_hidden / look_view) — two 'not there'
answers from one line of sight prove nothing. A thing the eye boxes but can
never place (above the horizon: on a table; beyond 2 m) is kept by name as a
place note (PLACE_UNPLACED_NOTE), so a later visit counts it present instead
of 'new here' every time; placed later, it is stored, not declared. The
answer carries per_look (each look's own parts, evidence, signals, runner-up
and ranking) and signal_list, so a two-look verdict can be replayed.
"""
from __future__ import annotations
import argparse
import asyncio
import contextlib
import copy
import hashlib
import io
import ipaddress
import json
import math
import os
import queue
import re
import socket
import sys
import threading
import time
import traceback
import warnings
from collections import deque
from urllib.parse import urlsplit

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np                                                    # noqa: E402
import mujoco                                                         # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for sub in ("gait", "perception", "sim"):
    sys.path.insert(0, os.path.join(ROOT, sub))
sys.path.insert(0, ROOT)

from playground import (Playground, TELEOP_HELP, HELP_RL,                     # noqa: E402
                        all_gestures, lexicon, chord_wav, CHORD_CUSTOM_DIR, gait_presets, GAIT_KEYS,
                        shove_args)
from pebble_keyframes import (KeyframeGesture, save_keyframe_gesture,           # noqa: E402
                              delete_keyframe_gesture, load_keyframe_gestures, GESTURE_DIR)
from hw_bridge import HardwareBridge, serial_ports, MIRRORS                     # noqa: E402
import hw_bridge                                                                # noqa: E402
from world_builder import (build as build_world, PRESETS, KINDS, random_course,   # noqa: E402
                           saved_worlds, save_world, load_world)
from pebble_reflex import ReflexSupervisor                             # noqa: E402
from sim_lidar import scan as lidar_scan, RANGE_MAX, PUCK_DZ, RATE_HZ as LIDAR_HZ   # noqa: E402
import rocky_model as rm                                               # noqa: E402
import pebble_feasibility as pf                                        # noqa: E402
import rl_common as rc                                                 # noqa: E402
from model_fingerprint import robot_fingerprint, fingerprint_note      # noqa: E402
from shove import Shove                                                # noqa: E402
from harness.backend import CHORD_WORDS, SIGNED                        # noqa: E402
import harness.capabilities as tool_registry                           # noqa: E402
# D052: the brains (roles, fallbacks, histories, voice, look) live in cockpit_brains
from cockpit_brains import (Brains, TOOLS, SYSTEM, VISION_PROMPT, TOOL_NAMES,   # noqa: E402,F401
                            validate_goto, goto_range_error, local_ai_key as _local_ai_key,
                            cockpit_flags)
# scene memory + situational awareness (the owner's "memory" and "awareness" asks)
from scene_memory import (SceneMemory, DEFAULT_DIR as MEMORY_DIR, fmt_age,     # noqa: E402
                          objects_from_description, same_thing, norm_name, LOOK_NOUNS)
# D057: which place is this, from what the robot senses (not the world's name)
import place_memory as pm                                              # noqa: E402
from cockpit_brains import EYE_HFOV_DEG, EYE_FWD_M, place_scope, PLACE_TOOLS   # noqa: E402,F401
from cockpit_brains import sighting_to_map, FIND_MIN_CONF, FIND_TRUSTED_M      # noqa: E402
from cockpit_brains import floor_to_pixel                                      # noqa: E402

V_GOTO = 45.0                # asked; WaveGait.budget fits it into the envelope (45.5 mm/s today)
GOTO_CAP_S = 40.0            # a goto that has not ended by then ends as "timeout"
GOTO_STUCK_S = 3.0           # D052: no 2 cm of progress for this long -> "stuck" (was 6 s: a
#                              robot shoving a low box for 6 s is 6 s of stalled servos)
GOTO_SCAN_HZ = LIDAR_HZ      # the reactive layer reads the puck at its own rate (8 Hz)
GOTO_CONE_DEG = 30.0         # a lidar return within +-30 deg of the travel heading ...
GOTO_CLEAR_M = 0.35          # ... closer than this (from the puck = torso centre) is in the way.
#                              The feet reach ~0.19 m out, so 0.35 m leaves ~15 cm to stop in.
GOTO_DETOURS = 2             # detour 1 = +-45 deg off the target heading, 2 = a sidestep (90 deg);
#                              blocked a third time -> stopped='blocked' (bearing + range)
GOTO_DETOUR_S = 8.0          # a detour walks at most this long (~36 cm at 45 mm/s: past a
#                              0.25 m-wide object with the leg span clear) ...
GOTO_RESUME_S = 0.8          # ... or until the corridor toward the target has been clear this long
GOTO_HALF_W = 0.25           # m: half the leg span (R0 185 mm + foot) — the corridor that must be clear
GOTO_DETOUR_RESET_M = 0.15   # this much new progress after a detour earns the detours back
TEACH_HZ = 20.0              # the studio's pose-stream recorder
TEACH_MAX_S = 60.0           # ... stops itself after this long (1200 samples)
HEARTBEAT_STALE_S = 2.0      # the UI calls the sim thread dead after this long without a loop
# ---- situational awareness (the sim thread; wall-clock seconds, so 4x speed does not
#      make the robot chattier). See CockpitSim._aware_tick.
AWARENESS_S = float(os.environ.get("ROCKY_AWARENESS_S", "20"))   # compose the situation this often
#                              while idle + NORMAL (0 = off); a chat turn always composes a fresh one
AWARE_CHECK_S = 1.0          # the lidar look-around for reactions (1 Hz; goto reads it at 8 Hz)
REACT_MIN_S = 30.0           # at most one spoken reaction per this long
CURIOUS_MIN_S = 60.0         # 'curious': at most one unprompted look (a vision-model call) per this long
NEW_OBSTACLE_M = 0.5         # a lidar return this close where there was none (>= +0.1 m) = something new
STILL_M, STILL_DEG = 0.03, 5.0   # ... judged only while the robot itself stood still
SCENE_CHANGE_M = 0.15        # a sector's nearest return moved this much: the scene changed (curious)
AWARE_FAILS_MAX = 3          # an awareness tick that keeps raising turns awareness off (never the sim)
SECTORS = ("ahead", "ahead-left", "left", "behind-left", "behind", "behind-right", "right", "ahead-right")
# bus events that are not guard trips: 'rate' (oversized goal steps, throttled to 1/s while
# streaming) and 'reopen' (a recovery) flooded the memory as guards (review 2026-09-24)
HW_ROUTINE = {"scan", "entry", "mirror", "rearm", "limp", "set_id", "center", "dir", "limits", "rate", "reopen"}
HW_ALARM = {"cut", "lost"}   # bus events that are guard trips (a servo cut, a lost servo/port)
HW_GUARD_EVERY_S = 30.0      # a bus guard (nan, cut, lost ...) goes to the memory at most once per kind per this
HEAT_SAY = 0.5               # the situation line mentions servo heat above this fraction of the budget
AUDIO_DIR = os.path.join(ROOT, "audio")
CHORD_SPEC_DIR = os.path.join(AUDIO_DIR, "custom")          # D051: chord words designed in the cockpit
# ---- place recognition (D057; OFF by default — awareness 'recognize', --recognize). See recognize_place.
PLACE_PENDING_KEY = "place-pending"   # the provisional scene memory (RAM only) until a verdict binds a place
RECOG_SETTLE_S = 1.0                  # sim s after a spawn before the sweep + look (the robot stands still)
RECOG_WAIT_S = 20.0                   # wall s a recognition waits for a standstill, then gives up (tried again)
RECOG_RETRY_S = 5.0                   # an auto recognition that could not run is tried again after this
RECOG_TIMEOUT_S = 180.0               # a recognition still 'running' after this is taken for dead (a lost loop)
PLACE_TURN_DEG = 30.0                 # the second look's turn (+ = left, counter-clockwise): the `turn` tool
PLACE_MATCH_M = 0.5                   # same name within this = the same object. UNMEASURED: a look description
#                                       places things ~0.3 m off (median, scene_memory), so place_memory's 0.3 m
#                                       MATCH_M would call half of them moved
VIEW_MAX_M = 2.0                      # the eye 'could see' a floor point out to here (cockpit_brains.FIND_TRUSTED_M)
VIEW_NEAR_M = 0.15                    # ... and not closer (the frame's bottom edge meets the floor ~0.18 m ahead)
PLACE_EMBED_WAIT_S = 8.0              # the embedding call's own timeout is 5 s (cockpit_brains.EMBED_TIMEOUT_S)
PLACE_NAME_WAIT_S = 30.0              # name_place waits this long for a recognition that is running
PLACE_OFF = "place recognition is off"
PLACE_OFF_HINT = ("the operator switches it on (cockpit awareness settings: POST /api/awareness "
                  "{\"recognize\": true}, or start the cockpit with --recognize)")
_STALE = "stale"                      # _place_observe / _place_second_look: the scene changed meanwhile
# the change check asks the EYE, not the prose (B38 follow-up, 2026-09-25): per name, the question
# find_object and /api/look {find, method: "bbox"} ask (Brains.detect: FIND_PROMPT, a box ->
# pixel_to_floor -> sighting_to_map; 0.7-0.9 deg / 3 cm measured on the vision bench)
PLACE_EYE_MAX = 10                    # questions (one vision-model call each) per look, at most ...
PLACE_EYE_NEW = 5                     # ... this many of them about what the description mentions (asked
#                                       first, as the look is taken), so the remembered names keep >= 5
PLACE_FIXTURES = frozenset({"wall", "door"})   # part of the room (the lidar's business): never asked
#                                       about, never 'added' or 'missing'
# occlusion (review 2026-09-25): the second look turns IN PLACE, so both looks share one line of sight
# to every spot — a thing hidden from the first is hidden from the second, and two 'not there' answers
# would erase it. A remembered spot a look cannot see past something is unobservable, never missing:
PLACE_HIDE_PAD = 0.02                 # ... its image point inside a box the eye drew this look (widened
#                                       by this fraction of the frame each side; a floor point inside a
#                                       box lies BEHIND that thing's foot), or
PLACE_HIDE_LIDAR_DEG = 2.0            # ... a lidar return within this many degrees of its bearing (from
PLACE_HIDE_LIDAR_M = 0.15             # the torso) and this much nearer than it (the stored spot is the
#                                       thing's centre): something >= ~0.18 m tall stands in the way (the
#                                       eye is 0.21 m up, so it cannot see over it to the floor behind)
# the names the eye boxed at a place but could never place (a cup on a table, a lamp beyond 2 m): kept
# as a place note, so a later visit counts them present, not 'new here' (review 2026-09-25). A note is
# PlaceMemory's own persisted, per-place text (restored and forgotten with the place); place_memory has
# no field for them yet.
PLACE_UNPLACED_NOTE = "eye: boxed here, never placed: "
_PLACE_NEW_WORDS = frozenset({"", "new", "new place", "a new place", "somewhere new", "new room", "a new room"})

def memory_key(name, spec):
    """The scene-memory key (file name) of a world: a preset loaded as it
    ships keeps its name ('room.json'); anything else — an edit, an unnamed
    spec ('custom'), a replay's recorded world — is name + a hash of its
    spec, so unrelated 'custom' worlds never share (or overwrite) a memory
    (review 2026-09-24). Loading the same spec again finds its memory."""
    name = str(name)
    try:
        blob = json.dumps(spec, sort_keys=True, default=str)
    except (TypeError, ValueError):
        blob = repr(spec)
    if name in PRESETS and blob == json.dumps(PRESETS[name], sort_keys=True, default=str):
        return name
    return f"{name}-{hashlib.sha1(blob.encode()).hexdigest()[:8]}"


# ---------------------------------------------------------- D057: place recognition helpers (pure)
def eye_visible(pose, max_m=VIEW_MAX_M, near_m=VIEW_NEAR_M, hfov_deg=EYE_HFOV_DEG):
    """visible(x, y) for place_memory.diff_objects: could a look from `pose` ({x, y,
    yaw_deg}) have seen the floor point (x, y)? Inside the eye's horizontal field of view
    (hfov_deg, ~86 deg, centred on the heading, from the eye EYE_FWD_M ahead of the torso)
    and between near_m and max_m (~2 m: FIND_TRUSTED_M) from it. Occlusion is NOT modelled
    here (a thing hidden behind another counts as visible): a change check reads a look
    through look_view, which adds it (look_hidden)."""
    yaw = math.radians(float(pose.get("yaw_deg", pose.get("yaw", 0.0)) or 0.0))
    ex = float(pose["x"]) + EYE_FWD_M * math.cos(yaw)
    ey = float(pose["y"]) + EYE_FWD_M * math.sin(yaw)
    half = float(hfov_deg) / 2.0

    def visible(x, y):
        dx, dy = float(x) - ex, float(y) - ey
        d = math.hypot(dx, dy)
        if not near_m <= d <= max_m:
            return False
        b = (math.degrees(math.atan2(dy, dx) - yaw) + 180.0) % 360.0 - 180.0
        return abs(b) <= half
    return visible


def eye_pixel(pose, x, y):
    """Where the map floor point (x, y) appears in the eye image of a look from `pose`
    ({x, y, yaw_deg}): (u right, v down, 0..1 of the frame; outside 0..1 = outside the frame),
    or None behind the camera. cockpit_brains.floor_to_pixel, from the eye EYE_FWD_M ahead
    of the torso (the inverse of the box -> floor geometry the eye's answers use)."""
    yaw = math.radians(float(pose.get("yaw_deg", pose.get("yaw", 0.0)) or 0.0))
    dx = float(x) - (float(pose["x"]) + EYE_FWD_M * math.cos(yaw))
    dy = float(y) - (float(pose["y"]) + EYE_FWD_M * math.sin(yaw))
    fwd, left = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
    d = math.hypot(fwd, left)
    if d < 1e-6:
        return None
    return floor_to_pixel(d, -math.degrees(math.atan2(left, fwd)))


def look_hidden(look):
    """why(x, y) for one look: None when nothing this look SENSED stands between the eye and
    the floor point (x, y), else why it may be hidden —
      'hidden behind the <name>'  the point's image lies inside a box the eye drew in this look
                                  (look['eye'][name]: seen, with its bbox; widened PLACE_HIDE_PAD):
                                  a floor point inside a thing's box is BEHIND that thing's foot;
      'hidden (a lidar return in front)'  a return of the look's sweep (look['sweep']: body
                                  angles + ranges from the torso) within PLACE_HIDE_LIDAR_DEG of
                                  the point's bearing and PLACE_HIDE_LIDAR_M nearer than it.
    Both looks of a change check stand on one spot (the second turns in place), so what hides a
    spot from one hides it from the other: two 'not there' answers prove nothing there. Only
    what the look sensed can hide: an occluder the eye was not asked about and the lidar does
    not see (below its ~0.18 m plane) still counts as nothing (an upper bound on 'missing')."""
    pose = look.get("pose") or {}
    boxes = [(str(n), a["bbox"]) for n, a in sorted((look.get("eye") or {}).items())
             if isinstance(a, dict) and a.get("seen") is True and isinstance(a.get("bbox"), (list, tuple))
             and len(a["bbox"]) == 4]
    sweep = look.get("sweep")
    ang = rng = None
    if sweep is not None and len(sweep[0]):
        ang, rng = np.asarray(sweep[0], dtype=float), np.asarray(sweep[1], dtype=float)
    yaw = math.radians(float(pose.get("yaw_deg", 0.0) or 0.0))

    def why(x, y):
        if not pose:
            return None
        uv = eye_pixel(pose, x, y) if boxes else None
        if uv is not None:
            u, v = uv
            for n, (x1, y1, x2, y2) in boxes:
                if x1 - PLACE_HIDE_PAD <= u <= x2 + PLACE_HIDE_PAD and y1 - PLACE_HIDE_PAD <= v <= y2:
                    return f"hidden behind the {n}"
        if ang is not None:
            dx, dy = float(x) - float(pose["x"]), float(y) - float(pose["y"])
            d = math.hypot(dx, dy)
            th = math.atan2(dy, dx) - yaw
            off = np.abs((ang - th + np.pi) % (2 * np.pi) - np.pi)
            if np.any((off <= math.radians(PLACE_HIDE_LIDAR_DEG)) & (rng < d - PLACE_HIDE_LIDAR_M)):
                return "hidden (a lidar return in front)"
        return None
    return why


def look_view(look):
    """visible(x, y) for one look's change check: in the eye's view (eye_visible: 86 deg,
    0.15-2 m, from the look's pose) and not hidden behind anything the look sensed
    (look_hidden). A 'not there' counts only where this is True."""
    cone, hid = eye_visible(look["pose"]), look_hidden(look)
    return lambda x, y: bool(cone(x, y)) and hid(x, y) is None


def view_why(look, x, y):
    """Why a look could not see the floor point (x, y): 'out of view' | look_hidden's reason |
    None (it could)."""
    if not eye_visible(look["pose"])(x, y):
        return "out of view"
    return look_hidden(look)(x, y)


def boxed_unplaced(looks):
    """The names the eye boxed (a confident 'seen: true') in these GOOD looks without placing
    them in any of them (no floor spot within FIND_TRUSTED_M: beyond 2 m, above the horizon —
    on a table —, or no box). Sorted."""
    good = [lk for lk in looks or [] if lk.get("objects") is not None]
    boxed = {n for lk in good for n, a in (lk.get("eye") or {}).items() if a.get("seen") is True}
    placed = {n for lk in good for n in found_map(lk)}
    return sorted(n for n in boxed if not any(same_thing(n, p) for p in placed))


def unplaced_names(place):
    """The names a place's notes record as boxed there but never placed (PLACE_UNPLACED_NOTE),
    in the order recorded."""
    out = []
    for nt in (place or {}).get("notes") or []:
        t = str((nt or {}).get("text") or "")
        if t.startswith(PLACE_UNPLACED_NOTE):
            for n in t[len(PLACE_UNPLACED_NOTE):].split(","):
                n = norm_name(n)
                if n and n not in out:
                    out.append(n)
    return out


def _per_look_entry(r):
    """One look's own recognition for the answer's per_look: its verdict and confidence, the
    parts / evidence / signals (a list) it rests on, and its runner-up and ranking (with their
    parts when PlaceMemory.recognize gives them) — what a later replay of a two-look verdict
    under other thresholds needs."""
    parts = r.get("parts") if isinstance(r.get("parts"), dict) else None
    sig = r.get("signals")
    if not isinstance(sig, (list, tuple)):
        sig = [k for k in ("scan", "desc", "radio") if (parts or {}).get(k) is not None]
    out = {k: r.get(k) for k in ("verdict", "place_id", "name", "confidence", "evidence", "yaw_drift_deg")}
    out.update(parts=dict(parts) if parts else None, signals=list(sig),
               second=dict(r["second"]) if isinstance(r.get("second"), dict) else None,
               ranking=[dict(e) for e in r.get("ranking") or [] if isinstance(e, dict)])
    return out


def signal_list(rec):
    """The senses a verdict rests on, as a list (['scan', 'desc'], ...): the result's
    `signals`, else its non-None `parts`; [] for the operator's word or no parts."""
    rec = rec or {}
    if rec.get("by") == "operator":
        return []
    sig = rec.get("signals")
    if isinstance(sig, (list, tuple)):
        return [str(k) for k in sig]
    parts = rec.get("parts") if isinstance(rec.get("parts"), dict) else {}
    return [k for k in ("scan", "desc", "radio") if parts.get(k) is not None]


def combine_recognitions(r1, r2):
    """Two recognitions of one spot (the second after the PLACE_TURN_DEG turn) -> one, in
    PlaceMemory.recognize's shape: per place the mean of its two confidences (a place missing
    from one look's top 3 gets that look's lowest listed confidence: an upper bound for it,
    so a runner-up is never under-counted), the verdict by place_memory's own thresholds
    (KNOWN_T, MARGIN over a differently named runner-up, NEW_T). `per_look` keeps both;
    `signals` = the senses either look compared (each look's own are in per_look, with its
    parts, evidence, runner-up and ranking: _per_look_entry)."""
    names, confs = {}, {}
    for r in (r1, r2):
        for e in r.get("ranking") or []:
            names[e["place_id"]] = e["name"]
    for pid in names:
        cs = []
        for r in (r1, r2):
            rk = r.get("ranking") or []
            hit = next((e["confidence"] for e in rk if e["place_id"] == pid), None)
            if hit is None and rk:
                hit = min(e["confidence"] for e in rk)
            if hit is not None:
                cs.append(float(hit))
        confs[pid] = sum(cs) / len(cs) if cs else 0.0
    per_look = [_per_look_entry(r) for r in (r1, r2)]
    got = set()
    for r in (r1, r2):
        sig = r.get("signals")
        if not isinstance(sig, (list, tuple)):
            sig = [k for k in ("scan", "desc", "radio") if ((r.get("parts") or {}).get(k)) is not None]
        got.update(sig)
    signals = [k for k in ("scan", "desc", "radio") if k in got]
    if not confs:
        return dict(r2, per_look=per_look, signals=signals)
    order = sorted(confs, key=lambda k: confs[k], reverse=True)
    best = order[0]
    c = confs[best]
    sec = next((k for k in order[1:] if names[best] is None or names[k] != names[best]), None)
    c2 = confs[sec] if sec is not None else 0.0
    if c >= pm.KNOWN_T:
        verdict = "known" if c - c2 >= pm.MARGIN else "ambiguous"
    elif c < pm.NEW_T:
        verdict = "new"
    else:
        verdict = "ambiguous"
    base = r1 if r1.get("place_id") == best else r2
    return {"verdict": verdict, "place_id": best, "name": names[best], "confidence": round(c, 3),
            "second": ({"place_id": sec, "name": names[sec], "confidence": round(c2, 3)} if sec is not None
                       else {"place_id": None, "name": None, "confidence": 0.0}),
            "parts": base.get("parts"), "signals": signals, "evidence": base.get("evidence"),
            "yaw_drift_deg": base.get("yaw_drift_deg"),
            "ranking": [{"place_id": k, "name": names[k], "confidence": round(confs[k], 3)} for k in order[:3]],
            "per_look": per_look}


def apply_confirmed(stored, confirmed, match_m=PLACE_MATCH_M):
    """A place's object snapshot with the CONFIRMED changes applied (confirm_diff's added /
    missing / moved; pending ones never): the new snapshot to store. An addition the eye
    boxed but never placed (no x, y) is declared by the caller, not stored."""
    objs = [dict(o) for o in stored or []]

    def take(name, x, y):
        cand = [(math.hypot(float(o["x"]) - x, float(o["y"]) - y), i) for i, o in enumerate(objs)
                if same_thing(o["name"], name)]
        cand = [c for c in cand if c[0] <= match_m]
        return objs.pop(min(cand)[1]) if cand else None
    for m in (confirmed or {}).get("missing") or []:
        take(m["name"], float(m["x"]), float(m["y"]))
    for mv in (confirmed or {}).get("moved") or []:
        o = take(mv["name"], float(mv["from"][0]), float(mv["from"][1])) or {"name": mv["name"], "confidence": 0.2}
        objs.append(dict(o, x=float(mv["to"][0]), y=float(mv["to"][1])))
    for a in (confirmed or {}).get("added") or []:
        if a.get("x") is None or a.get("y") is None:
            continue                                     # boxed twice but never placed: declared, not stored
        objs.append({"name": a["name"], "x": float(a["x"]), "y": float(a["y"]),
                     "confidence": float(a.get("confidence") or 0.2)})
    return objs


def split_seen_before(confirmed):
    """A change check's confirmed diff (confirm_diff's {added, missing, moved, pending}) with
    the additions marked seen_before (a thing this place's earlier looks boxed but never
    placed, placed now: _place_eye_check) taken out of the changes: both looks agree ->
    'placed' (stored at its spot, never declared new here); one look only -> dropped from
    pending (one look placing it is no change either)."""
    c = dict(confirmed or {})
    c["placed"] = [dict(a) for a in c.get("added") or [] if a.get("seen_before")]
    c["added"] = [a for a in c.get("added") or [] if not a.get("seen_before")]
    c["pending"] = [o for o in c.get("pending") or [] if not (o.get("change") == "added" and o.get("seen_before"))]
    return c


def _mem_mark(mem):
    """A SceneMemory's state at a place bind, so an operator's correction can put the wrongly
    bound place's memory back (_mem_rollback). Reaches into SceneMemory's lock and tables:
    scene_memory has no public snapshot / restore (a small API there would be cleaner).
    `live` keeps the records present now alive, so their ids cannot be reused."""
    with mem._lock:
        return {"world": mem.world, "live": list(mem.obs),
                "state": copy.deepcopy((mem.obs, mem._objects, mem._seq, mem._u, mem._epoch, mem.session_t0))}


def _mem_rollback(mem, mark):
    """Put a SceneMemory back to `mark` (same world only) -> the records added since (copies,
    oldest first: the provisional memory's carried ones and everything after the bind), for
    the right place. The file is saved by the next debounced save / world switch."""
    with mem._lock:
        if mem.world != mark["world"]:
            return []
        old = {id(o) for o in mark["live"]}
        since = [dict(o) for o in mem.obs if id(o) not in old]
        obs, objs, seq, u, ep, t0 = copy.deepcopy(mark["state"])
        mem.obs, mem._objects = obs, objs
        mem._seq, mem._u, mem._epoch, mem.session_t0 = seq, u, ep, t0
        mem._changed()
    return since


def _pm_restore(pmem, pid, record):
    """Put a place's record back as it was (a PlaceMemory.get copy taken before a visit): the
    undo of a visit the operator says was to the wrong place. PlaceMemory has no public
    restore, so this reaches into its lock and table (a restore(pid, record) there would be
    cleaner). -> True when restored (False: the place is gone, or no record)."""
    lock, table = getattr(pmem, "_lock", None), getattr(pmem, "_places", None)
    if lock is None or not isinstance(table, dict) or not record:
        return False
    with lock:
        if str(pid) not in table:
            return False
        table[str(pid)] = copy.deepcopy(record)
        pmem._changed()
    return True


def _moved(a, b):
    """Has the robot moved (STILL_M / STILL_DEG) between two poses?"""
    return (math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"])) > STILL_M
            or abs((float(a["yaw_deg"]) - float(b["yaw_deg"]) + 180.0) % 360.0 - 180.0) > STILL_DEG)


def _changes(d):
    """Does a diff (diff_objects / confirm_diff) hold any change?"""
    return bool(d) and any(d.get(k) for k in ("added", "missing", "moved"))


_SIGNAL_WORDS = {"scan": "scan", "desc": "description", "radio": "radio"}


def signals_text(rec):
    """The senses a verdict rests on, in words: 'scan + description' | 'scan only' |
    'description only' | 'radio only' | 'the operator's word' | 'no places to compare with'
    (the first place) | 'no signal'. From the result's `signals` (PlaceMemory.recognize), else
    its non-None `parts`."""
    rec = rec or {}
    if rec.get("by") == "operator":
        return "the operator's word"
    sig = rec.get("signals")
    if not isinstance(sig, (list, tuple)):
        parts = rec.get("parts") if isinstance(rec.get("parts"), dict) else {}
        sig = [k for k in ("scan", "desc", "radio") if parts.get(k) is not None]
    words = [_SIGNAL_WORDS.get(k, str(k)) for k in sig]
    if not words:
        return "no places to compare with" if rec.get("verdict") == "new" and not rec.get("place_id") \
            else "no signal"
    return f"{words[0]} only" if len(words) == 1 else " + ".join(words)


def _is_fixture(name):
    n = norm_name(name)
    return bool(n) and n.split()[-1] in PLACE_FIXTURES


def place_spots(objects):
    """{name: [(x, y), ...]}: a place snapshot's remembered objects by (normalised) name, the
    room's fixtures (PLACE_FIXTURES) left out — the names its change check asks the eye about."""
    out = {}
    for o in objects or []:
        if not isinstance(o, dict):
            continue
        n, x, y = norm_name(o.get("name")), o.get("x"), o.get("y")
        if n and not _is_fixture(n) and x is not None and y is not None:
            out.setdefault(n, []).append((float(x), float(y)))
    return out


def place_candidates(text, exclude=()):
    """What a look description mentions that the eye can be asked about (a thing that may be
    new here): the LOOK_NOUNS (scene_memory) among place_memory.mentioned_names(text) — a negated
    mention ('no ball') is not one — minus the room's fixtures and anything same_thing to a name
    in `exclude`. The prose only proposes a question; the eye's box answers it. Sorted."""
    out = []
    for n in sorted(pm.mentioned_names(text or "")):
        head = n.split()[-1] if n else ""
        if head in LOOK_NOUNS and head not in PLACE_FIXTURES and not any(same_thing(n, e) for e in exclude):
            out.append(n)
    return out


def seen_map(look, spots):
    """One look's eye answers (look['eye'], filled by CockpitSim._place_ask) as the change
    check's seen map {name: True | False | None}, for the snapshot's names (spots: place_spots)
    and every name the look asked about:
      True   the eye boxed it in this look (a parsed 'seen: true', confidence >= FIND_MIN_CONF);
      False  the eye answered that it is not there — and, for a remembered object, at least one
             of its stored spots this look could see (look_view: in the eye's 86 deg x 0.15-2 m
             view from the look's pose, and not hidden behind a box the eye drew or a lidar
             return: look_hidden);
      None   nothing is claimed: the look failed (no description: every name), the question was
             not asked or got no usable answer (the model erred, the answer did not parse, it was
             unsure, the robot had moved), or a remembered object whose every stored spot is out
             of this look's view or hidden (unobservable: 'not seen' there proves nothing)."""
    eye = look.get("eye") or {}
    failed = look.get("objects") is None
    view = look_view(look) if not failed else None
    out = {}
    for n in list(spots) + [k for k in eye if k not in spots]:
        a = None if failed else (eye.get(n) or {}).get("seen")
        if a is False and spots.get(n) and not any(view(x, y) for x, y in spots[n]):
            a = None
        out[n] = a if a in (True, False) else None
    return out


def found_map(look):
    """{name: {x, y, confidence}}: the look's PLACED sightings (the box's bottom edge on the
    floor, cockpit_brains.sighting_to_map; a box beyond FIND_TRUSTED_M or above the horizon is
    seen but not placed). A failed look places nothing."""
    if look.get("objects") is None:
        return {}
    return {n: {"x": a["x"], "y": a["y"], "confidence": a.get("confidence", 0.5)}
            for n, a in (look.get("eye") or {}).items()
            if a.get("seen") is True and a.get("x") is not None and a.get("y") is not None}


def eye_objects(looks, match_m=PLACE_MATCH_M):
    """A new place's first objects from its looks' PLACED sightings (found_map), each kept once
    (same name within match_m: the first look's position). A sighting another look contradicts
    (that look could see the spot — look_view: in view, not hidden — and the eye said the thing
    is not there) is left out: the looks disagree, so it is not stored."""
    out = []
    for i, lk in enumerate(looks):
        for n, f in found_map(lk).items():
            if any(same_thing(n, o["name"]) and math.hypot(f["x"] - o["x"], f["y"] - o["y"]) <= match_m
                   for o in out):
                continue
            if any(j != i and other.get("objects") is not None
                   and ((other.get("eye") or {}).get(n) or {}).get("seen") is False
                   and look_view(other)(f["x"], f["y"]) for j, other in enumerate(looks)):
                continue
            out.append({"name": n, "x": f["x"], "y": f["y"], "confidence": f["confidence"]})
    return out


def looking_again_text(rec, why):
    """The place clause while the robot turns for its second look (<= place_memory.VERDICT_MAX):
    why 'new' | 'ambiguous' | '<n> change(s)' (a change check)."""
    rec = rec or {}
    c = float(rec.get("confidence") or 0.0)
    lab = str(rec.get("name") or rec.get("place_id") or "?")[:24]
    sec = rec.get("second") or {}
    if why == "new":
        s = (f"place: NEW? looking again (best {lab} {c:.2f})" if rec.get("place_id")
             else "place: NEW? looking again (no places yet)")
    elif why == "ambiguous":
        s = f"place: {lab}? looking again ({c:.2f}"
        s += (f", or {str(sec.get('name') or sec.get('place_id'))[:24]} {float(sec.get('confidence') or 0):.2f})"
              if sec.get("place_id") else ")")
    else:
        s = f"place: {lab} ({c:.2f}) — looking again to confirm {why}"
    return s[:pm.VERDICT_MAX]


def eye_brief(look, spots=None):
    """A look's eye check for the answer (the bench's per-visit record): {pose, seen, placed,
    asked, why} — seen = seen_map, placed = found_map, why = the reasons a name stayed None
    (a 'no' the look could not see behind or around says so: 'said no, but hidden behind the
    box' / 'said no, but out of view')."""
    eye = look.get("eye") or {}
    seen = seen_map(look, spots or {})
    why = {n: a["why"] for n, a in eye.items() if a.get("why")}
    for n, v in seen.items():
        if v is None and (eye.get(n) or {}).get("seen") is False and look.get("objects") is not None \
                and n not in why and (spots or {}).get(n):
            reasons = [view_why(look, x, y) for x, y in spots[n]]
            why[n] = "said no, but " + (next((r for r in reasons if r and r != "out of view"), None)
                                        or "out of view")
    return {"pose": {k: look["pose"].get(k) for k in ("x", "y", "yaw_deg")},
            "seen": seen, "placed": found_map(look),
            "asked": sum(1 for a in eye.values() if a.get("asked")), "why": why}


class SimDead(RuntimeError):
    """The sim thread has stopped (a fatal exception or quit); nothing it owns answers."""


class _Job:
    """One queued sim-thread call: runs fn and resolves the caller's future,
    or fails it (fatal path) so no HTTP request waits forever (D052)."""
    __slots__ = ("fn", "loop", "fut")

    def __init__(self, fn, loop, fut):
        self.fn, self.loop, self.fut = fn, loop, fut

    def _set(self, how, val):
        def cb():
            if not self.fut.done():
                getattr(self.fut, how)(val)
        try:
            self.loop.call_soon_threadsafe(cb)
        except RuntimeError:                     # the loop is closed: nobody is waiting
            pass

    def run(self):
        try:
            r = self.fn()
        except Exception as e:                   # surface, don't kill the sim
            self._set("set_exception", e)
            return e
        self._set("set_result", r)
        return None

    def fail(self, exc):
        self._set("set_exception", exc)


def _jsonable(x):
    """numpy scalars/arrays, tuples and int dict keys -> plain JSON types."""
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return _jsonable(x.tolist())
    if isinstance(x, (np.bool_,)):
        return bool(x)
    if isinstance(x, np.integer):
        return int(x)
    if isinstance(x, (float, np.floating)):
        f = float(x)
        return f if np.isfinite(f) else None
    return x


def _wrap_rad(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class EventLog(deque):
    """sim.events: the bounded events feed that also counts every append
    (`seq`, never reset in one cockpit's lifetime), so a client can tell a NEW
    event from a window that only looks the same — twelve identical ('say',
    'yes') entries before and after one more 'yes' (the brain bench's blind
    spot). The state feed carries it as `event_seq`. Appends come from the
    sim thread, the event loop and the hardware bridge's thread: the lock
    keeps `seq` and the contents in step, and tail() reads both at once."""

    def __init__(self, iterable=(), maxlen=None):        # deque's own signature (copy/pickle call it so)
        super().__init__(maxlen=maxlen)
        self._lock = threading.Lock()
        self.seq = 0
        self.extend(iterable)

    def append(self, x):
        with self._lock:
            super().append(x)
            self.seq += 1

    def extend(self, xs):
        for x in xs:
            self.append(x)

    def tail(self, n):
        """(seq, the newest n events), read together."""
        with self._lock:
            return self.seq, (list(self)[-n:] if n > 0 else [])


class CockpitSim(Playground):
    """The Playground with a goto controller, world swapping, offscreen
    cameras and an async job queue; every method that touches MjData
    runs in the sim thread (via `call`) unless noted."""

    def __init__(self, world="flat"):
        spec = dict(PRESETS.get(world, PRESETS["flat"])) if isinstance(world, str) else dict(world)
        model, z0 = build_world(spec)
        super().__init__(model=model, z0=z0)
        self.world_name = world if isinstance(world, str) else "custom"
        self.world_spec = spec
        self.fids = [self.model.geom(f"foot{i}").id for i in range(5)]
        self.pending = queue.Queue()
        self.speed = 1.0
        self.paused = False
        self.mode = "idle"
        self.events = EventLog(maxlen=200)  # counts every append: the feed's event_seq
        self.console = deque(maxlen=200)
        # scene memory: RAM only by default (tests, headless); main() turns persistence on.
        # Keyed by memory_key: a preset by its name, anything else by name + a hash of its spec
        self.memory = SceneMemory(memory_key(self.world_name, spec), directory=None, log=self.log)
        # D057: sim.memory is the ACTIVE scene memory; _mem_main the persisted one (per world, or per
        # recognised place while recognition is on); a provisional RAM-only one stands in until a verdict
        self._mem_main = self.memory
        self._mem_scratch = None
        self.place_memory = pm.PlaceMemory(directory=None, log=self.log)   # main() turns persistence on
        self.embed_fn = None                # text -> vector for place recognition (None: brains.embed)
        self._place = self._place_blank()
        self._hw_guard_t = {}               # bus event kind -> when it last went to the memory
        self.awareness = dict(interval_s=AWARENESS_S, curious=False, reactions=False, recognize=False)
        self.situation = dict(text="", t=None)
        self._pose_cache = None             # the last pose the sim thread saw (for other threads)
        self._aw = dict(check_next=0.0, compose_next=0.0, last_react=-1e9, last_curious=-1e9,
                        prev=None, prev_pose=None, pending=None, fails=0, curious_scene=None,
                        curious_task=None)
        self._aloop = None                  # the asyncio loop (set by call): curious looks run there
        self.goto_state = None
        self._stop_req = False
        self.frames = {"chase": (0, b""), "eye": (0, b"")}
        self._renderers = None
        self._cams_live = False             # a renderer was built once: the cameras work in this process
        self._render_due = False            # render on the next loop turn, paused or not (_forget_scene)
        self.cam = dict(distance=1.4, elevation=-24.0, azimuth=0.0)     # behind the robot, looking +x
        self.render_every = int(round(1 / (15 * self.DT)))      # ~15 fps
        self.last = dict(con=np.zeros(5, bool), tilt=0.0, height=0.0, gxy=0.0)
        self.rec = None                     # recording: dict(frames, t0, cmds, world)
        self.cmd_log = []                   # (sim t, line) — everything that drove the robot
        self.walk = None                    # residual walking policy: dict(policy, name, t_next, res)
        self.walk_note = "gait: analytic wave gait"
        self.brains = Brains(self)          # D052: roles, fallbacks, histories, voice, look
        self.browser_audio = True           # D051: chord words play in the browser (phone too), not on the server
        self.lexicon = lexicon()
        self.gesture_names = sorted(self.gestures)
        # D052: the sim thread's pulse + its death certificate
        self.last_step_wall = time.time()
        self.last_loop_wall = time.time()
        self.fatal = None                   # "Type: message" once the sim thread died
        self.exit_on_fatal = False          # main() sets True: a dead sim thread ends the process (exit 1)
        self.refusal = None                 # (wall time, text): the last command a guard refused (overlay)
        self.teach = None                   # the studio's pose-stream recorder (dict) while recording
        self.fingerprint = robot_fingerprint(self.model)
        self._caps_lock = threading.Lock()
        self._caps = None                   # D056: (inputs, capabilities snapshot), built on demand
        self._caps_seq = 0                  # version changes since start (the state feed's caps_seq)
        self._caps_refresh()                # the baseline version: a later change counts
        self.log(f"world: {self.world_name} | {self.righter_note} | robot {self.fingerprint}")

    def note(self, kind, msg):
        """Playground guard/gesture notes -> the events feed + console (D052)."""
        super().note(kind, msg)
        ev = getattr(self, "events", None)
        if ev is None:                      # during Playground.__init__
            return
        ev.append((kind, str(msg)))
        self.log(f"{kind}: {msg}")
        if kind in ("void", "latch") or str(msg).startswith(("refused", "blocked")):
            self.refusal = (time.time(), f"{kind}: {msg}")
        if kind in ("void", "latch", "thermal"):
            self._remember_guard(f"{kind}: {msg}", alarm=kind in ("void", "latch"))

    def _remember_guard(self, text, alarm=False):
        """A guard fired: a 'guard' observation in the scene memory, and (void /
        latch / a bus cut) a pending alarm reaction for the awareness tick.
        Any thread; never raises (the sim thread calls it from inside step)."""
        mem = getattr(self, "memory", None)
        if mem is not None:
            try:
                mem.remember({"kind": "guard", "text": str(text), "pose": self._pose_cache})
            except Exception:                                   # noqa: BLE001
                pass
        if alarm and hasattr(self, "_aw"):
            self._aw["pending"] = ("alarm_help", str(text)[:80])

    def _hw_event(self, kind, msg):
        """The bridge's on_event (its own thread: append + log + memory only, no MjData).
        A bus guard goes to the memory at most once per kind per HW_GUARD_EVERY_S
        (a streaming fault must not push the look history out of the 500 entries,
        nor rewrite the file every debounce); an alarm kind still raises the
        alarm reaction every time (that one is rate-limited by REACT_MIN_S)."""
        self.events.append(("hw:" + kind, msg))
        self.log(f"hw {kind}: {msg}")
        if kind in HW_ROUTINE:
            return
        now = time.monotonic()
        last = getattr(self, "_hw_guard_t", {})
        if now - last.get(kind, -1e9) < HW_GUARD_EVERY_S:
            if kind in HW_ALARM and hasattr(self, "_aw"):
                self._aw["pending"] = ("alarm_help", f"bus {kind}: {msg}"[:80])
            return
        last[kind] = now
        self._remember_guard(f"bus {kind}: {msg}", alarm=kind in HW_ALARM)

    def note_refusal(self, reply):
        """A console/teleop reply that is a guard's refusal -> the overlay."""
        r = str(reply or "")
        body = r[len("[teleop] "):] if r.startswith("[teleop] ") else r
        if body.startswith(("blocked", "locomotion held", "busy", "reflex is", "a safe-stop",
                            "a goto is running", "wave refused", "gait presets change")):
            self.refusal = (time.time(), body)

    def idle_reason(self):
        """None when the sim is at a planted standstill (what sim2real and the
        studio's teach need), else why not — readable, for the UI."""
        if self.sup.state != "NORMAL":
            return f"reflex state is {self.sup.state}"
        if np.any(self.cmd_v):
            return "the sim is walking (velocity command) — stop first"
        why = self.motion_reason()           # a void retreat / pending stop / gate hold (review fix)
        if why:
            return why
        if self.goto_state is not None:
            return "a goto is running — stop first"
        if self.gesture is not None or self._ges is not None:
            return "a gesture or studio pose is running — release it first"
        return None

    def is_idle(self):
        """The Playground's standstill rule, plus (D052 V2) real-time speed:
        the sim2real stream is paced by the sim loop, so at 8x the real legs
        would get targets 8x faster than the sim's own 4.7 rad/s clamp."""
        return super().is_idle() and self.speed == 1.0 and not self.paused

    def sim2real_refusal(self):
        """None when sim2real may start, else why (the UI shows it)."""
        why = self.idle_reason()
        if why:
            return why
        hot = self.thermal_status()["tripped"]
        if hot:                              # review fix: the sim's servos are past their thermal budget
            return (f"{', '.join(hot)} past the thermal budget in the sim (the real servo would be at "
                    f"its over-temp cut) — let it cool")
        if self.speed != 1.0:
            return f"the sim runs at {self.speed:g}x — set speed 1x first (the stream is paced by the sim)"
        if self.paused:
            return "the sim is paused"
        return None

    def set_speed(self, speed):
        """Any thread. D052 V2: finite only (NaN used to stick and unthrottle
        the loop for good) and 1x only while mirroring sim->real."""
        v = float(speed)
        if not np.isfinite(v):
            raise ValueError("speed must be a finite number")
        v = float(np.clip(v, 0.1, 8.0))
        if v != 1.0 and self.sim2real:
            raise ValueError("speed is locked at 1x while mirroring sim->real (the stream is paced by the sim)")
        self.speed = v
        return v

    @property
    def brain(self):
        """The brain state dict (mode + role models) — live: writes stick."""
        return self.brains.state

    @property
    def chat_hist(self):
        return self.brains.hist

    @property
    def llm_base(self):
        return self.brains.base

    def reload_library(self):
        """Any thread: re-read keyframe gestures + chord words from disk (D056: a
        changed list is a new capabilities version: the state feed's caps_version)."""
        self.gestures = all_gestures()
        self.gesture_names = sorted(self.gestures)
        self.lexicon = lexicon()
        self._caps_refresh()

    def apply_gait(self, **kw):
        """Playground.apply_gait; then (D056) a new envelope is a new capabilities version."""
        out = super().apply_gait(**kw)
        if getattr(self, "_caps_lock", None) is not None:
            self._caps_refresh()
        return out

    # ------------------------------------------------------ D056: capabilities
    def _caps_envelope(self):
        """The registry's envelope with THIS cockpit's live gait in it (a gait preset
        or a walker's training gait changes the speed, the turn rate and the step);
        the registry's own numbers when the gait cannot say. Never raises, never warns
        (the state feed reads the version, so a bad gait that slipped past validation,
        T = 0, is rebuilt here: its non-finite numbers are refused below, not printed)."""
        env = tool_registry.default_envelope()
        try:
            g = self.gait
            with np.errstate(all="ignore"):
                mc = g.max_command()
            v, wz, hstep = float(mc["v"]), float(mc["wz"]), float(g.hstep)
            if not np.isfinite([v, wz, hstep]).all():
                raise ValueError(f"non-finite envelope v={v} wz={wz} hstep={hstep}")
            env.update(source="the cockpit's live gait: sim.gait.max_command() (WaveGait budget, "
                              "cad/params.yaml)",
                       goto_timeout_s=GOTO_CAP_S, goto_speed_m_s=round(min(V_GOTO, v) / 1000.0, 4),
                       speed_m_s=round(v / 1000.0, 4), turn_rad_s=round(wz, 3), step_height_mm=round(hstep, 1))
        except Exception:                                       # noqa: BLE001 — keep the registry's numbers
            pass
        return env

    def _caps_current(self):
        """Any thread. The cached capabilities snapshot (never mutate it: capabilities()
        hands out copies), rebuilt when an input changed — the gesture or chord-word
        list, the gait, the eye or the memory. A rebuild with a new version bumps
        _caps_seq and logs one console line, once per change — never an events-feed
        entry (the status tool returns that feed's tail to the models). Place recognition
        switched on or off is such a change (D057: the place tools come and go with it)."""
        g = self.gait
        # D057: the place tools exist (the registry's `places`) only while recognition is on, so
        # switching it changes the version (caps_seq counts it) and, off, the snapshot is the D056 one
        flags = dict(cockpit_flags(self), has_places=bool((self.awareness or {}).get("recognize")))
        key = (tuple(self.gesture_names), tuple(self.lexicon), tuple(SIGNED),
               tuple(float(getattr(g, k)) for k in GAIT_KEYS), tuple(sorted(flags.items())))
        with self._caps_lock:
            last = self._caps
            if last is not None and last[0] == key:
                return last[1]
            caps = tool_registry.build(list(key[0]), list(key[1]), list(SIGNED),
                                       envelope=self._caps_envelope(), **flags)
            self._caps = (key, caps)
            if last is not None and last[1]["version"] != caps["version"]:
                self._caps_seq += 1
                self.log(f"capabilities: {last[1]['version']} -> {caps['version']}")
            return caps

    def _caps_refresh(self):
        """_caps_current for its count; never raises (a gesture save must not fail over it)."""
        try:
            self._caps_current()
        except Exception as e:                                  # noqa: BLE001
            self.log(f"capabilities: not rebuilt ({type(e).__name__}: {e})")

    def _caps_brief(self):
        """Any thread. (caps_seq, caps_version) for the state feed: the version GET
        /api/capabilities answers now and how many times it changed since start, read
        together. Never raises and never logs (it runs on every state read, up to
        10 Hz per feed; reload_library / apply_gait log a failed rebuild): the last
        built version when a rebuild fails, None before the first."""
        try:
            self._caps_current()
        except Exception:                                       # noqa: BLE001 — the feed must not die
            pass
        with self._caps_lock:
            last = self._caps
            return self._caps_seq, (None if last is None else last[1]["version"])

    def capabilities(self):
        """Any thread. What this cockpit can do right now (GET /api/capabilities), as
        harness.capabilities.build makes it: {format, version, capabilities, gestures,
        signed, lexicon, envelope, robot, tools} — a copy."""
        return copy.deepcopy(self._caps_current())

    # ------------------------------------------------------ D051: hardware
    def hw_connect(self, port):
        """Any thread. Open the bus (or the mock); the sim thread mirrors from the next step."""
        self.hw_disconnect()
        # D052: is_idle gates sim2real (the Playground also ANDs its own is_idle in);
        # on_event must not block — append + log only
        hw = HardwareBridge(port, on_event=self._hw_event, is_idle=self.is_idle)
        hw.speed_cps = hw_bridge.ENTRY_SPEED_CPS    # the stream-speed slider starts gentle (200 c/s), not servo max
        self.hw = hw
        return hw.status()

    def hw_disconnect(self):
        hw, self.hw = self.hw, None
        if hw is not None:
            hw.close()
            self.log(f"hw: disconnected {hw.port}")
        return hw is not None

    # ------------------------------------------------------ D051: gesture studio
    def preview_pose(self, spec, t=None):
        """Sim thread: hold the keyframe gesture's pose at time t (None = the
        last AUTHORED frame) until preview_off / a gesture / a walk. D052: the
        Playground blends into it (no snap) and the same standstill rule as a
        gesture applies — a walking robot is refused, not stopped mid-stride."""
        kg = KeyframeGesture(spec)
        tt = kg.authored_total if t is None else float(t)
        why = self.gesture_refusal(switching=self._ges is not None and self._ges["phase"] != "out")
        if why:
            raise ValueError(f"preview refused: {why}")
        fn = lambda g, _t, kg=kg, tt=tt: kg(g, tt)            # noqa: E731
        with self.lock:
            self.gesture = (fn, float("inf"), self.t)
        self._ges_name = "studio pose"
        self.mode = "posing"
        return kg.total

    def preview_off(self):
        self.end_gesture()                     # blended out to the stance (D052)
        self.mode = "idle"

    # ------------------------------------------------------ D052: teach recorder
    def teach_start(self):
        """Sim thread. Record the sim's joint targets (ctrl) at TEACH_HZ — or,
        with the bridge in real2sim, the real legs' measured q — for the studio."""
        self.teach = dict(qs=[], claw=[], t0=self.t, t_next=self.t, src="sim ctrl")
        return {"ok": True, "recording": True, "hz": TEACH_HZ, "max_s": TEACH_MAX_S}

    def _teach_sample(self):
        tr = self.teach
        if self.t < tr["t_next"]:
            return
        tr["t_next"] += 1.0 / TEACH_HZ
        q = np.array(self.data.ctrl[:15], float).reshape(5, 3)
        hw = self.hw
        if hw is not None and getattr(hw, "mirror", "off") == "real2sim":
            q_real, legs = hw.real_pose()
            for i in range(5):
                if legs[i]:
                    q[i] = q_real[i]
            tr["src"] = "real legs (real2sim) + sim ctrl"
        tr["qs"].append(q.ravel().copy())
        tr["claw"].append(np.array(self.data.ctrl[15:20], float) if self.model.nu >= 20 else np.zeros(5))
        if len(tr["qs"]) >= int(TEACH_MAX_S * TEACH_HZ):
            tr["full"] = True

    def teach_stop(self):
        """Sim thread: end the recording; returns (qs (T,15), claw (T,5), meta)."""
        tr, self.teach = self.teach, None
        if tr is None:
            return None
        return (np.array(tr["qs"]).reshape(-1, 15), np.array(tr["claw"]).reshape(-1, 5),
                dict(src=tr["src"], samples=len(tr["qs"]), seconds=round(self.t - tr["t0"], 2)))

    # ---------------------------------------------------------- plumbing
    def log(self, line):
        self.console.append((round(self.t, 2), str(line)))

    def note_cmd(self, line):
        """Everything that moved the robot goes here (recordings replay it)."""
        self.cmd_log.append((round(self.t, 3), line))
        if self.rec is not None:
            self.rec["cmds"].append((round(self.t - self.rec["t0"], 3), line))

    async def call(self, fn):
        """Run fn() in the sim thread; await its result. Raises SimDead when the
        sim thread is gone (D052: before, a dead thread hung every request)."""
        if not self.alive:
            raise SimDead(self.fatal or "the sim thread has stopped")
        loop = asyncio.get_running_loop()
        self._aloop = loop                       # the awareness tick schedules curious looks on it
        fut = loop.create_future()
        self.pending.put(_Job(fn, loop, fut))
        while True:
            try:
                return await asyncio.wait_for(asyncio.shield(fut), 1.0)
            except asyncio.TimeoutError:
                if not self.alive and not fut.done():
                    raise SimDead(self.fatal or "the sim thread has stopped") from None

    def _drain(self):
        while True:
            try:
                job = self.pending.get_nowait()
            except queue.Empty:
                return
            e = job.run()
            if e is not None:
                self.log(f"job failed: {type(e).__name__}: {e}")

    def flush_memories(self):
        """Any thread: write the debounced saves now — the scene memory (the persisted one and
        the active one) and the place memory (D057). Never raises."""
        for m in (self._mem_main, self.memory, self.place_memory):
            try:
                m.flush()
            except Exception as e:                       # noqa: BLE001
                self.log(f"memory flush: {type(e).__name__}: {e}")

    def heartbeat(self):
        """Any thread: the sim thread's pulse (wall clock)."""
        now = time.time()
        return dict(alive=bool(self.alive), fatal=self.fatal, last_step_wall=round(self.last_step_wall, 3),
                    step_age_s=round(now - self.last_step_wall, 2), loop_age_s=round(now - self.last_loop_wall, 2),
                    paused=bool(self.paused), stale_after_s=HEARTBEAT_STALE_S)

    # ---------------------------------------------------------- the loop
    def run_forever(self):
        """The sim thread. D052: an exception out of step() is FATAL (the physics
        state is unknown): _fatal limps the real legs, fails every waiting
        request and — under main() — ends the process with exit code 1."""
        t_wall = time.monotonic()
        try:
            while self.alive:
                self.last_loop_wall = time.time()
                self._drain()
                if self.paused:
                    if self._render_due:                 # a world load / reset while paused: show it
                        self._render_safe()
                    time.sleep(0.02)
                    t_wall = time.monotonic()
                    continue
                self.step()
                self.last_step_wall = time.time()
                t_wall += self.DT / max(self.speed, 0.05)
                if not np.isfinite(t_wall):              # D052 V2: a NaN here never slept again
                    t_wall = time.monotonic()
                lag = t_wall - time.monotonic()
                if lag > 0:
                    time.sleep(lag)
                elif lag < -0.05:
                    t_wall = time.monotonic()
        except BaseException as e:                       # noqa: BLE001 — any death is reported
            self._fatal(e)
            return
        self._fail_pending(SimDead("the sim thread has stopped (quit)"))

    def _fail_pending(self, exc):
        while True:
            try:
                job = self.pending.get_nowait()
            except queue.Empty:
                break
            job.fail(exc)
        gs, self.goto_state = self.goto_state, None
        if gs is not None and not gs["fut"].done():
            try:
                gs["loop"].call_soon_threadsafe(
                    lambda f=gs["fut"]: f.done() or f.set_exception(exc))
            except RuntimeError:
                pass

    def _fatal(self, e):
        self.fatal = f"{type(e).__name__}: {e}"
        self.alive = False
        self.log(f"FATAL in the sim thread: {self.fatal} — real legs limped, requests failed")
        print(f"cockpit: FATAL in the sim thread:\n{traceback.format_exc()}", file=sys.stderr, flush=True)
        try:
            if self.hw is not None:
                self.hw_disconnect()                    # close() = stop the stream + limp every servo
        except Exception as e2:                          # noqa: BLE001
            print(f"cockpit: limp on fatal failed: {e2}", file=sys.stderr, flush=True)
        self._fail_pending(SimDead(f"sim thread died: {self.fatal}"))
        if self.exit_on_fatal:
            time.sleep(1.0)                              # let the failed requests answer first
            os._exit(1)

    def step(self):
        gs = self.goto_state
        if gs is not None:
            self._goto_pre(gs)
        if self.walk is not None:
            self._walk_residual()               # sets self.residual for this step's targets
        super().step()
        if gs is not None and self.goto_state is gs:
            self._goto_post(gs)
        if self.teach is not None:
            self._teach_sample()
        a = self.awareness
        if a["interval_s"] > 0 or a["reactions"] or a["curious"]:
            try:
                self._aware_tick()
            except Exception as e:                       # noqa: BLE001 — awareness is not physics
                self._aware_failed(e)
        if a.get("recognize"):
            try:
                self._place_tick()
            except Exception as e:                       # noqa: BLE001 — recognition is not physics
                self._place_tick_failed(e)
        if self._render_due or self._k % self.render_every == 0:
            self._render_safe()

    # ------------------------------------------------- residual walker
    def _walk_residual(self):
        """D050: the PPO residual gait policy on top of the supervisor's output
        while the reflex state is NORMAL. D052: the observation is built by
        rl_common's builder for the checkpoint's OWN obs version (v2: SimIMU
        gravity/gyro, quantised one-tick-late encoders, a_prev; v1: the legacy
        true-state vector), the command is the one the gait actually got
        (cmd_eff, after the budget) and the action goes through the
        checkpoint's EMA — exactly rocky_env.PebbleEnv.step."""
        w = self.walk
        if self.sup.state != "NORMAL" or self.gesture_busy:
            self.residual = None
            return
        if self.t >= w["t_next"]:
            w["t_next"] = self.t + rc.CTRL_DT
            ph = 2 * np.pi * ((self.sup.t_gait / self.gait.T) % 1.0)
            cmd_n = np.asarray(self.cmd_eff, float) / np.array([60.0, 60.0, 0.6])
            obs = w["obs"](self.data, self.t, ph, cmd_n, w["a_f"])
            a = np.clip(np.asarray(w["policy"](obs), float), -1, 1)
            w["a_f"] = w["ema"] * a + (1.0 - w["ema"]) * w["a_f"]
            w["res"] = w["act_scale"] * w["a_f"]
        self.residual = w["res"]

    def set_walk(self, name, _keep_gait=None):
        """Sim thread. name: 'off' or a runs/NAME with a gait checkpoint. D052:
        the checkpoint's contract decides the obs builder, EMA and the GAIT it
        trained on (pre-D052 walkers: T 1.6 s, step 32 mm — the phase in the obs
        means nothing on another T), which is applied here and restored on 'off'."""
        self.residual = None
        prev = self.walk
        if name in (None, "", "off", "analytic"):
            self.walk = None
            note = ""
            if prev is not None and prev.get("gait_prev") and self.idle_reason() is None:
                try:
                    self.apply_gait(**prev["gait_prev"])
                except ValueError as e:                  # D052 V2: validated (e.g. sim2real is on)
                    note = f" (gait not restored: {e})"
            self.walk_note = "gait: analytic wave gait" + note
            return self.walk_note
        path = os.path.join(HERE, "runs", name, "latest.pt")
        if not os.path.exists(path):
            return f"no runs/{name}/latest.pt"
        try:
            import torch
            from train_ppo import Agent, RunningMeanStd
            ck = torch.load(path, map_location="cpu", weights_only=False)
            contract = rc.checkpoint_contract(ck, env_hint="gait")
            rc.check_obs_contract(contract, "gait")         # ValueError: not a gait checkpoint we can feed
            obs_dim = int(contract["obs_dim"])
            agent = Agent(obs_dim, int(ck.get("act_dim", 15)))
            agent.load_state_dict(ck["model"])
            agent.eval()
            rms = RunningMeanStd((obs_dim,))
            rms.load_state_dict(ck["obs_rms"])

            def policy(obs):
                on = (obs - rms.mean) / np.sqrt(rms.var + 1e-8)
                with torch.no_grad():
                    return agent.actor(torch.as_tensor(np.clip(on, -10, 10), dtype=torch.float32)
                                       .unsqueeze(0)).squeeze(0).numpy()
            jadr, vadr = rc.joint_addrs(self.model)
            obs = rc.make_gait_obs(int(contract["obs_version"]), self.model, self.torso, jadr, vadr,
                                   rc.foot_geoms(self.model))
            obs.reset(noise=False)
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                fp_status = rc.check_fingerprint(contract, self.model, who=f"walker {name}")
        except Exception as e:
            return f"walk policy {name}: {type(e).__name__}: {e}"
        # the gait it trained on
        gp = contract.get("gait") or {}
        want = {"T": gp.get("cycle_time"), "h": gp.get("body_height"), "R0": gp.get("stance_radius"),
                "duty": gp.get("duty"), "hstep": gp.get("step_height")}
        want = {k: float(v) for k, v in want.items() if v is not None}
        cur = {k: float(getattr(self.gait, k)) for k in GAIT_KEYS}
        gait_prev = _keep_gait if _keep_gait is not None else (prev.get("gait_prev") if prev else None)
        gait_note = ""
        if any(abs(cur[k] - v) > 1e-6 for k, v in want.items()):
            why = self.idle_reason()
            if why:
                return f"walk policy {name} trained on gait {want}; switching the gait needs a standstill: {why}"
            gait_prev = gait_prev or cur
            try:
                self.apply_gait(**want)
            except ValueError as e:
                return f"walk policy {name}: its training gait was refused: {e}"
            gait_note = " | gait set to its training gait " + " ".join(f"{k}={v:g}" for k, v in want.items())
            if self.gait.max_command()["v"] < 1.0:
                # measured 2026-09-24: every pre-D052 walker trained on T 1.6 s / step 32 mm, whose
                # swing lifts the foot faster than the servo budget allows -> WaveGait.budget zeroes
                # every command. Honest answer: this walker cannot walk on the D052 servo.
                gait_note += (" — WARNING: that gait has NO speed envelope under the D052 servo budget "
                              "(its step lifts faster than the servo): every command is zeroed; retrain "
                              "the walker on the D052 env")
        self.walk = dict(policy=policy, name=name, t_next=self.t, res=np.zeros(15), a_f=np.zeros(15),
                         obs=obs, ema=float(contract.get("ema_alpha", 1.0)),
                         act_scale=float(contract.get("act_scale", 0.25)), contract=contract,
                         fp_status=fp_status, gait_prev=gait_prev)
        flags = contract.get("flags") or []
        self.walk_note = (f"gait: analytic + residual policy {name} ({ck.get('global_step', 0):,} steps, "
                          f"obs v{contract['obs_version']}, ema {self.walk['ema']:g}"
                          + (f", {', '.join(flags)}" if flags else "")
                          + (f", ROBOT CHANGED since training ({contract.get('robot_fingerprint')} vs "
                             f"{self.fingerprint})" if fp_status == "mismatch" else "")
                          + ")" + gait_note)
        for w in caught:
            self.log(f"walker: {w.message}")
        return self.walk_note

    # ------------------------------------------------------------- goto
    def _cone_block(self, angles, ranges, heading):
        """(range m, body bearing deg) of the nearest lidar return within
        +-GOTO_CONE_DEG of the body-frame heading (rad) closer than
        GOTO_CLEAR_M, else None."""
        near = np.isfinite(ranges) & (ranges < GOTO_CLEAR_M) & \
            (np.abs(_wrap_rad(angles - heading)) <= np.radians(GOTO_CONE_DEG))
        if not near.any():
            return None
        i = int(np.argmin(np.where(near, ranges, np.inf)))
        return float(ranges[i]), float(np.degrees(angles[i]))

    def _cone_min(self, angles, ranges, heading):
        m = np.isfinite(ranges) & (np.abs(_wrap_rad(angles - heading)) <= np.radians(GOTO_CONE_DEG))
        return float(ranges[m].min()) if m.any() else float("inf")

    @staticmethod
    def _corridor_clear(angles, ranges, heading, length):
        """No return inside the robot-wide strip (+-GOTO_HALF_W) ahead along the
        heading for `length` m. Stricter than the cone at the side: this is what
        ends a detour (the cone alone would let a leg clip the wall's end)."""
        f = np.isfinite(ranges)
        rel = _wrap_rad(angles[f] - heading)
        along, lat = ranges[f] * np.cos(rel), ranges[f] * np.sin(rel)
        return not bool(np.any((along > 0) & (along < length) & (np.abs(lat) < GOTO_HALF_W)))

    def _goto_react(self, gs, tw, hd_goal, dist):
        """D052 reactive layer (8 Hz): something within +-30 deg of where the
        robot is walking and closer than 0.35 m -> detour (1: +-45 deg off the
        target heading, toward the clearer side; 2: a sidestep, 90 deg, same
        side). A detour lasts until the robot-wide corridor toward the target
        has been clear for GOTO_RESUME_S (or GOTO_DETOUR_S at most); blocked a
        third time -> stopped='blocked'. The puck sees only what stands above
        its plane (~torso + 60 mm): lower things are left to the 3 s
        no-progress rule ('stuck')."""
        dt_ = gs.get("detour")
        if dt_ is not None and self.t >= dt_["until"]:
            gs["detour"] = None
        if gs["tries"] and gs["best"] < gs["best_at_detour"] - GOTO_DETOUR_RESET_M:
            gs["tries"] = 0                                  # made real progress: detours earned back
            gs["side"] = None
        if self._k % max(1, int(round(1.0 / (GOTO_SCAN_HZ * self.DT)))) != 0:
            return
        angles, ranges, _ = lidar_scan(self.model, self.data, self.torso)
        dt_ = gs.get("detour")
        if dt_ is not None:
            if self._corridor_clear(angles, ranges, hd_goal, min(dist + 0.05, GOTO_CLEAR_M + GOTO_HALF_W)):
                dt_["clear_since"] = dt_.get("clear_since") or self.t
                if self.t - dt_["clear_since"] >= GOTO_RESUME_S:
                    gs["detour"] = None
                    self.log("goto: way toward the target is clear — back on course")
            else:
                dt_["clear_since"] = None
        off = gs["detour"]["offset"] if gs.get("detour") else 0.0
        hit = self._cone_block(angles, ranges, hd_goal + off)
        if hit is None:
            return
        rng_m, bear = hit
        if gs["tries"] >= GOTO_DETOURS:
            gs["outcome"] = ("blocked", tw)
            gs["block"] = dict(range_m=round(rng_m, 3), bearing_deg=round(bear, 1), detours=gs["tries"])
            self.sup.request_stop()
            self.log(f"goto blocked: obstacle {rng_m:.2f} m at {bear:.0f} deg (body) after {gs['tries']} detours")
            return
        gs["tries"] += 1
        mag = np.radians(45.0 * gs["tries"])                 # 45 deg, then a 90 deg sidestep
        side = gs.get("side")
        if side is None:                                     # pick the clearer side once, then commit
            left, right = self._cone_min(angles, ranges, hd_goal + mag), self._cone_min(angles, ranges, hd_goal - mag)
            side = gs["side"] = 1.0 if left >= right else -1.0
        gs["detour"] = dict(offset=side * mag, until=self.t + GOTO_DETOUR_S)
        gs["best_t"] = tw                                    # a fresh no-progress window for the detour
        gs["best_at_detour"] = gs["best"]
        self.log(f"goto: obstacle {rng_m:.2f} m at {bear:.0f} deg — detour {gs['tries']}/{GOTO_DETOURS}: "
                 f"{'sidestep' if gs['tries'] == 2 else 'heading'} {np.degrees(side * mag):+.0f} deg")

    def _goto_pre(self, gs):
        tw = self.t - gs["t0"]
        p = self.data.xpos[self.torso]
        dx, dy = gs["tx"] - p[0], gs["ty"] - p[1]
        dist = float(np.hypot(dx, dy))
        v = self.void
        if gs["outcome"] is None and v is not None and v["t"] >= gs["t0"] - 1e-9:
            # D052: the Playground's always-on void guard fired during this goto. It
            # backs off and safe-stops by itself; the goto ends as 'cliff' once it has.
            gs["void"] = dict(v)
            if self._void_phase != "retreat":
                gs["outcome"] = ("cliff", tw)
        if gs["outcome"] is None and gs.get("void") is None:
            if dist < 0.025:
                gs["outcome"] = ("arrived", tw)
                self.sup.request_stop()
            elif self.sup.latched:
                gs["outcome"] = ("blocked", tw)
                gs["block"] = dict(detail=f"latched safe-stop ({self.sup.latch_reason}) — `clear` to release")
            elif self.locomotion_held():
                gs["outcome"] = ("blocked", tw)
                gs["block"] = dict(detail="locomotion held: the real legs are mirroring (sim2real)")
            elif dist < gs["best"] - 0.02:
                gs["best"], gs["best_t"] = dist, tw
            elif gs.get("detour") is not None:
                gs["best_t"] = tw                    # a sidestep makes no progress by design; the
                #                                      detour has its own time limit
            elif tw - gs["best_t"] > GOTO_STUCK_S and tw > 2.0:
                gs["outcome"] = ("stuck", tw)          # D050: blocked, not a void
                self.sup.request_stop()
                self.log(f"goto stuck {dist*100:.0f} cm short (no progress for {GOTO_STUCK_S:.0f} s)")
            elif tw > GOTO_CAP_S:
                gs["outcome"] = ("timeout", tw)
                self.sup.request_stop()
        if self._stop_req and gs["outcome"] is None:
            gs["outcome"] = ("user", tw)
            self.sup.request_stop()
        self._stop_req = False
        vx = vy = wz = 0.0
        if gs["outcome"] is None and gs.get("void") is None:
            ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0.0, 0.0)
            yaw = self.yaw()
            cy, sy = np.cos(yaw), np.sin(yaw)
            bx, by = cy * ux + sy * uy, -sy * ux + cy * uy     # target direction, body frame
            hd_goal = float(np.arctan2(by, bx))
            self._goto_react(gs, tw, hd_goal, dist)
            if gs["outcome"] is None:
                hd = hd_goal + (gs["detour"]["offset"] if gs.get("detour") else 0.0)
                ramp = min(tw / 0.6, 1.0)
                vx, vy = V_GOTO * ramp * np.cos(hd), V_GOTO * ramp * np.sin(hd)
        with self.lock:
            self.cmd_v[:] = [vx, vy, wz]

    def _goto_post(self, gs):
        tw = self.t - gs["t0"]
        if gs["outcome"] is None and self.last["tilt"] > 60:
            gs["outcome"] = ("FELL", tw)
        if gs["outcome"] is not None and tw > gs["outcome"][1] + 1.5:
            reason = gs["outcome"][0]
            res = {"ok": reason == "arrived", "stopped": reason, "pose": self.pose()}
            if reason == "cliff":
                v = gs.get("void") or {}
                res["detail"] = (f"VOID at {v.get('bearing_deg', float('nan')):.0f} deg (leg {v.get('leg')}) — "
                                 "the always-on void guard backed off and safe-stopped; `clear` releases it")
                res["void"] = _jsonable({k: v.get(k) for k in ("bearing_deg", "world_bearing_deg", "leg")})
            if reason == "FELL":
                res["detail"] = "the robot fell during the goto; the righter takes over"
            if reason == "stuck":
                res["detail"] = ("no progress toward the target for 3 s: something is in the way that the "
                                 "lidar cannot see (lower than the puck plane) — not a void")
            if reason == "blocked":
                b = gs.get("block") or {}
                if "range_m" in b:
                    res["detail"] = (f"obstacle {b['range_m']:.2f} m away at {b['bearing_deg']:.0f} deg "
                                     f"(body frame) — {b['detours']} detours tried; pick another target")
                    res["obstacle"] = b
                else:
                    res["detail"] = b.get("detail", "a guard stopped the goto")
            self.mode = "idle" if reason == "arrived" else "safe_stop"
            self.goto_state = None
            with self.lock:
                self.cmd_v[:] = 0
            self._resolve(gs, res)

    @staticmethod
    def _resolve(gs, res):
        fut, loop = gs["fut"], gs["loop"]
        if not fut.done():
            loop.call_soon_threadsafe(fut.set_result, res)

    def _start_goto(self, tx, ty, fut, loop):
        if self.goto_state is not None:
            self._resolve(self.goto_state, {"ok": False, "stopped": "preempted", "pose": self.pose()})
        self.goto_state = dict(tx=float(tx), ty=float(ty), t0=self.t, outcome=None, fut=fut, loop=loop,
                               best=float("inf"), best_t=0.0, detour=None, tries=0,
                               best_at_detour=float("inf"), void=None, block=None, side=None)
        self.mode = "walking"
        self.events.append(("goto", (round(float(tx), 3), round(float(ty), 3))))
        self.log(f"goto ({tx:.2f}, {ty:.2f})")

    # ------------------------------------------------------------ render
    def _render_safe(self):
        """Sim thread: both cameras now (step's cadence, or a due render after
        _forget_scene). Never raises: a failure turns the cameras off."""
        self._render_due = False
        try:
            self._render()
        except Exception as e:                           # noqa: BLE001 — cameras are not physics
            self.log(f"render failed ({type(e).__name__}: {e}); cameras off")
            self._renderers = False

    def _render(self):
        if self._renderers is None:
            try:
                chase = mujoco.Renderer(self.model, 400, 640)
                eye = mujoco.Renderer(self.model, 240, 320)
            except Exception as e:
                self.log(f"no offscreen renderer ({e}); cameras off")
                self._renderers = False
                return
            cam = mujoco.MjvCamera()
            ecam = mujoco.MjvCamera()
            ecam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            ecam.fixedcamid = self.model.camera("eye").id
            # world objects are geom group 3 (lidar-visible, sim_lidar's
            # convention) and renderers hide group 3 by default: show it
            self._vopt = mujoco.MjvOption()
            self._vopt.geomgroup[3] = 1
            self._renderers = (chase, eye, cam, ecam)
            self._cams_live = True
        if self._renderers is False:
            return
        chase, eye, cam, ecam = self._renderers
        cam.lookat[:] = self.data.xpos[self.torso]
        cam.distance, cam.elevation, cam.azimuth = self.cam["distance"], self.cam["elevation"], self.cam["azimuth"]
        chase.update_scene(self.data, cam, self._vopt)
        self._overlay(chase.scene)
        self._put("chase", chase.render())
        eye.update_scene(self.data, ecam, self._vopt)
        self._put("eye", eye.render())

    def _overlay(self, scn):
        """State marker + command arrow (the playground HUD) and the goto
        target as a flag, appended to the renderer's scene."""
        n0 = scn.ngeom
        try:
            self.draw_hud_into(scn)
        except Exception:
            scn.ngeom = n0
        gs = self.goto_state
        if gs is not None and scn.ngeom + 2 <= scn.maxgeom:
            base = np.array([gs["tx"], gs["ty"], self.z0 + 0.002])
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CYLINDER, np.array([0.03, 0.002, 0]),
                                base, np.eye(3).flatten(), np.array([0.4, 0.8, 1.0, 0.6], dtype=np.float32))
            scn.ngeom += 1
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                                np.eye(3).flatten(), np.array([0.4, 0.8, 1.0, 0.9], dtype=np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.006, base + [0, 0, 0.16], base + [0, 0, 0.02])
            scn.ngeom += 1

    def draw_hud_into(self, scn):
        """draw_hud without resetting ngeom (the renderer's scene already
        holds the world)."""
        n0 = scn.ngeom
        self.draw_hud(_SceneTail(scn, n0))

    def _put(self, name, rgb):
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=72)
        n = self.frames[name][0] + 1
        self.frames[name] = (n, buf.getvalue())
        if name == "chase" and self.rec is not None and len(self.rec["frames"]) < 15 * 300:
            self.rec["frames"].append(rgb.copy())

    # ------------------------------------------------------- world swaps
    def set_world(self, spec, name="custom", keep_pose=None):
        """Sim thread. Rebuild the world; the robot respawns upright. An EDIT
        of the current world keeps its pose and heading; a DIFFERENT world
        starts it at the origin facing +x (yaw 0), which is what GOTO_DOC and
        the brains are told (D052 review: the old heading was carried into a
        fresh world, so "forward" and the map's +x disagreed from the start).
        keep_pose: True for an edit (/api/world/add passes it — its first edit
        of a preset renames the world 'custom', which a name comparison took
        for a new world and teleported the robot home), False to force the
        origin; None = keep only when the name is unchanged (a reload)."""
        model, z0 = build_world(spec)
        old = self.data
        p_xy = old.xpos[self.torso][:2].copy()
        yaw = float(np.arctan2(old.xmat[self.torso].reshape(3, 3)[1, 0], old.xmat[self.torso].reshape(3, 3)[0, 0]))
        same_world = (name == self.world_name) if keep_pose is None else bool(keep_pose)
        self.model, self.z0 = model, z0
        self.world_spec, self.world_name = dict(spec), name
        self.data = mujoco.MjData(model)
        self.torso = model.body("torso").id
        self.fids = [model.geom(f"foot{i}").id for i in range(5)]
        self._renderers = None
        self._abandon_goto("world changed")
        self.gesture = None
        self.push = None
        self._respawn(p_xy if same_world else np.zeros(2), yaw if same_world else 0.0, keep_righter=True)
        self.fingerprint = robot_fingerprint(model)
        # scene memory follows the world: an edit carries it over, another world loads its own.
        # A memory error must never leave the world switch half done (the model is swapped).
        # carry only on an explicit EDIT (keep_pose=True: /api/world/add): a different spec under
        # the same name ('custom' again) is a different world with its own key and file
        if self.awareness.get("recognize"):
            # D057: the memory follows the recognised place, not the world's name. A respawn at
            # the origin leaves the robot not knowing where it is (a provisional memory until the
            # verdict); an edit keeps its pose, so it keeps its place (its odometry says it did
            # not move) and only looks again for what changed
            self._place_spawned(f"world {'edited' if same_world else 'loaded'}", moved=not same_world)
        else:
            try:
                self.memory.set_world(memory_key(name, spec), carry=bool(keep_pose))
            except Exception as e:                       # noqa: BLE001
                self.log(f"memory: world switch failed ({type(e).__name__}: {e}) — memory kept as it was")
        # D057: with recognition on, the memory record does not name the world (ground truth a
        # model could read back through recall; review 2026-09-25)
        wname = "a world" if self.awareness.get("recognize") else f"world '{name}'"
        self._memory_epoch(f"{wname} {'edited' if same_world else 'loaded'}: its objects are at "
                           "their spawn", spawn=not same_world)
        self._forget_scene(f"world '{name}' {'edited' if same_world else 'loaded'}")
        if self.walk is not None:
            self.set_walk(self.walk["name"], _keep_gait=self.walk.get("gait_prev"))
        self.log(f"world: {name} ({model.ngeom} geoms) | {self.righter_note}")

    def _abandon_goto(self, why):
        gs, self.goto_state = self.goto_state, None
        if gs is not None:
            self._resolve(gs, {"ok": False, "stopped": "preempted", "detail": why, "pose": self.pose()})

    def _respawn(self, xy=(0.0, 0.0), yaw=0.0, keep_righter=False):
        """keep_righter: re-install the chosen righter on the fresh supervisor —
        and keep 'righter off' off (D052: a world change used to bring the
        default checkpoint back)."""
        old = getattr(self.sup, "righter", None)
        from pebble_gait import leg_ik, body_to_leg
        q0 = np.array([leg_ik(body_to_leg(i, self.gait.p_nom[i])) for i in range(5)]).flatten()
        jadr = [self.model.joint(f"{n}{i}").qposadr[0] for i in range(5) for n in ("yaw", "hip", "knee")]
        mujoco.mj_resetData(self.model, self.data)      # world bodies (the ball) back to their spawn
        self.data.qpos[0:3] = [xy[0], xy[1], rm.spawn_z_m(self.gait.h, platform_z_m=self.z0)]   # D052: not +14 mm
        self.data.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        self.data.qpos[jadr] = q0
        self.data.ctrl[:] = 0
        self.data.ctrl[:15] = q0
        mujoco.mj_forward(self.model, self.data)
        with self.lock:
            self.cmd_v[:] = 0
        self.sup = ReflexSupervisor(self.gait)
        self.mode = "idle"
        if keep_righter:
            if old is not None:
                self.righter_note = self._install_righter(getattr(old, "ckpt", None))
            elif not self.righter_note.startswith("no righter ("):
                self.righter_note = "no righter (stall/deadline ramp only)"

    def reset(self):
        """Sim thread: respawn at the origin, fresh supervisor, same world."""
        self._abandon_goto("reset")
        self.gesture = None
        self.push = None
        self._respawn(keep_righter=True)
        if self.awareness.get("recognize"):
            self._place_spawned("reset", moved=True)    # D057: carried to the spawn: where is it now?
        self._memory_epoch("reset: the robot and the world's objects are back at their spawn")
        self._forget_scene("reset")
        self.log("reset")
        return "reset: origin, upright, planted"

    def _forget_scene(self, why):
        """Sim thread (set_world, reset): the scene the robot last described is
        gone — another world, an edited one, or the same one with the robot
        and every object back at their spawn. Drop what would describe it in
        the next situation line: the eye's last description and find_object's
        last report (Brains.last_look / Brains.last_find — the clause turns
        'eye: no description yet'), the composed situation itself (the feed
        and `status` showed it until the next compose), and the awareness
        tick's previous lidar scan / pose / curious scene (so the first scan
        in the new scene is a baseline, not a change); the next tick composes
        a fresh situation. The eye FRAME goes too (review 2026-09-25: it is
        only replaced by a render, and nothing renders while paused, with the
        cameras off, or for up to 1/15 s on the cadence — a look in that gap
        described the old world again): it turns empty with its counter moved
        on (so _put stays monotonic and the MJPEG route skips it), look /
        find_object / a multimodal turn say 'no eye frame yet', and when the
        cameras work a render is due on the next loop turn, paused or not
        (_render_due; never when no renderer was ever built — tests run
        GL-free with render_every = 10**9). The chase frame stays until that
        render (it is the operator's view, no model reads it). The scene
        memory is not touched here (its epoch is _memory_epoch's job). Never
        raises. A look already in flight when the world changes still lands
        afterwards (it describes the frame it took)."""
        try:
            b = getattr(self, "brains", None)
            dropped = [k for k in ("last_look", "last_find") if getattr(b, k, None) is not None]
            for k in ("last_look", "last_find"):
                if b is not None and hasattr(b, k):
                    setattr(b, k, None)
            n, jpg = self.frames.get("eye", (0, b""))
            self.frames["eye"] = (n + 1, b"")
            if jpg:
                dropped.append("the eye frame")
            self._render_due = bool(self._cams_live)
            self.situation = dict(text="", t=None)
            self._aw.update(prev=None, prev_pose=None, curious_scene=None, compose_next=0.0)
            if dropped:
                self.log(f"{why}: forgot {' + '.join(dropped)} (they described the old scene)")
        except Exception as e:                           # noqa: BLE001 — bookkeeping, never fatal
            self.log(f"{why}: scene context not cleared ({type(e).__name__}: {e})")

    def _memory_epoch(self, why, spawn=True):
        """Sim thread (or before it starts): the world's bodies are back at
        their spawn, so every earlier sighting may be wrong now
        (SceneMemory.new_epoch: they turn 'earlier_session' + stale). spawn:
        pin 'start' at the robot's spawn pose. Never raises."""
        try:
            pose = self.pose()
            self._pose_cache = pose
            self.memory.new_epoch(why=why, pose=pose,
                                  spawn={"x": pose["x"], "y": pose["y"], "yaw": pose["yaw_deg"]} if spawn else None)
        except Exception as e:                           # noqa: BLE001 — memory is not physics
            self.log(f"memory: epoch not recorded ({type(e).__name__}: {e})")

    # ------------------------------------------------ situational awareness
    def awareness_settings(self):
        return dict(self.awareness, check_s=AWARE_CHECK_S, react_min_s=REACT_MIN_S,
                    curious_min_s=CURIOUS_MIN_S, new_obstacle_m=NEW_OBSTACLE_M)

    def set_awareness(self, interval_s=None, curious=None, reactions=None, recognize=None):
        """Any thread. interval_s: seconds between situation updates while idle
        (0 = off; else 5..3600); curious: allow one unprompted look (a vision
        model call) per CURIOUS_MIN_S when the lidar scene changed; reactions:
        speak a chord on a guard latch or a new obstacle (once per REACT_MIN_S);
        recognize (D057): place recognition — the scene memory keyed by the place
        the robot recognises instead of the world's name (see recognize_place)."""
        a = dict(self.awareness)
        if interval_s is not None:
            if isinstance(interval_s, bool):
                raise ValueError("interval_s must be a number of seconds (0 = off)")
            v = float(interval_s)
            if not np.isfinite(v) or v < 0:
                raise ValueError("interval_s must be a finite number >= 0 (0 = off)")
            a["interval_s"] = 0.0 if v == 0 else float(np.clip(v, 5.0, 3600.0))
        for k, v in (("curious", curious), ("reactions", reactions), ("recognize", recognize)):
            if v is not None:
                if not isinstance(v, bool):
                    raise ValueError(f"{k} must be true or false")
                a[k] = v
        was = bool(self.awareness.get("recognize"))
        self.awareness = a
        self._aw["compose_next"] = 0.0
        self._aw["fails"] = 0
        self.log("awareness: " + ", ".join(f"{k} {v}" for k, v in a.items()))
        if bool(a.get("recognize")) != was:
            self._place_switch(bool(a["recognize"]))
        return self.awareness_settings()

    def _lidar_summary(self):
        """Sim thread: the puck's 360 deg scan as 8 body-frame sectors (nearest
        return per sector, None = nothing within range) + the nearest overall."""
        angles, ranges, _ = lidar_scan(self.model, self.data, self.torso)
        deg = np.degrees(angles)
        idx = (((deg + 22.5) % 360.0) // 45.0).astype(int)
        sectors = {}
        for i, name in enumerate(SECTORS):
            r = ranges[(idx == i) & np.isfinite(ranges)]
            sectors[name] = round(float(r.min()), 3) if r.size else None
        near = [(r, n) for n, r in sectors.items() if r is not None]
        r, n = min(near) if near else (None, None)
        plane = float(self.data.xpos[self.torso][2] - self.z0 + PUCK_DZ)
        return dict(sectors=sectors, nearest_m=r, nearest_dir=n, plane_m=round(plane, 3))

    @staticmethod
    def _lidar_words(lid):
        """'lidar: nearest 0.42 m ahead, 1.10 m left; clear elsewhere' — every
        sector is accounted for (review 2026-09-24: 'clear' used to be cut at
        4 names, so the unlisted clear sectors read as unknown). Its height
        caveat (sees only things taller than ~0.18 m) is in MEMORY_NOTE."""
        if not lid:
            return "lidar: no scan"
        s = lid["sectors"]
        close = sorted(((r, n) for n, r in s.items() if r is not None and r < 1.5))
        if not close:
            return "lidar: nothing within 1.5 m"
        shown = ", ".join(f"{r:.2f} m {n}" for r, n in close[:3])
        if len(close) <= 3:
            return f"lidar: nearest {shown}; clear elsewhere"
        clear = [n for n, r in s.items() if r is None or r >= 1.5]
        return (f"lidar: nearest {shown} (+{len(close) - 3} more within 1.5 m); "
                f"clear: {', '.join(clear) if clear else 'nothing'}")

    def _eye_words(self, now, quote=64):
        """The eye clause: the last description (quoted, <= `quote` chars), or —
        when find_object used the eye since — what it found."""
        lk = getattr(self.brains, "last_look", None)
        lf = getattr(self.brains, "last_find", None)
        if lf and (not lk or lf["t"] >= lk["t"]):
            return (f"eye: last used by find_object ({lf['name']} {'found' if lf['found'] else 'not found'}, "
                    f"{fmt_age(now - lf['t'])})")
        if lk:
            q = lk["text"] if len(lk["text"]) <= quote else lk["text"][:quote - 1].rsplit(" ", 1)[0] + "…"
            return f"eye ({lk.get('model')}, {fmt_age(now - lk['t']).replace('just now', 'now')}): \"{q}\""
        return "eye: no description yet"

    def compose_situation(self, lid=None, now=None):
        """Sim thread: the robot's situation as one line + the fields behind it
        -> self.situation. Reads only what the sim already has (pose, guards,
        a lidar scan, the last look, the memory, servo heat, the bus): it never
        calls a model. Aimed at < 400 chars (it rides in every chat turn): the
        nominal clauses are left out (no 'no guard latched', heat only above
        HEAT_SAY, the bus only when servos are connected) and the most useful
        come first — pose, guards, lidar, memory, heat, bus, eye — so a cut
        (Brains.situation_text, SITUATION_CHARS) drops the least useful tail."""
        now = time.time() if now is None else now
        lid = self._lidar_summary() if lid is None else lid
        pose = self.pose()
        self._pose_cache = pose
        g = self.guard_status()
        guards = []
        if self.void is not None:
            guards.append(f"VOID latched at {self.void['bearing_deg']:.0f} deg (body frame; `clear` releases)")
        if self.sup.latched:
            guards.append(f"LATCHED safe-stop ({self.sup.latch_reason})")
        if g.get("locomotion_held"):
            guards.append("locomotion held (sim2real)")
        th = self.thermal_status()
        if th["tripped"]:
            guards.append(f"servos PAST the thermal budget: {', '.join(th['tripped'])}")
        heat = (f"servos warm: {th['hot']} at {th['heat_max'] * 100:.0f}% of its thermal budget"
                if not th["tripped"] and th["heat_max"] > HEAT_SAY else None)
        hw = self._hw_brief()
        if hw is None:
            bus = "bus: no servos connected (sim only)"
        else:
            bus = (f"bus: {hw.get('port')}, {hw.get('n', 0)} servos, mirror {hw.get('mirror')}, "
                   f"{hw.get('errors', 0)} errors" + (f", tripped ids {hw['tripped']}" if hw.get("tripped") else ""))
            if hw.get("tripped"):
                guards.append(f"servo ids {hw['tripped']} tripped on the bus")
        lk = getattr(self.brains, "last_look", None)
        look = None
        if lk:
            look = dict(text=lk["text"], model=lk.get("model"), age_s=round(now - lk["t"], 1))
        mpose = {"x": pose["x"], "y": pose["y"], "yaw_deg": pose["yaw_deg"]}
        recognizing = bool(self.awareness.get("recognize"))
        # D057: the place clause (up to pm.VERDICT_MAX) leads while recognition is on; to keep
        # the line under 400 chars the memory names 2 objects instead of 3 and the eye quote
        # is 24 chars, not 64 (review 2026-09-25: 446 chars with a full memory and a long look;
        # measured after: 385 with a 90-char place clause)
        mem = self.memory.summary(mpose, now=now, max_objects=2 if recognizing else 3, short=True)
        idle = self.idle_reason() is None
        state = self.sup.state
        px, py, pyaw = (0.0 if abs(v) < 0.005 else v for v in (pose["x"], pose["y"], pose["yaw_deg"]))
        if recognizing:
            # D057: the recognised place comes first (verdict_text, <= 90 chars) and the world's name
            # is left out: it is ground truth a real robot never has, and a model would read it
            parts = [self._place_text(),
                     f"at ({px:.2f}, {py:.2f}) m facing {pyaw:.0f} deg; reflex {state}, {'idle' if idle else self.mode}"]
        else:
            parts = [f"at ({px:.2f}, {py:.2f}) m facing {pyaw:.0f} deg in world '{self.world_name}'; "
                     f"reflex {state}, {'idle' if idle else self.mode}"]
        if guards:
            parts.append("GUARDS: " + "; ".join(guards))
        parts += [self._lidar_words(lid), mem]
        if heat:
            parts.append(heat)
        if hw is not None:
            parts.append(bus)
        parts.append(self._eye_words(now, quote=24 if recognizing else 64))
        text = ". ".join(parts) + "."
        self.situation = dict(text=text, t=round(now, 2), pose=pose, world=self.world_name, state=state,
                              mode=self.mode, idle=idle, guards=guards, lidar=lid, look=look, memory=mem,
                              heat=th, bus=bus)
        if recognizing:
            self.situation["place"] = self._place_brief()
        return self.situation

    async def situation_now(self):
        """Any coroutine: a FRESH situation (composed in the sim thread) — what a
        chat turn prepends, so the brain's 'what's around you' is current."""
        return await self.call(self.compose_situation)

    def _aware_failed(self, e):
        self._aw["fails"] += 1
        self.log(f"awareness tick failed ({type(e).__name__}: {e}) [{self._aw['fails']}/{AWARE_FAILS_MAX}]")
        if self._aw["fails"] >= AWARE_FAILS_MAX:
            self.awareness = dict(self.awareness, interval_s=0.0, reactions=False, curious=False)
            self.log("awareness: OFF after repeated failures (POST /api/awareness turns it back on)")

    def _react(self, word, why, now):
        """Say a chord on the robot's own initiative, at most once per
        REACT_MIN_S, when reactions are on; Events + console + memory."""
        if not self.awareness["reactions"]:
            return False
        if now - self._aw["last_react"] < REACT_MIN_S:
            self.log(f"reaction {word} held back (one per {REACT_MIN_S:.0f} s): {why}")
            return False
        self._aw["last_react"] = now
        self.do(f"say {word}")
        self.events.append(("react", f"{word}: {why}"))
        self.log(f"react: {word} — {why}")
        try:
            self.memory.remember({"kind": "said", "text": f"{word} ({why})", "pose": self._pose_cache})
        except Exception:                                # noqa: BLE001
            pass
        return True

    def _aware_tick(self, now=None, lid=None):
        """Sim thread, from step(): at AWARE_CHECK_S (wall clock) —
          1. a pending guard alarm (void / latch / bus cut) -> 'alarm_help';
          2. a lidar look-around: while the robot stood still and idle, a return
             within NEW_OBSTACLE_M where there was none -> 'curious_question'
             (the robot's own walking does not count);
          3. every interval_s while idle + NORMAL: compose the situation;
             'curious' on: one look (a vision-model call, on the event loop)
             per CURIOUS_MIN_S when the lidar scene changed.
        `now` / `lid` are injectable (the tests: no physics, no model)."""
        now = time.monotonic() if now is None else now
        aw, a = self._aw, self.awareness
        if now < aw["check_next"]:
            return
        aw["check_next"] = now + AWARE_CHECK_S
        pend, aw["pending"] = aw["pending"], None
        if pend is not None:
            self._react(pend[0], pend[1], now)
        lid = self._lidar_summary() if lid is None else lid
        pose = self.pose()
        self._pose_cache = pose
        idle = self.sup.state == "NORMAL" and self.idle_reason() is None
        prev, pp = aw["prev"], aw["prev_pose"]
        still = pp is not None and np.hypot(pose["x"] - pp["x"], pose["y"] - pp["y"]) < STILL_M and \
            abs((pose["yaw_deg"] - pp["yaw_deg"] + 180.0) % 360.0 - 180.0) < STILL_DEG
        if prev is not None and still and idle:
            for name, r in lid["sectors"].items():
                was = prev["sectors"].get(name)
                if r is not None and r < NEW_OBSTACLE_M and (was is None or was > r + 0.1):
                    why = f"something new {r:.2f} m {name} (lidar)"
                    try:
                        self.memory.remember({"kind": "scan", "text": why, "pose": pose})
                    except Exception:                    # noqa: BLE001
                        pass
                    self._react("curious_question", why, now)
                    break
        aw["prev"], aw["prev_pose"] = lid, pose
        if a["interval_s"] > 0 and idle and now >= aw["compose_next"]:
            aw["compose_next"] = now + a["interval_s"]
            self.compose_situation(lid)
            if a["curious"]:
                self._maybe_curious(lid, now)

    def _maybe_curious(self, lid, now):
        """Sim thread: schedule ONE look on the event loop when the lidar scene
        changed since the last curious look (and CURIOUS_MIN_S has passed)."""
        aw = self._aw
        last = aw["curious_scene"]
        changed = last is None or any(
            (r is None) != (last["sectors"].get(n) is None) or
            (r is not None and abs(r - last["sectors"][n]) > SCENE_CHANGE_M)
            for n, r in lid["sectors"].items())
        task = aw["curious_task"]
        if not changed or now - aw["last_curious"] < CURIOUS_MIN_S or self._aloop is None or \
                (task is not None and not task.done()):
            return
        aw["last_curious"], aw["curious_scene"] = now, lid
        self.log("curious: the scene changed — one look")
        self.events.append(("curious", "look"))
        try:
            aw["curious_task"] = asyncio.run_coroutine_threadsafe(self.brains.look(), self._aloop)
        except RuntimeError:                              # the loop is closed
            aw["curious_task"] = None

    # ------------------------------------------------------ D057: place recognition
    @staticmethod
    def _place_blank():
        """The place state (self._place). spawn_seq counts spawns (world loads, resets, edits,
        recognition switched on); a result is for the spawn_seq it ran on (result_seq).
        edit: the last spawn kept the robot's pose (a world edit) — its recognition is a change
        check of the bound place, never a new verdict. spawn_wall: time.time() at that spawn.
        binding: what binding the current place did, so an operator's correction can take it
        back ({pid, pre: the place's record before this binding's first visit, own: the place
        was stored on this binding, mark: the scene memory at the bind}); None when unbound."""
        return dict(spawn_seq=0, spawn_t=0.0, spawn_wall=0.0, edit=False, result_seq=-1, result=None,
                    status="off", reason="", bound=None, binding=None, running=None, samples=[], diff=None,
                    looks=0, t=None, error=None, note=None, look_error=None, by=None, check_next=0.0,
                    task=None, task_t=0.0, fails=0, phase=None, checks=None)

    def _place_switch(self, on):
        """set_awareness: recognition switched on (the robot does not know where it is: a
        provisional memory, a recognition due) or off (the world-keyed memory, exactly as
        before; the place state goes)."""
        self._place = self._place_blank()
        if on:
            st = self._place
            st.update(spawn_seq=1, spawn_t=self.t - RECOG_SETTLE_S, spawn_wall=time.time(), status="due",
                      reason="place recognition switched on")
            self._place_provisional()
            self.log("place: recognition ON — the scene memory now follows the recognised place")
            return
        self._mem_scratch = None
        self.memory = self._mem_main
        try:
            self._mem_main.set_world(memory_key(self.world_name, self.world_spec), carry=False)
        except Exception as e:                           # noqa: BLE001
            self.log(f"memory: back to the world's memory failed ({type(e).__name__}: {e})")
        self.log("place: recognition OFF — the scene memory follows the world again")

    def _place_provisional(self):
        """Not in a known place (a respawn, recognition switched on, the place forgotten): a
        fresh RAM-only scene memory stands in until a verdict binds one (_place_bind carries
        what it recorded over). Never persisted: its key would name no place."""
        self._mem_scratch = SceneMemory(PLACE_PENDING_KEY, directory=None, log=self.log)
        self.memory = self._mem_scratch
        self._place["bound"] = None
        self._place["binding"] = None

    def _place_spawned(self, why, moved):
        """Sim thread (set_world, reset): a recognition is due once the robot stands still.
        moved: it was put somewhere (a respawn at the origin) — provisional memory until the
        verdict; an edit (pose kept) keeps its place, and its recognition is only a change
        check of that place (never a new verdict, never another place: _recognize)."""
        st = self._place
        st.update(spawn_seq=st["spawn_seq"] + 1, spawn_t=self.t, spawn_wall=time.time(), edit=not moved,
                  reason=why, status="due", diff=None, check_next=0.0, fails=0)
        if moved:
            self._place_provisional()
        self.log(f"place: {why} — recognition due")

    def _place_running(self, now=None):
        """A recognition is running on a live loop (a dead loop's or a timed-out one is not)."""
        r = self._place.get("running")
        if r is None:
            return False
        loop, t0 = r
        now = time.monotonic() if now is None else now
        try:
            alive = not loop.is_closed() and loop.is_running()
        except Exception:                                # noqa: BLE001
            alive = False
        return alive and now - t0 < RECOG_TIMEOUT_S

    def _place_settle_reason(self):
        """Sim thread: None when a recognition may look now (standing still, NORMAL, idle,
        RECOG_SETTLE_S after the spawn), else why not."""
        if self.paused:
            return "the sim is paused"
        if self.t - self._place["spawn_t"] < RECOG_SETTLE_S:
            return "settling after the spawn"
        if self.sup.state != "NORMAL":
            return f"reflex state is {self.sup.state}"
        return self.idle_reason()

    def _place_tick(self, now=None):
        """Sim thread, from step() while recognition is on: when a recognition is due (a
        spawn since the last result), none runs and the robot stands still, schedule
        recognize_place on the event loop (the loop `call` last ran on)."""
        st = self._place
        if st["result_seq"] == st["spawn_seq"] or self.fatal:
            return
        now = time.monotonic() if now is None else now
        if now < st["check_next"]:
            return
        st["check_next"] = now + 0.25
        if self._place_running(now):
            return
        task = st["task"]
        if task is not None and not task.done() and now - st["task_t"] < RECOG_TIMEOUT_S:
            return                                       # scheduled, not started yet
        if self._place_settle_reason() is not None:
            return
        loop = self._aloop
        if loop is None or loop.is_closed():
            return
        coro = self.recognize_place(f"auto: {st['reason']}")
        try:
            st["task"] = asyncio.run_coroutine_threadsafe(coro, loop)
            st["task_t"] = now
        except RuntimeError:                             # the loop is closed
            coro.close()
            st["task"] = None

    def _place_tick_failed(self, e):
        st = self._place
        st["fails"] = st.get("fails", 0) + 1
        self.log(f"place tick failed ({type(e).__name__}: {e}) [{st['fails']}/{AWARE_FAILS_MAX}]")
        if st["fails"] >= AWARE_FAILS_MAX:
            self.awareness = dict(self.awareness, recognize=False)
            self._place_switch(False)
            self.log("place: recognition OFF after repeated failures (POST /api/awareness turns it back on)")

    async def recognize_place(self, reason="operator", force=False, check=False):
        """Any coroutine (the tick schedules it; POST /api/place {action: recognize} awaits it).
        Which place is this? Waits out a recognition that is running; returns the current
        spawn's result when there is one (force: look again anyway), else: wait for a
        standstill, one sweep + one look (+ its embedding, + the eye's box answers on what it
        mentions) -> PlaceMemory.recognize ->
          ambiguous: a second look after a PLACE_TURN_DEG turn, the two combined;
          new: the same second look (the situation line says so), then a place stored ('new
                 place #k') from both looks, the scene memory bound to it;
          known: a visit, the memory bound to it, the place checked BY THE EYE (_place_changes:
                 per remembered / mentioned name a box question -> checked_objects) — a change
                 takes the second look, and only what both looks agree on is declared and
                 stored (confirm_diff).
        After an edit (the robot was not moved), or with check=True (place_check), a place the
        robot is bound to stays its place: the recognition is only that place's change check.
        A look that failed (no description) is never taken for a look that saw nothing.
        -> the where_am_i answer. Never raises; a spawn meanwhile discards it (the next one runs)."""
        st = self._place
        if not self.awareness.get("recognize"):
            return {"ok": False, "recognize": False, "error": PLACE_OFF, "hint": PLACE_OFF_HINT}
        t_end = time.monotonic() + RECOG_TIMEOUT_S
        while self._place_running() and time.monotonic() < t_end:
            await asyncio.sleep(0.05)
        st = self._place
        if not force and st["result"] is not None and st["result_seq"] == st["spawn_seq"]:
            return self._place_answer()
        st["running"] = (asyncio.get_running_loop(), time.monotonic())
        st["status"] = "running"
        seq = st["spawn_seq"]
        try:
            out = await self._recognize(st, seq, reason, check=bool(check))
        except Exception as e:                           # noqa: BLE001 — never kill the caller
            self.log(f"place: recognition failed ({type(e).__name__}: {e})")
            st["error"] = f"{type(e).__name__}: {e}"
            st["check_next"] = time.monotonic() + RECOG_RETRY_S
            out = {"ok": False, "error": f"place recognition failed: {type(e).__name__}: {e}"}
        finally:
            st["running"] = None
            st["phase"] = None
            if st["status"] == "running":
                st["status"] = "done" if st["result_seq"] == seq else "due"
        return out

    async def place_check(self, reason="operator"):
        """Any coroutine: a change check of the place the robot is in, NOW (talk mode's 'this
        master bedroom has a new chair' runs it after naming the place): one look, and a change
        only when a second look from another angle agrees. The robot stays in its place (no new
        verdict, no other place). -> the where_am_i answer."""
        if not self.awareness.get("recognize"):
            return {"ok": False, "recognize": False, "error": PLACE_OFF, "hint": PLACE_OFF_HINT}
        if not self._place.get("bound"):
            return {"ok": False, "error": "the robot is not in a known place: nothing to check against "
                                          "(name it first)"}
        return await self.recognize_place(reason, force=True, check=True)

    async def _place_wait_settled(self, seq):
        """None once the robot stands still after the spawn, else why it did not (in time)."""
        t_end = time.monotonic() + RECOG_WAIT_S
        while True:
            if self._place["spawn_seq"] != seq:
                return _STALE
            why = await self.call(self._place_settle_reason)
            if why is None:
                return None
            if why == "the sim is paused" or time.monotonic() > t_end:
                return why
            await asyncio.sleep(0.1)

    def _place_scan(self):
        """Sim thread: one puck sweep -> (the place signature, the pose, the sweep's returns as
        (body angles rad, ranges m) — what look_hidden reads for tall things in the way). Map
        heading = body angle + yaw; the sim's pose stands in for the robot's odometry (a
        perfect one)."""
        angles, ranges, _ = lidar_scan(self.model, self.data, self.torso)
        pose = self.pose()
        hit = np.isfinite(ranges)
        sweep = (np.asarray(angles, dtype=float)[hit], np.asarray(ranges, dtype=float)[hit])
        return pm.scan_signature(angles, ranges, np.radians(pose["yaw_deg"]), RANGE_MAX), pose, sweep

    async def _place_observe(self, seq):
        """(observation, None) or (None, why): a sweep, a look through the vision role
        (Brains.look: remembered, last_look) and its embedding; why = _STALE when the scene
        changed meanwhile, 'moved' when the robot moved while it looked. A look that FAILED
        (the model erred, no eye frame, a stale scene) has objects None and look_error set:
        it saw nothing, which is not the same as seeing that nothing is there — no change
        check may read it as absence (review 2026-09-25: two failed looks erased a ball)."""
        sig, pose, sweep = await self.call(self._place_scan)
        look = await self.brains.look()
        ok = isinstance(look, dict) and look.get("ok") and not look.get("stale_scene")
        text = str(look.get("description") or "").strip() if ok else ""
        emb = None
        if text:
            fn = self.embed_fn or self.brains.embed
            try:
                emb = await asyncio.wait_for(asyncio.to_thread(pm.embed_description, text, fn), PLACE_EMBED_WAIT_S)
            except asyncio.TimeoutError:
                emb = None
        if self._place["spawn_seq"] != seq:
            return None, _STALE
        after = await self.call(self.pose)
        if _moved(after, pose):
            return None, "moved"
        err = None
        if not text:
            err = (look.get("error") or look.get("note") or "") if isinstance(look, dict) else str(look)
            if isinstance(look, dict) and look.get("stale_scene"):
                err = "the look described a scene that is gone"
            err = str(err or "no description").strip()[:120]
        obs = {"sig": sig, "pose": pose, "sweep": sweep, "text": text, "emb": emb,
               "objects": objects_from_description(text, pose) if text else None,
               "model": look.get("model") if isinstance(look, dict) else None,
               "look_error": err, "info": pm.scan_info(sig),
               "mentioned": pm.mentioned_names(text) if text else set(), "eye": {}}
        # what the description names is only a question: the eye's box answers it (and places
        # it), asked NOW, while the robot still stands where it looked
        why = await self._place_ask(seq, obs, place_candidates(text)[:PLACE_EYE_NEW])
        if why is not None:
            return None, why
        return obs, None

    async def _place_ask(self, seq, look, names, spots=None):
        """Ask the eye about each name this look was not asked about yet, from where the look was
        taken: Brains.detect(name, model=<the look's own vision model>, method="bbox") — the
        question /api/look {find: name, method: "bbox"} and find_object ask (FIND_PROMPT: a box
        -> pixel_to_floor -> sighting_to_map, the map spot). look["eye"][name] =
          {seen: True, asked, x, y, confidence, bearing_deg, distance_m}  boxed; x, y None (+ why)
               when the box's bottom edge gives no floor point within FIND_TRUSTED_M;
          {seen: False, asked}   the eye answered that it is not there;
          {seen: None, why}      no usable answer, or not asked.
        A failed look asks nothing (every name None: it saw nothing, which is not 'not there').
        spots ({name: [(x, y)]}, a snapshot's): a remembered object whose every spot is out of
        this look's view and that its description does not name is not asked (a 'no' could not
        be read either way). The robot no longer where the look was taken: not asked. At most
        PLACE_EYE_MAX questions per look. -> None, or _STALE when a spawn came meanwhile."""
        eye = look.setdefault("eye", {})
        todo = []
        for n in names or ():
            n = norm_name(n)
            if n and not _is_fixture(n) and n not in eye and n not in todo:
                todo.append(n)
        if not todo:
            return None
        if look.get("objects") is None:
            for n in todo:
                eye[n] = {"seen": None, "why": "the look failed"}
            return None
        cone = eye_visible(look["pose"])
        ment = look.get("mentioned") or set()
        asked = sum(1 for a in eye.values() if a.get("asked"))
        for n in todo:
            if self._place["spawn_seq"] != seq:
                return _STALE
            sp = (spots or {}).get(n)
            if sp and not any(cone(x, y) for x, y in sp) and not any(same_thing(n, m) for m in ment):
                eye[n] = {"seen": None, "why": "out of view"}
                continue
            if asked >= PLACE_EYE_MAX:
                eye[n] = {"seen": None, "why": f"not asked (at most {PLACE_EYE_MAX} questions per look)"}
                continue
            if _moved(await self.call(self.pose), look["pose"]):
                eye[n] = {"seen": None, "why": "not asked: the robot is no longer where this look was taken"}
                continue
            asked += 1
            eye[n] = await self._place_eye_answer(n, look)
        return _STALE if self._place["spawn_seq"] != seq else None

    async def _place_eye_answer(self, name, look):
        """One eye question (see _place_ask) -> look["eye"][name]. Only a parsed 'seen: false'
        is a 'no'; an error, an unparsed answer, a 'seen: true' without a box or a bearing, or a
        sighting below FIND_MIN_CONF is no answer (None)."""
        try:
            r = await self.brains.detect(name, model=look.get("model"), method="bbox")
        except Exception as e:                           # noqa: BLE001 — no answer, never 'not there'
            r = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        if not (isinstance(r, dict) and r.get("ok")):
            err = r.get("error") if isinstance(r, dict) else r
            return {"seen": None, "asked": True, "why": f"the eye did not answer ({err})"[:120]}
        d = r.get("detection") or {}
        out = {"asked": True, "model": r.get("model")}
        if not d.get("parsed"):
            return dict(out, seen=None, why=f"the answer did not parse: {str(r.get('raw') or '')[:60]!r}")
        if not d.get("seen"):
            if "(no bearing given)" in str(d.get("what") or ""):   # parse_detection's mark: it said seen
                return dict(out, seen=None, why="said seen, but gave no box")
            return dict(out, seen=False)
        conf = float(d.get("confidence") or 0.0)
        if conf < FIND_MIN_CONF:
            return dict(out, seen=None, why=f"unsure (confidence {conf:.2f} < {FIND_MIN_CONF:g})")
        out.update(seen=True, confidence=round(conf, 2), bearing_deg=d.get("bearing_deg"),
                   distance_m=d.get("distance_m"), x=None, y=None,
                   bbox=list(d["bbox"]) if d.get("method") == "bbox" and d.get("bbox") else None)
        at = sighting_to_map(look["pose"], d) if d.get("method") == "bbox" else None
        if at is None:
            out["why"] = "boxed, not placed (no floor point)" if d.get("method") == "bbox" else \
                "seen, not placed (no box)"
        elif float(d["distance_m"]) > FIND_TRUSTED_M:
            out["why"] = f"boxed, not placed (beyond {FIND_TRUSTED_M:g} m)"
        else:
            out.update(x=at["x"], y=at["y"])
        return out

    async def _place_second_look(self, seq, n0):
        """(observation, None) after a PLACE_TURN_DEG turn (the `turn` tool: one gesture, the
        standstill rule and every guard), or (None, why): an operator stop since the
        recognition began, a refused turn, a spawn meanwhile (_STALE), the robot moving."""
        if self.brains._operator_stopped(n0):
            return None, "the operator stopped the robot: no turn for a second look"
        n_prev = self.frames.get("eye", (0, b""))[0]
        r = await self.brains.tool("turn", {"deg": PLACE_TURN_DEG})
        if self._place["spawn_seq"] != seq:
            return None, _STALE
        if not (isinstance(r, dict) and r.get("ok")):
            why = (r.get("error") or r.get("hint")) if isinstance(r, dict) else r
            return None, f"the {PLACE_TURN_DEG:.0f} deg turn for a second look did not run ({why})"
        if self._cams_live:
            await self.brains._fresh_eye(n_prev)
        obs, why = await self._place_observe(seq)
        if obs is not None:
            obs["turned_deg"] = r.get("turned_deg")
        return obs, why

    def _eye_visible(self, pose):
        return eye_visible(pose)

    @staticmethod
    def _place_eye_check(stored, look, seen_before=()):
        """One GOOD look's change check BY THE EYE -> place_memory.checked_objects(snapshot,
        seen, visible=<what this look could see>) = {added, missing, unobservable, present}.
        seen: the seen map (seen_map: True | False | None per name — a remembered object whose
        every spot is out of view or hidden is None) carried in the look's eye answers (so an
        'added' has the box's map spot, x, y); visible = look_view (in the eye's view and not
        hidden behind a box it drew or a lidar return: an unobservable one says why). The
        look's prose objects never decide: a description places things ~0.3 m off and names
        what it cannot place.
        seen_before: the names this place's earlier looks boxed but never placed
        (unplaced_names). One boxed again is not new here: unplaced again -> 'present'; placed
        now -> still 'added' (two looks must agree before its spot is stored) but marked
        seen_before, so it is stored, never declared (_split_seen_before)."""
        eye = look.get("eye") or {}
        seen = seen_map(look, place_spots(stored))
        arg = {n: dict(eye.get(n) or {}, seen=a) for n, a in seen.items()}
        out = pm.checked_objects(stored, arg, visible=look_view(look))
        if look.get("objects") is not None:
            for o in out.get("unobservable") or []:
                raw = [a.get("seen") for k, a in eye.items() if isinstance(a, dict) and same_thing(o["name"], k)]
                if False in raw and True not in raw and o.get("why") in ("out of view", "no answer"):
                    o["why"] = "said no, but " + (view_why(look, o["x"], o["y"]) or "out of view")
        added = []
        for a in out.get("added") or []:
            if any(same_thing(a["name"], n) for n in seen_before or ()):
                if a.get("x") is None or a.get("y") is None:
                    out.setdefault("present", []).append(dict(a, why="boxed here before, never placed"))
                    continue
                a = dict(a, seen_before=True)
            added.append(a)
        out["added"] = added
        return out

    async def _place_looking_again(self, seq, text):
        """Before the turn for a second look: the place clause says so (st['phase'] leads it
        while the recognition runs), the situation line is composed again, an event."""
        st = self._place
        if st["spawn_seq"] != seq:
            return
        st["phase"] = str(text)[:pm.VERDICT_MAX]
        self.events.append(("place", st["phase"][:80]))
        self.log(st["phase"])
        try:
            await self.call(self.compose_situation)
        except Exception as e:                           # noqa: BLE001 — the look goes on
            self.log(f"place: situation not composed ({type(e).__name__}: {e})")

    @staticmethod
    def _place_in_play(rec):
        """The places whose remembered objects a look asks the eye about: the recognised one
        (known), the two best (ambiguous: either may be it), none for a new place."""
        v = (rec or {}).get("verdict")
        if v == "known":
            return [rec["place_id"]] if rec.get("place_id") else []
        if v == "ambiguous":
            return [e["place_id"] for e in (rec.get("ranking") or [])[:2] if e.get("place_id")]
        return []

    def _place_spots_of(self, ids):
        """place_spots of these places' snapshots, merged: {name: [(x, y), ...]}."""
        out = {}
        for pid in ids:
            q = self.place_memory.get(pid) if pid else None
            for n, sp in place_spots((q or {}).get("objects")).items():
                out.setdefault(n, []).extend(sp)
        return out

    def _place_kept(self, pid, rec0, why):
        """A change check of the place the robot is bound to (an edit: the robot was not moved;
        or place_check): the verdict stays that place — never a new place, never another one —
        with how alike it senses now (its score in rec0's ranking; below the top 3, the 3rd's
        score: an upper bound) and a note when that is low. -> (rec, note)."""
        q = self.place_memory.get(pid) or {}
        rk = rec0.get("ranking") or []
        conf = next((e["confidence"] for e in rk if e["place_id"] == pid), None)
        upper = conf is None and bool(rk)
        if conf is None:
            conf = min((e["confidence"] for e in rk), default=0.0)
        name = q.get("name")
        sec = next((e for e in rk if e["place_id"] != pid and (name is None or e["name"] != name)), None)
        same = rec0.get("place_id") == pid
        rec = {"verdict": "known", "place_id": pid, "name": name, "confidence": round(float(conf), 3),
               "second": dict(sec) if sec else {"place_id": None, "name": None, "confidence": 0.0},
               "parts": rec0.get("parts") if same else None,
               "signals": rec0.get("signals"),        # the senses this look compared (its words: signals_text)
               "evidence": rec0.get("evidence") if same else None,
               "yaw_drift_deg": rec0.get("yaw_drift_deg") if same else None,
               "ranking": rk, "kept": why,
               "robot": {k: rec0.get(k) for k in ("verdict", "place_id", "name", "confidence")}}
        note = f"{why}: the robot was not moved, so it is still in {name or pid}"
        if conf < pm.NEW_T:
            note += (f" — it senses it only {'<= ' if upper else ''}{float(conf):.2f} alike now: "
                     "this place changed a lot")
        return rec, note

    async def _place_changes(self, st, seq, n0, rec, looks, notes, keep):
        """The change check of a known place BY THE EYE -> (diff, None), or (None, why) when the
        scene changed meanwhile (_STALE / 'moved'). diff: {stored, d1, confirmed, look, checks}
        — {fill: the good looks} when the place was never described (its first objects are the
        eye's placed sightings), None when nothing could be compared (notes say why). Every GOOD
        look (a description) is checked by the eye: each remembered name + each thing it
        mentions is a question (_place_ask; the last look is asked now what it was not asked
        yet — the robot still stands there), answered as a seen map -> checked_objects. A
        failed look is not a look that saw nothing. A change takes a second look PLACE_TURN_DEG
        further (unless two looks were taken already) that asks the same questions again; only
        what both looks agree on is confirmed (confirm_diff), the rest is pending."""
        pid = rec["place_id"]
        q = self.place_memory.get(pid) or {}
        stored = q.get("objects") or []
        spots = place_spots(stored)
        before = unplaced_names(q)                       # boxed here before, never placed: not new
        good = [lk for lk in looks if lk["objects"] is not None]
        if not good:
            notes.append("no change check: no look described the scene")
            return None, None
        if not any(t for t in q.get("desc_texts") or []):
            notes.append("no change check: this place was never described before (its objects start now)")
            return {"fill": good}, None
        if keep and float(q.get("created") or 0.0) >= float(st.get("spawn_wall") or 0.0):
            notes.append(f"no change check: {q.get('name') or pid} was first stored on this visit "
                         "(no earlier look to compare with)")
            return None, None
        why = await self._place_ask(seq, looks[-1], list(spots), spots)
        if why is not None:
            return None, why
        diffs = [self._place_eye_check(stored, lk, before) for lk in good]
        if len(good) == 1 and len(looks) == 1 and _changes(diffs[0]):
            n = sum(len(diffs[0].get(k) or []) for k in ("added", "missing", "moved"))
            await self._place_looking_again(seq, looking_again_text(rec, f"{n} change{'s' if n != 1 else ''}"))
            second, why = await self._place_second_look(seq, n0)
            if second is None and why in (_STALE, "moved"):
                return None, why
            if second is None:
                notes.append(why)
            else:
                looks.append(second)
                if second["objects"] is None:
                    notes.append(f"the second look failed ({second['look_error']}): nothing confirmed")
                else:
                    # the same questions again, from the new angle
                    why = await self._place_ask(seq, second, list(spots) + list(good[0].get("eye") or {}), spots)
                    if why is not None:
                        return None, why
                    good.append(second)
                    diffs.append(self._place_eye_check(stored, second, before))
        d1 = diffs[0]
        if len(diffs) >= 2:
            confirmed = pm.confirm_diff(diffs[0], diffs[1], PLACE_MATCH_M)
        else:                                            # one look only: nothing is declared
            confirmed = {"added": [], "missing": [], "moved": [],
                         "pending": [dict(o, change=k) for k in ("added", "missing", "moved")
                                     for o in d1.get(k) or []]}
        confirmed = split_seen_before(confirmed)
        return {"stored": stored, "d1": d1, "confirmed": confirmed, "look": good[0],
                "checks": [eye_brief(lk, spots) for lk in good]}, None

    async def _recognize(self, st, seq, reason, check=False):
        def interrupted(why):
            st["check_next"] = time.monotonic() + (0.0 if why == _STALE else RECOG_RETRY_S)
            msg = ("the scene changed while recognising (a world load, reset or edit): the next "
                   "recognition follows" if why == _STALE else f"not recognised: {why}")
            return {"ok": False, "error": msg, "status": "due"}
        why = await self._place_wait_settled(seq)
        if why is not None:
            return interrupted(why)
        n0 = self.brains._cmd_mark()
        first, why = await self._place_observe(seq)
        if first is None:
            return interrupted(why)
        pmem = self.place_memory
        notes = []
        if first["look_error"]:
            notes.append(f"the look failed ({first['look_error']}): the verdict rests on the lidar alone")
        elif first["emb"] is None:
            notes.append("the description was not embedded (the embedding model did not answer): the "
                         "verdict rests on the lidar alone")
        looks = [first]
        bound = st.get("bound")
        keep = None
        if bound is not None and pmem.get(bound) is not None and (check or st.get("edit")):
            keep = "a change check" if check else "an edit"
        rec = pmem.recognize(scan_sig=first["sig"], desc_emb=first["emb"])
        # the remembered objects of the place(s) in play are asked about NOW, before any turn:
        # the eye can only answer for the view it has
        spots = self._place_spots_of([bound] if keep else self._place_in_play(rec))
        why = await self._place_ask(seq, first, list(spots), spots)
        if why is not None:
            return interrupted(why)
        if keep:
            rec, kn = self._place_kept(bound, rec, keep)
            notes.append(kn)
        elif rec["verdict"] in ("ambiguous", "new"):
            # ambiguous: a second look decides. new: a second look before anything is stored, so
            # every place starts with two samples (the arrival and PLACE_TURN_DEG further)
            await self._place_looking_again(seq, looking_again_text(rec, rec["verdict"]))
            second, why = await self._place_second_look(seq, n0)
            if second is None and why in (_STALE, "moved"):
                return interrupted(why)
            if second is None:
                notes.append(why)
            else:
                looks.append(second)
                if second["look_error"] and not first["look_error"]:
                    notes.append(f"the second look failed ({second['look_error']})")
                rec = combine_recognitions(rec, pmem.recognize(scan_sig=second["sig"], desc_emb=second["emb"]))
                for n, sp in self._place_spots_of(self._place_in_play(rec)).items():
                    cur = spots.setdefault(n, [])
                    cur.extend(p for p in sp if p not in cur)
                why = await self._place_ask(seq, second, list(spots) + list(first.get("eye") or {}), spots)
                if why is not None:
                    return interrupted(why)
        diff = None
        if rec["verdict"] == "known":
            diff, why = await self._place_changes(st, seq, n0, rec, looks, notes, keep)
            if why is not None:
                return interrupted(why)
        out = await self.call(lambda: self._place_commit(st, seq, rec, looks, diff, reason, notes, keep))
        return out if out is not None else interrupted(_STALE)

    def _place_enroll(self, looks, name, notes=None):
        """Sim thread: a new place from this visit's looks -> its id. Every look is a sample
        (PlaceMemory.enroll_samples: the arrival and the second look PLACE_TURN_DEG further —
        two viewpoints from the start, the way _recognize takes them); the first objects are the
        eye's placed sightings the looks do not contradict (eye_objects), never the prose's
        guesses. What the eye boxed but could never place is remembered by name
        (_place_note_unplaced), so a later visit counts it present, not new here."""
        pmem = self.place_memory
        objs = eye_objects(looks)
        pid = pmem.enroll_samples(list(looks), objects=objs, name=name)
        boxed = sorted({n for lk in looks if lk.get("objects") is not None
                        for n, a in (lk.get("eye") or {}).items() if a.get("seen") is True})
        unplaced = [n for n in boxed if not any(same_thing(n, o["name"]) for o in objs)]
        kept = self._place_note_unplaced(pid, [n for n in boxed_unplaced(looks) if n in unplaced])
        rest = [n for n in unplaced if n not in kept]
        if notes is not None and kept:
            notes.append(f"boxed, never placed (no floor spot within {FIND_TRUSTED_M:g} m): "
                         f"{', '.join(kept[:4])} — kept by name, so a later visit counts it present")
        if notes is not None and rest:
            notes.append(f"boxed but not stored (the looks disagree): {', '.join(rest[:4])}")
        return pid

    def _place_note_unplaced(self, pid, names):
        """Sim thread: remember at place `pid` the names the eye boxed there but could not place
        (a PLACE_UNPLACED_NOTE place note: persisted with the place, restored by a correction's
        take-back, forgotten with it), so a later visit counts them present instead of declaring
        them 'new here' every time (review 2026-09-25: a cup on a table was new on every visit).
        A name already recorded, a stored object's name, or a fixture is skipped. -> the names
        added."""
        pmem = self.place_memory
        q = pmem.get(pid) if pid else None
        if q is None:
            return []
        have = unplaced_names(q) + [o.get("name") for o in q.get("objects") or [] if o.get("name")]
        new = []
        for n in names or ():
            n = norm_name(n)
            if n and not _is_fixture(n) and not any(same_thing(n, h) for h in have + new):
                new.append(n)
        room = pm.TEXT_MAX - len(PLACE_UNPLACED_NOTE)
        chunk = []
        for n in new:
            if chunk and len(", ".join(chunk + [n])) > room:
                pmem.note(pid, PLACE_UNPLACED_NOTE + ", ".join(chunk))
                chunk = []
            chunk.append(n[:room])
        if chunk:
            pmem.note(pid, PLACE_UNPLACED_NOTE + ", ".join(chunk))
        if new:
            self.log(f"place: {q.get('name') or pid}: boxed, never placed, kept by name: {', '.join(new)}")
        return new

    def _next_new_name(self):
        ks = [int(m.group(1)) for p in self.place_memory.places()
              for m in [re.match(r"new place #(\d+)$", p.get("name") or "")] if m]
        return f"new place #{max(ks, default=0) + 1}"

    def _place_commit(self, st, seq, rec, looks, diff, reason, notes=(), keep=None):
        """Sim thread (no spawn can slip in): store what the recognition concluded, bind the
        scene memory, react, compose. None when a spawn came first or recognition went off.
        keep (an edit / a change check): nothing is stored but CONFIRMED changes (or a never
        described place's first objects), and the robot stays bound where it was."""
        if st is not self._place or st["spawn_seq"] != seq or not self.awareness.get("recognize"):
            return None
        pmem = self.place_memory
        s0 = looks[0]
        verdict = rec["verdict"]
        now = time.monotonic()
        rec = dict(rec, looks=len(looks), by="robot")
        notes = list(notes)
        confirmed = None
        checks = None
        if verdict == "new":
            pid = self._place_enroll(looks, self._next_new_name(), notes)
            checks = [eye_brief(lk) for lk in looks if lk.get("objects") is not None]
            self._place_bind(pid, own=True)
            q = pmem.get(pid) or {}
            near = f"nearest known {rec['name'] or rec['place_id']} {rec['confidence']:.2f}" if rec["place_id"] \
                else "the first place"
            self._react("curious_question", f"a new place: stored as {q.get('name')} ({near})", now)
        elif verdict == "known":
            pid = rec["place_id"]
            pre = pmem.get(pid)                          # an operator's correction restores this (_place_take_back)
            objs, vis, store, unplaced = None, None, not keep, []
            if diff is None:
                pass                                     # nothing was compared: the snapshot stays
            elif "fill" in diff:                         # never described: the eye's placed sightings start it
                lks = diff["fill"]
                cones = [look_view(lk) for lk in lks]
                objs, store = eye_objects(lks), True
                vis = (lambda x, y, cones=cones: any(c(x, y) for c in cones))
                checks = [eye_brief(lk) for lk in lks]
                unplaced = [n for n in boxed_unplaced(lks) if not any(same_thing(n, o["name"]) for o in objs)]
            else:
                confirmed = diff["confirmed"]
                checks = diff.get("checks")
                if _changes(confirmed) or confirmed.get("placed"):   # only what both looks agree on is stored
                    both = dict(confirmed, added=list(confirmed["added"]) + list(confirmed.get("placed") or []))
                    objs, vis, store = apply_confirmed(diff["stored"], both), None, True
                # an addition boxed by both looks but never placed is declared once, and kept by name
                unplaced = [a["name"] for a in confirmed["added"] if a.get("x") is None or a.get("y") is None]
                # no confirmed change: the snapshot stays as it is (a look rewrites nothing)
            if store:
                pmem.visit(pid, scan_sig=s0["sig"], desc_emb=s0["emb"], desc_text=s0["text"] or None,
                           objects=objs, visible=vis)
                if not keep and len(looks) > 1:          # the second look is another view of this place
                    pmem.add_samples(pid, looks[1:])
            if unplaced:
                self._place_note_unplaced(pid, unplaced)
            self._place_bind(pid, pre=pre)
            if confirmed and confirmed["added"]:
                names = ", ".join(sorted({o["name"] for o in confirmed["added"]}))
                self._react("curious_question", f"new here: {names} (two looks agree)", now)
        if checks is None:                               # ambiguous, or a known place nothing was compared on:
            sp = self._place_spots_of(self._place_in_play(rec))                                # what it asked
            checks = [eye_brief(lk, sp) for lk in looks if lk.get("objects") is not None] or None
        notes = [n for n in notes if n and n != _STALE]
        st.update(result=rec, result_seq=seq, status="done", t=time.time(), looks=len(looks), diff=confirmed,
                  checks=checks, phase=None,
                  samples=[{k: o.get(k) for k in ("sig", "emb", "text", "objects", "pose", "eye", "mentioned")}
                           for o in looks],
                  error=None, note="; ".join(notes) or None, by="robot", fails=0,
                  look_error=next((o["look_error"] for o in looks if o.get("look_error")), None))
        text = self._place_text()
        self.events.append(("place", text[:80]))
        self.log(f"{text} [{len(looks)} look{'s' if len(looks) > 1 else ''}; {reason}]"
                 + (f" — {st['note']}" if st["note"] else ""))
        try:
            self.memory.remember({"kind": "look", "pose": looks[-1]["pose"],
                                  "text": f"{text} ({len(looks)} look{'s' if len(looks) > 1 else ''})"})
        except Exception:                                # noqa: BLE001
            pass
        try:
            self.compose_situation()
        except Exception as e:                           # noqa: BLE001
            self.log(f"place: situation not composed ({type(e).__name__}: {e})")
        return self._place_answer()

    def _place_bind(self, pid, pre=None, own=False, carry=None):
        """Sim thread: the scene memory is now the place's ('place-<id>', persisted with the
        rest): what the provisional memory recorded since the spawn (or `carry`: records an
        operator's correction took back from the wrong place) is carried over, after a new
        epoch (what was seen here on an earlier visit may have moved). Binding the place it is
        already bound to changes nothing (the binding keeps its first `pre`). pre / own: see
        _place_blank's binding. -> True when bound."""
        st = self._place
        main = self._mem_main
        if st.get("bound") == pid and self.memory is main:
            return True
        scratch = self.memory if self.memory is not main else None
        recs = [dict(r) for r in reversed(scratch.last(scratch.cap))] if scratch is not None else []
        recs += [dict(r) for r in carry or []]
        try:
            main.set_world(pid, carry=False)
            mark = _mem_mark(main)
            main.new_epoch()
            for r in recs:
                main.remember({k: r[k] for k in ("kind", "text", "pose", "objects", "t") if k in r})
        except Exception as e:                           # noqa: BLE001 — keep what was there
            self.log(f"memory: not bound to {pid} ({type(e).__name__}: {e})")
            return False
        self.memory = main
        self._mem_scratch = None
        st["bound"] = pid
        st["binding"] = {"pid": pid, "pre": pre, "own": bool(own), "mark": mark}
        return True

    def _place_take_back(self):
        """Sim thread: the operator says the robot is NOT in the place it is bound to. Undo what
        this binding did to that place: one stored on this binding is forgotten (it was this
        visit's mistake); a known one gets its record back as it was before (samples, objects,
        visits) and its scene memory back to the moment of the bind. -> {pid, name, how:
        'forgotten' | 'restored' | 'kept', records: what was recorded since the bind (for the
        right place)}. Review 2026-09-25: a wrong 'known' used to leave the other room's sample,
        objects and memory records in the wrong place, for good."""
        st = self._place
        pmem = self.place_memory
        main = self._mem_main
        pid = st.get("bound")
        b = st.get("binding") or {}
        q = pmem.get(pid) if pid else None
        out = {"pid": pid, "name": (q or {}).get("name"), "how": "kept", "records": []}
        if not pid or b.get("pid") != pid:
            return out
        mk = b.get("mark")
        if mk is not None and self.memory is main and main.world == pid:
            try:
                out["records"] = _mem_rollback(main, mk)
            except Exception as e:                       # noqa: BLE001 — the place record still goes back
                self.log(f"memory: {pid} not rolled back ({type(e).__name__}: {e})")
        if b.get("own"):
            if pmem.forget(pid):
                out["how"] = "forgotten"
        elif b.get("pre") is not None and _pm_restore(pmem, pid, b["pre"]):
            out["how"] = "restored"
        st["binding"] = None
        return out

    def _place_current(self):
        """The result for the current spawn, or None (none yet, or an older spawn's)."""
        st = self._place
        return st["result"] if st["result"] is not None and st["result_seq"] == st["spawn_seq"] else None

    def _place_text(self):
        """The situation line's place clause (<= pm.VERDICT_MAX): verdict_text of the current
        result (a known place carries the confirmed changes), else what is going on; while the
        robot turns for a second look, that it is looking again (st['phase'])."""
        st = self._place
        if st.get("phase") and self._place_running():
            return st["phase"]
        rec = self._place_current()
        if rec is None:
            return "place: recognising…" if st["status"] in ("due", "running") else "place: not recognised yet"
        if rec.get("forgotten"):
            return "place: unknown (forgotten; not looked for again until the next spawn)"
        d = st.get("diff")
        return pm.verdict_text(rec, d if _changes(d) else None)

    def _place_brief(self):
        """The state feed's `place`: {verdict, name, place_id, confidence, signals, text, status,
        looks, by}. signals: what the verdict rests on in words (signals_text: 'scan +
        description', 'scan only', 'description only', 'the operator's word')."""
        st = self._place
        rec = self._place_current()
        out = {"verdict": None, "name": None, "place_id": st.get("bound"), "confidence": None, "signals": None,
               "text": self._place_text(), "status": "running" if self._place_running() else st["status"],
               "looks": st["looks"], "by": st.get("by")}
        if rec is not None:
            q = self.place_memory.get(st["bound"]) if st.get("bound") else None
            out.update(verdict="unknown" if rec.get("forgotten") else rec["verdict"],
                       confidence=rec.get("confidence"),
                       signals=None if rec.get("forgotten") else signals_text(rec),
                       name=(q or {}).get("name") if q else rec.get("name"))
        else:
            out["verdict"] = "recognising" if out["status"] in ("due", "running") else None
        return _jsonable(out)

    def _place_answer(self):
        """where_am_i / recognize_place's answer: the current result + the place's summary.
        For the bench's per-visit record it also carries signal_list (the senses as a list;
        `signals` is the words), per_look (a two-look recognition: each look's own verdict,
        confidence, parts, evidence, signals, runner-up and ranking — the top-level parts are
        the base look's only), eye_checks (each look's eye answers: eye_brief), now_placed (a
        thing boxed here before but never placed, placed by both looks now: stored, not new)
        and boxed_never_placed (the bound place's names kept by name: _place_note_unplaced)."""
        st = self._place
        if not self.awareness.get("recognize"):
            return {"ok": False, "recognize": False, "error": PLACE_OFF, "hint": PLACE_OFF_HINT}
        rec = self._place_current()
        brief = self._place_brief()
        out = {"ok": True, "recognize": True, **brief, "memory_key": getattr(self.memory, "world", None),
               "reason": st["reason"], "honesty": "a similarity (0-1) from the lidar sweep and the look's "
                                                  "embedding, not a probability; the sim's revisits start from "
                                                  "the spawn pose it learned from (an upper bound)"}
        if rec is None:
            out["detail"] = ("recognising: the robot looks once it stands still after the spawn"
                             if brief["verdict"] == "recognising" else "no place recognised yet")
            return _jsonable(out)
        if rec.get("forgotten"):
            out["detail"] = "the place the robot was in was forgotten; it looks again after the next spawn"
            return _jsonable(out)
        sec = rec.get("second") or {}
        out.update(second=sec, parts=rec.get("parts"), evidence=rec.get("evidence"),
                   signal_list=signal_list(rec),
                   age_s=round(time.time() - st["t"], 1) if st.get("t") else None)
        if rec.get("per_look"):
            out["per_look"] = rec["per_look"]
        v = rec["verdict"]
        if v == "new":
            out["nearest_known"] = {"place_id": rec.get("place_id"), "name": rec.get("name"),
                                    "confidence": rec.get("confidence")}
            out["detail"] = (f"a place I had not seen: stored as '{brief['name']}'"
                             + (f" (the nearest place I know, {rec.get('name') or rec.get('place_id')}, is "
                                f"{rec['confidence']:.2f} alike)" if rec.get("place_id") else " (the first place)"))
        elif v == "known" and rec.get("kept"):
            out["kept"] = rec["kept"]
            out["detail"] = (f"{brief['name'] or brief['place_id']}: kept ({rec['kept']}; the robot was not "
                             f"moved), {float(rec.get('confidence') or 0):.2f} alike now")
        elif v == "known":
            out["detail"] = (f"{brief['name'] or brief['place_id']}"
                             + (" (named by the operator)" if rec.get("by") == "operator" else
                                f", {rec['confidence']:.2f} alike"
                                + (f"; next best {sec.get('name') or sec.get('place_id')} {sec.get('confidence', 0):.2f}"
                                   if sec.get("place_id") else "")))
        elif v == "ambiguous":
            out["detail"] = (f"not sure: {rec.get('name') or rec.get('place_id')} {rec['confidence']:.2f} or "
                             f"{sec.get('name') or sec.get('place_id')} {float(sec.get('confidence') or 0):.2f} "
                             "— name_place settles it")
        else:
            out["detail"] = "nothing to compare (no sweep, no description)"
        if st.get("look_error") and rec.get("by") != "operator":
            out["look_error"] = st["look_error"]
            out["detail"] += f"; scan only: the look failed ({st['look_error']})"
        if rec.get("by") != "operator" and v in ("known", "new", "ambiguous"):
            out["detail"] += f" [signals: {brief['signals']}; confidence {float(rec.get('confidence') or 0):.2f}]"
        if st.get("checks"):
            out["eye_checks"] = st["checks"]
        d = st.get("diff")
        if d is not None:
            out["changes"] = {k: d.get(k) or [] for k in ("added", "missing", "moved")}
            out["pending"] = d.get("pending") or []
            if d.get("placed"):
                out["now_placed"] = d["placed"]
                out["detail"] += ("; placed now (boxed here before, never placed; not new): "
                                  + ", ".join(sorted({o["name"] for o in d["placed"]})))
            if d.get("pending"):
                out["detail"] += (f"; {len(d['pending'])} change(s) seen by one look only: not declared"
                                  + ("" if st["looks"] > 1 else " (no second look)"))
        if st.get("note"):
            out["note"] = st["note"]
        if st.get("bound"):
            out["place"] = next((p for p in self.place_memory.places() if p["id"] == st["bound"]), None)
            kept = unplaced_names(self.place_memory.get(st["bound"]))
            if kept:
                out["boxed_never_placed"] = kept
        return _jsonable(out)

    # the tools' backends (cockpit_brains.Brains.where_am_i / name_place / places / forget_place)
    async def place_where(self):
        """where_am_i: the current recognition (waits for one that is running) + the place."""
        if not self.awareness.get("recognize"):
            return {"ok": False, "recognize": False, "error": PLACE_OFF, "hint": PLACE_OFF_HINT}
        t_end = time.monotonic() + PLACE_NAME_WAIT_S
        while self._place_running() and time.monotonic() < t_end:
            await asyncio.sleep(0.05)
        return self._place_answer()

    async def place_name(self, name, new=False, rename=False):
        """name_place (see _place_name_now). Waits for a running recognition."""
        if not self.awareness.get("recognize"):
            return {"ok": False, "recognize": False, "error": PLACE_OFF, "hint": PLACE_OFF_HINT}
        nm = re.sub(r"\s+", " ", str(name if name is not None else "")).strip().strip(".!?,;:\"'")[:pm.NAME_MAX]
        t_end = time.monotonic() + PLACE_NAME_WAIT_S
        while self._place_running() and time.monotonic() < t_end:
            await asyncio.sleep(0.05)
        return await self.call(lambda: self._place_name_now(nm, bool(new), bool(rename)))

    def _place_name_now(self, nm, new, rename=False):
        """Sim thread. The operator says where the robot is:
          bound, the place has no name (or a 'new place #k' one), or it was stored on this
            visit, or rename=true -> it is renamed;
          bound, but the name is ANOTHER place's -> a correction: this is that place (settled);
          bound to a place the operator named differently -> a correction: a new place is
            stored under the name (rename=true renames instead);
          new=true -> a different, new place — but ONE per visit: a place this visit already
            stored stays the one (renamed at most), never a second copy;
          not bound (unsure / unknown) -> settled to the place of that name, else stored.
        A correction takes back what this visit wrote into the wrongly recognised place
        (_place_take_back) and moves the records since the bind to the right one."""
        st = self._place
        pmem = self.place_memory
        cur = self._place_current()
        samples = st["samples"] if cur is not None else []
        bound = st.get("bound")
        bq = pmem.get(bound) if bound else None
        if bq is None:
            bound = None
        b = st.get("binding") if bound else None
        own = bool(b and b.get("pid") == bound and b.get("own"))
        prev = (bq or {}).get("name")
        if new and rename:
            return {"ok": False, "error": "new and rename contradict each other: new=true stores a different "
                                          "place, rename=true renames this one"}
        auto = bool(new) and nm.lower() in _PLACE_NEW_WORDS
        if not nm and not auto:
            return {"ok": False, "error": "name_place needs the place's name, in the operator's words"}
        nothing = {"ok": False, "error": "nothing sensed here yet: the robot looks once it stands still after "
                                         "the spawn — try again in a few seconds", "status": st["status"]}
        prev_out, action, took = None, None, None
        others = [i for i in pmem.find(nm) if i != bound] if nm and not auto else []
        if new and bound is not None and own:
            # this visit already stored this place as new: one place, not a second copy of it
            prev_out = prev
            if auto or (prev or "").lower() == nm.lower():
                nm, action = prev or bound, "kept"
            else:
                pmem.name_place(bound, nm)
                action = "renamed"
            pid = bound
        elif new or bound is None:
            if not samples:
                return nothing
            if auto:
                nm = self._next_new_name()
            s0 = samples[0]
            ids = [] if new else pmem.find(nm)
            took = self._place_take_back() if bound is not None else None
            carry = (took or {}).get("records")
            if ids:
                pid, action = ids[0], "settled"
                pre = pmem.get(pid)
                pmem.visit(pid, scan_sig=s0["sig"], desc_emb=s0["emb"], desc_text=s0["text"] or None)
                if len(samples) > 1:
                    pmem.add_samples(pid, samples[1:])
                self._place_note_unplaced(pid, boxed_unplaced(samples))
                self._place_bind(pid, pre=pre, carry=carry)
            else:
                pid, action = self._place_enroll(samples, nm), "stored"
                self._place_bind(pid, own=True, carry=carry)
        elif prev is not None and prev.lower() == nm.lower():
            pid, action, prev_out = bound, "kept", prev
        elif others and not rename:
            # a correction: the robot took this for another place than the one the operator names
            took = self._place_take_back()
            pid, action = others[0], "settled"
            pre = pmem.get(pid)
            if samples:
                s0 = samples[0]
                pmem.visit(pid, scan_sig=s0["sig"], desc_emb=s0["emb"], desc_text=s0["text"] or None)
                if len(samples) > 1:
                    pmem.add_samples(pid, samples[1:])
                self._place_note_unplaced(pid, boxed_unplaced(samples))
            self._place_bind(pid, pre=pre, carry=took.get("records"))
        elif rename or own or prev is None or re.match(r"new place #\d+$", prev):
            pid, action, prev_out = bound, "renamed", prev
            pmem.name_place(pid, nm)
        else:
            # the robot took this for a place the operator named otherwise: a different place
            if not samples:
                return dict(nothing, hint=f"to rename '{prev}' instead: name_place(name='{nm}', rename=true)")
            took = self._place_take_back()
            pid, action = self._place_enroll(samples, nm), "stored"
            self._place_bind(pid, own=True, carry=took.get("records"))
        also = [i for i in pmem.find(nm) if i != pid]
        # the verdict now: the operator's (confidence 1.0, by 'operator'), except where it stays
        # the robot's own (a rename / a kept name of the place it recognised: its verdict and
        # confidence, with the name; 'new' stays new) and before this spawn's recognition ran
        rec = {"verdict": "known", "place_id": pid, "name": nm, "confidence": 1.0, "by": "operator",
               "second": {"place_id": None, "name": None, "confidence": 0.0}, "parts": None, "evidence": None,
               "robot": None if cur is None else {k: cur.get(k) for k in ("verdict", "place_id", "name",
                                                                          "confidence")},
               "looks": st.get("looks", 0)}
        if action in ("renamed", "kept") and cur is None:
            rec = None
        elif action in ("renamed", "kept") and cur.get("verdict") == "known" and cur.get("place_id") == pid:
            rec = dict(cur, name=(pmem.get(pid) or {}).get("name") or nm)
        elif action == "kept" and cur.get("verdict") == "new":
            rec = None                                   # the robot's own 'new' stands
        if rec is not None:
            st.update(result=rec, result_seq=st["spawn_seq"], status="done", by=rec.get("by", "operator"),
                      t=st.get("t") or time.time())
        if action not in ("renamed", "kept"):
            st["diff"] = None                            # changes against another place mean nothing here
        wrong = ""
        if took is not None and took["pid"]:
            wrong = (f" — I had taken it for '{took['name'] or took['pid']}', "
                     + ("which I had only just stored: dropped" if took["how"] == "forgotten" else
                        "which is left as it was before this visit" if took["how"] == "restored" else
                        "which keeps what it had"))
            if action == "stored" and took["how"] == "restored" and took["name"]:
                wrong += f" (if this IS '{took['name']}' under another name: name_place(name='{nm}', rename=true))"
        detail = {"renamed": f"this place is now '{nm}'" + (f" (was '{prev_out}')" if prev_out else ""),
                  "settled": f"settled: this is '{nm}', a place I knew" + (wrong or " (my recognition was not sure)"),
                  "stored": f"stored a new place: '{nm}'" + wrong,
                  "kept": (f"this place is already '{nm}'" if not new else
                           f"this place is already stored as a new place on this visit: '{nm}' (one place, not two)")
                  }[action]
        if also:
            detail += (f"; {len(also)} other place(s) carry that name too — recognition treats them as one "
                       "(they never compete)")
        self.events.append(("place", f"named: {nm}"))
        self.log(f"place: {detail}")
        try:
            self.memory.remember({"kind": "user", "text": f"this place is {nm}", "pose": self._pose_cache})
        except Exception:                                # noqa: BLE001
            pass
        try:
            self.compose_situation()
        except Exception:                                # noqa: BLE001
            pass
        return _jsonable({"ok": True, "place_id": pid, "name": nm, "previous_name": prev_out, "action": action,
                          "detail": detail, "text": self._place_text(), "also_named": also,
                          "took_back": None if took is None or not took["pid"] else
                          {"place_id": took["pid"], "name": took["name"], "how": took["how"],
                           "records_moved": len(took["records"])}})

    async def place_list(self):
        """places: every place (PlaceMemory.places(), newest visit first) + the current one."""
        ps = self.place_memory.places()
        cur = self._place.get("bound") if self.awareness.get("recognize") else None
        name = next((p["name"] or p["id"] for p in ps if p["id"] == cur), None) if cur else None
        return _jsonable({"ok": True, "recognize": bool(self.awareness.get("recognize")), "count": len(ps),
                          "current_id": cur, "current": name,
                          "places": [{k: p[k] for k in ("id", "name", "visits", "age_s", "objects", "object_names",
                                                        "samples", "last_description")} for p in ps]})

    async def place_forget(self, name):
        """forget_place: a name or an id, 'here' (the current place) or 'all'; a pronoun forgets
        nothing. The place the robot is in, forgotten: a provisional memory, no verdict until
        the next spawn (it would only store the place again). A forgotten place's scene-memory
        file stays on disk, unreachable; 'all' keeps places.json.bak."""
        return await self.call(lambda: self._place_forget_now(name))

    def _place_forget_now(self, name):
        st = self._place
        pmem = self.place_memory
        scope, key = place_scope(name)
        bound = st.get("bound") if self.awareness.get("recognize") else None
        if scope == "unclear":
            return {"ok": False, "error": f"forget which place? {name!r} names none — its name, 'here' or 'all'"}
        if scope == "here":
            if not bound:
                return {"ok": False, "error": "the robot is not in a known place right now: name the place to forget"}
            ids = [bound]
            n = pmem.forget(bound)
        elif scope == "all":
            ids = [p["id"] for p in pmem.places()]
            n = pmem.forget("all")
        else:
            ids = pmem.find(key) or ([key] if pmem.get(key) is not None else [])
            n = pmem.forget(key)
        here = bool(n) and bound in ids
        if here:
            self._place_provisional()
            st.update(result={"verdict": "unknown", "place_id": None, "name": None, "confidence": 0.0,
                              "forgotten": True}, result_seq=st["spawn_seq"], diff=None)
        detail = (f"forgot {n} place{'s' if n != 1 else ''}" + (" (including the one the robot is in)" if here else "")
                  if n else f"no place called {name!r}")
        self.events.append(("place", f"forget {name}: {n}"))
        self.log(f"place: {detail}")
        if here:
            try:
                self.compose_situation()
            except Exception:                            # noqa: BLE001
                pass
        return {"ok": bool(n) or scope == "all", "forgot": str(name), "places": n, "was_here": here,
                "detail": detail}

    # ------------------------------------------------------ tool surface
    def pose(self):
        p = self.data.xpos[self.torso]
        R = self.data.xmat[self.torso].reshape(3, 3)
        return {"x": round(float(p[0]), 3), "y": round(float(p[1]), 3),
                "yaw_deg": round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 1)}

    def _hw_brief(self):
        """The bridge's state for the 10 Hz feed: attributes only (no bus traffic)."""
        hw = self.hw
        if hw is None:
            return None
        try:
            entry = {str(leg): round(min(1.0, (time.monotonic() - b["t0"]) / max(b["T"], 1e-6)), 2)
                     for leg, b in list(getattr(hw, "_blend", {}).items())}
            return dict(port=hw.port, mirror=hw.mirror, legs=[bool(x) for x in hw.legs_present],
                        mirror_legs=[bool(x) for x in hw.mirror_legs], degraded=[bool(x) for x in hw.degraded],
                        n=len(hw.present), errors=hw.errors, port_ok=bool(hw.port_ok),
                        allow_locomotion=bool(hw.allow_locomotion), locomotion_ok=bool(hw.locomotion_ok),
                        entry=entry or None, speed_cps=hw.speed_cps,
                        tripped=sorted(int(i) for i in hw.robot.monitor.tripped))
        except Exception as e:                               # noqa: BLE001 — a feed must not die on the bridge
            return dict(port=hw.port, mirror=getattr(hw, "mirror", "?"), error=f"{type(e).__name__}: {e}",
                        legs=[False] * 5, n=0, errors=0)

    def _ckpt_fp(self):
        """Robot-fingerprint status of the loaded righter and walker checkpoints."""
        out = {}
        r = getattr(self.sup, "righter", None)
        if r is not None and getattr(r, "contract", None) is not None:
            out["righter"] = dict(status=getattr(r, "fingerprint", "unknown"),
                                  trained=r.contract.get("robot_fingerprint"), flags=list(r.contract.get("flags") or []))
        w = self.walk
        if w is not None:
            out["walker"] = dict(status=w.get("fp_status", "unknown"), trained=w["contract"].get("robot_fingerprint"),
                                 flags=list(w["contract"].get("flags") or []), name=w["name"])
        return out

    def snapshot(self):
        """Sim thread: the state feed. D052 V2: never raises — a broken field
        (a gait value that slipped past validation, a bridge mid-teardown)
        gives a partial state with 'error' instead of a 500, so the UI and the
        MCP server's liveness probe keep seeing a live cockpit."""
        try:
            return self._snapshot()
        except Exception as e:                           # noqa: BLE001 — the feed must not die
            return self._snapshot_min(e)

    def _snapshot_min(self, e):
        out = dict(t=round(self.t, 2), state=getattr(self.sup, "state", "?"), mode=self.mode,
                   world=self.world_name, tilt=0.0, events=[], event_seq=None, console=list(self.console)[-40:],
                   error=f"{type(e).__name__}: {e}", heartbeat=self.heartbeat())
        fields = [("pose", self.pose), ("hw", self._hw_brief), ("situation", self._situation_brief),
                  ("memory_objects", self.memory.to_map)]
        if self.awareness.get("recognize"):
            fields.append(("place", self._place_brief))
        for k, f in fields:
            try:
                out[k] = f()
            except Exception:                            # noqa: BLE001
                out[k] = None
        try:
            out["tilt"] = round(float(self.last["tilt"]), 1)
            out["event_seq"], ev = self.events.tail(12)
            out["events"] = _jsonable(ev)
        except Exception:                                # noqa: BLE001
            pass
        try:
            out["caps_seq"], out["caps_version"] = self._caps_brief()
        except Exception:                                # noqa: BLE001
            out["caps_seq"], out["caps_version"] = None, None
        return out

    def _snapshot(self):
        with self.lock:
            v = self.cmd_v.copy()
        self._pose_cache = self.pose()
        gs = self.goto_state
        ref = self.refusal if (self.refusal and time.time() - self.refusal[0] < 6.0) else None
        tr = self.teach
        seq, ev = self.events.tail(12)       # event_seq moves even when the 12-event window looks the same
        caps_seq, caps_version = self._caps_brief()     # D056: the tool surface's version (not an event)
        out = dict(t=round(self.t, 2), pose=self.pose(), state=self.sup.state, mode=self.mode,
                    cmd=[round(float(x), 2) for x in v], cmd_eff=[round(float(x), 2) for x in self.cmd_eff],
                    tilt=round(self.last["tilt"], 1),
                    height=round(self.last["height"] * 1000), kin_h=round(float(self.last.get("kin_h", 0.0)) * 1000),
                    contacts=[bool(c) for c in self.last["con"]],
                    gyro=round(self.last["gxy"], 2), world=self.world_name, righter=self.righter_note,
                    speed=self.speed, paused=self.paused, trips=self.sup.trip_count, falls=self.sup.fall_count,
                    recording=self.rec is not None, walk=self.walk_note, frame=self.frames["chase"][0],
                    said=list(self.said), servo=dict(self.servo.p), servo_note=self.servo.describe(),
                    browser_audio=self.browser_audio, hw=self._hw_brief(),
                    goto=None if gs is None else [gs["tx"], gs["ty"]], gesture=self.gesture_busy,
                    gesture_phase=self.gesture_phase,
                    events=_jsonable(ev), event_seq=seq, console=list(self.console)[-40:],
                    caps_version=caps_version, caps_seq=caps_seq,
                    brain=self.brain, gait=dict(T=self.gait.T, h=self.gait.h, R0=self.gait.R0,
                                                duty=self.gait.duty, hstep=self.gait.hstep,
                                                phase=round(float((self.sup.t_gait / self.gait.T) % 1.0), 3)
                                                if self.gait.T > 0 else None),
                    reflex=dict(trip=self.sup.gyro_trip, stall_s=self.sup.stall_s,
                                fallen_max_s=self.sup.fallen_max_s),
                    guards=_jsonable(self.guard_status()), refusal=None if ref is None else ref[1],
                    idle=self.idle_reason() is None, idle_reason=self.idle_reason(),
                    teach=None if tr is None else dict(samples=len(tr["qs"]), seconds=round(self.t - tr["t0"], 1),
                                                      src=tr["src"], full=bool(tr.get("full"))),
                    fingerprint=self.fingerprint, ckpt_fp=self._ckpt_fp(),
                    heartbeat=self.heartbeat(),
                    situation=self._situation_brief(), memory_objects=self.memory.to_map(),
                    awareness=dict(self.awareness))
        if self.awareness.get("recognize"):
            out["place"] = self._place_brief()          # D057: only while recognition is on
        return out

    def _situation_brief(self):
        """The situation for the 10 Hz feed: text, age and the small fields."""
        s = self.situation
        if not s.get("t"):
            return dict(text="", t=None, age_s=None)
        out = {k: s.get(k) for k in ("text", "t", "idle", "guards", "memory", "bus")}
        out["age_s"] = round(time.time() - s["t"], 1)
        lid = s.get("lidar") or {}
        out["lidar"] = {k: lid.get(k) for k in ("nearest_m", "nearest_dir", "sectors")}
        return _jsonable(out)

    async def tool_say(self, word):
        if chord_wav(word) is None:
            return {"ok": False, "error": f"unknown chord word {word!r}", "hint": f"lexicon: {', '.join(self.lexicon)}"}
        self.events.append(("say", word))
        r = await self.call(lambda: self.do(f"say {word}"))
        return {"ok": True, "word": word, "note": r}

    async def tool_gesture(self, name):
        """D052: through start_gesture (the Playground's standstill rule, the same
        one the MCP server enforces) and done only when the exit blend is."""
        if name not in self.gestures:
            return {"ok": False, "error": f"unknown gesture {name!r}", "hint": f"available: {', '.join(self.gesture_names)}"}
        fn, total = self.gestures[name]
        why = await self.call(lambda: self.start_gesture(fn, total, name))
        if why:
            return {"ok": False, "error": "busy", "hint": why}
        self.events.append(("gesture", name))
        self.mode = "gesturing"
        t_end = time.monotonic() + (total + 6.0) / max(self.speed, 0.05)     # + entry/exit blends
        await asyncio.sleep(0.1)
        while self.gesture_busy and time.monotonic() < t_end and self.alive:
            await asyncio.sleep(0.05)
        self.mode = "idle"
        return {"ok": True, "gesture": name, "duration_s": round(total, 1), "pose": self.pose(),
                "note": "rendered in physics"}

    async def tool_goto(self, x, y):
        """D052: arguments are checked BEFORE the sim thread sees them (goto(x='here')
        raised there and 500'd /api/chat; goto(NaN, 0) poisoned cmd_v), the target
        must be within GOTO_MAX_M of the robot, and a guard that forbids walking
        answers stopped='blocked' instead of a goto that can only end as 'stuck'."""
        tx, ty, err = validate_goto(x, y)
        if err:
            return {"ok": False, "error": err}
        loop = asyncio.get_running_loop()
        fut = loop.create_future()

        def start():
            pose = self.pose()
            err = goto_range_error(tx, ty, pose)
            if err:
                return {"ok": False, "error": err, "pose": pose}
            why = None
            p = self.data.xpos[self.torso]
            u = np.array([tx - p[0], ty - p[1]])
            u = u / max(float(np.linalg.norm(u)), 1e-9)
            yaw = self.yaw()
            ub = np.array([np.cos(yaw) * u[0] + np.sin(yaw) * u[1], -np.sin(yaw) * u[0] + np.cos(yaw) * u[1], 0.0])
            vb = self.void_blocks(V_GOTO * ub)
            if hasattr(self, "locomotion_held") and self.locomotion_held():
                why = "locomotion held: the real legs are mirroring (sim2real) without foot contacts"
            elif getattr(self.sup, "latched", False):
                why = f"latched safe-stop ({getattr(self.sup, 'latch_reason', '')}) — `clear` to release"
            elif vb is not None:
                why = f"blocked: void at {vb:.0f} deg (latched by the void guard) — `clear` to release"
            elif self.gesture_busy:
                why = "a gesture is running — stop first"
            if why:
                return {"ok": False, "stopped": "blocked", "detail": why, "pose": pose}
            self._start_goto(tx, ty, fut, loop)
            return None
        early = await self.call(start)
        if early is not None:
            return early
        return await fut

    async def tool_stop(self):
        def do_stop():
            with self.lock:
                self.cmd_v[:] = 0
            self.end_gesture()                  # D052: a gesture is blended out, not cut
            if self.goto_state is not None:
                self._stop_req = True
            else:
                self.sup.request_stop()
                self.mode = "safe_stop"
        await self.call(do_stop)
        if self.hw is not None and self.hw.mirror == "sim2real":
            self.hw.set_mirror("off")           # the real legs stop streaming; torque stays (they hold)
        self.events.append(("stop", None))
        return {"ok": True, "mode": self.mode}

    async def tool_scan_summary(self):
        def do_scan():
            mujoco.mj_forward(self.model, self.data)
            angles, ranges, _ = lidar_scan(self.model, self.data, self.torso)
            R = self.data.xmat[self.torso].reshape(3, 3)
            yaw = float(np.arctan2(R[1, 0], R[0, 0]))
            bear = np.degrees((angles + yaw + np.pi) % (2 * np.pi) - np.pi)
            sectors = []
            for c in range(-180, 180, 45):
                m = (bear >= c) & (bear < c + 45)
                hit = ranges[m][np.isfinite(ranges[m])]
                sectors.append({"bearing_deg": c + 22, "min_range_m": round(float(hit.min()), 3) if hit.size else None,
                                "clear": bool(hit.size == 0)})
            plane_z = float(self.data.xpos[self.torso][2] - self.z0 + PUCK_DZ)
            out = {"ok": True, "n_points": int(len(ranges)), "range_max_m": float(RANGE_MAX),
                   "world": self.world_name, "sectors": sectors,
                   "frontiers": [{"bearing_deg": s["bearing_deg"]} for s in sectors if s["clear"]],
                   "scan_plane_m": round(plane_z, 3),
                   "note": (f"2D horizontal scan from the puck ~{plane_z:.2f} m above the floor: walls and "
                            f"anything TALLER than ~{plane_z:.2f} m yes; lower obstacles (curbs, boxes, rubble, "
                            "steps) are INVISIBLE to it, and so are voids — a clear sector is not a clear path")}
            finite = np.isfinite(ranges)
            if finite.any():
                i = int(np.argmin(np.where(finite, ranges, np.inf)))
                out["nearest_obstacle_m"] = round(float(ranges[i]), 3)
                out["nearest_obstacle_bearing_deg"] = round(float(bear[i]), 1)
            return out
        out = await self.call(do_scan)
        if self.awareness.get("recognize"):
            out.pop("world", None)                  # D057: the world's name is ground truth a real robot never has
        self.events.append(("scan", out.get("nearest_obstacle_m")))
        if out.get("nearest_obstacle_m") is not None:
            try:
                self.memory.remember({"kind": "scan", "pose": self._pose_cache,
                                      "text": f"lidar: nearest {out['nearest_obstacle_m']:.2f} m at "
                                              f"{out['nearest_obstacle_bearing_deg']:.0f} deg (map bearing)"})
            except Exception:                            # noqa: BLE001
                pass
        return out

    async def tool_status(self):
        s = await self.call(self.snapshot)
        out = {"ok": True, "mode": s["mode"], "pose": s["pose"], "reflex_state": s["state"],
               "tilt_deg": s["tilt"], "world": s["world"], "battery_v": 11.9,
               "last_events": [list(e) for e in s["events"][-5:]]}
        if self.awareness.get("recognize"):
            # D057: no world name while recognition is on (ground truth a real robot never has);
            # the place the robot itself recognised instead
            out.pop("world", None)
            out["place"] = (s.get("place") or {}).get("text")
        sit = (s.get("situation") or {}).get("text")
        if sit:                                          # the awareness line (MCP clients get it here)
            out["situation"] = sit
            out["situation_age_s"] = s["situation"].get("age_s")
        return out

    async def tool_look(self, model=None):
        """The eye through the vision role's fallback chain (cockpit_brains)."""
        return await self.brains.look(model)

    TOOL_NAMES = TOOL_NAMES

    async def tool(self, name, args):
        """Every tool call (brains, MCP proxy, replays) — never raises (D052)."""
        return await self.brains.tool(name, args)

    @property
    def tools(self):
        """The harness backend surface (say/gesture/goto/...) as an object,
        for harness.intent.execute and anything else written against the
        MCP contract, routed through brains.tool (same guards and checks).
        (The Playground's `gesture` attribute is its gesture STATE, hence
        the indirection.)"""
        return self.brains.surface()

    # ------------------------------------------------------------ brains
    def llm_models(self):
        """[{id, name, vision, loaded, selector, targets, quarantined}] (blocking)."""
        return self.brains.catalog(refresh=True)[0] or []

    def claude_available(self):
        return self.brains.claude_available()

    async def chat(self, text, mode=None, model=None, source=None, trusted=False):
        return await self.brains.chat(text, mode, model, source=source, trusted=trusted)


class _SceneTail:
    """View of an MjvScene whose ngeom counts from an offset, so the
    playground's draw_hud (which starts at 0) appends instead of clearing."""

    def __init__(self, scn, n0):
        self._scn, self._n0 = scn, n0

    @property
    def ngeom(self):
        return self._scn.ngeom - self._n0

    @ngeom.setter
    def ngeom(self, v):
        self._scn.ngeom = self._n0 + v

    @property
    def maxgeom(self):
        return self._scn.maxgeom - self._n0

    @property
    def geoms(self):
        return _Shift(self._scn.geoms, self._n0)


class _Shift:
    def __init__(self, arr, n0):
        self._arr, self._n0 = arr, n0

    def __getitem__(self, i):
        return self._arr[self._n0 + i]


# ---------------------------------------------------------------- HTTP guard (D052)
TAILNET_V4 = ipaddress.ip_network("100.64.0.0/10")     # tailscale CGNAT range
TAILNET_SUFFIX = ".ts.net"                             # MagicDNS names (tailscale serve)
JSON_EXEMPT = {"/api/voice": "multipart/form-data"}    # the one non-JSON POST


def is_loopback(host):
    h = (host or "").strip("[]").lower()
    if h == "localhost":
        return True
    try:
        return ipaddress.ip_address(h).is_loopback
    except ValueError:
        return False


def _split_hostport(hp, default_port):
    """'Host: name:port' / '[::1]:port' / 'name' -> (name lowercased, port int)."""
    hp = (hp or "").strip().lower()
    if hp.startswith("["):
        name, _, rest = hp[1:].partition("]")
        port = rest[1:] if rest.startswith(":") else ""
    elif hp.count(":") == 1:
        name, _, port = hp.partition(":")
    else:
        name, port = hp, ""
    try:
        return name, int(port) if port else default_port
    except ValueError:
        return name, -1


def host_allowed(name, extra=()):
    """The Host header names this server answers to: loopback, the tailnet
    (MagicDNS *.ts.net and 100.64/10 — tailscale serve keeps the Host), this
    machine's hostname and whatever ROCKY_COCKPIT_HOSTS / extra add. Anything
    else is a DNS-rebinding attempt (a page on evil.example resolving to
    127.0.0.1 would otherwise pass the Origin == Host check)."""
    name = (name or "").strip("[]").lower()
    if not name:
        return False
    if is_loopback(name) or name.endswith(TAILNET_SUFFIX) or name in {h.lower() for h in extra}:
        return True
    try:
        if ipaddress.ip_address(name) in TAILNET_V4:
            return True
    except ValueError:
        pass
    me = socket.gethostname().lower()
    env = [h.strip().lower() for h in os.environ.get("ROCKY_COCKPIT_HOSTS", "").split(",") if h.strip()]
    return name in (me, me.split(".")[0]) or name in env


def request_refusal(method, path, headers, extra_hosts=(), check_host=True):
    """None, or (status, reason) — the D052 rules for one request:
      * Host must be one this server answers to (check_host; off with --unsafe-lan)
      * a POST carrying an Origin must be same-origin: Origin's host:port ==
        Host (default ports by the Origin's scheme — tailscale serve sends
        Host x.ts.net:9445 with Origin https://x.ts.net:9445, which passes)
      * a POST must be application/json (/api/voice: multipart/form-data) —
        a cross-site HTML form can only send urlencoded / multipart / text
        without a CORS preflight, so this closes the classic CSRF door."""
    host_hdr = headers.get("host", "")
    if check_host and not host_allowed(_split_hostport(host_hdr, 80)[0], extra_hosts):
        return 403, f"host {host_hdr!r} is not one this cockpit answers to (DNS-rebinding guard)"
    if method != "POST":
        return None
    origin = headers.get("origin")
    if origin is not None:
        if origin.strip().lower() == "null":
            return 403, "cross-origin POST refused (Origin: null)"
        o = urlsplit(origin.strip())
        dport = 443 if o.scheme == "https" else 80
        if o.scheme not in ("http", "https") or not o.hostname:
            return 403, f"cross-origin POST refused (Origin {origin!r})"
        oh = (o.hostname.lower(), o.port or dport)
        hh = _split_hostport(host_hdr, dport)
        if oh != hh:
            return 403, f"cross-origin POST refused (Origin {origin} != Host {host_hdr})"
    ctype = headers.get("content-type", "").split(";")[0].strip().lower()
    want = JSON_EXEMPT.get(path, "application/json")
    if ctype != want:
        return 415, f"POST {path} needs Content-Type {want} (got {ctype or 'none'})"
    return None


class RequestGuard:
    """Pure-ASGI middleware (BaseHTTPMiddleware buffers the endless MJPEG/SSE
    streams): applies request_refusal to every HTTP request."""

    def __init__(self, app, extra_hosts=(), check_host=True):
        self.app, self.extra_hosts, self.check_host = app, tuple(extra_hosts), check_host

    async def __call__(self, scope, receive, send):
        if scope["type"] == "http":
            headers = {k.decode("latin-1").lower(): v.decode("latin-1") for k, v in scope.get("headers", [])}
            bad = request_refusal(scope.get("method", "GET"), scope.get("path", ""), headers,
                                  self.extra_hosts, self.check_host)
            if bad is not None:
                status, why = bad
                body = json.dumps({"ok": False, "error": why}).encode()
                await send({"type": "http.response.start", "status": status,
                            "headers": [(b"content-type", b"application/json"),
                                        (b"content-length", str(len(body)).encode())]})
                await send({"type": "http.response.body", "body": body})
                return
        await self.app(scope, receive, send)


# ---------------------------------------------------------------- HTTP
def make_app(sim: CockpitSim, extra_hosts=(), check_host=True):
    """extra_hosts: more Host names to answer to (tests pass 'testserver');
    check_host=False (--unsafe-lan) answers to any Host."""
    from starlette.applications import Starlette
    from starlette.middleware import Middleware
    from starlette.responses import JSONResponse, HTMLResponse, StreamingResponse, Response
    from starlette.routing import Route

    ui_path = os.path.join(HERE, "cockpit_ui.html")

    async def index(_):
        return HTMLResponse(open(ui_path).read())

    def dead_state():
        return {"alive": False, "fatal": sim.fatal, "heartbeat": sim.heartbeat(),
                "console": list(sim.console)[-40:]}

    async def state(_):
        try:
            s = await sim.call(sim.snapshot)
        except SimDead:
            return JSONResponse(dead_state(), status_code=503)
        s["heartbeat"] = sim.heartbeat()                 # fresh, from this thread
        s["alive"] = True
        return JSONResponse(s)

    async def events(_):
        async def gen():
            while True:
                try:
                    s = await sim.call(sim.snapshot)
                    s["alive"] = True
                except SimDead:
                    yield f"data: {json.dumps(dead_state())}\n\n"
                    return
                yield f"data: {json.dumps(s)}\n\n"
                await asyncio.sleep(0.1)
        return StreamingResponse(gen(), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache"})

    async def video(request):
        cam = request.path_params["cam"]
        if cam not in sim.frames:
            return Response("no such camera", status_code=404)

        async def gen():
            last = -1
            while sim.alive:
                n, jpg = sim.frames[cam]
                if jpg and n != last:
                    last = n
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                           + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                await asyncio.sleep(0.03)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    async def frame(request):
        n, jpg = sim.frames.get(request.path_params["cam"], (0, b""))
        return Response(jpg or b"", media_type="image/jpeg")

    async def cmd(request):
        body = await request.json()
        line = str(body.get("line", "")).strip()
        if not line or line.split()[0].lower() == "quit":
            return JSONResponse({"reply": "(the header's quit button stops the server; quit is not a console command here)"})
        sim.note_cmd(line)
        r = await sim.call(lambda: sim.do(line))
        sim.log(f"> {line}")
        if r:
            for ln in str(r).splitlines():
                sim.log(ln)
        sim.note_refusal(r)
        return JSONResponse({"reply": r})

    async def teleop(request):
        body = await request.json()
        key = str(body.get("key", ""))
        sim.note_cmd("teleop " + key)
        r = await sim.call(lambda: sim.teleop(key))     # D052: in the sim thread (it touches the gesture machine)
        if r:
            sim.log(r)
        sim.note_refusal(r)
        return JSONResponse({"reply": r})

    async def chat(request):
        body = await request.json()
        r = await sim.chat(str(body.get("text", "")), body.get("mode"), body.get("model"),
                           source=body.get("source"), trusted=bool(body.get("trusted", False)))
        sim.log(f"[{r['mode']}] {body.get('text', '')[:80]}")
        for tcall in r["trace"]:
            sim.log(f"  {tcall['tool']}({json.dumps(tcall.get('args', {}))}) -> {json.dumps(tcall['result'])[:120]}")
        if r.get("reply"):
            sim.log(f"  reply: {r['reply'][:200]}")
        return JSONResponse(r)

    async def chat_clear(request):
        body = await request.json()
        sim.brains.clear(body.get("mode", "local"))
        return JSONResponse({"ok": True})

    async def brain(request):
        body = await request.json()
        return JSONResponse(await asyncio.to_thread(sim.brains.set_roles, body))

    async def models(_):
        return JSONResponse(await asyncio.to_thread(sim.brains.models_payload))

    async def world_get(_):
        return JSONResponse({"presets": list(PRESETS), "kinds": list(KINDS), "name": sim.world_name,
                             "spec": sim.world_spec})

    async def world_set(request):
        body = await request.json()
        if "preset" in body:
            name = body["preset"]
            spec = dict(PRESETS[name])
        else:
            name, spec = body.get("name", "custom"), body["spec"]
        await sim.call(lambda: sim.set_world(spec, name))
        return JSONResponse({"ok": True, "name": name, "spec": spec})

    async def world_add(request):
        body = await request.json()
        spec = json.loads(json.dumps(sim.world_spec))
        spec.setdefault("objects", [])
        if "object" in body:
            spec["objects"].append(body["object"])
        for k in ("friction", "gravity_tilt_deg", "gravity_tilt_dir_deg", "terrain", "base"):
            if k in body:
                spec[k] = body[k]
        if body.get("clear"):
            spec["objects"] = []
            spec["terrain"] = None
        await sim.call(lambda: sim.set_world(spec, "custom", keep_pose=True))   # an edit: keep the pose
        return JSONResponse({"ok": True, "spec": spec})

    async def rl_runs(_):
        from rl_dashboard import summarize_runs
        return JSONResponse({"runs": await asyncio.to_thread(summarize_runs)})

    async def rl_righter(request):
        body = await request.json()
        r = await sim.call(lambda: sim.do(f"righter {body.get('name', 'off')}"))
        sim.log(r)
        return JSONResponse({"reply": r})

    _curves = {"t": 0.0, "png": b""}

    async def rl_curves(_):
        if time.time() - _curves["t"] > 60:
            from rl_dashboard import curves
            path = await asyncio.to_thread(curves, os.path.join(HERE, "out", "rl_curves.png"))
            _curves["png"] = open(path, "rb").read()
            _curves["t"] = time.time()
        return Response(_curves["png"], media_type="image/png")

    async def look(request):
        """{model?} describes the eye; {find: 'ball', model?, method?: bbox|estimate}
        asks for the structured detection find_object uses (seen / bearing /
        distance / confidence, parsed); {question: '...', model?} asks the eye a
        custom question (the vision bench's floor-safety checks)."""
        body = await request.json() if request.headers.get("content-length", "0") != "0" else {}
        body = body if isinstance(body, dict) else {}
        if body.get("find"):
            return JSONResponse(await sim.brains.detect(str(body["find"]), body.get("model"),
                                                        method=str(body.get("method") or "bbox")))
        if body.get("question"):
            return JSONResponse(await sim.brains.look(body.get("model"), prompt=str(body["question"])[:600]))
        return JSONResponse(await sim.tool_look(body.get("model")))

    async def shove(request):
        body = await request.json()
        try:                                              # D052 V2: finite, clamped (1e9 N blew MuJoCo up)
            fx, fy, dur = shove_args(float(body.get("fx", 40)), float(body.get("fy", 0)),
                                     float(body.get("dur", 0.4)))
        except (TypeError, ValueError) as e:
            return JSONResponse({"ok": False, "error": str(e)}, status_code=400)
        if sim.sim2real:
            return JSONResponse({"ok": False, "error": "shove refused: mirroring sim->real (the real legs "
                                                      "would get the sim's reaction) — mirror off first"},
                                status_code=409)

        def do_push():
            sh = Shove(fx, fy, dur=dur, t0=sim.t)
            with sim.lock:
                sim.push = sh
            return sh.describe(sim.model)
        r = await sim.call(do_push)
        sim.log(f"shove: {r}")
        return JSONResponse({"reply": r})

    async def reset(_):
        return JSONResponse({"reply": await sim.call(sim.reset)})

    async def speed(request):
        body = await request.json()
        if "speed" in body:
            try:
                sim.set_speed(body["speed"])
            except (TypeError, ValueError) as e:
                return JSONResponse({"ok": False, "error": str(e), "speed": sim.speed, "paused": sim.paused},
                                    status_code=400)
        if "paused" in body:
            sim.paused = bool(body["paused"])
        return JSONResponse({"speed": sim.speed, "paused": sim.paused})

    async def ping(_):
        """D052 V2: liveness WITHOUT the sim thread (/api/state waits for it, so
        a 1 s console `check` made the cockpit look dead to the MCP server)."""
        hb = sim.heartbeat()
        return JSONResponse({"ok": bool(sim.alive), "heartbeat": hb}, status_code=200 if sim.alive else 503)

    VIEWS = {"follow": dict(distance=0.9, elevation=-18.0), "wide": dict(distance=2.2, elevation=-35.0),
             "top": dict(distance=2.6, elevation=-89.0), "low": dict(distance=1.0, elevation=-6.0)}

    async def camera(request):
        body = await request.json()
        if body.get("view") in VIEWS:
            sim.cam.update(VIEWS[body["view"]])
        for k in ("distance", "elevation", "azimuth"):
            if k in body:
                sim.cam[k] = float(body[k])
        return JSONResponse(sim.cam)

    async def scan(_):
        def do_scan():
            angles, ranges, origin = lidar_scan(sim.model, sim.data, sim.torso)
            R = sim.data.xmat[sim.torso].reshape(3, 3)
            yaw = float(np.arctan2(R[1, 0], R[0, 0]))
            pts = []
            for a, r in zip(angles[::4], ranges[::4]):
                if np.isfinite(r):
                    pts.append([round(float(origin[0] + r * np.cos(a + yaw)), 3),
                                round(float(origin[1] + r * np.sin(a + yaw)), 3)])
            return {"points": pts, "origin": [round(float(origin[0]), 3), round(float(origin[1]), 3)], "yaw": yaw}
        return JSONResponse(await sim.call(do_scan))

    async def tool(request):
        name = request.path_params["name"]
        try:
            args = await request.json()
        except Exception:
            args = {}
        return JSONResponse(await sim.tool(name, args or {}))

    async def favicon(_):
        return Response(b"", status_code=204)

    REC_DIR = os.path.join(HERE, "out", "recordings")

    async def record(request):
        body = await request.json()
        action = body.get("action", "start")
        if action == "start":
            if sim.rec is not None:
                return JSONResponse({"ok": False, "error": "already recording"})
            sim.rec = dict(frames=[], t0=sim.t, cmds=[], world=sim.world_name, spec=json.loads(json.dumps(sim.world_spec)))
            sim.log("● recording")
            return JSONResponse({"ok": True, "recording": True})
        rec, sim.rec = sim.rec, None
        if rec is None:
            return JSONResponse({"ok": False, "error": "not recording"})
        name = body.get("name") or time.strftime("rec_%Y%m%d_%H%M%S")
        name = "".join(c for c in name if c.isalnum() or c in "_-.")
        d = os.path.join(REC_DIR, name)
        os.makedirs(d, exist_ok=True)
        meta = dict(name=name, world=rec["world"], spec=rec["spec"], t0=rec["t0"],
                    duration_s=round(sim.t - rec["t0"], 2), cmds=rec["cmds"], frames=len(rec["frames"]))

        def write():
            import imageio
            with open(os.path.join(d, "run.json"), "w") as f:
                json.dump(meta, f, indent=1)
            if rec["frames"]:
                imageio.mimsave(os.path.join(d, "clip.mp4"), rec["frames"], fps=15, codec="libx264", quality=7)
                import shutil
                import subprocess
                if shutil.which("ffmpeg"):
                    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", os.path.join(d, "clip.mp4"),
                                    "-vf", "fps=10,scale=420:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=bayer:bayer_scale=5",
                                    os.path.join(d, "clip.gif")], check=False)
            return meta
        await asyncio.to_thread(write)
        sim.log(f"■ saved recording {name} ({meta['frames']} frames, {len(meta['cmds'])} commands, {meta['duration_s']} s)")
        return JSONResponse({"ok": True, "recording": False, "meta": meta})

    async def recordings(_):
        out = []
        if os.path.isdir(REC_DIR):
            for name in sorted(os.listdir(REC_DIR), reverse=True):
                rj = os.path.join(REC_DIR, name, "run.json")
                if os.path.exists(rj):
                    m = json.load(open(rj))
                    m["files"] = [f for f in os.listdir(os.path.join(REC_DIR, name)) if f != "run.json"]
                    out.append(m)
        return JSONResponse({"recordings": out})

    async def rec_file(request):
        from starlette.responses import FileResponse
        name, fn = request.path_params["name"], request.path_params["file"]
        path = os.path.join(REC_DIR, os.path.basename(name), os.path.basename(fn))
        if not os.path.exists(path):
            return Response("not found", status_code=404)
        return FileResponse(path)

    replay_state = {"task": None}

    async def replay(request):
        body = await request.json()
        rj = os.path.join(REC_DIR, os.path.basename(body.get("name", "")), "run.json")
        if not os.path.exists(rj):
            return JSONResponse({"ok": False, "error": "no such recording"})
        m = json.load(open(rj))
        if replay_state["task"] is not None and not replay_state["task"].done():
            replay_state["task"].cancel()
        await sim.call(lambda: sim.set_world(m["spec"], m["world"]))
        await sim.call(sim.reset)
        t_start = sim.t
        sim.log(f"▶ replaying {m['name']}: {len(m['cmds'])} commands over {m['duration_s']} s")

        async def run():
            for t_rel, line in m["cmds"]:
                while sim.t - t_start < t_rel:
                    await asyncio.sleep(0.02)
                if line.startswith("tool "):
                    _, name, args = line.split(" ", 2)
                    asyncio.ensure_future(sim.tool(name, json.loads(args)))
                elif line.startswith("teleop "):
                    sim.teleop(line[7:])
                else:
                    await sim.call(lambda l=line: sim.do(l))
            sim.log("▶ replay done")
        replay_state["task"] = asyncio.ensure_future(run())
        return JSONResponse({"ok": True, "cmds": len(m["cmds"])})

    async def world_saved(_):
        return JSONResponse({"saved": saved_worlds()})

    async def world_save(request):
        body = await request.json()
        name = save_world(body.get("name", "world"), sim.world_spec)
        return JSONResponse({"ok": True, "name": name, "saved": saved_worlds()})

    async def world_load(request):
        body = await request.json()
        spec = load_world(os.path.basename(body["name"]))
        await sim.call(lambda: sim.set_world(spec, body["name"]))
        return JSONResponse({"ok": True, "spec": spec})

    async def world_random(request):
        body = await request.json()
        spec = random_course(int(body.get("seed", 0)), int(body.get("n", 8)))
        name = f"random #{int(body.get('seed', 0))}"
        await sim.call(lambda: sim.set_world(spec, name))
        return JSONResponse({"ok": True, "name": name, "spec": spec})

    async def rl_walk(request):
        body = await request.json()
        r = await sim.call(lambda: sim.set_walk(body.get("name", "off")))
        sim.log(r)
        return JSONResponse({"reply": r})

    async def voice(request):
        """Browser audio blob -> whisper -> {ok, text, wake, motion_blocked, dropped}.
        Transcribe only: the UI shows the text and sends it to /api/chat with
        source='voice' (trusted=true once the operator confirmed it)."""
        form = await request.form()
        up = form.get("audio")
        if up is None:
            return JSONResponse({"ok": False, "error": "no audio"})
        r = await sim.brains.transcribe(await up.read(), mode=form.get("mode"),
                                        trusted=str(form.get("trusted", "")).lower() in ("1", "true", "yes"))
        if r.get("text"):
            sim.log(f"🎤 {r['text']}")
        elif r.get("dropped"):
            sim.log(f"🎤 (dropped: {'; '.join(r['dropped'])[:160]})")
        return JSONResponse(r)

    async def quit_(_):
        sim.log("quit requested from the page")
        try:
            await asyncio.to_thread(sim.hw_disconnect)     # D052: real legs limp before the process goes
        except Exception as e:                             # noqa: BLE001
            sim.log(f"hw disconnect on quit: {e}")
        sim.alive = False
        sim.flush_memories()                               # the debounced saves, before the process goes

        def bye():
            time.sleep(0.4)
            os._exit(0)
        threading.Thread(target=bye, daemon=True).start()
        return JSONResponse({"ok": True})

    # ---------------------------------------------------------- D051: audio in the browser
    async def audio_file(request):
        wav = chord_wav(request.path_params["word"])
        if wav is None:
            return JSONResponse({"error": "no such word"}, status_code=404)
        return Response(open(wav, "rb").read(), media_type="audio/wav", headers={"Cache-Control": "max-age=60"})

    async def audio_mode(request):
        body = await request.json()
        sim.browser_audio = bool(body.get("browser", True))
        return JSONResponse({"browser": sim.browser_audio, "player": sim.player})

    def _chord_mod():
        sys.path.insert(0, AUDIO_DIR)
        import chordspeak2
        return chordspeak2

    def _wav_bytes(buf):
        import wave
        bio = io.BytesIO()
        with wave.open(bio, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(_chord_mod().SR)
            w.writeframes((np.clip(buf, -1, 1) * 32767).astype(np.int16).tobytes())
        return bio.getvalue()

    def _render_spec(spec):
        cs = _chord_mod()
        syls = []
        for sy in spec.get("syllables", []):
            d = {k: (float(v) if k not in ("ratios",) else [float(x) for x in v]) for k, v in sy.items()
                 if k in ("dur", "gap", "ratios", "root_mul", "amp", "rough", "breath", "bend_cents",
                          "detune_cents", "wobble", "attack", "tail", "tilt")}
            d.setdefault("dur", 0.2)
            d.setdefault("ratios", [1.0, 1.5, 2.0])
            d["dur"] = float(np.clip(d["dur"], 0.03, 2.0))
            syls.append(d)
        if not syls:
            syls = [dict(dur=0.2, ratios=[1.0, 1.5, 2.0])]
        return cs.word(syls, root_hz=float(np.clip(spec.get("root", 120.0), 40.0, 400.0)))

    async def chord_list(_):
        cs = _chord_mod()
        canon = {k: {"root": v[0], "syllables": v[1]} for k, v in cs.vocabulary().items()}
        custom = {}
        if os.path.isdir(CHORD_SPEC_DIR):
            for fn in sorted(os.listdir(CHORD_SPEC_DIR)):
                if fn.endswith(".json"):
                    custom[fn[:-5]] = json.load(open(os.path.join(CHORD_SPEC_DIR, fn)))
        return JSONResponse({"canon": canon, "custom": custom, "lexicon": sim.lexicon})

    async def chord_render(request):
        spec = await request.json()
        buf = await asyncio.get_running_loop().run_in_executor(None, _render_spec, spec)
        return Response(_wav_bytes(buf), media_type="audio/wav")

    async def chord_save(request):
        body = await request.json()
        name = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(body.get("name", "")).strip())[:32]
        if not name or name in CHORD_WORDS:
            return JSONResponse({"ok": False, "error": "need a new name (canon words are read-only)"})
        spec = body.get("spec") or {}
        buf = await asyncio.get_running_loop().run_in_executor(None, _render_spec, spec)
        os.makedirs(CHORD_SPEC_DIR, exist_ok=True)
        os.makedirs(CHORD_CUSTOM_DIR, exist_ok=True)
        with open(os.path.join(CHORD_SPEC_DIR, f"{name}.json"), "w") as f:
            json.dump(spec, f, indent=1)
        with open(os.path.join(CHORD_CUSTOM_DIR, f"{name}.wav"), "wb") as f:
            f.write(_wav_bytes(buf))
        sim.reload_library()
        sim.log(f"chord word saved: {name} ({len(buf) / _chord_mod().SR:.2f} s)")
        return JSONResponse({"ok": True, "name": name, "lexicon": sim.lexicon})

    async def chord_delete(request):
        body = await request.json()
        name = str(body.get("name", ""))
        n = 0
        for p in (os.path.join(CHORD_SPEC_DIR, f"{name}.json"), os.path.join(CHORD_CUSTOM_DIR, f"{name}.wav")):
            if name and name not in CHORD_WORDS and os.path.exists(p):
                os.remove(p)
                n += 1
        sim.reload_library()
        return JSONResponse({"ok": n > 0, "lexicon": sim.lexicon})

    # ---------------------------------------------------------- D051: gesture studio
    async def gesture_list(_):
        kf = {k: kg.spec for k, kg in load_keyframe_gestures().items()}
        return JSONResponse({"code": [g for g in sim.gesture_names if g not in kf], "keyframe": kf,
                             "all": sim.gesture_names, "dir": os.path.relpath(GESTURE_DIR, ROOT)})

    def _spec_of(body):
        """A studio body: the spec itself or {spec, force}; returns (spec, force)."""
        if not isinstance(body, dict):
            raise ValueError("expected a JSON object")
        spec = body.get("spec") if isinstance(body.get("spec"), dict) else dict(body)
        force = bool(body.get("force", False))
        spec = {k: v for k, v in spec.items() if k not in ("force", "t", "fs")}
        return spec, force

    def _report_json(rep):
        """The studio's verdict: headline + the numbers the UI colours."""
        pk, leg, joint, tpk = rep.peak() if np.isfinite(rep.vmax).any() else (0.0, 0, "yaw", 0.0)
        lim = pf.speed_limits()
        return _jsonable(dict(
            ok=bool(rep.ok), feasible=bool(rep.ok), verdict=rep.lines[0] if rep.lines else "",
            lines=list(rep.lines), fails=rep.fails, warnings=rep.warnings, codes=dict(rep.codes),
            summary=dict(peak_rad_s=round(pk, 2), peak_leg=leg, peak_joint=joint, peak_t=round(tpk, 2),
                         limits=lim, margin_mm=rep.margin_min, support_min=rep.support_min,
                         jumps=len(rep.jumps), self_contacts=rep.self_contacts,
                         thermal="THERMAL" in rep.codes, claw_rad_s=rep.claw_vmax, total_s=rep.total),
            report=rep.to_json()))

    async def _check_spec(spec):
        return await asyncio.to_thread(pf.check_spec, spec, sim.gait, 100.0, sim.model)

    async def gesture_check(request):
        """D052: the studio's verdict for a spec (never moves anything)."""
        try:
            spec, _ = _spec_of(await request.json())
        except ValueError as e:
            return JSONResponse({"ok": False, "feasible": False, "error": str(e)})
        return JSONResponse(_report_json(await _check_spec(spec)))

    async def gesture_save(request):
        """D052: refused on a FAIL verdict unless force; never over a code gesture."""
        try:
            spec, force = _spec_of(await request.json())
        except ValueError as e:
            return JSONResponse({"ok": False, "error": str(e)})
        name = str(spec.get("name", ""))
        g = sim.gestures.get(name)
        if g is not None and not hasattr(g[0], "spec"):
            return JSONResponse({"ok": False, "error": f"{name!r} is a built-in gesture — pick another name"})
        rep = await _check_spec(spec)
        verdict = _report_json(rep)
        if not rep.ok and not force:
            return JSONResponse({"ok": False, "error": "not feasible — not saved (tick 'force' to save it anyway)",
                                 "check": verdict})
        try:
            path = await asyncio.to_thread(save_keyframe_gesture, spec, GESTURE_DIR, force, sim.gait)
        except (ValueError, KeyError, TypeError) as e:
            return JSONResponse({"ok": False, "error": str(e), "check": verdict})
        sim.reload_library()
        sim.log(f"gesture saved: {os.path.relpath(path, ROOT)}" + ("" if rep.ok else " (FORCED past a FAIL)"))
        return JSONResponse({"ok": True, "name": os.path.basename(path)[:-5], "all": sim.gesture_names,
                             "forced": bool(force and not rep.ok), "check": verdict})

    async def gesture_delete(request):
        body = await request.json()
        ok = delete_keyframe_gesture(str(body.get("name", "")))
        sim.reload_library()
        return JSONResponse({"ok": ok, "all": sim.gesture_names})

    async def gesture_preview(request):
        body = await request.json()
        if body.get("off"):
            await sim.call(sim.preview_off)
            return JSONResponse({"ok": True})
        spec = body.get("spec") or {}
        try:
            total = await sim.call(lambda: sim.preview_pose(spec, body.get("t")))
        except (ValueError, KeyError, TypeError) as e:
            return JSONResponse({"ok": False, "error": str(e)})
        return JSONResponse({"ok": True, "total": total})

    async def gesture_play(request):
        """Play an UNSAVED keyframe spec once (the studio's ▶). D052: checked
        first — a FAIL is refused unless force; through start_gesture (blended)."""
        try:
            spec, force = _spec_of(await request.json())
            kg = KeyframeGesture(spec)
        except (ValueError, KeyError, TypeError) as e:
            return JSONResponse({"ok": False, "error": str(e)})
        rep = await _check_spec(spec)
        verdict = _report_json(rep)
        if not rep.ok and not force:
            return JSONResponse({"ok": False, "error": "not feasible — not played (tick 'force' to play it anyway)",
                                 "check": verdict})
        why = await sim.call(lambda: sim.start_gesture(kg, kg.total, spec.get("name") or "studio"))
        if why:
            return JSONResponse({"ok": False, "error": f"refused: {why}", "check": verdict})
        sim.mode = "gesturing"
        for t_cue, w in kg.cues:                       # chord cues fire on their frame (after the entry blend)
            asyncio.get_running_loop().call_later((t_cue + 0.3) / max(sim.speed, 0.05), lambda w=w: asyncio.ensure_future(sim.tool_say(w)))
        return JSONResponse({"ok": True, "total": kg.total, "check": verdict})

    async def gesture_solve(request):
        """D052 REACH: whole-body pose for 'put leg L's hand at (x, y, z)' (GROUND
        frame mm: the unposed stance's body frame, floor at z = -h). ~1-3 s."""
        import pebble_pose_solver as ps
        body = await request.json()
        try:
            leg = int(body.get("leg", 0))
            tgt = [float(x) for x in body.get("target", [])]
            if leg not in range(5) or len(tgt) != 3 or not np.isfinite(tgt).all():
                raise ValueError("need leg 0..4 and target [x, y, z] (mm, finite)")
            kw = dict(keep_margin_mm=float(body.get("keep_margin_mm", 25.0)))
            if body.get("yaw_deg") is not None:
                kw["yaw_deg"] = float(body["yaw_deg"])
        except (TypeError, ValueError) as e:
            return JSONResponse({"ok": False, "error": str(e)})
        try:
            r = await asyncio.to_thread(ps.solve_reach, sim.gait, leg, tuple(tgt), None, **kw)
        except (ValueError, KeyError) as e:
            return JSONResponse({"ok": False, "error": str(e)})
        return JSONResponse(_jsonable(r))

    async def gesture_teach(request):
        """D052 TEACH: start / stop / status of the pose-stream recorder. stop ->
        keyframes_from_stream -> a spec for the studio + its feasibility verdict."""
        import pebble_pose_solver as ps
        body = await request.json()
        act = str(body.get("action", "status"))
        if act == "start":
            if sim.teach is not None:
                return JSONResponse({"ok": False, "error": "already recording"})
            r = await sim.call(sim.teach_start)
            sim.log(f"teach: recording the pose stream at {TEACH_HZ:.0f} Hz")
            return JSONResponse(r)
        if act == "status":
            tr = sim.teach
            return JSONResponse({"ok": True, "recording": tr is not None,
                                 "samples": 0 if tr is None else len(tr["qs"])})
        if act != "stop":
            return JSONResponse({"ok": False, "error": f"unknown action {act}"})
        got = await sim.call(sim.teach_stop)
        if got is None:
            return JSONResponse({"ok": False, "error": "not recording"})
        qs, claw, meta = got
        if len(qs) < 4:
            return JSONResponse({"ok": False, "error": f"only {len(qs)} samples — record at least 0.2 s"})
        name = str(body.get("name") or "taught")
        try:
            spec = await asyncio.to_thread(ps.keyframes_from_stream, qs, TEACH_HZ, claw,
                                           float(body.get("tol_deg", 2.0)), name, sim.gait)
        except (ValueError, KeyError) as e:
            return JSONResponse({"ok": False, "error": str(e), "meta": meta})
        verdict = _report_json(await _check_spec(spec))
        sim.log(f"teach: {meta['samples']} samples ({meta['seconds']} s, {meta['src']}) -> "
                f"{len(spec.get('keyframes', []))} keyframes; {verdict['verdict']}")
        return JSONResponse(_jsonable({"ok": True, "spec": spec, "meta": meta, "check": verdict}))

    async def model_info(_):
        """D052: what the sim believes the robot is (rocky_model) + fingerprints."""
        import pebble_pose_solver as ps

        def build():
            g = sim.gait
            p = rm.params()
            return dict(
                params_rev=rm.params_rev(), fingerprint=sim.fingerprint, fingerprint_note=fingerprint_note(sim.model),
                limits_deg=rm.joint_limits_deg(), claw_deg=list(rm.claw_limits("deg")),
                speeds=dict(pf.speed_limits(), no_load=rm.no_load_rad_s()),
                actuator=rm.actuator(), claw_actuator=rm.actuator("claw"),
                continuous_nm=rm.continuous_nm(), damping_nms=rm.damping_nms(),
                bus=dict(hz=rm.bus_hz(), latency_s=rm.latency_s()),
                gait_defaults=rm.gait_defaults(), reflex_defaults=rm.reflex_defaults(),
                gait={k: float(getattr(g, k)) for k in GAIT_KEYS}, envelope=g.max_command(),
                body=dict(circumradius_mm=float(p["body"]["circumradius"]), stance_radius_mm=float(g.R0),
                          body_height_mm=float(g.h), stance_torso_z_m=rm.stance_torso_z_m(g.h),
                          foot_contact_radius_mm=rm.foot_contact_radius_mm()),
                studio=dict(body_xy_mm=ps.OFFSET_XY_MM, body_z_mm=list(ps.OFFSET_Z_MM), yaw_deg=ps.YAW_MAX_DEG,
                            reach_xy_mm=round(float(g.R0) + 140.0), reach_z_mm=[-float(g.h), 160.0],
                            margin_fail_mm=pf.MARGIN_FAIL_MM, margin_warn_mm=pf.MARGIN_WARN_MM),
                p_nom=np.round(np.asarray(g.p_nom, float), 1).tolist(),
                leg_ids=rm.leg_ids(), hand_ids=rm.hand_ids(),
                checkpoints=sim._ckpt_fp())
        return JSONResponse(_jsonable(await asyncio.to_thread(build)))

    async def gait_list(_):
        return JSONResponse({"presets": gait_presets(), "current": {k: float(getattr(sim.gait, k)) for k in GAIT_KEYS},
                             "envelope": _jsonable(sim.gait.max_command())})

    # ---------------------------------------------------------- D051: servo realism
    async def servo_set(request):
        body = await request.json()
        p = await sim.call(lambda: sim.servo.set(**body))
        sim.log(sim.servo.describe())
        return JSONResponse({"servo": p, "note": sim.servo.describe()})

    # ---------------------------------------------------------- D051: hardware
    def _hw_or_err():
        return sim.hw

    async def hw_status(_):
        hw = _hw_or_err()
        return JSONResponse({"connected": hw is not None, "ports": serial_ports(), "mirrors": list(MIRRORS),
                             "status": None if hw is None else hw.status()})

    async def hw_action(request):
        body = await request.json()
        act = str(body.get("action", ""))
        loop = asyncio.get_running_loop()
        try:
            if act == "connect":
                st = await loop.run_in_executor(None, sim.hw_connect, str(body.get("port", "mock")))
                return JSONResponse({"ok": True, "status": st})
            if act == "disconnect":
                return JSONResponse({"ok": await loop.run_in_executor(None, sim.hw_disconnect)})
            hw = _hw_or_err()
            if hw is None:
                return JSONResponse({"ok": False, "error": "not connected"})
            if act == "scan":
                found = await loop.run_in_executor(None, hw.scan)
                return JSONResponse({"ok": True, "found": {str(k): v for k, v in found.items()}, "status": hw.status()})
            if act == "mirror":
                mode = str(body.get("mode", "off"))
                if mode == "sim2real":
                    why = sim.sim2real_refusal()                # D052: the bridge refuses too; say why here
                    if why:
                        return JSONResponse({"ok": False, "error": f"sim2real refused: {why}",
                                             "mirror": hw.mirror})
                return JSONResponse({"ok": True, "mirror": await loop.run_in_executor(None, hw.set_mirror, mode)})
            if act == "limits":
                ids = body.get("ids")
                ids = None if ids in (None, "", []) else [int(i) for i in ids]
                res = await loop.run_in_executor(None, lambda: hw.apply_limits(
                    ids, float(body.get("margin", 2.0)), bool(body.get("dry_run", False))))
                return JSONResponse({"ok": True, "dry_run": bool(body.get("dry_run", False)),
                                     "result": _jsonable(res)})
            if act == "torque":
                ids = body.get("ids")
                ids = None if ids is None else [int(i) for i in ids]
                on = bool(body.get("on", True))
                return JSONResponse({"ok": True, "ids": await loop.run_in_executor(None, hw.torque, on, ids)})
            if act == "limp":
                return JSONResponse({"ok": True, "ids": await loop.run_in_executor(None, hw.limp)})
            if act == "jog":
                return JSONResponse({"ok": True, "deg": hw.jog(int(body["id"]), float(body["deg"]), int(body.get("speed", 200)))})
            if act == "set_id":
                return JSONResponse({"ok": True, "id": hw.set_id(int(body["old"]), int(body["new"]))})
            if act == "center":
                return JSONResponse({"ok": True, **hw.center(str(body["key"]), body.get("jig_deg"))})
            if act == "dir":
                return JSONResponse({"ok": True, "dir": hw.set_dir(str(body["key"]), int(body.get("dir", 1)))})
            if act == "speed":
                hw.speed_cps = max(0, int(body.get("speed_cps", hw_bridge.ENTRY_SPEED_CPS)))
                return JSONResponse({"ok": True, "speed_cps": hw.speed_cps})
            return JSONResponse({"ok": False, "error": f"unknown action {act}"})
        except Exception as e:                          # a bus error is an answer, not a 500
            sim.log(f"hw {act}: {type(e).__name__}: {e}")
            return JSONResponse({"ok": False, "error": f"{type(e).__name__}: {e}"})

    # ---------------------------------------------------------- scene memory + awareness
    async def _json_object(request):
        """The request body as a dict, or None when it is not JSON (a 400, not a 500)."""
        try:
            body = await request.json()
        except ValueError:
            return None
        return body if isinstance(body, dict) else {}

    def _not_json():
        return JSONResponse({"ok": False, "error": "body must be JSON"}, status_code=400)

    def _memory_view(n=20):
        pose = sim._pose_cache
        out = {"ok": True, "world": sim.world_name, "objects": sim.memory.to_map(),
               "observations": sim.memory.last(n), "summary": sim.memory.summary(pose),
               "situation": sim.situation, "awareness": sim.awareness_settings(),
               "stats": sim.memory.stats(),
               # D057: the places the robot knows (+ the one it is in while recognition is on)
               "places": sim.place_memory.places()}
        if sim.awareness.get("recognize"):
            out["place"] = sim._place_brief()
        return _jsonable(out)

    async def memory_get(request):
        """GET /api/memory?n=20: the remembered objects (for the map), the last n
        observations (newest first), the memory summary and the situation."""
        try:
            n = int(request.query_params.get("n", 20))
        except ValueError:
            n = 20
        return JSONResponse(_memory_view(max(0, min(200, n))))

    async def memory_post(request):
        """POST /api/memory {action: remember {note} | {name, x, y} (pin on the
        map) | forget {name} | clear | recall {query, k?} | where_is {name}}."""
        body = await _json_object(request)
        if body is None:
            return _not_json()
        act = str(body.get("action", ""))
        b = sim.brains
        if act == "remember":
            r = await b.mem_remember(note=body.get("note"), name=body.get("name"), x=body.get("x"), y=body.get("y"))
        elif act == "forget":
            r = await b.mem_forget(body.get("name"))
        elif act == "clear":
            r = await b.mem_forget("all")
        elif act == "recall":
            r = await b.mem_recall(body.get("query", ""), body.get("k", 5))
        elif act == "where_is":
            r = await b.mem_where_is(body.get("name"))
        else:
            return JSONResponse({"ok": False, "error": f"unknown action {act!r} (remember | forget | clear | "
                                                       "recall | where_is)"}, status_code=400)
        view = _memory_view(10)
        return JSONResponse(_jsonable(dict(r, objects=view["objects"], stats=view["stats"])))

    async def awareness_get(_):
        return JSONResponse(_jsonable({"ok": True, "awareness": sim.awareness_settings(),
                                       "situation": sim.situation}))

    async def awareness_post(request):
        """POST /api/awareness {interval_s?, curious?, reactions?, recognize?, refresh?}
        (refresh: true composes a situation now; recognize: D057 place recognition)."""
        body = await _json_object(request)
        if body is None:
            return _not_json()
        try:
            a = sim.set_awareness(body.get("interval_s"), body.get("curious"), body.get("reactions"),
                                  body.get("recognize"))
        except (TypeError, ValueError) as e:
            return JSONResponse({"ok": False, "error": str(e), "awareness": sim.awareness_settings()},
                                status_code=400)
        sit = await sim.situation_now() if body.get("refresh") else sim.situation
        out = {"ok": True, "awareness": a, "situation": sit}
        if sim.awareness.get("recognize"):
            out["place"] = sim._place_brief()
        return JSONResponse(_jsonable(out))

    async def place_get(_):
        """GET /api/place (D057): the current recognition (where_am_i's answer) + every place."""
        cur = sim._place_answer() if sim.awareness.get("recognize") else None
        return JSONResponse(_jsonable({"ok": True, "recognize": bool(sim.awareness.get("recognize")),
                                       "place": cur, "places": sim.place_memory.places()}))

    async def place_post(request):
        """POST /api/place (D057) {action: recognize {force?} (waits for the answer: a running one,
        or one run now) | name {name, new?, rename?} | forget {name} | list}."""
        body = await _json_object(request)
        if body is None:
            return _not_json()
        act = str(body.get("action", ""))
        if act == "recognize":
            force = body.get("force", False)
            if not isinstance(force, bool):
                return JSONResponse({"ok": False, "error": "force must be true or false"}, status_code=400)
            r = await sim.recognize_place("operator: /api/place", force=force)
        elif act == "name":
            r = await sim.brains.tool("name_place", {"name": body.get("name"), "new": bool(body.get("new", False)),
                                                     "rename": bool(body.get("rename", False))})
        elif act == "forget":
            r = await sim.brains.tool("forget_place", {"name": body.get("name")})
        elif act == "list":
            r = await sim.brains.tool("places", {})
        else:
            return JSONResponse({"ok": False, "error": f"unknown action {act!r} (recognize | name | forget | "
                                                       "list)"}, status_code=400)
        return JSONResponse(_jsonable(r))

    async def capabilities_(_):
        """D056: what this cockpit can do right now (harness.capabilities); `version`
        changes when a gesture or chord word is saved or deleted or the gait changes."""
        return JSONResponse(sim._caps_current())

    async def help_(_):
        return JSONResponse({"teleop": TELEOP_HELP, "rl": HELP_RL,
                             "commands": __import__("playground").__doc__.split("Commands")[1].split("HONESTY")[0]})

    routes = [
        Route("/", index), Route("/api/state", state), Route("/api/events", events), Route("/api/ping", ping),
        Route("/video/{cam}.mjpg", video), Route("/frame/{cam}.jpg", frame),
        Route("/api/cmd", cmd, methods=["POST"]), Route("/api/teleop", teleop, methods=["POST"]),
        Route("/api/chat", chat, methods=["POST"]), Route("/api/chat/clear", chat_clear, methods=["POST"]),
        Route("/api/brain", brain, methods=["POST"]), Route("/api/models", models),
        Route("/api/world", world_get), Route("/api/world", world_set, methods=["POST"]),
        Route("/api/world/add", world_add, methods=["POST"]),
        Route("/api/rl/runs", rl_runs), Route("/api/rl/righter", rl_righter, methods=["POST"]),
        Route("/api/rl/curves.png", rl_curves),
        Route("/api/look", look, methods=["POST"]), Route("/api/shove", shove, methods=["POST"]),
        Route("/api/reset", reset, methods=["POST"]), Route("/api/speed", speed, methods=["POST"]),
        Route("/api/camera", camera, methods=["POST"]), Route("/api/scan", scan), Route("/api/tool/{name}", tool, methods=["POST"]),
        Route("/api/help", help_), Route("/favicon.ico", favicon), Route("/api/capabilities", capabilities_),
        Route("/api/record", record, methods=["POST"]), Route("/api/recordings", recordings),
        Route("/recordings/{name}/{file}", rec_file), Route("/api/replay", replay, methods=["POST"]),
        Route("/api/world/saved", world_saved), Route("/api/world/save", world_save, methods=["POST"]),
        Route("/api/world/load", world_load, methods=["POST"]), Route("/api/world/random", world_random, methods=["POST"]),
        Route("/api/rl/walk", rl_walk, methods=["POST"]), Route("/api/voice", voice, methods=["POST"]),
        Route("/api/quit", quit_, methods=["POST"]),
        Route("/audio/{word}.wav", audio_file), Route("/api/audio", audio_mode, methods=["POST"]),
        Route("/api/chord/list", chord_list), Route("/api/chord/render", chord_render, methods=["POST"]),
        Route("/api/chord/save", chord_save, methods=["POST"]), Route("/api/chord/delete", chord_delete, methods=["POST"]),
        Route("/api/gesture/list", gesture_list), Route("/api/gesture/save", gesture_save, methods=["POST"]),
        Route("/api/gesture/delete", gesture_delete, methods=["POST"]), Route("/api/gesture/preview", gesture_preview, methods=["POST"]),
        Route("/api/gesture/play", gesture_play, methods=["POST"]),
        Route("/api/gesture/check", gesture_check, methods=["POST"]),
        Route("/api/gesture/solve", gesture_solve, methods=["POST"]),
        Route("/api/gesture/teach", gesture_teach, methods=["POST"]),
        Route("/api/model", model_info), Route("/api/gait", gait_list),
        Route("/api/servo", servo_set, methods=["POST"]),
        Route("/api/hw", hw_status), Route("/api/hw", hw_action, methods=["POST"]),
        Route("/api/memory", memory_get), Route("/api/memory", memory_post, methods=["POST"]),
        Route("/api/awareness", awareness_get), Route("/api/awareness", awareness_post, methods=["POST"]),
        Route("/api/place", place_get), Route("/api/place", place_post, methods=["POST"]),
    ]

    async def sim_dead(_request, exc):
        return JSONResponse({"ok": False, "error": f"sim thread stopped: {exc}", "alive": False}, status_code=503)

    from cockpit_shared import routes as shared_routes
    routes += shared_routes(sim)          # GET/POST /api/ui, GET /api/ui/events (4 Hz SSE), GET /api/guide (docs/COCKPIT_GUIDE.md)

    @contextlib.asynccontextmanager
    async def lifespan(_app):
        # the server's loop, known before any request: the sim thread schedules work on it
        # (a place recognition after a --recognize start with no browser open yet; D057)
        sim._aloop = asyncio.get_running_loop()
        yield

    return Starlette(routes=routes, exception_handlers={SimDead: sim_dead}, lifespan=lifespan,
                     middleware=[Middleware(RequestGuard, extra_hosts=tuple(extra_hosts), check_host=check_host)])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=int(os.environ.get("ROCKY_COCKPIT_PORT", "8765")))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--unsafe-lan", action="store_true",
                    help="allow a non-loopback --host. NOTHING in the cockpit is authenticated: anyone who "
                         "reaches the port drives the robot (and the real servos when connected). Prefer "
                         "./rocky.sh tailnet (tailscale serve, tailnet-only HTTPS)")
    ap.add_argument("--world", default=os.environ.get("ROCKY_WORLD", "flat"))
    ap.add_argument("--brain", default="talk", choices=["talk", "local", "multimodal", "claude"])
    ap.add_argument("--awareness-s", type=float, default=AWARENESS_S,
                    help="compose the situation every N s while idle (0 = off; default %(default)s)")
    ap.add_argument("--curious", action="store_true",
                    help="allow one unprompted look (a vision-model call) per minute when the lidar scene changed")
    ap.add_argument("--no-reactions", action="store_true",
                    help="never speak a chord on the robot's own initiative (guard latch / new obstacle)")
    ap.add_argument("--no-memory", action="store_true",
                    help="keep the scene memory in RAM only (default: sim/out/memory/<world>.json, "
                         "or ROCKY_MEMORY_DIR)")
    ap.add_argument("--recognize", action="store_true",
                    help="D057 place recognition: key the scene memory by the place the robot recognises "
                         "(lidar + a look + its embedding) instead of the world's name; places in "
                         "<memory dir>/places.json (off by default; POST /api/awareness {recognize} switches it)")
    args = ap.parse_args(argv)
    if not is_loopback(args.host) and not args.unsafe_lan:
        print(f"cockpit: refusing --host {args.host}: the cockpit has NO authentication — anyone who can reach "
              "the port drives the robot. Use ./rocky.sh tailnet (tailscale serve keeps it on 127.0.0.1) "
              "or pass --unsafe-lan if you really mean it.", file=sys.stderr, flush=True)
        raise SystemExit(2)
    if args.unsafe_lan:
        print(f"cockpit: WARNING --unsafe-lan: serving on {args.host} with NO authentication and no Host "
              "check — every device that reaches this port can drive the robot.", file=sys.stderr, flush=True)
    import uvicorn
    sim = CockpitSim(args.world if args.world in PRESETS else "flat")
    sim.brain["mode"] = args.brain
    if not args.no_memory:
        mem_dir = os.environ.get("ROCKY_MEMORY_DIR") or MEMORY_DIR
        sim.memory.set_directory(mem_dir)
        st = sim.memory.stats()
        print(f"cockpit: scene memory {st['path']} ({st['observations']} observations, {st['objects']} objects)",
              flush=True)
        sim.place_memory.set_directory(mem_dir)       # D057: <dir>/places.json
        ps = sim.place_memory.stats()
        print(f"cockpit: places {ps['path']} ({ps['places']} places, {ps['named']} named)", flush=True)
    # recognition first (D057): with it on, the start epoch goes to the provisional memory
    sim.set_awareness(args.awareness_s, args.curious, not args.no_reactions, bool(args.recognize))
    sim._memory_epoch(f"cockpit started in world '{sim.world_name}': its objects are at their spawn")
    sim.exit_on_fatal = True                     # D052: a dead sim thread ends the process (exit 1)
    threading.Thread(target=sim.run_forever, daemon=True).start()
    app = make_app(sim, check_host=not args.unsafe_lan)
    print(f"cockpit: http://{args.host}:{args.port}  (world {sim.world_name}; {sim.righter_note})", flush=True)
    # uvicorn's graceful shutdown waits for open connections, and this server's
    # connections are endless streams (MJPEG, SSE): a Ctrl-C or `cockpit-stop`
    # would hang and stale instances pile up. uvicorn installs its OWN
    # SIGINT/SIGTERM handlers when it starts (Server.capture_signals), so a
    # handler set before uvicorn.run() is silently replaced; the exit has to be
    # the server's handle_exit itself.
    server = uvicorn.Server(uvicorn.Config(app, host=args.host, port=args.port, log_level="warning"))
    server.handle_exit = lambda *_: (sim.hw_disconnect() if sim.hw is not None else None,
                                     sim.flush_memories(), os._exit(0))
    server.run()


if __name__ == "__main__":
    main()
