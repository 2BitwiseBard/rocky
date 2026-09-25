"""cockpit_brains (D052) — the cockpit's brains, out of cockpit.py.

The owner asked for "multiple options for AI models — one for vision, or
one just multimodal, a small or big one" and for commands that are
followed reliably. This module holds all of it; sim/cockpit.py keeps thin
wrappers (chat / tool / tool_look / llm_models / the /api/chat, /api/brain,
/api/models, /api/look, /api/voice routes).

ROLES (state keys in brackets, persisted to ~/.config/rocky/cockpit.json):
  brain      [model]            text + tools   default tool-model -> qwen3.6-35b-a3b
  vision     [vision_model]     describes the eye   vision-model -> lfm2.5-vl
  multimodal [multimodal_model] ONE model sees the eye frame AND calls tools
                                (brain mode 'multimodal'; off by default)
                                                     gemma-4-26b-a4b
  claude     [claude_model]     brain mode 'claude' (needs ANTHROPIC_API_KEY)
  stt        [stt_model]        whisper-server :8082
Why that default pair: on this 16 GB card only qwen3.6-35b-a3b (10.4 G at
-ncmoe 26) and lfm2.5-vl (4.6 G) co-reside in llama-swap's `resident`
group; any other brain + vision pair pays a model swap on EVERY look.

MODES: talk (regex, harness/intent.py — no model) | local (brain + look
tool) | multimodal (the eye JPEG rides in the user turn) | claude.

FIND (find_object, every mode incl. talk: "find the ball"): look -> the
vision model BOXES the named object (0-1000 coords, LFM2.5-VL's native
grounding) -> bearing + distance from projecting the box's bottom-centre
onto the floor through the eye's measured pose (pixel_to_floor) -> a
0.1-0.4 m goto toward it (tool_goto: every guard) or, unseen / unsure
(< 0.4), a 30 deg scan turn (turn(): the turn_in_place gait, timed) ->
look again; found within 0.25 m | not found | stopped by a goto veto.
Gemma writes boxes y-first ([ymin, xmin, ymax, xmax]) whatever it is
asked: box_order() reads them that way.
Asking the model to TYPE bearing/distance instead ('estimate') was
measured useless on lfm2.5-vl (sim/vision_bench.py): it echoes the
prompt's numbers.

MEMORY (sim/scene_memory.py, sim.memory): look and find_object record what
they saw (find: the box's floor geometry -> a map position, sighting_to_map;
look: objects the description places with a direction AND a distance, at
confidence 0.2; a find farther than FIND_TRUSTED_M at <= 0.3); tools
remember / where_is / recall / go_back_to / forget (go_back_to = ordinary
gotos, every guard; it refuses the vague and anything seen before the last
reset, and flags a stale one; forget wipes only on 'all', and a spoken wipe
needs the wake word). Each chat turn to a model starts
with the cockpit's fresh 'Situation: ...' line (sim.situation_now) in the
USER turn — not the system prompt, which would re-prefill the tool list
every turn on llama.cpp.

FALLBACK: brain qwen3.6-35b-a3b -> qwen3.8-27b-iq4 -> talk; vision
lfm2.5-vl -> gemma-4-26b-a4b; multimodal qwen3.5-9b -> gemma -> the text brain chain
(images stripped). A chain advances on timeout / HTTP error / empty answer
and the reply SAYS which model answered and why the earlier ones did not.
Quarantined models (gpt-oss-20b, glm-flash-reap: GPU faults) are never
selected, whatever the UI sends.

RELIABILITY (the D052 findings this fixes):
  * a tool that raises is a RESULT {"ok": false, "error": ...}, never a 500;
  * every tool_call gets a tool result, even malformed/failing ones (an
    orphaned tool_call 400'd every later turn); NaN arguments are refused;
  * per-mode asyncio.Lock: a typed and a spoken message no longer interleave
    one transcript;
  * the transcript is system + the last 12 turns; older turns keep the user
    line and the final sentence only (no tool traffic, no images);
  * at most 6 tool rounds, then an explicit "gave up";
  * every model call: SDK timeout 90 s, max_retries 0, and asyncio.wait_for
    (the SDK default is 600 s — a model swap froze the chat panel);
  * thinking off (chat_template_kwargs enable_thinking=false) for the brain
    and for look: qwen3.8-27b defaults to xhigh reasoning and spent the whole
    160-token look budget thinking -> an empty description (now ok=false and
    the chain moves on);
  * vision detection from llama-swap's own metadata (name/description say
    VISION (mmproj) / Multimodal / Text-only), not from the id: the old
    'gemma' substring rule offered gemma-4-31b (text-only: an image 500s)
    and missed qwen3.8-27b.

VOICE: whisper verbose_json, per-segment no_speech_prob / avg_logprob
filter + a hallucination list (harness.intent.clean_transcript); a spoken
line may not MOVE the robot without the wake word ("pebble, ...") unless
the operator confirms it (trusted=true, i.e. pressed Enter on it).
"""
from __future__ import annotations

import asyncio
import base64
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (ROOT, os.path.join(ROOT, "gait"), HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from harness.intent import (plan as intent_plan, execute as intent_execute,        # noqa: E402
                            has_wake_word, moves, clean_transcript, MOTION_TOOLS)
from harness.local_brain import (build_tools, build_system, parse_args_json,          # noqa: E402
                                 MAX_HOPS, MAX_TURNS, CALL_TIMEOUT_S)
from harness.backend import SIGNED                                                    # noqa: E402
from scene_memory import (objects_from_description, parse_note, fmt_age,               # noqa: E402
                          direction_words, forget_scope, GO_MIN_CONF, WORLD_MAX_M, SPAWN_NAME)

DEFAULT_BASE = "http://127.0.0.1:8080/v1"
WHISPER_URL = os.environ.get("ROCKY_WHISPER_URL", "http://127.0.0.1:8082")
WHISPER_PROMPT = ("Pebble, walk forward thirty centimeters. Pebble, stop. Wave. Bow. Sit. Shake. Turn left. Turn right. What do you see? Say hello.")   # vocabulary hint for the small model
MIN_VOICE_BYTES = 1200      # a webm/opus container with no audio is ~200-900 bytes
MIN_VOICE_S = 0.4           # shorter than a word: the hold was lost, not the speech
CONF_PATH = os.environ.get("ROCKY_COCKPIT_CONF", os.path.expanduser("~/.config/rocky/cockpit.json"))
MODES = ("talk", "local", "multimodal", "claude")
QUARANTINED = frozenset({"gpt-oss-20b", "glm-flash-reap"})   # GPU faults (workstation CLAUDE.md)
SKIP_PREFIXES = ("embedding", "reranker", "lab")              # not chat models
ROLE_KEYS = {"brain": "model", "vision": "vision_model", "multimodal": "multimodal_model",
             "claude": "claude_model", "stt": "stt_model"}
ROLE_DEFAULTS = {"model": "tool-model", "vision_model": "vision-model",
                 "multimodal_model": "qwen3.5-9b", "claude_model": "claude-sonnet-5",
                 "stt_model": "whisper"}
FALLBACK = {"brain": ["qwen3.6-35b-a3b", "qwen3.8-27b-iq4"],
            "vision": ["lfm2.5-vl", "gemma-4-26b-a4b"],
            "multimodal": ["qwen3.5-9b", "gemma-4-26b-a4b"]}   # qwen3.5-9b: measured 2026-09-24, 7.2 GB, 58 t/s, tools + vision
GOTO_MAX_M = 3.0          # a goto target farther than this from the robot is an argument error
LOOK_MAX_TOKENS = 160
REPLY_MAX_TOKENS = 512
CATALOG_TTL_S = 20.0      # llama-swap's model list (loaded/cold changes as models swap)
VOICE_RECENT_S = 60.0     # a chat text equal to the last transcript within this is voice
MAX_TEXT = 2000
RESULT_CHARS = 4000       # a tool result is cut to this in the transcript
SITUATION_CHARS = 500     # the situation line rides in every chat turn (composed to < 400 chars; this is the
#                           hard cap, cut at a word — the least useful clauses come last)

_VISION_RE = re.compile(r"vision|multimodal|mmproj|\bvlm?\b", re.I)
_TEXT_ONLY_RE = re.compile(r"text[\s-]*only", re.I)
_THINK_RE = re.compile(r"<think>.*?(</think>|$)", re.S)

VISION_PROMPT = ("You are the eye of a small five-legged robot walking on a floor. Describe "
                 "what is in front of it in two short sentences: obstacles or objects, roughly "
                 "how far (near = within a few body lengths), and where the open floor is. "
                 "If the view is mostly floor, say so.")
LOOK_NOTE = ("You also have `look`: the robot's eye camera described by a vision model. Use it "
             "when asked what you see, before walking toward something, or after a goto came "
             "back stuck or blocked. To go to something the operator names ('find the ball'), "
             "call find_object once: it looks, turns and walks by itself.")
MULTIMODAL_NOTE = ("The operator's message may carry the robot's CURRENT eye-camera image: answer "
                   "'what do you see' from it directly; call look only for a fresh view after "
                   "the robot has moved.")
COMPOSE_NOTE = ("To create a new gesture from a description, call compose_gesture with keyframes; "
                "it is checked against the real servo limits and previewed once. Nothing is kept "
                "until the operator says save (then call save_gesture). If the check fails, fix "
                "the FAIL lines and compose again.")

COMPOSE_DOC = (
    "Create a NEW gesture from keyframes (the owner describes it; you write the frames). It is "
    "checked against the robot's real limits (joint range, servo speed, balance, self-contact); "
    "when feasible it is previewed once on the robot and kept as a draft until save_gesture. "
    "Frame fields (all optional except t; missing = standing): t seconds from start (first frame "
    "{\"t\": 0} = standing); body [dx, dy, dz] mm body shift, feet planted (z < 0 crouches; "
    "+-30 mm is a lot); yaw deg body twist (+-15); dz [5] mm extra crouch per leg; arm "
    "{\"leg\": [yaw, hip, knee] deg} lifts that leg as an arm (yaw -40..40, hip -70..90 (up), "
    "knee -150..-20); claw [5] 0..1 open; say a chord word cue; ease smooth|linear|hold. Legs: "
    "0 = left (+y), then counter-clockwise: 1 rear-left, 2 rear-right, 3 front-right, 4 "
    "front-left. Keep >= 4 feet down (raise ONE leg; shift the body away from it first, "
    "body [0, -15, 0] for leg 0), give each move >= 0.6 s. Example wave with leg 0: "
    "[{\"t\":0},{\"t\":1.0,\"body\":[0,-15,0]},{\"t\":1.8,\"body\":[0,-15,0],\"arm\":{\"0\":[0,70,-60]}},"
    "{\"t\":2.6,\"body\":[0,-15,0],\"arm\":{\"0\":[25,70,-60]}},{\"t\":3.4,\"body\":[0,-15,0],"
    "\"arm\":{\"0\":[-25,70,-60]}},{\"t\":4.2,\"body\":[0,-15,0],\"arm\":{\"0\":[0,70,-60]}},"
    "{\"t\":5.0,\"body\":[0,-15,0]},{\"t\":5.8}]")
_FRAME_SCHEMA = {"type": "object", "properties": {
    "t": {"type": "number"},
    "body": {"type": "array", "items": {"type": "number"}},
    "yaw": {"type": "number"},
    "dz": {"type": "array", "items": {"type": "number"}},
    "arm": {"type": "object"},
    "claw": {"type": "array", "items": {"type": "number"}},
    "say": {"type": "string"},
    "ease": {"type": "string", "enum": ["smooth", "linear", "hold"]}},
    "required": ["t"]}
EXTRA_TOOLS = [
    {"type": "function", "function": {
        "name": "compose_gesture", "description": COMPOSE_DOC,
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "short snake_case name"},
            "description": {"type": "string", "description": "what the gesture should look like"},
            "keyframes": {"type": "array", "items": _FRAME_SCHEMA},
            "loop": {"type": "boolean"}},
            "required": ["name", "keyframes"]}}},
    {"type": "function", "function": {
        "name": "check_gesture",
        "description": "Check a keyframe gesture spec {name, keyframes, loop?, end?} against the "
                       "robot's limits WITHOUT moving; returns the report lines.",
        "parameters": {"type": "object", "properties": {"spec": {"type": "object"}},
                       "required": ["spec"]}}},
    {"type": "function", "function": {
        "name": "save_gesture",
        "description": "Save the last composed (previewed) gesture so it can be played by name. "
                       "Only when the operator asked to keep it.",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "optional new name"},
            "overwrite": {"type": "boolean"}}}}},
]

# ------------------------------------------------------------ find_object (vision-driven)
# The eye (world_builder.EYE_CAM): 320x240, fovy 70 (-> ~86 deg wide), pitched 15 deg
# down, 0.10 m ahead of the torso centre. MEASURED in the flat world standing at the
# D052 gait height: the camera is 0.2126 m above the floor; the bottom edge of the
# image meets the floor ~0.18 m ahead of the camera, the image centre ~0.79 m.
EYE_W_PX, EYE_H_PX = 320, 240
EYE_FOVY_DEG = 70.0
EYE_PITCH_DEG = 15.0
EYE_HEIGHT_M = 0.2126
EYE_FWD_M = 0.10
EYE_HFOV_DEG = round(math.degrees(2 * math.atan(math.tan(math.radians(EYE_FOVY_DEG / 2)) * EYE_W_PX / EYE_H_PX)), 1)
FIND_MAX_STEPS = 6          # looks per find_object call (each look may be followed by one move)
FIND_MAX_STEPS_CAP = 16
FIND_NEAR_M = 0.25          # the object's near edge is this close to the camera (or closer): found
FIND_STEP_MIN_M = 0.10      # a step toward it is distance - FIND_NEAR_M, clamped to [min, max]
FIND_STEP_MAX_M = 0.40      # (min 0.10, not 0.25: a 0.25 m step from 0.30 m away would ram it)
FIND_STEP_BLIND_M = 0.25    # seen, confident, but no distance (a box above the horizon line)
FIND_MIN_CONF = 0.40        # below this, never walk toward it: turn and look again
FIND_TRUSTED_M = 2.0        # a sighting farther than this goes to memory as VAGUE (<= FIND_FAR_CONF):
FIND_FAR_CONF = 0.3         # near the horizon a few pixels are metres (v 0.32 -> 14 m, 0.36 -> 3 m);
#                             the vision bench measured the box geometry only out to 1.4 m
FIND_DEFAULT_CONF = 0.50    # "seen" with no confidence field at all
FIND_SCAN_DEG = 30.0        # a scan turn (counter-clockwise); at most one full circle
BEARING_LIMIT_DEG = 60.0
FIND_MAX_TOKENS = 120
FIND_METHODS = ("bbox", "estimate")
# turn(): the gaited turn runs at pg2.TURN_WZ fitted into the gait budget. MEASURED in
# the cockpit (flat world, D052 gait, 2026-09-24): turn_in_place (4.8 s) turns ~47 deg;
# with these constants turn(30) turned 31.3-31.7 deg, turn(-45) -45.2, turn(90) 86.1.
TURN_RATE_DPS = 14.2
TURN_EXTRA_S = 1.4
TURN_MIN_DEG, TURN_MAX_DEG = 5.0, 180.0

# "bbox" (default): the model draws a box (0-1000 image coordinates, what LFM2.5-VL
# grounds in natively) and the bearing and distance come from projecting the box's
# bottom-centre onto the floor through the camera pose above — geometry, not a guess.
FIND_PROMPT = (
    'Is there a {name} in this image? If yes, reply with ONLY this JSON: {{"seen": true, '
    '"bbox": [x1, y1, x2, y2], "confidence": 0 to 1, "what": "a few words"}} where bbox is the '
    "{name}'s bounding box in 0-1000 image coordinates. If there is no {name}, reply "
    '{{"seen": false, "what": "what you see instead"}}.')
# "estimate": the model itself estimates bearing and distance (the first design; the
# vision bench keeps it for comparison — small VLMs tend to echo the numbers in the prompt)
FIND_ESTIMATE_PROMPT = (
    "You are the eye of a small robot on a floor. This is its forward camera: about "
    f"{EYE_HFOV_DEG:.0f} degrees wide, looking slightly down from 0.2 m above the floor. The "
    "bottom edge of the image is about 0.2 m in front of the camera, the middle of the image "
    "about 0.8 m. Is there a {name} in the image? Reply with ONLY one JSON object, no other "
    'text: {{"seen": true or false, "bearing_deg": left-right position, -43 = left edge of the '
    'image, 0 = centre, 43 = right edge, "distance_m": distance from the camera in meters, '
    '"confidence": 0 to 1, "what": "a few words"}}. If there is no {name}, reply '
    '{{"seen": false, "confidence": 1, "what": "what you see instead"}}.')
FIND_DOC = (
    "Find an object by name with the eye and walk up to it (e.g. 'find the ball', 'go to the "
    "box'). It loops by itself: look (a vision model boxes the object; its bearing and distance "
    "come from the camera geometry), then walk 0.1-0.4 m toward it (an ordinary goto: every "
    "guard applies), or turn 30 deg to scan when it is not seen or the sighting is unsure; it "
    "ends found (within ~0.25 m) | not found | stopped (a goto came back cliff / blocked / "
    "stuck: report it, do not retry blindly). Takes up to ~1-2 minutes.")
EXTRA_TOOLS.append(
    {"type": "function", "function": {
        "name": "find_object", "description": FIND_DOC,
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "what to find, in plain words: 'ball', 'box'"},
            "max_steps": {"type": "integer", "description": f"looks before giving up (default "
                                                            f"{FIND_MAX_STEPS}, max {FIND_MAX_STEPS_CAP})"}},
            "required": ["name"]}}})

# ------------------------------------------------------------ scene memory tools
# (sim/scene_memory.py; the cockpit owns one SceneMemory per world — sim.memory)
GO_BACK_STANDOFF_M = FIND_NEAR_M + EYE_FWD_M   # 0.35 m torso-to-near-edge: where find_object stops
GO_BACK_LEG_M = 1.2         # one goto leg (goto's 40 s cap is ~1.8 m at 45 mm/s)
GO_BACK_MAX_LEGS = 4
GO_BACK_DEFAULT_SIZE_M = 0.10
REMEMBER_DOC = ("Remember a fact the operator states, in their words ('the charger is by the door'). "
                "'the X is here' / 'this spot is X' / 'call this spot X' pins X at the robot's current "
                "position; 'the X is at (1.0, 0.2)' pins map meters (the floor is +-6 m); anything else "
                "is kept as a note that recall finds by its words. Only for what the operator tells you "
                "— sightings are remembered automatically.")
WHERE_IS_DOC = ("Where is a named object, from memory (no looking, no walking): its map position "
                "(meters), how long ago it was seen, confidence and source (find = the eye's box "
                "geometry; user = the operator pinned it; look = a vague description). stale=true: "
                "seen over 10 min ago or before the last reset, so it may have moved — find_object "
                "re-checks. known=false when it was never located — then call find_object.")
RECALL_DOC = ("Search the robot's memory of this world — what it saw, found, was told, and which "
              "guards fired — for a query ('ball', 'charger', 'cliff'); returns the most relevant "
              "entries with their age. An empty query returns recent entries, the operator's notes "
              "and things near the robot first.")
GO_BACK_DOC = ("Walk back to a remembered object or place ('go back to the ball', 'go back to the "
               f"{SPAWN_NAME}' = where the robot spawned): ordinary gotos toward its remembered "
               "position (every guard applies), stopping ~0.4 m short of it (a place the operator "
               "pinned with 'X is here': onto it). Refused when it is not remembered, only vaguely, or "
               "only from before the last reset — then call find_object. A stale sighting (> 10 min) "
               "is walked to but flagged. It does NOT re-check with the eye: call look or "
               "find_object afterwards if the object may have moved.")
FORGET_DOC = ("Forget one named object (and every memory that mentions it), or everything with name "
              "'all'. A pronoun ('that', 'it') forgets nothing: name the thing. Only when the "
              "operator asks.")
EXTRA_TOOLS += [
    {"type": "function", "function": {
        "name": "remember", "description": REMEMBER_DOC,
        "parameters": {"type": "object", "properties": {
            "note": {"type": "string", "description": "the fact, in the operator's words"}},
            "required": ["note"]}}},
    {"type": "function", "function": {
        "name": "where_is", "description": WHERE_IS_DOC,
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "the object, in plain words: 'ball'"}},
            "required": ["name"]}}},
    {"type": "function", "function": {
        "name": "recall", "description": RECALL_DOC,
        "parameters": {"type": "object", "properties": {
            "query": {"type": "string"},
            "k": {"type": "integer", "description": "how many entries (default 5, max 20)"}}}}},
    {"type": "function", "function": {
        "name": "go_back_to", "description": GO_BACK_DOC,
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "the remembered object: 'ball'"}},
            "required": ["name"]}}},
    {"type": "function", "function": {
        "name": "forget", "description": FORGET_DOC,
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "the object, or 'all'"}},
            "required": ["name"]}}},
]
MEMORY_NOTE = ("Each operator message may start with 'Situation: ...' — the robot's own awareness "
               "(pose, guards, what the lidar sees nearby, remembered objects, the last look; servo "
               "heat and the servo bus only when they matter), written by the cockpit, not by the "
               "operator: answer 'what's around you' or 'what do you know' from it. The lidar sees "
               "only things taller than ~0.18 m (a ball or a low step is invisible to it). Memory: "
               "where_is / recall answer from what was seen or told before (a 'stale' entry may have "
               "moved); go_back_to walks to a remembered object, and to "
               f"'{SPAWN_NAME}' (the spawn point); remember keeps what the operator tells you; "
               "find_object and look are remembered automatically.")
MEMORY_TOOLS = ("remember", "where_is", "recall", "go_back_to", "forget")

TOOL_NAMES = ("say", "gesture", "goto", "stop", "scan_summary", "status", "look",
              "list_gestures", "compose_gesture", "check_gesture", "save_gesture",
              "find_object", "turn") + MEMORY_TOOLS   # turn: internal (find_object's scan; replays), not offered to models
NOTED = ("goto", "gesture", "say", "stop", "compose_gesture", "turn")     # recordings replay these
# find_object / go_back_to are not NOTED: the gotos and turns they make are, so a replay
# repeats the motion without asking a vision model (or the memory) again
GATED = tuple(MOTION_TOOLS) + ("find_object", "turn", "go_back_to")   # a spoken line needs the wake word

# the static defaults (the cockpit builds both per request with the live lists)
TOOLS = build_tools(look=True, extra=EXTRA_TOOLS)
UNITS_NOTE = ("\nUnits: goto and every distance argument are METRES. 'thirty centimeters' = 0.3, "
              "'half a meter' = 0.5, '2 meters' = 2.0 — never pass 30 for 30 cm (a 9B model did).")
SYSTEM = build_system(extra=LOOK_NOTE + UNITS_NOTE)


class ModelFailure(Exception):
    """A model call that did not produce an answer (timeout, HTTP error, empty)."""


# ------------------------------------------------------------------ pure helpers
def local_ai_key():
    for k in ("ROCKY_LLM_API_KEY", "LOCAL_AI_KEY"):
        if os.environ.get(k):
            return os.environ[k]
    conf = os.path.expanduser("~/.config/environment.d/local-ai.conf")
    if os.path.exists(conf):
        for line in open(conf):
            if line.startswith("LOCAL_AI_KEY="):
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    return "none"


def classify_models(entries):
    """llama-swap /v1/models `data` -> [{id, name, vision, loaded, selector,
    targets, strategy, quarantined}] for chat models. vision iff the id, name
    or description says vision / multimodal / mmproj / VL and not text-only;
    a selector inherits vision and loaded from its targets."""
    out = []
    for e in entries or []:
        mid = str(e.get("id", ""))
        if not mid or mid.startswith(SKIP_PREFIXES):
            continue
        meta = (e.get("meta") or {}).get("llamaswap") or {}
        text = " ".join([mid, str(e.get("name") or ""), str(e.get("description") or "")])
        out.append(dict(
            id=mid, name=str(e.get("name") or mid),
            vision=bool(_VISION_RE.search(text)) and not _TEXT_ONLY_RE.search(text),
            loaded=(e.get("status") or {}).get("value") == "loaded",
            selector=meta.get("type") == "selector",
            targets=list(meta.get("targets") or []), strategy=meta.get("strategy"),
            quarantined=mid in QUARANTINED or "QUARANTIN" in text.upper()))
    by = {m["id"]: m for m in out}
    for m in out:
        tg = [by[t] for t in m["targets"] if t in by]
        if m["selector"] and tg:
            m["vision"] = any(t["vision"] for t in tg)
            m["loaded"] = any(t["loaded"] for t in tg)
            m["quarantined"] = all(t["quarantined"] for t in tg)
    return out


def validate_goto(x, y):
    """(tx, ty, None) or (None, None, error): numbers in meters, finite.
    goto(x='here') used to raise in the sim thread and 500 /api/chat;
    goto(NaN, 0) got through json.loads and poisoned cmd_v."""
    if isinstance(x, bool) or isinstance(y, bool):
        return None, None, "goto needs x and y in meters (numbers), got a boolean"
    try:
        tx, ty = float(x), float(y)
    except (TypeError, ValueError):
        return None, None, f"goto needs x and y in meters (numbers), got x={x!r}, y={y!r}"
    if not (math.isfinite(tx) and math.isfinite(ty)):
        return None, None, f"goto needs finite numbers, got x={x!r}, y={y!r}"
    return tx, ty, None


def goto_range_error(tx, ty, pose, max_m=GOTO_MAX_M):
    d = math.hypot(tx - float(pose["x"]), ty - float(pose["y"]))
    if d > max_m:
        return (f"target is {d:.2f} m away; goto is capped at {max_m:.0f} m (and ~1.5 m is what "
                f"fits in its 40 s) — pick a closer waypoint")
    return None


def _short_err(e):
    code = getattr(e, "status_code", None)
    msg = str(getattr(e, "message", "") or e).replace("\n", " ")
    return (f"HTTP {code}: " if code else f"{type(e).__name__}: ") + msg[:200]


def _summ(res):
    if not isinstance(res, dict):
        return str(res)[:40]
    if res.get("stopped"):
        return str(res["stopped"])
    if res.get("ok") is False:
        return "refused: " + str(res.get("error", ""))[:60]
    return "ok"


def summarize_trace(trace):
    return "; ".join(f"{t['tool']} -> {_summ(t['result'])}" for t in trace[-4:]) or "nothing ran"


def _strip_images(msgs):
    """Replace image parts with a note (for a text-only fallback model)."""
    for m in msgs:
        c = m.get("content")
        if isinstance(c, list):
            txt = " ".join(p.get("text", "") for p in c if p.get("type") == "text")
            m["content"] = txt + " [the eye image was dropped: this model is text-only; call look]"
    return msgs


_OBJ_RE = re.compile(r"\{[^{}]*\}", re.S)
_FENCE_RE = re.compile(r"```(?:json)?", re.I)


def _num(v):
    """A finite float from a number or a numeric string, else None (bools refused)."""
    if isinstance(v, bool) or v is None:
        return None
    if isinstance(v, str):
        m = re.search(r"-?\d+(?:\.\d+)?", v)
        if not m:
            return None
        v = m.group(0)
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def pixel_to_floor(u, v, height_m=EYE_HEIGHT_M, pitch_deg=EYE_PITCH_DEG):
    """An eye-image point (u right, v down, both 0..1 of the frame) -> where its
    ray meets the floor: (distance_m from the camera, bearing_deg in the image
    convention, negative = LEFT), or None when the ray does not come down to
    the floor (at or above the horizon). A pinhole with square pixels; flat
    floor at the robot's standing height assumed."""
    f = (EYE_H_PX / 2) / math.tan(math.radians(EYE_FOVY_DEG / 2))
    xc = (float(u) - 0.5) * EYE_W_PX / f                  # right
    yc = -(float(v) - 0.5) * EYE_H_PX / f                 # up
    p = math.radians(pitch_deg)
    # camera axes in the body frame: forward (cos p, 0, -sin p), up (sin p, 0, cos p),
    # right (0, -1, 0) — world_builder's xyaxes="0 -1 0 0.26 0 0.97"
    rx, ry, rz = math.cos(p) + yc * math.sin(p), -xc, -math.sin(p) + yc * math.cos(p)
    if rz >= -1e-6:
        return None
    t = float(height_m) / -rz
    gx, gy = t * rx, t * ry
    return math.hypot(gx, gy), -math.degrees(math.atan2(gy, gx))


def box_order(model):
    """How a model family writes a box. Gemma (like Gemini / PaliGemma) answers
    [ymin, xmin, ymax, xmax] whatever the prompt asks — MEASURED in the vision
    bench 2026-09-24: read as x-first, its bearings were off by a median 21 deg;
    read y-first, within ~2 deg. LFM2.5-VL (and Qwen-VL) write [x1, y1, x2, y2]."""
    return "yx" if "gemma" in str(model or "").lower() else "xy"


def _bbox(v, order="xy"):
    """[x1, y1, x2, y2] (or [y1, x1, y2, x2] for order 'yx') -> x-first
    fractions of the frame (0..1), ordered; accepts 0..1 fractions or 0-1000
    coordinates. None when it is not a usable box."""
    if not isinstance(v, (list, tuple)) or len(v) != 4:
        return None
    n = [_num(x) for x in v]
    if any(x is None for x in n):
        return None
    scale = 1.0 if max(n) <= 1.0 else 1000.0
    if max(n) > 1000.0 or min(n) < 0.0:
        return None
    x1, y1, x2, y2 = (x / scale for x in n)
    if order == "yx":
        x1, y1, x2, y2 = y1, x1, y2, x2
    x1, x2 = sorted((x1, x2))
    y1, y2 = sorted((y1, y2))
    if x2 - x1 <= 0 or y2 - y1 <= 0:
        return None
    return [round(x1, 4), round(y1, 4), round(x2, 4), round(y2, 4)]


_LIST4_RE = re.compile(r"\[\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*,\s*(-?[\d.]+)\s*\]")


def parse_detection(text, order="xy"):
    """A vision model's answer to FIND_PROMPT / FIND_ESTIMATE_PROMPT ->
    {seen, bearing_deg, distance_m, confidence, what, parsed, method, bbox}.
    Robust to what small VLMs do: prose or a code fence around the JSON,
    single quotes, Python True/False, a trailing comma, a percent confidence,
    numbers as strings, a bare [x1, y1, x2, y2] list, bbox_2d for bbox
    (order 'yx': the model writes [ymin, xmin, ymax, xmax], see box_order). A box
    wins over any bearing/distance the model typed: they are computed from the
    box's bottom-centre (pixel_to_floor). Anything that does not parse is 'not
    seen' (parsed False) — never a guess. A sighting with no bearing is not
    actionable, so it is 'not seen' too. bearing_deg is clamped to
    +-BEARING_LIMIT_DEG (image convention: negative = LEFT)."""
    out = {"seen": False, "bearing_deg": None, "distance_m": None, "confidence": 0.0,
           "what": "", "parsed": False, "method": "estimate", "bbox": None}
    raw = _FENCE_RE.sub("", _THINK_RE.sub("", str(text or "")))
    m = _OBJ_RE.search(raw)
    d = None
    if m:
        blob = m.group(0)
        for cand in (blob, re.sub(r",\s*}", "}", re.sub(r"\bFalse\b", "false", re.sub(
                r"\bTrue\b", "true", re.sub(r"\bNone\b", "null", blob.replace("'", '"')))))):
            try:
                d = json.loads(cand, parse_constant=_no_const)
                break
            except ValueError:
                continue
    if not isinstance(d, dict):
        lm = _LIST4_RE.search(raw)
        if lm and _bbox(list(lm.groups())) is not None:
            d = {"seen": True, "bbox": list(lm.groups())}      # a bare box: the model's grounding answer
        else:
            out["what"] = raw.strip()[:80]
            return out
    out["parsed"] = True
    box = _bbox(d.get("bbox") if d.get("bbox") is not None else d.get("bbox_2d"), order)
    seen = d.get("seen", box is not None)
    if isinstance(seen, str):
        seen = seen.strip().lower() in ("true", "yes", "1", "y")
    seen = bool(seen) if isinstance(seen, (bool, int, float)) else False
    bearing, dist, conf = _num(d.get("bearing_deg")), _num(d.get("distance_m")), _num(d.get("confidence"))
    out["what"] = str(d.get("what") or d.get("label") or "")[:80]
    if box is not None:
        out["method"], out["bbox"] = "bbox", box
        u, v = (box[0] + box[2]) / 2, box[3]
        fl = pixel_to_floor(u, v)
        if fl is None:                               # its foot is at/above the horizon: bearing only
            bearing, dist = -math.degrees(math.atan((u - 0.5) * EYE_W_PX / (
                (EYE_H_PX / 2) / math.tan(math.radians(EYE_FOVY_DEG / 2))))), None
        else:
            dist, bearing = fl
    if bearing is not None:
        bearing = max(-BEARING_LIMIT_DEG, min(BEARING_LIMIT_DEG, bearing))
    if dist is not None and dist <= 0:
        dist = None
    if conf is None:
        conf = FIND_DEFAULT_CONF if seen else 0.0
    elif 1.0 < conf <= 100.0:                        # "confidence": 85 -> 0.85
        conf = conf / 100.0
    conf = max(0.0, min(1.0, conf))
    if seen and bearing is None:
        seen = False
        out["what"] = (out["what"] + " (no bearing given)").strip()
    out.update(seen=seen, bearing_deg=None if bearing is None else round(bearing, 1),
               distance_m=None if dist is None else round(dist, 3), confidence=round(conf, 2))
    return out


def bearing_to_map(pose, bearing_deg, dist_m):
    """A point dist_m along an eye bearing, in the map frame. bearing_deg is the
    image convention (negative = LEFT); the body frame has +y to the LEFT, so
    the body angle is -bearing_deg, and the map angle is yaw - bearing_deg.
    Measured from the torso centre (pose), which is where goto steers."""
    th = math.radians(float(pose["yaw_deg"]) - float(bearing_deg))
    return (float(pose["x"]) + float(dist_m) * math.cos(th),
            float(pose["y"]) + float(dist_m) * math.sin(th))


def find_policy(det, near_m=FIND_NEAR_M, min_conf=FIND_MIN_CONF):
    """One detection -> ("stop", 0.0) | ("goto", step_m) | ("scan", 0.0).
    Never walks on a low-confidence (or unseen) detection."""
    if not det.get("seen") or float(det.get("confidence") or 0.0) < min_conf:
        return "scan", 0.0
    d = det.get("distance_m")
    if d is None:
        return "goto", FIND_STEP_BLIND_M
    if d <= near_m:
        return "stop", 0.0
    return "goto", round(min(FIND_STEP_MAX_M, max(FIND_STEP_MIN_M, d - near_m)), 3)


def _wrap_deg(a):
    return (float(a) + 180.0) % 360.0 - 180.0


def floor_to_map(pose, bearing_deg, dist_from_camera_m):
    """A floor point seen by the eye (pixel_to_floor's bearing + distance, both
    from the CAMERA, which sits EYE_FWD_M ahead of the torso centre) -> map x, y.
    bearing_to_map measures from the torso (it aims gotos); this is the
    geometric position the memory keeps."""
    yaw = math.radians(float(pose["yaw_deg"]))
    cx = float(pose["x"]) + EYE_FWD_M * math.cos(yaw)
    cy = float(pose["y"]) + EYE_FWD_M * math.sin(yaw)
    th = yaw - math.radians(float(bearing_deg))
    return cx + float(dist_from_camera_m) * math.cos(th), cy + float(dist_from_camera_m) * math.sin(th)


def sighting_to_map(pose, det):
    """A find_object detection -> {x, y, size_m?} in the map frame, or None
    (unseen / no distance). With a box: the box's bottom corners projected to
    the floor give its floor width; the centre estimate is half that width
    beyond the near edge along the same bearing (a round or square object is
    about as deep as it is wide). Without a box (method 'estimate'): the
    model's own point, no size."""
    if not det or not det.get("seen") or det.get("distance_m") is None or det.get("bearing_deg") is None:
        return None
    dist, bearing, size = float(det["distance_m"]), float(det["bearing_deg"]), None
    box = det.get("bbox")
    if box:
        a, b = pixel_to_floor(box[0], box[3]), pixel_to_floor(box[2], box[3])
        if a is not None and b is not None:
            (da, ba), (db, bb) = a, b
            pa = (da * math.cos(math.radians(-ba)), da * math.sin(math.radians(-ba)))
            pb = (db * math.cos(math.radians(-bb)), db * math.sin(math.radians(-bb)))
            size = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            if 0.0 < size < 2.0:
                dist += size / 2.0
            else:
                size = None
    x, y = floor_to_map(pose, bearing, dist)
    out = {"x": round(x, 3), "y": round(y, 3)}
    if size is not None:
        out["size_m"] = round(size, 3)
    return out


class _Surface:
    """The harness backend contract (say/gesture/goto/...) routed through
    Brains.tool, so harness.intent.execute gets the same guards, argument
    checks and voice gate as the LLM brains."""

    def __init__(self, brains, voice=False):
        self._b, self._voice = brains, voice

    async def _t(self, tool, /, **args):          # positional-only: gesture's arg is `name`
        return await self._b.tool(tool, args, voice=self._voice)

    async def say(self, word):
        return await self._t("say", word=word)

    async def gesture(self, name, direction=None):
        return await self._t("gesture", name=name, **({"direction": direction} if direction else {}))

    async def goto(self, x, y):
        return await self._t("goto", x=x, y=y)

    async def stop(self):
        return await self._t("stop")

    async def scan_summary(self):
        return await self._t("scan_summary")

    async def status(self):
        return await self._t("status")

    async def look(self):
        return await self._t("look")

    async def list_gestures(self):
        return await self._t("list_gestures")

    async def find_object(self, name, max_steps=None):
        return await self._t("find_object", name=name,
                             **({"max_steps": max_steps} if max_steps is not None else {}))

    async def remember(self, note):
        return await self._t("remember", note=note)

    async def where_is(self, name):
        return await self._t("where_is", name=name)

    async def recall(self, query="", k=None):
        return await self._t("recall", query=query, **({"k": k} if k is not None else {}))

    async def go_back_to(self, name):
        return await self._t("go_back_to", name=name)

    async def forget(self, name):
        return await self._t("forget", name=name)


# ------------------------------------------------------------------ the brains
class Brains:
    """Roles, histories, locks and fallbacks for one cockpit sim.

    `sim` needs: gesture_names, lexicon, gestures {name: (fn, total)},
    frames['eye'] = (n, jpeg), log(str), events (deque), note_cmd(str),
    async call(fn), tool_<say|stop|scan_summary|status|goto|gesture>, and for
    previews start_gesture / gesture_busy / reload_library (Playground, D052).
    `complete` (tests) replaces the OpenAI call: fn(model, messages, tools,
    max_tokens) -> {"content": str, "tool_calls": [{"id", "name", "arguments"}]}."""

    def __init__(self, sim, base_url=None, api_key=None, conf_path=CONF_PATH, complete=None,
                 timeout_s=CALL_TIMEOUT_S, catalog=None):
        self.sim = sim
        self.base = (base_url or os.environ.get("ROCKY_LLM_BASE_URL") or DEFAULT_BASE).rstrip("/")
        self.key = api_key or local_ai_key()
        self.timeout_s = float(timeout_s)
        self.conf_path = conf_path
        self.state = dict(mode="talk", **ROLE_DEFAULTS)       # sim.brain IS this dict
        self.hist = {}                                         # mode -> [{user, reply, tools, t}]
        self._locks = {}
        self._complete_fn = complete
        self._client = None
        self._cat = (time.monotonic(), catalog, True) if catalog is not None else (0.0, None, False)
        self._probe = {}                                       # id -> vision bool from /props
        self._voice_last = (0.0, "")
        self.composed = None                                   # the last compose_gesture draft
        self.last_look = None                                  # {text, model, t, pose}: the awareness loop reads it
        self.last_find = None                                  # {name, found, detail, t, model}: ditto (find's looks)
        self._load_conf()

    @property
    def memory(self):
        """The cockpit's SceneMemory (sim.memory), or None (fake sims, tests)."""
        return getattr(self.sim, "memory", None)

    def _remember_obs(self, **obs):
        """Record one observation; never raises (memory is a convenience, not a guard)."""
        mem = self.memory
        if mem is None:
            return None
        try:
            return mem.remember(obs)
        except Exception as e:                                 # noqa: BLE001
            self._log(f"memory: not recorded ({type(e).__name__}: {e})")
            return None

    # ---------------------------------------------------------------- config
    def _load_conf(self):
        if not self.conf_path:
            return
        try:
            with open(self.conf_path) as f:
                saved = json.load(f).get("roles", {})
        except (OSError, ValueError, AttributeError):
            return
        for k, v in saved.items():
            if k in ROLE_DEFAULTS and isinstance(v, str) and v and v not in QUARANTINED:
                self.state[k] = v

    def _save_conf(self):
        if not self.conf_path:
            return
        try:
            os.makedirs(os.path.dirname(self.conf_path), exist_ok=True)
            with open(self.conf_path, "w") as f:
                json.dump({"roles": {k: self.state[k] for k in ROLE_DEFAULTS}}, f, indent=1)
        except OSError as e:
            self._log(f"brains: could not save {self.conf_path}: {e}")

    def roles(self):
        return {r: self.state[k] for r, k in ROLE_KEYS.items()}

    def _log(self, msg):
        try:
            self.sim.log(msg)
        except Exception:
            pass

    # --------------------------------------------------------------- catalog
    def catalog(self, refresh=False):
        """(models list or None when llama-swap is unreachable, reachable bool).
        Cached CATALOG_TTL_S; blocking — call it off the event loop."""
        t, cat, ok = self._cat
        if not refresh and cat is not None and time.monotonic() - t < CATALOG_TTL_S:
            return cat, ok
        try:
            import httpx
            r = httpx.get(f"{self.base}/models", headers={"Authorization": f"Bearer {self.key}"},
                          timeout=4.0)
            r.raise_for_status()
            cat, ok = classify_models(r.json().get("data", [])), True
        except Exception:
            cat, ok = (self._cat[1], False)                     # keep the last good list
        self._cat = (time.monotonic(), cat, ok)
        return cat, ok

    async def _acatalog(self):
        t, cat, _ = self._cat
        if cat is None or time.monotonic() - t >= CATALOG_TTL_S:
            await asyncio.to_thread(self.catalog, True)
        return {m["id"]: m for m in (self._cat[1] or [])} if self._cat[1] is not None else None

    def _probe_vision(self, entry):
        """Lazy truth from llama-server's /props (modalities.vision), only for a
        model that is ALREADY loaded — probing a cold one would load it."""
        mid = entry["id"]
        if mid in self._probe or entry["selector"] or not entry["loaded"]:
            return self._probe.get(mid)
        root = self.base[:-3] if self.base.endswith("/v1") else self.base
        try:
            import httpx
            r = httpx.get(f"{root}/upstream/{mid}/props",
                          headers={"Authorization": f"Bearer {self.key}"}, timeout=3.0)
            mod = r.json().get("modalities")
            self._probe[mid] = bool(mod.get("vision")) if isinstance(mod, dict) else None
        except Exception:
            self._probe[mid] = None
        return self._probe[mid]

    def chain(self, role, first=None, cat=None):
        """([model ids to try in order], [notes on what was skipped and why])."""
        key = ROLE_KEYS[role]
        cands = [first or self.state.get(key)] + FALLBACK.get(role, [])
        out, notes, seen = [], [], set()
        for c in cands:
            if not c or c in out or c in seen:
                continue
            if c in QUARANTINED:
                notes.append(f"{c} is quarantined (GPU faults) — never used")
                continue
            if cat is not None:
                e = cat.get(c)
                if e is None:
                    notes.append(f"{c} is not in llama-swap")
                    continue
                if e["quarantined"]:
                    notes.append(f"{c} is quarantined — skipped")
                    continue
                if role in ("vision", "multimodal"):
                    probed = self._probe.get(c)
                    if not e["vision"] or probed is False:
                        notes.append(f"{c} is text-only — skipped")
                        continue
                if e["selector"] and e["targets"]:
                    seen.add(e["targets"][0])                   # don't retry the same weights
            out.append(c)
        return out, notes

    def models_payload(self):
        """GET /api/models (blocking: run in a thread)."""
        cat, ok = self.catalog(refresh=True)
        cat = cat or []
        for e in cat:
            if e["loaded"] and e["vision"]:
                p = self._probe_vision(e)
                if p is False:
                    e["vision"] = False
        usable = [m for m in cat if not m["quarantined"]]
        cmap = {m["id"]: m for m in cat} if ok else None
        chains = {r: self.chain(r, cat=cmap)[0] for r in ("brain", "vision", "multimodal")}
        chains["brain"] = chains["brain"] + ["talk"]
        return {
            "models": [m["id"] for m in usable],                           # legacy UI: ids
            "vision_models": [m["id"] for m in usable if m["vision"]],     # legacy UI: ids
            "catalog": cat, "reachable": ok, "roles": self.roles(), "fallbacks": chains,
            "modes": list(MODES), "brain": self.state, "claude": self.claude_available(),
            "base_url": self.base, "stt": {"url": WHISPER_URL, "model": self.state["stt_model"]},
            "quarantined": sorted(QUARANTINED),
            "draft": None if self.composed is None else {        # the gesture studio can load it
                k: self.composed.get(k) for k in ("name", "spec", "ok", "report", "saved")}}

    def set_roles(self, body):
        """POST /api/brain (blocking: run in a thread). Keys: mode, and any role
        by name (brain|vision|multimodal|claude|stt) or state key (model,
        vision_model, ...), or roles={...}. Refuses quarantined models, text-only
        models for vision/multimodal, and unknown modes — saying why."""
        body = dict(body or {})
        items = dict(body.get("roles") or {}) if isinstance(body.get("roles"), dict) else {}
        items.update({k: v for k, v in body.items() if k != "roles"})
        errors, changed = [], False
        cat, ok = self.catalog()
        cmap = {m["id"]: m for m in cat} if (cat is not None and ok) else None
        if "mode" in items:
            if items["mode"] in MODES:
                self.state["mode"] = items["mode"]
            else:
                errors.append(f"unknown mode {items['mode']!r} (have {', '.join(MODES)})")
        for k, v in items.items():
            key = ROLE_KEYS.get(k, k if k in ROLE_DEFAULTS else None)
            if key is None or v is None:
                continue
            v = str(v).strip()
            err = self._role_error(key, v, cmap)
            if err:
                errors.append(err)
            elif self.state[key] != v:
                self.state[key] = v
                changed = True
        if changed:
            self._save_conf()
        return {"ok": not errors, "errors": errors, "brain": self.state, "roles": self.roles()}

    def _role_error(self, key, v, cmap):
        if not v:
            return f"{key}: empty model id"
        if v in QUARANTINED:
            return f"{v} is quarantined (GPU faults) — not selectable"
        if key == "claude_model":
            return None if v.startswith("claude") else f"{v} is not a Claude model id"
        if key == "stt_model":
            return None
        if cmap is not None:
            e = cmap.get(v)
            if e is None:
                return f"{v} is not in llama-swap's model list"
            if e["quarantined"]:
                return f"{v} is quarantined — not selectable"
            if key in ("vision_model", "multimodal_model") and not e["vision"]:
                return f"{v} is text-only (an image returns an error) — not a {key.split('_')[0]} model"
        return None

    def claude_available(self):
        try:
            import anthropic                                        # noqa: F401
        except ImportError:
            return False
        return bool(os.environ.get("ANTHROPIC_API_KEY"))

    def clear(self, mode=None):
        if mode in (None, "all"):
            self.hist.clear()
        else:
            self.hist.pop(mode, None)

    # ------------------------------------------------------------ model calls
    def _openai(self):
        if self._client is None:
            from openai import OpenAI
            self._client = OpenAI(base_url=self.base, api_key=self.key,
                                  timeout=self.timeout_s, max_retries=0)
        return self._client

    def _complete(self, model, messages, tools, max_tokens):
        if self._complete_fn is not None:
            return self._complete_fn(model, messages, tools, max_tokens)
        kw = dict(model=model, messages=messages, max_tokens=max_tokens,
                  extra_body={"chat_template_kwargs": {"enable_thinking": False}})
        if tools:
            kw["tools"] = tools
        r = self._openai().chat.completions.create(**kw)
        msg = r.choices[0].message
        return {"content": msg.content or "",
                "tool_calls": [{"id": tc.id, "name": tc.function.name,
                                "arguments": tc.function.arguments}
                               for tc in (msg.tool_calls or [])]}

    async def _acall(self, model, messages, tools, max_tokens):
        try:
            r = await asyncio.wait_for(
                asyncio.to_thread(self._complete, model, messages, tools, max_tokens),
                self.timeout_s + 5.0)
        except asyncio.TimeoutError:
            raise ModelFailure(f"timed out after {self.timeout_s:.0f} s") from None
        except ModelFailure:
            raise
        except Exception as e:
            raise ModelFailure(_short_err(e)) from None
        r = dict(r or {})
        r["content"] = _THINK_RE.sub("", r.get("content") or "").strip()
        r["tool_calls"] = list(r.get("tool_calls") or [])
        return r

    # ------------------------------------------------------------------ tools
    def tools_for(self, look=True):
        s = self.sim
        return build_tools(getattr(s, "gesture_names", None), getattr(s, "lexicon", None),
                           signed=SIGNED, look=look, extra=EXTRA_TOOLS)

    def system_for(self, multimodal=False):
        s = self.sim
        extra = (LOOK_NOTE + " " + COMPOSE_NOTE + UNITS_NOTE + (" " + MULTIMODAL_NOTE if multimodal else "")
                 + (" " + MEMORY_NOTE if self.memory is not None else ""))
        return build_system(getattr(s, "gesture_names", None), getattr(s, "lexicon", None), extra)

    def surface(self, voice=False):
        return _Surface(self, voice)

    async def tool(self, name, args=None, voice=False, look_model=None):
        """Run one tool for any brain or the MCP proxy. Never raises: a bad
        argument or a failing tool comes back as {"ok": False, "error": ...}."""
        if name not in TOOL_NAMES:
            return {"ok": False, "error": f"no such tool {name!r}", "hint": ", ".join(TOOL_NAMES)}
        if args is None:
            args = {}
        if not isinstance(args, dict):
            return {"ok": False, "error": f"arguments for {name} must be an object"}
        if voice and name in GATED:
            return {"ok": False, "error": "voice_unconfirmed",
                    "hint": "a spoken motion command needs the wake word ('pebble, ...') "
                            "or the operator's confirmation"}
        if voice and name == "forget" and forget_scope(args.get("name", ""))[0] == "all":
            return {"ok": False, "error": "voice_unconfirmed",   # a misheard line must not wipe the memory
                    "hint": "forgetting everything by voice needs the wake word ('pebble, forget "
                            "everything') or the typed chat"}
        if name in NOTED:
            try:
                self.sim.note_cmd(f"tool {name} {json.dumps(args)}")
            except Exception:
                pass
        try:
            if name == "look":
                return await self.look(args.get("model") or look_model)
            if name == "gesture":
                return await self._gesture(**args)
            if name == "list_gestures":
                return self.list_gestures(**args)
            if name == "compose_gesture":
                return await self.compose_gesture(**args)
            if name == "check_gesture":
                return await self.check_gesture(**args)
            if name == "save_gesture":
                return await self.save_gesture(**args)
            if name == "find_object":
                return await self.find_object(**args)
            if name == "turn":
                return await self.turn(**args)
            if name in MEMORY_TOOLS:
                return await getattr(self, "mem_" + name)(**args)
            return await getattr(self.sim, "tool_" + name)(**args)
        except TypeError as e:
            return {"ok": False, "error": f"bad arguments for {name}: {e}"}
        except Exception as e:
            self._log(f"tool {name} failed: {type(e).__name__}: {e}")
            return {"ok": False, "error": f"{name} failed: {type(e).__name__}: {e}"}

    def list_gestures(self):
        g = getattr(self.sim, "gestures", {}) or {}
        names = list(getattr(self.sim, "gesture_names", None) or sorted(g))
        kf = sorted(k for k, v in g.items() if hasattr(v[0], "spec"))
        return {"ok": True, "gestures": names, "keyframe": kf, "signed": list(SIGNED),
                "draft": None if self.composed is None else self.composed["name"]}

    async def _gesture(self, name, direction=None):
        if direction not in ("", None, "left", "right"):
            return {"ok": False, "error": f"direction must be left or right, got {direction!r}"}
        if direction and name not in SIGNED:
            return {"ok": False, "error": f"direction applies to {', '.join(SIGNED)} only"}
        if direction != "right":
            r = await self.sim.tool_gesture(name)
            if isinstance(r, dict) and direction == "left" and r.get("ok"):
                r["direction"] = "left"
            return r
        # the right-handed mirror: same gait, negated command (the gait turns both ways;
        # the registry only carries the left-handed canon)
        import pebble_gestures2 as pg2
        if name == "turn_in_place":
            total, cmd = pg2.TURN_TOTAL, (0.0, 0.0, -pg2.TURN_WZ)
        else:
            total, cmd = pg2.SIDE_TOTAL, (0.0, -pg2.SIDE_VY, 0.0)

        def fn(g, t, total=total, cmd=cmd):
            return pg2._gaited(g, t, total, cmd)
        fn.gaited = True                         # D052 V2: it walks — the sim2real locomotion hold applies
        r = await self._run_fn(fn, total, f"{name}_right")
        if r.get("ok"):
            r.update(gesture=name, direction="right")
        return r

    async def _run_fn(self, fn, total, label):
        """Start a gesture function through the Playground's blend (C, D052) and
        wait until its exit blend is done. Refusals come back as busy."""
        sim = self.sim
        why = await sim.call(lambda: sim.start_gesture(fn, total, name=label))
        if why:
            return {"ok": False, "error": "busy", "hint": str(why)}
        sim.mode = "gesturing"
        try:
            sim.events.append(("gesture", label))
        except Exception:
            pass
        t_end = time.monotonic() + total / max(getattr(sim, "speed", 1.0), 0.05) + 4.0
        await asyncio.sleep(0.1)
        while getattr(sim, "gesture_busy", False) and time.monotonic() < t_end:
            await asyncio.sleep(0.05)
        sim.mode = "idle"
        pose = sim.pose() if hasattr(sim, "pose") else None
        return {"ok": True, "gesture": label, "duration_s": round(float(total), 1), "pose": pose}

    # ---------------------------------------------------------- gesture studio
    def _spec(self, name=None, keyframes=None, loop=False, end="stand", description=None, spec=None):
        if spec is not None:
            if isinstance(spec, str):
                spec, err = parse_args_json(spec)
                if err:
                    raise ValueError(f"spec: {err}")
            if not isinstance(spec, dict):
                raise ValueError("spec must be an object {name, keyframes}")
            return dict(spec)
        if isinstance(keyframes, str):
            try:
                keyframes = json.loads(keyframes, parse_constant=_no_const)
            except ValueError as e:
                raise ValueError(f"keyframes are not valid JSON: {e}") from None
        if isinstance(keyframes, dict) and "keyframes" in keyframes:
            keyframes = keyframes["keyframes"]
        if not isinstance(keyframes, list) or not keyframes:
            raise ValueError("keyframes must be a non-empty list of frames")
        out = {"name": str(name or "composed"), "loop": bool(loop), "end": end or "stand",
               "keyframes": keyframes}
        if description:
            out["description"] = str(description)[:300]
        return out

    async def _check(self, spec):
        import pebble_feasibility as pf
        model = getattr(self.sim, "model", None)
        g = getattr(self.sim, "gait", None)
        rep = await asyncio.to_thread(pf.check_spec, spec, g, 100.0, model)
        return rep

    async def check_gesture(self, spec=None, name=None, **kw):
        if spec is None and name is not None:
            if self.composed is not None and name == self.composed["name"]:
                spec = self.composed["spec"]
            else:
                kg = (getattr(self.sim, "gestures", {}) or {}).get(name, (None,))[0]
                if not hasattr(kg, "spec"):
                    return {"ok": False, "error": f"{name!r} is not a keyframe gesture or the draft"}
                spec = kg.spec
        try:
            spec = self._spec(spec=spec, **kw) if spec is not None else self._spec(**kw)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        rep = await self._check(spec)
        return {"ok": bool(rep.ok), "feasible": bool(rep.ok), "name": spec.get("name"),
                "report": list(rep.lines), "peak_rad_s": round(float(rep.peak()[0]), 2)}

    async def compose_gesture(self, name, keyframes, description=None, loop=False, end="stand"):
        """Check a described gesture; if feasible, preview it once in the sim and
        keep it as the draft (save_gesture keeps it). Infeasible: nothing moves."""
        try:
            spec = self._spec(name=name, keyframes=keyframes, loop=loop, end=end,
                              description=description)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        rep = await self._check(spec)
        self.composed = {"name": spec["name"], "spec": spec, "ok": bool(rep.ok),
                         "report": list(rep.lines)}
        if not rep.ok:
            return {"ok": False, "feasible": False, "name": spec["name"], "report": list(rep.lines),
                    "hint": "not previewed: fix the FAIL lines and compose again"}
        from pebble_keyframes import KeyframeGesture
        kg = KeyframeGesture(spec)
        if kg.loop:
            return {"ok": True, "feasible": True, "name": spec["name"], "report": list(rep.lines),
                    "previewed": False, "note": "a looping gesture is not auto-previewed; "
                                                "say save to keep it, then play it by name"}
        run = await self._run_fn(kg, kg.total, spec["name"])
        if not run.get("ok"):
            return {"ok": False, "feasible": True, "name": spec["name"], "report": list(rep.lines),
                    "previewed": False, "error": run.get("error"), "hint": run.get("hint")}
        return {"ok": True, "feasible": True, "name": spec["name"], "report": list(rep.lines),
                "previewed": True, "duration_s": round(kg.total, 1),
                "note": "previewed, say save to keep it"}

    async def save_gesture(self, name=None, overwrite=False):
        if self.composed is None:
            return {"ok": False, "error": "nothing composed yet — compose_gesture first"}
        from pebble_keyframes import save_keyframe_gesture, _safe_name
        spec = dict(self.composed["spec"])
        try:
            spec["name"] = _safe_name(str(name or spec["name"]))
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        g = getattr(self.sim, "gestures", {}) or {}
        if spec["name"] in g and not hasattr(g[spec["name"]][0], "spec"):
            return {"ok": False, "error": f"{spec['name']!r} is a built-in gesture — pick another name"}
        if spec["name"] in g and not overwrite:
            return {"ok": False, "error": f"{spec['name']!r} exists — pass overwrite=true or a new name"}
        try:
            path = await asyncio.to_thread(save_keyframe_gesture, spec)
        except (ValueError, KeyError) as e:
            return {"ok": False, "error": str(e)}
        if hasattr(self.sim, "reload_library"):
            self.sim.reload_library()
        self._log(f"gesture saved: {os.path.relpath(path, ROOT)}")
        self.composed = dict(self.composed, name=spec["name"], saved=path)
        return {"ok": True, "name": spec["name"], "path": os.path.relpath(path, ROOT)}

    # ------------------------------------------------------------------ look
    def _eye_b64(self):
        frames = getattr(self.sim, "frames", {}) or {}
        _n, jpg = frames.get("eye", (0, b""))
        return base64.b64encode(jpg).decode() if jpg else None

    async def _vision(self, prompt, model=None, max_tokens=LOOK_MAX_TOKENS):
        """The eye frame + prompt through the vision role's fallback chain.
        -> {"ok": True, "model", "text", "latency_s", "fallback"?} or
        {"ok": False, "error", "tried"}. latency_s is the answering model's call
        alone (a cold model's load time included)."""
        b64 = self._eye_b64()
        if b64 is None:
            return {"ok": False, "error": "no eye frame yet (renderer off?)", "tried": []}
        cat = await self._acatalog()
        chain, tried = self.chain("vision", first=model, cat=cat)
        for m in chain:
            msgs = [{"role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}}]}]
            t0 = time.monotonic()
            try:
                r = await self._acall(m, msgs, None, max_tokens)
            except ModelFailure as e:
                tried.append(f"{m}: {e}")
                continue
            text = r["content"]
            if not text:
                tried.append(f"{m}: empty answer")
                continue
            out = {"ok": True, "model": m, "text": text, "latency_s": round(time.monotonic() - t0, 2)}
            if tried:
                out["fallback"] = tried
            return out
        return {"ok": False, "error": "no vision model answered" +
                (": " + "; ".join(tried) if tried else ""), "tried": tried}

    def _event(self, kind, what):
        try:
            self.sim.events.append((kind, what))
        except Exception:
            pass

    async def look(self, model=None, prompt=None):
        """The eye described by the vision role (prompt: a custom question,
        e.g. the vision bench's floor-safety checks). A standard description
        is remembered (scene memory: kind 'look'; objects it names with a
        direction AND a distance go on the map at confidence 0.2) and kept as
        last_look for the awareness loop; a custom question's answer is not."""
        pose = await self._pose() if prompt is None else None
        v = await self._vision(prompt or VISION_PROMPT, model)
        if not v["ok"]:
            if v["error"].startswith("no vision model answered"):
                v["error"] = v["error"].replace("answered", "described the frame", 1)
            return v
        text = v["text"]
        self._event("look", text[:60])
        self._log(f"look ({v['model']}): {text}")
        out = {"ok": True, "model": v["model"], "description": text, "latency_s": v["latency_s"]}
        if v.get("fallback"):
            out["fallback"] = v["fallback"]
        if prompt is None:
            self.last_look = {"text": text, "model": v["model"], "t": time.time(), "pose": pose}
            if pose is not None:
                objs = objects_from_description(text, pose)
                self._remember_obs(kind="look", text=text, pose=pose, objects=objs)
                if objs:
                    out["placed"] = [o["name"] for o in objs]
        return out

    async def detect(self, name, model=None, method="bbox"):
        """Is `name` in the eye frame, and where? method 'bbox' (FIND_PROMPT: the
        model boxes it, geometry gives bearing + distance) or 'estimate'
        (FIND_ESTIMATE_PROMPT: the model says bearing + distance itself), through
        the vision chain, parsed by parse_detection -> {"ok", "model",
        "detection", "raw", "latency_s"} (ok False only when no model answered)."""
        name = str(name or "").strip()[:60]
        if not name:
            return {"ok": False, "error": "detect needs the name of what to look for"}
        if method not in FIND_METHODS:
            return {"ok": False, "error": f"method must be one of {', '.join(FIND_METHODS)}"}
        prompt = (FIND_PROMPT if method == "bbox" else FIND_ESTIMATE_PROMPT).format(name=name)
        v = await self._vision(prompt, model, FIND_MAX_TOKENS)
        if not v["ok"]:
            return v
        det = parse_detection(v["text"], order=box_order(v["model"]))
        out = {"ok": True, "model": v["model"], "detection": det, "raw": v["text"][:300],
               "latency_s": v["latency_s"]}
        if v.get("fallback"):
            out["fallback"] = v["fallback"]
        return out

    # ------------------------------------------------------------- turn + find
    async def _pose(self):
        sim = self.sim
        if not hasattr(sim, "pose"):
            return None
        try:
            return await sim.call(sim.pose)
        except Exception:
            return sim.pose()

    async def turn(self, deg):
        """Turn on the spot by about `deg` degrees (+ = left, counter-clockwise):
        the turn_in_place gait for a duration fitted to the measured turn rate,
        through the Playground's blend like any gesture (so its standstill rule
        and the sim2real locomotion hold apply). Reports the MEASURED turn."""
        d = _num(deg)
        if d is None:
            return {"ok": False, "error": f"turn needs deg as a number, got {deg!r}"}
        if not TURN_MIN_DEG <= abs(d) <= TURN_MAX_DEG:
            return {"ok": False, "error": f"turn deg must be {TURN_MIN_DEG:g}..{TURN_MAX_DEG:g} "
                                          f"in magnitude (+ = left), got {d:g}"}
        import pebble_gestures2 as pg2
        total = abs(d) / TURN_RATE_DPS + TURN_EXTRA_S
        cmd = (0.0, 0.0, math.copysign(pg2.TURN_WZ, d))

        def fn(g, t, total=total, cmd=cmd):
            return pg2._gaited(g, t, total, cmd)
        fn.gaited = True                         # it walks: the sim2real locomotion hold applies
        before = await self._pose()
        r = await self._run_fn(fn, total, f"turn_{'left' if d > 0 else 'right'}_{abs(d):.0f}")
        if r.get("ok"):
            after = r.get("pose") or await self._pose()
            r.update(asked_deg=round(d, 1), duration_s=round(total, 1))
            if before and after:
                r["turned_deg"] = round(_wrap_deg(after["yaw_deg"] - before["yaw_deg"]), 1)
        return r

    def _cmds_since(self, n0):
        log = getattr(self.sim, "cmd_log", None)
        return [] if log is None else [line for _t, line in list(log)[n0:]]

    def _operator_stopped(self, n0):
        """A stop from anywhere since find_object started: the console `stop`,
        the STOP key (teleop ' '), or the stop tool (MCP, chat, /api/tool)."""
        for line in self._cmds_since(n0):
            s = str(line).strip()
            if s == "stop" or s == "teleop" or line == "teleop  " or s.startswith("tool stop"):
                return True
        return False

    async def _fresh_eye(self, n_prev):
        """Wait (up to frame_wait_s) until the eye has rendered 2 frames past
        n_prev, so the next look sees where the robot is NOW."""
        t_end = time.monotonic() + float(getattr(self, "frame_wait_s", 1.0))
        while time.monotonic() < t_end:
            n = (getattr(self.sim, "frames", {}) or {}).get("eye", (0, b""))[0]
            if n >= n_prev + 2:
                return
            await asyncio.sleep(0.05)

    async def find_object(self, name, max_steps=FIND_MAX_STEPS, model=None, method="bbox"):
        """Look -> (turn | walk toward it) -> look ..., until the vision model
        puts it within FIND_NEAR_M of the camera ('found'), max_steps looks
        are used ('not found'), a full scan circle saw nothing, or a goto
        comes back vetoed (cliff / blocked / stuck / ...: 'stopped', reported,
        not retried). Walks only on a confident sighting (>= FIND_MIN_CONF);
        every walk is an ordinary goto (tool_goto: every guard applies).
        Never raises; every step goes to the console and Events."""
        name = str(name or "").strip()[:60]
        if not name:
            return {"ok": False, "error": "find_object needs the name of what to find"}
        if method not in FIND_METHODS:
            return {"ok": False, "error": f"method must be one of {', '.join(FIND_METHODS)}"}
        n = _num(max_steps)
        if n is None:
            return {"ok": False, "error": f"max_steps must be a number, got {max_steps!r}"}
        max_steps = int(max(1, min(FIND_MAX_STEPS_CAP, n)))
        n0 = len(getattr(self.sim, "cmd_log", None) or [])
        steps, turned, used, last_seen = [], 0.0, None, None
        self._log(f"find {name}: start (up to {max_steps} looks)")
        self._event("find", f"{name}: start")

        async def end(found, detail, ok=True, **extra):
            # ok: the behaviour ran to a normal end (found or not found); False for a veto
            # (stopped=...) or a failure (error=...), like goto's own results
            pose = await self._pose()
            out = {"ok": bool(ok), "found": found, "name": name, "detail": detail,
                   "looks": len(steps), "turned_deg": round(turned, 1), "model": used,
                   "steps": steps, "pose": pose}
            if last_seen is not None and not found:
                out["last_seen"] = last_seen
            out.update(extra)
            self._log(f"find {name}: {detail}")
            self._event("find", f"{name}: {detail}"[:80])
            self._remember_obs(kind="find", text=f"find {name}: {detail}", pose=pose)
            self.last_find = {"name": name, "found": bool(found), "detail": detail, "t": time.time(),
                              "model": used, "looks": len(steps)}
            mem = self.memory
            if mem is not None:
                w = mem.where_is(name)
                if w is not None:
                    out["remembered"] = w
            return out

        for i in range(1, max_steps + 1):
            if self._operator_stopped(n0):
                return await end(False, "stopped by the operator", ok=False, stopped="user")
            frame_n = (getattr(self.sim, "frames", {}) or {}).get("eye", (0, b""))[0]
            pose_look = await self._pose() if self.memory is not None else None
            det = await self.detect(name, model=model, method=method)
            if not det.get("ok"):
                err = det.get("error", "no vision model answered")
                if not steps:
                    self._log(f"find {name}: {err}")
                    return {"ok": False, "found": False, "name": name, "error": err,
                            "tried": det.get("tried", []), "steps": []}
                return await end(False, f"the eye stopped answering: {err}", ok=False, error=err)
            used = det["model"]
            d = det["detection"]
            act, step_m = find_policy(d)
            rec = {"step": i, "method": d["method"], "seen": d["seen"], "bearing_deg": d["bearing_deg"],
                   "distance_m": d["distance_m"], "confidence": d["confidence"], "what": d["what"],
                   "parsed": d["parsed"], "action": act, "look_s": det["latency_s"]}
            steps.append(rec)
            if d["seen"]:
                last_seen = {k: d[k] for k in ("bearing_deg", "distance_m", "confidence")}
                at = sighting_to_map(pose_look, d) if pose_look is not None else None
                if at is not None:                   # scene memory: where it is, in the map frame
                    rec["map_xy"] = [at["x"], at["y"]]
                    self._remember_obs(
                        kind="find", pose=pose_look,
                        text=(f"{name} seen at {d['bearing_deg']:+.0f} deg, {d['distance_m']:.2f} m "
                              f"(conf {d['confidence']:.2f}, {d['method']}) -> map ({at['x']:.2f}, {at['y']:.2f})"),
                        # a typed estimate is not geometry (0.30 m median error measured): below
                        # go_back_to's bar whatever confidence the model claims
                        # and a far box is near the horizon, where a pixel is decimetres to metres:
                        # beyond FIND_TRUSTED_M it is a hint for the map, never a goal either
                        objects=[dict(at, name=name, source="find" if d["method"] == "bbox" else "find-estimate",
                                      confidence=min(d["confidence"], FIND_FAR_CONF)
                                      if d["method"] != "bbox" or d["distance_m"] > FIND_TRUSTED_M
                                      else d["confidence"])])
            dist = "?" if d["distance_m"] is None else f"{d['distance_m']:.2f}"
            what = d["what"][:40] if d["parsed"] else f"unparsed: {det['raw'][:40]!r}"
            saw = (f"seen at {d['bearing_deg']:+.0f} deg, {dist} m, conf {d['confidence']:.2f}"
                   if d["seen"] else f"not seen ({what})")
            self._log(f"find {name} {i}/{max_steps} ({used}, {det['latency_s']:.1f} s): {saw} -> {act}"
                      + (f" {step_m:.2f} m" if act == "goto" else ""))
            if act == "stop":
                side = "ahead" if abs(d["bearing_deg"]) < 10 else (
                    "ahead-left" if d["bearing_deg"] < 0 else "ahead-right")
                src = ("from its box and the camera geometry" if d["method"] == "bbox"
                       else "the vision model's own estimate")
                return await end(True, f"found the {name}: ~{d['distance_m']:.2f} m {side} "
                                       f"(bearing {d['bearing_deg']:+.0f} deg, {src})")
            if i == max_steps:
                break                                # no move after the last look: nothing would check it
            if act == "scan":
                if turned + FIND_SCAN_DEG >= 360.0 - 1e-6:
                    return await end(False, f"not found: scanned a full circle ({i} looks)")
                r = await self.tool("turn", {"deg": FIND_SCAN_DEG})
                rec["turn"] = {k: r.get(k) for k in ("ok", "turned_deg", "error", "hint") if k in r}
                if not r.get("ok"):
                    return await end(False, f"the scan turn was refused: {r.get('error')} "
                                            f"{r.get('hint') or ''}".strip(), ok=False, stopped="blocked")
                turned += FIND_SCAN_DEG
            else:
                pose = await self._pose()
                if pose is None:
                    return await end(False, "no pose: cannot aim a goto", ok=False)
                tx, ty = bearing_to_map(pose, d["bearing_deg"], step_m)
                tx, ty = round(tx, 3), round(ty, 3)
                r = await self.tool("goto", {"x": tx, "y": ty})
                how = r.get("stopped") or ("refused" if r.get("ok") is False else "?")
                rec["goto"] = {"x": tx, "y": ty, "step_m": step_m, "stopped": how}
                if r.get("stopped") != "arrived":
                    why = r.get("detail") or r.get("error") or how
                    return await end(False, f"stopped on the way: goto {how} ({why})", ok=False,
                                     stopped=how)
            await self._fresh_eye(frame_n)
        if last_seen is not None:
            ld = "?" if last_seen["distance_m"] is None else f"{last_seen['distance_m']:.2f}"
            detail = (f"not reached within {max_steps} looks (last seen at "
                      f"{last_seen['bearing_deg']:+.0f} deg, ~{ld} m)")
        else:
            detail = f"not found in {max_steps} looks (turned {turned:.0f} deg)"
        return await end(False, detail)

    # ------------------------------------------------------------ scene memory
    def _no_memory(self):
        return {"ok": False, "error": "no scene memory here: it lives in the cockpit (./rocky.sh cockpit)"}

    async def mem_remember(self, note=None, name=None, x=None, y=None):
        """remember(note): the operator's words; 'X is here' / 'X is at (x, y)'
        pin a position. (name + x + y: the cockpit map's 'remember here'.)"""
        mem = self.memory
        if mem is None:
            return self._no_memory()
        note = str(note or "").strip()[:300]
        pose = await self._pose()
        obj = None
        if name is not None and x is not None and y is not None:
            tx, ty, err = validate_goto(x, y)
            if err:
                return {"ok": False, "error": err.replace("goto", "remember")}
            obj = {"name": str(name)[:60], "x": tx, "y": ty, "confidence": 0.9, "source": "user"}
            note = note or f"the {name} is at ({tx:.2f}, {ty:.2f})"
        elif note:
            obj = parse_note(note, pose)         # 'X is here' carries anchor=robot: go_back_to goes onto it
        if not note:
            return {"ok": False, "error": "remember needs the note (what to remember, in words)"}
        if obj is not None and (abs(obj["x"]) > WORLD_MAX_M or abs(obj["y"]) > WORLD_MAX_M):
            # refused out loud: quietly keeping it as a note let the robot say 'remembered' with
            # nothing pinned (review 2026-09-24: a pin at (150, 0) came back ok)
            return {"ok": False, "error": f"({obj['x']:g}, {obj['y']:g}) is off the floor: a pinned place "
                                          f"must be within +-{WORLD_MAX_M:g} m of the origin (map meters)"}
        rec = self._remember_obs(kind="user", text=note, pose=pose, objects=[obj] if obj else [])
        if rec is None:
            return {"ok": False, "error": "not remembered (see the console)"}
        self._event("memory", f"remember: {note}"[:80])
        out = {"ok": True, "remembered": note, "object": rec["objects"][0] if rec["objects"] else None}
        if not rec["objects"]:
            out["note"] = ("kept as a note (no position in it): recall finds it by its words. To pin a "
                           "place, stand the robot at it and say 'the <thing> is here'.")
        return out

    async def mem_where_is(self, name=None):
        mem = self.memory
        if mem is None:
            return self._no_memory()
        name = str(name or "").strip()[:60]
        if not name:
            return {"ok": False, "error": "where_is needs the name of the object"}
        w = mem.where_is(name)
        if w is None:
            notes = mem.notes_about(name)
            out = {"ok": True, "known": False, "name": name,
                   "detail": f"no position remembered for the {name} in this world"
                             + (" (only notes: " + "; ".join(notes) + ")" if notes else "")
                             + " — find_object looks for it"}
            if notes:
                out["notes"] = notes
            return out
        pose = await self._pose()
        out = {"ok": True, "known": True, "name": name, **w}
        rel = ""
        if pose is not None:
            d, side = direction_words(pose, w["x"], w["y"])
            out.update(distance_m=round(d, 2), direction=side)
            rel = f", {d:.2f} m {side} of the robot"
        if w["confidence"] < GO_MIN_CONF:
            flag = " — VAGUE (a look description or a far sighting): find_object to locate it"
        elif w.get("stale"):
            flag = " — STALE: it may have moved; find_object re-checks"
        else:
            flag = ""
        also = ""
        if w.get("also_seen"):
            a = w["also_seen"]
            also = (f"; a vaguer newer sighting ({a['source']}, confidence {a['confidence']:.2f}, "
                    f"{fmt_age(a['age_s'])}) put it at ({a['x']:.2f}, {a['y']:.2f})")
        out["detail"] = (f"the {w['name']} was at ({w['x']:.2f}, {w['y']:.2f}){rel}, seen {fmt_age(w['age_s'])} "
                         f"({w['source']}, confidence {w['confidence']:.2f}"
                         + (", before the last reset / world load" if w.get("earlier_session") else "")
                         + ")" + flag + also)
        return out

    async def mem_recall(self, query="", k=5):
        mem = self.memory
        if mem is None:
            return self._no_memory()
        n = _num(k)
        k = int(max(1, min(20, n if n is not None else 5)))
        pose = await self._pose()
        hits = mem.recall(str(query or "")[:200], k=k, pose=pose)
        return {"ok": True, "query": str(query or ""), "found": len(hits),
                "entries": [{"kind": h["kind"], "text": h["text"], "age": fmt_age(h["age_s"]),
                             "objects": [{k2: o[k2] for k2 in ("name", "x", "y", "confidence")} for o in h["objects"]]}
                            for h in hits],
                "summary": mem.summary(pose)}

    async def mem_forget(self, name=None):
        mem = self.memory
        if mem is None:
            return self._no_memory()
        name = str(name or "").strip()[:60]
        scope = forget_scope(name)[0] if name else "unclear"
        if scope == "unclear":                # 'forget that' names nothing: it must not wipe everything
            return {"ok": False, "error": f"forget what? {name!r} names no object — name it ('forget the "
                                          "ball'), or 'all' to forget everything"}
        n = mem.forget("all" if scope == "all" else name)
        self._event("memory", f"forget {name}: {n} records")
        self._log(f"memory: forgot {name} ({n} records)")
        return {"ok": True, "forgot": name, "records": n}

    async def mem_go_back_to(self, name=None):
        """Walk back to a remembered object: gotos of at most GO_BACK_LEG_M
        toward it until within the standoff (find_object's stopping distance
        plus half its size; 0 for a place pinned with 'X is here'). Every leg
        is an ordinary goto (tool_goto: every guard); any veto ends it."""
        mem = self.memory
        if mem is None:
            return self._no_memory()
        name = str(name or "").strip()[:60]
        if not name:
            return {"ok": False, "error": "go_back_to needs the name of a remembered object"}
        w = mem.where_is(name)
        if w is None:
            notes = mem.notes_about(name)
            return {"ok": False, "error": "not remembered", "name": name,
                    "hint": f"no position remembered for the {name} — find_object looks for it"
                            + (" (notes: " + "; ".join(notes) + ")" if notes else "")}
        if w["confidence"] < GO_MIN_CONF:
            return {"ok": False, "error": f"only a vague position for the {w['name']} (confidence "
                                          f"{w['confidence']:.2f}, source {w['source']})",
                    "where": w, "hint": "find_object locates it with the eye's box geometry"}
        if w.get("earlier_session") and w.get("stale"):
            # the world respawned its bodies since (a reset / world load): 'back at the ball'
            # would be a claim about where it USED to be (review 2026-09-24). Pinned places
            # (anchor robot) and the operator's own pins are not stale / not refused.
            if w.get("source") != "user":
                return {"ok": False, "error": f"the {w['name']} was last seen before the last reset / "
                                              "world load: it may be anywhere now",
                        "where": w, "hint": f"find_object looks for it (or goto ({w['x']:.2f}, {w['y']:.2f}) "
                                            "to walk to where it was)"}
        standoff = 0.0 if w.get("anchor") == "robot" else \
            GO_BACK_STANDOFF_M + (w.get("size_m") or GO_BACK_DEFAULT_SIZE_M) / 2.0
        n0 = len(getattr(self.sim, "cmd_log", None) or [])
        legs = []
        self._log(f"go back to {name}: remembered at ({w['x']:.2f}, {w['y']:.2f}), {fmt_age(w['age_s'])}, "
                  f"stopping {standoff:.2f} m short")
        self._event("memory", f"go back to {name}")

        def result(ok, detail, pose, **extra):
            self._log(f"go back to {name}: {detail}")
            self._event("memory", f"{name}: {detail}"[:80])
            return {"ok": ok, "arrived": ok, "name": name, "where": w, "detail": detail, "legs": legs,
                    "pose": pose, **extra}

        for _ in range(GO_BACK_MAX_LEGS):
            pose = await self._pose()
            if pose is None:
                return result(False, "no pose: cannot aim a goto", None)
            if self._operator_stopped(n0):
                return result(False, "stopped by the operator", pose, stopped="user")
            d = math.hypot(w["x"] - pose["x"], w["y"] - pose["y"])
            if d <= standoff + 0.05:
                stale = (" — STALE: it may have moved since; find_object re-checks" if w.get("stale")
                         else "")
                return result(True, f"back at the {w['name']}: {d:.2f} m from where it was remembered "
                                    f"({fmt_age(w['age_s'])}; not re-checked with the eye){stale}", pose,
                              distance_m=round(d, 3), stale=bool(w.get("stale")))
            step = min(d - standoff, GO_BACK_LEG_M)
            tx = round(pose["x"] + (w["x"] - pose["x"]) / d * step, 3)
            ty = round(pose["y"] + (w["y"] - pose["y"]) / d * step, 3)
            r = await self.tool("goto", {"x": tx, "y": ty})
            how = r.get("stopped") or ("refused" if r.get("ok") is False else "?")
            legs.append({"x": tx, "y": ty, "stopped": how})
            if how != "arrived":
                why = r.get("detail") or r.get("error") or how
                return result(False, f"stopped on the way: goto {how} ({why})", r.get("pose") or pose,
                              stopped=how)
        pose = await self._pose()
        return result(False, f"not there after {GO_BACK_MAX_LEGS} legs", pose, stopped="timeout")

    # ------------------------------------------------------------------ voice
    async def transcribe(self, raw, mode=None, trusted=False):
        """Browser audio blob -> ffmpeg 16 kHz mono -> whisper verbose_json ->
        cleaned text. Returns {ok, text, motion_blocked, wake, dropped}."""
        # A phone that lost the hold (long-press callout, a scroll, a synthetic
        # mouse event) sends a container with no audio in it; say so with the
        # size instead of a misleading "nothing heard" (D052 phone test).
        if not raw or len(raw) < MIN_VOICE_BYTES:
            return {"ok": False, "error": f"recording too short ({len(raw) if raw else 0} bytes) — "
                                          "hold the button while you speak, or tap once to start and "
                                          "tap again to stop"}
        if not shutil.which("ffmpeg"):
            return {"ok": False, "error": "ffmpeg not installed"}
        with tempfile.TemporaryDirectory() as td:
            src, wav = os.path.join(td, "in.webm"), os.path.join(td, "in.wav")
            with open(src, "wb") as f:
                f.write(raw)
            await asyncio.to_thread(subprocess.run, ["ffmpeg", "-y", "-loglevel", "error", "-i", src,
                                                     "-ar", "16000", "-ac", "1", wav], check=False)
            if not os.path.exists(wav):
                return {"ok": False, "error": "could not decode the audio"}
            audio_s = max(os.path.getsize(wav) - 44, 0) / 32000.0        # 16 kHz mono int16
            if audio_s < MIN_VOICE_S:
                self._log(f"voice: {audio_s:.2f} s of audio in {len(raw)} bytes — too short")
                return {"ok": False, "audio_s": round(audio_s, 2),
                        "error": f"recording too short ({audio_s:.1f} s) — hold the button while you speak, "
                                 "or tap once to start and tap again to stop"}
            import httpx
            try:
                async with httpx.AsyncClient(timeout=60) as c:
                    with open(wav, "rb") as fw:
                        r = await c.post(f"{WHISPER_URL}/v1/audio/transcriptions",
                                         files={"file": ("in.wav", fw, "audio/wav")},
                                         data={"response_format": "verbose_json", "temperature": "0.0",
                                               "language": "en", "prompt": WHISPER_PROMPT})
                if r.status_code != 200:
                    return {"ok": False, "audio_s": round(audio_s, 2),
                            "error": f"whisper-server HTTP {r.status_code}: {r.text[:160]}"}
                try:
                    data = r.json()
                except ValueError:
                    return {"ok": False, "audio_s": round(audio_s, 2),
                            "error": f"whisper-server answered non-JSON: {r.text[:160]}"}
            except Exception as e:
                return {"ok": False, "error": f"whisper-server: {e}"}
        out = self.voice_result(data, mode)
        out["audio_s"] = round(audio_s, 2)
        if trusted and out.get("text"):            # hands-free: the operator opted out of the wake word
            out["motion_blocked"] = False
            out["trusted"] = True
        if not out.get("text"):
            self._log(f"voice: {audio_s:.1f} s of audio, whisper heard nothing"
                      + (f" (dropped: {'; '.join(out['dropped'])})" if out.get("dropped") else ""))
        return out

    def voice_result(self, data, mode=None):
        """whisper output -> the /api/voice JSON (pure apart from remembering
        the transcript for the chat's voice check)."""
        c = clean_transcript(data)
        text = c["text"]
        mode = mode or self.state["mode"]
        wake = has_wake_word(text)
        # talk mode knows exactly what moves; an LLM brain may turn any sentence into a goto
        motion = bool(text) and (mode != "talk" or moves(intent_plan(text)))
        self._voice_last = (time.monotonic(), text)
        if text:
            self._log(f"voice: {text}")
        return {"ok": True, "text": text, "wake": wake, "motion_blocked": motion and not wake,
                "dropped": c["dropped"]}

    def _recent_voice(self, text):
        t, last = self._voice_last
        return bool(last) and text.strip() == last.strip() and time.monotonic() - t < VOICE_RECENT_S

    # ------------------------------------------------------------------- chat
    def _lock(self, mode):
        if mode not in self._locks:
            self._locks[mode] = asyncio.Lock()
        return self._locks[mode]

    def _accept_model(self, mode, model):
        """A per-message model from the (legacy) UI: honoured when it fits the mode."""
        if not model or mode == "talk":
            return ""
        key = {"local": "model", "multimodal": "multimodal_model", "claude": "claude_model"}[mode]
        cat = self._cat[1]
        cmap = {m["id"]: m for m in cat} if cat is not None else None
        err = self._role_error(key, str(model), cmap)
        if err:
            return "" if mode in ("claude", "multimodal") else f"({err}; kept {self.state[key]})"
        self.state[key] = str(model)
        return ""

    async def chat(self, text, mode=None, model=None, source=None, trusted=False):
        """One operator message -> {reply, trace, mode, model?, fallback?, voice,
        motion_allowed, motion_blocked}. source 'voice' | 'typed' | None (None:
        voice iff the text equals the last transcript — the legacy UI auto-sent it)."""
        text = str(text or "").strip()[:MAX_TEXT]
        mode = mode or self.state["mode"]
        if mode not in MODES:
            return {"reply": f"unknown mode {mode!r} (have {', '.join(MODES)})", "trace": [], "mode": mode}
        self.state["mode"] = mode
        voice = source == "voice" or (source is None and self._recent_voice(text))
        allow = bool(trusted) or not voice or has_wake_word(text)
        if not text:
            return {"reply": "", "trace": [], "mode": mode}
        note = self._accept_model(mode, model)
        async with self._lock(mode):
            if mode == "talk":
                r = await self._talk(text, allow)
            elif mode == "claude":
                r = await self._claude(text, allow)
            else:
                r = await self._llm(text, mode, allow)
        if note:
            r["reply"] = f"{note} {r['reply']}"
        blocked = any(isinstance(t.get("result"), dict) and t["result"].get("error") == "voice_unconfirmed"
                      for t in r["trace"])
        r.update(voice=voice, motion_allowed=allow, motion_blocked=blocked)
        if blocked:
            r["reply"] += (" (heard by voice without the wake word, so I did not move — say "
                           "'pebble, …' or press Enter to confirm)")
        return r

    async def _talk(self, text, allow):
        p = intent_plan(text)
        results = await intent_execute(self.surface(voice=not allow), p, allow_motion=allow)
        if p["relative"] is not None:                 # execute() turned the delta into a goto
            argl = [{"dx_m": p["relative"][0], "dy_m": p["relative"][1]}]
        else:
            argl = [a for _, a in p["calls"]]
        trace = [{"tool": n, "args": a, "result": r}
                 for (n, r), a in zip(results, argl + [{}] * len(results))]
        reply = p["reply"]
        if results:
            name, res = results[-1]
            if name == "find_object" and isinstance(res, dict) and res.get("error") != "voice_unconfirmed":
                reply = (res.get("detail") if res.get("detail") else
                         f"find_object failed: {res.get('error')}")
            elif name in MEMORY_TOOLS and isinstance(res, dict) and res.get("error") != "voice_unconfirmed":
                reply = _memory_reply(name, res)
            elif name == "look" and isinstance(res, dict):
                reply = (f"eye ({res.get('model')}): {res['description']}" if res.get("ok")
                         else f"look failed: {res.get('error')}")
            elif name in ("status", "scan_summary"):
                reply += " " + json.dumps(res, default=str)[:400]
            elif isinstance(res, dict) and res.get("ok") is False and res.get("error") != "voice_unconfirmed":
                reply += f" — {name} refused: {res.get('error')}" + (
                    f" ({res['hint']})" if res.get("hint") else "")
            elif isinstance(res, dict) and res.get("stopped") and res.get("stopped") != "arrived":
                reply += f" — stopped: {res['stopped']}"
        return {"reply": reply, "trace": trace, "mode": "talk"}

    async def situation_text(self):
        """The cockpit's CURRENT situation line (composed fresh in the sim
        thread: pose, lidar, memory, guards), or None (no cockpit, or it failed)."""
        f = getattr(self.sim, "situation_now", None)
        if f is None:
            return None
        try:
            s = await f()
        except Exception as e:                                 # noqa: BLE001 — never block a chat turn
            self._log(f"situation: {type(e).__name__}: {e}")
            return None
        t = s.get("text") if isinstance(s, dict) else None
        if not t:
            return None
        t = str(t)
        if len(t) > SITUATION_CHARS:                           # cut at a word: the tail is the least useful
            t = t[:SITUATION_CHARS - 1].rsplit(" ", 1)[0] + "…"
        return t

    @staticmethod
    def _with_situation(text, situation):
        """The situation rides in the CURRENT user turn, not the system prompt:
        llama.cpp reuses the KV cache of the longest unchanged prefix, and the
        system prompt sits before the tool list (~3k tokens) in the Qwen
        template — a changing system prompt re-prefills all of it every turn.
        The history keeps the operator's words only."""
        return f"Situation: {situation}\n\nOperator: {text}" if situation else text

    def _messages(self, mode, text, image_b64=None, situation=None):
        msgs = [{"role": "system", "content": self.system_for(multimodal=mode == "multimodal")}]
        for turn in self.hist.get(mode, [])[-MAX_TURNS:]:
            msgs.append({"role": "user", "content": turn["user"]})
            msgs.append({"role": "assistant", "content": turn["reply"]})
        text = self._with_situation(text, situation)
        if image_b64:
            msgs.append({"role": "user", "content": [
                {"type": "text", "text": text},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}}]})
        else:
            msgs.append({"role": "user", "content": text})
        return msgs

    def _remember(self, mode, text, reply, trace):
        h = self.hist.setdefault(mode, [])
        h.append({"user": text, "reply": reply or "(no reply)", "tools": [t["tool"] for t in trace],
                  "t": round(time.time(), 1)})
        del h[:-MAX_TURNS]

    async def _tool_loop(self, model, msgs, tools, trace, allow, look_model=None):
        """(reply, gave_up). Raises ModelFailure only from a model call — by
        then every earlier tool_call already has its result in msgs."""
        for hop in range(MAX_HOPS):
            r = await self._acall(model, msgs, tools, REPLY_MAX_TOKENS)
            calls = r["tool_calls"]
            if not calls:
                msgs.append({"role": "assistant", "content": r["content"]})
                return r["content"] or (f"done: {summarize_trace(trace)}" if trace
                                        else "(the model answered with no text)"), False
            parsed = []
            for i, tc in enumerate(calls):
                args, err = parse_args_json(tc.get("arguments"))
                parsed.append((tc.get("id") or f"call_{hop}_{i}", str(tc.get("name")), args, err))
            msgs.append({"role": "assistant", "content": r["content"], "tool_calls": [
                {"id": cid, "type": "function",
                 "function": {"name": n, "arguments": json.dumps(a if a is not None else {})}}
                for cid, n, a, _e in parsed]})
            for cid, n, a, err in parsed:
                res = ({"ok": False, "error": f"bad arguments for {n}: {err}"} if err
                       else await self.tool(n, a, voice=not allow, look_model=look_model))
                trace.append({"tool": n, "args": a or {}, "result": res})
                msgs.append({"role": "tool", "tool_call_id": cid,
                             "content": json.dumps(res, default=str)[:RESULT_CHARS]})
        return (f"gave up after {MAX_HOPS} tool rounds without a final answer "
                f"(last: {summarize_trace(trace)})"), True

    async def _llm(self, text, mode, allow):
        trace, notes = [], []
        situation = await self.situation_text()
        cat = await self._acatalog()
        stages = []
        image = None
        if mode == "multimodal":
            image = self._eye_b64()
            if image is None:
                notes.append("no eye frame yet — sent text only")
            ch, sk = self.chain("multimodal", cat=cat)
            notes += sk
            stages.append(("multimodal", ch))
        ch, sk = self.chain("brain", cat=cat)
        if mode == "local":
            notes += sk
        stages.append(("brain", ch))
        msgs = self._messages(mode, text, image, situation)
        for stage, chain in stages:
            tools = self.tools_for(look=True)
            for mdl in chain:
                try:
                    reply, gave_up = await self._tool_loop(
                        mdl, msgs, tools, trace, allow,
                        look_model=mdl if stage == "multimodal" else None)
                except ModelFailure as e:
                    notes.append(f"{mdl} failed ({e})")
                    self._log(f"brain {mdl} failed: {e}")
                    continue
                if notes:
                    reply = f"[{'; '.join(notes)} → answered by {mdl}] {reply}"
                self._remember(mode, text, reply, trace)
                return {"reply": reply, "trace": trace, "mode": mode, "model": mdl,
                        "fallback": notes, "gave_up": gave_up}
            if stage == "multimodal":
                _strip_images(msgs)
                notes.append("multimodal chain exhausted — text brain + look")
        # nothing answered: the regex brain, unless tools already ran (never act twice)
        if trace:
            reply = (f"[no model finished the turn: {'; '.join(notes)}] what ran: "
                     f"{summarize_trace(trace)}")
            r = {"reply": reply, "trace": trace, "mode": mode, "model": None}
        else:
            r = await self._talk(text, allow)
            why = "; ".join(notes) or "none available"
            r["reply"] = f"[no model answered: {why} → regex brain] {r['reply']}"
            r["mode"], r["model"] = mode, "talk"
        r["fallback"] = notes
        self._remember(mode, text, r["reply"], trace)
        return r

    async def _claude(self, text, allow):
        if not self.claude_available():
            return {"reply": "Claude in the cockpit needs `pip install anthropic` and ANTHROPIC_API_KEY. "
                             "Without a key, run `./rocky.sh chat` in a terminal: with the cockpit up, "
                             "Claude Code drives THIS sim over MCP and you watch it here.",
                    "trace": [], "mode": "claude"}
        import anthropic
        client = anthropic.Anthropic(timeout=self.timeout_s, max_retries=0)
        model = self.state["claude_model"]
        tools = [{"name": t["function"]["name"], "description": t["function"]["description"],
                  "input_schema": t["function"]["parameters"]} for t in self.tools_for(look=True)]
        system = self.system_for()
        msgs = []
        for turn in self.hist.get("claude", [])[-MAX_TURNS:]:
            msgs += [{"role": "user", "content": turn["user"]},
                     {"role": "assistant", "content": turn["reply"]}]
        msgs.append({"role": "user", "content": self._with_situation(text, await self.situation_text())})
        trace, content, gave_up = [], "", True
        for _hop in range(MAX_HOPS):
            try:
                r = await asyncio.wait_for(asyncio.to_thread(lambda: client.messages.create(
                    model=model, max_tokens=REPLY_MAX_TOKENS, system=system, tools=tools,
                    messages=msgs)), self.timeout_s + 5.0)
            except Exception as e:
                err = "timed out" if isinstance(e, asyncio.TimeoutError) else _short_err(e)
                content = f"[Claude {model} failed: {err}] " + (
                    f"what ran: {summarize_trace(trace)}" if trace else "nothing was done")
                gave_up = False
                break
            msgs.append({"role": "assistant", "content": r.content})
            uses = [b for b in r.content if b.type == "tool_use"]
            content = " ".join(b.text for b in r.content if b.type == "text").strip()
            if not uses:
                gave_up = False
                content = content or (f"done: {summarize_trace(trace)}" if trace else "(no text)")
                break
            results = []
            for u in uses:                                  # every tool_use gets its tool_result
                args = dict(u.input) if isinstance(u.input, dict) else {}
                res = await self.tool(u.name, args, voice=not allow)
                trace.append({"tool": u.name, "args": args, "result": res})
                results.append({"type": "tool_result", "tool_use_id": u.id,
                                "content": json.dumps(res, default=str)[:RESULT_CHARS]})
            msgs.append({"role": "user", "content": results})
        if gave_up:
            content = (f"gave up after {MAX_HOPS} tool rounds without a final answer "
                       f"(last: {summarize_trace(trace)})")
        self._remember("claude", text, content, trace)
        return {"reply": content, "trace": trace, "mode": "claude", "model": model, "gave_up": gave_up}


def _memory_reply(name, res):
    """Talk mode's sentence for a memory tool result."""
    if res.get("ok") is False:
        return f"{name}: {res.get('error')}" + (f" — {res['hint']}" if res.get("hint") else "")
    if name == "where_is":
        return res.get("detail") or "?"
    if name == "recall":
        if not res.get("entries"):
            return f"nothing remembered about {res.get('query')!r}. {res.get('summary', '')}".strip()
        return "; ".join(f"{e['text']} ({e['age']})" for e in res["entries"][:4])
    if name == "remember":
        o = res.get("object")
        return (f"remembered: {res['remembered']}" +
                (f" — the {o['name']} pinned at ({o['x']:.2f}, {o['y']:.2f})" if o else " (as a note)"))
    if name == "forget":
        return f"forgot {res.get('forgot')} ({res.get('records')} records)"
    if name == "go_back_to":
        return res.get("detail") or "?"
    return json.dumps(res, default=str)[:300]


def _no_const(c):
    raise ValueError(f"non-finite number {c}")
