"""capabilities (D056) — ONE declarative registry of every tool Rocky's brains can call.

Until D056 the tools were defined four times: harness/local_brain.build_tools (the
OpenAI function list for local LLMs, with the *_DOC texts), sim/cockpit_brains
(EXTRA_TOOLS, GATED, Brains.tool — the cockpit's executor behind /api/tool/<name>),
harness/server.build_server (the FastMCP "rocky" server, whose texts partly differ)
and harness/cockpit_backend (the HTTP proxy). This module holds what they share, as
data, and generates each surface from it:

    caps = build(gestures, lexicon, signed, has_eye, has_memory, is_cockpit[, has_places])
    to_openai_tools(caps)     # == local_brain.build_tools(...) today, byte for byte
    to_anthropic_tools(caps)  # == the cockpit's Claude-mode conversion of that list
    to_mcp_specs(caps)        # the MCP server's tools: name, description, input schema, annotations
    gated_names(caps)         # == cockpit_brains.GATED (as a set) with every capability on
    to_markdown(caps)         # docs/TOOLS.md

ZERO behaviour change is the contract: every description here was moved VERBATIM
from the four places above (where the MCP text and the local-brain text differ
today, both are kept: `description` for the local brains, `mcp_description` for
MCP). The MCP docstring texts keep FastMCP's raw docstring indentation ("\\n        ")
because FastMCP 1.x publishes fn.__doc__ as is. harness/test_capabilities.py pins
the generated lists against the snapshots taken before this module existed.

A capabilities snapshot is a plain JSON dict (build()):
    {format, version, capabilities, gestures, signed, lexicon, envelope, robot, tools}
version = the first 12 hex of sha256 over the canonical JSON without `version`: it
changes when a tool, a gesture, a chord word or the envelope changes, and only then.

Enum-bearing parameters carry a marker ({"$live": "gestures"} / {"$live": "lexicon"})
that build() replaces with "enum": <the live list> (no enum when the list is empty,
exactly like build_tools with gestures=None).

CLI (runs without a cockpit: the canon lists from harness.backend, every capability on):
    python -m harness.capabilities --md                       # docs/TOOLS.md text to stdout
    python -m harness.capabilities --md --out docs/TOOLS.md   # write it
    python -m harness.capabilities --md --check docs/TOOLS.md # exit 1 when the file is stale
    python -m harness.capabilities --json                     # the snapshot JSON

Imports nothing from local_brain / server / cockpit_brains (they will import from
here); the envelope and robot sections come from gait/ when importable, else a stated
fallback. Never prints on import (stdio MCP servers import it).
"""
from __future__ import annotations

import argparse
import copy
import difflib
import functools
import hashlib
import json
import os
import sys
from dataclasses import dataclass, field
from typing import Callable, NamedTuple, Union

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from harness.backend import CHORD_WORDS, GESTURES, SIGNED   # noqa: E402  (pure: no harness imports)

FORMAT = 1
KINDS = ("voice", "motion", "query", "memory", "gesture_authoring", "meta")
SURFACES = ("openai", "mcp", "internal")      # local brains (OpenAI + Claude mode) | MCP | executor only
CAPABILITY_FLAGS = ("cockpit", "eye", "memory", "places")   # places (D057): the cockpit's place
#                                                           recognition is ON (awareness `recognize`):
#   reported by the cockpit itself (GET /api/capabilities lists it; CockpitBackend / AutoBackend pass
#   it on); never derived from a backend's methods — the mock and the in-process sim have no places
LIVE = "$live"                                # parameter marker: {"$live": "gestures" | "lexicon"}
LIVE_LISTS = ("gestures", "lexicon")

# ------------------------------------------------------------------ constants the texts use
# (the values the four definitions use today; test_capabilities pins them against theirs)
GOTO_REACH_M = 1.5        # local_brain.GOTO_REACH_M: ~0.045 m/s against goto's 40 s cap ~ 1.8 m of floor
MOVE_MAX_M = GOTO_REACH_M  # local_brain.MOVE_MAX_M
GOTO_CAP_S = 40.0         # sim/cockpit.py GOTO_CAP_S
GOTO_SPEED_MM_S = 45.0    # sim/cockpit.py V_GOTO (WaveGait.budget fits it into the envelope)
FIND_MAX_STEPS = 6        # sim/cockpit_brains.FIND_MAX_STEPS
FIND_MAX_STEPS_CAP = 16   # sim/cockpit_brains.FIND_MAX_STEPS_CAP
SPAWN_NAME = "start"      # sim/scene_memory.SPAWN_NAME


# ------------------------------------------------------------------ the texts, verbatim
def goto_doc(reach_m: float = GOTO_REACH_M) -> str:
    """local_brain.GOTO_DOC (reach_m = GOTO_REACH_M today)."""
    return ("Walk to (x, y) in METERS, map frame (the robot starts at (0, 0) "
            "facing +x; +y is its left) — a point the operator gives as coordinates or "
            "a remembered position; for a move relative to the robot ('forward 30 cm') "
            "use move instead. Blocks until it ends and returns "
            "stopped= arrived | cliff (the void reflex vetoed it: do NOT retry "
            "toward it) | blocked (EITHER the lidar saw an obstacle in the way "
            "and 2 detours — a 45 deg veer, then a sidestep — did not get past "
            "it: the result's obstacle has range_m and bearing_deg, body frame, "
            "0 = ahead, + = left; pick a target that avoids that side — OR a "
            "guard refused to start: read detail) | stuck (no progress for 3 s: "
            "something the lidar cannot see, lower than the puck, is in the way "
            "— look, then pick a different target; never re-send the same "
            "one) | timeout "
            f"(too far: ~0.045 m/s, 40 s cap, keep targets within ~{reach_m:g} m) | user "
            "(stop was called) | preempted (a newer goto took over) | FELL. "
            "A veto is a NORMAL result: report it.")


def move_doc(max_m: float = MOVE_MAX_M) -> str:
    """local_brain.MOVE_DOC (max_m = MOVE_MAX_M today)."""
    return ("Relative move in the robot's OWN frame, in METERS: forward_m + ahead / - back, "
            "left_m + left / - right (default 0). 'forward 30 cm' = move(forward_m=0.3); "
            "'back up 20 cm' = move(forward_m=-0.2); 'half a meter to your left' = "
            "move(forward_m=0, left_m=0.5). It reads the robot's pose and heading itself (no "
            "status call, no trigonometry) and walks there as a goto: every guard applies and it "
            "ends the same way, stopped= arrived | cliff | stuck | blocked | timeout | user | "
            f"preempted | FELL (a veto is a NORMAL result: report it, do not retry). At most "
            f"{max_m:g} m. goto is for MAP coordinates.")


GOTO_DOC = goto_doc()
MOVE_DOC = move_doc()
# server.py appends these to the two docs for MCP
MCP_MOVE_SUFFIX = " One motion intent at a time; a newer move or goto preempts it."
MCP_GOTO_SUFFIX = " One motion intent at a time; calling goto again preempts."

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
FIND_DOC = (
    "Find an object by name with the eye and walk up to it (e.g. 'find the ball', 'go to the "
    "box'). It loops by itself: look (a vision model boxes the object; its bearing and distance "
    "come from the camera geometry), then walk 0.1-0.4 m toward it (an ordinary goto: every "
    "guard applies), or turn 30 deg to scan when it is not seen or the sighting is unsure; it "
    "ends found (within ~0.25 m) | not found | stopped (a goto came back cliff / blocked / "
    "stuck: report it, do not retry blindly). Takes up to ~1-2 minutes.")
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
# D057 place recognition (sim/place_memory.py, run by the cockpit): which place is this, from the
# lidar and the eye, never from the world's name. The cockpit's brains and (F3) the MCP server over a
# cockpit, only while recognition is on (capability `places`): off, every tool list is the D056 one.
WHERE_AM_I_DOC = ("Which place is the robot in? Its own recognition from the lidar and the eye (never a map "
                  "name): verdict known | new | ambiguous | unknown with its confidence (0-1, a similarity), "
                  "the place's name, how often it has been here, and the changes two looks agreed on "
                  "(a single look never declares one). Needs place recognition on (the situation line then "
                  "starts with 'place: ...'); off, it says so.")
NAME_PLACE_DOC = ("Name the place the robot is in, in the operator's words: 'I'm in the basement' -> "
                  "name_place(name='basement'); 'this master bedroom has a new chair' -> "
                  "name_place(name='master bedroom'). It names the place the robot recognised when that place "
                  "has no name yet; when the name is another place's, or the recognised place already has a "
                  "different name, the operator is CORRECTING the robot: this is taken to be that other (or a "
                  "new) place and the wrong one is left as it was. rename=true: give the recognised place this "
                  "new name instead ('call this place the study'). new=true when the operator says this is a "
                  "DIFFERENT place ('this is completely new'). Only for what the operator says; needs place "
                  "recognition on.")
PLACES_DOC = ("The places the robot knows: names, visits, what was seen there, and which one it is in now "
              "(from memory: no looking, no walking).")
FORGET_PLACE_DOC = ("Forget a place: its name, 'here' (the place the robot is in) or 'all'. A pronoun ('that', "
                    "'it') forgets nothing. Only when the operator asks.")

# harness/server.py FastMCP(instructions=...)
MCP_INSTRUCTIONS = (
    "Tool server for Pebble/Rocky, a radial pentapod robot. Rocky "
    "understands speech but replies ONLY in chord-speak (never "
    "words). Motion tools are guarded by onboard reflexes: a "
    "result with stopped='cliff', 'stuck', 'blocked' or 'user' is a "
    "normal, successful veto — report it, don't retry blindly.")

# the MCP-only texts (server.py docstrings, raw indentation kept: FastMCP publishes __doc__ as is)
_I8 = "\n        "
_I12 = "\n            "
MCP_STOP_DOC = ("Safe-stop NOW (always accepted): finish the current step, brace" + _I8 +
                "into a full-contact crouch, hold. Resolves any in-flight goto with" + _I8 +
                "stopped='user'.")
MCP_SCAN_SUMMARY_DOC = ("Summarize the latest lidar scan: point count, open frontiers" + _I8 +
                        "(bearing/range), nearest obstacle.")
MCP_STATUS_DOC = ("Robot status: pose, mode (idle|walking|safe_stop), battery" + _I8 +
                  "voltage (mocked until hardware), recent events.")
MCP_LIST_GESTURES_DOC = ("The gestures the robot knows RIGHT NOW (saved keyframe gestures" + _I8 +
                         "included) and which ones take a direction.")
MCP_LOOK_DOC = ("Look through the robot's eye camera: a vision model describes" + _I12 +
                "what is in front of the robot (obstacles, objects, open floor).")
MCP_FIND_DOC = ("Find an object by name with the eye and walk up to it ('ball'," + _I12 +
                "'box'). Runs in the cockpit by itself: look (a vision model boxes" + _I12 +
                "it; bearing and distance come from the camera geometry), then a" + _I12 +
                "0.1-0.4 m goto toward it (every guard applies) or a 30 deg scan turn" + _I12 +
                "when it is unseen or unsure. Ends found (within ~0.25 m) | not" + _I12 +
                "found | stopped (a goto came back cliff / blocked / stuck: a veto," + _I12 +
                "report it). Up to ~1-2 minutes; stop ends it.")


def _gesture_doc(caps: dict) -> str:
    """local_brain.build_tools' gesture text (signed = the signed argument)."""
    return ("Perform one gesture and wait for it to finish. Refused with "
            "error='busy' while walking: stop first. direction left|right "
            f"applies to {' and '.join(caps['signed'])} only (turn_in_place turns "
            "on the spot for ~5 s; left = counter-clockwise).")


def _mcp_say_doc(caps: dict) -> str:
    return ("Speak one chord-speak word through Rocky's voice. word: one of "
            f"{', '.join(caps['lexicon'])}. Unknown words are refused with the lexicon in "
            "the error (Rocky never speaks human words — canon).")


def _mcp_gesture_doc(caps: dict) -> str:
    return (f"Perform a gesture. Available: {', '.join(caps['gestures'])}. direction "
            f"'left'|'right' applies to {' and '.join(caps['signed'])} only (turn_in_place "
            "turns on the spot for ~5 s; left = counter-clockwise). Refused with "
            "error='busy' while walking — stop() first. list_gestures has the "
            "current list.")


def _goto_doc(caps: dict) -> str:
    return goto_doc(caps["envelope"]["goto_reach_m"])


def _move_doc(caps: dict) -> str:
    return move_doc(caps["envelope"]["move_max_m"])


def _mcp_goto_doc(caps: dict) -> str:
    return _goto_doc(caps) + MCP_GOTO_SUFFIX


def _mcp_move_doc(caps: dict) -> str:
    return _move_doc(caps) + MCP_MOVE_SUFFIX


# ------------------------------------------------------------------ the spec types
_REQ = object()


class MCPArg(NamedTuple):
    """One MCP tool argument as FastMCP/pydantic publishes it (the schema is generated
    by _mcp_schema, which reproduces pydantic's output: title, default, type)."""
    name: str
    type: str                 # JSON schema type: string | number | integer | boolean | object
    default: object = _REQ    # _REQ = required


def _mcp_schema(tool: str, args) -> dict:
    """The input schema FastMCP 1.x generates for `async def tool(<args>)`."""
    props, req = {}, []
    for a in args:
        p = {}
        if a.default is _REQ:
            req.append(a.name)
        else:
            p["default"] = a.default
        p["title"] = a.name.title().replace("_", " ")
        p["type"] = a.type
        props[a.name] = p
    out = {"properties": props}
    if req:
        out["required"] = req
    out["title"] = f"{tool}Arguments"
    out["type"] = "object"
    return out


Text = Union[str, Callable[[dict], str]]


@dataclass(frozen=True)
class ToolSpec:
    """One tool. description / mcp_description: a str, or a callable taking the
    capabilities dict being built ({gestures, signed, lexicon, envelope, robot, ...})
    for the texts that enumerate live lists or the envelope."""
    name: str
    kind: str                                  # KINDS
    description: Text                          # the local brains' text (OpenAI + Claude mode)
    parameters: dict                           # OpenAI JSON schema; enums as {"$live": ...}
    mcp_description: Text | None = None        # None: the same text as description
    mcp_args: tuple | None = None              # MCPArg... ; None: not an MCP tool today
    annotations: dict = field(default_factory=dict)   # MCP ToolAnnotations hints
    gated: bool = False                        # a spoken line needs the wake word (cockpit GATED)
    requires: frozenset = frozenset()          # CAPABILITY_FLAGS the backend must have
    backend_method: str = ""                   # the backend attribute ("" -> name)
    surfaces: tuple = ("openai", "mcp")        # SURFACES
    note: str = ""                             # where today's definitions disagree, and how it routes

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"{self.name}: kind {self.kind!r} not in {KINDS}")
        bad = set(self.surfaces) - set(SURFACES)
        if bad:
            raise ValueError(f"{self.name}: unknown surfaces {sorted(bad)}")
        object.__setattr__(self, "requires", frozenset(self.requires))
        unknown = self.requires - set(CAPABILITY_FLAGS)
        if unknown:
            raise ValueError(f"{self.name}: unknown capabilities {sorted(unknown)}")
        if ("mcp" in self.surfaces) != (self.mcp_args is not None):
            raise ValueError(f"{self.name}: an MCP tool needs mcp_args, and only an MCP tool has them")
        if not self.backend_method:
            object.__setattr__(self, "backend_method", self.name)

    def mcp_input_schema(self) -> dict | None:
        return None if self.mcp_args is None else _mcp_schema(self.name, self.mcp_args)


def _obj(props=None, required=None) -> dict:
    out = {"type": "object", "properties": dict(props or {})}
    if required is not None:
        out["required"] = list(required)
    return out


_NUM = {"type": "number"}
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

_RO = {"readOnlyHint": True}
_RW = {"readOnlyHint": False}

# ------------------------------------------------------------------ THE REGISTRY (today's order)
REGISTRY: tuple = (
    ToolSpec(
        "say", "voice",
        "Speak one chord-speak word through the speakers (the robot's "
        "only voice — it never speaks human words).",
        _obj({"word": {"type": "string", "description": "a chord-speak word", LIVE: "lexicon"}}, ["word"]),
        mcp_description=_mcp_say_doc,
        mcp_args=(MCPArg("word", "string"),),
        annotations={"readOnlyHint": False, "idempotentHint": True}),
    ToolSpec(
        "gesture", "motion", _gesture_doc,
        _obj({"name": {"type": "string", "description": "gesture name", LIVE: "gestures"},
              "direction": {"type": "string", "enum": ["left", "right"]}}, ["name"]),
        mcp_description=_mcp_gesture_doc,
        mcp_args=(MCPArg("name", "string"), MCPArg("direction", "string", "")),
        annotations=dict(_RW), gated=True,
        note="MCP direction is a plain string defaulting to '' (no enum); the MCP text lists the "
             "gestures, the local-brain text puts them in the enum instead."),
    ToolSpec(
        "move", "motion", _move_doc,
        _obj({"forward_m": {"type": "number",
                            "description": "meters, + ahead / - back (0.3 = 30 cm)"},
              "left_m": {"type": "number",
                         "description": "meters, + left / - right (default 0)"}}, ["forward_m"]),
        mcp_description=_mcp_move_doc,
        mcp_args=(MCPArg("forward_m", "number"), MCPArg("left_m", "number", 0.0)),
        annotations=dict(_RW), gated=True,
        note="The harness backends have no move method: the MCP server and local_brain.call_tool "
             "run harness.local_brain.move_via (status + goto; a cockpit's own /api/tool/move). "
             "The cockpit executor runs Brains.move."),
    ToolSpec(
        "goto", "motion", _goto_doc,
        _obj({"x": dict(_NUM), "y": dict(_NUM)}, ["x", "y"]),
        mcp_description=_mcp_goto_doc,
        mcp_args=(MCPArg("x", "number"), MCPArg("y", "number")),
        annotations=dict(_RW), gated=True),
    ToolSpec(
        "stop", "motion",
        "Safe-stop immediately (PLANT->BRACE). Always accepted.",
        _obj(),
        mcp_description=MCP_STOP_DOC, mcp_args=(),
        annotations={"readOnlyHint": False, "idempotentHint": True},
        note="Never gated: a stop by voice needs no wake word."),
    ToolSpec(
        "scan_summary", "query",
        "What the lidar sees: nearest obstacles by direction (walls "
        "and objects yes; holes in the floor no).",
        _obj(),
        mcp_description=MCP_SCAN_SUMMARY_DOC, mcp_args=(), annotations=dict(_RO)),
    ToolSpec(
        "status", "query",
        "Robot status: pose (x, y m; yaw deg), mode, reflex state.",
        _obj(),
        mcp_description=MCP_STATUS_DOC, mcp_args=(), annotations=dict(_RO)),
    ToolSpec(
        "list_gestures", "meta",
        "The gestures this robot knows right now (saved ones included).",
        _obj(),
        mcp_description=MCP_LIST_GESTURES_DOC, mcp_args=(), annotations=dict(_RO),
        note="The MCP server answers from harness.backend GESTURES/SIGNED when the backend has "
             "no list_gestures."),
    ToolSpec(
        "look", "query",
        "Look through the robot's eye camera: a vision model describes "
        "what is in front of the robot (obstacles, open floor, objects).",
        _obj(),
        mcp_description=MCP_LOOK_DOC, mcp_args=(), annotations=dict(_RO),
        requires={"eye"},
        note="Local brains: offered when build_tools(look=True) (the cockpit always); MCP: when the "
             "backend has look. The cockpit executor also takes an undeclared 'model' argument."),
    ToolSpec(
        "compose_gesture", "gesture_authoring", COMPOSE_DOC,
        _obj({"name": {"type": "string", "description": "short snake_case name"},
              "description": {"type": "string", "description": "what the gesture should look like"},
              "keyframes": {"type": "array", "items": _FRAME_SCHEMA},
              "loop": {"type": "boolean"}}, ["name", "keyframes"]),
        annotations=dict(_RW), gated=True, requires={"cockpit"}, surfaces=("openai",),
        note="Cockpit brains only; not an MCP tool today."),
    ToolSpec(
        "check_gesture", "gesture_authoring",
        "Check a keyframe gesture spec {name, keyframes, loop?, end?} against the "
        "robot's limits WITHOUT moving; returns the report lines.",
        _obj({"spec": {"type": "object"}}, ["spec"]),
        annotations=dict(_RO), requires={"cockpit"}, surfaces=("openai",),
        note="Cockpit brains only; not an MCP tool today."),
    ToolSpec(
        "save_gesture", "gesture_authoring",
        "Save the last composed (previewed) gesture so it can be played by name. "
        "Only when the operator asked to keep it.",
        _obj({"name": {"type": "string", "description": "optional new name"},
              "overwrite": {"type": "boolean"}}),
        annotations=dict(_RW), requires={"cockpit"}, surfaces=("openai",),
        note="Cockpit brains only; not an MCP tool today. Changes the live gesture list."),
    ToolSpec(
        "find_object", "motion", FIND_DOC,
        _obj({"name": {"type": "string", "description": "what to find, in plain words: 'ball', 'box'"},
              "max_steps": {"type": "integer",
                            "description": f"looks before giving up (default "
                                           f"{FIND_MAX_STEPS}, max {FIND_MAX_STEPS_CAP})"}},
             ["name"]),
        mcp_description=MCP_FIND_DOC,
        mcp_args=(MCPArg("name", "string"), MCPArg("max_steps", "integer", FIND_MAX_STEPS)),
        annotations=dict(_RW), gated=True, requires={"eye"},
        note="Local brains: part of cockpit_brains.EXTRA_TOOLS (offered with the extras, whatever "
             "look says); MCP: when the backend has find_object."),
    ToolSpec(
        "remember", "memory", REMEMBER_DOC,
        _obj({"note": {"type": "string", "description": "the fact, in the operator's words"}}, ["note"]),
        mcp_args=(MCPArg("note", "string"),), annotations=dict(_RW), requires={"memory"}),
    ToolSpec(
        "where_is", "memory", WHERE_IS_DOC,
        _obj({"name": {"type": "string", "description": "the object, in plain words: 'ball'"}}, ["name"]),
        mcp_args=(MCPArg("name", "string"),), annotations=dict(_RO), requires={"memory"}),
    ToolSpec(
        "recall", "memory", RECALL_DOC,
        _obj({"query": {"type": "string"},
              "k": {"type": "integer", "description": "how many entries (default 5, max 20)"}}),
        mcp_args=(MCPArg("query", "string", ""), MCPArg("k", "integer", 5)),
        annotations=dict(_RO), requires={"memory"}),
    ToolSpec(
        "go_back_to", "motion", GO_BACK_DOC,
        _obj({"name": {"type": "string", "description": "the remembered object: 'ball'"}}, ["name"]),
        mcp_args=(MCPArg("name", "string"),), annotations=dict(_RW), gated=True, requires={"memory"},
        note="A memory tool (cockpit_brains.MEMORY_TOOLS: Brains.mem_go_back_to) that walks."),
    ToolSpec(
        "forget", "memory", FORGET_DOC,
        _obj({"name": {"type": "string", "description": "the object, or 'all'"}}, ["name"]),
        mcp_args=(MCPArg("name", "string"),),
        annotations={"readOnlyHint": False, "destructiveHint": True}, requires={"memory"},
        note="Not gated, but the cockpit refuses a spoken forget of 'all' without the wake word."),
    ToolSpec(
        "turn", "motion",
        "INTERNAL (never offered to a model): turn on the spot by about deg degrees (+ = left, "
        "counter-clockwise) with the turn_in_place gait, timed to the measured turn rate; "
        "find_object's scan turn, replayed by recordings. Reports the measured turn.",
        _obj({"deg": {"type": "number", "description": "degrees, + = left; 5..180 in magnitude"}}, ["deg"]),
        annotations=dict(_RW), gated=True, requires={"cockpit"}, surfaces=("internal",),
        note="In the cockpit executor (TOOL_NAMES, /api/tool/turn) and GATED, offered to no model."),
    # ---- D057 place recognition (appended: every tool above keeps its place and its text)
    ToolSpec(
        "where_am_i", "query", WHERE_AM_I_DOC,
        _obj(),
        mcp_args=(),
        annotations=dict(_RO), requires={"cockpit", "places"}, surfaces=("openai", "mcp"),
        note="D057: the cockpit's last place recognition (sim/place_memory.py, run by sim/cockpit.py) + the "
             "place's summary. Offered to the cockpit's models, and by the MCP server over a cockpit "
             "(CockpitBackend / AutoBackend: POST /api/tool/where_am_i), only while place recognition is on "
             "(capability places: POST /api/awareness {recognize: true} or cockpit.py --recognize; the MCP "
             "list follows it on its next refresh); /api/tool/where_am_i answers 'place recognition is off' "
             "otherwise."),
    ToolSpec(
        "name_place", "memory", NAME_PLACE_DOC,
        _obj({"name": {"type": "string", "description": "the place, in the operator's words: 'basement'"},
              "new": {"type": "boolean",
                      "description": "true: this is a different, new place (not the one recognised)"},
              "rename": {"type": "boolean",
                         "description": "true: rename the recognised place to this name (it is the same place)"}},
             ["name"]),
        mcp_args=(MCPArg("name", "string"), MCPArg("new", "boolean", False), MCPArg("rename", "boolean", False)),
        annotations={"readOnlyHint": False}, requires={"cockpit", "places"}, surfaces=("openai", "mcp"),
        note="D057: names an unnamed recognised place (or one stored on this visit), settles an unsure one "
             "to the place of that name, or stores a new place from the last recognition's samples. A name "
             "that is another place's, or differs from the recognised place's own name, is a correction: "
             "what this visit wrote into the wrongly recognised place (samples, objects, scene-memory "
             "records) is taken back and moved to the right one; rename=true renames instead. new=true "
             "twice on one visit stores one place, not two; across visits it stores another each time (not "
             "idempotent). Not gated (it moves nothing). Cockpit brains and MCP over a cockpit (while recognition "
             "is on), /api/tool always. MCP: new / rename are booleans refused otherwise, as the cockpit "
             "refuses them (null = false)."),
    ToolSpec(
        "places", "query", PLACES_DOC,
        _obj(),
        mcp_args=(),
        annotations=dict(_RO), requires={"cockpit", "places"}, surfaces=("openai", "mcp"),
        note="D057: PlaceMemory.places() + the current place. Cockpit brains and MCP over a cockpit (while "
             "recognition is on), /api/tool always."),
    ToolSpec(
        "forget_place", "memory", FORGET_PLACE_DOC,
        _obj({"name": {"type": "string", "description": "the place's name, 'here', or 'all'"}}, ["name"]),
        mcp_args=(MCPArg("name", "string"),),
        annotations={"readOnlyHint": False, "destructiveHint": True}, requires={"cockpit", "places"},
        surfaces=("openai", "mcp"),
        note="D057: not gated, but the cockpit refuses a spoken forget_place('all') without the wake word "
             "(like forget). places.json.bak is written before a wipe; a forgotten place's scene-memory "
             "file stays on disk, unreachable. Cockpit brains and MCP over a cockpit (while recognition is on), "
             "/api/tool always."),
)

BY_NAME = {t.name: t for t in REGISTRY}
TOOL_NAMES = tuple(t.name for t in REGISTRY)              # every name the cockpit executor accepts
GATED_NAMES = frozenset(t.name for t in REGISTRY if t.gated)
MEMORY_NAMES = tuple(t.name for t in REGISTRY if "memory" in t.requires)


# ------------------------------------------------------------------ envelope + robot
# what each envelope field means (docs/TOOLS.md prints it beside the value; the test
# requires one line per field, so a new field cannot be published without its meaning)
ENVELOPE_FIELDS = {
    "goto_reach_m": "m, the reach the goto text advises ('keep targets within ~N m'); advice, "
                  "not a refusal (goto refuses only past cockpit_brains.GOTO_MAX_M)",
    "move_max_m": "m, the longest relative move: move refuses beyond it (local_brain.validate_move)",
    "goto_timeout_s": "s, a goto still walking after this ends as stopped=timeout",
    "goto_speed_m_s": "m/s, the speed goto walks at (what it asks, fitted into the walking envelope)",
    "speed_m_s": "m/s, the top walking speed at any heading (WaveGait.budget)",
    "turn_rad_s": "rad/s, the top turn rate on the spot",
    "step_height_mm": "mm, how high a swinging foot LIFTS (params.yaml gait.step_height, WaveGait.hstep); "
               "not the stride length",
}
FALLBACK_ENVELOPE = {
    "source": "fallback: the D052 numbers (gait/pebble_gait not importable)",
    "goto_reach_m": GOTO_REACH_M, "move_max_m": MOVE_MAX_M, "goto_timeout_s": GOTO_CAP_S,
    "goto_speed_m_s": 0.045, "speed_m_s": 0.0455, "turn_rad_s": 0.246, "step_height_mm": 24.0,
}
FALLBACK_ROBOT = {
    "source": "fallback: the D053 description (gait/rocky_model not importable)",
    "legs": 5, "joints_per_leg": 3, "joint_names": ["yaw", "hip", "knee"], "tool_joints": ["claw"],
}


def _gait_import(mod: str):
    try:
        return __import__(mod)
    except ImportError:
        gait = os.path.join(ROOT, "gait")
        if gait not in sys.path:
            sys.path.append(gait)            # appended, not prepended: never shadows anything
        return __import__(mod)


@functools.lru_cache(maxsize=1)
def _envelope_cached() -> str:
    try:
        pg = _gait_import("pebble_gait")
        g = pg.WaveGait()
        mc = g.max_command()
        v = float(mc["v"])                                       # mm/s, any heading
        env = {
            "source": "gait/pebble_gait.WaveGait().max_command() on cad/params.yaml",
            "goto_reach_m": GOTO_REACH_M, "move_max_m": MOVE_MAX_M, "goto_timeout_s": GOTO_CAP_S,
            "goto_speed_m_s": round(min(GOTO_SPEED_MM_S, v) / 1000.0, 4),
            "speed_m_s": round(v / 1000.0, 4),
            "turn_rad_s": round(float(mc["wz"]), 3),
            "step_height_mm": round(float(g.hstep), 1),                # swing LIFT height, not stride
        }
    except Exception:                                           # noqa: BLE001 — a stated fallback
        env = dict(FALLBACK_ENVELOPE)
    return json.dumps(env)


def default_envelope() -> dict:
    """{source, goto_reach_m, move_max_m, goto_timeout_s, goto_speed_m_s, speed_m_s, turn_rad_s,
    step_height_mm} from WaveGait's budget on params.yaml, else FALLBACK_ENVELOPE. Cached.
    step_height_mm is the swing LIFT height (gait.step_height), not the stride; ENVELOPE_FIELDS
    says what each field means."""
    return json.loads(_envelope_cached())


@functools.lru_cache(maxsize=1)
def _robot_cached() -> str:
    try:
        rm = _gait_import("rocky_model")
        s = rm.robot()
        out = {
            "source": "gait/rocky_model.robot() (cad/params.yaml robot:)",
            "legs": int(s.n_legs),
            "joints_per_leg": int(s.uniform_dof()),
            "joint_names": list(s.leg_joint_names(0)),
            "tool_joints": list(s.tool_names()),
            "stations_deg": [round(float(a), 3) for a in s.stations_deg()],
            "actuators": len(s.actuator_order()),
            "topology_hash": s.topology_hash(),
            "params_rev": s.params_rev,
        }
    except Exception:                                           # noqa: BLE001 — a stated fallback
        out = dict(FALLBACK_ROBOT)
    return json.dumps(out)


def default_robot() -> dict:
    """{source, legs, joints_per_leg, joint_names, tool_joints, ...} from RobotSpec. Cached."""
    return json.loads(_robot_cached())


# ------------------------------------------------------------------ the snapshot
def _fill(schema, lists: dict):
    """Deep copy of an OpenAI schema with every {"$live": k} replaced in place by
    "enum": lists[k] — or dropped when that list is empty (build_tools' `if gestures:`)."""
    if isinstance(schema, list):
        return [_fill(v, lists) for v in schema]
    if not isinstance(schema, dict):
        return copy.deepcopy(schema)
    out = {}
    for k, v in schema.items():
        if k == LIVE:
            if v not in lists:
                raise KeyError(f"unknown live list {v!r}")
            if lists[v]:
                out["enum"] = list(lists[v])
        else:
            out[k] = _fill(v, lists)
    return out


def _text(t: Text, ctx: dict) -> str:
    return t(ctx) if callable(t) else t


def canonical_json(obj) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def snapshot_version(caps: dict) -> str:
    """First 12 hex of sha256 over the canonical JSON of caps without `version`."""
    body = {k: v for k, v in caps.items() if k != "version"}
    return hashlib.sha256(canonical_json(body).encode("utf-8")).hexdigest()[:12]


def build(gestures=None, lexicon=None, signed=None, has_eye=False, has_memory=False,
          is_cockpit=False, envelope=None, robot=None, has_places=False) -> dict:
    """The capabilities snapshot (a plain JSON dict) for one robot as it is now.

    gestures / lexicon: the LIVE lists (None or [] = no enum, as build_tools(None)).
    signed: the gestures that take direction left|right (default harness.backend.SIGNED).
    has_eye / has_memory / is_cockpit: which REGISTRY tools exist (ToolSpec.requires).
    has_places (D057): the cockpit's place recognition is on (the place tools); off by default,
    so every list built without it is exactly the D056 one.
    envelope / robot: default_envelope() / default_robot() when None.
    tools: the available tools in REGISTRY order, every text resolved, every enum filled.
    """
    flags = sorted(f for f, on in (("cockpit", is_cockpit), ("eye", has_eye), ("memory", has_memory),
                                   ("places", has_places)) if on)
    ctx = {
        "format": FORMAT,
        "capabilities": flags,
        "gestures": list(gestures or []),
        "signed": list(SIGNED if signed is None else signed),
        "lexicon": list(lexicon or []),
        "envelope": dict(default_envelope() if envelope is None else envelope),
        "robot": dict(default_robot() if robot is None else robot),
    }
    lists = {"gestures": ctx["gestures"], "lexicon": ctx["lexicon"]}
    tools = []
    for t in REGISTRY:
        if not t.requires <= set(flags):
            continue
        desc = _text(t.description, ctx)
        mdesc = None if t.mcp_description is None else _text(t.mcp_description, ctx)
        tools.append({
            "name": t.name,
            "kind": t.kind,
            "surfaces": list(t.surfaces),
            "gated": bool(t.gated),
            "requires": sorted(t.requires),
            "backend_method": t.backend_method,
            "annotations": dict(t.annotations),
            "description": desc,
            "mcp_description": mdesc,
            "parameters": _fill(t.parameters, lists),
            "mcp_input_schema": t.mcp_input_schema(),
        })
    caps = dict(ctx, tools=tools)
    caps["version"] = snapshot_version(caps)
    return caps


def backend_flags(be) -> dict:
    """build() flags for a harness backend, the way harness/server.py decides today:
    look where the backend has `look`, the memory tools where it has `where_is`.
    (server.py registers find_object on its own hasattr(be, 'find_object'); every
    backend today has both look and find_object, or neither.)"""
    return {"has_eye": hasattr(be, "look"), "has_memory": hasattr(be, "where_is"),
            "is_cockpit": callable(getattr(be, "_tool", None))}


def fallback_caps(**kw) -> dict:
    """No cockpit: the canon lists (harness.backend), every capability on (documents every tool)."""
    args = dict(gestures=list(GESTURES), lexicon=list(CHORD_WORDS), signed=list(SIGNED),
                has_eye=True, has_memory=True, is_cockpit=True, has_places=True)
    args.update(kw)
    return build(**args)


# ------------------------------------------------------------------ generators
def _on(caps: dict, surface: str) -> list:
    return [t for t in caps["tools"] if surface in t["surfaces"]]


def to_openai_tools(caps: dict) -> list:
    """The OpenAI function list — what local_brain.build_tools makes today for the same
    lists and capabilities (look=True <-> has_eye; EXTRA_TOOLS <-> is_cockpit + memory)."""
    return [{"type": "function", "function": {
        "name": t["name"], "description": t["description"],
        "parameters": copy.deepcopy(t["parameters"])}} for t in _on(caps, "openai")]


def to_anthropic_tools(caps: dict) -> list:
    """Claude mode's list (cockpit_brains converts the OpenAI list the same way)."""
    return [{"name": t["name"], "description": t["description"],
             "input_schema": copy.deepcopy(t["parameters"])} for t in _on(caps, "openai")]


def to_mcp_specs(caps: dict) -> list:
    """[{name, description, input_schema, annotations, requires, backend_method, kind, gated}]
    for the MCP server, in registry order. description = mcp_description or description;
    input_schema = what FastMCP generates from today's server.py signatures."""
    out = []
    for t in _on(caps, "mcp"):
        out.append({
            "name": t["name"],
            "description": t["mcp_description"] if t["mcp_description"] is not None else t["description"],
            "input_schema": copy.deepcopy(t["mcp_input_schema"]),
            "annotations": dict(t["annotations"]),
            "requires": list(t["requires"]),
            "backend_method": t["backend_method"],
            "kind": t["kind"],
            "gated": t["gated"],
        })
    return out


def gated_names(caps: dict) -> frozenset:
    """The tools a spoken line may not run without the wake word (cockpit_brains.GATED)."""
    return frozenset(t["name"] for t in caps["tools"] if t["gated"])


def tool_names(caps: dict, surface: str | None = None) -> list:
    return [t["name"] for t in (caps["tools"] if surface is None else _on(caps, surface))]


# ------------------------------------------------------------------ docs/TOOLS.md
_SURFACE_WORDS = {"openai": "local brains", "mcp": "MCP", "internal": "cockpit executor only"}
_LIVE_WORDS = {"gestures": "the gestures (the live list)", "lexicon": "the chord words (the live list)"}


def _flat(s: str) -> str:
    return " ".join(str(s).split())


def _cell(s: str) -> str:
    return _flat(s).replace("|", "\\|")


def _type_str(s: dict) -> str:
    if not isinstance(s, dict):
        return "any"
    ty = s.get("type", "any")
    if ty == "array":
        return f"array of {_type_str(s.get('items', {}))}"
    if ty == "object" and s.get("properties"):
        req = set(s.get("required") or [])
        inner = ", ".join(k + ("*" if k in req else "") for k in s["properties"])
        return f"object {{{inner}}}"
    if "enum" in s:
        return f"{ty}, one of: " + ", ".join(str(v) for v in s["enum"])
    return ty


def _param_rows(schema: dict, raw: dict | None) -> list:
    props = schema.get("properties") or {}
    req = set(schema.get("required") or [])
    rows = []
    for k, v in props.items():
        live = ((raw or {}).get("properties") or {}).get(k, {}).get(LIVE)
        ty = f"{v.get('type', 'any')}, one of {_LIVE_WORDS.get(live, live)}" if live else _type_str(v)
        rows.append(f"| `{k}` | {_cell(ty)} | {'yes' if k in req else 'no'} | "
                    f"{_cell(v.get('description', '')) or '—'} |")
    return rows


def _mcp_args_str(schema: dict | None) -> str:
    if not schema or not schema.get("properties"):
        return "none"
    req = set(schema.get("required") or [])
    parts = []
    for k, v in schema["properties"].items():
        p = f"`{k}` {v.get('type', 'any')}"
        if k not in req:
            p += f" = {json.dumps(v.get('default'))}"
        parts.append(p)
    return ", ".join(parts)


def _annot_str(a: dict) -> str:
    return ", ".join(f"{k}={json.dumps(v)}" for k, v in a.items()) or "none"


def to_markdown(caps: dict) -> str:
    """docs/TOOLS.md: deterministic for a given snapshot."""
    L = []
    add = L.append
    add("# Rocky's tools")
    add("")
    add("<!-- GENERATED by `python -m harness.capabilities --md --out docs/TOOLS.md` from the "
        "registry in harness/capabilities.py (D056). Do not edit by hand: change the registry and "
        "regenerate; CI fails when this file is stale. -->")
    add("")
    add("Every tool a brain can call, from one registry (`harness/capabilities.py` `REGISTRY`). "
        "The local brains' OpenAI tool list (`harness/local_brain.py`, the cockpit's "
        "`sim/cockpit_brains.py`, Claude mode) and the MCP server (`harness/server.py`) are "
        "generated from it; the cockpit's executor (`/api/tool/<name>`) is checked against it "
        "(`sim/cockpit_brains.registry_problems`, run by the tests): a new tool still needs its "
        "handler written there.")
    add("")
    add("This page is the no-cockpit snapshot: the canon gesture and chord-word lists "
        "(`harness/backend.py`) and every capability on. A running cockpit's lists also carry its "
        "saved keyframe gestures and custom chord words.")
    add("")
    add(f"- snapshot version: `{caps['version']}` (sha256 of the snapshot JSON, first 12 hex; it "
        "changes when a tool, a list or the envelope changes)")
    add(f"- capabilities: {', '.join(caps['capabilities']) or 'none'}")
    add("")
    add("## Summary")
    add("")
    add("| tool | kind | gated | requires | offered to |")
    add("|---|---|---|---|---|")
    for t in caps["tools"]:
        add(f"| `{t['name']}` | {t['kind']} | {'yes' if t['gated'] else 'no'} | "
            f"{', '.join(t['requires']) or '—'} | {', '.join(_SURFACE_WORDS[s] for s in t['surfaces'])} |")
    add("")
    add("- **gated**: a spoken line runs it only with the wake word ('pebble, ...') or the "
        "operator's confirmation. `stop` is never gated.")
    add("- **requires**: `eye` = a camera and a vision model, `memory` = the scene memory, "
        "`cockpit` = a running cockpit (`./rocky.sh cockpit`), `places` = the cockpit's place "
        "recognition switched on (`POST /api/awareness {\"recognize\": true}` or `--recognize`; D057), "
        "as the cockpit reports it in `GET /api/capabilities` (the MCP server follows it; the mock and "
        "the in-process sim have no places, so never this flag). "
        "A backend without it does not offer the tool (absent tool > lying tool).")
    add("- **offered to**: local brains = the OpenAI tool list (and Claude mode); MCP = the "
        "`rocky` MCP server; cockpit executor only = callable at `/api/tool/<name>`, offered to "
        "no model.")
    add("")
    env = caps["envelope"]
    add("## Envelope")
    add("")
    add("| quantity | value | meaning |")
    add("|---|---|---|")
    for k, v in env.items():
        if k != "source":
            add(f"| `{k}` | {v} | {_cell(ENVELOPE_FIELDS.get(k, '—'))} |")
    add("")
    add(f"Source: {env.get('source', '?')}.")
    add("")
    rb = caps["robot"]
    add("## Robot")
    add("")
    add("| field | value |")
    add("|---|---|")
    for k, v in rb.items():
        if k != "source":
            add(f"| `{k}` | {_cell(', '.join(map(str, v)) if isinstance(v, list) else v)} |")
    add("")
    add(f"Source: {rb.get('source', '?')}.")
    add("")
    add("## Gestures (as of generation)")
    add("")
    add(", ".join(f"`{g}`" for g in caps["gestures"]) or "(none)")
    add("")
    add("Take `direction` left|right: " + (", ".join(f"`{g}`" for g in caps["signed"]) or "(none)") + ".")
    add("")
    add("## Chord words (as of generation)")
    add("")
    add(", ".join(f"`{w}`" for w in caps["lexicon"]) or "(none)")
    add("")
    add("## Tools")
    for t in caps["tools"]:
        spec = BY_NAME.get(t["name"])
        add("")
        add(f"### `{t['name']}`")
        add("")
        add(f"- kind: {t['kind']}; gated: {'yes' if t['gated'] else 'no'}; requires: "
            f"{', '.join(t['requires']) or '—'}; offered to: "
            f"{', '.join(_SURFACE_WORDS[s] for s in t['surfaces'])}; backend method: `{t['backend_method']}`")
        add(f"- annotations{'' if 'mcp' in t['surfaces'] else ' (not published: no MCP tool today)'}: "
            f"{_annot_str(t['annotations'])}")
        if t["mcp_input_schema"] is not None:
            add(f"- MCP arguments: {_mcp_args_str(t['mcp_input_schema'])}")
        if spec is not None and spec.note:
            add(f"- note: {_flat(spec.note)}")
        add("")
        rows = _param_rows(t["parameters"], spec.parameters if spec else None)
        if rows:
            add("| parameter | type | required | description |")
            add("|---|---|---|---|")
            L.extend(rows)
        else:
            add("No parameters.")
        add("")
        if t["mcp_description"] is None or _flat(t["mcp_description"]) == _flat(t["description"]):
            label = "Description" if "mcp" not in t["surfaces"] else "Description (local brains and MCP)"
            add(f"{label}:")
            add("")
            add("> " + _flat(t["description"]))
        else:
            add("Description (local brains):")
            add("")
            add("> " + _flat(t["description"]))
            add("")
            add("Description (MCP):")
            add("")
            add("> " + _flat(t["mcp_description"]))
    add("")
    return "\n".join(L)


# ------------------------------------------------------------------ CLI
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m harness.capabilities",
                                 description="Rocky's tool registry: docs/TOOLS.md and the snapshot JSON "
                                             "(canon lists, every capability on; no cockpit needed).")
    ap.add_argument("--md", action="store_true", help="markdown (docs/TOOLS.md); the default")
    ap.add_argument("--json", action="store_true", help="the capabilities snapshot as JSON")
    ap.add_argument("--check", metavar="PATH", help="exit 1 when PATH differs from what would be generated")
    ap.add_argument("--out", metavar="PATH", help="write to PATH instead of stdout")
    a = ap.parse_args(argv)
    caps = fallback_caps()
    as_json = a.json and not a.md
    text = (json.dumps(caps, indent=1, ensure_ascii=False) + "\n") if as_json else to_markdown(caps)
    if a.check:
        try:
            with open(a.check, encoding="utf-8") as f:
                have = f.read()
        except OSError as e:
            print(f"{a.check}: {e} — generate it with: python -m harness.capabilities "
                  f"{'--json' if as_json else '--md'} --out {a.check}", file=sys.stderr)
            return 1
        if have != text:
            diff = list(difflib.unified_diff(have.splitlines(), text.splitlines(), a.check, "generated",
                                             lineterm="", n=1))
            print("\n".join(diff[:60]), file=sys.stderr)
            print(f"{a.check} is stale: regenerate with python -m harness.capabilities "
                  f"{'--json' if as_json else '--md'} --out {a.check}", file=sys.stderr)
            return 1
        return 0
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(text)
        return 0
    sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
