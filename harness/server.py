"""Rocky-MCP tool server v0 (docs/MCP_CONTRACT_v0.md) — FastMCP over stdio.

The model proposes, the reflex layer disposes: every tool result is an
ordinary value (vetoes included — a goto stopped by the cliff reflex
returns {"stopped": "cliff"}, it does not raise). The backend owns all
robot state; kill the brain any time and the robot is safe.

Run against the mock (default) or the MuJoCo sim:

    python3 -m harness.server                 # mock backend
    ROCKY_BACKEND=sim python3 -m harness.server   # MuJoCo cliff world
    ROCKY_BACKEND=auto python3 -m harness.server  # the cockpit whenever one answers, else the sim
                                                  # (re-resolved per call, logged to stderr — D052)

Wire into a client (e.g. Claude Code .mcp.json):
    {"rocky": {"command": "python3", "args": ["-m", "harness.server"],
               "cwd": "<repo>/rocky"}}

Tools: say, gesture, move, goto, stop, scan_summary, status, list_gestures, and
look / find_object where an eye exists and remember / where_is / recall /
go_back_to / forget where the scene memory exists (a cockpit backend). map_query / patrol / dock are
not stubbed — absent tool > lying tool. D052: the gesture and say docs are
built from the backend's LIVE lists when the server starts (saved keyframe
gestures and custom chord words included); gesture takes direction
left|right for turn_in_place / sidestep. move (2026-09-25) is a relative move in
the robot's frame (forward_m, left_m): the server reads the pose from the
backend's status and calls the backend's own goto (harness.local_brain.move_via),
so every backend gets it with goto's guards and results, unchanged; a cockpit
backend runs it itself (/api/tool/move: its stop check between the pose read and
the goto). move and goto refuse a boolean for a distance, as the cockpit does
(pydantic's lax mode made move(forward_m=true) a 1 m walk).
"""
from __future__ import annotations

import os
import sys
from typing import Annotated

from mcp.server.fastmcp import FastMCP
from pydantic import BeforeValidator

from harness.backend import MockBackend, CHORD_WORDS, GESTURES, SIGNED
from harness.intent import _accepts
from harness.local_brain import GOTO_DOC, MOVE_DOC, move_via


def _no_bool(v):
    """MCP arguments go through pydantic's lax mode, where true is 1.0: move(forward_m=true)
    walked a metre and goto(true, 0) went to (1, 0), while the cockpit refuses a boolean
    (validate_move / validate_goto). Refused here the same way; the schema stays 'number'."""
    if isinstance(v, bool):
        raise ValueError("meters as a number, got a boolean")     # noqa: TRY004 — pydantic reports a ValueError
    return v


Meters = Annotated[float, BeforeValidator(_no_bool)]
# left_m: null reads as 0 (the default), as the cockpit's validate_move reads it
LeftMeters = Annotated[float, BeforeValidator(lambda v: 0.0 if v is None else _no_bool(v))]


def _lists(be):
    """(gestures, words) the backend really has, else the canon lists."""
    g = w = None
    if hasattr(be, "live_lists"):
        try:
            g, w = be.live_lists()
        except Exception:
            g = w = None
    return list(g or GESTURES), list(w or CHORD_WORDS)


def build_server(backend=None) -> FastMCP:
    be = backend or MockBackend()
    gestures, words = _lists(be)
    mcp = FastMCP(
        "rocky",
        instructions=(
            "Tool server for Pebble/Rocky, a radial pentapod robot. Rocky "
            "understands speech but replies ONLY in chord-speak (never "
            "words). Motion tools are guarded by onboard reflexes: a "
            "result with stopped='cliff', 'stuck', 'blocked' or 'user' is a "
            "normal, successful veto — report it, don't retry blindly."),
    )

    @mcp.tool(annotations={"readOnlyHint": False, "idempotentHint": True},
              description=("Speak one chord-speak word through Rocky's voice. word: one of "
                           f"{', '.join(words)}. Unknown words are refused with the lexicon in "
                           "the error (Rocky never speaks human words — canon)."))
    async def say(word: str) -> dict:
        return await be.say(word)

    @mcp.tool(annotations={"readOnlyHint": False},
              description=(f"Perform a gesture. Available: {', '.join(gestures)}. direction "
                           f"'left'|'right' applies to {' and '.join(SIGNED)} only (turn_in_place "
                           "turns on the spot for ~5 s; left = counter-clockwise). Refused with "
                           "error='busy' while walking — stop() first. list_gestures has the "
                           "current list."))
    async def gesture(name: str, direction: str = "") -> dict:
        if not direction:
            return await be.gesture(name)
        if _accepts(be.gesture, "direction"):
            return await be.gesture(name, direction=direction)
        if direction == "left":
            return await be.gesture(name)          # the canon turns/steps left
        return {"ok": False, "error": f"this backend cannot {name} {direction}"}

    @mcp.tool(annotations={"readOnlyHint": False}, description=(
        MOVE_DOC + " One motion intent at a time; a newer move or goto preempts it."))
    async def move(forward_m: Meters, left_m: LeftMeters = 0.0) -> dict:
        return await move_via(be, forward_m, left_m)

    @mcp.tool(annotations={"readOnlyHint": False}, description=(
        GOTO_DOC + " One motion intent at a time; calling goto again preempts."))
    async def goto(x: Meters, y: Meters) -> dict:
        return await be.goto(x, y)

    @mcp.tool(annotations={"readOnlyHint": False, "idempotentHint": True})
    async def stop() -> dict:
        """Safe-stop NOW (always accepted): finish the current step, brace
        into a full-contact crouch, hold. Resolves any in-flight goto with
        stopped='user'."""
        return await be.stop()

    @mcp.tool(annotations={"readOnlyHint": True})
    async def scan_summary() -> dict:
        """Summarize the latest lidar scan: point count, open frontiers
        (bearing/range), nearest obstacle."""
        return await be.scan_summary()

    @mcp.tool(annotations={"readOnlyHint": True})
    async def status() -> dict:
        """Robot status: pose, mode (idle|walking|safe_stop), battery
        voltage (mocked until hardware), recent events."""
        return await be.status()

    @mcp.tool(annotations={"readOnlyHint": True})
    async def list_gestures() -> dict:
        """The gestures the robot knows RIGHT NOW (saved keyframe gestures
        included) and which ones take a direction."""
        if hasattr(be, "list_gestures"):
            return await be.list_gestures()
        return {"ok": True, "gestures": list(GESTURES), "signed": list(SIGNED)}

    if hasattr(be, "look"):                      # D049: only where an eye exists
        @mcp.tool(annotations={"readOnlyHint": True})
        async def look() -> dict:
            """Look through the robot's eye camera: a vision model describes
            what is in front of the robot (obstacles, objects, open floor)."""
            return await be.look()

    if hasattr(be, "find_object"):
        @mcp.tool(annotations={"readOnlyHint": False})
        async def find_object(name: str, max_steps: int = 6) -> dict:
            """Find an object by name with the eye and walk up to it ('ball',
            'box'). Runs in the cockpit by itself: look (a vision model boxes
            it; bearing and distance come from the camera geometry), then a
            0.1-0.4 m goto toward it (every guard applies) or a 30 deg scan turn
            when it is unseen or unsure. Ends found (within ~0.25 m) | not
            found | stopped (a goto came back cliff / blocked / stuck: a veto,
            report it). Up to ~1-2 minutes; stop ends it."""
            return await be.find_object(name, max_steps)

    if hasattr(be, "where_is"):                  # the cockpit's scene memory (sim/scene_memory.py)
        try:                                     # one source of truth for the docs (the cockpit's)
            from sim.cockpit_brains import (REMEMBER_DOC, WHERE_IS_DOC, RECALL_DOC, GO_BACK_DOC,
                                            FORGET_DOC)
        except Exception:                        # noqa: BLE001 — a docs import must not kill the server
            REMEMBER_DOC = "Remember a fact the operator states ('the charger is here' pins the robot's pose)."
            WHERE_IS_DOC = "Where a named object was seen or pinned (map meters, age, confidence), from memory."
            RECALL_DOC = "Search the robot's memory of this world for a query."
            GO_BACK_DOC = "Walk back to a remembered object (ordinary gotos, every guard applies)."
            FORGET_DOC = "Forget one object, or everything with name 'all'."

        @mcp.tool(annotations={"readOnlyHint": False}, description=REMEMBER_DOC)
        async def remember(note: str) -> dict:
            return await be.remember(note)

        @mcp.tool(annotations={"readOnlyHint": True}, description=WHERE_IS_DOC)
        async def where_is(name: str) -> dict:
            return await be.where_is(name)

        @mcp.tool(annotations={"readOnlyHint": True}, description=RECALL_DOC)
        async def recall(query: str = "", k: int = 5) -> dict:
            return await be.recall(query, k)

        @mcp.tool(annotations={"readOnlyHint": False}, description=GO_BACK_DOC)
        async def go_back_to(name: str) -> dict:
            return await be.go_back_to(name)

        @mcp.tool(annotations={"readOnlyHint": False, "destructiveHint": True}, description=FORGET_DOC)
        async def forget(name: str) -> dict:
            return await be.forget(name)

    return mcp


def _pick_backend():
    kind = os.environ.get("ROCKY_BACKEND", "mock")
    if kind == "auto":
        from harness.cockpit_backend import AutoBackend, DEFAULT_URL
        be = AutoBackend()
        print(f"[rocky-mcp] ROCKY_BACKEND=auto: the cockpit at {DEFAULT_URL} whenever it "
              "answers, else the in-process sim — checked per call", file=sys.stderr, flush=True)
        return be
    if kind == "cockpit":
        from harness.cockpit_backend import CockpitBackend, cockpit_alive, DEFAULT_URL
        if not cockpit_alive():
            raise SystemExit(f"ROCKY_BACKEND=cockpit but nothing answers at {DEFAULT_URL} "
                             "(start ./rocky.sh cockpit first)")
        be = CockpitBackend()
    elif kind == "sim":
        from harness.sim_backend import SimBackend
        be = SimBackend()
    else:
        be = MockBackend()
    print(f"[rocky-mcp] backend: {type(be).__name__} (ROCKY_BACKEND={kind})",
          file=sys.stderr, flush=True)
    return be


if __name__ == "__main__":
    build_server(_pick_backend()).run()          # stdio transport
