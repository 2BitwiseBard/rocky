"""Rocky-MCP tool server v0 (docs/MCP_CONTRACT_v0.md) — FastMCP over stdio.

The model proposes, the reflex layer disposes: every tool result is an
ordinary value (vetoes included — a goto stopped by the cliff reflex
returns {"stopped": "cliff"}, it does not raise). The backend owns all
robot state; kill the brain any time and the robot is safe.

Run against the mock (default) or the MuJoCo sim:

    python3 -m harness.server                 # mock backend
    ROCKY_BACKEND=sim python3 -m harness.server   # MuJoCo cliff world

Wire into a client (e.g. Claude Code .mcp.json):
    {"rocky": {"command": "python3", "args": ["-m", "harness.server"],
               "cwd": "<repo>/rocky"}}

v0 tool set: say, gesture, goto, stop, scan_summary, status. Deferred
(v1, do not stub): look, map_query, patrol, dock — absent tool > lying
tool.
"""
from __future__ import annotations

import os

from mcp.server.fastmcp import FastMCP

from harness.backend import MockBackend, CHORD_WORDS, GESTURES


def build_server(backend=None) -> FastMCP:
    be = backend or MockBackend()
    mcp = FastMCP(
        "rocky",
        instructions=(
            "Tool server for Pebble/Rocky, a radial pentapod robot. Rocky "
            "understands speech but replies ONLY in chord-speak (never "
            "words). Motion tools are guarded by onboard reflexes: a "
            "result with stopped='cliff' or stopped='user' is a normal, "
            "successful veto — report it, don't retry blindly."),
    )

    @mcp.tool(annotations={"readOnlyHint": False, "idempotentHint": True})
    async def say(word: str) -> dict:
        """Speak one chord-speak word through Rocky's voice.

        word: one of the 16-word Eridian lexicon, e.g. 'acknowledge',
        'found_it', 'curious_question'. Unknown words are refused with the
        lexicon in the error (Rocky never speaks human words — canon)."""
        return await be.say(word)

    @mcp.tool(annotations={"readOnlyHint": False})
    async def gesture(name: str) -> dict:
        """Perform a gesture. Available: 'jazz_hands', 'fist_bump', 'beckon',
        'wave', 'bow', 'look_around', 'shake', 'sit', 'turn_in_place', 'sidestep'.
        Refused with error='busy' while walking — stop() first."""
        return await be.gesture(name)

    @mcp.tool(annotations={"readOnlyHint": False})
    async def goto(x: float, y: float) -> dict:
        """Walk to (x, y) in the map frame (meters). Returns the terminal
        outcome: stopped='arrived' | 'cliff' (void reflex veto — the robot
        halted safely short of an edge) | 'user' (stop() was called) |
        'preempted' (a newer goto took over). A veto is a NORMAL result.
        One motion intent at a time; calling goto again preempts."""
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

    return mcp


def _pick_backend():
    if os.environ.get("ROCKY_BACKEND", "mock") == "sim":
        from harness.sim_backend import SimBackend
        return SimBackend()
    return MockBackend()


if __name__ == "__main__":
    build_server(_pick_backend()).run()          # stdio transport
