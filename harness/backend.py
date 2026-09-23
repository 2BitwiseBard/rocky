"""Robot backends for the Rocky-MCP harness (docs/MCP_CONTRACT_v0.md).

Two implementations of one contract:

  MockBackend — fast, deterministic, no MuJoCo. Encodes the BEHAVIORAL
    contract (guard supremacy, single motion intent, honest async) so the
    tool-layer tests run in milliseconds. The mock world has a real void
    at x = EDGE_X, and its "guard" refuses to carry the body past the
    last safe stance — the same *observable* behavior the sim produces,
    without physics. The mock is for testing the SERVER; it proves
    nothing about the robot.

  SimBackend (sim_backend.py) — the truth: MuJoCo cliff world + the real
    CliffDetector + ReflexSupervisor safe-stop (session 6 integration,
    sim/run_cliff_safestop.py). Slow (~minutes); the guard-supremacy test
    runs it once, marked slow.

Contract invariants implemented here (tests in test_harness.py):
  1. guard supremacy — a goto into the void returns stopped("cliff"),
     never an exception, and the pose stays on safe ground;
  2. single writer — one motion intent in flight; a second goto preempts
     politely (first resolves stopped("preempted"));
  3. honest async — stop() during goto resolves the goto with
     stopped("user"); stop() itself always succeeds;
  4. no state in the brain — every tool result carries the full outcome.

v0 scope notes (deliberate): goto returns its terminal result only —
streamed progress notifications are v0.1; gestures/words are validated
against the real lexicons but the mock does not render them.
"""
from __future__ import annotations

import asyncio
from dataclasses import dataclass, field

# the real lexicons — shared by every backend (absent tool > lying tool:
# a word or gesture not in these lists is refused, not faked)
CHORD_WORDS = [
    "acknowledge", "alarm_help", "amaze", "confused", "curious_question",
    "determined", "discovery", "error", "found_it", "greeting",
    "low_battery", "no", "sleepy", "startup", "thinking", "yes",
]
GESTURES = ["jazz_hands", "fist_bump", "beckon",   # B18 shipped session 6
            "wave", "bow", "look_around", "shake", "sit",
            "turn_in_place", "sidestep"]                 # v2 library, session 8

EDGE_X = 0.35          # the mock world's void, same as sim/run_cliff.py
SAFE_MARGIN = 0.18     # the detector historically stops ~185 mm short
SPEED = 0.045          # m/s, the gait's V_X


@dataclass
class MockBackend:
    """Kinematic mock with the contract's observable behavior."""
    x: float = 0.0
    y: float = 0.0
    mode: str = "idle"                 # idle | walking | safe_stop
    events: list = field(default_factory=list)
    _stop_evt: asyncio.Event = field(default_factory=asyncio.Event)
    _preempt_evt: asyncio.Event = field(default_factory=asyncio.Event)
    _motion_task: object = None        # the in-flight goto, if any
    battery_v: float = 11.9            # mocked telemetry (sim mocks it too)

    # ---------------------------------------------------------------- say
    async def say(self, word: str) -> dict:
        if word not in CHORD_WORDS:
            return {"ok": False,
                    "error": f"unknown chord word {word!r}",
                    "hint": f"lexicon: {', '.join(CHORD_WORDS)}"}
        self.events.append(("say", word))
        return {"ok": True, "word": word}

    # ------------------------------------------------------------ gesture
    async def gesture(self, name: str) -> dict:
        if name not in GESTURES:
            return {"ok": False, "error": f"unknown gesture {name!r}",
                    "hint": f"available: {', '.join(GESTURES)}"}
        if self.mode == "walking":
            return {"ok": False, "error": "busy",
                    "hint": "robot is walking; stop() first or wait for "
                            "the goto to resolve"}
        self.events.append(("gesture", name))
        return {"ok": True, "gesture": name}

    # --------------------------------------------------------------- goto
    async def goto(self, x: float, y: float) -> dict:
        # single writer: preempt any in-flight motion at a "cycle boundary"
        if self._motion_task is not None and not self._motion_task.done():
            self._preempt_evt.set()
            try:
                await self._motion_task
            except Exception:
                pass
        self._stop_evt.clear()
        self._preempt_evt.clear()
        self._motion_task = asyncio.current_task()
        self.mode = "walking"
        try:
            dt = 0.02
            while True:
                dx, dy = x - self.x, y - self.y
                dist = (dx * dx + dy * dy) ** 0.5
                if dist < 0.01:
                    self.mode = "idle"
                    return {"ok": True, "stopped": "arrived",
                            "pose": self._pose()}
                step = min(SPEED * dt, dist)
                nx = self.x + step * dx / dist
                ny = self.y + step * dy / dist
                # GUARD: the void. The detector fires when a stance foot
                # probes past the edge; behaviorally: we halt with the body
                # short of the edge and enter the safe-stop posture.
                if nx > EDGE_X - SAFE_MARGIN:
                    self.mode = "safe_stop"
                    self.events.append(("void", round(self.x, 3)))
                    return {"ok": False, "stopped": "cliff",
                            "pose": self._pose(),
                            "detail": "VOID detected; PLANT->BRACE halt, "
                                      "holding short of the edge"}
                self.x, self.y = nx, ny
                if self._stop_evt.is_set():
                    self.mode = "safe_stop"
                    return {"ok": False, "stopped": "user",
                            "pose": self._pose()}
                if self._preempt_evt.is_set():
                    self.mode = "idle"
                    return {"ok": False, "stopped": "preempted",
                            "pose": self._pose()}
                await asyncio.sleep(0.001)   # yield: honest-async interleave
        finally:
            if self.mode == "walking":
                self.mode = "idle"

    # --------------------------------------------------------------- stop
    async def stop(self) -> dict:
        self._stop_evt.set()
        if self._motion_task is None or self._motion_task.done():
            self.mode = "safe_stop"
        return {"ok": True, "mode": self.mode}

    # ------------------------------------------------------ scan_summary
    async def scan_summary(self) -> dict:
        # mock lidar: the void reads as open space, walls behind
        return {"ok": True, "n_points": 360,
                "frontiers": [{"bearing_deg": 0.0,
                               "range_m": round(EDGE_X - self.x, 3)}],
                "nearest_obstacle_bearing_deg": 180.0,
                "nearest_obstacle_m": 0.45 + self.x}

    # ------------------------------------------------------------- status
    async def status(self) -> dict:
        return {"ok": True, "pose": self._pose(), "mode": self.mode,
                "battery_v": self.battery_v,
                "last_events": self.events[-5:]}

    def _pose(self):
        return {"x": round(self.x, 3), "y": round(self.y, 3)}
