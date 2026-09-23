"""Stuck detection + retry-higher-step — D017's terrain lever, implemented.

D017: on rubble above ~30 mm the blind gait SNAGS but never falls — it
stalls with tilt barely moving. So the trigger is PROGRESS, not tilt, and
the cheap fix is step height (the 32 mm default is the binding constraint).

ProgressWatchdog
----------------
Feed it (t, odom_xy, commanded_v). It maintains expected vs actual progress
over a sliding window of gait cycles and fires STUCK when the ratio drops
under `ratio_trip` for `patience` consecutive checks.

RetryPolicy
-----------
On STUCK: escalate through recovery stages, each tried for `stage_cycles`
gait cycles, then drop back to nominal if progress resumes:
  stage 1: step height 32 -> 46 mm, speed x0.7      (step over it)
  stage 2: step height 52 mm, speed x0.55, body +10 (high-step + clearance)
  stage 3: retreat: reverse 0.6 cycles, then stage 2 again (unhook the foot)
Odometry source: sim uses torso ground truth; hardware will use the legged
odometry EKF (perception/legged_odom.py) — same interface.
"""
from __future__ import annotations
import numpy as np


class ProgressWatchdog:
    def __init__(self, cycle_time: float, window_cycles=1.5, ratio_trip=0.40,
                 patience=2, check_every=0.5):
        self.T = cycle_time
        self.window = window_cycles * cycle_time
        self.ratio_trip = ratio_trip
        self.patience = patience
        self.check_every = check_every
        self._hist: list[tuple[float, np.ndarray]] = []   # (t, xy)
        self._cmd_hist: list[tuple[float, float]] = []    # (t, |v_cmd|)
        self._next_check = None
        self._strikes = 0
        self.last_ratio = 1.0

    def reset(self):
        """After a STUCK event: clear history AND report pessimistic ratio
        until a full fresh window exists (a stale optimistic ratio here made
        the retry policy relax back to nominal while still wedged)."""
        self._hist.clear()
        self._cmd_hist.clear()
        self._next_check = None
        self._strikes = 0
        self.last_ratio = 0.0

    def update(self, t: float, odom_xy, v_cmd_mm_s: float) -> bool:
        """Returns True exactly when a STUCK event fires."""
        xy = np.asarray(odom_xy, dtype=float)
        self._hist.append((t, xy))
        self._cmd_hist.append((t, float(v_cmd_mm_s)))
        t0 = t - self.window
        while self._hist and self._hist[0][0] < t0:
            self._hist.pop(0)
        while self._cmd_hist and self._cmd_hist[0][0] < t0:
            self._cmd_hist.pop(0)
        if self._next_check is None:
            self._next_check = t + self.window          # let the window fill
        if t < self._next_check or len(self._hist) < 2:
            return False
        self._next_check = t + self.check_every
        span = self._hist[-1][0] - self._hist[0][0]
        actual = float(np.linalg.norm(self._hist[-1][1] - self._hist[0][1]))
        expected = float(np.mean([v for _, v in self._cmd_hist]) * span)
        if expected < 5.0:                              # not commanded to move
            self._strikes = 0
            return False
        self.last_ratio = actual / expected
        if self.last_ratio < self.ratio_trip:
            self._strikes += 1
            if self._strikes >= self.patience:
                self._strikes = 0
                return True
        else:
            self._strikes = 0
        return False


class RetryPolicy:
    """Escalating recovery stages applied to a WaveGait in place."""

    STAGES = [
        dict(name="nominal", hstep=32.0, speed_mult=1.00, dh=0.0, reverse=0.0),
        dict(name="high-step", hstep=46.0, speed_mult=0.70, dh=0.0, reverse=0.0),
        dict(name="higher+clear", hstep=52.0, speed_mult=0.55, dh=10.0, reverse=0.0),
        dict(name="retreat", hstep=52.0, speed_mult=0.55, dh=10.0, reverse=0.6),
    ]

    def __init__(self, gait, stage_cycles=3.0, settle_cycles=2.0):
        self.g = gait
        self.h0 = gait.h
        self.stage_cycles = stage_cycles
        self.settle_cycles = settle_cycles
        self.stage = 0
        self._stage_t0 = None
        self._reverse_until = None
        self.events: list[tuple[float, str]] = []

    @property
    def stage_name(self):
        return self.STAGES[self.stage]["name"]

    def _apply(self, s):
        self.g.hstep = s["hstep"]
        self.g.h = self.h0 + s["dh"]
        a = np.deg2rad(90 + 72 * np.arange(5))
        self.g.p_nom = np.stack([self.g.R0 * np.cos(a), self.g.R0 * np.sin(a),
                                 -self.g.h * np.ones(5)], axis=1)

    def on_stuck(self, t):
        if self.stage < len(self.STAGES) - 1:
            self.stage += 1
        self._stage_t0 = t
        s = self.STAGES[self.stage]
        self._apply(s)
        if s["reverse"] > 0:
            self._reverse_until = t + s["reverse"] * self.g.T
        self.events.append((round(t, 2), f"STUCK -> {s['name']}"))

    def maybe_relax(self, t, making_progress: bool):
        """Call each check: after stage_cycles of progress, step back down."""
        if self.stage == 0 or self._stage_t0 is None:
            return
        if making_progress and (t - self._stage_t0) > \
                self.stage_cycles * self.g.T:
            self.stage = 0
            self._apply(self.STAGES[0])
            self._stage_t0 = None
            self.events.append((round(t, 2), "recovered -> nominal"))

    def command(self, t, vx, vy, wz):
        """Transform the operator command per current stage."""
        s = self.STAGES[self.stage]
        m = s["speed_mult"]
        if self._reverse_until is not None:
            if t < self._reverse_until:
                return -vx * m * 0.8, -vy * m * 0.8, wz * 0.3
            self._reverse_until = None
        return vx * m, vy * m, wz
