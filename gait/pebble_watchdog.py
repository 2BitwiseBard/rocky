"""Stuck detection + retry-higher-step — D017's terrain lever, implemented.

D017: on rubble above ~30 mm the blind gait SNAGS but never falls — it
stalls with tilt barely moving. So the trigger is PROGRESS, not tilt, and
the cheap fix is a higher step.

ProgressWatchdog
----------------
Feed it (t, odom_xy, commanded_v). It maintains expected vs actual progress
over a sliding window of gait cycles and fires STUCK when the ratio drops
under `ratio_trip` for `patience` consecutive checks.

RetryPolicy
-----------
On STUCK: escalate through recovery stages, each tried for `stage_cycles`
gait cycles, then drop back to nominal if progress resumes:
  nominal       params gait.step_height, cycle time and speed
  high-step     step x1.45, speed x0.70            (step over it)
  higher+clear  step x1.75, body +10 mm, speed x0.55 (high-step + clearance)
  retreat       reverse 0.6 cycles, then higher+clear again (unhook the foot)
Under the D052 servo budget a higher step at the same cycle time asks the
knee for more speed than it has (WaveGait.vf_limit()'s lift ceiling drops to
0), so each stage gets the shortest cycle time (a multiple of the nominal)
whose speed envelope min(vf_limit()) is at least MIN_VF_MM_S — worked out
once from the gait's own budget, never hard-coded. Every command the policy
returns goes through the gait's budget(). A stage switch keeps the gait
phase continuous (a new cycle time would otherwise jump every leg's phase).
Odometry source: sim uses torso ground truth; hardware will use the legged
odometry EKF (perception/legged_odom.py) — same interface.

Not on a live path: the cockpit's goto has its own no-progress rule; the
sim/experiments run_stuck*.py scripts drive this.
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

    # (name, step height x nominal, body lift mm, speed multiplier, reverse cycles)
    LADDER = [
        ("nominal", 1.00, 0.0, 1.00, 0.0),
        ("high-step", 1.45, 0.0, 0.70, 0.0),
        ("higher+clear", 1.75, 10.0, 0.55, 0.0),
        ("retreat", 1.75, 10.0, 0.55, 0.6),
    ]
    T_MULTS = (1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0)   # cycle-time search, x nominal
    MIN_VF_MM_S = 20.0        # a stage must still walk: its envelope's foot-speed ceiling

    def __init__(self, gait, stage_cycles=3.0, settle_cycles=2.0):
        self.g = gait
        self.h0 = gait.h
        self.stage_cycles = stage_cycles
        self.settle_cycles = settle_cycles
        self.stage = 0
        self._stage_t0 = None
        self._reverse_until = None
        self._phase_off0 = np.array(gait.phase_off, dtype=float)
        self._shift = 0.0                 # phase added so a cycle-time change is continuous
        self.events: list[tuple[float, str]] = []
        self.STAGES = self.derive_stages(gait)

    @classmethod
    def derive_stages(cls, gait) -> list[dict]:
        """The ladder on this gait: per stage the step height, body lift and the
        shortest cycle time whose speed envelope is >= MIN_VF_MM_S (vf = that
        envelope, mm/s). ValueError if a stage cannot walk at any T_MULTS."""
        out = []
        for name, k_step, dh, mult, rev in cls.LADDER:
            hstep = gait.hstep * k_step
            for k in cls.T_MULTS:
                T = gait.T * k
                trial = type(gait)(body_height=gait.h + dh, stance_radius=gait.R0,
                                   cycle_time=T, duty=gait.duty, step_height=hstep)
                vf = min(trial.vf_limit())
                if vf >= cls.MIN_VF_MM_S:
                    break
            else:
                raise ValueError(f"retry stage {name!r} (step {hstep:.0f} mm, body +{dh:.0f} mm) "
                                 f"has no speed envelope >= {cls.MIN_VF_MM_S} mm/s at any cycle time "
                                 f"up to {cls.T_MULTS[-1]}x nominal")
            out.append(dict(name=name, hstep=hstep, T=T, dh=dh, speed_mult=mult,
                            reverse=rev, vf=vf))
        return out

    @property
    def stage_name(self):
        return self.STAGES[self.stage]["name"]

    def _apply(self, s, t):
        ph = t / self.g.T + self._shift                  # keep the gait phase continuous at t
        self.g.T = s["T"]
        self._shift = ph - t / self.g.T
        self.g.phase_off = self._phase_off0 + self._shift
        self.g.hstep = s["hstep"]
        self.g.h = self.h0 + s["dh"]
        p = np.array(self.g.p_nom, dtype=float)          # stations come from the gait (robot spec)
        p[:, 2] = -self.g.h
        self.g.p_nom = p

    def on_stuck(self, t):
        if self.stage < len(self.STAGES) - 1:
            self.stage += 1
        self._stage_t0 = t
        s = self.STAGES[self.stage]
        self._apply(s, t)
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
            self._apply(self.STAGES[0], t)
            self._stage_t0 = None
            self.events.append((round(t, 2), "recovered -> nominal"))

    def command(self, t, vx, vy, wz):
        """Transform the operator command per current stage, fitted into the
        current gait's budget() (D052)."""
        s = self.STAGES[self.stage]
        m = s["speed_mult"]
        if self._reverse_until is not None:
            if t < self._reverse_until:
                return self.g.budget(-vx * m * 0.8, -vy * m * 0.8, wz * 0.3)
            self._reverse_until = None
        return self.g.budget(vx * m, vy * m, wz)
