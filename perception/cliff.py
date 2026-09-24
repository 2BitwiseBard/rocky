"""Software cliff detection — a void is a stance leg that never found ground.

Zero new hardware (D010's microswitch is the sensor). First design used a
depth-overshoot trigger — wrong for stiff position servos: the swing foot
stops AT its commanded z whether or not ground is there; it never gets
driven deeper. The real signature (exactly D017's advice: trigger on
contact-TIMING) is a leg the gait believes is PLANTED whose contact switch
stays open:

    VOID(i) := stance_cmd[i] held for > confirm_s  AND  contact[i] == False
               (evaluated only past the touchdown settling margin)

On rubble this same signal fires on bridged/late feet — there it feeds the
stuck-watchdog; on commanded-flat ground it means the world ends. The
arbitration lives in the caller (SENSING_PLAN.md §reflex-arbitration).

Terrain-aware option (D050): on rough ground or an obstacle course the
same timing signal fires on bridged feet, bumps and blocked swings — none
of them a void. The fix is the one a robot with a foot switch and no
depth sensor has: FEEL for the floor. The caller lowers a contactless
planted foot step by step (sim/playground.py `_probe`, PROBE_MAX mm) and
passes `probed_out[i]` = "this foot ran out of probe and still touched
nothing"; a void then requires it too. Without it the detector behaves
exactly as before (the harness's in-process sim backend, the run_cliff
experiments).

CliffReaction: halt drive, retreat ~1.6 cycles, stop, await orders.
(The harness sim backend and run_cliff use it. The playground's always-on
guard does NOT since D052 P2: walking backwards along the void bearing lifts
the next leg in the forward wave order while the void leg still hangs over the
edge, so it plays the approach backwards instead, and a touchdown gate holds
the gait on a foot that has not felt ground — sim/playground.py GATE_TICKS.)
"""
from __future__ import annotations


class CliffDetector:
    def __init__(self, n_legs=5, confirm_s=0.10, phase_margin=0.12):
        """confirm_s: how long a planted-commanded leg may stay contactless
        before we call it a void (covers touchdown transients + switch
        debounce). phase_margin handled by the caller passing stance flags
        already offset past touchdown."""
        self.n = n_legs
        self.confirm_s = confirm_s
        self._open_since = [None] * n_legs
        self.events: list[tuple[float, int]] = []

    def update(self, t, stance_cmd, contact, probed_out=None) -> list[int]:
        fired = []
        for i in range(self.n):
            suspect = stance_cmd[i] and not contact[i]
            if suspect and probed_out is not None and not probed_out[i]:
                suspect = False                  # still feeling for the floor: not a void (yet)
            if suspect:
                if self._open_since[i] is None:
                    self._open_since[i] = t
                elif t - self._open_since[i] >= self.confirm_s:
                    self._open_since[i] = None
                    self.events.append((round(t, 3), i))
                    fired.append(i)
            else:
                self._open_since[i] = None
        return fired


class CliffReaction:
    """On VOID: kill forward drive, retreat, then hold and wait."""

    def __init__(self, gait, retreat_cycles=1.6):
        self.g = gait
        self.retreat_cycles = retreat_cycles
        self.retreat_until = None
        self.triggered = False

    def on_void(self, t):
        if not self.triggered:
            self.retreat_until = t + self.g.T * self.retreat_cycles
        self.triggered = True

    def command(self, t, vx, vy, wz):
        if self.retreat_until is None:
            return vx, vy, wz
        if t < self.retreat_until:
            return -0.6 * vx, -0.6 * vy, 0.0        # back away, no turning
        return 0.0, 0.0, 0.0                        # stop; ask the operator
