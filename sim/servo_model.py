"""Servo realism layer (D051) — what sits between a joint target and the
position actuator on the real robot, and was missing from the sim.

The MJCF actuators are ideal position servos updated every physics step
(2 ms). An ST3215 on a Feetech bus is not: the controller sends a new
goal every bus tick (50 Hz sync write), the goal arrives a tick or so
late, the servo slews toward it at a bounded speed (no-load ~0.22 s/60°
at 12 V → ~4.7 rad/s, less under load) and reports positions in 4096
counts per turn. This module applies exactly those four things to the
target stream, so a gait that only works because the sim servo teleports
shows up here first, not on the bench:

    hold_hz     zero-order hold — targets change only at the bus rate
    latency_s   delay before a new goal takes effect (bus + servo loop)
    rate_rad_s  slew limit on the goal the actuator sees
    quant       goal quantised to counts (360/4096 deg for STS)

`ServoModel.filter(target, dt)` is called once per physics step with the
15 joint targets; it returns the targets the actuator should track. All
parameters live-tunable (the cockpit's Realism panel); `off` bypasses.

D052: the defaults are the servo identity from params.yaml (via
gait/rocky_model.py), not copies. And the slew now slows under load when
the caller hands in the previous step's actuator forces:

    rate = rate_rad_s * max(0.2, 1 - |tau| / stall)

— the DC motor's torque-speed line again (the MJCF joint damping is the
same line on the physics side). `filter(target, dt)` without forces is the
old behaviour exactly, so existing callers are unchanged.

D052 V2 (review): that derate is now OFF by default (`load_derate`). It
counted the torque-speed line twice: the D052 MJCF already carries it
(joint damping = stall / no-load, forcerange), and a position actuator's
|tau| saturates as soon as the tracking error passes ~5.5 deg, so every lag
slowed the goal further — the sim followed ~25 % slower than the servo
(review measurement, audit_gestures streams: wave peak 2.54 vs 3.25 rad/s,
p95 lag 25.2 vs 18.0 deg). A real ST3215's goal register is not slowed by
load; only its motor is, which the damping models. Callers still pass their
forces (the API is unchanged); `set(load_derate=True)` brings the old
behaviour back for an A/B.
"""
from __future__ import annotations
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gait"))
import rocky_model as _rm                                              # noqa: E402

STS_COUNTS_PER_RAD = _rm.counts_per_rad()          # 4096 / 2 pi
ST3215_NO_LOAD_RAD_S = _rm.no_load_rad_s()         # 0.222 s / 60 deg at 12 V (Feetech datasheet)
LOAD_SLEW_FLOOR = 0.2                               # even at stall the goal still creeps (never freeze)

DEFAULTS = dict(on=False, hold_hz=_rm.bus_hz(), latency_s=_rm.latency_s(),
                rate_rad_s=ST3215_NO_LOAD_RAD_S, quant=True, stall_nm=_rm.stall_nm(),
                load_derate=False)     # D052 V2: one torque-speed line (the MJCF's), see above


class ServoModel:
    def __init__(self, n=15, **kw):
        self.n = n
        self.p = dict(DEFAULTS)
        self.p.update({k: v for k, v in kw.items() if k in DEFAULTS})
        self.reset()

    def reset(self, q0=None):
        q0 = np.zeros(self.n) if q0 is None else np.asarray(q0, float).copy()
        self.held = q0.copy()          # last goal latched by the bus tick
        self.goal = q0.copy()          # goal the servo currently slews toward (after latency)
        self.pos = q0.copy()           # the slewed target the actuator sees
        self.t = 0.0
        self.t_hold = 0.0
        self.queue = []                # (t_effective, goal) latency pipeline

    def set(self, **kw):
        for k, v in kw.items():
            if k in DEFAULTS:
                self.p[k] = type(DEFAULTS[k])(v)
        return dict(self.p)

    def filter(self, target, dt, force=None):
        """target: (n,) rad the controller wants NOW; returns what the actuator sees.
        force: optional (n,) actuator forces from the PREVIOUS step (N.m) —
        used only with load_derate on (off by default since D052 V2)."""
        target = np.asarray(target, float)
        if not self.p["on"]:
            self.pos = target.copy()
            self.held = target.copy()
            self.goal = target.copy()
            self.queue.clear()
            return target
        self.t += dt
        # 1. zero-order hold at the bus rate
        if self.t >= self.t_hold:
            self.t_hold = self.t + 1.0 / max(self.p["hold_hz"], 1.0)
            g = target.copy()
            if self.p["quant"]:
                g = np.round(g * STS_COUNTS_PER_RAD) / STS_COUNTS_PER_RAD
            self.held = g
            # 2. latency: the goal becomes effective later
            self.queue.append((self.t + self.p["latency_s"], g))
        while self.queue and self.queue[0][0] <= self.t:
            self.goal = self.queue.pop(0)[1]
        # 3. slew limit toward the effective goal (load-derated when forces are given)
        rate = self.p["rate_rad_s"]
        if force is not None and self.p["load_derate"]:
            load = np.abs(np.asarray(force, float)) / max(self.p["stall_nm"], 1e-9)
            rate = rate * np.maximum(LOAD_SLEW_FLOOR, 1.0 - load)
        step = rate * dt
        d = self.goal - self.pos
        self.pos = self.pos + np.clip(d, -step, step)
        return self.pos.copy()

    def describe(self):
        p = self.p
        if not p["on"]:
            return "servo model: off (ideal position actuators)"
        return (f"servo model: hold {p['hold_hz']:.0f} Hz, latency {p['latency_s'] * 1000:.0f} ms, "
                f"slew {p['rate_rad_s']:.1f} rad/s{' load-derated' if p['load_derate'] else ''}, "
                f"{'4096-count' if p['quant'] else 'no'} quantisation")
