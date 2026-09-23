"""Shove model for the sim (D048, 2026-09-23).

Until this file every push in the project was a RECTANGULAR force pulse
applied at the torso's centre of mass: `run_push*.py` use 0.15 s, the
fallen demo used 120 N x 0.25 s (30 N·s = 4.6 bodyweight-seconds, which
launches a 2.67 kg robot at ~10 m/s and 11-15 m across the floor), and the
playground's `push` was the same model. That is a strike, not a shove,
and it also has no gentle regime: below the foot-friction limit (~31 N
standing) the robot is a rigid block, above it the feet let go and it
cartwheels.

A shove from a hand is (a) smooth — it ramps up and off, (b) short but
not instantaneous, (c) applied HIGH on the body, which is what tips a
squat robot: the torque about the feet does the work, not the linear
impulse. This module models exactly that: a half-sine force profile
(default 0.4 s) applied at the shell's top rim, `LEVER_Z` above the torso
frame, which MuJoCo receives as the force at the centre of mass plus the
torque the offset makes (`xfrc_applied` semantics). Reported in N·s and
bodyweights so the number means the same thing after bench day changes
the masses (D039).

The push-envelope experiment scripts keep their rectangular CoM pulse:
those JSONs are decision records (D017/D022/D025) and are compared in
bodyweights; `shove_envelope.py` is the new-model counterpart.
"""
from __future__ import annotations
import numpy as np

LEVER_Z = 0.060        # m above the torso frame origin: the carapace's top rim
G = 9.81


def profile(t_rel: float, dur: float, kind: str = "halfsine") -> float:
    """Force scale in [0, 1] at t_rel seconds into a shove of length dur."""
    if t_rel < 0.0 or t_rel >= dur:
        return 0.0
    if kind == "halfsine":
        return float(np.sin(np.pi * t_rel / dur))
    return 1.0                                   # "rect"


def impulse_ns(peak_n: float, dur: float, kind: str = "halfsine") -> float:
    """Linear impulse of the profile (N·s)."""
    return peak_n * dur * (2.0 / np.pi if kind == "halfsine" else 1.0)


def total_mass(model) -> float:
    return float(model.body_subtreemass[0]) if model.nbody else 0.0


def bodyweights(force_n: float, model) -> float:
    return float(force_n) / (total_mass(model) * G)


class Shove:
    """A timed shove: world-frame peak force (fx, fy, fz) N, profile `kind`
    over `dur` s starting at t0, applied at the point LEVER_Z above the
    torso origin (body z axis). `apply()` writes xfrc_applied for the torso
    and returns False once the shove is over (the wrench is zeroed then).
    """

    def __init__(self, fx, fy, fz=0.0, dur=0.4, t0=0.0, lever_z=LEVER_Z,
                 kind="halfsine"):
        self.f = np.array([fx, fy, fz], dtype=float)
        self.dur, self.t0, self.lever_z, self.kind = float(dur), float(t0), float(lever_z), kind
        self.peak_n = float(np.linalg.norm(self.f))

    def active(self, t) -> bool:
        return self.t0 <= t < self.t0 + self.dur

    def wrench(self, data, torso, t):
        """(force, torque) in world frame at time t, both zero when inactive."""
        s = profile(t - self.t0, self.dur, self.kind)
        if s == 0.0:
            return np.zeros(3), np.zeros(3)
        F = self.f * s
        R = data.xmat[torso].reshape(3, 3)
        p_contact = data.xpos[torso] + R @ np.array([0.0, 0.0, self.lever_z])
        r = p_contact - data.xipos[torso]         # lever from the CoM
        return F, np.cross(r, F)

    def apply(self, model, data, torso, t) -> bool:
        F, tau = self.wrench(data, torso, t)
        data.xfrc_applied[torso, :3] = F
        data.xfrc_applied[torso, 3:] = tau
        return self.active(t)

    def impulse(self) -> float:
        return impulse_ns(self.peak_n, self.dur, self.kind)

    def describe(self, model) -> str:
        return (f"{self.peak_n:.0f} N peak {self.kind} over {self.dur:.2f} s at the shell rim: "
                f"{self.impulse():.1f} N·s, {bodyweights(self.peak_n, model):.2f} BW peak")
