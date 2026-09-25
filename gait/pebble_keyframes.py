"""Keyframe gestures (D051) — gestures as DATA, not code.

The canon gestures (pebble_gestures*.py) are hand-written functions of
time. This module plays a gesture written as a list of keyframes, so a
new one can be composed in the cockpit's Gesture studio (pose the body,
lift a leg, snap a frame, scrub, save) and then runs everywhere the code
ones run: the playground, the cockpit, the MCP `gesture` tool and, later,
the servo bus on the real robot — the output is the same (q[5,3] rad,
claw[5] rad) stream.

File: gait/gestures/NAME.json
    {"name": "peek", "loop": false, "end": "stand",
     "keyframes": [
       {"t": 0.0},
       {"t": 0.8, "body": [10, 0, -15], "yaw": 12, "dz": [0, 0, 8, 8, 0],
        "arm": {"0": [0, 70, -50]}, "claw": [1, 0, 0, 0, 0], "say": "greeting",
        "ease": "smooth"},
       {"t": 2.0, "reach": {"2": [-120, -260, 40]}}]}

Per keyframe (all optional, missing = nominal):
    t            seconds from the start (frames sorted by t; the last t = duration)
    body         [dx, dy, dz] mm body displacement (z < 0 crouches)
    yaw          deg body yaw, feet fixed in the world
    dz           [5] mm extra crouch per corner (positive = that corner lower)
    arm          {leg: [yaw, hip, knee] deg} joint overrides — a raised leg/arm
    reach        {leg: [x, y, z] mm} D052: put that leg's hand (the IK foot
                 point, foot_tip) HERE, in the frame's (posed) BODY frame:
                 origin at the deck-plane centre, +z up, leg 0 = +y. Solved
                 with leg_ik(body_to_leg(leg, p)). Unreachable or past a
                 joint limit -> the spec FAILS check with a REACH line that
                 says by how much (playback uses the nearest reachable point:
                 never a silent NaN).
    reach_world  {leg: [x, y, z] mm} the same, in the GROUND frame = the body
                 frame of the unposed stance (floor at z = -body_height): the
                 hand stays put in the world while this frame's body/yaw move.
    claw         [5] 0..1 open fraction per hand
    say          chord-speak cue fired when the frame is reached
    ease         how to get INTO this frame: smooth (default) | linear | hold
Spec level:
    loop         true: plays forever; the last frame must equal the first
                 (LOOP_WRAP fails otherwise — the wrap would be a step)
    end          "stand" (default): a nominal tail frame is appended when the
                 last frame is not the planted stance, timed so no joint
                 exceeds 90 % of its speed budget. "hold": stay in the last
                 pose (the player's live blend owns the way back).

D052 — "the studio cannot author what the robot cannot do":
  * `hold` was a jump at the frame time (216 rad/s at the bus). Now it is
    "arrive early, then hold": leave the previous frame at once, move with
    a smoothstep sized so every joint stays under 90 % of its speed class
    (loaded 3.0 for planted legs, free 4.0 for arm legs, claw 8), arrive,
    hold until the frame time. If the gap is too short it simply takes the
    whole gap — and check() says too fast.
  * planted legs are interpolated in POSE space (body/yaw/dz) and solved by
    the planted IK at every instant, so the feet really stay planted; arm /
    reach legs blend in joint space (always inside the joint box).
  * KeyframeGesture.report() / pebble_feasibility.check_spec() judge a spec;
    save_keyframe_gesture() refuses an infeasible one (ValueError carrying
    the report lines) unless force=True.

Pure Python, no sim.
"""
from __future__ import annotations
import json
import math
import os

import numpy as np
from pebble_gait import (N_LEGS, leg_ik, body_to_leg, L1, L2, L3, Z_HIP, WaveGait, ik_ok)
from pebble_gestures2 import posed, CLAW_MAX, _rotz
import rocky_model as _rm

HERE = os.path.dirname(os.path.abspath(__file__))
# ROCKY_GESTURE_DIR points a process at another library (sim/brain_bench.py gives its own
# cockpit a scratch copy, so a model's compose_gesture never lands in the repo's gait/gestures).
GESTURE_DIR = os.environ.get("ROCKY_GESTURE_DIR") or os.path.join(HERE, "gestures")
KEYS = ("body", "yaw", "dz", "arm", "reach", "reach_world", "claw", "say", "ease", "t")
EASES = ("smooth", "linear", "hold")
ENDS = ("stand", "hold")
BUDGET_FRAC = 0.9                 # tail / hold moves use 90 % of the speed class
MIN_TAIL_S = 0.6
_REACH_EPS = 0.5                  # mm inside the reach annulus when clamping


def _ease(a, kind):
    a = float(np.clip(a, 0.0, 1.0))
    if kind == "linear":
        return a
    return a * a * (3 - 2 * a)                       # smooth (and the hold move)


def _vec(d, key, n, default):
    v = d.get(key, default)
    arr = np.asarray(v, float)
    if arr.shape != (n,):
        raise ValueError(f"keyframe '{key}' must have {n} numbers, got {v!r}")
    if not np.isfinite(arr).all():
        raise ValueError(f"keyframe '{key}' has a non-finite number: {v!r}")
    return arr


def _legmap(d, key):
    out = {}
    for k, v in (d.get(key) or {}).items():
        i = int(k)
        if not 0 <= i < N_LEGS:
            raise ValueError(f"keyframe '{key}': leg {k} is not 0..4")
        arr = np.asarray(v, float)
        if arr.shape != (3,) or not np.isfinite(arr).all():
            raise ValueError(f"keyframe '{key}' leg {k}: need 3 finite numbers, got {v!r}")
        out[i] = arr
    return out


def solve_reach(leg, p_body):
    """(q (3,), error str or None): the leg's joint angles putting its IK foot
    point at p_body (mm, BODY frame). Unreachable -> the nearest reachable
    point on the same ray from the femur pivot, plus a message saying how far
    off it is; past a joint limit -> the IK answer plus a message naming the
    joint. Never NaN."""
    p = body_to_leg(leg, np.asarray(p_body, float))
    q = leg_ik(p)
    err = None
    if not np.isfinite(q).all():
        r = np.hypot(p[0], p[1])
        d = np.array([r - L1, p[2] - Z_HIP])
        dist = float(np.linalg.norm(d))
        lo, hi = abs(L2 - L3) + _REACH_EPS, L2 + L3 - _REACH_EPS
        want = min(max(dist, lo), hi)
        if dist < 1e-9:
            d, dist = np.array([1.0, 0.0]), 1.0
        d2 = d / dist * want
        rr = L1 + d2[0]
        phi = math.atan2(p[1], p[0])
        q = leg_ik(np.array([rr * math.cos(phi), rr * math.sin(phi), Z_HIP + d2[1]]))
        if dist > hi:
            err = (f"leg {leg} reach {np.round(p_body, 1).tolist()} is {dist - hi:.0f} mm beyond the "
                   f"arm (max {L2 + L3:.0f} mm from the hip pivot)")
        else:
            err = (f"leg {leg} reach {np.round(p_body, 1).tolist()} is {lo - dist:.0f} mm inside the "
                   f"arm's dead zone (min {abs(L2 - L3):.0f} mm from the hip pivot)")
    elif not ik_ok(q, guard_deg=2.0):
        lo, hi = _rm.joint_limits_rad()
        bad = []
        for j, name in enumerate(_rm.LEG_JOINTS):
            if q[j] < lo[j] + math.radians(2) or q[j] > hi[j] - math.radians(2):
                bad.append(f"{name} {math.degrees(q[j]):.0f} deg (limit "
                           f"{math.degrees(lo[j]):.0f}..{math.degrees(hi[j]):.0f}, 2 deg guard)")
        err = f"leg {leg} reach {np.round(p_body, 1).tolist()} needs " + ", ".join(bad)
    return q, err


class Frame:
    def __init__(self, d: dict):
        if not isinstance(d, dict):
            raise ValueError(f"a keyframe must be an object, got {d!r}")
        self.raw = {k: v for k, v in d.items() if k in KEYS}
        self.t = float(d.get("t", 0.0))
        if not math.isfinite(self.t) or self.t < 0:
            raise ValueError(f"keyframe t must be a finite time >= 0, got {d.get('t')!r}")
        self.body = _vec(d, "body", 3, [0, 0, 0])
        self.yaw = float(d.get("yaw", 0.0))
        self.dz = _vec(d, "dz", N_LEGS, [0] * N_LEGS)
        self.arm = {i: np.deg2rad(v) for i, v in _legmap(d, "arm").items()}
        self.reach = _legmap(d, "reach")
        self.reach_world = _legmap(d, "reach_world")
        both = (set(self.arm) & (set(self.reach) | set(self.reach_world))) | \
            (set(self.reach) & set(self.reach_world))
        if both:
            raise ValueError(f"keyframe t={self.t:g}: leg(s) {sorted(both)} have more than one of "
                             "arm / reach / reach_world")
        self.claw = _vec(d, "claw", N_LEGS, [0] * N_LEGS)
        self.say = d.get("say")
        self.ease = d.get("ease", "smooth")
        if self.ease not in EASES:
            raise ValueError(f"keyframe ease must be one of {EASES}, got {self.ease!r}")
        # solve the reach targets once (body pose is fixed per frame)
        self.errors = []
        self.override = dict(self.arm)
        for i, p in self.reach.items():
            self.override[i], err = solve_reach(i, p)
            if err:
                self.errors.append(f"t={self.t:g}: {err}")
        for i, pw in self.reach_world.items():
            pb = _rotz(pw, -np.deg2rad(self.yaw)) - self.body       # the posed() convention
            self.override[i], err = solve_reach(i, pb)
            if err:
                self.errors.append(f"t={self.t:g} (reach_world): {err}")

    @property
    def nominal(self) -> bool:
        return (not self.override and not np.any(self.body) and self.yaw == 0.0
                and not np.any(self.dz) and not np.any(self.claw))

    def planted(self, g):
        """(5,3) the planted IK for this frame's body pose (overrides NOT applied)."""
        return posed(g, offset=self.body, yaw=np.deg2rad(self.yaw), dz_leg=self.dz)[0]

    def pose(self, g):
        """(q[5,3], claw_rad[5]) for this frame's body pose; arms applied."""
        q = self.planted(g)
        for i, qi in self.override.items():
            q[i] = qi
        return q, np.clip(self.claw, 0, 1) * CLAW_MAX


def _max_move_time(fa, fb, g):
    """Shortest smoothstep duration (s) from frame a to frame b that keeps every
    joint under BUDGET_FRAC of its class: loaded for planted legs, free for a
    leg overridden in either frame, 8 rad/s for the claws. Peak = 1.5 x mean."""
    qa, ca = fa.pose(g)
    qb, cb = fb.pose(g)
    dq = np.abs(qb - qa)
    free = np.zeros(N_LEGS, bool)
    free[list(set(fa.override) | set(fb.override))] = True
    v = np.where(free, _rm.servo_speed("free"), _rm.servo_speed("loaded"))[:, None]
    t_leg = float(np.nanmax(1.5 * dq / (BUDGET_FRAC * v))) if np.isfinite(dq).all() else 0.0
    t_claw = float(np.max(1.5 * np.abs(cb - ca) / (BUDGET_FRAC * 8.0)))
    return max(t_leg, t_claw, 0.05)


def with_tail(spec: dict, g=None) -> dict:
    """The spec with a nominal tail frame appended when it ends away from the
    planted stance (unless loop or end == "hold"). The tail's length keeps
    every joint under 90 % of its budget, at least 0.6 s, rounded up to 0.1 s."""
    spec = dict(spec)
    frames = [dict(f) for f in spec.get("keyframes", [])]
    spec["keyframes"] = frames
    if spec.get("loop") or spec.get("end", "stand") == "hold" or not frames:
        return spec
    fr = sorted((Frame(f) for f in frames), key=lambda f: f.t)
    last = fr[-1]
    if last.nominal:
        return spec
    g = g or WaveGait()
    nom = Frame({"t": last.t})
    dur = max(MIN_TAIL_S, _max_move_time(last, nom, g))
    dur = math.ceil(dur * 10 - 1e-9) / 10
    frames.append({"t": round(last.t + dur, 3), "ease": "smooth"})
    return spec


class KeyframeGesture:
    def __init__(self, spec: dict, tail: bool = True):
        if not isinstance(spec, dict):
            raise ValueError("a gesture spec must be an object")
        self.name = spec.get("name", "untitled")
        self.loop = bool(spec.get("loop", False))
        self.end = spec.get("end", "stand")
        if self.end not in ENDS:
            raise ValueError(f"spec 'end' must be one of {ENDS}, got {self.end!r}")
        kf = spec.get("keyframes", [])
        if not isinstance(kf, list):
            raise ValueError("'keyframes' must be a list")
        authored = [float(f.get("t", 0.0)) for f in kf if isinstance(f, dict)]
        self.authored_total = max(authored + [0.05])   # the last AUTHORED frame (preview it, not the tail)
        if tail:
            spec = with_tail(spec)
        frames = sorted((Frame(f) for f in spec.get("keyframes", [])), key=lambda f: f.t)
        if not frames:
            frames = [Frame({"t": 0.0})]
        if frames[0].t > 0:
            frames.insert(0, Frame({"t": 0.0}))     # start from the nominal stance
        self.frames = frames
        self.total = max(frames[-1].t, 0.05)
        self.cues = [(f.t, f.say) for f in frames if f.say]
        self.errors = [e for f in frames for e in f.errors]
        self.spec = spec
        self._seg = {}                               # (k, g key) -> hold move duration

    def _hold_dur(self, k, g):
        key = (k, g.h, g.R0)
        if key not in self._seg:
            a, b = self.frames[k], self.frames[k + 1]
            self._seg[key] = min(b.t - a.t, _max_move_time(a, b, g))
        return self._seg[key]

    def __call__(self, g, t):
        """(q, claw) at time t — the same contract as pebble_gestures*."""
        if self.loop and self.total > 0:
            t = t % self.total
        fr = self.frames
        if t >= fr[-1].t:
            return fr[-1].pose(g)
        if t <= fr[0].t:
            return fr[0].pose(g)
        k = 0
        while k + 1 < len(fr) and fr[k + 1].t <= t:
            k += 1
        a, b = fr[k], fr[k + 1]
        span = max(b.t - a.t, 1e-6)
        if b.ease == "hold":                          # arrive early, then hold (D052)
            u = _ease((t - a.t) / max(self._hold_dur(k, g), 1e-6), "smooth")
        else:
            u = _ease((t - a.t) / span, b.ease)
        # planted legs: interpolate the BODY POSE, then planted IK (feet stay put)
        q = posed(g, offset=(1 - u) * a.body + u * b.body,
                  yaw=np.deg2rad((1 - u) * a.yaw + u * b.yaw),
                  dz_leg=(1 - u) * a.dz + u * b.dz)[0]
        legs = set(a.override) | set(b.override)
        if legs:
            pa, pb = a.planted(g), b.planted(g)
            for i in legs:
                qa = a.override.get(i, pa[i])
                qb = b.override.get(i, pb[i])
                q[i] = (1 - u) * qa + u * qb            # arms: joint space (stays in the box)
        claw = ((1 - u) * np.clip(a.claw, 0, 1) + u * np.clip(b.claw, 0, 1)) * CLAW_MAX
        return q, claw

    def as_entry(self):
        """GESTURES2-style tuple: (fn, total, [(t, chord word)])."""
        return (self, self.total, list(self.cues))

    def report(self, g=None, fs: float = 100.0, model=None):
        """pebble_feasibility.Report for this gesture as it will play (tail
        included), plus REACH and LOOP_WRAP. The studio shows .lines."""
        import pebble_feasibility as pf
        g = g or WaveGait()
        q0 = self.frames[0].pose(g)[0]
        if self.loop:
            q_to, claw_to = q0, self.frames[0].pose(g)[1]
        elif self.end == "hold":
            q_to, claw_to = False, None
        else:
            q_to, claw_to = None, np.zeros(N_LEGS)
        rep = pf.check(self, self.total, g=g, fs=fs, q_to=q_to, name=self.name, model=model,
                       claw_from=np.zeros(N_LEGS), claw_to=claw_to)
        for e in self.errors:
            pf._add(rep, "REACH", 1, msg=e)
        if self.loop:
            ql, cl = self.frames[-1].pose(g)
            qf, cf = self.frames[0].pose(g)
            d = float(np.nanmax(np.abs(ql - qf)))
            dc = float(np.max(np.abs(cl - cf)))
            if math.degrees(max(d, dc)) > pf.JUMP_DEG:
                pf._add(rep, "LOOP_WRAP", 1,
                        msg=f"loop: the last frame is {math.degrees(max(d, dc)):.1f} deg away from "
                            f"the first — the wrap would be a step; make them equal")
        if self.frames[0].t == 0 and not self.frames[0].nominal and not self.loop:
            rep.notes.append("frame 0 is not the planted stance: the gesture starts with a step "
                             "(add a nominal frame at t=0, or start the first pose at t>0)")
        rep.ok = not rep.fails
        rep.lines = pf._lines(rep, pf.speed_limits())
        return rep


def load_keyframe_gestures(dirpath: str = GESTURE_DIR) -> dict[str, KeyframeGesture]:
    out = {}
    if not os.path.isdir(dirpath):
        return out
    for fn in sorted(os.listdir(dirpath)):
        if fn.endswith(".json"):
            try:
                with open(os.path.join(dirpath, fn)) as f:
                    spec = json.load(f)
                spec.setdefault("name", fn[:-5])
                out[spec["name"]] = KeyframeGesture(spec)
            except (ValueError, KeyError, TypeError) as e:   # a half-written file must not kill the loader
                print(f"keyframes: skipped {fn}: {e}")
    return out


def _safe_name(name: str) -> str:
    name = "".join(c if c.isalnum() or c in "-_" else "_" for c in name.strip())[:40]
    if not name:
        raise ValueError("empty gesture name")
    return name


def normalize_spec(spec: dict, g=None) -> dict:
    """What save writes: known keys only, sorted frames, the tail appended."""
    name = _safe_name(str(spec.get("name", "")))
    out = {k: spec[k] for k in ("loop", "end", "taught") if k in spec}
    out["name"] = name
    frames = [{k: v for k, v in f.items() if k in KEYS} for f in spec.get("keyframes", [])]
    out["keyframes"] = sorted(frames, key=lambda f: float(f.get("t", 0.0)))
    return with_tail(out, g)


def save_keyframe_gesture(spec: dict, dirpath: str = GESTURE_DIR, force: bool = False,
                          g=None) -> str:
    """Normalise, CHECK (pebble_feasibility), write gait/gestures/NAME.json.
    An infeasible spec raises ValueError whose message is the report's lines
    (the cockpit shows it) — unless force=True, which saves it anyway with
    "checked": {"ok": false, ...} on the record."""
    spec = normalize_spec(spec, g)
    kg = KeyframeGesture(spec, tail=False)            # parses (raises on a malformed spec)
    rep = kg.report(g=g)
    if not rep.ok and not force:
        raise ValueError("gesture is not feasible — not saved:\n" + "\n".join(rep.lines))
    spec["checked"] = {"ok": rep.ok, "params": _rm.params_rev(), "fails": rep.fails,
                       "peak_rad_s": round(rep.peak()[0], 2)}
    os.makedirs(dirpath, exist_ok=True)
    path = os.path.join(dirpath, f"{spec['name']}.json")
    with open(path, "w") as f:
        json.dump(spec, f, indent=1)
    return path


def delete_keyframe_gesture(name: str, dirpath: str = GESTURE_DIR) -> bool:
    path = os.path.join(dirpath, f"{_safe_name(name)}.json")
    if os.path.exists(path):
        os.remove(path)
        return True
    return False
