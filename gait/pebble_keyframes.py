"""Keyframe gestures (D051) — gestures as DATA, not code.

The canon gestures (pebble_gestures*.py) are hand-written functions of
time. This module plays a gesture written as a list of keyframes, so a
new one can be composed in the cockpit's Gesture studio (pose the body,
lift a leg, snap a frame, scrub, save) and then runs everywhere the code
ones run: the playground, the cockpit, the MCP `gesture` tool and, later,
the servo bus on the real robot — the output is the same (q[5,3] rad,
claw[5] rad) stream.

File: gait/gestures/NAME.json
    {"name": "peek", "loop": false,
     "keyframes": [
       {"t": 0.0},
       {"t": 0.8, "body": [10, 0, -15], "yaw": 12, "dz": [0, 0, 8, 8, 0],
        "arm": {"0": [0, 70, -50]}, "claw": [1, 0, 0, 0, 0], "say": "greeting",
        "ease": "smooth"}]}

Per keyframe (all optional, missing = nominal):
    t      seconds from the start (frames sorted by t; the last t = duration)
    body   [dx, dy, dz] mm body displacement (z < 0 crouches)
    yaw    deg body yaw, feet fixed in the world
    dz     [5] mm extra crouch per corner (positive = that corner lower)
    arm    {leg: [yaw, hip, knee] deg} joint overrides — a raised leg/arm
    claw   [5] 0..1 open fraction per hand
    say    chord-speak cue fired when the frame is reached
    ease   how to get INTO this frame: smooth (default) | linear | hold
           (hold = jump at the frame time)

Between frames the body pose is interpolated and run through the planted
IK; an `arm` override is blended in joint space. Pure Python, no sim.
"""
from __future__ import annotations
import json
import os

import numpy as np
from pebble_gait import N_LEGS
from pebble_gestures2 import posed, CLAW_MAX

HERE = os.path.dirname(os.path.abspath(__file__))
GESTURE_DIR = os.path.join(HERE, "gestures")
KEYS = ("body", "yaw", "dz", "arm", "claw", "say", "ease", "t")


def _ease(a, kind):
    a = float(np.clip(a, 0.0, 1.0))
    if kind == "linear":
        return a
    if kind == "hold":
        return 1.0 if a >= 1.0 else 0.0
    return a * a * (3 - 2 * a)                       # smooth


class Frame:
    def __init__(self, d: dict):
        self.t = float(d.get("t", 0.0))
        self.body = np.asarray(d.get("body", [0, 0, 0]), float)
        self.yaw = float(d.get("yaw", 0.0))
        self.dz = np.asarray(d.get("dz", [0] * N_LEGS), float)
        self.arm = {int(k): np.deg2rad(np.asarray(v, float)) for k, v in (d.get("arm") or {}).items()}
        self.claw = np.asarray(d.get("claw", [0] * N_LEGS), float)
        self.say = d.get("say")
        self.ease = d.get("ease", "smooth")

    def pose(self, g):
        """(q[5,3], claw_rad[5]) for this frame's body pose; arms applied."""
        q, _c = posed(g, offset=self.body, yaw=np.deg2rad(self.yaw), dz_leg=self.dz)
        for i, qi in self.arm.items():
            q[i] = qi
        return q, np.clip(self.claw, 0, 1) * CLAW_MAX


class KeyframeGesture:
    def __init__(self, spec: dict):
        self.name = spec.get("name", "untitled")
        self.loop = bool(spec.get("loop", False))
        frames = sorted((Frame(f) for f in spec.get("keyframes", [])), key=lambda f: f.t)
        if not frames:
            frames = [Frame({"t": 0.0})]
        if frames[0].t > 0:
            frames.insert(0, Frame({"t": 0.0}))     # start from the nominal stance
        self.frames = frames
        self.total = max(frames[-1].t, 0.05)
        self.cues = [(f.t, f.say) for f in frames if f.say]
        self.spec = spec

    def __call__(self, g, t):
        """(q, claw) at time t — the same contract as pebble_gestures*."""
        if self.loop and self.total > 0:
            t = t % self.total
        fr = self.frames
        if t >= fr[-1].t:
            return fr[-1].pose(g)
        k = 0
        while k + 1 < len(fr) and fr[k + 1].t <= t:
            k += 1
        a, b = fr[k], fr[k + 1]
        u = _ease((t - a.t) / max(b.t - a.t, 1e-6), b.ease)
        qa, ca = a.pose(g)
        qb, cb = b.pose(g)
        return (1 - u) * qa + u * qb, (1 - u) * ca + u * cb

    def as_entry(self):
        """GESTURES2-style tuple: (fn, total, [(t, chord word)])."""
        return (self, self.total, list(self.cues))


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
            except (ValueError, KeyError) as e:          # a half-written file must not kill the loader
                print(f"keyframes: skipped {fn}: {e}")
    return out


def _safe_name(name: str) -> str:
    name = "".join(c if c.isalnum() or c in "-_" else "_" for c in name.strip())[:40]
    if not name:
        raise ValueError("empty gesture name")
    return name


def save_keyframe_gesture(spec: dict, dirpath: str = GESTURE_DIR) -> str:
    name = _safe_name(spec.get("name", ""))
    spec = dict(spec, name=name)
    spec["keyframes"] = [{k: v for k, v in f.items() if k in KEYS} for f in spec.get("keyframes", [])]
    KeyframeGesture(spec)                              # validates
    os.makedirs(dirpath, exist_ok=True)
    path = os.path.join(dirpath, f"{name}.json")
    with open(path, "w") as f:
        json.dump(spec, f, indent=1)
    return path


def delete_keyframe_gesture(name: str, dirpath: str = GESTURE_DIR) -> bool:
    path = os.path.join(dirpath, f"{_safe_name(name)}.json")
    if os.path.exists(path):
        os.remove(path)
        return True
    return False
