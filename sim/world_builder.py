"""World builder for the cockpit (D049): compose a MuJoCo world around the
generated robot from a small JSON spec, at runtime.

    spec = {"base": "flat" | "room" | "cliff",
            "friction": 1.2,              # floor + obstacle sliding friction
            "gravity_tilt_deg": 0.0,      # tilt the gravity vector (a slope, cheaply)
            "gravity_tilt_dir_deg": 0.0,  # ... toward this map bearing
            "terrain": None | {"kind": "rough", "amp_m": 0.02, "size_m": 1.6,
                               "pos": [1.3, 0], "seed": 1, "cell_m": 0.05},
            "objects": [ {"kind": "box",    "pos": [x, y], "size": [sx, sy, sz], "yaw_deg": 0},
                         {"kind": "wall",   "pos": [x, y], "len_m": 1.0, "yaw_deg": 90},
                         {"kind": "ramp",   "pos": [x, y], "len_m": 0.6, "rise_m": 0.06, "yaw_deg": 0},
                         {"kind": "stairs", "pos": [x, y], "steps": 3, "rise_m": 0.02, "yaw_deg": 0},
                         {"kind": "rubble", "pos": [x, y], "radius_m": 0.5, "n": 25, "size_m": 0.03, "seed": 2},
                         {"kind": "ball",   "pos": [x, y], "radius_m": 0.05, "mass_kg": 0.1} ]}

Everything the lidar should see is geom group 3 (sim_lidar's convention),
so scan_summary works in every world. The robot gets an "eye" camera on
the torso (forward, slightly down) for the vision tools. build() returns
(MjModel, spawn_z): the base worlds place the robot at z = spawn_z (the
cliff platform's top).
"""
from __future__ import annotations
import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DEFAULT = {"base": "flat", "friction": 1.2, "gravity_tilt_deg": 0.0,
           "gravity_tilt_dir_deg": 0.0, "terrain": None, "objects": []}
KINDS = ("box", "wall", "ramp", "stairs", "rubble", "ball")
OBST_RGBA = "0.55 0.5 0.62 1"
EYE_CAM = ('<camera name="eye" pos="0.10 0 0.095" mode="fixed" '
           'xyaxes="0 -1 0 0.26 0 0.97" fovy="70"/>')   # above the shoulder blocks, forward, 15 deg down


def _fric(spec):
    return f'{float(spec.get("friction", 1.2)):.3f} 0.01 0.001'


def _obj_xml(o, i, spec):
    k = o.get("kind", "box")
    x, y = (float(v) for v in o.get("pos", [0.6, 0.0]))
    yaw = float(o.get("yaw_deg", 0.0))
    fr = _fric(spec)
    if k == "box":
        sx, sy, sz = (float(v) for v in o.get("size", [0.1, 0.1, 0.05]))
        return (f'<geom name="obj{i}" type="box" size="{sx/2:.4f} {sy/2:.4f} {sz/2:.4f}" '
                f'pos="{x:.3f} {y:.3f} {sz/2:.4f}" euler="0 0 {yaw}" group="3" '
                f'friction="{fr}" rgba="{OBST_RGBA}"/>')
    if k == "wall":
        L = float(o.get("len_m", 1.0))
        h = float(o.get("height_m", 0.25))
        return (f'<geom name="obj{i}" type="box" size="{L/2:.4f} 0.02 {h/2:.4f}" '
                f'pos="{x:.3f} {y:.3f} {h/2:.4f}" euler="0 0 {yaw}" group="3" '
                f'friction="{fr}" rgba="0.5 0.47 0.58 1"/>')
    if k == "ramp":
        L = float(o.get("len_m", 0.6))
        rise = float(o.get("rise_m", 0.06))
        w = float(o.get("width_m", 0.6))
        ang = np.degrees(np.arctan2(rise, L))
        # a thin slab tilted about its own y, resting on its low edge
        t = 0.01
        cx = x + np.cos(np.radians(yaw)) * L / 2
        cy = y + np.sin(np.radians(yaw)) * L / 2
        return (f'<geom name="obj{i}" type="box" size="{np.hypot(L, rise)/2:.4f} {w/2:.4f} {t:.4f}" '
                f'pos="{cx:.3f} {cy:.3f} {rise/2 + t:.4f}" euler="0 {-ang:.2f} {yaw}" group="3" '
                f'friction="{fr}" rgba="0.5 0.55 0.5 1"/>')
    if k == "stairs":
        n = int(o.get("steps", 3))
        rise = float(o.get("rise_m", 0.02))
        run = float(o.get("run_m", 0.15))
        w = float(o.get("width_m", 0.6))
        out = []
        for s in range(n):
            d = run * (s + 0.5)
            cx = x + np.cos(np.radians(yaw)) * d
            cy = y + np.sin(np.radians(yaw)) * d
            hz = rise * (s + 1)
            out.append(f'<geom name="obj{i}_{s}" type="box" size="{run/2:.4f} {w/2:.4f} {hz/2:.4f}" '
                       f'pos="{cx:.3f} {cy:.3f} {hz/2:.4f}" euler="0 0 {yaw}" group="3" '
                       f'friction="{fr}" rgba="0.5 0.5 0.6 1"/>')
        return "\n".join(out)
    if k == "rubble":
        rng = np.random.default_rng(int(o.get("seed", 2)))
        n = int(o.get("n", 25))
        rad = float(o.get("radius_m", 0.5))
        sz = float(o.get("size_m", 0.03))
        out = []
        for s in range(n):
            r = rad * np.sqrt(rng.uniform(0.05, 1.0))
            a = rng.uniform(0, 2 * np.pi)
            px, py = x + r * np.cos(a), y + r * np.sin(a)
            e = rng.uniform(0.5, 1.0, 3) * sz
            if rng.uniform() < 0.5:
                out.append(f'<geom name="obj{i}_{s}" type="box" size="{e[0]/2:.4f} {e[1]/2:.4f} {e[2]/2:.4f}" '
                           f'pos="{px:.3f} {py:.3f} {e[2]/2:.4f}" euler="0 0 {rng.uniform(0, 90):.1f}" '
                           f'group="3" friction="{fr}" rgba="0.45 0.42 0.5 1"/>')
            else:
                out.append(f'<geom name="obj{i}_{s}" type="sphere" size="{e[0]/2:.4f}" '
                           f'pos="{px:.3f} {py:.3f} {e[0]/2:.4f}" group="3" friction="{fr}" '
                           f'rgba="0.45 0.42 0.5 1"/>')
        return "\n".join(out)
    if k == "ball":
        r = float(o.get("radius_m", 0.05))
        m = float(o.get("mass_kg", 0.1))
        return (f'<body name="obj{i}" pos="{x:.3f} {y:.3f} {r + 0.001:.4f}"><freejoint/>'
                f'<geom type="sphere" size="{r:.4f}" mass="{m:.3f}" group="3" friction="{fr}" '
                f'rgba="0.85 0.6 0.3 1"/></body>')
    raise ValueError(f"unknown object kind {k!r} (have {KINDS})")


def _terrain_xml(t):
    n = int(round(float(t.get("size_m", 1.6)) / float(t.get("cell_m", 0.05))))
    n = max(8, min(n, 128))
    half = float(t.get("size_m", 1.6)) / 2
    amp = float(t.get("amp_m", 0.02))
    px, py = (float(v) for v in t.get("pos", [1.3, 0.0]))
    asset = f'<hfield name="rough" nrow="{n}" ncol="{n}" size="{half:.3f} {half:.3f} {amp:.4f} 0.01"/>'
    geom = (f'<geom name="terrain" type="hfield" hfield="rough" pos="{px:.3f} {py:.3f} 0" '
            f'group="3" rgba="0.38 0.36 0.42 1"/>')
    return asset, geom, n


def _terrain_data(n, seed):
    """Smooth-ish random heights in [0, 1]: low-pass noise so the field is
    rubble-like ridges, not a bed of needles."""
    rng = np.random.default_rng(int(seed))
    z = rng.uniform(0, 1, (n + 4, n + 4))
    for _ in range(2):
        z = (z[:-2, :-2] + z[1:-1, :-2] + z[2:, :-2] + z[:-2, 1:-1] + z[1:-1, 1:-1] +
             z[2:, 1:-1] + z[:-2, 2:] + z[1:-1, 2:] + z[2:, 2:]) / 9.0
        z = np.pad(z, 1, mode="edge")
    z = z[1:n + 1, 1:n + 1]
    z = (z - z.min()) / max(z.max() - z.min(), 1e-9)
    return z.astype(np.float64).ravel()


def world_xml(spec):
    spec = {**DEFAULT, **(spec or {})}
    with open(os.path.join(HERE, "pebble.xml")) as f:
        xml = f.read()
    fr = _fric(spec)
    # floor friction + lidar-visible floor (walls/obstacles are what the scan reports)
    xml = xml.replace('friction="1.2 0.01 0.001"/>\n    <body name="torso"',
                      f'friction="{fr}"/>\n    <body name="torso"')
    xml = xml.replace('<geom name="floor" type="plane" size="6 6 0.1" material="grid" friction="1.2 0.01 0.001"/>',
                      f'<geom name="floor" type="plane" size="6 6 0.1" material="grid" friction="{fr}"/>')
    # the robot's eye
    xml = xml.replace("<freejoint/>", "<freejoint/>\n      " + EYE_CAM, 1)
    body, assets = [], []
    spawn_z = 0.0
    base = spec.get("base", "flat")
    if base == "room":
        from sim_lidar import ROOM
        body.append(ROOM)
    elif base == "cliff":
        from run_cliff import EDGE_X, PLAT_H
        body.append(f'<geom name="platform" type="box" size="{(EDGE_X + 0.45) / 2:.3f} 0.5 {PLAT_H / 2:.3f}" '
                    f'pos="{(EDGE_X - 0.45) / 2:.3f} 0 {PLAT_H / 2:.3f}" friction="{fr}" '
                    f'rgba="0.45 0.4 0.55 1"/>')
        spawn_z = PLAT_H
    for i, o in enumerate(spec.get("objects") or []):
        body.append(_obj_xml(o, i, spec))
    n_hf = 0
    if spec.get("terrain"):
        asset, geom, n_hf = _terrain_xml(spec["terrain"])
        assets.append(asset)
        body.append(geom)
    if assets:
        xml = xml.replace("<asset>", "<asset>\n    " + "\n    ".join(assets), 1)
    if body:
        # AFTER the torso: every sim entry point addresses the robot's free
        # joint as qpos[0:7], so no world body may come before it
        xml = xml.replace("</worldbody>", "    " + "\n    ".join(body) + "\n  </worldbody>", 1)
    tilt = float(spec.get("gravity_tilt_deg", 0.0))
    if abs(tilt) > 1e-6:
        d = np.radians(float(spec.get("gravity_tilt_dir_deg", 0.0)))
        g = 9.81
        gx = g * np.sin(np.radians(tilt)) * np.cos(d)
        gy = g * np.sin(np.radians(tilt)) * np.sin(d)
        gz = -g * np.cos(np.radians(tilt))
        xml = xml.replace('<option timestep="0.002"', f'<option gravity="{gx:.4f} {gy:.4f} {gz:.4f}" timestep="0.002"', 1)
    return xml, spawn_z, n_hf, spec


def build(spec):
    xml, spawn_z, n_hf, spec = world_xml(spec)
    model = mujoco.MjModel.from_xml_string(xml)
    if n_hf:
        model.hfield_data[:] = _terrain_data(n_hf, (spec.get("terrain") or {}).get("seed", 1))
    return model, spawn_z


PRESETS = {
    "flat": {"base": "flat"},
    "room": {"base": "room"},
    "cliff": {"base": "cliff"},
    "obstacle course": {"base": "flat", "objects": [
        {"kind": "box", "pos": [0.7, 0.15], "size": [0.15, 0.15, 0.06]},
        {"kind": "wall", "pos": [1.2, -0.3], "len_m": 0.8, "yaw_deg": 90},
        {"kind": "ramp", "pos": [0.4, -0.6], "len_m": 0.6, "rise_m": 0.05, "yaw_deg": 0},
        {"kind": "ball", "pos": [0.5, 0.5], "radius_m": 0.05}]},
    "rubble field": {"base": "flat", "objects": [
        {"kind": "rubble", "pos": [0.9, 0.0], "radius_m": 0.5, "n": 30, "size_m": 0.03}]},
    "rough terrain": {"base": "flat", "terrain": {"kind": "rough", "amp_m": 0.02, "size_m": 1.6,
                                                   "pos": [1.3, 0.0], "seed": 1}},
    "stairs": {"base": "flat", "objects": [
        {"kind": "stairs", "pos": [0.6, 0.0], "steps": 4, "rise_m": 0.015, "run_m": 0.15}]},
    "slope 8 deg": {"base": "flat", "gravity_tilt_deg": 8.0, "gravity_tilt_dir_deg": 0.0},
    "icy floor": {"base": "flat", "friction": 0.35},
}


if __name__ == "__main__":
    for name, spec in PRESETS.items():
        m, z = build(spec)
        print(f"{name:16s} ngeom {m.ngeom:3d} nbody {m.nbody:3d} ncam {m.ncam} hfield {m.nhfield} spawn_z {z}")
