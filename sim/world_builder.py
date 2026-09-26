"""World builder for the cockpit (D049): compose a MuJoCo world around the
generated robot from a small JSON spec, at runtime.

    spec = {"base": "flat" | "room" | "cliff",
            "friction": 0.8,              # world sliding friction (default = params leg.foot.mu_slide)
            "gravity_tilt_deg": 0.0,      # tilt the gravity vector (a slope, cheaply)
            "gravity_tilt_dir_deg": 0.0,  # ... toward this map bearing
            "terrain": None | {"kind": "rough", "amp_m": 0.02, "size_m": 1.6,
                               "pos": [1.3, 0], "seed": 1, "cell_m": 0.05},
            "objects": [ {"kind": "box",    "pos": [x, y], "size": [sx, sy, sz], "yaw_deg": 0},
                         {"kind": "wall",   "pos": [x, y], "len_m": 1.0, "yaw_deg": 90},
                         {"kind": "ramp",   "pos": [x, y], "len_m": 0.6, "rise_m": 0.06, "yaw_deg": 0,
                                            "width_m": 0.6, "landing_m": 0.2, "far": "down"},
                         {"kind": "stairs", "pos": [x, y], "steps": 3, "rise_m": 0.02, "run_m": 0.15,
                                            "yaw_deg": 0, "landing_m": 0.2, "far": "down"},
                         {"kind": "rubble", "pos": [x, y], "radius_m": 0.5, "n": 25, "size_m": 0.03, "seed": 2},
                         {"kind": "ball",   "pos": [x, y], "radius_m": 0.05, "mass_kg": 0.1} ]}

Ramps and stairs start at `pos` and climb along `yaw_deg`. D052: by default
they have a far side (`"far": "down"`: a landing, then the mirror image back
to the floor), because a walker that tops a ramp and meets a 6 cm cliff is
testing the cliff reflex, not the ramp. `"far": "drop"` keeps a deliberate
ledge at the top (the pre-D052 shape, minus the float).

Friction (D052): every WORLD geom is emitted with priority="1", so for any
robot/world contact MuJoCo takes the world geom's friction/condim/solref
instead of the element-wise max of the pair. Before that, the feet (0.8,
1.2 before D052) always won over an "icy" 0.35 floor and the friction
setting did nothing. World geoms are condim 4 so the feet keep the
torsional friction build_mjcf gave them (the higher-priority geom's condim
is the one used). Robot geoms stay priority 0; pebble.xml is untouched.

Everything the lidar should see is geom group 3 (sim_lidar's convention),
so scan_summary works in every world. The robot gets an "eye" camera on
the torso (forward, slightly down) for the vision tools. build() returns
(MjModel, spawn_z): the base worlds place the robot at z = spawn_z (the
cliff platform's top).
"""
from __future__ import annotations
import os
import re
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
import rocky_model as rm    # noqa: E402

_FOOT = rm.params()["leg"]["foot"]
MU_BASE = float(_FOOT["mu_slide"])        # 0.8: the floor the robot was built for (D052)
MU_TORSION = float(_FOOT["mu_torsion_m"])  # 0.005 m, same as the feet (build_mjcf.FRICTION)
MU_ROLL = 0.0001                           # m, same as build_mjcf.FRICTION
CLEAR_M = 0.30      # random_course: no object footprint this close to the spawn point

DEFAULT = {"base": "flat", "friction": MU_BASE, "gravity_tilt_deg": 0.0,
           "gravity_tilt_dir_deg": 0.0, "terrain": None, "objects": []}
KINDS = ("box", "wall", "ramp", "stairs", "rubble", "ball")
OBST_RGBA = "0.55 0.5 0.62 1"
EYE_CAM = ('<camera name="eye" pos="0.10 0 0.095" mode="fixed" '
           'xyaxes="0 -1 0 0.26 0 0.97" fovy="70"/>')   # above the shoulder blocks, forward, 15 deg down


def _fric(spec):
    return f'{float(spec.get("friction", MU_BASE)):.3f} {MU_TORSION:g} {MU_ROLL:g}'


def _wattr(spec):
    """Contact attributes of every world geom (D052): the world's friction
    wins the pair (priority 1), condim 4 keeps the feet's torsional term."""
    return f'friction="{_fric(spec)}" condim="4" priority="1"'


def _quat(yaw_deg, pitch_rad=0.0):
    """qz(yaw) * qy(pitch): yaw the object first, then pitch it about its OWN
    y axis. (MJCF euler="0 -ang yaw" is the other order: the slope ended up
    canted sideways on any yawed ramp — found in the D052 review.)"""
    a = np.radians(float(yaw_deg)) / 2
    qz = np.array([np.cos(a), 0.0, 0.0, np.sin(a)])
    qy = np.array([np.cos(pitch_rad / 2), 0.0, np.sin(pitch_rad / 2), 0.0])
    q = np.zeros(4)
    mujoco.mju_mulQuat(q, qz, qy)
    return " ".join(f"{v:.7f}" for v in q)


def _run_pieces(o):
    """Ramp / stairs as boxes in the object's own frame: x runs from `pos`
    along yaw, z up from the floor. Each piece is (cx, cz, hx, hy, hz, pitch)
    with pitch about local y (MuJoCo sign: negative pitch climbs toward +x).
    Pure geometry — the XML and the footprint check both read it."""
    k = o.get("kind")
    far = o.get("far", "down")
    if far not in ("down", "drop"):
        raise ValueError(f"far must be 'down' or 'drop', not {far!r}")
    land = float(o.get("landing_m", 0.2))
    w = float(o.get("width_m", 0.6))
    out = []
    if k == "ramp":
        L = float(o.get("len_m", 0.6))
        rise = float(o.get("rise_m", 0.06))
        a = float(np.arctan2(rise, L))
        # a THICK slab sunk into the floor: its top face runs from (0, 0) to
        # (L, rise), and it is thick enough (2T cos a >= 2 rise) that the high
        # end reaches below the floor. The old 1 cm slab floated at the top
        # with a gap under it; the buried part is harmless.
        T = max(0.01, rise / np.cos(a))
        hl = np.hypot(L, rise) / 2
        out.append((L / 2 + T * np.sin(a), rise / 2 - T * np.cos(a), hl, w / 2, T, -a))
        if far == "down":
            if land > 0:
                out.append((L + land / 2, rise / 2, land / 2, w / 2, rise / 2, 0.0))
            x0 = L + max(land, 0.0)
            out.append((x0 + L / 2 - T * np.sin(a), rise / 2 - T * np.cos(a), hl, w / 2, T, a))
        return out
    if k == "stairs":
        n = int(o.get("steps", 3))
        rise = float(o.get("rise_m", 0.02))
        run = float(o.get("run_m", 0.15))
        heights = [(run * (s + 0.5), run / 2, rise * (s + 1)) for s in range(n)]
        if far == "down":
            x0 = n * run
            if land > 0:
                heights.append((x0 + land / 2, land / 2, rise * n))
            x0 += max(land, 0.0)
            # mirror image: every drop on the way down is one rise, never more
            heights += [(x0 + run * (j + 0.5), run / 2, rise * (n - 1 - j)) for j in range(n - 1)]
        return [(cx, hz / 2, hx, w / 2, hz / 2, 0.0) for cx, hx, hz in heights]
    raise ValueError(f"{k!r} is not a ramp or stairs")


def _obj_xml(o, i, spec):
    k = o.get("kind", "box")
    x, y = (float(v) for v in o.get("pos", [0.6, 0.0]))
    yaw = float(o.get("yaw_deg", 0.0))
    fr = _wattr(spec)
    if k == "box":
        sx, sy, sz = (float(v) for v in o.get("size", [0.1, 0.1, 0.05]))
        return (f'<geom name="obj{i}" type="box" size="{sx/2:.4f} {sy/2:.4f} {sz/2:.4f}" '
                f'pos="{x:.3f} {y:.3f} {sz/2:.4f}" euler="0 0 {yaw}" group="3" '
                f'{fr} rgba="{OBST_RGBA}"/>')
    if k == "wall":
        L = float(o.get("len_m", 1.0))
        h = float(o.get("height_m", 0.25))
        return (f'<geom name="obj{i}" type="box" size="{L/2:.4f} 0.02 {h/2:.4f}" '
                f'pos="{x:.3f} {y:.3f} {h/2:.4f}" euler="0 0 {yaw}" group="3" '
                f'{fr} rgba="0.5 0.47 0.58 1"/>')
    if k in ("ramp", "stairs"):
        c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
        rgba = "0.5 0.55 0.5 1" if k == "ramp" else "0.5 0.5 0.6 1"
        out = []
        for j, (cx, cz, hx, hy, hz, pitch) in enumerate(_run_pieces(o)):
            # the ramp's climbing slab keeps the plain name obj{i} (it always did)
            nm = f"obj{i}" if (k == "ramp" and j == 0) else f"obj{i}_{j}"
            out.append(f'<geom name="{nm}" type="box" size="{hx:.4f} {hy:.4f} {hz:.4f}" '
                       f'pos="{x + c * cx:.4f} {y + s * cx:.4f} {cz:.4f}" quat="{_quat(yaw, pitch)}" '
                       f'group="3" {fr} rgba="{rgba}"/>')
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
                           f'group="3" {fr} rgba="0.45 0.42 0.5 1"/>')
            else:
                out.append(f'<geom name="obj{i}_{s}" type="sphere" size="{e[0]/2:.4f}" '
                           f'pos="{px:.3f} {py:.3f} {e[0]/2:.4f}" group="3" {fr} '
                           f'rgba="0.45 0.42 0.5 1"/>')
        return "\n".join(out)
    if k == "ball":
        r = float(o.get("radius_m", 0.05))
        m = float(o.get("mass_kg", 0.1))
        return (f'<body name="obj{i}" pos="{x:.3f} {y:.3f} {r + 0.001:.4f}"><freejoint/>'
                f'<geom type="sphere" size="{r:.4f}" mass="{m:.3f}" group="3" {fr} '
                f'rgba="0.85 0.6 0.3 1"/></body>')
    raise ValueError(f"unknown object kind {k!r} (have {KINDS})")


# ------------------------------------------------------------ footprints
def footprint(o):
    """The object's plan-view footprint as shapes: ("rect", cx, cy, hx, hy,
    yaw_rad) or ("disk", cx, cy, r). Conservative (buried ramp slab ends and
    the rubble's largest possible stone are included)."""
    k = o.get("kind", "box")
    x, y = (float(v) for v in o.get("pos", [0.6, 0.0]))
    yaw = np.radians(float(o.get("yaw_deg", 0.0)))
    if k == "box":
        sx, sy, _ = (float(v) for v in o.get("size", [0.1, 0.1, 0.05]))
        return [("rect", x, y, sx / 2, sy / 2, yaw)]
    if k == "wall":
        return [("rect", x, y, float(o.get("len_m", 1.0)) / 2, 0.02, yaw)]
    if k in ("ramp", "stairs"):
        c, s = np.cos(yaw), np.sin(yaw)
        out = []
        for cx, _cz, hx, hy, hz, p in _run_pieces(o):
            ex = hx * abs(np.cos(p)) + hz * abs(np.sin(p))
            out.append(("rect", x + c * cx, y + s * cx, ex, hy, yaw))
        return out
    if k == "rubble":
        return [("disk", x, y, float(o.get("radius_m", 0.5)) + float(o.get("size_m", 0.03)))]
    if k == "ball":
        return [("disk", x, y, float(o.get("radius_m", 0.05)))]
    raise ValueError(f"unknown object kind {k!r} (have {KINDS})")


def footprint_dist(o, p=(0.0, 0.0)):
    """Plan-view distance from point p to the object's footprint (0 = inside)."""
    best = np.inf
    for sh in footprint(o):
        dx, dy = float(p[0]) - sh[1], float(p[1]) - sh[2]
        if sh[0] == "disk":
            d = max(np.hypot(dx, dy) - sh[3], 0.0)
        else:
            _, _, _, hx, hy, yaw = sh
            c, s = np.cos(yaw), np.sin(yaw)
            u, v = c * dx + s * dy, -s * dx + c * dy
            d = np.hypot(max(abs(u) - hx, 0.0), max(abs(v) - hy, 0.0))
        best = min(best, d)
    return best


def _terrain_xml(t, spec):
    n = int(round(float(t.get("size_m", 1.6)) / float(t.get("cell_m", 0.05))))
    n = max(8, min(n, 128))
    half = float(t.get("size_m", 1.6)) / 2
    amp = float(t.get("amp_m", 0.02))
    px, py = (float(v) for v in t.get("pos", [1.3, 0.0]))
    asset = f'<hfield name="rough" nrow="{n}" ncol="{n}" size="{half:.3f} {half:.3f} {amp:.4f} 0.01"/>'
    # (before D052 the terrain carried no friction at all: MuJoCo's default 1.0)
    geom = (f'<geom name="terrain" type="hfield" hfield="rough" pos="{px:.3f} {py:.3f} 0" '
            f'group="3" {_wattr(spec)} rgba="0.38 0.36 0.42 1"/>')
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


_ATTR_RE = r'\s(?:friction|condim|priority)="[^"]*"'


def _world_geom(tag, spec):
    """Rewrite one existing <geom .../> tag as a world geom (priority 1)."""
    tag = re.sub(_ATTR_RE, "", tag)
    return re.sub(r"\s*/>$", f" {_wattr(spec)}/>", tag)


def world_xml(spec):
    spec = {**DEFAULT, **(spec or {})}
    with open(os.path.join(HERE, "pebble.xml")) as f:
        xml = f.read()
    # the floor: matched by name, not by its exact text (build_mjcf owns that
    # line and changed it in D052, which silently broke the old str.replace)
    xml, nfloor = re.subn(r'<geom name="floor"[^>]*/>', lambda m: _world_geom(m.group(0), spec), xml, count=1)
    if nfloor != 1:
        raise RuntimeError("pebble.xml has no <geom name=\"floor\" .../> line to rewrite")
    # the robot's eye
    xml = xml.replace("<freejoint/>", "<freejoint/>\n      " + EYE_CAM, 1)
    body, assets = [], []
    spawn_z = 0.0
    base = spec.get("base", "flat")
    if base == "room":
        from sim_lidar import ROOM
        body.append(re.sub(r"<geom [^>]*/>", lambda m: _world_geom(m.group(0), spec), ROOM))
    elif base == "cliff":
        from run_cliff import EDGE_X, PLAT_H
        body.append(f'<geom name="platform" type="box" size="{(EDGE_X + 0.45) / 2:.3f} 0.5 {PLAT_H / 2:.3f}" '
                    f'pos="{(EDGE_X - 0.45) / 2:.3f} 0 {PLAT_H / 2:.3f}" {_wattr(spec)} '
                    f'rgba="0.45 0.4 0.55 1"/>')
        spawn_z = PLAT_H
    for i, o in enumerate(spec.get("objects") or []):
        body.append(_obj_xml(o, i, spec))
    n_hf = 0
    if spec.get("terrain"):
        asset, geom, n_hf = _terrain_xml(spec["terrain"], spec)
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
    # D052: the ramp now has a far side (1.1 m long in all), so the wall moved
    # from y -0.3 to +0.1 to stay off its landing; width 0.5 leaves a 5 cm gap
    "obstacle course": {"base": "flat", "objects": [
        {"kind": "box", "pos": [0.7, 0.15], "size": [0.15, 0.15, 0.06]},
        {"kind": "wall", "pos": [1.2, 0.1], "len_m": 0.8, "yaw_deg": 90},
        {"kind": "ramp", "pos": [0.4, -0.6], "len_m": 0.45, "rise_m": 0.05, "width_m": 0.5,
         "landing_m": 0.2, "yaw_deg": 0},
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


# ---- the place bench's rooms (sim/place_bench.py, 2026-09-25) --------------
# Added after the presets above, which stay as they were. Three distinct rooms
# the bench enrols (a, b, c), one it never enrols (d), and the same rooms with
# one thing changed. The lidar's scan plane is ~0.18 m above the floor, so
# what should shape a room's lidar signature is at least 0.2 m tall (walls
# 0.25 m, the room base's walls 0.5 m); low things (the 6 cm box, the ramp,
# the ball) are for the eye only. With the robot at its spawn (the origin,
# facing +x):
#   room a  the `room` base: 3.2 x 2.6 m walls, two pillars, a crate behind.
#   room b  the same walls (the lidar's near-twin of a) + an inner wall ahead
#           on the right and two tall boxes BEHIND the robot: the eye sees the
#           wall, so a box put in front of it later is new to the eye.
#   room c  a flat floor, obstacle-course style: a long wall on the right, one
#           across ahead, a stub behind on the left; a low box, a ramp on the
#           left, the ball ahead-left (the thing "room c - ball" takes away).
#   room d  a hallway: two parallel 3 m walls 0.9 m apart, stairs ahead.
#           The bench never enrols it (the "never seen" room).
#   "+ chair"  a 0.2 x 0.2 x 0.25 m box ahead where there was none, in view
#           of the eye at the spawn and after the bench's 30 deg second-look
#           turn to the left.
# The bench loads them under opaque world names: a name never tells the
# robot where it is.
_CHAIR = {"kind": "box", "size": [0.2, 0.2, 0.25]}
_ROOM_B_OBJECTS = [
    {"kind": "wall", "pos": [0.95, -0.45], "len_m": 0.9, "yaw_deg": 90},
    {"kind": "box", "pos": [-0.55, 0.6], "size": [0.3, 0.2, 0.22], "yaw_deg": 20},
    {"kind": "box", "pos": [-0.5, -0.7], "size": [0.2, 0.2, 0.22]}]
_ROOM_C_OBJECTS = [
    {"kind": "wall", "pos": [0.5, -0.65], "len_m": 2.2, "yaw_deg": 0},
    {"kind": "wall", "pos": [1.6, 0.0], "len_m": 1.3, "yaw_deg": 90},
    {"kind": "wall", "pos": [-0.7, 0.35], "len_m": 0.8, "yaw_deg": 90},
    {"kind": "box", "pos": [0.9, -0.3], "size": [0.15, 0.15, 0.06]},
    {"kind": "ramp", "pos": [0.1, 0.55], "len_m": 0.45, "rise_m": 0.05, "width_m": 0.4,
     "landing_m": 0.2, "yaw_deg": 0},
    {"kind": "ball", "pos": [0.75, 0.2], "radius_m": 0.05}]
_ROOM_D_OBJECTS = [
    {"kind": "wall", "pos": [0.4, 0.45], "len_m": 3.0, "yaw_deg": 0},
    {"kind": "wall", "pos": [0.4, -0.45], "len_m": 3.0, "yaw_deg": 0},
    {"kind": "stairs", "pos": [1.1, 0.0], "steps": 3, "rise_m": 0.015, "run_m": 0.15, "width_m": 0.5}]


def _bench_room(base, objects, chair_at=None, drop_kind=None):
    """A fresh (deep-copied) spec: `objects`, minus every `drop_kind`, plus a chair at chair_at."""
    objs = [dict(o) for o in objects if o["kind"] != drop_kind]
    if chair_at is not None:
        objs.append(dict(_CHAIR, pos=list(chair_at)))
    return {"base": base, "objects": [{k: (list(v) if isinstance(v, list) else v) for k, v in o.items()}
                                      for o in objs]}


PRESETS.update({
    "room a": _bench_room("room", []),
    "room b": _bench_room("room", _ROOM_B_OBJECTS),
    "room c": _bench_room("flat", _ROOM_C_OBJECTS),
    "room d": _bench_room("flat", _ROOM_D_OBJECTS),
    "room a + chair": _bench_room("room", [], chair_at=(0.7, 0.05)),
    "room b + chair": _bench_room("room", _ROOM_B_OBJECTS, chair_at=(0.65, 0.15)),
    "room c + chair": _bench_room("flat", _ROOM_C_OBJECTS, chair_at=(1.25, 0.1)),
    "room c - ball": _bench_room("flat", _ROOM_C_OBJECTS, drop_kind="ball"),
})


WORLDS_DIR = os.path.join(HERE, "worlds")


def random_course(seed=0, n=8, base="flat", r_min=0.45, r_max=1.6, clear_m=CLEAR_M, tries=50):
    """A seeded scatter of obstacles in an annulus around the origin: the
    RL-curriculum world. D052: `pos` in the annulus did not keep the spawn
    clear (a yawed box corner, or stairs pointing inward, reached the robot
    in 40 of 200 seeds), so each object's rotated footprint must stay
    `clear_m` from the origin; a placement that fails is resampled (same
    kind, same rng stream, so the course is still a pure function of seed).
    Friction is drawn around the D052 base (0.5-1.0; it used to be 0.6-1.4,
    which the feet then overrode anyway)."""
    rng = np.random.default_rng(int(seed))
    kinds = ["box", "box", "wall", "ramp", "stairs", "ball", "rubble"]
    objs = []
    for _ in range(int(n)):
        k = kinds[rng.integers(len(kinds))]
        for _try in range(int(tries)):
            r = rng.uniform(r_min, r_max)
            a = rng.uniform(0, 2 * np.pi)
            o = {"kind": k, "pos": [round(float(r * np.cos(a)), 2), round(float(r * np.sin(a)), 2)],
                 "yaw_deg": round(float(rng.uniform(0, 180)), 0)}
            if k == "box":
                o["size"] = [round(float(rng.uniform(0.08, 0.25)), 2), round(float(rng.uniform(0.08, 0.25)), 2),
                             round(float(rng.uniform(0.02, 0.08)), 3)]
            elif k == "wall":
                o["len_m"] = round(float(rng.uniform(0.4, 1.0)), 2)
            elif k == "rubble":
                o.update(radius_m=0.3, n=12, size_m=0.03, seed=int(rng.integers(1000)))
            elif k == "stairs":
                o.update(steps=3, rise_m=0.015)
            if footprint_dist(o) >= clear_m:
                objs.append(o)
                break
    return {"base": base, "objects": objs, "friction": round(float(rng.uniform(0.5, 1.0)), 2)}


def saved_worlds():
    if not os.path.isdir(WORLDS_DIR):
        return []
    return sorted(f[:-5] for f in os.listdir(WORLDS_DIR) if f.endswith(".json"))


def save_world(name, spec):
    import json
    os.makedirs(WORLDS_DIR, exist_ok=True)
    name = re.sub(r"[^A-Za-z0-9_. -]", "", name).strip() or "world"
    with open(os.path.join(WORLDS_DIR, name + ".json"), "w") as f:
        json.dump(spec, f, indent=1)
    return name


def load_world(name):
    import json
    with open(os.path.join(WORLDS_DIR, name + ".json")) as f:
        return json.load(f)


if __name__ == "__main__":
    for name, spec in PRESETS.items():
        m, z = build(spec)
        print(f"{name:16s} ngeom {m.ngeom:3d} nbody {m.nbody:3d} ncam {m.ncam} hfield {m.nhfield} spawn_z {z}")
