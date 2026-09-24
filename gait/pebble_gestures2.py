"""Gesture library v2 (session 8) — wave, bow, look-around, shake, sit,
turn-in-place, sidestep.

Same contract as pebble_gestures.py: every gesture is fn(g, t) -> (q[5,3]
rad, claw[5] rad), pure Python, no sim imports, so the driver on the Pi
runs the identical code. All expressive poses are authored in JOINT space
inside the real ranges (D035): yaw ±40°, hip [-70, 90], knee [-150, -20],
claw [0, 55]. Planted-foot poses go through the same _planted_q() the
canon gestures use (feet stay where they are; the BODY moves).

New here: `posed()` accepts a BODY YAW with planted feet (look-around —
lidar is the eyes, the whole rock turns to look) and PER-LEG height
offsets (bow — the north side dips, the body pitches). The self-test runs
every entry through pebble_feasibility.check (D052): guarded joint ranges,
loaded 3.0 / free 4.0 / hard 4.7 rad/s, no steps, >= 3 feet, CoM margin,
slip. stability_margin() is CoM-aware now (it projected the body ORIGIN,
which ignores the raised arm's own mass — two adjacent arms in a JSON
gesture scored positive while the CoM sat 57 mm outside).

D052 changes, measured by the checker: wave wag 1.5 -> 1.25 Hz (3.95 ->
3.29 rad/s on the yaw, free limit 4.0); shake 5.5 Hz x 6 mm -> 4 Hz x 4 mm
(0.51 g of CoM acceleration over mu 0.8 pads -> 0.18); turn_in_place and
sidestep go through WaveGait.budget() (the old wz 0.35 swung the coxa at
5.17 rad/s, past the servo's no-load 4.7) and end on a planted stance.

Registry: GESTURES2 = {name: (fn, total_s, narration)} — playground.py,
harness/backend.py and run_gestures2.py all read it.
"""
from __future__ import annotations
import numpy as np
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS, STATION_DEG
from pebble_gestures import _smooth, _planted_q, _lean_dir, CLAW_MAX
import rocky_model as _rm

_LO, _HI = _rm.joint_limits_rad()               # D052: from params, not literals
YAW_LIM = _HI[0]
HIP_LIM = (_LO[1], _HI[1])
KNEE_LIM = (_LO[2], _HI[2])


def _rotz(p, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]])


def posed(g, offset=(0, 0, 0), yaw=0.0, dz_leg=None, overrides=None,
          claw=None):
    """All-planted joint targets with the BODY displaced by `offset` (mm,
    body frame; z<0 = crouch), rotated by `yaw` (rad, feet fixed in the
    world), plus optional per-leg extra crouch dz_leg[5] (mm, positive =
    that corner lower) and joint overrides {leg: q}."""
    offset = np.asarray(offset, float)
    q = np.zeros((N_LEGS, 3))
    for i in range(N_LEGS):
        p = _rotz(g.p_nom[i], -yaw) - offset
        if dz_leg is not None:
            p = p + np.array([0, 0, dz_leg[i]])
        q[i] = leg_ik(body_to_leg(i, p))
    if overrides:
        for i, qi in overrides.items():
            q[i] = qi
    c = np.zeros(N_LEGS) if claw is None else np.asarray(claw, float)
    return q, c


def _blend(a, qa, qb):
    return (1 - a) * qa[0] + a * qb[0], (1 - a) * qa[1] + a * qb[1]


# ------------------------------------------------------------------ wave
WAVE_T = dict(lean=0.9, raise_=0.8, wave=2.4, lower=0.8, recenter=0.5)
WAVE_TOTAL = sum(WAVE_T.values())
WAVE_ARM = 0                                     # north — faces the friend
WAVE_HZ = 1.25                                   # D052: 1.5 Hz x 24 deg = 3.95 rad/s (free 4.0)
_Q_WAVE_HI = np.array([0.0, np.deg2rad(72), np.deg2rad(-48)])   # hand high


def wave(g: WaveGait, t: float):
    """Hello: north arm high, open hand, yaw wagging at 1.25 Hz."""
    t1 = WAVE_T["lean"]; t2 = t1 + WAVE_T["raise_"]
    t3 = t2 + WAVE_T["wave"]; t4 = t3 + WAVE_T["lower"]
    lean = _lean_dir((WAVE_ARM,)) * 13.0
    lean[2] = -8.0
    if t < t1:
        return posed(g, _smooth(t / t1) * lean)
    if t < t2:
        a = _smooth((t - t1) / (t2 - t1))
        q, c = posed(g, lean)
        q[WAVE_ARM] = (1 - a) * q[WAVE_ARM] + a * _Q_WAVE_HI
        c[WAVE_ARM] = a * 0.7 * CLAW_MAX
        return q, c
    if t < t3:
        tw = t - t2
        env = min(1.0, tw / 0.3, (t3 - t) / 0.3)          # ease in/out
        q, c = posed(g, lean)
        qa = _Q_WAVE_HI.copy()
        qa[0] = np.deg2rad(24) * env * np.sin(2 * np.pi * WAVE_HZ * tw)
        qa[2] += np.deg2rad(6) * env * np.sin(2 * np.pi * 2 * WAVE_HZ * tw)
        q[WAVE_ARM] = qa
        c[WAVE_ARM] = 0.7 * CLAW_MAX
        return q, c
    if t < t4:
        a = _smooth((t - t3) / (t4 - t3))
        q, c = posed(g, lean)
        q[WAVE_ARM] = (1 - a) * _Q_WAVE_HI + a * q[WAVE_ARM]
        c[WAVE_ARM] = (1 - a) * 0.7 * CLAW_MAX
        return q, c
    a = _smooth((t - t4) / WAVE_T["recenter"])
    return posed(g, (1 - a) * lean)


# ------------------------------------------------------------------- bow
BOW_T = dict(dip=1.1, hold=1.0, rise=1.1)
BOW_TOTAL = sum(BOW_T.values())
_BOW_DIP, _BOW_TILT, _BOW_FWD = 16.0, 14.0, 9.0   # mm: crouch, N-S pitch, lean


def _bow_pose(g, a):
    """a in 0..1: nominal -> full bow (north side down, body pitched)."""
    st = np.deg2rad(STATION_DEG)
    dz = a * _BOW_TILT * np.sin(st)              # north (+y) feet rise most
    off = np.array([0.0, a * _BOW_FWD, -a * _BOW_DIP])
    return posed(g, off, dz_leg=dz)


def bow(g: WaveGait, t: float):
    t1 = BOW_T["dip"]; t2 = t1 + BOW_T["hold"]
    if t < t1:
        return _bow_pose(g, _smooth(t / t1))
    if t < t2:
        return _bow_pose(g, 1.0)
    return _bow_pose(g, 1.0 - _smooth((t - t2) / BOW_T["rise"]))


# ------------------------------------------------------------ look-around
LOOK_T = dict(right=1.4, left=2.2, center=1.4)
LOOK_TOTAL = sum(LOOK_T.values())
_LOOK_DEG = 16.0                                 # body yaw; the coxa sees ~2x
                                                 # (feet at R185 about a coxa at R100):
                                                 # 16° body = 33° coxa, inside ±40


def look_around(g: WaveGait, t: float):
    """The rock turns to look (lidar is the eyes): yaw +16 -> -16 -> 0 on
    planted feet, with a small rise at each extreme."""
    t1 = LOOK_T["right"]; t2 = t1 + LOOK_T["left"]
    if t < t1:
        a = _smooth(t / t1)
        yaw = a * _LOOK_DEG
    elif t < t2:
        a = _smooth((t - t1) / (t2 - t1))
        yaw = (1 - 2 * a) * _LOOK_DEG
    else:
        a = _smooth((t - t2) / LOOK_T["center"])
        yaw = -(1 - a) * _LOOK_DEG
    lift = 5.0 * (abs(yaw) / _LOOK_DEG)          # perk up at the extremes
    return posed(g, (0, 0, lift), yaw=np.deg2rad(yaw))


# ----------------------------------------------------------------- shake
SHAKE_T = dict(shake=1.6, settle=0.5)
SHAKE_TOTAL = sum(SHAKE_T.values())


SHAKE_HZ, SHAKE_MM = 4.0, 4.0                    # D052: was 5.5 Hz x 6 mm (0.51 g on mu 0.8 pads)


def shake(g: WaveGait, t: float):
    """Wet-dog shake: 4 Hz lateral shimmy with a bob, feet planted."""
    if t < SHAKE_T["shake"]:
        env = max(0.0, min(1.0, t / 0.25, (SHAKE_T["shake"] - t) / 0.35))
        x = SHAKE_MM * env * np.sin(2 * np.pi * SHAKE_HZ * t)
        z = -4.0 * env * (1 - np.cos(2 * np.pi * SHAKE_HZ / 2 * t)) / 2
        return posed(g, (x, 0, z))
    return posed(g, (0, 0, 0))


# ------------------------------------------------------------------- sit
SIT_T = dict(down=1.3, hold=1.6, up=1.3)
SIT_TOTAL = sum(SIT_T.values())
_SIT_MM = 56.0                                   # 118 -> 62 mm body height


def sit(g: WaveGait, t: float):
    """Settle onto the belly skids: deep symmetric crouch, hold, stand."""
    t1 = SIT_T["down"]; t2 = t1 + SIT_T["hold"]
    if t < t1:
        a = _smooth(t / t1)
    elif t < t2:
        a = 1.0
    else:
        a = 1.0 - _smooth((t - t2) / SIT_T["up"])
    return posed(g, (0, 0, -a * _SIT_MM))


# ------------------------------------------------- turn-in-place, sidestep
TURN_TOTAL, SIDE_TOTAL = 4.8, 4.8
TURN_WZ, SIDE_VY = 0.35, 40.0                    # ASKED; WaveGait.budget() decides what runs
_PLANT_S = 0.5                                   # last 0.5 s: blend to the planted stance


def _gaited(g, t, total, cmd):
    """The wave gait at budget(cmd), ramped in/out over 0.6 s, and blended
    onto the all-planted stance over the last _PLANT_S so the gesture ENDS
    standing (zero command still steps in place — the old exit was a step
    from a lifted foot to the floor)."""
    if t >= total:
        return _planted_q(g, np.zeros(3)), np.zeros(N_LEGS)
    env = max(0.0, min(1.0, t / 0.6, (total - _PLANT_S - t) / 0.6))
    vx, vy, wz = g.budget(*cmd)
    q, _stance, _feet = g.joint_targets(t, vx * env, vy * env, wz * env)
    a = _smooth((t - (total - _PLANT_S)) / _PLANT_S)
    if a > 0:
        q = (1 - a) * q + a * _planted_q(g, np.zeros(3))
    return q, np.zeros(N_LEGS)


def turn_in_place(g: WaveGait, t: float):
    """Turn on the spot at min(0.35 rad/s, the budget) — 0.246 at the D052
    gait (the servo cannot swing the coxa any faster; the owner's derated
    physics reached ~0.3 of a commanded 0.5 anyway)."""
    return _gaited(g, t, TURN_TOTAL, (0.0, 0.0, TURN_WZ))


def sidestep(g: WaveGait, t: float):
    return _gaited(g, t, SIDE_TOTAL, (0.0, SIDE_VY, 0.0))


# D052 V2: tagged so a caller can tell a gesture that WALKS from one that stands
# (the sim2real locomotion hold covers these; a wrapper sets the same flag)
turn_in_place.gaited = sidestep.gaited = True


# ---------------------------------------------------------------- registry
# name: (fn, total_s, [(t_offset, narrator_event), ...])
GAITED = ("turn_in_place", "sidestep")           # pebble_feasibility kind="gait"
GESTURES2 = {
    "wave":          (wave, WAVE_TOTAL, [(0.9, "greet")]),
    "bow":           (bow, BOW_TOTAL, [(1.0, "resume")]),
    "look_around":   (look_around, LOOK_TOTAL, [(0.2, "question"), (3.0, "discovery")]),
    "shake":         (shake, SHAKE_TOTAL, [(1.5, "goal")]),
    "sit":           (sit, SIT_TOTAL, [(1.4, "sleep")]),
    "turn_in_place": (turn_in_place, TURN_TOTAL, [(0.1, "walk_start")]),
    "sidestep":      (sidestep, SIDE_TOTAL, [(0.1, "walk_start")]),
}


# ---------------------------------------------------------------- checks
def stability_margin(g, q, arm_legs=()):
    """Static margin (mm): distance from the whole-robot CoM's ground
    projection to the nearest edge of the support polygon of the planted
    feet (all legs not in arm_legs). Negative = outside the polygon.
    D052: delegates to pebble_feasibility.com_margin — torso + every leg
    segment through the FK chain (masses = sim/mass_budget.json, the same
    layout as the MJCF). It used to project the body ORIGIN, which is blind
    to the raised arm's own 244 g; `g` is kept for the old signature."""
    import pebble_feasibility as pf
    support = [i for i in range(N_LEGS) if i not in arm_legs]
    return pf.com_margin(q, support)


def kind_of(name):
    """pebble_feasibility kind for a registry entry."""
    return "gait" if name in GAITED else "static"


if __name__ == "__main__":
    import pebble_feasibility as pf
    g = WaveGait()
    fail = 0
    V = pf.speed_limits()
    for name, (fn, total, _ev) in GESTURES2.items():
        r = pf.check(fn, total, g=g, name=name, kind=kind_of(name))
        print("\n".join(r.lines))
        # D052 asserts (the old ones checked the raw ranges only; these are the guarded
        # ranges, the speed classes and the CoM margin — whatever the checker says)
        assert r.nan_count == 0, (name, "unreachable")
        assert not any(c.startswith("LIMIT") for c in r.fails), (name, "limit")
        assert np.all(r.vmax <= V["free"] + 1e-6), (name, "speed", r.peak())
        assert "SPEED_LOADED" not in r.fails and "JUMP" not in r.fails, (name, r.fails)
        if kind_of(name) == "static":
            arms = (WAVE_ARM,) if name == "wave" else ()
            m = min(stability_margin(g, fn(g, t)[0], arms) for t in np.linspace(0, total, 60))
            assert m > pf.MARGIN_FAIL_MM, (name, "margin", m)
        fail += not r.ok
    print(f"gesture library v2: {len(GESTURES2)} gestures, {fail} failing")
    assert fail == 0
