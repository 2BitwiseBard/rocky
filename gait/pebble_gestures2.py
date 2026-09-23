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
offsets (bow — the north side dips, the body pitches). The self-test
asserts joint ranges, no NaN, claw range AND a static-stability margin:
the body origin's projection must stay inside the planted-foot polygon.

Registry: GESTURES2 = {name: (fn, total_s, narration)} — playground.py,
harness/backend.py and run_gestures2.py all read it.
"""
from __future__ import annotations
import numpy as np
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS, STATION_DEG
from pebble_gestures import _smooth, _planted_q, _lean_dir, CLAW_MAX

YAW_LIM = np.deg2rad(40.0)
HIP_LIM = (np.deg2rad(-70.0), np.deg2rad(90.0))
KNEE_LIM = (np.deg2rad(-150.0), np.deg2rad(-20.0))


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
_Q_WAVE_HI = np.array([0.0, np.deg2rad(72), np.deg2rad(-48)])   # hand high


def wave(g: WaveGait, t: float):
    """Hello: north arm high, open hand, yaw wagging at 1.5 Hz."""
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
        qa[0] = np.deg2rad(24) * env * np.sin(2 * np.pi * 1.5 * tw)
        qa[2] += np.deg2rad(6) * env * np.sin(2 * np.pi * 3.0 * tw)
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


def shake(g: WaveGait, t: float):
    """Wet-dog shake: 5.5 Hz lateral shimmy with a bob, feet planted."""
    if t < SHAKE_T["shake"]:
        env = min(1.0, t / 0.25, (SHAKE_T["shake"] - t) / 0.35)
        x = 6.0 * env * np.sin(2 * np.pi * 5.5 * t)
        z = -4.0 * env * (1 - np.cos(2 * np.pi * 2.75 * t)) / 2
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


def turn_in_place(g: WaveGait, t: float):
    env = min(1.0, t / 0.6, (TURN_TOTAL - t) / 0.6) if t < TURN_TOTAL else 0.0
    q, _stance, _feet = g.joint_targets(t, 0.0, 0.0, 0.35 * env)
    return q, np.zeros(N_LEGS)


def sidestep(g: WaveGait, t: float):
    env = min(1.0, t / 0.6, (SIDE_TOTAL - t) / 0.6) if t < SIDE_TOTAL else 0.0
    q, _stance, _feet = g.joint_targets(t, 0.0, 40.0 * env, 0.0)
    return q, np.zeros(N_LEGS)


# ---------------------------------------------------------------- registry
# name: (fn, total_s, [(t_offset, narrator_event), ...])
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
    """Static margin (mm): distance from the body origin's ground
    projection to the nearest edge of the support polygon of the planted
    feet (all legs not in arm_legs). Negative = outside the polygon.
    Body-frame XY is used: for a level body that is the ground projection;
    for the bow's few degrees of pitch the error is < 1 mm."""
    from pebble_gait import leg_fk, leg_to_body
    feet = []
    for i in range(N_LEGS):
        if i in arm_legs:
            continue
        p = leg_to_body(i, leg_fk(q[i]))
        feet.append(p[:2])
    if len(feet) < 3:
        return -1e9
    pts = np.array(feet)
    c = pts.mean(axis=0)
    order = np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))
    pts = pts[order]
    # body origin projects to (0,0) in the body frame; edges of the polygon
    margin = 1e9
    n = len(pts)
    for k in range(n):
        a, b = pts[k], pts[(k + 1) % n]
        e = b - a
        nrm = np.array([e[1], -e[0]]) / (np.linalg.norm(e) + 1e-9)
        d = np.dot(-a, nrm)                      # signed distance of (0,0)
        # orient inward: centroid must be positive
        if np.dot(c - a, nrm) < 0:
            d = -d
        margin = min(margin, d)
    return margin


if __name__ == "__main__":
    g = WaveGait()
    worst = {}
    fail = 0
    for name, (fn, total, _ev) in GESTURES2.items():
        m_min, nan = 1e9, 0
        for t in np.linspace(0, total, 200):
            q, claw = fn(g, t)
            if np.isnan(q).any():
                nan += 1
                continue
            assert np.all(np.abs(q[:, 0]) <= YAW_LIM + 1e-6), (name, t, "yaw")
            assert np.all((q[:, 1] >= HIP_LIM[0] - 1e-6) & (q[:, 1] <= HIP_LIM[1] + 1e-6)), (name, t, "hip")
            assert np.all((q[:, 2] >= KNEE_LIM[0] - 1e-6) & (q[:, 2] <= KNEE_LIM[1] + 1e-6)), (name, t, "knee")
            assert claw.min() >= -1e-9 and claw.max() <= CLAW_MAX + 1e-9, (name, t, "claw")
            if name not in ("turn_in_place", "sidestep"):
                arms = (WAVE_ARM,) if name == "wave" else ()
                m_min = min(m_min, stability_margin(g, q, arms))
        worst[name] = (nan, m_min)
        ok = nan == 0 and (m_min > 15.0 or name in ("turn_in_place", "sidestep"))
        fail += not ok
        print(f"{name:14s} {total:4.1f} s  unreachable {nan:3d}/200  "
              f"static margin {m_min if m_min < 1e8 else float('nan'):6.1f} mm  "
              f"{'OK' if ok else 'FAIL'}")
    print(f"gesture library v2: {len(GESTURES2)} gestures, {fail} failing")
    assert fail == 0
