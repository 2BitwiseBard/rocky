"""Canon gestures — JAZZ HANDS and FIST-MY-BUMP (session 5b, by request).

Two scripted social behaviors from the source material, built on the same
stance choreography the manipulation work proved out (lean -> raise ->
work -> lower -> recenter, D013/D019):

jazz_hands   — non-adjacent two-arm stance (legs 0 & 2, the D019 default),
               both hands high and OPEN, claws fluttering at 3 Hz (D052;
               was 4.5) with a wrist shimmy and a light body bounce. AMAZE.
fist_bump    — one arm (leg 0), hand CLOSED into the cone (canon: the
               closed hand IS the fist), extended forward at friend
               height; holds for the bump. The sim harness detects the
               actual contact (SEA-microswitch stand-in) and pushes a
               10 mm "bump return" — on hardware this maps 1:1 to the
               foot switch. fist my bump.

Both return (q[5,3] rad, claw[5] rad) target streams; pure Python, no sim
imports — the same functions later feed the driver on the Pi.

D052 (the sim stops flattering the servo): every gesture here passes
pebble_feasibility.check — joint limits minus a 2 deg guard, 3.0 rad/s on a
loaded leg / 4.0 free / 4.7 never, claw <= 8 rad/s, no step at any phase
boundary, CoM margin >= 10 mm. What that took, measured by the checker:
  fist_bump  was authored in BODY space and held hip 96-112 / knee -163 (past
             both limits) for the whole hold, and stepped 14.8 rad/s at the
             raise because it blended from p_nom, not the leaned foot. Now
             joint space (Q_CARRY / Q_EXTEND) blended from the leaned pose.
  beckon     0.9 Hz x 80 deg knee curl = 5.92 rad/s -> 0.6 Hz x 70 deg, two
             whole curls so the arm ends exactly at Q_OUT.
  jazz_hands claw flutter 4.5 Hz x +-25 deg = 12.2 rad/s (SCS0009 ~9.5) and a
             23 deg claw step at raise->jazz -> 3 Hz x +-15 deg from the raise
             value; knee centre -30 -> -34 (its shimmy touched the -20 stop);
             the body bounce and claw now END where the next phase starts.
"""
from __future__ import annotations
import numpy as np
from pebble_gait import WaveGait, leg_ik, leg_fk, body_to_leg, N_LEGS, STATION_DEG
import rocky_model as _rm

CLAW_MAX = _rm.claw_limits("rad")[1]


def _smooth(a):
    a = np.clip(a, 0.0, 1.0)
    return a * a * (3 - 2 * a)


def _planted_q(g, offset):
    """All-planted joint targets with the body displaced by `offset`."""
    q = np.zeros((N_LEGS, 3))
    for i in range(N_LEGS):
        q[i] = leg_ik(body_to_leg(i, g.p_nom[i] - offset))
    return q


def _lean_dir(arm_legs):
    """Unit vector toward the stance-leg centroid (lean AWAY from arms)."""
    stance = [i for i in range(N_LEGS) if i not in arm_legs]
    a = np.deg2rad(STATION_DEG[stance])
    d = np.array([np.cos(a).mean(), np.sin(a).mean(), 0.0])
    return d / (np.linalg.norm(d) + 1e-9)


# ------------------------------------------------------------- jazz hands
JAZZ_T = dict(lean=1.0, raise_=0.9, jazz=3.6, lower=0.9, recenter=0.6)
JAZZ_TOTAL = sum(JAZZ_T.values())


JAZZ_CLAW_RAISE = 0.5          # open fraction the raise ends at = where the flutter starts
JAZZ_FLUTTER_HZ = 3.0          # D052: was 4.5 Hz x +-25 deg = 12.2 rad/s
JAZZ_FLUTTER_DEG = 15.0


def _jazz_claw(tj, k):
    """Claw angle (rad) during THE JAZZ: flutter about the raise value, ramped
    in over 0.25 s so the first sample IS the raise value (no step)."""
    env = min(1.0, tj / 0.25)
    return JAZZ_CLAW_RAISE * CLAW_MAX + np.deg2rad(JAZZ_FLUTTER_DEG) * env * \
        np.sin(2 * np.pi * JAZZ_FLUTTER_HZ * tj + 2.2 * k)


def _jazz_bounce(tj):
    """Body bounce (mm, z) — enveloped so it is 0 at both ends of the jazz."""
    T = JAZZ_T["jazz"]
    env = max(0.0, min(1.0, tj / 0.3, (T - tj) / 0.3))
    return -4.0 * env * (1 - np.cos(2 * np.pi * 1.3 * tj)) / 2


def jazz_hands(g: WaveGait, t: float, arm_legs=(0, 2)):
    """(q, claw) at time t of the jazz-hands routine."""
    t1 = JAZZ_T["lean"]
    t2 = t1 + JAZZ_T["raise_"]
    t3 = t2 + JAZZ_T["jazz"]
    t4 = t3 + JAZZ_T["lower"]
    lean_mm, crouch_mm = 14.0, 12.0
    lean = _lean_dir(arm_legs) * lean_mm
    lean[2] = -crouch_mm
    arms = np.isin(np.arange(N_LEGS), arm_legs)

    if t < t1:                                   # settle + lean
        a = _smooth(t / t1)
        return _planted_q(g, a * lean), np.zeros(N_LEGS)
    if t < t2:                                   # raise both arms
        a = _smooth((t - t1) / (t2 - t1))
        q = _planted_q(g, lean)
        for k, i in enumerate(arm_legs):
            hi = _jazz_arm_pose(0.0, k)
            q[i] = (1 - a) * q[i] + a * hi
        return q, a * JAZZ_CLAW_RAISE * CLAW_MAX * arms
    if t < t3:                                   # THE JAZZ
        tj = t - t2
        q = _planted_q(g, lean + np.array([0, 0, _jazz_bounce(tj)]))
        claw = np.zeros(N_LEGS)
        for k, i in enumerate(arm_legs):
            q[i] = _jazz_arm_pose(tj, k)
            claw[i] = _jazz_claw(tj, k)
        return q, claw
    if t < t4:                                   # lower (from exactly where the jazz ended)
        a = _smooth((t - t3) / (t4 - t3))
        q = _planted_q(g, lean)
        claw = np.zeros(N_LEGS)
        for k, i in enumerate(arm_legs):
            q[i] = (1 - a) * _jazz_arm_pose(JAZZ_T["jazz"], k) + a * q[i]
            claw[i] = (1 - a) * _jazz_claw(JAZZ_T["jazz"], k)
        return q, claw
    a = _smooth((t - t4) / JAZZ_T["recenter"])   # recenter
    return _planted_q(g, (1 - a) * lean), np.zeros(N_LEGS)


def _jazz_arm_pose(tj, k):
    """High spread-hand pose with a shimmy (leg frame joints). D052: knee
    centre -34 (was -30: -30 +- 10 rode the -20 stop), shimmy 8/6/9 deg at
    3.0/3.4/2.8 Hz (was 9/7/10 at 3.4/3.9/3.1: 3.4 rad/s, hot all 3.6 s)."""
    q1 = np.deg2rad(18 * (1 if k == 0 else -1)) \
        + np.deg2rad(8) * np.sin(2 * np.pi * 3.0 * tj + 1.7 * k)
    q2 = np.deg2rad(64) + np.deg2rad(6) * np.sin(2 * np.pi * 3.4 * tj + k)
    q3 = np.deg2rad(-34) + np.deg2rad(9) * np.sin(2 * np.pi * 2.8 * tj
                                                  + 0.8 + 1.3 * k)
    return np.array([q1, q2, q3])


# -------------------------------------------------------------- fist bump
BUMP_T = dict(lean=1.0, raise_=1.2, extend=0.7, hold=2.2, retract=0.8,
              lower=1.2, recenter=0.6)       # D052: raise/lower 1.0/0.8 -> 1.2 s (119 deg of hip)
BUMP_TOTAL = sum(BUMP_T.values())
BUMP_ARM = 0                     # leg 0, the "north" arm — faces the friend

# D052: JOINT-space poses, leg-0 frame (yaw, hip, knee), inside the guarded
# ranges. The old body-frame waypoints (0,205,45) / (0,235,55) IK'd to hip
# 96-112 / knee -163 — past both limits, so the real servo would have parked
# at the stops for the whole hold. Carry: upper arm high, forearm level,
# fist cocked at the friend (leg frame r 188, z 156 mm). Extend: the punch —
# forward 46 mm and down 17 (r 234, z 139). Both clear the |L2-L3| annulus.
Q_CARRY = np.deg2rad([0.0, 85.0, -85.0])
Q_EXTEND = np.deg2rad([0.0, 55.0, -55.0])
_BUMP_TRAVEL_MM = float(np.linalg.norm(leg_fk(Q_EXTEND) - leg_fk(Q_CARRY)))   # ~49 mm: the give's scale


def fist_bump(g: WaveGait, t: float, bump_mm: float = 0.0):
    """(q, claw) at time t. bump_mm: harness-supplied push-back along the
    arm axis after contact (the bump itself — 0 before contact). The give is
    a joint-space blend back toward Q_CARRY (bump_mm / ~49 of the way), so it
    can never leave the joint box."""
    t1 = BUMP_T["lean"]
    t2 = t1 + BUMP_T["raise_"]
    t3 = t2 + BUMP_T["extend"]
    t4 = t3 + BUMP_T["hold"]
    t5 = t4 + BUMP_T["retract"]
    t6 = t5 + BUMP_T["lower"]
    lean_mm, crouch_mm = 13.0, 10.0
    lean = _lean_dir((BUMP_ARM,)) * lean_mm
    lean[2] = -crouch_mm
    claw = np.zeros(N_LEGS)                      # claws CLOSED: cone fist
    q = _planted_q(g, lean) if t >= t1 else None
    if t < t1:
        a = _smooth(t / t1)
        return _planted_q(g, a * lean), claw
    q_leaned = q[BUMP_ARM].copy()                # the planted, LEANED foot: every blend
    if t < t2:                                   # raise to carry      starts/ends here
        a = _smooth((t - t1) / (t2 - t1))
        q[BUMP_ARM] = (1 - a) * q_leaned + a * Q_CARRY
        return q, claw
    if t < t3:                                   # extend the fist
        a = _smooth((t - t2) / (t3 - t2))
        q[BUMP_ARM] = (1 - a) * Q_CARRY + a * Q_EXTEND
        return q, claw
    if t < t4:                                   # HOLD (the bump happens)
        give = float(np.clip(bump_mm / _BUMP_TRAVEL_MM, 0.0, 1.0))
        q[BUMP_ARM] = (1 - give) * Q_EXTEND + give * Q_CARRY
        return q, claw
    if t < t5:                                   # retract to carry
        a = _smooth((t - t4) / (t5 - t4))
        q[BUMP_ARM] = (1 - a) * Q_EXTEND + a * Q_CARRY
        return q, claw
    if t < t6:                                   # lower onto the leaned foothold
        a = _smooth((t - t5) / (t6 - t5))
        q[BUMP_ARM] = (1 - a) * Q_CARRY + a * q_leaned
        return q, claw
    a = _smooth((t - t6) / BUMP_T["recenter"])
    return _planted_q(g, (1 - a) * lean), claw


# ---------------------------------------------------------------- beckon
# B18 (session 6): the "come here" — raise the north arm to carry, then
# three slow curls between the known-reachable carry and extend waypoints
# (the same pair fist_bump proved — both clear the |L2-L3| inner annulus),
# claw opening on the pull-in of each curl like a beckoning finger. Meant
# to play under a rising curious_question chord (run_beckon.py syncs the
# word onset to curl 2).
BECKON_HZ = 0.6                  # curl rate. D052: was 0.9 Hz x 80 deg = 5.92 rad/s of knee
BECKON_CURLS = 2                 # whole curls, so the arm is back at Q_OUT when `lower` starts
BECKON_T = dict(lean=1.0, raise_=1.0, beckon=BECKON_CURLS / BECKON_HZ, lower=0.9, recenter=0.6)
BECKON_TOTAL = sum(BECKON_T.values())
BECKON_ARM = BUMP_ARM            # leg 0 — the arm that faces the friend
# JOINT-SPACE poses (session-6 lesson: the carry/extend waypoints IK to hip
# 98-117 deg — past the +90 joint limit — so a position-space curl SATURATES
# at the limit and the arm barely moves). D052: Q_IN knee -115 -> -105: a 70
# deg curl at 0.6 Hz peaks 3.45 rad/s (1.5 pi f x curl), inside the free 4.0.
BECKON_Q_OUT = np.array([0.0, np.deg2rad(55), np.deg2rad(-35)])    # arm up-out
BECKON_Q_IN = np.array([0.0, np.deg2rad(75), np.deg2rad(-105)])    # curled in


def beckon(g: WaveGait, t: float):
    """(q, claw) at time t of the beckon routine."""
    t1 = BECKON_T["lean"]
    t2 = t1 + BECKON_T["raise_"]
    t3 = t2 + BECKON_T["beckon"]
    t4 = t3 + BECKON_T["lower"]
    lean_mm, crouch_mm = 13.0, 10.0
    lean = _lean_dir((BECKON_ARM,)) * lean_mm
    lean[2] = -crouch_mm
    claw = np.zeros(N_LEGS)

    Q_OUT, Q_IN = BECKON_Q_OUT, BECKON_Q_IN

    if t < t1:                                   # settle + lean
        a = _smooth(t / t1)
        return _planted_q(g, a * lean), claw
    if t < t2:                                   # raise to up-out
        a = _smooth((t - t1) / (t2 - t1))
        q = _planted_q(g, lean)
        q[BECKON_ARM] = (1 - a) * q[BECKON_ARM] + a * Q_OUT
        return q, claw
    if t < t3:                                   # THE CURLS (70 deg of knee)
        tb = t - t2
        c = 0.5 * (1 - np.cos(2 * np.pi * BECKON_HZ * tb))   # 0=out, 1=in
        c = c * c * (3 - 2 * c)                  # ease the reversal points
        q = _planted_q(g, lean)
        q[BECKON_ARM] = (1 - c) * Q_OUT + c * Q_IN
        # claw opens as the forearm curls IN — the beckoning finger
        claw[BECKON_ARM] = 0.6 * CLAW_MAX * c
        return q, claw
    if t < t4:                                   # lower
        a = _smooth((t - t3) / (t4 - t3))
        q = _planted_q(g, lean)
        q[BECKON_ARM] = (1 - a) * Q_OUT + a * q[BECKON_ARM]
        return q, claw
    a = _smooth((t - t4) / BECKON_T["recenter"])
    return _planted_q(g, (1 - a) * lean), claw


CANON = {"jazz_hands": (jazz_hands, JAZZ_TOTAL),
         "fist_bump": (fist_bump, BUMP_TOTAL),
         "beckon": (beckon, BECKON_TOTAL)}


# ------------------------------------------------------------- self-test
if __name__ == "__main__":
    import pebble_feasibility as pf
    g = WaveGait()
    fail = 0
    for name, (fn, total) in CANON.items():
        r = pf.check(fn, total, g=g, name=name)
        print("\n".join(r.lines))
        # D052 asserts: limits, speeds, no steps, standing — the checker's verdict
        assert np.all(r.vmax <= pf.speed_limits()["free"] + 1e-6), (name, "speed")
        assert r.claw_vmax <= pf.CLAW_MAX_RAD_S + 1e-6, (name, "claw speed")
        assert not any(c.startswith("LIMIT") for c in r.fails), (name, "limit")
        fail += not r.ok
    # the bump's give stays inside the joint box whatever the harness pushes
    for bump in (0.0, 10.0, 60.0):
        q, _ = fist_bump(g, 4.5, bump_mm=bump)
        assert pf.q_ok(q), bump
    q, _ = fist_bump(g, 4.0)
    print(f"extended-fist coxa angle: {np.rad2deg(q[BUMP_ARM][0]):+.1f} deg (limit +/-40)")
    print(f"canon gestures: {fail} failing")
    assert fail == 0
