"""Canon gestures — JAZZ HANDS and FIST-MY-BUMP (session 5b, by request).

Two scripted social behaviors from the source material, built on the same
stance choreography the manipulation work proved out (lean -> raise ->
work -> lower -> recenter, D013/D019):

jazz_hands   — non-adjacent two-arm stance (legs 0 & 2, the D019 default),
               both hands high and OPEN, claws fluttering at ~4.5 Hz with
               fast wrist shimmy and a light body bounce. AMAZE.
fist_bump    — one arm (leg 0), hand CLOSED into the cone (canon: the
               closed hand IS the fist), extended forward at friend
               height; holds for the bump. The sim harness detects the
               actual contact (SEA-microswitch stand-in) and pushes a
               10 mm "bump return" — on hardware this maps 1:1 to the
               foot switch. fist my bump.

Both return (q[5,3] rad, claw[5] rad) target streams; pure Python, no sim
imports — the same functions later feed the driver on the Pi.
"""
from __future__ import annotations
import numpy as np
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS, STATION_DEG

CLAW_MAX = np.deg2rad(55)


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


def jazz_hands(g: WaveGait, t: float, arm_legs=(0, 2)):
    """(q, claw) at time t of the jazz-hands routine."""
    t1 = JAZZ_T["lean"]
    t2 = t1 + JAZZ_T["raise_"]
    t3 = t2 + JAZZ_T["jazz"]
    t4 = t3 + JAZZ_T["lower"]
    lean_mm, crouch_mm = 14.0, 12.0
    lean = _lean_dir(arm_legs) * lean_mm
    lean[2] = -crouch_mm

    if t < t1:                                   # settle + lean
        a = _smooth(t / t1)
        return _planted_q(g, a * lean), np.zeros(N_LEGS)
    if t < t2:                                   # raise both arms
        a = _smooth((t - t1) / (t2 - t1))
        q = _planted_q(g, lean)
        for k, i in enumerate(arm_legs):
            hi = _jazz_arm_pose(0.0, k)
            q[i] = (1 - a) * q[i] + a * hi
        return q, a * 0.5 * CLAW_MAX * np.ones(N_LEGS) * \
            np.isin(np.arange(N_LEGS), arm_legs)
    if t < t3:                                   # THE JAZZ
        tj = t - t2
        bounce = np.array([0, 0, -4.0 * (1 - np.cos(2 * np.pi * 1.3 * tj)) / 2])
        q = _planted_q(g, lean + bounce)
        claw = np.zeros(N_LEGS)
        for k, i in enumerate(arm_legs):
            q[i] = _jazz_arm_pose(tj, k)
            claw[i] = (0.55 + 0.45 * np.sin(2 * np.pi * 4.5 * tj + 2.2 * k)) \
                * CLAW_MAX
        return q, claw
    if t < t4:                                   # lower
        a = _smooth((t - t3) / (t4 - t3))
        q_hold = _planted_q(g, lean)
        q = _planted_q(g, lean)
        for k, i in enumerate(arm_legs):
            q[i] = (1 - a) * _jazz_arm_pose(JAZZ_T["jazz"], k) + a * q_hold[i]
        return q, (1 - a) * 0.4 * CLAW_MAX * \
            np.isin(np.arange(N_LEGS), arm_legs)
    a = _smooth((t - t4) / JAZZ_T["recenter"])   # recenter
    return _planted_q(g, (1 - a) * lean), np.zeros(N_LEGS)


def _jazz_arm_pose(tj, k):
    """High spread-hand pose with fast shimmy (leg frame joints)."""
    q1 = np.deg2rad(18 * (1 if k == 0 else -1)) \
        + np.deg2rad(9) * np.sin(2 * np.pi * 3.4 * tj + 1.7 * k)
    q2 = np.deg2rad(64) + np.deg2rad(7) * np.sin(2 * np.pi * 3.9 * tj + k)
    q3 = np.deg2rad(-30) + np.deg2rad(10) * np.sin(2 * np.pi * 3.1 * tj
                                                   + 0.8 + 1.3 * k)
    return np.array([q1, q2, q3])


# -------------------------------------------------------------- fist bump
BUMP_T = dict(lean=1.0, raise_=1.0, extend=0.7, hold=2.2, retract=0.8,
              lower=0.8, recenter=0.6)
BUMP_TOTAL = sum(BUMP_T.values())
BUMP_ARM = 0                     # leg 0, the "north" arm — faces the friend

# fist waypoints in BODY frame (mm): carry -> extended (out along +y).
# Mind the INNER reach limit too: |L2-L3| = 40 mm around the hip axis —
# the first carry pose sat 28.7 mm from it (unreachable from the inside).
_CARRY = np.array([0.0, 205.0, 45.0])
_EXTEND = np.array([0.0, 235.0, 55.0])


def fist_bump(g: WaveGait, t: float, bump_mm: float = 0.0):
    """(q, claw) at time t. bump_mm: harness-supplied push-back along the
    arm axis after contact (the bump itself — 0 before contact)."""
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

    def arm_q(p_body):
        return leg_ik(body_to_leg(BUMP_ARM, p_body))

    if t < t1:
        a = _smooth(t / t1)
        return _planted_q(g, a * lean), claw
    if t < t2:                                   # raise to carry
        a = _smooth((t - t1) / (t2 - t1))
        q = _planted_q(g, lean)
        p = (1 - a) * (g.p_nom[BUMP_ARM]) + a * _CARRY
        q[BUMP_ARM] = arm_q(p)
        return q, claw
    if t < t3:                                   # extend the fist
        a = _smooth((t - t2) / (t3 - t2))
        q = _planted_q(g, lean)
        q[BUMP_ARM] = arm_q((1 - a) * _CARRY + a * _EXTEND)
        return q, claw
    if t < t4:                                   # HOLD (the bump happens)
        q = _planted_q(g, lean)
        p = _EXTEND.copy()
        p[1] -= bump_mm                          # give with the contact
        q[BUMP_ARM] = arm_q(p)
        return q, claw
    if t < t5:                                   # retract to carry
        a = _smooth((t - t4) / (t5 - t4))
        q = _planted_q(g, lean)
        q[BUMP_ARM] = arm_q((1 - a) * _EXTEND + a * _CARRY)
        return q, claw
    if t < t6:                                   # lower
        a = _smooth((t - t5) / (t6 - t5))
        q = _planted_q(g, lean)
        p = (1 - a) * _CARRY + a * g.p_nom[BUMP_ARM]
        q[BUMP_ARM] = arm_q(p)
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
BECKON_T = dict(lean=1.0, raise_=1.0, beckon=3.3, lower=0.9, recenter=0.6)
BECKON_TOTAL = sum(BECKON_T.values())
BECKON_ARM = BUMP_ARM            # leg 0 — the arm that faces the friend
BECKON_HZ = 0.9                  # curl rate: slow enough to read as a wave


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

    # JOINT-SPACE poses (session-6 lesson: the carry/extend waypoints IK to
    # hip 98-117 deg — past the +90 joint limit — so a position-space curl
    # SATURATES at the limit and the arm barely moves. The jazz arm had it
    # right: author expressive poses directly in joint space, inside the
    # real ranges hip [-70,90] / knee [-150,-20].)
    Q_OUT = np.array([0.0, np.deg2rad(55), np.deg2rad(-35)])    # arm up-out
    Q_IN = np.array([0.0, np.deg2rad(75), np.deg2rad(-115)])    # curled in

    if t < t1:                                   # settle + lean
        a = _smooth(t / t1)
        return _planted_q(g, a * lean), claw
    if t < t2:                                   # raise to up-out
        a = _smooth((t - t1) / (t2 - t1))
        q = _planted_q(g, lean)
        q[BECKON_ARM] = (1 - a) * q[BECKON_ARM] + a * Q_OUT
        return q, claw
    if t < t3:                                   # THE CURLS (80 deg of knee)
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


# ------------------------------------------------------------- self-test
if __name__ == "__main__":
    g = WaveGait()
    bad = 0
    for t in np.linspace(0, JAZZ_TOTAL, 240):
        q, claw = jazz_hands(g, t)
        bad += int(np.isnan(q).any())
        assert claw.max() <= CLAW_MAX + 1e-9
    for t in np.linspace(0, BUMP_TOTAL, 240):
        q, claw = fist_bump(g, t, bump_mm=10.0 if 3.5 < t < 4.5 else 0.0)
        bad += int(np.isnan(q).any())
    for t in np.linspace(0, BECKON_TOTAL, 240):
        q, claw = beckon(g, t)
        bad += int(np.isnan(q).any())
        assert claw.max() <= CLAW_MAX + 1e-9
    print(f"gesture streams: {bad} unreachable targets over 720 samples "
          f"({'PASS' if bad == 0 else 'FAIL — retune waypoints'})")
    q, _ = fist_bump(g, 3.2)
    print(f"extended-fist coxa angle: {np.rad2deg(q[BUMP_ARM][0]):+.1f} deg "
          f"(limit +/-40)")
    assert bad == 0
