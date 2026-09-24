"""Adjacent-arm manipulation via foot repositioning (gait v0.3 feature).

D013 established that raising two ADJACENT limbs from nominal stance is
statically infeasible: the CoM sits ~57 mm outside the 3-leg support
triangle (the stance legs span only 144 deg of the circle). This module adds
the missing move: a choreographed FOOT-REPOSITIONING step that rebuilds the
support triangle around the CoM before the arms come up.

Geometry (worked out 2026-07-29, respects the CAD-validated coxa +/-40 deg):
- Crouch FIRST (h 118 -> 98 mm). Crouching extends radial reach
  (leg-frame max 148 -> ~214 mm), which is what makes the wide stance
  reachable at all.
- Step the two flanking stance legs out and around: body-angle +/-21 deg
  from station at R = 300 mm (D052 retune, see the constants; was 22 deg /
  280 mm with the coxa 1.1 deg from its stop; a straight swing at nominal
  radius would need ~44+ deg and is impossible).
- Support triangle after repositioning: flanking feet 170 deg apart around
  the "back" -- chord clears the CoM by ~23 mm on the critical edge, plus a
  small 5 mm lean => ~28 mm static margin (vs -57 mm without repositioning).
- Keep arm-phase body sway small (2.5 mm): the flanking coxas sit ~1.5 deg
  from their limit.

Choreography: stand -> crouch -> step flank A -> step flank B -> lean ->
raise adjacent arms + gripper work -> reverse everything.
"""
import numpy as np
from pebble_gait import (WaveGait, leg_ik, leg_to_body, body_to_leg,
                        arm_pose, claw_cycle, N_LEGS, STATION_DEG)

ARM_LEGS = (0, 1)                 # adjacent pair; any pair works by symmetry
CROUCH_MM = 30.0
LEAN_MM = 18.0                    # D052: 12 -> 18
SWAY_MM = 2.2
SWING_DEG = 21.0                  # flank feet move this far in body angle (D052: 22 -> 21)
R_WIDE = 300.0                    # ...at this body radius (D052: 280 -> 300)
# D052, measured by pebble_feasibility.check (CoM through the FK chain, not
# the body-origin proxy the numbers above were designed with): 22/280/12 put
# the flank coxas at 38.9 deg (inside the 2 deg guard of the 40 stop) with a
# 15.6 mm CoM margin at arms_down. A 48-point sweep of swing/radius/lean:
# 21/300/18 -> coxa <= 37.4 deg, CoM margin 17.5 mm (under the 25 mm comfort
# line — a WARNING, honestly: adjacent-arm work is the tightest thing Pebble
# does). The flank feet also used to hang 30 mm in the air at t=0 (the
# crouched home was used in every phase) and the work sway switched off
# with a step; both fixed in targets().
STEP_LIFT = 35.0                  # mm foot lift during repositioning steps


def arm_carry_pose(t, phase=0.0):
    """Folded 'carry' pose for ADJACENT-arm work: elbows tucked, hands high.

    v1 used the reach-out arm_pose() and the sim tipped the robot: two
    raised legs reaching outboard drag the whole-robot CoM ~35 mm toward
    the arm side — more than the entire support margin. Folding the limbs
    keeps their mass near the body axis. (Sim lesson, 2026-07-29.)
    """
    q1 = np.deg2rad(8) * np.sin(2 * np.pi * 0.16 * t + phase)
    q2 = np.deg2rad(72) + np.deg2rad(6) * np.sin(2 * np.pi * 0.28 * t + phase)
    q3 = np.deg2rad(-125) + np.deg2rad(8) * np.sin(2 * np.pi * 0.22 * t + 1.1 + phase)
    return np.array([q1, q2, q3])

# phase table: (name, duration s)
PHASES = [
    ("stand",       1.0),
    ("crouch",      1.2),
    ("step_a",      1.4),         # flank leg nearer arm pair CW side
    ("step_b",      1.4),
    ("lean",        1.0),
    ("arms_up",     1.6),
    ("work",        6.0),
    ("arms_down",   1.6),
    ("unlean",      1.0),
    ("unstep_b",    1.4),
    ("unstep_a",    1.4),
    ("uncrouch",    1.2),
    ("stand_end",   1.0),
]
T_TOTAL = sum(d for _, d in PHASES)


def smooth(s):
    s = np.clip(s, 0.0, 1.0)
    return s * s * (3 - 2 * s)


class AdjacentManip:
    def __init__(self, gait=None, arm_legs=ARM_LEGS):
        self.g = gait or WaveGait()
        self.arm_legs = tuple(arm_legs)
        self.stance_legs = [i for i in range(N_LEGS) if i not in self.arm_legs]
        # arm pair mean direction; lean points away from it
        a = np.deg2rad(STATION_DEG[list(self.arm_legs)])
        self.arm_dir = np.arctan2(np.sin(a).mean(), np.cos(a).mean())
        self.lean_dir = self.arm_dir + np.pi
        # flanking stance legs = the two adjacent to the arm pair;
        # middle stance leg stays put.
        mid = self.stance_legs[np.argmax([
            min((STATION_DEG[i] - np.rad2deg(self.lean_dir)) % 360,
                (np.rad2deg(self.lean_dir) - STATION_DEG[i]) % 360)
            for i in self.stance_legs]) if False else 0]
        # simpler: middle leg is the one whose station is closest to lean_dir
        ang_to_lean = [abs(((STATION_DEG[i] - np.rad2deg(self.lean_dir)) + 180) % 360 - 180)
                       for i in self.stance_legs]
        self.mid_leg = self.stance_legs[int(np.argmin(ang_to_lean))]
        self.flank = [i for i in self.stance_legs if i != self.mid_leg]
        # each flank swings TOWARD the arm pair (shrinking the >180deg gap)
        self.p_wide = {}
        for i in self.flank:
            st = STATION_DEG[i]
            d = ((self.arm_dir * 180 / np.pi - st) + 180) % 360 - 180   # signed angle to arm dir
            sgn = np.sign(d)
            ang = np.deg2rad(st + sgn * SWING_DEG)
            self.p_wide[i] = np.array([R_WIDE * np.cos(ang), R_WIDE * np.sin(ang),
                                       -(self.g.h - CROUCH_MM)])
        # step order: flank A = the one on the CW side (arbitrary but fixed)
        self.flank_a, self.flank_b = self.flank
        self.lean_vec = LEAN_MM * np.array([np.cos(self.lean_dir), np.sin(self.lean_dir), 0.0])

    # ---- piecewise timeline ----
    def _phase_at(self, t):
        acc = 0.0
        for name, dur in PHASES:
            if t < acc + dur:
                return name, (t - acc) / dur
            acc += dur
        return "stand_end", 1.0

    def foot_home(self, i, crouched):
        p = self.g.p_nom[i].copy()
        if crouched:
            p[2] = -(self.g.h - CROUCH_MM)
        return p

    def targets(self, t):
        """(q[5,3] joint targets, claw[5] 0..1, label, planted[5] bool)"""
        name, s = self._phase_at(t)
        ss = smooth(s)
        crouch_b = {"stand": 0, "crouch": ss, "uncrouch": 1 - ss, "stand_end": 0}.get(name, 1.0)
        lean_b = {"lean": ss, "unlean": 1 - ss}.get(
            name, 1.0 if name in ("arms_up", "work", "arms_down") else 0.0)
        arm_b = {"arms_up": ss, "arms_down": 1 - ss, "work": 1.0}.get(name, 0.0)
        wide_a = {"step_a": ss, "unstep_a": 1 - ss}.get(
            name, 1.0 if name in ("step_b", "lean", "arms_up", "work",
                                  "arms_down", "unlean", "unstep_b") else 0.0)
        wide_b = {"step_b": ss, "unstep_b": 1 - ss}.get(
            name, 1.0 if name in ("lean", "arms_up", "work", "arms_down",
                                  "unlean") else 0.0)

        crouched = crouch_b > 1e-9
        offset = self.lean_vec * lean_b
        offset = offset + np.array([0.0, 0.0, 0.0])       # crouch handled via foot z
        if name == "work":
            # D052: the sway is enveloped to 0 at both ends of `work` (it used to
            # switch off with a step when arms_down began: 6.5 rad/s on a knee)
            tw = s * PHASES[6][1]
            env = min(1.0, tw / 0.8, (PHASES[6][1] - tw) / 0.8)
            offset = offset + arm_b * env * np.array([
                SWAY_MM * np.sin(2 * np.pi * 0.18 * tw),
                SWAY_MM * (1 - np.cos(2 * np.pi * 0.18 * tw)),
                3.0 * np.sin(2 * np.pi * 0.30 * tw)])

        q = np.zeros((N_LEGS, 3))
        claw = np.zeros(N_LEGS)
        planted = np.ones(N_LEGS, dtype=bool)
        for i in range(N_LEGS):
            # where is this foot "home" right now?
            if i in self.flank:
                wb = wide_a if i == self.flank_a else wide_b
                # D052: the flank's home follows the crouch like every other
                # foot. It was foot_home(i, crouched=True) in EVERY phase, so at
                # t=0 both flank feet hung 30 mm in the air (a 22 deg knee step
                # at entry and exit). The wide foothold is only reached while
                # fully crouched, where the two agree.
                p_from = self.foot_home(i, crouched=False)
                p_from[2] = -(self.g.h - CROUCH_MM * crouch_b)
                p_to = self.p_wide[i].copy()
                p_to[2] = p_from[2]
                p = p_from + (p_to - p_from) * smooth(wb)
                stepping = (name in ("step_a", "unstep_a") and i == self.flank_a) or \
                           (name in ("step_b", "unstep_b") and i == self.flank_b)
                if stepping:
                    p[2] += STEP_LIFT * np.sin(np.pi * s)
                    planted[i] = False
            else:
                p = self.foot_home(i, crouched=False)
                p[2] = -(self.g.h - CROUCH_MM * crouch_b)
            if i in self.arm_legs and arm_b > 0:
                k = self.arm_legs.index(i)
                q_planted = leg_ik(body_to_leg(i, p - offset))
                q[i] = (1 - arm_b) * q_planted + arm_b * arm_carry_pose(t, phase=np.pi * k)
                claw[i] = arm_b * claw_cycle(t, phase=np.pi * k)
                planted[i] = arm_b < 0.5
            else:
                q[i] = leg_ik(body_to_leg(i, p - offset))
        return q, claw, name, planted


# ---------------------------------------------------------------- self-test
if __name__ == "__main__":
    m = AdjacentManip()
    print(f"arm pair {m.arm_legs} (stations {STATION_DEG[list(m.arm_legs)]})")
    print(f"mid leg {m.mid_leg}, flanks {m.flank}")
    for i in m.flank:
        ang = np.rad2deg(np.arctan2(m.p_wide[i][1], m.p_wide[i][0])) % 360
        print(f"  leg {i}: station {STATION_DEG[i]:.0f} -> foot at {ang:.1f} deg, R={R_WIDE}")
    # static margin of the wide triangle (ground plane, CoM ~ origin + lean)
    feet = [m.p_wide[m.flank_a][:2], m.p_wide[m.flank_b][:2],
            m.foot_home(m.mid_leg, True)[:2]]
    com = m.lean_vec[:2]
    def edge_margin(a, b, p):
        n = np.array([-(b - a)[1], (b - a)[0]]); n /= np.linalg.norm(n)
        c = np.mean(feet, axis=0)
        if np.dot(n, c - a) < 0: n = -n
        return np.dot(n, p - a)
    ms = [edge_margin(feet[i], feet[(i + 1) % 3], com) for i in range(3)]
    print(f"static support margin with lean: {min(ms):.1f} mm  (all edges: {[f'{x:.0f}' for x in ms]})")
    # sweep the whole timeline: NaN check + coxa limit
    worst_q1, bad = 0.0, 0
    for t in np.linspace(0, T_TOTAL, 2000):
        q, claw, name, planted = m.targets(t)
        if np.isnan(q).any():
            bad += 1
        stance_q1 = [abs(q[i, 0]) for i in range(N_LEGS)
                     if planted[i] and i not in m.arm_legs]
        worst_q1 = max(worst_q1, max(np.rad2deg(np.array(stance_q1))))
    print(f"timeline {T_TOTAL:.1f} s: NaN targets = {bad}, "
          f"max planted coxa |q1| = {worst_q1:.1f} deg (limit 40)")
    import pebble_feasibility as pf                       # D052: the full verdict
    from pebble_gestures import CLAW_MAX
    r = pf.check(lambda g, t: (m.targets(t)[0], m.targets(t)[1] * CLAW_MAX), T_TOTAL,
                 g=m.g, name="manip_adjacent")
    print("\n".join(r.lines))
    assert r.ok
