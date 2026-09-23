"""Pebble gait engine v0 — closed-form IK + omnidirectional 5-phase wave gait.

Pure Python/numpy, no ROS dependency: this exact module later becomes the core
of the ros2_control gait node, and drives MuJoCo in simulation. Frames match
the CAD exactly (params.yaml is shared).

Conventions
-----------
BODY frame: origin at pentagon center on the DECK-TOP plane, +Z up.
Leg i station: angle a_i = 90 + 72*i deg (leg 0 points "north"), at radius R_BODY.
LEG frame (matches CAD leg-local): origin at the yaw axis on the deck plane,
+X radial outward, +Z up. Femur pivot: (L1, 0, +58). Foot = tip of tibia.

Joints per leg: q1 yaw (coxa), q2 femur pitch (0 = horizontal, + = up),
q3 knee pitch (0 = straight with femur, - = knee down). Neutral: (0, 0, -90deg).

Servo mapping (Phase 1, for reference):
ticks = 2048 + q * 4096 / (2*pi) * DIR[j] + OFFSET[j]   (calibrated per joint)
"""
import numpy as np
import os, yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(_HERE, "..", "cad", "params.yaml")) as f:
    _P = yaml.safe_load(f)

L1 = _P["leg"]["l1_coxa"]
L2 = _P["leg"]["l2_femur"]
L3 = _P["leg"]["l3_tibia"]
R_BODY = _P["body"]["circumradius"]
Z_HIP = float(_P["leg"]["hip_axis_z"])   # femur pivot height above the deck plane (params SSOT, D047)
N_LEGS = 5
STATION_DEG = 90 + 72 * np.arange(N_LEGS)   # leg 0 north, CCW

# ---------------------------------------------------------------- kinematics
def leg_fk(q):
    """Joint angles (q1,q2,q3) -> foot position in LEG frame."""
    q1, q2, q3 = q
    r = L1 + L2*np.cos(q2) + L3*np.cos(q2 + q3)
    z = Z_HIP + L2*np.sin(q2) + L3*np.sin(q2 + q3)
    return np.array([r*np.cos(q1), r*np.sin(q1), z])

def leg_ik(p):
    """Foot position in LEG frame -> (q1,q2,q3). Knee-down branch. NaN if unreachable."""
    x, y, z = p
    q1 = np.arctan2(y, x)
    dx = np.hypot(x, y) - L1
    dz = z - Z_HIP
    d2 = dx*dx + dz*dz
    c3 = (d2 - L2*L2 - L3*L3) / (2*L2*L3)
    if abs(c3) > 1.0:
        return np.array([np.nan]*3)
    q3 = -np.arccos(c3)                                   # knee down
    q2 = np.arctan2(dz, dx) - np.arctan2(L3*np.sin(q3), L2 + L3*np.cos(q3))
    return np.array([q1, q2, q3])

def body_to_leg(i, p_body):
    """Point in BODY frame -> LEG-i frame."""
    a = np.deg2rad(STATION_DEG[i])
    c, s = np.cos(a), np.sin(a)
    st = np.array([R_BODY*c, R_BODY*s, 0.0])
    d = p_body - st
    return np.array([ c*d[0] + s*d[1], -s*d[0] + c*d[1], d[2]])

def leg_to_body(i, p_leg):
    a = np.deg2rad(STATION_DEG[i])
    c, s = np.cos(a), np.sin(a)
    st = np.array([R_BODY*c, R_BODY*s, 0.0])
    return st + np.array([c*p_leg[0] - s*p_leg[1], s*p_leg[0] + c*p_leg[1], p_leg[2]])

# ---------------------------------------------------------------- wave gait
class WaveGait:
    """Omnidirectional 5-phase wave gait (duty 0.8: always 4 feet down).

    Command: vx, vy (mm/s, BODY frame), wz (rad/s). Radial symmetry means any
    heading is equivalent - 'forward' is just a command angle.
    """
    def __init__(self, body_height=118.0, stance_radius=185.0,
                 cycle_time=1.6, duty=0.8, step_height=32.0):
        # defaults tuned so the coxa stays inside the CAD-validated +/-40 deg
        # sweep at 45 mm/s: stride = v*duty*T = 57.6 mm at leg-frame radius 75
        self.h = body_height              # deck plane above ground
        self.R0 = stance_radius           # nominal foot circle radius (BODY frame)
        self.T = cycle_time
        self.duty = duty
        self.hstep = step_height
        self.phase_off = np.arange(N_LEGS)[::-1] / N_LEGS   # wave order 0,4,3,2,1
        # nominal foothold, BODY frame, per leg
        a = np.deg2rad(STATION_DEG)
        self.p_nom = np.stack([self.R0*np.cos(a), self.R0*np.sin(a),
                               -self.h*np.ones(N_LEGS)], axis=1)

    def foot_targets(self, t, vx, vy, wz):
        """BODY-frame foot targets for all legs at time t."""
        v = np.array([vx, vy, 0.0])
        T_st = self.duty * self.T
        out = np.zeros((N_LEGS, 3))
        stance = np.zeros(N_LEGS, dtype=bool)
        for i in range(N_LEGS):
            ph = (t / self.T + self.phase_off[i]) % 1.0
            pn = self.p_nom[i]
            wxr = np.cross(np.array([0, 0, wz]), pn)      # rotation-induced velocity
            vf = v + wxr                                   # body-frame velocity of ground
            if ph < self.duty:                             # STANCE: sweep with the ground
                s = ph / self.duty - 0.5                   # -0.5 .. +0.5
                out[i] = pn - vf * (s * T_st)
                stance[i] = True
            else:                                          # SWING: fly to next touchdown
                s = (ph - self.duty) / (1 - self.duty)     # 0..1
                s_smooth = s*s*(3 - 2*s)                   # smoothstep for xy
                # continuity: stance ENDS at pn - vf*Tst/2 (behind), so swing
                # lifts there and flies FORWARD to pn + vf*Tst/2 where the next
                # stance begins. (Reversed signs here = the walking-in-place bug
                # that only physics simulation caught, 2026-07-28.)
                p_lift = pn - vf * (0.5 * T_st)
                p_land = pn + vf * (0.5 * T_st)
                out[i] = p_lift + (p_land - p_lift) * s_smooth
                out[i, 2] += self.hstep * np.sin(np.pi * s)
                stance[i] = False
        return out, stance

    def joint_targets(self, t, vx, vy, wz):
        """All 15 joint angles at time t. Returns (angles[5,3], stance[5], feet_body[5,3])."""
        feet, stance = self.foot_targets(t, vx, vy, wz)
        q = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            q[i] = leg_ik(body_to_leg(i, feet[i]))
        return q, stance, feet

# ---------------------------------------------------------------- arm modes
def arm_pose(t, phase=0.0):
    """A raised limb's joint targets (leg frame): reach forward and gesture."""
    q1 = np.deg2rad(14) * np.sin(2*np.pi*0.20*t + phase)
    q2 = np.deg2rad(52) + np.deg2rad(10) * np.sin(2*np.pi*0.33*t + phase)
    q3 = np.deg2rad(-42) + np.deg2rad(16) * np.sin(2*np.pi*0.26*t + 1.3 + phase)
    return np.array([q1, q2, q3])

def claw_cycle(t, phase=0.0):
    """Gripper open/close command in [0,1]."""
    return 0.5 + 0.5*np.sin(2*np.pi*0.3*t + phase)

class ArmedGait(WaveGait):
    """Wave gait on a SUBSET of legs; the rest are raised as arms.

    4 active legs -> duty 0.78 (always >=3 feet down): Rocky walking while
    holding something up. Arm legs get arm_pose() targets from the caller.
    """
    def __init__(self, arm_legs=(0,), body_shift_mm=16.0, **kw):
        super().__init__(**kw)
        self.arm_legs = tuple(arm_legs)
        self.active = [i for i in range(N_LEGS) if i not in self.arm_legs]
        n = len(self.active)
        if n == 4:
            self.duty = 0.78
        self._phase = {leg: (n - 1 - j) / n for j, leg in enumerate(self.active)}
        # lean the body AWAY from the raised limb(s), toward the stance centroid
        # (physics lesson: without this, the CoM leaves the support polygon)
        a = np.deg2rad(STATION_DEG[list(self.arm_legs)])
        away = -np.array([np.cos(a).mean(), np.sin(a).mean(), 0.0])
        away /= (np.linalg.norm(away) + 1e-9)
        self.body_shift = away * body_shift_mm

    def foot_targets(self, t, vx, vy, wz):
        v = np.array([vx, vy, 0.0])
        T_st = self.duty * self.T
        out = np.zeros((N_LEGS, 3))
        stance = np.zeros(N_LEGS, dtype=bool)
        for i in self.active:
            ph = (t / self.T + self._phase[i]) % 1.0
            pn = self.p_nom[i]
            vf = v + np.cross(np.array([0, 0, wz]), pn)
            if ph < self.duty:
                s = ph / self.duty - 0.5
                out[i] = pn - vf * (s * T_st)
                stance[i] = True
            else:
                s = (ph - self.duty) / (1 - self.duty)
                sm = s*s*(3 - 2*s)
                p_lift = pn - vf * (0.5 * T_st)
                p_land = pn + vf * (0.5 * T_st)
                out[i] = p_lift + (p_land - p_lift) * sm
                out[i, 2] += self.hstep * np.sin(np.pi * s)
            out[i] = out[i] - self.body_shift        # body leans toward stance side
        return out, stance

    def joint_targets(self, t, vx, vy, wz):
        feet, stance = self.foot_targets(t, vx, vy, wz)
        q = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            if i in self.arm_legs:
                q[i] = arm_pose(t, phase=0.9*i)
            else:
                q[i] = leg_ik(body_to_leg(i, feet[i]))
        return q, stance, feet

def stance_manip_targets(g, t, arm_legs=(0, 2), lean_blend=1.0, arm_blend=1.0,
                         lean_mm=12.0, sway_mm=5.0, crouch_mm=10.0):
    """3-leg stance manipulation with a weight shift + slight crouch.

    Physics lessons from the sim (2026-07-28): with ADJACENT arms raised the
    CoM sits ~57 mm outside the support triangle — an infeasible lean at this
    stance radius. NON-ADJACENT arms (default 0 & 2) leave a wide tripod with
    ~57 mm of margin, needing only a small comfort lean + crouch. lean_blend
    ramps the shift with all feet planted; arm_blend raises arms and starts
    sway + gripper cycles. Returns (q[5,3], claw[5] in 0..1)."""
    arm_legs = tuple(arm_legs)
    stance = [i for i in range(N_LEGS) if i not in arm_legs]
    a = np.deg2rad(STATION_DEG[stance])
    lean_dir = np.array([np.cos(a).mean(), np.sin(a).mean(), 0.0])
    lean_dir /= (np.linalg.norm(lean_dir) + 1e-9)
    lean = lean_dir * lean_mm * lean_blend
    lean[2] = -crouch_mm * lean_blend          # crouch: body drops, easing reach
    sway = arm_blend * np.array([sway_mm*np.cos(2*np.pi*0.20*t),
                                 sway_mm*np.sin(2*np.pi*0.20*t),
                                 4.0*np.sin(2*np.pi*0.35*t)])
    offset = lean + sway                       # desired body displacement
    q = np.zeros((N_LEGS, 3))
    claw = np.zeros(N_LEGS)
    for i in range(N_LEGS):
        q_planted = leg_ik(body_to_leg(i, g.p_nom[i] - offset))
        if i in arm_legs and arm_blend > 0:
            k = arm_legs.index(i)
            q[i] = (1 - arm_blend)*q_planted + arm_blend*arm_pose(t, phase=np.pi*k)
            claw[i] = arm_blend * claw_cycle(t, phase=np.pi*k)
        else:
            q[i] = q_planted
    return q, claw

# ---------------------------------------------------------------- self-test
if __name__ == "__main__":
    rng = np.random.default_rng(0)
    # FK/IK roundtrip over the useful workspace
    errs = []
    for _ in range(2000):
        q = np.array([rng.uniform(-0.7, 0.7),
                      rng.uniform(-0.6, 0.9),
                      rng.uniform(-2.4, -0.5)])
        q2 = leg_ik(leg_fk(q))
        errs.append(np.max(np.abs(leg_fk(q2) - leg_fk(q))))
    print(f"IK/FK roundtrip: max foot error {max(errs):.2e} mm over 2000 samples")

    g = WaveGait()
    ts = np.linspace(0, g.T, 401)
    min_stance, qmax = 5, np.zeros(3)
    for t in ts:
        q, st, _ = g.joint_targets(t, 45.0, 0.0, 0.0)
        assert not np.isnan(q).any(), "unreachable target!"
        min_stance = min(min_stance, st.sum())
        qmax = np.maximum(qmax, np.abs(q).max(axis=0))
    print(f"walk @45mm/s: min feet in stance = {min_stance} (target >= 4)")
    print(f"max |q1,q2,q3| = {np.rad2deg(qmax).round(1)} deg  "
          f"(coxa limit +/-40, servo range +/-180)")
    for cmd, name in [((0, 45, 0), "strafe"), ((0, 0, 0.5), "turn-in-place"),
                      ((45, 45, 0.3), "arc")]:
        ok = True
        for t in ts:
            q, st, _ = g.joint_targets(t, *cmd)
            ok &= not np.isnan(q).any()
        print(f"{name}: all targets reachable = {ok}")
