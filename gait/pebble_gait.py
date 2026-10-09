"""Pebble gait engine v0 — closed-form IK + omnidirectional 5-phase wave gait.

Pure Python/numpy, no ROS dependency: this exact module later becomes the core
of the ros2_control gait node, and drives MuJoCo in simulation. Frames match
the CAD exactly (params.yaml is shared).

Conventions
-----------
BODY frame: origin at pentagon center on the DECK-TOP plane, +Z up.
Leg i station: angle a_i = 90 + 72*i deg (leg 0 points "north"), at radius R_BODY.
LEG frame (matches CAD leg-local): origin at the yaw axis on the deck plane,
+X radial outward, +Z up. Femur pivot: (L1, 0, Z_HIP) — params
leg.hip_axis_z. Foot = tip of tibia.

Joints per leg: q1 yaw (coxa), q2 femur pitch (0 = horizontal, + = up),
q3 knee pitch (0 = straight with femur, - = knee down). Calibration/jig pose:
(0, 0, -90 deg); the standing stance is leg_ik(p_nom), about (0, -34, -77) deg.

Servo mapping (Phase 1, for reference):
ticks = 2048 + q * 4096 / (2*pi) * DIR[j] + OFFSET[j]   (calibrated per joint)
"""
import numpy as np
import os, sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:                  # rocky_model lives next to this file
    sys.path.insert(0, _HERE)
import rocky_model as _rm                  # noqa: E402  (D052: the one params loader)

_P = _rm.params()

L1 = _P["leg"]["l1_coxa"]
L2 = _P["leg"]["l2_femur"]
L3 = _P["leg"]["l3_tibia"]
R_BODY = _P["body"]["circumradius"]
Z_HIP = float(_P["leg"]["hip_axis_z"])   # femur pivot height above the deck plane (params SSOT, D047)
N_LEGS = _rm.n_legs()                      # D053: params robot.legs (5)
STATION_DEG = np.array(_rm.stations_deg())  # leg 0 north, CCW: 90 + 72 i, unwrapped (D053: from the spec)
SUPPORT_TOL_MM = 15.0      # D052: a foot this close to the ground counts as LOADED (speed class)

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

def _leg_ik_rows(P):
    """leg_ik over the rows of P (n, 3) at once -> (n, 3), NaN rows where unreachable.
    The same algebra, vectorised for the D065 leveled-plane envelope (budget(level_slope=)
    bisects 26 swings per slope step); leg_ik stays the reference everywhere else."""
    P = np.asarray(P, float)
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    q1 = np.arctan2(y, x)
    dx = np.hypot(x, y) - L1
    dz = z - Z_HIP
    c3 = (dx*dx + dz*dz - L2*L2 - L3*L3) / (2*L2*L3)
    q3 = -np.arccos(np.clip(c3, -1.0, 1.0))
    q2 = np.arctan2(dz, dx) - np.arctan2(L3*np.sin(q3), L2 + L3*np.cos(q3))
    Q = np.stack([q1, q2, q3], axis=1)
    Q[np.abs(c3) > 1.0] = np.nan
    return Q

def smooth01(x):
    """Smoothstep of x clipped to [0, 1] (float or array)."""
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)

def ik_ok(q, guard_deg=0.0):
    """True if q (3,) or (5,3) is finite and inside the soft joint limits
    (params joints.pos_deg) shrunk by guard_deg. leg_ik answers geometry
    only — a finite answer can still be past a limit (D052: the studio's
    yaw slider at 25 deg asked the coxa for 53.6). pebble_feasibility uses
    a 2 deg guard; the driver clamps at the limit itself."""
    q = np.atleast_2d(np.asarray(q, float))
    if q.shape[-1] != 3 or not np.isfinite(q).all():
        return False
    lo, hi = _rm.joint_limits_rad()
    g = np.deg2rad(guard_deg)
    return bool(np.all(q >= np.array(lo) + g - 1e-9) and np.all(q <= np.array(hi) - g + 1e-9))

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

# ---------------------------------------------------------------- swing (B76)
def swing_profile(s, duty):
    """The soft-landing swing (D063, B76 fix 2) at swing progress s in [0, 1]
    (float or array): (xy, z) as fractions of the stride (lift-off 0 ->
    touchdown 1) and of the step height.

    xy: a cubic Hermite whose slope at BOTH ends is -k, k = (1-duty)/duty —
        the stance foot's own velocity in swing time — so the foot neither
        brakes at lift-off nor lands moving. Peak slope 1.5 + 0.5 k times the
        mean (1.625 at duty 0.8; the old smoothstep's was 1.5, but it started
        and ended at rest while stance moves at -v_f).
    z:  a smoothstep up to the apex at s = 0.5 and a mirrored one down: zero
        vertical velocity at lift-off, apex and touchdown. Peak 3 h / T_sw.
    The old h sin(pi s) left and landed at pi h / T_sw (188 mm/s): joint
    targets stepped 2.7-3.1 rad/s at every lift-off and touchdown. Measured
    against the D052 budget (h 24 / T 2.0): this pair keeps a 34.2 mm/s
    envelope; h sin^2(pi s) 29.8; the C2 h 64 s^3 (1-s)^3 only 18.7 — its
    vertical peak (3.44 h / T_sw) lands inside the 15 mm band where a joint
    is held to the LOADED 3.0 rad/s."""
    k = (1.0 - duty) / duty
    xy = (1.0 + k) * s * s * (3.0 - 2.0 * s) - k * s
    x = 1.0 - np.abs(2.0 * s - 1.0)
    return xy, x * x * (3.0 - 2.0 * x)

def swing_xy_peak(duty):
    """max d(xy)/ds of swing_profile (at s = 0.5): 1.5 (1 + k) - k."""
    return 1.5 + 0.5 * (1.0 - duty) / duty

# ---------------------------------------------------------------- wave gait
class WaveGait:
    """Omnidirectional 5-phase wave gait (duty 0.8: always 4 feet down).

    Command: vx, vy (mm/s, BODY frame), wz (rad/s). Radial symmetry means any
    heading is equivalent - 'forward' is just a command angle.

    Defaults come from params.yaml `gait:` (rocky_model.gait_defaults()); any
    kwarg overrides. D052 retune (checked by pebble_feasibility.check_gait,
    free leg 4.0 / loaded 3.0 rad/s): T 1.6 -> 2.0 s and step 32 -> 24 mm,
    duty stays 0.8. At (45, 0, 0) the old gait asked the knee for 4.62 rad/s
    (the 32 mm lift in a 0.32 s swing); then 2.95, peak joint 3.60 (yaw).
    Stride 57.6 -> 72 mm (coxa +/-20.7 -> +/-25.6 deg). Duty 0.75 was
    rejected: two adjacent legs swing together and the CoM leaves the stance
    triangle by up to 59 mm for a quarter of every cycle. Swing speed is
    stride / T_swing = v * duty / (1 - duty): T does not change it at all,
    so the speed envelope is budget()'s job, not the cycle time's.

    D063 (B76): the swing is swing_profile() — velocity-matched at lift-off
    and touchdown — and the lift fades in with the command (lift_scale), so
    the gait is continuous in the command down to zero: foot_targets at a
    zero command IS the planted stance (it used to march in place). The
    envelope this costs: 45.5 -> 34.2 mm/s, 0.246 -> 0.185 rad/s (the soft
    lift spends longer inside the 15 mm loaded band). Command changes go
    through CommandSlew.
    """
    COXA_SWEEP_DEG = 33.0     # budget: half-stride <= (R0 - R_BODY) tan(33) (limit 40 - guard - lean room)
    SPEED_SAFETY = 0.97       # budget: predicted swing peak <= 0.97 x the free-leg limit
    LIFT_FULL_MM_S = 15.0     # D063: the lift fades in (smoothstep) up to this fastest-foot ground
    #                           speed: one cockpit tap (15 mm/s) already walks with the full lift,
    #                           and a slewed start/stop is continuous (no swing foot pops up / slams)
    CMD_ACCEL_MM_S2 = 25.0    # D063 CommandSlew: fastest-foot ground-speed change, mm/s per s (see there)
    CMD_LAG_S = 0.2           # D063 CommandSlew: the lag after the rate limit. Without it a ramp ending
    #                           at the envelope peaked at 3.03 rad/s (loaded 3.0); 0.2 s: 2.992 worst

    def __init__(self, body_height=None, stance_radius=None,
                 cycle_time=None, duty=None, step_height=None):
        d = _rm.gait_defaults()
        body_height = d["body_height"] if body_height is None else body_height
        stance_radius = d["stance_radius"] if stance_radius is None else stance_radius
        cycle_time = d["cycle_time"] if cycle_time is None else cycle_time
        duty = d["duty"] if duty is None else duty
        step_height = d["step_height"] if step_height is None else step_height
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

    # ---------------------------------------------------- D052 speed envelope
    def _vf_max(self, vx, vy, wz):
        """max over legs of |v + wz x p_nom_i| (mm/s): the ground speed under the fastest foot."""
        v = np.array([vx, vy, 0.0])
        vf = v[None, :] + np.cross(np.array([0.0, 0.0, wz]), self.p_nom)
        return float(np.linalg.norm(vf[:, :2], axis=1).max())

    def vf_limit(self, level_slope=0.0, level_blend=None):
        """(coxa, tangential, lift) mm/s: the ceilings on
        max_i |v_f,i| (the ground speed under the fastest foot). budget() uses the min.
        level_slope / level_blend (D065): the lift ceiling on a leveled plane (_lift_limit).
        coxa:        half-stride |v_f| duty T / 2 <= r_leg tan(33 deg) (~49 mm),
                     r_leg = R0 - R_BODY.
        tangential:  the swing foot covers the stride in (1-duty) T along
                     swing_profile (peak swing_xy_peak = 1.625x the mean),
                     tangential to the coxa at r_leg, so yaw peaks at
                     1.625 |v_f| duty / ((1-duty) r_leg): 44.8 mm/s.
        lift:        every joint while the foot is within SUPPORT_TOL_MM of the
                     ground (the loaded class, 3.0 rad/s) or airborne (free,
                     4.0): one leg's swing simulated and bisected over 13
                     stride directions (_lift_limit, cached). It binds: D052's
                     sine swing hit 3.02 rad/s on the knee at 48.5 mm/s (radial
                     stride); D063's soft swing peaks its vertical speed at
                     z = h/2 = 12 mm, inside the loaded band, and a radial
                     stride reaches the knee's 2.985 at 34.2 mm/s. The lift is
                     nonlinear in |v_f| (diagonal strides jump), hence no fit."""
        r_leg = self.R0 - R_BODY
        coxa = 2.0 * r_leg * np.tan(np.deg2rad(self.COXA_SWEEP_DEG)) / (self.duty * self.T)
        tang = (self.SPEED_SAFETY * _rm.servo_speed("free") * (1.0 - self.duty) * r_leg
                / (swing_xy_peak(self.duty) * self.duty))
        return float(coxa), float(tang), self._lift_limit(level_slope, level_blend)

    _LIFT_CACHE = {}
    LIFT_SAFETY = 0.995       # the swing sim IS the checker's computation, sampled 6x finer
    LEVEL_SLOPE_STEP = 0.0025  # D065: budget(level_slope=) rounds the plane slope UP to this (mm/mm,
    #                           0.14 deg): one cached lift ceiling per step, never an optimistic one

    @classmethod
    def level_slope_q(cls, level_slope):
        """|level_slope| rounded UP to LEVEL_SLOPE_STEP; exactly 0.0 for a bare plane, inf if
        non-finite (no envelope: budget() then stops the robot)."""
        s = abs(float(level_slope))
        if not np.isfinite(s):
            return float("inf")
        if s <= 1e-12:
            return 0.0
        return round(float(np.ceil(s / cls.LEVEL_SLOPE_STEP - 1e-9)) * cls.LEVEL_SLOPE_STEP, 9)

    def lift_scale(self, vf_max):
        """Fraction of hstep a swing lifts at this fastest-foot ground speed
        (mm/s): a smoothstep from 0 at standing to 1 at LIFT_FULL_MM_S (D063).
        At a zero command the swing foot stays down, so standing, starting and
        stopping are continuous in the command."""
        x = min(1.0, max(0.0, float(vf_max) / self.LIFT_FULL_MM_S))
        return x * x * (3.0 - 2.0 * x)

    def _swing_speeds(self, u, speed, n=160, lift=None, pn=None):
        """(max joint speed while the foot is near the ground, max while airborne),
        rad/s, for ONE leg's swing with ground velocity speed*u (LEG frame).
        lift: mm, default the gait's own at this speed (hstep x lift_scale).
        pn: the LEG-frame foothold, default the nominal one on the leg's axis."""
        pn = np.array([self.R0 - R_BODY, 0.0, -self.h]) if pn is None else np.asarray(pn, float)
        vf = speed * np.array([u[0], u[1], 0.0])
        T_st, T_sw = self.duty * self.T, (1.0 - self.duty) * self.T
        s = np.linspace(0.0, 1.0, n)
        xy, z = swing_profile(s, self.duty)
        p_lift, p_land = pn - vf * (0.5 * T_st), pn + vf * (0.5 * T_st)
        P = p_lift + (p_land - p_lift) * xy[:, None]
        P[:, 2] += (self.hstep * self.lift_scale(speed) if lift is None else lift) * z
        Q = np.array([leg_ik(p) for p in P])
        if not np.isfinite(Q).all():
            return np.inf, np.inf
        V = np.abs(np.diff(Q, axis=0)).max(axis=1) / (T_sw / (n - 1))
        low = (P[:, 2] + self.h) < SUPPORT_TOL_MM
        both = low[:-1] & low[1:]
        vl = float(V[both].max()) if both.any() else 0.0
        vfree = float(V[~both].max()) if (~both).any() else 0.0
        return vl, vfree

    def _swing_speeds_level(self, u, speed, plane, blend, n=160, lift=None, pn=None):
        """_swing_speeds on a leveled plane (D065): plane (mm/mm) is its slope along the
        stride (+ = the ground rises ahead), through the foothold. The swing carries its
        lift-off point's plane offset and moves to its touchdown point's by a smoothstep
        over swing progress blend = (a, b) — BodyLeveler's transfer, so it lands on the
        plane — and the 15 mm band is judged over the plane under the foot, as
        pebble_feasibility.check's support_plane does. The foothold sits unraised: a
        raised one (BodyLeveler raises 0..raise_mm) has the higher ceiling."""
        pn = np.array([self.R0 - R_BODY, 0.0, -self.h]) if pn is None else np.asarray(pn, float)
        vf = speed * np.array([u[0], u[1], 0.0])
        T_st, T_sw = self.duty * self.T, (1.0 - self.duty) * self.T
        s = np.linspace(0.0, 1.0, n)
        xy, z = swing_profile(s, self.duty)
        p_lift, p_land = pn - vf * (0.5 * T_st), pn + vf * (0.5 * T_st)
        P = p_lift + (p_land - p_lift) * xy[:, None]
        P[:, 2] += (self.hstep * self.lift_scale(speed) if lift is None else lift) * z
        ux = np.array([u[0], u[1]], float)
        ground = plane * ((P[:, :2] - pn[:2]) @ ux)                 # the plane under the foot
        g0, g1 = plane * ((p_lift[:2] - pn[:2]) @ ux), plane * ((p_land[:2] - pn[:2]) @ ux)
        P[:, 2] += g0 + (g1 - g0) * smooth01((s - blend[0]) / (blend[1] - blend[0]))
        Q = _leg_ik_rows(P)
        if not np.isfinite(Q).all():
            return np.inf, np.inf
        V = np.abs(np.diff(Q, axis=0)).max(axis=1) / (T_sw / (n - 1))
        low = (P[:, 2] - (pn[2] + ground)) < SUPPORT_TOL_MM
        both = low[:-1] & low[1:]
        vl = float(V[both].max()) if both.any() else 0.0
        vfree = float(V[~both].max()) if (~both).any() else 0.0
        return vl, vfree

    def _lift_feet(self):
        """[(LEG-frame foothold, stride directions in deg)] the lift ceiling is
        simulated at: every WaveGait foot stands on its leg's axis at
        R0 - R_BODY, and the leg is mirror-symmetric about that axis, so
        0..180 deg covers every stride. ArmedGait's leaned feet override it."""
        return [(np.array([self.R0 - R_BODY, 0.0, -self.h]), np.arange(0, 181, 15))]

    def _lift_limit(self, level_slope=0.0, level_blend=None):
        """Largest |v_f| (mm/s) whose swing keeps near-ground joints under the
        loaded limit and airborne ones under the free limit, worst of the
        stride directions at every foothold of _lift_feet (13 at one for
        WaveGait). Bisection, cached per gait parameter set (~0.1 s the first
        time). The full lift is tested alone first: lift_scale fades it out
        near standing, which must not hide a step height the servo cannot lift
        at any speed.
        D065: level_slope > 0 (rounded up by level_slope_q) judges every swing on
        a leveled plane of that slope along its stride, uphill and downhill
        (_swing_speeds_level), with the transfer over level_blend (None = params
        level.swing_blend). The transfer keeps a swing foot inside the band a
        little longer, so the ceiling drops (34.20 -> 33.51 mm/s at tan 2 deg,
        32.48 at tan 5, 31.42 at tan 8; 30-77 ms per slope step, measured
        2026-10-08/09: prime_level() computes them up front). 0 is the bare
        gait: the same key and the same number as before."""
        feet = self._lift_feet()
        key = (self.h, self.R0, self.T, self.duty, self.hstep, self.LIFT_FULL_MM_S,
               tuple(tuple(np.round(f, 6)) for f, _a in feet))
        q = self.level_slope_q(level_slope)
        if not np.isfinite(q):
            return 0.0                   # an unknown plane: no envelope
        blend = None
        if q > 0.0:
            blend = tuple(float(x) for x in (_rm.level_defaults()["swing_blend"]
                                             if level_blend is None else level_blend))
            key = key + (("level", q, blend),)
        if key not in self._LIFT_CACHE:
            lo_l = self.LIFT_SAFETY * _rm.servo_speed("loaded")
            lo_f = self.LIFT_SAFETY * _rm.servo_speed("free")

            def ok(pn, u, v, lift=None, plane=0.0):
                if plane:
                    vl, vf = self._swing_speeds_level(u, v, plane, blend, lift=lift, pn=pn)
                else:
                    vl, vf = self._swing_speeds(u, v, lift=lift, pn=pn)
                return vl <= lo_l and vf <= lo_f

            def limit():
                best = np.inf
                planes = (0.0,) if q == 0.0 else (q, -q)
                for pn, angs in feet:
                    for ang in np.deg2rad(angs):
                        u = (np.cos(ang), np.sin(ang))
                        if not ok(pn, u, 0.0, lift=self.hstep):
                            return 0.0           # the lift alone breaks the budget: fix h / T
                        for pl in planes:
                            a, b = 0.0, 150.0
                            if ok(pn, u, b, plane=pl):
                                continue
                            for _ in range(12):
                                m = 0.5 * (a + b)
                                a, b = (m, b) if ok(pn, u, m, plane=pl) else (a, m)
                            best = min(best, a)
                return best
            self._LIFT_CACHE[key] = float(limit())
        return self._LIFT_CACHE[key]

    def prime_level(self, max_slope, level_blend=None):
        """Compute the lift ceiling for every LEVEL_SLOPE_STEP up to max_slope now
        (D065, review 9r: lazily, budget(level_slope=) bisected inside the control
        step, 30-77 ms per new step). BodyLeveler.prime() calls it. Returns how
        many were computed (the rest were cached)."""
        n0 = len(self._LIFT_CACHE)
        q = self.level_slope_q(max_slope)
        if np.isfinite(q):
            for k in range(1, int(round(q / self.LEVEL_SLOPE_STEP)) + 1):
                self._lift_limit(k * self.LEVEL_SLOPE_STEP, level_blend)
        return len(self._LIFT_CACHE) - n0

    def budget(self, vx, vy, wz, level_slope=0.0, level_blend=None):
        """(vx, vy, wz) scaled UNIFORMLY (heading and curvature kept) so the
        command fits the coxa sweep and the free-leg speed limit. Commands
        inside the envelope come back unchanged. The step-lift speed does not
        depend on the command once the lift is full (3 h / ((1-duty) T)): it
        is the gait parameters' job — check_gait flags it.
        D065: level_slope = BodyLeveler.level_slope() (mm/mm) derates the
        envelope for the leveled plane (_lift_limit); 0 is the bare one."""
        m = self._vf_max(vx, vy, wz)
        if m < 1e-9:
            return float(vx), float(vy), float(wz)
        k = min(1.0, min(self.vf_limit(level_slope, level_blend)) / m)
        return float(vx * k), float(vy * k), float(wz * k)

    def max_command(self, level_slope=0.0, level_blend=None):
        """The envelope for a UI: {'v': mm/s any heading (wz = 0), 'wz': rad/s
        turning in place, 'vf': the foot ground-speed ceiling (mm/s)}. A mixed
        command is feasible when max_i |v + wz x p_i| <= vf (use budget())."""
        vf = min(self.vf_limit(level_slope, level_blend))
        return dict(v=vf, wz=vf / self.R0, vf=vf, vx=vf, vy=vf)

    def _stride_ends(self, pn, vf):
        """(lift-off, touchdown) BODY-frame points of a swing over foothold pn at
        ground velocity vf: stance ENDS at pn - vf*Tst/2 (behind), so the swing
        lifts there and flies FORWARD to pn + vf*Tst/2 where the next stance
        begins. (Reversed signs here = the walking-in-place bug that only
        physics simulation caught, 2026-07-28.)"""
        T_st = self.duty * self.T
        return pn - vf * (0.5 * T_st), pn + vf * (0.5 * T_st)

    def _leg_target(self, ph, pn, vf, lift):
        """(BODY-frame target, in stance) for one leg at gait phase ph (0..1),
        nominal foothold pn, ground velocity vf (body frame) and lift (mm)."""
        T_st = self.duty * self.T
        if ph < self.duty:                                 # STANCE: sweep with the ground
            return pn - vf * ((ph / self.duty - 0.5) * T_st), True   # s = -0.5 .. +0.5
        s = (ph - self.duty) / (1 - self.duty)             # SWING: fly to next touchdown, 0..1
        xy, z = swing_profile(s, self.duty)
        p_lift, p_land = self._stride_ends(pn, vf)
        p = p_lift + (p_land - p_lift) * xy
        p[2] += lift * z
        return p, False

    def level_xy(self, t, vx, vy, wz, blend=None, foot=False):
        """Where BodyLeveler evaluates its plane for each leg at time t (D065):
        (xy (5,2) BODY mm, s (5,) swing progress, NaN in stance, clear (5,) mm the
        swing foot's lift over its lift-off ground, 0 in stance). Stance: the
        foot's own xy (_leg_target), so a planted foot sweeping a slope stays on it.
        Swing: its lift-off point, moved to its touchdown point by a smoothstep
        over s in blend (None = params level.swing_blend), so the foot lands on the
        plane — evaluating at the stations instead lands a downhill foot off it
        every step, and the plain swing xy rides the plane's slope through the
        15 mm band at full speed. foot=True adds a fourth: the feet's real xy
        (5, 2), where the band is judged."""
        if blend is None:
            blend = _rm.level_defaults()["swing_blend"]
        v = np.array([vx, vy, 0.0])
        lift = self.hstep * self.lift_scale(self._vf_max(vx, vy, wz))
        xy = np.zeros((N_LEGS, 2))
        fxy = np.zeros((N_LEGS, 2))
        s = np.full(N_LEGS, np.nan)
        clear = np.zeros(N_LEGS)
        a, b = blend
        for i in range(N_LEGS):
            ph = (t / self.T + self.phase_off[i]) % 1.0     # = foot_targets'
            pn = self.p_nom[i]
            vf = v + np.cross(np.array([0, 0, wz]), pn)
            if ph < self.duty:
                xy[i] = fxy[i] = self._leg_target(ph, pn, vf, lift)[0][:2]
                continue
            si = (ph - self.duty) / (1 - self.duty)
            p_lift, p_land = self._stride_ends(pn, vf)
            xy[i] = (p_lift + (p_land - p_lift) * smooth01((si - a) / (b - a)))[:2]
            pxy, pz = swing_profile(si, self.duty)
            fxy[i] = (p_lift + (p_land - p_lift) * pxy)[:2]
            s[i] = si
            clear[i] = lift * pz
        return (xy, s, clear, fxy) if foot else (xy, s, clear)

    def foot_targets(self, t, vx, vy, wz):
        """BODY-frame foot targets for all legs at time t."""
        v = np.array([vx, vy, 0.0])
        lift = self.hstep * self.lift_scale(self._vf_max(vx, vy, wz))
        out = np.zeros((N_LEGS, 3))
        stance = np.zeros(N_LEGS, dtype=bool)
        for i in range(N_LEGS):
            ph = (t / self.T + self.phase_off[i]) % 1.0
            pn = self.p_nom[i]
            vf = v + np.cross(np.array([0, 0, wz]), pn)    # body-frame velocity of ground
            out[i], stance[i] = self._leg_target(ph, pn, vf, lift)
        return out, stance

    def joint_targets(self, t, vx, vy, wz):
        """All 15 joint angles at time t. Returns (angles[5,3], stance[5], feet_body[5,3])."""
        feet, stance = self.foot_targets(t, vx, vy, wz)
        q = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            q[i] = leg_ik(body_to_leg(i, feet[i]))
        return q, stance, feet

# ---------------------------------------------------------------- command slew (B76)
class CommandSlew:
    """The walking command, acceleration-limited (D063, B76 fix 1).

    foot_targets() reads the command at every tick, so a change used to land
    in one tick: standing -> 45 mm/s moved a joint target up to 28 deg in
    20 ms (each stance foot sits v_f T_st (ph/duty - 1/2) from its foothold,
    36 mm at the end of stance). Here a change moves the fastest foot's
    ground speed (WaveGait._vf_max of the difference: walk and turn judged
    alike) by at most `accel` mm/s^2, along a straight line in (vx, vy, wz)
    so the change keeps its heading and curvature. That ramp (u) then runs
    through a first-order lag of `lag` s (v, what the gait gets), so the
    command's acceleration eases in and out instead of switching on and off.
    v is a running average of u, so it stays inside the envelope when the
    target is (the envelope is convex), and its acceleration never exceeds
    `accel`. With the lift fading in (WaveGait.lift_scale) a start from
    standing is continuous too.

    Why the lag: a ramp adds foot motion, up to accel x T_st/2 (20 mm/s at
    25 mm/s^2), and the envelope keeps no headroom for it at full speed
    (steady peak 2.982 rad/s against the loaded 3.0). With the bare rate
    limit, a ramp that ends at the envelope as a leg lifts off peaked at
    3.03 rad/s (standing -> 34.2 mm/s at 18 deg, t0 0.54 s: SPEED_LOADED),
    and 5 of 1440 grid ramps failed at 25 (start, reversal, turn -> walk).
    Lowering accel does not fix that: over 200 start phases at that heading
    the worst still peaked at 3.012 at 15 and 3.0003 at 10. With the lag,
    whenever v accelerates at a it is still a x lag short of where u holds,
    so the foot motion a ramp adds is paid for by speed not yet reached,
    and the command eases out of a steady speed instead of leaving it at
    full rate (the onset is what a slowdown or a reversal from full speed
    runs into). At the defaults (accel 25, lag 0.2 s)
    none of 1440 grid ramps, 2400 finely phased ramps or 280 mid-ramp
    retargets fails. The worst is 2.992, at the start of a slowdown or a
    reversal from full speed; a start never tops the steady gait (2.982).
    A lag of 0.1 s reached 2.999. Standing -> the 34.2 mm/s envelope takes
    1.37 s on u and 2.36 s on v (the lag's tail ends, ~4.9 lags after u
    does: a zero target reads exactly zero). The worst joint step per 20 ms
    tick is the gait's own 3.4 deg (28.4 deg unslewed at HEAD).

    step() integrates this exactly for a target held over dt, so one step
    of t is the same as many smaller steps (pebble_feasibility.ramp_fn uses
    that).

    A STOP is never slewed: stop() — or step(..., direct=True) — sets the
    command at once. Use it for the safe-stop, the void guard (retreat and
    hold), a reflex brace, a latch, FALLEN. A zero TARGET is not a stop: it
    slows down smoothly, which removes the stop snap B76 names. A non-finite
    target is a stop.

        slew = CommandSlew(g)
        vx, vy, wz = slew.step(g.budget(*ask), dt)     # every tick
        vx, vy, wz = slew.stop()                        # a stop: zero now
    """
    FINISH_MM_S = 0.1         # the lag's tail closes the last 0.1 mm/s (fastest foot) at a constant
    #                           0.5 mm/s^2 (FINISH / lag), so it ENDS (an exponential never does) and
    #                           never steps: a 0.1 mm/s snap instead put a 100 Hz sample 8 mm/s off
    #                           (0.08 mm in 10 ms) and a ramp at 3.005 rad/s

    def __init__(self, gait, accel=None, lag=None):
        self.g = gait
        self.accel = float(gait.CMD_ACCEL_MM_S2 if accel is None else accel)
        self.lag = float(gait.CMD_LAG_S if lag is None else lag)
        self.u = np.zeros(3)                     # the rate-limited command
        self.v = np.zeros(3)                     # what runs: u through the lag

    def hold(self, v):
        """Set the command to v at once, with nothing still ramping (a stop, a
        replay, a steady starting point)."""
        self.u = np.asarray(v, float).reshape(3).copy()
        self.v = self.u.copy()
        return tuple(float(x) for x in self.v)

    def stop(self):
        """Zero the command NOW (never slewed)."""
        return self.hold(np.zeros(3))

    def step(self, target, dt, direct=False):
        """Advance dt (s) toward target (vx, vy, wz); returns the command to run.
        direct=True sets it at once (a stop, the void retreat's replay)."""
        tgt = np.asarray(target, float).reshape(3)
        if not np.isfinite(tgt).all():
            return self.stop()
        if direct:
            return self.hold(tgt)
        dt = max(0.0, float(dt)) if np.isfinite(dt) else 0.0
        d = tgt - self.u
        m = self.g._vf_max(*d)
        arrives = m <= self.accel * dt
        t_r = (m / self.accel if m > 0.0 else 0.0) if arrives else dt   # u ramps for t_r, then holds
        w = d * (self.accel / m) if m > 0.0 else np.zeros(3)             # u's rate on the ramp
        u0, v0 = self.u, self.v
        self.u = tgt.copy() if arrives else u0 + w * dt
        if self.lag <= 0.0:
            self.v = self.u.copy()
        else:                                    # the lag, solved exactly over the ramp ...
            v1 = u0 + w * (t_r - self.lag) + (v0 - u0 + w * self.lag) * np.exp(-t_r / self.lag)
            self.v = self._settle(v1, self.u, dt - t_r) if arrives else v1
        return tuple(float(x) for x in self.v)

    def _settle(self, v, u, t):
        """... and over t of holding u: the gap decays by the lag down to
        FINISH_MM_S, then closes at FINISH_MM_S / lag (same direction, no step)."""
        g = v - u
        n = self.g._vf_max(*g)
        if n <= 0.0:
            return u.copy()
        f = self.FINISH_MM_S
        t_exp = self.lag * np.log(n / f) if n > f else 0.0
        if t <= t_exp:
            return u + g * np.exp(-t / self.lag)
        n1 = min(n, f)                           # the gap when the linear finish starts
        left = n1 - (f / self.lag) * (t - t_exp)
        return u + g * (max(0.0, left) / n)

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

    B113 (2026-09-30): the speed envelope is judged at the feet this gait
    stands on. It inherited WaveGait's ceilings, which are worked out at the
    un-leaned foothold on each leg's axis (75 mm from the coxa); the lean
    brings the two legs opposite the arm 12.9 mm closer (62.8 mm) and 8.6 deg
    off-axis. Its budget read 44.6 mm/s (the lift ceiling at duty 0.78 on
    that axis foothold) and at 44.6 the checker failed SPEED_LOADED (yaw
    3.78 rad/s inside the 15 mm band: the lift ceiling), SPEED_FREE (4.11:
    the tangential one) and LIMIT_YAW (the coxa reached 42.7 deg). KINK was
    never skipped (0.003 rad/s: the same soft swing). vf_limit is now each
    ceiling at every active leg's leaned foothold, the stride in any
    direction: coxa 33.2, tangential 41.9, lift 37.3 -> 33.2 mm/s
    (0.174 rad/s in place), and _vf_max / foot_targets use those footholds
    too (a turn moves a leaned foot at wz x its own position; it used to
    take the un-leaned one, 3.9 mm/s of stance slide at the old envelope).
    """
    _LIFT_CACHE = {}          # its own: the key carries the leaned footholds

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

    @property
    def p_foot(self):
        """Where the active feet stand (BODY frame): p_nom with the lean. Read
        from p_nom every time, as foot_targets always did, so a caller that
        moves p_nom (the watchdog's body height) moves these feet too."""
        return self.p_nom - self.body_shift

    # ---------------------------------------------------- B113 speed envelope
    def _feet_leg(self):
        """The active legs' footholds in their LEG frames, the lean included,
        one per mirror pair: the leg is symmetric about its x axis, so (x, y)
        and (x, -y) share every ceiling."""
        out = {}
        for i in self.active:
            f = body_to_leg(i, self.p_foot[i])
            out.setdefault((round(float(f[0]), 6), round(abs(float(f[1])), 6)), f)
        return list(out.values())

    def _vf_max(self, vx, vy, wz):
        """max over the ACTIVE legs of |v + wz x p_foot_i| (mm/s): the ground
        speed under the fastest foot that is on the ground."""
        v = np.array([vx, vy, 0.0])
        vf = v[None, :] + np.cross(np.array([0.0, 0.0, wz]), self.p_foot[self.active])
        return float(np.linalg.norm(vf[:, :2], axis=1).max())

    def _lift_feet(self):
        """Every leaned foothold, 24 stride directions (0..345 deg): off the
        leg's axis the mirror symmetry no longer halves them."""
        return [(f, np.arange(0, 360, 15)) for f in self._feet_leg()]

    def vf_limit(self, level_slope=0.0, level_blend=None):
        """(coxa, tangential, lift) mm/s, each the worst over the active legs'
        leaned footholds f = (x, y) (LEG frame); budget() uses the min.
        coxa:        the stride may point anywhere, so the half-stride
                     |v_f| duty T / 2 must fit between f and the nearer edge
                     of the +-COXA_SWEEP_DEG wedge: x sin(33) - |y| cos(33).
                     (WaveGait's r_leg tan(33) is the tangential stride only;
                     a diagonal one at its 60.9 would reach 40.9 deg. It does
                     not bind there: 34.2.)
        tangential:  WaveGait's, at |f| from the coxa axis instead of r_leg:
                     the swing crosses f at 1.625x its mean speed.
        lift:        WaveGait's swing simulation (_lift_limit) at every f.
        Measured (arm leg 0, lean 16 mm): 33.2 / 41.9 / 37.3."""
        c = np.deg2rad(self.COXA_SWEEP_DEG)
        feet = self._feet_leg()
        half = min(float(f[0] * np.sin(c) - abs(f[1]) * np.cos(c)) for f in feet)
        coxa = 2.0 * max(half, 0.0) / (self.duty * self.T)
        r = min(float(np.hypot(f[0], f[1])) for f in feet)
        tang = (self.SPEED_SAFETY * _rm.servo_speed("free") * (1.0 - self.duty) * r
                / (swing_xy_peak(self.duty) * self.duty))
        return float(coxa), float(tang), self._lift_limit(level_slope, level_blend)

    def max_command(self, level_slope=0.0, level_blend=None):
        """WaveGait's, but in place the fastest foot is the leaned one farthest
        from the body centre (190.5 mm, not R0)."""
        vf = min(self.vf_limit(level_slope, level_blend))
        return dict(v=vf, wz=vf / self._vf_max(0.0, 0.0, 1.0), vf=vf, vx=vf, vy=vf)

    def level_xy(self, t, vx, vy, wz, blend=None, foot=False):
        """Not on the arm gait (D065): its raised limbs are not footholds, so the
        leveler's window and contact count would read them as ground."""
        raise NotImplementedError("D065: BodyLeveler runs on the WaveGait only")

    def foot_targets(self, t, vx, vy, wz):
        v = np.array([vx, vy, 0.0])
        lift = self.hstep * self.lift_scale(self._vf_max(vx, vy, wz))
        out = np.zeros((N_LEGS, 3))
        stance = np.zeros(N_LEGS, dtype=bool)
        for i in self.active:
            ph = (t / self.T + self._phase[i]) % 1.0
            pn = self.p_foot[i]                          # the body leans toward the stance side
            vf = v + np.cross(np.array([0, 0, wz]), pn)
            out[i], stance[i] = self._leg_target(ph, pn, vf, lift)   # the same soft swing (D063)
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
    # D052: every command goes through budget(); the checker judges the result
    import pebble_feasibility as pf
    mc = g.max_command()
    print(f"envelope: {mc['v']:.1f} mm/s any heading, {mc['wz']:.3f} rad/s in place "
          f"(ceilings coxa/tangential/lift = {np.round(g.vf_limit(), 1)} mm/s)")
    for cmd, name in [((45, 0, 0), "walk"), ((0, 45, 0), "strafe"), ((0, 0, 0.5), "turn-in-place"),
                      ((45, 45, 0.3), "arc")]:
        b = g.budget(*cmd)
        r = pf.check_gait(g, b)
        v, i, jn, _t = r.peak()
        print(f"{name:14s} asked {cmd} -> {tuple(round(x, 3) for x in b)}: peak {v:.2f} rad/s "
              f"(L{i} {jn}), lift-off/touchdown step {r.kink_max:.3f} rad/s "
              f"{'PASS' if r.ok else 'FAIL ' + ','.join(r.fails)}")
        assert r.ok
    # D063: standing -> the envelope through CommandSlew, the worst start phase, 50 Hz ticks
    q0, dt, worst, t_at = pf.planted_q(g), 0.02, 0.0, None
    target = g.budget(45.0, 0.0, 0.0)
    for t0 in np.linspace(0.0, g.T, 10, endpoint=False):
        slew, q_prev = CommandSlew(g), q0
        for k in range(1, int(3.0 / dt)):
            v = slew.step(target, dt)
            t_at = k * dt if (t_at is None and v == target) else t_at
            q = g.joint_targets(t0 + k * dt, *v)[0]
            worst, q_prev = max(worst, float(np.abs(q - q_prev).max())), q
    assert np.allclose(g.joint_targets(0.7, *slew.stop())[0], q0)      # a stop: planted at once
    print(f"slewed start ({g.CMD_ACCEL_MM_S2:g} mm/s^2, lag {g.CMD_LAG_S:g} s): at {target[0]:.1f} mm/s "
          f"after {t_at:.2f} s; worst joint step {np.rad2deg(worst):.2f} deg per 20 ms tick "
          f"(unslewed at HEAD: 28.4)")
