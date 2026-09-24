"""Motion feasibility (D052) — "the studio cannot author what the robot cannot do".

Owner, 2026-09-24: "I don't want to make a gesture the robot can't do, or
have it do things way quicker than possible." Before D052 nothing in gait/
checked anything: leg_ik went NaN silently, the keyframe studio could author
66 of 81 slider corners past a joint limit, `hold` was a 216 rad/s step and
fist_bump held its arm past the hip AND knee limits for 4 s. This module is
the one judge every motion source goes through — code gestures, keyframe
JSON, the gait at a command — and it answers in the same terms the servo
datasheet does.

    import pebble_feasibility as pf
    r = pf.check(fn, total)            # fn(g, t) -> (q[5,3] rad, claw[5] rad)
    r = pf.check_gait(g, (45, 0, 0))   # the wave gait at a command
    r = pf.check_spec(spec)            # a keyframe JSON spec
    r.ok, r.lines, r.to_json()

What a Report judges (every threshold comes from gait/rocky_model.py):
  LIMIT_*      a joint outside its soft range minus a 2 deg guard band
               (the servo's own clamp is AT the range; a target that rides
               the stop has no room for tracking overshoot). Claw: 0..55, no
               guard — 0 is "closed" and gestures sit there on purpose.
  NAN          an unreachable target (leg_ik returned NaN).
  SPEED_*      finite-difference joint speed per sample interval. A joint on
               a LOADED leg (its foot within 15 mm of the support plane at
               both ends of the interval) may move 3.0 rad/s, a FREE leg
               4.0, and nothing may exceed the no-load 4.7 (SPEED_HARD).
               Claw <= 8 rad/s (SCS0009 no-load ~9.5, VERIFY).
  JUMP         a step: > 2 deg between two consecutive samples at the entry
               (from q_from, default the planted stance) or exit (to q_to),
               or an interior speed violation that is a true discontinuity
               (re-probed at 0.1 ms).
  SUPPORT      kind="static": fewer than 3 feet on the ground at a sample.
  MARGIN       kind="static": the CoM (torso + every segment, masses from
               sim/mass_budget.json laid out exactly as sim/build_mjcf.py
               does) closer than 10 mm to the support-polygon edge (FAIL) /
               25 mm (warning). The old body-origin proxy is kept as a
               second column: it ignores the raised arm's own mass, which is
               what let two adjacent arms author a -57 mm pose.
  SELF_CONTACT with a MuJoCo model: robot-on-robot penetration > 1 mm with
               the body lifted clear of the floor (every 3rd sample).
  THERMAL      warning: a joint spends > 30 % of the samples above 0.6x its
               speed class limit (sustained speed = sustained current).
  THERMAL_LOAD from a MuJoCo PLAYBACK (judge_load(rep, load_rms); sim/
               audit_gestures.py runs it): a joint's RMS electrical torque
               (actuator_force - damping x qvel, rl_common.motor_torque) /
               stall over the motion — RMS because heat goes as tau^2, so a
               duty-cycled overload is not averaged away — above
               continuous_frac (0.65) = WARN
               (THERMAL_LOAD_WARN), above the thermal trip fraction (0.85,
               ~3 min to the servo's over-temp cut) = FAIL. D052 amendment:
               the MJCF now clips at the PEAK (stall) torque, so a motion
               that leans on more than the continuous torque is no longer
               stopped by the sim — this is where it is caught.
  SLIP         kind="static": the CoM's horizontal acceleration relative to
               the planted feet needs more friction than the feet have
               (> mu_slide g = FAIL, > half of it = SLIP_WARN). mu from params
               leg.foot.mu_slide (0.8). The old 5.5 Hz x 6 mm shake asked for
               0.73 g — 91 % of the pads' grip.

kind="gait" judges limits, speeds, NaN and support >= 3 with the gait's own
stance schedule (fn may return (q, claw, stance)); the CoM must stay inside
the STANCE polygon (< 0 mm = FAIL, < 10 mm = warning). That rule is what
rejected duty 0.75 for D052: in the 0,4,3,2,1 wave order two ADJACENT legs
overlap in swing for 25 % of the cycle and the CoM sits up to 59 mm outside
the remaining triangle — the 15 mm geometric rule alone would have counted
the low swing feet as support and passed it.

Pure numpy + rocky_model; MuJoCo only if a model is passed.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from itertools import combinations

import numpy as np

import rocky_model as rm
from pebble_gait import (WaveGait, leg_ik, body_to_leg, leg_to_body, N_LEGS,
                         STATION_DEG, L1, L2, L3, Z_HIP, R_BODY, SUPPORT_TOL_MM)

HERE = os.path.dirname(os.path.abspath(__file__))

GUARD_DEG = 2.0                 # margin inside the soft limits
JUMP_DEG = 2.0                  # consecutive-sample step that counts as a jump
CLAW_MAX_RAD_S = 8.0            # SCS0009 no-load ~9.5 rad/s (VERIFY) with margin
THERMAL_FRAC = 0.6              # "hot" = above this fraction of the class limit ...
THERMAL_WARN = 0.30             # ... for more than this fraction of the samples
MARGIN_FAIL_MM = 10.0
MARGIN_WARN_MM = 25.0
SELF_PEN_MM = 1.0
MAX_VIOLATIONS = 60
JOINTS = rm.LEG_JOINTS

FAIL_CODES = ("NAN", "LIMIT_YAW", "LIMIT_HIP", "LIMIT_KNEE", "LIMIT_CLAW", "SPEED_LOADED",
              "SPEED_FREE", "SPEED_HARD", "SPEED_CLAW", "JUMP", "SUPPORT", "MARGIN",
              "SELF_CONTACT", "REACH", "LOOP_WRAP", "SPEC", "SLIP", "THERMAL_LOAD")
WARN_CODES = ("THERMAL", "MARGIN_WARN", "MARGIN_GAIT", "SLIP_WARN", "THERMAL_LOAD_WARN")
LOAD_WARN = rm.continuous_frac()        # 0.65 x stall: the sustained (thermal) budget
LOAD_FAIL = rm.thermal()["trip_frac"]   # 0.85 x stall: the servo's over-temp cut in ~3 min
MU_SLIDE = float(rm.params()["leg"]["foot"]["mu_slide"])
SLIP_WINDOW_S = 0.04            # acceleration is judged over +-40 ms (servo response)
G_MM_S2 = 9810.0


# ------------------------------------------------------------------ limits
def joint_limits(guard_deg: float = GUARD_DEG, override: dict | None = None) -> dict:
    """{'yaw': (lo, hi) rad, ..., 'claw': (lo, hi) rad} — soft limits minus the guard
    (the claw gets no guard). override: {joint: (lo_deg, hi_deg)} replaces the params."""
    d = rm.joint_limits_deg()
    if override:
        d.update({k: tuple(v) for k, v in override.items()})
    out = {}
    for k, (lo, hi) in d.items():
        gd = 0.0 if k == "claw" else guard_deg
        out[k] = (np.deg2rad(lo + gd), np.deg2rad(hi - gd))
    return out


limits = joint_limits                           # module-level alias (check() takes a `limits` kwarg)


def speed_limits() -> dict:
    return dict(loaded=rm.servo_speed("loaded"), free=rm.servo_speed("free"),
                hard=rm.servo_speed("hard"), claw=CLAW_MAX_RAD_S)


def q_ok(q, guard_deg: float = GUARD_DEG) -> bool:
    """True if q (3,) or (5,3) is finite and inside the guarded soft limits."""
    q = np.atleast_2d(np.asarray(q, float))
    if not np.isfinite(q).all():
        return False
    L = joint_limits(guard_deg)
    for j, k in enumerate(JOINTS):
        if (q[:, j] < L[k][0] - 1e-9).any() or (q[:, j] > L[k][1] + 1e-9).any():
            return False
    return True


# ------------------------------------------------------------------ mass model
def _mass_budget() -> dict:
    with open(os.path.join(HERE, "..", "sim", "mass_budget.json")) as f:
        return json.load(f)


_MB = _mass_budget()
M_TORSO = _MB["torso"]
M_COXA = _MB["coxa"]
M_FEMUR = _MB["femur"]
M_TIBIA = _MB["tibia"]
M_PRONG = 4.0                                  # g, = build_mjcf.M_PRONG (two per leg)
M_FOOT = 0.3 * M_TIBIA                         # build_mjcf: foot sphere
M_SHIN = 0.7 * M_TIBIA - 2 * M_PRONG           # build_mjcf: shin capsule (D052)
R_C = rm.foot_contact_radius_mm()
M_LEG = M_COXA + M_FEMUR + M_TIBIA
M_TOTAL = M_TORSO + N_LEGS * M_LEG
TORSO_COM = np.array([0.0, 0.0, 0.75 * 18.0 + 0.25 * 44.0])   # two torso cylinders (build_mjcf)
_COXA_BOX = np.array([20.0, 0.0, 30.0])        # coxa box centre, coxa frame
_ST = np.deg2rad(STATION_DEG)


def leg_com_leg_frame(q) -> tuple[np.ndarray, float]:
    """(mass-weighted CoM (3,) in the LEG frame, leg mass g) — the same segment
    layout as sim/build_mjcf.leg_xml (checked against MuJoCo's subtree_com in
    gait/test_feasibility.py)."""
    q1, q2, q3 = q
    c1, s1 = np.cos(q1), np.sin(q1)

    def along(s_fem, s_tib):
        r = L1 + s_fem * np.cos(q2) + s_tib * np.cos(q2 + q3)
        z = Z_HIP + s_fem * np.sin(q2) + s_tib * np.sin(q2 + q3)
        return np.array([r * c1, r * s1, z])
    coxa = np.array([c1 * _COXA_BOX[0], s1 * _COXA_BOX[0], _COXA_BOX[2]])
    parts = ((M_COXA, coxa), (M_FEMUR, along(L2 / 2, 0.0)),
             (M_SHIN, along(L2, 0.46 * L3)), (M_FOOT, along(L2, L3 - R_C)),
             (2 * M_PRONG, along(L2, L3 + 12.0)))
    m = sum(p[0] for p in parts)
    return sum(p[0] * p[1] for p in parts) / m, m


def com_body(q) -> np.ndarray:
    """Whole-robot CoM (mm, BODY frame) for joint angles q[5,3]."""
    q = np.asarray(q, float).reshape(N_LEGS, 3)
    acc = M_TORSO * TORSO_COM
    for i in range(N_LEGS):
        c, m = leg_com_leg_frame(q[i])
        acc = acc + m * leg_to_body(i, c)
    return acc / M_TOTAL


def feet_body(q) -> np.ndarray:
    """IK foot points (5,3) mm, BODY frame (vectorised leg_fk + leg_to_body)."""
    q = np.asarray(q, float).reshape(N_LEGS, 3)
    r = L1 + L2 * np.cos(q[:, 1]) + L3 * np.cos(q[:, 1] + q[:, 2])
    z = Z_HIP + L2 * np.sin(q[:, 1]) + L3 * np.sin(q[:, 1] + q[:, 2])
    xl, yl = r * np.cos(q[:, 0]), r * np.sin(q[:, 0])
    c, s = np.cos(_ST), np.sin(_ST)
    return np.stack([R_BODY * c + c * xl - s * yl, R_BODY * s + s * xl + c * yl, z], axis=1)


# ------------------------------------------------------------------ support + margin
_TRIPLES = np.array(list(combinations(range(N_LEGS), 3)))


def support_plane(feet, tol: float = SUPPORT_TOL_MM, max_tilt_deg: float = 20.0, com=None):
    """(normal (3,), point (3,), support bool[5]) — the ground plane the feet
    stand on, in the BODY frame. Candidates: the level plane through the
    lowest foot, and every plane through three feet tilted < max_tilt_deg with
    no foot more than 2 mm below it. Pick the one that carries the most feet
    (then the flattest). This is what lets the bow's pitched body keep five
    supporting feet while a raised arm is still an arm. Two guards against
    a raised arm masquerading as ground (two arms 24 mm into their raise
    made a tilted 4-foot plane beat the level 3-foot one): a tilted plane
    must carry EVERY foot the level plane carries, and, com given, it must
    hold the CoM (a body rests on a plane that holds its CoM)."""
    feet = np.asarray(feet, float)
    ok = np.isfinite(feet).all(axis=1)
    if not ok.any():
        return np.array([0, 0, 1.0]), np.zeros(3), np.zeros(N_LEGS, bool)
    f = np.where(ok[:, None], feet, 1e6)                 # a NaN foot is "far above": never support
    lo = int(np.argmin(np.where(ok, feet[:, 2], np.inf)))
    tri = _TRIPLES[ok[_TRIPLES].all(axis=1)]
    A, B, C = f[tri[:, 0]], f[tri[:, 1]], f[tri[:, 2]]
    N = np.cross(B - A, C - A)
    nn = np.linalg.norm(N, axis=1)
    keep = nn > 1e-6
    N = N[keep] / nn[keep, None]
    A = A[keep]
    N = N * np.sign(N[:, 2:3] + 1e-12)
    keep = N[:, 2] >= np.cos(np.deg2rad(max_tilt_deg))
    N = np.vstack([[0.0, 0.0, 1.0], N[keep]])
    P = np.vstack([feet[lo], A[keep]])
    H = np.einsum("kij,kj->ki", f[None, :, :] - P[:, None, :], N)        # (cands, 5) height above plane
    H = np.where(ok[None, :], H, 1e6)
    valid = H.min(axis=1) >= -2.0
    lvl = H[0] < tol                                     # the level plane through the lowest foot
    # a tilted plane must carry every foot the level one does, and more: a
    # pitched body (bow) passes, an arm on its way up does not
    valid &= ((H < tol) | ~lvl[None, :]).all(axis=1)
    valid[0] = True
    cnt = (H < tol).sum(axis=1)
    score = np.where(valid, cnt + 0.5 * N[:, 2], -1.0)   # most feet, then the flattest
    order = np.argsort(-score)
    k = int(order[0])
    if com is not None and k != 0:
        for kk in order:
            if score[kk] < 0:
                break
            sup = (H[kk] < tol) & ok
            if sup.sum() >= 3 and _plane_margin(feet, sup, N[kk], com) >= 0:
                k = int(kk)
                break
    return N[k], P[k], (H[k] < tol) & ok


def _basis(n):
    e1 = np.cross(n, [0.0, 1.0, 0.0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.cross(n, [1.0, 0.0, 0.0])
    e1 /= np.linalg.norm(e1)
    return np.stack([e1, np.cross(n, e1)])


def _plane_margin(feet, sup, n, p3):
    B = _basis(n)
    return polygon_margin(feet[sup] @ B.T, B @ p3)


def _hull(pts):
    """Convex hull (CCW) of 2D points — monotone chain."""
    P = sorted(map(tuple, pts))
    if len(P) <= 2:
        return np.array(P)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in P:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(P):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def polygon_margin(pts2, p2) -> float:
    """Signed distance (mm) of p2 from the edge of the convex hull of pts2:
    positive inside. < 3 points -> minus the distance to the segment/point."""
    H = _hull(np.asarray(pts2, float))
    p2 = np.asarray(p2, float)
    if len(H) == 0:
        return -1e9
    if len(H) == 1:
        return -float(np.linalg.norm(p2 - H[0]))
    if len(H) == 2:
        a, b = H
        e = b - a
        u = np.clip(np.dot(p2 - a, e) / (e @ e + 1e-12), 0, 1)
        return -float(np.linalg.norm(p2 - (a + u * e)))
    dmin, inside = 1e9, True
    for k in range(len(H)):
        a, b = H[k], H[(k + 1) % len(H)]
        e = b - a
        nrm = np.array([-e[1], e[0]]) / (np.linalg.norm(e) + 1e-12)   # inward for CCW
        d = float(np.dot(p2 - a, nrm))
        inside &= d >= 0
        dmin = min(dmin, d)
    if inside:
        return dmin
    # outside: true distance to the hull boundary
    best = 1e9
    for k in range(len(H)):
        a, b = H[k], H[(k + 1) % len(H)]
        e = b - a
        u = np.clip(np.dot(p2 - a, e) / (e @ e + 1e-12), 0, 1)
        best = min(best, float(np.linalg.norm(p2 - (a + u * e))))
    return -best


def margins(q, support=None, normal=None):
    """(com_margin_mm, origin_margin_mm, support bool[5]) for a pose: the CoM and
    the body origin projected along the support-plane normal onto it (the
    plane is level in the world when the robot stands on it). support /
    normal: skip the plane search when the caller already has them."""
    feet = feet_body(q)
    com = com_body(q)
    if support is None or normal is None:
        n, _p0, sup = support_plane(feet, com=com)
    else:
        n = np.asarray(normal, float)
    if support is not None:
        sup = np.asarray(support, bool)
    if sup.sum() < 3:
        return -1e9, -1e9, sup
    B = _basis(n)
    pts = feet[sup] @ B.T
    return (polygon_margin(pts, B @ com), polygon_margin(pts, B @ np.zeros(3)), sup)


def com_margin(q, support_legs=None) -> float:
    """CoM-aware static margin (mm). support_legs: the planted legs (default:
    found from the pose)."""
    sup = None
    if support_legs is not None:
        sup = np.zeros(N_LEGS, bool)
        sup[list(support_legs)] = True
    return margins(q, sup)[0]


# ------------------------------------------------------------------ poses
def planted_q(g=None, offset=(0.0, 0.0, 0.0)) -> np.ndarray:
    """(5,3) the all-planted stance with the body displaced by offset (mm)."""
    g = g or WaveGait()
    off = np.asarray(offset, float)
    return np.array([leg_ik(body_to_leg(i, g.p_nom[i] - off)) for i in range(N_LEGS)])


# ------------------------------------------------------------------ self contact
class _SelfContact:
    """Robot-on-robot penetration with the body hung 1 m in the air."""

    def __init__(self, model):
        import mujoco
        self.mj = mujoco
        self.m = model
        self.d = mujoco.MjData(model)
        self.torso = model.body("torso").id
        self.jadr = np.array([[model.joint(f"{n}{i}").qposadr[0] for n in JOINTS]
                              for i in range(N_LEGS)])
        try:
            self.cadr = np.array([model.joint(f"claw{i}").qposadr[0] for i in range(N_LEGS)])
        except KeyError:
            self.cadr = None
        self.root = model.body_rootid

    def __call__(self, q, claw=None):
        d = self.d
        d.qpos[:] = self.m.qpos0
        d.qpos[0:3] = [0, 0, 1.0]
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[self.jadr.ravel()] = np.asarray(q, float).ravel()
        if claw is not None and self.cadr is not None:
            d.qpos[self.cadr] = claw
        self.mj.mj_forward(self.m, d)
        out = []
        rt = self.root[self.torso]
        for c in d.contact[:d.ncon]:
            b1, b2 = self.m.geom_bodyid[c.geom1], self.m.geom_bodyid[c.geom2]
            if self.root[b1] == rt and self.root[b2] == rt and c.dist < -SELF_PEN_MM / 1000:
                out.append((self.m.body(b1).name, self.m.body(b2).name, -c.dist * 1000))
        return out


# ------------------------------------------------------------------ report
@dataclass
class Report:
    name: str
    kind: str
    total: float
    fs: float
    ok: bool = True
    vmax: np.ndarray = field(default_factory=lambda: np.zeros((N_LEGS, 3)))   # rad/s
    vmax_t: np.ndarray = field(default_factory=lambda: np.zeros((N_LEGS, 3)))
    claw_vmax: float = 0.0
    hot_frac: np.ndarray = field(default_factory=lambda: np.zeros((N_LEGS, 3)))
    violations: list = field(default_factory=list)       # capped, worst-first per code
    codes: dict = field(default_factory=dict)            # code -> sample count (FAIL + WARN)
    nan_count: int = 0
    jumps: list = field(default_factory=list)
    margin_min: float = float("nan")                     # CoM margin, mm
    margin_t: float = float("nan")
    origin_margin_min: float = float("nan")              # the old body-origin proxy
    support_min: int = N_LEGS
    support_t: float = float("nan")
    acc_max_g: float = 0.0                               # CoM horizontal accel vs planted feet, in g
    self_contacts: int | None = None
    self_contact_pairs: list = field(default_factory=list)
    load_rms: np.ndarray | None = None                  # (5,3) RMS tau_e/stall, MuJoCo playback
    lines: list = field(default_factory=list)
    notes: list = field(default_factory=list)

    @property
    def fails(self) -> list:
        return [c for c in self.codes if c in FAIL_CODES]

    @property
    def warnings(self) -> list:
        return [c for c in self.codes if c in WARN_CODES]

    def peak(self):
        """(rad/s, leg, joint name, t) of the fastest leg joint."""
        k = int(np.nanargmax(self.vmax))
        i, j = divmod(k, 3)
        return float(self.vmax[i, j]), i, JOINTS[j], float(self.vmax_t[i, j])

    def to_json(self) -> dict:
        def f(x):
            return None if x is None or (isinstance(x, float) and not np.isfinite(x)) else round(float(x), 3)
        return dict(name=self.name, kind=self.kind, total=f(self.total), ok=self.ok,
                    fails=self.fails, warnings=self.warnings, codes=dict(self.codes),
                    vmax=np.round(self.vmax, 3).tolist(), claw_vmax=f(self.claw_vmax),
                    hot_frac=np.round(self.hot_frac, 3).tolist(), nan_count=self.nan_count,
                    jumps=self.jumps, margin_min=f(self.margin_min), margin_t=f(self.margin_t),
                    origin_margin_min=f(self.origin_margin_min), support_min=self.support_min,
                    support_t=f(self.support_t), acc_max_g=f(self.acc_max_g),
                    self_contacts=self.self_contacts,
                    self_contact_pairs=self.self_contact_pairs[:10],
                    load_rms=None if self.load_rms is None else np.round(self.load_rms, 3).tolist(),
                    violations=self.violations, lines=self.lines, notes=self.notes)

    def __str__(self):
        return "\n".join(self.lines)


def _add(rep, code, n=1, **kw):
    rep.codes[code] = rep.codes.get(code, 0) + n
    if kw and len(rep.violations) < MAX_VIOLATIONS:
        rep.violations.append(dict(code=code, **{k: (round(float(v), 4) if isinstance(v, (float, np.floating))
                                                     else v) for k, v in kw.items()}))


def _as_q(out):
    """fn output -> (q (5,3), claw (5,), stance bool[5] or None). Accepts (q, claw),
    (q, claw, stance) — a gait that knows its own stance schedule — or q."""
    if isinstance(out, tuple):
        q = np.asarray(out[0], float).reshape(N_LEGS, 3)
        c = out[1] if len(out) > 1 else None
        c = np.zeros(N_LEGS) if c is None or np.ndim(c) == 0 or np.size(c) != N_LEGS else np.asarray(c, float)
        st = None
        if len(out) > 2 and out[2] is not None and np.size(out[2]) == N_LEGS:
            st = np.asarray(out[2], bool)
        return q, c, st
    return np.asarray(out, float).reshape(N_LEGS, 3), np.zeros(N_LEGS), None


# ------------------------------------------------------------------ the check
def check(fn, total, g=None, fs: float = 100.0, q_from=None, q_to=None, kind: str = "static",
          model=None, limits: dict | None = None, name: str | None = None,
          claw_from=None, claw_to=None, contact_every: int = 3, t0: float = 0.0,
          guard_deg: float = GUARD_DEG) -> Report:
    """Judge a motion. fn(g, t) -> (q[5,3] rad, claw[5] rad), t in [t0, t0+total].

    q_from / q_to: the pose before / after (default the planted stance, so the
    entry and exit steps count); pass False to skip one (a looping gait, a
    gesture that ends in a held pose). kind: "static" (every sample must stand
    on >= 3 feet with the CoM margin) or "gait". model: a MuJoCo model for the
    self-contact pass (optional). limits: {joint: (lo_deg, hi_deg)} overriding
    params joints.pos_deg (the guard still applies)."""
    g = g or WaveGait()
    rep = Report(name=name or getattr(fn, "__name__", getattr(fn, "name", "motion")),
                 kind=kind, total=float(total), fs=float(fs))
    L = joint_limits(guard_deg, limits)
    V = speed_limits()
    n = max(2, int(round(total * fs)) + 1)
    ts = t0 + np.linspace(0.0, total, n)
    Q = np.zeros((n, N_LEGS, 3))
    C = np.zeros((n, N_LEGS))
    S = [None] * n                                               # scheduled stance, if the fn knows it
    for k, t in enumerate(ts):
        Q[k], C[k], S[k] = _as_q(fn(g, t))
    planted = planted_q(g)
    pre = post = None
    if q_from is not False:
        pre = planted if q_from is None else np.asarray(q_from, float).reshape(N_LEGS, 3)
    if q_to is not False:
        post = planted if q_to is None else np.asarray(q_to, float).reshape(N_LEGS, 3)
    dt = total / (n - 1)

    # --- NaN / limits / support / margins per sample
    nanmask = ~np.isfinite(Q).all(axis=2)                         # (n,5)
    rep.nan_count = int(nanmask.any(axis=1).sum())
    for k in np.flatnonzero(nanmask.any(axis=1))[:5]:
        _add(rep, "NAN", 0, t=ts[k], legs=np.flatnonzero(nanmask[k]).tolist())
    if rep.nan_count:
        rep.codes["NAN"] = rep.nan_count
    loaded = np.zeros((n, N_LEGS), bool)
    com_rel = np.full((n, N_LEGS, 2), np.nan)                     # CoM xy minus each foot's xy
    m_com = np.full(n, np.nan)
    m_org = np.full(n, np.nan)
    nsup = np.zeros(n, int)
    # limits: one entry per (leg, joint) — the WORST excess, with its sample count
    for j, jn in enumerate(JOINTS):
        lo, hi = L[jn]
        with np.errstate(invalid="ignore"):
            exc = np.maximum(lo - Q[:, :, j], Q[:, :, j] - hi)          # (n,5) rad past the limit
        exc = np.where(nanmask, -np.inf, exc)
        for i in range(N_LEGS):
            bad = exc[:, i] > 1e-9
            if bad.any():
                k = int(np.argmax(exc[:, i]))
                v = Q[k, i, j]
                _add(rep, f"LIMIT_{jn.upper()}", int(bad.sum()), t=ts[k], leg=i, joint=jn,
                     value_deg=np.rad2deg(v), limit_deg=np.rad2deg(lo if v < lo else hi),
                     samples=int(bad.sum()))
    for k in range(n):
        q = Q[k]
        if nanmask[k].any():
            continue
        feet = feet_body(q)
        com = com_body(q)
        com_rel[k] = com[None, :2] - feet[:, :2]
        nrm, _p, sup = support_plane(feet, com=com)
        loaded[k] = sup                                          # speed class: the 15 mm rule
        if S[k] is not None:                                     # margin: the gait's own schedule —
            sup = S[k]                                           # a low swing foot carries no weight
        nsup[k] = int(sup.sum())
        if nsup[k] >= 3:
            m_com[k], m_org[k], _ = margins(q, sup, nrm)
    clo, chi = L["claw"]
    cexc = np.maximum(clo - C, C - chi)
    for i in range(N_LEGS):
        bad = cexc[:, i] > 1e-9
        if bad.any():
            k = int(np.argmax(cexc[:, i]))
            _add(rep, "LIMIT_CLAW", int(bad.sum()), t=ts[k], leg=i, joint="claw",
                 value_deg=np.rad2deg(C[k, i]), limit_deg=np.rad2deg(clo if C[k, i] < clo else chi),
                 samples=int(bad.sum()))

    # --- speeds on the sample intervals (entry/exit handled as jumps below)
    good = ~nanmask.any(axis=1)
    dQ = np.diff(Q, axis=0)                                        # (n-1,5,3)
    Vq = np.abs(dQ) / dt
    Vq[~(good[:-1] & good[1:])] = 0.0
    both_loaded = loaded[:-1] & loaded[1:]                         # (n-1,5)
    lim = np.where(both_loaded, V["loaded"], V["free"])[:, :, None] * np.ones(3)
    rep.vmax = Vq.max(axis=0) if len(Vq) else np.zeros((N_LEGS, 3))
    rep.vmax_t = ts[np.argmax(Vq, axis=0)] if len(Vq) else np.zeros((N_LEGS, 3))
    rep.hot_frac = (Vq > THERMAL_FRAC * lim).mean(axis=0) if len(Vq) else np.zeros((N_LEGS, 3))
    over_hard = Vq > V["hard"] + 1e-9
    over_cls = (Vq > lim + 1e-9) & ~over_hard
    for code, mask in (("SPEED_HARD", over_hard),
                       ("SPEED_LOADED", over_cls & both_loaded[:, :, None]),
                       ("SPEED_FREE", over_cls & ~both_loaded[:, :, None])):
        if not mask.any():
            continue
        rep.codes[code] = int(mask.sum())
        for i in range(N_LEGS):
            for j in range(3):
                ks = np.flatnonzero(mask[:, i, j])
                if len(ks):
                    kw = ks[np.argmax(Vq[ks, i, j])]
                    _add(rep, code, 0, t=ts[kw], leg=i, joint=JOINTS[j], value=Vq[kw, i, j],
                         limit=float(lim[kw, i, j]) if code != "SPEED_HARD" else V["hard"],
                         samples=len(ks))
    # interior discontinuities: re-probe the worst hard-speed intervals at 0.1 ms
    if over_hard.any():
        for kw in np.unique(np.argwhere(over_hard)[:, 0])[:20]:
            ta = ts[kw]
            for s in np.linspace(0, dt, 11)[:-1]:
                qa = _as_q(fn(g, ta + s))[0]
                qb = _as_q(fn(g, ta + s + 1e-4))[0]
                dq = np.nanmax(np.abs(qb - qa))
                if np.rad2deg(dq) > 0.5:
                    i, j = np.unravel_index(np.nanargmax(np.abs(qb - qa)), qa.shape)
                    rep.jumps.append(dict(t=round(float(ta + s), 4), where="interior", leg=int(i),
                                          joint=JOINTS[j], dq_deg=round(float(np.rad2deg(dq)), 2)))
                    _add(rep, "JUMP")
                    break
    Vc = np.abs(np.diff(C, axis=0)) / dt
    rep.claw_vmax = float(Vc.max()) if len(Vc) else 0.0
    if rep.claw_vmax > V["claw"] + 1e-9:
        k, i = np.unravel_index(np.argmax(Vc), Vc.shape)
        _add(rep, "SPEED_CLAW", int((Vc > V["claw"]).sum()), t=ts[k], leg=int(i), joint="claw",
             value=Vc[k, i], limit=V["claw"])

    # --- entry / exit steps
    for where, qa, qb, ca, cb, t in (("entry", pre, Q[0], claw_from, C[0], ts[0]),
                                     ("exit", Q[-1], post, C[-1], claw_to, ts[-1])):
        if qa is None or qb is None:
            continue
        dq = np.abs(np.asarray(qb) - np.asarray(qa))
        if not np.isfinite(dq).all():
            continue
        if np.rad2deg(dq.max()) > JUMP_DEG:
            i, j = np.unravel_index(np.argmax(dq), dq.shape)
            rep.jumps.append(dict(t=round(float(t), 4), where=where, leg=int(i), joint=JOINTS[j],
                                  dq_deg=round(float(np.rad2deg(dq.max())), 2)))
            _add(rep, "JUMP")
        if ca is not None and cb is not None:
            dc = np.abs(np.asarray(cb, float) - np.asarray(ca, float))
            if np.rad2deg(dc.max()) > JUMP_DEG:
                rep.jumps.append(dict(t=round(float(t), 4), where=where, leg=int(np.argmax(dc)),
                                      joint="claw", dq_deg=round(float(np.rad2deg(dc.max())), 2)))
                _add(rep, "JUMP")

    # --- support / margin
    if good.any():
        k = int(np.argmin(np.where(good, nsup, 99)))
        rep.support_min, rep.support_t = int(nsup[k]), float(ts[k])
    if np.isfinite(m_com).any():
        k = int(np.nanargmin(m_com))
        rep.margin_min, rep.margin_t = float(m_com[k]), float(ts[k])
        rep.origin_margin_min = float(np.nanmin(m_org))
    if kind == "static":
        few = good & (nsup < 3)
        if few.any():
            _add(rep, "SUPPORT", int(few.sum()), t=ts[np.argmax(few)], feet=int(nsup[np.argmax(few)]))
        if np.isfinite(rep.margin_min) and rep.margin_min < MARGIN_FAIL_MM:
            _add(rep, "MARGIN", int((m_com < MARGIN_FAIL_MM).sum()), t=rep.margin_t,
                 value=rep.margin_min, limit=MARGIN_FAIL_MM)
        elif np.isfinite(rep.margin_min) and rep.margin_min < MARGIN_WARN_MM:
            _add(rep, "MARGIN_WARN", int((m_com < MARGIN_WARN_MM).sum()), t=rep.margin_t,
                 value=rep.margin_min, limit=MARGIN_WARN_MM)
    else:
        few = good & (nsup < 3)
        if few.any():
            _add(rep, "SUPPORT", int(few.sum()), t=ts[np.argmax(few)], feet=int(nsup[np.argmax(few)]))
        if np.isfinite(rep.margin_min) and rep.margin_min < 0:     # CoM outside the STANCE polygon:
            _add(rep, "MARGIN", int((m_com < 0).sum()), t=rep.margin_t,   # the gait is no longer
                 value=rep.margin_min, limit=0.0)                           # quasi-static (duty 0.75!)
        elif np.isfinite(rep.margin_min) and rep.margin_min < MARGIN_FAIL_MM:
            _add(rep, "MARGIN_GAIT", int((m_com < MARGIN_FAIL_MM).sum()), t=rep.margin_t,
                 value=rep.margin_min, limit=MARGIN_FAIL_MM)

    # --- slip: the CoM's acceleration relative to the planted feet, as a
    # central difference over +-SLIP_WINDOW_S: the pads feel the acceleration
    # the servos deliver, and a velocity kink shorter than the servo's own
    # response (50 Hz bus + 20 ms latency) never reaches them — a 10 ms second
    # difference turned every linear-ease keyframe corner into a "slip".
    w = max(1, int(round(SLIP_WINDOW_S / dt)))
    if kind == "static" and n >= 2 * w + 1:
        keep = loaded[:-2 * w] & loaded[w:-w] & loaded[2 * w:]                   # (n-2w,5)
        A = (com_rel[2 * w:] - 2 * com_rel[w:-w] + com_rel[:-2 * w]) / (w * dt) ** 2   # mm/s^2
        a = np.where(keep, np.linalg.norm(A, axis=2), np.nan)
        cnt = keep.sum(axis=1)
        a_s = np.where(cnt > 0, np.nansum(np.nan_to_num(a), axis=1) / np.maximum(cnt, 1), 0.0)
        if len(a_s):
            k = int(np.argmax(a_s))
            rep.acc_max_g = float(a_s[k] / G_MM_S2)
            if rep.acc_max_g > MU_SLIDE:
                _add(rep, "SLIP", int((a_s / G_MM_S2 > MU_SLIDE).sum()), t=ts[k + w],
                     value=rep.acc_max_g, limit=MU_SLIDE)
            elif rep.acc_max_g > 0.5 * MU_SLIDE:
                _add(rep, "SLIP_WARN", int((a_s / G_MM_S2 > 0.5 * MU_SLIDE).sum()), t=ts[k + w],
                     value=rep.acc_max_g, limit=0.5 * MU_SLIDE)

    # --- thermal warning
    hot = rep.hot_frac > THERMAL_WARN
    if hot.any():
        i, j = np.unravel_index(np.argmax(rep.hot_frac), hot.shape)
        _add(rep, "THERMAL", int(hot.sum()), leg=int(i), joint=JOINTS[j], frac=rep.hot_frac[i, j])

    # --- self contact (MuJoCo, optional)
    if model is not None:
        sc = model if isinstance(model, _SelfContact) else _SelfContact(model)
        pairs, hits = {}, 0
        for k in range(0, n, max(1, contact_every)):
            if not good[k]:
                continue
            got = sc(Q[k], C[k])
            if got:
                hits += 1
                for a, b, pen in got:
                    key = tuple(sorted((a, b)))
                    pairs[key] = max(pairs.get(key, 0.0), pen)
                    seen = [tuple(v.get("pair", ())) for v in rep.violations]
                    if len(rep.violations) < MAX_VIOLATIONS and key not in seen:
                        _add(rep, "SELF_CONTACT", 0, t=ts[k], pair=list(key), pen_mm=pen)
        rep.self_contacts = hits
        rep.self_contact_pairs = [dict(pair=list(k), pen_mm=round(v, 2)) for k, v in pairs.items()]
        if hits:
            rep.codes["SELF_CONTACT"] = hits

    rep.ok = not rep.fails
    rep.lines = _lines(rep, V)
    return rep


def _lines(rep, V) -> list:
    """The human verdict: one headline, then one line per failing / warning code."""
    out = []
    v, i, jn, t = rep.peak() if np.isfinite(rep.vmax).any() else (0.0, 0, "-", 0.0)
    head = (f"{'PASS' if rep.ok else 'FAIL'} {rep.name} ({rep.kind}, {rep.total:.2f} s): "
            f"peak {v:.2f} rad/s at leg {i} {jn} t={t:.2f}s (loaded {V['loaded']:.1f} / free "
            f"{V['free']:.1f} / hard {V['hard']:.1f}); claw {rep.claw_vmax:.1f} rad/s; support >= "
            f"{rep.support_min}; CoM margin {rep.margin_min:.0f} mm (origin {rep.origin_margin_min:.0f})"
            + (f"; accel {rep.acc_max_g:.2f} g" if rep.kind == "static" else ""))
    if rep.self_contacts is not None:
        head += f"; self-contact samples {rep.self_contacts}"
    if rep.load_rms is not None:
        i, j = np.unravel_index(int(np.argmax(rep.load_rms)), rep.load_rms.shape)
        head += f"; RMS load {rep.load_rms[i, j]:.2f} x stall (leg {i} {JOINTS[j]})"
    out.append(head)
    seen = set()
    for vi in rep.violations:
        c = vi["code"]
        key = (c, vi.get("leg"), vi.get("joint"))
        if key in seen and c != "JUMP":
            continue
        seen.add(key)
        tag = "WARN" if c in WARN_CODES else "FAIL"
        where = f" leg {vi['leg']} {vi.get('joint', '')}" if "leg" in vi else ""
        if c.startswith("SPEED"):
            out.append(f"  {tag} {c}:{where} {vi['value']:.2f} rad/s > {vi['limit']:.1f} at t={vi['t']:.2f}s"
                       f" ({vi.get('samples', 1)} samples)")
        elif c.startswith("LIMIT"):
            guard = "" if c == "LIMIT_CLAW" else f" (soft limit - {GUARD_DEG:g} deg guard)"
            out.append(f"  {tag} {c}:{where} {vi['value_deg']:.1f} deg past {vi['limit_deg']:.1f}{guard}"
                       f" at t={vi['t']:.2f}s ({vi.get('samples', 1)} samples)")
        elif c in ("MARGIN", "MARGIN_WARN", "MARGIN_GAIT"):
            out.append(f"  {tag} {c}: CoM {vi['value']:.1f} mm from the support edge (< {vi['limit']:.0f}) "
                       f"at t={vi['t']:.2f}s")
        elif c == "SUPPORT":
            out.append(f"  {tag} SUPPORT: only {vi['feet']} feet down at t={vi['t']:.2f}s (need 3)")
        elif c == "NAN":
            out.append(f"  {tag} NAN: unreachable target, legs {vi['legs']} at t={vi['t']:.2f}s")
        elif c == "THERMAL":
            out.append(f"  {tag} THERMAL:{where} {100 * vi['frac']:.0f}% of the time above "
                       f"{THERMAL_FRAC:.1f}x its speed limit")
        elif c == "SELF_CONTACT":
            out.append(f"  {tag} SELF_CONTACT: {vi['pair'][0]} x {vi['pair'][1]} {vi['pen_mm']:.1f} mm "
                       f"at t={vi['t']:.2f}s")
        elif c in ("REACH", "LOOP_WRAP", "SPEC"):
            out.append(f"  {tag} {c}: {vi.get('msg', '')}")
        elif c in ("SLIP", "SLIP_WARN"):
            out.append(f"  {tag} {c}: the body accelerates {vi['value']:.2f} g over planted feet "
                       f"(> {vi['limit']:.2f}; foot mu {MU_SLIDE:g}) at t={vi['t']:.2f}s")
        elif c == "SPEED_CLAW":
            out.append(f"  {tag} SPEED_CLAW: {vi['value']:.1f} rad/s > {vi['limit']:.0f}")
        elif c in ("THERMAL_LOAD", "THERMAL_LOAD_WARN"):
            out.append(f"  {tag} {c}:{where} RMS load {vi['value']:.2f} x stall over the motion "
                       f"(> {vi['limit']:.2f}: " + ("the servo's over-temp cut in ~3 min if sustained)"
                                                    if c == "THERMAL_LOAD" else "above its continuous budget)"))
    for jmp in rep.jumps[:8]:
        out.append(f"  FAIL JUMP ({jmp['where']}): leg {jmp['leg']} {jmp['joint']} steps "
                   f"{jmp['dq_deg']:.1f} deg at t={jmp['t']:.2f}s")
    for n in rep.notes:
        out.append(f"  note: {n}")
    return out


# ------------------------------------------------------------------ thermal (MuJoCo playback)
def judge_load(rep: Report, load_rms) -> Report:
    """Add the THERMAL_LOAD verdict to a Report from a MuJoCo playback of the
    same motion: load_rms = each leg joint's sqrt(mean(tau_e^2)) / stall over
    the motion, (5,3) or (15,) leg-major, tau_e = rl_common.motor_torque (the
    current term: actuator_force - damping x qvel). RMS, not mean |tau|: the
    thermal model is in tau^2, so a 50 % duty of stall (mean 0.50, RMS 0.71)
    must not pass as sustainable — it trips rl_common.ThermalProxy in ~11 min. > continuous_frac (0.65) =
    THERMAL_LOAD_WARN, > the thermal trip fraction (0.85) = THERMAL_LOAD
    (FAIL). Mutates and returns rep (ok + lines recomputed)."""
    L = np.asarray(load_rms, float).reshape(N_LEGS, 3)
    rep.load_rms = L
    for code, lim, mask in (("THERMAL_LOAD", LOAD_FAIL, L > LOAD_FAIL),
                            ("THERMAL_LOAD_WARN", LOAD_WARN, (L > LOAD_WARN) & (L <= LOAD_FAIL))):
        if not mask.any():
            continue
        rep.codes[code] = int(mask.sum())
        for i, j in zip(*np.nonzero(mask)):
            _add(rep, code, 0, leg=int(i), joint=JOINTS[j], value=L[i, j], limit=lim)
    rep.ok = not rep.fails
    rep.lines = _lines(rep, speed_limits())
    return rep


# ------------------------------------------------------------------ gait + specs
def gait_fn(cmd):
    vx, vy, wz = cmd

    def fn(g, t):
        q, st, _f = g.joint_targets(t, vx, vy, wz)
        return q, np.zeros(N_LEGS), st
    fn.__name__ = f"gait({vx:g},{vy:g},{wz:g})"
    return fn


def check_gait(g=None, cmd=(45.0, 0.0, 0.0), cycles: float = 2.0, fs: float = 100.0,
               model=None) -> Report:
    """The wave gait at a steady command for `cycles` cycles (no entry/exit:
    the gait is cyclic; the command ramp is the caller's)."""
    g = g or WaveGait()
    return check(gait_fn(cmd), cycles * g.T, g=g, fs=fs, q_from=False, q_to=False,
                 kind="gait", model=model)


def check_spec(spec: dict, g=None, fs: float = 100.0, model=None) -> Report:
    """A keyframe spec (gait/gestures/*.json) — including the tail the player
    appends, unreachable `reach` targets and the loop wrap."""
    from pebble_keyframes import KeyframeGesture            # lazy: keyframes imports us
    g = g or WaveGait()
    try:
        kg = KeyframeGesture(spec)
    except (ValueError, KeyError, TypeError) as e:
        rep = Report(name=str(spec.get("name", "?")), kind="static", total=0.0, fs=fs, ok=False)
        _add(rep, "SPEC", 1, msg=f"spec does not parse: {e}")
        rep.lines = _lines(rep, speed_limits())
        return rep
    return kg.report(g=g, fs=fs, model=model)


if __name__ == "__main__":
    g = WaveGait()
    for cmd in ((45, 0, 0), (0, 45, 0), (0, 0, 0.35), (45, 0, 0.35)):
        print("\n".join(check_gait(g, cmd).lines))
