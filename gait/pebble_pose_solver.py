"""Whole-body IK for gesture authoring (D052) — "is there a way to have an
inverse kinematics workflow for learning or creating gestures?"

Two answers, both producing keyframe data (pebble_keyframes) that
pebble_feasibility has already judged:

1. solve_reach(g, leg, target) — AUTHOR by pointing: "put leg 0's hand
   there". A single leg's IK alone often cannot (the target is past its
   reach, or reaching it drags the CoM off the support polygon), so this
   searches the BODY pose too — offset +-40 mm in x/y, -45..+25 mm in z, yaw
   +-18 deg — keeping every planted leg inside its guarded limits and the
   whole-robot CoM >= keep_margin_mm inside the planted feet, with the
   least body motion that works. The result drops straight into a keyframe
   (`frame`).

2. keyframes_from_stream(qs, fs) — TEACH by demonstration: record a joint
   stream (the real robot back-driven by hand with torque off, a sim run,
   a policy rollout) and turn it into a keyframe spec: Ramer-Douglas-Peucker
   in joint space keeps the fewest frames whose linear interpolation stays
   within tol_deg of every recorded sample; each kept sample becomes a
   frame via pose_from_joint_state (body pose fitted from the planted legs,
   raised legs as joint `arm` overrides).

Honest limits of (2):
  * the body pose is fitted assuming the planted feet stand on their
    NOMINAL footholds (p_nom). A recording in which feet slide or step
    breaks that; the fit residual (mm) is reported per frame and overall —
    above ~5 mm, do not trust the body pose.
  * "planted" = within 15 mm of the support plane (pebble_feasibility); a
    foot hovering 10 mm up is called planted and gets pulled onto the floor.
  * playback interpolates planted legs in POSE space and arms in joint
    space, so between frames the replay differs from the recording by up to
    tol_deg on arms and by the pose-space/joint-space gap on planted legs
    (small for the few-mm, few-deg body moves a planted pose allows).
  * nothing here knows the claw unless you pass it.

scipy.optimize (Powell) refines the search when installed; without it the
coarse grid + a local pattern search does the job, a little less precisely.
"""
from __future__ import annotations

import math

import numpy as np

import rocky_model as rm
from pebble_gait import WaveGait, N_LEGS, L1, L2, L3, Z_HIP, body_to_leg, leg_ik
from pebble_gestures2 import posed, _rotz, CLAW_MAX
import pebble_feasibility as pf

OFFSET_XY_MM = 40.0
OFFSET_Z_MM = (-45.0, 25.0)
YAW_MAX_DEG = 18.0
_REACH_IN, _REACH_OUT = abs(L2 - L3) + 1.0, L2 + L3 - 1.0
_UP = np.array([0.0, 0.0, 1.0])


def _arm_ik(leg, p_body):
    """(q (3,), miss_mm): IK of the clamped-to-reachable point and how far the
    target lies outside the reachable annulus (0 = reachable)."""
    p = body_to_leg(leg, p_body)
    r = math.hypot(p[0], p[1])
    d = np.array([r - L1, p[2] - Z_HIP])
    dist = float(np.linalg.norm(d))
    miss = max(0.0, dist - _REACH_OUT, _REACH_IN - dist)
    if miss > 0:
        dd = d / max(dist, 1e-9) * min(max(dist, _REACH_IN), _REACH_OUT)
        phi = math.atan2(p[1], p[0])
        rr = L1 + dd[0]
        p = np.array([rr * math.cos(phi), rr * math.sin(phi), Z_HIP + dd[1]])
    return leg_ik(p), miss


def _excess_deg(q, guard=pf.GUARD_DEG):
    """Sum over joints of degrees past the guarded limits (0 = inside)."""
    q = np.atleast_2d(q)
    lo, hi = rm.joint_limits_rad()
    lo = np.array(lo) + math.radians(guard)
    hi = np.array(hi) - math.radians(guard)
    if not np.isfinite(q).all():
        return 1e3
    return float(np.degrees(np.maximum(0, lo - q) + np.maximum(0, q - hi)).sum())


def evaluate(g, leg, target, x, stance_legs, keep_margin_mm):
    """Score one body pose x = (dx, dy, dz, yaw_deg). Returns a dict with the
    joint targets, the CoM margin, the violations and a scalar cost."""
    off = np.asarray(x[:3], float)
    yaw = math.radians(x[3])
    q = posed(g, offset=off, yaw=yaw)[0]
    pb = _rotz(np.asarray(target, float), -yaw) - off          # target in the posed body frame
    q_arm, miss = _arm_ik(leg, pb)
    q[leg] = q_arm
    e_arm = _excess_deg(q_arm)
    e_plant = _excess_deg(q[list(stance_legs)])
    sup = np.zeros(N_LEGS, bool)
    sup[list(stance_legs)] = True
    margin = pf.margins(q, sup, _UP)[0] if np.isfinite(q).all() else -1e3   # level ground: no plane search
    short = max(0.0, keep_margin_mm - margin)
    ok = miss < 1e-6 and e_arm == 0 and e_plant == 0 and short == 0
    cost = (20.0 * miss + 5.0 * e_arm + 5.0 * e_plant + 2.0 * short
            + 0.002 * float(off @ off) + 0.004 * x[3] ** 2)
    return dict(q=q, margin=float(margin), miss=miss, e_arm=e_arm, e_plant=e_plant, ok=ok, cost=cost)


def _clip(x, yaw_fixed):
    x = np.array(x, float)
    x[0:2] = np.clip(x[0:2], -OFFSET_XY_MM, OFFSET_XY_MM)
    x[2] = np.clip(x[2], *OFFSET_Z_MM)
    x[3] = yaw_fixed if yaw_fixed is not None else np.clip(x[3], -YAW_MAX_DEG, YAW_MAX_DEG)
    return x


def solve_reach(g=None, leg=0, target_body_mm=(0.0, 250.0, 60.0), stance_legs=None,
                keep_margin_mm=25.0, yaw_deg=None, claw=None) -> dict:
    """Whole-body reach. target_body_mm: where leg `leg`'s hand (IK foot point)
    must go, in the GROUND frame = the unposed stance's body frame (deck-plane
    centre, floor at z = -body_height, leg 0 = +y) — i.e. a point in the world
    that stays put while the body moves. stance_legs: the legs that stay
    planted (default: all others). yaw_deg: fix the body yaw (None = search).

    Returns dict(body [dx, dy, dz] mm, yaw deg, arm [yaw, hip, knee] deg,
    q (5,3) rad as nested lists, margin mm (CoM), ok, note, frame) where
    `frame` is a keyframe dict ({"body", "yaw", "arm"}) ready for a spec.
    ok=False returns the NEAREST pose (least violation) with a note saying
    what is missing."""
    g = g or WaveGait()
    stance = tuple(i for i in range(N_LEGS) if i != leg) if stance_legs is None else tuple(stance_legs)
    if leg in stance:
        raise ValueError("the reaching leg cannot also be a stance leg")
    if len(stance) < 3:
        raise ValueError("need at least 3 stance legs")
    target = np.asarray(target_body_mm, float)

    def f(x):
        return evaluate(g, leg, target, _clip(x, yaw_deg), stance, keep_margin_mm)["cost"]

    # coarse grid, then refine the best few
    yaws = [yaw_deg] if yaw_deg is not None else [-18.0, -9.0, 0.0, 9.0, 18.0]
    cands = []
    for dx in np.linspace(-OFFSET_XY_MM, OFFSET_XY_MM, 5):
        for dy in np.linspace(-OFFSET_XY_MM, OFFSET_XY_MM, 5):
            for dz in (-45.0, -25.0, -10.0, 0.0, 12.0, 25.0):
                for yw in yaws:
                    x = np.array([dx, dy, dz, yw])
                    cands.append((f(x), x))
    cands.sort(key=lambda c: c[0])
    best_x, best_c = cands[0][1], cands[0][0]
    try:
        from scipy.optimize import minimize
        for c0, x0 in cands[:4]:
            res = minimize(f, x0, method="Powell",
                           options=dict(xtol=0.05, ftol=1e-6, maxfev=1500))
            xr = _clip(res.x, yaw_deg)
            cr = f(xr)
            if cr < best_c:
                best_x, best_c = xr, cr
        method = "grid+powell"
    except ImportError:                                           # pattern search fallback
        x, step = best_x.copy(), np.array([8.0, 8.0, 8.0, 4.0])
        while step.max() > 0.2:
            improved = False
            for j in range(4):
                for sgn in (1, -1):
                    xn = x.copy()
                    xn[j] += sgn * step[j]
                    xn = _clip(xn, yaw_deg)
                    cn = f(xn)
                    if cn < best_c - 1e-9:
                        x, best_c, improved = xn, cn, True
            if not improved:
                step *= 0.5
        best_x = x
        method = "grid+pattern"
    best_x = _clip(best_x, yaw_deg)
    ev = evaluate(g, leg, target, best_x, stance, keep_margin_mm)
    notes = []
    if ev["miss"] > 1e-6:
        notes.append(f"target is {ev['miss']:.0f} mm out of reach even with the body moved")
    if ev["e_arm"] > 0:
        notes.append(f"the arm needs {ev['e_arm']:.1f} deg past its guarded limits")
    if ev["e_plant"] > 0:
        notes.append(f"the planted legs need {ev['e_plant']:.1f} deg past their guarded limits")
    if ev["margin"] < keep_margin_mm:
        notes.append(f"CoM margin {ev['margin']:.0f} mm < {keep_margin_mm:.0f} wanted")
    arm_deg = np.degrees(ev["q"][leg])
    body = [round(float(v), 1) for v in best_x[:3]]
    yaw = round(float(best_x[3]), 1)
    frame = {"body": body, "yaw": yaw, "arm": {str(leg): [round(float(v), 1) for v in arm_deg]}}
    if claw is not None:
        c = np.zeros(N_LEGS)
        c[leg] = float(claw)
        frame["claw"] = c.tolist()
    return dict(body=body, yaw=yaw, arm=[round(float(v), 2) for v in arm_deg],
                q=ev["q"].tolist(), margin=round(ev["margin"], 1), ok=bool(ev["ok"]),
                note="; ".join(notes) if notes else f"reachable ({method})", frame=frame,
                leg=leg, target=target.tolist(), stance_legs=list(stance))


# ------------------------------------------------------------------ teach by demonstration
def pose_from_joint_state(q, g=None, claw=None) -> dict:
    """One recorded joint state (15,) or (5,3) rad -> a keyframe dict.

    Planted legs (within 15 mm of the support plane, pebble_feasibility)
    give the body pose: a rigid 2D fit (yaw + x/y offset) of their nominal
    footholds onto where FK puts them, z = the mean crouch, dz = each planted
    corner's residual height. Every other leg becomes a joint `arm` override
    (degrees). A foot near the floor but > 5 mm off its fitted foothold is
    lifting or landing: it is dropped from the fit (worst first, while >= 3
    remain) and becomes an arm. Returns the frame plus diagnostics under
    "_fit" (planted legs, residual_mm = the worst planted foot's distance
    from its fitted foothold: large = the feet were not on their nominal
    footholds, see the module docstring)."""
    g = g or WaveGait()
    q = np.asarray(q, float).reshape(N_LEGS, 3)
    feet = pf.feet_body(q)
    _n, _p, sup = pf.support_plane(feet, com=pf.com_body(q))
    idx = np.flatnonzero(sup)
    frame = {}
    fitted = _fit_body(g, feet, idx)
    # a foot that is near the floor but NOT on its foothold is lifting or
    # landing: drop the worst one while it is > FIT_TOL_MM off and >= 3 remain
    while fitted is not None and fitted[-1] > FIT_TOL_MM and len(idx) > 3:
        worst = idx[int(np.argmax(fitted[-2]))]
        idx = idx[idx != worst]
        sup[worst] = False
        fitted = _fit_body(g, feet, idx)
    if fitted is not None:
        yaw, off_xy, off_z, dz, _res, resid = fitted
        frame["body"] = [round(float(off_xy[0]), 1), round(float(off_xy[1]), 1), round(off_z, 1)]
        frame["yaw"] = round(math.degrees(yaw), 2)
        if np.any(np.abs(dz) > 0.5):
            frame["dz"] = [round(float(v), 1) for v in dz]
    else:
        resid = float("inf")
    arms = {str(i): [round(float(v), 2) for v in np.degrees(q[i])] for i in range(N_LEGS) if not sup[i]}
    if arms:
        frame["arm"] = arms
    if claw is not None:
        frame["claw"] = [round(float(c), 3) for c in np.clip(np.asarray(claw, float) / CLAW_MAX, 0, 1)]
    frame["_fit"] = dict(planted=idx.tolist(), residual_mm=round(resid, 2))
    return frame


FIT_TOL_MM = 5.0


def _fit_body(g, feet, idx):
    """Rigid 2D fit of the nominal footholds p_nom[idx] onto feet[idx]:
    (yaw, off_xy, off_z, dz[5], per-foot residual, worst residual) or None."""
    if len(idx) < 2:
        return None
    A = g.p_nom[idx, :2]
    B = feet[idx, :2]
    Ac, Bc = A - A.mean(0), B - B.mean(0)
    th = math.atan2(float(np.sum(Ac[:, 0] * Bc[:, 1] - Ac[:, 1] * Bc[:, 0])),
                    float(np.sum(Ac[:, 0] * Bc[:, 0] + Ac[:, 1] * Bc[:, 1])))
    yaw = -th                                                      # feet = rotz(p_nom, -yaw) - offset
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    off_xy = (R @ A.mean(0)) - B.mean(0)
    off_z = -g.h - float(feet[idx, 2].mean())                      # feet z = -h - off_z + dz
    dz = np.zeros(N_LEGS)
    dz[idx] = feet[idx, 2] - (-g.h - off_z)
    fit = np.array([_rotz(g.p_nom[i], -yaw)[:2] - off_xy for i in idx])
    res = np.linalg.norm(fit - B, axis=1)
    return yaw, off_xy, off_z, dz, res, float(res.max())


def rdp_indices(Y, t, tol):
    """Ramer-Douglas-Peucker on a time series: indices whose piecewise-linear
    interpolation (in t) stays within tol of every sample, max-abs over the
    columns of Y (T x D). Iterative, keeps the first and last sample."""
    Y = np.asarray(Y, float)
    keep = {0, len(Y) - 1}
    stack = [(0, len(Y) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        u = (t[i + 1:j] - t[i]) / max(t[j] - t[i], 1e-12)
        interp = Y[i] + u[:, None] * (Y[j] - Y[i])
        err = np.abs(Y[i + 1:j] - interp).max(axis=1)
        k = int(np.argmax(err))
        if err[k] > tol:
            m = i + 1 + k
            keep.add(m)
            stack += [(i, m), (m, j)]
    return sorted(keep)


def keyframes_from_stream(qs, fs: float = 50.0, claw=None, tol_deg: float = 2.0,
                          name: str = "taught", g=None, lead_in_s: float | None = None) -> dict:
    """A recorded joint stream -> a keyframe spec.

    qs: (T, 15) or (T, 5, 3) rad at fs Hz; claw: optional (T, 5) rad. RDP
    keeps the fewest samples whose linear interpolation is within tol_deg of
    every recorded sample (joints and claws). Frames use ease "linear" (RDP's
    own error model). If the recording does not start at the planted stance,
    a nominal frame is put first and everything is delayed by a lead-in long
    enough to get there within the speed budget. The spec carries "taught"
    metadata (samples, frames, tol, worst fit residual); run
    pebble_feasibility.check_spec on it before trusting it — a demonstration
    can be faster than the servo."""
    g = g or WaveGait()
    Q = np.asarray(qs, float).reshape(len(qs), N_LEGS * 3)
    if not np.isfinite(Q).all():
        raise ValueError("the recorded stream has NaN/inf")
    T = len(Q)
    t = np.arange(T) / float(fs)
    Y = np.degrees(Q)
    C = None
    if claw is not None:
        C = np.asarray(claw, float).reshape(T, N_LEGS)
        Y = np.hstack([Y, np.degrees(C)])
    idx = rdp_indices(Y, t, tol_deg)
    frames, worst = [], 0.0
    for k in idx:
        fr = pose_from_joint_state(Q[k], g, None if C is None else C[k])
        worst = max(worst, fr["_fit"]["residual_mm"])
        fr.pop("_fit")
        fr["t"] = round(float(t[k]), 3)
        fr["ease"] = "linear"
        frames.append(fr)
    # lead-in from the planted stance
    q0 = pf.planted_q(g)
    dq = float(np.max(np.abs(Q[0].reshape(N_LEGS, 3) - q0)))
    if lead_in_s is None:
        lead_in_s = 0.0 if math.degrees(dq) <= pf.JUMP_DEG else \
            math.ceil(max(0.6, 1.5 * dq / (0.9 * rm.servo_speed("loaded"))) * 10) / 10
    if lead_in_s > 0:
        for fr in frames:
            fr["t"] = round(fr["t"] + lead_in_s, 3)
        frames[0]["ease"] = "smooth"
        frames.insert(0, {"t": 0.0})
    spec = {"name": name, "keyframes": frames,
            "taught": dict(samples=T, fs=fs, frames=len(frames), tol_deg=tol_deg,
                           fit_residual_mm=round(worst, 2), lead_in_s=lead_in_s)}
    return spec
