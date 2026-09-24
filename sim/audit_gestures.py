#!/usr/bin/env python3
"""Motion feasibility audit (D052): every gesture and the gait, judged twice.

1. KINEMATIC — gait/pebble_feasibility.check on the target stream, with
   MuJoCo self-contact on (sim/pebble.xml): joint limits (2 deg guard),
   speed per class (loaded 3.0 / free 4.0 / hard 4.7 rad/s), entry/exit
   steps, >= 3 feet, CoM margin, slip, robot-on-robot penetration.
2. PHYSICS — the same stream through the servo realism layer (ServoModel:
   50 Hz bus, 20 ms latency, no-load slew (not load-derated, V2), 4096-count
   quantisation) into the D052 model (joint damping = stall / no-load,
   forcerange = continuous torque): how far the joints lag what was asked
   (tracking error), peak tilt, whether a planted foot let go, whether it fell.
   Gaits: distance / yaw achieved vs commanded over the run.

Entries: the canon gestures (pebble_gestures.CANON), GESTURES2, every
gait/gestures/*.json, the adjacent-arm manipulation choreography and the
wave gait on a (vx, vy, wz) grid clipped by WaveGait.budget().

    MUJOCO_GL=egl python sim/audit_gestures.py                 # everything
    python sim/audit_gestures.py --no-physics                  # kinematic only (fast)
    python sim/audit_gestures.py beckon fist_bump --json out.json

Exit code = number of FAIL rows (kinematic FAIL, fell in physics, or —
D052 V2 — a physics tracking lag past TRACK_FAIL_DEG).
Mirrors sim/audit_righter.py's table style.

D052 V2 (review): tracking is part of the verdict. The kinematic check
allows free-leg moves up to 4.0 rad/s, but the D052 MJCF caps a free joint
at forcerange / damping = 3.06 rad/s (test_model_consistency keeps that
mismatch visible as a strict xfail until the owner decides), so "0 FAIL,
nothing falls" overstated what the sim can follow. p95 lag > TRACK_WARN_DEG
warns, > TRACK_FAIL_DEG fails. The thresholds are a first guess: 10 deg is
about what 20 ms latency + a 50 Hz hold cost at 3-4 rad/s; 20 deg is where
a move visibly is not the move that was authored.
"""
import argparse
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
import rocky_model as rm                                                # noqa: E402
import pebble_feasibility as pf                                         # noqa: E402
from pebble_gait import WaveGait, N_LEGS                                # noqa: E402
from pebble_gestures import CANON, CLAW_MAX                             # noqa: E402
from pebble_gestures2 import GESTURES2, kind_of                         # noqa: E402
from pebble_keyframes import load_keyframe_gestures                     # noqa: E402
from pebble_manip_adjacent import AdjacentManip, T_TOTAL as MANIP_TOTAL  # noqa: E402
from contacts import foot_contacts                                      # noqa: E402
from servo_model import ServoModel                                      # noqa: E402

XML = os.path.join(HERE, "pebble.xml")
T_SETTLE, T_TAIL = 0.8, 0.8
GAIT_RUN_S = 6.0
TRACK_WARN_DEG, TRACK_FAIL_DEG = 10.0, 20.0     # p95 |q - asked| (V2; a first guess, see above)


def track_verdict(phys):
    """None | 'warn' | 'fail' for a physics result's p95 tracking lag."""
    if phys is None:
        return None
    p = phys["track_p95"]
    return "fail" if p > TRACK_FAIL_DEG else ("warn" if p > TRACK_WARN_DEG else None)


def entries(g):
    """name -> (fn, total, kind, q_to) for every gesture source."""
    out = {}
    for name, (fn, total) in CANON.items():
        out[name] = (fn, total, "static", None)
    for name, (fn, total, _ev) in GESTURES2.items():
        out[name] = (fn, total, kind_of(name), None)
    for name, kg in load_keyframe_gestures().items():
        q_to = False if kg.end == "hold" else None
        out[f"kf:{name}"] = (kg, kg.total, "static", q_to)
    m = AdjacentManip(g)

    def manip(gg, t, m=m):
        q, c, _n, _p = m.targets(t)
        return q, c * CLAW_MAX
    out["manip_adjacent"] = (manip, MANIP_TOTAL, "static", None)
    return out


def gait_grid(g):
    """Commands asked for -> what budget() lets through."""
    asks = [(45, 0, 0), (0, 45, 0), (-32, 32, 0), (60, 0, 0), (0, 0, 0.35), (0, 0, -0.5),
            (45, 0, 0.35), (30, 30, 0.2)]
    mc = g.max_command()
    asks += [(mc["v"], 0, 0), (0, 0, mc["wz"])]
    seen, out = set(), []
    for a in asks:
        b = tuple(round(v, 4) for v in g.budget(*a))
        if b not in seen:
            seen.add(b)
            out.append((a, b))
    return out


# ------------------------------------------------------------------ physics
class Sim:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(XML)
        m = self.model
        self.torso = m.body("torso").id
        self.jadr = np.array([m.joint(f"{n}{i}").qposadr[0] for i in range(N_LEGS)
                              for n in rm.LEG_JOINTS])
        self.fids = [m.geom(f"foot{i}").id for i in range(N_LEGS)]
        self.DT = m.opt.timestep

    def reset(self, q0):
        d = mujoco.MjData(self.model)
        d.qpos[0:3] = [0, 0, rm.spawn_z_m()]
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[self.jadr] = q0.ravel()
        d.ctrl[:15] = q0.ravel()
        mujoco.mj_forward(self.model, d)
        sm = ServoModel(on=True)
        sm.reset(q0.ravel())
        return d, sm

    def tilt(self, d):
        R = d.xmat[self.torso].reshape(3, 3)
        return float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))

    def yaw(self, d):
        R = d.xmat[self.torso].reshape(3, 3)
        return float(np.arctan2(R[1, 0], R[0, 0]))

    def run(self, fn, total, g, planted_legs=None):
        """Settle, play fn, tail. Tracking = |qpos - commanded target| (deg) while playing."""
        q0 = pf.planted_q(g)
        d, sm = self.reset(q0)
        err, tilt_max, breaks = [], 0.0, 0.0
        state = np.ones(N_LEGS, bool)
        x0, yaw0, yaw_prev, yaw_acc = None, None, 0.0, 0.0
        n = int((T_SETTLE + total + T_TAIL) / self.DT)
        for k in range(n):
            t = k * self.DT
            tg = min(max(t - T_SETTLE, 0.0), total)
            if t < T_SETTLE:
                q, claw = q0, np.zeros(N_LEGS)
            else:
                q, claw = fn(g, tg)[:2]
            q = np.asarray(q, float).reshape(N_LEGS, 3)
            d.ctrl[:15] = sm.filter(q.ravel(), self.DT, force=d.actuator_force[:15])
            d.ctrl[15:20] = claw
            mujoco.mj_step(self.model, d)
            if t >= T_SETTLE and x0 is None:
                x0, yaw0 = d.xpos[self.torso].copy(), self.yaw(d)
                yaw_prev = yaw0
            if T_SETTLE <= t <= T_SETTLE + total:
                err.append(np.abs(d.qpos[self.jadr] - q.ravel()))
                tilt_max = max(tilt_max, self.tilt(d))
                y = self.yaw(d)
                yaw_acc += (y - yaw_prev + np.pi) % (2 * np.pi) - np.pi
                yaw_prev = y
                if planted_legs is not None:
                    c = foot_contacts(self.model, d, self.fids, state=state)
                    if not all(c[i] for i in planted_legs):
                        breaks += self.DT
        E = np.degrees(np.array(err))
        disp = (d.xpos[self.torso] - x0) * 1000.0
        fell = self.tilt(d) > 25 or d.xpos[self.torso][2] < 0.05
        return dict(track_mean=float(E.mean()), track_p95=float(np.percentile(E.max(axis=1), 95)),
                    track_max=float(E.max()), tilt_max=tilt_max, breaks=breaks, fell=bool(fell),
                    disp=disp[:2].tolist(), yaw_deg=float(np.degrees(yaw_acc)))


def _planted_legs(fn, total, g):
    """Legs that stay planted for the whole gesture (the physics foot-break check)."""
    keep = np.ones(N_LEGS, bool)
    for t in np.linspace(0, total, 60):
        q = np.asarray(fn(g, t)[0]).reshape(N_LEGS, 3)
        if np.isfinite(q).all():
            keep &= pf.support_plane(pf.feet_body(q), com=pf.com_body(q))[2]
    return [i for i in range(N_LEGS) if keep[i]]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*")
    ap.add_argument("--no-physics", action="store_true")
    ap.add_argument("--no-gait", action="store_true")
    ap.add_argument("--fs", type=float, default=100.0)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    g = WaveGait()
    sim = Sim()
    sc = pf._SelfContact(sim.model)
    V = pf.speed_limits()
    ents = entries(g)
    if args.names:
        ents = {k: v for k, v in ents.items() if k in args.names or k.split(":")[-1] in args.names}
    print(f"D052 feasibility audit | {rm.summary()}")
    print(f"gait T {g.T} s, duty {g.duty}, step {g.hstep} mm | envelope {g.max_command()['v']:.1f} mm/s, "
          f"{g.max_command()['wz']:.3f} rad/s | physics: ServoModel on, "
          f"{'off' if args.no_physics else 'D052 model'}")
    head = (f"{'entry':22s} {'kind':6s} {'dur':>5s}  {'peak':>5s} {'joint':9s} {'hot%':>4s} {'lim':>3s} "
            f"{'jmp':>3s} {'sup':>3s} {'CoM':>5s} {'orig':>5s} {'acc g':>5s} {'self':>4s}")
    if not args.no_physics:
        head += f"  {'track':>5s} {'p95':>5s} {'tilt':>5s} {'brk s':>5s}"
    head += "  verdict"
    print(head)
    rows, nfail = [], 0

    def row(name, rep, phys):
        nonlocal nfail
        v, i, jn, _t = rep.peak()
        lim = sum(1 for c in rep.fails if c.startswith("LIMIT"))
        verdict = "PASS" if rep.ok else "FAIL " + ",".join(rep.fails)
        if rep.ok and rep.warnings:
            verdict = "PASS (warn " + ",".join(rep.warnings) + ")"
        tv = track_verdict(phys)
        if tv == "warn" and rep.ok:
            verdict = verdict[:-1] + ",TRACK)" if verdict.endswith(")") else verdict + " (warn TRACK)"
        if tv == "fail":
            verdict = ("FAIL TRACK" if rep.ok else verdict + ",TRACK")
        if phys and phys["fell"]:
            verdict = "FAIL fell" + ("" if rep.ok else " + " + ",".join(rep.fails))
        bad = (not rep.ok) or (phys is not None and phys["fell"]) or tv == "fail"
        nfail += bad
        s = (f"{name:22s} {rep.kind:6s} {rep.total:5.2f}  {v:5.2f} {f'L{i} {jn}':9s} "
             f"{100 * rep.hot_frac.max():4.0f} {lim:3d} {len(rep.jumps):3d} {rep.support_min:3d} "
             f"{rep.margin_min:5.0f} {rep.origin_margin_min:5.0f} {rep.acc_max_g:5.2f} "
             f"{rep.self_contacts if rep.self_contacts is not None else '-':>4}")
        if phys is not None:
            s += (f"  {phys['track_mean']:5.2f} {phys['track_p95']:5.1f} {phys['tilt_max']:5.1f} "
                  f"{phys['breaks']:5.2f}")
        s += "  " + verdict
        print(s, flush=True)
        rows.append(dict(name=name, report=rep.to_json(), physics=phys, fail=bool(bad)))

    for name, (fn, total, kind, q_to) in ents.items():
        rep = pf.check(fn, total, g=g, fs=args.fs, kind=kind, model=sc, name=name, q_to=q_to)
        phys = None
        if not args.no_physics:
            planted = _planted_legs(fn, total, g) if kind == "static" else None
            phys = sim.run(fn, total, g, planted)
        row(name, rep, phys)

    if not args.no_gait and not args.names:
        print(f"\n{'gait asked -> budget':38s} {'peak':>5s} {'joint':9s} {'hot%':>4s} {'lim':>3s} "
              f"{'sup':>3s} {'CoM':>5s}" + ("" if args.no_physics else
                                            f"  {'track':>5s} {'p95':>5s} {'tilt':>5s} "
                                            f"{'got mm':>7s} {'want':>5s} {'got deg':>7s} {'want':>5s}")
              + "  verdict")
        for ask, cmd in gait_grid(g):
            rep = pf.check_gait(g, cmd, model=sc)
            v, i, jn, _t = rep.peak()
            lim = sum(1 for c in rep.fails if c.startswith("LIMIT"))
            label = f"({ask[0]:g},{ask[1]:g},{ask[2]:g}) -> ({cmd[0]:.1f},{cmd[1]:.1f},{cmd[2]:.3f})"
            s = (f"{label:38s} {v:5.2f} {f'L{i} {jn}':9s} {100 * rep.hot_frac.max():4.0f} {lim:3d} "
                 f"{rep.support_min:3d} {rep.margin_min:5.0f}")
            phys = None
            if not args.no_physics:
                fn = pf.gait_fn(cmd)

                def ramped(gg, t, fn=fn, cmd=cmd):
                    env = min(1.0, t / 0.6)
                    q, _st, _f = gg.joint_targets(t, cmd[0] * env, cmd[1] * env, cmd[2] * env)
                    return q, np.zeros(N_LEGS)
                phys = sim.run(ramped, GAIT_RUN_S, g)
                want_mm = float(np.hypot(cmd[0], cmd[1]) * (GAIT_RUN_S - 0.3))
                want_deg = float(np.degrees(abs(cmd[2]) * (GAIT_RUN_S - 0.3)))
                got_mm = float(np.hypot(*phys["disp"]))
                s += (f"  {phys['track_mean']:5.2f} {phys['track_p95']:5.1f} {phys['tilt_max']:5.1f} "
                      f"{got_mm:7.0f} {want_mm:5.0f} {abs(phys['yaw_deg']):7.1f} {want_deg:5.1f}")
                phys.update(want_mm=want_mm, got_mm=got_mm, want_deg=want_deg)
            tv = track_verdict(phys)
            bad = (not rep.ok) or (phys is not None and phys["fell"]) or tv == "fail"
            nfail += bad
            verdict = ("PASS" if rep.ok else "FAIL " + ",".join(rep.fails)) + \
                (" FELL" if phys and phys["fell"] else "") + \
                {"warn": " (warn TRACK)", "fail": " FAIL TRACK"}.get(tv, "")
            print(s + "  " + verdict, flush=True)
            rows.append(dict(name=f"gait{cmd}", ask=list(ask), cmd=list(cmd), report=rep.to_json(),
                             physics=phys, fail=bool(bad)))

    print(f"\n{len(rows)} rows, {nfail} FAIL | limits: loaded {V['loaded']} / free {V['free']} / "
          f"hard {V['hard']} rad/s, claw {V['claw']}, guard {pf.GUARD_DEG} deg, margin fail "
          f"{pf.MARGIN_FAIL_MM:g} / warn {pf.MARGIN_WARN_MM:g} mm, slip mu {pf.MU_SLIDE:g}, "
          f"track p95 warn {TRACK_WARN_DEG:g} / fail {TRACK_FAIL_DEG:g} deg")
    print("columns: peak = fastest leg joint (rad/s); hot% = worst joint's share of samples above 0.6x "
          "its class limit; lim/jmp = limit codes / steps; sup = min feet down; CoM/orig = static "
          "margin (mm) from the CoM model / the old body-origin proxy; self = self-contact samples; "
          "track = mean |q - asked| (deg), p95 = 95th pct of the worst joint; brk = s a planted foot "
          "was off the floor")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(rows, f, indent=1)
    sys.exit(nfail)


if __name__ == "__main__":
    main()
