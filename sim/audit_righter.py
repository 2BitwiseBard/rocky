"""Righter jitter audit (D048): how staircase-y is a recovery checkpoint?

Runs the fallen demo's shove (run_reflex_fallen.episode) for N seeds with
the given checkpoint and reports, over the FALLEN phase, the servo-target
statistics that make a recovery LOOK jittery:
  pinned   fraction of per-tick joint moves at the checkpoint's rate limit
  rev/s    direction reversals per second per joint (mean over joints)
  |move|   mean per-tick joint move, degrees
  track    mean |qpos - target| while righting, degrees
plus the demo's own verdicts: fell, upright at the end, whether the FIRST
handoff came from the policy (the handoff criterion) or from the stall /
deadline ramp and how long it took, the landing tilt (~180 = on the back, the hard mode) and how many
times FALLEN was entered (a ramp that fails re-enters it). Reference: recover1 on 2026-09-23 measured pinned
0.73, 4-23 rev/s, 4.4 deg/tick — a 50 Hz staircase of 5.7 deg jumps.

  python audit_righter.py runs/recover1/latest.pt runs/recover5_v3/latest.pt --episodes 5
"""
import argparse
import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, N_LEGS                                # noqa: E402
from pebble_reflex import ReflexSupervisor, FALLEN, RIGHTED, NORMAL    # noqa: E402
from run_push_reflex import make_data, gyro_xy_of, T_SETTLE, V_X        # noqa: E402
from run_push_reflex_v2 import TRIP                                     # noqa: E402
from run_reflex_fallen import SHOVE_N, SHOVE_S, T_SHOVE, T_TOTAL        # noqa: E402
from righter import PolicyRighter, foot_contacts                        # noqa: E402
from rocky_recover_env import CTRL_DT                                   # noqa: E402
from shove import Shove                                                 # noqa: E402


def audit_episode(model, ckpt, seed, shove_n=SHOVE_N, shove_s=SHOVE_S):
    rng = np.random.default_rng(seed)
    gait = WaveGait()
    data, q0 = make_data(model, gait)
    torso = model.body("torso").id
    fids = [model.geom(f"foot{i}").id for i in range(N_LEGS)]
    DT = model.opt.timestep
    righter = PolicyRighter(ckpt, model, data, torso, fids)
    sup = ReflexSupervisor(gait, gyro_trip=TRIP, gyro_calm=TRIP / 2, righter=righter)
    az = rng.uniform(0, 2 * np.pi)
    shove = Shove(shove_n * np.cos(az), shove_n * np.sin(az), dur=shove_s, t0=T_SHOVE)
    ticks, last, terr = [], None, []
    t_fallen = t_righted = None               # first fall, first handoff
    land_tilt = None                          # body tilt when FALLEN was declared
    falls = 0                                 # FALLEN entries (re-falls after a ramp count)
    reason = None
    prev = NORMAL
    for k in range(int(T_TOTAL / DT)):
        t = k * DT
        R = data.xmat[torso].reshape(3, 3)
        tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
        w = R.T @ data.cvel[torso][0:3]
        vx = V_X * min(max(t - T_SETTLE, 0.0) / 0.6, 1.0)
        q, state = sup.step(t, vx, 0.0, 0.0, gyro_xy_of(model, data, torso),
                            contacts=foot_contacts(model, data, fids), gyro_vec=w[:2],
                            tilt_deg=tilt, height=float(data.xpos[torso][2]))
        if state == FALLEN and prev != FALLEN:
            falls += 1
            if t_fallen is None:
                t_fallen = t
                land_tilt = tilt
        if state == RIGHTED and prev == FALLEN and t_righted is None:
            t_righted = t
            reason = sup.right_reason
        prev = state
        data.ctrl[:15] = q.flatten()
        data.ctrl[15:20] = 0.0
        if state == FALLEN and righter._target is not None:
            tg = righter._target.copy()
            if last is None or not np.array_equal(tg, last):
                ticks.append(tg)
                last = tg
            terr.append(np.abs(data.qpos[righter._jadr] - data.ctrl[:15]))
        shove.apply(model, data, torso, t)
        mujoco.mj_step(model, data)
    R = data.xmat[torso].reshape(3, 3)
    end_tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    out = dict(seed=seed, fell=t_fallen is not None, upright_end=end_tilt < 25, falls=falls, reason=reason,
               land_tilt=None if land_tilt is None else round(land_tilt),
               by_policy=(reason == "handoff"),
               t_right=None if t_fallen is None or t_righted is None else round(t_righted - t_fallen, 2))
    if len(ticks) > 3:
        T = np.array(ticks)
        d = np.diff(T, axis=0)
        out.update(pinned=float((np.abs(d) >= righter.rate_limit - 1e-6).mean()),
                   rev_s=float(((np.sign(d[1:]) * np.sign(d[:-1])) < 0).sum(0).mean() / (len(T) * CTRL_DT)),
                   move_deg=float(np.degrees(np.abs(d).mean())),
                   track_deg=float(np.degrees(np.mean(terr))))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ckpts", nargs="+")
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--shove", type=float, default=SHOVE_N)
    args = ap.parse_args()
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    print(f"shove {args.shove:.0f} N peak half-sine x {SHOVE_S} s at the rim, {args.episodes} seeds")
    print(f"{'checkpoint':34s} rate  fell  upright  by-policy  t_right  pinned  rev/s  |move|  track")
    for ck in args.ckpts:
        rs = [audit_episode(model, ck, s, args.shove) for s in range(args.episodes)]
        rl = PolicyRighter(ck, model, mujoco.MjData(model), 0, []).rate_limit / CTRL_DT
        have = [r for r in rs if "pinned" in r]
        f = lambda k: np.mean([r[k] for r in have]) if have else float("nan")   # noqa: E731
        tr = [r["t_right"] for r in rs if r["t_right"] is not None]
        lands = [r["land_tilt"] for r in rs if r["land_tilt"] is not None]
        falls = [r["falls"] for r in rs]
        print(f"{os.path.relpath(ck, HERE):34s} {rl:4.1f}  {sum(r['fell'] for r in rs):2d}/{len(rs)}  "
              f"{sum(r['upright_end'] for r in rs):2d}/{len(rs)}    {sum(r['by_policy'] for r in rs):2d}/{len(rs)}     "
              f"{np.mean(tr) if tr else float('nan'):5.1f}   {f('pinned'):5.2f}  {f('rev_s'):5.1f}  "
              f"{f('move_deg'):5.2f}°  {f('track_deg'):4.1f}°   landed at tilt {lands}°, FALLEN entries {falls}, ramp reasons {[r['reason'] for r in rs]}")


if __name__ == "__main__":
    main()
