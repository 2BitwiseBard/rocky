#!/usr/bin/env python3
"""FALLEN branch end-to-end (session 8d, D042): the whole reflex stack in
one take — walk, take a shove no brace can hold, tumble, the supervisor
declares FALLEN, the learned self-righting policy gets the joints, the
handoff criterion fires, RIGHTED ramps into the planted stance, and the
robot walks away. One state machine, no operator.

    MUJOCO_GL=osmesa python3 run_reflex_fallen.py                 # metrics
    MUJOCO_GL=osmesa python3 run_reflex_fallen.py --video out.mp4
    MUJOCO_GL=osmesa python3 run_reflex_fallen.py --episodes 10   # stats

The PolicyRighter is the sim-side adapter between the torch-free
supervisor (pebble_reflex.set_righter) and a RecoverEnv checkpoint: it
rebuilds the 39-D obs from live MuJoCo state, applies the checkpoint's
obs normalization, and rate-limits targets exactly like RecoverEnv
(5 rad/s at 50 Hz) so the policy sees the dynamics it trained on.

Honesty (D025/D035): the shove here is chosen to ACTUALLY fell the robot
(120 N x 0.25 s, ~3.4x the D017 walking floor) — a showcase that never
falls tests nothing. Success is judged by post-recovery walking distance,
not by the state trace alone.
"""
import argparse
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, N_LEGS                            # noqa: E402
from pebble_reflex import ReflexSupervisor, FALLEN, RIGHTED, NORMAL  # noqa: E402
from run_push_reflex import make_data, gyro_xy_of, T_SETTLE, V_X    # noqa: E402
from run_push_reflex_v2 import TRIP                                 # noqa: E402
from righter import PolicyRighter, default_ckpt, foot_contacts      # noqa: E402  (D048: shared)
from shove import Shove                                             # noqa: E402

# D048 shove model: half-sine at the shell rim (see shove.py). 40 N peak over
# 0.4 s = 10.2 N·s, 1.5 BW peak: 1.6x the standing tip-over threshold
# (shove_envelope.py: 25 N survives, 30 N tips), fells the walking robot 5/5
# seeds and it ends up ~1 m from where it stood. The old 120 N x 0.25 s CoM
# pulse was 30 N·s and sent it 11-15 m across the floor at ~10 m/s. 45 N
# felled only 3/5 (two seeds rolled through and landed upright), so more is
# not more reliable here.
SHOVE_N = 40.0
SHOVE_S = 0.4
T_SHOVE = 3.0
T_TOTAL = 22.0


def episode(model, ckpt, seed=0, record=False, shove_n=SHOVE_N, shove_s=SHOVE_S):
    rng = np.random.default_rng(seed)
    gait = WaveGait()
    data, q0 = make_data(model, gait)
    torso = model.body("torso").id
    fids = [model.geom(f"foot{i}").id for i in range(N_LEGS)]
    jadr = [model.joint(f"{n}{i}").qposadr[0] for i in range(N_LEGS) for n in ("yaw", "hip", "knee")]
    DT = model.opt.timestep
    righter = PolicyRighter(ckpt, model, data, torso, fids)
    sup = ReflexSupervisor(gait, gyro_trip=TRIP, gyro_calm=TRIP / 2,
                           righter=righter)
    az = rng.uniform(0, 2 * np.pi)
    shove = Shove(shove_n * np.cos(az), shove_n * np.sin(az), dur=shove_s, t0=T_SHOVE)
    frames, renderer = [], None
    if record:
        renderer = mujoco.Renderer(model, 480, 720)
        cam = mujoco.MjvCamera()
    trace, seen = [], set()
    t_fallen = t_righted = None
    pos_righted = None
    pos_shove = None                          # where it stood when shoved
    prev_state = NORMAL
    for k in range(int(T_TOTAL / DT)):
        t = k * DT
        gyro = gyro_xy_of(model, data, torso)
        R = data.xmat[torso].reshape(3, 3)
        tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
        h = float(data.xpos[torso][2])
        con = foot_contacts(model, data, fids)
        w_body = R.T @ data.cvel[torso][0:3]
        vx = V_X * min(max(t - T_SETTLE, 0.0) / 0.6, 1.0)
        q, state = sup.step(t, vx, 0.0, 0.0, gyro, contacts=con,
                            gyro_vec=w_body[:2], tilt_deg=tilt, height=h,
                            q_meas=data.qpos[jadr].reshape(5, 3))   # V2: the handoff_ok criterion
        if state == FALLEN and prev_state != FALLEN:
            t_fallen = t                      # (the supervisor resets the righter)
        if t >= T_SHOVE and pos_shove is None:
            pos_shove = data.xpos[torso][:2].copy()
        if state == RIGHTED and prev_state == FALLEN:
            t_righted = t
        if prev_state in (FALLEN, RIGHTED) and state == NORMAL:
            pos_righted = data.xpos[torso][:2].copy()
        prev_state = state
        if state not in seen:
            seen.add(state)
            trace.append((round(t, 2), state))
        data.ctrl[:15] = q.flatten()
        data.ctrl[15:20] = 0.0
        shove.apply(model, data, torso, t)
        mujoco.mj_step(model, data)
        if record and k % int(0.04 / DT) == 0:
            cam.lookat[:] = data.xpos[torso]
            cam.distance, cam.elevation, cam.azimuth = 1.0, -16, 140
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
    walked = 0.0 if pos_righted is None else \
        float(np.linalg.norm(data.xpos[torso][:2] - pos_righted))
    thrown = 0.0 if pos_shove is None else float(np.linalg.norm(
        (pos_righted if pos_righted is not None else data.xpos[torso][:2]) - pos_shove))
    R = data.xmat[torso].reshape(3, 3)
    end_tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    fell = t_fallen is not None
    ok = fell and end_tilt < 25 and h > 0.07 and walked > 0.10
    return dict(seed=seed, fell=fell, ok=ok, trace=trace,
                t_fallen=t_fallen, t_righted=t_righted,
                walked_after_m=round(walked, 3), thrown_m=round(thrown, 2),
                end_tilt=round(end_tilt, 1),
                end_h=round(h, 3), frames=frames)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default=None)
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--video", default="")
    ap.add_argument("--shove", type=float, default=SHOVE_N, help="peak N at the shell rim")
    ap.add_argument("--shove-s", type=float, default=SHOVE_S, help="half-sine duration, s")
    args = ap.parse_args()
    ckpt = args.ckpt or default_ckpt()
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    print(f"righter: {ckpt}")
    rs = []
    for s in range(args.episodes):
        r = episode(model, ckpt, seed=s, shove_n=args.shove, shove_s=args.shove_s)
        rs.append(r)
        print(f"  seed {s}: fell={r['fell']} ok={r['ok']} "
              f"trace={r['trace']} thrown={r['thrown_m']} m walked_after={r['walked_after_m']} m "
              f"end_tilt={r['end_tilt']}°")
    n_fell = sum(r["fell"] for r in rs)
    n_ok = sum(r["ok"] for r in rs)
    print(f"shove {args.shove:.0f} N peak x {args.shove_s} s (half-sine, shell rim): fell {n_fell}/{len(rs)}, "
          f"full fall->recover->walk cycles {n_ok}/{len(rs)}")
    out = os.path.join(HERE, "out")
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "reflex_fallen.json"), "w") as fh:
        json.dump([{k: v for k, v in r.items() if k != "frames"}
                   for r in rs], fh, indent=1)
    if args.video:
        # first full success, else first fall
        pick = next((r["seed"] for r in rs if r["ok"]),
                    next((r["seed"] for r in rs if r["fell"]), 0))
        r = episode(model, ckpt, seed=pick, record=True, shove_n=args.shove, shove_s=args.shove_s)
        import imageio
        imageio.mimsave(args.video, r["frames"], fps=25,
                        codec="libx264", quality=7)
        print(f"wrote {args.video} (seed {pick}, ok={r['ok']})")


if __name__ == "__main__":
    main()
