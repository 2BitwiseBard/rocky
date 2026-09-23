#!/usr/bin/env python3
"""Evaluate a self-righting checkpoint (session 8).

    MUJOCO_GL=osmesa python3 eval_recover.py runs/recover1/latest.pt --episodes 20 --video out.mp4

Reports: success rate (stood for 1 s), end tilt / height distribution,
per-fall-mode breakdown, and the hold-pose / random baselines on the SAME
seeds. Deterministic policy (actor mean). Video: the first success and
the first failure, side by side in time (one after the other).
"""
import argparse
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from rocky_recover_env import RecoverEnv                   # noqa: E402
from train_ppo import Agent, RunningMeanStd                # noqa: E402


def ckpt_args(ckpt):
    """The trainer's argument dict as saved in the checkpoint ({} if absent)."""
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    return dict(ck.get("args") or {})


def load(ckpt):
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    obs_dim, act_dim = ck.get("obs_dim", 39), ck.get("act_dim", 15)
    agent = Agent(obs_dim, act_dim)
    agent.load_state_dict(ck["model"])
    agent.eval()
    rms = RunningMeanStd((obs_dim,))
    rms.load_state_dict(ck["obs_rms"])

    def policy(obs):
        on = (obs - rms.mean) / np.sqrt(rms.var + 1e-8)
        with torch.no_grad():
            return agent.actor(torch.as_tensor(on, dtype=torch.float32).unsqueeze(0)) \
                .squeeze(0).numpy()
    return policy, ck.get("global_step", 0)


def _planted_q0():
    sys.path.insert(0, os.path.join(HERE, "..", "gait"))
    from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS
    g = WaveGait()
    return np.array([leg_ik(body_to_leg(i, g.p_nom[i])) for i in range(N_LEGS)]).flatten()


def run(env, policy, seed, record=None, handoff=True):
    """Hybrid evaluation: the policy rights the body; once it has been
    upright (tilt < 25°) and off the floor (h > 0.09) for 0.5 s, the
    analytic planted pose takes over (ramped in over 0.6 s, held 1.5 s) —
    the way ReflexSupervisor would hand back to the gait. 'stood' = at the
    end of the hold: tilt < 15°, h > 0.11 m, ≥4 feet in contact."""
    import mujoco
    obs, _ = env.reset(seed=seed)
    mode = env.last_mode
    done, ret, frames = False, 0.0, []
    best_tilt, ok_hold, stood, t_handoff = 180.0, 0.0, False, None
    q0 = _planted_q0()
    n = 0
    while not done:
        a = policy(obs)
        obs, r, term, trunc, info = env.step(a)
        n += 1
        ret += r
        best_tilt = min(best_tilt, info["tilt_deg"])
        done = term or trunc
        if record is not None:
            frames.append(env.render())
        if handoff and info["tilt_deg"] < 25 and info["height"] > 0.09:
            ok_hold += 0.02
        else:
            ok_hold = 0.0
        if handoff and ok_hold >= 0.5:
            t_handoff = n * 0.02
            q_start = env.data.qpos[env._jadr].copy()
            for k in range(105):                        # 0.6 s ramp + 1.5 s hold
                a_ = min(1.0, k / 30)
                env.data.ctrl[:15] = (1 - a_) * q_start + a_ * q0
                for _ in range(env.substeps):
                    mujoco.mj_step(env.model, env.data)
                if record is not None:
                    frames.append(env.render())
            _g, _w, tilt, h = env._state()
            feet = env._feet_down()
            stood = bool(np.degrees(tilt) < 15 and h > 0.11 and feet >= 4)
            info = dict(info, tilt_deg=np.degrees(tilt), height=h, feet=feet)
            break
    return dict(seed=seed, mode=mode, success=bool(info.get("success", False)), stood=stood,
                t_handoff=t_handoff, ret=ret, end_tilt=info["tilt_deg"], end_h=info["height"],
                best_tilt=best_tilt, frames=frames)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("ckpt")
    p.add_argument("--episodes", type=int, default=20)
    p.add_argument("--video", type=str, default="")
    p.add_argument("--reward", type=str, default=None, choices=["v1", "v2", "v3"],
                   help="env success criterion for the pure-RL column "
                        "(stood-after-handoff is criterion-agnostic)")
    args = p.parse_args()
    policy, step = load(args.ckpt)
    meta = ckpt_args(args.ckpt)
    reward = args.reward or meta.get("reward", "v1")
    rate_limit = float(meta.get("rate_limit", 5.0))
    env = RecoverEnv(seed=0, reward=reward, rate_limit_rad_s=rate_limit)
    print(f"checkpoint {args.ckpt} @ {step:,} steps | reward {reward} | rate limit {rate_limit} rad/s")
    results = {}
    for name, pol in (("policy", policy),
                      ("hold-pose", lambda o: env._prev_action),
                      ("random", lambda o: env.rng.uniform(-1, 1, 15))):
        rs = [run(env, pol, seed=s) for s in range(args.episodes)]
        results[name] = rs
        succ = sum(r["success"] for r in rs)
        stood = sum(r["stood"] for r in rs)
        print(f"  {name:10s} stood-after-handoff {stood}/{len(rs)}  pure-RL success {succ}/{len(rs)}  mean return {np.mean([r['ret'] for r in rs]):6.1f}  "
              f"end tilt median {np.median([r['end_tilt'] for r in rs]):5.1f}°  "
              f"best tilt median {np.median([r['best_tilt'] for r in rs]):5.1f}°  "
              f"end height median {np.median([r['end_h'] for r in rs])*1000:5.1f} mm")
    rs = results["policy"]
    for mode in ("back", "side", "tumble"):
        sub = [r for r in rs if r["mode"] == mode]
        if sub:
            print(f"    {mode:7s}: {sum(r['stood'] for r in sub)}/{len(sub)} stood, "
                  f"end tilt median {np.median([r['end_tilt'] for r in sub]):5.1f}°")
    if args.video:
        import imageio
        picks = []
        for want in (True, False):
            for r in rs:
                if r["stood"] == want:
                    picks.append(r["seed"])
                    break
        frames = []
        for s in picks:
            frames += run(env, policy, seed=s, record=True)["frames"]
        if frames:
            imageio.mimsave(args.video, frames[::2], fps=25, codec="libx264", quality=7)
            print("wrote", args.video, f"({len(frames)//2} frames, seeds {picks})")


if __name__ == "__main__":
    main()
