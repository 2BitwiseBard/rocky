#!/usr/bin/env python3
"""Replay a PPO checkpoint into a rendered video + walking metrics.

    python3 eval_ppo.py runs/ppo/latest.pt --video ppo_eval.mp4
    python3 eval_ppo.py runs/ppo/latest.pt --episodes 5 --compare-zero
    MUJOCO_GL=osmesa ...   (headless boxes; the laptop's desktop GL is fine)

Reports per episode: return, distance along the command, mean speed, max
tilt, fall/upright, mean |residual| (how hard the policy works against the
analytic gait). --compare-zero also rolls the pure gait (action = 0) under
the SAME conditions — the number to beat.
"""
from __future__ import annotations
import argparse
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from rocky_env import PebbleEnv, CTRL_DT                    # noqa: E402
from train_ppo import Agent, RunningMeanStd                 # noqa: E402


def rollout(env, policy_fn, record=None):
    obs, _ = env.reset()
    done = False
    total_r, steps = 0.0, 0
    x0 = env.data.xpos[env.torso].copy()
    tilt_max, res_acc = 0.0, 0.0
    fell = False
    while not done:
        a = policy_fn(obs)
        obs, r, term, trunc, info = env.step(a)
        total_r += r
        steps += 1
        tilt_max = max(tilt_max, info["tilt_deg"])
        res_acc += float(np.mean(np.abs(a)))
        fell |= bool(term)
        done = term or trunc
        if record is not None:
            record.append(env.render())
    x1 = env.data.xpos[env.torso].copy()
    cmd = env.cmd
    cn = np.linalg.norm(cmd[:2])
    dvec = (x1 - x0)[:2] * 1000.0
    dist = float(dvec @ (cmd[:2] / cn)) if cn > 0 else float(
        np.linalg.norm(dvec))
    return dict(ret=round(total_r, 1), dist_mm=round(dist),
                speed_mm_s=round(dist / (steps * CTRL_DT), 1),
                tilt_max=round(tilt_max, 1), fell=fell,
                mean_abs_residual=round(res_acc / max(steps, 1), 3))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("ckpt")
    p.add_argument("--episodes", type=int, default=3)
    p.add_argument("--video", type=str, default="")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--cmd", type=str, default="")
    p.add_argument("--randomize", action="store_true")
    p.add_argument("--push-prob", type=float, default=0.0)
    p.add_argument("--stochastic", action="store_true",
                   help="sample actions instead of the mean")
    p.add_argument("--compare-zero", action="store_true")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    agent = Agent(ck.get("obs_dim", 41), ck.get("act_dim", 15))
    agent.load_state_dict(ck["model"])
    agent.eval()
    obs_rms = RunningMeanStd((ck.get("obs_dim", 41),))
    obs_rms.load_state_dict(ck["obs_rms"])
    cmd = tuple(float(x) for x in (args.cmd or
                                   ck["args"].get("cmd", "45,0,0")).split(","))
    print(f"checkpoint: update {ck['update']}, {ck['global_step']:,} steps, "
          f"{ck.get('elapsed', 0)/60:.0f} min trained | cmd {cmd}")

    def policy(obs):
        on = (obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8)
        ot = torch.as_tensor(np.clip(on, -10, 10), dtype=torch.float32)
        with torch.no_grad():
            if args.stochastic:
                a, *_ = agent.act(ot.unsqueeze(0))
                return a.squeeze(0).numpy()
            return agent.actor(ot.unsqueeze(0)).squeeze(0).numpy()

    frames = [] if args.video else None
    env = PebbleEnv(cmd=cmd, randomize=args.randomize,
                    push_prob=args.push_prob,
                    render_mode="rgb_array" if args.video else None,
                    seed=args.seed)
    stats = []
    for ep in range(args.episodes):
        rec = frames if (frames is not None and ep == 0) else None
        s = rollout(env, policy, record=rec)
        stats.append(s)
        print(f"  policy ep{ep}: {s}")
    mean_ret = np.mean([s["ret"] for s in stats])
    mean_d = np.mean([s["dist_mm"] for s in stats])
    print(f"policy mean: return {mean_ret:.1f}, dist {mean_d:.0f} mm")

    if args.compare_zero:
        env0 = PebbleEnv(cmd=cmd, randomize=args.randomize,
                         push_prob=args.push_prob, seed=args.seed)
        z = [rollout(env0, lambda o: np.zeros(15))
             for _ in range(args.episodes)]
        for ep, s in enumerate(z):
            print(f"  zero   ep{ep}: {s}")
        print(f"zero-action mean: return "
              f"{np.mean([s['ret'] for s in z]):.1f}, dist "
              f"{np.mean([s['dist_mm'] for s in z]):.0f} mm")

    if frames:
        import imageio
        out = args.video
        imageio.mimsave(out, frames, fps=args.fps, codec="libx264", quality=8)
        print(f"video ({len(frames)} frames) -> {out}")


if __name__ == "__main__":
    main()
