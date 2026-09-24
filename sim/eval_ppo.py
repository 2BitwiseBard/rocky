#!/usr/bin/env python3
"""Replay a PPO checkpoint into a rendered video + walking metrics.

    python3 eval_ppo.py runs/ppo/latest.pt --video ppo_eval.mp4
    python3 eval_ppo.py runs/ppo/latest.pt --episodes 5 --compare-zero
    python3 eval_ppo.py CKPT --servo nominal     # force the servo model on (legacy checkpoints)
    MUJOCO_GL=egl ...   (headless; osmesa without a GPU)

Reports per episode: return, distance along the command, mean speed, max
tilt, fall/upright, mean |residual| (how hard the policy works against the
analytic gait; the CLIPPED action the env applies — before D052 this was
the raw actor mean and could exceed 1) and mean |delta residual| per tick
(how much it chatters; D052). --compare-zero also rolls the pure gait
(action = 0) under the SAME conditions and seeds — the number to beat.

D052: the checkpoint is replayed under its own contract (obs version,
EMA, servo mode; rl_common.checkpoint_contract). Pre-D052 checkpoints are
'legacy obs' and replay with the ideal servo unless --servo says otherwise.
Episode ep uses env.reset(seed=args.seed + ep) for policy AND zero runs, so
with --randomize / --push-prob both columns see identical draws.
"""
from __future__ import annotations
import argparse
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rl_common as rc                                      # noqa: E402
from rocky_env import PebbleEnv, CTRL_DT                    # noqa: E402
from train_ppo import Agent, RunningMeanStd                 # noqa: E402


def rollout(env, policy_fn, seed=None, record=None):
    obs, _ = env.reset(seed=seed)
    done = False
    total_r, steps = 0.0, 0
    x0 = env.data.xpos[env.torso].copy()
    tilt_max, res_acc, dres_acc = 0.0, 0.0, 0.0
    a_prev = np.zeros(15)
    fell = False
    while not done:
        a = np.clip(np.asarray(policy_fn(obs), float), -1, 1)
        obs, r, term, trunc, info = env.step(a)
        total_r += r
        steps += 1
        tilt_max = max(tilt_max, float(info["tilt_deg"]))
        res_acc += float(np.mean(np.abs(a)))
        dres_acc += float(np.mean(np.abs(a - a_prev)))
        a_prev = a
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
                mean_abs_residual=round(res_acc / max(steps, 1), 3),
                mean_abs_dresidual=round(dres_acc / max(steps, 1), 3),
                shoved=env.shove is not None)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("ckpt")
    p.add_argument("--episodes", type=int, default=3)
    p.add_argument("--video", type=str, default="")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--cmd", type=str, default="")
    p.add_argument("--randomize", action="store_true",
                   help="DR + obs noise (and the servo draw if --servo random)")
    p.add_argument("--push-prob", type=float, default=0.0,
                   help="probability of one rim shove per episode")
    p.add_argument("--servo", type=str, default=None, choices=list(rc.SERVO_MODES),
                   help="servo model (default: the checkpoint's, made deterministic; legacy = off)")
    p.add_argument("--stochastic", action="store_true",
                   help="sample actions instead of the mean")
    p.add_argument("--compare-zero", action="store_true")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    ck = torch.load(args.ckpt, map_location="cpu", weights_only=False)
    contract = rc.checkpoint_contract(ck, env_hint="gait")
    rc.check_obs_contract(contract, "gait")
    obs_dim = int(ck.get("obs_dim", contract["obs_dim"]))
    agent = Agent(obs_dim, ck.get("act_dim", 15))
    agent.load_state_dict(ck["model"])
    agent.eval()
    obs_rms = RunningMeanStd((obs_dim,))
    obs_rms.load_state_dict(ck["obs_rms"])
    cmd = tuple(float(x) for x in (args.cmd or
                                   ck["args"].get("cmd", "45,0,0")).split(","))
    trained = contract.get("servo", "off")
    servo = args.servo or ("off" if trained == "off" else "nominal")
    env_kw = dict(cmd=cmd, randomize=args.randomize, push_prob=args.push_prob, servo=servo,
                  ema_alpha=contract.get("ema_alpha", 1.0), obs_version=contract["obs_version"],
                  gait_params=contract.get("gait"),
                  cmd_budget=bool(contract.get("cmd_budget", False)))   # V2: replay what it trained on

    def policy(obs):
        on = (obs - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8)
        ot = torch.as_tensor(np.clip(on, -10, 10), dtype=torch.float32)
        with torch.no_grad():
            if args.stochastic:
                a, *_ = agent.act(ot.unsqueeze(0))
                return a.squeeze(0).numpy()
            return agent.actor(ot.unsqueeze(0)).squeeze(0).numpy()

    frames = [] if args.video else None
    env = PebbleEnv(render_mode="rgb_array" if args.video else None, seed=args.seed, **env_kw)
    fp = rc.check_fingerprint(contract, env.model, who=args.ckpt)
    print(f"checkpoint: update {ck['update']}, {ck['global_step']:,} steps, "
          f"{ck.get('elapsed', 0)/60:.0f} min trained | cmd {cmd} | obs v{contract['obs_version']} "
          f"| servo {servo} (trained {trained}) | ema {env_kw['ema_alpha']} | gait T {env.gait.T} s "
          f"step {env.gait.hstep} mm | robot fingerprint {fp} "
          f"| [{', '.join(contract['flags']) or 'D052 contract'}]")
    stats = []
    for ep in range(args.episodes):
        rec = frames if (frames is not None and ep == 0) else None
        s = rollout(env, policy, seed=args.seed + ep, record=rec)
        stats.append(s)
        print(f"  policy ep{ep}: {s}")
    print(f"policy mean: return {np.mean([s['ret'] for s in stats]):.1f}, dist "
          f"{np.mean([s['dist_mm'] for s in stats]):.0f} mm, |res| "
          f"{np.mean([s['mean_abs_residual'] for s in stats]):.3f}, |dres| "
          f"{np.mean([s['mean_abs_dresidual'] for s in stats]):.3f}, fell "
          f"{sum(s['fell'] for s in stats)}/{len(stats)}")

    if args.compare_zero:
        env0 = PebbleEnv(seed=args.seed, **env_kw)
        z = [rollout(env0, lambda o: np.zeros(15), seed=args.seed + ep)
             for ep in range(args.episodes)]
        for ep, s in enumerate(z):
            print(f"  zero   ep{ep}: {s}")
        print(f"zero-action mean: return "
              f"{np.mean([s['ret'] for s in z]):.1f}, dist "
              f"{np.mean([s['dist_mm'] for s in z]):.0f} mm, fell {sum(s['fell'] for s in z)}/{len(z)}")

    if frames:
        import imageio
        out = args.video
        imageio.mimsave(out, frames, fps=args.fps, codec="libx264", quality=8)
        print(f"video ({len(frames)} frames) -> {out}")


if __name__ == "__main__":
    main()
