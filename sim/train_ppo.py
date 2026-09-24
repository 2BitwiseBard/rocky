#!/usr/bin/env python3
"""PPO training kit for PebbleEnv — laptop-ready (Phase-3 scale-up, D009).

Everything the 16 GB laptop needs in one file: vectorized envs, domain
randomization, observation/return normalization, checkpoint/RESUME (exact:
optimizer, normalizers, RNG), jsonl logging, CPU-first with CUDA auto.

The policy is a RESIDUAL on the analytic wave gait (rocky_env.py): action
zero already walks, so PPO starts from a walking robot and learns trims —
the sane sim-to-real on-ramp. Reward: velocity tracking - action cost -
tilt - height error.

Quick start (laptop):
    python3 train_ppo.py --num-envs 8                 # ~overnight defaults
    python3 train_ppo.py --resume runs/ppo/latest.pt  # continue training
    python3 eval_ppo.py runs/ppo/latest.pt --video out.mp4

Sanity smoke (2 min):
    python3 train_ppo.py --num-envs 2 --rollout 96 --total-steps 4000 \
        --run-dir runs/smoke

Notes
-----
* AsyncVectorEnv forks one MuJoCo per worker; use --sync on platforms where
  fork is flaky (Windows) or for debugging.
* Domain randomization is ON by default (--no-randomize to disable):
  rl_common.DomainRandomizer (friction U(0.5,1.5), per-link mass, torso
  CoM, kp, joint offsets, gravity tilt) + obs noise; --push-prob is the
  probability of one rim shove per episode (gait env).
* D052: --servo off|nominal|random (default random) puts the ST3215 model
  (hold, latency, slew, counts, voltage sag) in the loop;
  --rate-limit defaults to 4.0 rad/s (params 'free' speed) and --ema-alpha
  to 0.4 for NEW runs. The env's config() (obs names/version, servo and DR
  ranges, reward weights, robot fingerprint, MuJoCo version) is saved in
  every checkpoint as env_config; eval_* and the righter replay it.
* gymnasium >= 1.0 vector envs autoreset on the NEXT step by default; this
  loop assumes SAME_STEP (rl_common.make_vec_env pins it) — under
  NEXT_STEP every episode boundary fed PPO one transition whose action was
  ignored and whose reward was 0.
* Checkpoints are atomic (tmp+rename); Ctrl-C saves before exiting.
* KL early stop (--target-kl) keeps the residual policy from tearing up
  the gait prior in one bad update.
"""
from __future__ import annotations
import argparse
import json
import os
import signal
import sys
import time

import numpy as np
import torch
import torch.nn as nn

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rl_common as rc                                         # noqa: E402


# --------------------------------------------------------------- utilities
class RunningMeanStd:
    """Numerically stable streaming mean/var (Welford, batched)."""

    def __init__(self, shape=()):
        self.mean = np.zeros(shape, dtype=np.float64)
        self.var = np.ones(shape, dtype=np.float64)
        self.count = 1e-4

    def update(self, x):
        x = np.asarray(x, dtype=np.float64)
        b_mean = x.mean(axis=0)
        b_var = x.var(axis=0)
        b_count = x.shape[0]
        delta = b_mean - self.mean
        tot = self.count + b_count
        self.mean = self.mean + delta * b_count / tot
        m_a = self.var * self.count
        m_b = b_var * b_count
        m2 = m_a + m_b + delta ** 2 * self.count * b_count / tot
        self.var = m2 / tot
        self.count = tot

    def state_dict(self):
        return dict(mean=self.mean, var=self.var, count=self.count)

    def load_state_dict(self, d):
        self.mean = np.asarray(d["mean"])
        self.var = np.asarray(d["var"])
        self.count = float(d["count"])


def layer_init(layer, std=np.sqrt(2), bias=0.0):
    nn.init.orthogonal_(layer.weight, std)
    nn.init.constant_(layer.bias, bias)
    return layer


class Agent(nn.Module):
    def __init__(self, obs_dim=41, act_dim=15, log_std_init=-1.0):
        """log_std_init: residual policies ride an already-good controller —
        loud exploration WRECKS it and PPO then spends megasteps learning to
        mute its own noise (measured: sigma 0.6 start -> 1.4M steps still
        below the zero-action baseline). sigma ~0.35 starts near the gait
        and explores gently."""
        super().__init__()
        self.critic = nn.Sequential(
            layer_init(nn.Linear(obs_dim, 128)), nn.Tanh(),
            layer_init(nn.Linear(128, 128)), nn.Tanh(),
            layer_init(nn.Linear(128, 1), std=1.0))
        self.actor = nn.Sequential(
            layer_init(nn.Linear(obs_dim, 128)), nn.Tanh(),
            layer_init(nn.Linear(128, 128)), nn.Tanh(),
            layer_init(nn.Linear(128, act_dim), std=0.01))
        self.log_std = nn.Parameter(torch.full((act_dim,), log_std_init))
        self.log_std_max = None      # session 8d: optional cap (--log-std-max).
        # Why it exists: on the recover env the clip(-1,1) + 5 rad/s rate
        # limit make WIDE actions a strategy — saturated samples act as
        # bang-bang control, the policy gradient inflates sigma past 1.0
        # even with ent_coef 0, and the deterministic MEAN stops being
        # where the behavior lives. Capping log_std (e.g. -0.5) forces the
        # mean to carry the policy. No effect unless set.

    def value(self, x):
        return self.critic(x).squeeze(-1)

    def _log_std(self):
        if self.log_std_max is None:
            return self.log_std
        return self.log_std.clamp(max=self.log_std_max)

    def act(self, x, action=None):
        mu = self.actor(x)
        std = self._log_std().exp().expand_as(mu)
        dist = torch.distributions.Normal(mu, std)
        if action is None:
            action = dist.sample()
        logp = dist.log_prob(action).sum(-1)
        ent = dist.entropy().sum(-1)
        return action, logp, ent, self.value(x)


def env_kwargs(args, cmd):
    """The env constructor kwargs a run's args imply (both envs take **_)."""
    return dict(cmd=cmd, randomize=not args.no_randomize, push_prob=args.push_prob,
                cmd_sample=args.cmd_sample, reward=args.reward, rate_limit_rad_s=args.rate_limit,
                servo=args.servo, ema_alpha=args.ema_alpha)


def make_env(seed, env_name="gait", **kw):
    """Vector-env thunk (rl_common.make_env). kw: env constructor kwargs."""
    return rc.make_env(seed, env_name, **kw)


# ------------------------------------------------------------------- train
def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--total-steps", type=int, default=3_000_000)
    p.add_argument("--num-envs", type=int, default=8)
    p.add_argument("--rollout", type=int, default=256, help="steps/env/update")
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--no-anneal-lr", action="store_true")
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    p.add_argument("--update-epochs", type=int, default=5)
    p.add_argument("--minibatches", type=int, default=8)
    p.add_argument("--clip", type=float, default=0.2)
    p.add_argument("--ent-coef", type=float, default=0.003)
    p.add_argument("--vf-coef", type=float, default=0.5)
    p.add_argument("--max-grad-norm", type=float, default=0.5)
    p.add_argument("--target-kl", type=float, default=0.03)
    p.add_argument("--log-std-init", type=float, default=-1.0)
    p.add_argument("--log-std-max", type=float, default=None,
                   help="cap exploration sigma (e.g. -0.5): on clipped+rate-"
                        "limited action spaces sigma otherwise inflates into "
                        "bang-bang and the deterministic mean decays (8d)")
    p.add_argument("--cmd", type=str, default="45,0,0",
                   help="vx,vy(mm/s),wz(rad/s) command")
    p.add_argument("--no-randomize", action="store_true")
    p.add_argument("--cmd-sample", action="store_true",
                   help="re-draw (vx,vy,wz) each episode: train the full "
                        "command envelope (the command is in obs)")
    p.add_argument("--push-prob", type=float, default=0.7,
                   help="gait env: probability of ONE rim shove per episode (U(10,35) N "
                        "half-sine, 0.3-0.5 s; D052 — it was a per-step 0.24 N.s tap)")
    p.add_argument("--env", type=str, default="gait", choices=["gait", "walk", "recover"],
                   help="gait (= walk) = residual walking (PebbleEnv); recover = self-righting")
    p.add_argument("--servo", type=str, default="random", choices=list(rc.SERVO_MODES),
                   help="servo model in the loop: off (ideal, pre-D052), nominal (datasheet), "
                        "random (per-episode latency/slew/voltage draw; D052 default)")
    p.add_argument("--ema-alpha", type=float, default=rc.EMA_ALPHA,
                   help="in-env EMA on the policy output (1.0 = off); its state is in the obs")
    p.add_argument("--reward", type=str, default="v1", choices=["v1", "v2", "v3"],
                   help="recover env only: v2 = handoff-criterion success + feet term (D041); "
                        "v3 = v2 + smoothness cost on the servo target (D048)")
    p.add_argument("--rate-limit", type=float, default=rc.SERVO_SAFE_RAD_S,
                   help="recover env only: policy-side command clamp, rad/s (default 4.0 = params "
                        "'free' speed; the servo model slews on its own after it). Recorded in the "
                        "checkpoint; the righter replays it. v1-v3 runs used 5.0 (> the 4.7 no-load)")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--device", type=str, default="auto")
    p.add_argument("--torch-threads", type=int, default=0,
                   help="0 = leave torch default")
    p.add_argument("--sync", action="store_true", help="SyncVectorEnv")
    p.add_argument("--run-dir", type=str, default=os.path.join(HERE, "runs", "ppo"))
    p.add_argument("--save-every", type=int, default=10, help="updates")
    p.add_argument("--resume", type=str, default="",
                   help="checkpoint path, or 'auto' for <run-dir>/latest.pt")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.env == "walk":
        args.env = "gait"
    cmd = tuple(float(x) for x in args.cmd.split(","))
    device = torch.device(
        args.device if args.device != "auto"
        else ("cuda" if torch.cuda.is_available() else "cpu"))
    if args.torch_threads:
        torch.set_num_threads(args.torch_threads)
    os.makedirs(args.run_dir, exist_ok=True)
    ckpt_path = os.path.join(args.run_dir, "latest.pt")
    log_path = os.path.join(args.run_dir, "train_log.jsonl")

    kw = env_kwargs(args, cmd)
    # the env's own contract, from one throwaway instance in this process
    probe = make_env(args.seed, args.env, **kw)()
    env_config = probe.get_wrapper_attr("config")()
    probe.close()
    env_fns = [make_env(args.seed + i, args.env, **kw) for i in range(args.num_envs)]
    envs = rc.make_vec_env(env_fns, sync=args.sync)
    obs_dim = int(np.prod(envs.single_observation_space.shape))
    act_dim = int(np.prod(envs.single_action_space.shape))
    assert obs_dim == env_config["obs_dim"], (obs_dim, env_config["obs_dim"])

    agent = Agent(obs_dim, act_dim, log_std_init=args.log_std_init).to(device)
    agent.log_std_max = args.log_std_max
    opt = torch.optim.Adam(agent.parameters(), lr=args.lr, eps=1e-5)
    obs_rms = RunningMeanStd((obs_dim,))
    ret_rms = RunningMeanStd(())
    global_step, update0 = 0, 0
    t_prev = 0.0

    if args.resume:
        rp = ckpt_path if args.resume == "auto" else args.resume
        if os.path.exists(rp):
            ck = torch.load(rp, map_location=device, weights_only=False)
            old = rc.checkpoint_contract(ck, env_hint=args.env)
            if old["obs_version"] != env_config["obs_version"] or old["obs_dim"] != obs_dim:
                envs.close()
                raise SystemExit(f"cannot resume {rp}: it was trained on {old['env']} obs "
                                 f"v{old['obs_version']} ({old['obs_dim']} values), this env builds "
                                 f"v{env_config['obs_version']} ({obs_dim}). Pre-D052 checkpoints do not "
                                 f"resume under the D052 contract — start a fresh run")
            if old.get("robot_fingerprint") and old["robot_fingerprint"] != env_config["robot_fingerprint"]:
                print(f"WARNING: resuming a checkpoint trained on robot {old['robot_fingerprint']} "
                      f"against robot {env_config['robot_fingerprint']}")
            agent.load_state_dict(ck["model"])
            opt.load_state_dict(ck["optimizer"])
            obs_rms.load_state_dict(ck["obs_rms"])
            ret_rms.load_state_dict(ck["ret_rms"])
            global_step = ck["global_step"]
            update0 = ck["update"]
            t_prev = ck.get("elapsed", 0.0)
            torch.set_rng_state(ck["torch_rng"].cpu())
            np.random.set_state(ck["np_rng"])
            print(f"resumed {rp}: update {update0}, {global_step:,} steps, "
                  f"{t_prev/60:.1f} min trained")
        else:
            print(f"resume requested but {rp} missing — starting fresh")

    def save(update):
        tmp = ckpt_path + ".tmp"
        torch.save(dict(model=agent.state_dict(), optimizer=opt.state_dict(),
                        obs_rms=obs_rms.state_dict(),
                        ret_rms=ret_rms.state_dict(),
                        global_step=global_step, update=update,
                        elapsed=t_prev + time.time() - t_start,
                        args=vars(args), obs_dim=obs_dim, act_dim=act_dim,
                        env_config=env_config, obs_version=env_config["obs_version"],
                        torch_rng=torch.get_rng_state(),
                        np_rng=np.random.get_state()), tmp)
        os.replace(tmp, ckpt_path)

    torch.manual_seed(args.seed + update0)
    np.random.seed(args.seed + update0)

    n_rollout = args.rollout
    batch = args.num_envs * n_rollout
    mb_size = batch // args.minibatches
    n_updates = args.total_steps // batch

    obs_buf = torch.zeros((n_rollout, args.num_envs, obs_dim))
    act_buf = torch.zeros((n_rollout, args.num_envs, act_dim))
    logp_buf = torch.zeros((n_rollout, args.num_envs))
    rew_buf = torch.zeros((n_rollout, args.num_envs))
    done_buf = torch.zeros((n_rollout, args.num_envs))
    val_buf = torch.zeros((n_rollout, args.num_envs))

    next_obs_np, _ = envs.reset(seed=[args.seed + update0 * 1000 + i
                                      for i in range(args.num_envs)])
    next_done = torch.zeros(args.num_envs)
    ret_acc = np.zeros(args.num_envs)          # discounted return accumulator
    ep_returns, ep_lengths = [], []

    interrupted = {"flag": False}
    signal.signal(signal.SIGINT,
                  lambda *a: interrupted.__setitem__("flag", True))

    update = update0                       # defined even if no update runs (resume with nothing left to train)
    if n_updates <= update0:
        envs.close()
        print(f"nothing to train: the checkpoint is at update {update0} ({global_step:,} steps) and "
              f"--total-steps {args.total_steps:,} allows only {n_updates} updates of {batch}; "
              f"raise --total-steps (it is cumulative) with the same --rollout/--num-envs to continue")
        return 0
    t_start = time.time()
    print(f"device {device} | {args.num_envs} envs "
          f"({'sync' if args.sync else 'async'}) | batch {batch} | "
          f"{n_updates} updates to {args.total_steps:,} steps | "
          f"DR={'off' if args.no_randomize else 'on'} push={args.push_prob} | servo {args.servo} "
          f"ema {args.ema_alpha} | obs v{env_config['obs_version']} ({obs_dim}) | "
          f"{env_config['fingerprint_note']}")

    for update in range(update0 + 1, n_updates + 1):
        if not args.no_anneal_lr:
            frac = 1.0 - (update - 1) / n_updates
            opt.param_groups[0]["lr"] = frac * args.lr

        for step in range(n_rollout):
            global_step += args.num_envs
            obs_rms.update(next_obs_np)
            obs_n = (next_obs_np - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8)
            obs_t = torch.as_tensor(np.clip(obs_n, -10, 10),
                                    dtype=torch.float32)
            obs_buf[step] = obs_t
            done_buf[step] = next_done
            with torch.no_grad():
                a, lp, _, v = agent.act(obs_t.to(device))
            act_buf[step] = a.cpu()
            logp_buf[step] = lp.cpu()
            val_buf[step] = v.cpu()
            next_obs_np, rew, term, trunc, infos = envs.step(
                a.cpu().numpy().astype(np.float32))
            done = np.logical_or(term, trunc)
            ret_acc = ret_acc * args.gamma + rew
            ret_rms.update(ret_acc.copy())
            ret_acc[done] = 0.0
            rew_buf[step] = torch.as_tensor(
                np.clip(rew / np.sqrt(ret_rms.var + 1e-8), -10, 10),
                dtype=torch.float32)
            next_done = torch.as_tensor(done, dtype=torch.float32)
            r_, l_ = rc.episode_stats(infos)
            ep_returns += r_
            ep_lengths += l_

        with torch.no_grad():
            obs_n = (next_obs_np - obs_rms.mean) / np.sqrt(obs_rms.var + 1e-8)
            next_val = agent.value(torch.as_tensor(
                np.clip(obs_n, -10, 10), dtype=torch.float32
            ).to(device)).cpu()
        adv = torch.zeros_like(rew_buf)
        last_gae = torch.zeros(args.num_envs)
        for tstep in reversed(range(n_rollout)):
            if tstep == n_rollout - 1:
                nextnonterm = 1.0 - next_done
                nextval = next_val
            else:
                nextnonterm = 1.0 - done_buf[tstep + 1]
                nextval = val_buf[tstep + 1]
            delta = rew_buf[tstep] + args.gamma * nextval * nextnonterm \
                - val_buf[tstep]
            last_gae = delta + args.gamma * args.gae_lambda * nextnonterm \
                * last_gae
            adv[tstep] = last_gae
        ret = adv + val_buf

        b_obs = obs_buf.reshape(batch, obs_dim).to(device)
        b_act = act_buf.reshape(batch, act_dim).to(device)
        b_logp = logp_buf.reshape(batch).to(device)
        b_adv = adv.reshape(batch).to(device)
        b_ret = ret.reshape(batch).to(device)
        b_val = val_buf.reshape(batch).to(device)

        idx = np.arange(batch)
        kl_stop = False
        pg_l = v_l = ent_l = kl_last = 0.0
        for epoch in range(args.update_epochs):
            np.random.shuffle(idx)
            for s0 in range(0, batch, mb_size):
                mb = idx[s0:s0 + mb_size]
                _, newlogp, ent, newv = agent.act(b_obs[mb], b_act[mb])
                logratio = newlogp - b_logp[mb]
                ratio = logratio.exp()
                with torch.no_grad():
                    kl_last = ((ratio - 1) - logratio).mean().item()
                madv = b_adv[mb]
                madv = (madv - madv.mean()) / (madv.std() + 1e-8)
                pg1 = -madv * ratio
                pg2 = -madv * torch.clamp(ratio, 1 - args.clip, 1 + args.clip)
                pg_loss = torch.max(pg1, pg2).mean()
                v_clip = b_val[mb] + torch.clamp(newv - b_val[mb],
                                                 -args.clip, args.clip)
                v_loss = 0.5 * torch.max((newv - b_ret[mb]) ** 2,
                                         (v_clip - b_ret[mb]) ** 2).mean()
                ent_loss = ent.mean()
                loss = pg_loss - args.ent_coef * ent_loss \
                    + args.vf_coef * v_loss
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(),
                                         args.max_grad_norm)
                opt.step()
                pg_l, v_l, ent_l = pg_loss.item(), v_loss.item(), ent_loss.item()
            if args.target_kl and kl_last > args.target_kl:
                kl_stop = True
                break

        sps = int(global_step / max(time.time() - t_start, 1e-9)) \
            if update0 == 0 else \
            int((global_step - update0 * batch)
                / max(time.time() - t_start, 1e-9))
        rec = dict(update=update, step=global_step,
                   ep_return=round(float(np.mean(ep_returns[-40:])), 2)
                   if ep_returns else None,
                   ep_len=round(float(np.mean(ep_lengths[-40:])), 1)
                   if ep_lengths else None,
                   pg=round(pg_l, 4), v=round(v_l, 4), ent=round(ent_l, 3),
                   kl=round(kl_last, 4), kl_stop=kl_stop,
                   lr=round(opt.param_groups[0]["lr"], 6), sps=sps)
        with open(log_path, "a") as f:
            f.write(json.dumps(rec) + "\n")
        print(f"upd {update:4d}/{n_updates} step {global_step:>9,} "
              f"ret {rec['ep_return']} len {rec['ep_len']} "
              f"kl {rec['kl']:.4f}{'*' if kl_stop else ' '} sps {sps}",
              flush=True)

        if update % args.save_every == 0 or update == n_updates \
                or interrupted["flag"]:
            save(update)
            if interrupted["flag"]:
                print("SIGINT — checkpoint saved, exiting")
                break

    envs.close()
    save(update)
    print(f"done: {global_step:,} steps -> {ckpt_path}")


if __name__ == "__main__":
    main()
