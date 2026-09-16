"""PPO smoke test — proves the RL loop end-to-end on PebbleEnv (plan §5.3).

Self-contained ~150-line PPO (torch CPU, no stable-baselines dependency):
Gaussian MLP policy + value head, GAE(λ), clipped surrogate. This is NOT a
training run — it's the plumbing check that unblocks the real Phase-3 RL
work on the 16 GB-VRAM laptop: env API, batching, advantage math, checkpoint
save/load all exercised.

Pass criteria (asserted):
  * losses finite every update, approx-KL stays < 0.15
  * mean return over iterations does not collapse (>60 % of iter-1)
  * checkpoint round-trips (save -> load -> same action)

Usage: MUJOCO_GL=osmesa python3 ppo_smoke.py [--iters 3] [--horizon 1024]
"""
import argparse
import json
import os
import sys

import numpy as np
import torch
import torch.nn as nn

HERE = os.path.dirname(os.path.abspath(__file__))
from rocky_env import PebbleEnv                                     # noqa: E402

torch.manual_seed(0)


class ActorCritic(nn.Module):
    def __init__(self, obs_dim=41, act_dim=15, hidden=128):
        super().__init__()
        self.pi = nn.Sequential(nn.Linear(obs_dim, hidden), nn.Tanh(),
                                nn.Linear(hidden, hidden), nn.Tanh(),
                                nn.Linear(hidden, act_dim))
        self.v = nn.Sequential(nn.Linear(obs_dim, hidden), nn.Tanh(),
                               nn.Linear(hidden, hidden), nn.Tanh(),
                               nn.Linear(hidden, 1))
        self.log_std = nn.Parameter(torch.full((act_dim,), -1.0))
        # start the policy NEAR ZERO: the residual formulation means
        # "do nothing" already walks — small init keeps early exploration safe
        with torch.no_grad():
            self.pi[-1].weight *= 0.01
            self.pi[-1].bias.zero_()

    def dist(self, obs):
        mu = self.pi(obs)
        return torch.distributions.Normal(mu, self.log_std.exp())

    def act(self, obs):
        with torch.no_grad():
            d = self.dist(obs)
            a = d.sample()
            return (a, d.log_prob(a).sum(-1), self.v(obs).squeeze(-1))


def rollout(env, ac, horizon):
    O, A, LP, R, D, V = [], [], [], [], [], []
    obs, _ = env.reset()
    ep_returns, ep_ret = [], 0.0
    for _ in range(horizon):
        to = torch.as_tensor(obs, dtype=torch.float32)
        a, lp, v = ac.act(to)
        nobs, r, term, trunc, _ = env.step(np.tanh(a.numpy()))
        O.append(obs); A.append(a.numpy()); LP.append(lp.item())
        R.append(r); D.append(float(term)); V.append(v.item())
        ep_ret += r
        obs = nobs
        if term or trunc:
            ep_returns.append(ep_ret)
            ep_ret = 0.0
            obs, _ = env.reset()
    with torch.no_grad():
        last_v = ac.v(torch.as_tensor(obs, dtype=torch.float32)).item()
    return (np.array(O, np.float32), np.array(A, np.float32),
            np.array(LP, np.float32), np.array(R, np.float32),
            np.array(D, np.float32), np.array(V, np.float32),
            last_v, ep_returns)


def gae(R, D, V, last_v, gamma=0.99, lam=0.95):
    T = len(R)
    adv = np.zeros(T, np.float32)
    nxt = 0.0
    for t in reversed(range(T)):
        v_next = last_v if t == T - 1 else V[t + 1]
        nonterm = 1.0 - D[t]
        delta = R[t] + gamma * v_next * nonterm - V[t]
        adv[t] = nxt = delta + gamma * lam * nonterm * nxt
    return adv, adv + V


def update(ac, opt, O, A, LP_old, adv, ret, epochs=4, minibatch=256, clip=0.2):
    O = torch.as_tensor(O); A = torch.as_tensor(A)
    LP_old = torch.as_tensor(LP_old)
    adv_t = torch.as_tensor((adv - adv.mean()) / (adv.std() + 1e-8))
    ret_t = torch.as_tensor(ret)
    n = len(O)
    kls, pls, vls = [], [], []
    for _ in range(epochs):
        idx = torch.randperm(n)
        for k in range(0, n, minibatch):
            b = idx[k:k + minibatch]
            d = ac.dist(O[b])
            lp = d.log_prob(A[b]).sum(-1)
            ratio = (lp - LP_old[b]).exp()
            s1 = ratio * adv_t[b]
            s2 = torch.clamp(ratio, 1 - clip, 1 + clip) * adv_t[b]
            pi_loss = -torch.min(s1, s2).mean() - 0.001 * d.entropy().sum(-1).mean()
            v_loss = ((ac.v(O[b]).squeeze(-1) - ret_t[b]) ** 2).mean()
            loss = pi_loss + 0.5 * v_loss
            opt.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(ac.parameters(), 0.5)
            opt.step()
            kls.append((LP_old[b] - lp).mean().item())
            pls.append(pi_loss.item()); vls.append(v_loss.item())
    return float(np.mean(kls)), float(np.mean(pls)), float(np.mean(vls))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=3)
    ap.add_argument("--horizon", type=int, default=1024)
    args = ap.parse_args()

    env = PebbleEnv()
    ac = ActorCritic()
    opt = torch.optim.Adam(ac.parameters(), lr=3e-4)
    log = []
    for it in range(1, args.iters + 1):
        O, A, LP, R, D, V, last_v, eps = rollout(env, ac, args.horizon)
        adv, ret = gae(R, D, V, last_v)
        kl, pl, vl = update(ac, opt, O, A, LP, adv, ret)
        mean_ep = float(np.mean(eps)) if eps else float(R.sum())
        log.append(dict(iter=it, mean_ep_return=round(mean_ep, 1),
                        n_episodes=len(eps), kl=round(kl, 4),
                        pi_loss=round(pl, 4), v_loss=round(vl, 2)))
        print(f"iter {it}: ep_return {mean_ep:8.1f} ({len(eps)} eps)  "
              f"KL {kl:.4f}  pi {pl:+.4f}  v {vl:.1f}")
        assert np.isfinite([kl, pl, vl]).all(), "non-finite losses"
        assert abs(kl) < 0.15, f"KL blew up: {kl}"

    first, last = log[0]["mean_ep_return"], log[-1]["mean_ep_return"]
    assert last > 0.6 * first, f"return collapsed: {first} -> {last}"

    ckpt = os.path.join(HERE, "ppo_smoke_ckpt.pt")
    torch.save(ac.state_dict(), ckpt)
    ac2 = ActorCritic()
    ac2.load_state_dict(torch.load(ckpt, weights_only=True))
    o = torch.zeros(41)
    assert torch.allclose(ac.pi(o), ac2.pi(o)), "checkpoint roundtrip failed"
    with open(os.path.join(HERE, "ppo_smoke_log.json"), "w") as f:
        json.dump(log, f, indent=1)
    print(f"PPO SMOKE PASS — plumbing verified "
          f"(returns {first:.0f} -> {last:.0f}); checkpoint + log saved.")
    print("Phase-3 real training: lift this loop to the laptop, add domain "
          "randomization (randomize=True, push_prob>0), 8-16 parallel envs, "
          "~5M steps.")


if __name__ == "__main__":
    main()
