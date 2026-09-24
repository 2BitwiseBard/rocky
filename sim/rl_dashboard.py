"""RL runs at a glance (D048): a table of every checkpoint in sim/runs/ and a
training-curve figure.

  python rl_dashboard.py            # table + out/rl_curves.png
  python rl_dashboard.py --table    # table only (what the playground's `rl` prints)

Columns: env, reward version, rate limit, obs version, servo model it
trained with, steps, last logged return and episode length, final entropy
(sigma inflating past ~10 nats is the D045 bang-bang signature), the D052
contract flags and the recorded eval, if any (RESULTS below — keep it in
step with docs/RL_GUIDE.md §4).

D052 flags (rl_common.checkpoint_contract): 'legacy obs' = trained on the
pre-D052 observation (true state, torso height) — replayable, not
deployable; 'exceeds servo' = its rate limit is above the ST3215's 4.7
rad/s no-load speed (every v1-v3 recover run: 5.0); 'unbudgeted cmd' = a
gait run whose commands were not fitted to WaveGait.budget() (every gait
run before D052 V2: cmd_sample3 drew mostly outside the envelope).
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
RUNS = os.path.join(HERE, "runs")

# recorded evals (deterministic policy, this machine) — see RL_GUIDE §4. D052 = on the
# 2026-09-24 model (damping 0.62, forcerange 1.9, mu 0.8); the rest predate it.
RESULTS = {
    "robust_fwd2": "D052: 245 mm vs bare gait 320 (servo on: 236 vs 298) — loses (D031)",
    "cmd_sample3": "walk under full DR: same conclusion (pre-D052)",
    "recover1": "D052: stood 2/20 hybrid (0/20 hw handoff); system (handoff_ok) 20/20 vs 11/20 "
                "no-righter, all exits on the stall ramp — SHIPPED",
    "recover2": "stood 2/20 — sigma inflated (D045)",
    "recover3_capped": "stood 7/20 — carried by noise (D045)",
    "recover3_scratch": "stood 3/20",
    "recover5_v3": "stood 2/20 — v3 negative (D048)",
    "recover5_v3_warm": "stood 4/20 — smoothest yet: 58 % pinned, 6.5 rev/s (D048)",
}


def summarize_runs():
    rows = []
    if not os.path.isdir(RUNS):
        return rows
    for name in sorted(os.listdir(RUNS)):
        d = os.path.join(RUNS, name)
        log = os.path.join(d, "train_log.jsonl")
        ck = os.path.join(d, "latest.pt")
        if not os.path.exists(log):
            continue
        last, n = None, 0
        with open(log) as fh:
            for line in fh:
                line = line.strip()
                if line:
                    last = json.loads(line)
                    n += 1
        meta, contract = {}, {}
        if os.path.exists(ck):
            try:
                import torch
                import rl_common as rc
                blob = torch.load(ck, map_location="cpu", weights_only=False)
                meta = dict(blob.get("args") or {})
                contract = rc.checkpoint_contract(blob, env_hint="recover" if name.startswith("recover")
                                                  else "gait")
            except Exception:
                meta, contract = {}, {}
        # pre-D048 checkpoints saved no env/reward/rate_limit: recover* runs were
        # the recover env at reward v1 and 5 rad/s, everything else the gait env
        env = meta.get("env") or ("recover" if name.startswith("recover") else "gait")
        env = "gait" if env == "walk" else env
        reward = meta.get("reward") or "v1"
        rows.append(dict(run=name, env=env, reward=reward,
                         rate=meta.get("rate_limit", 5.0 if env == "recover" else None),
                         obs_version=contract.get("obs_version"), servo=contract.get("servo"),
                         flags=list(contract.get("flags", [])),
                         fingerprint=contract.get("robot_fingerprint"),
                         steps=(last or {}).get("step", 0), ret=(last or {}).get("ep_return"),
                         ep_len=(last or {}).get("ep_len"), ent=(last or {}).get("ent"),
                         updates=n, ckpt=os.path.exists(ck), note=RESULTS.get(name, "")))
    return rows


def table(rows):
    if not rows:
        return "no runs in sim/runs/"
    out = [f"{'run':18s} {'env':8s} {'rew':4s} {'rate':>4s} {'obs':>3s} {'servo':7s} {'steps':>10s} "
           f"{'return':>7s} {'len':>5s} {'ent':>5s}  {'flags':26s} eval / note"]
    for r in rows:
        rate = "" if r["rate"] is None else f"{r['rate']:.1f}"
        ov = "" if r.get("obs_version") is None else f"v{r['obs_version']}"
        flags = ", ".join(r.get("flags") or [])
        out.append(f"{r['run']:18s} {r['env']:8s} {str(r['reward']):4s} {rate:>4s} {ov:>3s} "
                   f"{str(r.get('servo') or ''):7s} {r['steps']:>10,} "
                   f"{(r['ret'] if r['ret'] is not None else float('nan')):7.1f} "
                   f"{(r['ep_len'] if r['ep_len'] is not None else float('nan')):5.0f} "
                   f"{(r['ent'] if r['ent'] is not None else float('nan')):5.1f}  {flags:26s} {r['note']}")
    return "\n".join(out)


def curves(out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 2, figsize=(11, 7), dpi=110)
    keys = [("ep_return", "episode return"), ("ep_len", "episode length (steps)"),
            ("ent", "policy entropy (nats)"), ("sps", "steps / s")]
    for name in sorted(os.listdir(RUNS)):
        log = os.path.join(RUNS, name, "train_log.jsonl")
        if not os.path.exists(log):
            continue
        recs = [json.loads(l) for l in open(log) if l.strip()]
        if not recs:
            continue
        x = [r["step"] / 1e6 for r in recs]
        for ax, (k, _) in zip(axs.flat, keys):
            y = [r.get(k) for r in recs]
            if any(v is not None for v in y):
                ax.plot(x, y, lw=1.2, label=name)
    for ax, (k, title) in zip(axs.flat, keys):
        ax.set_title(title)
        ax.set_xlabel("M steps")
        ax.grid(alpha=0.3)
    axs[0, 0].legend(fontsize=8)
    fig.suptitle("PPO runs in sim/runs/ (train_log.jsonl)")
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path)
    return out_path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--table", action="store_true")
    a = ap.parse_args()
    print(table(summarize_runs()))
    if not a.table:
        print("wrote", curves(os.path.join(HERE, "out", "rl_curves.png")))
