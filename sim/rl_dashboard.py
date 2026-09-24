"""RL runs at a glance (D048): a table of every checkpoint in sim/runs/ and a
training-curve figure.

  python rl_dashboard.py            # table + out/rl_curves.png
  python rl_dashboard.py --table    # table only (what the playground's `rl` prints)

Columns: env, reward version, rate limit, steps, last logged return and
episode length, final entropy (sigma inflating past ~10 nats is the D045
bang-bang signature) and the recorded eval, if any (RESULTS below — keep
it in step with docs/RL_GUIDE.md §4).
"""
import argparse
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")

# recorded evals (deterministic policy, this machine, current model) — see RL_GUIDE §4
RESULTS = {
    "robust_fwd2": "walk: 298 mm vs bare gait 365 (loses; D031)",
    "cmd_sample3": "walk under full DR: same conclusion",
    "recover1": "stood 7/20 — SHIPPED righter; staircase 73 % pinned, 10 rev/s",
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
        meta = {}
        if os.path.exists(ck):
            try:
                import torch
                meta = dict(torch.load(ck, map_location="cpu", weights_only=False).get("args") or {})
            except Exception:
                meta = {}
        # pre-D048 checkpoints saved no env/reward/rate_limit: recover* runs were
        # the recover env at reward v1 and 5 rad/s, everything else the gait env
        env = meta.get("env") or ("recover" if name.startswith("recover") else "gait")
        reward = meta.get("reward") or "v1"
        rows.append(dict(run=name, env=env, reward=reward,
                         rate=meta.get("rate_limit", 5.0 if env == "recover" else None),
                         steps=(last or {}).get("step", 0), ret=(last or {}).get("ep_return"),
                         ep_len=(last or {}).get("ep_len"), ent=(last or {}).get("ent"),
                         updates=n, ckpt=os.path.exists(ck), note=RESULTS.get(name, "")))
    return rows


def table(rows):
    if not rows:
        return "no runs in sim/runs/"
    out = [f"{'run':18s} {'env':8s} {'rew':4s} {'rate':>4s} {'steps':>10s} {'return':>7s} {'len':>5s} {'ent':>5s}  eval / note"]
    for r in rows:
        rate = "" if r["rate"] is None else f"{r['rate']:.0f}"
        out.append(f"{r['run']:18s} {r['env']:8s} {str(r['reward']):4s} {rate:>4s} {r['steps']:>10,} "
                   f"{(r['ret'] if r['ret'] is not None else float('nan')):7.1f} "
                   f"{(r['ep_len'] if r['ep_len'] is not None else float('nan')):5.0f} "
                   f"{(r['ent'] if r['ent'] is not None else float('nan')):5.1f}  {r['note']}")
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
