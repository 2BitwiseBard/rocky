"""Figures for the 2026-07-29 robustness session: push envelope + terrain.

Palette: dataviz reference instance (light mode) — blue #2a78d6 / orange
#eb6834 categorical pair, recessive chrome inks.
"""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
SURF, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, BASE = "#e1e0d9", "#c3c2b7"
BLUE, ORANGE, BLUE_LT = "#2a78d6", "#eb6834", "#86b6ef"
plt.rcParams.update({"font.family": "sans-serif", "text.color": INK,
                     "axes.edgecolor": BASE, "xtick.color": MUTED,
                     "ytick.color": MUTED, "axes.labelcolor": INK2})

# ---------------------------------------------------------------- push polar
ext = json.load(open(os.path.join(HERE, "push_results_ext.json")))
dirs = sorted(int(d) for d in ext["stand"])
th = np.deg2rad(dirs + [dirs[0]])
stand = [ext["stand"][str(d)] if str(d) in ext["stand"] else ext["stand"][d] for d in dirs]
walk = [ext["walk"][str(d)] if str(d) in ext["walk"] else ext["walk"][d] for d in dirs]
stand += stand[:1]; walk += walk[:1]

fig = plt.figure(figsize=(6.4, 5.6), dpi=150, facecolor=SURF)
ax = fig.add_subplot(111, projection="polar", facecolor=SURF)
ax.plot(th, stand, color=BLUE, lw=2, marker="o", ms=5, label="quiet stance")
ax.plot(th, walk, color=ORANGE, lw=2, marker="o", ms=5, label="walking @ 45 mm/s")
bw = 2.7 * 9.81
ax.plot(np.linspace(0, 2*np.pi, 120), [bw]*120, color=MUTED, lw=1, ls=(0, (4, 4)))
ax.text(np.deg2rad(105), bw + 2.5, "1× bodyweight (26 N)", color=MUTED, fontsize=8,
        ha="center")
for a in (90, 162, 234, 306, 18):
    ax.plot([np.deg2rad(a)]*2, [0, 6], color=BASE, lw=1)
ax.text(np.deg2rad(90), 9, "leg 0", color=MUTED, fontsize=7, ha="center")
ax.set_ylim(0, 60)
ax.set_rgrids([20, 40, 60], angle=54, color=MUTED, fontsize=8)
ax.grid(color=GRID, lw=0.7)
ax.spines["polar"].set_color(BASE)
ax.set_title("Push tolerance by direction — 0.15 s shove survived, no reflexes (N)",
             fontsize=10.5, color=INK, pad=18)
leg = ax.legend(loc="lower left", bbox_to_anchor=(-0.12, -0.12), frameon=False,
                fontsize=9, labelcolor=INK2)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "fig_push_envelope.png"), facecolor=SURF)
plt.close(fig)

# ---------------------------------------------------------------- terrain
res = json.load(open(os.path.join(HERE, "terrain_results.json")))
res += json.load(open(os.path.join(HERE, "terrain_results_ext.json")))
amps = sorted({r["amp"] for r in res})
fig, ax = plt.subplots(figsize=(6.8, 4.4), dpi=150, facecolor=SURF)
ax.set_facecolor(SURF)
for r in res:
    pct = 100 * r["disp_x"] / r["commanded"]
    ax.plot(r["amp"], pct, "o", color=BLUE_LT, ms=5, zorder=3)
means = [np.mean([100 * r["disp_x"] / r["commanded"] for r in res if r["amp"] == a])
         for a in amps]
ax.plot(amps, means, color=BLUE, lw=2, zorder=4)
ax.axhline(100, color=MUTED, lw=1, ls=(0, (4, 4)))
ax.text(40.3, 101.5, "commanded", color=MUTED, fontsize=8, ha="right")
ax.axvline(32, color=BASE, lw=1)
ax.text(32.6, 8, "step height (32 mm)", color=MUTED, fontsize=8, rotation=90,
        va="bottom")
ax.annotate("mean of 3 seeds", xy=(20, means[amps.index(20)]),
            xytext=(13, 72), color=INK2, fontsize=8.5,
            arrowprops=dict(arrowstyle="-", color=BASE, lw=0.8))
ax.annotate("per-seed runs", xy=(35, 39), xytext=(25.5, 30), color=INK2, fontsize=8.5,
            arrowprops=dict(arrowstyle="-", color=BASE, lw=0.8))
ax.set_xlabel("rubble obstacle height (mm)")
ax.set_ylabel("distance covered (% of commanded)")
ax.set_title("Blind wave gait vs rubble — degrades gracefully, snags near step height",
             fontsize=10.5, color=INK, loc="left", pad=10)
ax.set_xlim(-1.5, 42)
ax.set_ylim(0, 112)
ax.grid(color=GRID, lw=0.7, axis="y")
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "fig_terrain.png"), facecolor=SURF)
print("figures saved")
