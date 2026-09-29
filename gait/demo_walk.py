"""Animated stick-figure demo of the wave gait: walk -> strafe -> turn in place.

Every segment goes through WaveGait.budget() (D052), so the figure never asks
a joint for more than the servo budget or a coxa past its limit; the summary
line prints the facts for a caption (cycle, step height, speeds, joint peaks).

Outputs (next to this script, whatever the working directory):
  pebble_gait_demo.gif      Pebble walking omnidirectionally (the README's GIF)
  joint_trajectories.png    q1,q2,q3 of leg 0 over one cycle of the forward walk
  demo_contact_sheet.png    9 stills for quick review (--contact-sheet only)

    python gait/demo_walk.py [--contact-sheet] [--out DIR]
"""
import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                      # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection              # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter         # noqa: E402
import rocky_model as rm                                             # noqa: E402
from pebble_gait import (WaveGait, ik_ok, leg_to_body,               # noqa: E402
                         N_LEGS, STATION_DEG, R_BODY, L1, L2, Z_HIP)

HERE = os.path.dirname(os.path.abspath(__file__))
g = WaveGait()
FPS = 12
SEG_S = 4.0
# asked-for commands; each is fitted into the gait's envelope below
ASK = [((45.0, 0.0, 0.0), "walk +X"),
       ((0.0, 45.0, 0.0), "strafe +Y"),
       ((0.0, 0.0, 0.6), "turn in place")]
SEGS = []
for cmd, name in ASK:
    fit = g.budget(*cmd)
    if name.startswith("turn"):
        name = f"{name} ({fit[2]:.3f} rad/s envelope)"
    SEGS.append((fit, SEG_S, name))
DT = 1.0 / FPS

# integrate world pose
times, poses, cmds, seg_idx = [], [], [], []
X = np.zeros(3)                      # x, y, yaw
t = 0.0
for n, (cmd, dur, _) in enumerate(SEGS):
    for _ in range(int(dur * FPS)):
        times.append(t); poses.append(X.copy()); cmds.append(cmd); seg_idx.append(n)
        c, s = np.cos(X[2]), np.sin(X[2])
        X[0] += (c*cmd[0] - s*cmd[1]) * DT
        X[1] += (s*cmd[0] + c*cmd[1]) * DT
        X[2] += cmd[2] * DT
        t += DT
times = np.array(times); poses = np.array(poses)


def world(p_body, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return np.array([pose[0] + c*p_body[0] - s*p_body[1],
                     pose[1] + s*p_body[0] + c*p_body[1],
                     g.h + p_body[2]])


def frame_geometry(k):
    t, pose, cmd = times[k], poses[k], cmds[k]
    q, stance, feet_b = g.joint_targets(t, *cmd)
    segs, foot_pts = [], []
    for i in range(N_LEGS):
        a = np.deg2rad(STATION_DEG[i])
        st_b = np.array([R_BODY*np.cos(a), R_BODY*np.sin(a), 0.0])
        q1, q2, q3 = q[i]
        hip_l = np.array([L1*np.cos(q1), L1*np.sin(q1), Z_HIP])
        rk = L1 + L2*np.cos(q2)
        knee_l = np.array([rk*np.cos(q1), rk*np.sin(q1), Z_HIP + L2*np.sin(q2)])
        pts_b = [st_b, leg_to_body(i, hip_l), leg_to_body(i, knee_l), feet_b[i]]
        pts_w = [world(p, pose) for p in pts_b]
        segs += [[pts_w[j], pts_w[j+1]] for j in range(3)]
        if stance[i]:
            foot_pts.append(pts_w[3])
    # body polygon (deck) through the leg stations
    ring = [world(np.array([R_BODY*np.cos(np.deg2rad(a)),
                            R_BODY*np.sin(np.deg2rad(a)), 0]), pose)
            for a in STATION_DEG]
    penta = [[ring[i], ring[(i+1) % N_LEGS]] for i in range(N_LEGS)]
    return segs, penta, np.array(foot_pts), pose


def summary():
    """Caption facts + a check that every frame is inside the joint limits."""
    Q = np.array([g.joint_targets(times[k], *cmds[k])[0] for k in range(len(times))])
    peak = np.rad2deg(np.abs(Q).reshape(-1, 3).max(axis=0))
    ok = all(ik_ok(q) for q in Q)
    lim = rm.joint_limits_deg()
    print(f"gait: cycle {g.T:.1f} s, duty {g.duty}, step height {g.hstep:.0f} mm, "
          f"body {g.h:.0f} mm, stance radius {g.R0:.0f} mm")
    for (cmd, _, name) in SEGS:
        print(f"  {name}: vx {cmd[0]:.1f} mm/s, vy {cmd[1]:.1f} mm/s, wz {cmd[2]:.3f} rad/s")
    lo, hi = lim["yaw"]
    print(f"peak |q| yaw {peak[0]:.1f} (limit {lo:+.0f}..{hi:+.0f}), hip {peak[1]:.1f}, "
          f"knee {peak[2]:.1f} deg; every frame inside the limits: {ok}")
    return ok


def render_gif(path):
    fig = plt.figure(figsize=(8, 6.4), dpi=92)
    ax = fig.add_subplot(111, projection="3d")

    def draw(k):
        ax.cla()
        segs, penta, foot_pts, pose = frame_geometry(k)
        gx = np.arange(-300, 901, 100)                       # ground grid
        for x in gx:
            ax.plot([x, x], [-500, 500], [0, 0], color="#ddd6ee", lw=0.5, zorder=0)
            ax.plot([-300, 900], [x-300, x-300], [0, 0], color="#ddd6ee", lw=0.5, zorder=0)
        ax.add_collection3d(Line3DCollection(segs, colors="#5b4a8a", linewidths=2.4))
        ax.add_collection3d(Line3DCollection(penta, colors="#8d6fc9", linewidths=3.0))
        if len(foot_pts):
            ax.scatter(foot_pts[:, 0], foot_pts[:, 1], foot_pts[:, 2],
                       c="#2e2440", s=26, depthshade=False)
        ax.set_title(f"Pebble wave gait — {SEGS[seg_idx[k]][2]}   t={times[k]:4.1f}s")
        cx, cy = pose[0], pose[1]
        ax.set_xlim(cx-320, cx+320); ax.set_ylim(cy-320, cy+320); ax.set_zlim(0, 320)
        ax.set_box_aspect((1, 1, 0.5))
        ax.view_init(elev=24, azim=-55)
        ax.set_axis_off()

    anim = FuncAnimation(fig, draw, frames=len(times), interval=1000/FPS)
    anim.save(path, writer=PillowWriter(fps=FPS))
    plt.close(fig)
    print("gif saved:", path, len(times), "frames")


def render_contact_sheet(path):
    ks = np.linspace(0, len(times)-1, 9).astype(int)
    fig2 = plt.figure(figsize=(12, 9), dpi=80)
    for j, k in enumerate(ks):
        axs = fig2.add_subplot(3, 3, j+1, projection="3d")
        segs, penta, foot_pts, pose = frame_geometry(k)
        axs.add_collection3d(Line3DCollection(segs, colors="#5b4a8a", linewidths=1.6))
        axs.add_collection3d(Line3DCollection(penta, colors="#8d6fc9", linewidths=2.0))
        if len(foot_pts):
            axs.scatter(foot_pts[:, 0], foot_pts[:, 1], foot_pts[:, 2], c="#2e2440", s=12)
        axs.set_xlim(pose[0]-300, pose[0]+300); axs.set_ylim(pose[1]-300, pose[1]+300)
        axs.set_zlim(0, 300); axs.set_box_aspect((1, 1, 0.5))
        axs.view_init(elev=24, azim=-55); axs.set_axis_off()
        axs.set_title(f"t={times[k]:.1f}s", fontsize=8)
    plt.tight_layout(); plt.savefig(path); plt.close(fig2)
    print("contact sheet saved:", path)


def render_joint_plot(path):
    """Leg 0 over one cycle of the forward walk (the budgeted command)."""
    vx, vy, wz = SEGS[0][0]
    ts = np.linspace(0, g.T, 200)
    Q = np.array([g.joint_targets(t, vx, vy, wz)[0][0] for t in ts])
    lo, hi = rm.joint_limits_deg()["yaw"]
    fig3, ax3 = plt.subplots(figsize=(8, 4.2), dpi=100)
    for j, (lbl, colr) in enumerate([("coxa q1", "#5b4a8a"), ("femur q2", "#8d6fc9"),
                                     ("knee q3", "#c4b3e8")]):
        ax3.plot(ts, np.rad2deg(Q[:, j]), label=lbl, color=colr, lw=2)
    ax3.axhspan(lo, hi, color="#5b4a8a", alpha=0.06)
    ax3.set_xlabel(f"time (s), one {g.T:.1f} s cycle @ {vx:.0f} mm/s, step {g.hstep:.0f} mm")
    ax3.set_ylabel("joint angle (deg)")
    ax3.set_title("Leg 0 joint trajectories — wave gait (shaded: coxa soft-limit range)")
    ax3.legend(); ax3.grid(alpha=0.3)
    plt.tight_layout(); plt.savefig(path); plt.close(fig3)
    print("joint plot saved:", path)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=HERE, help="output directory (default: next to this script)")
    ap.add_argument("--contact-sheet", action="store_true", help="also write demo_contact_sheet.png")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    if not summary():
        raise SystemExit("a frame is outside the joint limits — fix the gait, not the demo")
    render_gif(os.path.join(args.out, "pebble_gait_demo.gif"))
    render_joint_plot(os.path.join(args.out, "joint_trajectories.png"))
    if args.contact_sheet:
        render_contact_sheet(os.path.join(args.out, "demo_contact_sheet.png"))


if __name__ == "__main__":
    main()
