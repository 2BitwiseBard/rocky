"""Animated stick-figure demo of the wave gait: walk -> strafe -> turn in place.

Outputs:
  pebble_gait_demo.gif      the money shot: Pebble walking omnidirectionally
  joint_trajectories.png    q1,q2,q3 of leg 0 over one cycle
  demo_contact_sheet.png    9 stills for quick review
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from matplotlib.animation import FuncAnimation, PillowWriter
from pebble_gait import (WaveGait, leg_ik, body_to_leg, leg_to_body,
                         N_LEGS, STATION_DEG, R_BODY, L1, L2, Z_HIP)

g = WaveGait()
FPS = 12
SEGS = [((45.0, 0.0, 0.0), 4.0, "walk +X"),
        ((0.0, 45.0, 0.0), 4.0, "strafe +Y"),
        ((0.0, 0.0, 0.6), 4.0, "turn in place")]
DT = 1.0 / FPS

# integrate world pose
times, poses, cmds = [], [], []
X = np.zeros(3)                      # x, y, yaw
t = 0.0
for cmd, dur, _ in SEGS:
    for _ in range(int(dur * FPS)):
        times.append(t); poses.append(X.copy()); cmds.append(cmd)
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
    # body pentagon (deck) + a simple carapace peak point for orientation
    ring = [world(np.array([R_BODY*np.cos(np.deg2rad(a)),
                            R_BODY*np.sin(np.deg2rad(a)), 0]), pose)
            for a in STATION_DEG]
    penta = [[ring[i], ring[(i+1) % 5]] for i in range(5)]
    return segs, penta, np.array(foot_pts), pose

fig = plt.figure(figsize=(8, 6.4), dpi=92)
ax = fig.add_subplot(111, projection="3d")

def draw(k):
    ax.cla()
    segs, penta, foot_pts, pose = frame_geometry(k)
    # ground grid
    gx = np.arange(-300, 901, 100)
    for x in gx:
        ax.plot([x, x], [-500, 500], [0, 0], color="#ddd6ee", lw=0.5, zorder=0)
        ax.plot([-300, 900], [x-300, x-300], [0, 0], color="#ddd6ee", lw=0.5, zorder=0)
    ax.add_collection3d(Line3DCollection(segs, colors="#5b4a8a", linewidths=2.4))
    ax.add_collection3d(Line3DCollection(penta, colors="#8d6fc9", linewidths=3.0))
    if len(foot_pts):
        ax.scatter(foot_pts[:, 0], foot_pts[:, 1], foot_pts[:, 2],
                   c="#2e2440", s=26, depthshade=False)
    seg_name = SEGS[min(int(times[k] // 4), 2)][2]
    ax.set_title(f"Pebble wave gait — {seg_name}   t={times[k]:4.1f}s")
    cx, cy = pose[0], pose[1]
    ax.set_xlim(cx-320, cx+320); ax.set_ylim(cy-320, cy+320); ax.set_zlim(0, 320)
    ax.set_box_aspect((1, 1, 0.5))
    ax.view_init(elev=24, azim=-55)
    ax.set_axis_off()

anim = FuncAnimation(fig, draw, frames=len(times), interval=1000/FPS)
anim.save("pebble_gait_demo.gif", writer=PillowWriter(fps=FPS))
print("gif saved:", len(times), "frames")

# contact sheet for review
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
plt.tight_layout(); plt.savefig("demo_contact_sheet.png")
print("contact sheet saved")

# joint trajectories, leg 0, one cycle of forward walk
ts = np.linspace(0, g.T, 200)
Q = np.array([g.joint_targets(t, 45, 0, 0)[0][0] for t in ts])
fig3, ax3 = plt.subplots(figsize=(8, 4.2), dpi=100)
for j, (lbl, colr) in enumerate([("coxa q1", "#5b4a8a"), ("femur q2", "#8d6fc9"),
                                 ("knee q3", "#c4b3e8")]):
    ax3.plot(ts, np.rad2deg(Q[:, j]), label=lbl, color=colr, lw=2)
ax3.axhspan(-40, 40, color="#5b4a8a", alpha=0.06)
ax3.set_xlabel("time (s), one cycle @ 45 mm/s")
ax3.set_ylabel("joint angle (deg)")
ax3.set_title("Leg 0 joint trajectories — wave gait (shaded: CAD-validated coxa range)")
ax3.legend(); ax3.grid(alpha=0.3)
plt.tight_layout(); plt.savefig("joint_trajectories.png")
print("joint plot saved")
