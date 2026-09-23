"""Run Pebble in MuJoCo under the wave gait; record video + stability metrics.

Sequence: settle -> stand -> walk +X -> strafe +Y -> turn in place.
Usage: MUJOCO_GL=osmesa python3 run_sim.py
"""
import os, sys
import numpy as np
import mujoco
import imageio

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, N_LEGS

model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
data = mujoco.MjData(model)
gait = WaveGait()
H_MM = gait.h                       # 118 mm deck height

FPS = 30
DT = model.opt.timestep
STEPS_PER_FRAME = int(round(1 / (FPS * DT)))

SEGMENTS = [                         # (vx mm/s, vy, wz rad/s, duration s, label)
    (0, 0, 0, 1.0, "stand"),
    (45, 0, 0, 6.0, "walk +X"),
    (0, 45, 0, 4.0, "strafe +Y"),
    (0, 0, 0.6, 4.0, "turn in place"),
    (0, 0, 0, 1.0, "stand"),
]

def joint_ctrl(t_gait, cmd):
    vx, vy, wz = cmd
    if abs(vx) + abs(vy) + abs(wz) < 1e-9:
        # idle: hold nominal stance
        q = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            from pebble_gait import leg_ik, body_to_leg
            q[i] = leg_ik(body_to_leg(i, gait.p_nom[i]))
        return q.flatten()
    q, _, _ = gait.joint_targets(t_gait, vx, vy, wz)
    return q.flatten()

# ---- initialize pose: nominal stance, feet just touching ----
q0 = joint_ctrl(0.0, (0, 0, 0))
data.qpos[0:3] = [0, 0, (H_MM + 14) / 1000.0]   # torso z: feet slightly above floor
data.qpos[3:7] = [1, 0, 0, 0]
data.qpos[7:7+15] = q0
data.ctrl[:15] = q0
mujoco.mj_forward(model, data)

# settle for 0.8 s holding stance
for _ in range(int(0.8 / DT)):
    data.ctrl[:15] = q0
    mujoco.mj_step(model, data)

renderer = mujoco.Renderer(model, 480, 720)
cam = mujoco.MjvCamera()
cam.distance, cam.elevation = 0.9, -22

frames = []
metrics = {"height": [], "tilt": [], "t": [], "seg": []}
seg_marks = []
torso = model.body("torso").id

t_gait = 0.0
t_video = 0.0
pos_start_walk = None
for vx, vy, wz, dur, label in SEGMENTS:
    seg_marks.append((t_video, label))
    if label == "walk +X" and pos_start_walk is None:
        pos_start_walk = data.xpos[torso].copy()
    n_frames = int(dur * FPS)
    # gentle 0.4 s command ramp to avoid jerk
    for f in range(n_frames):
        ramp = min(1.0, (f / FPS) / 0.4) if label != "stand" else 1.0
        cmd = (vx * ramp, vy * ramp, wz * ramp)
        for _ in range(STEPS_PER_FRAME):
            data.ctrl[:15] = joint_ctrl(t_gait, cmd)
            mujoco.mj_step(model, data)
            if abs(cmd[0]) + abs(cmd[1]) + abs(cmd[2]) > 1e-9:
                t_gait += DT
        # camera follows torso
        cam.lookat[:] = data.xpos[torso] + np.array([0, 0, -0.02])
        cam.azimuth = -55 + 6 * np.sin(t_video * 0.25)
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render().copy())
        # metrics
        zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
        metrics["height"].append(data.xpos[torso][2] * 1000)
        metrics["tilt"].append(np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1))))
        metrics["t"].append(t_video)
        metrics["seg"].append(label)
        t_video += 1 / FPS
    if label == "walk +X":
        pos_end_walk = data.xpos[torso].copy()

imageio.mimsave(os.path.join(HERE, "pebble_sim.mp4"), frames, fps=FPS,
                codec="libx264", quality=8)
print(f"video saved: {len(frames)} frames")

# ---- metrics report ----
h = np.array(metrics["height"]); tilt = np.array(metrics["tilt"])
seg = np.array(metrics["seg"])
walk_mask = seg == "walk +X"
walk_disp = (pos_end_walk - pos_start_walk) * 1000
commanded = 45 * (6.0 - 0.2)     # ramp-adjusted approx, mm
print("=== SIM METRICS ===")
print(f"body height: mean {h.mean():.1f} mm (target ~{H_MM+14:.0f} at foot r), "
      f"std {h.std():.2f} mm, min {h.min():.1f}")
print(f"tilt: mean {tilt.mean():.2f} deg, max {tilt.max():.2f} deg")
print(f"walk +X displacement: {walk_disp[0]:.0f} mm (commanded ~{commanded:.0f} mm), "
      f"lateral drift {walk_disp[1]:.0f} mm")
print(f"fell over: {'YES' if h.min() < 60 or tilt.max() > 30 else 'no'}")

# contact sheet for inspection
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
ks = np.linspace(0, len(frames) - 1, 9).astype(int)
fig, axes = plt.subplots(3, 3, figsize=(12, 8), dpi=80)
for ax, k in zip(axes.flat, ks):
    ax.imshow(frames[k]); ax.axis("off")
    ax.set_title(f"t={k/FPS:.1f}s {seg[k]}", fontsize=8)
plt.tight_layout()
plt.savefig(os.path.join(HERE, "sim_contact_sheet.png"))
print("contact sheet saved")
