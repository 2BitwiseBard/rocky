"""Full Rocky-behavior demo: walk -> strafe -> turn -> 4-LEG WALK WITH AN ARM
RAISED -> 3-LEG STANCE, TWO ARMS + GRIPPERS working, body swaying.

Usage: MUJOCO_GL=osmesa python3 run_sim_manip.py
"""
import os, sys
import numpy as np
import mujoco
import imageio

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import (WaveGait, ArmedGait, stance_manip_targets, arm_pose,
                         claw_cycle, leg_ik, body_to_leg, N_LEGS)
import rocky_model as rm                                             # noqa: E402

model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
data = mujoco.MjData(model)
g5 = WaveGait()
g4 = ArmedGait(arm_legs=(0,))
FPS = 30
DT = model.opt.timestep
SPF = int(round(1 / (FPS * DT)))
CLAW_MAX = np.deg2rad(55)

def nominal_q():
    q = np.zeros((N_LEGS, 3))
    for i in range(N_LEGS):
        q[i] = leg_ik(body_to_leg(i, g5.p_nom[i]))
    return q

Q_NOM = nominal_q()

# segment controllers: fn(t_seg) -> (q[5,3], claw[5])
def seg_stand(t):
    return Q_NOM.copy(), np.zeros(N_LEGS)

def seg_walk(cmd, gait):
    def f(t):
        ramp = min(1.0, t / 0.4)
        q, _, _ = gait.joint_targets(t, cmd[0]*ramp, cmd[1]*ramp, cmd[2]*ramp)
        claw = np.zeros(N_LEGS)
        if isinstance(gait, ArmedGait):
            # blend lean + arm-raise together over the first 0.9 s (all legs)
            a = min(1.0, t / 0.9)
            q = (1-a)*Q_NOM + a*q
            for i in gait.arm_legs:
                claw[i] = a * 0.35 * CLAW_MAX * claw_cycle(t)
        return q, claw
    return f

def seg_manip_phase(lean_fn, arm_fn, dur):
    def f(t):
        lb, ab = lean_fn(t), arm_fn(t)
        q, claw01 = stance_manip_targets(g5, t, arm_legs=(0, 2),
                                         lean_blend=lb, arm_blend=ab)
        return q, claw01 * CLAW_MAX
    return f, dur

SEGMENTS = [
    (seg_stand, 1.0, "stand"),
    (seg_walk((45, 0, 0), g5), 5.0, "walk +X (5 legs)"),
    (seg_walk((0, 45, 0), g5), 3.0, "strafe +Y"),
    (seg_walk((0, 0, 0.6), g5), 3.0, "turn in place"),
    (seg_stand, 0.8, "stand"),
    (seg_walk((30, 0, 0), g4), 6.0, "4-LEG WALK, arm raised"),
    (seg_stand, 0.8, "stand"),
    (*seg_manip_phase(lambda t: min(1, t/1.0), lambda t: 0.0, 1.2), "lean onto stance side"),
    (*seg_manip_phase(lambda t: 1.0, lambda t: min(1, t/0.9), 6.5), "3-LEG STANCE: two arms + grippers"),
    (*seg_manip_phase(lambda t: 1.0, lambda t: max(0, 1 - t/0.9), 1.2), "arms down"),
    (*seg_manip_phase(lambda t: max(0, 1 - t/1.0), lambda t: 0.0, 1.2), "recenter"),
    (seg_stand, 1.0, "stand"),
]

# init
data.qpos[0:3] = [0, 0, rm.spawn_z_m(g5.h)]            # D052: was h + 14 mm
data.qpos[3:7] = [1, 0, 0, 0]
data.qpos[[model.joint(f"{n}{i}").qposadr[0] for i in range(N_LEGS)
           for n in ("yaw", "hip", "knee")]] = Q_NOM.flatten()   # claws interleave in qpos: never qpos[7:22]
data.ctrl[:15] = Q_NOM.flatten()
mujoco.mj_forward(model, data)
for _ in range(int(0.8 / DT)):
    data.ctrl[:15] = Q_NOM.flatten()
    mujoco.mj_step(model, data)

renderer = mujoco.Renderer(model, 480, 720)
cam = mujoco.MjvCamera()
cam.distance, cam.elevation = 0.85, -20
torso = model.body("torso").id

frames, height, tilt, seg_names = [], [], [], []
t_video = 0.0
for fn, dur, label in SEGMENTS:
    t_seg = 0.0
    for _ in range(int(dur * FPS)):
        for _ in range(SPF):
            q, claw = fn(t_seg)
            data.ctrl[:15] = q.flatten()
            data.ctrl[15:20] = claw
            mujoco.mj_step(model, data)
            t_seg += DT
        cam.lookat[:] = data.xpos[torso] + np.array([0, 0, -0.01])
        cam.azimuth = -55 + 10*np.sin(t_video*0.18)
        renderer.update_scene(data, camera=cam)
        frames.append(renderer.render().copy())
        z = data.xmat[torso].reshape(3, 3)[:, 2]
        height.append(data.xpos[torso][2]*1000)
        tilt.append(np.rad2deg(np.arccos(np.clip(z[2], -1, 1))))
        seg_names.append(label)
        t_video += 1/FPS

imageio.mimsave(os.path.join(HERE, "pebble_rocky_demo.mp4"), frames, fps=FPS,
                codec="libx264", quality=8)
print(f"video saved: {len(frames)} frames, {t_video:.1f} s")

height = np.array(height); tilt = np.array(tilt); seg_names = np.array(seg_names)
print("=== METRICS BY SEGMENT ===")
for label in dict.fromkeys(seg_names):
    m = seg_names == label
    print(f"{label:34s} height {height[m].mean():6.1f}±{height[m].std():4.1f} mm  "
          f"tilt max {tilt[m].max():5.2f} deg")
print(f"fell over: {'YES' if height.min() < 60 or tilt.max() > 30 else 'no'}")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
ks = np.linspace(0, len(frames)-1, 12).astype(int)
fig, axes = plt.subplots(3, 4, figsize=(14, 7.5), dpi=80)
for ax, k in zip(axes.flat, ks):
    ax.imshow(frames[k]); ax.axis("off")
    ax.set_title(f"t={k/FPS:.1f}s {seg_names[k]}", fontsize=7)
plt.tight_layout()
plt.savefig(os.path.join(HERE, "manip_contact_sheet.png"))
print("contact sheet saved")
