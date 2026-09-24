"""Run Pebble in MuJoCo under the wave gait; record video + stability metrics.

Sequence: settle -> stand -> walk +X -> strafe +Y -> turn in place.
Usage: MUJOCO_GL=osmesa python3 run_sim.py [--out DIR]   (video + contact sheet, default sim/)

D052 V2 (review): every segment's command goes through WaveGait.budget()
(the turn asked 0.6 rad/s, 2.4x the 0.246 rad/s envelope, and nothing
checked what it did), the turn's yaw is checked against a band like the
walk, and the outputs can go elsewhere (--out) so a local run does not
rewrite the tracked pebble_sim.mp4 / sim_contact_sheet.png.
"""
import argparse
import os, sys
import numpy as np
import mujoco
import imageio

HERE = os.path.dirname(os.path.abspath(__file__))
_ap = argparse.ArgumentParser()
_ap.add_argument("--out", default=HERE, help="directory for pebble_sim.mp4 + sim_contact_sheet.png")
OUT = _ap.parse_args().out
os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, N_LEGS
import rocky_model as rm

model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
data = mujoco.MjData(model)
gait = WaveGait()
H_MM = gait.h                       # 118 mm deck height

FPS = 30
DT = model.opt.timestep
STEPS_PER_FRAME = int(round(1 / (FPS * DT)))

SEGMENTS = [                         # (vx mm/s, vy, wz rad/s, duration s, label) — ASKED
    (0, 0, 0, 1.0, "stand"),
    (45, 0, 0, 6.0, "walk +X"),
    (0, 45, 0, 4.0, "strafe +Y"),
    (0, 0, 0.6, 4.0, "turn in place"),
    (0, 0, 0, 1.0, "stand"),
]
# what the gait gets (D052 V2): fitted into the envelope like every UI command
SEGMENTS = [tuple(gait.budget(vx, vy, wz)) + (dur, label) for vx, vy, wz, dur, label in SEGMENTS]

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
data.qpos[0:3] = [0, 0, rm.spawn_z_m(H_MM)]     # torso z: feet ~1 mm above floor (D052: was h + 14)
data.qpos[3:7] = [1, 0, 0, 0]
data.qpos[7:7+15] = q0
data.ctrl[:15] = q0
mujoco.mj_forward(model, data)

# settle for 0.8 s holding stance
for _ in range(int(0.8 / DT)):
    data.ctrl[:15] = q0
    mujoco.mj_step(model, data)

try:                                   # headless boxes without EGL/OSMesa (CI) still run the physics
    renderer = mujoco.Renderer(model, 480, 720)
except Exception as e:                 # noqa: BLE001
    renderer = None
    print(f"no offscreen renderer ({type(e).__name__}): physics + metrics only, no video/contact sheet")
cam = mujoco.MjvCamera()
cam.distance, cam.elevation = 0.9, -22

frames = []
metrics = {"height": [], "tilt": [], "t": [], "seg": []}
seg_marks = []
torso = model.body("torso").id

t_gait = 0.0
t_video = 0.0
pos_start_walk = None
turn_yaw, yaw_prev = 0.0, None


def _yaw():
    R = data.xmat[torso].reshape(3, 3)
    return float(np.arctan2(R[1, 0], R[0, 0]))


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
            if label == "turn in place":
                y = _yaw()
                if yaw_prev is not None:
                    turn_yaw += (y - yaw_prev + np.pi) % (2 * np.pi) - np.pi
                yaw_prev = y
        # camera follows torso
        cam.lookat[:] = data.xpos[torso] + np.array([0, 0, -0.02])
        cam.azimuth = -55 + 6 * np.sin(t_video * 0.25)
        if renderer is not None:
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

if frames:
    imageio.mimsave(os.path.join(OUT, "pebble_sim.mp4"), frames, fps=FPS,
                    codec="libx264", quality=8)
    print(f"video saved: {len(frames)} frames")

# ---- metrics report ----
h = np.array(metrics["height"]); tilt = np.array(metrics["tilt"])
seg = np.array(metrics["seg"])
walk_mask = seg == "walk +X"
walk_disp = (pos_end_walk - pos_start_walk) * 1000
commanded = 45 * (6.0 - 0.2)     # ramp-adjusted approx, mm
print("=== SIM METRICS ===")
print(f"body height: mean {h.mean():.1f} mm (target ~{rm.stance_torso_z_m(H_MM) * 1000:.0f}, rigid ideal servos), "
      f"std {h.std():.2f} mm, min {h.min():.1f}")
print(f"tilt: mean {tilt.mean():.2f} deg, max {tilt.max():.2f} deg")
print(f"walk +X displacement: {walk_disp[0]:.0f} mm (commanded ~{commanded:.0f} mm), "
      f"lateral drift {walk_disp[1]:.0f} mm")
turn_wz = next(wz for _vx, _vy, wz, _d, lab in SEGMENTS if lab == "turn in place")
turn_want = float(np.degrees(turn_wz * (4.0 - 0.2)))
print(f"turn in place: {np.degrees(turn_yaw):.1f} deg at the budgeted {turn_wz:.3f} rad/s "
      f"(commanded ~{turn_want:.0f} deg; asked 0.6 rad/s)")
fell = bool(h.min() < 60 or tilt.max() > 30)
print(f"fell over: {'YES' if fell else 'no'}")

# D052: the CI smoke step used to pass on a fall (it only printed). Exit non-zero on a
# fall or on a walk that went nowhere / too far. The band is +-30% around the walk
# measured on the D052 model (servo damping 0.6255 N.m.s/rad, forcerange 1.911 N.m on
# the 15 leg joints): 235 mm of ~261 commanded, measured 2026-09-24 (the owner's
# scratch run on the same derating said 240). Re-measure and move it when the model
# changes on purpose; a drift outside the band is exactly what this should catch.
# V2: 246 mm re-measured 2026-09-24 after the spawn moved to rocky_model.spawn_z_m (the
# 235 was measured before that change; A's run said 231, V1's 246 — same model otherwise).
WALK_REF_MM = 246.0
WALK_BAND = (0.7 * WALK_REF_MM, 1.3 * WALK_REF_MM)
walk_ok = WALK_BAND[0] <= walk_disp[0] <= WALK_BAND[1]
if not walk_ok:
    print(f"walk +X displacement {walk_disp[0]:.0f} mm outside the smoke band "
          f"{WALK_BAND[0]:.0f}..{WALK_BAND[1]:.0f} mm (ref {WALK_REF_MM:.0f} mm, 2026-09-24)")
# V2: the turn segment, budgeted to the envelope, measured 2026-09-24 (band +-30 %)
TURN_REF_DEG = 55.8               # of ~54 commanded at 0.246 rad/s (it asked 0.6: 2.4x the envelope)
TURN_BAND = (0.7 * TURN_REF_DEG, 1.3 * TURN_REF_DEG)
turn_ok = TURN_BAND[0] <= np.degrees(turn_yaw) <= TURN_BAND[1]
if not turn_ok:
    print(f"turn in place {np.degrees(turn_yaw):.1f} deg outside the smoke band "
          f"{TURN_BAND[0]:.0f}..{TURN_BAND[1]:.0f} deg (ref {TURN_REF_DEG:.0f} deg, 2026-09-24)")

if frames:
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
    plt.savefig(os.path.join(OUT, "sim_contact_sheet.png"))
    print("contact sheet saved")

if fell or not walk_ok or not turn_ok:
    sys.exit(1)
