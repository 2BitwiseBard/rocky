"""Sim verification of ADJACENT-arm manipulation via foot repositioning.

A/B test of gait/pebble_manip_adjacent.py:
  default   — full choreography (crouch -> step flanks wide -> lean -> arms)
  --control — same timeline but NO repositioning (the D013 failure case):
              arms raise straight from nominal stance. Expected to tip.

Metrics: body tilt, dynamic support margin (whole-robot CoM projection vs
the planted-feet polygon), coxa angles vs the +/-40 deg limit.
`--video` renders the full run.

Usage: MUJOCO_GL=osmesa python3 run_manip_adjacent.py [--control] [--video]
"""
import os, sys, json
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
import pebble_manip_adjacent as pma
from pebble_manip_adjacent import AdjacentManip, T_TOTAL
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS

CLAW_MAX = np.deg2rad(55)


def make(control=False):
    if control:                       # no repositioning, no lean, no crouch
        pma.SWING_DEG = 0.0
        pma.R_WIDE = 185.0
        pma.CROUCH_MM = 1e-6
        pma.LEAN_MM = 0.0
    man = AdjacentManip()
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    data = mujoco.MjData(model)
    g = man.g
    q0 = np.array([leg_ik(body_to_leg(i, g.p_nom[i])) for i in range(N_LEGS)]).flatten()
    data.qpos[0:3] = [0, 0, (g.h + 14) / 1000.0]
    data.qpos[3:7] = [1, 0, 0, 0]
    # NOTE: qpos joint order is interleaved per leg (yaw,hip,knee,claw)x5 —
    # NOT [15 leg joints][5 claws] like ctrl. Write by joint address.
    for i in range(N_LEGS):
        for j, jn in enumerate(("yaw", "hip", "knee")):
            data.qpos[model.joint(f"{jn}{i}").qposadr[0]] = q0[3 * i + j]
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    return man, model, data


def support_margin(model, data, planted, arm_legs):
    """Signed distance (mm) of robot CoM xy to the planted-feet polygon."""
    torso = model.body("torso").id
    com = data.subtree_com[torso][:2] * 1000
    feet = [data.geom_xpos[model.geom(f"foot{i}").id][:2] * 1000
            for i in range(N_LEGS) if planted[i]]
    if len(feet) < 3:
        return np.nan
    c = np.mean(feet, axis=0)
    ang = sorted(range(len(feet)), key=lambda k: np.arctan2(*(feet[k] - c)[::-1]))
    feet = [feet[k] for k in ang]
    m = 1e9
    for k in range(len(feet)):
        a, b = feet[k], feet[(k + 1) % len(feet)]
        n = np.array([-(b - a)[1], (b - a)[0]])
        n /= (np.linalg.norm(n) + 1e-12)
        if np.dot(n, c - a) < 0:
            n = -n
        m = min(m, np.dot(n, com - a))
    return m


def run(control=False, video=False):
    man, model, data = make(control)
    torso = model.body("torso").id
    DT = model.opt.timestep
    FPS = 30
    spf = int(round(1 / (FPS * DT)))
    frames = []
    if video:
        import imageio
        renderer = mujoco.Renderer(model, 480, 720)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation = 0.85, -18

    tilt_max = 0.0
    margin_min_work = 1e9
    q1_max = 0.0
    fell_at = None
    label_of_frame = []
    n_steps = int(T_TOTAL / DT)
    for k in range(n_steps):
        t = k * DT
        q, claw, name, planted = man.targets(t)
        data.ctrl[:15] = q.flatten()
        data.ctrl[15:20] = claw * CLAW_MAX
        mujoco.mj_step(model, data)
        zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
        tilt = np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1)))
        tilt_max = max(tilt_max, tilt)
        q1s = [abs(data.qpos[model.joint(f"yaw{i}").qposadr[0]])
               for i in range(N_LEGS) if planted[i] and i not in man.arm_legs]
        if q1s:
            q1_max = max(q1_max, np.rad2deg(max(q1s)))
        if name == "work":
            mg = support_margin(model, data, planted, man.arm_legs)
            if not np.isnan(mg):
                margin_min_work = min(margin_min_work, mg)
        if fell_at is None and (tilt > 25 or data.xpos[torso][2] < 0.055):
            fell_at = (t, name)
            if not video:
                break
        if video and k % spf == 0:
            cam.lookat[:] = data.xpos[torso]
            cam.azimuth = 135 - 4.0 * t
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
            label_of_frame.append(name)

    tag = "control" if control else "repositioned"
    print(f"[{tag}] tilt_max {tilt_max:.2f} deg | "
          f"min work-phase margin {margin_min_work if margin_min_work < 1e8 else float('nan'):.1f} mm | "
          f"max planted coxa {q1_max:.1f} deg | "
          f"{'FELL at t=%.1f s (%s)' % fell_at if fell_at else 'stayed up'}")
    if video and frames:
        import imageio
        out = os.path.join(HERE, f"pebble_manip_adjacent_{tag}.mp4")
        imageio.mimsave(out, frames, fps=FPS, codec="libx264", quality=8)
        print("video saved:", out)
    return dict(tag=tag, tilt_max=float(tilt_max),
                margin_min_work=float(margin_min_work if margin_min_work < 1e8 else np.nan),
                coxa_max=float(q1_max), fell_at=fell_at)


if __name__ == "__main__":
    res = run(control="--control" in sys.argv, video="--video" in sys.argv)
    with open(os.path.join(HERE, f"manip_adjacent_{res['tag']}.json"), "w") as f:
        json.dump(res, f, indent=1, default=str)
