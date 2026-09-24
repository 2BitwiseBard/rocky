"""Rough-terrain robustness sweep for the blind wave gait.

Question: how rough can the ground get before the OPEN-LOOP wave gait (no
IMU, no contact sensing — Phase 0 controller) stops working? The answer is
the requirement spec for Phase 3's terrain adaptation.

Terrain model: a rubble field of randomly placed/rotated boxes (side
20-40 mm, height amp/2..amp) on the flat floor. Discrete obstacles give
well-conditioned contacts — an earlier heightfield version produced
spurious deep contacts at small amplitudes (near-degenerate prisms), see
BUILD_LOG 2026-07-29.

Sweep: amp 0..20 mm x 3 seeds, walk +X @ 45 mm/s for 8 s, no rendering.
`--video AMP` renders one trial.

Usage: MUJOCO_GL=osmesa python3 run_terrain.py [--video AMP]
"""
import os, sys, json
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS
import rocky_model as rm                                             # noqa: E402

AMPS_MM = [0, 5, 10, 15, 20]
SEEDS = [0, 1, 2]
V_X = 45.0            # mm/s
T_WALK = 8.0
T_STAND = 1.0
N_BOX = 150
FIELD = (-0.25, 0.75, -0.35, 0.35)   # x0 x1 y0 y1 walk corridor, m


def build_model(amp_mm, seed):
    """pebble.xml + a rubble field of boxes, max height amp_mm."""
    with open(os.path.join(HERE, "pebble.xml")) as f:
        xml = f.read()
    boxes = []
    if amp_mm > 0:
        rng = np.random.default_rng(seed)
        for b in range(N_BOX):
            x = rng.uniform(FIELD[0], FIELD[1])
            y = rng.uniform(FIELD[2], FIELD[3])
            if np.hypot(x, y) < 0.26:          # keep the start stance clear
                continue
            sx, sy = rng.uniform(0.010, 0.020, 2)          # half-sides 10-20 mm
            h = rng.uniform(0.5, 1.0) * amp_mm / 1000.0    # box height
            yaw = rng.uniform(0, 180)
            shade = rng.uniform(0.25, 0.45)
            boxes.append(
                f'<geom type="box" size="{sx:.3f} {sy:.3f} {h/2:.4f}" '
                f'pos="{x:.3f} {y:.3f} {h/2:.4f}" euler="0 0 {yaw:.0f}" '
                f'friction="1.2 0.01 0.001" rgba="{shade:.2f} {shade*0.9:.2f} {shade*1.25:.2f} 1"/>')
    xml = xml.replace("<worldbody>", "<worldbody>\n    " + "\n    ".join(boxes))
    return mujoco.MjModel.from_xml_string(xml)


def init_robot(model, gait, extra_z=0.0):
    data = mujoco.MjData(model)
    q0 = np.array([leg_ik(body_to_leg(i, gait.p_nom[i])) for i in range(N_LEGS)]).flatten()
    data.qpos[0:3] = [0, 0, rm.spawn_z_m(gait.h, platform_z_m=extra_z)]   # D052: was h + 14 mm
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[[model.joint(f"{n}{i}").qposadr[0] for i in range(N_LEGS)
               for n in ("yaw", "hip", "knee")]] = q0   # claws interleave in qpos: never qpos[7:22]
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    return data, q0


def ctrl_at(gait, q0, t):
    if t < T_STAND:
        return q0
    ramp = min((t - T_STAND) / 0.6, 1.0)
    q, _, _ = gait.joint_targets(t - T_STAND, V_X * ramp, 0.0, 0.0)
    return q.flatten()


def run_trial(amp_mm, seed):
    model = build_model(amp_mm, seed)
    gait = WaveGait()
    data, q0 = init_robot(model, gait, extra_z=amp_mm / 1000.0)
    torso = model.body("torso").id
    DT = model.opt.timestep
    tilt_max, h_min = 0.0, 1e9
    pos0 = None
    for k in range(int((T_STAND + T_WALK) / DT)):
        t = k * DT
        data.ctrl[:15] = ctrl_at(gait, q0, t)
        if t >= T_STAND and pos0 is None:
            pos0 = data.xpos[torso].copy()
        mujoco.mj_step(model, data)
        zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
        tilt = np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1)))
        tilt_max = max(tilt_max, tilt)
        h_min = min(h_min, data.xpos[torso][2])
        if tilt > 45:
            break
    disp = (data.xpos[torso] - pos0) * 1000 if pos0 is not None else np.zeros(3)
    commanded = V_X * (T_WALK - 0.3)
    fell = tilt_max > 30 or h_min < 0.060
    ok = (not fell) and disp[0] >= 0.55 * commanded
    return dict(amp=amp_mm, seed=seed, disp_x=float(disp[0]), drift_y=float(disp[1]),
                commanded=float(commanded), tilt_max=float(tilt_max),
                h_min_mm=float(h_min * 1000), fell=bool(fell), ok=bool(ok))


def render_video(amp_mm, seed, tag):
    import imageio
    model = build_model(amp_mm, seed)
    gait = WaveGait()
    data, q0 = init_robot(model, gait, extra_z=amp_mm / 1000.0)
    torso = model.body("torso").id
    FPS = 30
    DT = model.opt.timestep
    spf = int(round(1 / (FPS * DT)))
    renderer = mujoco.Renderer(model, 480, 720)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation = 0.85, -18
    frames = []
    for f in range(int((T_STAND + T_WALK) * FPS)):
        for s in range(spf):
            t = (f * spf + s) * DT
            data.ctrl[:15] = ctrl_at(gait, q0, t)
            mujoco.mj_step(model, data)
        cam.lookat[:] = data.xpos[torso]
        cam.azimuth = 145 - 5 * (f / FPS)
        renderer.update_scene(data, cam)
        frames.append(renderer.render())
    out = os.path.join(HERE, f"pebble_terrain_{tag}.mp4")
    imageio.mimsave(out, frames, fps=FPS, codec="libx264", quality=8)
    print(f"video saved: {out}")


if __name__ == "__main__":
    if "--video" in sys.argv:
        amp = int(sys.argv[sys.argv.index("--video") + 1])
        render_video(amp, 0, f"{amp}mm")
        sys.exit(0)
    results = []
    for amp in AMPS_MM:
        for seed in SEEDS:
            r = run_trial(amp, seed)
            results.append(r)
            print(f"amp {amp:3d} mm seed {seed}: disp {r['disp_x']:6.0f}/{r['commanded']:.0f} mm "
                  f"drift {r['drift_y']:5.0f}  tilt_max {r['tilt_max']:5.1f}°  "
                  f"h_min {r['h_min_mm']:5.1f}  {'FELL' if r['fell'] else ('ok' if r['ok'] else 'SLOW/STUCK')}")
    with open(os.path.join(HERE, "terrain_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    pass_amp = 0
    for amp in AMPS_MM:
        if all(r["ok"] for r in results if r["amp"] == amp):
            pass_amp = amp
    print(f"\n=== blind wave gait rubble envelope: {pass_amp} mm obstacle height ===")
