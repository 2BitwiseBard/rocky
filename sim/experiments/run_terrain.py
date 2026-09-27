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

Usage: MUJOCO_GL=egl .venv/bin/python sim/experiments/run_terrain.py [--video AMP]
"""
import sys, json
import numpy as np
import mujoco

import exp_paths as X                   # sys.path (sim/, gait/, perception/, audio/) + where results / clips go
from pebble_gait import WaveGait                                    # noqa: E402
from scenes import build_model, init_robot, N_BOX, FIELD            # noqa: E402,F401  (re-exported)

AMPS_MM = [0, 5, 10, 15, 20]
SEEDS = [0, 1, 2]
V_X = 45.0            # mm/s
T_WALK = 8.0
T_STAND = 1.0


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
    out = X.out(f"pebble_terrain_{tag}.mp4")
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
    with open(X.result("terrain_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    pass_amp = 0
    for amp in AMPS_MM:
        if all(r["ok"] for r in results if r["amp"] == amp):
            pass_amp = amp
    print(f"\n=== blind wave gait rubble envelope: {pass_amp} mm obstacle height ===")
