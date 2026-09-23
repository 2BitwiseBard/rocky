"""Stuck-detection + retry-higher-step on the >30 mm rubble that snags D017.

Baseline (run_terrain.py): blind gait crosses <=30 mm reliably; 35-40 mm
SNAGS (progress stalls, no fall). This harness re-runs 30/35/40/45 mm rubble
with the ProgressWatchdog + RetryPolicy in the loop and measures crossing
success + time vs the no-watchdog baseline on identical fields.

Sim odometry = torso ground truth (hardware will use the legged-odom EKF —
same watchdog interface).

Usage: MUJOCO_GL=osmesa python3 run_stuck.py [--video AMP SEED]
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS       # noqa: E402
from pebble_watchdog import ProgressWatchdog, RetryPolicy           # noqa: E402
from run_terrain import build_model, init_robot                     # noqa: E402

AMPS_MM = [30, 35, 40, 45]
SEEDS = [0, 1, 2, 3]
V_X = 45.0
T_STAND = 1.0
T_WALK = 25.0            # retries slow the gait: give recoveries time to pay
GOAL_X = 0.40            # m: through the worst of the field


def run_trial(amp_mm, seed, watchdog_on, record=None):
    model = build_model(amp_mm, seed)
    gait = WaveGait()
    data, q0 = init_robot(model, gait, extra_z=amp_mm / 1000.0)
    torso = model.body("torso").id
    DT = model.opt.timestep
    dog = ProgressWatchdog(gait.T)
    policy = RetryPolicy(gait)
    tilt_max, h_min = 0.0, 1e9
    pos0 = None
    t_cross = None
    for k in range(int((T_STAND + T_WALK) / DT)):
        t = k * DT
        if t < T_STAND:
            data.ctrl[:15] = q0
        else:
            tw = t - T_STAND
            ramp = min(tw / 0.6, 1.0)
            vx, vy, wz = V_X * ramp, 0.0, 0.0
            if watchdog_on:
                vx, vy, wz = policy.command(tw, vx, vy, wz)
                odom = data.xpos[torso][:2] * 1000.0
                if dog.update(tw, odom, abs(vx)):
                    policy.on_stuck(tw)
                    dog.reset()
                policy.maybe_relax(tw, dog.last_ratio > 0.6)
            q, _, _ = gait.joint_targets(tw, vx, vy, wz)
            data.ctrl[:15] = q.flatten()
        if t >= T_STAND and pos0 is None:
            pos0 = data.xpos[torso].copy()
        mujoco.mj_step(model, data)
        if record is not None and k % record[0] == 0:
            record[1](data, t)
        z = data.xmat[torso].reshape(3, 3)[:, 2]
        tilt = np.rad2deg(np.arccos(np.clip(z[2], -1, 1)))
        tilt_max = max(tilt_max, tilt)
        h_min = min(h_min, data.xpos[torso][2])
        if t_cross is None and data.xpos[torso][0] > GOAL_X:
            t_cross = t - T_STAND
        if tilt > 45:
            break
    disp = (data.xpos[torso] - pos0) * 1000 if pos0 is not None else np.zeros(3)
    fell = tilt_max > 30 or h_min < 0.055
    crossed = t_cross is not None and not fell
    return dict(amp=amp_mm, seed=seed, watchdog=watchdog_on,
                disp_x=round(float(disp[0])), crossed=bool(crossed),
                t_cross=None if t_cross is None else round(t_cross, 1),
                tilt_max=round(float(tilt_max), 1), fell=bool(fell),
                stuck_events=policy.events)


def render_video(amp, seed):
    import imageio
    frames = []
    model = build_model(amp, seed)
    renderer = mujoco.Renderer(model, 480, 720)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation = 0.9, -16

    def grab(data, t):
        cam.lookat[:] = data.xpos[model.body("torso").id]
        cam.azimuth = 150
        renderer.update_scene(data, cam)
        frames.append(renderer.render())

    spf = int(round(1 / (30 * model.opt.timestep)))
    run_trial(amp, seed, watchdog_on=True, record=(spf, grab))
    out = os.path.join(HERE, f"pebble_stuck_retry_{amp}mm.mp4")
    imageio.mimsave(out, frames, fps=30, codec="libx264", quality=8)
    print("video saved:", out)


def main():
    if "--video" in sys.argv:
        i = sys.argv.index("--video")
        render_video(int(sys.argv[i + 1]), int(sys.argv[i + 2]))
        return
    results = []
    for amp in AMPS_MM:
        for seed in SEEDS:
            for dog in (False, True):
                r = run_trial(amp, seed, dog)
                results.append(r)
                ev = f" events={len(r['stuck_events'])}" if dog else ""
                print(f"amp {amp} seed {seed} dog={int(dog)}: "
                      f"disp {r['disp_x']:4d} mm  crossed={int(r['crossed'])} "
                      f"t={r['t_cross']}  tilt {r['tilt_max']:4.1f}{ev}")
    with open(os.path.join(HERE, "stuck_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    print("\n=== crossing rate (of {} fields) ===".format(len(SEEDS)))
    for amp in AMPS_MM:
        base = sum(r["crossed"] for r in results
                   if r["amp"] == amp and not r["watchdog"])
        dog = sum(r["crossed"] for r in results
                  if r["amp"] == amp and r["watchdog"])
        print(f"  {amp} mm: baseline {base}/{len(SEEDS)}  ->  "
              f"watchdog {dog}/{len(SEEDS)}")
    print("wrote stuck_results.json")


if __name__ == "__main__":
    main()
