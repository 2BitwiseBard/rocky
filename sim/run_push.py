"""Push-recovery envelope for the blind wave gait (and quiet stance).

Question: how hard can Pebble be shoved before it falls, with NO reflexes —
just stiff position servos + the gait marching on? This is the baseline that
Phase 3's IMU stabilization must beat, and it tells us how alive the robot
needs to be around pets/feet/door thresholds.

Method: horizontal force F applied to the torso for 0.15 s (impulse J=F*0.15)
at t=3.5 s, swept over 12 directions x escalating magnitudes; last-survived
impulse per direction is the envelope. Recovery = 4 s later tilt < 25 deg,
height sane, and (walking case) still making forward progress.

By radial symmetry the envelope should repeat every 72 deg; deviations from
that are gait-phase effects (which leg was mid-swing when the push landed).

Usage: MUJOCO_GL=osmesa python3 run_push.py
"""
import os, sys, json
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS

V_X = 45.0
T_PUSH = 3.5          # gait has settled by then
DUR = 0.15            # push duration, s
T_AFTER = 4.0
DIRS = np.arange(0, 360, 30)
FORCES = [2, 4, 6, 8, 11, 14, 18, 22, 27]     # N (2.7 kg robot: 22 N ~ 0.8 g lateral)


def make_data(model, gait):
    data = mujoco.MjData(model)
    q0 = np.array([leg_ik(body_to_leg(i, gait.p_nom[i])) for i in range(N_LEGS)]).flatten()
    data.qpos[0:3] = [0, 0, (gait.h + 14) / 1000.0]
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[7:7 + 15] = q0
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    return data, q0


def trial(model, gait, q0, walk, dir_deg, force_n):
    data, q0 = make_data(model, gait)
    torso = model.body("torso").id
    DT = model.opt.timestep
    f = force_n * np.array([np.cos(np.deg2rad(dir_deg)), np.sin(np.deg2rad(dir_deg)), 0.0])
    tilt_max = 0.0
    fell = False
    pos_at_push = None
    for k in range(int((T_PUSH + DUR + T_AFTER) / DT)):
        t = k * DT
        if t < 1.0 or not walk:
            data.ctrl[:15] = q0
        else:
            ramp = min((t - 1.0) / 0.6, 1.0)
            q, _, _ = gait.joint_targets(t - 1.0, V_X * ramp, 0.0, 0.0)
            data.ctrl[:15] = q.flatten()
        if T_PUSH <= t < T_PUSH + DUR:
            data.xfrc_applied[torso, :3] = f
            if pos_at_push is None:
                pos_at_push = data.xpos[torso].copy()
        else:
            data.xfrc_applied[torso, :3] = 0.0
        mujoco.mj_step(model, data)
        zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
        tilt = np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1)))
        tilt_max = max(tilt_max, tilt)
        if tilt > 60 or data.xpos[torso][2] < 0.050:
            fell = True
            break
    zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
    tilt_end = np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1)))
    recovered = (not fell) and tilt_end < 25 and data.xpos[torso][2] > 0.075
    if walk and recovered:
        prog = (data.xpos[torso] - pos_at_push)[0] * 1000
        recovered = prog > 0.35 * V_X * (DUR + T_AFTER)    # still going somewhere
    return recovered, tilt_max


def envelope(walk):
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    gait = WaveGait()
    _, q0 = make_data(model, gait)
    out = {}
    for d in DIRS:
        best = 0.0
        for F in FORCES:
            ok, tilt_max = trial(model, gait, q0, walk, d, F)
            if ok:
                best = F
            else:
                break
        out[int(d)] = dict(force_n=best, impulse_ns=round(best * DUR, 2))
        print(f"  dir {d:3d}°: survives {best:5.1f} N  (J = {best*DUR:.2f} N·s)")
    return out


if __name__ == "__main__":
    print("=== quiet stance (all 5 feet down, holding pose) ===")
    stand = envelope(walk=False)
    print("=== walking @ 45 mm/s (wave gait, 4 feet down) ===")
    walk = envelope(walk=True)
    res = dict(stand=stand, walk=walk, dur_s=DUR,
               note="force on torso for 0.15 s; last survived per direction")
    with open(os.path.join(HERE, "push_results.json"), "w") as f:
        json.dump(res, f, indent=1)
    sm = min(v["force_n"] for v in stand.values())
    wm = min(v["force_n"] for v in walk.values())
    print(f"\nweakest direction: stance {sm:.0f} N, walking {wm:.0f} N "
          f"(robot weight = {2.7*9.81:.0f} N for scale)")
