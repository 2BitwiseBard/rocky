"""Yaw-observability fix for the legged-odom EKF — measure it.

The 07-30 result: 1.0–6.8 % translational drift but ~6° of yaw error in
21 s. That 6° is almost exactly GYRO_BIAS_z x time: nothing in the
IMU+legs stack observes yaw or gyro-z bias (legs constrain velocity,
gravity constrains roll/pitch). Fix (perception/legged_odom.py):
standing-still ZERO-YAW-RATE pseudo-updates — while the robot is quiet the
bias-corrected gyro-z should read 0, and the residual is exactly the bias
error. Fires during the pre-walk settle and every patrol pause. Optional
magnetometer stub (update_mag_yaw) also implemented, gated hard + weighted
low (indoor mag near motors is a liar); measured here as a comparison
column, default OFF in the recommendation.

The stillness gate matters: the settle after ANY pose change carries a
decaying real twist (position-servo relaxation, 0.3–1.2 deg/s — exactly
bias scale), so updates fire only after 1.2 s of quiet + full contact
(StillnessGate). Fired too early they calibrate the creep in with the
wrong sign — measured bg_z -0.0109 vs true +0.0050.

Scenes (2.5 s boot-settle so the gate opens once before walking):
  flat_straight  22.5 s continuous walk (one calibration window at boot)
  flat_turny     22.5 s walk with S-turns (same profile as 07-30)
  patrol_pauses  48 s: walk 6 s / stand 2.5 s cycles (realistic patrol)

Each scene runs with the v1 EKF and with +zupt (and +mag on patrol).
While standing the harness commands the PLANTED stance (marching in place
would lift feet) and freezes the gait clock, mirroring pebble_reflex.

Usage: python3 run_odom_v2.py
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS               # noqa: E402
from legged_odom import LeggedOdomEKF, quat_to_R, StillnessGate             # noqa: E402
from run_odom import (fk_body, foot_contacts, IMU_HZ, MM, GYRO_NOISE,       # noqa: E402
                      ACC_NOISE, QVEL_NOISE, GYRO_BIAS, ACC_BIAS, ENC_NOISE)
from run_terrain import build_model, init_robot                             # noqa: E402

RNG = np.random.default_rng(7)
MAG_NOISE = np.deg2rad(6.0)      # stub magnetometer 1-sigma
MAG_HZ = 10.0


def cmd_patrol(t):
    """46 s of walk-6 / stand-2.5 cycles with 0.4 s ramps."""
    seg = t % 8.5
    if seg >= 6.0:
        return (0.0, 0.0, 0.0)
    ramp = min(seg / 0.4, 1.0, max((6.0 - seg) / 0.4, 0.0))
    return (45.0 * ramp, 0.0, 0.0)


def run_scene(name, T_total, cmd_fn, zupt=False, mag=False, seed=3):
    rng = np.random.default_rng(seed)
    model = build_model(0, seed=1)
    gait = WaveGait()
    data, q0 = init_robot(model, gait, extra_z=0.0)
    torso = model.body("torso").id
    DT = model.opt.timestep
    imu_every = int(round(1 / (IMU_HZ * DT)))
    mag_every = int(round(1 / (MAG_HZ * DT)))
    jadr = [model.joint(f"{n}{i}").qposadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]
    vadr = [model.joint(f"{n}{i}").dofadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]
    ekf = LeggedOdomEKF(fk_body, sigma_a=0.6, sigma_g=0.03, sigma_v_meas=0.04)
    ekf.p = data.xpos[torso].copy()
    t_settle = 2.5          # realistic boot: stand a beat before walking
    gate = StillnessGate()  # creep-aware stillness (see legged_odom.py)
    gt_path, est_path = [], []
    yaw_err_series = []
    t_gait = 0.0
    p_prev, v_prev = None, np.zeros(3)
    zupt_fired = 0
    for k in range(int(T_total / DT)):
        t = k * DT
        vx, vy, wz = (0.0, 0.0, 0.0) if t < t_settle else cmd_fn(t - t_settle)
        moving = abs(vx) + abs(vy) + abs(wz) * 100 >= 1.0
        if moving:
            t_gait += DT
            q, _, _ = gait.joint_targets(t_gait, vx, vy, wz)
            data.ctrl[:15] = q.flatten()
        else:
            data.ctrl[:15] = q0                      # PLANTED stance
        mujoco.mj_step(model, data)

        if k % imu_every == 0:
            dt_i = imu_every * DT
            R_true = data.xmat[torso].reshape(3, 3)
            w_body = R_true.T @ data.cvel[torso][0:3]
            p_now = data.xpos[torso].copy()
            v_now = (p_now - p_prev) / dt_i if p_prev is not None else np.zeros(3)
            p_prev = p_now
            a_world = (v_now - v_prev) / dt_i
            v_prev = v_now
            a_meas = R_true.T @ (a_world - ekf.g)
            gyro = w_body + GYRO_BIAS + rng.normal(0, GYRO_NOISE, 3)
            accel = a_meas + ACC_BIAS + rng.normal(0, ACC_NOISE, 3)
            ekf.predict(gyro, accel, dt_i)
            qj = data.qpos[jadr].reshape(5, 3) + rng.normal(0, ENC_NOISE, (5, 3))
            qdj = data.qvel[vadr].reshape(5, 3) + rng.normal(0, QVEL_NOISE, (5, 3))
            contacts = foot_contacts(model, data)
            ekf.update_legs(qj, qdj, contacts)
            if zupt and gate.update(t, moving, bool(contacts.all())):
                zupt_fired += ekf.update_yaw_rate_zero()
            if mag and k % mag_every == 0:
                yaw_true = np.arctan2(R_true[1, 0], R_true[0, 0])
                ekf.update_mag_yaw(yaw_true + rng.normal(0, MAG_NOISE))
            gt_path.append(data.xpos[torso].copy())
            est_path.append(ekf.p.copy())
            yaw_true = np.arctan2(R_true[1, 0], R_true[0, 0])
            yerr = np.rad2deg(abs((ekf.yaw - yaw_true + np.pi) % (2 * np.pi) - np.pi))
            yaw_err_series.append(yerr)

    gt = np.array(gt_path)
    est = np.array(est_path)
    dist = np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))
    err_end = np.linalg.norm(est[-1, :2] - gt[-1, :2])
    tag = "zupt" if zupt else "v1"
    tag += "+mag" if mag else ""
    res = dict(scene=name, ekf=tag, dist_m=round(float(dist), 2),
               drift_end_mm=round(err_end * 1000, 1),
               drift_pct=round(err_end / max(dist, 1e-9) * 100, 2),
               yaw_err_end_deg=round(float(yaw_err_series[-1]), 2),
               yaw_err_max_deg=round(float(np.max(yaw_err_series)), 2),
               bgz_est=round(float(ekf.bg[2]), 5), bgz_true=float(GYRO_BIAS[2]),
               zupt_updates=int(zupt_fired))
    print(f"{name:14s} {tag:8s}: dist {dist:5.2f} m | drift "
          f"{err_end*1000:6.1f} mm ({res['drift_pct']:4.2f}%) | yaw end "
          f"{res['yaw_err_end_deg']:5.2f}° max {res['yaw_err_max_deg']:5.2f}° | "
          f"bg_z {ekf.bg[2]:+.4f} (true {GYRO_BIAS[2]:+.4f}) | zupt x{zupt_fired}")
    return res, yaw_err_series


def main():
    scenes = [
        ("flat_straight", 22.5, lambda t: (45.0, 0.0, 0.0)),
        ("flat_turny", 22.5,
         lambda t: (40.0, 0.0, 0.35 * np.sin(2 * np.pi * t / 8))),
        ("patrol_pauses", 48.0, cmd_patrol),
    ]
    results, series = [], {}
    for name, T, fn in scenes:
        for zupt in (False, True):
            r, ys = run_scene(name, T, fn, zupt=zupt)
            results.append(r)
            series[f"{name}_{r['ekf']}"] = ys
    r, ys = run_scene("patrol_pauses", 48.0, cmd_patrol, zupt=True, mag=True)
    results.append(r)
    series["patrol_pauses_zupt+mag"] = ys

    with open(os.path.join(HERE, "odom_v2_results.json"), "w") as f:
        json.dump(results, f, indent=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.4))
    for ax, scene in zip(axes, ("flat_straight", "flat_turny", "patrol_pauses")):
        for key, style, c in ((f"{scene}_v1", "--", "#bbb"),
                              (f"{scene}_zupt", "-", "#7b3fa0"),
                              (f"{scene}_zupt+mag", ":", "#2a9d8f")):
            if key in series:
                ys = series[key]
                ts = np.arange(len(ys)) / IMU_HZ
                ax.plot(ts, ys, style, color=c, lw=1.6,
                        label=key.split("_")[-1])
        ax.set_title(scene)
        ax.set_xlabel("t (s)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("|yaw error| (deg)")
    axes[0].legend(fontsize=8)
    fig.suptitle("EKF yaw drift: standing zero-yaw-rate pseudo-update "
                 "(+ optional mag stub)")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_odom_v2_yaw.png"), dpi=120)
    print("wrote odom_v2_results.json + fig_odom_v2_yaw.png")


if __name__ == "__main__":
    main()
