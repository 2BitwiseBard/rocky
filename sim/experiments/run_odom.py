"""Legged-odometry EKF vs MuJoCo ground truth — the drift number.

Runs the walking sim while feeding the EKF exactly what hardware will give:
noisy IMU (gyro/accel + bias), noisy joint encoders, and contact flags from
foot forces (the SEA microswitch stand-in). Compares against ground truth
and against the naive baseline (integrating the commanded gait velocity).

Scenarios: flat 20 s walk, flat walk with turns, 20 mm rubble walk.
Outputs drift stats + odom_results.json + fig_odom.png.
Usage: MUJOCO_GL=egl .venv/bin/python sim/experiments/run_odom.py
"""
import json

import mujoco
import numpy as np

import exp_paths as X                   # sys.path (sim/, gait/, perception/, audio/) + where results / clips go
from pebble_gait import WaveGait                                    # noqa: E402
from legged_odom import LeggedOdomEKF                                # noqa: E402
from scenes import (build_model, init_robot, fk_body, foot_contacts,  # noqa: E402,F401  (re-exported)
                    IMU_HZ, MM, GYRO_NOISE, ACC_NOISE, QVEL_NOISE, GYRO_BIAS, ACC_BIAS,
                    ENC_NOISE, CONTACT_FORCE_N)

RNG = np.random.default_rng(3)


def run_scene(name, amp_mm, T_total, cmd_fn):
    model = build_model(amp_mm, seed=1)
    gait = WaveGait()
    data, q0 = init_robot(model, gait, extra_z=amp_mm / 1000.0)
    torso = model.body("torso").id
    DT = model.opt.timestep
    imu_every = int(round(1 / (IMU_HZ * DT)))

    # joint ADDRESSES — the BUILD_LOG qpos gotcha: joints interleave
    # (yaw,hip,knee,claw)x5, so qpos[7:22] silently mixes claw angles into
    # the FK. Address lookup or bust.
    jadr = [model.joint(f"{n}{i}").qposadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]
    vadr = [model.joint(f"{n}{i}").dofadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]

    ekf = LeggedOdomEKF(fk_body, sigma_a=0.6, sigma_g=0.03,
                    sigma_v_meas=0.04)
    # init: true pose (odometry frames start aligned)
    ekf.p = data.xpos[torso].copy()
    naive = ekf.p[:2].copy()
    gt_path, ekf_path, naive_path = [], [], []
    t_settle = 1.0
    p_prev = None
    v_prev = np.zeros(3)
    for k in range(int(T_total / DT)):
        t = k * DT
        vx, vy, wz = (0.0, 0.0, 0.0) if t < t_settle else gait.budget(*cmd_fn(t - t_settle))
        #            D063: every scene's ask fitted into the envelope (45 asks 34.2, scenes.walk_ask)
        ramp = min((t - t_settle) / 0.6, 1.0) if t >= t_settle else 0.0
        q, _, _ = gait.joint_targets(max(t - t_settle, 0), vx * ramp, vy * ramp,
                                     wz * ramp)
        data.ctrl[:15] = q.flatten()
        mujoco.mj_step(model, data)

        if k % imu_every == 0:
            dt_i = imu_every * DT
            R_true = data.xmat[torso].reshape(3, 3)
            w_body = R_true.T @ data.cvel[torso][0:3]
            # accelerometer = specific force: R^T (a_world - g). a_world by
            # double-differencing the TORSO ORIGIN position (cvel's linear
            # part lives at the subtree COM — wrong point for an IMU)
            p_now = data.xpos[torso].copy()
            v_now = (p_now - p_prev) / dt_i if p_prev is not None else np.zeros(3)
            p_prev = p_now
            a_world = (v_now - v_prev) / dt_i
            v_prev = v_now
            a_meas = R_true.T @ (a_world - ekf.g)
            gyro = w_body + GYRO_BIAS + RNG.normal(0, GYRO_NOISE, 3)
            accel = a_meas + ACC_BIAS + RNG.normal(0, ACC_NOISE, 3)
            ekf.predict(gyro, accel, dt_i)
            # joints: encoder positions + SERVO VELOCITY TELEMETRY (the
            # STS3215 reports PRESENT_SPEED — differentiating noisy encoder
            # positions at 100 Hz instead poisons the filter, measured 62%
            # drift; with reported velocity it drops to a few percent)
            qj = data.qpos[jadr].reshape(5, 3) + \
                RNG.normal(0, ENC_NOISE, (5, 3))
            qdj = data.qvel[vadr].reshape(5, 3) + \
                RNG.normal(0, QVEL_NOISE, (5, 3))
            contacts = foot_contacts(model, data)
            ekf.update_legs(qj, qdj, contacts)
            # naive baseline: integrate the COMMANDED velocity (open loop)
            naive = naive + np.array([vx * ramp, vy * ramp]) * MM * dt_i
            gt_path.append(data.xpos[torso].copy())
            ekf_path.append(ekf.p.copy())
            naive_path.append(naive.copy())

    gt = np.array(gt_path)
    est = np.array(ekf_path)
    nav = np.array(naive_path)
    dist = np.sum(np.linalg.norm(np.diff(gt[:, :2], axis=0), axis=1))
    err_end = np.linalg.norm(est[-1, :2] - gt[-1, :2])
    err_naive = np.linalg.norm(nav[-1] - gt[-1, :2])
    err_rms = float(np.sqrt(np.mean(np.sum((est[:, :2] - gt[:, :2])**2, 1))))
    yaw_true = np.arctan2(data.xmat[torso].reshape(3, 3)[1, 0],
                          data.xmat[torso].reshape(3, 3)[0, 0])
    dyaw = np.rad2deg(abs((ekf.yaw - yaw_true + np.pi) % (2 * np.pi) - np.pi))
    res = dict(scene=name, dist_m=round(float(dist), 3),
               drift_end_mm=round(err_end * 1000, 1),
               drift_pct=round(err_end / max(dist, 1e-9) * 100, 2),
               rms_mm=round(err_rms * 1000, 1),
               naive_end_mm=round(err_naive * 1000, 1),
               yaw_err_deg=round(float(dyaw), 2))
    print(f"{name:14s}: dist {dist:5.2f} m | EKF end-drift "
          f"{err_end*1000:6.1f} mm ({res['drift_pct']:.2f}%) rms {err_rms*1000:5.1f}"
          f" | naive {err_naive*1000:6.1f} mm | yaw err {dyaw:5.2f} deg")
    return res, gt, est, nav


def main():
    scenes = [
        ("flat_straight", 0, 21.0, lambda t: (45.0, 0.0, 0.0)),
        ("flat_turny", 0, 21.0,
         lambda t: (40.0, 0.0, 0.35 * np.sin(2 * np.pi * t / 8))),
        ("rubble20_walk", 20, 21.0, lambda t: (45.0, 0.0, 0.0)),
    ]
    results, paths = [], {}
    for name, amp, T, fn in scenes:
        r, gt, est, nav = run_scene(name, amp, T, fn)
        results.append(r)
        paths[name] = (gt, est, nav)
    with open(X.result("odom_results.json"), "w") as f:
        json.dump(results, f, indent=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6))
    for ax, (name, (gt, est, nav)) in zip(axes, paths.items()):
        ax.plot(gt[:, 0], gt[:, 1], "k-", lw=2, label="ground truth")
        ax.plot(est[:, 0], est[:, 1], "-", color="#8b5fbf", lw=1.6,
                label="legged-odom EKF")
        ax.plot(nav[:, 0], nav[:, 1], "--", color="#bbb", lw=1.2,
                label="naive cmd integration")
        ax.set_title(name)
        ax.axis("equal")
        ax.grid(alpha=0.3)
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle("Legged odometry (FK-through-stance + IMU EKF) vs truth")
    fig.tight_layout()
    fig.savefig(X.result("fig_odom.png"), dpi=120)
    print("wrote odom_results.json + fig_odom.png")


if __name__ == "__main__":
    main()
