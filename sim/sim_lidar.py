"""Sim-lidar: fake the 360° 2D puck with mj_ray fans — SLAM work starts now.

The Phase-3 plan promotes a pulled vacuum-lidar (Batch 3) to core; this
harness generates exactly the data it will produce, so the scan-matching
pipeline, frames, and map hygiene get built BEFORE the puck exists.

World: 3.2 x 2.6 m room + two pillars + a crate. Pebble walks a lap while a
ray fan (N_RAYS over 360°, RATE Hz) fires from the puck pose (torso top,
+60 mm). Rays see only group-3 geoms (world), never the robot's own legs.

v2 (2026-07-31): the legged-odom EKF now runs IN THE LOOP (noisy IMU +
encoders + contacts, zero-yaw-rate settle calibration — run_odom_v2
config), so the archive carries the REAL odometry the SLAM front-end will
be seeded with, not just ground truth. run_slam_lite.py consumes it.

Outputs:
  lidar_scans.npz        scans + ground-truth poses + EKF odometry poses
  laserscan_spec.json    the exact sensor_msgs/LaserScan field contract
  fig_lidar_map.png      occupancy demo built from the scans (quality proof)

Usage: MUJOCO_GL=osmesa python3 sim_lidar.py
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS        # noqa: E402
import rocky_model as rm   # noqa: E402  (D052 spawn height)
from legged_odom import LeggedOdomEKF, quat_to_R, StillnessGate      # noqa: E402
from run_odom import (fk_body, IMU_HZ, GYRO_NOISE, ACC_NOISE,        # noqa: E402
                      QVEL_NOISE, GYRO_BIAS, ACC_BIAS, ENC_NOISE,
                      CONTACT_FORCE_N)

N_RAYS = 360
RATE_HZ = 8.0
RANGE_MAX = 6.0
RANGE_MIN = 0.12
PUCK_DZ = 0.060
T_SETTLE = 2.5           # boot-stand: one StillnessGate calibration window
T_TOTAL = 41.5

# Obstacles hug the corners so the center ring is WALKABLE: the leg span
# sweeps a 0.6 m disc, and the session-4 lap circle ran straight through
# pillar2 — the robot ground along it at 43% speed with skidding feet
# (which no one noticed, because ground-truth poses map fine even from a
# snagged crawl; the EKF noticed immediately: stance feet must not slip).
ROOM = """
    <geom name="wallN" type="box" size="1.6 0.02 0.25" pos="0 1.3 0.25" group="3" rgba="0.5 0.45 0.6 1"/>
    <geom name="wallS" type="box" size="1.6 0.02 0.25" pos="0 -1.3 0.25" group="3" rgba="0.5 0.45 0.6 1"/>
    <geom name="wallE" type="box" size="0.02 1.3 0.25" pos="1.6 0 0.25" group="3" rgba="0.5 0.45 0.6 1"/>
    <geom name="wallW" type="box" size="0.02 1.3 0.25" pos="-1.6 0 0.25" group="3" rgba="0.5 0.45 0.6 1"/>
    <geom name="pillar1" type="cylinder" size="0.09 0.25" pos="1.05 0.75 0.25" group="3" rgba="0.55 0.5 0.65 1"/>
    <geom name="pillar2" type="cylinder" size="0.07 0.25" pos="-1.0 -0.78 0.25" group="3" rgba="0.55 0.5 0.65 1"/>
    <geom name="crate" type="box" size="0.14 0.10 0.12" pos="-1.05 0.85 0.12" euler="0 0 25" group="3" rgba="0.55 0.5 0.65 1"/>
"""


def build_room():
    with open(os.path.join(HERE, "pebble.xml")) as f:
        xml = f.read()
    xml = xml.replace('<geom name="floor"', '<geom name="floor" group="3"')
    xml = xml.replace("<worldbody>", "<worldbody>" + ROOM)
    return mujoco.MjModel.from_xml_string(xml)


def scan(model, data, torso):
    """One 360° fan in the puck's (body-attached, ~horizontal) plane."""
    R = data.xmat[torso].reshape(3, 3)
    origin = data.xpos[torso] + R @ np.array([0, 0, PUCK_DZ])
    group = np.zeros(6, dtype=np.uint8)
    group[3] = 1
    geomid = np.zeros(1, dtype=np.int32)
    ranges = np.full(N_RAYS, np.inf, dtype=np.float32)
    angles = np.linspace(-np.pi, np.pi, N_RAYS, endpoint=False)
    for k, a in enumerate(angles):
        vec = R @ np.array([np.cos(a), np.sin(a), 0.0])
        d = mujoco.mj_ray(model, data, origin, vec, group, 1, -1, geomid)
        if 0 < d <= RANGE_MAX:
            ranges[k] = d if d >= RANGE_MIN else np.inf
    return angles, ranges, origin


def foot_contacts(model, data):
    out = np.zeros(N_LEGS, dtype=bool)
    for i in range(N_LEGS):
        gid = model.geom(f"foot{i}").id
        f = 0.0
        for c in range(data.ncon):
            con = data.contact[c]
            if gid in (con.geom1, con.geom2):
                F = np.zeros(6)
                mujoco.mj_contactForce(model, data, c, F)
                f += F[0]
        out[i] = f > CONTACT_FORCE_N
    return out


def main():
    rng_noise = np.random.default_rng(11)
    model = build_room()
    gait = WaveGait()
    data = mujoco.MjData(model)
    q0 = np.array([leg_ik(body_to_leg(i, gait.p_nom[i]))
                   for i in range(N_LEGS)]).flatten()
    jadr = [model.joint(f"{n}{i}").qposadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]
    vadr = [model.joint(f"{n}{i}").dofadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]
    # start ON the clear center ring (r = 0.55 m, CCW), heading tangent
    data.qpos[0:3] = [-0.55, 0.0, rm.spawn_z_m(gait.h)]    # D052: was h + 14 mm
    data.qpos[3:7] = [np.cos(-np.pi / 4), 0, 0, np.sin(-np.pi / 4)]  # yaw -90
    data.qpos[jadr] = q0
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    torso = model.body("torso").id
    DT = model.opt.timestep
    every = int(round(1 / (RATE_HZ * DT)))
    imu_every = int(round(1 / (IMU_HZ * DT)))

    ekf = LeggedOdomEKF(fk_body, sigma_a=0.6, sigma_g=0.03, sigma_v_meas=0.04)
    ekf.p = data.xpos[torso].copy()
    ekf.q = data.qpos[3:7].copy()     # odom frame starts aligned with truth
    gate = StillnessGate()
    p_prev, v_prev = None, np.zeros(3)

    scans, poses, poses_odom = [], [], []
    for k in range(int(T_TOTAL / DT)):
        t = k * DT
        moving = t >= T_SETTLE
        if not moving:
            data.ctrl[:15] = q0
        else:
            tw = t - T_SETTLE
            # lap: forward + slow turn = the r=0.55 m center ring
            vx, wz = 45.0 * min(tw / 0.6, 1), 0.082
            q, _, _ = gait.joint_targets(tw, vx, 0.0, wz)
            data.ctrl[:15] = q.flatten()
        mujoco.mj_step(model, data)

        if k % imu_every == 0:                    # EKF at 100 Hz
            dt_i = imu_every * DT
            R_true = data.xmat[torso].reshape(3, 3)
            w_body = R_true.T @ data.cvel[torso][0:3]
            p_now = data.xpos[torso].copy()
            v_now = (p_now - p_prev) / dt_i if p_prev is not None else np.zeros(3)
            p_prev = p_now
            a_world = (v_now - v_prev) / dt_i
            v_prev = v_now
            a_meas = R_true.T @ (a_world - ekf.g)
            ekf.predict(w_body + GYRO_BIAS + rng_noise.normal(0, GYRO_NOISE, 3),
                        a_meas + ACC_BIAS + rng_noise.normal(0, ACC_NOISE, 3),
                        dt_i)
            qj = data.qpos[jadr].reshape(5, 3) + \
                rng_noise.normal(0, ENC_NOISE, (5, 3))
            qdj = data.qvel[vadr].reshape(5, 3) + \
                rng_noise.normal(0, QVEL_NOISE, (5, 3))
            contacts = foot_contacts(model, data)
            ekf.update_legs(qj, qdj, contacts)
            if gate.update(t, moving, bool(contacts.all())):
                ekf.update_yaw_rate_zero()        # boot-stand calibration

        if k % every == 0 and t > T_SETTLE + 0.5:
            ang, rng, origin = scan(model, data, torso)
            R = data.xmat[torso].reshape(3, 3)
            yaw = np.arctan2(R[1, 0], R[0, 0])
            scans.append(rng)
            poses.append([origin[0], origin[1], yaw])
            R_e = quat_to_R(ekf.q)
            o_e = ekf.p + R_e @ np.array([0, 0, PUCK_DZ])
            yaw_e = np.arctan2(R_e[1, 0], R_e[0, 0])
            poses_odom.append([o_e[0], o_e[1], yaw_e])
    scans = np.array(scans)
    poses = np.array(poses)
    poses_odom = np.array(poses_odom)
    hit_rate = np.isfinite(scans).mean()
    end_err = np.linalg.norm(poses_odom[-1, :2] - poses[-1, :2])
    yaw_err = np.rad2deg(abs((poses_odom[-1, 2] - poses[-1, 2] + np.pi)
                             % (2 * np.pi) - np.pi))
    print(f"{len(scans)} scans x {N_RAYS} rays, hit rate {hit_rate*100:.0f}%")
    print(f"EKF at lap end: {end_err*1000:.0f} mm, {yaw_err:.2f} deg off truth")

    np.savez_compressed(os.path.join(HERE, "lidar_scans.npz"),
                        scans=scans, poses=poses, poses_odom=poses_odom,
                        angles=np.linspace(-np.pi, np.pi, N_RAYS, endpoint=False))
    spec = {
        "topic": "/pebble/scan",
        "msg": "sensor_msgs/LaserScan",
        "frame_id": "lidar_link",
        "tf_chain": "odom (legged_odom EKF) -> base_link -> lidar_link "
                    "(xyz 0 0 0.060, from URDF once the puck mount exists)",
        "angle_min": -np.pi, "angle_max": np.pi * (1 - 2 / N_RAYS),
        "angle_increment": 2 * np.pi / N_RAYS,
        "time_increment": 0.0, "scan_time": 1 / RATE_HZ,
        "range_min": RANGE_MIN, "range_max": RANGE_MAX,
        "slam_toolbox_notes": [
            "mode: mapping, odom from /pebble/odom (EKF, ~1-7% drift)",
            "the puck tilts with the body (<=2 deg walking) — within "
            "slam_toolbox tolerance; pause scans during BRACE reflex",
            "vacuum pucks: ~1800-2000 samples/rev at 5 Hz typical — this "
            "spec's 360x8 Hz is deliberately conservative",
        ],
    }
    with open(os.path.join(HERE, "laserscan_spec.json"), "w") as f:
        json.dump(spec, f, indent=1)

    # occupancy demo from ground-truth poses (data-quality proof, not SLAM)
    res = 0.02
    W = int(3.6 / res)
    H = int(3.0 / res)
    grid = np.zeros((H, W))
    angles = np.linspace(-np.pi, np.pi, N_RAYS, endpoint=False)
    for s, (px, py, yaw) in zip(scans, poses):
        for a, r in zip(angles, s):
            if not np.isfinite(r):
                continue
            wx = px + r * np.cos(a + yaw)
            wy = py + r * np.sin(a + yaw)
            gx = int((wx + 1.8) / res)
            gy = int((wy + 1.5) / res)
            if 0 <= gx < W and 0 <= gy < H:
                grid[gy, gx] += 1
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 6.4))
    ax.imshow(np.clip(grid, 0, 6), origin="lower", cmap="Purples",
              extent=[-1.8, 1.8, -1.5, 1.5])
    ax.plot(poses[:, 0], poses[:, 1], "-", color="#e0a030", lw=1.5,
            label="robot path")
    ax.set_title(f"sim-lidar occupancy ({len(scans)} scans @ {RATE_HZ:.0f} Hz, "
                 f"ground-truth poses)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_lidar_map.png"), dpi=120)
    print("wrote lidar_scans.npz + laserscan_spec.json + fig_lidar_map.png")


if __name__ == "__main__":
    main()
