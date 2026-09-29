"""Shared scene builders and helpers — the parts of the sim experiments that
live code uses.

These used to live inside one-off experiment scripts (run_cliff, run_terrain,
run_odom, run_push_reflex, run_reflex_fallen), so the MCP harness, the
playground, the world builder, sim_lidar, shove_envelope, audit_righter and
the tests imported experiment scripts. They live here now; the experiments in
sim/experiments/ import them from here too, so their numbers are unchanged.

    build_world()                 the cliff world: a 160 mm platform ending at EDGE_X
    build_model(amp_mm, seed)     pebble.xml + a rubble field of boxes (max height amp_mm)
    init_robot(model, gait, ...)  planted stance spawn on a rubble / raised floor
    make_data(model, gait)        planted stance spawn on the flat floor (the push harness)
    gyro_xy_of(model, data, id)   body-frame roll/pitch rate magnitude (what the IMU reports)
    fk_body(i, theta)             leg FK in the body frame, metres (the odometry EKF's model)
    foot_contacts(model, data)    world contact per foot at the experiments' 1.5 N, no hysteresis
    IMU noise model               IMU_HZ, GYRO_NOISE, ACC_NOISE, QVEL_NOISE, *_BIAS, ENC_NOISE
    fallen-demo shove (D048)      SHOVE_N, SHOVE_S, T_SHOVE, T_FALLEN
"""
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for _sub in ("gait", "perception"):
    _p = os.path.join(HERE, "..", _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
from pebble_gait import leg_ik, leg_fk, body_to_leg, leg_to_body, N_LEGS   # noqa: E402
import rocky_model as rm                                                  # noqa: E402
from pebble_reflex import body_gyro_xy                                    # noqa: E402

MODEL_XML = os.path.join(HERE, "pebble.xml")
MM = 1e-3
V_X = 45.0               # mm/s: the experiments' walking speed (an ASK; walk_ask() fits it)
T_SETTLE = 1.0           # s: planted stance before the walk starts


def walk_ask(gait, vx=V_X):
    """(vx, vy, wz): the walking ask fitted into the gait's envelope
    (WaveGait.budget, the budget every command source gets; D063). 45 mm/s
    asks 34.2 today. Pass this to a ReflexSupervisor, not V_X: its command
    slew would otherwise ramp toward 45, above the envelope."""
    return gait.budget(vx, 0.0, 0.0)


def load_xml():
    """pebble.xml as text (to splice a world into its <worldbody>)."""
    with open(MODEL_XML) as f:
        return f.read()


# ------------------------------------------------------------ cliff world (D043)
PLAT_H = 0.160           # deeper than max leg reach-below (52 mm) => a TRUE void
EDGE_X = 0.35            # the platform spans x in [-0.45, EDGE_X]


def build_world():
    """pebble.xml on a raised platform that ENDS at x = EDGE_X (the cliff world)."""
    plat = (f'<geom name="platform" type="box" '
            f'size="{(EDGE_X + 0.45) / 2:.3f} 0.5 {PLAT_H / 2:.3f}" '
            f'pos="{(EDGE_X - 0.45) / 2:.3f} 0 {PLAT_H / 2:.3f}" '
            f'friction="1.2 0.01 0.001" rgba="0.45 0.4 0.55 1"/>')
    xml = load_xml().replace("<worldbody>", "<worldbody>\n    " + plat)
    return mujoco.MjModel.from_xml_string(xml)


# ------------------------------------------------------------ rubble field
N_BOX = 150
FIELD = (-0.25, 0.75, -0.35, 0.35)   # x0 x1 y0 y1 walk corridor, m


def build_model(amp_mm, seed, xml=None):
    """pebble.xml (or `xml`, a patched copy of it) + a rubble field of boxes, max
    height amp_mm (side 20-40 mm, height amp/2..amp, random yaw; the start stance
    is kept clear). The same (amp_mm, seed) is the same field whatever the robot."""
    boxes = []
    if amp_mm > 0:
        rng = np.random.default_rng(seed)
        for _ in range(N_BOX):
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
    xml = (load_xml() if xml is None else xml).replace("<worldbody>", "<worldbody>\n    " + "\n    ".join(boxes))
    return mujoco.MjModel.from_xml_string(xml)


def _stance(gait):
    return np.array([leg_ik(body_to_leg(i, gait.p_nom[i])) for i in range(N_LEGS)]).flatten()


def _spawn(model, gait, z):
    data = mujoco.MjData(model)
    q0 = _stance(gait)
    data.qpos[0:3] = [0, 0, z]
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[[model.joint(f"{n}{i}").qposadr[0] for i in range(N_LEGS)
               for n in ("yaw", "hip", "knee")]] = q0   # claws interleave in qpos: never qpos[7:22]
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    return data, q0


def init_robot(model, gait, extra_z=0.0):
    """(data, q0): the planted stance, spawned extra_z above the floor (rubble)."""
    return _spawn(model, gait, rm.spawn_z_m(gait.h, platform_z_m=extra_z))   # D052: was h + 14 mm


def make_data(model, gait):
    """(data, q0): the planted stance on the flat floor (the push harness's spawn)."""
    return _spawn(model, gait, rm.spawn_z_m(gait.h))                          # D052: was h + 14 mm


def gyro_xy_of(model, data, torso):
    """Body-frame roll/pitch rate magnitude (what the BNO085 will report)."""
    R = data.xmat[torso].reshape(3, 3)
    w_world = data.cvel[torso][0:3]          # rotational part of spatial vel
    w_body = R.T @ w_world
    return body_gyro_xy(w_body)


# ------------------------------------------------------------ IMU + encoders + contacts
IMU_HZ = 100.0
GYRO_NOISE = 0.015       # rad/s per sample
ACC_NOISE = 0.35         # m/s^2 per sample (impact-laden legged accel)
QVEL_NOISE = 0.03        # rad/s — servo PRESENT_SPEED telemetry noise
GYRO_BIAS = np.array([0.006, -0.004, 0.005])
ACC_BIAS = np.array([0.05, -0.03, 0.08])
ENC_NOISE = np.deg2rad(0.12)
CONTACT_FORCE_N = 1.5    # the experiments' microswitch stand-in (the params switch is 2.0 / 1.0 N)


def fk_body(i, theta):
    """Foot position of leg i in the body frame, metres."""
    return leg_to_body(i, leg_fk(theta)) * MM


def foot_contacts(model, data):
    """Contact flags per foot geom (the microswitch stand-in), through
    perception/contacts.py — world contacts only — at CONTACT_FORCE_N, no
    hysteresis: the calibration the odometry / cliff / gesture experiments
    (and the harness's SimBackend) were measured with. New code: call
    contacts.foot_contacts with the params switch."""
    from contacts import foot_contacts as _fc
    fids = [model.geom(f"foot{i}").id for i in range(N_LEGS)]
    return _fc(model, data, fids, close_n=CONTACT_FORCE_N)


# ------------------------------------------------------------ the fallen demo's shove (D048)
# Half-sine at the shell rim (sim/shove.py). 40 N peak over 0.4 s = 10.2 N·s, 1.5 BW
# peak: 1.6x the standing tip-over threshold of the D048 model; it felled the walking
# robot 5/5 seeds. The fallen demo (experiments/run_reflex_fallen) and audit_righter use it.
SHOVE_N = 40.0
SHOVE_S = 0.4
T_SHOVE = 3.0            # s into the episode
T_FALLEN = 22.0          # s: the whole fallen episode (shove, fall, righting, walk away)
