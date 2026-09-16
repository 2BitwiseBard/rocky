"""Diagnose the D022 +0.25T weak pocket before fixing it.

push_reflex_results.json says: walking, dir 180, push at T_PUSH + 0.25T:
baseline survives 48 N but the v1 reflex only 33 N — the brace HURTS in
that phase window. Before changing pebble_reflex, watch one failing trial
closely: which leg is mid-swing at brace entry, what does the support
polygon look like through the shove, and which acceptance criterion
actually fails (fall vs tilt vs resumed-progress).

Usage: python3 diag_brace_phase.py
"""
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS       # noqa: E402
from pebble_reflex import ReflexSupervisor, body_gyro_xy            # noqa: E402
from run_push_reflex import make_data, gyro_xy_of, T_SETTLE, V_X    # noqa: E402

T_PUSH = 3.5
DUR = 0.15
T_AFTER = 4.0


def contacts_now(model, data):
    out = np.zeros(N_LEGS, dtype=bool)
    for i in range(N_LEGS):
        gid = model.geom(f"foot{i}").id
        for c in range(data.ncon):
            con = data.contact[c]
            if gid in (con.geom1, con.geom2):
                out[i] = True
                break
    return out


def run(model, dir_deg, force_n, frac, reflex_on, verbose=True):
    gait = WaveGait()
    data, q0 = make_data(model, gait)
    torso = model.body("torso").id
    DT = model.opt.timestep
    push_t = T_PUSH + frac * gait.T
    sup = ReflexSupervisor(gait, gyro_trip=1.8, gyro_calm=0.9) if reflex_on else None
    f = force_n * np.array([np.cos(np.deg2rad(dir_deg)),
                            np.sin(np.deg2rad(dir_deg)), 0.0])
    tilt_max, fell = 0.0, False
    pos_at_push = None
    braced_logged = False
    state_prev = "NORMAL"
    log = []
    total = push_t + DUR + T_AFTER
    for k in range(int(total / DT)):
        t = k * DT
        gyro = gyro_xy_of(model, data, torso)
        vx = 0.0 if t < T_SETTLE else V_X * min((t - T_SETTLE) / 0.6, 1.0)
        if sup is not None:
            q, state = sup.step(t, vx, 0.0, 0.0, gyro)
            data.ctrl[:15] = q.flatten()
            if state != state_prev:
                ph = (sup.t_gait / gait.T) % 1.0
                legph = [(ph + gait.phase_off[i]) % 1.0 for i in range(N_LEGS)]
                swing = [i for i in range(N_LEGS) if legph[i] >= gait.duty]
                sw_s = {i: round((legph[i] - gait.duty) / (1 - gait.duty), 2)
                        for i in swing}
                log.append(f"t={t:.3f}  {state_prev}->{state}  gait_ph={ph:.3f} "
                           f"swing_legs={sw_s}  contacts={contacts_now(model, data).astype(int)}")
                state_prev = state
            if state == "BRACE" and not braced_logged:
                braced_logged = True
        else:
            if t >= T_SETTLE:
                q, _, _ = gait.joint_targets(t - T_SETTLE, vx, 0.0, 0.0)
                data.ctrl[:15] = q.flatten()
            else:
                data.ctrl[:15] = q0
        if push_t <= t < push_t + DUR:
            data.xfrc_applied[torso, :3] = f
            if pos_at_push is None:
                pos_at_push = data.xpos[torso].copy()
        else:
            data.xfrc_applied[torso, :3] = 0.0
        mujoco.mj_step(model, data)
        zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
        tilt = np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1)))
        tilt_max = max(tilt_max, tilt)
        # dense sampling around the shove
        if verbose and push_t - 0.05 <= t <= push_t + 1.2 and k % 50 == 0:
            log.append(f"t={t:.2f} tilt={tilt:5.1f} contacts="
                       f"{contacts_now(model, data).astype(int)} "
                       f"x={data.xpos[torso][0]*1000:6.1f}mm "
                       f"{'REFLEX:' + sup.state if sup else 'base'}")
        if tilt > 60 or data.xpos[torso][2] < 0.050:
            fell = True
            break
    zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
    tilt_end = np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1)))
    prog = (data.xpos[torso] - pos_at_push)[0] * 1000 if pos_at_push is not None else 0
    need = (0.15 if reflex_on else 0.35) * V_X * (DUR + T_AFTER)
    ok_fall = not fell and tilt_end < 25 and data.xpos[torso][2] > 0.070
    ok = ok_fall and prog > need
    print(f"\n=== dir {dir_deg} F={force_n}N frac=+{frac:.2f}T "
          f"{'REFLEX' if reflex_on else 'BASELINE'} ===")
    print(f"  fell={fell} tilt_max={tilt_max:.1f} tilt_end={tilt_end:.1f} "
          f"h_end={data.xpos[torso][2]*1000:.0f}mm")
    print(f"  progress since push: {prog:.0f} mm (need >{need:.0f})  "
          f"-> upright_ok={ok_fall}  overall_ok={ok}")
    if sup:
        print(f"  reflex trips: {sup.trip_count}")
    for line in log:
        print("   ", line)
    return ok


def main():
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    # the pocket: dir 180, +0.25T. v1 reflex died at 40 N (survived 33).
    run(model, 180, 40, 0.25, reflex_on=True)
    run(model, 180, 40, 0.25, reflex_on=False)
    # sanity contrast: phase 0.00 where reflex WINS (48 vs 33)
    run(model, 180, 40, 0.00, reflex_on=True)
    run(model, 180, 40, 0.00, reflex_on=False)


if __name__ == "__main__":
    main()
