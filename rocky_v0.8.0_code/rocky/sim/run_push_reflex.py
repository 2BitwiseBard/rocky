"""Push envelope WITH the crouch-and-brace reflex vs the D017 baseline.

Same harness as run_push.py (0.15 s torso shove, 12 directions, escalating
force, recovery judged 4 s later) but:
  * extended force ladder (up to 90 N — we expect to outgrow the old 53 N cap)
  * reflex ON vs OFF measured back-to-back with identical conditions
  * both quiet stance and walking @ 45 mm/s
  * plus a phase-sensitivity probe: worst direction, push timed at 4 gait
    phases (D017 said envelope deviations from 72 deg symmetry are gait-phase
    effects — check the reflex flattens them)

First run also logs the no-push |gyro_xy| peak during clean walking, which is
what the 1.8 rad/s trip threshold is calibrated against.

Usage: MUJOCO_GL=osmesa python3 run_push_reflex.py [--quick]
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS       # noqa: E402
from pebble_reflex import ReflexSupervisor, body_gyro_xy            # noqa: E402

V_X = 45.0
T_SETTLE = 1.0
T_PUSH = 3.5
DUR = 0.15
T_AFTER = 4.0
DIRS = np.arange(0, 360, 30)
FORCES = [14, 22, 27, 33, 40, 48, 57, 67, 78, 90]


def make_data(model, gait):
    data = mujoco.MjData(model)
    q0 = np.array([leg_ik(body_to_leg(i, gait.p_nom[i]))
                   for i in range(N_LEGS)]).flatten()
    data.qpos[0:3] = [0, 0, (gait.h + 14) / 1000.0]
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[7:7 + 15] = q0
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    return data, q0


def gyro_xy_of(model, data, torso):
    """Body-frame roll/pitch rate magnitude (what the BNO085 will report)."""
    R = data.xmat[torso].reshape(3, 3)
    w_world = data.cvel[torso][0:3]          # rotational part of spatial vel
    w_body = R.T @ w_world
    return body_gyro_xy(w_body)


def trial(model, walk, dir_deg, force_n, reflex_on, push_t=T_PUSH, trip=1.8):
    gait = WaveGait()
    data, q0 = make_data(model, gait)
    torso = model.body("torso").id
    DT = model.opt.timestep
    sup = ReflexSupervisor(gait, gyro_trip=trip, gyro_calm=trip / 2) \
        if reflex_on else None
    f = force_n * np.array([np.cos(np.deg2rad(dir_deg)),
                            np.sin(np.deg2rad(dir_deg)), 0.0])
    tilt_max, gyro_peak, fell = 0.0, 0.0, False
    gyro_steady = 0.0                      # peak in the settled pre-push window
    pos_at_push = None
    total = push_t + DUR + T_AFTER
    for k in range(int(total / DT)):
        t = k * DT
        gyro = gyro_xy_of(model, data, torso)
        gyro_peak = max(gyro_peak, gyro)
        if 2.0 <= t < push_t:
            gyro_steady = max(gyro_steady, gyro)
        if t < T_SETTLE or not walk:
            vx = 0.0
        else:
            vx = V_X * min((t - T_SETTLE) / 0.6, 1.0)
        if sup is not None:
            q, _state = sup.step(t, vx, 0.0, 0.0, gyro)
            data.ctrl[:15] = q.flatten()
        else:
            if walk and t >= T_SETTLE:
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
        if tilt > 60 or data.xpos[torso][2] < 0.050:
            fell = True
            break
    zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
    tilt_end = np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1)))
    ok = (not fell) and tilt_end < 25 and data.xpos[torso][2] > 0.070
    if walk and ok and pos_at_push is not None and not reflex_on:
        prog = (data.xpos[torso] - pos_at_push)[0] * 1000
        ok = prog > 0.35 * V_X * (DUR + T_AFTER)
    # reflex case: braced robots pause on purpose; require SOME resumed progress
    if walk and ok and reflex_on and pos_at_push is not None:
        prog = (data.xpos[torso] - pos_at_push)[0] * 1000
        ok = prog > 0.15 * V_X * (DUR + T_AFTER)
    trips = sup.trip_count if sup else 0
    return ok, tilt_max, gyro_peak, gyro_steady, trips


def envelope(model, walk, reflex_on, dirs, quick=False, trip=1.8):
    out = {}
    for d in dirs:
        best = 0.0
        for F in (FORCES[::2] if quick else FORCES):
            ok, *_ = trial(model, walk, d, F, reflex_on, trip=trip)
            if ok:
                best = F
            else:
                break
        out[int(d)] = best
        print(f"    dir {d:3d}: survives {best:4.0f} N")
    return out


def main():
    quick = "--quick" in sys.argv
    dirs = DIRS[::2] if quick else DIRS
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))

    # -- calibration: steady clean-walk gyro (no push, t in [2.0, 3.5)) ----
    ok, tilt, gyro_pk, gyro_steady, _ = trial(model, walk=True, dir_deg=0,
                                              force_n=0, reflex_on=False)
    trip = round(max(1.8, 1.35 * gyro_steady), 2)
    print(f"clean walk: |gyro_xy| steady peak {gyro_steady:.2f} rad/s "
          f"(startup peak {gyro_pk:.2f}) -> trip threshold {trip} rad/s")

    results = {"clean_walk_gyro_steady": round(gyro_steady, 3),
               "gyro_trip_used": trip}
    for walk in (False, True):
        tag = "walk" if walk else "stand"
        for reflex_on in (False, True):
            name = f"{tag}_{'reflex' if reflex_on else 'baseline'}"
            print(f"  === {name} ===")
            results[name] = envelope(model, walk, reflex_on, dirs, quick,
                                     trip=trip)

    for tag in ("stand", "walk"):
        b = results[f"{tag}_baseline"]
        r = results[f"{tag}_reflex"]
        bmin, bmax = min(b.values()), max(b.values())
        rmin, rmax = min(r.values()), max(r.values())
        gain = (sum(r.values()) - sum(b.values())) / max(sum(b.values()), 1) * 100
        print(f"{tag:5s}: baseline {bmin:.0f}-{bmax:.0f} N -> "
              f"reflex {rmin:.0f}-{rmax:.0f} N   (mean envelope {gain:+.0f} %)")
        results[f"{tag}_summary"] = dict(base_min=bmin, base_max=bmax,
                                         reflex_min=rmin, reflex_max=rmax,
                                         mean_gain_pct=round(gain, 1))

    # -- phase sensitivity at the weakest walking direction ----------------
    if not quick:
        b = results["walk_baseline"]
        worst_dir = min(b, key=b.get)
        gait_T = WaveGait().T
        print(f"  phase probe at weakest walking dir {worst_dir} deg:")
        phases = {}
        for frac in (0.0, 0.25, 0.5, 0.75):
            pt = T_PUSH + frac * gait_T
            surv = {}
            for reflex_on in (False, True):
                lvl = 0
                for Fi in FORCES:
                    ok, *_ = trial(model, True, worst_dir, Fi, reflex_on,
                                   push_t=pt, trip=results["gyro_trip_used"])
                    if ok:
                        lvl = Fi
                    else:
                        break
                surv["reflex" if reflex_on else "base"] = lvl
            phases[f"{frac:.2f}"] = surv
            print(f"    phase +{frac:.2f}T: base {surv['base']} N, "
                  f"reflex {surv['reflex']} N")
        results["phase_probe_dir"] = int(worst_dir)
        results["phase_probe"] = phases

    with open(os.path.join(HERE, "push_reflex_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    print("wrote push_reflex_results.json")


if __name__ == "__main__":
    main()
