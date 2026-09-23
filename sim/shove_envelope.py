"""Shove envelope under the D048 shove model (half-sine at the shell rim).

The counterpart of run_push_reflex_v2.py's envelope for the new model:
for standing and walking (bare gait + ReflexSupervisor, as the playground
runs it), climb the peak force per direction until the robot ends up
fallen (tilt > 60°) or displaced more than 0.5 m, and report the last
survivable peak in newtons and bodyweights plus how far/fast it went.

  python shove_envelope.py            # 6 directions x {stand, walk}, ~2 min
  python shove_envelope.py --quick    # 2 directions
writes out/shove_envelope.json
"""
import argparse
import json
import os
import sys

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, N_LEGS                                # noqa: E402
from pebble_reflex import ReflexSupervisor, FALLEN                     # noqa: E402
from run_push_reflex import make_data, gyro_xy_of, T_SETTLE, V_X        # noqa: E402
from righter import foot_contacts                                       # noqa: E402
from shove import Shove, bodyweights, impulse_ns                        # noqa: E402

T_SHOVE = 3.0
DUR = 0.4
FORCES = [10, 15, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80]


def trial(model, walk, dir_deg, peak_n, dur=DUR):
    gait = WaveGait()
    data, q0 = make_data(model, gait)
    torso = model.body("torso").id
    fids = [model.geom(f"foot{i}").id for i in range(N_LEGS)]
    sup = ReflexSupervisor(gait)
    DT = model.opt.timestep
    sh = Shove(peak_n * np.cos(np.radians(dir_deg)), peak_n * np.sin(np.radians(dir_deg)),
               dur=dur, t0=T_SHOVE)
    p0 = None
    vmax = 0.0
    fallen = False
    for k in range(int((T_SHOVE + dur + 3.0) / DT)):
        t = k * DT
        R = data.xmat[torso].reshape(3, 3)
        tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
        w = R.T @ data.cvel[torso][0:3]
        vx = V_X * min(max(t - T_SETTLE, 0.0) / 0.6, 1.0) if walk else 0.0
        q, state = sup.step(t, vx, 0.0, 0.0, gyro_xy_of(model, data, torso),
                            contacts=foot_contacts(model, data, fids), gyro_vec=w[:2],
                            tilt_deg=tilt, height=float(data.xpos[torso][2]))
        data.ctrl[:15] = q.flatten()
        if t >= T_SHOVE and p0 is None:
            p0 = data.xpos[torso][:2].copy()
        sh.apply(model, data, torso, t)
        mujoco.mj_step(model, data)
        if p0 is not None:
            vmax = max(vmax, float(np.linalg.norm(data.cvel[torso][3:5])))
            if state == FALLEN or tilt > 60:
                fallen = True
    disp = float(np.linalg.norm(data.xpos[torso][:2] - p0)) if p0 is not None else 0.0
    R = data.xmat[torso].reshape(3, 3)
    end_tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
    ok = (not fallen) and end_tilt < 25 and disp < 0.5
    return ok, dict(peak_n=peak_n, ok=ok, fallen=fallen, end_tilt=round(end_tilt, 1),
                    disp_mm=round(disp * 1000), vmax=round(vmax, 2))


def climb(model, walk, dir_deg):
    last_ok, first_fail = None, None
    for F in FORCES:
        ok, row = trial(model, walk, dir_deg, F)
        if ok:
            last_ok = row
        else:
            first_fail = row
            break
    return last_ok, first_fail


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    dirs = [0, 180] if args.quick else [0, 60, 120, 180, 240, 300]
    out = {"model": "half-sine at the shell rim (shove.py), 0.4 s", "dur_s": DUR,
           "mass_kg": round(float(model.body_subtreemass[0]), 3), "rows": []}
    for walk in (False, True):
        floors = []
        for d in dirs:
            ok, fail = climb(model, walk, d)
            floor = ok["peak_n"] if ok else 0
            floors.append(floor)
            out["rows"].append(dict(walk=walk, dir_deg=d, survive_peak_n=floor,
                                    survive_bw=round(bodyweights(floor, model), 2),
                                    survive_impulse_ns=round(impulse_ns(floor, DUR), 1),
                                    last_ok=ok, first_fail=fail))
            print(f"  {'walk ' if walk else 'stand'} {d:3d}°: survives {floor:2d} N peak "
                  f"({bodyweights(floor, model):.2f} BW, {impulse_ns(floor, DUR):.1f} N·s)"
                  f" | last ok: {ok} | fails at: {fail}")
        out[f"{'walk' if walk else 'stand'}_min_bw"] = round(bodyweights(min(floors), model), 2)
        out[f"{'walk' if walk else 'stand'}_mean_bw"] = round(bodyweights(float(np.mean(floors)), model), 2)
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    with open(os.path.join(HERE, "out", "shove_envelope.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print(f"stand min {out['stand_min_bw']} BW / mean {out['stand_mean_bw']}; "
          f"walk min {out['walk_min_bw']} BW / mean {out['walk_mean_bw']}  -> out/shove_envelope.json")


if __name__ == "__main__":
    main()
