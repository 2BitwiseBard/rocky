"""Cliff-detection test: Pebble walks toward a table edge. Does it stop?

World: a raised platform (80 mm tall) that ENDS at x = 0.35 m. Control case
walks straight off (the fall D017 never sees, because rubble isn't a void).
Detector case: the first foot to probe past the edge fires VOID -> halt +
retreat. Metrics: did it stay on the table, worst body overhang, stop margin.

Usage: MUJOCO_GL=osmesa python3 run_cliff.py [--video]
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
from pebble_gait import WaveGait, leg_ik, leg_fk, body_to_leg, leg_to_body, N_LEGS  # noqa: E402
from cliff import CliffDetector, CliffReaction                       # noqa: E402
from run_odom import foot_contacts                                   # noqa: E402

PLAT_H = 0.160   # deeper than max leg reach-below (52 mm) => a TRUE void
EDGE_X = 0.35
V_X = 45.0
T_TOTAL = 14.0


def build_world():
    with open(os.path.join(HERE, "pebble.xml")) as f:
        xml = f.read()
    plat = (f'<geom name="platform" type="box" '
            f'size="{(EDGE_X + 0.45) / 2:.3f} 0.5 {PLAT_H / 2:.3f}" '
            f'pos="{(EDGE_X - (EDGE_X + 0.45)) / 2 + EDGE_X / 2 - 0.225 + 0.1:.3f} 0 '
            f'{PLAT_H / 2:.3f}" friction="1.2 0.01 0.001" '
            f'rgba="0.45 0.4 0.55 1"/>')
    # simpler: platform spans x in [-0.45, EDGE_X]
    plat = (f'<geom name="platform" type="box" '
            f'size="{(EDGE_X + 0.45) / 2:.3f} 0.5 {PLAT_H / 2:.3f}" '
            f'pos="{(EDGE_X - 0.45) / 2:.3f} 0 {PLAT_H / 2:.3f}" '
            f'friction="1.2 0.01 0.001" rgba="0.45 0.4 0.55 1"/>')
    xml = xml.replace("<worldbody>", "<worldbody>\n    " + plat)
    return mujoco.MjModel.from_xml_string(xml)


def run(detector_on, record=None):
    model = build_world()
    gait = WaveGait()
    data = mujoco.MjData(model)
    q0 = np.array([leg_ik(body_to_leg(i, gait.p_nom[i]))
                   for i in range(N_LEGS)]).flatten()
    jadr = [model.joint(f"{n}{i}").qposadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]
    data.qpos[0:3] = [0, 0, (gait.h + 14) / 1000.0 + PLAT_H]
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[jadr] = q0
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    torso = model.body("torso").id
    DT = model.opt.timestep
    det = CliffDetector()
    react = CliffReaction(gait)
    t_settle = 1.0
    fell = False
    max_edge_reach = -1.0
    stop_x = None
    for k in range(int(T_TOTAL / DT)):
        t = k * DT
        if t < t_settle:
            data.ctrl[:15] = q0
        else:
            tw = t - t_settle
            vx, vy, wz = V_X * min(tw / 0.6, 1.0), 0.0, 0.0
            if detector_on:
                vx, vy, wz = react.command(tw, vx, vy, wz)
            q, stance, feet_cmd = gait.joint_targets(tw, vx, vy, wz)
            data.ctrl[:15] = q.flatten()
            # --- detector inputs: commanded stance flags + contacts ---
            if detector_on and k % 10 == 0:
                contact = foot_contacts(model, data)
                # stance with a settling margin: only legs >12% into stance
                ph = [(tw / gait.T + gait.phase_off[i]) % 1.0 for i in range(5)]
                settled = [stance[i] and 0.12 < ph[i] / gait.duty < 0.95
                           for i in range(5)]
                fired = det.update(tw, settled, contact)
                if fired:
                    react.on_void(tw)
        mujoco.mj_step(model, data)
        if record is not None and k % record[0] == 0:
            record[1](data, t)
        x = data.xpos[torso][0]
        max_edge_reach = max(max_edge_reach, x - EDGE_X)
        z = data.xmat[torso].reshape(3, 3)[:, 2]
        tilt = np.rad2deg(np.arccos(np.clip(z[2], -1, 1)))
        if tilt > 50 or data.xpos[torso][2] < PLAT_H - 0.02:
            fell = True
            break
        if detector_on and react.retreat_until and t - t_settle > \
                react.retreat_until + 2.0:
            stop_x = x
            break
    return dict(detector=detector_on, fell=bool(fell),
                body_overhang_mm=round(max_edge_reach * 1000, 1),
                voids=det.events, stop_x_mm=None if stop_x is None
                else round((EDGE_X - stop_x) * 1000, 1))


def main():
    if "--video" in sys.argv:
        import imageio
        frames = []
        model = build_world()
        renderer = mujoco.Renderer(model, 480, 720)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 0.9, -14, 135

        def grab(data, t):
            cam.lookat[:] = data.xpos[model.body("torso").id]
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
        spf = int(round(1 / (30 * model.opt.timestep)))
        # re-run with recording via run()'s record hook needs the same model;
        # cheap: just rerun detector case
        run(True, record=(spf, grab))
        out = os.path.join(HERE, "pebble_cliff_stop.mp4")
        imageio.mimsave(out, frames, fps=30, codec="libx264", quality=8)
        print("video:", out)
        return
    ctrl = run(False)
    det = run(True)
    print(f"control  (no detector): fell={ctrl['fell']} "
          f"overhang {ctrl['body_overhang_mm']} mm")
    print(f"detector: fell={det['fell']} overhang {det['body_overhang_mm']} mm "
          f"stopped {det['stop_x_mm']} mm short of the edge; "
          f"voids fired: {det['voids']}")
    with open(os.path.join(HERE, "cliff_results.json"), "w") as f:
        json.dump(dict(control=ctrl, detector=det), f, indent=1)
    ok = ctrl["fell"] and not det["fell"]
    print("CLIFF DETECTION " + ("PASS — control falls, detector robot stays up"
                                if ok else "INCONCLUSIVE — inspect"))


if __name__ == "__main__":
    main()
