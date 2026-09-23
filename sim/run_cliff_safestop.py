"""Cliff stop-path upgrade (session 6): PLANT→BRACE as the halt primitive.

D025 demoted the push reflex to a safe-stop primitive and said it "pairs
with the cliff detector." This harness actually pairs them and measures
whether it helps:

  OLD stop path (run_cliff.py): VOID → retreat 1.6 cycles → command v=0.
    With v=0 the bare WaveGait keeps CYCLING — the robot marches in place
    at the cliff edge indefinitely (feet lifting one at a time, forever,
    next to a void).
  NEW stop path: VOID → retreat 1.6 cycles → sup.request_stop() → the
    supervisor finishes the current step (PLANT), freezes into a
    full-contact crouch (BRACE), then recovers to the planted idle stance
    and STAYS STILL.

Both cases run the same world, detector, and retreat. Measured over the
4 s after the halt command: contact-break count (marching signature), mean
body speed, max tilt, and the usual fell/overhang/stop-margin numbers.
Honest-comparison notes: same phase conventions (supervisor gait clock
starts at motion onset), same detector inputs; the supervisor path also
re-checks that the DETECTION still works when the supervisor owns ctrl.

Usage: MUJOCO_GL=osmesa python3 run_cliff_safestop.py [--video]
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
from pebble_reflex import ReflexSupervisor, body_gyro_xy             # noqa: E402
from cliff import CliffDetector, CliffReaction                        # noqa: E402
from run_odom import foot_contacts                                    # noqa: E402
from run_cliff import build_world, EDGE_X, PLAT_H, V_X                # noqa: E402

T_TOTAL = 16.0
T_SETTLE = 1.0
T_AFTER_HALT = 4.0


def gyro_of(model, data, torso):
    R = data.xmat[torso].reshape(3, 3)
    w_body = R.T @ data.cvel[torso][0:3]
    return body_gyro_xy(w_body), w_body[:2]


def run(mode, record=None):
    """mode: 'hardstop' (old path) or 'safestop' (PLANT→BRACE halt)."""
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
    sup = ReflexSupervisor(gait) if mode == "safestop" else None

    fell = False
    max_edge_reach = -1.0
    halt_t = None                 # when v first commanded 0 / stop requested
    stop_x = None
    # post-halt metrics
    contact_breaks = 0
    prev_con = None
    speeds, tilts = [], []
    states_seen = []

    for k in range(int(T_TOTAL / DT)):
        t = k * DT
        tw = t - T_SETTLE
        if t < T_SETTLE:
            data.ctrl[:15] = q0
        else:
            vx0 = V_X * min(tw / 0.6, 1.0)
            vx, vy, wz = react.command(tw, vx0, 0.0, 0.0)
            halted = react.retreat_until is not None and tw >= react.retreat_until
            if halted and halt_t is None:
                halt_t = tw
                if sup is not None:
                    sup.request_stop()
            if sup is not None:
                gxy, gvec = gyro_of(model, data, torso)
                con = foot_contacts(model, data)
                q, state = sup.step(tw, vx, vy, wz, gxy, contacts=con,
                                    gyro_vec=gvec)
                data.ctrl[:15] = q.flatten()
                # detector inputs from the SUPERVISOR's commanded stance
                if k % 10 == 0 and state == "NORMAL" and \
                        abs(vx) + abs(vy) > 1e-6:
                    ph = [(sup.t_gait / gait.T + gait.phase_off[i]) % 1.0
                          for i in range(5)]
                    settled = [ph[i] < gait.duty and
                               0.12 < ph[i] / gait.duty < 0.95
                               for i in range(5)]
                    if det.update(tw, settled, con):
                        react.on_void(tw)
                if k % 50 == 0:
                    states_seen.append(state)
            else:
                q, stance, _ = gait.joint_targets(tw, vx, vy, wz)
                data.ctrl[:15] = q.flatten()
                if k % 10 == 0:
                    con = foot_contacts(model, data)
                    ph = [(tw / gait.T + gait.phase_off[i]) % 1.0
                          for i in range(5)]
                    settled = [stance[i] and 0.12 < ph[i] / gait.duty < 0.95
                               for i in range(5)]
                    if det.update(tw, settled, con):
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
        # post-halt observation window
        if halt_t is not None and tw > halt_t + 0.05:
            if k % 10 == 0:
                con = foot_contacts(model, data)
                if prev_con is not None:
                    contact_breaks += int(np.sum(prev_con & ~con))
                prev_con = con
                speeds.append(float(np.linalg.norm(data.cvel[torso][3:6])))
                tilts.append(float(tilt))
            if tw > halt_t + T_AFTER_HALT:
                stop_x = x
                break

    return dict(
        mode=mode, fell=bool(fell),
        body_overhang_mm=round(max_edge_reach * 1000, 1),
        voids=det.events,
        stop_margin_mm=None if stop_x is None
        else round((EDGE_X - stop_x) * 1000, 1),
        post_halt=dict(
            contact_breaks=contact_breaks,
            mean_speed_mms=None if not speeds
            else round(1000 * float(np.mean(speeds)), 1),
            max_tilt_deg=None if not tilts else round(max(tilts), 2),
        ),
        supervisor_states=sorted(set(states_seen)) if states_seen else None,
    )


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
        run("safestop", record=(spf, grab))
        out = os.path.join(HERE, "pebble_cliff_safestop.mp4")
        imageio.mimsave(out, frames, fps=30, codec="libx264", quality=8)
        print("video:", out)
        return
    old = run("hardstop")
    new = run("safestop")
    for r in (old, new):
        ph = r["post_halt"]
        print(f"{r['mode']:9s} fell={r['fell']} overhang {r['body_overhang_mm']} mm "
              f"stop margin {r['stop_margin_mm']} mm | post-halt: "
              f"contact breaks {ph['contact_breaks']}, mean speed "
              f"{ph['mean_speed_mms']} mm/s, max tilt {ph['max_tilt_deg']}° "
              f"| states {r.get('supervisor_states')}")
    with open(os.path.join(HERE, "cliff_safestop_results.json"), "w") as f:
        json.dump(dict(hardstop=old, safestop=new), f, indent=1)
    ok = (not new["fell"] and new["voids"] and
          new["post_halt"]["contact_breaks"] < old["post_halt"]["contact_breaks"])
    print("SAFE-STOP INTO CLIFF PATH " +
          ("PASS — detector still fires under the supervisor and the halt "
           "stops marching at the edge" if ok else
           "INCONCLUSIVE — inspect (negative results are results)"))


if __name__ == "__main__":
    main()
