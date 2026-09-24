#!/usr/bin/env python3
"""Gesture library v2 in physics (session 8): every GESTURES2 entry runs on
the MuJoCo model from a planted idle, narrated by the chord-speak event
engine, and reports what the physics thought of it — peak tilt, height
sag, foot-contact breaks on the planted legs, and whether it fell.

    MUJOCO_GL=osmesa python3 run_gestures2.py            # metrics + videos
    MUJOCO_GL=osmesa python3 run_gestures2.py --metrics  # metrics only (fast)
    MUJOCO_GL=osmesa python3 run_gestures2.py wave bow   # subset
    python3 run_gestures2.py --metrics --ideal           # the old ideal-servo run

Outputs: pebble_gesture_<name>.mp4 per gesture, pebble_gestures2.mp4 (the
showcase), gestures2_results.json. Exit code = number of gestures that
fell, lifted a planted foot for >0.3 s, or failed the feasibility check.

D052: the targets now go through the servo realism layer (ServoModel on:
50 Hz bus, 20 ms latency, 4.7 rad/s slew, count quantisation) unless
--ideal, and every gesture carries its pebble_feasibility verdict (limits,
speed classes, steps, CoM margin) and the tracking error |qpos - asked|.
The robot spawns at rocky_model.spawn_z_m() (was h + 14 mm).
"""
import json
import os
import subprocess
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "audio"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS        # noqa: E402
from pebble_gestures2 import GESTURES2, WAVE_ARM, kind_of            # noqa: E402
from run_odom import foot_contacts                                    # noqa: E402
from servo_model import ServoModel                                    # noqa: E402
import pebble_feasibility as pf                                       # noqa: E402
import rocky_model as rm                                              # noqa: E402

FPS = 30
T_SETTLE = 0.8
T_TAIL = 0.6
W, H = 540, 360


def build():
    with open(os.path.join(HERE, "pebble.xml")) as f:
        model = mujoco.MjModel.from_xml_string(f.read())
    gait = WaveGait()
    data = mujoco.MjData(model)
    q0 = np.array([leg_ik(body_to_leg(i, gait.p_nom[i]))
                   for i in range(N_LEGS)]).flatten()
    jadr = [model.joint(f"{n}{i}").qposadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]
    data.qpos[0:3] = [0, 0, rm.spawn_z_m(gait.h)]                    # D052 (was h + 14 mm)
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[jadr] = q0
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    return model, data, gait, q0


def tilt_deg(data, torso):
    R = data.xmat[torso].reshape(3, 3)
    return float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))


def run_one(name, video=True, servo=True):
    fn, total, events = GESTURES2[name]
    model, data, gait, q0 = build()
    rep = pf.check(fn, total, g=gait, name=name, kind=kind_of(name))
    sm = ServoModel(on=servo)
    sm.reset(q0)
    jadr = [model.joint(f"{n}{i}").qposadr[0] for i in range(N_LEGS) for n in ("yaw", "hip", "knee")]
    track = []
    torso = model.body("torso").id
    DT = model.opt.timestep
    T_TOTAL = T_SETTLE + total + T_TAIL
    planted = [i for i in range(N_LEGS)
               if not (name == "wave" and i == WAVE_ARM)]
    gaited = name in ("turn_in_place", "sidestep")

    renderer = cam = None
    if video:
        renderer = mujoco.Renderer(model, H, W)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 0.95, -16, 55
    frames = []
    spf = int(round(1 / (FPS * DT)))
    tilt_max, h_min, h0 = 0.0, 1e9, None
    break_time = 0.0
    yaw_total = 0.0
    x0 = data.xpos[torso].copy()
    yaw_peak = 0.0
    for k in range(int(T_TOTAL / DT)):
        t = k * DT
        if t < T_SETTLE:
            q = q0.reshape(5, 3)
            claw = np.zeros(N_LEGS)
        else:
            q, claw = fn(gait, min(t - T_SETTLE, total))
        data.ctrl[:15] = sm.filter(np.asarray(q).flatten(), DT, force=data.actuator_force[:15])
        data.ctrl[15:20] = claw
        mujoco.mj_step(model, data)
        if T_SETTLE <= t <= T_SETTLE + total:
            track.append(np.abs(data.qpos[jadr] - np.asarray(q).flatten()))
        if t > T_SETTLE + 0.2:
            tilt_max = max(tilt_max, tilt_deg(data, torso))
            h = data.xpos[torso][2]
            h_min = min(h_min, h)
            if h0 is None:
                h0 = h
            Rk = data.xmat[torso].reshape(3, 3)
            yaw_peak = max(yaw_peak, abs(float(np.degrees(np.arctan2(Rk[1, 0], Rk[0, 0])))))
            if not gaited:
                c = foot_contacts(model, data)
                if not all(c[i] for i in planted):
                    break_time += DT
        if video and k % spf == 0:
            look = data.xpos[torso].copy()
            look[2] += 0.02
            cam.lookat[:] = look
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
    R = data.xmat[torso].reshape(3, 3)
    yaw_end = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
    disp = (data.xpos[torso] - x0) * 1000.0
    fell = tilt_deg(data, torso) > 25 or data.xpos[torso][2] < 0.05
    res = dict(gesture=name, total_s=round(total, 2), tilt_max_deg=round(tilt_max, 2),
               height_sag_mm=round((h0 - h_min) * 1000, 1) if h0 else None,
               planted_break_s=round(break_time, 2), yaw_end_deg=round(yaw_end, 1),
               yaw_peak_deg=round(yaw_peak, 1),
               disp_mm=[round(float(v), 1) for v in disp[:2]], fell=bool(fell))
    E = np.degrees(np.array(track))
    res.update(servo_model=bool(servo), track_mean_deg=round(float(E.mean()), 2),
               track_max_deg=round(float(E.max()), 1),
               feasible=bool(rep.ok), feasibility=rep.lines, peak_rad_s=round(rep.peak()[0], 2))
    ok = (not fell) and (gaited or break_time < 0.3) and rep.ok
    res["ok"] = bool(ok)
    print(f"{name:14s} tilt_max {tilt_max:5.2f}°  sag {res['height_sag_mm']:6.1f} mm  "
          f"planted-foot breaks {break_time:4.2f} s  yaw peak {yaw_peak:5.1f}° end {yaw_end:+6.1f}°  "
          f"disp {disp[0]:+6.0f},{disp[1]:+6.0f} mm  track {E.mean():4.2f}/{E.max():4.1f}°  "
          f"peak {rep.peak()[0]:4.2f} rad/s  "
          f"{'FELL' if fell else ('INFEASIBLE' if not rep.ok else ('OK' if ok else 'LIFTS'))}",
          flush=True)
    if not rep.ok:
        print("\n".join("    " + ln for ln in rep.lines))
    if video:
        out = os.path.join(HERE, f"pebble_gesture_{name}.mp4")
        _write_video(frames, events, total, out)
        res["video"] = os.path.basename(out)
    return res


def _write_video(frames, events, total, out):
    import imageio
    from chordspeak_events import Narrator
    from chordspeak2 import write_wav
    nar = Narrator()
    for dt, ev in events:
        nar.event(T_SETTLE + dt, ev)
    total_s = len(frames) / FPS
    track = nar.render(total_s=total_s)
    wav = out + ".wav"
    write_wav(wav, track)
    silent = out + ".silent.mp4"
    imageio.mimsave(silent, frames, fps=FPS, codec="libx264", quality=7)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", silent, "-i", wav,
                    "-c:v", "copy", "-c:a", "aac", "-b:a", "128k", "-shortest", out],
                   check=True)
    os.remove(silent)
    os.remove(wav)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    video = "--metrics" not in sys.argv
    servo = "--ideal" not in sys.argv
    names = args or list(GESTURES2)
    results = [run_one(n, video=video, servo=servo) for n in names]
    with open(os.path.join(HERE, "gestures2_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    bad = [r["gesture"] for r in results if not r["ok"]]
    if video and len(names) > 1:
        lst = os.path.join(HERE, "_g2list.txt")
        with open(lst, "w") as f:
            for r in results:
                f.write(f"file '{os.path.join(HERE, r['video'])}'\n")
        show = os.path.join(HERE, "pebble_gestures2.mp4")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0",
                        "-i", lst, "-c", "copy", show], check=True)
        os.remove(lst)
        print("showcase:", show)
    print(f"gesture library v2 in physics: {len(results) - len(bad)}/{len(results)} clean"
          + (f" — {', '.join(bad)}" if bad else ""))
    sys.exit(len(bad))


if __name__ == "__main__":
    main()
