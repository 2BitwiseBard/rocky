"""BECKON (B18) — rendered + narrated. "Come here."

Runs gait/pebble_gestures.beckon in physics: lean away from the north
arm, raise it palm-out to the extended waypoint, three slow curls between
extend and carry with the claw opening on each pull-in (the beckoning
finger), lower, recenter.

Chord-speak sync: greeting at the start, the RISING curious_question
lands on the second curl (the ask), acknowledge as the arm lowers ("ok,
I'll wait"). No mocap friend in this one — Rocky asks; whether anyone
comes is the viewer's problem.

Usage: MUJOCO_GL=osmesa python3 run_beckon.py
Output: pebble_beckon.mp4
"""
import os
import subprocess
import sys

import imageio
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "audio"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS        # noqa: E402
from pebble_gestures import beckon, BECKON_T, BECKON_TOTAL           # noqa: E402
from chordspeak_events import Narrator                               # noqa: E402
from chordspeak2 import write_wav                                    # noqa: E402

FPS = 30
T_SETTLE = 0.8


def main():
    with open(os.path.join(HERE, "pebble.xml")) as f:
        model = mujoco.MjModel.from_xml_string(f.read())
    gait = WaveGait()
    data = mujoco.MjData(model)
    q0 = np.array([leg_ik(body_to_leg(i, gait.p_nom[i]))
                   for i in range(N_LEGS)]).flatten()
    jadr = [model.joint(f"{n}{i}").qposadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]
    data.qpos[0:3] = [0, 0, (gait.h + 14) / 1000.0]
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[jadr] = q0
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    torso = model.body("torso").id
    DT = model.opt.timestep

    t_beck0 = T_SETTLE
    t_curls = t_beck0 + BECKON_T["lean"] + BECKON_T["raise_"]
    t_lower = t_curls + BECKON_T["beckon"]
    T_TOTAL = t_beck0 + BECKON_TOTAL + 0.8

    nar = Narrator()
    nar.event(0.25, "greet")
    nar.event(t_curls + 1.1, "question")         # the ask, on curl two
    nar.event(t_lower + 0.3, "resume")           # "ok." (acknowledge)

    renderer = mujoco.Renderer(model, 480, 720)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.95, -16, 55

    frames = []
    spf = int(round(1 / (FPS * DT)))
    for k in range(int(T_TOTAL / DT)):
        t = k * DT
        if t < t_beck0:
            q = q0.reshape(5, 3)
            claw = np.zeros(N_LEGS)
        else:
            q, claw = beckon(gait, t - t_beck0)
        data.ctrl[:15] = q.flatten()
        data.ctrl[15:20] = claw
        mujoco.mj_step(model, data)
        if k % spf == 0:
            look = data.xpos[torso].copy()
            look[2] += 0.02
            cam.lookat[:] = look
            renderer.update_scene(data, cam)
            frames.append(renderer.render())

    total_s = len(frames) / FPS
    print(f"{len(frames)} frames ({total_s:.1f} s)")
    for tt, w, cut in nar.schedule():
        print(f"  {tt:5.2f}s  {w}")
    track = nar.render(total_s=total_s)
    wav = os.path.join(HERE, "_beck.wav")
    write_wav(wav, track)
    silent = os.path.join(HERE, "_beck.mp4")
    imageio.mimsave(silent, frames, fps=FPS, codec="libx264", quality=8)
    out = os.path.join(HERE, "pebble_beckon.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", silent,
                    "-i", wav, "-c:v", "copy", "-c:a", "aac", "-b:a", "160k",
                    "-shortest", out], check=True)
    os.remove(silent)
    os.remove(wav)
    print("wrote", out)


if __name__ == "__main__":
    main()
