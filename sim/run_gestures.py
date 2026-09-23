"""JAZZ HANDS + FIST-MY-BUMP — rendered, contact-triggered, narrated.

Runs gait/pebble_gestures.py in physics:
  * jazz hands with arms 1 & 4 (the pair flanking north — symmetric to
    the camera), claws fluttering on the visual claw joints;
  * fist bump with a mocap "friend fist" (fleshy sphere) that approaches
    the extended cone-fist; the ACTUAL CONTACT (force on foot0, the SEA
    microswitch stand-in) triggers the 12 mm bump-give and the happy
    narration — the same signal path the hardware will use.

Chord-speak: greeting -> AMAZE (jazz peak) -> curious_question (the ask,
fist extended) -> yes (contact!) -> discovery (post-bump joy).

Usage: MUJOCO_GL=osmesa python3 run_gestures.py
Output: pebble_gestures.mp4
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
from pebble_gestures import (jazz_hands, fist_bump, JAZZ_TOTAL,      # noqa: E402
                             BUMP_T, BUMP_TOTAL, BUMP_ARM)
from chordspeak_events import Narrator                               # noqa: E402
from chordspeak2 import write_wav                                    # noqa: E402

FPS = 30
T_SETTLE = 0.8
T_GAP = 0.8
FIST = """
    <body name="friend_fist" mocap="true" pos="0 0.7 0.30">
      <geom name="friend_fist" type="sphere" size="0.034"
            rgba="0.85 0.68 0.55 1" contype="1" conaffinity="1"/>
    </body>
"""


def build_model():
    with open(os.path.join(HERE, "pebble.xml")) as f:
        xml = f.read()
    xml = xml.replace("<worldbody>", "<worldbody>" + FIST)
    return mujoco.MjModel.from_xml_string(xml)


def main():
    model = build_model()
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
    foot_gid = model.geom(f"foot{BUMP_ARM}").id
    fist_gid = model.geom("friend_fist").id
    fist_mocap = model.body("friend_fist").mocapid[0]
    fist_park = np.array([0.0, 0.95, 0.55])     # parked OUT of frame
    DT = model.opt.timestep

    t_jazz0 = T_SETTLE
    t_bump0 = t_jazz0 + JAZZ_TOTAL + T_GAP
    t_hold0 = t_bump0 + BUMP_T["lean"] + BUMP_T["raise_"] + BUMP_T["extend"]
    t_hold1 = t_hold0 + BUMP_T["hold"]
    T_TOTAL = t_bump0 + BUMP_TOTAL + 0.9

    nar = Narrator()
    nar.event(0.25, "greet")
    nar.event(t_jazz0 + 2.3, "goal")             # AMAZE at the jazz peak
    nar.event(t_hold0 - 0.5, "question")         # "fist my bump?"

    renderer = mujoco.Renderer(model, 480, 720)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.95, -14, 90

    frames = []
    spf = int(round(1 / (FPS * DT)))
    bump_t = None
    bump_mm = 0.0
    said_yes = False
    for k in range(int(T_TOTAL / DT)):
        t = k * DT
        # ---- gesture targets
        if t < t_jazz0:
            q = q0.reshape(5, 3)
            claw = np.zeros(N_LEGS)
        elif t < t_jazz0 + JAZZ_TOTAL:
            q, claw = jazz_hands(gait, t - t_jazz0, arm_legs=(1, 4))
        elif t < t_bump0:
            q = q0.reshape(5, 3)
            claw = np.zeros(N_LEGS)
        else:
            if bump_t is not None:
                dtb = t - bump_t
                bump_mm = 12.0 * np.exp(-((dtb - 0.12) / 0.12) ** 2) \
                    if dtb < 0.6 else 0.0
            q, claw = fist_bump(gait, t - t_bump0, bump_mm=bump_mm)
        data.ctrl[:15] = q.flatten()
        data.ctrl[15:20] = claw
        # ---- friend fist trajectory (world frame)
        if t_hold0 - 0.1 <= t < t_hold1 + 0.3:
            foot_w = data.geom_xpos[foot_gid]
            tgt = foot_w + np.array([0, 0.034 + 0.012, 0.0])
            if bump_t is None:
                a = min((t - (t_hold0 - 0.1)) / 0.6, 1.0)
                a = a * a * (3 - 2 * a)
                pos = (1 - a) * fist_park + a * tgt
                if a >= 1.0:                     # press the last few mm
                    pos = tgt + np.array([0, -0.006 * min((t - (t_hold0 + 0.5))
                                                          / 0.3, 1.0), 0]) \
                        if t > t_hold0 + 0.5 else tgt
            else:
                a = min((t - bump_t) / 0.7, 1.0)
                pos = (1 - a) * (foot_w + [0, 0.05, 0]) + a * fist_park
            data.mocap_pos[fist_mocap] = pos
        else:
            data.mocap_pos[fist_mocap] = fist_park
        mujoco.mj_step(model, data)
        # ---- contact detection: the SEA-switch stand-in
        if bump_t is None and t_hold0 <= t < t_hold1:
            for c in range(data.ncon):
                con = data.contact[c]
                if {con.geom1, con.geom2} == {foot_gid, fist_gid}:
                    F = np.zeros(6)
                    mujoco.mj_contactForce(model, data, c, F)
                    if F[0] > 0.5:
                        bump_t = t
                        nar.event(t + 0.05, "resume")      # "yes."
                        nar.event(t + 1.1, "discovery")    # post-bump joy
                        break
        if k % spf == 0:
            look = data.xpos[torso].copy()
            look[2] += 0.02
            cam.lookat[:] = look
            renderer.update_scene(data, cam)
            frames.append(renderer.render())

    total_s = len(frames) / FPS
    print(f"{len(frames)} frames ({total_s:.1f} s); bump contact at "
          f"{bump_t and round(bump_t, 2)} s "
          f"({'DETECTED' if bump_t else '*** NO CONTACT ***'})")
    for tt, w, cut in nar.schedule():
        print(f"  {tt:5.2f}s  {w}")
    track = nar.render(total_s=total_s)
    wav = os.path.join(HERE, "_gest.wav")
    write_wav(wav, track)
    silent = os.path.join(HERE, "_gest.mp4")
    imageio.mimsave(silent, frames, fps=FPS, codec="libx264", quality=8)
    out = os.path.join(HERE, "pebble_gestures.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", silent,
                    "-i", wav, "-c:v", "copy", "-c:a", "aac", "-b:a", "160k",
                    "-shortest", out], check=True)
    os.remove(silent)
    os.remove(wav)
    print("wrote", out)
    assert bump_t is not None, "fist never made contact — retune approach"


if __name__ == "__main__":
    main()
