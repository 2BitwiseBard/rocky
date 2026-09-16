"""Showcase reel v0.7 — the current repertoire in one narrated video.

Three scenes, chord-speak narrated, concatenated:
  1. LOCOMOTION MEDLEY  — walk / strafe / turn-in-place (the omni gait)
  2. PUSH + SAFE-STOP   — a 40 N shove mid-stride; the v2 PLANT->BRACE
                          supervisor (D025) rides real contacts + the gyro
                          vector, braces, recovers, resumes
  3. CLIFF STOP         — walks at a real void (D024 detector): VOID fires
                          on stance-without-contact, halt + retreat

Replaces the session-2-era pebble_sim.mp4 as the "what can it do" reel —
those older clips predate the reflex rework, the watchdog, and the voice.

Usage: MUJOCO_GL=osmesa python3 run_showcase.py
Output: pebble_showcase_v07.mp4
"""
import os
import subprocess
import sys

import imageio
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
sys.path.insert(0, os.path.join(HERE, "..", "audio"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS        # noqa: E402
from pebble_reflex import ReflexSupervisor                           # noqa: E402
from cliff import CliffDetector, CliffReaction                       # noqa: E402
from run_push_reflex import make_data, gyro_xy_of                    # noqa: E402
from run_push_reflex_v2 import foot_contacts, TRIP                   # noqa: E402
from run_cliff import build_world, PLAT_H, EDGE_X                    # noqa: E402
from run_odom import foot_contacts as foot_contacts_odom             # noqa: E402
from chordspeak_events import Narrator                               # noqa: E402
from chordspeak2 import write_wav                                    # noqa: E402

FPS = 30


def _cam(model, dist=0.95, elev=-14, azim=135):
    r = mujoco.Renderer(model, 480, 720)
    c = mujoco.MjvCamera()
    c.distance, c.elevation, c.azimuth = dist, elev, azim
    return r, c


def scene_medley():
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    gait = WaveGait()
    data, q0 = make_data(model, gait)
    torso = model.body("torso").id
    renderer, cam = _cam(model, azim=140)
    DT = model.opt.timestep
    spf = int(round(1 / (FPS * DT)))
    frames, ev = [], [(0.3, "greet")]
    T = 11.5
    for k in range(int(T / DT)):
        t = k * DT
        if t < 1.0:
            data.ctrl[:15] = q0
        else:
            tw = t - 1.0
            if tw < 4.0:
                cmd = (45.0 * min(tw / 0.6, 1), 0.0, 0.0)      # walk
            elif tw < 7.2:
                cmd = (0.0, 42.0, 0.0)                          # strafe
            else:
                cmd = (0.0, 0.0, 0.55)                          # turn
            q, _, _ = gait.joint_targets(tw, *cmd)
            data.ctrl[:15] = q.flatten()
        mujoco.mj_step(model, data)
        if k % spf == 0:
            cam.lookat[:] = data.xpos[torso]
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
    ev += [(5.4, "discovery"), (8.4, "resume")]
    return frames, ev


def scene_push_brace():
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    gait = WaveGait()
    data, q0 = make_data(model, gait)
    torso = model.body("torso").id
    fids = [model.geom(f"foot{i}").id for i in range(N_LEGS)]
    sup = ReflexSupervisor(gait, gyro_trip=TRIP, gyro_calm=TRIP / 2)
    renderer, cam = _cam(model, azim=115)
    DT = model.opt.timestep
    spf = int(round(1 / (FPS * DT)))
    frames, ev = [], []
    push_t, F = 3.9, 40.0
    prev = "NORMAL"
    T = 9.5
    for k in range(int(T / DT)):
        t = k * DT
        gyro = gyro_xy_of(model, data, torso)
        vx = 0.0 if t < 1.0 else 45.0 * min((t - 1.0) / 0.6, 1.0)
        con = foot_contacts(model, data, fids)
        R = data.xmat[torso].reshape(3, 3)
        w = R.T @ data.cvel[torso][0:3]
        q, state = sup.step(t, vx, 0, 0, gyro, contacts=con, gyro_vec=w[:2])
        data.ctrl[:15] = q.flatten()
        if state != prev:
            if state == "BRACE":
                ev.append((t + 0.05, "brace"))
            if prev in ("BRACE", "RECOVER") and state == "NORMAL":
                ev.append((t + 0.1, "resume"))
            prev = state
        data.xfrc_applied[torso, :3] = [-F, 0, 0] \
            if push_t <= t < push_t + 0.15 else [0, 0, 0]
        mujoco.mj_step(model, data)
        if k % spf == 0:
            cam.lookat[:] = data.xpos[torso]
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
    return frames, ev


def scene_cliff():
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
    det, react = CliffDetector(), CliffReaction(gait)
    renderer, cam = _cam(model, dist=1.0, elev=-12, azim=150)
    DT = model.opt.timestep
    spf = int(round(1 / (FPS * DT)))
    frames, ev = [], []
    fired_once = False
    T = 12.0
    stop_after = None
    for k in range(int(T / DT)):
        t = k * DT
        if t < 1.0:
            data.ctrl[:15] = q0
        else:
            tw = t - 1.0
            vx, vy, wz = 45.0 * min(tw / 0.6, 1.0), 0.0, 0.0
            vx, vy, wz = react.command(tw, vx, vy, wz)
            q, stance, _ = gait.joint_targets(tw, vx, vy, wz)
            data.ctrl[:15] = q.flatten()
            if k % 10 == 0:
                contact = foot_contacts_odom(model, data)
                ph = [(tw / gait.T + gait.phase_off[i]) % 1.0 for i in range(5)]
                settled = [stance[i] and 0.12 < ph[i] / gait.duty < 0.95
                           for i in range(5)]
                if det.update(tw, settled, contact):
                    react.on_void(tw)
                    if not fired_once:
                        ev += [(t + 0.05, "cliff"), (t + 2.3, "resume"),
                               (t + 4.0, "goal")]
                        fired_once = True
                        stop_after = t + 5.2
        mujoco.mj_step(model, data)
        if k % spf == 0:
            look = data.xpos[torso].copy()
            look[0] = min(look[0] + 0.06, EDGE_X)
            cam.lookat[:] = look
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
        if stop_after and t > stop_after:
            break
    return frames, ev


def main():
    nar = Narrator()
    all_frames = []
    t_off = 0.0
    for name, fn in (("medley", scene_medley),
                     ("push+brace", scene_push_brace),
                     ("cliff", scene_cliff)):
        frames, evs = fn()
        for te, en in evs:
            nar.event(t_off + te, en)
        all_frames += frames
        dur = len(frames) / FPS
        print(f"  scene {name}: {len(frames)} frames ({dur:.1f} s), "
              f"{len(evs)} events", flush=True)
        t_off += dur
    total_s = len(all_frames) / FPS
    for tt, w, cut in nar.schedule():
        print(f"  {tt:5.2f}s  {w}")
    track = nar.render(total_s=total_s)
    wav = os.path.join(HERE, "_show.wav")
    write_wav(wav, track)
    silent = os.path.join(HERE, "_show.mp4")
    imageio.mimsave(silent, all_frames, fps=FPS, codec="libx264", quality=8)
    out = os.path.join(HERE, "pebble_showcase_v07.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", silent,
                    "-i", wav, "-c:v", "copy", "-c:a", "aac", "-b:a", "160k",
                    "-shortest", out], check=True)
    os.remove(silent)
    os.remove(wav)
    print(f"wrote {out} ({total_s:.1f} s)")


if __name__ == "__main__":
    main()
