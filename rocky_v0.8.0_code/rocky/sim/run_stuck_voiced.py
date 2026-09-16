"""The stuck-retry video, narrated — Rocky tells you what he's doing.

Re-runs the 40 mm rubble crossing (run_stuck.py harness, watchdog ON) and
renders it WITH its chord-speak v0.2 soundtrack, driven by the actual
runtime events through audio/chordspeak_events.Narrator:

    walk starts        -> acknowledge      ("mm-hm")
    progress stalls    -> confused         (STUCK fires)
    retry engages      -> thinking         (stage 1: high-step)
    retry escalates    -> determined       (stage 2+: higher+clear/reverse)
    progress resumes   -> acknowledge
    goal line crossed  -> found_it
    field conquered    -> amaze            (video ends on the canon word)

The robot narrating itself — no post-hoc script, the events come from the
same watchdog state machine that runs the recovery.

Usage: MUJOCO_GL=osmesa python3 run_stuck_voiced.py [seed]
Output: pebble_stuck_retry_40mm_voiced.mp4
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
from pebble_gait import WaveGait                                     # noqa: E402
from pebble_watchdog import ProgressWatchdog, RetryPolicy            # noqa: E402
from run_terrain import build_model, init_robot                      # noqa: E402
from chordspeak_events import Narrator                               # noqa: E402
from chordspeak2 import SR, write_wav                                # noqa: E402

AMP, V_X = 40, 45.0
T_STAND = 1.0
T_WALK_MAX = 25.0
GOAL_X = 0.40
FPS = 30


def main(seed=0):
    model = build_model(AMP, seed)
    gait = WaveGait()
    data, q0 = init_robot(model, gait, extra_z=AMP / 1000.0)
    torso = model.body("torso").id
    DT = model.opt.timestep
    dog = ProgressWatchdog(gait.T)
    policy = RetryPolicy(gait)
    nar = Narrator()
    renderer = mujoco.Renderer(model, 480, 720)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.9, -16, 150

    frames = []
    spf = int(round(1 / (FPS * DT)))
    t_cross = None
    t_end = T_STAND + T_WALK_MAX
    n_events_seen = 0
    said_walk = False
    stage_seen = 0
    for k in range(int((T_STAND + T_WALK_MAX) / DT)):
        t = k * DT
        if t > t_end:
            break
        if t < T_STAND:
            data.ctrl[:15] = q0
        else:
            tw = t - T_STAND
            ramp = min(tw / 0.6, 1.0)
            vx, vy, wz = V_X * ramp, 0.0, 0.0
            if not said_walk and ramp >= 1.0:
                nar.event(t, "walk_start")
                said_walk = True
            vx, vy, wz = policy.command(tw, vx, vy, wz)
            odom = data.xpos[torso][:2] * 1000.0
            if dog.update(tw, odom, abs(vx)):
                policy.on_stuck(tw)
                dog.reset()
            policy.maybe_relax(tw, dog.last_ratio > 0.6)
            # translate watchdog events -> narration (until the goal:
            # once "crossing" fires the video is ending on found_it/amaze —
            # a fresh snag two steps past the finish line narrates as
            # celebration-then-grumble, which reads as a glitch)
            while t_cross is None and n_events_seen < len(policy.events):
                te, label = policy.events[n_events_seen]
                tv = te + T_STAND
                if label.startswith("STUCK"):
                    if policy.stage >= 2 or stage_seen >= 1:
                        nar.event(tv, "retry_escalate")
                    else:
                        nar.event(tv, "stuck")
                        nar.event(tv + 1.6, "retry")
                    stage_seen += 1
                elif label.startswith("recovered"):
                    nar.event(tv, "recovered")
                n_events_seen += 1
            q, _, _ = gait.joint_targets(tw, vx, vy, wz)
            data.ctrl[:15] = q.flatten()
        mujoco.mj_step(model, data)
        if t_cross is None and data.xpos[torso][0] > GOAL_X:
            t_cross = t
            nar.event(t, "crossing")
            nar.event(t + 1.6, "goal")
            t_end = min(t + 4.0, T_STAND + T_WALK_MAX)
        if k % spf == 0:
            cam.lookat[:] = data.xpos[torso]
            renderer.update_scene(data, cam)
            frames.append(renderer.render())

    total_s = len(frames) / FPS
    print(f"{len(frames)} frames ({total_s:.1f} s), crossed at "
          f"{t_cross and round(t_cross, 1)} s, "
          f"{len(policy.events)} watchdog events")
    for tt, wname, cut in nar.schedule():
        print(f"  {tt:5.2f}s  {wname}")

    track = nar.render(total_s=total_s)
    wav_path = os.path.join(HERE, "_stuck_narration.wav")
    write_wav(wav_path, track)
    silent = os.path.join(HERE, "_stuck_silent.mp4")
    imageio.mimsave(silent, frames, fps=FPS, codec="libx264", quality=8)
    out = os.path.join(HERE, f"pebble_stuck_retry_{AMP}mm_voiced.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", silent,
                    "-i", wav_path, "-c:v", "copy", "-c:a", "aac",
                    "-b:a", "160k", "-shortest", out], check=True)
    os.remove(silent)
    os.remove(wav_path)
    print("wrote", out)


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 0)
