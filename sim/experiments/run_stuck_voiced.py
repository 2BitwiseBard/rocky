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

Usage: MUJOCO_GL=egl .venv/bin/python sim/experiments/run_stuck_voiced.py [seed]
Output: sim/experiments/out/pebble_stuck_retry_40mm_voiced.mp4

The default rubble seed is 1: on the budgeted retry ladder (D060) seed 0 no
longer crosses 40 mm (run_stuck.py: the watchdog crosses 2/4 seeds there),
while seed 1 crosses at ~21 s — the video needs a crossing to narrate.
"""
import os
import subprocess
import sys

import imageio
import mujoco

import exp_paths as X                   # sys.path (sim/, gait/, perception/, audio/) + where results / clips go
from pebble_gait import WaveGait                                     # noqa: E402
from pebble_watchdog import ProgressWatchdog, RetryPolicy            # noqa: E402
from scenes import build_model, init_robot, walk_ask                 # noqa: E402
from chordspeak_events import Narrator                               # noqa: E402
from chordspeak2 import write_wav                                # noqa: E402

AMP = 40
V_X = walk_ask(WaveGait())[0]   # mm/s: scenes.V_X fitted into the envelope (D063: 45 asks 34.2)
#                                 T_WALK_MAX was sized at 45: on the D063 gait seed 1 does not cross within
#                                 it (nor at a raw 45: 394 mm); run_stuck at 33 s (25 x 45 / 34.2) crosses it
#                                 at 31.5 s. No crossing to narrate until the owner re-times it (run_stuck.py)
T_STAND = 1.0
T_WALK_MAX = 25.0
GOAL_X = 0.40
FPS = 30


def main(seed=1):
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
    wav_path = X.out("_stuck_narration.wav")
    write_wav(wav_path, track)
    silent = X.out("_stuck_silent.mp4")
    imageio.mimsave(silent, frames, fps=FPS, codec="libx264", quality=8)
    out = X.out(f"pebble_stuck_retry_{AMP}mm_voiced.mp4")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", silent,
                    "-i", wav_path, "-c:v", "copy", "-c:a", "aac",
                    "-b:a", "160k", "-shortest", out], check=True)
    os.remove(silent)
    os.remove(wav_path)
    print("wrote", out)


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 1)
