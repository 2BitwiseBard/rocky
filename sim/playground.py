"""Pebble PLAYGROUND — interactive sim: live viewer + command REPL.

The "drive it before it exists" tool (session 6c). One process runs the
physics with the real gait + real reflex supervisor; you type commands
while it runs. On a laptop with a display you get the native MuJoCo
window; headless (CI, cloud) the same commands run scripted.

    MUJOCO_GL=glfw  python3 playground.py --viewer          # laptop
    python3 playground.py --script "walk 45; wait 3; stop; quit"   # headless
    python3 playground.py --cliff --viewer                  # cliff world

Commands (REPL or --script, ';'-separated):
    walk VX [VY] [WZ]   walk (mm/s, mm/s, rad/s) — adjust any time
    stop                safe-stop: PLANT -> BRACE -> planted idle (D034)
    gesture NAME        jazz_hands | fist_bump | beckon | wave | bow | look_around |
                        shake | sit | turn_in_place | sidestep (from planted idle)
    (with --viewer)     KEYBOARD TELEOP in the window: W/S/A/D nudge velocity,
                        Q/E turn, SPACE = safe-stop, G = wave
    say WORD            chord-speak word (plays the v2 sample if a player
                        exists: aplay/afplay/ffplay; else prints)
    set PARAM VALUE     live-tune: gait.T gait.h gait.R0 gait.duty
                        gait.hstep reflex.trip  (phase-continuous: changing
                        T rescales the clock so feet don't teleport)
    show                current params + robot state
    push FX FY [DUR]    shove the shell rim: peak N, N, half-sine over DUR s (0.4)
    record on|off       capture an offscreen mp4 clip (playground_clip.mp4)
    wait S              (scripts) let S sim-seconds pass
    quit                exit

HONESTY BOX (read before trusting a number): this is the same MJCF the
whole sim stack uses — masses from CAD, servo = position actuator with
guessed kp/kv, friction guessed, no bus latency, no gear backlash. It is
EXCELLENT for logic (does the reflex trip? does the gesture reach? does a
gait tweak break the sweep limits?) and DIRECTIONAL for dynamics (push
envelopes, stability trends). It is NOT calibrated — that happens when a
real servo answers back (bench day: step response, stall, real masses ->
MJCF update). Treat absolute numbers as ±real-robot-TBD.
"""
import argparse
import os
import queue
import shutil
import subprocess
import sys
import threading
import time

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS        # noqa: E402
from pebble_reflex import ReflexSupervisor, body_gyro_xy             # noqa: E402
from pebble_gestures import (jazz_hands, fist_bump, beckon,          # noqa: E402
                             JAZZ_TOTAL, BUMP_TOTAL, BECKON_TOTAL)
from run_odom import foot_contacts                                    # noqa: E402
from shove import Shove                                               # noqa: E402

from pebble_gestures2 import GESTURES2                              # noqa: E402
GESTURES = {"jazz_hands": (jazz_hands, JAZZ_TOTAL),
            "fist_bump": (fist_bump, BUMP_TOTAL),
            "beckon": (beckon, BECKON_TOTAL)}
GESTURES.update({k: (fn, total) for k, (fn, total, _ev) in GESTURES2.items()})  # session 8: v2 library
CHORD_DIR = os.path.join(HERE, "..", "audio", "samples_v2")


def find_player():
    for p in ("aplay", "afplay", "ffplay"):
        if shutil.which(p):
            return p
    return None


class Playground:
    def __init__(self, cliff=False):
        if cliff:
            from run_cliff import build_world, PLAT_H
            self.model = build_world()
            z0 = PLAT_H
        else:
            with open(os.path.join(HERE, "pebble.xml")) as f:
                self.model = mujoco.MjModel.from_xml_string(f.read())
            z0 = 0.0
        self.gait = WaveGait()
        self.sup = ReflexSupervisor(self.gait)
        self.data = mujoco.MjData(self.model)
        q0 = np.array([leg_ik(body_to_leg(i, self.gait.p_nom[i]))
                       for i in range(N_LEGS)]).flatten()
        jadr = [self.model.joint(f"{n}{i}").qposadr[0]
                for i in range(5) for n in ("yaw", "hip", "knee")]
        self.data.qpos[0:3] = [0, 0, (self.gait.h + 14) / 1000.0 + z0]
        self.data.qpos[3:7] = [1, 0, 0, 0]
        self.data.qpos[jadr] = q0
        self.data.ctrl[:15] = q0
        mujoco.mj_forward(self.model, self.data)
        self.torso = self.model.body("torso").id
        self.DT = self.model.opt.timestep
        self.lock = threading.Lock()
        self.cmd_v = np.zeros(3)              # vx, vy, wz
        self.gesture = None                   # (fn, total, t0)
        self.push = None                      # shove.Shove or None
        self.righter_note = self._install_righter()
        self.t = 0.0
        self.player = find_player()
        self.recording = False
        self._frames = []
        self._renderer = None
        self._spf = int(round(1 / (30 * self.DT)))
        self._k = 0
        self.alive = True

    def _install_righter(self):
        """D048: the learned righter rides along when torch + a checkpoint
        are available, so a tip-over in the playground ends in a recovery
        instead of an upside-down standing pose. Optional: without torch the
        supervisor still declares FALLEN and uses its deadline ramp."""
        try:
            from righter import PolicyRighter, default_ckpt
            ckpt = default_ckpt()
            if ckpt is None:
                return "no righter (no checkpoint in sim/runs/)"
            fids = [self.model.geom(f"foot{i}").id for i in range(N_LEGS)]
            self.sup.set_righter(PolicyRighter(ckpt, self.model, self.data, self.torso, fids))
            return f"righter: {os.path.relpath(ckpt, HERE)}"
        except Exception as e:                       # torch missing, bad ckpt
            return f"no righter ({type(e).__name__}: {e}) — deadline ramp only"

    # ------------------------------------------------------------ physics
    def step(self):
        """One physics step; call from the run loop."""
        with self.lock:
            v = self.cmd_v.copy()
            ges = self.gesture
            push = self.push
        R = self.data.xmat[self.torso].reshape(3, 3)
        w_body = R.T @ self.data.cvel[self.torso][0:3]
        gxy = body_gyro_xy(w_body)
        con = foot_contacts(self.model, self.data)
        tilt_deg = float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
        height = float(self.data.xpos[self.torso][2])
        if ges is not None:
            fn, total, t0 = ges
            tg = self.t - t0
            if tg >= total:
                with self.lock:
                    self.gesture = None
            else:
                q, claw = fn(self.gait, tg)
                self.data.ctrl[:15] = q.flatten()
                if self.model.nu >= 20:
                    self.data.ctrl[15:20] = claw
        if self.gesture is None:
            q, state = self.sup.step(self.t, v[0], v[1], v[2], gxy,
                                     contacts=con, gyro_vec=w_body[:2],
                                     tilt_deg=tilt_deg, height=height)
            self.data.ctrl[:15] = q.flatten()
        if push is not None:
            if not push.apply(self.model, self.data, self.torso, self.t):
                with self.lock:               # over: wrench already zeroed
                    self.push = None
        mujoco.mj_step(self.model, self.data)
        self.t += self.DT
        self._k += 1
        if self.recording and self._k % self._spf == 0:
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, 480, 720)
                self._cam = mujoco.MjvCamera()
                self._cam.distance, self._cam.elevation, self._cam.azimuth = \
                    0.95, -16, 55
            self._cam.lookat[:] = self.data.xpos[self.torso]
            self._renderer.update_scene(self.data, self._cam)
            self._frames.append(self._renderer.render())

    # ------------------------------------------------------------ teleop
    # Keyboard drive (session 8c): the MuJoCo viewer delivers key PRESSES
    # (no releases), so teleop is incremental: taps nudge the velocity
    # command, space is the D034 safe-stop. Works alongside the REPL.
    TELEOP_KEYS = {
        87: ("vx", +15.0), 265: ("vx", +15.0),   # W / Up
        83: ("vx", -15.0), 264: ("vx", -15.0),   # S / Down
        65: ("vy", +15.0), 263: ("vy", +15.0),   # A / Left  (+y = leftward)
        68: ("vy", -15.0), 262: ("vy", -15.0),   # D / Right
        81: ("wz", +0.12),                        # Q  turn left
        69: ("wz", -0.12),                        # E  turn right
    }
    V_MAX = np.array([60.0, 60.0, 0.5])

    def on_key(self, keycode):
        if keycode == 32:                          # SPACE: stop everything
            with self.lock:
                self.cmd_v[:] = 0
            self.sup.request_stop()
            print("\n[teleop] SAFE-STOP", flush=True)
            return
        if keycode in (71,):                       # G: quick wave hello
            if "wave" in GESTURES and np.abs(self.cmd_v).sum() == 0:
                fn, total = GESTURES["wave"]
                with self.lock:
                    self.gesture = (fn, total, self.t)
                print("\n[teleop] wave", flush=True)
            return
        hit = self.TELEOP_KEYS.get(int(keycode))
        if hit is None:
            return
        axis, dv = hit
        i = {"vx": 0, "vy": 1, "wz": 2}[axis]
        with self.lock:
            self.cmd_v[i] = float(np.clip(self.cmd_v[i] + dv,
                                          -self.V_MAX[i], self.V_MAX[i]))
            v = self.cmd_v.copy()
        print(f"\n[teleop] v=({v[0]:.0f}, {v[1]:.0f}) mm/s  wz={v[2]:.2f}",
              flush=True)

    # ----------------------------------------------------------- commands
    def do(self, line):
        """Execute one command; returns a reply string ('' quits)."""
        parts = line.strip().split()
        if not parts:
            return None
        c, args = parts[0].lower(), parts[1:]
        if c == "quit":
            self.alive = False
            return ""
        if c == "walk":
            v = [float(a) for a in args[:3]] + [0.0] * (3 - len(args[:3]))
            with self.lock:
                self.cmd_v[:] = v
            return f"walking vx={v[0]} vy={v[1]} wz={v[2]}"
        if c == "stop":
            with self.lock:
                self.cmd_v[:] = 0
            self.sup.request_stop()
            return "safe-stop requested (PLANT->BRACE)"
        if c == "gesture":
            if not args or args[0] not in GESTURES:
                return f"gestures: {', '.join(GESTURES)}"
            if np.abs(self.cmd_v).sum() > 0:
                return "busy walking — stop first (the MCP server enforces "\
                       "the same rule)"
            fn, total = GESTURES[args[0]]
            with self.lock:
                self.gesture = (fn, total, self.t)
            return f"{args[0]} ({total:.1f} s)"
        if c == "say":
            w = args[0] if args else ""
            wav = os.path.join(CHORD_DIR, f"{w}.wav")
            if not os.path.exists(wav):
                have = sorted(x[:-4] for x in os.listdir(CHORD_DIR)
                              if x.endswith(".wav"))
                return f"unknown word; lexicon: {', '.join(have)}"
            if self.player:
                cmd = {"ffplay": ["ffplay", "-nodisp", "-autoexit",
                                  "-loglevel", "quiet", wav]}.get(
                    self.player, [self.player, wav])
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                return f"*{w}* (chord)"
            return f"*{w}* (no audio player found — install aplay/ffplay)"
        if c == "set":
            if len(args) != 2:
                return "set PARAM VALUE — params: gait.T gait.h gait.R0 "\
                       "gait.duty gait.hstep reflex.trip"
            p, val = args[0], float(args[1])
            g = self.gait
            with self.lock:
                if p == "gait.T":
                    ph = (self.sup.t_gait / g.T) % 1.0    # phase continuity
                    g.T = val
                    self.sup.t_gait = ph * val
                elif p == "gait.duty":
                    g.duty = val
                elif p == "gait.hstep":
                    g.hstep = val
                elif p in ("gait.h", "gait.R0"):
                    setattr(g, p.split(".")[1], val)
                    a = np.deg2rad([90 + 72 * i for i in range(N_LEGS)])
                    g.p_nom = np.stack([g.R0 * np.cos(a), g.R0 * np.sin(a),
                                        -g.h * np.ones(N_LEGS)], axis=1)
                elif p == "reflex.trip":
                    self.sup.gyro_trip = val
                else:
                    return f"unknown param {p}"
            return f"{p} = {val}  (sim-only until params.yaml + regen — "\
                   "note keepers in NOTES_INBOX)"
        if c == "show":
            g = self.gait
            p = self.data.xpos[self.torso]
            return (f"gait: T={g.T} h={g.h} R0={g.R0} duty={g.duty} "
                    f"hstep={g.hstep} | reflex: trip={self.sup.gyro_trip} "
                    f"state={self.sup.state} | pose=({p[0]*1000:.0f}, "
                    f"{p[1]*1000:.0f}) mm  t={self.t:.1f} s")
        if c == "push":
            fx, fy = float(args[0]), float(args[1])
            dur = float(args[2]) if len(args) > 2 else 0.4
            sh = Shove(fx, fy, dur=dur, t0=self.t)
            with self.lock:
                self.push = sh
            return f"shoving: {sh.describe(self.model)} — watch the reflex"
        if c == "record":
            if args and args[0] == "on":
                self.recording = True
                self._frames = []
                return "recording..."
            self.recording = False
            if self._frames:
                import imageio
                out = os.path.join(HERE, "playground_clip.mp4")
                imageio.mimsave(out, self._frames, fps=30,
                                codec="libx264", quality=8)
                n = len(self._frames)
                self._frames = []
                return f"wrote playground_clip.mp4 ({n} frames)"
            return "nothing recorded"
        return f"unknown command {c!r} — walk/stop/gesture/say/set/show/" \
               "push/record/wait/quit"


def run_headless(pg, script):
    for raw in script.split(";"):
        cmd = raw.strip()
        if not cmd:
            continue
        if cmd.startswith("wait"):
            secs = float(cmd.split()[1])
            for _ in range(int(secs / pg.DT)):
                pg.step()
            print(f"[{pg.t:6.2f}s] (waited {secs} s)")
            continue
        r = pg.do(cmd)
        print(f"[{pg.t:6.2f}s] > {cmd}\n          {r}")
        if not pg.alive:
            break
    # NaN audit — the playground must never emit one (reflex guarantees)
    assert not np.isnan(pg.data.qpos).any(), "NaN in qpos — report this"
    print("clean exit, no NaNs")


def run_interactive(pg, use_viewer):
    q = queue.Queue()

    def reader():
        while pg.alive:
            try:
                q.put(input("pebble> "))
            except EOFError:
                q.put("quit")
                return

    threading.Thread(target=reader, daemon=True).start()
    viewer_ctx = None
    if use_viewer:
        import mujoco.viewer
        viewer_ctx = mujoco.viewer.launch_passive(pg.model, pg.data,
                                                  key_callback=pg.on_key)
        print("teleop: W/S fwd-back  A/D strafe  Q/E turn  SPACE safe-stop  "
              "G wave  (taps nudge the command; REPL still works)")
    t_wall = time.monotonic()
    while pg.alive:
        pg.step()
        if viewer_ctx is not None:
            viewer_ctx.sync()
            if not viewer_ctx.is_running():
                break
        # real-time pacing
        t_wall += pg.DT
        lag = t_wall - time.monotonic()
        if lag > 0:
            time.sleep(lag)
        elif lag < -0.05:
            # D048: fell >50 ms behind (a render stall, a print, a key
            # burst): resync instead of fast-forwarding to catch up — the
            # catch-up burst looked like the robot teleporting/jittering
            t_wall = time.monotonic()
        try:
            r = pg.do(q.get_nowait())
            if r:
                print(r)
        except queue.Empty:
            pass
    if viewer_ctx is not None:
        viewer_ctx.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--viewer", action="store_true",
                    help="open the native MuJoCo window (needs a display; "
                         "use MUJOCO_GL=glfw on a laptop)")
    ap.add_argument("--cliff", action="store_true",
                    help="load the table-edge world (test the VOID reflex)")
    ap.add_argument("--script", default=None,
                    help="';'-separated commands, run headless as fast as "
                         "the CPU allows")
    args = ap.parse_args()
    pg = Playground(cliff=args.cliff)
    print(pg.righter_note)
    if args.script:
        run_headless(pg, args.script)
    else:
        print(__doc__.split("Commands")[1].split("HONESTY")[0])
        run_interactive(pg, args.viewer)
