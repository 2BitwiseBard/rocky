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
    TELEOP (keys)       in the TERMINAL, at an empty prompt: arrow keys or
                        Shift+W/S/A/D nudge velocity, Shift+Q/E turn,
                        SPACE = safe-stop, Shift+G = wave. In the viewer
                        window only the ARROW keys drive — every letter
                        there is one of MuJoCo's own render toggles
                        (W wireframe, S shadows, A auto-connect, D static
                        bodies, G fog, Q camera, E equality) and SPACE
                        pauses the viewer: use the terminal (D048).
    say WORD            chord-speak word (plays the v2 sample if a player
                        exists: aplay/afplay/ffplay; else prints)
    set PARAM VALUE     live-tune: gait.T gait.h gait.R0 gait.duty
                        gait.hstep reflex.trip reflex.stall_s
                        reflex.fallen_max_s  (phase-continuous: changing
                        T rescales the clock so feet don't teleport)
    show                current params + robot state
    push FX FY [DUR]    shove the shell rim: peak N, N, half-sine over DUR s (0.4)
    record on|off       capture an offscreen mp4 clip (playground_clip.mp4)
    help [rl]           this list (help rl: the RL hooks below)
    rl                  the RL runs table: every checkpoint, its reward
                        version, rate limit, steps, last return, eval note
    righter NAME|off    hot-swap the self-righting policy (runs/NAME) or
                        run without one (stall/deadline ramp only)
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
    def __init__(self, cliff=False, model=None, z0=0.0):
        if model is not None:                       # D049: the cockpit's world builder
            self.model = model
        elif cliff:
            from run_cliff import build_world, PLAT_H
            self.model = build_world()
            z0 = PLAT_H
        else:
            with open(os.path.join(HERE, "pebble.xml")) as f:
                self.model = mujoco.MjModel.from_xml_string(f.read())
            z0 = 0.0
        self.z0 = z0
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

    def _install_righter(self, ckpt=None):
        """D048: the learned righter rides along when torch + a checkpoint
        are available, so a tip-over in the playground ends in a recovery
        instead of an upside-down standing pose. Optional: without torch the
        supervisor still declares FALLEN and uses its stall/deadline ramp."""
        try:
            from righter import PolicyRighter, default_ckpt
            ckpt = ckpt or default_ckpt()
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
        self.last = dict(con=con, tilt=tilt_deg, height=height, gxy=gxy)   # for the cockpit's feeds
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

    # ------------------------------------------------------------ HUD
    STATE_RGBA = {"NORMAL": (0.2, 0.85, 0.3, 0.9), "PLANT": (0.95, 0.8, 0.2, 0.9),
                  "BRACE": (0.95, 0.55, 0.1, 0.9), "RECOVER": (0.3, 0.7, 0.95, 0.9),
                  "FALLEN": (0.9, 0.15, 0.15, 0.9), "RIGHTED": (0.6, 0.3, 0.9, 0.9)}

    def draw_hud(self, scn):
        """D048: an in-window HUD in the viewer's user scene — a marker
        above the torso coloured by the reflex state and an arrow for the
        velocity command (length = 2 s of travel; a curved tick for turn).
        scn: viewer.user_scn (or any MjvScene with spare geoms)."""
        scn.ngeom = 0
        p = self.data.xpos[self.torso]
        R = self.data.xmat[self.torso].reshape(3, 3)
        rgba = self.STATE_RGBA.get(self.sup.state, (1, 1, 1, 0.9))
        top = p + np.array([0, 0, 0.13])
        if scn.ngeom < scn.maxgeom:
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.012, 0, 0]),
                                top, np.eye(3).flatten(), np.array(rgba, dtype=np.float32))
            scn.ngeom += 1
        with self.lock:
            v = self.cmd_v.copy()
        if (abs(v[0]) + abs(v[1])) > 0 and scn.ngeom < scn.maxgeom:
            d = R @ np.array([v[0], v[1], 0.0]) / 1000.0 * 2.0
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                                np.eye(3).flatten(), np.array(rgba, dtype=np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.006, top, top + d)
            scn.ngeom += 1
        if abs(v[2]) > 0 and scn.ngeom < scn.maxgeom:
            a = np.sign(v[2]) * min(abs(v[2]) / 0.5, 1.0) * np.pi / 2
            r0 = R @ np.array([0.0, 0.10, 0.0])
            r1 = R @ np.array([-0.10 * np.sin(a), 0.10 * np.cos(a), 0.0])
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                                np.eye(3).flatten(), np.array(rgba, dtype=np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.004, top + r0, top + r1)
            scn.ngeom += 1

    # ------------------------------------------------------------ teleop
    # Keyboard drive (session 8c, reworked D048): taps nudge the velocity
    # command (no key-release events anywhere, so teleop is incremental).
    # Keys arrive from two places: the TERMINAL reader (single keys at an
    # empty prompt: arrows, Shift+WASD/QE, SPACE, Shift+G) and the viewer
    # window, where ONLY the arrow keys are ours — the MuJoCo viewer binds
    # every letter to a render toggle and SPACE to pause, and its bindings
    # fire alongside key_callback, which is what "W changes the lighting"
    # was (W = wireframe, S = shadows, A = auto-connect, D = static bodies).
    TELEOP = {
        "up": ("vx", +15.0), "down": ("vx", -15.0),
        "left": ("vy", +15.0), "right": ("vy", -15.0),    # +y = leftward
        "W": ("vx", +15.0), "S": ("vx", -15.0),
        "A": ("vy", +15.0), "D": ("vy", -15.0),
        "Q": ("wz", +0.12), "E": ("wz", -0.12),
    }
    GLFW_ARROWS = {265: "up", 264: "down", 263: "left", 262: "right"}
    V_MAX = np.array([60.0, 60.0, 0.5])

    def teleop(self, key):
        """key: 'up'/'down'/'left'/'right', 'W'.. 'E', ' ' (stop), 'G' (wave).
        Returns a one-line status or None if the key is not a teleop key."""
        if key == " ":                             # SPACE: stop everything
            with self.lock:
                self.cmd_v[:] = 0
            self.sup.request_stop()
            return "[teleop] SAFE-STOP"
        if key == "G":                             # quick wave hello
            if "wave" in GESTURES and np.abs(self.cmd_v).sum() == 0:
                fn, total = GESTURES["wave"]
                with self.lock:
                    self.gesture = (fn, total, self.t)
                return "[teleop] wave"
            return "[teleop] wave needs a standstill (SPACE first)"
        hit = self.TELEOP.get(key)
        if hit is None:
            return None
        axis, dv = hit
        i = {"vx": 0, "vy": 1, "wz": 2}[axis]
        with self.lock:
            self.cmd_v[i] = float(np.clip(self.cmd_v[i] + dv,
                                          -self.V_MAX[i], self.V_MAX[i]))
            v = self.cmd_v.copy()
        return f"[teleop] v=({v[0]:.0f}, {v[1]:.0f}) mm/s  wz={v[2]:.2f}"

    def on_key(self, keycode):
        """Viewer window key_callback: arrows only (see TELEOP note)."""
        name = self.GLFW_ARROWS.get(int(keycode))
        if name is not None:
            r = self.teleop(name)
            if r:
                print("\n" + r, flush=True)

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
        if c in ("help", "?"):
            if args and args[0] == "rl":
                return HELP_RL
            return __doc__.split("Commands")[1].split("HONESTY")[0].rstrip() + "\n" + TELEOP_HELP
        if c == "rl":
            from rl_dashboard import summarize_runs, table
            return table(summarize_runs()) + "\n(`righter NAME` loads one; " \
                   "python rl_dashboard.py draws the curves)"
        if c == "righter":
            if not args:
                return f"righter: {self.righter_note}  (righter NAME | off)"
            if args[0] == "off":
                self.sup.set_righter(None)
                self.righter_note = "no righter (stall/deadline ramp only)"
                return self.righter_note
            path = os.path.join(HERE, "runs", args[0], "latest.pt")
            if not os.path.exists(path):
                have = sorted(d for d in os.listdir(os.path.join(HERE, "runs"))
                              if os.path.exists(os.path.join(HERE, "runs", d, "latest.pt")))
                return f"no runs/{args[0]}/latest.pt — have: {', '.join(have)}"
            self.righter_note = self._install_righter(path)
            return self.righter_note
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
                       "gait.duty gait.hstep reflex.trip reflex.stall_s reflex.fallen_max_s"
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
                elif p == "reflex.stall_s":
                    self.sup.stall_s = val
                elif p == "reflex.fallen_max_s":
                    self.sup.fallen_max_s = val
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


HELP_RL = """RL hooks in the playground (D048):
  rl                  table of runs/ checkpoints (reward version, rate limit, steps,
                      last return/length/entropy, recorded eval)
  righter NAME        hot-swap the self-righting policy, e.g. righter recover5_v3_warm
  righter off         no policy: FALLEN -> stall/deadline ramp only (the analytic path)
  push 40 0           tip it over and watch FALLEN -> RIGHTED -> NORMAL with that righter
  set reflex.stall_s 2     ramp sooner/later when the righter makes no progress
  set reflex.fallen_max_s 10   the hard deadline
  HUD marker above the torso: green NORMAL, yellow PLANT, orange BRACE, blue RECOVER,
  red FALLEN, purple RIGHTED; the arrow is the velocity command.
Outside: python rl_dashboard.py (curves), audit_righter.py CKPT (jitter numbers),
eval_recover.py CKPT (20-fall eval), docs/RL_GUIDE.md (train/eval/resume)."""

TELEOP_HELP = ("teleop (terminal, empty prompt): arrows or Shift+W/S/A/D drive, "
               "Shift+Q/E turn, SPACE safe-stop, Shift+G wave; type commands as usual. "
               "Viewer window: arrows only (letters there are MuJoCo's render toggles).")


def _terminal_reader(pg, q):
    """Raw-key terminal reader: single teleop keys act immediately when the
    line is empty; anything else is line-edited (backspace, Enter) into a
    REPL command. Falls back to input() when stdin is not a tty."""
    if not sys.stdin.isatty():
        while pg.alive:
            try:
                q.put(input("pebble> "))
            except EOFError:
                q.put("quit")
                return
        return
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    out = sys.stdout
    buf = ""

    def prompt():
        out.write("pebble> " + buf)
        out.flush()

    prompt()
    try:
        while pg.alive:
            ch = sys.stdin.read(1)
            if not ch:
                q.put("quit")
                return
            if ch == "\x1b":                          # escape sequence: arrows
                seq = sys.stdin.read(2)
                name = {"[A": "up", "[B": "down", "[D": "left", "[C": "right"}.get(seq)
                if name and not buf:
                    r = pg.teleop(name)
                    out.write("\r\033[K" + (r or "") + "\n")
                    prompt()
                continue
            if not buf and (ch in pg.TELEOP or ch in (" ", "G")):
                r = pg.teleop(ch)
                out.write("\r\033[K" + (r or "") + "\n")
                prompt()
                continue
            if ch in ("\n", "\r"):
                out.write("\n")
                q.put(buf)
                buf = ""
                prompt()
            elif ch in ("\x7f", "\b"):
                if buf:
                    buf = buf[:-1]
                    out.write("\b \b")
                    out.flush()
            elif ch in ("\x03", "\x04"):               # Ctrl-C / Ctrl-D
                out.write("\n")
                q.put("quit")
                return
            else:
                buf += ch
                out.write(ch)
                out.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def run_interactive(pg, use_viewer):
    q = queue.Queue()
    threading.Thread(target=_terminal_reader, args=(pg, q), daemon=True).start()
    viewer_ctx = None
    if use_viewer:
        import mujoco.viewer
        viewer_ctx = mujoco.viewer.launch_passive(pg.model, pg.data,
                                                  key_callback=pg.on_key)
    print(TELEOP_HELP)
    print("type `help` for the command list, `help rl` for the RL hooks", flush=True)
    t_wall = time.monotonic()
    while pg.alive:
        pg.step()
        if viewer_ctx is not None:
            if pg._k % 8 == 0:                    # HUD at ~60 Hz is plenty
                with viewer_ctx.lock():
                    pg.draw_hud(viewer_ctx.user_scn)
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
                print("\r\033[K" + r + "\npebble> ", end="", flush=True)
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
