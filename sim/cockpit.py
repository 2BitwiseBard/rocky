"""Pebble COCKPIT (D049) — the browser playground: live cameras, teleop,
console, a chat with a switchable brain (regex / local LLM / Claude), the
robot's eye + a vision model, a world editor (obstacles, terrain, friction,
slopes) and the RL panel (checkpoint table, righter hot-swap, tuning),
all on ONE continuously running MuJoCo sim.

    ./rocky.sh cockpit                # http://127.0.0.1:8765
    MUJOCO_GL=egl python sim/cockpit.py --port 8765 --world "obstacle course"

Architecture: one sim thread owns physics (the Playground step loop with a
goto controller + cliff detector folded in, real-time paced, offscreen
rendering of a chase camera and the eye camera to JPEG). Everything else
talks to it through a job queue (`call`), so the HTTP layer, the brains
and the MCP proxy (harness/cockpit_backend.py — `./rocky.sh chat` drives
THIS sim while the cockpit is up) never touch MjData from another thread.
Guard supremacy is unchanged: goto runs the reflex supervisor and the
cliff detector exactly as harness/sim_backend.py does; a veto comes back
as stopped='cliff'.

Honesty: same MJCF, same guessed servo gains and friction as every other
sim entry point (see playground.py's box). The vision tool sends the eye
camera's JPEG to a local vision model; its words are the model's, not a
sensor's.
"""
from __future__ import annotations
import argparse
import asyncio
import base64
import io
import json
import os
import queue
import sys
import threading
import time
from collections import deque

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np                                                    # noqa: E402
import mujoco                                                         # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for sub in ("gait", "perception", "sim"):
    sys.path.insert(0, os.path.join(ROOT, sub))
sys.path.insert(0, ROOT)

from playground import (Playground, TELEOP_HELP, HELP_RL, PROBE_MAX,          # noqa: E402
                        all_gestures, lexicon, chord_wav, CHORD_CUSTOM_DIR)
from pebble_keyframes import (KeyframeGesture, save_keyframe_gesture,           # noqa: E402
                              delete_keyframe_gesture, load_keyframe_gestures, GESTURE_DIR)
from hw_bridge import HardwareBridge, serial_ports, MIRRORS                     # noqa: E402
from world_builder import (build as build_world, PRESETS, KINDS, random_course,   # noqa: E402
                           saved_worlds, save_world, load_world)
from pebble_reflex import ReflexSupervisor                             # noqa: E402
from cliff import CliffDetector, CliffReaction                         # noqa: E402
from sim_lidar import scan as lidar_scan, RANGE_MAX                    # noqa: E402
from shove import Shove                                                # noqa: E402
from harness.backend import CHORD_WORDS                                # noqa: E402
from harness.intent import plan as intent_plan, execute as intent_execute   # noqa: E402
from harness.local_brain import TOOLS as BRAIN_TOOLS, SYSTEM as BRAIN_SYSTEM, OpenAIChat   # noqa: E402

V_GOTO = 45.0
GOTO_CAP_S = 40.0            # a goto that has not ended by then ends as "timeout"
GOTO_STUCK_S = 6.0           # no 2 cm of progress toward the target for this long -> "stuck" (obstacle)
LOOK_TOOL = {"type": "function", "function": {
    "name": "look",
    "description": "Look through the robot's eye camera: a vision model describes what "
                   "is in front of the robot (obstacles, open space, objects).",
    "parameters": {"type": "object", "properties": {}}}}
TOOLS = BRAIN_TOOLS + [LOOK_TOOL]
AUDIO_DIR = os.path.join(ROOT, "audio")
CHORD_SPEC_DIR = os.path.join(AUDIO_DIR, "custom")          # D051: chord words designed in the cockpit
SYSTEM = BRAIN_SYSTEM + ("\nYou also have `look`: the robot's eye camera described by a vision "
                         "model. Use it when asked what you see, before walking toward "
                         "something, or when a goto was vetoed.")
VISION_PROMPT = ("You are the eye of a small five-legged robot walking on a floor. Describe "
                 "what is in front of it in two short sentences: obstacles or objects, roughly "
                 "how far (near = within a few body lengths), and where the open floor is. "
                 "If the view is mostly floor, say so.")


def _local_ai_key():
    for k in ("ROCKY_LLM_API_KEY", "LOCAL_AI_KEY"):
        if os.environ.get(k):
            return os.environ[k]
    conf = os.path.expanduser("~/.config/environment.d/local-ai.conf")
    if os.path.exists(conf):
        for line in open(conf):
            if line.startswith("LOCAL_AI_KEY="):
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    return "none"


class CockpitSim(Playground):
    """The Playground with a goto controller, world swapping, offscreen
    cameras and an async job queue; every method that touches MjData
    runs in the sim thread (via `call`) unless noted."""

    def __init__(self, world="flat"):
        spec = dict(PRESETS.get(world, PRESETS["flat"])) if isinstance(world, str) else dict(world)
        model, z0 = build_world(spec)
        super().__init__(model=model, z0=z0)
        self.world_name = world if isinstance(world, str) else "custom"
        self.world_spec = spec
        self.fids = [self.model.geom(f"foot{i}").id for i in range(5)]
        self.pending = queue.Queue()
        self.speed = 1.0
        self.paused = False
        self.mode = "idle"
        self.events = deque(maxlen=200)
        self.console = deque(maxlen=200)
        self.goto_state = None
        self._stop_req = False
        self.frames = {"chase": (0, b""), "eye": (0, b"")}
        self._renderers = None
        self.cam = dict(distance=1.4, elevation=-24.0, azimuth=0.0)     # behind the robot, looking +x
        self.render_every = int(round(1 / (15 * self.DT)))      # ~15 fps
        self.last = dict(con=np.zeros(5, bool), tilt=0.0, height=0.0, gxy=0.0)
        self.chat_hist = {}
        self.brain = dict(mode="talk", model=None, vision_model=None)
        self.rec = None                     # recording: dict(frames, t0, cmds, world)
        self.cmd_log = []                   # (sim t, line) — everything that drove the robot
        self.walk = None                    # residual walking policy: dict(policy, name, t_next, res)
        self.walk_note = "gait: analytic wave gait"
        self.llm_base = os.environ.get("ROCKY_LLM_BASE_URL", "http://127.0.0.1:8080/v1")
        self.llm_key = _local_ai_key()
        self._llm = None
        self.browser_audio = True           # D051: chord words play in the browser (phone too), not on the server
        self.lexicon = lexicon()
        self.gesture_names = sorted(self.gestures)
        self.log(f"world: {self.world_name} | {self.righter_note}")

    def reload_library(self):
        """Any thread: re-read keyframe gestures + chord words from disk."""
        self.gestures = all_gestures()
        self.gesture_names = sorted(self.gestures)
        self.lexicon = lexicon()

    # ------------------------------------------------------ D051: hardware
    def hw_connect(self, port):
        """Any thread. Open the bus (or the mock); the sim thread mirrors from the next step."""
        self.hw_disconnect()
        hw = HardwareBridge(port, on_event=lambda k, m: (self.events.append(("hw:" + k, m)), self.log(f"hw {k}: {m}")))
        self.hw = hw
        return hw.status()

    def hw_disconnect(self):
        hw, self.hw = self.hw, None
        if hw is not None:
            hw.close()
            self.log(f"hw: disconnected {hw.port}")
        return hw is not None

    # ------------------------------------------------------ D051: gesture studio
    def preview_pose(self, spec, t=None):
        """Sim thread: hold the keyframe gesture's pose at time t (None = whole
        gesture's last frame) until preview_off / a gesture / a walk."""
        kg = KeyframeGesture(spec)
        tt = kg.total if t is None else float(t)
        fn = lambda g, _t, kg=kg, tt=tt: kg(g, tt)            # noqa: E731
        with self.lock:
            self.gesture = (fn, float("inf"), self.t)
            self.cmd_v[:] = 0
        self.mode = "posing"
        return kg.total

    def preview_off(self):
        with self.lock:
            self.gesture = None
        self.mode = "idle"

    # ---------------------------------------------------------- plumbing
    def log(self, line):
        self.console.append((round(self.t, 2), str(line)))

    def note_cmd(self, line):
        """Everything that moved the robot goes here (recordings replay it)."""
        self.cmd_log.append((round(self.t, 3), line))
        if self.rec is not None:
            self.rec["cmds"].append((round(self.t - self.rec["t0"], 3), line))

    async def call(self, fn):
        """Run fn() in the sim thread; await its result."""
        loop = asyncio.get_running_loop()
        fut = loop.create_future()

        def job():
            try:
                r = fn()
                loop.call_soon_threadsafe(fut.set_result, r)
            except Exception as e:                       # surface, don't kill the sim
                loop.call_soon_threadsafe(fut.set_exception, e)
        self.pending.put(job)
        return await fut

    def _drain(self):
        while True:
            try:
                job = self.pending.get_nowait()
            except queue.Empty:
                return
            try:
                job()
            except Exception as e:
                self.log(f"job failed: {e}")

    # ---------------------------------------------------------- the loop
    def run_forever(self):
        t_wall = time.monotonic()
        while self.alive:
            self._drain()
            if self.paused:
                time.sleep(0.02)
                t_wall = time.monotonic()
                continue
            self.step()
            t_wall += self.DT / max(self.speed, 0.05)
            lag = t_wall - time.monotonic()
            if lag > 0:
                time.sleep(lag)
            elif lag < -0.05:
                t_wall = time.monotonic()

    def step(self):
        gs = self.goto_state
        if gs is not None:
            self._goto_pre(gs)
        if self.walk is not None:
            self._walk_residual()               # sets self.residual for this step's targets
        super().step()
        if gs is not None and self.goto_state is gs:
            self._goto_post(gs)
        if self._k % self.render_every == 0:
            self._render()

    # ------------------------------------------------- residual walker
    def _walk_residual(self):
        """D050: the PPO residual gait policy (rocky_env.PebbleEnv's contract:
        50 Hz, obs = gravity, gyro, qpos, qvel, gait phase, command/[60,60,0.6],
        action = ±0.25 rad added to the analytic targets) on top of the
        supervisor's output while the reflex state is NORMAL."""
        w = self.walk
        if self.sup.state != "NORMAL" or self.gesture is not None:
            self.residual = None
            return
        if self.t >= w["t_next"]:
            w["t_next"] = self.t + 0.02
            d = self.data
            R = d.xmat[self.torso].reshape(3, 3)
            grav = R.T @ np.array([0, 0, -1.0])
            gyro = R.T @ d.cvel[self.torso][0:3]
            ph = 2 * np.pi * ((self.sup.t_gait / self.gait.T) % 1.0)
            with self.lock:
                v = self.cmd_v.copy()
            obs = np.concatenate([grav, gyro, d.qpos[w["jadr"]], d.qvel[w["vadr"]],
                                  [np.sin(ph), np.cos(ph)], v / np.array([60.0, 60.0, 0.6])]).astype(np.float32)
            a = np.clip(w["policy"](obs), -1, 1)
            w["res"] = 0.25 * a
        self.residual = w["res"]

    def set_walk(self, name):
        """Sim thread. name: 'off' or a runs/NAME with a gait checkpoint."""
        self.residual = None
        if name in (None, "", "off", "analytic"):
            self.walk = None
            self.walk_note = "gait: analytic wave gait"
            return self.walk_note
        path = os.path.join(HERE, "runs", name, "latest.pt")
        if not os.path.exists(path):
            return f"no runs/{name}/latest.pt"
        try:
            import torch
            from train_ppo import Agent, RunningMeanStd
            ck = torch.load(path, map_location="cpu", weights_only=False)
            if ck.get("obs_dim", 41) != 41:
                return f"{name} is not a gait checkpoint (obs {ck.get('obs_dim')})"
            agent = Agent(41, ck.get("act_dim", 15))
            agent.load_state_dict(ck["model"])
            agent.eval()
            rms = RunningMeanStd((41,))
            rms.load_state_dict(ck["obs_rms"])

            def policy(obs):
                on = (obs - rms.mean) / np.sqrt(rms.var + 1e-8)
                with torch.no_grad():
                    return agent.actor(torch.as_tensor(np.clip(on, -10, 10), dtype=torch.float32)
                                       .unsqueeze(0)).squeeze(0).numpy()
        except Exception as e:
            return f"walk policy {name}: {e}"
        self.walk = dict(policy=policy, name=name, t_next=self.t, res=np.zeros(15),
                         jadr=[self.model.joint(f"{n}{i}").qposadr[0] for i in range(5) for n in ("yaw", "hip", "knee")],
                         vadr=[self.model.joint(f"{n}{i}").dofadr[0] for i in range(5) for n in ("yaw", "hip", "knee")])
        self.walk_note = f"gait: analytic + residual policy {name} ({ck.get('global_step', 0):,} steps)"
        return self.walk_note

    # ------------------------------------------------------------- goto
    def _goto_pre(self, gs):
        tw = self.t - gs["t0"]
        p = self.data.xpos[self.torso]
        dx, dy = gs["tx"] - p[0], gs["ty"] - p[1]
        dist = float(np.hypot(dx, dy))
        if gs["outcome"] is None and dist < 0.025:
            gs["outcome"] = ("arrived", tw)
            self.sup.request_stop()
        if gs["outcome"] is None:
            if dist < gs["best"] - 0.02:
                gs["best"], gs["best_t"] = dist, tw
            elif tw - gs["best_t"] > GOTO_STUCK_S and tw > 2.0:
                gs["outcome"] = ("stuck", tw)          # D050: blocked, not a void
                self.sup.request_stop()
                self.log(f"goto stuck {dist*100:.0f} cm short (no progress for {GOTO_STUCK_S:.0f} s)")
            elif tw > GOTO_CAP_S:
                gs["outcome"] = ("timeout", tw)
                self.sup.request_stop()
        if self._stop_req and gs["outcome"] is None:
            gs["outcome"] = ("user", tw)
            self.sup.request_stop()
        self._stop_req = False
        if gs["outcome"] is None:
            ramp = min(tw / 0.6, 1.0)
            ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0.0, 0.0)
            Rm = self.data.xmat[self.torso].reshape(3, 3)
            yaw = float(np.arctan2(Rm[1, 0], Rm[0, 0]))
            cy, sy = np.cos(yaw), np.sin(yaw)
            ux, uy = cy * ux + sy * uy, -sy * ux + cy * uy
            vx, vy, wz = gs["react"].command(tw, V_GOTO * ramp * ux, V_GOTO * ramp * uy, 0.0)
            halted = gs["react"].retreat_until is not None and tw >= gs["react"].retreat_until
            if halted:
                gs["outcome"] = ("cliff", tw)
                self.sup.request_stop()
                vx, vy, wz = 0.0, 0.0, 0.0
        else:
            vx, vy, wz = 0.0, 0.0, 0.0
        with self.lock:
            self.cmd_v[:] = [vx, vy, wz]

    def _goto_post(self, gs):
        tw = self.t - gs["t0"]
        if gs["outcome"] is None and self._k % 10 == 0 and self.sup.state == "NORMAL":
            g = self.gait
            ph = [(self.sup.t_gait / g.T + g.phase_off[i]) % 1.0 for i in range(5)]
            settled = [ph[i] < g.duty and 0.12 < ph[i] / g.duty < 0.95 for i in range(5)]
            # D050: a void needs the leg to have PROBED all the way down and found nothing
            fired = gs["det"].update(tw, settled, self.last["con"], probed_out=self.probe_out)
            if fired:
                gs["react"].on_void(tw)
                self.events.append(("void", round(float(self.data.xpos[self.torso][0]), 3)))
                self.log(f"VOID detected (leg {fired} probed {PROBE_MAX:.0f} mm down, nothing there) — retreating")
        if gs["outcome"] is None and self.last["tilt"] > 60:
            gs["outcome"] = ("FELL", tw)
        if gs["outcome"] is not None and tw > gs["outcome"][1] + 1.5:
            reason = gs["outcome"][0]
            res = {"ok": reason == "arrived", "stopped": reason, "pose": self.pose()}
            if reason == "cliff":
                res["detail"] = "VOID detected by the real detector; PLANT->BRACE halt"
            if reason == "FELL":
                res["detail"] = "the robot fell during the goto; the righter takes over"
            if reason == "stuck":
                res["detail"] = "no progress toward the target: something is in the way (not a void)"
            self.mode = "idle" if reason == "arrived" else "safe_stop"
            self.goto_state = None
            with self.lock:
                self.cmd_v[:] = 0
            self._resolve(gs, res)

    @staticmethod
    def _resolve(gs, res):
        fut, loop = gs["fut"], gs["loop"]
        if not fut.done():
            loop.call_soon_threadsafe(fut.set_result, res)

    def _start_goto(self, tx, ty, fut, loop):
        if self.goto_state is not None:
            self._resolve(self.goto_state, {"ok": False, "stopped": "preempted", "pose": self.pose()})
        self.goto_state = dict(tx=float(tx), ty=float(ty), t0=self.t, det=CliffDetector(),
                               react=CliffReaction(self.gait), outcome=None, fut=fut, loop=loop,
                               best=float("inf"), best_t=0.0)
        self.mode = "walking"
        self.events.append(("goto", (round(float(tx), 3), round(float(ty), 3))))
        self.log(f"goto ({tx:.2f}, {ty:.2f})")

    # ------------------------------------------------------------ render
    def _render(self):
        if self._renderers is None:
            try:
                chase = mujoco.Renderer(self.model, 400, 640)
                eye = mujoco.Renderer(self.model, 240, 320)
            except Exception as e:
                self.log(f"no offscreen renderer ({e}); cameras off")
                self._renderers = False
                return
            cam = mujoco.MjvCamera()
            ecam = mujoco.MjvCamera()
            ecam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            ecam.fixedcamid = self.model.camera("eye").id
            # world objects are geom group 3 (lidar-visible, sim_lidar's
            # convention) and renderers hide group 3 by default: show it
            self._vopt = mujoco.MjvOption()
            self._vopt.geomgroup[3] = 1
            self._renderers = (chase, eye, cam, ecam)
        if self._renderers is False:
            return
        chase, eye, cam, ecam = self._renderers
        cam.lookat[:] = self.data.xpos[self.torso]
        cam.distance, cam.elevation, cam.azimuth = self.cam["distance"], self.cam["elevation"], self.cam["azimuth"]
        chase.update_scene(self.data, cam, self._vopt)
        self._overlay(chase.scene)
        self._put("chase", chase.render())
        eye.update_scene(self.data, ecam, self._vopt)
        self._put("eye", eye.render())

    def _overlay(self, scn):
        """State marker + command arrow (the playground HUD) and the goto
        target as a flag, appended to the renderer's scene."""
        n0 = scn.ngeom
        try:
            self.draw_hud_into(scn)
        except Exception:
            scn.ngeom = n0
        gs = self.goto_state
        if gs is not None and scn.ngeom + 2 <= scn.maxgeom:
            base = np.array([gs["tx"], gs["ty"], self.z0 + 0.002])
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CYLINDER, np.array([0.03, 0.002, 0]),
                                base, np.eye(3).flatten(), np.array([0.4, 0.8, 1.0, 0.6], dtype=np.float32))
            scn.ngeom += 1
            g = scn.geoms[scn.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                                np.eye(3).flatten(), np.array([0.4, 0.8, 1.0, 0.9], dtype=np.float32))
            mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.006, base + [0, 0, 0.16], base + [0, 0, 0.02])
            scn.ngeom += 1

    def draw_hud_into(self, scn):
        """draw_hud without resetting ngeom (the renderer's scene already
        holds the world)."""
        n0 = scn.ngeom
        self.draw_hud(_SceneTail(scn, n0))

    def _put(self, name, rgb):
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=72)
        n = self.frames[name][0] + 1
        self.frames[name] = (n, buf.getvalue())
        if name == "chase" and self.rec is not None and len(self.rec["frames"]) < 15 * 300:
            self.rec["frames"].append(rgb.copy())

    # ------------------------------------------------------- world swaps
    def set_world(self, spec, name="custom"):
        """Sim thread. Rebuild the world; the robot respawns upright where it
        stands (or at the origin if that is now inside something)."""
        model, z0 = build_world(spec)
        old = self.data
        p_xy = old.xpos[self.torso][:2].copy()
        yaw = float(np.arctan2(old.xmat[self.torso].reshape(3, 3)[1, 0], old.xmat[self.torso].reshape(3, 3)[0, 0]))
        same_world = (name == self.world_name)
        self.model, self.z0 = model, z0
        self.world_spec, self.world_name = dict(spec), name
        self.data = mujoco.MjData(model)
        self.torso = model.body("torso").id
        self.fids = [model.geom(f"foot{i}").id for i in range(5)]
        self._renderers = None
        self.goto_state = None
        self.gesture = None
        self.push = None
        ck = getattr(getattr(self.sup, "righter", None), "ckpt", None)   # keep the chosen righter
        self._respawn(p_xy if same_world else np.zeros(2), yaw)
        self.righter_note = self._install_righter(ck)
        if self.walk is not None:
            self.set_walk(self.walk["name"])
        self.log(f"world: {name} ({model.ngeom} geoms) | {self.righter_note}")

    def _respawn(self, xy=(0.0, 0.0), yaw=0.0):
        from pebble_gait import leg_ik, body_to_leg
        q0 = np.array([leg_ik(body_to_leg(i, self.gait.p_nom[i])) for i in range(5)]).flatten()
        jadr = [self.model.joint(f"{n}{i}").qposadr[0] for i in range(5) for n in ("yaw", "hip", "knee")]
        mujoco.mj_resetData(self.model, self.data)      # world bodies (the ball) back to their spawn
        self.data.qpos[0:3] = [xy[0], xy[1], (self.gait.h + 14) / 1000.0 + self.z0]
        self.data.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        self.data.qpos[jadr] = q0
        self.data.ctrl[:] = 0
        self.data.ctrl[:15] = q0
        mujoco.mj_forward(self.model, self.data)
        with self.lock:
            self.cmd_v[:] = 0
        self.sup = ReflexSupervisor(self.gait)
        self.mode = "idle"

    def reset(self):
        """Sim thread: respawn at the origin, fresh supervisor, same world."""
        self.goto_state = None
        self.gesture = None
        self.push = None
        ck = getattr(getattr(self.sup, "righter", None), "ckpt", None)   # keep the chosen righter
        self._respawn()
        self.righter_note = self._install_righter(ck)
        self.log("reset")
        return "reset: origin, upright, planted"

    # ------------------------------------------------------ tool surface
    def pose(self):
        p = self.data.xpos[self.torso]
        R = self.data.xmat[self.torso].reshape(3, 3)
        return {"x": round(float(p[0]), 3), "y": round(float(p[1]), 3),
                "yaw_deg": round(float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))), 1)}

    def snapshot(self):
        """Sim thread: the state feed."""
        with self.lock:
            v = self.cmd_v.copy()
        gs = self.goto_state
        return dict(t=round(self.t, 2), pose=self.pose(), state=self.sup.state, mode=self.mode,
                    cmd=[round(float(x), 2) for x in v], tilt=round(self.last["tilt"], 1),
                    height=round(self.last["height"] * 1000), contacts=[bool(c) for c in self.last["con"]],
                    gyro=round(self.last["gxy"], 2), world=self.world_name, righter=self.righter_note,
                    speed=self.speed, paused=self.paused, trips=self.sup.trip_count, falls=self.sup.fall_count,
                    recording=self.rec is not None, walk=self.walk_note, frame=self.frames["chase"][0],
                    said=list(self.said), servo=dict(self.servo.p), browser_audio=self.browser_audio,
                    hw=None if self.hw is None else dict(port=self.hw.port, mirror=self.hw.mirror,
                                                         legs=[bool(x) for x in self.hw.legs_present],
                                                         n=len(self.hw.present), errors=self.hw.errors),
                    goto=None if gs is None else [gs["tx"], gs["ty"]], gesture=self.gesture is not None,
                    events=list(self.events)[-12:], console=list(self.console)[-40:],
                    brain=self.brain, gait=dict(T=self.gait.T, h=self.gait.h, R0=self.gait.R0,
                                                duty=self.gait.duty, hstep=self.gait.hstep,
                                                phase=round(float((self.sup.t_gait / self.gait.T) % 1.0), 3)),
                    reflex=dict(trip=self.sup.gyro_trip, stall_s=self.sup.stall_s,
                                fallen_max_s=self.sup.fallen_max_s))

    async def tool_say(self, word):
        if chord_wav(word) is None:
            return {"ok": False, "error": f"unknown chord word {word!r}", "hint": f"lexicon: {', '.join(self.lexicon)}"}
        self.events.append(("say", word))
        r = await self.call(lambda: self.do(f"say {word}"))
        return {"ok": True, "word": word, "note": r}

    async def tool_gesture(self, name):
        if name not in self.gestures:
            return {"ok": False, "error": f"unknown gesture {name!r}", "hint": f"available: {', '.join(self.gesture_names)}"}
        if self.goto_state is not None or float(np.abs(self.cmd_v).sum()) > 0:
            return {"ok": False, "error": "busy", "hint": "walking; stop() first"}
        fn, total = self.gestures[name]
        self.events.append(("gesture", name))

        def start():
            with self.lock:
                self.gesture = (fn, total, self.t)
            self.mode = "gesturing"
        await self.call(start)
        t_end = time.monotonic() + total / max(self.speed, 0.05) + 3.0
        while self.gesture is not None and time.monotonic() < t_end:
            await asyncio.sleep(0.05)
        self.mode = "idle"
        return {"ok": True, "gesture": name, "duration_s": round(total, 1), "pose": self.pose(),
                "note": "rendered in physics"}

    async def tool_goto(self, x, y):
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        await self.call(lambda: self._start_goto(x, y, fut, loop))
        return await fut

    async def tool_stop(self):
        def do_stop():
            with self.lock:
                self.cmd_v[:] = 0
            if self.goto_state is not None:
                self._stop_req = True
            else:
                self.sup.request_stop()
                self.mode = "safe_stop"
        await self.call(do_stop)
        if self.hw is not None and self.hw.mirror == "sim2real":
            self.hw.set_mirror("off")           # the real legs stop streaming; torque stays (they hold)
        self.events.append(("stop", None))
        return {"ok": True, "mode": self.mode}

    async def tool_scan_summary(self):
        def do_scan():
            mujoco.mj_forward(self.model, self.data)
            angles, ranges, _ = lidar_scan(self.model, self.data, self.torso)
            R = self.data.xmat[self.torso].reshape(3, 3)
            yaw = float(np.arctan2(R[1, 0], R[0, 0]))
            bear = np.degrees((angles + yaw + np.pi) % (2 * np.pi) - np.pi)
            sectors = []
            for c in range(-180, 180, 45):
                m = (bear >= c) & (bear < c + 45)
                hit = ranges[m][np.isfinite(ranges[m])]
                sectors.append({"bearing_deg": c + 22, "min_range_m": round(float(hit.min()), 3) if hit.size else None,
                                "clear": bool(hit.size == 0)})
            out = {"ok": True, "n_points": int(len(ranges)), "range_max_m": float(RANGE_MAX),
                   "world": self.world_name, "sectors": sectors,
                   "frontiers": [{"bearing_deg": s["bearing_deg"]} for s in sectors if s["clear"]],
                   "note": "2D horizontal scan: walls/obstacles yes, voids below its plane no"}
            finite = np.isfinite(ranges)
            if finite.any():
                i = int(np.argmin(np.where(finite, ranges, np.inf)))
                out["nearest_obstacle_m"] = round(float(ranges[i]), 3)
                out["nearest_obstacle_bearing_deg"] = round(float(bear[i]), 1)
            return out
        out = await self.call(do_scan)
        self.events.append(("scan", out.get("nearest_obstacle_m")))
        return out

    async def tool_status(self):
        s = await self.call(self.snapshot)
        return {"ok": True, "mode": s["mode"], "pose": s["pose"], "reflex_state": s["state"],
                "tilt_deg": s["tilt"], "world": s["world"], "battery_v": 11.9,
                "last_events": [list(e) for e in s["events"][-5:]]}

    async def tool_look(self, model=None):
        n, jpg = self.frames["eye"]
        if not jpg:
            return {"ok": False, "error": "no eye frame yet (renderer off?)"}
        m = model or self.brain.get("vision_model") or "vision-model"
        b64 = base64.b64encode(jpg).decode()

        def ask():
            from openai import OpenAI
            client = OpenAI(base_url=self.llm_base, api_key=self.llm_key, timeout=120)
            r = client.chat.completions.create(model=m, max_tokens=160, messages=[
                {"role": "user", "content": [
                    {"type": "text", "text": VISION_PROMPT},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}}]}])
            return (r.choices[0].message.content or "").strip()
        try:
            text = await asyncio.to_thread(ask)
        except Exception as e:
            return {"ok": False, "error": f"vision model {m}: {e}"}
        self.events.append(("look", text[:60]))
        self.log(f"look ({m}): {text}")
        return {"ok": True, "model": m, "description": text}

    TOOL_NAMES = ("say", "gesture", "goto", "stop", "scan_summary", "status", "look")

    async def tool(self, name, args):
        if name not in self.TOOL_NAMES:
            return {"ok": False, "error": f"no such tool {name}"}
        if name in ("goto", "gesture", "say", "stop"):
            self.note_cmd(f"tool {name} {json.dumps(args or {})}")
        try:
            return await getattr(self, "tool_" + name)(**(args or {}))
        except TypeError as e:
            return {"ok": False, "error": f"bad arguments for {name}: {e}"}

    @property
    def tools(self):
        """The harness backend surface (say/gesture/goto/...) as an object,
        for harness.intent.execute and anything else written against the
        MCP contract. (The Playground's `gesture` attribute is its gesture
        STATE, hence the indirection.)"""
        return _ToolSurface(self)


    # ------------------------------------------------------------ brains
    def llm_models(self):
        try:
            from openai import OpenAI
            client = OpenAI(base_url=self.llm_base, api_key=self.llm_key, timeout=5)
            ids = [m.id for m in client.models.list().data]
        except Exception:
            ids = []
        skip = ("embedding", "reranker", "lab")
        return [m for m in ids if not any(m.startswith(s) for s in skip)]

    def claude_available(self):
        try:
            import anthropic                                        # noqa: F401
        except ImportError:
            return False
        return bool(os.environ.get("ANTHROPIC_API_KEY"))

    async def chat(self, text, mode=None, model=None):
        mode = mode or self.brain["mode"]
        self.brain["mode"] = mode
        if model:
            self.brain["model"] = model
        trace = []
        if mode == "talk":
            p = intent_plan(text)
            results = await intent_execute(self.tools, p)
            for name, res in results:
                trace.append({"tool": name, "result": res})
            reply = p["reply"]
            if results and results[-1][0] in ("status", "scan_summary"):
                reply += " " + json.dumps(results[-1][1])[:400]
            return {"reply": reply, "trace": trace, "mode": mode}
        if mode == "local":
            return await self._chat_openai(text, self.brain.get("model") or "qwen3.6-35b-a3b", trace)
        if mode == "claude":
            if not self.claude_available():
                return {"reply": "Claude in the cockpit needs `pip install anthropic` and ANTHROPIC_API_KEY. "
                                 "Without a key, run `./rocky.sh chat` in a terminal: with the cockpit up, "
                                 "Claude Code drives THIS sim over MCP and you watch it here.",
                        "trace": [], "mode": mode}
            return await self._chat_claude(text, self.brain.get("model") or "claude-sonnet-5", trace)
        return {"reply": f"unknown mode {mode}", "trace": [], "mode": mode}

    async def _chat_openai(self, text, model, trace):
        hist = self.chat_hist.setdefault("local", [{"role": "system", "content": SYSTEM}])
        if self._llm is None:
            self._llm = OpenAIChat(self.llm_base, self.llm_key, think=False)
        hist.append({"role": "user", "content": text})
        content = ""
        for _hop in range(6):
            try:
                resp = await asyncio.to_thread(self._llm.chat, model, hist, TOOLS)
            except Exception as e:
                return {"reply": f"local model error: {e}", "trace": trace, "mode": "local"}
            msg = resp["message"]
            content = msg.get("content") or ""
            calls = msg.get("tool_calls") or []
            hist.append({"role": "assistant", "content": content, "tool_calls": calls})
            if not calls:
                break
            for tc in calls:
                name = tc["function"]["name"]
                args = tc["function"]["arguments"] or {}
                res = await self.tool(name, args)
                trace.append({"tool": name, "args": args, "result": res})
                hist.append({"role": "tool", "name": name, "tool_call_id": tc.get("id"),
                             "content": json.dumps(res)})
        return {"reply": content, "trace": trace, "mode": "local", "model": model}

    async def _chat_claude(self, text, model, trace):
        import anthropic
        client = anthropic.Anthropic()
        hist = self.chat_hist.setdefault("claude", [])
        hist.append({"role": "user", "content": text})
        tools = [{"name": t["function"]["name"], "description": t["function"]["description"],
                  "input_schema": t["function"]["parameters"]} for t in TOOLS]
        content = ""
        for _hop in range(6):
            r = await asyncio.to_thread(lambda: client.messages.create(
                model=model, max_tokens=600, system=SYSTEM, tools=tools, messages=hist))
            hist.append({"role": "assistant", "content": r.content})
            uses = [b for b in r.content if b.type == "tool_use"]
            content = " ".join(b.text for b in r.content if b.type == "text")
            if not uses:
                break
            results = []
            for u in uses:
                res = await self.tool(u.name, dict(u.input))
                trace.append({"tool": u.name, "args": dict(u.input), "result": res})
                results.append({"type": "tool_result", "tool_use_id": u.id, "content": json.dumps(res)})
            hist.append({"role": "user", "content": results})
        return {"reply": content, "trace": trace, "mode": "claude", "model": model}


class _SceneTail:
    """View of an MjvScene whose ngeom counts from an offset, so the
    playground's draw_hud (which starts at 0) appends instead of clearing."""

    def __init__(self, scn, n0):
        self._scn, self._n0 = scn, n0

    @property
    def ngeom(self):
        return self._scn.ngeom - self._n0

    @ngeom.setter
    def ngeom(self, v):
        self._scn.ngeom = self._n0 + v

    @property
    def maxgeom(self):
        return self._scn.maxgeom - self._n0

    @property
    def geoms(self):
        return _Shift(self._scn.geoms, self._n0)


class _Shift:
    def __init__(self, arr, n0):
        self._arr, self._n0 = arr, n0

    def __getitem__(self, i):
        return self._arr[self._n0 + i]


class _ToolSurface:
    def __init__(self, sim):
        self._sim = sim

    def __getattr__(self, name):
        if name in CockpitSim.TOOL_NAMES:
            return getattr(self._sim, "tool_" + name)
        raise AttributeError(name)


# ---------------------------------------------------------------- HTTP
def make_app(sim: CockpitSim):
    from starlette.applications import Starlette
    from starlette.responses import JSONResponse, HTMLResponse, StreamingResponse, Response
    from starlette.routing import Route

    ui_path = os.path.join(HERE, "cockpit_ui.html")

    async def index(_):
        return HTMLResponse(open(ui_path).read())

    async def state(_):
        return JSONResponse(await sim.call(sim.snapshot))

    async def events(_):
        async def gen():
            while sim.alive:
                s = await sim.call(sim.snapshot)
                yield f"data: {json.dumps(s)}\n\n"
                await asyncio.sleep(0.1)
        return StreamingResponse(gen(), media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache"})

    async def video(request):
        cam = request.path_params["cam"]
        if cam not in sim.frames:
            return Response("no such camera", status_code=404)

        async def gen():
            last = -1
            while sim.alive:
                n, jpg = sim.frames[cam]
                if jpg and n != last:
                    last = n
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                           + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                await asyncio.sleep(0.03)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    async def frame(request):
        n, jpg = sim.frames.get(request.path_params["cam"], (0, b""))
        return Response(jpg or b"", media_type="image/jpeg")

    async def cmd(request):
        body = await request.json()
        line = str(body.get("line", "")).strip()
        if not line or line.split()[0].lower() == "quit":
            return JSONResponse({"reply": "(the header's quit button stops the server; quit is not a console command here)"})
        sim.note_cmd(line)
        r = await sim.call(lambda: sim.do(line))
        sim.log(f"> {line}")
        if r:
            for ln in str(r).splitlines():
                sim.log(ln)
        return JSONResponse({"reply": r})

    async def teleop(request):
        body = await request.json()
        sim.note_cmd("teleop " + str(body.get("key", "")))
        r = sim.teleop(str(body.get("key", "")))
        if r:
            sim.log(r)
        return JSONResponse({"reply": r})

    async def chat(request):
        body = await request.json()
        r = await sim.chat(str(body.get("text", "")), body.get("mode"), body.get("model"))
        sim.log(f"[{r['mode']}] {body.get('text', '')[:80]}")
        for tcall in r["trace"]:
            sim.log(f"  {tcall['tool']}({json.dumps(tcall.get('args', {}))}) -> {json.dumps(tcall['result'])[:120]}")
        if r.get("reply"):
            sim.log(f"  reply: {r['reply'][:200]}")
        return JSONResponse(r)

    async def chat_clear(request):
        body = await request.json()
        sim.chat_hist.pop(body.get("mode", "local"), None)
        return JSONResponse({"ok": True})

    async def brain(request):
        body = await request.json()
        for k in ("mode", "model", "vision_model"):
            if k in body:
                sim.brain[k] = body[k]
        return JSONResponse(sim.brain)

    async def models(_):
        ids = await asyncio.to_thread(sim.llm_models)
        vis = [m for m in ids if any(k in m for k in ("vl", "vision", "gemma"))]
        return JSONResponse({"models": ids, "vision_models": vis, "claude": sim.claude_available(),
                             "base_url": sim.llm_base, "brain": sim.brain})

    async def world_get(_):
        return JSONResponse({"presets": list(PRESETS), "kinds": list(KINDS), "name": sim.world_name,
                             "spec": sim.world_spec})

    async def world_set(request):
        body = await request.json()
        if "preset" in body:
            name = body["preset"]
            spec = dict(PRESETS[name])
        else:
            name, spec = body.get("name", "custom"), body["spec"]
        await sim.call(lambda: sim.set_world(spec, name))
        return JSONResponse({"ok": True, "name": name, "spec": spec})

    async def world_add(request):
        body = await request.json()
        spec = json.loads(json.dumps(sim.world_spec))
        spec.setdefault("objects", [])
        if "object" in body:
            spec["objects"].append(body["object"])
        for k in ("friction", "gravity_tilt_deg", "gravity_tilt_dir_deg", "terrain", "base"):
            if k in body:
                spec[k] = body[k]
        if body.get("clear"):
            spec["objects"] = []
            spec["terrain"] = None
        await sim.call(lambda: sim.set_world(spec, "custom"))
        return JSONResponse({"ok": True, "spec": spec})

    async def rl_runs(_):
        from rl_dashboard import summarize_runs
        return JSONResponse({"runs": await asyncio.to_thread(summarize_runs)})

    async def rl_righter(request):
        body = await request.json()
        r = await sim.call(lambda: sim.do(f"righter {body.get('name', 'off')}"))
        sim.log(r)
        return JSONResponse({"reply": r})

    _curves = {"t": 0.0, "png": b""}

    async def rl_curves(_):
        if time.time() - _curves["t"] > 60:
            from rl_dashboard import curves
            path = await asyncio.to_thread(curves, os.path.join(HERE, "out", "rl_curves.png"))
            _curves["png"] = open(path, "rb").read()
            _curves["t"] = time.time()
        return Response(_curves["png"], media_type="image/png")

    async def look(request):
        body = await request.json() if request.headers.get("content-length", "0") != "0" else {}
        return JSONResponse(await sim.tool_look(body.get("model")))

    async def shove(request):
        body = await request.json()
        fx, fy = float(body.get("fx", 40)), float(body.get("fy", 0))
        dur = float(body.get("dur", 0.4))

        def do_push():
            sh = Shove(fx, fy, dur=dur, t0=sim.t)
            with sim.lock:
                sim.push = sh
            return sh.describe(sim.model)
        r = await sim.call(do_push)
        sim.log(f"shove: {r}")
        return JSONResponse({"reply": r})

    async def reset(_):
        return JSONResponse({"reply": await sim.call(sim.reset)})

    async def speed(request):
        body = await request.json()
        if "speed" in body:
            sim.speed = float(np.clip(float(body["speed"]), 0.1, 8.0))
        if "paused" in body:
            sim.paused = bool(body["paused"])
        return JSONResponse({"speed": sim.speed, "paused": sim.paused})

    VIEWS = {"follow": dict(distance=0.9, elevation=-18.0), "wide": dict(distance=2.2, elevation=-35.0),
             "top": dict(distance=2.6, elevation=-89.0), "low": dict(distance=1.0, elevation=-6.0)}

    async def camera(request):
        body = await request.json()
        if body.get("view") in VIEWS:
            sim.cam.update(VIEWS[body["view"]])
        for k in ("distance", "elevation", "azimuth"):
            if k in body:
                sim.cam[k] = float(body[k])
        return JSONResponse(sim.cam)

    async def scan(_):
        def do_scan():
            angles, ranges, origin = lidar_scan(sim.model, sim.data, sim.torso)
            R = sim.data.xmat[sim.torso].reshape(3, 3)
            yaw = float(np.arctan2(R[1, 0], R[0, 0]))
            pts = []
            for a, r in zip(angles[::4], ranges[::4]):
                if np.isfinite(r):
                    pts.append([round(float(origin[0] + r * np.cos(a + yaw)), 3),
                                round(float(origin[1] + r * np.sin(a + yaw)), 3)])
            return {"points": pts, "origin": [round(float(origin[0]), 3), round(float(origin[1]), 3)], "yaw": yaw}
        return JSONResponse(await sim.call(do_scan))

    async def tool(request):
        name = request.path_params["name"]
        try:
            args = await request.json()
        except Exception:
            args = {}
        return JSONResponse(await sim.tool(name, args or {}))

    async def favicon(_):
        return Response(b"", status_code=204)

    REC_DIR = os.path.join(HERE, "out", "recordings")

    async def record(request):
        body = await request.json()
        action = body.get("action", "start")
        if action == "start":
            if sim.rec is not None:
                return JSONResponse({"ok": False, "error": "already recording"})
            sim.rec = dict(frames=[], t0=sim.t, cmds=[], world=sim.world_name, spec=json.loads(json.dumps(sim.world_spec)))
            sim.log("● recording")
            return JSONResponse({"ok": True, "recording": True})
        rec, sim.rec = sim.rec, None
        if rec is None:
            return JSONResponse({"ok": False, "error": "not recording"})
        name = body.get("name") or time.strftime("rec_%Y%m%d_%H%M%S")
        name = "".join(c for c in name if c.isalnum() or c in "_-.")
        d = os.path.join(REC_DIR, name)
        os.makedirs(d, exist_ok=True)
        meta = dict(name=name, world=rec["world"], spec=rec["spec"], t0=rec["t0"],
                    duration_s=round(sim.t - rec["t0"], 2), cmds=rec["cmds"], frames=len(rec["frames"]))

        def write():
            import imageio
            with open(os.path.join(d, "run.json"), "w") as f:
                json.dump(meta, f, indent=1)
            if rec["frames"]:
                imageio.mimsave(os.path.join(d, "clip.mp4"), rec["frames"], fps=15, codec="libx264", quality=7)
                import shutil
                import subprocess
                if shutil.which("ffmpeg"):
                    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", os.path.join(d, "clip.mp4"),
                                    "-vf", "fps=10,scale=420:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=bayer:bayer_scale=5",
                                    os.path.join(d, "clip.gif")], check=False)
            return meta
        await asyncio.to_thread(write)
        sim.log(f"■ saved recording {name} ({meta['frames']} frames, {len(meta['cmds'])} commands, {meta['duration_s']} s)")
        return JSONResponse({"ok": True, "recording": False, "meta": meta})

    async def recordings(_):
        out = []
        if os.path.isdir(REC_DIR):
            for name in sorted(os.listdir(REC_DIR), reverse=True):
                rj = os.path.join(REC_DIR, name, "run.json")
                if os.path.exists(rj):
                    m = json.load(open(rj))
                    m["files"] = [f for f in os.listdir(os.path.join(REC_DIR, name)) if f != "run.json"]
                    out.append(m)
        return JSONResponse({"recordings": out})

    async def rec_file(request):
        from starlette.responses import FileResponse
        name, fn = request.path_params["name"], request.path_params["file"]
        path = os.path.join(REC_DIR, os.path.basename(name), os.path.basename(fn))
        if not os.path.exists(path):
            return Response("not found", status_code=404)
        return FileResponse(path)

    replay_state = {"task": None}

    async def replay(request):
        body = await request.json()
        rj = os.path.join(REC_DIR, os.path.basename(body.get("name", "")), "run.json")
        if not os.path.exists(rj):
            return JSONResponse({"ok": False, "error": "no such recording"})
        m = json.load(open(rj))
        if replay_state["task"] is not None and not replay_state["task"].done():
            replay_state["task"].cancel()
        await sim.call(lambda: sim.set_world(m["spec"], m["world"]))
        await sim.call(sim.reset)
        t_start = sim.t
        sim.log(f"▶ replaying {m['name']}: {len(m['cmds'])} commands over {m['duration_s']} s")

        async def run():
            for t_rel, line in m["cmds"]:
                while sim.t - t_start < t_rel:
                    await asyncio.sleep(0.02)
                if line.startswith("tool "):
                    _, name, args = line.split(" ", 2)
                    asyncio.ensure_future(sim.tool(name, json.loads(args)))
                elif line.startswith("teleop "):
                    sim.teleop(line[7:])
                else:
                    await sim.call(lambda l=line: sim.do(l))
            sim.log("▶ replay done")
        replay_state["task"] = asyncio.ensure_future(run())
        return JSONResponse({"ok": True, "cmds": len(m["cmds"])})

    async def world_saved(_):
        return JSONResponse({"saved": saved_worlds()})

    async def world_save(request):
        body = await request.json()
        name = save_world(body.get("name", "world"), sim.world_spec)
        return JSONResponse({"ok": True, "name": name, "saved": saved_worlds()})

    async def world_load(request):
        body = await request.json()
        spec = load_world(os.path.basename(body["name"]))
        await sim.call(lambda: sim.set_world(spec, body["name"]))
        return JSONResponse({"ok": True, "spec": spec})

    async def world_random(request):
        body = await request.json()
        spec = random_course(int(body.get("seed", 0)), int(body.get("n", 8)))
        name = f"random #{int(body.get('seed', 0))}"
        await sim.call(lambda: sim.set_world(spec, name))
        return JSONResponse({"ok": True, "name": name, "spec": spec})

    async def rl_walk(request):
        body = await request.json()
        r = await sim.call(lambda: sim.set_walk(body.get("name", "off")))
        sim.log(r)
        return JSONResponse({"reply": r})

    async def voice(request):
        """Browser audio blob -> ffmpeg -> whisper-server -> text."""
        import shutil
        import subprocess
        import tempfile
        form = await request.form()
        up = form.get("audio")
        if up is None:
            return JSONResponse({"ok": False, "error": "no audio"})
        raw = await up.read()
        if not shutil.which("ffmpeg"):
            return JSONResponse({"ok": False, "error": "ffmpeg not installed"})
        with tempfile.TemporaryDirectory() as td:
            src = os.path.join(td, "in.webm")
            wav = os.path.join(td, "in.wav")
            open(src, "wb").write(raw)
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", src, "-ar", "16000", "-ac", "1", wav], check=False)
            if not os.path.exists(wav):
                return JSONResponse({"ok": False, "error": "could not decode the audio"})
            import httpx
            try:
                async with httpx.AsyncClient(timeout=60) as c:
                    r = await c.post(os.environ.get("ROCKY_WHISPER_URL", "http://127.0.0.1:8082") + "/v1/audio/transcriptions",
                                     files={"file": ("in.wav", open(wav, "rb"), "audio/wav")},
                                     data={"response_format": "text", "temperature": "0.0"})
                text = r.text.strip()
            except Exception as e:
                return JSONResponse({"ok": False, "error": f"whisper-server: {e}"})
        sim.log(f"🎤 {text}")
        return JSONResponse({"ok": True, "text": text})

    async def quit_(_):
        sim.log("quit requested from the page")
        sim.alive = False

        def bye():
            time.sleep(0.4)
            os._exit(0)
        threading.Thread(target=bye, daemon=True).start()
        return JSONResponse({"ok": True})

    # ---------------------------------------------------------- D051: audio in the browser
    async def audio_file(request):
        wav = chord_wav(request.path_params["word"])
        if wav is None:
            return JSONResponse({"error": "no such word"}, status_code=404)
        return Response(open(wav, "rb").read(), media_type="audio/wav", headers={"Cache-Control": "max-age=60"})

    async def audio_mode(request):
        body = await request.json()
        sim.browser_audio = bool(body.get("browser", True))
        return JSONResponse({"browser": sim.browser_audio, "player": sim.player})

    def _chord_mod():
        sys.path.insert(0, AUDIO_DIR)
        import chordspeak2
        return chordspeak2

    def _wav_bytes(buf):
        import wave
        bio = io.BytesIO()
        with wave.open(bio, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(_chord_mod().SR)
            w.writeframes((np.clip(buf, -1, 1) * 32767).astype(np.int16).tobytes())
        return bio.getvalue()

    def _render_spec(spec):
        cs = _chord_mod()
        syls = []
        for sy in spec.get("syllables", []):
            d = {k: (float(v) if k not in ("ratios",) else [float(x) for x in v]) for k, v in sy.items()
                 if k in ("dur", "gap", "ratios", "root_mul", "amp", "rough", "breath", "bend_cents",
                          "detune_cents", "wobble", "attack", "tail", "tilt")}
            d.setdefault("dur", 0.2)
            d.setdefault("ratios", [1.0, 1.5, 2.0])
            d["dur"] = float(np.clip(d["dur"], 0.03, 2.0))
            syls.append(d)
        if not syls:
            syls = [dict(dur=0.2, ratios=[1.0, 1.5, 2.0])]
        return cs.word(syls, root_hz=float(np.clip(spec.get("root", 120.0), 40.0, 400.0)))

    async def chord_list(_):
        cs = _chord_mod()
        canon = {k: {"root": v[0], "syllables": v[1]} for k, v in cs.vocabulary().items()}
        custom = {}
        if os.path.isdir(CHORD_SPEC_DIR):
            for fn in sorted(os.listdir(CHORD_SPEC_DIR)):
                if fn.endswith(".json"):
                    custom[fn[:-5]] = json.load(open(os.path.join(CHORD_SPEC_DIR, fn)))
        return JSONResponse({"canon": canon, "custom": custom, "lexicon": sim.lexicon})

    async def chord_render(request):
        spec = await request.json()
        buf = await asyncio.get_running_loop().run_in_executor(None, _render_spec, spec)
        return Response(_wav_bytes(buf), media_type="audio/wav")

    async def chord_save(request):
        body = await request.json()
        name = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(body.get("name", "")).strip())[:32]
        if not name or name in CHORD_WORDS:
            return JSONResponse({"ok": False, "error": "need a new name (canon words are read-only)"})
        spec = body.get("spec") or {}
        buf = await asyncio.get_running_loop().run_in_executor(None, _render_spec, spec)
        os.makedirs(CHORD_SPEC_DIR, exist_ok=True)
        os.makedirs(CHORD_CUSTOM_DIR, exist_ok=True)
        with open(os.path.join(CHORD_SPEC_DIR, f"{name}.json"), "w") as f:
            json.dump(spec, f, indent=1)
        with open(os.path.join(CHORD_CUSTOM_DIR, f"{name}.wav"), "wb") as f:
            f.write(_wav_bytes(buf))
        sim.reload_library()
        sim.log(f"chord word saved: {name} ({len(buf) / _chord_mod().SR:.2f} s)")
        return JSONResponse({"ok": True, "name": name, "lexicon": sim.lexicon})

    async def chord_delete(request):
        body = await request.json()
        name = str(body.get("name", ""))
        n = 0
        for p in (os.path.join(CHORD_SPEC_DIR, f"{name}.json"), os.path.join(CHORD_CUSTOM_DIR, f"{name}.wav")):
            if name and name not in CHORD_WORDS and os.path.exists(p):
                os.remove(p)
                n += 1
        sim.reload_library()
        return JSONResponse({"ok": n > 0, "lexicon": sim.lexicon})

    # ---------------------------------------------------------- D051: gesture studio
    async def gesture_list(_):
        kf = {k: kg.spec for k, kg in load_keyframe_gestures().items()}
        return JSONResponse({"code": [g for g in sim.gesture_names if g not in kf], "keyframe": kf,
                             "all": sim.gesture_names, "dir": os.path.relpath(GESTURE_DIR, ROOT)})

    async def gesture_save(request):
        spec = await request.json()
        try:
            path = save_keyframe_gesture(spec)
        except (ValueError, KeyError) as e:
            return JSONResponse({"ok": False, "error": str(e)})
        sim.reload_library()
        sim.log(f"gesture saved: {os.path.relpath(path, ROOT)}")
        return JSONResponse({"ok": True, "name": os.path.basename(path)[:-5], "all": sim.gesture_names})

    async def gesture_delete(request):
        body = await request.json()
        ok = delete_keyframe_gesture(str(body.get("name", "")))
        sim.reload_library()
        return JSONResponse({"ok": ok, "all": sim.gesture_names})

    async def gesture_preview(request):
        body = await request.json()
        if body.get("off"):
            await sim.call(sim.preview_off)
            return JSONResponse({"ok": True})
        spec = body.get("spec") or {}
        try:
            total = await sim.call(lambda: sim.preview_pose(spec, body.get("t")))
        except (ValueError, KeyError) as e:
            return JSONResponse({"ok": False, "error": str(e)})
        return JSONResponse({"ok": True, "total": total})

    async def gesture_play(request):
        """Play an UNSAVED keyframe spec once (the studio's ▶)."""
        spec = await request.json()
        try:
            kg = KeyframeGesture(spec)
        except (ValueError, KeyError) as e:
            return JSONResponse({"ok": False, "error": str(e)})

        def start():
            with sim.lock:
                sim.gesture = (kg, kg.total, sim.t)
                sim.cmd_v[:] = 0
            sim.mode = "gesturing"
        await sim.call(start)
        for t_cue, w in kg.cues:                       # chord cues fire on their frame
            asyncio.get_running_loop().call_later(t_cue / max(sim.speed, 0.05), lambda w=w: asyncio.ensure_future(sim.tool_say(w)))
        return JSONResponse({"ok": True, "total": kg.total})

    # ---------------------------------------------------------- D051: servo realism
    async def servo_set(request):
        body = await request.json()
        p = await sim.call(lambda: sim.servo.set(**body))
        sim.log(sim.servo.describe())
        return JSONResponse({"servo": p, "note": sim.servo.describe()})

    # ---------------------------------------------------------- D051: hardware
    def _hw_or_err():
        return sim.hw

    async def hw_status(_):
        hw = _hw_or_err()
        return JSONResponse({"connected": hw is not None, "ports": serial_ports(), "mirrors": list(MIRRORS),
                             "status": None if hw is None else hw.status()})

    async def hw_action(request):
        body = await request.json()
        act = str(body.get("action", ""))
        loop = asyncio.get_running_loop()
        try:
            if act == "connect":
                st = await loop.run_in_executor(None, sim.hw_connect, str(body.get("port", "mock")))
                return JSONResponse({"ok": True, "status": st})
            if act == "disconnect":
                return JSONResponse({"ok": await loop.run_in_executor(None, sim.hw_disconnect)})
            hw = _hw_or_err()
            if hw is None:
                return JSONResponse({"ok": False, "error": "not connected"})
            if act == "scan":
                found = await loop.run_in_executor(None, hw.scan)
                return JSONResponse({"ok": True, "found": {str(k): v for k, v in found.items()}, "status": hw.status()})
            if act == "mirror":
                return JSONResponse({"ok": True, "mirror": hw.set_mirror(str(body.get("mode", "off")))})
            if act == "torque":
                ids = body.get("ids")
                return JSONResponse({"ok": True, "ids": hw.torque(bool(body.get("on", True)), ids)})
            if act == "limp":
                return JSONResponse({"ok": True, "ids": hw.limp()})
            if act == "jog":
                return JSONResponse({"ok": True, "deg": hw.jog(int(body["id"]), float(body["deg"]), int(body.get("speed", 200)))})
            if act == "set_id":
                return JSONResponse({"ok": True, "id": hw.set_id(int(body["old"]), int(body["new"]))})
            if act == "center":
                return JSONResponse({"ok": True, **hw.center(str(body["key"]), body.get("jig_deg"))})
            if act == "dir":
                return JSONResponse({"ok": True, "dir": hw.set_dir(str(body["key"]), int(body.get("dir", 1)))})
            if act == "speed":
                hw.speed_cps = int(body.get("speed_cps", 0))
                return JSONResponse({"ok": True, "speed_cps": hw.speed_cps})
            return JSONResponse({"ok": False, "error": f"unknown action {act}"})
        except Exception as e:                          # a bus error is an answer, not a 500
            sim.log(f"hw {act}: {type(e).__name__}: {e}")
            return JSONResponse({"ok": False, "error": f"{type(e).__name__}: {e}"})

    async def help_(_):
        return JSONResponse({"teleop": TELEOP_HELP, "rl": HELP_RL,
                             "commands": __import__("playground").__doc__.split("Commands")[1].split("HONESTY")[0]})

    routes = [
        Route("/", index), Route("/api/state", state), Route("/api/events", events),
        Route("/video/{cam}.mjpg", video), Route("/frame/{cam}.jpg", frame),
        Route("/api/cmd", cmd, methods=["POST"]), Route("/api/teleop", teleop, methods=["POST"]),
        Route("/api/chat", chat, methods=["POST"]), Route("/api/chat/clear", chat_clear, methods=["POST"]),
        Route("/api/brain", brain, methods=["POST"]), Route("/api/models", models),
        Route("/api/world", world_get), Route("/api/world", world_set, methods=["POST"]),
        Route("/api/world/add", world_add, methods=["POST"]),
        Route("/api/rl/runs", rl_runs), Route("/api/rl/righter", rl_righter, methods=["POST"]),
        Route("/api/rl/curves.png", rl_curves),
        Route("/api/look", look, methods=["POST"]), Route("/api/shove", shove, methods=["POST"]),
        Route("/api/reset", reset, methods=["POST"]), Route("/api/speed", speed, methods=["POST"]),
        Route("/api/camera", camera, methods=["POST"]), Route("/api/scan", scan), Route("/api/tool/{name}", tool, methods=["POST"]),
        Route("/api/help", help_), Route("/favicon.ico", favicon),
        Route("/api/record", record, methods=["POST"]), Route("/api/recordings", recordings),
        Route("/recordings/{name}/{file}", rec_file), Route("/api/replay", replay, methods=["POST"]),
        Route("/api/world/saved", world_saved), Route("/api/world/save", world_save, methods=["POST"]),
        Route("/api/world/load", world_load, methods=["POST"]), Route("/api/world/random", world_random, methods=["POST"]),
        Route("/api/rl/walk", rl_walk, methods=["POST"]), Route("/api/voice", voice, methods=["POST"]),
        Route("/api/quit", quit_, methods=["POST"]),
        Route("/audio/{word}.wav", audio_file), Route("/api/audio", audio_mode, methods=["POST"]),
        Route("/api/chord/list", chord_list), Route("/api/chord/render", chord_render, methods=["POST"]),
        Route("/api/chord/save", chord_save, methods=["POST"]), Route("/api/chord/delete", chord_delete, methods=["POST"]),
        Route("/api/gesture/list", gesture_list), Route("/api/gesture/save", gesture_save, methods=["POST"]),
        Route("/api/gesture/delete", gesture_delete, methods=["POST"]), Route("/api/gesture/preview", gesture_preview, methods=["POST"]),
        Route("/api/gesture/play", gesture_play, methods=["POST"]),
        Route("/api/servo", servo_set, methods=["POST"]),
        Route("/api/hw", hw_status), Route("/api/hw", hw_action, methods=["POST"]),
    ]
    return Starlette(routes=routes)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=int(os.environ.get("ROCKY_COCKPIT_PORT", "8765")))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--world", default=os.environ.get("ROCKY_WORLD", "flat"))
    ap.add_argument("--brain", default="talk", choices=["talk", "local", "claude"])
    args = ap.parse_args(argv)
    import uvicorn
    sim = CockpitSim(args.world if args.world in PRESETS else "flat")
    sim.brain["mode"] = args.brain
    threading.Thread(target=sim.run_forever, daemon=True).start()
    app = make_app(sim)
    print(f"cockpit: http://{args.host}:{args.port}  (world {sim.world_name}; {sim.righter_note})", flush=True)
    # uvicorn's graceful shutdown waits for open connections, and this server's
    # connections are endless streams (MJPEG, SSE): a Ctrl-C or `cockpit-stop`
    # would hang and stale instances pile up. uvicorn installs its OWN
    # SIGINT/SIGTERM handlers when it starts (Server.capture_signals), so a
    # handler set before uvicorn.run() is silently replaced; the exit has to be
    # the server's handle_exit itself.
    server = uvicorn.Server(uvicorn.Config(app, host=args.host, port=args.port, log_level="warning"))
    server.handle_exit = lambda *_: (sim.hw_disconnect() if sim.hw is not None else None, os._exit(0))
    server.run()


if __name__ == "__main__":
    main()
