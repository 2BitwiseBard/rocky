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

from playground import Playground, GESTURES, TELEOP_HELP, HELP_RL      # noqa: E402
from world_builder import build as build_world, PRESETS, KINDS         # noqa: E402
from pebble_reflex import ReflexSupervisor                             # noqa: E402
from cliff import CliffDetector, CliffReaction                         # noqa: E402
from sim_lidar import scan as lidar_scan, RANGE_MAX                    # noqa: E402
from shove import Shove                                                # noqa: E402
from harness.backend import CHORD_WORDS, GESTURES as GESTURE_NAMES     # noqa: E402
from harness.intent import plan as intent_plan, execute as intent_execute   # noqa: E402
from harness.local_brain import TOOLS as BRAIN_TOOLS, SYSTEM as BRAIN_SYSTEM, OpenAIChat   # noqa: E402

V_GOTO = 45.0
LOOK_TOOL = {"type": "function", "function": {
    "name": "look",
    "description": "Look through the robot's eye camera: a vision model describes what "
                   "is in front of the robot (obstacles, open space, objects).",
    "parameters": {"type": "object", "properties": {}}}}
TOOLS = BRAIN_TOOLS + [LOOK_TOOL]
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
        self.cam = dict(distance=0.9, elevation=-18.0, azimuth=135.0)
        self.render_every = int(round(1 / (15 * self.DT)))      # ~15 fps
        self.last = dict(con=np.zeros(5, bool), tilt=0.0, height=0.0, gxy=0.0)
        self.chat_hist = {}
        self.brain = dict(mode="talk", model=None, vision_model=None)
        self.llm_base = os.environ.get("ROCKY_LLM_BASE_URL", "http://127.0.0.1:8080/v1")
        self.llm_key = _local_ai_key()
        self._llm = None
        self.log(f"world: {self.world_name} | {self.righter_note}")

    # ---------------------------------------------------------- plumbing
    def log(self, line):
        self.console.append((round(self.t, 2), str(line)))

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
        super().step()
        if gs is not None and self.goto_state is gs:
            self._goto_post(gs)
        if self._k % self.render_every == 0:
            self._render()

    # ------------------------------------------------------------- goto
    def _goto_pre(self, gs):
        tw = self.t - gs["t0"]
        p = self.data.xpos[self.torso]
        dx, dy = gs["tx"] - p[0], gs["ty"] - p[1]
        dist = float(np.hypot(dx, dy))
        if gs["outcome"] is None and dist < 0.025:
            gs["outcome"] = ("arrived", tw)
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
            if gs["det"].update(tw, settled, self.last["con"]):
                gs["react"].on_void(tw)
                self.events.append(("void", round(float(self.data.xpos[self.torso][0]), 3)))
                self.log("VOID detected — retreating")
        if gs["outcome"] is None and self.last["tilt"] > 60:
            gs["outcome"] = ("FELL", tw)
        if gs["outcome"] is not None and tw > gs["outcome"][1] + 1.5:
            reason = gs["outcome"][0]
            res = {"ok": reason == "arrived", "stopped": reason, "pose": self.pose()}
            if reason == "cliff":
                res["detail"] = "VOID detected by the real detector; PLANT->BRACE halt"
            if reason == "FELL":
                res["detail"] = "the robot fell during the goto; the righter takes over"
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
                               react=CliffReaction(self.gait), outcome=None, fut=fut, loop=loop)
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
            self._renderers = (chase, eye, cam, ecam)
        if self._renderers is False:
            return
        chase, eye, cam, ecam = self._renderers
        cam.lookat[:] = self.data.xpos[self.torso]
        cam.distance, cam.elevation, cam.azimuth = self.cam["distance"], self.cam["elevation"], self.cam["azimuth"]
        chase.update_scene(self.data, cam)
        self._put("chase", chase.render())
        eye.update_scene(self.data, ecam)
        self._put("eye", eye.render())

    def _put(self, name, rgb):
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=72)
        n = self.frames[name][0] + 1
        self.frames[name] = (n, buf.getvalue())

    # ------------------------------------------------------- world swaps
    def set_world(self, spec, name="custom"):
        """Sim thread. Rebuild the world; the robot respawns upright where it
        stands (or at the origin if that is now inside something)."""
        model, z0 = build_world(spec)
        old = self.data
        p_xy = old.xpos[self.torso][:2].copy()
        yaw = float(np.arctan2(old.xmat[self.torso].reshape(3, 3)[1, 0], old.xmat[self.torso].reshape(3, 3)[0, 0]))
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
        self._respawn(p_xy if name == self.world_name else np.zeros(2), yaw)
        self.righter_note = self._install_righter(ck)
        self.log(f"world: {name} ({model.ngeom} geoms) | {self.righter_note}")

    def _respawn(self, xy=(0.0, 0.0), yaw=0.0):
        from pebble_gait import leg_ik, body_to_leg
        q0 = np.array([leg_ik(body_to_leg(i, self.gait.p_nom[i])) for i in range(5)]).flatten()
        jadr = [self.model.joint(f"{n}{i}").qposadr[0] for i in range(5) for n in ("yaw", "hip", "knee")]
        self.data.qpos[:] = 0
        self.data.qvel[:] = 0
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
                    goto=None if gs is None else [gs["tx"], gs["ty"]], gesture=self.gesture is not None,
                    events=list(self.events)[-12:], console=list(self.console)[-40:],
                    brain=self.brain, gait=dict(T=self.gait.T, h=self.gait.h, R0=self.gait.R0,
                                                duty=self.gait.duty, hstep=self.gait.hstep),
                    reflex=dict(trip=self.sup.gyro_trip, stall_s=self.sup.stall_s,
                                fallen_max_s=self.sup.fallen_max_s))

    async def tool_say(self, word):
        if word not in CHORD_WORDS:
            return {"ok": False, "error": f"unknown chord word {word!r}", "hint": f"lexicon: {', '.join(CHORD_WORDS)}"}
        self.events.append(("say", word))
        r = await self.call(lambda: self.do(f"say {word}"))
        return {"ok": True, "word": word, "note": r}

    async def tool_gesture(self, name):
        if name not in GESTURE_NAMES or name not in GESTURES:
            return {"ok": False, "error": f"unknown gesture {name!r}", "hint": f"available: {', '.join(GESTURE_NAMES)}"}
        if self.goto_state is not None or float(np.abs(self.cmd_v).sum()) > 0:
            return {"ok": False, "error": "busy", "hint": "walking; stop() first"}
        fn, total = GESTURES[name]
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
            return JSONResponse({"reply": "(use the browser's close button; quit is not a cockpit command)"})
        r = await sim.call(lambda: sim.do(line))
        sim.log(f"> {line}")
        if r:
            for ln in str(r).splitlines():
                sim.log(ln)
        return JSONResponse({"reply": r})

    async def teleop(request):
        body = await request.json()
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

    async def camera(request):
        body = await request.json()
        for k in ("distance", "elevation", "azimuth"):
            if k in body:
                sim.cam[k] = float(body[k])
        return JSONResponse(sim.cam)

    async def tool(request):
        name = request.path_params["name"]
        try:
            args = await request.json()
        except Exception:
            args = {}
        return JSONResponse(await sim.tool(name, args or {}))

    async def favicon(_):
        return Response(b"", status_code=204)

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
        Route("/api/camera", camera, methods=["POST"]), Route("/api/tool/{name}", tool, methods=["POST"]),
        Route("/api/help", help_), Route("/favicon.ico", favicon),
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
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
