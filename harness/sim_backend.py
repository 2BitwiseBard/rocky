"""SimBackend — the Rocky-MCP contract bound to the MuJoCo cliff world.

This is the honest half of the harness: goto() steps real physics with the
real CliffDetector + ReflexSupervisor safe-stop (session 6 integration,
sim/run_cliff_safestop.py). A goto toward the void therefore exercises the
ACTUAL reflex stack, and stopped='cliff' means a simulated robot really
halted on a simulated table edge.

Scope (v0, deliberate):
  * goto blocks the tool call on real physics (runs in a worker thread so
    the event loop stays live; the mock covers async interleaving tests);
    planar targets, simple straight-line steering, 25 mm arrival window,
    20 s time cap.
  * scan_summary is NOT implemented here — the cliff world has no lidar
    model (absent tool > lying tool); use the mock or the sim_lidar
    binding when it lands (v0.1).
  * say/gesture validate against the real lexicons and log; rendering
    audio/gesture motion in-world is the B18/session-7 demo.

Laptop additions (2026-09-08, Tyler's machine — declared in NOTES_INBOX):
  * ROCKY_VIEWER=1 opens the passive MuJoCo window on this backend, so
    chat-driving (Claude over MCP, local_brain, intent) is WATCHABLE;
    motion then paces to real time instead of running CPU-fast. Fail-soft:
    no display -> a stderr note and the backend runs headless as before.
  * gesture() now renders the gesture in physics (same fn(gait, t) tables
    the playground uses) instead of only logging it; FELL is reported the
    way goto reports it.
  * ROCKY_AUDIO=1 plays the chord-speak sample for say() through
    aplay/ffplay when audio/samples_v2/<word>.wav exists.

D052 V2 (review) — HONESTY NOTE: this backend is NOT the D052 control loop
(no ServoModel, no WaveGait.budget, no void-probe, its own CliffDetector
loop); the cockpit (sim/cockpit.py, on sim.playground.Playground) is. What
it now shares: every joint target is NaN-guarded and rate-clamped (4.7 rad/s,
params 'hard'), and a stop / the end of a gesture ramps back to the stance at
the LOADED 3.0 rad/s instead of snapping in one physics step (119 deg for
fist_bump). AutoBackend tags every result with the backend it ran on.
Rebuilding this on Playground is the real fix (left for the owner).
"""
from __future__ import annotations

import asyncio
import os
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for sub in ("gait", "perception", "sim"):
    sys.path.insert(0, os.path.join(HERE, "..", sub))

from harness.backend import CHORD_WORDS, GESTURES, derived_capabilities   # noqa: E402


class SimBackend:
    def __init__(self, world=None):
        import mujoco                                  # noqa: F401
        from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS
        self._mujoco = __import__("mujoco")
        self._gait_mod = dict(WaveGait=WaveGait, leg_ik=leg_ik,
                              body_to_leg=body_to_leg, N_LEGS=N_LEGS)
        # session 8c: ROCKY_WORLD=flat gives an open floor for chat-driving
        # (the default cliff platform is a ~0.6 m test island — every goto
        # meets an edge, which is the point of the guard tests but a lousy
        # living room). The cliff detector stays armed either way; on the
        # flat floor it simply never fires.
        world = world or os.environ.get("ROCKY_WORLD", "cliff")
        self.world = world
        if world == "flat":
            sim_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "..", "sim")
            with open(os.path.join(sim_dir, "pebble.xml")) as f:
                self.model = mujoco.MjModel.from_xml_string(f.read())
            self.plat_h = 0.0
        elif world == "room":
            # session 8d: the sim_lidar room (walls + pillars + crate, all
            # lidar-visible group-3 geoms) — the patrol/scan playground
            from sim_lidar import build_room
            self.model = build_room()
            self.plat_h = 0.0
        else:
            from run_cliff import build_world, PLAT_H
            self.model = build_world()
            self.plat_h = PLAT_H
        self.events = []
        self.mode = "idle"
        self._stop_req = False
        self._reset()
        # laptop 2026-09-08: optional live window + speakers (both default off)
        self.viewer = None
        self._t_wall = 0.0
        if os.environ.get("ROCKY_VIEWER", "").lower() in ("1", "true", "yes"):
            try:
                import mujoco.viewer
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
                with self.viewer.lock():
                    self.viewer.opt.geomgroup[3] = 1     # room walls / obstacles are group 3
            except Exception as e:                       # no display, no GL…
                print(f"[sim_backend] viewer unavailable ({e}); running "
                      "headless", file=sys.stderr)
        self._player = None
        if os.environ.get("ROCKY_AUDIO", "").lower() in ("1", "true", "yes"):
            self._player = next((p for p in ("aplay", "ffplay")
                                 if shutil.which(p)), None)
        self._chord_dir = os.path.join(HERE, "..", "audio", "samples_v2")
        self._gestures = None

    def _reset(self):
        mujoco = self._mujoco
        g = self._gait_mod
        self.gait = g["WaveGait"]()
        self.data = mujoco.MjData(self.model)
        q0 = np.array([g["leg_ik"](g["body_to_leg"](i, self.gait.p_nom[i]))
                       for i in range(g["N_LEGS"])]).flatten()
        jadr = [self.model.joint(f"{n}{i}").qposadr[0]
                for i in range(5) for n in ("yaw", "hip", "knee")]
        import rocky_model as rm                     # D052: spawn from the loader, not +14 mm
        self.data.qpos[0:3] = [0, 0, rm.spawn_z_m(self.gait.h, platform_z_m=self.plat_h)]
        self.data.qpos[3:7] = [1, 0, 0, 0]
        self.data.qpos[jadr] = q0
        self.data.ctrl[:15] = q0
        mujoco.mj_forward(self.model, self.data)
        self._q0 = q0
        self._q_cmd = q0.copy()                      # last guarded target (V2)
        self.torso = self.model.body("torso").id

    def _guard(self, q, speed="hard"):
        """V2: NaN hold + rate clamp on the 15 leg targets (per physics step)."""
        import rocky_model as rm
        q = np.asarray(q, float).ravel()
        q = np.where(np.isfinite(q), q, self._q_cmd)
        step = rm.servo_speed(speed) * self.model.opt.timestep
        self._q_cmd = np.clip(q, self._q_cmd - step, self._q_cmd + step)
        return self._q_cmd

    # ------------------------------------------------------------- tools
    def capabilities(self) -> set:
        """D056: none (no eye, no scene memory, not a cockpit; F3: no places either — place
        recognition lives in a cockpit with it switched on): the MCP server offers the
        eight base tools on it."""
        return derived_capabilities(self)

    async def say(self, word: str) -> dict:
        if word not in CHORD_WORDS:
            return {"ok": False, "error": f"unknown chord word {word!r}",
                    "hint": f"lexicon: {', '.join(CHORD_WORDS)}"}
        self.events.append(("say", word))
        wav = os.path.join(self._chord_dir, f"{word}.wav")
        if self._player and os.path.exists(wav):
            cmd = (["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", wav]
                   if self._player == "ffplay" else [self._player, wav])
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            return {"ok": True, "word": word, "note": "played on the speakers"}
        return {"ok": True, "word": word, "note": "logged (ROCKY_AUDIO=1 + "
                "audio/samples_v2 rendered plays it on the speakers)"}

    async def gesture(self, name: str) -> dict:
        if name not in GESTURES:
            return {"ok": False, "error": f"unknown gesture {name!r}",
                    "hint": f"available: {', '.join(GESTURES)}"}
        if self.mode in ("walking", "gesturing"):
            return {"ok": False, "error": "busy",
                    "hint": "stop() first or wait for the goto"}
        self.events.append(("gesture", name))
        self.mode = "gesturing"
        try:
            return await asyncio.to_thread(self._gesture_blocking, name)
        finally:
            if self.mode == "gesturing":
                self.mode = "idle"

    async def goto(self, x: float, y: float) -> dict:
        if self.mode in ("walking", "gesturing"):
            return {"ok": False, "error": "busy",
                    "hint": "one motion intent at a time in the sim "
                            "backend (v0); stop() or wait"}
        self.mode = "walking"
        try:
            return await asyncio.to_thread(self._goto_blocking, x, y)
        finally:
            if self.mode == "walking":
                self.mode = "idle"

    def _goto_blocking(self, tx, ty):
        from pebble_reflex import ReflexSupervisor, body_gyro_xy
        from cliff import CliffDetector, CliffReaction
        from run_odom import foot_contacts
        mujoco = self._mujoco
        model, data, gait = self.model, self.data, self.gait
        det = CliffDetector()
        react = CliffReaction(gait)
        sup = ReflexSupervisor(gait)
        DT = model.opt.timestep
        V = 45.0
        t_settle, t_cap = 1.0, 20.0
        stop_sent = False
        outcome = None
        self._t_wall = time.monotonic()
        for k in range(int(t_cap / DT)):
            t = k * DT
            p = data.xpos[self.torso]
            if t < t_settle:
                data.ctrl[:15] = self._guard(self._q0, "loaded")
                mujoco.mj_step(model, data)
                self._sync_viewer(k)
                continue
            tw = t - t_settle
            dx, dy = tx - p[0], ty - p[1]
            dist = float(np.hypot(dx, dy))
            if outcome is None and dist < 0.025:
                outcome = ("arrived", tw)
                sup.request_stop()
            if self._stop_req and outcome is None:
                outcome = ("user", tw)
                sup.request_stop()
            ramp = min(tw / 0.6, 1.0)
            if outcome is None:
                ux, uy = (dx / dist, dy / dist) if dist > 1e-6 else (0, 0)
                # the gait takes BODY-frame velocities; the target is map
                # frame — rotate by the current yaw or any heading change
                # (turn_in_place, drift) walks the robot off-target
                Rm = data.xmat[self.torso].reshape(3, 3)
                yaw = float(np.arctan2(Rm[1, 0], Rm[0, 0]))
                cy, sy = np.cos(yaw), np.sin(yaw)
                ux, uy = cy * ux + sy * uy, -sy * ux + cy * uy
                vx, vy = V * ramp * ux, V * ramp * uy
                vx, vy, wz = react.command(tw, vx, vy, 0.0)
                halted = (react.retreat_until is not None and
                          tw >= react.retreat_until)
                if halted and not stop_sent:
                    stop_sent = True
                    outcome = ("cliff", tw)
                    sup.request_stop()
            else:
                vx, vy, wz = 0.0, 0.0, 0.0
            R = data.xmat[self.torso].reshape(3, 3)
            w_body = R.T @ data.cvel[self.torso][0:3]
            gxy = body_gyro_xy(w_body)
            con = foot_contacts(model, data)
            q, state = sup.step(tw, vx, vy, wz, gxy, contacts=con,
                                gyro_vec=w_body[:2])
            data.ctrl[:15] = self._guard(q)
            if k % 10 == 0 and state == "NORMAL" and outcome is None:
                ph = [(sup.t_gait / gait.T + gait.phase_off[i]) % 1.0
                      for i in range(5)]
                settled = [ph[i] < gait.duty and
                           0.12 < ph[i] / gait.duty < 0.95 for i in range(5)]
                if det.update(tw, settled, con):
                    react.on_void(tw)
                    self.events.append(("void", round(float(p[0]), 3)))
            mujoco.mj_step(model, data)
            self._sync_viewer(k)
            if self._fell():
                return {"ok": False, "stopped": "FELL", "pose": self._pose(),
                        "detail": "physics says no — this is a bug, report it"}
            # linger 1.5 s after an outcome so the safe-stop completes
            if outcome is not None and tw > outcome[1] + 1.5:
                break
        self._stop_req = False
        self.mode = "safe_stop" if outcome and outcome[0] != "arrived" \
            else "idle"
        reason = outcome[0] if outcome else "timeout"
        return {"ok": reason == "arrived", "stopped": reason,
                "pose": self._pose(),
                **({"detail": "VOID detected by the real detector; "
                              "PLANT->BRACE halt on the real physics"}
                   if reason == "cliff" else {})}

    async def stop(self) -> dict:
        self._stop_req = True
        if self.mode != "walking":
            self.mode = "safe_stop"
        return {"ok": True, "mode": self.mode}

    async def scan_summary(self) -> dict:
        """Session 8d: the sim_lidar ray fan, live, in ANY world. One 360°
        scan from the puck pose, summarized into 8 sectors (map frame,
        bearing 0° = +x, CCW). Honesty note baked into the result: a
        horizontal 2D scan physically cannot see a void below its plane —
        walls yes, cliffs no. Cliff safety is the foot-contact detector's
        job (D043), and this sensor does not pretend otherwise."""
        from sim_lidar import scan, RANGE_MAX
        mujoco = self._mujoco
        mujoco.mj_forward(self.model, self.data)
        angles, ranges, _origin = scan(self.model, self.data, self.torso)
        # angles are in the BODY frame plane; rotate bearings to map frame
        R = self.data.xmat[self.torso].reshape(3, 3)
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        bear = np.degrees((angles + yaw + np.pi) % (2 * np.pi) - np.pi)
        sectors = []
        for c in range(-180, 180, 45):
            m = (bear >= c) & (bear < c + 45)
            rr = ranges[m]
            hit = rr[np.isfinite(rr)]
            sectors.append({
                "bearing_deg": c + 22,
                "min_range_m": round(float(hit.min()), 3) if hit.size else None,
                "clear": bool(hit.size == 0)})
        finite = np.isfinite(ranges)
        out = {"ok": True, "n_points": int(len(ranges)),
               "range_max_m": float(RANGE_MAX), "world": self.world,
               "sectors": sectors,
               "frontiers": [{"bearing_deg": s["bearing_deg"]}
                             for s in sectors if s["clear"]],
               "note": "2D horizontal scan: sees walls/obstacles, can NOT "
                       "see voids below its plane — cliff safety is the "
                       "foot-contact guard, not this sensor"}
        if finite.any():
            i = int(np.argmin(np.where(finite, ranges, np.inf)))
            out["nearest_obstacle_m"] = round(float(ranges[i]), 3)
            out["nearest_obstacle_bearing_deg"] = round(float(bear[i]), 1)
        self.events.append(("scan", out.get("nearest_obstacle_m")))
        return out

    async def status(self) -> dict:
        p = self.data.xpos[self.torso]
        return {"ok": True, "mode": self.mode,
                "pose": {"x": round(float(p[0]), 3),
                         "y": round(float(p[1]), 3)},
                "battery_v": 11.9, "last_events": self.events[-5:]}

    def _pose(self):
        p = self.data.xpos[self.torso]
        return {"x": round(float(p[0]), 3), "y": round(float(p[1]), 3)}

    def _fell(self):
        """Same fall test goto has always used; sets mode='fallen'."""
        p = self.data.xpos[self.torso]
        z = self.data.xmat[self.torso].reshape(3, 3)[:, 2]
        tilt = np.rad2deg(np.arccos(np.clip(z[2], -1, 1)))
        if tilt > 50 or p[2] < self.plat_h - 0.02:
            self.mode = "fallen"
            return True
        return False

    # ------------------------------------------- laptop: viewer + gestures
    def _sync_viewer(self, k):
        """With the live window: refresh it (every 4th 500 Hz step is plenty)
        and hold the motion loop to real time — otherwise a 20 s goto is a
        0.3 s blur. Without it: no-op, physics runs CPU-fast as before."""
        v = self.viewer
        if v is None:
            return
        if not v.is_running():                 # user closed the window
            self.viewer = None
            return
        if k % 4 == 0:
            v.sync()
        self._t_wall += self.model.opt.timestep
        lag = self._t_wall - time.monotonic()
        if lag > 0:
            time.sleep(lag)

    def _gesture_table(self):
        if self._gestures is None:
            from pebble_gestures import (jazz_hands, fist_bump, beckon,
                                         JAZZ_TOTAL, BUMP_TOTAL, BECKON_TOTAL)
            from pebble_gestures2 import GESTURES2
            t = {"jazz_hands": (jazz_hands, JAZZ_TOTAL),
                 "fist_bump": (fist_bump, BUMP_TOTAL),
                 "beckon": (beckon, BECKON_TOTAL)}
            t.update({k: (fn, total) for k, (fn, total, _ev) in GESTURES2.items()})
            self._gestures = t
        return self._gestures

    def _gesture_blocking(self, name):
        """Play the gesture's joint-space script (run_gestures2's loop:
        settle -> fn(gait, t) -> tail on the planted pose). stop() cuts it
        short; a fall is reported like goto reports it."""
        mujoco = self._mujoco
        model, data = self.model, self.data
        fn, total = self._gesture_table()[name]
        DT = model.opt.timestep
        t_settle, t_tail = 0.5, 0.8
        has_claw = model.nu >= 20
        self._t_wall = time.monotonic()
        cut = False
        for k in range(int((t_settle + total + t_tail) / DT)):
            t = k * DT
            if self._stop_req and not cut:
                cut = True
            if cut or t < t_settle or t >= t_settle + total:
                data.ctrl[:15] = self._guard(self._q0, "loaded")      # V2: ramps back, no snap
                if has_claw:
                    data.ctrl[15:20] = 0.0
            else:
                q, claw = fn(self.gait, t - t_settle)
                data.ctrl[:15] = self._guard(q)
                if has_claw:
                    data.ctrl[15:20] = claw
            mujoco.mj_step(model, data)
            self._sync_viewer(k)
            if self._fell():
                return {"ok": False, "gesture": name, "stopped": "FELL",
                        "pose": self._pose(),
                        "detail": "physics says no — this is a bug, report it"}
        self._stop_req = False
        out = {"ok": not cut, "gesture": name, "duration_s": round(total, 1),
               "pose": self._pose(), "note": "rendered in physics"}
        if cut:
            out["stopped"] = "user"
        return out
