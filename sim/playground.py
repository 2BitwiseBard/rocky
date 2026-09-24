"""Pebble PLAYGROUND — interactive sim: live viewer + command REPL.

The "drive it before it exists" tool (session 6c). One process runs the
physics with the real gait + real reflex supervisor; you type commands
while it runs. On a laptop with a display you get the native MuJoCo
window; headless (CI, cloud) the same commands run scripted.

    MUJOCO_GL=glfw  python3 playground.py --viewer          # laptop
    python3 playground.py --script "walk 45; wait 3; stop; quit"   # headless
    python3 playground.py --cliff --viewer                  # cliff world

Commands (REPL or --script, ';'-separated):
    walk VX [VY] [WZ]   walk (mm/s, mm/s, rad/s) — adjust any time. Every
                        command is fitted into the gait's speed envelope
                        (WaveGait.budget, D052): `show` says when it was scaled
    stop                safe-stop: PLANT -> BRACE -> planted idle (D034);
                        also ends a running gesture (blended out)
    gesture NAME        jazz_hands | fist_bump | beckon | wave | bow | look_around |
                        shake | sit | turn_in_place | sidestep | any saved keyframe
                        gesture. Needs a planted standstill (state NORMAL, no
                        walk). Blended in and out at <= 3 rad/s; the reflex
                        keeps watching, and a fall drops the gesture (D052)
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
                        reflex.fallen_max_s servo.on servo.hold_hz
                        servo.latency_s servo.rate_rad_s servo.quant
                        (phase-continuous: changing T rescales the clock so
                        feet don't teleport)
    gait NAME           load a gait preset (gait/gaits/NAME.json; `default`
                        = params.yaml). gait save NAME | gait list
    check [NAME]        feasibility report (pebble_feasibility): no NAME = the
                        gait at the current command and at the envelope
                        corners; NAME = that gesture
    clear               release a latched void (cliff guard) and a latched
                        safe-stop (3 gyro trips in 5 s) — D052
    probe on|off        the stance contact probe (feet feel for the floor)
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

ALWAYS-ON GUARDS (D052 — every command source goes through them: walk,
teleop, the cockpit's goto and residual walker, the hardware mirror):
    void guard          a stance foot that probes 30 mm and finds nothing
                        while walking = a void: back away, safe-stop, and
                        refuse any command with a component toward it
                        ("blocked: void at N deg") until `clear`
    trip escalation     3 gyro trips in 5 s latch a safe-stop; velocity is
                        ignored until `clear`
    servo realism       ON: 50 Hz bus hold, 20 ms latency, 4.7 rad/s slew,
                        4096-count goals (`set servo.on 0` = ideal actuators);
                        every joint target is rate-clamped at 4.7 rad/s and a
                        non-finite target is dropped (last good one held)
    hardware            while mirroring sim->real: no probe on the real feet,
                        no walking (velocity OR a gaited gesture) until the
                        bridge allows locomotion, no gesture during its soft
                        entry, no push / gait change; the mirror is DROPPED
                        (real legs hold) when the sim's reflex leaves NORMAL
                        or the sim is respawned — the sim's reaction to an
                        event only the sim lived through never reaches the
                        real legs (D052 V2)

HONESTY BOX (read before trusting a number): this is the same MJCF the
whole sim stack uses — masses from CAD, D052 servo identity (damping =
stall / no-load, forcerange = continuous torque) and the servo realism
layer above, friction guessed, no gear backlash. It is EXCELLENT for logic
(does the reflex trip? does the gesture reach? does a gait tweak break the
sweep limits?) and DIRECTIONAL for dynamics (push envelopes, stability
trends). It is NOT calibrated — that happens when a real servo answers
back (bench day: step response, stall, real masses -> MJCF update). Treat
absolute numbers as ±real-robot-TBD.
"""
import argparse
import json
import os
import queue
import re
import shutil
import subprocess
import sys
import threading
import time
from collections import deque

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
sys.path.insert(0, HERE)
import rocky_model as rm                                              # noqa: E402
import pebble_feasibility as pf                                       # noqa: E402
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS        # noqa: E402
from pebble_reflex import ReflexSupervisor, body_gyro_xy             # noqa: E402,F401
from pebble_gestures import (jazz_hands, fist_bump, beckon,          # noqa: E402
                             JAZZ_TOTAL, BUMP_TOTAL, BECKON_TOTAL)
from contacts import foot_contacts                                    # noqa: E402
from cliff import CliffDetector, CliffReaction                        # noqa: E402
from sim_imu import SimIMU                                            # noqa: E402
from shove import Shove                                               # noqa: E402

from pebble_gestures2 import GESTURES2, GAITED, kind_of              # noqa: E402
from pebble_keyframes import load_keyframe_gestures                 # noqa: E402
from servo_model import ServoModel                                   # noqa: E402
GESTURES = {"jazz_hands": (jazz_hands, JAZZ_TOTAL),
            "fist_bump": (fist_bump, BUMP_TOTAL),
            "beckon": (beckon, BECKON_TOTAL)}
GESTURES.update({k: (fn, total) for k, (fn, total, _ev) in GESTURES2.items()})  # session 8: v2 library
CHORD_DIR = os.path.join(HERE, "..", "audio", "samples_v2")
CHORD_CUSTOM_DIR = os.path.join(HERE, "..", "audio", "samples_custom")   # D051: words made in the cockpit
GAIT_DIR = os.path.join(HERE, "..", "gait", "gaits")                     # D052: gait presets as data
GAIT_KEYS = ("T", "h", "R0", "duty", "hstep")


def all_gestures():
    """Code gestures + the keyframe gestures on disk (gait/gestures/*.json, D051)."""
    g = dict(GESTURES)
    g.update({k: (kg, kg.total) for k, kg in load_keyframe_gestures().items()})
    return g


def chord_wav(word):
    """Path of a chord-speak word's wav (canon lexicon first, then custom), or None."""
    for d in (CHORD_DIR, CHORD_CUSTOM_DIR):
        p = os.path.join(d, f"{word}.wav")
        if os.path.exists(p):
            return p
    return None


def lexicon():
    words = []
    for d in (CHORD_DIR, CHORD_CUSTOM_DIR):
        if os.path.isdir(d):
            words += sorted(x[:-4] for x in os.listdir(d) if x.endswith(".wav") and not x.startswith("demo_reel"))
    return words


def find_player():
    for p in ("aplay", "afplay", "ffplay"):
        if shutil.which(p):
            return p
    return None


# ------------------------------------------------------------------ gait presets
def gait_presets():
    """name -> {T, h, R0, duty, hstep}: `default` from params.yaml plus every
    gait/gaits/*.json (a file named default.json would override params)."""
    d = rm.gait_defaults()
    out = {"default": dict(T=d["cycle_time"], h=d["body_height"], R0=d["stance_radius"],
                           duty=d["duty"], hstep=d["step_height"])}
    if os.path.isdir(GAIT_DIR):
        for f in sorted(os.listdir(GAIT_DIR)):
            if not f.endswith(".json"):
                continue
            try:
                with open(os.path.join(GAIT_DIR, f)) as fh:
                    spec = json.load(fh)
                out[f[:-5]] = {k: float(spec[k]) for k in GAIT_KEYS}
            except (OSError, ValueError, KeyError, TypeError):
                continue                               # a broken preset is skipped, not fatal
    return out


def save_gait_preset(name, gait, note=None):
    """Write gait/gaits/NAME.json from a WaveGait; returns the path."""
    if not re.fullmatch(r"[A-Za-z0-9_\-]{1,40}", name or "") or name == "default":
        raise ValueError("preset name: 1-40 of [A-Za-z0-9_-], not 'default'")
    os.makedirs(GAIT_DIR, exist_ok=True)
    spec = {k: float(getattr(gait, k)) for k in GAIT_KEYS}
    spec["note"] = note or f"saved from the playground (params {rm.params_rev()})"
    path = os.path.join(GAIT_DIR, f"{name}.json")
    with open(path, "w") as fh:
        json.dump(spec, fh, indent=2)
        fh.write("\n")
    return path


# ------------------------------------------------------------------ constants
# D050 contact-seeking stance, rebuilt as a per-leg state machine (D052). The
# D050 probe was a bang-bang force regulator: lower while the switch is open,
# relax at 2x the rate the moment it closes — so a probed foot chattered on
# the switch threshold and carried almost no load, and the offsets vanished
# the moment the reflex left NORMAL. Now, per stance:
#   SWING   -> (touchdown) SEEK: lower at PROBE_RATE until the switch has been
#   closed for PROBE_CONFIRM_TICKS consecutive 50 Hz ticks
#   -> PRELOAD: PROBE_PRELOAD_MM further (about half the ~7 mm SEA travel) so
#   the foot actually carries load -> HOLD for the rest of the stance (a
#   contact loss inside the stance never retracts it) -> (liftoff) SWING,
#   offset ramps out at PROBE_RELAX_RATE.
PROBE_RATE = 120.0          # mm/s a contactless planted foot is lowered
PROBE_MAX = 30.0            # mm: probed this far with nothing under it = the ground is not there
PROBE_PRELOAD_MM = 3.5      # mm past first contact (half the SEA's ~7 mm travel)
PROBE_PRELOAD_MIN_MM = 3.0  # preload only a foot that had to SEEK this far: a foot that lands
#                             on the nominal plane is already loaded by the stance geometry
#                             (without this, flat-ground walking pushed every foot 3.5 mm deep)
PROBE_CONFIRM_TICKS = 2     # 50 Hz ticks of closed switch before the seek ends
PROBE_LEAD_MM = 3.5         # the SEEK command may lead the MEASURED foot (encoder FK) by at most
#                             this much. Measured 2026-09-24 on a 10 mm step-down: an open-loop
#                             120 mm/s seek overshot the real foot by 7 mm with ideal actuators and
#                             12 mm with the servo model (latency + slew + D052 damping), so the
#                             "3.5 mm preload" became 15 mm, 11 N on one foot and a 2 deg body tilt.
#                             Leading by <= the preload makes the overshoot the preload.
PROBE_SETTLE_FRAC = 0.12    # of the stance: wait this long after the COMMANDED touchdown before
#                             lowering. Measured 2026-09-24 (servo realism on, flat floor): the
#                             switch closes 4-5 ticks (80-100 ms) after the commanded touchdown —
#                             bus hold + latency + slew. Seeking at once lowered every flat-ground
#                             foot ~20 mm, which lifted the body and made the next foot seek too
#                             (a ratchet). 0.12 is the same settling margin the cliff detector uses.
PROBE_RELAX_RATE = 240.0    # mm/s the offset ramps out during swing
PROBE_TICK_S = 0.02         # the probe runs at the bus rate, like the Pi will
SEEK, PRELOAD, HOLD, SWING = "SEEK", "PRELOAD", "HOLD", "SWING"

GESTURE_BLEND_MIN_S = 0.3   # entry/exit blend floor (D052)
VOID_RETREAT_CYCLES = 0.6   # the always-on guard backs off 0.6 gait cycles (goto keeps 1.6)
VOID_RETREAT_MIN = 25.0     # mm/s toward-speed assumed when the void was found turning
VOID_TOL = 1.0              # mm/s: a command component toward a latched void above this is refused
NAN_LOG_S = 1.0             # throttle for the non-finite-target note
SHOVE_MAX_N = 200.0         # D052 V2: a console/cockpit shove is clamped to this peak (the tip
#                             threshold is ~25-35 N; 1e9 N blew MuJoCo up and auto-reset the sim)
SHOVE_DUR_S = (0.05, 2.0)   # and to this duration range
GAIT_T_RANGE = (0.4, 10.0)  # D052 V2: `set gait.T 0` divided by zero in every phase reader and
GAIT_DUTY_RANGE = (0.5, 0.95)   # made the cockpit unrecoverable; these are sanity bounds, not tuning
GAIT_HSTEP_RANGE = (0.0, 80.0)  # (check / pebble_feasibility says whether a value is actually good)


def is_gaited(name=None, fn=None):
    """True for a gesture that WALKS (runs the wave gait: its planted feet move):
    turn_in_place / sidestep, their signed variants (D2's `turn_in_place_right`)
    and any fn tagged `gaited = True`. The sim2real locomotion hold covers these
    exactly like a velocity command (D052 V2 — before, only cmd_v was held)."""
    if fn is not None and getattr(fn, "gaited", False):
        return True
    for n in (name, getattr(fn, "__name__", None)):
        if n and any(n == g or n.startswith(g + "_") for g in GAITED):
            return True
    return False


def shove_args(fx, fy, dur):
    """(fx, fy, dur) checked for a shove: finite, peak clamped to SHOVE_MAX_N,
    dur clamped to SHOVE_DUR_S. Raises ValueError on a non-finite value."""
    v = np.array([fx, fy, dur], float)
    if not np.isfinite(v).all():
        raise ValueError("shove needs finite fx, fy, dur")
    f = float(np.hypot(v[0], v[1]))
    k = min(1.0, SHOVE_MAX_N / f) if f > 0 else 1.0
    return float(v[0] * k), float(v[1] * k), float(np.clip(v[2], *SHOVE_DUR_S))


def _smooth(a):
    a = min(max(a, 0.0), 1.0)
    return a * a * (3 - 2 * a)


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def blend_time(q_from, q_to, min_s=GESTURE_BLEND_MIN_S):
    """Smoothstep duration (s) whose PEAK joint speed (1.5 x mean) stays under
    the loaded budget (params joints.vel_rad_s.loaded, 3.0 rad/s). Same rule as
    ReflexSupervisor.ramp_time — planted legs carry the body through a blend."""
    dq = np.abs(np.asarray(q_to, float).ravel() - np.asarray(q_from, float).ravel())
    dq = float(np.nanmax(dq)) if dq.size and np.isfinite(dq).any() else 0.0
    return max(min_s, 1.5 * dq / rm.servo_speed("loaded"))


def kinematic_height_m(q, contacts, grav_body):
    """Torso height (m) the ROBOT can compute: max over the feet whose switch
    is closed of the foot's depth below the torso, measured along 'up' from
    the IMU gravity vector (FK of the joint state, rotated by the attitude).
    0.0 when no foot touches. The supervisor's handoff uses this, not world z."""
    con = np.asarray(contacts, bool)
    if not con.any():
        return 0.0
    g = np.asarray(grav_body, float)
    up = -g / max(float(np.linalg.norm(g)), 1e-9)
    P = pf.feet_body(np.asarray(q, float).reshape(N_LEGS, 3))       # mm, body frame
    depth = -(P @ up)
    return float(depth[con].max()) / 1000.0


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
        # D052: spawn from the model loader, not a hard-coded +14 mm
        self.data.qpos[0:3] = [0, 0, rm.spawn_z_m(self.gait.h, platform_z_m=z0)]
        self.data.qpos[3:7] = [1, 0, 0, 0]
        self.data.qpos[jadr] = q0
        self.data.ctrl[:15] = q0
        mujoco.mj_forward(self.model, self.data)
        self.torso = self.model.body("torso").id
        self.DT = self.model.opt.timestep
        self.lock = threading.Lock()
        self.cmd_v = np.zeros(3)              # vx, vy, wz — what was ASKED (walk/teleop/goto)
        self.cmd_eff = np.zeros(3)            # D052: what the gait got after the guards + budget
        self.gesture = None                   # (fn, total, t0) — a request; see _gesture_sync
        self.push = None                      # shove.Shove or None
        self.residual = None                  # (15,) rad added to the joint targets (D050 walk policy)
        self.probe_on = True                  # D050: planted feet without contact seek the ground
        self.probe_dz = np.zeros(N_LEGS)      # mm each foot is currently lowered below its target
        self.probe_out = np.zeros(N_LEGS, bool)   # probed PROBE_MAX with no contact (the void signal)
        self.probe_state = [SWING] * N_LEGS
        self.notes = deque(maxlen=200)        # D052: (t, kind, msg) guard/gesture events for UIs
        self.note_seq = 0                     # total notes ever (a reader keeps its own cursor)
        self.righter_note = self._install_righter()
        self.gestures = all_gestures()          # D051: includes keyframe gestures from gait/gestures/
        # D052: servo realism ON by default (was off) — the sim stops flattering the servo
        self.servo = ServoModel(on=True)
        self.hw = None                          # D051: HardwareBridge when the real bus is connected
        self.said = (0, None)                   # (count, word) — the cockpit plays it in the browser
        self.t = 0.0
        self.player = find_player()
        self.recording = False
        self._frames = []
        self._renderer = None
        self._spf = int(round(1 / (30 * self.DT)))
        self._k = 0
        self.alive = True
        self.nan_count = 0
        self._nan_t = -1e9
        self._budget_cache = {}
        self.budget_k = 1.0
        self._ges = None                        # the running gesture machine (in/run/out)
        self._hw_seen = None
        self._loco_note_t = -1e9
        self._model_seen = self._data_seen = self._sup_seen = None
        self.reset_guards()
        self.last = dict(con=np.zeros(N_LEGS, bool), tilt=0.0, height=float(self.data.xpos[self.torso][2]),
                         kin_h=0.0, gxy=0.0, grav=np.array([0.0, 0.0, -1.0]), gyro=np.zeros(3))

    # ------------------------------------------------------------ plumbing
    def note(self, kind, msg):
        """A guard/gesture event for whoever is watching (the cockpit may
        override this to route into its events + console). Never blocks."""
        self.notes.append((round(self.t, 2), kind, str(msg)))
        self.note_seq += 1

    def reset_guards(self):
        """Re-derive every per-model/per-spawn piece of guard state from the
        CURRENT model/data/sup: IMU, contact switch memory, probe, cliff guard,
        gesture machine, rate-clamp memory and the servo model. Called at init
        and automatically when the cockpit swaps model/data/sup (a world
        change or a respawn); calling it explicitly is also fine."""
        m = self.model
        self.torso = m.body("torso").id
        self.DT = m.opt.timestep
        self._jadr = np.array([m.joint(f"{n}{i}").qposadr[0] for i in range(N_LEGS)
                               for n in ("yaw", "hip", "knee")])
        self.fids = [m.geom(f"foot{i}").id for i in range(N_LEGS)]
        self.imu = SimIMU(m, self.torso)          # ideal; gravity from model.opt.gravity
        self._con_state = np.zeros(N_LEGS, bool)  # switch hysteresis memory
        self.probe_dz = np.zeros(N_LEGS)
        self.probe_out = np.zeros(N_LEGS, bool)
        self.probe_state = [SWING] * N_LEGS
        self._probe_hits = np.zeros(N_LEGS, int)
        self._probe_goal = np.zeros(N_LEGS)
        self._probe_age = np.zeros(N_LEGS, int)       # 50 Hz ticks since this stance's touchdown
        self.cliff_det = CliffDetector()
        self.cliff_react = CliffReaction(self.gait, retreat_cycles=VOID_RETREAT_CYCLES)
        self.void = None
        self._void_bearings = []                  # world-frame bearings (deg) of latched voids
        self._void_phase = None                   # None | 'retreat' | 'held'
        self._void_speed = 0.0
        self._ges = None
        self._latch_seen = bool(self.sup.latched)
        self._q_cmd = self.data.ctrl[:15].copy()  # last good, rate-clamped joint target
        self._claw_cmd = self.data.ctrl[15:20].copy() if m.nu >= 20 else None
        self.servo.reset(self._q_cmd)
        self._model_seen, self._data_seen, self._sup_seen = m, self.data, self.sup

    def _sync_external(self):
        """The cockpit replaces model/data (set_world) or the supervisor
        (_respawn) behind our back: resync the guards when it does. D052 V2:
        while mirroring sim->real the mirror is dropped FIRST (the real legs
        hold their last goal) — reset_guards re-seeds the rate clamp from the
        new spawn, so the next stream would jump the real legs to it."""
        if (self.model is not self._model_seen or self.data is not self._data_seen
                or self.sup is not self._sup_seen):
            if self.sim2real:
                self._hw_drop("the sim was respawned (reset / world change)")
            with self.lock:
                if self._ges is not None and self.gesture is self._ges["src"]:
                    self.gesture = None
            self.reset_guards()

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

    # ------------------------------------------------------------ status
    def yaw(self):
        R = self.data.xmat[self.torso].reshape(3, 3)
        return float(np.arctan2(R[1, 0], R[0, 0]))

    @property
    def sim2real(self):
        hw = self.hw
        return hw is not None and getattr(hw, "mirror", "off") == "sim2real"

    def _hw_drop(self, why):
        """D052 V2: end sim2real — the real legs hold their last goal (torque
        stays on). For sim-only events the real robot did not live through: a
        reflex reaction to a sim shove or fall (the righter's stream is
        open-loop on the real legs), a respawn, a world change."""
        hw = self.hw
        try:
            hw.set_mirror("off")
        except Exception as e:                        # noqa: BLE001 — a lost port already went off
            self.note("hw", f"mirror off failed ({type(e).__name__}: {e})")
        self.note("hw", f"sim2real dropped: {why} — the real legs hold their last goal")

    def entry_active(self):
        """True while the bridge's soft entry is still blending a leg in."""
        hw = self.hw
        return self.sim2real and bool(getattr(hw, "_blend", None))

    def locomotion_held(self):
        """True while the hardware mirror forbids walking (sim2real without
        real foot contacts, or during a soft entry) — D052 / hw_bridge."""
        hw = self.hw
        return self.sim2real and not bool(getattr(hw, "allow_locomotion", False))

    def is_idle(self):
        """What the hardware bridge may start sim2real from: planted standstill
        (state NORMAL, no velocity asked, no gesture or blend, no goto)."""
        return (self.sup.state == "NORMAL" and not np.any(self.cmd_v)
                and self.gesture is None and self._ges is None
                and getattr(self, "goto_state", None) is None)

    @property
    def gesture_phase(self):
        """None | 'in' (entry blend) | 'run' | 'out' (exit blend)."""
        return None if self._ges is None else self._ges["phase"]

    @property
    def gesture_busy(self):
        """True from the gesture request until the exit blend has finished —
        wait on this (not on `gesture is None`) before commanding anything."""
        return self._ges is not None or self.gesture is not None

    @property
    def V_MAX(self):
        """Teleop per-axis caps (mm/s, mm/s, rad/s) = the gait's envelope (D052)."""
        mc = self.gait.max_command()
        return np.array([mc["vx"], mc["vy"], mc["wz"]])

    def budget_note(self):
        """'' when the last command fit the envelope, else what scaled it."""
        return self._budget_words(self.budget_k)

    def _budget_words(self, k):
        if k >= 0.999:
            return ""
        lim = self.gait.vf_limit()
        which = ("coxa", "swing-speed", "lift-speed")[int(np.argmin(lim))]
        if k < 1e-3:
            return (f"the {which} budget allows NO motion on this gait "
                    f"(its step alone is too fast: raise T or lower hstep) — cmd zeroed")
        return f"cmd scaled to {k:.2f}x by the {which} budget"

    def guard_status(self):
        """JSON-safe summary of every D052 guard (for a HUD / the cockpit)."""
        v = self.void
        return dict(
            void=None if v is None else {k: (round(float(x), 2) if isinstance(x, (float, np.floating)) else x)
                                         for k, x in v.items() if k != "xy"},
            void_phase=self._void_phase, latched=bool(self.sup.latched), latch_reason=self.sup.latch_reason,
            locomotion_held=self.locomotion_held(), budget_k=round(float(self.budget_k), 3),
            budget_note=self.budget_note(), cmd_eff=[round(float(x), 3) for x in self.cmd_eff],
            probe_on=bool(self.probe_on), probe_state=list(self.probe_state),
            probe_dz=[round(float(x), 1) for x in self.probe_dz], probe_out=[bool(x) for x in self.probe_out],
            gesture_phase=self.gesture_phase, nan_count=int(self.nan_count),
            servo_on=bool(self.servo.p["on"]), kin_h=round(float(self.last.get("kin_h", 0.0)), 4))

    # ------------------------------------------------------------ physics
    def step(self):
        """One physics step; call from the run loop."""
        self._sync_external()
        with self.lock:
            v = self.cmd_v.copy()
            ges_req = self.gesture
            push = self.push
        d, m = self.data, self.model
        # --- sensors: what the robot can measure (D052) ------------------
        r = self.imu.read(d)
        grav, w_body = r["grav"], r["gyro"]
        tilt_deg = float(np.degrees(r["tilt"]))
        gxy = float(np.hypot(w_body[0], w_body[1]))
        con = foot_contacts(m, d, self.fids, self._con_state)
        q_meas = d.qpos[self._jadr].reshape(N_LEGS, 3)
        kin_h = kinematic_height_m(q_meas, con, grav)
        self.last = dict(con=con, tilt=tilt_deg, height=float(d.xpos[self.torso][2]),   # height: world z, HUD only
                         kin_h=kin_h, gxy=gxy, grav=grav, gyro=w_body)
        # --- hardware gating ---------------------------------------------
        hw = self.hw
        if hw is not None and hw is not self._hw_seen:
            self._hw_seen = hw
            if not getattr(hw, "_pg_idle", False) and hasattr(hw, "is_idle"):
                prev = hw.is_idle
                hw.is_idle = lambda prev=prev: bool(prev()) and self.is_idle()
                hw._pg_idle = True
        if self.locomotion_held() and np.any(v):
            with self.lock:
                self.cmd_v[:] = 0
            v = np.zeros(3)
            if self.t - self._loco_note_t > 1.0:
                self._loco_note_t = self.t
                self.note("hw", "locomotion held: sim2real without real foot contacts — velocity zeroed")
        # --- gesture requests (start / switch / external end) ------------
        self._gesture_sync(ges_req)
        monitor = self._ges is not None
        # --- velocity pipeline: latch -> void guard -> envelope ----------
        v_eff = self._effective_cmd(v)
        v_sup = np.zeros(3) if monitor else v_eff
        # --- stance probe (not on the real feet, not during a gesture) ----
        state0 = self.sup.state
        probing = (self.probe_on and not self.sim2real and not monitor
                   and state0 in ("NORMAL", "PLANT", "RECOVER", "BRACE"))
        if not probing:
            self._probe_reset()
        elif state0 != "BRACE" and self._k % int(round(PROBE_TICK_S / self.DT)) == 0:
            self._probe_tick(con, self.sup.last_stance, q_meas)
        q, state = self.sup.step(self.t, v_sup[0], v_sup[1], v_sup[2], gxy,
                                 contacts=con, gyro_vec=w_body[:2], tilt_deg=tilt_deg,
                                 height=kin_h, monitor=monitor,
                                 probe_dz=self.probe_dz.copy() if probing else None,
                                 q_meas=q_meas)
        if self.sup.latched != self._latch_seen:
            self._latch_seen = self.sup.latched
            if self.sup.latched:                 # D052 trip escalation: the ask is dropped too, so
                with self.lock:                  # `clear` does not resume the old walk by surprise
                    self.cmd_v[:] = 0
                self.note("latch", f"LATCHED safe-stop: {self.sup.latch_reason} — `clear` to release")
        if self.sim2real and state != "NORMAL":
            # D052 V2: the sim's reflex is reacting to something only the SIM lived
            # through (locomotion is held, so a trip/fall here is a sim shove, a
            # sim collision or a stop) — streaming its PLANT/BRACE/RECOVER or the
            # righter to the real legs would run that reaction open-loop on them
            self._hw_drop(f"the sim's reflex went {state}")
        claw = None
        if monitor and state in ("FALLEN", "RIGHTED"):
            self._abort_gesture(f"aborted: {state} — the righter has the legs")
            monitor = False
        if self._ges is not None:
            q, claw = self._gesture_targets(q)
        elif self.residual is not None and state == "NORMAL":
            q = np.asarray(q, float).reshape(N_LEGS, 3) + np.asarray(self.residual, float).reshape(N_LEGS, 3)
        # --- always-on void guard (after the probe + supervisor) ---------
        self._void_update(con, state)
        target = np.asarray(q, float).ravel().copy()
        if hw is not None and getattr(hw, "mirror", "off") == "real2sim":
            q_real, legs = hw.real_pose()
            for i in range(N_LEGS):
                if legs[i]:
                    target[3 * i:3 * i + 3] = q_real[i]
        target = self._guard_target(target)
        if hw is not None:                    # D051: the real bus beside the sim — gets the guarded stream
            hw.push_targets(target)
        d.ctrl[:15] = self.servo.filter(target, self.DT, force=d.actuator_force[:15])
        if claw is not None and m.nu >= 20:
            d.ctrl[15:20] = self._guard_claw(claw)
        if push is not None:
            if not push.apply(m, d, self.torso, self.t):
                with self.lock:               # over: wrench already zeroed
                    self.push = None
        mujoco.mj_step(m, d)
        self.t += self.DT
        self._k += 1
        if self.recording and self._k % self._spf == 0:
            if self._renderer is None:
                self._renderer = mujoco.Renderer(m, 480, 720)
                self._cam = mujoco.MjvCamera()
                self._cam.distance, self._cam.elevation, self._cam.azimuth = \
                    0.95, -16, 55
            self._cam.lookat[:] = d.xpos[self.torso]
            self._renderer.update_scene(d, self._cam)
            self._frames.append(self._renderer.render())

    # ------------------------------------------------------------ target guards
    def _guard_target(self, target):
        """NaN guard + global rate clamp on the 15 leg targets (D052). A
        non-finite joint keeps its last good target (counted, noted at most
        once a second); every joint moves at most the servo's hard limit
        (no-load 4.7 rad/s) per step — whatever produced the target."""
        bad = ~np.isfinite(target)
        if bad.any():
            self.nan_count += 1
            target = np.where(bad, self._q_cmd, target)
            if self.t - self._nan_t >= NAN_LOG_S:
                self._nan_t = self.t
                self.note("nan", f"non-finite joint target on {int(bad.sum())} joint(s) — held the last "
                                 f"good one ({self.nan_count} so far)")
        step = rm.servo_speed("hard") * self.DT
        target = np.clip(target, self._q_cmd - step, self._q_cmd + step)
        self._q_cmd = target.copy()
        return target

    def _guard_claw(self, claw):
        c = np.asarray(claw, float).ravel()[:5]
        prev = self._claw_cmd if self._claw_cmd is not None else np.zeros(5)
        c = np.where(np.isfinite(c), c, prev)
        step = rm.no_load_rad_s("claw") * self.DT
        c = np.clip(c, prev - step, prev + step)
        self._claw_cmd = c.copy()
        return c

    # ------------------------------------------------------------ velocity
    def _budget(self, v):
        key = (round(float(v[0]), 4), round(float(v[1]), 4), round(float(v[2]), 5),
               self.gait.h, self.gait.R0, self.gait.T, self.gait.duty, self.gait.hstep)
        b = self._budget_cache.get(key)
        if b is None:
            if len(self._budget_cache) > 512:
                self._budget_cache.clear()
            b = np.array(self.gait.budget(*key[:3]))
            self._budget_cache[key] = b
        return b

    def _effective_cmd(self, v):
        """cmd_v -> the command the gait gets: latch (zero), void guard
        (retreat / project out), then WaveGait.budget (uniform scale)."""
        v = np.asarray(v, float).copy()
        if not np.isfinite(v).all():
            v = np.zeros(3)
        if self.sup.latched:
            v[:] = 0
        v = self._void_filter(v)
        if np.any(v):
            b = self._budget(v)
            i = int(np.argmax(np.abs(v * np.array([1.0, 1.0, self.gait.R0]))))
            self.budget_k = float(b[i] / v[i]) if abs(v[i]) > 1e-9 else 1.0
            v = b
        else:
            self.budget_k = 1.0
        self.cmd_eff = v
        return v

    # ------------------------------------------------------------ void guard
    def _bearing_unit(self, world_deg):
        a = np.radians(world_deg) - self.yaw()
        return np.array([np.cos(a), np.sin(a)])

    def void_blocks(self, v):
        """None, or the body-frame bearing (deg) of a latched void that the
        command (vx, vy, _) has a positive component toward."""
        for wb in self._void_bearings:
            u = self._bearing_unit(wb)
            if float(np.dot(np.asarray(v, float)[:2], u)) > VOID_TOL:
                return _wrap_deg(wb - np.degrees(self.yaw()))
        return None

    def _void_filter(self, v):
        if not self._void_bearings:
            return v
        if self._void_phase == "retreat":
            if self.t < (self.cliff_react.retreat_until or 0.0):
                u = self._bearing_unit(self._void_bearings[-1])
                rv = self.cliff_react.command(self.t, *(self._void_speed * u), 0.0)
                return np.array([rv[0], rv[1], 0.0])
            self._void_phase = "held"            # retreat done: stop and wait for `clear`
            with self.lock:
                self.cmd_v[:] = 0
            self.sup.request_stop()
            self.note("void", "retreat done — safe-stop; `clear` releases the void guard")
            return np.zeros(3)
        for wb in self._void_bearings:           # project out anything toward a latched void
            u = self._bearing_unit(wb)
            c = float(np.dot(v[:2], u))
            if c > 0:
                v[:2] -= c * u
        return v

    def _void_update(self, con, state):
        """The settled-phase detector goto used (D050), run on EVERY walk: at
        50 Hz, NORMAL, moving, not already retreating, no gesture."""
        if (self._k % 10 != 0 or state != "NORMAL" or self._ges is not None
                or self._void_phase == "retreat" or not np.any(self.cmd_eff)):
            return
        g = self.gait
        ph = [(self.sup.t_gait / g.T + g.phase_off[i]) % 1.0 for i in range(N_LEGS)]
        settled = [ph[i] < g.duty and 0.12 < ph[i] / g.duty < 0.95 for i in range(N_LEGS)]
        fired = self.cliff_det.update(self.t, settled, con, probed_out=self.probe_out)
        if fired:
            self._on_void(int(fired[0]))

    def _on_void(self, leg):
        feet = self.sup.last_feet_raw if self.sup.last_feet_raw is not None else self.gait.p_nom
        body_deg = float(np.degrees(np.arctan2(feet[leg][1], feet[leg][0])))
        world_deg = _wrap_deg(body_deg + np.degrees(self.yaw()))
        u = self._bearing_unit(world_deg)
        toward = float(np.dot(self.cmd_eff[:2], u))
        self._void_speed = max(toward, VOID_RETREAT_MIN)
        self.cliff_react = CliffReaction(self.gait, retreat_cycles=VOID_RETREAT_CYCLES)
        self.cliff_react.on_void(self.t)
        self._void_bearings.append(world_deg)
        self._void_phase = "retreat"
        p = self.data.xpos[self.torso]
        self.void = dict(bearing_deg=round(body_deg, 1), leg=leg, t=round(self.t, 2),
                         world_bearing_deg=round(world_deg, 1), xy=(float(p[0]), float(p[1])))
        with self.lock:
            self.cmd_v[:] = 0
        self.note("void", f"VOID at {body_deg:.0f} deg (leg {leg} probed {PROBE_MAX:.0f} mm, nothing there) "
                          f"— backing off, then safe-stop")

    def clear_void(self):
        """Operator: release the void latch (and forget every latched bearing)."""
        had = bool(self._void_bearings)
        self.void = None
        self._void_bearings = []
        self._void_phase = None
        self.cliff_det = CliffDetector()
        self.cliff_react = CliffReaction(self.gait, retreat_cycles=VOID_RETREAT_CYCLES)
        return had

    def clear(self):
        """`clear`: void latch + latched safe-stop. Returns a reply line."""
        v = self.clear_void()
        s = self.sup.clear_latch()
        if not (v or s):
            return "nothing latched"
        return "cleared: " + ", ".join(x for x, y in (("void guard", v), ("latched safe-stop", s)) if y)

    def _refuse_motion(self, v):
        """Why a NEW velocity command must be refused, or None (D052)."""
        if not np.any(v):
            return None
        if self.locomotion_held():
            return "locomotion held: mirroring sim->real without real foot contacts"
        if self.sup.latched:
            return f"blocked: latched safe-stop ({self.sup.latch_reason}) — `clear` to release"
        b = self.void_blocks(v)
        if b is not None:
            return f"blocked: void at {b:.0f} deg — `clear` to release"
        return None

    def set_velocity(self, vx, vy=0.0, wz=0.0, source="walk"):
        """The one door for a velocity command from any UI: refused (returns
        the reason) when a guard forbids it, else sets cmd_v and returns None.
        A running gesture is blended out first (the gait starts after)."""
        v = np.array([vx, vy, wz], float)
        if not np.isfinite(v).all():
            return "non-finite command ignored"
        why = self._refuse_motion(v)
        if why:
            return why
        if np.any(v) and self.gesture_busy:
            self.end_gesture()
        with self.lock:
            self.cmd_v[:] = v
        return None

    # ------------------------------------------------------------ probe
    def _probe_reset(self):
        """Outside probing (FALLEN/RIGHTED, a gesture, probe off, sim2real): the
        offsets are not applied (the supervisor gets probe_dz=None; the rate
        clamp smooths the release) and restart from zero."""
        self.probe_dz[:] = 0.0
        self.probe_state = [SWING] * N_LEGS
        self._probe_hits[:] = 0
        self.probe_out[:] = False

    def _probe_tick(self, con, stance, q_meas=None):
        """One 50 Hz tick of the per-leg stance probe (see the constants).
        q_meas (5,3): measured joints — the seek then leads the real foot by
        at most PROBE_LEAD_MM and the preload is measured from where the foot
        actually touched (without it: open-loop, the pre-lead behaviour)."""
        dt = PROBE_TICK_S
        meas = None
        raw = self.sup.last_feet_raw
        if q_meas is not None and raw is not None:
            meas = raw[:, 2] - pf.feet_body(q_meas)[:, 2]     # mm the real foot is below its raw target
        settle = int(np.ceil(PROBE_SETTLE_FRAC * self.gait.duty * self.gait.T / dt))
        stance = np.ones(N_LEGS, bool) if stance is None else np.asarray(stance, bool)
        for i in range(N_LEGS):
            st = self.probe_state[i]
            if not stance[i]:                               # liftoff / swing: ramp out
                self.probe_state[i] = SWING
                self.probe_dz[i] = max(0.0, self.probe_dz[i] - PROBE_RELAX_RATE * dt)
                self._probe_hits[i] = 0
                self.probe_out[i] = False
                continue
            if st == SWING:                                 # touchdown: a new stance
                st = SEEK
                self._probe_hits[i] = 0
                self._probe_age[i] = 0
            if st == SEEK:
                self._probe_age[i] += 1
                if con[i]:
                    self._probe_hits[i] += 1
                    if self._probe_hits[i] >= PROBE_CONFIRM_TICKS:
                        base = self.probe_dz[i] if meas is None else min(self.probe_dz[i], meas[i])
                        if base >= PROBE_PRELOAD_MIN_MM:
                            st = PRELOAD
                            self._probe_goal[i] = max(self.probe_dz[i], base + PROBE_PRELOAD_MM)
                        else:
                            st = HOLD
                else:
                    self._probe_hits[i] = 0
                    if self._probe_age[i] > settle:
                        nxt = self.probe_dz[i] + PROBE_RATE * dt
                        if meas is not None:                # never lead the real foot by more than the preload
                            nxt = min(nxt, max(self.probe_dz[i], meas[i] + PROBE_LEAD_MM))
                        self.probe_dz[i] = min(nxt, PROBE_MAX)
            elif st == PRELOAD:
                self.probe_dz[i] = min(self.probe_dz[i] + PROBE_RATE * dt, self._probe_goal[i])
                if self.probe_dz[i] >= self._probe_goal[i] - 1e-9:
                    st = HOLD
            self.probe_state[i] = st
            self.probe_out[i] = (st == SEEK and self.probe_dz[i] >= PROBE_MAX - 1e-6 and not con[i])

    # ------------------------------------------------------------ gestures
    def gesture_refusal(self, switching=False, name=None, fn=None):
        """Why a gesture cannot start now, or None. Needs a planted standstill:
        reflex NORMAL with no stop pending, no velocity asked, no goto. A new
        gesture while one is running (the studio scrubbing a preview) is a
        switch and only needs NORMAL. D052 V2, while mirroring sim->real: a
        gesture that walks (is_gaited) is held like a velocity command, and
        nothing starts while the bridge's soft entry is still landing."""
        s = self.sup
        if s.state != "NORMAL":
            return f"reflex is {s.state} — wait for NORMAL"
        if self.entry_active():
            return "the hardware soft entry is still blending the real legs in — wait for it"
        if self.locomotion_held() and is_gaited(name, fn):
            return (f"locomotion held: {name or getattr(fn, '__name__', 'that gesture')} walks, and the real "
                    "legs are mirroring sim->real without foot contacts")
        if switching:
            return None
        if s._stop_req:
            return "a safe-stop is still settling"
        if np.any(self.cmd_v) or np.any(self.cmd_eff):
            return "busy walking — stop first (the MCP server enforces the same rule)"
        if getattr(self, "goto_state", None) is not None:
            return "a goto is running — stop first"
        return None

    def start_gesture(self, fn, total, name=None):
        """Request a gesture (any thread). Returns None when accepted, else the
        reason. The step loop blends it in; see gesture_busy."""
        why = self.gesture_refusal(switching=self._ges is not None and self._ges["phase"] != "out",
                                   name=name, fn=fn)
        if why:
            return why
        with self.lock:
            self.gesture = (fn, float(total), self.t)
        self._ges_name = name
        return None

    def end_gesture(self):
        """Blend a running gesture (or studio preview hold) out to the stance."""
        with self.lock:
            self.gesture = None
        if self._ges is not None and self._ges["phase"] in ("in", "run"):
            self._begin_exit(self._q_cmd.copy())

    def _abort_gesture(self, why):
        with self.lock:
            if self._ges is not None and self.gesture is self._ges["src"]:
                self.gesture = None
        name = None if self._ges is None else self._ges.get("name")
        self._ges = None
        self.note("gesture", f"{name or 'gesture'} {why}")

    def _gesture_sync(self, req):
        """Turn the `gesture` request slot into the in/run/out machine. The
        slot is written by start_gesture and (directly) by the cockpit; a new
        tuple is a start (re-checked here: direct writers skip the refusal),
        a cleared slot mid-gesture is an external end (preview_off)."""
        cur = self._ges
        if req is not None and (cur is None or cur["src"] is not req):
            # a switch (studio scrubbing) needs only NORMAL; during an exit blend
            # the full standstill rule applies (a walk may be what ended it)
            why = self.gesture_refusal(switching=cur is not None and cur["phase"] != "out",
                                       name=getattr(self, "_ges_name", None), fn=req[0])
            if why:
                self._ges_name = None
                with self.lock:
                    if self.gesture is req:
                        self.gesture = None
                self.note("gesture", f"refused: {why}")
                return
            fn, total = req[0], float(req[1])
            try:
                q0, c0 = self._call_gesture(fn, 0.0)
            except Exception as e:                      # noqa: BLE001 — a broken spec must not kill the sim
                with self.lock:
                    if self.gesture is req:
                        self.gesture = None
                self.note("gesture", f"refused: {type(e).__name__}: {e}")
                return
            q_from = self._q_cmd.copy()
            name = getattr(self, "_ges_name", None) or getattr(fn, "__name__", None) or \
                getattr(getattr(fn, "spec", None), "get", lambda *_: None)("name")
            self._ges_name = None
            self._ges = dict(src=req, fn=fn, total=total, phase="in", t0=self.t,
                             dur=blend_time(q_from, q0), q_from=q_from, name=name,
                             c_from=None if self._claw_cmd is None else self._claw_cmd.copy())
            self.probe_dz[:] = 0.0                       # the gesture owns the feet now
            return
        if req is None and cur is not None and cur["phase"] in ("in", "run"):
            self._begin_exit(self._q_cmd.copy())         # external end (preview_off, reset of the slot)

    def _call_gesture(self, fn, t):
        """fn(g, t) -> (q (15,) rad, claw (5,) rad or None). Accepts (q, claw[, ...])
        or a bare q; a wrong shape raises (the caller turns that into a note)."""
        out = fn(self.gait, t)
        if isinstance(out, tuple):
            q, c = out[0], (out[1] if len(out) > 1 else None)
        else:
            q, c = out, None
        q = np.asarray(q, float).ravel()
        if q.size != 3 * N_LEGS:
            raise ValueError(f"gesture returned {q.size} joint targets, need 15")
        if c is not None:
            c = np.asarray(c, float).ravel()
            c = c if c.size == N_LEGS else None
        return q, c

    def _begin_exit(self, q_from):
        G = self._ges
        G["phase"] = "out"
        G["t0"] = self.t
        G["q_from"] = np.asarray(q_from, float).ravel().copy()
        G["dur"] = blend_time(G["q_from"], self.sup._planted_q())
        with self.lock:
            if self.gesture is G["src"]:
                self.gesture = None

    def _gesture_targets(self, q_sup):
        """(q (15,), claw (5,) or None) for this step of the gesture machine.
        in: smoothstep from the last commanded target to fn(g, 0); run: fn;
        out: smoothstep from the last gesture pose to the supervisor's stance."""
        G = self._ges
        claw = None
        if G["phase"] == "in":
            a = (self.t - G["t0"]) / max(G["dur"], 1e-6)
            q0, c0 = self._call_gesture(G["fn"], 0.0)
            s = _smooth(a)
            q = (1 - s) * G["q_from"] + s * q0
            if c0 is not None and G["c_from"] is not None:
                claw = (1 - s) * G["c_from"] + s * c0
            if a >= 1.0:
                G["phase"], G["t_run0"] = "run", self.t
            return q, claw
        if G["phase"] == "run":
            tg = self.t - G["t_run0"]
            if tg < G["total"]:
                try:
                    return self._call_gesture(G["fn"], tg)
                except Exception as e:                  # noqa: BLE001
                    self.note("gesture", f"{G['name']} failed at t={tg:.2f}: {e}")
                    self._begin_exit(self._q_cmd.copy())
            else:
                q_end, _c = self._call_gesture(G["fn"], G["total"])
                self._begin_exit(q_end)
        a = (self.t - G["t0"]) / max(G["dur"], 1e-6)
        s = _smooth(a)
        q = (1 - s) * G["q_from"] + s * np.asarray(q_sup, float).ravel()
        if a >= 1.0:
            self._ges = None
        return q, None

    # ------------------------------------------------------------ gait presets
    def apply_gait(self, **kw):
        """Change gait parameters (T, h, R0, duty, hstep) live: phase-continuous
        for T, p_nom recomputed for h/R0 and the supervisor's planted-stance
        cache dropped (it went stale on `set gait.h` before D052). D052 V2:
        validated BEFORE anything changes (ValueError says why) — `set gait.T 0`
        used to be accepted and every phase reader then divided by zero."""
        g = self.gait
        kw = {k: v for k, v in kw.items() if k in GAIT_KEYS}
        new = {k: float(kw.get(k, getattr(g, k))) for k in GAIT_KEYS}
        bad = [k for k, v in new.items() if not np.isfinite(v)]
        if bad:
            raise ValueError(f"gait {', '.join(bad)}: not a finite number")
        for k, (lo, hi) in (("T", GAIT_T_RANGE), ("duty", GAIT_DUTY_RANGE), ("hstep", GAIT_HSTEP_RANGE)):
            if not lo <= new[k] <= hi:
                raise ValueError(f"gait.{k} = {new[k]:g} is outside {lo:g}..{hi:g}")
        if kw and self.sim2real:
            raise ValueError("gait changes move the stance: mirror off first (sim2real is on)")
        if "h" in kw or "R0" in kw:
            cand = WaveGait(new["h"], new["R0"], new["T"], new["duty"], new["hstep"])
            q = np.array([leg_ik(body_to_leg(i, cand.p_nom[i])) for i in range(N_LEGS)])
            if not np.isfinite(q).all():
                raise ValueError(f"gait h={new['h']:g} R0={new['R0']:g}: the stance is out of the legs' reach")
        with self.lock:
            if "T" in kw:
                ph = (self.sup.t_gait / g.T) % 1.0          # phase continuity
                g.T = float(kw["T"])
                self.sup.t_gait = ph * g.T
            for k in ("duty", "hstep", "h", "R0"):
                if k in kw:
                    setattr(g, k, float(kw[k]))
            if "h" in kw or "R0" in kw:
                g.p_nom = WaveGait(g.h, g.R0, g.T, g.duty, g.hstep).p_nom
                self.sup._q_planted = None
            self._budget_cache.clear()
        return {k: getattr(g, k) for k in GAIT_KEYS}

    def check_report(self, name=None):
        """Feasibility report lines (pebble_feasibility). name None: the gait at
        the current (budgeted) command and at the envelope corners; else the
        named gesture. Returns a list of str."""
        g = self.gait
        if name:
            if name not in self.gestures:
                return [f"unknown gesture {name!r}; have: {', '.join(sorted(self.gestures))}"]
            fn, total = self.gestures[name]
            if hasattr(fn, "report"):                      # a keyframe gesture
                rep = fn.report(g=g)
            else:
                rep = pf.check(fn, total, g=g, kind=kind_of(name), name=name)
            return list(rep.lines)
        mc = g.max_command()
        cmds = []
        if np.any(self.cmd_v):
            cmds.append(tuple(float(x) for x in self._budget(self.cmd_v)))
        cmds += [(mc["v"], 0.0, 0.0), (0.0, mc["v"], 0.0), (0.0, 0.0, mc["wz"])]
        out = [f"envelope: |v| <= {mc['v']:.1f} mm/s, |wz| <= {mc['wz']:.3f} rad/s in place "
               f"(gait T={g.T} h={g.h} R0={g.R0} duty={g.duty} hstep={g.hstep})"]
        for c in cmds:
            out += list(pf.check_gait(g, c).lines)
        return out

    # ------------------------------------------------------------ HUD
    STATE_RGBA = {"NORMAL": (0.2, 0.85, 0.3, 0.9), "PLANT": (0.95, 0.8, 0.2, 0.9),
                  "BRACE": (0.95, 0.55, 0.1, 0.9), "RECOVER": (0.3, 0.7, 0.95, 0.9),
                  "FALLEN": (0.9, 0.15, 0.15, 0.9), "RIGHTED": (0.6, 0.3, 0.9, 0.9)}

    def draw_hud(self, scn):
        """D048: an in-window HUD in the viewer's user scene — a marker
        above the torso coloured by the reflex state and an arrow for the
        velocity command (length = 2 s of travel; a curved tick for turn).
        D052: the arrow is the EFFECTIVE command (after guards + budget).
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
        v = np.asarray(self.cmd_eff, float).copy()
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
    # D052: the caps are the gait's envelope (V_MAX property, from
    # WaveGait.max_command) and every nudge goes through set_velocity's guards.
    TELEOP = {
        "up": ("vx", +15.0), "down": ("vx", -15.0),
        "left": ("vy", +15.0), "right": ("vy", -15.0),    # +y = leftward
        "W": ("vx", +15.0), "S": ("vx", -15.0),
        "A": ("vy", +15.0), "D": ("vy", -15.0),
        "Q": ("wz", +0.12), "E": ("wz", -0.12),
    }
    GLFW_ARROWS = {265: "up", 264: "down", 263: "left", 262: "right"}

    def teleop(self, key):
        """key: 'up'/'down'/'left'/'right', 'W'.. 'E', ' ' (stop), 'G' (wave).
        Returns a one-line status or None if the key is not a teleop key."""
        if key == " ":                             # SPACE: stop everything
            with self.lock:
                self.cmd_v[:] = 0
            self.end_gesture()
            self.sup.request_stop()
            return "[teleop] SAFE-STOP"
        if key == "G":                             # quick wave hello
            if "wave" in GESTURES:
                fn, total = GESTURES["wave"]
                why = self.start_gesture(fn, total, "wave")
                return "[teleop] wave" if why is None else f"[teleop] wave refused: {why}"
            return None
        hit = self.TELEOP.get(key)
        if hit is None:
            return None
        axis, dv = hit
        i = {"vx": 0, "vy": 1, "wz": 2}[axis]
        vmax = self.V_MAX
        with self.lock:
            v = self.cmd_v.copy()
        v[i] = float(np.clip(v[i] + dv, -vmax[i], vmax[i]))
        why = self.set_velocity(*v, source="teleop")
        if why:
            return f"[teleop] {why}"
        note = self.budget_note_for(v)
        return f"[teleop] v=({v[0]:.0f}, {v[1]:.0f}) mm/s  wz={v[2]:.2f}" + (f"  ({note})" if note else "")

    def budget_note_for(self, v):
        """budget_note() for a command that has not been stepped yet."""
        v = np.asarray(v, float)
        if not np.any(v):
            return ""
        b = self._budget(v)
        i = int(np.argmax(np.abs(v * np.array([1.0, 1.0, self.gait.R0]))))
        k = float(b[i] / v[i]) if abs(v[i]) > 1e-9 else 1.0
        return self._budget_words(k)

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
            try:
                v = [float(a) for a in args[:3]] + [0.0] * (3 - len(args[:3]))
            except ValueError:
                return "walk VX [VY] [WZ] (numbers)"
            why = self.set_velocity(*v)
            if why:
                return why
            note = self.budget_note_for(v)
            return f"walking vx={v[0]} vy={v[1]} wz={v[2]}" + (f" ({note})" if note else "")
        if c == "stop":
            with self.lock:
                self.cmd_v[:] = 0
            busy = self.gesture_busy
            self.end_gesture()
            self.sup.request_stop()
            return "safe-stop requested (PLANT->BRACE)" + ("; gesture blended out" if busy else "")
        if c == "gesture":
            if not args or args[0] not in self.gestures:
                return f"gestures: {', '.join(self.gestures)}"
            fn, total = self.gestures[args[0]]
            why = self.start_gesture(fn, total, args[0])
            if why:
                return why
            return f"{args[0]} ({total:.1f} s + blends)"
        if c == "clear":
            return self.clear()
        if c == "probe":
            if args and args[0] in ("on", "off"):
                self.probe_on = args[0] == "on"
            return f"probe {'on' if self.probe_on else 'off'}" + \
                   (" (inactive while mirroring sim->real)" if self.sim2real else "")
        if c == "gait":
            presets = gait_presets()
            if not args or args[0] == "list":
                return "gait presets: " + "; ".join(
                    f"{k} (" + " ".join(f"{kk}={vv:g}" for kk, vv in p.items()) + ")"
                    for k, p in presets.items())
            if args[0] == "save":
                if len(args) < 2:
                    return "gait save NAME"
                try:
                    path = save_gait_preset(args[1], self.gait)
                except ValueError as e:
                    return str(e)
                return f"saved {os.path.relpath(path, os.path.join(HERE, '..'))}"
            if args[0] not in presets:
                return f"no gait preset {args[0]!r}; have: {', '.join(presets)}"
            if np.any(self.cmd_eff) or self.sup.state != "NORMAL":
                return "gait presets change the stance: stop first"
            try:
                p = self.apply_gait(**presets[args[0]])
            except ValueError as e:
                return f"gait {args[0]} refused: {e}"
            return f"gait {args[0]}: " + " ".join(f"{k}={v:g}" for k, v in p.items())
        if c == "check":
            return "\n".join(self.check_report(args[0] if args else None))
        if c == "say":
            w = args[0] if args else ""
            wav = chord_wav(w)
            if wav is None:
                return f"unknown word; lexicon: {', '.join(lexicon())}"
            self.said = (self.said[0] + 1, w)
            if getattr(self, "browser_audio", False):
                return f"*{w}* (chord, in the browser)"
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
                       "gait.duty gait.hstep reflex.trip reflex.stall_s reflex.fallen_max_s "\
                       "servo.on servo.hold_hz servo.latency_s servo.rate_rad_s servo.quant"
            p = args[0]
            try:
                val = float(args[1])
            except ValueError:
                return f"set {p}: not a number"
            if p.startswith("servo."):
                k = p.split(".", 1)[1]
                if k not in self.servo.p:
                    return f"unknown param {p}"
                with self.lock:
                    self.servo.set(**{k: val})
                    self.servo.reset(self._q_cmd)       # no jump when switching on/off
                return self.servo.describe()
            if p.startswith("gait."):
                k = p.split(".", 1)[1]
                if k not in GAIT_KEYS:
                    return f"unknown param {p}"
                try:
                    self.apply_gait(**{k: val})
                except ValueError as e:
                    return f"set {p} refused: {e}"
            elif p == "reflex.trip":
                self.sup.gyro_trip = val
            elif p == "reflex.stall_s":
                self.sup.stall_s = val
            elif p == "reflex.fallen_max_s":
                self.sup.fallen_max_s = val
            else:
                return f"unknown param {p}"
            return f"{p} = {val}  (sim-only until params.yaml + regen — "\
                   "note keepers in NOTES_INBOX; `gait save NAME` keeps a gait)"
        if c == "show":
            g = self.gait
            p = self.data.xpos[self.torso]
            gs = self.guard_status()
            extra = []
            if gs["budget_note"]:
                extra.append(gs["budget_note"])
            if self.void is not None:
                extra.append(f"VOID latched at {self.void['bearing_deg']:.0f} deg ({gs['void_phase']})")
            if gs["latched"]:
                extra.append(f"LATCHED safe-stop: {gs['latch_reason']}")
            if gs["locomotion_held"]:
                extra.append("locomotion held (sim2real)")
            if self.nan_count:
                extra.append(f"{self.nan_count} non-finite targets dropped")
            return (f"gait: T={g.T} h={g.h} R0={g.R0} duty={g.duty} "
                    f"hstep={g.hstep} | reflex: trip={self.sup.gyro_trip} "
                    f"state={self.sup.state} trips={self.sup.trip_count} | pose=({p[0]*1000:.0f}, "
                    f"{p[1]*1000:.0f}) mm  t={self.t:.1f} s | cmd {np.round(self.cmd_v, 2).tolist()} -> "
                    f"{np.round(self.cmd_eff, 2).tolist()} | probe {''.join(s[0] for s in self.probe_state)} "
                    f"dz {np.round(self.probe_dz, 1).tolist()} | {self.servo.describe()}"
                    + ("\n  " + "; ".join(extra) if extra else ""))
        if c == "push":
            try:
                fx, fy = float(args[0]), float(args[1])
                dur = float(args[2]) if len(args) > 2 else 0.4
            except (IndexError, ValueError):
                return "push FX FY [DUR]"
            if self.sim2real:
                return "push refused: mirroring sim->real (the real legs would get the sim's reaction) — mirror off first"
            try:
                fx, fy, dur = shove_args(fx, fy, dur)
            except ValueError as e:
                return f"push refused: {e}"
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
        return f"unknown command {c!r} — walk/stop/gesture/say/set/show/gait/check/" \
               "clear/probe/push/record/rl/righter/wait/quit"


def new_notes(pg, cursor):
    """(notes appended since cursor, new cursor) — for printing UIs."""
    n = pg.note_seq - cursor
    notes = list(pg.notes)[-n:] if n > 0 else []
    return notes, pg.note_seq


def run_headless(pg, script):
    seen = pg.note_seq

    def flush_notes():
        nonlocal seen
        notes, seen = new_notes(pg, seen)
        for t, kind, msg in notes:
            print(f"[{t:6.2f}s] ({kind}) {msg}")

    for raw in script.split(";"):
        cmd = raw.strip()
        if not cmd:
            continue
        if cmd.startswith("wait"):
            secs = float(cmd.split()[1])
            for _ in range(int(secs / pg.DT)):
                pg.step()
            flush_notes()
            print(f"[{pg.t:6.2f}s] (waited {secs} s)")
            continue
        r = pg.do(cmd)
        print(f"[{pg.t:6.2f}s] > {cmd}\n          {r}")
        flush_notes()
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
        with viewer_ctx.lock():
            viewer_ctx.opt.geomgroup[3] = 1       # world objects / room walls are group 3
    print(TELEOP_HELP)
    print("type `help` for the command list, `help rl` for the RL hooks", flush=True)
    t_wall = time.monotonic()
    seen = pg.note_seq
    while pg.alive:
        pg.step()
        if pg.note_seq != seen:
            notes, seen = new_notes(pg, seen)
            for t, kind, msg in notes:
                print(f"\r\033[K[{t:.1f}s] ({kind}) {msg}\npebble> ", end="", flush=True)
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
