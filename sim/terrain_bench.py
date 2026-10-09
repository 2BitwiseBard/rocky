"""Terrain bench (D065): the body leveler's ship gate. Every control stack on every
terrain family, measured from the TRUE attitude, with the pass rules evaluated here.

    MUJOCO_GL=egl .venv/bin/python sim/terrain_bench.py --quick --stack S0,S1 --imu ideal
    MUJOCO_GL=egl .venv/bin/python sim/terrain_bench.py --jobs 6        # everything
    MUJOCO_GL=egl .venv/bin/python sim/terrain_bench.py --list          # the rows
    MUJOCO_GL=egl .venv/bin/python sim/terrain_bench.py --one W3.stone20.f0.stand --stack S1
writes sim/out/terrain_bench.json (--quick: terrain_bench_quick.json; --out PATH)

Stacks (the real Playground, servo realism on, righter off, driven through do()):
  S0  `probe off`: no stance probe, and with it no touchdown gate and no void guard
      (playground.py step: not probing -> _probe_reset, so probe_out stays False and
      _void_update never fires; _gate_update: ok = probing and ...)
  S1  the shipped D064 stack: probe + late touchdown gate + void guard, leveler off
  S2  S1 + `level on` (gait/pebble_level.py BodyLeveler)
  S3  reserved: S2 + the terrain RL residual (B160) — always "unavailable" here
IMU modes (seeded per row from the row's bench seed):
  ideal     the Playground's default SimIMU
  noisy     rl_common.OBS_NOISE (grav / gyro sigma, gyro bias) + 20 ms latency
  mount1.5  the ideal IMU seen through a fixed 1.5 deg roll mount error (the bias
            alone: what an uncalibrated mount does to the leveler, B159)
  A mode goes through the Playground's IMU-spec hook (set_imu(spec), kept across a
  respawn) and is verified on the IMU it built (a latency mode also on the first read:
  step() must pass t); a mode the Playground cannot apply is reported "unavailable",
  never run as ideal. The pass rules gate ideal and noisy; mount1.5 is reported only.

Worlds (specs live here, NOT in world_builder.PRESETS: test_place_bench pins that list).
Every row spawns at the origin (yaw 0 unless it is a cliff approach), settles 1 s,
then walks `walk 45` (fitted to the 34.2 mm/s envelope) or stands. A walking row's
start phase is the gait clock set to 0, 1/3 or 2/3 of a cycle before the walk (the
clock does not run at a standstill, so a longer settle would not change it).
  W1 flat: walk 20 s x 3 phases, stand 10 s; and (review 9r: the flat-zero claim held only
     for the straight walk) a turn in place (wz 0.4) and a walk + turn (45, wz 0.2), each
     8 s then `stop` and 3 s, and a 25 N rim shove at 1 s on a stand (+x and +y), 7 s
  W2 gravity slopes 5 / 10 deg toward 0 / 90 / 180 deg (walking +x: down, across,
     up): walk 20 s x 3 phases; stand 10 s at 5 / 8 / 10 deg (P3 judges 5 and 8)
  W3 a 40 x 40 mm stone of 10 / 20 / 30 mm under foot 0 or foot 3, stand 8 s
     (Playground(z0=stone): the other four feet settle onto the floor)
  W4 scenes.build_model box rubble 10 / 20 / 30 mm, seeds 0-4, walk 20 s
  W5 ramps 5 / 10 deg (L 0.6, width 0.8, landing 0.4, far side down) at x 0.30, walk 30 s x 3
  W6 stairs 4 x 15 and 4 x 20 mm (run 0.15, width 0.8) at x 0.30, walk 30 s x 3
  W7 8 deg slope, stand, a 25 N rim shove (sim/shove.py, 0.4 s) at 3 s, downhill / uphill
  W8 regressions: the cliff approach grid (0/15/20/30/45 deg at walk 25 / 45, the
     test_void_guard_holds_at_every_approach_angle grid), a cliff on a 5 deg slope
     walked to downhill (d0), across (d90) and uphill (d180: the leading feet are the
     raised ones), run_sim's flat band (bare gait: once, not per stack)

Metrics per run (units in the key): tilt_rms/p95/peak/end_deg (end = mean of the last
2 s) from sim_imu.grav_body (gravity, not world z: on a gravity slope world z is not
up), roll/pitch_rms_deg; height_err_mean/rms_mm = the torso origin above the plane of
the switch-closed feet's lowest points along true up, minus rocky_model.stance_torso_z_m;
progress_mm along the command (walks; % of S1's same row in pct_s1), creep_mm (stands);
fell (fall_count or tilt > 30), tipped (tilt > the row's slope / ramp angle + TIPPED_DEG; a
cliff row holds = fired, no fall, the torso on the platform, no tip after the fire); gate_holds;
probe_out_edges; void; void_ground_mm (under the void foot, as lowered, at the fire) and
void_ground_nom_mm (under its un-probed target: + the lowering the foot made), void_nohit
(the ray found nothing); false_void = a void on a world with no drop, or with ground within
PROBE_MAX of the un-probed target (the probe could have reached it); W3 stone rows add
stone_foot_n / stone_foot_closed_pct (the stone foot's mean normal force and switch over
the last END_S: in S1 and S2 alike it stands unloaded); min_clear_mm = belly (rocky_model.belly_boxes
+ belly_posts bottoms) to the ground along gravity by mj_ray, belly_contact_s;
load_rms_max / load_peak (|tau_e| / stall, rl_common.motor_torque), heat_max (fraction
of the thermal budget), thermal_trip; level_sat_pct, level_max_mm (|offset|).
B162 adds: void_class (false | probe_reach | drop, _void_class: false + probe_reach = the
D065 false_void); on a walk, level_slope_mean(_deg) and derate_pct (the envelope the leveled
plane allowed, tick by tick, as % of the bare one for the same ask: the deliberate speed cost
P4 is judged after); level_sinkcap_pct and level_rough_p95/max_mm (the rough-ground sink cap's
share of the ticks and the roughness it read); on a stand or a shove settle_s (from the spawn,
or the push, until the tilt stays within SETTLE_DEG of tilt_end); on a shove shove_peak_deg and
shove_excursion_deg (the tilt after the push, and its peak over the 0.5 s before it: on a leveled
slope tilt_peak_deg is the leveler still converging at the row's start, not the shove).
The rules (evaluate()) carry the D065 wording of the three B162 restated (P1, P4, P6) beside
them, with that wording's verdict.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import inspect
import json
import os
import re
import subprocess
import sys
import tempfile
import time
import warnings
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (os.path.join(ROOT, "gait"), os.path.join(ROOT, "perception"), HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import rocky_model as rm                                              # noqa: E402
import rl_common as rc                                                # noqa: E402
import world_builder as wb                                            # noqa: E402
from sim_imu import SimIMU, grav_body, gravity_dir, tilt_from_grav   # noqa: E402

OUT_DIR = os.path.join(HERE, "out")
DECISION = "D065"

# ---- seed tiers (pinned by sim/tests/test_terrain_bench.py) ---------------------------
BENCH_SEEDS = (0, 1, 2, 3, 4)          # what this bench scores (never trained on)
HELDOUT_SEEDS = range(100, 200)        # held-out evaluation (eval_ppo); --tier heldout scores 100-104
TRAIN_SEED_MIN = 1000                  # training draws seeds >= this, never below


def tier_of(seed):
    """'bench' | 'heldout' | 'train' | None (a seed in no tier)."""
    s = int(seed)
    if s in BENCH_SEEDS:
        return "bench"
    if s in HELDOUT_SEEDS:
        return "heldout"
    return "train" if s >= TRAIN_SEED_MIN else None


# ---- stacks and IMU modes ------------------------------------------------------------
STACKS = ("S0", "S1", "S2", "S3")
NOISE = {k: rc.OBS_NOISE[k] for k in ("grav_sigma", "gyro_sigma", "gyro_bias")}
LATENCY_S = 0.02
MOUNT_DEG, MOUNT_AXIS = 1.5, (1.0, 0.0, 0.0)     # roll about body x (review_simreal's rotx)
IMU_MODES = {"ideal": {}, "noisy": dict(NOISE, latency_s=LATENCY_S),
             "mount1.5": {"mount_deg": MOUNT_DEG, "mount_axis": list(MOUNT_AXIS)}}
GATED_IMU = ("ideal", "noisy")          # the pass rules; mount1.5 waits for calibration (B159)

# ---- timing --------------------------------------------------------------------------
T_SETTLE = 1.0          # s planted after the spawn, before the row's clock starts
TICK_S = 0.02           # metrics sample at the bus rate
PHASES = (0.0, 1 / 3, 2 / 3)
END_S = 2.0             # tilt_end: the mean over the row's last END_S
SETTLE_DEG = 0.5        # settle_s (B162): the tilt stays within this of tilt_end from then on
WALK_ASK = 45.0         # mm/s asked: the budget fits it (34.2 on the D064 gait)
CLIFF_MAX_S = 20.0      # a cliff approach runs until 3 s after the fire, at most this
CLIFF_AFTER_S = 3.0
CLIFF_SPEEDS = (25, 45)
CLIFF_DEG = (0, 15, 20, 30, 45)
FALL_TILT_DEG = 30.0
from playground import PROBE_MAX, TIPPED_DEG   # noqa: E402  the probe's reach; a tip after a fire
SHOVE_N, SHOVE_S, SHOVE_AT = 25.0, 0.4, 3.0


def _stone_spec(h_mm, foot):
    p = _foot_xy(foot)
    return {"base": "flat", "objects": [{"kind": "box", "pos": [round(p[0], 4), round(p[1], 4)],
                                         "size": [0.04, 0.04, h_mm / 1000.0]}]}


def _foot_xy(foot):
    """Leg `foot`'s nominal foothold (m, world = body at the spawn): WaveGait().p_nom."""
    from pebble_gait import WaveGait
    return WaveGait().p_nom[int(foot), :2] / 1000.0


def _slope(deg, dir_deg):
    return {"base": "flat", "gravity_tilt_deg": float(deg), "gravity_tilt_dir_deg": float(dir_deg)}


def _ramp(deg):
    L = 0.6
    return {"base": "flat", "objects": [{"kind": "ramp", "pos": [0.30, 0.0], "len_m": L,
                                         "rise_m": round(L * float(np.tan(np.radians(deg))), 5),
                                         "width_m": 0.8, "landing_m": 0.4, "far": "down"}]}


def _stairs(rise_mm):
    return {"base": "flat", "objects": [{"kind": "stairs", "pos": [0.30, 0.0], "steps": 4,
                                         "rise_m": rise_mm / 1000.0, "run_m": 0.15, "width_m": 0.8}]}


def rows(quick=False, tier="bench"):
    """The bench's rows, in order. quick: one phase, one seed (the cliff grid stays whole)."""
    if tier not in ("bench", "heldout"):
        raise ValueError("the bench scores the bench or the held-out tier, never a training seed")
    phases = list(enumerate(PHASES))[:1 if quick else None]
    rubble_seeds = list(BENCH_SEEDS if tier == "bench" else HELDOUT_SEEDS[:len(BENCH_SEEDS)])
    rubble_seeds = rubble_seeds[:1 if quick else None]
    out = []

    def add(rid, world, kind, build, seconds, **kw):
        out.append(dict(id=rid, world=world, kind=kind, build=build, seconds=float(seconds),
                        z0=kw.pop("z0", 0.0), yaw_deg=kw.pop("yaw_deg", 0.0),
                        walk=kw.pop("walk", WALK_ASK if kind == "walk" else 0.0),
                        phase=kw.pop("phase", 0.0), seed=kw.pop("seed", 0),
                        slope_deg=kw.pop("slope_deg", 0.0), drop=kw.pop("drop", False), **kw))

    for k, ph in phases:
        add(f"W1.flat.walk.p{k}", "W1", "walk", {"wb": {"base": "flat"}}, 20, phase=ph, seed=k)
    add("W1.flat.stand", "W1", "stand", {"wb": {"base": "flat"}}, 10)
    add("W1.flat.turn", "W1", "motion", {"wb": {"base": "flat"}}, 11, cmd=[0.0, 0.0, 0.4], stop_at=8.0)
    add("W1.flat.walkturn", "W1", "motion", {"wb": {"base": "flat"}}, 11, cmd=[WALK_ASK, 0.0, 0.2],
        stop_at=8.0)
    for name, fx, fy in (("x", SHOVE_N, 0.0), ("y", 0.0, SHOVE_N)):
        add(f"W1.flat.shove.{name}", "W1", "shove", {"wb": {"base": "flat"}}, 7, shove=[fx, fy, SHOVE_S, 1.0])
    for deg in (5, 10):
        for d in (0, 90, 180):
            for k, ph in phases:
                add(f"W2.slope{deg}.d{d}.walk.p{k}", "W2", "walk", {"wb": _slope(deg, d)}, 20,
                    phase=ph, seed=k, slope_deg=deg)
    for deg in (5, 8, 10):
        for d in (0, 90, 180):
            add(f"W2.slope{deg}.d{d}.stand", "W2", "stand", {"wb": _slope(deg, d)}, 10, slope_deg=deg)
    for foot in (0, 3):
        for h in (10, 20, 30):
            add(f"W3.stone{h}.f{foot}.stand", "W3", "stand", {"wb": _stone_spec(h, foot)}, 8,
                z0=h / 1000.0, stone_foot=foot)
    for amp in (10, 20, 30):
        for s in rubble_seeds:
            add(f"W4.rubble{amp}.s{s}.walk", "W4", "walk", {"rubble": [amp, s]}, 20, seed=s)
    for deg in (5, 10):
        for k, ph in phases:
            add(f"W5.ramp{deg}.walk.p{k}", "W5", "walk", {"wb": _ramp(deg)}, 30, phase=ph, seed=k,
                slope_deg=deg)
    for rise in (15, 20):
        for k, ph in phases:
            add(f"W6.stairs{rise}.walk.p{k}", "W6", "walk", {"wb": _stairs(rise)}, 30, phase=ph, seed=k)
    for name, fx in (("down", SHOVE_N), ("up", -SHOVE_N)):
        add(f"W7.slope8.shove.{name}", "W7", "shove", {"wb": _slope(8, 0)}, 10, slope_deg=8,
            shove=[fx, 0.0, SHOVE_S, SHOVE_AT])
    for v in CLIFF_SPEEDS:
        for a in CLIFF_DEG:
            add(f"W8.cliff.a{a}.v{v}", "W8", "cliff", {"cliff": True}, CLIFF_MAX_S, yaw_deg=a,
                walk=float(v), drop=True)
    for d in (0, 90, 180):
        for v in CLIFF_SPEEDS:
            add(f"W8.slopecliff5.v{v}" if d == 0 else f"W8.slopecliff5.d{d}.v{v}", "W8", "cliff",
                {"wb": {"base": "cliff", "gravity_tilt_deg": 5.0, "gravity_tilt_dir_deg": float(d)}},
                CLIFF_MAX_S, walk=float(v), drop=True, slope_deg=5)
    add("W8.run_sim", "W8", "run_sim", {}, 0)
    return out


def row_by_id(rid, tier="bench"):
    for r in rows(tier=tier):
        if r["id"] == rid:
            return r
    raise KeyError(f"no row {rid!r} (--list shows them)")


# ---- the Playground under a stack and an IMU mode ------------------------------------------
def _playground(row):
    from playground import Playground
    b = row["build"]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        if b.get("cliff"):
            pg = Playground(cliff=True)
        else:
            if "rubble" in b:
                from scenes import build_model
                model, z = build_model(*b["rubble"]), 0.0
            else:
                model, z = wb.build(b["wb"])
            pg = Playground(model=model, z0=z + float(row["z0"]))
    pg.do("righter off")
    a = np.radians(float(row["yaw_deg"])) / 2
    if a:
        pg.data.qpos[3:7] = [np.cos(a), 0.0, 0.0, np.sin(a)]
        mujoco.mj_forward(pg.model, pg.data)
    return pg


def imu_spec(mode, seed):
    """The Playground IMU spec for a mode (None = ideal: the Playground's own IMU), or
    raise ValueError when this sim_imu cannot express it. "seed" is the noise stream:
    the row's bench seed, so S1 and S2 see the same draws."""
    if mode not in IMU_MODES:
        raise ValueError(f"unknown IMU mode {mode!r} (have {', '.join(IMU_MODES)})")
    if mode == "ideal":
        return None
    spec = dict(IMU_MODES[mode], seed=int(seed))
    missing = [k for k in spec if k != "seed" and k not in inspect.signature(SimIMU).parameters]
    if missing:
        raise ValueError(f"sim_imu.SimIMU has no {', '.join(missing)} option (D065 item 4)")
    return spec


def _apply_imu(pg, mode, seed):
    """Put the mode's IMU on the Playground through its hook (set_imu(spec), kept across
    a respawn) and verify it on the IMU it built. Returns None, or why the mode is
    unavailable (the run is then not made)."""
    try:
        spec = imu_spec(mode, seed)
    except ValueError as e:
        return str(e)
    if spec is None:
        return None
    hook = getattr(pg, "set_imu", None)
    if not callable(hook):
        return "the Playground has no IMU-spec hook (set_imu(spec))"
    try:
        hook(spec)
    except (TypeError, ValueError) as e:
        return f"the Playground's IMU hook refused the spec: {e}"
    imu = pg.imu
    if mode == "noisy":
        got = {k: getattr(imu, k, None) for k in ("grav_sigma", "gyro_sigma", "latency_s")}
        want = {"grav_sigma": NOISE["grav_sigma"], "gyro_sigma": NOISE["gyro_sigma"], "latency_s": LATENCY_S}
        if any(got[k] is None or abs(float(got[k]) - v) > 1e-12 for k, v in want.items()):
            return f"the Playground's IMU is not the noisy one ({got})"
    else:
        meas = dict(imu.measure(pg.data))["grav"]
        true = grav_body(pg.model, pg.data, pg.torso)
        ang = float(np.degrees(np.arccos(np.clip(np.dot(meas, true) / np.linalg.norm(meas), -1, 1))))
        if abs(ang - MOUNT_DEG) > 0.05:
            return f"the IMU's mount error reads {ang:.2f} deg, not {MOUNT_DEG}"
    return None


def _check_latency(pg):
    """After a step: a latency IMU must have been read with the sim time (read(d, t))."""
    imu = pg.imu
    if float(getattr(imu, "latency_s", 0.0)) > 0 and not getattr(imu, "_buf", [None]):
        return "Playground.step reads the IMU without t: its latency is never applied"
    return None


def _level(pg):
    return getattr(pg.sup, "leveler", None)


def _apply_stack(pg, stack):
    """Returns None, or why the stack is unavailable on this Playground."""
    if stack == "S3":
        return "S3 is reserved for the terrain RL residual (B160)"
    if stack in ("S0", "S1"):
        if getattr(pg, "level_on", False):            # the un-leveled stacks, whatever params says
            pg.do("level off")
        if stack == "S0":
            pg.do("probe off")
        return None
    r = pg.do("level on")
    if _level(pg) is None or not getattr(pg, "level_on", False):
        return f"`level on` is not wired in the Playground (reply: {r!r})"
    return None


def _check_level(pg, stack):
    """After the settle: S2's leveler must be the one ticking (the Playground engages
    it in step()), S0 / S1's must never have been."""
    lv = _level(pg)
    on = bool(lv is not None and lv.enabled)
    if stack == "S2" and not on:
        return "`level on` did not engage the supervisor's leveler"
    if stack in ("S0", "S1") and on:
        return "the leveler is on in an un-leveled stack (params level.enabled?)"
    return None


# ---- metrics ---------------------------------------------------------------------------
def _belly_points_m():
    """Body-frame points (m) on the belly's lowest faces: the tub's and the shelf plate's
    bottom corners and centres, each post's bottom centre."""
    pts = []
    for (x0, x1), (y0, y1), (z0, _z1) in rm.belly_boxes().values():
        pts += [(x, y, z0) for x in (x0, x1) for y in (y0, y1)] + [((x0 + x1) / 2, (y0 + y1) / 2, z0)]
    for (x, y), _r, (z0, _z1) in rm.belly_posts().values():
        pts.append((x, y, z0))
    return np.array(pts, float) / 1000.0


def _foot_normal_n(m, d, geom):
    """Normal force (N) on one foot geom, summed over its contacts."""
    f = np.zeros(6)
    tot = 0.0
    for k in range(d.ncon):
        c = d.contact[k]
        if int(c.geom1) == geom or int(c.geom2) == geom:
            mujoco.mj_contactForce(m, d, k, f)
            tot += abs(float(f[0]))
    return tot


class Meter:
    """Samples one run at the bus rate (and the servo load every physics step)."""

    def __init__(self, pg, row):
        m = pg.model
        self.pg, self.row = pg, row
        self.gdir = gravity_dir(m)
        self.up = -self.gdir
        a = np.array([1.0, 0.0, 0.0]) if abs(self.up[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        e1 = a - np.dot(a, self.up) * self.up
        e1 /= np.linalg.norm(e1)
        self.E = np.array([e1, np.cross(self.up, e1), self.up])          # world -> gravity frame
        self.r_foot = np.array([float(m.geom_size[f][0]) for f in pg.fids])
        self.belly_pts = _belly_points_m()
        self.belly = set(rc.belly_geoms(m))
        self.root = int(m.body_rootid[pg.torso])
        self.vadr = np.array(rc.joint_addrs(m)[1])
        self.stall = rm.stall_nm()
        self.h0 = rm.stance_torso_z_m() * 1000.0
        self.geomid = np.zeros(1, np.int32)
        self.p0 = pg.data.xpos[pg.torso][:2].copy()
        yaw0 = pg.yaw()
        self.u_cmd = np.array([np.cos(yaw0), np.sin(yaw0)])               # walk +x at the start heading
        c = row.get("cmd")
        if c is not None and np.hypot(c[0], c[1]) > 0:                    # a motion row: its own heading
            u = np.array([c[0], c[1]]) / np.hypot(c[0], c[1])
            self.u_cmd = np.array([[np.cos(yaw0), -np.sin(yaw0)], [np.sin(yaw0), np.cos(yaw0)]]) @ u
        self.gate0 = int(pg.gate_count)
        self.tilt, self.roll, self.pitch, self.t = [], [], [], []
        self.herr, self.clear = [], []
        self.belly_ticks = 0
        self.probe_edges = 0
        self._out_prev = np.zeros(len(pg.fids), bool)
        self.lv_ticks = self.lv_sat = self.lv_cap = 0
        self.ls, self.derate, self._bare = [], [], {}   # B162: per walking tick, the plane slope and its derate
        self.rough = []                                 # B162: the leveler's held roughness, per enabled tick
        self.lv_max = 0.0
        self.probe_max = 0.0
        self.heat_max = 0.0
        self.min_loaded = len(pg.fids)
        self.x2 = np.zeros(15)
        self.x_peak = 0.0
        self.n_phys = 0
        self.digest = hashlib.sha256()
        self.t_fire = None
        self.void_ground_mm = None
        self.void_ground_nom_mm = None
        self.tilt_at_fire = None
        self.stone = row.get("stone_foot")
        self.stone_n, self.stone_closed = [], []
        self.zmin = float(pg.data.xpos[pg.torso][2])
        self.progress_stop_mm = None                    # B162: a motion row's progress when `stop` is sent

    def mark_stop(self):
        """A motion row's progress along its command when `stop` is sent: the walk's own,
        before the stop's BRACE / RECOVER and what the stance does after it (B163)."""
        disp = self.pg.data.xpos[self.pg.torso][:2] - self.p0
        self.progress_stop_mm = round(1000.0 * float(disp @ self.u_cmd), 1)

    def _ray(self, p):
        d = mujoco.mj_ray(self.pg.model, self.pg.data, np.asarray(p, float), self.gdir, None, 1,
                          int(self.pg.torso), self.geomid)
        return None if d < 0 else 1000.0 * float(d)

    def physics(self, t_row):
        """Every physics step: servo load, the void's fire."""
        pg, d = self.pg, self.pg.data
        x = np.abs(rc.motor_torque(pg.model, d, self.vadr)) / self.stall
        self.x2 += x * x
        self.x_peak = max(self.x_peak, float(x.max()))
        self.n_phys += 1
        self.zmin = min(self.zmin, float(d.xpos[pg.torso][2]))
        if self.t_fire is None and pg.void is not None:
            self.t_fire = t_row
            self.tilt_at_fire = self._tilt()
            leg = int(pg.void["leg"])
            fp = d.geom_xpos[pg.fids[leg]] + (self.r_foot[leg] + 1e-3) * self.gdir
            self.void_ground_mm = self._ray(fp)
            if self.void_ground_mm is not None:
                # + how far the foot is below its un-probed target along up (the probe's own
                # meas: the target against FK of the measured joints, in the gait's foot
                # convention): the probe's reach (PROBE_MAX) is from there
                raw = getattr(pg.sup, "last_feet_raw", None)
                if raw is not None:
                    import pebble_feasibility as pf
                    q = d.qpos[pg._jadr].reshape(len(pg.fids), 3)
                    up_b = -grav_body(pg.model, d, pg.torso)
                    low = float(up_b @ (np.asarray(raw[leg], float) - pf.feet_body(q)[leg]))
                    self.void_ground_nom_mm = self.void_ground_mm + low

    def _tilt(self):
        return float(np.degrees(tilt_from_grav(grav_body(self.pg.model, self.pg.data, self.pg.torso))))

    def sample(self, t_row):
        """Every TICK_S of the row."""
        pg, d, m = self.pg, self.pg.data, self.pg.model
        g = grav_body(m, d, pg.torso)
        self.t.append(t_row)
        self.tilt.append(float(np.degrees(tilt_from_grav(g))))
        self.roll.append(float(np.degrees(np.arctan2(-g[1], -g[2]))))
        self.pitch.append(float(np.degrees(np.arctan2(g[0], np.hypot(g[1], g[2])))))
        con = np.asarray(pg.last["con"], bool)
        self.min_loaded = min(self.min_loaded, int(con.sum()))
        if con.sum() >= 3:
            low = np.array([d.geom_xpos[f] for f in pg.fids])[con] - self.r_foot[con, None] * self.up
            q = low @ self.E.T
            A = np.c_[q[:, 0], q[:, 1], np.ones(len(q))]
            k = np.linalg.lstsq(A, q[:, 2], rcond=None)[0]
            tq = self.E @ d.xpos[pg.torso]
            self.herr.append(1000.0 * float(tq[2] - (k[0] * tq[0] + k[1] * tq[1] + k[2])) - self.h0)
        R = d.xmat[pg.torso].reshape(3, 3)
        cl = [self._ray(d.xpos[pg.torso] + R @ p) for p in self.belly_pts]
        cl = [c for c in cl if c is not None]
        if cl:
            self.clear.append(min(cl))
        for c in d.contact[:d.ncon]:
            g1, g2 = int(c.geom1), int(c.geom2)
            if (g1 in self.belly) != (g2 in self.belly):
                other = g2 if g1 in self.belly else g1
                if int(m.body_rootid[m.geom_bodyid[other]]) != self.root:
                    self.belly_ticks += 1
                    break
        out = np.asarray(pg.probe_out, bool)
        self.probe_edges += int(np.count_nonzero(out & ~self._out_prev))
        self._out_prev = out.copy()
        self.probe_max = max(self.probe_max, float(np.max(pg.probe_dz)))
        lv, dz = _level(pg), getattr(pg.sup, "level_dz", None)
        if dz is not None:
            self.lv_max = max(self.lv_max, float(np.max(np.abs(dz))))
        if lv is not None and lv.enabled:
            self.lv_ticks += 1
            self.lv_sat += int(bool(lv.saturated))
            self.lv_cap += int(bool(getattr(lv, "sink_capped", False)))
            self.rough.append(float(getattr(lv, "rough_mm", 0.0)))
        ask = np.asarray(pg.cmd_v, float)
        if self.row["kind"] in ("walk", "cliff") and ask.any():
            # B162 (P4): the envelope the derated plane allowed this tick against the bare one, for
            # this ask (WaveGait.budget; the leveler's plane slope as the Playground quantizes it)
            ls = float(pg._level_slope())
            key = (round(float(ask[0]), 4), round(float(ask[1]), 4), round(float(ask[2]), 5))
            bare = self._bare.get(key)
            if bare is None:
                bare = self._bare[key] = float(np.linalg.norm(pg.gait.budget(*key)))
            self.ls.append(ls)
            self.derate.append(float(np.linalg.norm(pg._budget(ask))) / bare if bare > 0 else 1.0)
        th = pg.thermal
        self.heat_max = max(self.heat_max, float(np.max(th.heat / th.budget)))
        if self.stone is not None:
            self.stone_n.append(_foot_normal_n(m, d, pg.fids[self.stone]))
            self.stone_closed.append(bool(con[self.stone]))
        self.digest.update(d.qpos.tobytes())

    def _void_class(self, void):
        """B162 (P6), a void split by the probe's reach: None (no void) | 'false' (the D065
        false_void within reach: ground within PROBE_MAX of the un-probed target, or on a
        world with no drop its depth unknown) | 'probe_reach' (a world with no drop, ground
        PROBE_MAX or more under the un-probed target, or none found: the probe cannot reach
        it by design, a safe stop; B161) | 'drop' (a drop world, ground beyond the probe's
        reach or none found: the guard's job). 'false' + 'probe_reach' = D065's false_void."""
        if not void:
            return None
        nom, under = self.void_ground_nom_mm, self.void_ground_mm
        if self.row["drop"]:
            return "false" if nom is not None and nom < PROBE_MAX else "drop"
        if nom is not None:
            deep = nom >= PROBE_MAX
        else:                                     # no ground found, or its lower bound (the lowered foot's)
            deep = under is None or under >= PROBE_MAX
        return "probe_reach" if deep else "false"

    def _settle_s(self, T, t):
        """B162: how long the tilt takes to settle for good within SETTLE_DEG of tilt_end:
        a stand from the spawn (the settle's samples included: the leveler is on from the
        first step), a shove from the push. 0 = never left the band."""
        end = float(T[t >= t[-1] - END_S + 1e-9].mean())
        if self.row["kind"] == "shove":
            t0 = float(self.row["shove"][3])
            ts, xs = t[t >= t0] - t0, T[t >= t0]
        else:
            pre = getattr(self, "pre", [])
            ts = np.r_[[p[0] for p in pre], t + T_SETTLE]
            xs = np.r_[[p[1] for p in pre], T]
        out = np.nonzero(np.abs(xs - end) > SETTLE_DEG)[0]
        return 0.0 if not len(out) else round(float(ts[out[-1] + 1] if out[-1] + 1 < len(ts) else ts[-1]), 2)

    def result(self):
        pg, row = self.pg, self.row
        T = np.array(self.tilt)
        t = np.array(self.t)
        rms = lambda a: round(float(np.sqrt(np.mean(np.square(a)))), 3) if len(a) else None  # noqa: E731
        disp = pg.data.xpos[pg.torso][:2] - self.p0
        fell = bool(pg.sup.fall_count > 0 or (len(T) and T.max() > FALL_TILT_DEG))
        lr = np.sqrt(self.x2 / max(self.n_phys, 1))
        void = pg.void is not None
        tip_deg = row["slope_deg"] + TIPPED_DEG       # the excursion past the world's own tilt
        # no drop in the world: any void stops the robot on walkable ground; on a drop world a
        # void is false when the ground was within the probe's reach of the un-probed target
        fv = bool(void and (not row["drop"] or (self.void_ground_nom_mm is not None
                                                 and self.void_ground_nom_mm < PROBE_MAX)))
        r = dict(
            tilt_rms_deg=rms(T), tilt_p95_deg=round(float(np.percentile(T, 95)), 3),
            tilt_peak_deg=round(float(T.max()), 3),
            tilt_end_deg=round(float(T[t >= t[-1] - END_S + 1e-9].mean()), 3),
            roll_rms_deg=rms(self.roll), pitch_rms_deg=rms(self.pitch),
            height_err_mean_mm=round(float(np.mean(self.herr)), 2) if self.herr else None,
            height_err_rms_mm=rms(self.herr) if self.herr else None,
            progress_mm=(round(1000.0 * float(disp @ self.u_cmd), 1)
                         if row["kind"] in ("walk", "cliff") or (row["kind"] == "motion" and any(row["cmd"][:2]))
                         else None),
            creep_mm=round(1000.0 * float(np.linalg.norm(disp)), 1) if row["kind"] in ("stand", "shove", "motion") else None,
            fell=fell, tipped=bool(len(T) and T.max() > tip_deg),
            fall_count=int(pg.sup.fall_count), trips=int(pg.sup.trip_count), latched=bool(pg.sup.latched),
            gate_holds=int(pg.gate_count) - self.gate0, probe_out_edges=self.probe_edges,
            probe_max_mm=round(self.probe_max, 1),
            void=void, void_t_s=None if self.t_fire is None else round(self.t_fire, 2),
            void_ground_mm=None if self.void_ground_mm is None else round(self.void_ground_mm, 1),
            void_ground_nom_mm=None if self.void_ground_nom_mm is None else round(self.void_ground_nom_mm, 1),
            void_nohit=bool(void and self.void_ground_mm is None),
            false_void=fv, void_class=self._void_class(void),
            min_clear_mm=round(min(self.clear), 1) if self.clear else None,
            belly_contact_s=round(self.belly_ticks * TICK_S, 2),
            min_loaded=int(self.min_loaded),
            load_rms_max=round(float(lr.max()), 3), load_peak=round(self.x_peak, 3),
            heat_max=round(self.heat_max, 3), thermal_trip=bool(self.heat_max > 1.0),
            level_sat_pct=round(100.0 * self.lv_sat / self.lv_ticks, 1) if self.lv_ticks else None,
            level_max_mm=round(self.lv_max, 2),
            level_sinkcap_pct=round(100.0 * self.lv_cap / self.lv_ticks, 1) if self.lv_ticks else None,
            level_rough_p95_mm=round(float(np.percentile(self.rough, 95)), 1) if self.rough else None,
            level_rough_max_mm=round(float(np.max(self.rough)), 1) if self.rough else None,
            nan_count=int(pg.nan_count), digest=self.digest.hexdigest()[:16])
        if row["kind"] == "motion" and any(row["cmd"][:2]):
            r["progress_stop_mm"] = self.progress_stop_mm
        if self.derate:
            r.update(level_slope_mean=round(float(np.mean(self.ls)), 5),
                     level_slope_mean_deg=round(float(np.degrees(np.arctan(np.mean(self.ls)))), 3),
                     derate_pct=round(100.0 * float(np.mean(self.derate)), 2))
        if row["kind"] in ("stand", "shove") and len(T):
            r["settle_s"] = self._settle_s(T, t)
        if row["kind"] == "shove" and len(T):
            # B162: tilt_peak_deg is the row's peak, and on a leveled slope that is the leveler still
            # converging when the row's clock starts (the 8 deg stand's own peak), not the shove: the
            # shove's own peak and its excursion over the 0.5 s before the push
            t0 = float(row["shove"][3])
            pre_s = T[(t > t0 - 0.5) & (t <= t0)]
            post = T[t > t0]
            r.update(shove_peak_deg=round(float(post.max()), 3) if len(post) else None,
                     shove_excursion_deg=(round(float(post.max() - pre_s.mean()), 3)
                                          if len(post) and len(pre_s) else None))
        if self.stone is not None:
            last = t >= t[-1] - END_S + 1e-9
            r.update(stone_foot_n=round(float(np.mean(np.array(self.stone_n)[last])), 2),
                     stone_foot_closed_pct=round(100.0 * float(np.mean(np.array(self.stone_closed)[last])), 1))
        if row["kind"] == "cliff":
            after = T[t > self.t_fire] if self.t_fire is not None else np.zeros(0)
            r.update(fired=void, tilt_at_fire_deg=None if self.tilt_at_fire is None else round(self.tilt_at_fire, 2),
                     tilt_after_deg=round(float(after.max()), 2) if len(after) else None,
                     torso_zmin_m=round(self.zmin, 4),
                     cliff_ok=bool(void and not fell and self.zmin > pg.z0 + 0.08
                                   and (not len(after) or after.max() <= tip_deg)))
        return r


# ---- one run -------------------------------------------------------------------------
def _run_sim():
    """run_sim.py's flat band: bare gait, stack-independent (the D064 walk / turn refs)."""
    with tempfile.TemporaryDirectory() as tmp:
        p = subprocess.run([sys.executable, os.path.join(HERE, "run_sim.py"), "--out", tmp],
                           capture_output=True, text=True, timeout=600,
                           env=dict(os.environ, MUJOCO_GL="disable"))
    walk = re.search(r"walk \+X displacement: (-?\d+) mm", p.stdout)
    turn = re.search(r"turn in place: (-?[\d.]+) deg", p.stdout)
    return dict(status="ok" if walk and turn else "error", exit_code=p.returncode,
                in_band=p.returncode == 0, walk_mm=float(walk.group(1)) if walk else None,
                turn_deg=float(turn.group(1)) if turn else None,
                why=None if walk and turn else p.stderr[-800:])


def run_one(row, stack, imu, seconds=None):
    """One row under one stack and IMU mode -> a JSON-safe dict (status ok | unavailable | error)."""
    t_wall = time.time()
    base = dict(row=row["id"], world=row["world"], kind=row["kind"], stack=stack, imu=imu)
    if row["kind"] == "run_sim":
        return dict(base, stack="-", imu="-", **_run_sim(), wall_s=round(time.time() - t_wall, 1))
    if stack not in STACKS:
        return dict(base, status="error", why=f"unknown stack {stack!r}")
    if stack == "S3":
        return dict(base, status="unavailable", why="S3 is reserved for the terrain RL residual (B160)")
    pg = _playground(row)
    why = _apply_imu(pg, imu, row["seed"]) or _apply_stack(pg, stack)
    if why:
        return dict(base, status="unavailable", why=why)
    n_tick = int(round(TICK_S / pg.DT))
    pre = []                                     # (t from the spawn, tilt): the settle's samples, for settle_s
    for k in range(int(round(T_SETTLE / pg.DT))):
        pg.step()
        if (k + 1) % n_tick == 0:
            pre.append(((k + 1) * pg.DT, float(np.degrees(tilt_from_grav(grav_body(pg.model, pg.data, pg.torso))))))
    why = (_check_latency(pg) if IMU_MODES[imu].get("latency_s") else None) or _check_level(pg, stack)
    if why:
        return dict(base, status="unavailable", why=why)
    meter = Meter(pg, row)
    meter.pre = pre
    secs = float(seconds if seconds is not None else row["seconds"])
    if row["kind"] in ("walk", "cliff", "motion"):
        pg.sup.t_gait = float(row["phase"]) * pg.gait.T
        cmd = row.get("cmd") or [row["walk"], 0.0, 0.0]
        reply = pg.do("walk " + " ".join(f"{x:g}" for x in cmd))
        if not reply.startswith("walking"):
            return dict(base, status="error", why=f"walk refused: {reply}")
    shove = row.get("shove")
    k_stop = int(round(row["stop_at"] / pg.DT)) if row.get("stop_at") is not None else None
    n = int(round(secs / pg.DT))
    for k in range(n):
        t_row = k * pg.DT
        if shove is not None and k == int(round(shove[3] / pg.DT)):
            pg.do(f"push {shove[0]:g} {shove[1]:g} {shove[2]:g}")
        if k == k_stop:
            meter.mark_stop()
            pg.do("stop")
        pg.step()
        meter.physics(t_row + pg.DT)
        if (k + 1) % n_tick == 0:
            meter.sample(t_row + pg.DT)
        if row["kind"] == "cliff" and meter.t_fire is not None and t_row > meter.t_fire + CLIFF_AFTER_S:
            break
        if row["kind"] == "cliff" and (pg.sup.fall_count or meter.zmin < pg.z0 + 0.08):
            break
    out = dict(base, status="ok", seed=row["seed"], seconds=round(len(meter.t) * TICK_S, 2), **meter.result())
    lv = _level(pg)
    if lv is not None:
        out["level_status"] = {k: v for k, v in lv.status().items() if k in ("enabled", "hold", "saturated",
                                                                               "slope_deg", "offsets")}
    out["wall_s"] = round(time.time() - t_wall, 1)
    return out


def _safe_run(row, stack, imu):
    try:
        return run_one(row, stack, imu)
    except Exception as e:                       # noqa: BLE001 — one broken run must not sink the bench
        import traceback
        return dict(row=row["id"], world=row["world"], kind=row["kind"], stack=stack, imu=imu,
                    status="error", why=f"{type(e).__name__}: {e}", trace=traceback.format_exc()[-1500:])


# ---- the plan, the rules, the record ---------------------------------------------------------
def plan(rows_, stacks, imus, imu_explicit=False):
    """(row, stack, imu) tasks. S0 is the probe-off reference: ideal only unless asked."""
    tasks, seen_run_sim = [], False
    for r in rows_:
        if r["kind"] == "run_sim":
            if not seen_run_sim:
                tasks.append((r, "-", "-"))
                seen_run_sim = True
            continue
        for s in stacks:
            for i in imus:
                if s == "S0" and i != "ideal" and not imu_explicit:
                    continue
                tasks.append((r, s, i))
    return tasks


def _idx(runs):
    return {(r["row"], r["stack"], r["imu"]): r for r in runs if r.get("status") == "ok"}


def add_ratios(runs):
    """pct_s1: a walk's progress as % of S1's same row and IMU mode."""
    ix = _idx(runs)
    for r in runs:
        b = ix.get((r["row"], "S1", r["imu"]))
        if r.get("status") == "ok" and r.get("progress_mm") is not None and b and b.get("progress_mm"):
            r["pct_s1"] = round(100.0 * r["progress_mm"] / b["progress_mm"], 1)


def evaluate(runs):
    """The S2 pass rules over the GATED_IMU modes in which both S1 and S2 ran:
    {rules: {rule: {status: pass | fail | unavailable | external, fails: [...], note}},
     d065: P1 / P4 / P6 under their D065 wording (B162 restated them; kept so the original
     verdict stays visible), reports: what is measured but not gated, verdict}."""
    ix = _idx(runs)
    pairs = [(row, imu, s1, ix[(row, "S2", imu)]) for (row, st, imu), s1 in ix.items()
             if st == "S1" and imu in GATED_IMU and (row, "S2", imu) in ix]
    rules, d065 = {}, {}

    def rule(name, note, bad, rows_=None, into=None):
        into = rules if into is None else into
        sel = [p for p in pairs if rows_ is None or rows_(p[0])]
        if not sel:
            into[name] = dict(status="unavailable", note=note, fails=[])
            return
        fails = [f for f in (bad(*p) for p in sel) if f]
        into[name] = dict(status="fail" if fails else "pass", note=note, n=len(sel), fails=fails)

    def f(row, imu, msg):
        return f"{row} [{imu}]: {msg}"

    def tilt_prog(a, b):
        msgs = []
        if b["tilt_rms_deg"] - a["tilt_rms_deg"] > 0.05:
            msgs.append(f"tilt RMS +{b['tilt_rms_deg'] - a['tilt_rms_deg']:.3f} deg")
        if a.get("progress_mm") and abs(b["progress_mm"] - a["progress_mm"]) > 0.01 * abs(a["progress_mm"]):
            msgs.append(f"progress {b['progress_mm']:.0f} vs {a['progress_mm']:.0f} mm")
        return msgs

    def p1_d065(row, imu, a, b):
        msgs = tilt_prog(a, b)
        if b["level_max_mm"] > 0.5:
            msgs.append(f"max offset {b['level_max_mm']} mm")
        if imu == "ideal" and b["digest"] != a["digest"]:
            msgs.append("not bit-identical to S1")
        return f(row, imu, "; ".join(msgs)) if msgs else None
    straight = lambda r: r.startswith("W1.flat.walk.") or r == "W1.flat.stand"     # noqa: E731
    flat_motion = lambda r: r.startswith("W1.") and not straight(r)                  # noqa: E731

    def at_stop(r):
        """A motion row judged on its walk: the progress when `stop` was sent (B162)."""
        return dict(r, progress_mm=r["progress_stop_mm"]) if r.get("progress_stop_mm") is not None else r

    def p1_motion(row, imu, a, b):
        msgs = tilt_prog(at_stop(a), at_stop(b))
        return f(row, imu, "; ".join(msgs)) if msgs else None
    rule("P1", "flat. The straight walks and the stand: S2 - S1 tilt RMS <= 0.05 deg, progress within 1 %, "
               "|offset| <= 0.5 mm; ideal: bit-identical. The turn in place, the walk + turn and its stop, the "
               "rim shoves: tilt RMS <= S1 + 0.05 deg and the walk's progress (at `stop`) within 1 %; their "
               "offsets, end progress and bit-identity reported in 'flat_motion' (B162 restatement: these rows "
               "tilt the body past the deadband for real, so a leveler acting on them is not drift; the walk + "
               "turn is bit-identical to S1 until `stop`, after which its offset and its few mm of extra travel "
               "are the leveler levelling the tilt the probe's HOLD leaves after a turning stop, B163)",
         lambda row, imu, a, b: p1_d065(row, imu, a, b) if straight(row) else p1_motion(row, imu, a, b),
         lambda r: r.startswith("W1."))
    rules["P1"]["flat_motion"] = [
        f(row, imu, f"max offset {b['level_max_mm']} mm, end tilt {a['tilt_end_deg']} -> {b['tilt_end_deg']} deg, "
                    f"tilt RMS {a['tilt_rms_deg']} -> {b['tilt_rms_deg']}"
                    + (f", progress at stop {a.get('progress_stop_mm')} -> {b.get('progress_stop_mm')} mm, at the end "
                       f"{a.get('progress_mm')} -> {b.get('progress_mm')}" if a.get("progress_stop_mm") is not None else "")
                    + f", bit-identical {b['digest'] == a['digest']}")
        for row, imu, a, b in pairs if flat_motion(row)]
    rule("P1", "D065 wording: flat (straight walks, the stand, a turn in place, a walk + turn and its stop, rim "
               "shoves): S2 - S1 tilt RMS <= 0.05 deg, progress within 1 %, |offset| <= 0.5 mm; ideal: "
               "bit-identical", p1_d065, lambda r: r.startswith("W1."), into=d065)
    rule("P2", "a 20 mm stone, standing: end tilt <= 1.0 deg (the stone foot's load is reported, not "
               "gated: it stands unloaded in S1 and S2 alike)",
         lambda row, imu, a, b: f(row, imu, f"end tilt {b['tilt_end_deg']} deg") if b["tilt_end_deg"] > 1.0 else None,
         lambda r: r.startswith("W3.stone20."))
    rules["P2"]["stone_foot"] = [f(row, imu, f"stone foot {a.get('stone_foot_n')} N / {b.get('stone_foot_n')} N, "
                                            f"switch {a.get('stone_foot_closed_pct')} / {b.get('stone_foot_closed_pct')} % "
                                            "(S1 / S2, last 2 s)")
                                 for row, imu, a, b in pairs if row.startswith("W3.")]
    lim3 = {"W2.slope8.": 3.5, "W2.slope5.": 0.6}
    rule("P3", "standing on 8 deg: end tilt <= 3.5 deg; on 5 deg: <= 0.6 deg",
         lambda row, imu, a, b: next((f(row, imu, f"end tilt {b['tilt_end_deg']} deg") for k, v in lim3.items()
                                      if row.startswith(k) and b["tilt_end_deg"] > v), None),
         lambda r: r.endswith(".stand") and any(r.startswith(k) for k in lim3))

    # P4: the derate (WaveGait.budget(level_slope=)) is the leveler's deliberate speed cost; judged on what
    # remains after it. Rubble seeds are chaotic per row (S1 alone moves 341-398 mm across IMU modes on one
    # seed): the family, per IMU mode
    walk = lambda r: ".walk" in r                                                   # noqa: E731
    rubble = lambda r: r.startswith("W4.")                                         # noqa: E731
    der = lambda b: float(b.get("derate_pct") or 100.0) / 100.0                    # noqa: E731
    rule("P4", "D065 wording: every walk: progress >= 95 % of S1's",
         lambda row, imu, a, b: (f(row, imu, f"{b['progress_mm']:.0f} vs {a['progress_mm']:.0f} mm")
                                 if a.get("progress_mm") and b["progress_mm"] < 0.95 * a["progress_mm"] else None),
         walk, into=d065)
    rule("P4", "every walk off rubble, per row: progress >= 95 % of S1's x the row's derate (derate_pct: the "
               "envelope the leveled plane allowed, tick by tick, against the bare one for the same ask: the "
               "deliberate cost, 94.97 % on a plane saturated along x); rubble: the family per IMU mode on the "
               "same line, the rows' spread reported; the stairs stay per row, their trade (progress and tips, "
               "S1 vs S2) reported in 'stairs' (B162 restatement: the derate is a design cost, not a loss, and "
               "the rubble seeds are chaotic per row: S1 alone walks 341-398 mm on one seed across IMU modes)",
         lambda row, imu, a, b: (f(row, imu, f"{b['progress_mm']:.0f} vs {a['progress_mm']:.0f} mm x derate "
                                             f"{100 * der(b):.2f} % = {100 * b['progress_mm'] / (a['progress_mm'] * der(b)):.1f} %")
                                 if a.get("progress_mm") and b["progress_mm"] < 0.95 * a["progress_mm"] * der(b) else None),
         lambda r: walk(r) and not rubble(r))
    fam = {}
    for imu in GATED_IMU:
        sel = [(row, a, b) for row, i, a, b in pairs if i == imu and walk(row) and rubble(row) and a.get("progress_mm")]
        if not sel:
            continue
        s1 = sum(a["progress_mm"] * der(b) for _, a, b in sel)
        s2 = sum(b["progress_mm"] for _, a, b in sel)
        per = [100.0 * b["progress_mm"] / (a["progress_mm"] * der(b)) for _, a, b in sel]
        fam[imu] = dict(n=len(sel), family_pct=round(100.0 * s2 / s1, 1), row_min_pct=round(min(per), 1),
                        row_max_pct=round(max(per), 1),
                        rows_under_95=[row for (row, a, b), p in zip(sel, per) if p < 95.0])
        if s2 < 0.95 * s1:
            rules["P4"]["fails"].append(f"W4 rubble family [{imu}]: {100.0 * s2 / s1:.1f} % of S1 x derate")
    rules["P4"]["rubble_family"] = fam
    stairs = {}
    for imu in GATED_IMU:
        sel = [(row, a, b) for row, i, a, b in pairs if i == imu and row.startswith("W6.")]
        if sel:
            stairs[imu] = dict(
                s1_tips=sum(int(bool(a.get("tipped"))) for _, a, _b in sel),
                s2_tips=sum(int(bool(b.get("tipped"))) for *_, b in sel),
                s1_tilt_rms=round(float(np.mean([a["tilt_rms_deg"] for _, a, _b in sel])), 2),
                s2_tilt_rms=round(float(np.mean([b["tilt_rms_deg"] for *_, b in sel])), 2),
                progress_pct=round(100.0 * sum(b["progress_mm"] for *_, b in sel)
                                   / sum(a["progress_mm"] for _, a, _b in sel), 1),
                rows=[f"{row}: {a['progress_mm']:.0f} -> {b['progress_mm']:.0f} mm "
                      f"({100.0 * b['progress_mm'] / (a['progress_mm'] * der(b)):.1f} % of S1 x derate), tilt RMS "
                      f"{a['tilt_rms_deg']} -> {b['tilt_rms_deg']}, tipped {bool(a.get('tipped'))} -> {bool(b.get('tipped'))}"
                      for row, a, b in sel if a.get("progress_mm")])
    rules["P4"]["stairs"] = stairs
    if rules["P4"]["status"] == "pass" and rules["P4"]["fails"]:
        rules["P4"]["status"] = "fail"

    rule("P5", "no new falls", lambda row, imu, a, b: f(row, imu, "falls") if b["fell"] and not a["fell"] else None)

    def cliff(row, imu, a, b):
        if a["kind"] != "cliff":
            return None
        return (f(row, imu, "never fired") if not b["fired"] else f(row, imu, "fell") if b["fell"]
                else f(row, imu, "not held") if a.get("cliff_ok") and not b.get("cliff_ok") else None)
    rule("P6", "D065 wording: every cliff approach fires without a fall, holds where S1 held; no new false void "
               "anywhere (a void on a world without a drop, or ground within PROBE_MAX of the un-probed target)",
         lambda row, imu, a, b: (cliff(row, imu, a, b) or (f(row, imu, "false void" + (" (no ground found under the foot)"
                                                                                       if b.get("void_nohit") else ""))
                                                            if b["false_void"] and not a["false_void"] else None)),
         into=d065)
    rule("P6", "every cliff approach fires without a fall, holds where S1 held; no new false void within the "
               "probe's reach (void_class 'false': ground within PROBE_MAX of the un-probed target, or its depth "
               "unknown). A void with the ground deeper than PROBE_MAX under the un-probed target on a world with "
               "no drop is a 'probe_reach' void (B161: the probe cannot reach it by design; S1 has the same class; "
               "a safe stop), reported, not gated (B162 restatement)",
         lambda row, imu, a, b: (cliff(row, imu, a, b) or (f(row, imu, "false void within the probe's reach")
                                                            if b.get("void_class") == "false"
                                                            and a.get("void_class") != "false" else None)))
    rules["P6"]["probe_reach"] = [f(row, imu, f"S1 {a.get('void_class')} / S2 {b.get('void_class')} "
                                             f"(ground {b.get('void_ground_mm')} mm under the foot, "
                                             f"{b.get('void_ground_nom_mm')} under its target)")
                                  for row, imu, a, b in pairs
                                  if "probe_reach" in (a.get("void_class"), b.get("void_class"))]
    rule("P7", "min belly clearance >= 25 mm; no new belly contact",
         lambda row, imu, a, b: (f(row, imu, f"clearance {b['min_clear_mm']} mm")
                                 if b["min_clear_mm"] is not None and b["min_clear_mm"] < 25.0
                                 else f(row, imu, f"belly contact {b['belly_contact_s']} s")
                                 if b["belly_contact_s"] > 0 and a["belly_contact_s"] == 0 else None))
    rule("P8", "load RMS <= 0.65 x stall, no thermal trip",
         lambda row, imu, a, b: (f(row, imu, f"load {b['load_rms_max']}") if b["load_rms_max"] > 0.65
                                 else f(row, imu, "thermal trip") if b["thermal_trip"] else None))
    rules["P9"] = dict(status="external", fails=[],
                       note="full gait + sim/tests green with level on: pytest, not this script")
    rule("R1", "no row's tilt RMS worse than S1's by > 0.1 deg",
         lambda row, imu, a, b: (f(row, imu, f"+{b['tilt_rms_deg'] - a['tilt_rms_deg']:.2f} deg")
                                 if b["tilt_rms_deg"] - a["tilt_rms_deg"] > 0.1 else None))
    reports = dict(
        settle_s=[f(row, imu, f"S1 {a.get('settle_s')} / S2 {b.get('settle_s')} s, end tilt {a.get('tilt_end_deg')} / "
                              f"{b.get('tilt_end_deg')} deg") for row, imu, a, b in pairs if a["kind"] == "stand"],
        shove=[f(row, imu, f"shove peak S1 {a.get('shove_peak_deg')} / S2 {b.get('shove_peak_deg')} deg, excursion "
                           f"{a.get('shove_excursion_deg')} / {b.get('shove_excursion_deg')} (the row's tilt_peak "
                           f"{a.get('tilt_peak_deg')} / {b.get('tilt_peak_deg')})") for row, imu, a, b in pairs if a["kind"] == "shove"])
    gated = [rules[k]["status"] for k in rules if k != "P9"]
    verdict = ("unavailable" if "unavailable" in gated else "fail" if "fail" in gated else "pass")
    v65 = [(d065.get(k) or rules[k])["status"] for k in rules if k != "P9"]
    verdict_d065 = ("unavailable" if "unavailable" in v65 else "fail" if "fail" in v65 else "pass")
    return dict(rules=rules, verdict=verdict, d065=d065, verdict_d065_wording=verdict_d065, reports=reports,
                gated_imu=list(GATED_IMU))


def _git(*args):
    try:
        return subprocess.run(["git", "-C", ROOT, *args], capture_output=True, text=True, timeout=30).stdout.strip()
    except Exception:                                # noqa: BLE001
        return None


CODE_PATHSPEC = (".", ":!*.md", ":!docs", ":!sim/out", ":!sim/experiments/results")


def code_diff_sha256():
    """sha256 of `git diff HEAD` over the code (not the docs or the records), None when the
    code is clean: a record run on a dirty tree names the exact code it ran on, checkable
    after the commit with `git diff HEAD~1 HEAD -- <CODE_PATHSPEC> | sha256sum`."""
    d = _git("diff", "HEAD", "--", *CODE_PATHSPEC)
    if not d:
        return None
    return hashlib.sha256((d + "\n").encode()).hexdigest()


def provenance(args=None):
    from model_fingerprint import robot_fingerprint
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    return dict(decision=DECISION, head=_git("rev-parse", "HEAD"),
                dirty=bool(_git("status", "--porcelain", "--untracked-files=no")),
                code_diff_sha256=code_diff_sha256(),
                fingerprint=robot_fingerprint(model), params_rev=rm.params_rev(),
                level=rm.params().get("level"), imu_modes=IMU_MODES, gated_imu=list(GATED_IMU),
                date=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
                mujoco=mujoco.__version__, righter="off", settle_s=T_SETTLE, tick_s=TICK_S,
                seed_tiers=dict(bench=list(BENCH_SEEDS), heldout=[HELDOUT_SEEDS.start, HELDOUT_SEEDS.stop - 1],
                                train_min=TRAIN_SEED_MIN),
                args=None if args is None else {k: v for k, v in vars(args).items()})


def _fmt(r):
    if r.get("status") != "ok":
        return f"  {r['row']:<28} {r['stack']:<3} {r['imu']:<8} {r.get('status')}: {r.get('why')}"
    if r["kind"] == "run_sim":
        return f"  {r['row']:<28} walk {r['walk_mm']:.0f} mm, turn {r['turn_deg']:.1f} deg, in band {r['in_band']}"
    prog = (f"prog {r['progress_mm']:6.0f}" if r.get("progress_mm") is not None else f"creep {r['creep_mm']:5.0f}")
    extra = ""
    if r["kind"] == "cliff":
        extra = f" fired {r['fired']} ok {r['cliff_ok']} t {r['void_t_s']}"
    return (f"  {r['row']:<28} {r['stack']:<3} {r['imu']:<8} tilt rms {r['tilt_rms_deg']:5.2f} pk "
            f"{r['tilt_peak_deg']:5.2f} end {r['tilt_end_deg']:5.2f} | h {r['height_err_mean_mm']} | {prog} | "
            f"fell {int(r['fell'])} holds {r['gate_holds']} pout {r['probe_out_edges']} fv {int(r['false_void'])} | "
            f"clear {r['min_clear_mm']} | load {r['load_rms_max']} | lv {r['level_max_mm']} | "
            f"{r['wall_s']} s{extra}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quick", action="store_true", help="one phase, one seed (the cliff grid stays whole)")
    ap.add_argument("--jobs", type=int, default=1, help="sim processes at a time (keep <= 6)")
    ap.add_argument("--one", metavar="ROW", help="run one row (its id, see --list) and print its JSON")
    ap.add_argument("--stack", default="S0,S1,S2", help="comma list of S0 S1 S2 S3 (default S0,S1,S2)")
    ap.add_argument("--imu", default=None, help="comma list of ideal noisy mount1.5 (default: all; S0 ideal)")
    ap.add_argument("--tier", default="bench", choices=("bench", "heldout"), help="rubble seeds (default bench 0-4)")
    ap.add_argument("--list", action="store_true", help="print the rows and their sim seconds")
    ap.add_argument("--out", help="JSON path (default sim/out/terrain_bench.json, --quick: terrain_bench_quick.json)")
    a = ap.parse_args(argv)
    stacks = [s.strip() for s in a.stack.split(",") if s.strip()]
    imus = [s.strip() for s in (a.imu or ",".join(IMU_MODES)).split(",") if s.strip()]
    for s in stacks:
        if s not in STACKS:
            ap.error(f"unknown stack {s!r}")
    for i in imus:
        if i not in IMU_MODES:
            ap.error(f"unknown IMU mode {i!r}")
    rs = rows(quick=a.quick, tier=a.tier)
    if a.list:
        for r in rs:
            print(f"  {r['id']:<28} {r['kind']:<7} {r['seconds']:5.0f} s  seed {r['seed']}")
        print(f"{len(rs)} rows, {sum(r['seconds'] + T_SETTLE for r in rs if r['kind'] != 'run_sim'):.0f} "
              f"sim s per stack and IMU mode (cliff rows: at most)")
        return 0
    if a.one:
        rs = [row_by_id(a.one, tier=a.tier)]
    tasks = plan(rs, stacks, imus, imu_explicit=a.imu is not None)
    tasks.sort(key=lambda x: -(x[0]["seconds"] if x[0]["kind"] != "run_sim" else 30.0))
    t0 = time.time()
    runs = []
    if a.jobs <= 1 or len(tasks) == 1:
        for task in tasks:
            runs.append(_safe_run(*task))
            print(_fmt(runs[-1]), flush=True)
    else:
        for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
            os.environ.setdefault(k, "1")
        import multiprocessing as mp
        with ProcessPoolExecutor(max_workers=a.jobs, mp_context=mp.get_context("spawn")) as ex:
            futs = [ex.submit(_safe_run, *task) for task in tasks]
            for fu in as_completed(futs):
                runs.append(fu.result())
                print(_fmt(runs[-1]), flush=True)
    order = {r["id"]: k for k, r in enumerate(rs)}
    runs.sort(key=lambda r: (order.get(r["row"], 1e9), r["stack"], r["imu"]))
    add_ratios(runs)
    if a.one:
        print(json.dumps(runs, indent=1))
        return 0
    rec = dict(bench="terrain_bench", quick=bool(a.quick), tier=a.tier, provenance=provenance(a),
               wall_s=round(time.time() - t0, 1), rows=rs, runs=runs, verdict=evaluate(runs))
    out = a.out or os.path.join(OUT_DIR, "terrain_bench_quick.json" if a.quick else "terrain_bench.json")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w") as fh:
        json.dump(rec, fh, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
    v = rec["verdict"]
    print(f"{len(runs)} runs in {rec['wall_s']:.0f} s -> {os.path.relpath(out, ROOT)}; verdict {v['verdict']}: "
          + ", ".join(f"{k} {r['status']}" for k, r in v["rules"].items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
