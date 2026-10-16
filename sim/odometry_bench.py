"""odometry_bench — how far the robot's OWN pose estimate drifts from the truth.

sim/odometry.py runs the legged-odometry EKF on what the hardware will measure
(noisy, biased IMU; encoder + speed telemetry; foot switches). This drives the
cockpit's own pipeline (sim/playground.py: the supervisor, the gait budget, the
servo model; flat floor, righter off) along standard paths and compares the
estimate with MuJoCo's torso at the end of each and at its worst along the way:

  straight_1m   stand 3 s, walk forward until the TRUE torso is 1 m out, stand 3 s
  turn_90       stand 3 s, turn in place until the true yaw is +90 deg, stand 3 s
  square_1m     four (1 m straight, stand 2 s, +90 deg turn, stand 2 s): back at the start
  mixed_180s    3 minutes of seeded random segments: forward / back / arc / turn /
                sidestep / stand (2-8 s each, inside the gait envelope)

Every path from a fresh spawn, for each --seeds noise seed (the IMU / encoder
noise draws; the turn-on bias is the scenes.py calibration). The bench decides
when a segment ends from the truth — it is the experimenter, the estimate never
sees it.

    MUJOCO_GL=disable .venv/bin/python sim/odometry_bench.py              # 3 seeds -> the record
    MUJOCO_GL=disable .venv/bin/python sim/odometry_bench.py --seeds 0 --paths straight_1m --out /tmp/x.json

Record: sim/out/odometry_bench.json (commit it with the docs that quote it).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from odometry import CockpitOdometry                      # noqa: E402

OUT = os.path.join(HERE, "out", "odometry_bench.json")
PATHS = ("straight_1m", "turn_90", "square_1m", "mixed_180s")
VX = 34.0               # mm/s: the D063 envelope's forward speed (34.2)
WZ = 0.18               # rad/s: inside the 0.185 turn envelope
SETTLE_S = 3.0


def _truth(pg):
    p = pg.data.xpos[pg.torso]
    R = pg.data.xmat[pg.torso].reshape(3, 3)
    return {"x": float(p[0]), "y": float(p[1]), "yaw_deg": math.degrees(math.atan2(R[1, 0], R[0, 0]))}


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


class Run:
    def __init__(self, seed):
        from playground import Playground
        self.pg = Playground()
        self.pg.do("righter off")
        self.odo = CockpitOdometry(self.pg.model, seed=seed)
        self.odo.reset(_truth(self.pg), self.pg.data)
        self.path_true = 0.0
        self.turn_true = 0.0
        self.worst = {"pos_m": 0.0, "yaw_deg": 0.0}
        self._prev = _truth(self.pg)
        self.wall = 0.0
        self.ticks = 0

    def step(self, v):
        pg = self.pg
        pg.cmd_v[:] = v
        pg.step()
        moving = bool(np.any(pg.cmd_v)) or pg.sup.state != "NORMAL" or getattr(pg, "_ges", None) is not None
        t0 = time.perf_counter()
        self.odo.step(pg.model, pg.data, pg.t, moving, pg.last["con"], pg.sup.state)
        self.wall += time.perf_counter() - t0
        self.ticks += 1
        tr = _truth(pg)
        self.path_true += math.hypot(tr["x"] - self._prev["x"], tr["y"] - self._prev["y"])
        self.turn_true += _wrap(tr["yaw_deg"] - self._prev["yaw_deg"])
        self._prev = tr
        if self.ticks % 50 == 0:
            d = self.odo.drift(tr)
            self.worst["pos_m"] = max(self.worst["pos_m"], d["pos_m"])
            self.worst["yaw_deg"] = max(self.worst["yaw_deg"], abs(d["yaw_deg"]))
        return tr

    def hold(self, secs, v=(0.0, 0.0, 0.0)):
        for _ in range(int(round(secs / self.pg.DT))):
            self.step(v)

    def forward_until(self, dist_m, cap_s=60.0):
        start = _truth(self.pg)
        for _ in range(int(cap_s / self.pg.DT)):
            tr = self.step((VX, 0.0, 0.0))
            if math.hypot(tr["x"] - start["x"], tr["y"] - start["y"]) >= dist_m:
                return
        raise RuntimeError(f"did not cover {dist_m} m in {cap_s} s")

    def turn_until(self, deg, cap_s=30.0):
        start = self.turn_true
        for _ in range(int(cap_s / self.pg.DT)):
            self.step((0.0, 0.0, math.copysign(WZ, deg)))
            if abs(self.turn_true - start) >= abs(deg):
                return
        raise RuntimeError(f"did not turn {deg} deg in {cap_s} s")

    def result(self, name, seed):
        tr = _truth(self.pg)
        d = self.odo.drift(tr)
        od = self.odo.odometer()
        st = self.odo.status()
        return {"path": name, "seed": seed, "sim_s": round(self.pg.t, 1),
                "true": {k: round(v, 3) for k, v in tr.items()}, "estimate": self.odo.estimate(),
                "path_true_m": round(self.path_true, 3), "path_est_m": od["dist_m"],
                "turn_true_deg": round(self.turn_true, 1), "turn_est_deg": od["turned_deg"],
                "end_drift": d,
                "end_drift_pct_of_path": round(100 * d["pos_m"] / max(self.path_true, 1e-6), 1),
                "worst_drift": {k: round(v, 3) for k, v in self.worst.items()},
                "sigma": st["sigma"], "zupts": st["zupts"], "gyro_bias_z_err": st["gyro_bias_z_err"],
                "events": st["events"], "falls_or_suspends": sum(1 for e in self.odo.events if "suspended" in e[1]),
                "ekf_us_per_physics_tick": round(1e6 * self.wall / max(self.ticks, 1), 1),
                "ekf_us_per_imu_tick": st["cost_us"]}


def run_path(name, seed):
    r = Run(seed)
    r.hold(SETTLE_S)
    if name == "straight_1m":
        r.forward_until(1.0)
        r.hold(SETTLE_S)
    elif name == "turn_90":
        r.turn_until(90.0)
        r.hold(SETTLE_S)
    elif name == "square_1m":
        for _ in range(4):
            r.forward_until(1.0)
            r.hold(2.0)
            r.turn_until(90.0)
            r.hold(2.0)
    elif name == "mixed_180s":
        rng = np.random.default_rng(1000 + seed)
        moves = [(VX, 0, 0), (-0.6 * VX, 0, 0), (VX, 0, WZ * 0.5), (VX, 0, -WZ * 0.5),
                 (0, 0, WZ), (0, 0, -WZ), (0, 0.6 * VX, 0), (0, 0, 0)]
        while r.pg.t < SETTLE_S + 180.0:
            v = moves[rng.integers(len(moves))]
            r.hold(float(rng.uniform(2.0, 8.0)), v)
        r.hold(SETTLE_S)
    else:
        raise ValueError(name)
    return r.result(name, seed)


def _git(*a):
    try:
        return subprocess.run(["git", *a], capture_output=True, text=True, cwd=HERE, timeout=10).stdout.strip()
    except Exception:                                   # noqa: BLE001
        return ""


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--paths", default=",".join(PATHS))
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args(argv)
    seeds = [int(s) for s in a.seeds.split(",") if s.strip()]
    paths = [p for p in a.paths.split(",") if p.strip()]
    rows = []
    for name in paths:
        for seed in seeds:
            t0 = time.time()
            row = run_path(name, seed)
            row["wall_s"] = round(time.time() - t0, 1)
            rows.append(row)
            d = row["end_drift"]
            print(f"{name:12s} seed {seed}: path {row['path_true_m']:.2f} m (est {row['path_est_m']:.2f}), "
                  f"turn {row['turn_true_deg']:+.1f} (est {row['turn_est_deg']:+.1f}) deg | end drift "
                  f"{1000 * d['pos_m']:.0f} mm ({row['end_drift_pct_of_path']:.1f} %), yaw {d['yaw_deg']:+.1f} deg | "
                  f"worst {1000 * row['worst_drift']['pos_m']:.0f} mm / {row['worst_drift']['yaw_deg']:.1f} deg | "
                  f"{row['ekf_us_per_imu_tick']:.0f} us/EKF tick", flush=True)
    summary = {}
    for name in paths:
        rs = [r for r in rows if r["path"] == name]
        summary[name] = {
            "end_pos_mm_mean": round(1000 * float(np.mean([r["end_drift"]["pos_m"] for r in rs])), 1),
            "end_pos_mm_max": round(1000 * float(np.max([r["end_drift"]["pos_m"] for r in rs])), 1),
            "end_pct_of_path_mean": round(float(np.mean([r["end_drift_pct_of_path"] for r in rs])), 2),
            "end_yaw_deg_absmean": round(float(np.mean([abs(r["end_drift"]["yaw_deg"]) for r in rs])), 2),
            "end_yaw_deg_absmax": round(float(np.max([abs(r["end_drift"]["yaw_deg"]) for r in rs])), 2),
            "worst_pos_mm_max": round(1000 * float(np.max([r["worst_drift"]["pos_m"] for r in rs])), 1),
        }
    rec = {"bench": "odometry_bench", "date": time.strftime("%Y-%m-%d"), "commit": _git("rev-parse", "--short", "HEAD"),
           "dirty": bool(_git("status", "--porcelain", "--", ".")),
           "setup": {"floor": "flat (pebble.xml)", "righter": "off", "vx_mm_s": VX, "wz_rad_s": WZ,
                     "noise": "sim/scenes.py IMU/encoder calibration", "seeds": seeds},
           "summary": summary, "runs": rows}
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(rec, f, indent=1)
        f.write("\n")
    print(json.dumps(summary, indent=1))
    print(f"-> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
