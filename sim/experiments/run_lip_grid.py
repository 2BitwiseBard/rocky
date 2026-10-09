"""The void guard's lip band (D052a, D063, D064): every approach to the cliff on a 1 deg grid.

The grid BUILD_LOG 9i / 9q and SIM_GUIDE ("the void guard's lip band") report: walk 15 / 25 /
35 / 45 x approach -30..60 deg (364 approaches; 35 and 45 are both fitted to the 34.2 mm/s
envelope, so they are one run twice), `--half` adds the 0.5 deg-offset grid (724). Each
approach is test_playground_guards._approach: Playground(cliff=True), righter off, the spawn
yawed to the approach, 1 s planted, `walk V`, run until FALLEN, the torso under 0.24 m, or 3 s
after the void fires (MAX_S at most: walk 15 at 48-60 deg needs more than 20 s to reach the
edge). Outcomes, as the tests count them:
  fall   FALLEN or the torso under 0.24 m (the D063 criterion)
  tip    no fall, but the tilt passed TIPPED_DEG (10) with no fire before it, or after the
         fire: D064's keel hang (the tub lands on the edge, the void fires late) is a tip
  stop   fired, held, no tip
  short  never reached the edge in MAX_S (no fire, no tip): never counted as a stop
Tilt is the TRUE attitude (sim_imu.grav_body), not the IMU's. `--level` runs it with the D065
body leveler on (`level on`), `--careful` with `set gate.wait 1`.

    MUJOCO_GL=egl .venv/bin/python sim/experiments/run_lip_grid.py [--jobs 6] [--half] [--level]
    ... --quick          walk 15 / 25 / 45 x 5..20 deg only (the band and its edges, 48 approaches)
Writes results/lip_grid[_level][_careful]_results.json (--level, --careful), or --out PATH.
"""
import argparse
import datetime
import json
import os
import sys
import time
import warnings
from concurrent.futures import ProcessPoolExecutor

import exp_paths as X                                     # noqa: E402  (sys.path: sim/, gait/, root)

SPEEDS = (15, 25, 35, 45)
ANGLES = (-30, 60)
QUICK_SPEEDS, QUICK_ANGLES = (15, 25, 45), (5, 20)
FLOOR_Z = 0.24           # the platform is 0.16 m; a standing torso is ~0.32, one hung on the keel 0.243
TIPPED_DEG = 10.0        # = test_playground_guards.TIPPED_DEG
AFTER_S, MAX_S = 3.0, 40.0


def approach(task):
    """One approach -> its outcome row."""
    import numpy as np
    import mujoco
    import rocky_model as rm
    from playground import Playground
    from sim_imu import grav_body, tilt_from_grav
    v, deg, level, careful = task
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        pg = Playground(cliff=True, level=bool(level))       # never the ROCKY_LEVEL default
    pg.do("righter off")
    if careful:
        pg.do("set gate.wait 1")
    a = np.radians(deg) / 2
    pg.data.qpos[3:7] = [np.cos(a), 0, 0, np.sin(a)]
    mujoco.mj_forward(pg.model, pg.data)
    for _ in range(int(round(1.0 / pg.DT))):
        pg.step()
    pg.do(f"walk {v:g}")
    belly = set(rm.belly_geom_names())
    names = {i: pg.model.geom(i).name for i in range(pg.model.ngeom)}
    zmin, t_fire, t_hold, n_hold, t_belly = 9.0, None, None, pg.gate_count, None
    tilt_max = tilt_after = 0.0
    t0 = pg.t
    for _ in range(int(round(MAX_S / pg.DT))):
        pg.step()
        z = float(pg.data.xpos[pg.torso][2])
        zmin = min(zmin, z)
        tilt = float(np.degrees(tilt_from_grav(grav_body(pg.model, pg.data, pg.torso))))
        tilt_max = max(tilt_max, tilt)
        if pg.gate_count != n_hold and t_fire is None:
            n_hold, t_hold = pg.gate_count, pg.t - t0
        if t_belly is None and any((names[int(c.geom1)] in belly) != (names[int(c.geom2)] in belly)
                                   and "platform" in (names[int(c.geom1)], names[int(c.geom2)])
                                   for c in pg.data.contact[:pg.data.ncon]):
            t_belly = pg.t - t0
        if t_fire is None and pg.void is not None:
            t_fire = pg.t - t0
        if t_fire is not None:
            tilt_after = max(tilt_after, tilt)
        if pg.sup.fall_count or zmin < FLOOR_Z or (t_fire is not None and pg.t - t0 > t_fire + AFTER_S):
            break
    fell = bool(pg.sup.fall_count > 0 or zmin < FLOOR_Z)
    tip = not fell and (tilt_after if t_fire is not None else tilt_max) > TIPPED_DEG
    out = "fall" if fell else "tip" if tip else "stop" if t_fire is not None else "short"
    r2 = lambda x: None if x is None else round(float(x), 2)            # noqa: E731
    return dict(v=v, deg=deg, outcome=out, fired=t_fire is not None,
                t_fire_s=r2(t_fire), t_hold_s=r2(t_hold), t_belly_s=r2(t_belly), tilt_max_deg=r2(tilt_max),
                tilt_after_deg=r2(tilt_after) if t_fire is not None else None, torso_zmin_m=round(zmin, 4),
                gate_holds=int(pg.gate_count), fall_count=int(pg.sup.fall_count))


def grid(quick=False, half=False):
    speeds, (lo, hi) = (QUICK_SPEEDS, QUICK_ANGLES) if quick else (SPEEDS, ANGLES)
    degs = [float(d) for d in range(lo, hi + 1)]
    if half:
        degs += [d + 0.5 for d in range(lo, hi)]
    return [(v, d) for v in speeds for d in sorted(degs)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--jobs", type=int, default=6, help="sim processes at a time (keep <= 6)")
    ap.add_argument("--quick", action="store_true", help="walk 15 / 25 / 45 x 5..20 deg")
    ap.add_argument("--half", action="store_true", help="add the 0.5 deg-offset grid (364 -> 724)")
    ap.add_argument("--level", action="store_true", help="the D065 body leveler on (`level on`)")
    ap.add_argument("--careful", action="store_true", help="`set gate.wait 1` (the careful walk)")
    ap.add_argument("--out", help="JSON path (default results/lip_grid[_level]_results.json)")
    a = ap.parse_args(argv)
    tasks = [(v, d, a.level, a.careful) for v, d in grid(a.quick, a.half)]
    t0 = time.time()
    for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(k, "1")
    import multiprocessing as mp
    with ProcessPoolExecutor(max_workers=max(1, a.jobs), mp_context=mp.get_context("spawn")) as ex:
        runs = list(ex.map(approach, tasks, chunksize=1))
    count = {o: sum(r["outcome"] == o for r in runs) for o in ("stop", "tip", "fall", "short")}
    bad = [r for r in runs if r["outcome"] != "stop"]
    after = [r["tilt_after_deg"] for r in runs if r["outcome"] == "stop"]
    for r in bad:
        print(f"  walk {r['v']:>2} @ {r['deg']:5.1f} deg: {r['outcome']:<4} fired {r['t_fire_s']} "
              f"tilt max {r['tilt_max_deg']} after {r['tilt_after_deg']} belly {r['t_belly_s']} "
              f"zmin {r['torso_zmin_m']}")
    import rocky_model as rm
    import mujoco
    from model_fingerprint import robot_fingerprint
    fp = robot_fingerprint(mujoco.MjModel.from_xml_path(X.MODEL_XML))
    rec = dict(experiment="lip_grid", fingerprint=fp, params_rev=rm.params_rev(), level=bool(a.level),
               careful=bool(a.careful), quick=bool(a.quick), half=bool(a.half), n=len(runs), count=count,
               worst_safe_tilt_after_deg=max(after) if after else None,
               date=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
               wall_s=round(time.time() - t0, 1), runs=runs)
    out = a.out or X.result("lip_grid" + "_level" * a.level + "_careful" * a.careful + "_results.json")
    with open(out, "w") as fh:
        json.dump(rec, fh, indent=1)
    print(f"{len(runs)} approaches in {rec['wall_s']:.0f} s on {fp}: {count['stop']} stop, "
          f"{count['tip']} tip, {count['fall']} fall, {count['short']} short; worst safe tilt after a fire "
          f"{rec['worst_safe_tilt_after_deg']} deg -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
