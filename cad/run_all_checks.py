#!/usr/bin/env python3
"""Run EVERY part module's built-in checks — the CAD tree's CI.

Each part file self-verifies in its __main__ (boolean interference, layout
audits, engagement/retention, bed fit, keep-outs...). This runs them all
and summarizes: one command answers "does the whole tree still make
sense" after any params.yaml or interface change.

    python3 run_all_checks.py            # full tree, PARALLEL (default)
    python3 run_all_checks.py --serial   # one at a time (easier log reading)
    python3 run_all_checks.py part_shell part_deck   # subset
    python3 run_all_checks.py --derived  # + regenerate the derived outputs
    python3 run_all_checks.py --fem      # + the leg stress check (fem_check, D061)

Parallel notes: modules are already independent subprocesses, so they run
N-at-a-time (N = cpu count, min 2). Each module only writes its OWN exports
— no shared files, no races. Results print in completion order; the summary
is the same either way. ~150 s wall on a laptop (part_hand is the long pole).

The servo model is checked first against the reference STEP (servo_st3215:
cad/ref/STS3215_03a.step); servo_mount and part_port_coupon are the only
writers of servo_cup / port_coupon_* that check_printability audits;
leg_assembly writes the posed dry-fit exports the viewer shows; and
check_interference sweeps the yaw stage with the real servo solids.

--derived (after a clean tree) rebuilds the outputs nothing checks but
people look at: pentapod_preview (full-robot meshes + render), print_estimate
(cad/out/print_estimate.json), gen_print_pack (cad/out/PRINT_PREP_PACK.pdf +
views_*.png), make_viewer (cad/pebble_viewer.html) and gen_drawings (the
leg parts' TechDraw sheets in cad/out/drawings/; needs FreeCAD, SKIPPED
without it). The first four write deterministic bytes and gen_drawings
redraws only a part whose STEP changed, so an unchanged tree leaves git clean.

--fem (after a clean tree) runs fem_check: linear-static FEA of the load-
bearing leg parts under servo-limited loads (cad/out/fem/FEM_REPORT.md). It
needs gmsh + CalculiX (the FreeCAD Flatpak carries both) and reports SKIPPED
without them, so CI stays as it is.

Exit code = number of failing modules.
"""
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

MODULES = [
    "servo_st3215", "servo_mount", "part_port_coupon",
    "part_coxa", "part_femur", "part_tibia", "part_hand", "part_deck",
    "part_panel", "part_battery", "part_avionics", "part_bench_jig",
    "part_coupler", "part_footpad", "part_fit_ladder", "part_smallwins",
    "part_shell", "part_busboard", "part_stand", "part_tools",
    "part_clips", "part_dock", "part_servo_blank",
    "part_leg_coupons", "check_assembly",        # D046: joint coupons + joint suite
    "leg_assembly", "check_interference",
]
TIMEOUT_S = 900


def run_one(m):
    t0 = time.time()
    try:
        env = dict(os.environ, MPLBACKEND="Agg")
        r = subprocess.run([sys.executable, f"{m}.py"], cwd=HERE, env=env,
                           capture_output=True, text=True, timeout=TIMEOUT_S)
        ok = r.returncode == 0
        tail = (r.stdout + r.stderr).strip().split("\n")[-1][:90]
    except subprocess.TimeoutExpired:
        ok, tail = False, f"TIMEOUT ({TIMEOUT_S} s)"
    except FileNotFoundError:
        ok, tail = False, "missing file"
    return (m, ok, time.time() - t0, tail)


POST = ["check_printability"]     # runs AFTER every module has exported (D038)
DERIVED = ["pentapod_preview", "print_estimate", "gen_print_pack", "make_viewer", "gen_drawings"]
HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    args = sys.argv[1:]
    serial = "--serial" in args
    mods = [a for a in args if not a.startswith("--")] or MODULES
    t00 = time.time()
    results = []

    def record(res):
        m, ok, dt, tail = res
        results.append(res)
        print(f"{'PASS' if ok else 'FAIL':4s}  {m:18s} {dt:5.0f}s  {tail}",
              flush=True)

    if serial or len(mods) == 1:
        for m in mods:
            record(run_one(m))
    else:
        workers = max(2, os.cpu_count() or 2)
        with ThreadPoolExecutor(max_workers=workers) as ex:
            for res in ex.map(run_one, mods):
                record(res)
    # post stage: whole-tree audits that need every export on disk
    if mods == MODULES and "--no-post" not in args:
        for m in POST:
            record(run_one(m))
    if "--derived" in args:
        if any(not ok for _, ok, *_ in results):
            print("derived outputs NOT rebuilt: the checks failed")
        else:
            for m in DERIVED:                  # in order: each reads the one before
                record(run_one(m))
    if "--fem" in args:
        if any(not ok for _, ok, *_ in results):
            print("fem_check NOT run: the checks failed")
        else:
            record(run_one("fem_check"))
    bad = [m for m, ok, *_ in results if not ok]
    print(f"\n{len(results) - len(bad)}/{len(results)} modules pass "
          f"({time.time()-t00:.0f} s total)"
          + (f" — FAILING: {', '.join(bad)}" if bad else " — TREE CLEAN"))
    sys.exit(len(bad))


if __name__ == "__main__":
    main()
