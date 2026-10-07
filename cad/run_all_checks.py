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
    python3 run_all_checks.py --jobs 6   # at most 6 modules at once (a shared machine)

Parallel notes: modules are already independent subprocesses, so they run
N-at-a-time (N = cpu count, min 2, or --jobs N). Each module only writes its OWN exports
— no shared files, no races. Results print in completion order; the summary
is the same either way. ~150 s wall on a laptop (part_hand is the long pole).

The servo model is checked first against the reference STEP (servo_st3215:
cad/ref/STS3215_03a.step); servo_mount and part_port_coupon are the only
writers of servo_cup / port_coupon_* that check_printability audits;
leg_assembly writes the posed dry-fit exports the viewer shows;
check_interference sweeps the yaw stage with the real servo solids; and
check_dock moves a coxa base onto the deck along stored dock paths (planar +
3D replay) and checks the docked seats (B117: the D020 port could not dock).

--derived (after a clean tree) rebuilds the outputs nothing checks but
people look at, in this order: pentapod_preview (full-robot meshes + render),
print_estimate (cad/out/print_estimate.json), gen_drawings (the leg parts'
TechDraw sheets in cad/out/drawings/; needs FreeCAD, SKIPPED without it),
gen_print_pack (cad/out/PRINT_PREP_PACK.pdf + views_*.png), make_viewer
(cad/pebble_viewer.html) and gen_assembly_views (cad/out/assembly/). All but
gen_drawings write deterministic bytes, and gen_drawings redraws only a part
whose STEP (or td_sheet.py, the drawer) changed, so an unchanged tree leaves
git clean.

--fem (after a clean tree) runs fem_check: linear-static FEA of the load-
bearing leg parts under servo-limited loads (cad/out/fem/FEM_REPORT.md). It
needs gmsh + CalculiX (the FreeCAD Flatpak carries both) and reports SKIPPED
without them, so CI stays as it is.

With both, one pass leaves a current print pack (B110): fem_check runs
BEFORE the derived outputs, and gen_drawings before gen_print_pack, because
the pack prints each leg part's FEM verdict (fem/fem_results.json) and merges
its TechDraw sheet (drawings/). A failing FEM verdict does not stop the
derived outputs: the pack is where a FAIL has to show. Until D063 the order
was derived-then-FEM, and a leg-part change needed a second pass to reach the
pack.

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
    "check_dock",                                # 2026-10-07: the I1 dock path + docked seats
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
# in order, each may read the ones before it: gen_drawings reads print_estimate.json (the
# leg batch, the grams in the title block); gen_print_pack reads print_estimate.json, the
# drawings and fem/fem_results.json (B110: so --fem runs before all of these)
DERIVED = ["pentapod_preview", "print_estimate", "gen_drawings", "gen_print_pack", "make_viewer",
           "gen_assembly_views"]
HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    args = sys.argv[1:]
    serial = "--serial" in args
    jobs = None
    if "--jobs" in args:
        k = args.index("--jobs")
        jobs = max(1, int(args[k + 1]))
        del args[k:k + 2]
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
        workers = jobs or max(2, os.cpu_count() or 2)
        with ThreadPoolExecutor(max_workers=workers) as ex:
            for res in ex.map(run_one, mods):
                record(res)
    # post stage: whole-tree audits that need every export on disk
    if mods == MODULES and "--no-post" not in args:
        for m in POST:
            record(run_one(m))
    checks_ok = all(ok for _, ok, *_ in results)
    if "--fem" in args:                        # B110: before the derived outputs, which print it
        if not checks_ok:
            print("fem_check NOT run: the checks failed")
        else:
            record(run_one("fem_check"))
    if "--derived" in args:
        if not checks_ok:                      # a FEM FAIL still rebuilds: the pack shows it
            print("derived outputs NOT rebuilt: the checks failed")
        else:
            for m in DERIVED:                  # in order: each reads the ones before
                record(run_one(m))
    bad = [m for m, ok, *_ in results if not ok]
    print(f"\n{len(results) - len(bad)}/{len(results)} modules pass "
          f"({time.time()-t00:.0f} s total)"
          + (f" — FAILING: {', '.join(bad)}" if bad else " — TREE CLEAN"))
    sys.exit(len(bad))


if __name__ == "__main__":
    main()
