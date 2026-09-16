#!/usr/bin/env python3
"""Run EVERY part module's built-in checks — the CAD tree's CI.

Each part file self-verifies in its __main__ (boolean interference, layout
audits, engagement/retention, bed fit, keep-outs...). This runs them all
and summarizes: one command answers "does the whole tree still make
sense" after any params.yaml or interface change.

    python3 run_all_checks.py            # full tree, PARALLEL (default)
    python3 run_all_checks.py --serial   # one at a time (easier log reading)
    python3 run_all_checks.py part_shell part_deck   # subset

Parallel notes (session 6): modules are already independent subprocesses,
so they run N-at-a-time (N = cpu count, min 2). Each module only writes its
OWN exports — no shared files, no races. Results print in completion order;
the summary is the same either way. 188 s serial → ~half on 2 cores.

Exit code = number of failing modules.
"""
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

MODULES = [
    "part_coxa", "part_femur", "part_tibia", "part_hand", "part_deck",
    "part_panel", "part_battery", "part_avionics", "part_bench_jig",
    "part_coupler", "part_footpad", "part_fit_ladder", "part_smallwins",
    "part_shell", "part_busboard", "part_stand", "part_tools",
    "part_clips", "part_dock", "part_servo_blank",
]


def run_one(m):
    t0 = time.time()
    try:
        r = subprocess.run([sys.executable, f"{m}.py"],
                           capture_output=True, text=True, timeout=900)
        ok = r.returncode == 0
        tail = (r.stdout + r.stderr).strip().split("\n")[-1][:90]
    except subprocess.TimeoutExpired:
        ok, tail = False, "TIMEOUT (420 s)"
    except FileNotFoundError:
        ok, tail = False, "missing file"
    return (m, ok, time.time() - t0, tail)


POST = ["check_printability"]     # runs AFTER every module has exported (D038)


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
    bad = [m for m, ok, *_ in results if not ok]
    print(f"\n{len(results) - len(bad)}/{len(results)} modules pass "
          f"({time.time()-t00:.0f} s total)"
          + (f" — FAILING: {', '.join(bad)}" if bad else " — TREE CLEAN"))
    sys.exit(len(bad))


if __name__ == "__main__":
    main()
