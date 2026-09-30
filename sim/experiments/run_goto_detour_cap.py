"""B116: how long does a plain goto need once it has entered a detour?

After D063 a goto walks at the 34.2 mm/s envelope and times out at `cockpit.GOTO_CAP_S`
(55 s: `harness.capabilities.goto_cap_s`, the 1.5 m reach x 1.2 at the envelope + the 2.36 s
ease-in). A goto whose straight line runs into an obstacle the lidar sees detours around it
(`cockpit.GOTO_DETOUR_S`, 15.5 s at most per detour), and at 1.2-1.5 m it then runs out of time on
the way. This harness measures what a detour costs and what cap would let those gotos arrive.

Each goto runs in a fresh headless CockpitSim (the cockpit's own goto controller, righter off,
awareness off, no GL, no HTTP), stepped in-process from standing, in its own process. Nothing
here touches a running cockpit.

  detour    gotos of 1.0 / 1.2 / 1.5 / 2.0 m at bearings 0 and 36 deg, straight into a 0.25 m wall
            or a 25 cm box (both taller than the lidar plane), 0.45 m out ("near") or 0.6 m before
            the target ("late"), seeds 0-1: +-3 cm along, +-8 cm across, +-15 deg of yaw
  tail      1.2 and 1.5 m again with a "mid" placement (half way), seeds 2-4
  close     1.2 and 1.5 m with the target 0.4 / 0.45 / 0.5 / 0.55 m behind the obstacle, seeds 0-1:
            the closer the target sits behind it, the dearer the detour (the 1.0 m "near" goto,
            0.55 m behind, was the dearest of the first two batches), until it ends 'blocked'
  plain     the same distances with nothing in the way (they never enter a detour)
  blocked   gotos that cannot arrive: the stuck / blocked rules must end them, not the cap

Every run is UNCAPPED; the cap only ever ends a goto (`_goto_pre`: `tw > GOTO_CAP_S and det is
None`), it never steers, so one uncapped run gives the outcome under any rule. The rule
measured: a plain cap P until the goto has entered a detour, then a detour cap D (a detour
running at the cap still runs out its own limit first, as today). `--verify P D` re-runs the
detour and close batches with that rule applied for real and checks them against the derivation.

Usage (from the repo root; 214 runs, + 128 with --verify; see the record's wall_s for the time):
    .venv/bin/python sim/experiments/run_goto_detour_cap.py [--jobs 6] [--quick] [--verify 55 71]
Writes results/goto_detour_cap_results.json (the 2026-09-30 record: --verify 55 71).
"""
import argparse
import json
import math
import os
import random
import subprocess
import sys
import time
import warnings
from concurrent.futures import ThreadPoolExecutor

os.environ.setdefault("ROCKY_ENV_FILE", os.devnull)      # the reference setup, as the tests run
import exp_paths as X                                     # noqa: E402  (sys.path: sim/, gait/, root)

REACHES = (1.0, 1.2, 1.5, 2.0)
BEARINGS = (0.0, 36.0)
KINDS = ("wall", "box")
LIMIT_S = 150.0          # a harness limit: an uncapped goto that has not ended by then is reported


# ------------------------------------------------------------------ one goto (a child process)
def run_one(sc):
    import numpy as np
    import cockpit
    cap_plain = float(sc.get("cap", 1e9))
    cap_detour = float(sc.get("cap_detour", cap_plain))
    cockpit.GOTO_CAP_S = cap_plain
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        sim = cockpit.CockpitSim("flat")
    sim.do("righter off")
    sim.render_every = 10 ** 9
    sim.brains.conf_path = None
    sim.awareness = dict(sim.awareness, interval_s=0.0, reactions=False, curious=False, recognize=False)
    if sc.get("objects"):
        sim.set_world({"base": "flat", "objects": sc["objects"]}, "b116", keep_pose=False)
        sim.do("righter off")
    logs = []
    _log = sim.log
    sim.log = lambda m, *a, **k: (logs.append((round(sim.t, 3), m)), _log(m, *a, **k))
    for _ in range(int(round(1.0 / sim.DT))):             # stand 1 s: a goto starts from standing
        sim.step()
    ticks, entered = [], [False]
    _pre = sim._goto_pre

    def pre(gs):                                          # at the cap check: is a detour running?
        tw = sim.t - gs["t0"]
        if gs.get("detour") is not None:
            entered[0] = True
        cockpit.GOTO_CAP_S = cap_detour if entered[0] else cap_plain
        p = sim.data.xpos[sim.torso]
        ticks.append((tw, gs.get("detour") is not None, float(np.hypot(gs["tx"] - p[0], gs["ty"] - p[1]))))
        return _pre(gs)
    sim._goto_pre = pre

    class Fut:
        res = None

        def done(self):
            return self.res is not None

        def set_result(self, r):
            self.res = r

    class Loop:
        @staticmethod
        def call_soon_threadsafe(fn, *a):
            fn(*a)

    fut = Fut()
    p0 = sim.data.xpos[sim.torso].copy()
    sim._start_goto(sc["tx"], sc["ty"], fut, Loop())
    gs, t0 = sim.goto_state, sim.t
    while not fut.done() and sim.t - t0 < LIMIT_S:
        sim.step()
    res = fut.res or {"stopped": "harness_limit"}
    detours, on = [], None                                # [start, end) of each run of detour ticks
    for tw, d, _ in ticks:
        if d and on is None:
            on = tw
        elif not d and on is not None:
            detours.append((round(on, 3), round(tw, 3)))
            on = None
    if on is not None:
        detours.append((round(on, 3), round(ticks[-1][0], 3)))
    track, last = [], -1.0                                # distance to go, ~10 Hz (<= 3.4 mm stale at 34.2 mm/s)
    for tw, d, dist in ticks:
        if tw - last >= 0.1 - 1e-9:
            track.append((round(tw, 3), round(dist, 4)))
            last = tw
    p1 = sim.data.xpos[sim.torso]
    return dict(name=sc["name"], stopped=res.get("stopped"),
                outcome_t=None if gs["outcome"] is None else round(gs["outcome"][1], 3),
                end_dist_m=round(float(np.hypot(sc["tx"] - p1[0], sc["ty"] - p1[1])), 4),
                walked_m=round(float(np.hypot(p1[0] - p0[0], p1[1] - p0[1])), 4),
                detours=detours, n_detour_logs=sum(1 for _, m in logs if "— detour " in m),
                falls=int(sim.sup.fall_count), cap=cap_plain, cap_detour=cap_detour,
                logs=[m for m in logs if "goto" in m[1]], track=track)


# ------------------------------------------------------------------ scenarios
def _rot(x, y, th):
    return math.cos(th) * x - math.sin(th) * y, math.sin(th) * x + math.cos(th) * y


def _obstacle(kind, xo, yo, yaw, th):
    ox, oy = _rot(xo, yo, th)
    if kind == "wall":
        return {"kind": "wall", "pos": [round(ox, 4), round(oy, 4)], "len_m": 0.25,
                "yaw_deg": round(90 + yaw + math.degrees(th), 2)}
    return {"kind": "box", "pos": [round(ox, 4), round(oy, 4)], "size": [0.25, 0.25, 0.25],
            "yaw_deg": round(yaw + math.degrees(th), 2)}


def _detour(L, bearing, kind, where, xo, seed):
    rnd = random.Random(f"{L}-{bearing}-{kind}-{where}-{seed}")
    jx, jy, jyaw = rnd.uniform(-0.03, 0.03), rnd.uniform(-0.08, 0.08), rnd.uniform(-15.0, 15.0)
    th = math.radians(bearing)
    tx, ty = _rot(L, 0.0, th)
    return dict(name=f"L{L}_b{bearing:g}_{kind}_{where}_s{seed}", batch="detour", L=L, bearing=bearing,
                kind=kind, where=where, seed=seed, objects=[_obstacle(kind, xo + jx, jy, jyaw, th)],
                tx=round(tx, 4), ty=round(ty, 4))


def scenarios(quick=False):
    out = []
    for L in REACHES:
        for b in BEARINGS:
            for k in KINDS:
                for where, xo in (("near", 0.45), ("late", L - 0.6)):
                    for seed in ((0,) if quick else (0, 1)):
                        out.append(_detour(L, b, k, where, xo, seed))
    if not quick:
        for L in (1.2, 1.5):
            for b in BEARINGS:
                for k in KINDS:
                    for where, xo in (("near", 0.45), ("mid", L / 2), ("late", L - 0.6)):
                        for seed in (2, 3, 4):
                            out.append(dict(_detour(L, b, k, where, xo, seed), batch="tail"))
        for L in (1.2, 1.5):
            for b in BEARINGS:
                for k in KINDS:
                    for behind in (0.4, 0.45, 0.5, 0.55):
                        for seed in (0, 1):
                            out.append(dict(_detour(L, b, k, f"behind{behind:g}", L - behind, seed),
                                            batch="close", behind=behind))
    for L in REACHES:
        for b in BEARINGS:
            tx, ty = _rot(L, 0.0, math.radians(b))
            out.append(dict(name=f"L{L}_b{b:g}_clear", batch="plain", L=L, bearing=b, objects=[],
                            tx=round(tx, 4), ty=round(ty, 4)))
    low = lambda x, y, sx=0.4, sy=0.25: {"kind": "box", "pos": [x, y], "size": [sx, sy, 0.14]}   # below the puck
    wall = lambda x, y, L, yaw=90: {"kind": "wall", "pos": [x, y], "len_m": L, "yaw_deg": yaw}
    for name, objs in (
            ("lowbox_beside_wall", [wall(0.6, 0.0, 0.25)] + [low(0.45, s * 0.36) for s in (1, -1)]),
            ("lowbox_ahead", [low(0.5, 0.0, 0.2, 0.6)]),
            ("long_wall", [wall(0.6, 0.0, 0.8)]),
            ("cul_de_sac", [wall(0.8, 0.0, 0.9), wall(0.4, 0.43, 0.8, 0), wall(0.4, -0.43, 0.8, 0)]),
            ("wall_then_lowbox", [wall(0.5, 0.0, 0.25), low(1.0, 0.0, 0.25, 0.9)]),
            ("closed_pen", [wall(0.45, 0.0, 0.9), wall(-0.45, 0.0, 0.9), wall(0.0, 0.45, 0.9, 0),
                            wall(0.0, -0.45, 0.9, 0)])):
        out.append(dict(name=name, batch="blocked", L=1.5, bearing=0.0, objects=objs, tx=1.5, ty=0.0))
    return out


# ------------------------------------------------------------------ the rule, from an uncapped run
def ends_at(r, plain, detour):
    """(stopped, t, m to go) under the rule: cap `plain`, `detour` once a detour began before the
    plain cap; the first cap check past the cap with no detour running ends it as a timeout."""
    first = r["detours"][0][0] if r["detours"] else None
    t = detour if (first is not None and first <= plain) else plain
    for a, b in r["detours"]:
        if a <= t < b:
            t = b
    t_end = r["outcome_t"] if r["outcome_t"] is not None else math.inf
    if t >= t_end:
        return r["stopped"], r["outcome_t"], None
    togo = [d for tw, d in r["track"] if tw <= t + 1e-9]
    return "timeout", round(t, 2), togo[-1] if togo else None


def min_detour_cap(r, plain):
    """The smallest detour cap (0.1 s grid) under which this run ends as it does uncapped."""
    c = plain
    while c < LIMIT_S and ends_at(r, plain, c)[0] == "timeout":
        c = round(c + 0.1, 1)
    return c


DETOUR_BATCHES = ("detour", "tail", "close")


def table(runs, caps, plain_cap, clear, key=lambda r: r["L"], keys=REACHES):
    """One row per key (default: per distance) over every goto that met an obstacle."""
    rows = []
    for L in keys:
        rs = [r for r in runs if r["batch"] in DETOUR_BATCHES and key(r) == L]
        if not rs:
            continue
        Ls = {r["L"] for r in rs}
        arr = sorted(r["outcome_t"] for r in rs if r["stopped"] == "arrived")
        costs = sorted(r["outcome_t"] - clear[r["L"]] for r in rs if r["stopped"] == "arrived" and r["L"] in clear)
        row = dict(L_m=L, n=len(rs), entered_detour=sum(1 for r in rs if r["detours"]),
                   uncapped={s: sum(1 for r in rs if r["stopped"] == s) for s in sorted({r["stopped"] for r in rs})},
                   t_arrive_s=dict(min=arr[0], median=arr[len(arr) // 2], max=arr[-1]) if arr else None,
                   clear_t_arrive_s=clear.get(next(iter(Ls))) if len(Ls) == 1 else None,
                   detour_cost_s=dict(min=round(costs[0], 1), median=round(costs[len(costs) // 2], 1),
                                      max=round(costs[-1], 1)) if costs else None,
                   min_detour_cap_s=max((min_detour_cap(r, plain_cap) for r in rs if r["stopped"] == "arrived"),
                                        default=None),
                   arrived={}, timeout_to_go_cm={})
        for c in caps:
            o = [ends_at(r, plain_cap, c) for r in rs]
            row["arrived"][str(c)] = sum(1 for s, _, _ in o if s == "arrived")
            togo = [d for s, _, d in o if s == "timeout" and d is not None]
            if togo:
                row["timeout_to_go_cm"][str(c)] = [round(min(togo) * 100), round(max(togo) * 100)]
        rows.append(row)
    return rows


# ------------------------------------------------------------------ driver
def run_batch(scs, jobs):
    env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")

    def one(sc):
        p = subprocess.run([sys.executable, os.path.abspath(__file__), "--one", json.dumps(sc)],
                           capture_output=True, text=True, env=env, timeout=1800)
        line = [ln for ln in p.stdout.splitlines() if ln.startswith("{")]
        if not line:
            raise RuntimeError(f"{sc['name']}: {p.stderr[-1500:]}")
        return dict(json.loads(line[-1]), **{k: sc[k] for k in ("batch", "L", "bearing", "kind", "where", "seed",
                                                                 "behind") if k in sc})
    out = []
    with ThreadPoolExecutor(max_workers=jobs) as ex:
        for r in ex.map(one, scs):
            print(f"  {r['name']:<26} {r['stopped']:<8} {r['outcome_t']!s:>7} s  detours {len(r['detours'])}",
                  flush=True)
            out.append(r)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--one", help=argparse.SUPPRESS)
    ap.add_argument("--jobs", type=int, default=6, help="sim processes at a time (default 6)")
    ap.add_argument("--quick", action="store_true", help="seed 0 only, no tail or close batch")
    ap.add_argument("--verify", nargs=2, type=float, metavar=("PLAIN", "DETOUR"),
                    help="also run the detour and close batches with this rule applied and compare")
    a = ap.parse_args()
    if a.one:
        print(json.dumps(run_one(json.loads(a.one))))
        return 0
    import cockpit
    import harness.capabilities as C
    env = C.default_envelope()
    v, e = env["goto_speed_m_s"], C.default_ease_in_s()
    plain_cap = float(cockpit.GOTO_CAP_S)
    one = C.goto_cap_s(v, e, C.GOTO_REACH_M + cockpit.GOTO_DETOUR_M)                         # one detour
    formula = C.goto_cap_s(v, e, C.GOTO_REACH_M + cockpit.GOTO_DETOURS * cockpit.GOTO_DETOUR_M)   # both
    caps = sorted({plain_cap, one, 75.0, formula})
    reach_all = C.GOTO_REACH_M + cockpit.GOTO_DETOURS * cockpit.GOTO_DETOUR_M
    print(f"envelope {v * 1000:.1f} mm/s, ease-in {e:.2f} s: plain cap {plain_cap:g} s,"
          f" detour {cockpit.GOTO_DETOUR_S} s; goto_cap_s(reach + GOTO_DETOUR_M) = {one:g} s,"
          f" goto_cap_s(reach + GOTO_DETOURS x GOTO_DETOUR_M = {reach_all:g} m) = {formula:g} s")
    t0 = time.monotonic()
    scs = scenarios(a.quick)
    runs = run_batch(scs, max(1, a.jobs))
    clear = {}
    for r in runs:
        if r["batch"] == "plain" and r["stopped"] == "arrived":
            clear.setdefault(r["L"], []).append(r["outcome_t"])
    clear = {L: round(sum(t) / len(t), 2) for L, t in clear.items()}
    rows = table(runs, caps, plain_cap, clear)
    behinds = sorted({r["behind"] for r in runs if r["batch"] == "close"})
    close_rows = []                                       # one row per (distance, target behind the obstacle)
    for L in (1.2, 1.5):
        for w in table([r for r in runs if r["batch"] == "close" and r["L"] == L], caps, plain_cap, clear,
                       key=lambda r: r["behind"], keys=behinds):
            close_rows.append(dict(w, behind_m=w["L_m"], L_m=L))
    hdr = (f"{'':>11} {'n':>3} {'uncapped':>28} {'t_arrive min/med/max s':>24} {'cost min/med/max s':>19} "
           + " ".join(f"{'@' + format(c, 'g') + ' s':>8}" for c in caps) + f" {'min cap':>8}")

    def show(w, label):
        t, c = w["t_arrive_s"] or dict(min=math.nan, median=math.nan, max=math.nan), w["detour_cost_s"]
        c = c or dict(min=math.nan, median=math.nan, max=math.nan)
        print(f"{label:>11} {w['n']:>3} {json.dumps(w['uncapped']):>28} "
              f"{t['min']:>8.1f}/{t['median']:>6.1f}/{t['max']:>6.1f} "
              f"{c['min']:>7.1f}/{c['median']:>4.1f}/{c['max']:>5.1f} "
              + " ".join(f"{w['arrived'][str(x)]:>4}/{w['n']:<3}" for x in caps) + f" {w['min_detour_cap_s']!s:>8}")
    print("\nper distance, every goto that met an obstacle\n" + hdr)
    for w in rows:
        show(w, f"{w['L_m']:g} m")
    print("\nclose batch: the target this far behind the obstacle\n" + hdr)
    for w in close_rows:
        show(w, f"{w['L_m']:g} m {w['behind_m']:g}")
    det_runs = [r for r in runs if r["batch"] in DETOUR_BATCHES]
    longest = max((b - a for r in runs for a, b in r["detours"]), default=0.0)
    arrived = [r["outcome_t"] for r in det_runs if r["stopped"] == "arrived" and r["L"] <= C.GOTO_REACH_M]
    costs = sorted(r["outcome_t"] - clear[r["L"]] for r in det_runs if r["stopped"] == "arrived" and r["L"] in clear)
    summary = dict(latest_arrival_to_reach_s=max(arrived), slack_s={str(c): round(c - max(arrived), 2) for c in caps},
                   detour_cost_s=dict(n=len(costs), min=round(costs[0], 1), median=round(costs[len(costs) // 2], 1),
                                      max=round(costs[-1], 1)),
                   longest_detour_interval_s=round(longest, 2),
                   worst_answer_s={str(c): round(c + longest + 1.5, 1) for c in caps})
    print("\nsummary:", summary)
    clear_out = {r["name"]: dict(uncapped=(r["stopped"], r["outcome_t"]), at_plain_cap=ends_at(r, plain_cap, formula))
                 for r in runs if r["batch"] == "plain"}
    blocked_out = {r["name"]: (r["stopped"], r["outcome_t"]) for r in runs if r["batch"] == "blocked"}
    print("clear gotos (no detour, the rule never applies):", clear_out)
    print("gotos that cannot arrive (uncapped: what ends them):", blocked_out)
    verify = None
    if a.verify:
        p, d = a.verify
        det = [dict(s, cap=p, cap_detour=d) for s in scs if s["batch"] in ("detour", "close")]
        got = {r["name"]: r for r in run_batch(det, max(1, a.jobs))}
        uncapped = {r["name"]: r for r in runs}
        bad = []
        for n, r in got.items():
            want = ends_at(uncapped[n], p, d)
            if r["stopped"] != want[0] or abs((r["outcome_t"] or 0) - (want[1] or 0)) > 0.05:
                bad.append((n, r["stopped"], r["outcome_t"], want[:2]))
        verify = dict(plain=p, detour=d, n=len(got), mismatches=bad,
                      timeouts=sum(1 for r in got.values() if r["stopped"] == "timeout"))
        print(f"verify {p:g}/{d:g}: {len(got) - len(bad)}/{len(got)} runs end as derived "
              f"({verify['timeouts']} of them timeouts)", bad or "")
    out = dict(question="B116: the cap for a plain goto that has entered a detour",
               envelope=dict(goto_speed_m_s=v, ease_in_s=e, goto_cap_s=plain_cap, goto_detour_s=cockpit.GOTO_DETOUR_S,
                             goto_detour_m=cockpit.GOTO_DETOUR_M, goto_detours=cockpit.GOTO_DETOURS,
                             reach_m=C.GOTO_REACH_M),
               formula=dict(expr="goto_cap_s(speed, ease_in, GOTO_REACH_M + GOTO_DETOURS * GOTO_DETOUR_M)",
                            value_s=formula,
                            one_detour=dict(expr="goto_cap_s(speed, ease_in, GOTO_REACH_M + GOTO_DETOUR_M)",
                                            value_s=one)),
               caps=caps, clear_t_arrive_s=clear, table=rows, close_table=close_rows, summary=summary,
               clear=clear_out, blocked=blocked_out, verify=verify,
               runs=[{k: v for k, v in r.items() if k not in ("track", "logs")} for r in runs],
               wall_s=round(time.monotonic() - t0))
    path = X.result("goto_detour_cap_results.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    print("->", os.path.relpath(path, X.ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
