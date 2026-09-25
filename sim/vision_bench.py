"""Vision bench — what the robot's eye + a vision-language model can actually do.

    python sim/vision_bench.py                        # starts its own cockpit on :8791
    python sim/vision_bench.py --models lfm2.5-vl --no-find
    python sim/vision_bench.py --url http://127.0.0.1:8792   # a cockpit you started (NOT :8765)

Against a running cockpit (default: a fresh one it starts itself on port 8791
with --world "obstacle course", and stops at the end), for each vision model:

  DETECT   the ball / a box is placed at a known bearing and distance (world
           swaps through /api/world; the robot respawns at the origin facing +x,
           which is the aim), then /api/look {find, model, method} asks the
           model. Two methods: `bbox` (the model boxes the object; bearing and
           distance come from projecting the box's bottom-centre onto the floor
           through the known camera pose — what find_object uses) and
           `estimate` (the model types bearing_deg / distance_m itself).
           Scored: seen? (and false sightings when the object is absent or
           outside the ~86 deg view), bearing error (deg, to the object's
           centre), distance error (m, to the object's NEAR EDGE from the
           camera — what 'how far until I touch it' means; the bbox bottom
           approximates it), latency (the model call, s).
  FLOOR    two floor-safety questions, yes/no: the cliff preset facing the edge
           ("does the floor end ahead?" -> yes), the flat floor (-> no), the flat
           floor ("is the floor ahead clear?" -> yes) and a box straight ahead
           (-> no).
  FIND     (unless --no-find) find_object end to end: the ball ahead-left, the
           box ahead-right, the ball behind the robot (needs the scan turns).
           Success = found AND the true near-edge distance from the camera at
           the end is <= 0.35 m.

Prints a table and writes sim/out/vision_bench.json. Models: default
lfm2.5-vl, then gemma-4-26b-a4b if llama-swap lists it; quarantined models
(gpt-oss-20b, glm-flash-reap) are never used. A model that does not answer
its first call within --load-timeout is skipped and says so; an answer from a
DIFFERENT model (the cockpit's vision fallback chain) is never scored as the
asked model's. A model the bench loaded is unloaded again at the end (unless
--keep-loaded) so the GPU is left as it was found.

Honesty: a MuJoCo render (flat shading, a checker floor, one orange ball and
grey boxes) is far easier than a real camera, so these numbers are an upper
bound for the real eye; the ground truth is the world spec plus the robot's
reported pose; the physics speed (--speed, default 2x) changes nothing but
wall time.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import subprocess
import sys
import tempfile
import time

import httpx

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (HERE, ROOT, os.path.join(ROOT, "gait")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from world_builder import footprint_dist, PRESETS                     # noqa: E402
from cockpit_brains import (QUARANTINED, EYE_FWD_M, EYE_HFOV_DEG,      # noqa: E402
                            local_ai_key, DEFAULT_BASE)

OWNER_PORT = 8765
DEFAULT_PORT = 8791
DEFAULT_MODELS = ("lfm2.5-vl", "gemma-4-26b-a4b")
OUT = os.path.join(HERE, "out", "vision_bench.json")
BALL_R = 0.05
BOX = [0.15, 0.15, 0.06]
FIND_OK_M = 0.35
YES_RE = re.compile(r"\b(yes|no)\b", re.I)

FLOOR_END_Q = ("Look at the floor directly ahead of the robot. Does the floor end or drop away "
               "(an edge, a step down, a cliff) ahead? Answer yes or no first, then one short sentence.")
FLOOR_CLEAR_Q = ("Look at the floor directly ahead of the robot. Is the floor ahead clear of objects "
                 "and obstacles? Answer yes or no first, then one short sentence.")


# ------------------------------------------------------------------ cases
def _at(bearing_img_deg, centre_m):
    """Object centre `centre_m` from the camera at an image bearing (negative =
    LEFT), with the robot at the origin facing +x (camera at (EYE_FWD_M, 0))."""
    a = math.radians(-bearing_img_deg)
    return [round(EYE_FWD_M + centre_m * math.cos(a), 3), round(centre_m * math.sin(a), 3)]


def ball(bearing, dist):
    return {"kind": "ball", "pos": _at(bearing, dist), "radius_m": BALL_R}


def box(bearing, dist):
    return {"kind": "box", "pos": _at(bearing, dist), "size": list(BOX)}


def detect_cases():
    """(label, world spec, query, index of the target object or None = absent)."""
    flat = lambda *objs: {"base": "flat", "objects": list(objs)}         # noqa: E731
    cases = []
    for b, d in ((0, 0.5), (-25, 0.5), (25, 0.5), (0, 0.9), (-30, 0.9), (30, 0.9), (0, 1.4), (-15, 0.3)):
        cases.append((f"ball {b:+d} deg {d} m", flat(ball(b, d)), "ball", 0))
    for b, d in ((0, 0.6), (20, 0.9), (-30, 0.7)):
        cases.append((f"box {b:+d} deg {d} m", flat(box(b, d)), "box", 0))
    oc = json.loads(json.dumps(PRESETS["obstacle course"]))
    cases.append(("obstacle course: box", oc, "box", 0))
    cases.append(("obstacle course: ball out of view (51 deg left)", oc, "ball", 3))
    cases.append(("absent: ball (only a box)", flat(box(0, 0.6)), "ball", None))
    cases.append(("absent: box (empty floor)", flat(), "box", None))
    cases.append(("absent: ball (empty floor)", flat(), "ball", None))
    return cases


def floor_cases():
    return [("cliff ahead: floor ends?", {"base": "cliff"}, FLOOR_END_Q, "yes"),
            ("flat: floor ends?", {"base": "flat"}, FLOOR_END_Q, "no"),
            ("flat: floor clear?", {"base": "flat"}, FLOOR_CLEAR_Q, "yes"),
            ("box ahead: floor clear?", {"base": "flat", "objects": [box(0, 0.45)]}, FLOOR_CLEAR_Q, "no")]


def find_cases():
    return [("ball ahead-left 0.9 m", {"base": "flat", "objects": [ball(-25, 0.9)]}, "ball", 0),
            ("box ahead-right 0.8 m", {"base": "flat", "objects": [box(20, 0.8)]}, "box", 0),
            ("ball behind-left (scan)", {"base": "flat", "objects": [ball(-150, 0.7)]}, "ball", 0)]


# ------------------------------------------------------------------ truth + parsing
def truth(pose, obj):
    """Image bearing (deg, negative = LEFT) of the object's centre and the plan
    distance from the camera to its near edge, from the robot's pose."""
    yaw = math.radians(pose["yaw_deg"])
    ex, ey = pose["x"] + EYE_FWD_M * math.cos(yaw), pose["y"] + EYE_FWD_M * math.sin(yaw)
    ox, oy = obj["pos"]
    dx, dy = ox - ex, oy - ey
    bx, by = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
    return {"bearing_deg": round(-math.degrees(math.atan2(by, bx)), 1),
            "near_m": round(float(footprint_dist(obj, (ex, ey))), 3),
            "centre_m": round(math.hypot(dx, dy), 3)}


def in_view(t):
    return abs(t["bearing_deg"]) <= EYE_HFOV_DEG / 2 - 2.0


def yes_no(text):
    m = YES_RE.search(str(text or ""))
    return m.group(1).lower() if m else None


def _median(xs):
    xs = [x for x in xs if x is not None]
    return round(statistics.median(xs), 3) if xs else None


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return round(sum(xs) / len(xs), 3) if xs else None


# ------------------------------------------------------------------ the cockpit
class Cockpit:
    def __init__(self, url, timeout=180.0):
        self.url = url.rstrip("/")
        self.c = httpx.Client(timeout=timeout)

    def post(self, path, body=None, timeout=None):
        kw = {"timeout": timeout} if timeout else {}
        r = self.c.post(self.url + path, json=body or {}, **kw)
        return r.json()

    def get(self, path):
        return self.c.get(self.url + path).json()

    def alive(self):
        try:
            return self.c.get(self.url + "/api/ping", timeout=2.0).status_code == 200
        except Exception:
            return False

    def stage(self, name, spec, settle_s=1.5):
        """Swap the world (a new name -> the robot respawns at the origin facing +x),
        reset to be sure, wait for it to settle and the eye to render it."""
        self.post("/api/world", {"name": name, "spec": spec})
        self.post("/api/reset")
        time.sleep(settle_s)
        return self.post("/api/tool/status")["pose"]


def start_cockpit(port, world="obstacle course"):
    env = dict(os.environ, MUJOCO_GL=os.environ.get("MUJOCO_GL", "egl"))
    tmp = tempfile.gettempdir()
    env.setdefault("ROCKY_COCKPIT_CONF", os.path.join(tmp, "vision_bench_cockpit.json"))   # never ~/.config
    log = open(os.path.join(tmp, f"vision_bench_cockpit_{port}.log"), "w")
    proc = subprocess.Popen([sys.executable, os.path.join("sim", "cockpit.py"), "--port", str(port),
                             "--world", world], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    ck = Cockpit(f"http://127.0.0.1:{port}")
    t_end = time.monotonic() + 90
    while time.monotonic() < t_end:
        if proc.poll() is not None:
            raise SystemExit(f"the bench cockpit exited ({proc.returncode}); see {log.name}")
        if ck.alive():
            return proc, ck
        time.sleep(0.5)
    proc.terminate()
    raise SystemExit("the bench cockpit did not come up in 90 s")


# ------------------------------------------------------------------ llama-swap
def llama_swap(path, method="GET", timeout=10.0):
    root = DEFAULT_BASE[:-3]
    h = {"Authorization": f"Bearer {local_ai_key()}"}
    return httpx.request(method, root + path, headers=h, timeout=timeout)


def listed_models():
    try:
        return {m["id"] for m in llama_swap("/v1/models").json().get("data", [])}
    except Exception:
        return set()


def running_models():
    try:
        return {m.get("model") for m in llama_swap("/running").json().get("running", [])}
    except Exception:
        return set()


def unload(model):
    for method in ("POST", "GET"):
        try:
            r = llama_swap(f"/api/models/unload/{model}", method=method, timeout=30.0)
            if r.status_code < 400:
                return True
        except Exception:
            pass
    return False


# ------------------------------------------------------------------ the runs
def run_detect(ck, model, method, log):
    rows = []
    for label, spec, query, idx in detect_cases():
        pose = ck.stage(f"vb-{label}", spec)
        target = spec["objects"][idx] if idx is not None else None
        t = truth(pose, target) if target is not None else None
        present = t is not None and in_view(t)       # in the world but outside the view = absent
        t0 = time.monotonic()
        r = ck.post("/api/look", {"find": query, "model": model, "method": method})
        wall = round(time.monotonic() - t0, 2)
        row = {"case": label, "query": query, "present": present, "truth": t, "wall_s": wall}
        if not r.get("ok"):
            row.update(error=r.get("error"), answered_by=None)
        elif r.get("model") != model:
            row.update(error=f"answered by {r.get('model')} (fallback), not {model}", answered_by=r.get("model"))
        else:
            d = r["detection"]
            row.update(answered_by=model, latency_s=r.get("latency_s"), seen=d["seen"],
                       bearing_deg=d["bearing_deg"], distance_m=d["distance_m"], confidence=d["confidence"],
                       parsed=d["parsed"], method_used=d["method"], raw=r.get("raw", "")[:200])
            if present and d["seen"]:
                if d["bearing_deg"] is not None:
                    row["bearing_err"] = round(d["bearing_deg"] - t["bearing_deg"], 1)
                if d["distance_m"] is not None:
                    row["dist_err"] = round(d["distance_m"] - t["near_m"], 3)
        rows.append(row)
        log(f"  {model} {method:8s} {label:48s} "
            + (f"ERR {row['error'][:60]}" if row.get("error") else
               f"seen={row['seen']!s:5} b={row['bearing_deg']} d={row['distance_m']} "
               f"(truth b={t['bearing_deg'] if t else '-'} near={t['near_m'] if t else '-'}) "
               f"{row['latency_s']} s"))
    return rows


def run_floor(ck, model, log):
    rows = []
    for label, spec, q, expect in floor_cases():
        ck.stage(f"vb-{label}", spec)
        r = ck.post("/api/look", {"question": q, "model": model})
        row = {"case": label, "expect": expect}
        if not r.get("ok") or r.get("model") != model:
            row["error"] = r.get("error") or f"answered by {r.get('model')} (fallback)"
        else:
            yn = yes_no(r["description"])
            row.update(answer=yn, correct=yn == expect, text=r["description"][:200], latency_s=r.get("latency_s"))
        rows.append(row)
        log(f"  {model} floor    {label:48s} "
            + (f"ERR {row['error'][:60]}" if row.get("error") else
               f"{row['answer']} (want {expect}) {'OK' if row['correct'] else 'WRONG'}: {row['text'][:70]!r}"))
    return rows


def run_find(ck, model, log, max_steps=10):
    rows = []
    for label, spec, query, idx in find_cases():
        ck.stage(f"vb-find-{label}", spec)
        t0 = time.monotonic()
        r = ck.post("/api/tool/find_object", {"name": query, "max_steps": max_steps, "model": model},
                    timeout=600.0)
        wall = round(time.monotonic() - t0, 1)
        pose = ck.post("/api/tool/status")["pose"]
        t = truth(pose, spec["objects"][idx])
        ok = bool(r.get("found")) and t["near_m"] <= FIND_OK_M
        rows.append({"case": label, "found": r.get("found"), "success": ok, "detail": r.get("detail") or r.get("error"),
                     "stopped": r.get("stopped"), "looks": r.get("looks"), "turned_deg": r.get("turned_deg"),
                     "end_truth": t, "wall_s": wall, "models": sorted({r.get("model") or "?"})})
        log(f"  {model} find     {label:48s} {'OK ' if ok else 'NO '} {r.get('detail') or r.get('error')} "
            f"(true near edge {t['near_m']} m, {wall} s)")
    return rows


def summarize(rows):
    ans = [r for r in rows if not r.get("error")]
    pres = [r for r in ans if r["present"]]
    absn = [r for r in ans if not r["present"]]
    hits = [r for r in pres if r["seen"]]
    return {"n": len(rows), "answered": len(ans),
            "detected": f"{len(hits)}/{len(pres)}",
            "false_sightings": f"{sum(1 for r in absn if r['seen'])}/{len(absn)}",
            "bearing_err_med_deg": _median([abs(r["bearing_err"]) for r in hits if "bearing_err" in r]),
            "bearing_err_max_deg": max([abs(r["bearing_err"]) for r in hits if "bearing_err" in r], default=None),
            "dist_err_med_m": _median([abs(r["dist_err"]) for r in hits if "dist_err" in r]),
            "dist_err_mean_signed_m": _mean([r["dist_err"] for r in hits if "dist_err" in r]),
            "latency_med_s": _median([r.get("latency_s") for r in ans])}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--url", help="a running cockpit (default: start one on --port)")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--models", default=",".join(DEFAULT_MODELS))
    ap.add_argument("--methods", default="bbox,estimate")
    ap.add_argument("--load-timeout", type=float, default=120.0)
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--no-find", action="store_true")
    ap.add_argument("--no-floor", action="store_true")
    ap.add_argument("--keep-loaded", action="store_true")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args(argv)

    if args.url and httpx.URL(args.url).port == OWNER_PORT:
        raise SystemExit(f"refusing {args.url}: :{OWNER_PORT} is the owner's live cockpit and the bench "
                         "swaps its world — start one with --port instead")
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    listed = listed_models()
    before = running_models()
    models, skipped = [], {}
    for m in [x.strip() for x in args.models.split(",") if x.strip()]:
        if m in QUARANTINED:
            skipped[m] = "quarantined (GPU faults) — never used"
        elif listed and m not in listed:
            skipped[m] = "not listed by llama-swap"
        else:
            models.append(m)
    methods = [x.strip() for x in args.methods.split(",") if x.strip()]

    proc = None
    if args.url:
        ck = Cockpit(args.url)
        if not ck.alive():
            raise SystemExit(f"no cockpit answers at {args.url}")
    else:
        proc, ck = start_cockpit(args.port)
    lines = []

    def log(s):
        print(s, flush=True)
        lines.append(s)

    result = {"date": time.strftime("%Y-%m-%d %H:%M"), "cockpit": ck.url, "speed": args.speed,
              "eye": {"w": 320, "h": 240, "hfov_deg": EYE_HFOV_DEG}, "skipped": skipped, "models": {}}
    loaded_by_us = []
    try:
        ck.post("/api/speed", {"speed": args.speed})
        for m in models:
            ck.stage("vb-warmup", {"base": "flat", "objects": [ball(0, 0.6)]})
            t0 = time.monotonic()
            try:
                r = ck.post("/api/look", {"find": "ball", "model": m}, timeout=args.load_timeout + 10)
            except httpx.TimeoutException:
                r = {"ok": False, "error": f"no answer within {args.load_timeout:.0f} s"}
            first = round(time.monotonic() - t0, 1)
            if not r.get("ok") or r.get("model") != m or first > args.load_timeout:
                why = r.get("error") or (f"answered by {r.get('model')} (fallback)" if r.get("model") != m
                                         else f"first answer took {first} s (> {args.load_timeout:.0f} s)")
                log(f"{m}: SKIPPED — {why}")
                result["skipped"][m] = why
                continue
            if m not in before:
                loaded_by_us.append(m)
            log(f"{m}: first call {first} s (includes a cold load: {m not in before})")
            res = {"first_call_s": first, "detect": {}, "summary": {}}
            for meth in methods:
                rows = run_detect(ck, m, meth, log)
                res["detect"][meth] = rows
                res["summary"][meth] = summarize(rows)
            if not args.no_floor:
                fr = run_floor(ck, m, log)
                res["floor"] = fr
                res["floor_correct"] = f"{sum(1 for r in fr if r.get('correct'))}/{len(fr)}"
            if not args.no_find:
                fd = run_find(ck, m, log)
                res["find"] = fd
                res["find_success"] = f"{sum(1 for r in fd if r['success'])}/{len(fd)}"
            result["models"][m] = res
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(10)
            except subprocess.TimeoutExpired:
                proc.kill()
        if not args.keep_loaded:
            for m in loaded_by_us:
                if m != "lfm2.5-vl":                 # the resident vision model stays warm
                    result.setdefault("unloaded", {})[m] = unload(m)

    # ------------------------------------------------------------ the table
    hdr = (f"{'model':18s} {'method':8s} {'detected':>8s} {'false+':>7s} {'|bearing err|':>14s} "
           f"{'|dist err|':>11s} {'dist bias':>9s} {'latency':>8s}")
    log("")
    log(hdr)
    log("-" * len(hdr))
    for m, res in result["models"].items():
        for meth, s in res["summary"].items():
            log(f"{m:18s} {meth:8s} {s['detected']:>8s} {s['false_sightings']:>7s} "
                f"{str(s['bearing_err_med_deg']) + ' deg (max ' + str(s['bearing_err_max_deg']) + ')':>14s} "
                f"{str(s['dist_err_med_m']) + ' m':>11s} {str(s['dist_err_mean_signed_m']):>9s} "
                f"{str(s['latency_med_s']) + ' s':>8s}")
        extra = []
        if "floor_correct" in res:
            extra.append(f"floor safety {res['floor_correct']}")
        if "find_success" in res:
            extra.append(f"find_object {res['find_success']}")
        extra.append(f"first call {res['first_call_s']} s")
        log(f"{'':18s} {' · '.join(extra)}")
    for m, why in result["skipped"].items():
        log(f"{m:18s} skipped: {why}")
    log("(errors are medians over detected cases; distance = camera to the object's near edge; "
        "bias = mean signed error, + = too far)")
    result["log"] = lines
    with open(args.out, "w") as f:
        json.dump(result, f, indent=1)
    print(f"wrote {os.path.relpath(args.out, ROOT)}")
    return result


if __name__ == "__main__":
    main()
