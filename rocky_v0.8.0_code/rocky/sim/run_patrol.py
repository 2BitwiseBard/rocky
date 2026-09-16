#!/usr/bin/env python3
"""Patrol-and-report demo (session 8d): the harness tools run a MISSION.

The first behavior built entirely on the tool surface — no direct model
access, no cheating past the contract: goto (real physics, guards live),
scan_summary (the sim_lidar ray fan, session-8d binding), say (chord-
speak). A brain — Claude over MCP, the intent REPL, local_brain — could
issue exactly these calls; this script IS the transcript we want them to
produce, pinned as a repeatable artifact.

    ROCKY_WORLD=room MUJOCO_GL=osmesa python3 run_patrol.py
    ... --waypoints 4          # shorter lap
    ... --alert-m 0.45         # paranoia knob

Route: a diamond inside the sim_lidar room's walkable center ring
(obstacles hug the corners by design — see sim_lidar.py's ROOM note).
At each waypoint: stop, scan, chord-speak the finding (found_it when an
obstacle is inside alert range, acknowledge when clear), log everything.
Ends with a patrol report: per-waypoint nearest obstacle + bearing, walls
seen, chords spoken — written to out/patrol_report.json.

Honesty: waypoint arrival is the backend's own 'arrived' outcome (25 mm),
distances are lidar measurements, and a goto that ends 'timeout' or
'FELL' is REPORTED as such, not smoothed over. The 2D scan cannot see
voids (D043) — in the room world there are none; the cliff guard stays
armed regardless.
"""
import argparse
import asyncio
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

# the walkable diamond: comfortably inside walls (±1.6/±1.3) and outside
# the corner obstacles' reach
ROUTE = [(0.55, 0.0), (0.0, 0.55), (-0.55, 0.0), (0.0, -0.55), (0.0, 0.0)]


async def patrol(alert_m=0.45, waypoints=None):
    os.environ.setdefault("ROCKY_WORLD", "room")
    from harness.sim_backend import SimBackend
    b = SimBackend(world="room")
    route = ROUTE[:waypoints] if waypoints else ROUTE
    log = {"world": "room", "alert_m": alert_m, "waypoints": []}
    await b.say("startup")
    print("patrol: startup chord, beginning lap of", len(route), "waypoints")
    for n, (x, y) in enumerate(route):
        leg = {"target": [x, y]}
        r = await b.goto(x, y)
        leg["goto"] = {k: r[k] for k in ("ok", "stopped", "pose") if k in r}
        print(f"  wp{n} ({x:+.2f},{y:+.2f}): goto -> {r.get('stopped')}"
              f" pose={r.get('pose')}")
        if not r.get("ok"):
            await b.say("error")
            leg["chord"] = "error"
            log["waypoints"].append(leg)
            print("    goto failed — aborting lap (reported, not hidden)")
            break
        s = await b.scan_summary()
        near = s.get("nearest_obstacle_m")
        leg["scan"] = {"nearest_m": near,
                       "nearest_bearing_deg": s.get("nearest_obstacle_bearing_deg"),
                       "sectors": s["sectors"]}
        if near is not None and near < alert_m:
            word = "found_it"
        else:
            word = "acknowledge"
        await b.say(word)
        leg["chord"] = word
        print(f"    scan: nearest {near} m @ "
              f"{s.get('nearest_obstacle_bearing_deg')}° -> chord {word}")
        log["waypoints"].append(leg)
    done = [w for w in log["waypoints"] if w.get("goto", {}).get("ok")]
    complete = len(done) == len(route)
    await b.say("discovery" if complete else "confused")
    log["complete"] = complete
    log["chords"] = [e[1] for e in b.events if e[0] == "say"]
    nearest_all = [w["scan"]["nearest_m"] for w in done
                   if w.get("scan", {}).get("nearest_m") is not None]
    log["nearest_obstacle_overall_m"] = min(nearest_all) if nearest_all else None
    out = os.path.join(HERE, "out")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "patrol_report.json")
    with open(path, "w") as fh:
        json.dump(log, fh, indent=1)
    print(f"patrol {'COMPLETE' if complete else 'INCOMPLETE'}: "
          f"{len(done)}/{len(route)} waypoints, nearest obstacle overall "
          f"{log['nearest_obstacle_overall_m']} m, report -> {path}")
    return log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--alert-m", type=float, default=0.45)
    ap.add_argument("--waypoints", type=int, default=None)
    args = ap.parse_args()
    log = asyncio.run(patrol(args.alert_m, args.waypoints))
    sys.exit(0 if log["complete"] else 1)


if __name__ == "__main__":
    main()
