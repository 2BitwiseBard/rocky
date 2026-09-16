#!/usr/bin/env python3
"""WiFi RSSI fingerprint logger — free coarse localization, zero hardware.

The Pi 5's own radio is a sensor: every AP the house can hear is a beacon
with a position-dependent signal strength. Log (pose, {bssid: rssi}) tuples
while the robot roams; later, k-NN over fingerprints gives a coarse global
prior (typ. 1–3 m indoors) that de-lobotomizes SLAM relocalization after a
carry ("kidnapped robot") and lets chord-speak announce "I think I'm in the
kitchen" before lidar exists.

Modes:
  --scan       run on the Pi later: `iw dev <if> scan` (fallback nmcli),
               pose from /pebble/odom, appends JSONL to rssi_log.jsonl
  --sim        RIGHT NOW: synthesizes a 3-AP apartment (log-distance path
               loss, sigma 3 dB), walks a coverage path, logs fingerprints,
               then k-NN-localizes hold-out samples and reports error.

The record format is the contract; the Pi script and the sim write the same
JSONL. Analysis lives here too (fingerprint_localize) so the pipeline is
end-to-end today.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

import numpy as np

REC_VERSION = 1


# ------------------------------------------------------------------ Pi side
def scan_iw(iface="wlan0"):
    """Parse `iw dev IFACE scan` -> {bssid: rssi_dbm}. Needs sudo or caps."""
    out = subprocess.run(["iw", "dev", iface, "scan"],
                         capture_output=True, text=True, timeout=15).stdout
    res = {}
    bssid = None
    for line in out.splitlines():
        m = re.match(r"^BSS ([0-9a-f:]{17})", line.strip())
        if m:
            bssid = m.group(1)
        m = re.search(r"signal:\s*(-?\d+\.?\d*) dBm", line)
        if m and bssid:
            res[bssid] = float(m.group(1))
    return res


def scan_nmcli():
    out = subprocess.run(["nmcli", "-t", "-f", "BSSID,SIGNAL", "dev", "wifi",
                          "list", "--rescan", "yes"],
                         capture_output=True, text=True, timeout=20).stdout
    res = {}
    for line in out.splitlines():
        parts = line.replace("\\:", "|").rsplit(":", 1)
        if len(parts) == 2 and parts[1].isdigit():
            # nmcli SIGNAL is 0-100; map roughly to dBm
            res[parts[0].replace("|", ":")] = int(parts[1]) / 2 - 100
    return res


def record(pose_xy_yaw, readings, path):
    rec = dict(v=REC_VERSION, t=time.time(),
               pose=[round(float(x), 4) for x in pose_xy_yaw],
               rssi={k: round(float(v), 1) for k, v in readings.items()})
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")
    return rec


# ------------------------------------------------------------------ analysis
def fingerprint_localize(db, query, k=4):
    """db: list of records; query: {bssid: rssi}. Weighted k-NN in RSSI space."""
    def dist(a, b):
        keys = set(a) | set(b)
        MISSING = -95.0
        return math.sqrt(sum((a.get(x, MISSING) - b.get(x, MISSING)) ** 2
                             for x in keys) / len(keys))
    scored = sorted(((dist(r["rssi"], query), r) for r in db),
                    key=lambda p: p[0])[:k]
    w = np.array([1.0 / (d + 1e-3) for d, _ in scored])
    w /= w.sum()
    xy = np.array([r["pose"][:2] for _, r in scored])
    return (w[:, None] * xy).sum(0)


# ------------------------------------------------------------------ sim mode
def run_sim(path):
    rng = np.random.default_rng(11)
    APS = [(-4.0, 2.0, -32), (5.0, 4.0, -35), (1.0, -5.0, -30)]  # x, y, tx dBm

    def rssi_at(x, y):
        out = {}
        for i, (ax, ay, tx) in enumerate(APS):
            d = max(math.hypot(x - ax, y - ay), 0.5)
            r = tx - 10 * 2.4 * math.log10(d) + rng.normal(0, 3.0)
            if r > -92:
                out[f"aa:bb:cc:00:00:{i:02x}"] = r
        return out

    # coverage path: serpentine over an 8x6 m "apartment"
    if os.path.exists(path):
        os.remove(path)
    pts = []
    for row in range(7):
        y = -3 + row
        xs = np.arange(-4, 4.01, 0.5)
        for x in (xs if row % 2 == 0 else xs[::-1]):
            pts.append((x, y))
    for (x, y) in pts:
        record((x, y, 0.0), rssi_at(x, y), path)
    db = [json.loads(line) for line in open(path)]
    print(f"sim: {len(db)} fingerprints over 8x6 m, 3 APs")

    errs = []
    for _ in range(80):
        qx, qy = rng.uniform(-4, 4), rng.uniform(-3, 3)
        est = fingerprint_localize(db, rssi_at(qx, qy))
        errs.append(math.hypot(est[0] - qx, est[1] - qy))
    errs = np.array(errs)
    print(f"k-NN localization on 80 hold-out queries: "
          f"median {np.median(errs):.2f} m, p90 {np.percentile(errs, 90):.2f} m")
    print("(that's the expected indoor-RSSI class of accuracy: a ROOM-level "
          "prior for SLAM relocalization, not a position sensor)")
    return float(np.median(errs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--scan", action="store_true")
    ap.add_argument("--iface", default="wlan0")
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "rssi_log.jsonl"))
    ap.add_argument("--pose", default="0,0,0",
                    help="x,y,yaw (until the odom topic wires in)")
    args = ap.parse_args()
    if args.sim:
        run_sim(args.out)
        return
    if args.scan:
        pose = [float(v) for v in args.pose.split(",")]
        try:
            readings = scan_iw(args.iface)
        except Exception:
            readings = scan_nmcli()
        rec = record(pose, readings, args.out)
        print(f"logged {len(rec['rssi'])} APs -> {args.out}")
        return
    ap.print_help()


if __name__ == "__main__":
    main()
