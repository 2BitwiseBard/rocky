#!/usr/bin/env python3
"""Bus scan — find every servo on the wire, whatever ID/baud it woke up with.

    python3 bus_scan.py --port /dev/ttyACM0          # the real thing
    python3 bus_scan.py --mock                       # rehearsal

Prints a table (id, baud, model bytes both-endian, volts, temp, position) and
writes bench/out/last_scan.json (bench/out/mock/ for --mock). Run this FIRST on bench day, before anything
is commanded to move: it also catches the classic factory state (every servo
ID 1 — which is why assignment happens one servo at a time).
"""
import json
import os
import sys
from _common import base_parser, make_bus, hline, out_dir, parse_ids


def main():
    p = base_parser("Scan the Feetech bus across IDs and bauds")
    p.add_argument("--ids", default="0-30", help="id range, e.g. 0-30 or 1,2,7")
    p.add_argument("--bauds", default="1000000,500000,250000,128000,115200")
    args = p.parse_args()

    id_range = parse_ids(args.ids)
    bauds = [int(b) for b in args.bauds.split(",")]

    bus, clock, _ = make_bus(args)
    print(f"scanning ids {min(id_range)}..{max(id_range)} at bauds {bauds} ...")
    found = bus.scan(id_range=id_range, bauds=bauds)

    hline("=")
    print(f"{'ID':>3} {'baud':>8} {'model LE':>9} {'model BE':>9} "
          f"{'family(cfg)':>11} {'V':>5} {'degC':>4} {'pos':>5}")
    hline()
    for sid in sorted(found):
        i = found[sid]
        print(f"{sid:>3} {i['baud']:>8} {i['model_le']:>9} {i['model_be']:>9} "
              f"{i['family_cfg']:>11} {i.get('volt', float('nan')):>5.1f} "
              f"{i.get('temp', '?'):>4} {i.get('pos', '?'):>5}")
    hline("=")
    print(f"{len(found)} servo(s) found")
    expect = 20
    if len(found) == expect:
        print("full Pebble complement present — proceed to calibrate_centers.py")
    elif len(found) == 0:
        print("NOTHING FOUND. Checklist: 12 V rail on? TX/RX not swapped? "
              "servo plugged to the adapter's bus side? try --bauds with more rates")
    if set(found) == {1}:
        print("NOTE: only id 1 answers — if several fresh servos are plugged in they "
              "are ALL id 1; unplug all but one and run assign_ids.py")
    out = os.path.join(out_dir(args), "last_scan.json")
    with open(out, "w") as f:
        json.dump({str(k): v for k, v in found.items()}, f, indent=1)
    print(f"wrote {out}")
    return 0 if found else 1


if __name__ == "__main__":
    sys.exit(main())
