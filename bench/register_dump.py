#!/usr/bin/env python3
"""Register dump — archive every byte of every servo (0..73) to CSV.

Two jobs:
  1. Day-one archive: the as-shipped EEPROM of every servo, before we touch
     anything (settles every VERIFY-ON-BENCH flag in registers.py).
  2. Diff tool: --diff compares against a previous dump and prints changes.

    python3 register_dump.py --port /dev/ttyACM0
    python3 register_dump.py --mock
    python3 register_dump.py --mock --diff bench/out/dump_XXXX.csv
"""
import csv
import os
import sys
import time
from _common import base_parser, make_bus, hline, OUT_DIR

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "driver"))
from rocky_driver.registers import MAPS                             # noqa: E402

SPAN = 74


def annotate(family):
    names = {}
    for name, r in MAPS[family].items():
        for k in range(r.nbytes):
            names[r.addr + k] = name + ("" if r.nbytes == 1 else f"[{k}]")
    return names


def main():
    p = base_parser("Dump all servo registers to CSV")
    p.add_argument("--ids", default=None, help="e.g. 1-20 (default: scan)")
    p.add_argument("--diff", default=None, help="previous dump CSV to compare")
    args = p.parse_args()
    bus, clock, _ = make_bus(args)

    if args.ids:
        a, b = args.ids.split("-") if "-" in args.ids else (args.ids, args.ids)
        ids = [i for i in range(int(a), int(b) + 1) if bus.ping(i)]
    else:
        ids = sorted(bus.scan(id_range=range(0, 31), bauds=(1_000_000,)))
    print(f"dumping {len(ids)} servos: {ids}")

    rows = {}
    for sid in ids:
        raw = bus.read_raw(sid, 0, SPAN)
        rows[sid] = list(raw)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = os.path.join(OUT_DIR, f"dump_{stamp}.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["addr", "name_sts", "name_scs"] + [f"id{sid}" for sid in ids])
        n_sts, n_scs = annotate(list(MAPS)[0]), annotate(list(MAPS)[1])
        for a in range(SPAN):
            w.writerow([a, n_sts.get(a, ""), n_scs.get(a, "")]
                       + [rows[sid][a] for sid in ids])
    print(f"wrote {out}")

    if args.diff:
        with open(args.diff) as f:
            r = csv.reader(f)
            head = next(r)
            old_ids = [int(h[2:]) for h in head[3:]]
            old = {a: {sid: int(v) for sid, v in zip(old_ids, line[3:])}
                   for a, *_, line in
                   ((int(line[0]), line) for line in r)}
        hline()
        print(f"diff vs {args.diff}:")
        changes = 0
        for a in range(SPAN):
            for sid in ids:
                if sid in old.get(a, {}) and old[a][sid] != rows[sid][a]:
                    print(f"  id {sid} addr {a:2d}: {old[a][sid]:3d} -> "
                          f"{rows[sid][a]:3d}")
                    changes += 1
        print(f"{changes} changed bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
