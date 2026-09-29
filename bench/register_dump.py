#!/usr/bin/env python3
"""Register dump — archive every byte of every servo to CSV.

Two jobs:
  1. Day-one archive: the as-shipped EEPROM of every servo, before we touch
     anything (settles every VERIFY-ON-BENCH flag in registers.py).
  2. Diff tool: --diff compares against a previous dump and prints changes.

    python3 register_dump.py --port /dev/ttyACM0
    python3 register_dump.py --mock
    python3 register_dump.py --mock --diff bench/out/mock/dump_XXXX.csv
    python3 register_dump.py --mock --out /tmp/dumps      # anywhere but bench/out

Spans (D063, B79): an STS servo is read 0..87 — past the public V3.7 table's
73, into the factory block (80-86) where 85/86 set a hidden acceleration ramp
that acts even at ACC 0 (LeRobot writes 85 = 254, Open Duck Mini 0), and 87
as a guard byte. An SCS0009 is read 0..83, the end of its own factory block
(78-83); its column is blank past that. A servo that refuses the long read
(a firmware whose table ends sooner) falls back to 0..73, blank above, with a
warning. Each servo also gets one summary line: firmware, return delay, the
lock byte and, on STS, ACC and 85/86 — the bytes B79 asks to record.

Dumps go to bench/out/ (bench/out/mock/ for --mock), or --out DIR.
"""
import csv
import os
import sys
import time
from _common import base_parser, make_bus, hline, out_dir

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "driver"))
from rocky_driver.protocol import Family                            # noqa: E402
from rocky_driver.registers import MAPS                             # noqa: E402
from rocky_driver.bus import BusError                               # noqa: E402

SPAN = {Family.STS: 88, Family.SCS: 84}   # bytes read from addr 0 (see above)
LEGACY_SPAN = 74                          # the pre-D063 dump: 0..73, every family
ROWS = max(SPAN.values())


def annotate(family):
    names = {}
    for name, r in MAPS[family].items():
        for k in range(r.nbytes):
            names[r.addr + k] = name + ("" if r.nbytes == 1 else f"[{k}]")
    return names


def read_span(bus, sid):
    """(bytes from addr 0, note): the family's span, or the legacy 0..73 when
    the long read is refused (note says so)."""
    fam = bus.family_of(sid)
    try:
        return list(bus.read_raw(sid, 0, SPAN[fam])), ""
    except BusError as e:                     # NoResponse included
        return list(bus.read_raw(sid, 0, LEGACY_SPAN)), \
            f"id {sid}: 0..{SPAN[fam] - 1} refused ({e}); read 0..{LEGACY_SPAN - 1}"


def summary(sid, fam, raw):
    """One line of the motion-relevant bytes (B79)."""
    R = MAPS[fam]

    def b(name):
        a = R[name].addr
        return raw[a] if a < len(raw) else "-"                   # "-": past a refused span
    parts = [f"id {sid:2d} [{fam.name}] fw {b('FIRMWARE_MAJOR')}.{b('FIRMWARE_MINOR')}",
             f"return delay {2 * b('RETURN_DELAY')} us", f"lock {b('LOCK')}"]
    if fam is Family.STS:
        parts += [f"ACC {b('ACC')}", f"MAX_ACC(85) {b('MAX_ACC')}",
                  f"ACC_MULTIPLIER(86) {b('ACC_MULTIPLIER')}"]
    return "  ".join(parts)


def load_dump(path):
    """{addr: {id: byte}} from a dump CSV; blank cells (not read) are skipped,
    so a pre-D063 0..73 dump diffs against a 0..87 one on the bytes both have."""
    with open(path) as f:
        r = csv.reader(f)
        head = next(r)
        ids = [int(h[2:]) for h in head[3:]]
        return {int(line[0]): {sid: int(v) for sid, v in zip(ids, line[3:]) if v != ""}
                for line in r}


def main():
    p = base_parser("Dump all servo registers to CSV")
    p.add_argument("--ids", default=None, help="e.g. 1-20 (default: scan)")
    p.add_argument("--diff", default=None, help="previous dump CSV to compare")
    p.add_argument("--out", default=None, help="directory for the CSV (default bench/out[/mock])")
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
        rows[sid], note = read_span(bus, sid)
        if note:
            print(f"  WARNING {note}")
        print("  " + summary(sid, bus.family_of(sid), rows[sid]))

    stamp = time.strftime("%Y%m%d_%H%M%S")
    folder = args.out or out_dir(args)
    os.makedirs(folder, exist_ok=True)
    out = os.path.join(folder, f"dump_{stamp}.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["addr", "name_sts", "name_scs"] + [f"id{sid}" for sid in ids])
        n_sts, n_scs = annotate(Family.STS), annotate(Family.SCS)
        for a in range(ROWS):
            w.writerow([a, n_sts.get(a, ""), n_scs.get(a, "")]
                       + [rows[sid][a] if a < len(rows[sid]) else "" for sid in ids])
    print(f"wrote {out}")

    if args.diff:
        old = load_dump(args.diff)
        hline()
        print(f"diff vs {args.diff}:")
        changes = 0
        for a in range(ROWS):
            for sid in ids:
                if sid in old.get(a, {}) and a < len(rows[sid]) and old[a][sid] != rows[sid][a]:
                    print(f"  id {sid} addr {a:2d}: {old[a][sid]:3d} -> "
                          f"{rows[sid][a]:3d}")
                    changes += 1
        print(f"{changes} changed bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
