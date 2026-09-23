#!/usr/bin/env python3
"""ID assignment — burn bus IDs 1–20, ONE SERVO AT A TIME.

Fresh Feetech servos all ship as ID 1, so plugging a bag of them onto one bus
gives you twenty servos answering at once. The drill:

    1. plug exactly ONE servo into the adapter
    2. script finds it (whatever its current id/baud), shows what it will become
    3. you confirm -> EEPROM id written + verified -> label the case NOW
    4. plug the next one

Target order = params.yaml bus map: legs first (leg0 yaw=1, hip=2, knee=3,
leg1 4..6, ... leg4 13..15), then hands 16..20. SCS0009 hands: the script
knows they're protocol 0 and REMINDS YOU they must be on the 5–6 V rail
(D016) — on the bench, power them from the BEC/5 V supply, never 12 V.

    python3 assign_ids.py --port /dev/ttyACM0
    python3 assign_ids.py --mock --yes            # full 20-servo rehearsal
    python3 assign_ids.py --port ... --start-from 16   # resume at the hands

Every assignment is appended to bench/out/id_log.txt.
"""
import os
import sys
from _common import (base_parser, make_bus, confirm, hline, OUT_DIR)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "driver"))
from rocky_driver import Family, load_bus_params                    # noqa: E402
from rocky_driver.mock import MockServo, MockTransport              # noqa: E402

LOG = os.path.join(OUT_DIR, "id_log.txt")


def target_list():
    bp = load_bus_params()
    joints = ("yaw", "hip(femur)", "knee")
    out = []
    for leg, ids in enumerate(bp["leg_ids"]):
        for j, sid in enumerate(ids):
            out.append((sid, Family.STS, f"leg{leg} {joints[j]}  (ST3215, 12 V rail)"))
    for i, sid in enumerate(bp["hand_ids"]):
        out.append((sid, Family.SCS,
                    f"hand{i} claw  (SCS0009 — 5-6 V RAIL ONLY, D016)"))
    return out


def find_single_servo(bus, clock, args):
    """Scan for exactly one servo; complain loudly otherwise."""
    while True:
        found = bus.scan(id_range=range(0, 31),
                         bauds=(1_000_000, 500_000, 115_200))
        if len(found) == 1:
            sid = next(iter(found))
            return sid, found[sid]
        if len(found) == 0:
            print("  no servo found — check the plug/power.", end=" ")
        else:
            print(f"  {len(found)} servos answered ({sorted(found)}) — "
                  f"unplug all but ONE.", end=" ")
        if not confirm("rescan?", args):
            return None, None


def main():
    p = base_parser("Assign Pebble bus IDs one servo at a time")
    p.add_argument("--start-from", type=int, default=1,
                   help="first target id to assign (resume support)")
    args = p.parse_args()

    mock_mt = None
    if args.mock:
        # rehearsal: exactly one factory-fresh servo appears per round
        mock_mt = MockTransport([MockServo(1, Family.STS)])
    bus, clock, mt = make_bus(args, mock_transport=mock_mt)

    targets = [t for t in target_list() if t[0] >= args.start_from]
    os.makedirs(OUT_DIR, exist_ok=True)
    done = []
    for n, (tid, fam, label) in enumerate(targets):
        hline("=")
        print(f"[{n + 1}/{len(targets)}] next target: id {tid} = {label}")
        if fam is Family.SCS:
            print("  *** SCS0009: bench-power this from 5-6 V. 12 V cooks it. ***")
        if not args.yes:
            input("  plug in EXACTLY ONE servo, then press Enter... ")
        elif mt is not None and n > 0:
            # rehearsal: simulate swapping in the next fresh servo
            mt.servos = {1: MockServo(1, fam)}
        cur_id, info = find_single_servo(bus, clock, args)
        if cur_id is None:
            print("aborted."); return 1
        bus.families[cur_id] = fam            # treat it as its real family
        print(f"  found servo: current id {cur_id} @ {info['baud']} baud "
              f"(model LE {info['model_le']} / BE {info['model_be']})")
        if info["baud"] != 1_000_000:
            if confirm(f"  servo is at {info['baud']} — set to 1 Mbps?", args):
                bus.t.set_baud(info["baud"])
                bus.set_baud_reg(cur_id, 1_000_000)
                bus.t.set_baud(1_000_000)
                if not bus.ping(cur_id):
                    print("  lost it after baud change — rescan"); return 1
        if cur_id == tid:
            print(f"  already id {tid} — nothing to do")
        else:
            if not confirm(f"  write id {cur_id} -> {tid}?", args):
                print("skipped."); continue
            bus.set_id(cur_id, tid)
            print(f"  id {tid} verified (ping + readback OK)")
        with open(LOG, "a") as f:
            f.write(f"{tid}\t{label}\tfrom_id={cur_id}\n")
        done.append(tid)
        print(f"  >>> LABEL THE CASE NOW: '{tid}' <<<")
    hline("=")
    print(f"assigned/verified {len(done)} servos: {done}")
    print(f"log: {LOG}\nnext: calibrate_centers.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
