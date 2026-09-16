#!/usr/bin/env python3
"""Center calibration — teach the software where mechanical zero is.

For each joint: torque OFF, you hold the joint at its CAD-zero pose (use the
calibration comb / bench jig faces — see BENCH_RUNBOOK.md §5), the script
reads the position and stores offset = center_counts - present_counts, plus
the direction sign measured by a small nudge test.

Offsets are stored in bench/calibration.yaml (software side) by default —
they ride along in git and survive servo swaps. `--burn` additionally writes
the STS POSITION_OFFSET EEPROM register so even raw tools see centered
counts (SCS0009 has no such register; hands are always software-offset).

CAD-zero poses (frames from pebble_gait.py / the CAD status log):
  yaw   : leg pointing straight out along its station radial (q1 = 0)
  hip   : femur horizontal (q2 = 0)
  knee  : tibia straight in line with femur (q3 = 0)  <- comb holds -90; the
          script accepts a --knee-at -90 flag for the comb pose and shifts it
  claw  : fingers fully CLOSED into the cone (0 deg = walking foot)

    python3 calibrate_centers.py --port /dev/ttyACM0
    python3 calibrate_centers.py --mock --yes
"""
import os
import sys
import yaml
from _common import base_parser, make_bus, confirm, hline, CAL_PATH

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "driver"))
from rocky_driver import Family, load_bus_params                    # noqa: E402
from rocky_driver.registers import CENTER, COUNTS, SWEEP_DEG        # noqa: E402

JOINTS = ("yaw", "hip", "knee")


def main():
    p = base_parser("Center-calibrate every joint (torque-off, hold, read)")
    p.add_argument("--only", default=None,
                   help="subset, e.g. 'leg0' or 'hands' or 'leg2_knee'")
    p.add_argument("--knee-at", type=float, default=-90.0,
                   help="knee angle (deg) the comb/jig holds during calibration")
    p.add_argument("--burn", action="store_true",
                   help="also write STS POSITION_OFFSET EEPROM")
    p.add_argument("--skip-dir-test", action="store_true",
                   help="skip the nudge direction test (keep existing/def +1)")
    args = p.parse_args()

    bus, clock, mt = make_bus(args)
    bp = load_bus_params()
    cal = {"dir": {}, "offset": {}, "meta": {"knee_jig_deg": args.knee_at}}
    if os.path.exists(CAL_PATH):
        with open(CAL_PATH) as f:
            old = yaml.safe_load(f) or {}
        cal["dir"].update(old.get("dir", {}))
        cal["offset"].update(old.get("offset", {}))

    jobs = []
    for leg, ids in enumerate(bp["leg_ids"]):
        for j, sid in enumerate(ids):
            key = f"leg{leg}_{JOINTS[j]}"
            if args.only and args.only not in (f"leg{leg}", key):
                continue
            jig_deg = args.knee_at if JOINTS[j] == "knee" else 0.0
            jobs.append((key, sid, Family.STS, jig_deg,
                         f"leg{leg} {JOINTS[j]} (id {sid})"))
    for i, sid in enumerate(bp["hand_ids"]):
        key = f"hand{i}_claw"
        if args.only and args.only not in ("hands", key):
            continue
        jobs.append((key, sid, Family.SCS, 0.0,
                     f"hand{i} claw CLOSED (id {sid})"))

    for key, sid, fam, jig_deg, label in jobs:
        hline()
        print(f"{label}: torque OFF — hold at jig pose "
              f"({jig_deg:+.0f} deg CAD frame)")
        bus.torque(sid, False)
        if mt is not None:
            # rehearsal: pretend the joint rests a few degrees off center
            mt.servo(sid)._pos_f = CENTER[fam] + (hash(key) % 91) - 45
            mt.servo(sid).put("PRESENT_POSITION", int(mt.servo(sid)._pos_f))
        if not args.yes:
            input("  press Enter when held steady... ")
        pos = bus.read_reg(sid, "PRESENT_POSITION")
        jig_counts = round(jig_deg * COUNTS[fam] / SWEEP_DEG[fam])
        # model: servo_deg = dir*q_deg + off  =>  off = (pos-center) - dir*jig
        d = cal["dir"].get(key, +1)
        offset = (pos - CENTER[fam]) - d * jig_counts
        cal["offset"][key] = int(offset)
        print(f"  present {pos} counts -> offset {offset:+d} counts "
              f"({offset * SWEEP_DEG[fam] / COUNTS[fam]:+.2f} deg)")
        if not args.skip_dir_test and fam is Family.STS:
            print("  nudge test: rotate the joint a little toward CAD +q "
                  "(yaw: CCW from above / hip: lift up / knee: unfold)")
            if not args.yes:
                input("  press Enter after the nudge... ")
                pos2 = bus.read_reg(sid, "PRESENT_POSITION")
                d = pos2 - pos
                if abs(d) < 5:
                    print("  (too small to judge — keeping dir "
                          f"{cal['dir'].get(key, +1)})")
                else:
                    cal["dir"][key] = 1 if d > 0 else -1
                    print(f"  moved {d:+d} counts -> dir {cal['dir'][key]:+d}")
            else:
                cal["dir"].setdefault(key, +1)
        if args.burn and fam is Family.STS:
            bus.set_position_offset(sid, offset)
            print("  EEPROM POSITION_OFFSET burned")

    with open(CAL_PATH, "w") as f:
        yaml.safe_dump(cal, f, sort_keys=True)
    hline("=")
    print(f"wrote {CAL_PATH} ({len(cal['offset'])} offsets)")
    print("NOTE: offsets assume the dir signs already in calibration.yaml — "
          "if the nudge test flipped any dir, RE-RUN this script once.")
    print("verify: python3 pose_check.py --port ... (commands the neutral "
          "stance at low speed; every leg should match the comb)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
