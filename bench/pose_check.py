#!/usr/bin/env python3
"""Pose check — command the jig pose slowly and compare against the jig.

The calibration acceptance test: after calibrate_centers.py, this commands
every leg joint that answers to the jig pose (femur horizontal, knee -90:
q = 0, 0, -90 deg — the calibration pose, not the standing stance) at LOW
speed and low torque, then prints commanded-vs-read errors. In the jig
everything should visually line up; errors > ~2 deg mean a bad offset or dir
sign.

It works on a partial bus: by default it pings every id of the params bus map
and checks the ones that answer (the one-leg bench is ids 1-3); --ids narrows
that to a list. A joint that is not on the bus is shown as '--'.

    python3 pose_check.py --port /dev/ttyACM0
    python3 pose_check.py --port /dev/ttyACM0 --ids 1-3
    python3 pose_check.py --mock --yes
    python3 pose_check.py --mock --yes --mock-ids 1-3,5
"""
import math
import os
import sys
from _common import base_parser, make_bus, confirm, hline, cal_path, parse_ids

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "driver"))
from rocky_driver import Family, PebbleRobot, load_calibration      # noqa: E402
from rocky_driver.registers import COUNTS, SWEEP_DEG                # noqa: E402

JIG_POSE = [0.0, 0.0, math.radians(-90)]       # yaw, hip, knee (gait frame): the jig pose
JOINTS = ("yaw", "hip", "knee")


def main():
    p = base_parser("Slow jig-pose acceptance test after calibration")
    p.add_argument("--speed", type=int, default=200,
                   help="counts/s (200 ~ 17 deg/s — slow)")
    p.add_argument("--ids", default=None,
                   help="ids to check, e.g. 1-3 or 1-3,16 (default: every map id that answers)")
    args = p.parse_args()
    bus, clock, mt = make_bus(args)
    cal = cal_path(args)
    robot = PebbleRobot(bus, calibration=load_calibration(cal))
    print(f"calibration: {cal}")

    mapped = robot.all_ids()
    wanted = parse_ids(args.ids) if args.ids else mapped
    unknown = sorted(set(wanted) - set(mapped))
    if unknown:
        sys.exit(f"ids {unknown} are not in the params bus map {mapped[0]}..{mapped[-1]}")
    present = [i for i in wanted if bus.ping(i)]
    silent = [i for i in wanted if i not in present]
    legs = [i for row in robot.leg_ids for i in row if i in present]
    hands = [i for i in robot.hand_ids if i in present]
    if not legs and not hands:
        print(f"no servo answered (tried {wanted}) — nothing to check")
        return 1
    print(f"on the bus: {present}" + (f"; not answering: {silent}" if silent else ""))

    print("SAFETY: legs clear of the bench? torque comes on at low speed.")
    if not confirm("command the jig pose?", args):
        return 1
    # D052 V2: park each goal where the servo IS, at 40 % torque, THEN enable
    # (enable-first let 15 servos drive toward stale goals at full torque)
    here = robot.soft_enable(legs, torque_limit=400, speed_cps=args.speed) if legs else {}
    missing = sorted(set(legs) - set(here))
    if missing:
        print(f"no answer from {missing} — they stay limp; the rest continue")
    if hands:
        robot.soft_enable(hands, torque_limit=1000, speed_cps=args.speed)
    joints = [(leg, j, sid) for leg, row in enumerate(robot.leg_ids)
              for j, sid in enumerate(row) if sid in here]
    targets = {sid: robot.q_to_deg(leg, j, JIG_POSE[j]) for leg, j, sid in joints}
    if targets:
        bus.sync_positions(targets, args.speed)
    if hands:                                   # claws CLOSED (open_frac 0), as send_claws does
        closed = robot.soft_limits_deg["claw"][0]
        k = SWEEP_DEG[Family.SCS] / COUNTS[Family.SCS]
        bus.sync_positions({sid: robot.dir.get(f"hand{i}_claw", 1) * closed
                            + robot.offset.get(f"hand{i}_claw", 0) * k
                            for i, sid in enumerate(robot.hand_ids) if sid in hands})
    # wait for the slowest joint at this speed (+1 s): a fixed 3 s left a 90 deg knee
    # move from a limp 0 deg a third short at 200 cps and reported it as a bad offset
    gap = max([abs(targets[sid] - here[sid]) for sid in targets] or [0.0])
    dps = max(args.speed, 1) * 360.0 / 4096.0
    for _ in range(int(10 * (gap / dps + 1.0)) + 1):
        clock.sleep(0.1)

    tels = bus.sync_telemetry([sid for _, _, sid in joints]) if joints else {}
    hline()
    worst = 0.0
    for leg, row in enumerate(robot.leg_ids):
        if not any(sid in here for sid in row):
            continue
        cells = []
        for j, sid in enumerate(row):
            if sid in tels:
                e = math.degrees(robot.deg_to_q(leg, j, tels[sid].position_deg) - JIG_POSE[j])
                worst = max(worst, abs(e))
                cells.append(f"{JOINTS[j]} {e:+6.2f}")
            else:
                cells.append(f"{JOINTS[j]} {'--':>6}")
        print(f"leg{leg}: err " + "  ".join(cells) + " deg")
    robot.release_limits(list(here) + hands)
    if joints:
        print(f"worst joint error over {len(joints)} joint(s) {worst:.2f} deg -> "
              f"{'PASS (<2 deg)' if worst < 2 else 'CHECK offsets/dir signs'}")
    else:
        print("no leg joint answered — the hands were only closed")
    if confirm("torque off (limp)?", args):
        bus.torque_all(present, False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
