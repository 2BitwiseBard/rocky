#!/usr/bin/env python3
"""Pose check — command the neutral stance slowly and compare against the jig.

The calibration acceptance test: after calibrate_centers.py, this commands
every leg joint to the gait-neutral pose (q = 0, 0, -90 deg) at LOW speed and
low torque, then prints commanded-vs-read errors. On the jig/comb everything
should visually line up; errors > ~2 deg mean a bad offset or dir sign.

    python3 pose_check.py --port /dev/ttyACM0
    python3 pose_check.py --mock --yes
"""
import math
import os
import sys
from _common import base_parser, make_bus, confirm, hline, CAL_PATH

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "driver"))
from rocky_driver import PebbleRobot, load_calibration              # noqa: E402

NEUTRAL = [0.0, 0.0, math.radians(-90)]        # yaw, hip, knee (gait frame)


def main():
    p = base_parser("Slow neutral-pose acceptance test after calibration")
    p.add_argument("--speed", type=int, default=200,
                   help="counts/s (200 ~ 17 deg/s — slow)")
    args = p.parse_args()
    bus, clock, mt = make_bus(args)
    robot = PebbleRobot(bus, calibration=load_calibration(CAL_PATH))

    print("SAFETY: legs clear of the bench? torque comes on at low speed.")
    if not confirm("command neutral stance?", args):
        return 1
    # D052 V2: park each goal where the servo IS, at 40 % torque, THEN enable
    # (enable-first let 15 servos drive toward stale goals at full torque)
    legs = [i for row in robot.leg_ids for i in row]
    here = robot.soft_enable(legs, torque_limit=400, speed_cps=args.speed)
    missing = sorted(set(legs) - set(here))
    if missing:
        print(f"no answer from {missing} — they stay limp; the rest continue")
    robot.soft_enable(robot.hand_ids, torque_limit=1000, speed_cps=args.speed)
    robot.send_leg_targets([NEUTRAL] * 5, speed_cps=args.speed)
    robot.send_claws([0.0] * 5)
    # wait for the slowest joint at this speed (+1 s): a fixed 3 s left a 90 deg knee
    # move from a limp 0 deg a third short at 200 cps and reported it as a bad offset
    gap = max([abs(robot.q_to_deg(leg, j, NEUTRAL[j]) - here[sid])
               for leg, row in enumerate(robot.leg_ids) for j, sid in enumerate(row) if sid in here] or [0.0])
    dps = max(args.speed, 1) * 360.0 / 4096.0
    for _ in range(int(10 * (gap / dps + 1.0)) + 1):
        clock.sleep(0.1)

    q, tels = robot.read_joint_state()
    hline()
    worst = 0.0
    for leg in range(5):
        errs = [math.degrees(q[leg][j] - NEUTRAL[j]) for j in range(3)]
        worst = max(worst, max(abs(e) for e in errs))
        print(f"leg{leg}: err yaw {errs[0]:+6.2f}  hip {errs[1]:+6.2f}  "
              f"knee {errs[2]:+6.2f} deg")
    robot.release_limits(legs)
    print(f"worst joint error {worst:.2f} deg -> "
          f"{'PASS (<2 deg)' if worst < 2 else 'CHECK offsets/dir signs'}")
    if confirm("torque off (limp)?", args):
        robot.limp()
    return 0


if __name__ == "__main__":
    sys.exit(main())
