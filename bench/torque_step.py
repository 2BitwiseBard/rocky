#!/usr/bin/env python3
"""Torque-step test — measure the real holding margin against D015's numbers.

Setup: servo on the jig with a lever arm + known mass (BENCH_RUNBOOK.md §8),
so gravity applies a KNOWN torque. The script then steps TORQUE_LIMIT down
from 100 % and watches position error: the limit where the servo starts to
sag equals the actual load as a fraction of stall. Compare with prediction:

    predicted %stall = m * g * arm / 2.94 N*m * 100

D015 margins to reproduce: walking hips 22–33 %, 3-leg stance 29–36 %
(both should HOLD comfortably); untucked-manip 43–65 % (should sag/derate —
that's the STS3250 sales pitch, and why manip keeps feet tucked until then).

Safety: temp cut at 65 C, sag limit 15 deg (test aborts + torque limit
restored), and the mass should be a bag of screws on a string — nothing rigid.

    python3 torque_step.py --port ... --id 2 --mass-g 350 --arm-mm 100
    python3 torque_step.py --mock --id 2 --mass-g 350 --arm-mm 100 --mock-load 33
"""
import csv
import os
import sys
import time
from _common import base_parser, make_bus, hline, OUT_DIR

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "driver"))
from rocky_driver.registers import COUNTS, SWEEP_DEG                # noqa: E402
from rocky_driver.protocol import Family                            # noqa: E402
from rocky_driver import soft_enable                                # noqa: E402

STALL_NM = 2.94          # ST3215 @ 12 V (D002); STS3250: 4.90 — use --stall-nm
G = 9.81


def main():
    p = base_parser("Step torque limit down to find the holding threshold")
    p.add_argument("--id", type=int, default=2)
    p.add_argument("--mass-g", type=float, required=True)
    p.add_argument("--arm-mm", type=float, required=True)
    p.add_argument("--stall-nm", type=float, default=STALL_NM)
    p.add_argument("--hold-deg", type=float, default=0.0,
                   help="held pose (lever horizontal = worst case = 0)")
    p.add_argument("--steps", default="100,80,60,50,45,40,35,30,25,20,15,10")
    p.add_argument("--dwell-s", type=float, default=8.0)
    p.add_argument("--sag-limit-deg", type=float, default=8.0)
    args = p.parse_args()

    sid = args.id
    load_nm = args.mass_g / 1000 * G * args.arm_mm / 1000
    pred_pct = load_nm / args.stall_nm * 100
    print(f"lever load: {load_nm:.3f} N*m = {pred_pct:.1f} % of "
          f"{args.stall_nm} N*m stall (prediction to beat)")

    bus, clock, mt = make_bus(args)
    fam = bus.family_of(sid)
    if fam is not Family.STS:
        sys.exit("torque_step is for the STS legs (TORQUE_LIMIT register)")
    if mt is not None and args.mock_load == 0.0:
        mt.servo(sid).external_load_pct = pred_pct   # mock: lever = predicted

    cpd = COUNTS[fam] / SWEEP_DEG[fam]
    # D052 V2: goal parked where the servo IS before torque comes on, then a slow move
    here = soft_enable(bus, [sid], torque_limit=1000, acc=10, speed_cps=200)
    if sid not in here:
        sys.exit(f"id {sid}: no answer")
    bus.set_position(sid, args.hold_deg, 200)
    clock.sleep(abs(args.hold_deg - here[sid]) / (200 * 360 / 4096) + 1.5)
    ref = bus.read_reg(sid, "PRESENT_POSITION")
    print(f"reference position {ref} counts; stepping torque limit...")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    rows = []
    threshold_pct = None
    for step_pct in [float(x) for x in args.steps.split(",")]:
        bus.write_reg(sid, "TORQUE_LIMIT", int(step_pct * 10))
        sagged = False
        t0 = clock.now()
        worst = 0.0
        while clock.now() - t0 < args.dwell_s:
            clock.sleep(0.5)
            tel = bus.telemetry(sid)
            sag_deg = abs(tel.position_counts - ref) / cpd
            worst = max(worst, sag_deg)
            rows.append((f"{clock.now():.1f}", step_pct, tel.position_counts,
                         f"{sag_deg:.2f}", f"{tel.load_pct:.1f}",
                         f"{tel.current_a or 0:.2f}", tel.temp_c))
            if tel.temp_c >= 65:
                print("  ABORT: over-temp"); sagged = True; break
            if sag_deg > args.sag_limit_deg:
                sagged = True
                break
        print(f"  limit {step_pct:5.1f} %: worst sag {worst:5.2f} deg "
              f"{'-> SAGGED' if sagged else '(holds)'}")
        if sagged:
            threshold_pct = step_pct
            break

    bus.write_reg(sid, "TORQUE_LIMIT", 1000)
    bus.set_position(sid, args.hold_deg)
    hline("=")
    csv_path = os.path.join(OUT_DIR, f"torque_step_{stamp}.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([f"# torque step id={sid} mass_g={args.mass_g} "
                    f"arm_mm={args.arm_mm} pred_pct={pred_pct:.1f}"])
        w.writerow(["t_s", "limit_pct", "pos", "sag_deg", "load_pct",
                    "current_a", "temp_c"])
        w.writerows(rows)
    if threshold_pct is None:
        print(f"never sagged — actual load < {args.steps.split(',')[-1]} % "
              f"of stall (predicted {pred_pct:.1f} %)")
    else:
        print(f"sag threshold: {threshold_pct:.0f} % limit vs predicted "
              f"{pred_pct:.1f} % -> measured/predicted = "
              f"{threshold_pct / pred_pct:.2f}")
        print("within ~0.8–1.3 => torque model trustworthy; outside that, "
              "re-check the lever arm, supply sag, and D015's margins")
    print(f"wrote {csv_path}\nfile the ratio in NOTES_INBOX.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
