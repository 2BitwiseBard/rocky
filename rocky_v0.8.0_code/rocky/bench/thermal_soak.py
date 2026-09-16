#!/usr/bin/env python3
"""Thermal soak — how long can a servo hold a pose before it cooks?

The master plan's #1 hardware risk (§7): femur servos overheating in long
stands. This test holds a commanded pose under load and logs temperature /
load / current / voltage at 1 Hz until steady-state or the 65 C cut.

Bench setup (see BENCH_RUNBOOK.md §7): servo on the jig, lever + known mass
giving the D015 stance load (0.85–1.06 N·m => 29–36 % of stall). Run once at
walking-load, once at 3-leg-stance load, once (carefully) at the untucked
43–65 % band D015 flags as a no-go — the whole point is to SEE the no-go.

Outputs: CSV + PNG plot + a summary (time-to-60 C, time-to-cut, projected
steady state) into bench/out/.

    python3 thermal_soak.py --port /dev/ttyACM0 --ids 2 --minutes 20
    python3 thermal_soak.py --mock --ids 2 --minutes 20 --mock-load 62
"""
import csv
import os
import sys
import time
from _common import base_parser, make_bus, hline, OUT_DIR

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "driver"))
from rocky_driver import SafetyMonitor, SafetyLimits                # noqa: E402


def main():
    p = base_parser("Hold-pose thermal soak with 65 C auto-cut")
    p.add_argument("--ids", default="2", help="servo ids, e.g. 2 or 2,5,8")
    p.add_argument("--minutes", type=float, default=20.0)
    p.add_argument("--pose-deg", type=float, default=0.0,
                   help="held position (center-relative deg)")
    p.add_argument("--warn-c", type=int, default=60)
    p.add_argument("--cut-c", type=int, default=65)
    p.add_argument("--note", default="", help="test note for the CSV header")
    args = p.parse_args()

    ids = [int(x) for x in args.ids.split(",")]
    bus, clock, mt = make_bus(args)
    limits = SafetyLimits(temp_warn_c=args.warn_c, temp_cut_c=args.cut_c)
    events = []
    mon = SafetyMonitor(bus, ids, limits,
                        on_event=lambda kind, tel: events.append(
                            (clock.now(), kind, tel.servo_id, tel.temp_c)))

    for sid in ids:
        bus.torque(sid, True)
        bus.set_position(sid, args.pose_deg)
    print(f"holding {args.pose_deg:+.1f} deg on ids {ids} for "
          f"{args.minutes:.0f} min (cut at {args.cut_c} C) ...")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(OUT_DIR, f"soak_{stamp}.csv")
    t0 = clock.now()
    hist = {sid: [] for sid in ids}
    time_to = {sid: {} for sid in ids}
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([f"# thermal soak {stamp} ids={ids} pose={args.pose_deg} "
                    f"note={args.note!r}"])
        w.writerow(["t_s", "id", "temp_c", "load_pct", "current_a",
                    "voltage_v", "pos_counts"])
        while clock.now() - t0 < args.minutes * 60:
            tels = mon.step()
            t = clock.now() - t0
            line = []
            for sid in ids:
                tel = tels[sid]
                hist[sid].append((t, tel.temp_c, tel.load_pct,
                                  tel.current_a or 0.0))
                w.writerow([f"{t:.0f}", sid, tel.temp_c, f"{tel.load_pct:.1f}",
                            f"{tel.current_a or 0:.2f}", f"{tel.voltage_v:.1f}",
                            tel.position_counts])
                for thresh in (50, 55, 60, args.cut_c):
                    if tel.temp_c >= thresh and thresh not in time_to[sid]:
                        time_to[sid][thresh] = t
                line.append(f"id{sid} {tel.temp_c:3d}C {tel.load_pct:4.1f}% "
                            f"{(tel.current_a or 0):4.2f}A")
            if int(t) % (60 if not clock.is_mock else 9999) < 1 or clock.is_mock \
                    and int(t) % 60 == 0:
                print(f"  t={t:5.0f}s  " + "  ".join(line))
            if mon.tripped:
                print(f"  CUT at t={t:.0f}s: ids {sorted(mon.tripped)} — "
                      f"torque disabled, letting them cool")
                break
            clock.sleep(1.0)

    for sid in ids:
        bus.torque(sid, False)
    hline("=")
    for sid in ids:
        temps = [h[1] for h in hist[sid]]
        marks = "  ".join(f"->{k}C @ {v:.0f}s" for k, v in
                          sorted(time_to[sid].items()))
        print(f"id {sid}: start {temps[0]} C, end {temps[-1]} C, "
              f"peak {max(temps)} C   {marks or '(never warmed past 50)'}")
    verdict = ("NO-GO at this load — servo hit the cut. This pose needs "
               "STS3250s, a lower duty, or the sleep-pose timer."
               if mon.tripped else
               "PASS — steady state below the cut at this load.")
    print(verdict)

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for sid in ids:
            ts = [h[0] / 60 for h in hist[sid]]
            ax[0].plot(ts, [h[1] for h in hist[sid]], label=f"id {sid}")
            ax[1].plot(ts, [h[3] for h in hist[sid]], label=f"id {sid}")
        ax[0].axhline(args.cut_c, color="r", ls="--", lw=1, label=f"cut {args.cut_c} C")
        ax[0].axhline(args.warn_c, color="orange", ls=":", lw=1)
        ax[0].set_ylabel("temp (C)"); ax[0].legend(); ax[0].grid(alpha=0.3)
        ax[1].set_ylabel("current (A)"); ax[1].set_xlabel("minutes")
        ax[1].grid(alpha=0.3)
        fig.suptitle(f"thermal soak — {args.note or stamp}")
        png = os.path.join(OUT_DIR, f"soak_{stamp}.png")
        fig.savefig(png, dpi=110, bbox_inches="tight")
        print(f"wrote {csv_path}\nwrote {png}")
    except Exception as e:                                   # pragma: no cover
        print(f"(plot skipped: {e})")
    print("file the numbers in NOTES_INBOX.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
