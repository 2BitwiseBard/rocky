#!/usr/bin/env python3
"""Hardware angle limits — the servo refuses what the software should never ask (D052).

Every command already passes the software soft limits (PebbleRobot.q_to_deg,
the bridge's jog, the gait clamps). Those are code; code has bugs. This burns
the same limits into each servo's EEPROM MIN/MAX_ANGLE_LIMIT, so a NaN that
slipped through, a wrong dir sign or a stray raw write cannot drive a joint
into the frame:

    servo_deg = dir * q_deg + offset_deg          (the PebbleRobot model)
    q range   = params joints.pos_deg  widened by MARGIN_DEG on both sides
    counts    = CENTER + servo_deg * COUNTS / SWEEP, rounded OUTWARD

The margin is outward on purpose: the hardware limit is a backstop BEHIND the
soft limit, never the thing that normally stops a joint (hitting it can set the
servo's angle-error status bit, which the SafetyMonitor treats as a fault).

Offsets — ONE place only (D052 decision): the software offset in
bench/calibration.yaml is the calibration. The STS POSITION_OFFSET EEPROM
register must be 0 wherever a software offset is non-zero; a servo with both
non-zero is refused (its limits would be computed in the wrong frame). A
servo burned with `calibrate_centers.py --burn` carries software offset 0 and
its limits are computed in the offset-applied frame — VERIFY-ON-BENCH that
the STS angle limits live in that frame.

    python3 apply_limits.py --port /dev/ttyACM0            # all servos that answer
    python3 apply_limits.py --port /dev/ttyACM0 --ids 1,2,3 --dry-run
    python3 apply_limits.py --mock --yes
    python3 apply_limits.py --port ... --clear             # back to 0..4095 (factory)

Re-run after every calibrate_centers.py (limits depend on dir + offset).
The cockpit's Hardware panel calls the same functions (HardwareBridge.apply_limits).
"""
from __future__ import annotations
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
for _sub in ("driver", "gait"):
    _p = os.path.normpath(os.path.join(HERE, "..", _sub))
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rocky_model as rm                                            # noqa: E402
from rocky_driver import Family, BusError                           # noqa: E402
from rocky_driver.registers import CENTER, COUNTS, SWEEP_DEG        # noqa: E402

MARGIN_DEG = 2.0


def joint_of(sid: int) -> tuple[int, str, str, Family]:
    """sid -> (leg, joint, calibration key, family). KeyError if not in the map."""
    leg, joint = rm.id_to_joint(sid)
    if joint == "claw":
        return leg, joint, f"hand{leg}_claw", Family.SCS
    return leg, joint, f"leg{leg}_{joint}", Family.STS


def plan(cal: dict | None, ids=None, margin_deg: float = MARGIN_DEG) -> dict[int, dict]:
    """The limits each servo should carry. ids=None -> every id in the params map.
    An entry with `problem` set must not be written (the range crosses the
    0/4095 counts wrap: re-seat the horn one spline and recalibrate)."""
    cal = cal or {}
    dirs, offs = cal.get("dir", {}) or {}, cal.get("offset", {}) or {}
    lim = rm.joint_limits_deg()
    if ids is None:
        ids = [i for row in rm.leg_ids() for i in row] + rm.hand_ids()
    out: dict[int, dict] = {}
    for sid in ids:
        leg, joint, key, fam = joint_of(int(sid))
        d = 1 if int(dirs.get(key, 1)) >= 0 else -1
        off = int(offs.get(key, 0))
        k = COUNTS[fam] / SWEEP_DEG[fam]
        lo, hi = lim[joint]
        q_lo, q_hi = lo - margin_deg, hi + margin_deg
        c = sorted(CENTER[fam] + (d * q + off / k) * k for q in (q_lo, q_hi))
        mn, mx = math.floor(c[0]), math.ceil(c[1])
        e = dict(id=int(sid), key=key, family=fam.name, dir=d, offset=off,
                 q_lo=q_lo, q_hi=q_hi, min=mn, max=mx, problem=None)
        if mn < 0 or mx > COUNTS[fam] - 1:
            e["problem"] = (f"range {mn}..{mx} counts crosses the 0/{COUNTS[fam] - 1} wrap "
                            f"(offset {off:+d}) — re-seat the horn and recalibrate")
            e["min"], e["max"] = max(0, mn), min(COUNTS[fam] - 1, mx)
        out[int(sid)] = e
    return out


def apply(bus, plan_: dict[int, dict], log=print) -> dict[int, dict]:
    """Write + read back each planned servo. Returns {id: {ok, min, max, error}}.
    Refuses (ok False) a planned problem and the double-offset case."""
    res: dict[int, dict] = {}
    for sid, p in plan_.items():
        r = dict(id=sid, key=p["key"], ok=False, min=None, max=None, error=None)
        res[sid] = r
        if p["problem"]:
            r["error"] = p["problem"]
            log(f"  id {sid:2d} {p['key']:<11} REFUSED: {p['problem']}")
            continue
        try:
            if p["family"] == "STS":
                eo = bus.read_reg(sid, "POSITION_OFFSET")
                if eo and p["offset"]:
                    r["error"] = (f"EEPROM POSITION_OFFSET {eo:+d} AND software offset "
                                  f"{p['offset']:+d} are both non-zero (D052: one place only)")
                    log(f"  id {sid:2d} {p['key']:<11} REFUSED: {r['error']}")
                    continue
            r["min"], r["max"] = bus.set_angle_limits(sid, p["min"], p["max"])
            r["ok"] = True
            log(f"  id {sid:2d} {p['key']:<11} q {p['q_lo']:+7.1f}..{p['q_hi']:+7.1f} deg "
                f"-> counts {r['min']:4d}..{r['max']:4d}  verified")
        except (BusError, ValueError) as e:
            r["error"] = f"{type(e).__name__}: {e}"
            log(f"  id {sid:2d} {p['key']:<11} FAILED: {r['error']}")
    return res


def clear(bus, ids, log=print) -> dict[int, tuple[int, int]]:
    """Factory limits (0..COUNTS-1) — before a horn re-seat or a recalibration."""
    out = {}
    for sid in ids:
        fam = bus.family_of(sid)
        out[sid] = bus.set_angle_limits(sid, 0, COUNTS[fam] - 1)
        log(f"  id {sid:2d} limits cleared -> {out[sid]}")
    return out


def main():
    from _common import base_parser, make_bus, confirm, hline, CAL_PATH
    from rocky_driver import load_calibration

    p = base_parser("Write MIN/MAX_ANGLE_LIMIT from the params joint limits + calibration")
    p.add_argument("--ids", default=None, help="comma list (default: every map id that answers)")
    p.add_argument("--margin", type=float, default=MARGIN_DEG,
                   help="deg outside the soft limits (default 2)")
    p.add_argument("--dry-run", action="store_true", help="print the plan, write nothing")
    p.add_argument("--clear", action="store_true", help="reset limits to 0..4095 / 0..1023")
    args = p.parse_args()

    bus, clock, mt = make_bus(args)
    cal = load_calibration(CAL_PATH)
    if args.ids:
        ids = [int(x) for x in args.ids.split(",")]
    else:
        ids = [i for row in rm.leg_ids() for i in row] + rm.hand_ids()
        ids = [i for i in ids if bus.ping(i)]
    if not ids:
        print("no servo answered — nothing to do")
        return 1
    hline()
    if args.clear:
        if not confirm(f"clear angle limits on {ids}?", args):
            return 1
        clear(bus, ids)
        return 0
    pl = plan(cal, ids, args.margin)
    print(f"calibration: {CAL_PATH if cal else '(none — dir +1, offset 0)'}")
    for sid, e in pl.items():
        flag = f"  !! {e['problem']}" if e["problem"] else ""
        print(f"  id {sid:2d} {e['key']:<11} dir {e['dir']:+d} off {e['offset']:+5d}  "
              f"q {e['q_lo']:+7.1f}..{e['q_hi']:+7.1f} -> {e['min']:4d}..{e['max']:4d}{flag}")
    if args.dry_run:
        return 0
    if not confirm("write these limits to EEPROM?", args):
        return 1
    hline()
    res = apply(bus, pl)
    bad = [sid for sid, r in res.items() if not r["ok"]]
    hline("=")
    print(f"{len(res) - len(bad)}/{len(res)} verified" + (f"; NOT applied: {bad}" if bad else ""))
    return 0 if not bad else 2


if __name__ == "__main__":
    sys.exit(main())
