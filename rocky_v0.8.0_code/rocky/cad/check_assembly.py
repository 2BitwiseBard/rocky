"""D046 joint suite — the checks the per-part modules can't ask: can each
LEG-CHAIN joint be put together in order, and what holds it once it is.

Five joints: J1 yaw horn<->fork (+cap/axle), J2 hip horn<->coupler<->femur,
J3 knee horn<->coupler<->femur, J4 knee servo<->carrier, J5 carrier<->tube
(pinch clamp, checked in part_tibia). Each joint: alignment across both
parts, driver access against EVERYTHING present when the screw is driven,
thread/nut path, and capture (a 2 mm nudge must meet material — or be a
declared bolted direction). Written against the v0.8.0 tree, where it found
the coxa assembly deadlock and the unlocated femur link (NOTES_INBOX
2026-09-17); it must stay CLEAN from here on. Exit code = failures.
"""
import math, sys
from build123d import *
from common import params
from servo_st3215 import horn_screw_positions
from part_coxa import (coxa_yaw_base, coxa_fork, coxa_crown_cap, fork_screw_positions,
                       H_HORN_TOP, HUB_T, Z_BOSS_TOP, HEAD_POCKET_Z1, L1, CASE_BASE_Y,
                       Z_FEMUR_AXIS, HORN_TOP_Y)
from part_coupler import RECESS_CLAMP_ANGS, TAP_R_POS, DISC_T
from part_femur import T as FEM_T
from part_tibia import KNEE_X, Z_AXIS
from leg_assembly import build_dryfit

P = params(); S = P["servo_st3215"]; PR = P["print"]
v = lambda x: 0.0 if x is None else x.volume
fails = []
def check(name, val, good, ok_msg="OK", bad_msg="FAIL"):
    ok = good(val)
    print(f"  {name}: {val:.2f} mm^3 ({ok_msg if ok else bad_msg})")
    if not ok: fails.append(name)

df = build_dryfit()
base, fork, cap = df["coxa_yaw_base"], df["coxa_fork"], df["coxa_crown_cap"]
link, carrier = df["femur_link"], df["tibia_knee_carrier"]

print("== J1 yaw: horn <-> fork <-> cap/axle")
for hx, hy in fork_screw_positions():
    col = Pos(hx, hy, (H_HORN_TOP + HUB_T + 60) / 2) * Cylinder(1.9, 60 - (H_HORN_TOP + HUB_T))
    check(f"M2 driver column ({hx:+.1f},{hy:+.1f}) vs fork+base", v(col & fork) + v(col & base),
          lambda x: x < 1, "CLEAR", "BLOCKED")
worst = max(v((Pos(0, 0, dz) * fork) & (base + df["blank_yaw"])) for dz in (20, 10, 5, 1))
check("fork drop-on path (+Z) vs base+servo", worst, lambda x: x < 1, "OPEN", "BLOCKED")
worst = max(v((Pos(0, 0, dz) * cap) & (base + fork)) for dz in (30, 20, 10, 5, 1))
check("cap drop-on path (+Z) vs base+fork", worst, lambda x: x < 1, "OPEN", "BLOCKED")
axle = Pos(0, 0, (HEAD_POCKET_Z1 + 56) / 2) * Cylinder(1.4, 56 - HEAD_POCKET_Z1)   # M3 shank
check("axle M3 path through cap Ø3.4 (pin Ø2.8)", v(axle & cap), lambda x: x < 1, "CLEAR", "BLOCKED")
check("fork+axle nudged 2 mm x cap (yaw stage captured)", v((Pos(2, 0, 0) * (fork + axle)) & cap),
      lambda x: x > 1, "CAPTURED", "FREE")
check("cap x fork at rest (0.4 mm boss gap)", v(cap & fork), lambda x: x < 1, "OK", "CLASH")

for J, which, blank_key, cradle_key, cx in (("J2 hip", "A", "blank_femur", "coxa_fork", L1),
                                            ("J3 knee", "B", "blank_knee", "tibia_knee_carrier", KNEE_X)):
    print(f"== {J}: horn <-> coupler <-> femur hub {which}")
    c = df["coupler_hip" if which == "A" else "coupler_knee"]
    blank, cradle = df[blank_key], df[cradle_key]
    check("coupler lobes in hub recess", v(c & link), lambda x: x < 1, "FITS", "INTERFERES")
    check("coupler disc on horn top", v(c & blank), lambda x: x < 1, "OK", "CLASH")
    check("coupler x cradle", v(c & cradle), lambda x: x < 1, "OK", "CLASH")
    check("link nudged +X x coupler (castellation)", v((Pos(2, 0, 0) * link) & c), lambda x: x > 1, "LOCATES", "LOOSE")
    check("link nudged +Z x coupler (castellation)", v((Pos(0, 0, 2) * link) & c), lambda x: x > 1, "LOCATES", "LOOSE")
    # M2 horn screws: driven from the lobe side BEFORE the link goes on
    worst = 0.0
    for hx, hz in horn_screw_positions(P):
        col = Pos(cx + hx, HORN_TOP_Y - DISC_T - 15, Z_FEMUR_AXIS + hz) * Rot(90, 0, 0) * Cylinder(2.2, 30)
        worst = max(worst, v(col & cradle), v(col & blank), v(col & c))
        pin = Pos(cx + hx, HORN_TOP_Y - 2, Z_FEMUR_AXIS + hz) * Rot(90, 0, 0) * Cylinder(1.0, 12)
        worst = max(worst, v(pin & blank), v(pin & c))
    check("M2 driver columns + Ø2.0 pin to the nut", worst, lambda x: x < 1, "CLEAR", "BLOCKED")
    # M3 clamps: driven from the outboard face of the link, into the lobes
    worst = 0.0
    for ang in RECESS_CLAMP_ANGS:
        pin = Pos(cx, HORN_TOP_Y - DISC_T - FEM_T, Z_FEMUR_AXIS) * Rot(-90, 0, 0) * \
            Rot(0, 0, ang) * Pos(TAP_R_POS, 0, -2 + 9.9 / 2) * Cylinder(PR["screw_m3_tap"]/2 - 0.05, 9.9)
        col = Pos(cx, HORN_TOP_Y - DISC_T - FEM_T - 12, Z_FEMUR_AXIS) * Rot(-90, 0, 0) * \
            Rot(0, 0, ang) * Pos(TAP_R_POS, 0, 0) * Cylinder(3.0, 24)
        worst = max(worst, v(pin & link), v(pin & c), v(col & cradle), v(col & blank))
    check("M3 clamp bores coaxial + driver columns", worst, lambda x: x < 1, "CLEAR", "BLOCKED")

print("== J4 servo retention in every cradle (2 mm nudges)")
for name, blank, cradle, strap in (("yaw", df["blank_yaw"], base + fork + cap, None),
                                   ("femur", df["blank_femur"], fork, df["coxa_fork_strap"]),
                                   ("knee", df["blank_knee"], carrier, df["tibia_knee_strap"])):
    if strap is None:
        # yaw servo: horn bolted to the fork, fork pinned by the cap — the
        # servo moves WITH the fork; the cradle walls take its torque reaction
        check(f"{name} servo+fork+axle nudged +X x base+cap", v((Pos(2, 0, 0) * (blank + fork + axle)) & (base + cap)),
              lambda x: x > 1, "CAPTURED", "FREE")
        check(f"{name} servo yawed 5 deg x base (torque reaction)", v((Rot(0, 0, 5) * blank) & base),
              lambda x: x > 1, "WALLS TAKE IT", "FREE")
    else:
        check(f"{name} servo nudged -Y x cradle (lips)", v((Pos(0, -2, 0) * blank) & cradle), lambda x: x > 1, "HELD", "FREE")
        check(f"{name} servo nudged +Z x strap", v((Pos(0, 0, 2) * blank) & strap), lambda x: x > 1, "HELD", "FREE")
        check(f"{name} servo nudged +X x cradle", v((Pos(2, 0, 0) * blank) & cradle), lambda x: x > 1, "HELD", "FREE")

print()
print("check_assembly:", "CLEAN" if not fails else f"{len(fails)} FAILURES")
for f in fails: print("  -", f)
sys.exit(len(fails))
