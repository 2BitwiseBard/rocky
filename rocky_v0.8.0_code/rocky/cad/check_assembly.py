"""D046/D047 joint suite — the checks the per-part modules can't ask: can each
LEG-CHAIN joint be put together in order, and what holds it once it is.

Joints (D047): J1 yaw horn+idler <-> fork, J2 hip horn <-> coupler <-> plate A
and idler <-> plate B, J3 the same at the knee, J4 servo <-> cup (all three),
J5 carrier <-> tube (pinch clamp, checked in part_tibia). Each joint:
alignment across both mating parts, driver access against EVERYTHING present
when the screw is driven, thread/nut path, capture (a 2 mm nudge must meet
material — or be a declared bolted direction), and the cable plugs' keep-out.
Written against the v0.8.0 tree, where it found the coxa assembly deadlock
and the unlocated femur link (NOTES_INBOX 2026-09-17); it must stay CLEAN
from here on. Exit code = failures.
"""
import sys
from build123d import *
from common import params
from servo_st3215 import horn_screw_angles, plug_envelope, spec
from servo_mount import cup_screw_columns
from leg_frame import YAW_TF, HIP_TF, KNEE_TF, L1, KNEE_X, Z_HIP
from part_coxa import HUB_Z0, CB_DEPTH
from part_coupler import RECESS_CLAMP_ANGS, TAP_R_POS, DISC_T, HEAD_CB_DEPTH
from part_femur import (LINK_TF, YA0, YA1, T as FEM_T, BOSS_X, BOSS_ZC, RAIL_Y0, YB1, BOSS_H)
from leg_assembly import build_dryfit

P = params(); S = spec(P); PR = P["print"]
v = lambda x: 0.0 if x is None else x.volume
fails = []


def check(name, val, good, ok_msg="OK", bad_msg="FAIL"):
    ok = good(val)
    print(f"  {name}: {val:.2f} mm^3 ({ok_msg if ok else bad_msg})")
    if not ok: fails.append(name)


df = build_dryfit()
base, fork = df["coxa_yaw_base"], df["coxa_fork"]
link, plate_b, carrier = df["femur_link"], df["femur_plate_b"], df["tibia_knee_carrier"]
by, bh, bk = df["blank_yaw"], df["blank_femur"], df["blank_knee"]
everything = base + fork + link + plate_b + carrier + by + bh + bk

print("== J1 yaw: horn + idler <-> fork")
for ang in horn_screw_angles(P):
    for r in (S["horn_bcd"] / 2, S["horn_bcd"] / 2 + S["horn_bcd_slot"]):
        col = Rot(0, 0, ang) * Pos(r, 0, HUB_Z0 + CB_DEPTH - 20) * Cylinder(2.2, 40)
        col = col & Pos(0, 0, HUB_Z0 + CB_DEPTH - 20) * Box(100, 100, 40)   # below the counterbore floor
        check(f"M3 driver column ({ang} deg, r {r:.1f}) from below vs fork", v(col & fork), lambda x: x < 1, "CLEAR", "BLOCKED")
check("fork x yaw blank (horn + idler pockets)", v(fork & by), lambda x: x < 1, "OK", "CLASH")
check("fork nudged +X x yaw blank (horn pocket locates)", v((Pos(2, 0, 0) * fork) & by), lambda x: x > 1, "LOCATED", "LOOSE")
check("fork nudged -X x yaw blank", v((Pos(-2, 0, 0) * fork) & by), lambda x: x > 1, "LOCATED", "LOOSE")
worst = max(v((Pos(dx, 0, 0) * (fork + by)) & base) for dx in (1, 2, 5, 10, 20, 40))
check("servo + fork slide-in path (+X) vs base", worst, lambda x: x < 1, "OPEN", "BLOCKED")
check("fork x yaw plug keep-out", v(fork & (YAW_TF * plug_envelope(P))), lambda x: x < 1, "CLEAR", "BLOCKS PLUGS")
worst = 0.0
for col in cup_screw_columns("bot"):
    worst = max(worst, v((YAW_TF * col) & (fork + bh)))
check("base rim screws (from above) vs fork + hip blank", worst, lambda x: x < 1, "CLEAR", "BLOCKED")

for J, which, blank, cradle, cx, tf in (("J2 hip", "A", bh, fork, L1, HIP_TF),
                                        ("J3 knee", "B", bk, carrier, KNEE_X, KNEE_TF)):
    print(f"== {J}: horn <-> coupler <-> plate A; idler <-> plate B")
    c = df["coupler_hip" if which == "A" else "coupler_knee"]
    check("coupler lobes in hub recess", v(c & link), lambda x: x < 1, "FITS", "INTERFERES")
    check("coupler disc on horn top", v(c & blank), lambda x: x < 1, "OK", "CLASH")
    check("coupler x cradle", v(c & cradle), lambda x: x < 1, "OK", "CLASH")
    check("link nudged +X x coupler (castellation)", v((Pos(2, 0, 0) * link) & c), lambda x: x > 1, "LOCATES", "LOOSE")
    check("link nudged +Z x coupler (castellation)", v((Pos(0, 0, 2) * link) & c), lambda x: x > 1, "LOCATES", "LOOSE")
    # horn screws: driven from the lobe side (-Y) BEFORE the link goes on, at both slot ends
    worst = 0.0
    for ang in horn_screw_angles(P):
        for r in (S["horn_bcd"] / 2, S["horn_bcd"] / 2 + S["horn_bcd_slot"]):
            col = tf * (Rot(0, 0, ang) * Pos(r, 0, S["body_h"] + 40) * Cylinder(2.2, 40))   # off the horn face
            worst = max(worst, v(col & cradle), v(col & blank), v(col & c))
    check("horn-screw driver columns (both slot ends) vs cradle + blank + coupler", worst, lambda x: x < 1, "CLEAR", "BLOCKED")
    # M3 clamps: driven from the outboard face of plate A into the lobes
    worst = 0.0
    for ang in RECESS_CLAMP_ANGS:
        pin = LINK_TF * (Pos(cx - L1, YA0, 0) * Rot(-90, 0, 0) * Rot(0, 0, ang) *
                         Pos(TAP_R_POS, 0, -2 + 9.9 / 2) * Cylinder(PR["screw_m3_tap"] / 2 - 0.05, 9.9))
        col = LINK_TF * (Pos(cx - L1, YA0 - 12, 0) * Rot(-90, 0, 0) * Rot(0, 0, ang) *
                         Pos(TAP_R_POS, 0, 0) * Cylinder(3.0, 24))
        worst = max(worst, v(pin & link), v(pin & c), v(col & cradle), v(col & blank), v(col & fork))
    check("M3 clamp bores coaxial + driver columns", worst, lambda x: x < 1, "CLEAR", "BLOCKED")
    check("plate B tower on the idler", v(plate_b & blank), lambda x: x < 1, "OK", "CLASH")
    check("plate B nudged +Z x idler (pocket locates)", v((Pos(0, 0, 2) * plate_b) & blank), lambda x: x > 1, "LOCATED", "LOOSE")
    check("plate B x cradle", v(plate_b & cradle), lambda x: x < 1, "OK", "CLASH")
    check("plate B x plug keep-out", v(plate_b & (tf * plug_envelope(P))), lambda x: x < 1, "CLEAR", "BLOCKS PLUGS")
    check("cradle x plug keep-out", v(cradle & (tf * plug_envelope(P))), lambda x: x < 1, "CLEAR", "BLOCKS PLUGS")

print("== plate B: 4x M3 from +Y into the bridge bosses")
worst = 0.0
for bx in BOSS_X:
    for sz in (1, -1):
        pin = LINK_TF * (Pos(bx, (RAIL_Y0 - BOSS_H + 1 + YB1 + 2) / 2, sz * BOSS_ZC) * Rot(90, 0, 0) *
                         Cylinder(PR["screw_m3_tap"] / 2 - 0.05, YB1 + 2 - (RAIL_Y0 - BOSS_H + 1)))
        col = LINK_TF * (Pos(bx, YB1 + 12, sz * BOSS_ZC) * Rot(90, 0, 0) * Cylinder(3.0, 24))
        worst = max(worst, v(pin & link), v(pin & plate_b), v(col & everything))
check("plate B screws coaxial + driver columns vs everything", worst, lambda x: x < 1, "CLEAR", "BLOCKED")
worst = max(v((Pos(0, dy, 0) * plate_b) & (everything - plate_b)) for dy in (1, 2, 5, 10, 20))
check("plate B drop-on path (+Y) vs everything", worst, lambda x: x < 1, "OPEN", "BLOCKED")

print("== J4 servo retention in every cup (2 mm nudges; +X is the screwed direction)")
for name, blank, cradle in (("yaw", by, base), ("hip", bh, fork), ("knee", bk, carrier)):
    weakest = min(v((Pos(*d) * blank) & cradle) for d in ((0, 2, 0), (0, -2, 0), (0, 0, 2), (0, 0, -2), (-2, 0, 0)))
    check(f"{name} servo: weakest nudge meets the cup", weakest, lambda x: x > 1, "HELD", "FREE")
    if name == "yaw":
        check("yaw servo yawed 5 deg x base (torque reaction through the walls)", v((Rot(0, 0, 5) * blank) & base),
              lambda x: x > 1, "WALLS TAKE IT", "FREE")
    # rim screws are driven when the servo goes into its cup: BEFORE the femur
    # (couplers, plate A, plate B) goes on — so the neighbours present are the
    # other cups and blanks. Consequence, documented in D047: a knee (or hip)
    # servo swap means plate B + the coupler clamps off first.
    present = {"yaw": fork + carrier, "hip": base + carrier, "knee": base + fork}[name] + by + bh + bk
    worst = 0.0
    tf = {"yaw": YAW_TF, "hip": HIP_TF, "knee": KNEE_TF}[name]
    for face in ("top", "bot"):
        for col in cup_screw_columns(face):
            worst = max(worst, v((tf * col) & present))
    check(f"{name} cup rim-screw driver columns vs the parts present at that step", worst, lambda x: x < 1, "CLEAR", "BLOCKED")

print("== sweeps")
worst = 0.0
for hip in range(-70, 91, 10):
    r = Pos(L1, 0, Z_HIP) * Rot(0, -hip, 0) * Pos(-L1, 0, -Z_HIP)
    moving = r * (link + plate_b)
    worst = max(worst, v(moving & fork), v(moving & bh), v(moving & base))
check("femur (link + plate B) x fork/base/hip servo over hip -70..90", worst, lambda x: x < 1, "OK", "CLASH")

print()
print("check_assembly:", "CLEAN" if not fails else f"{len(fails)} FAILURES")
for f in fails: print("  -", f)
sys.exit(len(fails))
