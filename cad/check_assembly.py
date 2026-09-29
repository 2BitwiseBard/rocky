"""D046/D047 joint suite — the checks the per-part modules can't ask: can each
LEG-CHAIN joint be put together in order, and what holds it once it is.

Joints (D047): J1 yaw horn+idler <-> fork, J2 hip horn <-> coupler <-> plate A
and idler <-> plate B, J3 the same at the knee, J4 servo <-> cup (all three),
J5 carrier <-> tube (pinch clamp, checked in part_tibia). Plus the leg
HARNESS: a clear 11 x 11 path from the deck's I1 cable pass-through to the
yaw servo's plugs, against every part at every yaw. Each joint:
alignment across both mating parts, driver access against EVERYTHING present
when the screw is driven, thread/nut path, capture (a 2 mm nudge must meet
material — or be a declared bolted direction), and the cable plugs' keep-out.
It found the D046 coxa assembly deadlock and the unlocated femur link
(D046, docs/decisions.md); it missed the fork that could not go onto the yaw
servo (B80: no check moved the fork against the servo) until D063 added J1's
slide-on paths. It must stay CLEAN from here on. Exit code = failures.
"""
import sys
from build123d import *
from common import params
from servo_st3215 import horn_screw_angles, plug_envelope, servo_body, spec, z_levels
from servo_mount import cup_screw_columns
from leg_frame import YAW_TF, HIP_TF, KNEE_TF, L1, KNEE_X, Z_HIP
from part_coxa import (HUB_Z0, CB_DEPTH, harness_path, harness_solid, HARNESS_W,
                       fork_slide_worst, horn_screw_shanks)
from iface import CABLE_CUTOUT_X, CABLE_CUTOUT_W
from part_coupler import RECESS_CLAMP_ANGS, TAP_R_POS
from part_femur import (LINK_TF, YA0, BOSS_X, BOSS_ZC, RAIL_Y0, YB1, BOSS_H)
from leg_assembly import build_dryfit

P = params(); S = spec(P); Z = z_levels(P); PR = P["print"]
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
check("fork x yaw blank (horn face on the hub top, idler in its pocket)", v(fork & by),
      lambda x: x < 1, "OK", "CLASH")
# step 1, the fork's own path (B80): on from the servo's front, moving -X (the web closes +X)
ys = YAW_TF * servo_body(P)
check("fork slide-on path (-X) onto the yaw blank (+ glued idler), dx 0.5..30", fork_slide_worst(fork, by),
      lambda x: x < 1, "OPEN", "BLOCKED")
check("fork slide-on path (-X) onto the real yaw servo, dx 0.5..30", fork_slide_worst(fork, ys),
      lambda x: x < 1, "OPEN", "BLOCKED")
# the seat (B81): the screws pull the hub onto the horn. Pulled 0.2 (less than the centre
# head's 0.3 relief gap) it must meet the horn FACE, and the head nothing
horn_face = YAW_TF * (Pos(0, 0, (Z["horn0"] + Z["horn1"]) / 2) * Cylinder(S["horn_d"] / 2, S["horn_t"]))
horn_head = YAW_TF * (Pos(0, 0, (Z["horn1"] + Z["horn_head"]) / 2) *
                      Cylinder(S["horn_center_head_d"] / 2, S["horn_center_head_h"]))
pulled = Pos(0, 0, 0.2) * fork
check("fork pulled 0.2 onto the horn x its face (the face bears on the hub top)", v(pulled & horn_face),
      lambda x: x > 1, "BEARS", "AIR UNDER THE HORN")
check("fork pulled 0.2 onto the horn x its centre head (clear of the relief floor)", v(pulled & horn_head),
      lambda x: x < 1, "CLEAR", "BEARS ON THE HEAD")
# capture (2 mm nudges): the idler pocket's kept walls take -X and +-Y; +X, toward the open
# mouth, is the BOLTED direction: the four horn screws in their 45 deg radial slots take it
check("fork nudged -X x yaw blank (idler pocket's +X wall locates)", v((Pos(-2, 0, 0) * fork) & by),
      lambda x: x > 1, "LOCATED", "LOOSE")
check("fork nudged +-Y x yaw blank (idler pocket's side walls locate)",
      min(v((Pos(0, dy, 0) * fork) & by) for dy in (2, -2)), lambda x: x > 1, "LOCATED", "LOOSE")
check("fork nudged +X x the 4 horn screws (bolted direction)", v((Pos(2, 0, 0) * fork) & horn_screw_shanks()),
      lambda x: x > 1, "BOLTED", "LOOSE")
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

print("== leg harness: deck I1 cable pass-through -> yaw servo plugs (XT30 + JST-XH-5)")
segs = harness_path()
_ax = {"x": 0, "y": 1, "z": 2}
narrow = min(min(b[2 * k + 1] - b[2 * k] for k in range(3) if k != _ax[t]) for _, b, t in segs)
check(f"narrowest section across the travel (>= {HARNESS_W:.0f} mm, as mm)", narrow,
      lambda x: x >= HARNESS_W - 1e-6, "OPEN", "TOO TIGHT")
_boxes = [Pos((b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2) *
          Box(b[1] - b[0], b[3] - b[2], b[5] - b[4]) for _, b, _ in segs]
worst = min(v(a & b) for a, b in zip(_boxes, _boxes[1:]))
check("consecutive path segments overlap (continuous route)", worst, lambda x: x > 1, "CONTINUOUS", "BROKEN")
harness = harness_solid()
mouth = Pos(CABLE_CUTOUT_X, 0, -4 + 0.5) * Box(CABLE_CUTOUT_W, CABLE_CUTOUT_W, 1.0)
check("path starts on the whole deck cutout (11 x 11 x 1 slab above it)", v(harness & mouth),
      lambda x: x > CABLE_CUTOUT_W ** 2 - 1, "ON THE CUTOUT", "MISSES IT")
check("path ends on the yaw plugs (plug keep-out lifted 1 mm meets it)",
      v(harness & (Pos(0, 0, 1) * YAW_TF * plug_envelope(P))), lambda x: x > 1, "REACHES", "SHORT")
check("path x base + yaw blank (static)", v(harness & (base + by)), lambda x: x < 1, "CLEAR", "BLOCKED")
yaw_lo, yaw_hi = P["joints"]["pos_deg"]["yaw"]
moving = fork + bh + bk + link + plate_b + carrier + df["coupler_hip"] + df["coupler_knee"]
worst = max(v((Rot(0, 0, yaw) * moving) & harness) for yaw in range(yaw_lo, yaw_hi + 1, 10))
check(f"path x the yawing leg (fork, femur, knee, blanks) over yaw {yaw_lo}..{yaw_hi}", worst,
      lambda x: x < 1, "CLEAR", "BLOCKED")

print()
print("check_assembly:", "CLEAN" if not fails else f"{len(fails)} FAILURES")
for f in fails: print("  -", f)
sys.exit(len(fails))
