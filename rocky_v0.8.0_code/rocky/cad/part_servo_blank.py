"""Printed ST3215 servo BLANK v0.2 (D046) — dry-fit stand-in for the real thing.

Why: servos are the joints; without them the chain can't hang together. This
is a dimensionally exact PLA stand-in for the ST3215 envelope (case + output
boss + horn disc) so every cradle, lip, strap and horn-coupler interface can
be exercised before hardware lands.

v0.1 gave the horn disc Ø1.7 pilots for self-tapping M2s: 3 mm of PLA
thread per screw, then 3 mm of AIR under the disc (the BCD sits outside
the Ø6 boss), then un-piloted case — and a Ø20 disc on a Ø6 boss printed
"horn up, no support" is 7 mm of 90° overhang. v0.2:
  * the boss undercut is filled to Ø14 (r 7 = the BCD) from case top to
    horn — no overhang, and a floor for the nuts. Ø14 not Ø20 so the
    cradles' retention lips (4 mm reach) still clear it;
  * 4x Ø2.4 THROUGH holes on the BCD with side-loaded M2 nut slots
    (4.3 x 1.9, z 33.0..34.9): the coupler's M2 x 8 screws pull the nut up
    against the 3.1 mm horn flange. Real servo: horn is tapped, M2 x 6;
  * 3-dot engraving on both case sides = blank, not a servo.

Print: horn UP, 0.2 mm, 2 walls, 10 % — ~25 g, qty 4. NOT structural.

Frame: identical to servo_st3215.servo_body — shaft axis +Z at origin,
case hanging -X, base at z=0.
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body, horn_screw_positions

P = params()
S = P["servo_st3215"]
PR = P["print"]
FILL_D = S["horn_bcd"]                     # 14: undercut fill diameter
NUT_AF, NUT_T = 4.0, 1.6                   # M2 hex nut
NUT_Z0 = S["body_h"] + 1.0                 # slot floor 1 mm above the case top


def _undercut_fill(extra=0.0):
    return Pos(0, 0, S["body_h"] + S["boss_h"]/2) * Cylinder(FILL_D/2 + extra, S["boss_h"])


def servo_blank():
    blank = servo_body(P, clearance=0.0) + _undercut_fill()
    horn_top = S["body_h"] + S["boss_h"] + S["horn_h"]
    r = S["horn_bcd"] / 2
    for k, (hx, hy) in enumerate(horn_screw_positions(P)):
        ang = k * 90.0
        blank -= Pos(hx, hy, horn_top - 4) * Cylinder(PR["screw_m2_clear"]/2, 12)   # through
        slot_r0, slot_r1 = r - (NUT_AF + 0.3)/2, S["horn_d"]/2 + 1.0            # side-loaded nut
        blank -= Rot(0, 0, ang) * Pos((slot_r0 + slot_r1)/2, 0, NUT_Z0 + (NUT_T + 0.3)/2) * \
            Box(slot_r1 - slot_r0, NUT_AF + 0.3, NUT_T + 0.3)
    blank -= Pos(0, 0, horn_top - 5) * Cylinder(2.1 / 2, 10)                   # centre pilot
    front = S["shaft_offset"]
    cx = front - S["body_l"] / 2
    for sy in (1, -1):
        for d in range(3):
            blank -= Pos(cx - 6 + 6 * d, sy * (S["body_w"] / 2 - 0.5),
                         S["body_h"] / 2) * Rot(90, 0, 0) * Cylinder(1.5, 1.2)
    return blank


if __name__ == "__main__":
    import sys
    b = servo_blank()
    export(b, "servo_blank")
    v = lambda x: 0.0 if x is None else x.volume
    fails = []
    # 1: envelope identity — nothing outside the reference dummy except the
    # declared Ø14 undercut fill
    ref = servo_body(P, clearance=0.001) + _undercut_fill(0.001)
    v_ex = v(b - ref)
    print(f"blank outside reference envelope (+Ø{FILL_D} fill): {v_ex:.2f} mm^3 "
          f"({'OK' if v_ex < 1.0 else 'OVERSIZE — would bind'})")
    if v_ex >= 1: fails.append("envelope")
    # 2: drops into all three cradles like the real dummy, and is RETAINED
    from part_coxa import coxa_yaw_base, coxa_fork, coxa_fork_strap, CASE_BASE_Y, Z_FEMUR_AXIS, L1
    from part_tibia import tibia_knee_carrier, tibia_knee_strap, KNEE_X, Z_AXIS
    fork, carrier = coxa_fork(), tibia_knee_carrier()
    poses = [
        ("coxa base cradle (yaw pose)", coxa_yaw_base(), None, b),
        ("coxa fork rails (femur pose)", fork, coxa_fork_strap(),
         Pos(L1, CASE_BASE_Y, Z_FEMUR_AXIS) * Rot(90, 0, 0) * b),
        ("knee carrier (knee pose)", carrier, tibia_knee_strap(),
         Pos(KNEE_X, CASE_BASE_Y, Z_AXIS) * Rot(90, 0, 0) * b),
    ]
    for name, cradle, strap, posed in poses:
        fit = v(cradle & posed)
        line = f"blank x {name}: {fit:.2f} mm^3 ({'OK' if fit < 1.0 else 'BINDS'})"
        if fit >= 1: fails.append(name)
        if strap is not None:
            lips = v((Pos(0, -2, 0) * posed) & cradle)
            held = v((Pos(0, 0, 2) * posed) & strap)
            line += f"; nudge -Y {lips:.0f} mm^3 ({'lips hold' if lips > 1 else 'FREE'}), " \
                    f"+Z {held:.0f} mm^3 ({'strap holds' if held > 1 else 'FREE'})"
            if lips <= 1 or held <= 1: fails.append(name + " retention")
        print(line)
    # 3: nut slots empty, M2 passes coupler-side to nut
    for k, (hx, hy) in enumerate(horn_screw_positions(P)):
        pin = Pos(hx, hy, S["body_h"] + 3) * Cylinder(1.0, 8)
        if v(pin & b) > 0.5: fails.append(f"M2 path {k}")
    print(f"M2 through-paths clear: {'yes' if not any(f.startswith('M2') for f in fails) else 'NO'}")
    bb = b.bounding_box()
    print(f"servo_blank: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm — PLA, horn UP, 2 walls, 10%, qty 4")
    print(f"part_servo_blank checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails: sys.exit(1)
