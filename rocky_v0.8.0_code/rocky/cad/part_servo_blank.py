"""Printed ST3215 servo BLANK — session 6. Dry-fit stand-in for the real thing.

Why: session 6's status was "nothing ordered yet" — servo lead time is 1–4
weeks, but the print weekend wants the body + one leg ASSEMBLED. The servos
are the joints; without them the chain can't hang together. This part is a
dimensionally exact PLA stand-in for the ST3215 envelope (case + output boss
+ horn disc), so every cradle, wall, zip slot and horn-coupler interface can
be exercised weeks before hardware lands:

  * drops into the coxa base cradle (yaw pose), the coxa fork rails (femur
    pose) and the tibia knee carrier (knee pose) — all three verified by
    boolean probe below, same FIT clearances as the real servo dummy;
  * the horn disc carries the true 4x M2 BCD (Ø1.7 self-tap) + center
    pilot, so femur_link hubs and sacrificial horn couplers BOLT ON and the
    whole leg chain assembles rigid (joints frozen at print angle — fine,
    dry fit is about interfaces, not motion);
  * 3-dot engraving on both case sides = instantly tells a blank from a
    real servo in a parts bin (font-free convention).

Print: PLA, 0.2 mm, 2 walls, 10 % — ~25 g each, qty 4 (3 for one leg + 1
spare/hand-of-second-leg). NOT a structural part: it exists to be replaced.
When real servos arrive the blanks retire to the bench jig / mass-mockup
drawer. (If a CoM-realistic mockup is ever needed: the real servo is 60 g,
the blank ~25 — tape washers to the case, don't redesign.)

Frame: identical to servo_st3215.servo_body — shaft axis +Z at origin,
case hanging -X, base at z=0.
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body, horn_screw_positions

P = params()
S = P["servo_st3215"]
PR = P["print"]


def servo_blank():
    """Exact ST3215 envelope, printable: solid case + boss + drilled horn."""
    blank = servo_body(P, clearance=0.0)
    # horn screws: 4x M2 self-tap on the true BCD + center pilot, cut deep
    # enough to reach through horn + boss into the case top
    horn_top = S["body_h"] + S["boss_h"] + S["horn_h"]
    for hx, hy in horn_screw_positions(P):
        blank -= Pos(hx, hy, horn_top - 5) * Cylinder(1.7 / 2, 10)
    blank -= Pos(0, 0, horn_top - 5) * Cylinder(2.1 / 2, 10)
    # 3-dot "blank" engraving on both case side faces (font-free)
    front = S["shaft_offset"]
    cx = front - S["body_l"] / 2                      # case x-center
    for sy in (1, -1):
        for d in range(3):
            blank -= Pos(cx - 6 + 6 * d, sy * (S["body_w"] / 2 - 0.5),
                         S["body_h"] / 2) * Rot(90, 0, 0) * Cylinder(1.5, 1.2)
    return blank


if __name__ == "__main__":
    b = servo_blank()
    export(b, "servo_blank")

    # ---- check 1: envelope identity — the blank must not exceed the dummy
    # every pocket in the tree was subtracted with (blank material outside the
    # reference dummy = a blank that binds where a real servo would not)
    dummy = servo_body(P, clearance=0.001)
    excess = b - dummy
    v_ex = 0.0 if excess is None else excess.volume
    print(f"blank outside reference dummy envelope: {v_ex:.2f} mm^3 "
          f"({'OK' if v_ex < 1.0 else 'OVERSIZE — would bind'})")

    # ---- check 2: the blank must drop into all three servo cradles exactly
    # like the real dummy (poses copied from the consuming modules)
    from part_coxa import coxa_yaw_base, coxa_fork, femur_servo_placed, \
        CASE_BASE_Y, Z_FEMUR_AXIS, L1
    from part_tibia import tibia_knee_carrier, KNEE_X, Z_AXIS
    poses = [
        ("coxa base cradle (yaw pose)", coxa_yaw_base(), b),
        ("coxa fork rails (femur pose)", coxa_fork(),
         Pos(L1, CASE_BASE_Y, Z_FEMUR_AXIS) * Rot(90, 0, 0) * b),
        ("knee carrier (knee pose)", tibia_knee_carrier(),
         Pos(KNEE_X, CASE_BASE_Y, Z_AXIS) * Rot(90, 0, 0) * b),
    ]
    worst = 0.0
    for name, cradle, posed in poses:
        inter = cradle & posed
        v = 0.0 if inter is None else inter.volume
        worst = max(worst, v)
        print(f"blank x {name}: {v:.2f} mm^3 ({'OK' if v < 1.0 else 'BINDS'})")
    assert v_ex < 1.0 and worst < 1.0, "servo blank fails fit checks"
    bb = b.bounding_box()
    print(f"servo_blank: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm "
          f"— PLA, 0.2 mm, 2 walls, 10%, qty 4 (~25 g ea)")
