"""Printed ST3215 servo BLANK v0.3 (D047) — dry-fit stand-in for the real thing.

Why: servos are the joints; without them the chain can't hang together. This
is a dimensionally exact PLA stand-in for the ST3215 envelope as MEASURED on
the STEP (servo_st3215 v2) so every cup, hub pocket and coupler interface can
be exercised before hardware lands — and, once the servos are here, kept as
the bench dummy for print-fit tests.

Two prints per blank:
  servo_blank : case + rims + plateau + output flange + horn disc. Bottom-down,
                no support. Declared deviations vs the reference solid: the
                connector housing and pins are omitted (the cable side is open
                in every cup anyway), the horn's undercut is filled to Ø20 so
                the disc is not a 5.5 mm cantilever, and the bottom is filled
                flat to the rim plane (the real case is recessed 1.5 mm there)
                so the rims-down print has no overhang. The horn carries four Ø2.5 pilots
                on the nominal pattern (self-tap M3 x 6 / M2 x 6 straight
                into PLA — dry-fit only) and the idler pocket.
  blank_idler : the rear idler wheel (Ø19.9 x 2.1 + centre head) with a Ø6
                stub (= the real axle boss) that glues into the case bottom. Separate because a disc
                under the case would be an island in any print pose.

Frame: identical to servo_st3215.servo_body — shaft axis +Z at origin, case
hanging -X, z=0 at the case bottom flat face.
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body, horn_screw_positions, spec, z_levels, case_xspan

P = params()
S = spec(P)
Z = z_levels(P)
PR = P["print"]
STUB_D = 6.0                          # = the real axle boss; the puck's stub runs through the gap into a glue pocket
STUB_INTO_CASE = 2.5


def _undercut_fill(extra=0.0):
    return Pos(0, 0, (Z["top"] + Z["horn0"]) / 2) * Cylinder(S["horn_d"] / 2 + extra, Z["horn0"] - Z["top"])


def _bottom_fill(extra=0.0):
    """The blank's bottom is FLAT at the rim plane (the real case is recessed
    1.5 mm around the idler): prints rims-down with no overhang. Every part
    that rides the idler clears the rim plane by 0.3 anyway."""
    x0, x1 = case_xspan(P)
    w = S["body_w"] / 2
    return Pos((x0 + x1) / 2, 0, Z["rim_bot"] / 2) * Box(x1 - x0 + 2 * extra, 2 * w + 2 * extra, -Z["rim_bot"] + 0.001)


def _idler_and_pins(extra=0.0):
    """The bottom-face features the blank omits (idler, its head, housing, pins)."""
    cn = S["conn"]
    d = Pos(0, 0, (Z["idler0"] + Z["idler1"]) / 2 - 0.5) * Cylinder(S["idler_d"] / 2 + extra, S["idler_t"] + 1)
    d += Pos(0, 0, Z["idler0"] / 2 - 0.5) * Cylinder(S["idler_boss_d"] / 2 + extra, -Z["idler0"] + 1)
    d += Pos(0, 0, (Z["idler1"] + Z["idler_head"]) / 2 - 0.5) * Cylinder(S["idler_center_head_d"] / 2 + extra, S["idler_center_head_h"] + 1)
    d += Pos((cn["housing_x"][0] + cn["housing_x"][1]) / 2, 0, Z["housing"] / 2 - 0.5) * \
        Box(cn["housing_x"][1] - cn["housing_x"][0] + 2 * extra, 2 * cn["housing_half_w"] + 2 * extra, -Z["housing"] + 1)
    d += Pos((cn["pins_x"][0] + cn["pins_x"][1]) / 2, 0, Z["pins"] / 2 - 0.5) * \
        Box(cn["pins_x"][1] - cn["pins_x"][0] + 2 * extra, 2 * cn["pins_y"][1] + 2 * extra, -Z["pins"] + 1)
    return d


def servo_blank():
    blank = servo_body(P, clearance=0.0) - _idler_and_pins() + _undercut_fill() + _bottom_fill()
    for hx, hy in horn_screw_positions(P):                    # pilots through the horn + fill
        blank -= Pos(hx, hy, Z["horn1"] - 4) * Cylinder(S["horn_hole_d"] / 2, 12)
    blank -= Pos(0, 0, (STUB_INTO_CASE + Z["rim_bot"]) / 2) * Cylinder((STUB_D + 0.2) / 2, STUB_INTO_CASE - Z["rim_bot"] + 0.01)   # idler stub glue pocket (through the fill)
    x0, x1 = case_xspan(P)
    cx = (x0 + x1) / 2
    for sy in (1, -1):                                        # 3-dot engraving = blank
        for d in range(3):
            blank -= Pos(cx - 6 + 6 * d, sy * (S["body_w"] / 2 - 0.5), S["body_h"] / 2) * \
                Rot(90, 0, 0) * Cylinder(1.5, 1.2)
    return blank


def blank_idler():
    """Local: disc on the bed (z 0..2.1), glue stub up. The idler's centre
    screw head is omitted (declared): a 0.6 mm nub under the disc would be a
    7 mm overhang, and every pocket that rides the idler relieves the head."""
    d = Pos(0, 0, S["idler_t"] / 2) * Cylinder(S["idler_d"] / 2, S["idler_t"])
    stub_h = S["idler_gap"] + STUB_INTO_CASE
    d += Pos(0, 0, S["idler_t"] + stub_h / 2) * Cylinder(STUB_D / 2, stub_h)
    return d


def blank_with_idler():
    """The blank with its idler glued on, posed like servo_body (for dry-fits)."""
    return servo_blank() + Pos(0, 0, Z["idler1"]) * blank_idler()


if __name__ == "__main__":
    import sys
    v = lambda x: 0.0 if x is None else x.volume
    b, idl = servo_blank(), blank_idler()
    export(b, "servo_blank")
    export(idl, "blank_idler")
    fails = []
    # 1: envelope identity — nothing outside the reference solid except the declared fill
    ref = servo_body(P, clearance=0.001) + _undercut_fill(0.001) + _bottom_fill(0.001)
    v_ex = v(b - ref)
    print(f"blank outside reference envelope (+Ø{S['horn_d']} horn fill, + flat bottom): {v_ex:.2f} mm^3 "
          f"({'OK' if v_ex < 1.0 else 'OVERSIZE — would bind'})")
    if v_ex >= 1: fails.append("envelope")
    full = blank_with_idler()
    v_ex2 = v(full - ref)
    print(f"blank + idler outside reference: {v_ex2:.2f} mm^3 ({'OK' if v_ex2 < 1.0 else 'OVERSIZE'})")
    if v_ex2 >= 1: fails.append("idler envelope")
    # 2: sits in all three cradles exactly like the reference, and is captured
    from part_coxa import coxa_yaw_base, coxa_fork
    from part_tibia import tibia_knee_carrier
    from leg_frame import YAW_TF, HIP_TF, KNEE_TF
    base, fork, carrier = coxa_yaw_base(), coxa_fork(), tibia_knee_carrier()
    for name, cradle, posed in (("coxa base cup (yaw pose)", base, YAW_TF * full),
                                ("fork hip cup (hip pose)", fork, HIP_TF * full),
                                ("knee carrier cup", carrier, KNEE_TF * full)):
        fit = v(cradle & posed)
        held = min(v((Pos(0, 2, 0) * posed) & cradle), v((Pos(0, -2, 0) * posed) & cradle),
                   v((Pos(0, 0, 2) * posed) & cradle), v((Pos(0, 0, -2) * posed) & cradle))
        print(f"blank x {name}: {fit:.2f} mm^3 ({'OK' if fit < 1 else 'BINDS'}); "
              f"weakest 2 mm nudge meets {held:.0f} mm^3 ({'held' if held > 1 else 'FREE'})")
        if fit >= 1: fails.append(name)
        if held <= 1: fails.append(name + " retention")
    check_fork = v(fork & (YAW_TF * full))
    print(f"fork rides the blank's horn + idler: {check_fork:.2f} mm^3 ({'OK' if check_fork < 1 else 'BINDS'})")
    if check_fork >= 1: fails.append("fork on blank")
    bb = b.bounding_box()
    print(f"servo_blank: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm — PLA, bottom DOWN, 2 walls, 15%, qty 3 (+3 idlers)")
    print(f"part_servo_blank checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails: sys.exit(1)
