"""Femur link, v0.3 (D047) — a twin-plate link that rides BOTH sides of the
hip and knee servos.

  femur_link    : plate A (horn side, -Y) with the two coupler recesses
                  (part_coupler) + a bridge of two walls between the servos
                  carrying four bosses for plate B. One print, recess face up.
  femur_plate_b : plate B (idler side, +Y): two towers whose pockets ride the
                  hip and knee IDLER discs, joined by two rails outside the
                  cable-plug zone, bolted to the bridge with 4x M3.

v0.2 (D046) was a single plate on the horn side; all leg bending then went
through one servo output shaft per joint. With the idler carried, each joint
is a couple across the case (~37 mm), which is how every twin-plate hexapod
kit does it. The plates never touch a horn: hip/knee torque goes horn ->
coupler lobes -> plate A; plate B only locates.

Link-local frame = leg frame shifted to the hip axis: (x, y, z)_link =
(x - L1, y, z - Z_HIP). Hub A at the origin, hub B at (+L2, 0). Plate A
mating face at y = Y_HORN_TOP - DISC_T; plate B pocket face at the idler.
"""
from build123d import *
from common import params, export
from part_coupler import (coupler_recess, coupler_on_face, DISC_T, RECESS_CLAMP_ANGS,
                          TAP_R_POS, POCKET_DEPTH, CLAMP_BORE_DEPTH)
from servo_st3215 import spec, z_levels, plug_envelope
from leg_frame import L1, L2, Z_HIP, YB, Y_HORN_TOP, Y_IDLER_FACE, FIT, HIP_TF, KNEE_TF, KNEE_X

P = params()
S = spec(P)
Z = z_levels(P)
PR = P["print"]
T = 6.0                               # plate A thickness
HUB_A_D = 30.0                        # plate A hub: 2.3 mm of wall outside the Ø25.4 coupler pocket
TOWER_D = 26.0                        # plate B towers: 2.9 mm outside the Ø20.2 idler pocket
BEAM_H = 24.0
YA1 = Y_HORN_TOP - DISC_T             # -14.1 plate A mating face (leg y)
YA0 = YA1 - T                         # -20.1 plate A outer face
# plate B: towers ride the idler; rails sit above the cups' idler-face bars
YB0 = YB + S["rim"]["h"] + FIT         # 23.8 tower pocket face (clears the case rim)
POCKET_Y1 = Y_IDLER_FACE + FIT         # 25.6
HEAD_Y1 = YB - Z["idler_head"] + FIT   # 26.2
RAIL_Y0 = 27.6                         # above the cup's idler-face bar (26.8) + 0.8
RAIL_T = 6.0
YB1 = RAIL_Y0 + RAIL_T                 # 33.6 plate B outer face (flat: prints on it)
RAIL_Z = (11.0, 19.0)                  # |z| band: outside the plug keep-out (|z| <= 9.45)
NOTCH_X = -9.5                         # tower cut flat behind this (plugs), pocket open toward -X
NOTCH_HALF_W = 10.3
BRIDGE_X = (21.0, 52.0)                # link x: clears the hip case sweep (r 17) and the tibia cup sweep (r 42)
WALL_Z = (10.0, 13.0)                  # |z| band of the two bridge walls
BOSS_D, BOSS_H = 8.0, 8.0
BOSS_X = (25.0, 48.0)
BOSS_ZC = 11.5


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _cyl_y(cx, cz, y0, y1, r):
    return Pos(cx, (y0 + y1) / 2, cz) * Rot(90, 0, 0) * Cylinder(r, y1 - y0)


def _flat_plate_a():
    """Plate A in a flat frame: XY plane, thickness along +z (z 0..T), mating
    face z = T. flat (x, y, z) -> link (x, z + YA0, -y)."""
    beam = Pos(L2 / 2, 0, T / 2) * Box(L2, BEAM_H, T)
    hubs = None
    for cx in (0.0, L2):
        h = Pos(cx, 0, T / 2) * Cylinder(HUB_A_D / 2, T)
        hubs = h if hubs is None else hubs + h
    part = beam + hubs
    for cx in (L2 * 0.32, L2 * 0.68):                      # lightening slots
        part -= Pos(cx, 0, T / 2) * Box(L2 * 0.2, BEAM_H * 0.45, T + 2)
    for cx in (0.0, L2):                                   # coupler recess per hub
        part = Pos(cx, 0, 0) * coupler_recess(Pos(-cx, 0, 0) * part, face_z=T, clamp_len=T)
    return part


FLAT_TO_LINK = Pos(0, YA0, 0) * Rot(-90, 0, 0)


def femur_link():
    link = FLAT_TO_LINK * _flat_plate_a()
    # bridge walls from plate A's mating face up to the rail underside
    for sz in (1, -1):
        z0, z1 = sorted((sz * WALL_Z[0], sz * WALL_Z[1]))
        link += _box(BRIDGE_X[0], BRIDGE_X[1], YA1 - 0.01, RAIL_Y0, z0, z1)
        for bx in BOSS_X:
            link += _cyl_y(bx, sz * BOSS_ZC, RAIL_Y0 - BOSS_H, RAIL_Y0, BOSS_D / 2)
            link -= _cyl_y(bx, sz * BOSS_ZC, RAIL_Y0 - BOSS_H + 0.8, RAIL_Y0 + 1, PR["screw_m3_tap"] / 2)
    return link


def femur_plate_b():
    b = None
    for sz in (1, -1):
        z0, z1 = sorted((sz * RAIL_Z[0], sz * RAIL_Z[1]))
        r = _box(-TOWER_D / 2, L2 + TOWER_D / 2, RAIL_Y0, YB1, z0, z1)
        b = r if b is None else b + r
    for cx in (0.0, L2):
        b += _cyl_y(cx, 0, YB0, YB1, TOWER_D / 2)
        b -= _cyl_y(cx, 0, YB0 - 1, POCKET_Y1, (S["idler_d"] + FIT + 0.1) / 2)
        b -= _cyl_y(cx, 0, YB0 - 1, HEAD_Y1, (S["idler_center_head_d"] + 1.0) / 2)
        b -= _box(cx - 40, cx + NOTCH_X, YB0 - 1, YB1 + 1, -NOTCH_HALF_W, NOTCH_HALF_W)   # plugs: full height
    for bx in BOSS_X:
        for sz in (1, -1):
            b -= _cyl_y(bx, sz * BOSS_ZC, RAIL_Y0 - 1, YB1 + 1, PR["screw_m3_clear"] / 2)
    return b


def hub_face_tf(which="A"):
    """Transform whose z=0 plane is hub A/B's mating (+Y) face, +z toward the
    horn — feed to part_coupler.coupler_on_face. Link-local coords."""
    cx = 0.0 if which == "A" else L2
    return Pos(cx, YA1, 0) * Rot(-90, 0, 0)


LINK_TF = Pos(L1, 0, Z_HIP)               # link-local -> leg


if __name__ == "__main__":
    import sys
    v = lambda x: 0.0 if x is None else x.volume
    link, plate_b = femur_link(), femur_plate_b()
    export(link, "femur_link")
    export(plate_b, "femur_plate_b")
    fails = []
    def check(name, val, good, ok="OK", bad="FAIL"):
        okk = good(val)
        print(f"{name}: {val:.2f} mm^3 ({ok if okk else bad})")
        if not okk: fails.append(name)
    for which in ("A", "B"):
        c = coupler_on_face(hub_face_tf(which))
        check(f"hub {which}: coupler fit", v(c & link), lambda q: q < 1, "FITS", "INTERFERES")
        check(f"hub {which}: capture x", v((Pos(1.0, 0, 0) * c) & link), lambda q: q > 1, "LOCATES", "LOOSE")
        check(f"hub {which}: capture z", v((Pos(0, 0, 1.0) * c) & link), lambda q: q > 1, "LOCATES", "LOOSE")
        cx = 0.0 if which == "A" else L2
        worst = 0.0
        for ang in RECESS_CLAMP_ANGS:
            Lp = 2 + T + (CLAMP_BORE_DEPTH - (POCKET_DEPTH - 0.4)) - 0.3
            pin = Pos(cx, YA0, 0) * Rot(-90, 0, 0) * Rot(0, 0, ang) * \
                Pos(TAP_R_POS, 0, -2 + Lp / 2) * Cylinder(PR["screw_m3_tap"] / 2 - 0.05, Lp)
            worst = max(worst, v(pin & link), v(pin & c))
        check(f"hub {which}: clamp bores coaxial", worst, lambda q: q < 1, "COAXIAL", "MISALIGNED")
    # plate B on the servos' idlers (link frame: servos posed then shifted)
    from servo_st3215 import servo_body
    inv = Pos(-L1, 0, -Z_HIP)
    hip_s, knee_s = inv * HIP_TF * servo_body(P), inv * KNEE_TF * servo_body(P)
    check("plate B x hip servo", v(plate_b & hip_s), lambda q: q < 1, "OK", "CLASH")
    check("plate B x knee servo", v(plate_b & knee_s), lambda q: q < 1, "OK", "CLASH")
    check("plate B nudged +Z x hip idler (pocket locates)", v((Pos(0, 0, 2) * plate_b) & hip_s), lambda q: q > 1, "LOCATED", "LOOSE")
    check("plate B nudged +X x hip idler", v((Pos(2, 0, 0) * plate_b) & hip_s), lambda q: q > 1, "LOCATED", "LOOSE")
    check("plate B x hip plug keep-out", v(plate_b & (inv * HIP_TF * plug_envelope(P))), lambda q: q < 1, "CLEAR", "BLOCKS PLUGS")
    check("plate B x knee plug keep-out", v(plate_b & (inv * KNEE_TF * plug_envelope(P))), lambda q: q < 1, "CLEAR", "BLOCKS PLUGS")
    check("link x hip servo", v(link & hip_s), lambda q: q < 1, "OK", "CLASH")
    check("link x knee servo", v(link & knee_s), lambda q: q < 1, "OK", "CLASH")
    check("link x plate B (touch at the bosses only)", v(link & plate_b), lambda q: q < 1, "OK", "CLASH")
    # plate B screw path: Ø2.8 pin through plate B into the boss bore
    worst = 0.0
    for bx in BOSS_X:
        for sz in (1, -1):
            pin = _cyl_y(bx, sz * BOSS_ZC, RAIL_Y0 - BOSS_H + 1.0, YB1 + 2, PR["screw_m3_tap"] / 2 - 0.05)
            worst = max(worst, v(pin & link), v(pin & plate_b))
    check("plate B screws coaxial with the bridge bosses", worst, lambda q: q < 1, "COAXIAL", "MISALIGNED")
    # plate B drop-on path (+Y) with the servos in place
    worst = max(v((Pos(0, dy, 0) * plate_b) & (hip_s + knee_s + link)) for dy in (1, 2, 5, 10, 20))
    check("plate B drop-on path (+Y)", worst, lambda q: q < 1, "OPEN", "BLOCKED")
    bb = link.bounding_box(); bb2 = plate_b.bounding_box()
    print(f"femur_link {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f}; plate B {bb2.size.X:.1f} x {bb2.size.Y:.1f} x {bb2.size.Z:.1f}")
    print(f"part_femur checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails: sys.exit(1)
