"""Coxa yaw module, v0.4 (D047, 2026-09-22) — TWO printed parts.

Frame: yaw axis = +Z at origin (leg_frame). The yaw servo sits SHAFT-DOWN:
horn toward the deck, rear idler + cable plugs facing up.

  coxa_yaw_base : deck-mounted bracket = I1 leg-port plate + a servo CUP
                  (servo_mount) around the rear of the yaw servo. The servo
                  slides in from the front, four self-tappers into its rim
                  holes, the cup walls take the torque reaction. Nothing sits
                  over the axis: the fork can be installed on the servo first.
  coxa_fork     : ONE C-shaped part riding BOTH sides of the yaw servo —
                  lower hub bolted to the horn (D063, B81: the horn face
                  bears on the FLAT hub top, its centre head sits in a Ø6.4
                  relief, and four slotted M3 through-holes, heads
                  counterbored from below, locate it radially, as at the
                  couplers), a vertical web in front of the servo, and an
                  upper plate whose pocket rides the rear IDLER disc. The
                  overturning couple is reacted horn <-> idler (36 mm arm)
                  instead of horn <-> a 683ZZ in a bolt-on cap 15 mm away
                  (D046, J1: computed at ~240 N on the bearing in a recovery
                  push). The upper plate continues into the HIP servo's cup.
                  The idler pocket and the horn head's relief run out
                  through the C's mouth (-X) as channels (D063, B80): the
                  C is 34.9 between its jaws, the servo 38.0 head to head,
                  so without them the fork could not go on at all. That
                  leaves the couple one-sided: when the load tips the upper
                  jaw toward the mouth (an inward push on the foot) the idler
                  bears on nothing, so two side cheeks outside the servo's
                  swept case tie the jaws to the web (D063 review).

Vertical stack (leg z): plate -4..0 | hub 1.3..7.3 = horn top | servo case
11.6..40.4 | idler face 43.7 | upper plate 42.2..48.2 | hip servo 48.6.. |
hip axis 61 (params leg.hip_axis_z).

Assembly order (every path and final fit below is asserted here and in
check_assembly): fork onto the yaw servo, off the base: it slides on from
the servo's front (moving -X; the web closes +X), the horn's centre head and
the idler running in their channels, and stays free toward the mouth until
4x M3 x 6 go in from below (hold it seated while driving them) -> servo +
fork slide into the base cup from +X -> 2 rim screws from
above (idler face) + 2 from below the plate (horn face, deep pockets) -> hip
servo slides into the fork's cup from +X -> 4 rim screws -> module hooks
onto the deck (I1).

Leg harness (D047 follow-up): the deck's frozen I1 cable pass-through (leg x
-29) sits UNDER the yaw servo, and the D047 base covered it. The base now
carries an 11 x 11 channel (harness_path): up out of the cutout and forward
under the gearbox plateau, sideways to -Y under the cup's side wall, up
beside the cup, over the top to the yaw servo's plugs. check_assembly
asserts the path clear of every part at every yaw.
"""
from build123d import *
from common import params, export
from iface import CABLE_CUTOUT_X, CABLE_CUTOUT_W, PLATE_X0, leg_port_socket_walls
from servo_st3215 import servo_body, plug_envelope, horn_slot_cutter, horn_screw_angles, spec, z_levels
from servo_mount import servo_cup, cup_extents, cup_screw_columns, FRONT_X, T as CUP_T
from leg_frame import (YAW_TF, HIP_TF, Z_HIP, HUB_Z0, HUB_Z1, ZC_YAW,
                       Z_YAW_IDLER_FACE, Z_YAW_RIM_TOP, Z_CUP_FLOOR_TOP, FIT, yaw_z)

P = params()
S = spec(P)
Z = z_levels(P)
PR = P["print"]
IF = P["interfaces"]["leg_port"]

HUB_D = 26.0
# B81 (D063): no horn pocket. The hub top IS the horn face plane (leg_frame.HUB_Z1), so a
# pocket cut down from it was 1.0 of air the horn never entered, and the screwed hub rode
# up 0.3 onto the centre head. Now the face bears flat on the hub top, the centre head
# clears its relief floor by FIT, and the four radial slots locate the hub.
HORN_HEAD_Z0 = HUB_Z1 - (S["horn_center_head_h"] + FIT)   # 6.0: the centre-head relief floor
HORN_HEAD_W = S["horn_center_head_d"] + 1.0               # 6.4: the relief and its channel
CB_DEPTH = 3.0                       # head counterbores from the hub underside
WEB_X0, WEB_X1 = 18.0, 22.0          # vertical web in front of the yaw servo (case front at +10.2)
WEB_HALF_W = 13.0
# The C's side cheeks (D063 review). With the idler pocket open to the mouth (B80), an inward
# push on the foot (fem_check R-) tips the upper jaw toward the mouth, where the idler bears on
# nothing: the fork hangs off the horn face alone, and the 4 mm web alone gave SF 0.49 (22.9 mm
# at the hip cup). Two cheeks, x CHEEK_X0..WEB_X1, and the web and both jaws run out to them:
# R- is SF 2.47 (1.8 mm), for +8.1 cm^3 (+5.5 g a fork).
CHEEK_X0 = 2.0                        # from 0, the cheek's rear corner came 1.4 from the base cup at yaw 40
CHEEK_Y0 = 16.6                       # the servo's front case corners (r 16.06) pass >= 0.67 inside it
CHEEK_Y1 = CHEEK_Y0 + 3.0             # 19.6
UP_Z0 = Z_YAW_RIM_TOP + FIT           # 42.2: upper plate underside clears the bottom rim
UP_T = 6.0
UP_Z1 = UP_Z0 + UP_T                  # 48.2
IDLER_POCKET_D = S["idler_d"] + FIT + 0.1
IDLER_POCKET_Z1 = Z_YAW_IDLER_FACE + FIT   # 44.0
IDLER_HEAD_W = S["idler_center_head_d"] + 1.0
IDLER_HEAD_Z1 = yaw_z(Z["idler_head"]) + FIT   # 44.6
MOUTH_X = -40.0                       # the channels run out through the C's mouth, past everything
FIN_X = -6.3                          # behind this the Ø26 plate edge left the idler channel's walls < 1.2
                                      # thick (knife edges to 0 at x -8.1): they are cut square here
NOTCH_X = -9.5                        # upper hub cut flat here on the -X side: the cable plugs live behind
NOTCH_HALF_W = 10.3
BACK_HALF_W = 9.5                     # base cup back wall width: I1 thumbscrew heads at |y| >= 11
REAR_TRIM_X = -33.9                   # nothing wider than BACK_HALF_W behind this (thumbscrew heads end at x -34)
# The I1 spine rib (2026-10-07, decision 15, B119): in stance the ground on the leg's foot lifts the plate's
# outboard end, the plate pivots on its inboard pads and the two thumbscrews (x -41) hold
# the whole peel moment, 4629 N mm in the design case (3 legs x 3, mu 0.5). Across the bare
# 4 mm plate at the screw line that was 47.4 MPa, SF 0.74 (J/j3, measured on the solid).
# A rib on the plate's centre line, plate top to 6 up, from the plate's inboard edge to
# the cup's back wall, takes it to 15.0 MPa, SF 2.33; part_coxa's __main__ re-measures it.
I1_RIB_X0 = PLATE_X0                  # the plate's inboard edge (-46): inboard of it the leg-4 Pi corner overhangs
I1_RIB_HALF_W = 8.0                   # |y| <= 8: 3 mm inside the thumbscrew knobs (|y| >= 11)
I1_RIB_H = 6.0                        # z 0..6 over the plate top; under the cup's floor rails (6.8)
I1_PLATE_W = 44.0                     # the plate's y +-22 edges: the seat sockets' mouths leave 1.5 / 1.20 to them

_e = cup_extents()

# leg harness route (see harness_path): -Y side, because the hip cup and the
# hip servo's plugs sweep the +Y side at yaw +40. x -24..-13: the front face
# is the fork's lower-hub radius (never inside the hub at any yaw). Its rear
# face is no longer bound by the port: since 2026-10-07 the nearest I1 feature
# is the -y seat socket at (-32, -17.3), whose Ø6.4 mouth ends at x -28.8, 4.8
# behind it (the D020 dowel bore at (-28, -19) left 1.85). The spine rib runs
# to x -38.0, fused 0.5 into the cup's back wall (outer face x -38.5), and ends
# 3.5 behind the channel's first leg (x -34.5).
HARNESS_W = CABLE_CUTOUT_W                   # 11: the XT30 + JST-XH-5 pair's clear section
HARNESS_X1 = -HUB_D / 2                      # -13
HARNESS_X0 = HARNESS_X1 - HARNESS_W          # -24
HARNESS_Z0 = -4.0                            # the deck top (the plate's underside)
HARNESS_Z1 = HARNESS_Z0 + HARNESS_W          # 7: under the plateau fill (8.7), rails at 6.8 bridge it


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def harness_path():
    """The leg drop's route in the leg frame, deck cutout -> yaw servo plugs:
    [(name, (x0, x1, y0, y1, z0, z1), travel axis)]. Every box has at least
    an 11 x 11 section across its travel axis and overlaps the next one."""
    h = HARNESS_W / 2
    y_riser1 = -(_e["y_out"] + 0.2)                  # -15.9: just outside the cup's side wall
    y_riser0 = y_riser1 - HARNESS_W                  # -26.9
    pe = (YAW_TF * plug_envelope(P)).bounding_box()  # the yaw plugs + their wire bend
    z_top0, z_top1 = pe.max.Z, pe.max.Z + HARNESS_W  # 57.4 .. 68.4
    return [
        ("up out of the deck cutout, forward under the plateau",
         (CABLE_CUTOUT_X - h, HARNESS_X1, -h, h, HARNESS_Z0, HARNESS_Z1), "x"),
        ("sideways under the cup's -Y side wall",
         (HARNESS_X0, HARNESS_X1, y_riser0, h, HARNESS_Z0, HARNESS_Z1), "y"),
        ("up beside the cup",
         (HARNESS_X0, HARNESS_X1, y_riser0, y_riser1, HARNESS_Z0, z_top1), "z"),
        ("over the top to the yaw plugs",
         (HARNESS_X0, HARNESS_X1, y_riser0, pe.max.Y, z_top0, z_top1), "y"),
    ]


def harness_solid():
    """Union of the harness_path boxes (the cable's keep-out)."""
    out = None
    for _, b, _ in harness_path():
        out = _box(*b) if out is None else out + _box(*b)
    return out


# ---------------------------------------------------------------- base
def yaw_servo_placed(clearance=0.0):
    return YAW_TF * servo_body(P, clearance)


def i1_rib():
    """The I1 spine rib (B119): plate top to I1_RIB_H, x I1_RIB_X0 to the cup's back wall (fused 0.5 into it)."""
    return _box(I1_RIB_X0, _e["x_back_out"] + 0.5, -I1_RIB_HALF_W, I1_RIB_HALF_W, -0.01, I1_RIB_H)


def coxa_yaw_base():
    from iface import leg_port_plate_features
    # x PLATE_X0..20: iface's plate extension runs inboard of PLATE_X0 to the hook's stem,
    # so the plate's inboard edge is that one constant, not a literal here
    plate = Pos((PLATE_X0 + 20) / 2, 0, -2) * Box(20 - PLATE_X0, I1_PLATE_W, 4)
    for hx, hy in [(15, 17), (15, -17)]:                     # spare pair only
        plate -= Pos(hx, hy, -2) * Cylinder(PR["screw_m3_clear"] / 2, 6)
    plate = leg_port_plate_features(plate, plate_top_z=0.0, plate_bot_z=-4.0)
    cup = YAW_TF * servo_cup(back_half_w=BACK_HALF_W)
    # fill between the plate and the cup's floor rails / plateau channel
    z_rail_bot = yaw_z(_e["z_hi_out"])                       # 6.8
    z_plateau_floor = yaw_z(Z["plateau"]) - FIT              # 8.7
    fill = _box(_e["x_back_out"], FRONT_X, -_e["y_out"], _e["y_out"], 0.0, z_rail_bot + 0.01)
    fill += _box(_e["x_back_out"], FRONT_X, -_e["y_top_rail"], _e["y_top_rail"], 0.0, z_plateau_floor)
    base = plate + cup + fill
    base += i1_rib()                                         # B119
    # the I1 thumbscrew heads (|y| >= 11, x <= -35) own the space behind REAR_TRIM_X
    for sy in (1, -1):
        base -= _box(-60, REAR_TRIM_X, min(sy * BACK_HALF_W, sy * 30), max(sy * BACK_HALF_W, sy * 30), 0.0, 60)
    # horn-face rim screws from BELOW the plate: Ø5 pockets up to 3 mm under the rail top
    for x, y in [(x, -y) for x, y in __import__("servo_st3215").case_screw_positions("top", P) if x < FRONT_X]:
        base -= Pos(x, y, (-5 + yaw_z(_e["z_hi_out"]) + CUP_T - 3) / 2) * \
            Cylinder(5.0 / 2, yaw_z(_e["z_hi_out"]) + CUP_T - 3 + 5)
        base -= Pos(x, y, 5) * Cylinder(S["case_screw"]["clear_d"] / 2, 30)
    base -= harness_solid()                                  # the leg drop's channel
    return base


# ---------------------------------------------------------------- fork
def hip_servo_placed(clearance=0.0):
    return HIP_TF * servo_body(P, clearance)


def _to_mouth(d, z0, z1):
    """A Ø d pocket on the yaw axis, z0..z1, run out through the C's mouth
    (-X) at its full width: the path a servo feature takes as the fork goes on."""
    return Pos(0, 0, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0) + _box(MOUTH_X, 0, -d / 2, d / 2, z0, z1)


def coxa_fork():
    # lower hub on the yaw horn (horn faces down; hub sits under it)
    hub = Pos(0, 0, (HUB_Z0 + HUB_Z1) / 2) * Cylinder(HUB_D / 2, HUB_Z1 - HUB_Z0)
    arm = _box(0, WEB_X1, -WEB_HALF_W, WEB_HALF_W, HUB_Z0, HUB_Z1)
    web = _box(WEB_X0, WEB_X1, -WEB_HALF_W, WEB_HALF_W, HUB_Z0, UP_Z1)
    up = _box(0, WEB_X1, -WEB_HALF_W, WEB_HALF_W, UP_Z0, UP_Z1)
    up += Pos(0, 0, (UP_Z0 + UP_Z1) / 2) * Cylinder(HUB_D / 2, UP_T)
    cup = HIP_TF * servo_cup()
    fork = hub + arm + web + up + cup
    for sy in (1, -1):                    # the side cheeks: a block minus the window the servo sweeps
        ya, yb = sorted((sy * WEB_HALF_W, sy * CHEEK_Y1))
        side = _box(CHEEK_X0, WEB_X1, ya, yb, HUB_Z0, UP_Z1)
        ya, yb = sorted((sy * (WEB_HALF_W - 1), sy * CHEEK_Y0))
        fork += side - _box(CHEEK_X0 - 1, WEB_X0, ya, yb, HUB_Z1, UP_Z0)
    # The fork's only way on is from the servo's front, moving -X (the web closes +X), and
    # the servo is 38.0 head to head against a 34.9 C (B80): the horn's centre head (1.0
    # under the hub top) and the idler (1.5 into the upper plate) each run in a channel
    # out through the mouth. So the fork is free toward the mouth until the four horn
    # screws are in (their radial slots at 45 deg take +X in shear). The standing load
    # (foot pushed up) and an outward push tip the upper plate -X, onto the idler pocket's
    # kept +X wall; the channel's side walls take the lateral cases. An inward push past
    # 0.6x the vertical load (or a foot pulled down) loads the open side: then the clamped
    # horn face alone holds the fork and the cheeks carry the upper jaw down to it.
    # --- yaw horn interface (from below): flat seat (B81), centre-head relief + channel, slots + counterbores
    # (the channel passes 0.05 from the two rear slots: over its 1.3 depth they print as one
    # opening, harmless, as the heads seat 3 lower and the shanks keep 1.7 of slot wall)
    fork -= _to_mouth(HORN_HEAD_W, HORN_HEAD_Z0, HUB_Z1 + 1)
    fork -= horn_slot_cutter(HUB_Z0 - 1, HUB_Z1 + 1, p=P)
    fork -= horn_slot_cutter(HUB_Z0 - 1, HUB_Z0 + CB_DEPTH,
                             extra=S["horn_screw_head_d"] - S["horn_screw_clear_d"], p=P)
    # --- yaw idler interface (from above): OD pocket + centre-head relief, both open to the mouth
    fork -= _to_mouth(IDLER_POCKET_D, UP_Z0 - 1, IDLER_POCKET_Z1)
    fork -= _to_mouth(IDLER_HEAD_W, UP_Z0 - 1, IDLER_HEAD_Z1)
    fork -= _box(MOUTH_X, FIN_X, -HUB_D / 2 - 1, HUB_D / 2 + 1, UP_Z0 - 1, IDLER_POCKET_Z1)
    fork -= _box(-40, NOTCH_X, -NOTCH_HALF_W, NOTCH_HALF_W, UP_Z0 - 1, UP_Z1 + 1)
    return fork


FORK_SLIDE_DX = [0.5 * k for k in range(1, 61)]   # 0.5..30: past 23 the fork is wholly in front of the servo


def fork_slide_worst(fork, target, dxs=FORK_SLIDE_DX):
    """Worst overlap (mm^3) of the fork slid back +dx along its only path
    against a posed yaw servo or blank. Only the fork inside the target's box
    (grown 1 mm, stretched back along the travel) can meet it: clip first."""
    bb = target.bounding_box()
    zone = _box(bb.min.X - max(dxs) - 1, bb.max.X + 1, bb.min.Y - 1, bb.max.Y + 1, bb.min.Z - 1, bb.max.Z + 1)
    near = fork & zone
    return max(_v((Pos(dx, 0, 0) * near) & target) for dx in dxs)


def horn_screw_shanks():
    """The four M3 horn-screw shanks as driven (nominal radius, leg frame):
    head on the counterbore floor, through the hub's slot band and the horn.
    They carry the fork's +X (the open mouth) once they are in."""
    z0, z1 = HUB_Z0 + CB_DEPTH, HUB_Z1 + S["horn_t"]
    out = None
    for ang in horn_screw_angles(P):
        s = Rot(0, 0, ang) * Pos(S["horn_bcd"] / 2, 0, (z0 + z1) / 2) * \
            Cylinder(S["horn_screw_m"] / 2, z1 - z0)
        out = s if out is None else out + s
    return out


def _v(x):
    return 0.0 if x is None else x.volume


I1_PEEL_X = IF["thumbscrew_xy"][0][0]   # -41: the thumbscrew line
I1_PEEL_M = 4629.0                      # the stance design moment there (N mm, J/j3: V 26.76 N
                                        # at x 75 + H 13.38 N at 114 up, 3 legs x 3, mu 0.5)


def plate_section(base, x, dx=0.02):
    """The base's bending section at x (a dx slab of the real solid): (area mm^2, centroid z,
    second moment about its horizontal centroidal axis mm^4, z min, z max)."""
    sl = base & Pos(x, 0, 0) * Box(dx, 200, 200)
    a = sl.volume / dx
    i = sl.matrix_of_inertia[1][1] / dx - a * dx * dx / 12        # about y, through the centroid
    bb = sl.bounding_box()
    return a, sl.center(CenterOf.MASS).Z, i, bb.min.Z, bb.max.Z


if __name__ == "__main__":
    import sys
    base = coxa_yaw_base()
    fork = coxa_fork()
    export(base, "coxa_yaw_base")
    export(fork, "coxa_fork")
    yaw_s, hip_s = yaw_servo_placed(), hip_servo_placed()
    print(f"hub {HUB_Z0:.1f}..{HUB_Z1:.1f}  ZC_YAW={ZC_YAW:.1f}  upper plate {UP_Z0:.1f}..{UP_Z1:.1f}  "
          f"hip cup floor top {Z_CUP_FLOOR_TOP:.1f}  Z_HIP={Z_HIP:.1f}")
    fails = []
    def check(name, v, good, msg_ok="OK", msg_bad="FAIL"):
        ok = good(v)
        print(f"{name}: {v:.2f} mm^3 ({msg_ok if ok else msg_bad})")
        if not ok: fails.append(name)
    worst = 0.0
    for dx, dy in IF["thumbscrew_xy"]:
        probe = Pos(dx, dy, 30) * Cylinder(IF["thumbscrew_head_d"] / 2 + 1, 56)
        worst = max(worst, _v(probe & base), _v(probe & fork))
    check("I1 thumbscrew access columns (base + fork)", worst, lambda v: v < 1, "CLEAR", "BLOCKED")
    check("base x yaw servo", _v(base & yaw_s), lambda v: v < 1, "OK", "CLASH")
    check("fork x yaw servo (horn face on the hub top, idler in its pocket)", _v(fork & yaw_s),
          lambda v: v < 1, "OK", "CLASH")
    check("fork x hip servo", _v(fork & hip_s), lambda v: v < 1, "OK", "CLASH")
    # the fork turns on the horn against the fixed case: its cheeks must clear the swept case
    worst = max(_v((Rot(0, 0, yaw) * fork) & yaw_s) for yaw in range(-40, 41, 5))
    check("fork (side cheeks) x yaw servo over yaw -40..40", worst, lambda v: v < 1, "CLEAR", "CLASH")
    check("base x hip servo", _v(base & hip_s), lambda v: v < 1, "OK", "CLASH")
    worst = 0.0
    for yaw in range(-40, 41, 10):
        r = Rot(0, 0, yaw)
        worst = max(worst, _v((r * fork) & base), _v((r * hip_s) & base))
    check("base x (fork + hip servo) over yaw -40..40", worst, lambda v: v < 1, "OK", "CLASH")
    check("fork x yaw plug keep-out", _v(fork & (YAW_TF * plug_envelope(P))), lambda v: v < 1, "CLEAR", "BLOCKS PLUGS")
    check("base x yaw plug keep-out", _v(base & (YAW_TF * plug_envelope(P))), lambda v: v < 1, "CLEAR", "BLOCKS PLUGS")
    harness = harness_solid()
    check("base x leg harness path", _v(base & harness), lambda v: v < 1, "CLEAR", "BLOCKED")
    # I1 spine rib (B119): it must leave the channel whole (the channel is cut last, so a
    # rib in its way would be notched silently), and lift the plate at the screw line
    check("I1 spine rib x leg harness path (before the channel is cut)", _v(i1_rib() & harness),
          lambda v: v < 1e-3, "CLEAR", "IN THE CHANNEL")
    a, zc, i, z0, z1 = plate_section(base, I1_PEEL_X)
    sig = I1_PEEL_M * max(z1 - zc, zc - z0) / i
    fem = P["fem"]
    sf = fem[fem["material"]]["strength_xy_mpa"] / sig
    ok = sf >= fem["sf_target"]
    print(f"I1 plate + rib at the screw line x {I1_PEEL_X:.0f}: A {a:.1f} mm^2, I {i:.0f} mm^4, z {z0:.1f}..{z1:.1f}; "
          f"{I1_PEEL_M:.0f} N mm -> {sig:.1f} MPa, SF {sf:.2f} ({'OK' if ok else 'WEAK'}: bare plate 47.4, SF 0.74)")
    if not ok: fails.append("I1 plate section at the screw line")
    # the seat sockets' mouth walls to the plate's y edges: a NOTE, not a failure. 1.20 at the vee
    # is the proven seat_fit-0 value (I1_DOCK_OPTIONS 7); filing seat_fit takes it straight off
    wc, wv = leg_port_socket_walls(I1_PLATE_W / 2, PR["seat_fit"])
    print(f"I1 seat socket mouths to the plate's y edges (seat_fit {PR['seat_fit']}): cone {wc:.2f}, vee {wv:.2f}"
          + (" NOTE: under 1.20; I1_DOCK_OPTIONS 7: mouth 0.2 or the seats at |y| 17.1" if wv < 1.2 - 1e-6 else ""))
    worst = max(_v((Rot(0, 0, yaw) * (fork + hip_s)) & harness) for yaw in range(-40, 41, 10))
    check("leg harness path x (fork + hip servo) over yaw -40..40", worst, lambda v: v < 1, "CLEAR", "BLOCKED")
    check("fork x hip plug keep-out", _v(fork & (HIP_TF * plug_envelope(P))), lambda v: v < 1, "CLEAR", "BLOCKS PLUGS")
    # assembly paths, in build order (B80: the first one was never checked and was blocked)
    check("fork slide-on path (-X) onto the yaw servo, dx 0.5..30", fork_slide_worst(fork, yaw_s),
          lambda v: v < 1, "OPEN", "BLOCKED")
    worst = max(_v((Pos(dx, 0, 0) * (fork + yaw_s)) & base) for dx in (1, 2, 5, 10, 20, 40))
    check("servo + fork slide-in path (+X) vs base", worst, lambda v: v < 1, "OPEN", "BLOCKED")
    worst = max(_v((Pos(dx, 0, 0) * hip_s) & (fork + base)) for dx in (1, 2, 5, 10, 20, 40))
    check("hip servo slide-in path (+X) vs fork + base", worst, lambda v: v < 1, "OPEN", "BLOCKED")
    # capture: the idler pocket's kept walls locate -X and +-Y; +X, toward the open mouth, is
    # the BOLTED direction (the horn meets nothing in any nudge: it bears on a flat face, B81)
    check("fork nudged -X x yaw servo (idler pocket's +X wall locates)", _v((Pos(-2, 0, 0) * fork) & yaw_s),
          lambda v: v > 1, "LOCATED", "LOOSE")
    check("fork nudged +Y x yaw servo (idler pocket's side walls locate)", _v((Pos(0, 2, 0) * fork) & yaw_s),
          lambda v: v > 1, "LOCATED", "LOOSE")
    check("fork nudged +X x the 4 horn screws (bolted direction)",
          _v((Pos(2, 0, 0) * fork) & horn_screw_shanks()), lambda v: v > 1, "BOLTED", "LOOSE")
    # rim-screw driver columns: idler-face screws from above vs the fork at every yaw
    worst = 0.0
    for col in cup_screw_columns("bot"):
        c = YAW_TF * col
        for yaw in range(-40, 41, 20):
            worst = max(worst, _v(c & (Rot(0, 0, yaw) * fork)))
    check("base idler-face rim screws (from above) vs fork over yaw", worst, lambda v: v < 1, "CLEAR", "BLOCKED")
    print(f"part_coxa checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails: sys.exit(1)
