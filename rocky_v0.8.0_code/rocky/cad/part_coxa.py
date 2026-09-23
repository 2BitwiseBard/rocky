"""Coxa yaw module, v0.4 (D047, 2026-09-22) — TWO printed parts.

Frame: yaw axis = +Z at origin (leg_frame). The yaw servo sits SHAFT-DOWN:
horn toward the deck, rear idler + cable plugs facing up.

  coxa_yaw_base : deck-mounted bracket = I1 leg-port plate + a servo CUP
                  (servo_mount) around the rear of the yaw servo. The servo
                  slides in from the front, four self-tappers into its rim
                  holes, the cup walls take the torque reaction. Nothing sits
                  over the axis: the fork can be installed on the servo first.
  coxa_fork     : ONE C-shaped part riding BOTH sides of the yaw servo —
                  lower hub bolted to the horn (Ø20.3 pocket on the horn OD,
                  four slotted M3 through-holes, heads counterbored from
                  below), a vertical web in front of the servo, and an upper
                  plate whose pocket rides the rear IDLER disc. The overturning
                  couple is reacted horn <-> idler (36 mm arm) instead of
                  horn <-> a 683ZZ in a bolt-on cap 15 mm away (D046, J1:
                  computed at ~240 N on the bearing in a recovery push).
                  The upper plate continues into the HIP servo's cup.

Vertical stack (leg z): plate -4..0 | hub 1.3..7.3 = horn top | servo case
11.6..40.4 | idler face 43.7 | upper plate 42.3..48.3 | hip servo 48.6.. |
hip axis 61 (params leg.hip_axis_z).

Assembly order (asserted below and in check_assembly): fork onto the yaw
servo's horn (4x M3 x 6 from below, off the base) -> servo + fork slide into
the base cup from +X -> 2 rim screws from above (idler face) + 2 from below
the plate (horn face, deep pockets) -> hip servo slides into the fork's cup
from +X -> 4 rim screws -> module hooks onto the deck (I1).
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body, plug_envelope, horn_slot_cutter, spec, z_levels
from servo_mount import servo_cup, cup_extents, cup_screw_columns, FRONT_X, T as CUP_T
from leg_frame import (YAW_TF, HIP_TF, L1, Z_HIP, YB, HUB_Z0, HUB_Z1, Z_YAW_TOP, ZC_YAW,
                       Z_YAW_IDLER_FACE, Z_YAW_RIM_TOP, Z_CUP_FLOOR_TOP, FIT, HALF_W, yaw_z)

P = params()
S = spec(P)
Z = z_levels(P)
PR = P["print"]
IF = P["interfaces"]["leg_port"]

HUB_D = 26.0
HORN_POCKET_DEPTH = 1.0              # the hub rides the horn OD this deep (plateau is 1.7 further)
CB_DEPTH = 3.0                       # head counterbores from the hub underside
WEB_X0, WEB_X1 = 18.0, 22.0          # vertical web in front of the yaw servo (case front at +10.2)
WEB_HALF_W = 13.0
UP_Z0 = Z_YAW_RIM_TOP + FIT           # 42.2: upper plate underside clears the bottom rim
UP_T = 6.0
UP_Z1 = UP_Z0 + UP_T                  # 48.2
IDLER_POCKET_D = S["idler_d"] + FIT + 0.1
IDLER_POCKET_Z1 = Z_YAW_IDLER_FACE + FIT   # 44.0
NOTCH_X = -9.5                        # upper hub cut flat here on the -X side: the cable plugs live behind
NOTCH_HALF_W = 10.3
BACK_HALF_W = 9.5                     # base cup back wall width: I1 thumbscrew heads at |y| >= 11
REAR_TRIM_X = -33.9                   # nothing wider than BACK_HALF_W behind this (thumbscrew heads end at x -34)

_e = cup_extents()


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


# ---------------------------------------------------------------- base
def yaw_servo_placed(clearance=0.0):
    return YAW_TF * servo_body(P, clearance)


def coxa_yaw_base():
    from iface import leg_port_plate_features
    plate = Pos((-46 + 20) / 2, 0, -2) * Box(66, 44, 4)
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
    # the I1 thumbscrew heads (|y| >= 11, x <= -35) own the space behind REAR_TRIM_X
    for sy in (1, -1):
        base -= _box(-60, REAR_TRIM_X, min(sy * BACK_HALF_W, sy * 30), max(sy * BACK_HALF_W, sy * 30), 0.0, 60)
    # horn-face rim screws from BELOW the plate: Ø5 pockets up to 3 mm under the rail top
    for x, y in [(x, -y) for x, y in __import__("servo_st3215").case_screw_positions("top", P) if x < FRONT_X]:
        base -= Pos(x, y, (-5 + yaw_z(_e["z_hi_out"]) + CUP_T - 3) / 2) * \
            Cylinder(5.0 / 2, yaw_z(_e["z_hi_out"]) + CUP_T - 3 + 5)
        base -= Pos(x, y, 5) * Cylinder(S["case_screw"]["clear_d"] / 2, 30)
    return base


# ---------------------------------------------------------------- fork
def hip_servo_placed(clearance=0.0):
    return HIP_TF * servo_body(P, clearance)


def coxa_fork():
    # lower hub on the yaw horn (horn faces down; hub sits under it)
    hub = Pos(0, 0, (HUB_Z0 + HUB_Z1) / 2) * Cylinder(HUB_D / 2, HUB_Z1 - HUB_Z0)
    arm = _box(0, WEB_X1, -WEB_HALF_W, WEB_HALF_W, HUB_Z0, HUB_Z1)
    web = _box(WEB_X0, WEB_X1, -WEB_HALF_W, WEB_HALF_W, HUB_Z0, UP_Z1)
    up = _box(0, WEB_X1, -WEB_HALF_W, WEB_HALF_W, UP_Z0, UP_Z1)
    up += Pos(0, 0, (UP_Z0 + UP_Z1) / 2) * Cylinder(HUB_D / 2, UP_T)
    cup = HIP_TF * servo_cup()
    fork = hub + arm + web + up + cup
    # --- yaw horn interface (from below): OD pocket, centre-head relief, slots + counterbores
    fork -= Pos(0, 0, HUB_Z1 - HORN_POCKET_DEPTH / 2 + 0.5) * \
        Cylinder((S["horn_d"] + FIT) / 2, HORN_POCKET_DEPTH + 1)
    fork -= Pos(0, 0, HUB_Z1 - (S["horn_center_head_h"] + FIT) / 2 + 0.5) * \
        Cylinder((S["horn_center_head_d"] + 1.0) / 2, S["horn_center_head_h"] + FIT + 1)
    fork -= horn_slot_cutter(HUB_Z0 - 1, HUB_Z1 + 1, p=P)
    fork -= horn_slot_cutter(HUB_Z0 - 1, HUB_Z0 + CB_DEPTH,
                             extra=S["horn_screw_head_d"] - S["horn_screw_clear_d"], p=P)
    # --- yaw idler interface (from above): OD pocket + centre-head relief, open toward the plugs
    fork -= Pos(0, 0, (UP_Z0 - 1 + IDLER_POCKET_Z1) / 2) * Cylinder(IDLER_POCKET_D / 2, IDLER_POCKET_Z1 - UP_Z0 + 1)
    fork -= Pos(0, 0, (UP_Z0 - 1 + yaw_z(Z["idler_head"]) + FIT) / 2) * \
        Cylinder((S["idler_center_head_d"] + 1.0) / 2, yaw_z(Z["idler_head"]) + FIT - UP_Z0 + 1)
    fork -= _box(-40, NOTCH_X, -NOTCH_HALF_W, NOTCH_HALF_W, UP_Z0 - 1, UP_Z1 + 1)
    return fork


def _v(x):
    return 0.0 if x is None else x.volume


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
    check("fork x yaw servo (rides horn + idler only)", _v(fork & yaw_s), lambda v: v < 1, "OK", "CLASH")
    check("fork x hip servo", _v(fork & hip_s), lambda v: v < 1, "OK", "CLASH")
    check("base x hip servo", _v(base & hip_s), lambda v: v < 1, "OK", "CLASH")
    worst = 0.0
    for yaw in range(-40, 41, 10):
        r = Rot(0, 0, yaw)
        worst = max(worst, _v((r * fork) & base), _v((r * hip_s) & base))
    check("base x (fork + hip servo) over yaw -40..40", worst, lambda v: v < 1, "OK", "CLASH")
    check("fork x yaw plug keep-out", _v(fork & (YAW_TF * plug_envelope(P))), lambda v: v < 1, "CLEAR", "BLOCKS PLUGS")
    check("base x yaw plug keep-out", _v(base & (YAW_TF * plug_envelope(P))), lambda v: v < 1, "CLEAR", "BLOCKS PLUGS")
    check("fork x hip plug keep-out", _v(fork & (HIP_TF * plug_envelope(P))), lambda v: v < 1, "CLEAR", "BLOCKS PLUGS")
    # assembly paths
    worst = max(_v((Pos(dx, 0, 0) * (fork + yaw_s)) & base) for dx in (1, 2, 5, 10, 20, 40))
    check("servo + fork slide-in path (+X) vs base", worst, lambda v: v < 1, "OPEN", "BLOCKED")
    worst = max(_v((Pos(dx, 0, 0) * hip_s) & (fork + base)) for dx in (1, 2, 5, 10, 20, 40))
    check("hip servo slide-in path (+X) vs fork + base", worst, lambda v: v < 1, "OPEN", "BLOCKED")
    # the horn pocket locates the fork radially; the idler pocket the top
    check("fork nudged +X x yaw servo (horn pocket locates)", _v((Pos(2, 0, 0) * fork) & yaw_s), lambda v: v > 1, "LOCATED", "LOOSE")
    check("fork nudged +Y x yaw servo", _v((Pos(0, 2, 0) * fork) & yaw_s), lambda v: v > 1, "LOCATED", "LOOSE")
    # rim-screw driver columns: idler-face screws from above vs the fork at every yaw
    worst = 0.0
    for col in cup_screw_columns("bot"):
        c = YAW_TF * col
        for yaw in range(-40, 41, 20):
            worst = max(worst, _v(c & (Rot(0, 0, yaw) * fork)))
    check("base idler-face rim screws (from above) vs fork over yaw", worst, lambda v: v < 1, "CLEAR", "BLOCKED")
    print(f"part_coxa checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails: sys.exit(1)
