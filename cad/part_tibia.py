"""Tibia assembly, v0.2 (D047) — completes the leg's printable part set.

Three printed parts + the carbon tube + spring + microswitch:

  tibia_knee_carrier : the knee servo's CUP (servo_mount, four rim screws —
                       the D046 lips + strap are gone) + a floor extension
                       under the servo's front + the tube clamp boss dropping
                       below the knee axis. The knee servo's horn bolts to
                       femur plate A (through a coupler) and its idler rides
                       plate B, so this whole assembly swings about the knee.
  tibia_sea_outer    : sleeve clamped to the tube's bottom end; spring bore.
  tibia_sea_slider   : slides ~7 mm inside the outer against the spring;
                       bottom stub = tube-OD so it plugs straight into the
                       existing hand hub socket. Carries the microswitch
                       striker (switch sits in the outer's pocket).

Frames: leg-local (knee axis along Y at (KNEE_X, *, Z_HIP)); the SEA parts
are modeled in their own local +Z-down frame and get posed by the assembly.
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body, plug_envelope, spec
from servo_mount import servo_cup, cup_extents, FRONT_X
from leg_frame import KNEE_TF, KNEE_X, Z_HIP, YB, FIT, HALF_W, Z_CUP_FLOOR_TOP, L1, L2

P = params()
S = spec(P)
PR = P["print"]
TUBE_OD = P["leg"]["tibia_tube_od"]       # 10
SEA_TRAVEL = P["leg"]["sea_travel"]       # 7
Z_AXIS = Z_HIP                            # knee axis height (params leg.hip_axis_z)
CUP_FLOOR_Z0 = Z_CUP_FLOOR_TOP - 3.0      # 45.3 (cup wall thickness 3)
BOSS_D, BOSS_H = 19.0, 22.0
BOSS_Z1 = Z_CUP_FLOOR_TOP                 # boss top flush with the cup floor top (under the case)
BOSS_Z0 = BOSS_Z1 - BOSS_H                # 26.3
SOCKET_DEPTH = 18.0
EXT_HALF_W = 9.5                          # floor extension under the case: inside the horn/idler stacks


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def knee_servo_placed(clearance=0.0):
    return KNEE_TF * servo_body(P, clearance)


def tibia_knee_carrier():
    """Cup around the rear of the knee servo + tube clamp under the knee."""
    car = KNEE_TF * servo_cup()
    e = cup_extents()
    # floor extension from the cup's front edge to past the tube boss
    ext = _box(KNEE_X + FRONT_X - 0.01, KNEE_X + BOSS_D / 2, -EXT_HALF_W, EXT_HALF_W,
               CUP_FLOOR_Z0, Z_CUP_FLOOR_TOP)
    # tube clamp boss under the knee, tube axis vertical at (KNEE_X, 0)
    boss = Pos(KNEE_X, 0, (BOSS_Z0 + BOSS_Z1) / 2) * Cylinder(BOSS_D / 2, BOSS_H)
    boss -= Pos(KNEE_X, 0, BOSS_Z0 - 1 + (SOCKET_DEPTH + 1) / 2) * Cylinder((TUBE_OD + FIT) / 2, SOCKET_DEPTH + 1)
    # clamp slit + M3 pinch bolt (v0.2, session 6): clearance on the +X (head)
    # side up to the slit, thread-forming Ø2.8 on the far side (≥4 mm of
    # engagement in the boss wall), spot-faced counterbore so the M3x10 head
    # seats on a FLAT, not the curve.
    zb = BOSS_Z0 + SOCKET_DEPTH / 2
    boss -= Pos(KNEE_X, 5, zb) * Box(1.6, 12, SOCKET_DEPTH + 2)
    boss -= Pos(KNEE_X + 7.6, 8, zb) * Rot(0, 90, 0) * Cylinder(PR["screw_m3_clear"] / 2, 16.8)
    boss -= Pos(KNEE_X - 7.9, 8, zb) * Rot(0, 90, 0) * Cylinder(PR["screw_m3_tap"] / 2, 14.2)
    boss -= Pos(KNEE_X + 9.8, 8, zb) * Rot(0, 90, 0) * Cylinder(6.5 / 2, 11.6)
    return car + ext + boss


def tibia_sea_outer():
    """Local frame: +Z down the shin; z=0 at tube end. Clamps tube, houses spring."""
    body = Pos(0, 0, 14) * Cylinder(19 / 2, 28)               # z 0..28
    body -= Pos(0, 0, 5) * Cylinder((TUBE_OD + FIT) / 2, 12)  # tube socket z 0..11 (glue/clamp)
    body -= Pos(0, 0, 20.5) * Cylinder(13.2 / 2, 17)          # spring + slider bore z 12..28+
    for sy in (1, -1):
        body += Pos(0, sy * 9.7, 20) * Box(6.0, 2.4, 16)      # rib, outside (z 12..28)
        body -= Pos(0, sy * 7.2, 21) * Box(3.2, 3.0, 16)
    body -= Pos(8.2, 0, 24) * Box(6.8, 13.2, 6.2)             # microswitch pocket (KW10-class)
    body -= Pos(8.2, 0, 18) * Box(3, 4, 14)
    return body


def tibia_sea_slider():
    """Slides in the outer: flange + keys ride the bore; stub mates the hand
    hub socket and carries tool-socket I2 (D020) bayonet lugs."""
    IF2 = P["interfaces"]["tool_socket"]
    flange = Pos(0, 0, 2.5) * Cylinder(12.9 / 2, 5)
    for sy in (1, -1):
        flange += Pos(0, sy * 7.0, 2.5) * Box(2.9, 2.6, 5)
    stem = Pos(0, 0, 7.5) * Cylinder(8 / 2, 5)
    stub = Pos(0, 0, 10 + 7) * Cylinder((TUBE_OD - 0.1) / 2, 14)
    lug_z = 24 - IF2["lug_z_from_face"]
    for sx in (1, -1):
        stub += Pos(sx * (TUBE_OD / 2 + IF2["lug_h"] / 2 - 0.15), 0, lug_z) * \
            Rot(0, 90, 0) * Cylinder(IF2["lug_d"] / 2, IF2["lug_h"] + 0.3)
    striker = Pos(7.4, 0, 4) * Box(3.4, 3.4, 3)
    return flange + stem + stub + striker


if __name__ == "__main__":
    import sys
    v = lambda x: 0.0 if x is None else x.volume
    carrier, outer, slider = tibia_knee_carrier(), tibia_sea_outer(), tibia_sea_slider()
    export(carrier, "tibia_knee_carrier")
    export(outer, "tibia_sea_outer")
    export(slider, "tibia_sea_slider")
    fails = []
    def check(name, val, good, ok="OK", bad="FAIL"):
        okk = good(val)
        print(f"{name}: {val:.2f} mm^3 ({ok if okk else bad})")
        if not okk: fails.append(name)
    servo = knee_servo_placed()
    check("carrier x knee servo", v(carrier & servo), lambda q: q < 1, "OK", "CLASH")
    check("carrier x knee plug keep-out", v(carrier & (KNEE_TF * plug_envelope(P))), lambda q: q < 1, "CLEAR", "BLOCKS PLUGS")
    for name, d in (("+Y", (0, 2, 0)), ("-Y", (0, -2, 0)), ("+Z", (0, 0, 2)), ("-Z", (0, 0, -2)), ("-X", (-2, 0, 0))):
        check(f"knee servo nudged {name} x carrier", v((Pos(*d) * servo) & carrier), lambda q: q > 1, "HELD", "FREE")
    worst = max(v((Pos(dx, 0, 0) * servo) & carrier) for dx in (1, 2, 5, 10, 20, 40))
    check("knee servo slide-in path (+X)", worst, lambda q: q < 1, "OPEN", "BLOCKED")
    # knee sweep: the carrier must clear the femur link + plate B through -150..-20
    from part_femur import femur_link, femur_plate_b, LINK_TF
    link, pb = LINK_TF * femur_link(), LINK_TF * femur_plate_b()
    worst = 0.0
    for knee in range(-150, -19, 10):
        r = Pos(KNEE_X, 0, Z_AXIS) * Rot(0, -(knee + 90), 0) * Pos(-KNEE_X, 0, -Z_AXIS)
        c = r * carrier
        worst = max(worst, v(c & link), v(c & pb))
    check("carrier x femur (link + plate B) over knee -150..-20", worst, lambda q: q < 1, "OK", "CLASH")
    # ---- I2 bayonet engagement check (D020) ----
    from part_hand import hand_hub
    hub = hand_hub()
    for name, rot in (("inserted", 0), ("locked", 90)):
        posed = Pos(0, 0, -23.4) * Rot(0, 0, rot) * slider
        vv = v(posed & hub)
        print(f"I2 bayonet {name}: stub x hub = {vv:.2f} mm^3 ({'OK' if vv < 1.0 else 'BINDS'})")
        if vv >= 1: fails.append(f"I2 {name}")
    pulled = Pos(0, 0, -23.4 - 3.0) * Rot(0, 0, 90) * slider
    vv = v(pulled & hub)
    print(f"I2 locked + pulled 3 mm: intersection = {vv:.2f} mm^3 "
          f"({'RETAINS (good)' if vv > 1.0 else 'FALLS OUT — slot geometry wrong'})")
    if vv <= 1.0: fails.append("I2 retention")
    print(f"part_tibia checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails: sys.exit(1)
