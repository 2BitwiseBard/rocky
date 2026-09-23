"""Tibia assembly, v0.1 — completes the leg's printable part set.

Three printed parts + the carbon tube + spring + microswitch:

  tibia_knee_carrier : carries the TIBIA SERVO (whose horn bolts to femur link
                       hub B, so this whole assembly swings about the knee) and
                       clamps the top of the carbon tube.
  tibia_sea_outer    : sleeve clamped to the tube's bottom end; spring bore.
  tibia_sea_slider   : slides ~7 mm inside the outer against the spring;
                       bottom stub = tube-OD so it plugs straight into the
                       existing hand hub socket. Carries the microswitch
                       striker (switch sits in the outer's pocket).

Frames: modeled in the LEG-LOCAL frame used everywhere (knee axis along Y at
(L1+L2, *, 58)); the SEA parts are modeled in their own local +Z-down frame
and get posed by the assembly/preview scripts.
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body, horn_screw_positions
from servo_mount import retention_lips, strap_tap_bores, servo_strap

P = params()
S = P["servo_st3215"]
PR = P["print"]
FIT = PR["clearance_fit"]
L1 = P["leg"]["l1_coxa"]; L2 = P["leg"]["l2_femur"]
KNEE_X = L1 + L2                          # 140
Z_AXIS = 58.0                             # knee axis height in leg frame
TUBE_OD = P["leg"]["tibia_tube_od"]       # 10
SEA_TRAVEL = P["leg"]["sea_travel"]       # 7

# knee servo pose (same convention as femur servo): shaft along Y, horn toward -Y,
# case base plane at y=+22 -> case spans y -10..22, x KNEE_X+x0 .. KNEE_X+shaft_offset
CASE_BASE_Y = 22.0

def knee_servo_placed(clearance=0.0):
    return Pos(KNEE_X, CASE_BASE_Y, Z_AXIS) * Rot(90, 0, 0) * servo_body(P, clearance)

def tibia_knee_carrier():
    """Cradle wrapping the knee servo + tube clamp dropping below it."""
    x0 = KNEE_X + S["shaft_offset"] - S["body_l"]          # 106.05
    x1 = KNEE_X + S["shaft_offset"]                        # 151.25
    y0, y1 = CASE_BASE_Y - S["body_h"], CASE_BASE_Y        # -10..22
    wall = 3.3
    # floor under the case (case bottom z = Z_AXIS - body_w/2 = 45.65)
    z_floor_top = Z_AXIS - S["body_w"]/2 - FIT             # 45.35
    floor = Pos((x0+x1)/2, (y0+y1)/2, z_floor_top - 2) * \
            Box(x1-x0 + 2*(FIT+wall), y1-y0 + FIT + wall, 4)
    # end walls (X faces) + back wall (+Y)
    car = floor
    wall_top = Z_AXIS + S["body_w"]/2 + 5
    for wx in (x0 - FIT - wall/2, x1 + FIT + wall/2):
        car += Pos(wx, (y0+y1)/2, (z_floor_top - 4 + wall_top)/2) * \
               Box(wall, y1-y0, wall_top - z_floor_top + 4)
    car += Pos((x0+x1)/2, y1 + FIT + wall/2, (z_floor_top - 4 + wall_top)/2) * \
           Box(x1-x0 + 2*(FIT+wall), wall, wall_top - z_floor_top + 4)
    # D046 retention (was zip-tie slots): lips over the horn-side corners
    # block the pull toward the femur link; a strap (tibia_knee_strap) holds
    # the case down with 2x M3 thread-forming into the wall tops
    car += retention_lips(x0, x1, y0, z_floor_top, z_floor_top, wall_top, wall)
    car = strap_tap_bores(car, x0, x1, (y0+y1)/2, wall_top, wall)
    # tube clamp boss under the knee, tube axis vertical at (KNEE_X, 0)
    boss = Pos(KNEE_X, 0, z_floor_top - 4 - 11) * Cylinder(19/2, 22)
    boss -= Pos(KNEE_X, 0, z_floor_top - 4 - 12) * Cylinder((TUBE_OD + FIT)/2, 26)
    # clamp slit + M3 pinch bolt (v0.2, session 6): the v0.1 bolt was a plain
    # Ø3.4 through-bore — nothing to thread into, and a loose nut would sit
    # on the boss's curved surface. Standard printed-clamp pattern instead:
    # clearance on the +X (head) side up to the slit, thread-forming Ø2.8 on
    # the far side (≥4 mm of engagement in the boss wall), and a spot-faced
    # counterbore so the M3x10 head seats on a FLAT, not the curve.
    zb = z_floor_top - 4 - 11
    boss -= Pos(KNEE_X, 5, zb) * Box(1.6, 12, 24)
    boss -= Pos(KNEE_X + 7.6, 8, zb) * Rot(0, 90, 0) * \
            Cylinder(PR["screw_m3_clear"]/2, 16.8)          # clear: slit -> +X
    boss -= Pos(KNEE_X - 7.9, 8, zb) * Rot(0, 90, 0) * \
            Cylinder(PR["screw_m3_tap"]/2, 14.2)            # tap: slit -> -X
    boss -= Pos(KNEE_X + 9.8, 8, zb) * Rot(0, 90, 0) * \
            Cylinder(6.5/2, 11.6)   # head spot-face, seat at x +4.0 (Ø6.5
                                    # keeps >=1 mm wall to the tube bore)
    # bridge web joining boss to floor
    web = Pos(KNEE_X, (y0+y1)/2 - 6, z_floor_top - 3) * Box(24, 12, 6)
    return car + boss + web

def tibia_knee_strap():
    x0 = KNEE_X + S["shaft_offset"] - S["body_l"]
    x1 = KNEE_X + S["shaft_offset"]
    y0, y1 = CASE_BASE_Y - S["body_h"], CASE_BASE_Y
    return servo_strap(x0, x1, (y0+y1)/2, Z_AXIS + S["body_w"]/2,
                       Z_AXIS + S["body_w"]/2 + 5, 3.3)

def tibia_sea_outer():
    """Local frame: +Z down the shin; z=0 at tube end. Clamps tube, houses spring."""
    body = Pos(0, 0, 14) * Cylinder(19/2, 28)               # z 0..28
    body -= Pos(0, 0, 5) * Cylinder((TUBE_OD + FIT)/2, 12)  # tube socket z 0..11 (glue/clamp)
    body -= Pos(0, 0, 20.5) * Cylinder(13.2/2, 17)          # spring + slider bore z 12..28+
    # slider anti-rotation keyways (2x). v0.2.1 (session 8, D038 audit): the
    # keyway reaches y 8.7 inside a r 9.5 body — 0.8 mm of skin, exactly two
    # perimeters. External ribs behind each keyway take the skin to 2.8.
    for sy in (1, -1):
        body += Pos(0, sy*9.7, 20) * Box(6.0, 2.4, 16)      # rib, outside (z 12..28)
        body -= Pos(0, sy*7.2, 21) * Box(3.2, 3.0, 16)
    # microswitch pocket (KW10-class micro lever switch ~12.7x5.8x6.5) + wire slot
    body -= Pos(8.2, 0, 24) * Box(6.8, 13.2, 6.2)
    body -= Pos(8.2, 0, 18) * Box(3, 4, 14)
    return body

def tibia_sea_slider():
    """Slides in the outer: flange + keys ride the bore; stub mates hand hub
    socket. v0.2: the stub carries tool-socket I2 (D020) — two radial bayonet
    lugs; the hand hub (and any future tool) provides the L-slots. Walking
    compression still goes stub FACE -> socket shoulder; lugs only retain."""
    IF2 = P["interfaces"]["tool_socket"]
    flange = Pos(0, 0, 2.5) * Cylinder(12.9/2, 5)           # z 0..5 inside bore
    for sy in (1, -1):
        flange += Pos(0, sy*7.0, 2.5) * Box(2.9, 2.6, 5)    # keys
    stem = Pos(0, 0, 7.5) * Cylinder(8/2, 5)                # through spring ID
    stub = Pos(0, 0, 10 + 7) * Cylinder((TUBE_OD - 0.1)/2, 14)   # z 10..24: hand socket stub
    # I2 bayonet lugs: at lug_z_from_face above the stub tip (z 24)
    lug_z = 24 - IF2["lug_z_from_face"]
    for sx in (1, -1):
        stub += Pos(sx * (TUBE_OD/2 + IF2["lug_h"]/2 - 0.15), 0, lug_z) * \
            Rot(0, 90, 0) * Cylinder(IF2["lug_d"]/2, IF2["lug_h"] + 0.3)
    # striker bump that trips the microswitch near end of travel
    striker = Pos(7.4, 0, 4) * Box(3.4, 3.4, 3)
    return flange + stem + stub + striker

if __name__ == "__main__":
    carrier = tibia_knee_carrier()
    outer = tibia_sea_outer()
    slider = tibia_sea_slider()
    export(carrier, "tibia_knee_carrier")
    export(outer, "tibia_sea_outer")
    export(slider, "tibia_sea_slider")
    strap = tibia_knee_strap()
    export(strap, "tibia_knee_strap")
    fails = []

    # quick static interference: carrier vs knee servo, carrier vs femur link plane
    servo = knee_servo_placed()
    inter = carrier & servo
    print("carrier x knee-servo intersection mm^3:", 0.0 if inter is None else round(inter.volume, 2))
    if inter is not None and inter.volume > 1: fails.append("carrier x servo")
    vv = lambda x: 0.0 if x is None else x.volume
    for name, val, good in (
            ("strap x knee servo", vv(strap & servo), lambda v: v < 1),
            ("strap x carrier", vv(strap & carrier), lambda v: v < 1),
            ("knee servo nudged -Y x carrier (lips)", vv((Pos(0, -2, 0) * servo) & carrier), lambda v: v > 1),
            ("knee servo nudged +Z x strap", vv((Pos(0, 0, 2) * servo) & strap), lambda v: v > 1),
            ("knee servo drop-in (+Z 10) x carrier", vv((Pos(0, 0, 10) * servo) & carrier), lambda v: v < 1)):
        print(f"{name}: {val:.2f} mm^3 ({'OK' if good(val) else 'FAIL'})")
        if not good(val): fails.append(name)

    # ---- I2 bayonet engagement check (D020) ----
    # pose the slider stub into the hand hub socket: slider z24 (tip) lands at
    # hub z +0.6 (bore-end shoulder is at z+1) => hand_z = slider_z - 23.4.
    # Check BOTH bayonet positions: inserted (lugs in the axial entry) and
    # locked (twisted 90 deg) — neither may intersect hub material.
    from part_hand import hand_hub
    hub = hand_hub()
    for name, rot in (("inserted", 0), ("locked", 90)):
        posed = Pos(0, 0, -23.4) * Rot(0, 0, rot) * slider
        inter = posed & hub
        v = 0.0 if inter is None else inter.volume
        print(f"I2 bayonet {name}: stub x hub = {v:.2f} mm^3 "
              f"({'OK' if v < 1.0 else 'BINDS'})")
    # and the lugs must NOT be able to pull straight out when locked:
    # pull the locked slider up 3 mm — the lugs should now hit slot ceiling
    pulled = Pos(0, 0, -23.4 - 3.0) * Rot(0, 0, 90) * slider
    inter = pulled & hub
    v = 0.0 if inter is None else inter.volume
    print(f"I2 locked + pulled 3 mm: intersection = {v:.2f} mm^3 "
          f"({'RETAINS (good)' if v > 1.0 else 'FALLS OUT — slot geometry wrong'})")
    if v <= 1.0: fails.append("I2 retention")
    print(f"part_tibia checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails:
        import sys; sys.exit(1)
