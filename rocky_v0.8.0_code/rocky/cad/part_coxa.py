"""Coxa yaw module, v0.1 — two printed parts.

Frame: yaw axis = +Z at origin. Yaw servo sits shaft-up (as servo_st3215 frame).
Key planes (from params, servo h=32, boss 3, horn 3):
  z=0        base plate top / servo base
  z=38       horn top -> fork hub bottom
  z=45.65    fork floor top (femur cradle floor)
  z=58       femur joint axis height (horizontal, along Y, at x = L1 = 45)

Parts:
  coxa_yaw_base : deck-mounted bracket = base plate + servo cradle walls
                  + crown post & arm with 683ZZ bearing pocket over the yaw axis
  coxa_fork     : rotates on the yaw servo horn = hub + floor plate + servo rails
                  + top stub axle that rides in the crown bearing
The femur servo drops between the fork rails (shaft along +Y, axis at x=45,z=58);
the +Y rail has a clearance hole so the horn pokes through to drive the femur link.
v0.1 retention is zip-ties through rail slots (real screw bosses come in v0.2 once
we've measured the servo's tapped holes on hardware).
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body, horn_screw_positions

P = params()
S = P["servo_st3215"]
B = P["bearing_683zz"]
PR = P["print"]
FIT = PR["clearance_fit"]

H_HORN_TOP = S["body_h"] + S["boss_h"] + S["horn_h"]        # 38.0
HUB_T = 3.0
FLOOR_T = 4.65
Z_FLOOR_TOP = H_HORN_TOP + HUB_T + FLOOR_T                   # 46.65 -> see below
# femur servo lies on its side on the fork floor; its case half-width sets axis height
Z_FEMUR_AXIS = Z_FLOOR_TOP + S["body_w"] / 2                 # floor top + 12.35

L1 = P["leg"]["l1_coxa"]                                     # 45: yaw axis -> femur axis

def _case_xspan():
    front = S["shaft_offset"]
    return front - S["body_l"], front                        # (-33.95, +11.25)

def coxa_yaw_base():
    """v0.2: the base plate now carries leg port I1 (D020) — dowel bores,
    captive thumbscrews at the reachable inboard corners, inboard hook lip.
    v0.1's six M3 holes are gone (mid pair was unreachable under the servo
    AND off the deck pentagon — layout audit 07-30); the outer pair (x=15,
    off-deck, D012 cantilever) stays as spare holes for jigs/full-scale."""
    from iface import leg_port_plate_features
    x0, x1 = _case_xspan()
    wall_in = S["body_w"]/2 + FIT                            # 12.65
    wall_t = 3.0
    # base plate
    plate = Pos((-46+20)/2, 0, -2) * Box(66, 44, 4)
    for hx, hy in [(15,17),(15,-17)]:                        # spare pair only
        plate -= Pos(hx, hy, -2) * Cylinder(PR["screw_m3_clear"]/2, 6)
    # cable slot: yaw servo lead exits down through the plate behind the case
    plate -= Pos(-29, 0, -2) * Box(9, 9, 8)
    # leg port I1: dowel bores + captive thumbscrew bores + inboard hook lip
    plate = leg_port_plate_features(plate, plate_top_z=0.0, plate_bot_z=-4.0)
    # cradle walls
    wall_len = (x1 + FIT) - (x0 - FIT)
    wall_cx = (x0 + x1) / 2
    for sy in (1, -1):
        plate += Pos(wall_cx, sy*(wall_in + wall_t/2), 13) * Box(wall_len, wall_t, 26)
    # crown: post behind servo + arm over the yaw axis
    arm_z0 = Z_FLOOR_TOP + 0.5                               # clear rotating floor
    arm_t = 4.0
    post = Pos(-41, 0, (arm_z0 + arm_t)/2) * Box(10, 12, arm_z0 + arm_t)
    arm = Pos((-46+8)/2, 0, arm_z0 + arm_t/2) * Box(54, 16, arm_t)
    crown = post + arm
    # bearing pocket, cut upward from arm underside at the yaw axis
    pocket_d = B["od"] - PR["clearance_press"]                # light press
    crown -= Pos(0, 0, arm_z0 + (B["w"]+0.2)/2) * Cylinder(pocket_d/2, B["w"]+0.2)
    return plate + crown

# Femur servo pose (corrected v0.1b): shaft axis along Y at (L1, *, Z_FEMUR_AXIS),
# horn OUTBOARD toward -Y (link plate rides there), case body y in [-10, +22]:
#   world = Pos(L1, CASE_BASE_Y, Z_FEMUR_AXIS) * Rot(90,0,0) * servo_local
CASE_BASE_Y = 22.0            # case base plane; case extends to y = CASE_BASE_Y - 32
HORN_TOP_Y = CASE_BASE_Y - H_HORN_TOP   # -16.0: femur-link plate mounts here

def fork_screw_positions():
    """Yaw-horn M2 pattern as the FORK uses it: the servo BCD rotated 45°."""
    import math
    c, s_ = math.cos(math.pi/4), math.sin(math.pi/4)
    return [(hx*c - hy*s_, hx*s_ + hy*c) for hx, hy in horn_screw_positions(P)]

def coxa_fork():
    hub = Pos(0, 0, H_HORN_TOP + HUB_T/2) * Cylinder(26/2, HUB_T)
    # horn screw holes + center pilot
    # v0.2.2: screw pattern rotated 45° on the horn BCD — the (7,0) hole sat
    # directly under the -X end wall (x 7.45+), so its driver column was
    # blocked; at 45° all four sit at x = ±4.95 with 0.4 mm to spare. The
    # horn's spline angle is free, so this costs nothing at assembly.
    for hx, hy in fork_screw_positions():
        hub -= Pos(hx, hy, H_HORN_TOP + HUB_T/2) * Cylinder(S["horn_screw_d"]/2, HUB_T+2)
    hub -= Pos(0, 0, H_HORN_TOP + HUB_T/2) * Cylinder(6.4/2, HUB_T+2)
    # floor plate spans x -13..60, y -13..25.6 (case footprint + wall margins + hub)
    # v0.2.2 (session 8, D038 printability audit): the floor used to start at
    # hub-top − 0.5 (z 40.5) — 2.5 mm ABOVE the hub bottom, so 'hub-down, no
    # support' printed a 73 x 39 plate in mid-air off a Ø26 hub. The floor
    # now starts at the horn-top plane (z 38): flat underside, zero support.
    # (The base x fork sweep check below proves nothing lives in that band.)
    z_f0 = H_HORN_TOP
    floor = Pos((-13+60)/2, (-13+25.6)/2, (z_f0 + Z_FLOOR_TOP)/2) * \
            Box(73, 38.6, Z_FLOOR_TOP - z_f0)
    # Also v0.2.2: the floor COVERED the four horn-screw holes and the centre
    # pilot — as drawn since v0.1, the fork could not be bolted to its yaw
    # horn (boolean probe: 17.7 mm³ of plastic above each hole, 158 mm³ over
    # the pilot). Assembly order is now explicit: horn onto the spline with
    # its centre screw FIRST, then the fork drops on and four M2 x 5 go in
    # from above through Ø4.2 counterbores that open the floor down to the
    # hub top (heads seat on the 3 mm hub, under the crown-bearing stub's
    # plane; counterbores at r 7 clear the Ø2.9 stub and the femur case).
    for hx, hy in fork_screw_positions():
        floor -= Pos(hx, hy, (H_HORN_TOP + HUB_T + Z_FLOOR_TOP + 1) / 2) * \
            Cylinder(4.2/2, Z_FLOOR_TOP + 1 - (H_HORN_TOP + HUB_T))
        floor -= Pos(hx, hy, (z_f0 + Z_FLOOR_TOP) / 2) * \
            Cylinder(S["horn_screw_d"]/2, Z_FLOOR_TOP - z_f0 + 2)
    x0, x1 = _case_xspan()
    case_x0 = L1 + x0                                         # 11.05
    case_x1 = L1 + S["shaft_offset"]                          # 56.25
    case_y0 = CASE_BASE_Y - S["body_h"]                       # -10.0
    case_y1 = CASE_BASE_Y                                     # +22.0
    wall_t = 3.3
    wall_top = Z_FEMUR_AXIS + S["body_w"]/2 + 5               # above case top
    # -X end wall must clear the fixed crown arm (z 46.15..50.15) through the full
    # +/-40 deg yaw sweep -> it starts above the arm as a "high collar".
    Z_ARM_CLEAR = 51.0
    walls = None
    # v0.2.1 (session 8): the end walls used to span exactly the case (y0..y1)
    # while the back wall sits FIT beyond it — a 0.3 mm gap at both corners.
    # The +X wall didn't care (it stands on the floor); the -X high collar
    # starts above the floor at Z_ARM_CLEAR and so was attached to NOTHING —
    # the fork exported as two bodies (D036 class). End walls now run through
    # the back wall (y to case_y1 + FIT + wall_t) so the collar hangs off it.
    y_wall_hi = case_y1 + FIT + wall_t
    for wx, z_lo in ((case_x0 - FIT - wall_t/2, Z_ARM_CLEAR),
                     (case_x1 + FIT + wall_t/2, Z_FLOOR_TOP)):
        w = Pos(wx, (case_y0 + y_wall_hi)/2, (z_lo + wall_top)/2) * \
            Box(wall_t, y_wall_hi - case_y0, wall_top - z_lo)
        walls = w if walls is None else walls + w
    # back wall on +Y (opposite the horn) ties the end walls together
    walls += Pos((case_x0 + case_x1)/2, case_y1 + FIT + wall_t/2,
                 (Z_FLOOR_TOP + wall_top)/2) * \
             Box(case_x1 - case_x0 + 2*(FIT + wall_t), wall_t, wall_top - Z_FLOOR_TOP)
    # zip-tie slots through the end walls, above the case top
    slot_z = Z_FEMUR_AXIS + S["body_w"]/2 + 2
    for wx in (case_x0 - FIT - wall_t/2, case_x1 + FIT + wall_t/2):
        walls -= Pos(wx, (case_y0 + case_y1)/2, slot_z) * Box(6, 4, 3.2)
    # top stub axle into the crown bearing
    stub = Pos(0, 0, Z_FLOOR_TOP + 3.2/2) * Cylinder((B["id"] - 0.1)/2, 3.2)
    return hub + floor + walls + stub

def femur_servo_placed(clearance=0.0):
    """Femur servo dummy in its fork pose: shaft axis along Y at (L1, *, Z_FEMUR_AXIS),
    horn outboard toward -Y (horn top plane at y = HORN_TOP_Y)."""
    s = servo_body(P, clearance)
    return Pos(L1, CASE_BASE_Y, Z_FEMUR_AXIS) * Rot(90, 0, 0) * s

if __name__ == "__main__":
    base = coxa_yaw_base()
    fork = coxa_fork()
    export(base, "coxa_yaw_base")
    export(fork, "coxa_fork")
    print(f"Z_FLOOR_TOP={Z_FLOOR_TOP:.2f}  Z_FEMUR_AXIS={Z_FEMUR_AXIS:.2f}")
    # v0.2 checks: thumbscrew access columns must be CLEAR above the plate
    # (that's the whole point of moving them), dowel bores clear of walls
    from iface import IF
    lp = IF["leg_port"]
    worst = 0.0
    for dx, dy in lp["thumbscrew_xy"]:
        probe = Pos(dx, dy, 30) * Cylinder(lp["thumbscrew_head_d"]/2 + 1, 56)
        inter = probe & base
        v = 0.0 if inter is None else inter.volume
        worst = max(worst, v)
    print(f"thumbscrew access columns: worst obstruction {worst:.1f} mm^3 "
          f"({'CLEAR — screwdriver-free swap works' if worst < 1 else 'BLOCKED'})")
    fork_i = fork & base
    v = 0.0 if fork_i is None else fork_i.volume
    print(f"base x fork: {v:.2f} mm^3 ({'OK' if v < 1 else 'CLASH'})")
    # v0.2.2 (session 8): the yaw SWEEP is checked in-module now (v0.1 did it
    # once by hand) — the floor now reaches the horn-top plane, so the base
    # must be clear of it at every yaw angle, not just zero.
    worst_sweep = 0.0
    for yaw in range(-40, 41, 10):
        inter = (Rot(0, 0, yaw) * fork) & base
        worst_sweep = max(worst_sweep, 0.0 if inter is None else inter.volume)
    print(f"base x fork over yaw -40..40: worst {worst_sweep:.2f} mm^3 "
          f"({'OK' if worst_sweep < 1 else 'CLASH'})")
    # femur servo must drop into the fork cradle (tangent on the floor)
    fs = fork & femur_servo_placed()
    vfs = 0.0 if fs is None else fs.volume
    print(f"fork x femur servo: {vfs:.2f} mm^3 ({'OK' if vfs < 1 else 'CLASH'})")
    # horn-screw access: a driver column above every hole must be EMPTY
    # (the v0.1-v0.2.1 floor blocked all four — regression test)
    worst_access = 0.0
    for hx, hy in fork_screw_positions():
        col = Pos(hx, hy, (H_HORN_TOP + HUB_T + 60) / 2) * \
            Cylinder(1.9, 60 - (H_HORN_TOP + HUB_T))
        inter = col & fork
        worst_access = max(worst_access, 0.0 if inter is None else inter.volume)
    print(f"horn-screw driver access: worst obstruction {worst_access:.2f} mm^3 "
          f"({'CLEAR' if worst_access < 1 else 'BLOCKED'})")
    ok = v < 1 and worst_sweep < 1 and vfs < 1 and worst_access < 1
    print(f"part_coxa checks: {'ALL CLEAN' if ok else 'FAILED'}")
    if not ok:
        import sys
        sys.exit(1)
