"""Coxa yaw module, v0.3 (D046, laptop session 3 2026-09-17) — THREE printed
parts + a strap.

Frame: yaw axis = +Z at origin. Yaw servo sits shaft-up (as servo_st3215 frame).
Key planes (from params, servo h=32, boss 3, horn 3):
  z=0        base plate top / servo base
  z=38       horn top -> fork hub bottom
  z=45.65    fork floor top (femur cradle floor)
  z=47.15    axle boss top (683ZZ inner race seats here)
  z=47.55    crown CAP underside (0.4 nominal gap over the boss)
  z=58       femur joint axis height (horizontal, along Y, at x = L1 = 45)

Parts:
  coxa_yaw_base : deck-mounted bracket = base plate + servo cradle walls
                  + crown POST (2x M3 thread-forming in its top)
  coxa_crown_cap: the arm that used to be part of the base. Carries the
                  683ZZ pocket; bolts onto the post AFTER the fork is on.
  coxa_fork     : rotates on the yaw servo horn = hub + floor + servo rails
                  + retention lips; an M3 AXLE screw threads down into the
                  fork's boss through the cap's bearing (inner race clamped
                  between screw head and boss — no printed stub any more).
  coxa_fork_strap: bar over the rail tops holding the femur servo down.

Why v0.3 (check_assembly.py on the v0.8.0 tree): the crown arm was one
solid with the base, and all four yaw-horn screws sat under it — the fork
could not be bolted on in place, and pre-bolted it could not slide in (the
stub hit the arm). No assembly order existed. The vertical budget was 0.5 mm
(floor vs arm) with a 0 mm horn/hub gap; the horn's CENTRE screw head was
still covered by the floor (only the four M2 holes got counterbores in
v0.2.2); the servo was retained by zip ties only.

Assembly order (asserted below): yaw servo drops into the cradle from +Z ->
fork drops onto the horn, 4x M2 x 6 through the Ø4.2 counterbores (driver
columns asserted clear of base AND fork) -> cap onto the post, 2x M3 x 8 ->
M3 x 10 axle down through the cap bearing into the fork boss -> femur servo
drops into the rails from +Z (lips stop it toward the horn) -> strap, 2x M3.
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body, horn_screw_positions
from servo_mount import retention_lips, strap_tap_bores, servo_strap

P = params()
S = P["servo_st3215"]
B = P["bearing_683zz"]
PR = P["print"]
FIT = PR["clearance_fit"]

H_HORN_TOP = S["body_h"] + S["boss_h"] + S["horn_h"]        # 38.0
HUB_T = 3.0
FLOOR_T = 4.65
Z_FLOOR_TOP = H_HORN_TOP + HUB_T + FLOOR_T                   # 45.65
Z_FEMUR_AXIS = Z_FLOOR_TOP + S["body_w"] / 2                 # 58.0
L1 = P["leg"]["l1_coxa"]                                     # 45

# yaw axle (D046): boss on the fork, M3 thread-forming, cap bearing over it
HEAD_POCKET_D, HEAD_POCKET_Z1 = 6.6, H_HORN_TOP + 3.5        # horn centre screw head
AXLE_BOSS_D, AXLE_BOSS_H = 4.6, 1.5
Z_BOSS_TOP = Z_FLOOR_TOP + AXLE_BOSS_H                       # 47.15
CAP_GAP = 0.4
CAP_Z0 = Z_BOSS_TOP + CAP_GAP                                # 47.55
CAP_T = 5.0
CAP_Z1 = CAP_Z0 + CAP_T                                      # 52.55
Z_ARM_CLEAR = CAP_Z1 + 1.0                                   # -X collar starts here
POST_X, POST_W, POST_D = -41.0, 10.0, 18.0
ARM_X1 = 5.4                                                 # bearing pocket r3.43 + 2 mm wall
POST_SCREW_XY = [(POST_X, 5.0), (POST_X, -5.0)]

def _case_xspan():
    front = S["shaft_offset"]
    return front - S["body_l"], front                        # (-33.95, +11.25)

def coxa_yaw_base():
    """Base plate (leg port I1, D020) + cradle walls + crown POST."""
    from iface import leg_port_plate_features
    x0, x1 = _case_xspan()
    wall_in = S["body_w"]/2 + FIT                            # 12.65
    wall_t = 3.0
    plate = Pos((-46+20)/2, 0, -2) * Box(66, 44, 4)
    for hx, hy in [(15,17),(15,-17)]:                        # spare pair only
        plate -= Pos(hx, hy, -2) * Cylinder(PR["screw_m3_clear"]/2, 6)
    plate -= Pos(-29, 0, -2) * Box(9, 9, 8)                  # yaw servo lead
    plate = leg_port_plate_features(plate, plate_top_z=0.0, plate_bot_z=-4.0)
    wall_len = (x1 + FIT) - (x0 - FIT)
    wall_cx = (x0 + x1) / 2
    for sy in (1, -1):
        plate += Pos(wall_cx, sy*(wall_in + wall_t/2), 13) * Box(wall_len, wall_t, 26)
    # crown post behind the servo, top = cap underside; 2x M3 thread-forming
    post = Pos(POST_X, 0, CAP_Z0/2) * Box(POST_W, POST_D, CAP_Z0)
    for px, py in POST_SCREW_XY:
        post -= Pos(px, py, CAP_Z0 - 4) * Cylinder(PR["screw_m3_tap"]/2, 8.2)
    return plate + post

def coxa_crown_cap():
    """Arm over the yaw axis with the 683ZZ pocket (light press, from the
    underside) + Ø3.4 axle pass + 2x M3 clearance onto the post."""
    # arm ends at x=ARM_X1: the fork collar (x 7.45+, z 53.55+) overhangs the
    # axis at every yaw, so the cap must clear it to DROP on (asserted below)
    arm = Pos((-46+ARM_X1)/2, 0, CAP_Z0 + CAP_T/2) * Box(46+ARM_X1, 16, CAP_T)
    arm += Pos(POST_X, 0, CAP_Z0 + CAP_T/2) * Box(12, POST_D, CAP_T)
    pocket_d = B["od"] - PR["clearance_press"]
    arm -= Pos(0, 0, CAP_Z0 + (B["w"]+0.2)/2) * Cylinder(pocket_d/2, B["w"]+0.2)
    arm -= Pos(0, 0, CAP_Z0 + CAP_T/2) * Cylinder(PR["screw_m3_clear"]/2, CAP_T+2)
    for px, py in POST_SCREW_XY:
        arm -= Pos(px, py, CAP_Z0 + CAP_T/2) * Cylinder(PR["screw_m3_clear"]/2, CAP_T+2)
    return arm

# Femur servo pose: shaft axis along Y at (L1, *, Z_FEMUR_AXIS), horn OUTBOARD
# toward -Y (coupler + link ride there), case body y in [-10, +22]:
#   world = Pos(L1, CASE_BASE_Y, Z_FEMUR_AXIS) * Rot(90,0,0) * servo_local
CASE_BASE_Y = 22.0
HORN_TOP_Y = CASE_BASE_Y - H_HORN_TOP   # -16.0: coupler disc sits on this plane
FEMUR_CASE = dict(x0=L1 + _case_xspan()[0], x1=L1 + S["shaft_offset"],
                  y0=CASE_BASE_Y - S["body_h"], y1=CASE_BASE_Y,
                  z0=Z_FEMUR_AXIS - S["body_w"]/2, z1=Z_FEMUR_AXIS + S["body_w"]/2)
WALL_T = 3.3
WALL_TOP = FEMUR_CASE["z1"] + 5


def fork_screw_positions():
    """Yaw-horn M2 pattern as the FORK uses it: the servo BCD rotated 45°."""
    import math
    c, s_ = math.cos(math.pi/4), math.sin(math.pi/4)
    return [(hx*c - hy*s_, hx*s_ + hy*c) for hx, hy in horn_screw_positions(P)]

def coxa_fork():
    c = FEMUR_CASE
    hub = Pos(0, 0, H_HORN_TOP + HUB_T/2) * Cylinder(26/2, HUB_T)
    z_f0 = H_HORN_TOP
    floor = Pos((-13+60)/2, (-13+25.6)/2, (z_f0 + Z_FLOOR_TOP)/2) * \
            Box(73, 38.6, Z_FLOOR_TOP - z_f0)
    body = hub + floor
    # 4x M2 into the yaw horn: Ø2.2 through hub, Ø4.2 counterbore down to the
    # hub top so the heads seat on the 3 mm hub (driver from above)
    for hx, hy in fork_screw_positions():
        body -= Pos(hx, hy, (H_HORN_TOP + HUB_T + Z_FLOOR_TOP + 1) / 2) * \
            Cylinder(4.2/2, Z_FLOOR_TOP + 1 - (H_HORN_TOP + HUB_T))
        body -= Pos(hx, hy, (z_f0 + Z_FLOOR_TOP) / 2) * \
            Cylinder(S["horn_screw_d"]/2, Z_FLOOR_TOP - z_f0 + 2)
    # horn centre-screw head pocket (was covered by the floor through v0.8.0)
    body -= Pos(0, 0, (H_HORN_TOP - 1 + HEAD_POCKET_Z1)/2) * \
        Cylinder(HEAD_POCKET_D/2, HEAD_POCKET_Z1 - H_HORN_TOP + 1)
    # axle boss + M3 thread-forming bore (pocket top -> boss top = 5.65 mm)
    body += Pos(0, 0, Z_FLOOR_TOP + AXLE_BOSS_H/2) * Cylinder(AXLE_BOSS_D/2, AXLE_BOSS_H)
    body -= Pos(0, 0, (HEAD_POCKET_Z1 + Z_BOSS_TOP + 1)/2) * \
        Cylinder(PR["screw_m3_tap"]/2, Z_BOSS_TOP + 1 - HEAD_POCKET_Z1)
    # rails: end walls run through the back wall; -X wall is a high collar
    # that clears the crown cap through the yaw sweep
    y_wall_hi = c["y1"] + FIT + WALL_T
    for wx, z_lo in ((c["x0"] - FIT - WALL_T/2, Z_ARM_CLEAR),
                     (c["x1"] + FIT + WALL_T/2, Z_FLOOR_TOP)):
        body += Pos(wx, (c["y0"] + y_wall_hi)/2, (z_lo + WALL_TOP)/2) * \
            Box(WALL_T, y_wall_hi - c["y0"], WALL_TOP - z_lo)
    body += Pos((c["x0"] + c["x1"])/2, c["y1"] + FIT + WALL_T/2,
                (Z_FLOOR_TOP + WALL_TOP)/2) * \
            Box(c["x1"] - c["x0"] + 2*(FIT + WALL_T), WALL_T, WALL_TOP - Z_FLOOR_TOP)
    # D046 retention: lips over the horn-side top corners + strap tap bores
    body += retention_lips(c["x0"], c["x1"], c["y0"], Z_ARM_CLEAR, Z_FLOOR_TOP, WALL_TOP, WALL_T)
    body = strap_tap_bores(body, c["x0"], c["x1"], (c["y0"] + c["y1"])/2, WALL_TOP, WALL_T)
    return body

def coxa_fork_strap():
    c = FEMUR_CASE
    return servo_strap(c["x0"], c["x1"], (c["y0"] + c["y1"])/2, c["z1"], WALL_TOP, WALL_T)

def femur_servo_placed(clearance=0.0):
    """Femur servo dummy in its fork pose: shaft axis along Y at (L1, *, Z_FEMUR_AXIS),
    horn outboard toward -Y (horn top plane at y = HORN_TOP_Y)."""
    s = servo_body(P, clearance)
    return Pos(L1, CASE_BASE_Y, Z_FEMUR_AXIS) * Rot(90, 0, 0) * s

def _v(x):
    return 0.0 if x is None else x.volume

if __name__ == "__main__":
    import sys
    base = coxa_yaw_base()
    fork = coxa_fork()
    cap = coxa_crown_cap()
    strap = coxa_fork_strap()
    export(base, "coxa_yaw_base")
    export(fork, "coxa_fork")
    export(cap, "coxa_crown_cap")
    export(strap, "coxa_fork_strap")
    print(f"Z_FLOOR_TOP={Z_FLOOR_TOP:.2f}  Z_FEMUR_AXIS={Z_FEMUR_AXIS:.2f}  CAP_Z0={CAP_Z0:.2f}")
    from iface import IF
    lp = IF["leg_port"]
    fails = []
    def check(name, v, good, msg_ok, msg_bad):
        ok = good(v)
        print(f"{name}: {v:.2f} mm^3 ({msg_ok if ok else msg_bad})")
        if not ok: fails.append(name)
    # thumbscrew access columns stay clear (cap pad y ±9 vs column y ≥ 10)
    worst = 0.0
    for dx, dy in lp["thumbscrew_xy"]:
        probe = Pos(dx, dy, 30) * Cylinder(lp["thumbscrew_head_d"]/2 + 1, 56)
        worst = max(worst, _v(probe & base), _v(probe & cap))
    check("thumbscrew access columns (base+cap)", worst, lambda v: v < 1, "CLEAR", "BLOCKED")
    # yaw sweep: fork vs base AND cap (cap is fixed with the base)
    worst = 0.0
    for yaw in range(-40, 41, 10):
        f = Rot(0, 0, yaw) * fork
        worst = max(worst, _v(f & base), _v(f & cap))
    check("base+cap x fork over yaw -40..40", worst, lambda v: v < 1, "OK", "CLASH")
    check("cap x base (touching at the post top)", _v(cap & base), lambda v: v < 1, "OK", "CLASH")
    check("fork x femur servo", _v(fork & femur_servo_placed()), lambda v: v < 1, "OK", "CLASH")
    check("strap x femur servo", _v(strap & femur_servo_placed()), lambda v: v < 1, "OK", "CLASH")
    check("strap x fork", _v(strap & fork), lambda v: v < 1, "OK", "CLASH")
    # horn-screw driver access vs EVERYTHING present when they are driven
    worst = 0.0
    for hx, hy in fork_screw_positions():
        col = Pos(hx, hy, (H_HORN_TOP + HUB_T + 60) / 2) * Cylinder(1.9, 60 - (H_HORN_TOP + HUB_T))
        worst = max(worst, _v(col & fork), _v(col & base))
    check("horn-screw driver access (fork+base)", worst, lambda v: v < 1, "CLEAR", "BLOCKED")
    # assembly order: each part arrives along +Z with nothing in the way
    worst = 0.0
    for dz in (40, 20, 10, 5, 1):
        worst = max(worst, _v((Pos(0, 0, dz) * servo_body(P)) & base))
    check("yaw servo drop-in path (+Z)", worst, lambda v: v < 1, "OPEN", "BLOCKED")
    worst = 0.0
    for dz in (20, 10, 5, 1):
        worst = max(worst, _v((Pos(0, 0, dz) * fork) & (base + servo_body(P))))
    check("fork drop-on path (+Z)", worst, lambda v: v < 1, "OPEN", "BLOCKED")
    worst = 0.0
    for dz in (20, 10, 5, 1):
        worst = max(worst, _v((Pos(0, 0, dz) * cap) & (base + fork)))
    check("cap drop-on path (+Z)", worst, lambda v: v < 1, "OPEN", "BLOCKED")
    # retention: a 2 mm nudge of the femur servo must meet material
    fs = femur_servo_placed()
    check("femur servo nudged -Y (toward link) x fork", _v((Pos(0, -2, 0) * fs) & fork),
          lambda v: v > 1, "RETAINED by lips", "FREE")
    check("femur servo nudged +Z x strap", _v((Pos(0, 0, 2) * fs) & strap),
          lambda v: v > 1, "RETAINED by strap", "FREE")
    # axle: thread length in the boss + cap pocket over the boss
    print(f"axle M3 thread-forming length: {Z_BOSS_TOP - HEAD_POCKET_Z1:.2f} mm; "
          f"cap bearing pocket z {CAP_Z0:.2f}..{CAP_Z0 + B['w'] + 0.2:.2f}; boss top {Z_BOSS_TOP:.2f}")
    print(f"part_coxa checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails:
        sys.exit(1)
