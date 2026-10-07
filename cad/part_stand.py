"""Bench/maintenance stand — legs swing free, joints are printed I6.

The missing bench-day enabler: pose_check, calibrate_centers and the
torque-step all want the robot held with UNLOADED legs. Three parts, one
joint: every stack interface is the same spigot joint — the LOWER part's
top 16 mm carries five vertical I6 male bars on its pentagon facets, the
UPPER part's skirt drops over them with female dovetail channels (FIT
clearance). Gravity closes the joint; the trapezoids carry shear/moment.

  stand_base    — floor plate + pentagon tube stub, males on top (z 35)
  stand_section — pentagon tube H=80: skirt below, males on top.
                  Print 0/1/2: deck-bottom heights 89.4 / 169.4 / 249.4
                  (the legs reach 153.3 below the deck bottom, 183.5 with
                  the B95 hand, the proposal's 172 between: two sections
                  free the whole envelope, one does not)
  stand_crown   — the U-CRADLE (2026-10-07: B85, option A = pick 1). It
                  holds the robot by its keel tub (iface.bay_tub_extent)
                  and touches nothing else, so the carapace stays on and
                  both tub ends are free: the sled swaps on the stand. On
                  today's skirt: a seat ring, the top plate, a floor
                  (params stand.cradle_x, +-45) under the tub's bottom, two
                  walls wall_h (25) tall, tub_clear (0.5) outside the tub's
                  y faces, open at both x ends, and an x stop: a pin up from
                  the floor into a blind hole in the tub floor (the cradle
                  alone holds the tub in y only; proposal s3 risks). No arms:
                  the old deck-edge saddles at az 54 / 126 would run through
                  the hub shelf, and the tub fills the belly they spanned.

Four things the proposal's cradle did not have (2026-10-07, measured here):
  * the SEAT RING. The old crown had none: its plate's underside sat 3 mm
    over the lower part's male tops, so it dropped to 32 and every
    'deck-bottom 47 / 127 / 207' was really 44 / 124 / 204 (measured: 0 mm^3
    against the base from seat 35 down to 32.0, 97 mm^3 at 31.9). The crown
    now carries stand_section's ring and lands at 35 (binds at 34.9).
  * the PLATE RELIEF. Outside the floor the top plate stands PLATE_RELIEF
    lower, so the floor is the only face the tub rests on and the hub
    shelf's face-down boards (bottom_z -54.4) keep 4.00 to the plate, not
    1.00 (B134: that 1.00 goes with +1.0 of plate height or a 1.42 deg
    roll, and the cradle's walls allow 2.00 with the pin, 2.49 without).
    Relieved, +1.0 leaves 3.00 and the shelf would need 5.68 deg north (2.79
    south, the pin's play taken): the walls stop every roll first. The shelf's nearest crown face is now the north
    wall's outside, 1.30 beside it (1.00 slid by the pin's play, 0.87 after
    a knock south, 1.04 at the north wall's give under B134's CoM).
  * the SOUTH WALL'S ENDS stop at x +-40.5: at +-45 its outer corners ran
    into legs 2 / 3's drops + 5 (iface.leg_drop_keepout(5): 35 + 5 deep,
    which the 25 walls reach). _wall_x derives the end from the keep-out.
  * a 2 mm LEAD-IN on the walls' inner top edges: with the carapace on,
    the tub goes into its 0.5 slot unseen.

Checks (__main__, the robot posed in the cradle in the body frame): crown
x tub (+ bosses, latch pad, hole), shelf, deck, docked bases, hook
envelopes 'any', drops + 5, lanes, carapace, sled path, both tub ends =
0 mm^3 with the gaps; registration (tub +-1 in y binds on the walls, +-1 in
x on the pin, 0 free); the pin's play and engagement; the shelf margin at
+1.0 of plate height (either plate) and the roll the cradle allows (both
ways) vs the roll that closes it; the north wall as the tipping stop B134's
CoM offset makes it (load, stress, give: an ESTIMATE); the heights; the
legs' reach (below the deck, and on the stand into the tub, the cradle and
the stand's column); the stack joints (aligned slides, 36 deg misaligned
registers, 0.1 low binds); bed fit, print orientation.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import (IF, dovetail_male, bay_tub_extent, body_keepouts, STATIONS, R_STATION,
                   CABLE_CUTOUT_X, DROP_HALF, DECK_BOT_Z)

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
DV = IF["dovetail"]
ST = IF["stand"]
BS = IF["battery_sled"]
HS = IF["hub_shelf"]

APO = 40.0                       # pentagon tube apothem (facet radius)
WALL_T = 3.0
SEC_H = 80.0
ENG = 16.0                       # male bar length / joint engagement
FACETS = [18 + 72 * k for k in range(5)]
SKIRT_T = DV["depth"] + FIT + 2.6            # wall swallows crest + meat
SEAT_Z = 35.0                    # the base's male tops (plate 5 + tube 30): the next seat lands here
SEAT_T = 3.0                     # the seat ring (stand_section's): the hard stop on the male tops
PLATE_T = 6.0                    # the crown's plate over the ring = the floor's thickness
PLATE_RELIEF = 3.0               # the plate outside the floor stands this much lower (B134)
FLOOR_Z = SEAT_T + PLATE_T       # 9: the cradle floor = the tub's bottom, crown frame
# the tub as params place it, read when this module loads: bay_l is PROVISIONAL (pick 4: the
# carrier printed into the nose wall brings the nose in ~10), so the nose x is a run-time number
TUB = bay_tub_extent()
BODY_DZ = FLOOR_Z - TUB["z"][0]  # crown z = body z + 64.4; the stand's axis under the body origin
DROP_EXTRA = 5.0                 # the drop keep-out the cradle clears: iface.leg_drop_keepout(5)
WALL_DROP_GAP = 0.5              # ... by this much at a wall's end
PIN_CHAMFER = 0.3                # the x stop's lead-in (45 deg) at its tip
WALL_LEAD_IN = 2.0               # 45-deg chamfer on each wall's inner top edge: with the carapace on
                                 # the tub goes into its 0.5 slot unseen, so the walls catch it +-2.5
LATCH_PAD = (14.6, 14.6, 6.5)    # the door latch's strike boss, y x z x along-x (proposal s2:
                                 # 'outside the south wall, x -94.5 .. -88', pick 12; part_bay's)
HAND_SOLE = 165.2                # the B95 hand on the shortest tube: sole below the knee axis
                                 # (PREP_REPORT_2, B131; one agent, unverified)
COM_PAST_EDGE = 17.0             # B134: joint-box poses put the CoM 15-17 north of the tub's
                                 # north-bottom edge (K3 s7_roll_com, unverified): then the robot
                                 # tips north and the north wall is what holds it


def _pent(r_apo, rot=54):
    return RegularPolygon(r_apo / np.cos(np.pi / 5), 5, rotation=rot)


def _tube(h, z0=0.0, apo=APO, wall=WALL_T):
    t = Pos(0, 0, z0) * extrude(_pent(apo), h)
    t -= Pos(0, 0, z0 - 1) * extrude(_pent(apo - wall), h + 2)
    return t


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _males(z_top):
    out = None
    for az in FACETS:
        m = Rot(0, 0, az) * Pos(APO - 1.2, 0, z_top - ENG / 2) * \
            Rot(0, 90, 0) * dovetail_male(length=ENG)
        out = m if out is None else out + m
    return out


def _male_neg(L):
    b = DV["base_w"] + 2 * FIT
    c = DV["crest_w"] + 2 * FIT
    d = DV["depth"] + FIT
    with BuildPart() as bp:
        with BuildSketch(Plane.YZ):
            with BuildLine():
                Polyline((-b / 2, -0.3), (b / 2, -0.3), (c / 2, d),
                         (-c / 2, d), (-b / 2, -0.3))
            make_face()
        extrude(amount=L / 2, both=True)
    return bp.part


def _skirt(z_seat=0.0):
    """Skirt from z_seat-18 to z_seat+3, hugging a lower tube (outer facet
    APO) with FIT, female channels swallowing its males."""
    sk = _tube(21.0, z0=z_seat - 18.0, apo=APO + FIT + SKIRT_T,
               wall=SKIRT_T)
    for az in FACETS:
        sk -= Rot(0, 0, az) * Pos(APO - 1.2, 0, z_seat - 9.0) * \
            Rot(0, 90, 0) * _male_neg(26.0)
    return sk


def _seat():
    """The seat ring z 0..SEAT_T: ties a part's tube (or plate) to its skirt, and is the hard
    stop the lower part's male bars land against (their tops sit at z 0)."""
    return extrude(_pent(APO + FIT + SKIRT_T), SEAT_T) - \
        Pos(0, 0, -1.0) * extrude(_pent(APO - WALL_T), SEAT_T + 2.0)


def stand_base():
    plate = extrude(_pent(52), 5) + Pos(0, 0, 0) * extrude(_pent(52), 5)
    plate = extrude(RegularPolygon(85, 5, rotation=54), 5)
    tube = _tube(30.0, z0=5.0)
    return plate + tube + _males(35.0)


def stand_section():
    # v0.1 (session 8): the skirt's inner facet is APO + FIT — it hugs the
    # LOWER tube, so it never touched THIS part's tube (0.3 mm gap all
    # round) and the section exported as two bodies (D036 class; the crown
    # was fine only because its top plate happens to bridge the gap). A
    # 3 mm seat ring z 0..3 now ties tube and skirt — it is also the hard
    # stop the lower part's male bars land against (their tops sit at z 0).
    return _tube(SEC_H, z0=0.0) + _skirt(0.0) + _seat() + _males(SEC_H)


# ---------------------------------------------------------------- the cradle
def _cradle_y():
    """The two walls' faces, body y: (south out, south in, north in, north out). The tub's
    outer y faces (bay_tub_extent: y -47.4 / 7.4) + tub_clear, + wall_t."""
    c, t = ST["tub_clear"], ST["wall_t"]
    y0, y1 = TUB["y"]
    return y0 - c - t, y0 - c, y1 + c, y1 + c + t


def _wall_x(y_faces, gap=WALL_DROP_GAP):
    """The x span a wall between the faces y_faces may run: params cradle_x, cut back where a
    leg drop's keep-out (iface.leg_drop_keepout(DROP_EXTRA): z -10 .. -50, and the walls' tops
    are at -30.4) would come within `gap`. Plan only, each drop as the half-plane
    of its inboard face r = R_STATION + CABLE_CUTOUT_X - (DROP_HALF + DROP_EXTRA) about its
    station (conservative: it ignores the faces' 15.5 half-width); rounded 0.1 inward. Legs 2 /
    3 cut the south wall to +-40.5; nothing reaches the north wall."""
    x0, x1 = ST["cradle_x"]
    r_in = R_STATION + CABLE_CUTOUT_X - (DROP_HALF + DROP_EXTRA) - gap
    for a in np.deg2rad(STATIONS):
        c, s = np.cos(a), np.sin(a)
        for y in y_faces:
            if abs(c) < 1e-9:                    # a face parallel to the wall: in or out whole
                assert y * s <= r_in, "a wall face inside a drop keep-out"
                continue
            xb = (r_in - y * s) / c              # where this face line meets the grown face
            if c < 0:
                x0 = max(x0, np.ceil(xb * 10) / 10)
            else:
                x1 = min(x1, np.floor(xb * 10) / 10)
    assert x1 > x0, "no wall left between the drops"
    return x0, x1


def xstop_pin():
    """The x stop, crown frame: Ø xstop_pin_d - 2 FIT, xstop_hole_depth - FIT tall on the floor at
    xstop_xy, under the tub floor's Ø xstop_pin_d x xstop_hole_depth blind hole (part_bay cuts
    it): FIT of radial play, as every printed fit in this tree, and its tip FIT under the hole's
    ceiling, so the floor carries the robot, never the pin. A PIN_CHAMFER lead-in on the tip.
    Rooted 0.5 into the floor (no coplanar union)."""
    r = ST["xstop_pin_d"] / 2 - FIT
    h = ST["xstop_hole_depth"] - FIT
    x, y = ST["xstop_xy"]
    pin = Pos(x, y, FLOOR_Z + (h - PIN_CHAMFER - 0.5) / 2) * Cylinder(r, h - PIN_CHAMFER + 0.5)
    pin += Pos(x, y, FLOOR_Z + h - PIN_CHAMFER / 2) * Cone(r, r - PIN_CHAMFER, PIN_CHAMFER)
    return pin


def cradle_parts(relief=PLATE_RELIEF):
    """The crown's pieces, crown frame (x, y = body): skirt, seat (ring), plate, floor, walls
    [south, north], pin. The walls root in the floor (z SEAT_T up) and rise wall_h over it."""
    ys0, ys1, yn0, yn1 = _cradle_y()
    x0, x1 = ST["cradle_x"]
    zt = FLOOR_Z + ST["wall_h"]
    s = WALL_LEAD_IN * np.sqrt(2.0)                       # a square turned 45 deg on the edge
    walls = []
    for wy0, wy1, y_in in ((ys0, ys1, ys1), (yn0, yn1, yn0)):
        wx0, wx1 = _wall_x((wy0, wy1))
        w = _box(wx0, wx1, wy0, wy1, SEAT_T, zt)
        w -= Pos((wx0 + wx1) / 2, y_in, zt) * Rot(45, 0, 0) * Box(wx1 - wx0 + 2, s, s)
        walls.append(w)
    return {"skirt": _skirt(0.0), "seat": _seat(),
            "plate": Pos(0, 0, SEAT_T) * extrude(_pent(APO + FIT + SKIRT_T), PLATE_T - relief),
            "floor": _box(x0, x1, ys0, yn1, SEAT_T, FLOOR_Z), "walls": walls, "pin": xstop_pin()}


def stand_crown(relief=PLATE_RELIEF, pin=True):
    """The U-cradle (B85). For the checks: relief 0 is the proposal's crown with a full-height
    plate (the 1.00 to the shelf B134 measured), pin=False the cradle with no x stop."""
    p = cradle_parts(relief)
    crown = p["skirt"] + p["seat"] + p["plate"] + p["floor"]
    for w in p["walls"]:
        crown += w
    return crown + p["pin"] if pin else crown


# ---------------------------------------------------------------- what the cradle holds and meets
def to_body(s):
    """Crown frame -> body frame, the robot standing in the cradle (the tub's bottom on the floor)."""
    return Pos(0, 0, -BODY_DZ) * s


def tub_model(dx=0.0, dy=0.0):
    """The keel tub as the stand meets it, body frame, shifted (dx, dy). part_bay builds the real
    one; this is params + the proposal: bay_tub_extent()'s outer box (the door_t door at -x
    included), the nose's two 45-deg nose_chamfer plan chamfers, the door latch's strike boss
    (LATCH_PAD outside the south wall from the door's inner face; set at the tub's bottom, the
    nearest the cradle could be), the six Ø TUB_BOSS_D lid bosses from the roof top to the deck
    bottom at part_deck.DECK_HOLES' hanger xy, and the x stop's blind hole in the floor."""
    from part_deck import DECK_HOLES, TUB_BOSS_D
    (x0, x1), (y0, y1), (z0, z1) = TUB["x"], TUB["y"], TUB["z"]
    tub = _box(x0, x1, y0, y1, z0, z1)
    c = BS["nose_chamfer"]
    for yc, sy in ((y0, 1), (y1, -1)):                    # the corner beyond the 45-deg line
        tri = make_face(Polyline((x1 - c - 1, yc - sy, z0 - 1), (x1 + 1, yc - sy, z0 - 1),
                                 (x1 + 1, yc + sy * (c + 1), z0 - 1), close=True))
        tub -= extrude(tri, amount=z1 - z0 + 2, dir=(0, 0, 1))
    ly, lz, lx = LATCH_PAD
    tub += _box(TUB["x_in"][0], TUB["x_in"][0] + lx, y0 - ly, y0 + 0.5, z0, z0 + lz)
    for name, (bx, by), *_ in DECK_HOLES:
        if name.startswith(("tray_tab_tub_", "tub_hanger_")):
            tub += Pos(bx, by, (z1 - 0.5 + DECK_BOT_Z) / 2) * Cylinder(TUB_BOSS_D / 2, DECK_BOT_Z - z1 + 0.5)
    hx, hy = ST["xstop_xy"]
    hd = ST["xstop_hole_depth"]
    tub -= Pos(hx, hy, z0 + (hd - 1.0) / 2) * Cylinder(ST["xstop_pin_d"] / 2, hd + 1.0)
    return Pos(dx, dy, 0) * tub


def shelf_model():
    """The hub shelf as params hub_shelf gives it (part_busboard builds the real one), body frame:
    its plan box (the plate as packed + the five boards' footprints and plug / lead halos) from
    bottom_z (the face-down adapter's pins) to plate_top_z, and a Ø14 pad round each post from
    bottom_z to the deck bottom (post, screw head from below)."""
    (x0, x1), (y0, y1) = HS["x"], HS["y"]
    s = _box(x0, x1, y0, y1, HS["bottom_z"], HS["plate_top_z"])
    for px, py in HS["posts"]:
        s += Pos(px, py, (HS["bottom_z"] + DECK_BOT_Z) / 2) * Cylinder(7.0, DECK_BOT_Z - HS["bottom_z"])
    return s


def lane_models():
    """The leg 2 / 3 looms over the tub (params hub_shelf.lanes: A, B, C for +x, mirrored)."""
    ln = HS["lanes"]
    out = []
    for xs, ys in ln["route"]:
        for m in (1, -1):
            xa, xb = sorted((m * xs[0], m * xs[1]))
            out.append(_box(xa, xb, ys[0], ys[1], *ln["z"]))
    return out


def _vol(a, b):
    v = a & b
    return 0.0 if v is None else v.volume


def _roll(s, deg, pivot):
    """s rolled `deg` about the x-parallel axis through pivot (y, z); deg > 0 = north side down."""
    py, pz = pivot
    return Pos(0, py, pz) * Rot(-deg, 0, 0) * Pos(0, -py, -pz) * s


def _first_contact(moving, fixed, pivot, hi=12.0, tol=0.005, sign=1):
    """The smallest roll (deg) about pivot at which `moving` meets `fixed` (> 1e-3 mm^3), by
    bisection; north side down (sign -1: south side down); None if it is still clear at `hi`."""
    hit = lambda d: _vol(_roll(moving, sign * d, pivot), fixed) > 1e-3
    if not hit(hi):
        return None
    lo = 0.0
    while hi - lo > tol:
        mid = (lo + hi) / 2
        lo, hi = (lo, mid) if hit(mid) else (mid, hi)
    return hi


def robot_mass_g():
    """The robot on the stand, g: sim/mass_budget.json's torso (battery and electronics in it)
    + five legs with their hands. The tub, lid, door and shelf are not in that budget yet (the
    proposal's A torso is ~30 g heavier)."""
    import json
    import os
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim",
                           "mass_budget.json")) as f:
        mb = json.load(f)
    return mb["torso"] + 5 * (mb["coxa"] + mb["femur"] + mb["tibia"] + mb["hand"])


def north_wall_stop(d_com=COM_PAST_EDGE):
    """The north wall as the tipping stop (B134): with the CoM d_com north of the tub's
    north-bottom edge the robot rolls onto the wall, which takes F = W d_com / h at the foot of
    its lead-in (h over the floor, the pivot's height) along its whole length. The wall as a
    cantilever from the floor: root bending stress (across the layers: it prints upright) and
    the give at h, as extra roll. params fem.petg allowables (PLA is stiffer and stronger: this
    is the conservative side); static, friction and the roll's own CoM shift left out. Returns
    (W N, F N, sigma MPa, SF across the layers, give mm, extra roll deg)."""
    pg = P["fem"]["petg"]
    w = robot_mass_g() / 1000.0 * 9.81
    h = ST["wall_h"] - WALL_LEAD_IN
    b, t = np.diff(_wall_x(_cradle_y()[2:]))[0], ST["wall_t"]
    f = w * d_com / h
    sig = 6.0 * f * h / (b * t * t)
    give = f * h ** 3 / (3.0 * pg["E_mpa"] * b * t ** 3 / 12.0)
    return w, f, sig, pg["strength_z_mpa"] / sig, give, np.rad2deg(np.arctan2(give, h))


def leg_depth_below_deck(sole=None, step=0.25):
    """The deepest the leg's sole gets under the deck bottom over the soft-limit box (params
    joints.pos_deg; the chain of params robot.leg: hip axis hip_axis_z over leg z 0, femur l2,
    then `sole` along the tibia, default l3 = the IK foot point, which the foot's contact sphere
    surface IS)."""
    L, J = P["leg"], P["joints"]["pos_deg"]
    sole = L["l3_tibia"] if sole is None else sole
    h = np.deg2rad(np.arange(J["hip"][0], J["hip"][1] + 1e-9, step))
    k = np.deg2rad(np.arange(J["knee"][0], J["knee"][1] + 1e-9, step))
    H, K = np.meshgrid(h, k)
    z = L["hip_axis_z"] + L["l2_femur"] * np.sin(H) + sole * np.sin(H + K)
    return DECK_BOT_Z - z.min()


def legs_into_cradle(step=5.0, r_femur=15.0, r_tibia=12.0, n_sections=2):
    """On the stand the legs sweep their soft-limit box. Which poses put a link capsule (the
    proposal's r 15 / 12, along the femur and tibia axes) into the cradle's walls or floor, which
    into the tub (B97), and which into the stand's column under the plate (skirt, ring, sections:
    a pentagonal prism of the skirt's outer apothem from the relieved plate's top down to the
    bench under n_sections; the plan distance is the facets' max, which never overstates it)?
    Boxes from the cradle's own numbers. Per leg: (tub poses, cradle poses, cradle-not-tub
    poses, the cradle poses' hip range, column poses, column-not-tub poses, the column poses'
    (hip, knee) ranges)."""
    L, J = P["leg"], P["joints"]["pos_deg"]
    ys0, ys1, yn0, yn1 = _cradle_y()
    z0 = TUB["z"][0]
    zw = (z0, z0 + ST["wall_h"])
    boxes = [(_wall_x((ys0, ys1)), (ys0, ys1), zw), (_wall_x((yn0, yn1)), (yn0, yn1), zw),
             (tuple(ST["cradle_x"]), (ys0, yn1), (z0 - PLATE_T, z0))]
    tub = (TUB["x"], TUB["y"], TUB["z"])
    col_apo = APO + FIT + SKIRT_T
    col_z = (z0 - FLOOR_Z - SEAT_Z - n_sections * SEC_H, z0 - PLATE_RELIEF)
    col_n = np.array([[np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a))] for a in FACETS])

    def dist(p, b):
        lo = np.array([b[0][0], b[1][0], b[2][0]])
        hi = np.array([b[0][1], b[1][1], b[2][1]])
        return np.linalg.norm(np.maximum(np.maximum(lo - p, 0), p - hi), axis=-1)

    def dist_col(p):
        dr = np.maximum((p[..., :2] @ col_n.T).max(-1) - col_apo, 0)
        dz = np.maximum(np.maximum(p[..., 2] - col_z[1], col_z[0] - p[..., 2]), 0)
        return np.hypot(dr, dz)

    ax = [np.deg2rad(np.arange(J[j][0], J[j][1] + 1e-9, step)) for j in ("yaw", "hip", "knee")]
    Y, H, K = np.meshgrid(*ax, indexing="ij")
    cy, sy = np.cos(Y), np.sin(Y)
    hip = np.stack([L["l1_coxa"] * cy, L["l1_coxa"] * sy, L["hip_axis_z"] + 0 * Y], -1)
    knee = hip + L["l2_femur"] * np.stack([np.cos(H) * cy, np.cos(H) * sy, np.sin(H)], -1)
    foot = knee + L["l3_tibia"] * np.stack([np.cos(H + K) * cy, np.cos(H + K) * sy, np.sin(H + K)], -1)
    s = np.linspace(0, 1, 15)[:, None]
    out = []
    for a in np.deg2rad(STATIONS):
        R = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        hb, kb, fb = [(p + [R_STATION, 0, 0]) @ R.T for p in (hip, knee, foot)]
        fem = hb[..., None, :] + s * (kb - hb)[..., None, :]
        tib = kb[..., None, :] + s * (fb - kb)[..., None, :]
        meets = lambda b: (dist(fem, b).min(-1) < r_femur) | (dist(tib, b).min(-1) < r_tibia)
        m_tub = meets(tub)
        m_cr = np.zeros_like(m_tub)
        for b in boxes:
            m_cr |= meets(b)
        m_col = (dist_col(fem).min(-1) < r_femur) | (dist_col(tib).min(-1) < r_tibia)
        hs = np.rad2deg(H[m_cr])
        hc, kc = np.rad2deg(H[m_col]), np.rad2deg(K[m_col])
        out.append((int(m_tub.sum()), int(m_cr.sum()), int((m_cr & ~m_tub).sum()),
                    (hs.min(), hs.max()) if hs.size else None,
                    int(m_col.sum()), int((m_col & ~m_tub).sum()),
                    ((hc.min(), hc.max()), (kc.min(), kc.max())) if hc.size else None))
    return out, Y.size


if __name__ == "__main__":
    from part_deck import body_deck
    from part_coxa import coxa_yaw_base
    from part_shell import shell_sector, shell_cap
    from iface import station_tf
    ok = True

    def verdict(good, yes="OK", no="FAIL"):
        global ok
        ok &= bool(good)
        return yes if good else no

    parts = dict(stand_base=stand_base(), stand_section=stand_section(),
                 stand_crown=stand_crown())
    for n, p in parts.items():
        export(p, n)                                      # D036: raises unless ONE solid
        bb = p.bounding_box()
        fits = bb.size.X <= PR["bed_mm"][0] and bb.size.Y <= PR["bed_mm"][1] and \
            bb.size.Z <= PR["bed_mm"][2]
        print(f"  {n}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, "
              f"{p.volume / 1000:.1f} cm^3, {len(p.solids())} solid "
              f"({verdict(fits, 'fits the bed', 'TOO BIG for the bed')})")
    crown_c = parts["stand_crown"]
    cp = cradle_parts()
    crown = to_body(crown_c)
    walls = [to_body(w) for w in cp["walls"]]
    pin = to_body(cp["pin"])
    ys0, ys1, yn0, yn1 = _cradle_y()
    sx, nx = _wall_x((ys0, ys1)), _wall_x((yn0, yn1))
    print(f"  the tub (iface.bay_tub_extent at run time): x {TUB['x'][0]:.1f}..{TUB['x'][1]:.1f} "
          f"(door face .. nose; bay_l {BS['bay_l']} PROVISIONAL), y {TUB['y'][0]:.1f}..{TUB['y'][1]:.1f}, "
          f"z {TUB['z'][0]:.1f}..{TUB['z'][1]:.1f}")
    print(f"  cradle (body): floor x {ST['cradle_x'][0]:.1f}..{ST['cradle_x'][1]:.1f}, y {ys0:.1f}..{yn1:.1f}, "
          f"top z {TUB['z'][0]:.1f}; walls z {TUB['z'][0]:.1f}..{TUB['z'][0] + ST['wall_h']:.1f}: south y "
          f"{ys0:.1f}..{ys1:.1f} x {sx[0]:.1f}..{sx[1]:.1f}, north y {yn0:.1f}..{yn1:.1f} x {nx[0]:.1f}..{nx[1]:.1f}; "
          f"plate outside the floor top z {SEAT_T + PLATE_T - PLATE_RELIEF - BODY_DZ:.1f}")

    # ---- the robot standing in the cradle: the crown touches the tub and nothing else
    tub = tub_model()
    shelf = shelf_model()
    deck = Pos(0, 0, DECK_BOT_Z) * body_deck()
    bases = [station_tf(i) * coxa_yaw_base() for i in range(len(STATIONS))]
    ko5, ko0 = body_keepouts(DROP_EXTRA, "any"), body_keepouts(0.0, "any")
    sector, cap = shell_sector(), shell_cap()
    shell = [Rot(0, 0, a) * sector for a in STATIONS] + [Rot(0, 0, STATIONS[0]) * cap]
    (tx0, tx1), (ty0, ty1), (tz0, tz1) = TUB["x"], TUB["y"], TUB["z"]
    xi, yi, zi = TUB["x_in"], TUB["y_in"], TUB["z_in"]
    sled_path = _box(xi[0] - 200.0, xi[1], yi[0], yi[1], zi[0], zi[1])    # the sled out -x, door off
    ey0 = ty0 - LATCH_PAD[0]
    ends = [_box(tx0 - 10.0, tx0, ey0, ty1, tz0, tz1), _box(tx1, tx1 + 10.0, ey0, ty1, tz0, tz1)]
    groups = [("tub (+ lid bosses, latch pad, x-stop hole)", [tub]),
              ("hub shelf (params box + Ø14 post pads)", [shelf]),
              ("deck (body_deck at z -10..-4)", [deck]),
              ("docked coxa bases x5", bases),
              ("hook envelopes 'any' x5", ko5["hooks"]),
              (f"drops + {DROP_EXTRA:.0f} x5", ko5["drops"]),
              ("drops + 0 x5", ko0["drops"]),
              ("lanes (legs 2 / 3) x6", lane_models()),
              ("carapace (5 sectors + cap, posed as part_shell's check)", shell),
              ("sled path (-x, door off, 200 out)", [sled_path]),
              ("door end + nose end (10 each, the tub's height)", ends)]
    print("  crown x (the robot standing in the cradle):")
    for name, solids in groups:
        v = sum(_vol(crown, s) for s in solids)
        g = min(crown.distance_to(s) for s in solids)
        print(f"    {name}: {v:.4f} mm^3, closest {g:.3f} ({verdict(v < 1e-3, 'CLEAR', 'CLASH')})")
    g_w = min(w.distance_to(tub) for w in walls)
    g_pin = pin.distance_to(tub)
    g_sw = min(w.distance_to(shelf) for w in walls)
    g_plate = to_body(cp["plate"]).distance_to(shelf)
    print(f"    the walls to the tub {g_w:.3f} ({verdict(abs(g_w - ST['tub_clear']) < 1e-3)}: tub_clear "
          f"{ST['tub_clear']}); the pin to the tub {g_pin:.3f}; the floor carries it (touching)")
    print(f"    the shelf: {g_plate:.3f} to the relieved plate (face-down boards at {HS['bottom_z']}), "
          f"{g_sw:.3f} to the north wall's outer face (the shelf box from y {HS['y'][0]}), "
          f"{g_sw - FIT:.2f} with the robot slid south by the pin's play: wall_t may grow that much")
    g_d5 = min(w.distance_to(d) for w in walls for d in ko5["drops"])
    print(f"    the south wall's ends x {sx[0]:.1f} / {sx[1]:.1f} to legs 2 / 3's drops + {DROP_EXTRA:.0f}: "
          f"{g_d5:.3f} ({verdict(g_d5 >= WALL_DROP_GAP - 1e-3)}; at x +-{ST['cradle_x'][1]:.0f} they ran in)")
    cbb = crown.bounding_box()
    print(f"    crown x extent {cbb.min.X:.2f}..{cbb.max.X:.2f}: the tub's ends {tx0:.1f} / {tx1:.1f} keep "
          f"{cbb.min.X - tx0:.1f} / {tx1 - cbb.max.X:.1f} (the sled swaps on the stand)")

    # ---- registration: the walls hold y, the pin holds x; the play
    print("  registration (the tub shifted in the cradle):")
    for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
        t = tub_model(dx, dy)
        vw, vp = sum(_vol(w, t) for w in walls), _vol(pin, t)
        want = (dx == 0 and dy == 0 and vw + vp < 1e-3) or (dx != 0 and vp > 1e-3) or \
            (dy != 0 and vw > 1e-3)
        print(f"    ({dx:+d}, {dy:+d}): walls {vw:8.2f}, pin {vp:6.2f} mm^3 "
              f"({verdict(want, 'FREE' if dx == dy == 0 else 'BINDS', 'WRONG')})")
    play = FIT
    for d in (play - 0.05, play + 0.05):
        vx = max(_vol(pin, tub_model(s * d, 0)) for s in (1, -1))
        vy = max(_vol(pin, tub_model(0, s * d)) for s in (1, -1))
        good = (vx < 1e-3 and vy < 1e-3) if d < play else (vx > 1e-4 and vy > 1e-4)
        print(f"    +-{d:.2f} on the pin: x {vx:.4f}, y {vy:.4f} mm^3 "
              f"({verdict(good, 'free' if d < play else 'binds')})")
    h_pin = ST["xstop_hole_depth"] - FIT
    print(f"    the x stop: Ø{ST['xstop_pin_d'] - 2 * FIT:.1f} x {h_pin:.1f} pin in the tub's "
          f"Ø{ST['xstop_pin_d']:.1f} x {ST['xstop_hole_depth']:.1f} hole: play +-{play:.2f} in x and y "
          f"(the walls allow +-{ST['tub_clear']:.2f}), engaged {h_pin:.1f} ({h_pin - PIN_CHAMFER:.1f} "
          f"straight + {PIN_CHAMFER} lead-in), tip {ST['xstop_hole_depth'] - h_pin:.1f} under the ceiling; "
          f"it holds x until the robot is lifted {h_pin - PIN_CHAMFER:.1f}, then the lead-in cams it out")

    # ---- the shelf margin at +1.0 of plate height (B134), either plate
    crown_full = to_body(stand_crown(relief=0.0))
    crown_nopin = to_body(stand_crown(pin=False))
    g_full = to_body(cradle_parts(0.0)["plate"]).distance_to(shelf)
    g_up1 = to_body(cradle_parts(PLATE_RELIEF - 1.0)["plate"]).distance_to(shelf)
    shelf_dn1 = Pos(0, 0, -1.0) * shelf                   # the shelf's plate_t + 1: the boards 1 lower
    g_dn1, g_dn1_plate = crown.distance_to(shelf_dn1), to_body(cp["plate"]).distance_to(shelf_dn1)
    ok_up1 = g_up1 > 1e-3 and g_dn1 > 1e-3
    print(f"  the shelf at +1.0 of plate height: the crown's plate 1.0 higher leaves {g_up1:.2f} to the "
          f"shelf; the shelf's plate 1.0 thicker (boards to {HS['bottom_z'] - 1.0:.1f}) leaves {g_dn1_plate:.2f} "
          f"to the plate and {g_dn1:.2f} to the whole crown (the north wall, beside it: +1.0 down does not "
          f"move that one) ({verdict(ok_up1)}); a full-height plate: {g_full:.2f}, gone at +1.0")

    # ---- the roll the cradle allows vs the roll that closes the shelf gap (B134)
    print("  roll, north side down (the shelf side; pivot on the tub's north-bottom edge):")
    rows = []
    # the slide first (the robot shifted south, so the edge it tips on is further from the
    # wall): 0 = the CoM goes north of the edge in place (joint-box poses, B134: 15-17 past it);
    # FIT = the pin's play; tub_clear = the walls' alone (K3's 2.29 deg: a cradle with no pin)
    for tag, slide, cr in (("tipping on the edge in place", 0.0, crown),
                           ("slid south by the pin's play first", play, crown),
                           ("no pin, slid south by the walls' clearance", ST["tub_clear"], crown_nopin)):
        pv = (ty1 - slide, tz0)
        sh = Pos(0, -slide, 0) * shelf
        allow = _first_contact(tub_model(0, -slide), cr, pv)
        th_s = _first_contact(sh, cr, pv)
        th_f = _first_contact(sh, crown_full, pv)
        g_at = _roll(sh, allow, pv).distance_to(cr)
        rows.append((allow, th_s, th_f))
        print(f"    {tag}: the walls stop it at {allow:.2f} deg; the shelf would meet this crown at "
              f"{th_s if th_s is None else round(th_s, 2)} deg ({g_at:.2f} left at {allow:.2f}), a "
              f"full-height plate at {th_f:.2f} deg")
    ok_roll = all(a < s for a, s, _ in rows if s is not None)
    print(f"    shelf gap to the plate: {g_plate:.2f}; a full-height plate {g_full:.2f} closes at "
          f"{min(r[2] for r in rows):.2f} deg, under the walls' {max(r[0] for r in rows):.2f} (B134's "
          f"1.42): here the walls always stop it first ({verdict(ok_roll)})")
    # south side down: nothing static gets the CoM there (it rests ~45 north of that edge), a
    # knock could; the shelf's south face (outside the north wall) then swings south, toward the
    # wall's outer face. A slide south first brings the shelf nearer and leaves less roll; north,
    # more roll and the shelf further off (0.90 at 2.00 with the pin's play): south governs
    print("  roll, south side down (a knock; pivot on the tub's south-bottom edge):")
    for tag, slide, cr, gate in (("in place", 0.0, crown, True),
                                 ("slid south by the pin's play first", play, crown, True),
                                 ("no pin, slid south by the walls' clearance", ST["tub_clear"],
                                  crown_nopin, False)):
        pv = (ty0 - slide, tz0)
        sh = Pos(0, -slide, 0) * shelf
        allow_s = _first_contact(tub_model(0, -slide), cr, pv, sign=-1)
        th_ss = _first_contact(sh, cr, pv, sign=-1)
        sh_r = _roll(sh, -allow_s, pv)
        g_ss, g_nw = sh_r.distance_to(cr), sh_r.distance_to(walls[1])
        res = verdict(g_ss > 1e-3) if gate else ("clear" if g_ss > 1e-3 else "touches: the pin is why")
        print(f"    {tag}: the walls stop it at {allow_s:.2f} deg, the shelf keeps {g_ss:.2f} to the "
              f"crown there ({g_nw:.2f} to the north wall; it would meet the crown at "
              f"{th_ss if th_ss is None else round(th_ss, 2)} deg) ({res})")

    # ---- the north wall is the tipping stop (B134's CoM past the edge): load, stress, give
    w_n, f_n, sig_n, sf_n, give_n, roll_n = north_wall_stop()
    allow_p = rows[1][0]                                   # with the pin, the robot slid by its play
    ang = allow_p + roll_n
    g_give = _roll(Pos(0, -play, 0) * shelf, ang, (ty1 - play, tz0)).distance_to(crown)
    sf_ok = sf_n >= P["fem"]["sf_target"]
    print(f"  the north wall as the tipping stop (ESTIMATE: CoM {COM_PAST_EDGE:.0f} past the edge, "
          f"B134 one agent; {robot_mass_g():.0f} g = sim/mass_budget.json + the hands): {f_n:.1f} N at "
          f"{ST['wall_h'] - WALL_LEAD_IN:.0f} over the floor along its {np.diff(nx)[0]:.0f}, root "
          f"{sig_n:.1f} MPa across the layers = SF {sf_n:.1f} on PETG's {P['fem']['petg']['strength_z_mpa']:.0f} "
          f"({verdict(sf_ok, 'OK', 'WEAK')}: fem sf_target {P['fem']['sf_target']}); it gives {give_n:.2f} = "
          f"{roll_n:.2f} deg more roll, {ang:.2f} in all, where the shelf keeps {g_give:.2f} "
          f"({verdict(g_give > 1e-3)})")

    # ---- heights, and the legs' reach
    rise = DECK_BOT_Z - TUB["z"][0]                       # the tub's bottom to the deck bottom
    hts = [SEAT_Z + n * SEC_H + FLOOR_Z + rise for n in (0, 1, 2)]
    for n, upper in (("stand_crown", parts["stand_crown"]), ("stand_section", parts["stand_section"])):
        v_ok = _vol(Pos(0, 0, SEAT_Z) * upper, parts["stand_base"])
        v_bad = _vol(Pos(0, 0, SEAT_Z) * Rot(0, 0, 36) * upper, parts["stand_base"])
        v_low = _vol(Pos(0, 0, SEAT_Z - 0.1) * upper, parts["stand_base"])
        good = v_ok < 1.0 and v_bad > 50.0 and v_low > 1.0
        print(f"  {n} on the base: aligned {v_ok:.2f} mm^3 ({'SLIDES' if v_ok < 1 else 'BINDS'}), "
              f"36 deg off {v_bad:.0f} ({'REGISTERS' if v_bad > 50 else 'no reg!'}), 0.1 low "
              f"{v_low:.0f} (the seat is the stop: {verdict(good, 'OK', 'FLOATS')})")
    d_foot, d_hand = leg_depth_below_deck(), leg_depth_below_deck(HAND_SOLE)
    print(f"  deck-bottom heights over the floor: base + crown {hts[0]:.1f} | + 1 section {hts[1]:.1f} | "
          f"+ 2 sections {hts[2]:.1f} mm (seat {SEAT_Z:.0f} + n x {SEC_H:.0f} + floor {FLOOR_Z:.0f} + the "
          f"tub's bottom to the deck bottom {rise:.1f})")
    print(f"  the legs' reach below the deck bottom (hip {P['joints']['pos_deg']['hip'][0]}, tibia down): "
          f"{d_foot:.2f} to the foot point, {d_hand:.2f} with the B95 hand (sole {HAND_SOLE} past the knee, "
          f"B131); two sections leave {hts[2] - d_foot:.1f} / {hts[2] - d_hand:.1f} "
          f"({verdict(hts[2] - d_hand > 5.0)}: two sections free the whole envelope); one leaves "
          f"{hts[1] - d_foot:.1f} / {hts[1] - d_hand:.1f}")
    reach, n_poses = legs_into_cradle()
    print(f"  legs on the stand, the soft-limit box on a 5 deg grid ({n_poses} poses a leg), capsules "
          f"r 15 / 12 (report, B97's class; the column = skirt + ring + two sections, apothem "
          f"{APO + FIT + SKIRT_T:.1f}, under the plate; today's stand had the same column):")
    for i, (n_tub, n_cr, n_only, hr, n_col, n_col_only, cr) in enumerate(reach):
        print(f"    leg {i}: into the tub {n_tub}, into the cradle {n_cr}, the cradle and not the tub "
              f"{n_only}" + (f" (hip {hr[0]:.0f}..{hr[1]:.0f})" if hr else "") +
              f"; into the column {n_col}, not the tub {n_col_only}" +
              (f" (hip {cr[0][0]:.0f}..{cr[0][1]:.0f}, knee {cr[1][0]:.0f}..{cr[1][1]:.0f})" if cr else ""))

    # ---- print orientation
    flo = cp["floor"].bounding_box()
    over = (Pos((flo.min.X + flo.max.X) / 2, (flo.min.Y + flo.max.Y) / 2) *
            Rectangle(flo.size.X, flo.size.Y) - _pent(APO + FIT + SKIRT_T)).area
    print(f"  print as exported, the skirt on the bed: supports from the bed under the plate inside "
          f"the skirt (as before) and under the floor's two south corners outside it ({over:.0f} mm^2, "
          f"{SEAT_T + 18:.0f} over the bed); the walls and the pin print up, no support")
    raise SystemExit(0 if ok else 1)
