"""Shared interface geometry (D020) — every part that carries one of the six
standard interfaces builds its features THROUGH these helpers, reading frozen
dims from params.yaml. The deck, the bench jig, and the coxa plate all call
the same functions: fit is guaranteed by construction, not by diligence.

Frames: leg-port features are defined in LEG-LOCAL coords (yaw axis at
origin, +x outboard) exactly like part_coxa.py; callers pass a build123d
transform (e.g. Rot(0,0,ang) * Pos(110,0,0)) placing the leg station.
"""
import numpy as np
from build123d import *
from common import params

P = params()
IF = P["interfaces"]
PR = P["print"]
FIT = PR["clearance_fit"]

# I1 cable pass-through in the deck (leg-local; frozen D020 convention): the
# leg drop's XT30 + JST-XH-5 mate here. part_coxa.harness_path() starts on it.
CABLE_CUTOUT_X = -29.0
CABLE_CUTOUT_W = 11.0


# ====================================================================== I1
def leg_port_deck_features(deck, top_z, station_tf=None, dowels=True):
    """Apply the DECK side of the leg port to `deck` (top surface at z=top_z):
    + dowel posts (Ø4 x engage), heat-set pockets under the thumbscrew
    stations, the hook catch bar at the outboard edge, 11 mm cable cutout.
    station_tf: transform from leg-local to deck coords (default: identity).
    """
    lp = IF["leg_port"]
    tf = station_tf if station_tf is not None else Pos(0, 0, 0)
    # dowel posts (printed, proud of the deck top)
    if dowels:
        for dx, dy in lp["dowel_xy"]:
            post = Pos(dx, dy, top_z + lp["dowel_engage"] / 2) * \
                Cylinder((lp["dowel_d"] - 0.05) / 2, lp["dowel_engage"])
            tip = Pos(dx, dy, top_z + lp["dowel_engage"]) * \
                Cone((lp["dowel_d"] - 0.05) / 2, (lp["dowel_d"] - 0.05) / 2 - 0.8, 1.2)
            deck += tf * (post + tip)
    # heat-set pockets for the thumbscrews
    for dx, dy in lp["thumbscrew_xy"]:
        deck -= tf * Pos(dx, dy, top_z - PR["heatset_m3_h"] / 2 + 0.01) * \
            Cylinder(PR["heatset_m3_d"] / 2, PR["heatset_m3_h"])
        deck -= tf * Pos(dx, dy, top_z - 8) * Cylinder(3.4 / 2, 16)  # through
    # hook catch: THROUGH-SLOT at the plate's inboard edge; the plate's
    # downturned lip drops through and its foot hooks under the deck bottom
    # (the deck ends at R100 — there is no deck outboard of the plate, D012)
    deck -= tf * Pos(lp["hook_slot_x"], 0, top_z - 10) * \
        Box(lp["hook_slot_w"], lp["hook_lip_w"] + 2 * FIT, 24)
    # cable pass-through (existing deck convention: leg-local x=-29)
    deck -= tf * Pos(CABLE_CUTOUT_X, 0, top_z - 10) * Box(CABLE_CUTOUT_W, CABLE_CUTOUT_W, 24)
    return deck


def leg_port_plate_features(plate, plate_top_z=2.0, plate_bot_z=-4.0):
    """Apply the LEG side of the port to a coxa base plate solid (part_coxa
    frame: the plate is z -4..0, top at 0): dowel bores, thumbscrew
    clearance bores (not captive: no lip is modelled, B82), and the inboard hook lip.
    Pass the actual plate z-extents."""
    lp = IF["leg_port"]
    t = plate_top_z - plate_bot_z
    # dowel bores (through)
    for dx, dy in lp["dowel_xy"]:
        plate -= Pos(dx, dy, (plate_top_z + plate_bot_z) / 2) * \
            Cylinder((lp["dowel_d"] + FIT) / 2, t + 2)
        # lead-in chamfer from below
        plate -= Pos(dx, dy, plate_bot_z + 0.6) * \
            Cone((lp["dowel_d"] + FIT) / 2 + 0.8, (lp["dowel_d"] + FIT) / 2, 1.3)
    # thumbscrews: a loose Ø3.7 shaft pass. The 'head well' below sits ABOVE
    # the plate (z top..top+8), so on the plate alone it removes nothing; no lip (B82).
    for dx, dy in lp["thumbscrew_xy"]:
        plate -= Pos(dx, dy, (plate_top_z + plate_bot_z) / 2) * \
            Cylinder(3.7 / 2, t + 2)                     # loose shaft pass
        plate -= Pos(dx, dy, plate_top_z + 4.0) * \
            Cylinder((lp["thumbscrew_head_d"] + 1.2) / 2, 8)   # head well
    # hook lip: down-turned tab at the INBOARD edge (plate ends x=-46):
    # reach-out to the slot, down-turn through it (deck is 6 thick), then a
    # foot pointing further inboard (-x) that hooks under the deck bottom
    lipw = lp["hook_lip_w"]
    deck_t = 6.0                     # B27 OPEN: params body.deck_t (4.0) is unread
    lip = Pos(-47.2, 0, plate_bot_z + 1.0) * Box(4.4, lipw, 2)         # reach
    lip += Pos(lp["hook_slot_x"] - 0.4, 0,
               plate_bot_z - (deck_t + 1.6) / 2 + 2.0) * \
        Box(lp["hook_lip_t"] - 0.8, lipw, deck_t + 5.6)                # down-turn
    lip += Pos(lp["hook_slot_x"] - 0.4 - (lp["hook_foot"] + 1.1) / 2 + 1.1 / 2,
               0, plate_bot_z - deck_t - 2.4) * \
        Box(lp["hook_foot"] + 1.1, lipw, 1.8)                          # foot
    return plate + lip


# ====================================================================== I3
# B87 (D063): the cartridge is a bayonet. As first drawn the rotor's pegs had no
# way into the housing's closed track, the rotor ended flush with the housing
# bottom (0 mm^3 below it, 0.75 clear of the strike), the strike was blind with no
# undercut, and with a sector down no coin reached the head. Now the rotor drops
# through the housing (two keyways pass its lugs; turned off them it is captive,
# head above, lugs below), hangs LATCH_REACH below the panel's seating face, passes
# the frame's entry slots at OPEN, and a quarter turn cams its lugs under the land
# in the frame's underside recess (latch_strike). The operating end is the rotor's
# BOTTOM: a flat-screwdriver slot 0.2 above the frame's underside, reached from
# below with the panel on. Not a coin: the slot floor is 1.7 up the Ø6.6 bore, and a
# Ø16-20 coin gets only ~0.5 of the slot's 1.5 before it meets the frame mid-turn
# (1.0 in: 0.9-1.5 mm^3 at 30-60 deg, measured). Frame: z 0 = the seating plane (the
# housing's bottom, flush with the panel face that sits on the frame), +z into the panel.
LATCH_SHAFT_D = 6.0
LUG_R, LUG_W, LUG_H = 5.5, 3.0, 2.0          # lug reach, tangential width, height
LATCH_HEAD_D, LATCH_HEAD_H = 10.0, 2.0      # 2.0 overhang on the shaft (printed head up)
LATCH_SLOT = (2.0, 1.5)           # the operating slot, width x depth: a flat screwdriver, <= 5 wide
LATCH_BITE = 0.2                  # cam preload at LOCKED: the lug top 0.2 into the land. VERIFY
LATCH_GAP = IF["panel_latch"]["rotor_cam_rise"] - LATCH_BITE   # axial play at OPEN (0.6)
LATCH_LAND = 3.2                  # frame top -> the recess ceiling at the entry
LATCH_REACH = LATCH_LAND + LATCH_GAP + LUG_H  # rotor bottom below the seating plane (5.8)
LATCH_ENTRY_DEG = 45.0            # the entry line (lugs at OPEN) from the frame's +x: at a
                                  # deck strike +x is radial, and 45 keeps the recess off the edge
LATCH_RAMP_DEG = 50.0             # the ceiling ramps down over 0..50 deg, dwells to 90, stops
LATCH_RECESS_R = 6.1              # the lug's corner (5.70) + FIT, rounded up
# B107: the keyway index. The housing is a D (a flat LATCH_FLAT_R from its axis, square
# to the keyways) and every panel's pocket is the same D, 0.15 bigger all round, so it
# goes in one way: its keyways LATCH_KEYWAY_DEG from the entry line. On a strike the
# rotor turns 0..90 between the recess's stops (latch_strike), so at 135 / 315 the
# keyways sit outside that travel: the lugs pass them only at 132..138 deg, 42 past LOCKED
# and 42 back past OPEN (part_panel.latch_index). No angle the latch is worked at, or comes
# off at (OPEN), lines the lugs up with them. Before, the housing was round and glued at
# any angle: keyways at OPEN would drop a rotor out of every panel lifted off. The index
# is a place, not a stop: off the robot a rotor turned 45 deg back past OPEN still
# reaches them (nothing holds its 3.8 mm of axial play without a strike under it).
LATCH_KEYWAY_DEG = 135.0
LATCH_FLAT_R = 6.0                # leaves 2.7 of wall to the Ø6.6 bore at the flat
# the carapace latch station (D030): each sector's latch pad over the deck's strike.
# Worked from under the deck, so nothing may hang under a strike: station 162's sits
# at body (-73.5, -12.3), inside the 175 x 50 battery bay's footprint (B84), and the
# bay must leave that column open (a Ø5 driver + its hand) or that sector cannot come off
SHELL_LATCH_AZ, SHELL_LATCH_R = 27.5, 74.5


def latch_lug(grow=0.0, z0=0.0, z1=LUG_H, x0=2.0):
    """One lug along +x (x0 inside the shaft, out to LUG_R), grown by `grow` on its
    free faces, spanning z0..z1."""
    return Pos((x0 + LUG_R + grow) / 2, 0, (z0 + z1) / 2) * \
        Box(LUG_R + grow - x0, LUG_W + 2 * grow, z1 - z0)


def latch_d_profile(r, flat, z0, z1):
    """The I3 index shape (B107): a cylinder of radius r cut flat at y = flat (housing
    frame: keyways along x, the flat on +y), spanning z0..z1."""
    return Pos(0, 0, (z0 + z1) / 2) * Cylinder(r, z1 - z0) & \
        Pos(0, (flat - r - 1) / 2, (z0 + z1) / 2) * Box(2 * r + 2, flat + r + 1, z1 - z0 + 2)


def latch_insert_housing():
    """The replaceable latch cartridge's OUTER: Ø14 x 6, glued into the panel's
    Ø14.3 pocket with its bottom flush with the seating face. A Ø6.6 bearing bore
    and two keyways that pass the rotor's lugs at assembly only; the flat on its
    side is the keyway index (B107): it fits the pocket's flat one way."""
    pl = IF["panel_latch"]
    t = pl["housing_t"]
    h = latch_d_profile(pl["housing_d"] / 2, LATCH_FLAT_R, 0.0, t)
    h -= Pos(0, 0, t / 2) * Cylinder(LATCH_SHAFT_D / 2 + FIT, t + 2)
    for s in (0, 180):          # keyways: 1.2 of wall left to the Ø14 skin
        h -= Rot(0, 0, s) * latch_lug(FIT, -1.0, t + 1.0)
    return h


def latch_housing_tf():
    """The housing in its pocket, in the latch frame (z 0 = the seating plane, +x the
    strike's +x): its keyways on the index, LATCH_KEYWAY_DEG from the entry line."""
    return Rot(0, 0, LATCH_ENTRY_DEG + LATCH_KEYWAY_DEG)


def latch_pocket(z0, z1):
    """Cutter for a panel's housing pocket (B107), in the latch frame, spanning z0..z1:
    the Ø housing_pocket_d with the index flat, both FIT/2 off the housing's, turned as
    latch_housing_tf. A panel with no strike yet (the belly door, the tray tongue) takes
    its own +x as the strike's: draw the strike to match."""
    pl = IF["panel_latch"]
    gap = (pl["housing_pocket_d"] - pl["housing_d"]) / 2
    return latch_housing_tf() * latch_d_profile(pl["housing_pocket_d"] / 2, LATCH_FLAT_R + gap, z0, z1)


def latch_insert_rotor():
    """The rotor, seated (head on the housing top) with its lugs along +x: a Ø6
    shaft through the housing, two lugs with their tops LATCH_LAND + LATCH_GAP below
    the seating plane, and the operating slot across its bottom end, square to the
    lugs (it shows where they point)."""
    t = IF["panel_latch"]["housing_t"]
    z0 = -LATCH_REACH
    r = Pos(0, 0, (z0 + t) / 2) * Cylinder(LATCH_SHAFT_D / 2, t - z0)
    r += Pos(0, 0, t + LATCH_HEAD_H / 2) * Cylinder(LATCH_HEAD_D / 2, LATCH_HEAD_H)
    for s in (0, 180):
        r += Rot(0, 0, s) * latch_lug(0.0, z0, z0 + LUG_H)
    r -= Pos(0, 0, z0 + LATCH_SLOT[1] / 2 - 0.01) * \
        Box(LATCH_SLOT[0], LATCH_SHAFT_D + 2, LATCH_SLOT[1])            # the slot
    return r


def latch_strike(frame_t):
    """Cutter for the FRAME side of I3 (deck, coupon): z 0 = the frame's top (the
    seating plane), its underside at -frame_t (>= LATCH_REACH + 0.2). A bore for the
    shaft, two entry slots through the land on the LATCH_ENTRY_DEG line, and an
    underside recess that is the lugs' swept path over the quarter turn (CCW seen
    from above): its ceiling ramps down rotor_cam_rise over 0..LATCH_RAMP_DEG, from
    LATCH_GAP above the lug tops to LATCH_BITE into them, dwells to 90 and stops."""
    # a thinner frame lets the rotor's slot end hang out of it (the deck's 6 is B27 OPEN)
    assert frame_t >= LATCH_REACH + 0.2 - 1e-6, f"I3 strike needs a {LATCH_REACH + 0.2:.1f} frame"
    rise = IF["panel_latch"]["rotor_cam_rise"]
    ceil_lock = -LATCH_LAND - rise
    bot = -frame_t - 1.0
    cut = Pos(0, 0, -frame_t / 2) * Cylinder(LATCH_SHAFT_D / 2 + FIT, frame_t + 2)
    w = LUG_W / 2 + FIT
    for s in (0, 180):
        a = LATCH_ENTRY_DEG + s
        cut += Rot(0, 0, a) * latch_lug(FIT, bot, 1.0)                        # entry slot
        # the swept path's plan: the quarter disc + the lug's half-widths at both ends
        disc = Pos(0, 0, (bot + ceil_lock) / 2) * Cylinder(LATCH_RECESS_R, ceil_lock - bot)
        path = Pos(LATCH_RECESS_R / 2, LATCH_RECESS_R / 2 - w / 2, (bot + ceil_lock) / 2) * \
            Box(LATCH_RECESS_R, LATCH_RECESS_R + w, ceil_lock - bot)
        path += Pos(LATCH_RECESS_R / 2 - w / 2, LATCH_RECESS_R / 2, (bot + ceil_lock) / 2) * \
            Box(LATCH_RECESS_R + w, LATCH_RECESS_R, ceil_lock - bot)
        cut += Rot(0, 0, a) * (disc & path)
        # the ramp: higher ceilings where the lug passes early in the turn (5 deg steps)
        n = int(LATCH_RAMP_DEG // 5)
        for k in range(n):
            phi = LATCH_RAMP_DEG * k / n
            c = -LATCH_LAND - rise * phi / LATCH_RAMP_DEG
            cut += Rot(0, 0, a + phi) * latch_lug(FIT, ceil_lock - 0.01, c)
    return cut


def magnet_pocket(solid, at_tf, from_below=False):
    pl = IF["panel_latch"]
    d = pl["magnet_d"] + 0.25
    t = pl["magnet_t"] + 0.2
    return solid - at_tf * Cylinder(d / 2, t)


# ====================================================================== I6
def _trapezoid_bar(w_root, w_face, d, L):
    """Trapezoid in YZ (w_root wide on z 0, w_face wide at z d), run along x."""
    with BuildPart() as bp:
        with BuildSketch(Plane.YZ):
            with BuildLine():
                Polyline((-w_root / 2, 0), (w_root / 2, 0), (w_face / 2, d),
                         (-w_face / 2, d), (-w_root / 2, 0))
            make_face()
        extrude(amount=L / 2, both=True)
    return bp.part


def dovetail_male(length=None, undercut=False):
    """Male bar, root on z 0, depth +z, runs along x.

    undercut=True is the I6 accessory profile (B83): the frozen 12 / 8 turned
    over, the 8 neck at the root and the 12 face outboard, so a shoe cannot
    lift off and its set screw wedges it. As first drawn the 12 sat at the root:
    a KEY, off which a shoe lifts with 0 mm^3 of clash at any lift, so no set
    screw could clamp it. undercut=False keeps that key for the stand's spigot
    joints (part_stand), where a closed skirt round five facets holds it."""
    dv = IF["dovetail"]
    b, c = dv["base_w"], dv["crest_w"]
    root, face = (c, b) if undercut else (b, c)
    return _trapezoid_bar(root, face, dv["depth"], length or dv["segment_len"])


# I6 set knob (B83): a thumb_knob_m3 on a 45-deg spot face at the shoe's +y upper
# edge. Its bore aims at the male's +y flank at mid-depth; the tip meets the male
# (first at the face's edge) and pushes it toward the mouth, so the undercut flanks
# wedge the shoe. 45 deg keeps the Ø12 knob 1.7 above the shoe's bottom face and
# clear of the whisker_shoe's wires (they leave its top at y +/-4).
SET_KNOB_TILT = 45.0
SET_KNOB_STANDOFF = 5.5          # spot face centre from the flank point, along the bore


def set_knob_axis():
    """(the flank point M, the spot-face centre E, the bore's outward unit u) in the
    shoe frame (x along the slot, y across, z up from the mouth)."""
    dv = IF["dovetail"]
    n, w, d = dv["crest_w"], dv["base_w"], dv["depth"]
    zm = d / 2
    M = np.array([0.0, n / 2 + (w - n) / 2 * zm / d, zm])
    b = np.deg2rad(SET_KNOB_TILT)
    u = np.array([0.0, np.cos(b), np.sin(b)])
    return M, M + SET_KNOB_STANDOFF * u, u


def set_knob_tf():
    """Shoe frame -> the knob's frame (knob base on the spot face, z out along the bore)."""
    _, E, _ = set_knob_axis()
    return Pos(*E) * Rot(SET_KNOB_TILT - 90, 0, 0)


def dovetail_female_shoe(length=None, body_h=10.0):
    """Female I6 shoe blank: the undercut slot (the male offset by FIT) + the set
    knob's tapped bore and spot face (B83; the knob is part_panel.thumb_knob_m3)."""
    dv = IF["dovetail"]
    n, w, d = dv["crest_w"], dv["base_w"], dv["depth"]
    L = (length or dv["segment_len"]) + 2
    blk = Pos(0, 0, body_h / 2) * Box(L, dv["base_w"] + 2 * FIT + 8, body_h)
    dg = d + FIT
    k = (w - n) / 2 / d                               # flank run per mm of depth
    r0 = n / 2 + FIT - 0.5 * k                        # half-width 0.5 below the mouth
    r1 = n / 2 + FIT + k * dg                         # at the slot roof
    with BuildPart() as cut:
        with BuildSketch(Plane.YZ):
            with BuildLine():
                Polyline((-r0, -0.5), (r0, -0.5), (r1, dg), (-r1, dg), (-r0, -0.5))
            make_face()
        extrude(amount=L / 2 + 2, both=True)
    blk -= cut.part
    tf = set_knob_tf()
    blk -= tf * Pos(0, 0, 15) * Cylinder(IF["leg_port"]["thumbscrew_head_d"] / 2 + FIT, 30)  # spot face
    blk -= tf * Cylinder(PR["screw_m3_tap"] / 2, 2 * (SET_KNOB_STANDOFF + 1))   # tapped bore
    return blk
