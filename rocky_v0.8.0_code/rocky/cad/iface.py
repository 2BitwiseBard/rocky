"""Shared interface geometry (D020) — every part that carries one of the six
standard interfaces builds its features THROUGH these helpers, reading frozen
dims from params.yaml. The deck, the bench jig, and the coxa plate all call
the same functions: fit is guaranteed by construction, not by diligence.

Frames: leg-port features are defined in LEG-LOCAL coords (yaw axis at
origin, +x outboard) exactly like part_coxa.py; callers pass a build123d
transform (e.g. Rot(0,0,ang) * Pos(110,0,0)) placing the leg station.
"""
from build123d import *
from common import params

P = params()
IF = P["interfaces"]
PR = P["print"]
FIT = PR["clearance_fit"]


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
    deck -= tf * Pos(-29, 0, top_z - 10) * Box(11, 11, 24)
    return deck


def leg_port_plate_features(plate, plate_top_z=2.0, plate_bot_z=-4.0):
    """Apply the LEG side of the port to a coxa base plate solid (existing
    part_coxa frame: plate z -4..+2 wait — v0.1 plate is z -4..0 top at 0):
    dowel bores, captive-thumbscrew bores + head counterbores with retainer
    lips, and the outboard hook lip. Pass the actual plate z-extents."""
    lp = IF["leg_port"]
    t = plate_top_z - plate_bot_z
    # dowel bores (through)
    for dx, dy in lp["dowel_xy"]:
        plate -= Pos(dx, dy, (plate_top_z + plate_bot_z) / 2) * \
            Cylinder((lp["dowel_d"] + FIT) / 2, t + 2)
        # lead-in chamfer from below
        plate -= Pos(dx, dy, plate_bot_z + 0.6) * \
            Cone((lp["dowel_d"] + FIT) / 2 + 0.8, (lp["dowel_d"] + FIT) / 2, 1.3)
    # captive thumbscrews: shaft clearance + head pocket with retainer lip
    # (screw drops in from above; a printed lip at the pocket bottom keeps it)
    for dx, dy in lp["thumbscrew_xy"]:
        plate -= Pos(dx, dy, (plate_top_z + plate_bot_z) / 2) * \
            Cylinder(3.7 / 2, t + 2)                     # loose shaft pass
        plate -= Pos(dx, dy, plate_top_z + 4.0) * \
            Cylinder((lp["thumbscrew_head_d"] + 1.2) / 2, 8)   # head well
    # hook lip: down-turned tab at the INBOARD edge (plate ends x=-46):
    # reach-out to the slot, down-turn through it (deck is 6 thick), then a
    # foot pointing further inboard (-x) that hooks under the deck bottom
    lipw = lp["hook_lip_w"]
    deck_t = 6.0
    lip = Pos(-47.2, 0, plate_bot_z + 1.0) * Box(4.4, lipw, 2)         # reach
    lip += Pos(lp["hook_slot_x"] - 0.4, 0,
               plate_bot_z - (deck_t + 1.6) / 2 + 2.0) * \
        Box(lp["hook_lip_t"] - 0.8, lipw, deck_t + 5.6)                # down-turn
    lip += Pos(lp["hook_slot_x"] - 0.4 - (lp["hook_foot"] + 1.1) / 2 + 1.1 / 2,
               0, plate_bot_z - deck_t - 2.4) * \
        Box(lp["hook_foot"] + 1.1, lipw, 1.8)                          # foot
    return plate + lip


# ====================================================================== I3
def latch_insert_housing():
    """The replaceable quarter-turn latch cartridge OUTER (glued into panels)."""
    pl = IF["panel_latch"]
    h = Pos(0, 0, pl["housing_t"] / 2) * Cylinder(pl["housing_d"] / 2, pl["housing_t"])
    h -= Pos(0, 0, pl["housing_t"] / 2) * Cylinder(9.4 / 2, pl["housing_t"] + 2)
    # rotor track: two 90-deg helical-ish ramps approximated by stepped arcs
    # (step cut 2.4 tall so the peg is covered through the full cam rise)
    for s in (0, 180):
        for k in range(6):
            a = s + k * 18
            z = 1.2 + pl["rotor_cam_rise"] * (k / 5)
            h -= Rot(0, 0, a) * Pos(5.6, 0, z + 0.4) * Box(3.4, 3.4, 2.4)
    return h


def latch_insert_rotor():
    """Rotor: coin-slot head, two cam pegs riding the housing track."""
    pl = IF["panel_latch"]
    r = Pos(0, 0, pl["housing_t"] / 2) * Cylinder(9.4 / 2 - FIT, pl["housing_t"])
    head = Pos(0, 0, pl["housing_t"] + 1.0) * Cylinder(6.5, 2.0)
    head -= Pos(0, 0, pl["housing_t"] + 2.2) * Box(14, 1.6, 2.0)   # coin slot
    for s in (0, 180):
        r += Rot(0, 0, s) * Pos(5.5, 0, 1.6) * Box(2.8, 2.8, 1.4)  # cam pegs
    return r + head


def magnet_pocket(solid, at_tf, from_below=False):
    pl = IF["panel_latch"]
    d = pl["magnet_d"] + 0.25
    t = pl["magnet_t"] + 0.2
    return solid - at_tf * Cylinder(d / 2, t)


# ====================================================================== I6
def dovetail_male(length=None):
    """Male dovetail bar (on shells): trapezoid, +y = outward face."""
    dv = IF["dovetail"]
    L = length or dv["segment_len"]
    with BuildPart() as bp:
        with BuildSketch(Plane.YZ):
            with BuildLine():
                b, c, d = dv["base_w"], dv["crest_w"], dv["depth"]
                Polyline((-b / 2, 0), (b / 2, 0), (c / 2, d), (-c / 2, d),
                         (-b / 2, 0))
            make_face()
        extrude(amount=L / 2, both=True)
    return bp.part


def dovetail_female_shoe(length=None, body_h=10.0):
    """Female shoe blank: block with the dovetail slot + M3 set-knob boss."""
    dv = IF["dovetail"]
    L = (length or dv["segment_len"]) + 2
    b = dv["base_w"] + 2 * FIT
    c = dv["crest_w"] + 2 * FIT
    d = dv["depth"] + FIT
    blk = Pos(0, 0, body_h / 2) * Box(L, b + 8, body_h)
    with BuildPart() as cut:
        with BuildSketch(Plane.YZ):
            with BuildLine():
                Polyline((-b / 2, 0), (b / 2, 0), (c / 2, d), (-c / 2, d),
                         (-b / 2, 0))
            make_face()
        extrude(amount=L / 2 + 2, both=True)
    blk -= cut.part
    blk -= Pos(0, 0, body_h - 2.6) * Rot(90, 0, 0) * \
        Cylinder(PR["screw_m3_tap"] / 2, b + 12)     # set-knob cross bore
    return blk
