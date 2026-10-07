"""Avionics tray (I4) — the Pi 5 + the IMU on one slide-out, at its measured home (B51).

  avionics_tray   : plate with three 10 mm lugs a side (pick 7), Pi 5 standoffs (58 x 49,
                    11 tall), the IMU seats under the Pi (TPU grommets, B4, B90), the latch
                    tongue with its I3 insert pocket, and the bulkhead wall with the connector
                    cutouts. One latch, one slide, one lift, everything out.
  tray_rail       : deck rail block, 2 needed (the second is the same part turned 180): a
                    C-channel opening toward the tray, its top lip windowed for the lugs, its
                    three bolt tabs INBOARD (under the plate).
  tray_latch_boss : the tongue's I3 boss, glued under the tongue (the cartridge's housing
                    runs through both): the plate stands 5.1 over the deck, and the cartridge's
                    seating face must be the deck top, where the strike is. A separate print
                    because the tray prints plate-down (a boss under the plate would float).

Pose (2026-10-07, decision 11: I4 revised; docs/BODY_LAYOUT_PROPOSAL.md s2 'The tray (B51)'):
params tray_xy (0, 0) turned tray_rot_deg 180, bulkhead north, tongue south over the deck's
strike at (0, -43) (part_deck.DECK_HOLES 'tray_strike', rot 180). The rails' channel sits
tray_lift 2.0 higher than D063 drew it, so the plate stands 5.4 over the deck top (5.1 resting
on the channel floor) and the M3 button heads on the inboard tabs fit under it. tray_tf() is
the pose; tray_assembly() returns the posed set for the views.

Release (pick 7): the wings are three 10 mm lugs on a 25 pitch, the channel's top lip is
three 6 mm segments over them, so the tray lifts free after a slide of 8.3 .. 16.7 either way
(lug_release 16 inside it, FIT from both ends). The slide is SOUTH, toward the tongue: slid
north the plate covers the deck's (0, 50) trunk slot (707 mm^3). The plate's two corners at
the tongue end are cut square to stations 234 / 306 (corner_cuts): uncut, they met the coxa
bases' inboard faces by slide 16, the plate now standing 2 higher (tray_lift). OPEN: the I3
rotor in a round strike pins the slide (it hangs 5.8 into the deck and rises only 3.8 at
OPEN), so as cut today the tray does not come out. __main__ measures the fix it needs, a
strike drawn out along the slide (slotted_strike_proposal: the deck's strike turned 225 and
the tongue's frame 45, see TONGUE_LATCH_ROT); it is not cut anywhere yet.

Pick 14: the bus adapter, the 5 V buck and the 6 V UBEC live face-down on the hub shelf
(part_busboard); the bus adapter runs over the Pi's UART. BOARDS stays here as the envelope
library the shelf imports (B108's datasheet envelopes). B127: the Pi is modelled from the
official Raspberry Pi 5 STEP (cad/ref/rpi5_envelope.json, MIT), not to its board top.

Cutouts sized for: XT30 panel (10.4x6.4), JST-XH 5-pin (16.6x6.2), 2x JST-SH (6.4x3.2),
Qwiic/JST-SH 4-pin I2C (7.4x3.2), USB-C slot (10x4). All VERIFY against the real connectors
before printing five of anything — one tray is cheap, that's the point of the standard. The
J6 foot-switch lines have no bulkhead connector yet (B123).
"""
import json
import os
import numpy as np
from build123d import *
from common import params, export
from iface import IF, latch_pocket, STATIONS, station_tf, DECK_BOT_Z

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
AT = IF["avionics_tray"]
PL = IF["panel_latch"]
HERE = os.path.dirname(os.path.abspath(__file__))

W, L, T = AT["plate_w"], AT["plate_l"], AT["plate_t"]     # 84 x 70 x 3
RAIL = AT["rail"]                                          # 3: the wing's reach past the plate edge
BH_W, BH_H = AT["bulkhead_w"], AT["bulkhead_h"]            # 70 x 26
BH_T = 2.4                                                 # the rear bulkhead wall

# ---- the pose (params, decision 11) -------------------------------------------------------
TRAY_XY = tuple(float(v) for v in AT["tray_xy"])           # (0, 0): was the literal (0, -38)
TRAY_ROT = float(AT["tray_rot_deg"])                        # 180: bulkhead north, tongue south
assert TRAY_ROT % 180 == 0, "the Pi's USB end and the rails assume the tray turned 0 or 180"
DECK_TOP = DECK_BOT_Z + 6.0                                 # -4: body z of the deck top (the 6 deck, B27)

# ---- the rails (I4's C-channels, kept; tabs inboard and the lip windowed) -------------------
# Rail frame: the block runs along local x (posed: the tray's y), the channel opens toward
# local +y (posed: inboard), z 0 on the deck top. Turned 90 about z and set RAIL_POSE_X out,
# its open face stands FIT off the plate's edge.
RAIL_L, RAIL_BLK_W = 56.0, 10.0
RAIL_POSE_X = W / 2 + FIT + RAIL_BLK_W / 2                  # 47.3, derived (was a literal W/2 + 5.3)
WING_H = 2.0
WING_Z0 = T - WING_H                                        # the wing / lugs: tray z 1..3
CH_H = WING_H + 2 * FIT                                     # 2.6: 0.3 over and under the lug
CH_Y0 = 1.4                                                 # channel back wall (rail y): 0.9 behind the lug tip
RAIL_LIP = 1.3                                              # the lip over the channel
CH_Z = 5.4 + AT["tray_lift"]                                # 7.4: the channel centre over the deck top
                                                            # (D063 drew 5.4: tray_lift raises it 2)
RAIL_H = CH_Z + CH_H / 2 + RAIL_LIP                         # 10.0 (was 8)
SEAT_Z = CH_Z - (WING_Z0 + WING_H / 2)                      # 5.4: plate underside over the deck top, lugs centred
SEAT_REST = SEAT_Z - FIT                                    # 5.1: the lugs resting on the channel floor
# the tabs: holes on the deck's 20 mm grid columns x +-40 (part_deck.DECK_HOLES tray_tab_* entries,
# ray-tested in __main__), so 7.3 inboard of the block's axis; under the plate since the turn
TAB_X, TAB_Y = 40.0, (-20.0, 0.0, 20.0)
TAB_HOLE_OFF = RAIL_POSE_X - TAB_X                          # 7.3
TAB_T, TAB_RIM = 2.0, 3.0                                   # tab thickness; tab past the hole's centre
HEAD_D, HEAD_K = 5.7, 1.65                                  # ISO 7380 M3 button head (a socket head's 3.0 does not fit)
HEAD_RELIEF = 3.0                                           # the block's inboard face notched round each
                                                            # head, from the tab top up
# pick 7: three 10 mm lugs on a 25 pitch, under three 6 mm lip segments at rest. Slid d, lug k
# sits in the window between segments k and k+1 (the end lugs run off the rail's ends): the
# solids part for LIP_SEG/2 + LUG_LEN/2 <= d <= LUG_PITCH - LIP_SEG/2 - LUG_LEN/2 (LIFT_WINDOW,
# 8 .. 17), and with FIT kept at both ends RELEASE is 8.3 .. 16.7. Three lugs need a 2 x 25 + 6
# = 56 rail: the longest lip segment that fits the 56 block is 6
LUG_LEN, LUG_PITCH, LIP_SEG = 10.0, 25.0, 6.0
LUG_Y = (-LUG_PITCH, 0.0, LUG_PITCH)
LIFT_WINDOW = (LIP_SEG / 2 + LUG_LEN / 2, LUG_PITCH - LIP_SEG / 2 - LUG_LEN / 2)
RELEASE = (LIFT_WINDOW[0] + FIT, LIFT_WINDOW[1] - FIT)
LUG_RELEASE = AT["lug_release"]                              # 16
# Slid toward the tongue, the plate's two corners there met coxa_yaw_base's inboard face at
# stations 234 / 306 by slide 16 (2 x 11.96 mm^3; slide 12 was clear; 2026-10-07): that face stands
# above the coxa plate at r BASE_FACE_R on the station axis (leg x -46.0) and reaches z 1.4, where
# the raised plate now is. corner_cuts() takes the corners (and the end lugs' outer corners) off
# square to those stations, CORNER_CLEAR off the face at the far end of RELEASE
BASE_FACE_R = 64.0
CORNER_CLEAR = FIT
assert RELEASE[0] <= LUG_RELEASE <= RELEASE[1], f"lug_release {LUG_RELEASE} outside the lugs' window {RELEASE}"
assert 2 * LUG_PITCH + LIP_SEG <= RAIL_L + 1e-9, "the three lip segments must fit the rail"

# ---- the latch tongue (I3) ------------------------------------------------------------------
TONGUE_W, TONGUE_L = 30.0, 16.0
TONGUE_Y = L / 2 + TONGUE_L / 2                             # 43: the latch axis (tray frame); the deck's
                                                            # strike is at (0, -43) with the tray turned
BOSS_H = SEAT_REST                                          # 5.1: the tray resting -> the boss on the deck
BOSS_Y0 = 28.0                                              # the boss runs under the plate to here: 7 of rim
                                                            # north of the Ø14.3 pocket (the tongue has 0.85)
# The tongue's latch frame about z, tray frame. 0 = the tray's +x, what part_deck.DECK_HOLES cuts
# this round ('tray_strike' rot 180 = 0 + tray_rot_deg). 45 is the measured fix for the slide
# (slotted_strike_proposal): the entry line then runs along the slide. Flip it only together
# with the deck's strike (rot 225, slotted): deck_hole_audit compares the two
TONGUE_LATCH_ROT = 0.0

# ---- the Pi 5 -------------------------------------------------------------------------------
# Mounting (mechanical drawing RP-008347-DS): 85 x 56 board, 4 x Ø2.7 holes 3.5 in from the
# edges = a 58 x 49 rectangle, M2.5. The rectangle is off the board's centre along x: the
# USB/Ethernet end runs 52.5 past its centre, the far end 32.5.
PI_HOLES = [(-29, -24.5), (29, -24.5), (-29, 24.5), (29, 24.5)]
PI_BOARD = (85.0, 56.0, 1.6)
PI_UNDERSIDE = 2.0                   # keep-out under the board: solder tails, microSD (STEP: 1.45), passives
PI_STANDOFF_H = AT["pi_standoff_h"]  # 11. D063 (B90): 6 -> 11, so the IMU fits under the Pi; in params since
                                     # 2026-10-07 (pick 14 keeps it), so the carapace and USB-plug checks read
                                     # the same number (BODY_LAYOUT s11: a copied 6 once overstated A by 5)
# pick 14: the USB end faces body +x (leg 4). With the tray turned 180 that is the tray's -x, so
# the board centre sits 10 toward tray -x of the hole pattern's centre
PI_USB_BODY = {"+x": 1, "-x": -1}[AT["pi_usb_end"]]
PI_USB_SIGN = PI_USB_BODY * int(round(np.cos(np.deg2rad(TRAY_ROT))))   # -1: the USB end, tray frame
PI_BOARD_DX = 10.0 * PI_USB_SIGN     # board centre off the hole-pattern centre (tray frame)
PI5_REF = os.path.join(HERE, "ref", "rpi5_envelope.json")
# BNO085 (Adafruit 4754 board file): 25.4 x 22.86 board, 4.6 tall with its parts, 4 x Ø2.5
# holes 2.54 in from the edges = 20.32 x 17.78. It sits on four imu_grommets (B4) under the
# Pi: an 84 x 70 plate has no 28 x 26 patch (board + grommet flanges) beside an 85 x 56 Pi
# (the free strips are 9.5 and 7 mm wide), so the Pi goes up instead.
IMU_BOARD = (25.4, 22.86, 4.6)
IMU_HOLES = [(sx * 10.16, sy * 8.89) for sy in (-1, 1) for sx in (-1, 1)]
IMU_XY = (0.0, 13.5)                 # board centre (tray frame; body (0, -13.5) turned)
IMU_SEAT_LIFT = 1.2                  # each grommet seat is the 3 mm plate lifted 1.2 (see avionics_tray)
IMU_SEAT_D = 11.0                    # each seat's boss (see avionics_tray)
ZIP_X, ZIP_Y = 30.0, (-14.0, 0.0, 14.0)   # weight relief + tie points for the Pi's leads (they were the
                                          # buck's and UBEC's zip slots); y +-18 cut under the standoff
                                          # feet, +-14 leaves 3.1 mm

# ---- the three boards on the hub shelf (pick 14): their envelopes, for part_busboard ---------
# B108: plan w x l, parts `up` above the board's underside, pins `down` below it, held `lift`
# off its mount; a board without a mount carries a zip tie over its top. Sources: the BOM rows
# and their datasheets, as fetched 2026-09-30.
#  bus_adapter: Waveshare Bus Servo Adapter (A), BOM A-02. 42 x 33, Ø2.5 holes on 37 x 28
#    (waveshare.com/bus-servo-adapter-a.htm). Heights measured on Waveshare's own STEP model
#    (wiki "Bus Servo Adapter (A)", Resources, 3D model): a 1.6 board, parts to 11.0 above it
#    (the DC5521 jack; pin headers 9.0, the screw terminal 8.4, the servo ports 5.9) and
#    through-hole pins 2.0 below it. Held on 2.5 mm standoffs at its own holes (pins + 0.5).
#  buck_5v: Pololu D24V50F5, BOM B-10. 0.7 x 0.8 x 0.35 in = 17.8 x 20.3 x 8.8 overall
#    (pololu.com/product/2851). Its 1000 uF capacitor is not modelled: its size is the part
#    bought's (VERIFY; params hub_shelf.boards takes 27.8 x 20.3 x 10.0 with it, ESTIMATE).
#  ubec_6v: Hobbywing UBEC-3A, BOM B-11. 43 x 17 x 7 (hobbywingdirect.com, UBEC-3A specs).
# TIE_T: a 3.6 mm nylon zip tie over a board's top, ~1.2 thick (guessed, not measured).
TIE_T = 1.2
BOARDS = {
    "bus_adapter": dict(w=42.0, l=33.0, up=1.6 + 11.0, down=2.0, lift=2.5, tie=0.0),
    "buck_5v": dict(w=17.8, l=20.3, up=8.8, down=0.0, lift=0.0, tie=TIE_T),
    "ubec_6v": dict(w=43.0, l=17.0, up=7.0, down=0.0, lift=0.0, tie=TIE_T),
}


def board_envelope(name, x, y, rot=0, floor_z=T):
    """B108: a board's envelope on a floor at floor_z (default the tray plate's top), from its
    pins (or the floor) up through its parts and its zip tie; rot 0 or 90 about z."""
    b = BOARDS[name]
    z0 = floor_z + b["lift"] - b["down"]
    z1 = floor_z + b["lift"] + b["up"] + b["tie"]
    w, l = (b["w"], b["l"]) if rot % 180 == 0 else (b["l"], b["w"])
    return Pos(x, y, (z0 + z1) / 2) * Box(w, l, z1 - z0)


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _vol(s):
    if s is None:
        return 0.0
    try:
        return float(s.volume)
    except Exception:
        return 0.0


# ====================================================================== pose
def tray_tf(slide=0.0, lift=0.0, dx=0.0, dy=0.0, dz=0.0, rest=False):
    """Tray frame -> body frame (z 0 = the plate's underside). At params tray_xy turned
    tray_rot_deg, the lugs centred in the channel (plate underside SEAT_Z over the deck top;
    rest=True: resting on the channel floor, FIT lower). slide runs along the tray's +y (with the
    tray turned 180 that is body SOUTH, toward the tongue), lift up, dx/dy/dz the float."""
    z = DECK_TOP + (SEAT_REST if rest else SEAT_Z)
    return Pos(TRAY_XY[0], TRAY_XY[1], z) * Rot(0, 0, TRAY_ROT) * Pos(dx, slide + dy, lift + dz)


def rail_tf(side):
    """A rail block in the TRAY frame (side +1 / -1: the tray's +x / -x edge): its open face
    FIT off the plate, its underside on the deck top (SEAT_Z under the plate)."""
    base = Pos(RAIL_POSE_X, 0, -SEAT_Z) * Rot(0, 0, 90)
    return base if side > 0 else Rot(0, 0, 180) * base


def tongue_latch_tf(rot=None):
    """The tongue's latch frame, tray frame: z 0 on the boss's underside (the cartridge's seating
    face; on the deck top with the tray resting), +z up, +x the tray's +x turned TONGUE_LATCH_ROT
    (0). Turned with the tray (tray_rot_deg 180) its +x is body -x: part_deck.DECK_HOLES cuts the
    tray strike 'rot 180', the same convention (part_deck.deck_hole_audit checks the xy and the
    direction). rot overrides TONGUE_LATCH_ROT (the slotted-strike measurement)."""
    return Pos(0, TONGUE_Y, -BOSS_H) * Rot(0, 0, TONGUE_LATCH_ROT if rot is None else rot)


def slotted_strike_proposal(frame_t=6.0):
    """PROPOSAL, cut nowhere (for iface.latch_strike + part_deck): the I3 strike with its bore
    drawn out along the entry line (+LATCH_ENTRY_DEG in the strike frame), through, LATCH_SHAFT_D
    + 2 FIT wide, so a rotor at OPEN slides out with the tray. With the tongue's frame turned 45
    (the entry line along the tray's +y, the slide) the slot runs from the bore toward the slide,
    the leading lug in it and the trailing one back along its own entry slot. Its end stops the
    leading lug's tip at a slide of LUG_RELEASE + 0.5, inside the lift window: the slot is the
    release's stop. At LOCKED the lugs lie square to it, under the land."""
    from iface import latch_strike, LATCH_SHAFT_D, LATCH_ENTRY_DEG, LUG_R
    ln = LUG_RELEASE + 0.5 + LUG_R
    slot = Rot(0, 0, LATCH_ENTRY_DEG) * Pos(ln / 2, 0, -frame_t / 2) * Box(ln, LATCH_SHAFT_D + 2 * FIT, frame_t + 2)
    return latch_strike(frame_t) + slot


def corner_cuts():
    """Half-space cutters (tray frame) for the plate + lugs: one per station AHEAD of the slide
    (tray +y), its face square to that station's axis, so that slid RELEASE[1] the tray stays
    CORNER_CLEAR inside BASE_FACE_R. With the tray at (0, 0) turned 180 these are stations 234 and
    306: each cuts a 4.8 x 3.5 corner off the plate and 0.5 off the end lug's outer corner."""
    out = []
    for a in STATIONS:
        at = np.deg2rad(a - TRAY_ROT)
        uy = np.sin(at)
        if uy <= 1e-6:
            continue
        ub = (np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a)))
        c = BASE_FACE_R - CORNER_CLEAR - (TRAY_XY[0] * ub[0] + TRAY_XY[1] * ub[1]) - RELEASE[1] * uy
        out.append(Rot(0, 0, np.rad2deg(at)) * Pos(c + 50.0, 0, T / 2) * Box(100.0, 200.0, 40.0))
    return out


# ====================================================================== parts
def avionics_tray():
    tray = Pos(0, 0, T / 2) * Box(W, L, T)
    # pick 7: the wings are three lugs a side (were one 54 mm wing): they ride in the rails'
    # channels and pass up through the lip's windows once slid (see tray_rail)
    for sx in (1, -1):
        for y in LUG_Y:
            tray += Pos(sx * (W / 2 + RAIL / 2), y, WING_Z0 + WING_H / 2) * Box(RAIL, LUG_LEN, WING_H)
    # the tongue end's corners, square to the coxa bases ahead of the slide (corner_cuts); before
    # the tongue goes on, which stays 0.42 off the same face at RELEASE[1]
    for cut in corner_cuts():
        tray -= cut
    # Pi 5 standoffs (M2.5 thread-forming, Ø2.05 bore the full height) on the Pi's own
    # 58 x 49 pattern. D063 (B89): until then the y of PI_HOLES was scaled by 0.9 (58 x 44.1,
    # since the first import), 2.45 mm off every Pi hole
    for hx, hy in PI_HOLES:
        s = Pos(hx, hy, T + PI_STANDOFF_H / 2) * Cylinder(3.4, PI_STANDOFF_H)
        s -= Pos(hx, hy, T + (PI_STANDOFF_H + 1) / 2) * Cylinder(2.05 / 2, PI_STANDOFF_H + 1)
        tray += s
    # (pick 14: the bus-adapter pad's two M3 holes are gone with the adapter, to the hub shelf)
    # IMU pad: 4x Ø4.8 grommet holes (BNO085 on TPU grommets, B4) on the board's 20.32 x 17.78.
    # D063 (B90): each seat is the 3 mm plate (the grommet's grip) lifted IMU_SEAT_LIFT, a Ø11
    # boss on top over a Ø9 recess below that takes the grommet's 1.2 mm bottom flange. Flat,
    # the flange + a DIN 934 nut + the M2.5 x 10's tail hung 3.4 under the plate; lifted, they
    # end 2.2 down (__main__ measures the room under the plate)
    for hx, hy in IMU_HOLES:
        x, y = IMU_XY[0] + hx, IMU_XY[1] + hy
        tray += Pos(x, y, T + IMU_SEAT_LIFT / 2) * Cylinder(IMU_SEAT_D / 2, IMU_SEAT_LIFT)
        tray -= Pos(x, y, (IMU_SEAT_LIFT - 1) / 2) * Cylinder(9 / 2, IMU_SEAT_LIFT + 1)
        tray -= Pos(x, y, (T + IMU_SEAT_LIFT) / 2) * Cylinder(4.8 / 2, T + IMU_SEAT_LIFT + 2)
    # weight-relief + tie field
    for zx in (-ZIP_X, ZIP_X):
        for zy in ZIP_Y:
            tray -= Pos(zx, zy, T / 2) * Box(4, 8, T + 2)
    # the latch tongue with the I3 insert pocket (the D of the keyway index, B107), through: the
    # housing runs down through the tray_latch_boss glued under it to the deck top
    tongue = Pos(0, TONGUE_Y, T / 2) * Box(TONGUE_W, TONGUE_L, T)
    tray += tongue
    tray -= tongue_latch_tf() * latch_pocket(-1.0, BOSS_H + T + 1.0)
    # rear bulkhead wall + connector cutouts. 2026-10-07: the tray XT30 and the trunk's XH-5 moved
    # next to each other about x 0, over the deck's (0, 50) trunk slot (12.5 x 7.5) that both drop
    # through to the hub shelf (B122); the two spare SH moved out to make room
    bh = Pos(0, -L / 2 + BH_T / 2, T + BH_H / 2) * Box(BH_W, BH_T, BH_H)
    for w, h, x, z in BULKHEAD_CUTS:
        bh -= Pos(x, -L / 2 + BH_T / 2, T + z) * Box(w, 4, h)
    tray += bh
    return tray


# (w, h, x, z) on the bulkhead face, tray frame (body x = -x with the tray turned 180)
BULKHEAD_CUTS = [
    (10.4 + FIT, 6.4 + FIT, 9.5, 8),          # XT30 power in (5 V from the shelf's buck, pick 14)
    (16.6 + FIT, 6.2 + FIT, -7.0, 8),         # JST-XH 5-pin trunk (the Pi's UART to the bus adapter)
    (6.4 + FIT, 3.2 + FIT, 21.0, 8),          # JST-SH spare
    (6.4 + FIT, 3.2 + FIT, 29.5, 8),          # JST-SH spare
    (7.4 + FIT, 3.2 + FIT, -24.0, 18),        # Qwiic I2C (sensor mux)
    (10.0, 4.0, 0.0, 18),                     # USB-C pass slot
]
TRUNK_CUTS = {"XT30": 0, "XH-5": 1}           # the two that drop through the (0, 50) slot


def tray_latch_boss():
    """The tongue's I3 boss, modelled in the latch frame (z 0 = its underside, the cartridge's
    seating face, on the bed): TONGUE_W wide, from under the plate (BOSS_Y0) to the tongue's end,
    BOSS_H tall, the housing's D pocket through it on the tongue's index. Glued under the tongue
    with the housing through both: the D locates the boss on the tray."""
    y0, y1 = BOSS_Y0 - TONGUE_Y, TONGUE_L / 2
    boss = _box(-TONGUE_W / 2, TONGUE_W / 2, y0, y1, 0.0, BOSS_H)
    return boss - latch_pocket(-1.0, BOSS_H + 1.0)


def tray_rail():
    """Deck rail block: C-channel OPENING at its inner (+y local) face, the lugs slide straight
    in. The lip over the channel is three LIP_SEG segments over the lugs at rest, windows between
    (pick 7). Three bolt tabs on the OPEN side since 2026-10-07 (inboard, under the plate: holes
    on the deck's x +-40 grid columns), each with a notch in the block's face for its M3 button head."""
    blk = Pos(0, 0, RAIL_H / 2) * Box(RAIL_L, RAIL_BLK_W, RAIL_H)
    # channel: opens through the +y face, swallows the lug to CH_Y0
    y1 = RAIL_BLK_W / 2 + 0.2
    blk -= Pos(0, (CH_Y0 + y1) / 2, CH_Z) * Box(RAIL_L + 2, y1 - CH_Y0, CH_H)
    # the lip's windows: everything over the channel between the segments at LUG_Y
    segs = sorted(LUG_Y)
    edges = [-RAIL_L / 2 - 1] + [e for c in segs for e in (c - LIP_SEG / 2, c + LIP_SEG / 2)] + [RAIL_L / 2 + 1]
    for a, b in zip(edges[0::2], edges[1::2]):
        if b - a > 1e-6:
            blk -= _box(a, b, CH_Y0, y1 + 0.5, CH_Z + CH_H / 2 - 0.01, RAIL_H + 1)
    for sx in TAB_Y:
        y0t = RAIL_BLK_W / 2 - 2.0                     # 2 into the block's face
        y1t = TAB_HOLE_OFF + TAB_RIM
        tab = _box(sx - 5.0, sx + 5.0, y0t, y1t, 0.0, TAB_T)
        tab -= Pos(sx, TAB_HOLE_OFF, TAB_T / 2) * Cylinder(PR["screw_m3_clear"] / 2, TAB_T + 2)
        blk += tab
        # the head (Ø5.7 at 7.3 off the axis) reaches 0.55 into the block's face: a notch
        blk -= Pos(sx, TAB_HOLE_OFF, TAB_T + HEAD_RELIEF / 2) * Cylinder(HEAD_D / 2 + FIT, HEAD_RELIEF)
    return blk


def tray_footprint():
    """The tray's plan envelope on the deck as offsets from TRAY_XY in BODY axes (turned by
    tray_rot_deg): [(cx, cy, w, h)] — plate + lugs, both rail blocks with their inboard tabs, the
    tongue + boss. For layout audits (a keep-out, not a model)."""
    s = int(round(np.cos(np.deg2rad(TRAY_ROT))))
    x0, x1 = RAIL_POSE_X - (TAB_HOLE_OFF + TAB_RIM), RAIL_POSE_X + RAIL_BLK_W / 2
    return [(0.0, 0.0, W + 2 * RAIL, L),
            ((x0 + x1) / 2, 0.0, x1 - x0, RAIL_L),
            (-(x0 + x1) / 2, 0.0, x1 - x0, RAIL_L),
            (0.0, s * (BOSS_Y0 + L / 2 + TONGUE_L) / 2, TONGUE_W, L / 2 + TONGUE_L - BOSS_Y0)]


# ====================================================================== the Pi 5 (B127)
# The official Raspberry Pi 5 STEP (datasheets.raspberrypi.com/rpi5/RaspberryPi5-step.zip,
# rpi-5b_no_graphics.step, 2026-05-27; MIT, cad/ref/LICENSE-MIT-RaspberryPi.txt) is 77.6 MB with
# 2689 solids; its four big connector shells alone are 31 MB as STEP. cad/ref/rpi5_envelope.json
# keeps what the checks need, generated from it by pi5_boxes_from_step(): every part above the
# board as a box (the USB / Ethernet stacks in 1 mm z slabs, so their real top is kept, not their
# bounding box's corners), a layer box over the board for everything <= LAYER_Z, and the underside.
# STEP frame: x along the 85 side, 0 at the far end (the USB / Ethernet end is x 85); y from the
# USB-C edge; z from the PCB's underside (its solid spans 0.03 .. 1.31).
PI5_SHA256 = "78164070c1cc7ab854382950d2e85926c6e35432f1e5a146b733397a7c6afb44"   # the STEP
PI5_ZIP_SHA256 = "6841637b4cfa97637bf34b529bfe896f24d9634b2c40b57a156781f55d88f06f"
PI5_NAMES = {1339: "usb_middle (USB 3, next to the RJ45)", 1340: "usb_outer (USB 2)", 1600: "rj45",
             1342: "gpio_header pins", 1383: "poe_header pins", 546: "fan connector",
             491: "uart connector", 1209: "rtc battery connector", 1259: "cam/disp 1 FPC",
             1307: "cam/disp 0 FPC", 1911: "pcie FPC", 1876: "power button", 1939: "usb-c power"}


def pi5_boxes_from_step(step_path, layer_z=4.0, gap=2.0, slab=1.0):
    """B127: build rpi5_envelope.json's content from the official STEP (slow: ~1.5 min). Every
    solid inside the board outline that tops out <= layer_z goes into one layer box; the rest are
    clustered (overlapping bounding boxes are one part; clusters of pins, every member under 10
    mm^3, merge within `gap`, so a pin header is one box) and each cluster is one box, or, if it
    holds a big shell (> 100 mm^3, > 10 tall), its union per `slab` mm of z (consecutive equal
    slabs merged)."""
    import hashlib
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopoDS import TopoDS
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    with open(step_path, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    r = STEPControl_Reader()
    assert r.ReadFile(step_path) == IFSelect_RetDone, step_path
    r.TransferRoots()
    sol = []
    ex = TopExp_Explorer(r.OneShape(), TopAbs_SOLID)
    while ex.More():
        sol.append(TopoDS.Solid_s(ex.Current()))
        ex.Next()

    def bb(s):
        b = Bnd_Box()
        BRepBndLib.Add_s(s, b, True)
        return None if b.IsVoid() else list(b.Get())

    def vol(s):
        g = GProp_GProps()
        BRepGProp.VolumeProperties_s(s, g)
        return g.Mass()
    boxes = [bb(s) for s in sol]
    vols = [vol(s) for s in sol]
    # the PCB: the solid with the board's 85 x 56 plan
    pcb = max(range(len(sol)), key=lambda i: (boxes[i][3] - boxes[i][0]) * (boxes[i][4] - boxes[i][1])
              if boxes[i][5] - boxes[i][2] < 2.0 else 0)
    px0, py0, pz0, px1, py1, pz1 = boxes[pcb]
    inside = lambda b: b[0] >= px0 - 0.01 and b[3] <= px1 + 0.01 and b[1] >= py0 - 0.01 and b[4] <= py1 + 0.01
    layer = [i for i in range(len(sol)) if i != pcb and inside(boxes[i]) and boxes[i][5] <= layer_z]
    rest = [i for i in range(len(sol)) if i != pcb and i not in set(layer) and boxes[i][5] > pz1]
    under = min(b[2] for b in boxes)
    # cluster `rest`: overlapping bounding boxes are one part; then clusters of pins merge
    # within `gap` (not two connectors: the RJ45's LED pipes sit 1.95 from the USB stack)
    parent = {i: i for i in rest}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    near = lambda a, b, g: all(a[k] - g <= b[k + 3] and b[k] - g <= a[k + 3] for k in range(3))
    for a in rest:
        for b in rest:
            if a < b and near(boxes[a], boxes[b], 0.05):
                parent[find(a)] = find(b)
    small = {}
    for i in rest:
        small[find(i)] = small.get(find(i), True) and vols[i] < 10.0
    for a in rest:
        for b in rest:
            if a < b and small[find(a)] and small[find(b)] and near(boxes[a], boxes[b], gap):
                ra, rb = find(a), find(b)
                if ra != rb:
                    parent[ra] = rb
    clusters = {}
    for i in rest:
        clusters.setdefault(find(i), []).append(i)
    groups = []
    for members in sorted(clusters.values(), key=lambda m: min(m)):
        big = [i for i in members if vols[i] > 100 and boxes[i][5] - boxes[i][2] > 10]
        u = [min(boxes[i][k] for i in members) for k in range(3)] + \
            [max(boxes[i][k] for i in members) for k in range(3, 6)]
        lead = max(members, key=lambda i: vols[i])
        name = PI5_NAMES.get(lead) or next((PI5_NAMES[i] for i in sorted(members) if i in PI5_NAMES),
                                           f"solid {lead}")
        if not big:
            groups.append(dict(name=name, solids=sorted(members), boxes=[[round(v, 3) for v in u]]))
            continue
        cuts = [u[2]] + [float(k) * slab for k in range(int(np.ceil(u[2] / slab)), int(np.floor(u[5] / slab)) + 1)
                         if u[2] < k * slab < u[5]] + [u[5]]
        rows = []
        for z0, z1 in zip(cuts[:-1], cuts[1:]):
            q = None
            for i in members:
                b = boxes[i]
                if b[5] <= z0 or b[2] >= z1:
                    continue
                if i in big:
                    c = BRepAlgoAPI_Common(sol[i], BRepPrimAPI_MakeBox(
                        gp_Pnt(b[0] - 1, b[1] - 1, z0), gp_Pnt(b[3] + 1, b[4] + 1, z1)).Shape()).Shape()
                    cb = bb(c)
                    if cb is None:
                        continue
                    b = cb
                q = list(b[:2]) + list(b[3:5]) if q is None else \
                    [min(q[0], b[0]), min(q[1], b[1]), max(q[2], b[3]), max(q[3], b[4])]
            if q is None:
                continue
            row = [round(v, 3) for v in (q[0], q[1], z0, q[2], q[3], z1)]
            if rows and all(abs(rows[-1][k] - row[k]) < 0.05 for k in (0, 1, 3, 4)) and abs(rows[-1][5] - z0) < 1e-6:
                rows[-1][5] = row[5]                    # the same plan: one taller box
            else:
                rows.append(row)
        groups.append(dict(name=name, solids=sorted(members), boxes=rows))
    return dict(
        source=dict(url="https://datasheets.raspberrypi.com/rpi5/RaspberryPi5-step.zip",
                    file="rpi-5b_no_graphics.step (2026-05-27)", step_sha256=sha, zip_sha256=PI5_ZIP_SHA256,
                    license="MIT, Copyright (c) 2026 Raspberry Pi Ltd (cad/ref/LICENSE-MIT-RaspberryPi.txt)",
                    generated_by="part_avionics.pi5_boxes_from_step"),
        frame="STEP: x along the 85 side, 0 at the far end (USB/Ethernet end at 85); y from the USB-C "
              "edge; z from the PCB's underside. Boxes are [x0, y0, z0, x1, y1, z1], mm",
        solids=len(sol), pcb=[round(v, 3) for v in boxes[pcb]],
        hole_centre=[32.5, 28.0],
        layer=dict(z1=round(max(boxes[i][5] for i in layer), 3), solids=len(layer)),
        underside_z0=round(under, 3), groups=groups)


_PI5 = {}


def pi5_ref():
    if "ref" not in _PI5:
        with open(PI5_REF) as f:
            _PI5["ref"] = json.load(f)
    return _PI5["ref"]


def pi5_box(b, usb_sign=None, standoff_h=None):
    """A box [x0, y0, z0, x1, y1, z1] in the STEP's frame, posed in the TRAY frame as the Pi sits
    on its standoffs: the hole pattern's centre on (0, 0), the USB end toward tray +x (usb_sign
    +1) or -x (-1: PI_USB_SIGN, pick 14 with the tray turned; a half turn about z, not a mirror),
    the PCB's underside on the standoff tops."""
    ref = pi5_ref()
    s = PI_USB_SIGN if usb_sign is None else usb_sign
    h = PI_STANDOFF_H if standoff_h is None else standoff_h
    cx, cy = ref["hole_centre"]
    zb = T + h - ref["pcb"][2]
    x0, y0, z0, x1, y1, z1 = b
    xs = (x0 - cx, x1 - cx) if s > 0 else (cx - x1, cx - x0)
    ys = (y0 - cy, y1 - cy) if s > 0 else (cy - y1, cy - y0)
    return _box(xs[0], xs[1], ys[0], ys[1], zb + z0, zb + z1)


def pi5_envelope(usb_sign=None, standoff_h=None, underside=True):
    """B127: the Pi 5 as the STEP has it (board, every part above it, the underside), in the TRAY
    frame on its standoffs (pi5_box). The underside box leaves a Ø7 column round each standoff
    (as B90's keep-out). A compound of OVERLAPPING boxes: intersect it solid by solid (OCCT's
    boolean on such a compound is wrong). Unmodelled: the Active Cooler (or any HAT) over the SoC,
    and anything plugged into the GPIO header (__main__ adds the Dupont housings as a record)."""
    ref = pi5_ref()
    px0, py0, pz0, px1, py1, pz1 = ref["pcb"]
    zb = T + (PI_STANDOFF_H if standoff_h is None else standoff_h) - pz0
    parts = [pi5_box([px0, py0, pz0, px1, py1, ref["layer"]["z1"]], usb_sign, standoff_h)]
    for g in ref["groups"]:
        parts += [pi5_box(b, usb_sign, standoff_h) for b in g["boxes"]]
    if underside:
        u = pi5_box([px0, py0, ref["underside_z0"], px1, py1, pz0], usb_sign, standoff_h)
        for hx, hy in PI_HOLES:
            u -= Pos(hx, hy, zb) * Cylinder(3.5, 10)
        parts.append(u)
    return Compound(children=parts)


def pi5_port(name="usb_middle", which="lower", usb_sign=None, standoff_h=None):
    """A USB-A port's face centre and its outward direction (tray frame): (x, y, z, dir_x). The
    middle stack's lower port: the STEP's receptacle z 2.91..8.03 at y 29 (prep2 q_usb s1_ports,
    sliced from the same solids), its face the stack's +x end."""
    ref = pi5_ref()
    s = PI_USB_SIGN if usb_sign is None else usb_sign
    h = PI_STANDOFF_H if standoff_h is None else standoff_h
    g = next(g for g in ref["groups"] if g["name"].startswith(name))
    face = max(b[3] for b in g["boxes"])
    yc = {"usb_middle": 29.0, "usb_outer": 47.0}[name]
    zc = {"lower": (2.91 + 8.03) / 2, "upper": (11.41 + 16.53) / 2}[which]
    cx, cy = ref["hole_centre"]
    zb = T + h - ref["pcb"][2]
    return (s * (face - cx), s * (yc - cy), zb + zc, s)


# ====================================================================== the assembly
def tray_assembly(slide=0.0, lift=0.0, rest=False):
    """The posed set in the body frame: {'tray', 'boss', 'pi', 'rails'} (the rails stay on the
    deck; the rest moves with slide / lift). For gen_assembly_views and the layout checks."""
    tf = tray_tf(slide, lift, rest=rest)
    rails = tray_tf(rest=False)
    return {"tray": tf * avionics_tray(), "boss": tf * tongue_latch_tf() * tray_latch_boss(),
            "pi": tf * pi5_envelope(),
            "rails": [rails * rail_tf(s) * tray_rail() for s in (1, -1)]}


# ====================================================================== checks
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2 and sys.argv[1] == "--pi5-from-step":
        # regenerate cad/ref/rpi5_envelope.json from the official STEP (B127)
        d = pi5_boxes_from_step(sys.argv[2])
        if d["source"]["step_sha256"] != PI5_SHA256:
            print(f"WARNING: the STEP's sha256 {d['source']['step_sha256']} is not the one PI5_NAMES was read on")
        with open(PI5_REF, "w") as f:
            json.dump(d, f, indent=1)
            f.write("\n")
        print(f"written {PI5_REF}: {len(d['groups'])} groups, "
              f"{sum(len(g['boxes']) for g in d['groups'])} boxes")
        raise SystemExit(0)
    vol = _vol
    bad = []

    def need(name, val, good, ok, fail, unit="mm^3"):
        print(f"{name}: {val:.2f} {unit} ({ok if good(val) else fail})")
        if not good(val):
            bad.append(name.strip(" ."))

    def solids(*parts):
        """A flat list of solids. Every overlap here is taken solid by solid: on a Compound whose
        members overlap (the Pi's boxes do) OCCT's boolean is wrong (2026-10-07: 193 mm^3 over the
        trunk slot at every slide, where the solids give 0 at rest, 0 slid south, 707 slid north)."""
        out = []
        for p in parts:
            if isinstance(p, (list, tuple)):
                out += solids(*p)
            elif p is not None:
                out += list(p.solids())
        return out

    def boxed(*parts):
        return [(s, s.bounding_box()) for s in solids(*parts)]

    def near(p, q, m=0.5):
        return (p.min.X - m < q.max.X and q.min.X - m < p.max.X and p.min.Y - m < q.max.Y and
                q.min.Y - m < p.max.Y and p.min.Z - m < q.max.Z and q.min.Z - m < p.max.Z)

    def xvol(a, b):
        """Overlap volume of a and b (solids, lists or compounds), solid by solid; b may be boxed()."""
        bb_ = b if (isinstance(b, list) and b and isinstance(b[0], tuple)) else boxed(b)
        return sum(vol(p & q) for p, pb in boxed(a) for q, qb in bb_ if near(pb, qb))

    tray, rail, boss = avionics_tray(), tray_rail(), tray_latch_boss()
    export(tray, "avionics_tray")
    export(rail, "tray_rail")
    export(boss, "tray_latch_boss")
    bb = tray.bounding_box()
    print(f"tray envelope {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm; pose (params) {TRAY_XY} "
          f"turned {TRAY_ROT:.0f}, plate underside {SEAT_Z:.1f} over the deck top ({SEAT_REST:.1f} resting); "
          f"rails at x +-{RAIL_POSE_X:.1f}, channel centre {CH_Z:.1f} over the deck top (tray_lift "
          f"{AT['tray_lift']:g})")
    boss_t = tongue_latch_tf() * boss                       # the boss under the tongue, tray frame

    # ---- I4's channels, the lugs and the release (pick 7), tray frame -----------------------------
    rails_t = [rail_tf(s) * rail for s in (1, -1)]
    movers_t = [tray, boss_t]
    need("tray + boss (lugs centred in the channels) x rails", xvol(movers_t, rails_t), lambda q: q < 0.01,
         "SLIDES", "BINDS")
    need("  ... the tray lifted 1.0 at rest x rails (lugs under the lip segments)",
         xvol(Pos(0, 0, 1.0) * tray, rails_t), lambda q: q > 5, "CAPTIVE", "NOT CAPTIVE")
    drop = 0.0
    while drop < 2 and xvol(Pos(0, 0, -(drop + 0.05)) * tray, rails_t) < 0.01:
        drop += 0.05
    need("  ... the tray settles onto the channel floor", drop, lambda q: abs(q - FIT) < 0.06,
         f"RESTS {FIT} DOWN", "OFF", "mm")
    for d, way in ((LUG_RELEASE, "toward the tongue (body south: the release)"),
                   (-LUG_RELEASE, "toward the bulkhead (north)")):
        path = max(xvol([Pos(0, s, 0) * m for m in movers_t], rails_t) for s in np.linspace(0, d, 9))
        up = max(xvol([Pos(0, d, z) * m for m in movers_t], rails_t) for z in (0.5, 1, 2, 3, 4, 5, 10))
        need(f"slid {abs(d):.0f} {way} x rails (path, lugs in the channel) / then lifted 0.5..10", max(path, up),
             lambda q: q < 0.01, "RELEASES", "HELD")
    win = [xvol(Pos(0, d, 1.0) * tray, rails_t) for d in (LIFT_WINDOW[0] - 0.2, LIFT_WINDOW[0] + 0.05,
                                                          LIFT_WINDOW[1] - 0.05, LIFT_WINDOW[1] + 0.2)]
    need(f"  ... the lift window is slide {LIFT_WINDOW[0]:.1f}..{LIFT_WINDOW[1]:.1f} (blocked 0.2 outside it); "
         f"lug_release {LUG_RELEASE:g} inside {RELEASE[0]:.1f}..{RELEASE[1]:.1f} (FIT from both ends)",
         float(win[0] > 1 and win[1] < 0.01 and win[2] < 0.01 and win[3] > 1), lambda q: q > 0.5,
         "AS DESIGNED", "WINDOW WRONG", "(1 = as designed)")

    # ---- the body: the bases, the yaw servos, the forks, the deck ------------------------------
    from part_coxa import coxa_yaw_base, yaw_servo_placed, coxa_fork, hip_servo_placed, harness_solid
    from part_deck import body_deck, DECK_HOLES, deck_hole_cutter
    n_st = len(STATIONS)
    base_l, yaw_l, fh_l = coxa_yaw_base(), yaw_servo_placed(), coxa_fork() + hip_servo_placed()
    bases = [station_tf(i) * base_l for i in range(n_st)]
    yaws = [station_tf(i) * yaw_l for i in range(n_st)]
    YAWS = range(-40, 41, 10)
    forks = {(i, a): station_tf(i) * Rot(0, 0, a) * fh_l for i in range(n_st) for a in YAWS}  # yaw about its station
    pi_t = pi5_envelope()
    moving_t = solids(tray, boss_t, pi_t)
    rails_b = [tray_tf() * r for r in rails_t]
    fixed = boxed(bases, yaws)
    at_rest = [tray_tf() * s for s in moving_t]
    need("tray + boss + Pi (STEP) + rails x the five docked coxa bases + yaw servos",
         xvol(at_rest + rails_b, fixed), lambda q: q < 0.01, "CLEAR", "CLASH")
    deck = Pos(0, 0, DECK_BOT_Z) * body_deck()
    need("  ... x the deck (body_deck as cut in this tree; rails 0.01 up off its top; the holes: the table, below)",
         xvol(at_rest, deck) + xvol([Pos(0, 0, 0.01) * r for r in rails_b], deck), lambda q: q < 0.01,
         "CLEAR", "CLASH")
    obst = boxed(rails_b, bases, yaws, list(forks.values()))
    worst, worst_at = 0.0, ""
    for d in (LUG_RELEASE, -LUG_RELEASE):
        poses = [(s, 0.0) for s in np.linspace(0, d, 5)] + [(d, z) for z in range(5, 46, 5)]
        for s, z in poses:
            v = xvol([tray_tf(s, z) * q for q in moving_t], obst)
            if v > worst:
                worst, worst_at = v, f"slide {s:+.0f} lift {z:.0f}"
    need("the release path (slid +-16 in 4 mm steps, then lifted 5..45) x rails, bases, yaw servos and "
         "the forks + hip servos at yaw -40..40 (tray + boss + Pi)", worst, lambda q: q < 0.01, "CLEAR",
         f"CLASH at {worst_at}")
    # the slide room toward the tongue (bisection on the tray + boss + Pi against the bases and the
    # yaw servos), and the gap to the bases at the release and at the window's far end (corner_cuts)
    ahead = boxed(bases, yaws)
    lo_, hi_ = LUG_RELEASE, 40.0
    for _ in range(14):
        mid = (lo_ + hi_) / 2
        lo_, hi_ = (mid, hi_) if xvol([tray_tf(mid) * q for q in moving_t], ahead) < 1e-4 else (lo_, mid)
    gaps = {s: Compound(children=[tray_tf(s) * q for q in moving_t]).distance_to(Compound(children=bases + yaws))
            for s in (LUG_RELEASE, RELEASE[1])}
    print(f"  (slide room toward the tongue before a coxa base or yaw servo: {lo_:.2f} with corner_cuts; the "
          f"gap to them slid {LUG_RELEASE:g}: {gaps[LUG_RELEASE]:.2f}, slid {RELEASE[1]:g}: {gaps[RELEASE[1]]:.2f})")
    rest_c = Compound(children=at_rest + rails_b)
    gmin = min((rest_c.distance_to(forks[(i, a)]), f"leg {i} yaw {a:+d}") for i in range(n_st) for a in (-40, 0, 40))
    print(f"  (closest fork + hip servo to the tray + rails at rest: {gmin[0]:.2f} mm, {gmin[1]})")

    # ---- the tabs on the deck's holes (ray test against part_deck.DECK_HOLES, the table) --------
    table = {e[1]: e for e in DECK_HOLES if e[0].startswith("tray_tab")}
    holes = sorted({(round(e.arc_center.X, 4), round(e.arc_center.Y, 4)) for r in rails_b
                    for e in r.edges().filter_by(GeomType.CIRCLE)
                    if abs(e.radius - PR["screw_m3_clear"] / 2) < 1e-3 and e.arc_center.Z < DECK_TOP + 0.01})
    worst_xy, ray, ring_min, used = 0.0, 0.0, 1.0, set()
    for hx, hy in holes:
        key = min(table, key=lambda k: np.hypot(k[0] - hx, k[1] - hy))
        used.add(key)
        worst_xy = max(worst_xy, float(np.hypot(key[0] - hx, key[1] - hy)))
        probe = Pos(hx, hy, DECK_TOP) * Cylinder(0.5, 30)
        patch = Pos(key[0], key[1], DECK_BOT_Z + 3) * Box(14, 14, 6) - \
            Pos(0, 0, DECK_BOT_Z) * deck_hole_cutter(table[key])
        ray = max(ray, vol(probe & patch), xvol(probe, rails_b))
        ring = Pos(hx, hy, DECK_TOP + TAB_T / 2) * (Cylinder(2.9, TAB_T - 0.2) - Cylinder(2.0, TAB_T))
        ring_min = min(ring_min, xvol(ring, rails_b) / ring.volume)
    need(f"rail tab holes ({len(holes)}: {holes}) on DECK_HOLES' tray_tab entries", worst_xy,
         lambda q: len(holes) == 6 and len(used) == 6 and len(table) == 6 and q < 0.01, "ON THE TABLE", "OFF", "mm")
    need("  ... a Ø1 ray down each tab axis x the tab + a deck patch cut with the table's hole", ray,
         lambda q: q < 0.001, "THROUGH", "BLOCKED")
    need("  ... the tab round each hole (Ø4..5.8 ring, fraction that is plastic)", ring_min,
         lambda q: q > 0.99, "SOLID", "MISSING", "")
    half = max(max(abs(r.bounding_box().min.X), abs(r.bounding_box().max.X)) for r in rails_b)
    hp = Compound(children=at_rest).bounding_box()
    need("tray + rails half-span (the tabs inboard: the rail blocks' outer faces; outboard tabs reached "
         f"{RAIL_POSE_X + 7.0 + 4.0:.1f})", half, lambda q: q <= 56.0, "INSIDE the 56 grid zone", "OUTSIDE", "mm")
    print(f"  (with the Pi 5's stack: x {hp.min.X:.2f}..{hp.max.X:.2f}, y {hp.min.Y:.2f}..{hp.max.Y:.2f}, "
          f"z {hp.min.Z:.2f}..{hp.max.Z:.2f} body)")

    # ---- screw heads over the inboard tabs, tray_lift 2.0 ---------------------------------------
    print(f"screw-head room over the inboard tabs: plate underside - tab top = {SEAT_Z - TAB_T:.2f} centred / "
          f"{SEAT_REST - TAB_T:.2f} resting; an M3 button ({HEAD_K}) leaves {SEAT_Z - TAB_T - HEAD_K:.2f} / "
          f"{SEAT_REST - TAB_T - HEAD_K:.2f}")
    heads = [Pos(hx, hy, DECK_TOP + TAB_T + HEAD_K / 2) * Cylinder(HEAD_D / 2, HEAD_K) for hx, hy in holes]
    need("  ... six ISO 7380 heads on the tabs x the tray resting + the boss + the rails",
         xvol(heads, [tray_tf(rest=True) * m for m in movers_t] + rails_b), lambda q: q < 0.01, "FIT", "CLASH")

    # ---- B89 / B90: the standoffs on the Pi's holes, the IMU under the Pi ------------------------
    # PI_HOLES is the drawing's pattern on the posed board (3.5 and 61.5 in from the far end, 3.5 in
    # from both long edges; the far end is the one away from the USB end, PI_USB_SIGN), the standoff
    # axes are PI_HOLES (a Ø1.8 pin down each hole's axis meets no plastic and the wall round the
    # bore is solid out to r 3.2: off-axis by > 0.13 mm fails), and no cut in the plate comes
    # within 1 mm of a standoff's foot (the zip slots did)
    far = PI_BOARD_DX - PI_USB_SIGN * PI_BOARD[0] / 2       # the far end's x (tray frame)
    want = [(far + PI_USB_SIGN * ex, sy * (PI_BOARD[1] / 2 - 3.5)) for ex in (3.5, 61.5) for sy in (-1, 1)]
    need("PI_HOLES vs the Pi 5 drawing (58 x 49)",
         max(((hx - wx) ** 2 + (hy - wy) ** 2) ** 0.5
             for (hx, hy), (wx, wy) in zip(sorted(PI_HOLES), sorted(want))),
         lambda q: q < 0.01, "ON THE DRAWING", "OFF THE DRAWING", "mm")
    h = PI_STANDOFF_H - 0.2
    worst_pin = worst_wall = worst_foot = 0.0
    for hx, hy in PI_HOLES:
        worst_pin = max(worst_pin, vol(Pos(hx, hy, T + PI_STANDOFF_H / 2) * Cylinder(0.9, h) & tray))
        wall = Pos(hx, hy, T + PI_STANDOFF_H / 2) * (Cylinder(3.2, h) - Cylinder(1.2, h + 1))
        worst_wall = max(worst_wall, vol(wall - tray))
        worst_foot = max(worst_foot, vol(Pos(hx, hy, T / 2) * Cylinder(3.4 + 1.0, T - 0.02) - tray))
    need("Pi standoff axes = PI_HOLES: pin down each axis x tray", worst_pin,
         lambda q: q < 0.01, "COAXIAL", "OFF AXIS")
    need("  ... standoff wall missing round the bore", worst_wall, lambda q: q < 0.01, "SOLID", "OFF AXIS")
    need("plate missing within 1 mm of a standoff foot (tie slots, holes)", worst_foot,
         lambda q: q < 0.01, "CLEAR", "UNDERCUT")
    from part_smallwins import imu_grommet
    grommets = [Pos(IMU_XY[0] + hx, IMU_XY[1] + hy, T + IMU_SEAT_LIFT) * imu_grommet() for hx, hy in IMU_HOLES]
    gb = Compound(children=grommets).bounding_box()
    imu_z0 = gb.max.Z                                  # the top flanges carry the board
    imu = Pos(IMU_XY[0], IMU_XY[1], imu_z0 + IMU_BOARD[2] / 2) * Box(*IMU_BOARD)
    need("imu_grommet x 4 in their seats x tray", xvol(grommets, tray), lambda q: q < 1, "SEATED", "CLASH")
    need("  ... nudged 0.5 mm up or down (the flanges grip the 3 mm seat)",
         min(xvol([Pos(0, 0, dz) * g for g in grommets], tray) for dz in (0.5, -0.5)), lambda q: q > 1,
         "GRIPPED", "LOOSE")
    screws = [Pos(IMU_XY[0] + hx, IMU_XY[1] + hy, imu_z0 + 1.6 - 5.0) * Cylinder(2.5 / 2, 10)
              for hx, hy in IMU_HOLES]                 # M2.5 x 10, head on the IMU's top face
    need("M2.5 through the BNO085's holes x grommet bores + tray", xvol(screws, grommets + [tray]),
         lambda q: q < 0.01, "ON AXIS", "OFF AXIS")
    need("IMU envelope x tray", vol(imu & tray), lambda q: q < 0.01, "CLEAR", "CLASH")
    top = T + PI_STANDOFF_H
    worst = 0.0
    for dx in (PI_BOARD_DX, -PI_BOARD_DX):
        board = Pos(dx, 0, top + PI_BOARD[2] / 2) * Box(*PI_BOARD)
        under = Pos(dx, 0, top - PI_UNDERSIDE / 2) * Box(PI_BOARD[0], PI_BOARD[1], PI_UNDERSIDE)
        for hx, hy in PI_HOLES:
            under -= Pos(hx, hy, top - PI_UNDERSIDE / 2) * Cylinder(3.5, PI_UNDERSIDE + 1)
        worst = max(worst, xvol([board, under], [tray] + rails_t), xvol(under, [imu] + grommets))
    need("Pi board + underside keep-out (USB end either way) x tray, rails, IMU", worst,
         lambda q: q < 0.01, "CLEAR", "CLASH")
    need("  ... the Pi 5's STEP parts (USB end +x body) x tray + boss, rails, IMU + grommets",
         xvol(pi_t, movers_t + rails_t + [imu] + grommets), lambda q: q < 0.01, "CLEAR", "CLASH")
    need("IMU top to the Pi's underside keep-out", (top - PI_UNDERSIDE) - (imu_z0 + IMU_BOARD[2]),
         lambda q: q >= 1.0, "CLEAR", "TOO CLOSE", "mm")
    # the nuts: a DIN 934 M2.5 (5 A/F, 2 thick; its Ø5.8 corners) under each bottom flange, the
    # M2.5 x 10's tail below it, against the deck top with the tray resting on the channel floor
    deck_z = rails_t[0].bounding_box().min.Z + drop
    nuts = [Pos(IMU_XY[0] + hx, IMU_XY[1] + hy, gb.min.Z - 1.0) * Cylinder(5.0 / 3 ** 0.5, 2.0)
            for hx, hy in IMU_HOLES]
    need("IMU nuts x tray", xvol(nuts, tray), lambda q: q < 0.01, "FIT", "CLASH")
    screw_end = imu_z0 + 1.6 - 10.0
    need("M2.5 x 10 runs through the whole nut (end below the nut)", (gb.min.Z - 2.0) - screw_end,
         lambda q: q >= 0, "ENGAGED", "SHORT", "mm")
    need(f"nut + screw tail to the deck (room under the plate {-deck_z:.2f}, the tray resting {drop:.2f} down)",
         min(gb.min.Z - 2.0, screw_end) - deck_z, lambda q: q >= 0.5, "CLEAR", "HITS THE DECK", "mm")
    print(f"stack (tray frame, plate z 0..{T:.0f}): IMU z {imu_z0:.1f}..{imu_z0 + IMU_BOARD[2]:.1f}, "
          f"Pi board z {top:.1f}..{top + PI_BOARD[2]:.1f}, its USB stack to z "
          f"{pi_t.bounding_box().max.Z:.2f}, deck top z {deck_z:.2f}")

    # ---- B127: the Pi 5's real height vs the carapace, a 3D gap with the tray's float ----------
    # The float is the tray's x / y play (+-FIT) at the lugs' centred height, as the prep measured
    # it (6.84 / 3.56). Latched, the tongue sits FIT lower (the boss cammed onto the strike) and
    # the bulkhead end can rise FIT to the lip: both for the record
    from part_shell import shell_sector, shell_cap
    sec, cap = shell_sector(), shell_cap()
    shell = [Rot(0, 0, a) * sec for a in STATIONS] + [Rot(0, 0, STATIONS[0]) * cap]   # as part_shell poses them
    carapace = Compound(children=shell)

    def gap_at(ss):
        d, p, q = Compound(children=ss).distance_to_with_closest_points(carapace)
        r, az = float(np.hypot(q.X, q.Y)), float(np.rad2deg(np.arctan2(q.Y, q.X)) % 360)
        return d, f"carapace at ({q.X:.1f}, {q.Y:.1f}, {q.Z:.1f}) r {r:.1f} az {az:.0f}, the stack at " \
                  f"({p.X:.1f}, {p.Y:.1f}, {p.Z:.1f})"
    xy_float = [(dx, dy) for dx in (-FIT, 0.0, FIT) for dy in (-FIT, 0.0, FIT)]
    for usb, tag, gate in ((PI_USB_SIGN, f"USB end {AT['pi_usb_end']} (pick 14)", True),
                           (-PI_USB_SIGN, "USB end the other way (for the record)", False)):
        env = solids(tray, boss_t, pi5_envelope(usb))
        gs = {f: gap_at([tray_tf(dx=f[0], dy=f[1]) * s for s in env]) for f in xy_float}
        (g0, w0), fw = gs[(0.0, 0.0)], min(gs, key=lambda f: gs[f][0])
        gz = {dz: gap_at([tray_tf(dz=dz) * s for s in env])[0] for dz in (-FIT, FIT)}
        msg = (f"tray + Pi 5 (STEP, {PI_STANDOFF_H:g} standoffs), {tag}, to the carapace (3D): {g0:.2f} centred "
               f"({w0}); {gs[fw][0]:.2f}..{max(g[0] for g in gs.values()):.2f} over the x/y float +-{FIT} (worst at "
               f"{fw}: {gs[fw][1]}); latched (tongue {FIT} down) {gz[-FIT]:.2f}, bulkhead end up {FIT} to the lip "
               f"{gz[FIT]:.2f}")
        if gate:
            need(msg, gs[fw][0], lambda q: q >= 3.0, "PASSES the 3 mm rule", "FAILS the 3 mm rule", "mm")
        else:
            print(msg + f" ({'passes' if gs[fw][0] >= 3.0 else 'fails'} the 3 mm rule)")
    # pick 14 puts the bus adapter's UART (pins 8 / 10 + a GND on 9 or 14) and the buck's 5 V feed
    # (2 / 4 / 6) on the GPIO header. The STEP has its pins only: ESTIMATE a 2.54 female housing 14
    # long on the header's 2.5 spacer plus 6 of wire bend, over pins 1..14 (both rows; pin 1 is at
    # the far end, STEP x 8, as on every 40-pin Pi)
    ref5 = pi5_ref()
    gx0, gy0, _, _, gy1, _ = next(g for g in ref5["groups"] if g["name"].startswith("gpio"))["boxes"][0]
    ptop = ref5["pcb"][5]
    dup = pi5_box([gx0 + 0.32 - 1.27, gy0 + 0.32 - 1.27, ptop + 2.5, gx0 + 0.32 + 6 * 2.54 + 1.27,
                   gy1 - 0.32 + 1.27, ptop + 2.5 + 14.0 + 6.0])
    gd = min(gap_at([tray_tf(dx=f[0], dy=f[1]) * dup])[0] for f in xy_float)
    gd0 = gap_at([tray_tf() * dup])
    print(f"  (the GPIO header's Dupont housings over pins 1..14, ESTIMATE 14 + 6 of bend: {gd0[0]:.2f} to the "
          f"carapace centred ({gd0[1]}), {gd:.2f} worst float; the housing top at body z "
          f"{(tray_tf() * dup).bounding_box().max.Z:.2f})")
    print("  (unmodelled: the Active Cooler or any HAT over the SoC)")

    # ---- pick 14: the USB end +x, the lower middle port's plug room -----------------------------
    px, py, pz, pd = pi5_port("usb_middle", "lower")
    face = (tray_tf() * Pos(px, py, pz)).position
    dirx = int(round(np.cos(np.deg2rad(TRAY_ROT)))) * pd     # the port's outward x, body
    loom4 = station_tf(4) * harness_solid()
    obst = {"the carapace": boxed(shell),
            "leg 4's docked coxa base + yaw servo + its loom keep-out": boxed(bases[4], yaws[4], loom4),
            "leg 4's fork + hip servo (yaw -40..40)": boxed([forks[(4, a)] for a in YAWS])}

    def room(section, obstacles, shift=(0.0, 0.0, 0.0), lo=0.0, hi=80.0):
        """How far a plug of `section` (w along y, h along z) runs out of the port before it meets
        `obstacles` (bisection), the tray (and the port) moved by `shift`."""
        w, hh = section
        for _ in range(18):
            mid = (lo + hi) / 2
            p = Pos(face.X + shift[0] + dirx * mid / 2, face.Y + shift[1], face.Z + shift[2]) * Box(mid, w, hh)
            if xvol(p, obstacles) > 1e-4:
                hi = mid
            else:
                lo = mid
        return lo
    print(f"Pi USB face (lower middle port, USB 3) at body x {face.X:.2f}, y {face.Y:.2f}, centre z {face.Z:.2f} "
          f"(USB end {'+x, toward leg 4' if dirx > 0 else '-x'}; PI_BOARD_DX {PI_BOARD_DX:+g} in the tray frame)")
    every = sum(obst.values(), [])
    for sec_, tag in (((14.0, 7.0), "a right-angle overmold 14 x 7"), ((12.0, 4.5), "a bare 12 x 4.5 shell")):
        rooms = {k: room(sec_, o) for k, o in obst.items()}
        worst_f = min(room(sec_, every, (dx, dy, 0.0)) for dx in (-FIT, FIT) for dy in (-FIT, FIT))
        up = room(sec_, every, (0.0, 0.0, FIT))
        print(f"  room for {tag}: " + ", ".join(f"{v:.2f} to {k}" for k, v in rooms.items()) +
              f"; worst x/y float {worst_f:.2f}; the bulkhead end up {FIT} {up:.2f}")
        if sec_[1] == 7.0:
            need("  ... a right-angle plug <= 20 long (overmold <= 14 x 7) fits the lower middle port", worst_f,
                 lambda q: q >= 20.0, "FITS", "NO ROOM", "mm")

    # ---- I3: the tongue's cartridge on the deck's strike (0, -43) ------------------------------
    from part_panel import latch_seat, latch_engagement, latch_verdict, latch_report
    from iface import latch_insert_rotor, LATCH_ENTRY_DEG, LATCH_LAND, LATCH_GAP, LATCH_SHAFT_D, LUG_R
    ms = latch_seat(tray.fuse(boss_t), tongue_latch_tf())
    need("latch housing on its index in the tongue + boss x tray + boss", ms["seated"], lambda q: q < 0.01,
         "SEATED", "CLASH")
    need("  ... turned +-10 deg or half a turn", ms["turned"], lambda q: q >= 1.0, "INDEXED", "LOOSE")
    st = next(e for e in DECK_HOLES if e[2] == "strike")
    strike_tf = Pos(st[1][0], st[1][1], DECK_TOP) * Rot(0, 0, st[3][0])
    patch = Pos(st[1][0], st[1][1], DECK_BOT_Z + 3) * Box(40, 40, 6) - Pos(0, 0, DECK_BOT_Z) * deck_hole_cutter(st)
    lf = tray_tf(rest=True) * tongue_latch_tf()
    off = (lf.position - strike_tf.position).length
    xa = (lf * Pos(1, 0, 0)).position - lf.position
    ang = abs((np.rad2deg(np.arctan2(xa.Y, xa.X)) - st[3][0] + 180) % 360 - 180)
    need(f"the boss's seating face on the strike's frame (DECK_HOLES '{st[0]}' rot {st[3][0]:g}: its +x = body "
         f"-x; the tongue frame's +x = the tray's +x turned {TONGUE_LATCH_ROT:g}, posed {TRAY_ROT:g})", off + ang,
         lambda q: q < 0.01, "MATCHES", "OFF", "mm + deg")
    m = latch_engagement(patch, lf)
    v = latch_verdict(m)
    need(f"I3 latch, the tray resting, on a deck patch cut with DECK_HOLES' strike: {latch_report(m)}",
         float(len(v)), lambda q: q == 0, "LATCHES", "FAIL: " + "; ".join(v), "faults")
    bare = latch_engagement(patch, tray_tf(rest=True) * Pos(0, TONGUE_Y, 0))
    print(f"  (without the boss the seating face is the tongue's underside, {BOSS_H:.2f} over the strike: the "
          f"lugs' tops {BOSS_H - LATCH_LAND - LATCH_GAP:+.2f} over the deck top instead of "
          f"{-LATCH_LAND - LATCH_GAP:.2f}, "
          f"short by {BOSS_H:.2f}; verdict {latch_verdict(bare) or 'none'})")

    # the latch vs the release. At OPEN the rotor hangs LATCH_REACH (5.8) into the strike and rises
    # at most LATCH_LAND + LATCH_GAP (3.8) before its lugs meet the housing, so 2.0 of shaft stays
    # in the bore: in a round strike it pins the slide. The fix, measured here, not cut anywhere:
    # slotted_strike_proposal with the tongue's frame turned 45 and the deck's strike turned 225
    def rotor_at(s, z, rot, phi=0.0):
        return tray_tf(s, z, rest=True) * tongue_latch_tf(rot) * Rot(0, 0, LATCH_ENTRY_DEG + phi) * latch_insert_rotor()
    pinned = [vol(rotor_at(s, 0.0, None) & patch) for s in (0.5, 1.0, 2.0)]
    pushed = vol((tray_tf(0.5, 0.0, rest=True) * tongue_latch_tf() * Pos(0, 0, LATCH_LAND + LATCH_GAP) *
                  Rot(0, 0, LATCH_ENTRY_DEG) * latch_insert_rotor()) & patch)
    print(f"the latch vs the release, as cut (round strike): OPEN, the tray slid 0.5 / 1 / 2 toward the tongue: "
          f"rotor x strike {pinned[0]:.2f} / {pinned[1]:.2f} / {pinned[2]:.2f} mm^3, the rotor pushed up its "
          f"{LATCH_LAND + LATCH_GAP:.1f} {pushed:.2f} ({'OPEN: THE CARTRIDGE PINS THE SLIDE, the tray does not come '
          'out; not gated, the fix below needs iface + part_deck' if max(pinned) > 0.01 else 'free'})")
    sdir = (tray_tf(1.0) * Pos(0, 0, 0)).position - (tray_tf() * Pos(0, 0, 0)).position
    srot = TRAY_ROT + 45.0
    spatch = Pos(st[1][0] + 12 * sdir.X, st[1][1] + 12 * sdir.Y, DECK_BOT_Z + 3) * Box(40, 60, 6) - \
        Pos(st[1][0], st[1][1], DECK_TOP) * Rot(0, 0, srot) * slotted_strike_proposal(6.0)
    sm = latch_engagement(spatch, tray_tf(rest=True) * tongue_latch_tf(45.0))
    sv = latch_verdict(sm)
    s_end = LUG_RELEASE + 0.5
    s_path = max(vol(rotor_at(s, 0.0, 45.0) & spatch) for s in np.arange(0.0, s_end + 1e-9, 0.5))
    s_lift = max(vol(rotor_at(LUG_RELEASE, z, 45.0) & spatch) for z in (0.5, 1, 2, 4, 6, 8, 10))
    s_stop = vol(rotor_at(s_end + 0.5, 0.0, 45.0) & spatch)
    s_hold = vol(rotor_at(0.5, 0.0, 45.0, 90.0) & spatch)
    print(f"  PROPOSAL (not gated, cut nowhere): the strike drawn out {s_end + LUG_R:.1f} along the slide, "
          f"{LATCH_SHAFT_D + 2 * FIT:.1f} wide, through (slotted_strike_proposal), the deck's strike rot "
          f"{srot:g}, the tongue's frame 45: {'LATCHES' if not sv else 'FAILS: ' + '; '.join(sv)} "
          f"({latch_report(sm)}); OPEN slid 0..{s_end:g} x strike {s_path:.2f}, then lifted 0.5..10 at "
          f"{LUG_RELEASE:g} {s_lift:.2f} mm^3; the slot's end stops it at "
          f"{s_end + 0.5:g} ({s_stop:.2f}); LOCKED, slid 0.5: {s_hold:.2f} mm^3 "
          f"({'holds the slide' if s_hold > 1 else 'DOES NOT HOLD'})")

    # ---- the trunk + the tray XT30 down the deck's (0, 50) slot (B122) --------------------------
    sl = next(e for e in DECK_HOLES if e[0] == "trunk_slot")
    (sx_, sy_), (sw, sh, sr) = sl[1], sl[3]
    col = Pos(sx_, sy_, DECK_BOT_Z) * extrude(RectangleRounded(sw, sh, sr), 80.0)
    bh_out = (tray_tf() * Pos(0, -L / 2, 0)).position.Y                # the bulkhead's outer face, body y
    for name, k in TRUNK_CUTS.items():
        w, hh, x, z = BULKHEAD_CUTS[k]
        c = (tray_tf() * Pos(x, -L / 2, T + z)).position
        dxo = max(0.0, abs(c.X - sx_) + w / 2 - sw / 2)
        print(f"  bulkhead {name} cutout ({w:.1f} wide) at body x {c.X:+.1f}, z {c.Z:.1f} on the face y {bh_out:.1f}: "
              f"the slot's near edge {sy_ - sh / 2 - bh_out:.2f} north of the face (its centre {sy_ - bh_out:.2f}), "
              f"{abs(c.X - sx_):.2f} off its axis, {dxo:.2f} of the cutout past its {sw:g} width")
    for s, tag in ((0.0, "at rest"), (LUG_RELEASE, f"slid {LUG_RELEASE:g} south (the release)")):
        need(f"the (0, 50) slot's column, deck bottom up, x tray + boss + Pi {tag}",
             xvol(col, [tray_tf(s) * q for q in moving_t]), lambda q: q < 0.01, "OPEN", "COVERED")
    print(f"  (slid {LUG_RELEASE:g} north instead: {xvol(col, [tray_tf(-LUG_RELEASE) * q for q in moving_t]):.1f} "
          f"mm^3 over the slot, so the release is south; the trunk and the XT30 unplug at the bulkhead first)")
    print("  (B123: the J6 foot-switch lines have no bulkhead connector: the tray does not come out with them on)")

    # ---- pick 14, print ------------------------------------------------------------------------
    print("bus adapter, 5 V buck, 6 V UBEC: on the hub shelf, face-down (decision 14); BOARDS / board_envelope "
          "stay here for part_busboard")
    for n, p in (("avionics_tray", tray), ("tray_rail", rail), ("tray_latch_boss", boss)):
        b = p.bounding_box()
        fits = sorted((b.size.X, b.size.Y))[1] <= max(PR["bed_mm"][:2]) and \
            sorted((b.size.X, b.size.Y))[0] <= min(PR["bed_mm"][:2]) and b.size.Z <= PR["bed_mm"][2]
        need(f"{n}: {len(p.solids())} solid, {b.size.X:.1f} x {b.size.Y:.1f} x {b.size.Z:.1f}",
             float(fits and len(p.solids()) == 1),
             lambda q: q > 0.5, "ONE SOLID, FITS THE BED", "SPLIT OR TOO BIG", "")
    print(f"part_avionics checks: {'CLEAN' if not bad else 'FAIL — ' + '; '.join(bad)}")
    raise SystemExit(1 if bad else 0)
