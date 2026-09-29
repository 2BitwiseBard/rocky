"""Avionics tray (I4) — Pi 5 + bus adapter + BEC + IMU on one slide-out.

  avionics_tray : plate with side rails, Pi 5 standoffs (58 x 49, 11 tall),
                  bus-adapter pad, IMU seats under the Pi (TPU grommets,
                  backlog B4), latch tongue at the front (I3 insert pocket),
                  rear bulkhead wall with the connector cutouts. One latch,
                  one pull, everything out.
  tray_rail     : deck-grid rail block, 2 needed (mirror = same part flipped).

Cutouts sized for: XT30 panel (10.4x6.4), JST-XH 5-pin (16.6x6.2), 2x
JST-SH (6.4x3.2), Qwiic/JST-SH 4-pin I2C (7.4x3.2), USB-C slot (10x4).
All VERIFY against the real connectors before printing five of anything —
one tray is cheap, that's the point of the standard.

Checks (run this file): the wing in the rail channel; D063 (B89, B90) the
standoffs on the Pi's holes, no cut under their feet, the BNO085 on its
grommets and the Pi (USB end either way) posed with their clearances, and
the IMU nuts against the deck. Not checked here: the tray's place between
the legs (B51) and the bus adapter, buck and UBEC (no board envelopes yet).
"""
from build123d import *
from common import params, export
from iface import IF

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
AT = IF["avionics_tray"]
PL = IF["panel_latch"]

W, L, T = AT["plate_w"], AT["plate_l"], AT["plate_t"]     # 84 x 70 x 3
RAIL = AT["rail"]                                          # 3
BH_W, BH_H = AT["bulkhead_w"], AT["bulkhead_h"]            # 70 x 26

# Pi 5 mounting (mechanical drawing RP-008347-DS): 85 x 56 x 1.6 board, 4 x Ø2.7 holes
# 3.5 mm in from the edges = a 58 x 49 rectangle, M2.5. The rectangle is off the
# board's centre along x: the USB/Ethernet end runs 52.5 past its centre, the far end 32.5.
PI_HOLES = [(-29, -24.5), (29, -24.5), (-29, 24.5), (29, 24.5)]
PI_BOARD = (85.0, 56.0, 1.6)
PI_BOARD_DX = 10.0                   # board centre off the hole-pattern centre, toward the USB end
PI_UNDERSIDE = 2.0                   # keep-out under the board: solder tails, microSD, passives
PI_STANDOFF_H = 11.0                 # D063 (B90): 6 -> 11, so the IMU fits under the Pi (see __main__)
# BNO085 (Adafruit 4754 board file): 25.4 x 22.86 board, 4.6 tall with its parts, 4 x Ø2.5
# holes 2.54 in from the edges = 20.32 x 17.78. It sits on four imu_grommets (B4) under the
# Pi: an 84 x 70 plate has no 28 x 26 patch (board + grommet flanges) beside an 85 x 56 Pi
# (the free strips are 9.5 and 7 mm wide), so the Pi goes up instead.
IMU_BOARD = (25.4, 22.86, 4.6)
IMU_HOLES = [(sx * 10.16, sy * 8.89) for sy in (-1, 1) for sx in (-1, 1)]
IMU_XY = (0.0, 13.5)                 # board centre: north of the bus-adapter pad, as before
IMU_SEAT_LIFT = 1.2                  # each grommet seat is the 3 mm plate lifted 1.2 (see avionics_tray)
ZIP_X, ZIP_Y = 30.0, (-14.0, 0.0, 14.0)   # zip-tie slots (buck, UBEC): y +-18 cut under the
                                          # standoff feet; +-14 leaves 3.1 mm
# where the tray sits on the deck (body frame): the ASSUMED layout of
# docs/WIRING_HARNESS.md (star board), a proposal until the deck layout freezes. The
# star-board bracket's layout audit (part_busboard) keeps clear of it.
TRAY_XY = (0.0, -38.0)
RAIL_POSE_X = W / 2 + 5.3            # rail block centre off the tray centre (see __main__)


def avionics_tray():
    tray = Pos(0, 0, T / 2) * Box(W, L, T)
    # side rail wings that ride the deck rail blocks
    for sx in (1, -1):
        tray += Pos(sx * (W / 2 + RAIL / 2), 0, T - 1.0) * Box(RAIL, L - 16, 2.0)
    # Pi 5 standoffs (M2.5 thread-forming, Ø2.05 bore the full height) on the Pi's own
    # 58 x 49 pattern. D063 (B89): until now the y of PI_HOLES was scaled by 0.9 (58 x 44.1,
    # since the first import), 2.45 mm off every Pi hole
    for hx, hy in PI_HOLES:
        s = Pos(hx, hy, T + PI_STANDOFF_H / 2) * Cylinder(3.4, PI_STANDOFF_H)
        s -= Pos(hx, hy, T + (PI_STANDOFF_H + 1) / 2) * Cylinder(2.05 / 2, PI_STANDOFF_H + 1)
        tray += s
    # bus-adapter pad: 2x M3 grid holes
    for hx in (-6, 14):
        tray -= Pos(hx, -L / 2 + 8, T / 2) * Cylinder(PR["screw_m3_tap"] / 2, T + 2)
    # IMU pad: 4x Ø4.8 grommet holes (BNO085 on TPU grommets, B4) on the board's 20.32 x 17.78.
    # D063 (B90): each seat is the 3 mm plate (the grommet's grip) lifted IMU_SEAT_LIFT, a Ø11
    # boss on top over a Ø9 recess below that takes the grommet's 1.2 mm bottom flange. Flat,
    # the flange + a DIN 934 nut + the M2.5 x 10's tail hung 3.4 under the plate with the deck
    # 3.1 down; lifted, they end 2.2 down (0.9 clear)
    for hx, hy in IMU_HOLES:
        x, y = IMU_XY[0] + hx, IMU_XY[1] + hy
        tray += Pos(x, y, T + IMU_SEAT_LIFT / 2) * Cylinder(11 / 2, IMU_SEAT_LIFT)
        tray -= Pos(x, y, (IMU_SEAT_LIFT - 1) / 2) * Cylinder(9 / 2, IMU_SEAT_LIFT + 1)
        tray -= Pos(x, y, (T + IMU_SEAT_LIFT) / 2) * Cylinder(4.8 / 2, T + IMU_SEAT_LIFT + 2)
    # weight-relief + zip-tie field
    for zx in (-ZIP_X, ZIP_X):
        for zy in ZIP_Y:
            tray -= Pos(zx, zy, T / 2) * Box(4, 8, T + 2)
    # front latch tongue with I3 insert pocket
    tongue = Pos(0, L / 2 + 8, T / 2) * Box(30, 16, T)
    tongue -= Pos(0, L / 2 + 8, T / 2) * Cylinder(PL["housing_pocket_d"] / 2, T + 2)
    tray += tongue
    # rear bulkhead wall + connector cutouts
    bh = Pos(0, -L / 2 + 1.2, T + BH_H / 2) * Box(BH_W, 2.4, BH_H)
    cuts = [  # (w, h, x, z) on the bulkhead face
        (10.4 + FIT, 6.4 + FIT, -24, 8),          # XT30 power in
        (16.6 + FIT, 6.2 + FIT, -2, 8),           # JST-XH 5-pin bus
        (6.4 + FIT, 3.2 + FIT, 12, 8),            # JST-SH spare
        (6.4 + FIT, 3.2 + FIT, 21, 8),            # JST-SH spare
        (7.4 + FIT, 3.2 + FIT, -24, 18),          # Qwiic I2C (sensor mux)
        (10.0, 4.0, 0, 18),                       # USB-C pass slot
    ]
    for w, h, x, z in cuts:
        bh -= Pos(x, -L / 2 + 1.2, T + z) * Box(w, 4, h)
    tray += bh
    return tray


def tray_footprint():
    """The tray's plan envelope on the deck, tray-local: [(cx, cy, w, h)] —
    plate + rail wings, both rail blocks with their bolt tabs, the latch
    tongue. For layout audits (a keep-out, not a model)."""
    rail_x0 = RAIL_POSE_X - 5.0                  # block 10 wide, tabs 8 more outboard
    rail_x1 = RAIL_POSE_X + 7.0 + 4.0
    return [(0.0, 0.0, W + 2 * RAIL, L),
            ((rail_x0 + rail_x1) / 2, 0.0, rail_x1 - rail_x0, 56.0),
            (-(rail_x0 + rail_x1) / 2, 0.0, rail_x1 - rail_x0, 56.0),
            (0.0, L / 2 + 8, 30.0, 16.0)]


def tray_rail():
    """Deck rail block: C-channel OPENING at its inner (+y local) face — the
    tray wing slides straight in. Bolt tabs on the closed side (outboard when
    posed), 20 mm deck-grid pitch."""
    L_R = 56
    blk = Pos(0, 0, 4) * Box(L_R, 10, 8)
    # channel: opens through the +y face, swallows the wing 3.6 deep
    blk -= Pos(0, 3.3, 4 + 1.4) * Box(L_R + 2, RAIL + 2 * FIT + 0.2, 2.0 + 2 * FIT)
    for sx in (-20, 0, 20):
        tab = Pos(sx, -7, 1) * Box(10, 8, 2)
        tab -= Pos(sx, -8, 1) * Cylinder(PR["screw_m3_clear"] / 2, 4)
        blk += tab
    return blk


if __name__ == "__main__":
    tray = avionics_tray()
    rail = tray_rail()
    export(tray, "avionics_tray")
    export(rail, "tray_rail")
    bb = tray.bounding_box()
    print(f"tray envelope {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm")
    # deck fit: tray + rails must fit inside the R100 pentagon's inner zone
    # (electronics grid r<56): W/2 + rail + block = 42+3+10/2... report it
    half_span = W / 2 + RAIL + 5
    bad = []
    if half_span > 56:
        bad.append("tray + rails leave the deck grid zone")
    print(f"tray+rails half-span {half_span:.0f} mm vs deck grid zone 56 mm "
          f"({'FITS the grid zone' if half_span <= 56 else 'CHECK deck layout'})")
    # rail engagement: pose one rail so its open channel faces the tray and
    # swallows the +x wing. Rot(z,90): local +y face -> posed -x (inboard).
    # Open face lands at plate edge + 0.3 clearance: x = 42.3 -> block center
    # x = 47.3; channel z center 5.4 -> offset so it hits the wing at z=2.
    posed = Pos(RAIL_POSE_X, 0, (T - 1.0) - 5.4) * Rot(0, 0, 90) * rail
    inter = tray & posed
    v = 0.0 if inter is None else inter.volume
    if v >= 1:
        bad.append("tray wing binds in the rail")
    print(f"tray wing x rail channel: {v:.2f} mm^3 ({'SLIDES' if v < 1 else 'BINDS'})")
    # and the wing must actually be INSIDE the channel (engagement > 2 mm):
    lifted = Pos(0, 0, 2.5) * posed               # lift rail: should now hit wing
    inter = tray & lifted
    v2 = 0.0 if inter is None else inter.volume
    if v2 <= 5:
        bad.append("tray wing not captive in the rail")
    print(f"rail lifted 2.5 mm: intersection {v2:.1f} mm^3 "
          f"({'ENGAGED (wing captive)' if v2 > 5 else 'wing not captive?'})")

    vol = lambda s: 0.0 if s is None else s.volume

    def need(name, val, good, ok, fail, unit="mm^3"):
        print(f"{name}: {val:.2f} {unit} ({ok if good(val) else fail})")
        if not good(val):
            bad.append(name)

    # D063 (B89): PI_HOLES is the drawing's pattern on the posed board (3.5 and 61.5 in from
    # the far end, 3.5 in from both long edges), the standoff axes are PI_HOLES (a Ø1.8 pin
    # down each hole's axis meets no plastic and the wall round the bore is solid out to
    # r 3.2: off-axis by > 0.13 mm fails), and no cut in the plate comes within 1 mm of a
    # standoff's foot (the zip slots did)
    far = PI_BOARD_DX - PI_BOARD[0] / 2
    want = [(far + ex, sy * (PI_BOARD[1] / 2 - 3.5)) for ex in (3.5, 61.5) for sy in (-1, 1)]
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
    need("plate missing within 1 mm of a standoff foot (zip slots, holes)", worst_foot,
         lambda q: q < 0.01, "CLEAR", "UNDERCUT")

    # D063 (B90): the BNO085 on its four grommets in the lifted seats, the Pi (85 x 56 x 1.6,
    # USB end either way) on the standoff tops with a 2 mm keep-out under it (less the Ø7
    # round each hole the standoff tops land on), the rails posed both sides
    from part_smallwins import imu_grommet
    rails = posed + Rot(0, 0, 180) * posed
    grommets = None
    for hx, hy in IMU_HOLES:
        g = Pos(IMU_XY[0] + hx, IMU_XY[1] + hy, T + IMU_SEAT_LIFT) * imu_grommet()
        grommets = g if grommets is None else grommets + g
    gb = grommets.bounding_box()
    imu_z0 = gb.max.Z                                  # the top flanges carry the board
    imu = Pos(IMU_XY[0], IMU_XY[1], imu_z0 + IMU_BOARD[2] / 2) * Box(*IMU_BOARD)
    need("imu_grommet x 4 in their seats x tray", vol(grommets & tray), lambda q: q < 1, "SEATED", "CLASH")
    need("  ... nudged 0.5 mm up or down (the flanges grip the 3 mm seat)",
         min(vol((Pos(0, 0, dz) * grommets) & tray) for dz in (0.5, -0.5)), lambda q: q > 1,
         "GRIPPED", "LOOSE")
    screws = None                                      # M2.5 x 10, head on the IMU's top face
    for hx, hy in IMU_HOLES:
        s = Pos(IMU_XY[0] + hx, IMU_XY[1] + hy, imu_z0 + 1.6 - 5.0) * Cylinder(2.5 / 2, 10)
        screws = s if screws is None else screws + s
    need("M2.5 through the BNO085's holes x grommet bores + tray", vol(screws & (grommets + tray)),
         lambda q: q < 0.01, "ON AXIS", "OFF AXIS")
    need("IMU envelope x tray", vol(imu & tray), lambda q: q < 0.01, "CLEAR", "CLASH")
    top = T + PI_STANDOFF_H
    worst = 0.0
    for dx in (PI_BOARD_DX, -PI_BOARD_DX):
        board = Pos(dx, 0, top + PI_BOARD[2] / 2) * Box(*PI_BOARD)
        under = Pos(dx, 0, top - PI_UNDERSIDE / 2) * Box(PI_BOARD[0], PI_BOARD[1], PI_UNDERSIDE)
        for hx, hy in PI_HOLES:
            under -= Pos(hx, hy, top - PI_UNDERSIDE / 2) * Cylinder(3.5, PI_UNDERSIDE + 1)
        worst = max(worst, vol(board & tray), vol(board & rails), vol(under & tray),
                    vol(under & rails), vol(under & imu), vol(under & grommets))
    need("Pi board + underside keep-out (USB end +x and -x) x tray, rails, IMU", worst,
         lambda q: q < 0.01, "CLEAR", "CLASH")
    need("IMU top to the Pi's underside keep-out", (top - PI_UNDERSIDE) - (imu_z0 + IMU_BOARD[2]),
         lambda q: q >= 1.0, "CLEAR", "TOO CLOSE", "mm")
    # the nuts: a DIN 934 M2.5 (5 A/F, 2 thick; its Ø5.8 corners) under each bottom flange,
    # the M2.5 x 10's tail below it, against the deck top with the tray resting on the
    # rails' channel floor
    drop = 0.0
    while drop < 2 and vol((Pos(0, 0, -(drop + 0.05)) * tray) & posed) < 0.01:
        drop += 0.05
    deck_z = posed.bounding_box().min.Z + drop
    nuts = None
    for hx, hy in IMU_HOLES:
        n = Pos(IMU_XY[0] + hx, IMU_XY[1] + hy, gb.min.Z - 1.0) * Cylinder(5.0 / 3 ** 0.5, 2.0)
        nuts = n if nuts is None else nuts + n
    need("IMU nuts x tray", vol(nuts & tray), lambda q: q < 0.01, "FIT", "CLASH")
    screw_end = imu_z0 + 1.6 - 10.0
    need("M2.5 x 10 runs through the whole nut (end below the nut)", (gb.min.Z - 2.0) - screw_end,
         lambda q: q >= 0, "ENGAGED", "SHORT", "mm")
    need(f"nut + screw tail to the deck (tray resting {drop:.2f} mm down)",
         min(gb.min.Z - 2.0, screw_end) - deck_z, lambda q: q >= 0.5, "CLEAR", "HITS THE DECK", "mm")
    print(f"stack (tray frame, plate z 0..{T:.0f}): IMU z {imu_z0:.1f}..{imu_z0 + IMU_BOARD[2]:.1f}, "
          f"Pi board z {top:.1f}..{top + PI_BOARD[2]:.1f}, deck top z {deck_z:.2f}")
    print(f"part_avionics checks: {'CLEAN' if not bad else 'FAIL — ' + '; '.join(bad)}")
    raise SystemExit(1 if bad else 0)
