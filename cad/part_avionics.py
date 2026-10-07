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
the IMU nuts against the deck; B107 the tongue's latch pocket on its keyway
index. B108: the bus adapter, the buck and the UBEC as envelopes from their
datasheets, posed where the docs put them and searched for any flat place on
the floor. None has one under the Pi on its 11 mm standoffs (the adapter's
jack reaches z 18.1 against the keep-out's 12.0), so that audit is reported,
not gated: their place is the owner's call with the tray's (B51). Not
checked here: the tray's place between the legs (B51).
"""
from build123d import *
from common import params, export
from iface import IF, latch_pocket

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
AT = IF["avionics_tray"]
PL = IF["panel_latch"]

W, L, T = AT["plate_w"], AT["plate_l"], AT["plate_t"]     # 84 x 70 x 3
RAIL = AT["rail"]                                          # 3
BH_W, BH_H = AT["bulkhead_w"], AT["bulkhead_h"]            # 70 x 26
BH_T = 2.4                                                 # the rear bulkhead wall

# Pi 5 mounting (mechanical drawing RP-008347-DS): 85 x 56 x 1.6 board, 4 x Ø2.7 holes
# 3.5 mm in from the edges = a 58 x 49 rectangle, M2.5. The rectangle is off the
# board's centre along x: the USB/Ethernet end runs 52.5 past its centre, the far end 32.5.
PI_HOLES = [(-29, -24.5), (29, -24.5), (-29, 24.5), (29, 24.5)]
PI_BOARD = (85.0, 56.0, 1.6)
PI_BOARD_DX = 10.0                   # board centre off the hole-pattern centre, toward the USB end
PI_UNDERSIDE = 2.0                   # keep-out under the board: solder tails, microSD, passives
PI_STANDOFF_H = AT["pi_standoff_h"]  # 11. D063 (B90): 6 -> 11, so the IMU fits under the Pi (see __main__);
                                     # in params since 2026-10-07 (pick 14 keeps it), so the carapace and
                                     # USB-plug checks read the same number (BODY_LAYOUT s11: a copied 6
                                     # once overstated A's carapace margin by 5)
# BNO085 (Adafruit 4754 board file): 25.4 x 22.86 board, 4.6 tall with its parts, 4 x Ø2.5
# holes 2.54 in from the edges = 20.32 x 17.78. It sits on four imu_grommets (B4) under the
# Pi: an 84 x 70 plate has no 28 x 26 patch (board + grommet flanges) beside an 85 x 56 Pi
# (the free strips are 9.5 and 7 mm wide), so the Pi goes up instead.
IMU_BOARD = (25.4, 22.86, 4.6)
IMU_HOLES = [(sx * 10.16, sy * 8.89) for sy in (-1, 1) for sx in (-1, 1)]
IMU_XY = (0.0, 13.5)                 # board centre: north of the bus-adapter pad, as before
IMU_SEAT_LIFT = 1.2                  # each grommet seat is the 3 mm plate lifted 1.2 (see avionics_tray)
IMU_SEAT_D = 11.0                    # each seat's boss (see avionics_tray)
ZIP_X, ZIP_Y = 30.0, (-14.0, 0.0, 14.0)   # zip-tie slots (buck, UBEC): y +-18 cut under the
                                          # standoff feet; +-14 leaves 3.1 mm
# B108: the tray's other three boards, as envelopes (plan w x l, parts `up` above the board's
# underside, pins `down` below it; a board without a mount carries a zip tie over its top).
# Sources: the BOM rows and their datasheets, as fetched 2026-09-30.
#  bus_adapter: Waveshare Bus Servo Adapter (A), BOM A-02. 42 x 33, Ø2.5 holes on 37 x 28
#    (waveshare.com/bus-servo-adapter-a.htm). Heights measured on Waveshare's own STEP model
#    (wiki "Bus Servo Adapter (A)", Resources, 3D model): a 1.6 board, parts to 11.0 above it
#    (the DC5521 jack; pin headers 9.0, the screw terminal 8.4, the servo ports 5.9) and
#    through-hole pins 2.0 below it. Held on 2.5 mm standoffs at its own holes (pins + 0.5).
#    Its four holes (37 x 28; diagonal 46.4) match no pair of the pad's two M3 holes
#    (ADAPTER_PAD_HX, 20 apart): the pad is not a mount for this board.
#  buck_5v: Pololu D24V50F5, BOM B-10. 0.7 x 0.8 x 0.35 in = 17.8 x 20.3 x 8.8 overall
#    (pololu.com/product/2851). Its 1000 uF capacitor is not modelled: its size is the part
#    bought's (VERIFY).
#  ubec_6v: Hobbywing UBEC-3A, BOM B-11. 43 x 17 x 7 (hobbywingdirect.com, UBEC-3A specs).
# TIE_T: a 3.6 mm nylon zip tie over a board's top, ~1.2 thick (guessed, not measured).
TIE_T = 1.2
ADAPTER_PAD_HX = (-6.0, 14.0)        # the bus-adapter pad's two M3 holes, at y -L/2 + 8
BOARDS = {
    "bus_adapter": dict(w=42.0, l=33.0, up=1.6 + 11.0, down=2.0, lift=2.5, tie=0.0),
    "buck_5v": dict(w=17.8, l=20.3, up=8.8, down=0.0, lift=0.0, tie=TIE_T),
    "ubec_6v": dict(w=43.0, l=17.0, up=7.0, down=0.0, lift=0.0, tie=TIE_T),
}
# where the docs put them (WIRING_HARNESS, ASSEMBLY_GUIDE 4.5): the adapter on its pad by the
# bulkhead (its south edge 0.5 off the bulkhead's inner face, centred on the pad's holes),
# the buck and the UBEC tied over the zip slots at x +30 / -30 (tray frame, rotation deg)
BOARD_DOC_POSE = {
    "bus_adapter": (sum(ADAPTER_PAD_HX) / 2, -L / 2 + BH_T + 0.5 + BOARDS["bus_adapter"]["l"] / 2, 0),
    "buck_5v": (ZIP_X, -7.0, 0),
    "ubec_6v": (-ZIP_X, 0.0, 90),
}
BOARD_CLEAR = 1.0                    # to the Pi's underside keep-out, as the IMU (B90)
# where the tray sits on the deck (body frame): the ASSUMED layout of
# docs/WIRING_HARNESS.md (star board), a proposal until the deck layout freezes. The
# star-board bracket's layout audit (part_busboard) keeps clear of it.
TRAY_XY = (0.0, -38.0)
RAIL_POSE_X = W / 2 + 5.3            # rail block centre off the tray centre (see __main__)


def tongue_latch_tf():
    """The tongue's latch frame: z 0 on the tongue's underside (the housing flush with it),
    +z up; the tray's +x taken as the strike's until the deck has one under it (B51)."""
    return Pos(0, L / 2 + 8, 0)


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
    # bus-adapter pad: 2x M3 grid holes. B108: the adapter (42 x 33, 14.6 tall with its
    # pins) has no place here under the Pi (BOARD_DOC_POSE); the holes stay until it has one
    for hx in ADAPTER_PAD_HX:
        tray -= Pos(hx, -L / 2 + 8, T / 2) * Cylinder(PR["screw_m3_tap"] / 2, T + 2)
    # IMU pad: 4x Ø4.8 grommet holes (BNO085 on TPU grommets, B4) on the board's 20.32 x 17.78.
    # D063 (B90): each seat is the 3 mm plate (the grommet's grip) lifted IMU_SEAT_LIFT, a Ø11
    # boss on top over a Ø9 recess below that takes the grommet's 1.2 mm bottom flange. Flat,
    # the flange + a DIN 934 nut + the M2.5 x 10's tail hung 3.4 under the plate with the deck
    # 3.1 down; lifted, they end 2.2 down (0.9 clear)
    for hx, hy in IMU_HOLES:
        x, y = IMU_XY[0] + hx, IMU_XY[1] + hy
        tray += Pos(x, y, T + IMU_SEAT_LIFT / 2) * Cylinder(IMU_SEAT_D / 2, IMU_SEAT_LIFT)
        tray -= Pos(x, y, (IMU_SEAT_LIFT - 1) / 2) * Cylinder(9 / 2, IMU_SEAT_LIFT + 1)
        tray -= Pos(x, y, (T + IMU_SEAT_LIFT) / 2) * Cylinder(4.8 / 2, T + IMU_SEAT_LIFT + 2)
    # weight-relief + zip-tie field
    for zx in (-ZIP_X, ZIP_X):
        for zy in ZIP_Y:
            tray -= Pos(zx, zy, T / 2) * Box(4, 8, T + 2)
    # front latch tongue with I3 insert pocket: the housing flush with the tongue's
    # underside, the D of the keyway index (B107), the tray's +x taken as the strike's
    # (none yet, B51)
    tongue = Pos(0, L / 2 + 8, T / 2) * Box(30, 16, T)
    tongue -= tongue_latch_tf() * latch_pocket(-1.0, T + 1.0)
    tray += tongue
    # rear bulkhead wall + connector cutouts
    bh = Pos(0, -L / 2 + BH_T / 2, T + BH_H / 2) * Box(BH_W, BH_T, BH_H)
    cuts = [  # (w, h, x, z) on the bulkhead face
        (10.4 + FIT, 6.4 + FIT, -24, 8),          # XT30 power in
        (16.6 + FIT, 6.2 + FIT, -2, 8),           # JST-XH 5-pin bus
        (6.4 + FIT, 3.2 + FIT, 12, 8),            # JST-SH spare
        (6.4 + FIT, 3.2 + FIT, 21, 8),            # JST-SH spare
        (7.4 + FIT, 3.2 + FIT, -24, 18),          # Qwiic I2C (sensor mux)
        (10.0, 4.0, 0, 18),                       # USB-C pass slot
    ]
    for w, h, x, z in cuts:
        bh -= Pos(x, -L / 2 + BH_T / 2, T + z) * Box(w, 4, h)
    tray += bh
    return tray


def board_envelope(name, x, y, rot=0):
    """B108: a board's envelope on the tray floor (plate top z T), from its pins (or the
    plate) up through its parts and its zip tie; rot 0 or 90 about z."""
    b = BOARDS[name]
    z0 = T + b["lift"] - b["down"]
    z1 = T + b["lift"] + b["up"] + b["tie"]
    w, l = (b["w"], b["l"]) if rot % 180 == 0 else (b["l"], b["w"])
    return Pos(x, y, (z0 + z1) / 2) * Box(w, l, z1 - z0)


def board_spots(name, standoff_h=None, usb_dx=(None,), step=1.0, others=()):
    """B108: every plan pose (1 mm grid, rot 0 / 90) where a board lies flat on the tray
    floor: on the plate (0.5 in from its edges and the bulkhead), 0.5 clear of the Pi
    standoffs, the IMU (board + grommet seats) and `others` ([(x, y, w, l)]), and, under
    the Pi (USB end +x and -x unless usb_dx says), its top BOARD_CLEAR under the keep-out.
    Plan boxes and circles, not solids: a search, the solids check the poses it finds."""
    b = BOARDS[name]
    h = standoff_h if standoff_h is not None else PI_STANDOFF_H
    keep = T + h - PI_UNDERSIDE
    top = T + b["lift"] + b["up"] + b["tie"]
    dxs = (PI_BOARD_DX, -PI_BOARD_DX) if usb_dx == (None,) else usb_dx
    imu = (IMU_XY[0], IMU_XY[1], 2 * max(abs(hx) for hx, _ in IMU_HOLES) + IMU_SEAT_D,
           2 * max(abs(hy) for _, hy in IMU_HOLES) + IMU_SEAT_D)

    def hits(a, c, gap):
        return abs(a[0] - c[0]) < (a[2] + c[2]) / 2 + gap and abs(a[1] - c[1]) < (a[3] + c[3]) / 2 + gap
    out = []
    for w, l in {(b["w"], b["l"]), (b["l"], b["w"])}:
        for i in range(int(W / step) + 1):
            x = -W / 2 + i * step
            if abs(x) + w / 2 > W / 2 - 0.5:
                continue
            for j in range(int(L / step) + 1):
                y = -L / 2 + j * step
                if y - l / 2 < -L / 2 + BH_T + 0.5 or y + l / 2 > L / 2 - 0.5:
                    continue
                r = (x, y, w, l)
                if any(max(abs(px - x) - w / 2, 0) ** 2 + max(abs(py - y) - l / 2, 0) ** 2 < (3.4 + 0.5) ** 2
                       for px, py in PI_HOLES):
                    continue
                if hits(r, imu, 0.5) or any(hits(r, o, 0.5) for o in others):
                    continue
                if top > keep - BOARD_CLEAR and any(hits(r, (dx, 0, PI_BOARD[0], PI_BOARD[1]), 0.0)
                                                   for dx in dxs):
                    continue
                out.append(r)
    return out


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
    # B108: the bus adapter, the buck and the UBEC. Each envelope where the docs put it,
    # against the tray, the rails, the IMU + grommets and the Pi + its keep-out (USB end
    # either way), then the plan search for any flat place on this tray. Reported, not
    # gated: none has one, and where they go is the owner's call with the tray's layout
    # (B51, docs/BODY_LAYOUT_PROPOSAL.md); B108 has the measured options
    keep_z = top - PI_UNDERSIDE
    pis = [Pos(dx, 0, (keep_z + top + PI_BOARD[2]) / 2) *
           Box(PI_BOARD[0], PI_BOARD[1], top + PI_BOARD[2] - keep_z) for dx in (PI_BOARD_DX, -PI_BOARD_DX)]
    for name, (bx, by, rot) in BOARD_DOC_POSE.items():
        env = board_envelope(name, bx, by, rot)
        spots = board_spots(name)
        print(f"{name} where the docs put it ({bx:g}, {by:g}): x tray {vol(env & tray):.1f}, rails "
              f"{vol(env & rails):.1f}, IMU + grommets {vol(env & (imu + grommets)):.1f}, Pi + keep-out "
              f"{max(vol(env & p) for p in pis):.1f} mm^3; top z {env.bounding_box().max.Z:.1f} against the "
              f"keep-out's {keep_z:.1f}; flat anywhere on this tray with {BOARD_CLEAR:g} under the keep-out: "
              + (f"{len(spots)} poses" if spots else "NO PLACE") + " (B108, the owner's call)")
    # B107: the tongue's pocket holds a latch housing on its keyway index, one way
    from part_panel import latch_seat
    ms = latch_seat(tray, tongue_latch_tf())
    need("latch housing on its index in the tongue x tray", ms["seated"], lambda q: q < 0.01, "SEATED", "CLASH")
    need("  ... turned +-10 deg or half a turn", ms["turned"], lambda q: q >= 1.0, "INDEXED", "LOOSE")
    print(f"stack (tray frame, plate z 0..{T:.0f}): IMU z {imu_z0:.1f}..{imu_z0 + IMU_BOARD[2]:.1f}, "
          f"Pi board z {top:.1f}..{top + PI_BOARD[2]:.1f}, deck top z {deck_z:.2f}")
    print(f"part_avionics checks: {'CLEAN' if not bad else 'FAIL — ' + '; '.join(bad)}")
    raise SystemExit(1 if bad else 0)
