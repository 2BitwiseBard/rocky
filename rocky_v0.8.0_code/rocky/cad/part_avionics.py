"""Avionics tray (I4) — Pi 5 + bus adapter + BEC + IMU on one slide-out.

  avionics_tray : plate with side rails, Pi 5 + adapter standoffs, IMU pad
                  (TPU-grommet holes, backlog B4), latch tongue at the front
                  (I3 insert pocket), rear bulkhead wall with the connector
                  cutouts. One latch, one pull, everything out.
  tray_rail     : deck-grid rail block, 2 needed (mirror = same part flipped).

Cutouts sized for: XT30 panel (10.4x6.4), JST-XH 5-pin (16.6x6.2), 2x
JST-SH (6.4x3.2), Qwiic/JST-SH 4-pin I2C (7.4x3.2), USB-C slot (10x4).
All VERIFY against the real connectors before printing five of anything —
one tray is cheap, that's the point of the standard.
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

# Pi 5 mounting: 58 x 49 hole rectangle, M2.5
PI_HOLES = [(-29, -24.5), (29, -24.5), (-29, 24.5), (29, 24.5)]


def avionics_tray():
    tray = Pos(0, 0, T / 2) * Box(W, L, T)
    # side rail wings that ride the deck rail blocks
    for sx in (1, -1):
        tray += Pos(sx * (W / 2 + RAIL / 2), 0, T - 1.0) * Box(RAIL, L - 16, 2.0)
    # Pi 5 standoffs (M2.5 thread-forming, 6 tall) — Pi occupies the center
    for hx, hy in PI_HOLES:
        s = Pos(hx * 1.0, hy * 0.9, T + 3) * Cylinder(3.4, 6)
        s -= Pos(hx * 1.0, hy * 0.9, T + 3.5) * Cylinder(2.05 / 2, 7)
        tray += s
    # bus-adapter pad: 2x M3 grid holes
    for hx in (-6, 14):
        tray -= Pos(hx, -L / 2 + 8, T / 2) * Cylinder(PR["screw_m3_tap"] / 2, T + 2)
    # IMU pad: 4x Ø4.8 grommet holes (BNO085 board on TPU grommets, B4) at center
    for gx, gy in ((-8, -6), (8, -6), (-8, 6), (8, 6)):
        tray -= Pos(gx, gy + L / 4 - 4, T / 2) * Cylinder(4.8 / 2, T + 2)
    # weight-relief + zip-tie field
    for zx in (-30, 30):
        for zy in (-18, 0, 18):
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
    print(f"tray+rails half-span {half_span:.0f} mm vs deck grid zone 56 mm "
          f"({'FITS the grid zone' if half_span <= 56 else 'CHECK deck layout'})")
    # rail engagement: pose one rail so its open channel faces the tray and
    # swallows the +x wing. Rot(z,90): local +y face -> posed -x (inboard).
    # Open face lands at plate edge + 0.3 clearance: x = 42.3 -> block center
    # x = 47.3; channel z center 5.4 -> offset so it hits the wing at z=2.
    posed = Pos(W / 2 + 5.3, 0, (T - 1.0) - 5.4) * Rot(0, 0, 90) * rail
    inter = tray & posed
    v = 0.0 if inter is None else inter.volume
    print(f"tray wing x rail channel: {v:.2f} mm^3 ({'SLIDES' if v < 1 else 'BINDS'})")
    # and the wing must actually be INSIDE the channel (engagement > 2 mm):
    lifted = Pos(0, 0, 2.5) * posed               # lift rail: should now hit wing
    inter = tray & lifted
    v2 = 0.0 if inter is None else inter.volume
    print(f"rail lifted 2.5 mm: intersection {v2:.1f} mm^3 "
          f"({'ENGAGED (wing captive)' if v2 > 5 else 'wing not captive?'})")
