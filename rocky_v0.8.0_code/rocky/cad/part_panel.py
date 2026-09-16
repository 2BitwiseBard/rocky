"""Panel standard demo set (I3) + accessory dovetail coupons (I6) — D020.

  shell_sector_demo : one carapace shell sector blank (1/5 of the pentagon
                      perimeter, flat demo version) carrying the standard:
                      seating lip, ONE latch insert pocket, 2 magnet pockets,
                      and TWO male dovetail segments (I6) on its outer face.
  frame_coupon      : the mating frame edge: lip groove + latch strike +
                      magnet pockets — print both, exercise the standard
                      before committing the real (curved, textured) shells.
  dovetail_shoe     : female accessory shoe (I6) with M3 set-knob bore.
  thumb_knob_m3     : printed knurled knob that captures an M3x10 — used by
                      the leg port (I1) and the dovetail shoe set screw.

The real shells stay D006 (cosmetic, bolt-on, sculpted later); this file
freezes how EVERY one of them will attach.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import (IF, latch_insert_housing, latch_insert_rotor,
                   dovetail_male, dovetail_female_shoe)

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
PL = IF["panel_latch"]
DV = IF["dovetail"]


def shell_sector_demo():
    W, H, T = 116, 64, 2.4                       # one sector blank (flat demo)
    p = Pos(0, 0, T / 2) * Box(W, H, T)
    # seating lip (loads live here)
    lip = Pos(0, 0, T + 1.2) * Box(W - 8, H - 8, 2.4)
    lip -= Pos(0, 0, T + 1.2) * Box(W - 14, H - 14, 4)
    p += lip
    # one latch insert pocket, center
    p -= Pos(0, 0, T / 2) * Cylinder(PL["housing_pocket_d"] / 2, T + 4)
    # two magnet pockets
    for sx in (-W / 2 + 12, W / 2 - 12):
        p -= Pos(sx, 0, T + 0.1 - (PL["magnet_t"] + 0.2) / 2) * \
            Cylinder((PL["magnet_d"] + 0.25) / 2, PL["magnet_t"] + 0.2)
    # two I6 male dovetail segments on the OUTER face (z=0 side), running x
    for sx in (-W / 4, W / 4):
        p += Pos(sx, H / 4, 0) * Rot(0, 180, 0) * Rot(0, 0, 90) * dovetail_male()
    # vent gill slots (backlog B9 preview — they cost nothing here)
    for k in range(4):
        p -= Pos(-W / 2 + 20 + k * 10, -H / 4, T / 2) * Rot(0, 0, 20) * \
            Box(3, 16, T + 2)
    return p


def frame_coupon():
    """The body-side edge a panel seats against: groove + strike + magnets."""
    W, H, T = 60, 30, 8
    f = Pos(0, 0, T / 2) * Box(W, H, T)
    # lip groove
    f -= Pos(0, H / 2 - 5, T - 1.4) * Box(W + 2, 2.4 + 2 * FIT, 3)
    # latch strike: ramped slot the rotor pegs cam behind
    f -= Pos(0, -H / 2 + 9, T / 2 + 1) * Cylinder(9.7 / 2 + FIT, T + 2)
    for s in (0, 180):
        f -= Rot(0, 0, s) * Pos(5.5, -H / 2 + 9, T - 2.2) * Box(3.2, 3.2, 4.5)
    # magnet pockets (polarity: NORTH out, per the spec)
    for sx in (-W / 2 + 10, W / 2 - 10):
        f -= Pos(sx, -H / 2 + 9, T - (PL["magnet_t"] + 0.2) / 2 + 0.1) * \
            Cylinder((PL["magnet_d"] + 0.25) / 2, PL["magnet_t"] + 0.2)
    return f


def dovetail_male_coupon():
    """Standalone I6 male segment on a thin base — fit-test against the shoe
    tonight without printing the whole 116 mm shell sector."""
    base = Pos(0, 0, 1.5) * Box(34, 22, 3)
    return base + Pos(0, 0, 3) * Rot(0, 0, 90) * dovetail_male()


def thumb_knob_m3():
    """Knurled M3 captive knob: hex pocket for the screw head, 12 dia grip."""
    knob = Pos(0, 0, 4) * Cylinder(IF["leg_port"]["thumbscrew_head_d"] / 2, 8)
    for k in range(12):
        a = k * 30
        knob -= Rot(0, 0, a) * Pos(IF["leg_port"]["thumbscrew_head_d"] / 2 + 0.4,
                                   0, 4) * Cylinder(1.1, 9)
    knob -= Pos(0, 0, 6.5) * extrude(RegularPolygon(5.6 / 2 / np.cos(np.pi / 6),
                                                    6), 3.1)   # M3 hex head pocket
    knob -= Pos(0, 0, 2) * Cylinder(3.4 / 2, 9)                # shaft clear
    return knob


if __name__ == "__main__":
    parts = dict(shell_sector_demo=shell_sector_demo(),
                 frame_coupon=frame_coupon(),
                 dovetail_shoe=dovetail_female_shoe(),
                 dovetail_male_coupon=dovetail_male_coupon(),
                 thumb_knob_m3=thumb_knob_m3())
    for n, p in parts.items():
        export(p, n)
    # I6 engagement: male segment must slide through the shoe slot
    male = dovetail_male()
    shoe = parts["dovetail_shoe"]
    posed = Pos(0, 0, 0.0) * male                 # both z=0-based, same axis
    inter = posed & shoe
    v = 0.0 if inter is None else inter.volume
    print(f"I6 dovetail male x shoe: {v:.2f} mm^3 ({'SLIDES' if v < 1 else 'BINDS'})")
    # I3 rotor through housing bore
    r, h = latch_insert_rotor(), latch_insert_housing()
    inter = r & h
    v = 0.0 if inter is None else inter.volume
    print(f"I3 rotor x housing: {v:.2f} mm^3 ({'ASSEMBLES' if v < 2 else 'JAMS'})")
    for n, p in parts.items():
        bb = p.bounding_box()
        print(f"  {n}: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f}")
