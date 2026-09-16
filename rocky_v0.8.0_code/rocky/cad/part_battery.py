"""Battery sled + XT60 dock (I5) + belly door demo (I3) — D020.

  battery_sled : the 3S pack straps to this; nose carries a panel-mount XT60
                 pocket; tail has the door-retained lip. Slides on two rails.
  sled_rail    : one of two mirrored rails; bolts to the deck M3 grid
                 (20 mm pitch) from below.
  dock_block   : fixed XT60 pocket, FLOATING (+/-0.8 mm) in a frame that
                 bolts to the deck grid; lead-in chamfers self-align the
                 blind mate. Bus-bar cavity breaks out 2x XT30 spare taps.
  belly_door   : demo door blank: seats on lips, 2x I3 latch inserts, 2x
                 magnet pockets. (The real door joins the shell set later;
                 this one is printable now to exercise the standard.)

Frames: sled local, +x = insertion direction (toward the dock), z=0 at rail
plane. Pack dims from params (VERIFY on purchase — bay is parametric).
"""
from build123d import *
from common import params, export
from iface import IF, latch_insert_housing, latch_insert_rotor

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
BS = IF["battery_sled"]
PL = IF["panel_latch"]

PACK_L, PACK_W, PACK_H = BS["pack_l"], BS["pack_w"], BS["pack_h"]
SLED_T = 3.0
RAIL = 4.0                 # rail bar cross-section
XT60_W, XT60_H, XT60_D = 16.4, 8.6, 16.0      # panel-mount body (VERIFY)


def battery_sled():
    L = PACK_L + 14
    W = PACK_W + 2 * 2.4
    base = Pos(0, 0, SLED_T / 2) * Box(L, W, SLED_T)
    # side walls (low, straps do the holding)
    for sy in (1, -1):
        base += Pos(0, sy * (W / 2 - 1.2), SLED_T + 4) * Box(L - 30, 2.4, 8)
    # strap slots (2 straps, matches deck spacing idea)
    for sx in (-PACK_L / 4, PACK_L / 4):
        for sy in (W / 2 - 6, -W / 2 + 6):
            base -= Pos(sx, sy, SLED_T / 2) * Box(6, 5, SLED_T + 2)
    # runner grooves that ride the rails
    for sy in (1, -1):
        base -= Pos(0, sy * (W / 2 - RAIL / 2 - 1.6), 0) * \
            Box(L + 2, RAIL + 2 * FIT, 2 * (1.6 + FIT))
    # nose: XT60 pocket (connector glues/screws in; wires exit up)
    nose = Pos(L / 2 + 5, 0, (XT60_H + 6) / 2) * Box(10, XT60_W + 8, XT60_H + 6)
    nose -= Pos(L / 2 + 5, 0, 3 + XT60_H / 2) * Box(12, XT60_W + FIT, XT60_H + FIT)
    base += nose
    # tail lip: the belly door closes over this (retention per I5)
    base += Pos(-L / 2 - 3, 0, 2.5) * Box(6, 24, 5)
    # finger scallop for extraction. v0.1 (session 8): the v0 cylinder sat
    # at z = SLED_T + 6 with r 9 — its bottom was EXACTLY z 0, so it cut the
    # floor clean through across the full width and the tail lip came off
    # as a second body touching the sled along one tangent line (D036
    # class: 0.000 mm gap, still two solids). Raised so 1.8 mm of floor
    # survives under the trough.
    base -= Pos(-L / 2 + 6, 0, SLED_T - 1.2 + 9) * Rot(90, 0, 0) * \
        Cylinder(9, W + 4)
    return base


def sled_rail():
    """One rail: bar + two bolt tabs at the deck grid pitch (20)."""
    L = PACK_L + 24
    bar = Pos(0, 0, -RAIL / 2) * Box(L, RAIL, RAIL)
    for sx in (-40, -20, 0, 20, 40):
        tab = Pos(sx, 3.5, -RAIL / 2 - 1) * Box(10, 11, 2)
        tab -= Pos(sx, 7, -RAIL / 2 - 1) * Cylinder(PR["screw_m3_clear"] / 2, 4)
        bar += tab
    return bar


def dock_block():
    """Floating XT60 receiver: outer frame (bolts down) + inner carrier with
    +/-0.8 float, joined by 4 thin printed flexure webs. Lead-in chamfers."""
    fl = BS["xt60_float"]
    frame = Pos(0, 0, 10) * Box(26, XT60_W + 16, 20)
    cavity = Pos(0, 0, 10 + 1) * Box(26 + 2, XT60_W + 8 + 2 * fl, 14 + 2 * fl)
    frame -= cavity
    for sx in (-8, 8):
        frame -= Pos(sx, (XT60_W + 16) / 2 - 4, 2) * Cylinder(PR["screw_m3_clear"] / 2, 6)
    carrier = Pos(0, 0, 10 + 1) * Box(22, XT60_W + 6, 13)
    carrier -= Pos(2, 0, 10 + 1) * Box(20, XT60_W + FIT, XT60_H + FIT)
    # lead-in chamfer mouth
    carrier -= Pos(11, 0, 10 + 1) * Rot(0, -14, 0) * Box(6, XT60_W + 5, XT60_H + 5)
    # flexure webs to the frame (they shear before the connector does)
    for sy in (1, -1):
        for sz in (5, 17):
            carrier += Pos(0, sy * (XT60_W / 2 + 3 + 1.0), sz) * Box(14, 2.2, 1.2)
    # XT30 spare-tap wire channel
    frame -= Pos(0, -(XT60_W + 16) / 2 + 2, 16) * Box(8, 6, 10)
    return frame + carrier


def belly_door():
    """Door blank exercising panel standard I3: lip seat + 2 latches + 2 magnets."""
    W, H, T = 120, 70, 3.0
    door = Pos(0, 0, T / 2) * Box(W, H, T)
    # perimeter seating lip (takes all loads)
    lip = Pos(0, 0, T + 1.2) * Box(W - 6, H - 6, 2.4)
    lip -= Pos(0, 0, T + 1.2) * Box(W - 12, H - 12, 4)
    door += lip
    # latch insert pockets (through)
    for sx in (-W / 2 + 14, W / 2 - 14):
        door -= Pos(sx, 0, T / 2) * Cylinder(PL["housing_pocket_d"] / 2, T + 4)
    # magnet pockets from inside
    for sy in (-H / 2 + 10, H / 2 - 10):
        door -= Pos(0, sy, T + 0.1 - (PL["magnet_t"] + 0.2) / 2) * \
            Cylinder((PL["magnet_d"] + 0.25) / 2, PL["magnet_t"] + 0.2)
    # grip dish
    door -= Pos(0, 0, -0.1) * Cylinder(14, 1.0)
    return door


if __name__ == "__main__":
    parts = dict(battery_sled=battery_sled(), sled_rail=sled_rail(),
                 dock_block=dock_block(), belly_door=belly_door(),
                 latch_housing=latch_insert_housing(),
                 latch_rotor=latch_insert_rotor())
    for n, p in parts.items():
        export(p, n)
    # checks: sled rides rails without clash; pack fits the bay envelope
    sled = parts["battery_sled"]
    r1 = Pos(0, (PACK_W / 2 + 2.4 - RAIL / 2 - 1.6), 1.6 + FIT / 2) * parts["sled_rail"]
    inter = sled & r1
    v = 0.0 if inter is None else inter.volume
    print(f"sled x rail (posed): {v:.2f} mm^3 ({'SLIDES' if v < 1 else 'BINDS'})")
    bb = sled.bounding_box()
    print(f"sled envelope {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} "
          f"vs bay {BS['bay_l']:.0f} x {BS['bay_w']:.0f} x {BS['bay_h']:.0f}")
    rotor, housing = parts["latch_rotor"], parts["latch_housing"]
    inter = rotor & housing
    v = 0.0 if inter is None else inter.volume
    print(f"latch rotor x housing (open pose): {v:.2f} mm^3 "
          f"({'TURNS' if v < 2 else 'JAMS'})")
