"""Battery sled (I5) + the I3 latch cartridge — D020, revised with the keel tub (B84).

  battery_sled : the 3S pack sits on this; the nose carries a panel-mount XT60E-M
                 pocket, the tail the lip the bay door presses (I5 retention). Its two
                 runner grooves ride the keel tub's floor rails (part_bay.bay_tub).
  latch_housing, latch_rotor : the I3 quarter-turn cartridge (iface), exported here
                 since D020; part_panel checks it, part_deck / part_shell / part_bay run
                 it on their strikes and pockets.

2026-10-07 (the owner's body-layout picks, option A): the bay is a part now, part_bay
(bay_tub + bay_lid + bay_door). What moved out of this file:
  * sled_rail RETIRED: the two rails are printed into the tub's floor (part_bay).
  * dock_block RETIRED (pick 4): its floating XT60 carrier is printed into the tub's
    nose (part_bay), so B86's unboltable frame is gone with it.
  * belly_door RETIRED: part_bay.bay_door is the real end door (pick 12: one I3 latch
    in a south-wall boss + a hook tab at its north edge).
  * the B72 sled-in-bay check moved to part_bay, posed in the real tub (the rail, the
    foam pad and the side play counted).
  * the strap slots GO (pick 8 (b), closes B93 that way): nothing straps the pack inside
    the robot. The tub boxes it (floor, walls, the 2 mm foam pad under the roof, the
    door and the nose) and it rides loose on the sled whenever the sled is out. The
    sled's geometry is otherwise unchanged.

Frames: sled local, +x = insertion direction (toward the nose), z 0 = the sled's
underside. Pack dims from params (VERIFY on purchase: the bay is parametric).
"""
from build123d import *
from common import params, export
from iface import IF, latch_insert_housing, latch_insert_rotor

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
BS = IF["battery_sled"]

PACK_L, PACK_W, PACK_H = BS["pack_l"], BS["pack_w"], BS["pack_h"]
SLED_T = 3.0
RAIL = 4.0                 # the tub's rail bar section (part_bay draws the rails)
XT60_W, XT60_H, XT60_D = BS["xt60_panel_mm"]  # panel-mount body, params (VERIFY)
# the sled's own numbers the tub builds to (sled local frame)
SLED_L = PACK_L + 14                           # base length (7 overhang each end of the pack)
SLED_W = PACK_W + 2 * 2.4                      # 48.8 (B72)
GROOVE_Y = SLED_W / 2 - RAIL / 2 - 1.6         # runner groove centre, +-20.8
GROOVE_D = 1.6 + FIT                           # groove depth into the underside (1.9)
NOSE_X1 = SLED_L / 2 + 10                      # the nose's front face (86)
TAIL_X0 = -SLED_L / 2 - 6                      # the tail lip's back face (-82)
XT60_Z = 3 + XT60_H / 2                        # the nose pocket's centre height (7.3)


def battery_sled():
    L, W = SLED_L, SLED_W
    base = Pos(0, 0, SLED_T / 2) * Box(L, W, SLED_T)
    # low side walls: they locate the pack on the sled; the tub boxes it on the robot
    for sy in (1, -1):
        base += Pos(0, sy * (W / 2 - 1.2), SLED_T + 4) * Box(L - 30, 2.4, 8)
    # runner grooves that ride the tub's rails
    for sy in (1, -1):
        base -= Pos(0, sy * GROOVE_Y, 0) * Box(L + 2, RAIL + 2 * FIT, 2 * GROOVE_D)
    # nose: XT60 pocket (the connector glues in; wires exit up)
    nose = Pos(L / 2 + 5, 0, (XT60_H + 6) / 2) * Box(10, XT60_W + 8, XT60_H + 6)
    nose -= Pos(L / 2 + 5, 0, XT60_Z) * Box(12, XT60_W + FIT, XT60_H + FIT)
    base += nose
    # tail lip: the bay door's press boss closes over this (retention per I5)
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


if __name__ == "__main__":
    parts = dict(battery_sled=battery_sled(),
                 latch_housing=latch_insert_housing(),
                 latch_rotor=latch_insert_rotor())
    for n, p in parts.items():
        export(p, n)
    bad = []
    sled = parts["battery_sled"]
    bb = sled.bounding_box()
    # the sled's own numbers the tub builds to, measured on the solid (part_bay poses it in
    # the tub and runs the B72 fit, the slide-out and the XT60 mate there)
    nose_ok = abs(bb.max.X - NOSE_X1) < 1e-6 and abs(bb.min.X - TAIL_X0) < 1e-6
    wide_ok = abs(bb.size.Y - SLED_W) < 1e-6
    if not (nose_ok and wide_ok):
        bad.append("sled envelope moved: part_bay's stack reads NOSE_X1 / TAIL_X0 / SLED_W")
    print(f"sled envelope {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} (tail lip x {bb.min.X:.1f}, "
          f"nose face x {bb.max.X:.1f}, width {bb.size.Y:.2f}; no strap slots: pick 8 (b))")
    bed = sorted(PR["bed_mm"][:2])
    fits = sorted((bb.size.X, bb.size.Y))[0] <= bed[0] and max(bb.size.X, bb.size.Y) <= bed[1]
    if not fits:
        bad.append("sled does not fit the bed")
    print(f"sled on the bed {PR['bed_mm'][0]:.0f} x {PR['bed_mm'][1]:.0f}: {'FITS' if fits else 'TOO BIG'}")
    rotor, housing = parts["latch_rotor"], parts["latch_housing"]
    inter = rotor & housing
    v = 0.0 if inter is None else inter.volume
    if v >= 2:
        bad.append("latch rotor jams")
    print(f"latch rotor x housing (open pose): {v:.2f} mm^3 "
          f"({'TURNS' if v < 2 else 'JAMS'})")
    print(f"part_battery checks: {'CLEAN' if not bad else 'FAIL — ' + '; '.join(bad)}")
    raise SystemExit(1 if bad else 0)
