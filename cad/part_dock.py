"""Charging dock v0 (B14) — walk on, straddle, crouch, mate. MECHANICAL.

The robot walks +X onto the floor plate, the funnel rails capture the
belly-bay sides and center it (±8 mm capture -> ±0.8, the I5 XT60 float's
own tolerance), the shin bumper stops it at depth, and a 20 mm crouch
(the reflex/gesture crouch primitive — already proven) lowers the belly
connector onto the tower plug. Walk-off is the reverse: stand, back up.

ELECTRICAL IS DELIBERATELY OPEN (Batch-2 decision, spec in the docstring
bottom): the tower carries a parametric plug carrier sized for an XT60
panel mount at VERIFY height. Robot-side options, decided when the BEC/
harness exists: (a) charge XT30 pigtail from an I5 spare tap presented in
a belly-door recess — simplest, print a new door insert then; (b) pogo
pads on an I3 panel insert. Either mates the same tower.

Geometry (body z, deck top -4, ground -118 at nominal stance h):
  belly-bay floor ......... z -40  (deck -10 minus bay_h 30)
  ground at stance ........ z -118 -> bay floor is 78 above ground
  crouch 20 mm ............ bay floor 58 above ground = MATE height
  tower plug face ......... 52 above plate (6 mm engagement + float)
Feet at stance radius 185 never touch the 96-wide plate (fillet feet land
outboard); checks below print every margin.
"""
import numpy as np
from build123d import *
from common import params, export

P = params()
FIT = P["print"]["clearance_fit"]
I5 = P["interfaces"]["battery_sled"]

PLATE_L, PLATE_W, PLATE_T = 170.0, 96.0, 4.0
RAIL_H = 26.0
CAPTURE = 8.0                 # funnel half-entry beyond the bay half-width
TOWER_X = 46.0                # plug tower center (robot noses up to it)
PLUG_H = 52.0                 # plug face above plate — VERIFY vs real belly
XT60_W_, XT60_H_ = 16.2, 8.6  # VERIFY panel-mount envelope (Amass XT60E-M)


def dock_base():
    b = Pos(0, 0, PLATE_T / 2) * Box(PLATE_L, PLATE_W, PLATE_T)
    # approach ramp (rear edge, robot walks up 4 mm painlessly)
    b += Pos(-PLATE_L / 2 - 9, 0, PLATE_T / 2) * Rot(0, -14, 0) * \
        Box(20, PLATE_W, PLATE_T)
    # v0.1 (session 8, printability audit D038): the tilted ramp box dipped
    # 2.4 mm BELOW the floor plane — on the bed the whole 189 x 96 plate sat
    # on a knife-edge and printed as a mid-air island. Trim at z = 0.
    b -= Pos(0, 0, -25) * Box(600, 600, 50)
    half_bay = I5["bay_w"] / 2 + 1.5            # 26.5: sled + shell wall-ish
    # funnel rails: entry at +/-(half_bay+CAPTURE) narrowing to +/-half_bay
    for sy in (1, -1):
        entry_y = sy * (half_bay + CAPTURE)
        exit_y = sy * (half_bay + 0.8)
        ang = np.rad2deg(np.arctan2(abs(entry_y) - abs(exit_y), 70.0)) * -sy
        rail = Rot(0, 0, ang) * Pos(0, 0, RAIL_H / 2) * Box(78, 3.0, RAIL_H)
        b += Pos(-PLATE_L / 2 + 52, (entry_y + exit_y) / 2, PLATE_T) * rail
        b += Pos(30, exit_y, PLATE_T + RAIL_H / 2) * Box(52, 3.0, RAIL_H)
    # shin-bumper stop at depth (soft face, robot noses against it)
    b += Pos(PLATE_L / 2 - 6, 0, PLATE_T + 16) * Box(8, 44, 32)
    # zip anchors for the charge lead
    for zx in (60, 76):
        b -= Pos(zx, -34, PLATE_T / 2) * Cylinder(1.7, PLATE_T + 2)
    return b


def dock_tower():
    t = Pos(0, 0, PLUG_H / 2 + 2) * Box(26, 34, PLUG_H - 4)
    # plug pocket: XT60 panel envelope + float slop, opening UP (+Z face)
    t -= Pos(0, 0, PLUG_H - XT60_H_ / 2 + 0.01) * \
        Box(XT60_W_ + 2 * FIT + 1.6, 24, XT60_H_ + 1.0)
    # lead-in chamfer walls (the crouch self-aligns the last 1.5 mm)
    t -= Pos(0, 0, PLUG_H + 1.2) * Rot(18, 0, 0) * Box(30, 30, 6)
    # wire exit down the spine
    t -= Pos(0, 0, PLUG_H / 2) * Box(8, 26, PLUG_H + 4)
    # base tabs onto the plate (M3 x2)
    for sy in (1, -1):
        tab = Pos(0, sy * 21, 2 + 1.5) * Box(20, 8, 3)
        tab -= Pos(0, sy * 21, 2 + 1.5) * Cylinder(1.7, 5)
        t += tab
    return t


if __name__ == "__main__":
    base, tower = dock_base(), dock_tower()
    export(base, "dock_base")
    export(tower, "dock_tower")
    # ---- geometric acceptance ----
    half_bay = I5["bay_w"] / 2 + 1.5
    print(f"  funnel: captures +/-{half_bay + CAPTURE:.1f} mm -> guides to "
          f"+/-{half_bay + 0.8:.1f} (sled half-width {I5['bay_w'] / 2}; "
          f"XT60 float tolerance +/-{I5['xt60_float']})")
    bay_floor_stance = 118 - 10 - I5["bay_h"]   # above ground at stance
    mate = bay_floor_stance - 20
    print(f"  heights: bay floor {bay_floor_stance} mm above ground at "
          f"stance, {mate} at 20 mm crouch; plug face at "
          f"{PLUG_H + PLATE_T} mm -> engagement "
          f"{PLUG_H + PLATE_T - mate:+.0f} mm (VERIFY on the real belly)")
    # stance feet vs plate: nominal footholds at R185, stations 90+72k;
    # walking ON the dock the front feet land at |y| >= 185*sin(18)=57.2
    foot_y = 185 * np.sin(np.deg2rad(18))
    print(f"  stance clearance: nearest foothold |y| {foot_y:.0f} vs plate "
          f"half-width {PLATE_W / 2:.0f} + rail — feet land on GROUND "
          f"({'OK' if foot_y > PLATE_W / 2 + 6 else 'TOO WIDE'})")
    for n, p in (("dock_base", base), ("dock_tower", tower)):
        bb = p.bounding_box()
        print(f"  {n}: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm,"
              f" {p.volume / 1000:.0f} cm^3")
    assert foot_y > PLATE_W / 2 + 6
    assert abs((PLUG_H + PLATE_T) - mate) <= 8, "plug height outside crouch window"
