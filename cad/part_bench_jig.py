"""Phase-1 single-leg bench jig — the first customer of leg port I1 (D020).

Two flat prints, no calipers needed (all interfaces are our own):

  jig_base    : 220 x 140 x 6 floor plate. Clamp tabs both ends (C-clamp to
                the bench), 4x M5 through-holes as an alternative, engraved
                foot-target arcs where the gait's stance circle lands, and a
                bolt pattern for the column.
  jig_column  : vertical C-column + top deck-proxy plate carrying the FULL
                leg port: dowel posts, heat-set pockets, hook catch bar,
                cable cutout — printed lying on its back, bolts to the base.

Deck-proxy top sits 130 mm above the floor -> commanding body_height 118
puts the foot 12 mm above the floor; body_height 130 touches down. The
protractor arc around the yaw axis reads coxa sweep by eye.

Because the jig consumes the SAME iface.leg_port_deck_features() as the real
deck, a leg that bolts to the jig bolts to the robot. That's the point.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import leg_port_deck_features

P = params()
PR = P["print"]

DECK_H = 130.0            # deck-proxy TOP above the base floor
PLATE_T = 6.0
COL_W = 84.0              # column width (y)
COL_T = 6.0
BASE_L, BASE_W = 220.0, 140.0
# frame: yaw axis at (0,0); leg reaches +x over the base; column behind (-x)
BASE_X0, BASE_X1 = -78.0, 142.0
COL_X = -58.0             # column front face x (clear of the I1 hook slot @ -48)


def jig_base():
    b = Pos((BASE_X0 + BASE_X1) / 2, 0, PLATE_T / 2) * Box(BASE_L, BASE_W, PLATE_T)
    # clamp tabs: thinned zones at both x ends for C-clamps
    for cx in (BASE_X0 + 14, BASE_X1 - 14):
        b -= Pos(cx, 0, PLATE_T - 1.0) * Box(28, 60, 2.1)
    # M5 bench bolt holes
    for hx, hy in ((BASE_X0 + 10, 55), (BASE_X0 + 10, -55),
                   (BASE_X1 - 10, 55), (BASE_X1 - 10, -55)):
        b -= Pos(hx, hy, PLATE_T / 2) * Cylinder(5.4 / 2, PLATE_T + 2)
    # column bolt pattern: 6x M3 heat-set pockets (column flange bolts down)
    for hx in (COL_X - 10, COL_X - 26):
        for hy in (-30, 0, 30):
            b -= Pos(hx, hy, PLATE_T - PR["heatset_m3_h"] / 2 + 0.01) * \
                Cylinder(PR["heatset_m3_d"] / 2, PR["heatset_m3_h"])
    # engraved foot-target arcs: stance-radius circle around the yaw axis
    # (leg-frame stance lands near r=75 from the yaw axis) + swing extremes
    for r, depth in ((60.0, 0.8), (75.0, 1.2), (90.0, 0.8)):
        ring = Pos(0, 0, PLATE_T - depth / 2 + 0.01) * \
            (Cylinder(r + 0.6, depth) - Cylinder(r - 0.6, depth + 1))
        wedge = Pos(60, 0, PLATE_T - depth / 2) * Box(120, 160, depth + 2)
        b -= ring & wedge                    # only the forward arc
    # locating groove for the knee calibration V-block (part_smallwins):
    # x=140 = leg-frame knee/tube station
    b -= Pos(140, 0, PLATE_T - 0.6) * Box(3.8, 30, 1.3)
    return b


def jig_column():
    z0 = PLATE_T                              # column sits on the base top
    top_t = 6.0
    deck_top = DECK_H                         # top surface of deck-proxy
    # spine (vertical plate) + two side gussets + bottom flange
    spine = Pos(COL_X - COL_T / 2, 0, (z0 + deck_top) / 2) * \
        Box(COL_T, COL_W, deck_top - z0)
    flange = Pos(COL_X - 20, 0, z0 + 2) * Box(40, COL_W, 4)
    for hx in (COL_X - 10, COL_X - 26):
        for hy in (-30, 0, 30):
            flange -= Pos(hx, hy, z0 + 2) * Cylinder(3.4 / 2, 8)
    guss = None
    # v0.1 (session 8, D038 printability audit): printed on its back the
    # spine face and the flange edge each bridged 84 mm between the two
    # edge gussets. Two more gussets at ±COL_W/6 cut every bridge to ≤28 mm
    # and give the print four bed-contact fins instead of two.
    for gy in (-(COL_W / 2 - 2), -(COL_W / 6), (COL_W / 6), (COL_W / 2 - 2)):
        sy = 1 if gy > 0 else -1
        with BuildPart() as g:
            with BuildSketch(Plane.XZ.offset(-gy)):
                with BuildLine():
                    Polyline((COL_X - COL_T, z0), (COL_X - COL_T, deck_top - 4),
                             (COL_X - 46, z0), (COL_X - COL_T, z0))
                make_face()
            extrude(amount=sy * 4)
        guss = g.part if guss is None else guss + g.part
    # deck-proxy plate: spans the coxa plate footprint (x -46..+20 + margins)
    # v0.1 (session 8): the v0 plate was placed off the LEG-PORT footprint
    # (x -52..24) and stopped 6 mm short of the spine face — the deck-proxy
    # was a separate body sitting in mid-air (D036 class). It now runs from
    # the spine's back face to x +24, so it's welded across the whole
    # column thickness.
    px0, px1 = COL_X - COL_T, 24.0
    plate = Pos((px0 + px1) / 2, 0, deck_top - top_t / 2) * \
        Box(px1 - px0, COL_W, top_t)
    # protractor: engraved ticks every 10 deg over +/-50 around the yaw axis
    for ang in range(-50, 51, 10):
        a = np.deg2rad(ang)
        r0, r1 = 26.0, (32.0 if ang % 30 == 0 else 29.0)
        tick = Pos((r0 + r1) / 2, 0, deck_top - 0.5) * Box(r1 - r0, 1.0, 1.2)
        plate -= Rot(0, 0, ang) * tick
    col = spine + flange + guss + plate
    # the leg port itself — same code path as the real deck
    col = leg_port_deck_features(col, top_z=deck_top)
    return col


if __name__ == "__main__":
    b = jig_base()
    c = jig_column()
    export(b, "jig_base")
    export(c, "jig_column")
    for name, part, maxx, maxy in (("base", b, 250, 210), ("column", c, 250, 210)):
        bb = part.bounding_box()
        dims = sorted([bb.size.X, bb.size.Y, bb.size.Z], reverse=True)
        fits = dims[0] <= 250 and dims[1] <= 210
        print(f"jig_{name}: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} "
              f"-> print footprint {dims[0]:.0f} x {dims[1]:.0f} "
              f"({'FITS Prusa bed' if fits else 'TOO BIG'})")
    inter = b & c
    v = 0.0 if inter is None else inter.volume
    print(f"base x column interference: {v:.1f} mm^3 "
          f"({'OK' if v < 1 else 'CLASH'})")
    # port sanity: dowel posts + catch present at the right stations
    from iface import IF
    lp = IF["leg_port"]
    print(f"leg port on jig: dowels at {lp['dowel_xy']}, "
          f"thumbscrews at {lp['thumbscrew_xy']} — same params the deck uses")
