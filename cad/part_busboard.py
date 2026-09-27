"""Bus star-board bracket — see docs/WIRING_HARNESS.md (star board).

Holds the 40x30 perfboard star hub flat at 12 mm standoff: 4 posts (M3
thread-forming, 34x24 pattern), two zip-tie wings for the r=62 loom ring.
Print flat, no supports, PLA fine (no load).

Deck mounting (v0.2): two M3 screws through the plate's centreline into the
EXISTING deck grid holes (0, 20) and (0, 40) — the plate sits north of the
avionics tray's latch tongue, between the strap slots. v0.1 hung two tabs off
the -y edge for grid holes (20, -20) / (40, -20): the first of those does not
exist (the strap-slot exclusion removes it) and the footprint sat on the tray.
The layout audit below ray-tests both bolt axes against the real deck and
keeps the footprint off the tray, the strap slots, the power entry and every
leg port. Screws first, then the board on its posts.
"""
from build123d import *
from common import params, export

P = params()
PR = P["print"]

PLATE_W, PLATE_L, PLATE_T = 46.0, 36.0, 3.0
POST_DX, POST_DY, POST_H, POST_D = 34.0, 24.0, 8.0, 5.6
DECK_HOLES = [(0.0, 20.0), (0.0, 40.0)]     # existing deck grid holes (body frame)
BRACKET_XY = (0.0, 32.0)                    # plate centre on the deck: y 14..50, wings to 58


def busboard_bracket():
    b = Pos(0, 0, PLATE_T / 2) * Box(PLATE_W, PLATE_L, PLATE_T)
    # lightening window, between the two bolt holes
    b -= Pos(0, -2, PLATE_T / 2) * Box(24, 10, PLATE_T + 2)
    # corner posts, M3 thread-forming bores
    for sx in (-POST_DX / 2, POST_DX / 2):
        for sy in (-POST_DY / 2, POST_DY / 2):
            b += Pos(sx, sy, PLATE_T + POST_H / 2) * Cylinder(POST_D / 2, POST_H)
            b -= Pos(sx, sy, PLATE_T + POST_H / 2 + 0.5) * \
                Cylinder(PR["screw_m3_tap"] / 2, POST_H + 2)
    # deck bolts: M3 clearance on the centreline, over the deck grid holes
    for hx, hy in DECK_HOLES:
        b -= Pos(hx - BRACKET_XY[0], hy - BRACKET_XY[1], PLATE_T / 2) * \
            Cylinder(PR["screw_m3_clear"] / 2, PLATE_T + 2)
    # zip-tie wings for the loom ring (+y edge)
    for sx in (-16, 16):
        wing = Pos(sx, PLATE_L / 2 + 4, PLATE_T / 2) * Box(8, 8, PLATE_T)
        wing -= Pos(sx, PLATE_L / 2 + 4, PLATE_T / 2) * Box(3.2, 4.5, PLATE_T + 2)
        b += wing
    return b


def layout_audit(part):
    """D033-style deck audit: [(check, ok, detail)] for the bracket posed on
    body_deck() at BRACKET_XY."""
    import numpy as np
    from part_deck import (body_deck, grid_holes, pentagon_contains, STATIONS, R_STATION,
                           STRAP_SLOTS, STRAP_SLOT_WH, GROMMET_XY, GROMMET_D, ZIP_ANCHORS,
                           T as DECK_T)
    from part_avionics import tray_footprint, TRAY_XY
    from iface import IF
    v = lambda x: 0.0 if x is None else x.volume
    lp = IF["leg_port"]
    deck = body_deck()
    posed = Pos(*BRACKET_XY, DECK_T) * part          # deck modelled z 0..T, bracket on its top
    tall = lambda cx, cy, w, h: Pos(cx, cy, 0) * Box(w, h, 200)
    out = []
    for hx, hy in DECK_HOLES:
        probe = Pos(hx, hy, DECK_T) * Cylinder(0.5, 2 * DECK_T + 20)   # the bolt axis, through both
        vd, vb = v(probe & deck), v(probe & posed)
        out.append((f"bolt axis ({hx:.0f}, {hy:.0f}) through the deck grid hole and the bracket",
                    (hx, hy) in grid_holes() and vd < 1e-3 and vb < 1e-3,
                    f"deck {vd:.3f} / bracket {vb:.3f} mm^3 on a Ø1 ray"))
    out.append(("bracket sits ON the deck (no overlap)", v(posed & deck) < 1e-3, ""))
    tray = None
    for cx, cy, w, h in tray_footprint():
        box = tall(TRAY_XY[0] + cx, TRAY_XY[1] + cy, w, h)
        tray = box if tray is None else tray + box
    out.append((f"footprint clear of the avionics tray (plate, rails, tongue at {TRAY_XY})",
                v(posed & tray) < 1e-3, ""))
    straps = None
    for sx, sy in STRAP_SLOTS:
        box = tall(sx, sy, STRAP_SLOT_WH[0] + 1.0, STRAP_SLOT_WH[1] + 1.0)
        straps = box if straps is None else straps + box
    out.append(("footprint clear of the strap slots (+0.5 mm)", v(posed & straps) < 1e-3, ""))
    power = Pos(*GROMMET_XY, 0) * Cylinder(GROMMET_D / 2 + 3.0, 200)
    for zx, zy in ZIP_ANCHORS:
        power += Pos(zx, zy, 0) * Cylinder(1.7 + 2.0, 200)
    out.append(("footprint clear of the power grommet + zip anchors", v(posed & power) < 1e-3, ""))
    x0 = lp["hook_slot_x"] - lp["hook_slot_w"] / 2               # the leg port on the deck:
    ports = None                                                 # hook slot .. plate front, +-22
    for ang in STATIONS:
        box = Rot(0, 0, ang) * Pos(R_STATION, 0, 0) * tall((x0 + 20.0) / 2, 0.0, 20.0 - x0, 44.0)
        ports = box if ports is None else ports + box
    out.append(("footprint clear of every leg port (hook slot + coxa plate)", v(posed & ports) < 1e-3, ""))
    bb = posed.bounding_box()
    corners = [(x, y) for x in (bb.min.X, bb.max.X) for y in (bb.min.Y, bb.max.Y)]
    out.append(("footprint corners on the deck (2.5 mm to the pentagon edge)",
                all(pentagon_contains(x, y) for x, y in corners),
                f"x {bb.min.X:.1f}..{bb.max.X:.1f}, y {bb.min.Y:.1f}..{bb.max.Y:.1f}, "
                f"r max {max(np.hypot(x, y) for x, y in corners):.1f}"))
    return out


if __name__ == "__main__":
    part = busboard_bracket()
    export(part, "busboard_bracket")
    bb = part.bounding_box()
    print(f"bracket: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm")
    # sanity: posts land on the 34x24 corner-drill pattern
    assert abs(POST_DX - 34) < 1e-9 and abs(POST_DY - 24) < 1e-9
    print("post pattern matches the WIRING_HARNESS.md star-board drill spec (34 x 24)")
    bad = 0
    for name, ok, detail in layout_audit(part):
        bad += not ok
        print(f"  {name}: {'OK' if ok else 'FAIL'}" + (f" ({detail})" if detail else ""))
    print(f"part_busboard layout audit: {'CLEAN' if not bad else f'{bad} FAILURE(S)'}")
    raise SystemExit(1 if bad else 0)
