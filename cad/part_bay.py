"""The keel tub (I5, option A, B84): bay_tub + bay_lid + bay_door — 2026-10-07.

The battery bay as parts, built to params interfaces.battery_sled and iface.bay_tub_extent()
(the owner's picks 1-12, docs/BODY_LAYOUT_PROPOSAL.md option A). Body frame throughout (z =
leg z, deck z -10..-4, +x east, leg 0 north); the tub hangs east-west under the deck at
y -20, nose (XT60) +x, door -x.

  bay_tub  : a U printed open-top: the floor (bay_floor) with the sled's two rails INTEGRAL
             (sled_rail retires), two side walls and the nose wall (bay_wall), the nose's two
             vertical edges chamfered (pick 10). Printed into the nose (pick 4, dock_block
             retires): the dock's floating XT60 carrier on four flexure webs inside a stop
             window, and above it the loop key's panel XT60 pocket facing +x (pick 3). At the
             door end the south-wall latch boss carrying the I3 strike (pick 12) and the
             north wall's keeper for the door's hook tab. Two pilasters on the south wall
             take the lid screws; the north wall's top returns over the lid's north edge.
  bay_lid  : the roof (bay_roof), with the six Ø8 hanger bosses up to the deck bottom at
             part_deck.DECK_HOLES' tub hangers (an M3 x 16 from the deck top, through a
             tray-rail tab where there is one, into each boss's heat-set insert) and two
             insert bosses over the pilasters. It ends at the door plane: the door covers it.
  bay_door : the -x end door (54.8 x 37.4 x door_t, replaces part_battery.belly_door): a
             spigot inside the opening (locates it in y and z), the press boss on the sled's
             tail lip (I5 retention), the I3 cartridge in a 6-thick south ear over the strike,
             and the hook tab at its north edge (a snap: its barb hooks behind the keeper).

The lid-to-tub joint (the proposal left it open): north, the north wall's top returns 1.2 over
the lid's north edge (the tub goes on shifted 1.6 north, rises to the lid, slides 1.6 south
under it); south, one M3 x 10 from below in each pilaster (a Ø6.4 bore up from the belly to
its seat) through the lid's tab into a heat-set insert in its boss. Nothing outside the north
wall takes a screw: between the hub shelf (y >= 12.2), the leg 1/4 hook envelopes and their
drops there is no room for one (measured below, 'why no north screw').

B126: three under-deck turn points sit inside the tub's plan, carapace latches 162 and 306 and
the tray's latch strike at (0, -43). Each gets a Ø6 driver column through the floor and the
roof (a 1.0 notch in the south wall's inner face at (0, -43), the south rail broken there). The
interior between is the SLED's: those three latches are turned with the sled OUT.

Every check prints, and the module exits non-zero on a failure (the CAD tree's CI).
"""
import numpy as np
from build123d import *
from common import params, export
from iface import (IF, bay_tub_extent, body_keepouts, latch_strike, latch_pocket, station_tf,
                   STATIONS, DECK_BOT_Z, SHELL_LATCH_AZ, SHELL_LATCH_R, LATCH_REACH)
import part_battery as PB
from part_deck import DECK_HOLES, TUB_BOSS_D     # the deck's hole audit holds the bosses on this plan

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
BS = IF["battery_sled"]
ST = IF["stand"]
HS = IF["hub_shelf"]
PL = IF["panel_latch"]
R2 = np.sqrt(2.0)

# ---- the tub's box, from params alone (iface.bay_tub_extent: the sim's belly takes the same sums)
EXT = bay_tub_extent()
XD = BS["bay_door_x"]                   # -97.5: the door's OUTER face (pins the tub in x)
XI0, XI1 = EXT["x_in"]                  # -94.5 .. 97.5: the interior, door face to nose wall
XN = EXT["x"][1]                        # 99.9: the nose's outer face
YO0, YO1 = EXT["y"]                     # -47.4 .. 7.4
YI0, YI1 = EXT["y_in"]                  # -45.0 .. 5.0
ZB, ZT = EXT["z"]                       # -55.4 (belly) .. -18.0 (the roof's top, pick 2)
ZF, ZR = EXT["z_in"]                    # -53.0 (floor top) .. -20.0 (roof underside)
YC = BS["bay_centre"][1]                # -20: the tub's (and the pack's) axis in y
WALL, FLOOR, ROOF, DOOR_T = BS["bay_wall"], BS["bay_floor"], BS["bay_roof"], BS["door_t"]
CHAMF = BS["nose_chamfer"]              # pick 10: the outer plan chamfer's leg
CHAMF_IN = CHAMF - WALL * (2 - R2)      # the interior's, so the wall stays WALL thick (8.59)

# ---- the sled in the tub (part_battery's geometry; sled local z 0 = its underside)
SLED_X = BS["bay_centre"][0]            # -12: the pack's centre docked = the sled's origin
SLED_Z0 = ZF + PB.RAIL - PB.GROOVE_D    # -50.9: at rest its groove ceilings sit on the rail tops
RAIL_X0, RAIL_X1 = XI0 + 2.5, 64.0      # the rails: clear of the door's spigot, under the sled
RAIL_LEADIN = 1.5                       # 45-deg chamfer on each rail's -x top edge (the sled enters)

# ---- the XT60 mate (pick 4). VERIFY every one with the real XT60E-M / XT60 female pair:
XT60_W, XT60_H, XT60_D = PB.XT60_W, PB.XT60_H, PB.XT60_D
XT60_ENGAGE = 7.0       # how far the mated housings telescope (the female's nose in the male's
                        # shroud). VERIFY with calipers: the door's press boss is the feature to file
NOSE_GAP = 0.5          # mated: the sled's nose face to the carrier's mouth (the housings bottom first)
MOUTH_RECESS = 0.5      # the female's front face inside the carrier's mouth
SLED_XT60_PROUD = NOSE_GAP + MOUTH_RECESS + XT60_ENGAGE   # 8.0: glue the sled's XT60E-M this far proud
LEADIN = 1.2            # the mouth's 45-deg lead-in (the float's 0.8 + the rails' 0.3 of play)
LEAD_ROOM = 6.0         # ESTIMATE: behind the female, a 14 AWG pair turns up (r ~1.5 OD)
CARRIER_L, CARRIER_W, CARRIER_H = 22.0, XT60_W + 6, 13.0   # dock_block's carrier, ported
FLOAT = BS["xt60_float"]                # +-0.8
STOP = FLOAT + 0.05                     # the stop window's clearance: free at +-0.8, stops at 0.85
WEB = (14.0, 1.2, 1.2)                  # flexure webs, x long, y, z. NOT dock_block's 2.2: bent
                                        # fixed-guided 0.85 to the stop a 2.2 face would carry
                                        # 3 E t d / L^2 = 42.9 MPa (> the 35 allowable); 1.2: 23.4
WEB_GAP = 1.0                           # web to the carrier's side (0.2 left at full float)
FRAME_T = 3.0                           # the stop window's thickness in x

# ---- the loop key (pick 3), a chassis XT60E-M in the nose wall facing +x
KEY_TOP_GAP = 1.2       # its pocket's top under the roof seat (the walls' top, z ZR)

# ---- the door end (pick 12)
STRIKE_T = LATCH_REACH + 0.2            # 6.0: the I3 strike's frame (iface.latch_strike asserts it)
PAD = 14.6                              # the strike's pad (r 6.1 cutter + 1.2 of wall each side)
LB_X0 = XI0 + DOOR_T                    # -91.5: the strike's seating plane (the door ear sits on it)
POCKET_R = PL["housing_pocket_d"] / 2   # 7.15: the door's cartridge pocket (iface.latch_pocket) ...
POCKET_FLAT = 6.0 + (PL["housing_pocket_d"] - PL["housing_d"]) / 2   # ... and its index flat (6.15)
EAR_WALL = 1.2                          # the door ear's wall round that pocket
# the latch axis: outside the south wall, low. Its pocket's round side keeps EAR_WALL to the ear's
# north face (FIT off the wall), its flat faces the belly with EAR_WALL under it: the ear then
# never hangs below the tub, and the driver (Ø5 along +x from the slot) passes 0.55 under leg 2's
# drop keep-out. A pocket centred in the 14.6 pad left 0.075 of skin: Ø14.45 in a 14.3 ear
LB_Y = YO0 - FIT - EAR_WALL - POCKET_R
LB_Z = ZB + EAR_WALL + POCKET_FLAT
PRESS = 0.5                             # the door's boss into the sled's tail lip (I5; 0.3..0.8)
SPIGOT = (1.2, 1.5)                     # the door's spigot inside the opening: wall, depth (FIT round)
TAB_T, TAB_Z = 1.2, (-42.0, -30.0)      # the door's hook tab (a snap): thickness, z span
KEEP_X = (-83.0, -80.5)                 # the north wall's keeper, x (its -x face a 45-deg ramp)
KEEP_P = 1.2                            # the keeper's height off the north wall = the barb's bite
BARB_L = 1.8

# ---- the lid-to-tub joint
RETURN_IN, RETURN_T = 1.2, 0.9          # the north wall's return over the lid: reach, thickness
SLIDE = 1.6                             # the tub goes on shifted this far north, then slides south
LID_N = YO1 - WALL / 2 - FIT            # 5.9: the lid's north edge, on the north wall's top
PIL_D, PIL_BORE = 9.6, 6.4              # the south pilasters: outside the wall, a bore up to the seat
PIL_SEAT = ZR - 6.0                     # -26.0: the screw head's seat in the pilaster
PIL_Z0 = -41.0                          # the round part's bottom; a 45-deg wedge under it into the
                                        # wall, so the door latch's driver (z -48.1 +- 2.5, along +x
                                        # from its slot) passes under the west one
PIL_XY = [(-70.0, YO0 - 3.2), (70.0, YO0 - 3.2)]
LID_BOSS_D = 8.0
LID_SCREW_L = 10.0                      # M3 x 10 from below: 6 through the pilaster's top, 4 in the insert
LANE_SKIP = 1.0                         # the north return stops this far from the leg 2/3 lanes

# ---- B126: the three under-deck turn points inside the tub's plan, worked with the sled OUT
COL_D = 6.0                             # a Ø5 flat screwdriver + 0.5 a side


def _latch_xy(i):
    a = np.deg2rad(STATIONS[i] + SHELL_LATCH_AZ)
    return (SHELL_LATCH_R * np.cos(a), SHELL_LATCH_R * np.sin(a))


TURN_POINTS = [(f"carapace latch {STATIONS[1]:.0f}", _latch_xy(1)),
               (f"carapace latch {STATIONS[3]:.0f}", _latch_xy(3)),
               ("tray strike", next(e[1] for e in DECK_HOLES if e[2] == "strike"))]
HANGERS = [e for e in DECK_HOLES if e[0].startswith(("tray_tab_tub_", "tub_hanger_"))]


def _v(s):
    return 0.0 if s is None else s.volume


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _prism(pts, z0, z1):
    return Pos(0, 0, z0) * extrude(make_face(Polyline(*pts, close=True)), z1 - z0)


def plan(x0, x1, y0, y1, c):
    """The tub's plan outline: square at x0 (the door end), its two corners at x1 chamfered
    with legs c (45 deg)."""
    return [(x0, y0), (x1 - c, y0), (x1, y0 + c), (x1, y1 - c), (x1 - c, y1), (x0, y1)]


# ====================================================================== the mate, measured
def sled_tf(dx=0.0):
    """The sled in the tub: docked (dx 0: the pack's centre on bay_centre, at rest on the rails)
    or slid out dx along x."""
    return Pos(SLED_X + dx, YC, SLED_Z0)


def pack_solid():
    """The pack on the sled (sled local), params pack_l x pack_w x pack_h."""
    return Pos(0, 0, PB.SLED_T + PB.PACK_H / 2) * Box(PB.PACK_L, PB.PACK_W, PB.PACK_H)


def male_xt60():
    """The sled's XT60E-M envelope (sled local), glued SLED_XT60_PROUD past the nose face."""
    x1 = PB.NOSE_X1 + SLED_XT60_PROUD
    return Pos(x1 - XT60_D / 2, 0, PB.XT60_Z) * Box(XT60_D, XT60_W, XT60_H)


def _sled_pocket_centre(sled):
    """The sled nose pocket's centre (y, z) in the sled frame, MEASURED: the void the pocket
    leaves in a probe box over the nose."""
    probe = _box(PB.NOSE_X1 - 9.0, PB.NOSE_X1 - 1.0, -12.0, 12.0, 0.5, 14.0)
    void = probe - sled
    bb = void.bounding_box()
    return (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2, (bb.size.Y, bb.size.Z)


_SLED = PB.battery_sled()
_PY, _PZ, _PSIZE = _sled_pocket_centre(_SLED)
ZC = SLED_Z0 + _PZ                      # the carrier's axis = the sled's XT60 axis at rest
YK = YC + _PY
XC0 = SLED_X + PB.NOSE_X1 + NOSE_GAP    # 74.5: the carrier's mouth
XF0 = XC0 + MOUTH_RECESS                # 75.0: the female's front face ...
XF1 = XF0 + XT60_D                      # 91.0: ... and back face, on the carrier's back rims
XI1_NEED = max(XC0 + CARRIER_L + 0.5, XF1 + LEAD_ROOM + 0.3)   # the nose wall's inner face, at least
KEY_Z = ZR - KEY_TOP_GAP - (XT60_H + FIT) / 2                   # the key pocket's centre (-27.15)


def female_xt60():
    return _box(XF0, XF1, YK - XT60_W / 2, YK + XT60_W / 2, ZC - XT60_H / 2, ZC + XT60_H / 2)


U_W = XT60_W - 3.0                     # the carrier's U behind the pocket (its rims stop the female)


def lead_room():
    """ESTIMATE keep-out: the dock female's 14 AWG pair leaving its solder cups and turning up,
    LEAD_ROOM behind its back face, inside the U's width (the cups sit inside the housing's
    section) and above the U's floor."""
    return _box(XF1, XF1 + LEAD_ROOM, YK - U_W / 2 + 0.1, YK + U_W / 2 - 0.1,
                ZC - (XT60_H + FIT) / 2 + 1.6, ZC + XT60_H / 2)


def key_xt60():
    """The loop key's chassis XT60E-M envelope (VERIFY): its mating face flush with the nose's
    outer face, the body XT60_D deep into the tub (its flange on the outside, unmodelled)."""
    return _box(XN - XT60_D, XN, YC - XT60_W / 2, YC + XT60_W / 2, KEY_Z - XT60_H / 2, KEY_Z + XT60_H / 2)


# ====================================================================== the carrier (pick 4)
def carrier_body():
    """The floating carrier alone (no webs): dock_block's 22 x (XT60_W + 6) x 13 block, the
    female's pocket from the mouth (-x) with a 45-deg lead-in, and behind the pocket a U open
    upward (the female's leads turn up through it; its rims are the female's back stop)."""
    x1 = XC0 + CARRIER_L
    c = _box(XC0, x1, YK - CARRIER_W / 2, YK + CARRIER_W / 2, ZC - CARRIER_H / 2, ZC + CARRIER_H / 2)
    pw, ph = XT60_W + FIT, XT60_H + FIT
    c -= _box(XC0 - 0.01, XF1, YK - pw / 2, YK + pw / 2, ZC - ph / 2, ZC + ph / 2)
    mouth = Plane.YZ.offset(XC0 - 0.01)
    c -= loft([mouth * Pos(YK, ZC) * Rectangle(pw + 2 * LEADIN, ph + 2 * LEADIN),
               Plane.YZ.offset(XC0 + LEADIN) * Pos(YK, ZC) * Rectangle(pw, ph)])
    c -= _box(XF1 - 0.01, x1 + 0.01, YK - U_W / 2, YK + U_W / 2, ZC - ph / 2 + 1.5, ZC + CARRIER_H)
    return c


def carrier_frame():
    """The stop window round the carrier's front (fused to the floor: it is the frame the nose
    became) + the four flexure webs + their stubs on the carrier. The webs run +x from the
    window's back face to stubs at the carrier's rear sides, so the mating push puts them in
    tension; bent fixed-guided they give the carrier +-FLOAT in y and z, and the window stops it
    at +-STOP before the webs see more."""
    fx0, fx1 = XC0 + 2.0, XC0 + 2.0 + FRAME_T
    hw, hh = CARRIER_W / 2 + STOP, CARRIER_H / 2 + STOP
    frame = _box(fx0, fx1, YK - hw - WALL, YK + hw + WALL, ZF - 0.01, ZC + hh + WALL)
    frame -= _box(fx0 - 1, fx1 + 1, YK - hw, YK + hw, ZC - hh, ZC + hh)
    wl, wy, wz = WEB
    yw = CARRIER_W / 2 + WEB_GAP + wy / 2
    webs = []
    for sy in (1, -1):
        for zz in (ZC + CARRIER_H / 2 - 2.0, ZC - CARRIER_H / 2 + 2.0):
            x0 = fx1 - 0.1
            webs.append(_box(x0, x0 + wl, YK + sy * yw - wy / 2, YK + sy * yw + wy / 2, zz - wz / 2, zz + wz / 2))
            y_in, y_out = YK + sy * (CARRIER_W / 2 - 0.05), YK + sy * (CARRIER_W / 2 + WEB_GAP + 0.05)
            webs.append(_box(x0 + wl - 2.0, x0 + wl, min(y_in, y_out), max(y_in, y_out), zz - wz / 2, zz + wz / 2))
    return frame, webs


# ====================================================================== bay_tub
def _strike_tf():
    """The I3 latch frame at the door end (z 0 = the seating plane at LB_X0, +z toward the door,
    i.e. -x; the strike's +x along body +z)."""
    # x_dir -y puts the frame's -y (where latch_housing_tf turns the index flat) on body -z
    return Location(Plane(origin=(LB_X0, LB_Y, LB_Z), x_dir=(0, -1, 0), z_dir=(-1, 0, 0)))


def _turn_columns(z0=-70.0, z1=-5.0, d=COL_D):
    return [Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, z1 - z0) for _, (x, y) in TURN_POINTS]


def _lane_boxes():
    """Params hub_shelf.lanes (correction 12): A, B, C for +x and mirrored, z 7 tall."""
    ln = HS["lanes"]
    out = []
    for (xs, ys) in ln["route"]:
        for sx in (1, -1):
            xa, xb = sorted((sx * xs[0], sx * xs[1]))
            out.append(_box(xa, xb, ys[0], ys[1], ln["z"][0], ln["z"][1]))
    return out


def _return_spans():
    """The x spans of the north wall's return: the whole wall, minus the lanes' x +- LANE_SKIP
    (lane A crosses the north wall in the 8 mm layer), short of the door end and the nose chamfer."""
    cut = sorted({(min(sx * xs[0], sx * xs[1]) - LANE_SKIP, max(sx * xs[0], sx * xs[1]) + LANE_SKIP)
                  for xs, ys in HS["lanes"]["route"] if ys[1] >= YO1 - 3 for sx in (1, -1)})
    spans, x = [], XI0 + 2.0
    for a, b in cut:
        if a > x:
            spans.append((x, a))
        x = max(x, b)
    spans.append((x, XN - CHAMF - 1.0))
    return spans


def bay_tub():
    tub = _prism(plan(XI0, XN, YO0, YO1, CHAMF), ZB, ZR)
    tub -= _prism(plan(XI0 - 1.0, XI1, YI0, YI1, CHAMF_IN), ZF, ZR + 1.0)
    # the north wall's top: its outer half rises past the lid and returns over the lid's edge
    lip_y0 = YO1 - WALL / 2
    for x0, x1 in _return_spans():
        tub += _box(x0, x1, lip_y0, YO1, ZR - 0.01, ZT + RETURN_T + 0.1)
        tub += _box(x0, x1, LID_N - RETURN_IN, YO1, ZT + 0.1, ZT + 0.1 + RETURN_T)
    # the rails, integral: part_battery's 4 x 4 bar where its runner grooves ride (+-20.8)
    for sy in (1, -1):
        y = YC + sy * PB.GROOVE_Y
        r = _box(RAIL_X0, RAIL_X1, y - PB.RAIL / 2, y + PB.RAIL / 2, ZF - 0.01, ZF + PB.RAIL)
        r -= Pos(RAIL_X0, y, ZF + PB.RAIL) * Rot(0, 45, 0) * Box(RAIL_LEADIN * R2, PB.RAIL + 2, RAIL_LEADIN * R2)
        for _, (cx, cy) in TURN_POINTS:            # broken where a B126 column crosses it (+0.5)
            if abs(cy - y) < COL_D / 2 + PB.RAIL / 2:
                r -= _box(cx - COL_D / 2 - 0.5, cx + COL_D / 2 + 0.5, y - 5, y + 5, ZF - 1, ZF + PB.RAIL + 1)
        tub += r
    # pick 4: the carrier's stop window, webs and stubs, then the carrier itself
    frame, webs = carrier_frame()
    tub += frame
    for w in webs:
        tub += w
    tub += carrier_body()
    # pick 3: the loop key's pocket through the nose wall, above the carrier
    pw, ph = XT60_W + FIT, XT60_H + FIT
    tub -= _box(XI1 - 0.5, XN + 0.5, YC - pw / 2, YC + pw / 2, KEY_Z - ph / 2, KEY_Z + ph / 2)
    # pick 12: the latch boss outside the south wall at the door end, the I3 strike in it
    tub += _box(LB_X0, LB_X0 + STRIKE_T, LB_Y - PAD / 2, YO0 + 0.5, ZB, LB_Z + PAD / 2)
    tub -= _strike_tf() * latch_strike(STRIKE_T)
    # ... and the keeper for the door's hook tab on the north wall (a 45-deg ramp on its -x face)
    k0, k1 = KEEP_X
    tub += _prism([(k0, YO1 - 0.5), (k0, YO1), (k0 + KEEP_P, YO1 + KEEP_P), (k1, YO1 + KEEP_P), (k1, YO1 - 0.5)],
                  TAB_Z[0] - 1.0, TAB_Z[1] + 1.0)
    # the lid screws' pilasters on the south wall: a bore up from the belly to the head's seat
    for x, y in PIL_XY:
        tub += Pos(x, y, (PIL_Z0 + ZR) / 2) * Cylinder(PIL_D / 2, ZR - PIL_Z0)
        yo, yw, xw = y - PIL_D / 2, YO0 + 0.5, x - PIL_D / 2
        wedge = make_face(Polyline((xw, yo, PIL_Z0 + 0.01), (xw, yw, PIL_Z0 + 0.01),
                                   (xw, yw, PIL_Z0 - (yw - yo)), close=True))
        tub += extrude(wedge, amount=PIL_D, dir=(1, 0, 0))
        tub -= Pos(x, y, (ZB - 1 + PIL_SEAT) / 2) * Cylinder(PIL_BORE / 2, PIL_SEAT - ZB + 1)
        tub -= Pos(x, y, (PIL_SEAT + ZR) / 2) * Cylinder(PR["screw_m3_clear"] / 2, ZR - PIL_SEAT + 1)
    # B126 driver columns through the floor (and a notch in the south wall at (0, -43)); the
    # south rail is broken where the tray strike's column crosses it
    for c in _turn_columns():
        tub -= c
    # the stand's x stop (params stand): a blind hole in the belly for the cradle's pin
    sx, sy = ST["xstop_xy"]
    d = ST["xstop_pin_d"] + FIT
    tub -= Pos(sx, sy, ZB + (ST["xstop_hole_depth"] - 0.02) / 2 - 0.01) * Cylinder(d / 2, ST["xstop_hole_depth"] + 0.02)
    return tub


# ====================================================================== bay_lid
def bay_lid():
    lid = _prism(plan(XI0, XN, YO0, LID_N, CHAMF), ZR, ZT)
    # the six hanger bosses up to the deck bottom: an insert from the top, the screw's tail through
    hd, hh = PR["heatset_m3_d"], PR["heatset_m3_h"]
    for _, (x, y), *_ in HANGERS:
        lid += Pos(x, y, (ZT - 0.01 + DECK_BOT_Z) / 2) * Cylinder(TUB_BOSS_D / 2, DECK_BOT_Z - ZT + 0.01)
        lid -= Pos(x, y, DECK_BOT_Z - hh / 2 + 0.01) * Cylinder(hd / 2, hh)
        lid -= Pos(x, y, (ZR - 1 + DECK_BOT_Z - hh + 0.5) / 2) * Cylinder(PR["screw_m3_clear"] / 2, DECK_BOT_Z - hh + 0.5 - ZR + 1)
    # over each pilaster a tab of the roof and an insert boss on it (the insert from the tab's
    # underside, where the screw comes up)
    for x, y in PIL_XY:
        lid += Pos(x, y, (ZR + ZT) / 2) * Cylinder(PIL_D / 2, ZT - ZR)
        lid += _box(x - PIL_D / 2, x + PIL_D / 2, y, YO0 + 1.0, ZR, ZT)
        lid += Pos(x, y, (ZT - 0.01 + ZR + hh + 1.0) / 2) * Cylinder(LID_BOSS_D / 2, ZR + hh + 1.0 - ZT + 0.01)
        lid -= Pos(x, y, ZR + hh / 2 - 0.01) * Cylinder(hd / 2, hh)
    for c in _turn_columns():
        lid -= c
    return lid



# ====================================================================== bay_door
def bay_door():
    """In the body frame, closed. Printed lying on its outer face (DOOR_PRINT)."""
    d = _box(XD, XI0, YO0, YO1, ZB, ZT)
    # the south ear: 6 thick over the strike (its +x face is the latch's seating plane)
    ey0, ez1 = LB_Y - POCKET_R - EAR_WALL, LB_Z + POCKET_R + EAR_WALL
    d += _box(XD, XI0, ey0, YO0 + 0.01, ZB, ez1)
    d += _box(XI0 - 0.01, LB_X0, ey0, YO0 - FIT, ZB, ez1)
    d -= _strike_tf() * latch_pocket(-1.0, PL["housing_t"] + 1.0)
    # the spigot: a ring inside the opening, FIT off its edges, open nowhere
    sw, sd = SPIGOT
    y0, y1, z0, z1 = YI0 + FIT, YI1 - FIT, ZF + FIT, ZR - FIT
    ring = _box(XI0 - 0.01, XI0 + sd, y0, y1, z0, z1) - _box(XI0 - 1, XI0 + sd + 1, y0 + sw, y1 - sw, z0 + sw, z1 - sw)
    d += ring
    # the press boss on the sled's tail lip (I5: the door, not the connector, retains the sled)
    lip_z = SLED_Z0 + 2.5
    d += _box(XI0 - 0.01, (SLED_X + PB.TAIL_X0) + PRESS, YC - 8.0, YC + 8.0, lip_z - 2.0, lip_z + 2.0)
    # the hook tab at the north edge: a cantilever outside the keeper, its barb behind it
    ty0 = YO1 + KEEP_P + FIT
    bx0 = KEEP_X[1] + FIT
    d += _box(XD, XI0, YO1 - 0.01, ty0 + TAB_T, *TAB_Z)
    d += _box(XI0 - 0.01, bx0 + BARB_L, ty0, ty0 + TAB_T, *TAB_Z)
    d += _box(bx0, bx0 + BARB_L, YO1 + FIT, ty0 + 0.01, *TAB_Z)
    return d


DOOR_PRINT = Pos(0, 0, -XD) * Rot(0, -90, 0)   # body -> print: the outer face on the bed


# ====================================================================== the keep-outs
def stance_leg_moving(yaw_deg):
    """What yaws on a leg at the stance pose (pebble_gait's leg_ik(p_nom), about (0, -34, -77)):
    the fork, the hip servo, the femur group and the tibia group (leg_assembly's solids, posed
    about the hip and knee axes), turned yaw_deg about the yaw axis. Leg frame."""
    import leg_assembly as LA
    from leg_frame import L1, L2, L3, KNEE_X, Z_HIP
    parts = _LEG_PARTS.setdefault("p", LA.build())
    r = P["gait"]["stance_radius"] - P["body"]["circumradius"]
    dx, dz = r - L1, -P["gait"]["body_height"] - Z_HIP
    d2 = dx * dx + dz * dz
    q3 = -np.arccos((d2 - L2 * L2 - L3 * L3) / (2 * L2 * L3))
    q2 = np.arctan2(dz, dx) - np.arctan2(L3 * np.sin(q3), L2 + L3 * np.cos(q3))
    fem = Pos(L1, 0, Z_HIP) * Rot(0, -np.rad2deg(q2), 0) * Pos(-L1, 0, -Z_HIP)
    tib = fem * Pos(KNEE_X, 0, Z_HIP) * Rot(0, -np.rad2deg(q3) - 90.0, 0) * Pos(-KNEE_X, 0, -Z_HIP)
    yaw = Rot(0, 0, yaw_deg)
    out = [yaw * parts["coxa_fork"], yaw * parts["servo_femur"]]
    out += [yaw * fem * parts[k] for k in ("coupler_hip", "femur_link", "femur_plate_b", "coupler_knee")]
    out += [yaw * tib * parts[k] for k in ("servo_tibia", "tibia_knee_carrier", "tibia_tube", "foot")]
    return out, (np.rad2deg(q2), np.rad2deg(q3))


_LEG_PARTS = {}


def cradle_box():
    """params stand: the U-cradle's floor under the tub (x cradle_x, 3 thick: ESTIMATE), its two
    walls wall_h tall at tub_clear off the tub's sides, and the x-stop pin (FIT shorter than its
    hole is deep)."""
    x0, x1 = ST["cradle_x"]
    g, t, h = ST["tub_clear"], ST["wall_t"], ST["wall_h"]
    out = [_box(x0, x1, YO0 - g - t, YO1 + g + t, ZB - 3.0, ZB),
           _box(x0, x1, YO1 + g, YO1 + g + t, ZB, ZB + h),
           _box(x0, x1, YO0 - g - t, YO0 - g, ZB, ZB + h)]
    px, py = ST["xstop_xy"]
    ph = ST["xstop_hole_depth"] - FIT
    out.append(Pos(px, py, ZB + ph / 2 - 0.01) * Cylinder(ST["xstop_pin_d"] / 2, ph + 0.02))
    return out


def shelf_box():
    """params hub_shelf: its plan envelope from bottom_z to the deck bottom, + Ø14 pads at the posts."""
    (x0, x1), (y0, y1) = HS["x"], HS["y"]
    out = [_box(x0, x1, y0, y1, HS["bottom_z"], DECK_BOT_Z)]
    for px, py in HS["posts"]:
        out.append(Pos(px, py, (HS["bottom_z"] + DECK_BOT_Z) / 2) * Cylinder(7.0, DECK_BOT_Z - HS["bottom_z"]))
    return out


def skirt_r(az_deg):
    """The carapace skirt's plan radius at body azimuth az (tier 1's outline, part_shell; the
    five sectors are its az-0 sector turned to the stations, the noise tiles at 72 deg)."""
    from part_shell import _outline_r, TIERS
    t1 = TIERS[0]
    return float(_outline_r(np.deg2rad(az_deg - STATIONS[0]), t1[2], t1[3], t1[4], t1[5]))


# ====================================================================== the checks
def main():
    bad = []

    def check(name, ok, detail=""):
        if not ok:
            bad.append(name)
        print(f"  {name}: {'OK' if ok else 'FAIL'}" + (f" ({detail})" if detail else ""), flush=True)

    tub, lid, door = bay_tub(), bay_lid(), bay_door()
    door_print = DOOR_PRINT * door
    export(tub, "bay_tub")
    export(lid, "bay_lid")
    export(door_print, "bay_door")
    sled = sled_tf() * _SLED
    pack = sled_tf() * pack_solid()
    male = sled_tf() * male_xt60()
    frame, webs = carrier_frame()
    carrier = carrier_body()

    print("== params and the stack")
    check(f"bay_l {BS['bay_l']:.1f}: the nose wall's inner face x {XI1:.2f} >= the mate's {XI1_NEED:.2f}",
          XI1 >= XI1_NEED - 1e-6 and XI1 - XI1_NEED < 1.0,
          f"slack {XI1 - XI1_NEED:.2f}; the mate from the sled nose x {SLED_X + PB.NOSE_X1:.1f}: gap {NOSE_GAP}, "
          f"female {XF0:.1f}..{XF1:.1f}, lead room to {XF1 + LEAD_ROOM:.1f}, carrier back {XC0 + CARRIER_L:.1f}")
    bt = tub.bounding_box()
    print(f"  tub outer x {bt.min.X:.2f}..{bt.max.X:.2f} (the nose {XN:.2f}: was 101.9 at bay_l 194, "
          f"{101.9 - XN:+.2f}), y {bt.min.Y:.2f}..{bt.max.Y:.2f}, z {bt.min.Z:.2f}..{bt.max.Z:.2f}")
    sb = sled.bounding_box()
    rest = sb.min.Z - ZF
    side = min(sb.min.Y - YI0, YI1 - sb.max.Y)
    pk = pack.bounding_box()
    foam = ZR - BS["foam_pad_t"]
    stack = rest + PB.SLED_T + PB.PACK_H + BS["foam_pad_t"]
    check("the stack: floor -> rail lift + sled + pack + foam pad <= bay_h",
          stack <= BS["bay_h"] + 1e-6 and pk.max.Z <= foam + 1e-6,
          f"{rest:.2f} (rail {PB.RAIL:.1f} into the {PB.GROOVE_D:.1f} groove) + {PB.SLED_T:.1f} + {PB.PACK_H:.1f} + "
          f"{BS['foam_pad_t']:.1f} = {stack:.2f} of {BS['bay_h']:.1f}; {foam - pk.max.Z:.2f} between the pack and the "
          f"pad (the old pose's 2.25 lift: {2.25 + PB.SLED_T + PB.PACK_H + BS['foam_pad_t']:.2f})")
    check("B72: the sled in bay_w (0.6 a side)", 0.55 < side < 0.65 and abs(sb.size.Y - PB.SLED_W) < 1e-6,
          f"sled {sb.size.Y:.2f} in {BS['bay_w']:.1f}: {side:.2f} a side")
    v = _v(sled & tub)
    dropped = _v((Pos(0, 0, -0.3) * sled) & tub)
    check("the sled rides the rails: sled x tub at rest 0, dropped 0.3 it bears", v < 1e-3 and dropped > 1.0,
          f"{v:.4f} mm^3; dropped 0.3: {dropped:.1f} mm^3 on the rails")
    gy = max(_v((Pos(0, dy, 0.01) * sled) & tub) for dy in (-0.25, 0.25))
    gr = min(_v((Pos(0, dy, 0.01) * sled) & tub) for dy in (-0.4, 0.4))
    check("the rails guide the sled: shifted +-0.25 in y it clears, +-0.4 a groove meets its rail",
          gy < 1e-3 and gr > 0.01, f"{gy:.4f} / {gr:.2f} mm^3 (grooves {PB.RAIL + 2 * FIT:.1f} on {PB.RAIL:.1f} rails: "
          f"+-{FIT}; the walls are {side:.2f} off)")
    check("pack x tub (docked)", _v(pack & (tub + lid)) < 1e-3, f"{_v(pack & (tub + lid)):.4f} mm^3")

    print("== the XT60 mate (pick 4) and the carrier")
    cyz = (carrier - _box(XF1 - 1, XC0 + CARRIER_L + 1, -1e3, 1e3, -1e3, 1e3))
    probe = _box(XC0 + 3.0, XF1 - 3.0, YK - 10, YK + 10, ZC - 6, ZC + 6)
    vb = (probe - cyz).bounding_box()
    dz = (vb.min.Z + vb.max.Z) / 2 - (SLED_Z0 + _PZ)
    dy = (vb.min.Y + vb.max.Y) / 2 - (YC + _PY)
    check("XT60 centres: the carrier's pocket vs the sled's (measured voids), within 0.3",
          abs(dz) <= 0.3 and abs(dy) <= 0.3, f"dz {dz:+.3f}, dy {dy:+.3f}; sled pocket {_PSIZE[0]:.2f} x {_PSIZE[1]:.2f} "
          f"at z {SLED_Z0 + _PZ:.2f}, carrier pocket {vb.size.Y:.2f} x {vb.size.Z:.2f}")
    fem = female_xt60()
    ov = (male & fem).bounding_box() if (male & fem) is not None else None
    eng = 0.0 if ov is None else ov.size.X
    nose_gap = (carrier.bounding_box().min.X) - sb.max.X
    check(f"pushed home the plug mates: male x female overlap {XT60_ENGAGE:.1f} (VERIFY), the nose clear of the mouth",
          abs(eng - XT60_ENGAGE) < 0.01 and nose_gap > 0.3,
          f"{eng:.2f} telescoped; nose face to mouth {nose_gap:.2f}; male x carrier {_v(male & tub):.4f}, "
          f"female x carrier {_v(fem & tub):.4f} mm^3 (glue the sled's male {SLED_XT60_PROUD:.1f} proud of its nose)")
    rest_tub = tub - carrier
    for w in webs:
        rest_tub -= w
    fl = max(_v((Pos(0, sy * FLOAT * a, sz * FLOAT * b) * carrier) & rest_tub)
             for sy in (1, -1) for sz in (1, -1) for a, b in ((1, 0), (0, 1), (1, 1)))
    stop = min(_v((Pos(0, sy * (STOP + 0.15), 0) * carrier) & frame) for sy in (1, -1))
    stopz = min(_v((Pos(0, 0, sz * (STOP + 0.15)) * carrier) & frame) for sz in (1, -1))
    check(f"carrier float +-{FLOAT} in y and z (and both) free of the tub; the window stops it by {STOP + 0.15:.2f}",
          fl < 1e-3 and stop > 0.01 and stopz > 0.01, f"{fl:.4f} mm^3 at +-{FLOAT}; at +-{STOP + 0.15:.2f}: "
          f"y {stop:.2f}, z {stopz:.2f} mm^3 into the window")
    wl, wy, wz = WEB
    E_ = P["fem"]["petg"]["E_mpa"]
    sig = lambda t, dd: 3 * E_ * t * dd / wl ** 2
    k = lambda t, b: 4 * 12 * E_ * (b * t ** 3 / 12) / wl ** 3
    print(f"  flexure webs {wl:.0f} x {wy} x {wz}: fixed-guided at the {FLOAT} float {sig(wy, FLOAT):.1f} / at the "
          f"{STOP:.2f} stop {sig(wy, STOP):.1f} MPa (PETG allowable {P['fem']['petg']['strength_xy_mpa']:.0f}); "
          f"k {k(wy, wz):.1f} N/mm a direction (0.8 at {k(wy, wz) * FLOAT:.1f} N); axial "
          f"{4 * E_ * wy * wz / wl:.0f} N/mm (the push: tension); buckling on the pull "
          f"{4 * 4 * np.pi ** 2 * E_ * (wy * wz ** 3 / 12) / wl ** 2:.0f} N")
    one = len(tub.solids()) == 1
    check("the carrier is one solid with the tub (the webs join it)", one, f"{len(tub.solids())} solid(s)")

    print("== the nose (picks 3 and 10)")
    key = key_xt60()
    pw, ph = XT60_W + FIT, XT60_H + FIT
    kp = _box(XI1 - 0.5, XN + 0.5, YC - pw / 2, YC + pw / 2, KEY_Z - ph / 2, KEY_Z + ph / 2)
    g_frame = kp.distance_to(frame + carrier)
    g_seat = (ZR - (KEY_Z + ph / 2))
    v_key = _v(key & (tub + lid))
    check("the loop key's pocket above the carrier: clear of the carrier window and the roof seat",
          g_frame >= 1.2 and g_seat >= 1.2 - 1e-6 and v_key < 1e-3,
          f"pocket z {KEY_Z - ph / 2:.2f}..{KEY_Z + ph / 2:.2f}; {g_frame:.2f} to the carrier + window, {g_seat:.2f} "
          f"under the roof seat (z {ZR}); the key's body x tub + lid {v_key:.4f} mm^3")
    lr = lead_room()
    free = [("lead room", lr), ("female", fem), ("key", key)]
    vv = {n: _v(s & (tub + lid)) for n, s in free}
    check("the female's lead room, the female and the key's body free of tub + lid",
          max(vv.values()) < 1e-3 and _v(lr & key) < 1e-3,
          ", ".join(f"{n} {v:.4f}" for n, v in vv.items()) + f"; lead room x key {_v(lr & key):.4f} mm^3")
    over = ZR - (ZC + CARRIER_H / 2 + FLOAT)
    over_fr = ZR - (ZC + CARRIER_H / 2 + STOP + WALL)
    print(f"  free height over the carrier for the fuse holder (VERIFY with the real holder; was 14.4): "
          f"{over:.2f} (carrier top + float to the roof), {over_fr:.2f} over its stop window (x {XC0 + 2:.1f}.."
          f"{XC0 + 2 + FRAME_T:.1f}); the key's body takes x {XN - XT60_D:.1f}..{XI1:.1f} of it at y "
          f"{YC - XT60_W / 2:.2f}..{YC + XT60_W / 2:.2f}, beside it {YC - XT60_W / 2 - YI0:.2f} / "
          f"{YI1 - (YC + XT60_W / 2):.2f} wide (to the walls, before the chamfer)")
    # pick 10: the chamfers vs the carrier, the key and the wall thickness. What the interior
    # chamfer takes off the square interior's two nose corners, against what sits in the nose
    corner_cut = _prism([(XI1 - CHAMF_IN, YI1), (XI1 + 5, YI1), (XI1 + 5, YI1 - CHAMF_IN - 5), (XI1, YI1 - CHAMF_IN)], ZF, ZR) + \
        _prism([(XI1 - CHAMF_IN, YI0), (XI1, YI0 + CHAMF_IN), (XI1 + 5, YI0 + CHAMF_IN + 5), (XI1 + 5, YI0)], ZF, ZR)
    g_cc = corner_cut.distance_to(frame + carrier + key + lr)
    wall_ch = ((XN + YO1 - CHAMF) - (XI1 + YI1 - CHAMF_IN)) / R2      # the two chamfer lines' spacing
    g_kp = min(abs((YC + pw / 2) - (YO1 - CHAMF)), abs((YO0 + CHAMF) - (YC - pw / 2)))
    check(f"nose chamfers ({CHAMF:.0f} at 45 deg, interior {CHAMF_IN:.2f}) miss the carrier, its window, the key "
          f"and keep the {WALL} wall", g_cc > 1.0 and abs(wall_ch - WALL) < 1e-6 and g_kp > 1.2,
          f"{g_cc:.2f} from the chamfered-off corners to the carrier + window + key + lead room; the key pocket "
          f"{g_kp:.2f} from where the chamfers start on the nose face; wall normal to the chamfer {wall_ch:.3f}")
    for name, y_edge, sgn in (("south", YO0, 1), ("north", YO1, -1)):
        sq = (XN, y_edge)
        ch = [(XN - CHAMF + t * CHAMF, y_edge + sgn * t * CHAMF) for t in np.linspace(0, 1, 41)]
        r_sq = np.hypot(*sq)
        r_ch = max(np.hypot(x, y) for x, y in ch)
        past_sq = r_sq - skirt_r(np.rad2deg(np.arctan2(sq[1], sq[0])))
        past_ch = max(np.hypot(x, y) - skirt_r(np.rad2deg(np.arctan2(y, x))) for x, y in ch)
        print(f"  nose-{name} corner: body r {r_sq:.2f} -> {r_ch:.2f} chamfered; past the carapace skirt's plan "
              f"{past_sq:+.2f} -> {past_ch:+.2f} (+ = outside it)")

    print("== the door end (pick 12)")
    tf = _strike_tf()
    from part_panel import latch_engagement, latch_verdict, latch_report, latch_seat
    m = latch_engagement(tub, tf)
    check("I3 latch on the south boss's strike (part_panel.latch_engagement + latch_verdict)", not latch_verdict(m),
          latch_report(m) + ("" if not latch_verdict(m) else "; " + "; ".join(latch_verdict(m))))
    cut = (tf * latch_strike(STRIKE_T)).bounding_box()
    w_y = min(cut.min.Y - (LB_Y - PAD / 2), YO0 - cut.max.Y)
    w_z = min(cut.min.Z - ZB, LB_Z + PAD / 2 - cut.max.Z)
    check("the strike cutter inside its boss with >= 1.2 of wall (and the south wall untouched)",
          w_y >= 1.2 - 1e-3 and w_z >= 1.2 - 1e-3 and cut.max.Y < YO0, f"cutter y {cut.min.Y:.2f}..{cut.max.Y:.2f}, "
          f"z {cut.min.Z:.2f}..{cut.max.Z:.2f}: wall {w_y:.2f} (y) / {w_z:.2f} (z); boss x {LB_X0:.1f}..{LB_X0 + STRIKE_T:.1f}")
    ms = latch_seat(door, tf)
    pk = (tf * latch_pocket(-1.0, PL["housing_t"] + 1.0)).bounding_box()
    ear_w = min(pk.min.Y - (LB_Y - POCKET_R - EAR_WALL), (YO0 - FIT) - pk.max.Y, pk.min.Z - ZB,
                (LB_Z + POCKET_R + EAR_WALL) - pk.max.Z)
    check("the door's cartridge pocket holds the housing on its keyway index (part_panel.latch_seat), "
          f">= {EAR_WALL} of ear round it", ms["seated"] < 0.01 and ms["turned"] >= 1.0 and ear_w >= EAR_WALL - 1e-3,
          f"on its index {ms['seated']:.3f} mm^3 (play +-{ms['play']:.1f} deg), turned +-10 / a half turn >= "
          f"{ms['turned']:.1f}; pocket y {pk.min.Y:.2f}..{pk.max.Y:.2f}, z {pk.min.Z:.2f}..{pk.max.Z:.2f}: ear wall {ear_w:.2f}")
    run = None
    drops0 = body_keepouts(0.0)["drops"]
    for L in np.arange(10, 201, 10):
        col = tf * Pos(0, 0, -LATCH_REACH - L / 2 - 0.3) * Cylinder(2.5, L)
        if _v(col & (tub + lid)) > 1e-3 or any(_v(col & dd) > 1e-3 for dd in drops0):
            break
        run = L
    print(f"  the latch's slot faces +x at x {LB_X0 + LATCH_REACH:.1f} (y {LB_Y:.1f}, z {LB_Z:.1f}): a Ø5 driver along it "
          f"is free of tub, lid and the five drop keep-outs for {run} mm (its axis {DECK_BOT_Z - 35 - LB_Z:.2f} under "
          f"leg 2's modelled drop bottom, the Ø5's top {DECK_BOT_Z - 35 - LB_Z - 2.5:.2f}); or slot the rotor's head, "
          f"outside the door")
    v_d = _v(door & (tub + lid))
    hooked = _v((Pos(-0.5, 0, 0) * door) & tub)
    barb_in = _v((Pos(-0.5, 0, 0) * door) & _box(KEEP_X[0], KEEP_X[1], YO1 - 1, YO1 + KEEP_P, *TAB_Z))
    check("the door closed: x tub + lid 0; pulled 0.5 off, its barb meets the keeper (hooked)",
          v_d < 1e-3 and barb_in > 0.01, f"{v_d:.4f} mm^3; pulled 0.5: {hooked:.2f} mm^3 in all, {barb_in:.2f} at the keeper")
    tab_l = (KEEP_X[1] + FIT + BARB_L / 2) - XI0
    print(f"  the hook tab: a {TAB_T} x {TAB_Z[1] - TAB_Z[0]:.0f} cantilever {tab_l:.1f} long, the barb {KEEP_P} "
          f"onto the keeper: bent {KEEP_P + FIT:.1f} to pass it, strain 1.5 t d / L^2 = "
          f"{100 * 1.5 * TAB_T * (KEEP_P + FIT) / tab_l ** 2:.2f} % (PETG ~4 % to yield); push its free end out to release")
    press = _v(door & sled)
    lip_x = SLED_X + PB.TAIL_X0
    boss_face = (SLED_X + PB.TAIL_X0) + PRESS
    check(f"I5: the door's boss presses the sled's tail lip 0.3..0.8", 0.3 <= boss_face - lip_x <= 0.8 and press > 1.0,
          f"press {boss_face - lip_x:.2f} (the boss face x {boss_face:.2f}, the lip x {lip_x:.2f} with the plug mated); "
          f"{press:.1f} mm^3 of overlap, taken by the door's 3 mm plate and the latch's 0.2 bite")

    print("== the lid-to-tub joint")
    v_lt = _v(lid & tub)
    held = _v((Pos(0, 0, -0.3) * tub) & lid)
    check("lid on tub: 0 mm^3 posed; the tub dropped 0.3 meets the lid (the north return holds it)",
          v_lt < 1e-3 and held > 1.0, f"{v_lt:.4f}; dropped 0.3: {held:.1f} mm^3")
    path = max(_v((Pos(0, SLIDE, dz) * tub) & lid) for dz in (-4.0, -2.0, -0.5, -0.05))
    slide = max(_v((Pos(0, dy, -0.02) * tub) & lid) for dy in np.arange(SLIDE, -0.01, -0.4))
    check(f"the tub goes on: {SLIDE} north it rises past the lid's edge, then slides south under it",
          path < 1e-3 and slide < 1e-3, f"rising {path:.4f}, sliding {slide:.4f} mm^3 (0.02 under the lid)")
    hh = PR["heatset_m3_h"]
    screws, cols = [], []
    for x, y in PIL_XY:
        s = Pos(x, y, PIL_SEAT - 1.5) * Cylinder(2.75, 3.0) + Pos(x, y, PIL_SEAT + LID_SCREW_L / 2) * Cylinder(1.5, LID_SCREW_L)
        screws.append(s)
        z0, z1 = -100.0, PIL_SEAT - 3.0 - 0.01
        cols.append(Pos(x, y, (z0 + z1) / 2) * Cylinder(2.5, z1 - z0))
    v_s = sum(_v(s & (tub + lid)) for s in screws)
    tip = PIL_SEAT + LID_SCREW_L
    eng = tip - ZR
    col_v = sum(_v(c & (tub + lid + door)) for c in cols)
    check(f"the two M3 x {LID_SCREW_L:.0f} lid screws seat, bite the inserts and are reached from below",
          v_s < 1e-3 and 3.5 <= eng <= hh - 0.3 and col_v < 1e-3,
          f"screws x tub + lid {v_s:.4f}; tip z {tip:.1f}: {eng:.1f} into the insert (z {ZR:.1f}..{ZR + hh:.1f}, "
          f"{ZR + hh - tip:.1f} short of its end); a Ø5 column up each pilaster bore x tub + lid + door {col_v:.4f} mm^3")
    # why no north screw: the room outside the north wall
    hooks_any = body_keepouts(0.0, "any")["hooks"]
    nb = []
    for x in np.arange(-95.0, 96.0, 5.0):
        p = Pos(x, YO1 + 0.3 + 4.0, (ZR + ZT) / 2 + 3.0) * Cylinder(4.0, 8.0)       # an Ø8 insert boss
        ok = _v(p & shelf_box()[0]) < 1e-3 and all(_v(p & h) < 1e-3 for h in hooks_any) and \
            all(_v(p & dd) < 1e-3 for dd in body_keepouts(0.0)["drops"])
        nb.append(ok)
    xs = [x for x, ok in zip(np.arange(-95.0, 96.0, 5.0), nb) if ok]
    print(f"  why no north screw: an Ø8 insert boss outside the north wall (y {YO1 + 0.3:.1f}..{YO1 + 8.3:.1f}, z "
          f"{ZR:.0f}..{ZT + 6:.0f}) clears the shelf, the hook envelopes and the drops only at x "
          f"{', '.join(f'{x:.0f}' for x in xs)} (of -95..95 step 5): the door's hook tab holds the west end, the "
          f"east end is the nose chamfer; the north return holds the whole north edge instead")

    print("== B126: the three turn points (the sled OUT)")
    for (name, (x, y)), c in zip(TURN_POINTS, _turn_columns(d=COL_D)):
        rail_g = min(c.distance_to(_box(RAIL_X0, RAIL_X1, YC + s * PB.GROOVE_Y - 2, YC + s * PB.GROOVE_Y + 2,
                                        ZF, ZF + PB.RAIL) & tub) for s in (1, -1))
        gc = c.distance_to(carrier + frame)
        bosses = Compound(children=[Pos(hx, hy, -14) * Cylinder(TUB_BOSS_D / 2, 8) for _, (hx, hy), *_ in HANGERS] +
                          [Pos(px, py, -25) * Cylinder(PIL_D / 2, 30) for px, py in PIL_XY])
        gb = c.distance_to(bosses)
        dr = Pos(x, y, (-60 + -10) / 2) * Cylinder(2.5, 50)
        vdr = _v(dr & (tub + lid))
        check(f"{name} at ({x:.2f}, {y:.2f}): its Ø{COL_D:.0f} column clear of the rails, the carrier and the bosses; "
              f"a Ø5 z -60..-10 meets nothing", vdr < 1e-3 and rail_g > 0.3 and gc > 1.0 and gb > 1.0,
              f"Ø5 x tub + lid {vdr:.4f} mm^3; to the rails {rail_g:.2f}, the carrier {gc:.2f}, the bosses {gb:.2f}")
    sw = YO0 + WALL
    print(f"  the tray strike's column cuts {max(0.0, (sw) - (TURN_POINTS[2][1][1] - COL_D / 2)):.2f} into the south "
          f"wall's inner face (a half-round notch; {WALL - max(0.0, sw - (TURN_POINTS[2][1][1] - COL_D / 2)):.2f} of "
          f"wall left), the south rail is broken over it")

    print("== hangers, lanes, the deck's hole table")
    hd_ok = []
    for name, (x, y), kind, *_ in HANGERS:
        ray = Pos(x, y, (ZT + -4.0) / 2) * Cylinder(0.25, -4.0 - ZT)
        hd_ok.append((name, kind == "m3_top", _v(ray & lid)))
    worst = max(v for *_, v in hd_ok)
    check("each hanger boss's axis is a DECK_HOLES m3_top entry; a thin ray z -18..-4 meets no lid material",
          len(hd_ok) == 6 and all(k for _, k, _ in hd_ok) and worst < 1e-3,
          f"{len(hd_ok)} axes, ray x lid {worst:.4f} mm^3 (the deck's own holes are part_deck's: its audit)")
    lanes = _lane_boxes()
    bosses8 = [Pos(x, y, (ZT + DECK_BOT_Z) / 2) * Cylinder(TUB_BOSS_D / 2, DECK_BOT_Z - ZT) for _, (x, y), *_ in HANGERS]
    bosses8 += [Pos(x, y, (ZT + ZR + hh + 1.0) / 2) * Cylinder(LID_BOSS_D / 2, ZR + hh + 1.0 - ZT) for x, y in PIL_XY]
    bc = Compound(children=bosses8)
    per = [(l.distance_to(bc), l.distance_to(tub + lid)) for l in lanes]
    lv = sum(_v(l & (tub + lid + door)) for l in lanes)
    names = [f"{n}{'+' if s > 0 else '-'}x" for n in "ABC" for s in (1, -1)]
    check("the leg 2/3 lanes (8 x 7, correction 12) clear of tub + lid, >= 1.0 to the bosses",
          lv < 1e-3 and min(g for g, _ in per) >= 1.0 - 1e-6,
          f"{lv:.4f} mm^3; each lane to the bosses / to tub + lid: " +
          ", ".join(f"{n} {g:.2f}/{t:.2f}" for n, (g, t) in zip(names, per)))
    print(f"  the hanger screws (M3 x 16 from the deck top): at (+-58, -18) the tip reaches z "
          f"{-4.0 - 16.0:.1f}, flush with the roof's underside (no longer); through a 2 mm tray-rail tab it stops "
          f"{2.0:.1f} above it")

    print("== keep-outs (body frame, posed)")
    assy = tub + lid + door
    ko0, ko5, koa = body_keepouts(0.0), body_keepouts(5.0), body_keepouts(0.0, "any")
    for tag, group in (("hook envelopes ('any')", koa["hooks"]), ("drop keep-outs", ko0["drops"])):
        vs = [_v(assy & s) for s in group]
        gs = [assy.distance_to(s) for s in group]
        check(f"tub + lid + door x the five {tag}", max(vs) < 1e-3,
              f"{max(vs):.4f} mm^3; closest {min(gs):.2f} (" + ", ".join(f"{g:.2f}" for g in gs) + ")")
    v5 = [_v(assy & s) for s in ko5["drops"]]
    print(f"  INFO the drops + 5 (iface.leg_drop_keepout(5), +-15.5): {', '.join(f'{v:.1f}' for v in v5)} mm^3. The bare "
          f"params box takes the same (196.7 legs 1/4, 416.9 legs 2/3): the tub's band is fact 3's 5 mm round each "
          f"cutout, iface's +0 (DROP_HALF 10.5 already is the cutout + 5)")
    from part_deck import body_deck
    deck = Pos(0, 0, -10) * body_deck()
    check("x the posed deck", _v(assy & deck) < 1e-3, f"{_v(assy & deck):.4f} mm^3")
    from part_coxa import coxa_yaw_base
    bases = [station_tf(i) * coxa_yaw_base() for i in range(len(STATIONS))]
    vb_ = [_v(assy & b) for b in bases]
    check("x the five docked coxa bases", max(vb_) < 1e-3, f"{max(vb_):.4f} mm^3; closest {min(assy.distance_to(b) for b in bases):.2f}")
    from part_shell import shell_sector, shell_cap
    sec = shell_sector()
    shell = [Rot(0, 0, a) * sec for a in STATIONS] + [Rot(0, 0, STATIONS[0]) * shell_cap()]
    zmin = min(s.bounding_box().min.Z for s in shell)
    vs = sum(_v(assy & s) for s in shell)
    check("x the carapace (five sectors + cap, at the stations)", vs < 1e-3,
          f"{vs:.4f} mm^3; its lowest z {zmin:.2f} vs the assembly's top {assy.bounding_box().max.Z:.2f}")
    cr = cradle_box()
    vc = sum(_v(assy & c) for c in cr)
    gw = min(assy.distance_to(c) for c in cr[1:3])
    xbind = min(_v((Pos(dx, 0, 0) * tub) & cr[3]) for dx in (-1.0, 1.0))
    check("x the stand cradle (floor, walls, x-stop pin) 0; shifted 1 along x the pin binds",
          vc < 1e-3 and xbind > 0.1, f"{vc:.4f} mm^3; walls {gw:.2f} off; the tub shifted +-1: {xbind:.2f} mm^3 on the pin")
    sh = shelf_box()
    vsh = sum(_v(assy & s) for s in sh)
    check("x the hub shelf box (+ Ø14 post pads)", vsh < 1e-3, f"{vsh:.4f} mm^3; closest {min(assy.distance_to(s) for s in sh):.2f}")
    worst_leg, q = 0.0, None
    abb = assy.bounding_box()
    for i in range(len(STATIONS)):
        for yaw in range(-40, 41, 10):
            mov, q = stance_leg_moving(yaw)
            for s in mov:
                ps = station_tf(i) * s
                sb2 = ps.bounding_box()
                if sb2.max.X < abb.min.X or sb2.min.X > abb.max.X or sb2.max.Y < abb.min.Y or \
                        sb2.min.Y > abb.max.Y or sb2.max.Z < abb.min.Z or sb2.min.Z > abb.max.Z:
                    continue
                worst_leg = max(worst_leg, _v(ps & assy))
    check(f"x the stance legs (q {q[0]:.1f} / {q[1]:.1f} deg), yaw -40..40 step 10, all five stations",
          worst_leg < 1e-3, f"{worst_leg:.4f} mm^3 (fork, hip servo, femur group, tibia group; bounding boxes first)")

    print("== the sled's way out (door off)")
    hooks_a = koa["hooks"]
    worst = (0.0, None)
    for dx in range(0, 181, 10):
        moving = sled_tf(-dx) * (_SLED + pack_solid() + male_xt60())
        v1 = _v(moving & (tub + lid))
        v2 = max(_v(moving & s) for s in bases + hooks_a + ko0["drops"])
        if max(v1, v2) > worst[0]:
            worst = (max(v1, v2), dx)
    check("sled + pack + its XT60 slid -x 0..180 (10 mm steps) x tub + lid, the bases, hooks and drops",
          worst[0] < 1e-3, f"worst {worst[0]:.4f} mm^3" + (f" at {worst[1]} mm" if worst[1] is not None else ""))

    print("== print")
    bed = PR["bed_mm"]
    for n, s in (("bay_tub", tub), ("bay_lid", lid), ("bay_door", door_print)):
        b = s.bounding_box()
        fits = sorted((b.size.X, b.size.Y))[0] <= min(bed[:2]) and max(b.size.X, b.size.Y) <= max(bed[:2]) and b.size.Z <= bed[2]
        check(f"{n}: one solid, fits the bed", len(s.solids()) == 1 and fits,
              f"{len(s.solids())} solid, {b.size.X:.1f} x {b.size.Y:.1f} x {b.size.Z:.1f} mm, {s.volume / 1000:.1f} cm^3")
    print("  printability: bay_tub open top up, floor on the bed; the carrier floats 2.9 over the floor (supports "
          "under it, through the open top) and its webs bridge 14; the latch boss stands on the bed, the "
          "pilasters on 45-deg wedges off the wall; the north return is a 1.2 overhang at the top. bay_lid bottom down, bosses up. bay_door on its outer "
          "face: the ear, the spigot, the boss and the tab stand up from it")
    print(f"part_bay checks: {'CLEAN' if not bad else 'FAIL — ' + '; '.join(bad)}")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
