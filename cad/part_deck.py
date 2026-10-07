"""Body deck, v0.5 — the body layout (D064): the deck cuts option A's hole table.

v0.2 -> v0.3: the per-station M3 hole pairs are replaced by the leg-port
interface from iface.py (seat POSTS since 2026-10-07, were dowel posts;
thumbscrew heat-set pockets, hook through-slot, cable cutout) — the same
code path the bench jig uses, so a leg moves between jig and robot without
re-fitting (check_dock.py proves the dock itself). The v0.2 mid-pair holes
turned out to sit ~2 mm OUTSIDE the pentagon (layout audit 07-30, caught by
the new containment check below) — nothing was printed, no filament died.

v0.3 -> v0.4 (session 5b): the carapace sectors (part_shell.py) carry
latch pads + magnets but had nothing to bite on — now the deck answers:
  * 5 LATCH STRIKES at az = station+27.5°, r 74.5 (under each sector's
    latch pad): iface.latch_strike (B87, D063) — a bore, two entry slots
    through a 3.2 land and an underside recess; a quarter-turn from below
    cams the rotor's lugs up under the land (0.2 bite at 90°).
  * 5 WASHER RECESSES (Ø8 x 1.4, glue an M3 washer) at az = station
    -25.5°, r 76 — under each sector's foot magnet. (The v0 shell wanted
    three magnets per web at r 77.5; their recesses would breach the
    pentagon edge at the az±34 spots — shell amended to one per web.)
  * (retired in v0.5) a power-entry grommet Ø9 at (52, -6) + two zip anchors
    (the star-board proposal in docs/WIRING_HARNESS.md).

v0.4 -> v0.5 (D064, 2026-10-07, the owner's body-layout picks: option A, the keel tub under
the deck + the north hub shelf, the 8 mm layer, the tray at (0, 0) turned 180): body_deck()
cuts DECK_HOLES, the one table the tray rails, the tub's lid, the hub shelf and the tray latch
build to (21 entries; deck_hole_audit holds the table, __main__ measures it as cut):
  * (+-40, -20 / 0 / 20): six v0.4 grid holes opened 2.8 -> 3.4 for the tray rails' tabs,
    turned inboard (B51: (0, 0) is the only tray pose clear of the five coxa bases); the four
    south ones also hang the tub (one M3 x 16 through tab + deck into the bay_lid boss's insert);
  * (+-58, -18): the tub's other two hangers (B84), new;
  * the hub shelf's three posts (+-50, 34), (26, 64): BLIND Ø2.8 from the underside, 5 deep
    under a 1.0 skin. The M3 comes up through the shelf plate and the post: a head on the deck
    top at (+-50, 34) sits under the docked coxa plate's extension (the seats fix runs it
    inboard to the hook's stem; __main__ measures the clash), and the shelf would come off only
    with legs 1 and 4 undocked (picks 1 + 14, BODY_LAYOUT correction 10);
  * (0, 50): a 12.5 x 7.5 r1 slot (E-W) for the tray trunk (XHP-5) and the tray XT30 with their
    plugs on, and the J6 foot-switch lines (B122, B123, correction 15: the proposal's Ø10
    grommet passed no plug), zip anchors at (+-12, 50);
  * (0, -43): the tray's I3 strike under its tongue (B87's strike; the tray turned 180 puts the
    tongue south; B126: the driver column crosses the tub's plan and the tub leaves it open).
    Its seating plane is the deck top, as at the sector strikes: the tray's tongue must come
    down to it (the strike convention at DECK_HOLES; the pre-D064 tongue floats 3.4 over it);
  * six grid holes stay Ø2.8, unused (trim-cup points).
  Removed: the four strap slots (+-26, +-22) (B50 closes: pick 8 (b), no strap inside the
  robot, the tub boxes the pack), the (52, -6) grommet and its two zip anchors (the power comes
  from under the deck now: the tub's nose to the shelf), grid hole (0, -40) (it overlaps the
  strike). Unchanged: the five I1 ports (seats, d99cdc2), the station dots, the north arrow,
  the five sector strikes and the washer recesses (kept_cuts(): the one list body_deck() cuts
  and the audit tests against).

Deck frame = BODY frame (origin center). Deck TOP surface = leg-frame z -4
(coxa plates are 4 mm; servo bases sit at leg z 0). Stations at
robot.legs first_station_deg + i*360/count (90 + 72i), radius
body.circumradius. The 6 mm thickness is B27 (OPEN: params body.deck_t says 4, unread). v0.5
builds more on the 6: the shelf posts' 5 deep + 1.0 skin and the tray strike's frame (>=
LATCH_REACH + 0.2 = 6.0, as the sector strikes), so unifying B27 at 4 would now break the posts
and both kinds of strike, not only the hook (iface.hook_geometry, part_port_coupon.T_DECK).
Part is modeled z 0..T (print flat, seat posts up); assembly -10..-4.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import (leg_port_deck_features, leg_port_seats, IF, latch_strike, SHELL_LATCH_AZ,
                   SHELL_LATCH_R, STATIONS, R_STATION, station_tf, bay_tub_extent)

P = params()
PR = P["print"]
R_DECK = 100.0
T = 6.0                                   # B27 OPEN: params body.deck_t (4.0) is unread
# STATIONS (90, 162, 234, 306, 378: not wrapped, like pebble.xml) and R_STATION (110) are
# iface's since 2026-10-07, so the deck and the under-deck keep-outs pose a port the same way

# ---- option A's deck holes: the CONTRACT the body layout builds to (2026-10-07) -------------
# docs/BODY_LAYOUT_PROPOSAL.md s3 'Deck holes (A)' with its corrections 10 (the shelf posts) and
# 15 (the (0, 50) slot), on the owner's picks (option A, the 8 mm layer, the tray at (0, 0)
# turned 180 with its tabs inboard, the three boards on the hub shelf). The tray rails, the
# tub's lid bosses, the shelf's posts and the tray's latch bolt to THESE; body_deck() cuts them
# (v0.5) and every other part ray-tests against this TABLE, not the solid. Of v0.4's 13 grid
# holes 6 are reused, 6 stay unused, (0, -40) goes (it overlaps the strike); 9 features are new.
#   (name, (x, y) body, kind, size, job). kind / size:
#     m3_top       (Ø,)           through, an M3 driven from the deck top (its head on top)
#     zip          (Ø,)           through, a zip-tie anchor (no head)
#     tap          (Ø,)           through, thread-forming: unused grid holes, as before
#     m3_below     (Ø, depth)     BLIND from the underside, thread-forming: the top skin stays
#     slot         (w, h, r)      through, w along x, rounded corners r
#     strike       (frame deg,)   iface.latch_strike(T), cut at strike_tf(): Pos(x, y, T) *
#                                 Rot(0, 0, deg). THE CONVENTION: that is the panel's own latch
#                                 frame posed on the deck (z 0 = the seating plane = the deck top,
#                                 +x = the strike's +x, the lugs' entry line LATCH_ENTRY_DEG from
#                                 it). For the tray it is part_avionics.tongue_latch_tf() (tray-
#                                 local; +x = the tray's +x, z 0 = the tongue's underside, where the
#                                 housing sits flush) posed by Pos(*tray_xy, 0) * Rot(0, 0,
#                                 tray_rot_deg): deg = tray_rot_deg (180: the strike's +x is body
#                                 -x). The strike is two-fold (lugs at 0 and 180), so 0 and 180 cut
#                                 the same solid here; the angle still names the frame.
#                                 AND the seating plane must sit ON the deck top, as a sector foot
#                                 does: the cartridge reaches LATCH_REACH 5.8 below it, the lugs
#                                 cam under the land 3.2 down. The pre-D064 tray's tongue floats
#                                 3.4 over the deck (its rails' channel pose), 5.4 with tray_lift,
#                                 where the latch fails three of its five rules, the cam, the land
#                                 and the stops (the lugs meet the land at 5 deg; __main__): the
#                                 tongue needs a boss down to the deck, its underside the seating
#                                 face. deck_hole_audit checks it once part_avionics names TRAY_Z
#                                 (the plate underside over the deck top): TRAY_Z +
#                                 tongue_latch_tf().Z = 0
_HS = IF["hub_shelf"]
_AT = IF["avionics_tray"]
_POST = (("m3_below", (PR["screw_m3_tap"], 5.0)) if _HS["post_screw"] == "below"
         else ("m3_top", (PR["screw_m3_clear"],)))
_M3 = (PR["screw_m3_clear"],)
_TAP = (PR["screw_m3_tap"],)
DECK_HOLES = (
    [(f"tray_tab_tub_{sx:+d}_{sy}", (float(sx), float(sy)), "m3_top", _M3,
      "tray-rail tab + tub hanger: one M3 x 16 through the tab and the deck into the bay_lid "
      "boss's insert (was a 2.8 grid hole)") for sy in (-20, 0) for sx in (-40, 40)] +
    [(f"tray_tab_{sx:+d}_20", (float(sx), 20.0), "m3_top", _M3,
      "tray-rail tab (no longer a shelf hanger: correction 10; was a 2.8 grid hole)")
     for sx in (-40, 40)] +
    [(f"tub_hanger_{sx:+d}", (float(sx), -18.0), "m3_top", _M3,
      "tub hanger: M3 x 16 into the bay_lid boss's insert (new)") for sx in (-58, 58)] +
    [(f"shelf_post_{k}", (float(x), float(y)), _POST[0], _POST[1],
      f"hub shelf post (params hub_shelf.posts, post_screw {_HS['post_screw']}): the M3 comes up "
      "through the plate and the Ø8 post; a head on the deck top at (+-50, 34) would sit under "
      "the docked coxa plate (new)") for k, (x, y) in enumerate(_HS["posts"])] +
    [("trunk_slot", (0.0, 50.0), "slot", (12.5, 7.5, 1.0),
      "the tray trunk (XHP-5) and the tray XT30 with their plugs on (XT30 1.15 a side, XHP-5 22 AWG "
      "two-layer 0.36), and the J6 foot-switch lines, down to the shelf (B122, B123; was a Ø10 "
      "grommet, correction 15: it passes no plug)")] +
    [(f"zip_{sx:+d}_50", (float(sx), 50.0), "zip", _M3, "zip anchor for the trunk (new)")
     for sx in (-12, 12)] +
    [("tray_strike", (0.0, -43.0), "strike", (float(_AT["tray_rot_deg"]),),
      "the tray's I3 latch strike: the tongue's frame turned with the tray (tray_rot_deg); its "
      "rotor is worked from below, and that driver column crosses the tub's plan (B126: the tub "
      "leaves it open)")] +
    [(f"grid_{x:+d}_{y:+d}", (float(x), float(y)), "tap", _TAP,
      "unused grid hole: a trim-cup point" + (" (was the star-board bracket's)" if x == 0 and y > 0
                                                else ""))
     for x, y in ((0, 0), (-20, 0), (20, 0), (0, -20), (0, 20), (0, 40))])
# what the table retired from v0.4's body_deck() (B50, B51, B84, B122): __main__ proves each is
# solid deck again (outside the table's own cutters), with these v0.4 cutters as the probes
DECK_RETIRED = (
    [(f"strap slot ({sx:+d}, {sy:+d})", Pos(sx, sy, T / 2) * Box(5.0, 30.0, T - 0.002))
     for sx in (-26, 26) for sy in (-22, 22)] +
    [("grommet (52, -6)", Pos(52.0, -6.0, T / 2) * Cylinder(4.5, T - 0.002))] +
    [(f"zip anchor ({zx:.0f}, -14)", Pos(zx, -14.0, T / 2) * Cylinder(1.7, T - 0.002)) for zx in (44.0, 60.0)] +
    [("grid (0, -40)", Pos(0.0, -40.0, T / 2) * Cylinder(PR["screw_m3_tap"] / 2, T - 0.002))])
DECK_HOLE_GAP, DECK_HOLE_GAP_PREF = 2.0, 3.3     # to every other cut / an I1 feature set; the
DECK_HOLE_EDGE = 2.0                             # table's new holes keep 3.3 from each other
HEAD_D, HEAD_CLEAR = 6.0, 0.5                    # an M3 head + driver from above vs the docked bases
TUB_BOSS_D = 8.0                                 # the bay_lid's hanger bosses (params bay_roof; part_bay's)
TAB_HOLE_TOL = 0.05                              # a rail tab hole on its deck hole (a Ø3.4 pair: 0.05 of 0.6)
TAB_HOLE_INBOARD = 7.3                           # BODY_LAYOUT s2 'The tray (B51)': the inboard tabs' hole
                                                 # off the rail block's axis (used only on a pre-D064 rail)
TRAY_PRE_D064_Z = 3.4                            # the pre-D064 tray's plate underside over the deck top:
                                                 # part_avionics.__main__ poses the rail at z (T - 1) - 5.4
                                                 # in the tray's frame, the rail's foot on the deck


def strike_tf(entry):
    """A strike entry's latch frame in the deck's model frame (see the convention above)."""
    _, (x, y), kind, size, _ = entry
    assert kind == "strike", kind
    return Pos(x, y, T) * Rot(0, 0, size[0])


def deck_hole_cutter(entry):
    """The cutter of one DECK_HOLES entry in the deck's model frame (z 0..T, top at T)."""
    _, (x, y), kind, size, _ = entry
    if kind in ("m3_top", "zip", "tap"):
        return Pos(x, y, T / 2) * Cylinder(size[0] / 2, T + 2)
    if kind == "m3_below":
        return Pos(x, y, (size[1] - 1.0) / 2) * Cylinder(size[0] / 2, size[1] + 1.0)
    if kind == "slot":
        w, h, r = size
        return Pos(x, y, -1.0) * extrude(RectangleRounded(w, h, r), T + 2)
    if kind == "strike":
        return strike_tf(entry) * latch_strike(T)
    raise ValueError(kind)


def latch_strike_tf(ang):
    """The I3 strike under station ang's sector latch pad, in the deck's model
    frame: z 0 of the strike at the deck top (T), its +x radial."""
    return Rot(0, 0, ang + SHELL_LATCH_AZ) * Pos(SHELL_LATCH_R, 0, T)


def kept_cuts():
    """Everything body_deck() cuts besides the five I1 ports and DECK_HOLES, model frame, as
    [(name, cutter)]: the five sector strikes, the five washer recesses, the station dots and
    the north arrow. body_deck() cuts exactly these and deck_hole_audit() tests the table
    against exactly these (until v0.5 the audit carried hand copies)."""
    out = []
    for k, ang in enumerate(STATIONS):
        # latch strike under the sector's latch pad (az +27.5, r 74.5): the I3 bayonet's frame
        # side, iface.latch_strike (B87, D063): the bore, two entry slots through a 3.2 land, and
        # the underside recess the rotor's lugs cam up into. The old strike was blind from the top
        # with a 2 mm land and no undercut: pegs in it hit 7.95 mm^3 of deck at 18 deg.
        out.append((f"sector_strike_{k}", latch_strike_tf(ang) * latch_strike(T)))
        # washer recess under the sector's foot magnet (az -25.5, r 76): Ø8 x 1.4 pocket in the
        # top face, M3 washer glued in
        out.append((f"washer_{k}", Rot(0, 0, ang - 25.5) * Pos(76.0, 0, T - 0.7 + 0.01) * Cylinder(4.0, 1.4)))
        # station numbering (backlog B10): k+1 engraved dots by each port — font-free, readable
        # with a headlamp, survives every slicer
        out += [(f"dot_{k}_{d}", station_tf(k) * Pos(-36 + 5 * d, -24, T - 0.5) * Cylinder(1.5, 1.2))
                for d in range(k + 1)]
    # leg-0 = "north" arrow (heading is software, but humans need a datum)
    out.append(("north_arrow", Pos(0, 30, T - 0.5) * extrude(Triangle(a=10, b=10, c=10), 1.2)))
    return out


def grid_holes():
    """The 12 holes left of v0.4's 20 mm electronics grid, body xy: the six opened to Ø3.4 for
    the tray tabs / tub hangers and the six unused Ø2.8 ones. A thin wrapper kept for importers:
    DECK_HOLES is the contract (kind, size and job per hole)."""
    return [xy for _, xy, kind, _, _ in DECK_HOLES
            if kind in ("m3_top", "tap") and xy[0] % 20 == 0 and xy[1] % 20 == 0]


def _station_xy(ang_deg, leg_x, leg_y, rb=R_STATION):
    a = np.deg2rad(ang_deg)
    c, s = np.cos(a), np.sin(a)
    return rb*c + c*leg_x - s*leg_y, rb*s + s*leg_x + c*leg_y

def pentagon_contains(px, py, margin=2.5):
    """Is (px,py) inside the R100 pentagon with `margin` to every edge?"""
    r = np.hypot(px, py)
    if r < 1e-9:
        return True
    phi = np.rad2deg(np.arctan2(py, px))
    off = abs(((phi - 90) + 36) % 72 - 36)          # angle from nearest vertex
    boundary = R_DECK * np.cos(np.deg2rad(36)) / np.cos(np.deg2rad(36 - off)) \
        if off <= 36 else 0
    return r <= boundary - margin


def body_deck():
    deck = extrude(RegularPolygon(R_DECK, 5, major_radius=True, rotation=90), T)
    for k in range(len(STATIONS)):
        deck = leg_port_deck_features(deck, top_z=T, station_tf=station_tf(k))
    for _, c in kept_cuts():
        deck -= c
    # v0.5 (D064): option A's hole table
    for e in DECK_HOLES:
        deck -= deck_hole_cutter(e)
    return deck


# ------------------------------------------------------------------ the audit (in plan)
def _plan(shape, tol=0.002):
    """A solid's plan shadow: its tessellation's triangles projected on xy and unioned (vertical
    faces drop out), a shapely geometry. Chords sit inside the true outline by <= ~tol, so a
    plan distance reads up to ~tol long."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    vs, ts = shape.tessellate(tol)
    xy = np.array([(v.X, v.Y) for v in vs])
    tri = [Polygon(xy[list(t)]) for t in ts]
    return unary_union([p for p in tri if p.area > 1e-9]).buffer(0)


def _filled(g):
    """g with its holes filled (a foot over a pocket still covers it)."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    parts = g.geoms if hasattr(g, "geoms") else [g]
    return unary_union([Polygon(p.exterior) for p in parts if p.geom_type == "Polygon"])


def pentagon_plan(r=R_DECK):
    """The deck outline in plan (shapely): corners at the five stations, as body_deck()'s
    RegularPolygon(r, 5, rotation=90)."""
    from shapely.geometry import Polygon
    return Polygon([(r * np.cos(np.deg2rad(90 + 72 * k)), r * np.sin(np.deg2rad(90 + 72 * k)))
                    for k in range(5)])


def carapace_feet():
    """What the five carapace sectors stand on the deck with, in the deck's MODEL frame:
    part_shell.shell_sector posed at each station, cut to its z -4..0 slab (the feet, the latch
    pads, the skirt wall outside the deck), and each pad's latch cartridge housing on its
    strike. The housing counts because the pad's pocket is not closed: the leg arch (sector x >
    72) truncates the Ø18 pad and the Ø14.3 pocket breaks out of it there (8.3 mm of its 44.6
    mm outline has no wall, up to 1.2 deep; measured 2026-10-07), so the pad's plan alone leaves
    the cartridge's footprint open. ~25 s: it builds a sector."""
    from part_shell import shell_sector
    from iface import latch_insert_housing, latch_housing_tf
    slab = shell_sector() & (Pos(0, 0, -2.0) * Box(400.0, 400.0, 4.0))
    housing = latch_insert_housing()
    return ([Pos(0, 0, 10.0) * Rot(0, 0, a) * slab for a in STATIONS] +
            [latch_strike_tf(a) * latch_housing_tf() * housing for a in STATIONS])


def deck_hole_audit(bases=None, feet=None):
    """The DECK_HOLES table against the rest of the deck, IN PLAN: [(check, ok, detail)].
    Every cutter's plan >= DECK_HOLE_EDGE inside the pentagon; >= DECK_HOLE_GAP (3.3 preferred)
    from every other entry and every kept_cuts() cut; >= DECK_HOLE_GAP from the five I1 feature
    sets (the leg port's own cuts + its seat posts); each m3_top head column (Ø6, deck top to
    +10) and every other through hole's own column >= HEAD_CLEAR from the docked coxa bases
    (`bases`: posed coxa_yaw_base solids in the deck's model frame, or None to skip, 3D); no
    hole under a carapace foot (`feet`: carapace_feet(), or None to skip: an m3_top's head disc
    >= HEAD_CLEAR from the feet's plan, any other hole off it); the strike where the tray's
    tongue is, in plan, turn and height (ok None: PENDING on the pre-D064 tray); the tray rails'
    tab holes on the tray_tab holes (tray_on_deck); the posts where params put them; the tub's
    hangers on its lid. In plan, so a blind hole and an engraving that overlap in plan count as
    touching at any depths (until v0.5 it was 3D, and the edge test read a rim solid 4 under
    the deck: it saturated at 4.00)."""
    from shapely.ops import unary_union
    out = []
    plans = {e[0]: _plan(deck_hole_cutter(e)) for e in DECK_HOLES}
    kept = {n: _plan(c) for n, c in kept_cuts()}
    # the five I1 feature sets: what leg_port_deck_features removes from a slab round the port,
    # plus the seat posts it adds (B117)
    ports = []
    for i in range(len(STATIONS)):
        tf = station_tf(i)
        slab = tf * Pos(-40.0, 0, T / 2) * Box(60.0, 60.0, T)
        ports.append(_plan(slab - leg_port_deck_features(slab, T, tf, seats=False)))
        ports.append(_plan(tf * leg_port_seats(T)))
    ports = unary_union(ports)
    pent = pentagon_plan()
    feet_p = None if feet is None else unary_union([_filled(_plan(f)) for f in feet])
    worst = {k: (np.inf, "") for k in ("edge", "cut", "table", "port", "head", "col", "feet")}
    pref, pref_kept = [], []
    for name, xy, kind, size, job in DECK_HOLES:
        p = plans[name]
        edge = pent.exterior.distance(p) if p.within(pent) else -p.difference(pent).area
        near = min((p.distance(plans[n]), n) for n in plans if n != name)
        near_k = min((p.distance(g), n) for n, g in kept.items())
        nn = min(near, near_k)
        cand = {"edge": (edge, name), "cut": (nn[0], f"{name} to {nn[1]}"),
                "table": (near[0], f"{name} to {near[1]}"), "port": (p.distance(ports), name)}
        if near[0] < DECK_HOLE_GAP_PREF:
            pref.append(f"{name} to {near[1]} {near[0]:.2f}")
        if near_k[0] < DECK_HOLE_GAP_PREF:
            pref_kept.append(f"{name} to {near_k[1]} {near_k[0]:.2f}")
        if kind == "m3_top" and bases is not None:
            head = Pos(xy[0], xy[1], T + 5.0) * Cylinder(HEAD_D / 2, 10.0)
            cand["head"] = (min(head.distance_to(b) for b in bases), name)
        elif kind != "m3_below" and bases is not None:
            # not under a docked coxa plate (the proposal's run): the hole's own column, the
            # deck top up 8 (the plate is 4). A blind m3_below keeps its skin, so it may sit
            # under a plate: the shelf posts at (+-50, 34) do, which is why they are blind
            c = deck_hole_cutter((name, xy, kind, size, job))
            c = Pos(0, 0, T - c.bounding_box().min.Z) * c
            c &= Pos(xy[0], xy[1], T + 4.0) * Box(40.0, 40.0, 8.0)
            cand["col"] = (min(c.distance_to(b) for b in bases), name)
        if feet_p is not None:
            from shapely.geometry import Point
            if kind == "m3_top":
                cand["feet"] = (Point(*xy).buffer(HEAD_D / 2, 64).distance(feet_p) - HEAD_CLEAR, name + " (head)")
            else:
                cand["feet"] = (p.distance(feet_p) - 1e-9, name)
        for k, v in cand.items():
            if v[0] < worst[k][0]:
                worst[k] = v
    out.append((f"every hole >= {DECK_HOLE_EDGE} inside the pentagon (plan)", worst["edge"][0] >= DECK_HOLE_EDGE - 1e-6,
                f"tightest {worst['edge'][1]} {worst['edge'][0]:.2f}"))
    out.append((f"every hole >= {DECK_HOLE_GAP} from every other cut (the table + the strikes, washers, "
                f"dots, arrow body_deck keeps)", worst["cut"][0] >= DECK_HOLE_GAP - 1e-6,
                f"tightest {worst['cut'][1]} {worst['cut'][0]:.2f}; hole to hole {worst['table'][1]} "
                f"{worst['table'][0]:.2f}; under {DECK_HOLE_GAP_PREF}: hole to hole "
                + (", ".join(pref) if pref else "none") + ", to a kept cut "
                + (", ".join(pref_kept) if pref_kept else "none")))
    out.append((f"every hole >= {DECK_HOLE_GAP} from the five I1 feature sets (cuts + seat posts)",
                worst["port"][0] >= DECK_HOLE_GAP - 1e-6, f"tightest {worst['port'][1]} {worst['port'][0]:.2f}"))
    if bases is not None:
        out.append((f"every m3_top head column (Ø{HEAD_D:.0f}, deck top up) >= {HEAD_CLEAR} from the docked "
                    f"coxa bases (3D)", worst["head"][0] >= HEAD_CLEAR, f"tightest {worst['head'][1]} "
                    f"{worst['head'][0]:.2f}"))
        out.append((f"no other through hole under a docked coxa base (its column, deck top up 8, >= {HEAD_CLEAR}; "
                    f"the blind shelf posts exempt, 3D)", worst["col"][0] >= HEAD_CLEAR,
                    f"tightest {worst['col'][1]} {worst['col'][0]:.2f}"))
    if feet is not None:
        out.append((f"no hole under a carapace foot (an m3_top's Ø{HEAD_D:.0f} head >= {HEAD_CLEAR} off the feet, "
                    f"the rest off them; part_shell's sectors at the five stations)", worst["feet"][0] > 0,
                    f"tightest {worst['feet'][1]}: {worst['feet'][0] + (HEAD_CLEAR if '(head)' in worst['feet'][1] else 0):.2f} "
                    f"to the feet's plan"))
    # the strike sits under the tongue: the tray's latch frame (part_avionics.tongue_latch_tf,
    # tray-local) posed by params tray_xy / tray_rot_deg, in plan and turn ...
    from part_avionics import tongue_latch_tf
    tray = Pos(*_AT["tray_xy"], 0) * Rot(0, 0, _AT["tray_rot_deg"]) * tongue_latch_tf()
    st = next(e for e in DECK_HOLES if e[2] == "strike")
    p, xd = tray.position, (tray * Pos(1, 0, 0)).position - tray.position
    d_xy = float(np.hypot(p.X - st[1][0], p.Y - st[1][1]))
    d_ang = abs((np.rad2deg(np.arctan2(xd.Y, xd.X)) - st[3][0] + 180) % 360 - 180)
    out.append(("the tray strike under the tongue's latch (params tray pose x part_avionics.tongue_latch_tf)",
                d_xy < 0.01 and d_ang < 0.01, f"off {d_xy:.3f} mm, {d_ang:.3f} deg"))
    # ... and in height: the seating plane ON the deck top (the convention above). ok None =
    # PENDING: the pre-D064 tray, part_avionics' to rebuild this round (it is not a deck fault)
    import part_avionics as AV
    tr = tray_on_deck()
    sz = tr["seat_z"]
    ok = (None if not tr["d064"] and not hasattr(AV, "TRAY_Z") else sz is not None and abs(sz) < 0.01)
    out.append(("the tongue's latch seating plane on the deck top (the strike's z 0)", ok,
                (f"{sz:+.2f} over it" if sz is not None else "unknown") + f": {tr['seat_src']}"
                + ("; the tongue needs a boss down to the deck (part_avionics)" if ok is not True else "")))
    # the tray rails' tab holes on the table's tray_tab holes (a ray test: each rail bore's axis
    # through a deck hole of the same Ø, TAB_HOLE_TOL apart at most, and one bore per hole)
    tabs = tr["tabs"]
    table_tabs = [e[1] for e in DECK_HOLES if e[0].startswith("tray_tab")]
    miss = [min(np.hypot(tx - hx, ty - hy) for hx, hy in table_tabs) for tx, ty in tabs] or [np.inf]
    hit = [min(np.hypot(tx - hx, ty - hy) for tx, ty in tabs) for hx, hy in table_tabs] if tabs else [np.inf]
    out.append((f"the tray rails' tab holes on the {len(table_tabs)} tray_tab holes (params tray_xy "
                f"{tuple(_AT['tray_xy'])}, rot {_AT['tray_rot_deg']:.0f})",
                len(tabs) == len(table_tabs) == 6 and max(miss + hit) <= TAB_HOLE_TOL,
                f"{tr['tabs_src']}: " + ", ".join(f"({x:+.2f}, {y:+.2f})" for x, y in sorted(tabs))
                + f"; worst {max(miss + hit):.3f}"))
    posts = [e[1] for e in DECK_HOLES if e[0].startswith("shelf_post")]
    out.append(("the shelf posts are params hub_shelf.posts", posts == [tuple(map(float, q)) for q in _HS["posts"]],
                f"{posts}"))
    # the tub's six hangers land on its lid where params put the tub (iface.bay_tub_extent: x
    # pinned by battery_sled.bay_door_x, the nose by bay_l): each Ø8 boss wholly inside the tub's
    # plan, so re-pinning the tub or shortening it (pick 4) cannot strand one off the lid unseen
    tub = bay_tub_extent()
    hang = [e for e in DECK_HOLES if e[0].startswith(("tray_tab_tub_", "tub_hanger_"))]
    m, mn = min((min(x - tub["x"][0], tub["x"][1] - x, y - tub["y"][0], tub["y"][1] - y)
                 - TUB_BOSS_D / 2, n) for n, (x, y), *_ in hang)
    out.append((f"the {len(hang)} tub hangers' Ø{TUB_BOSS_D:.0f} bosses on the tub's lid (iface.bay_tub_extent)",
                len(hang) == 6 and m >= 0, f"tightest {mn} {m:.2f} inside the plan (tub x {tub['x'][0]:.1f}.."
                f"{tub['x'][1]:.1f}, y {tub['y'][0]:.1f}..{tub['y'][1]:.1f})"))
    return out


def tray_on_deck():
    """What the deck's tray strike and tray-tab holes must meet, read off part_avionics and
    posed by params tray_xy / tray_rot_deg: {'tabs': [(x, y)] body, 'tabs_src', 'd064': is the
    rail the D064 one, 'seat_z': the tongue's latch seating plane over the deck top (None:
    not knowable), 'seat_src'}.
    Tabs: the vertical Ø screw_m3_clear bores of part_avionics.tray_rail(), posed by
    part_avionics.RAIL_POSES (the two rails, tray-local) if it names them, else the way its
    __main__ poses them (the +x block at RAIL_POSE_X turned 90 about z, the -x block its
    half-turn copy). Bores OUTBOARD of the block axis are the pre-D064 rail (pick 7 and B51
    turn the tabs inboard, part_avionics' job this round): then the holes are the proposal's,
    TAB_HOLE_INBOARD off the block axis on the same 20 mm pitch, and the source says so.
    Seat: part_avionics.TRAY_Z (the plate underside over the deck top) + tongue_latch_tf()'s z
    if it names TRAY_Z; on the pre-D064 tray TRAY_PRE_D064_Z (it has no tray_lift yet); a D064
    tray that does not name it leaves the seat unknowable (None), which the audit fails."""
    import part_avionics as AV
    r = PR["screw_m3_clear"] / 2
    rail = AV.tray_rail()
    loc = sorted({(round(f.axis_of_rotation.position.X, 4), round(f.axis_of_rotation.position.Y, 4))
                  for f in rail.faces().filter_by(GeomType.CYLINDER)
                  if abs(f.radius - r) < 1e-4 and abs(abs(f.axis_of_rotation.direction.Z) - 1) < 1e-9})
    rails = getattr(AV, "RAIL_POSES", None)
    pose_src = "part_avionics.RAIL_POSES" if rails is not None else "part_avionics.__main__'s rail pose"
    if rails is None:
        rails = [Pos(AV.RAIL_POSE_X, 0, 0) * Rot(0, 0, 90), Rot(0, 0, 180) * Pos(AV.RAIL_POSE_X, 0, 0) * Rot(0, 0, 90)]
    tray_local = [(rp * Pos(x, y, 0)).position for rp in rails for x, y in loc]
    tray = Pos(*_AT["tray_xy"], 0) * Rot(0, 0, _AT["tray_rot_deg"])
    d064 = bool(loc) and all(abs(q.X) < AV.RAIL_POSE_X - 1e-6 for q in tray_local)
    if d064:
        pts = tray_local
        src = f"part_avionics.tray_rail()'s {len(loc)} bores x {len(rails)} rails ({pose_src}), inboard of the block axis"
    else:
        # the same pitch along the block (a bore's rail-local x is its tray-local y, turned 90)
        ys = sorted({round(x, 4) for x, _ in loc}) or [-20.0, 0.0, 20.0]
        pts = [Vector(sx * (AV.RAIL_POSE_X - TAB_HOLE_INBOARD), y, 0) for sx in (1, -1) for y in ys]
        out_x = max((abs(q.X) for q in tray_local), default=float("nan"))
        src = (f"the worktree's tray_rail() is pre-D064 (bores at x +-{out_x:.2f}, outboard of the block "
               f"axis +-{AV.RAIL_POSE_X:.2f}): the proposal's {TAB_HOLE_INBOARD} inboard of it instead")
    body = [(tray * Pos(q.X, q.Y, 0)).position for q in pts]
    tz = AV.tongue_latch_tf().position.Z
    if hasattr(AV, "TRAY_Z"):
        seat, seat_src = AV.TRAY_Z + tz, f"part_avionics.TRAY_Z {AV.TRAY_Z:.2f} + tongue_latch_tf z {tz:.2f}"
    elif not d064:
        seat = TRAY_PRE_D064_Z + tz
        seat_src = (f"pre-D064 tray: the plate underside {TRAY_PRE_D064_Z} over the deck (its rails' channel pose; "
                    f"{TRAY_PRE_D064_Z + _AT['tray_lift']:.1f} once tray_lift is built) + tongue_latch_tf z {tz:.2f}")
    else:
        seat, seat_src = None, "part_avionics names no TRAY_Z (the plate underside over the deck top)"
    return {"tabs": [(float(q.X), float(q.Y)) for q in body], "tabs_src": src, "d064": d064,
            "seat_z": seat, "seat_src": seat_src}


def deck_holes_as_cut(deck):
    """Every DECK_HOLES entry measured on a deck SOLID (model frame), and the retired v0.4 cuts:
    [(name, ok, detail)]. A probe 1 bigger than the hole (z 0.001..T-0.001) minus the deck is
    the hole as cut: its plan size (Ø, or the slot's w x h) to 0.01 and its open area to 1 %;
    a through hole runs the whole thickness and a Ø1 pin down its axis meets 0; a blind hole
    opens at the underside, reaches its depth +-0.05 and leaves T - depth of skin (a pin above it
    meets solid deck). The strike is the latch check's (part_panel.latch_engagement). The
    retired cuts must be solid deck again outside the table's own cutters ((0, -40) lies in the
    strike's bore)."""
    vol = lambda s: 0.0 if s is None else s.volume
    pin = lambda x, y, z0, z1, r=0.5: Pos(x, y, (z0 + z1) / 2) * Cylinder(r, z1 - z0)
    out = []
    for name, (x, y), kind, size, _ in DECK_HOLES:
        at = f"({x:+6.1f}, {y:+6.1f})"
        if kind == "strike":
            out.append((name, True, f"{at} strike, frame {size[0]:.0f} deg: the latch check (latch_engagement)"))
            continue
        if kind == "slot":
            w, h, r = size
            probe = Pos(x, y, T / 2) * Box(w + 2.0, h + 2.0, T - 0.002)
            want = (w, h, w * h - (4 - np.pi) * r * r)
        else:
            probe = Pos(x, y, T / 2) * Cylinder(size[0] / 2 + 1.0, T - 0.002)
            want = (size[0], size[0], np.pi * size[0] ** 2 / 4)
        void = probe - deck
        if void is None or void.volume < 1e-6:
            out.append((name, False, f"{at} {kind}: NOT CUT"))
            continue
        vb = void.bounding_box()
        top, bot = vb.max.Z, vb.min.Z
        area = void.volume / (top - bot)
        size_ok = abs(vb.size.X - want[0]) < 0.01 and abs(vb.size.Y - want[1]) < 0.01 and \
            abs(area - want[2]) < 0.01 * want[2]
        if kind == "m3_below":
            skin = T - top                        # the void starts at the underside (z 0.001)
            if skin > 0.03:
                above = pin(x, y, top + 0.01, T - 0.01)
                skin_v, full = vol(above & deck), above.volume
            else:                                 # drilled through: no skin to probe
                skin_v, full = 0.0, 1.0
            ok = size_ok and bot < 0.01 and abs(top - size[1]) <= 0.05 and abs(skin - (T - size[1])) <= 0.05 \
                and skin_v > 0.99 * full
            out.append((name, ok, f"{at} blind from below Ø{vb.size.X:.3f}: {top:.3f} deep from the underside, "
                        f"skin {skin:.3f} (a pin above it meets {skin_v:.3f} mm^3 of deck)"))
        else:
            thru = vol(pin(x, y, -1.0, T + 1.0) & deck)
            ok = size_ok and bot < 0.01 and top > T - 0.01 and thru < 1e-6
            shape = f"{vb.size.X:.3f} x {vb.size.Y:.3f} slot" if kind == "slot" else f"Ø{vb.size.X:.3f}"
            out.append((name, ok, f"{at} {kind:6s} {shape} through ({area:.2f} mm^2 open, a Ø1 pin down its axis "
                        f"meets {thru:.4f} mm^3)"))
    table_cut = Compound(children=[deck_hole_cutter(e) for e in DECK_HOLES])
    ret = [(name, vol((probe - table_cut) - deck)) for name, probe in DECK_RETIRED]
    out.append(("retired", all(v <= 1e-3 for _, v in ret), "v0.4 cutters outside the table's, minus the deck "
                "(mm^3 of hole left): " + ", ".join(f"{n} {v:.4f}" for n, v in ret)))
    return out


if __name__ == "__main__":
    import time
    t00 = time.time()
    # ---- layout audit FIRST: every I1 feature must sit ON the pentagon ----
    lp = IF["leg_port"]
    feats = ([("seat", xy) for xy in lp["seat_xy"]] +
             [("thumbscrew", xy) for xy in lp["thumbscrew_xy"]] +
             [("hook-slot", (lp["hook_slot_x"], 0))])
    audit_fail = 0
    for ang in STATIONS[:1]:                       # symmetric — one station
        for name, (lx, ly) in feats:
            px, py = _station_xy(ang, lx, ly)
            ok = pentagon_contains(px, py)
            if not ok:
                audit_fail += 1
            print(f"  I1 {name} @ leg({lx},{ly}) -> deck r "
                  f"{np.hypot(px, py):.1f}: {'ON-DECK' if ok else 'OFF-DECK!'}")
    # v0.4 shell-interface features: the latch strike (B87: its underside recess
    # reaches r 6.1 on two quarter arcs, so the cutter itself is tested against the
    # pentagon inset 1 mm), washer (Ø8)
    inset = Pos(0, 0, -5) * extrude(RegularPolygon(R_DECK - 1.0 / np.cos(np.deg2rad(36)), 5,
                                                   major_radius=True, rotation=90), T + 10)
    v_out = (latch_strike_tf(STATIONS[0]) * latch_strike(T)) - inset
    v_out = 0.0 if v_out is None else v_out.volume
    audit_fail += v_out > 0.01
    print(f"  I3 latch strike @ az +{SHELL_LATCH_AZ:.1f} r {SHELL_LATCH_R}: {v_out:.3f} mm^3 "
          f"within 1 mm of the deck edge ({'ON-DECK (cutter-checked)' if v_out <= 0.01 else 'OFF-DECK!'})")
    for name, az_off, r, feat_r in (("washer recess", -25.5, 76.0, 4.0),):
        a = np.deg2rad(STATIONS[0] + az_off)
        px, py = r * np.cos(a), r * np.sin(a)
        # containment must hold for the feature EDGE, not just the center
        ok = pentagon_contains(px, py, margin=feat_r + 1.0)
        if not ok:
            audit_fail += 1
        print(f"  I3 {name} @ az {az_off:+.1f} r {r}: "
              f"{'ON-DECK (edge-checked)' if ok else 'OFF-DECK!'}")
    # document the v0.2 bug for the record:
    px, py = _station_xy(STATIONS[0], -20, 17)
    v02_verdict = ("on" if pentagon_contains(px, py, 0)
                   else "OFF the pentagon — the v0.2 bug, now retired")
    print(f"  (v0.2 mid-pair hole @ leg(-20,17) -> r {np.hypot(px, py):.1f}: "
          f"{v02_verdict})")
    assert audit_fail == 0, "I1 feature off the deck — fix params before export"

    d = body_deck()
    export(d, "body_deck")
    from part_coxa import coxa_yaw_base
    # pose: deck top at leg z=-4 -> deck body z in [-10,-4]; part modeled 0..6
    d_posed = Pos(0, 0, -10) * d
    b0 = Rot(0, 0, STATIONS[0]) * Pos(R_STATION, 0, 0) * coxa_yaw_base()
    b1 = Rot(0, 0, STATIONS[1]) * Pos(R_STATION, 0, 0) * coxa_yaw_base()
    bad = 0
    vol = lambda s: 0.0 if s is None else s.volume
    for name, pair in [("adjacent coxa bases", b0 & b1),
                       ("deck x coxa base (docked: seats in sockets, lip in slot)",
                        d_posed & b0)]:
        v = 0.0 if pair is None else pair.volume
        bad += v >= 1
        print(f"{name} intersection: {v:.2f} mm^3 ({'OK' if v < 1 else 'CLASH'})")
    # I3 (B87): the sector's latch cartridge on this deck's strike (station 0) and, since v0.5,
    # the tray's cartridge on its strike at (0, -43): the same rules, the same cartridge
    from part_panel import latch_engagement, latch_verdict, latch_report
    st = next(e for e in DECK_HOLES if e[2] == "strike")
    for tag, tf in (("the deck strike (sector, station 0)", latch_strike_tf(STATIONS[0])),
                    (f"the tray strike {st[1]} (frame turned {st[3][0]:.0f})", strike_tf(st))):
        m = latch_engagement(d, tf)
        bad += len(latch_verdict(m))
        print(f"I3 latch on {tag}: {latch_report(m)} "
              f"({'OK' if not latch_verdict(m) else 'FAIL: ' + '; '.join(latch_verdict(m))})")

    # ---- v0.5: the hole table AS CUT, measured on the real deck (model frame) ----
    print(f"DECK_HOLES as cut ({len(DECK_HOLES)} entries; measured on body_deck()):")
    for name, ok, detail in deck_holes_as_cut(d):
        bad += not ok
        print(f"  {name:20s} {detail} ({'OK' if ok else 'FAIL'})")

    # ---- the table's audit (plan) + the carapace feet + the docked bases ----
    base = coxa_yaw_base()
    bases_model = [Pos(0, 0, T + 4.0) * station_tf(i) * base for i in range(len(STATIONS))]
    feet = carapace_feet()
    print("DECK_HOLES audit (plan; retired: the four strap slots, the (52, -6) grommet + its two anchors, "
          "grid (0, -40)):")
    for name, ok, detail in deck_hole_audit(bases_model, feet):
        bad += ok is False                        # None = PENDING (another part's to close)
        print(f"  {name}: {'OK' if ok else 'PENDING' if ok is None else 'FAIL'}" + (f" ({detail})" if detail else ""))
    # the tray's latch where its tongue really is: the cartridge on this strike with the seating
    # plane at the tongue's height (measured only while that is not the deck top)
    tr = tray_on_deck()
    if tr["seat_z"] is not None and abs(tr["seat_z"]) >= 0.01:
        heights = sorted({round(tr["seat_z"], 2), round(tr["seat_z"] + (0.0 if tr["d064"] else _AT["tray_lift"]), 2)})
        for h in heights:
            m = latch_engagement(d, strike_tf(st) * Pos(0, 0, h))
            print(f"  (the tray's cartridge with its seating plane {h:.1f} over the deck top: "
                  + ("; ".join(latch_verdict(m)) or "OK") + f"; first contact {m['contact_deg']} deg, LOCKED "
                  f"{m['bite']:.2f} mm^3, land over the lugs {100 * m['land']:.0f} %)")

    # ---- the under-deck keep-outs (iface): the deck, the shelf posts ----
    from iface import body_keepouts, leg_port_hook_envelope, DECK_BOT_Z
    ko = body_keepouts()
    for tag, s in (("hook envelope ('any') at station 0", ko["hooks"][0]),
                   ("drop keep-out at station 0", ko["drops"][0])):
        sb = s.bounding_box()
        print(f"  {tag}: body x {sb.min.X:.2f}..{sb.max.X:.2f}, y {sb.min.Y:.2f}..{sb.max.Y:.2f}, "
              f"z {sb.min.Z:.2f}..{sb.max.Z:.2f}")
    lb = leg_port_hook_envelope("any").bounding_box()
    v_hd = sum(vol(d_posed & h) for h in ko["hooks"])
    ok = v_hd < 1e-3 and abs(lb.max.Z - DECK_BOT_Z) < 1e-6
    bad += not ok
    print(f"  posed deck x the five hook envelopes: {v_hd:.4f} mm^3, envelope top z {lb.max.Z:.2f} "
          f"(the sweep lives below the deck: {'OK' if ok else 'FAIL'})")
    v_dd = sum(vol(d_posed & k) for k in ko["drops"])
    bad += v_dd >= 1e-3
    print(f"  posed deck x the five drop keep-outs: {v_dd:.4f} mm^3 ({'OK' if v_dd < 1e-3 else 'FAIL'})")
    hs = IF["hub_shelf"]
    cols = [Pos(px, py, (DECK_BOT_Z + hs["plate_top_z"]) / 2) * Cylinder(hs["post_d"] / 2, DECK_BOT_Z - hs["plate_top_z"])
            for px, py in hs["posts"]]
    bases_body = [station_tf(i) * base for i in range(len(STATIONS))]
    # the posts' margins (BODY_LAYOUT correction 10, params hub_shelf.posts): 0.885 to the docked
    # bases; to the 'any' hook envelope 0.39 before iface.HOOK_ROLL_PAD, 0.339 with it; 0.59 to
    # the planar 'path' one
    for tag, group in (("docked coxa bases", bases_body), ("hook envelopes ('any')", ko["hooks"]),
                       ("hook envelopes ('path')", body_keepouts(mode="path")["hooks"]),
                       ("drops", ko["drops"]), ("drops + 5", body_keepouts(5.0)["drops"])):
        g = [min(c.distance_to(k) for k in group) for c in cols]
        v = sum(vol(c & k) for c in cols for k in group)
        bad += v > 1e-3
        print(f"  shelf posts (Ø{hs['post_d']:.0f}, deck bottom to the plate) x {tag}: {v:.4f} mm^3, closest "
              + ", ".join(f"{tuple(int(q) for q in p)} {x:.3f}" for p, x in zip(hs["posts"], g))
              + f" ({'OK' if v <= 1e-3 else 'CLASH'})")
    # why the posts are blind from below (reported): an M3 head on the deck top at each post
    heads = [(p, sum(vol((Pos(p[0], p[1], -4.0 + 2.0) * Cylinder(HEAD_D / 2, 4.0)) & b) for b in bases_body))
             for p in hs["posts"]]
    print("  (an M3 head (Ø6 x 4) on the deck top at each post would meet the docked bases: "
          + ", ".join(f"{tuple(int(q) for q in p)} {v:.2f}" for p, v in heads) + " mm^3: why post_screw is below)")
    bb = d.bounding_box()
    bed_x, bed_y = PR["bed_mm"][:2]
    fits = bb.size.X <= bed_x and bb.size.Y <= bed_y
    bad += not fits
    print(f"deck bbox: {bb.size.X:.0f} x {bb.size.Y:.0f} mm, {d.volume / 1000:.1f} cm^3 "
          f"({f'FITS bed {bed_x:.0f}x{bed_y:.0f}' if fits else 'TOO BIG'})")
    print(f"part_deck: {'CLEAN' if not bad else f'{bad} FAILURES'} ({time.time() - t00:.0f} s)")
    raise SystemExit(1 if bad else 0)
