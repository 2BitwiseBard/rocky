"""Hub shelf (I4/I5 body layout, option A): the harness hub UNDER the deck — picks 1, 9, 14
(2026-10-07), B92, B108, B128, B133, B134. The module keeps its name (run_all_checks and
check_printability call it); the part is `hub_shelf`. It retires `busboard_bracket`, the 46 x 36
star-board bracket on the deck grid (0, 20) / (0, 40): the leg drops meet the body under the deck
(BODY_LAYOUT_PROPOSAL fact 1), so the hub hangs there and the deck top carries the tray alone.

One printed part, printed as it is built (the plate's underside on the bed, the posts up):
  plate   params hub_shelf: the measured packing's outline (x / y: the union of the five boards'
          footprints + their plug / lead halos, PREP_REPORT_2 s3 'offtray' = case iii at bay_w 50,
          G3 b4b_pack + g3_verify), plate_t thick, its top at plate_top_z; Ø14 pads round the
          posts (a 10-wide arm only if a post were off the outline: none is); notches narrower
          than 6 closed with r 3 fillets (the packing left a 4.1 x 12 notch and a 2 mm slit)
  posts   three Ø8 at hub_shelf.posts from the plate top to the deck bottom, Ø3.4 bore, a
          Ø6.4 x plate_t counterbore in the underside: an M3 socket head from BELOW (post_screw
          below), flush, into the deck's blind Ø2.8 x 5 thread-forming hole (part_deck.DECK_HOLES
          shelf_post_*). From the top a head would sit under the docked coxa plate (params)
  top     the 12 V node (pick 9: a 30.5 x 30.5 FPV PDB, 36 x 36) on four Ø6.4 standoffs, the 40 x 30
          star on four Ø6.4 posts (34 x 24), standoff_h tall, blind Ø2.8 x 6 M3 thread-forming. The
          node's parts and its flat-soldered leads are taken as <= 6.0 over its board (NODE_LEADS,
          VERIFY with the PDB bought): leg 1's loom passes 1.20 over that, the tray cable 0.25
  under   pick 14: the bus adapter face-down (driven over the Pi's UART) on four bought Ø5 x 2.5
          round spacers, M2.5 x 8 from below into Ø2.05 thread-forming bores through the plate (a
          printed 2.5 standoff under the plate would float the whole plate 2.5 off the bed); the
          buck and the UBEC face-down flat on the underside, one / two 3.6 zip ties each through
          slots (they have no holes; part_avionics' tie). The buck's tie crosses the top face
          under the star in a groove; the UBEC's wrap the plate's north edge
  anchors two zip-anchor slot pairs at the plate's south corners: the face-down boards' leads (the
          adapter's west plugs, the buck's east leads) come round the plate's edge there to the node
          and the star. The looms (correction 12: they leave the shelf through its south face) run
          17-22 over the plate, at the star's lead band: the posts are the leg 1 / 4 looms' tie
          points, the deck's (12, 50) zip anchor the leg 0 loom's; the leg 2 / 3 looms are tied in
          their lanes (nothing on the shelf stands at their height: see the report)

The boards ARE the measured case iii (params hub_shelf.boards, asserted below): the adapter
(-20.2, 28.8) rot 0, the buck (35.8, 26.2) rot 90, the UBEC (-1.8, 55.8) rot 0, all face-down;
the node (-20.8, 36.2) and the star (25.2, 28.2) rot 0, up. Envelopes: part_avionics.BOARDS for
the three small boards (imported); the node / star stacks and every halo are G3's ESTIMATEs.

Checks (run this file, exit non-zero on a failure): the poses are params'; the shelf + boards +
hardware (shelf_model: one fused solid) against iface.body_keepouts (hooks 'any', drops +5), the
posed deck, the post axes through the DECK_HOLES table (ray test: the table, not today's deck
solid), the keel tub as part_bay builds it (tub + lid + door), the carapace, the docked coxa
bases, the stand as part_stand builds it (stand_keepouts: the relieved crown plate, the cradle
floor and walls, the pin), the lanes, the leg 2 / 3 lanes + the leg 0 / 1 / 4 looms (Ø7), the 14
AWG feed (B133, Ø5: from inside the nose compartment out through part_bay's exit in the NE
chamfer), the tray's (0, 50) cable round the plate's north-west edge (Ø9.42), the latch driver
columns; the boards pairwise; one solid, the bed. Reported, not gated: which adapter screws a
driver reaches from below on the stand (B134), and how deep a straight Dupont on the adapter's H2
would hang.

shelf_model() / shelf_parts() are the posed shelf for the other modules (part_stand, part_bay):
the real solid in place of params hub_shelf's box.
"""
import numpy as np
from build123d import *
from shapely.geometry import box as sbox, Point
from shapely.ops import unary_union
from common import params, export
from iface import (IF, FIT, STATIONS, station_tf, DECK_BOT_Z, CABLE_CUTOUT_X, DROP_HALF,
                   DROP_DEPTH, body_keepouts, bay_tub_extent, SHELL_LATCH_AZ, SHELL_LATCH_R)
from part_avionics import BOARDS, TIE_T

P = params()
PR = P["print"]
HS = IF["hub_shelf"]
BS = IF["battery_sled"]
STAND = IF["stand"]

PLATE_T = HS["plate_t"]                       # 3.0
PLATE_TOP = HS["plate_top_z"]                 # -36.3
PLATE_BOT = PLATE_TOP - PLATE_T               # -39.3
POSTS = [tuple(map(float, p)) for p in HS["posts"]]
POST_D = HS["post_d"]                         # 8
POST_L = DECK_BOT_Z - PLATE_TOP               # 26.3: plate top -> deck bottom
PAD_D, ARM_W = 14.0, 10.0                     # G3's pads / arms (g3_verify), plate-thick
NOTCH_R = 3.0                                 # closing radius: notches < 6 wide filled
STANDOFF_H = HS["standoff_h"]                 # 5: the node and star standoffs (as packed)

# ---- the M3 that hangs the shelf (post_screw below). The deck's hole is DECK_HOLES' m3_below
# (Ø screw_m3_tap x 5 blind, 1.0 skin). The head is FLUSH in a plate_t counterbore (the plate's
# underside is lead / plug room of three boards: (-50, 34) the adapter's west plug halo, (50, 34)
# the buck's east leads, (26, 64) the UBEC's east leads), so it bears on the post's own base
SCREW_HEAD_D, SCREW_HEAD_H = 5.5, 3.0         # ISO 4762 M3 socket head
CB_D, CB_DEPTH = SCREW_HEAD_D + 0.9, PLATE_T  # Ø6.4 x 3.0: head flush with the underside
POST_BORE = PR["screw_m3_clear"]              # 3.4
M3_LENGTHS = (25.0, 30.0, 35.0, 40.0)         # stocked ISO 4762 lengths
DECK_HOLE_DEPTH = 5.0                         # part_deck's shelf_post_* (asserted against the table)
SCREW_END_GAP = 0.5                           # the tip stops this far off the blind hole's bottom

# ---- board-side hardware
TAP = PR["screw_m3_tap"]                      # 2.8: M3 thread-forming (star, node)
TAP_DEPTH = STANDOFF_H + 1.0                  # 6: into the plate 1.0, 2.0 of plate left under it
STAR_POST_D, NODE_POST_D = 6.4, 6.4           # 1.8 of wall round the Ø2.8 tap: the old bracket's Ø5.6
                                              # left 1.4 and a Ø6 exactly 1.6, both under check_print-
                                              # ability's 1.6 (4 perimeters). The star's holes sit 3 in
                                              # from its edges, the PDB's 2.75: the boss stands 0.2 /
                                              # 0.45 past the board's edge, under it
M25_TAP = 2.05                                # M2.5 thread-forming, as part_avionics' Pi standoffs
ADAPTER_HOLES = (37.0, 28.0)                  # Waveshare Bus Servo Adapter (A): Ø2.5 on 37 x 28
                                              # (part_avionics' BOARDS comment; not in BOARDS: B108).
                                              # To import from BOARDS once it carries holes / cap_l
SPACER_D = 5.0                                # bought round nylon spacer, ID >= 2.6, 2.5 long
TIE_W = 3.6                                   # part_avionics' 3.6 mm zip tie (TIE_T thick)
TIE_SLOT = (TIE_W + 2 * FIT, TIE_T + 2 * FIT)  # 4.2 along the band's width x 1.8
GROOVE_D = TIE_T + 0.2                        # the buck's tie sunk under the star's header tails
BUCK_CAP_L = 10.0                             # its 1000 uF can (Ø10 x 16) lying beside the D24V50F5
                                              # (ESTIMATE, the part bought): 17.8 + 10 = 27.8. To
                                              # import from BOARDS once it carries holes / cap_l
# the adapter's H2 (UART, pick 14): where it sits on the board is not in the repo (no STEP; the
# prep's 30.42 'headroom' is the Pi's GPIO header on the tray, q_usb/s4_other.py). Face-down, a
# STRAIGHT 2.54 Dupont on a vertical header hangs header base + housing under the component face
H2_BASE, DUPONT_L = 2.5, 14.0                 # ESTIMATEs: a 2.54 header's base, a 1 x 3 female housing

# ---- the five boards (rot 0: w along x). stack = from the plate face away from it (UP: plate top
# up, DOWN: plate underside down); halo = plug / lead room (x-, x+, y-, y+), an ESTIMATE each (G3
# b4_pack's COMP). The three small boards' numbers ARE part_avionics.BOARDS'.
STAR_STACK = STANDOFF_H + 1.6 + 9.8 + 6.0     # 22.4: posts + perfboard + mated XH (eXH.pdf) + bend
STAR_TAILS = 3.4                              # its header tails under the board (eXH.pdf)
NODE_LEADS = 6.0                              # VERIFY: the PDB's parts AND its flat-soldered 14/16 AWG
                                              # leads stand <= 6.0 over its 1.6 board (no BEC, pick 9)
NODE_STACK = STANDOFF_H + 1.6 + NODE_LEADS    # 12.6
_BA, _BU, _UB = BOARDS["bus_adapter"], BOARDS["buck_5v"], BOARDS["ubec_6v"]
SHELF_BOARDS = {
    "bus_adapter": dict(w=_BA["w"], l=_BA["l"], stack=_BA["lift"] + _BA["up"] + _BA["tie"],
                        halo=(15.0, 15.0, 0.0, 0.0)),   # plugs off both 33 edges (which is which: VERIFY)
    "buck_5v": dict(w=_BU["w"] + BUCK_CAP_L, l=_BU["l"], stack=_BU["up"] + _BU["tie"],
                    halo=(0.0, 0.0, 8.0, 8.0)),         # leads off both 27.8 edges
    "ubec_6v": dict(w=_UB["w"], l=_UB["l"], stack=_UB["up"] + _UB["tie"],
                    halo=(8.0, 8.0, 0.0, 0.0)),         # leads off both 17 ends
    "node_12v": dict(w=36.0, l=36.0, stack=NODE_STACK, halo=(6.0, 6.0, 6.0, 6.0)),
    "star": dict(w=40.0, l=30.0, stack=STAR_STACK, halo=(0.0, 0.0, 0.0, 0.0)),
}
POSE = {k: (tuple(map(float, v["xy"])), float(v["rot_deg"]), v["face"]) for k, v in HS["boards"].items()}
# the measured arrangement (PREP_REPORT_2 s3, G3 logs/pack_50_h3_cradle_f-54.4_c3.log): if params
# move a board, the packing behind every margin below has to be re-run, not just this file
CASE_III = {"bus_adapter": ((-20.2, 28.8), 0.0, "down"), "buck_5v": ((35.8, 26.2), 90.0, "down"),
            "ubec_6v": ((-1.8, 55.8), 0.0, "down"), "node_12v": ((-20.8, 36.2), 0.0, "up"),
            "star": ((25.2, 28.2), 0.0, "up")}


def _rot2(lx, ly, rot):
    a = np.deg2rad(rot)
    return lx * np.cos(a) - ly * np.sin(a), lx * np.sin(a) + ly * np.cos(a)


def board_plan(name, halo=True):
    """(x0, x1, y0, y1) of a board's footprint (+ its halo) in the body frame."""
    b = SHELF_BOARDS[name]
    (cx, cy), rot, _ = POSE[name]
    a, bb, c, d = b["halo"] if halo else (0.0, 0.0, 0.0, 0.0)
    x0, x1, y0, y1 = -b["w"] / 2 - a, b["w"] / 2 + bb, -b["l"] / 2 - c, b["l"] / 2 + d
    pts = [_rot2(x, y, rot) for x in (x0, x1) for y in (y0, y1)]
    xs, ys = [cx + p[0] for p in pts], [cy + p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


def board_z(name):
    """(z0, z1): UP boards stand on the plate top, DOWN boards hang from its underside."""
    s = SHELF_BOARDS[name]["stack"]
    return (PLATE_TOP, PLATE_TOP + s) if POSE[name][2] == "up" else (PLATE_BOT - s, PLATE_BOT)


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def board_box(name, halo=True):
    return _box(*board_plan(name, halo), *board_z(name))


def board_point(name, lx, ly):
    """A board-local point (board centre, its own +x) in the body frame."""
    (cx, cy), rot, _ = POSE[name]
    dx, dy = _rot2(lx, ly, rot)
    return cx + dx, cy + dy


def mounts(name):
    """The board's hole axes, body frame."""
    if name == "star":
        hx, hy = HS["star_posts"][0] / 2, HS["star_posts"][1] / 2
    elif name == "node_12v":
        hx = hy = HS["node_pattern"] / 2
    elif name == "bus_adapter":
        hx, hy = ADAPTER_HOLES[0] / 2, ADAPTER_HOLES[1] / 2
    else:
        return []
    return [board_point(name, sx * hx, sy * hy) for sx in (-1, 1) for sy in (-1, 1)]


def post_screw():
    """(length, engagement in the deck): the shortest stocked M3 that engages >= 3 mm and stops
    SCREW_END_GAP short of the blind hole's bottom. Its head bears CB_DEPTH up the plate."""
    shank = (PLATE_T - CB_DEPTH) + POST_L
    for L in M3_LENGTHS:
        e = L - shank
        if 3.0 <= e <= DECK_HOLE_DEPTH - SCREW_END_GAP:
            return L, e
    raise RuntimeError(f"no stocked M3 fits a {shank:.1f} shank into a {DECK_HOLE_DEPTH} hole")


# ---- ties and anchors (body xy of each slot centre, and which way the band runs)
def tie_slots():
    """[(name, (x, y), band_axis)] — every slot through the plate. band_axis 'x': the tie's band
    runs along x through it (the slot is TIE_SLOT[1] along x, TIE_SLOT[0] along y)."""
    out = []
    # the buck (rot 90: x 25.65..45.95, y 12.3..40.1): ONE band along x over the D24V50F5 (the can
    # end, y 30.1..40.1, is held by its leads + a dab of glue: VERIFY with the part bought). y 23 sits
    # between the star's posts (y 16.2 / 40.2: >= 1.5 of plate round the groove) and post (50, 34)
    # (>= 1.2 to its Ø8); the slots hug the board's two 20.3 edges
    x0, x1, y0, y1 = board_plan("buck_5v", halo=False)
    yt = 23.0
    out += [("buck tie W", (x0 - TIE_SLOT[1] / 2, yt), "x"), ("buck tie E", (x1 + TIE_SLOT[1] / 2, yt), "x")]
    # the UBEC (x -23.3..19.7, y 47.3..64.3, its north edge the plate's): two bands along y, round
    # the plate's north edge, up through a slot hugging its south edge. x 9: clear of the node's
    # lead halo (x <= 3.2) and the star; x -12: under the node's board (3.8 to its underside) and
    # 1.45 off its standoff (-5.55, 51.45)
    x0, x1, y0, y1 = board_plan("ubec_6v", halo=False)
    for xt in (-12.0, 9.0):
        out.append((f"UBEC tie x {xt:+.0f}", (xt, y0 - TIE_SLOT[1] / 2), "y"))
    # lead anchors at the plate's south corners, where the face-down boards' leads turn round the
    # edge up to the node and the star: the SW pair over the adapter's west plug halo, the SE pair
    # over the buck's east lead halo (no board under or over either)
    # The SE pair shares the 8.6 between the plate's south edge (12.3) and the buck's E tie slot
    # (y 20.9) as three equal 1.67 walls, and keeps 1.65 to the east edge (53.95)
    out += [("anchor SW a", (-52.5, 17.0), "x"), ("anchor SW b", (-47.5, 17.0), "x"),
            ("anchor SE a", (50.2, 14.85), "y"), ("anchor SE b", (50.2, 18.35), "y")]
    return out


def slot_cutter(xy, axis, z0, z1):
    w, t = TIE_SLOT
    sx, sy = (t, w) if axis == "x" else (w, t)
    return Pos(xy[0], xy[1], (z0 + z1) / 2) * Box(sx, sy, z1 - z0)


def plate_outline():
    """The plate in plan (shapely): the five footprint + halo rectangles, the Ø14 pads, arms to any
    post off the outline, notches < 2 NOTCH_R closed (round fillets)."""
    rects = [sbox(*[board_plan(n)[i] for i in (0, 2, 1, 3)]) for n in SHELF_BOARDS]
    core = unary_union(rects)
    parts = rects + [Point(p).buffer(PAD_D / 2, 64) for p in POSTS]
    for p in POSTS:
        if not core.contains(Point(p)):                     # G3's 10-wide arm to the nearest footprint
            q = core.exterior.interpolate(core.exterior.project(Point(p)))
            parts.append(sbox(min(p[0], q.x), min(p[1], q.y), max(p[0], q.x), max(p[1], q.y))
                         .buffer(ARM_W / 2, join_style=2))
    u = unary_union(parts).buffer(NOTCH_R, 32, join_style=1).buffer(-NOTCH_R, 32, join_style=1)
    assert u.geom_type == "Polygon", u.geom_type
    return u


def _face(poly):
    f = make_face(Polyline(*[(x, y) for x, y in list(poly.exterior.coords)[:-1]], close=True))
    for hole in poly.interiors:
        f -= make_face(Polyline(*[(x, y) for x, y in list(hole.coords)[:-1]], close=True))
    return f


def hub_shelf():
    poly = plate_outline()
    s = Pos(0, 0, PLATE_BOT) * extrude(_face(poly), PLATE_T)
    for px, py in POSTS:                                     # posts from the underside up (fused)
        s += Pos(px, py, (PLATE_BOT + DECK_BOT_Z) / 2) * Cylinder(POST_D / 2, DECK_BOT_Z - PLATE_BOT)
    for name, d in (("star", STAR_POST_D), ("node_12v", NODE_POST_D)):
        for x, y in mounts(name):
            s += Pos(x, y, PLATE_TOP + STANDOFF_H / 2) * Cylinder(d / 2, STANDOFF_H)
    # cuts
    for px, py in POSTS:
        s -= Pos(px, py, (PLATE_BOT + DECK_BOT_Z) / 2) * Cylinder(POST_BORE / 2, DECK_BOT_Z - PLATE_BOT + 2)
        s -= Pos(px, py, PLATE_BOT + CB_DEPTH / 2 - 0.5) * Cylinder(CB_D / 2, CB_DEPTH + 1.0)
    top = PLATE_TOP + STANDOFF_H
    for name in ("star", "node_12v"):
        for x, y in mounts(name):
            s -= Pos(x, y, top - TAP_DEPTH / 2 + 0.5) * Cylinder(TAP / 2, TAP_DEPTH + 1.0)
    for x, y in mounts("bus_adapter"):
        s -= Pos(x, y, PLATE_TOP - PLATE_T / 2) * Cylinder(M25_TAP / 2, PLATE_T + 2)
    for _, xy, axis in tie_slots():
        s -= slot_cutter(xy, axis, PLATE_BOT - 1, PLATE_TOP + 1)
    (wx, wy), (ex, ey) = [xy for n, xy, _ in tie_slots() if n.startswith("buck tie")]
    s -= _box(wx - TIE_SLOT[1] / 2, ex + TIE_SLOT[1] / 2, wy - TIE_SLOT[0] / 2, wy + TIE_SLOT[0] / 2,
              PLATE_TOP - GROOVE_D, PLATE_TOP + 1)            # the buck's tie under the star
    return s


def hardware():
    """What is bolted or tied on that is not a board envelope: the adapter's spacers and its
    M2.5 heads (on the board's component face), the three M3 heads (flush), the zip ties' bands."""
    hw = []
    zb = PLATE_BOT - _BA["lift"]                             # the adapter's back (its pins' side)
    zc = zb - 1.6                                            # its component face
    for x, y in mounts("bus_adapter"):
        hw.append(Pos(x, y, (PLATE_BOT + zb) / 2) * (Cylinder(SPACER_D / 2, _BA["lift"]) -
                                                     Cylinder(1.35, _BA["lift"] + 1)))
        hw.append(Pos(x, y, zc - 0.9) * Cylinder(2.25, 1.8))  # M2.5 pan head (ISO 7045 Ø4.5 x 1.8)
    for px, py in POSTS:
        hw.append(Pos(px, py, PLATE_BOT + SCREW_HEAD_H / 2) * Cylinder(SCREW_HEAD_D / 2, SCREW_HEAD_H))
    return hw


def tie_bands():
    """The ties' bands on the plate's top face (the parts under the boards are in their stacks):
    the buck's in its groove, the UBEC's from the slot to the north edge."""
    out = []
    sl = {n: xy for n, xy, _ in tie_slots()}
    (wx, wy), (ex, _) = sl["buck tie W"], sl["buck tie E"]
    zt = PLATE_TOP - GROOVE_D
    out.append(_box(wx, ex, wy - TIE_W / 2, wy + TIE_W / 2, zt, zt + TIE_T))
    yn = board_plan("ubec_6v", halo=False)[3]
    for n, (x, y) in sl.items():
        if n.startswith("UBEC"):
            out.append(_box(x - TIE_W / 2, x + TIE_W / 2, y, yn, PLATE_TOP, PLATE_TOP + TIE_T))
    return out


# ====================================================================== the shelf as others see it
def shelf_parts(halo=True):
    """The shelf as built and loaded, BODY frame, solid by solid: {'hub_shelf' (plate + posts +
    Ø14 pads + standoffs), 'board <name>' (the five envelopes, each with its plug / lead halo
    when halo), 'hardware <k>' (the adapter's spacers and M2.5 heads, the flush M3 heads), 'tie
    band <k>'}. Overlap checks go solid by solid: OCCT booleans on a Compound of overlapping
    solids can be wrong (part_avionics found 193 mm^3 where each solid gave 0)."""
    out = {"hub_shelf": hub_shelf()}
    out.update({f"board {n}": board_box(n, halo) for n in SHELF_BOARDS})
    out.update({f"hardware {k}": s for k, s in enumerate(hardware())})
    out.update({f"tie band {k}": s for k, s in enumerate(tie_bands())})
    return out


_SHELF_MODEL = {}


def shelf_model(halo=True):
    """The posed shelf as ONE fused solid in the body frame (shelf_parts fused, so no overlapping
    children): what part_stand and part_bay check the cradle and the keel tub against, in place
    of params hub_shelf's box. Cached: build123d's booleans return new shapes, none mutates it."""
    if halo not in _SHELF_MODEL:
        parts = list(shelf_parts(halo).values())
        s = parts[0]
        for p in parts[1:]:
            s = s + p
        _SHELF_MODEL[halo] = s
    return _SHELF_MODEL[halo]


# ====================================================================== the audit's keep-outs
def capsule(points, r):
    """A tube of radius r along a polyline: cylinders + spheres at the nodes (a flexible run)."""
    pts = [np.array(p, float) for p in points]
    s = Pos(*pts[0]) * Sphere(r)
    for p, q in zip(pts[:-1], pts[1:]):
        d = q - p
        L = float(np.linalg.norm(d))
        s += Plane(origin=tuple((p + q) / 2), z_dir=tuple(d / L)).location * Cylinder(r, L)
        s += Pos(*q) * Sphere(r)
    return s


_TUB = {}


def tub_solid():
    """The keel tub as part_bay builds it (bay_tub + bay_lid + bay_door, fused, body frame: the
    door closed, the six Ø8 lid bosses up to the deck, the latch boss, the pilasters, the 14 AWG
    exit through the NE nose chamfer and the riser clip). It replaced params' chamfered box and
    G3's crown-plate ESTIMATE on 2026-10-07 (the wiring round). Cached."""
    if "t" not in _TUB:
        import part_bay as PBY                       # local: part_bay imports this module
        _TUB["t"] = PBY.bay_tub() + PBY.bay_lid() + PBY.bay_door()
    return _TUB["t"]


def stand_solids():
    """The stand as part_stand builds it, body frame, the robot standing in it by its tub:
    part_stand.stand_keepouts() ({'crown plate' (seat ring + skirt + the relieved plate),
    'cradle floor', 'cradle walls', 'x-stop pin'}; its whole 'crown' left out of the audit's
    rows: each piece is there)."""
    from part_stand import stand_keepouts            # local: part_stand imports this module
    return {k: v for k, v in stand_keepouts().items() if k != "crown"}


def lane_boxes():
    """The leg 2 / 3 lanes (params hub_shelf.lanes, correction 12): A, B, C for +x and their
    mirrors, in the 8 mm layer. The riser from C down to the mated connectors stays inside the
    drop keep-out box, which the audit already holds everything to."""
    ln = HS["lanes"]
    z0, z1 = ln["z"]
    out = {}
    for tag, (xs, ys) in zip("ABC", ln["route"]):
        for sx, side in ((1, "+x"), (-1, "-x")):
            xa, xb = sorted((sx * xs[0], sx * xs[1]))
            out[f"lane {tag} {side}"] = _box(xa, xb, ys[0], ys[1], z0, z1)
    return out


def latch_columns(d=5.0):
    """The Ø5 driver columns under the five carapace latches (iface SHELL_LATCH_AZ / R, B87, B126)
    and the tray's strike (DECK_HOLES), from the deck bottom down past the tub."""
    from part_deck import DECK_HOLES
    xy = [(f"latch {int(a) % 360}", (SHELL_LATCH_R * np.cos(np.deg2rad(a + SHELL_LATCH_AZ)),
                                     SHELL_LATCH_R * np.sin(np.deg2rad(a + SHELL_LATCH_AZ)))) for a in STATIONS]
    xy += [("tray strike", e[1]) for e in DECK_HOLES if e[2] == "strike"]
    return {n: Pos(x, y, DECK_BOT_Z - 50) * Cylinder(d / 2, 100) for n, (x, y) in xy}


def drop_plan(i, extra):
    """Leg i's drop keep-out (+extra) in plan, shapely (the box iface.leg_drop_keepout poses)."""
    from shapely.geometry import Polygon
    h = DROP_HALF + extra
    pts = []
    for lx, ly in ((-h, -h), (h, -h), (h, h), (-h, h)):
        v = (station_tf(i) * Pos(CABLE_CUTOUT_X + lx, ly, 0)).position
        pts.append((v.X, v.Y))
    return Polygon(pts)


# ---- B133: the 14 AWG feed, the tub's nose -> the node's input pads
AWG_R = 2.5                        # the orchestrated Ø5 envelope (14 AWG silicone ~3.4 + its sleeve)


AWG_OFF = 0.4                      # the feed's gap to the tub's outer faces it runs along
AWG_IN = 3.0                       # where the feed starts inside the nose compartment: this far in
                                   # from the NE chamfer's inner face (it comes from the fuse and the
                                   # loop key, part_bay), so the checked tube runs through the hole


def awg_route(t=None, inside=False):
    """The feed as a polyline (body frame), derived from the keep-outs, not typed in:
      - it leaves the nose compartment (pick 3: the loop key's panel XT60 is in the nose wall)
        through the NE nose chamfer, just over the interior floor, AWG_OFF + its radius out from
        the chamfer's middle: so its drop to z_u stays inside the unchamfered corner's plan and
        adds nothing to the robot's outline (part_bay cuts the Ø5.5 exit there, on this axis;
        inside=True starts the tube AWG_IN inside the chamfer's inner face, through that hole);
      - along the chamfer to y_c, then west at z_u UNDER leg 4's drop +5 (z -10..-50): z_u is midway
        between the drop box's floor and the tub's bottom. North of the tub there is no way past
        that drop: its +5 box reaches y 5.50 at x 67.1, inside the tub's plan (north wall y 7.4),
        and over the roof lane A + the hook leave a -18 .. -16.26 window a Ø5 cannot use (B133);
      - up at x_r, midway in the gap between the plate / buck halo corner (53.95, 12.3) and the
        drop +5's west edge;
      - west at z_b (between the cradle's north wall top and lane A's floor) at y_c, midway
        between the tub's north wall and the star's south edge, to the node; north to its pads.
    The node (its board + lead halo) is the termination, not a keep-out of the feed."""
    t = t or bay_tub_extent()
    x1, y1, zt = t["x"][1], t["y"][1], t["z"][0]
    c = BS["nose_chamfer"]
    d5 = DROP_DEPTH + 5.0
    z_u = ((DECK_BOT_Z - d5) + zt) / 2                       # -52.7
    z_0 = t["z_in"][0] + AWG_R                               # on the interior floor: -50.5
    star_y0 = board_plan("star")[2]
    y_c = (y1 + star_y0) / 2                                 # 10.3: 2.9 off the north wall
    o = (AWG_R + AWG_OFF) / np.sqrt(2)
    s0 = (x1 - c / 2 + o, y1 - c / 2 + o)                    # off the NE chamfer's middle
    s2 = (s0[0] - (y_c - s0[1]), y_c)                        # along the chamfer, same offset
    # x_r: the riser's plan circle >= its radius from the plate and from leg 4's drop +5
    plate = plate_outline()
    dp = drop_plan(4, 5.0)
    xs = np.arange(40.0, 75.0, 0.05)
    ok = [x for x in xs if not plate.contains(Point(x, y_c)) and Point(x, y_c).distance(plate) >= AWG_R
          and not dp.contains(Point(x, y_c)) and Point(x, y_c).distance(dp) >= AWG_R]
    x_r = round((min(ok) + max(ok)) / 2, 2)
    lane_z0 = HS["lanes"]["z"][0]
    wall_top = t["z"][0] + STAND["wall_h"]
    z_b = round(((wall_top + AWG_R) + (lane_z0 - AWG_R)) / 2, 1)   # -23.7: the middle
    nb = board_plan("node_12v", halo=False)
    x_j = nb[1] - 4.0                                        # the node's SE pads
    pts = [(s0[0], s0[1], z_0), (s0[0], s0[1], z_u), (s2[0], s2[1], z_u), (x_r, y_c, z_u),
           (x_r, y_c, z_b), (x_j, y_c, z_b), (x_j, nb[2], z_b)]
    if inside:                                               # back along the chamfer's normal
        d_in = (AWG_R + AWG_OFF + BS["bay_wall"] + AWG_IN) / np.sqrt(2)
        pts.insert(0, (s0[0] - d_in, s0[1] - d_in, z_0))
    return pts, dict(z_u=z_u, y_c=y_c, x_r=x_r, z_b=z_b, z_0=z_0, s0=s0, ok=(min(ok), max(ok)), x1=x1)


# ---- the looms: legs 0, 1, 4 straight from the star to their drops below the hook feet (Ø7);
# leg 2's along the shelf's south edge to its lane A (the cross run); leg 3 drops into its lane A
LOOM_R = 3.5
LOOM_Z = -19.0                     # the star's lead band (its top -13.9, the bend 6) under the hooks
CROSS_Z = -14.0                    # leg 2's cross run, in lane A's layer (z -17..-10)


def looms():
    """{name: (points, own drop index or None)} — the loom ends inside its own drop box."""
    s = board_plan("star")
    yl = 25.0                      # legs 1 / 4: 1.5 under the posts (+-50, 34) at Ø7
    return {"leg 0 loom": ([(12.0, s[3], LOOM_Z), (6.0, 72.0, LOOM_Z)], 0),
            "leg 1 loom": ([(s[0], yl, LOOM_Z), (-68.0, yl, LOOM_Z)], 1),
            "leg 4 loom": ([(s[1], yl, LOOM_Z), (68.0, yl, LOOM_Z)], 4),
            "leg 2 cross run": ([(s[0], 9.5, CROSS_Z), (-49.0, 9.5, CROSS_Z)], None)}


# ---- the tray's trunk through the deck's (0, 50) slot, round the plate's NORTH-WEST edge to the
# adapter's west plug zone (K3 s4_cable: a straight drop cuts the node's halo and the plate)
TRAY_CABLE_D = 9.42


def tray_cable():
    r = TRAY_CABLE_D / 2
    from iface import leg_port_hook_envelope
    hook_z = leg_port_hook_envelope("any").bounding_box().min.Z
    node_top = board_z("node_12v")[1]
    z_t = ((hook_z - r) + (node_top + r)) / 2                # between the leg-0 hook and the node halo
    xw = board_plan("node_12v")[0] - r - 1.0                 # 1.0 west of the node's halo
    yn = board_plan("bus_adapter")[3] + r + 1.5              # 1.5 north of the adapter's halo
    z_d = PLATE_BOT - r - 2.0                                # under the plate (and the pads)
    return [(0.0, 50.0, DECK_BOT_Z - r - 0.05), (0.0, 50.0, z_t), (xw, yn, z_t), (xw, yn, z_d),
            (xw, 42.0, z_d)], z_t


# ====================================================================== checks
def _v(s):
    return 0.0 if s is None else s.volume


def gap(a, b):
    """(overlap mm^3, distance mm)."""
    return _v(a & b), a.distance_to(b)


def ray_through_table(xy, k):
    """Post k's screw against the TABLE (DECK_HOLES shelf_post_k, its cutter posed in the body
    frame), not today's deck solid: (entry, the screw's Ø3 shank x the shelf mm^3, the Ø1 axis
    inside the deck NOT in the hole mm^3, the screw's tip z). The shank runs from the head's
    bearing face (the counterbore's ceiling) to its tip; the axis from the deck bottom to the tip
    must lie inside the hole (the thread forms in its Ø2.8 wall)."""
    from part_deck import DECK_HOLES, deck_hole_cutter
    e = next(e for e in DECK_HOLES if e[0] == f"shelf_post_{k}")
    L, _ = post_screw()
    z_head = PLATE_BOT + CB_DEPTH                            # the head's bearing face
    z_tip = z_head + L
    shank = Pos(xy[0], xy[1], (z_head + DECK_BOT_Z) / 2) * Cylinder(1.5, DECK_BOT_Z - z_head)
    axis = Pos(xy[0], xy[1], (DECK_BOT_Z + z_tip) / 2) * Cylinder(0.5, z_tip - DECK_BOT_Z)
    cut = Pos(0, 0, DECK_BOT_Z) * deck_hole_cutter(e)
    return e, shank, _v(axis - cut), z_tip


def driver_reach(xy, z_from, obstacles, d=5.0):
    """A Ø d driver column from z_from straight down to the floor under the stand: the first
    obstacle it meets (name, overlap) or None."""
    col = Pos(xy[0], xy[1], (z_from - 120.0) / 2) * Cylinder(d / 2, z_from + 120.0)
    hits = [(n, _v(col & s)) for n, s in obstacles.items()]
    hits = [h for h in hits if h[1] > 1e-3]
    return hits


if __name__ == "__main__":
    bad = []

    def need(name, ok, detail=""):
        print(f"  {name}: {'OK' if ok else 'FAIL'}" + (f" ({detail})" if detail else ""))
        if not ok:
            bad.append(name)

    print("hub shelf (option A, case iii at bay_w 50):")
    # ---- the contract: the poses are params', params are the measured packing
    need("board poses = params hub_shelf.boards = the measured case iii",
         all(POSE[k] == (tuple(CASE_III[k][0]), CASE_III[k][1], CASE_III[k][2]) for k in CASE_III)
         and set(POSE) == set(CASE_III), ", ".join(f"{k} {POSE[k][0]} rot {POSE[k][1]:g} {POSE[k][2]}" for k in POSE))
    zb = board_z("bus_adapter")[0]
    need("the face-down adapter's pins = params bottom_z", abs(zb - HS["bottom_z"]) < 1e-6,
         f"{PLATE_BOT:.1f} - {SHELF_BOARDS['bus_adapter']['stack']:.1f} = {zb:.2f} vs {HS['bottom_z']}")
    assert _BA["down"] < _BA["lift"], "the adapter's pins must clear the plate on its spacers"
    u = unary_union([sbox(*[board_plan(n)[i] for i in (0, 2, 1, 3)]) for n in SHELF_BOARDS])
    ub = u.bounds
    need("footprint + halo union = params hub_shelf x / y", max(abs(ub[0] - HS["x"][0]), abs(ub[2] - HS["x"][1]),
                                                               abs(ub[1] - HS["y"][0]), abs(ub[3] - HS["y"][1])) < 0.11,
         f"x {ub[0]:.2f}..{ub[2]:.2f}, y {ub[1]:.2f}..{ub[3]:.2f} vs {HS['x']}, {HS['y']}")
    for n in SHELF_BOARDS:
        x0, x1, y0, y1 = board_plan(n, halo=False)
        z0, z1 = board_z(n)
        print(f"    {n:12s} {POSE[n][2]:4s} board x {x0:.2f}..{x1:.2f} y {y0:.2f}..{y1:.2f} z {z0:.2f}..{z1:.2f}")

    shelf = hub_shelf()
    export(shelf, "hub_shelf")
    bb = shelf.bounding_box()
    poly = plate_outline()
    print(f"  hub_shelf: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, x {bb.min.X:.2f}..{bb.max.X:.2f}, "
          f"y {bb.min.Y:.2f}..{bb.max.Y:.2f}, z {bb.min.Z:.2f}..{bb.max.Z:.2f}; plate {poly.area:.0f} mm^2")
    need("one solid", len(shelf.solids()) == 1, f"{len(shelf.solids())}")
    bed = PR["bed_mm"]
    need("fits the bed (plate down, posts up)", bb.size.X <= bed[0] and bb.size.Y <= bed[1] and bb.size.Z <= bed[2],
         f"{bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.1f} in {bed[0]:.0f} x {bed[1]:.0f} x {bed[2]:.0f}")
    need("the posts reach the deck bottom", abs(bb.max.Z - DECK_BOT_Z) < 1e-6, f"top z {bb.max.Z:.3f}")
    L, e = post_screw()
    print(f"  post screws: 3 x M3 x {L:.0f} ISO 4762 from below, head flush in Ø{CB_D:.1f} x {CB_DEPTH:.1f}, "
          f"through {PLATE_T - CB_DEPTH + POST_L:.1f} of post, {e:.1f} into the deck's Ø{TAP} x {DECK_HOLE_DEPTH:.0f} "
          f"({DECK_HOLE_DEPTH - e:.1f} short of its bottom)")
    n_tie = {"board": 1 + 2, "lead anchor": 2, "loom at a post": 2}
    print(f"  board hardware: star + node 8 x M3 x 6 (4.4 engaged in the {STANDOFF_H:.0f} posts, Ø{TAP} x "
          f"{TAP_DEPTH:.0f} blind); adapter 4 x M2.5 x 8 pan head from below + 4 x Ø{SPACER_D:.0f} x "
          f"{_BA['lift']} round spacers, ID >= 2.6 ({PLATE_T:.1f} engaged in the Ø{M25_TAP} bores, the tip "
          f"{8 - 1.6 - _BA['lift'] - PLATE_T:.1f} proud of the top face); {sum(n_tie.values())} x 3.6 zip ties ("
          + ", ".join(f"{k} {v}" for k, v in n_tie.items()) + ")")

    # ---- the assembly the keep-outs see: the shelf, the boards (with halos), the hardware
    boards = {n: board_box(n) for n in SHELF_BOARDS}
    hw = hardware()
    bands = tie_bands()
    asm = {"hub_shelf": shelf, **{f"board {n}": b for n, b in boards.items()},
           "hardware": Compound(children=hw), "tie bands": Compound(children=bands)}
    posts_only = Compound(children=[Pos(px, py, (PLATE_TOP + DECK_BOT_Z) / 2) * Cylinder(POST_D / 2, POST_L)
                                    for px, py in POSTS])
    plate_pads = Pos(0, 0, PLATE_BOT) * extrude(_face(poly), PLATE_T)
    # the whole loaded shelf as ONE fused solid (shelf_model: what part_stand and part_bay check
    # against); a Compound of its overlapping children (the spacers in the adapter's envelope) is
    # what the OCCT booleans can get wrong
    every = shelf_model()
    assert asm["hub_shelf"] is shelf

    def audit(tag, ko, expect=None, items=None, gate=True):
        rows = []
        for n, s in (items or {"shelf + boards + hardware": every, "posts": posts_only,
                               "plate + pads": plate_pads}).items():
            v, g = gap(s, ko)
            rows.append((n, v, g))
        v_all = sum(r[1] for r in rows[:1])
        txt = "; ".join(f"{n} {v:.2f} / {g:.3f}" for n, v, g in rows)
        ok = v_all < 1e-3 if gate else True
        if expect is not None:
            txt += f" (expected {expect})"
        need(f"x {tag} (mm^3 / gap)", ok, txt)
        return rows

    ko = body_keepouts(0.0, "any")
    ko5 = body_keepouts(5.0)
    audit("hook envelopes 'any' (5)", Compound(children=ko["hooks"]), "posts 0.339")
    audit("drops +5 (5)", Compound(children=ko5["drops"]), "posts 3.57, plate + pads 0.57")
    audit("drops +0 (5)", Compound(children=ko["drops"]))
    # the deck: posed as part_deck poses it; the posts end ON its underside (0 mm^3, 0 gap)
    from part_deck import body_deck, T as T_DECK
    deck = Pos(0, 0, -10) * body_deck()
    audit("the posed deck (Pos(0, 0, -10) * body_deck())", deck)
    for k, xy in enumerate(POSTS):
        ent, shank, outside, z_tip = ray_through_table(xy, k)
        v_shelf = _v(shank & shelf)
        hole_top = DECK_BOT_Z + ent[3][1]
        need(f"post {k} {xy} M3: its Ø3 shank free in the shelf's Ø{POST_BORE} bore, its tip inside DECK_HOLES "
             f"{ent[0]} ({ent[2]} Ø{ent[3][0]} x {ent[3][1]})",
             ent[1] == xy and ent[2] == "m3_below" and abs(ent[3][1] - DECK_HOLE_DEPTH) < 1e-9 and
             v_shelf < 1e-3 and outside < 1e-3 and z_tip <= hole_top - SCREW_END_GAP + 1e-9,
             f"shank x shelf {v_shelf:.3f} mm^3, axis outside the hole {outside:.3f} mm^3, tip z {z_tip:.2f} "
             f"({hole_top - z_tip:.2f} under the hole's end z {hole_top:.1f}, {DECK_BOT_Z + T_DECK - hole_top:.1f} skin)")
    tub = tub_solid()
    audit("the keel tub as part_bay builds it (bay_tub + bay_lid + bay_door, the exit hole, the "
          "riser clip)", tub, "4.80 from the params box")
    t_ext = bay_tub_extent()
    sb_ = every.bounding_box()
    print(f"    the shelf's south face y {sb_.min.Y:.2f} (params y {HS['y'][0]}): {sb_.min.Y - t_ext['y'][1]:.2f} "
          f"north of the tub's north wall (y {t_ext['y'][1]}), its lowest z {sb_.min.Z:.2f} "
          f"(params bottom_z {HS['bottom_z']})")
    # the carapace: part_shell builds a sector (and the cap's magnets) at station az 0, body z, so
    # each goes on its station, Rot(0, 0, STATIONS[k]) (Rot(0, 0, 72 k) is 18 deg off every leg)
    from part_shell import shell_sector, shell_cap
    sec = shell_sector()
    shell = Compound(children=[Rot(0, 0, a) * sec for a in STATIONS] + [Rot(0, 0, STATIONS[0]) * shell_cap()])
    audit("the carapace (5 sectors on their stations + cap)", shell)
    from part_coxa import coxa_yaw_base
    base = coxa_yaw_base()
    bases = Compound(children=[station_tf(i) * base for i in range(len(STATIONS))])
    audit("the docked coxa bases (5)", bases, "posts 0.885")
    stand = stand_solids()
    expect_st = {"crown plate": "4.00 (the adapter over the relieved plate)",
                 "cradle walls": "1.30 (the shelf's south face to the north wall's outer face)"}
    for n, s in stand.items():
        audit(f"the stand (part_stand, posed): {n}", s, expect_st.get(n),
              items={"shelf + boards + hardware": every, "board bus_adapter": boards["bus_adapter"]})
    lanes = lane_boxes()
    audit("the lanes A / B / C (+x, -x)", Compound(children=list(lanes.values())), "star 1.20",
          items={"shelf + boards + hardware": every, "board star": boards["star"]})
    cols = latch_columns()
    for n, c in cols.items():
        rows = audit(f"the {n} driver column (Ø5)", c, "UBEC 1.08" if n == "latch 90" else None,
                     items={"shelf + boards + hardware": every, "board ubec_6v": boards["ubec_6v"]})

    # ---- the boards pairwise (with halos), on their plate, on their mounts
    names = list(SHELF_BOARDS)
    pair = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            v, g = gap(boards[names[i]], boards[names[j]])
            pair.append((g, v, names[i], names[j]))
    pair.sort()
    need("the five boards pairwise (with halos)", all(p[1] < 1e-3 for p in pair),
         "; ".join(f"{a} - {b} {g:.2f}" for g, v, a, b in pair[:4]))
    real = {n: board_box(n, halo=False) for n in names}
    for n in names:
        z0, z1 = board_z(n)
        side = _box(*board_plan(n, halo=False), z0 + 0.01, z1 - 0.01)
        v = _v(side & shelf)
        need(f"{n} envelope x the printed shelf (it bears on its {'posts' if POSE[n][2] == 'up' else 'face'})",
             v < 1e-3 if POSE[n][2] == "down" or n == "bus_adapter" else True, f"{v:.3f} mm^3")
    # the up boards' real stacks on their posts: the board itself clears the plate's posts / ties
    for n in ("star", "node_12v"):
        x0, x1, y0, y1 = board_plan(n, halo=False)
        zb0 = PLATE_TOP + STANDOFF_H
        under = STAR_TAILS if n == "star" else 1.0
        brd = _box(x0, x1, y0, y1, zb0 - under, zb0 + 1.6)
        v_p, g_p = gap(brd, posts_only)
        v_t, g_t = gap(brd, Compound(children=bands))
        need(f"{n} board (+ {under:.1f} under it) x the shelf posts / the tie bands", v_p < 1e-3 and v_t < 1e-3,
             f"posts {g_p:.2f}, ties {g_t:.2f}")
    v, g = gap(boards["star"], posts_only)
    need("star envelope to post (50, 34)", v < 1e-3, f"{g:.2f} (K3 0.80)")
    print(f"  star top z {board_z('star')[1]:.2f}: {DECK_BOT_Z - board_z('star')[1]:.2f} under the deck "
          f"(standoff_h {STANDOFF_H:g}; the old 8 mm posts would leave {DECK_BOT_Z - board_z('star')[1] - 3:.2f}, K3 0.90); "
          f"node top {board_z('node_12v')[1]:.2f} ({DECK_BOT_Z - board_z('node_12v')[1]:.2f} under the deck)")
    # the slots: none over a face-down board's footprint, none under an up board or its standoffs
    worst = (np.inf, "")
    for n_s, xy, axis in tie_slots():
        sc = slot_cutter(xy, axis, PLATE_BOT, PLATE_TOP)
        for n in names:
            if n_s.startswith(("buck tie", "UBEC tie")) and n in ("buck_5v", "ubec_6v"):
                continue                                   # a tie slot hugs its own board
            fz = _box(*board_plan(n, halo=False), PLATE_BOT - 1, PLATE_TOP + 1)
            g = sc.distance_to(fz)
            if g < worst[0]:
                worst = (g, f"{n_s} - {n}")
    need("every tie / anchor slot off the boards' footprints (both faces)", worst[0] > 0.1,
         f"tightest {worst[1]} {worst[0]:.2f}")

    # ---- B133: the 14 AWG feed, from inside the nose compartment out through part_bay's exit in
    # the NE chamfer (the real tub: the hole, the riser clip), under leg 4's drop, up, to the node
    pts, info = awg_route(inside=True)
    tube = capsule(pts, AWG_R)
    awg = capsule(pts[1:], AWG_R)                        # the looms meet the feed outside the tub
    feed_ko = {"drops +5": Compound(children=ko5["drops"]), "hooks 'any'": Compound(children=ko["hooks"]),
               "deck": deck, "tub + lid + door (part_bay)": tub, **stand,
               **{k: v for k, v in lanes.items()}, "coxa bases": bases, **cols,
               "shelf": shelf, **{f"board {n}": b for n, b in boards.items() if n != "node_12v"},
               "hardware": Compound(children=hw), "tie bands": Compound(children=bands)}
    rows = sorted((gap(tube, s)[::-1] + (n,)) for n, s in feed_ko.items())
    hit = [r for r in rows if r[1] > 1e-3]
    L_feed = sum(float(np.linalg.norm(np.subtract(q, p))) for p, q in zip(pts[1:-1], pts[2:]))
    print(f"  B133 14 AWG feed (Ø{2 * AWG_R:g}), the nose x {info['x1']:.1f} -> the node's SE pads, {L_feed:.0f} long "
          f"from the chamfer: " + " -> ".join(f"({x:.2f}, {y:.2f}, {z:.2f})" for x, y, z in pts))
    print(f"    through part_bay's exit in the NE chamfer over the interior floor (z {info['z_0']:.2f}); under leg 4's "
          f"drop +5 at z {info['z_u']:.2f} (its floor {DECK_BOT_Z - DROP_DEPTH - 5:.1f}, the tub bottom "
          f"{bay_tub_extent()['z'][0]:.1f}); riser x {info['x_r']:.2f} (feasible {info['ok'][0]:.2f}..{info['ok'][1]:.2f}, "
          f"part_bay's clip on it); band y {info['y_c']:.2f} z {info['z_b']:.1f}")
    need("the 14 AWG feed through the real tub's exit >= 0 from drops +5, hooks, deck, tub + lid + door, the "
         "stand, lanes, bases, latch columns, the shelf, its ties and the other boards", not hit,
         "tightest " + "; ".join(f"{n} {g:.2f}" for g, v, n in rows[:5]) +
         ("; HITS " + "; ".join(f"{n} {v:.2f} mm^3" for g, v, n in hit) if hit else ""))
    print(f"    against the drops +0 (the connectors' own box): {gap(tube, Compound(children=ko['drops']))[1]:.2f}")
    v_n, g_n = gap(tube, boards["node_12v"])
    print(f"    the node (its termination): {v_n:.1f} mm^3 into its board + lead halo, as it must")
    # the proposal's band (y 8..11, z -26..-22, the nose -> the node) for the record
    nb = board_plan("node_12v", halo=False)
    t = bay_tub_extent()
    for y0 in (8.0, 7.9, 8.8):
        band = _box((nb[0] + nb[1]) / 2 - 1.5, t["x"][1], y0, y0 + 3.0, -26.0, -22.0)
        v5, g5 = gap(band, Compound(children=ko5["drops"]))
        v0, g0 = gap(band, Compound(children=ko["drops"]))
        print(f"    the proposal's band y {y0:.1f}..{y0 + 3:.1f}, z -26..-22: drops +5 {v5:.2f} mm^3 / {g5:.2f}; "
              f"drops +0 {v0:.2f} / {g0:.2f}; tub {gap(band, tub)[1]:.2f}")

    # ---- the looms (Ø7)
    for n, (lp, own) in looms().items():
        tube = capsule(lp, LOOM_R)
        kos = {"hooks 'any'": Compound(children=ko["hooks"]),
               "drops +0 (others)": Compound(children=[d for i, d in enumerate(ko["drops"]) if i != own]),
               "deck": deck, "tub + bosses": tub, "coxa bases": bases, "shelf": shelf,
               "tie bands": Compound(children=bands), "awg": awg, **cols,
               **{f"board {b}": s for b, s in boards.items() if b != "star"}}
        rows = sorted((gap(tube, s)[::-1] + (k,)) for k, s in kos.items())
        hit = [r for r in rows if r[1] > 1e-3]
        need(f"{n} (Ø{2 * LOOM_R:.0f}) " + " -> ".join(f"({x:.1f}, {y:.1f}, {z:.1f})" for x, y, z in lp), not hit,
             "tightest " + "; ".join(f"{k} {g:.2f}" for g, v, k in rows[:3]) +
             ("; HITS " + "; ".join(f"{k} {v:.2f}" for g, v, k in hit) if hit else ""))

    # ---- the tray's cable through the deck's (0, 50) slot, round the plate's north-west edge
    tp, z_t = tray_cable()
    cab = capsule(tp, TRAY_CABLE_D / 2)
    straight = Pos(0, 50, (DECK_BOT_Z + PLATE_BOT - 6) / 2) * Cylinder(TRAY_CABLE_D / 2, DECK_BOT_Z - PLATE_BOT + 6)
    print(f"  the tray's (0, 50) cable straight down: node + halo {gap(straight, boards['node_12v'])[0]:.1f}, "
          f"plate {_v(straight & plate_pads):.1f} mm^3 (K3: 158 / 37.7 with its packing)")
    kos = {"hooks 'any'": Compound(children=ko["hooks"]), "drops +5": Compound(children=ko5["drops"]),
           "tub + bosses": tub, "deck": deck, **cols, "shelf": shelf, "hardware": Compound(children=hw),
           **{f"board {b}": s for b, s in boards.items() if b != "bus_adapter"},
           "adapter board": real["bus_adapter"], **{n: capsule(lp, LOOM_R) for n, (lp, _) in looms().items()}}
    rows = sorted((gap(cab, s)[::-1] + (k,)) for k, s in kos.items())
    hit = [r for r in rows if r[1] > 1e-3]
    need(f"the tray cable (Ø{TRAY_CABLE_D}) (0, 50) -> round the NW edge -> the adapter's west plug zone: " +
         " -> ".join(f"({x:.1f}, {y:.1f}, {z:.1f})" for x, y, z in tp), not hit,
         "tightest " + "; ".join(f"{k} {g:.2f}" for g, v, k in rows[:4]) +
         ("; HITS " + "; ".join(f"{k} {v:.2f}" for g, v, k in hit) if hit else ""))

    # ---- B134: service from below with the robot on the stand (reported, not gated)
    zc = PLATE_BOT - _BA["lift"] - 1.6 - 1.8                 # under the adapter's M2.5 heads
    reach = []
    for x, y in mounts("bus_adapter"):
        h = driver_reach((x, y), zc, stand)
        reach.append(f"({x:.1f}, {y:.1f}) " + ("free" if not h else "blocked by " + ", ".join(n for n, _ in h)))
    print("  B134 (report): adapter screws from below on the stand: " + "; ".join(reach))
    reach = []
    for x, y in POSTS:
        h = driver_reach((x, y), PLATE_BOT, stand)
        g = min((Pos(x, y, PLATE_BOT - 60) * Cylinder(2.5, 120)).distance_to(s) for s in stand.values())
        reach.append(f"({x:.0f}, {y:.0f}) " + (f"free ({g:.2f} to the stand)" if not h else "blocked"))
    print("    the shelf's M3s from below on the stand: " + "; ".join(reach))
    v, g = gap(boards["bus_adapter"], stand["crown plate"])
    print(f"    the shelf lowered on the stand: the adapter meets the relieved crown plate after {g:.2f} mm; "
          "hub service = the robot off the stand (B134)")
    # ---- the adapter's H2 (report): a straight Dupont, face-down, against its own envelope, the
    # tub's bottom (the robot's lowest point) and the crown plate's top on the stand
    zc = PLATE_BOT - _BA["lift"] - 1.6
    z_dp = zc - H2_BASE - DUPONT_L
    print(f"  H2 (report): a straight Dupont on a vertical H2 header ends at z {z_dp:.1f} ({H2_BASE:g} base + "
          f"{DUPONT_L:g} housing under the component face z {zc:.1f}, no lead bend): {HS['bottom_z'] - z_dp:.1f} under "
          f"bottom_z, {bay_tub_extent()['z'][0] - z_dp:.1f} under the tub's bottom = into the crown plate on the stand. "
          f"Within the {_BA['up'] - 1.6:.1f} parts envelope only soldered leads or a right-angle housing along the "
          "board into an end halo (VERIFY where H2 sits)")
    print(f"part_busboard (hub_shelf) checks: {'CLEAN' if not bad else f'{len(bad)} FAILURE(S): ' + '; '.join(bad)}")
    raise SystemExit(1 if bad else 0)
