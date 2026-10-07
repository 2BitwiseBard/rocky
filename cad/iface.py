"""Shared interface geometry (D020) — every part that carries one of the six
standard interfaces builds its features THROUGH these helpers, reading frozen
dims from params.yaml. The deck, the bench jig, and the coxa plate all call
the same functions: fit is guaranteed by construction, not by diligence.

Frames: leg-port features are defined in LEG-LOCAL coords (yaw axis at
origin, +x outboard) exactly like part_coxa.py; callers pass a build123d
transform (e.g. Rot(0,0,ang) * Pos(110,0,0), station_tf(i)) placing the leg station.

Since 2026-10-07 the under-deck keep-outs live here too (leg_port_hook_envelope,
leg_drop_keepout, body_keepouts): every body-layout part checks against the same
solids. iface never imports a part module (part_deck imports iface).
"""
import numpy as np
from build123d import *
from common import params

P = params()
IF = P["interfaces"]
PR = P["print"]
FIT = PR["clearance_fit"]

# I1 cable pass-through in the deck (leg-local; frozen D020 convention): the
# leg drop's XT30 + JST-XH-5 mate here. part_coxa.harness_path() starts on it.
CABLE_CUTOUT_X = -29.0
CABLE_CUTOUT_W = 11.0


# ====================================================================== I1
# The seats (2026-10-07, body-layout decision 15, B117). As frozen in D020 the port could
# not dock: the L's 5.15 foot met a 4.2 slot and the Ø4 x 8 dowels blocked the pivot. Now
# the deck carries two printed 30-deg cone posts at seat_xy and the plate two blind sockets
# that are the posts' own flanks: the plate is hooked at 7-10 deg with a radial approach
# and lowered onto the cones, which close by design in the last 0.61 of the drop
# (check_dock.py proves the path and the docked fit on the real solids).
PLATE_X0 = -46.0      # the coxa plate's inboard edge (part_coxa, the port coupon): the plate
                      # extension below runs inboard of it to the hook's stem


def _seat_frustum(x, y0, y1, z0, r0, h, ta):
    """A cone frustum (base radius r0 at z0, flank tan ta, height h) stretched along +y from
    y0 to y1 (y1 == y0: a plain cone frustum): the two end frustums + the prism of their
    axial trapezoid, i.e. their hull."""
    r1 = r0 - h * ta
    s = Pos(x, y0, z0 + h / 2) * Cone(r0, r1, h)
    if y1 > y0 + 1e-9:
        s += Pos(x, y1, z0 + h / 2) * Cone(r0, r1, h)
        f = make_face(Polyline((x - r0, y0, z0), (x + r0, y0, z0), (x + r1, y0, z0 + h),
                               (x - r1, y0, z0 + h), close=True))
        s += extrude(f, amount=y1 - y0, dir=(0, 1, 0))
    return s


def leg_port_seats(top_z):
    """The deck's two seat posts (leg-local, deck top at top_z): the cone at seat_xy[0] and
    the vee (the cone stretched seat_vee_len toward +y) at seat_xy[1]."""
    lp = IF["leg_port"]
    ta = np.tan(np.deg2rad(lp["seat_half_angle"]))
    (cx, cy), (vx, vy) = lp["seat_xy"]
    out = _seat_frustum(cx, cy, cy, top_z, lp["seat_r"], lp["seat_h"], ta)
    out += _seat_frustum(vx, vy, vy + lp["seat_vee_len"], top_z, lp["seat_r"], lp["seat_h"], ta)
    return out


def leg_port_seat_zone(top_z):
    """A box over each seat post's footprint (+0.5 all round), from 0.01 over the deck top
    at top_z to 0.5 over the posts' tips: a REAL deck (or jig, or deck coupon) intersected
    with it is that part's own posts, which a check compares with leg_port_seats(top_z) &
    this zone and then docks against. Checking the sockets against leg_port_seats() alone
    cannot see a deck that lost its posts (2026-10-07 review: with seats=False check_dock
    stayed CLEAN, though such a leg has no x / y / yaw location). The 0.01 keeps the zone's
    floor off the deck top (no coincident faces); the posts' bottom 0.01 sits in the
    sockets' mouth relief, which nothing touches."""
    lp = IF["leg_port"]
    g, h = lp["seat_r"] + 0.5, lp["seat_h"] + 0.49
    (cx, cy), (vx, vy) = lp["seat_xy"]
    out = Pos(cx, cy, top_z + 0.01 + h / 2) * Box(2 * g, 2 * g, h)
    vl = lp["seat_vee_len"]
    out += Pos(vx, vy + vl / 2, top_z + 0.01 + h / 2) * Box(2 * g, 2 * g + vl, h)
    return out


def seat_posts_symdiff(part, top_z, tf=None):
    """How far `part`'s OWN seat posts (part & the zone, both placed by tf: leg-local -> part)
    are from leg_port_seats(top_z) as drawn: (symmetric-difference volume, the posts or None).
    0: as drawn; the drawn posts' whole volume (82.17 mm^3): the part has none."""
    tf = tf if tf is not None else Pos(0, 0, 0)
    zone = tf * leg_port_seat_zone(top_z)
    ref = (tf * leg_port_seats(top_z)) & zone
    real = part & zone
    if real is None or real.volume < 1e-9:          # an empty compound: no posts at all
        return ref.volume, None
    return (real - ref).volume + (ref - real).volume, real


def leg_port_socket_walls(plate_half_w, fit):
    """Plate left between each seat socket's mouth and the plate's y edge (|y| plate_half_w):
    (cone, vee). At fit 0 that is 1.5 / exactly 1.20 (I1_DOCK_OPTIONS 7); a filed seat_fit
    takes it straight off both (fit 0.2: 1.3 / 1.00). If the vee wall drops under 1.20,
    I1_DOCK_OPTIONS 7's options are mouth 0.2 or the seats at |y| 17.1 (not re-proven)."""
    lp = IF["leg_port"]
    rm = lp["seat_r"] + fit + lp["seat_mouth"]
    (cx, cy), (vx, vy) = lp["seat_xy"]
    sl, vl = lp["seat_vee_slack"], lp["seat_vee_len"]
    vee = max(abs(vy - sl - rm), abs(vy + vl + sl + rm))
    return plate_half_w - (abs(cy) + rm), plate_half_w - vee


def seat_socket(x, y0, y1, plate_bot_z, plate_top_z, fit=0.0):
    """ONE seat socket (a cutter), blind from the plate underside at plate_bot_z: the post's
    own flank grown radially by `fit` (0: it touches the post when docked, nominal), stopped
    seat_roof under the plate top, run from y0 to y1 (y1 > y0: the vee's slot), plus the
    mouth relief: seat_mouth over the post's base radius, seat_mouth + seat_relief tall from
    the underside, i.e. 0.3 over the relieved face, so the post's base corner (and its
    elephant foot) never meets the socket's edge. The port coupons print it at fit 0 /
    +0.1 / +0.2 (port_coupon_plate_relief46 / _fit10 / _fit20, I1_DOCK_OPTIONS 6.2) and
    the winner is filed as print.seat_fit: docked play grows from 0 at about +0.05 to
    x +-0.15 at +0.2 (J/f2_relief)."""
    lp = IF["leg_port"]
    ta = np.tan(np.deg2rad(lp["seat_half_angle"]))
    h = plate_top_z - lp["seat_roof"] - plate_bot_z
    r0 = lp["seat_r"] + fit
    # 0.01 below the underside, so the cutter breaks the face cleanly (same flank line)
    out = _seat_frustum(x, y0, y1, plate_bot_z - 0.01, r0 + 0.01 * ta, h + 0.01, ta)
    rm, m = r0 + lp["seat_mouth"], lp["seat_mouth"] + lp["seat_relief"]
    zc = plate_bot_z + m / 2 - 0.01
    out += Pos(x, y0, zc) * Cylinder(rm, m + 0.02)
    if y1 > y0 + 1e-9:                               # the vee's mouth: a stadium
        out += Pos(x, y1, zc) * Cylinder(rm, m + 0.02)
        out += Pos(x, (y0 + y1) / 2, zc) * Box(2 * rm, y1 - y0, m + 0.02)
    return out


def leg_port_sockets(plate_bot_z, plate_top_z, fit=0.0):
    """The plate's two seat sockets (cutters): the cone's at seat_xy[0], and the vee's
    socket seat_vee_slack longer at each end of seat_xy[1]'s post (free along y)."""
    lp = IF["leg_port"]
    (cx, cy), (vx, vy) = lp["seat_xy"]
    sl = lp["seat_vee_slack"]
    return seat_socket(cx, cy, cy, plate_bot_z, plate_top_z, fit) + \
        seat_socket(vx, vy - sl, vy + lp["seat_vee_len"] + sl, plate_bot_z, plate_top_z, fit)


def hook_geometry(plate_bot_z=-4.0):
    """The hook in the leg frame: the deck slot [xi, xo] and the L. The L itself is the
    D020 one (stem hook_lip_t - 0.8 = 2.2 wide, foot hook_foot - 0.55 = 2.95 past the
    stem, 1.8 thick, its top hook_foot_gap under the deck): its stem sits hook_stem_gap off
    the slot's inboard edge, which stayed at -50.1 when the slot was widened OUTBOARD to
    6.2 (2026-10-07; the L moved 0.05 inboard of where D020 drew it)."""
    lp = IF["leg_port"]
    deck_t = 6.0                     # B27 OPEN: params body.deck_t (4.0) is unread
    xi = lp["hook_slot_x"] - lp["hook_slot_w"] / 2
    xo = lp["hook_slot_x"] + lp["hook_slot_w"] / 2
    x0 = xi + lp["hook_stem_gap"]
    x1 = x0 + lp["hook_lip_t"] - 0.8
    ft1 = plate_bot_z - deck_t - lp["hook_foot_gap"]
    return dict(xi=xi, xo=xo, x0=x0, x1=x1, foot_x0=x0 - (lp["hook_foot"] - 0.55),
                foot_x1=x1 - 0.55, foot_z0=ft1 - 1.8, foot_z1=ft1, stem_z0=ft1 - 0.1)


# The L's reach under the deck (z < -10, leg-local), as check_dock measured it 2026-10-07 on
# the real solids. check_dock re-measures it on every run and FAILS if it moves more than
# HOOK_ENVELOPE_TOL; leg_port_hook_envelope() (below) builds the layout's keep-outs from it
# (body_keepouts), and the tub roof may read it directly, so a param change that moves the L
# cannot pass silently (it did before: only z > -15 was asserted). "paths": over check_dock's
# stored dock paths; "any": at any free pose of the base (z is the rock on the pads' inboard
# edge, 17.6 deg). r = R_STATION 110 + x on the station axis: paths r 57.50..65.29, any r
# 56.88..66.61; the L is hook_lip_w wide, so its corners (|y| 12; 12.3 with the unseated
# lip-end play) reach body radius hypot(110 + x, 12) = 67.68 (67.73).
HOOK_ENVELOPE = {"paths": {"z": -13.300, "x": (-52.500, -44.707)},
                 "any": {"z": -13.757, "x": (-53.121, -43.395)}}
HOOK_ENVELOPE_TOL = 0.02
# ... and the params it was measured WITH (2026-10-07, the seats tree d99cdc2). The record is
# only as good as these: the docked-L test in leg_port_hook_envelope sees a param that moves
# the docked L, but not one that only changes where the base can go (a seat's height, the
# slot's outboard edge, the plate extension, a fit), and the 'any' extremes are exactly such
# poses (2026-10-07 layout verifier). Any difference raises (hook_envelope_stale): re-run
# check_dock, re-record both dicts together.
HOOK_ENVELOPE_PARAMS = {
    "leg_port": {"hook_slot_x": -47.0, "hook_slot_w": 6.2, "hook_lip_w": 24.0, "hook_lip_t": 3.0,
                 "hook_foot": 3.5, "hook_foot_gap": 1.5, "hook_stem_gap": 0.55,
                 "plate_ext_half_w": 18.0, "seat_xy": [[-32, 17.3], [-32, -17.3]], "seat_r": 2.9,
                 "seat_half_angle": 30.0, "seat_h": 3.0, "seat_vee_len": 0.0, "seat_vee_slack": 0.3,
                 "seat_roof": 0.6, "seat_mouth": 0.3, "seat_relief": 0.2, "seat_relief_x0": -46.0},
    "print": {"clearance_fit": 0.30, "seat_fit": 0.0}}
# 'any' only: roll (about the leg's x) is not in check_dock's planar search. The 2026-10-07
# layout verifier's 6-DoF search on the real solids (free at >= 0.0002) found the L's foot
# ~0.002 outside this envelope's plan (the yaw ear + tol) at roll 2.06 deg, tilt 0.84, yaw
# -0.13, lifted 0.67: the inboard foot corner at leg (-52.44, 12.386, -12.26), the raw ear
# there 12.364 + 0.02 = 12.384. The built solid held that corner by only 0.0004 (its mitred
# buffer and simplify happen to reach 12.397 there), luck, not a bound: a sampled search can
# miss the true maximum. A flat pad on the whole 'any' plan (box half-width and ear) takes the
# corner to 0.061 inside, ~28x the miss (measured 2026-10-07)
HOOK_ROLL_PAD = 0.05


def hook_envelope_stale():
    """[(key, as recorded, now)] for every param in HOOK_ENVELOPE_PARAMS that differs from
    params.yaml now (a key that vanished reads None). [] = the record still applies."""
    out = []
    for blk, src in (("leg_port", IF["leg_port"]), ("print", PR)):
        for k, rec in HOOK_ENVELOPE_PARAMS[blk].items():
            now = src.get(k)
            a, b = np.asarray(rec, float), None if now is None else np.asarray(now, float)
            if b is None or a.shape != b.shape or not np.allclose(a, b, rtol=0, atol=1e-9):
                out.append((f"{blk}.{k}", rec, now))
    return out


def leg_port_deck_features(deck, top_z, station_tf=None, seats=True):
    """Apply the DECK side of the leg port to `deck` (top surface at z=top_z):
    + the two seat posts (cone + vee), heat-set pockets under the thumbscrew
    stations, the hook through-slot, the 11 mm cable cutout.
    station_tf: transform from leg-local to deck coords (default: identity).
    """
    lp = IF["leg_port"]
    tf = station_tf if station_tf is not None else Pos(0, 0, 0)
    # seat posts (printed with the deck, proud of its top; seats=False leaves them off)
    if seats:
        deck += tf * leg_port_seats(top_z)
    # heat-set pockets for the thumbscrews
    for dx, dy in lp["thumbscrew_xy"]:
        deck -= tf * Pos(dx, dy, top_z - PR["heatset_m3_h"] / 2 + 0.01) * \
            Cylinder(PR["heatset_m3_d"] / 2, PR["heatset_m3_h"])
        deck -= tf * Pos(dx, dy, top_z - 8) * Cylinder(3.4 / 2, 16)  # through
    # hook catch: THROUGH-SLOT at the plate's inboard edge; the plate's
    # downturned lip drops through and its foot hooks under the deck bottom
    # (the deck ends at R100 — there is no deck outboard of the plate, D012).
    # 2026-10-07: 6.2 wide at x -47, so x -50.1..-43.9 (was 4.2 at -48: the L could not pass)
    deck -= tf * Pos(lp["hook_slot_x"], 0, top_z - 10) * \
        Box(lp["hook_slot_w"], lp["hook_lip_w"] + 2 * FIT, 24)
    # cable pass-through (existing deck convention: leg-local x=-29)
    deck -= tf * Pos(CABLE_CUTOUT_X, 0, top_z - 10) * Box(CABLE_CUTOUT_W, CABLE_CUTOUT_W, 24)
    return deck


def leg_port_plate_features(plate, plate_top_z=2.0, plate_bot_z=-4.0, relief_x0=None, fit=None):
    """Apply the LEG side of the port to a coxa base plate solid (part_coxa
    frame: the plate is z -4..0, top at 0, its inboard edge at PLATE_X0): the plate
    extension to the hook's stem, the underside relief, the two seat sockets, thumbscrew
    clearance bores (not captive: no lip is modelled, B82), and the inboard hook's L.
    Pass the actual plate z-extents. relief_x0 overrides params seat_relief_x0 (the port
    coupon prints the designer's -42 beside the repo's -46); fit overrides params
    print.seat_fit, the sockets' radial offset (the port coupons print 0 / 0.1 / 0.2)."""
    lp = IF["leg_port"]
    t = plate_top_z - plate_bot_z
    hg = hook_geometry(plate_bot_z)
    # the plate runs inboard to the hook's stem for |y| <= plate_ext_half_w: over the slot,
    # and over the deck beside it (|y| 12.3..18), whose underside there is the inboard pad
    # pair, the stance fulcrum and docked z datum. Replaces D020's 2 mm reach under the lip.
    ew = lp["plate_ext_half_w"]
    plate += Pos((hg["x0"] + PLATE_X0 + 0.5) / 2, 0, (plate_top_z + plate_bot_z) / 2) * \
        Box(PLATE_X0 + 0.5 - hg["x0"], 2 * ew, t)
    # one layer off the underside outboard of relief_x0 (to past any plate's outboard end):
    # docked, z is set by the two seats and the pads (x < relief_x0), not by the whole face,
    # which sits seat_relief over the deck (J/f2: 0.200)
    rx0 = lp["seat_relief_x0"] if relief_x0 is None else relief_x0
    rel = lp["seat_relief"]
    plate -= Pos((rx0 + 100.0) / 2, 0, plate_bot_z + rel / 2 - 0.01) * \
        Box(100.0 - rx0, 2 * ew + 30.0, rel + 0.02)
    # the seat sockets (blind, from below), grown by the printer's filed seat fit
    plate -= leg_port_sockets(plate_bot_z, plate_top_z, PR["seat_fit"] if fit is None else fit)
    # thumbscrews: a loose Ø3.7 shaft pass. The 'head well' below sits ABOVE
    # the plate (z top..top+8), so on the plate alone it removes nothing; no lip (B82).
    for dx, dy in lp["thumbscrew_xy"]:
        plate -= Pos(dx, dy, (plate_top_z + plate_bot_z) / 2) * \
            Cylinder(3.7 / 2, t + 2)                     # loose shaft pass
        plate -= Pos(dx, dy, plate_top_z + 4.0) * \
            Cylinder((lp["thumbscrew_head_d"] + 1.2) / 2, 8)   # head well
    # the hook's L at the INBOARD edge: the down-turn (stem) from the plate top through the
    # 6 deck (B27), then a foot pointing further inboard (-x) under the deck bottom. The
    # reach is the plate extension above. A catch for a hanging leg, not a clamp
    lipw = lp["hook_lip_w"]
    lip = Pos((hg["x0"] + hg["x1"]) / 2, 0, (hg["stem_z0"] + plate_top_z) / 2) * \
        Box(hg["x1"] - hg["x0"], lipw, plate_top_z - hg["stem_z0"])    # down-turn
    lip += Pos((hg["foot_x0"] + hg["foot_x1"]) / 2, 0, (hg["foot_z0"] + hg["foot_z1"]) / 2) * \
        Box(hg["foot_x1"] - hg["foot_x0"], lipw, hg["foot_z1"] - hg["foot_z0"])   # foot
    return plate + lip


# ====================================================================== under-deck keep-outs
# 2026-10-07 (the body layout, option A, the owner's picks): what hangs under each leg port and
# must stay clear of the keel tub (part_bay), the hub shelf (part_busboard), the stand's cradle
# (part_stand) and the deck's own holes (part_deck.DECK_HOLES). Built ONCE here in the leg frame
# and posed by station_tf, so those parts cannot each carry their own copy (the prep rounds did:
# docs/archive/prep-2026-10-01, keepouts.py). Body frame z = leg z; the deck spans z -10..-4.
_LEGS = P["robot"]["legs"]
STATIONS = [_LEGS["first_station_deg"] + i * 360.0 / _LEGS["count"]
            for i in range(_LEGS["count"])]     # 90, 162, 234, 306, 378 (not wrapped, like pebble.xml)
R_STATION = P["body"]["circumradius"]           # pentagon centre -> leg station (110)
DECK_BOT_Z = -10.0                              # the deck's underside, leg / body z (6 deck, B27)
HOOK_HALF_W = IF["leg_port"]["hook_lip_w"] / 2 + FIT   # 12.3: the 24 lip's play along its 24.6 slot
DROP_HALF = CABLE_CUTOUT_W / 2 + 5.0            # 10.5: the cutout + 5 mm round it (the proposal's
                                                # +-10.5 IS that; fact 3's bands reproduce only so)
DROP_DEPTH = 35.0                               # under the deck bottom: the mated XT30 (20.1) + its
                                                # wire bend (8) hang 28.1, the XH-5 pair 21.2 (ESTIMATE)


def station_tf(i):
    """Leg i's frame -> the body frame: the yaw axis on the station, +x radially outboard, z
    unchanged. The deck, the jig and the check modules pose a leg port with exactly this."""
    return Rot(0, 0, STATIONS[i]) * Pos(R_STATION, 0, 0)


def _hook_yaw_ear(step_deg=0.25):
    """Plan region (leg x, y) of the L's foot when the stem yaws and slides inside the deck slot
    (tilt 0), as a shapely polygon. For a yaw psi the stem's free translations are a box (its
    rotated bbox inside the slot), so the foot covers the hull of its rotated copy moved to that
    box's four corners; the union runs over every feasible psi (|psi| <= 9.6 at slot 6.2). The
    stored dock paths and check_dock's envelope are planar (x, z): this is the y they leave out.
    Measured 2026-10-07: |y| 12.79 at x -49.0, yaw 9.6 deg (the planar L stops at 12.3)."""
    from shapely.geometry import box as sbox, MultiPoint
    from shapely import affinity
    from shapely.ops import unary_union
    hg = hook_geometry()
    hw = IF["leg_port"]["hook_lip_w"] / 2
    o = ((hg["x0"] + hg["x1"]) / 2, 0.0)
    stem = sbox(hg["x0"], -hw, hg["x1"], hw)
    foot = sbox(hg["foot_x0"], -hw, hg["foot_x1"], hw)
    out = []
    for psi in np.arange(-20.0, 20.0 + 1e-9, step_deg):
        sx0, sy0, sx1, sy1 = affinity.rotate(stem, psi, origin=o).bounds
        dx = (hg["xi"] - sx0, hg["xo"] - sx1)
        dy = (-HOOK_HALF_W - sy0, HOOK_HALF_W - sy1)
        if dx[0] > dx[1] or dy[0] > dy[1]:
            continue                                    # the stem no longer fits the slot
        f = affinity.rotate(foot, psi, origin=o)
        out.append(MultiPoint([p for a in dx for b in dy
                               for p in affinity.translate(f, a, b).exterior.coords]).convex_hull)
    return unary_union(out)


def leg_port_hook_envelope(mode="any"):
    """The volume the hook's L can sweep UNDER the deck (z <= -10), leg frame: a keep-out for
    anything hung from the deck near a port. A conservative hull, not the exact swept set:
    the box of check_dock's measured extremes (iface.HOOK_ENVELOPE, which check_dock re-measures
    on the real solids and holds to +-HOOK_ENVELOPE_TOL), grown by that tolerance, +-HOOK_HALF_W
    in y, from the deck bottom down; 'any' adds the yaw ear (_hook_yaw_ear), full height, and
    HOOK_ROLL_PAD on its whole plan (roll).

      'path'  the L along check_dock's four stored dock paths (tilt 7-10 deg, a radial approach,
              lowered onto the cones). The exact region (2026-10-07, the union of the L's
              sections along the densified paths): x -52.50..-44.71, z -13.30 (docked), 20.75
              mm^2 in section; this box 26.01. Planar: the paths neither yaw nor roll.
      'any'   the L at ANY free pose of the hooked base (check_dock's tilt -3..40 x slide box,
              refined; the deepest is the rock on the pads' inboard edge, 17.6 deg): x
              -53.12..-43.39, z -13.757; its grid region's hull is 32.02 mm^2 in section, this
              box 36.89. The yaw ear takes |y| 12.30 -> 12.79 at x -49 (12.30 at -53 and -46,
              linear between; tilt 0: tilted, the stem fills more of the slot and yaws less),
              inside the box's x span. Roll is covered too: the layout verifier's 6-DoF search
              (roll 2.06 deg, see HOOK_ROLL_PAD) put the foot ~0.002 past the ear, so the whole
              plan is grown 0.05 (box |y| 12.35, the ear 12.86 at x -49).
              The yaw ear is an UPPER BOUND the seats do not allow: it is the stem yawing 9.6
              deg in the slot at tilt 0 with the plate free, but at tilt 0 a lift dz frees only
              about dz tan 30 at each cone (17.3 off the port axis), and the posts leave their
              sockets only when lifted ~3, when the foot hangs at most ~0.3 under the deck. It
              is extruded the envelope's full height anyway. It is what brings the shelf posts
              (+-50, 34) to 0.34 of this envelope (0.59 to the planar box; 0.39 before the pad)

    Measured on these solids (2026-10-07, before HOOK_ROLL_PAD): 'path' x -52.520..-44.687, y
    +-12.300, z -13.320..-10.000 (r 57.480..65.313 on the station axis), 639.7 mm^3; 'any' x
    -53.141..-43.375, y +-12.807, z -13.777..-10.000 (r 56.859..66.625), 921.6 mm^3. The prep
    rounds' r 56.94..66.79 (BODY_LAYOUT correction 3) was their seats model, not this tree (the
    second I1 verifier measured 56.88..66.61 on it). Raises if the record no longer holds the
    docked L, or if any param it was recorded with has changed (HOOK_ENVELOPE_PARAMS): re-run
    check_dock and re-record both."""
    rec = HOOK_ENVELOPE["paths" if mode == "path" else "any"] if mode in ("path", "any") else None
    if rec is None:
        raise ValueError(f"mode {mode!r}: 'path' or 'any'")
    moved = hook_envelope_stale()
    if moved:
        raise RuntimeError("iface.HOOK_ENVELOPE was recorded with other params: " + "; ".join(
            f"{k} {a} -> {b}" for k, a, b in moved) + ": run check_dock.py and re-record "
            "HOOK_ENVELOPE + HOOK_ENVELOPE_PARAMS")
    hg, tol = hook_geometry(), HOOK_ENVELOPE_TOL
    stale = (hg["foot_z0"] < rec["z"] - tol or hg["foot_x0"] < rec["x"][0] - tol or
             hg["x1"] > rec["x"][1] + tol)
    if stale:
        raise RuntimeError(f"iface.HOOK_ENVELOPE['{mode}'] no longer holds the docked L (x "
                           f"{hg['foot_x0']:.2f}..{hg['x1']:.2f}, z {hg['foot_z0']:.2f}): run "
                           f"check_dock.py and re-record it")
    pad = HOOK_ROLL_PAD if mode == "any" else 0.0
    x0, x1, z0 = rec["x"][0] - tol, rec["x"][1] + tol, rec["z"] - tol
    h = DECK_BOT_Z - z0
    env = Pos((x0 + x1) / 2, 0, DECK_BOT_Z - h / 2) * Box(x1 - x0, 2 * (HOOK_HALF_W + pad), h)
    if mode == "any":
        ear = _hook_yaw_ear()
        if ear.geom_type != "Polygon":                  # overlapping hulls: one polygon, but be safe
            ear = ear.convex_hull
        # simplify cuts <= 0.005, the buffer grows tol + the roll pad
        ear = ear.simplify(0.005).buffer(tol + pad, join_style=2)
        pts = [(float(x), float(y)) for x, y in list(ear.exterior.coords)[:-1]]
        env += Pos(0, 0, z0) * extrude(make_face(Polyline(*pts, close=True)), h)
    return env


def leg_drop_keepout(extra=0.0):
    """The mated leg drop (XT30 pair + JST-XH-5 on its header) hanging under the I1 cable cutout,
    leg frame: a leg-aligned box 2 x (DROP_HALF + extra) square about (CABLE_CUTOUT_X, 0), from
    the deck bottom (z -10) down DROP_DEPTH + extra; `extra` grows plan and depth, never up
    (above -10 is the deck and its cutout). As the prep's keepouts.drop_keepout: the 10.5 is
    already the cutout's 5.5 + 5, so extra 5 on top (+-15.5) is the stricter 'drops + 5' the
    prep measured the tub (0 mm^3) and the shelf (its plate 0.57 off) against. B120's cross
    arm (7 x 16 on the cutout) sits inside it (2.50); legs 2/3 reach their own box (B136)."""
    h, d = DROP_HALF + extra, DROP_DEPTH + extra
    return Pos(CABLE_CUTOUT_X, 0, DECK_BOT_Z - d / 2) * Box(2 * h, 2 * h, d)


def body_keepouts(extra=0.0, mode="any"):
    """The five ports' under-deck keep-outs, posed in the body frame: {'hooks': [the hook
    envelope (mode) at stations 0..4], 'drops': [the drop (+extra) at 0..4]}. The tub, the
    shelf, the cradle and the deck's hole audit all intersect against these same solids."""
    hook, drop = leg_port_hook_envelope(mode), leg_drop_keepout(extra)
    n = len(STATIONS)
    return {"hooks": [station_tf(i) * hook for i in range(n)],
            "drops": [station_tf(i) * drop for i in range(n)]}


def bay_tub_extent():
    """The keel tub (I5, option A, B84) in the body frame, from params alone: {'x', 'y', 'z'}
    its outer box (door included: the 54.8 x 37.4 door covers the whole end), {'x_in', 'y_in',
    'z_in'} the interior, each (lo, hi). x is pinned at the DOOR end by battery_sled.bay_door_x
    (bay_l moves the nose, pick 4); y is centred on bay_centre[1] (the pack's y, the clear band);
    z hangs from bay_top_z (pick 2). A box: the nose's two vertical edges are chamfered
    (nose_chamfer), the latch boss, the hanger bosses and the door's hook tab are part_bay's. The
    sim's belly takes these same sums from params (it does not import CAD). At bay_l 194: x
    -97.5..101.9, y -47.4..7.4, z -55.4..-18.0, as BODY_LAYOUT_PROPOSAL s2 measured the tub."""
    bs = IF["battery_sled"]
    xi0 = bs["bay_door_x"] + bs["door_t"]
    xi1 = xi0 + bs["bay_l"]
    yc, wi = bs["bay_centre"][1], bs["bay_w"] / 2
    zi1 = bs["bay_top_z"] - bs["bay_roof"]
    zi0 = zi1 - bs["bay_h"]
    return {"x": (bs["bay_door_x"], xi1 + bs["bay_wall"]), "x_in": (xi0, xi1),
            "y": (yc - wi - bs["bay_wall"], yc + wi + bs["bay_wall"]), "y_in": (yc - wi, yc + wi),
            "z": (zi0 - bs["bay_floor"], bs["bay_top_z"]), "z_in": (zi0, zi1)}


# ====================================================================== I3
# B87 (D063): the cartridge is a bayonet. As first drawn the rotor's pegs had no
# way into the housing's closed track, the rotor ended flush with the housing
# bottom (0 mm^3 below it, 0.75 clear of the strike), the strike was blind with no
# undercut, and with a sector down no coin reached the head. Now the rotor drops
# through the housing (two keyways pass its lugs; turned off them it is captive,
# head above, lugs below), hangs LATCH_REACH below the panel's seating face, passes
# the frame's entry slots at OPEN, and a quarter turn cams its lugs under the land
# in the frame's underside recess (latch_strike). The operating end is the rotor's
# BOTTOM: a flat-screwdriver slot 0.2 above the frame's underside, reached from
# below with the panel on. Not a coin: the slot floor is 1.7 up the Ø6.6 bore, and a
# Ø16-20 coin gets only ~0.5 of the slot's 1.5 before it meets the frame mid-turn
# (1.0 in: 0.9-1.5 mm^3 at 30-60 deg, measured). Frame: z 0 = the seating plane (the
# housing's bottom, flush with the panel face that sits on the frame), +z into the panel.
LATCH_SHAFT_D = 6.0
LUG_R, LUG_W, LUG_H = 5.5, 3.0, 2.0          # lug reach, tangential width, height
LATCH_HEAD_D, LATCH_HEAD_H = 10.0, 2.0      # 2.0 overhang on the shaft (printed head up)
LATCH_SLOT = (2.0, 1.5)           # the operating slot, width x depth: a flat screwdriver, <= 5 wide
LATCH_BITE = 0.2                  # cam preload at LOCKED: the lug top 0.2 into the land. VERIFY
LATCH_GAP = IF["panel_latch"]["rotor_cam_rise"] - LATCH_BITE   # axial play at OPEN (0.6)
LATCH_LAND = 3.2                  # frame top -> the recess ceiling at the entry
LATCH_REACH = LATCH_LAND + LATCH_GAP + LUG_H  # rotor bottom below the seating plane (5.8)
LATCH_ENTRY_DEG = 45.0            # the entry line (lugs at OPEN) from the frame's +x: at a
                                  # deck strike +x is radial, and 45 keeps the recess off the edge
LATCH_RAMP_DEG = 50.0             # the ceiling ramps down over 0..50 deg, dwells to 90, stops
LATCH_RECESS_R = 6.1              # the lug's corner (5.70) + FIT, rounded up
# B107: the keyway index. The housing is a D (a flat LATCH_FLAT_R from its axis, square
# to the keyways) and every panel's pocket is the same D, 0.15 bigger all round, so it
# goes in one way: its keyways LATCH_KEYWAY_DEG from the entry line. On a strike the
# rotor turns 0..90 between the recess's stops (latch_strike), so at 135 / 315 the
# keyways sit outside that travel: the lugs pass them only at 132..138 deg, 42 past LOCKED
# and 42 back past OPEN (part_panel.latch_index). No angle the latch is worked at, or comes
# off at (OPEN), lines the lugs up with them. Before, the housing was round and glued at
# any angle: keyways at OPEN would drop a rotor out of every panel lifted off. The index
# is a place, not a stop: off the robot a rotor turned 45 deg back past OPEN still
# reaches them (nothing holds its 3.8 mm of axial play without a strike under it).
LATCH_KEYWAY_DEG = 135.0
LATCH_FLAT_R = 6.0                # leaves 2.7 of wall to the Ø6.6 bore at the flat
# the carapace latch station (D030): each sector's latch pad over the deck's strike.
# Worked from under the deck, so nothing may hang under a strike: station 162's sits
# at body (-73.5, -12.3), inside the keel tub's plan (bay_tub_extent: x -97.5..101.9, y
# -47.4..7.4 at bay_l 194; B84, B100), as do station 306's and the tray's (B126), and the tub must
# leave those columns open (a Ø5 driver + its hand) or that sector cannot come off
SHELL_LATCH_AZ, SHELL_LATCH_R = 27.5, 74.5


def latch_lug(grow=0.0, z0=0.0, z1=LUG_H, x0=2.0):
    """One lug along +x (x0 inside the shaft, out to LUG_R), grown by `grow` on its
    free faces, spanning z0..z1."""
    return Pos((x0 + LUG_R + grow) / 2, 0, (z0 + z1) / 2) * \
        Box(LUG_R + grow - x0, LUG_W + 2 * grow, z1 - z0)


def latch_d_profile(r, flat, z0, z1):
    """The I3 index shape (B107): a cylinder of radius r cut flat at y = flat (housing
    frame: keyways along x, the flat on +y), spanning z0..z1."""
    return Pos(0, 0, (z0 + z1) / 2) * Cylinder(r, z1 - z0) & \
        Pos(0, (flat - r - 1) / 2, (z0 + z1) / 2) * Box(2 * r + 2, flat + r + 1, z1 - z0 + 2)


def latch_insert_housing():
    """The replaceable latch cartridge's OUTER: Ø14 x 6, glued into the panel's
    Ø14.3 pocket with its bottom flush with the seating face. A Ø6.6 bearing bore
    and two keyways that pass the rotor's lugs at assembly only; the flat on its
    side is the keyway index (B107): it fits the pocket's flat one way."""
    pl = IF["panel_latch"]
    t = pl["housing_t"]
    h = latch_d_profile(pl["housing_d"] / 2, LATCH_FLAT_R, 0.0, t)
    h -= Pos(0, 0, t / 2) * Cylinder(LATCH_SHAFT_D / 2 + FIT, t + 2)
    for s in (0, 180):          # keyways: 1.2 of wall left to the Ø14 skin
        h -= Rot(0, 0, s) * latch_lug(FIT, -1.0, t + 1.0)
    return h


def latch_housing_tf():
    """The housing in its pocket, in the latch frame (z 0 = the seating plane, +x the
    strike's +x): its keyways on the index, LATCH_KEYWAY_DEG from the entry line."""
    return Rot(0, 0, LATCH_ENTRY_DEG + LATCH_KEYWAY_DEG)


def latch_pocket(z0, z1):
    """Cutter for a panel's housing pocket (B107), in the latch frame, spanning z0..z1:
    the Ø housing_pocket_d with the index flat, both FIT/2 off the housing's, turned as
    latch_housing_tf. A panel with no strike yet (the belly door, the tray tongue) takes
    its own +x as the strike's: draw the strike to match."""
    pl = IF["panel_latch"]
    gap = (pl["housing_pocket_d"] - pl["housing_d"]) / 2
    return latch_housing_tf() * latch_d_profile(pl["housing_pocket_d"] / 2, LATCH_FLAT_R + gap, z0, z1)


def latch_insert_rotor():
    """The rotor, seated (head on the housing top) with its lugs along +x: a Ø6
    shaft through the housing, two lugs with their tops LATCH_LAND + LATCH_GAP below
    the seating plane, and the operating slot across its bottom end, square to the
    lugs (it shows where they point)."""
    t = IF["panel_latch"]["housing_t"]
    z0 = -LATCH_REACH
    r = Pos(0, 0, (z0 + t) / 2) * Cylinder(LATCH_SHAFT_D / 2, t - z0)
    r += Pos(0, 0, t + LATCH_HEAD_H / 2) * Cylinder(LATCH_HEAD_D / 2, LATCH_HEAD_H)
    for s in (0, 180):
        r += Rot(0, 0, s) * latch_lug(0.0, z0, z0 + LUG_H)
    r -= Pos(0, 0, z0 + LATCH_SLOT[1] / 2 - 0.01) * \
        Box(LATCH_SLOT[0], LATCH_SHAFT_D + 2, LATCH_SLOT[1])            # the slot
    return r


def latch_strike(frame_t):
    """Cutter for the FRAME side of I3 (deck, coupon): z 0 = the frame's top (the
    seating plane), its underside at -frame_t (>= LATCH_REACH + 0.2). A bore for the
    shaft, two entry slots through the land on the LATCH_ENTRY_DEG line, and an
    underside recess that is the lugs' swept path over the quarter turn (CCW seen
    from above): its ceiling ramps down rotor_cam_rise over 0..LATCH_RAMP_DEG, from
    LATCH_GAP above the lug tops to LATCH_BITE into them, dwells to 90 and stops."""
    # a thinner frame lets the rotor's slot end hang out of it (the deck's 6 is B27 OPEN)
    assert frame_t >= LATCH_REACH + 0.2 - 1e-6, f"I3 strike needs a {LATCH_REACH + 0.2:.1f} frame"
    rise = IF["panel_latch"]["rotor_cam_rise"]
    ceil_lock = -LATCH_LAND - rise
    bot = -frame_t - 1.0
    cut = Pos(0, 0, -frame_t / 2) * Cylinder(LATCH_SHAFT_D / 2 + FIT, frame_t + 2)
    w = LUG_W / 2 + FIT
    for s in (0, 180):
        a = LATCH_ENTRY_DEG + s
        cut += Rot(0, 0, a) * latch_lug(FIT, bot, 1.0)                        # entry slot
        # the swept path's plan: the quarter disc + the lug's half-widths at both ends
        disc = Pos(0, 0, (bot + ceil_lock) / 2) * Cylinder(LATCH_RECESS_R, ceil_lock - bot)
        path = Pos(LATCH_RECESS_R / 2, LATCH_RECESS_R / 2 - w / 2, (bot + ceil_lock) / 2) * \
            Box(LATCH_RECESS_R, LATCH_RECESS_R + w, ceil_lock - bot)
        path += Pos(LATCH_RECESS_R / 2 - w / 2, LATCH_RECESS_R / 2, (bot + ceil_lock) / 2) * \
            Box(LATCH_RECESS_R + w, LATCH_RECESS_R, ceil_lock - bot)
        cut += Rot(0, 0, a) * (disc & path)
        # the ramp: higher ceilings where the lug passes early in the turn (5 deg steps)
        n = int(LATCH_RAMP_DEG // 5)
        for k in range(n):
            phi = LATCH_RAMP_DEG * k / n
            c = -LATCH_LAND - rise * phi / LATCH_RAMP_DEG
            cut += Rot(0, 0, a + phi) * latch_lug(FIT, ceil_lock - 0.01, c)
    return cut


def magnet_pocket(solid, at_tf, from_below=False):
    pl = IF["panel_latch"]
    d = pl["magnet_d"] + 0.25
    t = pl["magnet_t"] + 0.2
    return solid - at_tf * Cylinder(d / 2, t)


# ====================================================================== I6
def _trapezoid_bar(w_root, w_face, d, L):
    """Trapezoid in YZ (w_root wide on z 0, w_face wide at z d), run along x."""
    with BuildPart() as bp:
        with BuildSketch(Plane.YZ):
            with BuildLine():
                Polyline((-w_root / 2, 0), (w_root / 2, 0), (w_face / 2, d),
                         (-w_face / 2, d), (-w_root / 2, 0))
            make_face()
        extrude(amount=L / 2, both=True)
    return bp.part


def dovetail_male(length=None, undercut=False):
    """Male bar, root on z 0, depth +z, runs along x.

    undercut=True is the I6 accessory profile (B83): the frozen 12 / 8 turned
    over, the 8 neck at the root and the 12 face outboard, so a shoe cannot
    lift off and its set screw wedges it. As first drawn the 12 sat at the root:
    a KEY, off which a shoe lifts with 0 mm^3 of clash at any lift, so no set
    screw could clamp it. undercut=False keeps that key for the stand's spigot
    joints (part_stand), where a closed skirt round five facets holds it."""
    dv = IF["dovetail"]
    b, c = dv["base_w"], dv["crest_w"]
    root, face = (c, b) if undercut else (b, c)
    return _trapezoid_bar(root, face, dv["depth"], length or dv["segment_len"])


# I6 set knob (B83): a thumb_knob_m3 on a 45-deg spot face at the shoe's +y upper
# edge. Its bore aims at the male's +y flank at mid-depth; the tip meets the male
# (first at the face's edge) and pushes it toward the mouth, so the undercut flanks
# wedge the shoe. 45 deg keeps the Ø12 knob 1.7 above the shoe's bottom face and
# clear of the whisker_shoe's wires (they leave its top at y +/-4).
SET_KNOB_TILT = 45.0
SET_KNOB_STANDOFF = 5.5          # spot face centre from the flank point, along the bore


def set_knob_axis():
    """(the flank point M, the spot-face centre E, the bore's outward unit u) in the
    shoe frame (x along the slot, y across, z up from the mouth)."""
    dv = IF["dovetail"]
    n, w, d = dv["crest_w"], dv["base_w"], dv["depth"]
    zm = d / 2
    M = np.array([0.0, n / 2 + (w - n) / 2 * zm / d, zm])
    b = np.deg2rad(SET_KNOB_TILT)
    u = np.array([0.0, np.cos(b), np.sin(b)])
    return M, M + SET_KNOB_STANDOFF * u, u


def set_knob_tf():
    """Shoe frame -> the knob's frame (knob base on the spot face, z out along the bore)."""
    _, E, _ = set_knob_axis()
    return Pos(*E) * Rot(SET_KNOB_TILT - 90, 0, 0)


def dovetail_female_shoe(length=None, body_h=10.0):
    """Female I6 shoe blank: the undercut slot (the male offset by FIT) + the set
    knob's tapped bore and spot face (B83; the knob is part_panel.thumb_knob_m3)."""
    dv = IF["dovetail"]
    n, w, d = dv["crest_w"], dv["base_w"], dv["depth"]
    L = (length or dv["segment_len"]) + 2
    blk = Pos(0, 0, body_h / 2) * Box(L, dv["base_w"] + 2 * FIT + 8, body_h)
    dg = d + FIT
    k = (w - n) / 2 / d                               # flank run per mm of depth
    r0 = n / 2 + FIT - 0.5 * k                        # half-width 0.5 below the mouth
    r1 = n / 2 + FIT + k * dg                         # at the slot roof
    with BuildPart() as cut:
        with BuildSketch(Plane.YZ):
            with BuildLine():
                Polyline((-r0, -0.5), (r0, -0.5), (r1, dg), (-r1, dg), (-r0, -0.5))
            make_face()
        extrude(amount=L / 2 + 2, both=True)
    blk -= cut.part
    tf = set_knob_tf()
    blk -= tf * Pos(0, 0, 15) * Cylinder(IF["leg_port"]["thumbscrew_head_d"] / 2 + FIT, 30)  # spot face
    blk -= tf * Cylinder(PR["screw_m3_tap"] / 2, 2 * (SET_KNOB_STANDOFF + 1))   # tapped bore
    return blk
