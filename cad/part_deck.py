"""Body deck, v0.4 — the pentagon closes the SHELL loop (I3 strikes).

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
  * power-entry grommet Ø9 at (52, -6) + two zip anchors (star-board
    proposal in docs/WIRING_HARNESS.md, adopted); the star-board bracket bolts to existing grid
    holes (0,20)/(0,40) — nothing new needed there (part_busboard's layout
    audit ray-tests both against this deck).

2026-10-07 (the body layout, option A, the owner's picks): DECK_HOLES is the explicit hole
table the tray rails, the keel tub, the hub shelf and the tray latch build to, with its audit
(deck_hole_audit); __main__ also places iface's under-deck keep-outs (body_keepouts) and
checks them. The table is NOT cut yet: body_deck() still carries the v0.4 grid, strap slots
and power grommet until the deck agent swaps them.

Deck frame = BODY frame (origin center). Deck TOP surface = leg-frame z -4
(coxa plates are 4 mm; servo bases sit at leg z 0). Stations at
robot.legs first_station_deg + i*360/count (90 + 72i), radius
body.circumradius. The 6 mm thickness is B27 (OPEN: params body.deck_t says 4).
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
STRAP_SLOTS = [(sx, sy) for sx in (-26, 26) for sy in (-22, 22)]   # 5 x 30 through-slots
STRAP_SLOT_WH = (5.0, 30.0)
GROMMET_XY, GROMMET_D = (52.0, -6.0), 9.0                          # power entry
ZIP_ANCHORS = [(44.0, -14.0), (60.0, -14.0)]

# ---- option A's deck holes: the CONTRACT the body layout builds to (2026-10-07) -------------
# docs/BODY_LAYOUT_PROPOSAL.md s3 'Deck holes (A)' with its corrections 10 (the shelf posts) and
# 15 (the (0, 50) slot), on the owner's picks (option A, the 8 mm layer, the tray at (0, 0)
# turned 180 with its tabs inboard, the three boards on the hub shelf). The tray rails, the
# tub's lid bosses, the shelf's posts and the tray's latch bolt to THESE. NOT cut into
# body_deck() yet: the deck agent swaps the grid holes, the strap slots, the (52, -6) grommet
# and its anchors for this table next (B50, B51, B84, B122); deck_hole_audit() already checks
# it against the deck as it will be. Of today's 13 grid holes 6 are reused, 6 stay unused,
# (0, -40) goes (it overlaps the strike); 9 features are new.
#   (name, (x, y) body, kind, size, job). kind / size:
#     m3_top       (Ø,)           through, an M3 driven from the deck top (its head on top)
#     zip          (Ø,)           through, a zip-tie anchor (no head)
#     tap          (Ø,)           through, thread-forming: unused grid holes, as today
#     m3_below     (Ø, depth)     BLIND from the underside, thread-forming: the top skin stays
#     slot         (w, h, r)      through, w along x, rounded corners r
#     strike       (frame deg,)   iface.latch_strike(T), its frame turned this far about z
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
    [("tray_strike", (0.0, -43.0), "strike", (180.0,),
      "the tray's I3 latch strike: the tongue's frame turned with the tray (tray_rot_deg); its "
      "rotor is worked from below, and that driver column crosses the tub's plan (B126: the tub "
      "leaves it open)")] +
    [(f"grid_{x:+d}_{y:+d}", (float(x), float(y)), "tap", _TAP,
      "unused grid hole: a trim-cup point" + (" (was the star-board bracket's)" if x == 0 and y > 0
                                                else ""))
     for x, y in ((0, 0), (-20, 0), (20, 0), (0, -20), (0, 20), (0, 40))])
# what the table retires from today's body_deck(): (0, -40) (it overlaps the strike), the four
# strap slots (B50: the deck-strap fallback is gone), the (52, -6) grommet and its two anchors
# (the power comes from under the deck)
DECK_RETIRED = ["grid (0, -40)", "strap slots (+-26, +-22)", f"grommet {GROMMET_XY}",
                f"zip anchors {ZIP_ANCHORS}"]
DECK_HOLE_GAP, DECK_HOLE_GAP_PREF = 2.0, 3.3     # to every other cut / an I1 feature set; the
DECK_HOLE_EDGE = 2.0                             # table's new holes keep 3.3 from each other
HEAD_D, HEAD_CLEAR = 6.0, 0.5                    # an M3 head + driver from above vs the docked bases
TUB_BOSS_D = 8.0                                 # the bay_lid's hanger bosses (params bay_roof; part_bay's)


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
        return Pos(x, y, T) * Rot(0, 0, size[0]) * latch_strike(T)
    raise ValueError(kind)


def deck_hole_audit(bases=None):
    """The DECK_HOLES table against the deck as the body layout leaves it: [(check, ok, detail)].
    Every cutter >= DECK_HOLE_EDGE inside the pentagon; >= DECK_HOLE_GAP (3.3 preferred) from
    every other entry and every cut body_deck() keeps (the five shell latch strikes, washer
    recesses, station dots, the north arrow) and from the five I1 feature sets (the leg port's
    own cuts + its seat posts); each m3_top head column (Ø6, deck top to +10) >= HEAD_CLEAR from
    the docked coxa bases (`bases`: posed coxa_yaw_base solids in the deck's model frame, or None
    to skip); the strike where the tray's tongue is; the posts where params put them."""
    out = []
    cut = {e[0]: deck_hole_cutter(e) for e in DECK_HOLES}
    # what body_deck() keeps besides the leg ports (the strap slots, grommet, its anchors and the
    # grid holes are the table's to replace, so they are not 'other cuts'). These four are COPIES
    # of body_deck's cutters, kept in step by hand until the deck agent cuts the table and both
    # read one helper
    kept = []
    for k, ang in enumerate(STATIONS):
        kept.append(latch_strike_tf(ang) * latch_strike(T))
        kept.append(Rot(0, 0, ang - 25.5) * Pos(76.0, 0, T - 0.7 + 0.01) * Cylinder(4.0, 1.4))
        kept += [station_tf(k) * Pos(-36 + 5 * d, -24, T - 0.5) * Cylinder(1.5, 1.2)
                 for d in range(k + 1)]
    kept.append(Pos(0, 30, T - 0.5) * extrude(Triangle(a=10, b=10, c=10), 1.2))
    kept = Compound(children=kept)
    # the five I1 feature sets: what leg_port_deck_features removes from a slab round the port,
    # plus the seat posts it adds (B117)
    ports = []
    for i in range(len(STATIONS)):
        tf = station_tf(i)
        slab = tf * Pos(-40.0, 0, T / 2) * Box(60.0, 60.0, T)
        ports.append(slab - leg_port_deck_features(slab, T, tf, seats=False))
        ports.append(tf * leg_port_seats(T))
    ports = Compound(children=ports)
    a = np.deg2rad(36)
    inset = Pos(0, 0, -5) * extrude(RegularPolygon((R_DECK * np.cos(a) - DECK_HOLE_EDGE) / np.cos(a), 5,
                                                   major_radius=True, rotation=90), T + 10)
    rim = Pos(0, 0, -5) * Box(400, 400, T + 10) - \
        Pos(0, 0, -5) * extrude(RegularPolygon(R_DECK, 5, major_radius=True, rotation=90), T + 10)
    worst = {"edge": (np.inf, ""), "cut": (np.inf, ""), "port": (np.inf, ""), "head": (np.inf, "")}
    pref = []
    for name, xy, kind, size, job in DECK_HOLES:
        c = cut[name]
        out_v = (c - inset)
        out_v = 0.0 if out_v is None else out_v.volume
        edge = c.distance_to(rim)
        others = Compound(children=[cut[n] for n in cut if n != name] + [kept])
        g_cut, g_port = c.distance_to(others), c.distance_to(ports)
        for key, v in (("edge", edge if out_v < 1e-6 else -out_v), ("cut", g_cut), ("port", g_port)):
            if v < worst[key][0]:
                worst[key] = (v, name)
        if g_cut < DECK_HOLE_GAP_PREF:
            pref.append(f"{name} {g_cut:.2f}")
        if kind == "m3_top" and bases is not None:
            head = Pos(xy[0], xy[1], T + 5.0) * Cylinder(HEAD_D / 2, 10.0)
            g = min(head.distance_to(b) for b in bases)
            if g < worst["head"][0]:
                worst["head"] = (g, name)
    out.append((f"every hole >= {DECK_HOLE_EDGE} inside the pentagon", worst["edge"][0] >= DECK_HOLE_EDGE - 1e-6,
                f"tightest {worst['edge'][1]} {worst['edge'][0]:.2f}"))
    out.append((f"every hole >= {DECK_HOLE_GAP} from every other cut (the table + the strikes, washers, "
                f"dots, arrow body_deck keeps)", worst["cut"][0] >= DECK_HOLE_GAP - 1e-6,
                f"tightest {worst['cut'][1]} {worst['cut'][0]:.2f}; under {DECK_HOLE_GAP_PREF}: "
                + (", ".join(pref) if pref else "none")))
    out.append((f"every hole >= {DECK_HOLE_GAP} from the five I1 feature sets (cuts + seat posts)",
                worst["port"][0] >= DECK_HOLE_GAP - 1e-6, f"tightest {worst['port'][1]} {worst['port'][0]:.2f}"))
    if bases is not None:
        out.append((f"every m3_top head column (Ø{HEAD_D:.0f}, deck top up) >= {HEAD_CLEAR} from the docked "
                    f"coxa bases", worst["head"][0] >= HEAD_CLEAR, f"tightest {worst['head'][1]} "
                    f"{worst['head'][0]:.2f}"))
    # the strike sits under the tongue: the tray's latch frame (part_avionics.tongue_latch_tf,
    # tray-local) posed by params tray_xy / tray_rot_deg
    from part_avionics import tongue_latch_tf
    tray = Pos(*_AT["tray_xy"], 0) * Rot(0, 0, _AT["tray_rot_deg"]) * tongue_latch_tf()
    st = next(e for e in DECK_HOLES if e[2] == "strike")
    p, xd = tray.position, (tray * Pos(1, 0, 0)).position - tray.position
    d_xy = float(np.hypot(p.X - st[1][0], p.Y - st[1][1]))
    d_ang = abs((np.rad2deg(np.arctan2(xd.Y, xd.X)) - st[3][0] + 180) % 360 - 180)
    out.append(("the tray strike under the tongue's latch (params tray pose x part_avionics.tongue_latch_tf)",
                d_xy < 0.01 and d_ang < 0.01, f"off {d_xy:.3f} mm, {d_ang:.3f} deg"))
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


def grid_holes():
    """Electronics grid: M3 thread-forming holes, 20 mm pitch, r < 56,
    skipping the strap-slot neighbourhoods."""
    return [(gx, gy) for gx in range(-40, 41, 20) for gy in range(-40, 41, 20)
            if np.hypot(gx, gy) < 56 and not (abs(abs(gx) - 26) < 7 and abs(abs(gy) - 22) < 19)]


def latch_strike_tf(ang):
    """The I3 strike under station ang's sector latch pad, in the deck's model
    frame: z 0 of the strike at the deck top (T), its +x radial."""
    return Rot(0, 0, ang + SHELL_LATCH_AZ) * Pos(SHELL_LATCH_R, 0, T)


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
    for k, ang in enumerate(STATIONS):
        tf = Rot(0, 0, ang) * Pos(R_STATION, 0, 0)
        deck = leg_port_deck_features(deck, top_z=T, station_tf=tf)
        # station numbering (backlog B10): k+1 engraved dots by each port —
        # font-free, readable with a headlamp, survives every slicer
        for d in range(k + 1):
            deck -= tf * Pos(-36 + 5 * d, -24, T - 0.5) * Cylinder(1.5, 1.2)
    # leg-0 = "north" arrow (heading is software, but humans need a datum)
    deck -= Pos(0, 30, T - 0.5) * extrude(Triangle(a=10, b=10, c=10), 1.2)
    # battery straps: two parallel straps along +X at y = +/-22 (slots at x = +/-26)
    for sx, sy in STRAP_SLOTS:
        deck -= Pos(sx, sy, T/2) * Box(*STRAP_SLOT_WH, T+2)
    # electronics mounting grid: M3 thread-forming holes, 20 mm pitch, r < 56,
    # skipping strap-slot neighborhoods
    for gx, gy in grid_holes():
        deck -= Pos(gx, gy, T/2) * Cylinder(PR["screw_m3_tap"]/2, T+2)

    # ---- v0.4: shell attachment (I3) — the sectors' deck-side answers ----
    for ang in STATIONS:
        # latch strike under the sector's latch pad (az +27.5, r 74.5): the
        # I3 bayonet's frame side, iface.latch_strike (B87, D063): the bore,
        # two entry slots through a 3.2 land, and the underside recess the
        # rotor's lugs cam up into. The old strike was blind from the top with
        # a 2 mm land and no undercut: pegs in it hit 7.95 mm^3 of deck at 18 deg.
        deck -= latch_strike_tf(ang) * latch_strike(T)
        # washer recess under the sector's foot magnet (az -25.5, r 76):
        # Ø8 x 1.4 pocket in the top face, M3 washer glued in
        deck -= Rot(0, 0, ang - 25.5) * Pos(76.0, 0, T - 0.7 + 0.01) * \
            Cylinder(4.0, 1.4)
    # power-entry grommet + zip anchors (star-board proposal, docs/WIRING_HARNESS.md)
    deck -= Pos(*GROMMET_XY, T / 2) * Cylinder(GROMMET_D / 2, T + 2)
    for zx, zy in ZIP_ANCHORS:
        deck -= Pos(zx, zy, T / 2) * Cylinder(1.7, T + 2)
    return deck

if __name__ == "__main__":
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
    for name, pair in [("adjacent coxa bases", b0 & b1),
                       ("deck x coxa base (docked: seats in sockets, lip in slot)",
                        d_posed & b0)]:
        v = 0.0 if pair is None else pair.volume
        bad += v >= 1
        print(f"{name} intersection: {v:.2f} mm^3 ({'OK' if v < 1 else 'CLASH'})")
    # I3 (B87): the sector's latch cartridge on this deck's strike (station 0)
    from part_panel import latch_engagement, latch_verdict, latch_report
    m = latch_engagement(d, latch_strike_tf(STATIONS[0]))
    bad += len(latch_verdict(m))
    print(f"I3 latch on the deck strike: {latch_report(m)} "
          f"({'OK' if not latch_verdict(m) else 'FAIL: ' + '; '.join(latch_verdict(m))})")
    # ---- 2026-10-07: option A's hole table (the contract; not cut yet) + the under-deck keep-outs
    base = coxa_yaw_base()
    bases_model = [Pos(0, 0, T + 4.0) * station_tf(i) * base for i in range(len(STATIONS))]
    print(f"DECK_HOLES ({len(DECK_HOLES)} entries; retires {', '.join(DECK_RETIRED)}):")
    for name, ok, detail in deck_hole_audit(bases_model):
        bad += not ok
        print(f"  {name}: {'OK' if ok else 'FAIL'}" + (f" ({detail})" if detail else ""))
    # smoke test of iface's keep-outs: placed at the stations, below the deck, and the shelf
    # posts (Ø post_d from the deck bottom to the plate top) clear of them
    from iface import body_keepouts, leg_port_hook_envelope, DECK_BOT_Z
    ko = body_keepouts()
    for tag, s in (("hook envelope ('any') at station 0", ko["hooks"][0]),
                   ("drop keep-out at station 0", ko["drops"][0])):
        sb = s.bounding_box()
        print(f"  {tag}: body x {sb.min.X:.2f}..{sb.max.X:.2f}, y {sb.min.Y:.2f}..{sb.max.Y:.2f}, "
              f"z {sb.min.Z:.2f}..{sb.max.Z:.2f}")
    lb = leg_port_hook_envelope("any").bounding_box()
    v_hd = sum(0.0 if (d_posed & h) is None else (d_posed & h).volume for h in ko["hooks"])
    ok = v_hd < 1e-3 and abs(lb.max.Z - DECK_BOT_Z) < 1e-6
    bad += not ok
    print(f"  posed deck x the five hook envelopes: {v_hd:.4f} mm^3, envelope top z {lb.max.Z:.2f} "
          f"(the sweep lives below the deck: {'OK' if ok else 'FAIL'})")
    hs = IF["hub_shelf"]
    cols = [Pos(px, py, (DECK_BOT_Z + hs["plate_top_z"]) / 2) * Cylinder(hs["post_d"] / 2, DECK_BOT_Z - hs["plate_top_z"])
            for px, py in hs["posts"]]
    for tag, group in (("hook envelopes", ko["hooks"]), ("drops", ko["drops"]),
                       ("drops + 5", body_keepouts(5.0)["drops"])):
        g = min(c.distance_to(k) for c in cols for k in group)
        v = sum(0.0 if (c & k) is None else (c & k).volume for c in cols for k in group)
        bad += v > 1e-3
        print(f"  shelf posts x {tag}: {v:.4f} mm^3, closest {g:.3f} ({'OK' if v <= 1e-3 else 'CLASH'})")
    bb = d.bounding_box()
    bed_x, bed_y = PR["bed_mm"][:2]
    fits = bb.size.X <= bed_x and bb.size.Y <= bed_y
    bad += not fits
    print(f"deck bbox: {bb.size.X:.0f} x {bb.size.Y:.0f} mm "
          f"({f'FITS bed {bed_x:.0f}x{bed_y:.0f}' if fits else 'TOO BIG'})")
    raise SystemExit(1 if bad else 0)
