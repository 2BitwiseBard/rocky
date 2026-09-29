"""Body deck, v0.4 — the pentagon closes the SHELL loop (I3 strikes).

v0.2 -> v0.3: the per-station M3 hole pairs are replaced by the leg-port
interface from iface.py (dowel POSTS, thumbscrew heat-set pockets, hook
through-slot, cable cutout) — the same code path the bench jig uses, so a
leg moves between jig and robot without re-fitting. The v0.2 mid-pair holes
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

Deck frame = BODY frame (origin center). Deck TOP surface = leg-frame z -4
(coxa plates are 4 mm; servo bases sit at leg z 0). Stations at
robot.legs first_station_deg + i*360/count (90 + 72i), radius
body.circumradius. The 6 mm thickness is B27 (OPEN: params body.deck_t says 4).
Part is modeled z 0..T (print flat, dowel posts up); assembly -10..-4.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import leg_port_deck_features, IF, latch_strike, SHELL_LATCH_AZ, SHELL_LATCH_R

P = params()
PR = P["print"]
R_DECK = 100.0
T = 6.0                                   # B27 OPEN: params body.deck_t (4.0) is unread
_LEGS = P["robot"]["legs"]
STATIONS = [_LEGS["first_station_deg"] + i * 360.0 / _LEGS["count"]
            for i in range(_LEGS["count"])]      # 90, 162, 234, 306, 378 (not wrapped, like pebble.xml)
R_STATION = P["body"]["circumradius"]     # pentagon center -> leg station (110)
STRAP_SLOTS = [(sx, sy) for sx in (-26, 26) for sy in (-22, 22)]   # 5 x 30 through-slots
STRAP_SLOT_WH = (5.0, 30.0)
GROMMET_XY, GROMMET_D = (52.0, -6.0), 9.0                          # power entry
ZIP_ANCHORS = [(44.0, -14.0), (60.0, -14.0)]


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
    feats = ([("dowel", xy) for xy in lp["dowel_xy"]] +
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
                       ("deck x coxa base (docked: dowels in bores, lip in slot)",
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
    bb = d.bounding_box()
    bed_x, bed_y = PR["bed_mm"][:2]
    fits = bb.size.X <= bed_x and bb.size.Y <= bed_y
    bad += not fits
    print(f"deck bbox: {bb.size.X:.0f} x {bb.size.Y:.0f} mm "
          f"({f'FITS bed {bed_x:.0f}x{bed_y:.0f}' if fits else 'TOO BIG'})")
    raise SystemExit(1 if bad else 0)
