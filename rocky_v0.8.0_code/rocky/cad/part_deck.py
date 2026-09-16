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
    latch pad): rotor bore + radial peg-entry slots; a quarter-turn cams
    the pegs against the 2 mm land left at the deck bottom.
  * 5 WASHER RECESSES (Ø8 x 1.4, glue an M3 washer) at az = station
    -25.5°, r 76 — under each sector's foot magnet. (The v0 shell wanted
    three magnets per web at r 77.5; their recesses would breach the
    pentagon edge at the az±34 spots — shell amended to one per web.)
  * power-entry grommet Ø9 at (52, -6) + two zip anchors (BUS_STARBOARD
    proposal adopted); the star-board bracket bolts to existing grid
    holes (20,-20)/(40,-20) — nothing new needed there.

Deck frame = BODY frame (origin center). Deck TOP surface = leg-frame z -4
(coxa plates are 4 mm; servo bases sit at leg z 0). Stations at 90 + 72i deg.
Part is modeled z 0..T (print flat, dowel posts up); assembly -10..-4.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import leg_port_deck_features, IF

P = params()
PR = P["print"]
R_DECK = 100.0
T = 6.0
STATIONS = [90 + 72*i for i in range(5)]

def _station_xy(ang_deg, leg_x, leg_y, rb=110.0):
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
        tf = Rot(0, 0, ang) * Pos(110, 0, 0)
        deck = leg_port_deck_features(deck, top_z=T, station_tf=tf)
        # station numbering (backlog B10): k+1 engraved dots by each port —
        # font-free, readable with a headlamp, survives every slicer
        for d in range(k + 1):
            deck -= tf * Pos(-36 + 5 * d, -24, T - 0.5) * Cylinder(1.5, 1.2)
    # leg-0 = "north" arrow (heading is software, but humans need a datum)
    deck -= Pos(0, 30, T - 0.5) * extrude(Triangle(a=10, b=10, c=10), 1.2)
    # battery straps: two parallel straps along +X at y = +/-22 (slots at x = +/-26)
    for sx in (-26, 26):
        for sy in (-22, 22):
            deck -= Pos(sx, sy, T/2) * Box(5, 30, T+2)
    # electronics mounting grid: M3 thread-forming holes, 20 mm pitch, r < 56,
    # skipping strap-slot neighborhoods
    for gx in range(-40, 41, 20):
        for gy in range(-40, 41, 20):
            if np.hypot(gx, gy) < 56 and not (abs(abs(gx)-26) < 7 and abs(abs(gy)-22) < 19):
                deck -= Pos(gx, gy, T/2) * Cylinder(PR["screw_m3_tap"]/2, T+2)

    # ---- v0.4: shell attachment (I3) — the sectors' deck-side answers ----
    FIT = PR["clearance_fit"]
    for ang in STATIONS:
        # latch strike under the sector's latch pad (az +27.5, r 74.5):
        # rotor bore + two TANGENTIAL peg-entry slots cut from the TOP
        # (radial slots breached the pentagon edge by 0.9 mm — the audit
        # caught it; pegs insert tangential, quarter-turn to radial, and
        # cam against the 2 mm land left at the deck bottom)
        stf = Rot(0, 0, ang + 27.5) * Pos(74.5, 0, 0)
        deck -= stf * Pos(0, 0, T / 2) * Cylinder(9.7 / 2 + FIT, T + 2)
        for s in (90, 270):
            deck -= stf * Rot(0, 0, s) * Pos(5.5, 0, T / 2 + 1.0) * \
                Box(3.4, 3.4, T - 2.0)
        # washer recess under the sector's foot magnet (az -25.5, r 76):
        # Ø8 x 1.4 pocket in the top face, M3 washer glued in
        deck -= Rot(0, 0, ang - 25.5) * Pos(76.0, 0, T - 0.7 + 0.01) * \
            Cylinder(4.0, 1.4)
    # power-entry grommet + zip anchors (BUS_STARBOARD.md proposal)
    deck -= Pos(52, -6, T / 2) * Cylinder(4.5, T + 2)
    for zx, zy in ((44, -14), (60, -14)):
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
    # v0.4 shell-interface features: strike (bore Ø9.7 + tangential slots —
    # radial footprint is just the bore), washer (Ø8)
    for name, az_off, r, feat_r in (("latch strike", 27.5, 74.5, 5.2),
                                    ("washer recess", -25.5, 76.0, 4.0)):
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
    b0 = Rot(0, 0, STATIONS[0]) * Pos(110, 0, 0) * coxa_yaw_base()
    b1 = Rot(0, 0, STATIONS[1]) * Pos(110, 0, 0) * coxa_yaw_base()
    for name, pair in [("adjacent coxa bases", b0 & b1),
                       ("deck x coxa base (docked: dowels in bores, lip in slot)",
                        d_posed & b0)]:
        v = 0.0 if pair is None else pair.volume
        print(f"{name} intersection: {v:.2f} mm^3 ({'OK' if v < 1 else 'CLASH'})")
    bb = d.bounding_box()
    print(f"deck bbox: {bb.size.X:.0f} x {bb.size.Y:.0f} mm "
          f"({'FITS Prusa i3 250x210' if bb.size.X <= 250 and bb.size.Y <= 210 else 'TOO BIG'})")
