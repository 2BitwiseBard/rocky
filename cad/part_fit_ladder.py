"""Fit ladder — the FIRST print after the Prusa refurb. Measures the printer.

Every clearance in params.yaml (fit 0.30, press 0.15, M3 tap 2.8, heat-set
4.6) is a guess calibrated to "old Prusa, generally". Tonight's refurbed
machine is a new animal. This one 25-minute plate answers, with actual
plastic: which dowel bore slides, which M3 hole threads, which pocket grips
an insert — and session 5 writes the MEASURED values back into params and
regenerates everything.

One plate, five labeled rows (tally dots above each feature, font-free):
  row E: (v2, session 6) the HARDWARE fits the v1 ladder missed —
         Ø2 hinge-pin holes at 2.00 / 2.10 / 2.20 (Ø2 filament = pin stock,
         testable TONIGHT with no ordered parts), 683ZZ bearing PRESS
         pockets at 6.85 / 6.95 / 7.05 x 3.4 deep (coxa crown seat), and
         Ø10 carbon-tube sockets at 10.15 / 10.30 / 10.45 (tibia clamp,
         SEA outer, hand boss all ride this fit). Bearing/tube columns
         re-tested when Batch 0/1 arrives — the coupon keeps.
  row A: Ø4 dowel BORES at 4.15 / 4.20 / 4.30 / 4.40 / 4.50
         (+ three Ø3.90 / 3.95 / 4.00 PEGS on the plate corner to test with)
  row B: M3 thread-forming holes at 2.5 / 2.6 / 2.7 / 2.8 / 2.9
  row C: M3 heat-set pockets at 4.4 / 4.6 / 4.8 (x 5.7 deep)
  row D: slot ladder 2.2-lip test: slots 3.8 / 4.2 / 4.6 wide x 26
         (drop the hook-lip edge of the port coupon through each; v2 plate
         growth encloses the slots fully — truer to the deck's real
         through-slot than v1's open-edged notches)

Record in NOTES_INBOX: first bore the peg SLIDES into without force, first
M3 hole a screw forms clean threads in, which pocket holds an insert snug,
first hole Ø2 filament slides through snug-not-forced, and (when hardware
lands) which pocket the bearing presses into by thumb + which socket the
tube seats in.
"""
from build123d import *
from common import params, export

P = params()

PLATE_W, PLATE_H, T = 118, 90, 4.0

DOWEL_BORES = [4.15, 4.20, 4.30, 4.40, 4.50]
M3_TAPS = [2.5, 2.6, 2.7, 2.8, 2.9]
HEATSETS = [4.4, 4.6, 4.8]
SLOTS = [3.8, 4.2, 4.6]
PEGS = [3.90, 3.95, 4.00]
# v2 row E (session 6): fits the v1 ladder missed
PIN_HOLES = [2.00, 2.10, 2.20]         # hand hinge pins (Ø2 filament stock)
BEARING_POCKETS = [6.85, 6.95, 7.05]   # 683ZZ od 7.0 press seat, 3.4 deep
TUBE_BORES = [10.15, 10.30, 10.45]     # Ø10 carbon tube sockets


def _dots(plate, x, y, n):
    for d in range(n):
        plate -= Pos(x + 3.2 * d, y, T - 0.5) * Cylinder(0.9, 1.2)
    return plate


def fit_ladder():
    plate = Pos(0, 0, T / 2) * Box(PLATE_W, PLATE_H, T)
    x0 = -PLATE_W / 2 + 14
    # row E (v2): pin holes, bearing press pockets, tube sockets (top row)
    for i, d in enumerate(PIN_HOLES):
        plate -= Pos(x0 - 4 + i * 8, 38, T / 2) * Cylinder(d / 2, T + 2)
        plate = _dots(plate, x0 - 4 + i * 8 - 2.4, 43, i + 1)
    for i, d in enumerate(BEARING_POCKETS):
        # blind press pocket from the top face, 3.4 deep (bearing w 3.0)
        plate -= Pos(x0 + 24 + i * 13, 38, T - 3.4 / 2 + 0.01) * \
            Cylinder(d / 2, 3.4)
        plate = _dots(plate, x0 + 24 + i * 13 - 3.2, 43, i + 1)
    for i, d in enumerate(TUBE_BORES):
        # dots BELOW these (the Ø10.45 bore leaves no room above at the edge)
        plate -= Pos(x0 + 62 + i * 16, 38, T / 2) * Cylinder(d / 2, T + 2)
        plate = _dots(plate, x0 + 62 + i * 16 - 3.2, 28.5, i + 1)
    # row A: dowel bores
    for i, d in enumerate(DOWEL_BORES):
        plate -= Pos(x0 + i * 18, 16, T / 2) * Cylinder(d / 2, T + 2)
        plate = _dots(plate, x0 + i * 18 - 4.8, 23, i + 1)
    # row B: M3 thread-forming
    for i, d in enumerate(M3_TAPS):
        plate -= Pos(x0 + i * 18, 4, T / 2) * Cylinder(d / 2, T + 2)
        plate = _dots(plate, x0 + i * 18 - 4.8, 10.5, i + 1)
    # row C: heat-set pockets (blind, from top, 5.7 deep -> through 4 mm plate
    # is fine for the test: melt from the top face)
    for i, d in enumerate(HEATSETS):
        plate -= Pos(x0 + i * 18, -12, T / 2) * Cylinder(d / 2, T + 2)
        plate = _dots(plate, x0 + i * 18 - 3.2, -5.5, i + 1)
    # row D: slot ladder (v2: dropped to y -29 — at -24 the slot tops
    # merged with the Ø4.8 heat-set pocket, breaking its wall open; the v1
    # coupon shipped with that pocket reading falsely loose. Caught by the
    # section audit, session 6.)
    for i, w in enumerate(SLOTS):
        plate -= Pos(x0 + 8 + i * 30, -29, T / 2) * Box(w, 26, T + 2)
        # v2.1 (session 8): the dots stayed at y -17 when the slots dropped
        # to -29 (top edge -16) — they were engraved INSIDE the slot mouth
        # (invisible) and two sat tangent to slot walls, leaving a
        # non-manifold seam in the STL. Now beside each slot, to the right.
        plate = _dots(plate, x0 + 8 + i * 30 + w / 2 + 2.5, -20, i + 1)
    # pegs on the right end (vertical prints, same as the deck's dowel posts)
    for i, d in enumerate(PEGS):
        px = PLATE_W / 2 - 12
        py = 18 - i * 16
        plate += Pos(px, py, T + 4) * Cylinder((d - 0.05) / 2, 8)
        plate += Pos(px, py, T + 8.6) * Cone((d - 0.05) / 2, (d - 0.05) / 2 - 0.8, 1.2)
        plate = _dots(plate, px - 3.2, py - 6.5, i + 1)
    return plate


if __name__ == "__main__":
    # layout audit: every feature (+ its dots) must live ON the plate with
    # ≥1.5 mm edge margin — catches the row-shuffle class of bug before plastic
    x0 = -PLATE_W / 2 + 14
    feats = ([(x0 - 4 + i * 8, 38, d / 2) for i, d in enumerate(PIN_HOLES)] +
             [(x0 + 24 + i * 13, 38, d / 2) for i, d in enumerate(BEARING_POCKETS)] +
             [(x0 + 62 + i * 16, 38, d / 2) for i, d in enumerate(TUBE_BORES)] +
             [(x0 + i * 18, 16, d / 2) for i, d in enumerate(DOWEL_BORES)] +
             [(x0 + i * 18, 4, d / 2) for i, d in enumerate(M3_TAPS)] +
             [(x0 + i * 18, -12, d / 2) for i, d in enumerate(HEATSETS)])
    bad = [(x, y, r) for x, y, r in feats
           if abs(x) + r > PLATE_W / 2 - 1.5 or abs(y) + r > PLATE_H / 2 - 1.5]
    assert not bad, f"feature(s) breach the plate edge: {bad}"
    # min-wall audit: every bore must keep >=1.0 mm of plastic to every other
    # feature INCLUDING the slots (the v1 heat-set/slot merge class of bug)
    slots = [(x0 + 8 + i * 30, -29, w / 2, 13.0) for i, w in enumerate(SLOTS)]
    worst = 1e9
    for i in range(len(feats)):
        x1, y1, r1 = feats[i]
        for x2, y2, r2 in feats[i + 1:]:
            worst = min(worst, ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5 - r1 - r2)
        for sx, sy, shw, shh in slots:          # circle-to-rectangle wall
            dx = max(abs(x1 - sx) - shw, 0.0)
            dy = max(abs(y1 - sy) - shh, 0.0)
            worst = min(worst, (dx * dx + dy * dy) ** 0.5 - r1)
    assert worst >= 1.0, f"feature walls too thin: min {worst:.2f} mm"
    print(f"  layout audit: {len(feats)} features on-plate, >=1.5 mm edge "
          f"margin, min feature-to-feature wall {worst:.2f} mm")

    p = fit_ladder()
    export(p, "fit_ladder")
    bb = p.bounding_box()
    print(f"fit ladder v2: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.1f} "
          f"— PLA, 0.2 mm, 3 walls, 25% infill, ~35 min")
    print("MEASURE + file in NOTES_INBOX: sliding dowel bore, best M3 tap, "
          "snug heat-set pocket, slot the 2.2 lip passes, snug Ø2 pin hole "
          "(+ bearing press pocket and tube socket when hardware lands)")
