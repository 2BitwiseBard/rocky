"""Carapace shell sectors v0 — the creature pass (D006, B8, B9, I3, I6).

Five IDENTICAL bolt-on sectors (radial symmetry: print one STL 5x) + a top
hatch cap. The mechanism underneath stays pure (D006); everything here is
cosmetic-and-services: rock look, LED channel, venting, accessory ring.

Anatomy (body frame, deck TOP at z = -4 per the frozen conventions):
  * terraced ROCK silhouette: four tiers of perturbed soft-pentagon rings
    (per-vertex seeded noise, pentagon corners at the leg stations exactly
    like the deck) — the stepped-boulder look from the v0.2 preview, now
    printable and hollow (2.8 mm walls, 2.6 mm tier roofs).
  * five LEG ARCHES: trapezoid-prism openings through tiers 1–3 sized from
    the real sweep geometry — the coxa fork+yaw hardware lives inside a
    r=26 disc about the yaw axis (yaw rotation NEVER leaves +/-14 deg of
    the station azimuth; the femur exits above tier 1), plus the coxa
    plate slab passing through the skirt at deck level. Tier 4 stays a
    continuous ring — the "neck" the dome caps.
  * skirt drops to z -9: covers the deck edge from outside (turtle
    overbite), clearing the deck (deck corners r=100 < skirt inner face).
  * I6 dovetail ring: one VERTICAL male segment per sector web at +/-27
    deg (slide a shoe down from above, gravity seats it, set-knob locks) —
    placed flush against the queried perturbed wall radius.
  * B8 LED channel: revolved groove around tier 2 (8.4 mm tall, ~2.4 deep,
    45 deg chamfered top so it prints upright without support), fed from
    an I5 XT30 spare tap through the cable notch.
  * B9 vent gills: three angled slots per side through the tier-1 wall,
    outside the arch, over the leg bays (servo airflow).
  * attachment (I3): flange ring seats on the deck top with ONE
    quarter-turn latch insert pocket per sector (az 0, r 75) + TWO magnet
    pockets (az +/-28); deck-side strikes land in deck v0.4 (deck v0.3
    predates this part — noted in TODO).
  * top hatch: rocky cap, plug matches the seat outline with print
    clearance, five seat magnets (one per sector, az 0 — the only pattern
    compatible with 5 identical sectors).

Checks (run this file): fork-swing keep-out, port-knob keep-out, coxa
plate slab, deck slab — all must intersect at 0.00 mm^3; bed-fit report.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import IF, dovetail_male

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
PL = IF["panel_latch"]

DECK_TOP = -4.0
N_VERT = 40                      # outline vertices (chunky facets = rock);
                                 # divisible by 5: the noise TILES at 72 deg
WALL = 2.8
ROOF = 2.6

# tiers: (z0, z1, nominal r, noise amp, noise seed, twist deg)
TIERS = [(-9.0, 16.0, 112.0, 4.5, 11, 0.0),
         (16.0, 30.0, 100.0, 5.0, 22, 36.0),
         (30.0, 42.0, 88.0, 5.0, 33, 12.0),
         (42.0, 52.0, 74.0, 4.0, 44, 24.0)]
HATCH_R = 46.0
HATCH_SEED = 55


def _outline_r(theta, r0, amp, seed, twist_deg=0.0, pent_k=0.55):
    """Soft-pentagon + seeded rock noise, radius at azimuth theta (rad).

    NOTE the pentagon factor SHRINKS mid-edge radii (~0.89x at the web) —
    every radial placement near a wall must query this, never assume r0.

    The noise is 72-DEG PERIODIC (one 8-vertex pattern tiled 5x, smoothed
    circularly): five IDENTICAL printed sectors then meet each other with
    FLUSH walls at every seam. Non-periodic noise gave every seam a
    radius step — invisible in the ring render (disjoint wedges can't
    interfere) but fatal for the tongue-and-groove edge joint."""
    th = np.atleast_1d(theta) - np.deg2rad(twist_deg)
    d = (np.rad2deg(th) % 72.0) - 36.0
    pent = (np.cos(np.deg2rad(36)) / np.cos(np.deg2rad(d))) ** pent_k
    rng = np.random.default_rng(seed)
    raw = np.tile(rng.normal(0, 1.0, N_VERT // 5), 5)
    for _ in range(2):                       # circular smoothing, keeps facets
        raw = 0.5 * raw + 0.25 * np.roll(raw, 1) + 0.25 * np.roll(raw, -1)
    raw = raw / (np.abs(raw).max() + 1e-9)
    k = (th / (2 * np.pi) * N_VERT) % N_VERT
    k0 = np.floor(k).astype(int) % N_VERT
    k1 = (k0 + 1) % N_VERT
    f = k - np.floor(k)
    noise = raw[k0] * (1 - f) + raw[k1] * f
    r = r0 * pent * (1 + noise * amp / r0)
    return r if r.shape[0] > 1 else float(r[0])


def _ring(z0, z1, r0, amp, seed, twist=0.0, dr=0.0, pent_k=0.55):
    th = np.linspace(0, 2 * np.pi, N_VERT, endpoint=False)
    r = _outline_r(th, r0, amp, seed, twist, pent_k) + dr
    pts = [(float(ri * np.cos(t)), float(ri * np.sin(t)))
           for ri, t in zip(r, th)]
    with BuildPart() as bp:
        with BuildSketch(Plane.XY.offset(z0)):
            with BuildLine():
                Polyline(*pts, close=True)
            make_face()
        extrude(amount=z1 - z0)
    return bp.part


def _az_wedge(az0_deg, az1_deg, z0=-15.0, z1=90.0, R=400.0):
    a0, a1 = np.deg2rad(az0_deg), np.deg2rad(az1_deg)
    mid = (a0 + a1) / 2
    pts = [(0.0, 0.0),
           (R * np.cos(a0), R * np.sin(a0)),
           (R * np.cos(mid), R * np.sin(mid)),
           (R * np.cos(a1), R * np.sin(a1))]
    with BuildPart() as bp:
        with BuildSketch(Plane.XY.offset(z0)):
            with BuildLine():
                Polyline(*pts, close=True)
            make_face()
        extrude(amount=z1 - z0)
    return bp.part


def _mother():
    """The full 360-deg carapace with az-0-station features (the wedge cut
    at +/-36 deg then yields one sector; all five are this same solid)."""
    body = None
    for (z0, z1, r, amp, seed, tw) in TIERS:
        t = _ring(z0, z1, r, amp, seed, tw)
        body = t if body is None else body + t
    # hollow: parallel inner stack (same seeds -> constant wall). FLANGE
    # FEET COME AFTER THIS — v0's first build added the flange first and
    # the cavity silently ate it, latch pockets and all (found by boolean
    # probes, not by the render: Poly3D shading hides interiors).
    # v0.1 (session 8, D038 printability audit): v0's cavities were one
    # solid prism per tier, so every tier roof was a FULL DISC — the shell
    # was four sealed chambers separated by 2.4 mm floors spanning ~80–100
    # mm: unprintable bridges over cavities no support could be pulled
    # from, and no route for the LED feed to reach the deck. Now: one
    # connected cavity. Each tier's ceiling is a 45° cone stepping in from
    # its wall to the next tier's inner wall (self-supporting upright), and
    # a throat in the next tier's inner outline joins the chambers.
    cavity = None
    for k, (z0, z1, r, amp, seed, tw) in enumerate(TIERS):
        if k + 1 == len(TIERS):
            c = _ring(z0 - 0.2, z1 - ROOF, r, amp, seed, tw, dr=-WALL)
        else:
            r_next = TIERS[k + 1][2]
            depth = r - r_next                       # terrace width ≈ 12–14
            z_cone0 = z1 - ROOF - depth              # where the 45° ceiling starts
            c = None
            if z_cone0 > z0 - 0.2 + 0.5:
                c = _ring(z0 - 0.2, z_cone0 + 0.05, r, amp, seed, tw, dr=-WALL)
            steps = int(np.ceil(depth))
            for j in range(steps):                   # 1 mm slabs, 45° ceiling
                d0 = depth - j                        # depth below the roof
                z_lo = z1 - ROOF - d0
                slab = _ring(z_lo, z_lo + 1.0 + 0.05, r, amp, seed, tw,
                             dr=-WALL - (depth - d0))
                c = slab if c is None else c + slab
            # throat through the roof, in the next tier's inner outline
            nz0, nz1, nr, namp, nseed, ntw = TIERS[k + 1]
            c += _ring(z1 - ROOF - 0.2, nz0 + 0.2, nr, namp, nseed, ntw,
                       dr=-WALL)
        cavity = c if cavity is None else cavity + c
    body -= cavity

    # ---- web FEET (true annulus, no pentagon factor): r 73..82 at az
    # 21..36 both sides of every station — OUTSIDE the coxa plate (y +/-22
    # +margin) and the knob columns (edge az 18.6 deg), ON the deck (edge
    # apothem 80.9 at the web). The latch insert (dia 14.3) outgrows the
    # 9 mm annulus, so it gets a local round PAD reaching inboard where
    # the y-clearance is generous.
    # v0.1 (session 8): the v0 foot was a bare r 73..82 annulus — at the web
    # the tier-1 wall sits at r ~97, so every foot (latch pad, magnet pocket
    # and all) FLOATED 13 mm inboard of the skirt with nothing joining it;
    # the sector exported as three bodies (D036 class). The flange is back:
    # the foot's outer boundary is now the tier-1 outline itself, offset to
    # reach 1.0 mm INTO the wall (a fused overlap, not a tangent face).
    t1 = TIERS[0]
    foot_ring = _ring(DECK_TOP, 0.0, t1[2], t1[3], t1[4], t1[5],
                      dr=-WALL + 1.0) - \
        _ring(DECK_TOP - 1, 1.0, 73.0, 0.0, 1, pent_k=0.0)
    for a0, a1 in ((21.0, 36.0), (-36.0, -21.0)):
        body += foot_ring & _az_wedge(a0, a1)
    body += Rot(0, 0, 27.5) * Pos(74.5, 0, (DECK_TOP + 0.0) / 2) * \
        Cylinder(9.0, -DECK_TOP)                      # latch pad

    # ---- leg arch (station az 0): through tiers 1-3 + the skirt
    with BuildPart() as arch:
        with BuildSketch(Plane.YZ.offset(72.0)):
            with BuildLine():
                Polyline((-40, -9.5), (40, -9.5), (40, 30), (22, 46),
                         (-22, 46), (-40, 30), close=True)
            make_face()
        extrude(amount=135.0 - 72.0)
    body -= arch.part

    # ---- B9 vent gills: 3 angled slots per side through the tier-1 wall
    # (session 8, D038 audit: the 22° tilt swings a gill ±1.7° across its
    # height — the outer gill at 34° reached 35.7°, 0.5 mm from the seam
    # face. Pitch 4.5° -> 3.5°: outer gill 32°, worst reach 33.7°, ≥3.8 mm
    # of wall to the seam; 2.6 mm between gills.)
    for sgn in (+1, -1):
        for k in range(3):
            az = sgn * (25.0 + 3.5 * k)
            r_wall = float(_outline_r(np.deg2rad(az), *TIERS[0][2:5],
                                      TIERS[0][5]))
            body -= Rot(0, 0, az) * Pos(r_wall - 1.0, 0, 4.5) * \
                Rot(22, 0, 0) * Box(26, 3.2, 15)

    # ---- B8 LED channel: 2.4-deep band FOLLOWING the tier-2 rock wall
    # (a circular revolve missed the wall at the webs — pentagon shrink)
    t2 = TIERS[1]
    # session 8 (D038): groove depth 2.4 -> 1.4 — a 2.4 groove in a 2.8 wall
    # left a 0.4 skin; where the LED band crosses the seam tongue and the
    # arch corner nothing else backs it. 1.4 deep still seats a 10 mm
    # COB strip proud by ~0.6 for diffusion.
    groove = _ring(19.2, 27.6, t2[2], t2[3], t2[4], t2[5], dr=+8.0) - \
        _ring(19.0, 27.8, t2[2], t2[3], t2[4], t2[5], dr=-1.4)
    body -= groove
    # v0.1 (session 8, D038 printability audit): a 2.4-deep groove in a
    # 2.8 wall left 0.4 mm of skin behind the LED strip — one perimeter.
    # An inner backing band (2.0 mm, following the same outline) takes the
    # skin to 2.4; it is cut by the arch again so it can't refill the leg
    # opening. Keep-out audit below still applies to it.
    backing = _ring(18.8, 28.0, t2[2], t2[3], t2[4], t2[5], dr=-WALL + 0.5) - \
        _ring(18.6, 28.2, t2[2], t2[3], t2[4], t2[5], dr=-WALL - 2.0)
    body += backing
    body -= arch.part
    # LED feed notch through the tier-2 wall into the cavity (az 33 web)
    # session 8 (D038): was az 33 — an 8 mm notch there reaches az 35.5,
    # 0.8 mm short of the 36° seam plane (one perimeter, with the seam
    # tongue hanging off it). At az 30.5 the notch spans 28..33: 4.7 mm of
    # wall to the seam, and it still lands on the web between arches.
    body -= Rot(0, 0, 30.5) * Pos(90, 0, 21) * Box(18, 8, 9)

    # ---- I6 vertical dovetail bars on the tier-1 wall at the webs
    for az in (+27.0, -27.0):
        r_wall = float(_outline_r(np.deg2rad(az), *TIERS[0][2:5], TIERS[0][5]))
        body += Rot(0, 0, az) * Pos(r_wall - 1.4, 0, 3.0) * Rot(0, 90, 0) * \
            dovetail_male(length=16.0)

    # ---- I3 attachment on the feet: one latch (pad, az +27.5) + ONE
    # magnet (az -25.5, r 76) — each web joint = neighbor A's latch +
    # neighbor B's magnet. v0 had three magnets per web at r 77.5; their
    # deck-side washer recesses (Ø8-10) would breach the pentagon edge at
    # the az±34 positions (boundary ~81 there) — deck v0.4 taught the
    # shell where the deck actually ends.
    body -= Rot(0, 0, 27.5) * Pos(74.5, 0, (DECK_TOP + 0.0) / 2) * \
        Cylinder(PL["housing_pocket_d"] / 2, 6)
    body -= Rot(0, 0, -25.5) * Pos(76.0, 0, DECK_TOP +
                                   (PL["magnet_t"] + 0.2) / 2 - 0.01) * \
        Cylinder((PL["magnet_d"] + 0.25) / 2, PL["magnet_t"] + 0.2)

    # ---- hatch: opening + seat rebate + one seat magnet per sector (az 0)
    body -= _ring(41.0, 53.5, HATCH_R, 2.0, HATCH_SEED)
    body -= _ring(49.6, 52.4, HATCH_R + 4.5, 2.0, HATCH_SEED)
    body -= Pos(HATCH_R + 1.0, 0, 49.6 - (PL["magnet_t"] + 0.2) / 2 + 0.01) * \
        Cylinder((PL["magnet_d"] + 0.25) / 2, PL["magnet_t"] + 0.2)
    return body


def shell_sector(edge_joint=True):
    """One printable sector. edge_joint adds the seam registration: a
    vertical TONGUE on the +36 deg edge and a matching GROOVE on the -36
    edge — five identical sectors chain tongue-into-groove around the
    ring (drop-in vertical engagement, radial+tangential registration,
    stiffer seams). Periodic outline noise makes the mating walls flush."""
    R = 400.0
    a = np.deg2rad(36)
    with BuildPart() as wedge:
        with BuildSketch(Plane.XY.offset(-15)):
            with BuildLine():
                Polyline((0.0, 0.0), (R * np.cos(a), -R * np.sin(a)),
                         (R * np.cos(a), R * np.sin(a)), close=True)
            make_face()
        extrude(amount=90)
    sector = _mother() & wedge.part
    if edge_joint:
        t2 = TIERS[1]
        r_edge = float(_outline_r(np.deg2rad(36), *t2[2:5], t2[5]))
        r_c = r_edge - WALL / 2
        # tongue: protrudes 1.6 past the +36 plane, mid-wall, z 18..30.
        # In an edge's local frame the sector BODY is on the +y side of a
        # -36 edge and the -y side of a +36 edge — so the tongue extends
        # +y past the +36 plane, and the groove cuts INTO +y at the -36
        # edge (the first version cut -y: open air; the pair-interference
        # check caught the resulting bind).
        sector += Rot(0, 0, 36) * Pos(r_c, 0.8, 24) * Box(3.0, 1.6, 12)
        sector -= Rot(0, 0, -36) * Pos(r_c, 0.9, 24) * \
            Box(3.0 + 2 * FIT, 1.9, 12 + 2 * FIT)
    return sector


def shell_cap():
    """Rocky top hatch: plug matches the seat outline minus print fit."""
    cap = _ring(49.7, 52.2, HATCH_R + 4.5, 2.0, HATCH_SEED, dr=-FIT)  # seat plug
    cap += _ring(52.2, 58.0, HATCH_R + 7.0, 2.4, 66)
    cap += _ring(58.0, 63.0, HATCH_R - 9.0, 2.2, 77, twist=18)
    # thumb notch + five plug magnets (mate the five seat magnets)
    for k in range(5):
        cap -= Rot(0, 0, 72 * k) * Pos(HATCH_R + 1.0, 0,
                                       49.7 + (PL["magnet_t"] + 0.2) / 2 - 0.01) * \
            Cylinder((PL["magnet_d"] + 0.25) / 2, PL["magnet_t"] + 0.2)
    cap -= Pos(0, -(HATCH_R + 9), 53.5) * Rot(30, 0, 0) * Box(16, 10, 6)
    return cap


# ------------------------------------------------------------------ checks
def keepouts():
    """Solids the shell must NOT touch (from the frozen frame conventions +
    part_coxa/part_deck geometry)."""
    ko = {}
    # coxa fork + yaw sweep: hardware stays inside a r=26 disc about the
    # yaw axis (station r=110); +2 mm margin, z from plate top to fork top
    ko["fork_swing"] = Pos(110, 0, 28) * Cylinder(28, 48)
    # leg-port knobs (dia 12 at leg x -41, y +/-17) + dowel tops: columns
    # to z 13 over leg-local x -49..-22 -> body r 61..88, y +/-25
    ko["port_hardware"] = Pos(74.5, 0, 6.5) * Box(27, 50, 13)
    # coxa base plate slab: Box(66, 44, 4) leg x -46..20 (+ cantilever to
    # the fork zone), y +/-22 (+2.5 margin), z -4..0 (+0.5)
    ko["coxa_plate"] = Pos(97.0, 0, -2.0) * Box(70, 49, 5.0)
    # the deck itself: R100 pentagon, corners AT stations, spans z -8..-4;
    # keep-out top 0.1 below the seating plane so contact is allowed
    ko["deck_slab"] = Pos(0, 0, DECK_TOP - 4.1) * \
        extrude(RegularPolygon(100.2, 5), 4.0)
    return ko


if __name__ == "__main__":
    sector = shell_sector()
    cap = shell_cap()
    export(sector, "shell_sector")
    export(cap, "shell_cap")
    ok = True
    for name, solid in keepouts().items():
        inter = sector & solid
        v = 0.0 if inter is None else inter.volume
        good = v < 0.5
        ok &= good
        print(f"  keep-out {name:14s}: {v:8.2f} mm^3  "
              f"{'CLEAR' if good else '*** CLASH ***'}")
    # ---- seam joint verification: neighbor pair must not interfere, and
    # the tongue must actually land INSIDE the neighbor's groove
    nb = Rot(0, 0, 72) * shell_sector()
    inter = sector & nb
    v_pair = 0.0 if inter is None else inter.volume
    ok &= v_pair < 0.5
    print(f"  assembled pair interference: {v_pair:.2f} mm^3 "
          f"({'OK' if v_pair < 0.5 else '*** BINDS ***'})")
    nb_nogroove = Rot(0, 0, 72) * shell_sector(edge_joint=False)
    inter = sector & nb_nogroove
    v_engage = 0.0 if inter is None else inter.volume
    ok &= v_engage > 10.0
    print(f"  tongue-in-groove engagement: {v_engage:.1f} mm^3 displaced "
          f"({'ENGAGED' if v_engage > 10 else '*** NOT ENGAGING ***'})")
    for n, p in (("sector", sector), ("cap", cap)):
        bb = p.bounding_box()
        print(f"  {n}: bbox {bb.size.X:.0f} x {bb.size.Y:.0f} x "
              f"{bb.size.Z:.0f} mm, {p.volume / 1000:.0f} cm^3 "
              f"({'fits bed' if bb.size.X <= 250 and bb.size.Y <= 210 else 'TOO BIG'})")
    print("ALL CLEAR" if ok else "FIX BEFORE PRINTING")
