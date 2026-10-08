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
    the real sweep geometry — the coxa fork + yaw hardware (the fork's side
    cheeks reach r 29.5 about the yaw axis, its hip cup r 41; the femur
    exits above tier 1), plus the coxa plate slab passing through the skirt
    at deck level. The fork yawed -40..40 meets the sector at 0.00 mm^3,
    15.89 mm apart at the closest (D063). Tier 4 stays a continuous ring —
    the "neck" the dome caps.
  * skirt drops to z -9: covers the deck edge from outside (turtle
    overbite), clearing the deck (deck corners r=100 < skirt inner face).
  * I6 dovetail ring: one VERTICAL male segment per sector web at +/-27
    deg (slide a shoe down from above; its set knob, on the ARCH side,
    wedges it) — the undercut profile (B83), square to the rock wall's
    chord under a shoe, root 0.5 off the wall on a neck fused into it.
  * B8 LED channel: revolved groove around tier 2 (8.4 mm tall, ~2.4 deep,
    45 deg chamfered top so it prints upright without support), fed from
    an I5 XT30 spare tap through the cable notch.
  * B9 vent gills: three angled slots per side through the tier-1 wall,
    outside the arch, over the leg bays (servo airflow).
  * attachment (I3): each sector foot carries ONE quarter-turn latch
    insert pocket (az +27.5, r 74.5) and ONE magnet pocket (az -25.5,
    r 76) — every web joint pairs neighbour A's latch with neighbour B's
    magnet (D030); the deck (v0.5, part_deck.DECK_HOLES) carries the washer
    recesses and the latch strikes. The latch is a bayonet (B87): its rotor hangs through the
    deck's strike and is turned from UNDER the deck.
  * top hatch: rocky cap, plug matches the seat outline with print
    clearance, five magnet pairs (one per sector, az 0 — the only pattern
    compatible with 5 identical sectors) at r 42.5, the seat magnet in a
    boss at the seat ledge that stands 7.3 into the opening (B88).

Checks (run this file): fork-swing keep-out, port-knob keep-out, coxa
plate slab, deck slab — all must intersect at 0.00 mm^3; the docked
coxa_yaw_base + yaw servo themselves at 0 mm^3 and the base FIT away
(review 9q: the cup's back wall cut 42.8 into every sector); the seam pair;
an I6 shoe + knob on every bar (B83); the cap seated with its magnets,
every hatch pocket walled (B88); the latch cartridge in its pad (B87);
bed-fit report.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import IF, dovetail_male, latch_pocket, SHELL_LATCH_AZ, SHELL_LATCH_R

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
# The leg arch's inboard face (body x at station az 0). The docked coxa_yaw_base's cup back wall
# (part_coxa x_back_out -38.5 = body r 71.5, |y| <= BACK_HALF_W 9.5, z 0..45.2) stood 0.5 inside it:
# 42.80 mm^3 into every sector at z 33.45..42, the tier-3 ceiling's inner skin, on main as well, and no
# keep-out held the real base (review 9q). The arch keeps x 72: moved in to 71.2 whole, its |y| <= 40
# profile would also cut the latch pad and the feet at deck level. A relief over the cup back's width
# (cup_back_relief) takes the sector FIT off that face instead, and keepouts() holds the real base.
ARCH_X0 = 72.0
HATCH_R = 46.0
HATCH_SEED = 55
HATCH_SEAT_Z = 49.6              # the rebate floor the cap's plug sits on (plug from 49.7)
# B88: the hatch magnet pair (seat + cap plug) at r 42.5, not 47: the plug's edge
# over a magnet's span at az 0 is only 47.0 (48.9 at az 0 itself, 47.3 at +4 deg),
# so a Ø6.25 pocket at 47 broke out of it; at 42.5 it keeps a 1.2 wall
HATCH_MAG_R = 42.5
MAG_POCKET_D = PL["magnet_d"] + 0.25
MAG_POCKET_T = PL["magnet_t"] + 0.2


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


I6_BARS = (27.0, -27.0)          # web azimuths of the two I6 bars per sector
I6_BAR_Z, I6_BAR_L = 3.0, 16.0
I6_ROOT_GAP = 0.5                # the wall stays within 0.12 of the chord plane under a shoe
I6_SHOE_HALF = 11.3              # half a 20.6 shoe + 1


def i6_bar_frame(az):
    """Frame of the I6 bar at azimuth az: origin on the tier-1 wall, +x the
    outward normal of the wall's chord across a shoe (+13.1 deg off the radius
    at +27, -7.8 at -27), z up."""
    t1 = TIERS[0]

    def pt(a):
        r = float(_outline_r(np.deg2rad(a), *t1[2:5], t1[5]))
        return np.array([r * np.cos(np.deg2rad(a)), r * np.sin(np.deg2rad(a))])
    c = pt(az)
    da = np.rad2deg(I6_SHOE_HALF / np.linalg.norm(c))
    t = pt(az + da) - pt(az - da)
    n_az = float(np.rad2deg(np.arctan2(-t[0], t[1])))
    a = np.deg2rad(n_az)
    x, y = c[0] * np.cos(a) + c[1] * np.sin(a), -c[0] * np.sin(a) + c[1] * np.cos(a)
    return Rot(0, 0, n_az) * Pos(float(x), float(y), 0)


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


def cup_back_relief():
    """The sector's relief over the docked cup's back wall (station az 0, body frame): FIT off its
    back face (x), its sides (|y|) and its top (z), from the wall's foot to past ARCH_X0."""
    from servo_mount import cup_extents
    from part_coxa import BACK_HALF_W
    from leg_frame import yaw_z
    e = cup_extents()
    x0 = 110.0 + e["x_back_out"] - FIT                          # 71.2
    z1 = yaw_z(e["z_lo_out"]) + FIT                              # 45.5: the cup's top (leg z 45.2)
    hw = BACK_HALF_W + FIT
    return Pos((x0 + ARCH_X0 + 1.0) / 2, 0, (z1 - FIT) / 2) * Box(ARCH_X0 + 1.0 - x0, 2 * hw, z1 + FIT)


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
    body += Rot(0, 0, SHELL_LATCH_AZ) * Pos(SHELL_LATCH_R, 0, (DECK_TOP + 0.0) / 2) * \
        Cylinder(9.0, -DECK_TOP)                      # latch pad

    # ---- leg arch (station az 0): through tiers 1-3 + the skirt
    with BuildPart() as arch:
        with BuildSketch(Plane.YZ.offset(ARCH_X0)):
            with BuildLine():
                Polyline((-40, -9.5), (40, -9.5), (40, 30), (22, 46),
                         (-22, 46), (-40, 30), close=True)
            make_face()
        extrude(amount=135.0 - ARCH_X0)
    body -= arch.part
    body -= cup_back_relief()                         # review 9q: FIT off the docked cup's back wall

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
    body -= cup_back_relief()
    # LED feed notch through the tier-2 wall into the cavity (az 33 web)
    # session 8 (D038): was az 33 — an 8 mm notch there reaches az 35.5,
    # 0.8 mm short of the 36° seam plane (one perimeter, with the seam
    # tongue hanging off it). At az 30.5 the notch spans 28..33: 4.7 mm of
    # wall to the seam, and it still lands on the web between arches.
    body -= Rot(0, 0, 30.5) * Pos(90, 0, 21) * Box(18, 8, 9)

    # ---- I6 vertical dovetail bars on the tier-1 wall at the webs. B83: the
    # undercut profile (the old key let a shoe lift off), square to the wall's
    # chord across a shoe (the rock wall is not square to the radius here: a
    # radial shoe met it 3.1 mm high on one side), root I6_ROOT_GAP off the wall
    # on a neck that fuses 1.4 into it
    for az in I6_BARS:
        frame = i6_bar_frame(az)
        body += frame * Pos(I6_ROOT_GAP, 0, I6_BAR_Z) * Rot(0, 90, 0) * \
            dovetail_male(length=I6_BAR_L, undercut=True)
        body += frame * Pos((I6_ROOT_GAP - 1.4) / 2, 0, I6_BAR_Z) * \
            Box(I6_ROOT_GAP + 1.4 + 0.01, IF["dovetail"]["crest_w"], I6_BAR_L)

    # ---- I3 attachment on the feet: one latch (pad, az +27.5) + ONE
    # magnet (az -25.5, r 76) — each web joint = neighbor A's latch +
    # neighbor B's magnet. v0 had three magnets per web at r 77.5; their
    # deck-side washer recesses (Ø8-10) would breach the pentagon edge at
    # the az±34 positions (boundary ~81 there) — deck v0.4 taught the
    # shell where the deck actually ends.
    body -= Rot(0, 0, SHELL_LATCH_AZ) * Pos(SHELL_LATCH_R, 0, DECK_TOP) * \
        latch_pocket(-1.0, 5.0)                       # the D of the keyway index (B107)
    body -= Rot(0, 0, -25.5) * Pos(76.0, 0, DECK_TOP +
                                   (PL["magnet_t"] + 0.2) / 2 - 0.01) * \
        Cylinder((PL["magnet_d"] + 0.25) / 2, PL["magnet_t"] + 0.2)

    # ---- hatch: opening + seat rebate + one seat magnet per sector (az 0)
    body -= _ring(41.0, 53.5, HATCH_R, 2.0, HATCH_SEED)
    body -= _ring(HATCH_SEAT_Z, 52.4, HATCH_R + 4.5, 2.0, HATCH_SEED)
    # B88: the rebate is deeper than the tier-4 roof (z 49.4..52), so the seat
    # ledge is a 0.2 skin, and the old pocket (r 47) hung in the cavity with no
    # walls or floor. The seat magnet gets a boss instead: 2.0 walls, a 1.0 floor,
    # reaching out to r 54 to fuse with the full roof (it is an island in the
    # cavity: the sector prints with supports). The opening's edge is at r 44.7 at
    # az 0, so the boss's inner 7.3 is a tab into the opening, flush with the seat
    # (the plug covers it). Between the five bosses the ledge stays a 0.2 skin.
    rb = MAG_POCKET_D / 2 + 2.0
    z0 = HATCH_SEAT_Z - MAG_POCKET_T - 1.0
    boss = Pos(HATCH_MAG_R, 0, (z0 + HATCH_SEAT_Z) / 2) * Cylinder(rb, HATCH_SEAT_Z - z0)
    boss += Pos((HATCH_MAG_R + 54.0) / 2, 0, (z0 + HATCH_SEAT_Z) / 2) * \
        Box(54.0 - HATCH_MAG_R, 2 * rb, HATCH_SEAT_Z - z0)
    body += boss
    body -= Pos(HATCH_MAG_R, 0, HATCH_SEAT_Z - MAG_POCKET_T / 2 + 0.01) * \
        Cylinder(MAG_POCKET_D / 2, MAG_POCKET_T)
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
    # thumb notch + five plug magnets (mate the five seat magnets). B88: at r 47
    # these broke out of the plug's face and a magnet cut 10.2 mm^3 into the seat wall
    for k in range(5):
        cap -= Rot(0, 0, 72 * k) * Pos(HATCH_MAG_R, 0, 49.7 + MAG_POCKET_T / 2 - 0.01) * \
            Cylinder(MAG_POCKET_D / 2, MAG_POCKET_T)
    cap -= Pos(0, -(HATCH_R + 9), 53.5) * Rot(30, 0, 0) * Box(16, 10, 6)
    return cap


# ------------------------------------------------------------------ checks
def keepouts():
    """Solids the shell must NOT touch (from the frozen frame conventions +
    part_coxa/part_deck geometry)."""
    ko = {}
    # coxa fork + yaw sweep about the yaw axis (station r=110), z 4..52. A
    # proxy, not an envelope: it was a r=26 disc + 2 mm, but the fork's side
    # cheeks (D063) reach r 29.5 and its hip cup r 41. Measured directly, the
    # fork yawed -40..40 vs this sector: 0.00 mm^3, min gap 15.89 at yaw 40
    ko["fork_swing"] = Pos(110, 0, 28) * Cylinder(28, 48)
    # leg-port knobs (dia 12 at leg x -41, y +/-17): columns to z 13 over
    # leg-local x -49..-22 -> body r 61..88, y +/-25. It was sized for the D020
    # dowel tops too (z 8 over the deck); since 2026-10-07 the seat posts sit
    # inside the plate's sockets (top 1 under the plate top) and the plate's
    # spine rib (x -46..-38, |y| <= 8, z 0..6) is inside this box
    ko["port_hardware"] = Pos(74.5, 0, 6.5) * Box(27, 50, 13)
    # coxa base plate slab: Box(66, 44, 4) leg x -46..20 (+ cantilever to
    # the fork zone), y +/-22 (+2.5 margin), z -4..0 (+0.5). Since 2026-10-07
    # the plate runs inboard to the hook's stem (leg x -49.55, |y| <= 18), so
    # the slab starts at leg x -50 (was -48); the sector is 0.000 mm^3 clear of it
    ko["coxa_plate"] = Pos(96.0, 0, -2.0) * Box(72, 49, 5.0)
    # the leg harness route (part_coxa.harness_path): deck cutout -> up beside
    # the yaw cup -> over the top to the yaw plugs, 11 x 11
    from part_coxa import harness_solid
    ko["leg_harness"] = Pos(110.0, 0, 0) * harness_solid()
    # the deck itself: R100 pentagon, corners AT stations, spans z -8..-4;
    # keep-out top 0.1 below the seating plane so contact is allowed
    ko["deck_slab"] = Pos(0, 0, DECK_TOP - 4.1) * \
        extrude(RegularPolygon(100.2, 5), 4.0)
    ko.update(real_keepouts())
    return ko


# the keep-outs that are the real parts, not proxies: gated at 0 mm^3 (REAL_GATE), not < 0.5.
# Review 9q: none of the proxies above held the cup's back wall at r 71.5, z 33..45, so the docked
# base cut 42.80 mm^3 into every sector and this check passed
REAL_GATE = 1e-6


def real_keepouts():
    """The docked coxa_yaw_base and its yaw servo at this sector's station (az 0, r 110), as
    part_coxa builds them."""
    from part_coxa import coxa_yaw_base, yaw_servo_placed
    return {"coxa_yaw_base": Pos(110.0, 0, 0) * coxa_yaw_base(),
            "yaw_servo": Pos(110.0, 0, 0) * yaw_servo_placed()}


def _vol(s):
    return 0.0 if s is None else s.volume


def i6_shoe_pose(az, lift=0.0):
    """The default I6 shoe seated on this sector's bar at az (+ lifted off it along
    the wall normal), its set knob on the ARCH side: at a seam the two bars' knobs
    would otherwise meet."""
    flip = 180 if az > 0 else 0
    return i6_bar_frame(az) * Pos(I6_ROOT_GAP + lift, 0, I6_BAR_Z) * Rot(0, 90, 0) * \
        Rot(0, 0, flip)


def i6_shoe_checks(sector, nb):
    """B83: a dovetail_shoe + its set knob on every bar: seats and slides down from
    above clear of the rock wall, cannot lift off, and clears the leg keep-outs, the
    neighbour sector and the neighbour's shoe across the seam. On a bar the knob
    never reaches its spot face (the bolt meets the male first): it works from the
    M3 x 12 clamp (2.1 out along the bore) to the M3 x 16 one (6.1), and slides on
    1 further out with the tip clear of the slot, so those are the poses checked."""
    from iface import dovetail_female_shoe, set_knob_tf
    from part_panel import thumb_knob_m3, set_knob_reach
    shoe = dovetail_female_shoe()
    meet = set_knob_reach(shoe, dovetail_male(undercut=True))[3]      # 9.90
    backs = (12.0 - meet, (12.0 - meet + 16.0 - meet) / 2, 16.0 - meet, 17.0 - meet)
    kits = [shoe + set_knob_tf() * Pos(0, 0, s) * thumb_knob_m3() for s in backs]
    ko = {k: v for k, v in keepouts().items() if k != "deck_slab"}
    ok = True
    for az in I6_BARS:
        tf = i6_shoe_pose(az)
        seat = max(_vol((tf * kit) & sector) for kit in kits)
        path = max(_vol((Pos(0, 0, dz) * tf * kits[-1]) & sector) for dz in range(4, 33, 4))
        lift = _vol((i6_shoe_pose(az, lift=1.0) * shoe) & sector)
        kov = max(_vol((tf * kit) & s) for s in ko.values() for kit in kits)
        good = seat < 0.5 and path < 0.5 and lift >= 1.0 and kov < 0.5
        ok &= good
        print(f"  I6 shoe + knob ({backs[0]:.1f}-{backs[-1]:.1f} out) on the {az:+.0f} bar: "
              f"seated {seat:.2f}, slide-down path {path:.2f}, keep-outs {kov:.2f} mm^3; "
              f"lifted 1.0 {lift:.1f} mm^3 "
              f"({'OK' if good else '*** SHOE DOES NOT FIT OR LIFTS OFF ***'})")
    v_nb = v_pair = 0.0
    for kit in kits:
        a = i6_shoe_pose(max(I6_BARS)) * kit                    # this sector's seam side ...
        b = Rot(0, 0, 72) * i6_shoe_pose(min(I6_BARS)) * kit    # ... and the neighbour's
        v_nb, v_pair = max(v_nb, _vol(a & nb)), max(v_pair, _vol(a & b))
    ok &= v_nb < 0.5 and v_pair < 0.5
    print(f"  I6 shoe at the seam x neighbour sector {v_nb:.2f}, x neighbour's shoe "
          f"{v_pair:.2f} mm^3 ({'CLEAR' if v_nb < 0.5 and v_pair < 0.5 else '*** CLASH ***'})")
    return ok


def hatch_magnet_checks(sector, cap):
    """B88: the cap seats on the five sectors with its five magnets in (bottomed
    in their pockets), and every hatch pocket has walls and a floor."""
    from part_panel import walled
    md, mt = PL["magnet_d"], PL["magnet_t"]
    ring = [Rot(0, 0, 72 * k) * sector for k in range(5)]
    z_cap_open, z_seat_open = 49.7, HATCH_SEAT_Z
    cap_mag = Pos(HATCH_MAG_R, 0, z_cap_open + MAG_POCKET_T - mt / 2) * Cylinder(md / 2, mt)
    seat_mag = Pos(HATCH_MAG_R, 0, z_seat_open - MAG_POCKET_T + mt / 2) * Cylinder(md / 2, mt)
    posed = cap + sum((Rot(0, 0, 72 * k) * cap_mag for k in range(1, 5)), cap_mag)
    v_seat = sum(_vol(posed & s) for s in ring)
    v_mags = _vol(seat_mag & cap)
    gap = (z_cap_open + MAG_POCKET_T - mt) - (z_seat_open - MAG_POCKET_T + mt)
    ok = v_seat < 0.01 and v_mags < 0.01
    print(f"  hatch: cap + 5 magnets x 5 sectors {v_seat:.3f} mm^3, seat magnet x cap "
          f"{v_mags:.3f} mm^3, magnet faces {gap:.2f} apart (bottomed) "
          f"({'SEATS' if ok else '*** CAP DOES NOT SEAT ***'})")
    for name, part, z_open, z_closed in (
            ("seat pocket (sector)", sector, z_seat_open, z_seat_open - MAG_POCKET_T),
            ("plug pocket (cap)", cap, z_cap_open, z_cap_open + MAG_POCKET_T)):
        fw, ff = walled(part, (HATCH_MAG_R, 0), z_open, z_closed, MAG_POCKET_D)
        good = fw > 0.99 and ff > 0.99
        ok &= good
        print(f"  hatch {name}: 0.5 mm walls {100 * fw:.1f} %, floor {100 * ff:.1f} % "
              f"({'WALLED' if good else '*** OPEN POCKET ***'})")
    return ok


def latch_pad_checks(sector):
    """B87: the latch cartridge in the sector's pad, the housing's bottom flush with
    the foot's (on the deck top): housing and rotor clear of the sector at OPEN, half
    way and LOCKED. The lugs hang below the foot into the deck's strike (part_deck
    checks that side) and the rotor is worked from under the deck. B107: the pad's D
    pocket holds the housing on its keyway index, keyways outside the latch's travel."""
    from iface import latch_insert_rotor, LATCH_ENTRY_DEG
    from part_panel import latch_index, index_verdict, index_report
    tf = Rot(0, 0, SHELL_LATCH_AZ) * Pos(SHELL_LATCH_R, 0, DECK_TOP)
    rotor = latch_insert_rotor()
    v_r = max(_vol((tf * Rot(0, 0, LATCH_ENTRY_DEG + phi) * rotor) & sector) for phi in (0, 45, 90))
    m = latch_index(sector, tf)
    ok = m["seated"] < 0.01 and v_r < 0.01
    print(f"  I3 latch cartridge in the pad: housing x sector {m['seated']:.2f}, rotor x sector "
          f"(open / 45 / locked) {v_r:.2f} mm^3 ({'CLEAR' if ok else '*** CLASH ***'})")
    bad = index_verdict(m)
    print(f"  I3 keyway index in the pad: {index_report(m)} ({'INDEXED' if not bad else '*** ' + '; '.join(bad) + ' ***'})")
    return ok and not bad


if __name__ == "__main__":
    sector = shell_sector()
    cap = shell_cap()
    export(sector, "shell_sector")
    export(cap, "shell_cap")
    ok = True
    real = real_keepouts()
    for name, solid in keepouts().items():
        inter = sector & solid
        v = 0.0 if inter is None else inter.volume
        good = v <= REAL_GATE if name in real else v < 0.5
        ok &= good
        print(f"  keep-out {name:14s}: {v:8.2f} mm^3  "
              f"{'CLEAR' if good else '*** CLASH ***'}"
              + ("  (the real part, gated at 0)" if name in real else ""))
    # the relief's own margin: the sector's nearest point to the docked base
    gap = sector.distance_to(real["coxa_yaw_base"])
    ok &= gap >= FIT - 0.01
    print(f"  sector to the docked coxa_yaw_base: {gap:.2f} mm (cup_back_relief, FIT {FIT}) "
          f"{'CLEAR' if gap >= FIT - 0.01 else '*** TOO CLOSE ***'}")
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
    ok &= i6_shoe_checks(sector, nb)
    ok &= hatch_magnet_checks(sector, cap)
    ok &= latch_pad_checks(sector)
    for n, p in (("sector", sector), ("cap", cap)):
        bb = p.bounding_box()
        fits = bb.size.X <= PR["bed_mm"][0] and bb.size.Y <= PR["bed_mm"][1] \
            and bb.size.Z <= PR["bed_mm"][2]
        ok &= fits
        print(f"  {n}: bbox {bb.size.X:.0f} x {bb.size.Y:.0f} x "
              f"{bb.size.Z:.0f} mm, {p.volume / 1000:.0f} cm^3 "
              f"({'fits bed' if fits else '*** TOO BIG for the bed ***'})")
    print("ALL CLEAR" if ok else "FIX BEFORE PRINTING")
    raise SystemExit(0 if ok else 1)
