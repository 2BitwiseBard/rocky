"""I1 dock proof — can a leg be put on the deck, and does it sit right once it is.

The D020 port passed every boolean check (docked: 0 mm^3) and still could not be docked:
the L's 5.15 foot met a 4.2 slot and the 8 mm dowels blocked the pivot (B117, found
2026-10-01; no rigid path at 0.10 clearance). Nothing ever moved the base onto the deck.
Since the 2026-10-07 seats (body-layout decision 15) this module does, on the REAL solids
(coxa_yaw_base and body_deck at station 0, leg-local), so the port can never go back to
undockable without a failing check:

  docked   the deck's OWN seat posts, at every station, are leg_port_seats() as drawn
           (symmetric difference < 0.01 mm^3), and the tests below dock against them, not
           against iface's posts; the base on the deck: 0 mm^3; the relieved face
           seat_relief over the deck; pressed down the deck is met only by the inboard pads
           (x < seat_relief_x0); the cone flanks print.seat_fit (f) cos 30 apart, the radial
           play f (nudged f + 0.05 in x / y the base meets the posts; nudged f / 2 it does
           not), the cones bear a 0.05 press at f 0 and leave z to the pads at f 0.1 / 0.2;
           lifted dz they open by dz sin 30 + f cos 30
  planar   the dock path in the leg's x-z plane (tx, tz, tilt th about the y-parallel line
           through R0 = (-48, -4); + th lifts the outboard end): the clearance between the
           moved base and the deck is the minimum 2D distance over y-sections of fine
           meshes of both solids. Four stored paths, from 'clear above the deck' (L over
           the deck top, a free vertical sweep up from there) to docked, are walked at 0.004
           mm substeps: HAND (the designer's 10 deg hand path; its 0.305 is where its drop
           starts), MARGIN (the first verifier's: >= 0.40 at tilt <= 10), SEARCHED (what
           --search found on this tree: 0.314, the same all down its straight descent) and
           WIDE (the second verifier's, the widest stored: 0.469; the in-plane ceiling at
           tilt <= 10 is 0.50, set at the pre-seat by the cones and the foot-top /
           deck-bottom gap, 987,795-pose maximin). WIDE is the hand motion to describe: hold 10 deg up and 1.55
           outboard, lower straight to 2.2 over docked, swing flat while sliding inboard,
           drop onto the cones from 1.0. The proof is one path at >= FIT; the last 0.61 /
           0.825 / 0.65 / 1.0 of the drop is the cones closing, by design
  3D       every path replayed on the cropped solids: BRep common volume and BRepExtrema
           distance at poses no point of the base moves more than 0.2 between, so a gap
           >= 0.30 at every pose holds >= 0.20 continuously (every pose between is within
           0.1 of one that was measured): 0 mm^3 throughout, minimum 0.300 at the lip ends
           (the lip-end / slot-end y gap is FIT, which the planar model cannot see); with
           the lip ends trimmed 0.3 the rest of the base keeps the planar minimum
  hook     the L's lowest z and its x span under the deck (z < -10) on the paths and at ANY
           free pose (a tilt -3..40 grid, the extremes refined on every section, plus the
           exact rock on the pads' inboard edge), for the body layout's tub roof. Measured on
           the L's own 8-corner outline (its extremes are a corner or an edge crossing
           z -10); x is leg-local, r = 110 + x is on the station axis, and the L is
           hook_lip_w wide, so its corners reach body radius hypot(110 + x, 12). Both
           envelopes are held to iface.HOOK_ENVELOPE (+-0.02), which the keep-outs import

Numbers to expect (2026-10-07, I1_DOCK_OPTIONS.md + the verification; the judge's graft =
this tree, seat_fit 0): docked 0 mm^3, relieved face 0.200, in-plane 0.305 (HAND) / 0.400 (MARGIN) /
0.314 (SEARCHED) / 0.469 (WIDE), 3D 0.300 (trimmed: the planar minimum); hook on the paths
z -13.30, x -52.50..-44.71 (r 57.50..65.29); at any free pose z -13.757 (rocked 17.6 deg on
the pads' inboard edge), x -53.12..-43.39 (r 56.88..66.61; the L stays under the deck to
about 36 deg of tilt, the outboard extreme is at 34.4), corners to body radius 67.68. The
verifier's exact BRep sections (515) read the x span 0.005 narrower: -53.116..-43.397.

If a param change breaks a stored path, search a new one (seconds to minutes; paste it in):
    python3 check_dock.py --search 0.30
Exit code = number of failures. Writes nothing.
"""
import os
import sys
import tempfile
import time
import heapq
import numpy as np
import shapely
import trimesh
from shapely.geometry import MultiPolygon
from shapely.ops import unary_union
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from build123d import Pos, Rot, Box, export_stl
from iface import (IF, PR, FIT, hook_geometry, seat_posts_symdiff, HOOK_ENVELOPE,
                   HOOK_ENVELOPE_TOL)
from part_coxa import coxa_yaw_base
from part_deck import body_deck, STATIONS, R_STATION

LP = IF["leg_port"]
R0 = (-48.0, -4.0)            # the tilt pivot (leg x, z): the D020 slot centre, kept as the paths' frame
SIN30 = np.sin(np.deg2rad(LP["seat_half_angle"]))   # a lift dz opens the flanks by dz sin 30,
COS30 = np.cos(np.deg2rad(LP["seat_half_angle"]))   # a radial offset f by f cos 30
SEAT_FIT = PR["seat_fit"]     # the sockets' filed radial play over the posts (0: touching, nominal)
THRESH = FIT                  # the proof: one path at >= FIT (0.30)
BASE_CROP = Pos(-29.0, 0, -3.0) * Box(62.0, 50.0, 24.0)     # x -60..2, |y| <= 25, z -15..9
DECK_CROP = Pos(-39.5, 0, -5.5) * Box(61.0, 52.0, 11.0)     # x -70..-9, |y| <= 26, z -11..0
BASE_CLIP, DECK_CLIP = (-70, -20, -5, 0.5), (-64, -20, -5, 5)  # in-plane (x, z): only the plate,
                                                             # the sockets and the L meet the deck
# dock order, clear above -> pre-seat -> docked; (tx, tz, th)
HAND = [(30.0000, 10.7800, 10.00), (1.5800, 10.7800, 10.00), (1.5800, 1.5803, 10.00),
        (-0.4800, 1.5803, 10.00), (0.4748, 1.1512, 9.75), (0.5121, 1.1401, 9.50),
        (0.5120, 1.1289, 9.25), (0.5879, 1.1304, 9.00), (0.6263, 1.1192, 8.75),
        (0.6651, 1.1080, 8.50), (0.6908, 1.0967, 8.25), (0.7232, 1.0922, 8.00),
        (0.7767, 1.0808, 7.75), (0.7744, 1.0694, 7.50), (0.7425, 1.0578, 7.25),
        (0.7124, 1.0426, 7.00), (0.6657, 1.0272, 6.75), (0.6281, 1.0153, 6.50),
        (0.5887, 1.0034, 6.25), (0.5436, 0.9875, 6.00), (0.5005, 0.9753, 5.75),
        (0.4556, 0.9630, 5.50), (0.4224, 0.9496, 5.25), (0.3796, 0.9417, 5.00),
        (0.3316, 0.9285, 4.75), (0.2884, 0.9158, 4.50), (0.2240, 0.9034, 4.25),
        (0.2081, 0.8913, 4.00), (0.1762, 0.8848, 3.75), (0.1613, 0.8833, 3.50),
        (0.1465, 0.8911, 3.25), (0.1320, 0.8975, 3.00), (0.1131, 0.9069, 2.75),
        (0.1034, 0.9146, 2.50), (0.0853, 0.9124, 2.25), (0.0755, 0.9247, 2.00),
        (0.0582, 0.9272, 1.75), (0.0501, 0.9412, 1.50), (0.0311, 0.9445, 1.25),
        (0.0201, 0.9494, 1.00), (0.0093, 0.9612, 0.75), (-0.0014, 0.9733, 0.50),
        (-0.0015, 0.9857, 0.25), (0.0011, 1.0031, 0.00), (0.0000, 0.6100, 0.00),
        (0.0000, 0.0000, 0.00)]
MARGIN = [(1.500, 10.150, 6.00), (1.600, 10.050, 6.40), (1.625, 10.025, 6.40),
          (1.650, 9.950, 6.70), (1.650, 9.925, 6.70), (1.650, 9.875, 6.90),
          (1.500, 8.175, 9.50), (1.475, 7.975, 9.70), (1.600, 2.050, 8.80),
          (1.675, 1.850, 8.00), (1.675, 1.825, 7.90), (1.650, 1.700, 7.40),
          (1.600, 1.625, 7.10), (1.550, 1.575, 6.90), (1.250, 1.450, 5.70),
          (1.025, 1.375, 4.70), (0.000, 0.825, 0.00), (0.000, 0.000, 0.00)]
# found by `check_dock.py --search 0.30` on this tree (7546 lattice poses, 22 s): the simplest
# hand motion yet. Held 9.25 deg up and 1.45 outboard, straight down to 1.85 over docked, then
# flat and 1.45 inboard in one move, then down onto the cones
SEARCHED = [(1.4500, 13.5000, 9.25), (1.4500, 1.8500, 9.25), (0.0000, 0.6500, 0.00), (0.0000, 0.0000, 0.00)]
# the second verifier's (2026-10-07, a maximin over 987,795 lattice poses at tilt -1..10, then
# smoothed): the widest stored path, 0.469 continuous in-plane at (1.55, 1.955, 9.39). The hand
# motion to describe: 10 deg up and 1.55 outboard, straight down to 2.2, flat while 1.55 inboard
WIDE = [(1.55, 13.5, 10.0), (1.55, 2.95, 10.0), (1.55, 2.2, 10.0), (1.55, 1.7, 8.75),
        (0.95, 1.2, 5.5), (0.0, 1.0, 0.0), (0.0, 0.0, 0.0)]
PATHS = {"HAND (10 deg, radial)": HAND, "MARGIN (>= 0.40 at tilt <= 10)": MARGIN,
         "SEARCHED (9.25 deg, straight down)": SEARCHED, "WIDE (10 deg, 1.55 out, straight down)": WIDE}
REPLAY_DISP, REPLAY_REACH = 0.2, 52.0   # 3D poses: no point of the base crop (<= 52 from R0) moves
                                        # more than 0.2 between them, so a gap >= 0.30 at every pose
                                        # is >= 0.20 all the way between (each pose between is within
                                        # 0.1 of a measured one; the drop onto the cones aside)
FREE_EPS = 0.001              # 'free' in the hook envelope: the meshes' chord tolerance. The outboard
                              # extreme sits in a narrow wedge: at 0.002 it reads 0.025 short

fails = []


def _v(x):
    return 0.0 if x is None else x.volume


def check(name, ok, msg):
    print(f"  {name}: {msg} ({'OK' if ok else 'FAIL'})", flush=True)
    if not ok:
        fails.append(name)


def pose_tf(tx, tz, th):
    """Rotate th deg about the y-parallel line through R0 (+th lifts +x), then translate."""
    return Pos(tx, 0, tz) * Pos(R0[0], 0, R0[1]) * Rot(0, -th, 0) * Pos(-R0[0], 0, -R0[1])


def dist3(a, b):
    """BRepExtrema minimum distance between two solids, and where on `a` it is."""
    e = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    p = e.PointOnShape1(1)
    return e.Value(), (round(p.X(), 2), round(p.Y(), 2), round(p.Z(), 2))


# ------------------------------------------------------------------ planar model
def section_ys():
    """y-sections that see every port feature: a coarse sweep, the lip / slot ends, the plate
    extension's edge and a 0.15 sweep across each seat's socket (+ the vee's slack)."""
    ys = {0.0}
    for a in (1.0, 2.75, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 13.0, 14.0, 19.5, 20.5, 21.5, 21.95):
        ys |= {a, -a}
    hw, sw = LP["hook_lip_w"] / 2, LP["hook_lip_w"] / 2 + FIT
    for a in (hw - 0.5, hw - 0.05, (hw + sw) / 2, sw + 0.1, sw + 0.3, LP["plate_ext_half_w"] - 0.4,
              LP["plate_ext_half_w"] + 0.3):
        ys |= {a, -a}
    reach = LP["seat_r"] + LP["seat_mouth"] + LP["seat_vee_slack"] + 0.4
    for _, sy in LP["seat_xy"]:
        ys |= {round(sy + d, 3) for d in np.arange(-reach, reach + 1e-9, 0.15)}
    return sorted(ys)


def _section(mesh, y0):
    s = mesh.section(plane_origin=[0, y0, 0], plane_normal=[0, 1, 0])
    if s is None:
        return MultiPolygon()
    T = np.eye(4)
    T[:3, :3] = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]           # (x, y, z) -> (x, z, -(y - y0))
    T[2, 3] = y0
    p2, _ = s.to_2D(to_2D=T, check=False)
    g = unary_union([p.buffer(0) for p in p2.polygons_full])
    return g


def _mesh(shape):
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "m.stl")
        export_stl(shape, p, tolerance=0.001, angular_tolerance=0.02)
        return trimesh.load(p, force="mesh")


def load_pairs(base_c, deck_c, ys):
    """[(y, base section, deck section)] at the ys, identical neighbours dropped."""
    bm, dm = _mesh(base_c), _mesh(deck_c)
    pairs, seen = [], set()
    for y in ys:
        b = shapely.clip_by_rect(_section(bm, y), *BASE_CLIP)
        d = shapely.clip_by_rect(_section(dm, y), *DECK_CLIP)
        if b.is_empty or d.is_empty:
            continue
        b, d = shapely.simplify(b, 0.0005), shapely.simplify(d, 0.0005)
        key = (shapely.to_wkb(shapely.normalize(shapely.set_precision(b, 1e-4))),
               shapely.to_wkb(shapely.normalize(shapely.set_precision(d, 1e-4))))
        if key not in seen:
            seen.add(key)
            pairs.append((y, b, d))
    return pairs


def clearance(pairs, P):
    """Min in-plane distance, base (moved to each pose of P) to the deck, over the sections."""
    P = np.atleast_2d(np.asarray(P, float))
    n = len(P)
    tx, tz = P[:, 0], P[:, 1]
    c, s = np.cos(np.deg2rad(P[:, 2])), np.sin(np.deg2rad(P[:, 2]))
    out = np.full(n, np.inf)
    for _, b, d in pairs:
        k = shapely.get_num_coordinates(b)

        def f(xy, k=k):
            xy = xy.reshape(n, k, 2)
            x, z = xy[:, :, 0] - R0[0], xy[:, :, 1] - R0[1]
            return np.stack([c[:, None] * x - s[:, None] * z + R0[0] + tx[:, None],
                             s[:, None] * x + c[:, None] * z + R0[1] + tz[:, None]], -1).reshape(n * k, 2)
        out = np.minimum(out, shapely.distance(shapely.transform(np.full(n, b, dtype=object), f), d))
    return out


def densify(W, max_disp=0.004, reach=40.0):
    """Substeps so no point within `reach` of R0 moves more than max_disp between poses."""
    W = np.asarray(W, float)
    out = [W[0]]
    for a, b in zip(W[:-1], W[1:]):
        n = max(1, int(np.ceil((np.hypot(*(b[:2] - a[:2])) + abs(np.deg2rad(b[2] - a[2])) * reach) / max_disp)))
        out.extend(a + t * (b - a) for t in np.linspace(0, 1, n + 1)[1:])
    return np.array(out)


def hook_outline(plate_bot_z=-4.0):
    """The L's section (x, z) as its 8 corners in order: the foot (foot_x0..foot_x1, z
    foot_z0..foot_z1) under the stem (x0..x1, from stem_z0 up to the plate's underside).
    Until 2026-10-07 (verification) this was the 4 x 3 grid of those x's and z's, and one
    of its 12 points, (x1, foot_z0), is off the L (the foot's outboard bottom corner is at
    foot_x1, 0.55 inboard): it overstated the outboard reach by 0.2..0.5 mm."""
    hg = hook_geometry(plate_bot_z)
    return np.array([(hg["foot_x0"], hg["foot_z0"]), (hg["foot_x1"], hg["foot_z0"]),
                     (hg["foot_x1"], hg["stem_z0"]), (hg["x1"], hg["stem_z0"]),
                     (hg["x1"], plate_bot_z), (hg["x0"], plate_bot_z),
                     (hg["x0"], hg["foot_z1"]), (hg["foot_x0"], hg["foot_z1"])])


def under_span(X, Z, zc=-10.0):
    """Per pose (rows of the moved outline X, Z): the x span of the L below z = zc. A
    polygon's extremes under a level are its corners below it or its edges' crossings of it."""
    X2, Z2 = np.roll(X, -1, 1), np.roll(Z, -1, 1)
    cross = (Z - zc) * (Z2 - zc) < 0
    xc = X + np.where(cross, (zc - Z) / np.where(cross, Z2 - Z, 1.0), 0.0) * (X2 - X)
    lo = np.minimum(np.where(Z < zc, X, np.inf).min(1), np.where(cross, xc, np.inf).min(1))
    hi = np.maximum(np.where(Z < zc, X, -np.inf).max(1), np.where(cross, xc, -np.inf).max(1))
    return lo, hi


def moved(P, pts):
    P = np.atleast_2d(np.asarray(P, float))
    c, s = np.cos(np.deg2rad(P[:, 2]))[:, None], np.sin(np.deg2rad(P[:, 2]))[:, None]
    x, z = pts[:, 0][None, :] - R0[0], pts[:, 1][None, :] - R0[1]
    return c * x - s * z + R0[0] + P[:, 0:1], s * x + c * z + R0[1] + P[:, 1:2]


def clear_above(pairs, p, thresh):
    """Is p 'above the deck': the L over the deck top + thresh, and a free (>= thresh)
    vertical sweep from p to 13.5 up, where nothing of the port is in reach?
    Returns (verdict, the L's lowest z, the sweep's clearance or nan if not swept)."""
    _, Z = moved(p, hook_outline())
    if Z.min() < -4.0 + thresh:
        return False, Z.min(), float("nan")
    zz = np.arange(p[1], max(p[1], 13.5) + 1e-9, 0.05)
    sweep = clearance(pairs, np.column_stack([np.full(len(zz), p[0]), zz, np.full(len(zz), p[2])])).min()
    return bool(sweep >= thresh), Z.min(), sweep


# ------------------------------------------------------------------ the search (--search)
TOP = 13.5                    # tz from which nothing of the port is in reach (the search's ceiling)


def lifts_free(pairs, p, thresh):
    """From p, does a straight vertical lift to TOP stay >= thresh? Cheap test first: every
    point of the L below the deck top + thresh must sit inside the slot's x span (a foot
    still under the deck would meet its bottom)."""
    hg = hook_geometry(-4.0)
    X, Z = moved(p, hook_outline())
    low = Z < -4.0 + thresh
    if low.any() and not (X[low].min() >= hg["xi"] + thresh and X[low].max() <= hg["xo"] - thresh):
        return False
    zz = np.arange(p[1], TOP + 1e-9, 0.05)
    return bool(clearance(pairs, np.column_stack([np.full(len(zz), p[0]), zz, np.full(len(zz), p[2])])).min()
                >= thresh)


def search(pairs, thresh, st=(0.05, 0.05, 0.25), batch=800, budget=3_000_000):
    """Threshold connectivity on a (tx, tz, th) lattice from the pre-seat pose (docked, lifted
    until the cones open to thresh) to the first pose a vertical lift to TOP leaves free;
    greedy upward, complete over the reachable component. Returns the dock-order waypoints
    (TOP first, smoothed, re-walked) or None."""
    ax = [np.round(np.arange(a, b + 1e-9, d), 6) for (a, b), d in
          zip(((-1.5, 6.0), (0.0, TOP), (-3.0, 18.0)), st)]
    shp = tuple(len(a) for a in ax)

    def pose(i):
        return np.array([ax[0][i[0]], ax[1][i[1]], ax[2][i[2]]])
    i0, k0 = int(np.argmin(abs(ax[0]))), int(np.argmin(abs(ax[2])))
    src = next(((i0, j, k0) for j in range(shp[1]) if clearance(pairs, pose((i0, j, k0)))[0] >= thresh), None)
    if src is None:
        return None
    state = np.zeros(shp, np.int8)
    state[src] = 1
    parent, heap, n_eval, t0 = {src: None}, [(-src[1], src)], 1, time.time()
    rng = np.random.default_rng(1)
    nei = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0)]
    while heap and n_eval < budget:
        popped = [heapq.heappop(heap)[1] for _ in range(min(len(heap), batch // 8))]
        cand = {}
        for u in popped:
            for dv in nei:
                w = tuple(u[k] + dv[k] for k in range(3))
                if all(0 <= w[k] < shp[k] for k in range(3)) and state[w] == 0 and w not in cand:
                    cand[w] = u
        if not cand:
            continue
        ws = list(cand)
        cl = clearance(pairs, [pose(w) for w in ws])
        if n_eval // 50000 != (n_eval + len(ws)) // 50000:
            print(f"  ... {n_eval + len(ws)} poses, {int((state == 1).sum())} free, highest tz "
                  f"{max(ax[1][w[1]] for w in ws):.2f}, {time.time() - t0:.0f} s", flush=True)
        n_eval += len(ws)
        for w, cw in zip(ws, cl):
            state[w] = 1 if cw >= thresh else 2
            if cw < thresh:
                continue
            parent[w] = cand[w]
            if lifts_free(pairs, pose(w), thresh):
                path = [w]
                while parent[path[-1]] is not None:
                    path.append(parent[path[-1]])
                raw = [pose(q) for q in path]                   # exit -> pre-seat
                print(f"  found after {n_eval} poses, {time.time() - t0:.0f} s: {len(raw)} lattice nodes; "
                      f"smoothing", flush=True)
                W = list(raw)
                for _ in range(150):                            # shortcuts, each walked at 0.01 (the
                    if len(W) < 3:                              # result is re-walked at 0.004 below)
                        break
                    i, j = sorted(rng.choice(len(W), 2, replace=False))
                    if j - i > 1 and clearance(pairs, densify([W[i], W[j]], max_disp=0.01)).min() >= thresh:
                        W = W[:i + 1] + W[j:]
                lift = [np.array([raw[0][0], TOP, raw[0][2]])]  # the vertical lift's top comes first
                for cand_w in (W, raw):                         # the lattice's own diagonal steps were
                    out = [tuple(float(x) for x in np.round(q, 4)) for q in lift + cand_w]   # never walked
                    m = clearance(pairs, densify(out)).min()
                    print(f"  {len(out)} waypoints, continuous in-plane min {m:.4f} "
                          f"({'OK' if m >= thresh else 'BELOW THE THRESHOLD'}), {time.time() - t0:.0f} s", flush=True)
                    if m >= thresh:
                        return out + [(0.0, 0.0, 0.0)]
                return None
            heapq.heappush(heap, (-w[1], w))
    return None


# ------------------------------------------------------------------ the hook envelope
ENV_BOX = ((-2.5, 4.0, 0.25), (-1.0, 4.5, 0.05), (-3.0, 40.0, 1.0))   # tx, tz, tilt: (from, to, step)
_SIGN = {"z": 1.0, "lo": 1.0, "hi": -1.0}      # minimise the L's lowest z / its inboard x, maximise its outboard x


def _env_score(key, P, L):
    X, Z = moved(P, L)
    if key == "z":
        return Z.min(axis=1)
    lo, hi = under_span(X, Z)
    return hi if key == "hi" else lo


def _first_free(pairs, P, s, cur):
    """The best-scoring pose of P (score s, lower is better, below cur) free on every section."""
    idx = np.where(s < cur - 1e-9)[0]
    idx = idx[np.argsort(s[idx])]
    for k in range(0, len(idx), 64):
        ok = clearance(pairs, P[idx[k:k + 64]]) >= FREE_EPS
        if ok.any():
            return idx[k:k + 64][np.argmax(ok)]
    return None


def envelope(pairs, few, L):
    """The L's extremes at ANY free pose: a grid over ENV_BOX (free on the few sections at
    FREE_EPS, which frees a superset), its best per extreme confirmed free on every section,
    then refined on every section by a local lattice (thirds of a step over +-2 grid steps,
    then over +-1 step with the step halved, 8 levels). A greedy pattern search stalled up
    to 0.05 short here, and +-1 step from a tied grid pose 0.014 short: the outboard extreme
    sits in a thin wedge of free poses. Returns {key: (value, pose, on the grid's edge?)};
    values in z or leg-local x."""
    (a0, a1, da), (b0, b1, db), (c0, c1, dc) = ENV_BOX
    TX, TZ = np.meshgrid(np.arange(a0, a1 + 1e-9, da), np.arange(b0, b1 + 1e-9, db))
    F = []
    for th in np.arange(c0, c1 + 1e-9, dc):
        P = np.column_stack([TX.ravel(), TZ.ravel(), np.full(TX.size, th)])
        P = P[moved(P, L)[1].min(axis=1) < -10.0]  # only a pose with some of the L under the deck sets a record
        if len(P):
            F.append(P[clearance(few, P) >= FREE_EPS])
    F = np.concatenate(F)
    def lattice(span):
        g = np.arange(-3 * span, 3 * span + 1) / 3
        return np.array(np.meshgrid(g, g, g, indexing="ij")).reshape(3, -1).T
    out = {}
    for key in _SIGN:
        s = _SIGN[key] * _env_score(key, F, L)
        p = F[_first_free(pairs, F, s, np.inf)]            # the best grid pose free on every section
        edge = any(abs(p[i] - lim) < 1e-9 for i, (lo, hi, _) in enumerate(ENV_BOX) for lim in (lo, hi))
        cur, st = _SIGN[key] * _env_score(key, p[None], L)[0], np.array([da, db, dc])
        for lev in range(8):
            P = p + lattice(2 if lev == 0 else 1) * st
            s = _SIGN[key] * _env_score(key, P, L)
            j = _first_free(pairs, P, s, cur)
            if j is not None:
                p, cur = P[j], s[j]
            st = st / 2
        out[key] = (_SIGN[key] * cur, tuple(round(float(q), 3) for q in p), edge)
    return out


def pad_edge_rock(pairs, L):
    """The exact deepest rock: the base pivoting on the pads' inboard edge Q = (x0, -4) (the
    plate's underside ends there; the deck top carries it) until the L's corner farthest from
    Q hangs straight under it, at -4 - |QV|. Returns (z, tilt, pose, free lifted 2 FREE_EPS?)."""
    hg = hook_geometry(-4.0)
    Q = np.array([hg["x0"], -4.0])
    V = L[int(np.argmax(np.hypot(*(L - Q).T)))]
    d = np.hypot(*(V - Q))
    th = -90.0 - np.rad2deg(np.arctan2(*(V - Q)[::-1]))      # turns Q->V to straight down
    X, Z = moved((0.0, 0.0, th), Q[None])
    p = (Q[0] - X[0, 0], Q[1] - Z[0, 0] + 2 * FREE_EPS, th)   # back onto Q, then 0.002 up
    return Q[1] - d, th, tuple(round(float(q), 4) for q in p), bool(clearance(pairs, p)[0] >= FREE_EPS)


# ------------------------------------------------------------------ main
if __name__ == "__main__":
    t00 = time.time()
    base = coxa_yaw_base()
    station = Rot(0, 0, STATIONS[0]) * Pos(R_STATION, 0, 0)
    deck = station.inverse() * (Pos(0, 0, -10) * body_deck())   # part_deck's docking pose, leg-local
    base_c, deck_c = base & BASE_CROP, deck & DECK_CROP
    print(f"solids: coxa_yaw_base {base.volume:.1f} mm^3, body_deck at station 0 (leg-local) "
          f"({time.time() - t00:.0f} s)", flush=True)

    print("== docked (the real solids)")
    # the deck's OWN posts: the docked tests below run against them, and at every station
    # they must be leg_port_seats() as drawn (missing or shrunk posts only add clearance, so
    # no overlap or path test can see them; with virtual posts this module stayed CLEAN on a
    # deck without any, 2026-10-07 review)
    deck_b = Pos(0, 0, -10) * body_deck()
    sd = [seat_posts_symdiff(deck_b, -4.0, Rot(0, 0, az) * Pos(R_STATION, 0, 0))[0] for az in STATIONS]
    _, posts = seat_posts_symdiff(deck_c, -4.0)            # station 0's own posts, leg-local
    check("body_deck carries its seat posts as drawn, every station (symmetric difference with "
          "leg_port_seats)", max(sd) < 0.01 and posts is not None,
          ", ".join(f"{s:.4f}" for s in sd) + " mm^3" + ("; station 0 has NO posts" if posts is None else
          f" (station 0's own posts, {posts.volume:.2f} mm^3, are what the seat tests dock against)"))
    rx0 = LP["seat_relief_x0"]
    vd = _v(base & deck)
    check("coxa_yaw_base x body_deck, docked", vd < 0.01, f"{vd:.4f} mm^3")
    slab = deck_c & (Pos(-39.5, 0, -7.5) * Box(61.0, 52.0, 7.0))              # the deck, no posts
    relieved = base_c & (Pos((rx0 + 2.0) / 2 + 0.05, 0, -3.0) * Box(2.0 - rx0 - 0.1, 50.0, 24.0))
    dr, at = dist3(relieved, slab)
    check(f"relieved face (x > {rx0:.0f}) over the deck top", abs(dr - LP["seat_relief"]) < 0.005,
          f"{dr:.3f} at {at} (seat_relief {LP['seat_relief']})")
    pads = _v((Pos(0, 0, -0.05) * base_c) & slab)
    padx = ((Pos(0, 0, -0.05) * base_c) & slab).bounding_box().max.X if pads > 0 else float("nan")
    check("pressed 0.05 down, the deck meets only the inboard pads", pads > 0.01 and padx <= rx0 + 0.01,
          f"{pads:.3f} mm^3, x <= {padx:.2f}")
    # the sockets are the posts' flanks grown radially by f = print.seat_fit (filed from the
    # port coupons: 0 / 0.1 / 0.2). Docked they stand f cos 30 off, the radial play is f, and
    # z is the pads' alone once f cos 30 > 0.05 sin 30 (a 0.05 press no longer reaches the
    # cones). Filing 0.1 or 0.2 is the documented coupon outcome: these must stay green on it.
    if posts is None:                 # nothing to dock against: the seats test only that
        check("the seat tests (the cone flanks, the play, the lift)", False, "skipped: the deck has no seat posts")
    else:
        f = SEAT_FIT
        g0 = f * COS30
        d0, at = dist3(base_c, posts)
        ov0 = _v(base_c & posts)
        check(f"seats in their sockets, cone flanks {g0:.4f} apart (seat_fit {f})",
              abs(d0 - g0) < (1e-3 if f == 0 else 0.005) and ov0 < 0.01,
              f"gap {d0:.4f} at {at}, overlap {ov0:.4f} mm^3")
        dirs = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0)}
        meet = {k: _v((Pos(*(np.array(u) * (f + 0.05))) * base_c) & posts) for k, u in dirs.items()}
        check(f"nudged {f + 0.05:.2f} (seat_fit + 0.05) in +-x / +-y the base meets the posts: radial play "
              f"<= seat_fit{', zero, nominal' if f == 0 else ''}", min(meet.values()) > 1e-3,
              ", ".join(f"{k} {b:.3f}" for k, b in meet.items()) + " mm^3")
        if f > 0:
            free = {k: _v((Pos(*(np.array(u) * f / 2)) * base_c) & posts) for k, u in dirs.items()}
            check(f"nudged {f / 2:.3f} (seat_fit / 2) it does not: the filed play is in the sockets",
                  max(free.values()) < 1e-3, ", ".join(f"{k} {b:.4f}" for k, b in free.items()) + " mm^3")
        down = _v((Pos(0, 0, -0.05) * base_c) & posts)
        cones_carry = 0.05 * SIN30 > g0
        check(f"pressed 0.05 down the cones {'bear (with the pads)' if cones_carry else 'stay clear: the pads carry z'}",
              down > 1e-3 if cones_carry else down < 1e-3, f"{down:.3f} mm^3")
        dl, _ = dist3(Pos(0, 0, 0.2) * base_c, posts)
        check("lifted 0.2, the cones open by 0.2 sin 30 (+ seat_fit cos 30)", abs(dl - (g0 + 0.2 * SIN30)) < 0.005,
              f"{dl:.4f} (expect {g0 + 0.2 * SIN30:.4f})")

    print("== planar dock paths (x, z, tilt; every y-section of the real meshes)")
    t0 = time.time()
    pairs = load_pairs(base_c, deck_c, section_ys())
    print(f"  {len(pairs)} distinct sections ({time.time() - t0:.0f} s)", flush=True)
    if "--search" in sys.argv:
        thr = float(sys.argv[sys.argv.index("--search") + 1])
        W = search(pairs, thr)
        print("  NO PATH at this threshold" if W is None else "  dock waypoints:\n" +
              "\n".join(f"    ({w[0]:.4f}, {w[1]:.4f}, {w[2]:.2f})," for w in W))
        sys.exit(0 if W else 1)
    L = hook_outline()
    lowest, xs_under = 0.0, [1e9, -1e9]
    for name, W in PATHS.items():
        W = np.array(W, float)
        pre, last = W[-2], W[-1]
        assert np.allclose(last, 0) and abs(pre[0]) < 1e-9 and abs(pre[2]) < 1e-9, "a path ends with the vertical seat drop"
        up, zl, sweep = clear_above(pairs, W[0], THRESH)
        check(f"{name}: starts clear above the deck", up,
              f"L lowest z {zl:.2f}, vertical sweep up from it {sweep:.3f}")
        P = densify(W[:-1])
        cl = clearance(pairs, P)
        i = int(np.argmin(cl))
        check(f"{name}: in-plane clearance to the pre-seat pose ({len(P)} substeps)", cl.min() >= THRESH,
              f"min {cl.min():.4f} at (tx {P[i][0]:.3f}, tz {P[i][1]:.3f}, tilt {P[i][2]:.2f}); "
              f"at tilt <= 10: {cl[P[:, 2] <= 10 + 1e-9].min():.4f}")
        # to 0.02 above docked: there the cones' 0.01 gap is mesh-chord noise (0.001 tolerance)
        D = densify([pre, (0.0, 0.02, 0.0)], max_disp=0.002)
        cd = clearance(pairs, D)
        check(f"{name}: the seat drop from tz {pre[1]:.3f} closes only the cones (gap >= dz sin 30)",
              bool(np.all(cd >= D[:, 1] * SIN30 - 0.003)) and cd.min() > 0,
              f"{cd[0]:.4f} at the pre-seat, {cd[-1]:.4f} at tz {D[-1][1]:.3f}")
        X, Z = moved(densify(W), L)
        lo, hi = under_span(X, Z)
        lowest = min(lowest, Z.min())
        xs_under = [min(xs_under[0], lo.min()), max(xs_under[1], hi.max())]
        print(f"  {name}: hook lowest z {Z.min():.2f}; under the deck x {lo.min():.2f}..{hi.max():.2f} "
              f"(r {R_STATION + lo.min():.2f}..{R_STATION + hi.max():.2f})", flush=True)

    print(f"== hook envelope at any free pose (tilt {ENV_BOX[2][0]:.0f}..{ENV_BOX[2][1]:.0f} step "
          f"{ENV_BOX[2][2]:.0f}, tx {ENV_BOX[0][2]}, tz {ENV_BOX[1][2]}; extremes refined on every section)")
    t0 = time.time()
    few = [p for p in pairs if abs(p[0]) in {0.0, 6.0, 11.0, LP["hook_lip_w"] / 2 - 0.05} or
           min(abs(abs(p[0]) - abs(sy)) for _, sy in LP["seat_xy"]) < 0.01]
    env = envelope(pairs, few, L)
    zr, rock_th, pr, free = pad_edge_rock(pairs, L)
    check(f"the rock on the pads' inboard edge (x {hook_geometry()['x0']:.2f}, -4), {2 * FREE_EPS} lifted, is free",
          free, f"tilt {rock_th:.2f}, pose {pr}: the L at {zr:.3f}")
    zlow = min(env["z"][0], zr if free else 0.0)
    check("the envelope's grid extremes lie inside its box (else widen ENV_BOX)",
          not any(e[2] for e in env.values()),
          "; ".join(f"{k} at {e[1]}" for k, e in env.items()))
    hw = LP["hook_lip_w"] / 2
    lo, hi = env["lo"][0], env["hi"][0]
    check("hook lowest z at any free pose (a tub roof at -15 / -18 must stay under it)", zlow > -15.0,
          f"{zlow:.3f} (grid + refine {env['z'][0]:.3f} at {env['z'][1]}); under the deck x {lo:.2f}..{hi:.2f} "
          f"(at {env['lo'][1]} / {env['hi'][1]}), r {R_STATION + lo:.2f}..{R_STATION + hi:.2f} on the station "
          f"axis, corners (|y| {hw:.0f}) to body radius {np.hypot(R_STATION + hi, hw):.2f}; paths "
          f"{lowest:.2f}, x {xs_under[0]:.2f}..{xs_under[1]:.2f} (r {R_STATION + xs_under[0]:.2f}.."
          f"{R_STATION + xs_under[1]:.2f}, corners {np.hypot(R_STATION + xs_under[1], hw):.2f}) "
          f"({time.time() - t0:.0f} s)")
    # the radial span was printed and never asserted (2026-10-07 review): now the whole
    # envelope is held to the record the layout's keep-outs read (iface.HOOK_ENVELOPE)
    for key, (z, x0, x1) in (("paths", (lowest, *xs_under)), ("any", (zlow, lo, hi))):
        rec = HOOK_ENVELOPE[key]
        dev = max(abs(z - rec["z"]), abs(x0 - rec["x"][0]), abs(x1 - rec["x"][1]))
        check(f"hook envelope '{key}' as recorded in iface.HOOK_ENVELOPE (+-{HOOK_ENVELOPE_TOL})",
              dev <= HOOK_ENVELOPE_TOL,
              f"z {z:.3f}, x {x0:.3f}..{x1:.3f} (r {R_STATION + x0:.3f}..{R_STATION + x1:.3f}) vs z "
              f"{rec['z']:.3f}, x {rec['x'][0]:.3f}..{rec['x'][1]:.3f}: off {dev:.3f}"
              + ("" if dev <= HOOK_ENVELOPE_TOL else "; moved on purpose: re-check the tub roof and "
                 "the keep-outs, then re-record it"))

    print(f"== 3D replay (BRep common volume + BRepExtrema distance; <= {REPLAY_DISP} of motion between poses)")
    trim = None
    for sy in (1, -1):              # the outer 0.3 of each lip end below the plate
        cut = Pos(-50.0, sy * (LP["hook_lip_w"] / 2 + 0.35), -9.5) * Box(20.0, 1.3, 11.0)
        trim = cut if trim is None else trim + cut
    for name, W in PATHS.items():
        W = np.array(W, float)
        poses = densify(W, max_disp=REPLAY_DISP, reach=REPLAY_REACH)
        for tag, b in (("", base_c), (", lip ends trimmed 0.3", base_c - trim)):
            t0 = time.time()
            worst, dmin = 0.0, (np.inf, None, None)
            for p in poses:
                m = pose_tf(*p) * b
                worst = max(worst, _v(m & deck_c))
                if p[1] >= W[-2][1] - 1e-9 or abs(p[0]) + abs(p[2]) > 1e-9:   # before the seat drop
                    d, at = dist3(m, deck_c)
                    if d < dmin[0]:
                        dmin = (d, tuple(round(float(q), 3) for q in p), at)
            check(f"{name}{tag}: {len(poses)} poses", worst < 1e-3 and dmin[0] >= THRESH - 1e-3,
                  f"overlap max {worst:.4f} mm^3; min gap before the seat drop {dmin[0]:.4f} "
                  f"at pose {dmin[1]}, base point {dmin[2]} ({time.time() - t0:.0f} s)")

    print()
    print(f"check_dock: {'CLEAN' if not fails else f'{len(fails)} FAILURES'} ({time.time() - t00:.0f} s)")
    for f in fails:
        print("  -", f)
    sys.exit(len(fails))
