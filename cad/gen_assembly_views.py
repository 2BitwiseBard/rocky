#!/usr/bin/env python3
"""Assembly pictures for the one-leg build, the foot SEA and the body, drawn
from the CAD itself. Writes PNGs to cad/out/assembly/:

  leg_01.png .. leg_08.png  the eight steps of docs/PRINT_PLAN.md "Assembly
                            order". Grey = already assembled (dark grey: a
                            servo), purple = moves in this step, pulled back
                            30 mm along the way it goes in, pale purple = new
                            in this step but stays put; the dashed arrow is
                            the motion
  leg_done.png              the leg skeleton, assembled
  leg_harness.png           the leg on a patch of deck with the 11 x 11 harness
                            path (part_coxa.harness_solid) in orange; where a
                            part hides the path it shows through as a tint
  sea_exploded.png          the foot SEA exploded along the tube axis (the
                            spring and the KW10 are stand-ins)
  leg_on_deck.png           body_deck with one coxa_yaw_base docked on
                            station 0 (leg 0, north, the one-dot port), the
                            pose part_deck.py checks
  body_interior.png         option A from below (2026-10-07), the robot
                            turned over on the bench: the deck, the hub
                            shelf with its face-down boards and the keel
                            tub (bay_tub + bay_lid + bay_door) that hang
                            under it; the avionics tray on its rails on the
                            deck's other side shows through as a tint

    python3 gen_assembly_views.py              (from cad/, ~1 min)
    python3 gen_assembly_views.py leg_03 sea_exploded    (named images; 'leg' = the steps)

The servos are servo_st3215.servo_body (the envelope measured from the real
servo's STEP) posed by leg_assembly.build(), not the dry-fit blanks. Step 1
draws the fork sliding on from the front (its C opens to the back): the servo
passes between the fork's side cheeks (labelled: from LEG_CAM the near one
hides the jaws), and the horn's centre head and the idler run out to the
mouth in channels (D063, B80). check_assembly asserts that path
(part_coxa.fork_slide_worst), onto the blank and the real servo.

How it draws: each solid is tessellated (TOL), projected by an orthographic
camera and rasterised by a small numpy z-buffer at SS times the output size
(flat shading, ink where parts meet, at silhouettes and at creases), then
averaged down; matplotlib (Agg) adds the titles, arrows and labels. One
camera per family: every leg picture looks from LEG_CAM, and steps 1-7 share
one frame, so flipping through them the leg does not move (step 8 and the
finished leg zoom out for the tube, the harness picture for its deck patch);
leg_on_deck looks from BODY_CAM (above), body_interior from UNDER_CAM (below:
since option A the body's parts hang under the deck). Byte-stable: no timestamps, fixed size
and dpi, the PNG Software tag dropped, so an unchanged model rewrites
identical files. ~3 s a picture at 1200 x 1000, ~1.2 GB peak.
"""
import functools
import os
import re
import sys
import textwrap
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.patheffects as pe                        # noqa: E402
import matplotlib.pyplot as plt                            # noqa: E402
import numpy as np                                         # noqa: E402
from build123d import Box, Cylinder, Pos, Rot              # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "assembly")

TOL, ANG_TOL = 0.05, 0.2          # tessellation: 0.05 mm chord error, 0.2 rad between facets
W, H, SS, DPI = 1200, 1000, 2, 100
MARGIN = 40                       # px round the drawing
HEAD_H = 170                      # px: title + two subtitle lines
FOOT_H = 70                       # px: the legend row
NOTE_LINE = 26                    # px per note line (above the legend)
PULL = 30.0                       # mm a moving part is pulled back from its seat

GREY = (0.80, 0.80, 0.82)
SERVO = (0.34, 0.34, 0.38)
MOVE = (0.56, 0.38, 0.86)         # pebble purple, the accent
NEW = (0.84, 0.78, 0.94)
DECK = (0.88, 0.87, 0.84)
STEEL = (0.64, 0.66, 0.72)
ORANGE = (0.96, 0.55, 0.12)
INK = np.array([0.12, 0.12, 0.15])
ARROW = "#3d1f73"
TEXT = "#1d1d24"
MUTED = "#55555f"


# --------------------------------------------------------------------------
# geometry -> triangles

def mesh(shape):
    """(N, 3, 3) triangles of a build123d shape."""
    verts, tris = shape.tessellate(TOL, ANG_TOL)
    v = np.array([(p.X, p.Y, p.Z) for p in verts], float)
    return v[np.asarray(tris, int)]


def shift(tris, d):
    return tris + np.asarray(d, float)


def helix_spring(od, wire, length, coils, n_seg=24, n_per_coil=48):
    """A compression spring (z 0..length) as a tube swept along a helix; the
    first and last turns are closed flat."""
    r = (od - wire) / 2
    n = int(coils * n_per_coil) + 1
    t = np.linspace(0.0, coils * 2 * np.pi, n)
    rise = (length - wire) / (max(coils - 2, 1) * 2 * np.pi)            # per radian, active turns
    z = np.clip(wire / 2 + (t - 2 * np.pi) * rise, wire / 2, length - wire / 2)
    c = np.stack([r * np.cos(t), r * np.sin(t), z], 1)
    tan = np.gradient(c, axis=0)
    tan /= np.linalg.norm(tan, axis=1, keepdims=True)
    nrm = np.stack([np.cos(t), np.sin(t), np.zeros_like(t)], 1)
    nrm -= (nrm * tan).sum(1, keepdims=True) * tan
    nrm /= np.linalg.norm(nrm, axis=1, keepdims=True)
    bi = np.cross(tan, nrm)
    a = np.linspace(0, 2 * np.pi, n_seg, endpoint=False)
    ring = c[:, None, :] + wire / 2 * (np.cos(a)[None, :, None] * nrm[:, None, :]
                                       + np.sin(a)[None, :, None] * bi[:, None, :])
    i, j = np.meshgrid(np.arange(n - 1), np.arange(n_seg), indexing="ij")
    k = (j + 1) % n_seg
    q00, q01, q10, q11 = ring[i, j], ring[i, k], ring[i + 1, j], ring[i + 1, k]
    return np.concatenate([np.stack([q00, q10, q11], -2).reshape(-1, 3, 3),
                           np.stack([q00, q11, q01], -2).reshape(-1, 3, 3)])


@functools.lru_cache(maxsize=None)
def _deck():
    from part_deck import body_deck
    return body_deck()


# --------------------------------------------------------------------------
# camera, z-buffer, shading

def camera(azim, elev):
    """Rows: screen right, screen up, toward the viewer (matplotlib's azim/elev)."""
    a, e = np.radians(azim), np.radians(elev)
    d = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    r = np.cross([0.0, 0.0, 1.0], d)
    r /= np.linalg.norm(r)
    return np.array([r, np.cross(d, r), d])


def draw_box(notes=0, right=0):
    """The pixel box a drawing is fitted into: under the title, over the
    legend and `notes` note lines, `right` px kept free for labels."""
    return MARGIN, HEAD_H, W - MARGIN - right, H - FOOT_H - notes * NOTE_LINE


def fit(R, pts, box=None):
    """A frame (R, px per mm, ox, oy) fitting the points into box."""
    x0, y0, x1, y1 = box or draw_box()
    q = np.asarray(pts, float).reshape(-1, 3) @ R.T
    lo, hi = q.min(0), q.max(0)
    s = min((x1 - x0) / (hi[0] - lo[0]), (y1 - y0) / (hi[1] - lo[1]))
    cx, cy = (lo[:2] + hi[:2]) / 2
    return R, s, (x0 + x1) / 2 - s * cx, (y0 + y1) / 2 + s * cy


def to_px(frame, p):
    R, s, ox, oy = frame
    q = np.asarray(p, float) @ R.T
    return np.stack([ox + s * q[..., 0], oy - s * q[..., 1]], -1)


def zbuffer(P, ws, hs, chunk=1 << 20):
    """Nearest triangle per pixel. P: (N, 3, 3) of pixel x, pixel y, depth
    (larger = nearer). Returns (triangle index or -1, depth), both (hs, ws).
    Triangles are batched by bounding-box size and every candidate pixel
    centre is tested at once; ties keep the lower index, so it is exact and
    repeatable."""
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    area = (x[:, 1] - x[:, 0]) * (y[:, 2] - y[:, 0]) - (x[:, 2] - x[:, 0]) * (y[:, 1] - y[:, 0])
    bx0 = np.maximum(np.ceil(x.min(1) - 0.5), 0).astype(np.int64)
    bx1 = np.minimum(np.floor(x.max(1) - 0.5), ws - 1).astype(np.int64)
    by0 = np.maximum(np.ceil(y.min(1) - 0.5), 0).astype(np.int64)
    by1 = np.minimum(np.floor(y.max(1) - 0.5), hs - 1).astype(np.int64)
    ids = np.nonzero((bx1 >= bx0) & (by1 >= by0) & (np.abs(area) > 1e-12))[0]
    pw = 1 << np.ceil(np.log2(bx1[ids] - bx0[ids] + 1)).astype(np.int64)
    ph = 1 << np.ceil(np.log2(by1[ids] - by0[ids] + 1)).astype(np.int64)
    key = pw * 65536 + ph
    depth = np.full(ws * hs, -np.inf)
    best = np.full(ws * hs, -1, np.int64)
    for k in np.unique(key):
        sel = ids[key == k]
        kw, kh = int(k // 65536), int(k % 65536)
        gx, gy = np.tile(np.arange(kw), kh), np.repeat(np.arange(kh), kw)
        step = max(1, chunk // (kw * kh))
        for c0 in range(0, len(sel), step):
            t = sel[c0:c0 + step]
            X, Y = bx0[t, None] + gx, by0[t, None] + gy
            cx, cy = X + 0.5, Y + 0.5
            x0, x1, x2 = x[t, 0, None], x[t, 1, None], x[t, 2, None]
            y0, y1, y2 = y[t, 0, None], y[t, 1, None], y[t, 2, None]
            a = area[t, None]
            w0 = ((x1 - cx) * (y2 - cy) - (x2 - cx) * (y1 - cy)) / a
            w1 = ((x2 - cx) * (y0 - cy) - (x0 - cx) * (y2 - cy)) / a
            w2 = 1.0 - w0 - w1
            m = (w0 >= -1e-9) & (w1 >= -1e-9) & (w2 >= -1e-9) & (X <= bx1[t, None]) & (Y <= by1[t, None])
            if not m.any():
                continue
            d = (w0 * z[t, 0, None] + w1 * z[t, 1, None] + w2 * z[t, 2, None])[m]
            pix = (Y * ws + X)[m]
            tid = np.broadcast_to(t[:, None], m.shape)[m]
            o = np.lexsort((tid, -d, pix))
            pix, d, tid = pix[o], d[o], tid[o]
            first = np.ones(len(pix), bool)
            first[1:] = pix[1:] != pix[:-1]
            pix, d, tid = pix[first], d[first], tid[first]
            nearer = d > depth[pix]
            depth[pix[nearer]] = d[nearer]
            best[pix[nearer]] = tid[nearer]
    return best.reshape(hs, ws), depth.reshape(hs, ws)


def render(items, frame, xray=None):
    """items: [(tris, rgb)] -> (H, W, 3) float image on white. xray = (tris,
    rgb): drawn like the rest, and tinted through whatever hides it."""
    R, s, ox, oy = frame
    groups = list(items) + ([xray] if xray is not None else [])
    tris = np.concatenate([t for t, _ in groups])
    gid = np.concatenate([np.full(len(t), k) for k, (t, _) in enumerate(groups)])
    rgb = np.array([c for _, c in groups], float)
    q = tris @ R.T
    P = np.empty_like(q)
    P[..., 0] = (ox + s * q[..., 0]) * SS
    P[..., 1] = (oy - s * q[..., 1]) * SS
    P[..., 2] = q[..., 2]
    ws, hs = W * SS, H * SS
    tri, dep = zbuffer(P, ws, hs)
    hit = tri >= 0
    dep = np.where(hit, dep, -1e6)
    n = np.cross(q[:, 1] - q[:, 0], q[:, 2] - q[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    n *= np.where(n[:, 2:] < 0, -1.0, 1.0)                # two-sided: face the viewer
    key = np.array([-0.35, 0.55, 0.76])
    shade = 0.46 + 0.54 * np.clip(n @ (key / np.linalg.norm(key)), 0, 1)
    col = rgb[gid] * shade[:, None]
    img = np.ones((hs, ws, 3))
    img[hit] = col[tri[hit]]
    g = np.where(hit, gid[np.maximum(tri, 0)], -1)
    nn = np.where(hit[..., None], n[np.maximum(tri, 0)], 0.0)
    edge = np.zeros((hs, ws), bool)
    for a, b in (((slice(None), slice(0, -1)), (slice(None), slice(1, None))),
                 ((slice(0, -1), slice(None)), (slice(1, None), slice(None)))):
        e = g[a] != g[b]                                          # part / part or part / ground
        same = (g[a] >= 0) & ~e & (tri[a] != tri[b])
        crease = (nn[a] * nn[b]).sum(-1) < np.cos(np.radians(38))
        jump = np.abs(dep[a] - dep[b]) > 1.2                      # mm: a fold hiding a face behind
        e |= same & (crease | jump)
        edge[a] |= e
        edge[b] |= e
    img[edge] = img[edge] * 0.12 + INK * 0.88
    if xray is not None:
        xl = len(groups) - 1
        xtri, _ = zbuffer(P[gid == xl], ws, hs)
        hidden = (xtri >= 0) & (g != xl)
        img[hidden] = img[hidden] * 0.5 + np.array(xray[1]) * 0.5
    return img.reshape(H, SS, W, SS, 3).mean((1, 3))


# --------------------------------------------------------------------------
# page: picture + title + arrows + labels + legend

def _halo(lw):
    return [pe.withStroke(linewidth=lw, foreground="white")]


def save(img, name, title, subtitle="", arrows=(), labels=(), legend=(), note=""):
    """arrows: [(start_px, end_px)]; labels: [(text, anchor_px, text_px[, ha])]
    (ha defaults to the side of the anchor the text is on).
    The subtitle gets two lines at most (HEAD_H is sized for two)."""
    fig = plt.figure(figsize=(W / DPI, H / DPI), dpi=DPI)
    fig.figimage(np.clip(img, 0, 1), 0, 0, origin="upper", zorder=-10)   # under the axes
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.axis("off")
    ax.text(MARGIN, 28, title, fontsize=27, fontweight="bold", color=TEXT, va="top")
    lines = textwrap.wrap(subtitle, 74)
    if len(lines) > 2:
        raise ValueError(f"{name}: subtitle needs {len(lines)} lines, the header has room for 2")
    for k, line in enumerate(lines):
        ax.text(MARGIN, 84 + 34 * k, line, fontsize=19, color=MUTED, va="top")
    for p0, p1 in arrows:
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        v = p1 - p0
        head = p1 - v / np.hypot(*v) * min(24.0, 0.45 * np.hypot(*v))
        ax.plot([p0[0], head[0]], [p0[1], head[1]], ls=(0, (4, 2.5)), lw=2.8, color=ARROW,
                path_effects=_halo(7), solid_capstyle="butt", dash_capstyle="butt")
        ax.annotate("", xy=tuple(p1), xytext=tuple(head),
                    arrowprops=dict(arrowstyle="-|>,head_length=0.9,head_width=0.45", lw=2.8,
                                    color=ARROW, shrinkA=0, shrinkB=0, path_effects=_halo(7)))
        ax.plot([p0[0]], [p0[1]], "o", ms=7, color=ARROW, path_effects=_halo(5))
    for text, anchor, at, *ha in labels:
        ax.annotate(text, xy=tuple(anchor), xytext=tuple(at), fontsize=17, color=TEXT,
                    ha=ha[0] if ha else ("left" if at[0] >= anchor[0] else "right"), va="center",
                    arrowprops=dict(arrowstyle="-", lw=1.4, color=MUTED, shrinkA=4, shrinkB=0),
                    path_effects=_halo(5))
    x = MARGIN
    for rgb, text in legend:
        ax.add_patch(plt.Rectangle((x, H - 48), 28, 22, facecolor=rgb, edgecolor="#333"))
        t = ax.text(x + 36, H - 37, text, fontsize=16, color=TEXT, va="center")
        x += 36 + t.get_window_extent(fig.canvas.get_renderer()).width + 30
    for k, line in enumerate(textwrap.wrap(note, 92)[::-1]):
        ax.text(MARGIN, H - FOOT_H - 4 - NOTE_LINE * k, line, fontsize=15, color=MUTED, va="bottom")
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{name}.png")
    fig.savefig(path, dpi=DPI, metadata={"Software": None})
    plt.close(fig)
    return path


def note_lines(note):
    return len(textwrap.wrap(note, 92))


# --------------------------------------------------------------------------
# the leg

LEG_CAM = (-52, 24)
SERVOS = ("servo_yaw", "servo_femur", "servo_tibia")
ORDER = ["servo_yaw", "coxa_fork", "coxa_yaw_base", "servo_femur", "tibia_knee_carrier", "servo_tibia",
         "coupler_hip", "coupler_knee", "femur_link", "femur_plate_b", "tibia_tube"]
LEGEND = [(MOVE, "moves in this step"), (NEW, "new, stays put"), (GREY, "assembled"),
          (SERVO, "servo, assembled")]


def leg_steps_table():
    """[(title, subtitle, new parts that stay put, parts that move, unit motion
    to the seat, arrow seat points or None = the moved parts' leading face)].
    The text is docs/PRINT_PLAN.md's; the directions are check_assembly's
    (servos slide in along -X, couplers + plate A come on along +Y, plate B
    drops on along -Y)."""
    from leg_frame import L1, KNEE_X, Z_HIP, HUB_Z1, ZC_YAW
    from part_femur import YA1, YB0
    from servo_st3215 import case_xspan
    mx, py, my, pz = (-1.0, 0, 0), (0, 1.0, 0), (0, -1.0, 0), (0, 0, 1.0)
    return [
        ("Fork onto the yaw servo", "Slide it on from the front, between its cheeks; horn head and "
         "idler run in channels. Hold it seated: 4 × M3 × 6 from below into the horn.",
         ["servo_yaw"], ["coxa_fork"], mx, [(0.0, 0.0, HUB_Z1)]),
        ("Yaw servo + fork into the base", "Slide them into the base cup from the front. "
         "2 self-tappers from above into the rim, 2 from under the plate.",
         ["coxa_yaw_base"], ["servo_yaw", "coxa_fork"], mx, [(case_xspan()[0], 0.0, ZC_YAW)]),
        ("Hip servo into the fork's cup", "Slide it in from the front; 4 self-tappers into the "
         "rim holes.", [], ["servo_femur"], mx, None),
        ("Knee servo into the carrier", "Slide it into the tibia_knee_carrier's cup from the "
         "front; 4 self-tappers into the rim holes.",
         ["tibia_knee_carrier"], ["servo_tibia"], mx, None),
        ("Couplers onto the hip + knee horns", "Disc on the horn, lobes outward; 4 screws each, "
         "driven through the counterbores.", [], ["coupler_hip", "coupler_knee"], py, None),
        ("Plate A onto both couplers", "femur_link: the hub recesses take the coupler lobes; "
         "2 × M3 × 8 clamps per hub from the outer face.",
         [], ["femur_link"], py, [(L1, YA1, Z_HIP), (KNEE_X, YA1, Z_HIP)]),
        ("Plate B over both idlers", "femur_plate_b: its towers ride the idlers; 4 × M3 × 12 "
         "into the bridge bosses.",
         [], ["femur_plate_b"], my, [(L1, YB0, Z_HIP), (KNEE_X, YB0, Z_HIP)]),
        ("Tube into the carrier boss", "Push the carbon tube up into the boss socket; "
         "M3 × 10 pinch bolt.", [], ["tibia_tube"], pz, None),
    ]


def leg_meshes():
    from leg_assembly import build
    parts = build()
    return {k: mesh(parts[k]) for k in ORDER}


def _arrows(frame, moved, u, seats):
    """[(start px, end px)]: from the moved part to its seat, PULL long."""
    u = np.asarray(u, float)
    if seats is None:                    # each moved part's leading face, centred
        out = []
        for t in moved:
            v = t.reshape(-1, 3)
            c = (v.min(0) + v.max(0)) / 2
            p0 = c + u * ((v - c) @ u).max()
            out.append((to_px(frame, p0), to_px(frame, p0 + u * PULL)))
        return out
    return [(to_px(frame, np.asarray(p) - u * PULL), to_px(frame, p)) for p in seats]


def leg_step_labels():
    """{step: [(text, a point on the moving part at its seat (mm), text px)]}. Step 1:
    from LEG_CAM the fork's near side cheek (D063) hides the C's jaws, so name it."""
    from part_coxa import CHEEK_X0, WEB_X0, CHEEK_Y1, HUB_Z1, UP_Z0
    return {1: [("side cheek (one each side)",
                 ((CHEEK_X0 + WEB_X0) / 2, -CHEEK_Y1, HUB_Z1 + 0.15 * (UP_Z0 - HUB_Z1)), (375, 885))]}


def leg_steps(m, frames, want):
    out, done = [], []                   # done: assembled by the end of the previous step
    step_labels = leg_step_labels()
    for k, (title, sub, new, moving, u, seats) in enumerate(leg_steps_table(), 1):
        if want(f"leg_{k:02d}"):
            frame = frames[k - 1]
            items = [(m[p], SERVO if p in SERVOS else GREY) for p in done if p not in moving]
            items += [(m[p], NEW) for p in new]
            moved = [shift(m[p], [-PULL * c for c in u]) for p in moving]
            items += [(t, MOVE) for t in moved]
            labels = [(t, to_px(frame, np.asarray(a) - PULL * np.asarray(u)), at)
                      for t, a, at in step_labels.get(k, [])]
            out.append(save(render(items, frame), f"leg_{k:02d}", f"Step {k}: {title}", sub,
                            arrows=_arrows(frame, moved, u, seats), labels=labels, legend=LEGEND))
        done += [p for p in new + moving if p not in done]
    return out


def leg_frame_pts(m, steps):
    """Every point the given steps (1-based) show, so they can share a frame."""
    table = leg_steps_table()
    pts = []
    for k in steps:
        _, _, new, moving, u, _ = table[k - 1]
        last = max(ORDER.index(p) for p in new + moving)
        pts += [m[p].reshape(-1, 3) for p in ORDER[:last + 1]]
        pts += [shift(m[p], [-PULL * c for c in u]).reshape(-1, 3) for p in moving]
    return np.concatenate(pts)


def leg_done(m, frame):
    items = [(m[p], SERVO if p in SERVOS else GREY) for p in ORDER]
    return save(render(items, frame), "leg_done", "One leg, assembled",
                "After step 8: three ST3215s in their cups, plates A + B on the couplers and "
                "idlers, the tube in the carrier boss.",
                legend=[(GREY, "printed"), (SERVO, "ST3215 servo")])


def leg_harness(m):
    """The leg (no tube) on a patch of deck, the harness path through it. Same
    camera as the steps, zoomed to fit the deck patch."""
    from part_coxa import harness_solid
    from part_deck import STATIONS, R_STATION
    deck = Pos(-R_STATION, 0, 0) * Rot(0, 0, -STATIONS[0]) * Pos(0, 0, -10) * _deck()   # part_deck's pose
    patch = mesh(deck & (Pos(-18, 0, -7) * Box(84, 76, 8)))
    harness = mesh(harness_solid())
    items = [(patch, DECK)] + [(m[p], SERVO if p in SERVOS else GREY) for p in ORDER if p != "tibia_tube"]
    note = ("Where a part hides the path it shows through as an orange tint. The tube is left out; "
            "check_assembly keeps the path clear of the whole leg at every yaw.")
    frame = fit(camera(*LEG_CAM), np.concatenate([t.reshape(-1, 3) for t, _ in items]),
                draw_box(notes=note_lines(note)))
    return save(render(items, frame, xray=(harness, ORANGE)), "leg_harness", "Leg harness path, 11 × 11",
                "The leg drop (XT30 + JST-XH-5): deck cable cutout, the base's channel, up beside "
                "the cup, over the top to the yaw servo's plugs.",
                legend=[(ORANGE, "harness path"), (DECK, "deck"), (GREY, "printed"), (SERVO, "servo")],
                note=note)


# --------------------------------------------------------------------------
# the foot SEA

def sea_exploded():
    from part_tibia import tibia_sea_outer, tibia_sea_slider, TUBE_OD
    flip = Rot(180, 0, 0)          # SEA-local +Z runs down the shin; here the knee is up, z 0 = tube end
    outer = mesh(flip * tibia_sea_outer())                   # z 0 .. -28, socket -11 .. 0
    tube_len = 26.0                                          # a stub of it, seated in the socket
    tube = mesh(Pos(0, 0, (tube_len - 11) / 2) * Cylinder(TUBE_OD / 2, tube_len + 11))
    gap, spring_len = 16.0, 20.0                             # BOM A-16: 11 OD x 0.8 x 20 FL
    spring_top = -28 - gap
    spring = shift(helix_spring(11.0, 0.8, spring_len, 6.5), (0, 0, spring_top - spring_len))
    slider_top = spring_top - spring_len - gap
    slider = shift(mesh(flip * tibia_sea_slider()), (0, 0, slider_top))
    seat = np.array([8.2, 0.0, -24.0])                       # the KW10 pocket (SEA-local 8.2, 0, 24)
    out = 24.0                                               # pulled out radially, the way it goes in
    switch = mesh(Pos(*(seat + (out, 0, 0))) * Box(6, 13, 6))
    items = [(tube, GREY), (outer, MOVE), (spring, STEEL), (switch, SERVO), (slider, MOVE)]
    R = camera(-58, 16)
    note = "Spring and switch are stand-ins (BOM A-16, A-17): caliper the real parts first."
    frame = fit(R, np.concatenate([t.reshape(-1, 3) for t, _ in items]),
                draw_box(notes=note_lines(note), right=470))
    arrows = [(to_px(frame, (0, 0, spring_top + 1)), to_px(frame, (0, 0, -28 - 1))),
              (to_px(frame, (0, 0, slider_top + 1)), to_px(frame, (0, 0, spring_top - spring_len - 1))),
              (to_px(frame, seat + (out - 4, 0, 0)), to_px(frame, seat + (5, 0, 0)))]
    lx = W - MARGIN - 440
    lab = [("tube, seated in the socket", (TUBE_OD / 2, 0, 16), 16),
           ("tibia_sea_outer", (9.5, 0, -6), -6),
           ("KW10 switch (stand-in)", seat + (out + 3, 0, 1), -24),
           ("spring 11 OD × 20 (stand-in)", (5.5, 0, spring_top - spring_len / 2),
            spring_top - spring_len / 2),
           ("tibia_sea_slider", (TUBE_OD / 2, 0, slider_top - 17), slider_top - 17)]
    labels = [(t, to_px(frame, a), (lx, to_px(frame, (0, 0, zt))[1])) for t, a, zt in lab]
    return save(render(items, frame), "sea_exploded", "Foot SEA, exploded along the tube",
                "Tube glued into the outer; spring, then slider, up into its bore; the KW10 "
                "in the side pocket, where the slider's striker closes it.",
                arrows=arrows, labels=labels, note=note)


# --------------------------------------------------------------------------
# the body

BODY_CAM = (58, 34)             # from the north-east: station 0's (leg 0's) port in front


def leg_on_deck():
    from part_deck import STATIONS, R_STATION
    from part_coxa import coxa_yaw_base
    deck = mesh(Pos(0, 0, -10) * _deck())                                   # part_deck's docking pose
    base = mesh(Rot(0, 0, STATIONS[0]) * Pos(R_STATION, 0, 0) * coxa_yaw_base())
    frame = fit(camera(*BODY_CAM), np.concatenate([deck.reshape(-1, 3), base.reshape(-1, 3)]))
    a = to_px(frame, (0, R_STATION + 14, 0))                        # the port plate, outboard
    return save(render([(deck, DECK), (base, MOVE)], frame), "leg_on_deck",
                "Leg port I1: a coxa base on the deck",
                "Leg 0 (north, one dot), its carapace sector off. Hook at 7-10°, radially; "
                "plug the leg drop, lower onto the cones; 2 thumbscrews.",
                labels=[("coxa_yaw_base", a, (a[0] - 220, a[1] + 50))],
                legend=[(MOVE, "coxa_yaw_base"), (DECK, "body_deck")])


UNDER_CAM = (-60, 34)           # the robot turned over (about x): from the north-east, below it


def body_interior():
    """Option A (the owner's picks, 2026-10-07): the robot turned over on the bench, its belly up.
    Everything is posed in the BODY frame (deck z -10 .. -4) by its own module, then turned over
    about x (`turn`): part_busboard.hub_shelf + its five boards (three face-down under the
    plate, now on top), part_bay's tub, lid and door, and part_avionics.tray_assembly (the tray
    turned 180 at (0, 0) on its rails, its latch boss, the Pi) under the deck, drawn as the x-ray
    tint where the deck hides it."""
    import part_bay as PBY
    from part_avionics import tray_assembly
    from part_busboard import hub_shelf, board_box, SHELF_BOARDS
    turn = Rot(180, 0, 0)                                             # (x, y, z) -> (x, -y, -z)
    deck = mesh(turn * Pos(0, 0, -10) * _deck())                      # part_deck's pose, z -10 .. -4
    shelf = mesh(turn * hub_shelf())
    boards = np.concatenate([mesh(turn * board_box(n, halo=False)) for n in SHELF_BOARDS])
    tub = mesh(turn * PBY.bay_tub())
    lid = mesh(turn * PBY.bay_lid())
    door = mesh(turn * PBY.bay_door())
    ta = tray_assembly()
    tray = np.concatenate([mesh(turn * s) for s in [ta["tray"], ta["boss"], ta["pi"]] + ta["rails"]])
    note = ("Turned over about its east-west axis, so north is toward you. Not shown: the coxa bases "
            "and the legs; the battery sled rides inside the tub. The 14 AWG feed leaves the NE nose "
            "chamfer, runs under leg 4's drop and rises in the clip to the 12 V node (B133).")
    items = [(deck, DECK), (shelf, NEW), (boards, SERVO), (lid, GREY), (tub, MOVE), (door, MOVE)]
    pts = np.concatenate([t.reshape(-1, 3) for t, _ in items + [(tray, ORANGE)]])
    frame = fit(camera(*UNDER_CAM), pts, draw_box(notes=note_lines(note)))
    t_ext = PBY.EXT
    xn = t_ext["x"][1] - PBY.CHAMF / 2
    lab = [("bay_tub (its floor)", (0.0, -t_ext["y"][0] - 10, -t_ext["z"][0]), (780, 240)),
           ("bay_door", (t_ext["x"][0] + 1.0, 10.0, -t_ext["z"][0]), (40, 260)),
           ("hub_shelf + boards", (-45.0, -20.0, 39.3), (40, 760)),
           ("riser clip", (PBY.CLIP_XY[0], -PBY.CLIP_XY[1] - 3, -PBY.CLIP_Z[0] - 3), (930, 420)),
           ("14 AWG exit", (xn, -(t_ext["y"][1] - PBY.CHAMF / 2), -PBY.AWG_Z), (1000, 520))]
    lab = [(t, to_px(frame, p), at, "left") for t, p, at in lab]
    return save(render(items, frame, xray=(tray, ORANGE)),
                "body_interior", "The body from below (option A)",
                "The keel tub east-west (door -x, nose +x) and the hub shelf north of it hang "
                "from the deck; the tray on top shows through it.",
                labels=lab, note=note,
                legend=[(MOVE, "bay_tub + bay_door"), (GREY, "bay_lid"), (NEW, "hub_shelf"),
                        (SERVO, "hub boards"), (DECK, "body_deck"), (ORANGE, "tray (x-ray)")])


# --------------------------------------------------------------------------

def main(argv):
    """argv: image names to redraw (e.g. leg_03 sea_exploded; 'leg' = the eight steps); none = all."""
    t0 = time.time()

    def want(n):
        return not argv or n in argv or ("leg" in argv and re.fullmatch(r"leg_\d\d", n) is not None)

    paths = []
    if any(want(n) for n in [f"leg_{k:02d}" for k in range(1, 9)] + ["leg_done", "leg_harness"]):
        m = leg_meshes()
        R = camera(*LEG_CAM)
        upper = fit(R, leg_frame_pts(m, range(1, 8)))      # steps 1-7: no tube yet
        whole = fit(R, leg_frame_pts(m, [8]))              # step 8 and the finished leg
        paths += leg_steps(m, [upper] * 7 + [whole], want)
        if want("leg_done"):
            paths.append(leg_done(m, whole))
        if want("leg_harness"):
            paths.append(leg_harness(m))
    for name, fn in (("sea_exploded", sea_exploded), ("leg_on_deck", leg_on_deck),
                     ("body_interior", body_interior)):
        if want(name):
            paths.append(fn())
    for p in paths:
        print(f"wrote {os.path.relpath(p, HERE)} ({os.path.getsize(p) / 1024:.0f} KB)")
    print(f"gen_assembly_views: {len(paths)} images in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main(sys.argv[1:])
