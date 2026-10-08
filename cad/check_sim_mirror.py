#!/usr/bin/env python3
"""The sim's copies of the CAD, checked against the CAD (review 9q).

The sim cannot import build123d, so two of its files restate CAD geometry:

  * sim/mass_audit.py's pose table: the frame constants it poses the torso items with
    (TRAY_GAP_DRAWN, TRAY_REST_DROP, TONGUE_Y, RAIL_POSE_X, PI_BOARD_DX, SLED_RAIL_H, SLED_T,
    YAW_HUB_Z1, SHELF_BOARDS, DECK_TOP_Z), each a mirror of a part module's constant;
  * gait/rocky_model.py's belly: bay_tub_extent_mm() (iface.bay_tub_extent's sums) and
    belly_boxes() / belly_posts() (the shelf plate's box and its Ø pad_d post columns), the
    sim's contact geoms, the URDF's collisions and pebble_feasibility's SELF_CONTACT.

CI's design-pipeline diff regenerates the budget and the MJCF with the same mirrors, so a
part change used to move the robot's real CoM and its contact geoms with nothing failing.
This asserts every mirror against the module that owns the number, and that the exported
bay_tub / hub_shelf (cad/out STLs: run_all_checks runs this in its POST stage, after every
module has exported) stand outside the sim's boxes only where iface.BAY_TUB_OUTSIDE lists.

Exit code 0 = every mirror holds.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sim"))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))

import iface                                    # noqa: E402
import leg_frame                                # noqa: E402
import part_avionics as PA                      # noqa: E402
import part_battery as PBAT                     # noqa: E402
import part_bay as PBAY                         # noqa: E402
import part_busboard as PBB                     # noqa: E402
import mass_audit as MA                         # noqa: E402
import rocky_model as rm                        # noqa: E402

TOL = 1e-6          # the mirrors are the same sums: equal to float noise
STL_TOL = 0.02      # an STL vertex past a box face by more than this is outside it
# the shelf plate's outline is the boards' rectangles + the Ø pad_d pads, closed with NOTCH_R
# (part_busboard.plate_outline): the closing's fillets between a pad and the plate edge stand past
# the sim's box + columns, 0.60 at the most (at (18.4, 64.9), 2026-10-08). Report it, gate at 1.0:
# a contact geom 0.6 short is nothing to the sim, a plate grown by millimetres is
SHELF_SLACK = 1.0

bad = []


def need(name, got, want, tol=TOL):
    ok = bool(np.allclose(np.asarray(got, float), np.asarray(want, float), atol=tol, rtol=0))
    print(f"  {name}: sim {np.round(np.asarray(got, float), 4).tolist()} vs CAD "
          f"{np.round(np.asarray(want, float), 4).tolist()} ({'OK' if ok else 'MISMATCH'})")
    if not ok:
        bad.append(name)


def stl_points(stem):
    path = os.path.join(HERE, "out", f"{stem}.stl")
    raw = open(path, "rb").read()
    if raw.startswith(b"version https://git-lfs"):
        raise SystemExit(f"cad/out/{stem}.stl is a git-LFS pointer: run 'git lfs checkout' (or the part module)")
    n = int.from_bytes(raw[80:84], "little")
    pts = np.frombuffer(raw, MA._STL_DT, count=n, offset=84)["v"].reshape(-1, 3).astype(float)
    return np.unique(pts.round(4), axis=0)


def in_box(pts, box, tol=STL_TOL):
    lo = np.array([b[0] for b in box]) - tol
    hi = np.array([b[1] for b in box]) + tol
    return np.all((pts >= lo) & (pts <= hi), axis=1)


print("sim/mass_audit.py's pose table vs the part modules:")
AT = iface.IF["avionics_tray"]
need("DECK_TOP_Z = part_avionics.DECK_TOP (and rocky_model's deck bottom = iface.DECK_BOT_Z)",
     [MA.DECK_TOP_Z, rm.DECK_BOT_Z_MM], [PA.DECK_TOP, iface.DECK_BOT_Z])
need("TRAY_GAP_DRAWN + tray_lift = part_avionics.SEAT_Z (the centred plate over the deck top)",
     MA.TRAY_GAP_DRAWN + float(AT["tray_lift"]), PA.SEAT_Z)
need("TRAY_REST_DROP = part_avionics.SEAT_Z - TRAY_Z (the tray resting on the channel floor)",
     MA.TRAY_REST_DROP, PA.SEAT_Z - PA.TRAY_Z)
need("tray_plate_z() = DECK_TOP + part_avionics.TRAY_Z; its boss height = BOSS_H",
     [MA.tray_plate_z(), MA.tray_plate_z() - MA.DECK_TOP_Z], [PA.DECK_TOP + PA.TRAY_Z, PA.BOSS_H])
need("TONGUE_Y, TRAY_T = part_avionics.TONGUE_Y, T", [MA.TONGUE_Y, MA.TRAY_T], [PA.TONGUE_Y, PA.T])
need("RAIL_POSE_X = part_avionics.RAIL_POSE_X", MA.RAIL_POSE_X, PA.RAIL_POSE_X)
need("PI_BOARD_DX = |part_avionics.PI_BOARD_DX| (the sign is the USB end's)", MA.PI_BOARD_DX, abs(PA.PI_BOARD_DX))
need("SLED_RAIL_H = part_bay.SLED_Z0 - ZF; sled_z() = SLED_Z0", [MA.SLED_RAIL_H, MA.sled_z()],
     [PBAY.SLED_Z0 - PBAY.ZF, PBAY.SLED_Z0])
need("SLED_T = part_battery.SLED_T", MA.SLED_T, PBAT.SLED_T)
need("YAW_HUB_Z1 = leg_frame.HUB_Z1; the yaw case's mid-plane = leg_frame.ZC_YAW",
     [MA.YAW_HUB_Z1, MA.yaw_case_leg()[0][2]], [leg_frame.HUB_Z1, leg_frame.ZC_YAW])
for k in sorted(set(MA.SHELF_BOARDS) | set(PBB.SHELF_BOARDS)):
    b = PBB.SHELF_BOARDS.get(k)
    need(f"SHELF_BOARDS[{k!r}] (w, l, stack) = part_busboard.SHELF_BOARDS",
         MA.SHELF_BOARDS.get(k, (np.nan,) * 3), (b["w"], b["l"], b["stack"]) if b else (np.nan,) * 3)

print("gait/rocky_model.py's belly vs the CAD:")
ext = iface.bay_tub_extent()
tub = rm.bay_tub_extent_mm()
need("bay_tub_extent_mm() = iface.bay_tub_extent() x / y / z", [tub[k] for k in "xyz"], [ext[k] for k in "xyz"])
need("belly_boxes()['belly_tub'] = that box", rm.belly_boxes()["belly_tub"], [ext[k] for k in "xyz"])
posts = list(rm.belly_posts().values())
need("belly_posts(): xy = part_busboard.POSTS", [p[0] for p in posts], PBB.POSTS)
need("belly_posts(): Ø = part_busboard.PAD_D (params hub_shelf.pad_d)", [2 * p[1] for p in posts],
     [PBB.PAD_D] * len(posts))
need("belly_posts(): z = part_busboard's plate underside .. the deck's underside", [p[2] for p in posts],
     [(PBB.PLATE_BOT, iface.DECK_BOT_Z)] * len(posts))

# the exported parts against the sim's boxes: every STL vertex inside the box, or inside a listed
# protrusion (a vertex test: a facet between two kept vertices could still cut a corner of the
# union, but each listed block is box-like and touches the box, so that is not a gap in practice)
pts = stl_points("bay_tub")
box = [ext[k] for k in "xyz"]
inside = in_box(pts, box)
for name, b in iface.BAY_TUB_OUTSIDE.items():
    b = tuple(ext["x"] if r is None else r for r in b)
    hit = in_box(pts, b) & ~inside
    inside |= in_box(pts, b)
    print(f"    bay_tub outside the sim box in '{name}': {int(hit.sum())} vertices")
out = pts[~inside]
msg = "none" if not len(out) else f"x {out[:, 0].min():.2f}..{out[:, 0].max():.2f}, y {out[:, 1].min():.2f}.." \
    f"{out[:, 1].max():.2f}, z {out[:, 2].min():.2f}..{out[:, 2].max():.2f}"
print(f"  bay_tub (cad/out STL, {len(pts)} vertices) outside the sim's tub box and iface.BAY_TUB_OUTSIDE: "
      f"{len(out)} ({msg}) ({'OK' if not len(out) else 'UNLISTED PROTRUSION'})")
if len(out):
    bad.append("bay_tub outside the sim box")
pts = stl_points("hub_shelf")
sb = rm.belly_boxes()["belly_shelf"]
lo, hi = np.array([b[0] for b in sb]), np.array([b[1] for b in sb])
d = np.linalg.norm(np.maximum(0.0, np.maximum(lo - pts, pts - hi)), axis=1)      # outside the box
for (x, y), r, (z0, z1) in posts:                                                 # ... and each column
    dz = np.maximum(0.0, np.maximum(z0 - pts[:, 2], pts[:, 2] - z1))
    d = np.minimum(d, np.hypot(np.maximum(0.0, np.hypot(pts[:, 0] - x, pts[:, 1] - y) - r), dz))
i = int(d.argmax())
ok = d[i] <= SHELF_SLACK
print(f"  hub_shelf (cad/out STL, {len(pts)} vertices) past belly_shelf + the post columns: {int((d > STL_TOL).sum())} "
      f"vertices, {d[i]:.2f} at the most (at ({pts[i, 0]:.1f}, {pts[i, 1]:.1f}, {pts[i, 2]:.1f}): the plate's NOTCH_R "
      f"fillets; gated at {SHELF_SLACK:g}) ({'OK' if ok else 'THE SIM MISSES PART OF THE SHELF'})")
if not ok:
    bad.append("hub_shelf outside the sim's shelf")

print(f"check_sim_mirror: {'CLEAN' if not bad else 'FAIL — ' + '; '.join(bad)}")
raise SystemExit(1 if bad else 0)
