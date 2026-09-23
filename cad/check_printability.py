#!/usr/bin/env python3
"""Print-physics audit of every printable STL in its PRINT orientation (D038).

D036/D037 proved a part can pass every boolean check and still be unprintable
because the checks live in CAD space, not on the build plate. This module
slices each printable exactly the way the slicer will — layer by layer, in
the orientation the print plan prescribes — and asks the questions a slicer
preview answers only if you happen to scroll to the right layer:

  islands     a layer region with NOTHING under it in the previous layer.
              'floating' = no material anywhere below it either (it would be
              extruded into air — the D036 lugs, in print space);
              'supported' = material exists lower down (needs support).
  overhang    per-layer area that steps out more than OVERHANG_STEP beyond
              the layer below (≈ >55° from vertical at 0.2 mm) — support or
              droop; reported as total mm² and the widest single step.
  thin walls  regions thinner than THIN_MM anywhere in a sampled layer —
              morphological opening (erode + dilate by t/2) removes exactly
              the features thinner than t; the removed area is reported.
              < 0.8 mm = fewer than two perimeters → will not print.
  bed         footprint vs the Prusa i3 (250 × 210 × 210).

Usage:  python3 check_printability.py            # every registered part
        python3 check_printability.py hand_hub   # one part, verbose layers
Exit code = number of HARD failures (floating islands, unprintable thin
walls, bed). Supported islands / overhangs are reported, not failed — they
are what 'supports ON' means in the print plan, and the plan is checked
against these numbers.
"""
import json
import os
import sys
import numpy as np
import trimesh
from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union
import warnings
warnings.filterwarnings("ignore")

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

LAYER = 0.2
OVERHANG_STEP = 0.30       # mm of outward step per layer before it's an overhang
CANTILEVER_ADJ = 0.6      # boundary fraction touching the layer below: less = cantilever
CANTILEVER_HARD = 3.0     # mm of unsupported cantilever that fails a 'no support' plan
THIN_HARD = 0.8            # < 2 perimeters at 0.4 mm: unprintable
THIN_SOFT = 1.6            # < 4 perimeters: weak, worth knowing
THIN_MIN_AREA = 2.0        # mm² for a single thin blob before it's a wall (feather-edges are ~0.2)
ISLAND_MIN_AREA = 0.5      # mm² — below this it's tessellation noise
BED = (250.0, 210.0, 210.0)
THIN_EVERY = 5             # sample thin walls every N layers (speed)


def rot(axis, deg):
    return trimesh.transformations.rotation_matrix(np.deg2rad(deg), axis)


def lay_on_face_normal(n):
    """Rotation that turns outward face normal n to point at -Z (that face
    becomes the bed contact)."""
    return trimesh.geometry.align_vectors(np.asarray(n, float), [0, 0, -1.0])


# part -> print-pose transform (as the print plan prescribes). None = native.
ORIENT = {
    # hand: boss UP = flip (collar ring on the bed)
    "hand_hub": rot([1, 0, 0], 180),
    "hand_cam": None,
    # finger on its wedge side face: the +58° sector plane contains Z; its
    # outward normal is (-sin58, cos58, 0)
    "hand_finger": lay_on_face_normal([-np.sin(np.deg2rad(58)), np.cos(np.deg2rad(58)), 0]),
    "tibia_sea_slider": None, "tool_hook": None, "tool_scoop": None,
    # leg
    "coxa_yaw_base": None,                        # plate DOWN
    "coxa_fork": None,                            # lower hub DOWN, upright as assembled (D047)
    "femur_link": lay_on_face_normal([0, -1, 0]), # plate A outer face DOWN: recesses up, bridge walls vertical
    "femur_plate_b": lay_on_face_normal([0, 1, 0]),   # outer face DOWN: idler pockets up
    "tibia_knee_carrier": None, "tibia_sea_outer": None,
    "horn_coupler": None,
    "coupon_cup": lay_on_face_normal([-1, 0, 0]),     # D047 coupons: cup back wall DOWN
    "coupon_yaw_hub": None,
    "coupon_hip_hub": lay_on_face_normal([0, -1, 0]),
    "coupon_idler": lay_on_face_normal([0, 1, 0]),
    "servo_cup": lay_on_face_normal([-1, 0, 0]),
    "servo_blank": None,                          # bottom DOWN (D047: rims on the bed, horn up)
    "blank_idler": None,
    # body / stand / shell
    "body_deck": None, "stand_base": None, "stand_crown": None,
    "stand_section": None, "shell_sector": None, "shell_cap": None,
    # coupons + small parts
    "fit_ladder": None, "latch_housing": None, "latch_rotor": None,
    "thumb_knob_m3": None, "dovetail_male_coupon": None, "dovetail_shoe": None,
    "coupler_recess_demo": None, "port_coupon_deck": None,
    "port_coupon_plate": None, "frame_coupon": None,
    # bench / misc
    "jig_base": None,
    "jig_column": rot([0, 1, 0], -90),           # on its back, spine down
    "calib_gauge_knee": None,
    "calib_gauge_hip": None,   # standing (audit: both lying poses island a 150-270 mm² face)
    "trim_cup": None, "trim_cup_lid": None, "belly_skid": None,
    "imu_grommet": None, "busboard_bracket": None, "battery_sled": None,
    "sled_rail": None, "avionics_tray": None, "tray_rail": None,
    "belly_door": None, "shell_sector_demo": None, "tube_clip": None,
    "link_clip": None, "dock_base": None, "dock_block": None,
    "dock_tower": None, "whisker_shoe": None, "foot_pad_tpu": None,
}

# what the print plan says about supports for each part:
#   'yes'  — plan prescribes supports: islands are expected, reported only
#   'none' — plan says NO support: any island is a HARD failure
#   absent — plan is silent: islands are a SURPRISE warning (fix the plan)
# walls thinner than 0.8 mm that are DELIBERATE (printed flexures)
THIN_OK = {"dock_block"}

SUPPORT_POLICY = {
    "hand_hub": "yes", "tool_hook": "yes", "tool_scoop": "yes",
    "coxa_yaw_base": "yes", "tibia_knee_carrier": "yes",
    "shell_sector": "yes", "jig_column": "yes", "hand_finger": "yes",
    "servo_blank": "none", "calib_gauge_hip": "none", "stand_crown": "yes",
    "coxa_fork": "yes",   # only a 3 x 22 mm strip under the -X collar
    "hand_cam": "none", "femur_link": "none",
    "femur_plate_b": "none", "coupon_cup": "none", "coupon_yaw_hub": "none",
    "coupon_hip_hub": "none", "coupon_idler": "none", "servo_cup": "none",
    "blank_idler": "none",
    "tibia_sea_outer": "none", "horn_coupler": "none", "fit_ladder": "none",
    "body_deck": "none",
}


def _polys(path2d):
    """Path2D -> list of shapely polygons (with holes)."""
    out = []
    if path2d is None:
        return out
    for p in path2d.polygons_full:
        if p is None or p.is_empty:
            continue
        if p.area >= 1e-6:
            out.append(p)
    return out


def _union(polys):
    if not polys:
        return Polygon()
    return unary_union(polys)


def _width_of_blob(g):
    """Local width of a blob = 2 x its largest inscribed radius (bisection
    on negative buffers). A ring-shaped overhang reports its band width,
    not its diameter — the unsupported span that actually matters."""
    lo, hi = 0.0, 60.0
    for _ in range(9):
        mid = (lo + hi) / 2
        if g.buffer(-mid / 2).is_empty:
            hi = mid
        else:
            lo = mid
    return float(lo)


def audit(name, verbose=False):
    path = os.path.join(OUT, f"{name}.stl")
    m = trimesh.load(path, force="mesh")
    T = ORIENT.get(name)
    if T is not None:
        m.apply_transform(T)
    m.apply_translation(-m.bounds[0])          # bed at z=0, corner at origin
    ext = m.extents
    res = {"part": name, "size": [round(float(e), 1) for e in ext],
           "bed_ok": bool(sorted(ext[:2])[0] <= BED[1] and
                          sorted(ext[:2])[1] <= BED[0] and ext[2] <= BED[2])}
    zs = np.arange(LAYER / 2, ext[2], LAYER)
    sections = m.section_multiplane(plane_origin=[0, 0, 0],
                                    plane_normal=[0, 0, 1], heights=zs)
    layers = [_union(_polys(s)) for s in sections]
    prev = Polygon()
    ray = trimesh.ray.ray_triangle.RayMeshIntersector(m)
    floating, supported = [], []

    def has_material_below(g, z):
        # a few downward rays from inside the island: any hit below = the
        # island can stand on part-support; none = it needs bed support
        pts = [g.representative_point(), g.centroid]
        try:
            pts += [g.buffer(-0.3).representative_point()]
        except Exception:
            pass
        origins = np.array([[q.x, q.y, z - LAYER] for q in pts if not q.is_empty])
        if not len(origins):
            return False
        dirs = np.tile([0, 0, -1.0], (len(origins), 1))
        hits = ray.intersects_any(origins, dirs)
        return bool(np.any(hits))
    overhang_area, widest_step, widest_z = 0.0, 0.0, None
    # D046 (2026-09-17): a CANTILEVER is an overhang blob whose boundary
    # mostly does NOT touch the layer below (a disc on a boss); a BRIDGE
    # (roof over a bore) touches it all round. The v0.1 servo blank's Ø20
    # horn disc on a Ø6 boss — 7 mm of 90° cantilever, "no support" — only
    # ever produced a warning here.
    widest_cant, widest_cant_z = 0.0, None
    thin_hard_max, thin_soft_max, thin_hard_z = 0.0, 0.0, None
    prev_thin = []
    max_layer_area = 0.0
    for k, (z, cur) in enumerate(zip(zs, layers)):
        if cur.is_empty:
            prev = cur
            continue
        max_layer_area = max(max_layer_area, cur.area)
        if k > 0:
            # islands: connected regions of this layer with no overlap below
            geoms = list(cur.geoms) if isinstance(cur, MultiPolygon) else [cur]
            for g in geoms:
                if g.area < ISLAND_MIN_AREA:
                    continue
                if g.intersection(prev).area < 1e-6:
                    rec = {"z": round(float(z), 2), "area": round(g.area, 1),
                           "xy": [round(c, 1) for c in g.centroid.coords[0]]}
                    (supported if has_material_below(g, float(z))
                     else floating).append(rec)
            # overhang: material stepping out beyond the previous layer + step
            oh = cur.difference(prev.buffer(OVERHANG_STEP))
            if not oh.is_empty and oh.area > ISLAND_MIN_AREA:
                overhang_area += oh.area
                blobs = list(oh.geoms) if isinstance(oh, MultiPolygon) else [oh]
                for b in blobs:
                    if b.area > 2.0:
                        w = _width_of_blob(b)
                        if w > widest_step:
                            widest_step, widest_z = w, float(z)
                        bl = b.boundary.length
                        adj = b.boundary.intersection(prev.buffer(0.35)).length / bl if bl else 1.0
                        if adj < CANTILEVER_ADJ and w > widest_cant:
                            widest_cant, widest_cant_z = w, float(z)
        if k % THIN_EVERY == 0:
            # hard: the LARGEST single thin blob (a real wall segment); the
            # summed area is dominated by 0.2 mm² feather-edges where bores
            # break through curved faces at grazing angles — harmless
            thin_h = cur.difference(cur.buffer(-THIN_HARD / 2).buffer(THIN_HARD / 2))
            blobs = [b for b in ((list(thin_h.geoms) if isinstance(thin_h, MultiPolygon)
                                  else ([thin_h] if not thin_h.is_empty else [])))
                     if b.area > 0.5]
            # a WALL persists in height; a 45° feather (a plane grazing a
            # cone/cylinder) is gone one sample (THIN_EVERY layers) later.
            # Count a blob only if the previous sample had one overlapping it.
            for b in blobs:
                if any(b.intersects(pb.buffer(0.6)) for pb in prev_thin):
                    if b.area > thin_hard_max:
                        thin_hard_max, thin_hard_z = b.area, float(z)
            prev_thin = blobs
            opened = cur.buffer(-THIN_SOFT / 2).buffer(THIN_SOFT / 2)
            thin_soft_max = max(thin_soft_max, cur.area - opened.area)
        prev = cur
    res.update({
        "layers": int(len(zs)),
        "bed_support_islands": floating,
        "part_support_islands": len(supported),
        "supported_island_area": round(sum(s["area"] for s in supported), 1),
        "first_supported_island": supported[0] if supported else None,
        "overhang_area": round(overhang_area, 1),
        "widest_overhang_step": round(widest_step, 1),
        "widest_overhang_z": None if widest_z is None else round(widest_z, 1),
        "widest_cantilever": round(widest_cant, 1),
        "widest_cantilever_z": None if widest_cant_z is None else round(widest_cant_z, 1),
        "thin_hard_area": round(thin_hard_max, 2),
        "thin_hard_z": None if thin_hard_z is None else round(thin_hard_z, 2),
        "thin_soft_area": round(thin_soft_max, 2),
    })
    hard = []
    policy = SUPPORT_POLICY.get(name)
    islands = floating + supported
    if islands and policy == "none":
        hard.append(f"{len(islands)} island(s) but the plan says NO support — "
                    f"first at z={islands[0]['z']} area {islands[0]['area']} mm² "
                    f"xy {islands[0]['xy']}")
    big_float = [f for f in floating if f["area"] > 0.25 * max_layer_area and f["area"] > 500]
    if big_float:
        hard.append(f"the part FLOATS above the bed: a {big_float[0]['area']:.0f} mm² island "
                    f"at z={big_float[0]['z']} with nothing under it (bad orientation or "
                    f"geometry below the floor plane)")
    if thin_hard_max > THIN_MIN_AREA and name not in THIN_OK:
        hard.append(f"wall segment < {THIN_HARD} mm: {thin_hard_max:.1f} mm² at z={thin_hard_z:.1f}")
    if not res["bed_ok"]:
        hard.append(f"does not fit the bed: {res['size']}")
    if policy == "none" and widest_cant >= CANTILEVER_HARD:
        hard.append(f"{widest_cant:.1f} mm cantilevered overhang at z={widest_cant_z:.1f} "
                    f"but the plan says NO support")
    res["hard"] = hard
    warn = []
    if islands and policy != "none":
        tag = "expected" if policy == "yes" else "SURPRISE — plan is silent"
        kinds = []
        if floating:
            kinds.append(f"{len(floating)} from the BED (first z={floating[0]['z']}, "
                         f"{floating[0]['area']} mm²)")
        if supported:
            kinds.append(f"{len(supported)} from the part")
        warn.append("islands need support: " + ", ".join(kinds) + f" ({tag})")
    if name in THIN_OK and thin_hard_max > THIN_MIN_AREA:
        warn.append(f"deliberate flexure walls < {THIN_HARD} mm ({thin_hard_max:.1f} mm²)")
    if widest_step >= 3.0:
        kind = "cantilever" if widest_cant >= 3.0 else "bridge/step"
        warn.append(f"overhang step {widest_step:.1f} mm wide at z={widest_z:.1f} ({kind})")
    if thin_soft_max > 4.0:
        warn.append(f"walls < {THIN_SOFT} mm: {thin_soft_max:.1f} mm² in a layer")
    res["warn"] = warn
    if verbose:
        print(json.dumps(res, indent=1))
    return res


def main():
    names = [a for a in sys.argv[1:] if not a.startswith("--")] or list(ORIENT)
    verbose = len(names) == 1
    results, fails = [], 0
    for n in names:
        if not os.path.exists(os.path.join(OUT, f"{n}.stl")):
            print(f"skip  {n:24s} (no STL)")
            continue
        r = audit(n, verbose=verbose)
        results.append(r)
        fails += bool(r["hard"])
        status = "FAIL" if r["hard"] else ("warn" if r["warn"] else "ok  ")
        msg = "; ".join(r["hard"] + r["warn"]) or "clean"
        print(f"{status}  {n:24s} {r['layers']:4d} layers  {msg}")
    with open(os.path.join(OUT, "printability.json"), "w") as f:
        json.dump(results, f, indent=1)
    print(f"\nprintability: {len(results)} parts, {fails} hard failure(s)"
          f"{'' if fails else ' — PLATE CLEAN'}")
    sys.exit(fails)


if __name__ == "__main__":
    main()
