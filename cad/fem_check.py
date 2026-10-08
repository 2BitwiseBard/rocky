#!/usr/bin/env python3
"""Will the printed leg break? Linear-static FEA of the load-bearing leg
parts under the largest loads the servos can put through them (D061).

    python3 fem_check.py                 # every part, every load case
    python3 fem_check.py femur coupler   # a subset
    python3 fem_check.py --fine          # half the element size (a convergence look)

Needs gmsh + CalculiX (fem_tools.tool_cmd: ROCKY_GMSH / ROCKY_CCX, PATH, or
the FreeCAD Flatpak's copies). Without them it says SKIPPED and exits 0, so
CI and machines without FreeCAD are unaffected.

LOADS: the servo is the fuse. A foot can only be pushed as hard as the
weakest joint in the chain can hold at stall (actuators.st3215.stall_nm_12v),
so each case is the ground force at the nominal stance foot that brings the
first servo to stall:
  V    vertical (up): limited by the longer of the hip / knee levers
  R+/- radial (out / in): limited by the longer of the hip / knee heights
  L+/- lateral (+Y / -Y): limited by the yaw servo (reach from the yaw axis)
The horn coupler gets stall torque about its own axis, both ways. Impacts
beyond stall back-drive a servo; the safety factor target covers that.

MODEL: each part alone, linear elastic, one solid of printed material (no
infill, no layer lines: those enter through the allowables). "Held" nodes
(the mating servo / horn / deck) are pinned; "loaded" nodes are tied by a
rigid body to a reference point that takes the force + moment. A pin holds
both ways, so a case that lifts a part off one of its seats names its own
grips (held_cases: the coxa fork's R-, whose idler meets the open mouth).
The twin-plate femur is analysed as plate A + plate B bolted into one piece.

VERDICT per part = the worst case of
  von Mises                    vs fem.<material>.strength_xy_mpa
  tension across the layers    vs fem.<material>.strength_z_mpa
(the build direction comes from check_printability.ORIENT, the print plan's
poses). Stresses are read on the surface outside the grips (+ a margin: a
rigid grip makes its own spikes); the figure is the 99.9th percentile of
those nodes, and the raw peak is printed beside it with its location so a
real hot spot cannot hide in the last 0.1 %. SF >= fem.sf_target passes,
>= fem.sf_warn warns, less fails. Exit code = number of failing parts.

Outputs: cad/out/fem/FEM_REPORT.md, fem_results.json, one <part>.png (the
governing case); cad/out/fem/work/ (git-ignored) keeps the meshes, decks and
.frd results — `rocky.sh cad-open --fem <part>` opens one in FreeCAD.
"""
import json
import math
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

import threading

import fem_tools as ft

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "fem")
WORK = os.path.join(OUT, "work")
_OCC = threading.Lock()                  # build123d / OCCT is not thread-safe: one solid at a time


# ---------------------------------------------------------------- loads
def stance(P):
    """Nominal stance in the leg frame (x out, z up, deck plane z=0, yaw
    axis at the origin): hip, knee, foot (x, z) and the femur / tibia angles
    — the same knee-down IK as gait/rocky_model._stance_tibia_uz."""
    leg, g = P["leg"], P["gait"]
    L1, L2, L3 = leg["l1_coxa"], leg["l2_femur"], leg["l3_tibia"]
    zh = float(leg["hip_axis_z"])
    xf = g["stance_radius"] - P["body"]["circumradius"]
    zf = -g["body_height"]
    dx, dz = xf - L1, zf - zh
    c3 = (dx * dx + dz * dz - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    q3 = -math.acos(c3)
    q2 = math.atan2(dz, dx) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    hip = np.array([L1, zh])
    knee = hip + L2 * np.array([math.cos(q2), math.sin(q2)])
    foot = np.array([xf, zf])
    return {"hip": hip, "knee": knee, "foot": foot, "q_femur": q2, "q_tibia": q2 + q3}


def load_cases(P):
    """Ground force on the foot per case, leg frame (N), + the joint that
    limits it."""
    s = stance(P)
    stall = P["actuators"]["st3215"]["stall_nm_12v"] * 1000.0          # N.mm
    xf, zf = s["foot"]
    lev_v = {"hip": abs(xf - s["hip"][0]), "knee": abs(xf - s["knee"][0])}
    lev_r = {"hip": abs(zf - s["hip"][1]), "knee": abs(zf - s["knee"][1])}
    jv, jr = max(lev_v, key=lev_v.get), max(lev_r, key=lev_r.get)
    fv, fr, fl = stall / lev_v[jv], stall / lev_r[jr], stall / abs(xf)
    return {
        "V": (np.array([0, 0, fv]), jv),
        "R+": (np.array([fr, 0, 0]), jr), "R-": (np.array([-fr, 0, 0]), jr),
        "L+": (np.array([0, fl, 0]), "yaw"), "L-": (np.array([0, -fl, 0]), "yaw"),
    }


def _rot_xz(a):
    """Rotation by angle a in the leg's x-z plane (x toward z), as a 3x3."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


# ---------------------------------------------------------------- zones
def cyl(p, c, axis, r, a0, a1):
    """Nodes inside a cylinder: centre c (3,), axis 'x'|'y'|'z', radius r,
    axial span a0..a1 (absolute coordinate along the axis)."""
    k = "xyz".index(axis)
    o = [i for i in range(3) if i != k]
    rad = np.hypot(p[:, o[0]] - c[o[0]], p[:, o[1]] - c[o[1]])
    return (rad <= r) & (p[:, k] >= a0) & (p[:, k] <= a1)


def servo_case(p, to_local, S, Z, g):
    """Nodes within g of the servo CASE (box + rims + plateau), servo-local
    frame via to_local — the skin of a cup that holds the servo."""
    q = to_local(p)
    x0, x1 = S["case_x"]
    w = S["body_w"] / 2
    boxes = [(x0, x1, -w, w, 0.0, Z["top"]),
             (S["rim"]["x0"], S["rim"]["x1"], -w, w, Z["top"], Z["rim_top"]),
             (S["rim"]["x0"], S["rim"]["x1"], -w, w, Z["rim_bot"], 0.0),
             (S["plateau"]["x0"], S["plateau"]["x1"], -S["plateau"]["half_w"], S["plateau"]["half_w"],
              Z["top"], Z["plateau"])]
    m = np.zeros(len(p), bool)
    for b in boxes:
        m |= ((q[:, 0] >= b[0] - g) & (q[:, 0] <= b[1] + g) & (q[:, 1] >= b[2] - g)
              & (q[:, 1] <= b[3] + g) & (q[:, 2] >= b[4] - g) & (q[:, 2] <= b[5] + g))
    return m


# ---------------------------------------------------------------- parts
def part_specs(P):
    """name -> dict(solid(), build dir source, held(p), loaded(p), ref,
    loads {case: (F, M)} in the part frame). Imports build123d lazily."""
    from servo_st3215 import spec, z_levels, case_xspan
    import leg_frame as lf
    import part_coxa as pc
    import part_femur as pf
    import part_tibia as pt
    import part_coupler as pcu

    S, Z = dict(spec(P)), z_levels(P)
    S["case_x"] = case_xspan(P)
    FIT = lf.FIT
    G = FIT + 0.35                       # grip skin: the fit gap + ~one element
    st, lcs = stance(P), load_cases(P)
    foot_leg = np.array([st["foot"][0], 0.0, st["foot"][1]])
    stall = P["actuators"]["st3215"]["stall_nm_12v"] * 1000.0

    def from_foot(foot_part, R, ref):
        """{case: (F, M about ref)} for a part whose frame is the leg frame
        rotated by R (part = R @ leg), foot at foot_part."""
        out = {}
        for k, (F, _) in lcs.items():
            Fp = R @ F
            out[k] = (Fp, np.cross(foot_part - ref, Fp))
        return out

    specs = {}

    # coxa_yaw_base: docked on the I1 seats (2026-10-07, decision 15, B118). Until the D064
    # integration this case pinned the WHOLE underside, the 0.2-relieved face outboard of
    # seat_relief_x0 included, which the deck never touches. What really holds the plate:
    #   pads   the unrelieved underside inboard of seat_relief_x0 (-46) where there is deck
    #          under it (|y| >= the hook slot's 12.3): it bears DOWN (compression);
    #   seats  the two sockets' flanks on the deck's 30 deg cone posts (the cone at seat_xy[0],
    #          the vee at seat_xy[1] only along its x-locating strip): down + sideways;
    #   screws the two thumbscrew heads (Ø thumbscrew_head_d) on the plate top round their
    #          bores: they hold the plate DOWN (tension), nothing else.
    # A pin holds both ways, so each case is held only by what bears in it (rigid-body
    # statics about the plate, M' about the support line = M + dz F_x): V and R+ lift the
    # outboard end (M' -6994 / -1872 N mm about the screws / the underside): the plate rocks
    # on the pads against the screws, the seats LIFT in V (R+ keeps them: its push is
    # horizontal); R- presses the outboard end (M' +1901 about the seats): the pads lift, the
    # seats and the screws hold; L+ / L- roll it (M_x +-5645): the pads on the pressing side,
    # both seats, the screw on the lifting side.
    yaw_local = lambda p: np.c_[p[:, 0], -p[:, 1], lf.Z_YAW_TOP - p[:, 2]]
    ref = np.array([0.0, 0.0, lf.ZC_YAW])
    from iface import HOOK_HALF_W
    lp = P["interfaces"]["leg_port"]
    ZB, ZT = -4.0, 0.0                    # the plate's underside (= the deck top) and its top
    ta = math.tan(math.radians(lp["seat_half_angle"]))
    r_seat = lambda z: lp["seat_r"] + P["print"]["seat_fit"] - (z - ZB) * ta   # the socket's flank
    z_flank = (ZB + lp["seat_mouth"] + lp["seat_relief"], ZB + lp["seat_h"])     # mouth relief .. post tip
    GS = 0.5                              # the flank's skin: one clmin
    (cx, cy), (vx, vy) = lp["seat_xy"]
    slack = lp["seat_vee_slack"]

    def side(p, sy):
        return np.ones(len(p), bool) if sy == 0 else (np.sign(p[:, 1]) == sy)

    def pads(p, sy=0):
        return ((np.abs(p[:, 2] - ZB) <= 0.1) & (p[:, 0] <= lp["seat_relief_x0"] + 1e-6)
                & (np.abs(p[:, 1]) >= HOOK_HALF_W) & side(p, sy))

    def seats(p):
        inz = (p[:, 2] >= z_flank[0]) & (p[:, 2] <= z_flank[1])
        cone = np.abs(np.hypot(p[:, 0] - cx, p[:, 1] - cy) - r_seat(p[:, 2])) <= GS
        vee = (np.abs(np.abs(p[:, 0] - vx) - r_seat(p[:, 2])) <= GS) & (np.abs(p[:, 1] - vy) <= slack + GS)
        return inz & (cone | vee)

    def screws(p, sy=0):
        m = np.zeros(len(p), bool)
        for tx, ty in lp["thumbscrew_xy"]:
            r = np.hypot(p[:, 0] - tx, p[:, 1] - ty)
            m |= (p[:, 2] >= ZT - 0.1) & (r >= 3.7 / 2) & (r <= lp["thumbscrew_head_d"] / 2) & (np.sign(ty) == sy if sy else True)
        return m
    specs["coxa_yaw_base"] = dict(
        solid=pc.coxa_yaw_base,
        held=lambda p: pads(p) | seats(p) | screws(p),
        held_cases={"V": lambda p: pads(p) | screws(p),
                    "R-": lambda p: seats(p) | screws(p),
                    "L+": lambda p: pads(p, -1) | seats(p) | screws(p, 1),
                    "L-": lambda p: pads(p, 1) | seats(p) | screws(p, -1)},
        loaded=lambda p: servo_case(p, yaw_local, S, Z, G) & (p[:, 2] > ZB + 0.35),
        ref=ref, loads=from_foot(foot_leg, np.eye(3), ref),
        held_what="the docked I1 support (B118): the inboard pads (x <= seat_relief_x0, over the deck), "
                  "the two seat flanks, the two thumbscrew heads in tension; V: pads + screws (the seats "
                  "lift), R-: seats + screws (the pads lift), L+/L-: the pressing side's pads, both seats, "
                  "the lifting side's screw",
        loaded_what="yaw servo case in the cup")

    # coxa_fork: the yaw servo holds it where it bears (D063): the Ø20 horn face clamped on the
    # FLAT hub top (B81: there is no horn pocket) + the idler in its pocket, whose mouth side
    # is open (B80: the kept +X wall, the side walls, the roof). The hip servo pushes its cup.
    # R- (foot pushed in) needs the idler to push the upper plate -X, from the open side:
    # nothing bears there, so that case is held by the horn face alone (with the idler held
    # too it read SF 3.60; horn-only, before the side cheeks, 0.49).
    hip_local = lambda p: np.c_[p[:, 0] - lf.L1, p[:, 2] - lf.Z_HIP, lf.YB - p[:, 1]]
    horn_r = S["horn_d"] / 2 + 0.35
    idl_r = pc.IDLER_POCKET_D / 2 + 0.35
    ref = np.array([lf.L1, 0.0, lf.Z_HIP])
    horn_face = lambda p: cyl(p, (0, 0, 0), "z", horn_r, lf.HUB_Z1 - 0.35, lf.HUB_Z1 + 0.35)
    specs["coxa_fork"] = dict(
        solid=pc.coxa_fork,
        held=lambda p: horn_face(p) | cyl(p, (0, 0, 0), "z", idl_r, pc.UP_Z0 - 0.35, pc.IDLER_POCKET_Z1 + 0.35),
        held_cases={"R-": horn_face},
        loaded=lambda p: servo_case(p, hip_local, S, Z, G),
        ref=ref, loads=from_foot(foot_leg, np.eye(3), ref),
        held_what="yaw horn face on the flat hub top + yaw idler pocket (open to the mouth); "
                  "R-: the horn face alone",
        loaded_what="hip servo case in the cup")

    # femur: plate A + plate B bolted; the hip holds hub A, the knee pushes hub B
    Rf = _rot_xz(-st["q_femur"])                        # leg -> link (link x along the femur)
    foot_link = Rf @ (foot_leg - np.array([lf.L1, 0.0, lf.Z_HIP]))
    rec_r = pcu.DISC_D / 2 + 0.35
    pk_r = (S["idler_d"] + FIT + 0.1) / 2 + 0.35

    def hub(p, cx):
        c = (cx, 0, 0)
        return (cyl(p, c, "y", rec_r, pf.YA1 - pcu.POCKET_DEPTH - 0.35, pf.YA1 + 0.35)
                | cyl(p, c, "y", pk_r, pf.YB0 - 0.35, pf.POCKET_Y1 + 0.35))
    ref = np.array([lf.L2, (pf.YA1 + pf.POCKET_Y1) / 2, 0.0])
    def femur_bolted():
        """plate A + plate B joined ONLY through the four boss faces (the M3
        joints): the bridge walls touch plate B's rails but cannot pull on
        them, so a 0.4 mm slot is cut between them outside the bosses (a
        welded wall top would carry tension a real print cannot)."""
        from build123d import Pos, Box, Cylinder, Rot
        link = pf.femur_link()
        for sz in (1, -1):
            z0, z1 = sorted((sz * pf.WALL_Z[0], sz * pf.WALL_Z[1]))
            gap = Pos((pf.BRIDGE_X[0] + pf.BRIDGE_X[1]) / 2, pf.RAIL_Y0 - 0.2, (z0 + z1) / 2) * \
                Box(pf.BRIDGE_X[1] - pf.BRIDGE_X[0] + 2, 0.4, z1 - z0 + 2)
            for bx in pf.BOSS_X:
                gap -= Pos(bx, pf.RAIL_Y0 - 0.2, sz * pf.BOSS_ZC) * Rot(90, 0, 0) * Cylinder(pf.BOSS_D / 2, 2)
            link -= gap
        return link + pf.femur_plate_b()
    specs["femur"] = dict(
        solid=femur_bolted,
        orient_as="femur_link",
        held=lambda p: hub(p, 0.0), loaded=lambda p: hub(p, lf.L2),
        ref=ref, loads=from_foot(foot_link, Rf, ref),
        held_what="hip coupler recess (plate A) + hip idler pocket (plate B); plates joined at the 4 bosses only",
        loaded_what="knee coupler recess + knee idler pocket")

    # tibia_knee_carrier: the knee servo holds its cup, the tube pushes the socket
    knee_local = lambda p: np.c_[p[:, 0] - lf.KNEE_X, p[:, 2] - lf.Z_HIP, lf.YB - p[:, 1]]
    Rt = _rot_xz(-(st["q_tibia"] + math.pi / 2))        # leg -> carrier (tube straight down)
    knee_leg = np.array([st["knee"][0], 0.0, st["knee"][1]])
    foot_car = np.array([lf.KNEE_X, 0.0, lf.Z_HIP]) + Rt @ (foot_leg - knee_leg)
    sock_r = (pt.TUBE_OD + FIT) / 2 + 0.35
    ref = np.array([lf.KNEE_X, 0.0, pt.BOSS_Z0 + pt.SOCKET_DEPTH / 2])
    specs["tibia_knee_carrier"] = dict(
        solid=pt.tibia_knee_carrier,
        held=lambda p: servo_case(p, knee_local, S, Z, G),
        loaded=lambda p: cyl(p, (lf.KNEE_X, 0, 0), "z", sock_r, pt.BOSS_Z0 - 0.35,
                             pt.BOSS_Z0 + pt.SOCKET_DEPTH + 0.35),
        ref=ref, loads=from_foot(foot_car, Rt, ref),
        held_what="knee servo case in the cup", loaded_what="tibia tube in the clamp socket")

    # horn_coupler: the horn screws + disc face hold it, the recess drives the lobes (stall torque)
    from servo_st3215 import horn_screw_angles
    r0, L, w = S["horn_bcd"] / 2, S["horn_bcd_slot"], S["horn_screw_clear_d"] / 2 + 0.35
    angs = np.deg2rad(horn_screw_angles(P))

    def slots(p):
        m = p[:, 2] <= 0.35                              # disc face on the horn
        for a in angs:
            u = p[:, 0] * math.cos(a) + p[:, 1] * math.sin(a)
            v = -p[:, 0] * math.sin(a) + p[:, 1] * math.cos(a)
            m |= (np.abs(v) <= w) & (u >= r0 - w) & (u <= r0 + L + w)
        return m
    ref = np.array([0.0, 0.0, pcu.DISC_T + pcu.LOBE_H])
    specs["horn_coupler"] = dict(
        solid=pcu.horn_coupler, clmax=0.6, clmin=0.2,   # 2.6 mm lobes: several elements through
        held=slots,
        loaded=lambda p: p[:, 2] >= pcu.DISC_T + pcu.LOBE_H / 2,
        ref=ref, loads={"T+": (np.zeros(3), np.array([0, 0, stall])),
                        "T-": (np.zeros(3), np.array([0, 0, -stall]))},
        held_what="horn screw slots + disc face", loaded_what="upper half of the shear lobes")
    return specs


def build_dir(name):
    """Unit build (layer-stacking) direction in the part frame, from the
    print plan's pose (check_printability.ORIENT; None = as modelled)."""
    from check_printability import ORIENT
    R = ORIENT.get(name)
    if R is None:
        return np.array([0.0, 0.0, 1.0])
    return np.asarray(R)[2, :3] / np.linalg.norm(np.asarray(R)[2, :3])


# ---------------------------------------------------------------- one part
def analyse(name, sp, P, fine=False, tools=None):
    from build123d import export_step
    gmsh, ccx = tools
    FP = P["fem"]
    mat = FP[FP["material"]]
    mesh = FP["mesh"]
    clmax, clmin = sp.get("clmax", mesh["clmax"]), sp.get("clmin", mesh["clmin"])
    if fine:
        clmax, clmin = clmax / 2, clmin / 2
    t0 = time.time()
    base = os.path.join(WORK, name + ("_fine" if fine else ""))
    with _OCC:
        solid = sp["solid"]()
        if len(solid.solids()) != 1:
            raise RuntimeError(f"{name}: {len(solid.solids())} solids (expected one)")
        export_step(solid, base + ".step")
    ft.mesh_step(gmsh, base + ".step", base + ".msh", clmax, clmin, base + ".gmsh.log")
    nodes, tets = ft.read_msh2(base + ".msh")
    used = np.unique(tets)
    remap = np.full(len(nodes), -1)
    remap[used] = np.arange(len(used))
    nodes, tets = nodes[used], remap[tets]
    tets = ft.to_c3d10(nodes, tets)
    tris = ft.boundary_faces(tets)
    surf = np.zeros(len(nodes), bool)
    surf[tris] = True
    n_build = build_dir(sp.get("orient_as", name))
    sxy, sz = mat["strength_xy_mpa"], mat["strength_z_mpa"]
    from scipy.spatial import cKDTree
    # one deck per set of grips: a case whose load lifts the part off one of its seats is
    # held only where it still bears (held_cases; the fork's R-, D063)
    decks = {}
    for k, FM in sp["loads"].items():
        decks.setdefault(sp.get("held_cases", {}).get(k, sp["held"]), []).append((k, FM))
    held = sp["held"](nodes)
    counts = int(held.sum()), int((sp["loaded"](nodes) & ~held).sum())    # the default grips, reported
    rows, worst = {}, None
    for i, (held_fn, cases) in enumerate(decks.items()):
        held, loaded = held_fn(nodes), sp["loaded"](nodes)
        loaded &= ~held
        if held.sum() < 10 or loaded.sum() < 10:
            raise RuntimeError(f"{name}: grip zones missed the mesh (held {held.sum()}, loaded {loaded.sum()})")
        job = base + (f"_{i}" if i else "")
        ft.write_ccx(job + ".inp", nodes, tets, held, loaded, sp["ref"],
                     [(k, F, M) for k, (F, M) in cases], mat["E_mpa"], mat["nu"])
        ft.run_ccx(ccx, job + ".inp", job + ".ccx.log")
        res = ft.read_frd(job + ".frd", len(nodes))
        if len(res) != len(cases):
            raise RuntimeError(f"{name}: {len(res)} result steps for {len(cases)} cases")

        # judge on the surface, away from the grips
        grip = held | loaded
        d, _ = cKDTree(nodes[grip]).query(nodes, k=1)
        judge = surf & (d > FP["grip_margin_mm"])
        for (k, (F, M)), r in zip(cases, res):
            vm = ft.von_mises(r["S"])
            sn = np.maximum(ft.normal_stress(r["S"], n_build), 0.0)
            util = np.maximum(vm / sxy, sn / sz)
            j = np.flatnonzero(judge)
            p999 = lambda a: float(np.percentile(a[j], 99.9))
            ipk = j[np.argmax(util[j])]
            row = dict(case=k, F_N=[round(float(x), 2) for x in F], M_Nmm=[round(float(x), 1) for x in M],
                       vm_p999=round(p999(vm), 2), sn_p999=round(p999(sn), 2),
                       util_p999=round(p999(util), 3), util_peak=round(float(util[ipk]), 3),
                       peak_at=[round(float(x), 1) for x in nodes[ipk]],
                       defl_mm=round(float(np.linalg.norm(r["U"][loaded], axis=1).max()), 3))
            row["sf"] = round(1.0 / row["util_p999"], 2) if row["util_p999"] > 0 else float("inf")
            row["governs"] = "layers" if p999(sn / sz) > p999(vm / sxy) else "von Mises"
            rows[k] = row
            if worst is None or row["util_p999"] > worst[0]["util_p999"]:
                worst = (row, util, nodes[ipk], grip)
    rows = [rows[k] for k in sp["loads"]]
    sf = worst[0]["sf"]
    verdict = "PASS" if sf >= FP["sf_target"] else ("WARN" if sf >= FP["sf_warn"] else "FAIL")
    if not fine:
        ft.render(os.path.join(OUT, f"{name}.png"), nodes, tris, worst[1], worst[3],
                  f"{name}: case {worst[0]['case']}, SF {sf:.2f} ({verdict}) — "
                  f"{mat['name']}, {len(tets)} C3D10\npeak {worst[0]['util_peak']:.2f} at "
                  f"{tuple(worst[0]['peak_at'])} mm (part frame, ringed)", peak=worst[2])
    return dict(part=name, verdict=verdict, sf=sf, governing=worst[0]["case"],
                nodes=len(nodes), elements=len(tets), held_nodes=counts[0],
                loaded_nodes=counts[1], build_dir=[round(float(x), 3) for x in n_build],
                held=sp["held_what"], loaded=sp["loaded_what"], cases=rows,
                _seconds=round(time.time() - t0, 1))


# ---------------------------------------------------------------- report
def write_report(P, results):
    FP = P["fem"]
    mat = FP[FP["material"]]
    st, lcs = stance(P), load_cases(P)
    L = ["# FEM report (D061)", "",
         "Generated by `cad/fem_check.py` (`rocky.sh cad-check --fem`). Do not edit by hand.", "",
         f"Material **{mat['name']}**: E {mat['E_mpa']} MPa, von Mises allowable "
         f"{mat['strength_xy_mpa']} MPa, across-layer tension allowable {mat['strength_z_mpa']} MPa "
         f"(VERIFY: coupon pulls). Pass at SF >= {FP['sf_target']}, warn at >= {FP['sf_warn']}.", "",
         f"Stance (leg frame, mm): hip ({st['hip'][0]:.1f}, {st['hip'][1]:.1f}), knee "
         f"({st['knee'][0]:.1f}, {st['knee'][1]:.1f}), foot ({st['foot'][0]:.1f}, {st['foot'][1]:.1f}). "
         f"Servo stall {P['actuators']['st3215']['stall_nm_12v']} N.m.", "",
         "| case | foot force (N, leg frame) | limited by |", "|---|---|---|"]
    for k, (F, j) in lcs.items():
        L.append(f"| {k} | ({F[0]:.1f}, {F[1]:.1f}, {F[2]:.1f}) | {j} servo at stall |")
    L += ["| T+/T- | coupler only: stall torque about the horn axis | - |", "",
          "| part | verdict | SF | case | governs | peak util (raw) | deflection (mm) | mesh |",
          "|---|---|---|---|---|---|---|---|"]
    for r in results:
        if "error" in r:
            L.append(f"| {r['part']} | ERROR | - | - | - | - | - | {r['error']} |")
            continue
        w = next(c for c in r["cases"] if c["case"] == r["governing"])
        L.append(f"| {r['part']} | **{r['verdict']}** | {r['sf']:.2f} | {w['case']} | {w['governs']} | "
                 f"{w['util_peak']:.2f} at {tuple(w['peak_at'])} | {w['defl_mm']:.3f} | "
                 f"{r['elements']} el |")
    L += ["", "SF is the 99.9th-percentile utilisation over surface nodes more than "
          f"{FP['grip_margin_mm']} mm from a grip. When the raw peak is well above 1/SF, look "
          "at that spot in the picture: it is either a sharp modelled corner (a real print "
          "rounds it) or a real hot spot.", ""]
    for r in results:
        if "error" in r:
            continue
        L += [f"## {r['part']}", "", f"![{r['part']}]({r['part']}.png)", "",
              f"Held: {r['held']} ({r['held_nodes']} nodes). Loaded: {r['loaded']} "
              f"({r['loaded_nodes']} nodes). Build direction (part frame): {tuple(r['build_dir'])}.", "",
              "| case | F (N) | M (N.mm) | von Mises p99.9 | layer tension p99.9 | SF | deflection |",
              "|---|---|---|---|---|---|---|"]
        for c in r["cases"]:          # fixed decimals (B115): a bare float printed 2.10 as "2.1"
            L.append(f"| {c['case']} | {tuple(c['F_N'])} | {tuple(c['M_Nmm'])} | {c['vm_p999']:.2f} | "
                     f"{c['sn_p999']:.2f} | {c['sf']:.2f} | {c['defl_mm']:.3f} |")
        L.append("")
    with open(os.path.join(OUT, "FEM_REPORT.md"), "w") as f:
        f.write("\n".join(L))
    with open(os.path.join(OUT, "fem_results.json"), "w") as f:
        json.dump({"material": FP["material"],
                   "results": [{k: v for k, v in r.items() if not k.startswith("_")} for r in results]},
                  f, indent=1)


def main():
    from common import params
    args = sys.argv[1:]
    fine = "--fine" in args
    want = [a for a in args if not a.startswith("--")]
    gmsh, ccx = ft.tool_cmd("gmsh", "ROCKY_GMSH"), ft.tool_cmd("ccx", "ROCKY_CCX")
    if not gmsh or not ccx:
        print("fem_check: SKIPPED (no gmsh / ccx: install FreeCAD, or set ROCKY_GMSH / ROCKY_CCX)")
        return 0
    os.makedirs(WORK, exist_ok=True)
    P = params()
    specs = part_specs(P)
    names = [n for n in specs if not want or any(w in n for w in want)]
    t0 = time.time()

    def one(n):
        try:
            return analyse(n, specs[n], P, fine=fine, tools=(gmsh, ccx))
        except Exception as e:                     # report, don't hide the other parts
            return dict(part=n, error=f"{type(e).__name__}: {e}")
    with ThreadPoolExecutor(max_workers=min(len(names), 5)) as ex:
        results = list(ex.map(one, names))
    for r in results:
        if "error" in r:
            print(f"ERROR {r['part']:20s} {r['error']}")
        else:
            print(f"{r['verdict']:5s} {r['part']:20s} SF {r['sf']:5.2f} (case {r['governing']}, "
                  f"{r['elements']} el, {r['_seconds']:.0f} s)")
    if not fine and not want:
        write_report(P, results)
    bad = [r for r in results if "error" in r or r["verdict"] == "FAIL"]
    print(f"fem_check: {len(results) - len(bad)}/{len(results)} parts hold ({time.time() - t0:.0f} s)"
          + (f" — FAILING: {', '.join(r['part'] for r in bad)}" if bad else ""))
    return len(bad)


if __name__ == "__main__":
    sys.exit(main())
