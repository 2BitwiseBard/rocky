"""fem_tools / fem_check plumbing (D061) — pure numpy, no gmsh / CalculiX
needed; the one solver test skips without them.

    .venv/bin/python -m pytest cad/test_fem.py -q
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import fem_tools as ft  # noqa: E402


def _tet10(order):
    """One unit tet: 4 corners + 6 edge midpoints; midsides in the given
    vertex-pair order."""
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], float)
    mids = [(v[a] + v[b]) / 2 for a, b in order]
    return np.vstack([v, mids]), np.arange(10)[None, :]


def test_c3d10_order_found_from_geometry():
    gmsh_like = [(0, 1), (1, 2), (0, 2), (0, 3), (2, 3), (1, 3)]      # 8 and 9 swapped vs ccx
    nodes, tets = _tet10(gmsh_like)
    out = ft.to_c3d10(nodes, tets)[0]
    for k, (a, b) in enumerate(ft.C3D10_EDGES):
        assert np.allclose(nodes[out[4 + k]], (nodes[out[a]] + nodes[out[b]]) / 2)


def test_inverted_tet_is_flipped():
    nodes, tets = _tet10(ft.C3D10_EDGES)
    bad = tets.copy()
    bad[0, [1, 2]] = bad[0, [2, 1]]
    out = ft.to_c3d10(nodes, bad)[0]
    v = nodes[out[:4]]
    assert np.dot(np.cross(v[1] - v[0], v[2] - v[0]), v[3] - v[0]) > 0


def test_broken_midside_node_raises():
    nodes, tets = _tet10(ft.C3D10_EDGES)
    nodes[5] += 0.4
    with pytest.raises(RuntimeError):
        ft.to_c3d10(nodes, tets)


def test_boundary_faces_of_two_tets():
    tets = np.array([[0, 1, 2, 3], [1, 2, 3, 4]])                  # two tets sharing face 1-2-3
    tets = np.hstack([tets, np.zeros((2, 6), int)])
    f = ft.boundary_faces(tets)
    assert len(f) == 6                                   # 8 faces, the shared one twice


def test_stress_measures():
    s = np.array([[10.0, 0, 0, 0, 0, 0]])                # uniaxial 10 MPa along x
    assert ft.von_mises(s)[0] == pytest.approx(10.0)
    assert ft.normal_stress(s, (1, 0, 0))[0] == pytest.approx(10.0)
    assert ft.normal_stress(s, (0, 0, 1))[0] == pytest.approx(0.0)
    c = math.cos(math.radians(45))
    assert ft.normal_stress(s, (c, 0, c))[0] == pytest.approx(5.0)
    shear = np.array([[0, 0, 0, 4.0, 0, 0]])
    assert ft.von_mises(shear)[0] == pytest.approx(4.0 * math.sqrt(3))


def test_read_frd(tmp_path):
    def rec(nid, vals):
        return " -1" + f"{nid:10d}" + "".join(f"{v:12.5E}" for v in vals) + "\n"
    txt = (" -4  DISP        4    1\n -5  D1          1    2    1    0\n"
           + rec(1, [0.1, 0.2, 0.3]) + rec(2, [0, 0, -1]) + rec(3, [9, 9, 9]) + " -3\n"
           + " -4  STRESS      6    1\n"
           + rec(1, [1, 2, 3, 4, 5, 6]) + rec(2, [0, 0, 0, 0, 0, -6.5]) + " -3\n")
    p = tmp_path / "x.frd"
    p.write_text(txt)
    steps = ft.read_frd(str(p), n_nodes=2)               # node 3 is a rigid-body ref: dropped
    assert len(steps) == 1
    assert np.allclose(steps[0]["U"], [[0.1, 0.2, 0.3], [0, 0, -1]])
    assert np.allclose(steps[0]["S"][1], [0, 0, 0, 0, 0, -6.5])


def test_load_cases_put_exactly_one_servo_at_stall():
    import yaml
    with open(os.path.join(HERE, "params.yaml")) as f:
        P = yaml.safe_load(f)
    import fem_check as fc
    st = fc.stance(P)
    stall = P["actuators"]["st3215"]["stall_nm_12v"] * 1000
    foot = np.array([st["foot"][0], 0, st["foot"][1]])
    joints = {"yaw": (np.zeros(3), np.array([0, 0, 1.0])),
              "hip": (np.array([st["hip"][0], 0, st["hip"][1]]), np.array([0, 1.0, 0])),
              "knee": (np.array([st["knee"][0], 0, st["knee"][1]]), np.array([0, 1.0, 0]))}
    for name, (F, limit) in fc.load_cases(P).items():
        tq = {j: abs(np.dot(np.cross(foot - o, F), a)) for j, (o, a) in joints.items()}
        assert tq[limit] == pytest.approx(stall, rel=1e-9), name
        assert max(tq.values()) == pytest.approx(stall, rel=1e-9), name   # nobody else past stall


def test_stance_matches_the_gait_model():
    import yaml
    with open(os.path.join(HERE, "params.yaml")) as f:
        P = yaml.safe_load(f)
    import fem_check as fc
    st = fc.stance(P)
    L2, L3 = P["leg"]["l2_femur"], P["leg"]["l3_tibia"]
    assert np.linalg.norm(st["knee"] - st["hip"]) == pytest.approx(L2)
    assert np.linalg.norm(st["foot"] - st["knee"]) == pytest.approx(L3)
    sys.path.insert(0, os.path.join(HERE, "..", "gait"))
    import rocky_model as rm
    uz = rm._stance_tibia_uz(P["gait"]["body_height"], P["gait"]["stance_radius"])
    assert math.sin(st["q_tibia"]) == pytest.approx(uz)


def test_report_prints_fixed_decimals(tmp_path, monkeypatch):
    """B115: the per-case rows print SF and stresses to 2 dp and deflection to
    3 dp; a bare float printed 2.10 as "2.1" (so the yaw base's R+ and R-
    von Mises, the same 2.10446 on the same grips, read like different rows)."""
    import yaml
    with open(os.path.join(HERE, "params.yaml")) as f:
        P = yaml.safe_load(f)
    import fem_check as fc
    monkeypatch.setattr(fc, "OUT", str(tmp_path))
    case = dict(case="R+", F_N=[16.42, 0.0, 0.0], M_Nmm=[0.0, -2365.1, 0.0], vm_p999=2.1,
                sn_p999=2.08, util_p999=0.476, util_peak=0.39, peak_at=[-14.0, 5.5, 0.0],
                defl_mm=0.15, sf=2.1, governs="layers")
    res = dict(part="demo", verdict="PASS", sf=2.1, governing="R+", nodes=10, elements=20,
               held_nodes=5, loaded_nodes=5, build_dir=[0.0, 0.0, 1.0], held="h", loaded="l",
               cases=[case])
    fc.write_report(P, [res])
    txt = (tmp_path / "FEM_REPORT.md").read_text()
    assert "| R+ | (16.42, 0.0, 0.0) | (0.0, -2365.1, 0.0) | 2.10 | 2.08 | 2.10 | 0.150 |" in txt
    assert "| demo | **PASS** | 2.10 | R+ | layers | 0.39 at (-14.0, 5.5, 0.0) | 0.150 | 20 el |" in txt


@pytest.mark.skipif(not (ft.tool_cmd("gmsh", "ROCKY_GMSH") and ft.tool_cmd("ccx", "ROCKY_CCX")),
                    reason="no gmsh / CalculiX")
def test_cantilever_matches_beam_theory():
    """100 x 10 x 10 mm PETG bar, wall at x=0, 10 N down at the tip: bending
    stress at x=30 is M c / I = 700 * 5 / 833.3 = 4.20 MPa; tip deflection
    F L^3 / 3EI = 2.667 mm (Euler-Bernoulli; the pinned wall face stiffens it
    a little)."""
    from build123d import Box, Pos, export_step
    work = os.path.join(HERE, "out", "fem", "work")
    os.makedirs(work, exist_ok=True)
    b = os.path.join(work, "test_beam")
    export_step(Pos(50, 0, 0) * Box(100, 10, 10), b + ".step")
    ft.mesh_step(ft.tool_cmd("gmsh", "ROCKY_GMSH"), b + ".step", b + ".msh", 2.0, 0.5, b + ".gmsh.log")
    n, t = ft.read_msh2(b + ".msh")
    used = np.unique(t)
    remap = np.full(len(n), -1)
    remap[used] = np.arange(len(used))
    n, t = n[used], ft.to_c3d10(n[used], remap[t])
    fix, load = n[:, 0] <= 1e-6, n[:, 0] >= 100 - 1e-6
    ft.write_ccx(b + ".inp", n, t, fix, load, (100, 0, 0), [("tip", (0, 0, -10.0), (0, 0, 0))], 1500.0, 0.38)
    ft.run_ccx(ft.tool_cmd("ccx", "ROCKY_CCX"), b + ".inp", b + ".ccx.log")
    r = ft.read_frd(b + ".frd", len(n))[0]
    at30 = np.abs(n[:, 0] - 30) < 1e-6
    assert r["S"][at30, 0].max() == pytest.approx(4.20, rel=0.03)
    assert -r["U"][load, 2].mean() == pytest.approx(2.667, rel=0.02)
    for f in os.listdir(work):
        if f.startswith("test_beam"):
            os.remove(os.path.join(work, f))
