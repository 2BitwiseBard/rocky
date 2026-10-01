"""Finite-element plumbing for fem_check (D061): external tools, gmsh mesh in,
CalculiX deck out, CalculiX results back, stress pictures.

No build123d here and nothing imported at module level beyond numpy, so the
unit tests (cad/test_fem.py) run anywhere.

Tools: gmsh (tet mesher) and ccx (CalculiX solver). Each is found as
  1. the command in ROCKY_GMSH / ROCKY_CCX (shell words, rocky.env), else
  2. `gmsh` / `ccx` on PATH, else
  3. the FreeCAD Flatpak's bundled copy (`flatpak run --command=... org.freecad.FreeCAD`).
The Flatpak sees the host filesystem but has its own /tmp, so every file
passed to a tool lives under cad/out/fem/work/.

Units throughout: mm, N, MPa (so moments are N.mm).
"""
import os
import shlex
import shutil
import subprocess

import numpy as np

FLATPAK_APP = "org.freecad.FreeCAD"


def _flatpak_has_app():
    if not shutil.which("flatpak"):
        return False
    return subprocess.run(["flatpak", "info", FLATPAK_APP], capture_output=True).returncode == 0


def tool_cmd(name, env_var):
    """argv prefix for an external tool, or None when it is nowhere."""
    if os.environ.get(env_var):
        return shlex.split(os.environ[env_var])
    if shutil.which(name):
        return [name]
    if _flatpak_has_app():
        return ["flatpak", "run", f"--command={name}", FLATPAK_APP]
    return None


# ---------------------------------------------------------------- mesh in
# CalculiX C3D10 midside order, as vertex pairs (0-based): 5:(1,2) 6:(2,3)
# 7:(3,1) 8:(1,4) 9:(2,4) 10:(3,4) in the manual's 1-based numbering.
C3D10_EDGES = ((0, 1), (1, 2), (2, 0), (0, 3), (1, 3), (2, 3))


def mesh_step(gmsh, step, msh, clmax, clmin, log):
    """Second-order tet mesh of a STEP file (msh 2.2 ASCII). Midside nodes
    stay on the straight edges (SecondOrderLinear): snapping them to curved
    faces inverts small elements on tight radii (nonpositive jacobian in ccx)."""
    cmd = gmsh + [os.path.abspath(step), "-3", "-order", "2", "-clmax", str(clmax),
                  "-clmin", str(clmin), "-setnumber", "Mesh.SecondOrderLinear", "1",
                  "-format", "msh2", "-o", os.path.abspath(msh)]
    with open(log, "w") as f:
        r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0 or not os.path.exists(msh):
        raise RuntimeError(f"gmsh failed on {step} (log: {log})")


def read_msh2(path):
    """(nodes (N,3), tets (E,10) as 0-based row indices) from a gmsh msh 2.2
    ASCII file; only the 10-node tetrahedra (gmsh type 11) are kept."""
    with open(path) as f:
        lines = f.read().split("\n")
    i = lines.index("$Nodes")
    n = int(lines[i + 1])
    raw = np.array([ln.split() for ln in lines[i + 2:i + 2 + n]], dtype=float)
    ids = raw[:, 0].astype(int)
    nodes = raw[:, 1:4]
    row = np.full(ids.max() + 1, -1, dtype=int)
    row[ids] = np.arange(n)
    j = lines.index("$Elements")
    m = int(lines[j + 1])
    tets = []
    for ln in lines[j + 2:j + 2 + m]:
        p = ln.split()
        if p[1] == "11":
            ntags = int(p[2])
            tets.append([int(t) for t in p[3 + ntags:3 + ntags + 10]])
    if not tets:
        raise RuntimeError(f"{path}: no 10-node tetrahedra (did gmsh mesh a solid?)")
    return nodes, row[np.array(tets)]


def to_c3d10(nodes, tets):
    """Reorder each element's midside nodes to CalculiX's C3D10 convention
    (found from the geometry, so any mesher's order works) and flip inverted
    tets. Returns a new (E,10) array; raises if a midside node is not an edge
    midpoint (a broken mesh)."""
    t = tets.copy()
    v = nodes[t[:, :4]]
    vol = np.einsum("ij,ij->i", np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0]), v[:, 3] - v[:, 0])
    neg = vol < 0
    t[neg, 1], t[neg, 2] = tets[neg, 2], tets[neg, 1]          # swap two vertices
    mids = nodes[t[:, 4:]]                                      # (E,6,3) in the mesher's order
    out = t.copy()
    for k, (a, b) in enumerate(C3D10_EDGES):
        want = (nodes[t[:, a]] + nodes[t[:, b]]) / 2            # (E,3)
        d = np.linalg.norm(mids - want[:, None, :], axis=2)     # (E,6)
        j = d.argmin(axis=1)
        edge = np.linalg.norm(nodes[t[:, a]] - nodes[t[:, b]], axis=1)
        if np.any(d[np.arange(len(t)), j] > 0.25 * edge + 1e-9):
            raise RuntimeError("midside node is not near its edge midpoint: curved or broken mesh")
        out[:, 4 + k] = t[np.arange(len(t)), 4 + j]
    return out


def boundary_faces(tets):
    """Corner-node triangles on the mesh surface (faces used by one tet),
    oriented outward."""
    c = tets[:, :4]
    f = np.concatenate([c[:, [0, 2, 1]], c[:, [0, 1, 3]], c[:, [1, 2, 3]], c[:, [0, 3, 2]]])
    key = np.sort(f, axis=1)
    _, idx, cnt = np.unique(key, axis=0, return_index=True, return_counts=True)
    return f[idx[cnt == 1]]


# ---------------------------------------------------------------- deck out
def _id_lines(ids, per=12):
    ids = list(ids)
    return "\n".join(", ".join(str(i) for i in ids[k:k + per]) for k in range(0, len(ids), per))


def write_ccx(path, nodes, tets, fix, load, ref, steps, E, nu):
    """CalculiX deck: C3D10 solid, `fix` nodes pinned (all translations),
    `load` nodes tied by a *RIGID BODY to a reference point at `ref`, one
    linear static *STEP per entry of steps = [(name, F(3), M(3))] with F in N
    and M in N.mm about `ref`. Node/element ids are row index + 1."""
    n = len(nodes)
    r_ref, r_rot = n + 1, n + 2
    with open(path, "w") as f:
        f.write("*HEADING\nrocky fem_check (D061)\n*NODE, NSET=NALL\n")
        for i, (x, y, z) in enumerate(nodes, 1):
            f.write(f"{i}, {x:.6f}, {y:.6f}, {z:.6f}\n")
        f.write(f"{r_ref}, {ref[0]:.6f}, {ref[1]:.6f}, {ref[2]:.6f}\n")
        f.write(f"{r_rot}, {ref[0]:.6f}, {ref[1]:.6f}, {ref[2]:.6f}\n")
        f.write("*ELEMENT, TYPE=C3D10, ELSET=EALL\n")
        for e, row in enumerate(tets + 1, 1):
            f.write(f"{e}, " + ", ".join(str(k) for k in row) + "\n")
        f.write("*NSET, NSET=NFIX\n" + _id_lines(np.flatnonzero(fix) + 1) + "\n")
        f.write("*NSET, NSET=NLOAD\n" + _id_lines(np.flatnonzero(load) + 1) + "\n")
        f.write(f"*NSET, NSET=NREF\n{r_ref}\n")
        f.write(f"*MATERIAL, NAME=PRINT\n*ELASTIC\n{E}, {nu}\n")
        f.write("*SOLID SECTION, ELSET=EALL, MATERIAL=PRINT\n")
        f.write(f"*RIGID BODY, NSET=NLOAD, REF NODE={r_ref}, ROT NODE={r_rot}\n")
        f.write("*BOUNDARY\nNFIX, 1, 3\n")
        for name, F, M in steps:
            f.write(f"** load case {name}\n*STEP\n*STATIC\n*CLOAD, OP=NEW\n")
            for k in range(3):
                f.write(f"{r_ref}, {k + 1}, {F[k]:.6f}\n")
            for k in range(3):
                f.write(f"{r_rot}, {k + 1}, {M[k]:.6f}\n")
            f.write("*NODE FILE\nU\n*EL FILE\nS\n*NODE PRINT, NSET=NREF\nU\n*END STEP\n")


def run_ccx(ccx, inp, log, threads=4):
    """Solve deck `inp` (path WITH .inp); results land beside it (.frd/.dat).

    The equation solver runs on ONE thread (B110, 2026-09-30): the Flatpak's ccx
    factors with SPOOLES, and multi-threaded SPOOLES now and then returns a wrong
    solution for one load step (a regen pass wrote one, the femur's R+, into
    FEM_REPORT.md). The femur deck solved 6 times, 3 at once, with 4 solver threads:
    2 runs had a corrupt step (V: von Mises off by up to 74 MPa at a node, its p99.9
    over all nodes 17.0 against 4.05; R+: 0.85); with 1 solver thread all 6 were
    identical to the good runs. Assembly and stress recovery keep `threads` (OMP):
    they gave identical results either way."""
    job = os.path.abspath(inp)[:-4]
    env = dict(os.environ, OMP_NUM_THREADS=str(threads),
               CCX_NPROC_EQUATION_SOLVER="1")
    with open(log, "w") as f:
        r = subprocess.run(ccx + ["-i", job], cwd=os.path.dirname(job), env=env,
                           stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0 or not os.path.exists(job + ".frd"):
        raise RuntimeError(f"ccx failed on {inp} (log: {log})")
    with open(log) as f:
        txt = f.read()
    if "*ERROR" in txt:
        raise RuntimeError(f"ccx reported an error on {inp} (log: {log})")


# ---------------------------------------------------------------- results back
def read_frd(path, n_nodes):
    """Per step, in order: {'U': (N,3), 'S': (N,6) as xx yy zz xy yz zx}.
    Nodes beyond n_nodes (the rigid-body reference nodes) are dropped."""
    steps, cur, block, ncomp = [], None, None, 0
    with open(path) as f:
        for ln in f:
            if ln.startswith(" -4"):
                name = ln.split()[1]
                block = {"DISP": "U", "STRESS": "S"}.get(name)
                if block:
                    ncomp = 3 if block == "U" else 6
                    if block == "U":
                        cur = {}
                        steps.append(cur)
                    cur[block] = np.zeros((n_nodes, ncomp))
            elif ln.startswith(" -1") and block:
                nid = int(ln[3:13])
                if nid <= n_nodes:
                    cur[block][nid - 1] = [float(ln[13 + 12 * k:25 + 12 * k]) for k in range(ncomp)]
            elif ln.startswith(" -3"):
                block = None
    return steps


def von_mises(s):
    xx, yy, zz, xy, yz, zx = s.T
    return np.sqrt(0.5 * ((xx - yy) ** 2 + (yy - zz) ** 2 + (zz - xx) ** 2)
                   + 3.0 * (xy ** 2 + yz ** 2 + zx ** 2))


def normal_stress(s, n):
    """Normal stress on the plane with unit normal n (tension positive) —
    across the print layers when n is the build direction."""
    xx, yy, zz, xy, yz, zx = s.T
    a, b, c = n
    return xx * a * a + yy * b * b + zz * c * c + 2 * (xy * a * b + yz * b * c + zx * a * c)


# ---------------------------------------------------------------- pictures
def render(png, nodes, tris, value, grip, title, peak, vmax=1.0):
    """Two iso views of the surface coloured by `value` (per node, 0..vmax);
    grip nodes (held / loaded) in grey; `peak` (a point) ringed in black."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    cmap = plt.get_cmap("turbo")
    fv = value[tris].max(axis=1) / vmax                       # max: a hot spot stays visible
    colors = cmap(np.clip(fv, 0, 1))
    colors[grip[tris].all(axis=1)] = (0.62, 0.62, 0.62, 1.0)
    lo, hi = nodes.min(axis=0), nodes.max(axis=0)
    ctr, half = (lo + hi) / 2, (hi - lo).max() / 2
    fig = plt.figure(figsize=(12, 6), dpi=110)
    for k, (elev, azim) in enumerate(((24, -58), (-20, 122))):
        ax = fig.add_axes([0.0 + 0.44 * k, 0.02, 0.46, 0.9], projection="3d")
        ax.add_collection3d(Poly3DCollection(nodes[tris], facecolors=colors, linewidths=0))
        ax.scatter(*peak, s=260, facecolors="none", edgecolors="black", linewidths=2, depthshade=False)
        for i, set_lim in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
            set_lim(ctr[i] - half, ctr[i] + half)
        ax.set_box_aspect((1, 1, 1), zoom=1.25)
        ax.view_init(elev=elev, azim=azim)
        ax.set_axis_off()
    cax = fig.add_axes([0.91, 0.15, 0.015, 0.65])
    cb = fig.colorbar(plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(0, vmax)), cax=cax)
    cb.set_label("utilisation = stress / allowable (1.0 breaks); grey = held or loaded; ring = peak")
    fig.suptitle(title, fontsize=10)
    fig.savefig(png, metadata={"Software": None})
    plt.close(fig)
