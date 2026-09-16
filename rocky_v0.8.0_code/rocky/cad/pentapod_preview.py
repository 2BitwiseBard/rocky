"""Full-robot cosmetic preview: 5 legs at 72 deg + pentagon body + organic shells.

Strategy: build ONE leg (skeleton union + shell union) in leg-local frame with
coarse tessellation, then instance it 5x at the mesh level with numpy (fast,
and guarantees the five legs are identical). Body (deck + stepped carapace)
is built once in the body frame.

Outputs:
  out/preview_skeleton.stl   all mechanical parts, 5 legs
  out/preview_shells.stl     carapace + limb fairings + closed-hand feet
  out/pebble_full_preview.png  color overlay render (the "cohesion" shot)
"""
import numpy as np
from stl import mesh as stlmesh
from build123d import *
from common import params, OUT
import os

P = params()
S = P["servo_st3215"]
L1 = P["leg"]["l1_coxa"]; L2 = P["leg"]["l2_femur"]; L3 = P["leg"]["l3_tibia"]
R_BODY = P["body"]["circumradius"]

COARSE = dict(tolerance=0.2, angular_tolerance=0.5)

def export_coarse(part, name):
    path = os.path.join(OUT, f"{name}.stl")
    export_stl(part, path, **COARSE)
    print("coarse export:", name)
    return path

# ---------------- one leg, local frame ----------------
def leg_skeleton_solid():
    from leg_assembly import build
    parts = build()
    del parts["foot"]                       # replaced by the closed hand
    del parts["tibia_tube"]
    combined = None
    for s in parts.values():
        combined = s if combined is None else combined + s
    # short visible tube: knee down to the hand's tube socket
    combined += Pos(L1 + L2, 0, 28) * Cylinder(P["leg"]["tibia_tube_od"]/2, 60)
    return combined

def leg_shell_solid():
    from part_hand import hand_assembly
    knee_x = L1 + L2
    # femur fairing: chunky taper hip->knee
    fem = Pos((L1 + knee_x)/2, 0, 58) * Rot(0, 90, 0) * Cone(24, 16, L2 + 14)
    # hip bulb over the coxa fork zone
    hip = Pos(L1 - 12, 0, 56) * Sphere(24)
    # knee bulb over the tibia servo
    knee = Pos(knee_x + 6, 2, 58) * Sphere(21)
    # tibia fairing: taper down the shin
    tib = Pos(knee_x, 0, 12) * Cone(15, 10, 66)
    shell = fem + hip + knee + tib
    # closed hand as the foot: hand frame +Z -> world -Z, tip lands at z ~ -77
    hand = None
    for s in hand_assembly(0.0).values():
        hand = s if hand is None else hand + s
    shell += Pos(knee_x, 0, 1) * Rot(180, 0, 0) * hand
    return shell

# ---------------- body ----------------
def body_solid():
    """The deck slab (mechanism side). The carapace is no longer a
    placeholder: preview_shells now carries the REAL printed sector
    (part_shell.py, session 5) instanced 5x at mesh level, plus the cap."""
    return Pos(0, 0, -8) * extrude(RegularPolygon(100, 5), 4)


def body_sector_solid():
    from part_shell import shell_sector
    return shell_sector()


def body_cap_solid():
    from part_shell import shell_cap
    return shell_cap()

# ---------------- mesh instancing ----------------
def load(name):
    return stlmesh.Mesh.from_file(os.path.join(OUT, f"{name}.stl"))

def instance5(name, offset_x=None):
    """Rotate-instance a mesh 5x at 72 deg. offset_x: pre-rotation radial
    offset (legs are built leg-local and need R_BODY; body-frame parts
    like the shell sector pass 0)."""
    base = load(name)
    off = R_BODY if offset_x is None else offset_x
    metas = []
    for i in range(5):
        th = np.deg2rad(72 * i)
        Rz = np.array([[np.cos(th), -np.sin(th), 0],
                       [np.sin(th),  np.cos(th), 0],
                       [0, 0, 1]])
        m = stlmesh.Mesh(base.data.copy())
        v = m.vectors.reshape(-1, 3) + np.array([off, 0, 0])
        m.vectors[:] = (v @ Rz.T).reshape(-1, 3, 3)
        metas.append(m.data)
    out = stlmesh.Mesh(np.concatenate(metas))
    out.update_normals()
    return out

def concat(meshes, name):
    out = stlmesh.Mesh(np.concatenate([m.data for m in meshes]))
    out.update_normals()
    path = os.path.join(OUT, f"{name}.stl")
    out.save(path)
    tris = len(out.vectors)
    print(f"saved {name}: {tris} tris")
    return path

if __name__ == "__main__":
    export_coarse(leg_skeleton_solid(), "preview_leg_skeleton")
    export_coarse(leg_shell_solid(), "preview_leg_shell")
    export_coarse(body_solid(), "preview_body")
    export_coarse(body_sector_solid(), "preview_shell_sector")
    export_coarse(body_cap_solid(), "preview_shell_cap")

    skel5 = instance5("preview_leg_skeleton")
    shell5 = instance5("preview_leg_shell")
    sectors5 = instance5("preview_shell_sector", offset_x=0.0)
    body = load("preview_body")
    cap = load("preview_shell_cap")

    concat([skel5, body], "preview_skeleton")
    concat([shell5, sectors5, cap], "preview_shells")

    # ---- color overlay render: the cohesion shot ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(11, 8.5), dpi=110)
    ax = fig.add_subplot(111, projection="3d")
    for name, rgb, alpha in [("preview_shells", (0.55, 0.42, 0.75), 1.0),
                             ("preview_skeleton", (0.22, 0.22, 0.27), 1.0)]:
        m = load(name)
        n = m.normals / (np.linalg.norm(m.normals, axis=1, keepdims=True) + 1e-12)
        shade = 0.35 + 0.65 * np.clip(n @ np.array([0.4, 0.35, 0.85]), 0, 1)
        fc = np.zeros((len(m.vectors), 4))
        fc[:, :3] = np.array(rgb) * shade[:, None]
        fc[:, 3] = alpha
        ax.add_collection3d(Poly3DCollection(m.vectors, facecolors=fc, edgecolors="none"))
    r = 260
    ax.set_xlim(-r, r); ax.set_ylim(-r, r); ax.set_zlim(-170, 90)
    ax.set_box_aspect((1, 1, 0.5))
    ax.view_init(elev=16, azim=-50)
    ax.set_axis_off()
    ax.set_title("Pebble — full-robot cosmetic preview (shells over skeleton)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, "pebble_full_preview.png"), facecolor="white")
    print("render saved")
