"""Render STL previews to PNG for visual inspection (headless matplotlib)."""
import sys, os
import numpy as np
from stl import mesh as stlmesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

def render(stl_path, png_path, elev=22, azim=-60, title=""):
    m = stlmesh.Mesh.from_file(stl_path)
    tris = m.vectors
    # face shading by normal
    n = m.normals
    n = n / (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
    light = np.array([0.4, 0.35, 0.85])
    shade = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
    face_colors = np.zeros((len(tris), 4))
    base = np.array([0.55, 0.42, 0.75])  # pebble purple
    face_colors[:, :3] = base * shade[:, None]
    face_colors[:, 3] = 1.0

    fig = plt.figure(figsize=(9, 7), dpi=110)
    ax = fig.add_subplot(111, projection="3d")
    coll = Poly3DCollection(tris, facecolors=face_colors, edgecolors="none")
    ax.add_collection3d(coll)
    lo = tris.reshape(-1, 3).min(0); hi = tris.reshape(-1, 3).max(0)
    c = (lo + hi) / 2; r = (hi - lo).max() / 2 * 1.05
    ax.set_xlim(c[0]-r, c[0]+r); ax.set_ylim(c[1]-r, c[1]+r); ax.set_zlim(c[2]-r, c[2]+r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("X mm"); ax.set_ylabel("Y mm"); ax.set_zlabel("Z mm")
    ax.set_title(title or os.path.basename(stl_path))
    plt.tight_layout()
    plt.savefig(png_path)
    plt.close(fig)
    print("rendered", png_path)

if __name__ == "__main__":
    jobs = [
        ("servo_st3215_dummy", 20, -55, None),
        ("coxa_yaw_base", 22, -60, None),
        ("coxa_yaw_base", 18, 120, "coxa_yaw_base (rear)"),
        ("coxa_fork", 22, -60, None),
        ("coxa_fork", 30, 140, "coxa_fork (rear)"),
        ("femur_link", 25, -75, None),
        ("leg_skeleton_assembly", 18, -55, None),
        ("leg_skeleton_assembly", 8, -90, "leg skeleton (side, -Y)"),
    ]
    for name, elev, azim, title in jobs:
        suffix = "" if title is None else "_" + title.split("(")[-1].rstrip(")").replace(", ", "_").replace(" ", "_")
        render(os.path.join(OUT, f"{name}.stl"),
               os.path.join(OUT, f"{name}{suffix}.png"),
               elev, azim, title or name)
