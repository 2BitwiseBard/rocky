"""SLAM-lite: ICP-corrected mapping over the sim-lidar lap — no ROS needed.

Session-4 built the occupancy demo from GROUND-TRUTH poses (a data-quality
proof, not SLAM). This closes the loop the honest way:

  odometry (legged-odom EKF, in-loop, ~4-6 % / a few deg per lap)
      -> seeds KeyframeMatcher (point-to-line ICP, perception/scan_matching)
      -> corrected trajectory -> occupancy map

and quantifies all three against truth: raw-EKF map vs ICP-corrected map
vs ground-truth map. Metrics: trajectory ATE (rmse + end), yaw error, and
map IoU vs the ground-truth map (binarized occupancy, hits >= 2).

Usage: python3 run_slam_lite.py     (reads lidar_scans.npz)
Outputs: slam_results.json, fig_slam.png
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
from scan_matching import KeyframeMatcher, wrap                     # noqa: E402

RES = 0.02
X0, X1 = -1.9, 1.9
Y0, Y1 = -1.6, 1.6


def build_grid(scans, poses, angles):
    W = int((X1 - X0) / RES)
    H = int((Y1 - Y0) / RES)
    grid = np.zeros((H, W))
    for s, (px, py, yaw) in zip(scans, poses):
        ok = np.isfinite(s)
        wx = px + s[ok] * np.cos(angles[ok] + yaw)
        wy = py + s[ok] * np.sin(angles[ok] + yaw)
        gx = ((wx - X0) / RES).astype(int)
        gy = ((wy - Y0) / RES).astype(int)
        m = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        np.add.at(grid, (gy[m], gx[m]), 1)
    return grid


def map_iou(grid, ref_grid, thresh=2):
    a = grid >= thresh
    b = ref_grid >= thresh
    inter = (a & b).sum()
    union = (a | b).sum()
    return float(inter) / max(union, 1)


def traj_metrics(est, gt):
    e = est[:, :2] - gt[:, :2]
    d = np.linalg.norm(e, axis=1)
    ye = np.rad2deg(np.abs(wrap(est[:, 2] - gt[:, 2])))
    return dict(ate_rmse_mm=round(float(np.sqrt((d ** 2).mean())) * 1000, 1),
                end_err_mm=round(float(d[-1]) * 1000, 1),
                max_err_mm=round(float(d.max()) * 1000, 1),
                yaw_end_deg=round(float(ye[-1]), 2),
                yaw_max_deg=round(float(ye.max()), 2))


def main():
    d = np.load(os.path.join(HERE, "lidar_scans.npz"))
    scans, gt, odom, angles = d["scans"], d["poses"], d["poses_odom"], d["angles"]
    print(f"{len(scans)} scans; odom end error "
          f"{np.linalg.norm(odom[-1, :2] - gt[-1, :2]) * 1000:.0f} mm / "
          f"{np.rad2deg(abs(wrap(odom[-1, 2] - gt[-1, 2]))):.2f} deg")

    matcher = KeyframeMatcher(angles, kf_dist=0.18, kf_yaw=np.deg2rad(10),
                              stride=2, max_corr=0.30, trim=0.9)
    slam = np.zeros_like(odom)
    rmses = []
    for k in range(len(scans)):
        pose, info = matcher.update(scans[k], odom[k])
        slam[k] = pose
        if k:
            rmses.append(info["rmse"])
    print(f"keyframes: {matcher.n_keyframes}; scan-fit rmse "
          f"median {np.median(rmses) * 1000:.1f} mm")

    res = {
        "n_scans": int(len(scans)),
        "n_keyframes": int(matcher.n_keyframes),
        "icp_fit_rmse_median_mm": round(float(np.median(rmses)) * 1000, 2),
        "odom": traj_metrics(odom, gt),
        "slam": traj_metrics(slam, gt),
    }

    g_gt = build_grid(scans, gt, angles)
    g_odom = build_grid(scans, odom, angles)
    g_slam = build_grid(scans, slam, angles)
    res["map_iou_odom_vs_gt"] = round(map_iou(g_odom, g_gt), 3)
    res["map_iou_slam_vs_gt"] = round(map_iou(g_slam, g_gt), 3)

    with open(os.path.join(HERE, "slam_results.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps(res, indent=1))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 4, figsize=(17, 4.4))
    for ax, (g, title, iou) in zip(
            axes[:3],
            [(g_gt, "ground-truth poses", None),
             (g_odom, "raw EKF odometry", res["map_iou_odom_vs_gt"]),
             (g_slam, "ICP-corrected (SLAM-lite)", res["map_iou_slam_vs_gt"])]):
        ax.imshow(np.clip(g, 0, 6), origin="lower", cmap="Purples",
                  extent=[X0, X1, Y0, Y1])
        t = title if iou is None else f"{title}\nmap IoU {iou:.2f}"
        ax.set_title(t, fontsize=10)
    ax = axes[3]
    ax.plot(gt[:, 0], gt[:, 1], "k-", lw=2, label="truth")
    ax.plot(odom[:, 0], odom[:, 1], "--", color="#bbbbbb", lw=1.6,
            label=f"EKF ({res['odom']['end_err_mm']:.0f} mm end)")
    ax.plot(slam[:, 0], slam[:, 1], "-", color="#7b3fa0", lw=1.6,
            label=f"SLAM-lite ({res['slam']['end_err_mm']:.0f} mm end)")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("trajectories", fontsize=10)
    fig.suptitle("Scan-matching SLAM groundwork: EKF-seeded point-to-line ICP "
                 f"(keyframes: {matcher.n_keyframes})")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_slam.png"), dpi=120)
    print("wrote slam_results.json + fig_slam.png")


if __name__ == "__main__":
    main()
