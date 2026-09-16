"""2D lidar scan matching — point-to-line ICP, pure numpy/scipy.

The no-ROS SLAM front-end (D024 groundwork): slam_toolbox will eventually
own this job on the Pi, but the math is small enough to own outright, and
building it exposes every frame/convention decision the ROS stack will
need. Runs against sim/lidar_scans.npz today; against the LDS02RR puck the
day it exists (same LaserScan contract, sim/laserscan_spec.json).

Method: classic point-to-line ICP (Censi's PLICP shape, trimmed):
  * target scan -> 2D points + per-point normals (PCA of k nearest)
  * iterate: nearest-neighbor correspondences (cKDTree), reject beyond
    max_corr + trim the worst tail, then solve the linearized normal
    least-squares for (dx, dy, dtheta) and re-compose.
  * converges in <10 iterations seeded by legged-odom deltas (~mm here).

Conventions:
  SE(2) pose = np.array([x, y, theta]) mapping LOCAL -> WORLD:
      p_world = R(theta) @ p_local + [x, y]
  match(src, tgt, T0) returns T such that src points land on tgt points:
      p_tgt ≈ T ⊕ p_src   (T0 = odometry seed, e.g. inv(pose_tgt)⊕pose_src)

Self-test: python3 scan_matching.py  (synthetic-room recovery to <2 mm).
"""
from __future__ import annotations
import numpy as np
from scipy.spatial import cKDTree


# ----------------------------------------------------------- SE(2) helpers
def se2_apply(T, pts):
    c, s = np.cos(T[2]), np.sin(T[2])
    R = np.array([[c, -s], [s, c]])
    return pts @ R.T + T[:2]


def se2_compose(A, B):
    """A ⊕ B: apply B first, then A."""
    c, s = np.cos(A[2]), np.sin(A[2])
    return np.array([A[0] + c * B[0] - s * B[1],
                     A[1] + s * B[0] + c * B[1],
                     A[2] + B[2]])


def se2_inv(T):
    c, s = np.cos(T[2]), np.sin(T[2])
    return np.array([-(c * T[0] + s * T[1]),
                     -(-s * T[0] + c * T[1]),
                     -T[2]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# ----------------------------------------------------------- scan -> points
def scan_points(ranges, angles, r_min=0.05, r_max=8.0, stride=1):
    """Finite returns -> (N,2) points in the sensor frame."""
    r = np.asarray(ranges, dtype=float)[::stride]
    a = np.asarray(angles, dtype=float)[::stride]
    ok = np.isfinite(r) & (r > r_min) & (r < r_max)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def estimate_normals(pts, k=8):
    """Per-point unit normals from PCA of the k-neighborhood."""
    tree = cKDTree(pts)
    n = len(pts)
    k = min(k, n)
    _, idx = tree.query(pts, k=k)
    nb = pts[idx]                                # (n, k, 2)
    nb = nb - nb.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", nb, nb) / k
    evals, evecs = np.linalg.eigh(cov)
    return evecs[:, :, 0]                        # smallest-eigval direction


# ----------------------------------------------------------------- the ICP
def icp_p2l(src_pts, tgt_pts, T0=None, iters=25, max_corr=0.30,
            trim=0.9, tol=1e-5, tgt_tree=None, tgt_normals=None):
    """Point-to-line ICP. Returns (T, info) with p_tgt ≈ T ⊕ p_src.

    info: dict(rmse, matches, iters, converged). Degenerate geometry
    (a single straight wall) leaves the along-wall direction unconstrained —
    the caller's odometry seed is what anchors it; do not feed max_corr
    larger than the odometry error scale or wrong walls will pair up."""
    T = np.zeros(3) if T0 is None else np.asarray(T0, dtype=float).copy()
    if len(src_pts) < 10 or len(tgt_pts) < 10:
        return T, dict(rmse=np.inf, matches=0, iters=0, converged=False)
    tree = tgt_tree if tgt_tree is not None else cKDTree(tgt_pts)
    normals = tgt_normals if tgt_normals is not None \
        else estimate_normals(tgt_pts)
    info = dict(rmse=np.inf, matches=0, iters=0, converged=False)
    for it in range(iters):
        moved = se2_apply(T, src_pts)
        dist, j = tree.query(moved)
        keep = dist < max_corr
        if keep.sum() < 10:
            break
        # trim the worst tail (dynamic objects / clutter robustness)
        if trim < 1.0:
            thresh = np.quantile(dist[keep], trim)
            keep &= dist <= thresh
        p = moved[keep]
        q = tgt_pts[j[keep]]
        n = normals[j[keep]]
        r = np.einsum("ij,ij->i", p - q, n)      # signed point-line dists
        # unknowns (dx, dy, dth): residual model r + n·[dx,dy] + n·(J p)dth
        Jp = np.stack([-p[:, 1], p[:, 0]], axis=1)
        A = np.stack([n[:, 0], n[:, 1],
                      np.einsum("ij,ij->i", n, Jp)], axis=1)
        try:
            dx, *_ = np.linalg.lstsq(A, -r, rcond=None)
        except np.linalg.LinAlgError:
            break
        T = np.array([T[0] + dx[0], T[1] + dx[1], wrap(T[2] + dx[2])])
        info = dict(rmse=float(np.sqrt(np.mean(r * r))),
                    matches=int(keep.sum()), iters=it + 1,
                    converged=bool(np.abs(dx).max() < tol))
        if info["converged"]:
            break
    return T, info


class KeyframeMatcher:
    """Scan-to-keyframe odometry correction.

    Matching every scan to the PREVIOUS scan accumulates ICP noise at scan
    rate; matching to a held KEYFRAME accumulates it only when the keyframe
    hands off. Seed every match with the odometry DELTA since the keyframe
    (the EKF is exactly good enough for that), correct the absolute pose
    with the ICP result."""

    def __init__(self, angles, kf_dist=0.18, kf_yaw=np.deg2rad(10),
                 stride=2, **icp_kw):
        self.angles = np.asarray(angles)
        self.kf_dist = kf_dist
        self.kf_yaw = kf_yaw
        self.stride = stride
        self.icp_kw = icp_kw
        self.kf_pts = None
        self.kf_tree = None
        self.kf_normals = None
        self.kf_pose = None                      # corrected pose of keyframe
        self.kf_odom = None                      # odom pose at keyframe
        self.n_keyframes = 0

    def _set_kf(self, pts, pose, odom):
        self.kf_pts = pts
        self.kf_tree = cKDTree(pts)
        self.kf_normals = estimate_normals(pts)
        self.kf_pose = pose.copy()
        self.kf_odom = odom.copy()
        self.n_keyframes += 1

    def update(self, ranges, odom_pose):
        """Feed one scan + its odometry pose; returns the corrected pose."""
        pts = scan_points(ranges, self.angles, stride=self.stride)
        odom_pose = np.asarray(odom_pose, dtype=float)
        if self.kf_pts is None:
            self._set_kf(pts, odom_pose, odom_pose)
            return odom_pose.copy(), dict(rmse=0.0, matches=len(pts),
                                          iters=0, converged=True)
        # odometry delta since the keyframe = the ICP seed
        T_seed = se2_compose(se2_inv(self.kf_odom), odom_pose)
        T_seed[2] = wrap(T_seed[2])
        T, info = icp_p2l(pts, self.kf_pts, T0=T_seed,
                          tgt_tree=self.kf_tree, tgt_normals=self.kf_normals,
                          **self.icp_kw)
        pose = se2_compose(self.kf_pose, T)
        pose[2] = wrap(pose[2])
        if (np.hypot(*T[:2]) > self.kf_dist) or (abs(T[2]) > self.kf_yaw):
            self._set_kf(pts, pose, odom_pose)
        return pose, info


# ------------------------------------------------------------- self-test
def _synthetic_room(n=700, seed=0):
    rng = np.random.default_rng(seed)
    pts = []
    for (x0, y0, x1, y1) in [(-1.6, -1.3, 1.6, -1.3), (1.6, -1.3, 1.6, 1.3),
                             (1.6, 1.3, -1.6, 1.3), (-1.6, 1.3, -1.6, -1.3)]:
        t = rng.uniform(0, 1, n // 4)
        pts.append(np.stack([x0 + (x1 - x0) * t, y0 + (y1 - y0) * t], 1))
    th = rng.uniform(0, 2 * np.pi, 60)
    pts.append(np.stack([0.8 + 0.09 * np.cos(th), 0.55 + 0.09 * np.sin(th)], 1))
    return np.concatenate(pts)


def _selftest():
    rng = np.random.default_rng(1)
    room = _synthetic_room()
    true_T = np.array([0.062, -0.041, np.deg2rad(4.5)])
    # sensor sees the room from two poses: tgt at origin, src displaced
    tgt = room + rng.normal(0, 0.004, room.shape)
    src_world = room[rng.permutation(len(room))[:500]]
    src = se2_apply(se2_inv(true_T), src_world) + rng.normal(0, 0.004,
                                                             (500, 2))
    T, info = icp_p2l(src, tgt, T0=np.zeros(3))
    err_t = np.hypot(*(T[:2] - true_T[:2])) * 1000
    err_th = np.rad2deg(abs(wrap(T[2] - true_T[2])))
    print(f"recovered T = ({T[0]:+.4f}, {T[1]:+.4f}, {np.rad2deg(T[2]):+.2f}°)"
          f" vs true ({true_T[0]:+.4f}, {true_T[1]:+.4f}, "
          f"{np.rad2deg(true_T[2]):+.2f}°)")
    print(f"error: {err_t:.1f} mm, {err_th:.3f}°  "
          f"(rmse {info['rmse'] * 1000:.1f} mm, {info['matches']} matches, "
          f"{info['iters']} iters)")
    assert err_t < 5.0 and err_th < 0.15, "ICP self-test failed"
    # cold-start robustness: no seed at all
    T2, _ = icp_p2l(src, tgt, T0=np.zeros(3), iters=50)
    assert np.hypot(*(T2[:2] - true_T[:2])) * 1000 < 5.0
    print("self-test PASS (including unseeded start)")


if __name__ == "__main__":
    _selftest()
