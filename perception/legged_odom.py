"""Legged odometry — error-state EKF fusing IMU with leg-kinematic velocity.

The robot's only proprioceptive pose source until the lidar exists (and the
prior that makes lidar SLAM cheap afterwards). Inputs are exactly what the
hardware will have (D010, Batch 3): BNO085 gyro+accel, joint encoders (servo
positions), SEA microswitch contact flags. No cameras, no magic.

Method (Bloesch-style, trimmed):
  nominal state: p, v, q (world<-body), gyro bias, accel bias
  error state (15): dp, dv, dtheta, dbg, dba
  predict: strapdown IMU integration at imu_hz
  update: for each STANCE leg (contact switch closed), the foot is
  stationary, so body-frame velocity is fully observable from kinematics:
      v_body = -(J(theta) theta_dot + omega x p_foot(theta))
  measured world velocity r = R(q) v_body vs filter v -> standard EKF update
  (per-leg, so one slipping foot is one bad measurement, not a poisoned mean).

Pure numpy; runs on the Pi later. The MuJoCo harness is sim/run_odom.py.
"""
from __future__ import annotations
import numpy as np


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def small_quat(dtheta):
    half = 0.5 * dtheta
    return np.concatenate([[1.0], half]) / np.sqrt(1 + half @ half)


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


class StillnessGate:
    """Decide when the robot is REALLY still enough for zero-rate updates.

    Sim measurement (2026-07-31, sim_lidar settle): after any pose change a
    position-servo legged robot keeps a slowly DECAYING real twist — the
    stance relaxing — at 0.3–1.2 deg/s for over a second. That is exactly
    gyro-bias scale, so a zero-yaw-rate update fired too early calibrates
    the creep INTO the bias with the wrong sign (measured: bg_z -> -0.0109
    vs true +0.0050, which turned a 6 deg lap into a 37 deg lap). Rule:
    trust stillness only after quiet_wait seconds of zero command AND full
    contact; the creep has decayed below the bias by then. Calibrate at the
    END of pauses, never the start."""

    def __init__(self, quiet_wait=1.2):
        self.quiet_wait = quiet_wait
        self._quiet_since = None

    def update(self, t, commanded_moving, contacts_all) -> bool:
        if commanded_moving or not contacts_all:
            self._quiet_since = None
            return False
        if self._quiet_since is None:
            self._quiet_since = t
        return (t - self._quiet_since) >= self.quiet_wait


class LeggedOdomEKF:
    def __init__(self, fk_body, n_legs=5,
                 sigma_g=0.02, sigma_a=0.15,      # noise densities (per-sample)
                 sigma_bg=1e-4, sigma_ba=1e-3,
                 sigma_v_meas=0.03,               # kinematic velocity noise m/s
                 g=9.81):
        """fk_body(i, theta) -> foot position in BODY frame (meters)."""
        self.fk = fk_body
        self.n_legs = n_legs
        self.g = np.array([0, 0, -g])
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0, 0, 0])
        self.bg = np.zeros(3)
        self.ba = np.zeros(3)
        # gyro-bias prior: (0.01 rad/s)^2 — a realistic turn-on bias for a
        # BNO085-class MEMS gyro. The old 1e-6 prior said "bias is known to
        # 0.001 rad/s" — 5 sigma overconfident vs the actual 0.005 rad/s,
        # so the zero-yaw-rate update could barely move it (2026-07-31).
        self.P = np.diag([1e-6] * 3 + [1e-4] * 3 + [1e-6] * 3
                         + [1e-4] * 3 + [1e-4] * 3)
        self.Qc = np.diag([sigma_g**2] * 3 + [sigma_a**2] * 3
                          + [sigma_bg**2] * 3 + [sigma_ba**2] * 3)
        self.Rv = np.eye(3) * sigma_v_meas**2
        self.last_omega = np.zeros(3)

    # ---------------------------------------------------------- predict
    def predict(self, gyro, accel, dt):
        omega = gyro - self.bg
        acc_b = accel - self.ba
        R = quat_to_R(self.q)
        self.last_omega = omega
        # nominal
        self.q = quat_mul(self.q, small_quat(omega * dt))
        self.q /= np.linalg.norm(self.q)
        a_w = R @ acc_b + self.g
        self.p = self.p + self.v * dt + 0.5 * a_w * dt * dt
        self.v = self.v + a_w * dt
        # error-state transition
        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        F[3:6, 6:9] = -R @ skew(acc_b) * dt
        F[3:6, 12:15] = -R * dt
        F[6:9, 6:9] = np.eye(3) - skew(omega) * dt
        F[6:9, 9:12] = -np.eye(3) * dt
        G = np.zeros((15, 12))
        G[3:6, 3:6] = R
        G[6:9, 0:3] = np.eye(3)
        G[9:12, 6:9] = np.eye(3)
        G[12:15, 9:12] = np.eye(3)
        self.P = F @ self.P @ F.T + G @ self.Qc @ G.T * dt

    # ---------------------------------------------------------- update
    def _kinematic_v_body(self, i, theta, theta_dot):
        """v_body from a stationary stance foot (numeric jacobian)."""
        eps = 1e-6
        p0 = self.fk(i, theta)
        J = np.zeros((3, 3))
        for k in range(3):
            tp = np.array(theta, dtype=float)
            tp[k] += eps
            J[:, k] = (self.fk(i, tp) - p0) / eps
        return -(J @ theta_dot + np.cross(self.last_omega, p0)), p0

    def update_legs(self, thetas, theta_dots, contacts):
        """thetas: [n_legs][3] rad; contacts: bool[n_legs]."""
        for i in range(self.n_legs):
            if not contacts[i]:
                continue
            v_b, p_f = self._kinematic_v_body(i, thetas[i], theta_dots[i])
            R = quat_to_R(self.q)
            z = R @ v_b                       # measured world velocity
            r = z - self.v                    # residual
            # r = dv + R [v_b]x dtheta + n   (first-order error model)
            H = np.zeros((3, 15))
            H[:, 3:6] = np.eye(3)
            H[:, 6:9] = R @ skew(v_b)
            S = H @ self.P @ H.T + self.Rv
            # gate: a slipping foot produces a wild residual — skip it
            m2 = r @ np.linalg.solve(S, r)
            if m2 > 16.27:                    # chi2 0.999, dof 3
                continue
            K = self.P @ H.T @ np.linalg.inv(S)
            dx = K @ r
            self._inject(dx)
            IKH = np.eye(15) - K @ H
            self.P = IKH @ self.P @ IKH.T + K @ self.Rv @ K.T

    # ------------------------------------------------ yaw observability
    # Nothing in the IMU+legs stack observes yaw or gyro-z bias: the leg
    # updates constrain VELOCITY, gravity constrains roll/pitch, and yaw
    # integrates gyro-z open-loop. Measured 2026-07-30: ~6 deg / 21 s —
    # almost exactly GYRO_BIAS_z * t. Two fixes, both cheap:

    def update_yaw_rate_zero(self, sigma=0.01):
        """STANDING-STILL pseudo-measurement: true yaw rate is zero, so the
        bias-corrected gyro-z should read zero and any residual IS gyro-z
        bias error. Call while the robot is quiet (commanded v == 0, all
        feet in contact, kinematic v small) — e.g. the pre-walk settle and
        every patrol pause. Makes dbg_z observable; between stands the yaw
        drifts only at the RESIDUAL bias rate."""
        h = self.last_omega[2]                # predicted yaw rate (g_z - bg_z)
        r = 0.0 - h
        H = np.zeros((1, 15))
        H[0, 11] = -1.0                       # d(g_z - bg_z)/d(dbg_z)
        S = float((H @ self.P @ H.T).item()) + sigma * sigma
        if r * r / S > 10.83:                 # chi2 0.999, dof 1 — not still?
            return False
        K = (self.P @ H.T / S).reshape(15)
        self._inject(K * r)
        IKH = np.eye(15) - np.outer(K, H)
        self.P = IKH @ self.P @ IKH.T + np.outer(K, K) * sigma * sigma
        return True

    def update_mag_yaw(self, yaw_meas, sigma=np.deg2rad(8.0)):
        """Magnetometer stub: absolute yaw with generous noise (indoor mag
        is a liar near motors/steel — gate hard, weight low; the BNO085
        exposes it either way). Optional: the zero-yaw-rate update alone
        already kills most of the drift."""
        R = quat_to_R(self.q)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        r = (yaw_meas - yaw + np.pi) % (2 * np.pi) - np.pi
        H = np.zeros((1, 15))
        # body-frame attitude error -> yaw: world-z row of R
        H[0, 6:9] = R[2, :]
        S = float((H @ self.P @ H.T).item()) + sigma * sigma
        if r * r / S > 10.83:
            return False
        K = (self.P @ H.T / S).reshape(15)
        self._inject(K * r)
        IKH = np.eye(15) - np.outer(K, H)
        self.P = IKH @ self.P @ IKH.T + np.outer(K, K) * sigma * sigma
        return True

    def _inject(self, dx):
        self.p += dx[0:3]
        self.v += dx[3:6]
        self.q = quat_mul(self.q, small_quat(dx[6:9]))
        self.q /= np.linalg.norm(self.q)
        self.bg += dx[9:12]
        self.ba += dx[12:15]

    # ---------------------------------------------------------- accessors
    @property
    def yaw(self):
        R = quat_to_R(self.q)
        return np.arctan2(R[1, 0], R[0, 0])
