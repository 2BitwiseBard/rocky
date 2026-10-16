"""Estimated pose for the cockpit: what a REAL robot would know about where it is.

The cockpit's pose() reads MuJoCo's torso: perfect odometry no robot has. This
runs perception/legged_odom.py's LeggedOdomEKF on exactly what the hardware
will measure (D010): a BNO085-class IMU (gyro + accel, with turn-on bias and
white noise), the servos' position + speed telemetry (encoder noise) and the
foot contact flags. Its estimate drifts the way the real one will, so the
cockpit can drive on it and show true vs estimated.

Noise model = sim/scenes.py's IMU / encoder constants (GYRO_NOISE, GYRO_BIAS,
ACC_NOISE, ACC_BIAS, ENC_NOISE, QVEL_NOISE): the calibration every odometry
experiment (run_odom, run_odom_v2, sim_lidar) was measured with. Seeded, so a
run is repeatable. The zero-yaw-rate update (StillnessGate: 1.2 s of quiet +
all feet down) calibrates the gyro-z bias at every pause; nothing else
observes yaw, so between pauses yaw drifts at the residual bias.

    odo = CockpitOdometry(model, seed=0)        # once; again (or bind()) after a world rebuild
    odo.reset({"x": 0, "y": 0, "yaw_deg": 0}, data)   # spawn / respawn: the pose the robot KNOWS
    odo.step(model, data, t, moving, contacts)  # every physics tick (runs the EKF at IMU_HZ)
    odo.estimate()      # {"x", "y", "yaw_deg"} — CockpitSim.pose()'s frame, units and rounding
    odo.drift(sim.pose())   # {"pos_m", "yaw_deg"} estimate minus truth (yaw wrapped)
    odo.odometer()      # {"dist_m", "turned_deg"} since reset: path length, signed net turn
    odo.status()        # the above + degraded / why / events / gyro-z bias / cost

moving: True whenever the robot is commanded to move or is not simply standing
(a velocity ask, a goto, a gesture, the reflex not NORMAL): the stillness gate
must never calibrate during motion. contacts: the robot's own foot switches
(Playground.last["con"]); None computes the experiments' 1.5 N force switch.

Falls: while the reflex is FALLEN / RIGHTED, the body is tilted more
than FALL_TILT_DEG, or fewer than MIN_STANCE feet are down, the estimate is
SUSPENDED: attitude keeps integrating the gyro (it must, to know which way is
up), but position is frozen and velocity zeroed — an accelerometer through a
fall integrates into metres of nonsense. It resumes after RESUME_S of a sane
stance, with position and yaw uncertainty inflated: the robot honestly does
not know how far the fall moved it (never reseeded from truth). A non-finite
state rolls back to the last good one (counted in events)."""
from __future__ import annotations

import math
from collections import deque
import os
import sys
import time

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (HERE, os.path.join(HERE, "..", "gait"), os.path.join(HERE, "..", "perception")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from legged_odom import LeggedOdomEKF, StillnessGate, quat_to_R       # noqa: E402
from pebble_gait import L1, L2, L3, STATION_DEG                       # noqa: E402
from scenes import (fk_body, IMU_HZ, GYRO_NOISE, ACC_NOISE, QVEL_NOISE,   # noqa: E402
                    GYRO_BIAS, ACC_BIAS, ENC_NOISE, CONTACT_FORCE_N)

N_LEGS = 5
# the reflex states that mean a fall. NOT RECOVER: it is the last step of EVERY stop (request_stop:
# PLANT -> BRACE -> RECOVER into the planted stance), so counting it suspended the estimate and inflated
# its uncertainty by a "fall" at the end of every goto (found in the cockpit, 2026-10-15: 43 cm 1-sigma
# after one goto and one turn)
FALL_STATES = ("FALLEN", "RIGHTED")
FALL_TILT_DEG = 35.0     # body z more than this off "up" (from the ESTIMATED attitude): not walking
MIN_STANCE = 3           # fewer feet down than this (and not a swing phase of a walk): not standing
RESUME_S = 1.0           # a sane stance this long before the position estimate runs again
V_MAX_M_S = 0.25         # an estimated speed above this is not walking (the envelope is 0.034): a fall
                         # or a filter fault — treated as a fall
FEW_FEET_S = 0.2         # fewer than MIN_STANCE feet down this long (a walk always has 4): not standing
ACC_TAU_S = 0.1          # the accelerometer low-pass behind the measured-gravity tilt
LOOKBACK_S = 0.3         # on a fall, position goes back to the estimate from this long before it was
                         # detected (the first ticks of a fall already integrate nonsense)
FALL_POS_SIGMA_M = 0.10  # after a suspension: how much the fall may have moved it (P inflation)
FALL_YAW_SIGMA_DEG = 10.0
# the EKF tuning the in-the-loop runs used (sim_lidar v2, run_odom_v2)
EKF_KW = dict(sigma_a=0.6, sigma_g=0.03, sigma_v_meas=0.04)


_STATION_C = np.cos(np.deg2rad(STATION_DEG))
_STATION_S = np.sin(np.deg2rad(STATION_DEG))


def jac_body(i, theta):
    """d fk_body(i, theta) / d theta, analytic (metres per radian): pebble_gait.leg_fk's
    jacobian, rotated into the body frame by leg i's station. The EKF's default is a
    forward difference (3 more fk calls per stance leg per tick)."""
    q1, q2, q3 = theta
    c1, s1 = math.cos(q1), math.sin(q1)
    c2, s2 = math.cos(q2), math.sin(q2)
    c23, s23 = math.cos(q2 + q3), math.sin(q2 + q3)
    r = L1 + L2 * c2 + L3 * c23
    dr2, dr3 = -L2 * s2 - L3 * s23, -L3 * s23
    dz2, dz3 = L2 * c2 + L3 * c23, L3 * c23
    J = np.array([[-r * s1, dr2 * c1, dr3 * c1],
                  [r * c1, dr2 * s1, dr3 * s1],
                  [0.0, dz2, dz3]])
    c, s = _STATION_C[i], _STATION_S[i]
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return (Rz @ J) * 1e-3                       # pebble_gait works in mm


def _quat_level(f_body, yaw):
    """(w, x, y, z) with heading yaw (rad) and the roll / pitch that put the measured specific
    force f_body (the accelerometer at rest: -gravity in the body frame) straight up."""
    g = -np.asarray(f_body, float) / max(float(np.linalg.norm(f_body)), 1e-9)   # down, body frame
    th = math.asin(max(-1.0, min(1.0, g[0])))
    ph = math.atan2(-g[1], -g[2])

    def axis(a, i):
        q = np.zeros(4)
        q[0], q[i] = math.cos(a / 2), math.sin(a / 2)
        return q
    from legged_odom import quat_mul
    q = quat_mul(quat_mul(axis(yaw, 3), axis(th, 2)), axis(ph, 1))
    return q / np.linalg.norm(q)


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def _yaw_of_q(q):
    R = quat_to_R(q)
    return math.atan2(R[1, 0], R[0, 0])


class CockpitOdometry:
    """The EKF + the simulated sensors that feed it. One per cockpit; bind() after a world rebuild."""

    def __init__(self, model, seed=0, noise=True, zupt=True, imu_hz=IMU_HZ, torso="torso"):
        self.seed, self.noise, self.zupt, self.imu_hz = seed, bool(noise), bool(zupt), float(imu_hz)
        self.torso_name = torso
        self.rng = np.random.default_rng(seed)
        # the turn-on bias of THIS unit: the scenes calibration (fixed per seed, like a real chip)
        self.gyro_bias = np.asarray(GYRO_BIAS, float) if noise else np.zeros(3)
        self.acc_bias = np.asarray(ACC_BIAS, float) if noise else np.zeros(3)
        self.bind(model)
        self.ekf = None
        self.cost_us = 0.0          # EMA of the cost of one EKF tick (predict + leg updates)
        self.reset({"x": 0.0, "y": 0.0, "yaw_deg": 0.0})

    # ---------------------------------------------------------------- setup
    def bind(self, model):
        """Model ids (torso, joints, feet). Call after the cockpit swaps its model."""
        self.model = model
        self.torso = model.body(self.torso_name).id
        self.jadr = np.array([model.joint(f"{n}{i}").qposadr[0] for i in range(N_LEGS)
                              for n in ("yaw", "hip", "knee")])
        self.vadr = np.array([model.joint(f"{n}{i}").dofadr[0] for i in range(N_LEGS)
                              for n in ("yaw", "hip", "knee")])
        self.fids = [model.geom(f"foot{i}").id for i in range(N_LEGS)]
        self.every = max(1, int(round(1.0 / (self.imu_hz * model.opt.timestep))))

    def reset(self, pose, data=None):
        """Start the estimate at `pose` ({x, y, yaw_deg}: the spawn, which the robot knows by
        definition — the map's origin is where it woke up). data: the respawned MjData, for the
        torso height only (roll / pitch start level; the IMU's gravity settles them)."""
        yaw = math.radians(float(pose.get("yaw_deg", 0.0)))
        ekf = LeggedOdomEKF(fk_body, n_legs=N_LEGS, jac_body=jac_body, **EKF_KW)
        z = float(data.xpos[self.torso][2]) if data is not None else 0.0
        ekf.p = np.array([float(pose["x"]), float(pose["y"]), z])
        ekf.q = np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
        self.ekf = ekf
        self.gate = StillnessGate()
        self._k = 0
        self._v_prev = None
        self._good = self._snapshot()
        self._past = deque(maxlen=max(1, int(round(LOOKBACK_S * self.imu_hz))))   # (p) per IMU tick
        self.suspended, self.why = False, None
        self._sane_since = None
        self.events = []                 # [(t, what)] falls, resumes, rollbacks (last 20)
        self._f_lp = None                # low-passed accelerometer (body frame)
        self._few_since = None
        self.zupts = 0
        self._dist = 0.0
        self._turned = 0.0
        self._p_last = ekf.p[:2].copy()
        self._yaw_last = yaw

    def _snapshot(self):
        e = self.ekf
        return (e.p.copy(), e.v.copy(), e.q.copy(), e.bg.copy(), e.ba.copy(), e.P.copy())

    def _restore(self, s):
        e = self.ekf
        e.p, e.v, e.q, e.bg, e.ba, e.P = (x.copy() for x in s)

    def _event(self, t, what):
        self.events.append((round(float(t), 2), what))
        del self.events[:-20]

    # ---------------------------------------------------------------- sensors
    def _imu(self, data, dt):
        """(gyro, accel) in the body frame, as the chip reports them: true rate / specific force
        + turn-on bias + white noise. Accel = the torso origin's velocity change over the IMU
        period minus gravity (an accelerometer averages over its sample window)."""
        R = data.xmat[self.torso].reshape(3, 3)
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_BODY, self.torso, vel, 0)
        w_body = R.T @ vel[0:3]
        v = vel[3:6].copy()
        a_world = np.zeros(3) if self._v_prev is None else (v - self._v_prev) / dt
        self._v_prev = v
        g = np.asarray(self.model.opt.gravity, float)
        f_body = R.T @ (a_world - g)
        if self.noise:
            w_body = w_body + self.gyro_bias + self.rng.normal(0.0, GYRO_NOISE, 3)
            f_body = f_body + self.acc_bias + self.rng.normal(0.0, ACC_NOISE, 3)
        return w_body, f_body

    def _encoders(self, data):
        q = data.qpos[self.jadr].reshape(N_LEGS, 3).copy()
        qd = data.qvel[self.vadr].reshape(N_LEGS, 3).copy()
        if self.noise:
            q += self.rng.normal(0.0, ENC_NOISE, q.shape)
            qd += self.rng.normal(0.0, QVEL_NOISE, qd.shape)
        return q, qd

    def _contacts(self, data):
        from contacts import foot_contacts
        return foot_contacts(self.model, data, self.fids, close_n=CONTACT_FORCE_N)

    def _tilt_deg(self):
        """Tilt of the ESTIMATED attitude (deg from upright)."""
        R = quat_to_R(self.ekf.q)
        return math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))

    def _grav_tilt_deg(self):
        """Tilt the accelerometer says (low-passed specific force vs body +z): it sees a fall
        the attitude estimate missed, e.g. one too fast for the gyro's band."""
        f = self._f_lp
        n = float(np.linalg.norm(f)) if f is not None else 0.0
        return 0.0 if n < 1e-6 else math.degrees(math.acos(max(-1.0, min(1.0, f[2] / n))))

    # ---------------------------------------------------------------- the tick
    def step(self, model, data, t, moving, contacts=None, state=None):
        """Every physics tick (cheap between IMU samples). moving: commanded motion of any kind;
        contacts: bool[5] foot switches (None = the 1.5 N force switch); state: the reflex
        state (FALLEN / RIGHTED suspend the position estimate; RECOVER is an ordinary stop)."""
        if model is not self.model:
            self.bind(model)
        self._k += 1
        if self._k % self.every:
            return
        t0 = time.perf_counter()
        dt = self.every * model.opt.timestep
        e = self.ekf
        gyro, acc = self._imu(data, dt)
        con = np.asarray(self._contacts(data) if contacts is None else contacts, bool)
        a_lp = min(1.0, dt / ACC_TAU_S)
        self._f_lp = acc.copy() if self._f_lp is None else self._f_lp + a_lp * (acc - self._f_lp)
        e.predict(gyro, acc, dt)
        tilt = max(self._tilt_deg(), self._grav_tilt_deg())
        speed = float(np.linalg.norm(e.v))
        few = int(con.sum()) < MIN_STANCE
        self._few_since = (t if self._few_since is None else self._few_since) if few else None
        no_feet = few and t - self._few_since >= FEW_FEET_S
        fallen = (state in FALL_STATES) or tilt > FALL_TILT_DEG or speed > V_MAX_M_S or no_feet
        if fallen and not self.suspended:
            self.suspended = True
            self.why = (f"reflex {state}" if state in FALL_STATES else f"tilted {tilt:.0f} deg"
                        if tilt > FALL_TILT_DEG else f"estimated speed {speed:.2f} m/s"
                        if speed > V_MAX_M_S else f"{int(con.sum())} feet down")
            self._event(t, f"suspended: {self.why}")
            self._p_hold = (self._past[0] if self._past else self._good[0]).copy()
            e.p = self._p_hold.copy()
            self._sane_since = None
        if self.suspended:
            # attitude keeps integrating (gravity re-levels it below); position held, velocity zero
            e.p = self._p_hold.copy()
            e.v = np.zeros(3)
            sane = not fallen and int(con.sum()) >= MIN_STANCE
            if sane:
                self._sane_since = t if self._sane_since is None else self._sane_since
                if t - self._sane_since >= RESUME_S:
                    self.suspended, self.why = False, None
                    # re-level from the accelerometer (standing: it reads -gravity), keep the heading
                    e.q = _quat_level(self._f_lp, _yaw_of_q(e.q))
                    e.P[6:8, 6:8] = np.eye(2) * math.radians(2.0) ** 2
                    e.P[0:2, 0:2] += np.eye(2) * FALL_POS_SIGMA_M ** 2
                    e.P[8, 8] += math.radians(FALL_YAW_SIGMA_DEG) ** 2
                    self._event(t, "resumed (position and heading uncertain by the fall)")
            else:
                self._sane_since = None
        else:
            q, qd = self._encoders(data)
            e.update_legs(q, qd, con)
            if self.zupt and self.gate.update(t, bool(moving), bool(con.all())):
                if e.update_yaw_rate_zero():
                    self.zupts += 1
        if not (np.isfinite(e.p).all() and np.isfinite(e.q).all() and np.isfinite(e.P).all()):
            self._restore(self._good)
            e.v = np.zeros(3)
            self._event(t, "non-finite state: rolled back to the last good estimate")
        elif not self.suspended:
            self._good = self._snapshot()
            self._past.append(e.p.copy())
        # the odometer: path length and net turn of the ESTIMATE
        xy = e.p[:2]
        step = float(np.hypot(*(xy - self._p_last)))
        if moving and step < 0.05:                         # standing jitter and a re-seed jump are not travel
            self._dist += step
        self._p_last = xy.copy()
        yaw = _yaw_of_q(e.q)
        self._turned += math.degrees((yaw - self._yaw_last + math.pi) % (2 * math.pi) - math.pi)
        self._yaw_last = yaw
        us = (time.perf_counter() - t0) * 1e6
        self.cost_us = us if self.cost_us == 0.0 else 0.98 * self.cost_us + 0.02 * us

    # ---------------------------------------------------------------- read-outs
    def estimate(self):
        """{x, y, yaw_deg}: CockpitSim.pose()'s frame and rounding."""
        e = self.ekf
        return {"x": round(float(e.p[0]), 3), "y": round(float(e.p[1]), 3),
                "yaw_deg": round(math.degrees(_yaw_of_q(e.q)), 1)}

    def drift(self, truth):
        """Estimate minus truth: {pos_m, yaw_deg (wrapped, + = the estimate is left of truth)}."""
        est = self.estimate()
        return {"pos_m": round(math.hypot(est["x"] - truth["x"], est["y"] - truth["y"]), 3),
                "yaw_deg": round(_wrap_deg(est["yaw_deg"] - truth["yaw_deg"]), 1)}

    def odometer(self):
        """Since reset: the estimated path length (m) and the signed net turn (deg, + = left)."""
        return {"dist_m": round(self._dist, 3), "turned_deg": round(self._turned, 1)}

    def sigma(self):
        """1-sigma position (m) and heading (deg) uncertainty the filter believes."""
        P = self.ekf.P
        return {"pos_m": round(math.sqrt(max(P[0, 0] + P[1, 1], 0.0)), 3),
                "yaw_deg": round(math.degrees(math.sqrt(max(P[8, 8], 0.0))), 1)}

    def status(self, truth=None):
        out = {"estimate": self.estimate(), "odometer": self.odometer(), "sigma": self.sigma(),
               "suspended": self.suspended, "why": self.why, "events": list(self.events[-5:]),
               "zupts": self.zupts, "gyro_bias_z_err": round(float(self.ekf.bg[2] - self.gyro_bias[2]), 5),
               "cost_us": round(self.cost_us, 1), "noise": self.noise}
        if truth is not None:
            out["drift"] = self.drift(truth)
        return out
