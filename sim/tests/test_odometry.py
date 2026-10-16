"""sim/odometry.py: the robot's own pose estimate (legged-odometry EKF on simulated
hardware sensors), and perception/legged_odom.py's faster leg update.

Pinned: the sliced Joseph update is the dense one to rounding; the analytic leg
jacobian is the forward-difference one; the estimate starts at the spawn pose,
follows a short walk (tolerance measured: 22 mm after a 0.32 m walk, seed 0),
reset() re-seeds it; a fall suspends it without a NaN and holds the position."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
for p in ("sim", "gait", "perception"):
    sys.path.insert(0, os.path.join(ROOT, p))

import legged_odom as lo                                    # noqa: E402
import odometry as od                                       # noqa: E402
from scenes import fk_body                                  # noqa: E402


def _dense_update_legs(e, thetas, theta_dots, contacts):
    """The pre-2026-10-15 update_legs, verbatim math (dense H, np.cross, Joseph form)."""
    for i in range(e.n_legs):
        if not contacts[i]:
            continue
        p0 = e.fk(i, thetas[i])
        J = np.zeros((3, 3))
        for k in range(3):
            tp = np.array(thetas[i], dtype=float)
            tp[k] += 1e-6
            J[:, k] = (e.fk(i, tp) - p0) / 1e-6
        v_b = -(J @ theta_dots[i] + np.cross(e.last_omega, p0))
        R = lo.quat_to_R(e.q)
        r = R @ v_b - e.v
        H = np.zeros((3, 15))
        H[:, 3:6] = np.eye(3)
        H[:, 6:9] = R @ lo.skew(v_b)
        S = H @ e.P @ H.T + e.Rv
        if r @ np.linalg.solve(S, r) > 16.27:
            continue
        K = e.P @ H.T @ np.linalg.inv(S)
        e._inject(K @ r)
        IKH = np.eye(15) - K @ H
        e.P = IKH @ e.P @ IKH.T + K @ e.Rv @ K.T


def _filter(rng):
    e = lo.LeggedOdomEKF(fk_body, sigma_a=0.6, sigma_g=0.03, sigma_v_meas=0.04)
    A = rng.normal(0, 0.01, (15, 15))
    e.P = A @ A.T + np.eye(15) * 1e-4
    e.v = rng.normal(0, 0.02, 3)
    e.last_omega = rng.normal(0, 0.1, 3)
    return e


def test_the_sliced_leg_update_is_the_dense_one():
    rng = np.random.default_rng(4)
    for _ in range(5):
        e1 = _filter(rng)
        e2 = lo.LeggedOdomEKF(fk_body, sigma_a=0.6, sigma_g=0.03, sigma_v_meas=0.04)
        e2.P, e2.v, e2.last_omega = e1.P.copy(), e1.v.copy(), e1.last_omega.copy()
        th = rng.uniform([-0.5, 0.0, -1.8], [0.5, 0.9, -0.6], (5, 3))
        thd = rng.normal(0, 0.3, (5, 3))
        con = rng.random(5) > 0.3
        e1.update_legs(th, thd, con)
        _dense_update_legs(e2, th, thd, con)
        for a, b in ((e1.p, e2.p), (e1.v, e2.v), (e1.q, e2.q), (e1.P, e2.P)):
            assert np.allclose(a, b, atol=1e-7, rtol=1e-5)


def test_the_analytic_jacobian_is_the_numeric_one():
    rng = np.random.default_rng(0)
    for i in range(5):
        th = rng.uniform([-0.6, -0.3, -2.0], [0.6, 1.0, -0.4])
        p0 = fk_body(i, th)
        Jn = np.column_stack([(fk_body(i, th + 1e-6 * np.eye(3)[k]) - p0) / 1e-6 for k in range(3)])
        assert np.allclose(od.jac_body(i, th), Jn, atol=1e-5)


@pytest.fixture
def pg():
    from playground import Playground
    p = Playground()
    p.do("righter off")
    return p


def _truth(p):
    x, y = p.data.xpos[p.torso][:2]
    R = p.data.xmat[p.torso].reshape(3, 3)
    return {"x": float(x), "y": float(y), "yaw_deg": math.degrees(math.atan2(R[1, 0], R[0, 0]))}


def _run(p, odo, secs, v):
    p.cmd_v[:] = v
    for _ in range(int(secs / p.DT)):
        p.step()
        odo.step(p.model, p.data, p.t, bool(np.any(p.cmd_v)) or p.sup.state != "NORMAL",
                 p.last["con"], p.sup.state)


def test_it_starts_at_the_spawn_follows_a_walk_and_resets(pg):
    odo = od.CockpitOdometry(pg.model, seed=0)
    odo.reset({"x": 0.0, "y": 0.0, "yaw_deg": 0.0}, pg.data)
    assert odo.estimate() == {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    _run(pg, odo, 1.5, (0, 0, 0))
    _run(pg, odo, 6.0, (34.0, 0, 0))
    _run(pg, odo, 1.0, (0, 0, 0))
    truth, est = _truth(pg), odo.estimate()
    assert truth["x"] > 0.15                                          # it walked
    d = odo.drift(truth)
    assert d["pos_m"] < 0.04 and abs(d["yaw_deg"]) < 2.0, (truth, est, d)
    assert 0.6 * truth["x"] < odo.odometer()["dist_m"] < 1.4 * truth["x"]
    assert not odo.suspended and all(np.isfinite(list(est.values())))
    assert 0 < odo.cost_us < 20000
    odo.reset({"x": 1.0, "y": -2.0, "yaw_deg": 90.0}, pg.data)
    assert odo.estimate() == {"x": 1.0, "y": -2.0, "yaw_deg": 90.0}
    assert odo.odometer() == {"dist_m": 0.0, "turned_deg": 0.0} and odo.events == []


def test_a_fall_suspends_the_estimate_without_blowing_it_up(pg):
    odo = od.CockpitOdometry(pg.model, seed=1)
    odo.reset({"x": 0.0, "y": 0.0, "yaw_deg": 0.0}, pg.data)
    _run(pg, odo, 1.5, (34.0, 0, 0))
    held = odo.estimate()
    # flip it onto its back, mid-walk: the supervisor goes FALLEN, the IMU sees the roll
    pg.data.qpos[3:7] = [0.0, 1.0, 0.0, 0.0]
    pg.data.qpos[2] += 0.15
    _run(pg, odo, 3.0, (0, 0, 0))
    st = odo.status(_truth(pg))
    assert odo.suspended and any("suspended" in e for _t, e in odo.events), st
    est = odo.estimate()
    assert all(np.isfinite(list(est.values())))
    assert math.hypot(est["x"] - held["x"], est["y"] - held["y"]) < 0.02    # held, never re-seeded from truth
    assert np.isfinite(odo.ekf.P).all()
