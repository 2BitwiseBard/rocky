"""Simulated IMU (D052): what the BNO085 on the robot would report, from MuJoCo.

Before this every consumer computed the IMU inline, and all of them used
the WORLD z axis for "down" (R.T @ [0, 0, -1]). That is wrong the moment
gravity is tilted (the RL envs' uneven-floor stand-in) — an accelerometer
measures the real gravity vector, so on a 3 degree slope a level-looking
body reads 3 degrees of tilt. Here "down" is model.opt.gravity, always.

Pure numpy + mujoco (no gait / params imports): the RL envs, the righter
adapter and the playground all import it.

    imu = SimIMU(model)                               # ideal: no noise, no lag
    imu = SimIMU(model, grav_sigma=0.02, gyro_sigma=0.02, gyro_bias=0.01,
                 latency_s=0.01, rng=np.random.default_rng(0))
    r = imu.read(data, t)          # dict(grav=(3,), gyro=(3,), tilt=rad)

    grav_body(model, data, torso)  # the ideal quantities, stateless
    gyro_body(model, data, torso)
    tilt_rad(model, data, torso)

grav is the unit gravity direction in the body frame ([0, 0, -1] upright),
gyro the body-frame angular rate (rad/s), tilt the angle between the body
z axis and "up" (0 upright, pi on the back) — computed FROM the noisy grav
reading, as the robot would, so tilt and grav never disagree.
"""
from __future__ import annotations

import numpy as np


def gravity_dir(model) -> np.ndarray:
    """Unit world-frame gravity direction (falls back to -z if gravity is off)."""
    g = np.asarray(model.opt.gravity, dtype=float)
    n = float(np.linalg.norm(g))
    return g / n if n > 1e-9 else np.array([0.0, 0.0, -1.0])


def grav_body(model, data, body) -> np.ndarray:
    """Gravity direction in the body frame: [0, 0, -1] when upright."""
    R = data.xmat[body].reshape(3, 3)
    return R.T @ gravity_dir(model)


def gyro_body(model, data, body) -> np.ndarray:
    """Body-frame angular velocity (rad/s). cvel[:3] is the world-frame rate."""
    R = data.xmat[body].reshape(3, 3)
    return R.T @ data.cvel[body][0:3]


def tilt_from_grav(g_body) -> float:
    """Angle (rad) between body z and 'up', from a body-frame gravity direction."""
    g = np.asarray(g_body, dtype=float)
    n = float(np.linalg.norm(g))
    if n < 1e-9:
        return 0.0
    return float(np.arccos(np.clip(-g[2] / n, -1.0, 1.0)))


def tilt_rad(model, data, body) -> float:
    return tilt_from_grav(grav_body(model, data, body))


class SimIMU:
    """Noisy, biased, delayed IMU. Every option defaults to ideal.

    grav_sigma  per-axis white noise on the gravity direction (then renormalised)
    gyro_sigma  per-axis white noise on the rate (rad/s)
    gyro_bias   constant per-axis rate bias: a float b draws U(-b, b) per axis at
                reset (a new bias per episode, like a new power-up), or a (3,)
                array used as given
    latency_s   a reading becomes visible latency_s after the state it measures
    """

    def __init__(self, model, body="torso", grav_sigma=0.0, gyro_sigma=0.0,
                 gyro_bias=0.0, latency_s=0.0, rng=None):
        self.model = model
        self.body = model.body(body).id if isinstance(body, str) else int(body)
        self.grav_sigma = float(grav_sigma)
        self.gyro_sigma = float(gyro_sigma)
        self.gyro_bias_spec = gyro_bias
        self.latency_s = float(latency_s)
        self.rng = rng if rng is not None else np.random.default_rng()
        self.reset()

    def reset(self, rng=None, **kw):
        """New episode: optionally new noise settings (same keys as __init__), a
        fresh bias draw and an empty latency pipeline."""
        if rng is not None:
            self.rng = rng
        for k in ("grav_sigma", "gyro_sigma", "latency_s"):
            if k in kw:
                setattr(self, k, float(kw[k]))
        if "gyro_bias" in kw:
            self.gyro_bias_spec = kw["gyro_bias"]
        b = self.gyro_bias_spec
        if np.ndim(b) == 0:
            b = float(b)
            self.bias = self.rng.uniform(-b, b, 3) if b > 0 else np.zeros(3)
        else:
            self.bias = np.asarray(b, dtype=float).copy()
        self._buf = []                     # [(t, reading)] oldest first

    def measure(self, data) -> dict:
        """One reading of the current state with noise and bias, no latency."""
        g = grav_body(self.model, data, self.body)
        if self.grav_sigma > 0:
            g = g + self.rng.normal(0.0, self.grav_sigma, 3)
            g = g / max(float(np.linalg.norm(g)), 1e-9)
        w = gyro_body(self.model, data, self.body) + self.bias
        if self.gyro_sigma > 0:
            w = w + self.rng.normal(0.0, self.gyro_sigma, 3)
        return dict(grav=g, gyro=w, tilt=tilt_from_grav(g))

    def read(self, data, t=None) -> dict:
        """The reading the robot has at time t (s). Without latency (or without
        t) this is measure(data). With latency, the newest sample at least
        latency_s old (the oldest one while the pipeline fills)."""
        r = self.measure(data)
        if self.latency_s <= 0.0 or t is None:
            return r
        self._buf.append((float(t), r))
        # the head is the answer: drop it once the NEXT sample is old enough
        while len(self._buf) > 1 and self._buf[1][0] <= t - self.latency_s + 1e-12:
            self._buf.pop(0)
        return self._buf[0][1]
