"""PebbleEnv — gymnasium wrapper around the Pebble sim (RL groundwork, plan §5.3).

Residual-policy formulation (the sane on-ramp for sim-to-real):
  action (15,) in [-1,1]  ->  joint-target OFFSETS (max +/-0.25 rad) ADDED to
  the analytic wave-gait targets. Policy zero == the proven blind gait, so
  RL starts from a walking robot and learns corrections, not locomotion
  from scratch. The same trick deploys on hardware: the gait engine runs
  anyway; the policy output is a bounded trim.

obs (41,): gravity vector in body frame (3), body gyro (3), joint pos (15),
joint vel (15), gait phase (sin, cos), command (vx, vy, wz normalized).
Control at 50 Hz (10 physics substeps); episode 8 s.

reward = velocity tracking (xy + yaw rate) - action cost - tilt - height

Domain randomization (v1.1, per-EPISODE draws from stored base values —
the v1 friction randomizer multiplied the live model every reset, a
compounding random walk instead of a draw):
  friction x[0.7,1.4] | body masses x[0.85,1.15] | actuator torque
  x[0.85,1.15] | gravity tilted <=3 deg (uneven-floor stand-in) | action
  latency 0-1 control steps | random torso pushes (push_prob per step).
cmd_sample=True re-draws the command every episode (30% pure forward,
else vx[15,60] vy[-25,25] wz[-0.35,0.35]) -> ONE policy for the whole
command envelope; the command is already in obs.
"""
from __future__ import annotations
import os
import sys

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:                                    # keep import-safe
    gym = None

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS       # noqa: E402

CTRL_DT = 0.02              # 50 Hz policy rate
EP_SECONDS = 8.0
ACT_SCALE = 0.25            # rad of residual authority per joint


class PebbleEnv(gym.Env if gym else object):
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, cmd=(45.0, 0.0, 0.0), randomize=False, push_prob=0.0,
                 cmd_sample=False, render_mode=None, seed=None):
        self.model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
        self.data = mujoco.MjData(self.model)
        self.gait = WaveGait()
        self.cmd = np.array(cmd, dtype=np.float64)      # mm/s, mm/s, rad/s
        self.randomize = randomize
        self.push_prob = push_prob
        self.cmd_sample = cmd_sample
        self.render_mode = render_mode
        self._renderer = None
        self.rng = np.random.default_rng(seed)
        self.torso = self.model.body("torso").id
        self.substeps = int(round(CTRL_DT / self.model.opt.timestep))
        self.max_steps = int(EP_SECONDS / CTRL_DT)
        # DR base values: randomization DRAWS from these each episode
        # instead of mutating the live model cumulatively
        self._base_friction = self.model.geom_friction[:, 0].copy()
        self._base_mass = self.model.body_mass.copy()
        self._base_inertia = self.model.body_inertia.copy()
        self._base_gain = self.model.actuator_gainprm.copy()
        self._base_frange = self.model.actuator_forcerange.copy()
        self._base_gravity = self.model.opt.gravity.copy()
        self._act_delay = 0
        self._prev_action = np.zeros(15)
        # joint addresses (the BUILD_LOG qpos-interleave gotcha)
        self._jadr = [self.model.joint(f"{n}{i}").qposadr[0]
                      for i in range(5) for n in ("yaw", "hip", "knee")]
        self._vadr = [self.model.joint(f"{n}{i}").dofadr[0]
                      for i in range(5) for n in ("yaw", "hip", "knee")]
        if gym:
            self.observation_space = spaces.Box(-np.inf, np.inf, (41,),
                                                dtype=np.float32)
            self.action_space = spaces.Box(-1.0, 1.0, (15,), dtype=np.float32)
        self._q0 = None
        self._t = 0.0
        self._step_n = 0

    # ------------------------------------------------------------------
    def _obs(self):
        d = self.data
        R = d.xmat[self.torso].reshape(3, 3)
        grav_body = R.T @ np.array([0, 0, -1.0])
        w_body = R.T @ d.cvel[self.torso][0:3]
        qpos = d.qpos[self._jadr]
        qvel = d.qvel[self._vadr]
        ph = 2 * np.pi * ((self._t / self.gait.T) % 1.0)
        cmd_n = self.cmd / np.array([60.0, 60.0, 0.6])
        return np.concatenate([grav_body, w_body, qpos, qvel,
                               [np.sin(ph), np.cos(ph)], cmd_n]).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        if self.randomize:
            fr = self.rng.uniform(0.7, 1.4)
            self.model.geom_friction[:, 0] = np.clip(self._base_friction * fr,
                                                     0.3, 2.2)
            m = self.rng.uniform(0.85, 1.15)
            self.model.body_mass[:] = self._base_mass * m
            self.model.body_inertia[:] = self._base_inertia * m
            tq = self.rng.uniform(0.85, 1.15)
            self.model.actuator_forcerange[:] = self._base_frange * tq
            # uneven-floor stand-in: tilt gravity up to 3 degrees
            ang = self.rng.uniform(0, np.deg2rad(3.0))
            az = self.rng.uniform(0, 2 * np.pi)
            g = 9.81 * np.array([np.sin(ang) * np.cos(az),
                                 np.sin(ang) * np.sin(az), -np.cos(ang)])
            self.model.opt.gravity[:] = g
            self._act_delay = int(self.rng.random() < 0.5)
        else:
            self.model.geom_friction[:, 0] = self._base_friction
            self.model.body_mass[:] = self._base_mass
            self.model.body_inertia[:] = self._base_inertia
            self.model.actuator_forcerange[:] = self._base_frange
            self.model.opt.gravity[:] = self._base_gravity
            self._act_delay = 0
        self._prev_action = np.zeros(15)
        if self.cmd_sample:
            if self.rng.random() < 0.30:
                self.cmd = np.array([45.0, 0.0, 0.0])
            else:
                self.cmd = np.array([self.rng.uniform(15.0, 60.0),
                                     self.rng.uniform(-25.0, 25.0),
                                     self.rng.uniform(-0.35, 0.35)])
        q0 = np.array([leg_ik(body_to_leg(i, self.gait.p_nom[i]))
                       for i in range(N_LEGS)]).flatten()
        self.data.qpos[0:3] = [0, 0, (self.gait.h + 14) / 1000.0]
        self.data.qpos[3:7] = [1, 0, 0, 0]
        self.data.qpos[self._jadr] = q0          # by address (interleave gotcha)
        self.data.ctrl[:15] = q0
        mujoco.mj_forward(self.model, self.data)
        self._q0 = q0
        self._t = 0.0
        self._step_n = 0
        # settle 0.4 s at stance before the clock starts
        for _ in range(int(0.4 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        return self._obs(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1, 1)
        applied = self._prev_action if self._act_delay else action
        self._prev_action = action.copy()
        q_gait, _, _ = self.gait.joint_targets(self._t, *self.cmd)
        target = q_gait.flatten() + ACT_SCALE * applied
        self.data.ctrl[:15] = target
        if self.push_prob > 0 and self.rng.random() < self.push_prob:
            f = self.rng.normal(0, 12.0, 2)
            self.data.xfrc_applied[self.torso, :2] = f
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        self.data.xfrc_applied[self.torso, :3] = 0
        self._t += CTRL_DT
        self._step_n += 1

        d = self.data
        R = d.xmat[self.torso].reshape(3, 3)
        tilt = float(np.arccos(np.clip(R[2, 2], -1, 1)))
        v_world = d.cvel[self.torso][3:6] * 1000.0      # mm/s translational
        v_body = R.T @ v_world
        v_err = np.linalg.norm(v_body[:2] - self.cmd[:2])
        w_body = R.T @ d.cvel[self.torso][0:3]
        wz_err = abs(float(w_body[2]) - self.cmd[2])
        h = d.xpos[self.torso][2]
        h_err = abs(h - (self.gait.h + 14) / 1000.0)
        r_vel = np.exp(-(v_err / 40.0) ** 2)
        r_yaw = np.exp(-(wz_err / 0.4) ** 2)
        r_act = -0.02 * float(np.mean(action ** 2))
        r_tilt = -1.2 * tilt ** 2
        r_h = -8.0 * h_err
        reward = float(0.75 * r_vel + 0.25 * r_yaw + r_act + r_tilt + r_h)
        terminated = bool(tilt > np.deg2rad(45) or h < 0.05)
        if terminated:
            reward -= 5.0
        truncated = self._step_n >= self.max_steps
        info = dict(v_body=v_body[:2], tilt_deg=np.rad2deg(tilt), height=h)
        return self._obs(), reward, terminated, truncated, info

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 720)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = self.data.xpos[self.torso]
        cam.distance, cam.elevation, cam.azimuth = 0.85, -18, 145
        self._renderer.update_scene(self.data, cam)
        return self._renderer.render()


def sanity():
    """Zero-action rollout must walk (the analytic gait is the baseline)."""
    env = PebbleEnv()
    obs, _ = env.reset(seed=0)
    total, steps = 0.0, 0
    x0 = env.data.xpos[env.torso][0]
    while True:
        obs, r, term, trunc, info = env.step(np.zeros(15))
        total += r
        steps += 1
        if term or trunc:
            break
    x1 = env.data.xpos[env.torso][0]
    print(f"zero-action rollout: {steps} steps, return {total:.1f}, "
          f"walked {(x1 - x0) * 1000:.0f} mm, terminated={term}")
    assert not term, "the baseline gait fell — env wiring is wrong"
    assert (x1 - x0) > 0.20, "baseline should cover >200 mm in 8 s"
    return total


if __name__ == "__main__":
    sanity()
