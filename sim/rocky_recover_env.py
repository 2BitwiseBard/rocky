"""RecoverEnv — fall recovery / self-righting for Pebble (session 8).

The one learned behavior a pentapod actually needs that no analytic gait
provides: it is on its side or its back — legs anywhere — and must get
upright and standing. Radial symmetry helps (any side is "a side"), the
rock carapace hurts (a round-ish back is a rocking chair). No residual
here: there is no controller to trim, so the policy commands ABSOLUTE
joint targets, rate-limited to a servo-ish 5 rad/s.

obs (39,): gravity in body frame (3), body gyro (3), joint pos (15),
joint vel (15), torso height / 0.15, feet-in-contact / 5, tilt / pi.
action (15,) in [-1,1] -> joint targets mapped into the real ranges
(yaw ±40°, hip [-70, 90], knee [-150, -20]); claws held closed.

reward per 50 Hz step:
  +1.0 * uprightness   (0 upside-down .. 1 upright: (1 - g_z_body)/2)
  +0.5 * height/0.13   (torso above the floor, capped at standing)
  +3.0 when upright (tilt < 20°) AND standing (h > 0.10) AND >=4 feet down
  -0.01 * |action|²  -0.02 * |Δaction|²   (servo abuse)
  episode: 6 s; success = the standing condition held for 1.0 s (+10, end)

reset: torso dropped from 0.16 m with a random orientation drawn from
{on the back, left/right side, random tumble}, joints at random targets
inside the ranges, 0.5 s of settling so the robot lands before the clock
starts. DR (optional): friction x[0.7,1.4], masses x[0.85,1.15].

reward="v2" (session 8d, D041): the success condition IS the handoff
criterion ReflexSupervisor uses (tilt < 25°, h > 0.09 held 0.5 s) — so
the policy is trained to reach exactly the state the analytic planted
pose can take over from — plus a +0.2·feet_down/5 term while upright, so
"rights the body but props on 1.5 feet" (the v1 failure) stops paying.
Standing bonus is +2.0 (was +3.0) to keep the return scale comparable.

reward="v3" (session 9b, D048 — the anti-jitter recipe): v2's terms plus
a SMOOTHNESS cost on the target the servos actually receive,
  -0.3 * mean((Δtarget / rate_limit)²)
i.e. pinning a joint at the rate limit costs 0.3/step across the board.
The 2026-09-23 audit of recover1 measured 73 % of its per-tick joint moves
pinned at the limit and 4–23 direction reversals per second per joint: a
50 Hz staircase of 5.7° jumps, which is the "jitter" you see. v3 is meant
to be trained with a lower rate limit too (`rate_limit_rad_s`, 3 rad/s
recommended; the checkpoint records it and PolicyRighter/eval read it
back, so the deployed adapter always matches the training dynamics).

Honesty: the sim's torso is two stacked cylinders — flat-ish top and
bottom. The real Pebble has a domed rock shell and a belly with skids and
a battery door; the exact rolling behaviour WILL differ. What transfers
is the strategy class (which legs to sweep, when to push), not the timing.
Bench-day calibration (D017 re-baseline) applies here too.
"""
from __future__ import annotations
import os
import sys

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    gym = None

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import N_LEGS                                       # noqa: E402

CTRL_DT = 0.02
EP_SECONDS = 6.0
RATE_LIMIT_RAD_S = 5.0                     # default (v1/v2 checkpoints)
RATE_LIMIT = RATE_LIMIT_RAD_S * CTRL_DT    # rad per control step (default)
REWARDS = ("v1", "v2", "v3")
Q_LO = np.tile(np.deg2rad([-40.0, -70.0, -150.0]), N_LEGS)
Q_HI = np.tile(np.deg2rad([40.0, 90.0, -20.0]), N_LEGS)
STAND_H = 0.10
STAND_TILT = np.deg2rad(20.0)
HOLD_S = 1.0
# v2: the ReflexSupervisor / eval_recover HANDOFF criterion (keep in sync
# with eval_recover.run and harness.reflex FALLEN branch)
HANDOFF_TILT = np.deg2rad(25.0)
HANDOFF_H = 0.09
HANDOFF_HOLD_S = 0.5


def _quat_from_axis_angle(axis, ang):
    axis = np.asarray(axis, float)
    axis = axis / (np.linalg.norm(axis) + 1e-9)
    return np.concatenate([[np.cos(ang / 2)], axis * np.sin(ang / 2)])


class RecoverEnv(gym.Env if gym else object):
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, randomize=False, seed=None, render_mode=None, reward="v1",
                 rate_limit_rad_s=RATE_LIMIT_RAD_S, **_):
        assert reward in REWARDS, reward
        self.reward_version = reward
        self.rate_limit_rad_s = float(rate_limit_rad_s)
        self.rate_limit = self.rate_limit_rad_s * CTRL_DT      # rad per control step
        self.model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
        self.data = mujoco.MjData(self.model)
        self.randomize = randomize
        self.render_mode = render_mode
        self._renderer = None
        self.rng = np.random.default_rng(seed)
        self.torso = self.model.body("torso").id
        self.substeps = int(round(CTRL_DT / self.model.opt.timestep))
        self.max_steps = int(EP_SECONDS / CTRL_DT)
        self._base_friction = self.model.geom_friction[:, 0].copy()
        self._base_mass = self.model.body_mass.copy()
        self._base_inertia = self.model.body_inertia.copy()
        self._jadr = [self.model.joint(f"{n}{i}").qposadr[0]
                      for i in range(5) for n in ("yaw", "hip", "knee")]
        self._vadr = [self.model.joint(f"{n}{i}").dofadr[0]
                      for i in range(5) for n in ("yaw", "hip", "knee")]
        self._foot_gid = [self.model.geom(f"foot{i}").id for i in range(N_LEGS)]
        if gym:
            self.observation_space = spaces.Box(-np.inf, np.inf, (39,), dtype=np.float32)
            self.action_space = spaces.Box(-1.0, 1.0, (15,), dtype=np.float32)
        self._target = np.zeros(15)
        self._prev_action = np.zeros(15)
        self._step_n = 0
        self._hold = 0.0
        self.last_mode = ""

    # ------------------------------------------------------------------
    def _feet_down(self):
        n = 0
        for c in range(self.data.ncon):
            con = self.data.contact[c]
            if con.geom1 in self._foot_gid or con.geom2 in self._foot_gid:
                n += 1
        return min(n, N_LEGS)

    def _state(self):
        d = self.data
        R = d.xmat[self.torso].reshape(3, 3)
        grav_body = R.T @ np.array([0, 0, -1.0])
        w_body = R.T @ d.cvel[self.torso][0:3]
        tilt = float(np.arccos(np.clip(R[2, 2], -1, 1)))
        h = float(d.xpos[self.torso][2])
        return grav_body, w_body, tilt, h

    def _obs(self):
        grav_body, w_body, tilt, h = self._state()
        qpos = self.data.qpos[self._jadr]
        qvel = self.data.qvel[self._vadr]
        feet = self._feet_down()
        return np.concatenate([grav_body, w_body, qpos, qvel,
                               [h / 0.15, feet / N_LEGS, tilt / np.pi]]).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        if self.randomize:
            fr = self.rng.uniform(0.7, 1.4)
            self.model.geom_friction[:, 0] = np.clip(self._base_friction * fr, 0.3, 2.2)
            m = self.rng.uniform(0.85, 1.15)
            self.model.body_mass[:] = self._base_mass * m
            self.model.body_inertia[:] = self._base_inertia * m
        else:
            self.model.geom_friction[:, 0] = self._base_friction
            self.model.body_mass[:] = self._base_mass
            self.model.body_inertia[:] = self._base_inertia
        # random fallen pose: back / side / tumble
        mode = self.rng.choice(["back", "side", "tumble"], p=[0.4, 0.4, 0.2])
        self.last_mode = str(mode)
        if mode == "back":
            ax, ang = [1, 0, 0], np.pi + self.rng.uniform(-0.3, 0.3)
        elif mode == "side":
            az = self.rng.uniform(0, 2 * np.pi)
            ax, ang = [np.cos(az), np.sin(az), 0], np.pi / 2 + self.rng.uniform(-0.25, 0.25)
        else:
            ax = self.rng.normal(size=3)
            ang = self.rng.uniform(0.5, np.pi)
        q0 = self.rng.uniform(Q_LO, Q_HI)
        self.data.qpos[0:3] = [0, 0, 0.16]
        self.data.qpos[3:7] = _quat_from_axis_angle(ax, ang)
        self.data.qpos[self._jadr] = q0
        self.data.ctrl[:15] = q0
        self.data.ctrl[15:20] = 0.0
        self._target = q0.copy()
        self._prev_action = (2 * (q0 - Q_LO) / (Q_HI - Q_LO) - 1)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(int(0.5 / self.model.opt.timestep)):      # land
            mujoco.mj_step(self.model, self.data)
        self._step_n = 0
        self._hold = 0.0
        return self._obs(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1, 1)
        want = Q_LO + (action + 1) / 2 * (Q_HI - Q_LO)
        delta = np.clip(want - self._target, -self.rate_limit, self.rate_limit)
        self._target = self._target + delta
        self.data.ctrl[:15] = self._target
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)
        self._step_n += 1

        grav_body, w_body, tilt, h = self._state()
        feet = self._feet_down()
        upright = (1.0 - grav_body[2]) / 2.0        # g_z_body = -1 when upright
        # NOTE grav_body = R^T [0,0,-1]; upright -> grav_body[2] = -1 -> 1.0
        r = 1.0 * upright + 0.5 * min(h, 0.13) / 0.13
        if self.reward_version in ("v2", "v3"):
            # success == the handoff criterion (what the supervisor waits for)
            standing = tilt < HANDOFF_TILT and h > HANDOFF_H
            hold_s = HANDOFF_HOLD_S
            if standing:
                r += 2.0 + 0.2 * feet / N_LEGS          # feet down pays only when upright
            if self.reward_version == "v3":
                # smoothness: the servo target's per-tick move, in units of the
                # rate limit (1.0 = pinned). Bang-bang righting is what this buys off.
                r -= 0.3 * float(np.mean((delta / self.rate_limit) ** 2))
        else:
            standing = tilt < STAND_TILT and h > STAND_H and feet >= 4
            hold_s = HOLD_S
            if standing:
                r += 3.0
        if standing:
            self._hold += CTRL_DT
        else:
            self._hold = 0.0
        r -= 0.01 * float(np.mean(action ** 2))
        r -= 0.02 * float(np.mean((action - self._prev_action) ** 2))
        self._prev_action = action.copy()
        terminated = False
        if self._hold >= hold_s - 1e-9:
            r += 10.0
            terminated = True
        truncated = self._step_n >= self.max_steps
        info = dict(tilt_deg=np.rad2deg(tilt), height=h, feet=feet,
                    success=bool(terminated))
        return self._obs(), float(r), terminated, truncated, info

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 720)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = self.data.xpos[self.torso]
        cam.distance, cam.elevation, cam.azimuth = 0.9, -18, 145
        self._renderer.update_scene(self.data, cam)
        return self._renderer.render()


def sanity(episodes=3):
    """Zero-policy and random-policy baselines: does the robot ever right
    itself by accident? (The bar the learned policy has to beat.)"""
    env = RecoverEnv(seed=0)
    for name, pol in (("hold-pose", lambda o: env._prev_action),
                      ("random", lambda o: env.rng.uniform(-1, 1, 15))):
        succ, rets, tilts = 0, [], []
        for ep in range(episodes):
            obs, _ = env.reset(seed=ep)
            ret, done = 0.0, False
            while not done:
                obs, r, term, trunc, info = env.step(pol(obs))
                ret += r
                done = term or trunc
            succ += int(info["success"])
            rets.append(ret)
            tilts.append(info["tilt_deg"])
        print(f"  {name:10s}: success {succ}/{episodes}, mean return {np.mean(rets):6.1f}, "
              f"end tilt {np.mean(tilts):5.1f}°")


if __name__ == "__main__":
    sanity()
