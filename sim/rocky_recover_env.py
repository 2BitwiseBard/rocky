"""RecoverEnv — fall recovery / self-righting for Pebble (session 8; D052 contract).

The one learned behavior a pentapod actually needs that no analytic gait
provides: it is on its side or its back — legs anywhere — and must get
upright and standing. Radial symmetry helps (any side is "a side"), the
rock carapace hurts (a round-ish back is a rocking chair). No residual
here: there is no controller to trim, so the policy commands ABSOLUTE
joint targets.

D052 — the sim stops flattering the servo. The action path is now the
robot's, end to end, and every stage is in the loop the policy trains on:

  a (15,) in [-1,1]
    -> EMA filter, alpha 0.4 at 50 Hz (B30 jitter: a first-order filter the
       policy KNOWS about — its state is in the obs — instead of a smoother
       bolted on after training, which would be a different controller)
    -> mapped into the params joint range (yaw +-40, hip -70..90, knee -150..-20)
    -> command clamp, rate_limit_rad_s (default 4.0 = params 'free' speed for
       NEW runs; v1-v3 checkpoints trained at 5.0, above the servo's 4.7
       no-load — flagged 'exceeds servo' wherever they are listed)
    -> ServoModel per physics substep: 50 Hz hold, bus latency, slew at
       rate_rad_s (the load derate is off since D052 V2: the MJCF damping
       carries the torque-speed line), 4096-count goals
    -> + per-joint calibration offset (DR)  -> the MuJoCo position actuator
       (D052 damping 0.62 N.m.s/rad, forcerange 1.9 N.m x voltage)

obs v2 (57, rl_common.RecoverObs) — only what the Pi can read:
  gravity dir (3) + gyro (3) from sim_imu (noise, per-episode bias);
  q (15) quantised encoder counts, one tick late; qd (15) = finite
  difference of that q at 50 Hz, one tick late; the five foot switches
  (floor only, 2.0/1.0 N hysteresis, dropout, a stuck bit); tilt/pi (1);
  the filtered action a_prev (15). NO torso height (the robot has no way
  to measure it) and no MuJoCo qvel. obs v1 (39) is kept only so the old
  checkpoints replay (obs_version=1): true state, h/0.15, a contact-PAIR
  count that included self-contacts.

reward per 50 Hz step (h is privileged — fine in a reward, never in obs):
  +1.0 * uprightness (0 upside-down .. 1 upright, gravity-relative)
  +0.5 * min(h, 0.13)/0.13
  v1: +3.0 when tilt < 20, h > 0.10, >= 4 switches closed; success held 1 s
  v2/v3: +2.0 + 0.2*feet/5 when handoff_ok() — success = handoff_ok held
      0.5 s. D052: the criterion is HARDWARE-COMPUTABLE (tilt, switches,
      joint angles), so the supervisor on the Pi can use the very function
      the policy was trained to satisfy (see handoff_ok below)
  v3: - 0.1 * mean((dtarget / rate_limit)^2)  (pinned-at-the-clamp cost; was
      0.3 in D048 — with the EMA + servo in the loop it is a backstop)
  -0.01 * mean(a^2)  -0.2 * mean((a - a_last)^2) on the RAW action (was 0.02:
      the filter must not hide a chattering policy from the cost)
  success: +10 and the episode ends

reset: torso dropped from 0.16 m in a random orientation {back, side,
tumble}, joints at random targets inside the range, 0.5 s to land; then
the servos adopt the pose they landed in (target = the encoder reading —
what the righter does on the robot when it takes over). Per-episode DR
(rl_common.DomainRandomizer) and servo draw (rl_common.servo_params).

Honesty: the sim's torso is two stacked cylinders — flat-ish top and
bottom. The real Pebble has a domed rock shell and a belly with skids and
a battery door; the exact rolling behaviour WILL differ. What transfers
is the strategy class (which legs to sweep, when to push), not the timing.
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
try:                                   # handoff_ok must import on the robot (no mujoco there)
    import mujoco
except ImportError:
    mujoco = None

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, HERE)
from pebble_gait import N_LEGS, L2, L3, Z_HIP                        # noqa: E402
import rl_common as rc                                               # noqa: E402

CTRL_DT = rc.CTRL_DT
EP_SECONDS = 6.0
RATE_LIMIT_RAD_S = rc.LEGACY_RATE_LIMIT_RAD_S   # 5.0: the v1-v3 checkpoints' clamp — replay fallback ONLY
SERVO_SAFE_RAD_S = rc.SERVO_SAFE_RAD_S          # 4.0: the default for new runs
RATE_LIMIT = RATE_LIMIT_RAD_S * CTRL_DT         # rad per control step (legacy default)
REWARDS = ("v1", "v2", "v3")
Q_LO, Q_HI = rc.joint_limits()                  # params joints.pos_deg (D052), was hand-typed
STAND_H = 0.10
STAND_TILT = np.deg2rad(20.0)
HOLD_S = 1.0
# The HANDOFF criterion (ReflexSupervisor FALLEN -> RIGHTED, eval_recover's
# hybrid). D052: tilt + >= HANDOFF_FEET switches + the kinematic torso height
# from the joint angles — everything the robot can compute. HANDOFF_H is kept
# for the privileged (sim) variant the supervisor still uses today.
HANDOFF_TILT = np.deg2rad(25.0)
HANDOFF_TILT_DEG = 25.0
HANDOFF_H = 0.09
HANDOFF_HOLD_S = 0.5
HANDOFF_FEET = 3
REWARD_WEIGHTS = dict(upright=1.0, height=0.5, stand_v1=3.0, stand=2.0, feet=0.2, success=10.0,
                      act=0.01, dact=0.2, v3_pinned=0.1)


def kinematic_height_m(hip_knee_q, feet_contacts=None):
    """Torso deck height above the ground (m) implied by the joint angles of the
    legs whose switches are closed (all legs if feet_contacts is None): the
    median of -z_foot, z_foot = Z_HIP + L2 sin(hip) + L3 sin(hip + knee) in the
    body frame. Level-body assumption — at the 25 deg handoff tilt the error
    is < 10 %. hip_knee_q: (5, 2) [hip, knee], or (5, 3) / (15,) leg-major
    [yaw, hip, knee]. Returns 0.0 when no foot is down."""
    q = np.asarray(hip_knee_q, dtype=float)
    if q.size == 3 * N_LEGS:
        q = q.reshape(N_LEGS, 3)[:, 1:]
    else:
        q = q.reshape(N_LEGS, 2)
    z = Z_HIP + L2 * np.sin(q[:, 0]) + L3 * np.sin(q[:, 0] + q[:, 1])      # mm
    sel = np.ones(N_LEGS, bool) if feet_contacts is None else np.asarray(feet_contacts, bool)
    if not sel.any():
        return 0.0
    return float(np.median(-z[sel])) / 1000.0


def handoff_ok(tilt_deg, feet_contacts, hip_knee_q):
    """The handoff criterion, instantaneous (the caller holds it HANDOFF_HOLD_S):
    tilt < 25 deg AND >= 3 foot switches closed AND the closed legs' joint
    angles put the deck >= HANDOFF_H (90 mm) above them — i.e. the robot is
    upright, standing on its feet, and those feet are under it, not folded.
    Pure numpy on purpose: pebble_reflex (the Pi) can import it."""
    feet = np.asarray(feet_contacts, dtype=bool)
    if not (float(tilt_deg) < HANDOFF_TILT_DEG and int(feet.sum()) >= HANDOFF_FEET):
        return False
    return kinematic_height_m(hip_knee_q, feet) >= HANDOFF_H


def _quat_from_axis_angle(axis, ang):
    axis = np.asarray(axis, float)
    axis = axis / (np.linalg.norm(axis) + 1e-9)
    return np.concatenate([[np.cos(ang / 2)], axis * np.sin(ang / 2)])


def action_to_q(a):
    return Q_LO + (np.asarray(a, float) + 1) / 2 * (Q_HI - Q_LO)


def q_to_action(q):
    return np.clip(2 * (np.asarray(q, float) - Q_LO) / (Q_HI - Q_LO) - 1, -1, 1)


class RecoverEnv(gym.Env if gym else object):
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, randomize=False, seed=None, render_mode=None, reward="v1",
                 rate_limit_rad_s=SERVO_SAFE_RAD_S, servo="random", ema_alpha=rc.EMA_ALPHA,
                 obs_version=rc.OBS_VERSIONS["recover"], obs_noise=None,
                 ep_seconds=EP_SECONDS, **_):
        """servo: off | nominal | random (see rl_common.servo_params).
        obs_noise: None = follow `randomize`. ema_alpha 1.0 = no filter.
        obs_version 1 + ema_alpha 1.0 + servo 'off' + rate 5.0 = the pre-D052 env."""
        assert reward in REWARDS, reward
        self.reward_version = reward
        self.rate_limit_rad_s = float(rate_limit_rad_s)
        self.rate_limit = self.rate_limit_rad_s * CTRL_DT      # rad per control step
        self.ema_alpha = float(ema_alpha)
        self.servo_mode = servo
        rc.servo_params(servo)                                  # validates the mode
        self.model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
        self.data = mujoco.MjData(self.model)
        self._note = rc.env_note(self.model)                    # fingerprint BEFORE any DR
        self.randomize = randomize
        self.obs_noise = randomize if obs_noise is None else bool(obs_noise)
        self.render_mode = render_mode
        self._renderer = None
        self.rng = np.random.default_rng(seed)
        self.torso = self.model.body("torso").id
        self.substeps = int(round(CTRL_DT / self.model.opt.timestep))
        self.ep_seconds = float(ep_seconds)
        self.max_steps = int(round(self.ep_seconds / CTRL_DT))
        self._jadr, self._vadr = rc.joint_addrs(self.model)
        self._foot_gid = rc.foot_geoms(self.model)
        self.obs_builder = rc.make_recover_obs(obs_version, self.model, self.torso,
                                               self._jadr, self._vadr, self._foot_gid)
        self.obs_version = self.obs_builder.version
        self.dr = rc.DomainRandomizer(self.model, self.torso)
        self.servo = rc.make_servo()
        self.servo_draw = rc.servo_params("off")
        self.dr_draw = self.dr.nominal()
        self._q_offset = np.zeros(15)
        if gym:
            self.observation_space = spaces.Box(-np.inf, np.inf, (self.obs_builder.dim,), dtype=np.float32)
            self.action_space = spaces.Box(-1.0, 1.0, (15,), dtype=np.float32)
        self._target = np.zeros(15)
        self._a_f = np.zeros(15)
        self._prev_action = np.zeros(15)                        # raw action, for the delta cost
        self._feet_state = np.zeros(N_LEGS, dtype=bool)
        self._step_n = 0
        self._t = 0.0
        self._hold = 0.0
        self.last_mode = ""
        self.last_delta = np.zeros(15)

    # ------------------------------------------------------------------ contract
    def config(self):
        """Everything a checkpoint needs to replay this env (saved as env_config)."""
        key = ("recover", self.obs_version)
        return dict(env="recover", obs_version=self.obs_version, obs_dim=self.obs_builder.dim,
                    obs_names=list(rc.OBS_NAMES[key]), act_dim=15,
                    action="q = Q_LO + (ema(a)+1)/2 (Q_HI-Q_LO), rate-clamped, absolute targets",
                    q_lo=[float(x) for x in Q_LO[:3]], q_hi=[float(x) for x in Q_HI[:3]],
                    ctrl_dt=CTRL_DT, ep_seconds=self.ep_seconds,
                    rate_limit_rad_s=self.rate_limit_rad_s, ema_alpha=self.ema_alpha,
                    servo=self.servo_mode,
                    servo_ranges=dict(rc.SERVO_RANGES) if self.servo_mode == "random" else None,
                    randomize=bool(self.randomize), dr_ranges=dict(rc.DR_RANGES) if self.randomize else None,
                    obs_noise=dict(rc.OBS_NOISE) if self.obs_noise else None,
                    reward=self.reward_version, reward_rev=rc.REWARD_REV,
                    reward_weights=dict(REWARD_WEIGHTS),
                    handoff=dict(tilt_deg=HANDOFF_TILT_DEG, feet=HANDOFF_FEET, kin_h_m=HANDOFF_H,
                                 hold_s=HANDOFF_HOLD_S, fn="rocky_recover_env.handoff_ok"),
                    **self._note)

    # ------------------------------------------------------------------ state
    def _feet_down(self):
        """Obs-v1's count (contact pairs, self-contacts included). Kept for old callers."""
        return rc.legacy_feet_count(self.data, self._foot_gid)

    def true_feet(self):
        """Floor-only foot switches with hysteresis, no dropout (reward / eval truth)."""
        from contacts import foot_contacts
        return foot_contacts(self.model, self.data, self._foot_gid, self._feet_state)

    def _state(self):
        """(grav_body, w_body, tilt, h): gravity-relative (model.opt.gravity), true."""
        from sim_imu import grav_body, gyro_body, tilt_from_grav
        g = grav_body(self.model, self.data, self.torso)
        w = gyro_body(self.model, self.data, self.torso)
        return g, w, tilt_from_grav(g), float(self.data.xpos[self.torso][2])

    def true_q(self):
        return self.data.qpos[self._jadr].copy()

    # ------------------------------------------------------------------ physics
    def _substeps(self):
        """One control tick of physics: the servo model per substep (the forces are
        passed; ServoModel ignores them unless load_derate is on), then the offset."""
        d = self.data
        for _ in range(self.substeps):
            tgt = self.servo.filter(self._target, self.model.opt.timestep, force=d.actuator_force[:15])
            d.ctrl[:15] = tgt + self._q_offset
            mujoco.mj_step(self.model, d)
        self._t += CTRL_DT

    def hold_step(self, q_target):
        """Drive an ABSOLUTE target (15,) through the same servo path for one
        tick, bypassing the policy filter/clamp — the eval's analytic ramp."""
        self._target = np.asarray(q_target, float).copy()
        self._substeps()

    def _obs(self):
        return self.obs_builder(self.data, self._t, self._a_f)

    # ------------------------------------------------------------------ gym API
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.dr_draw = self.dr.draw(self.rng) if self.randomize else self.dr.nominal()
        self.servo_draw = rc.servo_params(self.servo_mode, self.rng)
        self.dr.apply(self.dr_draw, voltage=self.servo_draw["voltage"])
        rc.refresh_constants(self.model, self.data)
        mujoco.mj_resetData(self.model, self.data)
        rc.apply_servo_params(self.servo, self.servo_draw)
        self._q_offset = self.dr_draw["q_offset"].copy()
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
        self.data.ctrl[:15] = q0 + self._q_offset
        self.data.ctrl[15:20] = 0.0
        mujoco.mj_forward(self.model, self.data)
        for _ in range(int(0.5 / self.model.opt.timestep)):      # land
            mujoco.mj_step(self.model, self.data)
        self.obs_builder.reset(rng=self.rng, noise=self.obs_noise, q_offset=self._q_offset)
        if self.obs_version == 1:
            # the pre-D052 reset exactly: the servos keep the drop pose target
            self._target = q0.copy()
            self._a_f = q_to_action(q0)
        else:
            # D052: the servos adopt the pose they landed in (what the righter
            # does on the robot: read present positions, start from there)
            self._target = np.clip(self.obs_builder.encoder(self.data), Q_LO, Q_HI)
            self._a_f = q_to_action(self._target)
        self.servo.reset(self._target)
        self.data.ctrl[:15] = self._target + self._q_offset
        self._prev_action = self._a_f.copy()
        self._feet_state[:] = False
        self.true_feet()
        self._step_n = 0
        self._t = 0.0
        self._hold = 0.0
        return self._obs(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1, 1)
        self._a_f = self.ema_alpha * action + (1.0 - self.ema_alpha) * self._a_f
        want = action_to_q(self._a_f)
        delta = np.clip(want - self._target, -self.rate_limit, self.rate_limit)
        self._target = self._target + delta
        self.last_delta = delta
        self._substeps()
        self._step_n += 1
        obs = self._obs()

        g, _w, tilt, h = self._state()
        feet = self.true_feet()
        nfeet = int(feet.sum())
        tilt_deg = float(np.degrees(tilt))
        W = REWARD_WEIGHTS
        upright = (1.0 - g[2]) / 2.0                 # g_z_body = -1 when upright
        r = W["upright"] * upright + W["height"] * min(h, 0.13) / 0.13
        hand = handoff_ok(tilt_deg, feet, self.true_q())
        if self.reward_version in ("v2", "v3"):
            standing = hand
            hold_s = HANDOFF_HOLD_S
            if standing:
                r += W["stand"] + W["feet"] * nfeet / N_LEGS
            if self.reward_version == "v3":
                r -= W["v3_pinned"] * float(np.mean((delta / self.rate_limit) ** 2))
        else:
            standing = tilt < STAND_TILT and h > STAND_H and nfeet >= 4
            hold_s = HOLD_S
            if standing:
                r += W["stand_v1"]
        self._hold = self._hold + CTRL_DT if standing else 0.0
        r -= W["act"] * float(np.mean(action ** 2))
        r -= W["dact"] * float(np.mean((action - self._prev_action) ** 2))
        self._prev_action = action.copy()
        terminated = False
        if self._hold >= hold_s - 1e-9:
            r += W["success"]
            terminated = True
        truncated = self._step_n >= self.max_steps
        info = dict(tilt_deg=tilt_deg, height=h, feet=nfeet, handoff=bool(hand),
                    success=bool(terminated))
        return obs, float(r), terminated, truncated, info

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
    for name, pol in (("hold-pose", lambda o: env._a_f),
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
