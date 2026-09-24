"""PebbleEnv — gymnasium wrapper around the Pebble sim (RL groundwork, plan §5.3; D052 contract).

Residual-policy formulation (the sane on-ramp for sim-to-real):
  action (15,) in [-1,1]  ->  joint-target OFFSETS (max +/-0.25 rad) ADDED to
  the analytic wave-gait targets. Policy zero == the proven blind gait, so
  RL starts from a walking robot and learns corrections, not locomotion
  from scratch. The same trick deploys on hardware: the gait engine runs
  anyway; the policy output is a bounded trim.

D052 — what changed and why:
  * the residual goes through an EMA filter (alpha 0.4 at 50 Hz, state in
    the obs) and then the ServoModel (50 Hz hold, latency, slew — not
    load-derated since D052 V2 — 4096 counts) + a per-joint calibration
    offset before the MuJoCo actuator. Before, a residual flip was 0.5 rad in one 20 ms tick
    (25 rad/s) with an action cost of 0.02 — the policy could use a servo
    that does not exist.
  * obs v2 (56, rl_common.GaitObs): sim_imu gravity/gyro (noise, bias),
    quantised one-tick-late encoder q, finite-difference qd, phase,
    command, and the filtered residual a_prev. obs_version=1 replays the
    old 41-value true-state obs for v1 checkpoints.
  * pushes: the old "push" was N(0,12) N at the CoM for one tick = ~0.24 N.s,
    a tap; 6-8 N.s tips the robot. Now each episode, with probability
    push_prob, one sim/shove.py Shove: half-sine at the shell rim, peak
    U(10, 35) N (spans the tip threshold), U(0.3, 0.5) s, random heading,
    starting U(1.0, EP_SECONDS - 1.5) s — applied inside the substep loop
    (the lever torque depends on the current orientation). Surviving the
    episode pays +1.0 at the end.
  * DR is rl_common.DomainRandomizer (friction U(0.5,1.5), per-link mass,
    torso CoM +-15 mm, kp x U(0.7,1.3), forcerange x voltage, joint offsets
    +-1 deg, gravity tilt <= 3 deg); the old 0-1 tick action delay is gone
    (the servo draw's 10-40 ms latency replaces it).
  * height target = rocky_model.stance_torso_z_m() (the old (h+14)/1000
    was 14 mm above the D052 stance and biased the reward).
  * D052 amendment: the MJCF clips at the servo's PEAK (stall) torque; the
    continuous budget is a thermal one, enforced by rl_common.ThermalProxy
    (heat += ((|tau_e|/stall)^2 - 0.65^2) dt, tau_e = force - damping x qvel
    (rl_common.motor_torque, the current term); past the 54 budget the joint's
    forcerange ramps to continuous and the reward pays -0.5 x mean derate).
    From cold it cannot trip inside an 8 s episode (at stall it takes 93 s);
    thermal_heat0=(lo, hi) starts each episode with a warm servo (fraction
    of the budget, per joint) so a policy can meet a derated leg. Default
    (0, 0): off unless asked. info carries thermal_heat_max / _tripped.

reward = 0.75 vel tracking + 0.25 yaw-rate tracking - 0.02 |a|^2
         - 0.1 |a - a_last|^2 (raw action; the weight is a first guess)
         - 1.2 tilt^2 - 8 |h - h_stance| - 0.5 mean(thermal derate)
         ; -5 on a fall ; +1 on survival.
Control at 50 Hz (10 physics substeps); episode 8 s.
cmd_sample=True re-draws the command every episode (30% pure forward,
else vx[15,60] vy[-25,25] wz[-0.35,0.35]) -> ONE policy for the whole
command envelope; the command is in obs.

D052 V2 (review) — cmd_budget=True (the default for new runs): every
command, drawn or fixed, goes through WaveGait.budget() before the gait and
the obs see it, exactly as the cockpit feeds a walker (cmd_eff). Before,
33 of 40 draws (seed 0) were outside the gait's envelope (45.5 mm/s, 0.246
rad/s in place) and 31 exceeded the hard servo speed or a joint limit
(median peak 6.5 rad/s): the residual was learned on a gait the robot
cannot run, and deployed on a different command distribution. Checkpoints
without `cmd_budget` in their env_config trained on raw commands (flag
'unbudgeted cmd'); eval_ppo replays them that way.
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
sys.path.insert(0, HERE)
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS       # noqa: E402
import rocky_model as rm                                            # noqa: E402
import rl_common as rc                                              # noqa: E402
from shove import Shove                                             # noqa: E402

CTRL_DT = rc.CTRL_DT        # 50 Hz policy rate
EP_SECONDS = 8.0
ACT_SCALE = 0.25            # rad of residual authority per joint
SHOVE_PEAK_N = (10.0, 35.0)
SHOVE_DUR_S = (0.3, 0.5)
REWARD_WEIGHTS = dict(vel=0.75, yaw=0.25, act=0.02, dact=0.1, tilt=1.2, height=8.0,
                      fall=5.0, survive=1.0, thermal=rc.THERMAL_PENALTY)


class PebbleEnv(gym.Env if gym else object):
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, cmd=(45.0, 0.0, 0.0), randomize=False, push_prob=0.0,
                 cmd_sample=False, render_mode=None, seed=None, servo="random",
                 ema_alpha=rc.EMA_ALPHA, obs_version=rc.OBS_VERSIONS["gait"],
                 obs_noise=None, ep_seconds=EP_SECONDS, gait_params=None, cmd_budget=True,
                 thermal=True, thermal_heat0=(0.0, 0.0), **_):
        """push_prob: probability of ONE rim shove per episode (D052; it was a
        per-step tap probability). servo: off | nominal | random. ema_alpha 1.0
        = no filter. gait_params: WaveGait kwargs (default params `gait:`; a
        checkpoint replays the gait it trained on — rl_common.LEGACY_GAIT for
        pre-D052 ones). obs_version 1 + servo 'off' + ema 1.0 + LEGACY_GAIT =
        the pre-D052 env. thermal: the ThermalProxy derate + penalty (heat is
        tracked either way); thermal_heat0: (lo, hi) per-joint starting heat,
        fraction of the budget, drawn each episode."""
        self.model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
        self.data = mujoco.MjData(self.model)
        self._note = rc.env_note(self.model)            # fingerprint BEFORE any DR
        self.gait_params = dict(gait_params or rm.gait_defaults())
        self.gait = WaveGait(**self.gait_params)
        self.cmd_asked = np.array(cmd, dtype=np.float64)   # mm/s, mm/s, rad/s — what was asked
        self.cmd_budget = bool(cmd_budget)
        self.cmd = self._fit(self.cmd_asked)              # what the gait and the obs get
        self.randomize = randomize
        self.obs_noise = randomize if obs_noise is None else bool(obs_noise)
        self.push_prob = float(push_prob)
        self.cmd_sample = cmd_sample
        self.servo_mode = servo
        rc.servo_params(servo)                          # validates the mode
        self.ema_alpha = float(ema_alpha)
        self.render_mode = render_mode
        self._renderer = None
        self.rng = np.random.default_rng(seed)
        self.torso = self.model.body("torso").id
        self.substeps = int(round(CTRL_DT / self.model.opt.timestep))
        self.ep_seconds = float(ep_seconds)
        self.max_steps = int(round(self.ep_seconds / CTRL_DT))
        self.h_stance = rm.stance_torso_z_m(self.gait.h)
        self._jadr, self._vadr = rc.joint_addrs(self.model)
        self._fids = rc.foot_geoms(self.model)
        self.obs_builder = rc.make_gait_obs(obs_version, self.model, self.torso,
                                            self._jadr, self._vadr, self._fids)
        self.obs_version = self.obs_builder.version
        self.dr = rc.DomainRandomizer(self.model, self.torso)
        self.dr_draw = self.dr.nominal()
        self.servo = rc.make_servo()
        self.servo_draw = rc.servo_params("off")
        self._q_offset = np.zeros(15)
        self._a_f = np.zeros(15)
        self._prev_action = np.zeros(15)
        self.shove = None
        self.thermal = rc.ThermalProxy(on=thermal)
        self.thermal_heat0 = tuple(float(x) for x in thermal_heat0)
        if gym:
            self.observation_space = spaces.Box(-np.inf, np.inf, (self.obs_builder.dim,),
                                                dtype=np.float32)
            self.action_space = spaces.Box(-1.0, 1.0, (15,), dtype=np.float32)
        self._q0 = None
        self._t = 0.0
        self._step_n = 0

    # ------------------------------------------------------------------ contract
    def config(self):
        key = ("gait", self.obs_version)
        return dict(env="gait", obs_version=self.obs_version, obs_dim=self.obs_builder.dim,
                    obs_names=list(rc.OBS_NAMES[key]), act_dim=15,
                    action=f"residual: q = wave_gait + {ACT_SCALE} * ema(a)", act_scale=ACT_SCALE,
                    ctrl_dt=CTRL_DT, ep_seconds=self.ep_seconds, ema_alpha=self.ema_alpha,
                    servo=self.servo_mode,
                    servo_ranges=dict(rc.SERVO_RANGES) if self.servo_mode == "random" else None,
                    randomize=bool(self.randomize), dr_ranges=dict(rc.DR_RANGES) if self.randomize else None,
                    obs_noise=dict(rc.OBS_NOISE) if self.obs_noise else None,
                    push_prob=self.push_prob, shove=dict(peak_n=SHOVE_PEAK_N, dur_s=SHOVE_DUR_S,
                                                         t0_s=(1.0, self.ep_seconds - 1.5), at="shell rim"),
                    cmd_sample=bool(self.cmd_sample), cmd_budget=self.cmd_budget,
                    gait=dict(self.gait_params),
                    thermal=dict(self.thermal.config(), heat0=self.thermal_heat0),
                    h_stance_m=self.h_stance, reward_rev=rc.REWARD_REV,
                    reward_weights=dict(REWARD_WEIGHTS), **self._note)

    # ------------------------------------------------------------------
    def _phase(self):
        return 2 * np.pi * ((self._t / self.gait.T) % 1.0)

    def _fit(self, cmd):
        """The command the gait runs: WaveGait.budget(cmd) when cmd_budget (V2)."""
        cmd = np.asarray(cmd, dtype=np.float64)
        return np.array(self.gait.budget(*cmd), dtype=np.float64) if self.cmd_budget else cmd.copy()

    def _obs(self):
        cmd_n = self.cmd / np.array([60.0, 60.0, 0.6])
        return self.obs_builder(self.data, self._t, self._phase(), cmd_n, self._a_f)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.dr_draw = self.dr.draw(self.rng) if self.randomize else self.dr.nominal()
        self.servo_draw = rc.servo_params(self.servo_mode, self.rng)
        self.dr.apply(self.dr_draw, voltage=self.servo_draw["voltage"])
        lo, hi = self.thermal_heat0                      # the rng is only touched when asked
        self.thermal.reset(self.model, self.rng.uniform(lo, hi, 15) if hi > 0 else lo)
        rc.refresh_constants(self.model, self.data)
        mujoco.mj_resetData(self.model, self.data)
        rc.apply_servo_params(self.servo, self.servo_draw)
        self._q_offset = self.dr_draw["q_offset"].copy()
        self._a_f = np.zeros(15)
        self._prev_action = np.zeros(15)
        if self.cmd_sample:
            if self.rng.random() < 0.30:
                self.cmd_asked = np.array([45.0, 0.0, 0.0])
            else:
                self.cmd_asked = np.array([self.rng.uniform(15.0, 60.0),
                                           self.rng.uniform(-25.0, 25.0),
                                           self.rng.uniform(-0.35, 0.35)])
        self.cmd = self._fit(self.cmd_asked)
        self.shove = None
        if self.push_prob > 0 and self.rng.random() < self.push_prob:
            peak = self.rng.uniform(*SHOVE_PEAK_N)
            phi = self.rng.uniform(0, 2 * np.pi)
            self.shove = Shove(peak * np.cos(phi), peak * np.sin(phi),
                               dur=self.rng.uniform(*SHOVE_DUR_S),
                               t0=self.rng.uniform(1.0, self.ep_seconds - 1.5))
        q0 = np.array([leg_ik(body_to_leg(i, self.gait.p_nom[i]))
                       for i in range(N_LEGS)]).flatten()
        self.data.qpos[0:3] = [0, 0, rm.spawn_z_m(self.gait.h)]
        self.data.qpos[3:7] = [1, 0, 0, 0]
        self.data.qpos[self._jadr] = q0          # by address (interleave gotcha)
        self.data.ctrl[:15] = q0 + self._q_offset
        mujoco.mj_forward(self.model, self.data)
        self._q0 = q0
        self._t = 0.0
        self._step_n = 0
        self.servo.reset(q0)
        # settle 0.4 s at stance before the clock starts
        for _ in range(int(0.4 / self.model.opt.timestep)):
            mujoco.mj_step(self.model, self.data)
        self.obs_builder.reset(rng=self.rng, noise=self.obs_noise, q_offset=self._q_offset)
        return self._obs(), {}

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=np.float64), -1, 1)
        self._a_f = self.ema_alpha * action + (1.0 - self.ema_alpha) * self._a_f
        q_gait, _, _ = self.gait.joint_targets(self._t, *self.cmd)
        target = q_gait.flatten() + ACT_SCALE * self._a_f
        d = self.data
        dt = self.model.opt.timestep
        for k in range(self.substeps):
            if self.shove is not None:
                self.shove.apply(self.model, d, self.torso, self._t + k * dt)
            tgt = self.servo.filter(target, dt, force=d.actuator_force[:15])
            d.ctrl[:15] = tgt + self._q_offset
            mujoco.mj_step(self.model, d)
            self.thermal.accumulate(rc.motor_torque(self.model, d, self._vadr))
        derate = self.thermal.update(CTRL_DT, self.model)
        self._t += CTRL_DT
        self._step_n += 1

        R = d.xmat[self.torso].reshape(3, 3)
        from sim_imu import grav_body, tilt_from_grav
        tilt = tilt_from_grav(grav_body(self.model, d, self.torso))
        v_world = d.cvel[self.torso][3:6] * 1000.0      # mm/s translational
        v_body = R.T @ v_world
        v_err = np.linalg.norm(v_body[:2] - self.cmd[:2])
        w_body = R.T @ d.cvel[self.torso][0:3]
        wz_err = abs(float(w_body[2]) - self.cmd[2])
        h = d.xpos[self.torso][2]
        h_err = abs(h - self.h_stance)
        W = REWARD_WEIGHTS
        r_vel = np.exp(-(v_err / 40.0) ** 2)
        r_yaw = np.exp(-(wz_err / 0.4) ** 2)
        r_act = -W["act"] * float(np.mean(action ** 2))
        r_dact = -W["dact"] * float(np.mean((action - self._prev_action) ** 2))
        self._prev_action = action.copy()
        r_tilt = -W["tilt"] * tilt ** 2
        r_h = -W["height"] * h_err
        r_heat = -W["thermal"] * float(derate.mean())
        reward = float(W["vel"] * r_vel + W["yaw"] * r_yaw + r_act + r_dact + r_tilt + r_h + r_heat)
        terminated = bool(tilt > np.deg2rad(45) or h < 0.05)
        if terminated:
            reward -= W["fall"]
        truncated = self._step_n >= self.max_steps
        if truncated and not terminated:
            reward += W["survive"]
        info = dict(v_body=v_body[:2], tilt_deg=np.rad2deg(tilt), height=h,
                    shoved=self.shove is not None and self._t >= self.shove.t0, **self.thermal.info())
        return self._obs(), reward, terminated, truncated, info

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, 480, 720)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = self.data.xpos[self.torso]
        cam.distance, cam.elevation, cam.azimuth = 0.85, -18, 145
        self._renderer.update_scene(self.data, cam)
        return self._renderer.render()


def sanity(servo="nominal"):
    """Zero-action rollout must walk (the analytic gait is the baseline)."""
    env = PebbleEnv(servo=servo)
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
    print(f"zero-action rollout (servo {servo}): {steps} steps, return {total:.1f}, "
          f"walked {(x1 - x0) * 1000:.0f} mm, terminated={term}")
    assert not term, "the baseline gait fell — env wiring is wrong"
    assert (x1 - x0) > 0.20, "baseline should cover >200 mm in 8 s"
    return total


if __name__ == "__main__":
    sanity()
