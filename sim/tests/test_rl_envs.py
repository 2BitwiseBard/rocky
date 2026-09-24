"""D052: the RL envs train on the robot's contract, not the sim's flattery.

What these pin down (each one was a CONFIRMED finding of the 2026-09-24 RL
review): obs dims/versions per contract; ONE obs builder shared by the env
and the deployed righter (identical output on the same state); the servo
model is IN the loop (a step target does not reach the actuator in one
tick); the gait env's shove is a real rim shove; the vector env autoresets
in the SAME step (the trainer's assumption); config() carries the robot
fingerprint; the righter refuses a checkpoint whose obs it cannot build;
the hardware-computable handoff criterion.
"""
import os
import sys
import warnings

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
for sub in ("sim", "gait", "perception"):
    sys.path.insert(0, os.path.join(REPO, sub))
mujoco = pytest.importorskip("mujoco")
gym = pytest.importorskip("gymnasium")

import rl_common as rc                                               # noqa: E402
from rocky_recover_env import (RecoverEnv, handoff_ok, kinematic_height_m,  # noqa: E402
                               q_to_action)
from rocky_env import PebbleEnv                                      # noqa: E402
from righter import PolicyRighter                                    # noqa: E402
from model_fingerprint import robot_fingerprint                      # noqa: E402


# ------------------------------------------------------------------ obs contract
@pytest.mark.parametrize("cls,env,version,dim", [
    (RecoverEnv, "recover", 2, 57), (RecoverEnv, "recover", 1, 39),
    (PebbleEnv, "gait", 2, 56), (PebbleEnv, "gait", 1, 41)])
def test_obs_dims_and_versions(cls, env, version, dim):
    e = cls(obs_version=version, servo="nominal")
    obs, _ = e.reset(seed=0)
    assert obs.shape == (dim,) == e.observation_space.shape
    assert np.all(np.isfinite(obs))
    cfg = e.config()
    assert cfg["obs_version"] == version and cfg["obs_dim"] == dim == rc.OBS_DIMS[(env, version)]
    assert len(cfg["obs_names"]) == dim
    assert rc.OBS_VERSIONS[env] == 2                                  # the current contract


def test_recover_obs_v2_has_no_privileged_height():
    names = rc.OBS_NAMES[("recover", 2)]
    assert not any(n.startswith("h") for n in names)
    assert [f"foot{i}" for i in range(5)] == [n for n in names if n.startswith("foot")]


def _recover_env():
    return RecoverEnv(servo="nominal", randomize=False, reward="v2")


def test_env_and_righter_build_identical_obs():
    """Same state -> same obs, tick after tick, through the SAME builder class
    and the same action filter / clamp (the righter replays config())."""
    env = _recover_env()
    obs0, _ = env.reset(seed=4)
    seen = []
    actions = [np.full(15, 0.3), np.full(15, -0.5), np.linspace(-1, 1, 15)]

    def policy(o):
        seen.append(np.array(o, copy=True))
        return actions[min(len(seen) - 1, len(actions) - 1)]

    r = PolicyRighter(None, env.model, env.data, env.torso, env._foot_gid,
                      policy=policy, contract=env.config())
    t = 0.0
    q = r(t, env.model.opt.timestep)                  # first call: obs from the landed pose
    np.testing.assert_allclose(seen[0], obs0, atol=1e-6)
    # the righter already took action 0 through the SAME filter the env will apply
    np.testing.assert_allclose(r._a_f, 0.4 * actions[0] + 0.6 * env._a_f, atol=1e-12)
    for k, a in enumerate(actions[:2]):
        obs, *_ = env.step(a)
        # the env's own target after the step == the righter's after the same action
        np.testing.assert_allclose(env._target, np.asarray(q).flatten(), atol=1e-12)
        t += rc.CTRL_DT
        q = r(t, env.model.opt.timestep)
        np.testing.assert_allclose(seen[k + 1], obs, atol=1e-6)


def test_legacy_obs_is_shared_too():
    env = RecoverEnv(obs_version=1, servo="off", ema_alpha=1.0, rate_limit_rad_s=5.0)
    obs0, _ = env.reset(seed=2)
    seen = []
    c = dict(env.config())
    r = PolicyRighter(None, env.model, env.data, env.torso, env._foot_gid,
                      policy=lambda o: seen.append(o) or np.zeros(15), contract=c)
    r(0.0, 0.002)
    np.testing.assert_allclose(seen[0], obs0, atol=1e-6)


# ------------------------------------------------------------------ servo in the loop
def test_servo_filter_is_in_the_loop():
    env = RecoverEnv(servo="nominal", ema_alpha=1.0, rate_limit_rad_s=4.0)
    env.reset(seed=1)
    tgt0 = env._target.copy()
    a = -np.sign(q_to_action(tgt0) + 1e-9)            # the far end of every joint's range
    env.step(a)
    moved = np.abs(env._target - tgt0)
    assert moved.max() > 0.07                         # the command clamp let 0.08 rad through
    # ...but the actuator has not got it: 20 ms of bus latency = nothing yet
    ctrl = env.data.ctrl[:15] - env._q_offset
    assert np.abs(ctrl - env._target).max() > 0.05
    env2 = RecoverEnv(servo="off", ema_alpha=1.0, rate_limit_rad_s=4.0)
    env2.reset(seed=1)
    env2.step(a)
    np.testing.assert_allclose(env2.data.ctrl[:15] - env2._q_offset, env2._target, atol=1e-12)


def test_servo_random_draws_voltage_into_forcerange():
    env = RecoverEnv(servo="random", randomize=True)
    base = env.dr._base_frange[:15, 1].copy()
    env.reset(seed=7)
    sp = env.servo_draw
    assert 0.85 <= sp["voltage"] <= 1.0
    assert 0.010 <= sp["latency_s"] <= 0.040
    assert sp["rate_rad_s"] <= rc.SERVO_NO_LOAD_RAD_S * sp["voltage"] + 1e-9
    np.testing.assert_allclose(env.model.actuator_forcerange[:15, 1], base * sp["voltage"])
    # kp scaled consistently: position servo stays unbiased
    g, b = env.model.actuator_gainprm[:15, 0], env.model.actuator_biasprm[:15, 1]
    np.testing.assert_allclose(g, -b)
    env.randomize = False                              # DR draws never compound: nominal restores
    env.servo_mode = "nominal"
    env.reset(seed=0)
    np.testing.assert_allclose(env.model.actuator_forcerange[:15, 1], base)


def test_ema_filter_state_is_in_obs():
    env = RecoverEnv(servo="nominal", ema_alpha=0.4)
    env.reset(seed=0)
    af0 = env._a_f.copy()
    obs, *_ = env.step(np.ones(15))
    np.testing.assert_allclose(env._a_f, 0.4 + 0.6 * af0)
    np.testing.assert_allclose(obs[-15:], env._a_f, atol=1e-6)


# ------------------------------------------------------------------ shove
def test_gait_env_shove_is_applied_at_the_rim():
    env = PebbleEnv(push_prob=1.0, servo="nominal")
    env.reset(seed=3)
    sh = env.shove
    assert sh is not None and 10.0 <= sh.peak_n <= 35.0 and 0.3 <= sh.dur <= 0.5
    assert 1.0 <= sh.t0 <= env.ep_seconds - 1.5
    seen_f = seen_tau = 0.0
    while env._t < sh.t0 + sh.dur + 0.1:
        env.step(np.zeros(15))
        xf = env.data.xfrc_applied[env.torso]
        seen_f = max(seen_f, float(np.linalg.norm(xf[:3])))
        seen_tau = max(seen_tau, float(np.linalg.norm(xf[3:])))
    assert seen_f > 0.5 * sh.peak_n                   # the half-sine peak really went in
    assert seen_tau > 0.01                            # at the rim: a lever torque, not a CoM push
    np.testing.assert_allclose(env.data.xfrc_applied[env.torso], 0.0)   # and it ended
    quiet = PebbleEnv(push_prob=0.0, servo="nominal")
    quiet.reset(seed=3)
    assert quiet.shove is None


# ------------------------------------------------------------------ autoreset
@pytest.mark.parametrize("env_name", ["recover", "gait"])
def test_same_step_autoreset(env_name):
    fns = [rc.make_env(0, env_name, ep_seconds=0.06, servo="nominal")]
    venv = rc.make_vec_env(fns, sync=True)
    assert venv.autoreset_mode == gym.vector.AutoresetMode.SAME_STEP
    obs, _ = venv.reset(seed=0)
    for k in range(3):
        obs, rew, term, trunc, infos = venv.step(np.zeros((1, 15), np.float32))
    assert bool(trunc[0] or term[0])
    # the done step already carries the NEW episode's first obs...
    assert "final_obs" in infos
    inner = venv.envs[0].unwrapped
    assert inner._step_n == 0
    assert not np.allclose(infos["final_obs"][0], obs[0])
    r, l = rc.episode_stats(infos)
    assert l == [3]
    # ...and the next step is a real step of that episode (its action is used)
    venv.step(np.zeros((1, 15), np.float32))
    assert inner._step_n == 1
    venv.close()


# ------------------------------------------------------------------ config / fingerprint
@pytest.mark.parametrize("cls", [RecoverEnv, PebbleEnv])
def test_config_has_fingerprint_and_versions(cls):
    e = cls(randomize=True, servo="random")
    e.reset(seed=0)                                   # DR mutates the model...
    cfg = e.config()
    fresh = mujoco.MjModel.from_xml_path(os.path.join(REPO, "sim", "pebble.xml"))
    assert cfg["robot_fingerprint"] == robot_fingerprint(fresh)     # ...the fingerprint is the robot's
    assert cfg["mujoco"] == mujoco.__version__
    assert cfg["reward_rev"] == rc.REWARD_REV and cfg["servo"] == "random"
    assert cfg["dr_ranges"]["friction"] == (0.5, 1.5)


def test_checkpoint_contract_legacy_and_flags():
    c = rc.checkpoint_contract({"args": {"env": "recover", "reward": "v1"}, "obs_dim": 39})
    assert c["legacy"] and c["obs_version"] == 1 and c["rate_limit_rad_s"] == 5.0
    assert "legacy obs" in c["flags"] and "exceeds servo" in c["flags"]
    c = rc.checkpoint_contract({"args": {"env": "recover", "rate_limit": 3.0}, "obs_dim": 39})
    assert "exceeds servo" not in c["flags"]
    new = RecoverEnv(servo="nominal").config()
    c = rc.checkpoint_contract({"env_config": new, "args": {}})
    assert c["flags"] == [] and not c["legacy"]
    g = rc.checkpoint_contract({"args": {"env": "walk"}, "obs_dim": 41})
    assert g["env"] == "gait" and g["gait"]["cycle_time"] == 1.6


# ------------------------------------------------------------------ righter refuses
def test_righter_refuses_unknown_obs_version():
    env = _recover_env()
    env.reset(seed=0)
    bad = dict(env.config(), obs_version=99)
    with pytest.raises(ValueError, match="obs version"):
        PolicyRighter(None, env.model, env.data, env.torso, env._foot_gid,
                      policy=lambda o: np.zeros(15), contract=bad)
    bad = dict(env.config(), obs_dim=39)               # v2 claims 39 values: inconsistent
    with pytest.raises(ValueError, match="obs dim"):
        PolicyRighter(None, env.model, env.data, env.torso, env._foot_gid,
                      policy=lambda o: np.zeros(15), contract=bad)
    gait = dict(PebbleEnv(servo="nominal").config())
    with pytest.raises(ValueError, match="env"):
        PolicyRighter(None, env.model, env.data, env.torso, env._foot_gid,
                      policy=lambda o: np.zeros(15), contract=gait)


def test_righter_warns_on_fingerprint_mismatch():
    env = _recover_env()
    env.reset(seed=0)
    c = dict(env.config(), robot_fingerprint="000000000000")
    with pytest.warns(UserWarning, match="robot"):
        r = PolicyRighter(None, env.model, env.data, env.torso, env._foot_gid,
                          policy=lambda o: np.zeros(15), contract=c)
    assert r.fingerprint == "mismatch"


def test_righter_loads_a_real_checkpoint(tmp_path):
    torch = pytest.importorskip("torch")
    from train_ppo import Agent, RunningMeanStd
    env = _recover_env()
    env.reset(seed=0)
    cfg = env.config()
    agent = Agent(cfg["obs_dim"], 15)
    rms = RunningMeanStd((cfg["obs_dim"],))
    p = tmp_path / "ck.pt"
    torch.save(dict(model=agent.state_dict(), obs_rms=rms.state_dict(), obs_dim=cfg["obs_dim"],
                    act_dim=15, args={"env": "recover"}, env_config=cfg, global_step=0), p)
    with warnings.catch_warnings():
        warnings.simplefilter("error")                 # a matching D052 checkpoint: no warnings
        r = PolicyRighter(str(p), env.model, env.data, env.torso, env._foot_gid)
    q = r(0.0, 0.002)
    assert q.shape == (5, 3) and r.fingerprint == "match"
    cfg_bad = dict(cfg, obs_version=7)
    torch.save(dict(model=agent.state_dict(), obs_rms=rms.state_dict(), obs_dim=cfg["obs_dim"],
                    act_dim=15, args={}, env_config=cfg_bad), p)
    with pytest.raises(ValueError):
        PolicyRighter(str(p), env.model, env.data, env.torso, env._foot_gid)


# ------------------------------------------------------------------ handoff criterion
def _planted():
    from pebble_gait import WaveGait, leg_ik, body_to_leg
    g = WaveGait(body_height=118.0, stance_radius=185.0)
    return np.array([leg_ik(body_to_leg(i, g.p_nom[i])) for i in range(5)])


def test_handoff_ok_is_hardware_computable():
    q = _planted()
    assert abs(kinematic_height_m(q) - 0.118) < 0.002
    five = np.ones(5, bool)
    assert handoff_ok(0.0, five, q)
    assert handoff_ok(20.0, [1, 1, 1, 0, 0], q[:, 1:])       # (5, 2) hip/knee form
    assert not handoff_ok(30.0, five, q)                      # tilted
    assert not handoff_ok(0.0, [1, 1, 0, 0, 0], q)            # two feet is not standing
    folded = q.copy()
    folded[:, 1], folded[:, 2] = np.deg2rad(10.0), np.deg2rad(-150.0)   # kneeling on the shins
    assert kinematic_height_m(folded) < 0.09
    assert not handoff_ok(0.0, five, folded)


# ------------------------------------------------------------------ sensors
def test_sensors_quantise_delay_and_difference():
    env = RecoverEnv(servo="nominal")
    env.reset(seed=0)
    s = rc.Sensors(env.model, env.torso, env._jadr, env._foot_gid)
    s.reset(noise=False)
    a = s.sample(env.data)
    q_then = s.encoder(env.data)
    np.testing.assert_allclose(a["qd"], 0.0)
    counts = a["q"] * rc.COUNTS_PER_RAD
    np.testing.assert_allclose(counts, np.round(counts), atol=1e-6)     # whole counts
    env.step(np.ones(15))
    b = s.sample(env.data)                                   # one tick late: still the old q
    np.testing.assert_allclose(b["q"], q_then, atol=1e-12)
    c = s.sample(env.data)
    np.testing.assert_allclose(c["qd"], (c["q"] - b["q"]) / rc.CTRL_DT, atol=1e-9)


def test_sim_imu_follows_model_gravity():
    from sim_imu import SimIMU
    env = PebbleEnv(servo="nominal")
    env.reset(seed=0)
    imu = SimIMU(env.model, "torso")
    level = imu.read(env.data)
    assert level["tilt"] < np.deg2rad(2.0)
    ang = np.deg2rad(3.0)
    env.model.opt.gravity[:] = 9.81 * np.array([np.sin(ang), 0.0, -np.cos(ang)])
    tilted = imu.read(env.data)
    assert abs(tilted["tilt"] - level["tilt"]) > np.deg2rad(2.0)    # an accelerometer sees the slope
    lag = SimIMU(env.model, "torso", latency_s=0.02)
    first = lag.read(env.data, 0.0)
    env.step(np.zeros(15))
    second = lag.read(env.data, 0.01)                              # too fresh: still the first reading
    np.testing.assert_allclose(second["grav"], first["grav"])


# ------------------------------------------------------------------ V2: budgeted commands
def test_gait_env_trains_on_budgeted_commands():
    """Review: cmd_sample drew 33/40 commands outside the gait's envelope and
    trained the residual on them unbudgeted, while the cockpit feeds cmd_eff."""
    import pebble_feasibility as pf
    env = PebbleEnv(cmd_sample=True, servo="off", seed=0)
    assert env.config()["cmd_budget"] is True
    asked_out = 0
    for s in range(12):
        env.reset(seed=s)
        c = env.cmd
        assert np.allclose(c, env.gait.budget(*c), atol=1e-9)      # already inside: budget is a no-op
        asked_out += not np.allclose(env.cmd_asked, c)
        rep = pf.check_gait(env.gait, tuple(c))
        assert not [f for f in rep.fails if f.startswith(("SPEED_HARD", "LIMIT"))], (c, rep.fails)
    assert asked_out > 0                                            # the draw itself still spans past it
    raw = PebbleEnv(cmd=(60.0, 0.0, 0.5), cmd_budget=False, servo="off")
    raw.reset(seed=0)
    assert tuple(raw.cmd) == (60.0, 0.0, 0.5)                       # legacy replay: what it trained on
    old = rc.checkpoint_contract({"env_config": dict(raw.config(), cmd_budget=None), "args": {}})
    assert "unbudgeted cmd" in old["flags"]
    assert "unbudgeted cmd" not in rc.checkpoint_contract({"env_config": env.config(), "args": {}})["flags"]
