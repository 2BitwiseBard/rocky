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


# ------------------------------------------------------------------ D052 amendment: thermal proxy
def _proxy_trip_s(x, dt=0.1, limit_s=3600.0):
    th = rc.ThermalProxy(n=1)
    t = 0.0
    while not th.tripped()[0] and t < limit_s:
        th.accumulate(np.array([x * th.stall]))
        th.update(dt)
        t += dt
    return t


def test_thermal_proxy_matches_the_driver_mock():
    """heat += ((|tau|/stall)^2 - 0.65^2) dt, budget (0.85^2 - 0.65^2) x 180 = 54:
    3 min at 0.85 x stall trips, never at <= continuous — and the trip times
    sit within ~10 % of driver/rocky_driver/mock.py's 70 C cut (the model the
    budget was chosen to match)."""
    from rocky_driver.mock import MockServo
    assert rc.ThermalProxy().budget == pytest.approx(54.0)
    assert _proxy_trip_s(0.85) == pytest.approx(180.0, abs=0.2)
    assert _proxy_trip_s(1.0) == pytest.approx(54.0 / (1 - 0.65 ** 2), abs=0.2)     # 93.5 s at stall
    assert _proxy_trip_s(0.65, dt=1.0) >= 3600.0 and _proxy_trip_s(0.5, dt=1.0) >= 3600.0  # never
    for pct in (80, 85, 100):
        s = MockServo(servo_id=1)
        s.put("TORQUE_ENABLE", 1)
        s.external_load_pct = pct
        t = 0.0
        while s.get("TORQUE_ENABLE") == 1 and t < 600:
            s.advance(0.1)
            t += 0.1
        assert _proxy_trip_s(pct / 100) == pytest.approx(t, rel=0.1), (pct, t)
    # below continuous it cools at the same scale, never below 0
    th = rc.ThermalProxy(n=1)
    th.heat[:] = 10.0
    for _ in range(10):
        th.accumulate(np.array([0.1 * th.stall]))
    th.update(1.0)
    assert th.heat[0] == pytest.approx(10.0 - (0.65 ** 2 - 0.01))
    th.update(0.0)
    for _ in range(100):
        th.accumulate(np.zeros(1))
        th.update(1.0)
    assert th.heat[0] == 0.0


def test_thermal_proxy_trips_in_the_env_after_sustained_overload():
    """Drive every joint into its stop (the actuator saturates at the PEAK torque,
    x = 1): the proxy trips at ~93.5 s, then the hot joints' forcerange ramps
    from 2.94 toward the continuous 1.91 N.m and the heat stops climbing."""
    from rocky_recover_env import Q_LO, Q_HI
    env = RecoverEnv(servo="nominal", seed=0)
    env.reset(seed=0)
    peak = env.model.actuator_forcerange[:15, 1].copy()
    np.testing.assert_allclose(peak, rc.rm.stall_nm())                # MJCF clip = stall (amendment)
    into_stops = np.where(np.arange(15) % 3 == 1, Q_HI + 0.6, Q_LO - 0.6)
    trip_t, k = None, 0
    while trip_t is None and k < 6000:
        env.hold_step(into_stops)
        k += 1
        if env.thermal.tripped().any():
            trip_t = k * rc.CTRL_DT
    assert trip_t == pytest.approx(54.0 / (1 - 0.65 ** 2), abs=1.5), trip_t
    assert np.allclose(np.abs(env.data.actuator_force[:15]), peak, rtol=0.02)   # saturated at peak
    for _ in range(int(30 / rc.CTRL_DT)):                             # 30 s more in the stops
        env.hold_step(into_stops)
    fr = env.model.actuator_forcerange[:15, 1]
    assert np.all(fr < 0.85 * peak) and np.all(fr >= 0.65 * peak - 1e-9), fr / peak
    assert env.thermal.heat.max() <= (1 + rc.THERMAL_RAMP) * env.thermal.budget + 1e-9
    _obs, _r, _te, _tr, info = env.step(np.zeros(15))
    assert info["thermal_tripped"] == 15 and info["thermal_heat_max"] > 1.0
    assert 0.0 < info["thermal_derate_max"] <= 1.0
    cfg = env.config()["thermal"]
    assert cfg["budget"] == pytest.approx(54.0) and cfg["penalty"] == rc.THERMAL_PENALTY
    # a new episode starts cold and at peak again (the derate never leaks across resets)
    env.reset(seed=1)
    np.testing.assert_allclose(env.model.actuator_forcerange[:15, 1], peak)
    assert env.thermal.heat.max() == 0.0


@pytest.mark.parametrize("cls", [RecoverEnv, PebbleEnv])
def test_thermal_warm_start_derates_and_costs(cls):
    """thermal_heat0 starts an episode with warm servos: past the ramp the joint
    is at continuous from the first tick and the reward pays -0.5 x mean derate;
    thermal=False keeps the peak clip (heat still tracked)."""
    e = cls(servo="nominal", thermal_heat0=(1.3, 1.3))
    e.reset(seed=0)
    np.testing.assert_allclose(e.model.actuator_forcerange[:15, 1], rc.rm.continuous_nm())
    _o, _r, _te, _tr, info = e.step(np.zeros(15))
    assert info["thermal_derate_max"] == 1.0 and info["thermal_tripped"] == 15
    assert e.config()["thermal"]["heat0"] == (1.3, 1.3)
    off = cls(servo="nominal", thermal=False, thermal_heat0=(1.3, 1.3))
    off.reset(seed=0)
    np.testing.assert_allclose(off.model.actuator_forcerange[:15, 1], rc.rm.stall_nm())
    _o, _r, _te, _tr, info = off.step(np.zeros(15))
    assert info["thermal_derate_max"] == 0.0 and info["thermal_heat_max"] > 1.0


def test_thermal_proxy_does_not_trip_under_the_wave_gait():
    """The zero-residual wave gait at 45 mm/s for 30 s: stance loads are
    ~0.07-0.18 x stall mean per joint (peaks ~0.8), so the heat never builds
    (measured max 0.0002 of the budget) and nothing is derated."""
    e = PebbleEnv(servo="nominal", ep_seconds=30.0, cmd=(45.0, 0.0, 0.0))
    e.reset(seed=0)
    x0 = float(e.data.xpos[e.torso][0])
    worst = 0.0
    for _ in range(int(30.0 / rc.CTRL_DT)):
        _o, _r, term, _tr, info = e.step(np.zeros(15))
        assert not term
        assert info["thermal_tripped"] == 0 and info["thermal_derate_max"] == 0.0
        worst = max(worst, info["thermal_heat_max"])
    assert worst < 0.02, worst
    assert float(e.data.xpos[e.torso][0]) - x0 > 1.0                # it really walked (~1.26 m)


def test_thermal_heat_is_the_current_not_the_voltage():
    """Review fix: the proxy (and audit_gestures' load) integrate the ELECTRICAL
    torque, rl_common.motor_torque = actuator_force - dof_damping x qvel. A
    free yaw joint swung 0.6 rad at 2 Hz (torso pinned, peaks at the 4.70 rad/s
    no-load speed) holds actuator_force near stall (RMS ~0.87) but draws little
    current (RMS ~0.22): it must heat NOTHING, where the raw-force proxy gained
    ~19 of its 54 budget per minute."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(rc.os.path.join(rc.HERE, "pebble.xml"))
    _jadr, vadr = rc.joint_addrs(m)
    aid, dof = m.actuator("yaw0").id, vadr[0]
    d = mujoco.MjData(m)
    d.qpos[2] = 0.5
    mujoco.mj_forward(m, d)
    root = d.qpos[:7].copy()
    d.ctrl[:] = d.qpos[[m.jnt_qposadr[m.actuator_trnid[i, 0]] for i in range(m.nu)]]
    th, raw = rc.ThermalProxy(), rc.ThermalProxy()
    th.reset(m)
    raw.reset(m)
    F2, E2, vmax, n = 0.0, 0.0, 0.0, 0
    sub = int(round(rc.CTRL_DT / m.opt.timestep))
    for k in range(int(12.0 / m.opt.timestep)):
        t = k * m.opt.timestep
        d.ctrl[aid] = 0.6 * np.sin(2 * np.pi * 2.0 * t)
        d.qpos[:7] = root
        d.qvel[:6] = 0
        mujoco.mj_step(m, d)
        tau_e = rc.motor_torque(m, d, vadr)
        th.accumulate(tau_e)
        raw.accumulate(d.actuator_force[:15])
        if (k + 1) % sub == 0:
            th.update(rc.CTRL_DT)
            raw.update(rc.CTRL_DT)
        if t > 1.0:
            F2 += (d.actuator_force[aid] / th.stall) ** 2
            E2 += (tau_e[0] / th.stall) ** 2
            vmax = max(vmax, abs(d.qvel[dof]))
            n += 1
    assert vmax > 4.5                                              # really saturated
    assert np.sqrt(F2 / n) > 0.8 and np.sqrt(E2 / n) < 0.3, (np.sqrt(F2 / n), np.sqrt(E2 / n))
    assert th.heat[0] == 0.0 and raw.heat[0] > 2.0, (th.heat[0], raw.heat[0])
    # the check the review asked for: a saturated unloaded swing heats less than continuous^2
    assert E2 / n < th.cf ** 2


def test_duty_cycled_overload_is_judged_by_rms():
    """Review fix: heat goes as tau^2, so a 50 % duty of stall (mean |x| 0.50,
    RMS 0.71) trips the proxy (~11.5 min) and judge_load must warn on it —
    the old mean-|tau| column passed it clean."""
    import pebble_feasibility as pf
    x = np.where(np.arange(20) % 2 == 0, 1.0, 0.0)                # stall / zero, 1 s each
    assert abs(x).mean() == 0.5 and np.sqrt((x ** 2).mean()) == pytest.approx(0.7071, abs=1e-3)
    th, t = rc.ThermalProxy(n=1), 0.0
    while not th.tripped()[0] and t < 3600:
        th.accumulate(np.array([x[int(t) % 20] * th.stall]))
        th.update(0.1)
        t += 0.1
    assert t == pytest.approx(691, abs=5)
    q = pf.planted_q()
    load = np.zeros((rc.N_LEGS, 3))
    load[0, 1] = np.sqrt((x ** 2).mean())
    rep = pf.judge_load(pf.check(lambda g, tt: (q, np.zeros(rc.N_LEGS)), 1.0, name="duty"), load)
    assert rep.warnings == ["THERMAL_LOAD_WARN"]
