"""Shared RL plumbing (D052): ONE sensor model, ONE domain randomiser, ONE
servo draw, ONE checkpoint contract — for both envs AND the deployed
righter adapter, so "the policy sees what it trained on" is true by
construction instead of by a docstring.

Why this file exists (the 2026-09-24 RL review, all CONFIRMED):
  * both envs trained on the ideal actuator: no latency, no hold, the
    recover target slewed at 5 rad/s (> the ST3215's 4.7 no-load, ~2x its
    loaded speed) and the gait residual could flip 0.5 rad in one tick;
  * the recover obs could not be produced on the robot: world torso
    height, a contact count that included self-contacts, MuJoCo qvel,
    zero noise, zero delay — and the righter counted feet differently
    from the env it claimed to mirror;
  * DR drew friction around 1.2 (the old floor) and scaled nothing the
    servo actually varies (kp, voltage sag, calibration).

What lives here:
  Sensors       IMU (sim_imu) + joint encoders (quantised, noisy, 1-tick
                delay, finite-difference velocity at the control rate, the
                Pi's recipe) + foot switches (perception.contacts, floor
                only, hysteresis, dropout, a stuck bit)
  RecoverObs /  the obs builders for obs version 2 (and LegacyRecoverObs /
  GaitObs       LegacyGaitObs for version-1 checkpoints)
  DomainRandomizer   per-episode draws from stored base values
  servo_params  the per-episode servo draw (off / nominal / random)
  checkpoint_contract   what a checkpoint says about the env it trained in
  make_env / make_vec_env   the trainer's env factory (SAME_STEP autoreset)

mujoco is only imported inside functions: rocky_recover_env.handoff_ok must
stay importable on the robot's computer, which has no mujoco.
"""
from __future__ import annotations

import os
import sys
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
for _sub in ("gait", "perception"):
    _p = os.path.join(HERE, "..", _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import rocky_model as rm                                               # noqa: E402
from sim_imu import SimIMU                                              # noqa: E402

N_LEGS = 5
N_JOINTS = 15
CTRL_DT = 0.02                                   # 50 Hz policy = bus rate
SERVO_NO_LOAD_RAD_S = rm.no_load_rad_s()         # 4.7: nothing may be commanded faster
SERVO_SAFE_RAD_S = rm.servo_speed("free")        # 4.0: policy-side command clamp, NEW runs
LEGACY_RATE_LIMIT_RAD_S = 5.0                    # what every pre-D052 recover checkpoint used
EMA_ALPHA = 0.4                                  # in-env action filter at 50 Hz (B30 jitter)
COUNTS_PER_RAD = rm.counts_per_rad()             # 651.9

SERVO_MODES = ("off", "nominal", "random")
# per-episode servo draw for --servo random. voltage is ONE draw that scales both
# the slew and the forcerange: a sagging 3S pack makes the servo slower AND weaker
SERVO_RANGES = dict(latency_s=(0.010, 0.040), rate_rad_s=(2.0, SERVO_NO_LOAD_RAD_S),
                    voltage=(0.85, 1.0))

DR_RANGES = dict(
    friction=(0.5, 1.5),          # sliding mu of the foot/floor pair (params base 0.8)
    link_mass=(0.85, 1.15),       # per-body scale (mass AND inertia)
    com_xy_mm=15.0,               # torso CoM shift, each of x/y in +-this (battery / pod placement)
    kp=(0.7, 1.3),                # per-actuator position-gain scale (servo-to-servo spread)
    joint_offset_deg=1.0,         # per-joint zero error between command and joint (+-)
    grav_tilt_deg=3.0,            # uneven-floor stand-in
)

# observation noise when the env randomises (all 0 when it does not). A guess
# at BNO085 game-rotation-vector + ST3215 encoder quality, not a measurement.
OBS_NOISE = dict(grav_sigma=0.02, gyro_sigma=0.02, gyro_bias=0.02,
                 q_sigma_counts=1.0, qd_sigma=0.1, foot_dropout_max=0.1, foot_stuck_p=0.05)

OBS_VERSIONS = {"recover": 2, "gait": 2}         # the current contract
OBS_DIMS = {("recover", 1): 39, ("recover", 2): 57, ("gait", 1): 41, ("gait", 2): 56}
REWARD_REV = "D052"
# the WaveGait every pre-D052 gait checkpoint trained its residual on (D052
# retuned T 1.6 -> 2.0 s, step 32 -> 24 mm): a residual replayed on another
# gait is a different controller, so legacy replays pin this one
LEGACY_GAIT = dict(body_height=118.0, stance_radius=185.0, cycle_time=1.6, duty=0.8, step_height=32.0)


def _jnames(prefix):
    return [f"{prefix}_{j}{i}" for i in range(N_LEGS) for j in ("yaw", "hip", "knee")]


OBS_NAMES = {
    ("recover", 1): ["grav_x", "grav_y", "grav_z", "gyro_x", "gyro_y", "gyro_z"]
    + _jnames("q") + _jnames("qd") + ["h/0.15", "foot_pairs/5", "tilt/pi"],
    ("recover", 2): ["grav_x", "grav_y", "grav_z", "gyro_x", "gyro_y", "gyro_z"]
    + _jnames("q") + _jnames("qd") + [f"foot{i}" for i in range(N_LEGS)] + ["tilt/pi"]
    + _jnames("a_prev"),
    ("gait", 1): ["grav_x", "grav_y", "grav_z", "gyro_x", "gyro_y", "gyro_z"]
    + _jnames("q") + _jnames("qd") + ["sin_phase", "cos_phase", "vx/60", "vy/60", "wz/0.6"],
    ("gait", 2): ["grav_x", "grav_y", "grav_z", "gyro_x", "gyro_y", "gyro_z"]
    + _jnames("q") + _jnames("qd") + ["sin_phase", "cos_phase", "vx/60", "vy/60", "wz/0.6"]
    + _jnames("a_prev"),
}
for _k, _v in OBS_NAMES.items():
    assert len(_v) == OBS_DIMS[_k], (_k, len(_v))


def joint_addrs(model):
    """(qpos addresses, dof addresses) of the 15 leg joints, leg-major (the
    BUILD_LOG qpos-interleave gotcha: never slice qpos[7:22])."""
    jadr = [int(model.joint(f"{n}{i}").qposadr[0]) for i in range(N_LEGS) for n in ("yaw", "hip", "knee")]
    vadr = [int(model.joint(f"{n}{i}").dofadr[0]) for i in range(N_LEGS) for n in ("yaw", "hip", "knee")]
    return jadr, vadr


def foot_geoms(model):
    return [int(model.geom(f"foot{i}").id) for i in range(N_LEGS)]


def joint_limits():
    """(Q_LO, Q_HI) float arrays (15,) from params — the recover action range."""
    lo, hi = rm.joint_limits_rad()
    return np.tile(lo, N_LEGS), np.tile(hi, N_LEGS)


# ---------------------------------------------------------------- sensors
def legacy_feet_count(data, foot_gids):
    """Obs-v1 'feet': the number of contact PAIRS touching any foot geom, capped
    at 5 — self-contacts included. Wrong, but it is what v1 checkpoints saw."""
    n = 0
    fs = set(int(g) for g in foot_gids)
    for c in range(data.ncon):
        con = data.contact[c]
        if int(con.geom1) in fs or int(con.geom2) in fs:
            n += 1
    return min(n, N_LEGS)


class Sensors:
    """What the Pi can read once per 50 Hz tick, and nothing it cannot.

    IMU      sim_imu.SimIMU: gravity direction + gyro, noise + per-episode bias
    encoders q quantised to 1 count (+ sigma counts of noise), minus the
             per-joint calibration offset (the encoder shares the servo's
             miscalibrated zero); qd = finite difference of that quantised q
             at the control rate (+ noise) — the Pi has no velocity sensor;
             both delivered ONE TICK LATE (a sync-read reply arrives with
             the next bus cycle)
    feet     perception.contacts.foot_contacts: floor-only, 2.0/1.0 N
             hysteresis; per-episode dropout probability (a closed switch
             reads open) and, with foot_stuck_p, one foot stuck for the
             whole episode
    """

    def __init__(self, model, torso, jadr, fids):
        self.model, self.torso = model, int(torso)
        self.jadr = list(jadr)
        self.fids = list(fids)
        self.imu = SimIMU(model, self.torso)
        self.noise = {k: 0.0 for k in OBS_NOISE}
        self.q_offset = np.zeros(N_JOINTS)
        self.rng = np.random.default_rng(0)
        self._primed = False

    def reset(self, rng=None, noise=None, q_offset=None):
        """noise: None/False = ideal, True = OBS_NOISE, or a dict overriding it."""
        if rng is not None:
            self.rng = rng
        if noise is True:
            noise = dict(OBS_NOISE)
        elif not noise:
            noise = {k: 0.0 for k in OBS_NOISE}
        else:
            noise = {**{k: 0.0 for k in OBS_NOISE}, **noise}
        self.noise = noise
        self.q_offset = np.zeros(N_JOINTS) if q_offset is None else np.asarray(q_offset, float).copy()
        self.imu.reset(rng=self.rng, grav_sigma=noise["grav_sigma"], gyro_sigma=noise["gyro_sigma"],
                       gyro_bias=noise["gyro_bias"], latency_s=0.0)
        self.foot_state = np.zeros(N_LEGS, dtype=bool)
        self.dropout = self.rng.uniform(0.0, noise["foot_dropout_max"]) if noise["foot_dropout_max"] > 0 else 0.0
        self.stuck = None                                 # (foot, value)
        if noise["foot_stuck_p"] > 0 and self.rng.random() < noise["foot_stuck_p"]:
            self.stuck = (int(self.rng.integers(N_LEGS)), bool(self.rng.random() < 0.5))
        self._primed = False

    def encoder(self, data):
        """The joint positions the servos report NOW (quantised, noisy, offset)."""
        q = np.asarray(data.qpos[self.jadr], float) - self.q_offset
        counts = q * COUNTS_PER_RAD
        if self.noise["q_sigma_counts"] > 0:
            counts = counts + self.rng.normal(0.0, self.noise["q_sigma_counts"], N_JOINTS)
        return np.round(counts) / COUNTS_PER_RAD

    def feet(self, data):
        from contacts import foot_contacts
        f = foot_contacts(self.model, data, self.fids, self.foot_state)
        if self.dropout > 0:
            f = f & (self.rng.random(N_LEGS) >= self.dropout)
        if self.stuck is not None:
            f[self.stuck[0]] = self.stuck[1]
        return f

    def sample(self, data, t=None):
        """One control tick. Returns dict(grav, gyro, tilt, q, qd, feet) with q/qd
        one tick late. The first call after reset() primes the pipeline with
        the current reading (qd = 0)."""
        imu = self.imu.read(data, t)
        q_now = self.encoder(data)
        if not self._primed:
            self._q_prev = q_now.copy()
            self._delayed = (q_now.copy(), np.zeros(N_JOINTS))
            self._primed = True
        qd_now = (q_now - self._q_prev) / CTRL_DT
        if self.noise["qd_sigma"] > 0:
            qd_now = qd_now + self.rng.normal(0.0, self.noise["qd_sigma"], N_JOINTS)
        q_out, qd_out = self._delayed                     # 1-tick ring buffer
        self._delayed = (q_now, qd_now)
        self._q_prev = q_now
        return dict(grav=imu["grav"], gyro=imu["gyro"], tilt=imu["tilt"],
                    q=q_out.copy(), qd=qd_out.copy(), feet=self.feet(data))


class RecoverObs:
    """Recover obs v2 (57): grav(3) gyro(3) q(15) qd(15) feet(5) tilt/pi(1) a_prev(15).
    a_prev is the FILTERED action the env / righter last commanded (the EMA
    state): with it in the obs the filtered system stays Markov."""
    version = 2
    dim = OBS_DIMS[("recover", 2)]

    def __init__(self, model, torso, jadr, vadr, fids):
        self.sensors = Sensors(model, torso, jadr, fids)

    def reset(self, rng=None, noise=None, q_offset=None):
        self.sensors.reset(rng=rng, noise=noise, q_offset=q_offset)

    def encoder(self, data):
        return self.sensors.encoder(data)

    def __call__(self, data, t, prev_action):
        s = self.sensors.sample(data, t)
        self.last = s
        return np.concatenate([s["grav"], s["gyro"], s["q"], s["qd"], s["feet"].astype(float),
                               [s["tilt"] / np.pi], np.asarray(prev_action, float)]).astype(np.float32)


class LegacyRecoverObs:
    """Recover obs v1 (39), exactly as the pre-D052 env built it: world-z
    gravity, TRUE MuJoCo qpos/qvel, torso height, contact-pair count, world
    tilt. Kept only so v1 checkpoints replay; nothing new trains on it."""
    version = 1
    dim = OBS_DIMS[("recover", 1)]

    def __init__(self, model, torso, jadr, vadr, fids):
        self.model, self.torso = model, int(torso)
        self.jadr, self.vadr, self.fids = list(jadr), list(vadr), list(fids)

    def reset(self, rng=None, noise=None, q_offset=None):
        pass

    def encoder(self, data):
        return np.asarray(data.qpos[self.jadr], float).copy()

    def __call__(self, data, t, prev_action=None):
        d = data
        R = d.xmat[self.torso].reshape(3, 3)
        grav = R.T @ np.array([0, 0, -1.0])
        w = R.T @ d.cvel[self.torso][0:3]
        tilt = float(np.arccos(np.clip(R[2, 2], -1, 1)))
        h = float(d.xpos[self.torso][2])
        feet = legacy_feet_count(d, self.fids)
        self.last = dict(tilt=tilt, feet_n=feet)
        return np.concatenate([grav, w, d.qpos[self.jadr], d.qvel[self.vadr],
                               [h / 0.15, feet / N_LEGS, tilt / np.pi]]).astype(np.float32)


def make_recover_obs(version, model, torso, jadr, vadr, fids):
    if version == 2:
        return RecoverObs(model, torso, jadr, vadr, fids)
    if version == 1:
        return LegacyRecoverObs(model, torso, jadr, vadr, fids)
    raise ValueError(f"recover obs version {version!r} is not one this code can build "
                     f"(known: 1 legacy, 2 current)")


class GaitObs:
    """Gait obs v2 (56): grav gyro q qd (from Sensors) + phase sin/cos + command
    (vx/60, vy/60, wz/0.6) + the filtered residual a_prev (15)."""
    version = 2
    dim = OBS_DIMS[("gait", 2)]

    def __init__(self, model, torso, jadr, vadr, fids):
        self.sensors = Sensors(model, torso, jadr, fids)

    def reset(self, rng=None, noise=None, q_offset=None):
        self.sensors.reset(rng=rng, noise=noise, q_offset=q_offset)

    def __call__(self, data, t, phase, cmd_n, prev_action):
        s = self.sensors.sample(data, t)
        self.last = s
        return np.concatenate([s["grav"], s["gyro"], s["q"], s["qd"],
                               [np.sin(phase), np.cos(phase)], cmd_n,
                               np.asarray(prev_action, float)]).astype(np.float32)


class LegacyGaitObs:
    """Gait obs v1 (41), the pre-D052 build (true state, world-z gravity)."""
    version = 1
    dim = OBS_DIMS[("gait", 1)]

    def __init__(self, model, torso, jadr, vadr, fids):
        self.torso, self.jadr, self.vadr = int(torso), list(jadr), list(vadr)

    def reset(self, rng=None, noise=None, q_offset=None):
        pass

    def __call__(self, data, t, phase, cmd_n, prev_action=None):
        R = data.xmat[self.torso].reshape(3, 3)
        grav = R.T @ np.array([0, 0, -1.0])
        w = R.T @ data.cvel[self.torso][0:3]
        return np.concatenate([grav, w, data.qpos[self.jadr], data.qvel[self.vadr],
                               [np.sin(phase), np.cos(phase)], cmd_n]).astype(np.float32)


def make_gait_obs(version, model, torso, jadr, vadr, fids):
    if version == 2:
        return GaitObs(model, torso, jadr, vadr, fids)
    if version == 1:
        return LegacyGaitObs(model, torso, jadr, vadr, fids)
    raise ValueError(f"gait obs version {version!r} is not one this code can build (known: 1, 2)")


# ---------------------------------------------------------------- servo draw
def servo_params(mode, rng=None):
    """Per-episode servo settings for ServoModel.set() plus the voltage factor.
    off = ideal actuators (v1 training); nominal = the datasheet servo (20 ms,
    4.7 rad/s, full voltage); random = the SERVO_RANGES draw."""
    if mode not in SERVO_MODES:
        raise ValueError(f"servo mode must be one of {SERVO_MODES}, not {mode!r}")
    base = dict(hold_hz=rm.bus_hz(), quant=True)
    if mode == "off":
        return dict(base, on=False, latency_s=rm.latency_s(), rate_rad_s=SERVO_NO_LOAD_RAD_S, voltage=1.0)
    if mode == "nominal":
        return dict(base, on=True, latency_s=rm.latency_s(), rate_rad_s=SERVO_NO_LOAD_RAD_S, voltage=1.0)
    rng = rng if rng is not None else np.random.default_rng()
    v = float(rng.uniform(*SERVO_RANGES["voltage"]))
    return dict(base, on=True, latency_s=float(rng.uniform(*SERVO_RANGES["latency_s"])),
                rate_rad_s=float(rng.uniform(*SERVO_RANGES["rate_rad_s"])) * v, voltage=v)


def make_servo(n=N_JOINTS):
    from servo_model import ServoModel
    return ServoModel(n=n, on=True)


def apply_servo_params(servo, sp):
    servo.set(on=sp["on"], hold_hz=sp["hold_hz"], latency_s=sp["latency_s"],
              rate_rad_s=sp["rate_rad_s"], quant=sp["quant"])


# ---------------------------------------------------------------- domain randomisation
class DomainRandomizer:
    """Per-episode DR from stored BASE values (never compounding).

    friction   one sliding-mu draw for the foot/floor pair: every geom's mu is
               scaled by draw / floor base, so relative differences survive
               and MuJoCo's max() rule can't hide the draw behind the robot's
               own 0.8
    link mass  per body, mass and inertia together
    torso CoM  body_ipos shift in x/y — the simplest robust option: nothing
               about the geoms or contacts moves, only where the weight sits
    kp         per actuator, gainprm[:,0] and biasprm[:,1] together (a position
               servo is kp*(ctrl - q); scaling one alone would bias it)
    forcerange x the servo draw's voltage (correlated with the slew)
    offset     per joint, added between the servo output and ctrl (the env
               does that; the Sensors subtract it from the encoder)
    gravity    tilted up to grav_tilt_deg
    """

    def __init__(self, model, torso, n_act=N_JOINTS):
        self.model, self.torso, self.n_act = model, int(torso), n_act
        m = model
        self._base_friction = m.geom_friction[:, 0].copy()
        try:
            self._floor_mu = float(m.geom_friction[m.geom("floor").id, 0])
        except KeyError:
            self._floor_mu = float(rm.params()["leg"]["foot"]["mu_slide"])
        self._base_mass = m.body_mass.copy()
        self._base_inertia = m.body_inertia.copy()
        self._base_ipos = m.body_ipos.copy()
        self._base_gain = m.actuator_gainprm.copy()
        self._base_bias = m.actuator_biasprm.copy()
        self._base_frange = m.actuator_forcerange.copy()
        self._base_gravity = m.opt.gravity.copy()

    def nominal(self):
        return dict(mu=self._floor_mu, mass=np.ones(self.model.nbody), com_xy=np.zeros(2),
                    kp=np.ones(self.n_act), q_offset=np.zeros(self.n_act),
                    gravity=self._base_gravity.copy())

    def draw(self, rng):
        r = DR_RANGES
        ang = rng.uniform(0, np.deg2rad(r["grav_tilt_deg"]))
        az = rng.uniform(0, 2 * np.pi)
        gn = float(np.linalg.norm(self._base_gravity)) or 9.81
        return dict(mu=float(rng.uniform(*r["friction"])),
                    mass=rng.uniform(*r["link_mass"], self.model.nbody),
                    com_xy=rng.uniform(-r["com_xy_mm"], r["com_xy_mm"], 2) / 1000.0,
                    kp=rng.uniform(*r["kp"], self.n_act),
                    q_offset=np.deg2rad(rng.uniform(-r["joint_offset_deg"], r["joint_offset_deg"], self.n_act)),
                    gravity=gn * np.array([np.sin(ang) * np.cos(az), np.sin(ang) * np.sin(az), -np.cos(ang)]))

    def apply(self, draws, voltage=1.0):
        m, n = self.model, self.n_act
        m.geom_friction[:, 0] = self._base_friction * (draws["mu"] / self._floor_mu)
        m.body_mass[:] = self._base_mass * draws["mass"]
        m.body_inertia[:] = self._base_inertia * draws["mass"][:, None]
        m.body_ipos[:] = self._base_ipos
        m.body_ipos[self.torso, :2] += draws["com_xy"]
        m.actuator_gainprm[:] = self._base_gain
        m.actuator_biasprm[:] = self._base_bias
        m.actuator_gainprm[:n, 0] = self._base_gain[:n, 0] * draws["kp"]
        m.actuator_biasprm[:n, 1] = self._base_bias[:n, 1] * draws["kp"]
        m.actuator_forcerange[:] = self._base_frange
        m.actuator_forcerange[:n] = self._base_frange[:n] * float(voltage)
        m.opt.gravity[:] = draws["gravity"]

    def restore(self):
        self.apply(self.nominal(), 1.0)


def refresh_constants(model, data):
    """Recompute mass-derived constants (subtree masses, invweights) after DR
    changed masses; mj_setConst uses `data` as scratch, so call it BEFORE the
    env writes its initial state."""
    import mujoco
    mujoco.mj_setConst(model, data)


# ---------------------------------------------------------------- env config / checkpoints
def env_note(model):
    """Fingerprint + versions for env.config()."""
    import mujoco
    from model_fingerprint import robot_fingerprint, fingerprint_note
    return dict(robot_fingerprint=robot_fingerprint(model), fingerprint_note=fingerprint_note(model),
                mujoco=mujoco.__version__, params_rev=rm.params_rev())


def checkpoint_contract(ck, env_hint=None):
    """What a checkpoint (the dict torch.load returns) says about the env it
    trained in. D052 checkpoints carry env_config (the env's own config());
    older ones only carry CLI args, and are reconstructed as the legacy
    contract: obs v1, ideal servo, no action filter, 5 rad/s (recover).
    Adds 'legacy' and 'flags' (['legacy obs', 'exceeds servo', 'unbudgeted cmd'])."""
    cfg = ck.get("env_config")
    args = dict(ck.get("args") or {})
    if cfg:
        c = dict(cfg)
        c["legacy"] = False
    else:
        env = args.get("env") or env_hint or "gait"
        env = "gait" if env == "walk" else env
        c = dict(env=env, obs_version=1, obs_dim=int(ck.get("obs_dim", OBS_DIMS[(env, 1)])),
                 act_dim=int(ck.get("act_dim", N_JOINTS)), servo="off", ema_alpha=1.0,
                 reward=args.get("reward") or "v1", robot_fingerprint=None, legacy=True)
        if env == "recover":
            c["rate_limit_rad_s"] = float(args.get("rate_limit") or LEGACY_RATE_LIMIT_RAD_S)
        else:
            c["gait"] = dict(LEGACY_GAIT)
    flags = []
    if c.get("legacy") or int(c.get("obs_version", 1)) < OBS_VERSIONS.get(c["env"], 2):
        flags.append("legacy obs")
    rl = c.get("rate_limit_rad_s")
    if rl is not None and rl > SERVO_NO_LOAD_RAD_S + 1e-9:
        flags.append("exceeds servo")
    if c["env"] == "gait" and not c.get("cmd_budget"):
        flags.append("unbudgeted cmd")          # V2: trained on commands outside the gait's envelope
    c["flags"] = flags
    return c


def check_obs_contract(contract, env_name, known_dims=OBS_DIMS):
    """Raise ValueError unless this code can build the checkpoint's observation."""
    env = contract.get("env")
    if env != env_name:
        raise ValueError(f"checkpoint was trained on env {env!r}, not {env_name!r}")
    v = contract.get("obs_version")
    dim = contract.get("obs_dim")
    want = known_dims.get((env_name, v))
    if want is None:
        raise ValueError(f"checkpoint obs version {v!r} is unknown to this code for env {env_name!r} "
                         f"(known: {sorted(k[1] for k in known_dims if k[0] == env_name)}); "
                         f"it needs the code it was trained with")
    if dim is not None and int(dim) != want:
        raise ValueError(f"checkpoint obs dim {dim} does not match obs version {v} ({want} values) — "
                         f"the checkpoint and this code disagree about the observation")


def check_fingerprint(contract, model, who="checkpoint"):
    """Warn (never raise) when the checkpoint's robot differs from this model.
    Returns 'match', 'mismatch' or 'unknown' (legacy checkpoints carry none)."""
    fp = contract.get("robot_fingerprint")
    if not fp:
        return "unknown"
    from model_fingerprint import robot_fingerprint
    now = robot_fingerprint(model)
    if now != fp:
        warnings.warn(f"{who}: trained on robot {fp}, this model is {now} — the robot changed "
                      f"since training (D047/D052-style edits); re-evaluate before trusting it",
                      stacklevel=2)
        return "mismatch"
    return "match"


# ---------------------------------------------------------------- env factory (trainer)
def make_env(seed, env_name="gait", **kw):
    """Thunk for a vector env. kw go to the env constructor (unknown keys are
    ignored by the envs' **_)."""
    def thunk():
        import gymnasium as gym
        if env_name == "recover":
            from rocky_recover_env import RecoverEnv
            env = RecoverEnv(seed=seed, **kw)
        else:
            from rocky_env import PebbleEnv
            env = PebbleEnv(seed=seed, **kw)
        return gym.wrappers.RecordEpisodeStatistics(env)
    return thunk


def make_vec_env(env_fns, sync=False):
    """gymnasium >= 1.0 defaults to NEXT_STEP autoreset: the step after a done
    returns the reset obs with reward 0 and IGNORES the action — the CleanRL
    loop in train_ppo assumes the obs after a done is already the new
    episode's (SAME_STEP). Pinned here so both vector-env kinds agree."""
    import gymnasium as gym
    mode = gym.vector.AutoresetMode.SAME_STEP
    if sync:
        return gym.vector.SyncVectorEnv(env_fns, autoreset_mode=mode)
    return gym.vector.AsyncVectorEnv(env_fns, daemon=True, autoreset_mode=mode)


def episode_stats(infos):
    """(returns, lengths) of episodes that ended this vector step. SAME_STEP puts
    RecordEpisodeStatistics' 'episode' under final_info; NEXT_STEP at the top."""
    out_r, out_l = [], []
    for src in (infos.get("final_info"), infos):
        if not isinstance(src, dict) or "episode" not in src:
            continue
        m = np.asarray(src.get("_episode", np.ones(len(src["episode"]["r"]), bool)))
        out_r += list(np.asarray(src["episode"]["r"])[m])
        out_l += list(np.asarray(src["episode"]["l"])[m])
        break
    return out_r, out_l
