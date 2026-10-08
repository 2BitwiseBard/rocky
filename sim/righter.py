"""PolicyRighter — the learned self-righting policy as a ReflexSupervisor
`righter` callable (D042), factored out of the fallen demo
(experiments/run_reflex_fallen.py) so the playground can use it too (D048).

D052: RecoverEnv-faithful BY CONSTRUCTION. The observation comes from the
same builder object type the env uses (rl_common.RecoverObs for obs v2,
LegacyRecoverObs for v1 checkpoints) — before this the docstring claimed
"the same obs build" while the righter counted distinct feet > 0.3 N and
the env counted contact pairs. The controller-side action path is the
env's too: EMA filter (the checkpoint's ema_alpha), the joint-range map,
the command clamp at the checkpoint's rate limit, re-planned at 50 Hz and
held between ticks. The SERVO side (hold, latency, slew) is not here: it
is the robot's physics, and whoever steps the sim applies it (or not).

A checkpoint whose observation this code cannot build (unknown obs
version, or an obs dim that disagrees with its version) is REFUSED with a
ValueError; a robot-fingerprint mismatch only warns (the physics moved,
the policy may still work — evaluate it).

D064: the action map is the contract's. A checkpoint trained with the
righter's hip floor (env_config hip_floor_deg, legs 1-4) maps those hips
over [floor, 90] exactly as RecoverEnv did; an older one (no key: every
checkpoint on disk) keeps the uncut map it learned, and ReflexSupervisor's
FALLEN / RIGHTED clamp cuts its commands instead (the measured 0/200 form).
"""
from __future__ import annotations
import os
import sys
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
sys.path.insert(0, os.path.join(HERE, "..", "perception"))
sys.path.insert(0, HERE)
from pebble_gait import N_LEGS                                        # noqa: E402
from rocky_recover_env import (Q_LO, Q_HI, CTRL_DT, RATE_LIMIT_RAD_S,  # noqa: E402,F401
                               action_to_q, q_to_action, action_range, hip_floor)
import rl_common as rc                                                # noqa: E402

CONTACT_FORCE_N = 0.3     # foot switch stand-in threshold for the demos' supervisor feed


def foot_contacts(model, data, fids):
    """bool[5] feet on the WORLD (> 0.3 N), no hysteresis — the supervisor feed
    the fallen demo / audit_righter / shove_envelope / eval_recover use. D052: delegates to
    perception.contacts, so a foot pressed into the robot's own body no longer
    counts (the threshold stays 0.3 N so those demos keep their calibration)."""
    from contacts import foot_contacts as _fc
    return _fc(model, data, fids, close_n=CONTACT_FORCE_N)


def default_ckpt():
    """The shipped righter: sim/runs/recover8_d064 (2026-10-08, the first policy trained on the
    D064 model: belly, real torso inertial, hip floor), recover1 (D045) when it is absent. Add a
    newer run here only when an eval says it earned the handoff: recover8 did, 3 handoff exits in
    12 falls under the supervisor with the randomised servo (latency 10-40 ms, the realistic one;
    recover1 0 in 12), 12/20 raw, 0 belly contacts, pinned 0.31 vs recover5's 0.55; under the
    nominal (latency-free) servo it ends tilted and the 3 s stall ramp still stands it 20/20."""
    for name in ("recover8_d064", "recover1"):
        p = os.path.join(HERE, "runs", name, "latest.pt")
        if os.path.exists(p):
            return p
    return None


class PolicyRighter:
    """callable(t, dt) -> (5, 3) absolute joint targets, for ReflexSupervisor.

    PolicyRighter(ckpt, model, data, torso, fids) loads a checkpoint (torch).
    PolicyRighter(None, model, data, torso, fids, policy=fn, contract=dict)
    wraps any obs -> action callable under an explicit contract (tests, or a
    torch-free runtime); contract = env.config() or rl_common.checkpoint_contract(ck).
    """

    def __init__(self, ckpt, model, data, torso, fids, policy=None, contract=None):
        if policy is None:
            from eval_recover import load_full
            policy, ck, contract = load_full(ckpt)
            self.step_count = ck.get("global_step", 0)
        else:
            self.step_count = 0
            if contract is None:
                raise ValueError("PolicyRighter(policy=...) needs an explicit contract")
        contract = dict(contract)
        contract.setdefault("flags", [])
        rc.check_obs_contract(contract, "recover")          # refuses what it cannot build
        self.contract = contract
        self.policy = policy
        self.ckpt = ckpt
        self.model, self.data, self.torso, self.fids = model, data, int(torso), list(fids)
        self.fingerprint = rc.check_fingerprint(contract, model, who=f"righter {ckpt or 'policy'}")
        rate = contract.get("rate_limit_rad_s")
        self.rate_limit = float(RATE_LIMIT_RAD_S if rate is None else rate) * CTRL_DT
        self.ema_alpha = float(contract.get("ema_alpha", 1.0))
        self.obs_version = int(contract["obs_version"])
        # D064: the contract's floor (None: the uncut map); its legs, else the params set (1-4)
        self.hip_floor_deg, legs = hip_floor(contract.get("hip_floor_deg"), contract.get("hip_floor_legs"))
        self.q_lo, self.q_hi = action_range(self.hip_floor_deg, legs)
        self._jadr, self._vadr = rc.joint_addrs(model)
        # the builder wants the ROBOT's foot geoms: the caller's fids when given
        self.obs_builder = rc.make_recover_obs(self.obs_version, model, self.torso, self._jadr,
                                               self._vadr, self.fids or rc.foot_geoms(model))
        if "legacy obs" in contract["flags"] or "exceeds servo" in contract["flags"]:
            warnings.warn(f"righter {ckpt}: {', '.join(contract['flags'])} — a pre-D052 checkpoint, "
                          f"replayed with its own obs contract", stacklevel=2)
        self._target = None
        self._next_tick = None
        self._a_f = None
        self.last_obs = None

    def reset(self):
        """Forget the held target — called by the supervisor on FALLEN entry."""
        self._target = None
        self._next_tick = None
        self._a_f = None

    def _start(self, t):
        """First call this fall: read where the joints are and start from there
        (the env's reset does the same thing after landing)."""
        self.obs_builder.reset(noise=False)
        if self.obs_version == 1:
            self._target = self.data.qpos[self._jadr].copy()
            if self.hip_floor_deg is not None:
                self._target = np.clip(self._target, self.q_lo, self.q_hi)
        else:
            self._target = np.clip(self.obs_builder.encoder(self.data), self.q_lo, self.q_hi)
        self._a_f = q_to_action(self._target, self.q_lo, self.q_hi)
        self._next_tick = t

    def _obs(self, t=None):
        return self.obs_builder(self.data, t, self._a_f)

    def __call__(self, t, dt):
        if self._target is None:
            self._start(t)
        if t >= self._next_tick - 1e-9:                    # 50 Hz re-plan
            obs = self._obs(t)
            self.last_obs = obs
            a = np.clip(self.policy(obs.astype(np.float32)), -1, 1)
            self._a_f = self.ema_alpha * a + (1.0 - self.ema_alpha) * self._a_f
            want = action_to_q(self._a_f, self.q_lo, self.q_hi)
            self._target += np.clip(want - self._target, -self.rate_limit, self.rate_limit)
            self._next_tick = t + CTRL_DT
        return self._target.reshape(N_LEGS, 3)
