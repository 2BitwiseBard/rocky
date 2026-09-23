"""PolicyRighter — the learned self-righting policy as a ReflexSupervisor
`righter` callable (D042), factored out of run_reflex_fallen.py so the
playground can use it too (D048).

RecoverEnv-faithful: the same obs build + normalization, the same rate
limit the checkpoint was TRAINED with (read back from the checkpoint's
args; 5 rad/s for the v1/v2 runs, whatever `--rate-limit` said for v3),
re-planned at 50 Hz and held between control ticks. The policy must see
the dynamics it trained on — an adapter that smooths or re-times its
output is a different controller and needs its own evaluation.
"""
from __future__ import annotations
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import N_LEGS                                        # noqa: E402
from rocky_recover_env import Q_LO, Q_HI, CTRL_DT, RATE_LIMIT_RAD_S   # noqa: E402

CONTACT_FORCE_N = 0.3     # foot switch stand-in threshold (same as run_odom)


def foot_contacts(model, data, fids):
    import mujoco
    out = np.zeros(N_LEGS, dtype=bool)
    for i, gid in enumerate(fids):
        f = 0.0
        for c in range(data.ncon):
            con = data.contact[c]
            if gid in (con.geom1, con.geom2):
                F = np.zeros(6)
                mujoco.mj_contactForce(model, data, c, F)
                f += F[0]
        out[i] = f > CONTACT_FORCE_N
    return out


def default_ckpt():
    """The shipped righter: the first of these that exists.
    recover1 FIRST (D045): the v2/3.67M recover2 checkpoint regressed the
    deterministic handoff (2/20 vs recover1's 12/20). Re-order only when
    an eval says a newer checkpoint earned it."""
    for name in ("recover1", "recover2"):
        p = os.path.join(HERE, "runs", name, "latest.pt")
        if os.path.exists(p):
            return p
    return None


class PolicyRighter:
    def __init__(self, ckpt, model, data, torso, fids):
        from eval_recover import load, ckpt_args
        self.policy, self.step_count = load(ckpt)
        meta = ckpt_args(ckpt)
        self.rate_limit = float(meta.get("rate_limit", RATE_LIMIT_RAD_S)) * CTRL_DT
        self.ckpt = ckpt
        self.model, self.data, self.torso, self.fids = model, data, torso, fids
        self._jadr = [model.joint(f"{n}{i}").qposadr[0]
                      for i in range(5) for n in ("yaw", "hip", "knee")]
        self._vadr = [model.joint(f"{n}{i}").dofadr[0]
                      for i in range(5) for n in ("yaw", "hip", "knee")]
        self._target = None
        self._next_tick = None

    def reset(self):
        """Forget the held target — called by the supervisor on FALLEN entry."""
        self._target = None
        self._next_tick = None

    def _obs(self):
        d = self.data
        R = d.xmat[self.torso].reshape(3, 3)
        grav = R.T @ np.array([0, 0, -1.0])
        w = R.T @ d.cvel[self.torso][0:3]
        qpos = d.qpos[self._jadr]
        qvel = d.qvel[self._vadr]
        feet = float(foot_contacts(self.model, d, self.fids).sum())
        tilt = float(np.arccos(np.clip(R[2, 2], -1, 1)))
        h = float(d.xpos[self.torso][2])
        return np.concatenate([grav, w, qpos, qvel,
                               [h / 0.15, feet / N_LEGS, tilt / np.pi]])

    def __call__(self, t, dt):
        if self._target is None:                       # first call this fall
            self._target = self.data.qpos[self._jadr].copy()
            self._next_tick = t
        if t >= self._next_tick:                       # 50 Hz re-plan
            a = np.clip(self.policy(self._obs().astype(np.float32)), -1, 1)
            want = Q_LO + (a + 1) / 2 * (Q_HI - Q_LO)
            self._target += np.clip(want - self._target, -self.rate_limit, self.rate_limit)
            self._next_tick = t + CTRL_DT
        return self._target.reshape(N_LEGS, 3)
