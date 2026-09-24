"""Robot fingerprint (D052): a short hash of the ROBOT a MuJoCo model contains.

A policy checkpoint trained against one robot silently degrades on another
(D047 changed the leg, D052 changed damping, forcerange, friction, the foot
and 5 g of mass). The RL agent stores robot_fingerprint(model) in every
checkpoint and warns when the model it is loaded against hashes differently.

It hashes the robot ONLY — the torso's kinematic tree: body masses, inertias
and frames, geom types/sizes/frames/friction/condim, joint axes/ranges/
damping/armature, and the actuators that drive those joints (gain, bias,
force and ctrl ranges). NOT the world: the cockpit swaps worlds in place
(floor friction, obstacles, a ball, gravity tilt) and the robot is the same
robot. NOT the timestep or solver either (those are the sim, not the robot;
fingerprint_note records the MuJoCo version for that).

    fp = robot_fingerprint(model)     # '3f9a0c...' (12 hex)
    fingerprint_note(model)           # 'robot 3f9a.. | mujoco 3.x | params 0.1/D052'
"""
from __future__ import annotations
import hashlib
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gait"))
import rocky_model as _rm                                              # noqa: E402

DECIMALS = 9            # compiled floats are deterministic; round away last-bit noise anyway


def _robot_bodies(model, root_name="torso"):
    root = int(model.body(root_name).id)
    rid = int(model.body_rootid[root])
    return [b for b in range(model.nbody) if int(model.body_rootid[b]) == rid]


def _feed(h, arr):
    a = np.round(np.asarray(arr, dtype=np.float64), DECIMALS) + 0.0   # + 0.0 folds -0.0
    h.update(np.ascontiguousarray(a).tobytes())


def robot_fingerprint(model, root_name: str = "torso", n_hex: int = 12) -> str:
    """12-hex hash of the robot subtree (see module docstring for what counts)."""
    import mujoco
    h = hashlib.sha256()
    bodies = _robot_bodies(model, root_name)
    bset = set(bodies)
    for b in bodies:
        h.update(model.body(b).name.encode())
        for f in (model.body_mass, model.body_inertia, model.body_pos, model.body_quat,
                  model.body_ipos, model.body_iquat):
            _feed(h, f[b])
    for g in range(model.ngeom):
        if int(model.geom_bodyid[g]) not in bset:
            continue
        _feed(h, [model.geom_type[g], model.geom_condim[g], model.geom_contype[g],
                  model.geom_conaffinity[g]])
        for f in (model.geom_size, model.geom_pos, model.geom_quat, model.geom_friction):
            _feed(h, f[g])
    jset = set()
    for j in range(model.njnt):
        if int(model.jnt_bodyid[j]) not in bset:
            continue
        jset.add(j)
        h.update(model.joint(j).name.encode())
        _feed(h, [model.jnt_type[j], model.jnt_limited[j]])
        _feed(h, model.jnt_range[j])
        _feed(h, model.jnt_axis[j])
        d = int(model.jnt_dofadr[j])
        _feed(h, [model.dof_damping[d], model.dof_armature[d], model.dof_frictionloss[d]])
    for a in range(model.nu):
        if int(model.actuator_trntype[a]) != int(mujoco.mjtTrn.mjTRN_JOINT):
            continue
        if int(model.actuator_trnid[a, 0]) not in jset:
            continue
        h.update(model.actuator(a).name.encode())
        for f in (model.actuator_gainprm, model.actuator_biasprm, model.actuator_forcerange,
                  model.actuator_ctrlrange, model.actuator_gear):
            _feed(h, f[a])
        _feed(h, [model.actuator_forcelimited[a], model.actuator_ctrllimited[a]])
    return h.hexdigest()[:n_hex]


def fingerprint_note(model=None) -> str:
    """Human line for checkpoints / logs: robot hash, MuJoCo version, params revision."""
    import mujoco
    parts = []
    if model is not None:
        parts.append(f"robot {robot_fingerprint(model)}")
    parts.append(f"mujoco {mujoco.__version__}")
    parts.append(f"params {_rm.params_rev()}")
    return " | ".join(parts)


if __name__ == "__main__":
    import mujoco
    m = mujoco.MjModel.from_xml_path(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  "pebble.xml"))
    print(fingerprint_note(m))
