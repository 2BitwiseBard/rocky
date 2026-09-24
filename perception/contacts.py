"""Foot contact — ONE function for everyone (D052).

Before this there were five copies of "is foot i touching?" (run_odom,
sim_lidar, righter, run_push_reflex_v2, ...) with three thresholds (0.3,
1.5 N, ...) and one shared bug: they summed EVERY contact on the foot geom,
so a foot pressed into the robot's own body or another leg counted as
ground. The real sensor is a microswitch in the foot cartridge (SEA, KW10):
it closes at an operating force, opens again at a lower release force, and
it only feels what the foot stands on.

    fids = [model.geom(f"foot{i}").id for i in range(5)]
    state = np.zeros(5, bool)                      # optional: hysteresis memory
    con = foot_contacts(model, data, fids, state)  # bool[5]
    F = foot_forces(model, data, fids)             # N, normal force from the world

Thresholds default to params `sensing.foot_switch` (close 2.0 N, open 1.0 N,
both VERIFY on the bench). World = anything not in the robot's own kinematic
tree: the floor, static obstacles, AND free world objects (a foot on the
cockpit's ball is a foot on something).
"""
from __future__ import annotations
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gait"))
import rocky_model as _rm                                              # noqa: E402

CLOSE_N, OPEN_N = _rm.foot_switch()


def _robot_root(model, fids):
    """Root body of the robot's tree (the free torso) — a foot's root."""
    return int(model.body_rootid[model.geom_bodyid[fids[0]]])


def foot_forces(model, data, fids) -> np.ndarray:
    """Normal force (N) on each foot from NON-robot geoms, summed over contacts."""
    import mujoco
    fids = [int(g) for g in fids]
    out = np.zeros(len(fids))
    if data.ncon == 0:
        return out
    root = _robot_root(model, fids)
    slot = {g: i for i, g in enumerate(fids)}
    rootid = model.body_rootid
    gbody = model.geom_bodyid
    F = np.zeros(6)
    for c in range(data.ncon):
        con = data.contact[c]
        g1, g2 = int(con.geom1), int(con.geom2)
        if g1 in slot:
            i, other = slot[g1], g2
        elif g2 in slot:
            i, other = slot[g2], g1
        else:
            continue
        if rootid[gbody[other]] == root:          # robot-robot: a foot on its own body
            continue
        mujoco.mj_contactForce(model, data, c, F)
        out[i] += F[0]
    return out


def foot_contacts(model, data, fids, state=None, close_n=None, open_n=None) -> np.ndarray:
    """bool[len(fids)]: which feet the switch says are down.

    state: optional bool array (same length), updated IN PLACE — gives the
    switch its hysteresis (closed stays closed until the force drops below
    open_n). Without it, a plain `force > close_n` threshold.
    close_n / open_n: override the params thresholds (the old 1.5 N callers)."""
    close_n = CLOSE_N if close_n is None else float(close_n)
    open_n = min(OPEN_N if open_n is None else float(open_n), close_n)
    f = foot_forces(model, data, fids)
    if state is None:
        return f > close_n
    st = np.asarray(state, dtype=bool)
    new = np.where(st, f > open_n, f > close_n)
    state[:] = new
    return new.copy()
