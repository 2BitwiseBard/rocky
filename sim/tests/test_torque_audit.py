"""B106: the torque audit weighs the robot the MJCF weighs.

The tibia budget in sim/mass_budget.json already includes the hand
(mass_audit; build_mjcf splits it without adding mass). The audit used to
add the 5 hands again (2920.7 g against the MJCF's 2727.7 g) and put
tibia + hand at mid-shin on an airborne leg.
"""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "sim"))
mujoco = pytest.importorskip("mujoco")

import torque_audit as ta                                              # noqa: E402

XML = os.path.join(REPO, "sim", "pebble.xml")
RECORD = os.path.join(REPO, "sim", "out", "torque_audit.json")


def test_audit_weighs_the_mjcf_robot():
    model = mujoco.MjModel.from_xml_path(XML)
    compiled_g = model.body_subtreemass[model.body("torso").id] * 1000
    m = ta.load_masses()
    assert m["robot"] * 1000 == pytest.approx(compiled_g, abs=2.0)
    with open(RECORD) as f:                    # the committed record agrees
        rec = json.load(f)
    assert rec["masses_g"]["robot"] == pytest.approx(m["robot"] * 1000, abs=0.1)


def test_airborne_leg_hangs_the_hand_at_the_tip():
    m = ta.load_masses()
    hand, tibia = m["hand"], m["tibia"]
    assert 0 < hand < tibia
    th, tk = ta.cantilever_torques(m, extended=True)
    L2, L3 = ta.L2, ta.L3
    hip = ta.G * (m["femur"] * L2 / 2 + (tibia - hand) * (L2 + L3 / 2)
                  + hand * (L2 + L3)) / 1000.0
    knee = ta.G * ((tibia - hand) * L3 / 2 + hand * L3) / 1000.0
    assert th == pytest.approx(hip) and tk == pytest.approx(knee)
    # the payload hangs where the hand does
    th2, tk2 = ta.cantilever_torques(m, extended=True, payload_kg=0.1)
    assert th2 - th == pytest.approx(ta.G * 0.1 * (L2 + L3) / 1000.0)
    assert tk2 - tk == pytest.approx(ta.G * 0.1 * L3 / 1000.0)


def test_airborne_leg_moments_match_the_mjcf():
    """The extended leg's gravity torques equal the compiled MJCF's: subtree
    mass x the subtree COM along the link, femur and tibia straight (qpos0).
    Measured 2026-09-30: hip 0.254 / knee 0.112 N.m both ways (< 0.3 %); the
    pre-B106 hip (0.290, tibia + hand at mid-shin) was 14 % over."""
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def moment(body):                          # N.m about the body's joint
        b = model.body(body).id
        R = data.xmat[b].reshape(3, 3)
        x = (R.T @ (data.subtree_com[b] - data.xpos[b]))[0]
        return ta.G * model.body_subtreemass[b] * x

    th, tk = ta.cantilever_torques(ta.load_masses(), extended=True)
    assert th == pytest.approx(moment("femur0"), rel=0.02)
    assert tk == pytest.approx(moment("tibia0"), rel=0.02)
