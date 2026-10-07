"""D064 (pick 13, B97, B130, B138): the belly in the sim, the torso's real mass
distribution, and the righter's hip floor where the policy is trained.

The tub's sums are cad/iface.bay_tub_extent()'s; the sim cannot import it
(build123d), so this re-derives them from params at test time and pins the
values the owner's picks give today.
"""
import json
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
for sub in ("sim", "gait", "perception"):
    sys.path.insert(0, os.path.join(REPO, sub))
mujoco = pytest.importorskip("mujoco")

import rocky_model as rm                                               # noqa: E402

XML = os.path.join(REPO, "sim", "pebble.xml")
BUDGET = os.path.join(REPO, "sim", "mass_budget.json")


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(XML)


def flat(spans):
    return [v for lo_hi in spans for v in lo_hi]


# ------------------------------------------------------------------ the boxes, from params
def test_belly_tub_is_the_params_sum():
    bs = rm.params()["interfaces"]["battery_sled"]
    x0 = bs["bay_door_x"]
    x1 = bs["bay_door_x"] + bs["door_t"] + bs["bay_l"] + bs["bay_wall"]
    yh = bs["bay_w"] / 2 + bs["bay_wall"]
    z0 = bs["bay_top_z"] - bs["bay_roof"] - bs["bay_h"] - bs["bay_floor"]
    tub = rm.belly_boxes()["belly_tub"]
    assert flat(tub) == pytest.approx([x0, x1, bs["bay_centre"][1] - yh, bs["bay_centre"][1] + yh,
                                       z0, bs["bay_top_z"]])
    # the owner's picks today: the door end pinned at -97.5 (bay_l moves the nose, pick 4), y -20
    # (bay_w 50, pick 8b), roof top -18 (pick 2); the nose is whatever bay_l says now
    assert tub[0][0] == pytest.approx(-97.5) and tub[0][1] == pytest.approx(-97.5 + 3.0 + bs["bay_l"] + 2.4)
    assert list(tub[1]) == pytest.approx([-47.4, 7.4])
    assert list(tub[2]) == pytest.approx([-55.4, -18.0])
    assert rm.bay_tub_extent_mm() == {"x": tub[0], "y": tub[1], "z": tub[2]}


def test_belly_shelf_is_the_params_envelope_grown_to_the_post_pads():
    hs = rm.params()["interfaces"]["hub_shelf"]
    r = hs["post_d"] / 2 + rm.SHELF_PAD_WALL_MM            # the Ø14 pads
    want_x = (min(hs["x"][0], *[p[0] - r for p in hs["posts"]]), max(hs["x"][1], *[p[0] + r for p in hs["posts"]]))
    want_y = (min(hs["y"][0], *[p[1] - r for p in hs["posts"]]), max(hs["y"][1], *[p[1] + r for p in hs["posts"]]))
    shelf = rm.belly_boxes()["belly_shelf"]
    assert flat(shelf) == pytest.approx([*want_x, *want_y, hs["bottom_z"], -10.0])
    assert flat(shelf) == pytest.approx([-57.0, 57.0, 12.2, 71.0, -54.4, -10.0])
    # the tub and the shelf do not overlap (the shelf is north of the tub's 7.4)
    assert shelf[1][0] > rm.belly_boxes()["belly_tub"][1][1]


def test_mjcf_belly_geoms_are_the_boxes(model):
    t = model.body("torso").id
    floor = model.geom("floor").id
    for name, spans in rm.belly_boxes().items():
        g = model.geom(name)
        assert model.geom_bodyid[g.id] == t
        assert model.geom_type[g.id] == mujoco.mjtGeom.mjGEOM_BOX
        c = [(a + b) / 2 / 1000 for a, b in spans]
        h = [(b - a) / 2 / 1000 for a, b in spans]
        assert np.allclose(model.geom_pos[g.id], c, atol=1e-8) and np.allclose(model.geom_size[g.id], h, atol=1e-8)
        assert np.allclose(model.geom_quat[g.id], [1, 0, 0, 0])
        # default contact: it collides with the floor and with the legs
        assert model.geom_contype[g.id] == 1 and model.geom_conaffinity[g.id] == 1
        assert model.geom_condim[g.id] == 3
        assert np.allclose(model.geom_friction[g.id], model.geom_friction[floor])


# ------------------------------------------------------------------ the torso's inertial
def test_torso_inertial_is_the_budget(model):
    mb = json.load(open(BUDGET))
    assert {"torso_com_mm", "torso_inertia"} <= set(mb)
    t = model.body("torso").id
    assert model.body_mass[t] * 1000 == pytest.approx(mb["torso"], abs=0.05)
    assert np.allclose(model.body_ipos[t] * 1000, mb["torso_com_mm"], atol=1e-4)
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, model.body_iquat[t])
    R = R.reshape(3, 3)
    I = R @ np.diag(model.body_inertia[t]) @ R.T
    ixx, iyy, izz, ixy, ixz, iyz = mb["torso_inertia"]
    want = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    assert np.allclose(I, want, rtol=1e-5, atol=1e-10)
    # the CoM is where option A puts it: under the deck top, south of centre (the pack at y -20),
    # far below the cylinders' 24.5 (D063); the robot total stays the budget's
    assert mb["torso_com_mm"][2] < 0.0 and mb["torso_com_mm"][1] < 0.0
    total = mb["torso"] + 5 * (mb["coxa"] + mb["femur"] + mb["tibia"])
    assert model.body_subtreemass[t] * 1000 == pytest.approx(total, abs=2.0)
    assert rm.torso_inertial()["com_mm"] == pytest.approx(tuple(mb["torso_com_mm"]))


def test_the_budget_says_whether_it_is_provisional():
    mb = json.load(open(BUDGET))
    assert isinstance(mb.get("provisional"), bool)
    if mb["provisional"]:
        assert mb["provisional_items"], "a provisional budget names what was estimated"


def test_belly_clearance_at_stance(model):
    """The settled robot stands on its feet: the tub's bottom ~62 mm over the floor (the prep's
    62.13 with the mass), nothing touches the belly."""
    from pebble_gait import WaveGait, leg_ik, body_to_leg
    g = WaveGait(**rm.gait_defaults())
    q0 = np.array([leg_ik(body_to_leg(i, g.p_nom[i])) for i in range(5)]).ravel()
    d = mujoco.MjData(model)
    jadr = [model.joint(f"{n}{i}").qposadr[0] for i in range(5) for n in rm.LEG_JOINTS]
    d.qpos[0:3] = [0, 0, rm.spawn_z_m()]
    d.qpos[3] = 1.0
    d.qpos[jadr] = q0
    d.ctrl[:15] = q0
    for _ in range(int(1.0 / model.opt.timestep)):
        mujoco.mj_step(model, d)
    tub = model.geom("belly_tub").id
    clear = (d.geom_xpos[tub][2] - model.geom_size[tub][2]) * 1000
    assert 58.0 < clear < 66.0, clear
    belly = {model.geom(n).id for n in rm.belly_boxes()}
    assert not [c for c in d.contact[:d.ncon] if c.geom1 in belly or c.geom2 in belly]


# ------------------------------------------------------------------ the recover env
def test_domain_randomiser_bases_carry_the_belly_mass():
    """B138: the DR restores mass / inertia / ipos from copies of the COMPILED model, so a retrain
    sees the torso's <inertial>, not the cylinders'. Read back after reset."""
    from rocky_recover_env import RecoverEnv
    mb = json.load(open(BUDGET))
    env = RecoverEnv(seed=0)
    t = env.torso
    assert env.dr._base_mass[t] * 1000 == pytest.approx(mb["torso"], abs=0.05)
    assert np.allclose(env.dr._base_ipos[t] * 1000, mb["torso_com_mm"], atol=1e-4)
    env.reset(seed=1)
    assert env.model.body_mass[t] * 1000 == pytest.approx(mb["torso"], abs=0.05)
    assert np.allclose(env.model.body_ipos[t] * 1000, mb["torso_com_mm"], atol=1e-4)
    assert np.allclose(env.model.body_inertia[t], env.dr._base_inertia[t])
    rnd = RecoverEnv(seed=0, randomize=True)
    rnd.reset(seed=3)
    k = rnd.dr_draw["mass"][t]
    assert rnd.model.body_mass[t] == pytest.approx(rnd.dr._base_mass[t] * k)
    assert np.allclose(rnd.model.body_ipos[t, :2], rnd.dr._base_ipos[t, :2] + rnd.dr_draw["com_xy"])
    assert rnd.model.body_ipos[t, 2] == pytest.approx(mb["torso_com_mm"][2] / 1000, abs=1e-7)


def test_recover_env_maps_the_righter_hip_floor():
    """'action range remapped' (PREP_REPORT_2 s2 (d)): legs 1-4's hip spans [-51.05, 90], leg 0's
    keeps -70; the contract records it; the uncut map is still there for old checkpoints."""
    from rocky_recover_env import RecoverEnv, action_to_q, Q_LO, Q_HI
    env = RecoverEnv(seed=0)
    floor = rm.righter_hip_min_deg()
    assert floor == pytest.approx(-51.05)
    assert env.hip_floor_deg == pytest.approx(floor) and env.hip_floor_legs == (1, 2, 3, 4)
    q = action_to_q(-np.ones(15), env.q_lo, env.q_hi).reshape(5, 3)
    assert np.degrees(q[0, 1]) == pytest.approx(-70.0)
    assert np.degrees(q[1:, 1]) == pytest.approx([floor] * 4)
    assert np.allclose(action_to_q(np.ones(15), env.q_lo, env.q_hi), Q_HI)
    assert np.allclose(np.delete(env.q_lo, [4, 7, 10, 13]), np.delete(Q_LO, [4, 7, 10, 13]))
    cfg = env.config()
    assert cfg["hip_floor_deg"] == pytest.approx(floor) and cfg["hip_floor_legs"] == [1, 2, 3, 4]
    # a landed pose below the floor: the first target already sits on it
    env.reset(seed=5)
    assert (env._target.reshape(5, 3)[1:, 1] >= np.deg2rad(floor) - 1e-12).all()
    for _ in range(5):
        env.step(-np.ones(15))
    assert (env._target.reshape(5, 3)[1:, 1] >= np.deg2rad(floor) - 1e-12).all()
    old = RecoverEnv(seed=0, hip_clamp=None)
    assert old.hip_floor_deg is None and np.allclose(old.q_lo, Q_LO)
    assert old.config()["hip_floor_deg"] is None


def test_eval_replays_the_contract_map_and_the_righter_agrees():
    import eval_recover as er
    from righter import PolicyRighter
    import rl_common as rc
    legacy = rc.checkpoint_contract({"args": {"env": "recover"}}, env_hint="recover")
    assert er.env_for(legacy).hip_floor_deg is None              # every checkpoint on disk: the uncut map
    env = er.env_for(dict(RecoverEnvConfig(), obs_version=2))
    assert env.hip_floor_deg == pytest.approx(rm.righter_hip_min_deg())
    env.reset(seed=2)
    r = PolicyRighter(None, env.model, env.data, env.torso, env._foot_gid,
                      policy=lambda o: -np.ones(15), contract=env.config())
    assert np.allclose(r.q_lo, env.q_lo) and np.allclose(r.q_hi, env.q_hi)
    q = r(0.0, 0.0)
    for k in range(1, 40):
        q = r(k * 0.02, 0.02)
    assert (np.asarray(q)[1:, 1] >= np.deg2rad(rm.righter_hip_min_deg()) - 1e-12).all()


def RecoverEnvConfig():
    from rocky_recover_env import RecoverEnv
    return RecoverEnv(seed=0).config()


# ------------------------------------------------------------------ the system path
def test_supervisor_fallen_hips_stay_on_the_floor_in_the_sim():
    """One fall through the real stack (ReflexSupervisor + a policy pinned at the bottom of its
    range on the UNCUT map, as a legacy checkpoint would be; seed 2 lands on its back): FALLEN /
    RIGHTED never command legs 1-4's hips below the clamp (leg 0 keeps -70: it is not in the
    set); the run reports its belly contacts."""
    import eval_recover as er
    import rl_common as rc
    legacy = rc.checkpoint_contract({"args": {"env": "recover"}}, env_hint="recover")
    env = er.env_for(legacy)
    out = er.run_supervisor(env, lambda o: -np.ones(15), legacy, seed=2, seconds=4.0)
    assert out["fallen"]
    assert out["hip_cmd_min_deg"] == pytest.approx(rm.righter_hip_min_deg(), abs=1e-6)
    assert set(out["belly"]) == {"steps", "max_mm", "legs", "states"}
