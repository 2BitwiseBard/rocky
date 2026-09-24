"""world_builder.py: every preset builds, objects/terrain/conditions land in
the model, the eye camera exists (needs mujoco + sim/pebble.xml). D052: the
world's friction is what the feet feel, ramps are not canted or floating,
ramps/stairs have a far side, random courses keep the spawn clear."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "gait"))
mujoco = pytest.importorskip("mujoco")
pytestmark = pytest.mark.skipif(not os.path.exists(os.path.join(HERE, "..", "pebble.xml")),
                                reason="run sim/build_mjcf.py first")
from world_builder import (build, PRESETS, KINDS, world_xml, random_course, footprint,    # noqa: E402
                           footprint_dist, saved_worlds, load_world, CLEAR_M)


def test_presets_build_with_an_eye():
    for name, spec in PRESETS.items():
        m, z = build(spec)
        assert m.ncam == 1 and m.camera("eye").id >= 0, name
        assert m.nbody >= 22, name
        assert z == (0.16 if spec.get("base") == "cliff" else 0.0), name


def test_every_object_kind_builds_and_is_lidar_visible():
    spec = {"base": "flat", "objects": [{"kind": k, "pos": [0.6 + 0.3 * i, 0.0]} for i, k in enumerate(KINDS)]}
    m, _ = build(spec)
    base, _ = build({"base": "flat"})
    assert m.ngeom > base.ngeom + len(KINDS)          # rubble/stairs add several each
    extra = [g for g in range(m.ngeom) if m.geom_group[g] == 3]
    assert len(extra) == m.ngeom - base.ngeom          # every added geom is group 3
    assert m.nbody == base.nbody + 1                   # the ball is a free body
    # the robot's free joint stays first in qpos (every sim entry point assumes it)
    assert m.jnt_qposadr[m.body("torso").jntadr[0]] == 0
    ball = m.body("obj5")
    assert m.jnt_qposadr[ball.jntadr[0]] == 7 + 20     # after the torso's 7 + 15 joints + 5 claws


def test_world_geoms_win_the_contact_and_robot_geoms_do_not():
    for name in ("room", "cliff", "obstacle course", "rough terrain", "stairs"):
        m, _ = build(PRESETS[name])
        torso = m.body("torso").id
        for g in range(m.ngeom):
            robot = m.body_rootid[m.geom_bodyid[g]] == torso
            want = 0 if robot else 1
            assert m.geom_priority[g] == want, (name, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g))
            if not robot:
                assert m.geom_condim[g] == 4, name     # keeps the feet's torsional friction


def test_terrain_heightfield_and_conditions():
    m, _ = build({"base": "flat", "friction": 0.5, "gravity_tilt_deg": 8.0,
                  "terrain": {"kind": "rough", "amp_m": 0.03, "size_m": 1.0, "cell_m": 0.05, "seed": 3}})
    assert m.nhfield == 1
    d = m.hfield_data
    assert d.min() >= 0.0 and d.max() <= 1.0 and d.std() > 0.05
    for gname in ("floor", "terrain"):                 # (the contact-level proof is the next test)
        assert m.geom_friction[m.geom(gname).id, 0] == pytest.approx(0.5)
        assert m.geom_priority[m.geom(gname).id] == 1
    g = m.opt.gravity
    assert np.hypot(g[0], g[1]) == pytest.approx(9.81 * np.sin(np.radians(8)), rel=1e-3)
    assert g[2] < 0


def _stand(m, z0, steps=200):
    """Spawn as the cockpit does (IK stance, torso at h + spawn_dz above the
    ground) and let it settle under position control."""
    import rocky_model as rm
    from pebble_gait import WaveGait, leg_ik, body_to_leg
    g = WaveGait()
    d = mujoco.MjData(m)
    q0 = np.array([leg_ik(body_to_leg(i, g.p_nom[i])) for i in range(5)]).flatten()
    jadr = [m.joint(f"{n}{i}").qposadr[0] for i in range(5) for n in ("yaw", "hip", "knee")]
    d.qpos[0:3] = [0, 0, (g.h + rm.spawn_dz_mm()) / 1000.0 + z0]
    d.qpos[jadr] = q0
    d.ctrl[:15] = q0
    for _ in range(steps):
        mujoco.mj_step(m, d)
    return d


def _foot_floor_mu(m, d):
    fl = m.geom("floor").id
    feet = {m.geom(f"foot{i}").id for i in range(5)}
    out = []
    for c in d.contact[:d.ncon]:
        pair = {int(c.geom1), int(c.geom2)}
        if fl in pair and pair & feet:
            out.append((float(c.friction[0]), int(c.dim)))
    return out


def test_icy_floor_is_what_the_feet_feel():
    # the base mu comes from the compiled robot, not a literal: whatever
    # build_mjcf gave the feet is what an unmodified floor must give too
    robot = mujoco.MjModel.from_xml_path(os.path.join(HERE, "..", "pebble.xml"))
    base_mu = float(robot.geom_friction[robot.geom("foot0").id, 0])
    icy_mu = float(PRESETS["icy floor"]["friction"])
    assert icy_mu < base_mu                            # else the test proves nothing
    for name, want in (("flat", base_mu), ("icy floor", icy_mu)):
        m, z0 = build(PRESETS[name])
        d = _stand(m, z0)
        cs = _foot_floor_mu(m, d)
        assert len(cs) >= 4, (name, cs)                # standing on (almost) all feet
        for mu, dim in cs:
            assert mu == pytest.approx(want, abs=1e-9), name
            assert dim == 4, name


def _geom_R(m, gid):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, m.geom_quat[gid])
    return R.reshape(3, 3)


@pytest.mark.parametrize("yaw", [0.0, 37.0, 90.0, 161.0])
def test_ramp_top_is_not_canted_and_starts_on_the_floor(yaw):
    L, rise = 0.6, 0.06
    m, _ = build({"objects": [{"kind": "ramp", "pos": [0.5, -0.3], "len_m": L, "rise_m": rise,
                               "yaw_deg": yaw}]})
    gid = m.geom("obj0").id
    R = _geom_R(m, gid)
    n = R[:, 2]                                        # top-face normal
    ax = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw)), 0.0])
    side = np.array([-ax[1], ax[0], 0.0])
    assert abs(n @ side) < 1e-6                        # no sideways cant
    assert n @ ax < 0                                  # it faces back down the slope: it climbs along yaw
    assert n[2] == pytest.approx(np.cos(np.arctan2(rise, L)), abs=1e-6)
    hx, _, hz = m.geom_size[gid]
    c = m.geom_pos[gid]
    low, high = c + R @ [-hx, 0, hz], c + R @ [hx, 0, hz]
    assert low[2] == pytest.approx(0.0, abs=1e-4)      # the low edge is on the floor, at pos
    assert low[:2] == pytest.approx([0.5, -0.3], abs=1e-4)
    assert high[2] == pytest.approx(rise, abs=1e-4)
    assert (c + R @ [hx, 0, -hz])[2] < 0               # the high end is buried, not floating


def _profile(m, x0, y0, yaw, length, step=0.002):
    """Top-surface height along a line (vertical rays, world geoms + floor)."""
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    ax = np.array([np.cos(np.radians(yaw)), np.sin(np.radians(yaw))])
    gid = np.zeros(1, dtype=np.int32)
    torso = m.body("torso").id
    zs = []
    for s in np.arange(-0.1, length + 0.1, step):
        p = np.array([x0 + ax[0] * s, y0 + ax[1] * s, 1.0])
        dist = mujoco.mj_ray(m, d, p, np.array([0.0, 0.0, -1.0]), None, 1, torso, gid)
        zs.append(1.0 - dist)
    return np.array(zs)


def test_ramps_and_stairs_in_presets_have_no_far_end_drop():
    seen = 0
    for name, spec in PRESETS.items():
        m, _ = build(spec)
        for o in spec.get("objects") or []:
            if o["kind"] not in ("ramp", "stairs") or o.get("far") == "drop":
                continue
            seen += 1
            rise = float(o.get("rise_m", 0.06 if o["kind"] == "ramp" else 0.02))
            fp = footprint(o)
            length = max(np.hypot(sh[1] - o["pos"][0], sh[2] - o["pos"][1]) + sh[3] for sh in fp)
            z = _profile(m, o["pos"][0], o["pos"][1], o.get("yaw_deg", 0.0), length)
            assert z[0] == pytest.approx(0.0, abs=1e-6) and z[-1] == pytest.approx(0.0, abs=1e-6), name
            peak = z.max()
            assert peak > 0.9 * (rise * o.get("steps", 1)), name
            dz = np.diff(z)
            limit = 0.003 if o["kind"] == "ramp" else rise + 1e-6   # a ramp is continuous
            assert np.max(np.abs(dz)) <= limit, (name, float(np.max(np.abs(dz))))
    assert seen >= 2                                   # the obstacle-course ramp + the stairs


def test_far_drop_is_a_deliberate_ledge():
    o = {"kind": "stairs", "pos": [0.6, 0.0], "steps": 3, "rise_m": 0.02, "far": "drop"}
    m, _ = build({"objects": [o]})
    z = _profile(m, 0.6, 0.0, 0.0, 0.45)
    assert np.min(np.diff(z)) == pytest.approx(-0.06, abs=1e-6)
    with pytest.raises(ValueError):
        world_xml({"objects": [{"kind": "ramp", "far": "cliff"}]})


def test_footprint_distance():
    assert footprint_dist({"kind": "box", "pos": [0, 0], "size": [0.1, 0.1, 0.05]}) == 0.0
    assert footprint_dist({"kind": "box", "pos": [1, 0], "size": [0.2, 0.4, 0.05]}) == pytest.approx(0.9)
    # yawed 90: the long side now faces the origin
    assert footprint_dist({"kind": "box", "pos": [1, 0], "size": [0.2, 0.4, 0.05],
                           "yaw_deg": 90}) == pytest.approx(0.8)
    # stairs pointing AT the origin from 0.8 m reach through it (3 up + landing + 2 down)
    assert footprint_dist({"kind": "stairs", "pos": [0.8, 0], "yaw_deg": 180, "steps": 3}) == 0.0
    assert footprint_dist({"kind": "ball", "pos": [0.5, 0], "radius_m": 0.05}) == pytest.approx(0.45)


def test_random_courses_keep_the_spawn_clear():
    # pure geometry, no MuJoCo: before D052, 40 of these 200 seeds had an
    # object footprint within 0.3 m of the spawn point
    counts = []
    for seed in range(200):
        spec = random_course(seed)
        assert spec == random_course(seed)             # still a pure function of the seed
        counts.append(len(spec["objects"]))
        for o in spec["objects"]:
            assert footprint_dist(o) >= CLEAR_M, (seed, o)
        assert 0.5 <= spec["friction"] <= 1.0
    assert np.mean(counts) > 7.9                       # resampling, not dropping


def test_saved_worlds_build():
    for name in saved_worlds():
        m, _ = build(load_world(name))
        assert m.camera("eye").id >= 0, name


def test_unknown_kind_is_refused():
    with pytest.raises(ValueError):
        world_xml({"objects": [{"kind": "dragon"}]})
