"""D052: one source of truth for the robot model + actuator physics.

Every consumer that states a joint limit, a servo speed, a torque or a mass
must agree with cad/params.yaml as read by gait/rocky_model.py: the MJCF,
the URDF, the driver's soft-limit clamp, the servo realism layer, the shove
model. Plus the physics claims D052 makes (damping / forcerange derived,
no claw double-count, the foot surface is the IK point), and the two new
shared tools (perception/contacts.py, sim/model_fingerprint.py).
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
for sub in ("sim", "gait", "perception", "driver"):
    sys.path.insert(0, os.path.join(REPO, sub))
mujoco = pytest.importorskip("mujoco")

import rocky_model as rm                                               # noqa: E402

XML = os.path.join(REPO, "sim", "pebble.xml")
URDF = os.path.join(REPO, "ros2", "rocky_description", "urdf", "pebble.urdf")
BUDGET = os.path.join(REPO, "sim", "mass_budget.json")
N = 5


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_path(XML)


# ------------------------------------------------------------------ params itself
def test_params_aliases_are_one_number():
    P = rm.params()
    assert P["bus"]["soft_limits_deg"] == P["joints"]["pos_deg"]
    assert P["mass_hw"]["servo_st3215"] == P["actuators"]["st3215"]["mass_g"]
    assert P["mass_hw"]["servo_scs0009"] == P["actuators"]["scs0009"]["mass_g"]
    assert P["servo_st3215"]["mass_g"] == P["actuators"]["st3215"]["mass_g"]
    assert rm.servo_speed("hard") == rm.no_load_rad_s()
    assert rm.servo_speed("loaded") <= rm.servo_speed("free") <= rm.servo_speed("hard")


def test_loader_numbers():
    assert rm.stall_nm() == pytest.approx(2.94)
    assert rm.continuous_nm() == pytest.approx(0.65 * 2.94)
    assert rm.damping_nms() == pytest.approx(2.94 / 4.7)
    # the loaded speed budget IS where the continuous clip meets the damping line
    assert rm.continuous_nm() / rm.damping_nms() == pytest.approx(rm.servo_speed("loaded"), abs=0.1)
    # D052 amendment: peak = stall; the DC line at stall / at continuous torque, no load
    assert rm.peak_nm() == rm.stall_nm()
    assert rm.dc_speed() == pytest.approx(rm.no_load_rad_s())
    assert rm.dc_speed(rm.continuous_nm()) == pytest.approx(3.055, abs=1e-3)
    assert rm.dc_speed(load_nm=rm.continuous_nm()) == pytest.approx((2.94 - 1.911) / (2.94 / 4.7))
    th = rm.thermal()
    assert th["budget"] == pytest.approx((0.85 ** 2 - 0.65 ** 2) * 180.0)       # 54 load^2.s
    assert rm.thermal("claw")["trip_s"] == th["trip_s"]                          # inherits the leg's
    assert rm.foot_contact_radius_mm() == pytest.approx(6.5)
    lo, hi = rm.joint_limits_rad()
    assert np.allclose(np.degrees(lo), [-40, -70, -150]) and np.allclose(np.degrees(hi), [40, 90, -20])


def test_a_free_joint_can_reach_the_free_speed_budget(model):
    """The review's 'SIM vs BUDGET MISMATCH' (a strict xfail until the D052
    amendment): the fastest a joint can swing with no load in the MJCF is
    forcerange / damping — with forcerange = stall that is the 4.7 no-load
    speed, above the 4.0 free budget (it was 3.06 with the continuous clip)."""
    dofs = [model.jnt_dofadr[model.joint(f"{n}{i}").id] for i in range(N) for n in rm.LEG_JOINTS]
    reach = model.actuator_forcerange[:15, 1] / model.dof_damping[dofs]
    assert reach.min() >= rm.servo_speed("free") - 1e-6, reach.min()
    assert reach == pytest.approx(rm.no_load_rad_s(), abs=1e-3)


def _swing_peak(joint, q_from, q_to, forcerange=None, secs=0.8):
    """Peak |qdot| of one joint stepped from q_from to q_to in the MJCF, the
    torso pinned 1 m up with gravity off (no load: only the motor's own line)."""
    m = mujoco.MjModel.from_xml_path(XML)
    m.opt.gravity[:] = 0.0
    a = m.actuator(joint).id
    if forcerange is not None:
        m.actuator_forcerange[a] = [-forcerange, forcerange]
    d = mujoco.MjData(m)
    jadr = [m.joint(f"{n}{i}").qposadr[0] for i in range(N) for n in rm.LEG_JOINTS]
    j = m.joint(joint)
    d.qpos[0:7] = [0, 0, 1.0, 1, 0, 0, 0]
    d.qpos[j.qposadr[0]] = q_from
    d.ctrl[:15] = d.qpos[jadr]
    d.ctrl[a] = q_to
    peak = 0.0
    for _ in range(int(secs / m.opt.timestep)):
        mujoco.mj_step(m, d)
        d.qpos[0:7] = [0, 0, 1.0, 1, 0, 0, 0]
        d.qvel[0:6] = 0.0
        peak = max(peak, abs(float(d.qvel[j.dofadr[0]])))
    return peak


SWINGS = (("yaw0", -0.6, 0.6), ("hip2", -1.1, 1.4), ("knee4", -2.5, -0.4))


@pytest.mark.parametrize("joint,q_from,q_to", SWINGS)
def test_mjcf_free_joint_swings_past_the_free_budget(joint, q_from, q_to):
    """Dynamic, not just the ratio: a big unloaded step saturates the actuator
    at the peak torque and the joint runs up the DC line to ~4.7 rad/s (>= the
    4.0 free budget the studio / pebble_feasibility allow). Measured 4.71."""
    v = _swing_peak(joint, q_from, q_to)
    assert v >= rm.servo_speed("free"), v
    assert v == pytest.approx(rm.no_load_rad_s(), abs=0.1)


@pytest.mark.parametrize("joint,q_from,q_to", SWINGS)
def test_mjcf_joint_at_continuous_torque_reaches_3p05(joint, q_from, q_to):
    """A joint whose motor is held to the continuous torque (what a thermally
    derated joint gets, rl_common.ThermalProxy) tops out at continuous /
    damping = 3.05 rad/s — the 'loaded' speed budget. Measured 3.06."""
    v = _swing_peak(joint, q_from, q_to, forcerange=rm.continuous_nm())
    assert v == pytest.approx(rm.dc_speed(rm.continuous_nm()), abs=0.05)
    assert v >= rm.servo_speed("loaded")


def test_feasibility_thermal_load_verdict():
    """pebble_feasibility.judge_load (fed by audit_gestures' MuJoCo playback):
    RMS |tau_e|/stall > continuous_frac warns, > the 0.85 trip fraction fails."""
    import pebble_feasibility as pf
    q = pf.planted_q()

    def hold(g, t):
        return q, np.zeros(N)
    assert pf.LOAD_WARN == pytest.approx(0.65) and pf.LOAD_FAIL == pytest.approx(0.85)
    load = np.full((N, 3), 0.2)
    rep = pf.judge_load(pf.check(hold, 1.0, name="hold"), load)
    assert rep.ok and not [c for c in rep.codes if c.startswith("THERMAL_LOAD")]
    assert "RMS load 0.20 x stall" in rep.lines[0] and rep.to_json()["load_rms"][0][0] == 0.2
    load[2, 1] = 0.7
    rep = pf.judge_load(pf.check(hold, 1.0, name="hold"), load)
    assert rep.ok and rep.warnings == ["THERMAL_LOAD_WARN"]
    assert any("THERMAL_LOAD_WARN: leg 2 hip RMS load 0.70" in ln for ln in rep.lines)
    load[4, 2] = 0.9
    rep = pf.judge_load(pf.check(hold, 1.0, name="hold"), load.ravel())
    assert not rep.ok and rep.fails == ["THERMAL_LOAD"] and rep.lines[0].startswith("FAIL")
    assert any("FAIL THERMAL_LOAD: leg 4 knee RMS load 0.90" in ln for ln in rep.lines)


def test_id_map_inverts_params_not_divmod():
    for leg, row in enumerate(rm.leg_ids()):
        for j, sid in enumerate(row):
            assert rm.id_to_joint(sid) == (leg, rm.LEG_JOINTS[j])
            assert rm.joint_to_id(leg, rm.LEG_JOINTS[j]) == sid
    for leg, sid in enumerate(rm.hand_ids()):
        assert rm.id_to_joint(sid) == (leg, "claw")
    with pytest.raises(KeyError):
        rm.id_to_joint(99)


def test_loader_returns_copies():
    rm.params()["joints"]["pos_deg"]["yaw"][0] = -999
    rm.gait_defaults()["duty"] = 0.1
    assert rm.joint_limits_deg()["yaw"][0] == -40
    assert rm.gait_defaults()["duty"] == 0.8


# ------------------------------------------------------------------ MJCF
def test_pebble_xml_is_regenerated():
    import build_mjcf
    with open(XML) as f:
        assert f.read() == build_mjcf.build_xml(), "sim/pebble.xml is stale: run sim/build_mjcf.py"


def test_mjcf_ranges_damping_forcerange(model):
    lim = rm.joint_limits_deg()
    for i in range(N):
        for j in ("yaw", "hip", "knee", "claw"):
            jid = model.joint(f"{j}{i}").id
            assert np.allclose(np.degrees(model.jnt_range[jid]), lim[j]), f"{j}{i}"
        for j in ("yaw", "hip", "knee"):
            jid = model.joint(f"{j}{i}").id
            assert model.dof_damping[model.jnt_dofadr[jid]] == pytest.approx(rm.damping_nms(), abs=1e-4)
            a = model.actuator(f"{j}{i}").id
            # D052 amendment: the clip is the PEAK (stall) torque; continuous is thermal
            assert np.allclose(model.actuator_forcerange[a], [-rm.stall_nm(), rm.stall_nm()],
                               atol=1e-4)
        a = model.actuator(f"claw{i}").id
        assert model.actuator_forcerange[a][1] == pytest.approx(rm.stall_nm("claw"), abs=1e-4)


def test_compiled_mass_equals_budget(model):
    import json
    mb = json.load(open(BUDGET))
    total_g = mb["torso"] + N * (mb["coxa"] + mb["femur"] + mb["tibia"])
    compiled_g = model.body_subtreemass[model.body("torso").id] * 1000
    assert abs(compiled_g - total_g) < 2.0, (compiled_g, total_g)     # the old +40 g claw double-count


def test_foot_surface_is_the_ik_point(model):
    from pebble_gait import leg_fk, leg_to_body
    data = mujoco.MjData(model)
    q = np.array([0.2, 0.3, -1.4])
    for i in range(N):
        for k, name in enumerate(("yaw", "hip", "knee")):
            data.qpos[model.joint(f"{name}{i}").qposadr[0]] = q[k]
    mujoco.mj_forward(model, data)
    t = model.body("torso").id
    R, p0 = data.xmat[t].reshape(3, 3), data.xpos[t]
    for i in range(N):
        tip = R.T @ (data.site_xpos[model.site(f"foot_tip{i}").id] - p0) * 1000
        assert np.allclose(tip, leg_to_body(i, leg_fk(q)), atol=0.1)
        g = model.geom(f"foot{i}").id
        centre = data.geom_xpos[g]
        assert model.geom_size[g][0] * 1000 == pytest.approx(rm.foot_contact_radius_mm())
        # the sphere centre sits exactly r_c behind the tip: its surface passes through it
        assert np.linalg.norm(data.site_xpos[model.site(f"foot_tip{i}").id] - centre) \
            == pytest.approx(model.geom_size[g][0], abs=1e-6)
        assert model.geom_condim[g] == 4 and model.geom_friction[g][1] > 0


def test_spawn_settles_to_stance_height(model):
    """rocky_model.spawn_z_m() spawns ~1 mm above the floor and the robot
    settles at the gait's deck height h (not h + 13 as before D052)."""
    from pebble_gait import WaveGait, leg_ik, body_to_leg
    g = WaveGait(**rm.gait_defaults())
    q0 = np.array([leg_ik(body_to_leg(i, g.p_nom[i])) for i in range(N)]).ravel()
    data = mujoco.MjData(model)
    data.qpos[0:3] = [0, 0, rm.spawn_z_m()]
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[[model.joint(f"{n}{i}").qposadr[0] for i in range(N)
               for n in ("yaw", "hip", "knee")]] = q0   # claws interleave in qpos: never qpos[7:22]
    data.ctrl[:15] = q0
    for _ in range(int(0.8 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    z_mm = data.qpos[2] * 1000
    assert rm.stance_torso_z_m() * 1000 - 4.0 < z_mm < rm.stance_torso_z_m() * 1000 + 0.5, z_mm
    from contacts import foot_contacts
    fids = [model.geom(f"foot{i}").id for i in range(N)]
    assert foot_contacts(model, data, fids).all()


# ------------------------------------------------------------------ URDF
def test_urdf_limits_effort_velocity_damping():
    root = ET.parse(URDF).getroot()
    lim = rm.joint_limits_deg()
    seen = 0
    for j in root.iter("joint"):
        name = j.get("name")
        kind = name.rstrip("0123456789")
        if kind not in ("yaw", "hip", "knee", "claw"):
            continue
        L = j.find("limit")
        assert math.degrees(float(L.get("lower"))) == pytest.approx(lim[kind][0], abs=1e-3)
        assert math.degrees(float(L.get("upper"))) == pytest.approx(lim[kind][1], abs=1e-3)
        if kind == "claw":
            assert float(L.get("effort")) == pytest.approx(rm.stall_nm("claw"), abs=1e-3)
            assert float(L.get("velocity")) == pytest.approx(rm.no_load_rad_s("claw"))
        else:
            assert float(L.get("effort")) == pytest.approx(rm.stall_nm(), abs=1e-3)   # peak (amendment)
            assert float(L.get("velocity")) == pytest.approx(rm.servo_speed("hard"))
            assert float(j.find("dynamics").get("damping")) == pytest.approx(rm.damping_nms(), abs=1e-3)
        seen += 1
    assert seen == 20


# ------------------------------------------------------------------ driver + servo model
def test_driver_soft_limits_and_nan_guard():
    from rocky_driver import FeetechBus, PebbleRobot, make_pebble_mock
    from rocky_driver.registers import COUNTS, SWEEP_DEG
    from rocky_driver.protocol import Family
    robot = PebbleRobot(FeetechBus(make_pebble_mock()),
                        dict(leg_ids=rm.leg_ids(), hand_ids=rm.hand_ids()))
    lim = rm.joint_limits_deg()
    assert {k: tuple(v) for k, v in robot.soft_limits_deg.items()} == lim
    assert robot.q_to_deg(0, 1, math.radians(500)) == pytest.approx(lim["hip"][1])
    for bad in (float("nan"), float("inf")):
        with pytest.raises(ValueError):
            robot.q_to_deg(0, 2, bad)
        with pytest.raises(ValueError):
            robot.send_claws([0.5, 0.5, bad, 0.5, 0.5])
    assert COUNTS[Family.STS] / math.radians(SWEEP_DEG[Family.STS]) == pytest.approx(rm.counts_per_rad())
    assert COUNTS[Family.SCS] / math.radians(SWEEP_DEG[Family.SCS]) == pytest.approx(rm.counts_per_rad("claw"))


def test_servo_model_defaults_and_load_slew():
    from servo_model import DEFAULTS, STS_COUNTS_PER_RAD, ServoModel
    assert DEFAULTS["rate_rad_s"] == rm.no_load_rad_s()
    assert DEFAULTS["hold_hz"] == rm.bus_hz() and DEFAULTS["latency_s"] == rm.latency_s()
    assert DEFAULTS["stall_nm"] == rm.stall_nm()
    assert STS_COUNTS_PER_RAD == pytest.approx(4096 / (2 * np.pi))
    free, loaded, old = ServoModel(on=True), ServoModel(on=True), ServoModel(on=True)
    for _ in range(60):
        a = free.filter(np.ones(15), 0.002, force=np.zeros(15))
        b = loaded.filter(np.ones(15), 0.002, force=np.full(15, rm.continuous_nm()))
        c = old.filter(np.ones(15), 0.002)                             # backward compatible call
    assert np.allclose(a, c)                                           # zero load == the old slew
    assert np.allclose(b, a)          # D052 V2: load does NOT slow the goal by default (the MJCF has the line)
    assert DEFAULTS["load_derate"] is False
    derated = ServoModel(on=True, load_derate=True)                   # the A/B switch keeps the old rule
    for _ in range(60):
        b = derated.filter(np.ones(15), 0.002, force=np.full(15, rm.continuous_nm()))
    assert b[0] == pytest.approx(a[0] * (1 - rm.continuous_nm() / rm.stall_nm()), rel=0.05)
    stalled = ServoModel(on=True, load_derate=True)
    for _ in range(60):
        s = stalled.filter(np.ones(15), 0.002, force=np.full(15, 10.0))
    assert s[0] == pytest.approx(a[0] * 0.2, rel=0.05)                # floor: never frozen


# ------------------------------------------------------------------ shove, fingerprint (worlds)
def _world(spec):
    wb = pytest.importorskip("world_builder")
    return wb.build(spec)[0]


def test_shove_bodyweight_ignores_a_free_ball(model):
    from shove import total_mass, bodyweights, LEVER_Z
    assert LEVER_Z == pytest.approx(rm.shell_rim_z_m())
    with_ball = _world({"base": "flat", "objects": [{"kind": "ball", "pos": [0.5, 0.0],
                                                     "radius_m": 0.05, "mass_kg": 0.1}]})
    assert with_ball.body_subtreemass[0] > model.body_subtreemass[0] + 0.09   # the ball IS in the world
    assert total_mass(with_ball) == pytest.approx(total_mass(model))
    assert bodyweights(25.0, with_ball) == pytest.approx(bodyweights(25.0, model))


def test_robot_fingerprint_ignores_the_world_not_the_robot(model):
    from model_fingerprint import robot_fingerprint, fingerprint_note
    fp = robot_fingerprint(model)
    assert len(fp) == 12 and int(fp, 16) >= 0
    busy = _world({"base": "cliff", "friction": 0.35, "gravity_tilt_deg": 5.0, "objects": [
        {"kind": "box", "pos": [0.6, 0.2], "size": [0.1, 0.1, 0.05]},
        {"kind": "ball", "pos": [0.5, -0.4], "radius_m": 0.05}]})
    assert robot_fingerprint(busy) == fp
    changed = mujoco.MjModel.from_xml_path(XML)
    changed.dof_damping[changed.jnt_dofadr[changed.joint("hip2").id]] += 0.1
    assert robot_fingerprint(changed) != fp
    changed = mujoco.MjModel.from_xml_path(XML)
    changed.actuator_forcerange[changed.actuator("knee0").id] *= 1.5
    assert robot_fingerprint(changed) != fp
    note = fingerprint_note(model)
    assert fp in note and "mujoco" in note and "D052" in note


# ------------------------------------------------------------------ contacts
SELF_XML = """<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="1 1 0.1"/>
    <body name="torso" pos="0 0 0.05">
      <freejoint/>
      <geom type="box" size="0.03 0.03 0.01" mass="1"/>
      <body name="legA" pos="0.1 0 0"><joint type="hinge" axis="0 1 0"/>
        <geom name="foot0" type="sphere" size="0.01" pos="0 0 -0.02" mass="0.05"/></body>
      <body name="legB" pos="0.1 0 0"><joint type="hinge" axis="0 1 0"/>
        <geom name="shin" type="box" size="0.02 0.02 0.005" pos="0 0 -0.03" mass="0.05"/></body>
      <body name="legC" pos="-0.1 0 0"><joint type="hinge" axis="0 1 0"/>
        <geom name="foot1" type="sphere" size="0.01" pos="0 0 -0.049" mass="0.05"/></body>
      <body name="legD" pos="0 0.1 0"><joint type="hinge" axis="1 0 0"/>
        <geom name="foot2" type="sphere" size="0.01" pos="0 0 -0.02" mass="0.05"/></body>
    </body>
    <body name="ball" pos="0 0.1 0.0"><freejoint/>
      <geom name="ball" type="sphere" size="0.021" mass="0.1"/></body>
  </worldbody>
</mujoco>"""


def test_contacts_ignore_self_contact_count_world():
    from contacts import foot_contacts, foot_forces
    m = mujoco.MjModel.from_xml_string(SELF_XML)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    fids = [m.geom(f"foot{i}").id for i in range(3)]
    pairs = {tuple(sorted((int(c.geom1), int(c.geom2)))) for c in d.contact[:d.ncon]}
    assert tuple(sorted((fids[0], m.geom("shin").id))) in pairs          # foot0 IS in its own shin
    assert tuple(sorted((fids[2], m.geom("ball").id))) in pairs          # foot2 is on the ball
    F = foot_forces(m, d, fids)
    assert F[0] == 0.0                     # a foot pressed into the robot's own body does not count
    assert F[1] > 0.0                      # the floor does
    assert F[2] > 0.0                      # a FREE world object (not the robot's tree) does too
    # hysteresis: between open and close the switch keeps its state
    mid = F[1]
    st = np.zeros(3, bool)
    assert not foot_contacts(m, d, fids, st, close_n=mid + 1, open_n=mid - 1)[1]
    st[1] = True
    assert foot_contacts(m, d, fids, st, close_n=mid + 1, open_n=mid - 1)[1]
    assert st[1]                                                           # state updated in place
    assert not foot_contacts(m, d, fids, st, close_n=mid + 2, open_n=mid + 1)[1] and not st[1]
    # no state: a plain close_n threshold (params sensing.foot_switch by default)
    assert foot_contacts(m, d, fids, close_n=mid - 1)[1]
    assert not foot_contacts(m, d, fids, close_n=mid + 1)[1]


def test_run_odom_contacts_keep_their_1p5_newton_threshold(model):
    """V2 (review): the old version tested a robot in the air (every force 0), so a
    foot_contacts that ignored CONTACT_FORCE_N passed. Now the planted robot is
    unloaded by a lift on the torso until the feet carry 1.84 N (between 1.5 and
    the params switch's 2.0) and 1.04 N (between righter's 0.3 and 1.5)."""
    import run_odom
    from contacts import foot_forces
    from pebble_gait import WaveGait, leg_ik, body_to_leg
    assert run_odom.CONTACT_FORCE_N == 1.5
    g = WaveGait()
    q0 = np.array([leg_ik(body_to_leg(i, g.p_nom[i])) for i in range(N)]).ravel()
    jadr = [model.joint(f"{n}{i}").qposadr[0] for i in range(N) for n in rm.LEG_JOINTS]
    fids = [model.geom(f"foot{i}").id for i in range(N)]
    torso = model.body("torso").id
    bands = {17.0: (1.5, 2.0), 21.0: (0.3, 1.5)}      # lift N -> the band every foot load must fall in
    for lift, (lo, hi) in bands.items():
        d = mujoco.MjData(model)
        d.qpos[2] = rm.spawn_z_m(g.h)
        d.qpos[3] = 1.0
        d.qpos[jadr] = q0
        d.ctrl[:15] = q0
        mujoco.mj_forward(model, d)
        for k in range(1500):
            d.xfrc_applied[torso, 2] = lift * min(1.0, k / 500)
            mujoco.mj_step(model, d)
        F = foot_forces(model, d, fids)
        assert np.all((F > lo) & (F < hi)), (lift, F)
        assert np.array_equal(run_odom.foot_contacts(model, d), F > 1.5), (lift, F)
