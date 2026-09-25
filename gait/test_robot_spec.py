"""D053 — the robot's shape as data (docs/ROBOT_AS_DATA.md step 1).

rocky_model.robot() builds the robot description from params `robot:`. These
tests pin it to what the code assumed before D053 (five legs at 90 + 72 i, a
yaw/hip/knee chain, the leg-major actuator order behind ctrl[:15] /
ctrl[15:20], the bus map) and prove validate() refuses what the rest of the
code cannot honour yet. Pure Python + numpy, no MuJoCo:

    python -m pytest gait/test_robot_spec.py -q
"""
import copy
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, "driver"))
import rocky_model as rm                                               # noqa: E402
from pebble_gait import L1, L2, L3, Z_HIP, leg_fk, N_LEGS, STATION_DEG  # noqa: E402

XML = os.path.join(REPO, "sim", "pebble.xml")


def _write(tmp_path, P, name="params.yaml"):
    p = tmp_path / name
    p.write_text(yaml.safe_dump(P, sort_keys=False))
    return str(p)


def _live():
    return rm.params()


def _with_ankle(P, solver="numeric", redundancy="foot_pitch"):
    """The ROBOT_AS_DATA §2.3 fixture: a 4th pitch joint on the tibia."""
    ch = P["robot"]["leg"]["chain"]
    ch.append({"name": "ankle", "axis": [0, -1, 0], "offset_mm": [100.0, 0, 0], "link": "tarsus"})
    P["robot"]["leg"]["foot"] = {"offset_mm": [35.0, 0, 0]}
    P["robot"]["leg"]["ik"] = {"solver": solver, "branch": "knee_down"}
    if redundancy:
        P["robot"]["leg"]["ik"].update(redundancy=redundancy, foot_pitch_deg=-90)
    P["joints"]["pos_deg"]["ankle"] = [-60, 60]
    P["bus"]["leg_ids"] = [[4 * i + k + 1 for k in range(4)] for i in range(5)]
    P["bus"]["hand_ids"] = [21 + i for i in range(5)]
    return P


def _six_legs(P):
    """The §2.4 fixture: a hexapod with the same leg."""
    P["robot"]["legs"]["count"] = 6
    P["bus"]["leg_ids"] = [[3 * i + k + 1 for k in range(3)] for i in range(6)]
    P["bus"]["hand_ids"] = [19 + i for i in range(6)]
    P["gait"]["duty"] = round(1 - 1 / 6, 4)
    return P


# ------------------------------------------------------------------ (a) today's robot
def test_spec_equals_the_pre_d053_literals():
    s = rm.robot()
    assert rm.n_legs() == s.n_legs == N_LEGS == 5
    assert rm.stations_deg() == (90.0, 162.0, 234.0, 306.0, 378.0)
    assert np.array_equal(STATION_DEG, 90 + 72 * np.arange(5))       # the old literal, value-equal
    assert rm.LEG_JOINTS == ("yaw", "hip", "knee") and rm.ALL_JOINTS == ("yaw", "hip", "knee", "claw")
    from rocky_model import LEG_JOINTS                                  # PEP 562 serves from-imports too
    assert LEG_JOINTS == rm.LEG_JOINTS
    for L in s.legs:
        assert [j.offset_mm for j in L.joints] == [(0, 0, 0), (L1, 0, Z_HIP), (L2, 0, 0)]
        assert L.foot_offset_mm == (L3, 0, 0)
        assert [j.axis for j in L.joints] == [(0, 0, 1), (0, -1, 0), (0, -1, 0)]
        assert L.ik["resolved"] == "analytic" and L.n_dof == 3
        assert L.tool.name == "claw" and L.tool.role == "tool" and L.tool.servo == "scs0009"
        assert s.station_radius_mm == 110.0
        assert np.allclose(L.mount_mm[:2], [110 * np.cos(np.radians(L.station_deg)),
                                             110 * np.sin(np.radians(L.station_deg))])
    assert all(rm.servo_for(j, leg) == "st3215" for leg in range(5) for j in rm.LEG_JOINTS)
    assert rm.servo_for("claw") == "scs0009"
    assert rm.uniform_dof() == rm.n_leg_dof() == 3 and rm.leg_joint_names(4) == rm.LEG_JOINTS
    lo, hi = rm.joint_limits_rad()
    for L in s.legs:
        assert np.allclose(np.radians([j.range_deg for j in L.joints]).T, [lo, hi])


def test_id_table_is_the_bus_map():
    rows = rm.id_table()
    assert [r.sid for r in rows] == list(range(1, 21))
    for r in rows:
        assert rm.id_to_joint(r.sid) == (r.leg, r.joint)
        assert rm.joint_to_id(r.leg, r.joint) == r.sid
        assert r.limits_deg == rm.joint_limits_deg()[r.joint]
    s = rm.robot()
    assert [[j.bus_id for j in L.joints] for L in s.legs] == rm.leg_ids()
    assert [L.tool_bus_id for L in s.legs] == rm.hand_ids()
    legs = [r for r in rows if r.joint != "claw"]
    assert {(r.servo, r.protocol, r.counts, r.sweep_deg) for r in legs} == {("st3215", "sts", 4096, 360.0)}
    claws = [r for r in rows if r.joint == "claw"]
    assert {(r.servo, r.protocol, r.counts, r.sweep_deg) for r in claws} == {("scs0009", "scs", 1024, 300)}


def test_driver_id_map_and_families_equal_the_id_table():
    from rocky_driver import FeetechBus, PebbleRobot, make_pebble_mock
    bus = FeetechBus(make_pebble_mock())
    robot = PebbleRobot(bus)                                   # reads params.yaml bus: itself
    rows = rm.id_table()
    assert sorted(robot.all_ids()) == [r.sid for r in rows]
    for r in rows:
        assert bus.family_of(r.sid).name.lower() == r.protocol, r
    assert sorted(robot.monitor.ids) == [r.sid for r in rows]            # the safety monitor watches them all


# ------------------------------------------------------------------ (b) the chain IS the kinematics
def _rot(axis, q):
    a = np.asarray(axis, float)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def _chain_fk(leg, q):
    R, p = np.eye(3), np.zeros(3)
    for j, qi in zip(leg.joints, q):
        p = p + R @ np.asarray(j.offset_mm, float)
        R = R @ _rot(j.axis, qi)
    return p + R @ np.asarray(leg.foot_offset_mm, float)


def test_generic_chain_fk_equals_the_closed_form():
    rng = np.random.default_rng(53)
    leg = rm.robot().legs[0]
    lo = np.radians([j.range_deg[0] for j in leg.joints])
    hi = np.radians([j.range_deg[1] for j in leg.joints])
    worst = 0.0
    for _ in range(500):
        q = rng.uniform(lo, hi)
        worst = max(worst, float(np.linalg.norm(_chain_fk(leg, q) - leg_fk(q))))
    assert worst < 1e-9, worst


# ------------------------------------------------------------------ (c) the ctrl-order contract
def test_actuator_order_is_the_compiled_ctrl_order():
    names = [a.get("name") for a in ET.parse(XML).getroot().find("actuator")]
    assert names == rm.actuator_order()
    assert names[:15] == [f"{j}{i}" for i in range(5) for j in ("yaw", "hip", "knee")]   # ctrl[:15]
    assert names[15:20] == [f"claw{i}" for i in range(5)]                                 # ctrl[15:20]
    assert rm.robot().n_leg_actuators() == 15


# ------------------------------------------------------------------ (d) validate() refuses
def _refused(tmp_path, P, *needles):
    path = _write(tmp_path, P)
    with pytest.raises(rm.RobotSpecError) as e:
        rm.robot(path)
    msg = str(e.value)
    for n in needles:
        assert n in msg, msg
    with pytest.raises(rm.RobotSpecError):
        rm.validate(P)
    return msg


def test_refuses_more_id_rows_than_legs(tmp_path):
    P = _live()
    P["bus"]["leg_ids"].append([21, 22, 23])
    _refused(tmp_path, P, "bus.leg_ids has 6 rows for robot.legs.count 5")


def test_refuses_a_row_that_does_not_fit_the_chain(tmp_path):
    P = _live()
    P["bus"]["leg_ids"][2] = [7, 8, 9, 21]
    _refused(tmp_path, P, "bus.leg_ids[2] has 4 ids for a 3-joint chain")


def test_refuses_an_unknown_servo(tmp_path):
    P = _live()
    P["robot"]["leg"]["servo"] = "mg996r"
    _refused(tmp_path, P, "servo 'mg996r' is not in actuators", "actuators.mg996r")


def test_refuses_a_joint_without_a_range(tmp_path):
    P = _live()
    del P["joints"]["pos_deg"]["knee"]
    _refused(tmp_path, P, "joint 'knee' has no joints.pos_deg entry")


def test_refuses_a_duplicate_id(tmp_path):
    P = _live()
    P["bus"]["hand_ids"][0] = 3
    _refused(tmp_path, P, "servo id 3 is used twice")


def test_refuses_analytic_ik_on_an_ankle(tmp_path):
    P = _with_ankle(_live(), solver="analytic")
    msg = _refused(tmp_path, P, "solver 'analytic'", "numeric")
    assert "yaw/hip/knee/ankle" in msg


def test_refuses_a_redundant_leg_without_a_redundancy_rule(tmp_path):
    P = _with_ankle(_live(), redundancy=None)
    _refused(tmp_path, P, "1 spare DOF", "redundancy")
    P = _with_ankle(_live(), solver="auto", redundancy=None)      # auto resolves numeric -> same rule
    _refused(tmp_path, P, "redundancy")


def test_refuses_overrides_until_something_honours_them(tmp_path):
    P = _live()
    P["robot"]["overrides"] = {"leg2": {"knee": {"servo": "st3215"}}}
    _refused(tmp_path, P, "robot.overrides is not empty")


def test_refuses_uneven_stations_unless_asked(tmp_path):
    P = _live()
    P["robot"]["legs"]["station_deg"] = [90, 160, 234, 306, 378]
    _refused(tmp_path, P, "not evenly spaced")
    P["robot"]["legs"]["allow_asymmetric"] = True
    assert rm.robot(_write(tmp_path, P, "asym.yaml")).stations_deg()[1] == 160.0


def test_all_errors_are_reported_at_once(tmp_path):
    P = _live()
    P["robot"]["leg"]["servo"] = "mg996r"
    P["bus"]["hand_ids"][0] = 3
    msg = _refused(tmp_path, P, "mg996r", "used twice")
    assert msg.count("\n  - ") >= 2


# ------------------------------------------------------------------ (e) hypothetical robots
def test_six_legs_is_a_valid_description(tmp_path):
    path = _write(tmp_path, _six_legs(_live()))
    s = rm.robot(path)
    assert s.n_legs == 6 and np.allclose(np.diff(s.stations_deg()), 60.0)
    assert s.stations_deg()[0] == 90.0 and s.stations_deg()[-1] == 390.0
    order = rm.actuator_order(s)
    assert len(order) == 24 and order[:3] == ["yaw0", "hip0", "knee0"] and order[-1] == "claw5"
    assert s.n_leg_actuators() == 18                               # the RL action size for this robot
    ids = [r.sid for r in rm.id_table(path)]
    assert ids == list(range(1, 25))
    assert rm.topology_hash(s) != rm.topology_hash()
    assert rm.robot_warnings(path) == [w for w in rm.robot_warnings(path) if "duty" not in w]


def test_four_joints_is_a_valid_description(tmp_path):
    path = _write(tmp_path, _with_ankle(_live()))
    s = rm.robot(path)
    L = s.legs[0]
    assert L.joint_names == ("yaw", "hip", "knee", "ankle") and L.n_dof == 4
    assert L.ik["resolved"] == "numeric" and L.ik["redundancy"] == "foot_pitch"
    assert len(rm.actuator_order(s)) == 25 and s.n_leg_actuators() == 20
    assert [r.sid for r in rm.id_table(path)] == list(range(1, 26))
    assert rm.topology_hash(s) != rm.topology_hash()
    assert rm.description_digest(s) != rm.description_digest()


def test_duty_warning_for_a_six_legged_wave(tmp_path):
    P = _six_legs(_live())
    P["gait"]["duty"] = 0.8                                          # today's 5-leg value
    warns = rm.robot_warnings(_write(tmp_path, P))
    assert any("gait.duty 0.8" in w and "0.8333" in w for w in warns)


def test_lengths_rotate_the_digest_not_the_topology(tmp_path):
    P = _live()
    P["leg"]["l3_tibia"] = 140.0                                     # the anchor is gone after safe_dump:
    P["robot"]["leg"]["foot"]["offset_mm"][0] = 140.0                # set both, as the YAML alias would
    s = rm.robot(_write(tmp_path, P))
    assert rm.topology_hash(s) == rm.topology_hash()
    assert rm.description_digest(s) != rm.description_digest()


# ------------------------------------------------------------------ (f) absent block == today's block
def test_no_robot_block_builds_the_same_spec(tmp_path):
    P = _live()
    del P["robot"]
    s = rm.robot(_write(tmp_path, P))
    assert s == rm.robot()
    assert rm.topology_hash(s) == rm.topology_hash() and rm.description_digest(s) == rm.description_digest()


def test_params_block_is_the_default_block():
    P = _live()
    assert P["robot"] == rm._default_robot_block(P)


# ------------------------------------------------------------------ (g) copies
def test_robot_returns_copies():
    s = rm.robot()
    s.legs[0].ik["solver"] = "numeric"
    assert rm.robot().legs[0].ik["solver"] == "auto"
    t = rm.to_json()
    t["legs"][0]["joints"][0]["name"] = "nope"
    assert rm.to_json()["legs"][0]["joints"][0]["name"] == "yaw"
    assert t["topology_hash"] == rm.topology_hash() and len(t["topology_hash"]) == 12


# ------------------------------------------------------------------ (h) purity + partial edits
def _py(code, cwd=HERE):
    r = subprocess.run([sys.executable, "-c", code], cwd=cwd, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    return r.stdout


def test_import_stays_pure_python():
    out = _py("import sys; import rocky_model; print('numpy' in sys.modules, 'mujoco' in sys.modules)")
    assert out.split() == ["False", "False"]


def test_a_broken_robot_block_breaks_only_topology(tmp_path):
    P = _live()
    P["robot"]["legs"]["count"] = 4                                  # 5 id rows: invalid
    path = _write(tmp_path, P)
    out = _py(f"""
import rocky_model as rm
rm.PARAMS_PATH = {path!r}
print(rm.stall_nm(), len(rm.leg_ids()), rm.servo_speed('free'))
try:
    rm.LEG_JOINTS
except rm.RobotSpecError as e:
    print('refused', 'bus.leg_ids has 5 rows' in str(e))
""")
    assert out.split() == ["2.94", "5", "4.0", "refused", "True"]
