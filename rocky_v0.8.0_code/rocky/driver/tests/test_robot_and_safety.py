"""PebbleRobot mapping layer + SafetyMonitor behavior + a gait smoke test."""
import math
import os
import sys
import numpy as np
import pytest
from rocky_driver import (FeetechBus, Family, PebbleRobot, make_pebble_mock,
                          SafetyLimits)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "gait"))

BUS_PARAMS = dict(
    leg_ids=[[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12], [13, 14, 15]],
    hand_ids=[16, 17, 18, 19, 20],
)


def make_robot(cal=None):
    t = make_pebble_mock()
    bus = FeetechBus(t)
    return PebbleRobot(bus, BUS_PARAMS, cal), t


# ---------------------------------------------------------------- mapping
def test_family_map_installed():
    robot, _ = make_robot()
    assert robot.bus.family_of(1) is Family.STS
    assert robot.bus.family_of(16) is Family.SCS


def test_q_mapping_with_calibration():
    cal = {"dir": {"leg0_hip": -1}, "offset": {"leg0_hip": 51}}  # 51 ct = 4.48 deg
    robot, _ = make_robot(cal)
    deg = robot.q_to_deg(0, 1, math.radians(30))
    assert abs(deg - (-30 + 51 * 360 / 4096)) < 1e-6
    # roundtrip
    q = robot.deg_to_q(0, 1, deg)
    assert abs(q - math.radians(30)) < 1e-9


def test_soft_limit_clamp():
    robot, t = make_robot()
    robot.send_leg_targets([[math.radians(80), 0, math.radians(-90)]] * 5)
    yaw_counts = t.servo(1).get("GOAL_POSITION")
    assert yaw_counts == 2048 + round(40 * 4096 / 360)     # clamped at +40


def test_send_and_read_roundtrip():
    robot, t = make_robot()
    robot.enable(True)
    q_cmd = [[0.1, 0.3, -1.5]] * 5
    robot.send_leg_targets(q_cmd)
    t.advance(2.0)
    q_read, tels = robot.read_joint_state()
    for leg in range(5):
        for j in range(3):
            assert abs(q_read[leg][j] - q_cmd[leg][j]) < math.radians(0.2)
    assert len(tels) == 20


def test_claw_mapping():
    robot, t = make_robot()
    robot.send_claws([0.0, 0.5, 1.0, 0.0, 0.0])
    c16 = t.servo(16).get("GOAL_POSITION")
    c18 = t.servo(18).get("GOAL_POSITION")
    assert c16 == 512                                       # closed = 0 deg
    assert c18 == 512 + round(55 * 1024 / 300)              # open = 55 deg


# ---------------------------------------------------------------- safety
def test_safety_monitor_cuts_torque_on_overtemp():
    robot, t = make_robot()
    robot.enable(True)
    events = []
    robot.monitor.on_event = lambda kind, tel: events.append((kind, tel.servo_id))
    s = t.servo(2)
    s._temp_f = 66.0                                        # above 65 C cut
    s.advance(0.01)
    robot.health_step()
    assert 2 in robot.monitor.tripped
    assert s.get("TORQUE_ENABLE") == 0
    assert ("cut", 2) in events


def test_safety_monitor_warns_before_cut():
    robot, t = make_robot()
    events = []
    robot.monitor.on_event = lambda kind, tel: events.append((kind, tel.servo_id))
    s = t.servo(3)
    s._temp_f = 61.0
    s.advance(0.01)
    robot.health_step()
    assert ("temp_warn", 3) in events
    assert 3 not in robot.monitor.tripped


def test_mock_thermal_model_self_protects():
    """Sanity for the soak rehearsal: sustained load heats the mock servo to
    its MAX_TEMP, at which point it cuts its own torque (like the real one)."""
    robot, t = make_robot()
    robot.enable(True)
    s = t.servo(2)
    s.external_load_pct = 85.0
    peak = 0.0
    for _ in range(600):
        t.advance(1.0)
        peak = max(peak, s._temp_f)
    assert peak >= 65.0                                     # crossed the cut line
    assert s.get("TORQUE_ENABLE") == 0                      # servo self-cut
    assert s._temp_f < peak                                 # ...and cooled after


# ---------------------------------------------------------------- gait bridge
def test_gait_engine_drives_the_bus_end_to_end():
    """The actual pebble_gait targets flow through calibration to servo counts
    and back. This is the exact loop the Pi will run on bench day."""
    from pebble_gait import WaveGait
    robot, t = make_robot()
    robot.enable(True)
    g = WaveGait()
    for step in range(20):
        tt = step * 0.05
        q, stance, _ = g.joint_targets(tt, 45.0, 0.0, 0.0)
        assert not np.isnan(q).any()
        robot.send_leg_targets(q)
        t.advance(0.05)
    q_read, tels = robot.read_joint_state()
    q_last, _, _ = g.joint_targets(19 * 0.05, 45.0, 0.0, 0.0)
    # servos chase the last command; most joints should be close
    err = np.abs(np.array(q_read) - q_last)
    assert np.median(err) < math.radians(6)
    assert all(not tel.faults for tel in tels.values())
