"""D052 driver changes: tolerant telemetry, the monitor's lost/cut/rearm, sync
register writes, hardware angle limits, speed-0 semantics, a mock that cannot teleport."""
import math

import pytest
from rocky_driver import (FeetechBus, Family, SafetyMonitor, BusError, NoResponse,
                          make_pebble_mock)
from rocky_driver.registers import CENTER, COUNTS, max_speed_cps


def pebble_bus():
    t = make_pebble_mock()
    fams = {i: Family.STS for i in range(1, 16)} | {i: Family.SCS for i in range(16, 21)}
    return FeetechBus(t, fams), t


def tx_count(t):
    return len([e for e in t.log if e[0] == "tx"])


def test_mock_moves_no_faster_than_the_datasheet():
    bus, t = pebble_bus()
    bus.torque(2, True)
    bus.set_position(2, 90.0)                      # speed 0 = servo max
    t.advance(0.1)
    moved = (t.servo(2).get("PRESENT_POSITION") - CENTER[Family.STS]) * 2 * math.pi / COUNTS[Family.STS]
    assert moved == pytest.approx(4.7 * 0.1, abs=0.01)     # ST3215 no-load 0.222 s/60 deg
    assert max_speed_cps(Family.STS) == pytest.approx(3064, abs=2)


def test_read_telemetry_returns_what_answered_plus_missing():
    bus, t = pebble_bus()
    t.servo(2).silent = True
    t.servo(17).silent = True
    n = tx_count(t)
    tels, missing = bus.read_telemetry([1, 2, 3, 16, 17])
    assert sorted(tels) == [1, 3, 16] and missing == [2, 17]
    # one SYNC_READ + one single retry of id 2 + SCS reads (16 once, 17 twice)
    assert tx_count(t) - n == 1 + 1 + 1 + 2
    with pytest.raises(NoResponse):
        bus.sync_telemetry([1, 2, 3])              # the strict contract is kept for bench scripts
    t.servo(2).silent = False
    assert sorted(bus.sync_telemetry([1, 2, 3])) == [1, 2, 3]


def test_read_telemetry_survives_a_corrupt_frame():
    bus, t = pebble_bus()
    t.corrupt_next_checksum = 1                    # first reply of the sync read is garbage
    tels, missing = bus.read_telemetry([1, 2, 3])
    assert missing == [] and sorted(tels) == [1, 2, 3]      # the single retry picked id 1 up
    assert bus.stats["checksum_errors"] == 1


def test_silent_servo_still_takes_writes():
    bus, t = pebble_bus()
    t.servo(4).silent = True                       # a broken return wire, not a dead servo
    bus.torque_all([4], True)
    assert t.servo(4).get("TORQUE_ENABLE") == 1
    assert not bus.ping(4)


def test_monitor_lost_after_three_polls_and_only_rearm_clears():
    bus, t = pebble_bus()
    ev = []
    mon = SafetyMonitor(bus, [1, 2, 3, 16], on_event=lambda k, tel: ev.append((k, tel.servo_id)))
    t.servo(3).silent = True
    for _ in range(2):
        mon.step()
    assert 3 not in mon.tripped and mon.missing == [3]
    mon.step()
    assert 3 in mon.tripped and mon.lost == {3} and mon.reason[3] == "lost" and ("lost", 3) in ev
    t.servo(3).silent = False
    mon.step()
    assert 3 in mon.tripped                        # answering again does not re-arm
    assert mon.rearm([3]) == [3] and not mon.tripped and not mon.lost
    assert not any(k == "volt" for k, _ in ev)     # hands at 6 V are healthy on their own rail


def test_monitor_cut_hook_and_fault_reason():
    bus, t = pebble_bus()
    cut = []
    mon = SafetyMonitor(bus, [7, 8, 9])
    mon.cut = lambda sid, tel: cut.append(sid)
    t.servo(8).put("STATUS", 0b100)
    tels, missing = bus.read_telemetry([7, 8, 9])
    mon.check(tels, None, warn=False)
    assert cut == [8] and mon.reason[8].startswith("fault")
    mon.check(tels, None, warn=False)
    assert cut == [8]                              # once per trip


def test_sync_write_reg_and_eeprom_refusal():
    bus, t = pebble_bus()
    bus.sync_write_reg("TORQUE_LIMIT", {1: 400, 2: 400, 16: 400})   # SCS has no TORQUE_LIMIT: skipped
    assert t.servo(1).get("TORQUE_LIMIT") == 400 and t.servo(2).get("TORQUE_LIMIT") == 400
    bus.sync_write_reg("ACC", {3: 10})
    assert t.servo(3).get("ACC") == 10
    with pytest.raises(BusError):
        bus.sync_write_reg("MAX_ANGLE_LIMIT", {1: 3000})


def test_speed_zero_is_written_so_a_slow_speed_cannot_linger():
    bus, t = pebble_bus()
    bus.sync_positions({1: 10.0}, 200)
    assert t.servo(1).get("GOAL_SPEED") == 200
    bus.sync_positions({1: 20.0})
    assert t.servo(1).get("GOAL_SPEED") == 0       # back to servo max
    bus.set_position(2, 5.0, 150)
    bus.set_position(2, 6.0)
    assert t.servo(2).get("GOAL_SPEED") == 0


def test_angle_limits_written_verified_and_clamp_the_mock():
    bus, t = pebble_bus()
    assert bus.set_angle_limits(2, 1500, 2600) == (1500, 2600)
    assert t.servo(2).get("LOCK") == 1
    with pytest.raises(ValueError):
        bus.set_angle_limits(2, 3000, 1000)
    with pytest.raises(ValueError):
        bus.set_angle_limits(16, 0, 1024)          # SCS tops out at 1023
    bus.torque(2, True)
    bus.set_position(2, 170.0)
    t.advance(1.0)
    assert t.servo(2).get("PRESENT_POSITION") == 2600


def test_soft_enable_parks_the_goal_before_torque_comes_on():
    """D052 V2: the bench scripts enabled torque first — a Feetech servo then drives
    toward its LAST goal (anywhere, after the leg was moved by hand) at full torque."""
    from rocky_driver import soft_enable
    bus, t = pebble_bus()
    s = t.servo(2)
    s.put("GOAL_POSITION", CENTER[Family.STS] + 900)          # a stale goal ~79 deg away
    s._pos_f = float(CENTER[Family.STS] + 100)
    s.put("PRESENT_POSITION", CENTER[Family.STS] + 100)
    t.servo(5).silent = True
    here = soft_enable(bus, [2, 5], torque_limit=400, acc=10, speed_cps=200)
    assert list(here) == [2]                                   # the silent one stays limp
    assert t.servo(5).get("TORQUE_ENABLE") == 0
    assert s.get("TORQUE_ENABLE") == 1 and s.get("TORQUE_LIMIT") == 400 and s.get("ACC") == 10
    assert abs(s.get("GOAL_POSITION") - (CENTER[Family.STS] + 100)) <= 1
    t.advance(0.5)
    assert abs(s.get("PRESENT_POSITION") - (CENTER[Family.STS] + 100)) <= 1   # nothing moved


def test_soft_stream_enters_softly_clamps_rate_and_holds_nan():
    """D052 V2: the ROS 2 bridge's stream (rocky_driver.SoftStream) — the rules
    sim/hw_bridge.py applies, for a loop that owns its own timing."""
    import numpy as np
    from rocky_driver import PebbleRobot, SoftStream
    bus, t = pebble_bus()
    robot = PebbleRobot(bus)
    t.servo(2).put("GOAL_POSITION", CENTER[Family.STS] + 900)          # stale goal
    s = SoftStream(robot, blend_s=0.4, tail_s=0.2)
    assert len(s.enable()) == 15
    assert t.servo(2).get("TORQUE_LIMIT") == 400
    assert abs(t.servo(2).get("GOAL_POSITION") - CENTER[Family.STS]) <= 1   # parked, not 79 deg away
    target = np.zeros((5, 3))
    target[:, 1] = 1.0                                                   # hips +57 deg
    sent, now = [], 0.0
    for k in range(400):                                                 # 1.5 x 1 rad / (0.8 x 200 cps) = 6.1 s entry
        bad = target.copy()
        if k == 50:
            bad[0, 1] = np.nan
        sent.append(s.step(bad, now).copy())
        now += 0.02
    sent = np.array(sent)
    assert np.abs(np.diff(sent, axis=0)).max() <= 4.7 * 0.02 + 1e-9       # never past the hard speed
    assert sent[0, 0, 1] < 0.05                                          # the blend starts where it is
    assert s.nan_holds == 1 and np.isfinite(sent).all()
    assert abs(sent[-1, 0, 1] - 1.0) < 1e-6                              # arrives
    assert s.released and t.servo(2).get("TORQUE_LIMIT") == 1000
