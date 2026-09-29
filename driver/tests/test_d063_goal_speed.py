"""D063 (B76 fix 3): a stream writes each servo's goal speed per tick —
TRACK_K x its goal step / tick, floored (never 0 = servo max), capped — in one
SYNC_WRITE, and the mock moves each servo at its own speed under the no-load cap.
B79: the factory registers are named, and a locked mock refuses them."""
import math

import numpy as np
import pytest
from rocky_driver import FeetechBus, Family, PebbleRobot, SoftStream, make_pebble_mock
from rocky_driver.bus import TRACK_K, TRACK_FLOOR_CPS
from rocky_driver.registers import CENTER, MAPS, max_speed_cps

K = 4096 / 360.0                                      # STS counts per servo degree
TICK = 0.02


def pebble_bus():
    t = make_pebble_mock()
    fams = {i: Family.STS for i in range(1, 16)} | {i: Family.SCS for i in range(16, 21)}
    return FeetechBus(t, fams), t


def expected(step_counts, cap=None, fam=Family.STS):
    cap = max_speed_cps(fam) if cap is None else cap
    return max(1, min(int(round(max(TRACK_K * step_counts / TICK, TRACK_FLOOR_CPS))), int(cap)))


def test_goal_speeds_are_k_step_over_tick_floored_and_capped():
    bus, _ = pebble_bus()
    prev = {1: 0.0, 2: 10.0, 3: -5.0, 4: 0.0, 16: 0.0}
    goals = {1: 1.0, 2: 10.0, 3: 40.0, 4: 0.02, 5: 3.0, 16: 30.0}
    v = bus.goal_speeds(prev, goals, TICK)
    assert v[1] == expected(11) == 715                # 1 deg = 11 counts: 1.3 x 11 / 20 ms
    assert v[2] == v[4] == TRACK_FLOOR_CPS            # still, or a sub-count wobble: the floor, not 0
    assert v[3] == int(max_speed_cps(Family.STS)) == 3063   # 45 deg in one tick: capped at no-load
    assert v[5] == TRACK_FLOOR_CPS                    # no previous goal: an unknown step is never rushed
    assert v[16] == int(max_speed_cps(Family.SCS))    # the hand's own family cap
    assert bus.goal_speeds(prev, goals, TICK, cap_cps=200)[1] == 200
    assert min(v.values()) > 0


def test_sync_positions_takes_per_servo_speeds_in_one_write():
    bus, t = pebble_bus()
    n = len([e for e in t.log if e[0] == "tx"])
    bus.sync_positions({1: 5.0, 2: -5.0, 3: 0.0}, {1: 300, 2: 1200, 3: 50})
    assert len([e for e in t.log if e[0] == "tx"]) - n == 1
    assert [t.servo(i).get("GOAL_SPEED") for i in (1, 2, 3)] == [300, 1200, 50]
    assert t.servo(2).get("GOAL_POSITION") == CENTER[Family.STS] - round(5 * K)
    bus.sync_positions({1: 6.0}, {})                  # a missing id gets 0 (servo max), written
    assert t.servo(1).get("GOAL_SPEED") == 0
    bus.sync_positions({16: 10.0, 17: 10.0}, {16: 100, 17: 0})
    assert t.servo(16).get("GOAL_SPEED") == 0         # SCS: a 0 in the group -> no speed block at all
    bus.sync_positions({16: 10.0, 17: 10.0}, {16: 100, 17: 120})
    assert [t.servo(i).get("GOAL_SPEED") for i in (16, 17)] == [100, 120]


def test_mock_moves_each_servo_at_its_goal_speed_under_the_no_load_cap():
    bus, t = pebble_bus()
    bus.torque_all([1, 2], True)
    bus.sync_positions({1: 90.0, 2: 90.0}, {1: 300, 2: 9000})
    t.advance(0.1)
    moved = [t.servo(i).get("PRESENT_POSITION") - CENTER[Family.STS] for i in (1, 2)]
    assert moved[0] == pytest.approx(30, abs=1)                          # 300 cps x 0.1 s
    assert moved[1] == pytest.approx(max_speed_cps(Family.STS) * 0.1, abs=1)   # 9000 asked, 3064 got


def test_per_tick_speed_moves_through_the_tick_instead_of_rushing():
    """The fault (B76): at GOAL_SPEED 0 a 1 rad/s target moved the servo for
    ~4 ms of every 20 ms tick and parked it for the rest. Servo 1 streams the
    goal speed, servo 2 the old 0; both track the same ramp on the mock."""
    bus, t = pebble_bus()
    t.auto_advance_s = 0.0
    bus.torque_all([1, 2], True)
    w = math.degrees(1.0)                                                # 57.3 deg/s
    prev, moving, lag = {1: 0.0, 2: 0.0}, {1: 0, 2: 0}, {1: 0, 2: 0}
    for k in range(1, 51):
        goals = {1: w * k * TICK, 2: w * k * TICK}
        v = bus.goal_speeds(prev, goals, TICK)
        bus.sync_positions(goals, {1: v[1], 2: 0})
        for _ in range(20):                                             # 1 ms steps through the tick
            t.advance(0.001)
            for i in (1, 2):
                moving[i] += t.servo(i).get("MOVING")
        for i in (1, 2):
            lag[i] = max(lag[i], abs(t.servo(i).get("GOAL_POSITION") - t.servo(i).get("PRESENT_POSITION")))
        prev = goals
    assert moving[1] / 1000 > 0.7 and moving[2] / 1000 < 0.3, moving    # ~77 % vs ~21 % of the time
    assert lag[1] <= 1 and lag[2] <= 1                                  # both still land each step


def test_soft_stream_writes_goal_speeds_after_its_entry():
    bus, t = pebble_bus()
    robot = PebbleRobot(bus)
    s = SoftStream(robot, blend_s=0.2, tail_s=0.1, entry_speed_cps=1500)
    s.enable()
    sent = []
    real = bus.sync_positions

    def spy(targets, speed=0):
        sent.append((dict(targets), speed if isinstance(speed, int) else dict(speed)))
        return real(targets, speed)
    bus.sync_positions = spy
    now = 0.0
    for k in range(80):
        q = np.zeros((5, 3))
        q[:, 1] = 0.3 + 0.6 * math.sin(2.0 * now)                        # hips swing, knees still
        q[:, 2] = -1.0
        if k == 60:
            q[0, 1] = 1.4                                                # a jump: rate-clamped, speed capped
        s.step(q, now)
        now += TICK
    run = [(k, sp) for k, (_, sp) in enumerate(sent) if isinstance(sp, dict)]
    assert all(sp == 1500 for _, sp in sent[:run[0][0]])                # the entry is unchanged
    hard_cps = 4.7 * 4096 / (2 * math.pi)
    for k, sp in run:
        prev, goals = sent[k - 1][0], sent[k][0]
        for sid, deg in goals.items():
            step = abs(round(deg * K) - round(prev[sid] * K))
            assert sp[sid] == expected(step, hard_cps), (k, sid)
            assert 0 < sp[sid] <= hard_cps
    assert any(sp[3] == TRACK_FLOOR_CPS for _, sp in run)                # the still knee: the floor
    assert max(sp[2] for _, sp in run) == int(hard_cps)                  # the jump: capped
    assert t.servo(2).get("GOAL_SPEED") > 0                              # never 0 after entry


def test_factory_registers_are_named_and_locked_on_the_mock():
    assert MAPS[Family.STS]["MAX_ACC"].addr == 85 and MAPS[Family.STS]["ACC_MULTIPLIER"].addr == 86
    assert MAPS[Family.SCS]["ACC_2"].addr == 83
    bus, t = pebble_bus()
    bus.write_raw(2, 85, bytes((254,)))                                  # locked: refused, like EEPROM
    assert t.servo(2).get("MAX_ACC") == 0
    bus.write_reg(2, "MAX_ACC", 254)                                     # the LOCK dance writes it
    assert t.servo(2).get("MAX_ACC") == 254 and t.servo(2).get("LOCK") == 1
    assert bus.read_reg(2, "FIRMWARE_MAJOR") == 3
