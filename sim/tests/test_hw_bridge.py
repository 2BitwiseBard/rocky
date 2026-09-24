"""D052: the hardware bridge's safety rules, fault-injected on the byte-level mock bus.

Every test runs a real HardwareBridge thread against `make_pebble_mock()` (the
bench scripts' simulator), so timing is real time. Constants the rules are
built from are shrunk per test with monkeypatch where the default would only
make the test slower, never where the default IS the thing under test
(test_soft_entry runs the real 1.5 s / 3 s / 200 cps / 40 % numbers).
"""
import math
import os
import subprocess
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
for sub in ("gait", "sim", "driver"):
    sys.path.insert(0, os.path.join(ROOT, sub))

import hw_bridge                                                          # noqa: E402
from hw_bridge import Refused                                             # noqa: E402
from rocky_driver import Transport                                        # noqa: E402
from rocky_driver.registers import CENTER, COUNTS                         # noqa: E402
from rocky_driver.protocol import Family                                  # noqa: E402

K = COUNTS[Family.STS] / 360.0                        # counts per servo degree
ALL = range(1, 21)


def cnt(deg):
    return CENTER[Family.STS] + round(deg * K)


def wait_for(pred, timeout=2.0, dt=0.02):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if pred():
            return True
        time.sleep(dt)
    return pred()


def pose(hip=0.2, knee=-0.45):
    q = np.zeros((5, 3))
    q[:, 1], q[:, 2] = hip, knee
    return q


@pytest.fixture
def make(tmp_path, monkeypatch):
    monkeypatch.setattr(hw_bridge, "CAL_PATH", str(tmp_path / "calibration.yaml"))
    made = []

    def _make(fast=True, rest_knee_deg=-30.0, **kw):
        if fast:
            for k, v in dict(BLEND_S=0.2, ENTRY_S=0.3, ENTRY_TAIL_S=0.1, ENTRY_SPEED_CPS=1500,
                             MONITOR_S=0.1).items():
                monkeypatch.setattr(hw_bridge, k, v)
        ev = []
        b = hw_bridge.HardwareBridge("mock", on_event=lambda k, m: ev.append((k, m)), **kw)
        b.events = ev
        made.append(b)
        with b.bus_lock:                              # limp knees rest folded, inside the soft range
            for ids in b.leg_ids:
                s = b.mock.servo(ids[2])
                s._pos_f = float(cnt(rest_knee_deg))
                s.put("PRESENT_POSITION", cnt(rest_knee_deg))
        b.scan(id_range=ALL)
        return b

    yield _make
    for b in made:
        b.close()


def kinds(b):
    return [k for k, _ in b.events]


def goal(b, sid):
    return b.mock.servo(sid).get("GOAL_POSITION")


def enter(b, q=None):
    b.push_targets(pose() if q is None else q)
    b.set_mirror("sim2real")


# ------------------------------------------------------------------ soft entry
def test_soft_entry_blends_from_the_real_pose_at_low_torque(make):
    """Finding 1: the first sim->robot move snapped at servo max speed, full torque."""
    idle = [False]
    b = make(fast=False, is_idle=lambda: idle[0])
    with pytest.raises(Refused, match="standstill"):
        enter(b)
    idle[0] = True
    b.q_out = None                                    # never streamed
    with pytest.raises(Refused, match="not streaming"):
        b.set_mirror("sim2real")
    s2 = b.mock.servo(2)
    assert s2.get("TORQUE_ENABLE") == 0
    enter(b)                                          # hip 0 -> 11.5 deg, knee -30 -> -25.8 deg
    t0 = time.monotonic()
    # parked where it IS before torque came on, at 40 % torque, entry speed, gentle ACC
    assert s2.get("TORQUE_ENABLE") == 1 and s2.get("TORQUE_LIMIT") == 400
    assert s2.get("ACC") == hw_bridge.ENTRY_ACC and s2.get("GOAL_SPEED") == 200
    assert abs(goal(b, 2) - CENTER[Family.STS]) <= 1
    assert b.status()["entry"]["0"]["blend_s"] == pytest.approx(1.5)
    assert not b.allow_locomotion
    target = cnt(math.degrees(0.2))
    while time.monotonic() - t0 < 0.3:
        b.push_targets(pose())
        time.sleep(0.02)
    early = goal(b, 2) - CENTER[Family.STS]
    assert 0 <= early < 0.35 * (target - CENTER[Family.STS])      # smoothstep(0.2) ~ 0.10 of the way
    assert s2.get("GOAL_SPEED") == 200                             # never 0 (= servo max) during entry
    while time.monotonic() - t0 < 2.0:                             # blend 1.5 s lands
        b.push_targets(pose())
        time.sleep(0.02)
    assert abs(s2.get("PRESENT_POSITION") - target) <= 3
    assert s2.get("TORQUE_LIMIT") == 400                           # still limited: window is 3 s
    assert wait_for(lambda: (b.push_targets(pose()), b.status()["entry"] is None)[1], 1.6)
    assert s2.get("TORQUE_LIMIT") == 1000 and s2.get("ACC") == 0
    assert "entry" in kinds(b)
    assert not b.allow_locomotion                                  # no real contacts yet
    b.locomotion_ok = True
    assert b.allow_locomotion
    assert b.errors == 0


# ------------------------------------------------------------------ partial telemetry
def test_silent_servo_degrades_its_leg_only_and_trips_as_lost(make):
    """Finding 2: one late servo blanked the whole group; real2sim kept stale q."""
    b = make()
    enter(b)
    b.mock.servo(5).silent = True                                  # leg1 hip browns out
    assert wait_for(lambda: b.status()["degraded"][1], 2.0)
    assert wait_for(lambda: 5 in b.status()["lost"], 2.0)
    st = b.status()
    assert st["mirror"] == "sim2real"
    assert st["mirror_legs"] == [True, False, True, True, True]
    assert st["trip_reason"]["5"] == "lost"
    row = {r["id"]: r for r in st["tel"]}
    assert row[5]["missed"] >= hw_bridge.DEGRADE_TICKS and row[4]["missed"] == 0
    assert row[4]["age_s"] < 0.3 and len(row) == 20               # everyone else still reads, hands too
    assert b.errors == 0                                           # a silent servo is not an exception
    assert {"degraded", "lost"} <= set(kinds(b))
    q_real, legs = b.real_pose()
    assert not legs[1] and legs[0]
    # the healthy legs keep streaming
    q = pose(hip=0.3)
    assert wait_for(lambda: (b.push_targets(q), abs(goal(b, 8) - cnt(math.degrees(0.3))) <= 2)[1], 1.5)
    # recovery needs an explicit re-arm: not the servo answering, not a mirror change
    b.mock.servo(5).silent = False
    time.sleep(0.2)
    b.set_mirror("off")
    enter(b, q)
    assert 5 in b.status()["tripped"] and not b.status()["mirror_legs"][1]
    b.push_targets(q)
    assert sorted(b.torque(True, [4, 5, 6])) == [4, 5, 6]
    st = b.status()
    assert 5 not in st["tripped"] and st["mirror_legs"][1] and not st["degraded"][1]
    assert "rearm" in kinds(b)


def test_partial_leg_is_tabulated_and_monitored(make, monkeypatch):
    """The runbook's one-servo-at-a-time bench: a partial leg got ZERO monitoring."""
    b = make()
    with b.bus_lock:
        del b.mock.servos[6]                                       # leg1 knee not built yet
    b.scan(id_range=ALL)
    st = b.status()
    assert st["legs"][1] is False and 4 in st["present"] and 5 in st["present"]
    assert b.jog(4, 10.0) == pytest.approx(10.0) and b.jog(5, 5.0) == pytest.approx(5.0)
    assert wait_for(lambda: {r["id"]: r for r in b.status()["tel"]}[4]["pos_deg"] is not None, 1.0)
    with b.bus_lock:
        b.mock.servo(4)._temp_f = 66.0                             # past the 65 C cut
    assert wait_for(lambda: 4 in b.status()["tripped"], 1.0)
    assert b.mock.servo(4).get("TORQUE_ENABLE") == 0 and b.mock.servo(5).get("TORQUE_ENABLE") == 0
    assert "cut" in kinds(b)
    with pytest.raises(Refused, match="tripped"):
        b.jog(4, 0.0)


# ------------------------------------------------------------------ faults cut legs
def test_fault_bit_at_tick_rate_limps_the_whole_leg(make, monkeypatch):
    b = make()
    monkeypatch.setattr(hw_bridge, "MONITOR_S", 100.0)             # prove the 25 Hz path, not the poll
    enter(b)
    assert wait_for(lambda: all(b.mock.servo(i).get("TORQUE_ENABLE") for i in range(1, 16)), 1.0)
    with b.bus_lock:
        b.mock.servo(8).put("STATUS", 0b100)                       # overheat bit on leg2 hip
    assert wait_for(lambda: 8 in b.status()["tripped"], 0.5)
    assert [b.mock.servo(i).get("TORQUE_ENABLE") for i in (7, 8, 9)] == [0, 0, 0]
    assert all(b.mock.servo(i).get("TORQUE_ENABLE") for i in (1, 2, 3, 10, 11, 12))
    st = b.status()
    assert st["mirror"] == "sim2real" and st["mirror_legs"] == [True, True, False, True, True]
    assert "fault" in st["trip_reason"]["8"]
    cut = [m for k, m in b.events if k == "cut"]
    assert cut and "[7, 8, 9]" in cut[0]
    with b.bus_lock:
        b.mock.servo(11)._temp_f = 66.0                            # over-temperature on leg3 too
    assert wait_for(lambda: 11 in b.status()["tripped"], 0.5)
    assert [b.mock.servo(i).get("TORQUE_ENABLE") for i in (10, 11, 12)] == [0, 0, 0]
    b.set_mirror("off")                                            # a mirror change never re-arms
    enter(b)
    st = b.status()
    assert {8, 11} <= set(st["tripped"]) and b.mock.servo(8).get("TORQUE_ENABLE") == 0


# ------------------------------------------------------------------ NaN guard
def test_nan_push_keeps_last_good_targets(make):
    b = make()
    q = pose()
    enter(b, q)
    assert wait_for(lambda: (b.push_targets(q), b.status()["entry"] is None)[1], 2.0)
    g2 = goal(b, 2)
    bad = q.copy()
    bad[0, 1] = np.nan
    bad[1, :] = np.inf
    bad[2, 1] = 0.3
    t_end = time.monotonic() + 0.3
    while time.monotonic() < t_end:
        b.push_targets(bad)
        time.sleep(0.01)
    assert np.allclose(b.q_out[0], q[0]) and np.allclose(b.q_out[1], q[1]) and b.q_out[2, 1] == 0.3
    assert goal(b, 2) == g2
    assert abs(goal(b, 8) - cnt(math.degrees(0.3))) <= 2
    assert b.nan_count > 0 and b.errors >= b.nan_count and "nan" in kinds(b)
    assert sum(k == "nan" for k in kinds(b)) <= 2                  # throttled, not one per push


def test_leg_that_was_never_finite_is_never_written(make):
    b = make()
    q = pose()
    q[3, :] = np.nan
    enter(b, q)
    park = goal(b, 11)                                             # entry parked it where it stood
    assert wait_for(lambda: (b.push_targets(q), b.nan_skips > 3)[1], 1.0)
    # the others stream (waited for: at the 50 Hz stream, V2, nan_skips > 3 comes in 80 ms)
    assert wait_for(lambda: (b.push_targets(q), abs(goal(b, 2) - cnt(math.degrees(0.2))) <= 40)[1], 1.0)
    assert goal(b, 11) == park


# ------------------------------------------------------------------ rate limit
def test_rate_limit_caps_or_refuses_a_step(make):
    b = make()
    cap = math.degrees(4.7) * 0.04
    b._last_goal[2] = 0.0
    assert b._rate_limit(2, 60.0, 0.04) == pytest.approx(cap)
    assert b._rate_limit(2, -5.0, 0.04) == -5.0
    b.rate_mode = "refuse"
    assert b._rate_limit(2, 60.0, 0.04) is None and b.rate_refused == 1
    assert b.rate_capped == 1 and "rate" in kinds(b)
    b.rate_mode = "cap"
    q = pose()
    enter(b, q)
    assert wait_for(lambda: (b.push_targets(q), b.status()["entry"] is None)[1], 2.0)
    jump = pose(hip=1.4)                                           # +68.8 deg in one push
    b.push_targets(jump)
    t0 = time.monotonic()
    time.sleep(0.08)
    moved = (goal(b, 2) - cnt(math.degrees(0.2))) / K
    el = time.monotonic() - t0
    assert moved < math.degrees(4.7) * (el + 0.1) + 1.0            # a hard-speed ramp, not a 69 deg step
    assert moved < 68.0


# ------------------------------------------------------------------ heartbeat / port
def test_heartbeat_drops_the_mirror_when_the_sim_stops(make, monkeypatch):
    b = make()
    monkeypatch.setattr(hw_bridge, "STALE_S", 0.3)
    enter(b)
    assert wait_for(lambda: b.mirror == "off", 1.5)
    assert "stale" in kinds(b)
    assert b.mock.servo(2).get("TORQUE_ENABLE") == 1               # the legs hold


class _Unplugged(Transport):
    def write(self, data):
        raise OSError("device reports readiness to read but returned no data")

    def read(self, n, t):
        raise OSError("device disconnected")


def test_lost_port_goes_off_limps_and_reopens(make, monkeypatch):
    b = make()
    monkeypatch.setattr(hw_bridge, "LOST_ERRORS", 5)
    monkeypatch.setattr(hw_bridge, "REOPEN_S", 0.3)
    enter(b)
    with b.bus_lock:
        b.bus.t = _Unplugged()
    assert wait_for(lambda: not b.port_ok, 1.5)
    assert b.mirror == "off" and "lost" in kinds(b)
    with pytest.raises(Refused, match="lost"):
        b.jog(2, 0.0)
    assert wait_for(lambda: b.port_ok, 1.5)                        # the mock "replugs" on reopen
    assert "reopen" in kinds(b)
    assert b.mock.servo(2).get("TORQUE_ENABLE") == 0               # limp after a loss: state unknown


# ------------------------------------------------------------------ refusals
def test_scan_set_id_and_calibration_refuse_while_mirroring(make):
    b = make()
    enter(b)
    for call in (lambda: b.scan(id_range=ALL), lambda: b.set_id(20, 25), lambda: b.center("leg0_hip"),
                 lambda: b.set_dir("leg0_hip", -1), lambda: b.apply_limits()):
        with pytest.raises(Refused, match="mirror off"):
            call()
    b.set_mirror("off")
    with pytest.raises(Refused, match="already on the bus"):
        b.set_id(1, 2)
    with pytest.raises(Refused, match="not present"):
        b.set_id(29, 30)
    assert b.set_id(20, 25) == 25
    assert 25 in b.status()["present"] and 20 not in b.status()["present"]
    with pytest.raises(Refused, match="not in the params bus map"):
        b.jog(25, 5.0)


# ------------------------------------------------------------------ jog frame + calibration
def test_jog_clamps_in_the_gait_frame_through_the_calibration(make):
    b = make()
    assert b.jog(2, 80.0) == 80.0                                  # hip [-70, 90], dir +1
    assert b.set_dir("leg0_hip", -1) == -1
    assert b.jog(2, 80.0) == 70.0                                  # q = -80 -> -70 -> servo +70
    assert b.jog(2, -100.0) == -90.0
    b.set_dir("leg0_knee", -1)                                     # knee [-150, -20] mirrored = [20, 150]
    assert b.jog(3, 10.0) == 20.0 and b.jog(3, 170.0) == 150.0
    with b.bus_lock:                                               # yaw rests 100 counts off center
        b.mock.servo(1)._pos_f = float(CENTER[Family.STS] + 100)
        b.mock.servo(1).put("PRESENT_POSITION", CENTER[Family.STS] + 100)
    assert b.center("leg0_yaw")["offset"] == 100
    off = 100 / K
    assert b.jog(1, 60.0) == pytest.approx(40.0 + off)             # the clamp moved WITH the offset
    assert b.jog(1, -60.0) == pytest.approx(-40.0 + off)
    assert b.jog(16, 80.0) == 55.0 and b.jog(16, -10.0) == 0.0     # claw 0..55, not +-150
    with pytest.raises(Refused, match="params bus map"):
        b.jog(42, 0.0)


def test_set_dir_rederives_the_offset_from_the_stored_jig(make):
    b = make()
    rest = cnt(-55.0)                                              # knee resting on a -60 deg comb, 5 deg off
    with b.bus_lock:
        b.mock.servo(3)._pos_f = float(rest)
        b.mock.servo(3).put("PRESENT_POSITION", rest)
    r = b.center("leg0_knee", jig_deg=-60.0)
    assert b.cal["meta"]["knee_jig_deg"] == -60.0 and b.cal["meta"]["jig_deg"]["leg0_knee"] == -60.0
    pos_deg = (rest - CENTER[Family.STS]) / K
    assert math.degrees(b.robot.deg_to_q(0, 2, pos_deg)) == pytest.approx(-60.0, abs=0.1)
    b.set_dir("leg0_knee", -1)
    assert b.cal["offset"]["leg0_knee"] != r["offset"]
    assert math.degrees(b.robot.deg_to_q(0, 2, pos_deg)) == pytest.approx(-60.0, abs=0.1)   # still the jig
    assert any("re-derived" in m for k, m in b.events if k == "dir")
    with b.bus_lock:                                               # D052: one offset place only
        b.bus.write_reg(2, "POSITION_OFFSET", 12)
    with pytest.raises(Refused, match="one place"):
        b.center("leg0_hip")


def test_apply_limits_round_trip_and_hardware_clamp(make):
    b = make()
    with b.bus_lock:
        b.mock.servo(1)._pos_f = float(CENTER[Family.STS] + 100)
        b.mock.servo(1).put("PRESENT_POSITION", CENTER[Family.STS] + 100)
    b.center("leg0_yaw")
    plan = b.apply_limits(ids=[1, 2, 3, 16], dry_run=True)
    assert b.mock.servo(2).get("MAX_ANGLE_LIMIT") == COUNTS[Family.STS] - 1   # dry run wrote nothing
    res = b.apply_limits(ids=[1, 2, 3, 16])
    assert all(r["ok"] for r in res.values())
    for sid in (1, 2, 3, 16):
        s = b.mock.servo(sid)
        assert (s.get("MIN_ANGLE_LIMIT"), s.get("MAX_ANGLE_LIMIT")) == (plan[sid]["min"], plan[sid]["max"])
    assert plan[2]["min"] == math.floor(CENTER[Family.STS] - 72 * K)       # soft -70 - 2 deg margin
    assert plan[1]["max"] == math.ceil(CENTER[Family.STS] + 100 + 42 * K)  # yaw +40 + 2, offset applied
    with b.bus_lock:                                               # a raw write past the limit stops at it
        b.bus.set_position(2, 150.0)
        b.bus.torque(2, True)
    assert wait_for(lambda: b.mock.servo(2).get("PRESENT_POSITION") == plan[2]["max"], 1.0)
    time.sleep(0.1)
    assert b.mock.servo(2).get("PRESENT_POSITION") == plan[2]["max"]
    with b.bus_lock:
        b.bus.write_reg(1, "POSITION_OFFSET", 12)                  # EEPROM AND software offset
    r1 = b.apply_limits(ids=[1])[1]
    assert not r1["ok"] and "both non-zero" in r1["error"]


def test_bench_apply_limits_runs_on_the_mock():
    r = subprocess.run([sys.executable, os.path.join(ROOT, "bench", "apply_limits.py"), "--mock", "--yes",
                        "--ids", "1,2,3,16"], capture_output=True, text=True, timeout=60,
                       cwd=os.path.join(ROOT, "bench"))
    assert r.returncode == 0, r.stdout + r.stderr
    assert "4/4 verified" in r.stdout


# ------------------------------------------------------------------ V2 (review of D052)
def test_stream_runs_at_the_bus_rate_telemetry_at_half(make):
    """The sim, ServoModel and the envs plan against a 50 Hz bus; the bridge
    streamed at 25 Hz (every other target + 0-40 ms extra delay)."""
    b = make()
    q = pose()
    enter(b, q)
    assert wait_for(lambda: (b.push_targets(q), b.status()["entry"] is None)[1], 2.0)
    n = {"w": 0, "r": 0}
    sp, rt = b.bus.sync_positions, b.bus.read_telemetry

    def count_w(*a, **k):
        n["w"] += 1
        return sp(*a, **k)

    def count_r(*a, **k):
        n["r"] += 1
        return rt(*a, **k)
    b.bus.sync_positions, b.bus.read_telemetry = count_w, count_r
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end:
        b.push_targets(q)
        time.sleep(0.01)
    b.bus.sync_positions, b.bus.read_telemetry = sp, rt
    assert hw_bridge.TICK_HZ == 50.0
    assert 40 <= n["w"] <= 55, n                  # one stream write per 20 ms tick
    assert 18 <= n["r"] <= 36, n                  # telemetry every 2nd tick (+ the monitor polls)


def test_rearming_a_degraded_leg_goes_through_the_soft_entry(make):
    """Review repro: a degraded leg keeps its torque on, so torque(True, ids)
    skipped the entry and the next tick snapped the leg to the sim pose."""
    b = make(fast=False)
    q = pose()
    enter(b, q)
    b.mock.servo(5).silent = True
    assert wait_for(lambda: (b.push_targets(q), b.status()["degraded"][1])[1], 2.0)
    far = pose(hip=1.2)                                            # the sim moved while the leg was out
    b.mock.servo(5).silent = False
    time.sleep(0.1)
    g0 = goal(b, 5)
    b.push_targets(far)
    time.sleep(0.2)
    assert goal(b, 5) == g0                                        # out of the stream: never written
    assert sorted(b.torque(True, [4, 5, 6])) == [4, 5, 6]
    st = b.status()
    assert st["entry"] is not None and "1" in st["entry"]          # a fresh entry, not a snap
    assert b.mock.servo(5).get("TORQUE_LIMIT") == 400 and b.mock.servo(5).get("GOAL_SPEED") == 200
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.3:
        b.push_targets(far)
        time.sleep(0.02)
    moved = abs(goal(b, 5) - g0) / K
    assert moved < 0.35 * math.degrees(1.2 - 0.2)                  # the smoothstep's start, not 57 deg


def test_interrupted_entry_releases_its_limits(make, monkeypatch):
    """Review repro: a leg degraded DURING its entry kept 40 % torque and ACC 10
    forever (the blend was popped without the release)."""
    b = make()
    monkeypatch.setattr(hw_bridge, "ENTRY_S", 3.0)
    q = pose()
    enter(b, q)
    assert b.mock.servo(5).get("TORQUE_LIMIT") == 400
    b.mock.servo(5).silent = True
    assert wait_for(lambda: (b.push_targets(q), b.status()["degraded"][1])[1], 2.0)
    b.mock.servo(5).silent = False
    b.set_mirror("off")
    assert [b.mock.servo(i).get("TORQUE_LIMIT") for i in (4, 5, 6)] == [1000] * 3
    assert [b.mock.servo(i).get("ACC") for i in (4, 5, 6)] == [0] * 3


def test_entry_parks_a_joint_resting_outside_the_range_inside_the_burned_limits(make):
    """A limp knee resting at 0 deg (soft -150..-20): parking the goal THERE is a
    goal outside the MIN/MAX_ANGLE_LIMIT apply_limits burns (soft -+ 2 deg)."""
    b = make(rest_knee_deg=0.0)
    enter(b)
    lim = -20.0 + 0.5 * 2.0                                        # half a margin inside the burned -18
    assert abs(goal(b, 3) - cnt(lim)) <= 1
    assert b.status()["entry"]["0"]["gap_deg"] < 15.0              # the blend starts from the parked goal
