"""Register sign bits end to end (CORE-01) and the driver constants that mirror
params.yaml (the driver keeps no params dependency, so a test pins them)."""
import os
import sys

import pytest
from rocky_driver import FeetechBus, Family, SafetyMonitor, make_pebble_mock
from rocky_driver import stream
from rocky_driver.bus import FeetechBus as _Bus
from rocky_driver.registers import NO_LOAD_RAD_S, STS

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "gait"))
import rocky_model as rm                                          # noqa: E402


def pebble_bus():
    t = make_pebble_mock()
    fams = {i: Family.STS for i in range(1, 16)} | {i: Family.SCS for i in range(16, 21)}
    return FeetechBus(t, fams), t


def test_register_sign_bits():
    assert STS["PRESENT_LOAD"].sign_bit == 10
    assert STS["POSITION_OFFSET"].sign_bit == 11
    assert STS["PRESENT_SPEED"].sign_bit == 15 and STS["GOAL_SPEED"].sign_bit == 15


def test_decode_tel_reads_a_negative_load_from_bit_10():
    raw = bytearray(15)
    raw[0:2] = (0x00, 0x08)            # position 2048
    raw[4:6] = (0x64, 0x04)            # PRESENT_LOAD raw 0x0464 -> -10.0 %
    raw[6], raw[7] = 120, 30
    tel = _Bus._decode_tel(1, Family.STS, bytes(raw))
    assert tel.load_pct == pytest.approx(10.0)       # magnitude (was 112.4 with bit 15)


def test_mock_negative_load_round_trips_without_a_false_warning():
    bus, t = pebble_bus()
    s = t.servo(2)
    s.external_load_pct = -30.0
    s.put("TORQUE_ENABLE", 1)
    s.advance(0.01)
    assert s.mem[61] & 0x04 and not s.mem[61] & 0x80    # sign in bit 10, not bit 15
    assert bus.read_reg(2, "PRESENT_LOAD") == -300
    tel = bus.telemetry(2)
    assert tel.load_pct == pytest.approx(30.0, abs=0.5)
    events = []
    SafetyMonitor(bus, [2], on_event=lambda k, tel: events.append(k)).step()
    assert "load_warn" not in events


def test_position_offset_uses_bit_11_and_refuses_past_2047():
    bus, t = pebble_bus()
    bus.set_position_offset(3, -100)
    assert bytes(t.servo(3).mem[31:33]) == bytes((0x64, 0x08))
    assert bus.read_reg(3, "POSITION_OFFSET") == -100
    bus.set_position_offset(3, 2047)
    assert bus.read_reg(3, "POSITION_OFFSET") == 2047
    for bad in (2048, -2048, 4000):
        with pytest.raises(ValueError):
            bus.set_position_offset(3, bad)
    assert bus.read_reg(3, "POSITION_OFFSET") == 2047    # nothing written on a refusal


def test_driver_servo_constants_match_params():
    assert NO_LOAD_RAD_S[Family.STS] == pytest.approx(rm.no_load_rad_s())
    assert NO_LOAD_RAD_S[Family.SCS] == pytest.approx(rm.no_load_rad_s("claw"))
    assert stream.HARD_RAD_S == pytest.approx(rm.servo_speed("hard"))
