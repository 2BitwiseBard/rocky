"""Bus-level operations against the byte-level mock: the dual-protocol story."""
import pytest
from rocky_driver import (FeetechBus, Family, MockServo, MockTransport,
                          make_pebble_mock, NoResponse)
from rocky_driver.registers import CENTER


def pebble_bus():
    t = make_pebble_mock()
    fams = {i: Family.STS for i in range(1, 16)} | \
           {i: Family.SCS for i in range(16, 21)}
    return FeetechBus(t, fams), t


# ---------------------------------------------------------------- basics
def test_ping_present_and_absent():
    bus, _ = pebble_bus()
    assert bus.ping(1)
    assert bus.ping(20)
    assert not bus.ping(42)


def test_read_write_register_sts():
    bus, _ = pebble_bus()
    bus.write_reg(3, "GOAL_POSITION", 3000)
    assert bus.read_reg(3, "GOAL_POSITION") == 3000


def test_read_write_register_scs_big_endian_on_the_wire():
    bus, t = pebble_bus()
    bus.write_reg(17, "GOAL_POSITION", 0x0201)
    # inspect actual servo memory: big-endian hi=0x02 lo=0x01 at addr 42/43
    servo = t.servo(17)
    assert servo.mem[42] == 0x02 and servo.mem[43] == 0x01
    assert bus.read_reg(17, "GOAL_POSITION") == 0x0201


def test_same_value_different_bytes_per_family():
    """The heart of D016: identical logical write, different wire bytes."""
    bus, t = pebble_bus()
    bus.write_reg(1, "GOAL_POSITION", 0x0102)   # STS
    bus.write_reg(16, "GOAL_POSITION", 0x0102)  # SCS
    assert t.servo(1).mem[42:44] == bytes((0x02, 0x01))
    assert t.servo(16).mem[42:44] == bytes((0x01, 0x02))


def test_unknown_register_for_family_raises():
    bus, _ = pebble_bus()
    with pytest.raises(KeyError):
        bus.read_reg(16, "PRESENT_CURRENT")     # SCS0009 has no current sense


# ---------------------------------------------------------------- eeprom
def test_eeprom_write_respects_lock():
    bus, t = pebble_bus()
    servo = t.servo(5)
    assert servo.get("LOCK") == 1
    bus.write_reg(5, "MAX_TEMP", 62)            # driver unlocks, writes, relocks
    assert servo.get("MAX_TEMP") == 62
    assert servo.get("LOCK") == 1


def test_raw_eeprom_write_while_locked_is_ignored_by_servo():
    bus, t = pebble_bus()
    servo = t.servo(5)
    before = servo.get("MAX_TEMP")
    bus.write_raw(5, 13, bytes((99,)))          # no unlock -> servo ignores
    assert servo.get("MAX_TEMP") == before


def test_set_id_full_sequence():
    bus, t = pebble_bus()
    bus.set_id(1, 77)
    assert not bus.ping(1)
    assert bus.ping(77)
    assert bus.read_reg(77, "ID") == 77
    assert t.servo(77).get("LOCK") == 1         # re-locked after the move
    assert bus.family_of(77) is Family.STS      # family map followed the move


# ---------------------------------------------------------------- motion
def test_set_position_and_motion_model():
    bus, t = pebble_bus()
    bus.torque(2, True)
    bus.set_position(2, 45.0)                   # 45 deg -> 2048+512 = 2560
    assert t.servo(2).get("GOAL_POSITION") == 2560
    t.advance(1.0)                              # plenty of time at default speed
    tel = bus.telemetry(2)
    assert tel.position_counts == 2560
    assert abs(tel.position_deg - 45.0) < 0.1


def test_sync_positions_mixed_families_two_transactions():
    bus, t = pebble_bus()
    n_tx = len([e for e in t.log if e[0] == "tx"])
    bus.sync_positions({1: 10.0, 2: -10.0, 16: 20.0, 20: 5.0})
    tx = [e for e in t.log if e[0] == "tx"][n_tx:]
    assert len(tx) == 2                          # one SYNC_WRITE per family
    assert t.servo(1).get("GOAL_POSITION") == 2048 + round(10 * 4096 / 360)
    assert t.servo(16).get("GOAL_POSITION") == 512 + round(20 * 1024 / 300)


def test_torque_all_sync():
    bus, t = pebble_bus()
    bus.torque_all(list(range(1, 21)), True)
    assert all(t.servo(i).get("TORQUE_ENABLE") == 1 for i in range(1, 21))


# ---------------------------------------------------------------- telemetry
def test_telemetry_units():
    bus, t = pebble_bus()
    s = t.servo(4)
    s.put("PRESENT_VOLTAGE", 118)
    s._temp_f = 41.0                # advance() republishes this to the register
    s.external_load_pct = 30.0
    s.put("TORQUE_ENABLE", 1)
    s.advance(0.01)
    tel = bus.telemetry(4)
    assert abs(tel.voltage_v - 11.8) < 1e-9
    assert tel.temp_c == 41
    assert abs(tel.load_pct - 30.0) < 0.5
    assert tel.current_a is not None and tel.current_a > 0.5


def test_sync_telemetry_sts_one_transaction_plus_scs_loop():
    bus, t = pebble_bus()
    ids = [1, 2, 3, 16, 17]
    n_tx = len([e for e in t.log if e[0] == "tx"])
    tels = bus.sync_telemetry(ids)
    tx = [e for e in t.log if e[0] == "tx"][n_tx:]
    assert set(tels) == set(ids)
    assert len(tx) == 1 + 2                      # 1 SYNC_READ + 2 SCS reads
    assert tels[16].family is Family.SCS
    assert tels[1].current_a is not None
    assert tels[16].current_a is None


def test_scs_ignores_sync_read():
    """Protocol quirk we depend on: SYNC_READ including an SCS id must not
    hang the bus — the SCS servo stays silent and we read it individually."""
    t = MockTransport([MockServo(1, Family.STS), MockServo(16, Family.SCS)])
    bus = FeetechBus(t, {1: Family.STS, 16: Family.SCS})
    tels = bus.sync_telemetry([1, 16])
    assert set(tels) == {1, 16}


# ---------------------------------------------------------------- robustness
def test_retry_on_corrupted_checksum():
    bus, t = pebble_bus()
    t.corrupt_next_checksum = 1
    tel = bus.telemetry(1)                       # retries transparently
    assert tel.position_counts == CENTER[Family.STS]
    assert bus.stats["retries"] >= 1
    assert bus.stats["checksum_errors"] >= 1


def test_retry_on_dropped_response_then_fail_cleanly():
    bus, t = pebble_bus()
    t.drop_next_response = 1
    assert bus.ping(1)                           # one drop -> retry succeeds
    t.drop_next_response = 10
    with pytest.raises(NoResponse):
        bus.read_raw(1, 56, 2)


def test_noise_bytes_before_response_are_skipped():
    bus, t = pebble_bus()
    t.noise_before_next = b"\x00\xff\x13"
    tel = bus.telemetry(2)
    assert tel.servo_id == 2


def test_baud_mismatch_silence_and_scan_recovery():
    t = make_pebble_mock()
    for s in t.servos.values():
        s.baud = 500_000                         # servos left at a slower baud
    fams = {i: Family.STS for i in range(1, 16)} | \
           {i: Family.SCS for i in range(16, 21)}
    bus = FeetechBus(t, fams, timeout_s=0.005, retries=0)
    assert not bus.ping(1)                       # host at 1 M: silence
    found = bus.scan(id_range=range(1, 21), bauds=(1_000_000, 500_000))
    assert len(found) == 20
    assert all(v["baud"] == 500_000 for v in found.values())


def test_scan_finds_everyone_with_models():
    bus, _ = pebble_bus()
    found = bus.scan(id_range=range(1, 25), bauds=(1_000_000,))
    assert sorted(found) == list(range(1, 21))
    assert found[1]["model_le"] == 0x0309        # STS mock model, little-endian
    assert found[16]["model_be"] == 0x0505       # SCS mock model, big-endian
