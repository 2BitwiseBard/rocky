"""Register maps + unit conversions for both Feetech families on Pebble's bus.

Sources: Feetech SCServo SDK header tables (SMS_STS_* / SCSCL_*) cross-checked
against the Waveshare ST3215 register table. Addresses we could not confirm
from two independent sources carry VERIFY-ON-BENCH — `bench/register_dump.py`
prints every byte 0..73 so day-one hardware settles them.

Conventions
-----------
A register is (addr, nbytes, kind):
  kind "u8"  — one byte
  kind "u16" — plain 16-bit (endianness per family)
  kind "sm16"— signed-magnitude 16-bit (bit 15 = sign; STS speed/load/current)
EEPROM registers (addr < 40) persist and need LOCK released before writing
on STS (LOCK addr 55) and SCS (LOCK addr 48).
"""
from __future__ import annotations
from dataclasses import dataclass
from .protocol import Family


@dataclass(frozen=True)
class Reg:
    addr: int
    nbytes: int
    kind: str = "u8"        # "u8" | "u16" | "sm16"
    eeprom: bool = False


# ------------------------------------------------------------------ STS map
# ST3215 / STS3250 — protocol 1, little-endian, 4096 counts/rev
STS = {
    "MODEL":              Reg(3, 2, "u16", eeprom=True),
    "ID":                 Reg(5, 1, eeprom=True),
    "BAUD":               Reg(6, 1, eeprom=True),
    "RETURN_DELAY":       Reg(7, 1, eeprom=True),   # units of 2 us
    "RESPONSE_LEVEL":     Reg(8, 1, eeprom=True),   # 0: reply only READ/PING
    "MIN_ANGLE_LIMIT":    Reg(9, 2, "u16", eeprom=True),
    "MAX_ANGLE_LIMIT":    Reg(11, 2, "u16", eeprom=True),
    "MAX_TEMP":           Reg(13, 1, eeprom=True),  # degC
    "MAX_VOLT":           Reg(14, 1, eeprom=True),  # 0.1 V
    "MIN_VOLT":           Reg(15, 1, eeprom=True),  # 0.1 V
    "MAX_TORQUE":         Reg(16, 2, "u16", eeprom=True),  # 0.1% of stall
    "PHASE":              Reg(18, 1, eeprom=True),  # VERIFY-ON-BENCH (drive phase)
    "UNLOAD_CONDITION":   Reg(19, 1, eeprom=True),  # protection bit mask
    "LED_ALARM":          Reg(20, 1, eeprom=True),
    "KP":                 Reg(21, 1, eeprom=True),
    "KD":                 Reg(22, 1, eeprom=True),
    "KI":                 Reg(23, 1, eeprom=True),
    "MIN_STARTUP_FORCE":  Reg(24, 2, "u16", eeprom=True),
    "CW_DEAD":            Reg(26, 1, eeprom=True),
    "CCW_DEAD":           Reg(27, 1, eeprom=True),
    "PROTECT_CURRENT":    Reg(28, 2, "u16", eeprom=True),  # 6.5 mA units
    "ANGULAR_RESOLUTION": Reg(30, 1, eeprom=True),
    "POSITION_OFFSET":    Reg(31, 2, "sm16", eeprom=True), # counts, bit15 sign
    "MODE":               Reg(33, 1, eeprom=True),  # 0 servo, 1 wheel, 2 pwm, 3 step
    "PROTECT_TORQUE":     Reg(34, 1, eeprom=True),  # % after overload
    "PROTECT_TIME":       Reg(35, 1, eeprom=True),  # 10 ms units
    "OVERLOAD_TORQUE":    Reg(36, 1, eeprom=True),  # % threshold
    "SPEED_KP":           Reg(37, 1, eeprom=True),
    "OVERCURRENT_TIME":   Reg(38, 1, eeprom=True),
    "VELOCITY_KI":        Reg(39, 1, eeprom=True),
    # ---- SRAM ----
    "TORQUE_ENABLE":      Reg(40, 1),
    "ACC":                Reg(41, 1),               # 100 counts/s^2 units
    "GOAL_POSITION":      Reg(42, 2, "u16"),
    "GOAL_TIME":          Reg(44, 2, "u16"),        # ms
    "GOAL_SPEED":         Reg(46, 2, "sm16"),       # counts/s
    "TORQUE_LIMIT":       Reg(48, 2, "u16"),        # 0.1%
    "LOCK":               Reg(55, 1),               # 1 = EEPROM locked
    "PRESENT_POSITION":   Reg(56, 2, "u16"),
    "PRESENT_SPEED":      Reg(58, 2, "sm16"),
    "PRESENT_LOAD":       Reg(60, 2, "sm16"),       # 0.1% of stall
    "PRESENT_VOLTAGE":    Reg(62, 1),               # 0.1 V
    "PRESENT_TEMP":       Reg(63, 1),               # degC
    "ASYNC_ACTION":       Reg(64, 1),
    "STATUS":             Reg(65, 1),               # fault bits, see ERROR_BITS
    "MOVING":             Reg(66, 1),
    "PRESENT_CURRENT":    Reg(69, 2, "sm16"),       # 6.5 mA units
}

# ------------------------------------------------------------------ SCS map
# SCS0009 — protocol 0, big-endian, 1024 counts over ~300 deg (VERIFY sweep)
SCS = {
    "MODEL":              Reg(3, 2, "u16", eeprom=True),
    "ID":                 Reg(5, 1, eeprom=True),
    "BAUD":               Reg(6, 1, eeprom=True),
    "RETURN_DELAY":       Reg(7, 1, eeprom=True),
    "RESPONSE_LEVEL":     Reg(8, 1, eeprom=True),
    "MIN_ANGLE_LIMIT":    Reg(9, 2, "u16", eeprom=True),
    "MAX_ANGLE_LIMIT":    Reg(11, 2, "u16", eeprom=True),
    "MAX_TEMP":           Reg(13, 1, eeprom=True),
    "MAX_VOLT":           Reg(14, 1, eeprom=True),
    "MIN_VOLT":           Reg(15, 1, eeprom=True),
    "MAX_TORQUE":         Reg(16, 2, "u16", eeprom=True),
    "UNLOAD_CONDITION":   Reg(19, 1, eeprom=True),
    "LED_ALARM":          Reg(20, 1, eeprom=True),
    "CW_DEAD":            Reg(26, 1, eeprom=True),
    "CCW_DEAD":           Reg(27, 1, eeprom=True),
    # ---- SRAM ----
    "TORQUE_ENABLE":      Reg(40, 1),
    "GOAL_POSITION":      Reg(42, 2, "u16"),
    "GOAL_TIME":          Reg(44, 2, "u16"),
    "GOAL_SPEED":         Reg(46, 2, "u16"),
    "LOCK":               Reg(48, 1),               # NOTE: 48 here, 55 on STS
    "PRESENT_POSITION":   Reg(56, 2, "u16"),
    "PRESENT_SPEED":      Reg(58, 2, "u16"),
    "PRESENT_LOAD":       Reg(60, 2, "u16"),        # VERIFY-ON-BENCH direction bit
    "PRESENT_VOLTAGE":    Reg(62, 1),
    "PRESENT_TEMP":       Reg(63, 1),
    "MOVING":             Reg(66, 1),
}

MAPS = {Family.STS: STS, Family.SCS: SCS}

# Baud codes (register BAUD) — shared table, Feetech convention
BAUD_CODES = {0: 1_000_000, 1: 500_000, 2: 250_000, 3: 128_000,
              4: 115_200, 5: 76_800, 6: 57_600, 7: 38_400}
BAUD_TO_CODE = {v: k for k, v in BAUD_CODES.items()}

# ------------------------------------------------------------- conversions
COUNTS = {Family.STS: 4096, Family.SCS: 1024}
SWEEP_DEG = {Family.STS: 360.0, Family.SCS: 300.0}   # SCS: VERIFY-ON-BENCH
CENTER = {Family.STS: 2048, Family.SCS: 512}

CURRENT_LSB_A = 0.0065        # STS present-current: 6.5 mA / count
VOLTAGE_LSB_V = 0.1
LOAD_LSB_PCT = 0.1            # present-load: 0.1 % of stall / count


def deg_to_counts(deg: float, family: Family) -> int:
    """Angle relative to center (deg, +CCW looking at the horn) -> raw counts."""
    c = CENTER[family] + deg * COUNTS[family] / SWEEP_DEG[family]
    return max(0, min(COUNTS[family] - 1, round(c)))


def counts_to_deg(counts: int, family: Family) -> float:
    return (counts - CENTER[family]) * SWEEP_DEG[family] / COUNTS[family]


def degps_to_counts(dps: float, family: Family) -> int:
    return round(dps * COUNTS[family] / SWEEP_DEG[family])
