"""rocky_driver v0 — Feetech TTL bus driver for Pebble (and later Rocky).

Speaks protocol 0 (SCS0009 hands) and protocol 1 (ST3215/STS3250 legs) on
one shared bus per D016. Pure Python, zero ROS dependency (D009): the same
module is imported by bench scripts today and wrapped by ros2_control later.

Quick start (no hardware):
    from rocky_driver import FeetechBus, make_pebble_mock, PebbleRobot
    bus = FeetechBus(make_pebble_mock())
    robot = PebbleRobot(bus)
    robot.enable(); robot.send_leg_targets([[0.0, 0.2, -1.6]] * 5)

With hardware:
    from rocky_driver import FeetechBus, SerialTransport, PebbleRobot
    bus = FeetechBus(SerialTransport("/dev/ttyACM0", 1_000_000))
    robot = PebbleRobot(bus)
"""
from .protocol import Family, Instr, ChecksumError, Packet
from .registers import STS, SCS, MAPS, BAUD_CODES
from .transport import Transport, SerialTransport
from .mock import MockServo, MockTransport, make_pebble_mock, make_factory_fresh_mock
from .bus import (FeetechBus, Telemetry, SafetyMonitor, SafetyLimits,
                  BusError, NoResponse)
from .robot import PebbleRobot, load_bus_params, load_calibration

__version__ = "0.1.0"
__all__ = [
    "Family", "Instr", "ChecksumError", "Packet", "STS", "SCS", "MAPS",
    "BAUD_CODES", "Transport", "SerialTransport", "MockServo", "MockTransport",
    "make_pebble_mock", "make_factory_fresh_mock", "FeetechBus", "Telemetry",
    "SafetyMonitor", "SafetyLimits", "BusError", "NoResponse", "PebbleRobot",
    "load_bus_params", "load_calibration",
]
