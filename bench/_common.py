"""Shared plumbing for every bench script: bus construction + mock rehearsal.

Every script accepts:
  --port /dev/ttyACM0   real hardware (Waveshare Bus Servo Adapter A)
  --baud 1000000
  --mock                run against the byte-level simulator instead
  --mock-load PCT       simulated mechanical load in mock mode
  --yes                 auto-confirm prompts (mock rehearsals / scripted runs)

Mock mode fast-forwards time, so a 20-minute soak rehearses in seconds.
"""
from __future__ import annotations
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "driver"))

from rocky_driver import (FeetechBus, Family, SerialTransport,     # noqa: E402
                          make_pebble_mock, load_bus_params)
from rocky_driver.mock import MockServo, MockTransport             # noqa: E402

CAL_PATH = os.path.join(HERE, "calibration.yaml")
OUT_DIR = os.path.join(HERE, "out")


def base_parser(desc: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=desc)
    p.add_argument("--port", default=None, help="serial port (e.g. /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=1_000_000)
    p.add_argument("--mock", action="store_true", help="run against the simulator")
    p.add_argument("--mock-load", type=float, default=0.0,
                   help="simulated load %% in mock mode")
    p.add_argument("--yes", "-y", action="store_true", help="auto-confirm prompts")
    return p


class Clock:
    """Real time on hardware; fast-forward in mock mode (transport.advance)."""

    def __init__(self, mock_transport: MockTransport | None):
        self._mt = mock_transport
        self._t = 0.0

    @property
    def is_mock(self) -> bool:
        return self._mt is not None

    def sleep(self, dt: float) -> None:
        if self._mt is not None:
            self._mt.advance(dt)
            self._t += dt
        else:
            time.sleep(dt)

    def now(self) -> float:
        return self._t if self._mt is not None else time.monotonic()


def default_families() -> dict[int, Family]:
    bp = load_bus_params()
    fams = {i: Family.STS for leg in bp["leg_ids"] for i in leg}
    fams |= {i: Family.SCS for i in bp["hand_ids"]}
    return fams


def make_bus(args, mock_transport: MockTransport | None = None):
    """Returns (bus, clock, mock_transport_or_None)."""
    os.makedirs(OUT_DIR, exist_ok=True)
    if args.mock:
        mt = mock_transport or make_pebble_mock()
        if args.mock_load:
            for s in mt.servos.values():
                s.external_load_pct = args.mock_load
        bus = FeetechBus(mt, default_families(), timeout_s=0.02)
        return bus, Clock(mt), mt
    if not args.port:
        sys.exit("need --port (hardware) or --mock (rehearsal). "
                 "Tip: ls /dev/ttyACM* /dev/ttyUSB*")
    bus = FeetechBus(SerialTransport(args.port, args.baud), default_families())
    return bus, Clock(None), None


def confirm(msg: str, args) -> bool:
    if args.yes:
        print(f"{msg} [auto-yes]")
        return True
    return input(f"{msg} [y/N] ").strip().lower() in ("y", "yes")


def hline(ch="-", n=72):
    print(ch * n)
