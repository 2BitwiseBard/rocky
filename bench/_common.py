"""Shared plumbing for every bench script: bus construction + mock rehearsal.

Every script accepts:
  --port /dev/ttyACM0   real hardware (Waveshare Bus Servo Adapter A)
  --baud 1000000
  --mock                run against the byte-level simulator instead
  --mock-load PCT       simulated mechanical load in mock mode
  --mock-ids 1-3,5      servos on the simulated bus (default: the full robot, 1-20);
                        1-3 rehearses the one-leg bench
  --yes                 auto-confirm prompts (mock rehearsals / scripted runs)

Mock mode fast-forwards time, so a 20-minute soak rehearses in seconds, and
waits only 2 ms for a reply (the mock answers inside the write), so a scan's
silent ids cost little. Output is line-buffered so a piped run shows progress.

Where a run writes: hardware runs write bench/out/ and the robot's calibration
bench/calibration.yaml; a --mock rehearsal writes (and reads its calibration
from) bench/out/mock/ instead, so a rehearsal can never leave made-up offsets
for a real calibrate_centers.py run to merge into. Use out_dir(args) and
cal_path(args), never the constants, for anything a run writes or reads.
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

try:
    sys.stdout.reconfigure(line_buffering=True)
except (AttributeError, ValueError):
    pass

CAL_PATH = os.path.join(HERE, "calibration.yaml")    # the robot's calibration (hardware runs only)
MOCK_TIMEOUT_S = 0.002       # the mock queues its reply inside write(); 20 ms is for real wire
OUT_DIR = os.path.join(HERE, "out")                  # git-ignored
MOCK_DIR = os.path.join(OUT_DIR, "mock")             # every --mock rehearsal writes here


def out_dir(args) -> str:
    """The directory this run writes to (created): bench/out/, or bench/out/mock/ for --mock."""
    d = MOCK_DIR if getattr(args, "mock", False) else OUT_DIR
    os.makedirs(d, exist_ok=True)
    return d


def cal_path(args) -> str:
    """The calibration file this run reads and writes: bench/calibration.yaml on
    hardware, bench/out/mock/calibration.yaml in a --mock rehearsal."""
    return os.path.join(MOCK_DIR, "calibration.yaml") if getattr(args, "mock", False) else CAL_PATH


def parse_ids(spec: str) -> list[int]:
    """'1-3,5' -> [1, 2, 3, 5] (ranges inclusive, order kept, duplicates dropped)."""
    out: list[int] = []
    for part in str(spec).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = (int(x) for x in part.split("-", 1))
            rng = range(a, b + 1)
        else:
            rng = [int(part)]
        out += [i for i in rng if i not in out]
    return out


def base_parser(desc: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=desc)
    p.add_argument("--port", default=None, help="serial port (e.g. /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=1_000_000)
    p.add_argument("--mock", action="store_true", help="run against the simulator")
    p.add_argument("--mock-load", type=float, default=0.0,
                   help="simulated load %% in mock mode")
    p.add_argument("--mock-ids", default=None,
                   help="servo ids on the simulated bus, e.g. 1-3 (one-leg bench) or 1-3,5; "
                        "default: the full robot")
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
    out_dir(args)
    if args.mock:
        if mock_transport is None and getattr(args, "mock_ids", None):
            fams = default_families()
            mock_transport = MockTransport([MockServo(i, fams.get(i, Family.STS))
                                            for i in parse_ids(args.mock_ids)])
        mt = mock_transport or make_pebble_mock()
        if args.mock_load:
            for s in mt.servos.values():
                s.external_load_pct = args.mock_load
        bus = FeetechBus(mt, default_families(), timeout_s=MOCK_TIMEOUT_S)
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
