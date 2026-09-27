# rocky_driver: Feetech TTL bus driver (dual protocol, D016)

A pure-Python driver for Pebble's single servo bus. It speaks **protocol 1**
(ST3215 legs, little-endian, IDs 1–15) and **protocol 0** (SCS0009 hands,
big-endian, IDs 16–20) on one wire. It has no ROS dependency (D009): the
bench scripts, the cockpit's hardware bridge (`sim/hw_bridge.py`) and the
ROS 2 bridge (`ros2/rocky_driver`) all import it.

```
rocky_driver/
├── protocol.py    wire framing, checksums, packet build/parse, sign-magnitude codec (golden-tested)
├── registers.py   per-family register maps + unit conversions (VERIFY-ON-BENCH flags)
├── transport.py   Transport ABC; SerialTransport (pyserial) is the hardware path
├── mock.py        byte-level bus simulator: motion + thermal models, faults
├── bus.py         FeetechBus: ping/read/write/sync/scan/telemetry + SafetyMonitor
├── robot.py       PebbleRobot: ID map + calibration bridge to gait space; soft_enable
└── stream.py      SoftStream: soft entry (40 % torque, 200 c/s smoothstep), NaN hold,
                   4.7 rad/s clamp, for loops that own their timing (ros2 hw_bridge_node)
tests/             79 tests, all runnable with zero hardware: pytest driver/tests
```

**Install:** `pip install -e ".[hw]"` from the repo root for the serial
path (pyserial). The mock needs nothing beyond the core install.

**Quick start** (the no-hardware version is in `rocky_driver/__init__.py`):
```python
from rocky_driver import FeetechBus, SerialTransport, PebbleRobot
robot = PebbleRobot(FeetechBus(SerialTransport("/dev/ttyACM0", 1_000_000)))
robot.soft_enable()                                   # goal parked where each servo IS, 40 % torque
robot.send_leg_targets([[0.0, 0.2, -1.6]] * 5, speed_cps=200)
robot.release_limits()                                # full torque once the first move has landed
```
Never call `robot.enable()` first: a Feetech servo enables toward its LAST
goal (D052).

## Layers

| layer | speaks | used by |
|---|---|---|
| `PebbleRobot` / `SoftStream` | gait radians `q[5][3]`, claw fractions | gait engine, cockpit bridge, ROS 2 bridge |
| `FeetechBus` | register names + degrees | bench scripts |
| `protocol` | raw bytes | the wire / the mock |

## Design decisions

- **One bus, two protocols (D016):** the family is looked up per ID. Sync
  writes are grouped per family (one `SYNC_WRITE` for STS, one for SCS);
  `SYNC_READ` is STS-only (SCS servos ignore it, and that is tested).
- **Safety lives in the driver, not the app:** every loop can call
  `SafetyMonitor.step()`. It warns at 60 °C, cuts torque at 65 °C, and
  checks the voltage window and a sustained-load warning at D015's 60 %
  line (`bus.safety` in params).
- **Calibration is data, not code:** `bench/calibration.yaml` holds dir and
  offset per joint. It is per-robot and git-ignored, so back it up with the
  robot. `robot.py` applies it symmetrically in both directions.
- **The mock is byte-faithful:** it parses real packets off the wire,
  honours EEPROM locks and replies with real framing. Checksum faults,
  dropped responses and baud mismatch can be injected. If it passes against
  the mock, the only new variable on hardware is the electrons.
- **Sign-magnitude registers carry their sign bit** (`Reg.sign_bit`): bit 15
  for STS speed and current, bit 10 for `PRESENT_LOAD`, bit 11 for
  `POSITION_OFFSET`. A magnitude that would reach the sign bit raises
  `ValueError`, so `set_position_offset` refuses |offset| > 2047.

## Registers we could not verify from two sources

These are tagged `VERIFY-ON-BENCH` in `registers.py`.
`bench/register_dump.py` archives the truth on day one
([bench runbook](../bench/BENCH_RUNBOOK.md) §2). The highest-value
unknowns:

- **`PRESENT_LOAD` sign bit:** 10 (LeRobot's table) or 15 (our first reading
  of the Feetech SDK). To settle it, push one servo by hand both ways while
  `register_dump.py` runs; a bit-15 servo never sets bit 10.
- **`POSITION_OFFSET` sign bit:** 11 (LeRobot) or 15. Run `pose_check.py`
  after any `calibrate_centers.py --burn`.
- **Baud codes 5 / 6 / 7:** 76 800 / 57 600 / 38 400 here, but
  57 600 / 38 400 / 19 200 in LeRobot. Nothing writes a code above 4 (the
  bus runs at 1 Mbps).
- **SCS0009 model number:** 1284 in LeRobot; the mock uses 0x0505 = 1285.
  The real ST3215 number has never been read either (log `model LE` from
  the first `bus_scan.py`).
- **`PRESENT_CURRENT`:** assumed sign bit 15 and effectively unsigned
  (6.5 mA/count). Neither source lists it.
- Also: the SCS0009 sweep (300°? 220°?), the SCS load-direction bit, and
  STS `PHASE` semantics.
