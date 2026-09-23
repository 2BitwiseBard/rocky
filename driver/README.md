# rocky_driver v0 — Feetech TTL bus driver (dual protocol, D016)

Pure-Python driver for Pebble's single servo bus: **protocol 1**
(ST3215 / STS3250 legs, little-endian, IDs 1–15) and **protocol 0**
(SCS0009 hands, big-endian, IDs 16–20) on one wire. No ROS dependency
(D009): bench scripts import it today, ros2_control wraps it later
(`ros2/rocky_driver`).

```
rocky_driver/
├── protocol.py    wire framing, checksums, packet build/parse (golden-tested)
├── registers.py   per-family register maps + unit conversions (VERIFY flags)
├── transport.py   Transport ABC: SerialTransport (pyserial) — the hardware path
├── mock.py        byte-level bus simulator: motion + thermal models, faults
├── bus.py         FeetechBus: ping/read/write/sync/scan/telemetry + SafetyMonitor
└── robot.py       PebbleRobot: ID map + calibration bridge to gait-space
tests/             58 tests, all runnable with zero hardware: pytest tests/
```

## Layers

| layer | speaks | used by |
|---|---|---|
| `PebbleRobot` | gait radians `q[5][3]`, claw fractions | gait engine, ros2_control stub |
| `FeetechBus` | register names + degrees | bench scripts |
| `protocol` | raw bytes | the wire / the mock |

## Design decisions inherited

- **One bus, two protocols (D016):** family is looked up per-ID; sync writes
  are grouped per family (one `SYNC_WRITE` for STS, one for SCS); `SYNC_READ`
  is STS-only (SCS servos ignore it — tested).
- **Safety is in the driver, not the app (plan §7):** every loop can call
  `SafetyMonitor.step()` — warn 60 °C, torque-cut 65 °C, voltage window,
  sustained-load warning at D015's 60 % line.
- **Calibration is data, not code:** `bench/calibration.yaml` (dir/offset per
  joint) rides in git; `robot.py` applies it symmetrically both directions.
- **Mock is byte-faithful:** the simulator parses real packets off the wire,
  honors EEPROM locks, replies with real framing (checksum faults, dropped
  responses and baud mismatch injectable). If it passes against the mock, the
  only new variables on hardware are electrons.

## Registers we could not verify from two sources

Tagged `VERIFY-ON-BENCH` in `registers.py`; `bench/register_dump.py` archives
the truth on day one. Highest-value unknowns: real MODEL numbers, SCS0009
sweep (300°? 220°?), SCS load-direction bit, STS PHASE semantics.
