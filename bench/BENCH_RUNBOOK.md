# Bench runbook: the day the servos land

*The goal is to go from sealed bags to a calibrated, health-monitored,
torque-verified servo set in one sitting, without writing code that day.
Every step has a script, and every script has been rehearsed against the
byte-level simulator (`--mock`). If a step surprises you, write the
surprise in `NOTES_INBOX.md` and keep going or stop. Surprises are data.*

**Rehearse the whole day now, no hardware needed.** From the repo root,
`pip install -e .` gives numpy + pyyaml; the mock needs no pyserial.

```bash
cd bench
python3 bus_scan.py --mock
python3 assign_ids.py --mock --yes
python3 calibrate_centers.py --mock --yes
python3 apply_limits.py --mock --yes
python3 pose_check.py --mock --yes
python3 register_dump.py --mock
python3 thermal_soak.py --mock --ids 2 --minutes 20 --mock-load 62
python3 torque_step.py --mock --id 2 --mass-g 350 --arm-mm 100
```

Each command finishes within about 3 s. Expected: 20 servos found and
assigned, 20 offsets, limits 20/20 verified, pose check 0.00° PASS, a
register dump, the soak ending at 55 °C (PASS), torque step
measured/predicted = 0.86.

**Delete `bench/calibration.yaml` after rehearsing.** The rehearsal writes
made-up offsets for all 20 joints, and a real `calibrate_centers.py` run
merges into whatever file is there. `bench/out/` gets rehearsal CSVs too.

---

## 0. Before anything is plugged in

Parts are rows in [`bom/BOM.csv`](../bom/BOM.csv).

- [ ] **Variant check, per servo, straight out of the bag:** the label must
  say **12 V / 30 kg·cm** (ST3215). The 7.4 V / 19.5 kg·cm variant looks
  identical and does not close the torque budget (D002). Return or exchange
  any 7.4 V units; do not "just try" them.
- [ ] **Count:** the bench kit is 4 × ST3215 (A-01): one leg plus a spare.
  The whole robot is 16 ST3215 + 6 SCS0009 (B-01, B-02). The hands come
  with phase B.
- [ ] **Bench supply check, no servo attached:** use an adjustable CV/CC
  supply (A-04) at 12.0 V with a ~1.5 A current limit for first contact.
  Confirm it with a multimeter before the first servo sees power. The
  fixed-12 V alternative (A-05) has no current limit, so fuse its lead and
  watch the wattmeter.
- [ ] **D016, the rule that saves a servo: an SCS0009 NEVER touches the
  12 V rail.** Hands get 6 V from the UBEC (B-11), fed by the 12 V supply;
  measure 6.0 V at the plug first. TTL data and GND are shared; V+ is
  separate per servo family.
- [ ] **Adapter:** the Waveshare Bus Servo Adapter (A) (A-02) on USB. Check
  `ls /dev/ttyACM*` (or `ttyUSB*`). On Linux, run
  `sudo usermod -aG dialout $USER` once, then log in again.
- [ ] **Install:** `pip install -e ".[hw]"` from the repo root. That adds
  pyserial; numpy and pyyaml are core.

**Wiring for bring-up (one servo):**
```
bench supply 12.0 V ──► adapter servo-power in ──► ST3215 (3-pin: V+, GND, DATA)
host USB ──► adapter
   (hands, phase B: 12 V → 6 V UBEC → separate V+, shared GND + DATA)
```

## 1. First contact (one ST3215)

**Optional, no code:** the Waveshare Servo Driver with ESP32 (A-03) proves
the servo, supply and lead before any script runs. Feed it 12.0 V from the
bench supply (its input is rated 6–12 V, so never a full 3S pack at
12.6 V), put a phone on its `ESP32_DEV` hotspot, open 192.168.4.1, then
Start Searching → Set New ID → Position+. It is a tester only: the scripts
and the robot use the USB adapter (A).

```bash
python3 bus_scan.py --port /dev/ttyACM0
```
Expected: exactly one servo, factory `id 1` at 1 Mbps, ~12 V, room
temperature. Log the `model LE` number in `NOTES_INBOX.md`. The real value
has never been read, and the driver's register map has VERIFY-ON-BENCH flags
waiting on it ([driver/README.md](../driver/README.md)).

**Nothing found?** In order: the supply is on and at 12 V → the 3-pin plug
orientation → more bauds (`--bauds 1000000,500000,250000,128000,115200,57600,38400`)
→ a different USB cable (data, not charge-only) → the adapter's TX/RX (A/B
mode) jumper, if it has one.

## 2. Register archive (as shipped)

```bash
python3 register_dump.py --port /dev/ttyACM0
```
- [ ] Take one dump of the very first servo; it lands in `bench/out/`.
  It is the factory-defaults reference from then on. Re-run after any
  EEPROM change (`--diff` shows what moved).

## 3. ID assignment, ONE AT A TIME

Fresh servos are all `id 1`, so never plug in a whole fresh bag at once.

```bash
python3 assign_ids.py --port /dev/ttyACM0
```
The script works through the map: leg0 is 1 = yaw, 2 = hip, 3 = knee, on to
leg4 at 13/14/15, then hands 16–20. For each id it waits for you to plug in
one servo, burns and verifies the ID, and tells you to **label the case
right away** (an unlabelled servo is a future mystery).

With the bench kit's 4 × ST3215:
- assign leg0 (1, 2, 3) and stop the script after id 3;
- run `--start-from 5` for the spare: it becomes a hip (femur) and drops
  into leg 1 later;
- the hands come with phase B: `--start-from 16`.

- [ ] After the last one, run `bus_scan.py` again: every expected ID
  answers.

## 4. Smoke motion (first commanded move, one servo on the desk)

```python
# python3 - <<'EOF'   (from bench/, or interactively)
import sys; sys.path.insert(0, "../driver")
from rocky_driver import FeetechBus, SerialTransport, soft_enable
bus = FeetechBus(SerialTransport("/dev/ttyACM0", 1_000_000))
soft_enable(bus, [1])                       # goal parked where it IS, 40 % torque (D052)
bus.set_position(1, 15.0, speed_cps=400)    # slow, small
print(bus.telemetry(1))
bus.set_position(1, -15.0, speed_cps=400)
bus.torque(1, False)
# EOF
```
Never switch torque on first. A Feetech servo enables toward its LAST goal,
which can be anywhere; `soft_enable` reads the position, parks the goal
there, lowers the torque limit and only then enables.
- [ ] Motion is smooth both ways, and telemetry temp/volt look sane. **Do
  not skip the torque-off at the end:** a servo holding position on a desk
  edge walks itself off it.

## 5. Center calibration (in the bench jig)

Mount the servos in their printed parts first: zero lives in the assembly,
so calibrating a bare servo is meaningless. You need the bench jig
(`jig_base` + `jig_column`) and the calibration gauges `calib_gauge_hip`
(femur horizontal) and `calib_gauge_knee` (knee −90°). They are all in
[PRINT_PLAN.md](../docs/PRINT_PLAN.md), under Deferred.

```bash
python3 calibrate_centers.py --port /dev/ttyACM0 --only leg0
python3 pose_check.py --port /dev/ttyACM0     # acceptance: < 2° everywhere
```
**Known gap:** `pose_check.py` reads the whole 20-servo map. With servos
missing, as on the one-leg bench, it stops with `NoResponse`. Until it reads
only the servos that answer, check the leg in the cockpit's Hardware panel
instead (§5b): jog each joint to its jig pose and compare.
- **Jig poses:** yaw straight out, femur horizontal, knee at −90°
  (`--knee-at`, default −90), claw fully closed.
- **Where offsets go:** the script stores software offsets in
  `bench/calibration.yaml`. That file is per-robot and git-ignored, so back
  it up with the robot.
- **`--burn`** puts an offset in the STS POSITION_OFFSET EEPROM instead
  (±2047 counts, sign in bit 11, VERIFY-ON-BENCH). Run `pose_check` after
  any `--burn`. An offset too big for the register stays in the yaml.
- **Dir flips:** if the nudge test flips a `dir` sign, the offset is
  re-derived from the same reading (D052), so no second run is needed.
- **One place only (D052):** offsets live in the yaml, unless `--burn`
  (then the EEPROM holds it and the yaml stores 0). `apply_limits.py`
  refuses a servo with both non-zero.
- **Burn the hardware backstop:**
  `python3 apply_limits.py --port /dev/ttyACM0`. It sets MIN/MAX_ANGLE_LIMIT
  to the params joint limits + dir + offset + 2° (`--margin`), then reads
  them back and verifies. Re-run after every recalibration.
- **Over-current backstop (REVIEW §4):** set `PROTECT_CURRENT` to about
  2 A per ST3215, which is 308 counts at 6.5 mA/count. VERIFY that unit
  against the §2 dump first. No script writes it yet, so do it by hand
  with the §4 `bus` (`write_reg` handles the EEPROM lock), then archive it
  with a fresh dump:
  ```python
  for sid in (1, 2, 3, 5):
      bus.write_reg(sid, "PROTECT_CURRENT", 308); print(sid, bus.read_reg(sid, "PROTECT_CURRENT"))
  ```

## 5b. Cockpit bring-up (sim ↔ one real leg, D051/D052)

The cockpit's Hardware panel is `sim/hw_bridge.py`. These are the rules it
enforces, so you know what a refusal means:

1. **Close every bench script first:** one process per port. Connect to
   `mock` once to rehearse, then to the real port.
2. **Scan** (the mirror must be off). Partial legs and hands show in the
   table and are monitored (temp, volt, faults, "lost") but not mirrored,
   so a one-servo-at-a-time bench is covered.
3. **Jog** one servo. The clamp is in the gait frame, through your
   calibration: hip −70…90, knee −150…−20, yaw ±40, claw 0…55. A jog drops
   any stream.
4. **Center / dir / apply limits** from the panel (mirror off). Flipping a
   dir re-derives that joint's offset from the stored jig pose, and the
   event says so.
5. **real2sim** first: move the real leg by hand and the sim leg follows.
   This is the "does the frame match?" check. The torque state is left as
   it is.
6. **sim2real** only with the sim standing still (refused otherwise). The
   entry is soft: goal parked where the leg IS, 40 % torque limit,
   200 cps, a smoothstep blend of ≥ 1.5 s (longer for big gaps), released
   after ≥ 3 s. Walking stays disabled while streaming (no real foot
   contacts yet).
7. **What cuts the mirror by itself:**
   - a fault bit or ≥ 65 °C on any servo of a leg limps that whole leg
     (`hw:cut`);
   - 10 silent ticks drop a leg (`hw:degraded`);
   - 3 silent monitor polls trip the servo (`hw:lost`);
   - no sim targets for 2 s turns the mirror off (`hw:stale`);
   - 25 failed ticks mean the port is lost: it tries to limp, then reopens
     every 2 s (`hw:lost` / `hw:reopen`);
   - non-finite targets keep the leg's last good pose (`hw:nan`);
   - goal steps faster than 4.7 rad/s are capped (`hw:rate`).
8. **Re-arm** a tripped servo or a degraded leg only by torque-on with its
   ids named (the panel's re-arm button). A mirror change or a rescan never
   re-arms.
9. **Stop** turns the mirror off and the legs hold. **Limp** drops torque
   on every present servo. **Disconnect / quit** limps and closes the port.

## 6. Health monitor: always on

Any session longer than a smoke test runs with the monitor in the loop
(`PebbleRobot.health_step()` at 1–5 Hz): warn at 60 °C, **torque-cut at
65 °C** (`bus.safety`). The scripts above already do this.

## 7. Thermal soak: the #1 hardware risk, measured

Put the femur servo in the bench jig, with a lever and mass per §8's table
(stance load ≈ 0.9–1.0 N·m). Note the room temperature.

```bash
python3 thermal_soak.py --port /dev/ttyACM0 --ids 2 --minutes 20 \
        --note "stance load 0.95Nm, 22C room"
```
- [ ] Walking-load soak (~0.7 N·m): expect a comfortable steady state.
- [ ] Stance-load soak (~1.0 N·m): the number that matters. Time-to-60 °C
  is the robot's standing-still budget before the sleep pose must trigger.
- [ ] (Optional, supervised) untucked-manipulation load (~1.4 N·m): D015
  flags this band as a no-go, and watching it climb is the point. Abort
  early; there is no need to reach 65 °C.

File all three time-to-temperature numbers in `NOTES_INBOX.md`.

## 8. Torque step: calibrate trust in the torque model

Use a lever and a bag-of-screws mass (nothing rigid: it needs to fall
gracefully). Predictions use the params stall torque, 2.94 N·m at 12 V.

| test | mass @ 100 mm arm | predicted % stall |
|---|---|---|
| walking-hip proxy | ~700 g | ~23 % |
| stance-hip proxy | ~1000 g | ~33 % |
| no-go demo (optional) | ~1500 g | ~50 % |

```bash
python3 torque_step.py --port /dev/ttyACM0 --id 2 --mass-g 1000 --arm-mm 100
```
- [ ] measured/predicted within 0.8–1.3 means D015's margins are real.
  Outside that, investigate: lever geometry, supply sag under load, the
  wrong variant. The test stops at 8° of sag (`--sag-limit-deg`) or 65 °C.

**What the bench settles in `cad/params.yaml`** (each key is marked VERIFY
today):

| bench test | params key |
|---|---|
| `torque_step.py` | `actuators.st3215.stall_nm_12v` (2.94) |
| `thermal_soak.py`, stance hold ≥ 10 min | `actuators.st3215.continuous_frac` (0.65), `thermal_trip_frac` (0.85), `thermal_trip_s` (180) |
| kitchen scale | `actuators.st3215.mass_g` (60), `actuators.scs0009.mass_g` (13), `mass_hw.*` |
| echo test (no script yet) | `actuators.st3215.latency_s` (0.02) |
| SCS0009 sweep: command 0→300° in steps and watch where it stops | `actuators.scs0009.sweep_deg` (300) |
| `register_dump.py` | the driver's VERIFY-ON-BENCH registers ([driver/README.md](../driver/README.md)) |

## 9. End of day

- [ ] Run `register_dump.py` again (`--diff` against the morning dump) to
  archive what changed (IDs, `PROTECT_CURRENT`, offsets if burned).
- [ ] Put the servos in a labelled bin. Back up `bench/calibration.yaml`
  and `bench/out/` with the robot's files: both are per-robot and
  git-ignored.
- [ ] `NOTES_INBOX.md` gets the model numbers, every VERIFY-ON-BENCH
  answer, caliper measurements (if the calipers are out anyway) and every
  surprise. The measured numbers (IDs, offsets, soak times, torque ratios)
  go on into `BUILD_LOG.md` and params.
- [ ] Next: the whole leg on the jig and checked in its jig pose, then the
  gait engine on real metal through the cockpit (B32).

---

## Appendix: failure modes

| symptom | usual cause |
|---|---|
| no response at any baud | TX/RX swap, charge-only USB cable, dead supply channel |
| responds, then silence after an ID write | two servos shared the old ID: rescan and unplug one (the cockpit's set_id refuses an occupied id) |
| position jumps 180° on a power cycle | multi-turn wrap: power-cycle at center, check ANGULAR_RESOLUTION |
| servo hot at idle | holding torque against gravity: torque off when parked, sleep pose |
| checksum errors under load | brownout: supply current limit too low, or wire gauge too thin |
| SCS0009 dead after one glorious second | it met the 12 V rail. D016 exists for a reason |
