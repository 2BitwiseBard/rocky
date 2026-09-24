# BENCH RUNBOOK — the day the servos land

*Goal: from sealed bags to a calibrated, health-monitored, torque-verified
servo set in one sitting — with zero code written that day. Every step has a
script; every script was rehearsed against the simulator (`--mock`) before
the hardware existed. If a step surprises you, write the surprise in
`NOTES_INBOX.md` and keep going or stop — surprises are data.*

**Rehearse the whole day right now, no hardware needed:**
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

---

## 0. Before anything is plugged in

- [ ] **Variant check, per servo, out of the bag:** the label must say
  **12 V / 30 kg·cm** (ST3215). The 7.4 V / 19.5 kg·cm variant looks
  identical and does not close our torque budget (D002). Any 7.4 V units →
  return/exchange, do not "just try" them.
- [ ] Count: 4× ST3215 + 1× SCS0009 in Batch 1 (17 + 6 total by Batch 2).
- [ ] **Bench PSU sanity, no servo attached:** set 12.0 V, current limit
  ~1.5 A for first contact. Verify with a multimeter before the first servo
  ever sees power.
- [ ] **D016, the rule that saves a servo: SCS0009 NEVER touches the 12 V
  rail.** Hands get 5–6 V (bench PSU second channel or the BEC when it
  arrives). TTL data + GND are shared; V+ is per-family. When in doubt,
  measure the V+ pin before plugging a hand servo.
- [ ] Adapter: Waveshare Bus Servo Adapter (A) on USB. `ls /dev/ttyACM*`
  (or `ttyUSB*`). Linux: `sudo usermod -aG dialout $USER` once, re-login.
- [ ] `pip install pyserial pyyaml` in whatever venv runs the bench.

**Wiring for bring-up (one servo):**
```
PSU 12 V ──► adapter servo-power in ──► ST3215 (3-pin: V+, GND, DATA)
laptop USB ──► adapter
   (hands later: PSU ch2 at 6.0 V ──► separate V+ ── shared GND+DATA)
```

## 1. First contact (one ST3215)

```bash
python3 bus_scan.py --port /dev/ttyACM0
```
Expected: exactly one servo, factory `id 1` at 1 Mbps, ~12 V, room temp.
Log the `model LE` number in NOTES_INBOX (we've never seen the real value —
the driver's register map has VERIFY-ON-BENCH flags waiting on it).

**Nothing found?** In order: PSU actually on and at 12 V → 3-pin plug
orientation → try more bauds (`--bauds 1000000,500000,250000,128000,115200,57600,38400`)
→ different USB cable (data, not charge-only) → adapter's TX/RX jumper (A/B
mode) if present.

## 2. Register archive (as-shipped)

```bash
python3 register_dump.py --port /dev/ttyACM0
```
- [ ] One dump with the very first servo, filed under `bench/out/`.
  This is the factory-defaults reference forever. Re-run after any EEPROM
  change (`--diff` shows what moved).

## 3. ID assignment — ONE AT A TIME

Fresh servos are all `id 1`. Never bulk-plug a fresh bag.

```bash
python3 assign_ids.py --port /dev/ttyACM0
```
The script walks the map (leg0: 1=yaw 2=hip 3=knee … leg4: 13/14/15, hands
16–20), waits for you to plug each servo, burns + verifies the ID, and tells
you to **label the case with a Sharpie immediately** (the graveyard-box rule:
unlabeled servo = future mystery). Batch 1 has only 4× ST3215 + 1× SCS0009 —
assign them as leg0 (1,2,3) + a spare femur (5) + hand0 (16); rerun with
`--start-from` when Batch 2 lands.

- [ ] After the last one: `bus_scan.py` again — every expected ID answers.

## 4. Smoke motion (first commanded move — one servo on the desk)

```python
# python3 - <<'EOF'   (or interactively)
import sys; sys.path.insert(0, "../driver")
from rocky_driver import FeetechBus, SerialTransport
bus = FeetechBus(SerialTransport("/dev/ttyACM0", 1_000_000))
bus.torque(1, True)
bus.set_position(1, 15.0, speed_cps=400)   # slow, small
print(bus.telemetry(1))
bus.set_position(1, -15.0, speed_cps=400)
bus.torque(1, False)
# EOF
```
- [ ] Motion is smooth both ways; telemetry temp/volt sane. **Do not skip
  the torque-off at the end** — servos holding position on a desk edge walk
  themselves off it.

## 5. Center calibration (with the horn couplers / comb printed)

Mount servos in their printed parts first (calibration is meaningless on a
bare servo — zero lives in the assembly). Then:

```bash
python3 calibrate_centers.py --port /dev/ttyACM0
python3 pose_check.py --port /dev/ttyACM0     # acceptance: <2 deg everywhere
```
- Jig poses: yaw straight out, femur horizontal, knee at −90° (comb),
  claw fully closed. The script stores software offsets in
  `bench/calibration.yaml` (git-tracked). `--burn` also writes STS EEPROM.
- If the nudge test flips a `dir` sign the offset is re-derived from the same
  reading (D052) — no second run needed.
- Offsets live in ONE place (D052): the yaml, unless `--burn` (then the
  EEPROM holds it and the yaml stores 0). `apply_limits.py` refuses a servo
  with both non-zero.
- Then burn the hardware backstop: `python3 apply_limits.py --port /dev/ttyACM0`
  — MIN/MAX_ANGLE_LIMIT = the params joint limits + dir + offset + 2°, read
  back and verified. Re-run after every recalibration.

## 5b. Cockpit bring-up (sim ↔ one real leg, D051/D052)

The cockpit's Hardware panel is `sim/hw_bridge.py`. Rules it enforces, so you
know what a refusal means:

1. **Close every bench script first** — one process per port. Connect
   `mock` once to rehearse, then the real port.
2. **Scan** (mirror must be off). Partial legs and hands show in the table and
   are monitored (temp, volt, faults, "lost") but are not mirrored — the
   one-servo-at-a-time bench is covered.
3. **Jog** one servo: the clamp is in the gait frame through your calibration
   (hip −70…90, knee −150…−20, yaw ±40, claw 0…55); a jog drops any stream.
4. **Center / dir / apply limits** from the panel (mirror off). Flipping a dir
   re-derives that joint's offset from the stored jig pose — the event says so.
5. **real2sim** first: move the real leg by hand, the sim leg follows — this is
   the "does the frame match?" check. Torque state is left as it is.
6. **sim2real** only with the sim standing still (refused otherwise). The entry
   is soft: goal parked where the leg IS, torque limit 40 %, 200 cps, a
   smoothstep blend of ≥ 1.5 s (longer for big gaps), released after ≥ 3 s.
   Walking stays disabled while streaming (no real foot contacts yet).
7. **What cuts the mirror by itself**: a fault bit or ≥ 65 °C on any servo of a
   leg limps that whole leg (`hw:cut`); 10 silent ticks drop a leg
   (`hw:degraded`); 3 silent monitor polls trip the servo (`hw:lost`); no sim
   targets for 2 s turns the mirror off (`hw:stale`); 25 failed ticks = port
   lost, limp attempted, reopen every 2 s (`hw:lost` / `hw:reopen`). Non-finite
   targets keep the leg's last good pose (`hw:nan`); goal steps faster than
   4.7 rad/s are capped (`hw:rate`).
8. **Re-arm** a tripped servo or a degraded leg only by torque-on with its ids
   named (the panel's re-arm button). A mirror change or rescan never re-arms.
9. **Stop** turns the mirror off and the legs hold; **Limp** drops torque on
   every present servo; **disconnect / quit** limps and closes the port.

## 6. Health monitor always-on habit

Any session longer than a smoke test runs with the monitor in the loop
(`PebbleRobot.health_step()` at 1–5 Hz): warn at 60 °C, **torque-cut at
65 °C** (master plan §7). The scripts above already do this.

## 7. Thermal soak — the master-plan risk, measured

Femur servo in the bench jig, lever + mass per §8's table (stance-load ≈
0.9–1.0 N·m). Room temp noted.

```bash
python3 thermal_soak.py --port /dev/ttyACM0 --ids 2 --minutes 20 \
        --note "stance load 0.95Nm, 22C room"
```
- [ ] Walking-load soak (~0.7 N·m): expect comfortable steady state.
- [ ] Stance-load soak (~1.0 N·m): the number we care about — time-to-60 °C
  is the robot's standing-still budget before the sleep pose must trigger.
- [ ] (Optional, supervised) untucked-manip load (~1.4 N·m): D015 predicts
  this cooks — watching it climb validates the STS3250 upgrade logic.
  Abort early; no need to actually hit 65.

File all three time-to-temperature numbers in NOTES_INBOX.

## 8. Torque-step — calibrate trust in the torque model

Lever + bag-of-screws mass (nothing rigid — it needs to fall gracefully).

| test | mass @ 100 mm arm | predicted % stall |
|---|---|---|
| walking-hip proxy | ~700 g | ~23 % |
| stance-hip proxy | ~1000 g | ~33 % |
| no-go demo (optional) | ~1500 g | ~50 % |

```bash
python3 torque_step.py --port /dev/ttyACM0 --id 2 --mass-g 1000 --arm-mm 100
```
- [ ] measured/predicted within 0.8–1.3 → D015's margins are real; outside →
  investigate (lever geometry, PSU sag under load, wrong variant!).

## 9. End of day

- [ ] `register_dump.py` again (`--diff` vs the morning dump) — archive what
  changed (IDs, offsets if burned).
- [ ] Servos → labeled bin; `calibration.yaml`, dumps, soak/torque CSVs →
  committed to the repo.
- [ ] NOTES_INBOX gets: model numbers, any VERIFY-ON-BENCH resolutions
  (SCS0009 sweep! measure it: command 0→300° in steps, watch where it stops),
  caliper measurements if the calipers are out anyway, and every surprise.
- [ ] Next stop: single-leg jig (cad/part_bench_jig) + `pose_check`, then the
  gait engine on real metal — Phase 1 gate.

---

## Appendix: failure modes seen in the wild

| symptom | usual cause |
|---|---|
| no response at any baud | TX/RX swap, charge-only USB cable, dead PSU channel |
| responds, then silence after ID write | two servos shared the old ID — rescan, unplug one (the cockpit's set_id refuses an occupied id) |
| position jumps 180° on power cycle | multi-turn wrap — power-cycle at center, check ANGULAR_RESOLUTION |
| servo hot at idle | holding torque against gravity — torque off when parked, sleep pose |
| checksum errors under load | brownout: PSU current limit too low, or wire gauge too thin |
| SCS0009 dead after one glorious second | it met the 12 V rail. D016 exists for a reason. |
