# Servo notes: the ST3215 and fluid motion

*A research pass on 2026-09-28, before any servo was bought. Four independent
lines of research (what this repo asks of the servo, the servo itself, the
alternatives, and how people get fluid motion out of position servos) were
each checked claim by claim against primary sources by a second reader.
Numbers carry their source; "computed" means derived here, not measured.
Nothing in this file has been measured on Pebble's own servos yet: the bench
runbook does that ([bench/BENCH_RUNBOOK.md](../bench/BENCH_RUNBOOK.md)).*

## Verdict

**The ST3215 (12 V, 30 kg·cm) is the right servo to start with.** It is the
standard hobby bus servo, not a compromise: Hugging Face's LeRobot SO-100 /
SO-101 arms run six each, Open Duck Mini walks on fourteen of the 7.4 V
version, and [lukas/hexapod](https://github.com/lukas/hexapod-cad/blob/main/PROTOTYPE.md)
(18 × C018, 1.3 kg, 3S, 20 mm disc horns) is the closest thing to Pebble.
It is cheap, it daisy-chains on one bus, it reports position, speed,
current, voltage and temperature, and it is strong enough: walking loads
the knee to about 12 % of stall.

**What it will not do on its own is move organically.** It is a stiff
position servo with a 1:345 gearbox and no current or torque loop, so it
cannot be made genuinely soft or back-drivable. Nearly all the jerkiness
Pebble would show today comes from the software, and that is fixable
(B76). Its weak spots are heat under sustained load and ~1° of gear
backlash.

## What Pebble asks of it

| case (12 V) | yaw | hip | knee | source |
|---|---|---|---|---|
| stand on 5 legs (static) | 0 | 5.8 % | 9.4 % | `sim/out/torque_audit.json` |
| 3-leg stance (static) | 0 | 9.6 % | 15.6 % | same |
| walk 45.5 mm/s, physics RMS | 3.8 % | 6.8 % | 11.8 % | `sim/out/audit_gestures.json` |
| self-right push (W/2 through one leg) | 0 | 7.8 % | **50.3 %** | `torque_audit.json` |
| the hottest gesture (`manip_adjacent`), RMS | 4.7 % | 39.2 % | 19.1 % | `audit_gestures.json` |

(Percent of the 2.94 N·m stall. Yaw reads 0 in the static audit because it
applies vertical foot forces only.)

- **Speed binds before torque.** The walking envelope (45.5 mm/s) is set by
  the knee reaching the 3.0 rad/s loaded speed budget near the ground; the
  commanded yaw peaks at 3.63 rad/s, 77 % of the 4.7 rad/s no-load speed.
- **The thin margin is the self-right knee push**, and it is thinner than
  the audit says:
  - The C018 datasheet's *rated* torque is 10 kg·cm (0.98 N·m, **33 % of
    stall**, at 0.9 A). The repo's sustained budget
    (`actuators.st3215.continuous_frac: 0.65`) is a guess about twice that,
    and the 3.0 rad/s loaded speed budget is derived from it. At the
    datasheet's rated torque it would be about 1.6 rad/s (computed).
  - Every audit number is at 12 V. A 3S pack runs 12.6 V down to the 9.9 V
    floor; if stall scales with voltage, the knee push is ~54 % of stall at
    11.1 V and ~61 % at 9.9 V (computed). Only the RL environments
    randomise voltage.
  - The firmware's own overload rule is 80 % load for 2 s → output drops
    to 20 % (memory table V3.7, addr 34–36), not the sim's 85 % for 180 s.
  - The bench thermal soak and torque step (runbook §7–8) settle all of
    this (B77).

## The servo

**Hardware** ([C018 datasheet](https://cdn.robotshop.com/media/F/Fit/RB-Fit-155/pdf/feetech_12v_30kg_cm_magnetic_encoding_servo_sts321_specification_pdf.pdf)):
iron-core ("core") motor, copper gears 1:345, glass-filled nylon case,
ball bearings, 12-bit magnetic encoder (0.088°/count), stall 30 kg·cm at
2.7 A, rated 10 kg·cm at 0.9 A, no-load 0.222 s/60° (45 RPM), backlash
spec ≤ 0.5°, 55 g, life test > 100,000 cycles of ±60° at 1/5 stall.

**Measured by others:**
- **Backlash:** 0.62° unloaded / 1.30° loaded
  ([HardwareX 2026](https://pmc.ncbi.nlm.nih.gov/articles/PMC13087586/)),
  0.87° on a C018 ([Robonine](https://robonine.com/testing-of-feetech-sts3215-servomotor-backlash-repeatability-and-torque/)).
  Repeatability about ±0.17°.
- **Heat:** holding 15 kg·cm settles at 48 °C; oscillating under load it
  reached 71 °C in 50 min, and Robonine saw no working cut-off there. One
  unit burnt its controller board in repeated 100 %-load tests.
- **Tracking under load:** about 2–2.6° error at 10–15 kg·cm (Robonine).

**Control** ([memory table V3.7](https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls);
a 7.4 V, firmware-3.7 document, while 12 V units ship with 3.10, so dump a
real one before trusting any default):
- **Modes** (addr 33): 0 position, 1 closed-loop speed, 2 open-loop PWM,
  3 step. There is **no current or torque mode**.
- **Motion registers:** P/D/I at 21/22/23 (defaults 32/32/0), acceleration
  at 41 (units of 100 steps/s²), goal speed at 46, torque limit at 48
  (a PWM cap, not a torque), dead zone 1 count each way, minimum start
  force 1.6 % (addr 24).
- **Hidden acceleration ramp:** addr 85/86 apply a ramp even with ACC = 0.
  LeRobot writes 85 = 254; Open Duck Mini and a 12 V identification dataset
  write 85 = 0. The repo's `bench/register_dump.py` stops at addr 73 and
  never sees them (B79).
- **Feedback:** position, speed, voltage, temperature, moving flag, and
  current in 6.5 mA counts. "Present load" is PWM duty, not torque.
- **Gain in physical units:** Rhoban's
  [BAM model](https://github.com/Rhoban/bam/blob/main/bam/feetech/actuator.py)
  (7.4 V units) is duty = error (rad) × P × ~0.17, capped at 0.97, so the
  drive saturates at ~10.5° of error at P = 32 and stiffness is linear in P.
  Halving P halves stiffness and doubles the droop under load.
- **Bus:** SYNC_WRITE and SYNC_READ. At 1 Mbps a 15-servo position + time +
  speed write is ~1.1 ms and a 15-byte telemetry read ~3.4 ms plus return
  delays (computed from the packet formats), so 100 Hz is feasible. The
  factory return delay is disputed (0 in the table, 500 µs per LeRobot).

**Field traps:**
- **Encoder seam:** position wraps 4095 → 0. LeRobot centres every joint on
  2048 before fitting the horn.
- **EEPROM writes can silently revert** on some batches unless re-locked,
  which has left two servos on one ID ([so101-lerobot](https://github.com/aakashvardhan/so101-lerobot)).
  Assign IDs one at a time and read them back.
- **Brownout:** one Open Duck Mini builder cured bus CRC errors with a
  higher-current cell (unconfirmed for the reported case). Loose plugs
  cause dropped status packets too.

**Order by suffix.** "STS3215" covers several servos. Pebble's is the 12 V
1:345, part code **C018 / C047**; C001 is the 7.4 V, and C044 (1:191) and
C046 (1:147) are different gearings. Buy a listing that ships the
aluminium disc horn the couplers are cut for (`bom/BOM.csv` A-01).

## Fluid motion: what limits it today

From the repo (B76):
- **The swing leg lands at 188 mm/s straight down.** The swing height is
  24·sin(πs), so the joint targets jump by ~2 rad/s (hip) and ~2.7–3.1
  rad/s (knee) at every lift-off and touchdown (`gait/pebble_gait.py`).
- **Command changes snap.** Going from standing to 45 mm/s moves a joint
  target up to 28° in one 20 ms tick; nothing rate-limits the command.
- **The servo rushes each 20 ms step:** after entry the bridge runs
  ACC 0 and goal speed 0 (= servo maximum), with no interpolation between
  ticks (~4° steps at 3.6 rad/s).
- **Keyframes stop at every key** (the default `smooth` ease); `linear`
  passes through with a velocity jump; nothing splines across keys.
- **No idle motion**, and the servo gain registers are never written.
- The RL righter is a known 50 Hz staircase (D048).

**Ranked fixes** (effort S/M/L):

| # | change | effect | effort | risk |
|---|---|---|---|---|
| 1 | Rate-limit the walking command | removes the start/turn/stop snaps | S | slower response; keep stop and void paths direct |
| 2 | Soft-landing swing (height ∝ s³(1−s)³, horizontal speed matched to stance at both ends) | no velocity steps at lift-off/touchdown | M | foot stays low longer; could stub obstacles |
| 3 | Goal speed per tick = k·distance/dt (never 0) | steady motion instead of rush-and-stop | S | lag if k is low: tune on the bench |
| 4 | Splined keyframes, per-joint time offsets, anticipation | the biggest "organic" gain for gestures | M | overshoot past limits (the checker samples at 100 Hz) |
| 5 | Idle "life" layer: breathing ±3–6 mm at 0.2–0.3 Hz, slow sway, glance before a turn | looks alive | S–M | stick-slip below the 1-count dead zone; must not trip the standing-still gates |
| 6 | Bus at 100 Hz | halves step size and hold delay | S | RL checkpoints were trained at 50 Hz |
| 7 | Per-joint P/D at runtime (Lock = 1 so EEPROM is not worn): stance 32, gestures 8–16 | softer, quieter gestures | S–M | more sag; the driver refuses EEPROM registers in sync writes today |
| 8 | Minimum-jerk blends instead of smoothstep | no acceleration kick at each start | S | ramps ~25 % longer at the same peak speed |
| 9 | Servo ACC for gestures and idle only | free smoothing on slow moves | S | rounds off the gait (it needs 20–34 rad/s², ACC ≥ ~150) |
| 10 | Fit the sim's servo model to 12 V data (friction, armature, backlash) | motion authored in the sim looks the same on the robot | M–L | retraining; the public fits are 7.4 V |

Exemplars: Open Duck Mini runs a 50 Hz policy at P 32 / D 0 / ACC 0 and
powers on at P 2 ([runtime](https://github.com/apirrone/Open_Duck_Mini_Runtime));
LeRobot runs P 16 "to avoid shakiness"; Disney's BD-X interpolates a 50 Hz
policy to 600 Hz through a 37.5 Hz low-pass and layers an always-on idle
loop under triggered clips ([arXiv 2501.05204](https://arxiv.org/abs/2501.05204)).

## If you want more than software can give

| step | $/joint | what it buys | cost |
|---|---:|---|---|
| **ST3215 12 V** (baseline) | ~21 | fine for everything the sim asks | stiff position control only |
| **Feetech HL-3930 / HL-3950** (same body, spline, bus, 9–12.6 V) | ~73 / ~85–110 | a **constant-current mode**: real torque ceilings and soft holds (not back-drivability: still 1:345) | evidence is one datasheet line (no HLS memory table found, units of the torque value undocumented); +15.5–19.5 g each; HL-3930's *rated* torque (8.7 kg·cm) is below the ST3215's; **the driver writes 0 to addr 44 in every goal block, which on an HL servo is goal torque**, so it needs its own register family first (B78) |
| STS3250 (same body) | ~65–73 | 50 kg·cm, faster, coreless, lowest backlash measured | runs hottest; no current mode; +19.5 g; FEM: with knee-limited loads the coxa base falls from SF 2.93 to ~1.76 (computed) |
| Dynamixel XM430-W350 | ~310 | mature current-based position control | new CAD, new bus, 15× the price |
| Brushless quasi-direct drive (CubeMars AK40-10, SteadyWin GIM4305) | ~105–135 | back-drivable, force-controllable joints: the most organic motion there is | 150–190 g each, 24 V class drivers, CAN, a new robot: the full-scale Rocky (D003), not Pebble |

**Recommendation:** build the bench leg on ST3215s and do fixes 1–5 first;
they are software and cost nothing. If soft, compliant joints then matter,
buy **one** HL-3950 and bench it (B78) before changing any leg.
