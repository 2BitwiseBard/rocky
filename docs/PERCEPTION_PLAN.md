# Perception plan

How Pebble knows anything, from reflex-fast to semantic-slow. Each layer
runs where its latency budget lives, keeps working when the layers above it
die, and exists in a testable form today (in the sim or in code). The
design rule is Rocky's canon: perception is **360° geometric first**. The
lidar is the eyes; the camera is a supplementary sense hidden behind the
shell (a gill), never styled as eyes; and chord-speak is the output
channel, so the robot can *tell* you what it senses.

Everything marked "sim" below runs on clean MuJoCo sensors, so its numbers
are upper bounds for the real robot, which has no sensor wired yet (B35).

## The layers

| layer | rate | lives on | sensors (hardware plan) | state produced | in the sim today | on the robot |
|---|---|---|---|---|---|---|
| **L0 proprioception** | 100 Hz IMU, 50 Hz bus | Pi | servo position / load / temperature over the bus, the SEA foot microswitches (D010), BNO085 IMU on SPI | joint state, contact flags, body attitude, the `perception/legged_odom.py` pose | legged-odometry EKF (D026): drift 4.49 / 5.02 / 8.06 % of distance on flat / turny / 20 mm rubble; the StillnessGate (zero yaw rate while standing) ends a patrol at 1.63° of yaw instead of 14.37°, drift 3.51 % | nothing yet: IMU driver and foot switches are B35 |
| **L1 local bubble** | 10–20 Hz | Pi | the cliff detector (software: a planted foot whose switch stays open), the contact-seeking foot probe, whiskers on an I6 shoe; later a sonar skirt and downward ToF | void events, "something within 40 cm at bearing θ", stuck | cliff detector (D034): `run_cliff` ends 252.3 mm short of the edge at x = 350 mm (the torso's closest approach was 170.7 mm short, then the retreat), no fall; the foot probe (D050) and the always-on void guard + touchdown gate (D052), except a lip band of 7 in 310 approach angles (D052a); goto `stuck` after 3 s without 2 cm of progress | nothing; sonar (X-09) and ToF (X-02) are optional and unbought |
| **L2 geometric map** | 1–10 Hz | Pi (SLAM) + server | a 360° 2D lidar (LDRobot D500, phase C), the L0 odometry prior, a Wi-Fi RSSI prior | occupancy map, global pose, a place signature | `sim/sim_lidar.py` + the frozen LaserScan contract (D024: 360 rays at 8 Hz over 0.12–6 m); ICP SLAM-lite (D027; the lap re-recorded inside the envelope 2026-09-30, B111): ATE 59.2 → 9.2 mm, map IoU 0.187 → 0.692, end yaw 0.13°; goto `blocked` detours; lidar place signatures (D057, off by default); the RSSI logger's k-NN demo (a ~1.8 m room-level prior) | lidar in phase C: mount B12, params block B46 |
| **L3 semantics + memory** | 0.2–1 Hz | server (vision model / LLM) | a fixed wide camera behind a gill; a mic | labelled objects with bearing and distance, the scene memory, places, narration | `look` (D049); `find_object` turns a vision model's box into bearing and distance through the eye's pose (D054: 0.7°, 3 cm); scene memory + a situation line per turn (D054); place recognition from lidar + look descriptions (D057: 21/21 verdicts, changes 3/6, [PLACES.md](PLACES.md)) | no camera yet (B16); the sim eye is not yet the real lens (B45) |

## Reflex arbitration

Who may override whom, fastest wins:

E-stop (the XT60 loop key) > thermal SafetyMonitor > push-reflex BRACE >
cliff VOID-halt > stuck-watchdog retry > gait command.

The same VOID signal that means "cliff!" on commanded-flat ground is
*terrain data* on rubble, where it fires on bridged or late feet. Routing:
while the stuck-watchdog is in a retry stage, VOIDs feed it; otherwise they
halt (`perception/cliff.py`). Since D050 a void also needs the foot to have
run out of probe (`probed_out`): a contactless planted foot first feels for
the floor, which turns most rubble false alarms into a measurement.

The stuck-watchdog (`gait/pebble_watchdog.py`, D023) fires on missing
*progress*, not tilt: on rubble the blind gait snags but does not fall
(D017). Its `RetryPolicy` escalates nominal → high-step → higher + clear →
retreat, and each stage is derived from the gait budget (D060): the
shortest cycle time whose speed envelope is at least 20 mm/s, every command
through `budget()`. Today (D063's soft-landing swing) that is 24 mm /
T 2.0 s / 34.2 mm/s, then 34.8 mm / T 2.5 s / 21.4 mm/s, then 42 mm with
the body 10 mm higher / T 3.5 s / 28.1 mm/s. On that ladder and the D063
model (`sim/experiments/run_stuck.py`, re-run 2026-09-30 for B111: 34.2 mm/s
for 32.9 s, the distance 25 s at 45 mm/s commanded) it lifted crossings of
4 rubble fields from 2/4 to 4/4 at 30 mm, 2/4 to 4/4 at 35 mm and 1/4 to
2/4 at 40 mm, and left 45 mm at 1/4, with no falls. These replace the D060
ladder's 2026-09-26 numbers (45 mm/s for 25 s: 2 → 4, 0 → 4, 0 → 2,
1 → 2 of 4). B54 redesigns it against the terrain. It is on no live path
yet: the cockpit's goto uses its own no-progress rule.

## The brain split: robot = reflexes, server = cortex

The Pi runs everything real-time: gait, reflexes, the EKF, SLAM-lite,
chord-speak. It streams to a home server (or a cloud API) that does the
heavy semantics:
- **Uplink:** JPEG keyframes on demand (1–2 Hz, not continuous), the
  current scan, pose, the event stream. The bandwidth is trivial.
- **Server:** a vision model and a brain answer structured queries ("label
  the salient objects with bearing and extent"); results are pinned to the
  map as annotations. In the sim this is the cockpit: `look` and
  `find_object` send the eye frame to a vision model, and any brain drives
  the one tool registry ([TOOLS.md](TOOLS.md), [BRAINS.md](BRAINS.md)).
- **Downlink:** sparse annotations and high-level goals ("the kitchen is
  bearing 40°, 3 m"), latency-tolerant by design.
- **Nothing safety-critical depends on the link.** Reflexes, the void guard
  and the safe stop are all onboard, and every tool result passes the
  arbitration above: a hallucinating model can command nothing the reflex
  layer will not veto. Link down, the robot keeps full L0/L1 competence
  (walk, avoid, patrol, narrate geometry).

The bottom of the brain stack needs no model at all: `harness/intent.py`
maps words to tools deterministically (`rocky.sh talk`), and
`harness/local_brain.py` rehearses the loop offline. Speech in, chords out
shipped in the cockpit (D054: a fast whisper server, a fuzzy "pebble" wake
word; `docs/COCKPIT_GUIDE.md`); the robot never speaks English.

**Big-Rocky onboard path.** The full-scale robot carries the cortex: one
NPU-class board (Orin Nano, a Pi AI HAT+, Hailo-8) for an onboard small
vision model and detector behind the same interface, so the server split is
the API contract and the software moves unchanged. Decide when the scale-up
starts ([SCALE_UP_NOTES.md](SCALE_UP_NOTES.md)); nothing before it depends
on the choice.

## Sensor hardware (Pebble)

Rows are in [bom/BOM.csv](../bom/BOM.csv); pins in
[WIRING_HARNESS.md](WIRING_HARNESS.md#pi-5-pin-map-draft-verify) (B43).

- **IMU: BNO085 on SPI** (C-01). Not I²C: a Pi's I²C controller mishandles
  its clock stretching. Not UART-RVC: that mode reports no angular rates,
  and the brace trip (`reflex.gyro_trip`, 1.8 rad/s) and the odometry EKF
  need them.
- **Lidar: LDRobot D500** (C-02, phase C): 54.0 × 46.3 × 35.0 mm (datasheet; VERIFY), 45 g,
  0.03–12 m, 10 Hz (6–13), ≤ 0.72° resolution, UART 230400 on a
  ZH1.5T-4P plug through its own USB adapter, 5 V 0.29 A. It sits on a
  hatch-cap variant (B12, the D029 cap seat), not a mast. The sim still
  models 360 rays at 8 Hz over 0.12–6 m until B46 gives it a
  `sensing.lidar` block.
- **Camera: Raspberry Pi Camera Module 3 Wide** (C-04): 120° diagonal
  (≈ 102° × 67°) at 16:9, fixed on an internal bracket, the lens looking
  through a gill slot in a shell sector (B16), no pan turret. One forward
  camera plus turning in place beats a camera ring at this scale; the lidar
  already covers 360°. The sim eye is fovy 70° at 4:3 (~86° wide) until
  B45 makes it the real lens and re-runs the vision bench.
- **Audio (phase D, optional):** a USB mic (D-02) and a MAX98357A I²S amp
  with a 40 mm speaker (D-01), the chord-speak mouth. A USB mic gives no
  sound bearing. The alternative, the ReSpeaker 2-Mics Pi HAT v2.0 (D-03:
  two mics and a codec with its own speaker out), conflicts with the rest:
  the Pi has one I²S, so the HAT and the MAX98357A cannot both have it; the
  HAT's APA102 LEDs sit on SPI0 MOSI/SCLK, which the SPI0 IMU would clock;
  and it takes GPIO17 and the whole header.
- **Foot switches:** each leg's SEA microswitch has its own Pi GPIO
  (star-board J6, [WIRING_HARNESS.md](WIRING_HARNESS.md)).

## Sensor pod standard (rides I6, the dovetail ring)

Every add-on sensor ships as a **pod**: a printed shoe (I6 female + M3 knob)
+ the sensor + a JST-SH pigtail to the avionics bulkhead. Pods are
position-free by design (any of the ring's 10 as-built stations, I6), so
coverage is a config choice. None has CAD, a driver or a sim model yet except the whisker
shoe.

| pod | sensor | bus | draw | notes |
|---|---|---|---|---|
| sonar ×5 | US-100 (3.3 V) | GPIO trig/echo (10 GPIO for five) or UART | 15 mA | one per shell sector = a 360° skirt at 72° pitch (X-09) |
| enviro | BME688 | I²C 0x76/0x77 | 3 mA | temperature / humidity / pressure / VOC: "the basement smells weird" |
| warm-body | PIR (AM312, 3.3 V) | GPIO | 0.1 mA | cheap presence; gates the camera / vision wake |
| thermal (opt) | AMG8833 8×8 | I²C 0x69 | 5 mA | pet-vs-obstacle disambiguation; not in the BOM |
| whiskers ×2 | microswitch + piano wire | GPIO | 0 | `whisker_shoe` exists in CAD (`cad/part_smallwins.py`) |
| downward ToF | VL53L4CD (or VL53L8CX 8×8) | I²C 0x29 (fixed: XSHUT or the mux) | — | sees an edge before a foot does (B33c, B40; X-02) |

## I²C budget and mux (the tray bulkhead's Qwiic port)

Native bus: the TCA9548A mux (0x70) only, plus the INA228 (0x40) if
fitted (B47). The IMU is on SPI, so it never shares wire time with slow
peripherals. Behind the mux: ch0 BME688 (0x76), ch1 AMG8833 (0x69), ch2 an
OLED (0x3C, someday), ch3–7 spare pod channels (the ToF sensors among them)
brought to the bulkhead Qwiic connector. Rule: anything sampled slower than
10 Hz lives behind the mux. Bus at 400 kHz; worst-case mux-side traffic is
about 6 kB/s.

## Data contracts (frozen in code)

- odometry: `perception/legged_odom.py` state → `/pebble/odom` (frame `odom`)
- scan: `sim/laserscan_spec.json` → `/pebble/scan` (frame `lidar_link`)
- contacts: `rocky_msgs/ContactState`
- health: `rocky_msgs/ServoHealth`
- RSSI: `perception/wifi_rssi_logger.py` JSONL v1 records

## What unblocks what

```
servos + IMU + foot switches on the Pi (B35) ─► L0 real: the EKF on hardware, drift re-measured
D500 lidar (C-02) + cap mount (B12/B46) ──────► L2 slam_toolbox (contract + sim data ready), place signatures on real scans
camera (C-04) + gill bracket (B16) + B45 ─────► L3 look / find_object / places on the robot
sonar or ToF pods (X-09 / X-02) ──────────────► L1 look-ahead (the foot-probe guard works without them)
```

## Behaviours this unlocks

Shipped in the sim are marked (sim); the rest are planned.
- **Explore and report:** frontier-driven patrol, then a chord-speak
  summary ("two rooms, one blocked doorway, found the dock"). A waypoint
  patrol through the harness tools runs in the room world (sim:
  `run_patrol`, 5/5 waypoints).
- **Fetch / point:** "where is X" → walk to it, point with an arm, say
  `found_it` (sim: `find_object`; pointing is the gesture library).
- **Guard / anomaly:** a map diff against the last visit: new obstacle =
  `curious`, missing object = `confused`, moving contact = `alarm` (sim:
  the D057 "changed" verdict; an added object is not yet confirmed, B38).
- **Follow-me:** a person = a moving L2 blob + an RSSI gradient; keep 1.2 m.
- **Dock ritual:** low battery → navigate to the dock (B14), funnel,
  crouch, mate, `sleepy`.
- **Near (lidar + odometry):** find-the-sunny-spot (BME688 gradient);
  obstacle-course runs (sim: the goto crosses the course, D050); "where's
  my phone" by BLE RSSI; dead-reckoning tricks (walk a square blindfolded,
  chirp `amaze` when the closure error is under 2 cm).
- **Mid (camera + server brain):** deliver small items in the scoop (an I2
  tool); greet known people with their own chord motif (voices: B39); a
  photograph-the-house patrol → a daily digest; plant-minder rounds (a
  soil probe is another I2 tool); hide-and-seek.
- **Canon-social:** the question game (yes/no in chords); jazz-hands echo;
  fist bump on offer (the completion, an extended arm + SEA-switch contact,
  runs in the sim; the camera only supplies the invitation, via a pose
  model or a one-word vision prompt); sing-along (the gesture engine takes a
  tempo); a bedtime round (doors checked on the map, `sleepy`, dock).
