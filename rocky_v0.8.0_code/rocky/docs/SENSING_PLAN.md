# SENSING_PLAN.md — Pebble's layered world model (session 4)

*How the robot knows anything, from reflex-fast to semantic-slow. Each layer
runs where its latency budget lives (plan §5.4), degrades gracefully when
layers above it die, and exists in some testable form TODAY (sim or code).*

## The four layers

| layer | rate | lives on | sensors | state produced | status today |
|---|---|---|---|---|---|
| **L0 proprioception** | 100 Hz | Pi (later ESP32) | servo pos/vel/load/temp, SEA microswitches, BNO085 | joint state, contact flags, body attitude, `legged_odom` pose (1–7 % drift, `sim/run_odom.py`) | **EKF built + measured** |
| **L1 local bubble** | 10–20 Hz | Pi | 5× ultrasonic skirt, cliff detector (software, free), whiskers (I6) | "something within 40 cm at bearing θ", void events | **cliff PASS in sim** (`run_cliff.py`); sonar in Batch 3 |
| **L2 geometric map** | 1–8 Hz | Pi (SLAM) + laptop | 360° lidar puck (vacuum pull, Batch 3 core), L0 odom prior, WiFi RSSI prior | occupancy map, global pose | **sim-lidar + scan contract ready** (`sim/sim_lidar.py`, `laserscan_spec.json`); RSSI logger + k-NN demo: room-level (~1.8 m) prior |
| **L3 semantics** | 0.2–1 Hz | laptop VLM | Pi camera stream (+ thermal/PIR pod flavor) | "that's the charger / a cat / Tyler", task goals | Phase 4 |

**Reflex arbitration** (who may override whom, fastest wins):
E-stop > thermal SafetyMonitor > push-reflex BRACE > cliff VOID-halt >
stuck-watchdog retry > gait command. The same VOID signal that means
"cliff!" on commanded-flat ground is *terrain data* on rubble — routing:
if the stuck-watchdog is in a retry stage, VOIDs feed it; otherwise they
halt. (Both behaviors sim-verified this session.)

## Sensor pod standard (rides I6, the dovetail ring)

Every add-on sensor ships as a **pod**: printed shoe (I6 female + M3 knob)
+ sensor + JST-SH pigtail to the avionics bulkhead. Pods are position-free
by design — any of the 11 ring stations — so coverage is a config choice.

| pod | sensor | bus | draw | notes |
|---|---|---|---|---|
| sonar ×5 | HC-SR04-class (or US-100 3.3 V) | GPIO trig/echo (or UART) | 15 mA | one per shell sector = 360° skirt at 72° pitch |
| enviro | BME688 | I²C 0x76/0x77 | 3 mA | temp/humidity/pressure/VOC — "the basement smells weird" |
| warm-body | PIR (AM312 3.3 V) | GPIO | 0.1 mA | cheap presence; gates the camera/VLM wake |
| thermal (opt) | AMG8833 8×8 | I²C 0x69 | 5 mA | pet-vs-obstacle disambiguation, warm-foot-print party trick |
| whiskers ×2 | microswitch + piano wire | GPIO | 0 | `whisker_shoe` printed already |
| mic array | ReSpeaker 2-mic HAT | I²S (Pi header) | 30 mA | NOT a pod — sits on the Pi via the tray standard; chord-speak's ears |

## I²C budget + mux (the tray bulkhead's Qwiic port)

Native bus: BNO085 (0x4A) + PCA9548A/TCA9548A mux (0x70) only — the IMU
never shares wire time with slow peripherals. Behind the mux: ch0 BME688
(0x76), ch1 AMG8833 (0x69), ch2 OLED debug (0x3C, someday), ch3–7 spare
pod channels brought to the bulkhead Qwiic connector. Rule: anything
sampled slower than 10 Hz lives behind the mux; the 100 Hz IMU does not.
Bus at 400 kHz; total mux-side traffic worst case ~6 kB/s — nothing.

## Data contracts (already frozen in code)

- odometry: `perception/legged_odom.py` state → `/pebble/odom` (frame `odom`)
- scan: `sim/laserscan_spec.json` → `/pebble/scan` (frame `lidar_link`)
- contacts: `rocky_msgs/ContactState`
- health: `rocky_msgs/ServoHealth`
- RSSI: `perception/wifi_rssi_logger.py` JSONL v1 records

## What unblocks what

```
servos land ──► L0 real (EKF on hardware, drift re-measured)
Batch 3 sonar ─► L1 skirt (the cliff detector already runs)
vacuum lidar ──► L2 slam_toolbox (scan contract + sim data ready NOW)
Pi camera ────► L3 VLM agent (Phase 4; RSSI room-prior helps it point)
```
