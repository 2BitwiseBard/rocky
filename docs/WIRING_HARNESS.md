# Wiring harness (D016 + I1 / I4 / I5)

How power and data move through Pebble: the power tree, the five leg drops,
the star board, the Pi's pins and the power budget. The electrical calls
are D058's ([decisions](decisions.md)), which settled the 2026-09-22
review's findings ([REVIEW_2026-09-22.md](REVIEW_2026-09-22.md) §4) and
kept the Pi + adapter as the bus master (the ESP32 board is a bench tool).
The rule that saves a servo: **hand servos never see 12 V** (D016).
Parts are rows in [`bom/BOM.csv`](../bom/BOM.csv); nothing is wired yet.

## Power tree

```
 3S LiPo 5200, soft case (I5) ── balance-lead voltage alarm (B-07)
   XT60 ── sled nose ── blind-mate dock (I5, spare XT30 taps ×2)
                                │ 14 AWG silicone, up through the power-entry grommet
                     15 A blade fuse ── XT60 loop key (arming plug + E-stop)
                                │
                        12 V power node
            ┌───────────────────┼──────────────────────────┐
            │                   │                          │
     ┌──────┴──────┐   ┌────────┴─────────┐      ┌─────────┴─────────┐
     │ 5 leg drops │   │ avionics tray I4 │      │ 6 V UBEC (D016)   │
     │ XT30 each,  │   │ 5 V 5 A buck     │      │ UBEC-3A, jumper   │
     │ 20 AWG      │   │ + 1000 µF → Pi 5 │      │ at 6 V: hand rail │
     │ (below)     │   │ GPIO 5 V pins    │      │ → star board J0.3 │
     └─────────────┘   └──────────────────┘      └───────────────────┘
```

- **Fuse, then loop key**, right after the pack (B-08, B-09). The loop key
  is the arming plug and the E-stop, and it must be reachable with the
  carapace on. A DC-rated ≥ 20 A rocker is the alternative.
- **The Pi 5** is fed through its GPIO 5 V pins from a Pololu D24V50F5 buck
  with 1000 µF (B-10). This skips USB-C PD, so tell the firmware it has 5 A
  ([pin map](#pi-5-pin-map-draft-verify)).
- **The 6 V UBEC** (B-11) is a Hobbywing UBEC-3A with its jumper at 6 V.
  Measure 6.0 V at the plug before any claw is connected.
- **Open (B66):** the dock's two XT30 spare taps (I5) break out of the dock
  block, ahead of the fuse. Fuse anything that is plugged into them.

## Per-leg drop (×5, ends at the I1 leg-port connectors)

| conductor | gauge | connector | carries |
|---|---|---|---|
| 12 V + GND | **20 AWG silicone** | **XT30** | 3 × ST3215 (~2.7 A stall per servo, ~2–3 A per leg walking; 20 AWG silicone is good for ~10 A on short runs). Fans out at the coxa into three servo leads |
| TTL data | 26 AWG | JST-XH-5 pin 1 | the shared bus (see topology) |
| GND (signal) | 26 AWG | JST-XH-5 pin 2 | bus reference |
| 6 V (hand) | 24 AWG | JST-XH-5 pin 3 | 1 × SCS0009 (≤ 1 A stall), from the UBEC rail, **never 12 V** |
| foot switch | 26 AWG | JST-XH-5 pin 4 | the SEA microswitch (D010); it closes to GND. Each leg's line goes to its own Pi GPIO through star-board J6 |
| spare | 26 AWG | JST-XH-5 pin 5 | reserved (future foot sensor / leg LED) |

## In-leg routing

1. **Deck to yaw servo.** The XT30 and XH-5 mate at the deck cutout (I1,
   leg-local x = −29). The leg side runs up the 11 × 11 channel in
   `coxa_yaw_base`: forward under the gearbox plateau, sideways to −Y, up
   beside the cup, over to the yaw servo's plugs. The coordinates are in
   [INTERFACES.md](INTERFACES.md) I1, and `check_assembly` asserts the path
   is clear at every yaw.
2. **Power fans out at the coxa.** The XT30 splits into three short 12 V
   leads, one per servo (B-14). Never carry a leg's 12 V through the servos'
   2 A connector pins. Data and GND daisy-chain yaw → hip → knee on stock
   Feetech 3-pin leads (A-06). The exact lead construction is VERIFY on
   bench day.
3. **Down the tube to the SEA and hand.** A JST-SH-3 runs through the I2
   tool socket (6 V, GND, data), and the microswitch pair runs back up to
   XH-5 pin 4.

**Check at the first bench torque test:** the cup's −Y side wall now
bridges the channel over 10 of its 24.5 mm footing. Watch it for flex.

## Bus topology (one logical TTL bus, D003)

The Pi drives the bus through the Waveshare Bus Servo Adapter (A) on the
tray (A-02), the only path `rocky_driver` implements. The ESP32 Servo
Driver is a bench first-motion tool, not the bus master (A-03).

It is a star of daisies. The adapter's data and GND, plus the UBEC's 6 V,
leave the tray as one XH-5 trunk through the bulkhead to the **star board**,
which fans out to five XH-5 leg drops. Each leg is its own daisy chain
(yaw → hip → knee → hand), so a wiring fault in one leg unplugs at its port
without taking down the other four. Star-to-port runs are 108–287 mm,
inside the 400 mm stub budget and fine at 1 Mbps; the leg's own chain adds
the leg's length.

## Star board (bus hub)

### Deck layout (body frame, mm)

- **Avionics tray (I4):** plate centre (0, −38), still an assumption
  (`part_avionics.TRAY_XY`); the bulkhead faces south at y ≈ −72.
- **Star board:** the bracket (v0.2) plate is centred at (0, 32), spans
  y 14 to 50, and its zip wings reach y 58. It bolts through its centreline
  to the existing deck grid holes **(0, 20) and (0, 40)**, so the deck needs
  no new holes. `part_busboard.layout_audit()` ray-tests both bolt axes
  against `body_deck()` and keeps the footprint off the tray (plate, rails,
  tongue), the strap slots, the power grommet and zip anchors, every leg
  port and the pentagon edge.
- **Power entry (fuse + loop key):** a Ø9 deck grommet at (52, −6), fed from
  the belly dock (the I5 nose points +X), with zip anchors at (44, −14) and
  (60, −14).
- **6 V UBEC:** on the tray floor next to the buck; its 6 V rides the
  bulkhead trunk to J0.
- **Loom ring lane, r ≈ 62:** outboard of the electronics grid and clear of
  the strap slots (r ≤ 47); inboard of the leg-port cable cutouts (r 81) and
  of the shell feet (r 73–82, at the webs only). The battery is below the
  deck, so nothing else is in the way.
- **Open:** the bracket sits over the north battery-strap run (y 7 to 37).
  It clears the slots, but if the fallback straps are ever used it needs
  feet or a new spot. At its assumed position the tray already covers the
  south strap run.

### The board

Cut a 2.54 mm proto board to 40 × 30 (16 × 12 holes). Drill the corners
Ø3.2 at 34 × 24 mm for the bracket posts. Parts: B-18.

| ref | part | job |
|---|---|---|
| J0 | JST-XH-5, right-angle | trunk from the bulkhead: 1 data, 2 GND, 3 6 V, 4 spare, 5 spare |
| J1–J5 | JST-XH-5, vertical | one per leg drop; mark the leg number |
| J6 | JST-XH-6, vertical | the five foot-switch lines to the Pi: sw0…sw4 + GND |
| W1 | data lane (26 AWG) | J0.1 → J1…J5 pin 1 |
| W2 | GND lane | J0.2 → J1…J5 pin 2 and J6 GND |
| W3 | 6 V lane (24 AWG) | J0.3 → J1…J5 pin 3 |
| W4 | foot-switch lines | J1…J5 pin 4, each to its own J6 pin (sw0…sw4); J0.4 unused |
| — | pin 5 | left open (reserved); continuity-test to nothing |

No resistors are needed. The Feetech bus is push-pull from the adapter, and
each switch line uses the Pi GPIO's internal pull-up.

### Printed bracket (`cad/part_busboard.py`)

A 46 × 36 × 3 plate with four Ø5.6 posts, 8 mm tall, in a 34 × 24 pattern
(M3 thread-forming bores, Ø2.8). Two M3 clearance holes on the centreline
sit over deck grid holes (0, 20) and (0, 40), and two zip-tie wings on the
+y edge hold the loom ring. Print it flat with no supports; PLA is fine
(no load). Screws go in first, then the board on its posts.

### Cut lengths (computed from the deck geometry)

Route model: radial from the source to the r = 62 ring lane, along the lane
the short way, then radial out to the leg-port cable cutout (r = 81). Add
25 mm for the station termination and 20 mm for the source termination,
then multiply by 1.15 for the service loop. The lengths end at the deck
cutout; the in-leg run through the coxa channel is extra, so measure it on
the first leg. Verify every length on the printed deck before cutting, then
write the real numbers next to these.

| leg | station az | XH-5 loom from the star board (0, 32) | XT30 12 V pair from the power node (52, −6) |
|---|---|---|---|
| 0 | 90° | **108 mm** | **205 mm** |
| 1 | 162° | **198 mm** | **294 mm** |
| 2 | 234° | **287 mm** | **233 mm** |
| 3 | 306° | **287 mm** | **144 mm** |
| 4 | 18° | **198 mm** | **115 mm** |

The trunk (bulkhead XH-5 → J0) is not computed (B67). The bulkhead faces
south at y ≈ −72 and the board sits at y 32, so the trunk runs round the
tray: measure it. The longest leg loom is 287 mm (legs 2 and 3), under the
400 mm budget.

### Star-board build order

1. Print the bracket and screw it to grid holes (0, 20) and (0, 40).
   Confirm it clears the tray tongue and the loom ring.
2. Populate J0–J6, run the four lanes, and continuity-buzz every pin pair.
3. Cut the looms per the table (after checking them on the deck). Crimp the
   XH at one end only.
4. Dry-route all five looms and the power pairs around the r = 62 ring,
   THEN crimp the station ends at their true lengths.

## Pi 5 pin map (draft, VERIFY)

A plan, not wiring: nothing is connected yet. The rows are B-03 (Pi),
C-01 (IMU), D-01 (amplifier), X-01 (INA228) and X-02 (ToF).

| function | pins | notes |
|---|---|---|
| BNO085 IMU on SPI0 | GPIO 8 (CE0), 9 (MISO), 10 (MOSI), 11 (SCLK), plus INT and RST on two free GPIOs | Set the breakout's P0/P1 for SPI (VERIFY in the Adafruit guide). Not I²C: a Pi mishandles its clock stretching. Not UART-RVC: it sends no gyro rates |
| I²S out → MAX98357A | GPIO 18 (BCLK), 19 (LRCLK), 21 (DOUT) | optional (D-01) |
| I²C1 | GPIO 2 (SDA), 3 (SCL) | INA228 at 0x40, TCA9548A mux at 0x70, and the downward ToF sensors behind the mux (VL53L4CDs all share 0x29) |
| foot switches × 5 + whiskers × 2 | seven free GPIOs, internal pull-ups | the J6 lines; each closes to GND |
| UART0 | GPIO 14 / 15 | spare: the D500 lidar uses its own USB adapter |
| 5 V in | header 5 V + GND, from the D24V50F5 + 1000 µF | Skips USB-C PD, so tell the firmware it has 5 A: EEPROM `PSU_MAX_CURRENT=5000`, or `usb_max_current_enable=1` |

**Free GPIOs** once those are taken: 4, 5, 6, 7, 12, 13, 16, 17, 20, 22,
23, 24, 25, 26, 27. That is 15 pins for INT, RST and the seven switch lines
(9), leaving 6. GPIO 7 is SPI0 CE1, free while the IMU is the only SPI
device.

**Conflicts:**
- **One I²S.** The Pi has one I²S interface, so the amplifier and an I²S
  microphone HAT cannot both use it.
- **The ReSpeaker 2-Mics HAT** (D-03, the alternative to amplifier + USB
  mic) takes the I²S pins, SPI0 MOSI/SCLK for its APA102 LEDs (which an
  SPI0 IMU would also clock), GPIO 17 and the whole header. With the IMU on
  SPI0, use the USB mic and the MAX98357A.
- **Sonar.** Trig/echo sonar needs 10 GPIOs for five pods. About 20 of the
  26 are spoken for without it, so sonar pods need an I²C sonar or an
  expander.

## Power budget

| load | nominal | peak |
|---|---|---|
| 15 × ST3215 walking | ~4–6 A @ 12 V | 8–10 A (stance transitions) |
| stall, worst case | — | the 15 A fuse is the ceiling, by design; per servo, `PROTECT_CURRENT` ≈ 2 A is set on bench day (D058; REVIEW §4; [bench runbook](../bench/BENCH_RUNBOOK.md) §5) |
| Pi 5 + camera | 1.2 A @ 5 V | 2.5 A (the 5 A buck has headroom) |
| D500 lidar | 0.29 A @ 5 V | — |
| 5 × SCS0009 | 0.2 A @ 6 V | ~1.5 A all stalled (UBEC 3 A) |

Voltage sag: a 3S pack under 10 A dips about 0.3–0.5 V. The driver's
SafetyMonitor floor (`bus.safety.volt_min_v`, 9.9 V) allows for it. That
floor is software; the balance-lead alarm is the hardware floor. Log
brownouts in `NOTES_INBOX.md`.

## Build order (phase B)

1. **Bench:** dock + fuse + loop key + one XT30 drop → one leg on the jig.
2. **Measure the loom lengths on the robot** (deck v0.4 printed), starting
   from the star-board table. Don't guess them.
3. **Colour code:** heat-shrink red for 12 V, yellow for 6 V, black for GND,
   white for data, blue for sensors. Label every drop with its leg number at
   BOTH ends.
4. **Test before any servo plugs in:** continuity and polarity on EVERY
   drop, and 6.0 V on every pin 3. The D016 rule exists because one
   backwards moment cooks a hand.

## Parts

In [`bom/BOM.csv`](../bom/BOM.csv):
- rows B-08…B-18: the loop key, fuse, buck, UBEC, XT60/XT30, leg splitters,
  wire, JST-XH/SH and the star board;
- A-06: servo leads;
- B-07: the balance-lead alarm.

Gauges (B-15): 14 AWG for the pack run, 20 AWG for each leg's 12 V, 24–26
AWG for data, 6 V and sensors.
