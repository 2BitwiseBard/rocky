# Wiring harness (D016 + I1 / I4 / I5)

How power and data move through Pebble: the power tree, the five leg drops,
what runs under the deck, the star board, the Pi's pins and the power
budget. The electrical calls are D058's ([decisions](decisions.md)), which
settled the 2026-09-22 review's findings
([archive/REVIEW_2026-09-22.md](archive/REVIEW_2026-09-22.md) §4) and kept
the Pi + adapter as the bus master (the ESP32 board is a bench tool). The
under-deck layout is D064's, built from the owner's body-layout picks of
2026-10-07 ([INTERFACES.md](INTERFACES.md) I4 / I5). The rule that saves a
servo: **hand servos never see 12 V** (D016). Parts are rows in
[`bom/BOM.csv`](../bom/BOM.csv); nothing is wired yet.

## Power tree

```
 3S LiPo 5200, soft case, on the sled (I5) ── balance-lead voltage alarm (B-07)
   XT60E-M on the sled's nose ══ female XT60 in the tub's floating carrier
     │ in the tub's nose compartment
   15 A blade fuse ── XT60 loop key (panel socket in the nose wall, facing +x)
     │ 14 AWG silicone: out through the NE nose chamfer, up the north-wall
     │ riser, west to the shelf
   12 V PDB (30.5 × 30.5) on the hub shelf
     ├─ 5 leg drops: XT30 each, 20 AWG (below)
     ├─ 5 V 5 A buck + 1000 µF (shelf) ── tray XT30 ── Pi 5 GPIO 5 V
     ├─ 6 V UBEC, jumper at 6 V (shelf) ── star board J0.3: the hand rail
     ├─ bus adapter's 12 V input (shelf): its logic, in UART mode
     └─ 2 × XT30 spare taps (the B8 LED ring, accessories)
```

- **Fuse, then loop key**, right after the pack (B-08, B-09), both in the
  tub's nose compartment. The fuse holder lies over the XT60 carrier: 16.3
  mm of height is free there (13.85 over the carrier's stop window), VERIFY
  with the holder bought (B149). If it does not fit, it moves onto the 14
  AWG run north of the tub and the key comes first. The loop key is the
  arming plug and the E-stop; its panel XT60E-M in the nose wall is
  reachable with the carapace on. A DC-rated ≥ 20 A rocker is the
  alternative.
- **The PDB** (B-24, pick 9) is a 30.5 × 30.5 FPV-style board, ≥ 15 A, no
  BEC. It feeds ten loads from one input: five leg drops, the buck, the
  UBEC, the adapter's 12 V and two spare taps. On a board with fewer pad
  pairs, leads share a pad: VERIFY with the PDB bought. The spare taps sit
  after the fuse now (B66 closed: the dock block that carried them is
  retired).
- **The Pi 5** is fed through its GPIO 5 V pins from a Pololu D24V50F5 buck
  with 1000 µF (B-10), over the tray's bulkhead XT30. This skips USB-C PD,
  so tell the firmware it has 5 A ([pin map](#pi-5-pin-map-draft-verify)).
- **The 6 V UBEC** (B-11) is a Hobbywing UBEC-3A with its jumper at 6 V.
  Measure 6.0 V at the plug before any claw is connected.
- **The bus adapter's logic** runs only from its 12 V input in UART mode:
  with 12 V off (a Pi on its own USB-C supply on the bench) its RX is
  unpowered and the bus reads dead.

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
   is clear at every yaw. The mated XH-5 pair does not pass the 11 × 11
   cutout (B120).
2. **Power fans out at the coxa.** The XT30 splits into three short 12 V
   leads, one per servo (B-14). Never carry a leg's 12 V through the servos'
   2 A connector pins. Data and GND daisy-chain yaw → hip → knee on stock
   Feetech 3-pin leads (A-06). The exact lead construction is VERIFY on
   bench day.
3. **Down the leg to the SEA and hand.** The hand's 6 V (XH-5 pin 3) runs
   the whole leg: the coxa channel (~130 mm to the yaw plugs), yaw → hip
   (~90 with the yaw loop), hip → knee (~125 with the hip and knee loops),
   knee → hand hub (~110 on the shortest tube, plus the tube's exposed
   length), about 0.45–0.55 m in all. Data and GND join it for the last
   hop (from the knee's spare port with the V+ pin pulled, or with it from
   XH-5 pins 1–2). It ends in a JST-SH-3 above the I2 socket with a loop
   for the 90° twist, and the microswitch pair runs back up to XH-5 pin 4
   the same way. B-17 is only the end of this run (100 mm, a plug on both
   ends), and the hand's mating half is not chosen yet. VERIFY on the
   first leg (B91). Along the femur the leads ride a `link_clip` over plate
   A's top edge, above its lightening slot (D063, B94; the tunnel is 4 × 6
   through a 2.6 mm slot).

**Check at the first bench torque test:** the cup's −Y side wall now
bridges the channel over 10 of its 24.5 mm footing. Watch it for flex.

## Bus topology (one logical TTL bus, D003)

The Pi drives the bus over its UART (GPIO 14 / 15) through the Waveshare
Bus Servo Adapter (A) on the hub shelf (A-02, pick 14); on the bench the
same adapter runs over USB-C. `rocky_driver.SerialTransport` takes any port
path, so only the `/dev/ttyACM0` defaults change for the robot. The ESP32
Servo Driver is a bench first-motion tool, not the bus master (A-03).

It is a star of daisies. The adapter's data and GND and the UBEC's 6 V go
into the **star board's** J0 (all three boards are on the shelf; the
adapter's own V+ goes nowhere, D016), and the star fans out to five XH-5
leg drops. Each leg is its own daisy chain (yaw → hip → knee → hand), so a
wiring fault in one leg unplugs at its port without taking down the other
four. In a straight line the cutouts are 52–119 mm from the star, well
inside the 400 mm stub budget at 1 Mbps; the leg's own chain adds the
leg's length.

## Under the deck (D064)

Nothing of the harness runs on the deck top: the coxa plates cover the
cable cutouts (B109), so every drop is reached from under the deck. The
deck top carries the tray alone. Body frame, mm; the deck is z −10 … −4.

### The hub shelf

`hub_shelf` hangs on three posts at (−50, 34), (50, 34), (26, 64), each on
an M3 × 30 from below; its plate's top is z −36.3
([INTERFACES.md](INTERFACES.md#hub-shelf-under-the-deck)).

| board | centre | turn | facing | z |
|---|---|---|---|---|
| 12 V PDB, 36 × 36 (30.5 pattern) | (−20.8, 36.2) | 0 | up, 5 mm standoffs, M3 × 6 | −36.3 … −23.7 |
| star board, 40 × 30 | (25.2, 28.2) | 0 | up, 5 mm posts, M3 × 6 | −36.3 … −13.9 |
| bus adapter (A), 42 × 33 | (−20.2, 28.8) | 0 | face-down, four 2.5 spacers, M2.5 × 8 from below | −54.4 … −39.3 |
| 5 V buck + 1000 µF | (35.8, 26.2) | 90° | face-down, one 3.6 zip tie | −49.3 … −39.3 |
| 6 V UBEC | (−1.8, 55.8) | 0 | face-down, two 3.6 zip ties | −47.5 … −39.3 |

- The face-down boards' leads come round the plate's south corners to the
  PDB and the star through two zip-anchor slot pairs: the adapter's west
  plugs at the SW pair, the buck's east leads at the SE pair.
- The PDB's parts and flat-soldered leads are taken as ≤ 6.0 over its board
  (VERIFY with the PDB bought, B152): leg 1's loom passes 1.20 over that.
- The buck's capacitor lies beside it, held by its leads and a dab of glue
  (VERIFY).
- Service means the robot off the stand (three of the adapter's four
  screws are under the stand's crown plate, B134).

### The 14 AWG feed (B133)

From the fuse and the key in the tub's nose, out through a Ø5.5 hole in the
middle of the NE nose chamfer just over the floor (centre (94.9, 2.4,
−50.5)), down to z −52.7, west under leg 4's drop along the tub's north
wall, up a riser at (59.15, 10.3) held by the snap clip on the north wall
(z −44 … −38), west at z −23.7 to x −6.8, north to the PDB's input pads:
145 mm from the chamfer as modelled (Ø5: the 14 AWG pair + a sleeve).

- Under leg 4's drop it is tight: 0.20 to the floor of the drop's +5 mm
  keep-out, 0.20 over the tub's bottom (the robot's lowest point), 0.40 off
  the north wall. Nothing can hold it there (the window is 5.4 for a Ø5), so
  it spans 26.5 mm free between the exit and the clip. A real bend of about
  R 8 at the riser's foot could come within 0–0.3 of the drop keep-out's
  floor (an estimate, not modelled).
- No grommet: the exit leaves 0.25 round the feed (a short heat-shrink
  sleeve there is optional). The clip's snap fit is VERIFY (PETG, 1.8 %
  strain): print one and try it.

### Leg looms

- **Legs 0, 1, 4** run straight from the star to their cutouts (Ø7, z −19,
  the star's lead band under the hook feet). Tie points: legs 1 and 4 at the
  shelf posts (±50, 34), leg 0 at the deck's (12, 50) zip anchor.
- **Legs 2, 3** run over the tub in the 8 mm layer (z −17 … −10; the tub's
  roof is at −18), each in an 8 × 7 lane: A south from the shelf at x ±45 …
  53 (y 12 … −38), B the jog out to x ±59.8 (it must turn north of y −38:
  south of it the lane meets the hook sweep, B135), C at x ±51.8 … 59.8
  down to y −53.4; the riser to the cutout stays inside the leg's drop
  keep-out. Leg 2 first crosses along the shelf's south edge (y 9.5, z −14)
  to its lane A, 0.16 from the tub's north return. Tie points: the lid's
  tie bars beside its (±40, 0) hanger bosses, inside lane A.

### Through the deck: the (0, 50) trunk slot

A 12.5 × 7.5 slot (r 1, east-west), 11.25 north of the tray bulkhead's
outer face (y 35.0). Through it go the tray's XT30 (5 V from the buck) and
XH-5 trunk (the Pi's UART), and the five J6 foot-switch lines to the Pi's
header.

- The plugs pass on: the XT30 with 1.15 a side, the XHP-5 with 0.36 a side
  if its 22 AWG leads fold in two layers (a single-layer fold, 8.0 wide,
  does not pass). Or route the leads before terminating them.
- Under the slot a cable straight down cuts the PDB's lead halo. The
  modelled route (≤ Ø9.42) runs west over the PDB at z −18.7, down round
  the plate's NW corner at x −50.5, and south under the plate into the
  adapter's west plug zone, about 91 mm from under the slot (VERIFY: where
  the adapter's H2 sits on its board is not in the repo, B151; the length,
  B67).
- The XT30 and the trunk unplug at the bulkhead, the J6 lines at the Pi's
  header, before the tray's 16 mm slide (B123: the J6 lines have no
  bulkhead connector).

### The UART link (pick 14)

The tray trunk's XH-5 carries three wires: the Pi's header to the
adapter's H2.

- **H2:** 1 GND, 2 RXD, 3 TXD (where it sits on the board: B151); jumper H4
  to A (UART). The adapter has 3.3 V
  buffers and switches direction in hardware (no echo).
- **Cross the signals:** H2 RXD → Pi pin 10 (GPIO 15), H2 TXD → pin 8
  (GPIO 14). A straight 1:1 lead swaps them.
- **GND from pin 9 or 14:** pin 6 is the buck's (5 V on pins 2 / 4).
- **At the adapter:** solder the leads to H2, or use a right-angle housing
  inside the board's 11.0 parts envelope. Face-down, a straight Dupont on a
  vertical H2 would hang to z −59.9, into the stand's crown plate.
- **Pi side, unverified:** `dtparam=uart0=on` gives `/dev/ttyAMA0` on GPIO
  14 / 15, 1 Mbaud on the RP1.

### The one USB-A port

The Pi's USB end faces body +x (leg 4) on 11 mm standoffs, so only the
lower middle USB 3 port (next to the RJ45) takes a plug: a right-angle plug
≤ 20 long from the port face, overmold ≤ 14 × 7, the cable leaving down or
south (21.59 of room at the worst float). It is the D500 lidar's USB
adapter (C-05). The other three ports have 8.4–11.8 of room: the mic (D-02)
or the Wi-Fi adapter (X-04) need a short extension or a hub, or the lidar
moves to a Pi UART (GPIO 4 / 5 or 12 / 13, its logic level unchecked;
B128).

## Star board (bus hub)

### The board

Cut a 2.54 mm proto board to 40 × 30 (16 × 12 holes). Drill the corners
Ø3.2 at 34 × 24 mm for the shelf's star posts. Parts: B-18.

| ref | part | job |
|---|---|---|
| J0 | JST-XH-5, right-angle | from the adapter and the UBEC on the shelf: 1 data, 2 GND, 3 6 V, 4 spare, 5 spare |
| J1–J5 | JST-XH-5, vertical | one per leg drop; mark the leg number |
| J6 | JST-XH-6, vertical | the five foot-switch lines to the Pi: sw0…sw4 + GND |
| W1 | data lane (26 AWG) | J0.1 → J1…J5 pin 1 |
| W2 | GND lane | J0.2 → J1…J5 pin 2 and J6 GND |
| W3 | 6 V lane (24 AWG) | J0.3 → J1…J5 pin 3 |
| W4 | foot-switch lines | J1…J5 pin 4, each to its own J6 pin (sw0…sw4); J0.4 unused |
| — | pin 5 | left open (reserved); continuity-test to nothing |

No resistors are needed. The Feetech bus is push-pull from the adapter, and
each switch line uses the Pi GPIO's internal pull-up.

### Cut lengths

The routes above are modelled as clearance envelopes, not cut lengths:
VERIFY every length on the printed parts (the deck, the shelf and the tub;
B109, B67).
Route each loom dry, then crimp. The old table (looms over the deck to a
star at (0, 32) and a node at (52, −6)) is void.

### Star-board build order

1. Populate J0–J6 on the board, run the four lanes, and continuity-buzz
   every pin pair.
2. Screw the PDB and the star onto the shelf (M3 × 6), the adapter under
   it on its spacers (M2.5 × 8), the buck and the UBEC under their ties.
   Then the shelf onto the deck (3 × M3 × 30 from below).
3. Dry-route the five looms, the 14 AWG feed and the tray's leads on the
   printed parts, THEN cut and crimp them at their true lengths.

## Pi 5 pin map (draft, VERIFY)

A plan, not wiring: nothing is connected yet. The rows are B-03 (Pi),
C-01 (IMU), D-01 (amplifier), X-01 (INA228) and X-02 (ToF). The Pi's own
OS and microSD setup (the image, SPI on, the 5 A setting below, dialout)
is not written yet (B96).

| function | pins | notes |
|---|---|---|
| BNO085 IMU on SPI0 | GPIO 8 (CE0), 9 (MISO), 10 (MOSI), 11 (SCLK), plus INT and RST on two free GPIOs | Set the breakout's P0/P1 for SPI (VERIFY in the Adafruit guide). Not I²C: a Pi mishandles its clock stretching. Not UART-RVC: it sends no gyro rates |
| I²S out → MAX98357A | GPIO 18 (BCLK), 19 (LRCLK), 21 (DOUT) | optional (D-01) |
| I²C1 | GPIO 2 (SDA), 3 (SCL) | INA228 at 0x40, TCA9548A mux at 0x70, and the downward ToF sensors behind the mux (VL53L4CDs all share 0x29) |
| foot switches × 5 + whiskers × 2 | seven free GPIOs, internal pull-ups | the J6 lines; each closes to GND |
| UART0 | GPIO 14 (pin 8) / 15 (pin 10), GND pin 9 or 14 | the bus adapter (pick 14; [traps](#the-uart-link-pick-14)) |
| 5 V in | header 5 V (pins 2 / 4) + GND (pin 6), from the D24V50F5 + 1000 µF | Skips USB-C PD, so tell the firmware it has 5 A: EEPROM `PSU_MAX_CURRENT=5000`, or `usb_max_current_enable=1` |

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

1. **Bench:** an XT60 feed lead + fuse + loop key + one XT30 drop → one leg
   on the jig (the tub comes with the body print, batch 3).
2. **Measure the loom lengths on the robot** (the deck, the hub shelf and
   the tub printed). Don't guess them.
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
- B-24: the 12 V PDB; B-25: the carrier's female XT60; B-28 and B-30: the
  adapter's spacers and its UART lead; C-05: the right-angle USB-A cable;
- A-06: servo leads; B-07: the balance-lead alarm;
- B-23: cable ties (2.5 mm for the deck's (±12, 50) anchors, 3.6 mm for the
  shelf and the lid's tie bars).

Gauges (B-15): 14 AWG for the pack run, 20 AWG for each leg's 12 V, 24–26
AWG for data, 6 V and sensors.
