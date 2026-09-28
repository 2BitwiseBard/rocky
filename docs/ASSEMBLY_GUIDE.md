# Assembly guide

From the printed parts and the [bill of materials](../bom/BOM.csv) to a robot
on its bench stand, every servo calibrated, powered from the bench supply and
ready for its first steps. Nothing has been built from this guide yet, so
every VERIFY in it is real, and so is every item in [8. Open on the first
build](#8-open-on-the-first-build). Several of those are design gaps that
block a print; the guide holds the print where they bite.

This guide is the order of work; the detail lives in the documents it links:
[PRINT_PLAN.md](PRINT_PLAN.md) (what to print, poses, supports, criteria),
[bench/BENCH_RUNBOOK.md](../bench/BENCH_RUNBOOK.md) (servo bring-up and bench
tests), [WIRING_HARNESS.md](WIRING_HARNESS.md) (power tree, bus, star board,
pin map, cut lengths), [INTERFACES.md](INTERFACES.md) (I1–I6) and
[bom/BOM.csv](../bom/BOM.csv) (bought parts by id; printed parts go by their
`cad/out/` names). Measurements go into [`NOTES_INBOX.md`](../NOTES_INBOX.md),
then `cad/params.yaml` and `BUILD_LOG.md` (D032). The pictures come from
`cad/gen_assembly_views.py` (`python3 gen_assembly_views.py leg_03` in `cad/`
redraws one).

## Contents

[The build at a glance](#the-build-at-a-glance) · [Safety](#safety) · [1.
Before you assemble](#1-before-you-assemble) · [2. The leg](#2-the-leg) · [3.
The tibia and foot](#3-the-tibia-and-foot) · [4. The body
frame](#4-the-body-frame) · [5. Wiring](#5-wiring) · [6. Dock the legs and
power up](#6-dock-the-legs-and-power-up) · [7. The carapace](#7-the-carapace)
· [8. Open on the first build](#8-open-on-the-first-build)

## The build at a glance

1. **Printer and coupons:** fit ladder filed, batch 1 and the port coupon
   passed, CAD regenerated ([1](#1-before-you-assemble)).
2. **The fork fixed before any leg print** ([8.2](#82-the-leg)).
3. **One leg dry-fitted on the blanks before any servo arrives**, with the
   harness threading tried ([2.2](#22-dry-fit-on-blanks)).
4. **Servos on the desk:** checked, numbered, smoke-moved
   ([1.5](#15-the-servos-arrive-on-the-desk)).
5. **The bench leg:** a long bench tube and its bench power lead (fuse,
   loop key, one XT30 drop, splitter), then calibrated and load-tested on the
   jig ([2](#2-the-leg)).
6. **Tube length and foot**, decided before five tubes are cut
   ([3](#3-the-tibia-and-foot)).
7. **Four more legs** ([2.11](#211-four-more-legs)); meanwhile the stand, the
   star board and the tray electronics on the bench.
8. **The deck only after its layout settles:** B27, B28, B51, the bay
   ([4.1](#41-when-to-print-the-deck)).
9. **Wiring before docking**, every drop tested unplugged ([5](#5-wiring)).
10. **Power-up on the stand, tethered:** legs docked one at a time, loop key
    out while plugging ([6](#6-dock-the-legs-and-power-up)).
11. **The pack once the bay exists** ([6.4](#64-the-pack)); **the carapace
    last** ([7](#7-the-carapace)).

## Safety

The full list is [bom/README.md](../bom/README.md#safety). During assembly:

- **ST3215: the 12 V / 30 kg·cm variant only** (the 7.4 V one looks the same,
  D002). **First contact current-limited:** 12.0 V, ~1.5 A, metered.
- **SCS0009 hand servos never touch 12 V (D016):** 6 V from their own UBEC,
  measured at the plug before a claw goes on.
- **Never switch torque on first** (a Feetech servo enables toward its last
  goal, D052): use `soft_enable`.
- **A leg's 12 V never rides servo connector pins** (~2 A): it fans out at the
  coxa (B-14).
- **Fuse, then loop key**, first after the supply or pack; loop key out before
  plugging or unplugging a leg.
- **Threadlocker (A-14) metal to metal only:** 243 crazes PETG.
- **LiPo:** soft case only (≤ 138 × 44 × 25 mm), alarm on the balance lead
  whenever the robot runs, charged in a LiPo bag, never unattended.
- **Carbon dust:** mask and glasses, cut wet or under extraction.

## 1. Before you assemble

Get to the first leg with your printer's fits filed, coupons that pass, clean
parts, the inserts in, and every servo checked, numbered and labelled on the
desk.

### You need

- **Printed, PLA:** `fit_ladder` (batch 0); `servo_blank`, `blank_idler`,
  `coupon_cup`, `coupon_yaw_hub`, `coupon_hip_hub`, `coupon_idler`,
  `horn_coupler` (batch 1); the [interface
  coupons](PRINT_PLAN.md#interface-coupons-pla-02-mm-with-batch-1) (2 ×
  `thumb_knob_m3`, the latch, dovetail and panel pairs, `tool_hook`,
  `tool_scoop`); the port coupons, `port_coupon_deck` + `port_coupon_plate`
  (PRINT_PLAN has them in batch 0's GO / NO-GO table and in batch 3; print
  them now). `coupler_recess_demo` is not needed: `coupon_hip_hub` covers
  it. Print from the current `cad/out/`; pre-D047 leg parts do not fit the
  servo.
- **Servo bring-up:** A-01 4 × ST3215 12 V, A-02 adapter, A-04 bench supply
  (or A-05: fuse its lead, watch the wattmeter), A-07 barrel pigtails; A-03
  optional (12.0 V from the bench supply, never a full 3S pack).
- **Hardware and tools:** A-08, A-09 M3 × 6, A-10 M3 × 8, A-12 (only for an M2
  horn), A-13 inserts, A-14 threadlocker + CA; X-11 calipers, X-12 multimeter,
  X-13 scales, X-14 iron with an insert tip, a Ø2 pin, X-19 dryer for PETG.
- **Added to the BOM with this guide:** A-19 M3 × 16 hex-head bolts (ISO
  4017 / DIN 933, 5.5 mm A/F) for the thumbscrews
  ([1.3](#13-batch-1-coupons-and-the-blank)) and the trim cup; B-23 cable
  ties; B-24 the 12 V node. B-21's kit carries the IMU's M2.5 nuts; a thin
  one may be needed ([4.5](#45-the-avionics-tray-on-the-bench-i4)). **Not in
  the BOM yet:** a mating JST-SH 3-pin header for the hand's side of the I2
  disconnect (B91) and pack straps (B93).

### 1.1 Batch 0: measure the printer

Every clearance in `params.print` is a generic FDM guess until your ladder is
filed (B28). Its answers move the fits of every mating part.

1. Set `print.bed_mm` to your printer; the bed-fit checks assert it.
2. Print one `fit_ladder`: PLA, 0.2 mm, 3 walls, 25 %, sliced like the parts,
   elephant-foot compensation ~0.15 mm. An older printed ladder is fine; on
   ones from before 2026-08-30 row D's dots hide in the slots (3.8 / 4.2 /
   4.6, left to right).
3. Measure each row into `NOTES_INBOX.md`:

   | row | find | params key it sets |
   |---|---|---|
   | A | first Ø4 bore the 3.95 peg slides into | `print.clearance_fit` (4.30 bore = 0.30) |
   | B | first hole an M3 (A-09) threads cleanly | `print.screw_m3_tap` (2.8) |
   | C | pocket that holds an insert (A-13) snugly | `print.heatset_m3_d` (4.6) |
   | D | does the port coupon's lip pass the 4.2 slot | `interfaces.leg_port.hook_slot_w` (4.2) |
   | E | snug Ø2 pin hole; which Ø10 socket the tube seats in | pins: `part_hand.py` (2.1, no key); tube = 10 + `clearance_fit` |

   Row E's 6.85 / 6.95 / 7.05 pockets are the obsolete bearing seat.
4. `clearance_fit` sets every mating printed fit (servo cups, Ø20.3 horn and
   idler pockets, dowel bores, tube sockets), `screw_m3_tap` every
   thread-forming hole, `heatset_m3_d` every insert pocket.
5. File the numbers in `cad/params.yaml` (the purpose in the comment: slide,
   press, tap), then run `./rocky.sh cad-check` and `./rocky.sh cad-check
   --derived` ([the
   loop](DESIGN_CHANGE_GUIDE.md#0-the-loop-every-change-goes-through)).
6. Printing before you regenerate is allowed for bores only, by the [GO /
   NO-GO table](PRINT_PLAN.md#batch-0-measure-the-printer-b28-blocking): 4.30
   slides and 4.20 binds is GO; only 4.40 / 4.50 slides means slicer XY hole
   compensation +0.05…+0.10 mm, GO, and file it. If only the 4.6 slot passes
   row D, regenerate with `hook_slot_w` 4.6 before the deck.

### 1.2 Heat-set inserts

Every production pocket is `heatset_m3_d` × `heatset_m3_h` (4.6 × 5.7), cut
from the top face; the ladder's row C has fixed 4.4 / 4.6 / 4.8 holes.

| part | inserts | where | when |
|---|---:|---|---|
| `fit_ladder` | 3 | row C, one per hole | now |
| `port_coupon_deck` | 2 | its I1 port | batch 1 |
| `jig_base`, `jig_column` | 6 + 2 | under the column flange; the column's I1 port | [2.9](#29-calibrate-the-first-leg-on-the-jig) |
| `body_deck` | 10 | 2 per I1 port, leg-local (−41, ±17) | [4.2](#42-prepare-the-deck) |

1. Insert tip on the iron (X-14); your first inserts go in row C.
2. Stand the insert on the pocket, press straight down lightly, let the heat
   work. Stop flush or just below: **a proud insert on the deck holds the coxa
   plate off it**, and plate face on deck face takes the load.
3. Keep it square: an M3 runs in by hand through the plate's Ø3.7 bore.

### 1.3 Batch 1: coupons and the blank

Each coupon is a boolean clip of its production part
(`cad/part_leg_coupons.py`): a coupon that fits proves that part.

1. Print batch 1 in the poses of its table in [PRINT_PLAN.md](PRINT_PLAN.md);
   none needs support.
2. Glue the `blank_idler` into the blank's Ø6.2 pocket with CA (A-14), disc
   flat on the bottom face; the dry-fit blanks later get the same.
3. `coupon_cup` (case fit, rim clearance, screw positions of all three cups):
   the blank slides in rear-first with finger pressure and does not rock; a Ø2
   pin through each rim hole lines up.
4. `coupon_yaw_hub` on the blank's horn: under 0.3 mm wobble; M3 × 6 screws
   pass at both ends of all four slots (the blank's Ø2.5 pilots take M3 × 6 or
   M2 × 6, dry fits only).
5. `coupon_hip_hub` + `horn_coupler`: the coupler drops into the recess and 2
   × A-10 from the outer face draw it flat; a 1 mm sideways push meets the
   lobes.
6. `coupon_idler` drops over the glued idler without rocking; the plug notch
   is clear.
7. **Port coupon (I1)**, the rehearsal the deck waits for:
   1. 2 inserts (A-13) in `port_coupon_deck`, flush.
   2. The thumbscrews: press an A-19 M3 × 16 hex-head bolt (not A-11's socket head)
      into each `thumb_knob_m3`'s hex pocket from the top, thread out of the
      bottom. Snug on purpose: the head must not turn in the knob; ~0.5 mm
      stands proud. A round socket head spins in this pocket.
   3. `port_coupon_plate` at ~15°, lip through the 4.2 mm slot, inboard to
      hook, pivot flat (both Ø4 dowels in without force). Then a knob in each
      hole, finger-tight: the M3 × 16 takes 5.5 of the insert's 5.7 mm and
      stays inside the underside. **Not longer than 16:** on the bench a
      longer screw hits the table before it clamps.
   4. Tug in every direction; note what binds. Fix `interfaces.leg_port` or
      `print.*` and regenerate, not the prints. The deck waits until this
      docks cleanly. The knobs go on to the bench leg.
8. **The other interface coupons** gate nothing. The latch pair: look at it;
   as drawn its halves cannot be joined ([8.4](#84-the-body)). The dovetail
   pair: check only that the shoe slides down the 24 mm segment, record "knob
   lock: not testable, the bore misses the dovetail", drive no screw.
   `shell_sector_demo` + `frame_coupon` are not a mating pair (latch, lip and
   magnets sit at different positions): feel the lip and the magnet pull
   separately. The tool coupons need a printed `tibia_sea_slider`
   ([3.5](#35-the-foot-on-the-i2-tool-socket)).
9. Every verdict goes in `NOTES_INBOX.md`.

The blanks are the dry-fit servos and stay as bench dummies. They have no
connector housing, so they cannot test the plug keep-outs; three engraved dots
on each side mark one.

### 1.4 Clean up the leg parts

For batch 2 as it comes off the printer: PETG, dry, supports only where its
PRINT_PLAN table says, sliced in the poses `check_printability.py` audits.

1. `coxa_yaw_base`: all support out from under the plate; first-layer flare
   off the hook lip's foot; the plate's underside scraped flat (it bears on
   the deck). The 11 × 11 harness channel clear end to end (its first stretch
   is roofed over an 11 mm bridge, plate on support), the Ø4.3 dowel bores
   (chamfer from below) and Ø3.7 thumbscrew bores clear.
2. `coxa_fork` has support in three places: under the upper plate where it
   leaves the web (z ~41), under the hip cup's lower wall where it steps out
   past the plate (the 13.7 mm ledge at z 44), and under the hip cup's upper
   side wall over the hip servo (the 21 mm step at z 72.5). Clear all three,
   clean the servo-side faces of both hip-cup walls, the Ø20.3 horn pocket,
   its slots and counterbores, and the idler pocket.
3. `tibia_knee_carrier`: support out; check the Ø10.3 × 18 socket, the 1.6 mm
   slit and the spot face.
4. `tibia_sea_outer`: the socket mouth carries the first-layer squish. Check
   the Ø13.2 bore and the 6.8 × 13.2 × 6.2 switch pocket against a real KW10
   (A-17) before you print five.
5. `femur_link`, `femur_plate_b`, `horn_coupler`, `tibia_sea_slider`: no
   supports; clear stringing.
6. Deburr a first-layer lip off a bore mouth. **A bore small all the way down
   is a params problem:** fix the key and reprint. Hand reaming is only for
   press seats and the hand's pin holes.
7. A blank slides into each cup from the front: finger pressure, no rock.

### 1.5 The servos arrive: on the desk

Fresh servos all answer as `id 1`, so IDs are set one servo at a time, loose
on the desk. Center calibration is the opposite: servo built in, on the jig
([2.9](#29-calibrate-the-first-leg-on-the-jig)).

1. Rehearse with the `--mock` commands of
   [BENCH_RUNBOOK.md](../bench/BENCH_RUNBOOK.md).
2. Every label says **12 V / 30 kg·cm**; the 7.4 V variant looks the same and
   does not close the torque budget (D002): return it.
3. Both discs present: the Ø20 metal output horn (the C018 sheet lists a POM
   one) and the Ø19.9 rear idler. Count the bag against Waveshare's listing
   (two aluminium horns, screws, one 150 mm lead) and compare its self-tappers
   with A-08 (PA2.0 × 6; pilot 1.5 mm).
4. Supply, no servo: A-04 at 12.0 V, ~1.5 A, confirmed with the meter (A-05:
   fuse its lead, watch the wattmeter). Adapter, `dialout`, `pip install -e
   ".[hw]"`: [runbook §0](../bench/BENCH_RUNBOOK.md#0-before-anything-is-plugged-in).
5. First contact from `bench/`: `python3 bus_scan.py --port /dev/ttyACM0`
   finds `id 1`, 1 Mbps, ~12 V; log `model LE`
   ([§1](../bench/BENCH_RUNBOOK.md#1-first-contact-one-st3215)); dump the
   first servo's registers
   ([§2](../bench/BENCH_RUNBOOK.md#2-register-archive-as-shipped)). The dump
   stops at addr 73; B79 (open) extends it to 87 before any servo is changed,
   and step 6 changes each one.
6. IDs with `assign_ids.py`, one servo at a time
   ([§3](../bench/BENCH_RUNBOOK.md#3-id-assignment-one-at-a-time)): leg i is
   yaw, hip, knee = 3i+1, 3i+2, 3i+3; hands 16–20. With the bench kit, stop
   after id 3 and give the spare `--start-from 5` (leg 1's hip). Label each
   case with id and joint near the output end (the cup sleeves only the rear).
   Rescan.
7. Smoke motion: `soft_enable`, ±15° slowly, torque off
   ([§4](../bench/BENCH_RUNBOOK.md#4-smoke-motion-first-commanded-move-one-servo-on-the-desk)).
   **Never torque on first; never leave a servo holding on a desk edge.**
8. Measure the first servo against `servo_st3215` (STEP numbers, VERIFY: case
   45.4 × 24.8 × 28.8, horn Ø20 × 2.5, idler Ø19.9 × 2.1) and weigh it
   (`mass_g` 60 with horn); file each, regenerate, reprint what moved
   (expected: the coupler and the yaw hub).
9. Horn thread and radius (B23): seat `coupon_yaw_hub` on the real horn, drive
   one A-09 M3 × 6 through a slot by hand. It threads: M3. Otherwise A-12 M2 ×
   6 + washer in the same slots. The slots take a screw anywhere from BCD 14.0
   to 17.2 (r 7.0–8.6), covering the STEP's 14.0–15.6 with margin: no reprint
   for the radius. File `horn_screw_m` and `horn_bcd`.
10. **Threadlocker (A-14) on horn screws only:** off the printed hub or
    coupler, never on a screw into a printed part.

### Check before moving on

- `params.print` holds your ladder's numbers; `cad-check` ran clean.
- Batch 1 passes and is filed; the port coupon docks without force and both
  thumbscrews run in.
- Blanks slide into every cup without rock; the base's channel is clear, its
  underside flat; inserts flush and square.
- Each servo: 12 V label, metal horn, idler, a unique labelled ID, moved ±15°,
  left torque-off; `horn_screw_m` and `horn_bcd` measured.

## 2. The leg

Coxa, femur and knee: one leg built, hung on the bench jig, calibrated and
load-tested before four more are printed.

### You need

**Printed, per leg** (PETG; batch 2 in [PRINT_PLAN.md](PRINT_PLAN.md)):
`coxa_yaw_base`, `coxa_fork` (**only with its fix**, [8.2](#82-the-leg)), 2 ×
`horn_coupler`, `femur_link` (plate A), `femur_plate_b`, `tibia_knee_carrier`,
2 × `thumb_knob_m3` (PLA in the print pack; PETG is reasonable, the knob is in
the clamp load path). The bench leg uses the coupon knobs; batch 3 prints the
other eight.

**First leg only:** 3 × `servo_blank` + 3 × `blank_idler` (PLA), and the jig:
`jig_base`, `jig_column`, `calib_gauge_hip`, `calib_gauge_knee`
([Deferred](PRINT_PLAN.md#deferred): once the servos are in hand).

**Hardware** (design counts, [`bom/BOM.csv`](../bom/BOM.csv)):

| BOM | part | where | per leg | 5 legs | BOM qty |
|---|---|---|---:|---:|---|
| A-01 + B-01 | ST3215, 12 V | yaw, hip, knee | 3 | 15 | 16, one spare |
| A-08 | PA2.0 × 6 self-tapper | 4 rim screws × 3 cups | 12 | 60 | 100 |
| A-09 | M3 × 6 | 4 per horn: fork hub, 2 couplers | 12 | 60 | 100 |
| A-12 | M2 × 6 + washer | instead of A-09 if the horn is M2 (B23) | 12 + 12 | 60 + 60 | 20 + 20: short if M2 |
| A-10 | M3 × 8 | coupler clamps | 4 | 20 | 100 |
| A-18 | M3 × 12 | plate B to the bridge bosses | 4 | 20 | 50 |
| A-11 | M3 × 10 socket head | tube pinch bolt | 1 | 5 | 50 |
| A-19 | M3 × 16 hex head | I1 thumbscrews | 2 | 10 | 25 |
| A-13 | M3 heat-set insert | deck I1; jig 6 + 2 | — | 10 + 8 | 100 |
| A-15 | Ø10 tube, roll-wrapped | tibia; one long bench tube ([2.6](#26-tube-step-8)) | 1 | 5 | 2 × 500 mm |
| A-06 | 3-pin lead, 300 mm | entry lead; hip → knee (+1 for a hand on the knee's port) | 2–3 | 10–15 | 24 |
| — | the servo's own 150 mm lead | yaw → hip | 1 | 5 | 16 |
| B-13 | XT30U pair | the leg drop | 1 | 5 | 10 ([5.1](#51-power-tree)) |
| B-14 | 12 V splitter (DIY) | XT30 → three servo power legs | 1 | 5 | 5, no spare |
| B-16 | JST-XH kit | the XH-5 drop | 1 | 5 | 1 kit |
| B-17 | JST-SH 3-pin, 100 mm | half of one: the plug on the leg's hand lead | ½ | 3 | 10 |

**Bench power lead:** B-09 fuse holder + 15 A fuse, B-08 loop key, a B-12
XT60H pair, B-15 wire. **Tools:** drivers, a Ø2 pin, X-11 to X-14, a set
square, A-02 and A-04 with the [bench scripts](../bench/BENCH_RUNBOOK.md)
rehearsed on `--mock`; for the tube (no BOM rows) a 32 TPI blade or cut-off
wheel, masking tape, fine abrasive paper, an FFP2 / N95 mask, glasses, a 2.5
mm hex key.

### 2.1 Servos and horn position

Every servo goes through [1.5](#15-the-servos-arrive-on-the-desk) first.

1. Fit each servo's horn and rear idler before it goes into a part: the fork
   and plate B ride the idler's Ø19.9 rim, and their pockets over the horn and
   idler centre heads are blind.
2. **Horn position.** Calibration stores any offset, with one hard limit:
   `apply_limits.py` refuses a joint whose range (`joints.pos_deg` ± 2°)
   crosses the encoder's 0/4095 wrap. At its jig pose a servo must read within
   ~±88° of centre (2048) at the hip, ±108° at the knee, ±138° at the yaw.
   Parts go on in 90° steps, so park each servo at centre (the
   [§4](../bench/BENCH_RUNBOOK.md#4-smoke-motion-first-commanded-move-one-servo-on-the-desk)
   snippet with `bus.set_position(<id>, 0.0, speed_cps=400)`, then
   `bus.torque(<id>, False)`) and take the step nearest the jig pose. Note the
   positions you assembled at.

### 2.2 Dry fit on blanks

First leg only, before any servo arrives.

1. Steps 1–8 below on three blanks with glued idlers, starting with step 1 on
   the fixed fork. Screws self-tap into the blank horn's pilots, no
   threadlocker. The blank has no rim pilots: decide whether to drill 1.5 mm
   pilots through the cup holes, and note it.
2. With the blanks in, try the harness threading ([2.7](#27-leg-harness)): the
   leg-side XT30 and XH-5 through the coxa channel (under the gearbox plateau
   that step 2 covers) and the pin 3 and 4 leads at full length.
3. Check the batch 1 criteria on the whole chain, take it apart in the swap
   order ([2.10](#210-swapping-a-servo)), keep the blanks.

### 2.3 Coxa: steps 1 and 2

1. **Fork onto the yaw servo, off the base. As drawn this step cannot be
   assembled:** the horn's centre screw head catches the hub's rear lip and
   the idler catches the upper plate, so the C would have to be sprung ~2.5 mm
   open for ~17 mm, untested and below the SF 2.0 target in the web. Horn or
   idler fitted afterwards fail too (blind faces), and the blank is blocked as
   well. **Do not force it** (B80, [8.2](#82-the-leg)); with the fork's fix, try
   the blank first. `check_assembly` checks the final fit, not this path;
   steps 2–8 are unaffected.

   The fit: yaw servo at centre, into the C from its open (−X) side, horn down
   onto the lower hub, idler up into the upper plate's pocket, notch toward
   the plugs. 4 × A-09 up through the hub's slots, heads in the counterbores,
   243 on each tip in the aluminium horn, **none on the printed part.** As
   modelled the hub seats on the horn's centre screw head: screwed down, the
   horn sits ~0.3 mm into the Ø20.3 pocket with ~0.7 mm of air under its face.
   That is the model, not a print fault: do not sand or shim. Snug the screws
   evenly (overtightening dishes the hub); the slotted screws and the idler
   pocket locate it.
   ![Step 1: the fork onto the yaw servo; as drawn it does not slide on](../cad/out/assembly/leg_01.png)
2. **Servo and fork into the base.** Together into `coxa_yaw_base`'s cup from
   outboard (+X), servo rear first; the hub passes 1.3 mm over the plate. The
   cup walls locate the case and take the yaw torque. 2 × A-08 from above into
   the idler-face rim holes, 2 up through the Ø5 pockets under the plate, so
   the base is off the deck. The yaw plugs face up behind the fork's notch.
   ![Step 2: servo and fork into the base cup](../cad/out/assembly/leg_02.png)

### 2.4 Hip and knee: steps 3 and 4

3. The hip servo into the fork's cup from +X, rear first: horn −Y (plate A),
   idler and plugs +Y (plate B). 4 × A-08: 2 into the horn face from −Y, 2
   into the idler face from +Y.
   ![Step 3: hip servo into the fork's cup](../cad/out/assembly/leg_03.png)
4. The knee servo into `tibia_knee_carrier`'s cup the same way, tube boss
   underneath, 4 × A-08. Its case belongs to the tibia; its horn drives
   against the femur.
   ![Step 4: knee servo into the carrier cup](../cad/out/assembly/leg_04.png)

**Drive all eight hip and knee rim screws now:** afterwards the knee cup's two
horn-face screws sit under plate A's beam (B22).

### 2.5 Femur: steps 5 to 7

5. **Couplers.** Both servos at centre. A `horn_coupler` disc-down on each
   horn, lobes −Y, clamp bores along the femur line in the jig pose (hip:
   femur horizontal; knee: tube boss straight down). 4 × A-09 each through the
   counterbored slots (the hub's BCD 14.0–17.2 pattern), 243 on the tips only.
   Drive them now: plate A covers them.
   ![Step 5: couplers onto the hip and knee horns](../cad/out/assembly/leg_05.png)
6. **Plate A (`femur_link`)** from −Y over both couplers' lobes at once,
   holding the knee assembly so its coupler meets hub B; the castellation
   locates each hub in X and Z. 2 × A-10 per hub from the outer face into the
   lobes' 4.8 mm blind thread-forming bores (tip ~0.2 mm short), no
   threadlocker, drawn in evenly until flat. Torque now runs horn → lobes →
   plate A; the lobes shear in a crash.
   ![Step 6: plate A onto both couplers](../cad/out/assembly/leg_06.png)
7. **Plate B (`femur_plate_b`)** from +Y: the tower pockets ride the hip and
   knee idlers, notches toward the plugs. 4 × A-18 through the 8 mm rails into
   the bridge bosses (4 mm into a 7.2 mm thread-forming bore). Plate B only
   locates; it never touches a horn.
   ![Step 7: plate B over both idlers](../cad/out/assembly/leg_07.png)

### 2.6 Tube: step 8

The robot's tube length is [3.1](#31-decide-the-tube-length-and-the-foot). The
bench tube is cut long, for calibration: `calib_gauge_knee`'s V spans leg z
−134 to −88 as modelled, and a seated tube's top is at leg z 44.3 (16.7 mm
below the knee axis at z 61), so a bare tube needs ~132 mm to reach the V and
~178 mm to span it (6 mm less on the printed jig, whose leg z 0 is jig z 134,
not the gauges' 140: [8.2](#82-the-leg)). A-15's ~120 mm stops above the V.
Cut it to sit in the V, and note the length.

1. Roll-wrapped only (A-15): pultruded tube splits under the pinch bolt.
   Caliper the OD (`leg.tibia_tube_od` 10.0, VERIFY); try it in ladder row E
   and both printed sockets (bored 10.3).
2. Two turns of masking tape over the mark; cut through it with light strokes,
   turning the tube. **Mask and glasses; vacuum at the blade or cut over a
   damp cloth; wipe up wet.** Square and deburr the end on abrasive paper laid
   flat; wash your hands.
3. Push the tube up into the carrier's Ø10.3 socket until it stops: 18 mm,
   blind, ending 16.7 mm below the knee axis.
4. One A-11 M3 × 10 from outboard (+X), head in the spot face; it clears the
   near jaw (Ø3.4) and forms its thread in the far one (Ø2.8, ≥ 4 mm). Tighten
   until the tube will not turn by hand, then stop. **Snug only: no crushing,
   the 1.6 mm slit still shows a gap** (it closes ~1 mm before it grips). **No
   threadlocker:** it threads into PETG.

With the femur horizontal and the knee at −90° the tube hangs vertical: the
jig's calibration pose.
![Step 8: tube into the carrier](../cad/out/assembly/leg_08.png)

![The finished leg skeleton](../cad/out/assembly/leg_done.png)

### 2.7 Leg harness

Try it on the blanks; lay it for good on the real servos. The lead
construction is VERIFY: write what you find into [In-leg
routing](WIRING_HARNESS.md#in-leg-routing).

1. The drop: one XT30 (12 V + GND, 20 AWG) and one JST-XH-5 (1 data, 2 GND, 3
   hand 6 V, 4 foot switch, 5 reserved), mated at the I1 cable cutout (leg x
   −29) with the plate hooked, not pivoted ([per-leg
   drop](WIRING_HARNESS.md#per-leg-drop-5-ends-at-the-i1-leg-port-connectors)).
2. The leg side runs up the base's 11 × 11 channel: under the gearbox plateau
   (x −34.5 to −13, z −4 to 7), to −Y under the cup wall, up beside the cup (y
   −26.9 to −15.9), over to the yaw plugs (z 57.4 to 68.4). It keeps to −Y,
   clear of the hip cup's +Y sweep over ±40° (`check_assembly`).
   ![The harness channel through the coxa base](../cad/out/assembly/leg_harness.png)
3. **Data, three leads per leg:** a 300 mm A-06 entry lead (one plug cut off,
   data and GND crimped into XH-5 pins 1–2), the yaw servo's own 150 mm lead
   for yaw → hip, a 300 mm A-06 for hip → knee; a fourth if the hand takes its
   data from the knee's spare port.
4. **12 V, the splitter (B-14):** the XT30 fans out into three 12 V legs, each
   spliced into the V+ wire of the lead plugged into its servo; one way is to
   cut that V+ back toward the previous servo, so no leg current rides a
   connector pin (VERIFY).
5. **The hand's 6 V:** B-17 is only the connector at its end. Run yellow 24
   AWG from pin 3 with the data leads (channel ~130 mm, yaw → hip ~90, hip →
   knee ~125, knee → hand ~110 plus any exposed tube): ~0.45 m on a 29 mm
   tube, ~0.55 m on 120 mm. Cut 0.6–0.7 m, leave loops at each joint and the
   bayonet, trim, log it in `BUILD_LOG.md`. Data and GND ride with it from
   pins 1–2, or come off the knee's spare port on a lead with its V+ pin
   pulled (a stock lead puts 12 V on the hand). The leg end is half a B-17,
   its red (middle-pin) wire re-marked; the hand end needs a mating SH-3
   header. **Pin 3 is 6 V, never 12 V (D016).**
6. **Foot switch pair:** two blue 26 AWG leads from pins 4 and 2 (GND), at
   full length now; the switch end is [3.3](#33-wire-the-foot-switch).
7. Each servo's two 3-pin headers sit on its idler face; no rigid part enters
   their 12 mm keep-out (VERIFY), so plugs come out with the leg built.
8. Slack across every joint (yaw ±40°, hip −70 to 90°, knee −150 to −20°).
   `link_clip` does not fit the D062 femur ([8.2](#82-the-leg)): tie the leads
   to the femur another way, e.g. a zip tie at the bridge walls.
9. Before any servo is plugged in: continuity and polarity on the drop, the
   leg number on both ends, and 6.0 V (never 12 V) on pin 3 once a UBEC feeds
   it.

### 2.8 The bench leg's power lead

Build order step 1 ([WIRING_HARNESS](WIRING_HARNESS.md#build-order-phase-b))
is "dock + fuse + loop key + one XT30 drop → one leg on the jig". The dock
waits for the bay ([4.3](#43-under-the-deck-nothing-yet)), so the bench supply
feeds the fuse on a loose XT60 lead. Build it before calibrating.

1. This leg's splitter (B-14, [2.7](#27-leg-harness) step 4), and the robot's
   15 A fuse (B-09) and loop key (B-08), built on the bench as in
   [5.1](#51-power-tree) step 2, with one XT30 pair (B-13) after the key to
   the jig's cutout, fed from A-04 on a loose XT60 lead (B-12 XT60H pair).
2. A data lead from the adapter (A-02) to the jig-side XH-5 pins 1–2 (not
   drawn in the repo). **The adapter's servo V+ never enters the XH-5.**
3. Nothing plugged in: continuity, polarity; supply 12.0 V, ~1.5 A, output
   off, checked; then output on, key in: 12.0 V, + on red, at the jig's XT30.
   Key out again before the leg plugs in.

### 2.9 Calibrate the first leg on the jig

Zero lives in the assembly, so the leg is calibrated built, on a jig with the
deck's I1 port: a leg that bolts to the jig bolts to the robot.

1. Inserts: 6 in `jig_base`, 2 in the deck-proxy top
   ([1.2](#12-heat-set-inserts)). The column flange down with 6 × A-10 M3 × 8
   (4 mm into the 5.7 mm insert); the base clamped by its end tabs or bolted
   through its four Ø5.4 holes.
2. **Hang the leg, loop key out:** tilt ~15°, lip through the slot, slide
   inboard to hook; mate the XT30 and XH-5 at the column's cutout while
   hooked; pivot flat onto the two dowels. Then a knob with its M3 × 16 in
   each inboard hole, finger-tight. They only clamp; the dowels and the hook
   lip take shear and torque
   ([I1](INTERFACES.md#i1--leg-port-deck--coxa-base-the-flagship)).
3. Jig pose: yaw on the protractor's 0 tick; femur horizontal on
   `calib_gauge_hip`; knee −90° with `calib_gauge_knee`'s rib in the groove at
   x 140 and the tube in its V. **Check both with a set square:** as modelled
   both gauges are off ([8.2](#82-the-leg)), and `pose_check` cannot see it.
4. Loop key in.
   [§5](../bench/BENCH_RUNBOOK.md#5-center-calibration-in-the-bench-jig):
   `calibrate_centers.py --only leg0` (torque off, you hold the pose; nudge:
   yaw CCW from above, hip up, knee unfolding), `apply_limits.py` (a `!!`:
   re-seat that horn one spline, recalibrate), `pose_check.py`,
   `PROTECT_CURRENT` ≈ 2 A (308 counts, unit VERIFY) on ids 1–3 and the spare,
   archived with a dump.
5. [§5b](../bench/BENCH_RUNBOOK.md#5b-cockpit-bring-up-sim--one-real-leg-d051d052):
   cockpit Hardware panel, scan, jog, real2sim (**one process per port**).
   Then [§6](../bench/BENCH_RUNBOOK.md#6-health-monitor-always-on) to
   [§8](../bench/BENCH_RUNBOOK.md#8-torque-step-calibrate-trust-in-the-torque-model):
   health monitor on, thermal soak and torque step on the hip (id 2), the base
   cup's −Y wall watched for flex (B49).
6. File what the bench settles (the table at the end of §8): `stall_nm_12v`,
   `continuous_frac`, `thermal_trip_frac`, `thermal_trip_s`, the masses,
   `latency_s` (echo test, no script yet), and the SCS0009 `sweep_deg` once a
   hand servo is on the bench. Then
   [§9](../bench/BENCH_RUNBOOK.md#9-end-of-day): dump with `--diff`; back up
   `bench/calibration.yaml` and `bench/out/`.

### 2.10 Swapping a servo

1. **Hip or knee:** plate B off (4 × A-18), clamps out (4 × A-10), plate A
   off, the cup's 4 rim screws; servo out toward +X, coupler moved over
   (4 × A-09). B22 keeps this order until swaps are frequent. Plate B's
   screws form their own thread in PETG: if it comes off more than twice,
   B26 moves its bosses to inserts.
2. **Yaw** (not written out in the repo): leg off (two rim screws are under
   the plate), yaw plugs out, 2 rim screws from above and 2 from below, servo
   and fork out toward +X, then the 4 hub screws.
3. The new servo gets its id alone on the bus (`--start-from <id>`, stop after
   one), parked at centre; recalibrate that joint alone (`--only
   leg<N>_<joint>`), then `apply_limits.py` and `pose_check.py`.

### 2.11 Four more legs

1. File everything the first leg measured (D032), regenerate, re-run
   `./rocky.sh cad-check`: VERIFY values are measured before printing 5×.
2. Print 4 × batch 2 without the blanks and `coxa_yaw_base` (batch 3 prints
   those with their knobs); cut the tubes as
   [3.1](#31-decide-the-tube-length-and-the-foot) decides.
3. IDs, one servo alone on the bus each time. Leg 1 is 4, 5, 6 and the bench
   spare already holds 5: one new servo `--start-from 4` (stop after one), the
   spare as leg 1's hip, the next `--start-from 6`. Legs 2–4 are 7–9, 10–12,
   13–15; one B-01 stays spare. Mark each leg's number: its ids tie it to one
   station ([4.2](#42-prepare-the-deck)).
4. Build each without the dry fit; calibrate it on the jig (`--only leg<N>`,
   `apply_limits.py`, `pose_check.py`) and set `PROTECT_CURRENT` on its ids,
   archived with `register_dump.py`.

### Check before moving on

- Servos in with finger pressure, no rock, 4 rim screws; the fork on the yaw
  horn under 0.3 mm wobble; plate A flat on both hubs, plate B on both idlers.
- Torque off, every joint through its range with no rub or tight lead; the
  tube fixed, the slit open; the drop passed continuity and polarity first.
- Jig pose squared; `pose_check` under 2°; no `!!`; `PROTECT_CURRENT`
  archived; real2sim follows; soak filed; torque step 0.8–1.3; no channel
  flex.
- First-leg numbers filed and the CAD regenerated before four more.

## 3. The tibia and foot

The spring foot cartridge (SEA, D010) on the tube's lower end, its switch and
wiring, and what goes on the I2 tool socket. The tube is already in the
carrier ([2.6](#26-tube-step-8)).

### You need

- Printed: `tibia_sea_outer`, `tibia_sea_slider` (PETG, batch 2); a
  `tube_clip` per tibia where tube is exposed; a tool coupon. Later:
  `foot_pad_tpu`, `hand_hub`, `hand_cam`, 3 × `hand_finger`.
- Per leg: A-16 spring (11 mm OD), A-17 KW10, B-15 26 AWG wire and
  heat-shrink, A-14 CA. Caliper a KW10 first: the pocket is 6.8 × 13.2 × 6.2
  and KW10 bodies ~12.8–13 × 5.8–6 × 6–6.5; KW11/KW12 (~20 mm) do not fit.
- Later: B-02 SCS0009 + B-11 UBEC (hand), X-07 2 mm rod, B-22 TPU.
- Tools: X-11, X-12, X-13 kitchen scale, X-14, the tube tools of
  [2](#2-the-leg).

### 3.1 Decide the tube length and the foot

**The cut length is not computed anywhere in the repo** (A-15; the tibia-stack
check of [ROBOT_AS_DATA](ROBOT_AS_DATA.md#5-cad-sync-rules) §5.2 is B36 step
8, not written). From the part modules (femur horizontal, knee −90°, slider at
its lowest):

| from | to | mm |
|---|---|---:|
| knee axis | tube top (carrier socket floor) | 16.7 |
| tube top | SEA outer top (11 mm of tube inside) | tube − 11 |
| outer top | stub face | 45.6 |
| stub face | closed hand's cone tip | 81.9 |
| cone tip | `foot_pad_tpu` sole | 3.0 |
| **knee axis** | **sole, with the hand** | **tube + 136.2** |

1. **With the hand as the foot, no tube gives `leg.l3_tibia` = 135.** The
   shortest tube, 29 mm (18 in the boss + 11 in the outer, none exposed),
   already gives 165.2 ([8.3](#83-the-foot)).
2. Decide before cutting five: a plain foot sized for 135 (B25, not modelled;
   at most 54.7 mm stub face to sole on a 29 mm tube), or a longer l3, a
   design change ([DESIGN_CHANGE_GUIDE
   §2](DESIGN_CHANGE_GUIDE.md#2-a-longer-tibia-or-other-link-lengths)). Also
   open: is 135 at spring rest or under standing load?
3. Not `HUB_H + CONE_LEN` (the cone starts at `KNUCKLE_Z` 27.0 and ends in a
   3.5 mm tip sphere: 12.5 mm short). A-15's ~120 mm matches the
   `leg_assembly.py` stand-in (118.3 mm: tube to the l3 point, no cartridge or
   foot): a stand-in, not a cut length, and too short to reach the knee
   gauge's V ([2.6](#26-tube-step-8)).
4. Cut one tube ([2.6](#26-tube-step-8)), build the foot on it, log the length
   and the measured knee-to-sole in `BUILD_LOG.md` ([DESIGN_CHANGE_GUIDE
   §6](DESIGN_CHANGE_GUIDE.md#6-the-first-real-leg)), then cut four. Before
   cutting the bench tube short, check what `calib_gauge_knee` grips (its V
   spans z −134 to −88; the l3 point in that pose is at z −74).

### 3.2 The SEA foot cartridge

![The SEA cartridge exploded: tube into tibia_sea_outer, the KW10 beside its pocket, the spring and tibia_sea_slider below](../cad/out/assembly/sea_exploded.png)

The outer is a Ø19 × 28 sleeve: tube socket (Ø10.3, 11 deep), a 1 mm floor,
then the Ø13.2 spring bore with two key slots, open at the bottom. A side
pocket (6.8 deep × 13.2 long × 6.2 high, open to the bore and the outside)
takes the KW10; a 3 × 4 mm slot runs up the bore wall for the striker. The
slider is a Ø12.9 flange (two keys, a 3.4 mm striker), a Ø8 stem and a Ø9.9 ×
14 stub with the I2 lugs. Design travel is 7 mm.

1. Push the outer onto the tube to the socket floor, dry: it has no clamp
   feature ("glue/clamp", no glue named). Glue it once the leg's length is
   settled (A-14's CA is the only glue in the BOM).
2. Turn the pocket to the side the leads run up, the same on all five.
3. Drop the spring into the bore from below, onto the floor.
4. Slide the slider in from below, flange first, keys in the slots, striker
   toward the pocket. Ease the striker past the 0.9 mm lip under the pocket;
   do not lever. That lip is then its only stop against the spring, and
   nothing checks that it holds ([8.3](#83-the-foot)).
5. Press the stub: the slider runs up without turning and returns.
6. Offer the KW10 into the pocket from outside, lever toward the bore. Meter
   on continuity, work the slider, find the click. If the striker fouls the
   body or misses the lever, stop and note it ([8.3](#83-the-foot)).

### 3.3 Wire the foot switch

1. Find the common and the contact that closes when the stub is pushed in;
   solder the blue leads from [2.7](#27-leg-harness) step 6 to them (pin 4's
   to one, pin 2's GND to the other) and heat-shrink. It closes to GND: no
   resistor, the Pi's pull-up does the rest.
2. Where tube is exposed, press the pair into a `tube_clip`'s tunnel (4 × 6,
   through its 2.6 mm slot) and snap the clip on (8 mm gap on the Ø10). A 29
   mm tube has none exposed.
3. Along the femur, tie it in with the data leads. Slack loops at knee and
   hip; servos limp, every joint through its range: nothing pulls.
4. At the coxa it runs with the drop to the leg-side XH-5; each pin 4 lands on
   its own J6 pin and GPIO ([star
   board](WIRING_HARNESS.md#star-board-bus-hub)).

### 3.4 Measure the switch force

`sensing.foot_switch` (close 2.0 N, open 1.0 N) is a guess; the sim's foot
contacts read it through `perception/contacts.py`. Leg upright over the
kitchen scale, meter across pins 4 and 2: press the stub slowly onto the scale
and note grams at the close, then easing off, at the open (N = g × 0.00981;
204 g is 2.0 N). File both for `close_n` / `open_n`.

### 3.5 The foot on the I2 tool socket

The stub is the male half of
[I2](INTERFACES.md#i2--tool-socket-sea-stub--hand--future-tools): Ø10, two
Ø2.5 × 2.2 lugs 180° apart, 9 mm from the stub face. The tools carry the
female socket (`part_tools.i2_socket_boss()`, the same cut as `hand_hub`'s
boss): L-slots, a 6 mm entry, a 90° twist, a 0.6 mm detent.

1. Lugs to the entry slots, push on 6 mm, twist 90° past the detent: the lug
   shows in its slot.
2. Load goes stub face to socket shoulder; the lugs only resist pull-off and
   twist (D020). The tools have that shoulder; the hand hub as modelled does
   not ([8.3](#83-the-foot)).
3. Prove it on the tool coupon: on free, quarter-turn, a tug holds.

**The plain stub foot (B25) is not modelled yet;** until it exists the bench
leg has no foot. **`foot_pad_tpu` is not a foot by itself:** it is a sock over
the closed hand's cone tip (TPU 95A, 0.2 mm, 2 walls, 25 % gyroid, 25 mm/s,
tip down on a brim; five plus spares).

### 3.6 The hand, later (B25)

The first I2 tool: `hand_hub`, `hand_cam`, 3 × `hand_finger`, v0.2.x geometry
tuned on the bench (fingertips wait for v0.3, B7, B15). **The SCS0009 gets the
6 V UBEC only (D016)**, measured at the plug first. Its lead is
[2.7](#27-leg-harness) step 5's, with a service loop for the 90° twist;
protocol 0, ids 16–20 one at a time, the §8 sweep on the first. Fingers hinge
on 2 mm pins (X-07, 15 for five hands) in 2.1 mm bores from outside the
knuckle collar (loose: glue; binding: ream with a 2 mm drill). The cam's Ø5
bore presses onto the horn (VERIFY). Claw range 0–55°, calibrated fully
closed.

### 3.7 Whiskers (optional)

`whisker_shoe` (B11), an I6 shoe with two Ø1.35 bores for piano wire (X-08),
waits for the shell, and its set-knob cannot lock it as drawn
([7.3](#73-accessories-i6)). Two KW10s switch it, each to GND on a GPIO.

### Check before moving on

- The tube home in boss and outer, no whitening or split; knee from −150 to
  −20 by hand touches nothing.
- The slider runs, returns, does not turn (travel with a tool measured); pins
  4 and 2 open free, closed pressed; forces filed.
- A tool coupon goes on, quarter-turns, holds a tug; the tube length is
  decided and the first cut logged in `BUILD_LOG.md`.

## 4. The body frame

The deck, the star-board bracket, the tray on the bench, and the stand.
Docking is [6](#6-dock-the-legs-and-power-up); the carapace
[7](#7-the-carapace).

### You need

- Printed (batch 3 and [Deferred](PRINT_PLAN.md#deferred)): `body_deck` once
  [4.1](#41-when-to-print-the-deck) allows it; four more `coxa_yaw_base`s with
  8 × `thumb_knob_m3`; `busboard_bracket`; `avionics_tray` as a bench carrier,
  after the standoff fix; 4 × `imu_grommet` (TPU); `stand_base`,
  `stand_crown`, 2 × `stand_section`.
- **Not for the robot yet:** `tray_rail`, `battery_sled`, `sled_rail`,
  `dock_block`, `belly_skid`; `belly_door` only as a bench demo.
- BOM: A-13 × 10; A-10 × 2 (bracket), 4 × A-09 or A-10 (board); B-20 steel
  washers; A-14 CA; B-21 M2.5; A-02, B-10, B-11, C-01; B-03/B-04 once
  bought (they wait until one leg walks, [5.4](#54-the-pis-operating-system)).

### 4.1 When to print the deck

Every interior decision cuts holes in `body_deck`: the tray rails, the
bracket, the power grommet and zip anchors, the bay's under-deck holes, the
strap slots. **Print it only once these are settled**, or it gets printed
twice (80 g, 4.2 h):

- the port coupon docks cleanly and the ladder is filed (B28);
- the deck's thickness is one number (B27);
- tray, rails, bracket and power entry are placed together (B51);
- the battery bay and the dock block's mounting exist ([8.4](#84-the-body)).

As drawn, the grid carries only the bracket (and optional ballast): no holes
for tray rails, sled rails or dock block, which `part_deck.py` cuts when B51
and the bay settle. **Do not drill a printed deck; regenerate it.**

### 4.2 Prepare the deck

190 × 181, flat, brim on, dry filament; the CAD cuts it 6 mm thick.

1. Brim off. Clear the five hook slots (4.2 × 24.6, through), five 11 × 11
   cable cutouts and 13 grid holes (Ø2.8, thread-forming).
2. The ten printed dowel posts (Ø3.95 × 8) are whole and square.
3. An insert (A-13) in each of the ten thumbscrew pockets, flush.
4. **Magnet washers:** glue a B-20 washer into each of the five Ø8 × 1.4
   recesses (station −25.5°, r 76), either face up; the shell feet's magnets
   land on them. **Zinc-plated steel, not stainless**, and not a 12 mm fender
   washer (it does not fit).
5. Learn the numbering; the bracket later covers the arrow. Station k has k +
   1 dots by its port; the triangle at (0, 30) points north, to station 0;
   stations run counter-clockwise from above:

| leg | station | dots | ids yaw / hip / knee |
|---|---|---|---|
| 0 | 90° (north) | 1 | 1 / 2 / 3 |
| 1 | 162° | 2 | 4 / 5 / 6 |
| 2 | 234° | 3 | 7 / 8 / 9 |
| 3 | 306° | 4 | 10 / 11 / 12 |
| 4 | 18° (378 in params) | 5 | 13 / 14 / 15 |

A leg goes to the station whose ids its servos carry; the bench kit's leg is
leg 0.

### 4.3 Under the deck: nothing yet

The battery bay (I5) is meant to be 175 × 50 × 30 under the deck, along x,
sled nose to +x. **It is not a part yet:** no floor, walls or door frame
([8.4](#84-the-body)). For this build:

- **Print no `battery_sled`, `sled_rail` or `dock_block` for the robot, and
  bolt nothing under the deck.** Only the five hook feet (3.3 mm) hang below
  it.
- The `sled_rail`s are floor rails: the sled rides on them, so they cannot
  hang from the deck.
- `dock_block` is the robot's half of I5, the bay's floating XT60 receiver
  with two spare taps, not part of the charging dock (B14); it waits for the
  bay and as drawn cannot be bolted down.
- `belly_door` has nothing to close on; `belly_skid` has no place until the
  bay and stand crown are redesigned.
- The robot runs tethered on the bench supply ([6.4](#64-the-pack)).

### 4.4 The star-board bracket

![The deck interior as modelled: the star-board bracket on grid holes (0, 20) and (0, 40), and the avionics tray on its rails at the assumed (0, −38), where they would run into the coxa bases at stations 2 and 3 (B51); the coxa bases and the battery sled are not drawn](../cad/out/assembly/body_interior.png)

1. `busboard_bracket` to grid holes (0, 20) and (0, 40): 2 × A-10 M3 × 8
   through its centreline, before the board (it covers them). It sits over the
   north strap run (B50).
2. The board ([5.5](#55-star-board)) on the four posts: 4 × M3 (A-09 × 6 or
   A-10 × 8) into the Ø2.8 bores, 8.5 deep, J1–J5 marked. It must clear the
   loom ring.

### 4.5 The avionics tray, on the bench (I4)

**As drawn, the tray and its rails fit nowhere between the legs:** at (0, −38)
they run into the coxa bases at stations 2 and 3, and no position or rotation
clears all five (B51). **Print no `tray_rail` and do not fit the tray to the
deck**; tray, rails, bracket and power entry get redesigned together before
the deck. Build the electronics on an `avionics_tray` printed only as a bench
carrier.

1. **Fix the Pi standoffs before printing it.** They are 58 × 44.1 because
   `part_avionics.py:44-45` scale `PI_HOLES` y by 0.9; the Pi 5 is 58 × 49
   (Ø2.7, 3.5 mm in from the edges), so each hole lands 2.45 mm off its bore.
   The fix is `hy` unscaled (standoff and bore); the x ±30 zip-tie slots must
   also move off the standoff feet.
2. **IMU: as drawn, do not mount the BNO085:** the pad's holes (16 × 12) miss
   the Adafruit 4754's (20.32 × 17.78) by 3.6 mm, and on its grommets the 4.6
   mm board stands 0.6 mm into the Pi. Once fixed, **the IMU goes on before
   the Pi:** SPI0, INT and RST soldered straight onto its pads (header pins
   would hit); the four `imu_grommet`s into the Ø4.8 holes (the lower flange
   stretches through); 4 × M2.5 from above, a nut under each (the Ø2.7 bore
   does not thread; ~3.1 mm under the tray, so a thin nut); then the Pi, which
   covers the pad.
3. Pi 5 + cooler, once bought: 4 × M2.5 × 6 (B-21), thread-forming into the
   Ø2.05 bores; not × 8 (the bore is 6 mm deep). The board overhangs ~10 mm
   at its USB end, clear of the rail block.
4. Adapter on its pad by the bulkhead: 2 × M3 into the thread-forming holes,
   20 mm apart (unchecked against the board: caliper).
5. Buck (B-10) and UBEC (B-11) have no modelled mount: zip-tie them to the
   tray's slots at x ±30 (moved by the standoff fix; 3.6 mm ties).
6. Rear bulkhead (70 × 26): XT30 in, XH-5 trunk, 2 × JST-SH, Qwiic, USB-C,
   every cutout VERIFY. The XT30 cutout (10.4 × 6.4) is for a panel XT30:
   check that an XT30U half seats before relying on it.
7. No `latch_housing` in the tongue: the I3 latch does not work as drawn and
   the tongue has no strike ([7.2](#72-the-sectors-and-the-cap)).

### 4.6 The bench stand

It holds the robot with the legs unloaded for `pose_check`,
`calibrate_centers` and the torque step (`part_stand.py`; the runbook writes
them for the one-leg jig). Print it now: it is safe while nothing hangs under
the deck.

1. Stack `stand_base`, the two `stand_section`s, `stand_crown`; each skirt
   drops over the five bars below when turned right (every 72°).
2. Deck-bottom height: 47 mm (base + crown), 127 (one section), 207 (two). A
   leg reaches 172 mm below the deck: two sections free every leg.
3. **The carapace comes off first:** the crown's arms run into the sector feet
   (591 mm³). **Nothing may hang under the deck on the stand:** the crown's
   top plate (apothem 47) is 3 mm below it and its arms bear on it from r 36
   to 86; only the hook feet are allowed for.
4. Lower the robot so the deck edge drops into the three saddles: leg 0
   between the arms at 54° and 126°, the third arm at 270°, between legs
   2 and 3. About 3 mm of radial slack is normal.

### Check before moving on

- `./rocky.sh cad-check part_port_coupon part_deck part_busboard part_avionics
  part_battery part_panel part_shell part_stand part_smallwins` exits 0 after
  any params change, before you print.
- The deck printed after 4.1 settled; inserts flush, washers below the top
  face, nothing under the deck; the tray's standoffs are 58 × 49.
- All three saddles carry the deck; on two sections no foot touches.

## 5. Wiring

The power tree, the tray, the star board and the deck-side looms, every drop
tested with nothing plugged in. The in-leg harness is [2.7](#27-leg-harness).

### You need

- Power: B-08, B-09 + 15 A fuses, B-10 buck + 1000 µF 16 V, B-11 UBEC (one
  spare), B-12, B-13, B-24 12 V node.
- Harness: B-14, B-15, B-16, B-18 star-board parts, A-06, B-23 cable ties, a
  USB data cable (not charge-only).
- Avionics: A-02, C-01, and B-03 Pi 5 + B-04 once bought
  ([5.4](#54-the-pis-operating-system)). None of C-02, C-04, D-01, D-02, X-01
  or X-03.
- Tools: A-04, X-12, X-14, X-15 (custom-length XH looms only).
- Done first: the bench leg passed the runbook; every servo labelled and in
  the leg of its ID.

### Rules for every step

- The [safety list](#safety) holds throughout.
- Colours: red 12 V, yellow 6 V, black GND, white data, blue sensors. Label
  every drop with its leg number at BOTH ends.
- Gauges (B-15): 14 AWG pack run, 20 AWG silicone per leg pair, 24–26 AWG for
  data, 6 V and sensors.

### 5.1 Power tree

As designed: pack → sled-nose XT60 → dock XT60 → 14 AWG up through the deck →
15 A fuse → XT60 loop key → 12 V node → five XT30 leg drops and the tray's
XT30 ([power tree](WIRING_HARNESS.md#power-tree)). Until the bay exists, the
chain starts at the bench supply's XT60 lead into the fuse
([2.8](#28-the-bench-legs-power-lead)).

1. With the dock (after the bay), its 14 AWG pair comes up through the Ø9
   grommet at (52, −6), zipped to anchors at (44, −14) and (60, −14); B51 may
   move them.
2. The 15 A fuse holder (B-09), then the loop key's chassis XT60E-M, in series
   in the feed. The key is an XT60H female with a short 12–14 AWG loop (B-08):
   arming plug and E-stop, reachable with the carapace on (nothing carries it
   yet). A DC-rated ≥ 20 A rocker is the alternative.
3. After the key, the 12 V node (B-24; no mount yet, B92): five XT30 pairs
   in 20 AWG silicone, one per port, and the tray's XT30. Cut the leg pairs by the
   [table](WIRING_HARNESS.md#cut-lengths-computed-from-the-deck-geometry)
   (115–294 mm) after checking each on the printed deck.
4. The dock's two XT30 spare taps sit ahead of the fuse (B66): **fuse anything
   plugged into them.** Nothing here needs them. Later options, X-01 (INA228,
   0x40) and X-03's relay contact, go after the fuse; no mounts yet (B47,
   B48).
   B-13's ten XT30 pairs: five drops, the tray power-in, two I5 taps, one
   bench lead for the jig leg; one spare.

### 5.2 Tray electronics

On the bench carrier ([4.5](#45-the-avionics-tray-on-the-bench-i4)).

1. USB data cable from the Pi (a laptop until the Pi is bought, 5.4) to the
   adapter's USB-C. The adapter is the bus master (`/dev/ttyACM0`); the ESP32
   driver (A-03) stays on the bench.
2. Buck and UBEC fed from the bulkhead XT30.
3. Buck output to the Pi header's 5 V and GND, the 1000 µF with it (side not
   written down, [8.5](#85-wiring)). This skips USB-C PD: set EEPROM
   `PSU_MAX_CURRENT=5000` (or `usb_max_current_enable=1` in `config.txt`), or
   the Pi will not know it has 5 A.
4. UBEC jumper to 6 V; its output goes to trunk pin 3 only, measured before
   the trunk is plugged in.
5. Trunk XH-5: pin 1 adapter data, 2 GND, 3 UBEC 6 V, 4–5 spare. **The
   adapter's servo V+ stays out of the trunk:** J0 has no 12 V pin.
6. The IMU on SPI0 once it can be mounted.

### 5.3 Pi 5 header (draft, VERIFY)

The [pin map](WIRING_HARNESS.md#pi-5-pin-map-draft-verify) is a plan (B43).
For this build: the BNO085 on SPI0 (GPIO 8 CE0, 9 MISO, 10 MOSI, 11 SCLK; INT
and RST on free GPIOs; protocol-select set for SPI, VERIFY), and the five foot
switches on free GPIOs with internal pull-ups, each closing to GND. Free: 4,
5, 6, 7, 12, 13, 16, 17, 20, 22–27 (GPIO 7 is SPI0 CE1, free while the IMU is
the only SPI device). Write your choices into the pin map; I²C, I²S, UART,
camera and lidar are optional.

### 5.4 The Pi's operating system

**The Pi's OS and microSD setup is not in the repo yet** ([8.5](#85-wiring));
this guide prescribes no image. What exists: ROS 2 Jazzy targets Ubuntu 24.04
([ros2/README.md](../ros2/README.md)); the 5 A setting (5.2); `dialout` and
`pip install -e ".[hw]"` ([runbook
§0](../bench/BENCH_RUNBOOK.md#0-before-anything-is-plugged-in),
[driver/README.md](../driver/README.md)). B-03/B-04 wait until one leg walks;
until then the adapter's host is a laptop.

### 5.5 Star board

Build it on the bench by the [star-board build
order](WIRING_HARNESS.md#star-board-build-order); it mounts once the deck and
bracket exist ([4.4](#44-the-star-board-bracket)).

1. Perfboard (2.54 mm) cut to 40 × 30, corners drilled Ø3.2 on 34 × 24. J0
   (XH-5 right-angle, trunk), J1–J5 (XH-5 vertical, leg numbers), J6 (XH-6,
   foot switches); lanes W1–W4 per [the board
   table](WIRING_HARNESS.md#the-board).
2. Buzz it: J1–J5 pin 1 to J0.1; pin 2 to J0.2 and J6 GND; pin 3 to J0.3; each
   pin 4 to its own J6 pin only; pins 5, J0.4, J0.5 to nothing.
3. J6 to the Pi header: five switch lines and GND.

### 5.6 Deck-side looms and power pairs

On the printed deck, bracket and board on.

1. XH-5 looms, board to each port, 108–287 mm by the table: check each on the
   deck, then crimp one end (with pre-crimped B-16 leads, the nearest length).
2. Dry-route looms and XT30 pairs: radial to the r ≈ 62 ring lane, round it
   the short way, radial to the cutout at r 81. Terminate the station ends at
   true length, zip the ring to the bracket's wings, label both ends, write
   the real lengths next to the table.
3. The coxa plate lies flat from r 64 outward and its channel starts only at
   the cutout's inboard edge (r 75.5): a lead on the deck top cannot reach the
   cutout under the plate. Decide on the first leg how it gets there
   ([8.5](#85-wiring)).
4. Trunk: bulkhead XH-5 to J0 round the tray; not computed (B67), so measure
   it once the tray has a place.

### 5.7 Test before any servo plugs in

No pack or supply, loop key out, every leg unplugged, trunk off J0, the Pi's 5
V lead off the header.

1. Continuity and polarity end to end: each XT30 red to red, black to black,
   no short; each XH-5 loom pin for pin.
2. No continuity from the node's + to any XH-5 pin.
3. Each pin 4 reaches its own J6 pin; every pin 5 reaches nothing.
4. Labels at both ends; each port's number matches its dots.

### Check before moving on

- Every drop checked and labelled; no path from 12 V to any XH-5 pin.
- Fuse and loop key after the feed; the node feeds five legs and the tray; the
  board buzzes clean; measured lengths are next to the table.

## 6. Dock the legs and power up

In steps, on the stand, from the bench supply: nothing plugged in, one leg,
all legs. The legs dock one at a time as the power-up asks.

### You need

- The deck on the stand, wired and tested, carapace off.
- The five calibrated legs, each with two knobs and M3 × 16 hex bolts.
- The tray electronics; A-04 with its XT60 lead to the fuse.
- B-05 pack, B-06 charger, B-07 alarm and LiPo bag.

### 6.1 Dock a leg (I1)

![Leg 0's coxa base docked on its I1 port, station 0 (north, one dot)](../cad/out/assembly/leg_on_deck.png)

The dowels and hook lip take shear and torque; the thumbscrews only clamp and,
with the hook, resist lifting
([I1](INTERFACES.md#i1--leg-port-deck--coxa-base-the-flagship)).

1. Loop key out.
2. Leg by its coxa, plate at ~15°: inboard lip through the station's hook slot
   (leg-local x −48), slide inboard until the lip's foot hooks under.
3. Hooked, not flat: plug the station's XT30 and XH-5 to the leg's drop at the
   cutout by hand; only a drop that passed
   [5.7](#57-test-before-any-servo-plugs-in).
4. Pivot flat onto the dowel posts at (−28, ±19); they drop in without force.
   If you push, stop and find what binds.
5. Then a knob with its M3 × 16 in each inboard hole at (−41, ±17),
   finger-tight: it takes 5.5 of the insert's 5.7 mm and stays inside the
   deck. No longer.
6. Push the coxa sideways, lift its end: the plate must not rock or rise.

To swap a leg: key out, knobs out, pivot up, unplug, unhook (target under two
minutes). **The thumbscrews are not captive:** unscrewed, knob and bolt lift
out. Keep each leg's pair with it, e.g. in a bag taped to it.

### 6.2 First power: bench supply, nothing plugged in

1. Supply 12.0 V, ~1.5 A, output off, checked; its XT60 lead into the fuse and
   loop key.
2. Output on, key in: 12.0 V, + on red, at the node, every port's XT30 and the
   tray's XT30.
3. The buck's 5 V measured before its lead goes onto the header; then the Pi,
   if bought yet, boots; its 5 A setting ([5.2](#52-tray-electronics)).
4. **6.0 V at the UBEC's plug**; only then the trunk into J0, and 6.0 V (pin 3
   to 2) on J1–J5 and every port's XH-5. Anything near 12 V: stop.
5. The adapter on its host (`ls /dev/ttyACM*`, `dialout`, the install); the
   Pi's own control loop is B35.

### 6.3 One leg, then all legs

1. Deck on the stand, carapace off, nothing under the deck, feet clear.
2. Key out, dock leg 0 ([6.1](#61-dock-a-leg-i1)), key in. From `bench/`:
   `python3 bus_scan.py --port /dev/ttyACM0` finds IDs 1, 2, 3 (16 with its
   hand) and nothing else, at ~12 V; nothing found: [runbook
   §1](../bench/BENCH_RUNBOOK.md#1-first-contact-one-st3215).
3. First move the soft way: the cockpit's Hardware panel or
   `robot.soft_enable()` ([driver](../driver/README.md)). The health monitor
   stays in the loop from here on.
4. Legs 1–4 one at a time: key out, dock, key in, scan. Three new IDs appear
   (hand 16 + *k*); IDs that do not match the port's dots mean the wrong
   station: move it before anything moves.
5. Hands (they can wait, B25): after the 6.0 V check, scan 16–20; each reads
   ~6 V (window 4.5–7.0 V).
6. A `register_dump.py` archive shows `PROTECT_CURRENT` ≈ 2 A on all 15
   ST3215s and the angle limits `apply_limits.py` burned.
7. Stay with scans and slow moves. The supply's 10 A ceiling is the walking
   peak (8–10 A, [power budget](WIRING_HARNESS.md#power-budget)); one walking
   leg draws ~2–3 A. No limit is given for five legs.

### 6.4 The pack

1. B-05: soft case, ≤ 138 × 44 × 25. **Hard-case packs (~37 mm) do not fit the
   30 mm bay**, nor most 50C soft 5200s (26.7–27 mm). Another size goes in
   `interfaces.battery_sled`, then `./rocky.sh cad-check part_battery` must
   print FITS (it counts sled + pack, not the rail under the sled).
2. Charge in the LiPo bag, never unattended; store at storage charge. The
   alarm (B-07) rides the balance lead whenever the robot runs: the 9.9 V
   floor is software, the alarm is not.
3. **The pack stays off the robot until the bay exists.** **No deck-strap
   fallback:** the bracket blocks the north strap run (B50) and the hook feet
   of legs 1 and 4 press into a 132 mm pack's ends. The sled never goes in on
   the stand. Later, with the bay
   ([I5](INTERFACES.md#i5--battery-sled--xt60-dock)): polarity checked at the
   sled nose and the dock before the first mate, made with the loop key out;
   to swap packs, limp the servos, swap the sled (~5 s), reboot the Pi.
4. Prove the E-stop on the bench supply: pull the loop key; everything goes
   dark, the Pi too.

### 6.5 Safety floors

The driver's floors come from `bus.safety` in `cad/params.yaml`
(`SafetyLimits`, `driver/rocky_driver/bus.py`), polled by `health_step()` at
1–5 Hz and by the cockpit's bridge every 25 Hz frame. 60 °C warns; 65 °C or
any fault bit cuts that servo's torque (the bridge limps the whole leg);
an ST3215 outside 9.9–12.9 V only warns; a hand outside 4.5–7.0 V warns (above
7 V it has probably met 12 V); 60 % load warns; 3 silent polls mark a servo
lost. The backstops: the balance-lead alarm (B-07, hardware; the 9.9 V floor
is software), `PROTECT_CURRENT` ≈ 2 A and the angle limits (limit + dir +
offset + 2°) in each servo's EEPROM, and the 15 A fuse. A 3S pack sags
0.3–0.5 V at 10 A, which the 9.9 V floor allows for; log every brownout in
`NOTES_INBOX.md`.

### Check before moving on

- 6.0 V at the UBEC plug and every port's pin 3 before any claw; the Pi, once
  fitted, runs from the buck with its 5 A setting.
- Every leg: dowels home, no light under the plate, knobs snug, no rock.
- IDs 1–15 (16–20 with hands) each on its labelled station; `PROTECT_CURRENT`
  archived on all 15; the monitor quiet at rest.
- The loop key cuts everything; the pack is charged, alarmed, off the robot.

## 7. The carapace

Last, after the power-up; it comes off whenever the robot goes on the stand
([4.6](#46-the-bench-stand)).

### You need

- 5 × `shell_sector` and `shell_cap`, printed last. No latch cartridges.
- B-19 N35 Ø6 × 3 magnets, one per sector foot (the hatch magnets wait,
  [7.1](#71-magnets)).
- Optional: `dovetail_shoe` or `whisker_shoe`; `trim_cup` + lid, B-20 washers,
  an A-19 M3 × 16.

### 7.1 Magnets

**Every magnet's NORTH face points away from the robot, the way the panel
lifts off, on panel and frame alike.** At each joint the frame magnet shows N
and the panel magnet S: two magnets stacked the same way up attract.

1. Mark N on every magnet before gluing (a compass's north end swings to a
   magnet's S face). Keep a reference pair (two loose magnets that snap
   together show the faces that must meet); pull-test each joint loose.
2. **Sector feet:** one in each foot's Ø6.25 × 3.2 pocket (az −25.5°, r 76,
   opening down). It meets a steel washer, so either face.
3. **Hatch: leave its magnets out** ([8.4](#84-the-body)). The rule would put
   seat and cap magnets both N up, but the seat "pocket" is a hole in a 0.2 mm
   ledge with no floor, and a cap magnet would stick 10 mm³ into the seat
   wall. Without them the cap sits by gravity.
4. **Demo parts:** `belly_door`'s and `shell_sector_demo`'s pockets go right
   through; glue those flush with the inner face.

### 7.2 The sectors and the cap

**The I3 latch as drawn does not work** ([8.4](#84-the-body)): its halves
cannot be joined, the rotor never reaches below the housing into the deck's
latch strike (which has no undercut), and with a sector down no coin reaches
the rotor. Glue no `latch_housing` anywhere. **The sectors are located by
their seam tongues and held only by each foot magnet on its washer:** lifting
one is unrestrained. **Do not carry the robot by the shell.**

1. Lower the first sector straight down over a docked leg, centred on its
   station; the foot magnet lands on its washer.
2. Add the others: each tongue (+36° edge) into the neighbour's groove (−36°
   edge), the last too. Any sector fits any station.
3. `shell_cap` on the hatch seat: no latch, no magnets; lift it by the thumb
   notch.
4. To reach a leg port, lift that sector straight up.

The loop key must stay reachable with the carapace on; nothing carries it yet.
Skip the charging dock (`dock_base`, `dock_tower`; B14).

### 7.3 Accessories (I6)

A `dovetail_shoe` or `whisker_shoe` slides down a vertical bar at ±27° (10
stations, 16 mm long). **Its set-knob cannot lock it:** the bore passes above
the male dovetail ([8.4](#84-the-body)). Drive no screw; the length is open
until the bore moves (a hex head, like the thumbscrews). The ~15 N rating is
for 24 mm: bench it on the 16 mm bar. Sensors, not handles.

### 7.4 Ballast (optional, B3)

`trim_cup` holds a stack of B-20 washers on any grid hole; one M3 × 16 through
lid, washers and cup floor (12.2 mm) leaves 3.8 mm in the deck. Note mass and
hole in `NOTES_INBOX.md`; the repo has no CoM target.

### Check before moving on

- Magnets marked and pull-tested before glue; N away from the robot.
- Every sector flat, magnet on washer, seams flush; the cap on its seat.
- Optional (B48): the Pi's Wi-Fi RSSI with the carapace on and off.

## 8. Open on the first build

One list, grouped by where it bites. **Blocks** marks a design gap that stops
a print or a step until the CAD changes; the rest are measurements, decisions
and checks. Ids refer to [DESIGN_BACKLOG.md](DESIGN_BACKLOG.md).

### 8.1 Prints

- **Fit ladder → params** (B28; **blocks** the deck).
- **PLA ladder, PETG parts:** `params.print` names no material; if a PETG part
  fits unlike its PLA coupon, note it with the material.
- **The tube socket has no key** (10 + `clearance_fit`): if row E disagrees
  with row A, decide which wins.
- **Counts the print plan misses:** 10 × `thumb_knob_m3` (it lists 2); two
  `stand_section`s (now in the estimate: 221 g / 8.1 h);
  latch sets once the latch works (5 carapace, 1 tongue, 2 door); one
  `tube_clip` per exposed tube and one `link_clip` per femur once it fits, not
  "3–4 per robot".
- **Printed-part strength** (B75): FEM allowables VERIFY; no pull coupon.

### 8.2 The leg

- **Coxa fork cannot go onto the yaw servo** (B80; **blocks**
  the leg print; [2.3](#23-coxa-steps-1-and-2)). Measured fix: open the idler
  pocket and its head relief out through the C's mouth, cut a 6.4 mm channel
  for the horn's centre head through the hub's rear lip, and add the path to
  `check_assembly` J1.
- **Yaw hub seat: the horn never sits in its pocket** (B81):
  the hub rides on the horn's centre head while the prose and `fem_check`
  assume the pocket; the CAD owner picks the intent, with the fork fix.
- **I1 thumbscrews: length, head and "captive"** (B82): the
  spec's M3 × 10 ends 0.5 mm short and its socket head spins in the hex
  pocket; the pocket is 1.5 mm deep for a 2.0 mm head; no "printed lips" are
  modelled.
- **The servo's numbers** (B23, VERIFY): horn thread and radius (if M2, A-12
  needs 60 + 60), rim screw size, plug height (then re-run `check_assembly`),
  envelope, mass, whether horn and idler ship fitted.
- **Blanks:** no rim pilots; the glued idler sits 0.3 mm proud of the real
  one, using up the idler pockets' 0.3 mm clearance, so a tight pocket on a
  blank may be this.
- **Coupler screw length:** under an M3 × 6 head, 1.9 mm of thread passes the
  horn against a 1.8 mm gap to the case (`out_boss_h`, VERIFY).
- **The gauges (as modelled):** `calib_gauge_hip`'s cradle is 6 mm high (it
  assumes leg z 0 at jig z 140; the proxy top is at 130, so leg z 0 is at
  134): ~4–6° of femur tilt, by where it stands (not specified).
  `calib_gauge_knee` seats a Ø10 tube at x ≈ 128.2 (the Ø19 sleeve ≈ 121.6),
  not under the knee axis at x 140.
- **Bench tests:** the adapter-to-jig data lead is not drawn and no current
  limit is given for a whole leg (1.5 A is first contact); `torque_step.py`
  predicts m·g·arm only, so decide whether §8 loads the built leg or a bare
  lever; watch the channel wall (B49).
- **In-leg harness:** threading order, lead construction, hip → knee route and
  lengths go into [In-leg routing](WIRING_HARNESS.md#in-leg-routing).
- **The hand's leg lead: length and mating half** (B91):
  B-17 is a 100 mm plug-to-plug cable against a ~0.45–0.55 m run, and two
  plugs do not mate ([2.7](#27-leg-harness)).
- **`link_clip` on the D062 femur** (B94): the bridge
  walls and couplers block plate A's edge; on the free span the inner jaw
  grazes the swinging knee carrier (2.5 mm gap, 2.7 mm jaw).

### 8.3 The foot

- **The hand-as-foot overshoots l3** (B95; **blocks**
  cutting five tubes): knee to sole is tube + 136.2 mm
  ([3.1](#31-decide-the-tube-length-and-the-foot)).
- **SEA travel with a tool on:** every tool's Ø16 socket mouth sits 3.0 mm
  below the outer at the slider's lowest: 3.0 mm of travel, not 7.
- **Slider assembly:** unchecked (`check_assembly` stops at J5); the striker
  must pass the 0.9 mm lip, then its only stop. Does it hold?
- **The switch:** not modelled; a 6 × 13 × 6 body centred in the pocket
  overlaps the resting slider by ~60 mm³ (the striker ~34, the flange the
  rest). Decide the lever side and the contact that closes on load, the same
  on all legs.
- **The spring** (B44): no free length, rate or preload in params; 9.6 mm of
  room against A-16's 25 mm (0.6 wire) and 20 mm (0.8) free lengths: measure
  the preload that leaves. A-16's two (~1 N and 3–4 N over 7 mm) are far
  softer than B44's ≥ 2 N/mm for a ~6.5 N stance (A-16 says ~9 N), so expect
  either on its stop when standing.
- **Outer to tube:** glue is the only retention; the pocket's direction is not
  set. Switch forces are VERIFY until [3.4](#34-measure-the-switch-force).
- **Lead route:** the tube is sealed in the CAD, so leads run outside in clips
  (`part_clips.py`), while WIRING_HARNESS says "down the tube" and INTERFACES
  I2 has the pigtail exit "the tube above the socket".
- **The plain foot** (B25): not modelled; needs the I2 female socket and must
  meet the l3 and travel items.
- **The hand's servo:** the SCS0009 envelope (VERIFY) overlaps `hand_hub` by
  ~416 mm³ and, with no bore-end shoulder, the stub's face bears on it:
  caliper one first. Also open: its lead, the pad with the claw open, the
  hinge pins (1.75 mm filament, X-07's zero-cost default, is likely loose in
  2.1; PRINT_PLAN says Ø2 filament).
- **Whiskers:** no switch mount; J6 carries only the five foot lines + GND.

### 8.4 The body

- **Deck thickness** (B27; **blocks** the deck): the CAD cuts 6 mm,
  `body.deck_t` says 4 (unread).
- **I4 tray position** (B51; **blocks** the deck): no pose of tray and rails
  clears the coxa bases ([4.5](#45-the-avionics-tray-on-the-bench-i4)), the
  rail holes miss the grid, and it slides out neither way. Measured lead: tabs
  inboard, tray at (0, 0), clear of every leg and on grid holes, but in the
  bracket's place and over the power grommet. Place tray, bracket and power
  entry together and check the tray against the leg ports (`part_avionics`
  counts a 50 mm half-span; the tabs reach 58.3). Trunk: B67.
- **Deck hole plan:** 13 grid holes against 27 wanted (bracket 2, tray rails
  6, sled rails 10, dock 2, skids 6, trim cup 1+), cut with B51 and the bay.
- **The battery bay is not a part** (B84; **blocks** the pack on
  the robot; [4.3](#43-under-the-deck-nothing-yet)). Also: floor to pack top
  30.25 mm against 30 (the B72 check counts sled + pack only); legs 1 and 4's
  hook feet 3.3 mm into it; sled + dock 194 mm against 175; south corners 10.6
  mm past the deck; `belly_door` fits no opening.
- **`dock_block` cannot be bolted down** (B86): its two holes
  sit under a closed roof, where no driver reaches and no nut fits, 16 mm
  apart on a 20 mm grid.
- **The stand crown fills the belly** (B85): its plate and arms
  occupy the bay, rails and skids ([4.6](#46-the-bench-stand)).
- **Sled strap slots take only ~5 mm straps** (B93): 6 × 5
  mm, and no BOM row straps the pack.
- **The dock's XT60:** it needs a female half, but its pocket is cut to the
  male XT60E-M (B-12 lists only male panel mounts); the pack's live side ends in
  exposed male pins at the sled nose. Its spare taps sit ahead of the fuse
  (B66).
- **Pi standoffs are 58 × 44.1, not 58 × 49** (B89;
  **blocks** printing the tray; [4.5](#45-the-avionics-tray-on-the-bench-i4)).
- **The IMU pad fits neither the BNO085 nor the space under the Pi**
  (B90; [4.5](#45-the-avionics-tray-on-the-bench-i4)):
  also 3.1 mm under the tray against 3.2 for flange + standard nut. The
  adapter pad is unchecked against its board; every bulkhead cutout is VERIFY.
- **The I3 latch cartridge cannot assemble or bite**
  (B87; **blocks** the carapace latch, tray tongue and
  belly door; [7.2](#72-the-sectors-and-the-cap)). For the redesign: only
  housing-bottom-flush seats (2 mm proud above the pad); `frame_coupon`'s
  second peg slot is misplaced.
- **Hatch magnets have no seat to sit in** (B88;
  [7.1](#71-magnets)): the robot's only magnet-to-magnet joint.
- **The I6 set-knob bore misses the dovetail** (B83;
  [7.3](#73-accessories-i6)): 2.0 mm above the male's crest (4.0 on the
  taller `whisker_shoe`).
- **Leg swap under the carapace:** knobs at r ≈ 69, arch from r 72. Does a leg
  come off with its sector on?
- **Not parts yet:** the loop key's mount (B-08), camera bracket (B16), lidar
  hatch cap (B12, B46), the charging dock's electrical side (B14, `PLUG_H` 52
  VERIFY); no CoM target for ballast.

### 8.5 Wiring

- **The harness is a design:** nothing wired yet; file every change in
  [WIRING_HARNESS.md](WIRING_HARNESS.md).
- **The 12 V power node** (B92): B-24 is a placeholder, no part picked, no mount (nor
  for the dock's bus-bar cavity); also which conductor the fuse and key break,
  and the tray feed's gauge.
- **Deck-side leads to the cutout:** nothing routes the r ≈ 62 lane under the
  coxa plate ([5.6](#56-deck-side-looms-and-power-pairs)).
- **Buck capacitor:** which side the 1000 µF (16 V) goes; on the input it has
  thin margin against 12.6 V.
- **Adapter power on the robot:** its servo-power input (9–12.6 V) gets no
  feed; bench whether it drives the bus on USB alone. The adapter-to-trunk
  lead is unspecified.
- **Across the tray boundary:** the J6 lines, camera ribbon and lidar USB have
  no bulkhead connector; the two JST-SH and the USB-C slot are unassigned.
- **Lengths and limits:** looms, power pairs, trunk (B67), tray feed; no
  bench-supply current limit for five legs.
- **Pi pin map** (B43): INT, RST, switch and whisker GPIOs, protocol-select,
  IMU and amplifier supply pins.
- **Pi 5 bring-up doc** (B96): nothing takes the Pi from a
  blank microSD to the driver ([5.4](#54-the-pis-operating-system)).
- **Also open:** `PROTECT_CURRENT`'s 6.5 mA/count unit (VERIFY; no script
  writes it); where the balance-lead alarm rides with the pack in the bay,
  and whether it is heard through the shell; nothing on the Pi reads the IMU
  or foot switches yet (B35).

### 8.6 Wording left to fix

The docs, BOM and CAD comments were corrected with this guide (2026-09-28).
Two items remain: PRINT_PLAN gives `tibia_knee_carrier` as "cup floor DOWN"
while `check_printability.py` audits it as exported, so check the pose before
slicing; and the dated [REVIEW_2026-09-22.md](REVIEW_2026-09-22.md) still
reads the horn slots as BCD 14.0–15.6 (they cover 14.0–17.2).
