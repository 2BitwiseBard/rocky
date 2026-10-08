# Assembly guide

From the printed parts and the [bill of materials](../bom/BOM.csv) to a robot
on its bench stand, every servo calibrated, powered from the bench supply and
ready for its first steps. Nothing has been built from this guide yet, so
every VERIFY in it is real, and so is every item in [8. Open on the first
build](#8-open-on-the-first-build). D064 (2026-10-07) put the owner's body
layout into the CAD: the leg port docks on two cone seats, the battery rides
in a keel tub under the deck, the hub boards sit on a shelf north of it, and
the stand holds the robot by the tub. One part is on hold: `coxa_yaw_base`
fails the FEM at servo stall (SF 1.10), so print one for the bench leg and
wait for the fix before the other four ([PRINT_PLAN.md](PRINT_PLAN.md)).

This guide is the order of work; the detail lives in the documents it links:
[PRINT_PLAN.md](PRINT_PLAN.md) (what to print, poses, supports, go / no-go),
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
The tibia and foot](#3-the-tibia-and-foot) · [4. The body](#4-the-body) · [5.
Wiring](#5-wiring) · [6. Dock the legs and power
up](#6-dock-the-legs-and-power-up) · [7. The carapace](#7-the-carapace) · [8.
Open on the first build](#8-open-on-the-first-build)

## The build at a glance

1. **Printer and port first:** the fit ladder and the port coupons filed, the
   CAD regenerated ([1.1](#11-batch-0-the-printer-and-the-port)).
2. **Coupons, then leg parts from the current tree only**
   ([1.3](#13-batch-1-coupons-and-the-blank)); one `coxa_yaw_base`, not five.
3. **One leg dry-fitted on the blanks before any servo arrives**, with the
   harness threading tried ([2.2](#22-dry-fit-on-blanks)).
4. **Servos on the desk:** checked, numbered, smoke-moved
   ([1.5](#15-the-servos-arrive-on-the-desk)).
5. **The bench leg:** a long bench tube and its bench power lead (fuse,
   loop key, one XT30 drop, splitter), then calibrated and load-tested on the
   jig ([2](#2-the-leg)).
6. **Tube length and foot**, decided before five tubes are cut
   ([3](#3-the-tibia-and-foot)).
7. **Four more legs** ([2.11](#211-four-more-legs)); their bases once the
   FEM hold lifts.
8. **The body:** the deck, the keel tub with its door and sled, the hub
   shelf, the tray, then the stand ([4](#4-the-body)).
9. **Wiring before docking**, every drop tested unplugged ([5](#5-wiring)).
10. **Power-up on the stand, tethered:** legs docked one at a time, loop key
    out while plugging ([6](#6-dock-the-legs-and-power-up)).
11. **The pack in its sled** ([6.4](#64-the-pack)); **the carapace last**
    ([7](#7-the-carapace)).

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

- **Printed, PLA:** batch 0, `fit_ladder` and the port coupons
  (`port_coupon_deck`, the four plates `port_coupon_plate_relief46`,
  `_relief42`, `_fit10`, `_fit20`, and 2 × `thumb_knob_m3`); batch 1,
  `servo_blank`, `blank_idler`, `coupon_cup`, `coupon_yaw_hub`,
  `coupon_hip_hub`, `coupon_idler`, `horn_coupler`; the [interface
  coupons](PRINT_PLAN.md#interface-coupons-pla-02-mm-with-batch-1) (the
  latch, dovetail and panel pairs, `tool_hook`, `tool_scoop`). Print from the
  current `cad/out/`; pre-D047 leg parts do not fit the servo.
- **Servo bring-up:** A-01 4 × ST3215 12 V, A-02 adapter, A-04 bench supply
  (or A-05: fuse its lead, watch the wattmeter), A-07 barrel pigtails; A-03
  optional (12.0 V from the bench supply, never a full 3S pack).
- **Hardware and tools:** A-08, A-09 M3 × 6, A-10 M3 × 8, A-12 (only for an M2
  horn), A-13 inserts, A-19 M3 × 16 hex-head bolts (ISO 4017 / DIN 933,
  5.5 mm A/F: the thumbscrews and the I6 set knobs), A-14 threadlocker + CA;
  X-11 calipers, X-12 multimeter, X-13 scales, X-14 iron with an insert tip,
  a Ø2 pin, X-19 dryer for PETG. For the port coupons, not in the BOM: a
  0.2 mm feeler, a dial indicator, and a scale that reads a 300 N pull
  (about 31 kg).
- **Not in the BOM yet:** a mating JST-SH 3-pin header for the hand's side of
  the I2 disconnect (B91).
- **For the I3 latch:** a flat screwdriver, blade ≤ 5 mm (not a coin).

### 1.1 Batch 0: the printer and the port

Every clearance in `params.print` is a generic FDM guess until your ladder is
filed (B28), and the leg port docks on its seats only in CAD so far
(decision 15, B117). Both answers move the fits of mating parts, so nothing
else prints until they are filed.

1. Set `print.bed_mm` to your printer; the bed-fit checks assert it.
2. Print one `fit_ladder` (PLA, 0.2 mm, 3 walls, 25 %, sliced like the parts,
   elephant-foot compensation ~0.15 mm) and the port coupons in the poses of
   [PRINT_PLAN's batch 0](PRINT_PLAN.md#the-port-coupons): the plates L-down,
   on support, with a support blocker in both seat sockets.
3. Measure the ladder into `NOTES_INBOX.md`:

   | row | find | params key it sets |
   |---|---|---|
   | A | first Ø4 bore the 3.95 peg slides into | `print.clearance_fit` (4.30 bore = 0.30) |
   | B | first hole an M3 (A-09) threads cleanly | `print.screw_m3_tap` (2.8) |
   | C | pocket that holds an insert (A-13) snugly | `print.heatset_m3_d` (4.6) |
   | D | the slot (5.8 / 6.2 / 6.6) a coupon plate's L passes at 7–10°, brought in radially | `interfaces.leg_port.hook_slot_w` (6.2) |
   | E | snug Ø2 pin hole; which Ø10 socket the tube seats in | pins: `part_hand.py` (2.1, no key); tube = 10 + `clearance_fit` |

   There is no seat row: the coupon plates test the seat fit. Row E's 6.85 /
   6.95 / 7.05 pockets are the obsolete bearing seat. A ladder from before
   2026-10-07 answers A, B, C and E; its row D (3.8 / 4.2 / 4.6) passes no L.
4. **The port coupons**, the rehearsal the deck and the coxa bases wait for:
   1. 2 inserts (A-13) in `port_coupon_deck`, flush ([1.2](#12-heat-set-inserts)).
   2. The thumbscrews: press an A-19 M3 × 16 hex-head bolt (not A-11's socket
      head) into each `thumb_knob_m3`'s hex pocket from the top, thread out of
      the bottom. Snug on purpose: the head must not turn in the knob. The
      whole head sits in the 2.1 mm pocket, 0.1 mm below the knob's top (B82);
      a round socket head spins in it.
   3. Clear the plates' support. The sockets must come out clean: smooth cone
      flanks, a bridged Ø2.5 roof, no support inside. Scrape only the inboard
      pads; the 0.2 mm relief outboard of x −46 is drawn so that face does
      not bear.
   4. `port_coupon_plate_relief46` first: hold it at 7–10°, outboard end up,
      bring the L in radially and pass it through the 6.2 slot by hand,
      without force. Lower it onto the two cones: it must centre itself.
   5. A knob in each hole, finger-tight: the M3 × 16 takes 5.5 of the
      insert's 5.7 mm. **Not longer than 16:** on the bench a longer screw
      hits the table before it clamps. Nothing rocks, there is no x or y
      play (< 0.05 on a dial), and a 0.2 feeler stays out from under the pads.
   6. Dock `relief42` the same way; note which of the two sits without rock.
      If relief46 rocks or shows light under its pads, dock `fit10`, then
      `fit20`: the smallest that sits flat is `print.seat_fit`.
   7. The rest of the go / no-go: each insert's pull-out (≥ 300 N toward the
      plate), the knob preload (≥ 285 N), the cone posts, the release test
      and 20 dock / undock cycles ([PRINT_PLAN](PRINT_PLAN.md#go--no-go)).
      The knobs go on to the bench leg.
5. `clearance_fit` sets every mating printed fit (servo cups, the Ø20.3
   idler pockets, tube sockets), `screw_m3_tap` every thread-forming hole,
   `heatset_m3_d` every insert pocket, `seat_fit` the seat sockets.
6. File the numbers in `cad/params.yaml` (the purpose in the comment: slide,
   press, tap), then run `./rocky.sh cad-check` and `./rocky.sh cad-check
   --derived` ([the
   loop](DESIGN_CHANGE_GUIDE.md#0-the-loop-every-change-goes-through)).
   Printing before you regenerate is allowed for bores only (the [GO /
   NO-GO table](PRINT_PLAN.md#go--no-go)). A slot that binds at 6.2 means
   `hook_slot_w` 6.6 before the deck; seats that rock at every offset mean
   the "minimal" port, an owner call.

### 1.2 Heat-set inserts

Every production pocket is `heatset_m3_d` × `heatset_m3_h` (4.6 × 5.7); the
ladder's row C has fixed 4.4 / 4.6 / 4.8 holes.

| part | inserts | where | when |
|---|---:|---|---|
| `fit_ladder` | 3 | row C, one per hole | now |
| `port_coupon_deck` | 2 | its I1 port | batch 0 |
| `jig_base`, `jig_column` | 6 + 2 | under the column flange; the column's I1 port | [2.9](#29-calibrate-the-first-leg-on-the-jig) |
| `body_deck` | 10 | 2 per I1 port, leg-local (−41, ±17) | [4.2](#42-prepare-the-deck) |
| `bay_lid` | 8 | the six hanger bosses and the two pilaster bosses | [4.3](#43-hang-the-tub) |

1. Insert tip on the iron (X-14); your first inserts go in row C.
2. Stand the insert on the pocket, press straight down lightly, let the heat
   work. Stop flush or just below: **a proud insert on the deck holds the coxa
   plate off it**, and the plate's inboard pads on the deck take the load.
3. Keep it square: an M3 runs in by hand through the plate's Ø3.7 bore.

### 1.3 Batch 1: coupons and the blank

After batch 0 is filed. Each coupon is a boolean clip of its production
part (`cad/part_leg_coupons.py`): a coupon that fits proves that part.

1. Print batch 1 in the poses of its table in [PRINT_PLAN.md](PRINT_PLAN.md);
   none needs support.
2. Glue the `blank_idler` into the blank's Ø6.2 pocket with CA (A-14), disc
   flat on the bottom face; the dry-fit blanks later get the same.
3. `coupon_cup` (case fit, rim clearance, screw positions of all three cups):
   the blank slides in rear-first with finger pressure and does not rock; a Ø2
   pin through each rim hole lines up.
4. `coupon_yaw_hub` on the blank's horn: the horn face flat on the hub top,
   the centre head in its Ø6.4 relief (D063: no pocket, B81), no rock; M3 × 6
   screws pass at both ends of all four slots, and snug they hold it (the
   blank's Ø2.5 pilots take M3 × 6 or M2 × 6, dry fits only). The relief runs
   out to one side as a 6.4 mm channel 0.05 mm from the two rear slots: over
   its 1.3 mm depth they print as one opening, which is harmless.
5. `coupon_hip_hub` + `horn_coupler`: the coupler drops into the recess and 2
   × A-10 from the outer face draw it flat; a 1 mm sideways push meets the
   lobes.
6. `coupon_idler` drops over the glued idler without rocking; the plug notch
   is clear.
7. **The interface coupons** gate nothing, but they are the first
   prints of the D063 latch and shoe (B87, B83):
   1. **Latch cartridge (I3):** the rotor into `latch_housing`, lugs through
      the two keyways, then a quarter turn off them: it is captive. Push the
      cartridge into `shell_sector_demo`'s pocket from the lip side, head
      first, the housing's flat to the pocket's flat (it goes in only that
      way: the keyway index, B107), housing bottom flush with that face;
      A-14 CA.
   2. **The panel pair:** `shell_sector_demo` + `frame_coupon` mate since
      D063. Magnets per [7.1](#71-magnets) (the coupon's show N, the demo's
      S). Rotor at OPEN, its lugs over the coupon's entry slots (the slot
      across the rotor's end is square to the lugs); lay the demo on,
      lip in the groove (free at ±0.25 mm, located at ±0.6). From below, a
      flat screwdriver in the rotor's slot, a quarter turn clockwise to the
      stop: it should start to bite at about 40° and stop at 90°; lifted, the
      demo holds. Back to OPEN, it lifts off.
   3. **The cam bite is VERIFY** (B99): 0.2 mm into PLA lugs (a hand calc puts
      ~108 MPa at their roots, PLA yields ~50), and the cam ceiling prints
      as a bridge, so the 0.8 mm ramp may come out in steps. Expect the first
      turn to set the lugs; note how it feels and whether it still holds
      after ten turns.
   4. **Dovetail (I6):** an A-19 in a `thumb_knob_m3` threaded into
      `dovetail_shoe`'s angled bore, backed out until its tip clears the
      slot; the shoe slides down the 24 mm male, then the knob in until the
      screw wedges it ([7.3](#73-accessories-i6)). Lifted, the shoe holds.
   5. The tool coupons need a printed `tibia_sea_slider`
      ([3.5](#35-the-foot-on-the-i2-tool-socket)).
8. Every verdict goes in `NOTES_INBOX.md`.

The blanks are the dry-fit servos and stay as bench dummies. They have no
connector housing, so they cannot test the plug keep-outs; three engraved dots
on each side mark one.

### 1.4 Clean up the leg parts

For batch 2 as it comes off the printer: PETG, dry, supports only where its
PRINT_PLAN table says, sliced in the poses `check_printability.py` audits.

1. `coxa_yaw_base` (the one for the bench): all support out from under the
   plate; first-layer flare off the L's foot; the two seat sockets clean (they
   printed over a blocker, nothing inside). Scrape only the inboard pads
   (x ≤ −46): the 0.2 mm relief outboard of them is meant not to bear. The
   11 × 11 harness channel clear end to end (its first stretch is roofed over
   an 11 mm bridge, plate on support) and the Ø3.7 thumbscrew bores clear.
2. `coxa_fork` has support in three places: under the upper plate where it
   leaves the web and the side cheeks (z ~41; since D063 the audit's widest
   cantilever, 11.5 mm), under the hip cup's lower wall where it steps out
   past the plate (z ~44), and under the hip cup's upper side wall over the
   hip servo (the 21 mm step at z 72.5). Clear all three, clean the
   servo-side faces of both hip-cup walls and both cheeks, the flat hub top,
   the horn-head relief and its 6.4 mm channel, the four slots and their
   counterbores, and the idler pocket with its channel out to the mouth.
   The two rear slots print joined to the horn channel (0.05 mm apart):
   expected.
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
   ([§2](../bench/BENCH_RUNBOOK.md#2-register-archive-as-shipped)) before
   step 6 changes it. Since D063 the dump reads STS 0–87 (SCS 0–83) and
   prints one line per servo: firmware, return delay, Lock, ACC, 85, 86.
   File that line: it is the bench half of B79 (B32).
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
- Batch 0 filed: the port coupon docks on its cones without force, the seat
  fit is chosen and both thumbscrews run in; batch 1 passes and is filed.
- Blanks slide into every cup without rock; the base's channel is clear, its
  pads flat; inserts flush and square.
- Each servo: 12 V label, metal horn, idler, a unique labelled ID, moved ±15°,
  left torque-off; `horn_screw_m` and `horn_bcd` measured.

## 2. The leg

Coxa, femur and knee: one leg built, hung on the bench jig, calibrated and
load-tested before four more are printed.

### You need

**Printed, per leg** (PETG; batch 2 in [PRINT_PLAN.md](PRINT_PLAN.md)):
`coxa_yaw_base`, `coxa_fork` (the D063 one, with side cheeks), 2 ×
`horn_coupler`, `femur_link` (plate A), `femur_plate_b`, `tibia_knee_carrier`,
2 × `thumb_knob_m3` (PLA in the print pack; PETG is reasonable, the knob is in
the clamp load path). The bench leg uses the coupon knobs; batch 3 prints the
other eight. **One `coxa_yaw_base` only** until its FEM hold lifts (SF 1.10 at
servo stall, B119, B139): it carries the bench leg, not a walking robot.

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
| B-13 | XT30U pair | the leg drop | 1 | 5 | 10 ([power tree](WIRING_HARNESS.md#power-tree)) |
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
   slides on over both ([2.3](#23-coxa-steps-1-and-2)), and plate B rides the
   idlers' Ø19.9 rims in blind pockets.
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

1. Steps 1–8 below on three blanks with glued idlers. Screws self-tap into
   the blank horn's pilots, no threadlocker. The blank has no rim pilots:
   decide whether to drill 1.5 mm pilots through the cup holes, and note it.
2. With the blanks in, try the harness threading ([2.7](#27-leg-harness)): the
   leg-side XT30 and XH-5 through the coxa channel (under the gearbox plateau
   that step 2 covers) and the pin 3 and 4 leads at full length.
3. Check the batch 1 criteria on the whole chain, take it apart in the swap
   order ([2.10](#210-swapping-a-servo)), keep the blanks.

### 2.3 Coxa: steps 1 and 2

1. **Fork onto the yaw servo, off the base.** Yaw servo at centre, horn and
   idler fitted. Slide the fork on from the servo's front, its open (−X) side
   leading: the servo passes between the two side cheeks, the horn face runs
   along the flat hub top, and the horn's centre head and the idler run in
   their channels until the idler stops at the end of its pocket. Notch
   toward the plugs. `check_assembly` checks this path on the blank and the
   real servo (0.00 mm³ all the way, D063): if it binds, stop and look; do
   not force it or sand.

   Until the screws are in, the fork can slide back out the way it came and
   drop 0.3 mm off the horn face. **Hold it home, hub flat on the horn
   face,** while you drive 4 × A-09 up through the hub's slots, heads in the
   counterbores, 243 on each tip in the aluminium horn, **none on the printed
   part.** Snug them evenly (overtightening dishes the hub): the horn face
   bears on the hub top, the centre head sits clear in its Ø6.4 relief, and
   the slotted screws locate the hub.
   ![Step 1: the fork slides onto the yaw servo between its side cheeks](../cad/out/assembly/leg_01.png)
2. **Servo and fork into the base.** Together into `coxa_yaw_base`'s cup from
   outboard (+X), servo rear first; the hub and the fork's cheeks pass
   1.3 mm over the plate, the cheeks ~2.9 mm beside the cup. The
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
   −29) with the plate hooked, before it goes down onto the cones ([per-leg
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
   Along the femur the leads ride a `link_clip` (D063, B94): press it down
   over plate A's top edge above the lightening slot, between the bridge
   walls and hub B (link x 60–68 with the femur horizontal), the thin
   (1.2 mm) jaw on the servo side, until its nubs snap under the slot's top
   edge. The leads go into its 4 × 6 tunnel through the 2.6 mm slot. It
   clears the swinging knee carrier by 1.0 mm (0.7 with its play), and it
   holds lightly: a cable clip, not a handle.
9. Before any servo is plugged in: continuity and polarity on the drop, the
   leg number on both ends, and 6.0 V (never 12 V) on pin 3 once a UBEC feeds
   it.

### 2.8 The bench leg's power lead

There is no tub on the bench yet, so the bench supply feeds the robot's
fuse and loop key on a loose XT60 lead, with one XT30 drop to the jig
([WIRING_HARNESS](WIRING_HARNESS.md#build-order-phase-b), step 1). Build it
before calibrating.

1. This leg's splitter (B-14, [2.7](#27-leg-harness) step 4), and the robot's
   15 A fuse (B-09) and loop key (B-08), built on the bench by the [power
   tree](WIRING_HARNESS.md#power-tree), with one XT30 pair (B-13) after the
   key to the jig's cutout, fed from A-04 on a loose XT60 lead (B-12 XT60H pair).
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
2. **Hang the leg, loop key out:** hold it at 7–10°, outboard end up, bring
   the L in radially through the jig's 6.2 slot and hook it; mate the XT30
   and XH-5 at the column's cutout while hooked; lower it onto the two cones,
   where it centres itself. Then a knob with its M3 × 16 in each inboard hole,
   finger-tight. The seats take shear and torque; the thumbscrews hold the
   plate down ([I1](INTERFACES.md#i1--leg-port-deck--coxa-base-the-flagship)).
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
   [§8b](../bench/BENCH_RUNBOOK.md#8b-goal-speed-sag-per-tick-speed-under-load-d063):
   health monitor on, thermal soak, torque step and the goal-speed sag test
   on the hip (id 2), the base cup's −Y wall watched for flex (B49).
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
   and fork out toward +X, then the 4 hub screws; the fork slides off the
   servo the way it went on ([2.3](#23-coxa-steps-1-and-2)).
3. The new servo gets its id alone on the bus (`--start-from <id>`, stop after
   one), parked at centre; recalibrate that joint alone (`--only
   leg<N>_<joint>`), then `apply_limits.py` and `pose_check.py`.

### 2.11 Four more legs

1. File everything the first leg measured (D032), regenerate, re-run
   `./rocky.sh cad-check`: VERIFY values are measured before printing 5×.
2. Print 4 × batch 2 without the blanks and without `coxa_yaw_base`: the
   four bases and their knobs print with batch 3 once the FEM hold lifts
   ([PRINT_PLAN.md](PRINT_PLAN.md)). Cut the tubes as
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

- Servos in with finger pressure, no rock, 4 rim screws; the fork flat on the
  yaw horn, no rock with its 4 screws snug; plate A flat on both hubs, plate B
  on both idlers.
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

**The cut length is not computed anywhere in the repo** (A-15; the
tibia-stack check, B36 step 8, is not written). From the part modules (femur
horizontal, knee −90°, slider at its lowest):

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
waits for the shell; its set-knob locks it since D063 and clears the wires
([7.3](#73-accessories-i6)). Two KW10s switch it, each to GND on a GPIO.

### Check before moving on

- The tube home in boss and outer, no whitening or split; knee from −150 to
  −20 by hand touches nothing.
- The slider runs, returns, does not turn (travel with a tool measured); pins
  4 and 2 open free, closed pressed; forces filed.
- A tool coupon goes on, quarter-turns, holds a tug; the tube length is
  decided and the first cut logged in `BUILD_LOG.md`.

## 4. The body

The deck, the keel tub with its door and the battery sled, the hub shelf, the
tray and the stand: option A, the owner's picks of 2026-10-07. Docking is
[6](#6-dock-the-legs-and-power-up); the carapace [7](#7-the-carapace).

![The body from below (option A), the robot turned over: the keel tub east-west (door −x, nose +x) and the hub shelf with its boards north of it hang from the deck; the tray on top shows through as an x-ray tint](../cad/out/assembly/body_interior.png)

### You need

- **Printed** (batch 3 and the stand in [PRINT_PLAN.md](PRINT_PLAN.md)):
  `body_deck`; `bay_tub`, `bay_lid`, `bay_door`, `battery_sled`; `hub_shelf`;
  `avionics_tray` + `tray_latch_boss` + 2 × `tray_rail`; 2 latch cartridges
  (the tray's and the door's); 4 × `imu_grommet` (TPU); `stand_base`,
  `stand_crown`, 2 × `stand_section`. Not yet: the four more
  `coxa_yaw_base`s and their 8 knobs (the FEM hold).
- **Screws and inserts** ([`bom/BOM.csv`](../bom/BOM.csv)): A-13 × 10 (deck)
  + 8 (lid); B-27: six M3 × 16 tub hangers, four of them ISO 7380 button heads
  through the rail tabs, and two M3 × 10 button heads thread-forming into the
  deck for the rails' north tabs; two
  M3 × 10 for the tub's pilasters; B-26 three M3 × 30 socket heads for the shelf
  posts; 8 × M3 × 6 (star board, PDB); 4 × M2.5 × 8 pan heads and B-28 four Ø5 ×
  2.5 spacers (bus adapter); B-21 M2.5 (Pi, IMU); B-23 3.6 mm ties; B-20
  steel washers; A-14 CA.
- **Bought parts:** B-25 the FEMALE XT60 for the tub's carrier (B-12 lists male
  panel mounts only), B-12's XT60E-M for the sled nose and for the loop key's
  pocket, B-08 the loop key, B-09 the fuse holder, B-29 the 2 mm foam pad
  (138 × 44), B-24 the 30.5 × 30.5 PDB, B-18 the star board, A-02, B-10, B-11,
  C-01, and B-03/B-04 once one leg walks.
- **Tools:** the iron with an insert tip; X-20 a flat screwdriver, blade ≤ 5 mm,
  with a shaft of at least 183 mm (200 recommended) for the tub door's latch.

### 4.1 When to print the deck

Once batch 0 is filed ([1.1](#11-batch-0-the-printer-and-the-port)): the
slot width and the seat fit cut its five ports, `clearance_fit` and
`screw_m3_tap` its holes. The body layout is settled (D064), so nothing else
waits. **Do not drill a printed deck; regenerate it** (80 g, 4.3 h a print).
The CAD cuts it 6 mm thick and v0.5's blind post holes and strikes depend on
that; params `body.deck_t` still says 4, unread (B27).

### 4.2 Prepare the deck

190 × 181, flat, brim on, dry filament; 6 mm thick, 9.0 with its cones.

1. Brim off. Clear the five hook slots (6.2 × 24.6, through) and the five
   11 × 11 cable cutouts.
2. **The ten cone seats**, two per port at leg-local (−32, ±17.3), 30°,
   3 mm tall: whole, with clean flanks and no first-layer flare round their
   bases. Do not sand them: they place the leg.
3. **The 21 holes** of the v0.5 table (`part_deck.DECK_HOLES`): four Ø3.4 tab
   and hanger holes at (±40, −20 / 0), the rails' two north tab holes at
   (±40, 20), Ø2.8 for a thread-forming screw, two tub hangers at (±58, −18),
   the (0, 50) trunk slot (12.5 × 7.5) and its two zip anchors at (±12, 50),
   the tray's latch strike at (0, −43) with its slot running south, six
   unused Ø2.8 grid holes (trim-cup points), and **three blind Ø2.8 holes
   from below** at (±50, 34) and (26, 64), 5 deep under a 1.0 mm skin, for the
   shelf posts: they do not show on the top.
4. **The six I3 latch strikes** (the five sectors' at station +27.5°, r 74.5,
   and the tray's): a Ø6.6 bore and two entry slots from the top, a recess
   underneath that the rotor's lugs turn into. The recess ceiling is the cam,
   printed as a bridge on the bed side: clear any sag and check the 0.8 mm
   ramp came out as a slope, not steps (VERIFY,
   [7.2](#72-the-sectors-and-the-cap)).
5. An insert (A-13) in each of the ten thumbscrew pockets, flush: **a proud
   insert holds the coxa plate's pads off the deck**
   ([1.2](#12-heat-set-inserts)).
6. **Magnet washers:** glue a B-20 washer into each of the five Ø8 × 1.4
   recesses (station −25.5°, r 76), either face up; the shell feet's magnets
   land on them. **Zinc-plated steel, not stainless**, and not a 12 mm fender
   washer (it does not fit).
7. Learn the numbering. Station k has k + 1 dots by its port; the triangle
   at (0, 30) points north, to station 0; stations run counter-clockwise from
   above:

| leg | station | dots | ids yaw / hip / knee |
|---|---|---|---|
| 0 | 90° (north) | 1 | 1 / 2 / 3 |
| 1 | 162° | 2 | 4 / 5 / 6 |
| 2 | 234° | 3 | 7 / 8 / 9 |
| 3 | 306° | 4 | 10 / 11 / 12 |
| 4 | 18° (378 in params) | 5 | 13 / 14 / 15 |

A leg goes to the station whose ids its servos carry; the bench kit's leg is
leg 0.

### 4.3 Hang the tub

The keel tub hangs east-west under the deck at y −20, door −x, nose +x, its
roof 8 mm under the deck (z −18). The lid goes up first, then the tub.

1. **Inserts:** 8 × A-13 in the lid's bosses ([1.2](#12-heat-set-inserts)).
2. **The nose, before the tub goes up** (the power tree in
   [WIRING_HARNESS](WIRING_HARNESS.md#power-tree)): a FEMALE XT60 into the
   floating carrier (it must float ±0.8 on its four webs; how it is held in
   its pocket is VERIFY), the loop key's chassis XT60E-M into its pocket
   above the carrier, facing +x, the fuse holder in the free room over the
   carrier (16.3 mm high, not modelled), and the 14 AWG feed out through the
   Ø5.5 exit in the NE nose chamfer. Mark the polarity at the carrier. Every
   XT60 number is VERIFY: caliper a mated pair first ([8.4](#84-the-body)).
3. **Lid and rails together:** the lid's six hanger bosses up against the
   deck's underside, the two `tray_rail`s on the deck's top over the inboard
   tab holes. Six M3 × 16 from the deck top into the lid's inserts: four
   through the rail tabs at (±40, −20 / 0), as ISO 7380 button heads (a socket
   head does not fit under the tray plate), two at (±58, −18). The rails'
   north tabs (±40, 20) take an ISO 7380 M3 × 10 each, thread-forming into
   the deck's Ø2.8 hole, no nut: nothing hangs under them, and the tip comes
   out 2.0 mm under the deck, 1.9 mm over the star board at (40, 20).
4. **The tub:** offer it up 1.6 mm north of its place, raise it to the lid,
   slide it 1.6 south so the north wall's return closes over the lid's north
   edge, then two M3 × 10 from below up through the south pilasters into the
   lid's inserts.
5. **The 14 AWG feed:** from the exit it runs under leg 4's drop, where it
   hangs free for 26.5 mm, 0.2 mm over the tub's bottom (nothing fits round
   it there), then up the riser, snapped into the clip on the tub's north
   wall at x ≈ 59, and west to the PDB on the hub shelf
   ([4.6](#46-the-hub-shelf)). The clip's snap is VERIFY.
6. **Looms:** the leg 2 and 3 looms tie (3.6 mm ties) to the lid's tie bars
   beside the (±40, 0) bosses.

### 4.4 The door

1. A latch cartridge, made captive as in
   [1.3](#13-batch-1-coupons-and-the-blank), glued flush in the door's south
   ear (A-14), the housing's flat to the pocket's flat.
2. Rotor at OPEN. The spigot into the opening, the hook tab's barb behind
   the north wall's keeper (it snaps); the ear lands on the strike in the
   tub's south boss. With the sled in, the door's boss presses its tail lip
   0.5 mm: that holds the sled.
3. **Lock it from +x:** the long screwdriver along the robot's south wall into
   the rotor's slot (it faces +x at x −85.7), a quarter turn to the stop. A
   straight Ø5 shaft is free there for 260 mm, on the stand too (2.65 mm to
   the cradle's south wall, 0.55 under the drop keep-outs of legs 2 and 3);
   it clears the carapace skirt at x 87.3, hence the 183 mm. A slotted rotor
   head that turns from the door side is a follow-up.
4. Off: latch to OPEN, push the tab's free end out, pull the door.

### 4.5 The battery sled

1. **Size the pack first:** the one you bought against
   `interfaces.battery_sled` (`pack_l` 138, `pack_w` 44, `pack_h` 25, soft
   case only). Another size goes into params, and `./rocky.sh cad-check
   part_bay` must pass before the sled and the tub print (B72).
2. The sled's XT60E-M (B-12) glued in its nose pocket 8.0 mm proud of the
   nose (VERIFY with the mated pair), on the pack's lead, polarity marked.
3. Glue the 2 mm foam pad to the tub's roof, over the pack. Nothing straps
   the pack (pick 8 b): the tub boxes it and the pad takes its play.
4. **Dry run, loop key out, no pack:** slide the empty sled in from the door
   end on the floor rails until its XT60 meets the carrier; the carrier
   floats and the pair telescopes 7 mm (VERIFY). Door on, then off, sled
   out. On the stand it comes out with leg 1 at yaw +20° or less (at +30
   leg 1 meets it 130 mm out).
5. Live, with the pack: [6.4](#64-the-pack).

### 4.6 The hub shelf

The harness hub under the deck, north of the tub (picks 1, 9, 14). Build it
on the bench, boards first.

1. **Up, on the printed standoffs:** the 40 × 30 star board and the
   30.5 × 30.5 PDB, 4 × M3 × 6 each, thread-forming.
2. **Face-down under the plate:** the bus adapter on four Ø5 × 2.5 spacers,
   4 × M2.5 × 8 pan heads from below into the plate's Ø2.05 bores; the buck
   and the UBEC under 3.6 mm zip ties. The adapter's H2 UART leads go on
   soldered or in a right-angle housing: a straight Dupont on a vertical H2
   header hangs 5.5 mm below the shelf and into the stand's crown plate
   (VERIFY on the board).
3. Wire the boards ([5](#5-wiring)) while the shelf is on the bench.
4. **Up to the deck:** three M3 × 30 socket heads from below, through the
   plate and the Ø8 posts, into the deck's blind holes (heads in their Ø6.4
   counterbores). From the top a head at (±50, 34) would sit under the docked
   coxa plates of legs 1 and 4.
5. **Service is with the robot off the stand:** on the stand the crown plate
   covers three of the adapter's four screws (B134).

### 4.7 The avionics tray (I4)

The Pi 5 and the IMU only (pick 14). The tray sits at (0, 0) turned 180° on
its two rails, three lugs a side under the rails' windowed lips, and its
tongue latches at (0, −43).

1. **Off the robot:** glue `tray_latch_boss` under the tongue (A-14), then a
   latch cartridge into the boss's pocket, flush with its underside, the
   housing's flat to the pocket's flat.
2. **The IMU goes on before the Pi,** which covers it. SPI0, INT and RST
   soldered straight onto its pads (header pins would hit). The four
   `imu_grommet`s into the Ø4.8 holes of the four raised seats (20.32 ×
   17.78 about (0, 13.5)), the bottom flange in the Ø9 recess under each. The
   BNO085 on them, then 4 × M2.5 × 10 (B-21) from above with a standard
   (DIN 934) nut under each: the Ø2.7 grommet bore does not thread. The
   × 10 is the checked length (its tail ends 2.9 mm above the deck); it
   passes the nut by only ~0.2 mm, so snug it.
3. **The Pi 5 + cooler,** once bought: 4 × M2.5 × 6 or × 8 (B-21),
   thread-forming into the 11 mm standoffs (58 × 49), **USB end +x**, toward
   leg 4. Caliper the Ø6.8 standoff tops against the Pi's pads (B108). Only
   the lower middle USB-A port, next to the RJ45, takes a plug: a right-angle
   one ≤ 20 mm long with an overmold ≤ 14 × 7, cable down or south (B128).
   Keep it for the lidar.
4. **Bulkhead** (the north end): the XT30 (the Pi's 5 V from the buck) and
   the XH-5 trunk (the Pi's UART to the adapter), every cutout VERIFY with
   the real panel parts. Both go down the deck's (0, 50)
   slot to the shelf and unplug at the bulkhead before the tray slides. The
   J6 foot-switch lines have no bulkhead connector (B123).
5. **On:** carapace off, sled out, robot off the stand. Rotor at OPEN. Lower
   the tray onto the rails 16 mm south of home, the lugs through the lips'
   windows and the rotor in the strike's slot; slide it 16 north to engage
   the lugs; from below, through the tub's driver column at (0, −43), a
   quarter turn to LOCKED. Plug the bulkhead.
6. **Off:** the carapace off (slid, the boss meets sector 2's latch pad from
   14 mm; lifted, the Pi meets sectors 3 and 4, the cap and by 42 mm sector
   1), the robot off the stand (the cradle blocks the strike's column), the
   sled out (the column runs through it), the bulkhead unplugged, the latch
   to OPEN, slide 16 south (the strike's slot stops it at 16.5), lift. It
   does not come out northward: the plate would cover the trunk slot.

### 4.8 The bench stand

It holds the robot by the keel tub and touches nothing else, so the carapace
stays on (`part_stand.py`). Print it after the tub: without the tub nothing
rests on it.

1. Stack `stand_base`, the two `stand_section`s, `stand_crown`; each skirt
   drops over the one below when turned right (every 72°).
2. Deck-bottom height: 89.4 mm (base + crown), 169.4 (one section), 249.4
   (two). A leg reaches 153.3 mm below the deck to its foot point, 183.5
   with the B95 hand: two sections free every leg, one does not.
3. Lower the robot so the tub drops between the cradle's walls (2 mm
   lead-ins), door end −x, the floor's Ø6.3 hole onto the pin at (0, −20).
   It goes in one way only. The pin has 0.45 mm of play; ±1 mm in x or y
   binds.
4. On the stand you can open the door, swap the sled (leg 1 at yaw ≤ +20°)
   and turn the sector latches at stations 162 and 306 with the sled out;
   not the tray's latch, and not the hub's adapter screws.

### Check before moving on

- `./rocky.sh cad-check` exits 0 after any params change, before you print.
- The deck printed after batch 0 was filed; inserts flush, cones clean,
  washers below the top face.
- The tub hangs square, the north return over the lid, both pilaster screws
  in; the door locks and comes off; the empty sled slides in and out.
- The shelf's boards fixed and its three screws in from below; the tray
  latches and releases in the order of 4.7.
- On the stand: the tub on the pin, the carapace clear, no foot touching on
  two sections.

## 5. Wiring

The power tree, the hub, the tray and the deck-side looms are specified in
[WIRING_HARNESS.md](WIRING_HARNESS.md): build them by it, every drop tested
with nothing plugged in. The in-leg harness is [2.7](#27-leg-harness).

### You need

- Power: B-08, B-09 + 15 A fuses, B-10 buck + 1000 µF 16 V, B-11 UBEC (one
  spare), B-12 and B-25 (the female XT60), B-13, B-24 the 30.5 × 30.5 PDB.
- Harness: B-14, B-15, B-16, B-18 star-board parts, A-06, B-23 cable ties.
- Avionics: A-02, B-30 the UART lead, C-01, C-05 the lidar's right-angle
  USB-A cable, and B-03 Pi 5 + B-04 once one leg walks.
- Tools: A-04, X-12, X-14, X-15 (custom-length XH looms only).
- Done first: the bench leg passed the runbook; every servo labelled and in
  the leg of its ID.

### Rules for every step

- The [safety list](#safety) holds throughout.
- Colours: red 12 V, yellow 6 V, black GND, white data, blue sensors. Label
  every drop with its leg number at BOTH ends.
- Gauges (B-15): 14 AWG pack run, 20 AWG silicone per leg pair, 24–26 AWG for
  data, 6 V and sensors.

### 5.1 The order of work

1. **Power tree** ([WIRING_HARNESS](WIRING_HARNESS.md#power-tree)): pack →
   sled XT60 → the tub's carrier → fuse and loop key in the tub's nose →
   14 AWG out of the NE chamfer and up the riser → the PDB on the shelf →
   five XT30 leg drops, the buck (the tray's 5 V XT30), the UBEC and the
   adapter's 12 V input. Until the tub exists the chain
   starts at the bench supply's XT60 lead into the fuse
   ([2.8](#28-the-bench-legs-power-lead)).
2. **The bus adapter on the Pi's UART** (pick 14): H2 RXD to Pi pin 10, TXD
   to pin 8 (crossed), GND from pin 9 or 14; jumper H4 to A. In UART mode the
   adapter's logic runs only from its 12 V input, so the bus is dead with the
   loop key out. Before the Pi is bought, the adapter's USB-C goes to a
   laptop.
3. **The Pi's 5 V and the hand's 6 V:** the buck feeds the Pi's header
   (WIRING_HARNESS has the route); set the Pi's EEPROM
   `PSU_MAX_CURRENT=5000` (or `usb_max_current_enable=1` in `config.txt`),
   or it will not know it has 5 A; the 5 V reaches the tray through its
   bulkhead XT30. The UBEC's jumper to 6 V, its output to the star board's
   J0.3 only (the hand rail), measured at its plug before it goes in.
4. **Star board** ([build
   order](WIRING_HARNESS.md#star-board-build-order)): build and buzz it on the
   bench before it goes on the shelf: J1–J5 pin 1 to J0.1; pin 2 to J0.2 and
   J6 GND; pin 3 to J0.3; each pin 4 to its own J6 pin only; pins 5, J0.4,
   J0.5 to nothing.
5. **Looms and power pairs:** every drop reaches its port from under the
   deck (B109). Check each length on the built body against WIRING_HARNESS's
   cut-length table, terminate at true length, label both ends with the leg
   number, and write the real lengths next to the table. Ties: legs 1 and 4
   at the shelf posts, leg 0 at the (12, 50) zip anchor, legs 2 and 3 at the
   lid's tie bars.
6. **The Pi:** its OS and microSD setup is not in the repo yet (B96); ROS 2
   Jazzy targets Ubuntu 24.04 ([ros2/README.md](../ros2/README.md)), plus
   `dialout` and `pip install -e ".[hw]"` ([runbook
   §0](../bench/BENCH_RUNBOOK.md#0-before-anything-is-plugged-in)). On its
   header: the BNO085 on SPI0 (GPIO 8 CE0, 9 MISO, 10 MOSI, 11 SCLK; INT and
   RST on free GPIOs) and the five foot switches on free GPIOs with pull-ups,
   each closing to GND. Write your choices into the [pin
   map](WIRING_HARNESS.md#pi-5-pin-map-draft-verify) (B43).

### 5.2 Test before any servo plugs in

No pack or supply, loop key out, every leg unplugged, J0 unplugged, the
tray's XT30 (the Pi's 5 V) unplugged.

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

- The robot on the stand by its tub, wired and tested; the carapace off.
- The five calibrated legs, each with two knobs and M3 × 16 hex bolts.
- The tray electronics; A-04 with its XT60 lead to the fuse.
- B-05 pack, B-06 charger, B-07 alarm and LiPo bag.

### 6.1 Dock a leg (I1)

![Leg 0's coxa base docked on its I1 port, station 0 (north, one dot)](../cad/out/assembly/leg_on_deck.png)

The two cone seats place the leg and take shear and torque. In stance the
ground lifts the plate's outboard end, the plate pivots on its inboard pads
and the two thumbscrews carry the pull (B118); the hook is a catch for a
hanging leg ([I1](INTERFACES.md#i1--leg-port-deck--coxa-base-the-flagship)).

1. Loop key out.
2. **The leg's own carapace sector off**, if the carapace is on: from below,
   its latch to OPEN, lift it (B125). The sectors at stations 162 and 306
   latch through the tub: the sled out first ([7.2](#72-the-sectors-and-the-cap)).
3. Leg by its coxa, the plate at 7–10°, outboard end up: bring it in
   radially and pass the L through the station's 6.2 slot (leg-local x −47)
   without force, then hook it.
4. Hooked, plug the station's XT30 and XH-5 to the leg's drop at the cutout,
   by hand; only a drop that passed [5.2](#52-test-before-any-servo-plugs-in).
   B120 is open: as drawn the mated XH-5 pair may not pass the 11 × 11
   cutout, so note what you find.
5. Lower the plate onto the two cones at (−32, ±17.3): it centres itself. If
   you push, stop and find what binds.
6. A knob with its M3 × 16 in each inboard hole at (−41, ±17), finger-tight:
   it takes 5.5 of the insert's 5.7 mm and stays inside the deck. No longer.
7. Push the coxa sideways, lift its end: the plate must not rock or rise.

To swap a leg: key out, its sector off, knobs out, lift it off the cones,
unplug, unhook. **The thumbscrews are not captive:** unscrewed, knob and bolt
lift out. Keep each leg's pair with it, e.g. in a bag taped to it.

### 6.2 First power: bench supply, nothing plugged in

1. Supply 12.0 V, ~1.5 A, output off, checked; its XT60 lead into the fuse and
   loop key.
2. Output on, key in: 12.0 V, + on red, at the PDB and every port's XT30.
3. 5.0 V from the buck at the tray's XT30 before it is plugged into the
   bulkhead; then the Pi, if bought yet, boots; its 5 A setting
   ([5.1](#51-the-order-of-work) step 3).
4. **6.0 V at the UBEC's plug**; only then its lead into J0, and 6.0 V (pin 3
   to 2) on J1–J5 and every port's XH-5. Anything near 12 V: stop.
5. The adapter on its host (`ls /dev/ttyACM*`, `dialout`, the install); the
   Pi's own control loop is B35.

### 6.3 One leg, then all legs

1. Robot on the stand by its tub, the carapace off, feet clear.
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

1. B-05: soft case, ≤ 138 × 44 × 25 (the BOM's pack is 132 long; pick 5
   keeps the bay at 138). **Hard-case packs (~37 mm) do not fit.** Another
   size goes into `interfaces.battery_sled` before the sled prints
   ([4.5](#45-the-battery-sled)).
2. Charge in the LiPo bag, never unattended; store at storage charge. The
   alarm (B-07) rides the balance lead whenever the robot runs: the 9.9 V
   floor is software, the alarm is not.
3. **First mate:** polarity checked at the sled nose and at the carrier,
   loop key out, robot on the stand. The pack in the sled, the sled in until
   the XT60s mate, the door on and latched ([4.4](#44-the-door)), then the
   loop key in.
4. **To swap packs:** limp the servos, key out, door off, sled out (on the
   stand with leg 1 at yaw ≤ +20°), swap, sled in, door on, key in; the Pi
   reboots.
5. Prove the E-stop on the bench supply first: pull the loop key; everything
   goes dark, the Pi too.

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
- Every leg: seated on its cones, no light under the inboard pads (the 0.2 mm
  gap outboard of them is designed), knobs snug, no rock.
- IDs 1–15 (16–20 with hands) each on its labelled station; `PROTECT_CURRENT`
  archived on all 15; the monitor quiet at rest.
- The loop key cuts everything; the pack is charged and alarmed, and goes in
  only with the key out.

## 7. The carapace

Last, after the power-up. The stand holds the robot by its tub, so the
carapace can stay on there; one sector comes off for a leg swap, and the
carapace for the tray ([4.7](#47-the-avionics-tray-i4)).

### You need

- 5 × `shell_sector` (from the current tree: since 2026-10-08 each clears
  the docked cup's back wall, B124) and `shell_cap`; 5 latch cartridges
  (`latch_housing` + `latch_rotor`), A-14 CA, a flat screwdriver with a
  blade ≤ 5 mm.
- B-19 N35 Ø6 × 3 magnets: one per sector foot, and ten for the hatch (five
  seats, five in the cap).
- Optional: `dovetail_shoe` or `whisker_shoe` with a `thumb_knob_m3` and an
  A-19 M3 × 16 each; `trim_cup` + lid, B-20 washers, an A-19 M3 × 16.

### 7.1 Magnets

**Every magnet's NORTH face points away from the robot, the way the panel
lifts off, on panel and frame alike.** At each joint the frame magnet shows N
and the panel magnet S: two magnets stacked the same way up attract.

1. Mark N on every magnet before gluing (a compass's north end swings to a
   magnet's S face). Keep a reference pair (two loose magnets that snap
   together show the faces that must meet); pull-test each joint loose.
2. **Sector feet:** one in each foot's Ø6.25 × 3.2 pocket (az −25.5°, r 76,
   opening down). It meets a steel washer, so either face.
3. **Hatch** (D063, B88): each sector's seat has a walled pocket (a boss
   under the seat ledge at r 42.5, opening up) and the cap's plug five
   (opening down). All ten go in **N up**: the seats show N, the cap's show
   S. Glue each flush; with all ten in, the cap still seats on the five
   sectors (0.000 mm³ as modelled).
4. **Demo parts:** `shell_sector_demo`'s pockets are blind (a boss under
   each).

### 7.2 The sectors and the cap

The seam tongues locate each sector; its foot magnet on its washer and, since
D063, its I3 latch hold it (B87): a bayonet cartridge in the foot's pad,
turned a quarter turn from under the deck into the deck's strike. The latch
only retains (D020): **do not carry the robot by the shell.** The 0.2 mm cam
bite is VERIFY on `frame_coupon` first (B99,
[1.3](#13-batch-1-coupons-and-the-blank) step 7).

1. **Cartridges, off the robot:** each rotor into its housing, lugs through
   the two keyways, then a quarter turn off them: it is captive.
2. Push each cartridge up into its sector's latch pad (az +27.5°) from the
   foot side, head first (the Ø10 head passes the Ø14.3 pocket), the housing's
   bottom flush with the foot face; A-14 CA. The housing's flat goes to the
   pocket's flat, the only way it fits (±2.5°, the keyway index, B107): its
   keyways then sit 135° from the entry line, outside the 0–90° the rotor
   is worked over, so on the robot a screwdriver pushing up never meets
   them. Off the robot, keep a sector's rotor between OPEN and LOCKED:
   turned 45° back past OPEN it reaches its keyways and can drop out.
3. Rotor at OPEN: its lugs in line with the strike's entry slots, which run
   at 45° to the radius (the slot across the rotor's end is square to the
   lugs).
4. Lower the first sector straight down over a docked leg, centred on its
   station: the rotor drops through the deck's strike and the foot magnet
   lands on its washer.
5. From under the deck at the strike (station +27.5°, r 74.5): the
   screwdriver in the rotor's slot, a quarter turn clockwise (seen from
   below) to the stop. It starts to bite at about 40°; lifted, the sector
   holds. Not a coin: one gets only ~0.5 mm into the 1.5 mm slot before it
   meets the deck.
6. Add the others the same way: each tongue (+36° edge) into the neighbour's
   groove (−36° edge), the last too. Any sector fits any station.
7. `shell_cap` on the hatch seat: the magnets pull it home; lift it by the
   thumb notch.
8. To reach a leg port: from below, a quarter turn back to OPEN, then lift
   that sector straight up.

**The sectors at stations 162 (leg 1) and 306 (leg 3)** latch over the keel
tub: the screwdriver goes up a Ø6 column through the tub's floor and roof,
which is the sled's space, so take the sled out first (on the stand too).

The loop key faces +x in the tub's nose wall (pick 3). Skip the charging dock
(`dock_base`, `dock_tower`; B14).

### 7.3 Accessories (I6)

A `dovetail_shoe` or `whisker_shoe` slides down a vertical bar at ±27° (10
stations, 16 mm long) and its set-knob locks it (D063, B83): the ring's male
is undercut, so the shoe cannot lift off, and the set screw wedges it
against the flanks.

1. An A-19 M3 × 16 hex head pressed into a `thumb_knob_m3` (as in
   [1.1](#11-batch-0-the-printer-and-the-port) step 4), threaded into the shoe's
   angled bore; back it out until its tip clears the slot (the knob about
   7 mm off its spot face).
2. Turn the shoe so its knob faces the leg's arch, away from the seam: two
   knobs facing one seam meet.
3. Slide it down the bar from above until it seats, then turn the knob in
   until the screw wedges the male. It clamps with the knob about 6 mm off
   its spot face and never reaches it (an M3 × 12 would clamp at 2.1 mm; 16
   is the one the BOM buys). Lifted, the shoe holds.

The ~15 N rating is an estimate for 24 mm: bench it on the 16 mm bar.
Sensors, not handles.

### 7.4 Ballast (optional, B3)

`trim_cup` holds a stack of B-20 washers on any grid hole; one M3 × 16 through
lid, washers and cup floor (12.2 mm) leaves 3.8 mm in the deck. Note mass and
hole in `NOTES_INBOX.md`; the repo has no CoM target.

### Check before moving on

- Magnets marked and pull-tested before glue; N away from the robot.
- Every sector flat, latched and on its washer, seams flush; the cap pulled
  home by its magnets.
- Optional (B48): the Pi's Wi-Fi RSSI with the carapace on and off.

## 8. Open on the first build

One list, grouped by where it bites. **Blocks** marks a design gap that stops
a print or a step until the CAD changes; the rest are measurements, decisions
and checks. Ids refer to [DESIGN_BACKLOG.md](DESIGN_BACKLOG.md).

### 8.1 Prints

- **Fit ladder and port coupons → params** (B28, B117; **block** the deck and
  every `coxa_yaw_base`).
- **`coxa_yaw_base` at servo stall** (B119, B139; **blocks** four of the five
  bases): SF 1.10 in FEM case V on the real I1 support; the fix is an owner
  decision after a measured option round.
- **PLA ladder, PETG parts:** `params.print` names no material; if a PETG part
  fits unlike its PLA coupon, note it with the material.
- **The tube socket has no key** (10 + `clearance_fit`): if row E disagrees
  with row A, decide which wins.
- **Not in the print estimate:** the port coupons, 10 × `thumb_knob_m3` and
  one more per I6 shoe, 7 latch cartridges (5 sectors, the tray, the door),
  4 IMU grommets, one `tube_clip` per exposed tube and one `link_clip` per
  femur.
- **Printed-part strength** (B75): FEM allowables VERIFY; no pull coupon.

### 8.2 The leg

- **The fork's inward push** (FEM case R−, SF 2.47) goes through the side
  cheeks into the horn face and its four screws alone: keep them snug, and
  check them after the first torque tests.
- **I1 thumbscrews are not captive** (B82): knob and bolt lift out with the
  leg ([6.1](#61-dock-a-leg-i1)).
- **The servo's numbers** (B23, VERIFY): horn thread and radius (if M2, A-12
  needs 60 + 60), rim screw size, plug height (then re-run `check_assembly`),
  envelope, mass, whether horn and idler ship fitted.
- **Blanks:** no rim pilots; the glued idler sits 0.3 mm proud of the real
  one and uses up the idler pockets' 0.3 mm clearance.
- **Coupler screw length:** under an M3 × 6 head, 1.9 mm of thread passes the
  horn against a 1.8 mm gap to the case (`out_boss_h`, VERIFY).
- **The gauges as modelled:** `calib_gauge_hip` assumes leg z 0 at jig z 140
  (it is at 134): ~4–6° of femur tilt. `calib_gauge_knee` seats the tube at
  x ≈ 128.2, not under the knee axis at x 140.
- **Bench tests:** the adapter-to-jig data lead is not drawn and no current
  limit is given for a whole leg; `torque_step.py` predicts m·g·arm only;
  watch the channel wall (B49); the per-tick goal speed is VERIFY until the
  runbook's §8b passes (B98).
- **Harness:** threading order, lead construction and lengths go into
  [In-leg routing](WIRING_HARNESS.md#in-leg-routing); the hand's lead (B91):
  B-17 is a 100 mm plug-to-plug cable against a ~0.45–0.55 m run.

### 8.3 The foot

- **The hand-as-foot overshoots l3** (B95; **blocks** cutting five tubes):
  knee to sole is tube + 136.2 mm ([3.1](#31-decide-the-tube-length-and-the-foot)).
- **SEA travel with a tool on** is 3.0 mm, not 7: every tool's socket mouth
  sits 3.0 below the outer at the slider's lowest.
- **The slider** is unchecked (`check_assembly` stops at J5): the 0.9 mm lip
  is the striker's only stop.
- **The switch** is not modelled: a 6 × 13 × 6 body in the pocket overlaps
  the resting slider by ~60 mm³; decide the lever side and the contact that
  closes on load, the same on all legs.
- **The spring** (B44): no free length, rate or preload in params; 9.6 mm of
  room, and A-16's two springs are far softer than B44's ≥ 2 N/mm, so expect
  either on its stop when standing.
- **Outer to tube:** glue is the only retention; switch forces are VERIFY
  until [3.4](#34-measure-the-switch-force).
- **Lead route:** the tube is sealed, so the leads run outside in clips,
  while WIRING_HARNESS says "down the tube" and INTERFACES I2 has the exit
  "the tube above the socket".
- **The plain foot** (B25) is not modelled. **The hand's servo:** the SCS0009
  envelope (VERIFY) overlaps `hand_hub` by ~416 mm³ and the stub's face bears
  on it; also its lead and the hinge pins (1.75 mm filament is likely loose
  in 2.1). **Whiskers:** no switch mount.

### 8.4 The body

- **Deck thickness** (B27): the CAD cuts 6 mm and v0.5 depends on it (the
  blind post holes, the strikes); params `body.deck_t` says 4, unread.
- **The XT60s** (VERIFY): the mated pair's 7.0 mm engagement, the sled's male
  glued 8.0 proud, the 6.0 mm of lead room behind the female, how the female
  is held in the carrier, the loop key's flange and screw holes (not drawn).
  Caliper a mated pair before the tub and the sled print; the door's press
  boss is the feature to file. The pack's live side ends in exposed male pins
  at the sled nose.
- **The tub door's latch** turns from +x with a ≥ 183 mm shaft; a slotted
  rotor head would let it turn from the door side.
- **Snaps:** the riser clip (1.0 mm jaws, 1.8 % strain in PETG) and the
  door's hook tab are VERIFY on the first print.
- **The 14 AWG feed** spans 26.5 mm free under leg 4's drop, 0.2 mm over the
  tub's bottom, the robot's lowest point. A real 14 AWG bend at the riser may
  come within 0–0.3 mm of that drop's keep-out (an estimate, not modelled).
- **The hub:** the PDB's height (parts and leads ≤ 6.0 mm over the board) and
  the adapter's connector positions (no published layout) are VERIFY; three
  of the adapter's four screws are covered on the stand (B134).
- **The tray:** the bulkhead connectors' rear bodies are unmodelled (4.6 mm
  from the bulkhead to the Pi board's edge in the lower row), the J6 lines
  have no bulkhead connector (B123), and the Active Cooler and HATs are
  unmodelled.
- **Leg swap under the carapace** (B125): the leg's own sector comes off
  first, with the screwdriver from below; for legs 1 and 3 the sled is out.
- **The shell over the docked base** (B124): fixed in the shell
  (`cup_back_relief`); moving the leg arch or trimming the cup's back wall
  are the owner's alternatives.
- **In the sim, not on the robot:** the tub's protrusions (latch boss and
  door ear, pilasters, keeper, north return, riser clip) are not in the sim's
  belly; leg 0 touches the shelf plate in 2 of 200 simulated shove falls
  (≤ 1.40 mm), which a leg-0 hip floor of −62.5° clears with identical
  outcomes (the clamp stays on legs 1–4, pick 13).
- **Not parts yet:** the camera bracket (B16), the lidar hatch cap (B12,
  B46), the charging dock's electrical side (B14); no CoM target for
  ballast.

### 8.5 Wiring

- **The harness is a design:** nothing wired yet; file every change in
  [WIRING_HARNESS.md](WIRING_HARNESS.md).
- **Deck-side leads** (B109): every drop reaches its port from under the
  deck.
- **Buck capacitor:** which side the 1000 µF (16 V) goes; on the input it has
  thin margin against 12.6 V.
- **The adapter on UART:** its H2 leads soldered or right-angle, its logic
  fed only from its 12 V input (VERIFY on the board).
- **Leg-side XH-5 pins 1–2** may carry two or three wires each (B121,
  unverified).
- **Lengths and limits:** looms, power pairs, the trunk (B67); no
  bench-supply current limit for five legs.
- **Also open:** the Pi pin map (B43) and bring-up doc (B96);
  `PROTECT_CURRENT`'s 6.5 mA/count unit (VERIFY; no script writes it); where
  the balance-lead alarm rides with the pack in the tub, and whether it is
  heard through the shell; nothing on the Pi reads the IMU or the foot
  switches yet (B35).

### 8.6 Older findings

The geometry check that wrote this guide (2026-09-28) found B80–B96; D063 fixed
nine of them in the CAD. Their record is [DESIGN_BACKLOG.md](DESIGN_BACKLOG.md)
and the 2026-09-28 entry of [BUILD_LOG.md](../BUILD_LOG.md) (older entries:
[archive/](archive/README.md)).
