# Print plan

What to print, in what order, and what each print proves. Every STL is in
`cad/out/`. The orientations and support policy below are the ones
`cad/check_printability.py` audits, so slice exactly as listed. It slices
each STL at 0.2 mm in that orientation and fails a part that floats (a layer
region with no material anywhere below it), a wall under 0.8 mm (two
perimeters) that persists 1 mm in height (a 45° feather where a plane grazes
a cone does not count), or a part that misses the bed; supported islands,
overhangs and walls under 1.6 mm are warnings, which the tables' supports
column answers (D038). The print pack,
[`cad/out/PRINT_PREP_PACK.pdf`](../cad/out/PRINT_PREP_PACK.pdf), has a
three-view sheet and slicer settings for the leg, coupon, deck and hand
parts; `./rocky.sh cad-check --derived` regenerates it along with the
estimates. Dimensioned A4 sheets of the leg parts (extents, a hole table
and the open channels, for checking a print with calipers) are in
[`cad/out/drawings/`](../cad/out/drawings/INDEX.md) (`./rocky.sh
cad-drawings`, FreeCAD). One `./rocky.sh cad-check --derived --fem` leaves
all of it current: the stress check and the drawings run before the pack
(B110).

- **Printer:** the reference is a Prusa i3-class bed-slinger with a
  250 × 210 bed (`print.bed_mm` in `cad/params.yaml`). The checks assert that
  number, so change it for your printer. The one-piece 190 × 181 deck is
  sized for this bed (D012).
- **Material:** PLA for the coupons, the blanks and anything expected to
  change (D011). PETG for the structural leg parts, once a coupon has passed.
- **Strength:** `./rocky.sh cad-check --fem` loads each structural leg part
  with the largest force the servos can put through it and reports a safety
  factor ([`cad/out/fem/FEM_REPORT.md`](../cad/out/fem/FEM_REPORT.md), D061).
  All five pass at SF ≥ 2: the femur is lowest at 2.18 (D062's deck + 8 mm
  rails; it was 1.22), then the fork at 2.47 (D063: an inward foot push, R−,
  held by the horn face alone, carried by the side cheeks). The allowables
  are VERIFY until your own pull coupons say otherwise (B75).
- **Grams and hours** come from
  [`cad/out/print_estimate.json`](../cad/out/print_estimate.json) (0.2 mm
  layers). The fill factors are guesses, so expect ±30 %. If the slicer
  disagrees wildly, suspect a wrong-scale import before the slicer.

Print leg parts only from the current tree. Every leg part from before D047
was modelled around the wrong servo envelope: coxa base, fork, crown cap,
femur link, knee carrier, straps, servo blanks v0.2 and the J1/J2 coupons.
The fit ladder, the port coupons, the hand set and the deck did not change
then. D063 (2026-09-28) changed the fork (a fork from before it does not go
onto the servo) and `coupon_yaw_hub`, the knob, the latch cartridge, the
dovetail coupon and shoes, the panel demo pair, the deck's latch strikes,
the shell sector and cap, the tray and `link_clip`: reprint any of those
printed earlier.

| batch | what | grams | hours |
|---|---|---:|---:|
| 0 | fit ladder | 27 | 1.4 |
| 1 | four joint coupons + one servo blank | 41 | 2.2 |
| 2 | one leg | 142 | 7.6 |
| 3 | the body | 184 | 9.8 |
| **0–3** | | **394** | **21.0** |
| deferred | four more legs 310 g / 16.5 h · bench jig 162 / 5.9 · stand (two sections) 221 / 8.1 · carapace 153 / 8.1 | 846 | 38.6 |
| **whole plan** | | **1240** | **59.6** |

## Batch 0: measure the printer (B28, blocking)

Every clearance in `params.print` is a generic FDM guess until the ladder
is measured. One `fit_ladder` (PLA, 0.2 mm, 3 walls, 25 %) answers the
questions below. Slice it the way the parts will be sliced: elephant-foot
compensation around 0.15 mm, because first-layer squish eats bore
diameters. If you already have a printed ladder, measure that one.
The numbers go into `NOTES_INBOX.md` and then into `cad/params.yaml`, and
everything is regenerated (D032: that is the canonical path).

| row | features | find | lands in |
|---|---|---|---|
| A | Ø4 bores 4.15 / 4.20 / 4.30 / 4.40 / 4.50, pegs 3.90 / 3.95 / 4.00 | the first bore the 3.95 peg slides into | `print.clearance_fit` (4.30 = today's 0.30) |
| B | M3 holes 2.5 / 2.6 / 2.7 / 2.8 / 2.9 | the first hole a screw threads cleanly | `print.screw_m3_tap` (2.8) |
| C | heat-set pockets 4.4 / 4.6 / 4.8 | the pocket that holds an insert snugly | `print.heatset_m3_d` (4.6) |
| D | slots 3.8 / 4.2 / 4.6 | does the port coupon's lip pass 4.2 | `interfaces.leg_port.hook_slot_w` (4.2) |
| E | Ø2 pin holes 2.00 / 2.10 / 2.20; Ø10 tube sockets 10.15 / 10.30 / 10.45 | snug pin (Ø2 filament works); the tube seats | hand hinge bore (`part_hand.py`, hard-coded 2.1); tube socket = 10 + `print.clearance_fit` |

Row E's middle column (6.85 / 6.95 / 7.05 pockets) was the 683ZZ bearing
seat. It is obsolete since D047. Rows B and C need M3 screws and inserts from
the bench kit (`bom/BOM.csv` A-09 to A-13).

On ladders printed before 2026-08-30, row D's index dots are engraved
inside the slots and can't be seen. The slots are 3.8 / 4.2 / 4.6 left to
right.

**GO / NO-GO** for printing before params are regenerated (D032 fallback,
bores only):

| measurement | result | action |
|---|---|---|
| Row A | 4.30 slides, 4.20 binds | the printer matches params: GO |
| | only 4.40 / 4.50 slides | holes print small: set slicer XY hole compensation to +0.05…+0.10 mm, GO, and file the numbers |
| | 4.15 / 4.20 already slides | true to CAD or roomy: GO and note it (a regen may tighten the fit to 0.20) |
| Row D | the lip passes 4.2 | GO |
| | only 4.6 passes | file it and regenerate with `hook_slot_w` 4.6 before printing the deck (the slicer fallback is for bores, D032) |
| Row E pins | snug at 2.10 | loose at 2.00: glue the pins in. Binds at 2.20: ream the finger pin holes with a 2 mm drill |
| Port coupons (`port_coupon_deck` + `port_coupon_plate`) | dock without force: tilt 15°, lip through the slot, slide inboard to hook, pivot flat onto the dowels | binds at the dowels: hold the deck and note which step binds. Slop after the pivot: note it and proceed (the thumbscrews close it) |

## Batch 1: four coupons + one blank (PLA, 41 g, 2.2 h)

Print these before any full leg part. Each coupon is a boolean clip of the
production solid (`cad/part_leg_coupons.py`), so a coupon that fits proves
the part it came from.

| part | qty | pose | supports | proves |
|---|---|---|---|---|
| `servo_blank` | 1 | bottom DOWN (rims on the bed) | none | the ST3215 stand-in (v0.3, measured from the STEP) |
| `blank_idler` | 1 | disc DOWN (stub up) | none | glues into the blank's Ø6.2 pocket |
| `coupon_cup` | 1 | back wall DOWN | none | the cup: slide the blank in rear-first, drive 4 self-tappers into the rim holes. The same cup holds all three servos |
| `coupon_yaw_hub` | 1 | hub DOWN | none | the fork's yaw hub (D063): the flat hub top the horn face bears on, the Ø6.4 relief for the horn's centre head (its channel runs out to one side, joined to the two rear slots over its 1.3 mm depth), four slotted holes, counterbores from below. On a real servo, this print answers M2-or-M3 and the hole radius (B23) |
| `coupon_hip_hub` | 1 | outer face DOWN | none | femur plate A's hub with `horn_coupler`: lobes into the recess, 2 × M3 × 8 clamps from the outer face |
| `coupon_idler` | 1 | outer face DOWN | none | plate B's tower: the pocket rides the idler, and the notch faces the plugs |
| `horn_coupler` | 1 | disc DOWN | none | goes with the hip-hub coupon |

Fit criteria (write them in `NOTES_INBOX.md`):
- **cup:** the blank slides in with finger pressure and doesn't rock. The
  rim holes line up with a Ø2 pin pushed through the coupon.
- **yaw hub:** the horn face sits flat on the hub top without rocking on its
  centre head. Screws pass at both ends of the slots, and snug they hold
  it.
- **hip hub:** the coupler drops into the recess and the clamps draw it
  flat. A 1 mm sideways push meets the lobes.
- **idler:** the tower drops over the glued idler without rocking, and the
  plug notch is clear.

## Interface coupons (PLA, 0.2 mm, with batch 1)

The standards the body, panels and tools attach by (D020,
[INTERFACES.md](INTERFACES.md)) each have a coupon. D063 changed all of them
but the tool coupons, so reprint a set printed before 2026-09-28. They are
small and not in the estimate. All print as exported
(`check_printability.py` audits that pose). The steps are in the
[assembly guide](ASSEMBLY_GUIDE.md#13-batch-1-coupons-and-the-blank), 1.3.

| part | qty | supports | proves |
|---|---|---|---|
| `latch_housing` + `latch_rotor` | 1 each | none | the I3 cartridge, a bayonet since D063 (B87): the rotor drops through the housing's two keyways and, a quarter turn off them, is captive. On `frame_coupon` it drops in at OPEN, starts to bite at ~40° and stops at 90°, turned from below with a flat screwdriver (blade ≤ 5 mm; a coin does not reach). The housing's walls beside the keyways are 1.2 mm (9.4 mm² under 1.6 mm in a layer: a warning, not a failure). Its flat is the keyway index (B107): it fits the panel's D pocket one way, which puts the keyways 135° from the entry line, outside the latch's 0–90° travel; turned within that travel, a rotor cannot leave its housing. The 0.2 mm cam bite is VERIFY (B99): expect the first turn to set the PLA lugs |
| `thumb_knob_m3` | 2 | none | an M3 × 16 hex-head bolt (A-19, 5.5 A/F) presses into the 5.6 A/F hex pocket from the top, threads out the bottom, and must not turn in it (note if it needs persuasion). Since D063 (B82) the knob is 8.6 tall and the whole head sits in the 2.1 mm pocket, 0.1 below the top. A round socket head spins in it. These two are the port coupon's and then the bench leg's thumbscrews; the robot's other 8 print with batch 3, plus one per I6 shoe |
| `dovetail_male_coupon` + `dovetail_shoe` | 1 each | none | I6 (D063, B83): the male is undercut (8 at the root, 12 outboard), so the shoe slides down the 24 mm segment with its knob backed out and cannot lift off; an A-19 M3 × 16 in a `thumb_knob_m3`, on the shoe's 45° spot face, wedges it (the knob ~6.1 mm off the face; an M3 × 12 clamps at 2.1). The shoe's slot roof bridges 12.2 mm (a warning). I6's load rating is benched later on a shell sector's as-built 16 mm segment |
| `shell_sector_demo` + `frame_coupon` | 1 each | the demo: yes, under the plate (its two I6 segments stand on the bed and the plate starts at z 4, a 37.3 mm cantilever in the audit); the frame: none | the I3 panel standard as one mating pair since D063: the demo flipped onto the 60 × 34 × 6 frame puts its lip in the groove (free at ±0.25 mm, located at ±0.6), its latch cartridge on the frame's strike (the deck's own, 6 mm frame) and its two magnets on the frame's. The demo's magnet pockets are blind now, on bosses. Seat, latch, lift, unlatch; feel the magnet pull (B99 for the bite) |
| `tool_hook`, `tool_scoop` | 1 each | yes (3 walls; the audit finds two supported islands on each) | I2 tool socket: on a printed `tibia_sea_slider`, the tool inserts free and quarter-turns, and a tug must not pull it off |

Write the verdicts in `NOTES_INBOX.md` with the batch 1 criteria.

## Batch 2: one leg (142 g, 7.6 h), after batch 1 passes

| part | qty | material | pose | supports |
|---|---|---|---|---|
| `coxa_yaw_base` | 1 | PETG | plate DOWN | yes. The I1 hook lip stands on the bed and holds the plate about 9.3 mm up (a 32.9 mm cantilever at z 9.5 in `cad/out/printability.json`), so support the whole plate from the bed. The harness channel's roof bridges 11 mm |
| `coxa_fork` | 1 | PETG | lower hub DOWN, upright | yes, in three places: under the upper plate where it leaves the web and the side cheeks (z ~41, the widest cantilever, 11.5 mm since D063), under the hip cup's lower wall where it steps out past the plate (z ~44), and under the hip cup's upper side wall, which spans the hip servo (the 21.1 mm step at z 72.5). Clear all three before the hip servo goes in. D063 (B80, B81): 19.6 g (was 14.1), two 3 mm side cheeks, no horn pocket (the horn face bears on the flat hub top), and the horn-head relief, idler pocket and idler-head relief run out through the C's mouth as channels; the horn channel prints joined to the two rear slots. Its drawing lists them as open channels (B105): Ø6.4 × 1.3 deep, Ø7 × 0.6 and Ø20.3 × 1.8 |
| `horn_coupler` | 2 | PETG | disc DOWN | none |
| `femur_link` | 1 | PETG (CF-PLA later) | plate A outer face DOWN | none: recesses up, bridge walls vertical |
| `femur_plate_b` | 1 | PETG | outer face DOWN | none: pockets up |
| `tibia_knee_carrier` | 1 | PETG | cup floor DOWN | yes, under the tube boss |
| `tibia_sea_outer` | 1 | PETG | tube socket DOWN | none |
| `tibia_sea_slider` | 1 | PETG | flange DOWN | none |
| `servo_blank` + `blank_idler` | 3 + 3 | PLA | as batch 1 | none |

**Assembly order** (`check_assembly.py` asserts the final fits and every
slide-in path, step 1's included since D063).
The full procedure, with pictures, is [ASSEMBLY_GUIDE.md](ASSEMBLY_GUIDE.md).

1. Slide the fork onto the yaw blank from the blank's front, off the base:
   the blank passes between the side cheeks, horn face along the flat hub
   top, the horn's centre head and the idler in their channels, until the
   idler stops at the end of its pocket. Hold it there, hub flat on the horn
   face (it can still slide back out), and drive 4 × M3 × 6 up through the
   hub (M2 × 6 + washer if the horn is M2).
2. Slide the yaw blank and fork into the base cup from the front. Drive
   2 self-tappers from above into the idler-face rim holes and 2 from under
   the plate.
3. Slide the hip blank into the fork's cup from the front; 4 self-tappers.
4. Slide the knee blank into the carrier cup from the front; 4 self-tappers.
5. Fit the couplers onto the hip and knee horns: 4 screws each, driven
   through the counterbores.
6. Fit plate A onto both couplers: 2 × M3 × 8 clamps per hub from the outer
   face.
7. Fit plate B over both idlers: 4 × M3 × 12 into the bridge bosses (the
   8 mm rails since D062).
8. Push the tube into the carrier boss; M3 × 10 pinch bolt.

To swap a hip or knee servo later: plate B off (4 screws), clamps out (4),
plate A off, then the cup's 4 rim screws.

With real servos, the leg drop (XT30 + XH-5) runs through the base's
harness channel ([WIRING_HARNESS.md](WIRING_HARNESS.md#in-leg-routing)).
Work out the threading order on the first leg.

**Hardware per leg.** Buy from [`bom/BOM.csv`](../bom/BOM.csv); these are
design counts, not a shopping list:
- A-08, PA2.0 × 6 self-tappers: 4 per cup, 3 cups, so 12. Check the size
  against the servo's own bag first; the STEP pilot is 1.5 mm.
- A-09, M3 × 6: 4 per horn (yaw hub + two couplers), so 12. A-12 (M2 × 6 +
  washers) only if the horn turns out to be M2.
- A-10, M3 × 8: coupler clamps 4.
- A-18, M3 × 12: plate B 4.
- A-11, M3 × 10: the tube pinch bolt 1.
- A-19, M3 × 16 hex head: the I1 thumbscrews 2, each pressed into a
  `thumb_knob_m3` (they dock the leg on the bench jig now, on the deck in
  batch 3).
- A-13, heat-set inserts: the deck's thumbscrew pockets.
- A-14, CA glue: for the blank idlers.
- A Ø2 pin or drill shank for lining up holes.

## Batch 3: the body (184 g, 9.8 h)

| part | qty | notes |
|---|---|---|
| `body_deck` | 1 | 190 × 181, flat, brim on, dry filament. Print it after B27 (`deck_t` 4 vs 6 unified, at least 6: the latch strike asserts it), B28 (the fit ladder), B51 (tray, rails, bracket and power entry placed together) and the bay (B84), and after the port coupons dock cleanly: each of them changes holes in the deck. B51 and B84 are a proposal for the owner, [BODY_LAYOUT_PROPOSAL.md](BODY_LAYOUT_PROPOSAL.md). Today's deck has grid holes only for the star-board bracket, and the five I3 latch strikes of D063 (B87): their underside recess is the cam, a small bridge on the bed side (134.7 mm² of new overhang in the audit, at most 2.5 mm wide at z 2.1, under the 3 mm warning step), so check its 0.8 mm ramp did not print as steps |
| `coxa_yaw_base` | 4 more | one per station |
| `thumb_knob_m3` | 8 more | 2 per `coxa_yaw_base`; the two interface-coupon knobs are the bench leg's. One A-19 M3 × 16 hex head in each (not in the estimate). Plus one per I6 shoe |
| `port_coupon_deck` + `port_coupon_plate` | 1 each | the I1 hook-pivot dance, if not tried yet (not in the estimate) |
| `busboard_bracket` | 1 | the star board; print once the electronics exist ([WIRING_HARNESS.md](WIRING_HARNESS.md)) |
| `avionics_tray` | 1 | a bench carrier for the electronics: since D063 its Pi standoffs are 58 × 49, 11 mm tall, and its IMU seats fit the BNO085 (B89, B90; 14.7 g, the recess ceilings under the seats add overhang). Its latch pocket is the D of the keyway index (B107). The bus adapter, buck and UBEC have no place under its Pi (B108: 9.0 mm of floor under the keep-out; the measured fixes raise the Pi), so keep them beside it. `tray_rail` × 2 waits for B51 (as drawn, tray + rails fit nowhere between the legs) |
| `battery_sled`, `sled_rail` × 2, `dock_block` | — | wait for the bay (B84, B86): nothing carries the sled yet (the proposal: [BODY_LAYOUT_PROPOSAL.md](BODY_LAYOUT_PROPOSAL.md)) |

The estimate above still prices `tray_rail` × 2, `battery_sled` and
`sled_rail` × 2 with this batch, although they wait.

## Deferred

- **Wait until servos are in hand:** the other four legs (4 × batch 2
  without the blanks and the `coxa_yaw_base`, which batch 3 prints), the
  bench jig (`jig_base` flat, `jig_column` on its back with the spine down
  and supports on; 327 cm³, 0.3 mm layers) and the calibration gauges
  (`calib_gauge_hip` standing as exported with no support, since both
  lying poses leave an island; `calib_gauge_knee` as exported, with
  support under its 15.8 mm step at z 1.1), which the bench runbook's
  calibration uses.
- **Print last:** the stand and the carapace. The fixtures weigh more
  than the legs.
  - Stand: 0.3 mm, 2 walls, 15 %. `stand_base` + `stand_crown` alone are
    the 47 mm deck cradle. Each `stand_section` stacks on the printed I6
    spigots and raises it 80 mm (127 / 207 mm); two free the leg's full
    172 mm reach below the deck (`part_stand.py`). The crown's top plate
    spans its hollow skirt, so support it from the bed inside the skirt;
    the supports pull out of the open bottom.
  - Carapace: 5 × `shell_sector` + `shell_cap`, 0.25 mm, 3 walls, 12 %
    gyroid, brim. The sector's cavity ceilings are 45° terraces and need
    nothing inside; support the outside overhangs (the audit's widest is a
    27.7 mm step at z 58.5). Since D063 each sector also has one island
    to support from the bed: the hatch magnet's seat boss (159 mm² at
    z 54.5, B88). Five latch cartridges (`latch_housing` + `latch_rotor`,
    not in the estimate) go into the sectors' pads (B87),
    and the hatch takes ten magnets in walled pockets (B88). Latches only
    retain: do not carry the robot by the shell.
- **Printable, not planned yet:** `belly_door` (an I3 latch-and-magnet
  demo: two latches + two magnets; no bay frame exists for it to close on
  and no strike for its latches, B84), `trim_cup` + `trim_cup_lid` (washer ballast for CoM trimming),
  `belly_skid` × 2 (the sacrificial skid; no place yet under the deck,
  B84, B85), `imu_grommet` × 4 (TPU eventually; a PLA one is a
  placeholder), `tube_clip` and `link_clip` (harness clips: one per tibia
  tube, which needs exposed tube, and one per femur, 5 each + spares;
  since D063 `link_clip` fits the D062 femur (B94), and its 1.2 mm inner
  jaw is a thin-wall warning, 12.5 mm² under 1.6 mm in a layer), `whisker_shoe` (an I6 shoe; like
  `dovetail_shoe`, its slot roof bridges 12.2 mm),
  `coupler_recess_demo` (the coupler pocket alone; batch 1's hip-hub
  coupon covers it) and the charging dock (`dock_base`, `dock_tower`),
  which waits for the charge-port decision (B14). `dock_block` is not
  part of it: it is the robot's I5 bay receiver (`part_battery`) and waits
  for the bay (B84, B86).
- **The hand is a later tool (B25):** `hand_hub`, `hand_cam` and
  3 × `hand_finger` (the tools are already coupons, above).
  `foot_pad_tpu` (TPU) is a sock over the closed hand's cone tip, so it
  waits for the hand. B25's plain stub foot is not modelled yet. None of
  these are in the estimate.
