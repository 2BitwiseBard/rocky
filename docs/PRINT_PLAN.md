# Print plan

What to print, in what order, and what each print proves. Every STL is in
`cad/out/`. The orientations and support policy below are the ones
`cad/check_printability.py` audits, so slice exactly as listed. The print
pack, [`cad/out/PRINT_PREP_PACK.pdf`](../cad/out/PRINT_PREP_PACK.pdf), has a
three-view sheet and slicer settings for the leg, coupon, deck and hand
parts; `./rocky.sh cad-check --derived` regenerates it along with the
estimates.

- **Printer:** the reference is a Prusa i3-class bed-slinger with a
  250 × 210 bed (`print.bed_mm` in `cad/params.yaml`). The checks assert that
  number, so change it for your printer. The one-piece 190 × 181 deck is
  sized for this bed (D012).
- **Material:** PLA for the coupons, the blanks and anything expected to
  change (D011). PETG for the structural leg parts, once a coupon has passed.
- **Grams and hours** come from
  [`cad/out/print_estimate.json`](../cad/out/print_estimate.json) (0.2 mm
  layers). The fill factors are guesses, so expect ±30 %. If the slicer
  disagrees wildly, suspect a wrong-scale import before the slicer.

Print leg parts only from the current tree. Every leg part from before D047
was modelled around the wrong servo envelope: coxa base, fork, crown cap,
femur link, knee carrier, straps, servo blanks v0.2 and the J1/J2 coupons.
The fit ladder, the port/latch/dovetail coupons, the hand set and the deck
did not change.

| batch | what | grams | hours |
|---|---|---:|---:|
| 0 | fit ladder | 27 | 1.4 |
| 1 | four joint coupons + one servo blank | 39 | 2.1 |
| 2 | one leg | 129 | 6.9 |
| 3 | the body | 183 | 9.8 |
| **0–3** | | **378** | **20.2** |
| deferred | four more legs 257 g / 13.7 h · bench jig 162 / 5.9 · stand 165 / 6.1 · carapace 149 / 7.9 | 733 | 33.6 |
| **whole plan** | | **1111** | **53.8** |

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

## Batch 1: four coupons + one blank (PLA, 39 g, 2.1 h)

Print these before any full leg part. Each coupon is a boolean clip of the
production solid (`cad/part_leg_coupons.py`), so a coupon that fits proves
the part it came from.

| part | qty | pose | supports | proves |
|---|---|---|---|---|
| `servo_blank` | 1 | bottom DOWN (rims on the bed) | none | the ST3215 stand-in (v0.3, measured from the STEP) |
| `blank_idler` | 1 | disc DOWN (stub up) | none | glues into the blank's Ø6.2 pocket |
| `coupon_cup` | 1 | back wall DOWN | none | the cup: slide the blank in rear-first, drive 4 self-tappers into the rim holes. The same cup holds all three servos |
| `coupon_yaw_hub` | 1 | hub DOWN | none | the fork's yaw hub: Ø20.3 pocket on the horn, four slotted holes, counterbores from below. On a real servo, this print answers M2-or-M3 and the hole radius (B23) |
| `coupon_hip_hub` | 1 | outer face DOWN | none | femur plate A's hub with `horn_coupler`: lobes into the recess, 2 × M3 × 8 clamps from the outer face |
| `coupon_idler` | 1 | outer face DOWN | none | plate B's tower: the pocket rides the idler, and the notch faces the plugs |
| `horn_coupler` | 1 | disc DOWN | none | goes with the hip-hub coupon |

Fit criteria (write them in `NOTES_INBOX.md`):
- **cup:** the blank slides in with finger pressure and doesn't rock. The
  rim holes line up with a Ø2 pin pushed through the coupon.
- **yaw hub:** seats on the blank's horn with less than 0.3 mm wobble.
  Screws pass at both ends of the slots.
- **hip hub:** the coupler drops into the recess and the clamps draw it
  flat. A 1 mm sideways push meets the lobes.
- **idler:** the tower drops over the glued idler without rocking, and the
  plug notch is clear.

## Batch 2: one leg (129 g, 6.9 h), after batch 1 passes

| part | qty | material | pose | supports |
|---|---|---|---|---|
| `coxa_yaw_base` | 1 | PETG | plate DOWN | yes. The I1 hook lip stands on the bed and holds the plate about 9.3 mm up, so support the whole plate from the bed. The harness channel's roof bridges 11 mm, and the roof rails over the cup are 3 mm cantilevers |
| `coxa_fork` | 1 | PETG | lower hub DOWN, upright | yes, under the upper plate (28 mm over the yaw-servo zone) |
| `horn_coupler` | 2 | PETG | disc DOWN | none |
| `femur_link` | 1 | PETG (CF-PLA later) | plate A outer face DOWN | none: recesses up, bridge walls vertical |
| `femur_plate_b` | 1 | PETG | outer face DOWN | none: pockets up |
| `tibia_knee_carrier` | 1 | PETG | cup floor DOWN | yes, under the tube boss |
| `tibia_sea_outer` | 1 | PETG | tube socket DOWN | none |
| `tibia_sea_slider` | 1 | PETG | flange DOWN | none |
| `servo_blank` + `blank_idler` | 3 + 3 | PLA | as batch 1 | none |

**Assembly order** (`check_assembly.py` asserts it):
1. Put the fork onto the yaw blank's horn, off the base: 4 × M3 × 6 from
   below the hub (M2 × 6 + washer if the horn is M2).
2. Slide the yaw blank and fork into the base cup from the front. Drive
   2 self-tappers from above into the idler-face rim holes and 2 from under
   the plate.
3. Slide the hip blank into the fork's cup from the front; 4 self-tappers.
4. Slide the knee blank into the carrier cup from the front; 4 self-tappers.
5. Fit the couplers onto the hip and knee horns: 4 screws each, driven
   through the counterbores.
6. Fit plate A onto both couplers: 2 × M3 × 8 clamps per hub from the outer
   face.
7. Fit plate B over both idlers: 4 × M3 × 8 into the bridge bosses.
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
- A-10, M3 × 8: coupler clamps 4 + plate B 4.
- A-11, M3 × 10: the tube pinch bolt 1 + the I1 thumbscrews 2 (batch 3).
- A-13, heat-set inserts: the deck's thumbscrew pockets.
- A-14, CA glue: for the blank idlers.
- A Ø2 pin or drill shank for lining up holes.

## Batch 3: the body (183 g, 9.8 h)

| part | qty | notes |
|---|---|---|
| `body_deck` | 1 | 190 × 181, flat, brim on, dry filament. Print it after B27 (`deck_t` 4 vs 6 unified) and after the port coupons dock cleanly |
| `coxa_yaw_base` | 4 more | one per station |
| `port_coupon_deck` + `port_coupon_plate` | 1 each | the I1 hook-pivot dance, if not tried yet (not in the estimate) |
| `busboard_bracket`, `avionics_tray`, `tray_rail` × 2, `battery_sled`, `sled_rail` × 2 | 1 each | the interior; print once the electronics exist ([WIRING_HARNESS.md](WIRING_HARNESS.md)) |

## Deferred

- **Wait until servos are in hand:** the other four legs (4 × batch 2
  without the blanks), the bench jig (`jig_base` + `jig_column`,
  327 cm³, 0.3 mm layers) and the calibration gauges
  (`calib_gauge_hip`, `calib_gauge_knee`), which the bench runbook's
  calibration uses.
- **Print last:** the stand (`stand_base`, `stand_section`, `stand_crown`)
  and the carapace (5 × `shell_sector` + `shell_cap`, 0.25 mm). The
  fixtures weigh more than the legs.
- **The hand is a later tool (B25):** `hand_hub`, `hand_cam`,
  3 × `hand_finger` and the tools. `foot_pad_tpu` (TPU) is a sock over the
  closed hand's cone tip, so it waits for the hand. B25's plain stub foot
  is not modelled yet. None of these are in the estimate.
