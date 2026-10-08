# Print plan

What to print, in what order, and what each print proves. Every STL is in
`cad/out/`. The orientations and support policy below are the ones
`cad/check_printability.py` audits, so slice exactly as listed. It slices
each STL at 0.2 mm in that orientation and fails a part that floats (a layer
region with no material anywhere below it), a wall under 0.8 mm (two
perimeters) that persists 1 mm in height, or a part that misses the bed;
supported islands, overhangs and walls under 1.6 mm are warnings, which the
tables' supports column answers (D038). The print pack,
[`cad/out/PRINT_PREP_PACK.pdf`](../cad/out/PRINT_PREP_PACK.pdf) (50 pages),
has a three-view sheet, the slicer settings and the audit line for every
part; dimensioned A4 sheets of the leg parts are in
[`cad/out/drawings/`](../cad/out/drawings/INDEX.md). One `./rocky.sh
cad-check --derived --fem` leaves all of it current (B110).

> **On hold (2026-10-08): `coxa_yaw_base` × 5.** On its real I1 support
> (the inboard pads, the two seats and the two thumbscrews in tension) the
> FEM puts it at **SF 1.10** in case V, the servo-stall push up on the foot:
> 30.4 MPa at the vee socket, 2.29 mm of cup deflection, and the seats lift
> 0.04–0.36 mm (B119, B139; the spine rib's SF 2.33 was at the 4629 N·mm
> design moment, the stall case is 6994 N·mm). The fix is an owner decision
> after a measured option round (an outboard hold-down, a deeper rib, a
> thicker plate, or screw preload as a design load). **Print one for the
> bench leg (batch 2), not five.** Nothing else is on hold.

- **Printer:** the reference is a Prusa i3-class bed-slinger with a
  250 × 210 bed (`print.bed_mm` in `cad/params.yaml`). The checks assert that
  number, so change it for your printer. The one-piece 190 × 181 deck and the
  194.4 mm tub and lid are sized for this bed.
- **Material:** PLA for the coupons, the blanks and anything expected to
  change (D011). PETG for the structural leg parts once a coupon has passed,
  and for the keel tub, its lid and door.
- **Strength:** `./rocky.sh cad-check --fem` loads each structural leg part
  with the largest force the servos can put through it
  ([`cad/out/fem/FEM_REPORT.md`](../cad/out/fem/FEM_REPORT.md), D061). Four
  pass at SF ≥ 2: the femur 2.18, the fork 2.47, the knee carrier 3.92, the
  horn coupler 7.81. `coxa_yaw_base` fails at 1.10 (the hold above). The
  allowables are VERIFY until your own pull coupons say otherwise (B75).
- **Grams and hours** come from
  [`cad/out/print_estimate.json`](../cad/out/print_estimate.json) (0.2 mm
  layers, PLA density). The fill factors are guesses, so expect ±30 %. If the
  slicer disagrees wildly, suspect a wrong-scale import before the slicer.

Print only from the current tree, and reprint anything printed before a
change: every leg part before D047 (the wrong servo envelope); the fork, the
knob, the latch, the I6 and panel coupons, the shell, the tray and
`link_clip` before D063 (2026-09-28); `coxa_yaw_base` (seats, spine rib), the
port coupons, the ladder's row D, the deck, the tray, the sled, the stand
crown and the shell sector before D064 (2026-10-07).

| batch | what | grams | hours |
|---|---|---:|---:|
| 0 | fit ladder + port coupons | 26 + 29 | 1.4 + 1.6 |
| 1 | four joint coupons + one servo blank | 41 | 2.2 |
| 2 | one leg (one `coxa_yaw_base`) | 142 | 7.6 |
| 3 | the body (of it, `coxa_yaw_base` × 4 on hold: 59 g / 3.2 h) | 301 | 16.1 |
| **0–3** (without the port coupons) | | **511** | **27.2** |
| deferred | four more legs 310 g / 16.5 h · bench jig 162 / 5.9 · stand 232 / 8.5 · carapace 152 / 8.1 | 856 | 39.1 |
| **whole plan** | | **1366** | **66.3** |

The port coupons (29 g, 1.6 h) are computed with the estimator's own formula
and FILL; `print_estimate.json` does not price them. Not priced at all: the
interface coupons, the knobs, the latch cartridges, the calibration gauges,
the IMU grommets and the hand.

## Batch 0: the printer and the port (B28, B117; the first print)

Two questions before anything else: what fits this printer prints, and
whether a leg docks on its seats (decision 15, signed 2026-10-07: two printed
30° cone posts on the deck, matching sockets in the coxa plate). The numbers
go into `NOTES_INBOX.md`, then into `cad/params.yaml`, and everything is
regenerated (D032: the canonical path).

### The fit ladder

One `fit_ladder`: PLA, 0.2 mm, 3 walls, 25 %, sliced the way the parts will
be sliced, elephant-foot compensation around 0.15 mm.

| row | features | find | lands in |
|---|---|---|---|
| A | Ø4 bores 4.15 / 4.20 / 4.30 / 4.40 / 4.50, pegs 3.90 / 3.95 / 4.00 | the first bore the 3.95 peg slides into | `print.clearance_fit` (4.30 = today's 0.30) |
| B | M3 holes 2.5 / 2.6 / 2.7 / 2.8 / 2.9 | the first hole a screw threads cleanly | `print.screw_m3_tap` (2.8) |
| C | heat-set pockets 4.4 / 4.6 / 4.8 | the pocket that holds an insert snugly | `print.heatset_m3_d` (4.6) |
| D | slots 5.8 / 6.2 / 6.6 × 24.6 (the deck slot's length) | which slot a port coupon plate's L passes at 7–10°, brought in radially | `interfaces.leg_port.hook_slot_w` (6.2) |
| E | Ø2 pin holes 2.00 / 2.10 / 2.20; Ø10 tube sockets 10.15 / 10.30 / 10.45 | snug pin (Ø2 filament works); the tube seats | hand hinge bore (`part_hand.py`, 2.1); tube socket = 10 + `print.clearance_fit` |

The ladder has **no seat row**: the seat fit is tested on the port coupon
plates, which print their sockets the way the coxa plate does. Row E's middle
column (6.85 / 6.95 / 7.05) is the obsolete bearing seat. A ladder printed
before 2026-10-07 still answers rows A, B, C and E; its row D (3.8 / 4.2 /
4.6) passes no L. Rows B and C need M3 screws and inserts from the bench kit
(`bom/BOM.csv` A-09 to A-13).

### The port coupons

| part | qty | pose | supports | what it is |
|---|---|---|---|---|
| `port_coupon_deck` | 1 | as exported (9.0 tall: the 6 mm deck patch and its 3 mm cones) | none | the deck half: the two cone posts, the 6.2 slot, 2 insert pockets (A-13), the cable cutout |
| `port_coupon_plate_relief46` | 1 | plate DOWN, the L on the bed (as `coxa_yaw_base`) | YES under the whole plate (the L holds it 9.3 off the bed, a 29.9 mm cantilever at z 9.7), with a **support blocker in both seat sockets**; the pads and the 0.2 relief sit on the support interface | the leg half, relieved outboard of x −46 (the repo's `seat_relief_x0`), sockets at fit 0. **Dock this one first** |
| `port_coupon_plate_relief42` | 1 | as relief46 | as relief46 | relieved outboard of x −42 (the designer's value) |
| `port_coupon_plate_fit10` | 1 | as relief46 | as relief46 | relief46 with both sockets 0.1 bigger radially |
| `port_coupon_plate_fit20` | 1 | as relief46 | as relief46 | 0.2 bigger: the last seat-fit step |
| `thumb_knob_m3` | 2 | as exported | none | an A-19 M3 × 16 hex head pressed into each; they become the bench leg's thumbscrews |

The plates print like the real `coxa_yaw_base`, so they also test that print:
the socket rule (a blocker, no support inside a socket) is the same.

### GO / NO-GO

Bores and slot first (D032 allows printing before a regeneration for bores
only), then the port (I1_DOCK_OPTIONS
[§6](archive/prep-2026-10-01/I1_DOCK_OPTIONS.md#6-print-first-and-what-go--no-go-means)).

| test | GO | otherwise |
|---|---|---|
| Row A | 4.30 slides, 4.20 binds: the printer matches params | only 4.40 / 4.50 slides: slicer XY hole compensation +0.05…+0.10 mm, GO, file it. 4.15 / 4.20 slides: GO, note it (a regeneration may tighten the fit) |
| Row D + relief46 | the L passes 6.2 by hand at 7–10°, radially | binds: file `hook_slot_w` 6.6 and regenerate before the deck. 5.8 also passes: GO, note it (6.2 keeps the proven 0.40 margin) |
| Seats (relief46) | lowered, it centres itself on the cones; knobs finger-tight, nothing rocks, no x or y play (< 0.05 on a dial), a 0.2 feeler stays out from under the pads | rock or light under the pads: dock fit10, then fit20. The smallest plate that sits flat is `print.seat_fit` (0 / 0.1 / 0.2) |
| | | if fit20 wins: its vee mouth leaves 1.0 of plate to the −y edge, so apply I1_DOCK_OPTIONS §7 (mouth 0.2, or the seats at \|y\| 17.1) before any `coxa_yaw_base` |
| | | all three rock, or play > 0.15 at every socket offset: the seats fail; fall back to "minimal" (I1_DOCK_OPTIONS §3), owner call |
| relief42 vs relief46 | note which sits without rock | the one that does sets `seat_relief_x0` (−46 today) |
| The sockets as printed | clean cone flanks, a bridged Ø2.5 roof | support inside a socket: the blocker failed; fix the slicer setup, reprint |
| Inserts and knobs (deck coupon) | each insert holds ≥ 300 N pulled toward the plate (the stance design case is 284 N a screw); a hand-tight knob gives ≥ 285 N | an outboard hold-down is needed: owner decision (B118) |
| Cone posts | 85 N on each seat (yaw stall), then 128 N on one post: no crack at the root layer | note it; owner decision |
| Release | hooked at 10° and let go, it rests on the deck vertex and the hook, then centres on the cones as the knobs go in | note it |
| 20 dock / undock cycles | tips and mouths do not round off; the play does not grow | note it |
| Row E pins | snug at 2.10 | loose at 2.00: glue the pins. Binds at 2.20: ream the finger pin holes with a 2 mm drill |

Filing `print.seat_fit` or `hook_slot_w` makes the leg-port keep-outs raise
on purpose until `check_dock` is re-run and `iface.HOOK_ENVELOPE` and
`HOOK_ENVELOPE_PARAMS` are re-recorded together. **The deck and every
`coxa_yaw_base` wait for this table.**

## Batch 1: four coupons + one blank (PLA, 40 g, 2.2 h)

After batch 0 is filed. Each coupon is a boolean clip of the production
solid (`cad/part_leg_coupons.py`), so a coupon that fits proves the part it
came from.

| part | qty | pose | supports | proves |
|---|---|---|---|---|
| `servo_blank` | 1 | bottom DOWN (rims on the bed) | none | the ST3215 stand-in (v0.3, measured from the STEP) |
| `blank_idler` | 1 | disc DOWN (stub up) | none | glues into the blank's Ø6.2 pocket |
| `coupon_cup` | 1 | back wall DOWN | none | the cup: slide the blank in rear-first, drive 4 self-tappers into the rim holes. The same cup holds all three servos |
| `coupon_yaw_hub` | 1 | hub DOWN | none | the fork's yaw hub (D063): the flat hub top the horn face bears on, the Ø6.4 relief for the horn's centre head (its channel prints joined to the two rear slots), four slotted holes, counterbores from below. On a real servo it answers M2-or-M3 and the hole radius (B23) |
| `coupon_hip_hub` | 1 | outer face DOWN | none | femur plate A's hub with `horn_coupler`: lobes into the recess, 2 × M3 × 8 clamps from the outer face |
| `coupon_idler` | 1 | outer face DOWN | none | plate B's tower: the pocket rides the idler, the notch faces the plugs |
| `horn_coupler` | 1 | disc DOWN | none | goes with the hip-hub coupon |

Fit criteria (write them in `NOTES_INBOX.md`):
- **cup:** the blank slides in with finger pressure and does not rock; the
  rim holes line up with a Ø2 pin pushed through the coupon.
- **yaw hub:** the horn face sits flat on the hub top without rocking on its
  centre head; screws pass at both ends of the slots and, snug, hold it.
- **hip hub:** the coupler drops into the recess and the clamps draw it flat;
  a 1 mm sideways push meets the lobes.
- **idler:** the tower drops over the glued idler without rocking; the plug
  notch is clear.

## Interface coupons (PLA, 0.2 mm, with batch 1)

The standards the body, panels and tools attach by (D020,
[INTERFACES.md](INTERFACES.md)). D063 changed all but the tool coupons, so
reprint a set printed before 2026-09-28. All print as exported; the steps are
in the [assembly guide](ASSEMBLY_GUIDE.md#13-batch-1-coupons-and-the-blank),
1.3.

| part | qty | supports | proves |
|---|---|---|---|
| `latch_housing` + `latch_rotor` | 1 each | none | the I3 cartridge, a bayonet (B87): the rotor drops through the housing's two keyways and, a quarter turn off them, is captive. On `frame_coupon` it drops in at OPEN, starts to bite at ~40° and stops at 90°, turned from below with a flat screwdriver (blade ≤ 5 mm). The housing's flat is the keyway index (B107). The 0.2 mm cam bite is VERIFY (B99): expect the first turn to set the PLA lugs |
| `dovetail_male_coupon` + `dovetail_shoe` | 1 each | none | I6 (B83): the shoe slides down the undercut male with its knob backed out and cannot lift off; an A-19 in a `thumb_knob_m3` on the shoe's 45° spot face wedges it. The slot roof bridges 12.2 mm (a warning) |
| `shell_sector_demo` + `frame_coupon` | 1 each | the demo: yes, under the plate (it stands on its two I6 segments); the frame: none | the I3 panel standard as one mating pair: lip in the groove, latch on the frame's strike (the deck's own, 6 mm frame), magnets on magnets. Seat, latch, lift, unlatch (B99 for the bite) |
| `tool_hook`, `tool_scoop` | 1 each | yes (3 walls; two supported islands each) | I2 tool socket: on a printed `tibia_sea_slider` the tool inserts free and quarter-turns, and a tug must not pull it off |

## Batch 2: one leg (142 g, 7.6 h), after batch 1 passes

| part | qty | material | pose | supports |
|---|---|---|---|---|
| `coxa_yaw_base` | **1** | PETG | plate DOWN, the I1 L on the bed | YES under the whole plate (the L holds it about 9.3 mm up: a 32.9 mm cantilever at z 9.7 in `cad/out/printability.json`), with a **support blocker in both seat sockets**: no support inside a socket, whose cone flank and bridged roof print clean. The pads and the 0.2 relief outboard of x −46 sit on the support interface. The harness channel's roof bridges 11 mm. **The one for the bench; the other four are on hold** |
| `coxa_fork` | 1 | PETG | lower hub DOWN, upright | yes, in three places: under the upper plate where it leaves the web and the side cheeks (z ~41, 11.5 mm), under the hip cup's lower wall (z ~44), and under the hip cup's upper side wall over the hip servo (the 21.1 mm step at z 72.5). Clear all three before the hip servo goes in. The horn channel prints joined to the two rear slots (B105 lists the open channels) |
| `horn_coupler` | 2 | PETG | disc DOWN | none |
| `femur_link` | 1 | PETG (CF-PLA later) | plate A outer face DOWN | none: recesses up, bridge walls vertical |
| `femur_plate_b` | 1 | PETG | outer face DOWN | none: pockets up |
| `tibia_knee_carrier` | 1 | PETG | as exported: the tube boss on the bed, the cup above it | yes, under the cup where it steps out past the boss (a 26.8 mm cantilever at z 19.1) |
| `tibia_sea_outer` | 1 | PETG | tube socket DOWN | none |
| `tibia_sea_slider` | 1 | PETG | flange DOWN | none |
| `servo_blank` + `blank_idler` | 3 + 3 | PLA | as batch 1 | none |

Clean up the base's pads only: the 0.2 relief outboard of x −46 is drawn so
that face does not bear. The assembly order, with pictures, is
[ASSEMBLY_GUIDE.md](ASSEMBLY_GUIDE.md#2-the-leg) §2 (`check_assembly.py`
asserts the final fits and every slide-in path). With real servos, the leg
drop (XT30 + XH-5) runs through the base's harness channel
([WIRING_HARNESS.md](WIRING_HARNESS.md#in-leg-routing)).

**Hardware per leg** (design counts; buy from [`bom/BOM.csv`](../bom/BOM.csv)):
A-08 PA2.0 × 6 self-tappers 12 (4 per cup; check the servo's own bag, the
STEP pilot is 1.5 mm) · A-09 M3 × 6 12 (4 per horn; A-12 M2 × 6 + washers
only if the horn is M2) · A-10 M3 × 8 4 (coupler clamps) · A-18 M3 × 12 4
(plate B) · A-11 M3 × 10 1 (tube pinch bolt) · A-19 M3 × 16 hex head 2 (the
I1 thumbscrews, each in a `thumb_knob_m3`) · A-13 inserts for the port
(jig or deck) · A-14 CA for the blank idlers · a Ø2 pin.

## Batch 3: the body (301 g, 16.1 h; 242 g / 12.9 h without the held bases)

Option A, the owner's picks of 2026-10-07: the deck v0.5, the keel tub under
it (tub, lid, door), the battery sled inside, the hub shelf north of the tub,
the avionics tray on top. Print the deck once batch 0 is filed; the tub and
the sled once the pack and the female XT60 are in hand and calipered (pack
size, XT60 engagement: all VERIFY); the stand after the tub.

| part | qty | material | pose | supports | notes |
|---|---|---|---|---|---|
| `body_deck` | 1 | PETG or PLA | flat | none | v0.5: 190 × 181 × 9.0 (the 6 mm deck and its 3 mm cones), 128.9 cm³, brim on, dry filament. It cuts the 21-hole table (`part_deck.DECK_HOLES`): the tray-rail tabs and tub hangers (the rails' two north tab holes are Ø2.8: their M3 × 10 threads into the deck, so do not drill them out), the shelf's three blind post holes from below, the (0, 50) trunk slot, the tray's slotted strike at (0, −43). The five sector strikes' cam is a small bridge on the bed side (a 2.5 mm step at z 2.1): check the 0.8 mm ramp did not print as steps. The CAD cuts 6 mm (B27: params `body.deck_t` still says 4, unread) |
| `bay_tub` | 1 | PETG | open top up, the floor on the bed | **YES under the XT60 carrier only**: a 488 mm² island 2.9 over the floor until its webs print, supported through the open top | the keel tub (I5): the sled's rails in its floor, the floating XT60 carrier on four 1.2 × 1.2 flexure webs and the loop key's pocket in the nose, the door's I3 strike in the south boss, the stand's x-stop hole, the 14 AWG exit in the NE chamfer and the riser clip. The latch boss stands on the bed, the pilasters and the clip on 45° wedges, the north return is a 1.2 step at the top. 194.4 × 76.6 × 38.4 |
| `bay_lid` | 1 | PETG | roof DOWN, bosses up | none (the audit's widest step is a 4.0 mm bridge at z 5.7) | six hanger bosses and two pilaster bosses: 8 heat-set inserts (A-13) |
| `bay_door` | 1 | PETG | on its outer face | none | the tub's −x door: a spigot, the press boss on the sled's tail lip, a latch cartridge glued flush in its south ear, a snap tab at the north edge |
| `hub_shelf` | 1 | PLA or PETG | plate DOWN, posts up | none | the harness hub under the deck: the star board and the 30.5 PDB up, the bus adapter, buck and UBEC face-down under it |
| `avionics_tray` | 1 | PLA or PETG | as exported | none | the Pi 5 (standoffs 58 × 49, 11 tall) and the IMU only; three lugs a side |
| `tray_latch_boss` | 1 | PLA or PETG | underside DOWN | none | glued under the tray's tongue (a boss there would float); its Ø14.3 pocket keeps 1.6 of wall all round (`LATCH_WALL`) |
| `tray_rail` | 2 | PLA or PETG | as exported | none | on the deck's inboard tab holes (±40, −20 / 0 / 20), M3 ISO 7380 button heads |
| `battery_sled` | 1 | PETG | as exported | none (the audit's 10 mm cantilever at z 11.9: check the preview) | sized from the pack you bought (B72): no strap slots (pick 8 b). Its XT60E-M is glued 8.0 proud of the nose (VERIFY) |
| `thumb_knob_m3` | 8 | PLA | as exported | none | 2 per `coxa_yaw_base`, an A-19 in each. Print with the bases |
| `latch_housing` + `latch_rotor` | 2 each | PLA | as exported | none | the tray's and the door's cartridges |
| `imu_grommet` | 4 | TPU (PLA placeholder) | as exported | none | under the IMU |
| `coxa_yaw_base` | 4 | PETG | as batch 2 | as batch 2 | **ON HOLD** (the box at the top) |

## Deferred

- **When the servos are in hand:** the other four legs (4 × batch 2 without
  the blanks and without `coxa_yaw_base`, which batch 3 prints once the hold
  lifts), the bench jig (`jig_base` flat, `jig_column` on its back with the
  spine down and supports on; 0.3 mm layers) and the calibration gauges
  (`calib_gauge_hip` standing as exported, no support; `calib_gauge_knee` as
  exported, supported under its 15.8 mm step at z 1.1), which the bench
  runbook's calibration uses.
- **The stand, after the tub** (0.3 mm, 2 walls, 15 %): `stand_base` +
  `stand_crown` are a U-cradle that holds the robot by its keel tub and
  touches nothing else, so the carapace stays on. Deck bottom 89.4 mm over
  the bench; each `stand_section` adds 80 (169.4 / 249.4), and two free the
  leg's whole reach (153.3 mm to the foot point, 183.5 with the B95 hand).
  Support the crown from the bed inside its skirt (the supports pull out of
  the open bottom) and under the floor's two south corners outside the skirt
  (878 mm²). Without the tub nothing rests on it.
- **The carapace, last:** 5 × `shell_sector` + `shell_cap`, 0.25 mm, 3 walls,
  12 % gyroid, brim. The sector's cavity ceilings are 45° terraces and need
  nothing inside; support the outside overhangs (the widest is a 27.7 mm step
  at z 58.5) and the hatch magnet's seat boss (159 mm² at z 54.5, B88). Since
  2026-10-08 each sector carries `cup_back_relief` over the docked cup's back
  wall (B124): reprint any sector from before. Five latch cartridges go into
  the sectors' pads (B87), ten magnets into the hatch (B88). Latches only
  retain: do not carry the robot by the shell.
- **Printable, not planned yet:** `trim_cup` + `trim_cup_lid` (washer
  ballast on the six unused grid holes), `tube_clip` and `link_clip` (one per
  exposed tube and one per femur, 5 each + spares; `link_clip`'s 1.2 mm jaw
  is a thin-wall warning), `whisker_shoe` (an I6 shoe), `coupler_recess_demo`
  (batch 1's hip-hub coupon covers it) and the charging dock (`dock_base`,
  `dock_tower`), which waits for B14.
- **The hand is a later tool (B25):** `hand_hub`, `hand_cam`, 3 ×
  `hand_finger`; `foot_pad_tpu` (TPU) is a sock over the closed hand's cone
  tip and waits for it. B25's plain stub foot is not modelled yet.

Retired on 2026-10-07, no STL any more: `sled_rail` (the tub's floor rails),
`dock_block` and `belly_door` (the tub's carrier and door), `belly_skid`,
`busboard_bracket` (the hub shelf).
