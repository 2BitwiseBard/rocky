# Print plan — the D047 reprint (2026-09-22)

Printer: the refurbished Prusa i3 (250 × 210 bed). PLA for everything in
batch 1 and for the blanks (D011); PETG for the structural leg parts once a
coupon has passed. All STLs in `cad/out/`; orientations and support policy
are what `cad/check_printability.py` audits, so slice exactly as listed.
Grams and minutes are `cad/out/print_estimate.json` (0.2 mm, the fill
factors are guesses ±30 %).

**Do not print anything from the old leg.** Every leg part in the 09-17
photo (coxa base, fork, crown cap, femur link, knee carrier, straps, servo
blanks v0.2, J1/J2 coupons) was modelled around the wrong servo envelope.
The hand set, the latch/dovetail coupons, the fit ladder, the port coupons
and the deck are unaffected.

## Batch 0 — measure first (if not already filed)

| part | qty | why |
|---|---|---|
| `fit_ladder` | 1 (you have it) | write the numbers into `NOTES_INBOX.md`: which Ø4 bore the peg slides in, which M3 pilot threads clean, which pocket grips a heat-set, whether a Ø10 rod enters the 10.3/10.45 sockets. These become `params.print`. |

## Batch 1 — four coupons + one blank (~2 h, ~45 g PLA)

Print these BEFORE any full leg part. Each is a boolean clip of the
production solid, so a coupon that fits proves the part.

| part | qty | pose | supports | proves |
|---|---|---|---|---|
| `servo_blank` | 1 | bottom DOWN (rims on the bed) | none | the servo stand-in v0.3 |
| `blank_idler` | 1 | centre nub DOWN | none | glues into the blank's Ø6.2 pocket |
| `coupon_cup` | 1 | back wall DOWN | none | the cup: slide the blank in rear-first, 4 × M2 self-tappers into the rim holes. The same cup holds all three servos. |
| `coupon_yaw_hub` | 1 | hub DOWN | none | the fork's yaw hub: Ø20.3 pocket on the horn, four slotted holes, counterbores from below. On the REAL servo this is the print that answers M2-or-M3 and the hole radius. |
| `coupon_hip_hub` | 1 | outer face DOWN | none | plate A hub + `horn_coupler` (print 1): lobes into the recess, 2 × M3 × 8 clamps from the outer face. |
| `coupon_idler` | 1 | outer face DOWN | none | plate B tower: the pocket rides the idler; the notch faces the plugs. |
| `horn_coupler` | 1 | disc DOWN | none | with the hip hub coupon |

Fit criteria, written to `NOTES_INBOX.md`:
- cup: blank slides in with finger pressure, no rock; rim holes line up with
  a Ø2 pin through the coupon.
- yaw hub: seats on the blank's horn with < 0.3 mm wobble; screws pass at
  BOTH ends of the slots.
- hip hub: coupler drops into the recess, clamps draw it flat, 1 mm
  lateral push meets the lobes.
- idler: tower drops over the glued idler, no rock, plug notch clear.

## Batch 2 — one leg (~7 h, ~125 g) — after batch 1 passes

| part | qty | material | pose | supports |
|---|---|---|---|---|
| `coxa_yaw_base` | 1 | PETG | plate DOWN | the roof rails over the cup are 3 mm cantilevers: report only, slice with support if your slicer flags them |
| `coxa_fork` | 1 | PETG | lower hub DOWN, upright | yes, under the upper plate (28 mm over the yaw-servo zone) |
| `horn_coupler` | 2 | PETG | disc DOWN | none |
| `femur_link` | 1 | PETG (CF-PLA later) | plate A outer face DOWN | none: recesses up, bridge walls vertical |
| `femur_plate_b` | 1 | PETG | outer face DOWN | none: pockets up |
| `tibia_knee_carrier` | 1 | PETG | cup floor DOWN | yes, under the tube boss |
| `tibia_sea_outer` | 1 | PETG | tube socket DOWN | none |
| `tibia_sea_slider` | 1 | PETG | flange DOWN | none |
| `servo_blank` + `blank_idler` | 3 + 3 | PLA | as batch 1 | none |
| `foot_pad_tpu` | 1 | TPU (or skip) | as exported | none |

Assembly order (asserted by `check_assembly.py`):
1. Fork onto the yaw blank's horn: 4 × M3 × 6 (M2 × 6 + washer if the horn
   is M2) from BELOW the hub, off the base.
2. Yaw blank + fork slide into the base cup from the front; 2 self-tappers
   from above into the idler-face rim holes, 2 from under the plate.
3. Hip blank into the fork's cup from the front; 4 self-tappers.
4. Knee blank into the carrier cup from the front; 4 self-tappers.
5. Couplers onto the hip and knee horns (4 screws each, driven through the
   counterbores).
6. Plate A onto both couplers; 2 × M3 × 8 clamps per hub from the outer face.
7. Plate B over both idlers; 4 × M3 × 8 into the bridge bosses.
8. Tube into the carrier boss; pinch bolt M3 × 10.

A hip or knee servo swap later = plate B off (4 screws), clamps out (4),
plate A off, then the cup's 4 rim screws.

## Batch 3 — the body (~5 h, ~170 g)

| part | qty | notes |
|---|---|---|
| `body_deck` | 1 | 190 × 181, brim, dry filament |
| `coxa_yaw_base` | 4 more | one per station |
| `port_coupon_deck` + `port_coupon_plate` | (have) | the I1 hook-pivot dance, if not yet tried |
| `busboard_bracket`, `avionics_tray`, `tray_rail` ×2, `battery_sled`, `sled_rail` ×2 | 1 each | interior, only once the electronics exist |

Deferred until servos are in hand: the other four legs (×4 of batch 2
minus blanks), `jig_base` + `jig_column` (330 cm³ of fixture), the stand,
the shell sectors, the calibration gauges.

## Hardware for batches 1–2 (also in the shopping list)

- M2 × 6 self-tapping (PA2.0 × 6) — 12 per servo cup → 40 with spares
  (VERIFY the size against the servo kit's own screws when they arrive)
- M3 × 6 pan — yaw horn (4/leg); M3 × 8 — coupler clamps (4/leg) + plate B
  (4/leg); M3 × 10 — tube pinch bolt (1/leg)
- M2 × 6 + M2 washers — only if the horn turns out to be M2
- CA glue for the blank idlers; a Ø2 pin or drill shank for hole alignment
