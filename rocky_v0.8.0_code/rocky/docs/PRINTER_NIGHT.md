# PRINTER NIGHT — refurb + first prints (v0.6 addendum, 2026-07-30)

> **Session 8 (08-30):** the live plan is now `docs/PRINT_NIGHT_s8.html` — P1, P2 and hand_cam + 3× hand_finger are PRINTED; six more STLs changed (D037: tool_scoop, coxa_fork, shell_sector, stand_section, battery_sled, jig_column) — always slice from the newest zip.


*Everything tonight is PLA (D011) — you own plenty and every one of these
parts is expected to be revised once. The purple stays in the drawer.
STLs are pre-exported in `cad/out/` — no build123d needed tonight.*

## 1. Refurb first
Master plan §3.5 / PRINT_PREP_PACK.pdf checklist: hot end reassembly → PTFE
square → belts → PID tune → **Live-Z on PLA** → E-steps/flow cube. Don't
skip Live-Z; half of "bad tolerances" is first-layer squish.

## 2. Print queue (in order — each print informs the next)

| # | part (STL in cad/out) | ~g | settings | why now |
|---|---|---|---|---|
| 1 | `fit_ladder` | 12 | 0.2 mm, 3 walls, 25% | **measures the printer.** Find: which Ø4 bore the printed pegs slide into, which M3 hole threads clean, which pocket grips an insert, which slot passes a 2.2 lip. WRITE THE ANSWERS IN NOTES_INBOX — session 5 turns them into params and regenerates everything. |
| 2 | `latch_housing` + `latch_rotor` ×2 | 2 | 0.15 mm if patient | tiny mechanism torture test. Rotor should quarter-turn with a coin, cam-tight at 90°. |
| 3 | `thumb_knob_m3` ×2 + `dovetail_male_coupon` + `dovetail_shoe` | 9 | 0.2 mm | knob hex pocket is deliberately snug (5.6 A/F) — note if the M3 head needs persuasion. Dovetail pair should slide + lock with the knob. |
| 4 | `horn_coupler` + `coupler_recess_demo` | 10 | 0.2 mm, 4 walls | castellation fit check (D020's sacrificial-coupler standard). |
| 5 | `port_coupon_deck` + `port_coupon_plate` | 15 | 0.2 mm, 3 walls | **the leg port, both halves.** The dance: tilt 15° → lip through slot → slide inboard to hook → pivot flat onto dowels → (inserts if on hand) thumbscrews. Must dock without force. |
| 6 | `hand_hub` + `hand_cam` + 3× `hand_finger` (v0.2.2 — re-slice, do NOT print an old v0.2.1 hub STL) | 40 | 0.2 mm, 3 walls, support under the collar windows + lug undersides | v0.2.1's lugs were FLOATING islands (D036); v0.2.2 adds the knuckle collar (~+5 g). Pin the fingers (Ø2 filament works as hinge pins — now inserted from OUTSIDE through the collar bores), turn the cam by hand: 0→55° with no scrape. |
| 7 | `calib_gauge_knee` + `trim_cup`(+lid) + `belly_skid` ×2 + `imu_grommet`* | 30 | 0.2 mm | bench-day helpers. *grommets want TPU eventually — a PLA one is a placeholder. |
| 8 (overnight) | `jig_base` then `jig_column` | 175+130 | 0.3 mm layers / 0.6 nozzle if fitted, 25–30% gyroid | the Phase-1 bench jig. Column prints on its BACK (spine down); the Ø4 dowel posts print horizontally — fine at this size, light support under the deck-proxy plate. |
| 9 (if the spool survives) | `calib_gauge_hip` (lying down), `battery_sled`, `avionics_tray`, `tray_rail` ×2, `belly_door`, `shell_sector_demo` | ~150 | 0.2–0.3 mm | the rest of the D020 demo set. |
| 10 (session-5 additions, AFTER the coupons pass) | `busboard_bracket` | 7 | 0.2 mm | wiring hub bracket — trivial, sneak it into any gap. |
| 11 (session-5, purely optional overnight #2) | `shell_sector` (ONE) + `shell_cap` | ~47+35 | 0.25 mm, 3 walls, 12% gyroid, brim | **the creature pass made printable** — one real rock sector to hold in your hand. Prints upright as-exported; the LED groove top edge bridges 2.4 mm (fine). If the silhouette pleases, the other four wait for the measured-clearance regen anyway. *(5b: sectors now seam-interlock — print TWO to feel the tongue-and-groove.)* |
| 12 (session-5b, after the coupons) | `tool_hook` + `tool_scoop` | 16 | 0.2 mm, 3 walls | the first real I2 tools — with a printed `tibia_sea_slider` they're the bayonet fit coupons (insert free, quarter-turn, tug: must retain). |
| 13 (session-5b, overnight-class, whenever) | `stand_base` + `stand_crown` (+1–2 × `stand_section` later) | ~95+107 (+112 ea) | 0.3 mm, 2 walls, 15% | the bench stand: base+crown alone = 47 mm low-service cradle for deck work; sections stack via the printed I6 spigot joints for bench-day leg-free calibration. |
| 14 (session-5c, filler-sized) | `tube_clip` ×3 + `link_clip` ×3 | 7 | 0.2 mm | harness clips — sneak into any gap. `dock_base`+`dock_tower` (~135 g) wait for the Batch-2 electrical decision; print when the charge-port call is made. |
| 15 (session-6) | `servo_blank` ×4 | ~100 | 0.2 mm, 2 walls, 10% | **printed ST3215 stand-ins** — the servos ARE the joints, so with no servos ordered the leg can't assemble without these. Exact envelope (boolean-proven in all three cradles), horn disc takes real M2 coupler/link screws. Retire them the day real servos land. |

**Deliberately NOT tonight:** deck v0.3 + coxa v0.2 (wait for coupon
verdict + measured clearances), foot pad (no TPU until Batch 0), anything
PETG (dial the machine on PLA first).

## 3. What to write in NOTES_INBOX (raw is fine)

```
fit ladder: sliding dowel bore = 4.__ ; M3 tap best = 2.__ ;
heat-set snug = 4.__ ; 2.2 lip passes slot = __
latch: turns? cam bite? | knob hex: fits M3 head?
coupler: seats? wobble? | dovetail: slide force? knob locks?
port coupon: hook-pivot dance works? dowel force? slop after screws?
hand v0.2.1: cam turns 0..55 free? fingers nest closed? binding where?
printer: Live-Z __, PLA temp __, anything weird
```

Session 5's first job is filing these + regenerating every part with YOUR
printer's measured numbers. Failed prints are first-class data — the
graveyard box exists for tonight's output.
