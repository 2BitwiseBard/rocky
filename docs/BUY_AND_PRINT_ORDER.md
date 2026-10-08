# Buy and print order

What to buy and what to print, in the order that spends the least before each
next step has proved itself. Written 2026-10-08 on the D064 tree. Prices are
`line_usd_est` from [`bom/BOM.csv`](../bom/BOM.csv) (checked 2026-09-28, before
tax and shipping); grams and hours are from
[`cad/out/print_estimate.json`](../cad/out/print_estimate.json) (0.2 mm, ±30 %).
How to print each part is [PRINT_PLAN.md](PRINT_PLAN.md); how to build it,
[ASSEMBLY_GUIDE.md](ASSEMBLY_GUIDE.md).

**Holds:**
- `coxa_yaw_base` × 5. It fails the FEM at servo stall (SF 1.10, case V, on
  its real I1 support; B119, B139). Print **one** for the bench leg; the other
  four wait for the owner's fix and a passing re-run. Nothing else is on hold.

**Check on arrival; each one changes a print:**
1. **The horn's thread and radius** (B23), on the first ST3215 with
   `coupon_yaw_hub`: M3 or M2 sets `horn_screw_m` and `horn_bcd`, which move
   the yaw hub and the couplers. M2 also means buying A-12.
2. **The pack's size** (B72): soft case, at most 138 × 44 × 25. Another size
   goes into `interfaces.battery_sled` before the sled and the tub print.
3. **The XT60s** in the tub's nose: the mated pair's engagement (7.0 mm in
   the CAD), the sled's male 8.0 proud, the female's fit in the carrier, the
   loop key's flange. All VERIFY; caliper a mated pair before the tub prints.

## 1. PRINT: the fit ladder and the port coupons

- **What:** `fit_ladder`; `port_coupon_deck` and the four plates
  `port_coupon_plate_relief46` (first), `_relief42`, `_fit10`, `_fit20`;
  2 × `thumb_knob_m3`. PLA. PRINT_PLAN batch 0.
- **Why now:** it costs only filament. The ladder measures how this printer
  prints fits, and the coupons show whether a leg docks on its cone seats,
  which so far is proven only in CAD (decision 15, B117). Every later part's
  fits depend on both.
- **Cost:** 26 g / 1.4 h (ladder) + 29 g / 1.6 h (port coupons, computed with
  the estimator's formula; not in its JSON).
- **Gate:** ladder rows A, D and E read; `relief46` passes the 6.2 slot by
  hand at 7–10°, brought in radially, and centres itself on the cones. The
  clamped tests need step 2's hardware.

## 2. BUY: the bench kit (BOM phase A)

- **What:** A-01 4 × ST3215 12 V ($82.52), A-02 bus adapter ($4.99), A-03
  ESP32 servo driver ($15.99), A-04 bench supply ($49.99), A-06 servo leads
  ($7.96), A-07 barrel pigtails ($7.99), A-08 PA2.0 × 6 ($5.69), A-09 M3 × 6
  ($7.69), A-10 M3 × 8 ($7.69), A-11 M3 × 10 ($6.69), A-18 M3 × 12 ($6.99),
  A-19 M3 × 16 hex head ($6.00), A-13 heat-set inserts ($9.99), A-14
  threadlocker + CA ($13.43), A-15 carbon tube ($21.95), A-16 springs
  ($16.38), A-17 KW10 switches ($10.09). Instead of A-04: A-05 ($31.80, no
  current limit). Only if the horn is M2: A-12 ($4.00).
- **Where:** the owner's desk page (a private claude.ai page) carries the
  seventeen rows (not A-05 or A-12) as Amazon links: A-10 and A-18 as one M3
  button-head kit, A-14 and A-16 as two items each, A-19 as a search (any
  DIN 933 / ISO 4017 pack), plus the optional X-19 dryer and a textured PEI
  sheet. On 2026-10-08 nothing on it was ticked as ordered.
- **Why now:** the coupons' clamped tests need A-09 to A-13 and A-19, and the
  first servo answers the horn question (B23) before any leg part prints.
- **Cost:** $282.03 at the BOM's prices (the servos from Waveshare direct,
  $20.63 each at 4+, shipped from China, duties on the buyer). The desk
  page's all-Amazon cart is $341.41, the optional dryer and PEI sheet $53.23
  more.
- **Gate:** every servo labelled 12 V (C018 / C047), aluminium horns in the
  bag; the A-19 bolts hex-headed (a round head spins in the knob).

## 3. TEST AND FILE: finish batch 0

- **What:** ladder rows B and C (screws and inserts), 2 inserts in
  `port_coupon_deck`, the knobs on their A-19 bolts; then the port's
  go / no-go (PRINT_PLAN): no rock and no x / y play (< 0.05) clamped, a
  0.2 feeler out from under the pads, the seat fit (relief46 / fit10 /
  fit20), relief42 against relief46, each insert ≥ 300 N pull-out, the knob
  ≥ 285 N preload, the cone posts, the release test, 20 dock / undock cycles.
  File the numbers (`NOTES_INBOX.md` → `cad/params.yaml`), run `./rocky.sh
  cad-check --derived`.
- **Why now:** the deck, the jig and every `coxa_yaw_base` are cut from these
  numbers.
- **Cost:** no purchase; a 0.2 mm feeler, a dial indicator and a scale that
  reads 300 N (about 31 kg) are not in the BOM.
- **Gate:** the GO / NO-GO table passes and the CAD is regenerated. Seats
  that rock at every socket offset: the "minimal" port, an owner call, before
  anything else prints.

## 4. PRINT: batch 1, the joint coupons

- **What:** `servo_blank`, `blank_idler`, `coupon_cup`, `coupon_yaw_hub`,
  `coupon_hip_hub`, `coupon_idler`, `horn_coupler` (PLA); with them the
  interface coupons (the latch, dovetail and panel pairs, `tool_hook`,
  `tool_scoop`).
- **Why now:** each coupon is a clip of a production part: if it fits, the
  part will. `coupon_yaw_hub` on the first real servo settles B23.
- **Cost:** 41 g / 2.2 h; the interface coupons are small and not priced.
- **Gate:** the batch 1 fit criteria (PRINT_PLAN), the horn thread and radius
  filed, the CAD regenerated.

## 5. PRINT: one leg and the bench jig

- **What:** PRINT_PLAN batch 2, one leg in PETG with **one**
  `coxa_yaw_base`, plus 3 servo blanks; the bench jig (`jig_base`,
  `jig_column`) and the two calibration gauges (PLA).
- **Why now:** the bench leg proves the leg before four more are printed and
  twelve more servos are bought.
- **Cost:** 142 g / 7.6 h (leg) + 162 g / 5.9 h (jig, 0.3 mm layers); the
  gauges are not priced. Filament: X-16 PETG, X-17 PLA, both owned.
- **Gate:** the leg dry-fits on the blanks (ASSEMBLY_GUIDE 2.2) and the
  harness threads through the base's channel.

## 6. BENCH: one leg on the jig

- **What:** [bench/BENCH_RUNBOOK.md](../bench/BENCH_RUNBOOK.md) §0–§9 with the
  four bench servos: first contact, IDs, calibration on the jig, the health
  monitor, the thermal soak, the torque step, the goal-speed sag test (§8b).
- **Why now:** the servos' real stall torque, thermal limits, masses and the
  leg's calibration go into params before the robot's other 12 servos, the
  Pi and the pack are bought (B-03 waits for this).
- **Cost:** no purchase.
- **Gate:** `pose_check` under 2°, no `!!` from `apply_limits.py`,
  `PROTECT_CURRENT` archived, the soak and the torque step filed, the
  first-leg numbers filed and the CAD regenerated (ASSEMBLY_GUIDE 2.11 step 1).

## 7. BUY: the rest of the robot (BOM phase B + the senses)

- **What:** B-01 the other 12 ST3215 ($247.56); B-03 Pi 5 4 GB ($110.00) +
  B-04 cooler and microSD ($42.94); B-05 the soft 3S pack ($34.99), B-06
  charger ($38.50), B-07 LiPo bag + alarms ($23.48); B-08 loop key ($5.00),
  B-09 fuse holder ($14.85), B-10 buck ($34.00), B-11 UBECs ($17.98), B-12
  XT60s ($21.98), B-13 XT30s ($8.58), B-14 splitters ($10.00), B-15 wire
  ($40.93), B-16 JST-XH kit ($7.99), B-17 JST-SH leads ($15.00), B-18 star
  board ($20.98), B-19 magnets ($6.99), B-20 steel washers ($7.99), B-21 M2.5
  screws ($10.97), B-22 TPU ($21.99), B-23 cable ties ($8.00), B-24 the 12 V
  node ($10.00); C-01 IMU ($29.50) and C-02 lidar ($62.10).
- **Also, the D064 body** (rows added 2026-10-08, prices estimated, not
  checked): B-25 2 × female XT60E-F for the tub's carrier ($2.00), B-26 M3 ×
  30 socket heads for the shelf posts ($4.00), B-27 M3 ISO 7380 button heads
  for the rail tabs and hangers ($6.00), B-28 Ø5 × 2.5 spacers, bore ≥ 2.6
  (or an M2.5 nut + washer each, $4.00), B-29 the 2 mm foam pad ($5.00), B-30
  the UART lead ($5.00), C-05 a right-angle USB-A cable for the lidar, plug
  ≤ 20 mm from the port face, overmold ≤ 14 × 7 ($8.00); and, if missing,
  X-20 a flat screwdriver, blade ≤ 5 mm, shaft ≥ 183 mm (200 recommended,
  $6.00). B-24 is now the 30.5 × 30.5 FPV-style power board (pick 9), its
  parts and leads ≤ 6.0 mm over the board (VERIFY).
- **Why now:** one leg works on the bench; everything here is needed to dock
  five legs and power the robot on its stand.
- **Cost:** $886.30 at the BOM's prices (phase B without B-02, plus C-01,
  C-02 and C-05; $34.00 of it the D064 rows' estimates). Waiting: B-02 hand servos ($54.00)
  for the hand (B25), C-04 camera ($42.45) for its bracket (B16).
- **Gate:** on arrival, the pack's size and the XT60 pair checked (the list
  at the top) and filed before the tub and the sled print.

## 8. PRINT: four more legs

- **What:** 4 × batch 2 without the blanks and without `coxa_yaw_base`.
- **Why now:** the bench leg's numbers are filed. Cut the tubes only once
  the tube length and the foot are decided (B95, ASSEMBLY_GUIDE 3.1).
- **Cost:** 310 g / 16.5 h.
- **Gate:** each leg calibrated on the jig (ASSEMBLY_GUIDE 2.11). These legs
  dock only once their bases exist (the hold).

## 9. PRINT: the body and the stand

- **What:** PRINT_PLAN batch 3 without the held bases: `body_deck`
  (v0.5), `bay_tub`, `bay_lid`, `bay_door`, `battery_sled`, `hub_shelf`,
  `avionics_tray` + `tray_latch_boss` + 2 × `tray_rail`, two latch
  cartridges (tray, door), 4 × `imu_grommet` (TPU). Then the stand cradle:
  `stand_base`, `stand_crown`, 2 × `stand_section`.
- **Why now:** the deck needs only step 3's numbers, but the tub and the sled
  need the pack and the XT60s in hand, and the stand holds the robot by the
  tub, so it prints after the tub.
- **Cost:** 242 g / 12.9 h (the tub, lid and door in PETG) + 232 g / 8.5 h
  (the stand, 0.3 mm layers).
- **Gate:** ASSEMBLY_GUIDE §4's checks: the tub hangs square, the door locks,
  the sled slides in and out, the tray latches and releases, the robot sits
  on the stand's pin.

## 10. PRINT: the carapace

- **What:** 5 × `shell_sector` + `shell_cap`, five latch cartridges.
- **Why now:** last: the robot is docked, wired and powered on its stand
  first (ASSEMBLY_GUIDE §5–§6), and the shell only retains.
- **Cost:** 152 g / 8.1 h (0.25 mm layers); B-19 magnets bought in step 7.
- **Gate:** every sector latched and on its washer, the cap pulled home.

## 11. PRINT: the four held bases, when the hold lifts

- **What:** 4 × `coxa_yaw_base` + 8 × `thumb_knob_m3`, from the tree that
  carries the owner's fix.
- **Why now:** only after the fix is chosen (an outboard hold-down, a deeper
  rib past the sockets, a thicker plate, or screw preload as a design load)
  and `./rocky.sh cad-check --derived --fem` passes for `coxa_yaw_base`.
- **Cost:** 59 g / 3.2 h today's geometry; the fix may change it.
- **Gate:** five legs docked; the robot stands on its own feet.

## Later, optional

The hand (B25: B-02, X-07 pins, the `hand_*` prints), the camera (C-04, once
its bracket exists, B16), voice (D-01, D-02), the X rows of the BOM, and the
printable extras in PRINT_PLAN's deferred list (`trim_cup`, the clips,
`whisker_shoe`, the charging dock once B14 is settled).
