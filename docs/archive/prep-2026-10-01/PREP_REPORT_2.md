# Body layout: second and final preparation report (read-only, 2026-10-02)

Round 2 of the measurements behind the owner's decisions in
`docs/BODY_LAYOUT_PROPOSAL.md` section 8 and decision 14 (the three small
boards). It builds on round 1 (`prep/PREP_REPORT.md`) and replaces it where
they differ. Nothing in the repo was changed.

Lengths in mm, volumes in mm³, angles in degrees. Body frame: z = leg z, deck
z −10 … −4, coxa plate −4 … 0, ground −118 at stance, +x east, leg 0 north.
Leg-local frame (section 1): +x outboard; a pose (tx, tz, tilt) is the coxa
base relative to docked, + tilt lifts the outboard end.

**Verdict labels.**
- **confirmed**: a second agent (verifier, skeptic or judge) re-measured it with its own scripts and agreed.
- **unverified (X only)**: one agent measured it; nobody re-checked it. This includes new findings made by a skeptic.
- **refuted (by X)**: a check disagreed. Both numbers are shown.
- **ESTIMATE**: an input or result that is a guess, not a measurement.

**Where the numbers come from.** Every number names a script. Folder
abbreviations (all under
`/tmp/claude-1000/-home-bitwisebard-Development-rocky/61c1816d-d6f0-4565-8ba4-46b722cb69ec/scratchpad/`):

| abbr. | folder | role |
|---|---|---|
| R1 | `prep/` | round 1 (its report and scripts) |
| G1, G2, G3 | `prep/gaps/1`, `/2`, `/3` | late round-1 gap runs |
| K1, K2, K3 | `prep2/gap1_skeptic`, `gap2_skeptic`, `gap3_skeptic` | skeptics of G1–G3 |
| M / MV | `prep2/i1_minimal` / `_verify` | I1 design "minimal" and its verifier |
| S / SV | `prep2/i1_seats` / `_verify` | I1 design "seats" and its verifier |
| Q / QV | `prep2/i1_sequence` / `_verify` | I1 design "sequence" and its verifier |
| J | `prep2/i1_judge` (report `prep2/I1_DOCK_OPTIONS.md`) | judge of the three I1 designs |
| L / LV | `prep2/q_lanes` / `_verify` | loom lanes |
| U / UV | `prep2/q_usb` / `_verify` | Pi USB plug room |
| B / BV | `prep2/q_optionb` / `_verify` | option B |
| C / CV | `prep2/q_connectors` / `_verify` | connector pass-through |
| O | `mycheck/` | the orchestrator's own check |

**Headline.**
1. **Today's leg cannot be docked or undocked** (I1). The judge recommends
   the "seats" fix with one graft; three frozen values change. Print the port
   coupons first.
2. **In stance the hook carries nothing.** The two thumbscrews and their
   inserts carry the stance load: 972 N per pair today, 568 with the seats
   fix. Today's coxa plate is over-stressed at the screw line (47.4 MPa
   against 35). The orchestrator's stance statics had the wrong sign.
3. **Decision 13:** the measurements support modelling the belly plus a hip
   clamp on the righter's commands, applied only in FALLEN and RIGHTED. They
   do not support a uniform hip limit at bay_w 53: no value clears that tub
   and keeps the cliff guard. Every leg-reach number assumes a 135 mm foot;
   the hand as the repo builds it today (B95) reaches 30.2 further and voids
   them.
4. **Decision 14:** drive the bus adapter over UART and put it face-down on
   the hub shelf (case i). With the Pi's USB end +x, only one USB-A port is
   usable, and only with a right-angle plug at 11 mm standoffs. At 13 mm no
   port is reliable.
5. **Decision 1:** option B as drawn is out. It survives only with the USB
   end +x, a reading of the 3 mm rule as a vertical margin, and a new
   side-entry hub board about 39 × 49.
6. **Decision 2:** the lanes no longer force the 8 mm layer. The 8 mm layer
   is still preferred for the hook margin and for tied looms.

---

## 1. The I1 leg port

### The fault (confirmed by M, MV, S, SV, Q, QV and J)

| item | value | source |
|---|---|---|
| hook L width at the bottom vs deck slot | 5.15 vs 4.2 | R1 `keepouts/dance_cspace.py`; J `j1_repro.py` |
| path from above to docked, hook alone, slot 4.2 or 4.6, tilt −30 … 90 | none, even at clearance 0 (sound test) | J `j1_repro.py` (`j1.log`); MV `s1_grid.py` + `s2_search.py` (`s2_repo_hook42.log`); S `c1b_sound.py`; Q `s03_repro.py` |
| hook alone, slot 5.2 | docks (control) | J `j1.log` |
| with today's 8 mm dowel posts, slot 5.2 or 7.0 | no path at clearance 0.10 (tilt −5 … 30, tx −3 … 5); at 0.05 inconclusive; at 0.30 the docked pose itself is blocked (dowel play 0.175) | J `j1_fine.log`; MV `s2_repo_post52.log`; M `r0b.log` |
| round 1's "no path at slot 7.0, clearance 0.05" | neither proven nor refuted: proven no path at 0.10 only | M `r0_reproduce.py`, MV, J |

So today's leg cannot be docked or undocked at all.

### The stance load path: the brief's sign is wrong (confirmed)

- In stance the foot (leg-local x 75) pushes the plate's outboard end UP.
  The plate pivots about its most inboard deck contact. The hook foot lies
  inboard of that, so it moves away from the deck and carries 0 N.
- Measured in the sim (settled 5-leg stand, `cfrc_int` on coxa_i): My
  +241.5 N·mm, Fz 2.82 N on the leg. Two agents agree: M `f1_stance_wrench.py`
  (`f1.log`) and Q `s01_port_wrench.py` / `s06_port_loads.py`.
- Hand statics (robot 2.7277 kg on 3 legs × 3, 26.76 N per leg, μ 0.5
  ESTIMATE), J `j2_statics.py` (`j2.log`); MV `f1_function.py`, SV
  `f3_hand.py` agree within 2%:

| design | fulcrum x | screw lever | screw pair, with friction / without | hook |
|---|---|---|---|---|
| today / minimal | −45.9 | 4.9 | 972 / 660 N (MV 654; SV 953–972) | 0 |
| seats | −49.55 | 8.55 | 568 / 390 N (SV 568.7 / 390.1) | 0 |
| sequence, knobs rigid | −46.0 | 5.0 | 953 / 648 N (QV: knobs 192–400 N each) | chevron 0–62 N (QV) |
| the brief's picture (hook at −51.85, deck contact at −10) | — | −41.85 | no equilibrium (negative force) | — |

- The hook works only for a hanging leg: 26.7 N at × 3 (J `j2.log`).
- The wording "the thumbscrews only clamp" (INTERFACES I1, frozen) is false
  for all three designs.
- Insert pull-out (ESTIMATE 400–900 N) and hand preload (ESTIMATE 150–1000 N
  per pair, depending on the agent) are unmeasured.

**Today's plate fails at the screw line** (confirmed: J `j3_section.py` →
`j3_today.log` 47.4 MPa; SV `f1_function.py` 47.5 MPa; S `f3_today_section.py`
47.5 MPa). Section at x −41: A 146.4 mm², I 195.2 mm⁴, M 4629 N·mm, SF 0.74
against 35 MPa. Without foot friction: 31.8 MPa, SF 1.10.

### The three fixes (each verified: docks and undocks, no fatal flaw)

| | minimal | **seats (+ graft)** | sequence |
|---|---|---|---|
| idea | slot 5.2 grown inboard; short foot 0.35 under the deck; 3 mm 10° tapered posts in tapered bores; tilt about 8°, drop, pivot | two 30° cone posts at (−32, ±17.3); one cone socket, one ±0.3 V socket; slot 6.2 grown outboard; plate bridges the slot; spine rib; hook at 7–10°, lower onto the cones | no posts; lower flat 3.34–5.0 outboard, slide in; inner L + new outer chevron under the deck corner; knob skirts take shear |
| smallest clearance on the path | 0.300 at docked (posts); 0.3035 en route on the deck's first-layer slot edge (MV `t1_min.log`); a 0.351 path exists (MV `m1_mypath.py`) | 3D 0.300 (lip end vs slot end, constant); in-plane 0.40 at tilt ≤ 10 (SV `c2_th10_040.json`), up to 0.495 (SV `c1_search.py`). The designer's "0.378 ceiling" is refuted (by SV): a solver artefact | 0.300 along the whole slide and docked; a zero-thickness corridor that holds only while the plate slides on the deck (QV `v05_cspace.py`, `v06_replay3d.py`) |
| docked play | x ±0.305, y ±0.30, yaw ±0.904 (MV `f1_function.py`); today ±0.175 / ±0.528 | nominal 0 (SV); real: 0 up to about 0.05 radial socket oversize; at 0.1 x ±0.05, yaw 0.08–0.10; at 0.2 ±0.15, 0.25 (SV `f2_relief.py`). Designer's "±0.11 becomes z only" refuted (by SV) | x/y ±0.15 (±0.30 with groove float), yaw ±0.51 (±1.0) (QV `v07_loads.py`) |
| shear, yaw stall 2.94 N·m | 77.4 N per post; root SF 1.04 (mid-flank) / 0.52 (tip) (MV) | 85 N per seat; SF 1.86–1.95 couple alone, 1.23–1.29 with friction + 30 N shove on one post (SV `f3_hand.py`) | 86.5 N per skirt, SF 1.97; 1.49 combined (QV; designer 1.58) |
| plate at the screw line, M 4629 | 46.2 MPa, SF 0.76 (J `j3_minimal.log`; MV 31.0 without friction) | 15.0 MPa, SF 2.33 (J `j3_graft.log`; SV 15.0) | 66.8 MPa, SF 0.52 (J `j3_sequence.log`; unverified, J only): Ø10 bores cut the section to 105 mm² |
| lowest hook z, path / any reachable pose | −12.45 / −13.49 (MV `e1_envelope.py`) | −13.30 / −13.735 (SV `e1_envelope.py`; analytic −13.757) | −12.30 / −12.367 (QV `v12_envelope.py`); designer's −13.25 not reproduced |
| margin to tub roof −18 / −15 (any pose) | 4.51 / 1.51 | 4.26 / 1.26 | 5.64 / 2.64 (QV, stations 1–4) |
| new under-deck parts | none | none | chevron at each deck corner, r 85.8–109.1 (Q `s15_envelope.py`) |
| print risk | low; chamfer the slot's bed-side edges (elephant foot sits on the bottleneck, MV) | medium; sockets on a supported face need a support blocker; the 0.2 relief is one layer on the support interface; V-socket wall exactly 1.200 (SV `p1_print.py`) | high: 223 mm² of 1.2 deck collars; supports on the chevron foot's 0.30 face (61.0 mm², QV `v08_print.py`); −y chevron arm unfused over the harness channel (fused fraction 0.40, QV `v02_attach.py`); knob seat 9.32 mm² (QV) |
| frozen values changed | 3 | 3 | 5 |

### The judge's recommendation: **seats, with the graft `seat_relief_x0` −42 → −46**

Source: J (`prep2/I1_DOCK_OPTIONS.md`).

- Why seats: it is the only design that also fixes the port's weak points.
  Play is nominally 0. Seat SF is 1.23–1.95. The rib lifts the plate from
  SF 0.74 to 2.33. Screw load falls 972 → 568 N per pair. It has the widest
  dock margin (0.40 in-plane at tilt ≤ 10).
- **The graft.** With the screws (x −41) outboard of the pad edge (x −42),
  the seats get 7.9% of the screw preload. With the pads ending at x −46
  they get 30.1% (J `j6_preload.py` → `j6.log`; SV's finding, J's numbers;
  unverified beyond J). The graft only removes material: 34.45 mm³, and graft
  minus seats is 0 mm³ (J `j4_subset.py` → `j4.log`). So every seats path
  keeps its clearance.
- 3D replay on the graft solids (J `r1_replay3d.py`): the 10° hand path,
  271 poses, 0 mm³, minimum 0.300 (0.305 with the lip ends trimmed); SV's
  0.40 path, 358 poses, 0 mm³, 0.4006 trimmed. Docked: 0 mm³ with seat
  contact. Unverified (J only), but J replays SV-verified paths on a subset
  solid.
- Cost of the graft: the stance fulcrum bears on 40.5 mm² of pads, 13.4 MPa
  mean in the design case (J `j6.log`, unverified). The coupon compares −42
  and −46.
- Fall-back: if the seat coupon fails (rock, or play > 0.15 at every socket
  offset), use minimal plus the seats' spine rib, bigger posts (dowel_d 5,
  plate 47 wide; owner call) and a 0.5 × 45° chamfer on the slot's bottom
  edges.

**Docking motion with seats** (S `c12_radial_path.py`, verified by SV):
present the plate at 7–10° outboard-end-up, about 10.8 above docked; come in
radially and stop about 1.6 outboard; lower the lip through the slot; let the
outboard end down onto the cones; fit the knobs. Keep the hooked tilt ≤ 10 at
leg 4 (the Pi touches at 14.8–15°: S `w1_sweep.py`, SV `w3_sweep_leg4_1.log`
0.026 mm³). The leg's own carapace sector must be off (up to 1010.8 mm³,
S/SV). Unclamped, the leg rests on the deck's pentagon vertex (−10, 0, −4)
and the hook foot, tilted 2.28°, sockets 0.275 off the cones (SV
`h1_hooked.py`). At the 10° hooked pose the foot reaches only 0.53 under the
deck (SV; the designer said 2.4: refuted).

### What the owner signs off (J section 4)

| item | old → new |
|---|---|
| **dowel_xy** (frozen) | [(−28, 19), (−28, −19)] → seat_xy [(−32, 17.3), (−32, −17.3)] |
| **hook_slot_x** (frozen) | −48.0 → −47.0 |
| **hook_slot_w** (frozen) | 4.2 → 6.2 (inboard edge stays −50.1; outboard edge −45.9 → −43.9) |
| dowel_d, dowel_engage | 4.0, 8.0 → removed |
| new seat params | seat_r 2.9, seat_half_angle 30, seat_h 3.0, seat_vee_len 0, seat_vee_slack 0.3, seat_roof 0.6, seat_mouth 0.3, seat_relief 0.2, **seat_relief_x0 −46.0** |
| new hook params (values unchanged) | hook_stem_gap 0.55, hook_foot_gap 1.5, plate_ext_half_w 18.0 |
| plate | runs inboard to the stem (x −49.55) for \|y\| ≤ 18 (replaces the 2 mm reach) |
| coxa_yaw_base | spine rib x −46 … −38, \|y\| ≤ 8, z 0 … 6 |
| insertion procedure | 7–10° with a radial approach (docs say ~15); own carapace sector off; leg 4 hooked tilt ≤ 10 |
| coupons | port coupon plate patch x −57 … −11 → x −46 … −11; fit-ladder row D 3.8/4.2/4.6 → 5.8/6.2/6.6 + a seat-fit row |
| open fixes to sign with it | V-socket wall 1.200: seat_mouth 0.3 → 0.2 or seat_xy \|y\| 17.3 → 17.1 (not re-proven); relief 0.2 → 0.4 (only removes material; screw-line section not re-checked) |
| unchanged | thumbscrew_xy (−41, ±17), hook_lip_w 24, hook_lip_t 3.0, hook_foot 3.5, cable cutout, heat-set pockets |

Also for sign-off, outside the fix: **the stance hold-down.** The
thumbscrews and inserts carry 568 N per pair. If the insert coupon is below
about 300 N per insert, an outboard hold-down is needed (owner decision).

### Print first (J section 6)

1. **Port coupon pair** (deck patch + plate patch x −46 … −11), two plates:
   seat_relief_x0 −46 and −42. Go: the L passes the 6.2 slot by hand at
   7–10°, lowers onto the cones and centres; finger-tight, no rock and < 0.05
   play on a dial. No-go: binding → print 6.6; play > 0.15 at every offset →
   minimal fall-back. The repo's coupon cannot do this today: its patch
   reaches x −57 and penetrates the deck coupon by 0.91 on any dance (MV
   `c2_coupon2d.py`; M `c1_coupon.py` 146.7 mm³).
2. **Seat-fit row:** socket radial offsets 0 / +0.1 / +0.2; pick the one with
   no rock and no light under the pads (0.2 feeler).
3. **Socket print**, plate-down with a support blocker in both sockets.
4. **Insert pull-out (toward the plate) and hand preload.** Go: ≥ 300 N per
   insert and ≥ 285 N preload.
5. **Cone post twist:** 85 N per seat, then 128 N on one post.
6. **Release test** at 10°: rests on the vertex + hook, centres when the
   knobs go in.
7. **20 dock/undock cycles.**

---

## 2. Decision 13 (legs vs the tub), final

### Exposure with no gate (the belly modelled, nothing else)

| case | FALLEN leg-tub contacts (200 drops) | max depth | outcome | source / verdict |
|---|---|---|---|---|
| bay_w 50, massless (v1) | 6/200 (legs 1, 2, 3) | 2.36 | 197/200 | G2 `righting_instrumented.py` + K2 `sk_run.py`: confirmed |
| bay_w 50, with the battery mass (v1m) | **12/200** (legs 1, 2), 319 ticks | **3.06** | 196/200 | G2 + K2 seed for seed: confirmed. Round 1's headline 6/200 / 2.36 understates the real robot |
| bay_w 50, v1m, datasheet servo model | 14/200 FALLEN + 2/200 RIGHTED (legs 1–3) | 4.89 | 198/200 | K2 `sk_run2.py --servo nominal`: unverified (K2 only) |
| bay_w 53, massless | 5/200 (legs 1, 2, 3) | 2.82 | 197/200 | G1 `righting_pin.py`; outcome confirmed by K1 `right_sk.py`, contact count G1 only |
| drop fixture at t ≈ 0 (no rule changes it) | v1 10/200; v1m 9/200; datasheet 11/200 | 1.97 / 2.66 / 2.08 | — | G2, K2 |

The mechanism (refuted, by K2): G2 said the contacts are servo lag between
clear targets (knee lag 20°). K2 measured knee error 32.6–40.8° and found a
pinch: the knee is torque-saturated and loaded through the floor while the
hip, still commanded to −66 … −70 at 2.94 N·m, drives the folded leg under the
settling keel (torso 67 → 56 mm, tub normal force up to 46.9 N; K2
`sk_probe18.py`). In 9 of the 12 v1m contact seeds the commanded pose itself
is inside the tub (K2 `sk_lag.py`).

### The four options

**(a) Model the belly.** Belly geoms in the MJCF and the tub in
`pebble_feasibility`. It is needed by every other option, because without it
nothing can be measured. Round 1 costs (confirmed): fingerprint
87215110e9c4 → 777fdb4eda14 (v1) / 47435187271f (v1m) / 4f77d216a700 (v2);
geom ids shift +2; `recover7` warns; SELF_CONTACT then gates the gesture audit
and the cockpit studio, **not the righter**, `save_keyframe_gesture()` alone or
the playground `check`. On its own it leaves the exposure above.

**(b) A uniform hip limit (joint range or servo limit, every state).**

| | bay_w 50 | bay_w 53 |
|---|---|---|
| zero-margin cut, legs 1/4, sim capsules | −51.05 (R1 confirmed; G3 +0.08 at −51.05) | −49.72 (K1 `cut_scan.py`, `kin_cut.py`; G1 −49.70): confirmed |
| same, proposal radii (r 15/12) | −49.25 (G1 summary; unverified) | −47.91 (K1; G1 −47.90): confirmed |
| same, within 3 mm | −48.40 (G1, sim) | −47.00 (G1; K1 −47.004): confirmed |
| cliff guard ceiling at h 118 | −47.91 | −47.91 |
| window under the ceiling, no overshoot (arithmetic) | 3.14 (sim) / 1.34 (prop) / 0.49 (3 mm) | **1.81 (sim) / 0.00 (prop) / none (3 mm)** (K1) |
| righting at the cut | 199/200 at −51.05 (R1, G1) | 199/200 at −48.5, −48.0, −46.5 (G1; K1 199/200 at −48.0): confirmed |
| FALLEN ticks pinned at the stop | 44.76% at −51.05 | 45.73% at −48.0 (G1; K1 45.70): confirmed |
| stairs h20 shortfall | 17.1 at −51.05 (R1) | 38.9 at −48.0 (G1; K1 38.92): confirmed |
| BRACE end delta | 1.1 at −51.05 (R1) | 3.30 at −48.0 (G1; K1 3.30): confirmed |
| reach lost (planar workspace) | 11.8% at −51.05 (R1, G1) | 13.7% at −48.0 (G1 `reach_loss_g.py`; unverified) |
| gait, turns, ramps, gestures clipped | 0 | 0 (G1 `cut_cost_g.py`) |

- **The cliff ceiling** (confirmed): `sim/playground.py` declares `probe_out`
  only when the measured foot is 26.5 deep (PROBE_MAX 30 − PROBE_LEAD 3.5).
  Analytically L* = asin((L3 − Z_HIP − h − 26.5)/L2) = −47.91 at h 118 (K1
  `cliff_kin.py`). Measured: void fired and held 12/12 at −48.5, −48.0,
  −47.75; 0/12 at −47.5, −47.0, −46.5, and the robot walks off the table
  (end tilt > 10° in 7–8/12, up to 37.2°) (G1 `pg_wrap_g.py`; K1
  `cliff_sk.py`).
- The ceiling moves about 0.9° per mm of body height: h 115 → −45.28,
  h 120 → −49.74, h 125 → −54.67 (K1 `cliff_kin.py`; unverified, K1 only).
  `gait.h` is live-tunable. At h 120 no window exists even at bay_w 50 with
  3 mm (arithmetic).
- MJCF stop overshoot grows with the cut: 1.09/1.15 at −51.05, 1.44/1.67 at
  −48.0, 1.72/1.94 at −46.5 (FALLEN/NORMAL, 200 seeds; G1, K1 `right_sk.py`):
  confirmed. The drop landing overshoots 2.11–2.25 at every limit (K1 only).
- **On hardware there is no stop at the cut.** With a driver-style command
  clamp (MJCF −70, servo force range 2.94 N·m), impacts back-drive the hip
  10.6–10.9 past −48, yet 0 tub contacts and a minimum of 7.30 over 200
  episodes (K1 `right_sk.py` drv/gcut runs; unverified, K1 only). With the
  driver clamp at −48 the cliff reading clears 26.5 by only 0.11–0.18 (K1).
- At −48.5 a stairs step-down reads as a void and the robot safe-stops (G1
  `pg_terrain`; one trial, unverified).
- **Per-leg limits** (−69, −47.5, −58.3, −58.3, −47.5): the cliff guard holds
  12/12 when leg 2 leads, but only 4/12 (late, by legs 0 or 2) when leg 1
  leads (K1 `cliff_sk.py`, lead-leg runs). G1's "per-leg would not rescue it"
  is refuted for approaches led by legs 0, 2 and 3 (K1); legs 1/4 bind both.

**(c) A keep-out on the supervisor's targets.**

| rule (projected each tick, legs 1–4) | v1 contacts | v1m contacts | outcome | FALLEN / RIGHTED ticks clipped | source / verdict |
|---|---|---|---|---|---|
| box hip < −51.05 AND knee < −90.75, nearer edge | 1/200, 1.33 | 3/200, 1.85 | 196/200 (v1 loses 90; p 1.0), v1m 0 discordant | 9.1 / 1.7% (v1); 8.0 / 1.6% (v1m) | G2 `righting_keepout.py`, K2 `sk_run.py box`: confirmed |
| same box + 3 mm (−48.35 / −89.15) | — | 2/200, 3.79 | 196/200 | — | G2, K2: confirmed |
| exact reach-set model | 2/200, 1.44 | 6/200 (G2 1.57; K2 2.60) | 196–197/200, 0 discordant | 0.41 / 0.22% | G2; K2 count matches, depth depends on the projection: cannot tell |
| **box, always resolved by raising the hip, measured knee also triggers, FALLEN/RIGHTED only (boxHM)** | 0/200 (all states; 196/200, loses 90) | **0/200**, 196/200, 0 discordant | unchanged | 8.02 / 2.65% (v1m); min clearance 4.12 (v1m), 7.71 (v1) | K2 `sk_run3.py`: unverified (K2 only) |
| boxHM, datasheet servo | — | 1/200 (seed 90, 1.13) | — | — | K2 only |

G2's conclusion "no keep-out on targets can be contact-free" is **refuted (by
K2)**: it failed because its projection raised the knee, which the saturated
knee cannot follow. A hip-first projection is contact-free in the sim, but
only K2 measured it. Seed 90 under the datasheet servo is a drop-fixture jam
(leg 2 self-contact at 800–1080 N, hip −71.8) that no target-side rule can
prevent (K2 `sk_probe18.py --seed 90`).

**(d) A hip clamp on the righter's commands.**

| form | bay_w | contacts | outcome | ticks clipped | min clearance | source / verdict |
|---|---|---|---|---|---|---|
| clamp hip ≥ −51.05, legs 1–4, every state | 50 | 0/200 v1m and v1 | v1m 196/200, 0 discordant; v1 195/200 (loses 90, 112; p 0.5); ramp median 0.00 s, worst +1.97 s | FALLEN 38.55% (v1m) / 42.53% (v1); RIGHTED 11.45 / 10.26%; BRACE 14.0 / 2.8% | 6.92 FALLEN, 6.12 RIGHTED | G2 `run_clamp.sh`, K2 `sk_run.py clamp`: confirmed |
| same, **FALLEN/RIGHTED only** | 50 | 0/200 v1m and v1 | identical to the all-state clamp | BRACE and RECOVER 0 | BRACE ≥ 55.6 | K2 `sk_run.py clamp --states FR`: unverified (K2 only) |
| same, datasheet servo | 50 | 0/200 | 198/200, 0 discordant | FALLEN 37.42% | 3.33 | K2 only |
| righter-only floor −48, action range remapped, clamp only in FALLEN/RIGHTED | 53 | 0/200 in every phase | 199/200 (fail 147, as the cut tree); paired worst +0.78 s | — | 7.30 (kinematic) | K1 `right_sk.py` (`rt_gcut_480_*`); cliff 12/12 identical to uncut (`cl_gated_480`): unverified (K1 only) |
| clip only, uncut map, clamp −49.7 | 53 | 0 FALLEN/RIGHTED | 195/200 (seeds 40, 90 newly fail; p ≈ 0.125) | — | — | K1 only |

- Under every rule the measured FALLEN hip still reaches −71.9 … −72.0: a
  command clamp does not keep the measured hip above the clamp value. The
  tub stays clear because of the poses the legs take (K2 `sk_near.py`, K1).
- The FALLEN/RIGHTED-only scope leaves NORMAL at −70, so the cliff guard,
  BRACE, the stance probe and stairs are untouched. The clamp value is not
  bound by the cliff ceiling, so it can carry margin.
- **The clamp value at bay_w 53:** −51.05 leaves legs 1/4 kinematically 1.42
  into the bay_w 53 tub (G3 `g3_reach.py`; LV `m5_reach.py`: 2 poses at
  −0.29 on a 2.5° grid; both agree it is not clear). It needs > −50.0 (G3) or
  ≥ −49.72 (K1).

### What each option costs, and what the measurements support

| option | contact-free on the fall path? | cost | bay_w 50 | bay_w 53 |
|---|---|---|---|---|
| (a) model only | no: 12/200 at 3.06 (v1m) | fingerprint change; the righter is not gated | as measured | 5/200 at 2.82 (massless) |
| (b) uniform limit | yes in the sim | stairs 17–39 short, BRACE 1–3.3, reach 11.8–13.7% lost, the cliff guard sits on a 0.5–3° window that moves 0.9°/mm with gait.h | possible, window 0.49–3.14 | **ruled out**: window 0 with the proposal radii, none with 3 mm |
| (c) target keep-out, G2's form | no (1–6/200) | ~8–9% of FALLEN ticks | refuted as contact-free | — |
| (c') hip-first keep-out (boxHM) | yes in the sim (K2 only) | ~8% FALLEN, ~2.7% RIGHTED clipped; needs servo read-back | unverified | not measured |
| **(d) FALLEN/RIGHTED clamp** | **yes**, at both widths and under the datasheet servo | ~38–43% of FALLEN ticks clipped, no outcome or timing change; a supervisor code change (a separate righter floor) | −51.05: confirmed (all-state), FR-only unverified | floor −48: unverified (K1 only); −51.05 not enough |

**Recommendation (orchestrator synthesis from the measurements):**
- Model the belly (a), as the precondition.
- Then the clamp on the righter's commands (d): in the supervisor, only in
  FALLEN and RIGHTED, with the action range remapped. Not a servo limit and
  not the joint range. Keep the joint range at −70.
- The hip-first keep-out (c') is the near-equal alternative at about a
  quarter of the clipping. It rests on one agent's runs.
- Do not take a uniform cut at bay_w 53. At bay_w 50 it is possible but
  fragile.

**Precondition for every option: B95.** The hand as the repo builds it puts
the sole 165.2 below the knee axis, 30.2 past l3. With it (K3
`s8_hand_fast.py`; unverified, K3 only):
- at −51.05 legs 1–4 pass 17.5–18.7 through the tub;
- clearing the tub needs ≥ −24 … −20 (legs 1/4) and ≥ −32 (legs 2/3);
- clearing the shelf needs ≥ −40.

Every reach and clamp number above assumes a 135 mm foot (capsules or B25's
plain foot). Settle B95 first, then derive the clamp value for the end
effector that is fitted.

---

## 3. Decision 14 (the three boards), final

### Pi to carapace, real Pi 5 STEP (U `s8_stack_gap.py`, `s9c_gap_mesh.py`; UV `v3_gap.py`: confirmed)

| standoffs | USB end +x, 3D gap (±0.3 float) | USB end −x, 3D gap | 3 mm rule |
|---|---|---|---|
| 11 | 6.84 (6.63–7.05); UV 6.86 | **3.56 (3.41–3.72)**; UV 3.58 | both pass. Round 1's "−x fails even at 11" is refuted (by U, UV): the box model's bbox is wider than the stack's top |
| 13 | 5.45 (5.24–5.67); UV 5.47 | 2.17 (2.03–2.32); UV 2.20 | −x fails |

The real STEP's middle USB stack stands 17.53 over the PCB underside (15.93
over the board top) and overhangs the board edge by 3.00 (U `s1_ports.py`;
UV `v1_ports.py`). `part_avionics` checks only to the board top. The −x pass
at 11 has 0.41 margin, and the Active Cooler is not modelled.

### USB plug room, USB end +x (U `s3_plugroom.py`, `s10_sections.py`, `s11_short_ra.py`; UV `v2_plug.py`, `v4_plug.py`, `v5_profile.py`: confirmed)

| port | 11 mm standoffs | 13 mm standoffs |
|---|---|---|
| **lower, middle stack** (next to the RJ45; body face x 55.50, y +1.0, centre z 20.87) | straight 35 plug: no (at most 32.65, a bare 12 × 4.5 shell, then leg 4's loom keep-out). **Right-angle ≤ 20 long, overmold ≤ 14 × 7: 21.88 (21.58 worst float)**; 16 × 8 marginal 20.23 (19.93). Cable leaves down (15.9–18.6) or south (59–61); north 1.3–4.3 (leg 4 yaw servo), up 0–2.3 (carapace) | 12 × 4.5: 20.23 (19.93 worst float); 16 × 8: 18.20. Only a right-angle body ≤ 17.9 (16 × 8) fits, zero margin at the worst float |
| upper, middle | 11.38 … 8.40 (carapace) | 9.35 … 6.33 |
| outer stack, both | 11.78 … 10.80 (leg 4's coxa base) | same |

- The fork + hip servos of legs 3/4 (yaw −40 … 40) are never the first limit
  (closest 43.85) (U, UV).
- The carapace roof over the middle stack is in 1 mm tiers. A plug whose top
  is ≤ 2.6 above the port centre runs to 32.65 at 11 mm (UV `v5_profile.py`;
  unverified, UV only).
- USB end −x at 11: the lower middle port has > 150 straight room; the
  USB-C power port then faces the tray bulkhead (3.40), but the plan feeds 5 V
  through GPIO (U `s4_other.py`, UV).
- With the Pi's USB end facing a leg's port (leg 4 for +x, leg 1 for −x), the
  docked leg module comes to \|x\| 66.08 within the board's y band and z
  17–33: 10.6 past the drawing's 3 mm overhang, 5.9 during the minimal dance
  (MV `u1.log`; unverified, MV only). That is less than a USB-A overmold
  (ESTIMATE 15–30). Plugs at that end must leave down or south.

**The USB budget.** Loads: the bus adapter, the D500 lidar's USB adapter
(phase C), an optional USB mic (D-02) and an optional Wi-Fi dongle (X-04)
(UV, from `bom/BOM.csv` and PERCEPTION_PLAN). At +x/11 one port works; at 13
none does reliably.

### The options

| option | Pi | adapter | buck, UBEC | smallest clearance | status |
|---|---|---|---|---|---|
| **pi13 + case i** | 13 mm; carapace 3D +x 5.45 | shelf, face-down at (−19.8, 30.8), z −54.4 … −39.3 | tray: buck (−31, 6, turned), UBEC (−20, −12), 0.000 mm³ (R1 confirmed) | crown plate 1.00 (ESTIMATE plate); leg-0 latch column 1.38; posts 0.57 / 0.59 / 0.89 | **static geometry confirmed** (G3 `b4b_pack.py` + `g3_verify.py`; K3 `s1_clear.py`: 0 mm³ at bay_w 50 and 53). But at 13 mm no USB port takes the adapter's cable reliably → needs UART |
| **offtray** (case iii) | 11 mm; carapace 3D +x 6.84, −x 3.56 | shelf, face-down at (−20.2, 28.8) | shelf: buck face-down (35.8, 26.2), UBEC face-down (−1.8, 55.8) | bay_w 50: star to post (50, 34) 0.80, crown 1.00, UBEC to latch column 1.08, star to lanes 1.20 | bay_w 50: confirmed (G3; K3 0 mm³, 0.80). bay_w 53: G3 "no arrangement" **refuted (by K3 `s5_case3_bw53.py`)**: the same arrangement moved 0.70 north gives 0 mm³ and a 0.50 smallest gap (0.71 at +0.95 against drops +0); unverified (K3 only) |
| pi18 | 18.5 mm | tray | tray | −x into the carapace 0.08 mm³; +x 3D 2.82 | fails the 3 mm rule (R1 confirmed) |
| hybrid (tray grown 30 north) | 11 mm | tray | shelf | carapace lowering 192.5 mm³ | refuted in round 1; the cheap fixes the critic listed (notched corners, no north cradle wall) were not measured |

Round 1's case i at 3.00 is **refuted (by G3, confirmed by K3)**: as packed it
puts 251.7 mm³ into the bay_w 53 cradle walls, the 14 AWG crosses the star for
24.0, and the adapter sits on the crown plate (G3 `g3_verify.py` →
`ver_doc_i53.log`).

**Shared caveats for any shelf option** (all from K3; unverified, K3 only):
- **Leg reach.** With today's limits, legs 0, 1 and 4 reach the boards and
  posts by up to 9.71 / 11.22 (G3 `g3_reach.py`; confirmed by K3 with
  capsules). Under the −51.05 cut every board clears by ≥ 13.46 with capsules
  (G3). A CAD plain foot (l3 135, ESTIMATE) already clears by 6.59 at −58.5
  (K3). With the B95 hand: leg 0 is 6.30
  into the PDB, leg 1 is 7.55 into the adapter. Clearing needs a limit
  ≥ −40 (K3 `s8_hand_fast.py`).
- **Stand.** The 1.00 crown margin goes with +1.0 of plate height or a
  1.42–1.47° roll. The cradle allows 2.29°. Joint-box poses put the CoM
  15–17 beyond the tub's north-bottom edge (K3 `s7_roll_com.py`, `s3_feed_stand.py`).
- **Service.** On the stand, 3 of the 4 adapter screws are blocked from below,
  and the shelf cannot be lowered (15163 mm³ into the crown plate), so hub
  service means taking the robot off the stand (K3 `s4_service.py`,
  `s5b_removal.py`).
- The down-facing boards reach z −54.4, below the z −38 shelf box that
  righting and rubble were run with (G3).
- With the repo's 8 mm star posts the star top is 0.90 under the deck (K3
  `s6_lanes_star.py`).

### Where the bus adapter goes, and how it connects

- **On the hub shelf, face-down** (case i with pi13, or case iii with Pi 11).
  It does not fit on the tray at any Pi height to 20 mm with the IMU where it
  is (B108).
- **Drive it over UART, not USB** (U, confirmed by UV from the Waveshare
  schematic and wiki):
  - H2 header (1 GND, 2 RXD, 3 TXD), jumper H4 to A;
  - 3.3 V buffers (SN74LVC1G125/126 on an AMS1117-3.3);
  - direction switching in hardware, no echo;
  - `rocky_driver.SerialTransport` takes any port path, so only the
    `/dev/ttyACM0` defaults change (driver, ros2 xacro / `rocky_system.cpp`,
    `hw_bridge_node.py`, bench scripts, docs).
  - The header has 30.42 / 28.42 of headroom at 11 / 13 (U `s4_other.py`;
    UV 30.45 / 28.45).
- UART wiring traps:
  - The signal wires must cross: H2 RXD → Pi pin 10, H2 TXD → pin 8.
    A straight 1:1 lead swaps them (UV, schematic).
  - Pin 6 (GND) is wanted by both the buck's 5 V feed (2/4/6) and a 3-pin
    UART housing (6/8/10). Take GND from pin 9 or 14. The repo pin map
    assigns no buck pins (U; UV "cannot tell").
  - In UART mode the adapter's logic runs only from its 12 V input, so its
    RX is unpowered until servo power is on (UV).
- Pi side, unverified: `dtparam=uart0=on` → `/dev/ttyAMA0` on GPIO 14/15,
  1 Mbaud on the RP1.

### The Pi's USB end and its plug

- **USB end +x on 11 mm standoffs.** Use the lower middle port for the lidar
  adapter with a right-angle plug ≤ 20 long, overmold ≤ 14 × 7, cable down or
  south. Leave nothing plugged that points at leg 4.
- The mic or Wi-Fi need a cable or hub, or the lidar moves to a Pi UART
  (GPIO 4/5 or 12/13; D500 logic level unchecked) (UV only).
- **USB end −x at 11** is back on the table (3.56 3D). It frees straight
  plugs, but the margin is 0.41 and the Active Cooler is unmodelled.
- **13 mm standoffs** (pi13) leave no reliable USB-A port: UART for the
  adapter is then mandatory.

### What passes the deck

| item | opening | result | source / verdict |
|---|---|---|---|
| Pi → adapter, UART | Ø10 grommet (0, 50) | 3 Dupont leads pass | R1; K3 |
| Pi → adapter, USB-C | Ø10 | no: overmold 12.35 × 6.5 needs Ø13.91–13.96 | C `s2_slab.py`; K3 `s4_service.py`: confirmed |
| tray trunk (XHP-5 both ends, 22 AWG) and tray XT30 | Ø10 | no plug passes with leads: XT30U Ø11.42–11.45 (Ø10.92 with its chamfers, ESTIMATE); XHP-5 + folded leads Ø11.47–12.88 | C, CV `v2_pass.py`: confirmed |
| same, routed before termination | Ø10 | yes: loose bundle Ø5.11, Ø6.36 with one SXH pin passing (wire ODs ESTIMATE) | C `b1_bundle.py`; CV |
| same, plugs on | slot 12.5 (E-W) × 7.5, r 1 | XT30 1.15/side; XHP-5 22 AWG two-layer stack 0.36/side (CV; C said 0.75: refuted), single-layer fold (8.0 wide) −0.25 (fails); gap 4.05 to option A's holes, 4.85 to today's (0, 40) hole (CV; C said 6.15) | C `s5_rounded.py`, CV `v3_fix.py` |
| J6 foot-switch lines (star → Pi GPIO) | (0, 50) | cross here, not in the hole table; the tray bulkhead has no connector for them, so tray removal is blocked by them | CV (unverified, CV only); C noted the hole-table gap |
| a cable straight down from (0, 50) | — | cuts the PDB lead halo (158 mm³) and the shelf plate (37.7); a route round the plate's north edge exists for cables ≤ 9.42 | K3 `s4_cable.py` (unverified) |

---

## 4. Decisions 1, 2, 8 and 10

### Decision 1: option B's status (B `ob1_lift.py` … `ob5b_holes.py`; BV `v1_lift.py` … `v8_carapace_cols.py`)

Lift is measured from today's centred tray (plate 3.40 over the deck top). The
B hub ceiling is 3.10 + lift (the plate bottom at rest).

| item | value | verdict |
|---|---|---|
| B as drawn (lift 18) | Pi into the carapace 492.2 (+x) / 492.3 (−x) mm³ | confirmed: **out** |
| USB end −x | vertical lift limit 4.15 (STACK16) / 1.15 (with the 3 mm overhang); 3D with the overhang 2.87 at lift 0 (limit −0.17); hub ceiling 4.25 | confirmed: **out** |
| USB end +x, vertical 3 mm margin | lift ≤ 11.10 floated (11.40 centred); hub ceiling **14.20** (12.00 under the IMU-nut sweep) | confirmed |
| USB end +x, 3D gap ≥ 3, Pi 5 drawing model | lift ≤ 8.28 (BV 8.27); ceiling 11.38 / 9.18 | confirmed |
| USB end +x, 3D gap ≥ 3, STACK16 + overhang | lift ≤ 6.12; ceiling 9.22 / 7.02 | confirmed |
| 13 mm standoffs | every limit 2.00 lower | confirmed |
| tray alone (26 bulkhead) | 3D ≥ 3 only to lift 14.87 | confirmed (BV bracket 14–15) |
| vertical B5B-XH star | 13.90 mated (no lead), 18.9–21.9 with a lead bend: −0.70 or worse | confirmed: never fits |
| side-entry S5B/S6B-XH star on 2.5 standoffs on the deck | 10.20; room 3.00 (0.80 under the nuts) vertical; 0.18 drawing-3D; −1.98 overhang-3D | confirmed |
| 30.5 PDB, flat-soldered 14 AWG | 8.30 (ESTIMATE); room 4.90 / 2.70 | confirmed |
| side-entry star board for J0 … J6 | 38.9 × 49.2 (halo 54.5 × 57.0 with a 5 mm lead); the 40 × 30 perfboard cannot carry it | confirmed |
| layout at lift 11.10, lead 5 | 0 mm³; coxa bases 1.50, yaw servos 16.05, forks 38.0, rails 9.06, nuts slid +16 1.80, PDB to strike post 0.70 (ESTIMATE post) | confirmed. With an 8 mm J0 lead: 33.5 mm³ into coxa base 0 |
| B rail height at lift 11.10 | 17.8 (B) | **refuted (by BV)**: 19.10 = 8 + lift; gaps unchanged |
| proposal holes (0, 52) and (29, 14) | collide with the hub | confirmed |
| leg-0 alternatives (±40, 52 … 56) | — | **refuted (by BV)**: under the carapace skirt (7.6–56.1 mm³), except (−40, 52) at 1.15; (±34, 46) only with lead ≤ 5 |
| grommet alternatives (36, −36), (20 … 26, −44) | clear of the hub | cannot tell (BV): they sit over the tub, 5 mm above its roof |

**B survives only with all of these:**
- the USB end fixed +x;
- the owner accepting the vertical-margin reading of the 3 mm rule (or the
  real-port 3D reading);
- tray_lift about 11 (not 18), rails 19.10;
- a new side-entry star board about 39 × 49 plus a 30.5 PDB, both on
  standoffs straight on the deck (no core plate);
- J0's lead turned within about 5;
- the leg-0 pass-through and the 14 AWG grommet moved;
- no decision-14 board under the tray.

Under the strict reading (3D with the overhang) no connectorised star fits.
The B lift limit with the real Pi STEP (section 3) is unmeasured.

### Decision 2: the lanes and the layer depth (L, LV: confirmed unless marked)

- **The lanes clear the new hook envelope.** Gaps 1.45 (A/B) and 1.75 (C) in
  both envelope modes and both layers (L `l1_measure.py`, `v1_verify.py`; LV
  `m1_lanes.py` 1.446 / 1.755). Boss clearance 0.60 (the proposal says 0.5).
  "1 mm wider touches: 15 mm³" is the 5 mm layer (15.03); the 8 mm layer is
  26.31 (L `l5_extras.py`; LV `m3_extras.py`).
- **The real loom per leg drop:** 7 conductors (2 × 20 AWG, 1 × 24, 4 × 26;
  WIRING_HARNESS), 14.17 mm² at nominal ODs (L `l3_bundle.py`; LV `m2_pack.py`).
  The proposal's 11 × 5.5 (60.5 mm²) is about 4.3 × the real bundle.
  - Bare in 5 × 7: fits for every OD set (confirmed).
  - Tied (1 mm strap, ESTIMATE): needs 8 × 7 (confirmed).
  - **5 × 4:** L said impossible (best 6.0). **Refuted (by LV
    `m2_pack.py`, `m2b_show.py`):** 5.068 needed at nominal ODs, and a 5 × 4
    packing exists at the −0.1 OD tolerance. Marginal, not impossible.
    7 × 4 fits every set (confirmed).
- **Recommended lane W8**, 8 × 7: A x ±45 … 53, y −38 … 12; B x ±45 … 59.8,
  y −38 … −30; C x ±51.8 … 59.8, y −53.43 … −30; z −17 … −10. 0 mm³ against
  every keep-out. Margins: tub/bosses 1.00, hook 1.755, coxa bases 2.10,
  latch column 4.37 (L `l4_variants.py`; LV `m1_lanes.py`). The same route in
  the 5 mm layer (z −14 … −10, 7–8 × 4) clears with the same gaps (L
  `l4_variants.py` W5/P5w7; LV).
- **Constraints the build must write down:**
  - The A → C jog must sit north of y −38; south of it, the lane hits the
    hook by 10.92 mm³ and the coxa base by 4.23 (LV only).
  - The wires climb inside the drop keep-out box. North of it they hit the
    bay_w 53 tub by 65.58 (8 mm) / 72.86 (5 mm) (L; LV arithmetic).
  - The loom leaves the shelf through its south face. Along the shelf's edge
    it hits the posts by 153.57 (L, LV).
  - Nothing holds the loom inside its lane against a 1.45–1.75 hook margin.
- **A 10 mm layer is not needed** (L `l2_corridor.py`; LV).
- **Hook margin to the roof with the chosen I1 fix (seats):** 4.70 / 4.26
  (path / any pose) at −18; 1.70 / 1.26 at −15 (S `e1_envelope.py`; SV). The
  5 mm layer is no longer an overlap (round 1 had −0.09 … −0.16 with the old
  5.2 slot), but it is thin.
- **Verdict:** the lanes no longer force 8 mm on wire section alone (a bare
  loom fits 7 × 4). 8 mm is still preferred: 4.26 vs 1.26 of hook margin,
  tied looms fit, and round 1's "no difference" for righting and rubble
  holds. 5 mm buys 3 mm of ground clearance (65.15 vs 62.15 settled, R1).

### Decision 8: what the wider tub (bay_w 53, 57.8 wide, y −48.9 … 8.9) costs

| item | bay_w 50 | bay_w 53 | source / verdict |
|---|---|---|---|
| zero-margin hip cut, legs 1/4, sim / proposal radii | −51.05 / −49.25 | −49.72 / −47.91 | G1, K1: confirmed (−49.25 G1 only) |
| uniform cut that keeps the cliff guard | window 0.49–3.14 | none with the proposal radii | section 2 |
| righting, uncut | 197/200 | 197/200; FALLEN contacts 5/200, 2.82 | G1, K1 (contacts G1 only) |
| −51.05 clamp clears legs 1/4? | yes (+0.08) | no: 1.42 into the tub | G3, LV |
| lanes (W8) | 1.00 to the tub | 1.00 | L, LV: confirmed |
| 14 AWG feed band y 8 … 11, z −26 … −22 | must sit at y 7.9 … 8.8 (0.80 from drop +0) | **no y position clears leg 4's drop** (2.45 mm³ at +0, 220.5 at +5): needs a new route near x 70–90 | K3 `s3_feed_stand.py`, `s1b_awg.py`: unverified (K3 only) |
| offtray shelf (case iii) | fits, 0.80 | fits only moved 0.70 north, 0.50 | G3; K3 only |
| cradle wall thickness tolerance (ESTIMATE stand) | t ≤ 6 (0.30 left) | t 4 leaves 0.80; t 5 collides (92.4 mm³) | K3 `s6_stand.py` (unverified) |
| tub north-bottom edge north of the CoM at rest | 8.5 | 10.0 | K3 `s7_roll_com.py` (unverified) |
| tray latch strike (0, −43), turned from below | inside the tub plan, column 734.3 mm³ | same | L, LV: confirmed |
| case i adapter plug room south (to the cradle wall) | 3.4 | 1.9 | K3 `s5b_removal.py` (unverified) |

The wider tub costs the uniform-cut option, the 14 AWG route and
cradle-wall slack. It does not change righting. Option (b) of decision 8 (no
strap inside, bay_w 50) keeps all of them.

### Decision 10: the nose

Round 2 measured nothing new on the nose. Round 1 stands:
- The only tub-caused stalls (64–70 mm rubble, outside the walking range) are
  the square nose corners jamming sideways (R1, verifier).
- The nose-south corner sits at r 112.4.
- Overhang at bay_w 53: 26.94 / 31.12 at the south corners.

The chamfer is an unsimulated ESTIMATE. The critic's checks (a chamfer
against the dock_block carrier, the B14 funnel and the stand's x stop) were
not run.

---

## 5. New faults for the backlog (B117 upward, ready to paste)

| id | title | verdict | evidence |
|---|---|---|---|
| B117 | The I1 leg port cannot dock or undock | **PROPOSAL (2026-10-02): seats + graft, owner sign-off on 3 frozen values; COUPON first** | Hook L 5.15 wide at the bottom in a 4.2 slot: no planar path from above to docked at slot 4.2 or 4.6, tilt −30 … 90, even at clearance 0. With the 8 mm dowels no path at 0.10 even at slot 5.2 or 7.0 (sound C-space tests by three designers, three verifiers and a judge; prep2 `I1_DOCK_OPTIONS.md`). Docs' 15° insertion and "plug at the cutout while hooked" are impossible (at 8° the plate is 1.6–2.5 over the cutout). Three fixes all verified to dock at ≥ 0.30; the judge picks "seats": two 30° cone posts at (−32, ±17.3), cone + V sockets, slot 6.2 grown outboard (x −50.1 … −43.9), plate bridged to the stem for \|y\| ≤ 18, spine rib x −46 … −38, 0.2 relief outboard of x −46. Dock at 7–10° with a radial approach, own sector off, leg 4 hooked tilt ≤ 10. Changes dowel_xy, hook_slot_x, hook_slot_w. Print the port coupon pair (seat_relief_x0 −46 and −42) and a seat-fit row first. |
| B118 | Stance load runs through the thumbscrews, not the hook | **OPEN (COUPON: insert pull-out + hand preload)** | In stance the foot lifts the plate's outboard end (sim My +241.5 N·mm per leg); the plate pivots on its most inboard deck contact, so the hook carries 0. The two thumbscrews and their heat-set inserts carry 972 N per pair today (3 legs × 3, μ 0.5 ESTIMATE; 660 without friction) on a 4.9 lever; 568 with the B117 seats fix. INTERFACES I1's "the thumbscrews only clamp" is false. Insert pull-out (ESTIMATE 400–900 N) and hand preload are unmeasured. Go ≥ 300 N per insert and ≥ 285 N preload; otherwise an outboard hold-down (owner). |
| B119 | The coxa plate is over-stressed at the screw line | **OPEN (fixed by B117's spine rib)** | Section of today's `coxa_yaw_base` at x −41 (two M3 holes): A 146.4 mm², I 195.2 mm⁴, 47.4 MPa at M 4629 N·mm (3 legs × 3 + foot friction), SF 0.74 against 35 MPa; 31.8 MPa without friction (judge `j3_section.py`; 47.5 by two other agents). Minimal's 45-wide plate: 46.2. The seats rib (x −46 … −38, \|y\| ≤ 8, z 0 … 6) gives 15.0 MPa, SF 2.33. |
| B120 | The documented leg-plug sequence cannot pass the deck cutout | **PROPOSAL: cross arm 7 × 16 r1 on the 11 × 11 cutout** | INTERFACES I1 / ASSEMBLY_GUIDE 2.9 mate the XT30 + XH-5 at the cutout with the plate hooked, then the mated pairs go down the cutout. The mated XH pair needs ≥ 13.63 rigid (end-on 14.60) in an 11 × 11: no motion passes it; an inline B5B-XH-A needs 11.65. The XT30 (either half or mated) passes at 0.40/side. Round 1's "XH-5 passes in no orientation" is wrong for the XHP-5 crimp housing: long side first with the 5 leads folded in one layer in a set order it passes at 0.65/side (26 AWG) or 0.42 (22 AWG); a two-layer stack fails at 22 AWG. Smallest fix: an arm 7.0 radial × 16 tangential, r 1, on the cutout: every half and both mated pairs pass at ≥ 0.41/side; 10.49 to other deck cuts, 7.69 to the pentagon edge, inside the ±10.5 drop keep-out (2.50). Without it: mate under the deck with the XHP-5 on the leg side, single-layer fold written into the guide. The coxa channel is open below with the leg off the deck, so it is never threaded. Wire ODs ESTIMATE. |
| B121 | Leg-side XH-5 pins 1–2 carry two or three wires each | **OPEN (unverified, one agent)** | ASSEMBLY_GUIDE 2.7 steps 3, 5, 6 put the entry lead's data + GND, the hand's data + GND and the foot-switch return on pins 1–2. One SXH-001T-P0.6 contact takes one insulation Ø0.9–1.9, AWG 28–22; three 26 AWG wires are about 0.39 mm². A splice before the housing is needed and undocumented. Seven d1.3 leads still fold through 11 × 11 (+0.64/side); seven d1.6 do not. |
| B122 | The Ø10 deck grommet at (0, 50) passes no plug | **PROPOSAL** | Option A's hole for the tray trunk + tray XT30. Clear bores needed: XT30U Ø11.42–11.45, XHP-5 with folded leads Ø11.47–12.88, USB-C overmold Ø13.91. Either route before terminating (loose bundle Ø5.11, Ø6.36 with one SXH pin passing; crimp the trunk's pins into the housing and solder the XT30's node end afterwards), or cut a slot 12.5 (E-W) × 7.5, r 1: XT30 1.15/side, XHP-5 22 AWG two-layer 0.36/side; 4.05 to option A's holes. Ø13 would leave 1.80 to the (0, 40) hanger. A cable straight down from it cuts the shelf's PDB halo; route it round the plate's north edge. |
| B123 | The tray bulkhead has no connector for the foot-switch lines | **OPEN (unverified, one agent)** | The five J6 lines (star board XH-6 → Pi GPIO) cross the deck at (0, 50) and end on the tray. The rear bulkhead (`part_avionics`: XT30, XH-5, 2 × SH, Qwiic, USB-C) has nothing for them, so "one latch, one pull, everything out" tray removal is blocked. Add a bulkhead connector or move the lines into the trunk. |
| B124 | Each docked coxa base meets its carapace sector | **OPEN** | 42.801 mm³ on all five stations: the yaw cup's back wall (19 × 0.5 × 8.55, r 71.5–72, z 33.45–42) against the sector arch, which starts at r 72. Measured by four agents (orchestrator `base_shell.py`; two I1 verifiers; round 1). `part_shell`'s checks use proxies and pass. Unchanged by every I1 fix. Thin the cup wall, move the arch out, or relieve the sector; then check `part_shell` against the posed `coxa_yaw_base`. |
| B125 | A leg swap needs its carapace sector off, which needs a tool | **OPEN (docs + design)** | With its own sector on, the leg module meets the sector by 634–1011 mm³ along any dock path (all three I1 designs, both verifiers); the docked base already overlaps by 42.8 (B124). The I3 sector latch is turned with a flat screwdriver from under the deck, so "no tools, under two minutes" fails on an assembled robot. The four other sectors and the cap can stay on. Rewrite ASSEMBLY_GUIDE 6.1 / 7.8 and INTERFACES I1. |
| B126 | Two more under-deck turn points sit inside the tub's plan | **OPEN (with the body layout, extends B100)** | Besides station 162 (−73.48, −12.30): station 306's carapace latch (66.67, −33.24) and the tray's I3 latch strike at (0, −43). Each Ø5 driver column meets the tub for 734.3 mm³, its full height, at both layers and both bay widths. Leave the columns open through the tub, or move the strikes. |
| B127 | The Pi's part height is missing from the carapace check | **OPEN** | `part_avionics` checks the Pi only to its board top. The Pi 5 STEP's USB stack stands 17.53 over the PCB underside and overhangs the edge by 3.00. Real 3D gap to the carapace: USB end +x 6.84 (11 mm standoffs) / 5.45 (13); USB end −x 3.56 / **2.17** (fails the 3 mm rule at 13). Tray float ±0.3 moves them 0.15–0.21. Add a part-height envelope from the STEP (or ~16 + 3 overhang), the float and a 3D gap. Active Cooler unmodelled. |
| B128 | One usable USB-A port for four loads | **PROPOSAL: bus adapter on UART** | USB end +x, Pi on 11 mm: only the lower middle port takes a plug, a right-angle ≤ 20 long with overmold ≤ 14 × 7, cable down or south (21.88, 21.58 worst float); the other three have 8.4–11.8. At 13 mm no port is reliable. Loads: bus adapter, D500 lidar USB adapter, optional mic (D-02), optional Wi-Fi (X-04). The adapter runs over UART (H2, jumper A, 3.3 V buffers, hardware direction); `SerialTransport` needs only a port path. Cross RX/TX (H2 RXD → pin 10); take GND off pin 9 or 14 (pin 6 is the buck's). Consider the lidar on GPIO 4/5 or 12/13 (logic level unchecked). |
| B129 | The cliff guard sets a ceiling on any hip limit | **OPEN** | `playground.py` declares `probe_out` only when the measured foot is 26.5 deep (PROBE_MAX − PROBE_LEAD). At h 118 that needs the measured hip ≤ −47.91; it moves 0.9° per mm of body height (−49.74 at h 120). Any hip stop at −47.5 or higher: 0/12 voids, the robot walks off the table. With a driver clamp at −48 the reading clears the threshold by 0.11–0.18 mm. A hip limit or clamp in NORMAL must stay below the ceiling, or the probe depth must change (PROBE_MAX 28 → −46.14). The live probe is not gated by `pebble_feasibility`. |
| B130 | The righter folds legs under the keel | **PROPOSAL (decision 13): belly in the sim + a FALLEN/RIGHTED hip clamp** | With the battery mass, recover1 + supervisor touches the tub in 12/200 drops (legs 1, 2, max 3.06; servo off). With the datasheet servo: 14 + 2 RIGHTED, max 4.89 (one agent). A pinch: the hip, commanded to −66 … −70 at full torque, presses the folded leg under the settling keel at up to 46.9 N. Target keep-outs that raise the knee leave 1–6/200. A hip clamp ≥ −51.05 on legs 1–4 gives 0/200, 196/200 unchanged, 38–43% of FALLEN ticks clipped; applied only in FALLEN/RIGHTED it leaves BRACE and the probes alone. Value must be re-derived for bay_w 53 (≥ −49.7) and for the fitted foot (B95). Drop-fixture contacts at t ≈ 0 (9/200, 2.66) are a fixture effect. |
| B131 | Every leg-reach verdict assumes a 135 mm foot | **OPEN (with B95; unverified, one agent)** | With the hand as `part_hand` builds it on the shortest tube (sole 165.2 below the knee axis): at hip −51.05 legs 1–4 pass 17.5–18.7 through the tub, leg 0 is 6.3 into the shelf PDB, leg 1 7.5 into the adapter. Clearing needs ≥ −24 … −20 (legs 1/4), ≥ −32 (legs 2/3) for the tub and ≥ −40 for the shelf. The tub, the shelf and the clamp value depend on B95's outcome; `pebble_feasibility` and the limits must know which end effector is fitted. |
| B132 | The supervisor and playground ignore the params joint limits | **OPEN** | BRACE (hip to −58.6), the stance probe (−60.84) and the cliff probe (−51.58) command poses that are stopped only by the MJCF range in the sim or the driver clamp on hardware; neither clamps to `joints.pos_deg`. Any limit change in params does not reach them. |
| B133 | The 14 AWG feed from the tub nose has no route at bay_w 53 | **OPEN (unverified, one agent)** | The proposal's band (y 8 … 11, z −26 … −22) overlaps the bay_w 53 tub wall; moved north of it (y 9.4 … 12.4) it runs 2.45 mm³ into leg 4's drop keep-out (220.5 at +5). No y position clears at bay_w 53; at bay_w 50 only y0 7.9 … 8.8. Route it under the drop, inside the tub, or near x 70–90. A cradle wall ≥ 30 tall also meets it. |
| B134 | The hub on the stand: thin margins, no service | **OPEN (with B85; unverified, one agent)** | Option A with down-facing boards (z −54.4): 1.00 to the crown plate (ESTIMATE stand), gone at +1.0 of plate height or a 1.42–1.47° roll; the 0.5-clearance cradle allows 2.29°, and joint-box poses put the CoM 15–17 past the tub's north-bottom edge (8.5 / 10.0 north of it at rest). 3 of 4 adapter screws are blocked from below, the shelf cannot be lowered on the stand: hub service means the robot off the stand, against the proposal's "serviced from under the deck on the stand". |
| B135 | The loom lanes have no retention and a fixed turn | **OPEN** | The leg 2/3 lanes keep 1.45–1.75 to the docking-hook sweep and 0.60 to the Ø8 bosses; nothing holds the loom in them. The A → C jog must sit north of y −38 (south of it: 10.9 mm³ into the hook envelope, 4.2 into the coxa base; about 2 mm of slack). The riser to the lane must be inside the drop keep-out box (outside it hits the bay_w 53 tub by 65.6). Put the lane in params and add zip anchors. |
| B136 | Legs reach their own drop keep-out box even with the hip cut | **OPEN** | Sim capsules on a 2.5° grid: legs 2/3 enter their own drop box (the mated XT30 + XH-5 under the cutout) in 828 poses at −10.28 with hip ≥ −51.05 (3828 at about −20 uncut); contacts up to hip −42.5. Not counted by round 1's census. 176–198 of the lane-contact poses are covered only by it. Gate or model it. |
| B137 | The port coupon and fit-ladder row D cannot rehearse a dock | **OPEN (with B117)** | `part_port_coupon.port_coupon_plate`'s patch runs inboard to x −57 and penetrates the deck coupon by 0.91 on any dance path (146.7 mm³ in 3D): it can never dock, today or with any fix. Cut it to the real plate, x −46 … −11; its `__main__` should replay the dock path, not only the docked boolean. Fit-ladder row D (3.8/4.2/4.6) tests a 2.2 lip, not the L; with B117 it becomes 5.8/6.2/6.6 plus a seat-fit row. |
| B138 | `pebble_feasibility` and `mass_budget` miss the belly's mass | **OPEN (with B97)** | The checker uses M_TORSO 1439.8 and TORSO_COM z 24.5; the proposal's torso is 1467 g at (−2.5, −2.0, −1.2), robot CoM at stance (−1.33, −1.07, −10.71). That mass alone doubles the righter's tub contacts (B130). RecoverEnv's domain randomiser restores mass/inertia/ipos from build-time copies, so an in-memory edit is undone unless `env.dr._base_*` is updated. |

---

## 6. Corrections to `docs/BODY_LAYOUT_PROPOSAL.md`

Each line is "section: old → new, source".

1. Intro, "Not checked by either run" paragraph: "station 162's carapace latch … the tub must leave that column open" → three columns are inside the tub's plan: latch 162 (−73.48, −12.30), latch 306 (66.67, −33.24) and the tray strike (0, −43); each Ø5 driver column meets the tub for 734.3 mm³. Source: R1 `keepouts/m3_tub.py` (confirmed), LV.
2. Intro review bullets: "A's carapace margin is 5.5 mm, not 10.5 (12.25 with the Pi's USB end +x)" → vertical 5.45 / 12.40 (STACK16); real Pi 5 STEP 3D gap −x 3.56 (3.41–3.72) / +x 6.84 (6.63–7.05) at 11 mm. Source: R1 `boards/b2_margin.py`; U `s8_stack_gap.py`, UV `v3_gap.py`.
3. Section 1 fact 2: "Swept through the docking dance (tilt 0–15°, 3.5 mm slide), a hook foot reaches z −13.8 (−13.3 docked), at r 57.5–68.4" → today's hook cannot dock at all (no path). Its deck-clear sweep reaches −14.80 (−15.05 with a 5.2 slot), r 56.95–69.31. With the seats fix: −13.30 on the path, −13.76 at any reachable pose, r 56.94–66.79 (58.19–67.86 at the lip corners). Source: R1 `keepouts/m2_hook.py`; S `e1_envelope.py`, SV `e1_envelope.py`.
4. Section 1 fact 2: "hits those envelopes by 372 mm³; even its pack alone … by 23.6 mm³" → not recoverable; ≥ 48 for every 50–55-wide deck-roof bay, pack alone 48 with a deck-clear envelope. Source: R1 `keepouts_verify/v4_bay.py`, `v5_bay_scan.py`.
5. Section 1 fact 3: "centre at y −23 … −16 (y −26 … −14 with 3 mm)" → y −21.9 … −17.2 for bay_w 53 (−26.2 … −13.1 for the 3 mm reading); y −20 stands. Source: R1 `keepouts/m4_band.py`.
6. Section 1 fact 1: "a loom arriving on the deck top meets 697 mm³" (11 × 5.5 loom) → the real per-leg loom is 7 conductors, 14.17 mm² (not 60.5); the conclusion holds. Source: L `l3_bundle.py`, LV `m2_pack.py`.
7. Section 2 table, bottom z: "ground clearance at stance 62.6 / 65.6" → 62.15 / 65.15 settled in the sim (62.13 with the mass). Source: R1 `belly/checks/variant_checks.py`.
8. Section 2 table, legs: "about 205 poses per leg on a 5° grid (capsules r 15 / 12). Leg 0 never reaches it" → 114 / 97 / 97 / 114 (sim capsules), 138 / 122 (r 15/12); leg 0 never reaches the tub but reaches the shelf box (15 poses) and the shelf boards. Source: R1 `exp_motions/reach_grid.py`.
9. Section 2 "Stand (B85)" / section 3 risks: "The hub is serviced from under the deck (on the stand the cradle leaves the north underside open)" → open only north of y 47.2; with down-facing boards at z −54.4 the adapter's screws are blocked from below and the shelf cannot be lowered on the stand: hub service needs the robot off the stand (unverified). Source: R1 `boards_verify/v4_arrangements.py`; K3 `s4_service.py`, `s5b_removal.py`.
10. Section 3, shelf hangers "(±40, 20), (0, 20), (0, 40)" → they block every arrangement with the bus adapter; use posts at (−50, 34), (50, 34), (26, 64) (0.59 to the hook envelope, 0.89 to the coxa bases, 0.57 to drops +5). Source: R1 `boards/b4c_verify.py`; G3 `g3_verify.py`, K3 `s1_clear.py`.
11. Section 3, "boards facing up … 28 mm deep (x ±45, y 12 … 54, z −38 … −10)" → with the bus adapter on the shelf it hangs face-down to z −54.4 (case i: adapter (−19.8, 30.8), PDB (−20.2, 38.2) up, star (21.8, 34.2) up, turned 90; plate top −36.3). Source: G3 `b4b_pack.py` (`pack_53_h3_cradle_f-54.4_c1.log`), K3 `s1_clear.py`.
12. Section 3, lanes "5 × 7 mm (x ±44.6 … 49.6 south to y −38, out past the hook envelope at x ±51.8 … 57, then down to the cutout)" → W8 8 × 7: A x ±45 … 53, y −38 … 12; B x ±45 … 59.8, y −38 … −30 (the jog north of y −38); C x ±51.8 … 59.8, y −53.43 … −30; the riser inside the drop keep-out box; leave the shelf through its south face. Source: L `l4_variants.py`, LV `m1_lanes.py`.
13. Section 3, "The 14 AWG … runs to the node along y 8–11, z −26 … −22" → overlaps the bay_w 53 tub's north wall (y 8.9). At bay_w 53 no y position clears leg 4's drop (2.45 mm³ at +0); at bay_w 50 only y 7.9 … 8.8. A new route near x 70–90 is needed. Source: R1 `boards/freezone.py`; K3 `s3_feed_stand.py` (unverified).
14. Section 3 margins: "Carapace over tray + Pi: 5.5 mm … 12.25" → as correction 2.
15. Section 3 margins: "Lane side clearance: 0.5 mm to the Ø8 hanger bosses (1 mm more and it touches: 15 mm³)" → 0.60; 1 mm more 26.31 mm³ in the 8 mm layer (15.03 is the 5 mm layer); W8 keeps 1.00. Source: L `l5_extras.py`, LV `m3_extras.py`.
16. Section 3 deck holes: "(0, 50) | 10 | grommet: the tray trunk + tray XT30 down to the shelf" → passes no plug with leads on (XT30U needs Ø11.42, XHP-5 + leads Ø11.47–12.88). Route before termination, or a 12.5 × 7.5 r1 slot (4.05 to option A's holes). The J6 foot-switch lines also cross here. Source: C `s2_slab.py`, CV `v2_pass.py`, `v3_fix.py`.
17. Section 3 deck holes: "(0, −43) … the tray's latch strike (I3)" → inside the tub's plan; its driver column meets the tub for 734.3 mm³. Source: LV (arithmetic + `keepouts.py` tub box), L `v1_verify.py`.
18. Section 3 CAD changes, `cad/iface.py`: "`leg_port_hook_envelope()` (the docking swing, bottom −13.8)" → derived from params after the I1 fix (B117): −13.30 path / −13.76 any pose with the seats fix. Source: S, SV `e1_envelope.py`.
19. Section 3 risks: "Belly falls load six M3 in heat-set inserts" → add: the righter folds legs under the keel (12/200 drops with the mass, max 3.06; B130). Source: G2, K2.
20. Section 3 risks / section 2 legs: "Nothing gates those poses" → SELF_CONTACT gates the gesture audit and studio once the MJCF has a belly; the righter, `save_keyframe_gesture()` alone and the playground `check` are not gated. Source: R1 `exp_motions/selfcontact_probe.py`.
21. Section 4 note: "A 3 mm margin needs … lift ≤ 11.4" → 11.40 centred / 11.10 floated, under the vertical-margin reading only. 3D gap with the Pi 5 drawing model: 8.28; with STACK16 + overhang: 6.12. Source: B `ob1_lift.py`, BV `v1_lift.py`.
22. Section 4 note: "(with it −x, … caps … lift at 4.4)" → 4.15 (STACK16), 1.15 with the 3 mm overhang; as a 3D gap with the overhang it fails at lift 0 (2.87). Source: B, BV.
23. Section 4 note: "B survives only with a hub at most about 14.3 tall (12.1 under the IMU) … That is not measured here" → measured: ceiling 14.20 / 12.00 (vertical rule), 11.38 / 9.18 (drawing 3D), 9.22 / 7.02 (strict). A vertical-header star (13.90 mated, no lead) never fits; a side-entry star on a new 38.9 × 49.2 board (10.20) plus a 30.5 PDB (8.30, ESTIMATE) fit the vertical rule only. Source: B `ob4_stacks.py`, `ob5_layout.py`; BV `v7_arith.py`, `v6_layout.py`.
24. Section 4 deck (B): "rails 26 tall" → 19.10 at lift 11.10 (8 + lift). "(0, 52)" pass-through and "14 AWG grommet Ø9 at (29, 14)" → both collide with a hub that fits; leg 0 at (−40, 52) (1.15 to the carapace skirt) or (±34, 46) with lead ≤ 5. Source: B `ob5b_holes.py`; BV `v8_carapace_cols.py`, `v6_layout.py`.
25. Section 6 table, tray height over the deck, B: "21.4" → about 14.5 if B is built at lift 11.10 (3.40 + 11.10; arithmetic). Source: B `oblib.py`.
26. Section 6 note: "The sim today puts a 1434.6 g torso at (0, 0, 24.5)" → 1439.7 g at z 24.50 (compiled v0). Source: R1 `belly/checks/variant_checks.py`.
27. Section 7: "The lanes are why the layer is 8 mm instead of 5: lanes 5 × 7 instead of 5 × 4" → a bare 7-conductor loom fits 7 × 4 in the 5 mm layer (5 × 4 is marginal: 5.07 needed); 8 mm is kept for the hook margin (4.26 vs 1.26 at any pose) and tied looms (8 × 7). Source: L, LV `m2_pack.py`; SV `e1_envelope.py`.
28. Section 8, decision 1: "B only with a hub ≤ ~14 mm tall, not yet measured" → measured (correction 23); B needs the USB end +x and the vertical reading of the 3 mm rule. Source: B, BV.
29. Section 8, decision 2: "lanes 5 × 4 … lanes 5 × 7" → 7–8 × 4 / 8 × 7. Source: L, LV.
30. Section 8, decision 10: "26.5 / 30.7 mm at the south corners" → 26.94 / 31.12 at bay_w 53; the nose-south corner reaches r 112.4. Source: R1 `keepouts/m3_tub.py`.
31. Section 8, decision 13: "Either give the sim a belly body and teach `pebble_feasibility` the tub, or narrow the soft limits" → options: model the belly; a uniform hip limit (no value at bay_w 53 keeps the cliff guard); a target keep-out; a hip clamp on the righter's commands in FALLEN/RIGHTED only (0/200 contacts, outcome unchanged). Source: G1, K1, G2, K2.
32. Section 10.8: "hip −70 … −45 and knee −130 … −90" → plus the battery mass doubles the contacts on the righter's fall path (12/200, 3.06). Source: G2, K2.
33. Section 11, `iface`: add an I1 dock-path regression (planar C-space + 3D replay at ≥ 0.30) and a connector pass-through check (cutout, grommet). Source: J `r1_replay3d.py`, SV `cs.py` / `c1_search.py`; C, CV.
34. Section 11, `part_avionics`: "tray + Pi vs the carapace (fail under 3 mm)" → with a Pi part-height envelope from the STEP (17.53 over the PCB underside, 3.00 overhang), the ±0.3 float, and a 3D gap. Source: U, UV.
35. Section 11: add `part_shell` × the posed `coxa_yaw_base` (42.801 mm³ today). Source: O `base_shell.py`; MV `c1_shell_docked.py`; QV `v11_sector.py`.

---

## 7. Still unmeasured

**I1**
- Every coupon question: slot passage by hand, seat fit, socket print, insert
  pull-out, hand preload, post twist, 20 cycles.
- Roll, yaw and y during hand docking. The planar proofs cover x, z and tilt
  only.
- The femur and tibia in the dock sweeps.
- The V-socket wall fix (seat_mouth 0.2 or \|y\| 17.1) and a 0.4 relief: not
  re-proven.
- The graft is measured by the judge only (J `j4`, `j6`, `r1`).
- B27 (deck_t 6 in the CAD, 4 in params): every hook gap assumes 6.

**Decision 13**
- The FALLEN/RIGHTED-only clamp and the hip-first keep-out (K1, K2 only), and
  every clamp value against the real CAD leg solids.
- Any option on the fitted end effector after B95 (with the hand, nothing
  above holds).
- Righting with the clamp under DR or obs-v2 checkpoints, and on the
  shove-to-fall path.
- bay_w 53 with the mass (v1m-style), and the 5 mm layer with the mass.
- Hardware overshoot, back-drive and calibration offsets at a clamp. The
  cliff reading margin is 0.11–0.18 mm at a −48 driver clamp.
- The v1m inertia tensor (ESTIMATE, 28.2 g residual).

**Decision 14**
- The real USB plug and cable sizes (calipers). Plug sizes are ESTIMATES.
- UART bring-up: `ttyAMA0` at 1 Mbaud on the Pi 5, the RX/TX labelling.
- Which edge of the bus adapter carries its USB-C (Waveshare STEP not read).
  Case i's south plug room is 3.4 / 1.9.
- The Active Cooler.
- Stand geometry: crown plate kept or cut, cradle wall thickness and height,
  x stop. All margins near it are ESTIMATE-based.
- The 14 AWG route at bay_w 53.
- Righting and rubble with boards hanging to z −54.4.
- The hybrid's fixes: notched tray corners, no north cradle wall.
- Insert pull-out for the shelf posts. The post screw at (50, 34) needs a
  shaft ≥ 42.
- Board envelopes: the PDB, the 1000 µF capacitor, lead halos (ESTIMATES).

**Decisions 1, 2, 8, 10**
- B's lift limit with the real Pi STEP (only box and drawing models were
  run).
- B in the sim (righting, rubble, CoM, 4-foot margin).
- B's relocated holes against the other deck cuts and the carapace feet.
- Hard-wired star.
- Lane retention (zip anchors).
- Wire ODs against the vendor listings: 20/26 AWG are listing values; 24 AWG
  and the B-16 leads are ESTIMATE.
- The tie strap thickness.
- The 5 × 4 packing: a random search found no 5 × 4 packing at nominal ODs,
  which is not a proof that none exists.
- Nose chamfer against corner jams, the dock_block carrier, the B14 funnel
  and the stand x stop.
- C in the sim.
- Pack mass 380 vs 343.

**Connectors**
- The leg-side splice (B121).
- Hand access to mate under the deck on the stand.
- A JST PHD header half as an alternative.
- Lead OD and bend radius at the XH back face (ESTIMATE). The single-layer
  fold's margin depends on them.

---

Repo check: every round-2 agent, verifier, skeptic and the judge reported
`git -C /home/bitwisebard/Development/rocky status --short` empty. Re-run when
this report was written (2026-10-02): empty output, rc 0.
