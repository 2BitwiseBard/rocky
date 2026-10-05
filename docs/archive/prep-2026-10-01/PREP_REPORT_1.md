# Body layout: preparation report (read-only, 2026-10-01)

Measurements ahead of the owner's decisions in `docs/BODY_LAYOUT_PROPOSAL.md`
section 8, plus decision 14 (the three small boards, backlog B108). Nothing in
the repo was changed. `S` below means
`/tmp/claude-1000/-home-bitwisebard-Development-rocky/61c1816d-d6f0-4565-8ba4-46b722cb69ec/scratchpad/prep`.
Frames are the proposal's body frame (z = leg z, deck −10 … −4, +x east, leg 0 north).
Lengths in mm, volumes in mm³, angles in degrees.

**How to read the verdicts.** "Confirmed" means a second agent re-measured the
claim with its own scripts, or it matches the documented number.
"Unverified" means one agent measured it and nobody checked it.
"Refuted (verifier: …)" means the check disagreed. Where two numbers differ,
both are shown.

Sim variant trees (copies of HEAD 44b51e3, in `S/belly/trees/`):

| tree | what it is | fingerprint |
|---|---|---|
| v0_control | today's robot, untouched (pebble.xml byte-identical to the repo) | 87215110e9c4 |
| v1_a8 | + massless tub (8 mm layer, z −55.4 … −18.0) + shelf box | 777fdb4eda14 |
| v1m_a8_mass | v1 + the proposal's torso mass and CoM (1467 g at (−2.5, −2.0, −1.2); inertia ESTIMATE) | 47435187271f |
| v2_a5 | + massless tub (5 mm layer, z −52.4 … −15.0) + shelf box | 4f77d216a700 |
| v0_limits / v1_limits | v0 / v1 with the hip lower limit −70 → −51.05 | 47db548b0a72 / c76977b87e64 |

Gate on the trees (`S/belly_verify/vchk.py`): PASS. Box bounds match the spec to
1e-12 mm, masses are bitwise equal to v0 in v1 and v2, and v0 reproduces walk
189.281 mm and turn 41.829°.

---

## 1. What moved

Numbers in the proposal (and the backlog rows it feeds) that today's tree does
not reproduce.

| # | claim (where) | documented | measured | verdict | why / evidence |
|---|---|---|---|---|---|
| 1 | I1 docking move: tilt 0–15° plus a 3.5 mm slide (fact 2) | possible | **impossible as drawn.** At the bottom, the hook foot plus its down-turn is 5.15 wide; the slot is 4.2. No path from above the deck exists at any tilt from −30 to 90, at slot 4.2 or 4.6 | confirmed (both agents) | `S/keepouts/dance_cspace.py`, `S/keepouts_verify/v10_sound.py` |
| 1b | Fix: widen the slot to about 5.2 | — (keepouts agent's proposal) | **Not enough on its own.** The I1 dowel posts (9.2 above the deck, 0.175 radial play) force a near-vertical last 9.2 mm. With 0.05 mm of clearance there is no path at slot 5.2, 6.0 or 7.0. At the documented 15° the plate sits 2.05 below the dowel tips | refuted (verifier: dowels were left out) | `S/keepouts_verify/v14_sound_margin.py`, `v7b_dowel_margin.py`, `v9_dowel_3d.py` |
| 2 | Hook foot lowest point, swept (fact 2) | −13.8 | **−14.80** (deck-clear motions); **−15.05** with a 5.2 slot. −13.8 comes only from a pivot that cuts into the deck (14.47 / 55.96 / 75.42 mm³ at 5 / 10 / 15°) | refuted (both agents) | `S/keepouts/m2_hook.py`, `S/keepouts_verify/v3_envelope.py`, `v6_pivotE_3d.py` |
| 2b | Hook foot, docked | −13.3 | −13.30 | confirmed | same |
| 3 | Hook envelope radius (fact 2) | r 57.5–68.4 | r 56.95–68.21 on the leg centreline; **69.31** at the lip corners (y ±12.3) | refuted | same |
| 4 | A bay with the deck as its roof hits the hooks (fact 2) | 372 | not recoverable: the proposal recorded neither the box nor the envelope. Every 50–55-wide deck-roof bay with y −26 … −14 hits them by **≥ 48** | cannot tell; the rejection holds | `S/keepouts_verify/v4_bay.py`, `v5_bay_scan.py` |
| 5 | The pack alone hits the hooks (fact 2) | 23.6 | 23.9, but only with the proposal's own envelope, which is not deck-clear. With deck-clear play: **48** | confirmed number; honest value 48 | `S/keepouts_verify/v4_bay.py` |
| 6 | Clear band for a 57.8-wide tub (bay_w 53) (section 2) | y −21.5 … −17.5 | y −21.9 … −17.2 (and −26.2 … −13.1 for the "+3" reading). y −20 stands | confirmed with small shifts | `S/keepouts/m4_band.py`, `S/keepouts_verify/v11_tub_drops_latch.py` |
| 7 | "Drops +5 mm" | the ±10.5 box | The ±10.5 box already **is** the 11 cutout + 5. Grown a further 5 (±15.5) it hits the tub by 1035 (verifier) / 1227 (keepouts, which also deepened it 35 → 40), and no clear band exists | convention clarified; use grow = 0 | same |
| 8 | B100: carapace latches inside the tub's plan (intro, backlog) | one (station 162 at (−73.5, −12.3)) | **two**: station 162 (−73.48, −12.30) and station 306 (66.67, −33.24). Each Ø5 driver column meets the tub for 734.3 (its full height) | refuted (both agents) | `S/keepouts/m3_tub.py`, `S/keepouts_verify/v11_tub_drops_latch.py` |
| 9 | Carapace margin over tray + Pi at 11 mm standoffs (fact 5, section 3) | 5.5 / 12.25 (USB −x / +x) | 5.45 / 12.40, **but only if the Pi carries about 16 mm of parts over the board** (USB-A stack 15.8–16.5 off the Pi 5 drawing). `part_avionics` models no part height: to the board top the margin is 21.45 / 28.40. Counting the drawing's 3 mm port overhang: −x **2.45**. The tray floats ±0.30 in its channel: worst case 5.15 / 12.10, and 2.15 at −x with the overhang. As a 3D gap the −x side fails the section-11 "under 3 mm" rule even at 11 mm (2.38 drawing model, 1.71 box + overhang) | confirmed with an added condition | `S/boards/b2_margin.py`, `S/boards_verify/v1_pi_drawing.py`, `v2_margin.py` |
| 10 | B108 derived margins | 3.5 / 10.25 (13 mm); −2.0 / 4.75 (18.5 mm) | 3.45 / 10.40; −2.05 / 4.90 (−x at 18.5 is 0.08 mm³ into the carapace) | confirmed within 0.15 | same |
| 11 | Ground clearance under the tub at stance | 62.6 / 65.6 | **62.15 / 65.15** settled in the sim (62.13 for v1m). 0.45 short from servo sag: the torso settles at 117.55, not 118.4 | confirmed within the stated ±1.5 | `S/belly/checks/variant_checks.py`, `S/belly_verify/vchk.py` |
| 12 | Legs reaching the tub (section 2, B97) | ~205 poses per leg (5° grid, capsules r 15 / 12) | legs 1–4: **114 / 97 / 97 / 114** (sim capsules r 12 / 10, foot 6.5); 138 / 122 with the proposal's radii; 156 / 133 within 3 mm. v2: 117 / 104 | refuted on the count, confirmed on the fact | `S/exp_motions/reach_grid.py`, `reach_analyze.py`; `S/verify_motions/v_reach.py` |
| 13 | "Leg 0 never reaches it" | never | true for the tub (closest 45.09). **Leg 0 reaches the shelf box**: 15 poses at 5°, 81 at 2°, deepest −1.51 at (yaw 0, hip −70, knee −110) | confirmed + new | `S/belly_verify/vchk.py` |
| 14 | "Nothing gates those poses" (section 3 risks) | no gate | Once the MJCF has belly geoms, `pebble_feasibility`'s existing SELF_CONTACT pass catches the tub (19 samples, 6.6). That only works for callers that pass a model: the gesture audit and the cockpit studio. The righter, `save_keyframe_gesture()` alone and the playground `check` are not gated | partly outdated | `S/exp_motions/selfcontact_probe.py`, `S/verify_motions/v_selfcontact.py` |
| 15 | Ø10 grommet at (0, 50) for "the tray trunk + tray XT30" (section 3) | passes | passes **none** of: XT30 (needs Ø11.45), XH-5 housing (Ø15.86), USB-C overmold (Ø13.96). Ø12.5 passes the XT30 only. A 15.5 × 7.5 E-W slot passes all three | unverified (boards agent only) | `S/boards/b6_grommet.py`, `b6b_slot.py` |
| 16 | 14 AWG run y 8–11, z −26 … −22 (section 3) | clear | overlaps the bay_w 53 tub, whose north wall is at y 8.9. It must move to y ≥ 9.4. Carried on west to the node, it runs into the star board / PDB in the packed shelf cases | unverified (boards); its extension confirmed by the verifier | `S/boards/freezone.py`; `S/boards_verify/v4_arrangements.py` |
| 17 | Shelf hangers (±40, 20), (0, 20), (0, 40) | fine | block every shelf arrangement that adds the bus adapter. Three new posts at (−50, 34), (50, 34), (26, 64) work, but sit 0.59 from the hook envelopes and 0.89 from the coxa hooks | confirmed (0.59); the 0.89 is the verifier's | `S/boards/b4c_verify.py`; `S/boards_verify/v4_arrangements.py` |
| 18 | "On the stand the cradle leaves the north underside open" | open | only north of y 47.2. With the cradle walls, the shelf arrangements collide at bay_w 53 (PDB 115 mm³; case iii up to 1571). Every down-facing board found reaches z −55.4, the crown plate top (gap 0.00, ESTIMATE that the plate stays) | refuted (verifier) | `S/boards/b4b_pack.py`; `S/boards_verify/v4_arrangements.py` |
| 19 | INTERFACES I1: XT30 + XH-5 "live in the 11 × 11 pass-through" | both pass | the XH-5 (14.8 × 5.7, needs a 14.50 square) passes neither the 11 × 11 deck cutout nor the coxa channel in any orientation; the XT30 passes | confirmed (both agents) | `S/keepouts/m6_drop_and_smoke.py`, `S/keepouts_verify/v13_connectors.py` |
| 20 | Coxa base vs carapace (not in the proposal) | 0 (part_shell's proxy checks) | each docked base meets its sector by **42.8** (cup back wall at r 71.5–72 against the arch starting at r 72) | unverified (keepouts agent only) | `S/keepouts/m0b_base_shell.py` |
| 21 | Sim torso (section 6 note) | 1434.6 g at (0, 0, 24.5) | 1439.7 g at z 24.50 (v0 compiled); feasibility uses M_TORSO 1439.8 | moved slightly (D063 mass changes) | `S/belly/checks/variant_checks.py`, `S/belly_verify/feas_com.py` |
| 22 | recover1 t_stood medians (RL_GUIDE, recover1 row) | 0.49 s (0.42) | 0.46 s (0.38) on v0, counts exact. RL_GUIDE says the D062/D063 re-runs checked counts only | stale today, not caused by the belly | `S/exp_righting/run_published.sh`, `S/verify_righting/run_eval_runpy.py` |

What reproduces (confirmed or matching the documented number): fact 1 (697.4 mm³
with an 11 × 5.5 loom, and the column under the cutout clear), the docked hook at
−13.30, the drop keep-out ±10.5 × 35 (datasheet margins 3.05 / 4.53 in plan,
≥ 6.9 deep, ESTIMATE), every row-4 zero (tub against hooks, bases, deck and
carapace; hook gap 3.21), the tub overhang table (18.0 / 26.5 / 9.5 and
22.2 / 30.7 / 13.7), the carapace ceiling (49.4; low spots 38.45 at r 59.5;
pads 45.4), the tray slide room (36.41 / 22.81), the shelf box clear of
everything (gaps: hooks 2.95, tub 4.60, drops 16.5).

---

## 2. Decision 14: where the three small boards go

The boards are the bus adapter (42 × 33), the 5 V buck and the 6 V UBEC (B108).
"Margin" is the vertical distance from the Pi's top to the carapace, with a
16 mm part stack on the Pi (STACK16). Every margin below is confirmed by the
verifier to 0.01 mm unless marked.

| option | what it is | margin USB −x / +x (STACK16) | with the 3 mm port overhang | smallest clearance | status |
|---|---|---|---|---|---|
| **pi13** | Pi on 13 mm standoffs; buck at (−31, 6, turned) and UBEC at (−20, −12) on the tray; **the adapter still has no place** | 3.45 / 10.40 (3D 2.24 / 6.76) | 0.45 / 10.40 (3D 0.45 / 4.45) | buck top 1.00 under the Pi keep-out | tray poses confirmed (0.000 mm³ pairwise). The adapter's home is open |
| **pi18** | Pi on 18.5, IMU moved to (18, 1); adapter (−20, −4) on 2.5 standoffs, UBEC (−3, 25), buck (12, −23, turned) | **−2.05 (0.08 mm³ into the carapace)** / 4.90 | −5.05 (11.97 mm³) / 4.90 (3D **2.82**, under the 3 mm rule) | adapter jack 1.40 under the keep-out | everything fits on the tray, but the USB end must face +x, the 3D gap fails the 3 mm rule, and the upper USB-A ports have 7.6 / 5.1 of plug room (unusable) |
| **offtray** | Pi stays on 11; all three boards on the option A shelf | 5.45 / 12.40 (3D 3.64 / 7.97) | 2.45 / 12.40 (3D 1.71 / 5.85) | shelf case iii: 1.43 (UBEC to leg 4's latch column) | **refuted (verifier):** it collides with the U-cradle walls at bay_w 53 (adapter 1571, PDB 530 mm³) and at bay_w 50 (184.8). The adapter bottom rests on the crown plate (0.00, ESTIMATE). The 14 AWG to the PDB crosses the star (192) and the PDB (62.4). Legs 0, 1 and 4 reach the boards by 5–9 |
| hybrid (boards agent's pick) | Pi on 11; tray grown 30 north with the adapter beside the Pi; buck + UBEC on the shelf with the star board and PDB (case iv) | 5.45 / 12.40 | 2.45 / 12.40 | bulkhead (narrowed to ±20) 1.31 to the carapace; tray 1.40 over leg 0's coxa plate | **refuted (verifier):** the carapace cannot be lowered over the grown tray (contact 23 above the seat, 192.5 mm³ at 5 above). Leg 0 cannot dock or undock with the tray fitted (it meets the bulkhead from 10–11° of tilt). The shelf case iv collides with the cradle walls at bay_w 53 (115 mm³) |

Points that apply to every option:

- **USB end +x.** As a 3D gap, the −x side fails the 3 mm rule even at 11 mm:
  2.38 with the drawing model, 1.71 with the box plus the overhang (verifier).
  Facing +x passes at 11 mm (12.40 vertical; 3D 7.97, or 5.85 with the overhang)
  and at 13 mm (10.40; 3D 6.76, or 4.45 with the overhang).
- **The Pi part height is not in the code.** `part_avionics` checks only to the
  board top, so any carapace check passes without a part-height envelope
  (about 16.0 plus the 3 mm overhang).
- **Tray float.** The tray moves ±0.30 in its channel. All margins move with it.
- **The Ø10 grommet.** It passes none of the XT30, the XH-5 housing or a USB-C
  overmold (section 1, row 15). Fixes: terminate the leads after routing, a
  Ø12.5 hole (XT30 only), or a 15.5 × 7.5 slot (all three; gaps ≥ 2.55).
  Unverified.
- **UART mode.** The adapter can be driven over UART (Waveshare jumper A, the
  Pi's GPIO 14/15). Its 3 Dupont leads pass Ø10, and the USB-C plug halo goes
  away. This needs a driver change (today `/dev/ttyACM0`). Unmeasured.

**Recommendation (orchestrator's synthesis, not a verified layout):**

- **No option is ready as measured.** Both arrangements that move boards to
  the shelf (offtray, hybrid) are refuted. pi18 fails the 3 mm rule.
- **What survives is pi13 with the USB end +x.** Buck and UBEC go on the tray;
  their poses are verified at 0.000 mm³. Margin 10.40 (3D 6.76, or 4.45 with
  the overhang).
- **The adapter still needs a home.** Alone on the shelf (case i) it was
  packed at 3.00. That case was not re-checked, and it inherits the shelf
  problems listed under offtray: cradle walls, crown plate, new posts,
  legs 0/1/4 reach.
- **If the owner wants the Pi kept at 11 mm** for the best margin, the
  off-tray boards need a different stand first: no cradle wall over the
  shelf, or the crown plate cut back north of the tub. They also need a leg
  keep-out over the shelf, and the 14 AWG route has to be modelled.

---

## 3. Decision 13: the legs vs the tub

### Which shipped motions touch the tub

Measured in v1_a8 through the shipped stack. Confirmed by the verifier unless
noted.

| motion | commanded poses inside the tub reach set | leg-tub contacts |
|---|---|---|
| wave gait at 34.20 mm/s, 24 headings; turns at ±0.1849 rad/s; 300 CommandSlew ramps | 0 (closest 73.74 to the tub) | 0 |
| reversal + safe-stop BRACE | 0 (hip down to −58.6) | 0 |
| 12 gestures + the gesture audit (19 rows, 0 FAIL, identical v0 / v1) | 0 (closest 78.86 tub, 100.17 shelf) | 0 |
| stance probe on rubble, rough, stairs, obstacle course; cliff void probe (12/12 fired) | 0 (closest 69.8 measured) | 0. The obstacle-course box touches the tub for 122 steps (terrain, not a leg) |
| **righter recover1 in FALLEN** | seeds 0–99: 29/100 drops, 88 / 15,536 ticks (0.57%). **Pooled 200 seeds (verifier): 48/200 drops, 136 / 30,810 ticks (0.44%)** | 4/100 drops (legs 1, 2, max 2.36). Pooled 200: **6/200 drops, legs 1, 2, 3**, max 2.36 |
| RIGHTED stall ramp | 0 in v1 over 200 seeds. In v0, judged against the tub, 21 ticks up to 15.75 (seeds 114, 193): **not structurally clear** (verifier) | 0 |
| drop fixture (RecoverEnv samples q0 over the whole joint box) | 5/100 drop poses start inside the tub (up to 12.07); 4/100 on seeds 100–199 | 5/100 at t ≈ 0 as the stance unfolds them, max 1.97 |

"The belly raises the righter's excursions from 1/100 to 29/100": **refuted
(verifier)**. Pooled over 200 seeds it is 9 vs 48 drops, about 5×, not 29×.
Outcomes are unchanged (section 4).

### What the limit cut costs

**The cut.** The smallest single-joint cut raises the hip lower limit from −70
to **−51.05** (one box for every leg). The per-leg values are legs 1/4 −51.05,
legs 2/3 −59.35 and leg 0 −69.0 (for the shelf). A knee cut (≥ −90.75) is not
viable: it clips 26.3% of the gait swing, sit, beckon and point_there.

**Margin (refuted, verifier).** −51.05 has zero margin: legs 1/4 clear the tub
by 0.03. In the sim the joint overshoots its limit to −51.96 … −52.14 (falls)
and −52.12 (BRACE), and at those angles legs 1/4 reach **1.0–1.2 into the
tub**. A robust cut is about **−50.0** (+1.20 nominal) or **−48.35** (+3.04).
Both are on sim capsules, which are smaller than the CAD solids.

Cut tested in trees v0_limits / v1_limits (−51.05 in params, the whole stack
reads it; verified):

| item | uncut | cut −51.05 | verdict |
|---|---|---|---|
| eval_recover recover1 --supervisor, seeds 0–19 | 20/20 (11/20 no righter) | 20/20 (11/20) | confirmed |
| recover1 + supervisor, 200 seeds (verifier pooled) | 197/200 | 199/200 | confirmed, not significant |
| no righter, seeds 0–99 | 43/100 | 44/100 (the drop fixture rescales with Q_LO) | confirmed |
| leg-tub contacts on the fall path (v1) | FALLEN 4/100, drop 6, NORMAL 5 | 0 / 0 / 0 (200 seeds) | confirmed |
| FALLEN → ramp, paired median | — | +0.27 s (no tub), −0.40 s (tub) | confirmed |
| run_sim walk / turn | 189.281 / 41.829 | identical | confirmed |
| gesture audit | 19 rows, 0 FAIL | identical (1704 values, 0 differ) | confirmed |
| Playground gait, 24 headings | — | bit-identical end poses | unverified beyond spot checks |
| BRACE / stance probe / cliff probe | hip to −58.6 / −60.84 / −51.58 commanded | **the same commands: the supervisor and Playground do not clamp to `joints.pos_deg`.** Only the MJCF range (sim) or the driver clamp (hardware) stops them. Stairs h20 ends 17.1 short (876.0 vs 893.1). BRACE ends within 1.1. Cliff 12/12 held | confirmed (verifier corrected the ranges: BRACE −58.6, cliff −51.58, rubble −51.97) |
| righter's action map | −70 … 90 | remapped: action 0 moves from hip 10° to 19.475°. Clamping the old map gives the same outcomes | confirmed; the map shift is new |
| reach lost (FK) | — | deepest foot −163.3 → −147.9; tallest stand at the stance radius 163.3 → 144.7 (guarded 162.0 → 141.9); innermost foot at z −118: plan r 86.2 → 125.6; under the station (leg x 0) z −139.1 → −98.7; **11.8%** of the planar workspace | confirmed (verifier's FK within 0.3) |

**Cost in clipped ticks.** Ticks commanded below −51.05: the righter in FALLEN
77.2%, the RIGHTED ramp 23.7%, BRACE 73 / 6,200 (worst 7.55°), the stance
probe 271 / 17,600 (worst 9.49°), the cliff probe 103 / 8,275 (worst 0.53°).
With the checker's 2° guard: BRACE 110, terrain 351, cliff 270. The gait,
turns, ramps and gestures: 0 (their lowest hip is −41.38).

**Combination keep-out.** Forbid hip < −51.05 AND knee < −90.75 on legs 1–4.
This clips no gait, gesture, probe or BRACE tick, 9.7% of the righter's FALLEN
ticks, 1.1% of RIGHTED ticks and 25/100 drop poses. As an exact keep-out of the
reach set (the "model" option) it clips 0.57% of FALLEN ticks. Source:
`S/exp_motions/cut_cost.py`; recount confirmed by `S/verify_motions/v_cutcount.py`.

### What modelling the belly costs

Massless belly boxes (v1, v2) leave run_sim, the gesture audit and rubble
bit-identical until the first belly contact. What changes:

- **Fingerprint.** 87215110e9c4 → 777fdb4eda14 (v1), 4f77d216a700 (v2),
  47435187271f (v1m). Massless geoms still change the hash.
- **Checkpoint warnings.** `recover7_d063_curriculum` goes "match" →
  "mismatch" with a warning. `recover1` (shipped) and the other legacy
  checkpoints have no fingerprint: they load silently ("unknown").
- **Geom ids shift by +2** (belly_tub is id 1). No repo code indexes geoms by
  number; data keyed by index from v0 does not carry over.
- **SELF_CONTACT gains belly hits.** `pebble_feasibility` counts torso-to-leg
  penetration over 1 mm, so it will flag belly penetrations it never flagged
  on v0.
- **Friction.** Belly-to-floor friction is 1.0 (the max of the geom default
  1.0 and the floor's 0.8).
- **With mass (v1m):**
  - robot 2727.7 → 2755.0 g;
  - torso 1439.7 g at z 24.5 → 1467.0 g at (−2.5, −2.0, −1.2);
  - robot CoM at stance (0, 0, +2.76) → (−1.33, −1.07, −10.71), within 0.4
    of the proposal's (−1.3, −1.0, −10.3);
  - run_sim 189.32 / 41.83.
  - `pebble_feasibility` still uses M_TORSO 1439.8 and TORSO_COM z 24.5, so
    its margins do not see the new mass (gate INFO).
  - Trap: RecoverEnv's domain randomiser restores body mass, inertia and ipos
    from copies taken at build time, so an in-memory mass edit is silently
    undone unless `env.dr._base_*` is updated too (verifier).

**Leg 0 and the shelf.** Leg 0 reaches the shelf box (15 poses), and
legs 0, 1 and 4 reach the packed shelf boards and the new posts by 5–9
(`S/boards_verify/v6_leg_reach.py`). Any keep-out or limit rule must cover
the shelf too, not only the tub.

---

## 4. Decisions 1, 2 and 10: the hub, the tub depth, the tub's ends

### Righting (shipped recover1 + supervisor)

| variant | seeds 0–19 (published form) | 100 drops (0–99) | 200 drops (verifier, 0–199) | no righter, 0–99 | no righter, 200 |
|---|---|---|---|---|---|
| v0 | 20/20 (11/20) | 99/100 | 197/200 | 43/100 | 87/200 |
| v1 (8 mm) | 20/20 (11/20) | 99/100, 0 discordant vs v0 | 197/200, 0 discordant | 43/100 | 87/200 |
| v2 (5 mm) | 20/20 (11/20) | 99/100, 0 discordant | 197/200, 0 discordant | 43/100 | 87/200 |
| v1m (8 mm + mass) | 20/20 (11/20) | 98/100 (seed 90, 0 tub contacts) | 196/200 (p 1.0) | **51/100** | **104/200** (17–0, p ≈ 1.5e-5) |

- **The tub's shape changes no outcome, at 8 mm or at 5 mm.** Confirmed over
  200 seeds.
- **The v1m gain comes from the lower CoM, not the keel.** Putting v1m's
  torso inertial into v0, with no tub, gives 104/200. Lowering the CoM z alone
  gives 103/200. Mass and inertia alone give 87/200 (verifier,
  `S/verify_righting/mass_split2.py`, `mass_split3.py`). The gain depends on
  the battery mass really sitting in the tub. The inertia is an ESTIMATE.
- **Timing.** Settling onto the keel improves the tilt by more than 10°,
  which restarts the 3 s stall clock. The ramp then starts later: paired
  median +0.74 s (seeds 0–99), +0.77 s (100–199), worst +2.0 s (seed 40). The
  worst time to stand is still lower than v0's: v1 9.72, v2 9.84, v1m 10.24 vs
  v0 10.30 s. Confirmed (`S/verify_righting/stall_clock.py`).
- **New resting attitude: upright on the keel, feet off the floor.** Torso
  65–70 mm up, tilt 9–16°. The ramp starts from it in 33/57 (v1) and 34/57
  (v2) of declared falls, and stands it as fast as v0's legs-only pose:
  0.91 vs 0.93 s.
- **The shelf** touches the floor at most 0.28 s in FALLEN. No leg ever
  touches it during righting.
- **Not modelled.** The sim has no carapace dome: upside down, it rests on the
  femurs. Back-landing dynamics on the real shell are not in this test.

### Rubble and steps

| item | v1 (8 mm) | v2 (5 mm) | verdict |
|---|---|---|---|
| run_stuck 30/35/40/45 mm, seeds 0–3 | identical to v0: bare 2/4, 2/4, 1/4, 1/4; watchdog 4/4, 4/4, 2/4, 1/4; 0 falls | identical | confirmed (byte-identical logs) |
| tub contacts at 30–45 mm | 0 in 96 + 96 trials (seeds 0–23) | 0 | confirmed |
| closest tub-to-box gap at 30–45 mm | 14.7 (seeds 0–11); 14.3 (12–23) | 17.7; 17.0 | confirmed within 0.4 |
| first tub contact (rubble height) | 62 mm (1/12 trials); verifier 0/24 at 60–62 mm on seeds 6–17 (closest 1.55) | 64 mm (1/12); verifier 0/24 at 63 | rare, seed-specific |
| tub changes an outcome | only at 64–70 mm: 68 and 70 mm bare seed 1 (v0 crosses 452 / 460, v1 stops at 200 / 199); high-centring from 64 mm (seed 3) | only 70 mm bare seed 1 (460 → 330) | confirmed; v0 itself crosses 1/6 there |
| mechanism of the headline stalls | **side jam of the tub's square nose corner** against a box taller than the tub bottom (63.6 and 65.2 mm boxes, horizontal contact normal) | ride-over drag on a box top | **refuted (verifier):** not friction on a box top, as first reported. Seed 3 is a real ride-up and prop |
| tilt and gap | tilt does not bring the tub within about 14 of a box at 30–45 mm | — | conclusion holds. The "≥ 20.1 above 8°" bound is refuted (18.3 at 9.9°; min 14.3 at 6.9°) |
| stairs 4 × 15, rubble-field, rough, ramp lane (cockpit goto) | arrived, identical to v0, 0 belly contacts; stairs gap 45.3 | 48.2 | unverified (not re-run) |
| risers ≥ 20 mm | the walker cannot cross them even without a belly, so no strike can be measured | — | unverified. The verifier notes that the 418–420 mm stop may be the goto's own guard |

### What this says for each decision

**Decision 1 (where the hub lives).**

- **The shelf is clear of everything it was checked against.** Option A's
  shelf box has 0 mm³ with hooks, drops, tub and deck, and 0 contacts in
  righting and rubble (shelf bottom 79.55 above the floor).
- **There is more room than the box uses.** The free zone north of the tub is
  381,718 mm³ against the box's 105,840. The biggest box is
  x −50.0 … 49.5, y 12 … 56.5 at z −38 … −10 (unverified).
- **Three problems are new:**
  - leg 0 reaches the shelf box, and legs 0, 1 and 4 reach packed boards;
  - the proposal's hangers block the adapter cases;
  - the stand's cradle walls and crown plate collide with down-facing boards.
- **B and C were not measured here.** Righting and rubble cover the tub and
  shelf boxes only.

**Decision 2 (8 mm or 5 mm layer).**

- **Righting:** no difference over 200 seeds.
- **Rubble:** no difference anywhere the walker can cross. The 5 mm layer
  moves the first touch up by 2–3 mm of rubble height, at 62–64 mm against a
  45 mm walking limit.
- **Leg reach:** the 5 mm tub is reached in *more* poses (117 / 104 vs
  114 / 97).
- **Hook margin** (confirmed): the 8 mm layer keeps 3.21 to the hook envelope
  (2.97 with a 5.2 slot). The 5 mm layer keeps **0.21** as drawn and
  **overlaps** by 0.09–0.16 mm³ with a 5.2 slot.
- **The I1 hook redesign is unsettled** (section 1, rows 1–1b), so its
  envelope will move. Only the 8 mm layer has margin for that.
- **On these measurements: 8 mm.**

**Decision 10 (the tub's protruding ends).**

- **The nose corners cause the stalls.** The only stalls the tub causes, at
  64–70 mm rubble (outside the walking range), are the square nose corners
  jamming sideways (verifier).
- **The nose-south corner sticks out.** At (101.9, −47.4) it sits at
  r 112.4, 2.4 past the 110 mm torso disc. It jammed in 70/1, 64/3 and 65/3.
- **Chamfer or radius the nose corners and the nose-bottom edge.** This
  plausibly turns jams into ride-overs (ESTIMATE, not simulated). It costs
  nothing in the crossable range.
- **Overhang past the deck** is confirmed at 18.0 / 26.5 / 9.5 (door) and
  22.2 / 30.7 / 13.7 (nose), centreline / south / north. At bay_w 53 the
  south corners reach 26.94 and 31.12.

---

## 5. Published numbers that stop being true once the robot has a belly

| number | source | today | with a belly | evidence |
|---|---|---|---|---|
| robot fingerprint | CLAUDE.md, README, docs/SIM_GUIDE.md, docs/RL_GUIDE.md, docs/decisions.md, docs/DESIGN_BACKLOG.md, docs/COCKPIT_GUIDE.md, docs/DESIGN_CHANGE_GUIDE.md, docs/ROBOT_AS_DATA.md, sim/experiments/README.md, BUILD_LOG.md | 87215110e9c4 | 777fdb4eda14 (8 mm, massless) / 47435187271f (8 mm + mass) / 4f77d216a700 (5 mm); with the hip cut too: c76977b87e64 | `S/belly/manifest.json`, `S/belly_verify/extras.py` |
| robot mass | SIM_GUIDE, RL_GUIDE, decisions, DESIGN_BACKLOG, bench/BENCH_RUNBOOK.md | 2727.7 g | 2755.0 g (proposal mass, 343 g pack; ESTIMATE inputs) | `S/belly_verify/vchk.py` |
| sim torso mass / CoM | sim/mass_budget.json; BODY_LAYOUT_PROPOSAL §6 (quoted as 1434.6 g at (0, 0, 24.5)); `pebble_feasibility` M_TORSO 1439.8, TORSO_COM z 24.5 | 1439.7 g at (0, 0, 24.5) | 1467.0 g at (−2.5, −2.0, −1.2); robot CoM at stance (−1.33, −1.07, −10.71) | same; `S/belly_verify/feas_com.py` |
| recover7_d063_curriculum fingerprint status | RL_GUIDE ("on `87215110e9c4`") | match | mismatch (warns). recover1 stays "unknown" and loads silently | `S/belly/checks/variant_checks.py` |
| no-righter system baseline | README, RL_GUIDE, SIM_GUIDE, decisions, DESIGN_BACKLOG (11/20) | 11/20 | still 11/20 on seeds 0–19. With the mass, 51/100 on 0–99 (43 today), 104/200 vs 87/200 | `S/exp_righting/out/inst_*`, `S/verify_righting/my_summary.json` |
| belly ground clearance | BODY_LAYOUT_PROPOSAL §2 / §6, DESIGN_BACKLOG | 78 as drawn (no part); proposal 62.6 / 65.6 | 62.15 (8 mm), 62.13 (with mass), 65.15 (5 mm), settled | `S/belly_verify/vchk.py` |
| legs reaching the tub | BODY_LAYOUT_PROPOSAL §2 / §10.8, DESIGN_BACKLOG B97 | ~205 poses per leg; leg 0 never | 114 / 97 / 97 / 114 (sim capsules); leg 0 reaches the shelf (15 poses) | `S/exp_motions/out/reach_summary.json` |
| latches blocked by the tub | BODY_LAYOUT_PROPOSAL intro, DESIGN_BACKLOG B100, `iface.py` comment | one (station 162) | two (162 and 306) | `S/keepouts/m3_tub.py` |
| "nothing gates the leg poses" | BODY_LAYOUT_PROPOSAL §3 risks, B97 | true | the audit and the cockpit studio gate them through SELF_CONTACT; the righter does not | `S/exp_motions/selfcontact_probe.py` |
| recover1 declared-fall time to stand (unpublished; RL_GUIDE has only the overall t_stood median) | — | median 7.19 s, worst 10.30 | 7.92 s, worst 9.52 (v1). The overall t_stood median 0.46 s is unchanged | `S/exp_righting/out/timing_righter.json` |

Unchanged with a belly (confirmed): recover1 system 20/20, run_sim walk 189 mm /
turn 41.8° (v1m 189.32 / 41.83), the gesture audit (19 rows, 0 FAIL), the
gait envelope 34.2 mm/s / 0.185 rad/s, and the run_stuck table
(sim/experiments/README.md: 2/4 → 4/4, 2/4 → 4/4, 1/4 → 2/4, 1/4 → 1/4, tilt ≤ 12.6°).

Stale today, without a belly: RL_GUIDE's recover1 t_stood medians 0.49 s / 0.42 s
measure 0.46 / 0.38 (counts exact).

---

## 6. Scratch artefacts worth promoting during the build

| scratch path | becomes |
|---|---|
| `S/keepouts/keepouts.py`: `hook_profile` / `hook_envelope(mode, slot_w)` | `iface.leg_port_hook_envelope()`. Use the deck-clear "free" mode or the union, not pivot_E; take the bottom from params; re-derive after the I1 redesign |
| `S/keepouts/keepouts.py`: `drop_keepout(grow=0)` and `drop_connectors()` | `iface.leg_drop_keepout()`, with the ±10.5 × 35 convention (already cutout + 5) written in its docstring |
| `S/keepouts/keepouts.py`: `tub()`, `shelf()`, `latch_axes()` + `driver_column()`, `outside_deck()` | `part_bay` checks: tub × hooks, drops, deck, carapace; both latch driver columns (162, 306) open; overhang past the deck printed |
| `S/keepouts/dance_cspace.py` + `S/keepouts_verify/v14_sound_margin.py` (the latter includes the dowels) | an I1 docking-path check for `iface` / `part_coxa`: a planar C-space path exists with ≥ 0.05 mm clearance. It fails today, which documents the hook problem |
| `S/keepouts/m0b_base_shell.py` | a `part_shell` check against the real posed `coxa_yaw_base`, not proxies (42.8 mm³ today) |
| `S/keepouts/m6_drop_and_smoke.py`, `S/boards/b6_grommet.py`, `b6b_slot.py` | a connector pass-through check (cross-section diagonal against each cutout, channel and grommet) for `part_deck` / `part_coxa` |
| `S/boards/b2_margin.py` (+ `S/boards_verify/v2_margin.py`) | the `part_avionics` carapace check from section 11, with a Pi part-height envelope (~16.0), the 3 mm port overhang, the tray float ±0.3 and a 3D gap |
| `S/boards/b5_tray_grow.py`, `S/boards_verify/v5b_assembly.py` | `part_avionics` checks: lug release (slide 16, lift) against the posed bases, yaw servos and forks; carapace lowering over the tray |
| `S/boards/freezone.py` + `S/boards/b4b_pack.py` + `S/boards/b4c_verify.py` | the hub shelf's `layout_audit` (re-targeted `part_busboard`) and a packing experiment. The keep-out set must add the cradle walls, the crown plate, the 14 AWG run to its node and leg reach (`S/boards_verify/v4_arrangements.py`, `v6_leg_reach.py`) |
| the BELLY VARIANT block in `S/belly/trees/v1_a8/sim/build_mjcf.py` (v1m adds the `<inertial>`) | belly geoms in `sim/build_mjcf.py` driven from params (`bay_top_z`, bay size and centre, shelf box) |
| `S/belly/checks/composite_inertia.py` | a torso mass / CoM / inertia composite from CAD volumes for `mass_budget.json`, replacing the hard-coded TORSO_COM z 24.5 (also in `pebble_feasibility`) |
| `S/belly/checks/variant_checks.py`, `S/belly_verify/vchk.py` | a sim test: belly clearance at stance, the contact filter (belly against floor and legs), the example-pose contact, the per-leg reach census |
| `S/exp_motions/bellylib.py`, `reach_grid.py`, `reach_analyze.py`, `cut_cost.py` (reach set in `out/reach_set.json`) | a `pebble_feasibility` tub/shelf keep-out rule (the "model" option or the hip-AND-knee combination) and `sim/experiments/run_belly_reach.py` (census + cut costs) |
| `S/exp_limits/stack_check.py` | a test that every layer reads the hip limit from params (rocky_model, ik_ok, feasibility, gestures2, RecoverEnv / righter Q_LO, MJCF, driver) |
| `S/exp_limits/reach_loss.py` | an experiment that prints the FK workspace lost by any limit change |
| `S/exp_righting/righting_instrumented.py` + `analyze.py` (validated against `eval_recover.run_supervisor` on 800 episodes); `S/verify_righting/stall_clock.py`, `mass_split2.py` | `sim/experiments/run_righting_belly.py`: per-phase contacts, stall-clock restarts, mass/CoM attribution. Note the DR-restore trap in its docstring |
| `S/exp_rubble/trial_belly.py` + `S/verify_rubble/detail.py` | an option on `run_stuck.py` that logs tub gap and tub contacts (partner box, normal) |

---

## 7. Still unmeasured

**I1 hook and dock**

- A buildable I1 docking path. It needs a design choice: post height, bore
  play or chamfer, or a shorter foot. The hook envelope cannot be frozen
  until then.
- Whether the hooked "connector-mating" state needs about 19.8° or more of
  tilt to clear the dowel tips.

**Leg and tub contact**

- A CAD-side leg × tub census with the real leg solids and the tub with its
  bosses, lid and latch. The ~205 count and the cut value depend on it.
- The tub with hanger bosses, door latch boss, deck underside and carapace as
  collision geometry.
- The legs' soft-limit reach under the shelf with real board envelopes. Only
  sim capsules have been checked.
- Righting with the hip cut in v1m and v2.
- Righting under the randomized-servo / DR condition, with the obs-v2
  checkpoints (recover6, recover7) on belly models, and on the shove-to-fall
  path. No checkpoint has been trained on a belly model.
- How the righter behaves on a real carapace dome. The sim has none.
- The real tub material and its friction. The sim uses MuJoCo defaults.

**Mass**

- The v1m inertia tensor and its 28.2 g residual (ESTIMATE). There is no
  "5 mm + mass" variant.
- Pack mass 380 g (params) vs 343 g (BOM), decision 6. v1m assumes 343.
- Worst 4-foot stability margin with the belly mass. `pebble_feasibility`
  still uses the v0 mass model.

**Pi and boards**

- The Pi 5's real part height (about 16.0), with the Active Cooler fitted,
  and its 3 mm port overhang.
- The bus adapter's connector edges and plug halos (Waveshare STEP not read);
  UART mode on the bench.
- Board envelopes (ESTIMATE): star stack 22.4, PDB with leads, the 1000 µF
  capacitor, lead halos.
- The stand under option A: U-cradle wall thickness, whether the crown plate
  stays, the x stop.
- The 14 AWG route from the tub nose to the node.
- Pads and arms from the shelf plate to the new posts (0.57 to the drops in
  one estimate).
- 5 V 5 A to the Pi over Dupont: contact rating and voltage drop (ESTIMATE
  30–70 mV).
- Shelf case (i), the adapter alone on the shelf (3.00): not re-checked
  against the cradle, the crown plate, the 14 AWG run or leg reach.

**Connectors and grommet**

- Mated XT30 length (20.1) and the 8 mm wire-bend allowance under the cutout
  (ESTIMATE).
- The Ø10 grommet alternatives (Ø12.5 hole or 15.5 × 7.5 slot): unverified.

**Rubble, shapes and other options**

- Nose chamfer or radius against the 64–70 mm corner jams (not simulated).
- A crossable stair or riser world with less than 62 mm under the tub.
  Whether the goto's 418–420 mm stop at ≥ 20 mm risers is its own guard.
- Options B and C in the sim.
- Coxa base × carapace clash (42.8 mm³): unverified, and no fix designed.

---

Repo check: every agent and verifier reported `git -C REPO status --short`
empty. Re-run when this report was written (2026-10-01): empty output, rc 0.
(The boards agent once used `pkill -f` on its own scratch runs; services were
checked active afterwards.)
