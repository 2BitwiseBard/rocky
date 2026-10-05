# I1 leg port: three ways to make it dock

Read-only judgement, 2026-10-02. Nothing in the repo changed (`git status --short` printed nothing).
Lengths in mm, volumes in mm³, angles in degrees. Leg-local frame: +x outboard, plate z −4..0, deck z −10..−4.
Poses are (tx, tz, tilt) of the coxa base relative to docked; + tilt lifts the outboard end.

Folders: `J` = `prep2/i1_judge` (this judgement), `M` / `S` / `Q` = `prep2/i1_minimal`, `prep2/i1_seats`,
`prep2/i1_sequence` (designers), `MV` / `SV` / `QV` = their verifiers (`..._verify`).

## 1. The fault

- Today's hook L is 5.15 wide at the bottom. The deck slot is 4.2 wide.
- My own planar search finds no path from above the deck to docked at slot 4.2 or 4.6, tilt −30..90, even at zero clearance. At slot 5.2 the hook alone does dock (`J/j1_repro.py`, `J/j1.log`).
- The 8 mm dowel posts then block the dock: with them there is no path at 0.10 clearance at slot 5.2 or 7.0 (tilt −5..30, tx −3..5; `J/j1_fine.log`). At 0.30 the docked pose itself is not free (dowel play is 0.175).
- So today's leg cannot be docked or undocked at all.
- Second finding: the stance load picture in the brief is upside down. In stance the foot pushes the plate's outboard end UP. A down-pull can only hold that if a deck contact lies inboard of it. The hook foot is the most inboard part, so it carries nothing in stance. The thumbscrews carry it on a 4.9 lever: 972 N per pair in the design case, or 660 N without foot friction (`J/j2_statics.py`, `J/j2.log`). The brief's picture is right only for a hanging leg (hook alone 26.7 N at ×3).

## 2. The three designs side by side

| | **minimal** | **seats** (+ my graft: pads end at x −46) | **sequence** |
|---|---|---|---|
| Idea | Same family. Wider slot, short foot, 3 mm tapered posts in tapered bores. Tilt about 8°, drop, pivot | Two 30° cone posts. One conical socket and one short V socket. Slot widened outboard. Plate bridges the slot. Spine rib on the plate. Hook at 7–10°, lower onto the cones | No posts. Lower the plate flat 3.3–5 outboard, slide inboard to a stop. An inner L plus a new outer chevron hook under the deck. Knob skirts take the shear |
| Parts added | none (reshaped features) | 2 cone posts, 2 sockets, a 0.2 relief, a rib (all printed into existing parts) | a chevron on the base, skirts on the 2 knobs, 10 deck grooves, Ø10 plate bores with ears |
| Docks and undocks (verifier) | yes, no fatal flaw | yes, no fatal flaw | yes, no fatal flaw |
| Smallest clearance on the dock path | 0.300 at the posts when docked. 0.3035 en route, on the deck's first-layer slot edge. A 0.351 path exists | 0.300 in 3D: the lip-end gap at the slot ends, constant. In-plane 0.40 at tilt ≤ 10 and up to 0.495. The cones close by design in the last 0.61 of drop | 0.300 along the whole slide and when docked. That corridor has zero thickness in z: it holds only while the plate slides on the deck top |
| Docked play | x ±0.305, y ±0.30, yaw ±0.91 (today ±0.53) | 0 in nominal CAD. With print error: still 0 up to about 0.05 radial socket oversize. At 0.1: x ±0.05, yaw ±0.08. At 0.2: x ±0.15, yaw ±0.25 (`J/f2_relief.out`) | x/y ±0.15 (±0.30 with groove float), yaw ±0.51 (±1.0) |
| Shear: yaw stall 2.94 N·m | 77 N per post. Post root SF 1.04 (load at mid-flank), 0.52 (load at the tip) | 85 N per seat. SF 1.86–1.95 for the couple alone. SF 1.23 for stall + friction + 30 N shove all on one post | 86.5 N per skirt. SF 1.97 alone, 1.49–1.58 combined. But the knob seat on the plate is only 9.3 mm² |
| Stance tension (design case: 3 legs ×3, μ 0.5) | Thumbscrews 972 N per pair, on a 4.9 lever | Thumbscrews 568 N per pair, on an 8.55 lever | The knobs carry it first, because the chevron has a 0.30 gap: 384–800 N per pair (QV). Chevron alone would need 164 N |
| Plate at the screw line (J/j3) | 46.2 MPa, SF 0.76 (31.0 / SF 1.13 without friction). Same as today: 47.4 MPa, SF 0.74 | 15.0 MPa, SF 2.33, because of the rib | **66.8 MPa, SF 0.52** (44.8 / SF 0.78 without friction): the Ø10 bores cut the section to 105 mm² |
| Hanging leg | Hook foot 1.5 × 24 = 36 mm², 0.74 MPa, engages after 0.35 of lift | Hook is a catch (1.5 slack). Screws 105 N per pair (leg CoM x 93) | Inner L 14–28 N, 0.38 MPa |
| Lowest hook z / margin to a tub roof at −18 (at −15) | −12.48 path, −13.49 any reachable pose / 5.52 (2.52); 4.51 (1.51) | −13.30 path, −13.76 any pose / 4.70 (1.70); 4.24 (1.24) | −12.59 path, −13.25 extreme hand tilt / 5.41 (2.41). Plus a NEW under-deck occupant at each deck corner, r 85.8–109.1 |
| Swap with the tray in | yes, with a low approach (13.5 up, from 24 outboard). Straight down hits the Pi at legs 1/4 (71–100 mm³) | yes, radial approach at about 10.8 up. At leg 4 keep the hooked tilt ≤ 10 (the Pi touches at 14.8) | yes. The leg the Pi's USB end faces must come in low (z ≤ 9). A drop from high touches the Pi stack |
| Swap with its own sector on | no (634–1011 mm³) | no (1011 mm³) | no (268 mm³ at z 40) |
| Docked base vs own sector | 42.801, unchanged | 42.801, unchanged | 42.801, unchanged |
| Print risk | Low. The bottleneck sits on the deck's first-layer slot edge: elephant foot eats the 0.30, so chamfer it. The foot top is 0.35 under the deck | Medium. Sockets open on the supported face (needs a support blocker). The 0.2 relief and the pads are on the support interface. The V socket wall is exactly 1.20 | High. 223 mm² of 1.2 deck walls (groove collars). Supports land on the chevron foot's 0.30 face. The −y chevron arm is not fused to the plate where it crosses the harness channel. Knob seat crush |
| Frozen values changed | 3: hook_slot_w, hook_slot_x, hook_foot (new meaning) | 3: dowel_xy → seat_xy, hook_slot_x, hook_slot_w | 5: dowel_xy (deleted), hook_lip_w, hook_slot_x, hook_slot_w, hook_foot (new meaning) |

Where the numbers come from:
- Designers and verifiers, unless marked J. Where they disagree, the verifier's number is shown.
- J/j3 is the plate section measured on each design's real solid, under the same moment: 4629 N·mm with friction, 3104 without (`J/j3_section.py`, `J/j3_<design>.log`).

## 3. Recommendation: **seats**, with the pads ending at x −46

Why seats:
- It is the only design that fixes the port's real weak points as well as the dock.
  - Docked play is nominally zero. The others are ±0.91 (minimal) and ±0.51 (sequence).
  - Seat shear SF is 1.23–1.95. Minimal's posts are at 0.52–1.04.
  - The spine rib brings the plate at the screw line from SF 0.74 to SF 2.33. Today's plate already fails the design case. Minimal leaves it at SF 0.76. Sequence makes it worse: SF 0.52.
  - The stance pull on the thumbscrews drops from 972 to 568 N per pair.
- It has the widest docking margin. In-plane 0.40 at tilt ≤ 10, up to 0.495 (SV). The designer's "0.378 ceiling" was a solver artefact.
- It changes only 3 frozen values. It keeps the L and the thumbscrews where they are.

The graft (`seat_relief_x0` −42 → −46):
- Problem it fixes: with the screws (x −41) outboard of the pad edge (x −42), the seats got only 8% of the screw preload (SV, J/j6).
- With the pads ending at −46, the seats get 30% (`J/j6_preload.py`). The zero-play window grows a little: from 0.05 to about 0.06 radial (`J/f2_relief.out`).
- Proof that it still docks:
  - It only removes material: 34.45 mm³, and the graft minus the seats base is 0 mm³ (`J/j4_subset.py`, `J/j4.log`). So every planar path proven for seats keeps at least its clearance.
  - 3D replay on the real solids (`J/r1_replay3d.py` on `J/cad_graft`):
    - designer's 10° hand path: 271 poses, 0 mm³, minimum 0.300 (the lip ends). With the lip ends trimmed: 0.305;
    - SV's 0.40 path: 52 poses at 0.4006 trimmed; 358 dense poses, 0 mm³.
  - Docked: 0 mm³ with seat contact. The relieved face sits 0.200 over the deck.
  - Printability audit: no hard failures (`J/j5.log`).
- Cost: the stance fulcrum now bears on two pads of 40.5 mm² in total, at 13.4 MPa mean in the design case. The coupon decides between −42 and −46 (section 6).

What would make **minimal** preferable: the seat coupon fails. That means the cones do not seat without rock, or the play exceeds about ±0.15 at every socket offset. Minimal would then need three things:
- the seats' spine rib (otherwise its plate is at SF 0.76);
- bigger posts (dowel_d 5, plate 47 wide; owner call) to lift the post SF toward 2;
- a 0.5 × 45° chamfer on the slot's bottom edges.

Its ±0.9 yaw play would remain.

What would make **sequence** preferable: the owner values a hands-free hooked state (it holds itself while you plug the leads) above everything else. It would need all of these, and a new proof after them:
- a re-routed harness channel, so the −y chevron arm is fused;
- a knob seat ring;
- a plate section restored at the Ø10 bores (ears or a rib);
- chamfers on the foot tips and the deck's bottom edges.

## 4. What the owner signs off (seats + graft)

| item | old → new |
|---|---|
| **dowel_xy** (frozen) | [(−28, 19), (−28, −19)] → seat_xy [(−32, 17.3), (−32, −17.3)] |
| **hook_slot_x** (frozen) | −48.0 → −47.0 |
| **hook_slot_w** (frozen) | 4.2 → 6.2 (the inboard edge stays at −50.1; it grows outboard to −43.9) |
| dowel_d, dowel_engage | 4.0, 8.0 → removed |
| new seat params | seat_r 2.9, seat_half_angle 30, seat_h 3.0, seat_vee_len 0, seat_vee_slack 0.3, seat_roof 0.6, seat_mouth 0.3, seat_relief 0.2, **seat_relief_x0 −46.0** (the designer drew −42.0; this is the graft) |
| new hook params (values unchanged) | hook_stem_gap 0.55, hook_foot_gap 1.5, plate_ext_half_w 18.0 |
| plate | runs inboard to the stem (x −49.55) for \|y\| ≤ 18 (replaces the 2 mm reach) |
| coxa_yaw_base | spine rib x −46..−38, \|y\| ≤ 8, z 0..6 |
| insertion | tilt 7–10 with a radial approach (the docs say about 15). Own carapace sector off |
| unchanged | thumbscrew_xy (−41, ±17), hook_lip_w 24, hook_lip_t 3.0, hook_foot 3.5, cable cutout, heat-set pockets |

## 5. What changes in the repo (when the owner signs)

CAD:
- `cad/params.yaml` leg_port: every row above. `S/cad/params.yaml` holds the text; set seat_relief_x0 to −46.
- `cad/iface.py`: `_seat_frustum`, `leg_port_seats`, `leg_port_sockets`, `hook_geometry`, `leg_port_deck_features`, `leg_port_plate_features`. Copy them from `S/cad/iface.py`. Fix the stale docstring "slot widened inboard": it is widened outboard.
- `cad/part_coxa.py`: the I1 rib constants. Fix the stale comment about the dowel bore near HARNESS_X0.
- `cad/part_deck.py`: the layout audit labels (dowel_xy → seat_xy).
- `cad/part_port_coupon.py`: cut the plate patch to x −46..−11. Today's patch at x −57 can never dock.
- `cad/part_bench_jig.py`: the print line.
- `cad/part_shell.py`: port_hardware comment.
- `cad/gen_print_pack.py`: the row-D go / no-go text.
- `cad/gen_assembly_views.py`: the "pivot onto the dowels" caption.

Checks:
- `check_assembly` / `run_all_checks`: add a docked seat check (seats × base 0 mm³, lift gap = dz·sin 30).
- Add a dock-path check: port `SV/cs.py` + `SV/c1_search.py` (planar) and `J/r1_replay3d.py` (3D) as a regression.
- `cad-check --derived`: the fingerprint and the print pack change.

Coupons:
- `part_fit_ladder.py` row D: 5.8 / 6.2 / 6.6.
- Add a seat-fit row: sockets at radial offset 0 / +0.1 / +0.2.
- The 3.95 dowel pegs become obsolete for the port.

Docs:
- `docs/INTERFACES.md` I1, including the table row at about line 299: the seats; the load path (stance on the thumbscrews, the hook is a catch); 7–10° with a radial approach.
- `docs/ASSEMBLY_GUIDE.md`: 1.1 row D, the 1.2/1.3 port coupon steps, the batch-2 clean-up, the deck checklist ("ten printed dowel posts"), 6.1 dock steps 2–6, and the own-sector note.
- `docs/PRINT_PLAN.md`: row D, the port coupon, the support blocker in the sockets, the deck height (14.6 → 9.0).
- `docs/BODY_LAYOUT_PROPOSAL.md` fact 2: hook envelope −13.30 / −13.76, r 56.9–66.8.
- `docs/decisions.md`: a new D-entry.
- `docs/DESIGN_BACKLOG.md`: close the I1 dock fault.
- `docs/SCALE_UP_NOTES.md`: dowel mentions.

## 6. Print first, and what go / no-go means

1. **Port coupon pair** (deck patch + plate patch x −46..−11, seats + graft). Print two plates: seat_relief_x0 −46 and −42.
   - Go: the L passes the 6.2 slot by hand at 7–10° with the radial approach. It lowers onto the cones and centres itself. Clamped finger-tight, nothing rocks, and you feel no x or y play (< 0.05 on a dial).
   - No-go: binding at the slot means print the 6.6 slot. Rock or play > 0.15 at every socket offset means fall back to minimal (section 3).
2. **Seat-fit row**: sockets at radial offset 0 / +0.1 / +0.2. Pick the offset with no rock and no light under the pads (0.2 feeler).
3. **Socket print**: plate-down with a support blocker in both sockets. Go: clean cone flanks and a bridged Ø2.5 roof. No-go: support inside the socket.
4. **Insert pull-out and knob preload**, in the deck coupon.
   - Go: each insert holds ≥ 300 N pulled toward the plate (the stance design case is 284 N per screw), and a hand-tight knob gives ≥ 285 N.
   - No-go: an outboard hold-down is needed (owner decision, outside this fix).
5. **Cone post twist**: 85 N per seat (yaw stall), then 128 N on one post. Go: no crack at the root layer.
6. **Release test**: hooked at 10°, let go. It should come to rest on the deck vertex and the hook, then centre on the cones when the knobs go in.
7. **20 dock / undock cycles**: tips and mouths do not round off, and the play does not grow.

## 7. Open points

- **Stance hold-down.** The thumbscrews and their inserts carry the stance tension in every design: 568 N per pair for seats. Insert pull-out (ESTIMATE 400–900 N) and hand preload are unmeasured. The frozen wording "the thumbscrews only clamp" is untrue for all three designs.
- **The brief's stance statics are reversed** (J/j2). The hook carries nothing in stance. Correct this before other agents use it.
- **V-socket wall** is exactly 1.20 to the plate's −y edge. Proposed fixes: seat_mouth 0.3 → 0.2, or seat_xy \|y\| 17.3 → 17.1. Not re-proven here.
- **The 0.2 relief is one layer on a supported face.** A support z-gap is about that size. A 0.4 relief also only removes material, so it still docks. The screw-line section would need a re-check. Not run.
- **The zero-play claim is nominal.** Real play depends on print accuracy, with a window of about 0.05–0.06 radial. Coupons 1 and 2 decide it.
- **Not modelled:**
  - roll, yaw and y during hand docking (the planar proofs are x, z, tilt);
  - the femur and tibia during the sweep;
  - the tray rails' 2 mm riser (assumed) and the Pi's 16 mm part stack (ESTIMATE).
- **Pi end facing a leg.** With USB plugs at that end, there is less room than a USB-A overmold needs (MV: 10.6 docked, 5.9 during the minimal dance). Decision 14.
- **XH-5 housing** passes neither the 11 × 11 deck cutout nor the coxa channel. Pre-existing; no design fixes it.
- **The leg's own carapace sector must come off** for a swap. On an assembled robot that needs a flat screwdriver from under the deck (I3). So "no tools, under two minutes" does not hold with the carapace on.
- **B27** (deck_t 6 in the CAD, 4 in params) is still open. Every hook gap assumes 6.
- **First round's "no path at slot 7.0 with dowels at 0.05".** My test proves no path at 0.10 (within tilt −5..30, tx −3..5). At 0.05 it is inconclusive, as MV found.
