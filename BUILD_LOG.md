# ROCKY Build Log — Engineering Notebook

*Convention: newest entries first. Every work session gets an entry: date,
what happened, what was decided, what broke, what's next. Field notes and raw
measurements go in `NOTES_INBOX.md` and are filed later. Keep entries honest —
failed prints and dumb bugs are the most valuable lines in this file.
Sessions 1a–9k are archived word for word in `docs/archive/`; the table at
the end indexes them.*

---

## 2026-10-08 → 10-09 · Session 9r — the body leveler and its terrain bench (D065), landed off

**Ask:** nothing levelled the body. The gait stands its feet on one body-frame plane, so on an 8°
slope the body sat at 8.06° and a 20 mm stone under a foot left 3.88°; the gait residual
(`robust_fwd2`) had lost to the bare gait on flat ground. A design round (three designs, two
judges, prototypes in scratch) picked the smallest law that could work, with two grafts (per-leg
rate-limited copies; freeze while a foot feels for the floor). Build it, wire it into the sim,
bench it against the shipped stack, and ship it off unless the bench passes.

**What was built (branch `build/level`):**
- 6745690: `gait/pebble_level.py` `BodyLeveler`, numpy only. The IMU tilt error (0.2 s low-pass,
  0.5° deadband) integrates (τ 0.6 s, ≤ 20 mm/s at R0) into a plane of foot offsets in a 0–30 mm
  raise-only window, evaluated at each foot's xy (`WaveGait.level_xy`), followed per leg at 20 / 40
  mm/s. `ReflexSupervisor(leveler=)` opts in: `None` reproduces the D064 trace digest
  `a3ebd8f92a6a38f0`, and so do an off leveler and an on one inside the deadband. The envelope
  derates with the plane, `WaveGait.budget(level_slope=)`: 34.20 / 32.48 / 31.42 mm/s at 0 / tan 5° /
  tan 8°. `pebble_feasibility.margins(normal=)` now works without `support=`. `params.yaml`
  `level:` with `enabled: false`.
- e174f22: the sim. `SimIMU(mount_deg, mount_axis)`; the playground reads the IMU with the sim
  time (before this a latency spec was silently never applied); `Playground(imu=, level=)` and
  `set_imu`, kept across a respawn; `_make_sup` (the cockpit's respawn uses it); holds for the
  gate, the void, `probe_out`, a gesture and a late seek; `level on|off`, `set level.*`, the
  `level_*` guard fields, the HUD line; off while mirroring sim → robot, and sim → robot refused
  while it holds a foot up; `ROCKY_LEVEL=1|0`. `sim/terrain_bench.py` (79 rows, S0 / S1 / S2 ×
  ideal / noisy / mount1.5, S3 reserved) and `sim/experiments/run_lip_grid.py`.
- This commit: the records, the docs, and two speed fixes the test flake below led to.

**The bench** (`sim/out/terrain_bench.json`: 937 runs in 945 s at `--jobs 6` on e174f22,
`965f4f70e5d1`; ideal IMU unless noted, S1 → S2):
- Standing: 5° 5.04 → 0.07–0.24°; 8° 8.06 → 2.96–3.22 (the window saturates); 10° 10.08 → 4.98–5.24;
  the stone 10 / 20 / 30 mm 2.18 / 3.88 / 5.63 → 0.45 / 0.36 / 0.42. On a slope the body sinks: the
  height error over the loaded feet is −14.3 to −17.1 mm (S1 −0.2 to +1.1).
- Walking: the slopes' mean tilt RMS 7.59 → 2.91°, ramps 5.49 → 2.20, stairs 4.72 → 1.96 (tips
  2 → 0), rubble 2.20 → 1.57 (S0, no probe, 1.54); the 8° shove's peak 12.08 → 5.15.
- Flat (straight walks and the stand): bit-identical to S1 with the ideal IMU; under the noisy one the plane drifts 1.19–1.68 mm
  (tilt RMS +0.016…+0.037°).
- No S1 or S2 run fell or touched the belly; load ≤ 0.238 × stall; heat ≤ 0.2 % of the budget.
  The cliff grid fired and held 10/10 in every mode, the 5° slope cliff 2/2 (S0: 0/10 fire, 8
  tips; the slope cliff 1 fall). `run_sim`: 189 mm, 41.7°, in band.
- mount1.5 (not gated): the leveler levels to the sensor, 1.045° off on a flat stand, the flat
  walk's tilt RMS 1.31° (B159).

**The lip grid, first run** (`sim/experiments/run_lip_grid.py`, 364 approaches, on `965f4f70e5d1`): 356 stop,
8 tip, 0 fall, 0 short, worst safe tilt after a fire 5.69° (424 s); with `--level` the same 8 tip,
no outcome changes, 314 of 364 approaches identical, every void within 0.02 s (439 s). It
reproduces the 9q record.

**First-run verdict: fail, so `level.enabled` stays false.** Pass: P2 (the 20 mm stone ≤ 1.0°: 0.346–0.357
ideal, 0.161–0.165 noisy), P3 (8° ≤ 3.5, 5° ≤ 0.6), P5 (no new falls), P8 (load and heat), R1 (the
worst row +0.043°, a noisy cliff approach), P9 (below). Fail: **P1** under the noisy IMU (offsets
1.19 / 1.68 / 1.38 mm against 0.5); **P4** on 21 row × IMU pairs: 14 on the slopes, where a saturated
plane walks at the derated 95 % and MuJoCo's stance creep moves progress (S1 690 / 544 mm down / up
a 5° slope, 619 on flat; S2 91.7 % down, 99.6 % up, 95.1 % across), 4 on three rubble seeds (the
family 101.2 % ideal, 103.7 % noisy), 3 noisy stairs rows; **P6** once (noisy `W6.stairs20.walk.p2`
false-voids in S2 where S1 walked; S1 itself false-voids that row in ideal and mount1.5, B161);
**P7** under the noisy IMU on 30 mm rubble (belly 16.1 / 15.2 mm against 25; S1 ≥ 31.3): raising
only sinks the body.

**Found on the way.**
- The probe adds tilt on a stone: S1's four floor feet reach down and hold the body up (height
  error +9.6 mm on the 20 mm stone, S0 −1.1), 3.88° against S0's 3.27.
- MuJoCo creeps a planted stance downhill: S1 standing 24.6 / 39.4 / 49.1 mm in 10 s at 5 / 8 /
  10°; a leveled stance creeps less (17.6 at 5°).
- `test_awareness.py::test_an_object_out_of_view_is_unobservable_and_never_missing` failed three
  times under bench load (looks 2 against 1), with the leveler on and off, and passed alone each
  time. Profiling the playground step found an off leveler costing 0.27 ms a step
  (`WaveGait.level_xy` on every call), now skipped while the leveler is idle (`BodyLeveler.idle`:
  off and at zero; a quick bench re-run matched all 396 shared digests), and
  `rocky_model._p()` deep-copying the whole params file on every `servo_speed()` call (0.22 ms a
  playground step, on main too), now a cache read. A walking step: 0.98 ms with no leveler before
  the fixes and 1.26 with an off one; after them 0.71 and 0.75 (1.04 with it on); `sim/tests` 9.7
  min → 4.8; the flake did not recur.

**The loop:** gait 125 passed; `sim/tests` fast 840 passed, 8 skipped, 2 xfailed, slow 6 + 3
xfailed, and the same counts with `ROCKY_LEVEL=1` (P9); driver + harness fast 204; ruff clean;
`docs/TOOLS.md` current; fingerprint `965f4f70e5d1`, params D064. Nothing on main opts in, so no
published record moves while `level.enabled` is false.

**If it were flipped:** the playground's and the cockpit's straight walks on flat ground do not
change with an ideal IMU (bit-identical; a turn, a stop or a shove does, the review found), but every record that stands or walks on a slope, a
stone, rubble, a ramp or stairs moves (the terrain bench's S2 column), the envelope falls to 32.48
mm/s on a 5° plane, and a sim → robot start waits for `level off`. Only the Playground builds a leveler (and so the cockpit and the
vision, brain and place benches, which start one; their rooms are flat); the harness sim, the RL
envs, `shove_envelope`, `eval_recover`, `audit_righter` and the experiments build their own
supervisors and stay as they are.

**Next:** B162: re-bench with `level.filter_s` 1.0 (a scratch probe read 0.00 mm on the noisy flat
walks and kept P2 / P3), a window that keeps the belly clear on rough ground, and the owner's word
on P4 for creep and derate; B161 (the stairs false void); before any hardware run, B159 (IMU
calibration) and B157 (the probe, gate and void guard into `gait/`); the classical rival B156 and
the terrain residual B160.


**The review round (wip 4).** Two reviewers (controls, sim → real) verified 21 findings against
the tree; this round fixed them or says why not, re-ran the bench and the lip grid on the fixed
code, and the docs above now quote that record.
- **Fixed, the law** (`gait/pebble_level.py`): the tick is a schedule (a call within half the
  caller's smoothed period of the slot ticks it): on a 50 Hz loop with 0.3 ms of timestamp jitter
  350 of 350 calls tick (the `t − t_last ≥ tick` rule ticked about 62 %), at 500 Hz the same calls
  as before. The copies move on every call by rate × the call's dt (the plane still on the ticks),
  so the 500 Hz stream has no 50 Hz staircase. A swing foot is airborne only when it clears the
  15 mm band + 2 mm (`BAND_MARGIN_MM`: `pebble_feasibility.support_plane` fits a near-band foot
  into its plane, ~1.5 mm low) over the highest copy's or the plane's ground under its REAL xy
  (`WaveGait.level_xy(foot=True)`). An enabled leveler primes the derated ceilings for every
  0.0025 slope step its window admits (`BodyLeveler.prime()` → `WaveGait.prime_level`, 37 steps
  to 0.0925, 2.8 s once per process; a `budget(level_slope=)` is then a 0.075 ms lookup), since
  computed in the control step a new step cost 30–77 ms.
- **Fixed, the sim:** `apply_gait` reset the leveler on any `h` / `R0` key, the same value too
  (`gait default` on a levelled 5° slope dropped the feet 30 mm in one step, 4.7 rad/s at the
  guard's clamp); it keeps the plane now and primes the new gait's derate. The envelope readouts
  are the derated one (`Playground.envelope()`, the cockpit's gait and model panels, a `check`
  line saying it judges the bare gait); `V_MAX` (the teleop cap on the ask) and the capability
  registry's envelope stay bare. `TIPPED_DEG` lives once in `sim/playground.py`; the bench takes
  `PROBE_MAX` from there; `level.raise_mm`'s cap is `PROBE_MAX`; `level_xy`'s blend defaults to
  params.
- **Fixed, the bench:** W1 gains a turn in place, a walk + turn with its stop and two rim shoves
  (all under P1); W8 the 5° slope cliff walked across and uphill; W3 reports the stone foot's load;
  a false void on a drop world is now ground within `PROBE_MAX` of the foot's un-probed target
  (it was 30 mm under the lowered foot), with `void_ground_nom_mm` and `void_nohit` recorded; the
  provenance carries `code_diff_sha256` on a dirty tree, and the lip grid's record `head` / `dirty`.
- **Tests that kill the mutations the review survived:** the floor-feel holds (each named, and a
  hold freezing the plane and the copies through `step()`), the cliff at a15 / a20 walk 45 with
  the leveler on (removing the holds tips both: 20.4 / 18.8°), `reset_guards`, the derate wiring,
  the gait change, the filter's 63 % at `filter_s`, the jittered tick, the per-call copies, the
  airborne band, the priming, the meter's false void, the mount error on the gyro.
- **Measured envelope, the moving plane:** the review's converging-plane probe (24 runs, 5 / 8 /
  10°): judged at the Pi's 50 Hz loaded ≤ 2.918, free ≤ 2.867 rad/s; the sim's stream at 100 Hz
  (check()'s rate) loaded ≤ 2.944 (was 3.395), free ≤ 2.910 (3.614); at 500 Hz raw loaded ≤ 3.003
  (7.108), free ≤ 3.004 (7.481), the 3.003 one 2 ms step of a swing copy at 40 mm/s that check()
  classes loaded. The moving vs held plane through the supervisor (20 ms windows, 12 runs): loaded
  ≤ 2.953 moving (was 3.030), 2.930 held. The 50 Hz jitter probe: offsets step ≤ 0.84 mm per call
  (1.19 on the 8° / 234° run; was up to 2.20).
- **Not done, and why:** no leak of the plane inside the deadband (it would walk a levelled
  slope's residual out to the deadband's edge; the turning stop's residue is the probe's, B163);
  `_swing_speeds_level` still repeats `_swing_speeds`' body (merging them risks the bare ceiling,
  34.2041015625, pinned bit for bit); P2 still judges tilt alone (the stone foot carries 0 N in S1
  and S2, so it is reported, not gated); P4 is the decided rule, now stated with its 94.97 %
  derate floor (B162 c).

**The record after the fixes** (`sim/out/terrain_bench.json`: 1033 runs, 775 made, in 643 s at
`--jobs 6`, on `b2bf1b9` plus this commit's code, `code_diff_sha256` `28db9fa2…`): every S0 and S1
run is bit-identical to the first run's (468 of 468); S2 moved in 177 of 234 shared rows, by
≤ 0.003° of tilt RMS on the slopes, ramps, stones and the shove, by −0.74…+0.47° on rubble and
stairs. The numbers are in SIM_GUIDE §3 and D065. The lip grid on the same code: level off
364 of 364 approaches identical to the first run (259 s); level on 356 stop, 8 tip, 0 fall, worst
safe tilt after a fire 5.69°, 314 identical to level off, every void within 0.02 s (334 s).

**Verdict, again: fail** — P1 (noisy drift, and now the ideal flat-motion rows), P4 (27 pairs),
P6 (two noisy false voids, both the probe's, B161), P7 (rubble 23.5 / 24.2 mm), R1 (+0.42° on the
noisy rubble row that false-voids); P2, P3, P5, P8 pass. `level.enabled` stays false.

**The loop after the fixes:** gait 130 passed; driver + harness fast 204; `sim/tests` fast 849
passed, 8 skipped, 2 xfailed, slow 6 passed + 3 xfailed (122 s); with `ROCKY_LEVEL=1` the same
849 / 8 / 2, but of its 859 tests only 125 ran an enabled leveler and 17 ever raised a foot, 10
of them D065 tests that pin `level=` themselves (the reviewer's spy plugin: P9 covers less than it
sounds, and the gait suite does not read the variable); ruff clean; `docs/TOOLS.md` current;
fingerprint `965f4f70e5d1`, params D064.
---

## 2026-10-07 → 10-08 · Session 9q — the body layout is built from the 15 picks (D064)

**Ask:** the owner sent the 15 body-layout picks on 2026-10-07: 1 option A; 2 the 8 mm layer
(tub roof z −18); 3 the loop key in the tub's nose wall facing +x; 4 the XT60 carrier printed into
the nose wall; 5 `pack_l` 138; 6 the 380 g battery kept; 7 tray lugs, slide 16 then lift; 8 (b) no
strap, `bay_w` 50, a 2 mm foam pad; 9 the 12 V node a 30.5 × 30.5 PDB; 10 a 10 mm nose chamfer;
11 the I4 / I5 D020 sign-off; 12 the door held by one I3 latch in a south-wall boss
(x −91.5 … −85.5) and a north snap hook; 13 the belly in the sim and the righter's hip clamp
−51.05 in FALLEN (and at the RIGHTED ramp's start) on legs 1–4; 14 the adapter, buck and UBEC
face-down on the hub shelf, the Pi on 11 mm with its USB end +x, the adapter on the Pi's UART;
15 the seats fix for I1. Build them. Five workflows ran in order: the I1 fix, verified; params,
keep-outs and the deck's hole table; six parts in parallel (bay, deck, tray, shelf, stand, sim);
integration, a re-run of every published number, two reviews and a fix round; these docs.

**Two picks came out short.** Pick 4 saved 2.0 mm, not the ~10 the proposal estimated: modelled,
the mated XT60 pair is about 25 mm long and the printed carrier only 4 mm shorter than
`dock_block`, so `bay_l` 194 → 192 and the nose x 101.9 → 99.9 (about 184 would need the female to
stand out into the sled, B145). So pick 10's chamfer does not bring the nose inside the carapace
skirt: its south corner goes r 110.57 → 106.67, still 7.76 outside.

**The leg port (decision 15, B117):** the seats fix is in `part_coxa`, `iface` and `part_deck`:
cone posts at (−32, ±17.3), a cone and a V socket, slot 6.2 at x −50.1 … −43.9, the spine rib, the
0.2 relief from x −46; the dowels are gone. A new CI module, `cad/check_dock.py`, proves it on the
real solids: docked 0.0000 mm³ with the pads bearing and the cones at zero play; four stored hand
paths walked in 145 sections at 0.305–0.469 mm and replayed in 3D at 0 mm³ (0.30 at the lip ends);
the hook sweeps z −13.30 on the paths and −13.76 at any free pose; a 4.2 slot fails it. The
verifier caught two faults: the first hook envelope came from a 12-point outline with a phantom
corner (r 56.94–66.79 → 56.88–66.61), and the fit ladder's new seat row could not be tried (the
deck coupon's second post lands on the ladder), so the seat fit moved onto two more plate coupons
(`fit10`, `fit20`; `print.seat_fit` 0.0). The plate coupons are cut to the real plate,
x −46 … −11, and all four dock (B137).

**The foundation:** `params_rev` D064 with the I4, I5, hub-shelf and stand keys and
`reflex.righter_hip_min_deg`; `iface` keep-out helpers (station frames, the hook envelope as a box
round `check_dock`'s record, the drop keep-outs, `bay_tub_extent()`); the deck's explicit 21-hole
table. Three departures from the brief, each measured: the shelf outline is the pick-14 plate
(x −56.2 … 53.95, y 12.2 … 64.3, not the boards-up box), the node and star take the measured
bay_w-50 poses, and the shelf posts are screwed from below into blind holes, because an M3 head on
the deck top at (±50, 34) sits under the docked coxa plate's new pads (5.41 mm³). The verifier
added `bay_door_x` −97.5: params had no x for the tub.

**The parts (a82f8ad):**
- `part_bay` (new, PETG): `bay_tub` (the sled rails printed into its floor, the nose chamfered,
  the carrier on four 14 × 1.2 × 1.2 flexure webs, ±0.8 float, 23.4 MPa at its ±0.85 stop; the
  loop key's pocket over it; Ø6 driver columns for the three under-deck turn points; the stand's
  x-stop hole), `bay_lid` (six hangers on deck holes, two pilaster inserts), `bay_door` (a
  spigot, a 0.50 press on the sled's tail lip, the latch in a 6 mm ear, a snap hook tab).
  Interior 192 × 50 × 33, stack 2.10 + 3.0 + 25.0 + 2.0 = 32.10. The tub slides south under the
  lid's north return and takes two M3 × 10 from below: no north screw fits. 107.9 g (the
  proposal guessed 56 for tub + door).
- deck v0.5: the 21 holes; the shelf posts blind from below; the (0, 50) Ø10 grommet becomes a
  12.5 × 7.5 slot (B122); the strap slots, the (52, −6) grommet, two anchors and grid (0, −40)
  are solid again.
- `part_avionics`: the tray at (0, 0) turned 180°, tabs inboard, lugs under windowed lips, the
  Pi 5 envelope from the official STEP (`cad/ref/rpi5_envelope.json`, MIT; 6.82 mm to the
  carapace, B127), the boards off the tray. Fixing a compound-boolean bug in its own checks
  exposed a real clash: slid 16, the plate's tongue-end corners met coxa bases 2 and 3; they are
  cut square (0.87 mm).
- `part_busboard` becomes `hub_shelf`: three posts, the PDB and the star face-up, the three
  boards face-down, the 14 AWG route out of the nose's NE chamfer (0.20 to leg 4's drop + 5
  floor), the five looms. Its first version posed the carapace sectors 18° off their legs.
- `part_stand`: a U-cradle that holds the robot by its tub, carapace on (B85). The old crown had
  no seat and sat 3 mm low: its 47 / 127 / 207 were 44 / 124 / 204; now 89.4 / 169.4 / 249.4.
- The sim (B97, B130, B138): the belly boxes, the torso `<inertial>` from a body-frame pose table
  in `mass_audit`, the clamp in `ReflexSupervisor`, RecoverEnv's hip remap, the feasibility
  checker on the budget (a leg in the belly is a `SELF_CONTACT`).
- Retired: `sled_rail`, `dock_block`, `belly_door`, `belly_skid`, `busboard_bracket`.

**Three findings between the parts:**
- **The tongue boss.** The deck agent found the tray's tongue standing 3.4–5.4 mm over the
  (0, −43) strike, where the I3 cartridge fails its cam, land and stop rules. Fix: a glued
  `tray_latch_boss`, 5.1 tall (`TRAY_Z`, wall 1.6), puts the latch's seat on the deck top.
- **The slotted strike.** The tray agent found the rotor still 2.0 mm into a round strike at
  OPEN, pinning the slide (2.20 / 12.15 / 36.18 mm³ at 0.5 / 1 / 2). Fix (cec4b15):
  `iface.latch_strike(frame, slide)` draws the strike out into a slot along the slide; the deck's
  is frame 225, slide 16.5, and its end stops the tray there (bite 2.63 mm³ at LOCKED; the five
  sector strikes byte-identical by their BREP hashes). Slid north the plate would cover the trunk
  slot, so the release is south only.
- **The shelf hull.** The sim agent's shelf was one box from the shelf's bottom, grown north to
  y 71 by a post pad; leg 0 touched it in 20/20 `recover1` falls, where the real plate ends at
  y 64.3. Integration modelled the plate box and three Ø14 post columns, written at 1 µm after
  `test_belly` caught five decimals rounding the plate's −1.125 mm centre: leg 0 no longer
  touches in the 40-seed eval.

The wiring round (80cebbb) closed the remaining requests between the bay, the shelf and the
stand: the 14 AWG exit and the riser clip (B133), tie bars on the lid for the leg 2 / 3 looms, the
x-stop hole at Ø6.3 (the pin has 0.45 of play, not 0.30), the real tub, shelf and crown in each
other's checks, the door latch's driver on the stand (a ≥ 183 mm shaft, B144), and the sled out
on the stand with leg 1 at yaw ≤ +20.

**Integration (ef7c923):** CAD 30/30 (29 modules + `check_printability`, 61 parts, 0 hard), every
derived output rebuilt (print pack 49 pages, 10/10 drawings, the viewer, 13 assembly views).
`mass_audit` on real STLs: torso 1439.8 g at z 24.5 → 1555.2 g at (−2.21, −4.63, −3.17), robot
2727.7 → 2843.2 g, fingerprint `87215110e9c4` → `deae868522cc` (`c10b970d2bb2` was the
provisional budget). Sim tests 816 passed / 7 skipped / 5 xfailed; driver + gait + harness 305.

**The FEM finding (B139):** integration replaced `fem_check`'s whole-underside grip on
`coxa_yaw_base` with what bears in each case: the inboard pads, the seat flanks, the two
thumbscrew heads in tension. In V (the foot pushed up at knee stall, 60.3 N) the seats lift
+0.04 … +0.36 mm, the plate rocks on its pads against the screws, and the base **FAILS at SF 1.10**
(30.4 MPa p99.9 at the vee socket's flank, the cup deflecting 2.29 mm); 3.32 on the old grip,
1.59 with the seats pinned too. B119's rib (SF 2.33) was sized at the 3 × 3 design moment,
4629 N·mm; at stall it is 6994 N·mm at the screw line. The other four leg parts pass unchanged
(fork 2.47, femur 2.18, knee carrier 3.92, coupler 7.81). Not fixed here: it changes the I1 part
and `check_dock`'s record, so the owner picks after an option round.

**The reviews and the fix round (3ec6ab5),** a CAD and a code reviewer, then a recheck by a third
agent:
- every docked coxa base cut 42.80 mm³ into its carapace sector, on main too (`part_shell`
  checked proxies): `cup_back_relief` clears it at 0.30, and the shell's keep-outs now pose the
  real base and yaw servo (B124);
- `part_avionics`' overlap helper caught every exception and returned 0 mm³, so a failed boolean
  read as clear: it fails again;
- the tray cannot come out with the carapace on (sector 2 from slide 14; sectors 3, 4, the cap and
  sector 1 on the lift): a reported row and a rewritten print pack (B143);
- CI's filter for the design-pipeline job missed `gait/rocky_model.py`, `sim/mass_budget.json`
  and `gait/pebble_gait.py`;
- `mass_audit`'s pose table copied part geometry as literals: two derive from params now, and
  `cad/check_sim_mirror.py` (in `run_all_checks`' POST stage) checks the rest, the sim belly
  against `iface.bay_tub_extent()` and the STLs against the sim boxes (params
  `hub_shelf.pad_d` is the one pad size);
- RecoverEnv's drops started a leg inside the belly in 15 of 400 seeds: `drop_clear_of_belly`,
  default on and recorded in the contract (the evals replay old checkpoints on their old stream);
- a resume could change a checkpoint's action map silently: `action_map_mismatch` refuses it,
  with `--hip-clamp` and `--belly-drops` flags;
- minors: the clamp's leg set read from params only, `part_deck` failing a pending row,
  `part_bay` gating its flexure stress and snap strain, `.claude/worktrees` ignored.

The relief's 0.2 g moved the fingerprint again, `deae868522cc` → **`c3e82f13b671`** (torso
1555.0 g at (−2.21, −4.63, −3.18), robot 2843.0 g), and flipped one chaotic test (a fall during a
gesture: 60–65 N fall on the old budget, 60–64 on the new; re-pinned 65 → 62 N). The docs critic then found the rails' north tab holes
at (±40, 20) still Ø3.4 with nothing under them to thread into (B155): they are Ø2.8 thread-forming
again, an M3 × 10 into the deck whose tip clears the star board by 1.90 (`part_busboard` measures
it), and that +0.02 g of deck moved the fingerprint to **`965f4f70e5d1`** (torso and robot unchanged
at 0.1 g; `run_sim` and the torque audit identical).

**The numbers, re-run on `deae868522cc` (c0947bf), old → new,** with a third tree (the belly's
contacts off) to separate the belly from the mass:
- `run_sim` walk 189 → 189 mm, turn 41.8 → 41.7°, max tilt 0.41 → 0.42°; the envelope
  34.2 mm/s / 0.185 rad/s (kinematic, no mass input).
- Righting, 200 seeds: 197 → 198/200, no righter 87 → 104/200 (the lower CoM, as 9p predicted);
  20 seeds: system 20/20, no righter 11/20, `recover1` hw 0/20; the clamp clips 41 % of FALLEN
  ticks.
- Shove: standing 30 N, 1.12 → 1.08 BW; walking mean 1.12 → 1.14 BW. The robot is 4.2 % heavier
  and the newtons hold or rise.
- Knee self-right 1.397 → 1.456 N·m, 47.5 → 49.5 % of stall, WARM, half a point under HOT.
- Rubble, the mass alone (byte-identical with the belly off): bare 30 mm 2/4 → 3/4, 35 mm
  2/4 → 1/4; with the watchdog 40 mm 2/4 → 3/4; 0 falls.
- Cliff safe-stop PASS, margin 228.8 → 228.9 mm.
- **The lip band changed its failure, not its count.** The approaches that fell on D063 now tip
  20–24° onto `belly_tub` at the edge and hang there (torso 0.243–0.245 m; the void fires
  0.3–1.3 s after the tub lands): 0 of 364 fall by the D063 criterion (was 8) and 8 tip past
  10°; of 724, 0 fall and 16 tip (was 17 falls); after a fire 1 → 0. With the belly's contacts off the same
  16 fall again. `run_cliff`'s no-guard control hangs the same way, so its verdict reads
  INCONCLUSIVE (B140). Whether a real keel catches on a table edge is unmeasured.
- Gestures 19 rows, 0 FAIL, 0 self-contact samples; the feasibility CoM margin at the envelope
  41.4 → 38.5 mm.
- Leg 0 touches the shelf plate in 2/200 shove-demo falls (≤ 1.40 mm); a leg-0 floor of −62.5
  gives 0/200 with identical outcomes; the clamp stays on legs 1–4 as picked (B141).
- Every RL checkpoint is legacy; `recover7` hw 3/20 → 5/20, still 0 supervisor handoffs (B146).

**On hold:** printing `coxa_yaw_base` × 5 (B139; one for the bench is fine). Nothing else. The
port coupons, the deck patch and four plates (`relief46`, `relief42`, `fit10`, `fit20`), are the
first print, then fit-ladder row D (5.8 / 6.2 / 6.6); the ladder has no seat row. Service, as
measured: the sled comes out before carapace latches 162 / 306 and the tray's latch can be turned
(their driver columns run through the tub); the hub is serviced off the stand; the tray needs the
carapace off; the door's latch is turned from +x with a long driver. UART: H2 RXD → Pi pin 10,
TXD → pin 8 (crossed), GND from pin 9 or 14, jumper H4 to A, the adapter's logic powered only
from its 12 V input in UART mode; the one usable USB-A (lower middle) takes a right-angle plug
≤ 20 long with an overmold ≤ 14 × 7.

**Not done:** the coxa base fix; a retrain; the careful-walk lip grid, the 90-walk terrain A/B,
the full 214-scenario goto verify (the 46 + 46 quick ones match) and the vision, brain and place
benches (each starts its own cockpit); the tub's protrusions in the sim (B142); a slotted rotor
head for the door (B144). The re-run numbers are on `deae868522cc`, not repeated on
`c3e82f13b671` (0.2 g of shell) or `965f4f70e5d1` (0.02 g of deck).

**The loop:** CAD 31/31 (29 modules + `check_printability` + `check_sim_mirror`), 37/37 with
`--derived` (FEM 4/5: the base); sim 820 passed, 7 skipped, 5 xfailed; driver + gait + harness
305, harness slow 2, `cad/test_fem` 10; ruff clean; `docs/TOOLS.md` current. Commits d99cdc2
(the seats fix and the foundation), a82f8ad (the six parts), ef7c923 (integration), 3ec6ab5 (the
review fixes), c0947bf (the re-run records), and this commit (the docs: `docs/CODE_MAP.md` and
`docs/BUY_AND_PRINT_ORDER.md` new; the proposal, the 09-22 review, the perception plan and
sessions 8e–9k archived; ROBOT_AS_DATA and SCALE_UP_NOTES merged into DESIGN_CHANGE_GUIDE §9 / §10).

**Same day, the retrain (B146 → `recover8_d064`, RL_GUIDE §4).** The owner asked for the RL to
run. Cockpit started on the new model (unit `rocky-cockpit`), then a 20 k-step smoke run, then the
`recover7` recipe for 12 M steps as unit `rocky-train-recover8` (08:26 → 10:06, ~2.0 k steps/s
beside the cockpit; final return 456 vs `recover7`'s 330, episode length 236 of 300). The ladder:
raw policy, nominal servo **0/20** (it stands up tilted ~19° and never settles inside the hand-off
hold); raw with the randomised servo + DR + noise **12/20** (back 5/9, side 7/9); the system on the
nominal servo **20/20** (no-righter 12/20), 8 falls, all 8 exits on the stall ramp, **0 belly
contacts** (`recover1` 2/20); the system with the randomised servo **18/20** (no-righter 8/20), 12
falls, **3 hand-off exits**, 9 stall — the first hand-offs under the supervisor on the D052
contract (`recover1` on the same model and servo: 19/20, 0 hand-offs, t_stood 7.88 s vs 5.44).
Shove audit beside `recover5_v3_warm`: 31 % pinned vs 55 %, 2.5 rev/s vs 3.9, upright 5/5 by the
ramp either way. Shipped as `righter.default_ckpt()`'s first choice, `recover1` the fallback; the
bench's real servo latency decides which servo model was right (B34 stays open for the back
landings and the nominal-servo tilt).

**Next:** the coxa base option round (an outboard hold-down, a deeper rib past the sockets, a
thicker plate, screw preload as a design load; each with `check_dock` and the FEM, B139); print
the port coupons, then file `print.seat_fit` and measure an insert's pull-out and the knob preload
(B118); `run_cliff`'s tip criterion (B140); the righter's back landings and nominal-servo tilt (B34).

---

## 2026-10-01 → 10-05 · Session 9p — the picks are not in, so the keel tub, the boards and the leg port get measured first (B117–B138)

**Ask:** the 9o handoff prompt: build the body layout from the owner's picks if they were sent,
otherwise say what is still open. The owner desk page's `desk/layout` still held only the three
pre-filled recommendations (1 A, 2 8 mm, 12 south-hook) with no send time, so nothing was built.
The owner added two standing rules mid-session: agents run on Opus 5.5 or Sonnet 5.5 by task, as
many as needed; and (10-05) record the findings in the repo docs. Everything below is read-only
measurement in scratch copies of the tree; `cad/params.yaml` and every part file are untouched,
fingerprint `87215110e9c4`. The records are `docs/archive/prep-2026-10-01/` (two reports and the
I1 judgement; the ~1,000 scripts and logs stay outside the repo and are listed for promotion in
report 1 §6). Two rounds, 39 agents (19 + 20), every headline re-measured by a second agent; the
second round was cut off once by a usage limit and resumed from its cache.

**Why measure first:** reading `sim/build_mjcf.py` for the handoff showed the sim has no belly:
the torso's lowest face is z +6 (124 mm over the ground at stance) while the proposal's tub bottom
is z −55.4 (62.6). Every rubble, righting and shove number the proposal rests on was measured with
no keel. So, while the picks were pending, the tub went into scratch copies of the MJCF (a massless
box, a box with the proposal's torso mass and CoM, and the 5 mm layer), and decisions 13 and 14,
which the page left half-measured, got their numbers.

**The tub is safe (confirmed):** righting with `recover1` + the supervisor 197/200 with or without
the tub, at 8 or 5 mm (20/20 and 11/20 reproduce on seeds 0–19); `run_stuck` byte-identical at
30–45 mm (0 tub contacts in 192 trials, closest 14.3 mm; first touch at 62 mm rubble, the only
tub-caused stalls at 64–70 mm are the square nose corners jamming sideways); the gait at the
envelope over 24 headings, the turns, 300 slew ramps, the 12 gestures and the audit, the stance
and cliff probes: 0 poses in the tub's reach set, 0 contacts (closest 69.8 mm). Only the righter's
fall path reaches it: 6/200 drops massless (max 2.36 mm), **12/200 with the battery mass** (max
3.06), legs 1–2, a pinch (the hip at full torque drives the knee-saturated leg under the settling
keel, tub force to 46.9 N), with no outcome changed. The battery in the tub lowers the robot CoM
26 mm and lifts the no-righter baseline 87 → 104/200 (sim torso 1467 g at (−2.5, −2.0, −1.2)).
Settled ground clearance 62.15 (8 mm) / 65.15 (5 mm). Fingerprints with a belly: `777fdb4eda14`
(8 mm, massless), `47435187271f` (with mass), `4f77d216a700` (5 mm); geom ids shift +2.

**The I1 leg port cannot dock (B117, confirmed by three designers, three verifiers and a judge):**
the hook's L is 5.15 wide at its foot against the 4.2 slot (4.6 fails too): no rigid-body path at
any tilt from −30 to 90, even at zero clearance; the 8 mm dowel posts then block the pivot even at
slot 5.2–7.0 (none at 0.10 clearance). The port coupons and fit-ladder row D cannot rehearse it:
the plate coupon's patch reaches x −57 and penetrates the deck coupon on any path (B137). Three
fixes were designed from different angles and each verified to dock and undock at ≥ 0.30:
"minimal" (slot 5.2, short foot, 3 mm tapered posts; play ±0.9°), "seats" (two 30° cone posts at
(−32, ±17.3), cone + V sockets, slot 6.2 grown outboard, a spine rib; nominal zero play), and
"sequence" (flat slide, chevron hook, knob skirts; print risk high, plate SF 0.52). The judge
recommends **seats** with the pad relief ending at x −46 (seat share of the screw preload
8 → 30 %), docking at 7–10° with a radial approach, the leg's own carapace sector off (B125);
it changes three frozen values (`dowel_xy`, `hook_slot_x`, `hook_slot_w`) and so waits for the
owner (decision 15). **The stance load path in INTERFACES I1 is wrong** (B118): the foot lifts
the plate's outboard end, the plate pivots on its most inboard deck contact, the hook carries
nothing in stance (sim My +241.5 N·mm per leg), and the two thumbscrews with their inserts carry
972 N per pair in the design case (3 legs × 3, μ 0.5; 568 with seats). Today's plate is at
47.4 MPa at the screw line, SF 0.74 (B119; seats' rib: 15.0, SF 2.33). The orchestrator's own
hand statics in the round-2 brief had the same wrong sign; the agents corrected it.

**Decision 13, measured:** a uniform hip limit that clears the tub costs 11.8–13.7 % of the planar
reach, 17–39 mm on a 20 mm stair and 1–3 mm of BRACE, and at bay_w 53 no value both clears the
tub (−49.72 sim capsules / −47.91 proposal radii) and keeps the cliff void guard, whose ceiling
is −47.91 at h 118 (`probe_out` needs 26.5 mm of measured depth; void 12/12 at ≤ −47.75, 0/12 at
≥ −47.5 and the robot walks off the table; B129). A keep-out on the supervisor's targets that
raises the knee leaves 1–6/200 contacts; a hip clamp ≥ −51.05 on legs 1–4 gives 0/200 with the
outcome unchanged, 38–43 % of FALLEN ticks clipped, and applied only in FALLEN/RIGHTED it leaves
BRACE and the probes alone (B130). Every reach number assumes a 135 mm foot; with the hand as
`part_hand` builds it (B95) the legs pass 17.5–18.7 mm through the tub at −51.05 (B131). The
supervisor and playground do not clamp to `joints.pos_deg` at all (B132).

**Decision 14, measured:** with the real Pi 5 STEP the 3D gap to the carapace is 6.84 (USB end +x)
/ 3.56 (−x) at 11 mm standoffs and 5.45 / 2.17 at 13; `part_avionics` checks only to the board top
(B127). Pi on 18.5 mm fails the 3 mm rule; at 13 mm no USB-A port takes a plug reliably, at 11 mm
and +x one does (right-angle ≤ 20 long; B128). The measured home for the bus adapter, buck and UBEC
is face-down on option A's hub shelf with the Pi at 11 mm (all three at bay_w 50, 0 mm³, 0.80
margin; the adapter alone also at 53), three new posts at (−50, 34), (50, 34), (26, 64), the
adapter on the Pi's UART (H2, jumper A; RX/TX cross; GND off pin 9 or 14). Round 1's "pi13 +
USB +x" recommendation did not survive round 2. Caveats: 1.00 mm to the stand's crown plate
(ESTIMATE stand), hub service means the robot off the stand (B134), the 14 AWG feed has no route
at bay_w 53 (B133), the Ø10 grommet passes no plug (B122), the J6 lines have no bulkhead connector
(B123).

**Also found:** each docked coxa base meets its carapace sector by 42.801 mm³ (cup back wall at
r 71.5–72; `part_shell`'s proxy checks miss it; B124, confirmed by my own script); three under-deck
turn points sit inside the tub's plan, not one (latches 162 and 306, the tray strike; B126); the
mated XH-5 pair does not pass the 11 × 11 cutout (a 7 × 16 cross arm would; B120); leg-side XH-5
pins 1–2 carry two or three wires (B121); the 5 × 7 loom lanes clear the corrected hook sweep by
1.45–1.75 but a zip-tied loom needs 8 × 7 (B135); legs 2/3 reach their own drop keep-out (B136);
`pebble_feasibility` hard-codes the torso mass and CoM (B138). Option B as drawn is out (the Pi
enters the carapace by 492 mm³); it survives only with a side-entry hub straight on the deck and
the 3 mm rule read vertically. The 5 mm layer keeps 1.26 mm to the hook sweep with the seats fix
(8 mm: 4.26) and takes a bare loom only, so 8 mm stays.

**Corrections to 9m's proposal:** 24 numbered corrections in BODY_LAYOUT_PROPOSAL's
"Re-measured 2026-10-01/02" block (the hook sweep −13.8 → never reachable as drawn, −13.30 / −13.76
with seats; the carapace margin 5.5 / 12.25 → 3.56 / 6.84 as a real 3D gap; the clear band
−21.9 … −17.2; the census 114 / 97 / 97 / 114; the grommet; the 14 AWG run; the shelf hangers; B's
ceiling). Round 1's own claims that round 2 refuted: "USB end +x is mandatory" (−x passes at 11 mm
with the real parts), "the servo lags between clear targets" (a pinch), "no target keep-out can be
contact-free" (a hip-first projection is, in one agent's runs), "no 5 × 4 packing exists" (marginal).

**Not done:** nothing in `cad/`, `sim/` or `gait/` changed; no number in README, SIM_GUIDE or
RL_GUIDE moved (they stay true for a robot without a belly). The owner's picks, decision 15's
sign-off and the stance hold-down coupon gate the build. Open from 9o unchanged: B101, B102,
B107's head cap, B105's partial arcs, `request_stop`'s docstring, `ArmedGait`'s MARGIN (B113),
B33 c, B34's next rungs, B54 at 45 mm. One round-1 agent used `pkill -f` against the brief; the
cockpit was checked alive afterwards.

**The loop:** docs only, `ruff check .` clean, fingerprint unchanged; CI runs the full suites on
the push.

**Next:** the owner sends the picks (the page now carries decisions 14 and 15 with the measured
options); then the build: the seats fix first (port coupon pair, seat-fit row, insert pull-out
≥ 300 N, knob preload ≥ 285 N), `iface` keep-out helpers from `keepouts.py`, `part_bay`, the belly
in `build_mjcf` from params with the torso mass and CoM, the righter's FALLEN/RIGHTED clamp, and
every published number re-run on the new fingerprint.

---

## 2026-09-30 · Session 9o — goto gets its detour cap, the torque audit stops counting the hands twice, one CAD pass (B116, B106, B105, B107, B108, B110)

**Ask:** the owner on 9n's two options: "yes, apply the 87 s goto cap and anything else".
B103 (T 2.2) is not applied: its recommendation is to keep 2.0. Three tracks: B116 applied;
the torque audit (B106); the CAD follow-ups that need no owner decision and no hardware
(B105, B107, B108, B110). Each was reviewed by a second agent that re-measured its claims,
then one pass over the whole tree. `cad/params.yaml` untouched: fingerprint `87215110e9c4`.

**B116, goto's detour cap (applied):** once a goto has entered a detour (a sticky flag per
goto) its cap is `goto_detour_cap_s`, `goto_cap_s` with the reach grown by both detours:
(1.5 + 2 × 0.45 m) × 1.2 / 0.0342 m/s + 2.36 s = 86.6 → **87 s**. A goto that never detoured
keeps 55 s. The cap now ends a goto even mid-detour, so a goto answers by 88.5 s; before, a
detour running at the 55 s cap ran out first, up to 87.5 s. The envelope carries it
(`goto_detour_timeout_s`), and goto's text says "55 s cap — a detour may extend it to ~87 s
—" (`docs/TOOLS.md` regenerated; the golden hashes moved on purpose). `go_back_to`'s legs
are ordinary gotos and get it too, so the MCP proxy's `GO_BACK_TIMEOUT_S` is derived: 4 ×
(87 + 1.5) + 12 = 366 s (was 300); its 120 s per call keeps 31.5 s over a goto's answer. The
in-process sim's goto has no detours and keeps 55 s. **Verified** on the shipped rule
(`run_goto_detour_cap.py --verify`, 214 scenarios, 40 min at 5 jobs): 214/214 ended as
derived from the uncapped runs (within 0.05 s). 195 arrived: around an obstacle 16 / 78 / 80
/ 15 of 16 / 84 / 84 / 16 at 1.0 / 1.2 / 1.5 / 2.0 m (at the old 55 s: 15 / 38 / 0 / 0). 13
ended `blocked` and 4 `stuck` by themselves, and 2 timed out (the clear 2.0 m gotos, at 55.0
s, 23 cm short). No detour goto reached 87 s (the latest answer 87.2 s), so the cap's own
timeout is shown by two new tests and the review: a 2.9 m goto past a 0.25 m wall times out
at 87.00 s, 58 cm short; with a second wall at x = 2.22 it ends at 87.00 s mid-detour and
answers at 88.50 s. `go_back_to` to a ball 2.0 m out behind a wall / box: 72.6 / 72.1 s, the
first leg arriving on the detour cap at 56.5 / 56.1 s (it was a 55 s timeout and a re-aim,
72.5 / 71.9 s); clear, 53.2 s, unchanged. The gait phase at the call moves that first leg by
~3.6 s (three runs behind the wall: 56.5 / 56.6 / 53.0 s).

**B106, the torque audit counted the hands twice:** the tibia budget already holds the hand,
so the audit weighed 2920.7 g against the MJCF's 2727.7 g and hung tibia + hand at mid-shin.
Now it weighs the MJCF's robot, with the hand at the tip: every stance case × 0.934 (3-leg
knee 0.466 → 0.435 N·m), carry hip 0.516 → 0.480, untucked hip 0.290 → 0.254 (the compiled
MJCF's subtree moment: 0.2541), and the self-right knee 1.496 → **1.397 N·m = 47.5 %** of
ST3215 stall, **HOT → WARM** (28.5 % of STS3250). Carried into SERVO_NOTES,
DESIGN_CHANGE_GUIDE §1, the bench runbook (§7 soak loads; §8 masses at 100 mm 330 / 445 /
490 / 1425 g), `thermal_soak.py` and D044's rerun trail. `sim/tests/test_torque_audit.py` (3
tests) pins the robot and the airborne leg's moments to the compiled MJCF; all 3 fail on the
old audit.

**B110, one CAD pass:** with `--fem`, `run_all_checks.py` runs the stress check before every
derived output and the drawings before the pack, so one `./rocky.sh cad-check --derived
--fem` leaves the pack current; `--jobs N` caps the parallel modules. The passes found a
determinism bug: multi-threaded SPOOLES in the Flatpak's CalculiX now and then returns a
wrong load step (pass 1 wrote the femur's R+ as SF 10.42 for 11.90; 2 of 6 four-thread
solves were corrupt, 6 of 6 one-thread solves identical). `fem_tools.run_ccx` solves on one
thread: `fem_check` 53–64 → 66–72 s, every governing SF unchanged (2.93 / 2.47 / 2.18 / 3.92
/ 7.81), the FEM files byte-identical to HEAD.

**B105, open channels on the drawings:** `td_sheet.features()` tells a slot end (a half
circle with a partner facing it) from an open channel. The fork's sheet lists Ø6.4 × 1.3, Ø7
× 0.6 and Ø20.3 × 1.8 in an OPEN CHANNELS table (also in `drawings.json`, the drawings index
and the pack); all 32 slot ends found their partners; the other nine sheets list none. Arcs
that are neither a full nor a half circle (7, in the row) are still not called out.

**B107, the latch's keyway index:** the housing is a D (a flat 6.0 mm from its axis) in a
matching D pocket on the demo, the sectors, the belly door and the tray tongue, so it goes
in one way, keyways 135° from the entry line. On its index 0.00 mm³ with ±2.5° of play;
turned ±10° or a half turn ≥ 1.4 mm³; the rotor is captive from −8° to 98° and drops through
only at 132–138°. Every D063 latch number unchanged. Not a stop: off the robot a rotor
turned 45° back past OPEN still drops out; only a cap glued over the head would hold it (not
drawn).

**B108, the tray's other boards (PARTLY SHIPPED):** the bus adapter, the buck and the UBEC
are envelopes in `part_avionics` (datasheets; the adapter from Waveshare's STEP), checked
where the docs put them; the check reports and does not fail the run. With the Pi on its 11
mm standoffs none has a place: the adapter's jack stands 6.1 mm into the Pi's keep-out (and
its pad's two M3 holes match none of the adapter's), the buck with its tie 1.0 mm, the UBEC
with its tie 0.8 mm under it (the rule is 1.0). Measured options: 13 mm standoffs fit the
buck and the UBEC; 18.5 mm with the IMU moved to (18, 1) fits all four. Each eats the body
layout's carapace margin over the Pi mm for mm (5.5 / 12.25 → at most 3.5 / 10.25, or −2.0 /
4.75). The owner's call, with B51.

**Found in the final pass:** B107's D fills add 0.10 g of torso print (the five sectors
0.075, the door 0.018, the tray 0.009), which CI's design-pipeline step (mass_audit on the
rebuilt STLs, then `git diff --exit-code` on `mass_budget.json` and the URDF) would have
failed on. Regenerated: torso 1439.7 → 1439.8 g in `mass_budget.json`, 1.4397 → 1.4398 kg in
the URDF, parity OK; the MJCF is byte-identical (it splits the torso into 1.0798 + 0.3599
kg), so the fingerprint stays. `torque_audit.json` re-run: robot 2727.8 g, no torque moved.

**Corrections to 9m and 9n:** 9m's "knee self-right 1.496 N·m = 50.9 % (HOT)" counted the
hands twice: 1.397 N·m, 47.5 %, WARM (B106; the D063 row now says so). 9n's open item,
`GO_BACK_TIMEOUT_S` under 4 × the worst goto answer, is closed by B116 (366 ≥ 4 × 88.5 +
12).

**The loop:** fingerprint `87215110e9c4`; MJCF and URDF regenerate byte-identical (after the
mass regen above), `docs/TOOLS.md` is current, ruff clean, `bash -n rocky.sh` OK. Fast
ladder 1101 passed (sim 800, harness 116, gait 97, driver 88), 2 strict xfails, 7 skipped,
9.1 min. CI's full `sim/tests` 806 passed (6 of them slow), 5 xfailed, 7 skipped, 10.6 min;
harness slow 2 passed; after the mass regen every suite was run again, same counts.
`cad-check --derived` 34/34 twice (283 / 270 s): neither pass changed any of the 270 files
under `cad/`. `cad/test_fem.py` 10, `run_sim` 189 mm / 41.8°, URDF parity OK. New tests:
`test_torque_audit.py` (3), two B116 physics tests through `/api/tool/goto` (slow, ~2 min in
CI), the detour cap's derivation and copies; the latch index and the board envelopes are CAD
checks.

**Next:** the owner decides B103 (recommended: keep T 2.0), B108 (where the boards go, with
B51) and the body layout's §8. Still open: B107's head cap and the door / tongue strikes
(B51, B84); B105's partial arcs; WIRING_HARNESS's "UBEC next to the buck" (B108, noted). The
proxy's waits are wall-clock while goto's caps are sim time: 120 s per call covers a cockpit
at ≥ 0.74× real time, `go_back_to`'s 366 s ≥ 0.97× in its 4-leg worst case (slow motion or a
pause is not covered, as before). The in-process sim's goto text quotes the 87 s it never
uses. `request_stop`'s docstring; `ArmedGait`'s MARGIN (B113); the lip band (B33 c); B34's
next rungs; B54 at 45 mm.

---

## 2026-09-30 · Session 9n — the D063 follow-ups, a curriculum retrain (negative), two options measured (B111–B116, B34, B103)

**Ask:** the owner could not find the mic, and sentences typed on the phone never reached
the robot. Fixed first (`da74878`): the chat row and 🎤 follow the operator to the Drive
tab, and a console line the parser does not know goes to the brain as chat. Then "what
else can we do right now", with nothing from the owner's list done yet: the D063
follow-ups that need no owner decision and no hardware (B111–B115), the B34 retrain, and
two measured options for the owner (B103, B116). Five tracks, each reviewed by a second
agent that re-measured its claims, then one pass over the whole tree. `cad/params.yaml`
untouched: fingerprint `87215110e9c4`.

**B111, the experiments' time limits:** `run_stuck.T_WALK` is the 1125 mm commanded
distance at the envelope ask (34.2 mm/s for 32.9 s; `run_fairing` and `run_stuck_voiced`
follow). Rubble 30 / 35 / 40 / 45 mm, bare → watchdog: 2/4 → 4/4, 2/4 → 4/4, 1/4 → 2/4,
1/4 → 1/4, 0 falls, tilt ≤ 12.6°; 10 of the 17 crossings came after the old 25 s. The
shin fairing crosses 1/4, 0/4, 0/4, 0/4 (B1 stays NEGATIVE); at 45 mm the watchdog no
longer helps (B54). The lidar lap asks `scenes.V_X` through the budget on its 0.55 m ring
(26.6 mm/s + 0.048 rad/s for 66.1 s, 528 scans, was 310); `sim/lidar_scans.npz` is
re-recorded, and SLAM-lite on it gives ATE 59.2 → 9.2 mm, map IoU 0.187 → 0.692 (the
2026-07-31 lap at a raw 45 mm/s: 55.6 → 8.2 mm, 0.255 → 0.716).

**B112, the cliff safe-stop verdict:** since D063 a zero command stands still, so the old
test (fewer contact breaks than the bare halt) could only read 0 against 0. `judge()` now
checks the stop itself: it fires, no fall, five feet down short of the edge, BRACE →
NORMAL, no foot lifts after the halt, < 1 mm/s over the last second. The void retreat
reaches the gait unslewed, as in the harness goto. **PASS**: margin 228.8 mm, the leading
foot 46.2 mm short of the edge, max tilt 0.48°, BRACE → NORMAL in 0.65 s. The bare halt
passes the same checks (213.5 mm); `--retreat-cycles 0` FAILS (a foot 11.5 mm over, 7
breaks), so the verdict can fail.

**B113, `ArmedGait`'s envelope:** its 44.6 mm/s was `WaveGait`'s lift ceiling at the
un-leaned foothold. At the leaned footholds 44.6 fails SPEED_LOADED (3.78 rad/s),
SPEED_FREE (4.11) and LIMIT_YAW (42.7°). `vf_limit` now judges every leg at its leaned
foothold with the stride in any direction: coxa 33.2 / tangential 41.9 / lift 37.3 →
**33.2 mm/s, 0.174 rad/s**; 98 commands pass every speed, limit and KINK check (peak 3.08
rad/s). `WaveGait` is unchanged (34.2 / 0.185). Found, not fixed: the checker's MARGIN
fails `ArmedGait` while a leg beside the arm swings (−67.6 mm walking, 44 % of the cycle),
and `WaveGait`'s coxa closed form covers a tangential stride only (it does not bind).

**B114, the in-process goto's cap:** `sim_backend.goto_cap_for(gait)` derives it the
cockpit's way: 55 s on the params gait (was a fixed 20 s), 57 s on `ArmedGait`. It counts
walking time, and a timeout ends in a safe-stop. A 1.5 m goto on the flat floor now
arrives (x 1.473 m); with a 20 s cap it stops at 0.622 m.

**B115:** `fem_check` prints SF and stresses to 2 dp and deflection to 3. 5/5 parts pass
with the same SF (2.93 / 2.47 R− / 2.18 / 3.92 / 7.81); `fem_results.json` and the
pictures are byte-identical.

**B34, the side → back curriculum: NEGATIVE.** `CommandSlew` is now in the gait RL env
(`cmd_slew`, on for new runs: a start moves a joint 3.38° a tick, 20.04° unslewed; older
walkers are flagged `unslewed cmd`). `--curriculum side-back` redraws a landing past 110°
with a probability that ramps 0 → 1 over 15–75 % of the run (0 → 46 % of landings, 120
seeds). `recover7_d063_curriculum`, 12 M steps on CPU in a transient unit (10:46 → 12:28,
1.96 k steps/s): hw **3/20** (side 3/7, back 0/6, tumble 0/7), the first handoffs on the
D052 contract; `--randomize` 4/20; **`--supervisor` 11/20, the same as no righter, with 0
handoff exits** (`recover1`: 20/20). Its ctrl rate is 1.9 rev/s against
`recover5_v3_warm`'s 4.5, but in the shove audit it lands on its back and never stands
(0/5). All 15 log σ sat at the −0.5 cap from 6 M steps on. It misses the bar on the handoff
exit, and the back, the curriculum's target, is still 0/6; `recover1` stays shipped. Next:
v3 at 10 M, `thermal_heat0` warm starts, and why the side handoffs never become a
supervisor handoff (not diagnosed).

**For the owner, measured, not applied:**
- **B103, T 2.2:** +16 % speed (34.2 → 39.8 mm/s), and every kinematic check passes, but no
  safety number improves. Cliff falls after a fired void guard go 1 → 5 of 724 approaches
  (all falls 17 → 22), the lowest walking shove 25 → 20 N, gait tracking p95 to 17.5°
  (fail 20). Recommended: keep T 2.0.
- **B116, goto's cap after a detour** (`sim/experiments/run_goto_detour_cap.py`, 342 runs):
  a detour costs 11.0–32.0 s up to 1.5 m, so at 55 s none of 84 detour gotos of 1.5 m
  arrives. Recommended: 87 s once a goto has detoured (the reach grown by both detours),
  ended at the cap even mid-detour so the answer stays ≤ 88.5 s. At 87 s, 80 of the 84
  arrive; the other 4 end `blocked` or `stuck` by themselves.

**Corrections to 9m:** "none after a fire" at the lip band holds only on its whole-degree
grid. Offset by 0.5°, 15 mm/s at 15.5° fires the void guard 0.62 s into the hold and still
tips, to 22° (B33 c). The walking shove's "30 N in all six directions" is one gait phase;
over 5 phases the minimum is 25 N = 0.93 BW (B104).

**The loop:** fingerprint `87215110e9c4`; MJCF and URDF regenerate byte-identical and
`docs/TOOLS.md` is current. ruff clean. Fast ladder 1097 passed (sim 797, harness 115,
gait 97, driver 88), 2 strict xfails, 7 skipped, 9.3 min. CI's full `sim/tests` 801 passed
(4 of them slow), 5 xfailed, 7 skipped; harness slow 2 passed. `cad-check` 28/28,
`cad/test_fem.py` 10, `run_sim` 189 mm / 41.8°, URDF parity OK. New tests:
`sim/tests/test_experiment_limits.py` (13), the `ArmedGait` envelope, the sim goto cap
(fast + slow), the FEM row format, the slew and the curriculum in the RL envs.

**Next:** the owner decides B103 and B116 (both measured, neither applied) and the body
layout's §8. Still open: `GO_BACK_TIMEOUT_S` (300 s for 4 legs) is under 4 × today's
worst goto answer of 87.5 s (B116); `request_stop`'s docstring still says a zero command
marches in place; `ArmedGait`'s MARGIN (B113); the lip band (B33 c); B34's next rungs;
B54 at 45 mm.

---

## 2026-09-28 · Session 9m — the fork slides on, the panels close, the motion stops snapping (D063)

**Ask:** "fix the fork ... do everything you can on yours": the CAD faults of 9l that one
part owner can fix, and the software half of the servo notes (B76, B79). Six groups (leg,
panels, small parts, gait, driver, body layout) each implemented their rows; a second
agent then reviewed each group, re-measured every claim and fixed what it found. Then the
slew was wired in, the gestures re-timed and every derived output regenerated.

**The leg (B80, B81):** the idler pocket (Ø20.3), its Ø7 head relief and a 6.4 mm channel
for the horn's centre head open out through the C's mouth (`part_coxa._to_mouth`): the
slide-on path reads 45.98 → 0.00 mm³ against the blank and 50.41 → 0.00 against the real
servo (new J1 checks; on HEAD's fork they read BLOCKED). B81 option (b): no horn pocket,
the horn face bears on the flat hub top (35.76 mm³ under a 0.2 mm pull, the centre head
clear); +X is the bolted direction (the four horn screws). **The reviewer caught R−:** the
inward foot push needs the idler to push the upper plate −X, which is now the mouth, but
`fem_check` still pinned the idler both ways (SF 3.60). On the horn face alone the fork read
**SF 0.49**, deflecting 22.96 mm at the hip cup. Fix: two 3 mm side cheeks
(x 2–22, |y| 16.6–19.6), outside the yaw servo's swept case (≥ 0.67 mm over yaw ±40°),
**+27.5 g a robot** (the fork 14.07 → 19.60 g over D062, channels included). `fem_check` now runs one CalculiX deck per set of
grips. Fork SF V / R+ / R− / L+ / L− 3.47 / 5.08 / 3.57 / 3.25 / 3.03 → **5.71 / 8.47 /
2.47 / 5.95 / 4.85**, R− governing (1.76 mm). The knife-edge fins beside the idler
channel are cut square.

**The panels (B82, B83, B87, B88):**
- B82: the knob is 8.6 tall with a 2.1 mm hex pocket; the M3 × 16 hex head sits 0.1 below
  its top (it stood 0.5 proud) and engages 5.5 mm. Still not captive.
- B83: the I6 male was a key (wide face at the root), so a shoe lifted straight off it
  (0 mm³ at any lift). The ring's male is now an undercut dovetail (the stand keeps the
  key, byte-identical) and the shoe's tapped bore aims 45° at its flank: lifted 1 mm it
  reads 27.61 mm³ (held); a force balance loads both flanks (1.19 F, 0.40 F).
- B87, a bayonet: the rotor drops through keyways in the housing and is captive once
  turned; the deck strike has entry slots, a 0.8 mm cam ramp over 0–50° and a dwell to
  90°. Rotor into housing 17.78 → 0 mm³, housing skin 73 → 100 %, bite at LOCKED 0 → 2.68
  mm³, a locked panel lifted 0.2 holds. Turned from under the deck with a flat
  screwdriver ≤ 5 mm wide: **the reviewer disproved the coin** (it gets ~0.5 mm into the
  1.5 mm slot before it meets the frame).
- B88: the hatch magnets sit at r 42.5 on seat bosses: 50.96 → 0.000 mm³, pockets walled
  100 %.
- The review also re-posed the I6 check at the knob's real working range, made
  `latch_strike` refuse a frame under 6.0 mm, and found station 162's latch inside the
  bay's footprint (B100) and a cam bite that may set PLA lugs (~108 MPa at their roots
  against ~50 MPa yield, B99).

**The tray and the clip (B89, B90, B94):** Pi standoffs 58 × 49 from `PI_HOLES` unscaled
(a pin down each hole 14.76 → 0 mm³; a check derives the holes from the Pi 5 drawing), 11 mm
tall so the IMU fits under the Pi (its top to the Pi's keep-out −2.60 → 1.20 mm); IMU holes
20.32 × 17.78 over nut recesses (M2.5 × 10 + DIN 934, tail to the deck −0.30 → 0.90 mm).
`link_clip`: inner jaw 2.4 → 1.2 mm, posed over plate A's slot, against the swinging knee
parts 14.67 → 0.00 mm³ with 1.0 mm running clearance. The reviewer made the jaw 8.2 deep
(at 8.0 the nubs cleared the slot edge by 0.1 mm; a new "snaps home" check fails there)
and added the knee plug to the sweep.

**The motion (B76 fixes 1, 2, 3, 8; B79):**
- **Fix 2, the soft-landing swing** (`pebble_gait.swing_profile`): the swing leaves and
  meets the ground at the stance foot's speed, with zero vertical speed. The velocity
  step at lift-off/touchdown is 3.05 / 3.06 / 2.77 → 0.003 rad/s (walk / strafe / turn),
  joint peaks 3.54–3.63 → 2.96–2.98 rad/s, and a new `KINK` verdict fails any step over
  0.5. A zero command is now exactly the standing pose (the lift fades in below 15 mm/s).
  **The cost:** the lift is fastest 12 mm up, inside the 15 mm band held to the loaded
  3.0 rad/s, so the envelope fell **45.5 → 34.2 mm/s, 0.246 → 0.185 rad/s**. The owner
  kept T 2.0; T 2.2 would give 39.8 / 0.215 (B103).
- **Fix 1, the command slew** (`CommandSlew`, 25 mm/s² on the fastest foot). **The
  reviewer found its ramps over the loaded budget:** a start to 30.1/16.2 mm/s peaked at
  3.021 rad/s, 5 of 1440 grid ramps failed (worst 3.036), and a lower acceleration did not
  cure it (3.0003 at 10 mm/s²). A 0.2 s first-order lag after the rate limit did: 0 fails
  over 1440 grid, 2400 fine-phased and 280 mid-ramp retargets, worst 2.992 (its first
  version snapped the tail and the checker caught that too, 3.0047). Standing → 34.2 mm/s
  takes 2.36 s; a start moves a joint 3.38° per 20 ms tick (was 28.4°). Wired into the
  supervisor's normal and recover states, the playground (the touchdown gate compares the
  target, so a ramp never ends a hold), the harness sim and `gait_node`; a stop is never
  eased (BRACE in the same tick). The review fixed two faults in the wiring patch: a
  gyro-trip PLANT mid-ramp jumped to full speed (9.11 → 4.04° a tick), and after righting the gait
  resumed mid-stride (16.75 → 3.38°). The RL envs stay unslewed until a retrain.
- **Fix 3, a per-tick goal speed** (`bus.goal_speeds`): 1.3 × the goal step per tick,
  floor 50 counts/s, cap 3063 (4.7 rad/s), the cockpit slider a ceiling. On the mock the
  servos move 63–77 % of each tick instead of 21 %. The sag under load is B98.
- **Fix 8, the minimum-jerk ease** (10u³ − 15u⁴ + 6u⁵): 10 ms into a key the acceleration
  is under 12 % of the smoothstep's, but the peak is 1.875× the mean (was 1.5×).
  **`point_there` failed** (SPEED_FREE 4.02 rad/s) until its tail moved t 4.5 → 4.7 (3.58
  rad/s, p95 15.9°); the `compose_gesture` example and the WAVE fixture went to 1 s per arm
  move (4.13 → 3.31); the pose solver's lead-in sizes itself with the ease.
- **B79:** `register_dump` reads STS 0–87 and SCS 0–83 (0–73 when a servo refuses) and
  prints firmware, return delay, Lock, ACC, 85, 86 per servo.

**The body layout (B51, B84–B86, B92, B93), a proposal:**
[docs/archive/BODY_LAYOUT_PROPOSAL_2026-09-28.md](docs/archive/BODY_LAYOUT_PROPOSAL_2026-09-28.md). Five measured facts decide
it (the leg drops meet under the deck, the bay hangs below the hook feet, one east-west
band is free, the tray fits only at (0, 0) with its tabs inboard, the carapace ceiling);
option A, a keel tub 8 mm under the deck plus a hub shelf, is recommended. Its review
rebuilt the parts from the tree after the other groups' edits: B no longer fits (its Pi
492 mm³ into the carapace), A's carapace margin is 5.5 mm, not 10.5, the strap needs width
(`bay_w` 53, or no strap inside), the door latch a south-wall boss, and legs 1–4 can fold
into the tub within their soft limits (B97). Thirteen owner decisions, its §8.

**The loop:** `params_rev` D063, fingerprint `1f953c89f979` → **`87215110e9c4`**, robot
2694.6 → **2727.7 g** (torso 1434.6 → 1439.7, coxa link 78.3 → 83.9). CAD 34/34 +
fem_check = 35/35, byte-stable on a third `--derived` pass (0 of 590 files changed; the
first pass builds the print pack before FEM runs, so it still carried the fork's old
verdict, B110). `run_sim`: walk 247 → **189 mm** (of ~198 commanded), turn 55.4 →
**41.8°**, max tilt 0.84 → 0.41°, height std 0.34 → 0.19 mm, drift 3 → 0 mm. Standing
shove floor 30 N = **1.12 BW** (per direction unchanged); the regen's walking run was not
like for like (a raw 45 ask, shoved at 3.0 s while the slew was at 43.2 mm/s), re-run
below (B104; the D062 JSON says walking min 20 N = 0.76 BW at a steady 45, and 9k's
"walking 24 N" disagrees with it: the JSON is the record). Knee self-right 1.479 → **1.496
N·m = 50.9 %** (HOT; slightly conservative, the audit counts the hand twice, B106).
`recover1` system 20/20 (back 6/6, side 7/7, tumble 7/7), hardware 0/20, no righter 11/20,
unchanged. Gestures 19 rows, 0 FAIL (4 clean, 14 TRACK, 1 MARGIN_WARN); gait-row p95
16.5–18.5 → 12.9–14.8°. Cliff lip band 11 of 364 (9–15°; 3 fire the void guard, then tip:
fixed below). Print: one leg 136 g / 7.3 h → 142 g / 7.6 h, four more 287 g / 15.3 h → 310
g / 16.5 h; the pack is 44 pages, the fork "SF 2.47, governing case R−". Fast ladder 1073
passed (sim 776, harness 113, gait 96, driver 88); slow 2 passed + 3 strict xfails;
`cad/test_fem.py` 9; ruff clean. After the fixes below: fast 1074 (sim 777), 2 strict
xfails; slow 3 passed + 3 strict xfails.

**After the regen (2026-09-29): the cliff and the walking shove.** Three of the 11 lip-band
falls fired the void guard and tipped while backing off: the gate held the gait on the leg
over the void while the leg it had put back down stood on the lip, alone holding the front
up (the CoM ~60 mm outside the other legs' triangle); at the 3.5 mm probe lead the 30 mm
probe took ~0.75 s, the lip foot slid off first, and the void fired 0.90 s into the hold.
Nothing after the fire saved them (a 3× retreat, 20–25 mm more lift, a 40–60 mm lean-back);
a fire forced up to 0.70 s into the hold did. Fix (`sim/playground.py`): a late foot held by
the gate seeks at once with a 7 mm lead (`PROBE_HOLD_LEAD_MM`; the void still needs the real
foot 26.5 mm down) while the body has not tilted 1° past its lowest tilt in the hold
(`PROBE_HOLD_ROLL_DEG`: without it a shin resting on a bump on rough ground rolled the body
to 7° into a false void); not in the careful walk. The void now fires 0.56 s into the hold,
tilt after ≤ 1.6°. The 364-approach grid: 11 → **8** falls, none after a fire (slew off 7,
careful walk 6); the 8 tip during the next leg's swing, before any touchdown to hold on, out
of a probe's reach (B33 c). Terrain A/B, 90 walks: 0 falls and 0 false voids before and
after, progress +1.1 %, 7 walks tilt 1–2° more (worst 7.3°), 3 probe 25 mm or deeper.
`shove_envelope` now asks the envelope (`scenes.walk_ask()`, 34.2 mm/s) and shoves at 5.0 s,
1.64 s after the ease-in: walking 30 N in all six directions, min and mean **1.12 BW** (B104
closed).

**Tests that pinned a number D063 moved** got the new number and a reason: the envelope
(34.2 / 0.185), the `run_sim` refs (189 mm / 41.8°), the RL walk 30 → 40 s (to keep its
1 m bar), the gyro-trip test 0.3 → 0.05 rad/s (the soft landing peaks the walk's gyro
at 0.11), the rewind rate at the envelope 1.12 → 1.35, the void-guard grid 10° → 15°
(10° is in the lip band now) with the strict lip-band xfails re-pinned to the new falls,
the fall-during-gesture shove 60 → 65 N (a chaotic tumble: on the new model 60 N tips it
to 129° and it rolls back), and the golden hashes for the re-timed `compose_gesture` text.

**Next:** the owner: the body layout's §8 (option A, and an I4/I5 revision) and T 2.2
(B103). The bench: the goal-speed sag (B98), `frame_coupon` + a cartridge for the 0.2 mm
bite and the I6 wedge (B99), 85/86, the return delay and Lock (B32); B80 no longer blocks
the leg's batch, and the horn coupon still comes first (B23). Done after the regen:
`point_there.json` re-saved (`checked.params` 0.1/D063, still PASS at 3.58 rad/s), the gait
GIF and joint plot re-made (`python gait/demo_walk.py`), the new printability warnings in
PRINT_PLAN, and goto's cap: the old 40 s stopped a 1.5 m goto at 1.275 m, so the cap is
now derived from the envelope (`harness.capabilities.goto_cap_s`: 1.5 m × 1.2 / 0.0342 m/s
+ the 2.36 s ease-in = 55 s; a 1.5 m goto arrives in 46.1 s), the texts say ~0.034 m/s and
55 s, a detour walks up to 15.5 s (0.45 m) and its no-progress clock counts its own progress
(2 cm farther from where it began: a sidestep into a 14 cm box the lidar cannot see ends `stuck`
after 6.0 s instead of pushing 16.6 s), `go_back_to` re-aims a leg a detour made time out (the
MCP proxy waits 300 s for it: a detour running at the cap finishes first), and the scan turn
is timed to the envelope's 10.6°/s (14.2 turned 23.7° of 30°).
Still open: the fork drawing's open channels (B105), the torque audit's double
hand (B106), the 8 lip-band falls (a lip landing needs its own detector, or a look-ahead
sensor, B33 c), the experiments' time limits still sized for 45 mm/s (B111), the cliff
safe-stop experiment's march-in-place verdict (B112), `ArmedGait`'s envelope (B113), and a
retrain on `87215110e9c4` with the slew in the RL envs (B34).

---

## 2026-09-28 · Session 9l — an assembly guide, and what writing it found (B80–B96)

**Ask:** research what to order (servos, electronics, filament), then: are the ST3215s
good servos (strong, robust, controllable, fluid motion)? And an assembly guide.

**Ordering:** `bom/BOM.csv` re-priced (bench kit $282, core $1,231). Servos from a source
that ships the aluminium horn the couplers are cut for; order by suffix (C018/C047);
Pi 5 4 GB (the brains are on the desk); the D500's real size is 54.0 × 46.3 × 35.0
(datasheet), not 38.6 × 38.6 × 33.5; magnet strikes must be steel.

**Servos ([docs/SERVO_NOTES.md](docs/SERVO_NOTES.md), B76–B79):** the right servo to
start with (walking loads the knee ~12 % of stall), but a stiff position servo with no
current loop. Most of the jerkiness Pebble would show is software: the swing lands at
188 mm/s, a command change can snap a joint 28° in one tick, the bridge streams ACC 0 /
speed 0, keyframes stop at every key. The sustained budget `continuous_frac` 0.65 is a
guess twice the datasheet's rated torque (B77). The HL-series constant-current servo is
a same-body option to bench one of (B78; the driver writes addr 44, its goal torque).

**The guide ([docs/ASSEMBLY_GUIDE.md](docs/ASSEMBLY_GUIDE.md)):** printed parts + BOM
to a robot powered up on its stand, in build order, with 13 step pictures drawn from the
CAD (`cad/gen_assembly_views.py`, now in `cad-check --derived`, 34/34). Five sections
were written from the code and each checked by a second reader; a critic then found
contradictions and suspected CAD faults, and each fault was settled by building the parts
and measuring. **Seventeen are real (B80–B96), and several block a print:**
- **B80: the coxa fork cannot go onto the yaw servo (leg step 1).** The horn's centre
  head catches the hub's rear lip by 1.0 mm and the idler the upper plate by 1.5; the C
  would have to spring 2.5 mm open. `check_assembly` tests the final fit, never this
  path. A fix (open the idler pocket and a head channel to the mouth) was measured on a
  scratch fork: 0 mm³ all the way in. Nothing printed from batch 2 until it lands.
- **B82:** the I1 thumbscrews are M3 × 16 hex head (A-19): an M3 × 10 in the knob stops
  0.5 mm short of the insert, and a socket head spins in its hex pocket. Not captive.
- **B84–B86:** the battery bay is not a part, the stand crown fills the belly, and
  `dock_block` cannot be bolted down: the robot runs tethered until they land.
- **B51 (rewritten), B89, B90:** no pose of the avionics tray + rails clears the five
  coxa bases; the Pi standoffs are 58 × 44.1 (a 0.9 scale since the first import), not
  58 × 49; the IMU pad misses the BNO085's holes and stands 0.6 mm into the Pi.
- **B87, B88:** the I3 latch cartridge cannot be assembled or reach its strike; the
  hatch magnets have no seat. The shell sits on its seam tongues and foot magnets.
- Also B81 (the yaw hub rides the horn's centre head, not its pocket), B83 (the I6
  set-knob bore misses the dovetail), B91 (the hand's lead is ~0.5 m, B-17 is 100 mm),
  B92 (no 12 V node part), B93, B94, B95 (with the hand no tube reaches l3 = 135), B96.
Docs, BOM, print plan and CAD comments were corrected to match (no geometry change).

**Next:** the CAD fixes, B80 first (it blocks the leg print), then B89/B90 and B82 before
the tray and the port coupon; B84/B85/B51 are one body-layout redesign.

---

## Earlier sessions (2026-07-27 → 09-27) — archived

Sessions 8e–9k (2026-09-08 → 09-27) are kept word for word in
[docs/archive/BUILD_LOG_2026-09-02_to_2026-09-27.md](docs/archive/BUILD_LOG_2026-09-02_to_2026-09-27.md),
sessions 1a–8d (the design era before D047) in
[docs/archive/BUILD_LOG_2026-07-27_to_2026-09-01.md](docs/archive/BUILD_LOG_2026-07-27_to_2026-09-01.md).

| Session | Date | What happened | Decisions |
|---|---|---|---|
| 9k | 09-27 | the femur becomes a box, the sled fits its bay, the print pack is rebuilt | D062 |
| 9j | 09-27 | FreeCAD in the loop: a stress check, a viewer, part drawings | D061 |
| 9i | 09-26 | repo review + clean-up: one BOM, the D047 CAD follow-ups, a generic setup, docs condensed and archived | D058–D060 |
| 9h | 09-25 | three brains installed and measured; stop never waits for a model | D055, D055a, D056, D057 |
| 9g | 09-24 | talk to it: fast speech, an eye that measures, memory, an all-in-one brain, the phone front end | D053, D054 |
| 9f | 09-24 | the full review and D052: the sim stops flattering the servo | D052, D052a |
| 9e | 09-24 | toward the real robot: gesture studio, chord designer, servo realism, the hardware bridge, tailnet | D051 |
| 9d | 09-23 | cockpit round two: feet that feel for the floor, recordings, worlds on disk, the walker, voice | D050 |
| 9c | 09-23 | the cockpit: a browser playground on one running sim | D049 |
| 9b | 09-23 | "the push flies and jitters": the shove model, two jitter sources, a righter retrain | D048 |
| 9 | 09-22 | full review, then the leg is rebuilt around the real servo; reprint plan + shopping list; repo goes public-ready | D047 |
| 8f | 09-17 | the dry-fit fails: a joint suite, then D046 | D046 |
| 8e | 09-08 | the dev laptop runs the loop: the harness in physics, a local model drives it, the capped retrain is negative | — |
| 8d | 09-01 | FALLEN / RIGHTED reflex states; recovery reward v2 (`recover2` negative, the sigma cap); the talk-to-Pebble intent layer; live lidar `scan_summary` + a 5/5 patrol; torque re-audit on CAD masses; servo order v2 | D041–D045 |
| 8c | 08-31 | the dev laptop takes over: setup guide, keyboard teleop, a local-LLM brain, the flat chat-driving world, the RL ground-up tour | — |
| 8b | 08-30 | print-physics audit as a CI stage, sim masses from the CAD tree, gesture library v2; RL gait runs 2–3 and the first self-righting training | D038–D040 |
| 8 | 08-30 | tree-wide floating-parts audit: six more printables were in pieces; the single-solid gate moves into `export()`; print night re-planned | D037 |
| 7 | 08-30 | the hand hub's floating lugs: a knuckle collar (hand hub v0.2.2); connectivity checks tree-wide | D036 |
| 6c | 08-07 | the playground: interactive sim, live tuning, chat-driving over MCP | — |
| 6b | 08-07 | MCP harness v0 live, the cliff safe-stop wired and measured, the beckon gesture, viewer weekend mode, parallel CI | D034, D035 |
| 6 | 08-07 | print-weekend prep: fit ladder v2 (and a real v1 bug), servo blanks, tibia clamp v0.2, a GO/NO-GO plan | D032, D033 |
| 5c | 07-31 | cable clips and dock mechanicals, the tree-wide CAD CI (19/19), the vision/interaction plan, viewer v0.3, a showcase reel | — |
| 5b | 07-31 | CAD interaction pass, deck v0.4, RL actually trains, jazz hands | D030, D031 |
| 5 | 07-31 | sim loops closed (brace, SLAM, yaw), the RL kit, chord-speak v0.2, carapace v0 | D025–D029 |
| 4 | 07-30 | bench-readiness pack, ROS 2 scaffold, reflexes, the six standard interfaces, the perception stack | D019–D024 |
| 3 | 07-29 | robustness proven in sim, the batch 0+1 order sheet, a servo trade study | D015–D018 |
| 2 | 07-28 | Pebble walks in physics; the deck; the first print pack | D011–D014 |
| 1b | 07-28 | CAD sprint: the full leg, the hand, the gait engine | D001–D010 |
| 1a | 07-27 | project founded: the founding plan ([docs/archive/ROCKY_MASTER_PLAN_v1.1.md](docs/archive/ROCKY_MASTER_PLAN_v1.1.md)) | — |

---

*Template for new entries:*

```
## YYYY-MM-DD · short title

**Done**       — what physically/digitally happened
**Decisions**  — anything future-you must not re-litigate (also → docs/decisions.md)
**Broke**      — failures, surprises, measurements that disagreed with the model
**Next**       — the first 1–3 actions of the next session
```
