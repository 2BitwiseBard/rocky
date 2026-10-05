# ROCKY Build Log — Engineering Notebook

*Convention: newest entries first. Every work session gets an entry: date,
what happened, what was decided, what broke, what's next. Field notes and raw
measurements go in `NOTES_INBOX.md` and are filed later. Keep entries honest —
failed prints and dumb bugs are the most valuable lines in this file.
Sessions 1a–8d (before D047) are archived word for word; the table at the end
indexes them.*

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
[docs/BODY_LAYOUT_PROPOSAL.md](docs/BODY_LAYOUT_PROPOSAL.md). Five measured facts decide
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

## 2026-09-27 · Session 9k — the femur becomes a box, the sled fits its bay, the print pack is rebuilt (D062)

**Ask:** apply B74 and B72, and redo the print pack.

**B74, the femur (D062):** plate B gets a deck over the bridge (x 21–52, |z| < 11) and
8 mm rails. `fem_check` femur **1.22 → 2.18** (predicted 2.16 by the what-if), knee
deflection 4.5 → 1.06 mm; all five leg parts pass. The four plate-B M3 holes, which broke
out of the rails, sit in solid plastic now (a new `part_femur` check) and take M3 × 12,
a new BOM line A-18 (an M3 × 8 through 8 mm reaches nothing; through the old 6 mm it had
2 mm of thread). Bench kit $284.94.

**B72, the sled (D062):** `pack_w` 46 → 44: the sled is 48.8 mm in the 50 mm bay, and
`part_battery` fails when sled + pack exceed the bay. The BOM's pack (43 mm) fitted already.

**The loop:** `params_rev` D062, robot 2694.6 g (+37.9), fingerprint `1f953c89f979`.
Re-run: walk 247 mm, turn 55.4°, standing shove floor 30 N (29 before: the heavier
femurs help), walking 24 N; knee self-right 1.48 N·m = 50.3 % (HOT by a point);
`recover1` system 20/20, hw 0/20, unchanged; gestures 0 FAIL. One test pinned the
MJCF's summed mass to 0.05 g of the budget; the 32 geom masses are written to 0.1 g, so
it now allows 0.5 g (~3σ of rounding). Fast ladder 768 passed.

**The print pack** (`cad/gen_print_pack.py`, 44 pages): a cover (params revision,
filament table, batches from `print_estimate.json`, the strength verdicts), the print
order, then one section per batch in print order with a tick list, a sheet per part
(three views in the PRINT pose with the bed line, supports, estimate, the live
printability numbers, the stress verdict + picture for leg parts) and its TechDraw
sheet merged in (pypdf, pinned; a content-hash `/ID`, so the file is byte-stable). The
PARTS data was re-checked against PRINT_PLAN.md (fit ladder added, per-batch quantities
and materials, M3 × 12 for plate B).

---

## 2026-09-27 · Session 9j — FreeCAD in the loop: a stress check, a viewer, part drawings (D061)

**Ask:** set up the FreeCAD MCP server, then find where FreeCAD improves the workflow:
a stress check of the printed leg, opening parts for inspection, dimensioned drawings
(the print pack may be redone).

**Setup (outside the repo):** the neka-nat `freecad-mcp` clone in `~/Development/`, its
add-on in the Flatpak's `Mod/`, the server registered through the MCP manifest. The
Flatpak carries gmsh 4.15 and CalculiX 2.23, which is all the FEA needs.

**`cad/fem_check.py` (`rocky.sh cad-check --fem`):** build123d solid → gmsh second-order
tets (straight-sided: curved midside nodes inverted small elements on tight radii) →
a CalculiX deck (held nodes pinned, loaded nodes on a rigid body) → `.frd` → von Mises
and tension across the print layers, judged at the 99.9th percentile outside the grips.
Loads are servo-limited (the foot force that stalls the first servo). The pipeline
matches a cantilever by hand (4.26 vs 4.20 MPa, 2.655 vs 2.667 mm; pinned by
`cad/test_fem.py`). Results: `coxa_yaw_base` 2.93, `coxa_fork` 3.03,
`tibia_knee_carrier` 3.92, `horn_coupler` 7.81 PASS; **`femur` 1.22 FAIL** in the
lateral cases, 1.16 on a 6× finer mesh. The first femur model welded the bridge walls
to plate B along their whole top edge (SF 1.20 with its peak on that weld); the model
now joins the plates only at the four bosses, as the bolts do.

**What-ifs (FEM only, the CAD unchanged):** rails 6 → 9 mm 1.64; walls 3 → 5 mm 1.29;
a plate-B deck closing the box over the bridge 1.71; deck + 8 mm rails **2.16** (+14 g,
knee deflection 4.5 → 1.05 mm), clear of both servos, plugs, the knee sweep and the
fork. Proposed as B74; not applied (it moves the mass and the fingerprint).

**`rocky.sh cad-open`:** opens STEP files in FreeCAD; `--fem PART` wraps the CalculiX
result in a FreeCAD Analysis first (`cad/fem_to_freecad.py`: FreeCAD 1.1's importer
raises on a bare multi-step `.frd`).

**`cad/gen_drawings.py` (`rocky.sh cad-drawings`, in `--derived`):** FreeCAD's console
binary with its GUI on Qt's `minimal:enable_fonts` platform (`offscreen` segfaults;
plain `minimal` draws boxes for text), in a private user directory so the owner's
FreeCAD and its MCP add-on are untouched. One A4 sheet per leg part (10), ~6 s each.
The hole table is read from the solid (concave cylinders that close a circle), which
is how plate B's missing M3 holes turned up: they break out of the rails (B74). The
PDF bytes differ on every run (threaded hidden-line removal), so a sheet is redrawn
only when its STEP's geometry changed.

**Next:** decide B74 (femur deck + rails) before printing the femur in PETG; pull
coupons for real allowables (B75); redo the print pack around the drawings.

---

## 2026-09-26 · Session 9i — repo review + clean-up: one BOM, the D047 CAD follow-ups, a generic setup, docs condensed and archived (D058, D059, D060)

**Ask:** go through the whole codebase; condense or remove old things, documentation
above all; update what is stale, the BOM and the CAD.

**The audit (read-only first):** nine areas (sim docs in two halves, BOM, the record,
CAD, sim code, core code, hardware docs, repo hygiene), **209 evidenced findings**
(74 update, 34 fix, 21 genericize, 18 delete, 17 condense, 17 keep, 11 investigate,
6 regenerate, 5 merge, 4 move, 2 archive), each with a file:line quote and a proposal,
45 of them flagged for the owner's call. Then two implementation stages and a rerun
of the sim experiments on today's model.

**Stage 1, code (five commits):**
- **BOM** (`c7fd4b1`, D058): `bom/BOM.csv` (63 rows; phases A bench kit, B full robot,
  C senses, D voice, X optional) + `bom/README.md` + `bom/totals.py` (checks every line
  total) replace the xlsx and five order sheets. Prices checked 2026-09-26: bench kit
  **$279.94**, core robot (A–C) **$1,257.91**, optional $422.59. 16 × ST3215 (no
  STS3250), one soft-case 3S 5200, the D500 lidar, BNO085 on SPI; no 683ZZ, no USB-UART
  dongle. Nothing ordered.
- **CAD** (`45fdae1`, D059): an 11 × 11 mm leg harness channel through `coxa_yaw_base`
  from the deck's I1 cutout to the yaw plugs, on the −Y side (the part 25.19 →
  21.23 cm³; six new `check_assembly` checks); the star-board bracket v0.2 bolts to
  existing deck grid holes (0,20)/(0,40) with a layout audit (5.38 → 5.07 cm³);
  `run_all_checks` 23 → **28 modules** (servo_st3215, servo_mount, part_port_coupon,
  leg_assembly, check_interference; four checks that printed CLASH but exited 0 now
  fail) and `rocky.sh cad-check --derived` adds the four generators (32/32, 230 s);
  byte-stable exports (STEP only on a geometry change, an invariant PDF, fixed STL
  headers); `print.bed_mm` and `interfaces.battery_sled.xt60_panel_mm` in params;
  the print pack moves to `cad/out/PRINT_PREP_PACK.pdf`; 60 stale PNGs and the old
  dry-fit posed files removed; the STEP's Apache-2.0 text in `cad/ref/`.
- **Generic setup** (`72e3f1d`, D060): machine knobs in a git-ignored `rocky.env`
  (template `rocky.env.example`, every `ROCKY_*` variable), read by `rocky.sh` and by
  `sim/envfile.py` for Python started without it; the environment wins. The code no
  longer reads one machine's own key variable (the key is `ROCKY_LLM_API_KEY` or
  `ROCKY_LLM_KEY_FILE`); the D055 model ids are the reference setup's
  overridable defaults; brain-install's models dir defaults to `~/models`
  (`ROCKY_MODELS_DIR`); CI pins MuJoCo 3.12.0 (`constraints.txt`) and build123d 0.11.1.
- **Core** (`730658d`, D060): driver sign bits per register (STS `PRESENT_LOAD` bit
  10, `POSITION_OFFSET` bit 11, VERIFY-ON-BENCH; an offset past ±2047 is refused);
  the ROS 2 plugin takes servo ids from joint names; the watchdog's retry ladder is
  derived inside the D052 envelope (24 mm / T 2.0 s / 45.5 mm/s → 34.8 mm / 3.0 s /
  40.6 → 42 mm + 10 mm body / 4.0 s / 30.4; the old 32/46/52 mm ladder had no
  envelope at all); the gait GIF and joint plot regenerated on the budgeted gait
  (T 2.0 s, step 24 mm, peak coxa 27.9° where the old demo reached 49.8°, past the
  ±40° limit); dead audio and harness files out. Driver tests 69 → 79.
- **Sim structure** (`5a70f37`): 20 one-off experiments + `diag_brace_phase.py` →
  `sim/experiments/` (results → `sim/experiments/results/`, clips →
  `sim/experiments/out/`, `ROCKY_EXPERIMENTS_OUT=DIR` redirects; run from the repo
  root); the 19 pre-D052 result JSONs and their figures →
  `sim/experiments/results/pre_d052/`; `sim/scenes.py` holds the scene helpers live
  code had imported from experiment scripts; `run_sim.py` writes to
  `sim/out/run_sim/` by default; dead code, the PPO smoke, four negative-run
  checkpoint folders and a stale walker checkpoint removed (git keeps them).
  **Bug found:** `run_fairing` built its rubble from a copy that skipped the per-box
  colour draw, so D023's fairing A/B ran on different fields than its baseline.

**Stage 2, docs (this entry):** one page per topic. `docs/PRINT_PLAN.md` (the dated
name dropped; the fit-ladder GO/NO-GO table from the deleted print-night pages),
`docs/WIRING_HARNESS.md` absorbs the star board, `docs/PERCEPTION_PLAN.md` replaces
the vision and sensing plans, `docs/RL_GUIDE.md` absorbs the RL tour,
`docs/SIM_GUIDE.md` absorbs the MCP contract and the playground page,
`docs/BRAINS.md` is the short brain guide, `docs/PLACES.md` is new, the README takes
the laptop-setup page. `docs/archive/` ([index](docs/archive/README.md)) keeps word for
word: the founding plan v1.1 (now bannered, with the D016 hand-servo correction), the
brain-model report, BUILD_LOG sessions 1a–8d, and decisions D036–D057 at full length
(the log condenses them to ADR-lite rows). D001–D035 stay as written, with AMENDED
markers where a later decision overrides them. `NOTES_INBOX.md` is back to its header
(its 09-08 and 09-17 entries are sessions 8e and 8f below). The backlog is one Open
and one Closed table, with the stage-1 follow-ups as B42–B65.

**Numbers that changed on purpose:** robot fingerprint `ceb63a1254c3` →
`7d376178fe27` (torso 1448.9 → 1435.2 g, robot 2670.4 → 2656.7 g, URDF ≡ MJCF at
2.657 kg); CAD checks 23 → 28 (32 with `--derived`); print estimate batches 0–3
378 g / 20.2 h, whole plan 1111 g / 53.8 h; fast tests **1,039 passed + 2 strict
xfail** (driver 79, harness 111, gait 75, sim 777 collected; last recorded 454 at
D053) in ~7.8 min; `run_stuck` on today's model with the budgeted ladder 4/4, 4/4,
2/4, 2/4 at 30/35/40/45 mm rubble (no watchdog 2/4, 0/4, 0/4, 1/4).

**Decisions:** D058 (one BOM: the 2026-09-22 electronics calls), D059 (the D047
follow-up CAD fixes), D060 (the repo goes generic and condensed).

**The rerun (same day, fingerprint `7d376178fe27`):** every experiment, the `sim/out/`
records (`shove_envelope`, `torque_audit`, `rl_curves`) and the demo clips re-made on
today's model; the table with both columns is `sim/experiments/README.md`. Held:
`run_sim` 246 of ~261 mm (tilt max 0.84°, turn 55.4° at 0.246 rad/s), `recover1` 0/20
`handoff_ok` with the system 20/20 vs 11/20, `recover6_d052` 0/20 (system 11/20 = no
righter), SLAM ATE 8.2 mm, and D018, D026, D027, D034, D043. Moved: the rim-shove floor
(BW now on the 2.657 kg torso) 5 N grid standing 25–35 N by direction (min 0.96 BW),
walking min 20 N (0.77 BW), 1 N grid 29 N (1.11 BW); the torque audit's self-righting
knee 1.488 → 1.459 N·m, 49.6 % of stall, **HOT → WARM (D044)**; the `recover1` legacy
height test 5/20 → 4/20; the cliff stop 186.8 → 252.3 mm short of the edge; odometry drift
on rubble 1.02 → 8.06 %. Verdicts that moved: **D017** rubble crossed reliably ≤ 30 →
≤ 25 mm (still snags, never falls); **D022** the v1 brace's walking floor 33 → 40 N no
longer reproduces (27 → 27 N); **D023** the watchdog at 40 mm 1/4 → 4/4 became 0/4 → 2/4,
and the fairing, first run on the same fields as its baseline, is still rejected but
"helps only at 45 mm" is false (it helps a little at 30–35 mm, hurts at 45); **D025** the
envelope order flips to v2 55.4 ≥ base 54.4 ≥ v1 53.2 N (one ladder step); **D040** turn
80° → 48°, sidestep 170 → 140 mm under the speed envelope; **D042/D045/D048** the
fallen demo falls 4/5, not 5/5 (seed 4 braces and stays up; the other 4 right and walk
away). The decision rows keep their dated numbers.

**Final pass (2026-09-27).** Code: ruff in CI (B64); a repo-root `conftest.py` sets
`ROCKY_ENV_FILE=/dev/null`, so the suite runs on the reference setup (B63); the benches
force their scratch cockpit conf (B62); `--mock` rehearsals, the cockpit's mock bridge
included, write only under `bench/out/mock/`, never `bench/calibration.yaml` (B71), and
`pose_check` works on a partial bus (B70); six part modules' checks and `part_shell`'s
bed fit now fail the exit code (B52); `params_rev` D052 → D059 (B53); `run_stuck_voiced`
defaults to seed 1, which crosses; the print pack and `bom/BOM.csv` cite the current
docs. Docs: a RERUN 2026-09-26 note on each decision row the rerun moved; B1 closed
NEGATIVE, B66–B68 opened; the facts that lived only in the D036–D057 archive moved into
SIM_GUIDE, PRINT_PLAN and DESIGN_CHANGE_GUIDE §8 (adding a tool); personal details cut
from the archive; every relative link and anchor in the Markdown resolves. Two
independent reviews (docs truth, code + CI + generic) then found 19 items, 18 fixed: the
MCP server now follows `ROCKY_COCKPIT_PORT` and reads `rocky.env` (B73); the gesture
audit's record is tracked, `sim/out/audit_gestures.json` (B69: 20 rows, 0 FAIL,
`manip_adjacent` 17.4 mm); the runbook's soak and torque-step loads come from
`sim/out/torque_audit.json` (walking 0.34, stance 0.45, self-righting 1.46 N·m), not
D015; the interface-coupon fit criteria and the fixture print poses, lost with the
print-night pages, are back in PRINT_PLAN; the photo's EXIF and phone trailer are
stripped (pixels unchanged); ruff is pinned in `constraints.txt`; the sled-vs-bay
width (50.8 vs 50 mm) is B72. Fast tests at the end: 1,035 passed, 7 skipped (the
brain-install checks, off on the reference setup), 2 strict xfail, in 5 min 43 s.
Left to the owner: the git history still carries the old machine paths and tailnet
name, so a public copy needs a fresh or filtered history.

**Next:** order the bench kit (`bom/BOM.csv` phase A); the fit ladder → `params.print`
(B28); print batches 0–1 (`docs/PRINT_PLAN.md`); the first servo settles the horn
(B23) and the VERIFY-ON-BENCH sign bits (B32).

## 2026-09-25 · Session 9h — three brains installed and measured; stop never waits for a model (D055, D055a, D056, D057)

**Done**       — `rocky.sh brain-install` + `brain-bench` (built by an 11-agent workflow, 26 review findings fixed). The owner's downloads (Qwen3.5-4B Q8_0, Qwen3.5-9B UD-Q6_K_XL, Gemma 4 12B UD-Q6_K_XL) installed with their projectors; five brains measured on the robot's jobs with one harness. Stop gate in every model mode, `move(forward_m, left_m)`, two prompt rules, per-model box order, world change forgets the eye, `event_seq`, scratch gesture library for the bench. Docs: BRAIN_MODELS status table + verdict, COCKPIT_GUIDE (choosing a brain, saying stop, move), SIM_GUIDE.
**Decisions**  — D055. `qwen3.5-9b` stays the default all-in-one brain, `qwen3.5-4b` is the small one, `gemma-4-12b` the Gemma option, the Q6 9B a delete candidate, the 26B and the 3B eye are not brains.
**Broke**      — Four `until ! pgrep -f` waiters spun all night (the pattern matches its own `zsh -c`); the auto-mode classifier refused the real install (it deletes downloads and edits the shared llama-swap config), so the owner ran `! ./rocky.sh brain-install --no-restart`; CI failed twice on the `--help` smoke test until the child ran with `MUJOCO_GL=disable` (cockpit.py's egl default leaks into subprocesses, the runner has no libEGL); a bench 'happy dance' saved a gesture into the repo; Gemma 12B's boxes are x-first, not y-first like the 26B (21.9° → 0.7°); both Gemma models answer a bare "stop" with a chord when unguarded.
**Done (later the same day)** — D056 shipped (b3df144): `harness/capabilities.py` registry, cockpit `/api/capabilities` with a version, the MCP list rebuilt live with `tools/list_changed`, `docs/TOOLS.md` generated and CI-checked; zero behaviour change proven against snapshots of the previous schemas. Fallback chains set to the D055 verdict. The owner's cockpit runs as the transient unit `rocky-cockpit.service`.
**Done (evening)** — B41 measured (D055a): per-family prompt notes lift Gemma 12B to 20/20 and the 9B to 19/20; thinking on hurts. `.gitignore` keeps the owner's personal to-do files (`OWNER_TODO*.md`, `TONIGHT*.md`) out of the public-facing repo.
**Done (night)** — D057 shipped (d9d73c8): place recognition from senses, off by default (`--recognize`), three-verdict bench 21/21 verdicts / changes 3/6 on the second run; `brain-install --uninstall`; the owner's evening checklist lives in the git-ignored `OWNER_TODO.md`.
**Next**       — B38 (h)–(j): confirm added objects from a sidestep viewpoint + name folding, widen the new-room margin, say "ambiguous until named" on the situation line; speaker id + people memory (B39); sensors as params (B40), judged with the three-verdict bench. (B41 closed: thinking on measured worse.)

## 2026-09-24 · Session 9g (laptop, evening) — talk to it: fast speech, an eye that measures, memory, an all-in-one brain, the phone front end (D053, D054)

**Ask:** the phone's mic said "nothing heard" and the whole loop was slow; "smaller
LLMs so it can all be on the GPU"; "a multimodal all-in-one"; "make the UI better for
the phone, a remote mode, a help guide, a better wake word"; "I'll revisit the CAD and
add joints / servos / materials"; "that memory and situational-awareness idea".

**Measured, then shipped (all pushed):**
- Speech: 18–20 s → ~1 s per command (`whisper-fast.service` :8086, base.en CPU greedy);
  hold-or-tap mic with a timer, hands-free listening, fuzzy wake word (`9a66eff`, `1740e26`).
- Brain: the research round (3 sweeps, 22 candidates, each verified) → Qwen3.5-9B
  downloaded, registered in llama-swap (runs alone), measured 7.2 GB / 6.5 s cold /
  57.7 t/s / tool calls 0.2–1.5 s / `look` OK; default multimodal role. lfm2.5-vl keeps
  the fast eye. Report: `docs/archive/BRAIN_MODELS_2026-09-24.md` (guide: `docs/BRAINS.md`).
- Vision: `find_object` with box→floor geometry, 0.7° bearing / 3 cm distance, 3/3 finds;
  `sim/vision_bench.py` (`46ee083`). A VLM is not a cliff sensor (1–3 of 4).
- Memory + awareness: `sim/scene_memory.py`, five tools for every brain and MCP, a
  situation line per turn, chord reactions; go_back_to measured (`876d9db`).
- The robot as data, step 1 (D053): RobotSpec from params, generators read it,
  byte-identical output; `docs/ROBOT_AS_DATA.md` + `docs/DESIGN_CHANGE_GUIDE.md` (`78df557`).
- B34 first retrain on obs v2: negative (0/20 hardware handoff); recover1 stays.
- Phone front end (tab bar, remote + follow, in-app guide, wake-word card): built and
  reviewed in this session; see the commit that follows this entry.

**Not verified:** any of it on hardware; tailnet on the phone was verified by the owner
(HTTPS works, plain http is refused by tailscale serve — type https).

## 2026-09-24 · Session 9f (laptop) — the full review and D052: the sim stops flattering the servo

**Ask:** review everything before a real servo goes on the bus. The owner
had measured that morning (scratch, not in the repo) that with joint
damping = stall / no-load (0.62 N·m·s/rad) the wave gait still walks
(242 of 273 mm; max joint speed 4.2 vs 7.4 rad/s ideal) but turns 145°
where the ideal model turned 157°, and with the torque derated to 1.9 N·m
as well only 103°; walk + turn yawed 74° vs 103°. "The turn envelope must
shrink, and that is the correct answer."

**The review:** 4 reviewers, 29 verified findings, worked as nine packages
(A model identity, B feasibility + authoring, C always-on loop, D1 cockpit,
D2 brains, E bus safety, F hardware-shaped RL, G launcher/CI, W worlds),
then a verification pass (V1) and a second review of the implemented tree
(V2, 27 findings; every HIGH/MEDIUM it could reproduce fixed). The theme
of the findings: the sim was right about logic and flattering about the
servo, nothing checked what a gesture or command asked of a joint, and the
D051 bridge would have enabled torque toward whatever goal a servo last held.

**What broke (found, not caused):**
- Gestures asked the impossible and nothing said so: fist_bump peaked at
  14.8 rad/s (the servo does 4.7 unloaded), beckon 5.9, turn_in_place 5.2,
  jazz_hands stepped its claw 23° in one tick; the gait at 45 mm/s asked
  a loaded knee 4.6 rad/s, and `walk 0 0 0.5` drove the yaw joint at
  7.4 rad/s past its limit.
- `recover1`'s 7/20 was flattered twice: an ideal 5 rad/s actuator, and a
  handoff test (torso z > 0.09) that passes a robot kneeling on its shins
  with 0–2 feet down. On the D052 model it is 2/20 on that test and 0/20
  on the hardware one.
- The icy-floor preset never did anything: MuJoCo takes the max friction of
  a pair and the feet said 1.2.
- 96/200 random courses put an object within 0.3 m of the spawn.
- `rocky.sh train-walk` passed `--env walk`, which the trainer rejected.
- Every MCP `gesture` sent to a running cockpit raised a TypeError (the
  tool wrapper's `name` argument collided with the gesture's).
- The sim thread could die and leave every HTTP request hanging forever.
- The 40 g claw mass was counted twice (2.705 kg compiled vs 2.670).
- In this round's own code, caught before shipping: the hands were checked
  against the 12 V leg window (a false "volt" every poll); the first soft
  entry jumped 20° on a knee resting outside its soft range; the servo
  model's load derate counted the torque-speed line a second time on top
  of the MJCF damping (now off by default).

**What changed (D052; details in `docs/decisions.md`):**
- **One servo identity** in `params.yaml` (`actuators`, `joints` with
  speed budgets loaded 3.0 / free 4.0 / hard 4.7 rad/s, `gait`, `reflex`,
  `sensing`), one loader `gait/rocky_model.py`; MJCF, URDF, servo model,
  shove, driver clamp and cockpit all read it. Damping 0.626 N·m·s/rad,
  forcerange 1.911 N·m, μ 0.8 + torsional friction, the foot surface is
  the IK foot point, spawn 1.4 mm. Robot fingerprint `5a32f772ca99`.
- **Gait** T 2.0 s / step 24 mm (duty 0.8 kept; 0.75 was rejected by the
  checker: CoM 59 mm outside the planted feet for 25 % of the cycle) and
  `WaveGait.budget()` on every command source: 45.5 mm/s at any heading,
  0.246 rad/s in place.
- **One feasibility checker** gating code gestures, keyframe saves, the
  studio, gait presets and the brain's `compose_gesture`; a reach solver
  and teach-by-demonstration in the studio. Gestures reworked until they
  pass.
- **Always-on loop** in the Playground: blended gestures with the reflex
  watching, one void guard for every source, trip latch, lead-limited
  probe, servo model on, NaN hold + 4.7 rad/s clamp on every target stream.
- **RL** on the robot's own action path and sensors (obs v2), checkpoint
  contracts with the fingerprint, `handoff_ok`, shoves in the gait env.
- **The bus gets a safe first move**: standstill-only mirror, soft entry
  (parked goal, 40 % torque, 200 c/s, blend), whole-leg fault cuts,
  degraded / lost with explicit re-arm, heartbeat, port-loss limp, EEPROM
  angle limits (`bench/apply_limits.py`), mirror dropped when the sim's
  reflex leaves NORMAL. Eight deliberate breakages, eight test failures.
- **Cockpit**: loud sim-thread death, request guard (Host allow-list,
  same-origin, JSON), `--unsafe-lan`, brains in `sim/cockpit_brains.py`,
  reactive lidar goto, model panel. CI diffs the generated model files.

**Measured on the D052 tree:**

| | before | D052 |
|---|---|---|
| `run_sim` walk (8 s, ~261 commanded) | 264 mm, tilt 1.03° | 246 mm, tilt 0.84°, height 117 mm (was 129) |
| walk 45 mm/s, 6 s | 259 mm, peak joint 4.20 rad/s | 231 mm, peak 3.14 rad/s |
| turn 0.5 rad/s, 6 s | 155°, peak 7.19 rad/s | 103° (owner 103), peak 3.16 |
| walk + turn (45, 0, 0.35) | 103° yaw, peak 8.13 rad/s | 76° yaw (owner 74), peak 3.18 |
| `recover1` stood, hybrid | 7/20 (pre-D052 model) | 2/20 legacy handoff · **0/20** `handoff_ok` · 0/20 servo nominal · 0/20 randomised |
| system (supervisor + righter + stall ramp) | — | **20/20** vs 11/20 with no righter; 9/9 declared falls end on the stall ramp, 0 on a handoff |
| `recover1` jitter, 40 N shove | 73–75 % pinned, ~10 rev/s, 4.5°/tick | 52 %, 6.7 rev/s, 3.3° (servo nominal) |
| shove, standing | 25 N (0.94 BW) all directions | 30–35 N (1.15–1.34 BW) |
| shove, walking | min 0.76 BW, mean ~0.91 | min 0.76 BW, mean 1.15 |
| gestures (`audit_gestures`) | 5 of 10 code gestures FAIL (speed / limits, same checker) | 20 rows, 0 FAIL; tracking p95 13–19° on arm + gait rows (warn at 10) |
| fast suite | ~101 (97 at D050 + 4 at D051) | 297 passed, 1 strict xfail |

What got worse is the point: the turn envelope shrank, mixed commands scale
down ((45, 0, 0.35) → (19.6, 0, 0.152)), the walkers on disk have no
envelope on the D052 servo (their gait's swing alone asks a loaded knee
4.43 rad/s) and are zeroed until retrained, and no righter earns a handoff
by itself. The shove envelope got *better*, which is suspicious in the
other direction (slower, lower gait plus joint damping; not isolated).

**Still open:**
- **No real servo** has touched any of this; every bridge rule ran on the
  mock. GOAL_SPEED 0, torque-enable toward the last goal, angle-limit
  behaviour and the POSITION_OFFSET sign are VERIFY-ON-BENCH (B32).
- **The tailnet path is unexercised** (tailscale was logged out).
- Owner decision: the 4.0 rad/s free budget exceeds what the MJCF lets any
  joint reach (1.911 / 0.626 = 3.05 rad/s) — a strict xfail until either
  peak torque + a thermal model goes into the sim or free drops to ≤ 3.0.
- The void guard misses cliffs approached at 10–20° (V2 finding); the void
  retreat barely retreats; goto's reactive layer is not a planner.
- On its back with no righter the supervisor loops FALLEN → RIGHTED every 3 s.
- No training on the new contract yet: B34 has the overnight recipe.
  μ 0.8, continuous 0.65 × stall, the SCS0009 numbers, foot-switch forces
  and all observation noise are guesses marked VERIFY.
- No real LLM tool loop or Claude brain call ran; CI has not run on GitHub.
- New backlog: B33 (CAD CoM/inertia, SEA slide joint, ToF), B34 (retrain),
  B35 (the onboard loop).

**Later the same day (D052 follow-ups, D052a):** the owner ruled on the
open speed question — a position servo's instantaneous torque is its stall
torque, and 0.65 × stall is a *thermal* budget. The 1.911 N·m clip had
capped an unloaded joint at 3.06 rad/s, below the 4.7 no-load and the 4.0
free budget; the leg clip is now 2.94 N·m (claws 0.23, with their own
DC-line damping), an unloaded joint reaches 4.71 rad/s, and the D052
strict xfail passes. The continuous budget moved to where time matters: a
heat integral in both RL envs (trips after 3 min at 0.85 × stall, then
derates the joint and costs reward), a `THERMAL_LOAD` verdict in
`audit_gestures`, and accounting in the live sim (sim → robot refused
while a joint is past its budget). A review of that round moved the heat
onto the motor current (force − damping × qvel), not the raw force, which
had counted a free joint spinning at no-load as stall-level heat. Second
package: the void guard's 10–20° misses. The wave gait lifts the next leg
the instant one lands, so two leading legs straddling the edge left the
CoM outside the other three and the robot tipped in ~0.3 s, before its
probe finished. A touchdown gate (a foot that has not felt ground 140 ms
after touchdown winds the gait back and waits) and a retreat that replays
the approach backwards stop walk 45 at all eleven headings tried from −30
to 60° (was: fell at 10, 15 and 20°) and cut a 72-approach sweep from 28
falls to 6; a 1° grid still finds a lip band (7 of 310 falls, a foot
centred on the edge's corner closes its switch and slides off), pinned as
a strict xfail. Third package: an older spawn bug — six files (and three
scripts built on one of them) wrote the leg angles into `qpos[7:22]`,
which interleaves the claws, so every push and terrain trial began with a
9.8° spawn tilt (every `tilt_max` read 9.83°); the tracked push / terrain
JSONs predate the fix. Re-measured on the peak model against an in-memory
1.911 A/B: nothing moves in normal operation (no joint reaches the old
clip: worst RMS load 0.39 × stall); `recover1` 5/20 legacy vs 3/20
(noise), 0/20 `handoff_ok`, system 20/20 vs 11/20 unchanged; the shove
floor is 29 N (1.11 BW) standing, 2 N *below* the clip — D052's "harder to
tip" was the clip letting legs yield into a slide, read on a 5 N grid.
Fast suite 325 passed, 2 strict xfail. Still open: the lip band, the
careful walk as default (1 fall in 72 but 17 % slower — the owner's call),
the thermal constants (VERIFY on the bench), and the rest of the list
above — except the free-budget question (closed) and the retreat that
barely retreated (fixed).

## 2026-09-24 · Session 9e (laptop) — toward the real robot: gesture studio, chord designer, servo realism, the hardware bridge, tailnet (D051)

**Ask:** "how do we get it set up with tailscale?" and "what else makes it
ready for playing with — gaits, gestures, sounds, better realism, more tools —
so I'm ready to use it with the robot one leg at a time?"

**Shipped (all in the cockpit, `./rocky.sh cockpit`):**
- **Gesture studio** — gestures as keyframe JSON (`gait/gestures/`), posed live
  in physics, snapped, scrubbed, played, saved; they join every gesture list
  (teleop, console, brains, MCP). `gait/pebble_keyframes.py` is the player and
  returns the same `(q, claw)` stream the code gestures do.
- **Voice & sounds** — chord-speak plays in the browser (phone included); a
  chord designer over `chordspeak2.py`'s syllable model, load a canon word,
  edit, hear, save as a new lexicon word (`audio/custom/`, `audio/samples_custom/`).
- **Gait lab & realism** — a footfall diagram, duty and stance-radius sliders,
  and a servo model (`sim/servo_model.py`): 50 Hz hold, 20 ms latency, 4.7 rad/s
  slew, 4096-count quantisation, off by default.
- **Hardware** — `sim/hw_bridge.py`: open the Feetech bus (or the mock), scan,
  legs present = legs whose three servos answered; sim → robot streams the
  sim's targets to those legs (25 Hz, counts/s cap), robot → sim makes the sim
  follow the real leg; telemetry table with jog, torque/LIMP, SafetyMonitor
  events, center + dir calibration into `bench/calibration.yaml`, set-ID.
- **`rocky.sh tailnet [PORT]`** — `tailscale serve` HTTPS in front of the
  cockpit (mic needs a secure origin); `tailnet off`. Tailscale was logged out on
  the laptop, so this is written but not exercised end to end.

**Broken and fixed:** `rocky.sh` failed to parse since f830ed4 (five help lines
had lost their `#`); `./rocky.sh cockpit` from the last session's notes would
have printed a syntax error. The 9c0416f SIGTERM fix never took: uvicorn
replaces signal handlers set before `uvicorn.run()` (`Server.capture_signals`),
so stale instances kept surviving `cockpit-stop`; the exit is now the server's
own `handle_exit`, verified with a camera stream open.

**Verified:** 4 new tests (`sim/tests/test_d051.py`), the full fast suite, a
Playwright pass through every new panel on the mock bus with zero console errors.
**Not verified:** a real servo — none has been on this bus yet (B32 is the runbook).

## 2026-09-23 · Session 9d (laptop) — cockpit round two: feet that feel for the floor, recordings, worlds on disk, the walker, voice (D050)

Asked for: all six follow-ups, plus how to start/stop the cockpit.

- **Contact-seeking stance** (`Playground._probe`): a planted foot with no
  contact is lowered up to 30 mm at 120 mm/s; a void = a foot that ran out
  of probe (`CliffDetector.update(..., probed_out=)`). Goto results from
  the origin: cliff → `cliff` at 0.15 m; flat 0.8 m, rough terrain 1.5 m,
  rubble 1.2 m, stairs (4 × 15 mm) 1.0 m → `arrived`; 6 cm box → `stuck`.
  Two wrong rules before it (foot-below-ground: never fires with position
  servos, walked off the cliff; foot-above-ground: still fired on bumps) —
  the cliff preset re-test after every change is what caught them.
- goto `stuck` (no 2 cm of progress in 6 s) and `timeout` (40 s) outcomes:
  the first obstacle goto had hung a tool call for 3 minutes.
- Recordings (`/api/record`, `/api/replay`): chase frames + every command
  (console, teleop, tool calls) → `sim/out/recordings/NAME/{clip.mp4,
  clip.gif, run.json}`; replay reloads the world and re-runs the commands.
  test1: 68 frames, 3 commands, replayed clean.
- Worlds on disk (`sim/worlds/*.json`, save/load) and `random_course(seed,
  n)`; the walker (`/api/rl/walk`: `robust_fwd2` residual on the analytic
  gait via `Playground.residual`, 0.16 m in 4 s); voice (`/api/voice`:
  browser blob → ffmpeg → whisper-server :8082 → chat); `/api/quit` + the
  page's ⏻ button, `rocky.sh cockpit-stop`, `cockpit` restarts a running
  one; a phone layout under 900 px.
- Bug: `set_world` compared the new name with itself → always respawned in
  place, so a preset switch from x = 1.2 m rebuilt the world around a robot
  standing inside a wall (that was the "cliff everywhere" I chased first).
- Tests 97 fast (+ terrain/random-course tests). Docs: SIM_GUIDE §3b, D050,
  B31, README.

## 2026-09-23 · Session 9c (laptop) — the cockpit: a browser playground on one running sim (D049)

Asked for: an in-app guide, brain/mode toggles, model switching, a chat
tab, collapsible panels, environmental conditions, obstacles, rough
terrain, and a way for the model to "see".

- `sim/cockpit.py` + `sim/cockpit_ui.html` (`./rocky.sh cockpit`,
  http://127.0.0.1:8765): one sim thread (Playground loop + the harness
  goto controller and cliff detector, real-time paced, speed 0.25–4×,
  pause), chase camera + torso eye camera as MJPEG, SSE state feed, a job
  queue so nothing else touches MjData. Panels: brain & chat (Talk / Local
  model / Claude, model picker, vision-model picker, tool trace), teleop
  (buttons + keys), gestures/chord-speak, shove buttons, orbit camera,
  world editor, RL, gait tuning, events, help; every panel collapses, the
  sidebar toggles.
- `sim/world_builder.py`: worlds from a JSON spec at runtime — presets
  (flat, room, cliff, obstacle course, rubble field, rough terrain,
  stairs, slope 8°, icy floor), objects (box, wall, ramp, stairs, rubble,
  pushable ball), floor friction, gravity tilt, a smoothed-noise
  heightfield; added geoms are lidar group 3; the robot gets an `eye`
  camera. The sim swaps models in place and respawns the robot upright.
- Vision: `look` = eye frame → local vision model (`vision-model` alias,
  lfm2.5-vl or gemma) → text. The local brain gets it as a tool; the MCP
  server exposes it where the backend has an eye.
- One sim for every driver: `harness/cockpit_backend.py` proxies the MCP
  contract to a running cockpit; `ROCKY_BACKEND=auto` (now in
  `.mcp.json.example` and my `.mcp.json`) picks it when it answers, else
  the in-process sim. So `./rocky.sh chat` drives what the browser shows.
- Fix after the first look: world objects are geom group 3 (the lidar
  convention) and MuJoCo renderers and viewers HIDE group 3 by default, so
  every preset built but drew nothing ("presets don't show anything").
  The cockpit renderers and both viewer windows now enable group 3. Also:
  a top-down map (objects, lidar hits, robot, goto target, click-to-goto),
  named camera views (follow/wide/top/low, default behind the robot), the
  HUD marker and goto flag drawn into the chase stream, the eye raised
  above the shoulder blocks, and respawn via mj_resetData (a plain qpos=0
  had dropped the pushable ball onto the origin with a zero quaternion).
- Verified: talk chat, local brain (qwen3.6-35b-a3b → say + look; the
  vision model read the obstacle course correctly enough), world swaps and
  edits, righter hot-swap (kept across reset/world change — a bug found
  and fixed), speed/pause, a 40 N shove with the full FALLEN → NORMAL
  cycle inside the cockpit, reset, the MCP proxy. 7 sim tests (world
  builder + shove). Docs: SIM_GUIDE §3b, README, RL_GUIDE, D049, B31.

## 2026-09-23 · Session 9b (laptop) — "the push flies and jitters": the shove model, two jitter sources, a righter retrain (D048)

**Report:** the push and the reflex correction in the playground / README
GIF "fly and jitter". Both true, neither a physics bug.

- **Flying = the push model.** Every push was a rectangular force pulse at
  the centre of mass. The demo's 120 N × 0.25 s was a 30 N·s strike
  (4.6 bodyweight-seconds): measured 9–11 m/s peak, 11–15 m of travel, with
  the camera tracking the torso. The model also has no gentle regime:
  below the ~31 N foot-friction limit the standing robot is a rigid block,
  above it the feet let go and it cartwheels (60 N: 2 m/s, 0.6 m, lands on
  its back). New `sim/shove.py`: half-sine over 0.4 s at the carapace's
  top rim (force + torque about the CoM), reported in N·s and BW.
  `shove_envelope.py`: standing survives 25 N peak, tips at 30 N (~1 BW,
  6–8 N·s); walking 20–30 N by direction. Demo shove 40 N (1.5 BW,
  10 N·s): fells 5/5 seeds, ends ~1 m away. 45 N felled only 3/5 — two
  rolled through and landed upright.
- **Jitter 1 = the righter.** `audit_righter.py` on `recover1`: 73–75 % of
  per-tick joint moves pinned at the 5 rad/s limit, ~10 direction
  reversals/s per joint, 4.5°/tick, 6.7° tracking error. A 50 Hz staircase
  of 5.7° jumps — the D045 bang-bang failure, deployed. And under the rim
  shove it never righted by policy (0/5): every stand in the demo came from
  the 10 s deadline ramp, which the old strike's random landings had hidden.
- **Jitter 2 = BRACE while airborne.** The tilt-vector seek flips its goal
  with the sign of ω×p every physics step; with zero contacts that is 58
  reversals in 0.4 s and legs thrashing mid-cartwheel. Now: no contacts →
  hold the foot targets (test added). The playground also never passed tilt
  to the supervisor, so after a flip it lay on its back cycling
  BRACE→RECOVER→NORMAL in a standing pose; it now feeds tilt/height and
  installs the righter (torch + checkpoint, optional), and the supervisor
  resets the righter on every FALLEN entry.
- **Ruled out:** servo stiffness (0.3° tracking standing, <7° walking),
  target smoothness on the ground (0 reversals in BRACE), loop pacing
  (5–9× real time headless). The viewer loop now resyncs instead of
  fast-forwarding after a >50 ms stall (unverified as a cause — the viewer
  probe hangs under Wayland from a script).
- **Righter retrain.** `RecoverEnv` reward `v3` = v2 + `-0.3·mean((Δtarget
  /rate_limit)²)`; `--rate-limit` per checkpoint (3 rad/s for v3), read back
  by `PolicyRighter`/`eval_recover` so a policy always replays its training
  dynamics. Two runs: `recover5_v3` (scratch, 3 M, 8 envs) and
  `recover5_v3_warm` (from recover1, +3 M, 4 envs), both `--log-std-max
  -0.5`, CPU, ~3.1k and ~1.7k sps. **Both negative on the handoff**: scratch 2/20, warm 4/20 vs recover1's 7/20 (re-measured today on the current model with the old and the new evaluator — the 10/20 in yesterday's table predates the D047 `pebble.xml` regen). Warm is the smoothest righter yet (58 % pinned, 6.5 reversals/s, 2.1°/tick vs 73 %, 9.8/s, 4.4°) — the trade exists, not yet won at this budget; recover1 stays shipped (B30 lists the next moves). What actually rights the robot from its BACK — where the rim shove lands it every time — is the planted-stance ramp, 5/5, not any policy (0/5 by handoff); so the supervisor now ramps after 3 s without a 10° improvement in tilt (stall rule, `right_reason` = handoff/stall/deadline, 2 tests) and the demo's righting takes ~5–6 s instead of the 10 s deadline. README GIF regenerated from `run_reflex_fallen.py --video` (seed 0: shove at 3.0 s, FALLEN 4.4, RIGHTED 9.2 by stall, walks 0.54 m).
- **Teleop moved to the terminal.** WASD in the viewer window "changed the
  lighting": the MuJoCo viewer binds every letter to a render toggle (W
  wireframe, S shadows, A auto-connect, D static bodies, G fog, Q camera,
  E equality) and SPACE to pause, and those fire alongside `key_callback`.
  Now: arrows / Shift+WASD / Shift+QE / SPACE / Shift+G as single keys at
  an empty `pebble>` prompt (raw-tty reader with line editing), arrows only
  in the window. Plus `help` / `help rl`, `rl` (checkpoint table from the
  new `sim/rl_dashboard.py`, which also draws `out/rl_curves.png`),
  `righter NAME|off` hot-swap, `set reflex.stall_s|fallen_max_s`, and a
  HUD in the viewer's user scene (state-coloured marker + command arrow).
- Drove it over MCP from this session: `gesture wave` (5.4 s, rendered in
  physics), `goto 0.3 0` → arrived at x = 0.274.
- Tests: 89 fast + 3 sim (`sim/tests/test_shove.py`, in CI after the MJCF
  build) + the airborne-hold reflex test. Docs: SIM_GUIDE (what a shove is,
  push command), RL_GUIDE (v3, the audit), D048, B29/B30.

## 2026-09-22 · Session 9 (laptop) — full review, then the leg is rebuilt around the real servo (D047); reprint plan + shopping list; repo goes public-ready

**Done**
- **Review** (`docs/REVIEW_2026-09-22.md`): three parallel passes over the
  D046 tree — mechanical, electronics/BOM, software/repo — with every
  finding dispositioned. Headline: `servo_st3215.py` v0.1 was a guessed box
  wrong in five independent ways, so every leg part the owner printed (photo in
  `media/`) was designed for a servo that does not exist; the D046 yaw
  stage would have failed structurally (~240 N through a 683ZZ 15 mm from
  the horn). Electronics: all-ST3215, ESP32 driver board is core, per-leg
  12 V fan-out, LiPo alarm. Software: 85 fast tests green, URDF parity
  had drifted since D046.
- **Servo model v2 from a measured STEP** (`cad/ref/STS3215_03a.step`,
  SO-ARM100): case 28.8 flat-to-flat with 1.5 mm rims carrying the screw
  holes, 2.6 mm plateau, axis 10.2 from the front, horn top 33.1, 45° hole
  pattern, Ø19.9 rear idler, connector housing + pins, plug keep-out. The
  model's self-check now diffs it against the STEP (43 mm³ unexplained).
- **D047 leg chain**: yaw servo shaft-down in a cup on the I1 base; one
  C-fork riding horn + idler; twin-plate femur (plate A on couplers, plate
  B on idlers, bridged, 4× M3); knee cup on the tibia; `servo_cup` = one
  retention shape for all three servos (four rim self-tappers); coupler
  re-cut (lobes 0/90/180/270, slotted M3/M2 holes); blank v0.3 + glue-on
  idler; four coupons that are clips of the production solids. Hip axis
  61 = `params.leg.hip_axis_z`, read by gait, MJCF and URDF (three hard-
  coded 58s gone). **CAD CI 23/23 tree clean**, joint suite clean with four
  new check classes, mass 2665 g, sim walk unchanged, URDF parity OK.
  (From the notes inbox: coxa 78 g (fork 14.1), femur 34 (link 16.1 +
  plate B 13.5), tibia 131, torso 1449; walk +X 264 mm, tilt max 1.0°;
  URDF parity FK 0.04 mm, mass 2.705 = 2.705 kg, the claw still counted
  twice until D052.)
- **Docs**: the print plan (batch 1 coupons → batch 2 one leg → batch 3
  body; now `docs/PRINT_PLAN.md`), a shopping list (bench kit ≈ $300–370,
  full robot adds ≈ $900–1,050; replaced by `bom/BOM.csv` on 2026-09-26), D047 row, B22–B28, print pack
  v0.5, viewer rebuilt.
- **Repo**: everything committed to git (drop/patch workflow retired),
  MIT LICENSE, `.mcp.json` untracked, duplicates and `NEXT_SESSION.md`
  removed, jpg/pdf on LFS.

**Broke / caught**
- Printability's first pass on D047 caught three real things: plate A's
  hub rim was 0.3 mm outside the coupler pocket (Ø26 → Ø30), the blank's
  rims-down pose had a 10.8 mm overhang (bottom filled flat), the idler
  puck's centre nub was a 6.6 mm overhang (omitted).
- OCC booleans return garbage on coincident faces: the STEP comparison had
  to run against a 0.02 mm-inflated model.
- The knee cup's two horn-face rim screws sit under plate A's beam: the
  suite now models them as driven before the femur goes on (B22).
- `generate_urdf.py` needed xacro (not installed here) and hard-coded
  session-2 masses; it now reads `mass_budget.json` and expands its own
  macro.

**Same evening — repo flattened, CI, guides, workflows verified**
- The code tree IS the repo root now (`cad/out` a real git-lfs directory,
  `rocky.sh` at the root defaulting to its own directory, `patches/` and the
  drop directories gone, `setup.py` → `pyproject.toml` with `sim`/`rl`/
  `harness`/`hw`/`cad`/`dev` extras). GitHub Actions `ci.yml`: fast suites
  + sim smoke + URDF parity + the slow harness test on every push, the CAD
  tree on pushes to main. Rehearsed in a fresh venv installed from
  pyproject alone: 85 passed, walk 264 mm, PARITY OK, slow test passed.
- `docs/SIM_GUIDE.md` and `docs/RL_GUIDE.md`: how each stack works,
  controls, step-by-step runs, the honest results table; README rewritten
  around a status table and two demo GIFs (`media/pebble_fallen_recover.gif`).
- Verified end to end: playground headless script (walk, strafe, stop,
  wave, live-tune, push — clean exit, no NaNs), `run_gestures2` 7/7,
  `run_patrol` 5/5 waypoints, `run_cliff_safestop` PASS; PPO smoke on both
  envs (2 × 96 × 10 updates, ~700–1100 sps on CPU), `eval_ppo --compare-zero`
  (`robust_fwd2`: 325 mm vs the bare gait's 366 — the D031 result stands),
  `eval_recover` on a smoke checkpoint.
- Fixed: `train_ppo.py --resume` with nothing left to train crashed with
  `UnboundLocalError: update` (session 8e); it now explains that
  `--total-steps` is cumulative and exits cleanly. New launcher commands
  `train-walk` / `eval-walk`.
- Retired `docs/SESSION_WORKFLOW.md` (cloud-drop process) and the zip
  instructions in the laptop setup page (since 2026-09-26 the README's Setup).

**Decisions**: D047.

**Next**
- Fit ladder numbers → `params.print` (B28) — still the gate for every fit.
- Order the bench kit (shopping list part A); one servo answers B23.
- Print batch 1 (four coupons + blank), file fits; then batch 2 (one leg).
- Repo pass 3: move the one-off `run_*_v2` / `diag_*` experiment scripts
  out of `sim/` into `experiments/`; a hardware backend for the harness.

## 2026-09-17 · Session 8f (laptop) — the dry-fit fails: a joint suite, then D046

*Filed 2026-09-26 from the notes inbox (09-17, 09-17 later, 09-18); the D046 parts
were superseded five days later by D047.*

**Field report:** the owner dry-fit the printed leg chain. Base → yaw blank → fork went
together "not great", the femur link attached to nothing, the blanks did not attach,
nothing past the first servo mount connected. **The cause was in the CAD, not the
printer:** the checks proved non-overlap and one solid per part; none asked whether a
joint can be assembled or what holds it. A new probe, `cad/check_assembly.py`, at
nominal params:
- **The coxa stage was an assembly deadlock.** All four yaw-horn M2 driver columns ran
  through the crown arm (45.4 mm³ of base in each; the in-module "driver access CLEAR"
  had tested only the fork). The other order, fork bolted to the servo first and slid in
  from +X, failed too: the Ø2.9 stub hit the arm for x 3..9 (up to 17.8 mm³). No order
  built it.
- Vertical budget at the coxa: horn top 38.0 = hub bottom (0 gap), floor top 45.65 vs
  the arm's underside 46.15 (0.5), collar vs arm top 0.85; fork +1 mm → 125 mm³ into the
  arm. Without the 683ZZ the stub sat in a Ø6.85 pocket: 3.95 mm radial slop at z 48.
- **Femur link:** each hub was 4 × M2 through Ø2.4 into a Ø1.7 pilot in the blank's
  3 mm horn disc, then 3 mm of air (BCD r 7 outside the Ø6 boss); pilot bore Ø6.4 vs the
  blank's Ø2.1 centre hole, so no radial location: a 2 mm nudge met zero material.
- Blanks: zip ties only in every cradle; "horn disc UP, no support" was a Ø20 disc on a
  Ø6 boss, 7 mm of 90° overhang that `check_printability` did not flag. The horn coupler
  (founding plan §3.4) was an orphan: 1709 mm³ of overlap if put between.

**D046 implemented, CAD CI 23/23 TREE CLEAN (106 s), printability 58 parts clean:**
- J1 yaw (`part_coxa` v0.3): the crown arm became a bolt-on `coxa_crown_cap`; an M3 × 10
  axle threading 5.65 mm into a fork boss replaced the stub; horn centre-screw pocket
  Ø6.6 × 3.5; cap/boss gap 0.4; collar 1 mm over the cap top.
- J2/J3 (`part_femur` v0.2 + `part_coupler`): the link carries the coupler's recess on
  both hubs. Coupler lobes moved to 45/135/225/315 (they sat 0.8 mm over their own M2
  heads), half-angle 28 → 22° so a Ø4.4 driver fits, blind M3 bores 4.8 mm deep. The
  recess clamp angles are mirrored (−45/−225): a real bug in the first cut, caught by
  the new coaxiality probe. Link plate at y −25..−19 (was −22.3..−16.3).
- Retention (`servo_mount.py`): full-height lips (4 mm reach) + a printed strap per
  cradle; 2 mm nudges now meet 300–1300 mm³ in every direction. Blank v0.2: Ø14 undercut
  fill, 4 × Ø2.4 through + M2 nut slots (4.3 × 1.9 at z 33..34.9). Four coupons
  (`part_leg_coupons.py`) clipped from the production solids.
- The joint suite joined `run_all_checks`: it failed the old tree with 4 findings.
- Hardware missing from the order sheets then: M2 × 8 (≥ 16) + nuts (16), M2 × 6 (16),
  M3 × 8 (≥ 12), M3 × 10 (5).

**09-18, same session:** a **cantilever rule** in `check_printability.py`: an overhang
blob whose boundary touches the layer below on < 60 % of its length is a cantilever;
≥ 3 mm with a "no support" plan fails. Regression: the v0.1 blank's horn disc 6.7 mm
FAIL, v0.2 2.7 pass; `tibia_sea_outer`'s 9.6 mm roof over the tube socket reads as a
bridge (2.8 cantilever), pass. It caught one more: `calib_gauge_hip`'s cradle shelf
overhung its shaft by 5 mm at 90° with no support, a drooping reference surface, now a
lofted pedestal (34–40° from vertical). Viewer rebuilt (30.8 MB), print pack
regenerated. Mass audit with the cap × 5 (PETG) and the straps: torso 1462.4 → 1477.3 g,
coxa 86.2 → 89.2, femur 15.7 → 15.1, tibia 135.0 → 138.5; robot 2649 → 2691 g (+1.6 %,
inside the ±30 % fill-factor band; no re-baseline).

**Deferred then (answered by D047):** a rear idler boss for two-sided brackets (B19),
case-screw retention (B20); the yaw joint deliberately had no coupler.

**Decisions:** D046. Backlog B19–B21.

## 2026-09-08 · Session 8e (laptop) — the dev laptop runs the loop: the harness in physics, a local model drives it, the capped retrain is negative

*Filed 2026-09-26 from the notes inbox (09-01, 09-08, 09-08 later).*

**Before it (09-01):** the sim environment stood up on the laptop (venv at the repo root,
uv, Python 3.12). `matplotlib` and `scipy` turned out to be hard imports and `mcp<2` a
requirement (the SDK 2.x renamed FastMCP): both now in the pyproject extras. torch
2.13.0+cu130 from plain PyPI saw the GPU. Verified that day: `run_sim` walks 265 mm, no
fall; driver 58/58; harness 21 fast + 1 slow; gait 6/6; gestures2 7/7 clean;
`eval_ppo` on `robust_fwd2` return 284.6; the PPO smoke passed.

**Done:**
- `harness/sim_backend.py`: `ROCKY_VIEWER=1` opens the passive MuJoCo window and paces
  goto and gestures to real time (no display → one stderr line, headless as before);
  `gesture()` now **runs** the gesture in physics (was log-only; all 10 render clean in
  the flat world, no falls; `stop()` cuts one short → `stopped: "user"`; a fall reports
  FELL like goto); `ROCKY_AUDIO=1` plays the chord samples. **Goto bug:** the target
  direction reached the gait in the MAP frame, but WaveGait takes BODY-frame
  velocities; invisible while gestures were log-only (yaw stayed ≈ 0). Now rotated by
  the current yaw: turn + sidestep → goto (0.25, 0) and (−0.10, 0.20) both arrive
  within the 25 mm ball. Harness 21 fast + 1 slow green.
- `harness/local_brain.py`: `--base-url` / `ROCKY_LLM_BASE_URL` adds an
  OpenAI-compatible client beside the Ollama path (`ROCKY_LLM_API_KEY`,
  `ROCKY_LLM_MODEL`, `--think`, thinking off by default). qwen3.6-35b-a3b through a
  local llama-swap drove the mock robot first try: say(greeting) →
  gesture(jazz_hands) → status → a one-sentence report.
- The chord samples rendered locally (`chordspeak2.py`: 16 words + a demo reel).
- The CAD extras installed on the laptop (build123d 0.11.1): `run_all_checks` 21/21
  TREE CLEAN in 103 s, so the caliper → params → regen → checks loop no longer needs a
  cloud session.
- `rocky.sh` born (play / chat / brain / talk / voice / test / jobs / train-recover /
  eval-recover / cad-check); `voice` = push-to-talk through a local whisper server
  (large-v3-turbo) into `harness.intent --stdin`, verified with a wav. A phone caliper
  worksheet exports NOTES_INBOX line blocks.
- Found: `train_ppo --resume` with nothing left to train died with
  `UnboundLocalError: update` (fixed in session 9); `recover1` was trained at batch
  1024 (8 envs × 128), not the documented 8 × 256.
- `eval_recover` on `recover1` here = **10/20 stood** (the cloud said 12/20 on the same
  seeds: MuJoCo 3.12 / CPU float drift); hold-pose 2/20, random 1/20. The laptop
  baseline for comparisons is 10/20.

**The D045 capped v2 retrain: NEGATIVE, both arms** (deterministic mean action,
hybrid handoff, 20 episodes, same machine):
- `recover3_capped` (warm from `recover1`, +2 M steps, `--reward v2 --log-std-max -0.5
  --rollout 128`, 4.0 M total, 19 min, ~1.8 k sps): **7/20 stood**, pure-RL 0/20, mean
  return 316.8, end-tilt median 5.2°; back 2/6, side 2/7, tumble 3/7. Final entropy
  **13.78 nats = exactly the cap ceiling** (15 dims × (½ ln 2πe − 0.5)): every dimension
  pinned at log σ = −0.5, so the policy still wants maximal noise and the cap merely
  held it there. Training ep_len fell to 252 (stochastic rollouts do end in success),
  return 543.
- `recover3_scratch` (same recipe from scratch, 3 M steps, 22 min, ~2.3 k sps):
  **3/20**, pure-RL 0/20, back 0/6 (end tilt median 113°: never rights from the back).
  Entropy 10.75, under the cap on its own.
- Reference: `recover2` (uncapped) ended at 22.13 nats and 2/20; `recover1` at 17.78
  nats, ep_len 300 (never a training-time success), 10/20.
- Reading (a hypothesis, not proven): under v2 the stochastic policy gets up but the
  mean does not; the success rides on the noise, so capping σ removes the bang-bang
  symptom without moving the mean toward a righting strategy. `recover1` stays the
  shipped righter. (Both checkpoints were removed on 2026-09-26; git keeps them.)

---

## Earlier sessions (2026-07-27 → 09-01) — archived

Sessions 1a–8d describe the design era before D047 (the leg rebuilt around the measured
servo). They are kept word for word in
[docs/archive/BUILD_LOG_2026-07-27_to_2026-09-01.md](docs/archive/BUILD_LOG_2026-07-27_to_2026-09-01.md),
with every measurement they recorded.

| Session | Date | What happened | Decisions |
|---|---|---|---|
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
