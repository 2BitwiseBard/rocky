# sim/experiments — one-off experiments and their record

Each script here asks the simulator one question, prints a verdict and writes
its result. They are the evidence behind the decisions in
[docs/decisions.md](../../docs/decisions.md); they are not part of the robot
software (that lives in `sim/`, `gait/`, `perception/` and `harness/`).

Run any of them from the repo root:

```bash
MUJOCO_GL=egl .venv/bin/python sim/experiments/run_cliff.py
```

- Result JSONs and figures go to `sim/experiments/results/`, videos and
  temporary mux files to `sim/experiments/out/` (git-ignored).
  `ROCKY_EXPERIMENTS_OUT=DIR` sends both to `DIR` instead, for a scratch run
  that leaves the record alone.
- `exp_paths.py` holds those paths and puts `sim/`, `gait/`, `perception/`
  and `audio/` on `sys.path`; every script imports it first.
- The helpers live code shares with these scripts (the cliff world, the
  rubble field, the push harness's spawn, the IMU noise model, the 1.5 N
  contact switch, the fallen demo's shove) are in `sim/scenes.py`.
- `results/pre_d052/` is the record as it was committed on 2026-09-22,
  before D052 changed the robot model. See its README.

Experiments that stayed in `sim/` because live code, CI or the docs use
them as tools: `run_sim.py` (CI smoke), `shove_envelope.py`,
`torque_audit.py`, `mass_audit.py`, `audit_*.py`, `eval_*.py`,
`train_ppo.py`, `sim_lidar.py` (it writes the `sim/lidar_scans.npz` fixture
and the `sim/laserscan_spec.json` contract).

## The current record

**2026-09-26, robot fingerprint `7d376178fe27`** (`sim/model_fingerprint.py`;
mujoco 3.12.0), every script with its default options. D062 moved the
fingerprint to `1f953c89f979` (+7.6 g per femur) and D063 to `87215110e9c4`
(+5.5 g per coxa fork); D063 also gave the gait a soft-landing swing, which
lowers its envelope to 34.2 mm/s, and a command slew. The headline checks
were re-run on it (the SIM_GUIDE header says which). On 2026-09-30, on
`87215110e9c4`, B111 and B112 re-ran `run_stuck`, `run_fairing`,
`run_cliff_safestop`, `run_stuck_voiced` and `run_slam_lite` (on the lap
`sim/sim_lidar.py` re-recorded inside the envelope); their rows say so and
keep the 2026-09-26 numbers beside the pre-D052 ones. The other experiments
below were not re-run. To reproduce it, from the repo root:

```bash
export MUJOCO_GL=egl P=.venv/bin/python E=sim/experiments
$P sim/sim_lidar.py      # re-records the lap run_slam_lite reads (the sim/lidar_scans.npz fixture)
for s in run_terrain run_push run_push_reflex diag_brace_phase run_stuck run_fairing \
         run_cliff run_cliff_safestop run_odom run_odom_v2 run_slam_lite \
         run_manip_adjacent run_gestures2 run_reflex_fallen run_patrol; do
  $P $E/$s.py
done
$P $E/run_manip_adjacent.py --control
$P $E/run_push_reflex_v2.py && $P $E/run_push_reflex_v2.py --sustained
```

`run_stuck` has to run before `run_fairing`: the fairing's bare-shin
baseline is `results/stuck_results.json`. The same day `sim/out/` was
regenerated with the tools (`shove_envelope.py`, `torque_audit.py`,
`rl_dashboard.py`) and the committed clips with
`run_reflex_fallen.py --video sim/pebble_fallen_recover.mp4` and
`sim/run_sim.py --out sim`.

"vs pre-D052" compares with `results/pre_d052/` (2026-09-22, the D047
model before D052 fixed the spawn pose and added the servo model and the
peak-torque clip). Where a verdict a decision rests on moved, the row says
so; the decisions keep their dated numbers.

| script | what it measures | decision | superseded by | how to run (from the repo root) | current result (2026-09-26 unless the row says otherwise) | vs pre-D052 |
|---|---|---|---|---|---|---|
| `run_terrain.py` | how rough rubble (0–20 mm boxes, 3 seeds) the blind wave gait crosses | D017 | — | `... run_terrain.py [--video AMP]` (13 s) | envelope **20 mm**: 15/15 trials ok, 0 falls. Flat 323 of 346 mm commanded, tilt 0.8°; 20 mm 284–299 mm, tilt ≤ 4.0°. Beyond the script's range (scratch run of its `run_trial`, not recorded): 25 mm 3/3, 30 mm 2/3, 35 and 40 mm 0/3 (snagged at 117–187 mm), 0 falls | flat 348 mm, tilt 5.8° (the spawn-tilt artefact); 20 mm 301–336 mm. The extension (`pre_d052/terrain_results_ext.json`) was 3/3, 3/3, 2/3, 2/3 at 25–40 mm: **D017's "≤ 30 mm reliably" is now ≤ 25 mm**; snag-not-fall still holds |
| `run_push.py` | push envelope of the bare gait, 0.15 s CoM shove, 12 directions | D017 | `sim/shove_envelope.py` (D048). Frozen: its 27 N ladder saturates | `... run_push.py` (67 s) | 27 N (4.05 N·s) in every direction, standing and walking: the top of its ladder, so it measures nothing | same saturation. The wider-ladder record is `pre_d052/push_results_ext.json` (27–60 N ladder, session-8 masses: stand 40–46 N, walk 31–46 N), which no script writes and which does not match D017's 35–53 / 46–53 N. For today's envelope use `sim/shove_envelope.py` |
| `run_push_reflex.py` | the v1 crouch-and-brace reflex vs the bare gait; calibrates the 1.8 rad/s trip | D022 | `run_push_reflex_v2.py` (D025) | `... run_push_reflex.py [--quick]` (2.5 min) | stand: bare 57–90 N (mean 68.5) vs reflex 48–90 (67.8); walk: bare 27–90 (54.4) vs reflex 27–90 (53.7); 90 N is the ladder top. Phase probe at the weakest walking direction (150°): bare 27/33/27/27 N, reflex 33 N at all four phases. Clean-walk gyro peak 0.40 rad/s, trip 1.8 | stand 40–48 (46.7) → 40–48 (44.0); walk 33–48 (43.5) → 40–48 (44.0). **D022's walking floor 33 → 40 N no longer reproduces (27 → 27)**; the stance cost is −1 % (was −6 %) |
| `diag_brace_phase.py` | one failing +0.25T push traced leg by leg: why the v1 brace hurt | D025 | — (diagnosis) | `... diag_brace_phase.py` (2 s; prints only) | 40 N at 180°: at +0.25T the reflex **and** the bare gait fall (tilt 60°); at +0.00T both stay up but lose ground (−26 / −24 mm, fail the progress test) | pre-D052 the bare gait survived 48 N at +0.25T and v1 only 33 N (the pocket D025 diagnosed); on this model 40 N tips the bare gait there too, so the trace no longer isolates a v1 fault |
| `run_push_reflex_v2.py` | gait-phase-aware brace (v2) vs v1 vs base: envelope, phase probe, sustained lean | D025 | shove model: `sim/shove_envelope.py` (D048) | `... run_push_reflex_v2.py --envelope \| --probe \| --sustained \| --merge` (no flag: envelope + probe + merge, 5 min; envelope 3.4 min, probe 1.5 min, sustained 21 s) | walk envelope: base 27–90 N mean 54.4, v1 27–90 mean 53.2, v2 27–90 mean 55.4 (90 N = ladder top at 30° and 60°; 120–240° hold only 27–33 N). Phase probe at 180°: base 27–33 N, v1 and v2 33 N at every phase. Sustained 1.2 s lean: base = v2 in every direction (16 / 16 / 12 / 16 N at 0 / 90 / 180 / 270°, skid within 4 mm of each other) | means base 43.5 > v2 40.9 > v1 37.1, floor 33 N in all modes. **D025's order flips to v2 55.4 ≥ base 54.4 ≥ v1 53.2** (one ladder step, in two directions); v2 ≥ base at every probe phase still holds; sustained was 16 / 16 / 16 / 12 N, also no reflex benefit |
| `run_stuck.py` | 30–45 mm rubble crossings with and without the stuck watchdog | D023 | — | `... run_stuck.py [--video AMP SEED]` (3.7 min) | **2026-09-30, `87215110e9c4`** (B111: 34.2 mm/s for 32.9 s, the 1125 mm that 25 s at 45 mm/s commanded): crossings of 4 fields, bare → watchdog: 30 mm 2/4 → 4/4, 35 mm 2/4 → 4/4, 40 mm 1/4 → **2/4**, 45 mm 1/4 → 1/4; 0 falls, tilt ≤ 12.6°, watchdog crossings at 21.1–31.5 s; 10 of the 17 crossings land after 25 s, the old limit | 2026-09-26 (`7d376178fe27`, the D060 ladder, 45 mm/s for 25 s): 2/4 → 4/4, 0/4 → 4/4, 0/4 → 2/4, 1/4 → 2/4, tilt ≤ 11.9°, crossings at 11.7–23.7 s. Pre-D052: 4/4 → 4/4, 4/4 → 4/4, 1/4 → 4/4, 1/4 → 1/4. **D023's 40 mm 1/4 → 4/4 is 1/4 → 2/4 today**; the watchdog helps at 30–40 mm, not at 45 mm |
| `run_fairing.py` | a ramped TPU shin fairing on the same rubble (re-run `run_stuck.py` first: it is the bare-shin baseline) | D023 (rejected), B1 | — | `... run_fairing.py` (2.3 min) | **2026-09-30, `87215110e9c4`**, `run_stuck`'s limits (34.2 mm/s for 32.9 s). Faired, no watchdog: 30 mm 1/4 (bare 2/4), 35 mm 0/4 (2/4), 40 mm 0/4 (1/4), 45 mm 0/4 (1/4). Faired + watchdog: 40 mm 1/4 (bare + watchdog 2/4), 45 mm 0/4 (1/4). 0 falls, tilt ≤ 13.3°. The fairing hurts at every height, with and without the watchdog | 2026-09-26 (`7d376178fe27`, 25 s, the first run on the same fields as its baseline): faired 3/4, 1/4, 0/4, 0/4 (bare 2/4, 0/4, 0/4, 1/4); faired + watchdog 40 and 45 mm 1/4 (bare 2/4). Pre-D052 (different fields: the builder bug): faired 4/4, 3/4, 2/4, 2/4; faired + watchdog 40 and 45 mm 2/4. **The rejection stands; D023's "helps only at 45 mm" does not, nor 2026-09-26's small gain at 30–35 mm** |
| `run_cliff.py` | the VOID detector at a table edge vs walking off it | D034 | — | `... run_cliff.py [--video]` (2 s) | PASS: the control walks off (fell, overhang 4.4 mm); the detector stops **252.3 mm** short of the edge (body overhang −170.7 mm), first VOID at 4.14 s, 7 events | stop 186.8 mm, first VOID at 4.94 s, 11 events |
| `run_cliff_safestop.py` | the VOID stop through PLANT→BRACE→idle, judged on its own (B112: detector, upright, five feet on the deck, BRACE → NORMAL, no foot lifts after the halt, torso < 1 mm/s over the last second); the bare v = 0 halt as the reference | D034 | — | `... run_cliff_safestop.py [--video] [--retreat-cycles N]` (6 s) | **2026-09-30, `87215110e9c4`: PASS** (exit 0). `request_stop`: stop margin 228.8 mm, leading foot 46.2 mm short of the edge, 5/5 feet down, 0 breaks, 0.0 mm/s over the last second, max tilt 0.48°, BRACE → RECOVER → NORMAL in 0.65 s. The bare halt passes the same checks (213.5 mm, foot 34.5 mm, 0 breaks, 0.41°): since D063 a zero command stands, so it no longer marches. Negative control `--retreat-cycles 0` FAILS (exit 1): a foot 11.5 mm over the edge, 4/5 feet down, 7 breaks, 1.34 mm/s | 2026-09-26 (`7d376178fe27`, the old verdict "fewer breaks than the bare halt"): v = 0 halt 252.3 mm margin, 20 post-halt contact breaks, max tilt 0.77°; `request_stop` 250.3 mm, **0 breaks**, 0.59°. On D063 that verdict could not pass (0 breaks against 0). Pre-D052: 187.5 mm / 24 breaks / 0.68° vs 184.8 mm / 0 / 0.49°. D034 holds: the safe-stop is its stop path; the march it replaced is gone from the gait itself |
| `run_odom.py` | legged-odometry EKF drift vs ground truth: flat, turns, rubble | D026 (baseline) | `run_odom_v2.py` | `... run_odom.py` (9 s) | EKF end drift: flat straight 38.1 mm (4.49 % of 0.85 m; naive 55.7 mm), flat turny 40.1 mm (5.02 %; naive 331.7), 20 mm rubble 77.6 mm (8.06 % of 0.96 m; naive 332.2); yaw error 5.8–6.8° (yaw unobservable without v2) | 6.30 / 6.76 / 1.02 %: flat better, **rubble worse (12.2 → 77.6 mm)** |
| `run_odom_v2.py` | the zero-yaw-rate standing update (StillnessGate) and the mag stub | D026 | — | `... run_odom_v2.py` (27 s) | end yaw v1 → StillnessGate: flat 6.28 → 0.05°, turny 6.14 → 0.10°, patrol with pauses 14.37 → 1.63° (+ mag 0.79°); patrol drift 10.65 → 3.51 %; bias estimate 0.00481–0.00482 rad/s vs 0.005 true | 6.75 → 0.25°, 6.61 → 0.28°, 15.11 → 1.70°; patrol drift 11.63 → 5.54 %. D026 holds |
| `run_slam_lite.py` | EKF-seeded point-to-line ICP over the recorded lidar lap: ATE, map IoU | D027 | — | `.venv/bin/python sim/experiments/run_slam_lite.py` (reads `sim/lidar_scans.npz`, which `sim/sim_lidar.py` records in 14 s; 2 s) | **2026-09-30, the lap re-recorded on `87215110e9c4` inside the envelope** (B111: 26.6 mm/s + 0.048 rad/s on the 0.55 m ring for 66.1 s, 528 scans, 170° of the ring): odometry → SLAM: ATE RMSE 59.2 → 9.2 mm, end error 96.2 → 10.5 mm, yaw 8.61 → 0.13°, map IoU 0.187 → 0.692; 17 keyframes, median scan fit 2.8 mm | 2026-09-26 and pre-D052, identical (the 2026-07-31 lap, 310 scans at a raw 45 mm/s + 0.082 rad/s): 55.6 → 8.2 mm, 75.4 → 9.5 mm, 4.77 → 0.06°, 0.255 → 0.716. The slower, longer lap drifts more on odometry; ICP still holds it to ~1 cm |
| `run_manip_adjacent.py` | adjacent-arm manipulation by foot repositioning vs the no-repositioning control | D018 | — | `... run_manip_adjacent.py [--control] [--video]` (2 s each) | repositioned: stays up, tilt ≤ 5.38°, work-phase margin 11.9 mm, planted coxa ≤ 38.0°; `--control`: falls at 7.22 s (arms_up), tilt 25.0° | 5.11°, 9.9 mm, 39.8°; control fell at 7.22 s. D018 holds |
| `run_gestures2.py` | every gesture-library-v2 entry in physics: tilt, sag, planted-foot breaks, feasibility | D040 | — | `... run_gestures2.py [--metrics] [NAME ...]` (12 s with videos) | 7/7 clean, 0 falls, 0 s of planted-foot breaks: bow pitches 4.31°, sit sags 54.6 mm, look_around peaks at 16.2°, turn_in_place yaws **48.0°**, sidestep travels **140 mm**; all 7 feasible; tracking error mean ≤ 2.37°, max ≤ 18.6° | bow 4.51°, sit 53.7 mm, look 16.0°, turn 80.0°, sidestep 170 mm, shake 0.18 s of micro-lifts (now 0). **D040's turn 80° / sidestep 170 mm are 48° / 140 mm under the D052 speed envelope** |
| `run_reflex_fallen.py` | 40 N rim shove → fall → the recovery policy rights it → walks away, 5 seeds | D042, D045, D048 | — | `... run_reflex_fallen.py [--episodes N] [--video PATH]` (14 s with the video) | 40 N × 0.4 s half-sine at the rim (10.2 N·s, 1.53 BW): **fell 4/5**, all 4 righted and walked away (FALLEN 4.37–4.45 s, RIGHTED 9.19–9.32 s, walked 0.47 m, thrown 0.78–1.01 m); seed 4 braced and stayed up (thrown 1.34 m) | fell 5/5, 5/5 full cycles, RIGHTED 9.15–10.79 s, walked 0.46–0.54 m |
| `run_patrol.py` | a waypoint patrol driven only through the harness tools (goto, scan_summary, say) in the room world | D043 | — | `... run_patrol.py [--waypoints N]` (20 s) | COMPLETE: 5/5 waypoints `arrived` (18–25 mm from target), nearest obstacle 0.743 m, chords startup + 5 × acknowledge + discovery | 5/5, nearest 0.737 m. D043 holds |
| `run_goto_detour_cap.py` | the time cap a plain goto needs once it has entered a detour: 200 gotos of 1.0–2.0 m straight into a 0.25 m wall or 25 cm box (seeded placements 0.45 m out, half way or 0.6 m short of the target, and at 1.2 and 1.5 m 0.4–0.55 m short of it; bearings 0° and 36°), 8 with nothing in the way, 6 that cannot arrive; each runs uncapped in a headless CockpitSim and its outcome under any cap is derived | B116 | — | `... run_goto_detour_cap.py [--quick] [--verify [PLAIN DETOUR]]` (`--verify` alone re-runs every scenario on the cockpit's shipped caps; 40 min for 428 runs at 5 jobs on a busy machine) | **2026-09-30, `87215110e9c4`**: arrived at a 55 / 71 / 75 / 87 s cap: 1.0 m 15 / 16 / 16 / 16 of 16, 1.2 m 38 / 78 / 78 / 78 of 84, 1.5 m 0 / 77 / 80 / 80 of 84, 2.0 m 0 / 1 / 4 / 15 of 16 (the rest end `blocked` or `stuck` by themselves); the latest arrival up to 1.5 m is 75.0 s (the target 0.4 m behind a box), so `goto_cap_s` at 1.5 + 0.45 m (71 s) misses 3 and at 1.5 + 2 × 0.45 m (87 s) leaves 12.0 s; a detour costs 11.0–32.0 s up to 1.5 m; the clear and cannot-arrive gotos are unchanged. SHIPPED the same day (DESIGN_BACKLOG B116: 87 s once a goto has detoured, ending it even mid-detour): `--verify` on the shipped rule 214/214 as derived, 195 arrived, 2 timeouts (the clear 2.0 m gotos at 55.0 s), no detour goto reached 87 s, the latest answer 87.2 s (the first record, `--verify 55 71` on the pre-B116 rule, 128/128 with 17 timeouts, is in git history) | — (new) |
| `run_sim_manip.py` | demo: walk, strafe, turn, 4-leg walk with an arm raised, 3-leg two-arm work | — (demo clip) | — | `... run_sim_manip.py` → `out/pebble_rocky_demo.mp4` (8 s) | 30.7 s clip, no fall; 4-leg walk with an arm raised tilts ≤ 7.15°, the 3-leg two-arm stance holds 106.9 mm, 0.19° | — |
| `run_gestures.py` | demo: jazz hands + a contact-triggered fist bump, narrated | — (demo clip) | `run_gestures2.py` for metrics | `... run_gestures.py` → `out/pebble_gestures.mp4` (4 s) | renders | — |
| `run_beckon.py` | demo: the beckon gesture, narrated | D035 | — | `... run_beckon.py` → `out/pebble_beckon.mp4` (3 s) | 8.3 s clip | — |
| `run_stuck_voiced.py` | demo: the 40 mm stuck-retry crossing, narrated by the watchdog's own events | — (demo clip) | — | `... run_stuck_voiced.py [SEED]` → `out/pebble_stuck_retry_40mm_voiced.mp4` (17 s) | **2026-09-30, `87215110e9c4`** (`run_stuck`'s walk, 32.9 s): seed 1, the default, crosses at 32.5 s after 5 watchdog events; the clip runs 4 s past the crossing (35.8 s) and ends on found_it, amaze. Seed 0 does not cross (5 events, 33.2 s, no found_it), matching `run_stuck`'s 40 mm row | 2026-09-26: seed 1 crossed at 21.3 s after 4 events; seed 0 did not (25.5 s) |
| `run_showcase.py` | demo: locomotion medley, push + brace, cliff stop in one narrated reel | — (demo clip) | — | `... run_showcase.py` → `out/pebble_showcase_v07.mp4` (11 s) | 30.8 s reel | — |

`...` = `MUJOCO_GL=egl .venv/bin/python sim/experiments/`. Times are wall
clock on the development laptop. The narrated clips need `ffmpeg` on the
PATH.
