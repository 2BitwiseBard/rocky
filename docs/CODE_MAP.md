# Code map

Where things are, how the pieces feed each other, and the exact command for the common jobs.
Checked 2026-10-08 against the D064 tree (`build/body-layout`); the deeper guides are linked.

## 1. The one-page map

```
rocky/
├── rocky.sh              the launcher: ./rocky.sh help prints every command (there is no tools/ directory)
├── rocky.env.example, .mcp.json.example   copy to rocky.env (this machine's ROCKY_* knobs) / .mcp.json
├── pyproject.toml        pip extras (sim, rl, harness, cockpit, cad, hw, dev); constraints.txt pins MuJoCo
├── .github/workflows/ci.yml   CI: jobs fast, changes, cad (§2.13)
├── BUILD_LOG.md          the notebook, newest session first · NOTES_INBOX.md raw measurements, filed later
├── cad/                  build123d parts. params.yaml is the one source of numbers
│   ├── params.yaml       every dimension, servo number, gait default, fit, interface (§4)
│   ├── common.py         params() loader + export() (write-if-changed STL/STEP into cad/out/)
│   ├── iface.py          the six interfaces' shared geometry + the under-deck keep-outs
│   ├── leg_frame.py, servo_st3215.py, servo_mount.py   leg-chain frames, the servo solid (from cad/ref/), the cup
│   ├── part_*.py         23 part modules: each builds, checks itself in __main__, exports (table below)
│   ├── leg_assembly.py, check_assembly.py, check_interference.py, check_dock.py   whole-leg and dock checks
│   ├── check_printability.py, check_sim_mirror.py   POST audits: print physics; the sim's CAD copies
│   ├── run_all_checks.py the CAD CI (§2.5) · fem_check.py (+ fem_tools.py, fem_to_freecad.py) the stress check
│   ├── pentapod_preview.py, print_estimate.py, gen_drawings.py (+ td_sheet.py), gen_print_pack.py,
│   │   make_viewer.py, gen_assembly_views.py, render.py   the derived outputs (§2.8)
│   ├── ref/              the measured ST3215 STEP (Apache-2.0) + rpi5_envelope.json (MIT)
│   ├── out/              STL/STEP/PNG (git-lfs), PRINT_PREP_PACK.pdf, drawings/, assembly/, fem/
│   └── pebble_viewer.html   the self-contained 3D viewer (git-lfs)
├── gait/                 pure numpy control; pip-installs as the pebble-core modules
│   ├── rocky_model.py    the ONE params loader for sim, gait, driver, harness, bench
│   ├── pebble_gait.py    IK/FK + WaveGait (the five-phase wave gait) + CommandSlew + ArmedGait
│   ├── pebble_feasibility.py   the judge every motion passes (limits, speed, CoM, self-contact)
│   ├── pebble_reflex.py  ReflexSupervisor: brace, FALLEN / RIGHTED, the righter's hip clamp
│   ├── pebble_watchdog.py   stuck detection + RetryPolicy (higher steps on rubble)
│   ├── pebble_gestures.py, pebble_gestures2.py (code) · pebble_keyframes.py, pebble_pose_solver.py (JSON, reach IK)
│   └── gaits/  gait presets (one file: legacy_d050.json) · gestures/  keyframe gestures · test_*.py
├── sim/                  MuJoCo
│   ├── mass_audit.py → mass_budget.json, build_mjcf.py → pebble.xml, model_fingerprint.py,
│   │   check_urdf_parity.py   the model pipeline
│   ├── playground.py     terminal REPL + viewer · cockpit.py, cockpit_brains.py, cockpit_ui.html   the browser cockpit
│   ├── rocky_env.py, rocky_recover_env.py, rl_common.py, train_ppo.py, eval_*.py, righter.py,
│   │   audit_righter.py, rl_dashboard.py   RL (§2.4) · runs/   checkpoints (git-lfs .pt)
│   ├── run_sim.py, shove_envelope.py, torque_audit.py, audit_gestures.py   tools that give published numbers
│   ├── hw_bridge.py      the cockpit's real-servo bridge · servo_model.py, sim_imu.py, sim_lidar.py, shove.py
│   ├── world_builder.py + worlds/, scenes.py, scene_memory.py, place_memory.py   worlds, memory, places
│   ├── brain_install.py, brain_bench.py, vision_bench.py, place_bench.py, envfile.py   brains and benches
│   └── experiments/ (one question per script; results/ = the record) · out/ (tool records) · tests/
├── harness/              capabilities.py (the tool registry → docs/TOOLS.md), server.py (MCP),
│                         backends (mock, sim, cockpit), local_brain.py (LLM), intent.py (regex brain)
├── driver/               rocky_driver: Feetech bus driver (protocol, bus, robot, stream) + byte-level mock
├── bench/                bench-day scripts, all runnable with --mock, + BENCH_RUNBOOK.md
├── perception/           legged-odometry EKF, ICP scan matching, cliff detector, foot contacts
├── ros2/                 ROS 2 Jazzy packages (scaffold, never built here); URDF generated from params
├── audio/                chord-speak voice + samples · media/  photos, GIFs
├── bom/                  BOM.csv (the one bill of materials), README.md, totals.py
└── docs/                 guides, decisions, backlog, interfaces, this map; archive/ = superseded records
```

The data flow, top to bottom. Nothing downstream is edited by hand; it is regenerated.

```
cad/params.yaml ── read by cad/common.params() and, without build123d, gait/rocky_model.params()
   │
cad/part_*.py  (build123d; iface.py for every shared interface feature)
   │
cad/run_all_checks.py   29 modules in parallel, then POST: check_printability, check_sim_mirror
   │   --fem      fem_check → cad/out/fem/FEM_REPORT.md
   │   --derived  pentapod_preview, print_estimate, gen_drawings, gen_print_pack, make_viewer,
   │              gen_assembly_views → PRINT_PREP_PACK.pdf, drawings/, pebble_viewer.html, assembly/
cad/out/*.stl, *.step
   │
sim/mass_audit.py   STL volume × PLA × print_estimate.FILL + params mass_hw, the torso pose table
   │                → sim/mass_budget.json (link masses, torso mass, CoM, inertia)
sim/build_mjcf.py   params (via rocky_model) + the budget → sim/pebble.xml (incl. the belly geoms)
ros2/rocky_description/generate_urdf.py → urdf/pebble.urdf(.xacro);  sim/check_urdf_parity.py
   │
sim/model_fingerprint.py   12-hex hash of the robot in pebble.xml; checkpoints and records carry it
   │
gait/   rocky_model (numbers) · pebble_gait.WaveGait (motion) · pebble_feasibility (gate)
        · pebble_reflex.ReflexSupervisor (supervision) · pebble_keyframes (gait/gestures/*.json)
   │
sim/playground.py · sim/cockpit.py · harness/ (MCP, brains) · sim/hw_bridge.py → driver/rocky_driver
```

The part modules (run any of them from `cad/`; each writes `cad/out/<name>.stl/.step`):

| file | makes |
|---|---|
| `part_coxa.py` | `coxa_yaw_base` (the I1 plate), `coxa_fork` |
| `part_femur.py` · `part_tibia.py` | `femur_link`, `femur_plate_b` · `tibia_knee_carrier`, `tibia_sea_outer`, `tibia_sea_slider` |
| `part_hand.py` · `part_footpad.py` | `hand_hub`, `hand_cam`, `hand_finger` · `foot_pad_tpu` |
| `part_coupler.py` · `part_servo_blank.py` | `horn_coupler` · `servo_blank`, `blank_idler` |
| `part_deck.py` | `body_deck` (v0.5: the D064 hole table, the leg ports, the seats) |
| `part_bay.py` | the keel tub: `bay_tub`, `bay_lid`, `bay_door` |
| `part_battery.py` | `battery_sled` + the I3 latch cartridge (`latch_housing`, `latch_rotor`) |
| `part_avionics.py` | `avionics_tray`, `tray_rail`, `tray_latch_boss` (I4) |
| `part_busboard.py` | `hub_shelf` (the module keeps its old name) |
| `part_stand.py` | the bench stand, now the tub cradle (`stand_base`, `stand_crown`, `stand_section`) |
| `part_shell.py` · `part_panel.py` | carapace `shell_sector`, `shell_cap` · I3 panel and I6 dovetail demo coupons |
| `part_port_coupon.py` · `part_fit_ladder.py` · `part_leg_coupons.py` | the I1 port coupons · `fit_ladder` · the four joint coupons |
| `part_bench_jig.py` · `part_dock.py` | `jig_base`, `jig_column` · charging dock `dock_base`, `dock_tower` |
| `part_tools.py` · `part_clips.py` · `part_smallwins.py` | I2 tools · cable clips · gauges and small bench parts |

## 2. How do I ...

### 2.1 Find the gait
- `gait/pebble_gait.py` `class WaveGait`: `foot_targets(t, vx, vy, wz)` and `joint_targets(...)`
  give the five feet and the 15 joints for a command (vx, vy in mm/s, body frame; wz in rad/s).
  `leg_ik` / `leg_fk` are the closed-form leg; `swing_profile` is the D063 soft-landing swing.
- Its defaults are `params.yaml` `gait:` (`body_height` 118, `stance_radius` 185, `cycle_time`
  2.0, `duty` 0.8, `step_height` 24), read through `rocky_model.gait_defaults()`.
- The speed envelope: `WaveGait.budget(vx, vy, wz)` scales any command into it, `vf_limit()`
  and `max_command()` report it. `python gait/pebble_gait.py` prints it (34.2 mm/s, 0.185 rad/s).
- `CommandSlew` eases a new command in (25 mm/s², 0.2 s lag; a stop is never eased).
  `ArmedGait(WaveGait)` walks on four legs with one held as an arm.
- `gait/gaits/` holds presets as JSON (`T`, `h`, `R0`, `duty`, `hstep`). The only file is
  `legacy_d050.json`, the pre-D052 gait, kept to show why it was retired.
- At run time `ReflexSupervisor` (`gait/pebble_reflex.py`) wraps the gait, and the
  playground and the cockpit drive the supervisor.

### 2.2 Change the walk, or make a new gait
1. Retune: edit `params.yaml` `gait:`. Try it live first in the cockpit (Make → Gait lab &
   realism: sliders, **check**, presets) or the playground (`gait NAME`, `gait save NAME`,
   `gait list`, `sim/playground.py` `gait_presets` / `save_gait_preset`).
2. A new gait type: subclass `WaveGait`. `ArmedGait` is the worked example: it overrides
   `foot_targets`, `joint_targets`, `vf_limit` and `max_command`.
3. Re-gate it: `python gait/pebble_gait.py`, then `python gait/pebble_feasibility.py`
   (`check_gait(g, cmd)` at the envelope, `check_ramp` for the slewed start), then
   `MUJOCO_GL=egl .venv/bin/python sim/audit_gestures.py` (the gait grid in physics), then
   `sim/run_sim.py` (exits 1 on a fall).
4. Tests that pin today's numbers and are meant to fail when you change them:
   `gait/test_feasibility.py::test_default_walk_passes_and_budget_leaves_it_alone` (envelope 30–40
   mm/s, 0.16–0.22 rad/s), `gait/test_gait_motion.py`, `gait/test_watchdog.py`,
   `sim/tests/test_rl_envs.py` (the 2.36 s ease-in), `harness/test_capabilities.py`. Update the
   number in the same commit; do not loosen a tolerance (DESIGN_CHANGE_GUIDE §0).
5. `docs/TOOLS.md` quotes the envelope: `python -m harness.capabilities --md --out docs/TOOLS.md`.

### 2.3 Author a gesture
- Code gestures: `gait/pebble_gestures.py` `CANON` (jazz_hands, fist_bump, beckon) and
  `gait/pebble_gestures2.py` `GESTURES2` (wave, bow, look_around, shake, sit, turn_in_place,
  sidestep). Each is a function of (gait, t).
- Data gestures: `gait/gestures/NAME.json`, played by `pebble_keyframes.KeyframeGesture`. The file
  format (keyframes with `t`, `body`, `yaw`, `dz`, `arm`, `reach`, `reach_world`, `claw`, `say`,
  `ease`) is in the module docstring; `point_there.json` is the example on disk.
- Make one in the cockpit: Make → Gesture studio. **snap frame**, **solve** (reach IK:
  `pebble_pose_solver.solve_reach`), **record pose stream** (teach by demonstration:
  `keyframes_from_stream`), **save** (`POST /api/gesture/save` → `save_keyframe_gesture`, which
  refuses an infeasible gesture unless **force** is ticked).
- The judge: `pebble_feasibility.check_spec`; its codes are in
  [COCKPIT_GUIDE.md](COCKPIT_GUIDE.md#the-feasibility-verdict). In physics:
  `MUJOCO_GL=egl .venv/bin/python sim/audit_gestures.py [NAME ...] [--no-physics]`.
  `gait/test_feasibility.py` fails if any JSON on disk is infeasible.

### 2.4 Understand the reinforcement learning
- Method: PPO, written out in `sim/train_ppo.py` (no framework; GAE, clipped surrogate,
  normalisers, exact resume). Two environments: `--env gait` (`sim/rocky_env.py` `PebbleEnv`, a
  ±0.25 rad residual on the wave gait) and `--env recover` (`sim/rocky_recover_env.py`
  `RecoverEnv`, absolute joint targets from a fallen pose). Full guide: [RL_GUIDE.md](RL_GUIDE.md).
- `RecoverEnv`: obs v2, 57 values the Pi can read (IMU, encoder q and its finite-difference qd,
  foot switches, tilt, the filtered action); action map clip → EMA 0.4 → the params joint
  range → 4.0 rad/s clamp → `ServoModel`. Since D064 legs 1–4's hip maps over [−51.05, 90]
  (`hip_floor()`), and drops that start a leg inside the belly are redrawn (`drop_clear_of_belly`).
- `sim/rl_common.py`: one `Sensors`, `RecoverObs`, `DomainRandomizer`, `ThermalProxy`,
  `checkpoint_contract`, `check_fingerprint`, `action_map_mismatch` (a resume must keep the map).
- The hand-off: `ReflexSupervisor` runs the learned righter (`sim/righter.py` `PolicyRighter`,
  `runs/recover1/latest.pt` by default) in FALLEN, clamps its hips to `reflex.righter_hip_min_deg`
  on `reflex.righter_clamp_legs`, stops it after the 3 s stall rule or the 10 s deadline, and
  ramps to the planted stance in RIGHTED. `handoff_ok()` is hardware-computable.
- Train: `./rocky.sh train-recover NAME [--hip-clamp params|none|DEG] [--belly-drops]`,
  `./rocky.sh jobs`; evaluate: `./rocky.sh eval-recover NAME --supervisor` (the system number),
  `cd sim && python audit_righter.py runs/NAME/latest.pt` (jitter), `python sim/rl_dashboard.py --table`.
  `--hip-clamp params` is the default for new runs; `none` is the uncut pre-D064 map.
- Time: `recover7_d063_curriculum`'s log ends at 1,960 steps/s (the run's average; CPU, 8 envs),
  so its 12 M steps took about 1.7 h. The trainer's default `--total-steps` is 3,000,000.
- Checkpoints: `sim/runs/NAME/latest.pt` + `train_log.jsonl`. Every one is legacy on the D064
  robot: `recover1`, `recover5_v3_warm`, `robust_fwd2`, `cmd_sample3` predate D052 (obs v1, no
  fingerprint); `recover6_d052` and `recover7_d063_curriculum` carry a fingerprint that no longer
  matches, which warns. Re-evaluate before trusting one.

### 2.5 Change a part
1. Edit `cad/part_x.py`; take numbers from `params.yaml` (§2.6, §2.7), not literals.
2. Run it alone: `cd cad && ../.venv/bin/python part_x.py`. Its `__main__` prints its checks and
   exports. Exit code non-zero = a failed check.
3. `./rocky.sh cad-check part_x [part_y]`: the subset runs without the POST audits.
4. `./rocky.sh cad-check`: all 29 modules, then `check_printability` (every printable in its print
   orientation) and `check_sim_mirror` (the sim's copies of CAD numbers). 31 must pass; the run
   took ~7 min on 2026-10-07 (`part_bay` is the long pole). Exit code = failing modules.
5. A leg part: add `--fem`. Then `--derived` for the pack, drawings and viewer (§2.8).
6. If the part has mass in the robot, run the sim chain (§2.10) and commit the regenerated
   `sim/mass_budget.json` and `sim/pebble.xml`: CI's cad job regenerates them and fails on a diff.
7. The fingerprint moves when the compiled robot changes (a mass, a geom, a joint, an actuator).
   Bump `meta.params_rev` to the change's D-number when a physics number changed.

### 2.6 Change a fit or a tolerance
- The keys: `params.yaml` `print:` — `clearance_fit` 0.30 (read by most parts through
  `iface.FIT`), `seat_fit` 0.0 (the I1 sockets, `iface.leg_port_plate_features`), `screw_m3_clear`
  3.4, `screw_m3_tap` 2.8, `heatset_m3_d` 4.6 / `heatset_m3_h` 5.7, `bed_mm`. `clearance_press`,
  `wall_min`, `screw_m2_clear` and `screw_m25_clear` are spec only (no script reads them).
- Measure your printer first: `part_fit_ladder.py` → `fit_ladder.stl` (PRINT_PLAN batch 0). Row A
  → `clearance_fit`, row B → `screw_m3_tap`, row C → `heatset_m3_d`, row D (5.8 / 6.2 / 6.6) →
  `interfaces.leg_port.hook_slot_w`, row E = pin, bearing and tube fits. Write the result in
  `NOTES_INBOX.md`, then params, then regenerate. Fix params, never a printed part's code.
- The seat fit is tested on the port coupons (`part_port_coupon.py`): `port_coupon_deck` with
  `port_coupon_plate_relief46` / `_relief42` / `_fit10` / `_fit20`. The smallest of 0 / 0.1 / 0.2
  that docks with no rock is `print.seat_fit`. The ladder has no seat row.
- `cad/check_dock.py` holds the dock to a record: `iface.HOOK_ENVELOPE` (where the L reaches under
  the deck) and `HOOK_ENVELOPE_PARAMS` (the params it was measured with). A change to any of
  those `leg_port` keys, `clearance_fit` or `seat_fit` makes `hook_envelope_stale()` raise. Then
  re-run `check_dock.py`, re-record both dicts together; if a stored dock path breaks, search a
  new one with `python3 check_dock.py --search 0.30` and paste it in.

### 2.7 Change an interface dimension
- Numbers: `params.yaml` `interfaces:` — `leg_port` (I1), `tool_socket` (I2), `panel_latch` (I3),
  `avionics_tray` (I4), `battery_sled` (I5), `hub_shelf`, `stand`, `dovetail` (I6). The contract
  and its rule (loads through geometry, never latches): [INTERFACES.md](INTERFACES.md).
- Geometry: `cad/iface.py`, so both mating parts are cut by one function. Who calls what:

| helper | parts |
|---|---|
| `leg_port_deck_features` · `leg_port_plate_features` | `part_deck`, `part_bench_jig`, `part_port_coupon` · `part_coxa`, `part_port_coupon` |
| `latch_insert_housing` · `latch_strike` | `part_battery`, `part_panel`, `part_avionics`, `part_deck` · `part_panel`, `part_bay`, `part_avionics`, `part_deck` |
| `dovetail_male` | `part_shell`, `part_panel`, `part_stand` |
| `body_keepouts` · `bay_tub_extent` | `part_deck`, `part_bay`, `part_busboard`, `part_stand`, `check_dock` · those + `part_dock`, `check_sim_mirror` |

- The sim restates some of these (it cannot import build123d): `rocky_model.bay_tub_extent_mm`,
  `belly_boxes`, `belly_posts` and the `mass_audit` pose table. `check_sim_mirror.py` fails when
  they drift from the parts. Then run the full tree, update INTERFACES.md and log a decision.

### 2.8 Redraw the CAD outputs
- `./rocky.sh cad-check --derived` (after a clean tree): preview, `cad/out/print_estimate.json`,
  `cad/out/drawings/` (FreeCAD TechDraw; skipped without FreeCAD), `cad/out/PRINT_PREP_PACK.pdf`,
  `cad/pebble_viewer.html`, `cad/out/assembly/`. With `--fem` too, FEM runs first so the pack
  prints its verdicts. An unchanged tree leaves git clean.
- `./rocky.sh cad-drawings [PART...] [--force]`: only the A4 sheets (redrawn when a STEP changed).
- `./rocky.sh cad-open [PART|FILE...]` opens STEP files in FreeCAD (default: the posed leg
  assembly); `cad-open --fem PART` opens a stress result after `cad-check --fem`.
- The FEM loads and allowables are `params.yaml` `fem:`; the report is `cad/out/fem/FEM_REPORT.md`.
  At HEAD `coxa_yaw_base` fails at SF 1.10 (case V); the other four leg parts pass.

### 2.9 Change the servo or a leg length
- Servo: `params.yaml` `actuators:` (one block per servo) and `leg.servo`; speed budgets in
  `joints.vel_rad_s`; `rocky_model.actuator()`, `stall_nm()`, `dc_speed()` read them. The CAD case
  is `servo_st3215.py` from `cad/ref/STS3215_03a.step`.
- Lengths: `leg.l1_coxa`, `l2_femur`, `l3_tibia`, `hip_axis_z`; the `robot:` block follows by
  YAML alias. MJCF, URDF, gait IK, spawn height and feasibility follow; parts of the CAD do not
  (no file owns L3, the tibia tube cut).
- `python gait/rocky_model.py` validates the description. The step-by-step, with what follows
  automatically and what is by hand: [DESIGN_CHANGE_GUIDE.md](DESIGN_CHANGE_GUIDE.md) §1–§2.

### 2.10 Check masses and the centre of mass
```bash
.venv/bin/python sim/mass_audit.py                          # tables + sim/mass_budget.json
MUJOCO_GL=egl .venv/bin/python sim/build_mjcf.py            # → sim/pebble.xml (torso <inertial>)
.venv/bin/python ros2/rocky_description/generate_urdf.py
MUJOCO_GL=egl .venv/bin/python sim/check_urdf_parity.py     # URDF = MJCF = analytic FK, masses, inertia
MUJOCO_GL=egl .venv/bin/python sim/model_fingerprint.py
```
- Printed parts: STL volume × PLA × `print_estimate.FILL` (±30 %). Bought parts:
  `params.yaml` `mass_hw` (every one VERIFY; the battery is 380 g by pick 6).
- The torso is posed item by item in `mass_audit.torso_items()`. At HEAD it is 1555.0 g at
  (−2.21, −4.63, −3.18) mm and the robot 2843.0 g. `pebble_feasibility` reads the torso mass
  and CoM from the budget. `--allow-missing` is for development only.

### 2.11 Run the sim and the cockpit

| command | what |
|---|---|
| `./rocky.sh cockpit` | browser cockpit at http://127.0.0.1:8765; `cockpit-stop` ends it; `tailnet` publishes it to the phone |
| `./rocky.sh play` | MuJoCo window + REPL + arrow-key teleop (`sim/playground.py`) |
| `./rocky.sh chat` | Claude Code drives the robot over MCP (`harness/server.py`, `ROCKY_BACKEND` in `.mcp.json`) |
| `./rocky.sh brain` · `talk` · `voice` | a local LLM (`harness.local_brain`) · the regex brain · push-to-talk through whisper |
| `./rocky.sh test` | the fast test ladder (§2.13) |

- Cockpit tabs: **Drive** (pad, More driving) · **Talk** (Brain & chat, Wake word) · **Make**
  (Gesture studio, Voice & sounds, Gait lab & realism) · **World** (World & map, Memory &
  awareness, RL) · **Robot** (Hardware, Model & fingerprint, Recordings). Guide:
  [COCKPIT_GUIDE.md](COCKPIT_GUIDE.md); internals: [SIM_GUIDE.md](SIM_GUIDE.md) §3b.
- The tools a brain or MCP client can call come from `harness/capabilities.py` `REGISTRY`;
  [TOOLS.md](TOOLS.md) is generated from it (CI checks it). A new tool: DESIGN_CHANGE_GUIDE §8.

### 2.12 Re-run a published number
From the repo root with `MUJOCO_GL=egl .venv/bin/python`. Experiments write the tracked record
(`sim/experiments/README.md`) unless `ROCKY_EXPERIMENTS_OUT=DIR` is set. In parentheses: the D064 re-run.

| number | command | record |
|---|---|---|
| walk, turn, tilt (189 mm, 41.7°, 0.42°) | `sim/run_sim.py` | printed; clip in `sim/out/run_sim/` |
| envelope (34.2 mm/s, 0.185 rad/s) | `gait/pebble_gait.py` | printed |
| standing shove floor (30 N, 1.08 BW) | `sim/shove_envelope.py [--quick]` | `sim/out/shove_envelope.json` |
| holding torques (self-right knee 1.46 N·m, 49.5 %) | `sim/torque_audit.py` | `sim/out/torque_audit.json` |
| gestures (19 rows, 0 FAIL) | `sim/audit_gestures.py --json sim/out/audit_gestures.json` | that file |
| righting, system (20/20; 11/20 without the righter) | `cd sim && …/python eval_recover.py runs/recover1/latest.pt --supervisor` | printed; RL_GUIDE §4 |
| righter jitter | `cd sim && …/python audit_righter.py runs/recover1/latest.pt` | printed |
| rubble (30 mm 3/4, 35 mm 1/4, watchdog 40 mm 3/4) | `sim/experiments/run_stuck.py` | `results/stuck_results.json` |
| cliff safe-stop (PASS, margin 228.9 mm) | `sim/experiments/run_cliff_safestop.py` | `results/cliff_safestop_results.json` |
| cliff control (INCONCLUSIVE since D064: it hangs on the tub) | `sim/experiments/run_cliff.py` | `results/cliff_results.json` |
| void-guard lip band | `.venv/bin/python -m pytest sim/tests/test_playground_guards.py` (strict xfail) | the test |
| masses, fingerprint | §2.10 | `sim/mass_budget.json` |

The 200-seed counts (righting 198/200, 104/200 without the righter) came from a scratch harness
over `eval_recover.run_supervisor` and `audit_righter.audit_episode` that is not in the repo.

### 2.13 Run the tests, and what CI runs
- `./rocky.sh test`: `driver/tests`, `harness -m "not slow"`, `gait`, `sim/tests -m "not slow"`, on
  the reference setup (rocky.env is not read). The 9q fix round (2026-10-08) ran all of
  `sim/tests`, slow included: 819 passed + 1 re-pinned, 7 skipped, 5 xfailed; driver + gait +
  harness fast: 305 passed.
- CI (`.github/workflows/ci.yml`; a push to `main`, a pull request, or by hand). **fast**: `bash -n
  rocky.sh`, `ruff check .`, the unit suites, MJCF + URDF regenerated and `git diff --exit-code`,
  `harness.capabilities --check docs/TOOLS.md`, `run_sim.py`, all of `sim/tests`, URDF parity, the
  harness slow tests. **changes**: on a PR, runs the cad job only if `cad/`, the mass / MJCF / URDF
  generators, `gait/rocky_model.py`, `gait/pebble_gait.py` or the pins changed. **cad**: pulls only
  `cad/ref/*.step` from LFS, `run_all_checks.py`, `cad/test_fem.py`, then mass_audit → build_mjcf →
  URDF → parity, and fails on any diff of the committed files.

### 2.14 The hardware path
- `driver/rocky_driver/`: `FeetechBus` (bus.py), `PebbleRobot` (robot.py: ID map, calibration,
  `soft_enable`), `SoftStream` (stream.py), the byte-level mock (mock.py). [driver/README.md](../driver/README.md).
- `bench/`: `bus_scan`, `register_dump`, `assign_ids`, `calibrate_centers`, `pose_check`,
  `apply_limits`, `thermal_soak`, `torque_step`; each takes `--port /dev/ttyACM0` or `--mock`. The
  order of the day: [bench/BENCH_RUNBOOK.md](../bench/BENCH_RUNBOOK.md).
- `sim/hw_bridge.py` `HardwareBridge` is the cockpit's Robot → Hardware panel (soft first move,
  LIMP, EEPROM limits). Mock-verified only: no real servo has been on the bus.
- `ros2/`: the packages and the generated URDF ([ros2/README.md](../ros2/README.md)).

### 2.15 Find a decision, and log a change
- [decisions.md](decisions.md): numbered decisions (D001…), one row each; a design change gets a
  D-number, and a physics change bumps `meta.params_rev` to it. [DESIGN_BACKLOG.md](DESIGN_BACKLOG.md):
  B-numbered ideas and follow-ups with verdicts.
- [BUILD_LOG.md](../BUILD_LOG.md): one entry per session, newest first. `NOTES_INBOX.md`: raw
  measurements until they are filed into params. [archive/](archive/README.md): superseded records.
- The owner's desk page (order ticks, the body-layout picks) is a private claude.ai page; its
  link is kept outside the repo.

## 3. Reading order for a newcomer
1. [README.md](../README.md): status, setup, the ground rules.
2. `cad/params.yaml`: the comments say where each number came from and what reads it.
3. `gait/pebble_gait.py`: the header (frames, joints) and the `WaveGait` docstring.
4. [DESIGN_CHANGE_GUIDE.md](DESIGN_CHANGE_GUIDE.md) §0: the loop every change goes through.
5. [SIM_GUIDE.md](SIM_GUIDE.md) §1: how the sim is built from the CAD.

## 4. Conventions
- **Frames.** Body: origin at the pentagon centre, +z up, leg 0 at 90° (north, +y), legs
  counter-clockwise every 72° (stations 90 + 72 i), +x east. Leg: the yaw axis at the origin, +x
  outboard, deck plane z 0 (`cad/leg_frame.py`); the 6 mm deck slab is drawn at z −10..−4
  (`iface.DECK_BOT_Z`; `body.deck_t` 4.0 is unread, B27). Joints per leg: q1 yaw, q2 hip
  (0 horizontal, + up), q3 knee (0 straight, − down).
- **Units.** params and CAD in mm, degrees and grams; gait commands in mm/s and rad/s, joint
  angles in code in rad; `pebble.xml` in metres and kilograms with its joint ranges written in
  degrees; torque in N·m; shoves in N, N·s and bodyweights.
- **params is the single source of truth.** Repeated numbers are YAML aliases (`&name` /
  `*name`), not copies. `pebble.xml`, the URDF, `mass_budget.json` and `docs/TOOLS.md` are
  generated; never edit them by hand.
- **VERIFY** marks a number to measure on hardware before trusting it; **ESTIMATE** marks a
  modelled number. A doc that cannot find a number says VERIFY.
- **The fingerprint** (`sim/model_fingerprint.py`) hashes the robot only: masses, inertias,
  geoms, friction, joints, actuators; not the world or the timestep. At HEAD it prints `robot
  965f4f70e5d1 | mujoco 3.12.0 | params 0.1/D064`. The D064 records were measured on
  `deae868522cc`, before the 9q shell relief took 0.2 g off the torso (`c3e82f13b671`) and the
  deck's north tray-tab holes went back to Ø2.8 (+0.02 g, B155; params.yaml `meta`): the same
  geometry, the physics identical.
- **Checks print a verdict, exit code = failures.** Each part's `__main__`, `run_all_checks`,
  `check_dock`, `check_sim_mirror`, `run_sim`.
- **Git is the record.** Binary and regenerated files (STL, STEP, PNG, PDF, PT, NPZ, MP4, GIF,
  WAV, and `cad/pebble_viewer.html`) are in git-lfs (`.gitattributes`); CI fetches only the
  servo STEP. `rocky.env` holds this machine's settings and is git-ignored.
