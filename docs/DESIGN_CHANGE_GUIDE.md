# Changing the robot's design

`cad/params.yaml` is the one source of truth. The CAD parts read it through
`cad/common.py`, everything else (the MJCF and URDF generators, the gait, the
reflexes, the driver, the harness, the cockpit) through `gait/rocky_model.py`,
and nothing downstream is edited by hand. This guide says what to edit, what to run and which checks catch what
you forgot, for a body part or a fit, a servo, a link length, a foot, a
material, or another joint or leg (§0–§7); adding a tool a brain can call
(§8); the robot-as-data plan and the status of each step (§9); and what
survives a scale-up to full Rocky (§10).

Today: step 1 of the robot-as-data plan has shipped (D053); steps 2–10 are
backlog B36. `.venv/bin/python gait/rocky_model.py` prints the description
(5 legs, 20 actuators, topology hash `7f066d9bd8c0`), and
`sim/model_fingerprint.py` the robot fingerprint: `965f4f70e5d1` since D064
(the belly model `deae868522cc`, on which the sim records were re-run, plus
the 9q review's shell relief, 0.2 g lighter, and the deck's north tray-tab
holes back to Ø2.8, 0.02 g heavier: the same physics).

Contents:
- §0 The loop every change goes through (a body part or a fit is in it)
- §1 A different servo
- §2 A longer tibia or other link lengths
- §3 A new foot
- §4 A different material or infill
- §5 A fourth joint or a sixth leg
- §6 The first real leg
- §7 Checklist
- §8 A new tool for the brains
- §9 The robot as data: the steps and their status
- §10 Scaling up to full Rocky

---

## 0. The loop every change goes through

The numbers live in `cad/params.yaml`. Its `robot:` block (D053) describes
the robot's shape and holds no numbers of its own: every length in it is a
YAML alias (`*l3_tibia`) of a key further up, and it links those numbers to
the joint ranges (`joints.pos_deg`) and the servo ids (`bus.leg_ids`,
`bus.hand_ids`).

```bash
# from the repo root
# 1. edit cad/params.yaml (and the CAD part file, if the change is geometry)

# 2. does the description still make sense? (prints warnings; exits 1 on an error)
.venv/bin/python gait/rocky_model.py

# 3. regenerate, in this order (a future `rocky.sh regen` will do this, §9 step 9)
./rocky.sh cad-check                                   # 31 checks (29 modules + check_printability + check_sim_mirror); rewrites cad/out/*.stl
./rocky.sh cad-check --derived --fem                   # + the leg stress check (D061; gmsh + CalculiX), the preview, print estimate,
                                                       #   drawings, print pack, viewer and assembly pictures in one pass (B110)
.venv/bin/python sim/mass_audit.py                     # STL volumes + mass_hw -> sim/mass_budget.json (the torso's pose table)
MUJOCO_GL=egl .venv/bin/python sim/build_mjcf.py       # -> sim/pebble.xml
.venv/bin/python ros2/rocky_description/generate_urdf.py   # -> urdf/pebble.urdf(.xacro)
MUJOCO_GL=egl .venv/bin/python sim/check_urdf_parity.py    # URDF == MJCF == analytic FK, masses, limits

# 4. what changed, and does it still work
git diff --stat sim/ ros2/                             # expect pebble.xml / mass_budget.json to move
MUJOCO_GL=egl .venv/bin/python sim/model_fingerprint.py    # the robot fingerprint (today 965f4f70e5d1)
./rocky.sh test                                        # the fast suite CI runs
.venv/bin/python gait/pebble_gait.py                   # the gait's speed envelope + 4 checked commands
.venv/bin/python gait/pebble_feasibility.py            # walk/strafe PASS; the raw 0.35 rad/s turn and arc
                                                       # FAIL by design (above the envelope): compare with HEAD's
MUJOCO_GL=egl .venv/bin/python sim/run_sim.py          # does it still walk (exits 1 on a fall)
```

The FEM stage fails today on `coxa_yaw_base` (SF 1.10 in case V, the
servo-stall foot push, on the real I1 support); the other four leg parts
pass. That is an owner decision still to come (B139, after a measured
option round), not a broken tool.

What each tool refuses:

- **`rocky_model.validate()`**, which runs whenever anything first asks
  about topology, raises `RobotSpecError` when the id rows do not match the
  leg count or the chain length, an id is used twice, a joint has no range, a
  servo has no `actuators:` block, the IK solver does not fit the chain, the
  stations are unevenly spaced, or `overrides` is not empty. It warns when
  `gait.duty` is not `1 - 1/legs` and when an actuator has no `cad:` key
  (expected until §9 step 2).
- **`cad/check_sim_mirror.py`** (run_all_checks' POST stage, review 9q): the
  sim cannot import build123d, so `sim/mass_audit.py`'s pose table and
  `gait/rocky_model.py`'s belly restate CAD numbers. Each must equal the part
  module that owns it, and the exported `bay_tub` / `hub_shelf` may stand
  outside the sim's belly boxes only where `iface.BAY_TUB_OUTSIDE` lists
  (`hub_shelf`: at most 1.0 mm; 0.60 today, the plate's corner fillets).
- **`sim/tests/test_model_consistency.py`**: the committed `pebble.xml` and
  `pebble.urdf.xacro` must equal what the generators write; the compiled
  links must equal the spec; masses, limits, forcerange and damping must
  agree with `params.yaml`. `sim/tests/test_belly.py` pins the belly geoms
  and refuses a provisional budget.
- **CI** regenerates everything and fails on any `git diff`.

**A body part or a fit.** The body parts are designs, not parameters
(`part_deck.py`, `part_shell.py`, `part_bay.py`, `part_busboard.py`,
`part_avionics.py`): edit the part module and the `interfaces:` keys it reads.
The torso in the sim is `mass_audit`'s pose table (every torso part weighed
from its STL and posed in the body frame; the sum's mass, CoM and inertia go
on the torso's `<inertial>`, D064), so a new torso part needs a row there, and
check_sim_mirror fails when a pose copies a part constant that has moved. The
belly the sim collides with (`rocky_model.belly_boxes()` / `belly_posts()`:
the keel tub's box, the hub shelf's plate box and its three Ø14 posts) is
read from params and the part modules, and the same check holds it to the
exported parts. A fit is printed geometry: `print.clearance_fit` (0.30, mating
printed parts), `print.seat_fit` (the I1 seats, filed from the port coupons)
and the per-interface keys. Change it in params, run the loop, and prove it on
a coupon or the fit ladder before printing five (`docs/PRINT_PLAN.md`).

**Numbers CAD reads directly.** Besides the leg and body lengths, the CAD
checks read `print.bed_mm` (the build volume the bed-fit checks use:
250 × 210 × 210 mm for the reference printer; change it for yours) and
`interfaces.battery_sled.xt60_panel_mm` (the XT60E-M body, 16.2 × 8.6 ×
16.0 mm, that the sled's nose, the tub's carrier and the charging dock are
cut for; VERIFY with calipers). Both change printed geometry, so they go through the whole loop.

**Tests that pin today's numbers are meant to fail.** For example,
`test_loader_numbers` asserts stall 2.94 N·m, and
`test_default_walk_passes_and_budget_leaves_it_alone` asserts that the walk
envelope stays between 30 and 40 mm/s (D063 moved it from 45.5 to 34.2).
When one fails, read it, decide whether the new number is right, and update
it in the same commit. Do not loosen the tolerance.

**Every design change is a decision.** Give it a D-number in
`docs/decisions.md`. If a physics number changed, bump `meta.params_rev` in
`params.yaml` to that D-number; a mass change counts (D064's body layout took
it from D063 to D064). `sim/model_fingerprint` prints `params_rev`, and RL
checkpoints record it. A control-only block is not physics: D065's `level:` (the body
leveler) left `params_rev` at D064 and the fingerprint as it was.

**RL checkpoints go stale on almost every change.** The robot fingerprint
covers masses, damping, forcerange, joint ranges, friction and geometry. A
checkpoint trained on a different fingerprint loads with a **warning**
(`rl_common.check_fingerprint`): on `965f4f70e5d1` every checkpoint on disk
is legacy, `recover6_d052` (`ceb63a1254c3`) and `recover7_d063_curriculum`
(`87215110e9c4`) warn, and the four older ones (`recover1`,
`recover5_v3_warm`, `robust_fwd2`, `cmd_sample3`) carry no fingerprint, so
nothing warns about them. A change to the righter's hip clamp
(`reflex.righter_hip_min_deg`, `reflex.righter_clamp_legs`) also changes the
recover env's action map, and `train_ppo` refuses to resume across it
(RL_GUIDE §3). Re-evaluate the righter after any change
(`./rocky.sh eval-recover recover1`, then `--supervisor`) before you trust
it. Refusing on a topology change (§9 step 5) is not built yet.

---

## 1. A different servo

Example: the STS3250, which D015 sanctioned. Same case and protocol as the
ST3215, 4.90 N·m at 12 V (`sim/torque_audit.py`), 74.5 g (`params.yaml`
`servo_st3215.mass_g` comment). The BOM buys none (D058): the worst load, the
self-righting knee push, is 1.46 N·m, 49.5 % of ST3215 stall (WARM, 0.5
points under the 50 % HOT line) and 29.7 % of STS3250 (`sim/torque_audit.py`
on the D064 masses, robot 2843.0 g; 1.40 N·m / 47.5 % on D063's 2727.7 g).

1. **Add an `actuators.<key>` block** next to `st3215` with every key
   `rocky_model.actuator()` reads: `stall_nm_<volts>`, `no_load_rad_s`,
   `continuous_frac`, `thermal_trip_frac` and `thermal_trip_s` (the leg servo
   only; claws inherit them), `counts_per_rev` (or `counts` plus
   `sweep_deg`), `latency_s`, `bus_hz`, `mass_g`. Take the values from the
   datasheet and mark every one `VERIFY` until the bench measures it. Do not
   copy the ST3215's numbers.
2. **Point the leg at it**: `leg.servo: &leg_servo <key>`. The `robot:` block
   follows through the alias.
3. **Re-derive the speed budget in `joints.vel_rad_s`** (derived by hand;
   `params.yaml` explains each): `hard` = the new servo's `no_load_rad_s`
   anchor (`test_params_aliases_are_one_number` fails if you forget);
   `loaded` ≈ `rm.dc_speed(rm.continuous_nm())`, rounded down; `free` ≈
   0.85 × no-load. Print the current values with
   `.venv/bin/python -c "import sys; sys.path.insert(0,'gait'); import rocky_model as rm; print(rm.dc_speed(rm.continuous_nm()), rm.no_load_rad_s())"`.
4. **Mass** follows: `mass_audit.py` reads `actuators[leg.servo].mass_g`.
5. **CAD.** The same case (STS3250) changes nothing. A different case changes
   the whole leg: every cup, hub and coupler is built from
   `servo_st3215.spec()`, the case measured on the STEP, so it needs a new
   geometry block, a new `servo_<key>.py`, and a pass through
   `cad/leg_frame.py`, `servo_mount.py` and every `part_*` that imports
   `servo_st3215`. Nothing checks that CAD matches the servo yet (§9 step 2).
6. **Driver.** A servo on the Feetech STS / SMS protocol needs nothing:
   `driver/rocky_driver/robot.py` and `sim/hw_bridge.py` treat every leg id as
   `Family.STS`, and `registers.COUNTS` / `SWEEP_DEG` are per family (4096 /
   360°). Another protocol or encoder needs driver work, because the protocol
   is not data yet (§9 steps 2 and 10).
7. **Run the loop in §0.** The fingerprint rotates because forcerange and
   damping change.
8. **Retune the gait through the budget, not by hand.** `gait/pebble_gait.py`
   prints the new envelope; `gait/pebble_feasibility.py` judges walk, strafe,
   turn and arc; `sim/audit_gestures.py` every gesture and keyframe file;
   `sim/torque_audit.py` the holding-torque margins;
   `sim/shove_envelope.py --quick` the push envelope. The stuck-watchdog's
   retry ladder (`gait/pebble_watchdog.py` `RetryPolicy`) re-derives itself
   from the new budget (D060), and `gait/test_watchdog.py` fails if a stage no
   longer walks. The RL envs' command scales are literals
   (`rocky_env.py:185, 203-208`) and must come from the new envelope before a
   walker retrain.
9. **Record the change**: a D-number, `params_rev`, the righter re-evaluated.

---

## 2. A longer tibia or other link lengths

Edit `leg.l1_coxa`, `leg.l2_femur`, `leg.l3_tibia` or `leg.hip_axis_z`. The
`robot:` chain follows through the aliases.

| Change | Follows automatically | Do by hand |
|---|---|---|
| **L1 / L2** | `cad/leg_frame.py` places the hip and knee servos from them; `run_all_checks` rebuilds and re-checks the leg; mass_audit, MJCF, URDF, gait IK and parity follow | `part_bench_jig`'s knee groove sits at a literal x = 140 (= L1 + L2), and so does the `calib_gauge_knee` plumb line (B5). Change both. |
| **L3** | Gait IK, MJCF, URDF, spawn height (`rm.spawn_dz_mm`), feasibility | **Nothing in CAD owns L3.** The tibia is a carbon tube between `part_tibia` and the SEA/hand, and no file computes the tube cut length (§9 step 8). Work out the stack by hand (§6) and update `mass_hw.tibia_tube` (5 g for "~120 mm"). |
| **hip_axis_z** | Gait, MJCF, URDF | It must equal the yaw-stack height CAD builds (`leg_frame.py` `Z_YAW_TOP` and the hub stack). Nothing asserts that yet. |

After a length change the nominal stance may no longer fit the servos'
sweeps. `gait.body_height` (118) and `gait.stance_radius` (185) are the knobs;
`rm.spawn_dz_mm()` raises "nominal stance is out of reach" when the stance is
impossible, and `pebble_feasibility.py` / `pebble_gait.py` say whether it fits
inside the joint limits with the 2° guard. Longer legs give a longer stride,
a lower envelope where the coxa sweep binds, and more knee torque: re-run
`sim/torque_audit.py`. The belly matters too: the righter's hip clamp
(-51.05° on legs 1–4) assumes a 135 mm foot, and with the B95 hand on the
shortest tube the tub needs -24…-20° on legs 1 and 4 (B131). Lengths change
the fingerprint and the `description_digest`, not the topology hash; existing
checkpoints load with a warning.

---

## 3. A new foot

The sim foot is a sphere whose surface is the IK foot point; its radius is
`leg.foot.tip_radius + leg.foot.pad_crown` (6.5 mm today), its friction
`leg.foot.mu_slide` and `mu_torsion_m`.

1. Edit `leg.foot.*`. MJCF, URDF, spawn height and parity follow.
2. **Two CAD literals mirror params without reading them**:
   `part_hand.CONE_TIP_R` (3.5) and `part_footpad.CROWN` (3.0). Change both
   (§9 step 8 adds the check).
3. A foot that changes the stack from the knee axis to the contact point is
   an **L3 change** (§2).
4. **A plain foot instead of the hand** (B25): `mass_audit.py` puts the hand
   into the tibia budget (hand_hub, hand_cam, 3 fingers, claw servo); edit
   that dict so the sim does not carry a hand that is not there. The claw
   joint stays in the model until the `robot:` block drops the `tool:`, a
   topology change (§5).
5. Friction moves the turn and push envelopes: re-run
   `sim/shove_envelope.py --quick` and the walk/turn check, and record the
   measured μ (a tilt-board test) in the params comment.

---

## 4. A different material or infill

Mass comes from **STL volume × density × fill factor** (`sim/mass_audit.py`)
plus the non-printed hardware in `params.mass_hw`: PLA 1.24 g/cm³
(`print_estimate.PLA`), PETG 1.27 (`mass_audit.PETG`); fill per part from
`print_estimate.FILL`, anything unlisted 0.6. Materials in params
(`materials:` / `part_material:` / `part_fill:`) are §9 step 7. Until then:

1. **A part's material**: pass the density in `mass_audit.py`, as
   `printed_g("femur_plate_b", PETG)`, the shell sectors and cap, and
   `NEW_PART_RHO` (the bay parts in PETG, the hub shelf in PLA) already do.
   Known gap: `docs/PRINT_PLAN.md` prints coxa_yaw_base, coxa_fork,
   horn_coupler, femur_link, tibia_knee_carrier and the SEA parts in
   **PETG**, but `mass_audit.py` weighs them as PLA, about 2.4 % light.
2. **Infill**: edit that part's entry in `cad/print_estimate.py` `FILL`.
3. Run `sim/mass_audit.py`, then the rest of the §0 loop, and review the
   `sim/mass_budget.json` diff: it is the change.
4. Mass moves the CoM, and so the feasibility support margins (the tightest
   is `manip_adjacent`'s 19.4 mm), the shove thresholds and the RL dynamics.
   Re-run the audits (§1 step 8). A kitchen-scale weight of the real part
   beats any fill factor.

---

## 5. A fourth joint or a sixth leg

A **topology change**: every RL checkpoint's observation and action sizes
change, so none can be reused. Since D053 the description can express both;
most of the code cannot run them yet.

**What works today.** Write the `robot:` block, `joints.pos_deg` and the bus
map for the new shape, for example:

```yaml
# a 6th leg
robot: {legs: {count: 6, first_station_deg: 90}}   # 60° spacing: 90, 150, … 390
bus.leg_ids: 6 rows                                 # e.g. leg 5 = [16, 17, 18]; hand_ids 19-24
gait.duty: 0.8333                                   # = 1 - 1/6, or 0.5 for a tripod

# a 4th joint per leg (ankle)
leg: {l3_tibia: &l3_tibia 100.0, l4_tarsus: &l4_tarsus 35.0}
robot.leg.chain: [yaw, hip, knee, {name: ankle, axis: [0, -1, 0], offset_mm: [*l3_tibia, 0, 0], link: tarsus, servo: <key>}]
robot.leg.foot: {offset_mm: [*l4_tarsus, 0, 0]}
robot.leg.ik: {solver: numeric, branch: knee_down, redundancy: foot_pitch, foot_pitch_deg: -90}
joints.pos_deg.ankle: [-60, 60]                     # bus.leg_ids 5 rows x 4, hand ids move up
```

Then `.venv/bin/python gait/rocky_model.py` validates it and prints legs,
chain, IK, actuator count and hashes:

| Description | Actuators | RL action (leg joints) | Servo ids | Topology hash |
|---|---|---|---|---|
| **today** | 20 | 15 | 1-15 legs, 16-20 claws | `7f066d9bd8c0` |
| **6 legs** | 24 | 18 | 1-18 legs, 19-24 claws | `9bd4fe37c615` |
| **ankle, 5 legs** | 25 | 20 | 1-20 legs, 21-25 claws | `ba8da0f5586d` |

`gait/test_robot_spec.py` builds both fixtures. The validator refuses an
ankle with `solver: analytic`, a 4-joint leg with no `redundancy` rule (it
would jump between IK solutions), id rows that do not match, and duplicate
ids. RL observation sizes are still literals in `sim/rl_common.py`.

**What happens if you regenerate anyway** (measured 2026-09-24, in scratch):

- **Six legs:** `build_mjcf` builds and compiles a 6-leg `pebble.xml` (24
  actuators) and `WaveGait` returns `(6, 3)` targets, but `mass_audit` counts
  5 yaw servos and 5 coxa bases into the torso, `rl_common.N_LEGS` is a
  literal 5 (the RL envs silently drive 5 of 6 legs through `ctrl[:15]`),
  `test_model_consistency` has `N = 5`, and the mock's hand ids collide with
  leg ids.
- **Ankle:** `build_mjcf` refuses ("unknown transmission target 'ankle0'":
  the leg template still has 3 bodies), and `pebble_gait` silently ignores
  the ankle (the closed-form 3-joint `leg_ik` still uses `leg.l3_tibia`).

Two rules matter before any topology change: §9 step 4 (actuators by name)
must land first, because a 4th joint would silently scramble the positional
`ctrl[:15]` slices; and step 5 (refusing checkpoints on a topology change)
must land together with `--accept-robot`. The deck, shell and body layout are
pentagon CAD, so a 6th leg is a redesign. After any topology change the
FALLEN path has no learned righter until one is trained; FALLEN then holds
its pose and the D048 stall ramp stands the robot.

---

## 6. The first real leg

The leg is built to `params.yaml` as it stands. Before assembling:

- **Tibia stack.** Work out the tube cut length T so that knee axis to foot
  contact = `l3_tibia` (135). At rest (the slider at its lowest pose) the
  stack is T + 136.2 mm: knee axis to the tube top 16.7 (`Z_HIP` −
  (`part_tibia.BOSS_Z0` + `SOCKET_DEPTH`)); the tube less the 11 mm it sits in
  `tibia_sea_outer`'s socket, T − 11; outer top to the stub face with the
  striker on the 0.9 mm lip, 45.6; stub face to the cone tip, 81.9 (the stub
  seats at hand z 0.6 and the tip sphere ends at `KNUCKLE_Z` + `CONE_LEN` +
  `CONE_TIP_R` = 82.5); `part_footpad` `CROWN`, 3.0. With the hand the stack
  cannot reach 135: T would be −1.2 mm, and the shortest tube (29 mm) gives
  165.2 (B95). No check does this yet (§9 step 8, planned for **before** the
  first assembly). Record the number you cut in `BUILD_LOG.md`.
- **Anything you measure goes into `params.yaml` with its `VERIFY` removed**
  (the servo's mass, the horn BCD, the real hip height), then the §0 loop, so
  the sim matches the robot you built, not the one you drew.
- Bench bring-up is `bench/BENCH_RUNBOOK.md` and backlog B32.

---

## 7. Checklist (every change)

- [ ] `cad/params.yaml` edited. A number used in two places is an alias
      (`&name` / `*name`), not a copy.
- [ ] `.venv/bin/python gait/rocky_model.py` shows no error; every warning
      is read.
- [ ] CAD part literals that mirror params are changed by hand (§2, §3); a
      new torso part has its row in `mass_audit`'s pose table.
- [ ] `./rocky.sh cad-check` (`--derived` for the preview, print pack and
      viewer; `--fem` when a leg part's shape, a servo or the stance changed)
      → `sim/mass_audit.py` → `sim/build_mjcf.py` → `generate_urdf.py` →
      `sim/check_urdf_parity.py`.
- [ ] `./rocky.sh test`. Pinned-number failures are updated deliberately,
      not loosened.
- [ ] Gait budget and audits re-run: `pebble_gait.py`,
      `pebble_feasibility.py`, `audit_gestures.py`, `torque_audit.py`,
      `shove_envelope.py --quick`; the numbers updated in the docs that quote
      them (README status, SIM_GUIDE).
- [ ] New robot fingerprint noted. The righter is re-evaluated
      (`eval-recover`, `--supervisor`); retrain if it regressed (RL_GUIDE).
- [ ] `docs/decisions.md` D-number; `meta.params_rev` bumped if physics
      changed; a `BUILD_LOG.md` line if hardware changed.
- [ ] The regenerated `sim/pebble.xml`, `sim/mass_budget.json` and
      `ros2/.../urdf/*` are committed **with** the params change (CI fails
      otherwise).

---

## 8. A new tool for the brains

Every tool a brain can call is one `ToolSpec` in `harness/capabilities.py`
(`REGISTRY`, D056). The local brains' OpenAI schema, the cockpit's list
(Claude mode converts it), the MCP server's live list and `docs/TOOLS.md`
are generated from it; the cockpit's executor is checked against it, not
generated. To add a tool:

1. **The spec.** Append a `ToolSpec` to `REGISTRY`: `name`; `kind` (`voice`,
   `motion`, `query`, `memory`, `gesture_authoring` or `meta`);
   `description`, the text the local brains read; `parameters`, an OpenAI
   JSON schema, where a gesture or chord-word enum is written
   `{"$live": "gestures"}` / `{"$live": "lexicon"}` and filled from the live
   lists. `surfaces` says who is offered it: `("openai", "mcp")` by default,
   `("openai",)` for the local brains only, `("internal",)` for the executor
   only (like `turn`). An MCP tool needs `mcp_args` (plus `annotations`, and
   `mcp_description` when MCP should read a different text), only an MCP
   tool may have them, and the constructor refuses a mismatch.
   `gated=True` if the tool moves the robot (a spoken line then needs the
   wake word). `requires` names what a backend must have to offer it (`eye`,
   `memory`, `cockpit`, `places`).
2. **The handler.** In the cockpit, `Brains.tool` runs it: add the name to
   its dispatch in `sim/cockpit_brains.py` (`TOOL_NAMES`, or a tuple beside
   it as D057 did with `PLACE_TOOLS`; `dispatch_names()` reads both) and a
   route (`OWN_ROUTES`, a `mem_<name>` method for a memory tool, or the
   sim's `tool_<name>`, which also serves `/api/tool/<name>`).
   `registry_problems()`, run by the tests, fails while the registry and the
   executor disagree. For MCP, each backend that offers the tool needs the
   method `backend_method` names (default: the tool's name); a backend
   without it does not list the tool. Talk mode (the regex parser in
   `harness/intent.py`) reaches only the tools it parses.
3. **The page.** `.venv/bin/python -m harness.capabilities --md --out docs/TOOLS.md`;
   CI runs `--check docs/TOOLS.md` and fails on a stale page.
4. **The pins.** The generated lists and the name sets are pinned, so a new
   tool (or a changed text) fails `./rocky.sh test` until the pins move with
   it, in the same commit, as the record that the change was meant:
   `GOLDEN`, `GOLDEN_D057` and the name sets (`DISPATCH_TODAY`,
   `ADDED_D057`) in `harness/test_capabilities.py`, `_BUILD_TOOLS_BEFORE` in
   `harness/test_harness.py`, and `EXTRA_BEFORE`, `EXTRA_BEFORE_SHA` and
   `GATED_BEFORE` in `sim/tests/test_cockpit_brains.py`. A tool behind a new
   capability flag leaves every list built without that flag unchanged: that
   is how D057 added its four place tools (`places`) without touching a D056
   hash.

The registry's snapshot carries `envelope` from `WaveGait().max_command()`
(0.0342 m/s, 0.185 rad/s, step 24 mm since D063) and `robot` from
`rocky_model.robot()`. The goto and move texts take their distance from
`envelope.goto_reach_m` / `move_max_m` (the constant 1.5 m, not derived:
`move` refuses beyond it, `goto` only advises it; the cockpit refuses a goto
target beyond 3 m), and the caps are derived: `goto_cap_s` walks that reach
1.2 times over plus the 2.36 s ease-in (55 s), `goto_detour_cap_s` the same
with both detours' 0.45 m added (87 s, B116). A params change that moves the
envelope or the topology makes `docs/TOOLS.md` stale, and CI refuses it as
it refuses a stale `pebble.xml`. The surfaces, the live MCP list and the CLI:
[SIM_GUIDE.md](SIM_GUIDE.md) §5.

---

## 9. The robot as data: the steps and their status

**The goal.** Before D053 the robot's *shape* (five legs, a yaw / hip / knee
chain, stations at `90 + 72·i`, the actuator order `ctrl[:15]` /
`ctrl[15:20]` rely on) was literals in about ten files. The end state for
any change, a CAD rebuild, a servo or material swap, another joint, the first
real leg: edit `cad/params.yaml`, run `rocky.sh regen` (step 9, not built),
read the report (the new fingerprint and topology hash, and which checkpoints
still load, need re-evaluation or are refused), and follow any check that
fails. Nothing breaks silently. **CAD is the exception**: the deck, shell,
tub and stand are designs (the pentagon is built into `part_deck.py` and
`part_shell.py`), kept in sync by checks that measure them against the
description, not generated from it.

Every step is its own commit, keeps `test_model_consistency`,
`gait/test_feasibility.py` and `gait/test_robot_spec.py` green (with the
staleness tests), keeps CI's regenerate-then-`git diff --exit-code` job
green, and leaves `sim/pebble.xml` byte-identical unless it is an explicit
D-number.

**What step 1 shipped (D053).** The `robot:` block (§0) with YAML anchors on
the existing keys (`&l1_coxa`, `&hip_axis_z`, `&l2_femur`, `&l3_tibia`,
`&leg_servo`, `&claw_servo`, `&body_r`; no tool rewrites `params.yaml`, so
the anchors survive). In `gait/rocky_model.py` (pure Python + PyYAML, no
numpy, so the driver imports it): frozen `JointSpec` / `LegSpec` /
`RobotSpec`; `robot()` (lazy, cached, validated on first build, so a broken
`robot:` block breaks only importers that touch topology); `validate()` and
`robot_warnings()`; `n_legs()`, `stations_deg()` (unwrapped: leg 4 is 378°),
`actuator_order()` (leg-major, then tools: the `ctrl[:15]` contract as a
tested fact), `id_table()`, `topology_hash()` (12 hex over legs, stations,
joints, axes, tool and IK solver: it decides whether a checkpoint can load at
all) and `description_digest()`. `build_mjcf` and `generate_urdf` read
stations, leg count and actuator order from it; `pebble.xml` and the URDF
stayed byte-identical; `gait/test_robot_spec.py` (24 tests) pins it.

| # | Step | Status | Check that proves it |
|---|---|---|---|
| 1 | Spec in the loader; stations and actuator order in both generators | **done** (D053) | byte-identical outputs; `test_robot_spec.py`; `check_urdf_parity` n_lim 20 |
| 2 | **Generators consume the chain**: bodies, joints, axes, offsets and kp / kv / armature from the spec and `actuators.*` (with `protocol` and `cad:` keys); `rocky.ros2_control.xacro` generated | open | `pebble.xml` / `pebble.urdf.xacro` byte-identical; one reviewed diff of ros2_control, then a staleness test. Without it an ankle validates but never reaches MJCF, URDF or ROS |
| 3 | **Close the paths that fail open**: `pebble_feasibility`'s `FAIL_CODES` and `divmod(k, 3)` from the spec; `check_urdf_parity`'s `n_lim != 20` and joint list from it; the mock's ids from `id_table()`; the driver's broad `except` around the limits import → `ImportError`; `hw_bridge` refuses an id without a NaN guard or rate limit; the generators fail on a missing `mass_budget.json` | open (the generators now warn on a missing or provisional budget, review 9q; they do not fail) | existing suites unchanged; a synthetic ankle limit counts as a fail; unique mock ids at 6 legs; a malformed params raises; the bridge refuses an uncovered id |
| 4 | **Actuators by name everywhere**: `sim/model_index.py` `ModelIndex` replaces every positional slice (`git grep -n 'ctrl\[:15\]\|ctrl\[15:20\]\|nu >= 20'`: the playground, both RL envs, `rl_common`, `sim_lidar`, the audits, `cockpit.py`, `harness/sim_backend.py`); `servo_model` per joint | open; **before any topology change** | the compiled actuator names equal `rm.actuator_order()`; the RL, guard and reflex tests unchanged |
| 5 | **Checkpoint safety**: stamp `topology_hash`, joint names and per-joint `q_lo/q_hi`; unstamped = the 5×3 legacy hash; `check_topology()` refuses a mismatch; strict mode `ROCKY_STRICT_FP=1` (default for train resume) with re-admission by `eval_recover --accept-robot <fp>`; the fingerprint extended (solref / solimp / margin, mesh data, `counts_per_rad`, `description_digest()`) | open (D064 added a narrower guard: `rl_common.action_map_mismatch` refuses a resume across a changed hip floor) | a synthetic topology change refuses all 5 checkpoints in `sim/runs/` (the 4 unfingerprinted ones too); a fingerprint-only change warns, refuses under strict, re-admits after `--accept-robot`. The fingerprint rotates once, by a D-number |
| 6 | **`gait/leg_kin.py` `LegKinematics`** replaces the FK copies (`pebble_feasibility`'s CoM model, `pebble_reflex`'s reach, `pebble_keyframes`, the pose solver): the analytic fast path chosen by a structural predicate (3 joints, yaw about z, two parallel ±y pitches, y = 0 offsets, +x foot), else damped least squares with the analytic Jacobian, a warm start, the branch and redundancy rules | open | analytic path bitwise equal to `leg_ik` / `leg_fk` on a grid; `fk(ik(p)) == p`; continuous q along a straight foot path on a 4-DOF fixture; a timing test. Numeric IK took 1.4–2.2 ms per leg in the prototype (about half of a Pi's 50 Hz budget for 5 legs): not the default until measured on the robot computer |
| 7 | **Materials and the mass layout** (a D-number): `mass_audit` and `print_estimate` read `materials` / `part_material` / `part_fill`; one `sim/body_layout.py` owns the primitive leg mass layout (coxa box, the 0.7 / 0.3 tibia split, `M_PRONG`) copied in `build_mjcf.py`, `generate_urdf.py` and the feasibility CoM model | open; partly done in D064: the torso is the pose table's real mass, CoM and inertia (the 0.75 / 0.25 cylinders are only the fallback), the bay and shell parts are weighed in PETG; the leg PETG parts still read PLA (§4) | `check_urdf_parity` compares CoM per link; the budget diff reviewed; feasibility, shove envelope and audits re-run |
| 8 | **CAD sync checks** (`cad/check_params_parity.py`, below), in parallel with 3–7; **the tibia-stack item before the first leg is assembled** | open (`check_sim_mirror` and `check_dock`, D064, cover the body side) | in `run_all_checks`; `rocky.sh cad-check` green |
| 9 | **`rocky.sh regen`**: validate → mass_audit → build_mjcf → generate_urdf (+ ros2_control) → check_urdf_parity → test_model_consistency, then the fingerprint and topology hash before and after and every checkpoint as ok / re-eval / refused | open | `rocky.sh regen` on a clean tree gives an empty `git diff` |
| 10 | **Only when a topology change is made**: the driver / bridge per-joint table from `id_table()`; reflex, gestures and keyframes generalised from `(N, 3)` (`reshape(N_LEGS, 3)`, the `n_con >= 4` crouch rule, `ArmGait`'s `n == 4`); `duty` from `n_legs`; the RL obs builder from the topology (a new obs version); an `overrides` consumer; the deck and shell as an n-gon | open | per-change fixtures; the full suite; a fresh policy trains |

### CAD sync rules

| Change | Follows automatically | Needs a manual step or a new check |
|---|---|---|
| L1 / L2 | `leg_frame.py` HIP_TF / KNEE_TF → cad-check → mass_audit → MJCF / URDF → parity | — |
| L3 | chain, gait, MJCF, URDF | nothing in CAD owns L3: the tibia-stack check below |
| `hip_axis_z` | chain, gait, MJCF, URDF | assert it equals `leg_frame`'s `Z_YAW_TOP` |
| servo, same case | physics, speeds, mass, kp / kv (step 2), protocol | the fingerprint rotates; re-evaluate checkpoints |
| servo, new case | — | a CAD geometry module named by `actuators.<k>.cad`; `servo_st3215.spec()` raises instead of building ST3215 cups for it |
| material | mass_audit and print_estimate via `part_material` (step 7) | a check fails on a part with no material or fill entry |
| a torso part | `mass_audit`'s pose table, the torso `<inertial>`, the fingerprint | the part's row and pose; `check_sim_mirror` holds the copied constants and the belly to the parts |
| leg count | spec, generators, gait stations | deck, shell, tub and stand redesign |

`cad/check_params_parity.py` (step 8, not written): the **tibia stack**
(knee axis → sole, §6, compared with `l3_tibia` within ±1 mm) and the **tube
cut length** printed; **foot geometry** (`part_hand.CONE_TIP_R ==
leg.foot.tip_radius`, `part_footpad.CROWN == leg.foot.pad_crown`); the deck's
station radius and count against `robot.legs`; the literal 110 / 100 / 118 /
185 / 140 in `part_shell`, `part_dock`, `part_bench_jig` and `part_smallwins`
against params; the dock heights against `gait.body_height`; CAD sweep
ranges covering `joints.pos_deg`. Once one spec drives both generators,
MJCF-vs-URDF parity mostly proves the generators agree, so three witnesses
stay independent and never read the spec: the closed-form `leg_fk` /
`leg_ik`, the CAD-measured HIP_TF / KNEE_TF, and the committed `pebble.xml`
body positions (`test_spec_matches_compiled_links`).

### Checkpoints and the fingerprint

| Change | Robot fingerprint | Topology hash | Policies |
|---|---|---|---|
| L3 ±5 mm, servo, material, friction, masses | rotates | unchanged | **warn** by default; refuse under strict mode and for the righter on the safety path, re-admitted by `--accept-robot` (step 5) |
| the righter's hip floor or its legs | rotates (params) | unchanged | the recover action map changes: `train_ppo` refuses the resume (`--hip-clamp none` continues a pre-D064 run); evaluation replays each checkpoint's own map |
| ankle, 6th leg, actuator reorder | rotates | changes | **all refused** once step 5 lands: there is no warm start across new obs / action sizes (the 39 → 57 history, B34) |

A checkpoint with no stamp is the 5×3 pentapod, the only topology that has
existed, so the unfingerprinted ones (`recover1` among them) are refused
after a topology change instead of running silently. Refusing on a
fingerprint mismatch ships together with `--accept-robot`, or the first
geometry change under strict mode locks out the cockpit's righter. A gait
retune goes through the budget, not RL: `WaveGait.budget` and
`pebble_feasibility` re-derive the envelope; the literal command scales in
`rocky_env.py` and the handoff thresholds (`HANDOFF_*`, metres) should come
from it and from the stance height.

### Risks to keep in view

- **Analytic IK shape.** `leg_ik` is exact only for yaw about +z plus two
  parallel −y pitches, y = 0 offsets, a +x foot and the knee-down branch;
  the fast path must be chosen by the structural predicate.
- **Symmetric stations.** `WaveGait`'s phase offsets, `pebble_manip_adjacent`'s
  arm pair, the touchdown gate and the CAD 72° sectors all assume equal CCW
  spacing; non-uniform stations stay refused until they are audited.
- **Positional ctrl slices** scramble silently until step 4; the
  actuator-order test is load-bearing.
- **Safety path.** Until step 3 a spec error in the driver can fall back to
  3-joint literal limits; until step 10 the bridge knows only yaw / hip /
  knee.
- **Mass and material changes** move feasibility margins, shove thresholds
  and RL dynamics: an explicit decision with an audit plan, never a drive-by
  refactor.

---

## 10. Scaling up to full Rocky

If Pebble lands, full-scale Rocky gets real fabrication (CNC, SLS, sent-out
drawings). What carries over, and what is a deliberate subscale shortcut
(written 2026-08-07, revised since):

- **Scales; keep investing:** `params.yaml` plus code-CAD (change the scale,
  regenerate STEP with real tolerances; every part exports STEP, which fab
  houses quote from); the check suite (boolean interference, keep-outs,
  engagement proofs, FEM: at full scale a failed fit costs a machined part
  and two weeks, not 40 g of PLA); the six interfaces I1–I6 (the dimensions
  rescale, the contract carries: loads through seats, lips and shoulders,
  never through latches, D020); the fit-ladder method (at scale a machining
  coupon on the chosen process before the real order); the decision and
  build logs.
- **Does not scale:** thread-forming M3 into printed parts and servo rims
  (machined bosses and inserts at scale); PLA / PETG structural sections
  (the cube-square law: Rocky's femur is CNC 7075 or SLS nylon-carbon with
  FEA, the printed geometry is only a starting shape); printed springs,
  detents and latch cams (steel at scale); printed horn couplers (a machined
  hub).
- **Hygiene now:** record a caliper value as the MEASURED number plus its
  FUNCTION (slide, press, tap) in the params comment, since a fab house needs
  the fit class, not our printer's compensation; keep VERIFY flags honest
  (each open one is a question a machinist bills to answer); STEP is the
  exchange format, STL is for our printer. The full-scale gate (the founding
  plan's Phase 5, [archive/ROCKY_MASTER_PLAN_v1.1.md](archive/ROCKY_MASTER_PLAN_v1.1.md))
  re-derives every tolerance: FDM clearances (0.30 fit / 0.15 press) do not
  transfer to CNC (ISO 286 fits) or SLS (process shrink, 1.0–1.5 mm minimum
  wall), and that pass regenerates from `params.yaml`.
- **The onboard brain at scale:** one NPU-class board (Orin Nano, a Pi AI
  HAT+, Hailo-8) carries an onboard vision model behind the same interface
  as the cockpit's server split, so the software moves unchanged; decide
  when the scale-up starts.
