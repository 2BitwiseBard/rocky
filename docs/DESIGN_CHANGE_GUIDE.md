# Changing the robot's design — how to do it today

This guide covers changing Pebble's hardware design: a different servo, a
longer link, a new foot, another material, or more joints or legs. It says
what to edit, what to run, and which checks catch what you forgot. The
tools a brain can call are data too; adding one is §8.

It describes what works **today** (after D053, step 1 of
[`ROBOT_AS_DATA.md`](ROBOT_AS_DATA.md)). The end state is "edit `params.yaml`,
run one command, read the report". The in-between steps are
`ROBOT_AS_DATA.md` §4 (backlog B36). When a step lands, this guide gets
shorter.

Contents:
- §0 The loop every change goes through
- §1 A different servo
- §2 A longer tibia or other link lengths
- §3 A new foot
- §4 A different material or infill
- §5 A fourth joint or a sixth leg
- §6 The first real leg
- §7 Checklist
- §8 A new tool for the brains

---

## 0. The loop every change goes through

The numbers live in one file, `cad/params.yaml`. Every consumer reads them
through `gait/rocky_model.py`. Since D053 the file also has a `robot:` block
that describes the robot's shape. That block holds no numbers of its own:
every length in it is a YAML alias (`*l3_tibia`) of a key further up. It
links those numbers to the joint ranges (`joints.pos_deg`) and to the servo
ids (`bus.leg_ids`, `bus.hand_ids`).

```bash
# from the repo root
# 1. edit cad/params.yaml (and the CAD part file, if the change is geometry)

# 2. does the description still make sense? (prints warnings; exits 1 on an error)
.venv/bin/python gait/rocky_model.py

# 3. regenerate, in this order (a future `rocky.sh regen` will do this, B36 step 9)
./rocky.sh cad-check                                   # the 28 CAD checks; rewrites cad/out/*.stl
./rocky.sh cad-check --derived --fem                   # + the leg stress check (D061; needs gmsh + CalculiX), then preview, print estimate, drawings, print pack, viewer, assembly pictures (35/35): one pass, the pack gets the new FEM verdicts and sheets (B110)
.venv/bin/python sim/mass_audit.py                     # STL volumes + mass_hw -> sim/mass_budget.json
MUJOCO_GL=egl .venv/bin/python sim/build_mjcf.py       # -> sim/pebble.xml
.venv/bin/python ros2/rocky_description/generate_urdf.py   # -> urdf/pebble.urdf(.xacro)
MUJOCO_GL=egl .venv/bin/python sim/check_urdf_parity.py    # URDF == MJCF == analytic FK, masses, limits

# 4. what changed, and does it still work
git diff --stat sim/ ros2/                             # expect pebble.xml / mass_budget.json to move
MUJOCO_GL=egl .venv/bin/python sim/model_fingerprint.py    # the robot fingerprint (today 87215110e9c4)
./rocky.sh test                                        # the fast suite CI runs
.venv/bin/python gait/pebble_gait.py                   # the gait's speed envelope + 4 checked commands
.venv/bin/python gait/pebble_feasibility.py            # walk/strafe PASS; the raw 0.35 rad/s turn and arc
                                                       # FAIL by design (above the envelope): compare with HEAD's
MUJOCO_GL=egl .venv/bin/python sim/run_sim.py          # does it still walk (exits 1 on a fall)
```

What each tool refuses:

- **`rocky_model.validate()`**, which runs whenever anything first asks
  about topology. It raises `RobotSpecError` when:
  - the number of id rows does not match the leg count
  - a row does not match the chain length
  - an id is used twice
  - a joint has no range
  - a servo has no `actuators:` block
  - the IK solver does not fit the chain
  - the stations are unevenly spaced
  - `overrides` is not empty

  It warns when `gait.duty` is not `1 - 1/legs`, and when an actuator has no
  `cad:` key. The second warning is expected until B36 step 2.
- **`sim/tests/test_model_consistency.py`**: the committed `pebble.xml` and
  `pebble.urdf.xacro` must equal what the generators write. The compiled
  links must equal the spec. Masses, limits, forcerange and damping must
  agree with `params.yaml`.
- **CI** regenerates everything and fails on any `git diff`.

**Numbers CAD reads directly.** Besides the leg and body lengths, the CAD
checks read `print.bed_mm` (the build volume the bed-fit checks use:
250 × 210 × 210 mm for the reference printer; change it for yours) and
`interfaces.battery_sled.xt60_panel_mm` (the XT60E-M body, 16.2 × 8.6 ×
16.0 mm, that the battery sled's nose and the dock's carrier are cut for;
VERIFY with calipers). Both change printed geometry, so they go through
the whole loop.

**Tests that pin today's numbers are meant to fail.** For example,
`test_loader_numbers` asserts stall 2.94 N·m, and
`test_default_walk_passes_and_budget_leaves_it_alone` asserts that the walk
envelope stays between 30 and 40 mm/s (D063 moved it from 45.5 to 34.2).
When one of these fails, read it, decide whether the new number is right,
and update it in the same commit. Do not loosen the tolerance.

**Every design change is a decision.** Give it a D-number in
`docs/decisions.md`. If a physics number changed, bump `meta.params_rev` in
`params.yaml` to that D-number; a mass change counts (D059's torso change
took it from D052 to D059). `sim/model_fingerprint` prints `params_rev`,
and RL checkpoints record it.

**RL checkpoints go stale on almost every change.** The robot fingerprint
(`sim/model_fingerprint.py`) covers masses, damping, forcerange, joint
ranges, friction and geometry:
- A checkpoint trained on a different fingerprint loads with a **warning**
  (`rl_common.check_fingerprint`). `recover6_d052`, trained on
  `ceb63a1254c3`, warns today.
- The 4 older checkpoints (`recover1`, `recover5_v3_warm`, `robust_fwd2`,
  `cmd_sample3`) carry no fingerprint, so nothing warns about them
  (`check_fingerprint` returns `'unknown'`).

Re-evaluate the righter after any change (`./rocky.sh eval-recover recover1`,
then `--supervisor`) before you trust it. Refusing on a topology change
(B36 step 5) is not built yet.

---

## 1. A different servo

Example: the STS3250, which D015 sanctioned. It uses the same case and the
same protocol as the ST3215, gives 4.90 N·m at 12 V (`sim/torque_audit.py`),
and weighs 74.5 g (`params.yaml` `servo_st3215.mass_g` comment). The BOM
buys none (D058): the worst load, the self-righting knee push, is 1.40 N·m,
47.5 % of ST3215 stall (WARM) and 28.5 % of STS3250
(`sim/torque_audit.py` on the D063 masses, 2026-09-30, B106).

1. **Add an `actuators.<key>` block** next to `st3215` with every key
   `rocky_model.actuator()` reads:
   - `stall_nm_<volts>`
   - `no_load_rad_s`
   - `continuous_frac`
   - `thermal_trip_frac` and `thermal_trip_s` (the leg servo only; claws
     inherit them)
   - `counts_per_rev`, or `counts` plus `sweep_deg`
   - `latency_s`, `bus_hz`, `mass_g`

   Take the values from the datasheet and mark every one `VERIFY` until the
   bench measures it. Do not copy the ST3215's numbers.
2. **Point the leg at it**: `leg.servo: &leg_servo <key>`. The `robot:` block
   follows through the alias.
3. **Re-derive the speed budget in `joints.vel_rad_s`.** These values were
   derived by hand, and `params.yaml` explains each one:
   - `hard`: repoint the alias to the new servo's `no_load_rad_s` anchor.
     `test_params_aliases_are_one_number` fails if you forget.
   - `loaded` ≈ `rm.dc_speed(rm.continuous_nm())`, rounded down.
   - `free` ≈ 0.85 × no-load.

   Print the current values with
   `.venv/bin/python -c "import sys; sys.path.insert(0,'gait'); import rocky_model as rm; print(rm.dc_speed(rm.continuous_nm()), rm.no_load_rad_s())"`.
4. **Mass** follows automatically: `mass_audit.py` reads
   `actuators[leg.servo].mass_g`. `mass_hw.servo_st3215` is an alias of the
   ST3215's mass; repoint it or leave it, since nothing reads it for the
   leg mass.
5. **CAD.**
   - If the case is the same (STS3250), nothing changes.
   - If the case is different, the whole leg changes. Every cup, hub and
     coupler is built from `servo_st3215.spec()`, which reads
     `params.servo_st3215`, the case measured on the STEP file. You need a
     new geometry block, a new `servo_<key>.py`, and a pass through
     `cad/leg_frame.py`, `servo_mount.py` and every `part_*` that imports
     `servo_st3215`.
   - Nothing checks that CAD matches the servo yet: the `cad:` key and its
     validator warning are B36 step 2.
6. **Driver.**
   - A servo on the same bus protocol (Feetech STS / SMS family) needs
     nothing. `driver/rocky_driver/robot.py` and `sim/hw_bridge.py` treat
     every leg id as `Family.STS`, and `registers.COUNTS` / `SWEEP_DEG` are
     per family (4096 / 360°).
   - A servo on a different protocol, or with a different encoder, needs
     driver work, because the protocol is not data yet (B36 steps 2 and
     10). Check `rm.id_table()` for the counts and sweep the description
     now expects.
7. **Run the loop in §0.** The fingerprint rotates because forcerange and
   damping change.
8. **Retune the gait through the budget, not by hand.**
   - `python gait/pebble_gait.py` prints the new envelope.
   - `python gait/pebble_feasibility.py` judges walk, strafe, turn and arc.
   - `MUJOCO_GL=egl python sim/audit_gestures.py` judges every gesture and
     keyframe file.
   - `python sim/torque_audit.py` gives the holding-torque margins.
   - `MUJOCO_GL=egl python sim/shove_envelope.py --quick` gives the push
     envelope.
   - A stronger servo widens the torque margins. The speed envelope moves
     with the no-load speed.
   - The stuck-watchdog's retry ladder (`gait/pebble_watchdog.py`
     `RetryPolicy`) re-derives itself from the new budget (D060): each
     stage takes the shortest cycle time whose speed envelope is at least
     20 mm/s, and `gait/test_watchdog.py` fails if a stage no longer
     walks.
   - The RL envs' command scales are literals (`rocky_env.py:185, 203-208`).
     They need to come from the new envelope before a walker retrain.
9. **Record the change.** Add a D-number, bump `params_rev`, and
   re-evaluate the righter (§0).

---

## 2. A longer tibia or other link lengths

Edit `leg.l1_coxa`, `leg.l2_femur`, `leg.l3_tibia` or `leg.hip_axis_z` in
`params.yaml`. The `robot:` chain follows them through the aliases.

| Change | Follows automatically | Do by hand |
|---|---|---|
| **L1 / L2** | `cad/leg_frame.py` places the hip and knee servos from them. `run_all_checks` rebuilds and re-checks the leg. Then mass_audit, MJCF, URDF, gait IK and parity follow. | `part_bench_jig`'s knee groove sits at a literal x = 140 (= L1 + L2), and so does the `calib_gauge_knee` plumb line (B5). Change both. |
| **L3** | Gait IK, MJCF, URDF, spawn height (`rm.spawn_dz_mm`), feasibility. | **Nothing in CAD owns L3.** The tibia is a carbon tube between `part_tibia` and the SEA/hand, and no file computes the tube cut length (B36 step 8 adds the check). Work out the stack by hand: knee carrier socket, tube, SEA, hand knuckle + cone + tip sphere, pad crown. Also update `mass_hw.tibia_tube` (5 g for "~120 mm"). |
| **hip_axis_z** | Gait, MJCF, URDF. | It must equal the yaw-stack height CAD builds (`leg_frame.py` `Z_YAW_TOP` and the hub stack). Nothing asserts that yet. |

After a length change, the nominal stance may no longer fit the servos'
sweeps. `gait.body_height` (118) and `gait.stance_radius` (185) are the
knobs. `rm.spawn_dz_mm()` raises "nominal stance is out of reach" if the
stance is geometrically impossible. `gait/pebble_feasibility.py` and
`gait/pebble_gait.py` tell you whether the stance fits inside the joint
limits with the 2° guard. Longer legs give a longer stride, a lower
envelope where the coxa sweep binds, and more knee torque. Re-run
`sim/torque_audit.py`.

Lengths change the robot fingerprint and the `description_digest`, but not
the topology hash. Existing checkpoints still load, with a warning.
Re-evaluate them before trusting them.

---

## 3. A new foot

The sim foot is a sphere whose surface is the IK foot point. Its radius is
`leg.foot.tip_radius + leg.foot.pad_crown` (6.5 mm today). Friction comes
from `leg.foot.mu_slide` and `mu_torsion_m`.

1. Edit `leg.foot.*` in `params.yaml`. MJCF, URDF, spawn height and parity
   follow.
2. **Two CAD literals must be changed by hand.** They mirror params but do
   not read them: `part_hand.CONE_TIP_R` (3.5) and `part_footpad.CROWN`
   (3.0). B36 step 8 adds the
   check for them.
3. If the new foot changes the stack length from the knee axis to the
   contact point, that is an **L3 change** (§2).
4. **A plain foot instead of the hand** (B25): `mass_audit.py` puts the hand
   into the tibia budget (hand_hub, hand_cam, 3 fingers, claw servo). Edit
   that dict so the sim does not carry a hand that is not there. The claw
   joint stays in the model and in `actuator_order()` until the `robot:`
   block drops the `tool:`, and dropping it is a topology change (§5).
5. Friction changes the turn and push envelopes. Re-run
   `sim/shove_envelope.py --quick` and the walk/turn check (§0). Record the
   μ you measured (a tilt-board test) in the params comment.

---

## 4. A different material or infill

Today mass comes from **STL volume × density × fill factor**
(`sim/mass_audit.py`) plus the non-printed hardware in `params.mass_hw`:
- Density: `print_estimate.PLA` = 1.24 g/cm³, plus a local `PETG = 1.27` in
  `mass_audit.py`.
- Fill factor: per part, from `print_estimate.FILL`. Anything unlisted
  gets 0.6.

Materials in params (`materials:` / `part_material:` / `part_fill:`) are
B36 step 7. Until then:

1. **Change a part's material**: pass the density in `mass_audit.py`'s
   dicts, as `printed_g("femur_plate_b", PETG)` already does.

   Known gap: the print plan (`docs/PRINT_PLAN.md`) prints
   coxa_yaw_base, coxa_fork, horn_coupler, femur_link, tibia_knee_carrier
   and the SEA parts in **PETG**, but `mass_audit.py` weighs them as PLA.
   That is about 2.4 % light on those parts.
2. **Change infill**: edit that part's entry in `cad/print_estimate.py`
   `FILL`, which `mass_audit` imports (`PLATES` is only the print batches).
3. Run `sim/mass_audit.py`, then the rest of the §0 loop.
   `test_compiled_mass_equals_budget` passes by construction. Review the
   `sim/mass_budget.json` diff: it is the change.
4. Mass moves the CoM, and therefore the feasibility support margins (the
   tightest is `manip_adjacent`'s 17.4 mm), the shove thresholds and the RL
   dynamics. Re-run the audits (§1 step 8). Weigh the real part on the
   kitchen scale when you have it. That number beats any fill factor.

---

## 5. A fourth joint or a sixth leg

Adding a joint or a leg is a **topology change**. Every RL checkpoint's
observation and action sizes change, so none can be reused. Since D053 the
description can express both kinds of robot. Most of the code cannot run
them yet. What works, what does not, and the order to fix it:

### What works today (step 1)

Write the `robot:` block, `joints.pos_deg` and the bus map as in
`ROBOT_AS_DATA.md` §2.3 (ankle) or §2.4 (six legs). Then:

```bash
.venv/bin/python gait/rocky_model.py      # validates; prints legs, chain, IK, actuator count, hashes
```

`rm.actuator_order()`, `rm.id_table()` and `rm.topology_hash()` then give:

| Description | Actuators | RL action (leg joints) | Servo ids | Topology hash |
|---|---|---|---|---|
| **today** | 20 | 15 | 1-15 legs, 16-20 claws | `7f066d9bd8c0` |
| **6 legs** | 24 | 18 | 1-18 legs, 19-24 claws | `9bd4fe37c615` |
| **ankle, 5 legs** | 25 | 20 | 1-20 legs, 21-25 claws | `ba8da0f5586d` |

`gait/test_robot_spec.py` builds both fixtures. The validator refuses:
- an ankle with `solver: analytic`
- a 4-joint leg with no `redundancy` rule (it would jump between IK
  solutions)
- id rows that do not match the leg count or the chain length
- duplicate ids

RL observation sizes are not computed from the description yet. They are
literals in `sim/rl_common.py` and get generated in B36 step 10.

### What happens if you regenerate anyway (measured 2026-09-24, in scratch)

- **Six legs**:
  - `build_mjcf` **builds and compiles** a 6-leg `pebble.xml` (24
    actuators). Stations and the leg count come from the description.
  - `pebble_gait.N_LEGS` becomes 6, and `WaveGait` returns `(6, 3)`
    targets.
  - The mass is wrong: `mass_audit` counts 5 yaw servos and 5 coxa bases
    into the torso.
  - `rl_common.N_LEGS` is still a literal 5, so the RL envs slice
    `ctrl[:15]` and silently drive only 5 of 6 legs.
  - `test_model_consistency` has `N = 5`.
  - The mock's hand ids collide with leg ids (`make_pebble_mock`: hands at
    15 + i).
- **Ankle**:
  - `build_mjcf` **refuses**: "unknown transmission target 'ankle0'". The
    leg template still has 3 bodies (B36 step 2).
  - Worse, `pebble_gait` **silently ignores** the ankle. `leg_ik` is the
    closed-form 3-joint solver and still uses `leg.l3_tibia`.

### What is still hard-coded

The remaining migration is `ROBOT_AS_DATA.md` §4 (backlog B36), one commit
per step. Two rules matter before any topology change: step 4 (actuators
by name) must land first, because a 4th joint would silently scramble the
positional `ctrl[:15]` slices; and step 5 (refusing checkpoints on a
topology change) must land together with `--accept-robot`. The deck and
shell are pentagon CAD, so a 6th leg is a redesign.

A sixth leg also needs a gait decision. `duty` 0.8333 keeps one leg in
swing at a time; `0.5` is a tripod. And after any topology change, the
FALLEN path has no learned righter until one is trained. With no righter
installed, FALLEN holds its pose and relies on the D048 stall ramp to stand
up.

---

## 6. The first real leg

The leg is built to `params.yaml` as it stands. Before assembling:

- **Tibia stack.** Work out the tube cut length T so that knee axis to foot
  contact = `l3_tibia` (135). At rest (the slider at its lowest pose) the
  stack is T + 136.2 mm:
  - knee axis to the tube top: `Z_HIP` - (`part_tibia.BOSS_Z0` +
    `SOCKET_DEPTH`) = 16.7 (`BOSS_Z0` = `leg_frame.Z_CUP_FLOOR_TOP` -
    `BOSS_H`);
  - the tube, less the 11 mm it sits in `tibia_sea_outer`'s socket: T - 11;
  - outer top to the stub face, slider at its lowest pose (the striker on
    the 0.9 mm lip): 45.6;
  - stub face to the cone tip: the stub seats at hand z 0.6 and the tip
    sphere ends at `part_hand` `KNUCKLE_Z` + `CONE_LEN` + `CONE_TIP_R` =
    82.5, so 81.9 (not `HUB_H` + `CONE_LEN`, which is 12.5 mm short);
  - `part_footpad` `CROWN`: 3.0.

  With the hand the stack cannot reach 135: T would be -1.2 mm, and the
  shortest tube (29 mm) gives 165.2 (B95).

  No check does this yet (B36 step 8, planned for **before** the first
  assembly). Record the number you cut in `BUILD_LOG.md`.
- **Anything you measure goes into `params.yaml` with its `VERIFY` removed.**
  Examples: the servo's mass, the horn BCD, the real hip height. Then run
  the §0 loop. The sim then matches the robot you built, not the one you
  drew.
- Bench bring-up is `bench/BENCH_RUNBOOK.md` and backlog B32.

---

## 7. Checklist (every change)

- [ ] `cad/params.yaml` edited. A number used in two places is an alias
      (`&name` / `*name`), not a copy.
- [ ] `.venv/bin/python gait/rocky_model.py` shows no error. Every warning
      is read.
- [ ] CAD part literals that mirror params are changed by hand (§2, §3).
- [ ] `./rocky.sh cad-check` (`--derived` for the preview, print pack and
      viewer; `--fem` when a leg part's shape, a servo or the stance changed) → `sim/mass_audit.py` → `sim/build_mjcf.py` →
      `generate_urdf.py` → `sim/check_urdf_parity.py`
- [ ] `./rocky.sh test`. Pinned-number failures are updated deliberately,
      not loosened.
- [ ] Gait budget and audits re-run: `pebble_gait.py`,
      `pebble_feasibility.py`, `audit_gestures.py`, `torque_audit.py`,
      `shove_envelope.py --quick`. Envelope numbers are updated in the docs
      that quote them (README status, SIM_GUIDE).
- [ ] New robot fingerprint noted. The righter is re-evaluated
      (`eval-recover`, `--supervisor`). Retrain if it regressed (RL_GUIDE).
- [ ] `docs/decisions.md` D-number. `meta.params_rev` bumped if physics
      changed. A `BUILD_LOG.md` line if hardware changed.
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
   sim's `tool_<name>`, which also serves `/api/tool/<name>`). `registry_problems()`, run by the tests,
   fails while the registry and the executor disagree. For MCP, each backend
   that offers the tool needs the method `backend_method` names (default:
   the tool's name); a backend without it does not list the tool. Talk mode
   (the regex parser in `harness/intent.py`) reaches only the tools it parses.
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

The registry's walkthrough (the surfaces, the live MCP list, the CLI) is
[SIM_GUIDE.md](SIM_GUIDE.md) §5; the generated page is [TOOLS.md](TOOLS.md).
