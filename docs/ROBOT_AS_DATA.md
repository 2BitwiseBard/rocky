# ROBOT_AS_DATA — the robot's shape lives in data

Status: design, 2026-09-24. Nothing is built yet. §7 is the step to build now.
Baseline: `pytest sim/tests/test_model_consistency.py gait/test_feasibility.py` gives 61 passed. The compiled `sim/pebble.xml` has robot fingerprint `ceb63a1254c3`. A scratchpad prototype confirmed most of the claims below against the live model. Its location is under §7.

---

## 1. Goal

`cad/params.yaml` already holds the robot's numbers: lengths, servo physics, limits and bus ids. D052 moved these into `gait/rocky_model.py`. The robot's shape is still written as code in about ten files:

- five legs
- a yaw/hip/knee chain
- stations at `90 + 72·i`
- the actuator order that `ctrl[:15]` / `ctrl[15:20]` rely on

Examples:
- `rocky_model.py:40` `LEG_JOINTS = ("yaw", "hip", "knee")`
- `pebble_gait.py:35-36` `N_LEGS = 5`, `STATION_DEG = 90 + 72 * np.arange(N_LEGS)`
- `build_mjcf.py:133,137,167` `range(5)`, plus the kp/kv literals at `:135,:138`
- `generate_urdf.py:196-197` `radians({90 + 72 * i})`, `range(5)`
- `check_urdf_parity.py:170` `n_lim != 20`

The owner plans to change the design as it goes:
- rebuild the CAD
- swap servos or materials
- add joints or actuators
- build the first physical leg soon

Each of those changes should work like this:

1. **Edit `cad/params.yaml`.**
2. **Run `rocky.sh regen`.** It regenerates mass_budget, MJCF, URDF and ros2_control, then runs the parity and staleness checks.
3. **Read the report.** It gives the new fingerprint and topology hash, and lists which RL checkpoints still load, need re-evaluation, or are refused.
4. **Follow any check that fails.** Checks fail loudly; nothing breaks silently.

**CAD is the exception.** The deck, shell and stand are designs, not parameters. The pentagon is built into `part_deck.py:36,55` and `part_shell.py:51-77`. CAD therefore stays in sync through parity checks that measure it against the description (§5), not through generation from it.

Requirements for every step below:
- It is its own commit.
- It keeps the 61-test suite green.
- It keeps the `pebble.xml` staleness test green (`test_model_consistency.py:167-170`).
- It keeps CI's regenerate-then-`git diff --exit-code` job green (`ci.yml:88-103`).
- It leaves `sim/pebble.xml` byte-identical unless the step is an explicit D-number.

---

## 2. Data model: a `robot:` block in `cad/params.yaml`

### 2.1 Placement and anchors

- The block goes **after `bus:`**, because YAML anchors must be defined before they are used.
- Existing keys get anchors with **values unchanged**. D052 already uses this pattern: `&st3215_no_load` at :28, `&st3215_mass_g` at :44, `&joint_pos_deg` at :57.
- Each number exists once. CAD keeps reading `leg.l1_coxa` as it does today.
- Only calibration files are ever written back with `safe_dump` (`bench/calibrate_centers.py:154`, `sim/hw_bridge.py:888`). No tool rewrites `params.yaml`, so the anchors survive.

```yaml
leg:
  l1_coxa: &l1_coxa 45.0         # :168
  hip_axis_z: &hip_axis_z 61.0   # :169
  l2_femur: &l2_femur 95.0       # :170
  l3_tibia: &l3_tibia 135.0      # :171
  servo: &leg_servo st3215       # :172  (still the key every current reader uses)
  claw_servo: &claw_servo scs0009
body:
  circumradius: &body_r 110.0    # :189
```

### 2.2 Example 1: today's pentapod, written out explicitly

```yaml
robot:                          # D053: topology. rocky_model.robot() reads this.
                                # If the block is absent, the loader builds exactly this from leg/body/bus.
  legs:
    count: 5
    first_station_deg: 90       # leg 0 north, CCW; station i = first + i*360/count
                                # NOT wrapped: leg 4 is 378 (pebble.xml:109 says euler="0 0 378")
    # station_deg: [...]        # optional explicit list; non-uniform needs allow_asymmetric: true
    station_radius_mm: *body_r
  leg:                          # template instantiated per leg; chain is base -> foot,
                                # origins in mm in the PARENT joint frame (= today's LEG frame)
    chain:
      - {name: yaw,  axis: [0, 0, 1],  offset_mm: [0, 0, 0],                  link: coxa}
      - {name: hip,  axis: [0, -1, 0], offset_mm: [*l1_coxa, 0, *hip_axis_z], link: femur}
      - {name: knee, axis: [0, -1, 0], offset_mm: [*l2_femur, 0, 0],          link: tibia}
    foot: {offset_mm: [*l3_tibia, 0, 0]}   # IK point; contact geometry stays in leg.foot
    tool: {name: claw, axis: [0, 0, 1], servo: *claw_servo}   # role tool: not in IK / RL action
    servo: *leg_servo           # default per joint; a chain entry may say servo: <actuators key>
    ik: {solver: auto, branch: knee_down}   # auto = analytic iff the chain is yaw+2R (§3.2)
  overrides: {}                 # per-leg/per-joint servo; validate() REFUSES non-empty
                                # until a consumer honours it (step 10)
```

Ranges stay in `joints.pos_deg`, keyed by joint name. Bus ids stay in `bus.leg_ids` (rows) and `bus.hand_ids`. `robot:` only links these together.

**The actuator blocks gain keys (step 2).** The kp/kv literals move out of `build_mjcf.py:135,138`. The protocol moves out of `robot.py:58-60` and `hw_bridge.py:158`, which today set `Family.STS` for every leg id.

```yaml
actuators:
  st3215:  {…existing…, protocol: sts, kp: 20,  kv: 0.6,  armature: <current literal>, cad: servo_st3215}
  scs0009: {…existing…, protocol: scs, kp: 0.5, kv: 0.02, armature: <current literal>, cad: servo_scs0009}
```

`cad:` names the CAD geometry module. The validator can then catch `leg.servo: X` while CAD still builds ST3215 cups (`servo_st3215.py:32-33`).

**Materials (step 7, not before):**

```yaml
materials:     {PLA: {density_g_cm3: 1.24}, PETG: {density_g_cm3: 1.27}, CF_PLA: {density_g_cm3: 1.30}}  # CF_PLA: VERIFY
part_material: {default: PLA, coxa_yaw_base: PETG, coxa_fork: PETG, femur_link: PETG, ...}             # = PRINT_PLAN batch 2
part_fill:     {default: 0.6, ...}   # replaces print_estimate PLATES and the silent 0.6 in mass_audit
```

### 2.3 Example 2: a 4th joint per leg (ankle)

```yaml
leg: {l3_tibia: &l3_tibia 100.0, l4_tarsus: &l4_tarsus 35.0}
robot:
  leg:
    chain:
      - …yaw, hip, knee as above…
      - {name: ankle, axis: [0, -1, 0], offset_mm: [*l3_tibia, 0, 0], link: tarsus, servo: <new key>}
    foot: {offset_mm: [*l4_tarsus, 0, 0]}
    ik: {solver: numeric, branch: knee_down, redundancy: foot_pitch, foot_pitch_deg: -90}
joints.pos_deg.ankle: [-60, 60]
bus.leg_ids: 5 rows x 4          # renumbered; hand_ids move up
actuators.<new key>: {stall/no-load/counts…, protocol, kp, kv, armature, mass_g, cad}
```

- A 4-joint leg reaching a 3-D point has one spare degree of freedom, so the IK needs a `redundancy` rule: `foot_pitch`, or `min_change` from the previous q.
- Without that rule, the prototype's numeric IK returned whichever solution was nearest its seed. That makes it jump between ticks.
- `validate()` refuses `solver: analytic` for this chain.
- `actuator_order()` becomes 20 leg actuators plus 5 claws. `topology_hash()` changes, so every policy is refused (§6).

### 2.4 Example 3: a 6th leg

```yaml
robot: {legs: {count: 6, first_station_deg: 90}}   # 60° spacing: 90, 150, … 390
bus.leg_ids: 6 rows          # e.g. leg 5 = [16, 17, 18]
bus.hand_ids: 6 ids          # 19-24
gait.duty: 0.8333            # = 1 - 1/6, or 0.5 for a tripod
```

- The validator refuses a mismatch between `count` and the number of `leg_ids` rows.
- It warns when `duty != 1 - 1/count`. `params.yaml:77` is literally "4 of 5 feet always down".
- The deck and shell are pentagon geometry, so CAD needs a hexagon redesign. The parity check in §5 fails until that is done; the loader cannot hide it.

---

## 3. Module API

### 3.1 `gait/rocky_model.py` (pure Python + PyYAML, no numpy; the driver imports it)

The dataclasses go **inside `rocky_model`**. A separate module would import `rocky_model` for `params()` while `rocky_model` derives constants from it, which is circular. Keeping them here also means there is one loader.

Frozen dataclasses:
- `JointSpec(name, axis, offset_mm, link, servo, range_deg, bus_id, role)`
- `LegSpec(index, station_deg, mount_mm, joints, foot_offset_mm, tool, tool_bus_id, ik)`, with `.n_dof` and `.joint_names`
- `RobotSpec(n_legs, station_radius_mm, legs, params_rev)`
- `RobotSpecError(ValueError)`

Functions:
- `robot(path=None) -> RobotSpec`: **lazy**, cached like `params()`, deep-copied out, validated on first build. A malformed `robot:` block breaks only importers that touch topology. `actuator()` and `leg_ids()` keep working during a partial edit.
- `validate(P) -> list[str]`: errors raise `RobotSpecError`; warnings are returned and printed. `validate()` also runs from `rocky.sh regen` and `cad-check`.
  - **Errors:**
    - `len(leg_ids) != count`
    - a row length that is not `n_dof`
    - duplicate ids across legs and hands
    - a chain joint with no `joints.pos_deg` entry
    - a servo not in `actuators`
    - `solver: analytic` on a chain that is not yaw+2R
    - `solver: numeric` with more than 3 DOF and no `redundancy`
    - non-empty `overrides`
    - non-uniform stations without `allow_asymmetric`
  - **Warnings:**
    - `duty != 1 - 1/count`
    - an actuator with no `cad` key, or one whose CAD module is not `servo_st3215` while CAD is still hard-keyed
- `n_legs()`, `stations_deg()`: a tuple of floats, unwrapped.
- `leg_joint_names(leg=0)`, `n_leg_dof()`, `uniform_dof()`: raises if legs differ. Consumers stay `(N, dof)` until step 10.
- `servo_for(joint, leg=0)`.
- `actuator_order()`: `['yaw0','hip0','knee0',…,'knee4','claw0',…,'claw4']`, leg-major, then tools. This makes today's `ctrl[:15]` contract a named, tested fact.
- `id_table()`: rows of `(sid, leg, joint, servo, protocol, counts, sweep_deg, limits_deg)`. This is the single table that the driver, `hw_bridge` and mock build from.
- `topology()` / `topology_hash()`: 12-hex over n_legs, stations, joint names and order, axes, tool and IK solver. It decides whether a checkpoint can load at all.
- `description_digest()`: over everything in the spec. It feeds the robot fingerprint in step 5.
- `to_json()` for a later cockpit model panel, and `summary()`.
- `LEG_JOINTS` / `ALL_JOINTS` stay tuples equal to today's.
  - They are served through a module `__getattr__` (PEP 562), which calls `robot()` on first access. Importing `rocky_model` never forces validation.
  - `_servo_name()` keeps reading `leg.servo` / `leg.claw_servo`. It routes through `servo_for` only when `overrides` gets a consumer.

### 3.2 `gait/leg_kin.py` (numpy only; step 6)

`LegKinematics(leg_spec)`:
- `.fk(q)`: a product of chain transforms.
- `.points(q)`: every joint point and the foot, for CoM and reach.
- `.jac(q)`: analytic, `axis × lever`.
- `.reach()` and `.within_limits(q, guard_deg)`.
- `.analytic`: a bool.
- `.ik(p, q_prev=None)`:
  - It takes the fast path when a **strict structural predicate** holds: 3 joints; axis 0 is z at the origin; axes 1 and 2 are parallel ±y; every offset and the foot have y = 0; knees and foot lie along +x.
  - On the fast path it runs today's `pebble_gait.leg_ik` code moved verbatim (`pebble_gait.py:47-59`). The path is chosen by that shape check, never by joint name.
  - Otherwise it uses damped least squares with the analytic Jacobian, a warm start from `q_prev`, limit clamping, the `branch` constraint and the `redundancy` rule. It returns NaN when the point is unreachable, as today.

`chains(spec)` returns one `LegKinematics` per leg. `pebble_gait.leg_fk` and `leg_ik` become thin wrappers.

Measured in the prototype: analytic `leg_ik` takes 10 µs. Numeric DLS with a finite-difference Jacobian takes 1.4-2.2 ms per leg, which is about half of a Pi's 50 Hz budget for 5 legs. Numeric IK therefore cannot become the default for any chain until a **measured per-leg time on the robot computer** is recorded and passes a budget test.

### 3.3 `sim/model_index.py` (numpy + MuJoCo; step 4)

`ModelIndex(model, spec)` addresses the compiled model by name, never by position:
- `.leg_qpos`, `.leg_dof`, `.leg_act`: arrays of shape `[n_legs, dof]`
- `.tool_act`, `.foot_geom`, `.foot_site`
- `.assert_matches()`: raises if `model` actuator names differ from `rm.actuator_order()`

### 3.4 Packaging

- Add `leg_kin` to `pyproject.toml` `[tool.setuptools] py-modules` (line 32-33) alongside `rocky_model`.
- A subprocess purity test asserts that `numpy` is not in `sys.modules` after `import rocky_model`.
- `hw_bridge` must not import `leg_kin` on the safety path.

---

## 4. Migration steps, in order, each with the check that proves it

| # | Step | Check that proves it |
|---|---|---|
| **1** | **Spec in the loader, derived stations and actuator order in both generators** (§7) | `pebble.xml` byte-identical; new URDF xacro staleness test; fingerprint still `ceb63a1254c3`; `check_urdf_parity` n_lim 20; new `gait/test_robot_spec.py` |
| **2** | **Generators consume the chain.** `build_mjcf.leg_xml` / `actuators_xml` (:103-140) and `generate_urdf.leg_macro` / `build_xacro` (:101-197) take bodies, joints, axes, offsets and kp/kv/armature from the spec and the `actuators.*` blocks. `rocky.ros2_control.xacro` (hand-written yaw/hip/knee/claw macro) becomes generated. | `pebble.xml` and `pebble.urdf.xacro` byte-identical. `ros2_control` gets a one-time reviewed diff against the hand-written file, then its own staleness test. The step-1 per-link test still passes. Without this step, an ankle added to the chain would validate but never reach MJCF, URDF or ROS. |
| **3** | **Close the paths that fail open.** None of these changes behaviour today, but each would fail silently or dangerously on a topology change:<br>• `pebble_feasibility` FAIL_CODES (:97-99) built from `LEG_JOINTS`; `divmod(k, 3)` → `divmod(k, n_dof)` (:434-438)<br>• `check_urdf_parity.py:170` `n_lim != 20` → `len(rm.actuator_order())`; joint list (:78) and random-q bounds (:98) from the spec<br>• `mock.py:342-346` ids from `rm.id_table()`; hands at `15+i` collide with legs at 6 legs<br>• `robot.py:41` broad `except Exception` → `except ImportError`, so a `RobotSpecError` is not swallowed into the 3-joint `_FALLBACK_LIMITS_DEG`<br>• `hw_bridge` refuses to start if any id in `id_table()` lacks a NaN guard or rate limit<br>• `build_mjcf.py:82-95` and `generate_urdf.py:60-68` fail when `mass_budget.json` is missing; explicit `--allow-hand-budget` for bootstrap | Existing suites pass unchanged. New tests: a synthetic `LIMIT_ANKLE` counts as a fail; the mock gives unique ids for a 6-leg fixture; a malformed params raises through `soft_limits_deg()`; the bridge refuses an uncovered id |
| **4** | **Index actuators by name everywhere.** `ModelIndex` replaces `ctrl[:15]`, `ctrl[15:20]` and `nu>=20` in:<br>• `playground.py` :384-482, :705-777<br>• `rocky_env.py` :142-245 (:222, :244-245)<br>• `rocky_recover_env.py` :112-128, :196-321<br>• `rl_common.py` motor_torque, ThermalProxy, DR (:54-55, :92-127, :369-539)<br>• `sim_lidar.py` :111-143<br>• `servo_model.py:66-67` `n=15` → per-joint vectors from each joint's servo | Guard test: the compiled model's actuator names equal `rm.actuator_order()`. `test_rl_envs`, `test_playground_guards` and `gait/test_reflex_fallen` are unchanged. Evaluating `recover6_d052` gives the same success count as before. This step comes before any topology change: a 4th joint would scramble `ctrl[:15]` with no error. |
| **5** | **Checkpoint safety** (§6):<br>• stamp `topology_hash`, joint names and per-joint `q_lo/q_hi` into env_config<br>• unstamped checkpoints default to the 5×3 legacy hash<br>• `check_topology()` refuses on mismatch<br>• recover `action_to_q` reads the stored `q_lo/q_hi` (`rocky_recover_env.py:220`; today nothing reads them back)<br>• strict mode (`ROCKY_STRICT_FP=1`, default for train resume)<br>• re-admission via `eval_recover --accept-robot <fp>`<br>• fingerprint extended to geom solref/solimp/margin, mesh data, `counts_per_rad`, actuator dyntype/biastype and `description_digest()` | Tests with a synthetic topology change: all 10 checkpoints under `sim/runs/` are refused, including the 9 unfingerprinted ones such as `recover1`. With the fingerprint changed but the topology unchanged: warn, refuse under strict, then re-admitted after `--accept-robot` passes. The fingerprint rotates once by design (a D-number); `recover6_d052` then warns. |
| **6** | **`LegKinematics`** (§3.2) replaces the four FK copies: `pebble_feasibility.py:150-200` → `.points`, `pebble_reflex.py:136-144` → `.reach()`, `pebble_keyframes.py:114-140` → reach bounds. Audit `gait/pebble_pose_solver.py` for a fifth copy. | Analytic path bitwise equal to the old `leg_ik`/`leg_fk` on a grid; `fk(ik(p)) == p`; numeric IK equal to analytic within 1e-6 mm on the 3-DOF chain and stays knee-down; a 4-DOF fixture with `foot_pitch` gives continuous q along a straight foot path (no branch jumps); a per-leg timing test records ms/leg |
| **7** | **Materials and mass layout (explicit D-number).** mass_audit and print_estimate read `materials` / `part_material` / `part_fill`. This fixes PETG parts being weighed as PLA. One `sim/body_layout.py` owns the primitive mass layout (coxa box, 0.7/0.3 tibia split, `M_PRONG`, torso 75/25), which today is copied in `build_mjcf.py:98-100,109,115,165-166`, `generate_urdf.py:102-107` and `pebble_feasibility.py:150-161`. | `check_urdf_parity` compares CoM **per link**, not only total mass (:131-141). `mass_budget.json` diff reviewed. Fingerprint rotates; feasibility, shove envelope and audits re-run and recorded in `decisions.md` with a retrain/audit plan |
| **8** | **CAD sync checks** (§5): runs in parallel with 3-7. **The tibia-stack item lands before the first leg is assembled.** | `cad/check_params_parity.py` in `run_all_checks` MODULES; `rocky.sh cad-check` green |
| **9** | **Operations and docs.** `rocky.sh regen` runs `validate` → mass_audit → build_mjcf → generate_urdf (+ ros2_control) → check_urdf_parity → pytest model_consistency, then prints the fingerprint and topology hash before and after and every checkpoint as ok / re-eval / refused. Update SIM_GUIDE §6 ("Regenerating after a CAD change") and RL_GUIDE §2 and §7 ("what a design change costs"). Fix drift: README:29 (`recover6_d052` exists and is negative), RL_GUIDE §1 (load_derate off since D052 V2). | Running `rocky.sh regen` on a clean tree gives an empty `git diff` |
| **10** | **Only when a topology change is actually made:**<br>• driver/hw_bridge per-joint table from `id_table()`: `robot.py:24-25, 58-60, 97-158`; `hw_bridge.py:117` JOINTS, `:137` calibration regex, `:158` Family, `:184-187`, `:407-424`, `:620-627`<br>• reflex/gestures/keyframes generalised from `(N,3)`: `pebble_reflex.py:275,307,522` reshape, `:433` `n_con >= 4` → `n-1`, `:603-619` 3-vector fallback, `pebble_gestures2.py:73-75`, `pebble_keyframes.py:107`, stored keyframe migration<br>• `ArmGait` `n == 4 → 0.78` (`pebble_gait.py:289-290`) → n-based<br>• `duty` defaults to `1 - 1/n_legs`<br>• RL obs builder generated from topology (new obs version); `OBS_DIMS` computed, legacy literals kept for decoding old checkpoints only<br>• `overrides` consumer (per-leg `servo_for` in `actuator()`)<br>• CAD deck/shell as an n-gon (a redesign) | Per-change fixtures for the topology in question; the full suite passes; a fresh policy trains |

The concurrent workflow owns `harness/*`, `sim/cockpit*.py`, `sim/cockpit_ui.html`, `sim/cockpit_shared.py` and `docs/COCKPIT_GUIDE.md`. `harness/sim_backend.py:124,201,240,373` and `cockpit.py` use positional ctrl slices. Their migration to `ModelIndex` is handed over and happens after that work lands. The cockpit gets checkpoint refusals through its existing error path. A `GET /api/robot` model panel (`to_json()` plus fingerprints plus checkpoint compatibility) is a later item for that workflow.

---

## 5. CAD sync rules

### 5.1 What each change triggers

| Change | Follows automatically | Needs a manual step or new check |
|---|---|---|
| L1 / L2 | `leg_frame.py:30-34` HIP_TF/KNEE_TF → cad-check → mass_audit → MJCF/URDF → parity | — |
| L3 | Chain, gait, MJCF, URDF (via anchors) | **Nothing in CAD owns L3.** The tibia-stack check below compares CAD to `l3_tibia` and prints the tube cut length. |
| `hip_axis_z` | Chain, gait, MJCF, URDF | Assert it equals the yaw-stack height leg_frame derives (`Z_YAW_TOP`, `leg_frame.py:42`) |
| Servo swap, same case | Physics, limits-derived speeds, mass (mass_audit follows `leg.servo`), kp/kv (step 2), protocol | Fingerprint rotates; re-evaluate checkpoints (§6) |
| Servo with a new case | — | A new CAD geometry module named by `actuators.<k>.cad`. `servo_st3215.spec()` reads `P[f"servo_{servo_for(joint)}"]` and raises when no geometry block exists, instead of building ST3215 cups for a different case. |
| Material | mass_audit and print_estimate via `part_material` (step 7) | A check fails on any part with no material or fill entry, instead of a silent 0.6 |
| Leg count | Spec, generators, gait stations | Deck, shell and stand redesign; the parity check fails until it is done |

### 5.2 `cad/check_params_parity.py` (new, in `run_all_checks` MODULES)

- **Tibia stack:** sums `part_tibia` BOSS_H + SOCKET_DEPTH, the tube cut, SEA, `part_hand` HUB_H/CONE_LEN and `part_footpad` CROWN, and compares the total to `l3_tibia` within ±1 mm.
- **Tube cut length:** prints it. None exists anywhere in the repo today, and the first leg build needs it.
- **Foot geometry:** `part_hand.CONE_TIP_R == leg.foot.tip_radius` (`part_hand.py:31`) and `part_footpad.CROWN == pad_crown` (`part_footpad.py:20`).
- **Deck:** station radius and count equal `robot.legs` (`part_deck.py:34,38,55,57`).
- **Literals:** checks the literal 110/100/118/185/140 in `part_deck`, `part_shell`, `part_dock`, `part_bench_jig` (KNEE_X 140 vs L1+L2, :13,52-61) and `part_smallwins` (:88) against params. This is a first pass, before those files read params directly.
- **Dock:** heights vs `gait.body_height` and stance radius (`part_dock.py:92,100`).
- **Sweeps:** CAD sweep ranges must cover `joints.pos_deg`. The sweeps at `part_coxa.py:153`, `check_assembly.py:123`, `part_tibia.py:123` and `check_interference.py:47` become driven by `joints.pos_deg`.

In the same step:
- Add `check_interference.py` to MODULES. Replace the MODULES list (`run_all_checks.py:26-33`) with glob discovery.
- `part_deck.py:139-145` asserts its clash verdict instead of only printing it.

### 5.3 Independent witnesses

Once one spec drives both generators, MJCF-vs-URDF parity mostly proves that the two generators agree with each other. These witnesses stay independent and are never rewritten to read the spec:
- the closed-form `pebble_gait.leg_fk` / `leg_ik` (moved verbatim, not regenerated)
- the CAD-measured HIP_TF/KNEE_TF from `cad/leg_frame.py`
- the committed `pebble.xml` body positions and axes, pinned by the step-1 per-link test

**CI is the end-to-end gate:** regenerate everything, then `git diff --exit-code`. Locally, `rocky.sh regen` plus the staleness tests cover the same ground.

---

## 6. RL and fingerprint rules

| Change | Robot fingerprint | Topology hash | Policies |
|---|---|---|---|
| L3 ±5 mm, servo swap, material, friction, masses | rotates | unchanged | **Warn** by default; **refuse** under strict mode (`ROCKY_STRICT_FP=1`, the default for train resume) and for the righter on the safety path. Re-admitted by `eval_recover --accept-robot <fp>`, which re-runs the recorded success baseline and writes an accepted-fingerprints sidecar. Recover checkpoints also flag an action-range change via the stored `q_lo/q_hi`. |
| Ankle, 6th leg, actuator reorder | rotates | changes | **All refused**: righter, eval_*, train resume, the cockpit. There is no warm start: obs/act dimensions change, as the 39→57 history (B34) showed. |

Rules:
- A checkpoint with no stamp is the 5×3 pentapod, the only topology that ever existed. All 9 unfingerprinted checkpoints (`recover1` is the default righter; `rl_common.py:613-626` returns 'unknown' for them) are therefore refused after a topology change instead of running silently.
- Refusing on fingerprint mismatch ships **together** with `--accept-robot`. Otherwise the first geometry change under strict mode locks out the cockpit's righter with no documented way back.
- After a topology change, the FALLEN path has no learned righter until one is retrained. `set_righter`'s scripted fallback degrades to the `recover1` staircase behaviour.
- Extending the fingerprint (step 5) rotates it once, deliberately, and gets a D-number.
- **Gait retune goes through the budget, not RL.**
  - `WaveGait.budget` and `pebble_feasibility` (speed classes, THERMAL_LOAD, support margin) re-derive the envelope from the new servo and lengths.
  - Re-run `audit_gestures`, `audit_righter` and `shove_envelope`, and read the new command envelope.
  - The literal command scales in `rocky_env.py:185, 203-208` come from that envelope.
  - The handoff thresholds (`rocky_recover_env.py:96-107`, metres) scale with stance height from the spec.

---

## 7. First step to build now (~3-4 h)

**Scope:** the `robot:` spec in the loader, derived stations and actuator order in both generators, and pinning tests. Output is byte-identical; nothing else changes.

A prototype is in the scratchpad (`/tmp/claude-1000/-home-bitwisebard-Development-rocky/617aee55-a110-4e95-989e-f8423ee0b4fa/scratchpad/proto/`: `robot_description.py`, `leg_chain.py`, `check.py`). It already reproduces the actuator order, body positions within 0.06 mm, the bus map, and FK agreement to 6.6e-14 mm. Port its logic into `rocky_model`; do not add a second module.

### 7.1 `cad/params.yaml`
- Add the anchors from §2.1 (lines 168-173, 189), values unchanged.
- Append the `robot:` block from §2.2 after `bus:`.
- Leave `params_rev` alone: no physics number changed.

### 7.2 `gait/rocky_model.py`
- Add the dataclasses, `RobotSpecError`, and lazy cached `robot(path=None)`. When `robot:` is absent, it builds the same spec from `leg`/`body`/`bus`.
- Add `validate()` with the §3.1 errors and warnings; non-empty `overrides` is refused.
- Add `n_legs`, `stations_deg`, `leg_joint_names`, `n_leg_dof`, `uniform_dof`, `servo_for`, `actuator_order`, `id_table`, `topology`, `topology_hash`, `description_digest` and `summary`.
- `LEG_JOINTS` / `ALL_JOINTS` go through module `__getattr__` and stay equal to today's tuples.
- `_servo_name` stays unchanged.

### 7.3 `gait/pebble_gait.py:35-36`
- `N_LEGS = _rm.n_legs()`.
- `STATION_DEG = np.array(_rm.stations_deg())`.
- The float dtype is harmless: every consumer passes it through `np.deg2rad`, indexes it, or prints it with `:.0f`.

### 7.4 `sim/build_mjcf.py`
- `leg_xml(i)` uses `ang = rm.stations_deg()[i]` and emits `{ang:g}`. The text must stay `euler="0 0 162"`, not `162.0`, and stay unwrapped (`378`).
- `range(5)` at :133, :137 and :167 becomes `range(rm.n_legs())`.
- `actuators_xml()` iterates `rm.actuator_order()`, keeping the kp/kv literals per kind. They move into params in step 2.

### 7.5 `ros2/rocky_description/generate_urdf.py:196-197`
- `radians({ang:g})` over `rm.stations_deg()`.
- `range(5)` becomes `range(rm.n_legs())`.

### 7.6 Tests

**New `gait/test_robot_spec.py`** (pure, no MuJoCo, under 1 s):
- **(a)** Today's spec equals the old literals:
  - n_legs 5 and stations `(90, 162, 234, 306, 378)`
  - `LEG_JOINTS == ('yaw','hip','knee')` and `ALL_JOINTS[-1] == 'claw'`
  - chain offsets equal `(L1, 0, Z_HIP)`, `(L2, 0, 0)`, `(L3, 0, 0)`
  - `servo_for` gives `st3215` for each leg joint
  - `id_table()` inverts identically to `rm.id_to_joint` for ids 1..20, and bus ids equal `rm.leg_ids()` / `hand_ids()`
- **(b)** A generic chain FK (numpy transform product from `JointSpec` axis and offset) equals `pebble_gait.leg_fk` within 1e-9 mm on 500 random in-limit q.
- **(c)** `actuator_order()` equals the `<position name=…>` sequence parsed with ElementTree from the committed `sim/pebble.xml`. This locks the `ctrl[:15]` / `[15:20]` contract.
- **(d)** Negative validation on tmp copies of `params.yaml` loaded via `robot(path=tmp)`. Each raises `RobotSpecError` with a useful message:
  - 6 `leg_ids` rows with count 5
  - a 4-id row
  - an unknown servo
  - a joint without `pos_deg`
  - a duplicate id
  - `solver: analytic` with an ankle (the message names `numeric`)
  - numeric 4-DOF without `redundancy`
  - non-empty `overrides`
- **(e)** Hypothetical fixtures load as specs:
  - 6 legs gives stations 60° apart and `len(actuator_order()) == 24`
  - 4 joints with `solver: numeric` + `redundancy` gives 25
  - both `topology_hash` values differ from today's
- **(f)** Params without a `robot:` block build a spec and hash equal to (a).
- **(g)** Mutating a `robot()` result does not leak into the cache (same pattern as `test_loader_returns_copies`).
- **(h)** Purity: in a subprocess, `import rocky_model` leaves `numpy` out of `sys.modules`. Importing it with a malformed `robot:` block still lets `actuator()` work.

**Added to `sim/tests/test_model_consistency.py`:**
- `test_urdf_xacro_is_regenerated`:
  `generate_urdf.build_xacro().replace('__TORSO_INERTIAL__', inertial(*combine_torso_inertia()))` equals the committed `urdf/pebble.urdf.xacro`. Today only CI's git diff catches URDF staleness.
- `test_spec_matches_compiled_links`: for every leg, from the compiled `pebble.xml`:
  - `coxa{i}.pos` equals the mount
  - `femur{i}.pos` equals the hip offset
  - `tibia{i}.pos` equals the knee offset
  - `foot_tip{i}` equals the foot offset, within 0.06 mm
  - joint axes and ranges equal
  
  Until step 2, this closes the gap between the spec's shape and the generator templates.

### 7.7 Verify
1. `MUJOCO_GL=egl .venv/bin/python sim/build_mjcf.py && .venv/bin/python ros2/rocky_description/generate_urdf.py`, then `git diff --stat sim/pebble.xml ros2/` must be empty. If it is not, fix the formatting; do not commit a regenerated model.
2. `.venv/bin/python -m pytest -q gait/test_robot_spec.py sim/tests/test_model_consistency.py gait/test_feasibility.py sim/tests/test_rl_envs.py sim/tests/test_hw_bridge.py sim/tests/test_playground_guards.py gait/test_reflex_fallen.py` must all pass. Model consistency plus feasibility is 61 today, plus the new tests.
3. `.venv/bin/python sim/check_urdf_parity.py` must pass with n_lim 20.
4. The robot fingerprint must still be `ceb63a1254c3`.
5. `.venv/bin/python gait/rocky_model.py` prints the summary with the topology line and hash.

### 7.8 Docs in the same step
- A "Robot description" paragraph in `docs/SIM_GUIDE.md` §6: what exists now, and what is not yet wired (steps 2-10).
- A `docs/DESIGN_BACKLOG.md` entry for steps 2-10.
- A `docs/decisions.md` D053 entry: robot shape as data; zero behaviour change.

### 7.9 Out of scope for this step
- Anything under `harness/`, cockpit files or `COCKPIT_GUIDE.md`.
- Port 8765: the running cockpit sees no model change because `pebble.xml` is byte-identical.
- kp/kv/protocol into params (step 2).
- Any positional-ctrl migration (step 4).
- No commit or push without the owner.

---

## 8. Risks to keep in view

- **Analytic IK shape.** `leg_ik` is exact only for yaw about +z plus two parallel −y pitches, with y = 0 offsets, a +x foot and the knee-down branch. `ik_ok` rejects any q whose last dimension is not 3 (`pebble_gait.py:68`). The fast path must be chosen by the structural predicate plus the cross-check test.
- **Numeric IK cost and redundancy.** It needs the analytic Jacobian, a warm start, a branch constraint, a redundancy rule and a measured Pi budget before it can be the default.
- **Symmetric-station assumptions.** These all assume equal CCW spacing and stay symmetric-only until audited:
  - `WaveGait` `phase_off = arange(N)[::-1]/N` (`pebble_gait.py:122`)
  - `pebble_manip_adjacent.py:30` ARM_LEGS "any pair works by symmetry", and its lean math at :91-108
  - the playground touchdown gate (`playground.py:257-276`)
  - the CAD 72° sectors
- **Positional ctrl slices** stay a silent scrambler until step 4. Steps 1-3 must keep leg-major actuator order, and test (c) is load-bearing.
- **Safety path.** Until step 3, a spec error in the driver could fall back to 3-joint literal limits. Until step 10, the bridge knows only yaw/hip/knee.
- **Mass and material changes** (step 7) shift feasibility margins, shove thresholds and RL dynamics. They are an explicit decision with a retrain/audit plan, not a drive-by refactor.
- **First leg build.** It will find any tibia-stack drift. Land the §5.2 tibia check before assembling.

---

## 9. Progress

- **2026-09-24: step 1 built (D053).** Status line above is the design as written; since then:
  - the `robot:` block is in `cad/params.yaml`
  - `rocky_model` has the `RobotSpec` API
  - `pebble_gait`, `build_mjcf` and `generate_urdf` read stations, leg count and actuator order from it

  `sim/pebble.xml` and the URDF are byte-identical. The fingerprint is still `ceb63a1254c3`. The topology hash is `7f066d9bd8c0`.

  Tests: `gait/test_robot_spec.py` (24) and three new tests in `test_model_consistency.py`.

  Deviations from §3/§7:
  - `summary()` keeps its servo line (sim/audit_gestures.py prints it). The description line is `robot_summary()` / `RobotSpec.summary()`.
  - Warnings are returned by `validate()` / `robot_warnings()`, and printed only by `python gait/rocky_model.py`. The loader never prints, because stdio MCP servers import it.
  - The module helpers take an optional `spec=` argument, so fixtures can be queried without touching `params.yaml`.
- **2026-09-25: tools are data too (D056).** The robot's tools follow the same rule as its shape: one declarative registry, `harness/capabilities.py`, and every surface generated from it (the local brains' OpenAI schema, the cockpit's list, the MCP server's list, `docs/TOOLS.md`). Its snapshot carries two sections read from this data: `envelope` from `WaveGait().max_command()` on `cad/params.yaml` (speed 0.0455 m/s, turn 0.246 rad/s, step 24 mm, goto 0.045 m/s) and `robot` from `rocky_model.robot()` (legs, joints per leg, stations, 20 actuators, topology hash `7f066d9bd8c0`, `params_rev`). The envelope reaches the descriptions through the snapshot: the goto and move texts take their number from `envelope.goto_max_m` / `move_max_m`, so every surface quotes the same one. For `move` it is the hard limit (a longer move is refused); for `goto` it is the advised reach the text quotes ("keep targets within ~1.5 m"), not a refusal: the cockpit refuses a goto target only beyond 3 m (`cockpit_brains.GOTO_MAX_M`), so despite its name `goto_max_m` is not goto's maximum. Today both are still the constant 1.5 m (inside what goto's 40 s cap covers at about 0.045 m/s, some 1.8 m), not derived from params; the goto text spells out "~0.045 m/s, 40 s cap" literally, and the other numbers in the descriptions (compose_gesture's joint ranges, find_object's 0.1-0.4 m steps) are literals in the registry. A params change that moves the envelope or the topology changes the snapshot version and makes `docs/TOOLS.md` stale, and CI refuses a stale page the way it refuses a stale `pebble.xml`.
- The design-change how-to for the current state is `docs/DESIGN_CHANGE_GUIDE.md`. The remaining steps are backlog B36.
