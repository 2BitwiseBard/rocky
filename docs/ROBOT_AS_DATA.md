# ROBOT_AS_DATA — the robot's shape lives in data

Status: step 1 shipped (D053, 2026-09-24); steps 2-10 are backlog B36. How
to change the design today: [DESIGN_CHANGE_GUIDE.md](DESIGN_CHANGE_GUIDE.md).
`python gait/rocky_model.py` prints the current description: 5 legs, 20
actuators, topology hash `7f066d9bd8c0`; `sim/model_fingerprint.py` gives
the robot fingerprint (`7d376178fe27` since D059).

---

## 1. Goal

`cad/params.yaml` holds the robot's numbers (lengths, servo physics,
limits, bus ids), and every consumer reads them through
`gait/rocky_model.py` (D052). Before D053 the robot's *shape* (five legs, a
yaw/hip/knee chain, stations at `90 + 72·i`, the actuator order that
`ctrl[:15]` / `ctrl[15:20]` rely on) was literals in about ten files. Step
1 moved the stations, the leg count and the actuator order into the spec;
what is still hard-coded is listed per step in §4.

The end state for any design change (a CAD rebuild, a servo or material
swap, another joint or actuator, the first physical leg):

1. **Edit `cad/params.yaml`.**
2. **Run `rocky.sh regen`** (step 9; not built yet). It regenerates
   mass_budget, MJCF, URDF and ros2_control, then runs the parity and
   staleness checks.
3. **Read the report**: the new fingerprint and topology hash, and which RL
   checkpoints still load, need re-evaluation, or are refused.
4. **Follow any check that fails.** Checks fail loudly; nothing breaks
   silently.

**CAD is the exception.** The deck, shell and stand are designs, not
parameters: the pentagon is built into `part_deck.py` and `part_shell.py`.
CAD stays in sync through parity checks that measure it against the
description (§5), not through generation from it.

Requirements for every step below: it is its own commit; it keeps
`sim/tests/test_model_consistency.py`, `gait/test_feasibility.py` and
`gait/test_robot_spec.py` green, including the staleness tests
(`test_pebble_xml_is_regenerated`, `test_urdf_xacro_is_regenerated`); it
keeps CI's regenerate-then-`git diff --exit-code` job green; and it leaves
`sim/pebble.xml` byte-identical unless the step is an explicit D-number.

---

## 2. Data model: the `robot:` block in `cad/params.yaml`

### 2.1 Placement and anchors

- The block sits **after `bus:`**, because YAML anchors must be defined
  before they are used.
- Existing keys carry anchors with **values unchanged** (`&l1_coxa`,
  `&hip_axis_z`, `&l2_femur`, `&l3_tibia`, `&leg_servo`, `&claw_servo`,
  `&body_r`; D052 set the pattern with `&st3215_no_load`, `&st3215_mass_g`,
  `&joint_pos_deg`). Each number exists once; CAD keeps reading
  `leg.l1_coxa`.
- Only calibration files are ever written back with `safe_dump`
  (`bench/calibrate_centers.py`, `sim/hw_bridge.py`). No tool rewrites
  `params.yaml`, so the anchors survive.

### 2.2 Example 1: today's pentapod (the shipped block)

```yaml
robot:                          # D053: topology. rocky_model.robot() reads this.
                                # If the block is absent, the loader builds exactly this from leg/body/bus.
  legs:
    count: 5
    first_station_deg: 90       # leg 0 north, CCW; station i = first + i*360/count
                                # NOT wrapped: leg 4 is 378 (pebble.xml says euler="0 0 378")
    # station_deg: [...]        # optional explicit list; non-uniform needs allow_asymmetric: true
    station_radius_mm: *body_r
  leg:                          # template instantiated per leg; chain is base -> foot,
                                # offsets in mm in the PARENT joint frame (= the gait's LEG frame)
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

Ranges stay in `joints.pos_deg`, keyed by joint name. Bus ids stay in
`bus.leg_ids` (rows) and `bus.hand_ids`. `robot:` only links these together.

**The actuator blocks gain keys (step 2).** The kp/kv literals move out of
`build_mjcf.actuators_xml()`, and the protocol out of `driver/rocky_driver/robot.py`
and `sim/hw_bridge.py`, which today set `Family.STS` for every leg id.

```yaml
actuators:
  st3215:  {…existing…, protocol: sts, kp: 20,  kv: 0.6,  armature: <current literal>, cad: servo_st3215}
  scs0009: {…existing…, protocol: scs, kp: 0.5, kv: 0.02, armature: <current literal>, cad: servo_scs0009}
```

`cad:` names the CAD geometry module, so the validator can catch
`leg.servo: X` while CAD still builds ST3215 cups (`servo_st3215.py`).
Until then `python gait/rocky_model.py` warns that neither actuator has a
`cad:` key.

**Materials (step 7, not before):**

```yaml
materials:     {PLA: {density_g_cm3: 1.24}, PETG: {density_g_cm3: 1.27}, CF_PLA: {density_g_cm3: 1.30}}  # CF_PLA: VERIFY
part_material: {default: PLA, coxa_yaw_base: PETG, coxa_fork: PETG, femur_link: PETG, ...}             # = PRINT_PLAN batch 2
part_fill:     {default: 0.6, ...}   # replaces print_estimate.FILL and the silent 0.6 in mass_audit
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

- A 4-joint leg reaching a 3-D point has one spare degree of freedom, so
  the IK needs a `redundancy` rule: `foot_pitch`, or `min_change` from the
  previous q. Without it, a numeric IK returns whichever solution is
  nearest its seed and jumps between ticks.
- `validate()` refuses `solver: analytic` for this chain.
- `actuator_order()` becomes 20 leg actuators plus 5 claws. `topology_hash()`
  changes, so every policy is refused (§6).

### 2.4 Example 3: a 6th leg

```yaml
robot: {legs: {count: 6, first_station_deg: 90}}   # 60° spacing: 90, 150, … 390
bus.leg_ids: 6 rows          # e.g. leg 5 = [16, 17, 18]
bus.hand_ids: 6 ids          # 19-24
gait.duty: 0.8333            # = 1 - 1/6, or 0.5 for a tripod
```

- The validator refuses a mismatch between `count` and the number of
  `leg_ids` rows, and warns when `duty != 1 - 1/count` (the params
  `gait.duty` comment is literally "4 of 5 feet always down").
- The deck and shell are pentagon geometry, so CAD needs a hexagon
  redesign. The parity check in §5 fails until that is done; the loader
  cannot hide it.

---

## 3. Module API

### 3.1 `gait/rocky_model.py` (pure Python + PyYAML, no numpy; the driver imports it) — shipped in step 1

The dataclasses live **inside `rocky_model`**: a separate module would
import `rocky_model` for `params()` while `rocky_model` derives constants
from it, which is circular, and one loader is simpler.

Frozen dataclasses: `JointSpec(name, axis, offset_mm, link, servo,
range_deg, bus_id, role)`; `LegSpec(index, station_deg, mount_mm, joints,
foot_offset_mm, tool, tool_bus_id, ik)` with `.n_dof` and `.joint_names`;
`RobotSpec(n_legs, station_radius_mm, legs, params_rev)`; and
`RobotSpecError(ValueError)`.

Functions (each takes an optional `spec=` so fixtures can be queried
without touching `params.yaml`):
- `robot(path=None) -> RobotSpec`: **lazy**, cached like `params()`,
  deep-copied out, validated on first build. A malformed `robot:` block
  breaks only importers that touch topology; `actuator()` and `leg_ids()`
  keep working during a partial edit.
- `validate(P) -> list[str]` and `robot_warnings()`: errors raise
  `RobotSpecError`; warnings are returned, and printed only by
  `python gait/rocky_model.py` (the loader never prints, because stdio MCP
  servers import it).
  - **Errors:** `len(leg_ids) != count`; a row length that is not `n_dof`;
    duplicate ids across legs and hands; a chain joint with no
    `joints.pos_deg` entry; a servo not in `actuators`; `solver: analytic`
    on a chain that is not yaw+2R; `solver: numeric` with more than 3 DOF
    and no `redundancy`; non-empty `overrides`; non-uniform stations
    without `allow_asymmetric`.
  - **Warnings:** `duty != 1 - 1/count`; an actuator with no `cad` key, or
    one whose CAD module is not `servo_st3215` while CAD is still
    hard-keyed.
- `n_legs()`, `stations_deg()` (a tuple of floats, unwrapped),
  `leg_joint_names(leg=0)`, `n_leg_dof()`, `uniform_dof()` (raises if legs
  differ; consumers stay `(N, dof)` until step 10), `servo_for(joint,
  leg=0)`.
- `actuator_order()`: `['yaw0','hip0','knee0',…,'knee4','claw0',…,'claw4']`,
  leg-major, then tools. This makes the `ctrl[:15]` contract a named,
  tested fact.
- `id_table()`: rows of `(sid, leg, joint, servo, protocol, counts,
  sweep_deg, limits_deg)`, the single table the driver, `hw_bridge` and
  the mock will build from (step 3/10).
- `topology()` / `topology_hash()`: 12 hex over n_legs, stations, joint
  names and order, axes, tool and IK solver. It decides whether a
  checkpoint can load at all.
- `description_digest()`: over everything in the spec; it feeds the robot
  fingerprint in step 5.
- `to_json()` (for a later cockpit model panel), `robot_summary()` /
  `RobotSpec.summary()` (the description line). The module `summary()`
  keeps its servo line, which `sim/audit_gestures.py` prints.
- `LEG_JOINTS` / `ALL_JOINTS` stay tuples equal to the old literals,
  served through a module `__getattr__` (PEP 562) that calls `robot()` on
  first access, so importing `rocky_model` never forces validation.
  `_servo_name()` keeps reading `leg.servo` / `leg.claw_servo`; it routes
  through `servo_for` only when `overrides` gets a consumer.

### 3.2 `gait/leg_kin.py` (numpy only; step 6)

`LegKinematics(leg_spec)`:
- `.fk(q)`: a product of chain transforms. `.points(q)`: every joint point
  and the foot, for CoM and reach. `.jac(q)`: analytic, `axis × lever`.
  `.reach()`, `.within_limits(q, guard_deg)`, `.analytic` (a bool).
- `.ik(p, q_prev=None)`:
  - It takes the fast path when a **strict structural predicate** holds: 3
    joints; axis 0 is z at the origin; axes 1 and 2 are parallel ±y; every
    offset and the foot have y = 0; knees and foot lie along +x. The path
    is chosen by that shape check, never by joint name (the predicate is
    already in `rocky_model`).
  - On the fast path it runs today's `pebble_gait.leg_ik` code moved
    verbatim.
  - Otherwise it uses damped least squares with the analytic Jacobian, a
    warm start from `q_prev`, limit clamping, the `branch` constraint and
    the `redundancy` rule. It returns NaN when the point is unreachable,
    as today.

`chains(spec)` returns one `LegKinematics` per leg; `pebble_gait.leg_fk`
and `leg_ik` become thin wrappers.

Measured in the step-1 prototype: analytic `leg_ik` takes 10 µs; numeric
DLS with a finite-difference Jacobian took 1.4-2.2 ms per leg, about half
of a Pi's 50 Hz budget for 5 legs. Numeric IK therefore cannot become the
default for any chain until a **measured per-leg time on the robot
computer** is recorded and passes a budget test.

### 3.3 `sim/model_index.py` (numpy + MuJoCo; step 4)

`ModelIndex(model, spec)` addresses the compiled model by name, never by
position: `.leg_qpos`, `.leg_dof`, `.leg_act` (arrays of shape
`[n_legs, dof]`), `.tool_act`, `.foot_geom`, `.foot_site`, and
`.assert_matches()`, which raises if the model's actuator names differ from
`rm.actuator_order()`.

### 3.4 Packaging

- Add `leg_kin` to `pyproject.toml` `[tool.setuptools] py-modules`
  alongside `rocky_model`.
- A subprocess purity test asserts that `numpy` is not in `sys.modules`
  after `import rocky_model` (`gait/test_robot_spec.py` already does this
  for `rocky_model`).
- `hw_bridge` must not import `leg_kin` on the safety path.

---

## 4. Migration steps, in order, each with the check that proves it

| # | Step | Check that proves it |
|---|---|---|
| **1** | ~~Spec in the loader, derived stations and actuator order in both generators~~ **Done (D053)**, see §7 | `pebble.xml` and the URDF byte-identical; `gait/test_robot_spec.py` (24 tests) + `test_urdf_xacro_is_regenerated`, `test_spec_matches_compiled_links`, `test_urdf_joints_are_the_spec`; `check_urdf_parity` n_lim 20 |
| **2** | **Generators consume the chain.** `build_mjcf.leg_xml` / `actuators_xml` and `generate_urdf.leg_macro` / `build_xacro` take bodies, joints, axes, offsets and kp/kv/armature from the spec and the `actuators.*` blocks. `rocky.ros2_control.xacro` (a hand-written yaw/hip/knee/claw macro) becomes generated. | `pebble.xml` and `pebble.urdf.xacro` byte-identical. `ros2_control` gets a one-time reviewed diff against the hand-written file, then its own staleness test. `test_spec_matches_compiled_links` still passes. Without this step an ankle added to the chain would validate but never reach MJCF, URDF or ROS. |
| **3** | **Close the paths that fail open.** None changes behaviour today, but each would fail silently or dangerously on a topology change:<br>• `pebble_feasibility` `FAIL_CODES` built from `LEG_JOINTS`; `divmod(k, 3)` → `divmod(k, n_dof)`<br>• `check_urdf_parity`'s `n_lim != 20` → `len(rm.actuator_order())`; its joint list and random-q bounds from the spec<br>• the mock (`make_pebble_mock`) takes ids from `rm.id_table()`; its hands at `15 + i` collide with legs at 6 legs<br>• `driver/rocky_driver/robot.py`'s broad `except Exception` around the limits import → `except ImportError`, so a `RobotSpecError` is not swallowed into the 3-joint `_FALLBACK_LIMITS_DEG`<br>• `hw_bridge` refuses to start if any id in `id_table()` lacks a NaN guard or rate limit<br>• `build_mjcf.py` and `generate_urdf.py` fail when `mass_budget.json` is missing (today both fall back to hard-coded fallback constants, the original hand budget); explicit `--allow-hand-budget` for bootstrap | Existing suites pass unchanged. New tests: a synthetic `LIMIT_ANKLE` counts as a fail; the mock gives unique ids for a 6-leg fixture; a malformed params raises through `soft_limits_deg()`; the bridge refuses an uncovered id |
| **4** | **Index actuators by name everywhere.** `ModelIndex` replaces every positional slice (`git grep -n 'ctrl\[:15\]\|ctrl\[15:20\]\|nu >= 20'`): the playground, both RL envs, `rl_common` (`motor_torque`, `ThermalProxy`, DR), `sim_lidar`, the audits and evaluators, `sim/cockpit.py` and `harness/sim_backend.py`; `servo_model`'s `n=15` default becomes per-joint vectors from each joint's servo | Guard test: the compiled model's actuator names equal `rm.actuator_order()`. `test_rl_envs`, `test_playground_guards` and `gait/test_reflex_fallen` unchanged; evaluating `recover6_d052` gives the same success count as before. This step comes before any topology change: a 4th joint would scramble `ctrl[:15]` with no error. |
| **5** | **Checkpoint safety** (§6):<br>• stamp `topology_hash`, joint names and per-joint `q_lo/q_hi` into env_config<br>• unstamped checkpoints default to the 5×3 legacy hash<br>• `check_topology()` refuses on mismatch<br>• recover `action_to_q` reads the stored `q_lo/q_hi` (today nothing reads them back)<br>• strict mode (`ROCKY_STRICT_FP=1`, default for train resume)<br>• re-admission via `eval_recover --accept-robot <fp>`<br>• fingerprint extended to geom solref/solimp/margin, mesh data, `counts_per_rad`, actuator dyntype/biastype and `description_digest()` | Tests with a synthetic topology change: all 5 checkpoints under `sim/runs/` are refused, including the 4 unfingerprinted ones (`recover1`, `recover5_v3_warm`, `robust_fwd2`, `cmd_sample3`). With the fingerprint changed but the topology unchanged (today's case for `recover6_d052`, trained on `ceb63a1254c3`): warn, refuse under strict, re-admitted after `--accept-robot` passes. The fingerprint rotates once by design (a D-number). |
| **6** | **`LegKinematics`** (§3.2) replaces the FK copies: `pebble_feasibility`'s CoM / support model (`com_body`) → `.points`, `pebble_reflex`'s reach check → `.reach()`, `pebble_keyframes`' `reach` bounds. Audit `gait/pebble_pose_solver.py` for another copy. | Analytic path bitwise equal to the old `leg_ik`/`leg_fk` on a grid; `fk(ik(p)) == p`; numeric IK equal to analytic within 1e-6 mm on the 3-DOF chain and stays knee-down; a 4-DOF fixture with `foot_pitch` gives continuous q along a straight foot path (no branch jumps); a per-leg timing test records ms/leg |
| **7** | **Materials and mass layout (explicit D-number).** `mass_audit` and `print_estimate` read `materials` / `part_material` / `part_fill`. This fixes the PETG parts weighed as PLA. One `sim/body_layout.py` owns the primitive mass layout (coxa box, the 0.7/0.3 tibia split, `M_PRONG`, torso 75/25), which today is copied in `build_mjcf.py`, `generate_urdf.py` and the feasibility CoM model. | `check_urdf_parity` compares CoM **per link**, not only total mass. `mass_budget.json` diff reviewed. Fingerprint rotates; feasibility, shove envelope and audits re-run and recorded in `decisions.md` with a retrain/audit plan |
| **8** | **CAD sync checks** (§5), in parallel with 3-7. **The tibia-stack item lands before the first leg is assembled.** | `cad/check_params_parity.py` in `run_all_checks` MODULES; `rocky.sh cad-check` green |
| **9** | **Operations and docs.** `rocky.sh regen` runs `validate` → mass_audit → build_mjcf → generate_urdf (+ ros2_control) → check_urdf_parity → pytest model_consistency, then prints the fingerprint and topology hash before and after and every checkpoint as ok / re-eval / refused. Update SIM_GUIDE §6, `DESIGN_CHANGE_GUIDE.md` §0 and RL_GUIDE §2 and §7. | Running `rocky.sh regen` on a clean tree gives an empty `git diff` |
| **10** | **Only when a topology change is actually made:**<br>• the driver/bridge per-joint table from `id_table()` (`robot.py`'s limits, families and calibration; `hw_bridge`'s `JOINTS`, calibration regex and per-id `Family`)<br>• reflex, gestures and keyframes generalised from `(N, 3)` (the `reshape(N_LEGS, 3)` calls and the `n_con >= 4` crouch rule in `pebble_reflex`, its 3-vector fallback, `pebble_gestures2`, `pebble_keyframes`, a stored-keyframe migration)<br>• `ArmGait`'s `n == 4` special case (`pebble_gait.py`) → n-based<br>• `duty` defaults to `1 - 1/n_legs`<br>• the RL obs builder generated from topology (a new obs version); `OBS_DIMS` computed, legacy literals kept only to decode old checkpoints<br>• an `overrides` consumer (per-leg `servo_for` in `actuator()`)<br>• the CAD deck and shell as an n-gon (a redesign) | Per-change fixtures for the topology in question; the full suite passes; a fresh policy trains |

`harness/sim_backend.py` and `sim/cockpit.py` also slice ctrl by position;
they migrate to `ModelIndex` in step 4. A `GET /api/robot` model panel
(`to_json()`, fingerprints, checkpoint compatibility) is a later cockpit
item; the cockpit already gets checkpoint refusals through its existing
error path.

---

## 5. CAD sync rules

### 5.1 What each change triggers

| Change | Follows automatically | Needs a manual step or new check |
|---|---|---|
| L1 / L2 | `leg_frame.py` HIP_TF/KNEE_TF → cad-check → mass_audit → MJCF/URDF → parity | — |
| L3 | Chain, gait, MJCF, URDF (via anchors) | **Nothing in CAD owns L3.** The tibia-stack check below compares CAD to `l3_tibia` and prints the tube cut length. |
| `hip_axis_z` | Chain, gait, MJCF, URDF | Assert it equals the yaw-stack height leg_frame derives (`Z_YAW_TOP`) |
| Servo swap, same case | Physics, limits-derived speeds, mass (mass_audit follows `leg.servo`), kp/kv (step 2), protocol | Fingerprint rotates; re-evaluate checkpoints (§6) |
| Servo with a new case | — | A new CAD geometry module named by `actuators.<k>.cad`. `servo_st3215.spec()` reads `P[f"servo_{servo_for(joint)}"]` and raises when no geometry block exists, instead of building ST3215 cups for a different case. |
| Material | mass_audit and print_estimate via `part_material` (step 7) | A check fails on any part with no material or fill entry, instead of a silent 0.6 |
| Leg count | Spec, generators, gait stations | Deck, shell and stand redesign; the parity check fails until it is done |

### 5.2 `cad/check_params_parity.py` (new, in `run_all_checks` MODULES)

- **Tibia stack:** sums `part_tibia` BOSS_H + SOCKET_DEPTH, the tube cut,
  SEA, `part_hand` HUB_H/CONE_LEN and `part_footpad` CROWN, and compares the
  total to `l3_tibia` within ±1 mm.
- **Tube cut length:** prints it. None exists anywhere in the repo today,
  and the first leg build needs it.
- **Foot geometry:** `part_hand.CONE_TIP_R == leg.foot.tip_radius` and
  `part_footpad.CROWN == leg.foot.pad_crown`.
- **Deck:** station radius and count equal `robot.legs` (`part_deck`
  derives both from params; the check pins it).
- **Literals:** checks the literal 110/100/118/185/140 in `part_shell`,
  `part_dock`, `part_bench_jig` (the knee groove at x = 140 = L1 + L2) and
  `part_smallwins` against params. A first pass, before those files read
  params directly.
- **Dock:** heights vs `gait.body_height` and stance radius (`part_dock`).
- **Sweeps:** CAD sweep ranges must cover `joints.pos_deg`. The sweeps in
  `part_coxa`, `check_assembly`, `part_tibia` and `check_interference`
  become driven by `joints.pos_deg`.

`check_interference.py` is in `run_all_checks` MODULES since D059 (28
modules); replacing the list with glob discovery lands in this step.

### 5.3 Independent witnesses

Once one spec drives both generators, MJCF-vs-URDF parity mostly proves
that the two generators agree with each other. These witnesses stay
independent and are never rewritten to read the spec:
- the closed-form `pebble_gait.leg_fk` / `leg_ik` (moved verbatim, not
  regenerated)
- the CAD-measured HIP_TF/KNEE_TF from `cad/leg_frame.py`
- the committed `pebble.xml` body positions and axes, pinned by
  `test_spec_matches_compiled_links`

**CI is the end-to-end gate:** regenerate everything, then
`git diff --exit-code`. Locally, `rocky.sh regen` plus the staleness tests
cover the same ground.

---

## 6. RL and fingerprint rules

| Change | Robot fingerprint | Topology hash | Policies |
|---|---|---|---|
| L3 ±5 mm, servo swap, material, friction, masses | rotates | unchanged | **Warn** by default; **refuse** under strict mode (`ROCKY_STRICT_FP=1`, the default for train resume) and for the righter on the safety path. Re-admitted by `eval_recover --accept-robot <fp>`, which re-runs the recorded success baseline and writes an accepted-fingerprints sidecar. Recover checkpoints also flag an action-range change via the stored `q_lo/q_hi`. |
| Ankle, 6th leg, actuator reorder | rotates | changes | **All refused**: righter, eval_*, train resume, the cockpit. There is no warm start: obs/act dimensions change, as the 39→57 history (B34) showed. |

Rules:
- A checkpoint with no stamp is the 5×3 pentapod, the only topology that
  ever existed. The unfingerprinted checkpoints (`recover1`, the default
  righter, among them; `rl_common.check_fingerprint` returns 'unknown' for
  them) are therefore refused after a topology change instead of running
  silently.
- Refusing on fingerprint mismatch ships **together** with
  `--accept-robot`. Otherwise the first geometry change under strict mode
  locks out the cockpit's righter with no documented way back.
- After a topology change, the FALLEN path has no learned righter until
  one is retrained; FALLEN then holds its pose and relies on the D048
  stall ramp.
- Extending the fingerprint (step 5) rotates it once, deliberately, and
  gets a D-number.
- **Gait retune goes through the budget, not RL.**
  - `WaveGait.budget` and `pebble_feasibility` (speed classes,
    THERMAL_LOAD, support margin) re-derive the envelope from the new servo
    and lengths.
  - Re-run `audit_gestures`, `audit_righter` and `shove_envelope`, and read
    the new command envelope.
  - The literal command scales in `rocky_env.py` (the `/60, /60, /0.6`
    observation scale and the `--cmd-sample` ranges) come from that
    envelope.
  - The handoff thresholds (`rocky_recover_env.py` `HANDOFF_*`, metres)
    scale with stance height from the spec.

---

## 7. Step 1, as shipped (D053)

Scope: the `robot:` spec in the loader, derived stations and actuator order
in both generators, and pinning tests; output byte-identical, nothing else
changed. Delivered:
- the anchors (§2.1) and the `robot:` block (§2.2) in `cad/params.yaml`,
  `params_rev` unchanged;
- the §3.1 API in `gait/rocky_model.py`;
- `pebble_gait.N_LEGS` / `STATION_DEG`, `build_mjcf` (`leg_xml`,
  `actuators_xml` over `rm.actuator_order()`) and `generate_urdf` read
  stations, leg count and actuator order from the spec; the MJCF text stays
  `euler="0 0 162"` and unwrapped (`378`);
- `gait/test_robot_spec.py` (24 tests: today's spec equals the old
  literals; a generic chain FK equals `leg_fk` within 1e-9 mm; the actuator
  order equals the committed `pebble.xml`; negative validation; the 6-leg
  and ankle fixtures; a params without `robot:`; cache isolation; numpy
  purity) and the staleness / per-link tests in
  `sim/tests/test_model_consistency.py`;
- `sim/pebble.xml` and the URDF byte-identical, topology hash
  `7f066d9bd8c0`.

Deviations from the design: `summary()` keeps its servo line and the
description line is `robot_summary()`; warnings are returned, never printed
by the loader; the helpers take `spec=`.

**Tools are data too (D056).** The robot's tools follow the same rule: one
registry, `harness/capabilities.py`, generates every surface (the local
brains' schema, the cockpit's list, the MCP server's list,
`docs/TOOLS.md`). Its snapshot carries `envelope` from
`WaveGait().max_command()` on `cad/params.yaml` (speed 0.0455 m/s, turn
0.246 rad/s, step 24 mm, goto 0.045 m/s) and `robot` from
`rocky_model.robot()` (legs, joints per leg, stations, 20 actuators,
topology hash, `params_rev`). The goto and move texts take their distance
from `envelope.goto_max_m` / `move_max_m`: for `move` it is the hard limit
(a longer move is refused); for `goto` it is the advised reach ("keep
targets within ~1.5 m"), not a refusal (the cockpit refuses a goto target
only beyond 3 m, `cockpit_brains.GOTO_MAX_M`). Both are still the constant
1.5 m (inside the ~1.8 m that goto's 40 s cap covers at ~0.045 m/s), not
derived from params; the goto text's "~0.045 m/s, 40 s cap" and
the other numbers in the descriptions (compose_gesture's joint ranges,
find_object's 0.1-0.4 m steps) are literals in the registry. A params
change that moves the envelope or the topology changes the snapshot and
makes `docs/TOOLS.md` stale, and CI refuses a stale page the way it refuses
a stale `pebble.xml`.

---

## 8. Risks to keep in view

- **Analytic IK shape.** `leg_ik` is exact only for yaw about +z plus two
  parallel −y pitches, with y = 0 offsets, a +x foot and the knee-down
  branch; `ik_ok` rejects any q whose last dimension is not 3. The fast
  path must be chosen by the structural predicate plus the cross-check
  test.
- **Numeric IK cost and redundancy.** It needs the analytic Jacobian, a
  warm start, a branch constraint, a redundancy rule and a measured Pi
  budget before it can be the default.
- **Symmetric-station assumptions.** These all assume equal CCW spacing and
  stay symmetric-only until audited: `WaveGait`'s
  `phase_off = arange(N)[::-1]/N`; `pebble_manip_adjacent`'s ARM_LEGS ("any
  pair works by symmetry") and its lean math; the playground touchdown
  gate; the CAD 72° sectors.
- **Positional ctrl slices** stay a silent scrambler until step 4. Steps
  1-3 must keep leg-major actuator order; the actuator-order test is
  load-bearing.
- **Safety path.** Until step 3, a spec error in the driver could fall back
  to 3-joint literal limits. Until step 10, the bridge knows only
  yaw/hip/knee.
- **Mass and material changes** (step 7) shift feasibility margins, shove
  thresholds and RL dynamics. They are an explicit decision with a
  retrain/audit plan, not a drive-by refactor.
- **First leg build.** It will find any tibia-stack drift. Land the §5.2
  tibia check before assembling.
