# Simulation guide

The developer reference for Pebble's MuJoCo simulation: how the model is
built, the control stack it runs, the playground, the cockpit's internals,
the MCP / language-model loop, worlds, the environment, what each number is
worth, and the sensing that is not built yet (§9). Elsewhere:

- using the cockpit: [COCKPIT_GUIDE.md](COCKPIT_GUIDE.md) (also in the page
  under **?**);
- place recognition: [PLACES.md](PLACES.md); the tool list:
  [TOOLS.md](TOOLS.md) (generated); brain models: [BRAINS.md](BRAINS.md);
- reinforcement learning: [RL_GUIDE.md](RL_GUIDE.md); design changes:
  [DESIGN_CHANGE_GUIDE.md](DESIGN_CHANGE_GUIDE.md);
- the one-off experiments and their record:
  [sim/experiments/README.md](../sim/experiments/README.md).

Commands run from the repo root with the repo venv (`.venv/bin/python`);
`MUJOCO_GL=egl` (or `osmesa`) renders offscreen, `glfw` opens a window. The
outputs quoted were re-run on 2026-10-07 on `deae868522cc`, D064's belly
model, against D063's `87215110e9c4` (BUILD_LOG 9q: every experiment the
docs quote, old against new, with a third tree whose belly does not collide
to separate the belly from the mass). The current fingerprint `965f4f70e5d1`
is the 9q review's shell relief (`c3e82f13b671`, 0.2 g lighter) plus the
deck's north tray-tab holes back to Ø2.8 (+0.02 g, B155); `run_sim`, the
torque audit and the test suites were re-run on both. The body leveler's
terrain bench and the lip grid (D065, §3, §8) ran on `965f4f70e5d1` on
2026-10-09.

## 1. How it works

**One model, generated.** `sim/build_mjcf.py` writes `sim/pebble.xml` from
two inputs: `cad/params.yaml` (leg lengths L1/L2/L3 = 45/95/135 mm, body
circumradius 110 mm, hip axis height, the servo and joint identity, the
`robot:` description) and `sim/mass_budget.json` (link masses summed from the
CAD tree's STL volumes by `sim/mass_audit.py`, D039). Five identical legs at
72° stations, three hinge joints each (yaw, hip, knee), a claw per leg, a
foot sphere per leg, and the torso. Since D064 the torso's mass, CoM and
inertia are an explicit `<inertial>` from the budget (1555.0 g at (−2.21,
−4.63, −3.18) mm: `mass_audit`'s pose table, every torso part weighed from
its STL and posed in the body frame), its two cylinders carry no mass, and
under the deck hangs a massless **belly** the floor and the legs can touch,
from `rocky_model.belly_boxes()` / `belly_posts()` (params and the part
modules; `cad/check_sim_mirror.py` holds them to the exported parts):
`belly_tub`, the keel tub's box (x −97.5…99.9, y −47.4…7.4, z −55.4…−18.0
mm); `belly_shelf`, the hub shelf's plate box (x −56.2…53.95, y 12.2…64.3,
z −54.4…−10); and its three Ø14 posts `belly_shelf_post0..2` at (−50, 34),
(50, 34), (26, 64), z −39.3…−10. The tub's real protrusions (the latch boss
and door ear, the pilasters, the riser clip) are not in the sim (B142). 2.843 kg
compiled, equal to the budget. The URDF in `ros2/` is generated from the same inputs,
and `sim/check_urdf_parity.py` proves the two agree (20 joints, FK within
0.05 mm over 60 random configurations, identical mass, foot sphere and
actuator limits). CI regenerates both and fails on any diff.

**The servo, from the datasheet and one rule of thumb (D052, D052a).** Every
number that describes a servo or a joint lives in `params.yaml`
(`actuators`, `joints`, `gait`, `reflex`, `sensing`) and is read through one
loader, `gait/rocky_model.py`; the MJCF, the URDF, `servo_model`, `shove`,
the driver's soft-limit clamp and the cockpit all use it. From the ST3215's
2.94 N·m stall and 4.7 rad/s no-load:

| quantity | value | where it comes from |
|---|---|---|
| joint damping | 0.626 N·m·s/rad | stall / no-load: the servo's torque-speed line |
| leg actuator forcerange | ±2.94 N·m | the stall (peak) torque, the servo's instantaneous limit (D052a) |
| continuous torque | 1.911 N·m | 0.65 × stall, a thermal budget judged over time, not a clip: `rl_common.ThermalProxy` (RL envs: derates a hot joint; playground and cockpit: accounting only) and `THERMAL_LOAD` in `audit_gestures`. 0.65 is a guess, VERIFY |
| loaded / free / hard speed | 3.0 / 4.0 / 4.7 rad/s | 3.0 ≈ continuous / damping (a joint held to the continuous torque tops out at 3.05, measured 3.06: an upper bound, not a speed at which it also carries a load); 4.7 = no-load, which an unloaded MJCF joint reaches (measured 4.71) |
| thermal proxy | heat += ((\|τ_e\| / stall)² − 0.65²)·dt, ≥ 0; budget 54 | = (0.85² − 0.65²) × 180 s (`thermal_trip_frac` / `thermal_trip_s`): trips after 180 s at 0.85 × stall, 93.5 s at stall, never at ≤ 0.65 (the driver mock's 70 °C cut: 167 s / 88 s; at 0.80 both 248 s). A tripped joint's forcerange ramps 2.94 → 1.911 N·m as heat goes 54 → 64.8 (RL envs only); `thermal_heat0` warm-starts an episode (off by default). 0.85 and 180 s are VERIFY |
| claw (SCS0009) | forcerange ±0.23 N·m, damping 0.0242 N·m·s/rad | its stall, and stall / no-load (0.23 / 9.5) like the legs; every SCS0009 number is a guess, VERIFY |
| joint soft limits | yaw ±40°, hip −70…90°, knee −150…−20°, claw 0…55° | the CAD-validated sweeps (D008) |
| foot | 6.5 mm sphere whose surface is the gait's IK foot point (site `foot_tip{i}`); μ 0.8, torsional 0.005 m | μ is a TPU-on-tile guess, VERIFY |

τ_e = actuator force − damping × qvel is the motor's *current* term: the raw
force counts the back-EMF voltage as heat (a free yaw swung at 4.7 rad/s
reads 0.87 × stall RMS of force but 0.22 of current, and the 45 mm/s gait's
mean load reads 0.184 × stall as force but 0.094 as current). Heat goes with
τ², so `audit_gestures` judges each joint's RMS τ_e / stall (> 0.65 warns
`THERMAL_LOAD_WARN`, > 0.85 fails `THERMAL_LOAD`): a 50 % duty of stall is
mean 0.50 but RMS 0.71 and trips the proxy at 691 s (1 s at stall, 1 s at
rest, pinned in `sim/tests/test_rl_envs.py`). Before D052 the
actuators were clamped at stall with joint damping 0.05, which let the gait
ask for 7.4 rad/s and turn 50 % further than the servo could; D052's first
cut clipped at 1.911 N·m, which capped every joint at 3.05 rad/s, and D052a
made the clip the peak and moved the continuous budget into the thermal
model. `sim/model_fingerprint.py` hashes the robot (not the world):
`965f4f70e5d1` since the deck's north tray-tab holes went back to Ø2.8 (B155,
+0.02 g of deck: the torso still 1555.0 g, its inertia moved in the 8th
decimal); `c3e82f13b671` for the 9q shell relief (torso 1555.2 → 1555.0 g,
robot 2843.0 g); `deae868522cc` for D064's body layout (the belly geoms and the
torso `<inertial>`: torso 1439.8 g at z 24.5 → 1555.2 g at (−2.21, −4.63,
−3.17), robot 2727.7 → 2843.2 g; `c10b970d2bb2` was its provisional
budget); `87215110e9c4` for D063's coxa-fork side cheeks (+5.5 g per leg;
robot 2727.7 g), `1f953c89f979` after D062's femur deck + 8 mm rails
(+7.6 g per leg, robot 2694.6 g), `7d376178fe27` after D059's CAD change
(torso 1448.9 → 1435.2 g), `ceb63a1254c3` for the D052a model before it,
`5a32f772ca99` for the 1.911 N·m clip (`4debe4e83893`, the peak clip with
the old claw damping, was never trained on). Every RL checkpoint records the
fingerprint it was trained on.

**The control stack is the real one.** The sim has no controller of its own.
It imports `gait/pebble_gait.py` (closed-form IK, the five-phase `WaveGait`
that turns a body-frame velocity command into joint targets; T 2.0 s, step
height 24 mm, duty 0.8 since D052), `gait/pebble_reflex.py` (the `ReflexSupervisor`
state machine: NORMAL → PLANT → BRACE → RECOVER for safe-stops and shoves,
FALLEN → RIGHTED for tumbles, D034/D042) and `gait/pebble_watchdog.py` (the
progress watchdog's escalating retries, D023; its stages are derived inside
the D052 envelope since D060). The bench scripts and the ROS bridge feed the
same modules to real servos. Two D052 pieces sit in front of them:

- **`WaveGait.budget()`** fits every velocity command into the gait's
  envelope (34.2 mm/s at any heading, 0.185 rad/s turning in place; 45.5 /
  0.246 before D063's soft-landing swing) by uniform scaling, so heading and
  curvature survive; walk, teleop, goto, the residual walker and the ROS
  `gait_node` all go through it. Since D063 the budgeted command reaches the
  gait through `CommandSlew` (25 mm/s² on the fastest foot, then a 0.2 s
  lag): standing to 34.2 mm/s takes 2.36 s at no more than 3.38° per 20 ms
  tick, and a stop is never eased.
- **`gait/pebble_feasibility.py`** checks a motion before it runs: guarded
  joint limits, peak speed per class (loaded / free / hard), steps at entry,
  exit and phase boundaries, ≥ 3 feet down, the CoM margin (the MJCF's own
  segment masses, and since D064 the torso's mass and CoM from the budget,
  B138: 38.5 mm at the envelope, 41.4 on D063's centred torso), slip and
  self-contact (a leg inside the belly is a `SELF_CONTACT` naming the belly
  geom). Code gestures, keyframe files, the studio, gait presets and
  the brain's `compose_gesture` all go through it; `sim/audit_gestures.py`
  runs it plus a physics pass over everything (19 rows, 0 FAIL, 0
  self-contact samples, on `deae868522cc`).

The playground's always-on guards (§3) wrap both, for every command source.

**Sensing in the sim.** `sim/sim_imu.py` gives gravity (from
`model.opt.gravity`, so a slope reads as a slope) and gyro with optional
noise, latency and (D065) a mount error; the playground reads it with the sim
time, so a latency applies (before D065 it never did; its default IMU is
ideal); `perception/contacts.py` is the one foot-switch function:
contacts against the world only (a foot pressed into its own body does not
count), 2.0 / 1.0 N close / open hysteresis. `perception/` also supplies a
legged-odometry EKF, an ICP scan matcher and the cliff detector;
`sim/sim_lidar.py` casts a 2D ray fan (a plane ~0.18 m above the floor) that
the harness exposes as `scan_summary`. `sim/scenes.py` holds the scene
helpers live code shares with the experiments (the cliff world, the rubble
field, the push harness's spawn, the IMU noise model, the 1.5 N contact
switch the harness sim uses, the fallen demo's shove).

### Worlds

The playground, the harness's in-process sim and the experiments use three
built-in worlds: `flat`, `cliff` (a table-edge island for the VOID reflex)
and `room` (walls, pillars, a crate, for lidar and patrols), by flag or
`ROCKY_WORLD`. The cockpit builds worlds from a spec (`sim/world_builder.py`)
with presets `flat`, `room`, `cliff`, `obstacle course`, `rubble field`,
`rough terrain`, `stairs`, `slope 8 deg`, `icy floor`, and the place bench's
rooms (`room a` to `room d`, `room a + chair`, `room b + chair`,
`room c + chair`, `room c - ball`), plus saved and random courses. Every world
geom has `priority="1"`, so the world's friction is what a foot contact gets
(the icy floor really is 0.35; before D052 the feet's own μ won); ramps and
stairs have a far side unless `"far": "drop"`; random courses keep every
object ≥ 0.3 m from the spawn. What the lidar sees in each preset:
[PLACES.md](PLACES.md) "Weak spots".

**What the numbers are worth (the honesty box).** Masses are from CAD; the
servo identity is datasheet stall and no-load plus a guessed thermal
fraction; friction, the foot-switch forces and every observation-noise level
are guesses; the leg links' CoMs and inertias are primitive shapes (the
torso's come from the CAD pose table since D064, but its belly is two boxes
and three posts, not the tub's chamfers), the SEA spring is rigid, and there
is no gear backlash or battery sag (B33). The servo realism layer (50 Hz
hold, 20 ms latency, 4096-count goals) is on by default in the playground and
cockpit. The sim is excellent for logic (does the reflex trip, does the
gesture reach, does a gait tweak break a joint limit or a speed budget) and
directional for dynamics (shove envelopes, stability trends). Absolute
numbers are "±real-robot-TBD" until a real servo answers on the bench (D017
re-baseline, B32). D052 made the sim harsher, not calibrated; the shove
envelope and the other known gaps are in §8.

## 2. First run

```bash
.venv/bin/python sim/build_mjcf.py                    # sim/pebble.xml from params + masses
MUJOCO_GL=egl .venv/bin/python sim/run_sim.py         # headless walk, strafe and turn, a few seconds
```

`run_sim.py` writes its video and contact sheet to `sim/out/run_sim/`
(`--out DIR` elsewhere; `--out sim` regenerates the committed
`sim/pebble_sim.mp4` on purpose). Expected on `deae868522cc`,
`c3e82f13b671` and `965f4f70e5d1`:

```
body height: mean 117.1 mm (target ~118, rigid ideal servos), std 0.19 mm, min 116.7
tilt: mean 0.27 deg, max 0.42 deg
walk +X displacement: 189 mm (commanded ~198 mm at the budgeted 34.2 mm/s; asked 45), lateral drift 0 mm
turn in place: 41.7 deg at the budgeted 0.185 rad/s (commanded ~40 deg; asked 0.6 rad/s)
fell over: no
```

D063's model gave min 116.8, tilt 0.26 / 0.41, turn 41.8°; the run is
identical with the belly switched off (the belly never touches in a walk),
so the change is the mass and CoM.

The walk asks 45 mm/s for 6 s and the budget fits it to 34.2 (D062, before
the soft-landing swing: 247 mm and 55.4° at 45.5 mm/s / 0.246 rad/s). The
script exits non-zero on a fall or when the walk or the turn leaves its smoke
band, so CI catches a regression. The
pre-D052 numbers (264 mm, drift −37 mm, tilt max 1.03°) were the flattering
servo.

## 3. The playground: drive it live

```bash
./rocky.sh play                                                # viewer window + REPL (flat world)
./rocky.sh play --cliff                                        # the table-edge world
MUJOCO_GL=egl .venv/bin/python sim/playground.py --script "walk 45; wait 3; stop; quit"   # headless
```

One process runs physics with the real gait and reflex supervisor; you type
commands at the `pebble>` prompt while it runs, and the window accepts keys.

**Keyboard teleop** (in the terminal, at an empty prompt): `↑` / `↓` (or
`Shift+W` / `Shift+S`) ±15 mm/s forward per tap, `←` / `→` (`Shift+A` /
`Shift+D`) ±15 mm/s strafe, `Shift+Q` / `Shift+E` ±0.12 rad/s turn, `SPACE`
the D034 safe-stop (PLANT → BRACE → planted idle; it also ends a gesture),
`Shift+G` wave from a planted standstill. There are no key-release events,
so a command persists until you change it; the caps are the gait's envelope
(`WaveGait.max_command()`: 34.2 mm/s, 0.185 rad/s), and every nudge goes
through the same guards as `walk`. In the viewer window only the **arrow
keys** drive: every letter there is one of MuJoCo's own render toggles (W
wireframe, S shadows, A auto-connect, D static bodies, G fog, Q camera
frames, E equality constraints) and SPACE pauses the viewer, and those fire
alongside ours (D048). The mouse orbits and zooms; Backspace resets the
physics state, so avoid it.

**REPL commands** (also headless via `--script "a; b; c"`):

| command | what it does |
|---|---|
| `walk VX [VY] [WZ]` | body-frame velocity (mm/s, mm/s, rad/s), fitted into the envelope by `WaveGait.budget`: `walk 60 0 0.5` runs as (18.77, 0, 0.16), "scaled to 0.31x by the lift-speed budget", and `show` prints `cmd asked -> cmd run` |
| `stop` | the D034 safe-stop; also blends a running gesture out |
| `gesture NAME` | `jazz_hands fist_bump beckon wave bow look_around shake sit turn_in_place sidestep`, or a saved keyframe gesture (`gait/gestures/*.json`). Only from a planted standstill (NORMAL, no walk, no goto); blended in and out at ≤ 3 rad/s; the reflex keeps watching, so a fall drops the gesture |
| `gait NAME` · `gait save NAME` · `gait list` | load a preset (`gait/gaits/NAME.json`; `default` = params.yaml, `legacy_d050` = the old T 1.6 s / 32 mm gait, which has no envelope under the D052 budget) · save the current gait · list them |
| `check [NAME]` | the feasibility report: no NAME = the gait at the current command and at the envelope corners (PASS/FAIL, peak rad/s and which joint against 3.0 / 4.0 / 4.7, support, CoM margin); NAME = that gesture |
| `clear` | release a latched void and a latched safe-stop; `nothing latched` when there is none. Does not resume the old walk |
| `probe on` / `probe off` | the stance contact probe (feet feel for the floor, D050) |
| `level on` / `level off` · `level` | the body leveler (D065, below; off by default) · its state: the plane's slope, the tilt error, the raise per foot, SATURATED, the hold |
| `say WORD` | play a chord-speak sample (`aplay` / `afplay` / `ffplay`) or print it |
| `set PARAM VALUE` | live-tune `gait.T gait.h gait.R0 gait.duty gait.hstep reflex.trip reflex.stall_s reflex.fallen_max_s gate.wait servo.on servo.hold_hz servo.latency_s servo.rate_rad_s servo.quant` (and `servo.load_derate`, §3c), and the leveler's `level.tau_s level.filter_s level.deadband_deg level.raise_mm level.rate_mm_s level.swing_rate_mm_s level.tilt_max_deg level.min_contacts level.gyro_calm` (range-checked, `raise_mm` ≤ 30, kept across a respawn). Phase-continuous, so feet do not teleport; gait values are validated (T 0.4–10 s, duty 0.5–0.95, step 0–80 mm, stance reachable) |
| `show` | gait params, reflex state and trips, pose, asked → run command, probe per leg, the servo model, sim time |
| `push FX FY [DUR]` | shove the shell rim: peak N, N, half-sine over DUR s (default 0.4); prints the impulse in N·s and bodyweights |
| `record on` / `record off` | an offscreen clip to `sim/playground_clip.mp4` |
| `rl` · `righter NAME` · `righter off` | the checkpoint table · hot-swap the self-righting policy (`sim/runs/NAME/latest.pt`) · the analytic ramp only |
| `help` / `help rl` · `wait S` · `quit` | the command list / the RL hooks · (scripts) let S sim-seconds pass · exit |

**Always-on guards (D052).** Every command source (`walk`, the keys, the
cockpit's goto, the residual walker, the hardware mirror) goes through the
same checks, in the Playground itself:

- **Void guard.** A stance foot that probes 30 mm and finds nothing while
  walking is a void. The robot backs off (the approach played backwards: every
  foot returns to a foothold it already stood on, 0.6 of a gait cycle),
  safe-stops and latches the world bearing; any command with a component
  toward it is refused (`blocked: void at N deg — clear to release`), and a
  goto has that component projected out, until `clear`.
- **Touchdown gate.** A stance foot that has not felt ground 140 ms
  (`GATE_TICKS` = 7 ticks at 50 Hz, `sim/playground.py`) after its commanded
  touchdown runs the gait clock back to that touchdown and holds it until the
  foot finds ground or the void guard fires. The wave gait lifts the next leg
  the instant one lands, so at a 10–20° approach a leading foot over the edge
  used to lose its neighbour before its probe finished. Seven ticks because
  the first switch contact lands 2–4 ticks after the commanded touchdown on
  flat ground and rubble (≤ 8 on rough terrain and stairs); any threshold
  ≤ 5 held the gait on almost every flat step (29–31 holds in 15 s). The
  clock runs back at most 3× real time, less when the gait's own peak joint
  rate would push a joint past its budget (1.35× at the 34.2 mm/s envelope
since D063; 1.12× at D052's 45.5), so the joints
  that are not probing stay ≤ 3.94 rad/s on stairs and rough ground (D052a).
  `set gate.wait 1` is the **careful walk**: every touchdown waits for its
  switch (about 17 % slower on every surface, safer at edges; off by
  default).
- **Trip escalation.** 3 gyro trips within 5 s latch a safe-stop; velocity is
  ignored until `clear`.
- **Target stream.** Every joint target is rate-clamped at 4.7 rad/s (claws
  9.5), a non-finite target is dropped with the last good one held, and the
  result goes both to the sim's servo model and to the real bus.
- **Thermal accounting.** Each leg joint's heat from τ_e (the proxy with the
  derate off): notes at 50 % of the budget and at the trip; a tripped joint
  refuses sim → robot until it cools.
- **Kinematic height** from the IMU and the joint angles (what the robot can
  compute) is what the supervisor gets; world z is for the HUD.

**The body leveler (D065, lands off: `params.yaml` `level.enabled: false`).**
Before it nothing levelled the body: the gait stands its feet on one
body-frame plane, so on an 8° slope the body sits at 8.06° and a 20 mm stone
under foot 0 leaves 3.88°. `gait/pebble_level.py` `BodyLeveler` (numpy only,
the file the Pi will run) rides on the supervisor
(`ReflexSupervisor(gait, leveler=...)`; without one the supervisor is the
D064 one bit for bit). The law, ticking at 50 Hz on a fixed schedule of the
sim time (a call within half a call period of the slot ticks it, so a 50 Hz
loop with timing jitter gets one tick per call):

1. Tilt error e = (u_x/u_z, u_y/u_z), u = −grav from the IMU; a 0.2 s low-pass, then a 0.5° radial deadband (a straight flat walk peaks at 0.40° of true tilt, so an ideal IMU reads exactly zero there; a turn, a stop's brace or a shove goes past it, and the plane it integrates then stays: the deadband never unwinds it, see the W1 rows below).
2. The plane slope P += e·dt/τ (τ 0.6 s), capped at 20 mm/s at the 185 mm stance radius; it integrates only in NORMAL with ≥ 3 switches closed, gyro < 0.9 rad/s, tilt ≤ 30° and no hold.
3. Offsets z_i = P·(x_i, y_i) + c, + = foot HIGHER (the opposite sign of `probe_dz`); c = −min over the five footholds, so every foot is **raised** 0–30 mm and none lowered; a plane that spans more is scaled down (saturated): full leveling to 4.87° for a slope along x and 5.12° along y, partial past it; the body sinks instead, by c: 13.4–16.6 mm on a full window, by the slope's heading (the slope stands' measured height error is below).
4. The plane is evaluated at each foot's own xy (`WaveGait.level_xy`): a stance foot where it stands, a swing foot moving from its lift-off to its touchdown point over swing progress 0.30–0.70.
5. Each leg follows the plane at ≤ 20 mm/s in stance and ≤ 40 mm/s airborne, on every supervisor call (the plane moves on the ticks), and holds while a swing foot is inside the 15 mm band; airborne means the foot clears the band + 2 mm over the highest copy's or the plane's ground under its real xy (the band `pebble_feasibility.check` judges).
6. It holds still while a foot feels for the floor (a gate hold, the void phase, `probe_out`, a commanded-stance foot still seeking past its settle window) and in BRACE (whose crouch plane captures the offsets); FALLEN, RIGHTED, a gesture and a respawn reset it; a gait change keeps the plane (its next tick re-fits the window over the new footholds; a reset there dropped the feet up to 30 mm in one step); mirroring sim → robot runs it off; `level off` slews back at the caps.

What it leaves of the probe and the void guard: the offsets are added to the
gait's foot targets before `last_feet_raw`, so the probe's measured depth
(`last_feet_raw.z` − FK(q_meas).z) and `probe_out` stay relative to the
target, and `probe_dz` is untouched (0 on a flat walk). Raising only keeps
the probe's whole 30 mm of lowering, so a void (seek + `probe_dz` ≥ 30, no
contact, the foot ≥ 26.5 mm down) means what it meant; the swing-low check
measures each foot from its own ground (−h + z_i). The leveled plane needs a
slower walk (a swing foot spends longer in the loaded band), so
`WaveGait.budget(..., level_slope=)` derates the envelope from the plane's
slope: 34.20 mm/s at 0, 32.48 at tan 5°, 31.42 at tan 8°; an enabled leveler
computes every 0.0025 slope step its window admits up front (`prime()`, 2.8 s
once per process), so the control step only looks one up. What runs is the
derated envelope (`Playground.envelope()`, the cockpit's gait and model panels,
the `check` line); the teleop caps (`V_MAX`) stay the bare one, since they cap
the ask.
It needs a calibrated IMU: on a 1.5° mount error it levels the body to the
sensor, 1.045° off true on flat ground (B159). `Playground(imu={...})` /
`set_imu(...)` give the sim's IMU noise, latency (`grav_sigma`, `gyro_sigma`,
`gyro_bias`, `latency_s`) and a mount error (`mount_deg`, `mount_axis`), with a
`seed`, kept across respawns; `ROCKY_LEVEL=1` (or `0`) overrides
`level.enabled` for every Playground built with the default.

**The terrain bench** (`sim/terrain_bench.py`, D065) is the leveler's ship
gate: stacks S0 (`probe off`, and with it no gate and no void guard), S1 (the
shipped stack), S2 (S1 + `level on`; S3 is reserved for the terrain residual,
B160) under three IMUs (ideal; noisy = `rl_common.OBS_NOISE` + 20 ms latency;
`mount1.5`), on 87 rows: W1 flat (straight walks, a stand, and since the
review a turn in place, a walk + turn with its stop and two rim shoves), W2
gravity slopes, W3 a stone under a foot, W4 box rubble, W5 ramps, W6 stairs,
W7 a shove on a slope, W8 the cliff grid, a cliff on a 5° slope walked to
downhill, across and uphill (the raised feet lead), and `run_sim`. Tilt is the
true attitude against gravity. The record (`sim/out/terrain_bench.json`: 1033
runs in 643 s at `--jobs 6` on `b2bf1b9` plus the wip 4 code, named by its
`code_diff_sha256`; `965f4f70e5d1`; S2 against S1, ideal IMU unless a column
says otherwise; every S0 and S1 run is bit-identical to the `e174f22` record's):

| world | measure | S0 | S1 | S2 | S2 noisy | S2 mount1.5 |
|---|---|---|---|---|---|---|
| W1 straight flat walk ×3 | tilt RMS ° · max offset mm | 0.33 · 0 | 0.33 · 0 | 0.33 · 0 (bit-identical to S1) | 0.35 · 1.68 | 1.31 · 7.45 |
| W1 turn · walk + turn · shove x · shove y | max offset mm (end tilt °) | 0 (0.01 · 0.01 · 0.01 · 0.01) | 0 (0.01 · **2.44** · 0.01 · 0.01) | 0.47 · 11.65 · 0.92 · 0.36 (0.08 · 0.75 · 0.16 · 0.09) | 0.79 · 0.94 · 0.42 · 0.24 | 14.93 · 14.93 · 6.72 · 6.09 |
| W2 stand 5° / 8° / 10° (×3 headings) | end tilt ° | 5.04 / 8.06 / 10.08 | same | 0.07–0.24 / 2.96–3.22 / 4.98–5.24 | 0.12–0.21 / 3.00–3.23 / 5.01–5.24 | 1.46–1.87 / 2.96–3.34 / 4.98–5.28 |
| W2 slope walks (18) | mean tilt RMS ° · family progress, % of S1's | 7.59 | 7.59 | 2.91 · 95.4 % | 2.92 · 95.4 % | 3.46 · 95.6 % |
| W3 stone 10 / 20 / 30 mm, foot 0 | end tilt ° · the stone foot's load | 1.56 / 3.27 / 4.97 · 11.4–12.0 N | 2.18 / 3.88 / 5.63 · 0 N | 0.45 / 0.36 / 0.42 · 0 N | 0.13 / 0.16 / 0.44 · 0 N | 1.12 / 1.34 / 0.48 · 0 N |
| W4 rubble 10–30 mm (15) | mean tilt RMS ° · family progress · min belly clearance mm | 1.54 · 108.6 % · 29.2 | 2.20 · 6121 mm · 31.3 | 1.60 · 100.6 % · 23.5 | 1.54 · 100.9 % · 24.2 | 2.02 · 102.9 % · 16.7 |
| W5 ramps 5° / 10° (6) | mean tilt RMS ° · family progress | 5.20 · 96.7 % | 5.49 | 2.20 · 103.2 % | 2.21 · 102.9 % | 2.67 · 102.0 % |
| W6 stairs 15 / 20 mm (6) | mean tilt RMS ° · tips · family progress | 4.15 · 0 · 92.5 % | 4.72 · 2 | 1.87 · 0 · 102.5 % | 2.20 · 0 · 87.8 % | 2.84 · 0 · 112.8 % |
| W7 8° slope, 25 N rim shove ×2 | tilt RMS ° · peak ° | 8.09 · 12.08 | 8.09 · 12.08 | 3.26 · 5.15 | 3.27 · 5.18 | 3.39 · 5.06 |
| W8 cliff grid (10) · cliff on 5° (6) | fired and held | 0/10 (8 tips) · 0/6 (1 fall) | 10/10 · 6/6 | 10/10 · 6/6 | 10/10 · 6/6 | 10/10 · 6/6 |

No S1 or S2 run fell, none touched the belly, the worst servo load was 0.261
× stall RMS and nothing came near a thermal trip. `run_sim`: walk 189 mm,
turn 41.7°, in band. On the 5° slope cliff S2 fires and holds with the void
foot raised 30.0 mm walking uphill (d180, both speeds) and 11.4 mm across
(d90, walk 45), the tilt after the fire 0.58–1.96° (S1 5.20–11.76). The pass rules and the verdict (it **fails**: P1, P4, P6
noisy, P7, R1 noisy): [decisions.md](decisions.md) D065; the backlog items
B156–B163. On W2 the derate alone puts a saturated plane under P4's line:
32.48 / 34.20 mm/s is 94.97 % (and 94.86 % for a slope along y). MuJoCo's
friction creep adds to it: with every foot planted the robot slides
downhill (S1 standing 24.6 / 39.4 / 49.1 mm in 10 s at 5 / 8 / 10°; a leveled
stance creeps less, 14.2–17.8 at 5°), which flatters S1 downhill (689–691 mm
at 5° against 618–620 on flat) and costs it uphill (544–545), so per row S2
reads 91.6–92.3 % downhill (5° and 10°), 99.3–102.7 % uphill and 94.2–95.8 %
across, where the phase-1 rows (94.2 at 5°, 94.4–94.5 at 10°) miss the line.
On W3 the stone foot carries no load in S1 or S2 (0 N, its switch open over
the last 2 s; S0 11.4–12.3 N): the probe's four floor feet reach down and
hold the body up (height error +9.6 mm on the 20 mm stone, S0 −1.1), so S1's
3.88° and S2's 0.36° are both a four-foot stance; P2 judges the tilt alone.

A headless example that exercises most of it:

```bash
MUJOCO_GL=egl .venv/bin/python sim/playground.py --script \
  "walk 45; wait 2; walk 0 30 0; wait 1.5; stop; wait 1; gesture wave; wait 3; set gait.T 1.2; show; push 30 0 0.15; wait 1.5; quit"
```

It ends with `clean exit, no NaNs` (the playground asserts that nothing it did
produced a NaN, the reflex stack's promise). The installed righter prints a
`legacy obs, exceeds servo` warning at start-up: the checkpoint contract doing
its job (RL_GUIDE §2).

**Shoves (D048).** `push` (and the cockpit's shove buttons) applies
`sim/shove.py`'s hand shove: a half-sine force at the carapace's top rim
(60 mm above the torso frame, from params), so the torque about the feet does
the tipping, reported in N·s and bodyweights (torso subtree mass only, so a
free ball in the world does not inflate them). It replaced a rectangular
CoM pulse whose demo 120 N × 0.25 s was a 30 N·s strike. `push 20 0` sways
and braces; `push 40 0` tips it on most seeds (4 of 5 in
`run_reflex_fallen`). The survivable envelope per direction: §8.

**Falls and the righter.** A fall runs the learned righter (torch plus a
checkpoint in `sim/runs/`; the start-up line says which) under the
supervisor, fed tilt, the kinematic height, the measured joint angles and the
foot switches, so a fall ends in FALLEN → RIGHTED → NORMAL. RIGHTED is a
joint-space smoothstep ramp to the planted stance (1.0 s, stretched so its
peak stays under the 3.0 rad/s loaded budget; at D042's 0.6 s a hip left at
+90° asked 5.4 rad/s), a 0.5 s hold, then NORMAL. On the D052 model
the policy never meets `handoff_ok` by itself and the 3 s stall ramp does the
standing. Comparing righters live, the `rl` / `righter` commands,
`set reflex.stall_s` and the training curves: RL_GUIDE §6.

**The righter's hip clamp (D064, pick 13).** In FALLEN the hip *command* of
legs 1–4 is held at ≥ −51.05° (`reflex.righter_hip_min_deg` and
`reflex.righter_clamp_legs` in params, applied in
`gait/pebble_reflex.py` `ReflexSupervisor`), whatever produced it, and the
RIGHTED ramp starts from that clamped pose, so no righter folds a leg under
the keel tub. It is a command clamp, not a stop (an impact still back-drives
the measured hip past it), and never in NORMAL, PLANT, BRACE or RECOVER or
under a probe, where a hip floor would switch off the void guard (B129).
Leg 0 is out by the pick: with `recover1` it touches the shelf plate in 2 of
200 shove-demo falls (at most 1.40 mm, hip −67.4°); a leg-0 floor of −62.5°
gives 0/200 with every outcome identical (B141, not applied). Over 200
seeds the clamp clips 41 % of FALLEN ticks.

**HUD (viewer window).** A marker above the torso shows the reflex state
(green NORMAL, yellow PLANT, orange BRACE, blue RECOVER, red FALLEN, purple
RIGHTED) and an arrow shows the velocity command (length = 2 s of travel; a
short tangential arrow for turn rate).

**Keepers.** Anything tuned with `set` is sim-only until it goes into
`params.yaml` and the tree is regenerated (§6); a gait can be kept as a
preset with `gait save NAME`. Write keepers in `NOTES_INBOX.md`.

## 3b. The cockpit: internals

The cockpit is the playground in a browser: one continuously running sim,
rendered offscreen to two camera streams (a chase camera you can orbit, and
the robot's **eye** on the torso), with a page for driving, talking, worlds,
memory, RL, the studio and hardware. What each panel does and how to use it:
[COCKPIT_GUIDE.md](COCKPIT_GUIDE.md). This section is what sits behind it.

```bash
./rocky.sh cockpit                                   # http://127.0.0.1:8765; stops any running one first
./rocky.sh cockpit --world "rubble field" --recognize
MUJOCO_GL=egl .venv/bin/python sim/cockpit.py --port 8796   # a second one, started by hand
```

| flag | default | meaning |
|---|---|---|
| `--port N` | `ROCKY_COCKPIT_PORT`, 8765 | the HTTP port (loopback) |
| `--host H` / `--unsafe-lan` | 127.0.0.1 | a non-loopback host is refused unless `--unsafe-lan` (which warns: nothing is authenticated) |
| `--world W` | `ROCKY_WORLD`, flat | the preset at start |
| `--brain M` | talk | the brain mode at start: `talk`, `local`, `multimodal` or `claude` (the page switches it live) |
| `--awareness-s S` | `ROCKY_AWARENESS_S`, 20 | how often an idle robot composes its situation line; 0 = off |
| `--curious` | off | at most one unprompted look a minute when the lidar scene changed (a vision-model call) |
| `--no-reactions` | reactions on | no chord when a guard latches or something new appears within 0.5 m |
| `--no-memory` | on disk | keep the scene memory in RAM only |
| `--recognize` | off | place recognition ([PLACES.md](PLACES.md)) |

Stop it with Ctrl-C, the page's ⏻ quit button (the real legs are limped
first) or `./rocky.sh cockpit-stop`. For a phone, `./rocky.sh tailnet`
publishes it over HTTPS with `tailscale serve` (port 9445,
`ROCKY_TAILNET_PORT`) at `https://<machine>.<tailnet>.ts.net:9445`; the
server itself stays on 127.0.0.1. The phone path was verified over HTTPS on
2026-09-24 (BUILD_LOG, session 9g); the request guard's `*.ts.net` rule is also tested with
fixtures.

**Architecture.** One sim thread steps the `Playground` (§3) and renders the
cameras; the Starlette app serves the page, a server-sent-events state feed
at 10 Hz and MJPEG camera streams. The feed (and `GET /api/state`) carries
the newest 12 `events` plus `event_seq`, a count of every event since start,
so a client sees a new event even when the 12 look the same. The page's shared
view (lead / follow) and the in-app guide (`GET /api/guide`, this repo's
`docs/COCKPIT_GUIDE.md` as Markdown) come from `sim/cockpit_shared.py`;
`sim/tests/test_cockpit_shared.py` pins every guide section the page links
to.

**Hardening (D052).** An exception out of a physics step limps the real legs,
fails every waiting HTTP request with 503 and ends the process with exit code
1; `/api/state` carries a `heartbeat` (`step_age_s`, `loop_age_s`, `alive`,
`fatal`). Nothing is authenticated, so: a non-loopback `--host` needs
`--unsafe-lan`; every POST must be same-origin (an `Origin` whose host:port
differs from `Host` is 403; the tailnet's `https://<machine>.<tailnet>.ts.net:9445`
passes, since tailscale serve keeps the Host) and `Content-Type:
application/json` (`/api/voice`: multipart), so a cross-site form cannot drive
the robot; and the `Host` header must be loopback, `*.ts.net`, a tailnet
100.64/10 address, this machine's name or one of `ROCKY_COCKPIT_HOSTS`, which
stops a DNS-rebinding page that would otherwise pass the Origin check.

**The HTTP API** (send JSON; `sim/tests/test_cockpit_api.py` and
`sim/tests/test_awareness.py` drive all of it headless):

- state and streams: `GET /api/state`, `/api/events` (the 10 Hz SSE feed),
  `/video/<cam>.mjpg`, `/frame/<cam>.jpg`, `/api/ping`, `/api/help`;
  `/api/ui` + `/api/ui/events` (the shared view) and `/api/guide`;
- driving: `POST /api/tool/<name>` runs any registry tool (`{"x": 0.3,
  "y": 0}` for goto, `{"forward_m": 0.3}` for move), `/api/cmd {"line":
  "walk 45"}` any console command, `/api/teleop`, `/api/shove`,
  `/api/reset`, `/api/speed`, `/api/camera`, `/api/quit`;
- brains and voice: `/api/chat {"text": ..., "mode": "local"}`,
  `/api/chat/clear`, `/api/brain`, `GET /api/models`, `/api/look`,
  `/api/voice` (multipart), `/api/audio`, `/audio/<word>.wav`;
- worlds, memory, places: `GET|POST /api/world` (+ `/add`, `/save`, `/load`,
  `/random`, `GET /saved`), `/api/scan`, `GET|POST /api/memory`,
  `/api/awareness`, `/api/place` (D057);
- the robot: `GET /api/model`, `/api/gait`, `/api/capabilities`,
  `/api/servo`, `GET|POST /api/hw`,
  `/api/gesture/{list,save,delete,preview,play,check,solve,teach}`,
  `/api/chord/{list,render,save,delete}`,
  `/api/rl/{runs,righter,walk,curves.png}`, `/api/record`,
  `/api/recordings`, `/recordings/<name>/<file>`, `/api/replay`.

The MCP proxy (`harness/cockpit_backend.py`) uses the same routes and needs
a cockpit from the same checkout (`/api/ping`, `/api/capabilities`).

**Brains.** `sim/cockpit_brains.py` holds the modes (talk, local,
multimodal, claude), the roles and their fallback chains, against any
OpenAI-compatible server (`ROCKY_LLM_BASE_URL`); the defaults are the
reference setup's (D055) and every one is an environment variable (§7).
Which model for which role: [BRAINS.md](BRAINS.md) and COCKPIT_GUIDE
"Choosing a brain". Reliability rules: a tool that raises is a result, never
a 500; every tool call gets a result; per-mode locks; at most 6 tool rounds
(`MAX_HOPS`); a 90 s timeout per model call with no retries; thinking off
unless the id is in `ROCKY_THINKING_MODELS`; per-family prompt notes
(`ROCKY_FAMILY_NOTES`, D055a). `./rocky.sh brain-install` and `brain-bench`
(llama-swap only) add and measure a model: BRAINS.md.

**Stop first (D055).** With a model brain, a line with a stop word
(`stop_line`: `harness.intent`'s stop words, minus "don't stop", "non-stop",
"stop sign", "bus stop") runs the stop tool at once, without a model and
without waiting for the running turn; `stop_follow_up` sends a question or a
note on to the model with motion refused and refuses anything else. Every
chat line takes a stop mark when it arrives, and an operator stop after it
(a stop line, the STOP button or key, the console `stop`, `/api/tool/stop`,
the MCP stop; not a model's own stop call) refuses its motion calls with
`operator_stopped`, also while it waits for the per-mode lock (Talk
included); a model chain that then fails does not fall back to the regex
brain. `ROCKY_STOP_FIRST=0` sends stop lines to the model (the brain bench
does, to measure it). The operator's view: COCKPIT_GUIDE "Driving".

**Goto (D052).** `goto(x, y)` walks toward the target at the gait's envelope
speed and ends as one of:

- `arrived`;
- `cliff`: the always-on void guard found a planted foot with no floor under
  30 mm of probe, backed off and safe-stopped;
- `blocked`: a reactive layer reads the lidar at 8 Hz, and a return within
  ±30° of the travel heading closer than 0.35 m from the torso centre
  triggers a detour, first 45° off the target heading toward the clearer
  side, then a 90° sidestep to the same side, each until the robot-wide
  corridor toward the target has been clear for 0.8 s, 15.5 s at most
  (`cockpit.GOTO_DETOUR_S`, D063); blocked a third time, the goto stops with
  the obstacle's bearing and range. A goto refused at the start (a void in
  that direction, a latched safe-stop, held locomotion) is also `blocked`;
- `stuck`: 3 s without 2 cm of progress (something below the lidar plane,
  ~0.18 m up, so boxes, curbs, rubble and steps are invisible to it); during
  a detour, 2 cm farther from where the detour began (D063);
- `timeout`: 55 s (`harness.capabilities.goto_cap_s`: the 1.5 m reach 1.2
  times over at 34.2 mm/s plus the 2.36 s ease-in); once a goto has entered a
  detour its cap is 87 s (`goto_detour_cap_s`, the reach grown by both
  detours' 0.45 m, B116), and the cap ends a goto even mid-detour, so a goto
  answers within 88.5 s. A target farther than 3 m is an argument error.

Measured on D063 (flat floor, servo realism on): 1.2 m arrives in 37.0 s,
1.5 m in 46.1 s; head-on at a 0.25 m-wide wall or box at x = 0.6 the
sidestep clears it (~31 mm/s sideways) when the target is 0.6 m or more
behind it; a detour costs 11.0–32.0 s up to 1.5 m. Over 200 such gotos on the
87 s detour cap (B116, 2026-09-30) 16 of 16 at 1.0 m, 78 of 84 at 1.2 m and 80
of 84 at 1.5 m arrive, every other one ends `blocked` or `stuck` by itself,
and 2.0 m arrives 15 of 16 times (the latest at 85.7 s); on `deae868522cc`
the 46-scenario quick set ends the same way. `go_back_to` re-aims a leg that
still runs out of time (COCKPIT_GUIDE "Memory and awareness"): a ball 2.0 m
away behind that wall → back at it in about 72 s, two legs.

**Move (D055).** `move(forward_m, left_m=0)` is a relative move in the
robot's frame, in metres. It reads the pose when the call starts and runs an
ordinary goto to `(x + f·cos yaw − l·sin yaw, y + f·sin yaw + l·cos yaw)`
(`harness.local_brain.move_target`), so every guard applies and it ends as a
goto does; the result adds `move` {`forward_m`, `left_m`, `from`, `target`}.
It refuses more than 1.5 m (the goto envelope: ~0.034 m/s for 55 s), so
`move(forward_m=30)` for "30 cm" walks nowhere, and a boolean distance.
Small models got `x += d·cos(yaw)` wrong (a 3B model's "forward 30 cm" went
96° off), so relative moves are `move` and `goto` is for map targets and
remembered positions. A stop that lands while the move reads the pose (one
sim tick) refuses it; `turn` and `find_object` check the same way before each
motion. A recording replays the goto the move made. Over MCP with a cockpit
behind it, the cockpit runs the move; the mock and the in-process sim report
no heading, so there forward is map +x and the result says so.

**Vision.** The eye is a 320×240 camera on the torso (fovy 70°, ~86° wide,
pitched 15° down, 0.21 m above the floor, 0.10 m ahead of the torso centre).
`look` sends its JPEG to the vision role for a two-sentence description.
`find_object(name)` (a chat tool, `/api/tool/find_object {"name": "ball"}`,
the MCP tool, and talk mode's "find the ball") runs a loop: the model draws a
**box** around the named object (0–1000 coordinates); bearing and distance
come from projecting the box's bottom-centre onto the floor through the
camera's pose (`pixel_to_floor`); the robot takes a 0.1–0.4 m ordinary goto
toward it; unseen, or below 0.4 confidence, it makes a 30° scan turn instead
(at most one full circle); it looks again after each move and ends found
(≤ 0.25 m from the camera), not found or stopped (a goto came back cliff,
blocked or stuck: reported, not retried). Gemma 4 26B and 31B write boxes
y-first (`[ymin, xmin, ymax, xmax]`) whatever the prompt asks, Gemma 4 12B
x-first; `box_order()` picks the order per model id (`BOX_Y_FIRST`).
`sim/vision_bench.py` measures it on its own cockpit (:8791), placing the
ball or a box at known bearings and distances. Measured 2026-09-24:

| model | method | detected | false sightings | median bearing error | median distance error | latency |
|---|---|---|---|---|---|---|
| lfm2.5-vl | box + geometry | 12/12 | 0/4 | 0.7° (max 10°) | 0.03 m | 0.4 s |
| lfm2.5-vl | model types the numbers | 12/12 | 0/4 | 15° (max 73°) | 0.30 m | 0.4 s |
| gemma-4-26b-a4b | box + geometry | 12/12 | 0/4 | 0.75° (max 3°) | 0.03 m | 2.8 s (cold load 26 s) |
| gemma-4-26b-a4b | model types the numbers | 11/12 | 0/4 | 10° (max 20°) | 0.28 m | 2.8 s |

`find_object` found its target 3/3 with each model (ball ahead-left 0.9 m,
box ahead-right 0.8 m, a ball behind-left that needs the scan turns): 9–16 s
per trial with lfm2.5-vl, 18–32 s with gemma. Typed numbers do not work:
lfm2.5-vl echoes the prompt's example numbers (0°/43°, 0.2/0.8 m). **Floor safety is not a vision
job**: asked yes/no "does the floor end ahead?" facing the cliff, both models
said no (1–2 of 4 right for lfm2.5-vl, 2–3 for gemma across two runs); the
void guard stays the cliff sensor. These are clean MuJoCo renders, an upper
bound for the real camera. Per-model box numbers for the D055 brains:
[BRAINS.md](BRAINS.md).

**Memory and awareness** (`sim/scene_memory.py`; the operator's view:
COCKPIT_GUIDE "Memory and awareness"). One memory per world: looks,
`find_object` sightings, the operator's notes and pins, guard events (void
latch, latched safe-stop, thermal, bus faults `nan` / `cut` / `lost` at most
once per kind per 30 s) and the chords the robot said on its own. An object
is a name, a map position (kept only on the floor, ±6 m), a confidence and a
source. Sightings of one name within 0.25 m merge (up to 4 instances per
name); one less than half as confident as the stored one only counts (it
does not move or relabel it). At most 500 observations per world. Saved to
`sim/out/memory/<key>.json` (git-ignored; `ROCKY_MEMORY_DIR` moves it,
`--no-memory` keeps it in RAM): a preset keys by its name (`room.json`), an
edited or unnamed world by its name plus a hash of its spec
(`custom-3fa91c0e.json`), so two custom worlds never share a memory, and an
edit carries the memory over. A file that does not load is moved aside as
`*.corrupt-<time>`; a hand-edited one is normalised on load (a missing
confidence loads as 0.2); `forget all` keeps `<key>.json.bak`. The numbers
behind the tools ([TOOLS.md](TOOLS.md)): a look placement carries confidence
0.2 (small models guess distances badly: 0.30 m median error); `go_back_to`
refuses anything below 0.4 or seen before the last reset and walks legs of at
most 1.2 m (up to 4), stopping 0.35 m plus half the object's size short; a
find sighting beyond 2 m is kept at ≤ 0.3 (near the horizon a few pixels are
metres: a box bottom at 0.32 of the frame height projects to 14 m, at 0.36
to 3 m); stale = older than 10 min or seen before the last reset or world
load (places pinned at the robot's pose never are). One end-to-end run
(2026-09-24) put the obstacle-course ball 0.034 m from its true centre, and
`go_back_to` ended 0.37 m from it after the robot had walked 0.3 m away.

The **situation line** (every `--awareness-s` s while idle, and fresh at the
start of every chat turn's user message) stays under 400 characters, since it
rides in every turn: pose, latched guards, the lidar by direction, the three
nearest remembered objects with age and staleness, servo heat above 50 % of
the budget, the bus when servos are connected, the last look or find. A
reset, a world load or an edit forgets the last look and find; one still
running across it comes back with `stale_scene: true` and is not remembered.
Reactions (`alarm_help` when a guard latches, `curious_question` when
something appears within 0.5 m of a still robot) come at most once per 30 s.
`POST /api/awareness {interval_s, curious, reactions, recognize, refresh}`
drives the settings (`refresh: true` composes a line now); a body that is not
JSON gets a 400.

**Places (D057).** Off by default (`--recognize`, or `POST /api/awareness
{"recognize": true}`). With it on, every world load or reset triggers one
lidar sweep and one look from a standstill; the place memory answers
*known*, *new* or *ambiguous* with a confidence, one sense alone never says
known, and a change is declared only when two looks agree. The four place
tools (`where_am_i`, `name_place`, `places`, `forget_place`) are offered only
while it is on. Last bench (2026-09-25 21:15): 21/21 verdicts, changes 3/6.
Everything else: [PLACES.md](PLACES.md).

## 3c. Toward the real robot: studio, gait lab, hardware (D051, D052)

D052: "the studio cannot author what the robot cannot do; the bus gets a safe
first move". The procedure is COCKPIT_GUIDE ("Making gestures", "Gait lab and
realism", "Hardware bring-up") and `bench/BENCH_RUNBOOK.md` §5b (the bridge's
rules and every cut); this is what the code does.

- **Gesture studio.** Gestures are data: `gait/gestures/NAME.json`
  (`ROCKY_GESTURE_DIR` moves the library), keyframes of time, body offset,
  yaw, per-corner crouch, a raised leg as an "arm", reach targets, claw, a
  chord cue and an easing. Slider ranges come from `/api/model` (body
  ±40 mm, z −45..+25, yaw ±18°, the params joint limits). Every change goes
  through `pebble_feasibility.check_spec` with the sim's gait and model; a
  FAIL is neither played nor saved unless forced (a forced save records
  `"checked": {"ok": false}`), and built-in names cannot be overwritten.
  **Solve** is the whole-body solver (`gait/pebble_pose_solver.py`, 1–3 s:
  body lean, height, yaw and arm joints that reach a hand target with the CoM
  ≥ 25 mm inside the planted feet). **Teach** samples the joint targets at
  20 Hz (the real legs' measured pose for legs mirrored robot → sim) and
  turns the stream into keyframes (Ramer-Douglas-Peucker, 2° tolerance; the
  body pose is fitted from the planted feet, untrustworthy above ~5 mm fit
  residual). The chord designer edits the v0.2 voice model
  (`audio/chordspeak2.py`) and saves words to `audio/custom/`.
- **Servo model** (`sim/servo_model.py`, on by default): 50 Hz hold, 20 ms
  latency, a 4.7 rad/s no-load slew, 4096-count goals. Its load derate (rate
  × max(0.2, 1 − |τ|/stall)) is off since D052 V2, because the MJCF's damping
  and forcerange already carry the torque-speed line and the derate counted
  it twice; `set servo.load_derate 1` brings it back for an A/B.
- **Hardware bridge** (`sim/hw_bridge.py`): the stream runs at the 50 Hz bus
  rate, telemetry every second tick. **sim → robot** is refused unless the
  sim is at a planted standstill (NORMAL, no velocity, gesture, studio pose
  or goto) at 1× speed, which then stays locked; the soft entry parks each
  goal where the leg is, at 40 % torque and 200 counts/s, blends for ≥ 1.5 s
  (a 3 s low-torque window, 1 s tail) and releases. While it streams,
  locomotion (and the gestures that walk: `turn_in_place`, `sidestep` and
  their mirrors) is held until real foot contacts exist, shoves and gait
  changes are refused, and the mirror is **dropped** (the real legs hold
  their last goal) the moment the sim's reflex leaves NORMAL or the sim is
  reset or its world changes, so a reaction to something only the sim lived
  through never reaches the real legs. **apply limits…** writes the EEPROM
  angle limits from the params soft limits plus calibration (2° margin).
  Verified on the byte-faithful mock only (§8).

## 4. Experiments and records

The one-off experiments live in `sim/experiments/`, each asking one question
and printing a verdict: [sim/experiments/README.md](../sim/experiments/README.md)
has the table (what each measures, the decision it backs, how to run it, the
current result, re-run on `deae868522cc` on 2026-10-07 unless the row says
otherwise, and the earlier records back to pre-D052). Run them from the repo
root:

```bash
MUJOCO_GL=egl .venv/bin/python sim/experiments/run_cliff.py
ROCKY_EXPERIMENTS_OUT=/tmp/rocky-exp MUJOCO_GL=egl .venv/bin/python sim/experiments/run_stuck.py   # leave the record alone
```

Results and figures go to `sim/experiments/results/` (the pre-D052 record in
`results/pre_d052/`), clips to `sim/experiments/out/` (git-ignored).

The tools that stay in `sim/` because live code, CI or the docs use them:

| script | what it does | time |
|---|---|---|
| `run_sim.py` | the CI smoke walk (§2) | seconds |
| `shove_envelope.py [--quick]` | survivable rim shove per direction, standing and walking (D048) → `sim/out/shove_envelope.json` | ~3 min |
| `terrain_bench.py [--quick] [--jobs 6] [--stack S0,S1,S2] [--imu ideal,noisy,mount1.5] [--one ROW] [--list]` | the body leveler's ship gate (§3, D065): stacks × IMU modes × 87 terrain rows, the pass rules P1–P8 and R1 evaluated → `sim/out/terrain_bench.json` (`--quick`: `terrain_bench_quick.json`) | 643 s for all 9 stack × IMU pairs at `--jobs 6` (~71 s per pair; S3 is reported unavailable) |
| `torque_audit.py`, `mass_audit.py` | static servo margins on CAD masses → `sim/out/torque_audit.json`; the mass budget itself | seconds |
| `audit_gestures.py [NAME ...] [--json F]` | every gesture and gait row through the feasibility checker plus a physics pass → `sim/out/audit_gestures.json` with `--json` | ~20 s |
| `audit_righter.py CKPT ...` | a righter's jitter and handoffs (RL_GUIDE) | minutes |
| `rl_dashboard.py [--table]` | the checkpoint table; the training curves → `sim/out/rl_curves.png` | seconds |
| `vision_bench.py`, `brain_bench.py`, `place_bench.py` | the eye, the brains, place recognition, each on its own cockpit (:8791, :8792, :8795) → `sim/out/*.json` | minutes |
| `sim_lidar.py [--out DIR]` | records the lidar lap (inside the envelope since B111) into the `sim/lidar_scans.npz` fixture and writes the `sim/laserscan_spec.json` contract; `--out DIR` for a scratch run | ~15 s |

The tracked records in `sim/out/` (`shove_envelope.json`, `torque_audit.json`,
`audit_gestures.json`, `rl_curves.png`, `vision_bench.json`, `brain_bench.json`, `place_bench.json`,
`terrain_bench.json`)
are the last real runs; a new run overwrites them, so commit them with the
docs that quote them. The committed clips are regenerated with
`sim/run_sim.py --out sim` and:

```bash
MUJOCO_GL=egl .venv/bin/python sim/experiments/run_reflex_fallen.py --video sim/pebble_fallen_recover.mp4
# the README GIF: from 1.52 s, every second frame (80 ms), 480 px wide, 64 colours, bayer dither
ffmpeg -ss 1.52 -t 12 -i sim/pebble_fallen_recover.mp4 -loop 0 -vf \
  "fps=12.5,scale=480:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors=64[p];[b][p]paletteuse=dither=bayer:bayer_scale=4" \
  media/pebble_fallen_recover.gif
```

The GIF recipe is reconstructed from the 2026-09-26 GIF (150 frames,
480 × 320); the exact palette options were never recorded, so it matches
that file's frames and size (within 3 %), not its bytes.

## 5. Driving it with words: MCP and the LLM loop

The harness is an MCP server (`harness/server.py`, FastMCP over stdio; on the
robot the plan is streamable HTTP) whose tools come from one registry
(below). The core eight, on every backend:
`say`, `gesture`, `move(forward_m, left_m)`, `goto(x, y)` in metres, `stop`,
`scan_summary`, `status`, `list_gestures`. Where the backend has them: `look`
and `find_object` (an eye), the five memory tools (a scene memory: the
cockpit) and the four place tools (the cockpit, with recognition on). That is
8 tools on the mock or the in-process sim, 15 over a running cockpit and 19
with place recognition on. Every tool, with its arguments and texts:
[TOOLS.md](TOOLS.md).

**Contract invariants** (tested in `harness/test_harness.py`; the tool texts
say the same to the models):

1. **Guard supremacy.** The model proposes, the reflex layer disposes. A goto
   into a void returns `{"stopped": "cliff"}` with the robot at a safe pose;
   a veto is an ordinary result, never an exception
   (`test_goto_into_void_is_vetoed_not_raised`, and on real physics
   `test_sim_goto_into_void_stops_on_real_physics`).
2. **Single writer.** One motion intent in flight: a second goto preempts the
   first, which resolves as `stopped: "preempted"`
   (`test_second_goto_preempts_first`). In the cockpit one Playground loop
   owns the actuators and every command source goes through it.
3. **Honest async.** `stop` always succeeds, and a stop during a goto
   resolves the goto as `stopped: "user"`, not an error
   (`test_stop_resolves_goto_with_user`). A goto returns its terminal result
   only; streamed progress is not implemented.
4. **No state in the brain.** The backend owns all robot state and every tool
   result carries the full outcome, so any model can be killed
   mid-conversation and the robot stays safe.
5. **An absent tool beats a lying tool.** A backend without an eye offers no
   `look`; nothing is stubbed (`test_v1_tools_are_absent_not_stubbed`), and a
   word or gesture outside the real lexicons is refused with the list.

```bash
# text brain, no LLM (regex intent parser), on the in-process sim or the mock
.venv/bin/python -m harness.intent --backend sim --once "go forward 30 cm"
./rocky.sh talk                                   # the same, as a REPL

# a local model as the brain: any OpenAI-compatible endpoint (without --base-url: an Ollama daemon)
./rocky.sh brain                                  # ROCKY_LLM_BASE_URL + ROCKY_LLM_MODEL (reference: qwen3.6-35b-a3b)
.venv/bin/python -m harness.local_brain --base-url http://127.0.0.1:8080/v1 --model <id> --backend mock
.venv/bin/python -m harness.local_brain --mock-llm --once "wave"   # the loop with a scripted fake LLM

# Claude Code as the brain, over MCP
cp .mcp.json.example .mcp.json     # works as is from the repo root
./rocky.sh chat                    # then: "wave, then walk to (0.25, 0) and tell me what you see"

# voice: push-to-talk through a whisper server into the intent parser (Enter = the operator's confirmation)
./rocky.sh voice

# any MCP client: the MCP Inspector's browser UI with a button per tool
npx @modelcontextprotocol/inspector env ROCKY_BACKEND=mock .venv/bin/python -m harness.server
```

`.mcp.json` needs no `"cwd"` for Claude Code: `./rocky.sh chat` starts
`claude` in the repo root, so the relative `.venv/bin/python` and `-m
harness.server` resolve there. Clients that start elsewhere (Claude Desktop,
other MCP hosts) need the absolute python path and a `"cwd"` pointing at the
repo.

Which body answers the tools, `ROCKY_BACKEND`:

| value | backend |
|---|---|
| `auto` (the example's choice) | the running cockpit (`ROCKY_COCKPIT_URL`, default `http://127.0.0.1` on `ROCKY_COCKPIT_PORT`, else :8765; the server reads rocky.env like the cockpit) whenever it answers, else the in-process sim; re-checked on every call and logged to stderr |
| `cockpit` | the cockpit, or refuse to start when nothing answers |
| `sim` | the in-process MuJoCo sim (`harness/sim_backend.py`) |
| `mock` | the millisecond contract, no physics; the default when unset |

**The in-process `SimBackend` is not the D052 loop.** It has no servo model,
no probe and no always-on void guard (those live in the Playground, which the
cockpit runs); its goto fits the command into `WaveGait.budget`, eases it with
the supervisor's command slew (D063) and caps at the cockpit's derived time
(`goto_cap_for`: 55 s of walking on the params gait, ending in a safe-stop;
B114, it was a fixed 20 s; it has no lidar detours, so never the cockpit's
87 s detour cap, B116), and runs the reflex supervisor and its own D034
cliff detector, with the NaN guard, the 4.7 rad/s clamp and a
3.0 rad/s ramp back after a stop or gesture, and uses the 1.5 N contact switch
of `sim/scenes.py`. For D052-true behaviour, drive the cockpit (`auto` picks it
when it is up); rebuilding the backend on the Playground loop is B60. With the
in-process sim only: `ROCKY_WORLD=flat|room|cliff` (default `cliff`; the
example sets `flat`), `ROCKY_VIEWER=1` (the passive window, paced to real
time), `ROCKY_AUDIO=1` (chord-speak samples), `MUJOCO_GL=glfw` for the
window. With the cockpit these belong to the cockpit. `say` is played by the
backend: the mock logs the word, the in-process sim plays its
`audio/samples_v2` wav with `ROCKY_AUDIO=1`, the cockpit renders it with
`audio/chordspeak2.py`; `chordspeak_events.Narrator` makes only the offline
narration tracks of the demo clips.

**One registry (D056).** Every tool is defined once, in
`harness/capabilities.py` (`REGISTRY`): name, kind, parameters, the text each
surface shows, whether it is gated (a spoken line needs the wake word) and
what it requires (`eye`, `memory`, `cockpit`, `places`; a backend without it
does not offer the tool). Three things come from it:

- the local brains' OpenAI tool schema (`harness/local_brain.py`, and the
  cockpit's list in `sim/cockpit_brains.py`, which Claude mode converts to
  Anthropic tools), with live gesture and chord-word enums;
- the MCP server's tool list, which is live: it follows the backend's current
  gestures, chord words and capabilities (re-read after 2 s; a watcher polls
  every 3 s and sends a list-changed notification; `ROCKY_MCP_LIVE=0` reads
  the lists once), so a gesture saved in the studio reaches Claude Code
  without a restart;
- `docs/TOOLS.md`: all 23 tools (22 offered to models, the four place tools
  only while recognition is on, plus the executor-only `turn`), which surface
  offers each, their arguments and descriptions, the envelope and the robot.

`docs/TOOLS.md` is generated and committed; never edit it by hand. After a
registry change (or a `params.yaml` change that moves the envelope):

```bash
.venv/bin/python -m harness.capabilities --md --out docs/TOOLS.md   # write the page
.venv/bin/python -m harness.capabilities --check docs/TOOLS.md      # what CI runs: exit 1 + a diff when stale
.venv/bin/python -m harness.capabilities --json                     # the capabilities snapshot and its version hash
```

A new tool also needs its handler in the cockpit's executor
(`sim/cockpit_brains.registry_problems`, run by the tests, catches a missing
one); the whole procedure, with the test pins it moves, is
[DESIGN_CHANGE_GUIDE.md](DESIGN_CHANGE_GUIDE.md) §8.

## 6. Regenerating after a CAD or params change

Follow [DESIGN_CHANGE_GUIDE.md](DESIGN_CHANGE_GUIDE.md) §0: the CAD checks,
`mass_audit`, `build_mjcf`, `generate_urdf` and the parity check, the tests,
and the fingerprint before and after. CI regenerates the MJCF, the URDF and
`docs/TOOLS.md` and fails on a diff. The hip axis height, leg lengths and
joint limits all come from `params.yaml`; nothing in `sim/` or `gait/`
carries its own copy. The robot's shape is data too (D053): the `robot:`
block is validated by `rocky_model.robot()`, and
`.venv/bin/python gait/rocky_model.py` prints the spec with its topology hash
(`7f066d9bd8c0`). What is not wired yet (a 4th joint or a 6th leg validates
but does not run) is [DESIGN_CHANGE_GUIDE.md](DESIGN_CHANGE_GUIDE.md) §9,
steps 2–10, backlog B36.

## 7. Environment

Machine settings live in a git-ignored `rocky.env` in the repo root; the
tracked [rocky.env.example](../rocky.env.example) lists every variable with
its default and a one-line meaning, in five groups: the LLM server
(`ROCKY_LLM_BASE_URL`, its key), the cockpit's model roles and fallback
chains, speech (the whisper servers), the cockpit (port, hosts, memory and
gesture folders, the tailnet port) and brain-install / brain-bench
(llama-swap only: `ROCKY_MODELS_DIR` and the llama-swap paths). `cp
rocky.env.example rocky.env`, then uncomment what differs. `./rocky.sh`
sources it; Python started without the launcher (a cockpit under systemd, a
bench, pytest) reads the same file through `sim/envfile.py`
(`.venv/bin/python sim/envfile.py` shows what it sets). A variable already in
the environment wins over the file, and `ROCKY_ENV_FILE` points at another
file (`/dev/null` = none). Tests run on the reference setup: the repo-root
`conftest.py` and `./rocky.sh test` set `ROCKY_ENV_FILE=/dev/null` unless it
is already set (`ROCKY_ENV_FILE=rocky.env ./rocky.sh test` tests this
machine's settings). The model ids quoted in these docs are the
reference setup's (D055): one laptop with a 16 GB GPU running llama-swap;
`rocky.env` overrides every one.

Per run, not per machine (set on the command line): `MUJOCO_GL`
(`glfw|egl|osmesa|disable`), `ROCKY_WORLD`, `ROCKY_VIEWER`, `ROCKY_AUDIO`,
`ROCKY_BACKEND`, `ROCKY_MCP_LIVE=0`, `ROCKY_EXPERIMENTS_OUT=DIR`,
`ROCKY_REPO`, `ROCKY_D056_SNAPSHOTS` (tests).

## 8. Known gaps

Hardware and the onboard loop:

- **No real servo has been on the bus.** The hardware bridge's soft entry,
  fault cuts, re-arm rules and EEPROM limits are tested on the byte-faithful
  mock only (B32); GOAL_SPEED 0, torque-enable heading to the last goal,
  behaviour at an angle limit, the `PRESENT_LOAD` and `POSITION_OFFSET` sign
  bits and the baud codes above 4 are VERIFY-ON-BENCH (`driver/README.md`).
- Nothing runs on the Pi yet: the IMU, foot-switch and lidar drivers, the
  50 Hz loop, a host watchdog, a hardware backend for the harness and a
  torch-free righter are B35; the sim provides the signals. The ROS 2
  packages are a scaffold that has never been `colcon build`-ed.
- The harness's in-process `SimBackend` is not the D052 loop (§5, B60).

The model (B33, D052a):

- **The shove envelope** (`sim/shove_envelope.py`, rim half-sine, 0.4 s; BW on
  the 2.843 kg torso subtree), 2026-10-07 on `deae868522cc` (D064), 5 N grid:
  standing 30 / 35 / 35 / 30 / 30 / 30 N at 0 / 60 / 120 / 180 / 240 / 300°
  (min 1.08 BW, 7.6 N·s; mean 1.14 BW), the same forces as D063 on a 4.2 %
  heavier robot; walking at a steady 34.2 mm/s (the ask fitted by
  `scenes.walk_ask()`, shoved at 5.0 s, 1.64 s after the slew's 2.36 s
  ease-in; the script refuses an unsettled command) 30 / 30 / 35 / 30 / 30 /
  35 N (min 1.08 BW, mean 1.14; D063: 30 N in every direction, the lower CoM
  holds 120° and 300° 5 N longer). That is one gait phase: over 5 phases × 6
  directions the walking minimum is 25 N (0.90 BW), mean 31.2 N (D063: 0.93
  BW, 30.2 N, B103). On a 1 N grid standing is 32 / 37 / 37 / 31 / 33 / 34 N
  (floor 31 N = 1.11 BW at 180°; D063 30 / 38 / 38 / 30 / 32 / 32) and
  walking 33 / 33 / 35 / 34 / 32 / 35 N (floor 32 N = 1.15 BW at 240°; D063
  32 / 31 / 33 / 34 / 30 / 33). Every one is identical with the belly
  switched off: the mass and CoM, not the keel. More torque makes Pebble
  slightly *easier* to tip: D052's 1.911 N·m clip tipped 2–3 N later, its
  clipped legs yielding into a slide (D052a). The sliding limit for a CoM
  push is about 22 N (0.8 × mass × g; ~33 N at μ 1.2). Neither number is
  calibrated. The rectangular CoM pulse of the push
  experiments (`run_push*.py`) survives only for the record (D017 / D025).
- **Feet creep down a gravity slope.** With every foot planted and its
  switch closed, MuJoCo lets the stance slide downhill (μ 0.8, no `noslip`
  or `impratio` in `pebble.xml`): S1 standing creeps 24.6 / 39.4 / 49.1 mm in
  10 s at 5 / 8 / 10° (`terrain_bench`), so a walk gains downhill and loses
  uphill, and the terrain bench's progress rule cannot tell creep from
  walking on W2 (B162). Whether a TPU pad creeps on a real tile is
  unmeasured.
- Tracking lag p95 is 12.9–14.8° on the gait rows of `audit_gestures` (14 of
  19 rows warn TRACK; warn at 10°, fail at 20°; 16.5–18.5° before D063's
  soft-landing swing): the sim is close to saying the gait asks more than the
  servo follows. `manip_adjacent` warns
  `MARGIN_WARN` (a 19.4 mm CoM margin against the 25 mm warn line; 17.4 on
  D063).
- **Heat** (the rest of the honesty box's guesses are in §1): the RL envs
  derate a hot joint; the playground and cockpit only account it
  (`guard_status()['thermal']`, the `servo heat` chip, sim → robot refused
  past the budget); the studio and `compose_gesture` judge kinematics only,
  and the thermal verdict is `audit_gestures`'. The proxy never trips from
  cold inside an 8 s gait or 6 s recover episode (a 40 s walk at the
  envelope stays near zero heat, `test_rl_envs`), so training meets a
  derated joint only through `thermal_heat0` warm starts, off by default with no trainer flag yet.
  Between 0.65 and ~0.77 × stall it trips slowly (800 s at 0.70) where the
  driver mock never does: conservative there. 0.65 / 0.85 / 180 s need a
  10-minute bench hold with a temperature log.
- The experiment records before 2026-09-26 (`results/pre_d052/`) predate the
  spawn-pose fix (a 9.8° spawn tilt every `tilt_max` reported) and the peak
  clip; `run_push_reflex_v2`'s TRIP = 1.8 was calibrated on that spawn.
  Decisions D017, D022, D023, D025, D040, D042 / D045 / D048 and D044 (the
  knee's torque label) rest on numbers the rerun moved; each row carries a
  RERUN 2026-09-26 note
  ([sim/experiments/README.md](../sim/experiments/README.md#the-current-record)).

Behaviour:

- **The void guard's lip band: the robot no longer falls, it hangs on the
  keel** (D052a, D064). The touchdown gate fixed most of the 10–20° misses,
  but a 1° grid (walk 15/25/35/45 × −30…60°, 364 approaches, the slew on;
  walk 35 and 45 are both budgeted to 34.2, so they are one run) still has 8
  approaches at the same angles as on D063 (15 at 14–15°, 25 at 9–10°, 35 and
  45 at 10–11°) where a leading foot lands on the edge's lip and gives way
  while the next leg swings, before any touchdown the gate could hold on (a
  foot sphere centred 0–9 mm past the edge sits on the corner, closes its
  switch, then slides off). On D063 those 8 fell. On `deae868522cc` the
  robot tips 20–24° onto `belly_tub` at the edge and hangs there (torso
  0.243–0.245 m), the void fires 0.27–1.3 s after the tub lands, and it is
  still hung 3 s later: 0/364 falls by the D063 criterion (FALLEN or torso
  < 0.24 m), 8/364 tips (> 10°, the test's `TIPPED_DEG`); with the
  0.5°-offset grid added, 0 falls and 16 tips in 724 (17 falls on D063).
  With the belly switched off the same 16 approaches fall again: the keel
  catches the robot. After a
  fired guard nothing tips any more (1 on D063; 15 at 15.5° now stops, with
  or without the belly, so that is the mass); the worst safe tilt after a
  fire is 5.7°. Whether the real keel catches on a table edge or slides off
  is unmeasured. Re-run on `965f4f70e5d1` (2026-10-09) by
  `sim/experiments/run_lip_grid.py`: 356 stop, 8 tip, 0 fall, worst safe
  tilt after a fire 5.69°; with the body leveler on (`--level`) the same 8
  tip, no outcome changes, 314 of 364 approaches identical, every void within
  0.02 s of its level-off time (`results/lip_grid[_level]_results.json`; the
  same after the review fixes, run on the bench record's code).
  Still pinned as a strict xfail
  (`test_void_guard_lip_band_known_gap`, a tip counts as a fall); a
  look-ahead ToF is B33 (c). Not re-run on D064: with the slew off 7 falls,
  the careful walk 6 (15 at 15–16°, 25 at 10–11°, 35 / 45 at 9°), all on
  D063. Tried and rejected (D052a): re-seeking a planted foot whose switch
  opens while another leg swings (it closes the band, but the switch flicker
  at every handoff and on stairs, rough ground and ice ratcheted the probes
  into false voids); the same gated on a ≥ 1.5° tilt rise; `GATE_TICKS` 5
  or 6.
  **The hold probe (D063, `PROBE_HOLD_LEAD_MM` 7):** a late foot held by the
  gate seeks at once with a 7 mm lead (the verdict still needs the real foot
  26.5 mm down), and stops seeking if the body tilts 1° more than its lowest
  tilt in the hold (`PROBE_HOLD_ROLL_DEG`: on rough ground a shin on a bump
  rolled the body to 7° and made a false void). Before it, three approaches
  fired the guard 0.90 s into the hold and tipped while backing off; with it
  the void fires 0.56 s into the hold, tilt after ≤ 1.6°. Not in the careful
  walk. Terrain A/B on D063, 90 walks (rubble, stairs, obstacle course,
  rough 20 / 30 mm): 0 falls and 0 false voids before and after, but 7 walks
  tilt 1–2° more (worst 7.3°) and a foot that finds ground in a hold may
  preload up to 7 mm. Pinned by
  `test_a_fired_void_guard_backs_off_without_tipping`. Outside the band the
  guard stops with room to spare (walk 45 at 0–45°, max torso x 164–225 mm
  on the 0.35 m platform, 0 falls), and the retreat backs off 39–59 mm at
  walk 45 (14–28 mm at walk 25).
- **`run_cliff.py`'s control no longer walks off** (D064): the guard-less
  control hangs on the tub at 22.1° from 8.78 s (torso 239 mm), and the
  script's fall test (tilt > 50° or the torso below the platform) cannot see
  a robot hung on its keel, so the verdict reads `CLIFF DETECTION
  INCONCLUSIVE` (exit 0) where it read PASS. With the belly off the control
  falls at 9.04 s and the verdict is PASS. The detector itself still stops
  213.3 mm short. The script needs the guard tests' tip criterion (B140). `run_cliff_safestop.py` passes: margin 228.9 mm, the leading foot
  46.2 mm short of the edge.
- **A false void on 20 mm stairs** (B161): the shipped stack (S1) on
  `W6.stairs20.walk.p2` fires the void guard at 5.74 s with ground 4.3 mm
  under the foot (35.1 mm under its un-probed target) and stops 51 mm in;
  under the noisy IMU the same row walks 440 mm in S1 and false-voids in S2
  (11.8 mm under the foot, 42.8 under the target, the void foot raised
  0.3 mm). It is the probe's, not the leveler's (S2 is bit-identical to S1 up
  to the fire); phases 0 and 1 tip to 10.2° and 12.4° in S1. One S1 rubble
  walk false-voids too (`W4.rubble20.s0`, noisy, 9.5 mm), and one S2 one
  (`W4.rubble30.s1`, noisy: 2.9 mm under the foot, 38.3 under the target, the
  void foot raised 0.1 mm): ground deeper than the probe's 30 mm reach on a
  world with no drop.
- **The leveler's open costs** (D065, B162): the raise-only window sinks the
  body, so on rubble the belly comes within 23.5 mm of a rock (20 mm rubble,
  ideal IMU; 24.2 on 30 mm, noisy; S1 ≥ 31.3); OBS_NOISE drifts the plane by
  up to 1.68 mm on a flat walk; a turn, a stop or a shove leaves its plane in
  place (up to 0.92 mm, and 11.65 against the probe's HOLD after a turning
  stop, B163); a saturated plane walks 5 % slower (the derate).
- The careful walk cuts the old 72-approach sweep from 6 falls to 1 but walks
  17 % slower on every surface (flat 0.625 → 0.521 m in 15 s); whether it
  becomes the default is open. Even the default gate costs 2.6–4 % of
  distance on rough terrain and stairs (occasional holds) and 5–6 holds per
  step-down run.
- The touchdown gate sets the supervisor's gait clock from the playground and
  reads its private `_last_t`; `gait/pebble_reflex.py` has no API for it yet,
  so the Pi's loop (B35) cannot reuse it as is.
- On its back with **no** righter, the supervisor loops FALLEN → RIGHTED →
  FALLEN every stall period (3 s) and almost never stands (back landings
  3 of 65 over 200 seeds).
- The probe constants (settle 0.12 of the stance, 3.5 mm lead, 7 mm in a
  hold) are tuned to this sim's actuator and need bench values.
- Every RL checkpoint is legacy on the D064 model (the four pre-D052 ones by
  their observation, `recover6_d052` and `recover7_d063_curriculum` by their
  fingerprint and their uncut hip map); the walkers have no speed envelope on
  the D052 servo and are zeroed; no righter earns a handoff under the
  supervisor, while the supervised system with `recover1` stands 20/20
  against 11/20 with no righter, and 198/200 against 104/200 over 200 seeds
  (197 / 87 on D063: the lower CoM helps most with no righter). RL_GUIDE §4.

## 9. Not built yet: perception on the robot

Everything above runs on clean MuJoCo sensors; on the robot no sensor is
wired yet (B35). The plan, short (the full plan as of 2026-09-30:
[archive/PERCEPTION_PLAN_2026-09-30.md](archive/PERCEPTION_PLAN_2026-09-30.md)):

- **Layers.** L0 proprioception on the Pi (servo position, load and
  temperature over the 50 Hz bus, the SEA foot switches, a 100 Hz IMU; the
  legged-odometry EKF, D026); L1 the local bubble on the Pi at 10–20 Hz (the
  cliff detector, the foot probe, later sonar or ToF pods); L2 the geometric
  map on the Pi plus a server (a 360° 2D lidar, the odometry prior, a Wi-Fi
  RSSI prior; in the sim ICP SLAM-lite, D027); L3 semantics and memory on the
  server at 0.2–1 Hz (`look`, `find_object`, the scene memory, places).
- **Reflex arbitration**, fastest wins: E-stop (the XT60 loop key) > the
  thermal SafetyMonitor > push-reflex BRACE > the cliff VOID halt > the
  stuck-watchdog retry > the gait command. A VOID while the watchdog is in a
  retry stage feeds the watchdog (terrain data on rubble); otherwise it halts
  (`perception/cliff.py`).
- **Robot = reflexes, server = cortex.** The Pi runs everything real-time
  (gait, reflexes, EKF, SLAM-lite, chord-speak) and sends JPEG keyframes on
  demand, the scan, the pose and the events to a server that runs the vision
  model and the brain; annotations and goals come back, latency-tolerant.
  Nothing safety-critical depends on the link: with it down the robot keeps
  L0 and L1 (walk, avoid, patrol). The bottom of the brain stack needs no
  model (`harness/intent.py`; `harness/local_brain.py` rehearses the loop
  offline).
- **Sensor hardware** (rows in [bom/BOM.csv](../bom/BOM.csv), pins in
  [WIRING_HARNESS.md](WIRING_HARNESS.md)): a BNO085 IMU on SPI (C-01; not
  I²C, whose Pi controller mishandles its clock stretching, and not UART-RVC,
  which reports no angular rates, and the brace trip and the EKF need them);
  an LDRobot D500 lidar (C-02, phase C: 54.0 × 46.3 × 35.0 mm VERIFY, 45 g,
  0.03–12 m, 10 Hz, UART 230400 through its USB adapter, seated on the cap,
  B12; the sim keeps 360 rays at 8 Hz over 0.12–6 m until B46 gives it a
  `sensing.lidar` block); a Pi Camera Module 3 Wide (C-04: 120° diagonal,
  fixed behind a gill slot in a shell sector, B16; the sim eye is fovy 70°
  until B45 makes it the real lens); each leg's SEA switch on its own Pi
  GPIO; optional audio (a USB mic and a MAX98357A amp; the ReSpeaker 2-Mics
  HAT would fight the amp for the one I²S, the IMU for SPI0, and take the
  header).
- **Sensor pods** on the I6 dovetail ring (a printed shoe, the sensor, a
  JST-SH pigtail to the avionics bulkhead; any of the ring's stations):
  sonar × 5 (US-100, a 360° skirt, X-09), enviro (BME688), PIR (AM312),
  thermal (AMG8833, not in the BOM), whiskers (`whisker_shoe` in
  `cad/part_smallwins.py`, the only pod with CAD), a downward ToF
  (VL53L4CD: it sees an edge before a foot does, B33 c, B40, X-02). None has
  a driver or a sim model.
- **I²C.** The native bus carries only the TCA9548A mux (0x70) and the
  INA228 (0x40) if fitted (B47). Anything sampled slower than 10 Hz lives
  behind the mux (BME688 0x76, AMG8833 0x69, an OLED 0x3C, the ToF
  sensors), brought to the tray bulkhead's Qwiic port; 400 kHz.
- **Data contracts, frozen in code:** `perception/legged_odom.py` →
  `/pebble/odom`; `sim/laserscan_spec.json` → `/pebble/scan`;
  `rocky_msgs/ContactState`, `rocky_msgs/ServoHealth`;
  `perception/wifi_rssi_logger.py` JSONL v1.
- **What unblocks what:** servos, IMU and foot switches on the Pi (B35) →
  the EKF on hardware; the D500 and its cap mount (B12, B46) → slam_toolbox
  and place signatures on real scans; the camera, the gill bracket (B16) and
  B45 → `look`, `find_object` and places on the robot; sonar or ToF pods →
  the L1 look-ahead (the foot-probe guard works without them).
- **Behaviours planned** beyond the sim's patrol, `find_object` and place
  changes: frontier exploration with a chord summary, follow-me, the dock
  ritual (B14), dead-reckoning tricks, deliveries in the scoop (an I2 tool),
  people greeted by their own chord motif (B39), the question game.
