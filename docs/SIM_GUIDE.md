# Simulation guide

How Pebble's MuJoCo simulation is built, what runs on it, how to drive it
by hand and by language model, and what each number is worth. Everything
here was run on 2026-09-22 from a fresh checkout; the commands are the
ones that produced the outputs quoted.

## 1. How it works

**One model, generated.** `sim/build_mjcf.py` writes `sim/pebble.xml`
from two inputs: `cad/params.yaml` (leg lengths L1/L2/L3 = 45/95/135 mm,
body circumradius 110, hip axis height, and since D052 the servo and joint
identity) and `sim/mass_budget.json` (link masses summed from the CAD
tree's STL volumes by `sim/mass_audit.py`, D039). Five identical legs at
72° stations, three hinge joints each (yaw, hip, knee), a foot sphere per
leg, and a torso made of two stacked cylinders; 2.670 kg compiled, equal
to the budget. The URDF in `ros2/` is generated from the same inputs, and
`sim/check_urdf_parity.py` proves the two agree (20 joints, FK within
0.05 mm, identical mass, identical foot sphere). CI regenerates both and
fails on any diff.

**The servo, as the datasheet and one rule of thumb say (D052).** Every
number that describes a servo or a joint lives in `params.yaml`
(`actuators`, `joints`, `gait`, `reflex`, `sensing`) and is read through
one loader, `gait/rocky_model.py`; the MJCF, the URDF, `servo_model`,
`shove`, the driver's soft-limit clamp and the cockpit all use it. From
the ST3215's 2.94 N·m stall and 4.7 rad/s no-load:

| quantity | value | where it comes from |
|---|---|---|
| joint damping | 0.626 N·m·s/rad | stall / no-load: the servo's torque-speed line |
| leg actuator forcerange | ±2.94 N·m | the stall (PEAK) torque: the servo's instantaneous limit (D052 amendment) |
| continuous torque | 1.911 N·m | 0.65 × stall, a THERMAL budget judged over time, not a clip — `rl_common.ThermalProxy` (RL envs: derates a hot joint; playground/cockpit: accounting only) and `THERMAL_LOAD` in `audit_gestures`. 0.65 is a guess, VERIFY |
| loaded / free / hard speed | 3.0 / 4.0 / 4.7 rad/s | 3.0 ≈ continuous / damping (3.05: the top speed of a joint whose motor is held to the continuous torque, measured 3.06 — an upper bound, not a speed at which it also carries a load); 4.7 = no-load, which an unloaded MJCF joint now reaches (measured 4.71) |
| thermal proxy (RL envs; accounting only in the playground) | heat += ((\|τ_e\| / stall)² − 0.65²)·dt, ≥ 0; budget 54 | = (0.85² − 0.65²) × 180 s (`actuators.st3215.thermal_trip_frac` / `thermal_trip_s`): trips after 180 s at 0.85 × stall, 93.5 s at stall, never at ≤ 0.65 (the driver mock's 70 °C cut: 167 s / 88 s; at 0.80 both 248 s). A tripped joint's forcerange ramps 2.94 → 1.911 N·m as heat goes 54 → 64.8; reward −0.5 × mean derate; `thermal_heat0` warm-starts an episode (default off). 0.85 and 180 s are VERIFY |
| claw (SCS0009) | forcerange ±0.23 N·m, damping 0.0242 N·m·s/rad | its stall, and stall / no-load (0.23 / 9.5) like the legs; every SCS0009 number is a guess, VERIFY |
| joint soft limits | yaw ±40°, hip −70…90°, knee −150…−20°, claw 0…55° | the CAD-validated sweeps (D008) |
| foot | 6.5 mm sphere whose surface is the gait's IK foot point (site `foot_tip{i}`); μ 0.8, torsional 0.005 m | μ is a TPU-on-tile guess, VERIFY |

With that an unloaded MJCF joint tops out at 4.70 rad/s (forcerange /
damping, measured 4.71) and at 3.05 rad/s with its motor held to the
continuous torque (what a thermally derated joint gets), so the 4.0 rad/s
"free" budget is reachable. The first D052 cut clipped the actuators at
1.911 N·m, which capped every joint at 3.05 rad/s; the owner's amendment
(2026-09-24) made the clip the peak and moved the continuous budget into a
thermal model: heat += ((|τ_e| / stall)² − 0.65²) dt, budget
(0.85² − 0.65²) × 180 s = 54 (3 min at 0.85 × stall trips, 93 s at stall,
never at ≤ 0.65), where τ_e = actuator force − damping × qvel is the
motor's CURRENT term — the raw force counts the back-EMF voltage as heat
(a free yaw swung at 4.7 rad/s reads ~0.87 × stall RMS of force but ~0.22
of current). `audit_gestures` judges each joint's RMS τ_e / stall (> 0.65
warns `THERMAL_LOAD_WARN`, > 0.85 fails `THERMAL_LOAD`). Before D052 the
actuators were clamped at stall with joint damping 0.05, which let the gait
ask for 7.4 rad/s and turn 50 % further than the servo could.
`sim/model_fingerprint.py` hashes the robot (not the world) —
`ceb63a1254c3` since the amendment (`5a32f772ca99` was the 1.911 N·m clip) —
and every RL checkpoint records it.

**The control stack is the real one.** The sim does not have its own
controller. It imports `gait/pebble_gait.py` (closed-form IK, the five-
phase `WaveGait` that turns a body-frame velocity command into joint
targets; T 2.0 s, step 24 mm, duty 0.8 since D052), `gait/pebble_reflex.py`
(the `ReflexSupervisor` state machine: NORMAL → PLANT → BRACE → RECOVER
for safe-stops and shoves, FALLEN → RIGHTED for tumbles, D034/D042) and
`gait/pebble_watchdog.py` (progress watchdog with escalating step-height
retries, D023). The same modules are what the bench scripts and the ROS
bridge feed to real servos. Two D052 pieces sit in front of them:
- **`WaveGait.budget()`** fits every velocity command into the gait's
  envelope (45.5 mm/s at any heading, 0.246 rad/s turning in place) by
  uniform scaling, so heading and curvature survive; walk, teleop, goto,
  the residual walker and the ROS `gait_node` all go through it.
- **`gait/pebble_feasibility.py`** checks a motion before it runs:
  guarded joint limits, peak speed per class (loaded / free / hard), steps
  at entry, exit and phase boundaries, ≥ 3 feet down, the CoM margin
  (computed from the MJCF's own segment masses; it matches MuJoCo to
  0.0002 mm), slip, and self-contact. Code gestures, keyframe files, the
  studio, gait presets and the brain's `compose_gesture` are all checked
  by it; `sim/audit_gestures.py` runs it plus a physics pass over
  everything (20 rows, 0 FAIL on 2026-09-24).
- **The Playground's always-on guards** (§3) wrap both for every
  command source. Since the D052 follow-up the void guard has a
  **touchdown gate**: a stance foot that has not felt ground 140 ms
  after its commanded touchdown runs the gait clock back to that
  touchdown and holds it until the foot finds ground or the void guard
  fires (the wave gait lifts the next leg the instant one lands, so at
  a 10–20° approach a leading foot over the edge used to lose its
  neighbour before its probe finished, and the robot tipped in ~0.3 s).
  The void retreat is the approach played backwards (every foot goes back
  to a foothold it already stood on), not a walk back along the void
  bearing. `set gate.wait 1` is the careful walk: every touchdown waits
  for its switch (~17 % slower on every surface, safer at edges; off by
  default).

**Sensing in the sim.** `sim/sim_imu.py` gives gravity (from
`model.opt.gravity`, so a slope reads as a slope) and gyro with optional
noise and latency; `perception/contacts.py` is the one foot-switch
function: contacts against the world only (a foot pressed into its own
body does not count), 2.0 / 1.0 N close / open hysteresis.
`perception/` also supplies a legged-odometry EKF, an ICP scan matcher
and the cliff detector; `sim/sim_lidar.py` casts a 2D ray fan (a plane
~0.18 m above the floor) that `harness/sim_backend.py` exposes as
`scan_summary`.

**Worlds.** The scripts use three: `flat` (the default for driving),
`cliff` (a table-edge island for the VOID reflex) and `room` (walls,
pillars, a crate, for lidar and patrols), by flag or `ROCKY_WORLD`. The
cockpit builds more from a spec (`sim/world_builder.py`: flat, room,
cliff, obstacle course, rubble field, rough terrain, stairs, slope 8°,
icy floor, saved and random courses). Since D052 every world geom has
`priority="1"`, so the world's friction is what a foot contact gets (the
icy floor really is 0.35 now; before, the feet's own μ won and the preset
did nothing); ramps and stairs have a far side unless `"far": "drop"`;
random courses keep every object ≥ 0.3 m from the spawn.

**What the numbers are worth (the honesty box).** Masses are from CAD;
the servo identity is datasheet stall and no-load plus a guessed thermal
fraction; friction, the foot-switch forces and every observation-noise
level are guesses; link CoMs and inertias are primitive shapes, the SEA
spring is rigid, and there is no gear backlash (B33). The servo realism
layer (50 Hz hold, 20 ms latency, 4096-count goals) is on by default in
the playground and cockpit. The sim is excellent for logic (does the
reflex trip, does the gesture reach, does a gait tweak break a joint
limit or a speed budget) and directional for dynamics (push envelopes,
stability trends). Absolute numbers are "±real-robot-TBD" until a real
servo answers on the bench (D017 re-baseline, B32). D052 made the sim
harsher, not calibrated. Its first cut reported a *better* shove envelope
(standing 25 → 30–35 N); that was the 1.911 N·m clip letting the legs
yield into a slide, read on a 5 N grid. On the peak-torque model the
standing tip threshold is 29 N (1.11 BW) at its weakest direction, 2 N
below the 1.911 clip on the same tree (D052a).

## 2. Step by step: first run

```bash
cd rocky && . .venv/bin/activate                     # or pip install -e ".[sim]"
python sim/build_mjcf.py                              # writes sim/pebble.xml from params + masses
MUJOCO_GL=egl python sim/run_sim.py                   # headless walk, ~10 s
```

Expected (measured 2026-09-24 on the D052 model as amended the same day:
DC-line joint damping 0.626 N.m.s/rad, forcerange = stall 2.94 N.m with
0.65 x stall as the thermal budget, the foot-sphere spawn; `--out DIR`
keeps the video and contact sheet out of `sim/`):

```
body height: mean 117.0 mm (target ~118, rigid ideal servos), std 0.34 mm, min 116.0
tilt: mean 0.31 deg, max 0.83 deg
walk +X displacement: 246 mm (commanded ~261 mm), lateral drift -14 mm
turn in place: 55.4 deg at the budgeted 0.246 rad/s (commanded ~54 deg; asked 0.6 rad/s)
fell over: no
```

The pre-D052 numbers (264 mm, drift -37 mm, tilt max 1.03 deg) were the
flattering servo. The script exits non-zero on a fall or when the walk or
the turn leaves its smoke band, so CI catches a regression.

`MUJOCO_GL=egl` (or `osmesa`) renders offscreen; use `glfw` when you want
a window.

## 3. The playground: drive it live

```bash
MUJOCO_GL=glfw python sim/playground.py --viewer          # window + REPL
MUJOCO_GL=glfw python sim/playground.py --viewer --cliff  # the table-edge world
./rocky.sh play                                           # the same, from the launcher
```

One process runs physics with the real gait and reflex supervisor; you
type commands at the `pebble>` prompt while it runs, and the window
accepts keys.

**Keyboard teleop (in the terminal, at an empty prompt).** Taps nudge the
velocity command; there are no key-release events, so the command
persists until you change it. Type commands as usual; the single keys
only fire when the line is empty. The caps are the gait's envelope
(`WaveGait.max_command()`: 45.5 mm/s and 0.246 rad/s at the default gait),
and every nudge goes through the same guards as `walk`.

| key | effect |
|---|---|
| `↑` / `↓` (or `Shift+W` / `Shift+S`) | forward / backward ±15 mm/s per tap (capped at the envelope, 45.5) |
| `←` / `→` (or `Shift+A` / `Shift+D`) | strafe left / right ±15 mm/s per tap |
| `Shift+Q` / `Shift+E` | turn left / right ±0.12 rad/s per tap (capped at 0.246) |
| `SPACE` | safe-stop: zero the command and run PLANT → BRACE → planted idle (D034); also ends a gesture |
| `Shift+G` | wave hello (from a planted standstill) |

In the viewer window only the **arrow keys** drive. Every letter there is
one of MuJoCo's own render toggles (W wireframe, S shadows, A auto-connect,
D static bodies, G fog, Q camera frames, E equality constraints) and SPACE
pauses the viewer, and those bindings fire alongside ours, which is what
"WASD changes the lighting" was (D048). Mouse in the window rotates and
zooms the camera; Backspace there resets the physics state, so avoid it.

**REPL commands** (also usable headless via `--script "a; b; c"`):

| command | what it does |
|---|---|
| `walk VX [VY] [WZ]` | set the body-frame velocity command (mm/s, mm/s, rad/s); change it any time. Fitted into the envelope by `WaveGait.budget`: `walk 60 0 0.5` runs as (18.8, 0, 0.16), "scaled to 0.31x by the lift-speed budget", and `show` prints `cmd asked -> cmd run` |
| `stop` | the D034 safe-stop; also blends a running gesture out |
| `gesture NAME` | `wave bow look_around shake sit turn_in_place sidestep jazz_hands fist_bump beckon`, or any saved keyframe gesture (`gait/gestures/*.json`). Only from a planted standstill (NORMAL, no walk, no goto); blended in and out at ≤ 3 rad/s, and the reflex keeps watching, so a fall drops the gesture and the righter takes over |
| `gait NAME` · `gait save NAME` · `gait list` | load a gait preset (`gait/gaits/NAME.json`; `default` = params.yaml, `legacy_d050` = the old T 1.6 s / 32 mm gait, which has no envelope under the D052 budget) · save the current gait · list them with their parameters |
| `check [NAME]` | the feasibility report: with no NAME, the gait at the current command and at the envelope corners (PASS/FAIL, peak rad/s and which joint against loaded 3.0 / free 4.0 / hard 4.7, support, CoM margin); with a NAME, that gesture |
| `clear` | release a latched void (the cliff guard's bearing) and a latched safe-stop (3 gyro trips in 5 s); prints `nothing latched` when there is none. Does not resume the old walk |
| `probe on` / `probe off` | the stance contact probe (feet feel for the floor, D050) |
| `say WORD` | play a chord-speak sample (`aplay`/`ffplay`) or print it |
| `set PARAM VALUE` | live-tune `gait.T gait.h gait.R0 gait.duty gait.hstep reflex.trip reflex.stall_s reflex.fallen_max_s servo.on servo.hold_hz servo.latency_s servo.rate_rad_s servo.quant` (`servo.load_derate 1` for the A/B in §3c); phase-continuous, so feet don't teleport. Gait values are validated (T 0.4–10 s, duty 0.5–0.95, step 0–80 mm, stance reachable) |
| `show` | current gait params, reflex state and trip count, pose, asked → run command, probe state per leg, the servo model, sim time |
| `push FX FY [DUR]` | shove the shell rim: peak N, N, half-sine over DUR s (default 0.4); it prints the impulse in N·s and bodyweights. `push 20 0` sways and braces; `push 40 0` tips it over on most seeds (4/5 in the audit) and the righter takes it from there |
| `record on` / `record off` | capture an offscreen clip to `sim/playground_clip.mp4` |
| `help` / `help rl` | the command list, teleop keys and the always-on guards / the RL hooks below |
| `rl` | every checkpoint in `sim/runs/`: env, reward version, rate limit, obs version, servo, steps, last return, flags, recorded eval |
| `righter NAME` / `righter off` | hot-swap the self-righting policy (`runs/NAME/latest.pt`) or run with the analytic stall/deadline ramp only |
| `wait S` | (scripts) let S sim-seconds pass |
| `quit` | exit |

**Always-on guards (D052).** Every command source — `walk`, the keys, the
cockpit's goto, the residual walker, the hardware mirror — goes through
the same checks, in the Playground itself:
- **Void guard**: a stance foot that probes 30 mm and finds nothing while
  walking is a void. The robot backs off for 0.6 of a gait cycle,
  safe-stops, and latches the world bearing; any command with a component
  toward it is refused (`blocked: void at N deg — clear to release`), and
  a goto has that component projected out, until `clear`.
- **Trip escalation**: 3 gyro trips within 5 s latch a safe-stop;
  velocity is ignored until `clear`.
- **Target stream**: every joint target is rate-clamped at 4.7 rad/s
  (claws 9.5), a non-finite target is dropped with the last good one
  held, and the result goes both to the sim's servo model and to the
  real bus.
- **Kinematic height** from the IMU and the joint angles (what the robot
  can compute) is what the supervisor gets; world z is for the HUD.

Headless example, as run for this guide (2026-09-24, D052 tree):

```bash
MUJOCO_GL=egl python sim/playground.py --script \
  "walk 45; wait 2; walk 0 30 0; wait 1.5; stop; wait 1; gesture wave; wait 3; set gait.T 1.2; show; push 30 0 0.15; wait 1.5; quit"
```

It ends with `clean exit, no NaNs`; the playground asserts that nothing it
did produced a NaN, which is the reflex stack's promise. The legacy
righter prints a `legacy obs, exceeds servo` warning at start-up; that is
the checkpoint contract doing its job (RL_GUIDE §2).

**What a shove is (D048).** Every push used to be a rectangular force
pulse at the centre of mass, which has no gentle regime: below the
foot-friction limit the robot is a rigid block, above it the feet let go
and it cartwheels, and the old demo's 120 N × 0.25 s was a 30 N·s strike
that launched the robot 11–15 m at 10 m/s. `sim/shove.py` models a hand
shove instead: a half-sine force at the carapace's top rim (60 mm above
the torso frame, from params), so the torque about the feet does the
tipping; bodyweights use the torso subtree mass only (a free ball in the
world no longer inflates them). On the D052 model the standing robot
survives a 30 N peak at 0/180/240/300° and 35 N at 60/120° (1.15–1.34 BW,
7.6–8.9 N·s) and tips at 35–40 N; walking, 20–35 N by direction (weakest
20 N at 60°, 0.76 BW). Pre-D052 (μ 1.2, no damping) it was 25 N standing
in every direction and walking 20–30 N; the robot got harder to tip under
the harsher model, which is flagged as possibly flattering rather than
celebrated (`sim/shove_envelope.py`). The sliding limit for a CoM push is
now about 21 N (0.8 × 2.67 kg × g; was ~31 N at μ 1.2). The push-envelope
experiment scripts keep the old pulse because their JSONs are decision
records (D017/D025).

When a shove does tip it, the playground feeds the supervisor tilt, the
kinematic height, the measured joint angles and the foot switches, and
installs the learned righter (torch + a checkpoint in `sim/runs/`; it
says so at start-up), so a fall ends in FALLEN → RIGHTED → NORMAL. The
handoff from righter to ramp uses `handoff_ok` (tilt + ≥ 3 switches +
joint-angle height); on the D052 model the policy never meets it by
itself and the 3 s stall ramp does the standing (RL_GUIDE §4).

**HUD (viewer window).** A marker above the torso shows the reflex state
(green NORMAL, yellow PLANT, orange BRACE, blue RECOVER, red FALLEN,
purple RIGHTED) and an arrow shows the velocity command (length = 2 s of
travel; a short tangential arrow for turn rate).

**Watching the RL in the loop.** `rl` lists the checkpoints with their
recorded scores; `righter recover5_v3_warm` swaps the policy that runs
when the robot is FALLEN; `push 40 0` tips it over so you can compare
righters on the same fall (the HUD turns red, then purple for the ramp);
`set reflex.stall_s 2` changes how long a stalled policy gets before the
analytic ramp takes over. `python sim/rl_dashboard.py` draws the training
curves of every run into `sim/out/rl_curves.png`.

**Keepers.** Anything you tune with `set` is sim-only until it goes into
`params.yaml` and the tree is regenerated (a gait can also be kept as a
preset with `gait save NAME`); write keepers in `NOTES_INBOX.md`.

## 3b. The cockpit: the playground in a browser

```bash
./rocky.sh cockpit                      # http://127.0.0.1:8765
./rocky.sh cockpit --world "rubble field"
```

One continuously running sim, rendered offscreen to two camera streams
(a chase camera you can orbit, and the robot's **eye** on the torso), with
everything the terminal playground has and the parts that need a screen:

- **Header**: reflex state, mode, pose, tilt, world, the brain, and since
  D052 the **sim heartbeat** (`live` / `paused` / `stalled` / `DEAD`), the
  **robot fingerprint** (amber `?` when a loaded checkpoint carries no
  fingerprint, red `≠ ckpt` when it was trained on another robot; hover
  for details) and the guard chips: **VOID N°** (the always-on void guard
  latched a bearing), **LATCHED** (3 gyro trips in 5 s), both with a
  `clear` button, and **locomotion held** while the sim streams to real
  legs.
- **Overlay** on the chase camera: the command asked and the command the
  gait actually got after the guards and `WaveGait.budget` (with the
  budget note when it was scaled), feet, the stance probe per leg
  (S/P/H/— + mm), the kinematic height, and for six seconds the last
  command a guard refused (`blocked: void at 24 deg — clear to release`).
- **Console** at the bottom: every playground command (`walk`, `stop`,
  `gesture`, `push`, `set`, `gait`, `check`, `clear`, `probe`, `show`,
  `rl`, `righter`, `help`).
- **Brain & chat**: modes *Talk* (regex intent, no model), *Local model*,
  *Multimodal* (one vision model sees and acts) and *Claude* (the API, or
  `./rocky.sh chat` driving this sim over MCP). Role selectors for brain,
  vision, multimodal and the whisper model, each marked warm (● loaded) or
  cold; quarantined models never appear. The fallback chain per role is
  shown, a note warns when brain + vision are not the
  qwen3.6-35b-a3b + lfm2.5-vl pair that shares the GPU, and `/api/brain`
  refusals (text-only model as vision, quarantined model) are printed.
  Replies show fallback notes and the tool trace; a `compose_gesture`
  result has **save gesture** / **open in studio** buttons.
- **Adding a brain model**: `./rocky.sh brain-install [FILE ...]`
  (`sim/brain_install.py`) moves a downloaded GGUF from `~/Downloads` into
  `/mnt/models/<id>/`, finds or fetches its mmproj, adds the llama-swap
  stanza and the manifest entry, and restarts llama-swap (every loaded
  model unloads); `--dry-run` shows the plan and touches nothing.
  `./rocky.sh brain-bench --models ID [--vision]` (`sim/brain_bench.py`)
  then measures it in its own cockpit on :8792, never the one on :8765:
  VRAM, cold first answer, decode t/s, 20 spoken commands scored for the
  right tool, missed stops and unasked motion, left/right descriptions,
  and with `--vision` the vision bench. The GPU is shared, so leave the
  live cockpit alone while it runs (no chat, hands-free off, no look or
  find): a model request there evicts the model under test. Its latency
  column includes the motion; compare the first-action column with the
  0.2–1.5 s tool-call times. It prints a row for
  `docs/BRAIN_MODELS_2026-09-24.md`, whose "Adding and measuring a model"
  section has the workflow; the cockpit guide's "Choosing a brain" says
  how to read a row and switch roles.
- **Voice**: hold 🎤 to talk, or tap once to start and tap again to stop
  (phones lose a long press). The transcript lands in the input box; with
  the wake word ("pebble, …" — the fast model's misspellings count) or no
  motion in it, it is sent at once; otherwise the page says "needs the
  wake word — press Enter" and only your Enter sends it as a (trusted)
  command. **Hands-free** (checkbox next to the mic) keeps the mic open: a
  level meter cuts the utterances, each one is sent as trusted and goes
  straight to the robot, no wake word. `./rocky.sh cockpit` uses the fast
  whisper (`whisper-fast.service`, base.en on the CPU, ~1 s per command)
  when it is up and the large model on :8082 (18–20 s per command on 6 CPU
  threads) otherwise; `ROCKY_WHISPER_URL` overrides. For voice use the
  Talk brain (instant) or the local brain with `lfm2.5-vl` (0.2–0.5 s per
  command, tool calls included, already resident beside the 35B driver);
  the 35B driver itself adds several seconds per turn.
- **Stop first** (`sim/cockpit_brains.py`, 2026-09-25): with a model brain,
  a line with a stop word (`stop_line`: harness.intent's stop words, minus
  "don't stop", "non-stop", "stop sign", "bus stop") runs the stop tool at
  once, without a model and without waiting for the running turn. What else
  the line says (`stop_follow_up`) decides the rest: a question or a note
  goes to the model after the stop with motion refused; anything else is
  not run and the reply says so. Every chat line takes a stop mark when it
  arrives: a stop after that (a stop line, the STOP button or key, the
  console `stop` in any case, `/api/tool/stop`, the MCP stop; not a model's
  own stop call) refuses its motion calls with `operator_stopped`, also while
  it still waits for the per-mode lock (Talk included), and a model chain
  that then fails does not fall back to the regex brain. `ROCKY_STOP_FIRST=0`
  sends stop lines to the model (the brain bench does, to measure it).
- **Teleop**: arrow buttons or the keyboard (page focused, no text box
  active), gestures and chord-speak from dropdowns, shove buttons. The
  movement buttons are disabled, with the reason, while locomotion is held.
- **World & map**: presets, objects, friction (default 0.8, the foot pad's
  μ), gravity tilt, rough terrain, saved/random worlds; the top-down map
  shows lidar hits, the goto target and a latched void as a red wedge.
  Click the map to goto.
- **RL**: the checkpoint table (obs version, robot fingerprint per run),
  righter hot-swap, stall and deadline sliders, the training curves, and
  the **walking policy**: a gait checkpoint's PPO residual rides on the
  analytic gait while NORMAL, fed exactly the observation it was trained
  on (`rl_common.checkpoint_contract`: obs version, EMA, and its training
  gait, which is applied while it runs and restored on "analytic").
  Measured 2026-09-24: every walker on disk is pre-D052 (obs v1, T 1.6 s,
  32 mm step) and that gait has **no** speed envelope under the D052 servo
  budget — its swing lifts faster than the servo — so the budget zeroes
  every command and the panel says so. They need retraining on the D052 env.
- **Model & fingerprint**: what the sim believes the robot is
  (`gait/rocky_model.py`): servo stall / continuous torque / no-load speed
  / damping, the loaded / free / hard speed budgets, soft limits, gait and
  its envelope, the studio's ranges, and each loaded checkpoint's
  fingerprint status.
- **Recordings**, **Events** (guard notes, `hw:` events), **Help**; every
  section collapses, the sidebar toggles, speed 0.25–4× and pause.
- **Stop it**: Ctrl-C, the ⏻ quit button (the real legs are limped first)
  or `./rocky.sh cockpit-stop`. For the phone: `./rocky.sh tailnet`
  publishes it on the tailnet over HTTPS (`tailscale serve`, port 9445);
  the server itself stays on 127.0.0.1.

**Goto (D052).** `goto(x, y)` walks toward the target at the gait's
envelope speed and ends as one of: `arrived`; `cliff` — the Playground's
always-on void guard (one detector for every command source; the goto no
longer runs its own) found a planted foot with no floor under 30 mm of
probe, backed off and safe-stopped; `blocked` — a reactive layer reads the
lidar at 8 Hz, and a return within ±30° of the travel heading closer than
0.35 m (from the torso centre) triggers a detour: first 45° off the target
heading toward the clearer side, then a 90° sidestep to the same side,
each until the robot-wide corridor toward the target has been clear for
0.8 s (8 s at most); blocked a third time, the goto stops with the
obstacle's bearing and range. A goto refused at the start (void in that
direction, latched safe-stop, locomotion held) is also `blocked`, with a
detail. `stuck` — 3 s without 2 cm of progress (was 6 s): something the
puck cannot see, because the lidar plane is ~0.18 m above the floor
(torso + 60 mm) and anything lower — boxes, curbs, rubble, steps — is
invisible to it (`scan_summary` says so). Measured 2026-09-24 (flat
floor, from the origin, servo realism on): target 0.45 m away → arrived
in 12.2 s; a 0.8 m wall at x = 0.6 → blocked at 0.27 m after both detours
(16 s); a 0.25 m-wide wall → also blocked (the sidestep made only ~25 mm/s,
0.2 m in its 8 s: the reactive layer is not a planner); an 8 cm box →
stuck after 3 s; the cliff world → cliff at x = 0.19.

**Move (2026-09-25).** `move(forward_m, left_m=0)` is a relative move in
the robot's own frame, in metres: `forward_m` + ahead / − back, `left_m` +
left / − right. It reads the pose when the call starts and runs an
ordinary `goto` to `(x + f·cos yaw − l·sin yaw, y + f·sin yaw + l·cos yaw)`
(`harness.local_brain.move_target`), so every guard above applies and it
ends exactly as a goto does; the result is the goto's plus `move`
{`forward_m`, `left_m`, `from`, `target`}. It refuses a move longer than
1.5 m (the goto envelope: ~0.045 m/s in 40 s), so `move(forward_m=30)` for
"30 cm" walks nowhere. The model brains' prompt used to ask the model for
`x += d·cos(yaw)` after a `status` call; small models get that wrong (a
3B model's "forward 30 cm" went 96° off), so relative moves are now `move`
and `goto` is for map targets and remembered positions. `move` is gated like
`goto` (a spoken move needs the wake word). A recording replays the goto
the move made, not the move. A stop that lands while the move reads the
pose (one sim tick) refuses it (`operator_stopped`): with no goto running
yet, the stop alone would only have set safe-stop and the goto would have
walked. `turn` and `find_object` check the same way before each motion. The
MCP server has the same tool (`harness/server.py`): with a cockpit behind
it, the cockpit runs the move (`/api/tool/move`); other backends get
status, then their own goto (the mock and the in-process sim report no
heading, so there forward is map +x and the result says so). `move` and
`goto` over MCP refuse `true` for a distance, as the cockpit does.

**Vision.** The eye is a 320×240 camera on the torso (~86° wide, pitched
15° down, 0.21 m above the floor, 0.10 m ahead of the torso centre).
`look` sends its JPEG to the vision role (default `lfm2.5-vl`) for a
two-sentence description. `find_object(name)` (a chat tool, `POST
/api/tool/find_object {"name": "ball"}`, the MCP tool, and talk mode:
"find the ball" / "go to the box") runs a loop. The model **draws a box**
around the named object. Bearing and distance come from projecting the
box's bottom-centre onto the floor through the camera's measured pose
(`pixel_to_floor`). The robot then takes a 0.1–0.4 m ordinary `goto`
toward it (every guard applies). If the object is unseen, or the model's
confidence is below 0.4, it makes a 30° scan turn instead (at most one
full circle). It looks again after each move, and ends found (≤ 0.25 m
from the camera) / not found / stopped (a goto came back cliff, blocked
or stuck: reported, not retried). Every step goes to the console and
Events, and the STOP key ends it. Gemma writes boxes y-first (`[ymin,
xmin, ymax, xmax]`) whatever the prompt asks, so `box_order()` reads them
that way. `sim/vision_bench.py` measures all of it. It starts its own
cockpit on :8791, places the ball or a box at known bearings and
distances, and scores each model. Measured 2026-09-24:

| model | method | detected | false sightings | median bearing error | median distance error | latency |
|---|---|---|---|---|---|---|
| lfm2.5-vl | box + geometry | 12/12 | 0/4 | 0.7° (max 10°) | 0.03 m | 0.4 s |
| lfm2.5-vl | model types the numbers | 12/12 | 0/4 | 15° (max 73°) | 0.30 m | 0.4 s |
| gemma-4-26b-a4b | box + geometry | 12/12 | 0/4 | 0.75° (max 3°) | 0.03 m | 2.8 s (cold load 26 s) |
| gemma-4-26b-a4b | model types the numbers | 11/12 | 0/4 | 10° (max 20°) | 0.28 m | 2.8 s |

`find_object` found its target 3/3 with each model (ball ahead-left 0.9 m,
box ahead-right 0.8 m, and a ball behind-left that needs the scan turns):
9–16 s per trial with lfm2.5-vl, 18–32 s with gemma. Asking the model to
type bearing and distance itself does not work: lfm2.5-vl echoes the
prompt's example numbers (0°/43°, 0.2/0.8 m). **Floor safety is not a
vision job.** Asked yes/no "does the floor end ahead?" facing the cliff,
both models said no. Over four questions, lfm2.5-vl got 1–2 right and
gemma 2–3 across two runs. The void guard stays the cliff sensor. These
are clean MuJoCo renders (one orange ball, grey boxes, a checker floor),
so treat the numbers as an upper bound for the real camera.

**Memory and awareness.** The cockpit keeps a **scene memory** per world
(`sim/scene_memory.py`). It records what the robot looked at, the objects
`find_object` located, what the operator told it, which guards fired (void
latch, latched safe-stop, thermal, bus faults) and the chords it said on its
own. Each object is kept as a name, a map position, a confidence and a
source. Sightings of the same name within 0.25 m merge into one object;
farther apart they stay separate (two boxes, or a ball that was moved). A
sighting less than half as confident as the stored one (a look near a find)
is only counted: it does not move or relabel the object. The memory is saved
to `sim/out/memory/<key>.json` (git-ignored; `ROCKY_MEMORY_DIR` moves it,
as `ROCKY_GESTURE_DIR` moves the keyframe-gesture library — the brain bench
gives its cockpit scratch copies of both, so a model's compose_gesture never
lands in `gait/gestures`;
`--no-memory` keeps it in RAM). A preset keeps its name as the key
(`room.json`); an edited or unnamed world (`custom`, a replay's recorded
world) is keyed by its name plus a hash of its spec (`custom-3fa91c0e.json`),
so two different custom worlds never share a memory. Editing a world
carries its memory over, and switching worlds loads that world's file. A
file that does not load is moved aside as `*.corrupt-<time>`; a hand-edited
one is normalised on load (a missing confidence loads as vague, 0.2).
Positions are kept only on the floor (within ±6 m). Tools (chat, MCP, talk
mode):

- `where_is(name)`: "where is the ball". It answers with the newest sighting
  good enough to walk to, and mentions a newer vague one alongside.
- `recall(query)`: "what do you remember about the box". An empty query lists
  recent entries, the operator's notes and things near the robot first.
- `remember(note)`: "remember that the charger is by the door". "The X is
  here", "this spot is X", "call this spot X" or "mark here as X" pins X at
  the robot's pose; "the X is at (1.0, 0.2)" pins those coordinates. A pin off
  the floor is refused, not quietly kept as a note.
- `go_back_to(name)`: "go back to the ball", "go back home" (after "this spot
  is home"), "go back to the start" (every reset and world load pins `start`
  at the spawn point). Ordinary gotos, so every guard applies. It stops about
  0.4 m short, or goes onto a pinned place, and does not re-check with the
  eye. "Go back to where you were" asks which place you mean; it never walks
  backwards.
- `forget(name | all)`. Only "all" / "everything" wipes the memory, and it
  keeps the previous file as `<key>.json.bak`. "Forget that" or "forget it"
  names nothing, so it asks instead. A spoken "forget everything" needs the
  wake word.

Every `--awareness-s` seconds (default 20 s, 0 = off) while the robot is
idle, the cockpit writes a **situation** line. It holds, most useful first:
the pose, any guard that is latched, what the lidar sees by direction, the
three nearest remembered objects (with age, source, and "stale" when they may
have moved), servo heat (only above 50% of the thermal budget), the servo bus
(only when servos are connected) and the last look (or what `find_object`
found). A reset, a world load and a world edit forget that last look and
find (the line says "eye: no description yet"); a look or find still
running across one comes back with `stale_scene: true` and is neither
remembered nor kept as the last one. It aims at under 400 characters, since it rides in every chat turn.
Every chat turn also gets a fresh situation line, put at the start of the
message, so "what's around you?" is answered from it. It also shows on the state feed (`situation`,
`memory_objects`) and in `status`. With **reactions** on (the default in
`./rocky.sh cockpit`; `--no-reactions` turns them off) the robot says
`alarm_help` when a guard latches, or `curious_question` when something new
appears within 0.5 m while it stands still. It does this at most once per
30 s, and each reaction is logged to Events. **Curious** (`--curious`, off by
default) lets it make one unprompted `look` per minute when the lidar scene
has changed. That look is a vision-model call, so it can load a model.
`GET /api/memory`, `POST /api/memory {action: remember|forget|clear|recall|where_is}`
and `POST /api/awareness {interval_s, curious, reactions, refresh}` drive all
of this (`refresh: true` composes a situation line now). A body that is not
JSON gets a 400.

Honesty notes:

- **Find positions** come from the vision box geometry (the box's floor
  point, pushed back by half its projected width to estimate the centre).
  The vision bench measured about 0.03 m median error in clean sim renders.
  One end-to-end run on 2026-09-24 put the obstacle-course ball 0.034 m from
  its true centre, and `go_back_to` ended 0.37 m from it after the robot had
  walked 0.3 m away. None of this is measured on real hardware.
- **Look placements** carry confidence 0.2 (small models guess distances
  badly), and `go_back_to` refuses anything below 0.4.
- **Far find sightings** (more than 2 m from the camera) are kept at
  confidence 0.3 at most. Near the horizon a few pixels are metres (the box
  bottom at 0.32 of the frame height projects to 14 m, at 0.36 to 3 m), and
  the vision bench measured the geometry only out to 1.4 m.
- **The lidar** only sees things taller than about 0.18 m, so the ball and low
  boxes never trigger a reaction.
- **Stale objects**: the world puts its objects back at their spawn on every
  reset, world load and world edit. So anything seen more than 10 minutes ago,
  or before the last reset or world load, is marked stale. The map fades it,
  and `where_is`, the situation line and `recall`'s summary say "stale".
  `go_back_to` refuses a sighting from before the last reset (it names the
  old coordinates, so a plain goto can still go there) and flags one that is
  only old. Places pinned at the robot's pose ("X is here", `start`) do not
  go stale.
- **Bus events**: a streaming fault (`nan`, `cut`, `lost`) goes into the
  memory at most once per kind per 30 s. `rate` and `reopen` are routine and
  are not recorded as guards.

**Places: which room is this, from the senses (D057).** The scene memory
above is keyed by the world's name, and the name is ground truth a real
robot never has. `sim/place_memory.py` recognises a place from what the
robot senses instead, and the owner names it ("hey, I'm in the basement at
home", "hey, this is completely new", "hey, this master bedroom has a new
chair"). Rooms are enough now; `parent` is reserved for a home → room → spot
hierarchy later. The module is pure Python + numpy (no MuJoCo, no network:
the caller hands it every sense, and the embedder is a function it is
given). `PlaceMemory(directory)` keeps `<directory>/places.json` the way the
scene memory keeps its files (atomic write, debounced, a file that does not
load moved aside as `*.corrupt-<time>`, `places.json.bak` before a wipe; no
directory = RAM only). A place is an id (`place-<6 hex>`), a name (none
until you give one), its visits, up to 8 samples of each fingerprint (a new
sample 0.98 alike an old one replaces it, otherwise the oldest goes), the
objects seen there in the map frame, and notes.

The three fingerprints:

- **Scan**: `scan_signature(angles, ranges, yaw)` turns one lidar sweep into
  72 numbers. The first 36 are the nearest return in each 10° of *map*
  heading (the ray's body angle plus the robot's yaw); the last 36 are the
  quantiles of all the ranges (a sorted range histogram, with no yaw in it
  anywhere, so odometry yaw drift cannot touch it). Every value is
  log-scaled into [0, 1] (1 = nothing within 6 m), so a near wall moving 10
  cm counts more than a far one. `scan_similarity` compares the map part at
  its best circular shift (a yaw offset of any size costs nothing, and the
  winning shift estimates it: `scan_align`, 10° resolution, and a
  rectangular room can alias it by 180°) and the histogram part directly: d
  = 0.6 × d_map + 0.4 × d_hist, similarity = exp(−(d / 0.05)²).
- **Description**: the cosine of two `look` descriptions' embeddings. The
  cockpit's embedder is llama-swap's always-warm `embedding` model
  (Qwen3-Embedding-0.6B, 1024 dimensions, on the CPU, about 50 ms: it never
  loads anything on the GPU). Descriptions of rooms are never unrelated
  texts, so the raw cosine is mapped onto a score (`desc_score`): 0.25 → 0
  and 0.90 → 1 since this round, set by replaying the first place-bench
  run's recorded looks (the THRESHOLDS note in `sim/place_memory.py`; the
  run itself used 0.55 → 0). Measured there, the right room's description
  scored cos 0.64 to 0.90+ (median 0.77: the model re-words the same view)
  and a wrong room's 0.60 to 0.89 (median 0.79). They overlap completely:
  **in these rooms the description does not tell one room from another.**
  The shallower map only stops a re-worded description of the right room
  from reading as a mismatch.
- **Radio**: `{bssid: rssi_dBm}`, compared as a weighted Jaccard over the
  access points (−100 dBm counts 0, −30 dBm counts 1, an access point one
  side did not hear counts 0). **The sim never produces one**; every sim
  caller passes `radio=None`. It is the real robot's Wi-Fi scan, and it
  tells buildings apart much better than rooms, so it weighs little.

`recognize(scan_sig, desc_emb, radio)` takes, for every place, the
best-matching sample of each signal that both the place and the query have,
and combines them: combined = Σ wᵢ sᵢ / Σ wᵢ with w_scan = 0.5 × (the share
of map bins with a return, full at 25 %, never below 0.2 of the weight),
w_desc = 0.35, w_radio = 0.15; evidence = min(1, Σ w / 0.5), capped at
`SINGLE_EVIDENCE` = 0.45 when only one sense was compared; confidence = 0.5
+ (combined − 0.5) × evidence. Thin evidence pulls the score toward 0.5,
"can't tell", and **one sense alone never says known**: a perfect lone
sweep or a perfect lone description stops at 0.725, radio alone at 0.65 and
an empty sweep alone at 0.60, all below the 0.75 that known needs. Each
sense has a blind spot another covers: the lidar cannot tell rooms of one
shape apart, descriptions of one kind of room read alike, and radio tells
buildings apart, not rooms. A lone sense can still say *new* (a combined
score under 0.5 stays under 0.5). Two senses: an informative sweep plus a
description is full evidence; an empty sweep plus a description reaches 0.9
of it, which is how a lidar-empty world can be recognised at all, on the
description. So a failed look, or an embedding model that did not answer,
leaves a scan-only answer that is at best *ambiguous*, and the answer says
so: the result's `signals` lists the senses compared, and `verdict_text`
adds `[scan only]` (or `[look only]`, `[radio only]`). The verdicts, on the
best place against the runner-up (a place with another name):

- **known**: best ≥ 0.75 and at least 0.08 ahead of the runner-up.
- **new**: best < 0.5 (and `new` with no place id when nothing is stored
  yet).
- **ambiguous**: anything in between, including two places too close to
  call.
- **unknown**, confidence 0: no signal at all, or nothing the query and the
  stored places have in common.

`confidence` is always the best place's match score, on `new` too: 0.41 on
`new` means "the nearest place I know is 0.41 alike". `verdict_text` is the
situation line's clause (at most 90 characters): `place: basement (0.91)`,
`place: NEW (best basement 0.41)`, `place: basement? (0.62, or bedroom
0.58)`, `place: kitchen? (0.72, or bedroom 0.55) [scan only]`, `place:
unknown (no signal)`, and on a known place with a confirmed change `place:
bedroom (0.88) — new here: chair; missing: ball`.

The two honesty rules the owner agreed to:

1. **A single glance never declares a change.** `diff_objects(place,
   objects_now, visible=..., mentioned=...)` sorts the place's stored
   objects against what a look found now: *same* (the same name within
   `match_m`), *moved*, *missing*, *unseen* and *added*. *Missing* and
   *moved* need `visible(x, y)` to say the look could have seen the old
   spot; otherwise the object is *unseen* and nothing is claimed, and a
   thing of that name seen elsewhere is *added* with `maybe_moved_from`.
   *Missing* also needs the look not to name the thing: `mentioned` (the
   description, read by `mentioned_names`; negated mentions such as "no
   ball" do not count) turns a stored thing the look named but could not
   place into *unseen*. `confirm_diff(first, second)` keeps only what two
   looks agree on (the name, and the position within `match_m`); what one
   look alone reported stays *pending*. The module's default `match_m` is
   0.3 m.
2. **Every verdict carries its confidence.** `verdict_text` always prints
   the number, and a change is only reported on a *known* place: a new or
   ambiguous one has nothing trustworthy to compare with.

**In the cockpit** place recognition is **off by default** (`--recognize`,
`./rocky.sh cockpit --recognize`, or `POST /api/awareness {"recognize":
true}`). Off, the memory is keyed by the world, and the situation line, the
system prompt and the models' tool list are the D056 ones: the four place
tools need the registry's `places` capability, which only recognition
switched on gives, so the D055 / D055a brain-bench scores (measured on that
list) still describe a cockpit with recognition off. The routes do not
check the switch: `/api/tool/places` and `/api/tool/forget_place` still act
on the stored places with recognition off (`where_am_i` and `name_place`
answer that recognition is off), and `GET /api/memory` still carries
`places`. As implemented in `sim/cockpit.py`:

- **When it looks.** Every world load or reset makes a recognition due. Once
  the robot stands still (1 s of sim time after the spawn, reflex NORMAL,
  idle, not paused; it gives up after 20 s and tries again), it takes one
  lidar sweep (`scan_signature`, with the sim's pose standing in for
  odometry, a perfect one) and one look through the vision role
  (`Brains.look`), embeds the description (`Brains.embed`: llama-swap's
  `embedding` model, 5 s timeout, a failed embedding is a missing signal)
  and calls `PlaceMemory.recognize`. The look loads the vision role if it
  is not loaded (lfm2.5-vl unloads after 10 min idle, and loading it
  unloads any model outside its `resident` group, such as a `qwen3.5-9b`
  brain) and falls back to Gemma 12B, then 26B, if it fails, so with
  recognition on a world load or reset can move models on the GPU. From
  the same spot, before any turn, the eye is asked about the remembered
  objects of the place or places in play (see **Changes**). A look that
  failed has no objects (not an empty list): it is never read as "the eye
  saw nothing there", asks the eye nothing, and the answer says the verdict
  rests on the lidar alone.
- **A world edit** keeps the robot's pose and its place: what follows is a
  *change check* of the place the robot is bound to, never a new verdict
  and never another place (`_place_kept`). The verdict stays that place
  with how alike the robot senses it now, and a note when that is below 0.5
  ("this place changed a lot"). Talk mode's "this X has a new Y" runs the
  same check now (`place_check`).
- **ambiguous** and **new** both take a second look after a 30° turn to
  the left (the `turn` tool: one gesture, every guard; no turn when the
  operator stopped the robot during the recognition), and the two
  recognitions are combined (`combine_recognitions`: per place the mean of
  the two looks' confidences, the verdict by the same thresholds). A place
  still **new** after both is stored as `new place #k` with both looks as
  samples (`enroll_samples`), so it knows two views from the start; its
  first objects are the eye's placed sightings that the two looks do not
  contradict (`eye_objects`). **known** records a visit. While the robot
  turns, the place clause says so: `place: den? looking again (0.72, or
  workshop 0.55)`, `place: NEW? looking again (best den 0.41)`, `place:
  den (0.88) — looking again to confirm 1 change`.
- **Changes are asked of the eye, not read from the prose** (this round,
  after the first bench run). On a known place the change check is a set
  of box questions: for each thing the place remembers (walls and doors
  left out: they belong to the room, which is the lidar's business) and
  each thing the look's description mentions (`place_candidates`: the
  scene memory's object nouns among `mentioned_names`, negated mentions
  left out; the prose only proposes the question), the cockpit asks
  `Brains.detect(name, method="bbox")`, the question `/api/look {find:
  NAME, method: "bbox"}` and `find_object` ask, through the look's own
  vision model and while the robot still stands where it looked. A box is
  projected onto the floor (`sighting_to_map`; the vision bench measured
  0.7–0.9° bearing and 3 cm distance error in clean renders). Only a parsed
  "not there" is a no, and only where one of the thing's stored spots lies
  in that look's view (86°, 0.15–2 m from the eye); an error, an unparsed
  or unsure answer (confidence below `FIND_MIN_CONF` = 0.4), a box with no
  floor point within 2 m, or a spot out of view claims nothing. At most
  `PLACE_EYE_MAX` = 6 questions per look, each one a vision-model call.
  `place_memory.checked_objects` turns the answers into *added*, *missing*,
  *present* and *unobservable* (presence per name, not per instance, so it
  reports no *moved*). Any change takes the second look after the 30° turn,
  which asks the same questions again, and only what both looks agree on
  (`confirm_diff`: the name, and the position within `PLACE_MATCH_M` = 0.5
  m when both looks placed it; the 0.5 m is unmeasured) is declared and
  stored. One look's claim stays *pending* and the snapshot stays as it
  was: a look with no confirmed change rewrites nothing. The prose objects
  (`objects_from_description`) no longer decide anything. What each look
  asked and heard is in the answer's `eye_checks`. None of this has been
  through the place bench yet.
- **The scene memory follows the place.** Until a verdict, a provisional
  RAM-only memory (`place-pending`) stands in; a known or new place binds it
  to `place-<id>`, saved as `<memory dir>/place-<id>.json` next to
  `places.json`, and what the provisional memory recorded since the spawn is
  carried over. An ambiguous verdict binds nothing until the operator names
  the place. When the operator corrects a wrong recognition (`name_place`,
  below), what the visit wrote into the wrongly recognised place is taken
  back (`_place_take_back`: a place stored on this visit is dropped, a known
  one gets its record from before the visit and its scene memory from the
  moment of the bind) and the records since the bind move to the right
  place.
- **Where it shows.** The situation line starts with the place clause
  (`place: recognising…` while it runs); the state feed carries `place`
  (`verdict`, `name`, `place_id`, `confidence`, `signals`, `text`,
  `status`, `looks`, `by`; `signals` in words: `scan + description`, `scan
  only`, `the operator's word`); `where_am_i`'s `detail` ends with the same
  (`[signals: scan + description; confidence 0.88]`); `GET /api/memory`
  carries `places`. With **reactions** on, a new place and a confirmed new
  object each say `curious_question` (the usual one reaction per 30 s). A recognition that is interrupted (no
  standstill within 20 s, paused, moved while looking) or raises is tried
  again every 5 s for as long as recognition is on; a failed look is not a
  failure (the sweep decides alone, at best *ambiguous*). Only
  `_place_tick` itself raising `AWARE_FAILS_MAX` = 3 times (counted since
  the last spawn or result) switches recognition off, with a console line.
- **Tools** (`harness/capabilities.py`, D056): `where_am_i` (the last
  recognition and the place: it waits for one that is running and never
  looks by itself), `name_place(name, new?, rename?)`, `places`,
  `forget_place(name | 'here' | 'all')` (a pronoun forgets nothing;
  `places.json.bak` before a wipe; a spoken 'all' needs the wake word).
  They are offered to the cockpit's model brains, and by the MCP server
  over a cockpit (`harness/server.py` through `CockpitBackend` /
  `AutoBackend`, which forward them to `/api/tool/<name>`), only while
  recognition is on: the cockpit's `GET /api/capabilities` then carries the
  registry's `places` capability, and the MCP list follows it on the next
  `tools/list` or watcher tick (3 s, with a list-changed notification);
  switched off, the four leave both lists. The mock and the in-process sim
  never have them; `/api/tool/<name>` always runs them. `name_place` names
  an unnamed recognised place (or one stored on this visit); a name that is
  another
  place's, or that differs from the recognised place's own name, is a
  **correction** (the robot was wrong: this is that place, or a new one
  under the name, and the wrongly recognised place is put back as it was);
  `rename=true` renames the recognised place instead ("call this place the
  study"); `new=true` stores a different place, one per visit however
  often it is said. While recognition is on, talk mode maps the owner's
  sentences onto them (`place_intent` in `sim/cockpit_brains.py`: "where am
  I", "I'm in the basement at home", "this is completely new", "call this
  place the study", "this master bedroom has a new chair", "what places do
  you know", "forget this place"), and the model brains' system prompt gets
  a place note. "This master bedroom has a new chair" names the place and
  then runs `place_check`, a change check of that place with the two-look
  rule; the reply says what two looks agreed on, what one look alone saw
  (not declared), or why nothing was checked.
- **Routes.** `GET /api/place` (the current answer and every place) and
  `POST /api/place {action: recognize {force?} | name {name, new?,
  rename?} | forget {name} | list}`. `sim/tests/test_awareness.py` drives
  the cockpit side headless with a fake eye (the module's own tests are
  `sim/tests/test_place_memory.py`).

Limits in MuJoCo, measured 2026-09-25 with the robot at the spawn pose (the
scan plane is 0.179 m above the floor, from `PUCK_DZ` = 0.06 m above the
torso; scratch scripts on `world_builder` + `sim_lidar.scan`, a sweep after
1 s of standing, no cockpit, no model):

| preset | rays with a return (of 360) | map bins with a return (of 36) |
|---|---|---|
| room | 360 | 36 |
| obstacle course | 37 (its one 0.25 m wall) | 5 (0.14: 0.56 of a full sweep's weight) |
| flat, cliff, rubble field, rough terrain, stairs, slope 8 deg, icy floor | 0 | 0 |
| room a, room b, room a + chair, room b + chair | 360 | 36 |
| room c, room c + chair, room c - ball | 208 | 23 |
| room d | 292 | 30 |

- **8 of the 9 general presets give the lidar nothing or next to nothing**:
  seven return nothing above the scan plane, and the obstacle course only
  its one wall. Every empty sweep matches every other, so in those worlds
  recognition rests on the description, which did not tell the bench's
  rooms apart (below): a verdict in a lidar-empty world is not measured and
  not to be trusted. A sweep alone (empty or not) stops at *ambiguous* by
  design. The place bench's rooms are built for the lidar (walls 0.25–0.5
  m).
- **MuJoCo rooms are simple**: flat-shaded walls and boxes on a checker floor,
  a perfect ray-cast lidar, a clean 320 × 240 render. Every sim number here
  is an upper bound for the real camera and the real puck, and none is
  measured on a walking robot's tilted sweep.
- **The description cannot break a lidar twin.** A room of the same shape
  whose furniture the lidar sees alike, described the way the bench's looks
  are (cos about 0.82), would read about 0.85 and come out *known*: the
  replay in `sim/place_memory.py` says so, and no description map fixes it.
  The eye's per-object answers are the sense that could; that is not
  measured.
- The scan similarity was calibrated on **synthetic** ray-cast rooms (360
  rays, the `room` preset's 3.2 × 2.6 m walls and pillars; the helpers are
  in `sim/tests/test_place_memory.py`): the same spot with 1 cm noise
  0.9996; 0.1 m away 0.94–0.98; 0.3 m away 0.61–0.83 (median 0.71); 0.5 m
  away 0.24–0.63, which is why a place keeps several samples; a room of
  another shape (5 × 4 m, a 6 × 1.2 m corridor, open floor) 0.005 or less; a
  3 × 3 m room up to 0.73; the same 3.2 × 2.6 m walls with other furniture
  0.84 (the caster is 2D, so all of that furniture is lidar-visible).
  **Rooms of one shape are near-twins to the lidar** unless much of the
  furniture it sees differs (rooms a and b in MuJoCo, where b adds a 0.9 m
  inner wall and two 0.22 m boxes: 0.49 alike at the spawn, 0.53 at the
  bench's enrolment), and furniture below the 0.179 m plane is invisible to
  it altogether. The description did not separate the bench's rooms (see
  the first run), so what can is the radio on the robot, or the eye's
  per-object answers (not measured).
- Every world load puts the robot back at the origin facing +x, the spot it
  first saw the place from, so a sim revisit is an **upper bound** for a
  robot that comes back from anywhere.
- Object positions are in the map frame of the visit. In the sim that is the
  world frame, so they compare directly; a real robot's map frame restarts
  every session, and `scan_align` gives only the yaw between two visits, not
  the translation.

**The place bench** (`sim/place_bench.py`, tests
`sim/tests/test_place_bench.py`) scores the owner's three sentences from the
senses alone: known, new, and known with a change. It measures the cockpit's
own protocol above: it stages a world, waits and reads (`POST /api/place
{action: recognize}`, then `where_am_i` and the state feed's `place`), and
never looks or turns for the robot. Each trial starts from an empty place
memory, enrols rooms a, b and c and names them the way a truthful owner
would (den / workshop / playroom), then runs its cases: `a same pose`; `b
named as a` and `a named as b` (each room loaded under the world name the
OTHER room was enrolled with: a recogniser that used the name would answer
the other room, flagged `name_leak`); `a moved + turned` (0.29 m away and a
quarter turn, with recognition off while it walks there); `d never seen`
(must be new); `b + chair` and `c - ball` (the change must be confirmed by
the cockpit's second look, counted only when the changed spot was in the
eye's view before and after the read; a change confirmed from a single look
counts as a broken two-look rule). Every world is loaded under an opaque
name (`pb-<8 hex>`) with a spec that differs on every visit, so nothing
keyed by a name or a spec can carry a room across visits. It starts its own
cockpit (default :8795, never :8765) behind a loopback fence that forwards
only the vision model and the CPU `embedding` model, so a failing vision
model cannot pull a 21 GB fallback onto the GPU.

```bash
.venv/bin/python sim/place_bench.py                          # own cockpit on :8795, 3 trials, lfm2.5-vl
.venv/bin/python sim/place_bench.py --trials 1 --no-embed    # the scan + the look, no description embeddings
.venv/bin/python sim/place_bench.py --dry-run --trials 1     # plumbing only: canned look, hashed embeddings, no GPU
.venv/bin/python sim/place_bench.py --trials 1 --cases "d never seen,b + chair"   # rooms a-c are always enrolled
```

`--vision-model ID` picks the vision role it measures (default `lfm2.5-vl`;
it loads if it is not loaded, which unloads any model outside its
`resident` group, and the fence keeps the Gemma fallbacks off the GPU).
`--no-embed` makes the fence refuse embeddings, so recognition runs on the
sweep alone (the look still runs: it feeds the changes), which after the
single-sense cap can answer *ambiguous* or *new* but never *known*.
`--relook` turns 30° after each read and forces a fresh recognition, to see
whether it agrees; it adds samples to the places, so it is off by default.
`--url` points it at a cockpit you started (loopback, never :8765; not
fenced, and each trial wipes its places with `forget_place all`, which
keeps `places.json.bak`). It prints a table (a confusion matrix expected ×
observed, the looks taken, change hits / false alarms / single-glance
declarations, confidence per case, the read's wall time) and a per-read
confidence table (the confidence, the per-sense parts, the evidence, the
senses compared, the looks, the runner-up and the lead over it), writes
`sim/out/place_bench.json` (`--out`: every read with the raw answers, its
parts, evidence, signals and per-look results, the change check's per-name
eye answers for each look, and the exact API calls; the place_memory
thresholds are derived from these records) and prints a Markdown block for
these docs. A `--dry-run` result is marked DRY RUN: plumbing, not a
measurement. Leave the owner's cockpit alone while it runs, for the same
GPU reasons as the brain bench.
`sim/out/place_bench.json` is committed as the record of the last real run,
like `sim/out/brain_bench.json`; a new run overwrites it, so commit it with
the docs that quote it.

**First run, 2026-09-25 19:39** (`sim/out/place_bench.json`): the bench's
own fenced cockpit on :8795, vision `lfm2.5-vl`, description embeddings on
(46 through the fence), 3 trials of the 7 cases, physics at 2.0×, and the
cockpit's change check as it stood then (objects read from the look's
prose).

| case | expected | verdicts | correct | confidence median (min–max) | second look taken | changes (two looks agree) |
|---|---|---|---|---|---|---|
| a same pose | known a | known ×2, ambiguous ×1 | 2/3 | 0.82 (0.73–0.88) | 2/3 | none expected, none declared |
| b named as a | known b | known ×2, ambiguous ×1 | 2/3 | 0.83 (0.73–0.85) | 2/3 | none expected, none declared |
| a named as b | known a | known ×3 | 3/3 | 0.97 (0.89–1.00) | 3/3 | none expected, none declared |
| a moved + turned | known a | known ×3 | 3/3 | 0.87 (0.84–0.87) | 3/3 | none expected, none declared |
| d never seen | new | new ×3 | 3/3 | 0.29 (0.26–0.29) | 0/3 | none expected, none declared |
| b + chair | known b | ambiguous ×2, known ×1 | 1/3 | 0.73 (0.67–0.78) | 2/3 | 0/3 found |
| c - ball | known c | ambiguous ×2, known ×1 | 1/3 | 0.73 (0.70–0.76) | 2/3 | 0/3 found |

- **15/21 correct and 0 wrong "known"**: a revisit was recognised (12) or
  called ambiguous (6), never taken for another room; the never-seen room
  was new 3/3, at 0.26–0.29, far below the 0.5 line. The name swap was
  right 5/6 with no name leak. In the 15 visits where nothing had changed,
  0 false alarms, and 0 changes were declared from one glance.
- **The true matches sit at the threshold.** The 12 known verdicts scored
  0.755–1.00 (median 0.86); the 6 ambiguous ones 0.67–0.735, just under
  0.75.
- **The scan carried it; the description is the weak sense.** Per sense
  (each read's `parts`): the scan scored 1.00 at every same-pose revisit,
  0.87 with the chair in the room (a 0.25 m box is lidar-tall), 0.79 after
  the 0.29 m move and the quarter turn, and 0.00 for room d against every
  stored room. The description (scores under the run's 0.55 → 0 map)
  scored the right room 0.26–1.00 (cos 0.64 to 0.90+, median 0.77) over
  the 18 revisits, while the never-seen room d's description scored
  0.64–0.71 (cos 0.77–0.80) against its nearest stored room: in 9 of the
  18 revisits the right room's description scored lower than that. Every
  ambiguous verdict is a scan of 0.87–1.00 pulled below 0.75 by a
  description score of 0.26–0.52 (cos 0.64–0.73). lfm2.5-vl mostly
  describes the floor of a 320 × 240 frame ("The floor ahead is mostly
  clear, with a few scattered dark specks"), and two looks at one spot can
  read differently.
- **Changes: 0/6 found, and why.** In trials 2 and 3 both changed rooms
  came back ambiguous, and a change is only checked on a known place, so
  the check never ran. In trial 1 both were known (one look each) and the
  check found nothing, because it read the objects out of the look's prose
  (`objects_from_description`), which places a thing only when one
  sentence gives a direction AND a distance: lfm2.5-vl gives no metres ("A
  white cube and a wall are within a few body lengths ahead"), so the
  chair was never placed, and the ball had never been placed at enrolment,
  so it could not go missing. The two-look rule did its job the other
  way: in 8 of the 15 unchanged visits one look placed a phantom "wall"
  from the prose (new or moved), the second look did not agree, and it
  stayed pending.

**Second run, the same evening (21:15), after the fix round** (description map
re-set from the run above: `DESC_COS_FLOOR` 0.55 → 0.25; two looks stored at
enrolment; the change check asks the eye for a box per remembered name instead
of reading the prose):

| case | expected | verdicts | confidence (median) | changes |
|---|---|---|---|---|
| a same pose | known a | known ×3 | 0.96 | none, none declared |
| b named as a | known b | known ×3 | 0.95 | none, none declared |
| a named as b | known a | known ×3 | 0.96 | none, none declared |
| a moved + turned | known a | known ×3 | 0.87 | none, none declared |
| d never seen | new | new ×3 | 0.34 | none, none declared |
| b + chair | known b | known ×3 | 0.86 | 0/3 found |
| c - ball | known c | known ×3 | 0.86 | 3/3 found |

- **21/21 verdicts, 0 wrong "known", 0 false alarms**; every true match now
  clears 0.75 with room to spare (0.84–0.98) and the never-seen room stays
  new (0.33–0.37).
- **Changes 3/6.** The removed ball was caught every time: the eye, asked
  for a ball at its remembered spot, said no on both looks. The added chair
  was never caught: the eye calls it a *box* (it is one), the first look
  boxed it and the second look, from the same spot after the 30° turn,
  denied it, so the two-look rule held it back. Naming (chair vs box) and
  the confirming look from the same spot are the two open items (B38).
- **Room b is "ambiguous" at enrolment (0.65–0.69) 3/3**: it shares room a's
  walls and the description does not separate them; naming it settles it,
  and it is recognised 3/3 afterwards. The never-seen room's lead over its
  runner-up is only 0.01–0.06: "new" is right here but not by much.
- **Timing**: arrival to verdict 3.7 s median, 4.9 s p95 at 2.0× physics
  (settling, the sweep, the look, the embedding, and the turn + second
  look when one is taken: 14 of the 21 visits). Enrolling room b came
  back new once and ambiguous twice (0.53, 0.58: b shares a's walls), and
  the bench then named it as a new place, as a truthful owner would.

**What changed after it, this round, not re-measured.** (1) The
description map was re-set from this run's records (0.25 / 0.90, above;
`KNOWN_T` 0.75 and `MARGIN` 0.08 kept): replayed on the run's first looks,
all 18 revisits come out known (the weakest 0.779, 0.029 over the line;
under the old map 6 fell below it, the run's 6 ambiguous), the never-seen
rooms at most 0.346 (new), and room b at its enrolment against room a
0.560–0.676 (ambiguous 3/3, never known). That is a replay of recorded
numbers from one run of three trials whose looks repeat, not a new
measurement. (2) The change check asks the eye with box questions instead
of reading the prose (**Changes** above), so the chair and the ball can be
boxed and placed. (3) A new place takes a second look before it is stored.
The place bench has not been re-run since the first run; its next run is
what measures all three.

**Hardening (D052).** The sim thread no longer dies silently: an exception
out of a physics step limps the real legs, fails every waiting HTTP
request with 503 (before, one exception froze every request forever) and
ends the process with exit code 1; `/api/state` carries a `heartbeat`
(`step_age_s`, `loop_age_s`, `alive`, `fatal`). Nothing in the cockpit is
authenticated, so: it refuses a non-loopback `--host` unless
`--unsafe-lan` (which prints that warning); every POST must be
same-origin (an `Origin` whose host:port differs from `Host` is 403 — the
tailnet's `https://x.ts.net:9445` passes, since tailscale serve keeps the
Host) and `Content-Type: application/json` (`/api/voice`: multipart), so a
cross-site form cannot drive the robot; and the `Host` header must be
loopback, `*.ts.net`, a tailnet 100.64/10 address, this machine's name or
`ROCKY_COCKPIT_HOSTS` — a DNS-rebinding page would otherwise pass the
Origin check.

The state feed is server-sent events at 10 Hz; the cameras are MJPEG
streams. The feed (and `/api/state`) carries the newest 12 `events` plus `event_seq`, a count of every event since the cockpit started, so a client sees a new event even when the 12 look the same.
The HTTP API under `/api/` is what the MCP proxy
(`harness/cockpit_backend.py`) and any script can use (send JSON):
`POST /api/tool/goto {"x": 0.3, "y": 0}`, `/api/tool/move {"forward_m": 0.3}`,
`/api/cmd {"line": "walk 45"}`,
`/api/world {"preset": "stairs"}`, `/api/chat {"text": ..., "mode": "local"}`,
`GET /api/model`, `GET /api/gait`, `POST /api/gesture/check|solve|teach`,
`GET|POST /api/memory`, `GET|POST /api/awareness`, `GET|POST /api/place` (D057).
`sim/tests/test_cockpit_api.py` and `sim/tests/test_awareness.py` drive
all of it headless.

## 3c. Toward the real robot: gesture studio, voice, realism, hardware (D051, D052)

The cockpit is where gaits, gestures and sounds are made, and where the
real servos plug in one leg at a time. D052: "the studio cannot author what
the robot cannot do; the bus gets a safe first move".

- **Gesture studio.** Gestures as data: `gait/gestures/NAME.json`, a list
  of keyframes (time, body offset, yaw, per-corner crouch, a raised leg as
  an "arm", reach targets, claw, a chord cue, an easing). Pose with the
  sliders — the sim blends to the pose at the loaded speed budget (no
  snap) and holds it; **snap** a frame, move the time, repeat; scrub; ▶
  plays it once; save. Slider ranges come from `/api/model` (body ±40 mm,
  z −45..+25, yaw ±18°, the params joint limits). Every change is
  **checked** (`pebble_feasibility.check_spec` with the sim's gait and
  MuJoCo model): the verdict line shows PASS / PASS with warnings / FAIL,
  the peak joint speed and which joint against loaded 3.0 / free 4.0 /
  hard 4.7 rad/s, the CoM margin, jumps, self-contacts and a thermal
  warning. A FAIL is neither played nor saved unless **force** is ticked
  (a forced save records `"checked": {"ok": false}`); built-in gesture
  names cannot be overwritten. **Reach**: set the arm leg's hand target
  (x/y/z mm in the ground frame, floor at −h) and **solve** — the
  whole-body solver (`gait/pebble_pose_solver.py`, 1–3 s) finds the body
  lean, height, yaw and arm joints that reach it with the CoM ≥ 25 mm
  inside the planted feet, fills the sliders and previews it; snap to
  keep it. **Teach**: ● record pose stream samples the sim's joint targets
  at 20 Hz (the real legs' measured pose for legs mirrored robot → sim)
  while you pose, play or back-drive; stop turns it into keyframes
  (Ramer-Douglas-Peucker, 2° tolerance, body pose fitted from the planted
  feet — above ~5 mm fit residual, do not trust the body pose) loaded
  into the studio with its verdict. **load draft** opens the brain's last
  `compose_gesture`.
- **Voice & sounds.** Chord-speak plays in the browser; the **chord
  designer** edits the v0.2 voice model (`audio/chordspeak2.py`) and saves
  new words into the lexicon. Unchanged in D052.
- **Gait lab & realism.** The footfall diagram, gait sliders initialised
  from the running gait (params default T 2.0 s, step 24 mm), gait
  **presets** (`gait/gaits/NAME.json`: select + load = `gait NAME`, save =
  `gait save NAME`), the current envelope, and **check** (`check` in the
  console: feasibility of the gait at the current command and at the
  envelope corners). The budget note appears when a command was scaled.
  The **servo model** is ON by default since D052 (50 Hz hold, 20 ms
  latency, slew 4.7 rad/s, 4096-count goals); untick it for ideal
  actuators. The load derate of the slew (rate × max(0.2, 1 − |τ|/stall))
  is OFF since D052 V2 — it counted the torque-speed line twice (the MJCF
  damping + forcerange already carry it); `set servo.load_derate 1` for an A/B.
- **Hardware.** The Feetech bus beside the sim (`sim/hw_bridge.py`). Pick
  the port (or `mock`), **scan**; the leg strip shows present, mirrored
  and **degraded** legs (a leg silent for 10 ticks leaves the mirror,
  greyed) and, while a leg enters, a **soft-entry** progress bar.
  **sim → robot** is refused (with the reason) unless the sim is at a
  planted standstill (NORMAL, no velocity, no gesture or studio pose, no
  goto); entering parks each leg where it is, at 40 % torque and 200 c/s,
  and blends to the sim pose before releasing. While streaming,
  **locomotion is held** (walk, teleop, goto and map clicks are refused and
  the buttons disabled) until real foot contacts exist; gestures are
  allowed after the entry — except the ones that walk (turn_in_place,
  sidestep and their right-hand variants), which are held like a walk
  (D052 V2). Also V2: sim → robot needs the sim at 1× speed (and the
  speed stays locked there), shoves and gait changes are refused, and the
  mirror is **dropped** (the real legs hold their last goal) the moment the
  sim's reflex leaves NORMAL or the sim is reset / its world changes — a
  reaction to something only the sim lived through never reaches the real
  legs. The stream runs at the 50 Hz bus rate (telemetry at 25 Hz); a
  re-armed leg always comes back through a soft entry. The stream-speed slider starts at 200 c/s
  (0 = servo max). A red banner shows a lost port; `hw:` events
  (degraded, lost, stale, cut, nan, rate, entry, limits) appear in the
  panel. The telemetry table adds missed ticks, age and trip reason per
  servo; a tripped row is red with a **re-arm** button (torque on for that
  whole leg — the only re-arm). **apply limits…** previews the EEPROM
  angle limits from the params soft limits + calibration (2° margin) and
  writes them after a confirm (mirror must be off). Center calibration,
  direction, ID assignment, jog and LIMP are unchanged. Everything in this
  panel was verified on the mock only: no servo has been on this
  laptop's bus yet.

## 4. The experiment scripts

Every `sim/run_*.py` is a self-contained experiment that prints a verdict
and usually writes a JSON or a video next to itself. Run them from `sim/`
with `MUJOCO_GL=egl`.

| script | question it answers | typical time |
|---|---|---|
| `run_sim.py` | does the bare gait walk straight | 10 s |
| `run_push.py`, `run_push_reflex_v2.py` | push envelope walking / standing, with and without the brace reflex (D022/D025) | minutes |
| `run_terrain.py`, `run_stuck.py` | rubble crossing and the stuck watchdog's escalating retries (D017/D023) | minutes |
| `run_cliff.py`, `run_cliff_safestop.py` | the VOID detector and the safe-stop path at a table edge (D034) | 1 min |
| `run_gestures2.py` | every gesture in physics with its signature metric asserted (D040) | 1–2 min |
| `run_reflex_fallen.py` | 40 N rim shove → tumble → the recovery policy rights it → walks away, 5 seeds (D042/D048) | 2–3 min |
| `shove_envelope.py` | survivable shove per direction, standing and walking, under the D048 shove model | 3 min |
| `run_odom.py`, `run_slam_lite.py` | legged odometry EKF and the ICP map on the room world (D026/D027) | minutes |
| `run_patrol.py` | a waypoint patrol driven purely through the six harness tools (D043) | 1–2 min |
| `torque_audit.py`, `mass_audit.py` | static servo margins on CAD masses; the mass budget itself | seconds |

Videos: `sim/pebble_sim.mp4`, `sim/pebble_fallen_recover.mp4`, and the
clips the scripts write (`*_voiced.mp4` include chord-speak narration).

## 5. Driving it with words: the harness

The six-tool contract (`docs/MCP_CONTRACT_v0.md`): `say`, `gesture`,
`goto(x, y)` in metres, `stop`, `scan_summary`, `status`, plus
`list_gestures` and `move(forward_m, left_m)` (a goto relative to the
robot, see *Move* in §3b). Vetoes come back
as ordinary results (`{"stopped": "cliff"}`), never exceptions, and the
reflex supervisor and cliff guard run inside `goto` regardless of who is
calling. Every tool, the model-facing extras included, is listed in
`docs/TOOLS.md`, generated from one registry (*One registry* below).

```bash
# text brain, no LLM (regex intent parser), on the mock or the sim
python -m harness.intent --backend sim --once "go forward 30 cm"
python -m harness.intent                                  # REPL

# a local model as the brain (any OpenAI-compatible endpoint; llama-swap here)
./rocky.sh brain                                          # qwen3.6-35b-a3b via llama-swap
python -m harness.local_brain --base-url http://127.0.0.1:8080/v1 --model qwen3.6-35b-a3b

# Claude Code as the brain, over MCP
cp .mcp.json.example .mcp.json     # works as is from the repo root: the command is .venv/bin/python (relative)
./rocky.sh chat                    # then: "wave, then walk to (0.25, 0) and tell me what you see"

# voice: push-to-talk through whisper-server into the intent parser
./rocky.sh voice
```

`.mcp.json` needs no `"cwd"` for Claude Code: `./rocky.sh chat` starts
`claude` in the repo root, so the relative `.venv/bin/python` and
`-m harness.server` resolve there. Clients that start elsewhere (Claude
Desktop, other MCP hosts) need the absolute python path and a `"cwd"`
pointing at the repo.

Which body answers the tools — `ROCKY_BACKEND`:

| value | backend |
|---|---|
| `auto` (the example's choice) | the running cockpit (`./rocky.sh cockpit`, `ROCKY_COCKPIT_URL`, default `http://127.0.0.1:8765`) whenever it answers, else the in-process sim; re-checked on every call and logged to stderr |
| `cockpit` | the cockpit, or refuse to start when nothing answers |
| `sim` | the in-process MuJoCo sim (`harness/sim_backend.py`) |
| `mock` | the millisecond contract, no physics — the default when unset |

The in-process `SimBackend` is **not** the D052 loop. It has no servo
model, no speed budget (`pebble_feasibility`) and no always-on void probe
(that lives in `sim/playground.py`, which the cockpit runs). Its goto runs
the reflex supervisor and the D034 cliff detector as before, and every leg
target is NaN-guarded and rate-clamped to the servo's hard speed (4.7 rad/s)
— nothing more. For D052-true behaviour, drive the cockpit (`auto` picks it
when it is up).

With the in-process sim only: `ROCKY_WORLD=flat|room|cliff` (default
`cliff`), `ROCKY_VIEWER=1` (open the passive window and pace to real
time), `ROCKY_AUDIO=1` (play chord-speak samples), `MUJOCO_GL=glfw` for the
window. With the cockpit these belong to the cockpit, not to the MCP server.

**One registry (D056).** Every tool is defined once, in
`harness/capabilities.py` (`REGISTRY`): its name, kind, parameters, the
text each surface shows, whether it is gated (a spoken line needs the wake
word) and what it requires (`eye`, `memory`, `cockpit`; a backend without
it does not offer the tool). Three things come from it:

- the local brains' OpenAI tool schema (`harness/local_brain.py`, and the
  cockpit's list in `sim/cockpit_brains.py`, which Claude mode converts
  to Anthropic tools), live gesture and chord-word enums included;
- the MCP server's tool list (`harness/server.py`). It is live: it follows
  the backend's current gestures, chord words and capabilities instead of
  the lists read once at start, so a gesture saved in the cockpit's studio
  reaches Claude Code without restarting the server;
- `docs/TOOLS.md`, the reference page: all 19 tools (18 offered to models
  plus the executor-only `turn`), which surface offers each, their
  arguments and descriptions (the local-brain and MCP texts side by side
  where they differ), the envelope and the robot.

`docs/TOOLS.md` is generated and committed; CI fails when it is stale.
After a registry change (or a `params.yaml` change that moves the
envelope), regenerate it:

```bash
python -m harness.capabilities --md --out docs/TOOLS.md   # write the page
python -m harness.capabilities --check docs/TOOLS.md      # what CI runs: exit 1 + a diff when stale
python -m harness.capabilities --json                     # the capabilities snapshot, with its version hash
```

## 6. Regenerating after a CAD change

```bash
cd cad && python run_all_checks.py          # 23/23, regenerates cad/out
cd ../sim && python mass_audit.py            # CAD volumes -> mass_budget.json
python build_mjcf.py                          # -> pebble.xml
python ../ros2/rocky_description/generate_urdf.py && python check_urdf_parity.py
python run_sim.py                             # does it still walk
```

The hip axis height, leg lengths and joint limits all come from
`params.yaml`; nothing in `sim/` or `gait/` carries its own copy.

**Robot description (D053).** The robot's shape is data too. The `robot:`
block in `params.yaml` holds:
- the leg count and station angles
- the per-leg joint chain: names, axes, offsets that alias `l1_coxa` /
  `hip_axis_z` / `l2_femur` / `l3_tibia`
- the foot point, the claw as a tool, and the IK solver

`rocky_model.robot()` turns the block into a `RobotSpec`. `validate()`
refuses a spec the code cannot run. `python gait/rocky_model.py` prints it
with its topology hash (`7f066d9bd8c0` today).

What already reads the spec:
- `pebble_gait.N_LEGS` / `STATION_DEG`
- the MJCF's station angles, leg count and actuator order
  (`rm.actuator_order()` is the `ctrl[:15]` / `ctrl[15:20]` contract)
- the URDF's stations

Tests pin the rest. The compiled links must equal the spec, the URDF joints
must equal it, and a stale `pebble.urdf.xacro` now fails a test instead of
only CI.

**Not wired yet** (backlog B36, steps 2-10 of `docs/ROBOT_AS_DATA.md`):
- The generators still build the leg from a yaw/hip/knee template, so an
  added joint validates but never reaches the MJCF.
- The playground and RL envs still index actuators by position.
- Checkpoints do not record the topology.
- IK is the closed-form 3-joint solver.
- The driver knows only yaw/hip/knee.

For a design change today (a servo, lengths, foot, material, a joint or a
leg), follow **`docs/DESIGN_CHANGE_GUIDE.md`**.

## 7. Known gaps

Hardware and the onboard loop (B32, B35):
- **No real servo has been on the bus.** The hardware bridge's soft
  entry, fault cuts, re-arm rules and EEPROM limits are tested on the
  byte-faithful mock only; GOAL_SPEED 0, torque-enable heading to the last
  goal, behaviour at an angle limit and the POSITION_OFFSET sign are
  VERIFY-ON-BENCH.
- **The tailnet path is unexercised** (`rocky.sh tailnet`: tailscale was
  logged out; the request guard passes a `*.ts.net` host only in tests).
- No hardware backend for the harness yet (`harness/hw_backend.py`: the
  same six tools on `rocky_driver.PebbleRobot`), no IMU, foot-switch or
  lidar drivers, no 50 Hz loop on the Pi and no host watchdog; the sim
  provides the signals (`sim_imu`, `perception/contacts`).
- The recovery policy needs torch; a numpy/ONNX inference path is wanted
  for the Pi.
- The ROS 2 packages are a scaffold that has never been `colcon build`-ed
  (`gait_node` now budgets its commands and `hw_bridge_node` uses
  `SoftStream`, both untested under ROS).
- The harness's in-process `SimBackend` is not the D052 loop: it has the
  NaN guard, the 4.7 rad/s clamp and a 3.0 rad/s ramp back after a stop
  or gesture, but no servo model, no `WaveGait.budget`, no probe and its
  own timing-only cliff detector instead of the void guard. The cockpit
  backend (`ROCKY_BACKEND=auto` with a cockpit running) is the D052 loop.

The model (B33, D052 open items):
- ~~The free speed budget (4.0 rad/s) exceeds what any MJCF joint can
  reach~~ — closed by the D052 amendment: forcerange = stall (peak), an
  unloaded joint reaches 4.71 rad/s, the continuous budget is a thermal
  model (§1); the strict xfail is now a passing test.
- Tracking lag p95 is 16.6–18.7° on every gait row of `audit_gestures`
  (warn at 10, fail at 20): the sim is close to saying the gait asks more
  than the servo follows.
- Link CoMs and inertias are primitive shapes, the SEA spring is rigid,
  no backlash; μ 0.8, the thermal fraction 0.65, every SCS0009 number and
  the switch forces are guesses.
- The shove envelope on the peak model (D052a): standing 29–≥ 37 N by
  direction on a 1 N grid (floor 1.11 BW at 0° and 180°; the 5 N grid of
  `sim/shove_envelope.py` rounds it to 25 N, 0.95 BW), walking 20–35 N
  (min 0.76 BW, mean 1.08). The 1.911 N·m clip tipped 2–3 N later on the
  same tree: more torque makes Pebble slightly *easier* to tip (one probe
  says the clipped legs yielded into a slide). Neither is calibrated.
- The thermal proxy never trips from cold inside an 8 s gait or 6 s
  recover episode (a 30 s wave-gait walk at 45 mm/s stays near zero
  heat), so training only meets a derated joint through `thermal_heat0`
  warm starts, an env argument that is off by default and has no trainer
  flag yet.
  Between 0.65 and ~0.77 × stall it trips slowly (800 s at 0.70) where the
  driver mock never does: conservative there. 0.65 / 0.85 / 180 s need a
  10-minute bench hold with a temperature log.
- Still only on fingerprint `5a32f772ca99`: the 6 s envelope runs (walk
  231 mm, turn 103°), `recover1` with the nominal / randomised servo and
  its jitter, `recover5_v3_warm`, and the RL smokes. `recover1`'s default,
  legacy-handoff and system evals were re-measured on the peak model and
  did not move beyond noise (RL_GUIDE §4).
- The tracked `push_results.json`, `push_reflex_results.json`,
  `push_reflex_v2_results.json`, `terrain_results.json` and
  `cliff_safestop_results.json` predate the spawn-pose fix (every leg
  spawned in the wrong pose, a 9.8° spawn tilt that every `tilt_max`
  reported) and the peak clip: historical until regenerated, and
  `run_push_reflex_v2`'s TRIP = 1.8 was calibrated on that bad spawn.
- Servo heat: the RL envs derate a hot joint; the playground / cockpit
  only account it (`guard_status()['thermal']`, the `servo heat` chip,
  sim2real refused past the budget); the studio and `compose_gesture`
  judge kinematics only — the thermal verdict is `audit_gestures`'.

Behaviour:
- **The void guard still falls in a narrow band of approach angles**
  (D052a). The touchdown gate (D052 P2) fixed most of the 10–20° misses,
  but a 1° grid (walk 15/25/35/45 × −30…60°, 310 approaches) still has 7
  falls with the late gate (45 mm/s @ 12.5–14.5°, 25 @ 15–16°, 35 @
  20–21°, 15 @ 17.5°) and 9 with the careful walk (`set gate.wait 1`), at
  other angles (14–20°): a leading foot lands on the edge's lip and gives
  way while the next leg swings. The foot switch cannot see this: a foot
  sphere centred 0–9 mm past the edge sits on the corner, closes its
  switch, then slides off. Pinned as a strict xfail
  (`test_void_guard_lip_band_known_gap`). Outside the band the guard
  stops with room to spare (walk 45 at 0–45°: max torso x 166–230 mm on
  the 0.35 m platform, 0 falls), and the retreat, now the approach played
  backwards, backs off 39–59 mm at walk 45 (14–28 mm at walk 25).
- The careful walk (`set gate.wait 1`) cuts the old 72-approach sweep
  from 6 falls to 1 but walks 17 % slower on every surface (flat 0.625 →
  0.521 m in 15 s); whether it becomes the default is the owner's call.
  Even the default gate costs 2.6–4 % of distance on rough terrain and
  stairs (occasional holds) and 5–6 holds per step-down run.
- The touchdown gate sets the supervisor's gait clock from the playground
  and reads its private `_last_t`; `gait/pebble_reflex.py` has no API for
  it yet, so the Pi's loop (B35) cannot reuse it as is.
- Goto's reactive layer is not a planner: a 0.25 m-wide wall ends it
  `blocked` (the sidestep makes ~25 mm/s), and anything below the lidar
  plane ends it `stuck`.
- On its back with **no** righter, the supervisor loops FALLEN → RIGHTED →
  FALLEN every stall period (3 s) and never stands.
- The probe constants (settle 0.12 of the stance, 3.5 mm lead) are tuned
  to this sim's slow actuator and need bench values.
- Every RL checkpoint on disk is pre-D052 (`legacy obs`); the walkers have
  no speed envelope on the D052 servo and are zeroed; no righter earns a
  handoff by itself on the D052 model (RL_GUIDE §4).
