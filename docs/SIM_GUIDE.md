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
| leg actuator forcerange | ±1.911 N·m | 0.65 × stall, the continuous (thermal) budget — a guess, VERIFY |
| loaded / free / hard speed | 3.0 / 4.0 / 4.7 rad/s | 3.0 ≈ continuous / damping (3.05); 4.7 = no-load |
| joint soft limits | yaw ±40°, hip −70…90°, knee −150…−20°, claw 0…55° | the CAD-validated sweeps (D008) |
| foot | 6.5 mm sphere whose surface is the gait's IK foot point (site `foot_tip{i}`); μ 0.8, torsional 0.005 m | μ is a TPU-on-tile guess, VERIFY |

With that, no MJCF joint can move faster than about 3.05 rad/s even
unloaded; that is below the 4.0 rad/s "free" budget the authoring tools
allow, and it is an open owner decision (§7). Before D052 the actuators
were clamped at stall (2.94 N·m) with joint damping 0.05, which let the gait ask for
7.4 rad/s and turn 50 % further than the servo could. `sim/model_fingerprint.py`
hashes the robot (not the world) — `5a32f772ca99` — and every RL
checkpoint records it.

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
harsher, not calibrated: the shove envelope actually got *better* under
it (standing 25 → 30–35 N), which may be flattery in the other direction.

## 2. Step by step: first run

```bash
cd rocky && . .venv/bin/activate                     # or pip install -e ".[sim]"
python sim/build_mjcf.py                              # writes sim/pebble.xml from params + masses
MUJOCO_GL=egl python sim/run_sim.py                   # headless walk, ~10 s
```

Expected:

```
tilt: mean 0.34 deg, max 1.03 deg
walk +X displacement: 264 mm (commanded ~261 mm), lateral drift -37 mm
fell over: no
```

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
- **Voice**: hold 🎤. The transcript always lands in the input box; with
  the wake word ("pebble, …") or no motion in it, it is sent at once;
  otherwise the page says "needs the wake word — press Enter" and only
  your Enter sends it as a (trusted) command.
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
streams. The HTTP API under `/api/` is what the MCP proxy
(`harness/cockpit_backend.py`) and any script can use (send JSON):
`POST /api/tool/goto {"x": 0.3, "y": 0}`, `/api/cmd {"line": "walk 45"}`,
`/api/world {"preset": "stairs"}`, `/api/chat {"text": ..., "mode": "local"}`,
`GET /api/model`, `GET /api/gait`, `POST /api/gesture/check|solve|teach`.
`sim/tests/test_cockpit_api.py` drives all of it headless.

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
`goto(x, y)` in metres, `stop`, `scan_summary`, `status`. Vetoes come back
as ordinary results (`{"stopped": "cliff"}`), never exceptions, and the
reflex supervisor and cliff guard run inside `goto` regardless of who is
calling.

```bash
# text brain, no LLM (regex intent parser), on the mock or the sim
python -m harness.intent --backend sim --once "go forward 30 cm"
python -m harness.intent                                  # REPL

# a local model as the brain (any OpenAI-compatible endpoint; llama-swap here)
./rocky.sh brain                                          # qwen3.6-35b-a3b via llama-swap
python -m harness.local_brain --base-url http://127.0.0.1:8080/v1 --model qwen3.6-35b-a3b

# Claude Code as the brain, over MCP, with the window and speakers on
cp .mcp.json.example .mcp.json     # set "cwd" to the repo; add ROCKY_VIEWER=1 ROCKY_AUDIO=1 MUJOCO_GL=glfw
./rocky.sh chat                    # then: "wave, then walk to (0.25, 0) and tell me what you see"

# voice: push-to-talk through whisper-server into the intent parser
./rocky.sh voice
```

Environment knobs: `ROCKY_BACKEND=sim|mock`, `ROCKY_WORLD=flat|room|cliff`,
`ROCKY_VIEWER=1` (open the passive window and pace to real time),
`ROCKY_AUDIO=1` (play chord-speak samples).

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
- **The free speed budget (4.0 rad/s) exceeds what any MJCF joint can
  reach** (forcerange / damping = 3.05 rad/s). Arm moves and swing yaw
  authored to the free budget lag 13–19° at p95 in physics. Kept visible
  as a strict xfail (`test_model_consistency`) until the owner chooses:
  peak torque plus a thermal model in the sim, or free ≤ 3.0.
- Tracking lag p95 is 16.6–18.7° on every gait row of `audit_gestures`
  (warn at 10, fail at 20): the sim is close to saying the gait asks more
  than the servo follows.
- Link CoMs and inertias are primitive shapes, the SEA spring is rigid,
  no backlash; μ 0.8, the thermal fraction 0.65, every SCS0009 number and
  the switch forces are guesses.
- The shove envelope got better under D052 (standing 25 → 30–35 N); the
  cause (slower, lower gait vs joint damping) is not isolated.

Behaviour:
- **The void guard misses cliffs approached at 10–20°** (D052 V2 review
  finding, not yet fixed); a plain head-on walk stops short (x 0.19–0.20 m
  on the 0.35 m platform).
- The void retreat barely retreats (~20 mm *forward* during a 1.2 s
  retreat in one run): `WaveGait.foot_targets` is position-from-phase, so
  reversing v mid-stance moves the stance feet by Δv·s·T. The safe-stop is
  what saves it.
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
