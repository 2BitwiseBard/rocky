# Simulation guide

How Pebble's MuJoCo simulation is built, what runs on it, how to drive it
by hand and by language model, and what each number is worth. Everything
here was run on 2026-09-22 from a fresh checkout; the commands are the
ones that produced the outputs quoted.

## 1. How it works

**One model, generated.** `sim/build_mjcf.py` writes `sim/pebble.xml`
from two inputs: `cad/params.yaml` (leg lengths L1/L2/L3 = 45/95/135 mm,
body circumradius 110, joint ranges, hip axis height) and
`sim/mass_budget.json` (link masses summed from the CAD tree's STL volumes
by `sim/mass_audit.py`, D039). Five identical legs at 72° stations, three
hinge joints each (yaw, hip, knee), position actuators clamped at the
ST3215's stall torque, a foot sphere per leg, and a torso made of two
stacked cylinders. The URDF in `ros2/` is generated from the same params
and `sim/check_urdf_parity.py` proves the two agree (FK within 0.05 mm,
identical mass).

**The control stack is the real one.** The sim does not have its own
controller. It imports `gait/pebble_gait.py` (closed-form IK, the five-
phase `WaveGait` that turns a body-frame velocity command into joint
targets), `gait/pebble_reflex.py` (the `ReflexSupervisor` state machine:
NORMAL → PLANT → BRACE → RECOVER for safe-stops and shoves, FALLEN →
RIGHTED for tumbles, D034/D042) and `gait/pebble_watchdog.py` (progress
watchdog with escalating step-height retries, D023). The same modules are
what the bench scripts and the ROS bridge feed to real servos.

**Sensing in the sim.** `perception/` supplies a legged-odometry EKF (IMU
+ leg kinematics, with a stillness gate for yaw), an ICP scan matcher, and
the contact-timing cliff detector; `sim/sim_lidar.py` casts a 2D ray fan
that `harness/sim_backend.py` exposes as `scan_summary`.

**Worlds.** Three: `flat` (an open floor, the default for driving),
`cliff` (a table-edge island for the VOID reflex) and `room` (walls,
pillars, a crate, for lidar and patrols). Selected by script flag or the
`ROCKY_WORLD` environment variable.

**What the numbers are worth (the honesty box).** Masses are from CAD, but
servo gains, friction, backlash and bus latency are guesses until a real
servo answers on the bench (D017 re-baseline). The sim is excellent for
logic (does the reflex trip, does the gesture reach, does a gait tweak
break a joint limit) and directional for dynamics (push envelopes,
stability trends). Absolute numbers are "±real-robot-TBD".

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
only fire when the line is empty.

| key | effect |
|---|---|
| `↑` / `↓` (or `Shift+W` / `Shift+S`) | forward / backward velocity ±15 mm/s per tap (max 60) |
| `←` / `→` (or `Shift+A` / `Shift+D`) | strafe left / right ±15 mm/s per tap |
| `Shift+Q` / `Shift+E` | turn left / right ±0.12 rad/s per tap (max 0.5) |
| `SPACE` | safe-stop: zero the command and run PLANT → BRACE → planted idle (D034) |
| `Shift+G` | wave hello (from a standstill) |

In the viewer window only the **arrow keys** drive. Every letter there is
one of MuJoCo's own render toggles (W wireframe, S shadows, A auto-connect,
D static bodies, G fog, Q camera frames, E equality constraints) and SPACE
pauses the viewer, and those bindings fire alongside ours, which is what
"WASD changes the lighting" was (D048). Mouse in the window rotates and
zooms the camera; Backspace there resets the physics state, so avoid it.

**REPL commands** (also usable headless via `--script "a; b; c"`):

| command | what it does |
|---|---|
| `walk VX [VY] [WZ]` | set the body-frame velocity command (mm/s, mm/s, rad/s); change it any time |
| `stop` | the D034 safe-stop |
| `gesture NAME` | `wave bow look_around shake sit turn_in_place sidestep jazz_hands fist_bump beckon` (from planted idle) |
| `say WORD` | play a chord-speak sample (`aplay`/`ffplay`) or print it |
| `set PARAM VALUE` | live-tune `gait.T gait.h gait.R0 gait.duty gait.hstep reflex.trip reflex.stall_s reflex.fallen_max_s`; phase-continuous, so feet don't teleport |
| `show` | current gait params, reflex state, pose, sim time |
| `push FX FY [DUR]` | shove the shell rim: peak N, N, half-sine over DUR s (default 0.4); it prints the impulse in N·s and bodyweights. `push 20 0` sways and braces; `push 40 0` tips it over and the righter takes it from there |
| `record on` / `record off` | capture an offscreen clip to `sim/playground_clip.mp4` |
| `help` / `help rl` | the command list and teleop keys / the RL hooks below |
| `rl` | every checkpoint in `sim/runs/`: env, reward version, rate limit, steps, last return, length, entropy, recorded eval |
| `righter NAME` / `righter off` | hot-swap the self-righting policy (`runs/NAME/latest.pt`) or run with the analytic stall/deadline ramp only |
| `wait S` | (scripts) let S sim-seconds pass |
| `quit` | exit |

Headless example, as run for this guide:

```bash
MUJOCO_GL=egl python sim/playground.py --script \
  "walk 45; wait 2; walk 0 30 0; wait 1.5; stop; wait 1; gesture wave; wait 3; set gait.T 1.2; show; push 30 0 0.15; wait 1.5; quit"
```

It ends with `clean exit, no NaNs`; the playground asserts that nothing it
did produced a NaN, which is the reflex stack's promise.

**What a shove is (D048).** Every push used to be a rectangular force
pulse at the centre of mass, which has no gentle regime: below the
foot-friction limit (about 31 N) the robot is a rigid block, above it the
feet let go and it cartwheels, and the old demo's 120 N × 0.25 s was a
30 N·s strike that launched the robot 11–15 m at 10 m/s. `sim/shove.py`
models a hand shove instead: a half-sine force at the carapace's top rim
(60 mm above the torso frame), so the torque about the feet does the
tipping. Under that model the standing robot survives a 25 N peak and
tips at 30 N (about one bodyweight of peak force, 6–8 N·s); walking,
20–30 N depending on direction (`sim/shove_envelope.py`,
`sim/out/shove_envelope.json`). The push-envelope experiment scripts keep
the old pulse because their JSONs are decision records (D017/D025).

When a shove does tip it, the playground now feeds tilt and height to the
supervisor and installs the learned righter (torch + a checkpoint in
`sim/runs/`; it says so at start-up), so a fall ends in FALLEN → RIGHTED →
NORMAL instead of an upside-down standing pose.

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
`params.yaml` and the tree is regenerated; write keepers in
`NOTES_INBOX.md`.

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

- No hardware backend for the harness yet (`harness/hw_backend.py` is the
  next piece: the same six tools on `rocky_driver.PebbleRobot`).
- No real IMU, contact-switch or lidar drivers; the sim provides those
  signals directly.
- The recovery policy needs torch; a numpy/ONNX inference path is wanted
  for the Pi.
- The ROS 2 packages are a scaffold that has never been `colcon build`-ed.
