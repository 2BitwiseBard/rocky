# Reinforcement-learning guide

How Pebble's learning stack works and how to run it: the ideas under it
(§0), the two environments (§1), what a run produces (§2), training,
evaluation and resume (§3), what the results say (§4), the next
experiments (§5), the playground and cockpit hooks (§6), gotchas (§7) and
further reading (§8).

## 0. Background

**The problem.** A Markov decision process: state s (what the physics is
doing), action a (what the servos are told), reward r(s, a), and a policy
π(a|s) tuned to maximise the expected discounted return E[Σ γᵗ rₜ]. Three
things make it hard: the state is continuous (~40-60 dimensions), the
physics cannot be differentiated through, and naive gradient estimates have
huge variance. The policy-gradient theorem gives the one identity everything
rests on: ∇J = E[∇ log π(a|s) · A(s, a)], the log-likelihood gradient of
each action weighted by its *advantage* A (how much better it turned out
than the policy's average from that state). Every trick below reduces the
variance of that estimate or controls the step size.

**The learner, `sim/train_ppo.py`** (PPO, ~450 lines, no framework). Each
piece and why it is there:
- *Vectorised envs* (`--num-envs`): N sims in parallel processes, which
  decorrelates the batch and uses the cores.
- *GAE* (γ 0.99, λ 0.95): the advantage estimator, a geometric blend of
  n-step returns (λ → 1 unbiased and noisy, λ → 0 biased and smooth).
- *The clipped surrogate* (`--clip 0.2`): optimise the importance-weighted
  objective but clip the ratio π_new/π_old, so one batch cannot drag the
  policy further than a trust region. `--target-kl` stops an epoch early
  when the measured KL divergence jumps.
- *Observation and return normalisation* (`RunningMeanStd`, Welford):
  networks want unit-scale inputs, and rewards are scaled by the std of the
  *return*, so the shaping scale does not set the learning rate.
- *Entropy bonus* (`--ent-coef`): keeps π from collapsing early.
- *`--log-std-init`*: the exploration knob. On a residual policy the noise
  starts quiet (−1.0): loud noise degrades the good controller and the
  gradient learns "mute yourself" instead of "improve" (D031).
  *`--log-std-max`* caps sigma from above (D045, §4).
- Atomic checkpoints, a jsonl log and exact RNG resume, so a run survives
  an interrupted session.

**Domain randomisation** draws each episode's physics from stored base
values, never by multiplying the live model (which would random-walk the
physics, D031). It is why sim numbers are honest ranges rather than one
lucky rollout, and why a return under DR cannot be compared with a return
without it.

**Honesty baselines.** Every evaluation runs a baseline on the same seeds:
the bare gait (`--compare-zero`) for a residual policy, hold-pose and random
for the righter. A policy that cannot beat its baseline is a negative
result, and negative results are recorded (§4, `BUILD_LOG.md`).

## 1. How it works

**One trainer, two environments.** `sim/train_ppo.py` (§0) trains either:

| `--env` | class | what the policy controls | starting point |
|---|---|---|---|
| `gait` (alias `walk`) | `sim/rocky_env.py` `PebbleEnv` | a **residual**: 15 joint-target offsets (±0.25 rad) added to the analytic wave gait's targets | policy zero already walks; PPO learns trims |
| `recover` | `sim/rocky_recover_env.py` `RecoverEnv` | **absolute** joint targets in the params joint range, from a fallen pose | nothing; there is no analytic righter |

**The D052 contract.** Every stage between the policy and the joint is in
the training loop, and the observation holds only what the Pi can read.
(Before D052 both envs trained on an ideal actuator: the recover target
slewed at 5 rad/s, above the ST3215's 4.7 no-load, and the recover
observation held world torso height, MuJoCo joint velocity and a foot count
that included self-contacts. Those checkpoints are "legacy", §2.) The shared
pieces live in `sim/rl_common.py` and `sim/sim_imu.py`, and the deployed
righter adapter (`sim/righter.py`) uses the same objects, so the policy
sees on deployment what it trained on.

The action path, per 50 Hz control tick:

1. `clip(a, -1, 1)`, then an **EMA filter** (`--ema-alpha`, 0.4). The filter
   state is in the observation, so the policy knows about it.
2. Mapped to targets: a residual on the gait (gait env), or into the
   `joints.pos_deg` range (recover env; since D064 the hips of legs 1–4 map
   over [−51.05°, 90°], the righter's hip floor, below).
3. Recover only: a **command clamp** of `--rate-limit` rad/s (4.0 by
   default, the params "free" speed; the legacy checkpoints used 5.0, the
   D048 v3 runs 3.0).
4. The **servo model** (`sim/servo_model.py`) on every 2 ms physics
   substep: 50 Hz hold, bus latency, slew at `rate` and 4096-count goals.
   The goal is not slowed by load (D052 V2): the MJCF's joint damping
   carries the torque-speed line, and `ServoModel.set(load_derate=True)`
   restores the old derate for an A/B.
5. Plus a per-joint calibration offset, then into the MuJoCo position
   actuator (joint damping 0.626 N·m·s/rad, forcerange 2.94 N·m — the
   stall/peak torque — × voltage, derated toward the continuous 1.91 N·m
   by `rl_common.ThermalProxy` when a joint's heat passes its budget; the
   heat is integrated from the ELECTRICAL torque, force − damping × qvel).

`--servo` selects step 4: `off` is the pre-D052 ideal actuator; `nominal`
is the datasheet servo (20 ms, 4.7 rad/s); `random` (the default) draws
per episode latency U(10, 40) ms, slew U(2.0, 4.7) rad/s and one "battery
voltage" v ~ U(0.85, 1.0) that scales both the slew and the forcerange.

**Observations** (obs version 2; each checkpoint records its version):

| env | dim | contents |
|---|---|---|
| recover | 57 | gravity direction (3) and gyro (3) from `sim_imu` · joint q (15): quantised to encoder counts, one tick late · joint qd (15): finite difference of that q at 50 Hz, one tick late (the Pi has no velocity sensor) · 5 foot switches (floor only, 2.0/1.0 N hysteresis) · tilt/π · the filtered action (15) |
| gait | 56 | the same IMU, q and qd · gait phase sin/cos · the command the gait runs (vx/60, vy/60, wz/0.6; slewed since B34, below) · the filtered residual (15) |

There is no torso height in the recover observation: the robot cannot
measure it. Height stays in the *reward*, which is allowed to be
privileged. Gravity is `model.opt.gravity`, not world z, because an
accelerometer sees a slope. With DR on, the observation also gets noise:
gravity σ 0.02, gyro σ 0.02 rad/s plus a per-episode bias of ±0.02,
encoders ±1 count, qd σ 0.1 rad/s, a per-episode foot-switch dropout
p ~ U(0, 0.1), and a 5 % chance of one stuck switch. These noise levels
are guesses, not measurements. Obs version 1 (41 / 39 values, true state)
is kept only so old checkpoints replay.

**Rewards.** Gait: velocity and yaw-rate tracking (of the command the gait
runs), −0.02·|a|², −0.1·|Δa|²
(the raw action), tilt (weight 1.2), |h − stance height| (from
`rocky_model`), −5 for a fall, +1 for surviving the episode. Recover `v1`:
uprightness + height + a standing bonus, success when standing is held 1 s.
Recover `v2` (D041): success is the handoff criterion held 0.5 s, plus a
feet-down term while upright. Recover `v3` (D048): v2 plus
0.1·mean((Δtarget / rate limit)²). Every version has −0.01·|a|² − 0.2·|Δa|²
on the raw action. Both envs also pay −0.5 × the mean thermal derate over
the 15 joints (below); it is 0 unless a joint has run past its heat budget.
`REWARD_REV` is `D052`.

**Servo heat** (D052a, `rl_common.ThermalProxy`). The actuator clips at
the stall (peak) torque, so the continuous budget (0.65 × stall) is
enforced over time instead: per joint, heat += ((|τ_e| / stall)² −
0.65²)·dt, clamped at 0, integrated once per 50 Hz tick from the mean over
its substeps, with τ_e = actuator force − damping × qvel
(`rl_common.motor_torque`, the motor-current term; the raw force would
count back-EMF as heat). Budget (0.85² − 0.65²) × 180 s = 54, from
`actuators.st3215.thermal_trip_frac` / `thermal_trip_s`: 3 min at
0.85 × stall or 93.5 s at stall trips it. Past the budget that joint's
forcerange ramps from peak down to 0.65 × peak as heat goes 54 → 64.8 (on
top of the episode's voltage draw), so a hot joint cannot exceed the
continuous torque and cools once the load drops. Heat and trips are in
`info` (`thermal_heat_max` as a fraction of the budget, `thermal_tripped`,
`thermal_derate_max`) and in `config()['thermal']`. **From cold nothing
trips inside an 8 s gait or 6 s recover episode**; the env argument
`thermal_heat0=(lo, hi)` (fractions of the budget, per joint) warm-starts
episodes, default (0, 0) — the rng is only drawn when hi > 0, so seeded
episode streams are unchanged — and `train_ppo` has no flag for it yet.

**The handoff criterion is hardware-computable** (D052):
`rocky_recover_env.handoff_ok(tilt_deg, feet_contacts, hip_knee_q)` is
true when tilt < 25°, at least 3 foot switches are closed, and the joint
angles of those legs put the deck at least 90 mm above them (the level-body
kinematic height). It uses only numpy and does not import mujoco, so the
supervisor on the Pi can call it. The old test (tilt < 25° and torso height
> 0.09 m) needs the true height, and it passed a robot kneeling on its
shins with one foot down: that is where most of `recover1`'s old
"handoffs" were (§4).

**Pushes** (gait env). With probability `--push-prob` (default 0.7) per
episode, one `sim/shove.py` shove: a half-sine at the shell rim, peak
U(10, 35) N (a range that spans the tip threshold), lasting U(0.3, 0.5) s,
in a random direction, starting between 1 s and 6.5 s, applied inside the
physics substeps. (Tipping the robot takes 6–8 N·s; the pre-D048 push was a
one-tick N(0, 12) N tap at the CoM, about 0.24 N·s.)

**The command slew** (gait env, B34). The budgeted command reaches the
gait through `pebble_gait.CommandSlew` (D063: 25 mm/s² on the fastest foot,
then a 0.2 s lag), as in the supervisor, the playground, the harness sim and
`gait_node`. An episode starts standing and the command eases in: at the
default 45 mm/s ask (budgeted to 34.2) it reaches the target at 2.36 s, the
largest joint-target step is 3.38° per tick (20.04° unslewed) and the
zero-action robot walks 228 mm in 8 s (255 unslewed; servo nominal, seed 0,
measured 2026-09-30). The observation and the velocity reward use the
slewed command, which is what the cockpit feeds a walker (`cmd_eff`).
`cmd_slew=False` is the pre-B34 pass-through; a checkpoint without
`cmd_slew` in its `env_config` replays that way and is flagged
`unslewed cmd`. The recover env has no velocity command.

**The side → back curriculum** (recover env, B34; `--curriculum side-back`,
training only). No checkpoint rights the robot from its back. With the
curriculum, a drop that lands more than 110° from upright is kept with
probability *reach* and otherwise drawn again (mode, pose and joints; the
episode's DR and servo draws stay). *reach* ramps 0 → 1 between the two
fractions of `--total-steps` in `--curriculum-ramp` (default `0.15,0.75`);
the trainer sets the progress before every rollout and logs `reach`. The
test is on the landed tilt because the drop's orientation does not decide
it (a side drop landed past 120° 29 % of the time over 120 seeds). Landings
past 110° at reach 0 / 0.25 / 0.5 / 0.75 / 1: 0 / 23 / 33 / 40 / 46 % of
120 (servo random + DR), 2.06 drops per reset at reach 0. At reach 1 nothing
is redrawn and the episode is the default env's, draw for draw; evaluation
never uses the curriculum, so every number in §4 is on the full fall mix.

**The belly and the hip floor** (recover env, D064). The sim carries the
keel tub and the hub shelf under the deck (SIM_GUIDE §1), and the supervisor
holds the righter's hip command on legs 1–4 at ≥ −51.05° in FALLEN and at
the RIGHTED ramp's start, so no righter folds a leg under the keel (SIM_GUIDE
§3). New runs train on the same floor: `--hip-clamp params` (the default:
`rocky_model.righter_hip_min_deg()` on `righter_clamp_legs()`, from
`reflex:` in params) maps the policy's [−1, 1] for those hips over
[−51.05°, 90°] instead of the full range, so no band of actions all clips to
the floor and the policy can command nothing the supervisor would cut;
`--hip-clamp none` is the uncut map every older checkpoint trained on (to
continue one, or for an A/B), and a number sets another floor. The map goes
into `env_config` as `hip_floor_deg` / `hip_floor_legs`; a checkpoint
without them replays the uncut map (`eval_recover.env_for`) and the
supervisor clamps its output. **Drop poses clear of the belly** (review 9q):
a drop's joint draw that starts a leg inside a belly geom is drawn again
(`RecoverEnv(drop_clear_of_belly=True)`, the default, in `config()`; 15 of
400 of the old stream's drops started inside the tub or the shelf, up to
18.8 mm deep, and MuJoCo landed them with a penetration impulse). A drop that
needs no redraw consumes the rng exactly as before (58 of 60 tested seeds land
identically). `--belly-drops` keeps the old stream, and a contract without
the key replays it.

**Domain randomisation** (on by default, `--no-randomize` to disable;
`rl_common.DomainRandomizer`). Every episode draws fresh values from the
stored base values:
- friction μ U(0.5, 1.5); the base is 0.8
- per-link mass ×U(0.85, 1.15), inertia with it
- torso CoM shifted ±15 mm in x and y
- per-servo kp ×U(0.7, 1.3), applied to gain and bias together
- forcerange × the voltage draw
- per-joint zero offset ±1°
- gravity tilted by up to 3°

The servo latency replaces the old 0–1 tick action delay. `--cmd-sample`
re-draws the velocity command every episode. Under D052 physics, wz 0.35
while walking only reaches about 0.25 rad/s, and the tracking kernel simply
pays less there.

**The deployment idea.** The residual gait policy is a bounded trim on a
controller that already runs on hardware. The recovery policy hands over to
the analytic planted stance once the handoff criterion is met
(`gait/pebble_reflex.py` FALLEN → RIGHTED, D042). The supervisor uses
`handoff_ok` itself whenever it is fed the measured joints and foot
contacts (`step(..., contacts=..., q_meas=...)`): the playground (and so the
cockpit), `eval_recover.py`, `audit_righter.py` and
`sim/experiments/run_reflex_fallen.py` feed both. A caller that passes no
`q_meas` (the in-process MCP `SimBackend`, some experiments) still gets the
privileged tilt + torso-height test, which the Pi cannot compute. Neither
policy replaces the analytic stack; both are gated by it.

## 2. What a run produces

`--run-dir runs/NAME` holds:

| file | content |
|---|---|
| `latest.pt` | the checkpoint: model, optimizer, normalisers, RNG state, `global_step`, `update`, the args, and **`env_config`** (below) |
| `train_log.jsonl` | one JSON line per update: `step`, `ep_return`, `ep_len`, policy/value/entropy losses, `kl`, `kl_stop`, `lr`, `sps`, and `reach` on a curriculum run |
| `train.out` | stdout, only if you redirect it: `mkdir -p sim/runs/NAME && ./rocky.sh train-recover NAME > sim/runs/NAME/train.out 2>&1 &`; `rocky.sh jobs` prints its last line |

`env_config` is the env's own `config()`: obs version, dim and names,
action mapping, rate limit, EMA alpha, servo mode and ranges, DR ranges,
obs noise, reward version, `reward_rev` and weights, the handoff
definition, the gait parameters (gait env), the hip floor and the drop rule
(recover env: `hip_floor_deg`, `hip_floor_legs`, `drop_clear_of_belly`),
and the **robot fingerprint**
(`sim/model_fingerprint.py`), MuJoCo version and params revision. The
evaluators and `PolicyRighter` read it back with
`rl_common.checkpoint_contract()`:
- A checkpoint whose observation this code cannot build is **refused**
  with a clear error (unknown obs version, or a dim that disagrees with it).
- A fingerprint mismatch only **warns**: the robot changed since training,
  so re-evaluate before trusting the checkpoint.
- A pre-D052 checkpoint (no `env_config`) replays under the legacy
  contract (obs v1, ideal servo, no filter, 5 rad/s, and for the gait env
  the old T 1.6 s / 32 mm gait) and is flagged **`legacy obs`**. A rate
  limit above 4.7 rad/s is flagged **`exceeds servo`**. A gait checkpoint
  trained on commands outside the gait's envelope is flagged
  **`unbudgeted cmd`**, and one trained without the command slew
  **`unslewed cmd`** (every walker on disk is both).
- A legacy checkpoint cannot be resumed under obs v2, and a recover
  checkpoint cannot be resumed under another hip map
  (`rl_common.action_map_mismatch`); the trainer says so.

Reading the log: `python sim/rl_dashboard.py` prints a table of every run
(env, reward, rate limit, obs version, servo, steps, last
return/length/entropy, flags, recorded eval) and draws the curves of the
tracked runs to `sim/out/rl_curves.png` (`--table` prints only). The
playground's `rl` command prints the same table live. What to look for:
- `ep_return` rising is good.
- On the recover env, `ep_len` dropping below the 300-step ceiling means
  episodes are ending in success.
- `ent` (entropy) climbing without bound is the D045 failure (sigma
  inflating into bang-bang control); cap it with `--log-std-max -0.5`.
- On a curriculum run the early returns come from easier episodes (side
  landings only until `reach` leaves 0), so compare returns with a flat run
  only after `reach` has reached 1.

## 3. Step by step: train, evaluate, resume

Install torch first: `pip install -e ".[sim,rl]"`. CPU works for
everything below; a GPU only speeds up the long runs.

The `python` commands in this section and in §5 run from `sim/` with the
repo venv active (the `rocky.sh` recipes `cd` there themselves); §2's
commands and the `./rocky.sh` lines run from the repo root.

**Smoke test (under a minute each).** This proves the pipeline end to end:

```bash
cd sim
python train_ppo.py --env gait    --num-envs 2 --rollout 96 --total-steps 2000 --run-dir runs/smoke_gait
python train_ppo.py --env recover --reward v2 --log-std-max -0.5 \
                    --num-envs 2 --rollout 96 --total-steps 2000 --run-dir runs/smoke_recover
```

The first line prints the contract: servo mode, EMA, obs version and dim,
and the robot fingerprint. The defaults are the D052 recipe:
`--servo random --ema-alpha 0.4 --rate-limit 4.0 --push-prob 0.7`, DR and
obs noise on. `--servo off --ema-alpha 1 --rate-limit 5 --no-randomize` is
the closest you can get to the old ideal-actuator env, but it still uses
obs v2.

**Evaluate** (deterministic: the actor mean; every checkpoint replays
under its own contract):

```bash
python eval_ppo.py runs/smoke_gait/latest.pt --episodes 5 --compare-zero     # vs the bare gait, same seeds
python eval_ppo.py runs/robust_fwd2/latest.pt --servo nominal --compare-zero # a legacy policy with the servo on
python eval_recover.py runs/smoke_recover/latest.pt --episodes 20            # vs hold-pose / random
python eval_recover.py runs/recover1/latest.pt --randomize                   # + servo-random/DR/noise column
python eval_recover.py runs/recover1/latest.pt --supervisor                  # the SYSTEM number
MUJOCO_GL=egl python eval_ppo.py runs/robust_fwd2/latest.pt --video walk.mp4 # write a clip
```

`eval_ppo.py` resets episode *k* with `seed + k` for both the policy and
the zero-action run, so with `--randomize` or `--push-prob` both columns
see identical draws. A residual policy has to beat the zero column to be
worth deploying. Besides distance, it reports mean |residual| (of the
clipped action) and mean |Δresidual| per tick (chatter). `--stochastic`
samples actions instead of taking the mean.

`eval_recover.py` reports two kinds of number:
- **stood-after-handoff (hybrid).** The policy rights the body; once
  `handoff_ok` has held for 0.5 s, the analytic planted pose ramps in
  through the same servo path. `--handoff legacy` restores the old
  privileged height test.
- **pure-RL success.**

Each comes with the end-tilt distribution, a per-fall-mode breakdown and
the hold-pose and random baselines on the same seeds.
- **Conditions.** `nominal` is the checkpoint's own servo, made
  deterministic, with no DR. `--servo X` overrides that servo.
  `--randomize` adds a servo-random + DR + noise column.
- **`--supervisor`.** Runs each seed's fall through 14 s of raw MuJoCo
  under `ReflexSupervisor` + `PolicyRighter`: fall detection, the 3 s
  stall rule, the 10 s deadline, the ramp. A `no-righter` row runs the
  same falls with no policy installed. This is the number to quote for the
  system. The hybrid number is policy-to-handoff only.

`audit_righter.py CKPT [--servo nominal]` measures jitter over the FALLEN
phase of the shove demo (5 falls by default): pinned fraction,
reversals/s, |move| per tick, track error, **ctrl rev/s** (reversals of the
actuator command *after* the servo filter) and **qvel flip/s**
(joint-velocity sign flips, 0.2 rad/s deadband). With the servo in the
loop, "pinned at the rate limit" no longer means jitter; the last two
columns do.

**A real run** (the recipes are also in `rocky.sh`):

```bash
./rocky.sh train-walk    walk1                      # residual gait, 8 envs, DR on
./rocky.sh train-recover recover8 --total-steps 3000000
./rocky.sh jobs                                     # what is training, + the last train.out line
./rocky.sh eval-walk     walk1
./rocky.sh eval-recover  recover8
```

A run that must outlive the terminal goes in a transient systemd unit,
with a CPU quota on a shared machine. B34's `recover7_d063_curriculum` was
started this way (from the repo root):

```bash
mkdir -p sim/runs/recover7_d063_curriculum
systemd-run --user --collect --unit=rocky-train-recover7 -p CPUQuota=800% \
  --working-directory="$PWD" sh -c './rocky.sh train-recover recover7_d063_curriculum \
  --curriculum side-back --total-steps 12000000 --device cpu --torch-threads 2 \
  > sim/runs/recover7_d063_curriculum/train.out 2>&1'
systemctl --user status rocky-train-recover7     # running? `./rocky.sh jobs` shows its last line
```

Defaults are overnight-sized (`--total-steps`, `--num-envs 8`,
`--rollout`). Pass `--sync` to debug with a single process, and
`--device cpu` to keep the GPU free. The trainer pins gymnasium's vector
autoreset to `SAME_STEP` (`rl_common.make_vec_env`). The gymnasium ≥ 1.0
default, NEXT_STEP, fed PPO one transition per episode boundary whose
action was ignored and whose reward was 0.

**Resume.** `--resume auto` picks up `<run-dir>/latest.pt`, or pass a
path. The resume is exact: optimizer, normalisers and RNG all come back.
`--total-steps` is the *cumulative* target, so to add 2 M steps to a 2 M
checkpoint, pass `--total-steps 4000000`. Keep `--rollout` and
`--num-envs` as in the checkpoint. If the target allows no new updates,
the trainer says so and exits. A checkpoint whose obs version differs
from the env's is refused (every pre-D052 run; warm-starting from
`recover1` is not possible), and so is one whose actions mean something else
here: a pre-D064 recover run trained on the uncut hip map, so continuing
`recover6_d052` or `recover7_d063_curriculum` needs `--hip-clamp none`
(their uncut map; the supervisor still clamps them when they run).

**Ctrl-C** saves a checkpoint before exiting.

## 4. The results so far (honest table)

Checkpoints ship in `sim/runs/` (git-lfs). "Stood" = stood-after-handoff
over 20 random falls on the same seeds, deterministic policy. Columns:
- **cloud**: the 2026-09-01 measurement;
- **local pre-D052**: the D047 model, re-measured 2026-09-23 (mujoco 3.12);
- **current model**: `deae868522cc`, D064's belly model (robot 2843.2 g,
  the keel tub and the hub shelf, the supervisor's hip clamp), re-measured
  2026-10-07 for the four righters on disk (the D063 value in square
  brackets, or named, where it moved; the current fingerprint `965f4f70e5d1` is the 9q shell relief,
  0.2 g lighter, plus the deck's north tab holes, 0.02 g heavier, and was not re-evaluated). The walkers' cells stand on
  `7d376178fe27` (2026-09-26); cells marked `5a32…` were measured on the
  first D052 model (`5a32f772ca99`, the 1.911 N·m clip, D052a) and not re-run
  (B68). A "not re-measured" cell is not a zero.

Terms in the current-model column: **legacy** = the old handoff test
(`--handoff legacy`); **hw** = `handoff_ok` (the default); **servo** =
`--servo nominal`; **rand** = `--randomize`; **system** = `--supervisor`
(supervisor + righter + stall ramp, 14 s, on `handoff_ok`), with the
no-righter row in brackets; **jitter** = `audit_righter.py --servo
nominal`. Training return is the last logged `ep_return`.

**Every checkpoint on disk is legacy on the D064 model.** Four by their
contract (obs v1, true-state observation, ideal actuator, 5 rad/s; the gait
ones on the old T 1.6 s / 32 mm gait): the evaluators replay them under it
and flag them `legacy obs, exceeds servo`, and none can be resumed or
warm-started under obs v2. `recover6_d052` and `recover7_d063_curriculum`
(B34) trained on the D052 contract (obs v2), on `ceb63a1254c3` and
`87215110e9c4`, so both load with a fingerprint warning. None of the four
righters trained on the belly or the hip floor: they replay the uncut hip map
and the supervisor clamps them.

| run | env | steps | training return | stood (cloud / local pre-D052) | pure-RL (pre-D052) | current model | note |
|---|---|---|---|---|---|---|---|
| `robust_fwd2` | gait | 3 M | 228.6 | — | — | on `5a32…`: legacy replay 245 mm vs zero 320; servo 236 vs 298; servo random + DR + shoves 216 vs 250 (0/3 falls each); **no envelope** on the D052 servo, so the cockpit zeroes it | legacy obs, T 1.6 gait. Pre-D052: loses to the bare gait on the clean task (298 vs 365 mm); D031: the wave gait is a strong controller. Still loses on D052 |
| `cmd_sample3` | gait, `--cmd-sample` | 7 M | 224 | — | — | not re-measured; no envelope (same gait) | same conclusion under full DR |
| **`recover1`** | recover v1, 5 rad/s | 2 M | 354 | **12/20 / 7/20** | 0/20 | stood **2/20** legacy (side 1/7, tumble 1/7; 4/20 on D063) · **0/20** hw (end tilt median 12.7°, best 1.1° [17.7°, 4.1°]; hold-pose 1/20, random 0/20) · pure-RL **0/20** · servo 0/20 (end tilt 8.1° [12.6°]) · rand 0/20 · system **20/20** (11/20; back 6/6 vs 0/6, side 7/7 vs 5/7, tumble 7/7 vs 6/7), 9 declared falls: 0 handoff, 9 stall, 0 deadline exits, t_stood median 0.46 s (0.40); the clamp clips 43.5 % of FALLEN ticks; over 200 seeds **198/200** (104/200 with no righter; 197 / 87 on D063) · jitter: 70 % pinned, 6.2 rev/s, 4.28°/tick, ctrl 6.9 rev/s, qvel flip 4.9/s, track 9.4°; 5/5 righted, all by the stall ramp (t_right 5.3 s) · in the 200-seed shove demo leg 0 touches the shelf plate in 2 falls (≤ 1.40 mm; leg 0 is outside the clamp) | **the shipped righter**, legacy. Pre-D052 jitter: 75 % pinned, 10 reversals/s, 4.5°/tick. Its legacy "handoffs" were a robot kneeling on its shins with 0–2 feet down and knees at −110…−150° |
| `recover2` | recover v2, uncapped | 3.67 M | ~610 (stochastic) | 2/20 | 0/20 | (checkpoint removed; see git history) | sigma inflated to 22 nats; mean policy decayed (D045) |
| `recover3_capped` | recover1 → v2 capped | 4 M | 543 | — / 7/20 | 0/20 | (checkpoint removed; see git history) | every dimension pinned at the cap; success carried by noise |
| `recover3_scratch` | recover v2 capped, scratch | 3 M | — | — / 3/20 | 0/20 | (checkpoint removed; see git history) | never rights from the back |
| `recover5_v3` | recover v3 capped, 3 rad/s, scratch | 3 M | 215 | — / 2/20 | 2/20 | (checkpoint removed; see git history) | D048 negative: smoothness cost, lower rate limit; back 0/6, side 0/7 |
| `recover5_v3_warm` | recover1 → v3 capped, 3 rad/s | 5 M | 483 | — / 4/20 | 4/20 | stood **1/20** legacy (3/20 on D063) · **0/20** hw · system 20/20 (11/20; the clamp clips 64.8 % of FALLEN ticks) · jitter: 57 % pinned, 3.9 rev/s, 2.01°/tick, ctrl 4.5 rev/s (4.5 on D063, 4.4 before it) · on `5a32…`: servo 0/20, rand 0/20, pure-RL 0/20 | legacy. D048 negative on the handoff (4 < 7) but the smoothest righter so far (pre-D052: 58 % pinned, 7 reversals/s, 2.1°/tick); on D052 its pure-RL 4/20 is gone |
| `recover6_d052` | recover v2, obs v2, servo random, 4.0 rad/s | 3 M | 268 | — | — | **0/20** hw (end tilt median 9.9° [7.5°]) · system 11/20 = no-righter 11/20 · jitter: 69 % pinned, ctrl 9.3 rev/s; in the shove demo it never stood (0/5, FALLEN re-entered 5–6× each) | B34's first obs-v2 run (CPU, 25 min; nominal eval mean return 421 at training, 423 on the current model): the servo model in the loop, IMU/encoder/switch noise, no torso height. It lowers the end tilt but earns no handoff, and the supervisor does no better with it than without. NEGATIVE; `recover1` stays shipped |
| `recover7_d063_curriculum` | recover v2, obs v2, servo random, 4.0 rad/s, `--curriculum side-back` | 12 M | 330 | — | — | **5/20** hw (back 0/6, side 4/7, tumble 1/7; mean return 469.5; 3/20 on D063, its own model, 2026-09-30) · rand 6/20 (side 4/9, tumble 2/2; 4/20) · system **11/20 = no-righter 11/20** (back 0/6, side 5/7, tumble 6/7), 9 declared falls: **0 handoff**, 9 stall, 0 deadline exits · jitter (beside `recover5_v3_warm`, 40 N, 5 seeds): 14 % pinned, 2.7 rev/s, 0.82°/tick, ctrl 2.7 rev/s (5 %, 2.1, 0.29, 1.9 on D063; `recover5_v3_warm` 4.5), but it lands on its back every time and never stands (0/5 upright, FALLEN entered 6× each, leg 0 on the shelf in 4 of 5, ≤ 2.12 mm; `recover5_v3_warm` 5/5 upright, by the ramp) · fingerprint mismatch | B34's curriculum run (CPU, unit `rocky-train-recover7`, 10:46 → 12:28, 1.96 k steps/s averaged; `reach` 1 from 9.0 M steps): the `recover6_d052` recipe + side → back (no landing past 110° for the first 1.8 M steps, back landings phased in by 9 M, the full fall mix for the last 3 M = `recover6_d052`'s whole budget). One knob against `recover6_d052` besides the budget (4× longer, so a slower learning-rate anneal) and the robot (D063's `87215110e9c4`, not `ceb63a1254c3`). The first handoffs on the D052 contract, from side landings (and one tumble on D064), but none survives into the supervisor (every declared fall exits on the stall ramp, and the ramp stands no more than with no righter), and the back is still 0. Its 15 log σ sit at the `--log-std-max -0.5` cap by 6 M steps and stay there (entropy 13.69 at 3.6 M, 13.78 = the cap from 6 M; `recover6_d052` ended at −0.80 … −0.94): the D045 inflation, held by the cap. Misses the rung 9 bar on the handoff exit. NEGATIVE; `recover1` stays shipped |
| **`recover8_d064`** | recover v2, obs v2, servo random, 4.0 rad/s, `--curriculum side-back`, hip floor −51.05 on legs 1–4, belly-free drops (D064 defaults) | 12 M | 456 | — | — | **0/20** hw on the nominal servo (end tilt median 18.8° [6.8°], end height 123 mm: it stands up tilted and never settles inside the hand-off hold; hold-pose 1/20, random 0/20) · **rand 12/20** (back 5/9, side 7/9, tumble 0/2; end tilt median 2.7°) · system nominal **20/20** (no-righter 12/20), 8 declared falls: 0 handoff, 8 stall, 0 deadline, t_stood median 0.53 s, the clamp clips 75.9 % of FALLEN ticks, **belly contacts 0/20** (recover1 2/20) · system with servo random + DR + noise **18/20** (no-righter 8/20), 12 declared falls: **3 handoff**, 9 stall, 0 deadline exits, t_stood median 5.44 s (recover1 on the same model and servo: 19/20, 0 handoff, 12 stall, 7.88 s) · jitter (beside `recover5_v3_warm`, 40 N, 5 seeds, servo nominal): 31 % pinned, 2.5 rev/s, 1.59°/tick, ctrl 2.5 rev/s, track 4.8°, 5/5 upright by the stall ramp (t_right 4.7 s; `recover5_v3_warm` 55 % pinned, 3.9 rev/s, 5.2 s) | The first run on the D064 model `965f4f70e5d1` (unit `rocky-train-recover8`, 2026-10-08 08:26 → 10:06, ~2.0 k steps/s beside a running cockpit; `reach` 1 from 9 M steps; the smoke run `smoke_recover8_d064` first): the `recover7` recipe, nothing else changed but the robot and the D064 defaults. **The first hand-offs under the supervisor** on the D052 contract, with the servo model that carries latency (10–40 ms) and a finite rate: the realistic one. With the latency-free nominal servo it gets up but stands tilted ~19° and the 3 s ramp finishes the job, as for every righter before it; the stall ramp still stands the system 20/20 and the policy no longer touches the belly. Back landings are its weak side (5/9 raw). POSITIVE on the hand-off bar, under the randomised servo only: **SHIPPED** as `righter.default_ckpt()`'s first choice (`recover1` stays as the fallback); the bench's real latency decides which servo model was right |

**What it means.** The PPO infrastructure works and reproduces; the
learned policies are marginal. Self-righting is a hybrid: the policy does
the hard part from a SIDE landing (getting tilt down) and the hand-written
planted ramp does the standing. From the BACK no checkpoint rights the
robot and the ramp does (5/5 in the demo), which is why the supervisor
ramps after 3 s without progress (D048 stall rule). The pre-D052 7/20 was
flattered twice, by an ideal actuator and by a handoff test that could not
tell standing from kneeling: on the current robot and `handoff_ok` only
`recover7_d063_curriculum` earns any (5/20 on D064, 4 of them side landings,
in the plain eval), and **no checkpoint earns a handoff under the
supervisor**. With the legacy righters the system still stands 20/20 against
11/20 without a righter (198/200 against 104/200 over 200 seeds on D064; the
belly's lower CoM lifted the no-righter case from 87), so they leave the body
in a pose the stall ramp can stand (back landings 6/6 vs 0/6), but that
number cannot tell `recover1` from `recover5_v3_warm`, so it is not a righter
score. The two obs-v2 runs do
no better than no righter (system 11/20; `recover7_d063_curriculum`'s back
landings 0/6 where `recover1`'s are 6/6). The v3
runs showed the trade: the smoothness cost halves the staircase but costs
handoffs at this budget. With the servo in the loop the staircase mostly
disappears. `recover8_d064` (2026-10-08, §4) is the installed righter: the
first to earn hand-offs under the supervisor, 3 of 12 falls with the
latency-carrying servo model (none with the latency-free one, where the stall
ramp still stands the system 20/20). `recover1` is the fallback.

## 5. The experiment ladder

In order, each teaching one thing:

0. **Retrain on the D052 contract.** Done twice for the righter:
   `recover6_d052`, 3 M steps, negative (§4), then B34's side → back
   curriculum (§1) as `recover7_d063_curriculum` (12 M steps, 2026-09-30),
   negative too: 3/20 hw from side landings, 0 `handoff` exits under the
   supervisor, back 0/6 (§4). **Done on the D064 model as `recover8_d064`**
   (2026-10-08: `./rocky.sh train-recover recover8_d064 --curriculum
   side-back --total-steps 12000000`, the `recover7` recipe with the D064
   defaults: the belly, the real torso inertial, the hip floor, belly-free
   drops): the first hand-offs under the supervisor, with the randomised
   servo (§4), shipped. Next for the righter: the back landings (5/9 raw),
   and why the nominal servo leaves it tilted ~19° (a longer hold, or train
   with latency 0 in the range); then `thermal_heat0` warm starts. Then the first obs-v2 walker:
   `./rocky.sh train-walk walk1` (budgeted and slewed commands, rim
   shoves) and `eval-walk walk1`; every walker on disk is zeroed by the
   D052 envelope. The gait env slews the command since B34 (§1).
   For a righter, evaluate in this order (from the repo root):

   ```bash
   tail -n 3 sim/runs/recover8_d064/train.out                # "done: ... steps -> ..."
   ./rocky.sh eval-recover recover8_d064                     # hw: handoff_ok, 20 falls
   ./rocky.sh eval-recover recover8_d064 --randomize         # + servo-random/DR/noise
   ./rocky.sh eval-recover recover8_d064 --supervisor        # the system; count `handoff` exits
   cd sim
   python audit_righter.py runs/recover8_d064/latest.pt \
       runs/recover5_v3_warm/latest.pt --servo nominal   # ctrl rev/s, the baseline in the same audit
   python rl_dashboard.py         # the table + sim/out/rl_curves.png (`reach` is in train_log.jsonl)
   ```

   **The bar** (rung 9): `stood-after-handoff` ≥ 1/20 in the hw row, at
   least one `handoff` exit under `--supervisor`, and `ctrl rev/s` no
   worse than `recover5_v3_warm`'s in the same audit (4.5 on `deae868522cc`
   and on D063, servo nominal, 5 seeds). The header must read `robot
   fingerprint match` and `[D052 contract]` (`recover7_d063_curriculum` now
   fails the fingerprint clause too). Then write the numbers into §4 and B34
   either way (a miss is a negative result: `BUILD_LOG.md` too).
1. **Watch before you train.** `MUJOCO_GL=egl python eval_ppo.py
   runs/cmd_sample3/latest.pt --video a.mp4 --compare-zero` and the same
   with `eval_recover.py runs/recover1/latest.pt --video b.mp4`. Read mean |residual|: how loud is the correction the
   network learned?
2. **Read the curves.** `python rl_dashboard.py`, or two lines of
   pandas on `runs/*/train_log.jsonl` (step vs `ep_return`, `kl`, `ent`).
   Learn what a healthy curve looks like before causing an unhealthy one.
3. **One reward knob.** In `rocky_env.py` double the tilt weight
   (1.2 → 2.4), retrain the smoke config at a longer budget, and compare
   tilt in eval: shaping → behaviour. Fresh runs only (no legacy warm
   start).
4. **The exploration lesson.** The same run twice, `--log-std-init -0.5`
   vs `-1.0`; watch the first 300 k steps. This reproduces D031.
5. **Turn DR off** (`--no-randomize`) and compare the eval on the clean
   task (D031). Then the fair comparison: policy and zero-action under the
   same DR (`eval_ppo.py --randomize`).
6. **Mean vs sample (D045).** Compare a checkpoint's training-time return
   in `train_log.jsonl` (sampled actions) with its deterministic eval; for
   a gait checkpoint, `eval_ppo.py --stochastic` runs the sampled policy
   directly. A large gap means sigma is doing the work.
7. **Sigma anneal.** Decay log_std to the floor over the last third of a
   run (a few lines in `train_ppo`), on the gait env and on a capped v2
   recover run, so the mean has to do the work.
8. **A new env.** Rubble recovery, a residual walker on the rubble field
   (`sim/scenes.py` `build_model(amp_mm, seed)`), or the room world with a
   goal.
9. **The righter's jitter** (D048): `audit_righter.py` before and after
   any change. The action filter inside the env shipped in D052 (EMA 0.4,
   state in the observation). **The bar on the current model**: any
   handoff at all — more than `recover1`'s 0/20 on `handoff_ok` and at
   least one `handoff` exit under `--supervisor` (both legacy checkpoints:
   0 of 9) — with ctrl rev/s no worse than `recover5_v3_warm`'s, audited
   side by side with `--servo nominal` (4.5 on `deae868522cc`, 2026-10-07).

**The terrain residual: planned, not built (B160).** The gait residual
(`robust_fwd2`) lost to the bare gait on flat ground, so the next learned
piece is narrower: a per-foot height correction on rough ground, riding the
D065 body leveler's composer (its offsets are added to the gait's foot z
before the probe measures, so the probe and the void guard keep their
meaning). The action is modal, not per joint: a few foot-z modes (k0 a
common offset, k1 / k2 the two tilt modes of the five feet), clipped inside
the leveler's 0–30 mm raise-only window. Seeds come in tiers
(`sim/terrain_bench.py`: bench 0–4, held out 100–199, training ≥ 1000), and
the switches the policy sees go through the same `contact_filter` as the
probe's. A 1 M-step run is the go / no-go; it ships only if a paired
bootstrap on the terrain bench beats S2 (the leveler alone) at τ 0.3, 0.6
and 1.2 s and beats the classical rival, a height loop that re-arms the
probe (B156). `PebbleEnv` stays the flat-ground env; the residual gets its
own.

## 6. In the playground and the cockpit

`./rocky.sh cockpit` has an RL panel: the checkpoint table, a righter
picker, stall/deadline sliders and the training curves, next to the shove
buttons, so a righter comparison is two clicks and a watch
(`docs/COCKPIT_GUIDE.md`, RL panel).

`./rocky.sh play`, then `help rl`: `rl` lists the checkpoints, `righter
NAME` hot-swaps the self-righting policy (or `righter off` for the
analytic ramp alone), `push 40 0` tips the robot so two righters can be
compared on the same fall, and `set reflex.stall_s` / `reflex.fallen_max_s`
tune the handoff. The HUD marker turns red in FALLEN and purple during
the ramp.

## 7. Gotchas

- `MUJOCO_GL` must be set for anything that renders offscreen (`egl` on a
  Linux box with a GPU, `osmesa` without one).
- AsyncVectorEnv forks one MuJoCo per worker; on platforms where fork is
  flaky use `--sync`.
- Checkpoints in `sim/runs/` are git-lfs objects. Name scratch runs
  `smoke_*` or `_*` and git ignores them; any other run dir is committed on
  purpose.
- Training-time success ≠ deployable policy when sigma is doing the work
  (D045): always evaluate the mean, and evaluate early.
- A design change rotates the robot fingerprint: re-evaluate the righter
  (`eval-recover recover1`, then `--supervisor`) before trusting it
  (`docs/DESIGN_CHANGE_GUIDE.md` §0). A change to the hip floor
  (`reflex.righter_hip_min_deg` / `righter_clamp_legs`) also changes the
  recover action map: a resume refuses it, and each checkpoint replays the
  map it trained on.

## 8. Reading list and house rules

Sutton & Barto (free online) ch. 13 for the policy-gradient theorem;
Schulman's PPO paper (arXiv 1707.06347) and GAE paper (1506.02438), both
short and readable after rung 4; "The 37 Implementation Details of PPO"
(ICLR blog track), effectively an annotated version of `train_ppo.py`. For
sim-to-real: the domain-randomisation paper (arXiv 1703.06907); the DR
block here is that idea, small.

House rules that keep results honest (each is a decision): quote push
results in bodyweights and compare same-DR to same-DR (D017, D039);
phase-align comparisons and re-measure before re-engineering (D025); DR
draws from bases, and exploration on a residual starts quiet (D031); check
that the motion you think happened actually happened, with eval videos and
metrics, not returns alone (D035). Negative results go in the BUILD_LOG:
they are the most valuable lines.
