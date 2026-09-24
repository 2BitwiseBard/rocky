# Reinforcement-learning guide

The practical companion to `docs/RL_TOUR.md` (which explains the theory
and the experiment ladder). This one is about running it: what the two
environments are, what a run produces, how to train, evaluate and resume,
and what the results actually say. Commands were run on 2026-09-22 from a
fresh checkout; §1-3 restated for the D052 contract on 2026-09-24.

## 1. How it works

**One trainer, two environments.** `sim/train_ppo.py` is a self-contained
PPO implementation (vectorised envs, GAE, clipped surrogate, KL early stop,
observation and return normalisation, atomic checkpoints, exact resume).
It trains either:

| `--env` | class | what the policy controls | starting point |
|---|---|---|---|
| `gait` (alias `walk`) | `sim/rocky_env.py` `PebbleEnv` | a **residual**: 15 joint-target offsets (±0.25 rad) added to the analytic wave gait's targets | policy zero already walks; PPO learns trims |
| `recover` | `sim/rocky_recover_env.py` `RecoverEnv` | **absolute** joint targets in the params joint range, from a fallen pose | nothing; there is no analytic righter |

**The D052 contract: the sim stops flattering the servo.** Until
2026-09-24 both envs trained on an ideal actuator (the recover target
slewed at 5 rad/s, above the ST3215's 4.7 no-load; a gait residual could
flip 0.5 rad in one 20 ms tick) and the recover observation held things
the robot cannot measure (world torso height, MuJoCo joint velocity, a
foot count that included self-contacts). Now every stage between the
policy and the joint is in the training loop, and the observation holds
only what the Pi can read. The shared pieces live in `sim/rl_common.py`
and `sim/sim_imu.py`, and the deployed righter adapter (`sim/righter.py`)
uses the same objects, so "the policy sees what it trained on" holds by
construction.

The action path, per 50 Hz control tick:

1. `clip(a, -1, 1)`, then an **EMA filter** (`--ema-alpha`, 0.4). The filter
   state is in the observation, so the policy knows about it.
2. Mapped to targets: a residual on the gait (gait env), or into the
   `joints.pos_deg` range (recover env).
3. Recover only: a **command clamp** of `--rate-limit` rad/s (4.0 by
   default, the params "free" speed; the v1–v3 checkpoints used 5.0).
4. The **servo model** (`sim/servo_model.py`) on every 2 ms physics
   substep: 50 Hz hold, bus latency, slew at `rate × (1 − |τ|/stall)`
   (it slows under load) and 4096-count goals.
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
| gait | 56 | the same IMU, q and qd · gait phase sin/cos · command (vx/60, vy/60, wz/0.6) · the filtered residual (15) |

There is no torso height in the recover observation: the robot cannot
measure it. Height stays in the *reward*, which is allowed to be
privileged. Gravity is `model.opt.gravity`, not world z, because an
accelerometer sees a slope. With DR on, the observation also gets noise:
gravity σ 0.02, gyro σ 0.02 rad/s plus a per-episode bias of ±0.02,
encoders ±1 count, qd σ 0.1 rad/s, a per-episode foot-switch dropout
p ~ U(0, 0.1), and a 5 % chance of one stuck switch. These noise levels
are guesses, not measurements. Obs version 1 (41 / 39 values, true state)
is kept only so old checkpoints replay.

**Rewards.** Gait: velocity and yaw-rate tracking, −0.02·|a|², −0.1·|Δa|²
(the raw action), tilt, |h − stance height| (from `rocky_model`, no longer
the old +14 mm), −5 for a fall, +1 for surviving the episode. Recover
`v1`: uprightness + height + a standing bonus, success when standing is
held 1 s. Recover `v2` (D041): success is the handoff criterion held
0.5 s, plus a feet-down term while upright. Recover `v3` (D048): v2 plus
0.1·mean((Δtarget / rate limit)²) (0.3 before D052). Every version has
−0.01·|a|² − 0.2·|Δa|² on the raw action (the Δa weight was 0.02). Both
envs also pay −0.5 × the mean thermal derate over the 15 joints (below);
it is 0 unless a joint has run past its heat budget. `REWARD_REV` stays
`D052`.

**Servo heat** (the D052 amendment, `rl_common.ThermalProxy`). The
actuator clips at the stall (peak) torque, so the continuous budget
(0.65 × stall) is enforced over time instead: per joint,
heat += ((|τ_e| / stall)² − 0.65²)·dt, clamped at 0, integrated once per
50 Hz tick from the mean over its substeps, with τ_e = actuator force −
damping × qvel (`rl_common.motor_torque`, the motor-current term; the raw
force would count back-EMF as heat). Budget (0.85² − 0.65²) × 180 s = 54,
from `actuators.st3215.thermal_trip_frac` / `thermal_trip_s`: 3 min at
0.85 × stall or 93.5 s at stall trips it. Past the budget that joint's
forcerange ramps from peak down to 0.65 × peak as heat goes 54 → 64.8
(on top of the episode's voltage draw), so a hot joint cannot exceed the
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
supervisor on the Pi can call it. The old test (tilt < 25° and torso
height > 0.09 m) needs the true height, and it passed a robot kneeling on
its shins with one foot down. That is where most of `recover1`'s "handoffs"
were (§4).

**Pushes** (gait env). With probability `--push-prob` (default 0.7) per
episode, one `sim/shove.py` shove: a half-sine at the shell rim, peak
U(10, 35) N (a range that spans the tip threshold), lasting U(0.3, 0.5) s,
in a random direction, starting between 1 s and 6.5 s. It is applied
inside the physics substeps. The old push was a one-tick N(0, 12) N tap at
the CoM, about 0.24 N·s; tipping the robot takes 6–8 N·s.

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

The old 0–1 tick action delay is gone; the servo latency replaces it.
`--cmd-sample` re-draws the velocity command every episode. Under D052
physics, wz 0.35 while walking only reaches about 0.25 rad/s, and the
tracking kernel simply pays less there.

**The deployment idea.** The residual gait policy is a bounded trim on a
controller that already runs on hardware. The recovery policy hands over
to the analytic planted stance once the handoff criterion is met
(`gait/pebble_reflex.py` FALLEN → RIGHTED, D042). Since D052 the
supervisor uses `handoff_ok` itself whenever it is fed the measured joints
and foot contacts (`step(..., contacts=..., q_meas=...)`) — the playground
(and so the cockpit), `run_reflex_fallen.py` and `eval_recover.py` feed
both. Only a caller that passes no `q_meas` (the older run scripts, the
in-process MCP `SimBackend`) still gets the privileged tilt + torso-height
test, which the Pi cannot compute.
Neither policy replaces the analytic stack; both are gated by it.

## 2. What a run produces

`--run-dir runs/NAME` holds:

| file | content |
|---|---|
| `latest.pt` | the checkpoint: model, optimizer, normalisers, RNG state, `global_step`, `update`, the args, and **`env_config`** (below) |
| `train_log.jsonl` | one JSON line per update: `step`, `ep_return`, `ep_len`, policy/value/entropy losses, `kl`, `kl_stop`, `lr`, `sps` |
| `train.out` | stdout, if you redirected it (the launcher does) |

`env_config` is the env's own `config()`: obs version, dim and names,
action mapping, rate limit, EMA alpha, servo mode and ranges, DR ranges,
obs noise, reward version, `reward_rev` and weights, the handoff
definition, the gait parameters (gait env), and the **robot fingerprint**
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
  limit above 4.7 rad/s is flagged **`exceeds servo`**.
- A legacy checkpoint cannot be resumed under obs v2; the trainer says so.

Reading the log: `python sim/rl_dashboard.py` prints a table of every run
(env, reward, rate limit, obs version, servo, steps, last
return/length/entropy, flags, recorded eval) and draws the curves to
`sim/out/rl_curves.png`. The playground's `rl` command prints the same
table live. What to look for:
- `ep_return` rising is good.
- On the recover env, `ep_len` dropping below the 300-step ceiling means
  episodes are ending in success.
- `ent` (entropy) climbing without bound is the D045 failure (sigma
  inflating into bang-bang control); cap it with `--log-std-max -0.5`.

## 3. Step by step: train, evaluate, resume

Install torch first: `pip install -e ".[sim,rl]"`. CPU works for
everything below; a GPU only speeds up the long runs.

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
clipped action) and mean |Δresidual| per tick (chatter).

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
phase of the shove demo. Its original columns are pinned fraction,
reversals/s, |move| and track. Since D052 it also reports **ctrl rev/s**
(reversals of the actuator command *after* the servo filter) and **qvel
flip/s** (joint-velocity sign flips, 0.2 rad/s deadband). With the servo
in the loop, "pinned at the rate limit" no longer means jitter; these two
columns do.

**A real run** (the laptop recipes, also in `rocky.sh`):

```bash
./rocky.sh train-walk    walk1                      # residual gait, 8 envs, DR on
./rocky.sh train-recover recover6 --total-steps 3000000
./rocky.sh jobs                                     # what is training, last log line each
./rocky.sh eval-walk     walk1
./rocky.sh eval-recover  recover6
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
`recover1` is no longer possible).

**Ctrl-C** saves a checkpoint before exiting.

## 4. The results so far (honest table)

Checkpoints ship in `sim/runs/` (git-lfs). "Stood" = stood-after-handoff
over 20 random falls on the same seeds, deterministic policy. Three
columns, none replacing another:
- **cloud**: what the 2026-09-01 session measured.
- **laptop, pre-D052**: this machine (mujoco 3.12) on the D047 model
  (re-measured 2026-09-23: `recover1` 7/20 with both the old and the new
  evaluator; the 10/20 quoted before predates the D047 regeneration of
  `pebble.xml`).
- **D052 model** (2026-09-24: joint damping 0.626 N·m·s/rad, forcerange
  1.911 N·m, foot μ 0.8, fingerprint `5a32f772ca99`) — superseded the same
  day by the **D052 amendment** (forcerange = stall 2.94 N·m peak, the
  continuous 1.911 N·m enforced by the thermal proxy, fingerprint `ceb63a1254c3`).
  The amended model is the baseline for any comparison from now on.
  `recover1`'s default (`handoff_ok`), `--handoff legacy` and
  `--supervisor` evals were re-measured on it the same day, with an A/B on
  the same tree with the 1.911 N·m clip restored in memory (**bold** below
  = peak model, "clip" = that A/B); its servo / rand / jitter cells, every
  `recover5_v3_warm` cell, the gait rows and the 2 000-step smokes are
  still the `5a32…` numbers.

**Every checkpoint below is legacy** (obs v1, true-state observation,
ideal actuator, 5 rad/s; the gait ones on the old T 1.6 s / 32 mm gait).
The evaluators replay them under that contract and flag them
**`legacy obs, exceeds servo`**; none can be resumed or warm-started
under obs v2. The D052 column therefore measures an old policy on the new
robot, not what the new contract can learn — nothing has been trained on
it yet beyond two 2 000-step smokes (the recipe is in §5 and B34).

What the D052 column's terms mean: **legacy** = the old handoff test
(tilt < 25° and torso height > 0.09 m, `--handoff legacy`); **hw** =
`handoff_ok` (tilt + ≥ 3 foot switches + joint-angle height, the default);
**servo** = `--servo nominal`; **rand** = `--randomize` (servo random + DR
+ obs noise); **system** = `--supervisor` (supervisor + righter + stall
ramp, 14 s, on `handoff_ok`), with the no-righter row in brackets.

| run | env | steps | training return | stood (cloud / laptop pre-D052) | pure-RL (pre-D052) | **D052 model (laptop)** | note |
|---|---|---|---|---|---|---|---|
| `robust_fwd2` | gait | 3 M | 228.6 | — | — | legacy replay 245 mm vs zero 320; servo 236 vs 298; servo random + DR + shoves 216 vs 250 (0/3 falls each); **no envelope** on the D052 servo, so the cockpit zeroes it | legacy obs, T 1.6 gait. Pre-D052: loses to the bare gait on the clean task (298 vs 365 mm); D031: the wave gait is a strong controller. Still loses on D052 |
| `cmd_sample3` | gait, `--cmd-sample` | 7 M | 224 | — | — | not re-measured; no envelope (same gait) | same conclusion under full DR |
| **`recover1`** | recover v1, 5 rad/s | 2 M | — | **12/20 / 7/20** | 0/20 | stood **5/20** legacy (back 0/6, side 3/7, tumble 2/7; clip 3/20, D052 run 2/20 — noise at 20 episodes) · **0/20** hw (back 0/6, side 0/7, tumble 0/7; end tilt median 16.6°, best 3.5°; hold-pose 1/20, random 0/20; clip 0/20) · pure-RL **0/20** · system **20/20** (no righter 11/20; back 6/6 vs 0/6, side 7/7 vs 5/7, tumble 7/7 vs 6/7), 9 declared falls: 0 handoff, 9 stall, 0 deadline exits, t_stood median **0.46 s** (no righter 0.40; clip 0.80 / 0.62) · on `5a32…`: 0/20 servo · 0/20 rand · jitter (servo): 52 % pinned, 6.7 rev/s, 3.3°/tick, ctrl rev/s 7.5 | **the shipped righter**, legacy. Pre-D052 jitter: 75 % pinned, 10 reversals/s, 4.5°/tick. Its legacy "handoffs" were a robot kneeling on its shins with 0–2 feet down and knees at −110…−150° |
| `recover2` | recover v2, uncapped | 3.67 M | ~610 (stochastic) | 2/20 | 0/20 | not re-measured | sigma inflated to 22 nats; mean policy decayed (D045) |
| `recover3_capped` | recover1 → v2 capped | 4 M | 543 | — / 7/20 | 0/20 | not re-measured | every dimension pinned at the cap; success carried by noise |
| `recover3_scratch` | recover v2 capped, scratch | 3 M | — | — / 3/20 | 0/20 | not re-measured | never rights from the back |
| `recover5_v3` | recover v3 capped, 3 rad/s, scratch | 3 M | 215 | — / 2/20 | 2/20 | not re-measured | D048 negative: smoothness cost, lower rate limit; back 0/6, side 0/7 |
| `recover5_v3_warm` | recover1 → v3 capped, 3 rad/s | 5 M | 483 | — / 4/20 | 4/20 | stood **3/20** legacy · **0/20** hw · 0/20 servo · 0/20 rand · pure-RL **0/20** · system 20/20 (no righter 11/20), output line-for-line identical to recover1's · jitter (servo): 55 % pinned, 3.9 rev/s, 2.0°/tick, ctrl rev/s 4.7 | legacy. D048 negative on the handoff (4 < 7) but the smoothest righter so far (pre-D052: 58 % pinned, 7 reversals/s, 2.1°/tick); on D052 its pure-RL 4/20 is gone |

A "not re-measured" cell is not a zero; those runs were not re-evaluated
in D052.

What this says, pre-D052: the PPO infrastructure works and reproduces; the
learned policies are marginal; the self-righting result is a hybrid in
which the policy does the hard part from a SIDE landing (getting tilt
down) and the hand-written ramp does the standing. From the BACK no
checkpoint rights the robot and the planted-stance ramp does (5/5 in the
demo), which is why the supervisor ramps after 3 s without progress (D048
stall rule). The v3 runs showed the trade: the smoothness cost halves the
staircase but costs handoffs at this budget.

What D052 adds: the 7/20 was flattered twice, by an ideal actuator and by
a handoff test that could not tell standing from kneeling. On the honest
robot and the honest test **no checkpoint earns a handoff**. The system
still stands 20/20 against 11/20 without a righter, so the policies do
something useful — they leave the body in a pose the stall ramp can stand
(back landings 6/6 vs 0/6) — but that number cannot tell `recover1` from
`recover5_v3_warm` (identical output), so it is not a righter score. With
the servo in the loop the staircase mostly disappears, which is what the
filter-in-the-env rung (§5.9) was meant to buy. `recover1` stays the
installed righter because it is the one that has been through the demos,
not because it wins anything on D052. The peak-torque amendment changes
none of this: 0/20 on the hardware test either way, 5/20 vs 3/20 on the
legacy test is inside the noise, and the system count is the same; only
the stall ramp stands the righter's pose faster (median 0.46 s vs 0.80).

## 5. The experiment ladder

From `docs/RL_TOUR.md` §8, in order, each teaching one thing:

0. **(D052, first)** The overnight retrain on the new contract. Nothing on
   disk was trained on obs v2, and no legacy checkpoint can be
   warm-started. Recipe (B34):
   `./rocky.sh train-recover recover6 --total-steps 3000000 --device cpu`
   (v2 reward, `--log-std-max -0.5`, 8 envs, servo random, EMA 0.4,
   4.0 rad/s, DR + obs noise), then `eval-recover recover6` (hw handoff),
   `--randomize`, `--supervisor` (count the `handoff` exits, not just
   stood) and `audit_righter.py --servo nominal`. Same night if the GPU
   is free: `./rocky.sh train-walk walk1` (budgeted commands, rim shoves)
   and `eval-walk walk1 --compare-zero`; every walker on disk is zeroed by
   the D052 envelope.
1. Evaluate `robust_fwd2` with `--compare-zero`; read the curves with
   `rl_dashboard.py`. (It replays on its own T 1.6 s gait, flagged legacy.)
2. Change one reward weight in `rocky_env.py`, retrain the smoke config,
   watch what moves.
3. Turn DR off (`--no-randomize`) and compare the eval on the clean task —
   the D031 lesson.
4. Fair comparison: evaluate policy and zero-action under the same DR
   (`eval_ppo.py --randomize`).
5. Sigma anneal / `--log-std-max` on the gait env.
6. Recover env: evaluate `recover3_capped` with `--stochastic` seeds — if
   sampled actions score far above its deterministic number, the
   mean/mode gap is the whole D045 story. (Pre-D052 bar 7/20; on D052,
   compare against its own `--handoff legacy` and hw numbers.)
7. Rung 6 from the tour: anneal sigma to the floor over the last third of
   a capped-v2 run so the mean has to do the work.
8. A new env: rubble recovery, or the room world with a goal.
9. The righter's jitter (D048): `audit_righter.py` before and after any
   change. The first candidate — an action filter INSIDE the env — shipped
   in D052 (EMA 0.4, state in the observation). Remaining: a curriculum
   from side landings to back landings, or the v3 cost with a 10 M budget.
   **The bar on the D052 model**: any handoff at all — more than
   `recover1`'s 0/20 on `handoff_ok`, and at least one `handoff` exit
   under `--supervisor` (both legacy checkpoints: 0 of 9) — with ctrl
   rev/s no worse than `recover5_v3_warm`'s 4.7. (Pre-D052 the bar was
   7/20 on the old handoff; do not compare against it.)

## 6. In the playground and the cockpit

`./rocky.sh cockpit` has an RL panel: the checkpoint table, a righter
picker, stall/deadline sliders and the training curves, next to the shove
buttons, so a righter comparison is two clicks and a watch.

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
- Checkpoints in `sim/runs/` are git-lfs objects; a smoke run in a
  `runs/smoke_*` directory will show up in `git status` — delete it or
  add it to `.gitignore` before committing.
- Training-time success ≠ deployable policy when sigma is doing the work
  (D045): always evaluate the mean, and evaluate early.
