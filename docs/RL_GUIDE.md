# Reinforcement-learning guide

The practical companion to `docs/RL_TOUR.md` (which explains the theory
and the experiment ladder). This one is about running it: what the two
environments are, what a run produces, how to train, evaluate and resume,
and what the results actually say. Commands were run on 2026-09-22 from a
fresh checkout.

## 1. How it works

**One trainer, two environments.** `sim/train_ppo.py` is a self-contained
PPO implementation (vectorised envs, GAE, clipped surrogate, KL early stop,
observation and return normalisation, atomic checkpoints, exact resume).
It trains either:

| `--env` | class | what the policy controls | starting point |
|---|---|---|---|
| `gait` | `sim/rocky_env.py` `PebbleEnv` | a **residual**: 15 joint-target offsets (±0.25 rad) added to the analytic wave gait's targets | policy zero already walks; PPO learns trims |
| `recover` | `sim/rocky_recover_env.py` `RecoverEnv` | **absolute** joint targets, rate-limited (`--rate-limit`, 5 rad/s for the v1/v2 runs, 3 rad/s recommended for v3), from a fallen pose | nothing; there is no analytic righter |

Observations (41 or 39 values): gravity in the body frame, gyro, joint
positions and velocities, plus gait phase and command (gait) or torso
height, feet in contact and tilt (recover). Control at 50 Hz, ten physics
substeps per control step. Episodes: 8 s (gait), 6 s (recover).

**Rewards.** Gait: velocity tracking (xy and yaw rate) minus action cost,
tilt and height error. Recover `v1`: uprightness + height + a standing
bonus, success when standing is held 1 s. Recover `v2` (D041): success is
literally the handoff criterion the reflex supervisor uses (tilt < 25°,
height > 0.09 m, held 0.5 s) plus a feet-down term while upright.
Recover `v3` (D048): v2 plus a smoothness cost on the servo target,
`-0.3 · mean((Δtarget / rate_limit)²)`, so pinning joints at the rate
limit is paid for. The checkpoint records `reward` and `rate_limit`;
`eval_recover.py` and the `PolicyRighter` adapter read them back, so a
policy is always replayed with the dynamics it trained on.

**Why v3 exists: the jitter.** `sim/audit_righter.py` measures how
staircase-y a righter is. `recover1` (v1, 5 rad/s) has 73 % of its
per-tick joint moves pinned at the rate limit and reverses direction 4–23
times per second per joint: a 50 Hz staircase of 5.7° jumps, which is
what "the recovery looks jittery" is. That is the D045 bang-bang failure
in the deployed policy, not a sim artefact.

**Domain randomisation** (on by default, `--no-randomize` to disable):
per-episode draws of friction ×[0.7, 1.4], masses ×[0.85, 1.15], actuator
torque ×[0.85, 1.15], gravity tilt ≤ 3°, 0–1 steps of action latency,
random pushes (`--push-prob`). `--cmd-sample` re-draws the velocity
command every episode so one policy covers the whole envelope.

**The deployment idea.** The residual gait policy is a bounded trim on a
controller that already runs on hardware; the recovery policy hands over
to the analytic planted stance the moment the handoff criterion is met
(`gait/pebble_reflex.py` FALLEN → RIGHTED, D042). Neither replaces the
analytic stack; both are gated by it.

## 2. What a run produces

`--run-dir runs/NAME` holds:

| file | content |
|---|---|
| `latest.pt` | the checkpoint: model, optimizer, normalisers, RNG state, `global_step`, `update`, the args |
| `train_log.jsonl` | one JSON line per update: `step`, `ep_return`, `ep_len`, policy/value/entropy losses, `kl`, `kl_stop`, `lr`, `sps` |
| `train.out` | stdout, if you redirected it (the launcher does) |

Reading the log: `python sim/rl_dashboard.py` prints a table of every
run (env, reward, rate limit, steps, last return/length/entropy, recorded
eval) and draws the curves to `sim/out/rl_curves.png`; the playground's
`rl` command prints the same table live. `ep_return` rising is good; `ep_len` on the recover env
dropping below the 300-step ceiling means episodes are ending in success;
`ent` (entropy) climbing without bound is the D045 failure (sigma inflating
into bang-bang control) — cap it with `--log-std-max -0.5`.

## 3. Step by step: train, evaluate, resume

Install torch first: `pip install -e ".[sim,rl]"`. CPU works for
everything below; a GPU only speeds up the long runs.

**Smoke test (about a minute each)** — proves the pipeline end to end
(`MUJOCO_GL` is only needed when something renders; the trainer does not):

```bash
cd sim
MUJOCO_GL=egl python train_ppo.py --env gait    --num-envs 2 --rollout 96 --total-steps 2000 --run-dir runs/smoke_gait
MUJOCO_GL=egl python train_ppo.py --env recover --reward v2 --log-std-max -0.5 \
                                  --num-envs 2 --rollout 96 --total-steps 2000 --run-dir runs/smoke_recover
```

**Evaluate** (deterministic: the actor mean):

```bash
MUJOCO_GL=egl python eval_ppo.py runs/smoke_gait/latest.pt --episodes 5 --compare-zero   # vs the bare gait
MUJOCO_GL=egl python eval_recover.py runs/smoke_recover/latest.pt --episodes 20           # vs hold-pose / random
MUJOCO_GL=egl python eval_ppo.py runs/robust_fwd2/latest.pt --video walk.mp4              # write a clip
```

`--compare-zero` runs the same seeds with a zero action, i.e. the analytic
gait alone; a residual policy has to beat that column to be worth
deploying. `eval_recover.py` reports stood-after-handoff (the number that
matters: the policy rights the body, the analytic ramp finishes), the
pure-RL success rate, the end-tilt distribution, a per-fall-mode
breakdown and the hold-pose and random baselines on the same seeds.

**A real run** (the laptop recipes, also in `rocky.sh`):

```bash
./rocky.sh train-walk    walk1                      # residual gait, 8 envs, DR on
./rocky.sh train-recover recover4 --total-steps 3000000   # D045 capped-v2 recipe
./rocky.sh jobs                                     # what is training, last log line each
./rocky.sh eval-walk     walk1
./rocky.sh eval-recover  recover4
```

Defaults are overnight-sized (`--total-steps`, `--num-envs 8`,
`--rollout`); pass `--sync` to debug with a single process; `--device cpu`
to keep the GPU free.

**Resume.** `--resume auto` picks up `<run-dir>/latest.pt` (or pass a
path). The resume is exact: optimizer, normalisers and RNG come back, and
`--total-steps` is the *cumulative* target, so to add 2 M steps to a 2 M
checkpoint pass `--total-steps 4000000`. Keep `--rollout` and
`--num-envs` as in the checkpoint; if the target allows no new updates
the trainer says so and exits instead of crashing (fixed 2026-09-22).

**Ctrl-C** saves a checkpoint before exiting.

## 4. The results so far (honest table)

Checkpoints ship in `sim/runs/` (git-lfs). "Stood" = stood-after-handoff
over 20 random falls on the same seeds, deterministic policy; the cloud
column is the number the 2026-09-01 session measured, the laptop column is
this machine (mujoco 3.12) on the **current** model and is the baseline
for any comparison here (re-measured 2026-09-23: `recover1` scores 7/20
with both the old and the new evaluator; the 10/20 quoted before predates
the D047 regeneration of `pebble.xml`).

| run | env | steps | training return | stood (cloud / laptop) | pure-RL | note |
|---|---|---|---|---|---|---|
| `robust_fwd2` | gait | 3 M | 228.6 | — | — | loses to the bare gait on the clean task (298 vs 365 mm); D031: the wave gait is a strong controller |
| `cmd_sample3` | gait, `--cmd-sample` | 7 M | 224 | — | — | same conclusion under full DR |
| **`recover1`** | recover v1, 5 rad/s | 2 M | — | **12/20 / 7/20** | 0/20 | **the shipped righter** (hold-pose 2/20, random 2/20); back 0/6, side 3/7, tumble 4/7; jitter audit: 75 % of tick moves pinned, 10 reversals/s, 4.5°/tick |
| `recover2` | recover v2, uncapped | 3.67 M | ~610 (stochastic) | 2/20 | 0/20 | sigma inflated to 22 nats; mean policy decayed (D045) |
| `recover3_capped` | recover1 → v2 capped | 4 M | 543 | — / 7/20 | 0/20 | every dimension pinned at the cap; success carried by noise |
| `recover3_scratch` | recover v2 capped, scratch | 3 M | — | — / 3/20 | 0/20 | never rights from the back |
| `recover5_v3` | recover v3 capped, 3 rad/s, scratch | 3 M | 215 | — / 2/20 | 2/20 | D048 negative: smoothness cost, lower rate limit; back 0/6, side 0/7 |
| `recover5_v3_warm` | recover1 → v3 capped, 3 rad/s | 5 M | 483 | — / 4/20 | 4/20 | D048 negative on the handoff (4 < 7) but the smoothest righter so far: 58 % pinned, 7 reversals/s, 2.1°/tick (recover1: 75 %, 10/s, 4.5°); the only checkpoint whose pure-RL success is non-zero |

What this says: the PPO infrastructure works and reproduces; the learned
policies are marginal; the self-righting result is a hybrid in which the
policy does the hard part from a SIDE landing (getting tilt down) and the
hand-written ramp does the standing. From the BACK no checkpoint rights
the robot, and the planted-stance ramp does (5/5 in the demo), which is
why the supervisor now ramps after 3 s without progress instead of
letting the policy flail to the 10 s deadline (D048 stall rule).
`recover1` beats the baselines and is what `run_reflex_fallen.py`, the
playground and the harness use. The v3 runs show the trade: the
smoothness cost halves the staircase but costs handoffs at this budget;
a smooth AND better righter needs either more steps or a different
action parameterisation (see the ladder).

## 5. The experiment ladder

From `docs/RL_TOUR.md` §8, in order, each teaching one thing:

1. Evaluate `robust_fwd2` with `--compare-zero`; read the curves with
   `rl_dashboard.py`.
2. Change one reward weight in `rocky_env.py`, retrain the smoke config,
   watch what moves.
3. Turn DR off (`--no-randomize`) and compare the eval on the clean task —
   the D031 lesson.
4. Fair comparison: evaluate policy and zero-action under the same DR
   (`eval_ppo.py --randomize`).
5. Sigma anneal / `--log-std-max` on the gait env.
6. Recover env: evaluate `recover3_capped` with `--stochastic` seeds — if
   sampled actions score far above 7/20, the mean/mode gap is the whole
   D045 story.
7. Rung 6 from the tour: anneal sigma to the floor over the last third of
   a capped-v2 run so the mean has to do the work.
8. A new env: rubble recovery, or the room world with a goal.
9. The righter's jitter (D048): `audit_righter.py` before and after any
   change. Candidates: a first-order action filter INSIDE the env (so the
   policy trains on it), a curriculum from side landings to back landings,
   or the v3 cost with a 10 M budget. The bar is recover1's 7/20 with the
   warm run's smoothness numbers.

## 6. In the playground

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
