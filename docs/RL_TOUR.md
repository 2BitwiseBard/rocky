# RL from the ground up — a guided tour of Pebble's learning stack

You asked for both "run the kit" and "understand it from zero." This is
the second one: what each piece is, why it's shaped that way, and a
ladder of experiments ordered so each teaches one thing. Wesleyan-math
footnotes included — the underlying objects are ones you already know.

## 1 · The problem, stated plainly

An MDP: state s (what the physics is doing), action a (what we tell the
servos), reward r(s, a) (how much we liked it), and a policy π(a|s) we
tune to maximize expected discounted return E[Σ γᵗ rₜ]. Everything else
is engineering around three difficulties: we can't enumerate states
(continuous, ~40-D), we can't differentiate through the physics, and
naive gradient estimates have monstrous variance.

**The math under it:** the policy gradient theorem says
∇J = E[∇ log π(a|s) · Aπ(s, a)] — you can estimate the gradient of the
return from samples, weighting the log-likelihood gradient of each
action by its *advantage* A (how much better it turned out than the
policy's average from that state). Every acronym below is a variance-
reduction or step-size-control trick bolted onto that one identity.

## 2 · The cast, file by file

**`sim/rocky_env.py` — PebbleEnv (the gait task).** The single most
important design choice in the repo: it's a **residual** policy. The
analytic wave gait computes joint targets anyway; the network's action
(15 values in [−1,1]) is scaled to ±0.25 rad and ADDED to them. Policy
zero = the proven gait, so learning starts from walking, not flailing,
and on hardware the network output is a bounded trim on a controller
that already works — the safe on-ramp for sim-to-real.
Obs (41): gravity direction in body frame + gyro (the IMU), joint
pos/vel (the encoders), gait phase as sin/cos (so the number is
periodic-continuous), and the command — including the command is what
makes ONE network a whole family of controllers (`--cmd-sample`).
Reward: Gaussian-shaped velocity+yaw tracking, minus action, tilt and
height penalties. Termination at 45° tilt (with a −5 exit fee so
falling isn't a way to escape a bad episode).

**`sim/rocky_recover_env.py` — RecoverEnv (self-righting).** The
opposite regime: no controller exists, so the policy commands ABSOLUTE
joint targets (rate-limited to 5 rad/s — real servos, not teleports)
from random fallen poses. Reward is dense shaping (uprightness +
height) plus a standing bonus. Read its "Honesty" docstring: the sim's
torso is not the real shell, so the *strategy* transfers, the timing
won't.

**`sim/train_ppo.py` — the learner.** PPO, ~300 lines, no framework.
The pieces and the one-line reason each exists:
* *Vectorized envs* (`--num-envs`): N sims in parallel processes —
  decorrelates the batch and uses your cores.
* *GAE* (γ=0.99, λ=0.95): the advantage estimator — a geometric blend
  of n-step returns trading bias against variance (λ→1 unbiased/noisy,
  λ→0 biased/smooth).
* *The clipped surrogate* (`--clip 0.2`): PPO's whole trick — optimize
  the importance-weighted objective but clip the ratio π_new/π_old, so
  a batch can't drag the policy further than trust-region distance.
  `--target-kl` is the belt to that suspender: stop the epoch early if
  the measured KL divergence jumps.
* *Obs/return normalization* (`RunningMeanStd` — Welford's algorithm):
  networks want unit-scale inputs; rewards get normalized by the std of
  the RETURN, not the reward, so shaping scale doesn't set the lr.
* *Entropy bonus* (`--ent-coef`): keeps π from collapsing early.
* *`--log-std-init`*: the exploration knob, and the site of the
  hard-won D031 lesson — on a residual policy, exploration noise starts
  QUIET (−1.0) because loud noise just degrades the good controller and
  the gradient learns "mute yourself" instead of "improve."
* Atomic checkpoints + jsonl log + exact RNG resume: boring, and the
  reason a run can survive your laptop lid.

**Domain randomization** (in the envs, per-episode draws — D031's other
lesson: draw from stored bases, never multiply the live model, or your
physics goes on a random walk): friction, masses, torque, gravity tilt,
action latency, random shoves. DR is why sim numbers are honest ranges
rather than one lucky rollout — and why returns under DR can't be
compared with returns without it.

**`sim/eval_ppo.py` / `sim/eval_recover.py`** — deterministic (mean-
action) evaluation, videos, and the two honesty baselines you should
never skip: `--compare-zero` (the bare gait) and, for recovery,
hold-pose/random. A learned policy that can't beat its own zero-action
baseline is a negative result, and we log those (BUILD_LOG 8b: the
residual currently LOSES to the bare gait on the clean task — that's
written down, not hidden).

## 3 · The experiment ladder (each rung teaches one thing)

Do them in order; each is one evening or less on your machine.

1. **Watch before you train.** `eval_ppo.py runs/cmd_sample3/latest.pt
   --video a.mp4 --compare-zero`, then the same for `runs/recover1` with
   `eval_recover.py`. Look at the mean_abs_residual number: how loud is
   the correction the network learned?
2. **Plot the shipped runs.** Two lines of pandas on
   `runs/*/train_log.jsonl` (step vs ep_return, kl, ent). Learn what a
   healthy curve looks like before you cause an unhealthy one.
3. **One reward knob.** In `rocky_env.py`, double the tilt penalty
   (−1.2 → −2.4). Train 1 M steps fresh AND 1 M warm-started. Compare
   tilt in eval. You'll see the whole loop: shaping → behavior.
4. **The exploration lesson, yourself.** Same run twice: `--log-std-init
   -0.5` vs `-1.0`. Watch the first 300k steps. This reproduces D031 —
   cheaper to learn from the curve than from a week of confusion.
5. **Make the comparison fair.** The bare-gait bar (345) is measured
   UNDER DR; eval compares live. Run eval with `--randomize` on and off
   for both policy and zero. Does the residual earn its keep under DR
   specifically? (This is the open question the BUILD_LOG left you.)
6. **Sigma-anneal experiment** (a real open item): add a linear decay of
   log_std over training (a few lines in train_ppo). Does run-3's
   plateau at ~224 move?
7. **Fix recovery's last gap — ATTEMPTED 8d, and the negative result is
   your best case study (D045).** The v2 reward (success = the handoff
   criterion + a feet term, `--reward v2`) produced training-time
   successes v1 never had — and the deterministic policy REGRESSED to
   2/20 (recover1: 12/20 cloud, 10/20 laptop) because sigma inflated into bang-bang control
   (clip + rate limit make wide actions a strategy; entropy grew even
   with ent_coef 0). The counter is shipped: `--log-std-max -0.5`. The
   open experiment is now the CAPPED v2 retrain — beating 12/20 is still
   a real contribution to the robot, and the diagnosis is half your work
   done. Read the 8d BUILD_LOG entry before starting.
8. **A new environment.** Rubble-recovery, push-recovery-to-stand, or a
   "walk on the 20 mm rubble field" residual (import the heightfield
   from `run_terrain.py`). You now know every piece you'd copy.

## 4 · Reading list, matched to what you just touched

Sutton & Barto (free online) ch. 13 for the policy-gradient theorem;
Schulman's PPO paper (arXiv 1707.06347) and GAE paper (1506.02438) —
both short and readable after step 4; and "The 37 Implementation
Details of PPO" (ICLR blog) which is effectively an annotated version
of our train_ppo.py. For sim-to-real: OpenAI's DR paper (1703.06907) —
our DR block is that idea, small.

## 5 · House rules that keep results honest (they're decisions)

D017/D039: quote push results in bodyweights; compare same-DR to
same-DR. D025: phase-align comparisons; re-measure before
re-engineering. D031: DR draws from bases; quiet exploration on
residuals. D035: sanity-check that the motion you think happened
actually happened (eval videos + metrics, not returns alone). Negative
results go in the BUILD_LOG — they're the most valuable lines.
