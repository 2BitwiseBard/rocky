# sim/experiments — one-off experiments and their record

Each script here asks the simulator one question, prints a verdict and writes
its result. They are the evidence behind the decisions in
[docs/decisions.md](../../docs/decisions.md); they are not part of the robot
software (that lives in `sim/`, `gait/`, `perception/` and `harness/`).

Run any of them from the repo root:

```bash
MUJOCO_GL=egl .venv/bin/python sim/experiments/run_cliff.py
```

- Result JSONs and figures go to `sim/experiments/results/`, videos and
  temporary mux files to `sim/experiments/out/` (git-ignored).
  `ROCKY_EXPERIMENTS_OUT=DIR` sends both to `DIR` instead, for a scratch run
  that leaves the record alone.
- `exp_paths.py` holds those paths and puts `sim/`, `gait/`, `perception/`
  and `audio/` on `sys.path`; every script imports it first.
- The helpers live code shares with these scripts (the cliff world, the
  rubble field, the push harness's spawn, the IMU noise model, the 1.5 N
  contact switch, the fallen demo's shove) are in `sim/scenes.py`.
- `results/pre_d052/` is the record as it was committed on 2026-09-22,
  before D052 changed the robot model. See its README.

Experiments that stayed in `sim/` because live code, CI or the docs use
them as tools: `run_sim.py` (CI smoke), `shove_envelope.py`,
`torque_audit.py`, `mass_audit.py`, `audit_*.py`, `eval_*.py`,
`train_ppo.py`, `sim_lidar.py` (it writes the `sim/lidar_scans.npz` fixture
and the `sim/laserscan_spec.json` contract).

| script | what it measures | decision | superseded by | how to run (from the repo root) | current result |
|---|---|---|---|---|---|
| `run_terrain.py` | how rough rubble (0–20 mm boxes, 3 seeds) the blind wave gait crosses | D017 | — | `MUJOCO_GL=egl .venv/bin/python sim/experiments/run_terrain.py [--video AMP]` (~2 min) | rerun pending |
| `run_push.py` | push envelope of the bare gait, 0.15 s CoM shove, 12 directions | D017 | `sim/shove_envelope.py` (D048). Frozen: its 27 N ladder saturates | `... run_push.py` (~2 min) | rerun pending |
| `run_push_reflex.py` | the v1 crouch-and-brace reflex vs the bare gait; calibrates the 1.8 rad/s trip | D022 | `run_push_reflex_v2.py` (D025) | `... run_push_reflex.py [--quick]` (minutes) | rerun pending |
| `diag_brace_phase.py` | one failing +0.25T push traced leg by leg: why the v1 brace hurt | D025 | — (diagnosis) | `... diag_brace_phase.py` (<1 min) | rerun pending |
| `run_push_reflex_v2.py` | gait-phase-aware brace (v2) vs v1 vs base: envelope, phase probe, sustained lean | D025 | shove model: `sim/shove_envelope.py` (D048) | `... run_push_reflex_v2.py --envelope \| --probe \| --sustained \| --merge` (no flag: all, well over 2 min) | rerun pending |
| `run_stuck.py` | 30–45 mm rubble crossings with and without the stuck watchdog | D023 | — | `... run_stuck.py [--video AMP SEED]` (minutes) | rerun pending |
| `run_fairing.py` | a ramped TPU shin fairing on the same rubble (re-run `run_stuck.py` first: it is the bare-shin baseline) | D023 (rejected), B1 | — | `... run_fairing.py` (minutes) | rerun pending |
| `run_cliff.py` | the VOID detector at a table edge vs walking off it | D034 | — | `... run_cliff.py [--video]` (<1 min) | rerun pending |
| `run_cliff_safestop.py` | the VOID stop through PLANT→BRACE→idle vs a bare v=0 halt | D034 | — | `... run_cliff_safestop.py [--video]` (~1 min) | rerun pending |
| `run_odom.py` | legged-odometry EKF drift vs ground truth: flat, turns, rubble | D026 (baseline) | `run_odom_v2.py` | `... run_odom.py` (~1 min) | rerun pending |
| `run_odom_v2.py` | the zero-yaw-rate standing update (StillnessGate) and the mag stub | D026 | — | `... run_odom_v2.py` (1–2 min) | rerun pending |
| `run_slam_lite.py` | EKF-seeded point-to-line ICP over the recorded lidar lap: ATE, map IoU | D027 | — | `.venv/bin/python sim/experiments/run_slam_lite.py` (reads `sim/lidar_scans.npz`; seconds) | rerun pending |
| `run_manip_adjacent.py` | adjacent-arm manipulation by foot repositioning vs the no-repositioning control | D018 | — | `... run_manip_adjacent.py [--control] [--video]` (<1 min) | rerun pending |
| `run_gestures2.py` | every gesture-library-v2 entry in physics: tilt, sag, planted-foot breaks, feasibility | D040 | — | `... run_gestures2.py [--metrics] [NAME ...]` (1–2 min) | rerun pending |
| `run_reflex_fallen.py` | 40 N rim shove → fall → the recovery policy rights it → walks away, 5 seeds | D042, D045, D048 | — | `... run_reflex_fallen.py [--episodes N] [--video PATH]` (2–3 min) | rerun pending |
| `run_patrol.py` | a waypoint patrol driven only through the harness tools (goto, scan_summary, say) in the room world | D043 | — | `... run_patrol.py [--waypoints N]` (1–2 min) | rerun pending |
| `run_sim_manip.py` | demo: walk, strafe, turn, 4-leg walk with an arm raised, 3-leg two-arm work | — (demo clip) | — | `... run_sim_manip.py` → `out/pebble_rocky_demo.mp4` | — |
| `run_gestures.py` | demo: jazz hands + a contact-triggered fist bump, narrated | — (demo clip) | `run_gestures2.py` for metrics | `... run_gestures.py` → `out/pebble_gestures.mp4` | — |
| `run_beckon.py` | demo: the beckon gesture, narrated | D035 | — | `... run_beckon.py` → `out/pebble_beckon.mp4` | — |
| `run_stuck_voiced.py` | demo: the 40 mm stuck-retry crossing, narrated by the watchdog's own events | — (demo clip) | — | `... run_stuck_voiced.py [SEED]` → `out/pebble_stuck_retry_40mm_voiced.mp4` | — |
| `run_showcase.py` | demo: locomotion medley, push + brace, cliff stop in one narrated reel | — (demo clip) | — | `... run_showcase.py` → `out/pebble_showcase_v07.mp4` | — |

`...` = `MUJOCO_GL=egl .venv/bin/python sim/experiments/`. The narrated clips
need `ffmpeg` on the PATH.
