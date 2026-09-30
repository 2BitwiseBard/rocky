# Pre-D052 experiment record (history)

These result JSONs and figures were produced by the experiment scripts on
2026-09-22 (commit 8dcb270; `reflex_fallen.json` on 2026-09-23, commit
8d635a3, D048) — before D052 fixed the spawn pose and added the servo model
and the peak-torque limit. They are the record behind decisions D017–D048,
not current numbers: re-run the script (`sim/experiments/README.md`) for
today's, which go to `sim/experiments/results/`. The decisions keep their
dated numbers.

Known caveats of this record:

- `push_results.json`: `run_push.py`'s force ladder stops at 27 N, below
  what the robot survives, so every direction reports the ceiling.
  `push_results_ext.json` is a wider ladder (27–60 N, 0.15 s shoves, on the
  session-8 mass budget) that no script in the repo writes: it holds stand
  40–46 N and walk 31–46 N. D017's 35–53 N walking / 46–53 N stance is an
  earlier run on the pre-D039 masses, kept only in the decision and the
  archived build log. `terrain_results_ext.json` (25–40 mm rubble) is the
  same kind of extension of `run_terrain.py`. `plot_results.py` here draws
  `fig_push_envelope.png` and `fig_terrain.png` from them.
- `run_push_reflex_v2.py`'s trip threshold (1.8 rad/s) was calibrated on
  the pre-D052 spawn.
- `fairing_results.json` (D023, shin fairing rejected) compared the faired
  robot on different rubble fields than its bare-shin baseline
  (`stuck_results.json`): the fairing script's copy of the field builder
  skipped one random draw per box. Fixed in the script; the rerun decides.
- `fig_lidar_map.png` is `sim/sim_lidar.py`'s occupancy demo; the lap's
  scans were the fixture `sim/lidar_scans.npz` (the 2026-07-31 lap at a raw
  45 mm/s), which B111 re-recorded inside the envelope on 2026-09-30: that
  lap is in git history only, and `slam_results.json` here was measured on it.
