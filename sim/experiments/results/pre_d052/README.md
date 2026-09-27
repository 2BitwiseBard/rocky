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
  what the robot survives, so every direction reports the ceiling. The D017
  envelope (35–53 N walking / 46–53 N stance) is `push_results_ext.json`,
  from a wider ladder that no script in the repo writes;
  `terrain_results_ext.json` (25–40 mm rubble) is the same kind of
  extension of `run_terrain.py`. `plot_results.py` here draws
  `fig_push_envelope.png` and `fig_terrain.png` from them.
- `run_push_reflex_v2.py`'s trip threshold (1.8 rad/s) was calibrated on
  the pre-D052 spawn.
- `fairing_results.json` (D023, shin fairing rejected) compared the faired
  robot on different rubble fields than its bare-shin baseline
  (`stuck_results.json`): the fairing script's copy of the field builder
  skipped one random draw per box. Fixed in the script; the rerun decides.
- `fig_lidar_map.png` is `sim/sim_lidar.py`'s occupancy demo; the lap's
  scans are the fixture `sim/lidar_scans.npz`, still in `sim/`.
