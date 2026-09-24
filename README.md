# PROJECT ROCKY — Pebble

A 1:3-scale, radially symmetric **pentapod walking robot** (five legs, three
servos each, transforming hands) built from parametric 3D-printed parts and
Feetech bus servos, with a MuJoCo simulation that walks, gestures and gets
itself back up, and a tool harness that lets a person or a language model
drive it by text, voice or MCP.

<p align="center">
  <img src="media/pebble_fallen_recover.gif" width="480" alt="shove → tumble → self-right → walk away (MuJoCo)"><br>
  <em>A 40 N shove to the shell rim (1.5× bodyweight), a tumble onto its back, the righter and the planted ramp get it up, and the analytic gait walks away.</em>
</p>
<p align="center">
  <img src="gait/pebble_gait_demo.gif" width="480" alt="the wave gait"><br>
  <em>The five-phase wave gait, closed-form IK, no learning involved.</em>
</p>

The physical robot is not assembled yet. Everything "green" below is green
in simulation or in CAD checks, which this project treats as a different
claim from "it works on hardware".

## Status (2026-09-22)

| area | state | evidence |
|---|---|---|
| **CAD** | print-clean, rebuilt around a **measured STEP of the real ST3215 servo** (D047); four joint coupons gate the next print run | `cad/run_all_checks.py` 23/23 (joint suite, printability, single-solid gate) |
| **Physical build** | nothing assembled; the first prints (pre-D047) did not fit and are retired; servos not yet ordered | `media/2026-09-22_first_prints.jpg`, `docs/PRINT_PLAN_2026-09-22.md`, `bom/SHOPPING_LIST_2026-09-22.md` |
| **Sim + control** | the analytic wave gait walks (264 mm in 8 s, 1° tilt); the reflex supervisor safe-stops, braces and hands off to a righter; self-righting is a **hybrid** (RL rights the body from a side landing, an analytic ramp stands it, and from the back the ramp does the righting: 7/20 stood on this machine, 12/20 in the cloud run, pure-RL 0/20); four retrains scored worse and are recorded as negatives, the latest pair (D048) traded handoffs for a smoother policy | `docs/SIM_GUIDE.md`, `docs/RL_GUIDE.md`, `docs/RL_TOUR.md` |
| **Tests** | driver 58, harness 22, gait 9, intent 14, sim 3 → 92 fast tests in ~3 s; URDF ≡ MJCF ≡ analytic FK | GitHub Actions `ci.yml` |
| **Harness** | six-tool MCP server on a mock and on MuJoCo; a local LLM (llama-swap / Ollama) or Claude drives it; no hardware backend yet | `harness/`, `docs/MCP_CONTRACT_v0.md` |
| **Review** | full project review with dispositions | `docs/REVIEW_2026-09-22.md` |

## Layout

```
rocky/
├── README.md, BUILD_LOG.md, NOTES_INBOX.md   this file · the notebook (newest first) · the raw inbox
├── rocky.sh             launcher: play / chat / brain / voice / test / train-* / eval-* / cad-check
├── pyproject.toml       pip-installable core (gait modules + bus driver) and the extras
├── cad/                 build123d parametric parts; params.yaml is the single source of truth;
│   ├── run_all_checks.py   the CAD CI: 23 modules incl. check_assembly (joints) + check_printability
│   ├── ref/             the ST3215 STEP model every leg part is derived from
│   ├── out/             exported STL/STEP/PNG (git-lfs) + pebble_viewer.html (self-contained WebGL)
│   └── leg_frame.py, servo_st3215.py, servo_mount.py, part_*.py
├── gait/                pure-numpy control: wave gait + IK, reflex supervisor, watchdog, gestures
├── sim/                 MuJoCo: build_mjcf.py (from params + CAD masses), playground.py,
│                        rocky_env.py + rocky_recover_env.py (gymnasium), train_ppo.py, eval_*.py,
│                        run_*.py experiments, runs/ (checkpoints, git-lfs)
├── harness/             MCP tool server (say/gesture/goto/stop/scan_summary/status), sim + mock
│                        backends, local_brain.py (LLM), intent.py (regex brain + voice pipe)
├── driver/              rocky_driver: dual-protocol Feetech bus driver, byte-faithful mock, 58 tests
├── bench/               bench-day scripts (all runnable --mock) + BENCH_RUNBOOK.md
├── perception/          legged-odometry EKF, ICP scan matching, cliff detector
├── ros2/                ROS 2 packages (scaffold; URDF generated from params, parity-checked)
├── audio/               chord-speak voice + samples
├── bom/, plan/, docs/   order sheets · master plan · decisions, interfaces, guides, review
└── media/               renders, photos, GIFs
```

## Quickstart

```bash
git clone https://github.com/2BitwiseBard/rocky && cd rocky
python3 -m venv .venv && . .venv/bin/activate        # or: uv venv && . .venv/bin/activate
pip install -e ".[sim,harness,dev]"                  # + ".[rl]" for torch, ".[cad]" for build123d
python -m pytest driver/tests gait harness -m "not slow" -q     # 89 passed (+3 in sim/tests after build_mjcf)
MUJOCO_GL=egl python sim/run_sim.py                  # the wave gait walks, headless
MUJOCO_GL=glfw python sim/playground.py --viewer     # live window + REPL + arrow-key teleop
```

`./rocky.sh help` lists the laptop shortcuts (they assume `.venv` in the
repo root and, for `brain`/`voice`, a local llama-swap and whisper-server).

## Driving it

- **Simulation** — `docs/SIM_GUIDE.md`: how the sim is built from the CAD,
  the playground (controls, commands, live tuning), the experiment scripts,
  worlds, videos, and the MCP/LLM driving loop.
- **Reinforcement learning** — `docs/RL_GUIDE.md`: the two environments,
  what a run produces, training/evaluation step by step, resuming, reading
  the logs, the honest results table, and the experiment ladder
  (`docs/RL_TOUR.md` is the from-first-principles companion).
- **Hardware** — `bench/BENCH_RUNBOOK.md` (bench day), `docs/WIRING_HARNESS.md`,
  `docs/PRINT_PLAN_2026-09-22.md`, `bom/SHOPPING_LIST_2026-09-22.md`.

## Ground rules

These are the ones that have actually cost something when broken.

- **`cad/params.yaml` is the single source of truth** for dimensions. Parts,
  the MJCF and the URDF are regenerated from it, never edited downstream.
- **Checks green before anything ships.** `cad/run_all_checks.py` 23/23 and
  the fast test suites; CI runs them on every push.
- **Sim honesty.** A negative result is a result and gets written down.
  Four separate recovery retrains scored worse than the policy they were
  meant to beat, and that is recorded rather than buried. Shoves are
  quoted in N·s and bodyweights, and the demo shove is a shove, not a
  strike (D048).
- **The SCS0009 hand servo never sees 12 V.**

## Where the record lives

- `BUILD_LOG.md` — engineering notebook, newest entry first.
- `docs/decisions.md` — numbered decisions (D001–D048), cited everywhere.
- `docs/DESIGN_BACKLOG.md` — ideas with verdicts (shipped / negative / deferred).
- `NOTES_INBOX.md` — raw measurements and results, filed later.

## License

MIT (see `LICENSE`). The ST3215 reference STEP in `cad/ref/` is from
TheRobotStudio/SO-ARM100, Apache-2.0.
