# PROJECT ROCKY — Pebble

A 1:3-scale, radially symmetric **pentapod walking robot** (five legs, three
servos each, transforming hands) built from parametric 3D-printed parts and
Feetech bus servos, with a MuJoCo simulation that walks, gestures and gets
itself back up, and a tool harness that lets a person or a language model
drive it by text, voice or MCP.

<p align="center">
  <img src="media/pebble_fallen_recover.gif" width="480" alt="shove → tumble → self-right → walk away (MuJoCo)"><br>
  <em>The D048 shove: 40 N at the shell rim (10 N·s, 1.5× bodyweight), a tumble, the <code>recover1</code> righter and the planted ramp stand it up, and the analytic gait walks away.</em>
</p>
<p align="center">
  <img src="gait/pebble_gait_demo.gif" width="480" alt="the wave gait"><br>
  <em>The five-phase wave gait, closed-form IK, no learning: 2.0 s cycle, 24 mm step, walking at 45 mm/s and turning at 0.246 rad/s, both inside the servo's budget.</em>
</p>

The physical robot is not assembled yet. Everything "green" below is green
in simulation or in CAD checks, which this project treats as a different
claim from "it works on hardware".

## Status (2026-09-26)

| area | state | evidence |
|---|---|---|
| **CAD** | print-clean, rebuilt around a **measured STEP of the real ST3215** (D047); the leg harness has a channel through the coxa and the star-board bracket sits on real deck holes (D059); four joint coupons gate the next print run | `cad/run_all_checks.py` **28/28**, [docs/PRINT_PLAN.md](docs/PRINT_PLAN.md) |
| **Physical build** | nothing assembled, nothing ordered; the first prints (pre-D047) did not fit and are retired. The bench kit (one leg: 4 × ST3215, bus adapter, adjustable supply, fasteners) is ~$280 at 2026-09-26 prices, the core robot ~$1,258 | [bom/BOM.csv](bom/BOM.csv), [bom/README.md](bom/README.md), `media/2026-09-22_first_prints.jpg` |
| **Sim + control** | the sim stops flattering the servo (D052/D052a): a peak-torque clip plus a thermal budget, every motion through one feasibility checker and the gait envelope (45.5 mm/s, 0.246 rad/s). Wave gait: 246 of ~261 mm in 6 s, 0.84° max tilt. An always-on void guard stops walks at an edge, except a lip band (7 of 310 approaches) | [docs/SIM_GUIDE.md](docs/SIM_GUIDE.md) |
| **RL / self-righting** | a **hybrid**: RL rights the body, an analytic ramp stands it. No policy earns a hardware handoff on the D052 contract (`recover1` 0/20 `handoff_ok`; the obs-v2 retrain `recover6_d052` negative, B34); the system (supervisor + righter + 3 s stall ramp) stands 20/20 vs 11/20 with no righter. The walkers predate D052 and are zeroed | [docs/RL_GUIDE.md](docs/RL_GUIDE.md) |
| **Tests** | 1,039 fast tests pass + 2 strict xfail (the void guard's lip band) in ~8 min; URDF ≡ MJCF ≡ analytic FK; CI also runs the slow tests and `run_sim`, and fails if the regenerated MJCF / URDF or `docs/TOOLS.md` differ from the committed ones | `.github/workflows/ci.yml`, `./rocky.sh test` |
| **Harness** | one tool registry (`harness/capabilities.py`, 23 tools, D056) generates every surface: the MCP server's live list (8 tools on the mock or in-process sim, 15 over a running cockpit, 19 with place recognition), the local brains' schema and the tools page; any OpenAI-compatible model server or Claude drives it | `harness/`, [docs/TOOLS.md](docs/TOOLS.md) |
| **Cockpit** | one running sim in the browser (D049–D057): cameras, map, switchable brains behind a stop gate (installed and benched, D055), ~1 s speech, `look` / `find_object` and a scene memory (D054), world editor, recordings, RL panel, gesture studio; the phone over HTTPS. Place recognition (D057, off by default): 21/21 verdicts, changes 3/6 | [docs/COCKPIT_GUIDE.md](docs/COCKPIT_GUIDE.md), [docs/BRAINS.md](docs/BRAINS.md), [docs/PLACES.md](docs/PLACES.md) |
| **Hardware bridge** | `rocky_driver` + the cockpit's bridge, made safe before any servo touched it (D052): standstill-only mirror, a soft first move (parked goal, 40 % torque, 200 c/s), whole-leg fault cuts, EEPROM angle limits. **Mock-verified only: no real servo has been on the bus** | `sim/hw_bridge.py`, [bench/BENCH_RUNBOOK.md](bench/BENCH_RUNBOOK.md), B32 |
| **Design changes** | the robot's shape is data (D053): params `robot:` is validated by `rocky_model.robot()`, and the MJCF / URDF read their stations and actuator order from it; servo, length, foot and material changes follow a documented loop; a 4th joint or a 6th leg validates but does not run yet (B36) | [docs/DESIGN_CHANGE_GUIDE.md](docs/DESIGN_CHANGE_GUIDE.md), [docs/ROBOT_AS_DATA.md](docs/ROBOT_AS_DATA.md) |

## Layout

```
rocky/
├── README.md, BUILD_LOG.md, NOTES_INBOX.md   this file · the notebook (newest first) · the raw inbox
├── rocky.sh             launcher: `./rocky.sh help` lists every command
├── rocky.env.example    every machine setting (ROCKY_*) with its default; copy to rocky.env (git-ignored)
├── pyproject.toml       pip-installable core (gait modules + bus driver) and the extras
├── cad/                 build123d parametric parts; params.yaml is the single source of truth
│   ├── run_all_checks.py   the CAD CI: 28 modules incl. the joint suite + printability (--derived: +4 generators)
│   ├── ref/             the measured ST3215 STEP every leg part is derived from (+ its Apache-2.0 license)
│   ├── out/             exported STL/STEP/PNG (git-lfs) and PRINT_PREP_PACK.pdf, the print pack
│   ├── pebble_viewer.html  self-contained WebGL viewer (git-lfs)
│   └── leg_frame.py, servo_st3215.py, servo_mount.py, part_*.py, check_*.py
├── gait/                pure-numpy control: params loader (rocky_model.py), wave gait + IK, feasibility
│                        checker, reflex supervisor, watchdog, gestures + keyframe player (gestures/*.json)
├── sim/                 MuJoCo: build_mjcf.py (from params + CAD masses), playground.py, cockpit.py +
│   │                    cockpit_ui.html, world_builder.py, scenes.py (shared scene helpers), envfile.py
│   │                    (reads rocky.env), rocky_env.py + rocky_recover_env.py (gymnasium), train_ppo.py,
│   │                    eval_*.py, runs/ (checkpoints), brain_install.py + brain_bench.py,
│   │                    place_memory.py + place_bench.py, run_sim.py (the CI smoke run)
│   └── experiments/     one-off experiments, run from the repo root; results/ (pre_d052/ = older records)
├── harness/             tool registry (capabilities.py → docs/TOOLS.md), MCP server, mock / sim / cockpit
│                        backends, local_brain.py (LLM), intent.py (regex brain + voice pipe)
├── driver/              rocky_driver: dual-protocol Feetech bus driver + a byte-faithful mock
├── bench/               bench-day scripts (all runnable --mock) + BENCH_RUNBOOK.md
├── perception/          legged-odometry EKF, ICP scan matching, cliff detector, foot contacts
├── ros2/                ROS 2 packages (scaffold; URDF generated from params, parity-checked)
├── audio/               chord-speak voice + samples
├── bom/                 BOM.csv, the one bill of materials, + README.md + totals.py
├── docs/                guides, decisions, backlog, interfaces, reviews; archive/ = superseded plans, old logs
└── media/               photos, GIFs
```

## Setup

Python ≥ 3.10 (the reference venv runs 3.12) on Linux or WSL2. Everything runs
from the repo root, and `rocky.sh` expects the venv at `.venv`.

```bash
git clone https://github.com/2BitwiseBard/rocky && cd rocky
python3 -m venv .venv && . .venv/bin/activate        # or: uv venv && . .venv/bin/activate
pip install -e ".[sim,harness,cockpit,rl,dev]"       # editable: the core reads ../cad and ../sim
cp rocky.env.example rocky.env                       # this machine's settings: uncomment what differs
./rocky.sh help                                      # every launcher command and the env it reads
```

- **Extras:** `sim` (MuJoCo, gymnasium, video), `harness` (MCP server, the
  OpenAI-compatible client), `cockpit` (the browser playground; `anthropic` for
  the Claude brain), `rl` (torch), `dev` (pytest, ruff); on demand `cad`
  (build123d, pinned, and the print pack), `hw` (pyserial, for a real servo bus),
  `ollama` (`harness.local_brain` without `--base-url`), `viewer` (playwright,
  for `cad/test_viewer.py`).
- **GPU torch:** install torch from pytorch.org's index for your CUDA *before*
  the `rl` extra so pip keeps it; CPU torch is fine for evaluation.
- **Sound and video:** chord-speak plays through `aplay` (alsa-utils) or
  `ffplay`; videos need ffmpeg.
- **Brains and voice:** any OpenAI-compatible model server (llama-swap,
  llama-server, Ollama, LM Studio, vLLM) and a whisper server with
  `/v1/audio/transcriptions`. Endpoints and model ids are `ROCKY_*` settings in
  `rocky.env`; the defaults are the reference setup measured in D055 (one 16 GB
  GPU running llama-swap), see [docs/BRAINS.md](docs/BRAINS.md).
- **Phone:** `./rocky.sh tailnet` publishes the cockpit over HTTPS on your
  tailnet (`https://<machine>.<tailnet>.ts.net:9445`); the cockpit itself binds
  127.0.0.1.
- **Claude Code over MCP:** `cp .mcp.json.example .mcp.json`, then
  `./rocky.sh chat` (the example documents `ROCKY_BACKEND`).
- **Windows:** WSL2 + Ubuntu runs everything verbatim (`wsl --install`; WSLg
  shows the MuJoCo window on Windows 11). Native Windows runs the sim with the
  same pip line, but `rocky.sh` is bash, so call the Python entry points the
  guides name; chord-speak then needs `ffplay` (ffmpeg).

## Quickstart

```bash
./rocky.sh test                                      # the fast ladder: driver, harness, gait, sim/tests (~8 min)
MUJOCO_GL=egl python sim/run_sim.py                  # the wave gait walks, headless; video in sim/out/run_sim/; exits 1 on a fall
MUJOCO_GL=glfw python sim/playground.py --viewer     # live window + REPL + arrow-key teleop (or ./rocky.sh play)
./rocky.sh cockpit                                   # browser playground: http://127.0.0.1:8765
./rocky.sh cad-check                                 # the CAD CI (needs the cad extra)
```

## Driving it

- **Simulation** — [docs/SIM_GUIDE.md](docs/SIM_GUIDE.md): how the sim is built
  from the CAD, the playground, the experiments, worlds, videos, and the MCP /
  LLM driving loop.
- **Cockpit** — [docs/COCKPIT_GUIDE.md](docs/COCKPIT_GUIDE.md): tabs, brains,
  saying stop, remote mode, the phone.
- **Tools** — [docs/TOOLS.md](docs/TOOLS.md) (generated from
  `harness/capabilities.py`): every tool a brain or an MCP client can call.
- **Brains** — [docs/BRAINS.md](docs/BRAINS.md): which models drive the robot,
  how to install and bench one.
- **Places** — [docs/PLACES.md](docs/PLACES.md): place recognition from the
  senses (D057).
- **Perception** — [docs/PERCEPTION_PLAN.md](docs/PERCEPTION_PLAN.md): the
  sensing and vision plan.
- **Reinforcement learning** — [docs/RL_GUIDE.md](docs/RL_GUIDE.md): the two
  environments, training and evaluation step by step, the honest results table.
- **Design changes** — [docs/DESIGN_CHANGE_GUIDE.md](docs/DESIGN_CHANGE_GUIDE.md),
  [docs/ROBOT_AS_DATA.md](docs/ROBOT_AS_DATA.md).
- **Hardware** — [bench/BENCH_RUNBOOK.md](bench/BENCH_RUNBOOK.md) (bench day),
  [docs/WIRING_HARNESS.md](docs/WIRING_HARNESS.md), [docs/INTERFACES.md](docs/INTERFACES.md),
  [docs/PRINT_PLAN.md](docs/PRINT_PLAN.md), [bom/BOM.csv](bom/BOM.csv).

## Ground rules

These are the ones that have actually cost something when broken.

- **`cad/params.yaml` is the single source of truth** for dimensions. Parts,
  the MJCF and the URDF are regenerated from it, never edited downstream.
- **Checks green before anything ships.** `cad/run_all_checks.py` 28/28 and
  the fast test suites; CI runs them on every push.
- **Sim honesty.** A negative result is a result and gets written down.
  Six recovery retrains (`recover2` → `recover6_d052`) failed to beat the
  policy they were meant to replace, and every one is recorded rather than
  buried. Shoves are quoted in N·s and bodyweights, and the demo shove is a
  shove, not a strike (D048).
- **The SCS0009 hand servo never sees 12 V.** It has its own 6 V rail (D016).

## Where the record lives

- [BUILD_LOG.md](BUILD_LOG.md) — the engineering notebook, newest entry first.
- [docs/decisions.md](docs/decisions.md) — numbered decisions D001–D060 (with
  the amendments D052a and D055a), cited everywhere.
- [docs/DESIGN_BACKLOG.md](docs/DESIGN_BACKLOG.md) — ideas and follow-ups with
  verdicts (B1–B65).
- Reviews: [docs/REVIEW_2026-09-22.md](docs/REVIEW_2026-09-22.md) (mechanical,
  electronics, repo), BUILD_LOG sessions 9f (2026-09-24, sim honesty, D052) and
  9i (2026-09-26, the whole-repo audit).
- [NOTES_INBOX.md](NOTES_INBOX.md) — raw measurements and results, filed later.
- [docs/archive/](docs/archive/README.md) — the founding plan, the early build
  log, full-length decision text and superseded reports.

## License

MIT (see `LICENSE`). The ST3215 reference STEP in `cad/ref/` is from
TheRobotStudio/SO-ARM100, Apache-2.0 (license text in
`cad/ref/LICENSE-Apache-2.0.txt`).
