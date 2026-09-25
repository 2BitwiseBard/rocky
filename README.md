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

## Status (2026-09-24)

| area | state | evidence |
|---|---|---|
| **CAD** | print-clean, rebuilt around a **measured STEP of the real ST3215 servo** (D047); four joint coupons gate the next print run | `cad/run_all_checks.py` 23/23 (joint suite, printability, single-solid gate) |
| **Physical build** | nothing assembled; the first prints (pre-D047) did not fit and are retired; servos not yet ordered | `media/2026-09-22_first_prints.jpg`, `docs/PRINT_PLAN_2026-09-22.md`, `bom/SHOPPING_LIST_2026-09-22.md` |
| **Sim + control** | since D052 the sim stops flattering the servo: one servo identity in `params.yaml` (damping = stall / no-load 0.63 N·m·s/rad, torque clipped at the 2.94 N·m stall/peak with the 1.9 N·m continuous budget enforced as a thermal model, foot μ 0.8), and every motion source — walk, teleop, goto, gestures, keyframe files, the brain — goes through one feasibility checker and the gait's speed envelope (45.5 mm/s, 0.246 rad/s in place). The wave gait walks 246 mm of ~261 commanded in 8 s, 0.84° tilt; turning shrank to what the servo can do (103° where the ideal model turned 155°). 20 gesture/gait rows audited, 0 FAIL, but tracking lag p95 is 13–19° on the fast ones. The reflex supervisor safe-stops, braces, hands off to a righter; an always-on void guard stops a walk at a table edge — since the D052 follow-up a foot that has not felt ground 140 ms after touchdown holds the gait until it does, which stopped the 10–20° approaches that walked off (walk 45 at 10 / 15 / 20°: fell → stopped 115–184 mm short; a 72-approach sweep 28 → 6 falls), and the retreat replays the approach backwards (backs off 39–59 mm); a 1° grid still finds a lip band that falls (7 of 310 approaches, D052a, strict xfail) | `docs/SIM_GUIDE.md`, `sim/audit_gestures.py`, `docs/decisions.md` D052 |
| **RL / self-righting** | a **hybrid**: RL rights the body, an analytic ramp stands it. Since D052 both envs train on the robot's own action path and sensors (EMA filter, servo model, noisy IMU / encoders / foot switches, no torso height) and the handoff is `handoff_ok`, computable on the Pi. Policy-to-handoff on the D052 model as amended (peak torque): `recover1` **5/20** on the legacy height test (3/20 with the old 1.911 N·m clip on the same tree, 2/20 in D052's run — noise at 20 episodes), **0/20** on `handoff_ok` (pure-RL 0/20) — pre-D052 it was 7/20 here, 12/20 in the cloud, flattered by an ideal actuator and a test that passed a robot kneeling on its shins. System level (supervisor + righter + the 3 s stall ramp): **20/20 vs 11/20 with no righter**, every declared fall ended by the stall ramp, none by a handoff (median 0.46 s to stand on the peak model, 0.80 with the clip). Every checkpoint on disk is pre-D052 (flagged `legacy obs`); no retrain on the new contract yet (B34). The walkers have no speed envelope on the D052 servo and are zeroed until retrained | `docs/RL_GUIDE.md` §4, `docs/RL_TOUR.md` |
| **Tests** | 325 fast tests + 2 strict xfail (driver 69, gait 47, harness 37, sim 172 + the 2 xfails: the void guard's lip band, D052a) in ~135 s; URDF ≡ MJCF ≡ analytic FK; CI also regenerates the model files and fails on a diff | GitHub Actions `ci.yml`, `./rocky.sh test` |
| **Harness** | six-tool MCP server on a mock and on MuJoCo (+ `look` where an eye exists); a local LLM (llama-swap / Ollama) or Claude drives it; no hardware backend yet | `harness/`, `docs/MCP_CONTRACT_v0.md` |
| **Cockpit** | browser playground on one running sim: chase + eye cameras, top-down map, chat with a switchable brain (regex / local model / multimodal / Claude, fallback chains, quarantined models unselectable) and wake-word-gated voice, a vision model behind `look`, world editor (presets, obstacles, terrain, friction that now reaches the feet, saved and random courses), recordings with replay, RL panel; feet feel for the floor (D050). D051: gesture studio, chord designer, servo realism, hardware panel. D052: every studio change is feasibility-checked (a FAIL is not played or saved), a reach solver and teach-by-demonstration, gait presets + `check`, guard chips (VOID, LATCHED, locomotion held), a model/fingerprint panel, goto with a reactive lidar layer; a dead sim thread fails loudly; loopback-only with a Host / same-origin guard, the phone via `rocky.sh tailnet` (written, **not yet exercised**) | `docs/SIM_GUIDE.md` §3b–3c, `sim/cockpit.py`, `sim/cockpit_brains.py` |
| **Hardware bus** | `rocky_driver` + the cockpit's bridge, made safe before any servo touched it (D052): sim → robot only from a standstill, a soft first move (goal parked where the leg is, 40 % torque, 200 c/s, blend), whole-leg fault cuts, silent servos dropped and re-armed only by name, NaN / 4.7 rad/s guards at the bus, heartbeat, port-loss limp, EEPROM angle limits (`bench/apply_limits.py`), the mirror dropped when the sim's reflex leaves NORMAL. Mutation-tested on the byte-faithful mock. **No real servo has been on the bus yet** | `sim/hw_bridge.py`, `bench/BENCH_RUNBOOK.md`, `docs/DESIGN_BACKLOG.md` B32 |
| **Design changes** | the robot's shape is data since D053. `params.yaml` `robot:` (legs, the joint chain, the foot, the tool, the IK solver) is validated by `rocky_model.robot()`, and the MJCF/URDF read their stations and actuator order from it. Servo, length, foot and material changes follow a documented loop. A 4th joint or a 6th leg validates as a description but is not runnable yet (steps 2-10) | `docs/DESIGN_CHANGE_GUIDE.md`, `docs/ROBOT_AS_DATA.md`, `docs/DESIGN_BACKLOG.md` B36 |
| **Review** | full project review with dispositions | `docs/REVIEW_2026-09-22.md` |

## Layout

```
rocky/
├── README.md, BUILD_LOG.md, NOTES_INBOX.md   this file · the notebook (newest first) · the raw inbox
├── rocky.sh             launcher: cockpit / tailnet / play / chat / brain / voice / test / train-* / eval-* / cad-check
├── pyproject.toml       pip-installable core (gait modules + bus driver) and the extras
├── cad/                 build123d parametric parts; params.yaml is the single source of truth;
│   ├── run_all_checks.py   the CAD CI: 23 modules incl. check_assembly (joints) + check_printability
│   ├── ref/             the ST3215 STEP model every leg part is derived from
│   ├── out/             exported STL/STEP/PNG (git-lfs) + pebble_viewer.html (self-contained WebGL)
│   └── leg_frame.py, servo_st3215.py, servo_mount.py, part_*.py
├── gait/                pure-numpy control: wave gait + IK, reflex supervisor, watchdog, gestures
├── sim/                 MuJoCo: build_mjcf.py (from params + CAD masses), playground.py,
│                        rocky_env.py + rocky_recover_env.py (gymnasium), train_ppo.py, eval_*.py,
│                        run_*.py experiments, runs/ (checkpoints, git-lfs),
│                        cockpit.py + cockpit_ui.html (browser playground), world_builder.py
│                        brain_install.py + brain_bench.py (rocky.sh brain-install / brain-bench: add a model, measure it)
├── harness/             MCP tool server (say/gesture/goto/stop/scan_summary/status), sim + mock
│                        backends, local_brain.py (LLM), intent.py (regex brain + voice pipe)
├── driver/              rocky_driver: dual-protocol Feetech bus driver, byte-faithful mock, 69 tests
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
pip install -e ".[sim,harness,cockpit,rl,dev]"      # + ".[cad]" for build123d
./rocky.sh test                                      # the fast suites CI runs (driver/harness/gait/sim)
MUJOCO_GL=egl python sim/run_sim.py                  # the wave gait walks, headless; exits 1 on a fall
MUJOCO_GL=glfw python sim/playground.py --viewer     # live window + REPL + arrow-key teleop
./rocky.sh cockpit                                   # browser playground: http://127.0.0.1:8765
```

`./rocky.sh help` lists every launcher command (they assume `.venv` in the
repo root and, for `brain`/`voice`, a local llama-swap and whisper-server).
The phone reaches the cockpit through `./rocky.sh tailnet` (HTTPS over the
tailnet); the cockpit itself stays on 127.0.0.1.

## Driving it

- **Simulation** — `docs/SIM_GUIDE.md`: how the sim is built from the CAD,
  the playground (controls, commands, live tuning), the browser cockpit
  (cameras, brains, vision, world editor), the experiment scripts, worlds,
  videos, and the MCP/LLM driving loop.
- **Tools** — `docs/TOOLS.md` (generated from `harness/capabilities.py`,
  D056): every tool a brain or an MCP client can call, its arguments,
  whether it needs the wake word, and the envelope its descriptions quote.
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
- `docs/decisions.md` — numbered decisions (D001–D052), cited everywhere.
- `docs/DESIGN_BACKLOG.md` — ideas with verdicts (shipped / negative / deferred).
- `NOTES_INBOX.md` — raw measurements and results, filed later.

## License

MIT (see `LICENSE`). The ST3215 reference STEP in `cad/ref/` is from
TheRobotStudio/SO-ARM100, Apache-2.0.
