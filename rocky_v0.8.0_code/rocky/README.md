# Project ROCKY 🪨

A radially symmetric pentapod robot inspired by Rocky from *Project Hail Mary*.
Current phase: **Pebble** — the 1:3 subscale prototype (cat-sized, ~2.7 kg,
15× ST3215 bus servos, transforming three-finger hands, purple).

**The three memory layers of this project:**

| Layer | Lives where | Holds |
|---|---|---|
| This repo | your machine / GitHub | source of truth: CAD, code, BOM, logs |
| Claude Project docs | claude.ai "Robotics" project | cross-session memory: plan, status log, conventions |
| `BUILD_LOG.md` + `NOTES_INBOX.md` | repo root | the engineering notebook (see below) |

## Layout

```
rocky/
├── README.md            you are here
├── BUILD_LOG.md         engineering notebook — dated entries, newest first
├── NOTES_INBOX.md       scratch inbox: paste field notes here, Claude files them
├── setup.py             `pip install -e .` -> pebble_gait + rocky_driver anywhere
├── plan/                master plan (phases, budget, physics)
├── bom/                 order sheets: Batch 0+1, Batch-3 addendum (live-priced),
│                        session-5 addendum (deltas, est. prices)
├── cad/                 parametric CAD (build123d) — params.yaml is the SSOT
│   ├── iface.py         the six standard interfaces (D020) as shared geometry
│   ├── run_all_checks.py  the tree's CI — every part's checks, one command,
│   │                      then check_printability.py (D038) over every STL
│   ├── pebble_viewer.html self-contained WebGL viewer v0.3 (7 modes)
│   └── out/             exported STEP (edit/measure) + STL (slice/print) + renders
├── driver/              rocky_driver: dual-protocol Feetech bus driver + 58 tests
├── bench/               bench-day scripts (all rehearsable --mock) + BENCH_RUNBOOK.md
├── gait/                gait engine + reflex supervisor (v2 safe-stop) + stuck watchdog
├── sim/                 MuJoCo: walking, terrain, push, odometry, cliff, lidar, SLAM-lite,
│                        RL env + train_ppo.py / eval_ppo.py (laptop training kit);
│                        rocky_recover_env.py (self-righting, --env recover);
│                        mass_audit.py (CAD volumes -> mass_budget.json -> MJCF, D039);
│                        run_gestures2.py (gesture library v2 in physics, D040);
│                        torque_audit.py (stall margins on CAD masses, D044);
│                        run_reflex_fallen.py (shove->tumble->self-right->walk, D042);
│                        run_patrol.py (tool-surface patrol mission, D043)
├── harness/             Rocky-MCP tool server v0 (mock + MuJoCo sim backends) + tests;
│                        local_brain.py (Ollama), intent.py (no-LLM text brain + voice pipe)
├── perception/          legged-odom EKF (+yaw zupt), ICP scan matching, cliff, WiFi RSSI
├── ros2/                ROS 2 Jazzy packages (URDF generated from params, parity-checked)
├── audio/               chord-speak v0.2 "Eridian" voice + event narrator + A/B audition
├── docs/                decisions, interfaces, harness, star-board, sensing,
│                        VISION_PLAN (VLM/interaction), NEXT_SESSION, backlog
└── media/               renders, screenshots
```

Videos (sim/): `pebble_showcase_v07.mp4` (gait medley + push/safe-stop +
cliff stop, narrated), `pebble_stuck_retry_40mm_voiced.mp4`,
`pebble_gestures.mp4` (jazz hands + fist bump).

## Quickstart

```bash
pip install -e .                       # pebble_gait + rocky_driver, no ROS needed
pip install mujoco build123d numpy-stl pytest   # dev extras

cd driver && pytest tests/             # 58 tests, zero hardware
cd bench && python3 bus_scan.py --mock # rehearse the whole servo bench day
cd sim && MUJOCO_GL=osmesa python3 run_sim.py        # it walks
cd sim && python3 train_ppo.py --num-envs 8          # overnight RL (laptop)
cd cad && python3 part_hand.py         # rebuild any part after params changes
cd cad && python3 run_all_checks.py    # verify the WHOLE tree (20 modules + printability, parallel)
cd cad && python3 check_printability.py hand_hub   # one part, verbose: islands/thin walls/overhangs
cd sim && python3 mass_audit.py && python3 build_mjcf.py   # CAD masses -> sim
cd sim && MUJOCO_GL=osmesa python3 run_gestures2.py --metrics   # 7 gestures in physics
cd sim && python3 train_ppo.py --env recover --run-dir runs/recover1   # self-righting RL
python3 -m pytest harness/ -m "not slow"   # MCP harness contract tests (~1 s)
cd sim && MUJOCO_GL=glfw python3 playground.py --viewer   # DRIVE it: REPL + KEYBOARD (WASD/QE/SPACE)
python3 -m harness.local_brain                # a LOCAL LLM (Ollama) as the brain, guards live
python3 -m harness.intent                     # talk to Pebble in PLAIN TEXT, no LLM (voice-pipe ready)
ROCKY_WORLD=room MUJOCO_GL=osmesa python3 sim/run_patrol.py   # lidar patrol + chord-speak report
cd sim && MUJOCO_GL=osmesa python3 run_reflex_fallen.py --episodes 5  # it gets shoved over and GETS UP
cd sim && python3 torque_audit.py             # stall margins on the CAD mass budget
# docs/LAPTOP_SETUP.md = the full own-machine guide; docs/RL_TOUR.md = RL from zero
cd ros2/rocky_description && python3 generate_urdf.py && \
    python3 ../../sim/check_urdf_parity.py           # URDF == MJCF == analytic FK
```

## Status (2026-07-31, session 5 — see BUILD_LOG for the full story)

- **Bench + ROS 2 readiness: DONE** (session 4) — servo-arrival day is a
  runbook; URDF/MJCF/analytic FK in parity.
- **Sim, phase-aligned truth (D025):** the bare wave gait is the best
  single-impulse push controller; reflex v2 = finish-the-step → tilt-vector
  brace, kept as the SAFE-STOP primitive. Stuck-watchdog 4/4 at 40 mm.
- **Perception:** EKF yaw drift fixed (6.8°→0.25° flat walk, 15°→1.7°
  patrol) via stillness-gated zero-yaw-rate updates (D026); SLAM-lite ICP
  corrects the lap to 8 mm ATE / 0.06° with map IoU 0.72 (D027) — no ROS
  needed yet.
- **RL:** `train_ppo.py`/`eval_ppo.py` — the laptop kit: vec envs, DR,
  checkpoint/resume, video eval. Verified end-to-end in-session.
- **Chord-speak v0.2 (D028):** the Eridian voice — JI chord-syllables, one
  formant throat, event-driven narration; the stuck-retry video narrates
  itself. `audio/audition_v2.html` for the v0-vs-v0.2 A/B.
- **Carapace v0 (D029):** five identical printable rock sectors + hatch cap
  with arches, gills, LED groove, I6 bars, latch feet — all keep-outs
  boolean-clean; viewer has a Carapace mode.
- **Wiring:** star-board build plan + bracket + computed cut-length table
  (`docs/BUS_STARBOARD.md`).
- STILL nothing printed (refurb is tonight) and no servos — the moment
  either changes, the prepared pipelines take over.

## Session 6 (2026-08-07) — print-weekend prep

- **Fit ladder v2**: + Ø2 pin / 683ZZ press / Ø10 tube rows; v1's
  heat-set-pocket-into-slot merge bug found and fixed (D033: plate layout
  audits now mandatory + coded into the module).
- **Servo blanks (B17)**: printed ST3215 stand-ins, boolean-proven in all
  three cradles — the leg chain assembles BEFORE the servo order lands.
- **Tibia clamp v0.2**: pinch bolt got a real thread side + spot-faced seat.
- **`docs/PRINT_WEEKEND_s6.html`**: the weekend plan — plates P1–P6, the
  Saturday GO/NO-GO caliper table (D032 slicer fallback), dry-fit guide.
- **`docs/MCP_CONTRACT_v0.md`** — and (6b) the harness IMPLEMENTING it:
  `harness/` MCP server, mock + real-physics backends, guard-supremacy
  proven by tests (goto into the void over MCP → `stopped:"cliff"`).
- **Cliff stop path** = PLANT→BRACE safe-stop (D034, measured: no more
  marching at the edge). **Beckon gesture shipped** (B18, narrated video;
  D035 joint-space lesson). Viewer has a "Print weekend" dry-fit mode.
- **`docs/SCALE_UP_NOTES.md`** (what survives the Pebble→Rocky jump).
- Tree CI: **20/20 modules** (now parallel, ~half the wall time).

Next: PLACE THE CARTS (servo lead time is the critical path — Seeed has
ST3215 in stock if speed beats price), then plates P1–P6 per the weekend
plan; measurements → NOTES_INBOX → session-7 regen.
