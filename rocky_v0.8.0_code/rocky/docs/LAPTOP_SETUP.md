# Laptop setup — run the whole Pebble stack on your own machine

Everything the cloud sessions run, on your Linux laptop, with three
upgrades the cloud can't give you: a LIVE MuJoCo window, your NVIDIA GPU
for RL, and real speakers for chord-speak. Written session 8c; Linux
first, Windows notes at the bottom.

## 0 · One-time install (Linux, ~5 min)

```bash
unzip rocky_v0.7.9_code.zip -d ~/rocky && cd ~/rocky/rocky   # or wherever
python3 -m venv .venv && source .venv/bin/activate
pip install -e .                          # pebble_gait + rocky_driver
pip install mujoco gymnasium imageio imageio-ffmpeg numpy-stl pyyaml pytest
# GPU torch (check https://pytorch.org for your CUDA; cu124 is typical):
pip install torch --index-url https://download.pytorch.org/whl/cu124
python3 -c "import torch; print(torch.cuda.is_available())"   # want True
sudo apt install ffmpeg                   # videos; alsa-utils gives aplay
```

CAD extras only if you'll regenerate parts locally (not needed for sim):
`pip install build123d trimesh shapely rtree`.

Smoke test — should print walk metrics and "fell over: no":

```bash
cd sim && python3 run_sim.py
```

On a laptop you do NOT need `MUJOCO_GL=osmesa` (that's the cloud's
headless crutch). If a live window ever fails to open, `export
MUJOCO_GL=glfw` and make sure you're not on Wayland-only without
XWayland.

## 1 · The live playground (window + REPL + keyboard)

```bash
cd sim && python3 playground.py --viewer
```

A MuJoCo window opens with the robot standing. Two ways to drive it:

* **Type at the `pebble>` prompt:** `walk 45 0 0`, `stop`, `gesture wave`
  (10 gestures), `say greeting`, `push 40 0`, `set gait.T 1.3`, `show`,
  `record on` / `record`, `quit`. `--cliff` loads the table-edge world so
  you can drive it at the void and watch the guard win.
* **Keyboard teleop (new, 8c):** click the window, then W/S/A/D nudge
  forward/back/strafe, Q/E turn, SPACE = safe-stop, G = wave. Taps
  accumulate (the viewer only reports key presses, not releases), so tap
  W three times for 45 mm/s and SPACE to kill everything.

`set` changes are SIM-ONLY (D004: keepers go NOTES_INBOX → params.yaml →
regen). The honesty box in `docs/PLAYGROUND.md` applies: trends real,
absolute forces directional until bench-day calibration.

## 2 · Claude chat-drives the robot (the brain, today)

The repo IS an MCP server. Point any MCP client at it and the six tools
(say / gesture / goto / stop / scan_summary / status) run on real
physics with the reflex guards live.

```bash
cp .mcp.json.example .mcp.json     # then edit: absolute repo path
```

For **Claude Code**: run `claude` from the repo root — it picks up
`.mcp.json` — then just talk: "introduce yourself and look around",
"walk to (0.5, 0.2)" *(meters!)*, "do jazz hands". For **Claude
Desktop**: paste the same block into its MCP settings file. Two
environment knobs in the config's `env`:

* `ROCKY_BACKEND=sim` (physics) or `mock` (millisecond-fast contract)
* `ROCKY_WORLD=flat` (new, 8c — open floor, good for driving around),
  `room` (new, 8d — walls/pillars/crate, lidar has something to see:
  `scan_summary` works everywhere now and the patrol demo lives here) or
  `cliff` (the guard-test island; every long goto meets an edge, which
  is the point)

The marquee behavior to try first: in the cliff world, tell it to walk
somewhere far away. The model can want whatever it wants — the goto
comes back `stopped: "cliff"` with the body held short of the edge.
Guard supremacy is the architecture, and you can feel it from the chat.

## 3 · A local LLM as the brain (fully offline)

```bash
# once:
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen2.5:7b            # good tool-caller; llama3.1:8b also fine
pip install ollama

# then, from the repo root:
ROCKY_WORLD=flat python3 -m harness.local_brain
```

Type to it; it decides which tools to call; the sim executes. This is
the VISION_PLAN rehearsal: the brain is swappable (Claude over MCP,
Ollama locally, someday the onboard NPU) while the robot's tool surface
and guards never change. `--mock-llm` runs a scripted fake brain to test
the plumbing without Ollama; `--backend mock` skips MuJoCo. Your GPU
makes 7B–14B models snappy.

## 4 · RL on the GPU

```bash
cd sim
python3 train_ppo.py --num-envs 8 --cmd-sample --run-dir runs/mine \
        --resume ../path/to/runs/cmd_sample3/latest.pt   # warm start (optional)
python3 eval_ppo.py runs/mine/latest.pt --episodes 5 --compare-zero --video out.mp4
python3 train_ppo.py --env recover --run-dir runs/recover2   # self-righting
python3 eval_recover.py runs/recover2/latest.pt --episodes 20 --video rec.mp4
```

Notes that save you an evening: these envs are physics-bound, so
`--num-envs 8`–16 (one per CPU core, minus a couple) matters more than
the GPU — the GPU speeds up the PPO update, the CPUs the rollouts.
Checkpoints save atomically every 10 updates; `--resume auto` continues
a run-dir; `train_log.jsonl` is plottable with pandas in two lines. The
shipped checkpoints: `runs/robust_fwd2` (forward, 3 M steps, return
228.6), `runs/cmd_sample3` (all-command, 7 M), `runs/recover1`
(self-righting, 2 M — 12/20 falls end standing via the hybrid handoff).
Read `docs/RL_TOUR.md` before changing rewards — it's the ground-up
guide with the experiment ladder.

## 5 · Keeping your laptop and the cloud sessions in sync

The zips are the source of truth (project rule). Work freely, but: sim
discoveries and parameter keepers go in **NOTES_INBOX.md**, and if you
change code, say so at the next session start so the session builds on
your copy (attach your zip) rather than the last delivered one. Best
split while you're learning: tweak rewards, gestures, playground and
training configs locally; leave params.yaml/CAD regeneration to sessions
(the D036/D038 gates live there) unless you install the CAD extras and
run `cad/run_all_checks.py` yourself — 21/21 or it didn't happen.

## Windows (the printer laptop)

Easiest: **WSL2 + Ubuntu**, then everything above works verbatim
(`wsl --install`, and WSLg shows the MuJoCo window on Win 11). Native
Windows also works for the sim: install Python 3.11+, the same pips
(torch CPU is fine there), and skip `aplay` (chord-speak uses ffplay if
ffmpeg is installed). The playground viewer runs natively on Windows;
`--script` mode always works. Keep training on the Linux box.

## What runs where — cheat sheet

| you want | command |
|---|---|
| watch it walk, poke it | `sim/playground.py --viewer` |
| drive it with keys | click the viewer window, W/A/S/D/Q/E/SPACE |
| chat-drive with Claude | `.mcp.json` + Claude Code/Desktop |
| offline LLM brain | `python3 -m harness.local_brain` |
| no-LLM text brain (fastest) | `python3 -m harness.intent` — plain text -> tools; voice: pipe whisper.cpp into `--stdin` |
| lidar patrol mission | `ROCKY_WORLD=room python3 sim/run_patrol.py` |
| shove -> tumble -> self-right -> walk | `sim/run_reflex_fallen.py --video rec.mp4` |
| stall margins on real masses | `sim/torque_audit.py` |
| train gait RL | `sim/train_ppo.py --num-envs 8 --cmd-sample` |
| train self-righting | `sim/train_ppo.py --env recover` |
| watch a policy | `sim/eval_ppo.py <ckpt> --video out.mp4` |
| all gestures, judged | `sim/run_gestures2.py --metrics` |
| whole-tree CAD check | `cad/run_all_checks.py` (needs CAD extras) |
| servo-day rehearsal | `bench/bus_scan.py --mock` etc. |
