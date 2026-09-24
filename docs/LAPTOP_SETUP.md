# Laptop setup — where each piece lives

This page used to carry the whole recipe and went stale every round
(teleop keys, righting scores, the RL command lines). It is now a map:
each topic has one owner document, and the numbers live only there.
(D052, 2026-09-24.)

## Install

Follow the **Quickstart in `README.md`**. On the laptop take every extra
the launcher uses:

```bash
pip install -e ".[sim,harness,cockpit,rl,dev]"     # + ".[cad]" to regenerate parts
./rocky.sh test                                      # the same fast suites CI runs
./rocky.sh help                                      # every launcher command
```

`rocky.sh` expects the venv at `.venv` in the repo root. For a GPU torch
build, install torch from pytorch.org's index for your CUDA *before* the
`rl` extra so pip keeps it. Chord-speak plays through `aplay`
(alsa-utils) or `ffplay` (ffmpeg); videos need ffmpeg.

## Drive it

`docs/SIM_GUIDE.md` — the playground window (`./rocky.sh play`), the
browser cockpit (`./rocky.sh cockpit`; on the phone via
`./rocky.sh tailnet`, never by binding the cockpit to the LAN), the
experiment scripts, worlds, and the MCP/LLM driving loop. For Claude
Code over MCP: `cp .mcp.json.example .mcp.json`, then `./rocky.sh chat`;
the example documents `ROCKY_BACKEND`.

## Train it

`docs/RL_GUIDE.md` — the environments, the honest results table, and
train / evaluate / resume step by step. The launcher shortcuts are
`./rocky.sh train-recover NAME`, `eval-recover NAME`, `train-walk NAME`,
`eval-walk NAME` and `jobs`. Read `docs/RL_TOUR.md` before changing a
reward.

## A local brain

`./rocky.sh brain` — llama-swap's local model drives the robot
(`ROCKY_LLM_MODEL` picks it); `./rocky.sh talk` is the no-LLM text
brain and `./rocky.sh voice` puts whisper-server in front of it. Details
in `docs/SIM_GUIDE.md` §5.

## Keeping in sync

Git is the source of truth (since 2026-09-22). `BUILD_LOG.md` and
`NOTES_INBOX.md` are the record; `docs/decisions.md` has the D-numbers.

## Windows (the printer laptop)

Easiest: **WSL2 + Ubuntu**, then everything above works verbatim
(`wsl --install`; WSLg shows the MuJoCo window on Windows 11). Native
Windows also runs the sim: Python 3.11+, the same `pip install -e` line
(CPU torch is fine there), no `aplay` (chord-speak uses `ffplay` if
ffmpeg is installed). `rocky.sh` is bash, so natively call the Python
entry points the guides name instead. Keep training on the Linux box.
