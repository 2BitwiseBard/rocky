#!/usr/bin/env bash
# rocky.sh — Pebble sim launcher for this laptop.
#
# Lives OUTSIDE the repo on purpose: the delivered drop is the project's source
# of truth, and this file carries only the machine-specific glue (venv path,
# llama-swap as the local brain, whisper-server as the ear, viewer + speakers on).
#
#   rocky.sh play [--cliff]        live MuJoCo window + REPL + WASD/QE/SPACE teleop
#   rocky.sh chat                  Claude Code from the repo root: chat-drives the
#                                  robot over MCP with the window + speakers on
#   rocky.sh brain [-- args]       local fleet (qwen3.6-35b-a3b via llama-swap) drives it
#   rocky.sh talk                  plain-text brain, no LLM (harness.intent REPL)
#   rocky.sh voice [--backend mock]  push-to-talk: mic -> whisper-server -> intent
#   rocky.sh test                  the fast verification ladder (driver/harness/gait/intent)
#   rocky.sh jobs                  training runs in flight + last log line of each
#   rocky.sh train-recover NAME [args]   capped v2 self-righting retrain (D045 recipe)
#   rocky.sh eval-recover NAME [args]    20-episode righting eval of runs/NAME
#   rocky.sh cad-check             whole-tree CAD CI (build123d) — 21/21 or it didn't happen
#
# Env overrides: ROCKY_REPO, ROCKY_WORLD (flat|room|cliff), ROCKY_VIEWER (1|0),
# ROCKY_AUDIO (1|0), ROCKY_LLM_MODEL, ROCKY_VOICE_SECS, ROCKY_VOICE_WAV (test file
# instead of the mic).
set -euo pipefail

ROCKY_REPO="${ROCKY_REPO:-$HOME/Development/rocky/rocky_v0.8.0_code/rocky}"
PY="$ROCKY_REPO/.venv/bin/python"
export MUJOCO_GL="${MUJOCO_GL:-glfw}"
export ROCKY_WORLD="${ROCKY_WORLD:-flat}"
export ROCKY_VIEWER="${ROCKY_VIEWER:-1}"
export ROCKY_AUDIO="${ROCKY_AUDIO:-1}"
export PYTHONUNBUFFERED=1

[[ -x "$PY" ]] || { echo "no venv at $PY — see docs/LAPTOP_SETUP.md §0" >&2; exit 1; }

cmd="${1:-help}"; shift || true
case "$cmd" in
  play)
    cd "$ROCKY_REPO/sim" && exec "$PY" playground.py --viewer "$@" ;;

  chat)
    cd "$ROCKY_REPO" && exec claude "$@" ;;

  brain)
    # shellcheck disable=SC1090
    source "$HOME/.config/environment.d/local-ai.conf"
    cd "$ROCKY_REPO"
    ROCKY_LLM_API_KEY="$LOCAL_AI_KEY" exec "$PY" -m harness.local_brain \
      --base-url http://127.0.0.1:8080/v1 --model "${ROCKY_LLM_MODEL:-qwen3.6-35b-a3b}" "$@" ;;

  talk)
    cd "$ROCKY_REPO" && exec "$PY" -m harness.intent "$@" ;;

  voice)
    cd "$ROCKY_REPO"
    secs="${ROCKY_VOICE_SECS:-4}"
    fifo="$(mktemp -u /tmp/rocky-voice.XXXXXX)"; mkfifo "$fifo"
    wav="$(mktemp /tmp/rocky-utt.XXXXXX)"
    "$PY" -m harness.intent --stdin "$@" < "$fifo" &
    brain_pid=$!
    exec 3>"$fifo"
    trap 'exec 3>&- 2>/dev/null; kill "$brain_pid" 2>/dev/null; rm -f "$fifo" "$wav"' EXIT
    echo "push-to-talk — Enter: record ${secs}s and send · q: quit"
    while IFS= read -r -p "mic> " key; do
      [[ "$key" == q ]] && break
      if [[ -n "${ROCKY_VOICE_WAV:-}" ]]; then
        cp "$ROCKY_VOICE_WAV" "$wav"
      else
        arecord -q -d "$secs" -f S16_LE -r 16000 -c 1 "$wav"
      fi
      text="$(curl -s -m 60 http://127.0.0.1:8082/v1/audio/transcriptions \
                -F file=@"$wav" -F response_format=text -F temperature=0.0 | tr '\n' ' ' | sed -E 's/^ +| +$//g')"
      echo "heard: ${text:-(nothing)}"
      [[ -n "$text" ]] && printf '%s\n' "$text" >&3
      sleep 0.3
    done ;;

  test)
    cd "$ROCKY_REPO"
    "$PY" -m pytest driver/tests -q "$@"
    "$PY" -m pytest harness -m "not slow" -q "$@"
    "$PY" -m pytest gait -q "$@" ;;

  jobs)
    # one line per run (AsyncVectorEnv forks a worker per env, all matching)
    pgrep -af "train_ppo.py" | grep -v pgrep | grep -o -- '--run-dir [^ ]*' | sort -u \
      | sed 's/^--run-dir /  running: /' || echo "  no training in flight"
    for f in "$ROCKY_REPO"/sim/runs/*/train.out; do
      [[ -f "$f" ]] || continue
      printf '  %-22s %s\n' "$(basename "$(dirname "$f")")" "$(tail -n1 "$f")"
    done ;;

  train-recover)
    name="${1:?run name}"; shift
    cd "$ROCKY_REPO/sim" && mkdir -p "runs/$name"
    exec nice -n 10 "$PY" train_ppo.py --env recover --reward v2 --log-std-max -0.5 \
      --num-envs 8 --run-dir "runs/$name" "$@" ;;

  eval-recover)
    name="${1:?run name}"; shift
    cd "$ROCKY_REPO/sim" && MUJOCO_GL=egl exec "$PY" eval_recover.py "runs/$name/latest.pt" --episodes 20 "$@" ;;

  cad-check)
    cd "$ROCKY_REPO/cad" && exec "$PY" run_all_checks.py "$@" ;;

  help|-h|--help)
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//' ;;

  *)
    echo "unknown command: $cmd (try: rocky.sh help)" >&2; exit 2 ;;
esac
