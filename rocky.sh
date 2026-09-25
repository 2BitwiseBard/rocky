#!/usr/bin/env bash
# rocky.sh — Pebble launcher (repo root). Machine-specific glue only: the venv
# path, llama-swap as the local brain, whisper-server as the ear, viewer +
# speakers on. Everything it calls is documented in docs/SIM_GUIDE.md and
# docs/RL_GUIDE.md.
#
#   rocky.sh play [--cliff]        live MuJoCo window + REPL + arrow-key/SPACE teleop (terminal)
#   rocky.sh cockpit [--world W]   browser cockpit at http://127.0.0.1:8765 (cameras, chat
#                                  with talk/local/Claude brains, vision, world editor, RL);
#                                  Ctrl-C, the page's quit button or `cockpit-stop` end it.
#                                  It binds loopback only: for the phone use `rocky.sh tailnet`
#                                  (HTTPS, tailnet-only); --unsafe-lan is the one way to bind a
#                                  non-loopback address and it has no auth (D052)
#   rocky.sh cockpit-stop          stop a cockpit started elsewhere (e.g. in the background)
#   rocky.sh tailnet [PORT]        publish the cockpit on the tailnet over HTTPS via
#                                  `tailscale serve` (default https port 9445) — the phone
#                                  gets the cameras, the mic button and chord audio;
#                                  `tailnet off` removes it. Needs `sudo tailscale up` once.
#   rocky.sh chat                  Claude Code from the repo root: chat-drives the
#                                  robot over MCP with the window + speakers on
#   rocky.sh brain [-- args]       local fleet (qwen3.6-35b-a3b via llama-swap) drives it
#   rocky.sh brain-install [files|flags]  downloaded brain GGUFs (default: every complete
#                                  ~/Downloads/*.gguf) -> /mnt/models/<id>/ + its mmproj + the
#                                  restore manifest + an UNMEASURED llama-swap stanza, then
#                                  restarts llama-swap (unloads EVERY model); --dry-run shows the
#                                  plan and touches nothing, --no-restart skips the restart;
#                                  --uninstall ID [--delete-files] removes one it installed
#   rocky.sh brain-bench --models ID[,ID]  bench brain models on the robot's jobs
#                                  (sim/brain_bench.py: its own cockpit, never :8765)
#   rocky.sh talk                  plain-text brain, no LLM (harness.intent REPL)
#   rocky.sh voice [--backend mock]  push-to-talk: mic -> whisper-server -> intent (no wake
#                                  word: pressing Enter is the operator's confirmation, D052 V2)
#   rocky.sh test [pytest args]    the fast ladder, same suites as CI's fast job
#                                  (driver/harness/gait/sim); `-m slow` suites are optional:
#                                  .venv/bin/python -m pytest harness sim/tests -m slow -q
#   rocky.sh jobs                  training runs in flight + last log line of each
#   rocky.sh train-recover NAME [args]   capped v2 self-righting retrain (D045 recipe)
#   rocky.sh eval-recover NAME [args]    20-episode righting eval of runs/NAME
#   rocky.sh train-walk NAME [args]  residual-gait PPO run (see docs/RL_GUIDE.md)
#   rocky.sh eval-walk NAME [args]   deterministic eval of runs/NAME vs the bare gait
#   rocky.sh cad-check             whole-tree CAD CI (build123d) — 23/23 or it didn't happen
#   rocky.sh help                  this text (the whole header, so new commands show up)
#
# Env overrides: ROCKY_REPO, ROCKY_WORLD (flat|room|cliff), ROCKY_VIEWER (1|0), ROCKY_GESTURE_DIR,
# ROCKY_AUDIO (1|0), ROCKY_LLM_MODEL, ROCKY_VOICE_SECS, ROCKY_VOICE_WAV (test file
# instead of the mic), ROCKY_COCKPIT_PORT (tailnet's proxy target).
set -euo pipefail

ROCKY_REPO="${ROCKY_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
PY="$ROCKY_REPO/.venv/bin/python"
export MUJOCO_GL="${MUJOCO_GL:-glfw}"
export ROCKY_WORLD="${ROCKY_WORLD:-flat}"
export ROCKY_VIEWER="${ROCKY_VIEWER:-1}"
export ROCKY_AUDIO="${ROCKY_AUDIO:-1}"
export PYTHONUNBUFFERED=1

cmd="${1:-help}"; shift || true
# help must work before the venv exists (CI's launcher test runs it on a bare checkout)
[[ -x "$PY" || "$cmd" =~ ^(help|-h|--help)$ ]] \
  || { echo "no venv at $PY — see README.md Quickstart" >&2; exit 1; }
case "$cmd" in
  play)
    cd "$ROCKY_REPO/sim" && exec "$PY" playground.py --viewer "$@" ;;

  cockpit)
    # D049: the browser playground — cameras, chat with a switchable brain,
    # world editor, RL panel — on one running sim; `chat` drives it too
    # shellcheck disable=SC1090
    source "$HOME/.config/environment.d/local-ai.conf" 2>/dev/null || true
    cd "$ROCKY_REPO"
    for p in $(pgrep -f "^[^ ]*python sim/cockpit.py"); do kill "$p"; done   # one cockpit at a time
    # D052 voice: the fast whisper (base.en, CPU greedy, ~1 s per command) on :8086 when it is
    # up (whisper-fast.service), else the large-v3-turbo on :8082 (18-20 s per command on 6 CPU threads)
    if [[ -z "${ROCKY_WHISPER_URL:-}" ]] && curl -s -m 1 -o /dev/null http://127.0.0.1:8086/ 2>/dev/null; then
      export ROCKY_WHISPER_URL="http://127.0.0.1:8086"
    fi
    LOCAL_AI_KEY="${LOCAL_AI_KEY:-}" MUJOCO_GL=egl exec "$PY" sim/cockpit.py "$@" ;;

  cockpit-stop)
    for p in $(pgrep -f "^[^ ]*python sim/cockpit.py"); do kill "$p" && echo "stopped $p"; done ;;

  tailnet)
    # D051: the cockpit stays bound to 127.0.0.1; tailscale serve terminates HTTPS with a
    # tailnet cert and proxies to it. HTTPS matters: the browser only allows the mic
    # (getUserMedia) on a secure origin, so plain http://laptop:8765 has no voice button.
    TS_PORT="${1:-9445}"
    if [[ "${1:-}" == "off" ]]; then
      tailscale serve --https="${2:-9445}" off && echo "tailnet: cockpit unpublished"; exit 0
    fi
    if ! tailscale status >/dev/null 2>&1; then
      echo "tailscale is not up on this machine — run:  sudo tailscale up   (then re-run this)"; exit 1
    fi
    tailscale serve --bg --https="$TS_PORT" "http://127.0.0.1:${ROCKY_COCKPIT_PORT:-8765}" || exit 1
    HOSTDNS=$(tailscale status --json 2>/dev/null | "$PY" -c 'import json,sys; print(json.load(sys.stdin)["Self"]["DNSName"].rstrip("."))' 2>/dev/null)
    echo "tailnet: https://${HOSTDNS:-<this-machine>.tail54f481.ts.net}:${TS_PORT}  (tailnet only; run ./rocky.sh cockpit if it is not up)"
    tailscale serve status ;;

  chat)
    cd "$ROCKY_REPO" && exec claude "$@" ;;

  brain)
    # shellcheck disable=SC1090
    source "$HOME/.config/environment.d/local-ai.conf"
    cd "$ROCKY_REPO"
    ROCKY_LLM_API_KEY="$LOCAL_AI_KEY" exec "$PY" -m harness.local_brain \
      --base-url http://127.0.0.1:8080/v1 --model "${ROCKY_LLM_MODEL:-qwen3.6-35b-a3b}" "$@" ;;

  brain-install)
    # sim/brain_install.py (conventions in its docstring); the key is for the /v1/models
    # poll after the llama-swap restart. No cd: a relative FILE (e.g. run from ~/Downloads)
    # means the caller's directory — the script itself has no cwd dependence
    # shellcheck disable=SC1090
    source "$HOME/.config/environment.d/local-ai.conf" 2>/dev/null || true
    LOCAL_AI_KEY="${LOCAL_AI_KEY:-}" exec "$PY" "$ROCKY_REPO/sim/brain_install.py" "$@" ;;

  brain-bench)
    # sim/brain_bench.py: loads models one at a time through llama-swap (bench etiquette).
    # No cd, so a relative --out lands where the caller is, not in the repo root (its own
    # cockpit is started with cwd = the repo by vision_bench.start_cockpit)
    # shellcheck disable=SC1090
    source "$HOME/.config/environment.d/local-ai.conf" 2>/dev/null || true
    LOCAL_AI_KEY="${LOCAL_AI_KEY:-}" MUJOCO_GL=egl exec "$PY" "$ROCKY_REPO/sim/brain_bench.py" "$@" ;;

  talk)
    cd "$ROCKY_REPO" && exec "$PY" -m harness.intent "$@" ;;

  voice)
    cd "$ROCKY_REPO"
    secs="${ROCKY_VOICE_SECS:-4}"
    fifo="$(mktemp -u /tmp/rocky-voice.XXXXXX)"; mkfifo "$fifo"
    wav="$(mktemp /tmp/rocky-utt.XXXXXX)"
    # --no-wake: push-to-talk is an explicit operator act, which is what the wake
    # word stands in for on an always-on mic (D052 V2 review: it silently needed one)
    "$PY" -m harness.intent --stdin --no-wake "$@" < "$fifo" &
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
    "$PY" -m pytest gait -q "$@"
    "$PY" -m pytest sim/tests -m "not slow" -q "$@" ;;   # D052: CI runs these too

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
    cd "$ROCKY_REPO/sim"                          # train_ppo makes the run dir itself
    exec nice -n 10 "$PY" train_ppo.py --env recover --reward v2 --log-std-max -0.5 \
      --num-envs 8 --run-dir "runs/$name" "$@" ;;

  train-walk)
    name="${1:?run name}"; shift
    # D052: the env is called "gait" (walk was never a choice, so every train-walk died
    # in argparse and left an empty run dir behind); train_ppo makes the run dir itself
    cd "$ROCKY_REPO/sim"
    exec nice -n 10 "$PY" train_ppo.py --env gait --num-envs 8 --run-dir "runs/$name" "$@" ;;

  eval-walk)
    name="${1:?run name}"; shift
    cd "$ROCKY_REPO/sim" && MUJOCO_GL=egl exec "$PY" eval_ppo.py "runs/$name/latest.pt" --compare-zero "$@" ;;

  eval-recover)
    name="${1:?run name}"; shift
    cd "$ROCKY_REPO/sim" && MUJOCO_GL=egl exec "$PY" eval_recover.py "runs/$name/latest.pt" --episodes 20 "$@" ;;

  cad-check)
    cd "$ROCKY_REPO/cad" && exec "$PY" run_all_checks.py "$@" ;;

  help|-h|--help)
    # the whole leading comment block after the shebang, however long it grows
    awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "${BASH_SOURCE[0]}" ;;

  *)
    echo "unknown command: $cmd (try: rocky.sh help)" >&2; exit 2 ;;
esac
