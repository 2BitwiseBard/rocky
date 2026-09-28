#!/usr/bin/env bash
# rocky.sh — Pebble launcher (repo root). Machine-specific glue only: the venv
# path, a local LLM server as the brain, a whisper server as the ear, viewer +
# speakers on. This machine's settings live in ./rocky.env (git-ignored; every
# knob and its default: rocky.env.example), sourced first — the caller's
# environment wins over it. Everything it calls is documented in docs/SIM_GUIDE.md
# and docs/RL_GUIDE.md.
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
#                                  `tailscale serve` (https port: PORT, else ROCKY_TAILNET_PORT,
#                                  default 9445) and print its https://<machine>.<tailnet>.ts.net
#                                  address — the phone gets the cameras, the mic button and chord
#                                  audio; `tailnet off [PORT]` removes it. Needs `sudo tailscale up` once.
#   rocky.sh chat                  Claude Code from the repo root: chat-drives the
#                                  robot over MCP with the window + speakers on
#   rocky.sh brain [-- args]       a local LLM drives it (harness.local_brain): any OpenAI-compatible
#                                  server, ROCKY_LLM_BASE_URL (default llama-swap on :8080) and
#                                  ROCKY_LLM_MODEL (default qwen3.6-35b-a3b); key optional
#   rocky.sh brain-install [files|flags]  downloaded brain GGUFs (default: every complete *.gguf
#                                  in --downloads, default ~/Downloads, ROCKY_DOWNLOADS_DIR) ->
#                                  <--models-dir>/<id>/ (default ~/models, ROCKY_MODELS_DIR) + its
#                                  mmproj + the restore manifest + an UNMEASURED llama-swap stanza,
#                                  then restarts llama-swap (unloads EVERY model); --dry-run shows the
#                                  plan and touches nothing, --no-restart skips the restart;
#                                  --uninstall ID [--delete-files] removes one it installed
#   rocky.sh brain-bench --models ID[,ID]  bench brain models on the robot's jobs
#                                  (sim/brain_bench.py, llama-swap only: its own cockpit, never the live one)
#   rocky.sh talk                  plain-text brain, no LLM (harness.intent REPL)
#   rocky.sh voice [--backend mock]  push-to-talk: mic -> whisper (ROCKY_WHISPER_URL, default
#                                  :8082) -> intent (no wake word: pressing Enter is the
#                                  operator's confirmation, D052 V2)
#   rocky.sh test [pytest args]    the fast ladder: driver, harness, gait, sim/tests (-m "not slow"),
#                                  on the reference setup like CI (rocky.env is NOT read; opt in
#                                  with ROCKY_ENV_FILE=rocky.env ./rocky.sh test). CI's fast job also
#                                  runs ruff and the slow tests
#                                  (.venv/bin/python -m pytest harness sim/tests -m slow -q), run_sim,
#                                  URDF parity and the MJCF/URDF/TOOLS.md freshness checks
#   rocky.sh jobs                  training runs in flight + last log line of each
#   rocky.sh train-recover NAME [args]   capped v2 self-righting retrain (D045 recipe)
#   rocky.sh eval-recover NAME [args]    20-episode righting eval of runs/NAME
#   rocky.sh train-walk NAME [args]  residual-gait PPO run (see docs/RL_GUIDE.md)
#   rocky.sh eval-walk NAME [args]   deterministic eval of runs/NAME vs the bare gait
#   rocky.sh cad-check [--derived] [--fem]  whole-tree CAD CI (build123d): 28/28 modules or it didn't happen;
#                                  --derived also rebuilds the preview, print estimate, print pack,
#                                  viewer and part drawings (33/33); --fem also runs the leg stress check
#                                  (cad/fem_check.py: gmsh + CalculiX, D061) — SKIPPED without them
#   rocky.sh cad-drawings [PART...] [--force]  A4 TechDraw sheets of the leg parts (or PART) in
#                                  cad/out/drawings/ (FreeCAD, no window; a part is redrawn only when
#                                  its STEP changed; also part of cad-check --derived)
#   rocky.sh cad-open [PART|FILE...]  open STEP files in FreeCAD (default: the posed leg assembly);
#                                  PART = a cad/out/PART.step name; --fem PART opens that part's
#                                  stress result (after cad-check --fem). With FreeCAD's MCP
#                                  add-on running, Claude can then inspect / measure / screenshot it
#   rocky.sh help                  this text (the whole header, so new commands show up)
#
# Env the launcher itself reads (all of them, with defaults: rocky.env.example): ROCKY_REPO,
# ROCKY_ENV_FILE (another rocky.env), ROCKY_WORLD (flat|room|cliff), ROCKY_VIEWER (1|0), ROCKY_AUDIO
# (1|0), ROCKY_LLM_BASE_URL, ROCKY_LLM_MODEL, ROCKY_LLM_API_KEY / ROCKY_LLM_KEY_FILE, ROCKY_WHISPER_URL,
# ROCKY_WHISPER_FAST_URL, ROCKY_VOICE_SECS, ROCKY_VOICE_WAV (test file instead of the mic),
# ROCKY_COCKPIT_PORT (tailnet's proxy target), ROCKY_TAILNET_PORT, ROCKY_FREECAD / ROCKY_FREECADCMD /
# ROCKY_GMSH / ROCKY_CCX (CAD tools; default: on PATH, else the FreeCAD Flatpak).
set -euo pipefail

ROCKY_REPO="${ROCKY_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
PY="$ROCKY_REPO/.venv/bin/python"
# this machine's settings; `set -a` exports them, then the caller's exported environment is put
# back on top, so `ROCKY_WORLD=room ./rocky.sh play` still wins (sim/envfile.py: the same rule)
# `test` runs on the reference setup, as CI does (the repo's conftest.py sets the same default)
if [[ "${1:-}" == test && -z "${ROCKY_ENV_FILE:-}" ]]; then export ROCKY_ENV_FILE=/dev/null; fi
ROCKY_ENV_FILE="${ROCKY_ENV_FILE:-$ROCKY_REPO/rocky.env}"
if [[ -f "$ROCKY_ENV_FILE" ]]; then
  _caller_env="$(export -p)"
  set +u; set -a
  # shellcheck disable=SC1090
  source "$ROCKY_ENV_FILE"
  set +a; set -u
  eval "$_caller_env" 2>/dev/null || true
  unset _caller_env
fi
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
    # world editor, RL panel — on one running sim; `chat` drives it too. The LLM key comes
    # from ROCKY_LLM_API_KEY / ROCKY_LLM_KEY_FILE (sim/envfile.py)
    cd "$ROCKY_REPO"
    for p in $(pgrep -f "^[^ ]*python sim/cockpit.py"); do kill "$p"; done   # one cockpit at a time
    # D052 voice: a fast whisper (the reference setup: base.en, CPU greedy, ~1 s per command) when
    # ROCKY_WHISPER_FAST_URL answers, else ROCKY_WHISPER_URL (default :8082; the reference's
    # large-v3-turbo takes 18-20 s per command on 6 CPU threads)
    if [[ -z "${ROCKY_WHISPER_URL:-}" && -n "${ROCKY_WHISPER_FAST_URL:-}" ]] \
        && curl -s -m 1 -o /dev/null "$ROCKY_WHISPER_FAST_URL/" 2>/dev/null; then
      export ROCKY_WHISPER_URL="$ROCKY_WHISPER_FAST_URL"
    fi
    MUJOCO_GL=egl exec "$PY" sim/cockpit.py "$@" ;;

  cockpit-stop)
    for p in $(pgrep -f "^[^ ]*python sim/cockpit.py"); do kill "$p" && echo "stopped $p"; done ;;

  tailnet)
    # D051: the cockpit stays bound to 127.0.0.1; tailscale serve terminates HTTPS with a
    # tailnet cert and proxies to it. HTTPS matters: the browser only allows the mic
    # (getUserMedia) on a secure origin, so plain http://laptop:8765 has no voice button.
    TS_PORT="${1:-${ROCKY_TAILNET_PORT:-9445}}"
    if [[ "${1:-}" == "off" ]]; then
      tailscale serve --https="${2:-${ROCKY_TAILNET_PORT:-9445}}" off && echo "tailnet: cockpit unpublished"; exit 0
    fi
    if ! tailscale status >/dev/null 2>&1; then
      echo "tailscale is not up on this machine — run:  sudo tailscale up   (then re-run this)"; exit 1
    fi
    tailscale serve --bg --https="$TS_PORT" "http://127.0.0.1:${ROCKY_COCKPIT_PORT:-8765}" || exit 1
    # this machine's MagicDNS name (Self.DNSName); `|| true`: under set -e a failed lookup would exit here
    HOSTDNS=$(tailscale status --json 2>/dev/null | "$PY" -c 'import json,sys; print(json.load(sys.stdin)["Self"]["DNSName"].rstrip("."))' 2>/dev/null || true)
    echo "tailnet: https://${HOSTDNS:-<this-machine>.<your-tailnet>.ts.net}:${TS_PORT}  (tailnet only; run ./rocky.sh cockpit if it is not up)"
    tailscale serve status ;;

  chat)
    cd "$ROCKY_REPO" && exec claude "$@" ;;

  brain)
    # the key is optional (a server without auth ignores it): ROCKY_LLM_API_KEY, else the one
    # in ROCKY_LLM_KEY_FILE (sim/envfile.py reads it)
    cd "$ROCKY_REPO"
    key="$("$PY" sim/envfile.py --llm-key 2>/dev/null || true)"
    ROCKY_LLM_API_KEY="$key" exec "$PY" -m harness.local_brain \
      --base-url "${ROCKY_LLM_BASE_URL:-http://127.0.0.1:8080/v1}" \
      --model "${ROCKY_LLM_MODEL:-qwen3.6-35b-a3b}" "$@" ;;

  brain-install)
    # sim/brain_install.py (conventions in its docstring); the LLM key is for the /v1/models
    # poll after the llama-swap restart. No cd: a relative FILE (e.g. run from ~/Downloads)
    # means the caller's directory — the script itself has no cwd dependence
    exec "$PY" "$ROCKY_REPO/sim/brain_install.py" "$@" ;;

  brain-bench)
    # sim/brain_bench.py: loads models one at a time through llama-swap (bench etiquette).
    # No cd, so a relative --out lands where the caller is, not in the repo root (its own
    # cockpit is started with cwd = the repo by vision_bench.start_cockpit)
    MUJOCO_GL=egl exec "$PY" "$ROCKY_REPO/sim/brain_bench.py" "$@" ;;

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
      text="$(curl -s -m 60 "${ROCKY_WHISPER_URL:-http://127.0.0.1:8082}/v1/audio/transcriptions" \
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
    "$PY" -m pytest sim/tests -m "not slow" -q "$@" ;;   # CI runs these too (and the slow ones)

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
    # train_ppo makes the run dir itself; --env gait (= walk: the residual gait policy)
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

  cad-drawings)
    cd "$ROCKY_REPO/cad" && exec "$PY" gen_drawings.py "$@" ;;

  cad-open)
    if [[ -n "${ROCKY_FREECAD:-}" ]]; then read -r -a fc <<< "$ROCKY_FREECAD"
    elif command -v freecad >/dev/null; then fc=(freecad)
    elif command -v FreeCAD >/dev/null; then fc=(FreeCAD)
    elif flatpak info org.freecad.FreeCAD >/dev/null 2>&1; then fc=(flatpak run org.freecad.FreeCAD)
    else echo "cad-open: no FreeCAD (install it, or set ROCKY_FREECAD in rocky.env)" >&2; exit 1; fi
    files=()
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --fem) shift; w="$ROCKY_REPO/cad/out/fem/work/${1:?--fem needs a part name}"
               [[ -f "$w.frd" ]] || { echo "cad-open: no $w.frd (run: rocky.sh cad-check --fem)" >&2; exit 1; }
               # FreeCAD 1.1 opens a CalculiX result only inside an Analysis: wrap it (cad/fem_to_freecad.py)
               if [[ ! "$w.FCStd" -nt "$w.frd" ]]; then
                 if [[ -n "${ROCKY_FREECADCMD:-}" ]]; then read -r -a fcc <<< "$ROCKY_FREECADCMD"
                 elif command -v FreeCADCmd >/dev/null; then fcc=(FreeCADCmd)
                 elif command -v freecadcmd >/dev/null; then fcc=(freecadcmd)
                 else fcc=(flatpak run --command=FreeCADCmd org.freecad.FreeCAD); fi
                 echo "cad-open: wrapping $(basename "$w").frd in a FreeCAD analysis ..."
                 command rm -f "$w.FCStd.status"
                 FEM_FRD="$w.frd" FEM_FCSTD="$w.FCStd" "${fcc[@]}" "$ROCKY_REPO/cad/fem_to_freecad.py" >/dev/null 2>&1 || true
                 grep -q '^ok' "$w.FCStd.status" 2>/dev/null \
                   || { echo "cad-open: the wrap failed:" >&2; cat "$w.FCStd.status" >&2 2>/dev/null; exit 1; }
               fi
               files+=("$w.FCStd") ;;
        *)     if [[ -f "$1" ]]; then files+=("$(realpath "$1")")
               elif [[ -f "$ROCKY_REPO/cad/out/$1.step" ]]; then files+=("$ROCKY_REPO/cad/out/$1.step")
               else echo "cad-open: no file $1 and no cad/out/$1.step" >&2; exit 1; fi ;;
      esac
      shift
    done
    [[ ${#files[@]} -gt 0 ]] || files=("$ROCKY_REPO/cad/out/leg_skeleton_assembly.step")
    setsid "${fc[@]}" "${files[@]}" >/dev/null 2>&1 < /dev/null &
    echo "opened in FreeCAD: ${files[*]#"$ROCKY_REPO"/}" ;;

  help|-h|--help)
    # the whole leading comment block after the shebang, however long it grows
    awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "${BASH_SOURCE[0]}" ;;

  *)
    echo "unknown command: $cmd (try: rocky.sh help)" >&2; exit 2 ;;
esac
