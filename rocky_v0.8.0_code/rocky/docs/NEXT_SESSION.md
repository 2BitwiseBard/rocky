# Next session (9) — prompt draft

Paste/adapt this to start session 9. Attach the newest repo zips
(rocky_v0.8.0_*) — or your own zip if you changed code on the laptop
(say so if you did!).

---

Session 9 of PROJECT ROCKY (Pebble build). Boot per the project
instructions (status log, zips into one folder, BUILD_LOG top entry,
NOTES_INBOX, this file). v0.8.0 state in one line: CAD is print-clean
(CI 21/21) and paused on my caliper numbers; the sim robot now walks,
gestures, self-rights through the reflex stack (FALLEN→RIGHTED, D042),
patrols with lidar + chord-speak (D043), and can be driven by text
(harness/intent.py), a local LLM, or Claude over MCP.

What I've done since 8d: [fill in — servo order placed? caliper numbers?
laptop runs? whisper.cpp tried? Ollama tried?]

Priorities (adjust as you like):
1. If I have caliper numbers in NOTES_INBOX: file into params.yaml,
   regen affected parts, run cad/run_all_checks.py, refresh print list.
2. If the capped recovery retrain ran on my laptop (RL_TOUR rung 7 /
   D045: `--reward v2 --log-std-max -0.5`): eval it vs recover1's 12/20
   and promote or file the negative. If it didn't run, don't re-run it
   in-container — it needs GPU-scale steps now.
3. If servos arrived: BENCH DAY — bench/BENCH_RUNBOOK.md, real IDs,
   D017 re-baseline plan; wire the D044 fuse note (2× STS3250 stall).
4. Otherwise: pick from DESIGN_BACKLOG (B-items) or the open threads:
   intent.py inside the playground prompt, whisper.cpp voice pipe on
   hardware, rubble-recovery env, SLAM front-end on the room world.

Hard rules I care about: params.yaml is SSOT; checks green before
delivery; sim honesty (D025/D035/D045 — negative results are results);
SCS0009 never sees 12 V; session close per the project instructions.

---

Open items carried out of 8d:
- Servo order NOT yet placed (sheet: docs/pebble_order_sheet_v2.html in
  the project — recommended split 10× ST3215 + 5× STS3250 knees ≈ $585).
- Tyler's caliper numbers still gate the params regen → final prints.
- Capped v2 recovery retrain queued for the laptop GPU (D045).
- Ollama local_brain path and the whisper.cpp → intent.py pipe are
  written but untested on real hardware.
- orphan cad/out/shell_ring_assembled.stl (stale, harmless) still there.
