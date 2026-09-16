# Rocky-MCP contract v0 — implementable spec (session 6 planning pass)

*Refines VISION_PLAN's harness section into a buildable v0. **STATUS: v0
IMPLEMENTED same session (6b) in `harness/`** — FastMCP stdio server, mock +
MuJoCo-sim backends, 8/8 tests through an in-process MCP client including
goto-into-void on real physics. This doc remains the contract; the code is
the proof. Originally targeted: session 7 stands this up against the SIM (same contract hardware serves later).
Transport: MCP over stdio for dev / streamable HTTP on the robot. One rule
above all: the model proposes, the reflex layer disposes — every tool call
crosses the onboard guard before it touches actuators.*

## v0 tool set (sim-first; cut anything camera-dependent)

| tool | args | returns | guard behavior |
|---|---|---|---|
| `say` | `word` ∈ chord-speak lexicon (16) | ok | queue via chordspeak_events (priority NORMAL, model can't preempt safety events) |
| `gesture` | `name` ∈ {jazz_hands, fist_bump, beckon (B18, shipped 6b)} | ok / `busy` | refused while walking or in safe-stop; runs the sim gesture engine verbatim |
| `goto` | `{x, y}` (map frame, m) | streamed progress events, terminal ok / `stopped(reason)` | route runs the cliff detector + stuck watchdog; PLANT→BRACE safe-stop can veto at any tick — the tool RESULT reports what actually happened |
| `stop` | — | ok | always accepted, maps to the safe-stop primitive (D025) |
| `scan_summary` | — | `{n_points, frontiers[], nearest_obstacle_bearing_deg, nearest_obstacle_m}` | read-only |
| `status` | — | `{pose, mode, battery_v*, temps*, last_events[]}` (*mocked in sim) | read-only |

Deferred to v1: `look`, `map_query`, `patrol`, `dock` (needs B14
electrical), camera resources. Do not stub them — absent tool > lying tool.

## Contract invariants (write tests for these, not prose)

1. **Guard supremacy:** a `goto` into the cliff-course void must return
   `stopped(cliff)` and the robot must be at a safe pose — the harness NEVER
   raises an exception past the guard; vetoes are ordinary results. Test =
   the existing cliff scenario driven through the MCP client.
2. **Single writer:** exactly one supervisor owns ctrl; tool calls enqueue
   INTENTS consumed at gait-cycle boundaries (no mid-cycle teleports; the
   D025/D026 phase-alignment lessons live here).
3. **Honest async:** long tools (`goto`) stream progress notifications;
   `stop` during a `goto` resolves the goto with `stopped(user)`, not an
   error. One in-flight motion intent max; a second `goto` preempts politely
   at the next cycle boundary.
4. **No state in the brain:** any model can be killed mid-conversation and
   the robot stays safe (link-down = safe by construction, VISION_PLAN).

## Wake-word / ASR skeleton (scope fence for session 7)

- v0 = laptop mic → openWakeWord (or Porcupine free tier) → whisper.cpp
  small → intent regex ("go to the …", "say …", "jazz hands", "stop") →
  MCP client calls → chord-speak reply WAV played locally.
- Explicitly OUT of v0: on-Pi audio (ReSpeaker not ordered/arrived), LLM
  intent parsing (regex first — measure where it fails before adding a
  model), any TTS that isn't chord-speak (canon: Rocky never speaks words).

## Demo definition-of-done (session 7)

Typed or spoken: "rocky, check what's around you" → harness calls
`scan_summary` → reply chord (`acknowledge` → `found_it`) + a gesture,
all in the MuJoCo sim with the guard layer live. Record it; that video is
the interaction stack's first honest artifact.
