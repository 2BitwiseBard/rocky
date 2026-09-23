# Vision & world-model plan — how Pebble/Rocky SEES (session 5c)

*Extends SENSING_PLAN.md's 4-layer world model with the semantic/VLM
layer Tyler asked for. Design principle: Rocky-canon perception is
**360° geometric first** — he "sees" the room as shape, all directions at
once, no face, no gaze direction. The lidar IS the eyes; cameras are a
supplementary sense, hidden behind the shell (a gill bay), never styled
as eyes. Chord-speak is the natural output channel: the robot should be
able to TELL you what it senses.*

## The layer stack

| layer | sensor | runs where | status |
|---|---|---|---|
| L0 geometric | 360° lidar (D024) + legged-odom EKF | Pi, onboard | **sim-proven**: EKF + ICP SLAM-lite, map IoU 0.72, yaw 0.06° (D026/D027) |
| L1 occupancy memory | L0 accumulated into a persistent grid + frontier list | Pi, onboard | occupancy demo done; persistence + frontiers = Phase-3 software |
| L2 semantic | camera frames + scan context → VLM → labeled regions ("couch", "doorway", "Tyler") pinned to L1 cells | **home server** (Pebble) | planned — the split below |
| L3 episodic/narrative | event log + semantic map → summaries, narration, "where did I see X" | server (LLM) | later phase |

## The Pebble split: robot = reflexes, server = cortex

Pebble's Pi 5 runs everything real-time (gait, reflexes, EKF, SLAM-lite,
chord-speak) and streams over WiFi to a home server (or cloud API) doing
the heavy semantics:

- uplink: JPEG keyframes (1–2 Hz on demand, not continuous), current scan,
  pose, event stream. Bandwidth trivial; WiFi-RSSI logger already maps
  coverage.
- server: VLM (e.g. an open VLM on a home GPU, or a cloud multimodal API)
  answers structured queries: "label the salient objects in this frame +
  bearing/extent" → results pinned into the L1 grid as semantic annotations.
- downlink: sparse semantic map patches + high-level goals ("kitchen is
  bearing 40°, 3 m"). Latency-tolerant by design: NOTHING safety-critical
  ever depends on the link — reflexes, cliff, safe-stop are all onboard
  (this boundary already exists in the reflex arbitration, SENSING_PLAN).
- graceful degradation: link down → robot keeps full L0/L1 competence
  (map, patrol, avoid, narrate geometry: "wall close behind").

## Big-Rocky onboard path

Full-scale Rocky carries the cortex: budget one NPU-class board (Orin
Nano / RPi AI HAT+ / Hailo-8) for an onboard small VLM + detector, same
L2 interface — the server split is the API CONTRACT so software moves
unchanged. Decide at the Phase-5 gate; nothing before it depends on the
choice.

## Camera hardware (Pebble, Batch-3-adjacent)

- Pi Camera 3 (wide) BEHIND A GILL: bracket inside a shell sector, lens
  looking through a slot aperture — keeps the eyeless canon face
  (backlog **B16**, printable once a camera is in hand; dims VERIFY).
- Coverage: ONE forward camera + rotate-in-place beats a camera ring at
  this scale (the robot yaws freely; the lidar already covers 360°).
- The ReSpeaker mic array (Batch 3) adds sound BEARING — pinned into L1
  like everything else ("noise at 120°" → chord-speak `curious_question`).

## Rocky-canon behaviors this unlocks (the "more than walking" list)

- **explore-and-report**: frontier-driven patrol of L1, then chord-speak
  summary at the human ("two rooms, one blocked doorway, found charging
  dock").
- **fetch/point**: semantic query "where is X" → walk to the L1 cell,
  point with an arm (the gesture library), say `found_it`.
- **guard/anomaly**: L1 diff vs last patrol — new obstacle = `curious`,
  missing object = `confused`, moving contact = `alarm`.
- **follow-me**: person = moving L2 blob + RSSI gradient; keep 1.2 m.
- **dock ritual**: low battery → navigate L1 to the dock (B14), funnel,
  crouch, mate, `sleepy`.

## Talking WITH Rocky (session 5c — Tyler's ask, and it's very buildable)

The hardware is ALREADY ON THE BATCH-3 SHEET: ReSpeaker v2.0 mic array
(4-mic, direction-of-arrival, hardware AEC) + MAX98357A amp + 40 mm
speaker (the chord-speak mouth). The conversation loop:

  human speech -> ReSpeaker (wake word "hey Rocky" via openWakeWord, then
  DOA bearing) -> ASR (whisper.cpp tiny/base ON the Pi — a few hundred ms
  for short utterances; or server whisper for accuracy) -> the HARNESS
  (below) decides -> replies in CHORD-SPEAK (the event engine is already
  the output API: intent -> word) + optionally a gesture. Rocky never
  speaks English — comprehension in, chords out, exactly canon. The
  audition page becomes the household's chord-speak phrasebook.

Human GESTURE recognition (jazz hands back at Rocky, offering a fist):
camera keyframe -> server: either a lightweight pose model (MediaPipe
hands/pose) or simply the VLM with a structured prompt ("is this person
doing jazz hands / offering a fist? one word"). Fist-bump COMPLETION is
already solved onboard — the extended-arm + SEA-switch contact path runs
in sim today; the camera only supplies the invitation.

## The harness: Rocky as an MCP server (the model is swappable)

Formalize the robot as a TOOL SERVER (MCP-style JSON-RPC over WiFi) so
ANY brain — a small local LLM on the home server, a cloud multimodal API,
an onboard NPU model on big Rocky — drives the same contract:

  tools: say(word), gesture(name), goto(x, y | frontier | semantic_label),
  stop(), scan_summary(), map_query(label), look(bearing), patrol(),
  dock(), status() -> pose/battery/temps/events
  resources: live L1 map patch, event log tail, camera keyframe
  guardrails ONBOARD, not in the model: the safe-stop primitive, cliff
  reflex, speed/torque clamps and the SafetyMonitor stay between the
  harness and the servos — a hallucinating model can command nothing the
  reflex layer won't veto (same arbitration boundary as SENSING_PLAN).

This is the same swap-the-backend pattern the repo already uses (driver
mock/serial, ROS hw backends): Pebble = server-hosted brain, big Rocky =
onboard brain, zero interface change.

## Fun & useful task list (beyond walking/gesturing) — roughly ordered

Near (Phase 3, lidar + odometry only): patrol-and-report with chord
summaries; guard mode (map-diff anomalies -> `curious`/`alarm`); find-the-
sunny-spot (BME688 temp gradient); follow-me (RSSI + moving blob); maze/
obstacle-course runs (the watchdog already earns it); "where's my phone"
via BLE RSSI sniffing; dead-reckoning tricks (walk a square blindfolded,
chirp `amaze` on closure error < 2 cm).

Mid (camera + server brain): fetch/point at named objects; deliver small
items in the scoop (I2 tool!); greet known faces with personal chord
motifs; photograph-the-house patrol -> daily digest; plant-minder rounds
(soil probe = another I2 tool); hide-and-seek (people = moving blobs).

Canon-social: the QUESTION game (human speaks, Rocky answers yes/no in
chords); jazz-hands echo (you jazz, Rocky jazzes back); fist bump on
offer; "sing-along" (map music beat -> body bounce — the gesture engine
takes a tempo already); bedtime round (checks doors closed on the map,
`sleepy`, docks).

## Immediate implications for the CAD/plan (do not defer)

1. B16 camera-gill bracket rides the shell regen (slot aperture already
   exists as the gill pattern).
2. Star-board pin 4 (wired-OR microswitch lane) and the I2 JST-SH-3 data
   line stay reserved — tool-tip sensors feed L1 too.
3. Keep the hatch cap swappable (D029 seat): the lidar mount (B12) is a
   cap variant; a future camera-ring cap would be too.
