# Driving Pebble before it exists — the interactive sim options

*(Session 6c. Three ways to poke the robot, from most-visual to
most-Claude. All of them run the REAL code — the same WaveGait,
ReflexSupervisor, gestures, and detector that will run on the Pi.)*

## 1. The playground (live window + typed commands) — `sim/playground.py`

On your laptop (needs `pip install -e . mujoco imageio` in the repo):

    cd sim
    MUJOCO_GL=glfw python3 playground.py --viewer

A MuJoCo window opens; the terminal is a REPL:

    pebble> walk 45            # it walks, live
    pebble> set gait.T 2.0     # slow the cycle — PHASE-CONTINUOUS, no hop
    pebble> set gait.hstep 40  # higher steps, immediately
    pebble> push 35 0          # shove it, watch the reflex catch it
    pebble> stop               # PLANT -> BRACE -> planted idle (D034)
    pebble> gesture beckon     # B18, live
    pebble> say found_it       # chord through your speakers
    pebble> record on ... record off   # save an mp4 clip of what you did
    pebble> show               # params + reflex state + pose

`--cliff` loads the table-edge world so you can drive it at the void and
watch the detector + safe-stop fire. `--script "walk 45; wait 3; quit"`
runs the same commands headless (CI-tested; it's how this file's claims
were verified).

Tuning workflow: `set` changes are SIM-ONLY. When a tweak feels right,
write it in NOTES_INBOX — next session bakes it into params.yaml and the
whole tree (CAD sweeps, URDF, MJCF) regenerates against it. That keeps
the SSOT honest while letting you iterate at typing speed.

## 2. Chat-driving over MCP (tests the actual harness)

The harness IS an MCP server, so any MCP client can drive the sim through
the exact interface a future brain will use.

**MCP Inspector** (zero config, browser UI with tool buttons):

    npx @modelcontextprotocol/inspector \
        env ROCKY_BACKEND=sim python3 -m harness.server

**Claude Code / Claude Desktop** — `.mcp.json` in the repo root:

    {"mcpServers": {"rocky": {
        "command": "python3", "args": ["-m", "harness.server"],
        "env": {"ROCKY_BACKEND": "sim"}, "cwd": "<repo>/rocky"}}}

then just talk: "walk to (0.3, 0) and tell me what stopped you" → Claude
calls `goto`, the sim runs real physics, and the answer comes back
`stopped: "cliff"` if you aimed at the void. `ROCKY_BACKEND=mock` gives
instant responses (contract behavior, no physics) for UI/flow testing.

Note the two front ends are complementary: the playground has eyes but
bypasses MCP; the harness is the real contract but v0 has no window.
Bolting the passive viewer onto SimBackend is a small session-7 item if
chat-driving-with-visuals turns out to be the thing you want most.

## 3. What the sim is honest about (and what it is not)

Trust it for **logic and geometry**: reflex arbitration, gait/sweep
limits vs the CAD-validated ±40°, gesture reachability (D035 joint-space
poses), interface contracts, detector behavior, "does this parameter
change break anything." Trends survive; that's why RL trains under
domain randomization.

Treat as **directional only**: absolute push envelopes, friction-
dependent anything, servo tracking under load (position actuator with
guessed kp/kv), thermal, bus latency (~none modeled), battery sag.

The calibration path when hardware lands (bench day, already scripted in
`bench/`): servo step-response + stall on the jig → fit kp/kv/torque
clamps; real part masses on a kitchen scale → MJCF update; then re-run
the sim baselines (D017's tables) and see which numbers moved. The gap
you measure THERE is the number that tells you how much to trust
everything else. Until then: the sim is a superb wrong-answer detector
and a decent right-answer suggester.
