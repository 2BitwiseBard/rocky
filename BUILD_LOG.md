# ROCKY Build Log — Engineering Notebook

*Convention: newest entries first. Every work session gets an entry: date,
what happened, what was decided, what broke, what's next. Claude writes these
at the end of each session; Tyler adds field entries any time (or dumps raw
notes in `NOTES_INBOX.md` for Claude to file). Keep entries honest — failed
prints and dumb bugs are the most valuable lines in this file.*

---

## 2026-09-23 · Session 9c (laptop) — the cockpit: a browser playground on one running sim (D049)

Asked for: an in-app guide, brain/mode toggles, model switching, a chat
tab, collapsible panels, environmental conditions, obstacles, rough
terrain, and a way for the model to "see".

- `sim/cockpit.py` + `sim/cockpit_ui.html` (`./rocky.sh cockpit`,
  http://127.0.0.1:8765): one sim thread (Playground loop + the harness
  goto controller and cliff detector, real-time paced, speed 0.25–4×,
  pause), chase camera + torso eye camera as MJPEG, SSE state feed, a job
  queue so nothing else touches MjData. Panels: brain & chat (Talk / Local
  model / Claude, model picker, vision-model picker, tool trace), teleop
  (buttons + keys), gestures/chord-speak, shove buttons, orbit camera,
  world editor, RL, gait tuning, events, help; every panel collapses, the
  sidebar toggles.
- `sim/world_builder.py`: worlds from a JSON spec at runtime — presets
  (flat, room, cliff, obstacle course, rubble field, rough terrain,
  stairs, slope 8°, icy floor), objects (box, wall, ramp, stairs, rubble,
  pushable ball), floor friction, gravity tilt, a smoothed-noise
  heightfield; added geoms are lidar group 3; the robot gets an `eye`
  camera. The sim swaps models in place and respawns the robot upright.
- Vision: `look` = eye frame → local vision model (`vision-model` alias,
  lfm2.5-vl or gemma) → text. The local brain gets it as a tool; the MCP
  server exposes it where the backend has an eye.
- One sim for every driver: `harness/cockpit_backend.py` proxies the MCP
  contract to a running cockpit; `ROCKY_BACKEND=auto` (now in
  `.mcp.json.example` and my `.mcp.json`) picks it when it answers, else
  the in-process sim. So `./rocky.sh chat` drives what the browser shows.
- Fix after the first look: world objects are geom group 3 (the lidar
  convention) and MuJoCo renderers and viewers HIDE group 3 by default, so
  every preset built but drew nothing ("presets don't show anything").
  The cockpit renderers and both viewer windows now enable group 3. Also:
  a top-down map (objects, lidar hits, robot, goto target, click-to-goto),
  named camera views (follow/wide/top/low, default behind the robot), the
  HUD marker and goto flag drawn into the chase stream, the eye raised
  above the shoulder blocks, and respawn via mj_resetData (a plain qpos=0
  had dropped the pushable ball onto the origin with a zero quaternion).
- Verified: talk chat, local brain (qwen3.6-35b-a3b → say + look; the
  vision model read the obstacle course correctly enough), world swaps and
  edits, righter hot-swap (kept across reset/world change — a bug found
  and fixed), speed/pause, a 40 N shove with the full FALLEN → NORMAL
  cycle inside the cockpit, reset, the MCP proxy. 7 sim tests (world
  builder + shove). Docs: SIM_GUIDE §3b, README, RL_GUIDE, D049, B31.

## 2026-09-23 · Session 9b (laptop) — "the push flies and jitters": the shove model, two jitter sources, a righter retrain (D048)

**Report:** the push and the reflex correction in the playground / README
GIF "fly and jitter". Both true, neither a physics bug.

- **Flying = the push model.** Every push was a rectangular force pulse at
  the centre of mass. The demo's 120 N × 0.25 s was a 30 N·s strike
  (4.6 bodyweight-seconds): measured 9–11 m/s peak, 11–15 m of travel, with
  the camera tracking the torso. The model also has no gentle regime:
  below the ~31 N foot-friction limit the standing robot is a rigid block,
  above it the feet let go and it cartwheels (60 N: 2 m/s, 0.6 m, lands on
  its back). New `sim/shove.py`: half-sine over 0.4 s at the carapace's
  top rim (force + torque about the CoM), reported in N·s and BW.
  `shove_envelope.py`: standing survives 25 N peak, tips at 30 N (~1 BW,
  6–8 N·s); walking 20–30 N by direction. Demo shove 40 N (1.5 BW,
  10 N·s): fells 5/5 seeds, ends ~1 m away. 45 N felled only 3/5 — two
  rolled through and landed upright.
- **Jitter 1 = the righter.** `audit_righter.py` on `recover1`: 73–75 % of
  per-tick joint moves pinned at the 5 rad/s limit, ~10 direction
  reversals/s per joint, 4.5°/tick, 6.7° tracking error. A 50 Hz staircase
  of 5.7° jumps — the D045 bang-bang failure, deployed. And under the rim
  shove it never righted by policy (0/5): every stand in the demo came from
  the 10 s deadline ramp, which the old strike's random landings had hidden.
- **Jitter 2 = BRACE while airborne.** The tilt-vector seek flips its goal
  with the sign of ω×p every physics step; with zero contacts that is 58
  reversals in 0.4 s and legs thrashing mid-cartwheel. Now: no contacts →
  hold the foot targets (test added). The playground also never passed tilt
  to the supervisor, so after a flip it lay on its back cycling
  BRACE→RECOVER→NORMAL in a standing pose; it now feeds tilt/height and
  installs the righter (torch + checkpoint, optional), and the supervisor
  resets the righter on every FALLEN entry.
- **Ruled out:** servo stiffness (0.3° tracking standing, <7° walking),
  target smoothness on the ground (0 reversals in BRACE), loop pacing
  (5–9× real time headless). The viewer loop now resyncs instead of
  fast-forwarding after a >50 ms stall (unverified as a cause — the viewer
  probe hangs under Wayland from a script).
- **Righter retrain.** `RecoverEnv` reward `v3` = v2 + `-0.3·mean((Δtarget
  /rate_limit)²)`; `--rate-limit` per checkpoint (3 rad/s for v3), read back
  by `PolicyRighter`/`eval_recover` so a policy always replays its training
  dynamics. Two runs: `recover5_v3` (scratch, 3 M, 8 envs) and
  `recover5_v3_warm` (from recover1, +3 M, 4 envs), both `--log-std-max
  -0.5`, CPU, ~3.1k and ~1.7k sps. **Both negative on the handoff**: scratch 2/20, warm 4/20 vs recover1's 7/20 (re-measured today on the current model with the old and the new evaluator — the 10/20 in yesterday's table predates the D047 `pebble.xml` regen). Warm is the smoothest righter yet (58 % pinned, 6.5 reversals/s, 2.1°/tick vs 73 %, 9.8/s, 4.4°) — the trade exists, not yet won at this budget; recover1 stays shipped (B30 lists the next moves). What actually rights the robot from its BACK — where the rim shove lands it every time — is the planted-stance ramp, 5/5, not any policy (0/5 by handoff); so the supervisor now ramps after 3 s without a 10° improvement in tilt (stall rule, `right_reason` = handoff/stall/deadline, 2 tests) and the demo's righting takes ~5–6 s instead of the 10 s deadline. README GIF regenerated from `run_reflex_fallen.py --video` (seed 0: shove at 3.0 s, FALLEN 4.4, RIGHTED 9.2 by stall, walks 0.54 m).
- **Teleop moved to the terminal.** WASD in the viewer window "changed the
  lighting": the MuJoCo viewer binds every letter to a render toggle (W
  wireframe, S shadows, A auto-connect, D static bodies, G fog, Q camera,
  E equality) and SPACE to pause, and those fire alongside `key_callback`.
  Now: arrows / Shift+WASD / Shift+QE / SPACE / Shift+G as single keys at
  an empty `pebble>` prompt (raw-tty reader with line editing), arrows only
  in the window. Plus `help` / `help rl`, `rl` (checkpoint table from the
  new `sim/rl_dashboard.py`, which also draws `out/rl_curves.png`),
  `righter NAME|off` hot-swap, `set reflex.stall_s|fallen_max_s`, and a
  HUD in the viewer's user scene (state-coloured marker + command arrow).
- Drove it over MCP from this session: `gesture wave` (5.4 s, rendered in
  physics), `goto 0.3 0` → arrived at x = 0.274.
- Tests: 89 fast + 3 sim (`sim/tests/test_shove.py`, in CI after the MJCF
  build) + the airborne-hold reflex test. Docs: SIM_GUIDE (what a shove is,
  push command), RL_GUIDE (v3, the audit), D048, B29/B30.

## 2026-09-22 · Session 9 (laptop) — full review, then the leg is rebuilt around the real servo (D047); reprint plan + shopping list; repo goes public-ready

**Done**
- **Review** (`docs/REVIEW_2026-09-22.md`): three parallel passes over the
  D046 tree — mechanical, electronics/BOM, software/repo — with every
  finding dispositioned. Headline: `servo_st3215.py` v0.1 was a guessed box
  wrong in five independent ways, so every leg part Tyler printed (photo in
  `media/`) was designed for a servo that does not exist; the D046 yaw
  stage would have failed structurally (~240 N through a 683ZZ 15 mm from
  the horn). Electronics: all-ST3215, ESP32 driver board is core, per-leg
  12 V fan-out, LiPo alarm. Software: 85 fast tests green, URDF parity
  had drifted since D046.
- **Servo model v2 from a measured STEP** (`cad/ref/STS3215_03a.step`,
  SO-ARM100): case 28.8 flat-to-flat with 1.5 mm rims carrying the screw
  holes, 2.6 mm plateau, axis 10.2 from the front, horn top 33.1, 45° hole
  pattern, Ø19.9 rear idler, connector housing + pins, plug keep-out. The
  model's self-check now diffs it against the STEP (43 mm³ unexplained).
- **D047 leg chain**: yaw servo shaft-down in a cup on the I1 base; one
  C-fork riding horn + idler; twin-plate femur (plate A on couplers, plate
  B on idlers, bridged, 4× M3); knee cup on the tibia; `servo_cup` = one
  retention shape for all three servos (four rim self-tappers); coupler
  re-cut (lobes 0/90/180/270, slotted M3/M2 holes); blank v0.3 + glue-on
  idler; four coupons that are clips of the production solids. Hip axis
  61 = `params.leg.hip_axis_z`, read by gait, MJCF and URDF (three hard-
  coded 58s gone). **CAD CI 23/23 tree clean**, joint suite clean with four
  new check classes, mass 2665 g, sim walk unchanged, URDF parity OK.
- **Docs**: `docs/PRINT_PLAN_2026-09-22.md` (batch 1 coupons → batch 2 one
  leg → batch 3 body), `bom/SHOPPING_LIST_2026-09-22.md` (bench kit ≈
  $300–370, full robot adds ≈ $900–1,050), D047 row, B22–B28, print pack
  v0.5, viewer rebuilt.
- **Repo**: everything committed to git (drop/patch workflow retired),
  MIT LICENSE, `.mcp.json` untracked, duplicates and `NEXT_SESSION.md`
  removed, jpg/pdf on LFS.

**Broke / caught**
- Printability's first pass on D047 caught three real things: plate A's
  hub rim was 0.3 mm outside the coupler pocket (Ø26 → Ø30), the blank's
  rims-down pose had a 10.8 mm overhang (bottom filled flat), the idler
  puck's centre nub was a 6.6 mm overhang (omitted).
- OCC booleans return garbage on coincident faces: the STEP comparison had
  to run against a 0.02 mm-inflated model.
- The knee cup's two horn-face rim screws sit under plate A's beam: the
  suite now models them as driven before the femur goes on (B22).
- `generate_urdf.py` needed xacro (not installed here) and hard-coded
  session-2 masses; it now reads `mass_budget.json` and expands its own
  macro.

**Same evening — repo flattened, CI, guides, workflows verified**
- The code tree IS the repo root now (`cad/out` a real git-lfs directory,
  `rocky.sh` at the root defaulting to its own directory, `patches/` and the
  drop directories gone, `setup.py` → `pyproject.toml` with `sim`/`rl`/
  `harness`/`hw`/`cad`/`dev` extras). GitHub Actions `ci.yml`: fast suites
  + sim smoke + URDF parity + the slow harness test on every push, the CAD
  tree on pushes to main. Rehearsed in a fresh venv installed from
  pyproject alone: 85 passed, walk 264 mm, PARITY OK, slow test passed.
- `docs/SIM_GUIDE.md` and `docs/RL_GUIDE.md`: how each stack works,
  controls, step-by-step runs, the honest results table; README rewritten
  around a status table and two demo GIFs (`media/pebble_fallen_recover.gif`).
- Verified end to end: playground headless script (walk, strafe, stop,
  wave, live-tune, push — clean exit, no NaNs), `run_gestures2` 7/7,
  `run_patrol` 5/5 waypoints, `run_cliff_safestop` PASS; PPO smoke on both
  envs (2 × 96 × 10 updates, ~700–1100 sps on CPU), `eval_ppo --compare-zero`
  (`robust_fwd2`: 325 mm vs the bare gait's 366 — the D031 result stands),
  `eval_recover` on a smoke checkpoint.
- Fixed: `train_ppo.py --resume` with nothing left to train crashed with
  `UnboundLocalError: update` (NOTES_INBOX 09-08); it now explains that
  `--total-steps` is cumulative and exits cleanly. New launcher commands
  `train-walk` / `eval-walk`.
- Retired `docs/SESSION_WORKFLOW.md` (cloud-drop process) and the zip
  instructions in `LAPTOP_SETUP.md`.

**Decisions**: D047.

**Next**
- Fit ladder numbers → `params.print` (B28) — still the gate for every fit.
- Order the bench kit (shopping list part A); one servo answers B23.
- Print batch 1 (four coupons + blank), file fits; then batch 2 (one leg).
- Repo pass 3: move the one-off `run_*_v2` / `diag_*` experiment scripts
  out of `sim/` into `experiments/`; a hardware backend for the harness.

---

## 2026-09-01 · Session 8d — the robot gets back up on its own: FALLEN reflex branch + recovery retrain, talk-to-Pebble intent layer, live lidar + patrol, torque re-audit, servo order v2

**Done**
- **Recovery reward v2 (D041) — and its honest ending (D045).** Success
  is now literally the handoff criterion (tilt<25°, h>0.09, held 0.5 s)
  + a feet-down term while upright; constants shared with eval + reflex.
  Retrained recover1 2M → 3.67M (v2, DR on; stopped the anneal at 3.16M
  and resumed with ent_coef 0 when entropy kept exploding). Episodes
  terminated with success in training — v1 never did once — **but the
  deterministic policy REGRESSED: 2/20 stood-after-handoff vs recover1's
  12/20 (re-verified, same seeds)**. Cause found: clip+rate-limit makes
  wide actions a bang-bang strategy; sigma inflated 13.8→22.1 nats even
  with the entropy bonus OFF, and the mean decayed. Fix shipped:
  `--log-std-max` sigma cap in train_ppo; the capped v2 retrain is the
  queued laptop overnight. recover1 stays the shipped righter.
- **ReflexSupervisor FALLEN + RIGHTED (D042)** — tilt>60° held 1 s hands
  the joints to a pluggable righter (the recovery policy); handoff
  criterion → smoothstep ramp to planted stance → NORMAL. Supervisor
  stays torch-free (Pi-ready). 6 state-machine tests green.
  **`sim/run_reflex_fallen.py`: 120 N shove → brace loses → tumble →
  policy rights it → walks away, 5/5 seeds** (one via the 10 s deadline
  ramp — that fallback earns its keep). Video: `pebble_fallen_recover.mp4`.
- **`harness/intent.py` (talk-to-Pebble)** — deterministic regex/keyword
  text → the six tools; REPL, `--once`, `--stdin` (the whisper.cpp voice
  pipe is documented in the docstring). 14 tests; "look around" the
  gesture vs "what's around" the scan disambiguated; guard supremacy
  test included (parser walks the mock at the void → `stopped:"cliff"`).
- **Lidar live in the harness (D043)** — `scan_summary` now fires the
  sim_lidar ray fan in ANY world (8 map-frame sectors, nearest,
  frontiers) with its blind spot documented in the result; new
  `ROCKY_WORLD=room` (walls/pillars/crate). **`sim/run_patrol.py`: 5/5
  waypoint patrol on the tool surface alone**, chord-speak per stop,
  JSON report; scans match room geometry exactly.
- **`sim/torque_audit.py` (D044)** — static stall margins on the D039
  masses (2840 g): walking 12–16% ST3215, and the ONLY warm joint is
  the recovery knee push, 50% ST3215 / 30% STS3250. Self-righting now
  sizes the servos, not manipulation.
- **Servo order sheet v2** (`docs/pebble_order_sheet_v2.html`, in the
  project) — prices re-verified 09-01 (Seeed ST3215 $23.99/$22.99@10+
  US stock; SCS0009 $9/$8@10+; Waveshare adapter $4.99; STS3250 ~$50–60
  eBay/AliExpress, variant trap flagged). Recommended: 10× ST3215 +
  5× STS3250 knees + spare ≈ $585; budget all-ST3215 $435. Playwright-
  verified (D007). Per Tyler: prefer torque/headroom over cheapest;
  everything stays in the printed ST3215 case.

**Broke / caught**
- v1 recovery success never fired in training (ep_len pinned at 300 for
  2M steps) — found reading the training log, not the reward code. The
  policy propped on 1.5 feet because feet paid nothing (D041's why).
- intent unit-regex `(cm|m|mm)` matched "m" inside "mm" → 100 mm parsed
  as 100 m. Alternation order matters: `(cm|mm|m)`.
- First intent executor test walked the mock robot into the mock void
  from (0.10, 0.05) — the guard line is x≈0.17. The test now starts at
  −0.30; the guard winning WAS the correct behavior.
- The sharpening run itself died silently at 3.67M (no traceback, no
  OOM — likely reaped between container turns); checkpoint intact, and
  by then the answer was already clear (sigma still rising).

**Decisions**: D041 (v2 reward = handoff criterion), D042 (FALLEN/RIGHTED
states, pluggable righter), D043 (scan_summary everywhere + honesty note,
room world), D044 (torque re-audit: self-righting is the sizing driver;
servo split recommendation), D045 (recover1 stays the righter; sigma-
inflation diagnosis; --log-std-max fix; capped retrain queued).

**Next**
- **The capped retrain (laptop overnight):** `train_ppo.py --env recover
  --reward v2 --log-std-max -0.5 --num-envs 8` from scratch or from
  recover1 — beat 12/20 or file another negative.
- Rewrite eval_recover's video note for recover2? No — recover1 is the
  shipped righter; leave videos pointing at it.
- Order the servos (sheet v2 — the STS3250 lead time gates Batch 2).
- Wire intent.py into the playground prompt; try the whisper.cpp pipe on
  the laptop; first real Ollama run of local_brain.
- Caliper numbers → NOTES_INBOX → params regen (still the print gate).


## 2026-08-31 · Session 8c (next day) — Tyler's laptop takes over: setup guide, keyboard teleop, local-LLM brain, flat chat-driving world, RL ground-up tour

**Done**
- **`docs/LAPTOP_SETUP.md`** — the whole stack on Tyler's Linux+CUDA
  laptop (venv, GPU torch, live viewer, chat-driving via `.mcp.json`,
  training, sync rules with sessions) + a Windows/WSL2 section for the
  printer laptop, + a what-runs-where cheat sheet.
- **Keyboard teleop in the playground** (`--viewer`): W/S/A/D nudge
  velocity, Q/E turn, SPACE = D034 safe-stop, G = wave — press-only key
  events, so taps accumulate; tested via the on_key path + headless
  script (no NaNs).
- **`harness/local_brain.py`** — an Ollama model drives sim-Pebble
  through the SAME six tools the MCP server exposes: type to it, it
  calls tools, physics executes, guards stay supreme. `--mock-llm`
  (scripted brain) tested end-to-end in-container: say → wave →
  scan/status → goto → `stopped:"arrived"`. The Ollama path is written
  to the current tool-calling API but UNTESTED here (no daemon) — the
  first laptop run will tell.
- **`ROCKY_WORLD=flat`** on SimBackend — an open floor for chat-driving
  (the default cliff island vetoes every long goto: great for guard
  tests, lousy living room). Harness tests still 7/7.
- **`docs/RL_TOUR.md`** — RL from the policy-gradient theorem up through
  every piece of train_ppo.py, then an 8-rung experiment ladder
  (eval → curves → reward knob → the D031 exploration lesson → fair-DR
  comparison → sigma-anneal → recovery success-criterion fix → new env)
  with a matched reading list.

**Broke / found**
- My own local_brain first described goto in MILLIMETERS — the contract
  is METERS (harness/server.py docstring). goto(-150, 50) walked 20 s
  toward a target 150 m away and "timed out"; in meters it ARRIVES
  within the 25 mm ball. Units in tool descriptions are load-bearing.
- Chat-driving the cliff world twice in one conversation can end
  `stopped:"FELL"` (a second goto from a BRACE pose at the edge) —
  noted, not chased; the flat world is the driving surface anyway.

**Next**
- Tyler: LAPTOP_SETUP top to bottom; first Ollama run of local_brain
  (report which model behaves); RL_TOUR rungs 1–4.
- Session 9 unchanged (calipers → params regen; recovery → FALLEN
  branch) + fold Tyler's laptop findings into NOTES_INBOX.

---

## 2026-08-30 · Session 8b (same day) — print-physics audit (D038), CAD-derived sim masses (D039), gesture library v2 (D040), RL: gait run 2 → 3 M steps, command-sampled run 3, self-righting env + first training

**Done**
- **`cad/check_printability.py`** — every printable STL sliced in its
  prescribed print pose: islands (bed/part), floating-above-bed, thin
  walls (per-blob, persistence rule), overhang steps (inscribed-radius
  width), bed fit; support policy per part; CI post-stage. **21/21 with
  it live, PLATE CLEAN.** Regression-tested on the pre-fix STLs.
- Fixes it forced: `coxa_fork` v0.2.2 (flat floor; horn-screw access —
  the fork was UNBOLTABLE since v0.1 — counterbores + 45° pattern; new
  in-module sweep/servo/driver-access checks), `tibia_sea_outer` ribs,
  `shell_sector` interior rebuilt (sealed chambers → one connected
  cavity with 45° ceilings, 41 → 24 cm³; LED groove 1.4; feed notch and
  gills moved off the seam), `dock_base` ramp trimmed, `jig_column` +2
  gussets, plan orientation fixes (servo_blank horn-up, gauge upright,
  coxa_yaw_base plate support, stand_crown skirt support).
- **`sim/mass_audit.py` → `mass_budget.json` → `build_mjcf.py`.** D017
  re-baselined: walk identical; rubble identical (25/30 ok, 35/40 1-of-3
  stuck, same seeds); push −13 % in N, unchanged in bodyweights.
- **Gesture library v2** (7 gestures, joint-space, stability-asserted,
  physics-verified 7/7, videos rendered) registered in playground +
  harness (`gesture` lists 10). Harness 7/7, driver 58/58 still green.
- **RL:** run 2 resumed 1.66 → 3.0 M steps, return 176 → **228.6** (bar
  345). Run 3 (`runs/cmd_sample3`, command-sampled, warm-started): 136 →
  221 by 4.2 M and climbing. **`sim/rocky_recover_env.py`** (self-righting:
  absolute targets, rate-limited, dense uprightness + standing bonus; the
  hold/random baselines score 0/3 successes) + `train_ppo.py --env
  recover`; run 1 (`runs/recover1`) trained to 2.0 M steps in-session.
  **Closing eval (20 random falls, deterministic policy + the hybrid
  handoff in `eval_recover.py` — RL rights the body, the analytic planted
  pose finishes): 12/20 STOOD vs 2/20 hold-pose / 1/20 random; best-tilt
  median 1.4°; side 5/7, tumble 5/7, back 2/6 on this seed set. Pure-RL
  "stand on ≥4 feet for 1 s" is still 0/20 — the policy rights and props
  but doesn't discover foot placement; the handoff is the architecture
  anyway (wire it as ReflexSupervisor's FALLEN branch). Next lever:
  train with the handoff criterion in the reward.** Video delivered
  (`pebble_recover.mp4`).
- `docs/PRINT_NIGHT_s8.html` updated with every audit-driven note.
- **Run 3 closed at 7.0 M steps, return 224 under full DR (bar 345).
  Honest deterministic check (`eval_ppo.py --compare-zero`, nominal
  forward, no DR): policy 298 / 294 mm / tilt 7.1° vs bare gait 365 /
  366 mm / 0.7° — the residual policy still LOSES to the analytic gait
  on the clean task. The gap is real, not noise; sigma-anneal or a
  DR-matched comparison is the next experiment, and the D031 lesson
  stands: the wave gait is a strong controller.**

**Broke / found**
- Thin-wall false positives are a real class: bore break-outs on curved
  faces (0.2 mm²) and 45° plane-meets-cone feathers (≤1 mm tall). The
  gate now needs a blob ≥2 mm² that persists across two 1 mm samples.
- The accumulated-union "material below" test blew up quadratically on
  the stand section (near-identical layer outlines) — replaced by three
  downward rays per island.
- A 22° body yaw on planted feet needs ~45° of coxa (R185 feet vs R100
  coxa) — the first look-around blew the ±40 limit. 16° it is.
- Random-policy return (287) > early PPO (225) on the recovery env:
  expected with dense shaping; watch whether successes appear.

**Decisions** — D038 (printability CI), D039 (CAD masses; quote push in
BW), D040 (gesture v2 + physics-verified metrics).

**Next** — file Tyler's caliper numbers; eval run 3 + recover1 checkpoints
(`eval_ppo.py`, video); if recovery succeeds in sim, wire it as the
`fallen` reflex path in ReflexSupervisor; carapace: regenerate/retire the
orphan `shell_ring_assembled.stl`; measure real masses on bench day.

---

## 2026-08-30 · Session 8 — tree-wide floating-parts audit: SIX more parts were in pieces; D036 gate rolled into export(); print night re-planned

**Field report (Tyler):** printed P1, P2 and hand_cam + 3 fingers; skipped the
hub (the floating lugs). Ladder not measured yet. All clean so far.

**Done**
- **Mesh-level audit of every STL in cad/out** (connected components,
  watertightness, degenerate flecks, bed fit) + the D036 gate now lives in
  `common.export()` (raises on ≠1 solid; `multi=True` only for assemblies).
  CI ran with the gate live: **20/20**. The hand hub was NOT a one-off —
  six more printables were disconnected bodies, every one "CLEAN" by its
  own interference checks (D037):
  - `tool_scoop` — THREE bodies (bowl / neck / bayonet socket, 5 mm gaps
    each way). Neck v0.2 rooted into the cap, trimmed 1 mm into the floor.
  - `coxa_fork` — the −X high-collar wall attached to nothing (starts at
    z 51 above the crown-arm sweep; 0.3 mm short of the back wall). End
    walls now run through the back wall. ON THE LEG PLATE.
  - `shell_sector` — both web feet (latch pad + magnet pocket) floating
    13 mm inboard of the skirt. Flange feet reach 1 mm into the wall.
  - `stand_section` — skirt hugs the LOWER tube (APO+FIT) so it never
    touched its own tube. 3 mm seat ring z 0..3 (also the male-bar stop).
  - `battery_sled` — finger scallop bottomed EXACTLY at z 0: floor cut
    through, tail lip severed (0.000 mm gap, two solids). Raised; 1.8 mm
    floor kept.
  - `jig_column` — deck-proxy plate placed off the leg-port footprint, 6 mm
    short of the spine. Now spans the column.
  - `fit_ladder` — row-D index dots stayed at y −17 when the slots dropped
    to −29: engraved INSIDE the slots (invisible), two tangent to slot
    walls → non-manifold seam. Moved beside the slots. Tyler's printed
    ladder is still valid (slots are 3.8/4.2/4.6 left→right).
- Every affected module's own checks still pass (fork sweep vs base,
  shell keep-outs + seam pair, stand slide/registration, tool retention,
  jig port, ladder layout audit). Viewer regenerated + Playwright-verified.
- **`docs/PRINT_NIGHT_s8.html`** (Playwright-verified, delivered): the
  "what's left" plan — plate A (hub v0.2.2 + slider + tools, ≈31 g),
  B (leg + blanks, ≈119 g, after calipers), C (deck, after port dance),
  D (stand base+crown), bonus sectors; ≈400 g / ≈19.5 h remaining. Carries
  the changed-STL table and the caliper GO/NO-GO block up front.
- `print_estimate.json` refreshed for the new geometry.

**Broke / found**
- `shell_ring_assembled.stl` is an ORPHAN — no generator in the tree writes
  it; the viewer's Carapace mode still embeds the stale file. Regenerate or
  retire it next time part_shell changes.
- Zero-volume tessellation flecks on hand_finger / tool_hook / foot_pad_tpu
  tips (slicers discard them). Noted, not chased.
- Pattern behind three of the bugs: a feature sized to ANOTHER part's
  envelope (+FIT) instead of its own → exactly one clearance short of
  touching. Worth a grep whenever a skirt/collar/wall is placed by FIT.

**Decisions**
- D037: single-solid gate in `common.export()`; `multi=True` explicit.

**Next**
- Tyler: calipers on the ladder → NOTES_INBOX → plate A tonight → B.
- Session 9: file the ladder numbers into params (`print.clearance_fit`
  etc.) and regen the tree; regenerate/retire `shell_ring_assembled`.

---

## 2026-08-30 · Session 7 — the FLOATING LUGS: hand hub v0.2.2 (knuckle collar), connectivity checks land tree-wide precedent

**Done**
- **Tyler's field report confirmed and root-caused:** `hand_hub.stl` was SEVEN
  bodies — the hub plus all six hinge lugs floating 4.3 mm above the hub top
  (three 120° pairs, z 22.3–29.3 vs hub top z=18; ~146 mm³ each). The v0.2
  KNUCKLE_Z raise (+4.2, the cam-clash fix) moved the lugs up but never
  re-attached them — and v0.1's lugs were already 0.1 mm proud: they were
  NEVER attached. Two releases shipped this way.
- **Why every check passed anyway:** interference checks detect OVERLAP; a
  disconnection is the absence of overlap. "CLEAN" was the bug. (D036.)
- **Hand hub v0.2.2 — knuckle collar:** annular wall (r 19.45–22, 0.45 mm
  running gap to the cam rim) from hub top to lug top; lugs widened to reach
  it; swept tab-envelope windows cut at each station (the finger tab's upper
  corner arcs to r≈20.8 mid-open — hand calc caught it before the boolean
  did; same inverse-pose sweep trick as the finger scallops). Pin bores
  lengthened through the collar: pins insert from OUTSIDE and the far wall
  retains them (they can no longer walk out). Load path lug → collar → hub
  is a proper shoulder (D020). Hub +4.5 cm³ (~+5 g).
- **Checks:** hub/cam/finger each = 1 solid (new assertion), collar root
  section 332 mm³, cam×hub 0.00, worst finger clash over the 14-pose dense
  sweep 0.23 mm³ @ 55° (≤0.5 bar). `part_hand.py` now EXITS NONZERO on any
  failure — CI enforces, not just prints. Tree CI **20/20**. Viewer
  regenerated + Playwright-verified (hand modes show the collar).
- Print queue item 6 updated: re-slice v0.2.2, never print a stale hub STL.

**Broke / found**
- `hand_finger.stl` carries two zero-volume tessellation flecks at the tip
  cap (z 75.5 / 82.5, sub-0.25 mm, no volume) — slicers discard them;
  cosmetic export artifact, not chased this session.
- `test_viewer.py` writes the "Hand open/closed" screenshot to
  `out/viewer_hand_open/closed.png` (slash in the slug makes a directory).
  Harmless; left as-is, noted here.

**Decisions**
- D036: single-solid connectivity audit + attachment probe mandatory for
  every printable module; module __main__ must exit nonzero on failure.

**Next**
- Roll the D036 connectivity assertion into the other 19 part modules
  (part_hand is the only one carrying it so far).
- Print night: hub v0.2.2 supersedes any sliced v0.2.1 plate.

---

## 2026-08-07 · Session 6c (same day) — the PLAYGROUND: interactive sim, live tuning, chat-driving over MCP

**Done**
- **`sim/playground.py`** — drive Pebble before it exists: native MuJoCo
  viewer + command REPL running the REAL gait/reflex/gesture/detector
  code. walk / stop (D034 safe-stop) / gesture / say (chords through the
  laptop speakers) / `set gait.T|h|R0|duty|hstep, reflex.trip` LIVE with
  phase-continuous clock rescaling / push (shove it, watch the reflex) /
  record (mp4 clips) / `--cliff` world / `--script` headless mode.
  End-to-end headless test green (param change mid-walk, beckon, push,
  clip written, zero NaNs).
- **`docs/PLAYGROUND.md`** — the three ways to poke the robot (playground,
  MCP Inspector, Claude-with-.mcp.json — `.mcp.json.example` added) and
  the honesty box: what the uncalibrated sim is good for (logic,
  geometry, trends) vs directional-only (absolute forces, friction,
  servo tracking), plus the bench-day calibration path that closes the
  gap (step response + stall -> kp/kv; kitchen-scale masses -> MJCF;
  re-run the D017 baselines and measure what moved).
- Tuning workflow rule: playground `set` is SIM-ONLY; keepers go to
  NOTES_INBOX -> params.yaml -> full regen. The SSOT stays the SSOT.

**Next** — unchanged; optional 7-item: passive viewer bolted onto
SimBackend so chat-driving has eyes too.

---

## 2026-08-07 · Session 6b (same day) — extension pass: MCP harness v0 LIVE, cliff safe-stop wired + measured, BECKON shipped, viewer weekend mode, parallel CI

**Done**
- **Rocky-MCP harness v0 IMPLEMENTED** (`harness/`): FastMCP stdio server,
  6 tools per `docs/MCP_CONTRACT_v0.md`, MockBackend (behavioral contract)
  + SimBackend (real MuJoCo cliff world + real detector + real safe-stop).
  **8/8 tests green through an actual in-process MCP client session** —
  including the marquee: `goto` into the void over MCP returns
  `stopped:"cliff"` with the body held 185 mm short of the edge ON REAL
  PHYSICS. Guard supremacy, single-writer preemption, honest async
  (stop() resolves a goto with `stopped:"user"`), and v1 tools absent-not-
  stubbed are all pinned by tests. `pytest harness/ -m "not slow"` = 1 s.
- **Cliff stop path (D034):** `ReflexSupervisor.request_stop()` routes any
  external halt through PLANT→BRACE→RECOVER. Measured
  (`sim/run_cliff_safestop.py`): old path leaves the gait MARCHING IN
  PLACE at the edge (24 post-halt contact breaks/4 s); new path: **0
  breaks**, same 185 mm stop margin, lower peak tilt (0.49° vs 0.68°),
  detector still fires with the supervisor owning ctrl. PASS.
- **BECKON (B18 SHIPPED):** `pebble_gestures.beckon` + narrated
  `sim/run_beckon.py` video (greeting → rising curious_question on curl
  two → acknowledge). Cost a real lesson (D035): the position-space curl
  IK'd to hip 98–117° against the +90° limit — both extremes clamped and
  the arm didn't move; caught by diffing extreme frames, re-authored in
  joint space (80° knee sweep, reads beautifully). Bonus find: fist-bump's
  carry/extend waypoints ALSO ride the hip clamp (benign, but hand-v0.3
  should re-author them).
- **Viewer "Print weekend" mode** (browser-verified, 8 modes now): the
  Sunday-night dry-fit — leg chain with the 3 posed servo blanks (their
  own boolean check in `leg_assembly.py`: blanks × printed structure =
  0.00 mm³), fit ladder v2, closed hand.
- **Parallel `run_all_checks.py`**: modules were already independent
  subprocesses — now N-at-a-time (`--serial` kept). 188 s → ~95 s on 2
  cores; scales with the machine.
- **`cad/print_estimate.py`**: honest ±30 % grams/time per weekend plate,
  totals in the plan. Headline: **the whole weekend ≈ 460 g — one 1 kg
  spool covers it with margin.**

**Broke / found**
- Beckon v1 saturation (above) — renders lie even in motion: the clamped
  arm looked plausible in every still. Extreme-frame diffs are now part
  of the gesture-video checklist.
- Fist-bump waypoints past the hip limit (benign today, D035 notes the
  v0.3 retune).

**Next** — unchanged from 6a (print weekend + carts), plus: session 7 can
extend the LIVE harness (wake-word → whisper.cpp → regex intent → these
exact tools) instead of building it from a spec.

---

## 2026-08-07 · Session 6 — print-weekend prep: fit ladder v2 (+ a real v1 bug), servo blanks, tibia clamp v0.2, weekend GO/NO-GO plan

**Status in:** printer z-calibrated (Live-Z done), sheet question answered
(textured PEI = correct for this project; smooth is a nice-to-have);
NOTHING printed, NOTHING ordered, no RL run. Goal: print the body + one
leg this weekend and dry-fit.

**Done**
- **Fit ladder v2** (`part_fit_ladder.py`): new row E — Ø2 hinge-pin holes
  (2.00/2.10/2.20 — testable with Ø2 filament TONIGHT), 683ZZ press
  pockets (6.85/6.95/7.05 ×3.4), Ø10 tube sockets (10.15/10.30/10.45).
  The v1 ladder never tested the bearing seat, the tube fit, or the pin
  holes — three of the leg's real interfaces. Plate 118×90.
- **Fit ladder v1 BUG found + fixed:** the Ø4.8 heat-set pocket MERGED
  with lip-slot #2 (wall broken open → falsely loose reading). Shipped in
  v0.7.3, caught by a section-slice pass. Slots dropped to y −29; module
  now asserts ≥1.5 mm edge margin + ≥1.0 mm min-wall between ALL features
  (D033: plate-layout audits are mandatory for multi-feature coupons).
- **`part_servo_blank.py` (B17, NEW):** printed exact-envelope ST3215
  stand-in — case + boss + horn disc with the true M2 BCD. Boolean-proven
  0.00 mm³ in all three cradles (coxa base / fork rails / knee carrier)
  and against the reference dummy envelope. With 3 blanks the whole leg
  chain assembles rigid THIS weekend despite zero servos on order. Tree
  CI now 20 modules.
- **Tibia clamp v0.2** (`part_tibia.py`): the pinch bolt was a bare Ø3.4
  through-bore (nothing to thread into, nut would sit on a curve). Now:
  clearance side + Ø2.8 thread-forming side + Ø6.5 spot-faced head seat
  (≥1 mm wall to the tube bore, checked analytically). M3×10 is already
  on the session-5 addendum.
- **`docs/PRINT_WEEKEND_s6.html`** (Playwright-verified, delivered): plates
  P1–P6 in dependency order, textured-sheet PLA settings (60–65 °C bed,
  elephant-foot comp 0.15 ON, re-run Live-Z on THIS sheet), the Saturday
  GO/NO-GO caliper table incl. the D032 slicer XY-hole-compensation
  fallback (bores only), the dry-fit reality check, NOTES_INBOX template.
- **Order spot-check (08-07):** Waveshare adapter $4.99 ✓, Seeed ST3215
  $22 IN STOCK (≈$5/servo over AliExpress but weeks faster — Tyler's
  call), SCS0009 $9.99 ✓, 683ZZ $4.38 ✓. **Carts still unplaced — the
  servo order remains the project's critical path.**
- **`docs/MCP_CONTRACT_v0.md`**: buildable Rocky-MCP v0 (6 tools, guard-
  supremacy invariants as tests, wake-word/ASR scope fence, session-7
  demo definition-of-done). `docs/SCALE_UP_NOTES.md`: what survives the
  Pebble→Rocky jump (params/CI/interfaces/coupon methodology) vs what
  deliberately doesn't (printed threads, PLA sections, printed detents).
- Backlog: B17 shipped, B18 filed (beckon gesture + name-motif greeting,
  sim session). Decisions D032, D033. PRINTER_NIGHT queue item 15.

**Broke / found**
- The v1 fit-ladder merge above — renders never showed it; the flat-plate
  version of the D014 lesson. Slices and min-wall math, every plate.

**Next**
- Tyler's weekend: place carts (FIRST), then P1→P6 per the plan; fill
  NOTES_INBOX. Session 7: file measurements → params → regen → 20/20;
  then MCP-contract v0 against the sim, or bench runbook if servos landed.

**Done**
- **Viewer v0.3** (`cad/pebble_viewer.html`, renamed version-agnostic —
  browser-verified): 7 modes with per-mode hint text — Full robot,
  Carapace, **Bench & field** (stand STACKED at the 127 mm config, dock
  with funnel + tower, I2 tools, clips), Leg, Hand, Parts: mechanism,
  **Parts: session 4-5** (deck v0.4, sled, tray, star-board bracket,
  sector, cap, clips). 16.7 MB self-contained.
- **Showcase reel** (`sim/pebble_showcase_v07.mp4`, 31.6 s, narrated):
  locomotion medley (walk/strafe/turn) → 40 N shove + the v2 PLANT→BRACE
  safe-stop recovering on real contacts (first time on film) → cliff
  approach + VOID stop + retreat (also first footage). Supersedes the
  session-2 reel; stuck-retry + gestures videos stand as-is.
- **bom/ADDENDUM_session5.html**: the deltas (washer strikes, magnet
  quantity bump to ≥20, M3×10s, perfboard, XT60E-M for the dock —
  flagged EST-not-live, quantities are the truth). Interaction stack:
  nothing to buy — mic array + speaker already on Batch 3.
- README refreshed (layout, videos, run_all_checks + train_ppo in the
  quickstart, status current).

---

## 2026-07-31 · Session 5c (same day) — refinement pass: B13/B14 shipped, tree-wide CI, vision/interaction plan

**Done**
- **Cable clips (B13 → SHIPPED)**: `part_clips.py` — snap tube_clip (80 %
  gap) + link_clip, wire tunnels + zip slots.
- **Charging dock mechanicals (B14)**: `part_dock.py` — walk-on plate,
  funnel rails (±8 mm capture → ±0.8 = the XT60 float's own tolerance),
  shin bumper, plug tower meeting the 20 mm-crouch mate plane within 2 mm
  (VERIFY); feet verified to land on ground. Robot-side charge port =
  Batch-2 electrical decision, options in the docstring.
- **`cad/run_all_checks.py`** — the tree's CI: every part module's own
  checks, one command. First full run: **19/19 PASS, TREE CLEAN** (199 s).
- **`docs/VISION_PLAN.md`** — the L0–L3 world-model stack (lidar-first,
  Rocky-canon 360° geometric sight; cameras hidden behind gills, B16),
  the Pebble split (reflexes onboard / VLM cortex on the home server,
  link-down safe by construction), big-Rocky onboard NPU path, **the
  talk-to-Rocky loop** (ReSpeaker + speaker are already on Batch 3:
  wake-word → whisper ASR → harness → chord-speak replies + gestures,
  human jazz-hands/fist-offer recognition via pose model or VLM), and
  **Rocky-as-MCP-server** — say/gesture/goto/scan/dock tools over a
  guarded reflex layer, so the brain (local LLM, cloud API, onboard NPU)
  is swappable without touching the robot. Task ideas filed: patrol-and-
  report, guard/map-diff, follow-me, fetch/point, scoop delivery,
  question game, jazz echo, bedtime round.
- RL run 2 (σ=0.37) finished the session at **1.57M steps, return 161** —
  past run 1's 1.4M-step peak (140) with better trend; both resumable.
- Backlog: B13/B14 shipped, B16 filed; fingertips (B15) stay caliper-gated
  on purpose.

**Next** — see docs/NEXT_SESSION (the session-6 prompt draft).

---

## 2026-07-31 · Session 5b (same day) — CAD interaction pass; deck v0.4; RL actually trains; JAZZ HANDS

*(Tyler: printer refurb finishing tonight/tomorrow; first part orders
imminent — so this half-session closed every "the printed body can't
actually assemble yet" gap and added the interacting mechanisms.)*

**Done**
- **Deck v0.4 (D030):** the shell finally has something to bite — 5 latch
  strikes (tangential peg entries after the audit caught radial ones 0.9 mm
  off the pentagon), 5 washer recesses under the foot magnets, power
  grommet + zip anchors per BUS_STARBOARD. Coxa dock re-checked 0.00 mm³.
- **Carapace seams became real joints (D030):** outline noise made
  72°-PERIODIC (five identical sectors previously met with mismatched wall
  radii at every seam — invisible in renders, fatal for joints) + a
  chaining tongue-and-groove on the sector edges: pair interference
  0.00 mm³, engagement proven 15.5 mm³ (and the first groove was cut into
  open air on the wrong side of the edge plane — the check caught it).
  Shell magnet layout simplified to one per web after the deck said no.
- **JAZZ HANDS + FIST MY BUMP** (`gait/pebble_gestures.py`,
  `sim/run_gestures.py`, video delivered): non-adjacent two-arm jazz with
  4.5 Hz claw flutter + shimmy + bounce; fist bump with the closed cone as
  the fist (canon), a mocap "friend fist", and the bump-give triggered by
  the ACTUAL detected contact force — the same signal path as the SEA foot
  switch. Narrated: greeting → AMAZE → curious_question → acknowledge →
  discovery. Lesson: the leg's INNER reach annulus (|L2−L3| = 40 mm)
  rejected the first carry pose — check both reach limits, not just outer.
- **Bench stand** (`part_stand.py`): base + stackable sections + crown,
  every joint the same printed I6 spigot (aligned slides 0.00 mm³,
  36°-misaligned registers at 6030 mm³). THREE arms at the free webs —
  the belly bay blocks two webs and coxa-plate cantilevers rule out all
  stations (layout checks encode it). Heights 47/127/207 mm; 207 frees
  the entire 172 mm leg envelope for bench-day calibration.
- **First real I2 tools** (`part_tools.py`): hook + scoop carrying the
  hand-hub's exact bayonet socket recipe — insert free / lock free /
  locked-and-pulled RETAINS (20.5 mm³), byte-for-byte the part_tibia check.
- **RL kit v1.1 + first real training (D031):** DR now DRAWS per episode
  from stored bases (v1 friction randomization COMPOUNDED across resets),
  adds mass/torque/gravity-tilt/action-latency, `--cmd-sample` trains the
  whole command envelope, reward tracks yaw rate. In-session run #1
  (σ=0.6 exploration): 1.4M steps, return 37→140 — but the bare gait
  scores 345 under the same DR: PPO was mostly learning to mute its own
  exploration noise, and the deterministic policy had wandered into a
  slow 20 mm/s crawl. **Residual-RL lesson: exploration must start QUIET
  on top of a good controller.** log_std init −0.5→−1.0; run #2 reached
  run #1's 500k-step return within 50k steps. Curve + checkpoints in
  `sim/runs/` (resumable on the laptop: `--resume`).
- Print queue items 12–13 (tools as I2 fit coupons; stand overnight-class);
  backlog B12–B15 (lidar cap awaits the puck decision, cable clips,
  charging dock spec, fingertip serrations).

**Decisions** — D030, D031 filed.

**Next**
1. Printer: fit ladder FIRST, then coupons; session 6 = measured regen of
   EVERYTHING (now including sector seams + deck strikes).
2. Laptop RL: `python3 train_ppo.py --num-envs 8 --cmd-sample` overnight;
   then eval-vs-zero with the video.
3. Order note for the body-test batch: M3 washers ×5-10 (deck strikes),
   Ø6×3 magnets, M3×10 for latch inserts ride the existing addendum lines.

---

## 2026-07-31 · Session 5 — Sim loops closed (brace/SLAM/yaw); RL kit; chord-speak v0.2; carapace v0

*(Another no-hardware session: printer repair happening tonight, carts &
servos still pending. Priorities 2–6 of the session brief; nothing printed
means nothing to file, so the fit-ladder → params regen pipeline stays
armed for next time.)*

**Done**
- **Push-reflex v2 + an honest re-measurement (D025):** diagnosed the
  +0.25T pocket down to mechanism (freeze breaks the polygon at the shove
  peak; the crouch EXTENDS loaded pivot-side legs and pushes the robot
  over its own support line — 51° vs 14.7° tilt, `diag_brace_phase.py`),
  then found the D022 harness itself was PHASE-CONFOUNDED (supervisor
  clock ran during idle: every reflex trial 0.625 cycles off its
  baseline). Phase-aligned sweep: **bare gait is the best single-impulse
  controller** (mean 43.5 N vs v1's 37.1); v2 (finish-the-step PLANT →
  tilt-vector contact-seeking BRACE) recovers to 40.9 N and ≥ everything
  at all 8 probe phases in the weakest direction. Sustained-lean test:
  no reflex benefit. Verdict: reflex OFF for pushes, v2 machinery becomes
  the SAFE-STOP primitive. `fig_push_reflex_v2.png`.
- **SLAM-lite (D027):** `perception/scan_matching.py` — point-to-line ICP
  (self-test recovers a synthetic pose to 0.5 mm/0.03°) + keyframe
  matcher seeded by EKF deltas. On the lap: **ATE 55.6→8.2 mm, yaw
  4.8°→0.06°, map IoU 0.26→0.72**. `sim_lidar.py` now runs the EKF
  in-loop (honest odometry in the archive) — and caught the session-4 lap
  physically grinding pillar2 at 43 % speed (GT maps hid it; the
  stance-feet-don't-slip assumption screamed). Routes now audited.
- **EKF yaw fix (D026):** standing zero-yaw-rate pseudo-update through a
  StillnessGate + realistic bias prior. Flat walk 6.8°→**0.25°** end yaw,
  patrol 15.1°→**1.7°**, bg_z estimate lands exactly on truth; position
  drift improves too (yaw error had been rotating the velocity frame).
  Mag stub implemented, gated hard, default off. `fig_odom_v2_yaw.png`.
- **RL training kit (laptop-ready):** `sim/train_ppo.py` — vector envs
  (async + sync), DR on by default (friction + pushes), obs/return
  normalization, atomic checkpoint + exact RESUME (optimizer, normalizers,
  RNG), KL early-stop, SIGINT-safe, jsonl logs; `sim/eval_ppo.py` replays
  any checkpoint into an mp4 + walking metrics with a zero-action
  comparison column. Verified here end-to-end (train → resume → render;
  658 sps on 2 container cores — the 16 GB laptop with 8 envs will fly).
- **Chord-speak v0.2 (D028):** the Eridian voice — JI chord-syllables
  (septimal/undecimal), 66–200 Hz roots, one formant throat, band-limited
  breath, growl=urgency, beating=uncertainty; no scale runs, no cadences.
  16 words (+`determined`), event engine with priorities/cooldowns/
  preemption, and the 40 mm stuck-retry video re-rendered NARRATED
  (confused → thinking → acknowledge → found_it → AMAZE). A/B audition
  page browser-verified. Objective shift: piano-grid energy 85→24–39 %,
  centroid 210–360 Hz.
- **Carapace v0 (D029, the creature pass):** `part_shell.py` — five
  identical rock sectors + hatch cap: terraced perturbed-pentagon tiers,
  fork-swing-derived leg arches, web feet with latch pads + magnets (I3),
  vertical I6 dovetail bars, wall-following B8 LED groove, B9 gills.
  Four keep-outs boolean-clean; full-robot preview + WebGL viewer updated
  (new "Carapace v0" mode), Playwright-verified. Sector 39 cm³ (~47 g) —
  thinning pass queued post-caliper.
- **Wiring star-board:** `docs/BUS_STARBOARD.md` (perfboard net-list,
  proposed deck placement, build order) + `part_busboard.py` bracket +
  a COMPUTED cut-length table from deck v0.3 geometry (longest stub
  310 mm < the 400 mm ceiling) — "measure on deck" upgraded to "verify".
- Printer-night queue extended (items 10–11): busboard bracket + one
  shell sector/cap as the optional second overnight.

**Broke / surprises**
- The D022 phase-probe pocket was real, but HALF the story was the
  harness: idle-running gait clocks phase-shifted every supervisor trial.
  Alignment rule now lives in the reflex itself (clock starts with
  motion). Re-measure before re-engineering.
- v2.0's ground-seek commanded unreachable depths → IK NaN → one NaN in
  ctrl detonates the whole MuJoCo state. Reachability clamp + NaN guard
  now in the supervisor.
- Position-servo relaxation creep: 0.3–1.2°/s of REAL decaying twist for
  >1 s after any pose change — exactly gyro-bias scale. A zero-rate
  update fired early calibrates it in with the wrong sign (bg_z −0.0109
  vs true +0.0050 → 37° lap error). StillnessGate: calibrate at the END
  of pauses.
- Session-4's lidar lap drove through a pillar; ground-truth mapping
  can't see collisions. Physics keeps auditing everything it touches.
- Poly3D renders hide interiors: the first shell build's flange (with its
  latch pockets) was silently eaten by the cavity boolean and the render
  looked fine — boolean PROBES caught it (D014's spirit, extended: slice
  checks and probe boxes over eyeballs).
- v0.2's first breath layer was full-band white noise — spectral centroid
  7 kHz, a whisper not a chest. Band-limit, then judge.

**Decisions** — D025–D029 in docs/decisions.md.

**Next**
1. Tonight: printer refurb + PRINTER_NIGHT.md queue; file fit-ladder
   numbers in NOTES_INBOX → session 6 regenerates every part.
2. Tyler: carts (STILL the long-pole) + lidar decision (LDS02RR pull vs
   LD19).
3. Servos land: BENCH_RUNBOOK.md is the whole day.
4. Sim: RL scale-up on the laptop (`train_ppo.py --num-envs 8`,
   overnight); wire the v2 PLANT→BRACE into the cliff detector's stop
   path; ICP loop-closure when a second lap dataset exists.
5. CAD: shell thinning + deck v0.4 (latch strikes + star-board bolt
   pattern + power grommet from BUS_STARBOARD.md's proposal).

---

## 2026-07-30 · Session 4 — Bench-readiness pack; ROS 2 scaffold; reflexes; D020 modularity; perception stack

*(No-hardware session: printer still down, carts not yet placed — assumed
no change since session 3. Everything below exists so that parts-arrival
day is plug-and-play.)*

**Done**
- **rocky_driver v0** (`driver/`): dual-protocol Feetech bus driver (D016 —
  protocol 0 SCS0009 big-endian + protocol 1 STS little-endian on one wire),
  byte-level mock bus with motion/thermal/torque-limit models and fault
  injection, SafetyMonitor (warn 60 °C / cut 65 °C), PebbleRobot calibration
  bridge to gait-space. **58 tests green**, golden wire vectors included.
  `pip install -e .` at repo root installs pebble_gait + rocky_driver anywhere.
- **Bench pack** (`bench/`): bus_scan, assign_ids (one-at-a-time, 1–20),
  calibrate_centers (+dir nudge test), pose_check acceptance, register_dump
  (+diff), thermal_soak (auto-cut, plots), torque_step (D015 margin check) —
  every one rehearses with `--mock`, and the rehearsal already caught a
  calibration sign bug before it could eat a bench evening. BENCH_RUNBOOK.md
  = the day-servos-land script, gates + failure table included.
- **ROS 2 scaffold** (`ros2/`): URDF/xacro GENERATED from params.yaml (D004),
  parity-checked against the MJCF + analytic FK (0.05 mm / 0.000 mm worst,
  masses to 1 g — `sim/check_urdf_parity.py`); rocky_msgs, gait node
  (Twist-compatible), controllers.yaml, launch files; hardware backends:
  mock / topic-based (tested Python driver — recommended) / C++ skeleton.
- **Push-recovery reflex** (`gait/pebble_reflex.py`, D022): walking floor
  33→40 N; two sim-taught lessons (don't slide loaded feet, ramp the crouch)
  and one honest negative (stance reflex is counterproductive — gated off).
- **Stuck watchdog + retry** (`gait/pebble_watchdog.py`, D023): 40 mm rubble
  1/4 → 4/4 crossings; escalating stages (higher step → slower+taller →
  reverse). Shin fairing evaluated in sim and REJECTED (helps only outside
  the envelope, hurts the watchdog at 40 mm). Video: `sim/pebble_stuck_retry_40mm.mp4`.
- **RL groundwork** (`sim/rocky_env.py`, `ppo_smoke.py`): gymnasium env with
  residual-policy actions on top of the analytic gait (zero action = walks);
  compact PPO loop runs end-to-end (KL sane, checkpoint roundtrip) — Phase-3
  training is now a scale-up, not a build.
- **Modularity D020** (`docs/INTERFACES.md` + `cad/iface.py` + params freeze):
  six standard interfaces, ALL CAD'd and boolean-verified: leg-port rework of
  coxa+deck (dock check 0.00 mm³), bench jig consuming the SAME port code,
  battery sled + XT60 float dock + belly door, avionics tray + rails +
  bulkhead, SEA bayonet tool socket (insert/lock free, retains under pull),
  panel latch insert + shell sector + dovetail coupons. Renders in cad/out.
- **Hand v0.2.1** (D021): the cam/skirt graze is dead — 0.00 mm³ across a
  14-pose sweep; design open now 55° matching the software cap.
- **Perception, sim-first** (`perception/`, D024): legged-odom EKF at
  **1.0–6.8 % drift** (naive baseline 10–24 %); cliff detection PASS (stops
  187 mm before a table edge the control robot walks off — trigger is
  stance-without-contact, exactly D017's contact-timing advice); sim-lidar
  (mj_ray fan) + frozen LaserScan contract + occupancy map demo; WiFi RSSI
  fingerprint logger with k-NN demo (~1.8 m room-level prior).
  `docs/SENSING_PLAN.md` = the layered world model + pod standard + I²C budget.
- **Wiring harness plan** (`docs/WIRING_HARNESS.md`): per-leg drops, gauges,
  the 6 V BEC domain, star-of-daisies topology, power budget, build order.
- **Batch-3 addendum sheet** (`bom/ORDER_ADDENDUM_batch3.html`, live-verified
  prices 07-30): lidar promoted to core via $17 LDS02RR pull; US-100 skirt;
  ReSpeaker v2; BME688; PIR; mux; genuine-Amass connector kit; UBEC; ~$155
  core. Mass audit: sensing payload 399 g (pull) / 251 g (LD19) vs 450 g
  margin — both fit, tightly documented.
- **Chord-speak v0** (`audio/`): 15 words synthesized (additive organ-bell,
  chorus, reverb) with a consistent musical grammar; self-contained audition
  page browser-verified + delivered. AMAZE!

**Broke / surprises**
- **Deck v0.2's mid-pair bolt holes were ~2 mm OFF the pentagon** (radial
  91.6 vs boundary 89.5) — found by the new containment audit while laying
  out the leg port; also those bolts were unreachable under the servo.
  Nothing printed, zero filament lost, but the audit is now a permanent
  check in part_deck. I1 layout: dowels (−28,±19), thumbscrews (−41,±17).
- **The qpos-interleave gotcha bit AGAIN** (documented 07-29, still got me):
  legged odometry read qpos[7:22] as 15 leg joints and measured 62 % drift
  until joints were indexed by address (claw angles were feeding the FK).
  Same latent bug fixed in rocky_env's observation. Lesson upgraded to:
  *never slice qpos/qvel — always joint addresses, everywhere, no exceptions.*
- Reflex v1 hurt before it helped (marched in place at idle, dragged loaded
  feet outward, step-crouched): three separate sim lessons now encoded in
  pebble_reflex docstrings.
- Cliff detector v1 (depth-overshoot trigger) could never fire on stiff
  position servos — the foot stops AT its command, ground or not. The
  working signature is stance-commanded-but-no-contact.
- torque_step mock rehearsal exposed that an under-torqued servo model must
  not "re-chase" its goal each tick — mock realism patched until threshold
  detection matched prediction (0.90 ratio).

**Decisions** — D019 (filed from session-3 Q&A), D020–D024 in docs/decisions.md.

**Next**
1. Tyler: place Batch 0+1 carts (still the long-pole!) + the Batch-3
   addendum's lidar decision (pull vs LD19); fix the Prusa (§3.5).
2. Printer night: interface demo set first (latch insert ×2, coupler +
   coupon, dovetail pair, thumb knobs) — they burn <40 g total and prove
   D020's fits; then bench jig + calibration gauges + hand v0.2.1 set.
3. Servos land: BENCH_RUNBOOK.md top to bottom; file every number in
   NOTES_INBOX (esp. SCS0009 sweep + real register dumps).
4. Sim: gait-phase-aware brace (the +0.25T weak pocket D022 found); PPO
   scale-up on the laptop; slam_toolbox against sim-lidar bags.

---

## 2026-07-29 · Session 3 — Robustness proven in sim; Batch 0+1 ordered (sheet); servo trade study

**Done**
- **Batch 0+1 order sheet built & delivered** (`bom/ORDER_SHEET_batch01.html`) —
  vendor-cart layout, prices verified live 2026-07-29, running-total checkboxes.
  Notable drift: bus adapter $4.99 & ESP32 driver $15.99 (both cheaper direct),
  bench PSUs drifted UP (~$56–84), US carbon-tube stock is dry (AliExpress it).
  KW10 switches pulled forward from Batch 3 (SEA foot needs them at the leg gate).
- **Servo trade study** (3 parallel research agents, full findings in the sheet
  + decision log): D015 keeps ST3215 for Batch 1, sanctions STS3250 as the
  Batch-2 femur upgrade; found the ST3235 name-trap (30 kg·cm, not 35); QDD
  price floor collapsed ($56–70 SteadyWin, $125 turnkey RobStride/Damiao —
  Pupper V3 ships on $68 GIM4305s) → Rocky Jr math improves at the Phase-5 gate.
- **D016 wiring bug found in the master plan:** SCS0009 hand servos are 4.8–6 V
  devices and were drawn on the 12 V rail. Fix: shared TTL data/GND, separate
  5–6 V power domain (BEC in Batch 2). Driver must speak protocol 0 (SCS) and
  protocol 1 (STS) on the same bus.
- **Terrain robustness sweep** (`sim/run_terrain.py`, fig_terrain.png): blind
  wave gait crosses ≤30 mm rubble reliably; above that it SNAGS but never
  falls (worst tilt 13° at 40 mm). Binding constraint = 32 mm step height.
- **Push-recovery envelope** (`sim/run_push.py`, fig_push_envelope.png):
  survives 46–53 N standing / 35–53 N walking (0.15 s shove, zero reflexes) —
  ≥1.3× bodyweight from any direction. Quasi-static tip estimate (~30 N)
  validates the harness. Pentapod geometry is intrinsically shove-proof.
- **ADJACENT-ARM MANIPULATION WORKS** (`gait/pebble_manip_adjacent.py`,
  `sim/run_manip_adjacent.py`, video `pebble_manip_adjacent_repositioned.mp4`):
  crouch → step flanking feet to R280/±22° → lean 12 mm → raise both adjacent
  limbs in a FOLDED carry pose. Tilt ≤5.1°, work margin +10 mm; the no-reposition
  control falls exactly as D013 predicted. D018 logged.

**Broke / surprises**
- First terrain harness used a MuJoCo heightfield; near-degenerate prisms at
  small amplitudes produced spurious deep capsule contacts (flat ground
  "failed" while 16 mm passed). Replaced with a scattered-box rubble field.
  Lesson: distrust a sweep whose control case misbehaves.
- First adjacent-arm attempt used the reach-out arm_pose and TIPPED: two raised
  legs reaching outboard drag the whole-robot CoM ~35 mm — more than the entire
  static margin. Static support math with a fixed CoM is not enough; the sim
  keeps earning its keep.
- qpos layout gotcha: joints interleave per leg (yaw,hip,knee,claw)×5 while
  ctrl is [15 leg][5 claw]. run_sim.py's v0.3-era indexing patched; init now
  writes by joint address in new scripts.

**Decisions** — D015–D018 (+ D011–D014 backfilled into docs/decisions.md).

**Next**
1. Tyler: place the carts (AliExpress + Waveshare today — lead time), fix the
   Prusa (checklist §3.5 / PRINT_PREP_PACK.pdf), file results in NOTES_INBOX.
2. Caliper session when servos land → params v1 → regenerate parts → hand v0.3.
3. Sim: push-recovery REFLEX (crouch-on-impulse), gait-phase-aware push timing,
   stuck-detection prototype (D017 says trigger on progress, not tilt).
4. ROS 2 Jazzy scaffold; chord-speak prototype still queued as the fun one.

---

## 2026-07-28 · Session 2 — PEBBLE WALKS IN PHYSICS (+ deck, print pack)

**Done**
- **MuJoCo simulation** (`sim/`): MJCF generated from params.yaml (masses from
  the budget, position actuators clamped at the ST3215's 2.94 N·m, friction
  contacts). Full sequence walk → strafe → turn-in-place on video.
  Final metrics: 266 mm traveled vs 261 commanded (102%), 2 mm lateral drift,
  body height ±0.38 mm, max tilt 1.05°, zero falls.
- **The physics found a real bug the animation couldn't:** first sim run
  walked in place (−37 mm). Instrumentation showed stance feet sliding
  backward through the world. Root cause: swing-phase lift/land points were
  REVERSED (foot flew ahead→behind, teleporting 58 mm at both stance
  boundaries). The kinematic GIF looked fine at 12 fps. One-line fix in
  `pebble_gait.py`; kinematics were verified frame-consistent to 0.0 mm
  against MuJoCo before the fix was trusted. Lesson logged: cartoons lie,
  contacts don't.
- **Body deck v0.2** (`part_deck.py`): first attempt (R110 + ear tabs) failed
  the bed-fit check — 289×275 mm can't fit a 250×210 Prusa bed in any
  orientation, and it clashed with the coxa base (wrong z assumption). Redesign:
  R100 pentagon (190×181, prints one-piece), coxa mounts by inner+mid bolt
  pairs (mid pair added to `part_coxa.py`), outer 25 mm cantilevers past the
  deck edge (negligible moment at this scale). Battery strap slots, per-station
  cable pass-throughs, M3 electronics grid. All interference checks clean.
- **Print-Prep Pack PDF**: per-part 3-view dimensioned sheets, per-part
  material/orientation/supports/infill table, printer-night checklist.
  First prints are the hand set in PLA (cheap mechanism validation).

**Session-2 late additions (same day):**
- **Hand v0.2 — the cross-sections don't lie.** Slice-view inspection revealed
  the v0.1 "wedge" fingers were nearly FULL CONES with side gaps (build123d
  `Cylinder(arc_size=...)` semantics ≠ assumption) — three of them could never
  nest; the closed-cone render only looked right because the union hid the
  overlap. Rebuilt with explicit half-space sector cuts (`sector_solid()`).
  Also: real spiral cam (pin radius 9.00→16.44 mm over 60° cam / 65° finger,
  slot widening with travel for pin tilt), cam raised clear of the hinge lugs,
  pin-sweep relief pockets in the hub. Residual: ~7 mm³ skirt-vs-cam-rim graze
  mid-swing (bbox x 9.8–19, y ±13, z 18.6–21.6) — bench-tune scale; commanded
  open capped at 55° until v0.3 with measured servos.
- **ARM MODES — the Rocky repertoire, in physics** (`run_sim_manip.py`,
  `pebble_rocky_demo.mp4`, 31 s): 4-leg wave walk with a limb raised and
  gesturing (duty 0.78, body auto-leans 16 mm away from the arm; max tilt
  11.9°, upright throughout), then 3-leg stance with TWO arms + working
  grippers (visual claw joints added to the MJCF).
- **Physics lesson #2 of the day:** first 3-leg attempt FELL (tilt 43°).
  Support-polygon math: with ADJACENT arms raised, the CoM sits ~57 mm OUTSIDE
  the stance triangle — the required lean exceeds leg reach. NON-ADJACENT arms
  (legs 0 & 2) leave a wide tripod with ~57 mm margin: 12 mm comfort lean +
  10 mm crouch → tilt max 0.26° with both arms working. Choreography encoded:
  lean → raise → work → lower → recenter.

**Decisions**
- D011: first prints in PLA even for parts that will finally be PETG — material
  cost of iteration beats thermal correctness for bench prototypes.
- D012: deck = R100 one-piece + coxa cantilever, NOT a split 289 mm plate.
- D013: manipulation stance uses NON-ADJACENT arm limbs (support-polygon math);
  adjacent-arm manip deferred until a foot-repositioning step exists.
- D014: cross-section slice inspection joins renders + boolean checks as a
  standard CAD verification tool (it caught the wedge bug the other two missed).

**Broke**
- Deck v0.1 (bed fit + z-clash) — caught by automated checks, zero filament lost.
- Gait swing sign (above).

**Next**
1. Printer night: follow PRINT_PREP_PACK.pdf checklist; log to NOTES_INBOX
2. Order Batch 0+1 (still the schedule long-pole!)
3. Next session: fresh chat, "pick up ROCKY" — MuJoCo terrain tests, hand cam
   spiral math, or start the ROS 2 scaffold

---

## 2026-07-28 · Session 1b — CAD sprint: full leg + hand + gait engine

**Done**
- CAD v0.1→v0.3 in build123d (code-CAD, `params.yaml` single source of truth):
  coxa yaw base + fork (bearing-supported, ±40° sweep verified clash-free),
  femur twin-hub link, knee carrier, SEA foot cartridge (7 mm travel, switch
  pocket), and the transforming hand — 3 cone-segment fingers that close into
  the walking foot, SCS0009 + cam drive.
- Full-robot cosmetic preview: shells-over-skeleton, stepped pentagonal
  carapace, 5 legs instanced at 72°. Answers the "less roboty" goal — the
  creature is bolt-on shells; the mechanism stays pure.
- Self-contained WebGL viewer (4 modes). First version failed in the chat
  preview (external three.js CDN blocked → "THREE is not defined"); rebuilt
  with a hand-rolled renderer, now verified in headless Chromium before every
  delivery. **Rule: interactive deliverables get browser-verified first.**
- Gait engine v0 (`gait/pebble_gait.py`): closed-form IK (roundtrip 1e-14 mm),
  omnidirectional 5-phase wave gait, duty 0.8 (always ≥4 feet down). Initial
  defaults exceeded the coxa's validated sweep (50.7° > 40°) — retuned to
  1.6 s cycle / 185 mm stance radius / 45 mm/s → coxa peaks 21.9°. Walk,
  strafe, turn-in-place all reachable. Demo GIF in `gait/`.

**Decisions** → see `docs/decisions.md` (D001–D009 logged)

**Broke / gotchas**
- Fork v0.1 side-rails clashed with the femur servo (wrong orientation
  assumption) and the rear wall swept into the crown arm at ±40° yaw — both
  caught by boolean interference checks before any print. Checks stay mandatory.
- Chat-preview sandbox blocks external CDNs. Self-contained HTML only.

**Next session**
1. Tyler: order Batch 0+1 (see BOM), reassemble Prusa hot end, start PETG calibration
2. When servos arrive: caliper session → params.yaml v1 (kill all VERIFY flags)
3. First prints: hand (standalone mechanism test) + one coxa pair
4. Claude: MuJoCo scene + URDF from params; port gait engine into sim;
   pentagon body deck as a real printable part

---

## 2026-07-27 · Session 1a — Project founded

- Master plan written: Pebble (1:3 subscale) → full-scale gate → Rocky.
  Phase gates 0–5, physics verified (torque/mass/power budgets computed).
- Priced BOM: $975 core / $1,276 all-options, under the $1,500 envelope.
  Owned gear credited (hot ends, PLA/CF-PLA/purple PETG, Xbox Elite pad).
- Key corrections vs the earlier Gemini brainstorm: 12 V servo variant is
  mandatory (7.4 V doesn't close torque); no per-leg MCUs at subscale (one
  1 Mbps bus); servos do NOT hold static load indefinitely (thermal
  monitoring + sleep pose designed in).
- Vendor/deals strategy logged in plan §6.1 (Waveshare direct / AliExpress
  official Feetech — verify 12V variant; rpilocator for Pi; vacuum-lidar hack;
  never used LiPos).

---

*Template for new entries:*

```
## YYYY-MM-DD · short title

**Done**       — what physically/digitally happened
**Decisions**  — anything future-you must not re-litigate (also → docs/decisions.md)
**Broke**      — failures, surprises, measurements that disagreed with the model
**Next**       — the first 1–3 actions of the next session
```
