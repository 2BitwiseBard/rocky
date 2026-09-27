# ROCKY Build Log — Engineering Notebook

*Convention: newest entries first. Every work session gets an entry: date,
what happened, what was decided, what broke, what's next. Field notes and raw
measurements go in `NOTES_INBOX.md` and are filed later. Keep entries honest —
failed prints and dumb bugs are the most valuable lines in this file.
Sessions 1a–8d (before D047) are archived word for word; the table at the end
indexes them.*

---

## 2026-09-26 · Session 9i — repo review + clean-up: one BOM, the D047 CAD follow-ups, a generic setup, docs condensed and archived (D058, D059, D060)

**Ask:** go through the whole codebase; condense or remove old things, documentation
above all; update what is stale, the BOM and the CAD.

**The audit (read-only first):** nine areas (sim docs in two halves, BOM, the record,
CAD, sim code, core code, hardware docs, repo hygiene), **209 evidenced findings**
(74 update, 34 fix, 21 genericize, 18 delete, 17 condense, 17 keep, 11 investigate,
6 regenerate, 5 merge, 4 move, 2 archive), each with a file:line quote and a proposal,
45 of them flagged for the owner's call. Then two implementation stages and a rerun
of the sim experiments on today's model.

**Stage 1, code (five commits):**
- **BOM** (`c7fd4b1`, D058): `bom/BOM.csv` (63 rows; phases A bench kit, B full robot,
  C senses, D voice, X optional) + `bom/README.md` + `bom/totals.py` (checks every line
  total) replace the xlsx and five order sheets. Prices checked 2026-09-26: bench kit
  **$279.94**, core robot (A–C) **$1,257.91**, optional $422.59. 16 × ST3215 (no
  STS3250), one soft-case 3S 5200, the D500 lidar, BNO085 on SPI; no 683ZZ, no USB-UART
  dongle. Nothing ordered.
- **CAD** (`45fdae1`, D059): an 11 × 11 mm leg harness channel through `coxa_yaw_base`
  from the deck's I1 cutout to the yaw plugs, on the −Y side (the part 25.19 →
  21.23 cm³; six new `check_assembly` checks); the star-board bracket v0.2 bolts to
  existing deck grid holes (0,20)/(0,40) with a layout audit (5.38 → 5.07 cm³);
  `run_all_checks` 23 → **28 modules** (servo_st3215, servo_mount, part_port_coupon,
  leg_assembly, check_interference; four checks that printed CLASH but exited 0 now
  fail) and `rocky.sh cad-check --derived` adds the four generators (32/32, 230 s);
  byte-stable exports (STEP only on a geometry change, an invariant PDF, fixed STL
  headers); `print.bed_mm` and `interfaces.battery_sled.xt60_panel_mm` in params;
  the print pack moves to `cad/out/PRINT_PREP_PACK.pdf`; 60 stale PNGs and the old
  dry-fit posed files removed; the STEP's Apache-2.0 text in `cad/ref/`.
- **Generic setup** (`72e3f1d`, D060): machine knobs in a git-ignored `rocky.env`
  (template `rocky.env.example`, every `ROCKY_*` variable), read by `rocky.sh` and by
  `sim/envfile.py` for Python started without it; the environment wins. The code no
  longer reads one machine's own key variable (the key is `ROCKY_LLM_API_KEY` or
  `ROCKY_LLM_KEY_FILE`); the D055 model ids are the reference setup's
  overridable defaults; brain-install's models dir defaults to `~/models`
  (`ROCKY_MODELS_DIR`); CI pins MuJoCo 3.12.0 (`constraints.txt`) and build123d 0.11.1.
- **Core** (`730658d`, D060): driver sign bits per register (STS `PRESENT_LOAD` bit
  10, `POSITION_OFFSET` bit 11, VERIFY-ON-BENCH; an offset past ±2047 is refused);
  the ROS 2 plugin takes servo ids from joint names; the watchdog's retry ladder is
  derived inside the D052 envelope (24 mm / T 2.0 s / 45.5 mm/s → 34.8 mm / 3.0 s /
  40.6 → 42 mm + 10 mm body / 4.0 s / 30.4; the old 32/46/52 mm ladder had no
  envelope at all); the gait GIF and joint plot regenerated on the budgeted gait
  (T 2.0 s, step 24 mm, peak coxa 27.9° where the old demo reached 49.8°, past the
  ±40° limit); dead audio and harness files out. Driver tests 69 → 79.
- **Sim structure** (`5a70f37`): 20 one-off experiments + `diag_brace_phase.py` →
  `sim/experiments/` (results → `sim/experiments/results/`, clips →
  `sim/experiments/out/`, `ROCKY_EXPERIMENTS_OUT=DIR` redirects; run from the repo
  root); the 19 pre-D052 result JSONs and their figures →
  `sim/experiments/results/pre_d052/`; `sim/scenes.py` holds the scene helpers live
  code had imported from experiment scripts; `run_sim.py` writes to
  `sim/out/run_sim/` by default; dead code, the PPO smoke, four negative-run
  checkpoint folders and a stale walker checkpoint removed (git keeps them).
  **Bug found:** `run_fairing` built its rubble from a copy that skipped the per-box
  colour draw, so D023's fairing A/B ran on different fields than its baseline.

**Stage 2, docs (this entry):** one page per topic. `docs/PRINT_PLAN.md` (the dated
name dropped; the fit-ladder GO/NO-GO table from the deleted print-night pages),
`docs/WIRING_HARNESS.md` absorbs the star board, `docs/PERCEPTION_PLAN.md` replaces
the vision and sensing plans, `docs/RL_GUIDE.md` absorbs the RL tour,
`docs/SIM_GUIDE.md` absorbs the MCP contract and the playground page,
`docs/BRAINS.md` is the short brain guide, `docs/PLACES.md` is new, the README takes
the laptop-setup page. `docs/archive/` ([index](docs/archive/README.md)) keeps word for
word: the founding plan v1.1 (now bannered, with the D016 hand-servo correction), the
brain-model report, BUILD_LOG sessions 1a–8d, and decisions D036–D057 at full length
(the log condenses them to ADR-lite rows). D001–D035 stay as written, with AMENDED
markers where a later decision overrides them. `NOTES_INBOX.md` is back to its header
(its 09-08 and 09-17 entries are sessions 8e and 8f below). The backlog is one Open
and one Closed table, with the stage-1 follow-ups as B42–B65.

**Numbers that changed on purpose:** robot fingerprint `ceb63a1254c3` →
`7d376178fe27` (torso 1448.9 → 1435.2 g, robot 2670.4 → 2656.7 g, URDF ≡ MJCF at
2.657 kg); CAD checks 23 → 28 (32 with `--derived`); print estimate batches 0–3
378 g / 20.2 h, whole plan 1111 g / 53.8 h; fast tests **1,039 passed + 2 strict
xfail** (driver 79, harness 111, gait 75, sim 777 collected; last recorded 454 at
D053) in ~7.8 min; `run_stuck` on today's model with the budgeted ladder 4/4, 4/4,
2/4, 2/4 at 30/35/40/45 mm rubble (no watchdog 2/4, 0/4, 0/4, 1/4).

**Decisions:** D058 (one BOM: the 2026-09-22 electronics calls), D059 (the D047
follow-up CAD fixes), D060 (the repo goes generic and condensed).

**The rerun (same day, fingerprint `7d376178fe27`):** every experiment, the `sim/out/`
records (`shove_envelope`, `torque_audit`, `rl_curves`) and the demo clips re-made on
today's model; the table with both columns is `sim/experiments/README.md`. Held:
`run_sim` 246 of ~261 mm (tilt max 0.84°, turn 55.4° at 0.246 rad/s), `recover1` 0/20
`handoff_ok` with the system 20/20 vs 11/20, `recover6_d052` 0/20 (system 11/20 = no
righter), SLAM ATE 8.2 mm, and D018, D026, D027, D034, D043. Moved: the rim-shove floor
(BW now on the 2.657 kg torso) 5 N grid standing 25–35 N by direction (min 0.96 BW),
walking min 20 N (0.77 BW), 1 N grid 29 N (1.11 BW); the torque audit's self-righting
knee 1.488 → 1.459 N·m, 49.6 % of stall, **HOT → WARM (D044)**; the `recover1` legacy
height test 5/20 → 4/20; the cliff stop 186.8 → 252.3 mm short of the edge; odometry drift
on rubble 1.02 → 8.06 %. Verdicts that moved: **D017** rubble crossed reliably ≤ 30 →
≤ 25 mm (still snags, never falls); **D022** the v1 brace's walking floor 33 → 40 N no
longer reproduces (27 → 27 N); **D023** the watchdog at 40 mm 1/4 → 4/4 became 0/4 → 2/4,
and the fairing, first run on the same fields as its baseline, is still rejected but
"helps only at 45 mm" is false (it helps a little at 30–35 mm, hurts at 45); **D025** the
envelope order flips to v2 55.4 ≥ base 54.4 ≥ v1 53.2 N (one ladder step); **D040** turn
80° → 48°, sidestep 170 → 140 mm under the speed envelope; **D042/D045/D048** the
fallen demo falls 4/5, not 5/5 (seed 4 braces and stays up; the other 4 right and walk
away). The decision rows keep their dated numbers.

**Final pass (2026-09-27).** Code: ruff in CI (B64); a repo-root `conftest.py` sets
`ROCKY_ENV_FILE=/dev/null`, so the suite runs on the reference setup (B63); the benches
force their scratch cockpit conf (B62); `--mock` rehearsals, the cockpit's mock bridge
included, write only under `bench/out/mock/`, never `bench/calibration.yaml` (B71), and
`pose_check` works on a partial bus (B70); six part modules' checks and `part_shell`'s
bed fit now fail the exit code (B52); `params_rev` D052 → D059 (B53); `run_stuck_voiced`
defaults to seed 1, which crosses; the print pack and `bom/BOM.csv` cite the current
docs. Docs: a RERUN 2026-09-26 note on each decision row the rerun moved; B1 closed
NEGATIVE, B66–B68 opened; the facts that lived only in the D036–D057 archive moved into
SIM_GUIDE, PRINT_PLAN and DESIGN_CHANGE_GUIDE §8 (adding a tool); personal details cut
from the archive; every relative link and anchor in the Markdown resolves. Two
independent reviews (docs truth, code + CI + generic) then found 19 items, 18 fixed: the
MCP server now follows `ROCKY_COCKPIT_PORT` and reads `rocky.env` (B73); the gesture
audit's record is tracked, `sim/out/audit_gestures.json` (B69: 20 rows, 0 FAIL,
`manip_adjacent` 17.4 mm); the runbook's soak and torque-step loads come from
`sim/out/torque_audit.json` (walking 0.34, stance 0.45, self-righting 1.46 N·m), not
D015; the interface-coupon fit criteria and the fixture print poses, lost with the
print-night pages, are back in PRINT_PLAN; the photo's EXIF and phone trailer are
stripped (pixels unchanged); ruff is pinned in `constraints.txt`; the sled-vs-bay
width (50.8 vs 50 mm) is B72. Fast tests at the end: 1,035 passed, 7 skipped (the
brain-install checks, off on the reference setup), 2 strict xfail, in 5 min 43 s.
Left to the owner: the git history still carries the old machine paths and tailnet
name, so a public copy needs a fresh or filtered history.

**Next:** order the bench kit (`bom/BOM.csv` phase A); the fit ladder → `params.print`
(B28); print batches 0–1 (`docs/PRINT_PLAN.md`); the first servo settles the horn
(B23) and the VERIFY-ON-BENCH sign bits (B32).

## 2026-09-25 · Session 9h — three brains installed and measured; stop never waits for a model (D055, D055a, D056, D057)

**Done**       — `rocky.sh brain-install` + `brain-bench` (built by an 11-agent workflow, 26 review findings fixed). The owner's downloads (Qwen3.5-4B Q8_0, Qwen3.5-9B UD-Q6_K_XL, Gemma 4 12B UD-Q6_K_XL) installed with their projectors; five brains measured on the robot's jobs with one harness. Stop gate in every model mode, `move(forward_m, left_m)`, two prompt rules, per-model box order, world change forgets the eye, `event_seq`, scratch gesture library for the bench. Docs: BRAIN_MODELS status table + verdict, COCKPIT_GUIDE (choosing a brain, saying stop, move), SIM_GUIDE.
**Decisions**  — D055. `qwen3.5-9b` stays the default all-in-one brain, `qwen3.5-4b` is the small one, `gemma-4-12b` the Gemma option, the Q6 9B a delete candidate, the 26B and the 3B eye are not brains.
**Broke**      — Four `until ! pgrep -f` waiters spun all night (the pattern matches its own `zsh -c`); the auto-mode classifier refused the real install (it deletes downloads and edits the shared llama-swap config), so the owner ran `! ./rocky.sh brain-install --no-restart`; CI failed twice on the `--help` smoke test until the child ran with `MUJOCO_GL=disable` (cockpit.py's egl default leaks into subprocesses, the runner has no libEGL); a bench 'happy dance' saved a gesture into the repo; Gemma 12B's boxes are x-first, not y-first like the 26B (21.9° → 0.7°); both Gemma models answer a bare "stop" with a chord when unguarded.
**Done (later the same day)** — D056 shipped (b3df144): `harness/capabilities.py` registry, cockpit `/api/capabilities` with a version, the MCP list rebuilt live with `tools/list_changed`, `docs/TOOLS.md` generated and CI-checked; zero behaviour change proven against snapshots of the previous schemas. Fallback chains set to the D055 verdict. The owner's cockpit runs as the transient unit `rocky-cockpit.service`.
**Done (evening)** — B41 measured (D055a): per-family prompt notes lift Gemma 12B to 20/20 and the 9B to 19/20; thinking on hurts. `.gitignore` keeps the owner's personal to-do files (`OWNER_TODO*.md`, `TONIGHT*.md`) out of the public-facing repo.
**Done (night)** — D057 shipped (d9d73c8): place recognition from senses, off by default (`--recognize`), three-verdict bench 21/21 verdicts / changes 3/6 on the second run; `brain-install --uninstall`; the owner's evening checklist lives in the git-ignored `OWNER_TODO.md`.
**Next**       — B38 (h)–(j): confirm added objects from a sidestep viewpoint + name folding, widen the new-room margin, say "ambiguous until named" on the situation line; speaker id + people memory (B39); sensors as params (B40), judged with the three-verdict bench. (B41 closed: thinking on measured worse.)

## 2026-09-24 · Session 9g (laptop, evening) — talk to it: fast speech, an eye that measures, memory, an all-in-one brain, the phone front end (D053, D054)

**Ask:** the phone's mic said "nothing heard" and the whole loop was slow; "smaller
LLMs so it can all be on the GPU"; "a multimodal all-in-one"; "make the UI better for
the phone, a remote mode, a help guide, a better wake word"; "I'll revisit the CAD and
add joints / servos / materials"; "that memory and situational-awareness idea".

**Measured, then shipped (all pushed):**
- Speech: 18–20 s → ~1 s per command (`whisper-fast.service` :8086, base.en CPU greedy);
  hold-or-tap mic with a timer, hands-free listening, fuzzy wake word (`9a66eff`, `1740e26`).
- Brain: the research round (3 sweeps, 22 candidates, each verified) → Qwen3.5-9B
  downloaded, registered in llama-swap (runs alone), measured 7.2 GB / 6.5 s cold /
  57.7 t/s / tool calls 0.2–1.5 s / `look` OK; default multimodal role. lfm2.5-vl keeps
  the fast eye. Report: `docs/archive/BRAIN_MODELS_2026-09-24.md` (guide: `docs/BRAINS.md`).
- Vision: `find_object` with box→floor geometry, 0.7° bearing / 3 cm distance, 3/3 finds;
  `sim/vision_bench.py` (`46ee083`). A VLM is not a cliff sensor (1–3 of 4).
- Memory + awareness: `sim/scene_memory.py`, five tools for every brain and MCP, a
  situation line per turn, chord reactions; go_back_to measured (`876d9db`).
- The robot as data, step 1 (D053): RobotSpec from params, generators read it,
  byte-identical output; `docs/ROBOT_AS_DATA.md` + `docs/DESIGN_CHANGE_GUIDE.md` (`78df557`).
- B34 first retrain on obs v2: negative (0/20 hardware handoff); recover1 stays.
- Phone front end (tab bar, remote + follow, in-app guide, wake-word card): built and
  reviewed in this session; see the commit that follows this entry.

**Not verified:** any of it on hardware; tailnet on the phone was verified by the owner
(HTTPS works, plain http is refused by tailscale serve — type https).

## 2026-09-24 · Session 9f (laptop) — the full review and D052: the sim stops flattering the servo

**Ask:** review everything before a real servo goes on the bus. The owner
had measured that morning (scratch, not in the repo) that with joint
damping = stall / no-load (0.62 N·m·s/rad) the wave gait still walks
(242 of 273 mm; max joint speed 4.2 vs 7.4 rad/s ideal) but turns 145°
where the ideal model turned 157°, and with the torque derated to 1.9 N·m
as well only 103°; walk + turn yawed 74° vs 103°. "The turn envelope must
shrink, and that is the correct answer."

**The review:** 4 reviewers, 29 verified findings, worked as nine packages
(A model identity, B feasibility + authoring, C always-on loop, D1 cockpit,
D2 brains, E bus safety, F hardware-shaped RL, G launcher/CI, W worlds),
then a verification pass (V1) and a second review of the implemented tree
(V2, 27 findings; every HIGH/MEDIUM it could reproduce fixed). The theme
of the findings: the sim was right about logic and flattering about the
servo, nothing checked what a gesture or command asked of a joint, and the
D051 bridge would have enabled torque toward whatever goal a servo last held.

**What broke (found, not caused):**
- Gestures asked the impossible and nothing said so: fist_bump peaked at
  14.8 rad/s (the servo does 4.7 unloaded), beckon 5.9, turn_in_place 5.2,
  jazz_hands stepped its claw 23° in one tick; the gait at 45 mm/s asked
  a loaded knee 4.6 rad/s, and `walk 0 0 0.5` drove the yaw joint at
  7.4 rad/s past its limit.
- `recover1`'s 7/20 was flattered twice: an ideal 5 rad/s actuator, and a
  handoff test (torso z > 0.09) that passes a robot kneeling on its shins
  with 0–2 feet down. On the D052 model it is 2/20 on that test and 0/20
  on the hardware one.
- The icy-floor preset never did anything: MuJoCo takes the max friction of
  a pair and the feet said 1.2.
- 96/200 random courses put an object within 0.3 m of the spawn.
- `rocky.sh train-walk` passed `--env walk`, which the trainer rejected.
- Every MCP `gesture` sent to a running cockpit raised a TypeError (the
  tool wrapper's `name` argument collided with the gesture's).
- The sim thread could die and leave every HTTP request hanging forever.
- The 40 g claw mass was counted twice (2.705 kg compiled vs 2.670).
- In this round's own code, caught before shipping: the hands were checked
  against the 12 V leg window (a false "volt" every poll); the first soft
  entry jumped 20° on a knee resting outside its soft range; the servo
  model's load derate counted the torque-speed line a second time on top
  of the MJCF damping (now off by default).

**What changed (D052; details in `docs/decisions.md`):**
- **One servo identity** in `params.yaml` (`actuators`, `joints` with
  speed budgets loaded 3.0 / free 4.0 / hard 4.7 rad/s, `gait`, `reflex`,
  `sensing`), one loader `gait/rocky_model.py`; MJCF, URDF, servo model,
  shove, driver clamp and cockpit all read it. Damping 0.626 N·m·s/rad,
  forcerange 1.911 N·m, μ 0.8 + torsional friction, the foot surface is
  the IK foot point, spawn 1.4 mm. Robot fingerprint `5a32f772ca99`.
- **Gait** T 2.0 s / step 24 mm (duty 0.8 kept; 0.75 was rejected by the
  checker: CoM 59 mm outside the planted feet for 25 % of the cycle) and
  `WaveGait.budget()` on every command source: 45.5 mm/s at any heading,
  0.246 rad/s in place.
- **One feasibility checker** gating code gestures, keyframe saves, the
  studio, gait presets and the brain's `compose_gesture`; a reach solver
  and teach-by-demonstration in the studio. Gestures reworked until they
  pass.
- **Always-on loop** in the Playground: blended gestures with the reflex
  watching, one void guard for every source, trip latch, lead-limited
  probe, servo model on, NaN hold + 4.7 rad/s clamp on every target stream.
- **RL** on the robot's own action path and sensors (obs v2), checkpoint
  contracts with the fingerprint, `handoff_ok`, shoves in the gait env.
- **The bus gets a safe first move**: standstill-only mirror, soft entry
  (parked goal, 40 % torque, 200 c/s, blend), whole-leg fault cuts,
  degraded / lost with explicit re-arm, heartbeat, port-loss limp, EEPROM
  angle limits (`bench/apply_limits.py`), mirror dropped when the sim's
  reflex leaves NORMAL. Eight deliberate breakages, eight test failures.
- **Cockpit**: loud sim-thread death, request guard (Host allow-list,
  same-origin, JSON), `--unsafe-lan`, brains in `sim/cockpit_brains.py`,
  reactive lidar goto, model panel. CI diffs the generated model files.

**Measured on the D052 tree:**

| | before | D052 |
|---|---|---|
| `run_sim` walk (8 s, ~261 commanded) | 264 mm, tilt 1.03° | 246 mm, tilt 0.84°, height 117 mm (was 129) |
| walk 45 mm/s, 6 s | 259 mm, peak joint 4.20 rad/s | 231 mm, peak 3.14 rad/s |
| turn 0.5 rad/s, 6 s | 155°, peak 7.19 rad/s | 103° (owner 103), peak 3.16 |
| walk + turn (45, 0, 0.35) | 103° yaw, peak 8.13 rad/s | 76° yaw (owner 74), peak 3.18 |
| `recover1` stood, hybrid | 7/20 (pre-D052 model) | 2/20 legacy handoff · **0/20** `handoff_ok` · 0/20 servo nominal · 0/20 randomised |
| system (supervisor + righter + stall ramp) | — | **20/20** vs 11/20 with no righter; 9/9 declared falls end on the stall ramp, 0 on a handoff |
| `recover1` jitter, 40 N shove | 73–75 % pinned, ~10 rev/s, 4.5°/tick | 52 %, 6.7 rev/s, 3.3° (servo nominal) |
| shove, standing | 25 N (0.94 BW) all directions | 30–35 N (1.15–1.34 BW) |
| shove, walking | min 0.76 BW, mean ~0.91 | min 0.76 BW, mean 1.15 |
| gestures (`audit_gestures`) | 5 of 10 code gestures FAIL (speed / limits, same checker) | 20 rows, 0 FAIL; tracking p95 13–19° on arm + gait rows (warn at 10) |
| fast suite | ~101 (97 at D050 + 4 at D051) | 297 passed, 1 strict xfail |

What got worse is the point: the turn envelope shrank, mixed commands scale
down ((45, 0, 0.35) → (19.6, 0, 0.152)), the walkers on disk have no
envelope on the D052 servo (their gait's swing alone asks a loaded knee
4.43 rad/s) and are zeroed until retrained, and no righter earns a handoff
by itself. The shove envelope got *better*, which is suspicious in the
other direction (slower, lower gait plus joint damping; not isolated).

**Still open:**
- **No real servo** has touched any of this; every bridge rule ran on the
  mock. GOAL_SPEED 0, torque-enable toward the last goal, angle-limit
  behaviour and the POSITION_OFFSET sign are VERIFY-ON-BENCH (B32).
- **The tailnet path is unexercised** (tailscale was logged out).
- Owner decision: the 4.0 rad/s free budget exceeds what the MJCF lets any
  joint reach (1.911 / 0.626 = 3.05 rad/s) — a strict xfail until either
  peak torque + a thermal model goes into the sim or free drops to ≤ 3.0.
- The void guard misses cliffs approached at 10–20° (V2 finding); the void
  retreat barely retreats; goto's reactive layer is not a planner.
- On its back with no righter the supervisor loops FALLEN → RIGHTED every 3 s.
- No training on the new contract yet: B34 has the overnight recipe.
  μ 0.8, continuous 0.65 × stall, the SCS0009 numbers, foot-switch forces
  and all observation noise are guesses marked VERIFY.
- No real LLM tool loop or Claude brain call ran; CI has not run on GitHub.
- New backlog: B33 (CAD CoM/inertia, SEA slide joint, ToF), B34 (retrain),
  B35 (the onboard loop).

**Later the same day (D052 follow-ups, D052a):** the owner ruled on the
open speed question — a position servo's instantaneous torque is its stall
torque, and 0.65 × stall is a *thermal* budget. The 1.911 N·m clip had
capped an unloaded joint at 3.06 rad/s, below the 4.7 no-load and the 4.0
free budget; the leg clip is now 2.94 N·m (claws 0.23, with their own
DC-line damping), an unloaded joint reaches 4.71 rad/s, and the D052
strict xfail passes. The continuous budget moved to where time matters: a
heat integral in both RL envs (trips after 3 min at 0.85 × stall, then
derates the joint and costs reward), a `THERMAL_LOAD` verdict in
`audit_gestures`, and accounting in the live sim (sim → robot refused
while a joint is past its budget). A review of that round moved the heat
onto the motor current (force − damping × qvel), not the raw force, which
had counted a free joint spinning at no-load as stall-level heat. Second
package: the void guard's 10–20° misses. The wave gait lifts the next leg
the instant one lands, so two leading legs straddling the edge left the
CoM outside the other three and the robot tipped in ~0.3 s, before its
probe finished. A touchdown gate (a foot that has not felt ground 140 ms
after touchdown winds the gait back and waits) and a retreat that replays
the approach backwards stop walk 45 at all eleven headings tried from −30
to 60° (was: fell at 10, 15 and 20°) and cut a 72-approach sweep from 28
falls to 6; a 1° grid still finds a lip band (7 of 310 falls, a foot
centred on the edge's corner closes its switch and slides off), pinned as
a strict xfail. Third package: an older spawn bug — six files (and three
scripts built on one of them) wrote the leg angles into `qpos[7:22]`,
which interleaves the claws, so every push and terrain trial began with a
9.8° spawn tilt (every `tilt_max` read 9.83°); the tracked push / terrain
JSONs predate the fix. Re-measured on the peak model against an in-memory
1.911 A/B: nothing moves in normal operation (no joint reaches the old
clip: worst RMS load 0.39 × stall); `recover1` 5/20 legacy vs 3/20
(noise), 0/20 `handoff_ok`, system 20/20 vs 11/20 unchanged; the shove
floor is 29 N (1.11 BW) standing, 2 N *below* the clip — D052's "harder to
tip" was the clip letting legs yield into a slide, read on a 5 N grid.
Fast suite 325 passed, 2 strict xfail. Still open: the lip band, the
careful walk as default (1 fall in 72 but 17 % slower — the owner's call),
the thermal constants (VERIFY on the bench), and the rest of the list
above — except the free-budget question (closed) and the retreat that
barely retreated (fixed).

## 2026-09-24 · Session 9e (laptop) — toward the real robot: gesture studio, chord designer, servo realism, the hardware bridge, tailnet (D051)

**Ask:** "how do we get it set up with tailscale?" and "what else makes it
ready for playing with — gaits, gestures, sounds, better realism, more tools —
so I'm ready to use it with the robot one leg at a time?"

**Shipped (all in the cockpit, `./rocky.sh cockpit`):**
- **Gesture studio** — gestures as keyframe JSON (`gait/gestures/`), posed live
  in physics, snapped, scrubbed, played, saved; they join every gesture list
  (teleop, console, brains, MCP). `gait/pebble_keyframes.py` is the player and
  returns the same `(q, claw)` stream the code gestures do.
- **Voice & sounds** — chord-speak plays in the browser (phone included); a
  chord designer over `chordspeak2.py`'s syllable model, load a canon word,
  edit, hear, save as a new lexicon word (`audio/custom/`, `audio/samples_custom/`).
- **Gait lab & realism** — a footfall diagram, duty and stance-radius sliders,
  and a servo model (`sim/servo_model.py`): 50 Hz hold, 20 ms latency, 4.7 rad/s
  slew, 4096-count quantisation, off by default.
- **Hardware** — `sim/hw_bridge.py`: open the Feetech bus (or the mock), scan,
  legs present = legs whose three servos answered; sim → robot streams the
  sim's targets to those legs (25 Hz, counts/s cap), robot → sim makes the sim
  follow the real leg; telemetry table with jog, torque/LIMP, SafetyMonitor
  events, center + dir calibration into `bench/calibration.yaml`, set-ID.
- **`rocky.sh tailnet [PORT]`** — `tailscale serve` HTTPS in front of the
  cockpit (mic needs a secure origin); `tailnet off`. Tailscale was logged out on
  the laptop, so this is written but not exercised end to end.

**Broken and fixed:** `rocky.sh` failed to parse since f830ed4 (five help lines
had lost their `#`); `./rocky.sh cockpit` from the last session's notes would
have printed a syntax error. The 9c0416f SIGTERM fix never took: uvicorn
replaces signal handlers set before `uvicorn.run()` (`Server.capture_signals`),
so stale instances kept surviving `cockpit-stop`; the exit is now the server's
own `handle_exit`, verified with a camera stream open.

**Verified:** 4 new tests (`sim/tests/test_d051.py`), the full fast suite, a
Playwright pass through every new panel on the mock bus with zero console errors.
**Not verified:** a real servo — none has been on this bus yet (B32 is the runbook).

## 2026-09-23 · Session 9d (laptop) — cockpit round two: feet that feel for the floor, recordings, worlds on disk, the walker, voice (D050)

Asked for: all six follow-ups, plus how to start/stop the cockpit.

- **Contact-seeking stance** (`Playground._probe`): a planted foot with no
  contact is lowered up to 30 mm at 120 mm/s; a void = a foot that ran out
  of probe (`CliffDetector.update(..., probed_out=)`). Goto results from
  the origin: cliff → `cliff` at 0.15 m; flat 0.8 m, rough terrain 1.5 m,
  rubble 1.2 m, stairs (4 × 15 mm) 1.0 m → `arrived`; 6 cm box → `stuck`.
  Two wrong rules before it (foot-below-ground: never fires with position
  servos, walked off the cliff; foot-above-ground: still fired on bumps) —
  the cliff preset re-test after every change is what caught them.
- goto `stuck` (no 2 cm of progress in 6 s) and `timeout` (40 s) outcomes:
  the first obstacle goto had hung a tool call for 3 minutes.
- Recordings (`/api/record`, `/api/replay`): chase frames + every command
  (console, teleop, tool calls) → `sim/out/recordings/NAME/{clip.mp4,
  clip.gif, run.json}`; replay reloads the world and re-runs the commands.
  test1: 68 frames, 3 commands, replayed clean.
- Worlds on disk (`sim/worlds/*.json`, save/load) and `random_course(seed,
  n)`; the walker (`/api/rl/walk`: `robust_fwd2` residual on the analytic
  gait via `Playground.residual`, 0.16 m in 4 s); voice (`/api/voice`:
  browser blob → ffmpeg → whisper-server :8082 → chat); `/api/quit` + the
  page's ⏻ button, `rocky.sh cockpit-stop`, `cockpit` restarts a running
  one; a phone layout under 900 px.
- Bug: `set_world` compared the new name with itself → always respawned in
  place, so a preset switch from x = 1.2 m rebuilt the world around a robot
  standing inside a wall (that was the "cliff everywhere" I chased first).
- Tests 97 fast (+ terrain/random-course tests). Docs: SIM_GUIDE §3b, D050,
  B31, README.

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
  wrong in five independent ways, so every leg part the owner printed (photo in
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
  (From the notes inbox: coxa 78 g (fork 14.1), femur 34 (link 16.1 +
  plate B 13.5), tibia 131, torso 1449; walk +X 264 mm, tilt max 1.0°;
  URDF parity FK 0.04 mm, mass 2.705 = 2.705 kg, the claw still counted
  twice until D052.)
- **Docs**: the print plan (batch 1 coupons → batch 2 one leg → batch 3
  body; now `docs/PRINT_PLAN.md`), a shopping list (bench kit ≈ $300–370,
  full robot adds ≈ $900–1,050; replaced by `bom/BOM.csv` on 2026-09-26), D047 row, B22–B28, print pack
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
  `UnboundLocalError: update` (session 8e); it now explains that
  `--total-steps` is cumulative and exits cleanly. New launcher commands
  `train-walk` / `eval-walk`.
- Retired `docs/SESSION_WORKFLOW.md` (cloud-drop process) and the zip
  instructions in the laptop setup page (since 2026-09-26 the README's Setup).

**Decisions**: D047.

**Next**
- Fit ladder numbers → `params.print` (B28) — still the gate for every fit.
- Order the bench kit (shopping list part A); one servo answers B23.
- Print batch 1 (four coupons + blank), file fits; then batch 2 (one leg).
- Repo pass 3: move the one-off `run_*_v2` / `diag_*` experiment scripts
  out of `sim/` into `experiments/`; a hardware backend for the harness.

## 2026-09-17 · Session 8f (laptop) — the dry-fit fails: a joint suite, then D046

*Filed 2026-09-26 from the notes inbox (09-17, 09-17 later, 09-18); the D046 parts
were superseded five days later by D047.*

**Field report:** the owner dry-fit the printed leg chain. Base → yaw blank → fork went
together "not great", the femur link attached to nothing, the blanks did not attach,
nothing past the first servo mount connected. **The cause was in the CAD, not the
printer:** the checks proved non-overlap and one solid per part; none asked whether a
joint can be assembled or what holds it. A new probe, `cad/check_assembly.py`, at
nominal params:
- **The coxa stage was an assembly deadlock.** All four yaw-horn M2 driver columns ran
  through the crown arm (45.4 mm³ of base in each; the in-module "driver access CLEAR"
  had tested only the fork). The other order, fork bolted to the servo first and slid in
  from +X, failed too: the Ø2.9 stub hit the arm for x 3..9 (up to 17.8 mm³). No order
  built it.
- Vertical budget at the coxa: horn top 38.0 = hub bottom (0 gap), floor top 45.65 vs
  the arm's underside 46.15 (0.5), collar vs arm top 0.85; fork +1 mm → 125 mm³ into the
  arm. Without the 683ZZ the stub sat in a Ø6.85 pocket: 3.95 mm radial slop at z 48.
- **Femur link:** each hub was 4 × M2 through Ø2.4 into a Ø1.7 pilot in the blank's
  3 mm horn disc, then 3 mm of air (BCD r 7 outside the Ø6 boss); pilot bore Ø6.4 vs the
  blank's Ø2.1 centre hole, so no radial location: a 2 mm nudge met zero material.
- Blanks: zip ties only in every cradle; "horn disc UP, no support" was a Ø20 disc on a
  Ø6 boss, 7 mm of 90° overhang that `check_printability` did not flag. The horn coupler
  (founding plan §3.4) was an orphan: 1709 mm³ of overlap if put between.

**D046 implemented, CAD CI 23/23 TREE CLEAN (106 s), printability 58 parts clean:**
- J1 yaw (`part_coxa` v0.3): the crown arm became a bolt-on `coxa_crown_cap`; an M3 × 10
  axle threading 5.65 mm into a fork boss replaced the stub; horn centre-screw pocket
  Ø6.6 × 3.5; cap/boss gap 0.4; collar 1 mm over the cap top.
- J2/J3 (`part_femur` v0.2 + `part_coupler`): the link carries the coupler's recess on
  both hubs. Coupler lobes moved to 45/135/225/315 (they sat 0.8 mm over their own M2
  heads), half-angle 28 → 22° so a Ø4.4 driver fits, blind M3 bores 4.8 mm deep. The
  recess clamp angles are mirrored (−45/−225): a real bug in the first cut, caught by
  the new coaxiality probe. Link plate at y −25..−19 (was −22.3..−16.3).
- Retention (`servo_mount.py`): full-height lips (4 mm reach) + a printed strap per
  cradle; 2 mm nudges now meet 300–1300 mm³ in every direction. Blank v0.2: Ø14 undercut
  fill, 4 × Ø2.4 through + M2 nut slots (4.3 × 1.9 at z 33..34.9). Four coupons
  (`part_leg_coupons.py`) clipped from the production solids.
- The joint suite joined `run_all_checks`: it failed the old tree with 4 findings.
- Hardware missing from the order sheets then: M2 × 8 (≥ 16) + nuts (16), M2 × 6 (16),
  M3 × 8 (≥ 12), M3 × 10 (5).

**09-18, same session:** a **cantilever rule** in `check_printability.py`: an overhang
blob whose boundary touches the layer below on < 60 % of its length is a cantilever;
≥ 3 mm with a "no support" plan fails. Regression: the v0.1 blank's horn disc 6.7 mm
FAIL, v0.2 2.7 pass; `tibia_sea_outer`'s 9.6 mm roof over the tube socket reads as a
bridge (2.8 cantilever), pass. It caught one more: `calib_gauge_hip`'s cradle shelf
overhung its shaft by 5 mm at 90° with no support, a drooping reference surface, now a
lofted pedestal (34–40° from vertical). Viewer rebuilt (30.8 MB), print pack
regenerated. Mass audit with the cap × 5 (PETG) and the straps: torso 1462.4 → 1477.3 g,
coxa 86.2 → 89.2, femur 15.7 → 15.1, tibia 135.0 → 138.5; robot 2649 → 2691 g (+1.6 %,
inside the ±30 % fill-factor band; no re-baseline).

**Deferred then (answered by D047):** a rear idler boss for two-sided brackets (B19),
case-screw retention (B20); the yaw joint deliberately had no coupler.

**Decisions:** D046. Backlog B19–B21.

## 2026-09-08 · Session 8e (laptop) — the dev laptop runs the loop: the harness in physics, a local model drives it, the capped retrain is negative

*Filed 2026-09-26 from the notes inbox (09-01, 09-08, 09-08 later).*

**Before it (09-01):** the sim environment stood up on the laptop (venv at the repo root,
uv, Python 3.12). `matplotlib` and `scipy` turned out to be hard imports and `mcp<2` a
requirement (the SDK 2.x renamed FastMCP): both now in the pyproject extras. torch
2.13.0+cu130 from plain PyPI saw the GPU. Verified that day: `run_sim` walks 265 mm, no
fall; driver 58/58; harness 21 fast + 1 slow; gait 6/6; gestures2 7/7 clean;
`eval_ppo` on `robust_fwd2` return 284.6; the PPO smoke passed.

**Done:**
- `harness/sim_backend.py`: `ROCKY_VIEWER=1` opens the passive MuJoCo window and paces
  goto and gestures to real time (no display → one stderr line, headless as before);
  `gesture()` now **runs** the gesture in physics (was log-only; all 10 render clean in
  the flat world, no falls; `stop()` cuts one short → `stopped: "user"`; a fall reports
  FELL like goto); `ROCKY_AUDIO=1` plays the chord samples. **Goto bug:** the target
  direction reached the gait in the MAP frame, but WaveGait takes BODY-frame
  velocities; invisible while gestures were log-only (yaw stayed ≈ 0). Now rotated by
  the current yaw: turn + sidestep → goto (0.25, 0) and (−0.10, 0.20) both arrive
  within the 25 mm ball. Harness 21 fast + 1 slow green.
- `harness/local_brain.py`: `--base-url` / `ROCKY_LLM_BASE_URL` adds an
  OpenAI-compatible client beside the Ollama path (`ROCKY_LLM_API_KEY`,
  `ROCKY_LLM_MODEL`, `--think`, thinking off by default). qwen3.6-35b-a3b through a
  local llama-swap drove the mock robot first try: say(greeting) →
  gesture(jazz_hands) → status → a one-sentence report.
- The chord samples rendered locally (`chordspeak2.py`: 16 words + a demo reel).
- The CAD extras installed on the laptop (build123d 0.11.1): `run_all_checks` 21/21
  TREE CLEAN in 103 s, so the caliper → params → regen → checks loop no longer needs a
  cloud session.
- `rocky.sh` born (play / chat / brain / talk / voice / test / jobs / train-recover /
  eval-recover / cad-check); `voice` = push-to-talk through a local whisper server
  (large-v3-turbo) into `harness.intent --stdin`, verified with a wav. A phone caliper
  worksheet exports NOTES_INBOX line blocks.
- Found: `train_ppo --resume` with nothing left to train died with
  `UnboundLocalError: update` (fixed in session 9); `recover1` was trained at batch
  1024 (8 envs × 128), not the documented 8 × 256.
- `eval_recover` on `recover1` here = **10/20 stood** (the cloud said 12/20 on the same
  seeds: MuJoCo 3.12 / CPU float drift); hold-pose 2/20, random 1/20. The laptop
  baseline for comparisons is 10/20.

**The D045 capped v2 retrain: NEGATIVE, both arms** (deterministic mean action,
hybrid handoff, 20 episodes, same machine):
- `recover3_capped` (warm from `recover1`, +2 M steps, `--reward v2 --log-std-max -0.5
  --rollout 128`, 4.0 M total, 19 min, ~1.8 k sps): **7/20 stood**, pure-RL 0/20, mean
  return 316.8, end-tilt median 5.2°; back 2/6, side 2/7, tumble 3/7. Final entropy
  **13.78 nats = exactly the cap ceiling** (15 dims × (½ ln 2πe − 0.5)): every dimension
  pinned at log σ = −0.5, so the policy still wants maximal noise and the cap merely
  held it there. Training ep_len fell to 252 (stochastic rollouts do end in success),
  return 543.
- `recover3_scratch` (same recipe from scratch, 3 M steps, 22 min, ~2.3 k sps):
  **3/20**, pure-RL 0/20, back 0/6 (end tilt median 113°: never rights from the back).
  Entropy 10.75, under the cap on its own.
- Reference: `recover2` (uncapped) ended at 22.13 nats and 2/20; `recover1` at 17.78
  nats, ep_len 300 (never a training-time success), 10/20.
- Reading (a hypothesis, not proven): under v2 the stochastic policy gets up but the
  mean does not; the success rides on the noise, so capping σ removes the bang-bang
  symptom without moving the mean toward a righting strategy. `recover1` stays the
  shipped righter. (Both checkpoints were removed on 2026-09-26; git keeps them.)

---

## Earlier sessions (2026-07-27 → 09-01) — archived

Sessions 1a–8d describe the design era before D047 (the leg rebuilt around the measured
servo). They are kept word for word in
[docs/archive/BUILD_LOG_2026-07-27_to_2026-09-01.md](docs/archive/BUILD_LOG_2026-07-27_to_2026-09-01.md),
with every measurement they recorded.

| Session | Date | What happened | Decisions |
|---|---|---|---|
| 8d | 09-01 | FALLEN / RIGHTED reflex states; recovery reward v2 (`recover2` negative, the sigma cap); the talk-to-Pebble intent layer; live lidar `scan_summary` + a 5/5 patrol; torque re-audit on CAD masses; servo order v2 | D041–D045 |
| 8c | 08-31 | the dev laptop takes over: setup guide, keyboard teleop, a local-LLM brain, the flat chat-driving world, the RL ground-up tour | — |
| 8b | 08-30 | print-physics audit as a CI stage, sim masses from the CAD tree, gesture library v2; RL gait runs 2–3 and the first self-righting training | D038–D040 |
| 8 | 08-30 | tree-wide floating-parts audit: six more printables were in pieces; the single-solid gate moves into `export()`; print night re-planned | D037 |
| 7 | 08-30 | the hand hub's floating lugs: a knuckle collar (hand hub v0.2.2); connectivity checks tree-wide | D036 |
| 6c | 08-07 | the playground: interactive sim, live tuning, chat-driving over MCP | — |
| 6b | 08-07 | MCP harness v0 live, the cliff safe-stop wired and measured, the beckon gesture, viewer weekend mode, parallel CI | D034, D035 |
| 6 | 08-07 | print-weekend prep: fit ladder v2 (and a real v1 bug), servo blanks, tibia clamp v0.2, a GO/NO-GO plan | D032, D033 |
| 5c | 07-31 | cable clips and dock mechanicals, the tree-wide CAD CI (19/19), the vision/interaction plan, viewer v0.3, a showcase reel | — |
| 5b | 07-31 | CAD interaction pass, deck v0.4, RL actually trains, jazz hands | D030, D031 |
| 5 | 07-31 | sim loops closed (brace, SLAM, yaw), the RL kit, chord-speak v0.2, carapace v0 | D025–D029 |
| 4 | 07-30 | bench-readiness pack, ROS 2 scaffold, reflexes, the six standard interfaces, the perception stack | D019–D024 |
| 3 | 07-29 | robustness proven in sim, the batch 0+1 order sheet, a servo trade study | D015–D018 |
| 2 | 07-28 | Pebble walks in physics; the deck; the first print pack | D011–D014 |
| 1b | 07-28 | CAD sprint: the full leg, the hand, the gait engine | D001–D010 |
| 1a | 07-27 | project founded: the founding plan ([docs/archive/ROCKY_MASTER_PLAN_v1.1.md](docs/archive/ROCKY_MASTER_PLAN_v1.1.md)) | — |

---

*Template for new entries:*

```
## YYYY-MM-DD · short title

**Done**       — what physically/digitally happened
**Decisions**  — anything future-you must not re-litigate (also → docs/decisions.md)
**Broke**      — failures, surprises, measurements that disagreed with the model
**Next**       — the first 1–3 actions of the next session
```
