# Design backlog — evaluated 2026-07-30 (session 4)

*Each idea got an hour-or-less verdict: sim test where physics could answer,
CAD where the part was obviously cheap, spec-and-defer where it depends on
hardware we don't have. Statuses: SHIPPED (in `cad/`, print-ready),
NEGATIVE (tested, don't build), DEFERRED (spec'd, waiting on a dependency).*

| # | idea | verdict | evidence / where |
|---|---|---|---|
| B1 | Ramped TPU shin fairing | **NEGATIVE (for now)** | `sim/run_fairing.py` + `fairing_results.json`: helps only at 45 mm rubble (2/4 vs 1/4 crossings), *slightly hurts* at 35 mm (3/4 vs 4/4), and **degrades the stuck-watchdog's 40 mm result (2/4 vs 4/4)** — the fat shin closes the gaps the high-step retry threads through. Watchdog alone owns the 33–44 mm band; 45 mm+ is outside the mission envelope. Revisit only if real rubble chews the bare shin. |
| B2 | Belly skid rails | **SHIPPED** | `part_smallwins.belly_skid` — sacrificial, deck-grid bolts, rounded scrape face. Print 2 in PLA (they're meant to die). |
| B3 | Trim-mass bosses | **SHIPPED** | `part_smallwins.trim_cup` + lid — M3-washer ballast cups on the deck grid. CoM trimming with measured masses beats taping coins (D018 taught us CoM millimeters matter). |
| B4 | IMU TPU grommets | **SHIPPED** | `part_smallwins.imu_grommet` (×4) — isolates the BNO085 from gait-frequency deck ring; the avionics tray (I4) already has the Ø4.8 holes. |
| B5 | Calibration comb | **SHIPPED (as 2 gauges)** | `part_smallwins.calib_gauge_hip` (tower cradles the knee-servo case bottom at leg z 45.65 ⇒ hip = 0) + `calib_gauge_knee` (V-block plumbs the tube at x = 140 ⇒ knee = −90). Registers on the bench jig; `bench/calibrate_centers.py --knee-at -90` matches. |
| B6 | Palm magnets + shell tool docks | **HALF-SHIPPED (session 5)** | Shell side DONE: vertical I6 male bars at ±27° on every carapace sector (`part_shell.py`) — any dovetail shoe (B11 whisker, future docks) slides on today. Palm side still waits for hand v0.3. |
| B7 | Fingernail lip + TPU tip caps | **DEFERRED to hand v0.3** | Fingertip geometry changes ride the post-caliper rebuild; TPU foot pad (shipped, `part_footpad.py`) covers the walking-wear case meanwhile. |
| B8 | LED ring channel | **SHIPPED (session 5)** | Wall-following groove around carapace tier 2 (`part_shell.py`) — 8.4 mm tall, ~2.4 deep, chamfered top, feed notch into the cavity at the az-33 web; powers from an I5 XT30 spare tap per the harness plan. (A circular groove missed the rocky wall at the webs — the pentagon factor strikes again; it follows the outline function now.) |
| B9 | Shell vent gills | **SHIPPED (session 5)** | Real gills on the real sectors: 3 angled slots per side through the tier-1 wall, over the leg bays, placed against the queried wall radius. |
| B10 | Per-leg numbering | **SHIPPED** | Deck v0.3: k+1 engraved dots at each station + north arrow (font-free, slicer-proof). Legs themselves stay identical (the whole point of radial symmetry) — the DECK carries identity. |
| B11 | Whisker mounts | **SHIPPED** | `part_smallwins.whisker_shoe` — I6 dovetail shoe, 2× Ø1.3 piano-wire bores at ±20°. Phase-3 toy, zero-cost now. |

**Order addendum implications** (fed into the Batch-3 sheet): M3 washers
(B3 ballast + the deck v0.4 shell-magnet strikes), no TPU fairing spool
needed (B1 negative), piano wire 0.8 mm (B11, hardware-store), magnets
Ø6×3 (B6/I3 — already on the connector addendum for the panel standard).

## Session-5b additions

| # | idea | verdict | evidence / where |
|---|---|---|---|
| B12 | Lidar mount on the hatch cap | **SPEC'D, blocked on the pull-vs-LD19 decision** | Cap variant with a puck seat + bolt-circle bosses + wire drop into the cavity. Every dim is VERIFY until a puck is in hand — params.yaml gets a `lidar:` block when it arrives; building it now would be guessing in plastic. |
| B13 | Femur/tube cable clips | **SHIPPED (5c)** | `part_clips.py`: snap tube_clip (80 % gap) + link_clip, both with 4×6 wire tunnels + zip slots. ID fine-tune rides the caliper regen. |
| B14 | Charging dock | **MECHANICAL SHIPPED (5c); electrical Batch-2** | `part_dock.py`: walk-on plate + funnel rails (±8 mm capture → ±0.8, the XT60 float's own tolerance), shin bumper, plug tower with VERIFY-height XT60 carrier; every margin printed (feet land on ground, plug meets the 20 mm-crouch mate plane within 2 mm). Robot-side port options spec'd in the docstring — decide with the BEC/harness. |
| B15 | Fingertip grip serrations (hand v0.3) | **QUEUED with the caliper rebuild** | Transverse V-grooves on the inner cone faces; zero-cost boolean once hand v0.3 regenerates. Deliberately NOT sculpted against nominal geometry — that would be printing guesses. |
| B16 | Camera-behind-gill bracket | **SPEC'D (VISION_PLAN.md)** | Pi Cam 3 wide on an internal bracket, lens through a gill-slot aperture — vision without breaking the eyeless canon face. Prints with the shell regen once a camera is in hand (dims VERIFY). |

## Session-6 additions

| # | idea | verdict | evidence / where |
|---|---|---|---|
| B17 | Printed ST3215 servo blanks | **SHIPPED (s6)** | `part_servo_blank.py` — exact-envelope PLA stand-ins so the leg chain assembles WEEKS before the servo order lands (the servos are the joints). Boolean-proven in all three cradles (coxa base / fork rails / knee carrier, 0.00 mm³ each); horn disc carries the true M2 BCD so couplers and femur_link bolt on. Retire on servo-arrival day. |
| B18 | Beckon / "come-here" gesture | **SHIPPED (s6, same day)** | `pebble_gestures.beckon` + `sim/run_beckon.py` (narrated video): arm up-out, three 80°-knee curls with the claw opening on each pull-in, synced to a rising `curious_question`. Lesson en route: position-waypoint poses near joint limits SATURATE silently — expressive poses are authored in JOINT space now (D035). Name-motif greeting still rides chord-speak v0.3. |
| B19 | Two-sided hip/knee brackets (rear idler) | **SHIPPED (D047, 2026-09-22)** — the STEP has the idler; femur plate B rides it at hip AND knee, the fork rides it at the yaw. Was: | D046 leaves hip and knee single-sided: all leg bending goes through the servo output shaft. VERIFY on servo arrival whether the ST3215 has a rear idler boss/hole; if yes, the femur link grows a second arm riding a bearing on it and the knee carrier likewise. |
| B20 | Case-screw servo retention | **SHIPPED (D047)** — `servo_mount.servo_cup`: four self-tappers into the rim holes measured on the STEP; lips + straps retired. Was: | D046 lips + straps are independent of the (unmeasured) case holes. Measure the ST3215 case screw pattern; replace strap tap bores with case screws through the rail walls. |
| B21 | Printability: cantilever rule | **SHIPPED (2026-09-17)** | `check_printability.py`: an overhang blob whose boundary mostly does not touch the layer below is a CANTILEVER (vs a bridge over a bore); ≥ 3 mm with a 'no support' plan is a hard failure. Regression: the v0.1 servo blank's horn disc (7 mm) fails, v0.2 passes. |

## Session-9 additions (2026-09-22 review, D047)

| # | idea | verdict | evidence / where |
|---|---|---|---|
| B22 | Servo swap without pulling the femur | **DEFERRED** | The knee cup's two horn-face rim screws sit under plate A's beam; a knee/hip servo swap = plate B (4) + coupler clamps (4) + plate A off, then the cup's 4 screws. `check_assembly` models the screws as driven before the femur goes on. Revisit only if swaps turn out frequent. |
| B23 | Horn hole radius + thread | **COUPON (batch 1)** | The STEP reads 14.0–15.6 BCD and RobotShop's kit list says M3 × 6; SO-ARM100 lore says M2. `coupon_yaw_hub` + `horn_coupler` carry radial slots and M3/M2+washer counterbores; one screw on the real horn settles `params.servo_st3215.horn_bcd` / `horn_screw_m`. |
| B24 | URDF masses from the CAD budget | **SHIPPED (2026-09-22)** | `generate_urdf.py` reads `sim/mass_budget.json` (was: session-2 constants → 3.09 vs 2.7 kg parity failure) and expands its own macro when xacro is not installed. |
| B25 | Plain foot before the iris hand | **RECOMMENDED** | The hand-as-foot (D005) is the most complex, least-tested subsystem and takes every ground strike. The I2 bayonet already accepts a plain TPU foot on the SEA stub; walk on that first, fit the hand as a tool later. |
| B26 | Heat-set inserts where screws are opened often | **OPTION** | D047 uses thread-forming M3 in PETG for plate B's four bosses and the tube pinch bolt. If plate B comes off more than twice, the Ø8 bosses take the Ø4.6 × 5.7 insert pocket already in `params.print`. |
| B27 | `deck_t` 4.0 in params vs 6.0 hard-coded in `iface.py` / `part_deck.py` | **OPEN** | One number, two values; the params one is dead. Unify before the deck reprint. |
| B28 | Fit ladder → params | **BLOCKING** | The ladder was printed weeks ago and nothing consumes it: `params.print` still holds the initial guesses. First item of `docs/PRINT_PLAN_2026-09-22.md`. |
| B29 | Shove model: half-sine at the shell rim, quoted in N·s and BW | **SHIPPED** (D048) | `sim/shove.py`; the playground `push`, the fallen demo and `shove_envelope.py` use it. The push-envelope experiment scripts keep the CoM pulse as decision records — re-run them under the new model only if a decision needs it (the BW invariant of D039 still holds). |
| B30 | Righter without the staircase | **NEGATIVE ×2, OPEN** (D048) | `recover1` is a 50 Hz staircase (73 % of tick moves pinned at 5 rad/s, ~10 reversals/s per joint — `sim/audit_righter.py`). Reward v3 (smoothness cost) + 3 rad/s: scratch 2/20, warm-from-recover1 4/20 vs 7/20 — smoother (58 %, 6.5/s, 2.1°/tick) but fewer handoffs at 3–5 M steps. Next candidates, in order: an action low-pass filter INSIDE the env so the policy trains on it; a side→back curriculum (no checkpoint rights from the back; the planted ramp does, which is what the D048 stall rule exploits); v3 at 10 M. The bar: ≥7/20 with the warm run's smoothness. |
| B32 | Bench bring-up through the cockpit | **OPEN** (D051 shipped the tooling, mock-verified only) | First real servo on the bus: `rocky.sh cockpit` → Hardware → connect the adapter port → scan → set ID one servo at a time → center each joint at the jig → dir test with jog → `robot → sim` to confirm the IK frame → `sim → robot` with the stream cap at 200 c/s for the first stance. Then: does the real leg track a gesture at the servo model's numbers (turn the model on, compare)? Record the measured no-load slew and latency into `servo_model.DEFAULTS`. Later: hands (SCS0009) in the bridge, a per-leg contact switch feeding `Playground._probe` so the real foot probes like the sim foot. |
| B31 | Cockpit follow-ups | **MOSTLY SHIPPED** (D050: contact-seeking stance, map, recording/replay, saved + random worlds, walker, voice, phone layout; open: websocket transport, climbing a step taller than the swing height) | Websocket instead of MJPEG+SSE if the LAN case matters (phone over tailscale works as is, just heavier); a top-down map with the lidar scan and the goto target; recording from the cockpit (`record on` already works in the console, the clip lands in sim/); a voice button (whisper-server) in the chat; save/load world specs as JSON files; the `walk` residual policy as a selectable gait (needs a PebbleEnv obs builder in the loop). |
