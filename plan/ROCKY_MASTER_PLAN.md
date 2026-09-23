# Project ROCKY — Radial Pentapod Robot
## Master Plan v1.1 · July 2026

**The dream:** a fully autonomous, German-shepherd-sized, radially symmetric pentapod inspired by Rocky from *Project Hail Mary* — five identical limbs at 72°, a carapace body, no front or back, limbs that end in three-fingered hands that fold into feet, driven by local multimodal AI.

**The strategy:** we get there by building a ~1:3 subscale prototype first — codename **Pebble** — that proves every hard subsystem (radial gait, transforming hands, ROS 2 control stack, sim-to-real, AI integration) for under $1,000, before a single expensive actuator is bought. Almost everything built for Pebble (URDF topology, gait engine, firmware protocol, AI layer) transfers to the full-scale robot unchanged; only the actuators and structure scale up.

---

## 1. Why subscale first: the scaling math

Legged robot cost is dominated by actuators, and actuator torque requirements scale brutally. Mass grows with length cubed (L³) and lever arms grow with L, so **joint torque grows roughly with L⁴**. Pebble at ~1:3 scale needs ~1 N·m hip joints ($17 servos). A GSD-sized Rocky needs on the order of 80× that — 20–40+ N·m quasi-direct-drive (QDD) actuators at $300–600 each, × 15 joints. That's a $5,000–9,000 actuator bill *alone*, before structure, battery, compute, and the mistakes every first build makes.

Every crashed prototype, stripped gear, and gait bug costs ~$17 on Pebble and ~$500 on Rocky. We make our mistakes at Pebble prices.

**Verified physics for Pebble (computed, not vibes):**

| Quantity | Value | Margin |
|---|---|---|
| Total mass (incl. 20% payload margin) | ~2.7 kg | budget table below |
| Hip (femur) torque, wave gait (4-leg stance) | 0.64–0.96 N·m | 22–32% of stall — comfortable |
| Hip torque, 3-leg stance (2 limbs raised as arms) | 0.85–1.06 N·m | OK **if feet kept tucked** (moment arm ≤ 10 cm) |
| Knee (tibia) torque | 0.4–0.85 N·m | 14–29% of stall |
| ST3215 stall torque @ 12 V | 2.94 N·m | continuous-safe band ≈ 0.9–1.2 N·m |
| Average walking power | ~60 W | peaks 3–4× |
| Runtime on 3S 5200 mAh | ~45 min | 80% depth of discharge |

**Critical consequence:** we must run the **12 V variant** of the ST3215 (30 kg·cm) on a 3S pack. The 7.4 V variant (19.5 kg·cm) does *not* close the torque budget — its continuous band is ~0.6–0.76 N·m, below our 3-leg-stance loads. (This is one of several places the earlier Gemini brainstorm needed correcting.)

---

## 2. Phase roadmap

Each phase has a **gate** — a demonstrable result — before money is spent on the next.

| Phase | Name | Duration | New spend | Gate to pass |
|---|---|---|---|---|
| 0 | Foundations | Wks 1–2 | ~$25 (TPU only — hot ends/nozzles & PLA/PETG stock owned) | Prusa reassembled, prints a calibration part cleanly; URDF pentapod walks in simulation |
| 1 | One True Leg | Wks 2–5 | ~$208 | Physical leg on a bench jig tracks foot trajectories from ROS 2; spring (SEA) shin survives impact testing; hand prototype opens/closes |
| 2 | Pebble walks | Wks 5–10 | ~$627 (+$12 optional wattmeter) | Full pentapod stands, wave-gait walks omnidirectionally, teleop via your Xbox Elite controller |
| 3 | Pebble senses | Wks 10–16 | ~$86 (+$227 optional depth cam/lidar) | IMU-stabilized body, foot-contact terrain adaptation, camera streaming; optional: RL gait trained in sim runs on hardware |
| 4 | Pebble thinks | Wks 16–24 | ~$29 | Laptop VLM agent commands robot skills by voice/vision ("go look at that", "pick that up"); chord-speak voice |
| 5 | Full-scale gate | — | decision | Design review: everything learned → Rocky full-scale spec + budget; go/no-go/fundraise |

Cumulative spend through Phase 4: **≈ $975 core / ≈ $1,276 with every optional** (v1.1: credits your owned hot ends/nozzles, filament stock, and Xbox Elite controller) — $224+ of headroom under the $1,500 envelope, with trim levers (spreadsheet Summary tab) that pull core near ~$900. Timeline assumes hobby pace (evenings/weekends); compress or stretch freely.

**Leg-cost amortization (a common worry, defused):** Batch 1's ~$208 is *not* the per-leg price. It buys leg #1's actuators **plus** all the one-time shared kit — bench PSU, bus adapter, USB-UART dongle, fastener/bearing/insert stock, spring assortment. The marginal cost of every additional leg is just its actuators: 3× ST3215 + 1× SCS0009 ≈ **$70–75/leg**, and all four remaining legs are already inside Batch 2's total.

---

## 3. Mechanical design

### 3.1 Body plan — true radial symmetry
- Pentagonal carapace, circumradius ~110 mm, two printed decks (electronics sandwich) + a domed top shell with Rocky's ridged, rocky texture (cosmetic panels can come last).
- Five identical leg modules bolted at 72° stations. **One leg CAD design, printed five times** — this is the manufacturing payoff of radial symmetry.
- No front. "Heading" is a software variable. The carapace gets a 360° sensor ring later (Phase 3): camera on a small pan turret and/or low-cost lidar puck center-top.
- Standing height ~150–180 mm, walking footprint ~Ø450 mm, leg-tip max span ~770 mm. Cat-sized, GSD-proportioned.

### 3.2 Leg — 3 DOF, insect-style (matches the movie image)
| Joint | Motion | Actuator | Notes |
|---|---|---|---|
| Coxa | yaw (swing) | ST3215 | Vertical axis in a bearing-supported clevis — takes bending loads off the servo shaft (683ZZ or MR128 bearings top & bottom) |
| Femur | pitch (lift) | ST3215 | Highest load; aluminum servo horn, short lever geometry, heat-sink bracket |
| Tibia | pitch (extend) | ST3215 | Drives a carbon-fiber tube shin |

- **Series-elastic shin:** the foot slides ~6–8 mm on a spring inside the carbon tube (die spring or stiff compression spring). Absorbs footfall shock, protects gears, and — bonus — spring compression + a microswitch = a free, robust **foot contact sensor**. This survives from the Gemini doc because it's a good idea.
- The movie robot's limbs read as "coxa out, femur up-and-over, long tapering tibia" — this geometry matches. Cosmetic shells over the linkage give the rocky, organic look without affecting kinematics.

### 3.3 The transforming three-fingered hand (your signature feature)
Each tibia ends in a **stowable 3-finger claw**, exactly like the movie still:
- **Walking mode:** three fingers fold together into a closed cone — the cone tip *is* the foot, with a TPU pad. Loads pass through the closed structure and the spring shin, **not** through the finger servo.
- **Hand mode:** leg lifts, wrist-less claw opens — one **SCS0009 micro bus servo** per hand drives all three fingers via a printed spiral-cam or bevel-ring linkage (like a drill chuck / iris). 120° finger spacing continues the radial-symmetry theme.
- **Wiring is trivial** because the SCS0009 sits on the *same serial bus*: a short 3-wire jumper from the tibia servo down the shin. IDs 16–20. (This daisy-chain point from the Gemini doc was correct and we keep it.)
- Subscale grip strength is modest (2.3 kg·cm servo) — enough to pick up light objects, hold tools, gesture. Full-scale Rocky gets real grippers.
- A pentapod bonus from the literature: radially symmetric multi-legged robots can *statically stand on 3 legs while 2 limbs manipulate* — Rocky's canonical "walk on some legs, work with others" behavior. Our 3-leg-stance torque math above is exactly this case, and it closes.

### 3.4 Materials & manufacturing
Filament strategy given current stock (plenty of PLA, carbon-fiber-filled PLA, some purple PETG, no TPU):
- **PLA (owned):** jigs, the Phase-1 bench fixture, gait-test mule parts, cosmetic shells, and any bracket you expect to revise twice anyway. Free iteration. Its weakness is **creep** — it slowly deforms under sustained load, especially warm — so keep it away from servo pockets on the final walking build (ST3215 cases reach 50–65 °C working hard).
- **CF-PLA (owned):** excellent for the *link* parts — femur plates, tibia clamps — much stiffer and more creep-resistant than plain PLA, prints dead flat. Use the hardened nozzle you already have (CF eats brass). Avoid it for thin snap-fits and living hinges; the fibers make it brittle in thin walls.
- **PETG (purple, owned):** reserve it for the heat-adjacent, permanently-loaded parts — servo pockets, coxa clevis, carapace decks. A purple Pebble has excellent energy. Restock only when it runs out.
- **TPU (buy, ~$25):** foot pads, bumpers, cable grommets. The only filament actually missing.
- General: 0.2 mm layers, 4+ perimeters, 30–50% gyroid for structural parts; **carbon tube** shins (8–10 mm OD), music-wire or die springs.
- Everything in Phases 0–4 prints on the Prusa i3. **Xometry is not needed at subscale** — save it for full-scale Rocky (CNC 7075 brackets, SLS nylon gear housings) or for a one-off part your printer can't do (they quote instantly from STEP files).
- Design rule: servo-horn interfaces get printed *replaceable sacrificial couplers* — crashes strip a $0.30 part, not a $17 servo.

### 3.5 Prusa i3 refurb checklist (Phase 0, do first)
1. Reassemble the hot end from your spares stock (you have hot ends + nozzles already — including hardened for CF-PLA). Run brass 0.4 mm default; a 0.6 mm speeds up big structural parts ~40%.
2. While it's apart: inspect thermistor + heater cartridge leads and connectors (replace only if crusty, ~$10).
3. Fresh PTFE tube cut square; check extruder idler tension and clean drive gear.
4. Re-tension X/Y belts (twang test or Prusa belt-status screen), check Z-couplers.
5. PID-tune hot end and bed; recalibrate first layer (Live-Z) on the actual PETG you'll print.
6. E-steps + flow calibration cube; then print one leg bracket as the acceptance test.
7. PETG settings starting point: 240 °C / bed 85 °C / fan 30–50% / brim on structural parts.

---

## 4. Electrical architecture

```
 3S LiPo 5200 mAh (11.1 V nom, XT60)
   ├── main fuse (15 A) ── power switch ── E-stop-capable relay
   ├──► Servo power rail 12 V class ──► Bus Servo Adapter (A) ──► TTL bus
   │        (direct battery; ST3215 rated 6–12.6 V, 3S max 12.6 V ✓)
   │        Bus: 15× ST3215 (IDs 1–15) + 5× SCS0009 (IDs 16–20), daisy-chained per leg
   └──► 5 V / 5 A buck ──► Raspberry Pi 5 (8 GB) + camera + IMU (I²C)
```

- **One bus, no leg-node microcontrollers at this scale.** At 1 Mbps, sync-writing 15 positions + sync-reading telemetry at 50–100 Hz uses a fraction of bus bandwidth. The Gemini doc's per-leg ESP32 "spinal cord" is real full-scale architecture (and we'll use it on Rocky, likely CAN-based), but at Pebble scale it's five extra firmwares' worth of complexity for zero benefit. Start with the $7 Waveshare Bus Servo Adapter (A) on the Pi's UART.
- **Upgrade slot (optional, Phase 2+):** Waveshare's ESP32 servo-driver board (~$25) as a dedicated bus master — offloads real-time servo I/O, adds a hardware watchdog (servos go limp if the Pi hangs). Nice-to-have, not required to walk.
- IMU: BNO085 (on-chip sensor fusion → clean quaternion at 100 Hz over I²C).
- Camera: Pi Camera Module 3 Wide on a mini pan turret (Phase 3); optional OAK-D Lite / lidar later.
- Safety: inline fuse, physical kill switch on the carapace, software torque/temperature limits (the ST3215 reports load, temperature, and voltage — poll and enforce), LiPo charging in a bag, storage-charge between sessions.

**Compute note (2026 pricing reality):** Pi prices jumped twice this winter from the AI memory squeeze — Pi 5 8 GB is now $125, 16 GB $205, while the Jetson Orin Nano Super dev kit sits at $249. The Pi 5 8 GB is still the right Phase 2 buy (ROS 2 + control needs CPU, not CUDA, and the heavy AI runs on your laptop). But if you catch a Jetson in stock at $249 and feel spendy, it's a defensible substitution that buys onboard inference later. Revisit at Phase 4.

---

## 5. Software architecture (ROS 2, sim-first)

### 5.1 Stack
- **ROS 2 Jazzy** on Ubuntu 24.04 — on the Pi 5 (robot) and your 32 GB laptop (dev + AI). Same codebase runs in sim and on hardware.
- Monorepo layout:

```
rocky/
├── rocky_description/   # URDF/xacro: ONE leg macro, instantiated 5× at 72°; meshes
├── rocky_gait/          # gait engine: stance machine, Bezier swing trajectories, body pose ctrl
├── rocky_control/       # ros2_control hardware interface for the Feetech bus; joint state broadcaster
├── rocky_driver/        # low-level Feetech SDK wrapper (sync read/write, health monitoring)
├── rocky_teleop/        # gamepad/keyboard teleop, RViz config
├── rocky_sim/           # MuJoCo scene + Gazebo world; domain randomization configs
├── rocky_hands/         # claw open/close actions, foot⇄hand mode manager
├── rocky_ai/            # laptop-side: VLM agent, voice, skill server (Phase 4)
└── rocky_msgs/          # gait commands, body pose, hand actions, contact states
```

- Languages: Python for gait prototyping and AI glue; port the inner IK/gait loop to **C++ (or Rust via rclrs)** when you want tighter timing. Firmware (if/when the ESP32 bus master appears): C++ or embedded Rust — your call, both work.

### 5.2 Gait engine for a robot with no front
Radial symmetry makes locomotion *nicer*, not harder:
- Kinematics: closed-form 3-DOF IK per leg (standard insect-leg solution), identical for all five by symmetry.
- **Wave gait (4 down / 1 swinging):** the workhorse. Maximum stability margin, lowest torque, omnidirectional by construction — the "direction of travel" is just a continuous parameter θ ∈ [0, 2π). Strafing, rotating in place, and arcs all fall out of the same math.
- **Ripple/2-lift gaits** for speed once stable (pentapods support elegant 5-phase ripples — each leg 72° out of phase — literally the leg geometry mirrored in time; there's real research on 5-legged gait optimality to draw from).
- **Tripod-stance manipulation mode:** freeze 3 legs in a wide stance, unlock 2 adjacent limbs as arms. Statics verified above.
- Body pose control: 6-DOF carapace attitude (height, roll, pitch, yaw offset) layered on top of gait — this is what makes it feel *alive*.

### 5.3 Sim-first workflow (and your RL itch)
- **MuJoCo** as the primary sim: fast, excellent contacts, first-class RL ecosystem. URDF→MJCF conversion once, then train/test gaits headless at 1000× realtime.
- **Gazebo (Harmonic)** for ROS-integration testing (nav, sensors, plugins) when needed.
- Phase 0 deliverable: pentapod walking in MuJoCo with the hand-designed wave gait *before hardware exists*.
- Phase 3 stretch: train a PPO locomotion policy in MuJoCo (domain-randomized), distill/deploy on the Pi, compare against the analytic gait. Your 16 GB-VRAM laptop trains this easily — this is a legitimately publishable-hobby-tier sim-to-real project, and it's the on-ramp to full-scale Rocky's controller.

### 5.4 AI layer (Phase 4) — hybrid compute
- **Pi 5 (onboard):** ROS 2 control, gait, safety, camera/IMU streaming, a "skill server" exposing discrete actions: `walk(θ, v)`, `rotate(ω)`, `stance_manip()`, `grasp(x,y,z)`, `look_at()`, `gesture(name)`.
- **Laptop (offboard, 16 GB VRAM):** local multimodal AI — a quantized 7–8 B VLM (Qwen-VL-class) sees through the robot's camera; streaming Whisper for your voice; an agent loop translates intent → skill calls over ROS 2 (WiFi, or rosbridge/Zenoh if you roam networks). Fully local, no cloud.
- **Chord-speak 🎵:** Rocky talks in musical chords. Small I²S speaker on the carapace; map robot states and agent replies to chord progressions (acknowledge = major triad up, error = diminished, "found it" = resolving cadence...). Cheap, delightful, and *deeply* canon.
- Latency reality: anything reflexive (balance, contact response, E-stop) lives on the Pi; the laptop only issues high-level intents at 1–5 Hz. WiFi dropouts then degrade gracefully (robot stops thinking, never stops balancing).

---

## 6. Shopping list summary (details in BOM spreadsheet)

| Batch | When | Contents | ~Cost |
|---|---|---|---|
| 0 | now | TPU filament (rest of stock owned); optional: PETG restock, calipers | $25 core (+$62 opt.) |
| 1 | Phase 1 | 4× ST3215 12 V, bus adapter, USB-UART dongle, bench PSU, springs, carbon tube, fasteners/bearings/inserts, 1× SCS0009 | ~$208 |
| 2 | Phase 2 | 11× more ST3215 (+2 spares), Pi 5 8 GB + SD + cooler, 2× 3S 5200 LiPo + charger + bag, buck converter, wiring, XT60/fuses/switch, 4× more SCS0009; optional wattmeter | ~$627 (+$12 opt.) |
| 3 | Phase 3 | BNO085 IMU, Pi Camera 3 Wide + pan servo, foot microswitches; (optional) OAK-D Lite + LD19-class lidar | $86 core, +$227 optional |
| 4 | Phase 4 | I²S speaker/amp, USB mic | ~$29 |

Running total, core path: **≈ $975**. With every optional: **≈ $1,276**. Trim levers on the spreadsheet's Summary tab (Waveshare-direct or AliExpress servos, skip spares, one battery to start, used-lidar hack below) pull the core near ~$900. Teleop uses your Xbox Elite controller (works with the ROS 2 `joy` node over USB or Bluetooth, zero config beyond pairing).

### 6.1 Vendors, deals, and used-gear strategy

**Servos (the big line item — 17× ST3215 + 6× SCS0009):**
- [Waveshare direct](https://www.waveshare.com/st3215-servo.htm) — $16.99–21.99, the reference source; frequent multi-unit discounts.
- AliExpress **official FEETECH store** — often $14–16/unit in multi-packs; 2–4 week shipping. **Trap to avoid:** listings mix the 7.4 V (19.5 kg·cm) and 12 V (30 kg·cm) variants — only the 12 V closes our torque budget. Confirm "30KG" + 12 V before ordering, and order a 4-pack first to verify authenticity before committing to 13 more.
- Amazon ([Waveshare storefront](https://www.amazon.com/waveshare-Precision-Programmable-Magnetic-Switchable/dp/B0CFY52HVV)) — $20–26, next-day; use for "I need one more *now*" moments, not the bulk buy.
- Seeed / RobotShop — legitimate alternates when others are out of stock.

**Raspberry Pi (pricing is volatile in 2026 — the memory squeeze):**
- Check [rpilocator.com](https://rpilocator.com) for live stock at *authorized* resellers (PiShop.us, CanaKit, Adafruit, Vilros, Micro Center) at official pricing — marketplace listings often run $20–40 over.
- **Micro Center** in-store (Brooklyn/Westbury/Yonkers if you're near NYC) regularly has Pis at list plus open-box deals on SBCs, PSUs, and filament.

**Used/refurb — where it's smart:**
- **eBay / Amazon Warehouse (open-box):** bench PSUs, LiPo chargers, OAK-D cameras, calipers, Prusa spares. Robotics gear gets abandoned constantly; 30–50% off is routine.
- **The robot-vacuum lidar hack:** used/pulled vacuum lidars (Neato XV, Xiaomi LDS02RR, Roborock units) go for **$15–30** on eBay and have open-source ROS drivers — a legitimate budget 360° lidar for Phase 3 in place of the $99 line item.
- **University surplus / GovDeals / ham swap meets:** lab bench supplies, oscilloscopes, tools for pennies.
- **Never buy used LiPo batteries.** Unknown history = fire risk. Batteries and chargers-with-frayed-leads are the two categories where used is a false economy; buy those new.

**Mechanical odds and ends:**
- McMaster-Carr — exact-spec die springs, shoulder screws, inserts, next-day, superb CAD models for every part (import straight into your assemblies); costs more, worth it when a dimension matters.
- AliExpress assortment kits — springs, bearings, carbon tube, fastener refills when you can wait.
- GoBilda / ServoCity / Pololu — quality brackets, hubs, and electronics when a printed part isn't the right answer.

---

## 7. Risks & mitigations

| Risk | Mitigation |
|---|---|
| Femur servos overheat in long stands | 12 V variant only; tucked stance (arm ≤10 cm); poll servo temp registers and auto-sit at 65 °C; aluminum horn/bracket as heatsink; "sleep pose" (belly down, torque off) when idle |
| Stripped gears on crashes | Sacrificial printed couplers; SEA shins; torque limits in driver; buy 2 spare servos in Batch 2 |
| Single-bus wiring fault takes down all legs | Per-leg connectorized drops off a star-ish harness; fuse; kill switch; servo health monitoring with auto-limp |
| Old Prusa can't hold tolerances | Phase 0 acceptance test part; design with 0.3 mm clearances + reamed bearing seats; 0.6 mm nozzle for strength over precision where possible |
| Pi availability/price swings (2026 memory squeeze) | Pi 5 8 GB suffices; Jetson Orin Nano Super $249 is the fallback/upgrade; dev happens on laptop regardless |
| Scope creep (the disease of dream projects) | Phase gates. Nothing from Phase N+1 gets bought until Phase N's gate demo exists. The plan document is the contract with yourself. |
| LiPo mishap | Charge in bag, never unattended; storage charge; low-voltage alarm/telemetry cutoff at 3.5 V/cell |

---

## 8. The full-scale gate (Phase 5 preview)

What Pebble's success unlocks — decided *then*, with data, not now:
- ~25–35 kg, ~0.55–0.6 m shoulder height, 5× QDD legs (CubeMars AK60/AK70- or MyActuator RMD-X-class, 15 joints, $4.5–8 k), steel/7075 structure with printed shells (this is where Xometry earns its keep), Jetson Orin NX/AGX onboard, 6S–12S LiFePO4, real grippers.
- Realistic all-in: **$8–15 k** and a year+ of builds. The controller, gait engine, URDF topology, hand mechanism concept, AI agent, and — most valuably — *your accumulated judgment* transfer directly from Pebble.
- Alternative branch if budget stays tight: a 2× Pebble (~8–10 kg, heavy serial servos or entry QDD like $60–90-class units) as Rocky Jr. — the math and vendor landscape will be reassessed at the gate.

---

## 9. This week (concrete, in order)

1. Order Batch 0 + Batch 1 (hot end, filament, 4 servos, bus adapter, PSU/LiPo, springs, carbon tube, fastener kit).
2. Refurb the Prusa (checklist §3.5) while parts ship.
3. Dev env on the 32 GB laptop: Ubuntu 24.04 (dual-boot or VM) → ROS 2 Jazzy → MuJoCo. `rocky_description` first: one parametric leg xacro, five instances at 72°.
4. Get the URDF standing in MuJoCo; implement IK + wave gait in Python; make it walk in sim.
5. CAD the leg (Onshape/Fusion): coxa clevis with bearings, femur with replaceable coupler, tibia tube mount, spring foot. Print acceptance part.
6. When servos arrive: bench-drive one servo from Python on the laptop (bus adapter on USB-UART) before any assembly.

*Then* Phase 1 properly: one true leg on a jig, tracking sim trajectories.

---

## Appendix A — CAD plan (Pebble parts tree)

**Tool choice.** Two good paths, pick by taste:
- **Onshape** (free tier, browser, proper parametric assemblies, excellent for the linkage work) — recommended if you want visual CAD.
- **Code-CAD: build123d or CadQuery (Python)** — parts defined as code, dimensions as variables, versioned in git next to the URDF so CAD and simulation *share one parameter file*. Fits how you think, and it means **I can write and iterate the actual part scripts with you in these sessions and hand you STEP/STL files directly**. Realistic split: code-CAD for the geometric/bracketry parts, Onshape (or quick sculpts) for the organic carapace shells.

**Design-parameter single source of truth** (`params.yaml`, consumed by both CAD scripts and the URDF xacro):
leg segment lengths (45/95/135 mm), body circumradius (110 mm), servo pocket dims (ST3215 body 45.2×24×32 mm class — verify against the unit in hand before printing five of anything), bearing seats (683ZZ 3×7×3 mm), tube OD, spring travel (7 mm), fastener sizes (M2/M2.5/M3).

**Parts tree** (per-leg parts ×5; print order ≈ risk order — hardest first):

| # | Part | Qty | Key features / interfaces | Difficulty |
|---|---|---|---|---|
| P1 | Coxa clevis + yaw base | 5 | Servo pocket, 2× 683ZZ bearing seats (top/bottom shaft support), harness pass-through | ★★★ start here |
| P2 | Femur link | 5 | Twin-plate box beam; replaceable sacrificial horn coupler; heat-sink bracket face | ★★ |
| P3 | Tibia servo mount + tube clamp | 5 | Servo pocket, carbon-tube clamp boss, cable channel to hand | ★★ |
| P4 | SEA foot cartridge | 5 | Spring bore, 7 mm slider, microswitch pocket, TPU pad seat | ★★★ |
| P5 | Hand: iris/cam hub + 3 fingers + SCS0009 mount | 5 | The showpiece. Drill-chuck-style spiral cam closing 3 fingers into a foot cone; fingers are identical parts ×3 | ★★★★ prototype standalone in Phase 1 |
| B1 | Carapace lower deck | 1 | 5× leg stations at 72°, battery bay (strap slots), fuse/switch pockets | ★★ |
| B2 | Carapace upper deck | 1 | Pi + bus adapter + IMU standoffs, harness routing | ★ |
| B3 | Top shell (cosmetic, ridged Rocky texture) | 1 | Snap-fit, split into printable sectors; do last | ★ |
| B4 | Sensor turret mount | 1 | Phase 3 | ★ |

**Design rules:** 0.3 mm clearance on all mating printed parts (old-Prusa-friendly); no printed part takes servo-shaft bending loads (bearings do); every servo horn interfaces through a replaceable coupler; orient prints so layer lines are perpendicular to bending loads; heat-set M3 inserts for anything opened more than twice.

**Sequence:** P5 hand prototype and P1 coxa first as standalone Phase 1 prints (highest risk, most learning), P2–P4 with the bench leg, B1–B2 only after the leg passes its gate, B3 shell last (pure cosmetics). Expect 2–3 revisions of P1 and P5 — that's normal and is the point of subscale.

---

*Doc history: v1.0 — initial master plan, replaces/supersedes the earlier "Biomimetic Hybrid Leg" Gemini brainstorm (its SEA shin, daisy-chained hand servo, and full-scale hybrid-actuation instinct survive; its servo sizing, per-leg microcontrollers-at-subscale, and static-hold claims were corrected against computed torque/thermal budgets). v1.1 — credited owned gear (hot ends/nozzles, PLA + CF-PLA + purple PETG, Xbox Elite controller), added filament strategy by material, leg-cost amortization note, and §6.1 vendors/deals/used-gear; core path $1,051 → $975.*
