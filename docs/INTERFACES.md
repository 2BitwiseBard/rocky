# INTERFACES.md — Pebble's six standard interfaces (D020)

*The modularity contract. Every part designed after 2026-07-30 mates through
one of these six interfaces or has a written reason not to. Dimensions are
FROZEN in `cad/params.yaml` under `interfaces:` — CAD scripts read them from
there; changing a number means changing it there, once.*

**The one structural rule: loads route through dowels, lips, and shoulders —
never through latches, thumbscrews, or magnets.** Fasteners and latches only
*retain*; positive geometry *carries*. A stripped latch or a loose thumbscrew
must never be able to become a structural failure, only a rattle.

Print-fit note: all mating clearances inherit `print.clearance_fit` (0.30) /
`print.clearance_press` (0.15); every frozen dimension below is nominal —
the scripts apply clearances at generation time. VERIFY items get measured
on real prints during Phase 1 and updated in params (not in part code).

---

## I1 · Leg port (deck ⇄ coxa base) — the flagship

One leg = one field-replaceable module. Target: swap a leg in **under two
minutes** with one tool-free motion sequence: *hook, pivot, two thumbscrews,
plug two connectors.*

**Mechanical**
- **Hook-pivot:** the coxa base plate's INBOARD edge carries a downturned
  lip whose foot drops through a through-slot in the deck and hooks under
  the deck's bottom face (D012 reality check: the deck ends at R100 and the
  plate cantilevers 25 mm past it outboard — there is no deck material out
  there to catch, so the hook lives inboard where the deck actually is).
  Insertion: present the plate at ~15°, drop the lip through the slot,
  slide inboard to hook, pivot flat onto the dowels.
- **Dowels:** 2× Ø4 × 8 mm engagement dowels (printed-in deck posts v0;
  steel pins at Rocky scale) at leg-local (−28, ±19) — verified on-deck
  (radial 84.2 vs pentagon boundary 88.4 there) and clear of the cradle
  walls. Dowels take ALL shear: walking loads, yaw torque reaction, shoves.
  No hand access needed, so living under the servo's wing is fine.
- **Captive thumbscrews:** 2× M3 × 10 with printed knurled heads at the
  inboard corners (−41, ±17) — reachable from above WITH the servo
  installed (the v0.2 mid-pair stations were not: servo sits over them;
  they were also 2 mm off the deck pentagon — layout audit, BUILD_LOG
  07-30). Retained in the plate by printed lips (no screws in the grass);
  thread into deck heat-set inserts. Clamp-down only, zero shear duty.
- Load path: vertical forces → plate face on deck face; shear/torque →
  dowels + hook lip; tension (leg hanging upside-down) → thumbscrews + hook.

**Electrical (per leg drop, from the D016 harness)**
- **XT30 pair** — 12 V servo power (3× ST3215).
- **JST-XH 5-pin** — TTL data, GND, 6 V hand-servo power, +2 spare
  conductors (frozen: 5 conductors so the spares exist on day one; spares
  reserved for the SEA microswitch signal + one future sensor).
- Both connectors live in the deck's existing 11 mm cable pass-through,
  reached with the plate hooked-but-not-pivoted. Not blind-mate at this
  scale (hand-plug is fine); Rocky goes blind-mate.

**Rocky-scale mapping:** dowels → Ø10 hardened steel; hook → machined
7075 lip; thumbscrews → cam-lever DIN quick releases; XT30 → XT90 or
Amphenol SurLok; JST-XH → M8 circular; add a leg-present sense pin.

---

## I2 · Tool socket (SEA stub ⇄ hand / future tools)

The hand is the first *tool*, not a permanent part. The SEA slider's
tube-OD stub (D010) grows a **quarter-turn bayonet**:

- Stub: Ø10 (tube OD) with 2× radial lugs Ø2.5 × 2.2 proud, 180° apart,
  9 mm from the stub face.
- Socket (in hand hub, unchanged outer geometry): L-slots — 6 mm axial
  entry, 90° twist, 0.6 mm detent bump before the seat.
- **Load path:** walking compression goes stub FACE → socket SHOULDER
  (full-ring contact, exactly as the glued v0 did); the lugs only resist
  pull-off and torsion. Grip cable tension < 15 N ≪ lug shear capacity.
- Electrical: **JST-SH 3-pin** (6 V, GND, TTL data) pigtail exits the tube
  above the socket — service loop long enough to twist 90°.
- Anti-rotation: the detent + the JST pigtail; if bench shows creep-walk,
  a M2 set screw boss is pre-modeled (unused by default).

**Rocky-scale:** powered tool flange (MOD-style), pogo-ring contacts,
tool-ID resistor on a sense pin.

---

## I3 · Panel standard (all shell/carapace panels)

Panels = cosmetics + access. They must come off in seconds, survive being
taken off hundreds of times, and never carry structure.

- **Location:** every panel seats on printed lips/bosses (0.3 fit) that take
  all impact/handling loads.
- **Retention: quarter-turn cam latch as a REPLACEABLE INSERT** — a printed
  2-part cartridge (rotor + housing, Ø14 housing in a Ø14.3 pocket, glued or
  press-fit). Wear part = reprint the ~1 g insert, not the panel. Rotor has
  a coin/fingernail slot; 90° between OPEN and CAM-TIGHT (0.8 mm cam rise).
- **Magnets:** Ø6 × 3 pockets (glue-in, N35+) where a soft-close/tactile
  seat is wanted; polarity convention: panel-side magnet NORTH faces
  outward, always (so any panel prototype snaps onto any frame station).
- First two implementations: **belly battery door** (2 latches + 2 magnets +
  hinge-lip) and **top service hatch** (1 latch + 2 magnets).

**Rocky-scale:** same cartridge idea in glass-nylon, Ø22; add captive
lanyard so field panels don't drop.

---

## I4 · Avionics tray

Everything electronic on ONE slide-in tray: Pi 5, bus adapter, BEC, IMU.

- Tray plate 84 × 70 × 3 with side rails 3 × 3 engaging deck-mounted rail
  blocks; slides in from the top-hatch direction.
- **Single latch** (I3 insert) at the tray front; rear edge seats under a
  deck lip (load path rule again).
- **Bulkhead** at the tray rear: a panel-mount wall carrying every
  connection that crosses the tray boundary — XT30 (power in from dock),
  JST-XH (bus to harness star), 2× JST-SH (spare), I2C/Qwiic 4-pin
  (sensor mux — see SENSING_PLAN.md), USB-C pass-through slot. Removing the
  tray = one latch + unplugging the bulkhead face. No reaching inside.
- IMU hard-mounts to the tray on TPU grommets (backlog item B4) at the
  body center mark.

**Rocky-scale:** 19"-subrack-style card cage, locking DIN connectors.

---

## I5 · Battery sled + XT60 dock

- Battery (3S 5200 LiPo, ~138 × 46 × 25 VERIFY on purchase) strapped to a
  printed **sled** (strap slots reuse deck geometry); sled slides on belly
  rails into the bay.
- **XT60 blind-mate dock:** sled nose carries a panel-mount XT60; the bay's
  fixed XT60 sits in a floating pocket (±0.8 mm) with lead-in chamfers —
  push to seat. Retention: the belly door (I3) closes over the sled tail
  lip; the connector is NOT the retention.
- **Spare power taps:** the dock's bus bar breaks out 2× XT30 pigtails
  (future: LED ring, heated whatever, Phase-4 speaker amp).
- Hot-swap reality at Pebble scale: swap = limp servos, swap sled (5 s),
  reboot Pi (or add a small UPS cap board later — pocket reserved on the
  avionics tray).

**Rocky-scale:** sled → drawer on HD slides, XT60 → SB120 / SurLok, add
precharge + battery BMS CAN to the bulkhead.

---

## I6 · Accessory dovetail ring

One profile for everything that clips onto the carapace perimeter:

- **Male profile (on the shell):** trapezoid dovetail, 12 wide at base,
  8 wide at crest, 4 deep, running tangentially in 24 mm segments at
  10 stations (2 per shell sector × 5 sectors), plus 1 center-top station.
- **Female shoe (on the accessory):** matching slot + M3 printed-knob set
  screw from below (clamps INTO the dovetail groove, not against paint).
- Users: sensor pods (sonar, PIR, thermal — SENSING_PLAN.md), whisker
  mounts, LED ring segments, shell tool-docks (palm-magnet tool holsters),
  camera turret, future lidar mast.
- Load rating (PLA, 24 mm engagement): ~15 N pull, ~0.4 N·m moment —
  sensors yes, handles no. The shell itself still carries nothing (D006).

**Rocky-scale:** Picatinny-adjacent aluminum rail ring (it IS a picatinny
cousin — intentionally, tooling exists).

---

## Frozen dimensions

All in `cad/params.yaml → interfaces:` — single source of truth. Summary:

| iface | key dims (mm) |
|---|---|
| I1 leg port | dowel Ø4×8 @ (−28,±19); thumbscrew M3 @ (−41,±17); inboard hook lip 24 wide through deck slot @ x=−48; XT30 + JST-XH 5-pin |
| I2 tool socket | lug Ø2.5×2.2 @ 9 from face; entry 6; twist 90°; detent 0.6; JST-SH 3-pin |
| I3 panel latch | insert housing Ø14 (pocket 14.3), rotor cam rise 0.8, 90° throw; magnet Ø6×3 |
| I4 avionics tray | plate 84×70×3; rail 3×3; bulkhead 70×26 face |
| I5 battery sled | bay 142×50×30 (VERIFY vs purchased pack); XT60 float ±0.8 |
| I6 dovetail | base 12 / crest 8 / depth 4, segment 24; M3 set-knob |

## Decision

Logged as **D020** in `docs/decisions.md`. Revisit any single interface
with bench evidence; the RULE (loads through geometry, never latches) is
not up for revision — it's what makes the whole robot serviceable by a
person with cold hands and no tray for screws.
