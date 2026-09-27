# Interfaces: Pebble's six standard interfaces (D020)

*The modularity contract. Every part mates through one of these six
interfaces, or has a written reason not to. The dimensions are frozen in
`cad/params.yaml` under `interfaces:`. CAD scripts read them from there,
through the helpers in `cad/iface.py`, so a number changes in one place.*

**The one structural rule: loads route through dowels, lips and shoulders,
never through latches, thumbscrews or magnets.** Fasteners and latches only
*retain*; positive geometry *carries*. A stripped latch or a loose
thumbscrew must never be able to become a structural failure, only a
rattle.

Print fit: mating clearances inherit `print.clearance_fit` (0.30), which the
CAD scripts apply at generation time; each frozen dimension below is
nominal. `print.clearance_press` (0.15) is recorded for press fits, but no
script reads it yet (press seats are reamed by hand). VERIFY items are
measured on real prints (the fit ladder, [PRINT_PLAN.md](PRINT_PLAN.md)
batch 0, and the [interface coupons](PRINT_PLAN.md#interface-coupons-pla-02-mm-with-batch-1))
and updated in params, never in part code.

---

## I1 · Leg port (deck ⇄ coxa base): the flagship

One leg is one field-replaceable module. The target is a leg swap in
**under two minutes** with one tool-free sequence: *hook, pivot, two
thumbscrews, plug two connectors.*

**Mechanical**
- **Hook-pivot:** the coxa base plate's INBOARD edge carries a downturned
  lip. Its foot drops through a through-slot in the deck and hooks under the
  deck's bottom face. The hook lives inboard because that is where the deck
  is: the deck ends at R100 and the plate cantilevers 25 mm past it (D012),
  so there is nothing outboard to catch. To insert: present the plate at
  ~15°, drop the lip through the slot, slide inboard to hook, then pivot
  flat onto the dowels.
- **Dowels:** 2 × Ø4 × 8 mm engagement dowels at leg-local (−28, ±19),
  printed-in deck posts at Pebble scale and steel pins at Rocky scale. They
  sit on the deck (radius 84.2 against the pentagon boundary of 88.4
  there) and clear of the cradle walls. The dowels take ALL the shear:
  walking loads, yaw torque reaction, shoves. They need no hand access, so
  they can live under the servo.
- **Captive thumbscrews:** 2 × M3 × 10 with printed knurled heads at the
  inboard corners (−41, ±17). They are reachable from above WITH the servo
  installed, retained in the plate by printed lips, and thread into deck
  heat-set inserts. They only clamp down and carry no shear.
- **Load path:** vertical forces go plate face onto deck face; shear and
  torque go into the dowels and the hook lip; tension (leg hanging
  upside-down) goes into the thumbscrews and the hook.

**Electrical (per leg drop; [WIRING_HARNESS.md](WIRING_HARNESS.md))**
- **XT30 pair:** 12 V servo power for the leg's 3 × ST3215. At the coxa it
  fans out into one short lead per servo. The servo connector pins are
  rated ~2 A, so the leg's current never goes through them.
- **JST-XH 5-pin:** 1 TTL data, 2 GND, 3 6 V hand-servo power (never 12 V,
  D016), 4 the foot-switch line (the SEA microswitch, D010, to its own Pi
  GPIO), 5 reserved. It is frozen at 5 conductors so the spares exist from
  day one.
- **Where the connectors mate:** both live in the deck's 11 × 11 mm cable
  pass-through at leg-local x = −29 (`iface.CABLE_CUTOUT_X` /
  `CABLE_CUTOUT_W`, r = 81 on the deck). You reach them with the plate
  hooked but not pivoted. They are not blind-mate at this scale (plugging
  by hand is fine); Rocky goes blind-mate.
- **Route through the coxa:** the yaw servo sits over the cutout, so
  `coxa_yaw_base` carries an 11 × 11 channel (`part_coxa.harness_path()`).
  It runs up out of the cutout and forward under the gearbox plateau
  (x −34.5 to −13, z −4 to 7), then sideways to −Y under the cup's side
  wall, up beside the cup (y −26.9 to −15.9), and over the top to the yaw
  servo's plugs (z 57.4 to 68.4). It uses the −Y side because the hip cup
  and its plugs sweep +Y at yaw +40. `check_assembly` asserts the path is
  clear of the base, the yaw blank and the whole yawing leg over the
  params yaw range.

**Rocky-scale mapping:** dowels become Ø10 hardened steel; the hook a
machined 7075 lip; the thumbscrews cam-lever DIN quick releases; XT30
becomes XT90 or Amphenol SurLok; JST-XH becomes M8 circular; add a
leg-present sense pin.

---

## I2 · Tool socket (SEA stub ⇄ hand / future tools)

The hand is the first *tool*, not a permanent part. The SEA slider's
tube-OD stub (D010) carries a **quarter-turn bayonet**:

- **Stub:** Ø10 (the tube OD) with 2 × radial lugs, Ø2.5 × 2.2 proud,
  180° apart, 9 mm from the stub face.
- **Socket** (in the hand hub): L-slots cut through the boss wall, with a
  6 mm axial entry, a 90° twist and a 0.6 mm detent before the seat.
- **Load path:** walking compression goes from the stub FACE to the socket
  SHOULDER (full-ring contact); the lugs only resist pull-off and torsion.
  Grip cable tension is < 15 N, far below the lugs' shear capacity.
- **Electrical:** a **JST-SH 3-pin** pigtail (6 V, GND, TTL data) exits the
  tube above the socket, with a service loop long enough for the 90° twist.
- **Anti-rotation:** the detent plus the JST pigtail. If the bench shows
  creep-walk, add an M2 set-screw boss (not modelled yet).

**Rocky-scale:** a powered tool flange (MOD-style), pogo-ring contacts and a
tool-ID resistor on a sense pin.

---

## I3 · Panel standard (all shell / carapace panels)

Panels are cosmetics and access. They must come off in seconds, survive
being removed hundreds of times, and never carry structure.

- **Location:** every panel seats on printed lips or bosses (0.3 fit),
  which take all the impact and handling loads.
- **Retention: a quarter-turn cam latch as a REPLACEABLE INSERT.** It is a
  printed two-part cartridge: rotor plus a Ø14 housing, glued or
  press-fitted into a Ø14.3 pocket. The wear part is the ~1 g insert, not
  the panel. The rotor has a coin/fingernail slot and turns 90° from OPEN
  to CAM-TIGHT (0.8 mm cam rise).
- **Magnets:** Ø6 × 3 glue-in pockets (N35+) wherever a soft-close seat is
  wanted. Polarity convention: the panel-side magnet has NORTH facing
  outward, always, so any panel snaps onto any frame station.
- **As built:**
  - the belly door (`part_battery.belly_door`): a perimeter seating lip,
    2 latch inserts and 2 magnets;
  - the carapace sectors (`part_shell`): each sector foot has one latch
    (az +27.5°, r 74.5) and one magnet (az −25.5°, r 76), mating the deck's
    latch strikes and washer recesses (D030);
  - the top hatch (`shell_cap`): five plug magnets on five seat magnets,
    one per sector. It has no latch.

**Rocky-scale:** the same cartridge in glass-filled nylon, Ø22, with a
captive lanyard so field panels don't drop.

---

## I4 · Avionics tray

Everything electronic on the robot sits on ONE slide-out tray
(`part_avionics`): the Pi 5 with its cooler, the Bus Servo Adapter (A),
the 5 V buck for the Pi, the 6 V UBEC and the BNO085 IMU.
The ESP32 Servo Driver is a bench tool and does not ride on the tray
(`bom/BOM.csv` A-03).

- **Tray plate:** 84 × 70 × 3, with 3 × 3 side wings captive in two
  deck-bolted C-channel rail blocks (`tray_rail`). The rails carry the
  load.
- **Single latch:** an I3 insert in the tongue at the tray front; it only
  retains.
- **Bulkhead** at the tray rear (70 × 26 face): a panel wall carrying every
  connection that crosses the tray boundary. That is the XT30 power in, the
  JST-XH 5-pin trunk to the star board, 2 × JST-SH spares, a Qwiic/JST-SH
  4-pin I²C (the sensor mux, [PERCEPTION_PLAN.md](PERCEPTION_PLAN.md)) and
  a USB-C pass-through slot. Removing the tray means one latch plus
  unplugging the bulkhead face, with no reaching inside.
- **IMU:** sits on four TPU grommets (B4) on the tray's IMU pad and talks
  over SPI ([WIRING_HARNESS.md](WIRING_HARNESS.md#pi-5-pin-map-draft-verify)).
- **Deck position:** the tray's place on the deck, (0, −38) in the body
  frame, is still an assumption (`part_avionics.TRAY_XY`). The star-board
  bracket's layout audit keeps clear of it.

**Rocky-scale:** a 19" subrack-style card cage with locking DIN connectors.

---

## I5 · Battery sled + XT60 dock

- **Battery:** one 3S 5200 mAh LiPo in a SOFT case, at most 138 × 46 × 25
  (VERIFY on purchase; `bom/BOM.csv` B-05). Hard-case packs are ~37 mm tall
  and do not fit the 30 mm bay. The pack straps to a printed **sled**, which
  slides on belly rails into the 175 × 50 × 30 bay.
- **XT60 blind-mate dock:** the sled nose carries a panel-mount XT60E-M
  (body 16.2 × 8.6 × 16.0, `xt60_panel_mm`). The bay's fixed XT60 sits in a
  floating pocket (±0.8 mm) with lead-in chamfers, so you push to seat.
  The belly door (I3) closes over the sled's tail lip and does the
  retaining; the connector does not.
- **Spare power taps:** the dock's bus bar breaks out 2 × XT30 pigtails
  (the B8 LED ring and later accessories); today they sit ahead of the
  15 A fuse, so fuse anything plugged in (B66).
- **Hot swap at Pebble scale:** limp the servos, swap the sled (~5 s), then
  reboot the Pi.

**Rocky-scale:** the sled becomes a drawer on HD slides and the XT60 an
SB120 / SurLok; add precharge and battery BMS CAN to the bulkhead.

---

## I6 · Accessory dovetail ring

One profile for everything that clips onto the carapace perimeter:

- **Male profile (on the shell):** a trapezoid dovetail, 12 wide at the
  base, 8 wide at the crest, 4 deep. As built, each sector web has one
  VERTICAL 16 mm segment at ±27°, giving 10 stations and no top station.
  A shoe slides down from above, gravity seats it and the set-knob locks
  it (D029). The params `segment_len` (24) is the spec length, used by the
  demo coupons (`part_panel`).
- **Female shoe (on the accessory):** a matching slot plus an M3 printed
  set-knob from below, which clamps into the dovetail groove.
- **Users:** sensor pods ([PERCEPTION_PLAN.md](PERCEPTION_PLAN.md)),
  whisker mounts (B11), LED ring segments and shell tool docks. The camera
  is fixed behind a gill (B16) and the lidar goes on a hatch-cap variant
  (B12); neither uses the ring.
- **Load rating:** ~15 N pull and ~0.4 N·m moment, estimated for 24 mm of
  PLA engagement. Bench it at the as-built 16 mm. Good for sensors, not for
  handles; the shell itself still carries nothing (D006).

**Rocky-scale:** an aluminium rail ring, deliberately a Picatinny cousin so
the tooling exists.

---

## Frozen dimensions

All are in `cad/params.yaml → interfaces:` (the I1 cable cutout is in
`cad/iface.py`). In summary:

| iface | key dims (mm) |
|---|---|
| I1 leg port | dowel Ø4 × 8 @ (−28, ±19); thumbscrew M3 @ (−41, ±17); inboard hook lip 24 wide through a 4.2 deck slot @ x = −48; cable cutout 11 × 11 @ x = −29; XT30 + JST-XH 5-pin |
| I2 tool socket | lug Ø2.5 × 2.2 @ 9 from the face; entry 6; twist 90°; detent 0.6; JST-SH 3-pin |
| I3 panel latch | insert housing Ø14 (pocket 14.3), rotor cam rise 0.8, 90° throw; magnet Ø6 × 3 |
| I4 avionics tray | plate 84 × 70 × 3; rail 3 × 3; bulkhead 70 × 26 face |
| I5 battery sled | bay 175 × 50 × 30 for a soft-case pack ≤ 138 × 46 × 25 (VERIFY vs the purchased pack); XT60E-M 16.2 × 8.6 × 16.0; XT60 float ±0.8 |
| I6 dovetail | base 12 / crest 8 / depth 4; segment 24 spec, 16 on the shell as built; M3 set-knob |

## Decision

Logged as **D020** in [decisions.md](decisions.md). Any single interface can
be revisited with bench evidence. The RULE (loads through geometry, never
latches) is not up for revision: it is what makes the whole robot
serviceable by a person with cold hands and no tray for screws.
