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

> ⚠ **As drawn, this port cannot be docked (found 2026-10-01, backlog B117).**
> The hook's L is 5.15 mm wide at its foot against the 4.2 mm slot (4.6 fails
> too), and the 8 mm dowel posts block the pivot even with a 5.2–7.0 slot: no
> rigid-body path exists, so neither the port coupons nor the fit ladder's
> row D can rehearse it (B137). Three verified fixes are compared in
> [archive/prep-2026-10-01/I1_DOCK_OPTIONS.md](archive/prep-2026-10-01/I1_DOCK_OPTIONS.md);
> the recommended one ("seats": two 30° cone posts at (−32, ±17.3), slot 6.2 at
> x −47, a plate rib, insertion at 7–10° with a radial approach) changes three
> of the frozen values below and waits for the owner's sign-off (body-layout
> decision 15). Also wrong below: the **load path**. In stance the foot lifts
> the plate's outboard end, the plate pivots on its most inboard deck contact,
> the hook carries nothing, and the two thumbscrews with their inserts carry
> about 972 N per pair in the design case (568 N with the seats fix); the plate
> is at SF 0.74 at the screw line (B118, B119). A leg swap also needs its own
> carapace sector off, which takes a flat screwdriver from under the deck
> (B125), and the mated XH-5 pair does not pass the 11 × 11 cutout (B120).
> Nothing in the frozen values has been changed yet.

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
- **Thumbscrews:** 2 × M3 × 16 hex head (ISO 4017, `bom/BOM.csv` A-19),
  each keyed into a printed knurled knob (`thumb_knob_m3`), at the inboard
  corners (−41, ±17). They are reachable from above WITH the servo
  installed and thread into deck heat-set inserts, 5.5 mm into the 5.7 mm
  insert. The knob (8.6 tall since D063) holds the head 6.5 mm above the
  plate, the whole head inside its 2.1 mm hex pocket, so an M3 × 10 never
  reaches the insert. They are not captive yet (B82): nothing retains them
  in the plate, so they lift out with the leg. They only clamp down and
  carry no shear.
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
- **Retention: a quarter-turn bayonet latch as a REPLACEABLE INSERT**
  (D063, B87; `iface.py`). It is a printed two-part cartridge: a Ø14 × 6
  housing, glued into a Ø14.3 through-pocket with its bottom flush with the
  panel's seating face, and a rotor: a Ø6 shaft with a Ø10 head on top and
  two lugs at its bottom end, which reaches 5.8 mm below the seating face.
  The rotor drops through two keyways in the housing and, turned off them,
  is captive. The wear part is the ~1 g insert, not the panel.
- **Keyway index (B107):** the housing is a D, flat 6.0 mm from its axis
  at 90° to the keyways, and every pocket is the same D 0.15 mm bigger
  (`iface.latch_pocket`), so the housing goes in one way (±2.5° of play),
  its keyways 135° from the entry line. The rotor works 0–90° between the
  strike's stops, so no angle it is worked at, or comes off at (OPEN),
  lines its lugs up with the keyways: they pass only at 132–138°, 42° past
  LOCKED and 42° back past OPEN (`part_panel.latch_index`, run on the demo
  and on the shell sector's pad: one part, printed five times). Either face
  up is the same: the housing is symmetric top to bottom. As D063 left it
  (round, glued 45° off the entry line), the lugs passed at 42–48°,
  mid-turn. The index is a place, not a stop: off the robot nothing holds
  the rotor's 3.8 mm of axial play, and turned 45° back past OPEN it still
  drops out of its housing.
- **Strike (frame side, `iface.latch_strike`):** a Ø6.6 bore and two entry
  slots through a 3.2 mm land, at 45° to the frame's +x (radial on the
  deck), and an underside recess shaped to the lugs' quarter-turn path. Its
  ceiling is the cam: 0.8 mm of rise over 0–50°, a dwell to 90°, a stop.
  At LOCKED the lugs bite 0.2 mm into the land (`LATCH_BITE`, VERIFY in
  PLA, B99). The frame must be ≥ 6.0 mm thick (the function asserts it; the
  deck is 6, B27).
- **Operating end: the rotor's bottom,** a 2.0 × 1.5 slot 0.2 mm above the
  frame's underside, square to the lugs. From below, a flat screwdriver
  (blade ≤ 5 mm), a quarter turn clockwise to lock: it starts to bite at
  ~40° and stops at 90°. Not a coin: one reaches only ~0.5 mm of the slot
  before it meets the frame. The tray tongue and the belly door have no
  strike yet (B51, B84); their pockets take their own +x as the strike's
  for the index, and their strikes must be drawn to match.
- **Magnets:** Ø6 × 3 glue-in pockets (N35+) wherever a soft-close seat is
  wanted. Polarity convention: every magnet, panel side and frame side, has
  its NORTH face pointing away from the robot (the way the panel lifts
  off). At each joint the frame magnet shows N and the panel magnet shows
  S, so any panel snaps onto any frame station. Steel strikes take either
  face.
- **As built:**
  - the belly door (`part_battery.belly_door`): a perimeter seating lip,
    2 latch inserts and 2 magnets;
  - the carapace sectors (`part_shell`): each sector foot has one latch
    (az +27.5°, r 74.5) and one magnet (az −25.5°, r 76), mating the deck's
    latch strikes and washer recesses (D030). The latch is turned from under
    the deck; station 162's sits inside the battery bay's footprint, so the
    bay must leave that column open (B100);
  - the top hatch (`shell_cap`): five plug magnets on five seat magnets at
    r 42.5, one per sector, each seat in a walled boss under the seat ledge
    (D063, B88). It has no latch. Both magnets of a pair go in N up.

**Rocky-scale:** the same cartridge in glass-filled nylon, Ø22, with a
captive lanyard so field panels don't drop.

---

## I4 · Avionics tray

Everything electronic on the robot sits on ONE slide-out tray
(`part_avionics`): the Pi 5 with its cooler, the Bus Servo Adapter (A),
the 5 V buck for the Pi, the 6 V UBEC and the BNO085 IMU. That is
the intent: as drawn, the adapter, the buck and the UBEC have no place
under the Pi (measured 2026-09-30, B108 below; the owner's call).
The ESP32 Servo Driver is a bench tool and does not ride on the tray
(`bom/BOM.csv` A-03).

- **Tray plate:** 84 × 70 × 3, with 3 × 3 side wings captive in two
  deck-bolted C-channel rail blocks (`tray_rail`). The rails carry the
  load.
- **Single latch:** an I3 insert in the tongue at the tray front; it only
  retains. Nothing under the tongue has a strike yet (the rotor hangs 5.8
  mm below it; B51).
- **Bulkhead** at the tray rear (70 × 26 face): a panel wall carrying every
  connection that crosses the tray boundary. That is the XT30 power in, the
  JST-XH 5-pin trunk to the star board, 2 × JST-SH spares, a Qwiic/JST-SH
  4-pin I²C (the sensor mux, [PERCEPTION_PLAN.md](PERCEPTION_PLAN.md)) and
  a USB-C pass-through slot. Removing the tray means one latch plus
  unplugging the bulkhead face, with no reaching inside.
- **Pi 5:** on four thread-forming standoffs, 58 × 49 and 11 mm tall
  (D063, B89, B90), M2.5 × 6 or × 8.
- **IMU:** the BNO085 sits on four TPU grommets (B4) under the Pi, so it
  goes on before the Pi. The grommet seats are on the board's 20.32 ×
  17.78 holes about (0, 13.5), each lifted 1.2 mm over a recess for the
  grommet's bottom flange (D063, B90). Each M2.5 × 10 needs a standard nut
  (the grommet bore does not thread); the tail ends 0.9 mm above the deck,
  and the IMU's top 1.2 mm below the Pi's underside keep-out. It talks over
  SPI ([WIRING_HARNESS.md](WIRING_HARNESS.md#pi-5-pin-map-draft-verify)).
  The bus adapter, buck and UBEC are envelopes in `part_avionics` (B108,
  from their datasheets and Waveshare's STEP of the adapter: 42 × 33 with
  parts 11.0 mm over its board and pins 2.0 under it; 17.8 × 20.3 × 8.8;
  43 × 17 × 7), and none has a place on this tray: under the Pi's 2 mm
  keep-out there is 9.0 mm, the adapter on its pad stands 6.1 mm into it,
  the buck with its tie 1.0 mm, and the UBEC with its tie is 0.8 under it
  (the IMU's rule is 1.0). Measured options, the owner's call with B51:
  the Pi on 13 mm standoffs fits the buck and the UBEC; on 18.5 mm, with
  the IMU moved to (18, 1), all four fit. Each raises the Pi stack 2.0 or
  7.5 mm under the carapace.
- **Deck position:** the tray's place on the deck, (0, −38) in the body
  frame, is still an assumption (`part_avionics.TRAY_XY`), and there the
  tray and its rails run into the coxa bases at stations 2 and 3. As drawn,
  no position clears all five legs, and the rail tabs' holes miss the deck
  grid (B51). D063 measured a layout that fits: the tray at (0, 0), turned
  180°, tabs inboard, the wings cut into lugs that release after 16 mm of
  slide. It is a proposal for the owner and revises this interface
  ([BODY_LAYOUT_PROPOSAL.md](BODY_LAYOUT_PROPOSAL.md)).

**Rocky-scale:** a 19" subrack-style card cage with locking DIN connectors.

---

## I5 · Battery sled + XT60 dock

- **Battery:** one 3S 5200 mAh LiPo in a SOFT case, at most 138 × 44 × 25
  (VERIFY on purchase; `bom/BOM.csv` B-05). Hard-case packs are ~37 mm tall
  and do not fit the 30 mm bay. The pack straps to a printed **sled**, which
  slides on belly rails into the 175 × 50 × 30 bay. The bay itself (floor,
  walls, the door's frame) is not modelled yet (B84). Until it is, nothing
  on the robot carries or retains the sled, and the robot runs tethered.
  D063 measured three layouts for it (a tub hung under the deck, option A
  recommended), which would revise this interface: a proposal for the
  owner, [BODY_LAYOUT_PROPOSAL.md](BODY_LAYOUT_PROPOSAL.md).
- **XT60 blind-mate dock:** the sled nose carries a panel-mount XT60E-M
  (body 16.2 × 8.6 × 16.0, `xt60_panel_mm`). The bay's fixed XT60 sits in a
  floating pocket (±0.8 mm) with lead-in chamfers, so you push to seat.
  That pocket is `part_battery.dock_block`, the robot's half of I5 (not
  part of the B14 charging dock). As drawn it cannot be bolted down: its
  two floor holes sit under its own roof, where no driver reaches (B86).
  The belly door (I3) closes over the sled's tail lip and does the
  retaining; the connector does not. Its latches have no strike yet.
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

- **Male profile (on the shell):** an undercut trapezoid dovetail, 4 deep:
  the frozen widths turned over since D063 (B83), 8 wide at the root (the
  neck) and 12 at the outboard face, so a shoe cannot lift off
  (`iface.dovetail_male(undercut=True)`; `part_stand`'s spigots keep the
  12-at-the-root key, and the params names `base_w` / `crest_w` read for
  that key). As built, each sector web has one VERTICAL 16 mm segment at
  ±27°, square to the wall's chord under a shoe, giving 10 stations and no
  top station. A shoe slides down from above, gravity seats it and the
  set-knob locks it (D029, D063). The params `segment_len` (24) is the spec
  length, used by the demo coupons (`part_panel`).
- **Female shoe (on the accessory):** the matching undercut slot plus a
  printed set-knob (`thumb_knob_m3`, a hex head, B82) on a 45° spot face at
  the shoe's upper edge. Its tapped bore aims at the male's flank: the screw
  meets it first and wedges the shoe against both flanks (6.0 mm of thread
  in the shoe). An M3 × 12 is the shortest that reaches (the knob 2.1 mm off
  its spot face); BOM A-19's M3 × 16 clamps with the knob 6.1 mm off, and
  backed out to ~7 mm its tip clears the slot for sliding on. On the shell
  the knob faces the leg's arch: two knobs facing one seam would meet.
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
| I1 leg port | dowel Ø4 × 8 @ (−28, ±19); thumbscrew M3 × 16 hex head @ (−41, ±17); inboard hook lip 24 wide through a 4.2 deck slot @ x = −48; cable cutout 11 × 11 @ x = −29; XT30 + JST-XH 5-pin |
| I2 tool socket | lug Ø2.5 × 2.2 @ 9 from the face; entry 6; twist 90°; detent 0.6; JST-SH 3-pin |
| I3 panel latch | insert housing Ø14 × 6, a D (flat at 6.0, keyways 135° from the entry line; pocket Ø14.3, flat at 6.15), bayonet rotor, cam rise 0.8 with a 0.2 bite, 90° throw, frame ≥ 6 thick; magnet Ø6 × 3 |
| I4 avionics tray | plate 84 × 70 × 3; rail 3 × 3; bulkhead 70 × 26 face |
| I5 battery sled | bay 175 × 50 × 30 for a soft-case pack ≤ 138 × 44 × 25 (VERIFY vs the purchased pack); XT60E-M 16.2 × 8.6 × 16.0; XT60 float ±0.8 |
| I6 dovetail | 12 / 8 / depth 4 (on the ring: 8 at the root, 12 outboard, D063); segment 24 spec, 16 on the shell as built; M3 set-knob on a 45° spot face |

## Decision

Logged as **D020** in [decisions.md](decisions.md). Any single interface can
be revisited with bench evidence. The RULE (loads through geometry, never
latches) is not up for revision: it is what makes the whole robot
serviceable by a person with cold hands and no tray for screws.
