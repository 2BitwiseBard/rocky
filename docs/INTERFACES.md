# Interfaces: Pebble's six standard interfaces (D020)

*The modularity contract. Every part mates through one of these six
interfaces, or has a written reason not to. The dimensions are frozen in
`cad/params.yaml` under `interfaces:`. CAD scripts read them from there,
through the helpers in `cad/iface.py`, so a number changes in one place.
Revised 2026-10-07/08 (D064, the owner's body-layout picks): I1 docks on
seats, I3 has a strike for sliding panels, and I4 and I5 are described as
built.*

**The one structural rule: loads route through seats, lips and
shoulders, never through latches, thumbscrews or magnets.** Fasteners and
latches only *retain*; positive geometry *carries*. A stripped latch or a
loose thumbscrew must never be able to become a structural failure, only a
rattle. I1 does not meet the rule in stance today: its thumbscrews carry
the peel (below, B118).

Print fit: mating clearances inherit `print.clearance_fit` (0.30), which the
CAD scripts apply at generation time; each frozen dimension below is
nominal. `print.clearance_press` (0.15) is recorded for press fits, but no
script reads it yet (press seats are reamed by hand). VERIFY items are
measured on real prints (the fit ladder and the port coupons,
[PRINT_PLAN.md](PRINT_PLAN.md) batch 0, and the
[interface coupons](PRINT_PLAN.md#interface-coupons-pla-02-mm-with-batch-1))
and updated in params, never in part code.

---

## I1 · Leg port (deck ⇄ coxa base): the flagship

One leg is one field-replaceable module. The target is a leg swap in
**under two minutes**: *own carapace sector off, hook, lower onto the seats,
two thumbscrews, plug two connectors.* It takes one tool: the sector's I3
latch is turned with a flat screwdriver from under the deck (B125).

Revised 2026-10-07 (decision 15, B117): as frozen in D020 the port could not
dock (the hook's 5.15 mm foot met a 4.2 slot; the Ø4 dowels blocked the
pivot even at slots 5.2–7.0). The fix is the "seats" design with the judge's
graft ([archive/prep-2026-10-01/I1_DOCK_OPTIONS.md](archive/prep-2026-10-01/I1_DOCK_OPTIONS.md)),
and `cad/check_dock.py` proves its dock path on the real solids.

**Mechanical**
- **Seats** (replace the dowels): two 30° cone posts on the deck at
  leg-local `seat_xy` (−32, ±17.3), base r 2.9, 3.0 tall. The +y socket in
  the plate is the matching cone (x and y); the −y one is that cone drawn
  out ±0.3 along y (x only), so x, y and yaw are set once. The plate's
  underside is relieved 0.2 outboard of x −46 (`seat_relief_x0`, the graft:
  the seats get 30 % of the screw preload, 8 % at the designer's −42), so z
  is set by the seats and the inboard pads. Docked play is 0 in nominal CAD,
  and stays 0 up to about 0.05 of radial socket oversize (`print.seat_fit`,
  0 until the coupons file it). The seats take the shear: 85 N a seat at
  yaw stall, SF 1.86–1.95 (1.23 with a 30 N shove on one post).
- **Hook:** a downturned L on the plate's inboard edge, 24 wide, 3.0 thick,
  its foot reaching 3.5 under the deck through a slot centred at x −47.0,
  6.2 wide (`hook_slot_x`, `hook_slot_w`; were −48 / 4.2), widened outboard
  only: the inboard edge stays at −50.1. The plate runs inboard to the stem
  (x −49.55) for |y| ≤ 18: those are the inboard pads. Docked, the foot's
  top sits 1.5 under the deck bottom.
- **Thumbscrews** (unchanged): 2 × M3 × 16 hex head (ISO 4017, `bom/BOM.csv`
  A-19) keyed into printed knobs (`thumb_knob_m3`, 8.6 tall) at (−41, ±17),
  reachable from above with the servo installed, 5.5 mm into the deck's
  5.7 mm heat-set inserts (an M3 × 10 never reaches). Not captive (B82).
- **Spine rib** (B119) on `coxa_yaw_base`: x −46 to the cup's back wall
  (−38), |y| ≤ 8, 6 tall.
- **Docking:** tilt the leg 7–10° (outboard end up), bring the L in
  radially through the slot, lower it: the cones close in the last 0.61 and
  centre the plate. `check_dock` keeps 0.469 in plane on the widest stored
  path, 0.300 in 3D at the lip ends. With the tray in, hold legs 1 and 4 at
  about 10° and ≥ 1.5 outboard until the L is below the tray rail's top (a
  level approach pushed inboard meets the rail; the Pi is met only from
  16.1 mm up).
- **Load path** (B118; D020 had it upside down). Shear and yaw torque go
  into the seats. In stance the ground lifts the plate's outboard end, the
  plate pivots on its inboard pads, and the thumbscrews and their inserts
  carry the peel: 568 N per pair in the design case (3 legs × 3, μ 0.5; 972
  N on the D020 port). The hook carries nothing in stance: it is a catch,
  1.5 under the deck, for a hanging leg (whose screws carry 105 N per
  pair). Insert pull-out (wanted
  ≥ 300 N, the design case is 284 N a screw) and hand-tight knob preload
  (wanted ≥ 285 N) are unmeasured; the deck coupon measures them, and a
  failure means an outboard hold-down (the owner's decision).
- **FEM: `coxa_yaw_base` FAILS at servo stall** (D064, B139). `fem_check` now
  holds the plate by what bears in each case (the pads, the seats, the
  thumbscrew heads in tension). In case V, the foot pushed up at ST3215
  stall (6994 N·mm about the screws), the plate rocks on the pads against
  the screws, the seats lift 0.04–0.36 mm, and von Mises reaches 30.4 MPa at
  the vee socket: **SF 1.10** (the other cases 2.40–5.75; the old
  whole-underside grip gave 3.32). The rib's SF 2.33 was analytic at
  the 3 × 3 design moment (4629 N·mm), not at stall. The fix (an outboard
  hold-down, a deeper rib past the sockets, a thicker plate, or the screw
  preload as a design load) needs a measured option round and is the
  owner's call (what the model leaves out: B148). Until then print one
  `coxa_yaw_base` for the bench, not five.

**Coupons first.** The first print is the port coupon deck patch with four
plate patches (`relief46`, `relief42`, `fit10`, `fit20`) and fit-ladder row
D (5.8 / 6.2 / 6.6); the ladder has no seat row. Go: the L passes the 6.2
slot by hand at 7–10°, radially; lowered, the plate centres itself;
finger-tight, nothing rocks, no x or y play (< 0.05 on a dial), a 0.2
feeler stays out from under the pads. No-go: binding prints the 6.6 slot;
rock tries `fit10`, then `fit20` (the smallest that sits flat is
`print.seat_fit`); all three rocking, or > 0.15 of play, means the
"minimal" fallback (owner call). The table is
[PRINT_PLAN.md](PRINT_PLAN.md#go--no-go) (I1_DOCK_OPTIONS §6).

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
  `CABLE_CUTOUT_W`, r = 81 on the deck), reached with the leg hooked but
  not yet lowered. They are not blind-mate at this scale; Rocky goes
  blind-mate. The mated XH-5 pair does not pass the 11 × 11 cutout (B120).
- **Route through the coxa:** the yaw servo sits over the cutout, so
  `coxa_yaw_base` carries an 11 × 11 channel (`part_coxa.harness_path()`).
  It runs up out of the cutout and forward under the gearbox plateau
  (x −34.5 to −13, z −4 to 7), then sideways to −Y under the cup's side
  wall, up beside the cup (y −26.9 to −15.9), and over the top to the yaw
  servo's plugs (z 57.4 to 68.4). It uses the −Y side because the hip cup
  and its plugs sweep +Y at yaw +40. `check_assembly` asserts the path is
  clear of the base, the yaw blank and the whole yawing leg over the
  params yaw range.

**Rocky-scale mapping:** the seats become hardened steel cones; the hook a
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
  up is the same: the housing is symmetric top to bottom. The index is a
  place, not a stop: off the robot nothing holds the rotor's 3.8 mm of
  axial play, and turned 45° back past OPEN it still drops out of its
  housing.
- **Strike (frame side, `iface.latch_strike`):** a Ø6.6 bore and two entry
  slots through a 3.2 mm land, at 45° to the frame's +x (radial on the
  deck), and an underside recess shaped to the lugs' quarter-turn path. Its
  ceiling is the cam: 0.8 mm of rise over 0–50°, a dwell to 90°, a stop.
  At LOCKED the lugs bite 0.2 mm into the land (`LATCH_BITE`, VERIFY in
  PLA, B99). The frame must be ≥ 6.0 mm thick (the function asserts it; the
  deck is 6, B27).
- **Strike for a sliding panel** (`latch_strike(frame_t, slide)`,
  2026-10-07): at OPEN the rotor still hangs 5.8 into the frame and rises
  only 3.8, so in the round bore it pins a panel that leaves sideways (the
  tray: 2.20 mm³ at 0.5 of slide, 36.18 at 2). The bore is drawn out along
  the entry line into a through slot 6.6 wide (the shaft + 2 FIT), slide +
  5.5 long from the axis: at OPEN the rotor slides out with the panel and
  the slot's end is its stop; at LOCKED the lugs lie square to the slot and
  hold the slide. The panel's latch frame must put the entry line along its
  slide. The tray is the one user (frame 225, 22.0 × 6.6, the stop at
  16.5); `slide` 0 cuts the round strike, unchanged for the sectors and the
  bay door.
- **Operating end: the rotor's bottom,** a 2.0 × 1.5 slot 0.2 mm above the
  frame's underside, square to the lugs. A flat screwdriver (blade ≤ 5 mm),
  a quarter turn clockwise to lock: it starts to bite at ~40° and stops at
  90°. Not a coin: one reaches only ~0.5 mm of the slot before it meets the
  frame. On the deck it is worked from below. The bay door's latch lies on
  its side, its slot facing +x at x −85.7: a Ø5 driver along the tub's
  south wall needs a shaft ≥ 183 mm to pass the carapace skirt (200
  recommended; a slotted rotor head would let it turn from the door side,
  B144).
- **Magnets:** Ø6 × 3 glue-in pockets (N35+) wherever a soft-close seat is
  wanted. Polarity convention: every magnet, panel side and frame side, has
  its NORTH face pointing away from the robot (the way the panel lifts
  off). At each joint the frame magnet shows N and the panel magnet shows
  S, so any panel snaps onto any frame station. Steel strikes take either
  face.
- **As built:**
  - the carapace sectors (`part_shell`): each sector foot has one latch
    (az +27.5°, r 74.5) and one magnet (az −25.5°, r 76), mating the deck's
    latch strikes and washer recesses (D030). The latch is turned from under
    the deck; the latches at stations 162 and 306 sit inside the tub's plan,
    which leaves a driver column open for each (B100, B126: turn them with
    the sled out);
  - the top hatch (`shell_cap`): five plug magnets on five seat magnets at
    r 42.5, one per sector, each seat in a walled boss under the seat ledge
    (D063, B88). It has no latch. Both magnets of a pair go in N up;
  - the bay door (`part_bay.bay_door`, I5): one latch in its south ear over
    the strike in the tub's south-wall boss, a snap hook tab at its north
    edge, a spigot that locates it; no magnets;
  - the tray tongue (I4): one latch in `tray_latch_boss` on the deck's
    slotted strike at (0, −43).

**Rocky-scale:** the same cartridge in glass-filled nylon, Ø22, with a
captive lanyard so field panels don't drop.

---

## I4 · Avionics tray

The Pi 5 with its cooler and the BNO085 IMU ride ONE slide-out tray
(`part_avionics`). Revised 2026-10-07 (decision 11, B51). The bus adapter,
the 5 V buck and the 6 V UBEC are on the hub shelf under the deck, not on
the tray (pick 14, [below](#hub-shelf-under-the-deck)). The ESP32 Servo
Driver is a bench tool and does not ride on the robot (`bom/BOM.csv` A-03).

- **Pose:** plate centre at body (0, 0), turned 180° (`tray_xy`,
  `tray_rot_deg`): bulkhead north, latch tongue south. It is the one pose
  clear of the five coxa bases and yaw servos (D063's (0, −38) ran 534 + 954
  mm³ into bases 2 and 3).
- **Plate and rails:** plate 84 × 70 × 3; its wings are three 10 mm lugs a
  side on a 25 pitch under three 6 mm lip segments of two C-channel rail
  blocks (`tray_rail`, 56 long) at x ±47.3. The rails carry the load. Their
  bolt tabs point inboard, under the plate, on the deck's (±40, −20 / 0 /
  20) holes, with M3 ISO 7380 button heads (a socket head's 3.0 does not
  fit: 3.40 of head room centred); the four south ones at (±40, −20 / 0)
  are also the tub's hanger screws (I5), M3 × 16 through Ø3.4 holes into the
  lid bosses' inserts; the two north ones at (±40, 20) are M3 × 10
  thread-forming into the deck itself (Ø2.8, B155: nothing hangs under them,
  the tip 1.90 over the star board). The channel sits `tray_lift` 2.0 higher than
  D063 drew it: the plate's underside stands 5.4 over the deck top centred,
  5.1 resting (`TRAY_Z`). The tongue-end corners are cut square to stations
  234 / 306 (0.87 to the bases at slide 16).
- **Single latch:** an I3 insert in the tongue, through a glued
  `tray_latch_boss` 5.1 tall (the tray prints plate-down, so the boss is its
  own print; 1.6 of wall round the pocket), so the cartridge seats on the
  deck top where its strike is: (0, −43), slotted along the slide (I3). It
  only retains.
- **Release** (pick 7; the lugs pass the windows after 8.3–16.7 of slide):
  the robot off the stand, the sled out (the strike's driver column runs
  through the pack and the cradle), the carapace off, the bulkhead face and
  the leads on the Pi's header unplugged, the latch to OPEN from below,
  slide 16 **south** (the slot stops it at 16.5), lift. North is blocked:
  the plate would cover the (0, 50) trunk slot, and the strike holds it.
  With the carapace on, the boss meets sector 2's latch pad from slide 14
  (4.04 mm³), and lifted the Pi meets sector 3 from lift 10, sector 4 by 14,
  the cap by 18 and sector 1 by 42; with it off the path is 0 mm³. So
  D020's "tray out with one latch" holds with the carapace off and the
  robot off the stand (B143).
- **Bulkhead** at the north end (70 × 26 face, outer face at body y 35.0):
  the XT30 (5 V in from the shelf's buck) at body x −9.5 and the JST-XH
  5-pin trunk (the Pi's UART to the bus adapter) at x +7.0, both at body z
  12.4; their leads run 11.25 north over the deck and drop through
  the (0, 50) slot ([WIRING_HARNESS.md](WIRING_HARNESS.md#through-the-deck-the-0-50-trunk-slot)).
  Also 2 × JST-SH spares, a Qwiic/JST-SH 4-pin I²C (the sensor mux) and a
  USB-C pass slot. The five J6 foot-switch lines have no bulkhead connector
  (B123): they plug onto the Pi's header.
- **Pi 5:** on four thread-forming standoffs, 58 × 49 and 11 mm tall (D063,
  B89, B90; pick 14 keeps 11), M2.5 × 6 or × 8, its USB end toward body +x
  (leg 4). Modelled from the official Pi 5 STEP's boxes
  (`cad/ref/rpi5_envelope.json`, MIT, B127): 6.82 mm in 3D to the carapace,
  6.54 at the worst ±0.3 float (the rule is 3.0). Only the lower middle
  USB-A port takes a plug (B128): the lidar adapter's, a right-angle plug
  ≤ 20 long, overmold ≤ 14 × 7, cable down or south.
- **IMU:** the BNO085 sits on four TPU grommets (B4) under the Pi, so it
  goes on first: on the board's 20.32 × 17.78 holes about tray (0, 13.5),
  each seat lifted 1.2 mm over a recess for the grommet's flange (D063,
  B90). Each M2.5 × 10 needs a standard nut (the grommet bore does not
  thread); the nut and tail keep 2.90 to the deck, the IMU's top 1.2 under
  the Pi's keep-out. It talks over SPI
  ([WIRING_HARNESS.md](WIRING_HARNESS.md#pi-5-pin-map-draft-verify)).

**Rocky-scale:** a 19" subrack-style card cage with locking DIN connectors.

---

## I5 · Battery sled + XT60 dock

Revised 2026-10-07 (decision 11, picks 1–8, 10, 12): the bay is a part,
option A's keel tub (`part_bay`: `bay_tub` + `bay_lid` + `bay_door`, PETG),
hung east-west under the deck at y −20, nose (XT60) +x, door −x.

- **Battery:** one 3S 5200 mAh LiPo, SOFT case, at most 138 × 44 × 25
  (VERIFY on purchase; pick 5 kept 138, the BOM's pack is 132; taken as 380
  g, pick 6). Hard cases (~37 mm) do not fit. It sits loose on a printed
  sled (48.8 wide in the 50 bay): no strap (pick 8 b), a 2 mm foam pad
  under the roof takes its play. Height: the sled rests 2.10 over the
  floor, + sled 3 + pack 25 + pad 2 = 32.10 of 33.
- **Tub:** interior 192 × 50 × 33 (`bay_l`, `bay_w`, `bay_h`; walls and
  floor 2.4, roof 2.0); outside x −97.5 (the door) … 99.9, y −47.4 … 7.4, z
  −55.4 … −18.0. The roof's top at z −18 leaves the 8 mm layer under the
  deck for the leg 2 / 3 looms (pick 2). The bottom is the robot's lowest
  point, 62.6 over the ground at stance (62.1 settled in the sim). The
  sled's rails are printed into the floor. The nose's vertical edges are
  chamfered 10 in plan (pick 10); the nose came in only 2.0 for the carrier
  (the mated XT60 pair's length sets `bay_l`), so its south corner still
  stands 7.76 outside the carapace skirt (B145). The sim's belly box takes
  the same sums, square-nosed (`iface.bay_tub_extent`); `check_sim_mirror`
  holds the printed tub to it plus the protrusions in
  `iface.BAY_TUB_OUTSIDE`.
- **Hangers:** `bay_lid` has six Ø8 bosses up to the deck bottom with M3
  heat-set inserts: at (±40, −20) and (±40, 0) under the tray-rail tabs (one
  M3 × 16 ISO 7380 clamps tab, deck and boss) and at (±58, −18) (M3 × 16
  from the deck top). The tub meets the lid with a north return (the north
  wall's top reaches 1.2 over the lid's edge: the tub goes on 1.6 north,
  rises, slides south under it) and two M3 × 10 from below through the south
  pilasters (Ø9.6 at (±70, −50.6)) into the lid's inserts; no north screw
  fits (the joint before the lid is on: B147). The lid also carries the leg
  2 / 3 loom tie bars beside its (±40, 0) bosses.
- **XT60 blind-mate dock** (pick 4): the sled's nose carries a panel
  XT60E-M glued 8.0 proud. The tub's nose carries a printed-in floating
  carrier for a FEMALE XT60 on four 14 × 1.2 × 1.2 flexure webs, free at
  ±0.8 in y and z (`xt60_float`), stopped at ±0.85 (23.4 MPa in the webs,
  PETG's allowable 35), a 1.2 lead-in at its mouth: push to seat. The
  housings telescope 7.0 when mated (VERIFY with the real pair, B150).
- **Loop key and fuse** (pick 3): the key's panel XT60E-M sits in the nose
  wall above the carrier, facing +x, reachable with the carapace on. The
  fuse holder goes over the carrier (16.3 of height free, VERIFY with the
  holder, B149). The 14 AWG feed leaves through a Ø5.5 hole in the NE
  chamfer, its riser held by a clip on the north wall (B133,
  [WIRING_HARNESS.md](WIRING_HARNESS.md#the-14-awg-feed-b133)). The spare
  taps come off the PDB, after the fuse (B66 closed).
- **Retention: the door** (pick 12). `bay_door` (54.8 × 37.4 × 3) has a
  spigot in the opening (y and z), a boss that presses the sled's tail lip
  0.5 (it retains the sled; the connector does not), one I3 latch in a 6 mm
  south ear over the strike in the tub's south-wall boss (x −91.5 …
  −85.5), turned from +x with a long driver (I3), and a snap hook tab at
  its north edge behind a keeper: push the tab's free end out to release.
- **Turn points inside the tub's plan** (B100, B126): the carapace latches
  at stations 162 (−73.48, −12.30) and 306 (66.67, −33.24) and the tray's
  strike (0, −43) each have a Ø6 driver column through the floor and the
  roof. The sled fills the room between, so they are turned with the sled
  OUT.
- **Stand:** a Ø6.3 × 1.5 blind hole at (0, −20) takes the stand's Ø5.4
  x-stop pin (0.45 of play); the tub's sides and its bottom over x ±45.5
  stay flat for the cradle ([below](#stand-cradle)).
- **Swapping the pack:** pull the loop key, turn the door's latch, release
  the tab, take the door off, slide the sled out −x (on the stand too), and
  reverse. Leg 1 must sit at yaw ≤ +20 (at +30 it meets the sled from 130
  mm out, at +40 from about 102). The Pi reboots when power returns.

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
- **Users:** sensor pods ([SIM_GUIDE.md §9](SIM_GUIDE.md#9-not-built-yet-perception-on-the-robot)),
  whisker mounts (B11), LED ring segments and shell tool docks. The camera
  is fixed behind a gill (B16) and the lidar goes on a hatch-cap variant
  (B12); neither uses the ring.
- **Load rating:** ~15 N pull and ~0.4 N·m moment, estimated for 24 mm of
  PLA engagement. Bench it at the as-built 16 mm. Good for sensors, not for
  handles; the shell itself still carries nothing (D006).

**Rocky-scale:** an aluminium rail ring, deliberately a Picatinny cousin so
the tooling exists.

---

## Under the deck: the hub shelf and the stand cradle

Neither is one of the six interfaces (nobody swaps them in the field), but
both meet I4 and I5.

### Hub shelf (under the deck)

- `hub_shelf` (`cad/part_busboard.py`, picks 1, 9, 14) is one print: a 3.0
  plate (top at z −36.3, plan x −56.2 … 53.95, y 12.2 … 64.3) on three Ø8
  posts at (−50, 34), (50, 34), (26, 64), each held by an M3 × 30 socket
  head from BELOW into a blind Ø2.8 × 5 hole in the deck (a head on the deck
  top at (±50, 34) would sit under the docked coxa plate).
- On top, the 12 V PDB (30.5 × 30.5, pick 9) and the star board on 5 mm
  standoffs; under the plate, face-down to z −54.4, the bus adapter, the
  5 V buck and the 6 V UBEC ([WIRING_HARNESS.md](WIRING_HARNESS.md#the-hub-shelf)).
- It keeps 0.885 to the docked coxa bases, 0.339 to the hook envelope and
  1.92 to the tub (its riser clip). Service means the robot off the stand:
  on it, three of the adapter's four screws are under the crown plate
  (B134).

### Stand cradle

- `stand_crown` (`cad/part_stand.py`, B85) holds the robot by its tub,
  carapace on: a floor at x ±45 under the tub, two 25 mm walls 0.5 off its
  sides, open ends, and a Ø5.4 pin at (0, −20) into the tub's hole. The
  robot fits one way, door end −x; ±1 mm in x or y binds.
- The deck bottom stands 89.4 / 169.4 / 249.4 over the bench (crown, + 1
  section, + 2); a leg reaches 153.3 below the deck bottom (183.5 with the
  B95 hand), so two sections free the whole envelope.
- On the stand the door opens, the sled slides out, and the carapace
  latches 162 / 306 turn with the sled out; the tray's latch does not (its
  column meets the cradle), and the hub is not serviceable. Print the stand
  after `part_bay`: it needs the tub.

---

## Frozen dimensions

All are in `cad/params.yaml → interfaces:` (the I1 cable cutout is in
`cad/iface.py`). In summary:

| iface | key dims (mm) |
|---|---|
| I1 leg port | seats: two 30° cone posts r 2.9 × 3.0 @ (−32, ±17.3), one cone and one vee (±0.3 in y) socket; plate relieved 0.2 outboard of x −46; thumbscrew M3 × 16 hex head @ (−41, ±17); inboard hook lip 24 wide through a 6.2 deck slot @ x = −47 (inboard edge −50.1); cable cutout 11 × 11 @ x = −29; XT30 + JST-XH 5-pin |
| I2 tool socket | lug Ø2.5 × 2.2 @ 9 from the face; entry 6; twist 90°; detent 0.6; JST-SH 3-pin |
| I3 panel latch | insert housing Ø14 × 6, a D (flat at 6.0, keyways 135° from the entry line; pocket Ø14.3, flat at 6.15), bayonet rotor, cam rise 0.8 with a 0.2 bite, 90° throw, frame ≥ 6 thick; sliding-panel strike: a 6.6 slot, slide + 5.5 long; magnet Ø6 × 3 |
| I4 avionics tray | plate 84 × 70 × 3 at (0, 0) turned 180°; 3 lugs × 10 a side on a 25 pitch, released after 16 of slide south; rails 56 × 10 @ x ±47.3, tabs on (±40, −20 / 0 / 20); plate 5.1 over the deck resting; bulkhead 70 × 26 face |
| I5 battery sled | tub interior 192 × 50 × 33 (roof top z −18, bottom −55.4) for a soft-case pack ≤ 138 × 44 × 25 (VERIFY vs the purchased pack); XT60E-M 16.2 × 8.6 × 16.0 on the sled, a female XT60 in the carrier, float ±0.8; nose chamfer 10 |
| I6 dovetail | 12 / 8 / depth 4 (on the ring: 8 at the root, 12 outboard, D063); segment 24 spec, 16 on the shell as built; M3 set-knob on a 45° spot face |

## Decision

Logged as **D020** in [decisions.md](decisions.md), revised by D064 (I1
seats, the sliding-panel strike, I4 and I5 as built). Any single interface
can be revisited with bench evidence. The RULE (loads through geometry,
never latches) is not up for revision: it is what makes the whole robot
serviceable by a person with cold hands and no tray for screws.
