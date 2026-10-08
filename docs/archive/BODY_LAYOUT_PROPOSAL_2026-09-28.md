# D063 body interior: three layouts for the bay, the tray and the hub

*Archived 2026-10-08. Built as D064 on 2026-10-07/08 from the owner's 15 picks; the as-built
numbers are in [INTERFACES](../INTERFACES.md) I4/I5 and the [ASSEMBLY_GUIDE](../ASSEMBLY_GUIDE.md).
Where this text and the tree disagree, the tree is right.*

**Status:** a proposal for the owner to choose from, not built (D063,
[decisions](../decisions.md), BUILD_LOG 9m). Every number below was
measured on 2026-09-28 in build123d scratch geometry posed against the repo's
own parts (`coxa_yaw_base`, the yaw servo, `body_deck`, the five carapace
sectors + cap, `battery_sled`, `dock_block`, `avionics_tray`, `stand_crown`'s
skirt). No CAD file was changed; the measurement scripts and the sketches were
scratch work and are not in the repo. Frames: body frame, z = leg z,
deck −10..−4, coxa plate top 0, ground −118 at stance; +x east, leg 0 north.

**How to decide.** Read the fifteen decisions in
[section 8](#8-decisions-only-the-owner-can-make) (13 from the proposal, 14
from B108, 15 from the I1 docking fault found 2026-10-01); each names the
numbers it turns on. The owner's phone page carries the same list. Decision 1 (where the hub lives) sets the rest: A is recommended
(section 7), B no longer fits as drawn (section 4), C trades 30 g and a big
print for an enclosed belly (section 5). Section 6 has the three side by side.
Once the decisions are in, section 11 lists the checks the implementation must
add, and the backlog rows B51, B84, B85, B86, B92 and B93
([DESIGN_BACKLOG](../DESIGN_BACKLOG.md)) close with it; B97 (the legs reach the
tub) and B109 (the leg drops meet under the deck) ride along. Not checked by
either run: station 162's carapace latch (B87) is turned from under the deck at
body (−73.5, −12.3), which is inside the tub's plan outline (section 2: x −97.5
… 101.9, y −47.4 … 7.4), so the tub must leave that column open for a
screwdriver (B100).

**Reviewed 2026-09-28** (an adversarial pass),
against parts rebuilt from the tree *after* the other D063 groups' edits: the
coxa base and hook feet are unchanged (0 mm³ difference), but the tray now
stands the Pi on 11 mm standoffs (B90, was 6), the carapace grew five pads
that hang to z 45.4, and the deck's latch strikes changed (B87). What the
review changed, each marked *(review)* where it sits:

- **Option B no longer fits.** Its Pi enters the carapace by 492 mm³ at lift 18,
  whichever way its USB end faces. A 3 mm margin needs lift ≤ 11.4, which
  leaves 14.8 mm under the tray for a hub modelled at 17.6 (section 4).
- **A's carapace margin is 5.5 mm, not 10.5** (12.25 with the Pi's USB end +x).
- **The sled's way out was computed with the wrong yaw.** The model turned
  each leg about the body centre, not about its own station. Now: leg 1
  blocks it at yaw +27 … +40°, and leg 2 never does.
- **Two things the proposal specified do not fit:** a 1.5 mm strap in the
  width budget (0.6 mm per side), and I3 latches in the door-end walls
  (the strike needs a 14.6 × 14.6 pad, 6 deep). Measured fixes are in section 2.
- **The tub's south corners overhang the deck by 26.5 / 30.7 mm**, not
  19 / 23 (those are the centreline figures).
- **The legs can reach the tub within their soft limits** (legs 1–4, femur
  steeply down and knee folded back). It is new, and nothing checks it (sections 3 and 10).
- **The robot CoM left out the five hands (193 g).** The margins move by
  +0.1 … +0.2 mm.

**Re-measured 2026-10-01/02** (two read-only prep rounds, 39 agents, every
headline claim re-measured by a second agent; BUILD_LOG 9p; the full records
are [archive/prep-2026-10-01/](prep-2026-10-01/): `PREP_REPORT_1.md`,
`PREP_REPORT_2.md`, `I1_DOCK_OPTIONS.md`). Nothing below changes the
recommendation (A, 8 mm layer); three things change what the build must do
first, and the owner's page gained decisions 14 and 15 (section 8).

- **The tub is safe in the sim.** With the tub as a box in the MJCF (the sim
  had no belly: its torso bottom was 124 mm over the ground, the tub's is
  62.15 settled), self-righting is unchanged over 200 falls (197 stand, with
  or without the tub, 8 or 5 mm), rubble at 30–45 mm is unchanged (0 tub
  contacts, closest 14.3; first touch at 62 mm rubble; the only tub-caused
  stalls are the square nose corners jamming at 64–70 mm), and the gait,
  gestures and floor probes never reach it. Only the righter's fall path does:
  6/200 drops massless, **12/200 with the battery mass** (max 3.06), without
  changing whether the robot stands. The battery in the tub also lowers the
  CoM 26 mm and lifts the no-righter baseline 87 → 104/200.
- **The I1 leg port cannot dock as drawn** (backlog B117): the hook L is
  5.15 wide at the bottom against the 4.2 slot (4.6 fails too), and the 8 mm
  dowel posts block the pivot even with a 5.2–7.0 slot. Three verified fixes
  exist; the judge recommends cone "seats" (decision 15). So the hook sweep
  used in fact 2 was never reachable; the figures that replace it are in the
  corrections below. The stance load path was also wrong in INTERFACES I1:
  the hook carries nothing in stance, the thumbscrews carry 972 N per pair
  today (B118) and the plate is at SF 0.74 at the screw line (B119).
- **Three under-deck turn points sit inside the tub's plan**, not one:
  carapace latches 162 and 306 and the tray's own strike at (0, −43) (B126).
- **Decision 13 is measured:** a uniform hip limit that clears the wider
  tub (bay_w 53) switches off the cliff void guard (its ceiling is −47.91 at
  h 118, B129); the measurements support modelling the belly and clamping the
  righter's hip commands only in FALLEN/RIGHTED (0/200 contacts, outcome
  unchanged; B130). Every reach number assumes a 135 mm foot (B131).
- **Decision 14 is measured:** with the real Pi 5 parts, the Pi on 18.5 mm
  fails the 3 mm rule, and on 13 mm no USB-A port takes a plug reliably
  (B128). The measured home for the three boards is face-down on option A's
  hub shelf with the Pi kept at 11 mm (all three at bay_w 50 with a 0.80
  margin; the adapter alone also at 53), the bus adapter driven over UART.
  The shelf then needs three new posts, the stand's cradle and crown plate
  in the keep-outs, and hub service means the robot off the stand (B134).

Corrections to this document, each "section: old → new":

1. Intro, "Not checked by either run": one latch column → three (latch 162
   at (−73.48, −12.30), latch 306 at (66.67, −33.24), the tray strike at
   (0, −43)); each Ø5 driver column meets the tub for 734.3 mm³.
2. Intro and section 3 margins, "A's carapace margin is 5.5 mm, not 10.5
   (12.25 with the Pi's USB end +x)" → vertical 5.45 / 12.40 with a 16 mm
   part stack; with the real Pi 5 STEP the 3D gap is −x 3.56 (3.41–3.72) /
   +x 6.84 (6.63–7.05) at 11 mm standoffs, 2.17 / 5.45 at 13.
3. Section 1 fact 2, "a hook foot reaches z −13.8 (−13.3 docked), at
   r 57.5–68.4" → today's hook cannot dock at all. Its deck-clear sweep would
   reach −14.80 (−15.05 with a 5.2 slot), r 56.95–69.31. With the seats fix:
   −13.30 on the path, −13.76 at any reachable pose, r 56.94–66.79.
4. Section 1 fact 2, "372 mm³ … 23.6 mm³" → not recoverable; ≥ 48 for every
   50–55-wide deck-roof bay, pack alone 48 with a deck-clear sweep. The
   rejection holds.
5. Section 1 fact 3, "y −23 … −16 (y −26 … −14 with 3 mm)" → y −21.9 … −17.2
   at bay_w 53 (−26.2 … −13.1 for the 3 mm reading); y −20 stands.
6. Section 1 fact 1: the 697 mm³ used an 11 × 5.5 loom; the real per-leg loom
   is 7 conductors, 14.17 mm². The conclusion holds.
7. Section 2 table, "ground clearance at stance 62.6 / 65.6" → 62.15 / 65.15
   settled in the sim (62.13 with the mass).
8. Section 2 table, legs: "about 205 poses per leg … Leg 0 never reaches it"
   → 114 / 97 / 97 / 114 (sim capsules), 138 / 122 with r 15 / 12; leg 0
   never reaches the tub but reaches the shelf box (15 poses) and the shelf
   boards.
9. Section 2 "Stand" and section 3 risks, "serviced from under the deck (on
   the stand the cradle leaves the north underside open)" → open only north
   of y 47.2; with face-down boards at z −54.4 the adapter screws are blocked
   from below and the shelf cannot be lowered on the stand (one agent).
10. Section 3, shelf hangers "(±40, 20), (0, 20), (0, 40)" → they block every
    arrangement with the bus adapter; posts at (−50, 34), (50, 34), (26, 64)
    (0.59 to the hook sweep, 0.89 to the coxa bases, 0.57 to drops +5).
11. Section 3, "boards facing up … 28 mm deep (… z −38 … −10)" → with the bus
    adapter on the shelf it hangs face-down to z −54.4 (adapter (−19.8, 30.8),
    node (−20.2, 38.2) up, star (21.8, 34.2) up turned 90; plate top −36.3).
12. Section 3, lanes "5 × 7 mm (x ±44.6 … 49.6 south to y −38, out past the
    hook envelope at x ±51.8 … 57, then down to the cutout)" → 8 × 7:
    A x ±45 … 53, y −38 … 12; B x ±45 … 59.8, y −38 … −30 (the jog north of
    y −38); C x ±51.8 … 59.8, y −53.43 … −30; the riser inside the drop
    keep-out box; the loom leaves the shelf through its south face. A bare
    loom fits 5 × 7; a zip-tied one needs 8 × 7.
13. Section 3, "The 14 AWG … along y 8–11, z −26 … −22" → overlaps the
    bay_w 53 tub's north wall; at bay_w 53 no y position clears leg 4's drop
    (one agent); at bay_w 50 only y 7.9 … 8.8. A route near x 70–90 is
    needed (B133).
14. Section 3 margins, "Lane side clearance: 0.5 mm to the Ø8 hanger bosses
    (1 mm more and it touches: 15 mm³)" → 0.60; 1 mm more is 26.31 mm³ in the
    8 mm layer (15.03 is the 5 mm layer); the 8 × 7 lane keeps 1.00.
15. Section 3 deck holes, "(0, 50) | 10 | grommet: the tray trunk + tray
    XT30" → passes no plug with leads on (XT30U needs Ø11.42, XHP-5 + leads
    Ø11.47–12.88). Route before termination, or a 12.5 × 7.5 r1 slot (4.05 to
    option A's holes). The J6 foot-switch lines also cross here (B122, B123).
16. Section 3 deck holes, "(0, −43) … the tray's latch strike" → inside the
    tub's plan; its driver column meets the tub for 734.3 mm³.
17. Section 3 CAD changes, `iface.leg_port_hook_envelope()` "bottom −13.8" →
    derived from params after the I1 fix: −13.30 path / −13.76 any pose with
    the seats fix.
18. Section 3 risks: add the righter folding legs under the keel (12/200 with
    the mass, B130), and "Nothing gates those poses" → once the MJCF has a
    belly, SELF_CONTACT gates the gesture audit and the cockpit studio; the
    righter, `save_keyframe_gesture()` alone and the playground `check` are
    not gated.
19. Section 4, "A 3 mm margin needs … lift ≤ 11.4" → 11.40 centred / 11.10
    floated under a vertical reading only; as a 3D gap 8.28 (Pi drawing
    model) or 6.12 (16 mm stack + overhang). "(with it −x … lift at 4.4)" →
    4.15, 1.15 with the overhang, fails at lift 0 as a 3D gap.
20. Section 4, "B survives only with a hub at most about 14.3 tall (12.1
    under the IMU) … not measured here" → measured: ceiling 14.20 / 12.00
    (vertical), 11.38 / 9.18 (drawing 3D), 9.22 / 7.02 (strict). A
    vertical-header star never fits; a side-entry star on a new 38.9 × 49.2
    board (10.20 tall) plus a 30.5 node (8.30, ESTIMATE) fit the vertical rule
    only. B's rails would be 19.10, not 26; its (0, 52) pass-through and the
    (29, 14) grommet collide with that hub. B as drawn is out.
21. Section 6 table, tray height over the deck, B "21.4" → about 14.5 at
    lift 11.10. Section 6 note, "1434.6 g torso at (0, 0, 24.5)" → 1439.7 g.
22. Section 7, "The lanes are why the layer is 8 mm instead of 5" → a bare
    loom fits 7 × 4 in the 5 mm layer (5 × 4 is marginal); 8 mm is kept for
    the hook margin (4.26 vs 1.26 at any pose with the seats fix) and for
    tied looms.
23. Section 8, decision 10: "26.5 / 30.7 mm at the south corners" → 26.94 /
    31.12 at bay_w 53; the nose-south corner reaches r 112.4.
24. Section 11: add an I1 dock-path regression (planar C-space + 3D replay
    at ≥ 0.30), a connector pass-through check (cutout, grommet), a Pi
    part-height envelope from the STEP with the ±0.3 tray float and a 3D gap,
    and `part_shell` against the posed `coxa_yaw_base` (42.801 mm³ today,
    B124).

It fixes these rows together:

| row | the fault today |
|---|---|
| B51 | no pose of tray + rails clears the five coxa bases; the rail holes miss the deck grid |
| B84 | the bay is not a part; floor rails; legs 1/4 hook feet 3.3 mm into it; 194 vs 175 long; corners off the deck |
| B85 | the stand crown fills the belly |
| B86 | `dock_block` cannot be bolted |
| B92 | the 12 V node has no part and no mount |
| B93 | the sled's strap slots take only ~5 mm straps |

## 1. Five measured facts that decide the layout

1. **The leg drops meet their body side under the deck.** The coxa plate
   covers the I1 cable cutout (leg x −29, body r 75.5–86.5): a loom arriving
   on the deck top meets 697 mm³ of plate (solid from leg x −49 to −35);
   from below, the column under the cutout is clear (0 mm³). So every leg
   drop is reached from under the deck, and the harness hub belongs where
   those runs are short. The "ring lane r ≈ 62" in WIRING_HARNESS cannot be
   on the deck top (the plates start at r 60.5), and under the deck r 62 is
   where the hook feet are.
2. **The bay must hang below the hook feet.** Swept through the docking
   dance (tilt 0–15°, 3.5 mm slide), a hook foot reaches z −13.8 (−13.3
   docked), at r 57.5–68.4 on every station. A bay whose roof is the deck
   bottom hits those envelopes by 372 mm³; even its pack alone (top 1.75
   under the deck) by 23.6 mm³. No straight 55 mm-wide bay passes between
   the five feet at deck level.
3. **Only one band under the deck is free for a 199 mm bay.** The mated
   XT30 + XH-5 hang under each cutout. An east-west tub clears all five
   (with 5 mm round each) for a centre at y −23 … −16 (y −26 … −14 with
   3 mm). A north-south tub has leg 0's cutout over its nose: its best pose
   needs y0 −130, 49 mm past the deck's south edge. **All three options put
   the tub east-west at y −20, nose (XT60) +x, as I5 and the power entry
   already assume.**
4. **The tray fits only at (0, 0), with the rail tabs turned inboard**
   (B51's lead, re-measured: 0 mm³ with bases and yaw servos, tab holes on
   the existing grid holes (±40, −20/0/20)). **And it cannot slide off its
   rails with the legs docked, anywhere:** at (0, 0) it can travel 36 mm
   north / 22 mm south before a coxa base, against 56 mm of continuous wing.
   I4's "slide-out" needs interrupted wings (lugs) that release in 16 mm,
   then a lift (the path is 0 mm³).
5. **The carapace ceiling** over the tray is z 49.4, with two kinds of low
   spot *(review, re-mapped on a 4 mm grid against the current `part_shell`)*:
   - z 38.4 at r 59.5, at five azimuths (62, 134, 206, 278 and 350°, ± 2).
     It is clear by r 55.
   - Five pads, new in this D063 round, hang to z 45.4 at r 38–53 on the
     station azimuths (90, 162, 234, 306, 18 ± 8°). The old carapace was
     49.4–52.2 there.

   Anything tall at the tray's corners must stay inside r ≈ 55. Anything
   taller than z 45 must also stay off the station azimuths.

The options below differ in **where the harness hub lives** (star board +
12 V node): A under the deck, B on the deck under a raised tray, C inside
one enclosed belly pan.

## 2. What all three share

**The tub (the bay as a part, B84).** Two printed parts + a door:
`bay_tub` (a U: floor with the two sled rails integral, two walls, the nose
wall) and `bay_lid` (the roof, carrying the six hanger bosses). Print sizes
196.4 × 54.8 × 35.4 and 196.4 × 54.8 × 10 (lid + bosses, A and C; 7 for
B); 56 g with the door at fill 0.5.

| item | value |
|---|---|
| pose | pack centre (−12, −20); tub x −97.5 (door face) … 101.9, y −47.4 … 7.4 |
| roof top z | −18.0 (A, C: an 8 mm layer under the deck) or −15.0 (B: 5 mm) |
| bottom z | −55.4 (A, C) / −52.4 (B); ground clearance at stance 62.6 / 65.6 (as drawn: 78) |
| interior | 194.0 × 50.0 × 33.0 (door face to nose wall; walls and floor 2.4, roof 2.0) |
| height stack | floor → sled 2.25 (the rail, now counted) + sled 3 + pack 25 + strap 1.5 = 31.75 of 33 |
| hangers | six Ø8 bosses to the deck bottom, M3 heat-set inserts, at (±40, −20), (±40, 0) (each shared with a tray-rail tab: one M3 clamps tab + deck + boss) and (±58, −18) |
| door | end door −x, 54.8 × 37.4 × 3; its boss presses the sled's tail lip (replaces `belly_door`). *(review)* **I3 latches cannot sit in the walls' ends.** The B87 strike (`iface.latch_strike`) is 10.8 × 12.2 and reaches r 6.1, so it needs a pad of about 14.6 × 14.6, ≥ 6.0 deep. The wall ends are 2.4 wide, the floor 2.4, the roof 2.0. Measured pads at the door end (x −94.5 … −88): outside the south wall, 0 mm³ with everything; outside the north wall, 102 mm³ into leg 1's drop keep-out; on the roof top, 99 mm³ into the carapace; under the floor, 0 mm³, but it takes 14.6 mm of ground clearance at that end. So: one latch in a south-wall boss and a hook tab at the door's north edge (owner decision 12) |
| loading | the sled slides out −x with the door off; the path is clear of the tub (0) and, at stance, of every leg except leg 1 at yaw +27 … +40° (normal gait yaw is about ±11°). *(review: the first run turned the legs about the body centre and gave leg 1 +14 … +33° and leg 2 −40 … −39°. With each leg yawing about its own station, as `check_assembly` poses it, leg 2 never blocks.)* |
| legs | the tub clears every stance leg at every yaw −40 … 40 (0, re-run with yaw about the station), the docking dance (0), the hanging drops (0), the carapace (0). *(review)* **It does not clear the legs' whole soft-limit box.** Legs 1–4 reach it with the femur steeply down and the knee folded back: hip −70 … −45, knee −130 … −90, yawed toward the tub, e.g. leg 1 at yaw −25, hip −65, knee −100. That is about 205 poses per leg on a 5° grid (capsules r 15 / 12). Leg 0 never reaches it. See section 10 |

**The dock (B86).** `dock_block` sits at the nose (x 73 … 99), mouth −x.
Its frame floor goes 3.2 → 1.75 mm (or a 1.45 mm pocket in the tub floor),
which puts its XT60 centre exactly on the sled's (dz 0.00, float ±0.8; today
1.45 apart). Two side flanges at its floor with Ø2.8 thread-forming bores
take M3 × 6 driven from below, through the tub floor. The bus-bar cavity
goes: the spare taps come off the node, after the fuse (closes B66).

**Fuse and loop key.** In the tub's nose compartment: 14.4 mm free over the
dock (x 73 … 99). The loop key's panel XT60 sits in the nose wall above the
dock, facing +x, reachable from outside with the carapace on (WIRING wants
exactly that). The socket's flange and the fuse holder over the dock are
VERIFY with the real parts; if the holder does not fit, it moves onto the
14 AWG run north of the tub (the key then sits a few cm ahead of the fuse).

**Sled straps (B93).** The strap slots move into the sled's side walls
(11 × 2.5 at sled z 5–7.5, two per wall), for a 10 mm hook-and-loop strap
≤ 1.5 mm thick that goes over the pack and never under the sled (under it,
it would cross the rails). Measured stack with the strap: 31.75 of 33.

*(review)* **The width does not fit.** The pack (`pack_w` 44) fills the sled's
44.0 interior, so a strap inside the walls has no room. Outside them, the sled
(48.8) leaves 0.6 mm per side in `bay_w` 50, against the strap's 1.5. The
height stack was checked; the width was not. Two ways out:

- **(a) Widen the tub:** `bay_w` 50 → 53, with the strap outside the walls:
  48.8 + 2 × (1.5 + 0.6). The tub's outer width becomes 57.8. Measured: it still
  clears the hanging drops (+5 mm) and the hook envelopes for a centre y
  −21.5 … −17.5, so y −20 stands, with 1.5 mm of slack each way instead of 3.
- **(b) Drop the strap inside the robot:** keep `bay_w` 50. The tub (floor,
  walls, roof 2.75 mm over the pack, door, dock) boxes the pack, and a 2 mm
  foam pad on the roof takes the play. The pack then rides loose on the sled
  whenever the sled is out.

This is owner decision 8.

**The tray (B51).** At (0, 0), turned 180° (bulkhead north, tongue south),
rails with their tabs inboard (hole 7.3 mm off the block axis → (±40,
−20/0/20)), and the wings cut into three 10 mm lugs with windows in the
channel lips: slide 16 mm, lift. Latch: the tongue's I3 insert bites a
strike in the deck at (0, −43) (geometry from B87's fix).

*(review)* Re-measured with the yawing leg as well: the coxa fork plus hip
servo, at yaw −40 … 40 on all five stations. Tray + Pi + rails are at 0 mm³
at rest, slid ±16 and lifted 5 … 45 (A and B). The slide room stays 36 north
and 22 south.

**Stand (B85).** The robot is held by its tub. A and B: a U-cradle on
today's skirt (floor x ±45, walls 25 tall with 0.5 mm to the tub; 110.5 cm³
vs 106.9 today). C: today's crown with its arms removed (91.7 cm³). In all
three the crown touches nothing but the tub (0 mm³ with the deck, bases,
hook envelopes, hanging drops and the carapace), so **the carapace can stay
on**, and both tub ends are free: the sled can be swapped on the stand.
Deck-bottom heights: 89.4 / 169.4 / 249.4 mm (A, C; B 3 mm lower) for base
+ crown / +1 section / +2, against 47 / 127 / 207 today. The legs' reach
below the deck is 172, so it stays "two sections for the whole envelope".

**Deck removals.** The four strap slots (the deck-strap fallback is gone,
closes B50), the power grommet (52, −6) and its two zip anchors (the power
now comes from under the deck), grid hole (0, −40) (it overlaps the tray's
latch strike).

## 3. Option A: keel tub + north shelf (recommended)

Under the deck: the tub, and a **north shelf** hung from (±40, 20),
(0, 20), (0, 40) that carries the star board (40 × 30 on its 34 × 24
posts) and the 12 V node, boards facing up, side-entry headers, 28 mm deep
(x ±45, y 12 … 54, z −38 … −10). On the deck: the tray alone, raised 2 mm.

Legs 0, 1 and 4 run straight from the shelf to their cutouts, below the
hook feet. Legs 2 and 3 cross over the tub in the 8 mm layer, in two lanes
5 × 7 mm (x ±44.6 … 49.6 south to y −38, out past the hook envelope at x
±51.8 … 57, then down to the cutout). The 14 AWG leaves the tub's north
wall at the nose and runs to the node along y 8–11, z −26 … −22.

| check (A, roof −18) | mm³ |
|---|---|
| tub + bosses × hook-dance envelopes (5) / coxa bases / deck / carapace | 0 / 0 / 0 / 0 |
| tub × hanging drops (+5 mm) / × stance legs, yaw ±40 | 0 / 0 |
| sled, pack × tub; dock (unsunk) × tub walls/roof | 0; 0 |
| sled slide-out (−x, 0–180 mm) × tub, bases, hooks, drops | 0 |
| tray, rails, Pi envelope × coxa bases, yaw servos, deck | 0 |
| tray + Pi slid ±16, then lifted 10–45 | 0 |
| *(review)* tray + Pi (the tray's 11 mm standoffs) × the current carapace | 0: margin 5.5 with the Pi's USB end −x (the z 38.4 spot at az 206), 12.25 with it +x |
| *(review)* tray + Pi + rails × the yawing fork + hip servo (yaw −40 … 40), at rest / slid ±16 / lifted 5–45 | 0 / 0 / 0 |
| *(review)* tub + door × the rebuilt deck (B87 strikes) and carapace | 0 / 0 |
| rails lifted 2.5 × tray (the lug is captive) | 583 (engaged) |
| shelf × hook envelopes / drops / tub / deck | 0 |
| lanes × hook envelopes / tub + bosses / deck / cradle / shelf | 0 |
| 14 AWG run × hooks / drops / deck / tub / cradle | 0 |
| U-cradle × tub / drops / deck / bases / hooks / carapace / shelf / sled path | 0 |

Margins:
- Carapace over tray + Pi: 5.5 mm *(review: 10.5 as first measured, with the
  Pi on 6 mm standoffs; the B90 tray raised it 5)*. It is 12.25 if the Pi's
  USB end faces +x.
- Screw-head room over the inboard tabs: 3.4 mm (M3 button 1.65).
- Room under the tray for the IMU nuts: 5.1 mm. B90's fixed tray hangs them
  2.2 under the plate.
- Lane side clearance: 0.5 mm to the Ø8 hanger bosses (1 mm more and it
  touches: 15 mm³).

**Deck holes (A).** Of today's 13 grid holes, 8 are reused, 4 stay unused
and (0, −40) goes; 6 features are new. Every hole checked: ≥ 2 mm to any other cut,
on the pentagon, not under a coxa plate or a carapace foot. *(review, re-run on the
rebuilt deck: the tightest is (0, 40), 2.5 mm from the engraved north arrow,
which is 1.2 deep and not through; the new holes are ≥ 3.3 mm from each other,
a pair the first run did not test.)*

| hole | Ø | gap to other cuts | job |
|---|---|---|---|
| (±40, −20), (±40, 0) | 3.4 (was 2.8 tap) | 16.9 | tray-rail tab + tub hanger, one M3 × 16 into the boss insert |
| (±40, 20) | 3.4 (was 2.8) | 14.0 | tray-rail tab + shelf hanger |
| (0, 20), (0, 40) | 3.4 (were the bracket's) | 5.4 / 2.5 | shelf hangers |
| (±58, −18) | 3.4, **new** | 9.7 / 8.7 *(review: 10.7 / 9.6 against the deck before B87's strikes)* | tub hangers |
| (0, 50) | 10, **new** | 3.6 | grommet: the tray trunk + tray XT30 down to the shelf |
| (±12, 50) | 3.4, **new** | 8.2 | zip anchors |
| (0, −43) | the I3 strike cutter, **new** (`iface.latch_strike(6)`: 10.8 × 12.2, r 6.1; not a Ø10.3) | 15.5 | the tray's latch strike (I3) |
| (0, 0), (±20, 0), (0, −20) | 2.8 | — | unused; keep as trim-cup points |
| (0, −40) | — | — | removed: it overlaps the latch strike |

**CAD changes by file (A).**

| file | change |
|---|---|
| `cad/params.yaml` | keys below |
| `cad/iface.py` | two keep-out helpers, leg frame: `leg_port_hook_envelope()` (the docking swing, bottom −13.8) and `leg_drop_keepout()` (the mated drop under the cutout, ±10.5, 35 deep); the deck, tub, shelf and stand check against them |
| `cad/part_bay.py` (new) | `bay_tub`, `bay_lid`, `bay_door` + the checks in section 7 |
| `cad/part_battery.py` | sled strap slots into the side walls; `sled_rail` retired (the rails are the tub's); `dock_block` floor 1.75 + two flanges, bus-bar cavity dropped; `belly_door` retired; the B72 check counts rail + strap and poses the sled in the tub |
| `cad/part_deck.py` | the hole table above; strap slots, the power grommet + anchors and (0, −40) removed; `grid_holes()` becomes an explicit list |
| `cad/part_avionics.py` | `TRAY_XY` (0, 0), rotation 180, tabs inboard, channel +2 mm, lugs + lip windows; checks in section 7; the half-span check counts the tabs (58.3 today) |
| `cad/part_busboard.py` | becomes the hub shelf (plate on 4 standoffs under the deck, 34 × 24 star-board posts + a 30.5 × 30.5 node pattern); `layout_audit` re-targeted to the under-deck keep-outs |
| `cad/part_stand.py` | `stand_crown` → the U-cradle; `_layout_checks` → posed checks; the heights line |
| `cad/part_dock.py` | (B14's owner) its bay floor constant −40 → the tub bottom −55.4; half-bay from the tub's outer 27.4 |
| `cad/part_smallwins.py` | `belly_skid` retired (the tub floor is the skid, B2) |
| `cad/gen_assembly_views.py`, `cad/print_estimate.py` | `body_interior` redrawn; FILL + plates for `bay_tub`, `bay_lid`, `bay_door`, the shelf |
| docs, BOM | INTERFACES I4/I5, WIRING_HARNESS (under-deck routing, power-tree order, the new cut lengths), PRINT_PLAN, ASSEMBLY_GUIDE 4.3/4.5/4.6/5, DESIGN_BACKLOG; BOM: B-24 → a 30.5 × 30.5 FPV-style PDB (≥ 6 pad pairs, ≥ 15 A, no BEC), a strap row (10 mm, ≤ 1.5 thick, ×2), M3 heat-set inserts ×10, M3 × 16 ×10, M3 × 6 ×2 |

**Params keys (A).** `interfaces.battery_sled`: `bay_l` 175 → 194, `bay_h`
30 → 33 (`bay_w` 50 stays; 53 if decision 8 takes the strap outside the
sled walls *(review)*), new `bay_top_z` −18.0, `bay_centre` [−12, −20],
`bay_wall` 2.4, `bay_floor` 2.4, `bay_roof` 2.0, `strap_w` 10, `strap_t`
1.5. `interfaces.avionics_tray`: new `tray_xy` [0, 0], `tray_rot_deg` 180,
`tray_lift` 2.0, `lug_release` 16. `mass_hw.battery_3s_5200` 380 → 343 if
the owner takes the BOM pack's mass. `meta.params_rev` → D063 (the torso
mass and CoM move). I4 and I5 are frozen (D020): D063 revises them.

**Risks (A).** The two lanes are tight in plan (0.5 mm to the bosses): flat,
zip-tied looms; check with the real XH-5 + XT30 pair. The hub is serviced
from under the deck (on the stand the cradle leaves the north underside
open). The tub's ends reach past the deck outline, measured square to the
nearest edge *(review: the first draft gave the centreline figures only)*:

| tub end | centreline | south corner | north corner |
|---|---|---|---|
| door | 18.0 | 26.5 | 9.5 |
| nose | 22.2 | 30.7 | 13.7 |

The nose corners sit ~10 mm past the carapace skirt below it (chamfer them).
Belly falls load six M3 in heat-set inserts. 62.6 mm of ground clearance.
The fuse holder over the dock is VERIFY.

*(review)* Two more risks:
- **Legs vs the tub:** legs 1–4 can reach it inside their soft limits
  (section 2, "legs"). Nothing gates those poses: the sim has no belly body,
  and `pebble_feasibility` knows nothing of the tub.
- **The stand holds the tub in y only.** The cradle is open at both ends, so
  a leg test on the stand can walk the robot along x. Give it an x stop, for
  example a Ø6 pin in the cradle floor under a 1.5 mm blind hole in the tub
  floor at (0, −20).

**Effort (A):** medium-large. Four new printed parts (`bay_tub`, `bay_lid`,
`bay_door`, the shelf), seven CAD files changed, about fifteen new checks.

## 4. Option B: keel tub + deck-top core

The hub stays on the deck as WIRING_HARNESS has it, under the tray: a core
plate 44 × 74 on grid holes (0, −20), (0, 0), (0, 20), with the node south
(15 mm tall, under the IMU pad) and the star board north (17.6 mm). The
tray rides 18 mm higher on 26 mm rail blocks. Each leg's drop goes straight
down a Ø10 pass-through near its port, then under the deck to its cutout:
no crossing, so the tub hangs at the minimum −15 (65.6 mm clearance).

> *(review)* **B, as drawn, no longer fits.** Two things changed after it
> was measured: the tray's Pi now sits on 11 mm standoffs (B90, was 6), and
> the carapace grew five pads down to z 45.4 at r 38–53 on the station
> azimuths.
>
> - At lift 18 the Pi envelope enters the carapace by 492 mm³, with its USB
>   end either way. With the old 6 mm standoffs and the current carapace, the
>   margin would be 1.25 (USB +x) and −0.25 (USB −x).
> - A 3 mm margin needs the Pi top at z ≤ 42.4 under the pads, with the USB
>   end +x (with it −x, the z 38.4 spot at az 206 caps the top at 35.4 and
>   lift at 4.4). That means lift ≤ 11.4.
> - At lift 11.4 the tray's underside is 14.8 over the deck top. The
>   modelled core is 17.6 (star board) and 15 + 2.2 of IMU nuts (node).
>
> B survives only with a hub at most about 14.3 tall (12.1 under the IMU),
> for example the boards straight on the deck with right-angle headers. That
> is not measured here. Its numbers below are the first run's, kept for the
> record.

| check (B, roof −15, tray +18) | mm³ |
|---|---|
| tub checks as in A (hooks, bases, deck, drops, carapace, legs) | 0 |
| tray, rails × bases / yaw servos; tray × rails | 0 |
| tray + Pi × carapace | first run: 0 (margin 3.0 over the bulkhead, 5.0 over the Pi, but only with the Pi's USB end +x; turned −x it touches the carapace, 0.08 mm³, at any lift from 14 to 20). *(review: 492 mm³ with the current tray and carapace, either way; see the note above)* |
| core × rails / tray; IMU nuts × core | 0 / 0; 0 |
| each pass-through column × bases, tray, rails, core (above) and hooks, drops, tub (below) | 0 |
| tray slide room | 36 north / 22 south |

Deck (B): the six rail holes (4 through for the tub, 2 tap), (±58, −18)
new, the core's three tap holes, five Ø10 pass-throughs (0, 52), (±40, 36),
(±16, −56) (gaps 2.9 / 9.4 / 4.3 mm), the 14 AWG grommet Ø9 at (29, 14)
(6.6 mm). Files as A, except: the core bracket instead of the shelf, rails
26 tall, a carapace-margin check that fails under 3 mm, and the Pi's USB end
fixed at +x. Params as A with `bay_top_z` −15, `tray_lift` 18.

**Risks (B).** 3 mm under the carapace: a taller connector or cooler breaks
it. That has already happened: the B90 standoffs and the new carapace pads
broke it (see the note). The tray sits on 26 mm levers. The hub is reached only with the tray
off. The torso CoM is 7 mm higher than A's and the worst 4-foot margin the
lowest (54.1 with the hands). **Effort:** medium-large, like A, plus a hub redesign.

## 5. Option C: one belly pan

One pan hung below the hook feet: a pentagon R78 with its corners at the
webs (so its edge stays at r 63 on the stations, 12 mm inside the cutouts)
joined to the bay channel. It holds the sled, the hub (north compartment)
and the drops; legs 2 and 3 still cross in the 8 mm layer above its roof,
as in A. 139 cm³, 86 g, printed as a pan + lid, each 196.4 × 141.1. The
stand becomes today's crown without its arms: the pan rests on the Ø94
plate.

| check (C, roof −18) | mm³ |
|---|---|
| pan × hook envelopes / bases / deck / drops / carapace | 0 |
| pack, sled × pan | 0 |
| flat crown × pan / deck, bases, hooks, drops, carapace | 0 / 0 |

Deck (C): A's list without the shelf hangers, plus pan hangers (±20, 45)
and (0, −62) (gaps 15.1 / 9.2). **Risks:** +30 g over A's tub + shelf; a big
two-part print; the hub sits in a closed pan (service = lower the pan, or
add a floor hatch). **Effort:** large (one big new part), but the simplest
stand.

## 6. Side by side

| | as drawn today | A | B | C |
|---|---|---|---|---|
| fixes B51 B84 B85 B86 B92 B93 | no | yes, with the review's strap and door-latch fixes | **no longer fits** *(review)* | yes, as A |
| hub | deck, (0, 32) | under the deck, north shelf | deck, under the tray | in the pan |
| tray height over the deck | 3.4 | 5.4 | 21.4 | 5.4 |
| carapace margin over tray + Pi *(review: the tray's 11 mm standoffs, the current carapace)* | — | 5.5 (USB −x) / 12.25 (USB +x) | Pi into the carapace, 492 mm³ | 5.5 / 12.25 |
| belly clearance at stance | 78 (no part) | 62.6 | 65.6 | 62.6 |
| looms crossing over the tub | — | 2 (lanes 5 × 7) | 0 | 2 |
| new deck features | — | 6 | 8 | 9 |
| torso (g) / CoM (x, y, z) | 1417 / (0.0, −3.8, 6.1) | 1467 / (−2.5, −2.0, −1.2) | 1475 / (−2.5, −5.7, 5.9) | 1480 / (−2.2, −1.7, −2.3) |
| robot CoM (x, y, z) *(review: + the five hands)* | (0.0, −1.9, −6.8) | (−1.3, −1.0, −10.3) | (−1.3, −2.9, −6.6) | (−1.1, −0.9, −10.7) |
| worst 4-foot margin (of 57.2) *(review: + the hands)* | 55.6 | 55.6 | 54.1 | 55.8 |
| XH-5 looms legs 0–4 (mm) | 108/198/287/287/198 | 113/116/191/235/166 | 142/165/198/198/165 | ≈ A |
| XT30 pairs legs 0–4 (mm) | 205/294/233/144/115 | 113/166/235/191/116 | 186/193/155/155/193 | ≈ A |
| trunk (B67) / tray XT30 / 14 AWG | not computed | 129 / 143 / 159 | 84 / 123 / 210 | ≈ A |
| stand deck-bottom (base+crown / +1 / +2) | 47 / 127 / 207 | 89.4 / 169.4 / 249.4 | 86.4 / 166.4 / 246.4 | 89.4 / 169.4 / 249.4 |

CoM assumptions: pack 343 g (BOM B-05), `electronics_pod` 400 g split 250 on
the tray and 150 at the hub, printed parts at `print_estimate.FILL`, five
legs 1260 g at z −7 (stance estimate), plus the five hands.

*(review)* The first run left out `sim/mass_budget.json`'s hand (5 × 38.6 =
193 g). It is added here at z −100, the claw at the foot, which is an
estimate. The robot CoM z drops 6–7 mm in every option. The worst 4-foot
margin, which depends on x and y only, rises 0.1–0.2 mm (was 55.5 / 55.5 /
53.9 / 55.7). The torso rows need no change. The sim today puts a 1434.6 g torso
at (0, 0, 24.5): every real layout is 18–26 mm lower. Loom lengths use
WIRING_HARNESS's formula ((path + 25 + 20) × 1.15) and end under the cutout,
like its table; A's and B's paths are the drawn routes, not the r 62 ring.

## 7. Recommendation

**A, with the 8 mm layer (tub roof at −18).** The review re-measured it and
it still holds.

- It keeps the tray low: 5.5 mm under the carapace with the B90 tray, 12.25
  if the Pi's USB end faces +x. There is room for the IMU nuts, and the rails
  stay short.
- It gives the lowest CoM and the shortest looms.
- It leaves the deck top to the tray alone.
- It puts the hub where the leg drops are (under the deck) without
  enclosing it.
- Its weak point, the two lanes over the tub, is measured clear. The lanes
  are why the layer is 8 mm instead of 5: lanes 5 × 7 instead of 5 × 4, for
  3 mm of ground clearance.

It still needs two fixes from the review:
- the strap's width: `bay_w` 53, or no strap inside the robot;
- the door latch: in a boss outside the south wall.

B was the choice if the hub had to stay on top and be serviceable from
above. *(review)* It no longer fits the current tray and carapace, and only
comes back with a hub no taller than about 14 mm (section 4). C is the choice
if an enclosed belly matters more than 30 g and a big print.

## 8. Decisions only the owner can make

1. **Where the hub lives:** A (under the deck), B (on the deck under a
   raised tray) or C (in a belly pan). *(review: B only with a hub ≤ ~14 mm
   tall, not yet measured.)*
2. **How deep the tub hangs:** a 5 mm layer (roof −15, 65.6 mm clearance,
   lanes 5 × 4) or 8 mm (−18, 62.6, lanes 5 × 7). B needs only 5. *(prep
   2026-10-02: righting and rubble show no difference; the 5 mm layer leaves
   1.26 mm to the hook sweep with the seats fix against 4.26 for 8 mm, and
   takes a bare loom only (7 × 4); 8 mm is the measured choice.)*
3. **The loop key in the belly** (tub nose wall, +x, reachable with the
   carapace on) or on a lead up through the carapace.
4. **A separate `dock_block`** (replaceable; B86's flanges) or its floating
   carrier printed into the tub's nose wall (about 10 mm shorter tub).
5. **Pack length:** keep `pack_l` 138 (VERIFY maximum) or pin it to the
   bought pack's 132 (a tub 6 mm shorter).
6. **Pack mass in the model:** 380 (params) or 343 (the BOM pack).
7. **The tray's release:** lugs (slide 16, lift; keeps I4's C-channels) or
   drop-on pins + the latch (simpler; I4's text changes).
8. **The strap:** a 10 mm hook-and-loop strap ≤ 1.5 mm thick, two. *(review)*
   It needs width as well as height:
   - **(a)** `bay_w` 50 → 53 and the strap outside the sled walls (the tub
     is 57.8 wide and still fits at y −20);
   - **(b)** no strap inside the robot: the tub boxes the pack, with a 2 mm
     foam pad on the roof.

   *(prep 2026-10-02: the wider tub of (a) costs the uniform-cut option of
   decision 13 (legs 1/4 reach it 1.4° sooner), leaves the 14 AWG feed no
   route past leg 4's drop (B133), fits all three boards on the shelf only
   moved 0.70 north with a 0.50 gap, and halves the slack to the leg drops;
   it changes no righting outcome. (b) keeps every margin.)*
9. **The 12 V node part:** a 30.5 × 30.5 FPV-style PDB (the mount is the
   pattern, so the pick does not move the CAD) or bus bars / lever blocks
   (then a new mount).
10. **The keel look:** the tub ends 18–22 mm past the deck outline on the
    centreline and 26.5 / 30.7 mm at the south corners *(review)*, with the nose
    corners ~10 mm past the skirt: accept, or chamfer the nose.
11. **I4 and I5 are frozen (D020):** D063 revises the bay size, the tray pose
    and the tab side. Sign-off.
12. *(review)* **How the door is held.** The I3 strike needs a pad of about
    14.6 × 14.6, 6 deep. Places at the door end:
    - **south-wall boss:** clears everything (0 mm³);
    - **under-floor boss:** clear, but −14.6 mm of ground clearance at that end;
    - **north wall:** 102 mm³ into leg 1's drop keep-out;
    - **roof top:** 99 mm³ into the carapace.

    Suggested: one latch in the south boss and a hook tab at the door's
    north edge, or a second latch under the floor.
13. *(review)* **The legs' soft limits vs the tub.** Legs 1–4 can fold into
    it (hip −70 … −45 with the knee −130 … −90). Either give the sim a belly
    body and teach `pebble_feasibility` the tub, or narrow the soft limits
    for those combinations. *(prep 2026-10-02, measured: only the righter's
    fall path reaches the tub (12/200 drops with the battery mass, max
    3.06 mm, outcome unchanged). A uniform hip limit costs 12–15 % of the
    reach, 17–39 mm on a 20 mm stair and, at bay_w 53, switches off the
    cliff void guard (B129). A hip clamp on the righter's commands in
    FALLEN/RIGHTED only gives 0/200 contacts with the outcome unchanged
    (B130). The measured path: model the belly, then that clamp; re-derive
    the value for the fitted foot (B131).)*
14. *(2026-09-30, B108)* **Where the three small power and bus boards go**
    (the bus adapter, the 5 V buck, the 6 V UBEC; none fits the tray as
    drawn). *(prep 2026-10-02, measured: Pi on 18.5 mm standoffs fails the
    3 mm carapace rule with the real Pi 5 parts; Pi on 13 mm leaves no USB-A
    port that takes a plug reliably (B128). The measured home is all three
    boards face-down on option A's hub shelf with the Pi kept at 11 mm
    (bay_w 50: 0 mm³, 0.80 margin; the adapter alone also at 53) and the bus
    adapter driven over the Pi's UART. The Pi's USB end: +x gives 6.84 mm to
    the carapace and one usable USB-A port with a right-angle plug ≤ 20 mm;
    −x gives 3.56 mm and straight plugs. B127, B128, B134 list what the
    build must add.)*
15. *(2026-10-02, B117)* **The I1 leg-port fix.** Today's port cannot dock.
    Three verified fixes ([archive/prep-2026-10-01/I1_DOCK_OPTIONS.md](prep-2026-10-01/I1_DOCK_OPTIONS.md)):
    **seats** (two 30° cone posts at (−32, ±17.3), slot 6.2 at x −47, a plate
    rib; recommended), **minimal** (slot 5.2, short foot, 3 mm tapered posts;
    play ±0.9°, plate still at SF 0.76), **sequence** (flat slide, chevron
    hook, knob skirts; print risk high). Seats changes three frozen values
    (`dowel_xy` → `seat_xy`, `hook_slot_x`, `hook_slot_w`), so it needs a
    D020 sign-off like decision 11. Print the port coupon pair first.

## 9. Rejected, with the measurement

| layout | why |
|---|---|
| bay with the deck as its roof | hits the hook-foot docking swing by 372 mm³; its pack alone by 23.6 mm³ |
| north-south bay, door on the south edge | leg 0's cutout is over its nose; the clear pose starts 49 mm past the south edge. (Its door path would be clear of every stance leg at every yaw. *(review: the first run said "blocked at \|yaw\| ≥ 26°", with the legs turned about the body centre.)*) |
| pack on the deck under the carapace | tray + pack do not fit under the 49.4 mm ceiling; I5's belly XT60 and the B14 dock would move |
| tray anywhere else | at (0, −38): 534 + 954 mm³ into bases 2 and 3; no pose with outboard tabs; (0, 0) with inboard tabs is the only clear one |
| star board beside the tray on the deck | the pockets left between the tray and the coxa plates are ~32 × 17 mm; the bracket needs 46 × 44 |
| deck-edge saddles on taller crown arms (A) | the arms at az 54 / 126 would run through the north shelf; the U-cradle holds the tub instead |

## 10. Found while measuring (outside these rows)

1. **The leg drops mate under the deck** (fact 1). WIRING_HARNESS's ring lane
   and cut-length route need rewriting in any option. *(review: ASSEMBLY_GUIDE's
   open list already says "nothing routes the r ≈ 62 lane under the coxa plate"
   (§5.6). What is new is the measurement and a route; it is backlog B109.)*
2. **I4's slide-out cannot happen with the legs docked** (fact 4): lugs or
   pins, in any option.
3. **The carapace drops to z 38.4 at r 59.5, az 208** (fact 5).
4. **The sim's torso CoM is 18–26 mm too high** for every real layout, and
   the sim has no belly surface (the tub is 62.6 mm over the ground). A
   model change (fingerprint) for later.
5. **`part_dock` (B14): the funnel never touches the belly.** Its rails top
   out 30 mm over the ground; the belly is 78 mm up at stance as drawn,
   62.6 with A.
6. **The deck strap slots serve nothing** once the bay exists (B50 closes).
7. **The dock's spare taps sit before the fuse** (B66): with the node after
   fuse and key, take them from the node (B66 closes).
8. *(review)* **The legs can reach the belly within their soft limits.** Legs
   1–4 get there with hip −70 … −45 and knee −130 … −90, yawed toward the
   tub, e.g. leg 1 at yaw −25, hip −65, knee −100. The femur/tibia capsules
   then enter the tub. The old 175 × 50 × 30 bay had
   the same exposure in a smaller form, and neither the sim (no belly body)
   nor `pebble_feasibility` models it. Rows for a righting or tuck motion
   need it (backlog B97).
9. *(review)* **The carapace grew five pads down to z 45.4** at r 38–53 on
   the station azimuths during this D063 round (fact 5). Any deck-top part
   taller than about z 42 must be checked against the current `part_shell`,
   not a cached one.

## 11. Checks the implementation must add

- `iface`: the hook envelope and the drop keep-out, exercised by
  `part_deck`'s audit.
- `part_bay`: tub × hook envelopes, drop keep-outs, deck, carapace = 0;
  sled + pack + strap × tub = 0 with the stack (rail counted) ≤ `bay_h`;
  XT60 centres within 0.3; the slide-out path × tub = 0; every hanger axis
  through a deck hole (ray test, as `part_busboard` does); tub × stance
  legs over yaw ±40 = 0; one solid per part.
- `part_avionics`: tray + rails × posed bases and yaw servos = 0; the lug
  release (±16, then lift) = 0; tab holes on deck holes (ray test); the
  half-span counts the tabs; tray + Pi vs the carapace (fail under 3 mm).
  *(review)* The Pi envelope must take `PI_STANDOFF_H` from `part_avionics`,
  not a copied 6. The first run's scratch copy did, and it overstated A's
  margin by 5 mm. Pose the yawing fork + hip servo against the lug release
  and lift as well.
- *(review)* the sled: the strap's width stack, (sled or pack) + 2 × strap ≤
  `bay_w` − 2 × fit, next to the height stack.
- *(review)* the door: its latch strike cutter inside its boss with ≥ 1.2 of
  wall; the boss × the drop keep-outs, hook envelopes and carapace = 0.
- *(review)* the stand: the tub shifted 1 mm along x also binds (an x stop),
  not only sideways.
- *(review)* the legs: the tub × each leg over the soft-limit box, or a
  feasibility rule that keeps them out of it.
- `part_stand`: crown × tub, deck, bases, hook envelopes, drop keep-outs,
  carapace = 0; the tub shifted 1 mm sideways binds (registration); the
  heights printed.
- the hub (shelf or core): its audit against the same keep-outs (A: the
  lanes and the 14 AWG run too).
- `part_battery`: the B72 check counts the rail and the strap; strap slot ≥
  `strap_w` + 1.

Suggested order: the owner's decisions → `iface` helpers + params →
`part_bay` + `part_battery` → `part_deck`, `part_avionics`, the hub →
`part_stand` → docs and BOM → print the tub, door and one sled as coupons
before the deck.

## Where the measurements came from

The scripts and sketches were scratch work, not in the repo. The review
rebuilt every part from the tree after the other D063 groups' edits. Before
building, re-measure against the tree as it is then: the carapace pads and the
B87 strikes both changed under the first run.
