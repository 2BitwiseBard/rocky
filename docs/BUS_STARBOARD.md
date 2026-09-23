# Bus star-board build plan v0 (session 5) — the star-of-daisies hub

*Implements WIRING_HARNESS.md's "small printed star board" as a concrete
perfboard + printed bracket, SIZED FROM DECK v0.3 GEOMETRY. Positions are
a PROPOSAL until deck v0.4 freezes the tray/star-board/power-entry spots —
everything here states its assumed coordinates so the deck pass can adopt
or move them deliberately. Rule inherited from the harness plan: measure
on the printed deck before cutting wire; the table below turns "measure"
into "verify" (computed route ± a few mm beats guessing by 100).*

## Assumed deck layout (body frame, mm — PROPOSED)

- avionics tray (I4): plate center **(0, −38)**, bulkhead connector face
  south at y ≈ −71 (service through the south web; az 270 is a sector
  seam, so two latches open that corner of the shell).
- **star board: centered near (30, −6)**, flat on the printed bracket at
  12 mm standoff; the bracket's two deck tabs bolt to grid holes
  **(20, −20) and (40, −20)** (both inside the r < 56 grid, 20 mm pitch).
- power entry (fuse + rocker): deck grommet near **(52, −6)** feeding up
  from the belly dock (I5 nose points +X). E-stop reachable at the east
  web gap.
- 6 V BEC: velcroed to the tray floor next to the buck; its output rides
  to the star board with the adapter lead.
- loom ring lane: **r ≈ 62** — outboard of the electronics grid, inboard
  of the leg-port cable cutouts (r 81) and the shell feet (r 73–82, which
  sit at the webs only). Battery is BELOW deck (I5 belly bay), so the
  ring is unobstructed; keep clear of the strap slots (|x| ≤ 31, |y| ≤ 25)
  anyway — straps may be in use as the pack retention fallback.

## The board

Cut a 2.54 mm proto board to **40 × 30 holes-inclusive (16 × 12 holes)**;
drill corners Ø3.2 at **34 × 24 mm** for the bracket posts.

| ref | part | job |
|---|---|---|
| J0 | JST-XH-5 header, right-angle | input: data, GND, 6 V, sw, spare from adapter+BEC |
| J1–J5 | JST-XH-5 header, vertical | one per leg drop (leg number silk/sharpie!) |
| W1 | bus lane (26 AWG on board) | J0.1 → J1..J5 pin 1 (TTL data star) |
| W2 | GND lane | J0.2 → all pin 2 + bracket zip lug |
| W3 | 6 V lane (24 AWG) | J0.3 → all pin 3 |
| W4 | sensor lane | pins 4 bussed to J0.4 (microswitch wired-OR for now; per-leg lines when the Pi grows GPIO expander) |
| — | pins 5 | left open (reserved), continuity-test to nothing |

No resistors v0 (Feetech bus is push-pull from the adapter; the wired-OR
switch lane gets its 10 k pull-up ON THE PI header, not here).

## Printed bracket — `cad/part_busboard.py`

Plate 46×36×3 with 4× Ø5.6 posts (M3 thread-forming Ø2.8, 8 mm tall,
34×24 pattern), two deck tabs on the −y edge at **20 mm pitch** (M3 clear,
matching grid holes (20,−20)/(40,−20)), and two zip-tie wings for the
loom ring. Print flat, 4 perimeters, no supports.

## Cut-length table (computed from deck v0.3 geometry)

Route model: radial to the r=62 ring lane, arc along it (short way),
radial in to the leg-port cable cutout (r=81); allowances +25 mm station
termination, +20 mm source termination, ×1.15 service loop. **Verify on
the printed deck before cutting** — then write the real numbers next to
these.

| leg | station az | XH-5 loom (from star board) | XT30 12 V pair (from power node) |
|---|---|---|---|
| 0 | 90° | **240 mm** | **205 mm** |
| 1 | 162° | **310 mm** | **295 mm** |
| 2 | 234° | **220 mm** | **235 mm** |
| 3 | 306° | **130 mm** | **145 mm** |
| 4 | 18° | **150 mm** | **115 mm** |
| adapter → J0 | — | **125 mm** | — |
| BEC → J0 (6 V pair) | — | **125 mm** | — |

Longest stub: leg 1 at ~310 mm < the 400 mm bus-stub ceiling from the
harness plan ✓. Wire per the harness gauge table (data/GND/sensor 26 AWG,
6 V 24 AWG, 12 V 20 AWG silicone); heat-shrink color code + both-end leg
labels as specified there.

## Build order

1. Print bracket; bolt to grid (20,−20)/(40,−20); confirm the board sits
   clear of the tray rail sweep and the shell foot at az 342.
2. Populate J0–J5, run the four lanes, continuity-buzz every pin pair.
3. Cut looms per table (after deck verification), crimp XH one end only.
4. Dry-route all five + power pairs around the r=62 ring, THEN crimp the
   station ends at their true trimmed lengths.
5. D016 polarity check on every drop before any servo sees power.
