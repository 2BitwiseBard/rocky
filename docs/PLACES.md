# Place recognition (D057)

Which room is this, have I been here, and what changed here, answered from
the senses instead of the world's name (ground truth a real robot never has).
The operator names the places ("hey, I'm in the basement at home", "this is
completely new", "this master bedroom has a new chair"). It is **off by
default**. Using it: [COCKPIT_GUIDE.md](COCKPIT_GUIDE.md) "Where am I" (also
in the cockpit under **?**). This page is what a developer needs: how to turn
it on, what it does, the bench and the weak spots. The full design notes of
2026-09-25 are in the git history of this file and the D057 row of
[archive/decisions_D036-D057_full.md](archive/decisions_D036-D057_full.md).

## Turning it on

```bash
./rocky.sh cockpit --recognize
curl -s -X POST http://127.0.0.1:8765/api/awareness -H 'Content-Type: application/json' -d '{"recognize": true}'   # a running cockpit; false = off
```

The page has no switch yet (B38 c). Off, the memory is keyed by the world and
the situation line, the system prompt and the tool list are the D056 ones, so
the D055 / D055a brain-bench scores still describe it. On, it costs a lidar
sweep, a look (a vision-model call, which can load a model on the GPU) and an
embedding (the always-warm CPU `embedding` model, about 50 ms) after every
world load or reset.

| piece | file | tests |
|---|---|---|
| signatures, scoring, verdicts, change rules, persistence (`places.json`) | `sim/place_memory.py` (Python + numpy, no MuJoCo) | `sim/tests/test_place_memory.py` |
| when it looks, the second look, the eye's questions, memory binding, tools, routes | `sim/cockpit.py` (`recognize_place`, `_place_tick`) | `sim/tests/test_awareness.py` |
| talk-mode sentences | `sim/cockpit_brains.py` `place_intent` | `sim/tests/test_awareness.py` |
| the three-verdict bench | `sim/place_bench.py` | `sim/tests/test_place_bench.py` |

## What it does

**Three senses**, each compared only when both the place and the moment have
it. The **scan**: one lidar sweep as 72 numbers, the nearest return in each
10° of map heading plus a yaw-free range histogram, compared at the best
circular shift (`scan_align` estimates the yaw between visits, 10°
resolution). The **description**: the cosine of two `look` descriptions'
embeddings, mapped 0.25 → 0, 0.90 → 1. The **radio**: a Wi-Fi
`{bssid: rssi}` weighted Jaccard, on the real robot only (the sim passes
none). A place keeps up to 8 samples of each; the arrival look and the second
look are both stored at enrolment.

**Scoring and verdicts.** Per place the best sample of each shared sense,
weighted (scan 0.5 × its information, description 0.35, radio 0.15, all
assumed, not measured); thin evidence pulls the score toward 0.5, and **one
sense alone never says known** (a perfect lone sweep or description stops at
0.725, radio at 0.65). Two empty sweeps are not compared at all, so a
lidar-empty world is never *known* by the robot alone.

- **known**: best ≥ 0.75 and ≥ 0.08 ahead of the next place with another name;
- **new**: best < 0.5 with every stored place compared;
- **ambiguous**: anything between; **unknown** (0): no signal, or places that
  share no sense with the moment.

*Ambiguous* and *new* take a second look after a 30° turn left (the `turn`
tool, every guard) and the two are combined; still new, the place is stored
as `new place #k`. The verdict starts the situation line
(`place: den (0.91)`, `place: NEW (best den 0.41)`,
`place: den? (0.62, or workshop 0.58) [scan only]`) and rides on the state
feed as `place`. The scene memory follows the place (`place-<id>.json`).

**Two honesty rules.** (1) A single glance never declares a change: on a
known place the eye is asked one box question per remembered thing and per
thing the description mentions (at most 10 per look), a "not there" counts
only where the old spot is in view and not hidden behind a box or a nearer
lidar return, and only what two looks agree on is declared; one look's claim
stays *pending*. (2) Every verdict carries its confidence, and a change is
reported only on a known place.

**Tools** (`harness/capabilities.py`, offered only while it is on, over the
cockpit's brains and over MCP with a cockpit): `where_am_i`,
`name_place(name, new?, rename?)` (naming a place it was sure and wrong about
corrects it and takes the visit's records back), `places`,
`forget_place(name | 'here' | 'all')`. Routes: `GET /api/place`,
`POST /api/place {action: recognize | name | forget | list}`.

## The bench

`sim/place_bench.py` starts its own fenced cockpit (:8795; it refuses the
live port), enrols rooms a, b and c with truthful names, and runs seven
cases per trial: the same pose, two name swaps (each room loaded under the
other's world name, so a recogniser that used names would fail), a moved and
turned revisit, a never-seen room, a room with a chair added, a room with
the ball removed. Every world gets an opaque name and a fresh spec.

```bash
.venv/bin/python sim/place_bench.py                          # 3 trials, vision lfm2.5-vl on the reference setup
.venv/bin/python sim/place_bench.py --dry-run --trials 1     # plumbing only: canned look, hashed embeddings, no GPU
```

It writes `sim/out/place_bench.json` (committed: the last real run; commit a
new one with the docs that quote it). Not re-run on the D064 model (the
belly does not change the 117.1 mm standing height or the lidar plane).

**Last run, 2026-09-25 21:15** (lfm2.5-vl, 3 trials of 7, physics 2.0×):

| case | expected | verdicts | confidence median (min–max) | changes |
|---|---|---|---|---|
| a same pose | known a | known ×3 | 0.96 (0.92–0.97) | none declared |
| b named as a | known b | known ×3 | 0.95 (0.94–0.97) | none declared |
| a named as b | known a | known ×3 | 0.96 (0.95–0.98) | none declared |
| a moved + turned | known a | known ×3 | 0.87 (0.86–0.88) | none declared |
| d never seen | new | new ×3 | 0.34 (0.33–0.37) | none declared |
| b + chair | known b | known ×3 | 0.86 (0.86–0.88) | 0/3 found |
| c - ball | known c | known ×3 | 0.86 (0.84–0.97) | 3/3 found |

21/21 verdicts, 0 wrong *known*, 0 false alarms, the name swap 6/6; changes
3/6 (the removed ball every time; the added chair never: the eye calls it a
box, and the second look denied it). Room b, which shares room a's walls, was
ambiguous when first seen in all three trials (0.645–0.685) until named.
Arrival to verdict 4.6 s median, 6.6 s p95 at 2.0× physics. The first run the
same evening scored 15/21 and 0/6 before the description map and the eye's
box questions were fixed.

## Weak spots

- **The lidar sees little in most worlds.** Rays with a return, of 360, at
  the spawn (the scan plane 0.179 m up): `room` 360, `obstacle course` 37
  (its one 0.25 m wall), the seven other general presets 0; the bench's
  rooms a and b 360, c 208, d 292. In the empty ones the robot rests on the
  description, which does not tell the bench's rooms apart (a wrong room's
  cosine overlapped the right one's completely: medians 0.79 vs 0.77), so a
  verdict in a lidar-empty world is not measured.
- **Rooms of one shape are near-twins to the lidar** (0.84 alike with other
  furniture the lidar sees, on synthetic rooms; the bench's rooms a and b
  0.49), and the description cannot break the tie. The radio or the eye's
  per-object answers might; neither is measured.
- **An added object is never confirmed** (the chair case); confirming from a
  sidestep viewpoint and folding names (chair ~ box ~ cube) are open.
- **Every sim number is an upper bound**: flat-shaded rooms, a perfect lidar,
  a clean 320 × 240 render, the sim's pose as perfect odometry, and every
  revisit from the spot the place was first seen. A real map frame restarts
  every session and `scan_align` gives only the yaw between visits.
- Open, in backlog order (B38, B59): the home → room → spot hierarchy; the
  radio on the robot; the switch on the cockpit page; the real camera and
  puck; a wider new-room margin; "ambiguous until named" on the situation
  line; on an ambiguous verdict `where_am_i`'s place id can name the bound
  place while the confidence is the best candidate's (B59).
