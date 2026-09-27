# Place recognition (D057)

Which room is this, have I been here, and what changed here, answered from
the senses. The cockpit's scene memory is keyed by the world's name, which is
ground truth a real robot never has; place recognition keys it by what the
robot senses instead, and the operator names the places ("hey, I'm in the
basement at home", "hey, this is completely new", "hey, this master bedroom
has a new chair"). Rooms are enough for now; `parent` is reserved for a
home → room → spot hierarchy (B38).

It is **off by default**. Using it: `docs/COCKPIT_GUIDE.md` "Where am I" (also
in the cockpit page under **?**). This page is the developer reference: the
module, the cockpit's protocol, the limits and the bench.

| piece | file | tests |
|---|---|---|
| fingerprints, scoring, verdicts, change rules, persistence | `sim/place_memory.py` (pure Python + numpy: no MuJoCo, no network) | `sim/tests/test_place_memory.py` |
| when it looks, the second look, the eye's questions, memory binding, tools, routes | `sim/cockpit.py` (`recognize_place`, `_place_tick`) | `sim/tests/test_awareness.py` (headless, a fake eye) |
| talk-mode sentences | `sim/cockpit_brains.py` `place_intent` | `sim/tests/test_awareness.py` |
| the three-verdict bench | `sim/place_bench.py` | `sim/tests/test_place_bench.py` |

## 1. The module

`PlaceMemory(directory)` keeps `<directory>/places.json` the way the scene
memory keeps its files: atomic write, debounced, a file that does not load
moved aside as `*.corrupt-<time>`, `places.json.bak` before a wipe; no
directory keeps it in RAM. A place is an id (`place-<6 hex>`), a name (none
until the operator gives one), its visits, up to 8 samples of each
fingerprint (a new sample 0.98 alike an old one replaces it, otherwise the
oldest goes), the objects seen there in the map frame, and notes. Every
sense arrives as an argument; the embedder is a function the caller injects.

### The three fingerprints

- **Scan.** `scan_signature(angles, ranges, yaw)` turns one lidar sweep into
  72 numbers. The first 36 are the nearest return in each 10° of *map*
  heading (the ray's body angle plus the robot's yaw); the last 36 are the
  quantiles of all the ranges (a sorted range histogram with no yaw in it,
  so odometry yaw drift cannot touch it). Every value is log-scaled into
  [0, 1] (1 = nothing within 6 m), so a near wall moving 10 cm counts more
  than a far one. `scan_similarity` compares the map part at its best
  circular shift (a yaw offset of any size costs nothing, and the winning
  shift estimates it: `scan_align`, 10° resolution; a rectangular room can
  alias it by 180°) and the histogram part directly:
  d = 0.6 × d_map + 0.4 × d_hist, similarity = exp(−(d / 0.05)²).
- **Description.** The cosine of two `look` descriptions' embeddings
  (`ROCKY_EMBED_MODEL`, default `embedding`; the reference setup's is
  Qwen3-Embedding-0.6B, 1024 dimensions, on the CPU, about 50 ms, so it
  never loads anything on the GPU). Room descriptions are never unrelated
  texts, so the raw cosine is mapped onto a score (`desc_score`): cos 0.25 →
  0, 0.90 → 1. On the first bench run the right room's description scored
  cos 0.64 to 0.90+ (median 0.77: the model re-words the same view) and a
  wrong room's 0.60 to 0.89 (median 0.79). They overlap completely: **in
  these rooms the description does not tell one room from another.** The
  shallow map only stops a re-worded description of the right room from
  reading as a mismatch.
- **Radio.** `{bssid: rssi_dBm}`, compared as a weighted Jaccard over the
  access points (−100 dBm counts 0, −30 dBm counts 1, an access point one
  side did not hear counts 0). **The sim never produces one** (every sim
  caller passes `radio=None`). It is the real robot's Wi-Fi scan; it tells
  buildings apart much better than rooms, so it weighs little.

### Scoring

`recognize(scan_sig, desc_emb, radio)` takes, for every place, the
best-matching sample of each signal that both the place and the query have,
and combines them:

```
combined   = Σ wᵢ sᵢ / Σ wᵢ
w_scan     = 0.5 × clip(max(info(query), info(sample)) / 0.25, 0.2, 1)   (info = share of map bins with a return)
w_desc     = 0.35,  w_radio = 0.15
evidence   = min(1, Σ w / 0.5), capped at SINGLE_EVIDENCE = 0.45 when one sense was compared
confidence = 0.5 + (combined − 0.5) × evidence
```

The weights are assumed, not measured; `recognize` over 50 places × 8
samples takes about 28 ms. Thin evidence pulls the score toward 0.5, "can't
tell", and **one sense alone never says known**: a perfect lone sweep or a
perfect lone description stops at 0.725 and radio alone at 0.65, all below
the 0.75 that *known* needs. Each sense has a blind spot another covers: the lidar cannot tell
rooms of one shape apart, descriptions of one kind of room read alike, and
radio tells buildings apart, not rooms. A lone sense can still say *new* (a
combined score under 0.5 stays under 0.5). Two empty sweeps are **not
compared at all** (1.0 alike says nothing about open floor), so a
lidar-empty world is compared on the description alone and is never *known*
by the robot alone: at best ambiguous at 0.725 `[look only]`, until the
operator names it. An empty sweep against a place that has returns is still
compared (open floor is evidence against that room). The result's `signals`
lists the senses compared, and `verdict_text` adds `[scan only]`, `[look
only]` or `[radio only]`.

The verdicts, on the best place against the runner-up (a place with another
name; places that share a name never compete):

- **known**: best ≥ `KNOWN_T` 0.75 and at least `MARGIN` 0.08 ahead of the
  runner-up.
- **new**: best < `NEW_T` 0.5 with every stored place compared (`new` with no
  place id when nothing is stored yet).
- **ambiguous**: anything in between, including two places too close to
  call.
- **unknown**, confidence 0: no signal at all (`place: unknown (no
  signal)`), or some stored places share no sense with the query, so they
  cannot be ruled out (`place: unknown (2 places not comparable)`).

`confidence` is always the best place's match score, on *new* too: 0.41 on
*new* means "the nearest place I know is 0.41 alike". `verdict_text` is the
situation line's clause (at most 90 characters): `place: basement (0.91)`,
`place: NEW (best basement 0.41)`, `place: basement? (0.62, or bedroom
0.58)`, `place: kitchen? (0.72, or bedroom 0.55) [scan only]`, and on a
known place with a confirmed change `place: bedroom (0.88) — new here:
chair; missing: ball`.

**How the thresholds were set.** The description map (0.25 / 0.90) and the
kept `KNOWN_T` / `MARGIN` come from replaying the first bench run's recorded
first looks (the THRESHOLDS note in `sim/place_memory.py`, a max-min grid
over five slacks): all 18 revisits come out known (weakest 0.779, 0.029 over
the line), the never-seen rooms at most 0.346 (new), and room b at its
enrolment against room a 0.560–0.676 (ambiguous 3/3, never known). The
second run (§4) was the first measurement under that map.

**Enrolment.** A look sees only the eye's 86° cone, so a place first seen
from one heading knows that view only. The cockpit stores the arrival look
and its second look (after the 30° turn) as two samples of a new place
(`enroll_samples`); a look taken after the place was stored goes in with
`add_samples`. Two sweeps from one spot are 0.98 alike and keep one scan
sample; both descriptions stay.

### The two honesty rules

1. **A single glance never declares a change.** `checked_objects(place,
   seen, visible=..., mentioned=...)` turns the eye's per-name answers into
   *added*, *missing*, *present* and *unobservable* (presence per name, not
   per instance, so it reports no *moved*). *Missing* needs a "not there"
   answer, a stored spot the look could see (`visible(x, y)`), and a
   description that does not name the thing (`mentioned_names`; negated
   mentions such as "no ball" do not count): the eye and the words of one
   look disagreeing is no evidence of absence. `diff_objects` does the same
   for positioned objects (*same* within `MATCH_M` 0.3 m, *moved*,
   *missing*, *unseen*, *added* with `maybe_moved_from`). `confirm_diff(first,
   second)` keeps only what two looks agree on; what one look alone reported
   stays *pending*.
2. **Every verdict carries its confidence.** `verdict_text` always prints the
   number, and a change is only reported on a *known* place: a new or
   ambiguous one has nothing trustworthy to compare with.

## 2. In the cockpit

Switch it on with `./rocky.sh cockpit --recognize` or `POST /api/awareness
{"recognize": true}` (`false` switches it off; the page has no switch yet,
B38 (c)). Off, the memory is keyed by the world, and the situation line, the
system prompt and the models' tool list are the D056 ones, so the D055 /
D055a brain-bench scores still describe a cockpit with recognition off. The
routes do not check the switch: `/api/tool/places` and
`/api/tool/forget_place` still act on the stored places (`where_am_i` and
`name_place` answer that recognition is off), and `GET /api/memory` still
carries `places`.

- **When it looks.** Every world load or reset makes a recognition due. Once
  the robot stands still (1 s of sim time after the spawn, reflex NORMAL,
  idle, not paused; it gives up after 20 s and tries again), it takes one
  lidar sweep (with the sim's pose standing in for odometry, a perfect one)
  and one look through the vision role, embeds the description (5 s
  timeout; a failed embedding is a missing signal) and calls
  `PlaceMemory.recognize`. The look loads the vision model if the server has
  unloaded it and falls back along the vision chain (reference:
  gemma-4-12b, then gemma-4-26b-a4b), so with recognition on a world load or
  reset can move models on the GPU. A look that failed has no objects (not
  an empty list): it is never read as "the eye saw nothing there", asks the
  eye nothing, and the answer says the verdict rests on the lidar alone.
- **A world edit** keeps the robot's pose and its place: what follows is a
  change check of the place the robot is bound to, never a new verdict and
  never another place (`_place_kept`). The verdict stays that place with how
  alike the robot senses it now, and a note below 0.5 ("this place changed a
  lot"). Talk mode's "this X has a new Y" runs the same check
  (`place_check`).
- **The second look.** *Ambiguous* and *new* take a second look after a 30°
  turn to the left (the `turn` tool: one gesture, every guard; no turn when
  the operator stopped the robot during the recognition), and the two
  recognitions are combined (`combine_recognitions`: per place the mean of
  the two looks' confidences, the verdict by the same thresholds). A place
  still *new* after both is stored as `new place #k` with both looks as
  samples; its first objects are the eye's placed sightings the two looks do
  not contradict (`eye_objects`). *Known* records a visit. While the robot
  turns, the clause says so: `place: den? looking again (0.72, or workshop
  0.55)`, `place: NEW? looking again (best den 0.41)`, `place: den (0.88) —
  looking again to confirm 1 change`.
- **Changes are asked of the eye, not read from the prose.** On a known
  place the change check is a set of box questions: one per thing the place
  remembers (walls and doors left out: they belong to the room, the lidar's
  business) and one per thing the look's description mentions
  (`place_candidates`: the scene memory's object nouns among
  `mentioned_names`; the prose only proposes the question). Each is
  `Brains.detect(name, method="bbox")`, the question `find_object` asks,
  through the look's own vision model while the robot still stands where it
  looked, and a box is projected onto the floor (`sighting_to_map`; the
  vision bench measured 0.7–0.9° bearing and 3 cm distance error in clean
  renders). At most `PLACE_EYE_MAX` = 10 questions per look, up to
  `PLACE_EYE_NEW` = 5 of them about names the description mentions (asked
  first), each one a vision-model call. Only a parsed "not there" is a no,
  and only where one of the thing's stored spots lies in that look's view
  (86°, 0.15–2 m from the eye); an error, an unsure answer (below
  `FIND_MIN_CONF` = 0.4), a box with no floor point within 2 m, or a spot out
  of view claims nothing. **Occlusion:** a remembered spot hidden behind
  something is unobservable, never missing — its image point inside a box
  the eye drew on this look (widened by 0.02 of the frame each side), or a
  lidar return within 2° of its bearing and at least 0.15 m nearer (anything
  lidar-tall stands higher than the eye can see over, since the eye is
  0.21 m up and the second look turns in place, sharing the line of sight).
  Names the eye boxed but could never place (a cup on a table, a lamp beyond
  2 m) are kept as a place note (`eye: boxed here, never placed: …`), so a
  later visit counts them present, not new. Any change takes the second look,
  which asks the same questions again, and only what both looks agree on
  (the name, and the position within `PLACE_MATCH_M` = 0.5 m when both placed
  it; the 0.5 m is unmeasured) is declared and stored. One look's claim stays
  *pending* and the snapshot stays as it was. What each look asked and heard
  is in the answer's `eye_checks`.
- **The scene memory follows the place.** Until a verdict, a provisional
  RAM-only memory (`place-pending`) stands in; a known or new place binds it
  to `place-<id>`, saved as `<memory dir>/place-<id>.json` next to
  `places.json`, and what the provisional memory recorded since the spawn is
  carried over. An ambiguous verdict binds nothing until the operator names
  the place. When the operator corrects a wrong recognition (`name_place`),
  what the visit wrote into the wrongly recognised place is taken back
  (`_place_take_back`: a place stored on this visit is dropped, a known one
  gets its record from before the visit and its scene memory from the moment
  of the bind) and the records since the bind move to the right place.
- **Where it shows.** The situation line starts with the place clause
  (`place: recognising…` while it runs); the state feed carries `place`
  (`verdict`, `name`, `place_id`, `confidence`, `signals`, `text`, `status`,
  `looks`, `by`; `signals` in words: `scan + description`, `scan only`, `the
  operator's word`); `where_am_i`'s `detail` ends with the same (`[signals:
  scan + description; confidence 0.88]`). With reactions on, a new place and
  a confirmed new object each say `curious_question` (at most one reaction
  per 30 s).
- **Failures.** A recognition that is interrupted (no standstill within
  20 s, paused, moved while looking) or raises is tried again every 5 s for
  as long as recognition is on; a failed look is not a failure (the sweep
  decides alone, at best *ambiguous*). Only `_place_tick` itself raising
  `AWARE_FAILS_MAX` = 3 times (counted since the last spawn or result)
  switches recognition off, with a console line.
- **Tools** (`harness/capabilities.py`, D056): `where_am_i` (the last
  recognition and the place; it waits for one that is running and never
  looks by itself), `name_place(name, new?, rename?)`, `places`,
  `forget_place(name | 'here' | 'all')` (a pronoun forgets nothing;
  `places.json.bak` before a wipe; a spoken 'all' needs the wake word).
  They are offered to the cockpit's model brains, and by the MCP server over
  a cockpit, only while recognition is on: `GET /api/capabilities` then
  carries the `places` capability, and the MCP list follows it on the next
  `tools/list` or watcher poll (3 s, with a list-changed notification). The
  mock and the in-process sim never have them. `name_place` names an
  unnamed recognised place (or one stored on this visit); a name that is
  another place's, or that differs from the recognised place's own name, is
  a **correction** (this is that place, or a new one under the name, and the
  wrongly recognised place is put back as it was); `rename=true` renames the
  recognised place instead; `new=true` stores a different place, one per
  visit however often it is said. Talk mode maps the operator's sentences
  onto them (`place_intent`: "where am I", "I'm in the basement at home",
  "this is completely new", "call this place the study", "this master
  bedroom has a new chair", "what places do you know", "forget this place"),
  and the model brains' system prompt gets a place note.
- **Routes.** `GET /api/place` (the current answer and every place) and
  `POST /api/place {action: recognize {force?} | name {name, new?, rename?} |
  forget {name} | list}`.

## 3. Limits in MuJoCo

Measured 2026-09-25 with the robot at the spawn pose (the scan plane is
0.179 m above the floor, `sim_lidar.PUCK_DZ` = 0.06 m above the torso; a
sweep after 1 s of standing):

| preset | rays with a return (of 360) | map bins with a return (of 36) |
|---|---|---|
| room | 360 | 36 |
| obstacle course | 37 (its one 0.25 m wall) | 5 (0.14: 0.56 of a full sweep's weight) |
| flat, cliff, rubble field, rough terrain, stairs, slope 8 deg, icy floor | 0 | 0 |
| room a, room b, room a + chair, room b + chair | 360 | 36 |
| room c, room c + chair, room c - ball | 208 | 23 |
| room d | 292 | 30 |

- **8 of the 9 general presets give the lidar nothing or next to nothing.**
  In the seven empty ones recognition rests on the description, which did
  not tell the bench's rooms apart, and the robot never says *known* there
  by itself (§1). A verdict in a lidar-empty world is not measured. The
  place bench's rooms are built for the lidar (walls 0.25–0.5 m).
- **MuJoCo rooms are simple**: flat-shaded walls and boxes on a checker
  floor, a perfect ray-cast lidar, a clean 320 × 240 render. Every sim number
  is an upper bound for the real camera and puck, and none is measured on a
  walking robot's tilted sweep.
- **Rooms of one shape are near-twins to the lidar.** The scan similarity was
  calibrated on synthetic ray-cast rooms (360 rays, the `room` preset's
  3.2 × 2.6 m walls and pillars; helpers in `sim/tests/test_place_memory.py`):
  the same spot with 1 cm noise 0.9996; 0.1 m away 0.94–0.98; 0.3 m away
  0.61–0.83 (median 0.71); 0.5 m away 0.24–0.63, which is why a place keeps
  several samples; a room of another shape (5 × 4 m, a 6 × 1.2 m corridor,
  open floor) 0.005 or less; a 3 × 3 m room up to 0.73; the same walls with
  other furniture 0.84 (the caster is 2D, so all of that furniture is
  lidar-visible). In MuJoCo, rooms a and b (b adds a 0.9 m inner wall and two
  0.22 m boxes) are 0.49 alike at the spawn and 0.53 at the bench's
  enrolment. Furniture below the 0.179 m plane is invisible to the lidar.
- **The description cannot break a lidar twin.** A room of the same shape
  whose furniture the lidar sees alike, described the way the bench's looks
  are (cos about 0.82), reads about 0.85 and comes out *known*; no
  description map fixes it. The radio on the robot, or the eye's per-object
  answers, could; neither is measured.
- Every world load puts the robot back at the origin facing +x, the spot it
  first saw the place from, so a sim revisit is an **upper bound** for a
  robot that comes back from anywhere.
- Object positions are in the map frame of the visit. In the sim that is the
  world frame, so they compare directly; a real robot's map frame restarts
  every session, and `scan_align` gives only the yaw between two visits, not
  the translation.

## 4. The place bench

`sim/place_bench.py` scores the three sentences from the senses alone: known,
new, and known with a change. It measures the cockpit's own protocol: it
stages a world, waits and reads (`POST /api/place {action: recognize}`, then
`where_am_i` and the state feed's `place`), and never looks or turns for the
robot. Each trial starts from an empty place memory, enrols rooms a, b and c
and names them the way a truthful operator would (den / workshop /
playroom), then runs seven cases: `a same pose`; `b named as a` and `a named
as b` (each room loaded under the world name the other was enrolled with: a
recogniser that used the name would answer the other room, flagged
`name_leak`); `a moved + turned` (0.29 m away and a quarter turn, with
recognition off while it walks there); `d never seen` (must be new); `b +
chair` and `c - ball` (the change must be confirmed by the cockpit's second
look, counted only when the changed spot was in the eye's view before and
after the read; a change confirmed from one look counts as a broken two-look
rule). Every world is loaded under an opaque name (`pb-<8 hex>`) with a spec
that differs on every visit, so nothing keyed by a name or a spec can carry a
room across visits. It starts its own cockpit (default :8795; it refuses the
live cockpit's port, `ROCKY_COCKPIT_PORT`, default 8765) behind a loopback
fence that forwards only the vision model and the embedding model, so a
failing vision model cannot pull a 21 GB fallback onto the GPU.

```bash
.venv/bin/python sim/place_bench.py                          # own cockpit on :8795, 3 trials
.venv/bin/python sim/place_bench.py --trials 1 --no-embed    # the scan + the look, no description embeddings
.venv/bin/python sim/place_bench.py --dry-run --trials 1     # plumbing only: canned look, hashed embeddings, no GPU
.venv/bin/python sim/place_bench.py --trials 1 --cases "d never seen,b + chair"   # rooms a-c are always enrolled
```

`--vision-model ID` picks the vision role it measures (default: the first of
the vision fallback chain, lfm2.5-vl on the reference setup; it loads if it
is not loaded, and the fence keeps the fallbacks off the GPU). `--no-embed`
makes the fence refuse embeddings, so recognition runs on the sweep alone
(the look still runs: it feeds the changes), which can answer *ambiguous* or
*new* but never *known*. `--relook` turns 30° after each read and forces a
fresh recognition, to see whether it agrees (it adds samples to the places,
so it is off by default). `--url` points it at a cockpit you started
(loopback, never the live port; not fenced, and each trial wipes its places
with `forget_place all`, which keeps `places.json.bak`). It prints the case
table, a confusion matrix, a per-read confidence table (the per-sense parts,
the evidence, the looks, the runner-up and the lead over it) and a Markdown
block for these docs, and writes `sim/out/place_bench.json` (`--out`: every
read with its raw answers, the change check's per-name eye answers for each
look, and the exact API calls). A `--dry-run` result is marked DRY RUN.
`sim/out/place_bench.json` is committed as the record of the last real run;
a new run overwrites it, so commit it with the docs that quote it. The bench
shares the GPU with any other cockpit: leave a running one alone while it
works (`docs/COCKPIT_GUIDE.md`, "Adding and measuring a model").

### The record

The committed record is the second run below. A first run the same evening
(19:39, lfm2.5-vl, before the fix round; its full numbers are in the D057
row of [archive/decisions_D036-D057_full.md](archive/decisions_D036-D057_full.md))
scored 15/21 verdicts with 0 wrong *known* and 0/6 changes: every miss was an
ambiguous revisit, a scan of 0.87–1.00 pulled under 0.75 by a description
score of 0.26–0.52 under the old 0.55 → 0 map, and the change check read
objects out of the look's prose, which places a thing only when one sentence
gives both a direction and a distance ("a few body lengths" does not), so the
chair was never placed and the ball could not go missing. The fix round
re-set the description map from that run's looks (§1), stores two looks at
enrolment and asks the eye for a box per name (§2).

**Latest run, 2026-09-25 21:15** (`sim/out/place_bench.json`): the bench's
own fenced cockpit, vision lfm2.5-vl, description embeddings on (48 through
the fence), 3 trials of the 7 cases, physics at 2.0×.

| case | expected | verdicts | confidence median (min–max) | second look | changes (two looks agree) |
|---|---|---|---|---|---|
| a same pose | known a | known ×3 | 0.96 (0.92–0.97) | 0/3 | none, none declared |
| b named as a | known b | known ×3 | 0.95 (0.94–0.97) | 0/3 | none, none declared |
| a named as b | known a | known ×3 | 0.96 (0.95–0.98) | 0/3 | none, none declared |
| a moved + turned | known a | known ×3 | 0.87 (0.86–0.88) | 0/3 | none, none declared |
| d never seen | new | new ×3 | 0.34 (0.33–0.37) | 3/3 | none, none declared |
| b + chair | known b | known ×3 | 0.86 (0.86–0.88) | 3/3 | 0/3 found |
| c - ball | known c | known ×3 | 0.86 (0.84–0.97) | 3/3 | 3/3 found |

- **21/21 verdicts, 0 wrong known, 0 false alarms** in the 15 unchanged
  visits, 0 changes declared from one glance, the name swap 6/6. Every true
  match clears 0.75 with room to spare (0.84–0.98); the never-seen room is
  new at 0.33–0.37, 0.13–0.17 under the 0.5 line (its two nearest stored
  rooms scored within 0.01–0.06 of each other).
- **Changes 3/6.** The removed ball was caught every time: asked for a ball
  at its remembered spot, the eye said no on both looks. The added chair was
  never caught: the eye calls it a *box* (it is one), the first look boxed it
  and the second look, from the same spot after the 30° turn, denied it, so
  the two-look rule held it back.
- **Enrolment:** room b, which shares room a's walls, was ambiguous when
  first seen (0.645, 0.657, 0.685) in all three trials; naming it settled it,
  and it was recognised 3/3 afterwards. Rooms a and c came back new.
- **Timing:** arrival to verdict 4.6 s median, 6.6 s p95 at 2.0× physics
  (settling, the sweep, the look, the embedding, and the turn + second look
  when one is taken: 9 of the 21 visits); a visit 5.0 s, a trial 48 s.

No look in this run asked more than 2 questions, so the 10-question budget
was never reached.

## 5. Open items (B38, B59)

In backlog order: the home → room → spot hierarchy; the radio fingerprint on
the real robot; a recognition switch and the place on the cockpit page; the
real camera and puck, with a translation between session map frames; an
added object is never confirmed (confirm from a sidestep viewpoint, and fold
names: chair ~ box ~ cube); widen the new-room margin (a second description
model, or the radio); say "ambiguous until named" on the situation line for a
room that shares another's walls. B59: on an ambiguous verdict,
`where_am_i`'s `place_id` / name is the bound place while `confidence`
belongs to the best candidate, so they can name different places.
