"""place_memory — which place is this, have I been here, and what changed here.

The scene memory (sim/scene_memory.py) is keyed by the world NAME, which is
ground truth a real robot never has. This module is what replaces it: a
place is recognised from what the robot senses, and the owner names it
("hey, I'm in the basement at home" / "hey, this is completely new" / "hey,
this master bedroom has a new chair"). Rooms are enough now; a hierarchy
(home -> room -> spot) comes later (`parent` is reserved for it).

Pure Python + numpy: no MuJoCo, no network. Every sense arrives as an
argument (the cockpit computes it): a lidar sweep, an embedding vector of a
look description (the caller injects embed_fn, e.g. llama-swap's "embedding"
model), a radio fingerprint.

SIGNALS
  scan   scan_signature(angles, ranges, yaw, range_max) -> SIG_LEN floats:
         [0:MAP_BINS)   min range per 10 deg of MAP heading (bin k covers
                        [10k, 10k+10) deg CCW from map +x; body angle + yaw,
                        so the robot's own heading does not matter); a bin
                        with no return = range_max.
         [MAP_BINS:)    the heading-invariant part: the sorted ranges (the
                        range distribution's quantiles at (k+0.5)/INV_BINS,
                        i.e. a sorted range histogram) of every ray. It never
                        uses yaw, so odometry yaw drift cannot touch it.
         Every value is log-scaled, log1p(r / LOG_R0) / log1p(range_max /
         LOG_R0), so it lies in [0, 1] (1 = nothing within range_max) and a
         near wall moving 10 cm counts more than a far one.
         scan_similarity(a, b) in [0, 1] (see its docstring for the metric).
         In the sim the puck only sees what is taller than its scan plane
         (~0.18 m: the torso top + sim_lidar.PUCK_DZ 0.06 m; 0.179 m at the
         spawn, measured in the D057 docs pass — 0.12 m is
         sim_lidar.RANGE_MIN, not a height). Of the nine original presets
         `room` gives a full sweep, `obstacle course` a partial one (its
         0.25 m wall: 37 of 360 rays, 5 of 36 bins, same pass), and the
         other seven an EMPTY sweep — and empty
         sweeps all look alike. recognize() therefore weights a scan by how
         many bins got a return (scan_info), and two empty sweeps (the query
         and the place's best sample) are NOT compared at all: 1.0 alike
         says nothing, so a lidar-empty world rests on the look alone.
  desc   desc_similarity(a, b) = cosine of two description embeddings.
         recognize() maps it through desc_score (DESC_COS_FLOOR -> 0,
         DESC_COS_FULL -> 1): same-domain texts are never orthogonal, so raw
         cosine is not a match probability. Both constants are MEASURED
         (THRESHOLDS below): on the place bench every pair of look
         descriptions lay between cos 0.60 and 0.90+, the right room's and a
         wrong room's overlapping, so the map is shallow — a re-worded
         description of the right room no longer reads as a mismatch.
  radio  RadioSignature = {bssid: rssi_dBm}. radio_similarity = weighted
         Jaccard over the union of access points (weight = RSSI mapped
         RSSI_FLOOR..RSSI_FULL -> 0..1, an unheard AP = 0). THE SIM NEVER
         PRODUCES ONE: the cockpit passes radio=None and it is skipped. The
         real robot will (its Wi-Fi scan). Radio tells buildings apart much
         better than rooms, so its weight is small (and, like every sense,
         it can never make a place "known" on its own: SINGLE_EVIDENCE).

RECOGNITION  recognize(scan_sig, desc_emb, radio) compares the query with
  every place's stored samples (best sample per signal, up to N_SAMPLES of
  each per place) and combines the signals BOTH sides have:
      combined = sum(w_i * s_i) / sum(w_i)
      w_scan  = W_SCAN * clip(max(scan_info(query), scan_info(sample)) / INFO_FULL, INFO_FLOOR, 1)
                (no scan term at all when that max is 0: two empty sweeps)
      w_desc  = W_DESC,  w_radio = W_RADIO
      evidence = min(1, sum(w_i) / EVIDENCE_FULL)
      evidence = min(evidence, SINGLE_EVIDENCE)  when only ONE sense was compared
      confidence = 0.5 + (combined - 0.5) * evidence
  so thin evidence pulls a score toward 0.5 ("can't tell"). ONE SENSE ALONE
  NEVER SAYS "known": a lone scan, description or radio match is capped at
  SINGLE_EVIDENCE (0.45 -> confidence at most 0.725 < KNOWN_T), because each
  has a blind spot the others cover — the lidar cannot tell same-shaped
  rooms apart (same walls, other furniture: 0.84 alike on the synthetic
  rooms), descriptions of one domain read alike (on the place bench every
  pair of room descriptions was cos 0.60-0.90: THRESHOLDS), radio tells
  buildings apart, not rooms. So a failed look or a dead
  embedder leaves a scan-only answer that is at best "ambiguous", and the
  result says so: `signals` lists the senses compared and verdict_text adds
  "[scan only]". A lone sense can still say "new" (combined < 0.5 stays
  below 0.5). Two senses: an informative scan + a description is full
  evidence. An EMPTY sweep against a place's empty sample is no sense at
  all (parts['scan'] None): both are open floor, 1.0 alike, which tells
  one lidar-empty world from another no better than nothing. So the
  lidar-empty worlds (7 of the 9 original presets) are compared on the
  description alone and are NEVER "known" by the robot alone: at best
  "ambiguous" 0.725 "[look only]", and the owner names the place
  (name_place settles it). Review 2026-09-25: counting two empty sweeps
  as a 0.1-weight sense made that two senses — evidence 0.9 — and with
  the shallow description map every never-seen open-floor world read
  "known" as the first one stored (flat, then stairs, then the rubble
  field: one place whose objects each world overwrote; wrong-world look
  cosines on the bench were 0.60-0.89, cos 0.77 -> "known" 0.81). An
  empty sweep against a place WITH returns is still compared (open floor
  is not that room: evidence against). Radio alone tops out at 0.65; an
  empty sweep alone is "unknown" (nothing compared) once places exist.
  Verdict on the best place vs the runner-up (a differently named /
  different place):
      best >= KNOWN_T and best - second >= MARGIN  -> "known"
      best <  NEW_T                                 -> "new"
      otherwise                                     -> "ambiguous"
  No signal at all -> "unknown", confidence 0. No places yet -> "new".
  "new" needs EVERY stored place compared: a place that shares no sense
  with the query (an open-floor place vs an empty sweep with no look)
  may be this one, so the best compared place under NEW_T then reads
  "unknown" (`uncompared` counts those places).
  `confidence` is always the BEST place's match score: 0.91 on "known",
  0.41 on "new" means "the nearest thing I know is 0.41 alike". `second`
  and every `ranking` entry carry their own `parts` too, so a runner-up's
  score can be read without guessing its split.

THRESHOLDS — MEASURED on sim/out/place_bench.json (2026-09-25 19:39: 3
  trials, lfm2.5-vl looks, llama-swap's CPU Qwen3-Embedding-0.6B
  'embedding'; MuJoCo rooms a, b, c enrolled, d never; per read the parts
  are under runs[].tests[].reads[].recognize and runs[].enrol[].reads[]).
  That run used the old map (DESC_COS_FLOOR 0.55, DESC_COS_FULL 0.90), so a
  recorded desc score s gives its cosine back: cos = 0.55 + 0.35 s (s = 1.0
  only says cos >= 0.90: replayed as 0.90, the worst case for a match).
  Every read's FIRST look is complete (scan, desc). The 8 reads whose first
  look was ambiguous took a second look, and for those the JSON has the
  mean of the two looks' confidences with the FIRST look's parts only (the
  cockpit's combine_recognitions), so the replay is of first looks; a
  runner-up's desc is backed out of its confidence where its scan is known
  (a <-> b from the spawn: 0.526 in all three trials; c vs a or b: 0.0).
  What the run showed:
    scan  the right room 0.785-1.0 (1.0 from the enrolment pose, 0.785
          moved 0.29 m and turned 90 deg, 0.867 with the chair); another
          room 0.526 (a vs b: the same walls) or 0.0. The scan separates.
    desc  the right room cos 0.640-0.90+ (18 first looks, median 0.77:
          the model re-words the same view), a wrong room 0.598-0.894 (20
          pairs: best matches of never-seen rooms and backed-out runners-up,
          median 0.79). They overlap completely: the look embedding does NOT
          tell these rooms apart. Under the old map 6 of the 18 true
          matches fell below KNOWN_T on a re-worded description (first
          looks 0.656-0.746) — the 6 "ambiguous" of the run's 15/21.
  Chosen by a max-min grid (FLOOR 0.10-0.60, FULL 0.90-1.00, KNOWN_T,
  MARGIN) over five slacks: weakest true match - KNOWN_T, KNOWN_T - the
  one-sense cap (0.725), KNOWN_T - the same-shaped unseen room, NEW_T - the
  never-seen rooms, smallest true margin - MARGIN. FULL stays >= 0.90
  because a wrong room reached cos 0.894: a full desc score only above
  anything a wrong room reached (without that bound the optimum sets FULL
  below every observed cosine — desc = 1 for every pair, a constant, not a
  sense, and the margins go negative).
      DESC_COS_FLOOR 0.25 (was 0.55)  DESC_COS_FULL 0.90  KNOWN_T 0.75  MARGIN 0.08
  Replayed on that run's first looks:
    true matches (18)   all "known"; weakest 0.779 (trial 2 "b + chair":
                        scan 0.867, cos 0.674; 0.656 under the old map);
                        lead over the runner-up 0.224-0.460 on the 8 rows
                        whose runner-up split is known (one look, its scan
                        0.526 or 0.0), >= 0.121 on the other 10 (a bound
                        over the runner-up's unknown split; the two-look
                        rows from the mean of their looks)
    never seen (6)      d and c at its enrolment (scan 0.0): <= 0.346 ->
                        "new", 0.154 under NEW_T (was <= 0.290: the price of
                        the shallower map; a scan similarity of 0 — both
                        sweeps have returns, full weight — caps any room at
                        0.35 / 0.85 = 0.412 < NEW_T whatever it is called.
                        That bound needs returns: two EMPTY sweeps are not
                        compared, and the look alone is capped at 0.725)
    same shape, unseen  b at its enrolment vs a (scan 0.526, cos 0.65-0.83):
                        0.560-0.676 -> "ambiguous" 3/3 (was new 1/3,
                        ambiguous 2/3), never "known": 0.074 under KNOWN_T
  KNOWN_T keeps 0.025 over the one-sense cap and 0.029 under the weakest
  true match; the grid's optimum (FLOOR 0.22, KNOWN_T 0.755: min slack
  0.030) is within a rounding step of this (0.025), and KNOWN_T / MARGIN
  stay the numbers the cockpit and the owner already know. Limits: one run,
  3 trials of 7 cases whose looks repeat (the model describes a frame nearly
  the same way each time), clean MuJoCo rooms and a perfect lidar — an upper
  bound for the real camera and puck. Every bench room has walls (scan
  info >= 0.5), so none of this covers a lidar-empty world: those are
  never "known" by the robot alone (RECOGNITION). The cost: a description now counts
  for less when it disagrees — the synthetic same-walls twin (0.84 to the
  lidar) described as another room (bag-of-words cos 0.60) reads 0.717,
  still "ambiguous" but 0.033 under KNOWN_T (0.553 under the old map). And
  no map makes the description break a real twin: with bench-like
  descriptions (cos ~0.82) that twin reads ~0.85 "known" here and ~0.81
  under the old map. Telling twins apart needs another sense — the eye's
  per-object presence answers (checked_objects).

ENROLMENT. The look describes only the eye's 86 deg cone, so a place first
  seen from one heading knows that view only, and a revisit facing 30 deg
  further round reads another description. The cockpit stores the arrival
  look AND its second look (after the PLACE_TURN_DEG turn) as two samples
  of the new place:
      pid = mem.enroll_samples([first, second], objects=..., name=...)
  (first / second: the cockpit's observations {sig, emb, text, ...} as they
  are, or {scan_sig, desc_emb, desc_text, radio}); a look taken after the
  place was stored goes in with mem.add_samples(pid, [look]) — samples, not
  a visit. recognize() scores each signal against the place's BEST sample
  of it (the max over its scans, the max over its descriptions), so either
  view matches. Two sweeps from one spot (a turn in place) are DUP_SIM
  alike and keep one scan sample; the two descriptions both stay.

CHANGE  checked_objects(place, seen, visible=..., mentioned=...) is the check
  by the EYE: seen = {name: True | False | None}, the eye's box answer per
  name (True = boxed now, False = asked and not there, None = not asked / no
  answer) -> {added, missing, unobservable, present}; missing only where the
  answer was False, the stored spot is visible() and the same look's
  description (mentioned) does not name it — the eye and the words of one
  look disagreeing is no evidence of absence, and visit() keeps a named
  object, so a declared 'missing' would never be stored and would repeat
  every visit. The caller passes mentioned= (the look's description text or
  its mentioned_names). diff_objects(place, current,
  visible=..., mentioned=...) compares a look's POSITIONED objects with the
  place's object snapshot. A SINGLE LOOK NEVER PROVES ABSENCE: an object is
  "missing" only where the look could see (the visible(x, y) predicate) AND
  the look's description does not name it (`mentioned`: the description
  text — the parser places a thing only when a sentence gives a direction
  and a distance, so "a ball on the right" names the ball without placing
  it, and that is not absence); the cockpit takes a second look from another
  angle before it declares any change (confirm_diff keeps only what both
  looks agree on, from either check). visit() keeps a stored object its
  desc_text still names for the same reason. Object positions are in the
  PLACE frame = the map frame at that visit: in the sim the map frame is the
  world frame (the robot respawns at the origin facing +x), so they compare
  directly; a real robot's map frame restarts every session, and the cockpit
  must transform first (the scan alignment gives a yaw offset only —
  scan_align — not a translation).

PERSISTENCE  PlaceMemory(directory=...) keeps <directory>/places.json
  (atomic tmp + replace, debounced and off-thread, a corrupt file is moved
  aside), the same pattern as SceneMemory; directory=None keeps RAM only.
  Thread-safe (one RLock).
"""
from __future__ import annotations

import copy
import json
import math
import os
import re
import secrets
import threading
import time

import numpy as np

try:                                                   # the cockpit puts sim/ on sys.path
    from scene_memory import norm_name, same_thing
except ImportError:                                    # pragma: no cover — imported as a package
    from sim.scene_memory import norm_name, same_thing

VERSION = 1
SIG_VERSION = 1              # bump when the signature layout changes: stored scan samples are dropped
FILENAME = "places.json"

# ---- scan signature
MAP_BINS = 36                # 10 deg of MAP heading each
INV_BINS = 36                # quantiles of the range distribution (the heading-invariant part)
SIG_LEN = MAP_BINS + INV_BINS
BIN_DEG = 360.0 / MAP_BINS
RANGE_MAX = 6.0              # = sim_lidar.RANGE_MAX (not imported: sim_lidar needs MuJoCo)
LOG_R0 = 0.25                # m: the log scale's knee (near geometry weighs more)
W_MAP_PART = 0.6             # scan distance = 0.6 * map-part distance (best rotation) + 0.4 * invariant part
SCAN_D0 = 0.05               # distance scale of the similarity kernel exp(-(d / SCAN_D0)^2)

# ---- recognition
N_SAMPLES = 8                # samples of each signal kept per place
DUP_SIM = 0.98               # a new sample this alike an old one replaces it (keeps the 8 diverse)
KNOWN_T = 0.75               # MEASURED (THRESHOLDS): 0.025 over the one-sense cap, 0.029 under a match
NEW_T = 0.5
MARGIN = 0.08                # "known" also needs this lead over the runner-up (MEASURED: true leads >= 0.121)
W_SCAN, W_DESC, W_RADIO = 0.5, 0.35, 0.15
EVIDENCE_FULL = 0.5          # total signal weight that counts as full evidence (reached only with 2+ senses)
SINGLE_EVIDENCE = 0.45       # evidence cap with ONE sense compared: 0.5 + 0.5 * 0.45 = 0.725 < KNOWN_T
INFO_FULL = 0.25             # a sweep with returns in >= 25 % of its map bins is fully informative
INFO_FLOOR = 0.2             # a near-empty comparison (few bins with a return) keeps 20 % of the scan weight
SCAN_INFO_MIN = 1.0 / MAP_BINS   # a scan is compared only when one side has a return in >= 1 map bin:
#                              an empty sweep vs an empty sample says nothing (not a sense)
DESC_COS_FLOOR = 0.25        # MEASURED (THRESHOLDS): every look pair was cos >= 0.60; the map starts below
DESC_COS_FULL = 0.90         # MEASURED: a wrong room reached cos 0.894; only above that is a full match
RSSI_FLOOR = -100.0          # dBm -> radio weight 0
RSSI_FULL = -30.0            # dBm -> radio weight 1

# ---- change detection
MATCH_M = 0.3                # same name within this = the same object, not moved
MAX_PLACE_OBJECTS = 60
MAX_NOTES = 50
NAME_MAX = 40
TEXT_MAX = 300
VERDICT_MAX = 90             # verdict_text length cap (the situation line)
SAVE_DEBOUNCE_S = 0.5
_WIPE = {"all", "everything"}


# ------------------------------------------------------------ small helpers
def _num(v):
    if isinstance(v, bool) or v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _log_scale(r, range_max):
    r = np.clip(np.asarray(r, dtype=float), 0.0, range_max)
    return np.log1p(r / LOG_R0) / math.log1p(range_max / LOG_R0)


def _sig(v):
    """Anything array-like -> a float64 signature of SIG_LEN, or ValueError."""
    a = np.asarray(v, dtype=float).ravel()
    if a.shape != (SIG_LEN,) or not np.all(np.isfinite(a)):
        raise ValueError(f"a scan signature is {SIG_LEN} finite floats (scan_signature())")
    return a


def _unit(v):
    """An embedding -> a unit float64 vector, or ValueError (empty / zero / non-finite)."""
    a = np.asarray(v, dtype=float).ravel()
    n = float(np.linalg.norm(a)) if a.size else 0.0
    if a.size == 0 or not np.all(np.isfinite(a)) or n == 0.0:
        raise ValueError("an embedding is a non-empty, non-zero vector of finite floats")
    return a / n


# ------------------------------------------------------------------- scan
def scan_signature(angles_rad, ranges_m, yaw_rad, range_max=RANGE_MAX):
    """One lidar sweep -> a SIG_LEN float vector (layout in the module
    docstring). angles_rad: body-frame ray angles (sim_lidar.scan: [-pi, pi),
    CCW); ranges_m: the ranges, inf / nan / <= 0 / > range_max = no return;
    yaw_rad: the robot's map yaw (map heading = body angle + yaw)."""
    a = np.asarray(angles_rad, dtype=float).ravel()
    r = np.asarray(ranges_m, dtype=float).ravel()
    if a.shape != r.shape:
        raise ValueError(f"{a.size} angles but {r.size} ranges")
    rmax = _num(range_max)
    if rmax is None or rmax <= 0:
        raise ValueError("range_max must be a positive number")
    yaw = _num(yaw_rad)
    if yaw is None:
        raise ValueError("yaw_rad must be a finite number")
    keep = np.isfinite(a)
    a, r = a[keep], r[keep]
    hit = np.isfinite(r) & (r > 0) & (r <= rmax)
    r_eff = np.where(hit, r, rmax)
    heading = np.degrees(a + yaw) % 360.0
    idx = np.floor(heading / BIN_DEG + 1e-9).astype(int) % MAP_BINS
    bins = np.full(MAP_BINS, rmax)
    if a.size:
        np.minimum.at(bins, idx, r_eff)
        q = np.quantile(r_eff, (np.arange(INV_BINS) + 0.5) / INV_BINS)
    else:
        q = np.full(INV_BINS, rmax)
    return np.concatenate([_log_scale(bins, rmax), _log_scale(q, rmax)])


def scan_info(sig):
    """The fraction of map bins with a return (0 = an empty sweep: open floor,
    or nothing taller than the scan plane)."""
    s = _sig(sig)
    return float(np.mean(s[:MAP_BINS] < 1.0 - 1e-6))


def _align_many(a, bs):
    """One signature vs a stack of them -> (similarities (K,), drifts_deg (K,)).
    roll(a, -s) vs b is roll(b, s) vs a: the 36 shifts of the query are built
    once, so K samples cost one numpy op (50 places x 8 samples: ~ms)."""
    am, ai = a[:MAP_BINS], a[MAP_BINS:]
    bs = np.asarray(bs, dtype=float).reshape(-1, SIG_LEN)
    shifted = np.stack([np.roll(am, -k) for k in range(MAP_BINS)])            # (shift, bin)
    d_shift = np.mean(np.abs(shifted[None, :, :] - bs[:, None, :MAP_BINS]), axis=2)   # (K, shift)
    s_best = np.argmin(d_shift, axis=1)
    d_map = d_shift[np.arange(len(bs)), s_best]
    d_inv = np.mean(np.abs(bs[:, MAP_BINS:] - ai[None, :]), axis=1)
    d = W_MAP_PART * d_map + (1.0 - W_MAP_PART) * d_inv
    sims = np.exp(-(d / SCAN_D0) ** 2)
    drifts = (-s_best * BIN_DEG + 180.0) % 360.0 - 180.0
    return sims, drifts


def scan_align(a, b):
    """(similarity, yaw_drift_deg): the scan similarity (below) and how far
    b's map yaw is off from a's (b believed it faced yaw_drift_deg more CCW
    than it did at a), from the best circular shift of the map part — a
    10 deg resolution estimate, meaningful only when similarity is high. A
    symmetric room aliases it: in a rectangle the 180 deg turn fits almost
    as well (0.2 m off the spot, the synthetic 3.2 x 2.6 m room preferred
    it), so trust it only near a sample's spot or with a second cue."""
    sims, drifts = _align_many(_sig(a), _sig(b)[None, :])
    return float(sims[0]), float(drifts[0])


def scan_similarity(a, b):
    """Two scan signatures -> [0, 1], 1 = identical. The metric:
        d_map = min over the 36 circular shifts s of mean|a_map - roll(b_map, s)|
                (the best rotation: an odometry yaw offset of any size costs
                nothing, and the winning shift estimates it — scan_align)
        d_inv = mean|a_inv - b_inv|  (the L1 distance of two quantile
                functions = the Wasserstein-1 distance of the two log-range
                distributions; no yaw anywhere)
        d     = 0.6 * d_map + 0.4 * d_inv          (both in [0, 1])
        sim   = exp(-(d / SCAN_D0)^2), SCAN_D0 = 0.05
    Measured 2026-09-25 on SYNTHETIC ray-cast rooms (360 rays, the room
    preset's 3.2 x 2.6 m walls + pillars; the helpers are in
    sim/tests/test_place_memory.py): the same spot re-scanned with 1 cm
    noise 0.9996; 0.1 m away 0.94-0.98; 0.3 m away 0.61-0.83 (median 0.71);
    0.5 m away 0.24-0.63 — so a place keeps several samples; a room of
    another shape (5 x 4 m, a 6 x 1.2 m corridor, open floor) <= 0.005; a
    3.0 x 3.0 m room up to 0.73; the SAME 3.2 x 2.6 m shape with other
    furniture 0.84 — the lidar alone cannot tell same-shaped rooms apart
    (and the place bench found the look description cannot either:
    THRESHOLDS). On the place bench's MuJoCo rooms (the robot standing, the
    sim's pose as its odometry): the enrolment spot again 1.0, 0.29 m away
    and a quarter turn 0.785, a 0.25 m chair in view 0.867, the same walls
    with an inner wall 0.526, other rooms 0.0. An upper bound: unmeasured
    with a walking robot's tilt, and on the real puck."""
    return scan_align(a, b)[0]


# ------------------------------------------------------------ description
def desc_similarity(a, b):
    """Cosine similarity of two description embeddings, in [-1, 1]
    (0.0 when the dimensions differ: another embedding model)."""
    ua, ub = _unit(a), _unit(b)
    if ua.shape != ub.shape:
        return 0.0
    return float(np.clip(np.dot(ua, ub), -1.0, 1.0))


def desc_score(cos):
    """Cosine -> the [0, 1] match score recognize() uses (see DESC_COS_*)."""
    return float(np.clip((float(cos) - DESC_COS_FLOOR) / (DESC_COS_FULL - DESC_COS_FLOOR), 0.0, 1.0))


def embed_description(text, embed_fn):
    """text -> a unit numpy vector through the injected embed_fn(text) ->
    vector (the cockpit's call to llama-swap's 'embedding' model); None when
    there is no text or embed_fn fails (a missing signal, never an error)."""
    if not text or not str(text).strip() or embed_fn is None:
        return None
    try:
        return _unit(embed_fn(str(text)))
    except Exception:                                  # noqa: BLE001 — a dead embedder is a skipped signal
        return None


# ------------------------------------------------------------------ radio
RadioSignature = dict
"""{bssid: rssi_dBm}, e.g. {"a4:2b:b0:11:22:33": -48.0}. The real robot's
Wi-Fi scan; the SIM NEVER PRODUCES ONE (radio=None everywhere in the sim)."""


def clean_radio(radio):
    """A RadioSignature (or a list of (bssid, rssi) pairs) -> {lowercase bssid:
    rssi in [-120, 0]}, or None when there is nothing usable."""
    if radio is None:
        return None
    items = radio.items() if isinstance(radio, dict) else radio
    out = {}
    try:
        for k, v in items:
            f = _num(v)
            if k and f is not None and -120.0 <= f <= 0.0:
                out[str(k).strip().lower()] = round(f, 1)
    except (TypeError, ValueError):
        return None
    return out or None


def _rssi_w(v):
    if v is None:
        return 0.0
    return min(1.0, max(0.0, (float(v) - RSSI_FLOOR) / (RSSI_FULL - RSSI_FLOOR)))


def radio_similarity(a, b):
    """Weighted Jaccard of two RadioSignatures, in [0, 1]:
    sum_ap min(w_a, w_b) / sum_ap max(w_a, w_b) over the union of access
    points, w = the RSSI mapped RSSI_FLOOR..RSSI_FULL -> 0..1 (an AP one side
    did not hear = 0). None when either side has no usable AP."""
    a, b = clean_radio(a), clean_radio(b)
    if not a or not b:
        return None
    num = den = 0.0
    for k in set(a) | set(b):
        wa, wb = _rssi_w(a.get(k)), _rssi_w(b.get(k))
        num += min(wa, wb)
        den += max(wa, wb)
    return float(num / den) if den > 0 else None


# ------------------------------------------------------------------ objects
def _clean_objects(objects, now):
    out = []
    for o in objects or []:
        if not isinstance(o, dict):
            continue
        name = norm_name(o.get("name"))
        x, y = _num(o.get("x")), _num(o.get("y"))
        if not name or x is None or y is None:
            continue
        c = _num(o.get("confidence"))
        t = _num(o.get("t"))
        out.append({"name": name, "x": round(x, 3), "y": round(y, 3),
                    "confidence": round(0.5 if c is None else min(1.0, max(0.0, c)), 2),
                    "t": round(now if t is None else t, 2)})
    return out[:MAX_PLACE_OBJECTS]


def _obj_list(place_or_objects):
    if isinstance(place_or_objects, dict):
        place_or_objects = place_or_objects.get("objects")
    return [o for o in (place_or_objects or []) if isinstance(o, dict)
            and norm_name(o.get("name")) and _num(o.get("x")) is not None and _num(o.get("y")) is not None]


def _has_xy(o):
    return _num(o.get("x")) is not None and _num(o.get("y")) is not None


def _pairs(a_list, b_list, max_m, loose=False):
    """Greedy one-to-one pairs (nearest first) of same-named objects within max_m
    (None = any distance). Returns [(i, j, d)]. loose: an entry without a position
    (checked_objects' 'added' when the eye did not place it) pairs on the name
    alone (d = 0); otherwise every entry needs x and y."""
    cand = []
    for i, a in enumerate(a_list):
        for j, b in enumerate(b_list):
            if same_thing(a["name"], b["name"]):
                if loose and not (_has_xy(a) and _has_xy(b)):
                    d = 0.0
                else:
                    d = math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))
                if max_m is None or d <= max_m:
                    cand.append((d, i, j))
    cand.sort()
    used_a, used_b, out = set(), set(), []
    for d, i, j in cand:
        if i not in used_a and j not in used_b:
            used_a.add(i)
            used_b.add(j)
            out.append((i, j, d))
    return out


def _xy(o):
    return [round(float(o["x"]), 3), round(float(o["y"]), 3)]


# a mention is negated when a negation opens its clause at most two words before the
# noun ('no ball', 'not a ball', 'without the red ball'); 'not far from the ball' is
# still a mention. Narrower than objects_from_description's 24-character window on
# purpose: a wrong negation could let a thing be called missing, a wrong mention
# only keeps a claim back.
_SENT_RE = re.compile(r"(?<!\d)\.(?!\d)|[;!?\n]+")                # = objects_from_description's split
_CLAUSE_RE = re.compile(r"[,:()]|\b(?:but|and|or|while|whereas|though|although|except|plus|only)\b")
_NEG_BEFORE_RE = re.compile(r"\b(?:no|not|without|nothing|none|never)\b(?:\s+[a-z]+){0,2}\s*$")


def mentioned_names(text):
    """The names a look description mentions affirmatively -> a set of
    normalised words (norm_name: plurals and synonyms folded, 'cubes' ->
    'box'). Unlike objects_from_description it needs no direction or
    distance: 'a ball on the right' mentions the ball even though the
    parser cannot place it. Negated mentions ('no ball', 'not a box') are
    left out, so they can still count as absence. Every word goes in (not
    only LOOK_NOUNS): a stored object matches on its head noun."""
    out = set()
    for sent in _SENT_RE.split(str(text or "").lower()):
        for m in re.finditer(r"[a-z]+", sent):
            w = norm_name(m.group(0))
            if not w or w in out:
                continue
            before = sent[:m.start()]
            cuts = list(_CLAUSE_RE.finditer(before))
            if cuts:
                before = before[cuts[-1].end():]
            if _NEG_BEFORE_RE.search(before):
                continue
            out.add(w)
    return out


def _mention_set(mentioned):
    """diff_objects' `mentioned` -> a set of normalised names (None -> None = no information)."""
    if mentioned is None:
        return None
    if isinstance(mentioned, str):
        return mentioned_names(mentioned)
    try:
        return {n for n in (norm_name(x) for x in mentioned) if n}
    except TypeError:
        return None


def _is_mentioned(name, ment):
    """Does the mention set name this object (same head noun, same_thing)?"""
    if not ment:
        return False
    n = norm_name(name)
    head = n.split()[-1] if n else ""
    return bool(head) and any(m.split()[-1] == head and same_thing(n, m) for m in ment)


def diff_objects(place, current_objects, match_m=MATCH_M, visible=None, mentioned=None):
    """The place's object snapshot vs the objects a look found now (same
    frame) -> {added, missing, moved, same, unseen}:
      same     {name, x, y, m}          same name within match_m of where it was
      moved    {name, from, to, m}      same name, farther than match_m, and
                                        the old spot WAS visible (it is gone from there)
      missing  {name, x, y}             not found, visible(x, y) — the look could
                                        have seen it there — and not named by
                                        the look's description (`mentioned`)
      unseen   {name, x, y, mentioned?} not found, but the look could not see its
                                        spot, or its description names it without
                                        placing it (mentioned: True): nothing is
                                        claimed
      added    {name, x, y, confidence, maybe_moved_from?}  seen now, not before;
                                        maybe_moved_from = an unseen stored object
                                        of that name (it may be the same one)
    Names match with scene_memory.same_thing ('ball' ~ 'red ball').
    visible(x, y) -> bool says whether this look could have seen the point
    (the cockpit: inside the eye's view cone and range, not occluded);
    default: everything visible. mentioned = the look's description text
    (or an iterable of names): a stored object it names is never 'missing',
    because current_objects holds only what the parser could PLACE (a
    direction and a distance in one sentence) and a mention without them is
    not absence; None = no text (the old behaviour: absence of a placement
    counts). A single look never proves absence — the cockpit takes a
    second look from another angle (confirm_diff)."""
    stored = _obj_list(place)
    cur = _obj_list(current_objects)
    vis = visible or (lambda x, y: True)
    ment = _mention_set(mentioned)

    def can_see(o):
        try:
            return bool(vis(float(o["x"]), float(o["y"])))
        except Exception:                              # noqa: BLE001 — a broken predicate claims nothing
            return False

    out = {"added": [], "missing": [], "moved": [], "same": [], "unseen": []}
    close = _pairs(stored, cur, match_m)
    s_used = {i for i, _, _ in close}
    c_used = {j for _, j, _ in close}
    for i, j, d in close:
        out["same"].append({"name": norm_name(cur[j]["name"]), "x": _xy(cur[j])[0], "y": _xy(cur[j])[1],
                            "m": round(d, 3)})
    s_left = [i for i in range(len(stored)) if i not in s_used]
    c_left = [j for j in range(len(cur)) if j not in c_used]
    seen_left = [i for i in s_left if can_see(stored[i])]
    far = _pairs([stored[i] for i in seen_left], [cur[j] for j in c_left], None)
    for ii, jj, d in far:
        s, c = stored[seen_left[ii]], cur[c_left[jj]]
        out["moved"].append({"name": norm_name(c["name"]), "from": _xy(s), "to": _xy(c), "m": round(d, 3)})
    moved_s = {seen_left[ii] for ii, _, _ in far}
    moved_c = {c_left[jj] for _, jj, _ in far}
    for i in s_left:
        if i in moved_s:
            continue
        o = stored[i]
        e = {"name": norm_name(o["name"]), "x": _xy(o)[0], "y": _xy(o)[1]}
        if i not in seen_left:
            out["unseen"].append(e)
        elif _is_mentioned(o["name"], ment):          # named, just not placed: no claim of absence
            out["unseen"].append(dict(e, mentioned=True))
        else:
            out["missing"].append(e)
    for j in c_left:
        if j in moved_c:
            continue
        c = cur[j]
        a = {"name": norm_name(c["name"]), "x": _xy(c)[0], "y": _xy(c)[1],
             "confidence": round(_num(c.get("confidence")) or 0.5, 2)}
        maybe = [u for u in out["unseen"] if same_thing(u["name"], a["name"])]
        if maybe:
            a["maybe_moved_from"] = [maybe[0]["x"], maybe[0]["y"]]
        out["added"].append(a)
    return out


def confirm_diff(first, second, match_m=MATCH_M):
    """Two diffs of the same place from two looks (the second from another
    angle) -> the changes BOTH agree on: {added, missing, moved, pending}.
    pending = what only one look reported (not a change yet). Added / missing
    agree on name and position within match_m (on the name alone when either
    side has no position: checked_objects' 'added' the eye did not place);
    moved agrees on name and the new position within match_m. Takes
    diff_objects' or checked_objects' results (or one of each)."""
    out = {"added": [], "missing": [], "moved": [], "pending": []}
    for key in ("added", "missing", "moved"):
        a = [dict(o) for o in (first or {}).get(key) or []]
        b = [dict(o) for o in (second or {}).get(key) or []]
        if key == "moved":
            for lst in (a, b):
                for o in lst:
                    o["x"], o["y"] = o["to"]
        pairs = _pairs(a, b, match_m, loose=True)
        ia = {i for i, _, _ in pairs}
        ib = {j for _, j, _ in pairs}
        for i, j, _ in pairs:
            o = {k: v for k, v in b[j].items() if not (key == "moved" and k in ("x", "y"))}
            out[key].append(o)
        for lst, used in ((a, ia), (b, ib)):
            for n, o in enumerate(lst):
                if n not in used:
                    p = {k: v for k, v in o.items() if not (key == "moved" and k in ("x", "y"))}
                    out["pending"].append(dict(p, change=key))
    return out


def _presence(v):
    """A seen-map value -> (True | False | None, {x, y, confidence} it carries).
    True / False / None as they are; a dict (the cockpit's look['eye'][name]:
    {seen, x?, y?, confidence?, ...}) by its 'seen'; anything else -> None."""
    if isinstance(v, dict):
        a = v.get("seen")
        extra = {k: _num(v.get(k)) for k in ("x", "y", "confidence") if _num(v.get(k)) is not None}
        if "x" not in extra or "y" not in extra:
            extra.pop("x", None)
            extra.pop("y", None)
    else:
        a, extra = v, {}
    return (a if a is True or a is False else None), extra


def checked_objects(place, seen, visible=None, mentioned=None):
    """The place's object snapshot vs the eye's per-name PRESENCE answers ->
    {added, missing, unobservable, present}. seen = {name: True | False | None}
    — or {name: {seen, x?, y?, confidence?}}, the cockpit's look['eye'] as it is:
      True   the eye boxed it now (/api/look {find: name, method: "bbox"}: seen)
      False  asked, and the eye answered that it is not there
      None   not asked, or no usable answer (an error, an unparsed or unsure
             answer, a failed look): nothing is claimed
    Names match with same_thing ('ball' ~ 'red ball'); a stored name the map
    does not carry counts as not asked. When two keys fold to one name, a
    True outranks a False and a False outranks a None.
      present       {name, x, y}        stored, and the eye boxed it (True)
      missing       {name, x, y}        stored, answered False, and visible(x, y)
                                        (default: everywhere): the look could
                                        have boxed it there and said it is not
      unobservable  {name, x, y, why}   stored, nothing claimed: why = 'not asked'
                                        (no key), 'no answer' (None), 'out of
                                        view' (False, but visible() says the look
                                        could not see its stored spot), 'the
                                        description names it' (False, but the
                                        same look's description mentions it)
      added         {name, x?, y?, confidence?}  boxed (True), nothing of that
                                        name stored; x, y when the answer placed it
    mentioned = the SAME look's description text (or an iterable of names;
    None = no text): the eye's "not there" for a stored name the description
    names affirmatively is a conflict between two answers of one look, not an
    absence — unobservable, never missing, so it can not be declared. This is
    the rule visit() stores by (a stored object its desc_text still names is
    kept), so the check and the memory agree: without it the eye's "no" was
    declared 'missing' while visit() kept the object, and the same false
    'missing' came back on every visit (review 2026-09-25). A negated mention
    ('no ball') is not a mention (mentioned_names).
    Presence is per NAME, not per instance: one box boxed where two were stored
    marks both present, and a second box beside a stored one is not 'added' —
    diff_objects, on positioned sightings, is the tool for where. One look never
    proves a change: confirm_diff(check_1, check_2) keeps what two looks (the
    second from another angle) agree on."""
    def rank(a):
        return 2 if a is True else 1 if a is False else 0
    stored = _obj_list(place)
    ans, extra = {}, {}
    for k, v in (seen.items() if isinstance(seen, dict) else ()):
        n = norm_name(k)
        if not n:
            continue
        a, ex = _presence(v)
        if n in ans and rank(ans[n]) >= rank(a):
            continue
        ans[n], extra[n] = a, ex
    vis = visible or (lambda x, y: True)
    ment = _mention_set(mentioned)
    out = {"added": [], "missing": [], "unobservable": [], "present": []}
    matched = set()
    for o in stored:
        nm = norm_name(o["name"])
        keys = [k for k in ans if same_thing(nm, k)]
        matched.update(keys)
        vals = [ans[k] for k in keys]
        a = True if True in vals else False if False in vals else None
        e = {"name": nm, "x": _xy(o)[0], "y": _xy(o)[1]}
        if a is True:
            out["present"].append(e)
        elif a is False:
            try:
                ok = bool(vis(float(o["x"]), float(o["y"])))
            except Exception:                      # noqa: BLE001 — a broken predicate claims nothing
                ok = False
            if not ok:
                out["unobservable"].append(dict(e, why="out of view"))
            elif _is_mentioned(nm, ment):          # the eye says no, the same look's words name it
                out["unobservable"].append(dict(e, why="the description names it"))
            else:
                out["missing"].append(e)
        else:
            out["unobservable"].append(dict(e, why="no answer" if keys else "not asked"))
    for k, a in ans.items():
        if a is True and k not in matched:
            ex = extra[k]
            add = {"name": k}
            if "x" in ex:
                add.update(x=round(ex["x"], 3), y=round(ex["y"], 3))
            if "confidence" in ex:
                add["confidence"] = round(min(1.0, max(0.0, ex["confidence"])), 2)
            out["added"].append(add)
    return out


# ------------------------------------------------------------------ text
def _short(s, n=24):
    s = str(s or "?")
    return s if len(s) <= n else s[:n - 1] + "…"


def _label(d):
    return _short((d or {}).get("name") or (d or {}).get("place_id") or "?")


def _names(items, n=None):
    names = [str(o.get("name") or "?") for o in items]
    if n is not None and len(names) > n:
        return ", ".join(_short(x, 16) for x in names[:n]) + f" +{len(names) - n}"
    return ", ".join(_short(x, 16) for x in names)


_ONLY_WORD = {"scan": "scan", "desc": "look", "radio": "radio"}


def verdict_text(rec, diff=None):
    """The situation line's place clause (<= VERDICT_MAX chars):
      'place: kitchen (0.91)' | 'place: NEW (best kitchen 0.41)' |
      'place: kitchen? (0.62, or bedroom 0.58)' | 'place: unknown (no signal)' |
      'place: kitchen (0.88) — new here: box; missing: ball' |
      'place: kitchen? (0.72, or bedroom 0.55) [scan only]' |
      'place: unknown (2 places not comparable)'
    An unnamed place shows its id. A result that compared ONE sense (its
    `signals`, else its non-None `parts`) says so: '[scan only]' / '[look
    only]' / '[radio only]'. The diff is shown on "known" only (a new or
    ambiguous place has nothing trustworthy to compare with)."""
    rec = rec or {}
    v = rec.get("verdict")
    c = float(rec.get("confidence") or 0.0)
    sec = rec.get("second") or {}
    if v == "known":
        base = f"place: {_label(rec)} ({c:.2f})"
    elif v == "new":
        base = (f"place: NEW (best {_label(rec)} {c:.2f})" if rec.get("place_id")
                else "place: NEW (no places yet)")
    elif v == "ambiguous":
        base = f"place: {_label(rec)}? ({c:.2f}"
        base += f", or {_label(sec)} {float(sec.get('confidence') or 0):.2f})" if sec.get("place_id") else ")"
    elif rec.get("uncompared"):
        n = int(rec["uncompared"])
        base = f"place: unknown ({n} place{'s' if n != 1 else ''} not comparable)"
    else:
        base = "place: unknown (no signal)"
    sig = rec.get("signals")
    if not isinstance(sig, (list, tuple)):
        sig = _signals(rec.get("parts") if isinstance(rec.get("parts"), dict) else None)
    if v in ("known", "new", "ambiguous") and len(sig) == 1:
        base += f" [{_ONLY_WORD.get(sig[0], sig[0])} only]"
    if len(base) > VERDICT_MAX:
        base = base[:VERDICT_MAX - 1] + "…"
    if v != "known" or not diff:
        return base
    cats = [("new here", diff.get("added") or []), ("missing", diff.get("missing") or []),
            ("moved", diff.get("moved") or [])]
    cats = [(k, items) for k, items in cats if items]
    if not cats:
        return base
    for n in (None, 2, 1):
        s = base + " — " + "; ".join(f"{k}: {_names(items, n)}" for k, items in cats)
        if len(s) <= VERDICT_MAX:
            return s
    s = base + " — " + "; ".join(f"{k}: {len(items)}" for k, items in cats)
    return s if len(s) <= VERDICT_MAX else s[:VERDICT_MAX - 1] + "…"


# ------------------------------------------------------------------ memory
def _place_name(name):
    s = re.sub(r"\s+", " ", str(name or "")).strip().strip(".!?,;:\"'")
    return s[:NAME_MAX].strip()


def _signals(parts):
    """The senses a result compared, in a fixed order: ['scan', 'desc', 'radio'] or fewer."""
    return [k for k in ("scan", "desc", "radio") if (parts or {}).get(k) is not None]


def _look_signals(look):
    """One look of enroll_samples / add_samples -> (scan_sig, desc_emb, desc_text, radio).
    place_memory's own keys (scan_sig, desc_emb, desc_text, radio) or the cockpit's
    observation keys (sig, emb, text) — an observation dict goes in as it is."""
    if not isinstance(look, dict):
        return None, None, None, None

    def pick(*keys):
        for k in keys:
            if look.get(k) is not None:
                return look[k]
        return None
    return pick("scan_sig", "sig"), pick("desc_emb", "emb"), pick("desc_text", "text"), look.get("radio")


def _no_match(verdict, parts=None):
    return {"verdict": verdict, "place_id": None, "name": None, "confidence": 0.0,
            "second": {"place_id": None, "name": None, "confidence": 0.0, "parts": None},
            "parts": parts or {"scan": None, "desc": None, "radio": None}, "signals": [],
            "evidence": 0.0, "yaw_drift_deg": None, "ranking": []}


class PlaceMemory:
    """The places this robot has been (see the module docstring). A place:
    {id 'place-<6 hex>', name (None until named), parent (None; reserved for
    home -> room -> spot), created, visits, last_seen, scan_sigs [[SIG_LEN]],
    desc_embs [[dim] | None], desc_texts [str] (in step with desc_embs),
    radios [RadioSignature], objects [{name, x, y, confidence, t}] (the place
    frame = the map frame at that visit), notes [{t, text}]}.
    Every public method is thread-safe and returns copies."""

    def __init__(self, directory=None, clock=time.time, log=None):
        self._lock = threading.RLock()
        self.clock = clock
        self.directory = directory
        self._places = {}
        self._log = log or (lambda m: None)
        self._dirty = False
        self._timer = None
        self._pending = {}
        self.loaded_from = None
        if directory:
            self.load()

    # ----------------------------------------------------------- files
    @property
    def path(self):
        return os.path.join(self.directory, FILENAME) if self.directory else None

    def _parse(self, d):
        if not isinstance(d, dict) or not isinstance(d.get("places"), list):
            raise ValueError("not a places file")
        saved = _num(d.get("saved"))
        saved = self.clock() if saved is None else saved
        sig_ok = d.get("sig_version") == SIG_VERSION and d.get("sig_len") == SIG_LEN
        out = {}
        for p in d["places"]:
            if not isinstance(p, dict):
                continue
            pid = str(p.get("id") or "")
            if not re.fullmatch(r"place-[0-9a-f]{6}", pid) or pid in out:
                continue
            q = self._blank(pid, _num(p.get("created")) or saved)
            q["name"] = _place_name(p.get("name")) or None
            q["parent"] = p.get("parent") if isinstance(p.get("parent"), str) else None
            v = _num(p.get("visits"))
            q["visits"] = int(v) if v is not None and 0 <= v < 1e9 else 1
            q["last_seen"] = _num(p.get("last_seen")) or q["created"]
            if sig_ok:
                for s in p.get("scan_sigs") or []:
                    try:
                        q["scan_sigs"].append(_sig(s))
                    except (ValueError, TypeError):
                        pass
            embs, texts = p.get("desc_embs") or [], p.get("desc_texts") or []
            for k, e in enumerate(embs):
                try:
                    u = None if e is None else _unit(e)
                except (ValueError, TypeError):
                    continue
                q["desc_embs"].append(u)
                q["desc_texts"].append(str(texts[k] if k < len(texts) and texts[k] else "")[:TEXT_MAX])
            q["radios"] = [r for r in (clean_radio(x) for x in p.get("radios") or []) if r]
            q["objects"] = _clean_objects(p.get("objects"), saved)
            q["notes"] = [{"t": _num(n.get("t")) or saved, "text": str(n.get("text") or "")[:TEXT_MAX]}
                          for n in p.get("notes") or [] if isinstance(n, dict) and n.get("text")][-MAX_NOTES:]
            for key in ("scan_sigs", "desc_embs", "desc_texts", "radios"):
                q[key] = q[key][-N_SAMPLES:]
            out[pid] = q
        return out

    def load(self):
        """Replace the RAM contents with <dir>/places.json (empty when absent).
        A file that does not load is moved aside (*.corrupt-<time>), never lost."""
        with self._lock:
            p = self.path
            self._places, self.loaded_from = {}, None
            pend = self._pending.get(p) if p else None
        if not p:
            return 0
        try:
            if pend is not None:
                d = json.loads(json.dumps(pend))
            elif os.path.exists(p):
                with open(p) as f:
                    d = json.load(f)
            else:
                return 0
            places = self._parse(d)
        except Exception as e:                         # noqa: BLE001 — a bad file is never fatal
            bad = f"{p}.corrupt-{int(self.clock())}"
            try:
                os.replace(p, bad)
            except OSError:
                pass
            self._log(f"places: {p} did not load ({type(e).__name__}: {e}); moved to {bad}")
            return 0
        with self._lock:
            places.update(self._places)                # anything enrolled while the file was read
            self._places = places
            self.loaded_from = p
            return len(places)

    @staticmethod
    def _to_json(q):
        d = {k: q[k] for k in ("id", "name", "parent", "created", "visits", "last_seen")}
        d["scan_sigs"] = [[round(float(x), 4) for x in s] for s in q["scan_sigs"]]
        d["desc_embs"] = [None if e is None else [round(float(x), 5) for x in e] for e in q["desc_embs"]]
        d["desc_texts"] = list(q["desc_texts"])
        d["radios"] = [dict(r) for r in q["radios"]]
        d["objects"] = [dict(o) for o in q["objects"]]
        d["notes"] = [dict(n) for n in q["notes"]]
        return d

    def _payload(self):
        return {"version": VERSION, "sig_version": SIG_VERSION, "sig_len": SIG_LEN,
                "saved": round(self.clock(), 1),
                "places": [self._to_json(q) for q in self._places.values()]}

    def _write(self, p, payload):
        """Atomic write (tmp + replace). Never raises."""
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            tmp = f"{p}.tmp{threading.get_ident()}"
            with open(tmp, "w") as f:
                json.dump(payload, f, indent=1)
            os.replace(tmp, p)
        except OSError as e:
            self._log(f"places: could not save {p}: {e}")
            return None
        finally:
            with self._lock:
                if self._pending.get(p) is payload:
                    del self._pending[p]
        return p

    def save(self):
        """Write now. No-op without a directory."""
        with self._lock:
            p = self.path
            self._dirty = False
            if not p:
                return None
            payload = self._payload()
            self._pending[p] = payload
        return self._write(p, payload)

    def flush(self):
        with self._lock:
            t, self._timer = self._timer, None
            dirty = self._dirty
        if t is not None:
            t.cancel()
        if dirty:
            self.save()

    def set_directory(self, directory):
        """Turn persistence on (a directory: loads its places.json) or off (None)."""
        with self._lock:
            self.flush()
            self.directory = directory
        if directory:
            self.load()

    def _changed(self):
        """Debounced, off-thread save."""
        self._dirty = True
        if not self.directory or self._timer is not None:
            return
        t = threading.Timer(SAVE_DEBOUNCE_S, self._timer_fire)
        t.daemon = True
        self._timer = t
        t.start()

    def _timer_fire(self):
        with self._lock:
            self._timer = None
        self.save()

    # -------------------------------------------------------- records
    @staticmethod
    def _blank(pid, now):
        return {"id": pid, "name": None, "parent": None, "created": round(now, 2), "visits": 1,
                "last_seen": round(now, 2), "scan_sigs": [], "desc_embs": [], "desc_texts": [],
                "radios": [], "objects": [], "notes": []}

    def _new_id(self):
        while True:
            pid = "place-" + secrets.token_hex(3)
            if pid not in self._places:
                return pid

    @staticmethod
    def _push(lst, item, alike, extra=None):
        """Append a sample, bounded to N_SAMPLES: one DUP_SIM-alike to an old
        sample replaces it (fresher, and the kept samples stay diverse), else
        the oldest goes. extra: a parallel list kept in step."""
        for k, old in enumerate(lst):
            if old is not None and item is not None and alike(old, item) >= DUP_SIM:
                del lst[k]
                if extra is not None:
                    del extra[0][k]
                break
        lst.append(item)
        if extra is not None:
            extra[0].append(extra[1])
        while len(lst) > N_SAMPLES:
            del lst[0]
            if extra is not None:
                del extra[0][0]

    def _add_samples(self, q, scan_sig, desc_emb, desc_text, radio):
        if scan_sig is not None:
            self._push(q["scan_sigs"], _sig(scan_sig), scan_similarity)
        if desc_emb is not None or desc_text:
            e = None if desc_emb is None else _unit(desc_emb)
            txt = str(desc_text or "")[:TEXT_MAX]
            self._push(q["desc_embs"], e, lambda a, b: desc_similarity(a, b) if a.shape == b.shape else 0.0,
                       extra=(q["desc_texts"], txt))
        r = clean_radio(radio)
        if r:
            self._push(q["radios"], r, lambda a, b: radio_similarity(a, b) or 0.0)

    @staticmethod
    def _merge_objects(q, objects, visible, now, mentioned=None):
        cur = _clean_objects(objects, now)
        vis = visible or (lambda x, y: True)
        ment = _mention_set(mentioned)
        kept = []
        for o in q["objects"]:
            try:
                seen = bool(vis(o["x"], o["y"]))
            except Exception:                          # noqa: BLE001
                seen = False
            if not seen:
                kept.append(o)                         # out of this look's view: still believed there
            elif _is_mentioned(o["name"], ment) and not any(same_thing(o["name"], c["name"]) for c in cur):
                kept.append(o)                         # the text names it, nothing placed it: not gone
        q["objects"] = (kept + cur)[-MAX_PLACE_OBJECTS:]

    def enroll(self, scan_sig=None, desc_emb=None, desc_text=None, objects=None, radio=None, name=None):
        """A new place from what the robot senses now -> its id. Needs at
        least one signal (scan_sig, desc_emb or radio). objects: the look's
        [{name, x, y, confidence?, t?}] in the map frame (the place frame)."""
        if scan_sig is None and desc_emb is None and not clean_radio(radio):
            raise ValueError("enroll needs at least one signal: scan_sig, desc_emb or radio")
        now = self.clock()
        with self._lock:
            pid = self._new_id()
            q = self._blank(pid, now)
            self._add_samples(q, scan_sig, desc_emb, desc_text, radio)
            q["objects"] = _clean_objects(objects, now)
            if name is not None and _place_name(name):
                q["name"] = _place_name(name)
            self._places[pid] = q
            self._changed()
        return pid

    def enroll_samples(self, looks, objects=None, name=None):
        """A new place from SEVERAL looks -> its id: every look's signals go in as
        samples (in order: the arrival look first), so the place knows more than
        one view from the start — the cockpit's arrival look and its second look
        after the PLACE_TURN_DEG turn (the module docstring's ENROLMENT). looks:
        dicts with scan_sig / desc_emb / desc_text / radio, or the cockpit's
        observations {sig, emb, text, ...} as they are; None entries and looks
        with no signal are skipped. ValueError when no look carries a signal.
        objects: the place's first snapshot (the looks' objects merged once)."""
        sigs = [_look_signals(lk) for lk in (looks or []) if lk is not None]
        sigs = [s for s in sigs if s[0] is not None or s[1] is not None or clean_radio(s[3])]
        if not sigs:
            raise ValueError("enroll_samples needs a look with a signal: scan_sig, desc_emb or radio")
        now = self.clock()
        with self._lock:
            pid = self._new_id()
            q = self._blank(pid, now)
            for scan_sig, desc_emb, desc_text, radio in sigs:
                self._add_samples(q, scan_sig, desc_emb, desc_text, radio)
            q["objects"] = _clean_objects(objects, now)
            if name is not None and _place_name(name):
                q["name"] = _place_name(name)
            self._places[pid] = q
            self._changed()
        return pid

    def add_samples(self, place_id, looks):
        """More samples for a place WITHOUT a visit (visits / last_seen / objects
        untouched): a second look of the same stay — taken after the place was
        stored — or the looks name_place stores a place from. looks as in
        enroll_samples. Returns the place summary, None for an unknown id."""
        sigs = [_look_signals(lk) for lk in (looks or []) if lk is not None]
        with self._lock:
            q = self._places.get(str(place_id))
            if q is None:
                return None
            n = 0
            for scan_sig, desc_emb, desc_text, radio in sigs:
                if scan_sig is not None or desc_emb is not None or clean_radio(radio):
                    self._add_samples(q, scan_sig, desc_emb, desc_text, radio)
                    n += 1
            if n:
                self._changed()
            return self._summary(q, self.clock())

    def visit(self, place_id, scan_sig=None, desc_emb=None, desc_text=None, objects=None, radio=None,
              visible=None):
        """Another visit to a known place: appends the samples (bounded,
        N_SAMPLES each), bumps visits / last_seen, and — when objects is not
        None — updates the snapshot: what this look could see (visible(x, y),
        default everything) is replaced by what it found; the rest is kept,
        and so is a stored object the look's desc_text still names while
        `objects` places nothing of that name (a mention without a distance
        is not absence — see diff_objects). Pass objects only after a
        CONFIRMED look (confirm_diff): one glance must not rewrite the place.
        Returns the place summary, None for an unknown id."""
        now = self.clock()
        with self._lock:
            q = self._places.get(str(place_id))
            if q is None:
                return None
            self._add_samples(q, scan_sig, desc_emb, desc_text, radio)
            if objects is not None:
                self._merge_objects(q, objects, visible, now, mentioned=desc_text or None)
            q["visits"] += 1
            q["last_seen"] = round(now, 2)
            self._changed()
            return self._summary(q, now)

    def name_place(self, place_id, name):
        """Name (or rename) a place: True, False for an unknown id; ValueError for an empty name."""
        nm = _place_name(name)
        if not nm:
            raise ValueError("a place name needs at least one character")
        with self._lock:
            q = self._places.get(str(place_id))
            if q is None:
                return False
            q["name"] = nm
            self._changed()
            return True

    def note(self, place_id, text):
        """Keep an operator note on a place ('the charger is behind the door')."""
        t = str(text or "").strip()[:TEXT_MAX]
        with self._lock:
            q = self._places.get(str(place_id))
            if q is None or not t:
                return False
            q["notes"] = (q["notes"] + [{"t": round(self.clock(), 2), "text": t}])[-MAX_NOTES:]
            self._changed()
            return True

    def forget(self, place_id):
        """Forget one place (by id, or every place with that name) or all of
        them ('all' / 'everything' only; the old file is kept as places.json.bak).
        Returns how many places went (0 for anything else, None included)."""
        key = str(place_id or "").strip().lower()
        backup = None
        with self._lock:
            if key in _WIPE:
                n = len(self._places)
                if n and self.path:
                    backup = (self.path + ".bak", self._payload())
                self._places = {}
            elif key in self._places:
                del self._places[key]
                n = 1
            else:
                ids = [pid for pid, q in self._places.items() if key and (q["name"] or "").lower() == key]
                for pid in ids:
                    del self._places[pid]
                n = len(ids)
            if n:
                self._changed()
        if backup is not None:
            threading.Thread(target=self._write, args=backup, daemon=True, name="places-bak").start()
        return n

    # ---------------------------------------------------------- queries
    def get(self, place_id):
        """A deep copy of one place (numpy vectors included), or None."""
        with self._lock:
            q = self._places.get(str(place_id))
            return copy.deepcopy(q) if q is not None else None

    def find(self, name):
        """Ids of the places with this name (case-insensitive), newest visit first."""
        key = _place_name(name).lower()
        with self._lock:
            qs = [q for q in self._places.values() if key and (q["name"] or "").lower() == key]
            return [q["id"] for q in sorted(qs, key=lambda q: q["last_seen"], reverse=True)]

    @staticmethod
    def _summary(q, now):
        last = next((t for t in reversed(q["desc_texts"]) if t), "")
        return {"id": q["id"], "name": q["name"], "parent": q["parent"], "created": q["created"],
                "visits": q["visits"], "last_seen": q["last_seen"],
                "age_s": round(max(0.0, now - q["last_seen"]), 1),
                "samples": {"scan": len(q["scan_sigs"]), "desc": sum(e is not None for e in q["desc_embs"]),
                            "radio": len(q["radios"])},
                "objects": len(q["objects"]), "object_names": sorted({o["name"] for o in q["objects"]})[:8],
                "notes": len(q["notes"]), "last_description": last[:80]}

    def places(self):
        """Every place, most recently seen first: {id, name, parent, created,
        visits, last_seen, age_s, samples {scan, desc, radio}, objects,
        object_names, notes, last_description}."""
        now = self.clock()
        with self._lock:
            qs = sorted(self._places.values(), key=lambda q: q["last_seen"], reverse=True)
            return [self._summary(q, now) for q in qs]

    @staticmethod
    def _snapshot(q):
        """What scoring needs, copied under the lock (the vectors are never
        changed in place, only the lists): scoring then runs without it."""
        return {"id": q["id"], "name": q["name"], "last_seen": q["last_seen"],
                "scan_sigs": list(q["scan_sigs"]), "desc_embs": [e for e in q["desc_embs"] if e is not None],
                "radios": list(q["radios"])}

    @staticmethod
    def _score(q, scan, scan_inf, emb, radio):
        """One place snapshot vs the query -> (confidence, parts, evidence,
        drift) or None (nothing comparable)."""
        parts = {"scan": None, "desc": None, "radio": None}
        num = wsum = 0.0
        drift = None
        if scan is not None and q["scan_sigs"]:
            stack = np.stack(q["scan_sigs"])
            sims, drifts = _align_many(scan, stack)
            k = int(np.argmax(sims))
            info = max(scan_inf, float(np.mean(stack[k, :MAP_BINS] < 1.0 - 1e-6)))
            # two EMPTY sweeps are 1.0 alike and say nothing (every open floor looks like
            # this): not a sense, so not compared — a lidar-empty world rests on the look
            # alone, which SINGLE_EVIDENCE keeps below "known" (review 2026-09-25: an empty
            # sweep counted as a second sense and 'known' a never-seen open-floor world)
            if info >= SCAN_INFO_MIN - 1e-9:
                sim, drift = float(sims[k]), float(drifts[k])
                w = W_SCAN * min(1.0, max(INFO_FLOOR, info / INFO_FULL))
                parts["scan"] = round(sim, 3)
                num, wsum = num + w * sim, wsum + w
        if emb is not None:
            es = [e for e in q["desc_embs"] if e.shape == emb.shape]
            if es:
                sc = desc_score(float(np.clip(np.max(np.stack(es) @ emb), -1.0, 1.0)))
                parts["desc"] = round(sc, 3)
                num, wsum = num + W_DESC * sc, wsum + W_DESC
        if radio is not None and q["radios"]:
            rs = [r for r in (radio_similarity(radio, x) for x in q["radios"]) if r is not None]
            if rs:
                parts["radio"] = round(max(rs), 3)
                num, wsum = num + W_RADIO * max(rs), wsum + W_RADIO
        if wsum <= 0:
            return None
        combined = num / wsum
        evidence = min(1.0, wsum / EVIDENCE_FULL)
        if sum(v is not None for v in parts.values()) == 1:
            evidence = min(evidence, SINGLE_EVIDENCE)   # one sense alone never says "known"
        return 0.5 + (combined - 0.5) * evidence, parts, evidence, drift

    def recognize(self, scan_sig=None, desc_emb=None, radio=None):
        """Which known place is this? -> {verdict 'known'|'new'|'ambiguous'|
        'unknown', place_id, name, confidence, second {place_id, name,
        confidence, parts}, parts {scan, desc, radio} (the best place's
        per-signal scores, each against that place's BEST sample of the
        signal; None = not compared), signals (the senses compared for the
        best place, e.g. ['scan'] when the look or the embedder failed — one
        sense alone is capped below "known"), evidence, yaw_drift_deg (the
        best scan alignment; None without a scan), ranking [top 3 {place_id,
        name, confidence, parts}], uncompared (how many stored places shared
        no sense with the query — two empty sweeps are not a shared sense)}.
        The combination and the thresholds are in the module docstring;
        `confidence` is the best place's match score. "new" needs every
        place compared: when the best compared place is under NEW_T but some
        place could not be compared, the verdict is "unknown" (that place may
        be this one): no place claimed (place_id None, confidence 0, ranking
        [] as for every "unknown"), the compared places (top 3) kept under
        `compared` for the record."""
        scan = None if scan_sig is None else _sig(scan_sig)
        emb = None if desc_emb is None else _unit(desc_emb)
        rad = clean_radio(radio)
        if scan is None and emb is None and rad is None:
            return _no_match("unknown")
        scan_inf = scan_info(scan) if scan is not None else 0.0
        with self._lock:
            qs = [self._snapshot(q) for q in self._places.values()]
        if not qs:
            return _no_match("new")
        scored = []
        for q in qs:
            r = self._score(q, scan, scan_inf, emb, rad)
            if r is not None:
                scored.append((r[0], q["last_seen"], q, r))
        if not scored:                                 # places exist, but none shares a signal with the query
            return dict(_no_match("unknown"), uncompared=len(qs))
        uncompared = len(qs) - len(scored)
        scored.sort(key=lambda s: (s[0], s[1]), reverse=True)
        conf, _, best, (_, parts, evidence, drift) = scored[0]
        second = next((s for s in scored[1:] if best["name"] is None or s[2]["name"] != best["name"]), None)
        c2 = second[0] if second else 0.0
        if conf >= KNOWN_T:
            verdict = "known" if conf - c2 >= MARGIN else "ambiguous"
        elif conf < NEW_T:
            # "new" means every place was compared and none is close; a place nothing could be
            # compared with (an open-floor place vs an empty sweep and no look) may be this one
            verdict = "unknown" if uncompared else "new"
        else:
            verdict = "ambiguous"
        ranking = [{"place_id": s[2]["id"], "name": s[2]["name"], "confidence": round(s[0], 3),
                    "parts": dict(s[3][1])} for s in scored[:3]]
        if verdict == "unknown":
            # no place claimed, and ranking stays [] as for any "unknown": a consumer reads a place
            # missing from a ranking as scoring below it (the cockpit's combine_recognitions), which
            # an uncompared place did not — what WAS compared stays on record under `compared`
            return dict(_no_match("unknown"), uncompared=uncompared, compared=ranking)
        return {"verdict": verdict, "place_id": best["id"], "name": best["name"], "uncompared": uncompared,
                "confidence": round(conf, 3),
                "second": ({"place_id": second[2]["id"], "name": second[2]["name"],
                            "confidence": round(c2, 3), "parts": dict(second[3][1])}
                           if second else {"place_id": None, "name": None, "confidence": 0.0, "parts": None}),
                "parts": parts, "signals": _signals(parts), "evidence": round(evidence, 3),
                "yaw_drift_deg": None if drift is None else round(drift, 1), "ranking": ranking}

    def stats(self):
        with self._lock:
            return {"places": len(self._places), "named": sum(1 for q in self._places.values() if q["name"]),
                    "path": self.path, "loaded_from": self.loaded_from}
