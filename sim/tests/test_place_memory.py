"""Place memory (sim/place_memory.py) — pure Python + numpy: synthetic
ray-cast rooms, a fake clock and a fake embedder; no MuJoCo, no network.

What is pinned: the scan signature's map part does not care which way the
robot faces (rotating in place leaves it unchanged) and its sorted-range
part does not care about odometry yaw drift at all; scan_similarity is 1
for the same sweep, ~1 with 1 cm noise, tiny for a room of another shape,
and scan_align recovers a yaw drift; recognize() says known / new /
ambiguous / unknown by the documented thresholds, and a description breaks
the tie two same-shaped rooms leave the lidar with; ONE sense alone (a scan,
a description, radio) never says "known" and the result says which senses it
compared; two EMPTY sweeps are not compared at all, so a never-seen
open-floor world is never "known" as another (review 2026-09-25) and "new"
needs every place compared; places.json round-trips (a corrupt one is moved
aside); diff_objects calls something missing only where the look could see
and only when its description does not name it, moved only when its old
spot was seen empty; confirm_diff keeps only what two looks agree on;
verdict_text stays <= 90 characters.

The thresholds are pinned on the measured place bench run (MEASURED_ROWS:
each read's first look from sim/out/place_bench.json, replayed through the
real recognize() with signatures and embeddings built to give exactly the
recorded scan similarity and cosine): every true match "known", the
never-seen rooms "new", the same-walls room at its enrolment never "known" —
and the run's own old map missing the six re-worded true matches.
enroll_samples keeps both views of a new place (recognize takes the best
sample per signal); add_samples is not a visit; checked_objects reads the
eye's per-name presence answers (a "no" for a name the same look's words
name is unobservable, never missing — the rule visit() stores by) and
confirm_diff needs two looks to agree."""
import json
import math
import os
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(ROOT, "sim"))

import place_memory as pm                                              # noqa: E402


class Clock:
    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


# ------------------------------------------------------- synthetic rooms
def rect(w, h, cx=0.0, cy=0.0):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def cast(ox, oy, th, segs, circles, rmax=pm.RANGE_MAX):
    """Map-frame rays from (ox, oy) at headings th -> ranges (inf = no return)."""
    d = np.stack([np.cos(th), np.sin(th)], 1)
    best = np.full(len(th), np.inf)
    for (px, py), (qx, qy) in segs:
        ex, ey = qx - px, qy - py
        den = d[:, 0] * ey - d[:, 1] * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((px - ox) * ey - (py - oy) * ex) / den
            u = ((px - ox) * d[:, 1] - (py - oy) * d[:, 0]) / den
        ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        best = np.where(ok & (t < best), t, best)
    for (cx, cy), r in circles:
        fx, fy = ox - cx, oy - cy
        b = d[:, 0] * fx + d[:, 1] * fy
        disc = b * b - (fx * fx + fy * fy - r * r)
        with np.errstate(invalid="ignore"):
            t = -b - np.sqrt(disc)
        ok = (disc >= 0) & (t > 0)
        best = np.where(ok & (t < best), t, best)
    return np.where(best <= rmax, best, np.inf)


def sweep(room, x=0.0, y=0.0, yaw=0.0, n=360, noise=0.0, seed=0):
    """What sim_lidar.scan returns: body-frame angles [-pi, pi) and ranges."""
    body = np.linspace(-np.pi, np.pi, n, endpoint=False)
    r = cast(x, y, body + yaw, *room)
    if noise:
        r = r + np.random.default_rng(seed).normal(0.0, noise, r.shape)
    return body, r


def sig(room, x=0.0, y=0.0, yaw=0.0, drift=0.0, **kw):
    """The signature the cockpit would compute; drift = odometry yaw error (rad)."""
    a, r = sweep(room, x, y, yaw, **kw)
    return pm.scan_signature(a, r, yaw + drift)


# the room preset (sim_lidar.ROOM, the crate as a disc) and rooms that are not it
ROOM = (rect(3.2, 2.6), [((1.05, 0.75), 0.09), ((-1.0, -0.78), 0.07), ((-1.05, 0.85), 0.15)])
ROOM_TWIN = (rect(3.2, 2.6), [((0.9, -0.8), 0.2), ((-0.6, 0.9), 0.25)])     # same walls, other furniture
BIG = (rect(5.0, 4.0), [((1.5, 1.2), 0.3)])
CORRIDOR = (rect(6.0, 1.2), [])
OPEN = ([], [])                                  # flat floor: nothing taller than the scan plane


def fake_embed(text, dim=64):
    """Bag of words hashed into dim buckets: shared words -> a higher cosine."""
    v = np.zeros(dim)
    for w in str(text).lower().split():
        v[sum(map(ord, w)) % dim] += 1.0
    return v


def onehot(k, dim=16):
    v = np.zeros(dim)
    v[k] = 1.0
    return v


# ------------------------------------------------------------ signatures
def test_signature_shape_range_and_empty_sweep():
    s = sig(ROOM)
    assert s.shape == (pm.SIG_LEN,) and pm.SIG_LEN == pm.MAP_BINS + pm.INV_BINS == 72
    assert np.all((s >= 0) & (s <= 1))
    assert pm.scan_info(s) == 1.0                # walls all round: every bin has a return
    e = sig(OPEN)
    assert np.allclose(e, 1.0) and pm.scan_info(e) == 0.0
    # garbage in the sweep is a missing return, never an error
    a, r = sweep(ROOM)
    r[:10] = np.nan
    r[10:20] = -1.0
    r[20:30] = 99.0
    assert pm.scan_signature(a, r, 0.0).shape == (pm.SIG_LEN,)
    with pytest.raises(ValueError):
        pm.scan_signature(a, r[:-1], 0.0)
    with pytest.raises(ValueError):
        pm.scan_similarity(s, s[:10])


def test_map_part_is_rotation_invariant_under_yaw():
    """Turning on the spot (the yaw is known) leaves the map part alone: the
    same world directions land in the same map-heading bins."""
    ref = sig(ROOM, 0.1, -0.2)
    for deg in (37, 90, -123, 180):              # whole degrees: the same world rays
        s = sig(ROOM, 0.1, -0.2, yaw=math.radians(deg))
        assert np.allclose(s[:pm.MAP_BINS], ref[:pm.MAP_BINS], atol=1e-9), deg
        assert np.allclose(s[pm.MAP_BINS:], ref[pm.MAP_BINS:], atol=1e-9), deg
    s = sig(ROOM, 0.1, -0.2, yaw=math.radians(37.4))    # rays between the old ones
    assert pm.scan_similarity(ref, s) > 0.99


def test_histogram_part_ignores_yaw_drift_and_map_part_rolls():
    """A wrong odometry yaw moves the map part round (by the drift, in bins)
    and never touches the sorted-range part; scan_align reads the drift back."""
    ref = sig(ROOM)
    for drift_deg in (30, -70, 170):
        s = sig(ROOM, drift=math.radians(drift_deg))
        assert np.array_equal(s[pm.MAP_BINS:], ref[pm.MAP_BINS:])
        k = drift_deg // 10
        assert np.allclose(s[:pm.MAP_BINS], np.roll(ref[:pm.MAP_BINS], k), atol=1e-9)
        sim, drift = pm.scan_align(ref, s)
        assert sim > 0.999 and drift == pytest.approx(drift_deg)
    # a drift between bins: still recognised, drift within one bin
    sim, drift = pm.scan_align(ref, sig(ROOM, drift=math.radians(95)))
    assert sim > 0.95 and abs(drift - 95) <= 10


def test_similarity_identical_noisy_moved_and_other_rooms():
    ref = sig(ROOM)
    assert pm.scan_similarity(ref, ref) == pytest.approx(1.0)
    assert pm.scan_similarity(ref, sig(ROOM, noise=0.01, seed=3)) > 0.99
    near = pm.scan_similarity(ref, sig(ROOM, 0.1, 0.0))
    far = pm.scan_similarity(ref, sig(ROOM, 0.5, 0.0))
    assert 1.0 > near > far                      # it falls off with distance in the same room
    for other in (BIG, CORRIDOR, OPEN):          # another shape
        assert pm.scan_similarity(ref, sig(other)) < 0.05
    a, b = sig(ROOM, 0.2, 0.1), sig(BIG, -0.3, 0.2)
    assert pm.scan_similarity(a, b) == pytest.approx(pm.scan_similarity(b, a))
    # the documented limit: same walls, other furniture -> the lidar alone says "alike"
    assert pm.scan_similarity(ref, sig(ROOM_TWIN)) > 0.75


# ------------------------------------------------------ desc and radio
def test_desc_similarity_is_cosine_and_embed_is_injected():
    a, b = np.array([1.0, 0.0, 0.0]), np.array([1.0, 1.0, 0.0])
    assert pm.desc_similarity(a, a) == pytest.approx(1.0)
    assert pm.desc_similarity(a, b) == pytest.approx(1 / math.sqrt(2))
    assert pm.desc_similarity(a, -a) == pytest.approx(-1.0)
    assert pm.desc_similarity(a, np.ones(4)) == 0.0           # another model's dimension
    assert pm.desc_score(pm.DESC_COS_FULL) == 1.0 and pm.desc_score(pm.DESC_COS_FLOOR) == 0.0
    e = pm.embed_description("a wooden table and a red chair", fake_embed)
    assert e.shape == (64,) and np.linalg.norm(e) == pytest.approx(1.0)
    assert pm.embed_description("", fake_embed) is None
    assert pm.embed_description("text", None) is None

    def dead(_):
        raise ConnectionError("no llama-swap")
    assert pm.embed_description("text", dead) is None          # a dead embedder = a missing signal


def test_radio_similarity_weighted_jaccard():
    home = {"AA:BB:CC:00:00:01": -40, "aa:bb:cc:00:00:02": -70}
    assert pm.radio_similarity(home, home) == pytest.approx(1.0)
    assert pm.radio_similarity(home, {"11:22:33:44:55:66": -40}) == 0.0
    # same APs, one weaker: min/max of the weights (-40 -> 6/7, -70 -> 3/7, -80 -> 2/7)
    other = {"aa:bb:cc:00:00:01": -40, "aa:bb:cc:00:00:02": -80}
    want = (6 / 7 + 2 / 7) / (6 / 7 + 3 / 7)
    assert pm.radio_similarity(home, other) == pytest.approx(want)
    assert pm.radio_similarity(home, None) is None and pm.radio_similarity({}, home) is None
    assert pm.clean_radio([("AA:00", -50), ("bad", "x"), ("far", 20)]) == {"aa:00": -50.0}


# --------------------------------------------------------- recognition
def test_recognize_unknown_and_first_new():
    mem = pm.PlaceMemory(clock=Clock())
    r = mem.recognize()
    assert r["verdict"] == "unknown" and r["confidence"] == 0.0 and r["place_id"] is None
    r = mem.recognize(scan_sig=sig(ROOM))
    assert r["verdict"] == "new" and r["place_id"] is None      # nothing remembered yet
    with pytest.raises(ValueError):
        mem.enroll()                                            # a place needs a signal


def test_recognize_known_new_ambiguous_on_scans():
    mem = pm.PlaceMemory(clock=Clock())
    room = mem.enroll(scan_sig=sig(ROOM), desc_emb=onehot(0), name="lab")
    corr = mem.enroll(scan_sig=sig(CORRIDOR), desc_emb=onehot(1), name="hall")
    r = mem.recognize(scan_sig=sig(ROOM, noise=0.01, seed=5), desc_emb=onehot(0))
    assert r["verdict"] == "known" and r["place_id"] == room and r["name"] == "lab"
    assert r["confidence"] >= pm.KNOWN_T and r["second"]["place_id"] == corr
    assert r["parts"]["scan"] > 0.99 and r["parts"]["desc"] == 1.0 and r["parts"]["radio"] is None
    assert r["signals"] == ["scan", "desc"] and r["evidence"] == pytest.approx(1.0)
    # the same sweep with no description: one sense, capped below "known"
    r = mem.recognize(scan_sig=sig(ROOM, noise=0.01, seed=5))
    assert r["verdict"] == "ambiguous" and r["place_id"] == room and r["signals"] == ["scan"]
    assert r["confidence"] == pytest.approx(0.5 + (r["parts"]["scan"] - 0.5) * pm.SINGLE_EVIDENCE, abs=1e-3)
    # a scan alone can still say "new": another shape stays below 0.5
    r = mem.recognize(scan_sig=sig(BIG))
    assert r["verdict"] == "new" and r["confidence"] < pm.NEW_T
    # between the thresholds: every value 0.035 off -> d = 0.035 -> exp(-0.49) = 0.61
    r = mem.recognize(scan_sig=np.clip(sig(ROOM) + 0.035, 0, 1))
    assert r["verdict"] == "ambiguous" and pm.NEW_T <= r["confidence"] < pm.KNOWN_T
    assert r["place_id"] == room


def test_recognize_two_equal_places_are_ambiguous_unless_same_name():
    mem = pm.PlaceMemory(clock=Clock())
    a = mem.enroll(scan_sig=sig(ROOM), desc_emb=onehot(0))
    b = mem.enroll(scan_sig=sig(ROOM), desc_emb=onehot(0))
    r = mem.recognize(scan_sig=sig(ROOM), desc_emb=onehot(0))
    assert r["verdict"] == "ambiguous" and {r["place_id"], r["second"]["place_id"]} == {a, b}
    mem.name_place(a, "kitchen")
    mem.name_place(b, "kitchen")                 # enrolled twice: still one kitchen
    r = mem.recognize(scan_sig=sig(ROOM), desc_emb=onehot(0))
    assert r["verdict"] == "known" and r["name"] == "kitchen"


def test_description_breaks_the_tie_the_lidar_cannot():
    """Two rooms with the same walls: the scan cannot tell them apart; the
    description (injected embeddings) does."""
    mem = pm.PlaceMemory(clock=Clock())
    s = sig(ROOM)
    a = mem.enroll(scan_sig=s, desc_emb=onehot(0), desc_text="a sofa and a lamp", name="den")
    b = mem.enroll(scan_sig=s, desc_emb=onehot(1), desc_text="a bed and a wardrobe", name="bedroom")
    assert mem.recognize(scan_sig=s)["verdict"] == "ambiguous"
    r = mem.recognize(scan_sig=s, desc_emb=onehot(1))
    assert r["verdict"] == "known" and r["place_id"] == b and r["second"]["place_id"] == a
    assert r["parts"] == {"scan": 1.0, "desc": 1.0, "radio": None}
    # the weights: the other place = (0.5 * 1 + 0.35 * 0) / 0.85
    assert r["second"]["confidence"] == pytest.approx(0.5 / 0.85, abs=1e-3)
    # a description alone is one sense: even a perfect match is capped below "known"
    r = mem.recognize(desc_emb=onehot(0))
    assert r["verdict"] == "ambiguous" and r["place_id"] == a and r["signals"] == ["desc"]
    assert r["confidence"] == pytest.approx(0.5 + 0.5 * pm.SINGLE_EVIDENCE)
    assert r["evidence"] == pytest.approx(pm.SINGLE_EVIDENCE) and r["parts"]["scan"] is None
    assert pm.verdict_text(r).endswith(" [look only]")


def test_a_scan_alone_never_says_known_even_in_a_twin_room():
    """Review 2026-09-25: the same walls with other furniture are ~0.84 alike
    to the lidar, and a scan used to count as full evidence, so with the look
    or the embedder down the twin room came back 'known' at 0.84 — silently.
    Now one sense is capped (SINGLE_EVIDENCE) and the answer says 'scan only'."""
    assert 0.5 + 0.5 * pm.SINGLE_EVIDENCE < pm.KNOWN_T          # no lone sense can reach "known"
    sofa = "a sofa and a lamp by the window"
    mem = pm.PlaceMemory(clock=Clock())
    den = mem.enroll(scan_sig=sig(ROOM), desc_emb=fake_embed(sofa), name="den")
    r = mem.recognize(scan_sig=sig(ROOM_TWIN))                  # the look failed: no description
    assert r["parts"]["scan"] > pm.KNOWN_T                      # the lidar alone would call it the den
    assert r["verdict"] == "ambiguous" and r["place_id"] == den and r["confidence"] < pm.KNOWN_T
    assert r["signals"] == ["scan"] and r["evidence"] == pytest.approx(pm.SINGLE_EVIDENCE)
    assert pm.verdict_text(r) == f"place: den? ({r['confidence']:.2f}) [scan only]"
    # the very same spot, scan only: still not "known" (at most 0.725)
    r = mem.recognize(scan_sig=sig(ROOM))
    assert r["verdict"] == "ambiguous" and r["confidence"] == pytest.approx(0.5 + 0.5 * pm.SINGLE_EVIDENCE)
    # with the description back, the den is known and nothing is flagged ...
    r = mem.recognize(scan_sig=sig(ROOM), desc_emb=fake_embed(sofa))
    assert r["verdict"] == "known" and r["signals"] == ["scan", "desc"]
    assert pm.verdict_text(r) == "place: den (1.00)"
    # ... and the twin described as another room is not the den
    r = mem.recognize(scan_sig=sig(ROOM_TWIN), desc_emb=fake_embed("a bed and a wardrobe"))
    assert r["verdict"] != "known" and r["signals"] == ["scan", "desc"]
    # nothing compared -> no marker
    assert pm.verdict_text(mem.recognize()) == "place: unknown (no signal)"
    assert pm.verdict_text(pm.PlaceMemory().recognize(scan_sig=sig(ROOM))) == "place: NEW (no places yet)"


def test_empty_sweep_or_radio_alone_never_says_known():
    mem = pm.PlaceMemory(clock=Clock())
    flat = mem.enroll(scan_sig=sig(OPEN), radio={"aa:01": -45})
    # every open floor looks like this: two empty sweeps are not compared at all
    r = mem.recognize(scan_sig=sig(OPEN))
    assert r["verdict"] == "unknown" and r["place_id"] is None and r["uncompared"] == 1
    assert pm.verdict_text(r) == "place: unknown (1 place not comparable)"
    r = mem.recognize(radio={"aa:01": -45})
    assert r["verdict"] == "ambiguous" and r["confidence"] == pytest.approx(0.65) and r["place_id"] == flat
    r = mem.recognize(radio={"ff:99": -45})       # another building's Wi-Fi
    assert r["verdict"] == "new" and r["confidence"] == pytest.approx(0.35) and r["uncompared"] == 0
    # the empty sweep beside the radio adds nothing: still the radio alone, capped
    r = mem.recognize(scan_sig=sig(OPEN), radio={"aa:01": -45})
    assert r["signals"] == ["radio"] and r["parts"]["scan"] is None and r["confidence"] == pytest.approx(0.65)
    # a query sharing no signal with any place: nothing to compare
    assert mem.recognize(desc_emb=onehot(2))["verdict"] == "unknown"
    # an empty sweep + a description: the sweep is no second sense against an empty sample,
    # so the SAME open floor described the same way is only "ambiguous" [look only] — a
    # lidar-empty world is never "known" by the robot alone; the owner names it
    mem2 = pm.PlaceMemory(clock=Clock())
    floor = mem2.enroll(scan_sig=sig(OPEN), desc_emb=onehot(5))
    r = mem2.recognize(scan_sig=sig(OPEN), desc_emb=onehot(5))
    assert r["verdict"] == "ambiguous" and r["place_id"] == floor and r["signals"] == ["desc"]
    assert r["parts"]["scan"] is None and r["yaw_drift_deg"] is None
    assert r["evidence"] == pytest.approx(pm.SINGLE_EVIDENCE) and r["confidence"] == pytest.approx(0.725)
    assert pm.verdict_text(r).endswith("[look only]")
    # ... while an empty sweep against a room WITH walls is still compared: evidence against
    room = mem2.enroll(scan_sig=sig(ROOM), desc_emb=onehot(7))
    r = mem2.recognize(scan_sig=sig(OPEN), desc_emb=onehot(7))
    ranked = {e["place_id"]: e for e in r["ranking"]}
    assert ranked[room]["parts"]["scan"] < 0.01 and ranked[room]["parts"]["desc"] == 1.0
    assert ranked[room]["confidence"] < pm.KNOWN_T and r["verdict"] != "known"


def _cos_pair(cos, dim=64, seed=0):
    """Two unit vectors exactly `cos` alike."""
    rng = np.random.default_rng(seed)
    a = rng.normal(size=dim)
    a /= np.linalg.norm(a)
    r = rng.normal(size=dim)
    r -= r.dot(a) * a
    r /= np.linalg.norm(r)
    return a, cos * a + math.sqrt(1.0 - cos * cos) * r


@pytest.mark.parametrize("cos", [0.60, 0.70, 0.77, 0.79, 0.85, 0.894, 0.99])
def test_a_never_seen_lidar_empty_world_is_never_known_as_another(cos):
    """Review 2026-09-25: flat, then stairs, then the rubble field — every open-floor world
    read 'known' as the first one stored (an empty sweep counted as a second sense: evidence
    0.9, and the bench's wrong-room look cosines 0.60-0.89 mapped to 'known' 0.73-0.94). Now
    the look alone decides, capped: never 'known', and 'new' needs every place compared."""
    flat_look, stairs_look = _cos_pair(cos)
    mem = pm.PlaceMemory(clock=Clock())
    flat = mem.enroll(scan_sig=sig(OPEN), desc_emb=flat_look, name="flat")
    r = mem.recognize(scan_sig=sig(OPEN), desc_emb=stairs_look)            # the stairs world
    assert r["verdict"] != "known" and r["place_id"] == flat, r
    assert r["signals"] == ["desc"] and r["confidence"] <= 0.5 + 0.5 * pm.SINGLE_EVIDENCE < pm.KNOWN_T
    assert pm.verdict_text(r).endswith("[look only]")
    # the first bench-measured wrong-room cosine (0.79) was 'known' 0.832 under the old rule
    if cos >= 0.77:
        assert r["verdict"] == "ambiguous"
    # a failed look in an open-floor world, a room with walls also stored: the room is
    # compared (and is not it), the open-floor place is not comparable -> "unknown", never
    # "new" (it may be the flat)
    mem.enroll(scan_sig=sig(ROOM), desc_emb=onehot(3), name="den")
    u = mem.recognize(scan_sig=sig(OPEN))
    assert u["verdict"] == "unknown" and u["uncompared"] == 1 and u["place_id"] is None, u
    assert u["confidence"] == 0.0 and u["ranking"] == []          # a ranking would read the flat as lower
    assert [e["name"] for e in u["compared"]] == ["den"] and u["compared"][0]["confidence"] < pm.NEW_T
    assert pm.verdict_text(u) == "place: unknown (1 place not comparable)"


# ------------------------------------------------------ records + files
def test_visit_bounds_samples_and_updates_the_snapshot():
    clk = Clock()
    mem = pm.PlaceMemory(clock=clk)
    pid = mem.enroll(scan_sig=sig(ROOM), desc_emb=onehot(0), desc_text="first",
                     objects=[{"name": "chair", "x": 1.0, "y": 0.0}, {"name": "ball", "x": -1.0, "y": 0.5}])
    for k in range(12):                          # 12 distinct spots in the room
        clk.t += 10
        mem.visit(pid, scan_sig=sig(ROOM, 0.05 * k, -0.04 * k), desc_emb=onehot(k % 16), desc_text=f"v{k}")
    q = mem.get(pid)
    assert len(q["scan_sigs"]) == pm.N_SAMPLES and len(q["desc_embs"]) == len(q["desc_texts"]) == pm.N_SAMPLES
    assert q["visits"] == 13 and q["last_seen"] == pytest.approx(1120.0)
    assert q["desc_texts"][-1] == "v11"
    # a near-duplicate replaces its twin instead of pushing an old sample out
    n0 = len(q["scan_sigs"])
    mem.visit(pid, scan_sig=q["scan_sigs"][-1])
    assert len(mem.get(pid)["scan_sigs"]) == n0
    assert mem.visit("place-000000") is None
    # the snapshot: what this look saw replaces what it could see; the rest stays
    mem.visit(pid, objects=[{"name": "box", "x": 0.8, "y": 0.1}], visible=lambda x, y: x > 0)
    names = sorted(o["name"] for o in mem.get(pid)["objects"])
    assert names == ["ball", "box"]              # chair: in view, not seen -> gone; ball: out of view -> kept


def test_persistence_round_trip_and_corrupt_file(tmp_path):
    clk = Clock()
    mem = pm.PlaceMemory(directory=str(tmp_path), clock=clk)
    pid = mem.enroll(scan_sig=sig(ROOM), desc_emb=fake_embed("a sofa by the window"),
                     desc_text="a sofa by the window", radio={"AA:01": -50},
                     objects=[{"name": "Sofa", "x": 1.2, "y": -0.4, "confidence": 0.9}])
    mem.name_place(pid, "  living room. ")
    mem.note(pid, "the charger is behind the sofa")
    mem.flush()
    path = tmp_path / "places.json"
    d = json.loads(path.read_text())
    assert d["version"] == pm.VERSION and d["sig_len"] == pm.SIG_LEN and len(d["places"]) == 1

    back = pm.PlaceMemory(directory=str(tmp_path), clock=clk)
    q = back.get(pid)
    assert q["name"] == "living room" and q["notes"][0]["text"].startswith("the charger")
    assert q["objects"] == [{"name": "sofa", "x": 1.2, "y": -0.4, "confidence": 0.9, "t": 1000.0}]
    assert q["radios"] == [{"aa:01": -50.0}]
    r = back.recognize(scan_sig=sig(ROOM), desc_emb=fake_embed("a sofa by the window"), radio={"aa:01": -50})
    assert r["verdict"] == "known" and r["place_id"] == pid and r["confidence"] > 0.99
    assert back.places()[0]["samples"] == {"scan": 1, "desc": 1, "radio": 1}

    path.write_text("{not json")
    broken = pm.PlaceMemory(directory=str(tmp_path), clock=clk)
    assert broken.places() == [] and not path.exists()
    assert any(f.startswith("places.json.corrupt-") for f in os.listdir(tmp_path))


def test_signature_layout_change_drops_old_scans(tmp_path):
    mem = pm.PlaceMemory(directory=str(tmp_path), clock=Clock())
    pid = mem.enroll(scan_sig=sig(ROOM), desc_emb=onehot(3))
    mem.flush()
    path = tmp_path / "places.json"
    d = json.loads(path.read_text())
    d["sig_version"] = pm.SIG_VERSION + 1        # a file from another signature layout
    path.write_text(json.dumps(d))
    q = pm.PlaceMemory(directory=str(tmp_path), clock=Clock()).get(pid)
    assert q["scan_sigs"] == [] and len(q["desc_embs"]) == 1


def test_name_place_find_and_forget(tmp_path):
    mem = pm.PlaceMemory(directory=str(tmp_path), clock=Clock())
    a = mem.enroll(scan_sig=sig(ROOM))
    b = mem.enroll(scan_sig=sig(BIG))
    c = mem.enroll(scan_sig=sig(CORRIDOR))
    assert mem.get(a)["name"] is None
    assert mem.name_place(a, "Master Bedroom") is True and mem.find("master bedroom") == [a]
    assert mem.name_place("place-ffffff", "x") is False
    with pytest.raises(ValueError):
        mem.name_place(a, "  ")
    assert mem.forget("that") == 0 and mem.forget(None) == 0     # nothing named: nothing forgotten
    assert mem.forget(b) == 1 and mem.get(b) is None
    assert mem.forget("master bedroom") == 1 and mem.get(a) is None
    mem.flush()
    assert mem.forget("all") == 1 and mem.places() == [] and mem.get(c) is None
    mem.flush()
    for _ in range(100):                         # the backup is written off-thread
        if (tmp_path / "places.json.bak").exists():
            break
        time.sleep(0.01)
    assert len(json.loads((tmp_path / "places.json.bak").read_text())["places"]) == 1
    assert json.loads((tmp_path / "places.json").read_text())["places"] == []


# ------------------------------------------------------- change detection
def _place(*objs):
    return {"objects": [{"name": n, "x": x, "y": y, "confidence": 0.9, "t": 0} for n, x, y in objs]}


def test_diff_added_same_and_synonyms():
    p = _place(("chair", 1.0, 0.0), ("box", -1.0, 0.5))
    d = pm.diff_objects(p, [{"name": "chair", "x": 1.1, "y": 0.1}, {"name": "cube", "x": -1.0, "y": 0.45},
                            {"name": "ball", "x": 0.3, "y": 0.3, "confidence": 0.8}])
    assert sorted(o["name"] for o in d["same"]) == ["box", "chair"]     # 'cube' is a box
    assert d["added"] == [{"name": "ball", "x": 0.3, "y": 0.3, "confidence": 0.8}]
    assert d["missing"] == d["moved"] == d["unseen"] == []


def test_diff_missing_only_where_the_look_could_see():
    p = _place(("chair", 1.0, 0.0), ("ball", -1.0, 0.5))
    d = pm.diff_objects(p, [])                               # default: everything visible
    assert sorted(o["name"] for o in d["missing"]) == ["ball", "chair"]
    d = pm.diff_objects(p, [], visible=lambda x, y: x > 0)   # the eye looked towards +x only
    assert [o["name"] for o in d["missing"]] == ["chair"]
    assert d["unseen"] == [{"name": "ball", "x": -1.0, "y": 0.5}]

    def broken(x, y):
        raise RuntimeError
    assert pm.diff_objects(p, [], visible=broken)["missing"] == []   # a broken predicate claims nothing


def test_diff_moved_needs_the_old_spot_seen():
    p = _place(("chair", 1.0, 0.0))
    d = pm.diff_objects(p, [{"name": "chair", "x": 1.0, "y": 0.6}])
    assert d["moved"] == [{"name": "chair", "from": [1.0, 0.0], "to": [1.0, 0.6], "m": 0.6}]
    assert d["added"] == d["missing"] == []
    # old spot out of view: it may be the same chair or a second one — no claim of a move
    d = pm.diff_objects(p, [{"name": "chair", "x": -1.0, "y": 0.6}], visible=lambda x, y: x < 0)
    assert d["moved"] == [] and d["unseen"] == [{"name": "chair", "x": 1.0, "y": 0.0}]
    assert d["added"][0]["maybe_moved_from"] == [1.0, 0.0]
    # within match_m it is the same chair, nudged
    assert pm.diff_objects(p, [{"name": "chair", "x": 1.2, "y": 0.0}])["same"][0]["m"] == pytest.approx(0.2)


def test_confirm_diff_keeps_what_both_looks_agree_on():
    p = _place(("chair", 1.0, 0.0), ("ball", -1.0, 0.5))
    first = pm.diff_objects(p, [{"name": "chair", "x": 1.0, "y": 0.0}, {"name": "box", "x": 0.5, "y": 0.5}])
    second = pm.diff_objects(p, [{"name": "chair", "x": 1.0, "y": 0.0}, {"name": "box", "x": 0.55, "y": 0.45},
                                 {"name": "ball", "x": -1.0, "y": 0.5}])
    c = pm.confirm_diff(first, second)
    assert [o["name"] for o in c["added"]] == ["box"]
    assert c["missing"] == []                    # the first look missed the ball; the second saw it
    assert [(o["name"], o["change"]) for o in c["pending"]] == [("ball", "missing")]
    # a move both looks saw (to nearly the same spot) is confirmed; one look's move is pending
    one = pm.diff_objects(p, [{"name": "chair", "x": 1.0, "y": 0.6}, {"name": "ball", "x": -1.0, "y": 0.5}])
    two = pm.diff_objects(p, [{"name": "chair", "x": 1.05, "y": 0.62}, {"name": "ball", "x": -1.0, "y": 0.5}])
    c = pm.confirm_diff(one, two)
    assert [(o["name"], o["to"]) for o in c["moved"]] == [("chair", [1.05, 0.62])] and c["pending"] == []
    c = pm.confirm_diff(one, pm.diff_objects(p, [{"name": "chair", "x": 1.0, "y": 0.0}]))
    assert c["moved"] == [] and sorted((o["name"], o["change"]) for o in c["pending"]) == [
        ("ball", "missing"), ("chair", "moved")]


def test_a_mention_without_a_distance_is_not_absence():
    """Review 2026-09-25: objects_from_description places a thing only when
    its sentence gives a direction AND a distance, so 'a ball on the right'
    placed nothing and the ball was declared missing — by two looks that
    both named it. The look's text now goes in as `mentioned`."""
    from scene_memory import objects_from_description
    pose = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    stored = objects_from_description("A small room with pale walls. A ball 0.8 m ahead.", pose)
    assert [(o["name"], o["x"], o["y"]) for o in stored] == [("ball", 0.8, 0.0)]
    p = {"objects": stored}
    again = "A small room with pale walls all around and two grey pillars. A ball on the right."
    cur = objects_from_description(again, pose)
    assert cur == []                                            # named, not placed
    assert [o["name"] for o in pm.diff_objects(p, cur)["missing"]] == ["ball"]   # without the text
    d = pm.diff_objects(p, cur, mentioned=again)
    assert d["missing"] == [] and d["unseen"] == [{"name": "ball", "x": 0.8, "y": 0.0, "mentioned": True}]
    # two such looks: nothing to declare, nothing pending
    d2 = pm.diff_objects(p, [], mentioned="The ball sits by the pillar.")
    assert pm.confirm_diff(d, d2) == {"added": [], "missing": [], "moved": [], "pending": []}
    # a negated mention or silence is absence (where the look could see) ...
    for text in ("A small room. There is no ball.", "A small room, not a ball in sight.",
                 "No red ball and no box.", "Pale walls.", ""):
        assert [o["name"] for o in pm.diff_objects(p, [], mentioned=text)["missing"]] == ["ball"], text
    # ... but a negation elsewhere in the sentence is not
    for text in ("No box, but a ball on the left.", "The chair is not far from the ball."):
        assert pm.diff_objects(p, [], mentioned=text)["missing"] == [], text
    # plurals / synonyms fold, an iterable of names works, and adjectives follow same_thing
    assert pm.diff_objects(_place(("box", 1.0, 0.0)), [], mentioned="two cubes on the left")["missing"] == []
    assert pm.diff_objects(_place(("red ball", 1.0, 0.0)), [], mentioned=["ball"])["missing"] == []
    assert pm.mentioned_names("No ball; the cubes and a Chair.") >= {"box", "chair"}
    assert "ball" not in pm.mentioned_names("No ball; the cubes and a Chair.")
    # a mention never hides what the look DID place: a move is still a move
    d = pm.diff_objects(p, [{"name": "ball", "x": 0.5, "y": 0.8}], mentioned="A ball 0.9 m to the left.")
    assert [o["name"] for o in d["moved"]] == ["ball"] and d["missing"] == []


def test_visit_keeps_what_the_look_names_but_could_not_place():
    """The same rule for the stored snapshot: visit(objects=..., desc_text=...)
    drops a visible stored object only when the text does not name it."""
    mem = pm.PlaceMemory(clock=Clock())
    pid = mem.enroll(scan_sig=sig(ROOM), objects=[{"name": "ball", "x": 0.8, "y": 0.0},
                                                  {"name": "chair", "x": 1.0, "y": 0.3}])
    mem.visit(pid, objects=[], visible=lambda x, y: True, desc_text="A ball on the right; no chair.")
    assert [o["name"] for o in mem.get(pid)["objects"]] == ["ball"]     # named: kept; the chair: gone
    # a placed ball replaces the stored one; the mention does not keep a duplicate
    mem.visit(pid, objects=[{"name": "ball", "x": 0.5, "y": -0.5}], desc_text="A ball near, on the left.")
    assert [(o["name"], o["x"], o["y"]) for o in mem.get(pid)["objects"]] == [("ball", 0.5, -0.5)]
    # no text: the old rule (what the look could see is replaced by what it found)
    mem.visit(pid, objects=[])
    assert mem.get(pid)["objects"] == []


# ----------------------------------------------------------------- text
def _rec(verdict, name="kitchen", conf=0.91, pid="place-abc123", second=None):
    return {"verdict": verdict, "place_id": pid, "name": name, "confidence": conf,
            "second": second or {"place_id": None, "name": None, "confidence": 0.0}}


def test_verdict_text_forms():
    assert pm.verdict_text(_rec("known")) == "place: kitchen (0.91)"
    assert pm.verdict_text(_rec("new", conf=0.41)) == "place: NEW (best kitchen 0.41)"
    assert pm.verdict_text(_rec("new", name=None, pid=None, conf=0.0)) == "place: NEW (no places yet)"
    assert pm.verdict_text(_rec("known", name=None)) == "place: place-abc123 (0.91)"
    sec = {"place_id": "place-def456", "name": "bedroom", "confidence": 0.58}
    assert pm.verdict_text(_rec("ambiguous", conf=0.62, second=sec)) == \
        "place: kitchen? (0.62, or bedroom 0.58)"
    assert pm.verdict_text({"verdict": "unknown"}) == "place: unknown (no signal)"
    d = {"added": [{"name": "box"}], "missing": [{"name": "ball"}], "moved": []}
    assert pm.verdict_text(_rec("known", conf=0.88), d) == \
        "place: kitchen (0.88) — new here: box; missing: ball"
    assert pm.verdict_text(_rec("new", conf=0.3), d) == "place: NEW (best kitchen 0.30)"   # no diff on new


def test_verdict_text_stays_short():
    many = {"added": [{"name": f"very long object name {k}"} for k in range(9)],
            "missing": [{"name": "an extraordinarily long missing thing"}] * 4,
            "moved": [{"name": "chair"}] * 3}
    long_name = "the master bedroom upstairs at grandmother's house"
    sec = {"place_id": "place-def456", "name": long_name, "confidence": 0.58}
    lone = dict(_rec("ambiguous", name=long_name, conf=0.7, second=sec), signals=["scan"])
    for rec in (_rec("known", name=long_name), _rec("new", name=long_name, conf=0.2),
                _rec("ambiguous", name=long_name, conf=0.7, second=sec), _rec("known"), lone,
                dict(_rec("known", name=long_name), parts={"scan": 0.9, "desc": None, "radio": None})):
        for d in (None, many, {"added": [{"name": "box"}]}):
            s = pm.verdict_text(rec, d)
            assert len(s) <= pm.VERDICT_MAX == 90, s
            assert s.startswith("place: ")
    assert "new here" in pm.verdict_text(_rec("known"), many)


def test_places_summary_and_verdict_from_recognize():
    clk = Clock()
    mem = pm.PlaceMemory(clock=clk)
    pid = mem.enroll(scan_sig=sig(ROOM), desc_text="a lamp", desc_emb=onehot(4),
                     objects=[{"name": "lamp", "x": 0.5, "y": 0.5}], name="study")
    clk.t += 30
    s = mem.places()
    assert s[0]["id"] == pid and s[0]["name"] == "study" and s[0]["age_s"] == 30.0
    assert s[0]["object_names"] == ["lamp"] and s[0]["last_description"] == "a lamp"
    r = mem.recognize(scan_sig=sig(ROOM), desc_emb=onehot(4))
    d = pm.diff_objects(mem.get(pid), [{"name": "lamp", "x": 0.5, "y": 0.5},
                                       {"name": "chair", "x": 1, "y": 0}])
    assert pm.verdict_text(r, d) == "place: study (1.00) — new here: chair"
    assert mem.stats()["places"] == 1 and mem.stats()["named"] == 1


# ------------------------------------------- thresholds: the measured bench run
# sim/out/place_bench.json, 2026-09-25 19:39 (3 trials; lfm2.5-vl looks, llama-swap's
# 'embedding'), each read's FIRST look copied verbatim: (trial, case, parts.scan,
# parts.desc — a desc SCORE under the run's old map 0.55 -> 0, 0.90 -> 1 — the recorded
# second.confidence, the runner-up's scan when it is known (a <-> b from the spawn 0.526,
# c vs a or b 0.0; None = unknown), two_looks (the first look was ambiguous: the recorded
# confidences are the mean of two looks, the parts the first look's), what the thresholds
# must answer). "twin" = b at its enrolment vs a: same walls, never seen -> never "known".
OLD_FLOOR, OLD_FULL = 0.55, 0.90
MEASURED_ROWS = [
    (1, "a same pose", 1.0, 0.697, 0.525, 0.526, False, "known"),
    (1, "b named as a", 1.0, 0.596, 0.629, 0.526, False, "known"),
    (1, "a named as b", 1.0, 1.0, 0.626, 0.526, False, "known"),
    (1, "a moved + turned", 0.786, 1.0, 0.521, None, False, "known"),
    (1, "b + chair", 0.867, 0.655, 0.436, None, False, "known"),
    (1, "c - ball", 1.0, 0.406, 0.405, 0.0, False, "known"),
    (2, "a same pose", 1.0, 0.569, 0.644, 0.526, False, "known"),
    (2, "b named as a", 1.0, 0.647, 0.614, 0.526, False, "known"),
    (2, "a named as b", 1.0, 0.917, 0.67, 0.526, False, "known"),
    (2, "a moved + turned", 0.785, 0.91, 0.543, None, False, "known"),
    (2, "b + chair", 0.867, 0.355, 0.424, None, True, "known"),
    (2, "c - ball", 1.0, 0.362, 0.293, 0.0, True, "known"),
    (3, "a same pose", 1.0, 0.384, 0.45, 0.526, True, "known"),
    (3, "b named as a", 1.0, 0.371, 0.474, 0.526, True, "known"),
    (3, "a named as b", 1.0, 0.723, 0.594, 0.526, False, "known"),
    (3, "a moved + turned", 0.786, 1.0, 0.51, None, False, "known"),
    (3, "b + chair", 0.867, 0.524, 0.42, None, True, "known"),
    (3, "c - ball", 1.0, 0.258, 0.262, 0.0, True, "known"),
    (1, "d never seen", 0.0, 0.696, 0.227, None, False, "new"),
    (2, "d never seen", 0.0, 0.705, 0.227, None, False, "new"),
    (3, "d never seen", 0.0, 0.636, 0.228, None, False, "new"),
    (1, "c at enrolment", 0.0, 0.303, 0.002, None, False, "new"),
    (2, "c at enrolment", 0.0, 0.271, 0.051, None, False, "new"),
    (3, "c at enrolment", 0.0, 0.138, 0.029, None, False, "new"),
    (1, "b at enrolment", 0.526, 0.273, 0.0, None, False, "twin"),
    (2, "b at enrolment", 0.526, 0.748, 0.0, None, False, "twin"),
    (3, "b at enrolment", 0.526, 0.794, 0.0, None, False, "twin"),
]


def _old_cos(desc_old):
    """The run's desc score back to its cosine (a score of 1.0 only says >= 0.90:
    replayed as 0.90, the worst case for a match)."""
    return OLD_FLOOR + (OLD_FULL - OLD_FLOOR) * desc_old


def _replay(scan, cos, runner=None):
    """A place whose stored samples give exactly (scan, cos) against one query, and
    optionally a differently named runner-up with its own (scan, cos) -> recognize()."""
    def offset(s):                       # a constant offset d gives exp(-(d / SCAN_D0)^2) at every shift
        return pm.SCAN_D0 * math.sqrt(-math.log(s)) if s > 0 else 0.5
    q_sig = np.full(pm.SIG_LEN, 0.4)
    q_emb = np.array([1.0, 0.0, 0.0, 0.0])
    mem = pm.PlaceMemory(clock=Clock())
    mem.enroll(scan_sig=q_sig + offset(scan), desc_emb=[cos, math.sqrt(1 - cos * cos), 0.0, 0.0],
               name="right")
    if runner is not None:
        s2, c2 = runner
        mem.enroll(scan_sig=q_sig + offset(s2), desc_emb=[c2, 0.0, math.sqrt(1 - c2 * c2), 0.0], name="other")
    return mem.recognize(scan_sig=q_sig, desc_emb=q_emb)


def _runner(row):
    """The runner-up's (scan, cos), backed out of its recorded confidence where its scan is
    known and the confidence is one look's (not a two-look mean); None otherwise."""
    _t, _case, _scan, _desc, second, ru_scan, two_looks, _want = row
    if ru_scan is None or two_looks:
        return None
    w = pm.W_SCAN + pm.W_DESC
    d = (w * second - pm.W_SCAN * ru_scan) / pm.W_DESC
    assert 0.0 < d < 1.0                  # not clipped: the cosine it gives back is exact
    return ru_scan, _old_cos(d)


def test_thresholds_pinned_on_the_measured_bench_rows():
    """The module docstring's THRESHOLDS, replayed through the real recognize(): every true
    match of the run is "known" (6 of them were not under the old map), the never-seen rooms
    stay "new", and the same-walls room b at its enrolment is never "known"."""
    assert (pm.DESC_COS_FLOOR, pm.DESC_COS_FULL, pm.KNOWN_T, pm.MARGIN, pm.NEW_T) == \
        (0.25, 0.90, 0.75, 0.08, 0.5)
    assert (pm.W_SCAN, pm.W_DESC, pm.SINGLE_EVIDENCE) == (0.5, 0.35, 0.45)
    got = {"known": [], "new": [], "twin": []}
    margins = []
    for row in MEASURED_ROWS:
        t, case, scan, desc, _second, _ru, _two, want = row
        runner = _runner(row) if want == "known" else None
        r = _replay(scan, _old_cos(desc), runner)
        assert r["signals"] == ["scan", "desc"] and r["evidence"] == pytest.approx(1.0)
        got[want].append(r["confidence"])
        if want == "known":
            assert r["verdict"] == "known" and r["name"] == "right", (t, case, r)
            if runner is not None:
                assert r["second"]["name"] == "other"
                margins.append(r["confidence"] - r["second"]["confidence"])
        elif want == "new":
            assert r["verdict"] == "new", (t, case, r)
        else:
            assert r["verdict"] == "ambiguous", (t, case, r)
    assert len(got["known"]) == 18 and len(got["new"]) == 6 and len(got["twin"]) == 3
    # the docstring's numbers
    assert min(got["known"]) == pytest.approx(0.779, abs=1e-3)          # trial 2 "b + chair"
    assert min(got["known"]) - pm.KNOWN_T >= 0.025
    assert pm.KNOWN_T - (0.5 + 0.5 * pm.SINGLE_EVIDENCE) == pytest.approx(0.025)
    assert max(got["new"]) == pytest.approx(0.346, abs=1e-3) and pm.NEW_T - max(got["new"]) > 0.15
    assert min(got["twin"]) == pytest.approx(0.560, abs=1e-3)
    assert max(got["twin"]) == pytest.approx(0.676, abs=1e-3)
    assert len(margins) == 8 and min(margins) == pytest.approx(0.225, abs=2e-3) and min(margins) > pm.MARGIN


def test_the_old_desc_map_misses_the_six_reworded_true_matches(monkeypatch):
    """Why the map moved: under the run's own map (0.55 -> 0.90) the same first looks give
    12 known + 6 below KNOWN_T — the run's 12 known / 6 ambiguous."""
    monkeypatch.setattr(pm, "DESC_COS_FLOOR", OLD_FLOOR)
    monkeypatch.setattr(pm, "DESC_COS_FULL", OLD_FULL)
    low = []
    for row in MEASURED_ROWS:
        t, case, scan, desc, *_rest, want = row
        if want == "known":
            r = _replay(scan, _old_cos(desc))
            if r["confidence"] < pm.KNOWN_T:
                low.append((t, case, r["confidence"]))
    assert sorted((t, c) for t, c, _ in low) == [(2, "b + chair"), (2, "c - ball"), (3, "a same pose"),
                                               (3, "b + chair"), (3, "b named as a"), (3, "c - ball")]
    assert min(c for *_, c in low) == pytest.approx(0.656, abs=1e-3)


def test_recognize_second_and_ranking_carry_their_parts():
    mem = pm.PlaceMemory(clock=Clock())
    a = mem.enroll(scan_sig=sig(ROOM), desc_emb=onehot(0), name="den")
    b = mem.enroll(scan_sig=sig(ROOM_TWIN), desc_emb=onehot(1), name="bedroom")
    r = mem.recognize(scan_sig=sig(ROOM), desc_emb=onehot(0))
    assert r["place_id"] == a and r["second"]["place_id"] == b
    assert r["second"]["parts"]["scan"] > 0.75 and r["second"]["parts"]["desc"] == 0.0
    assert r["second"]["parts"]["radio"] is None
    assert [e["parts"] for e in r["ranking"]] == [r["parts"], r["second"]["parts"]]
    assert mem.recognize()["second"]["parts"] is None                    # no signal: nothing compared


# ------------------------------------------------------- enrolment samples
def test_enroll_samples_stores_both_views_and_recognize_takes_the_best_sample():
    """The arrival look and the second look (after the 30 deg turn) enrolled together: a
    revisit that sees either view matches that view's description (the best sample per
    signal), where a place enrolled from the arrival alone reads the turned view as another
    description."""
    s0 = sig(ROOM)
    s30 = sig(ROOM, yaw=math.radians(30))            # a turn in place: the map-frame sweep is unchanged
    ahead, turned = onehot(0), onehot(1) * 0.2 + onehot(2)      # two views, two descriptions
    mem = pm.PlaceMemory(clock=Clock())
    first = {"sig": s0, "emb": ahead, "text": "a pillar ahead", "pose": {"x": 0, "y": 0, "yaw_deg": 0}}
    second = {"scan_sig": s30, "desc_emb": turned, "desc_text": "a chair on the left"}
    pid = mem.enroll_samples([first, None, {"text": "no signal"}, second],
                             objects=[{"name": "chair", "x": -1.05, "y": 0.85}], name="den")
    q = mem.get(pid)
    assert q["visits"] == 1 and q["name"] == "den" and [o["name"] for o in q["objects"]] == ["chair"]
    assert len(q["scan_sigs"]) == 1                  # the two sweeps are DUP_SIM alike: one sample
    assert len(q["desc_embs"]) == 2 and q["desc_texts"] == ["a pillar ahead", "a chair on the left"]
    for view in (ahead, turned):
        r = mem.recognize(scan_sig=s0, desc_emb=view)
        assert r["verdict"] == "known" and r["parts"]["desc"] == 1.0, r
    one = pm.PlaceMemory(clock=Clock())
    one.enroll(scan_sig=s0, desc_emb=ahead, name="den")
    assert one.recognize(scan_sig=s0, desc_emb=turned)["parts"]["desc"] < 1.0
    with pytest.raises(ValueError):
        mem.enroll_samples([None, {"text": "words only"}])


def test_add_samples_is_not_a_visit():
    clk = Clock()
    mem = pm.PlaceMemory(clock=clk)
    pid = mem.enroll(scan_sig=sig(ROOM), desc_emb=onehot(0), desc_text="ahead",
                     objects=[{"name": "ball", "x": 0.8, "y": 0.0}])
    clk.t += 5
    s = mem.add_samples(pid, [{"sig": sig(ROOM, 0.3, 0.1), "emb": onehot(3), "text": "turned"}])
    q = mem.get(pid)
    assert s["samples"] == {"scan": 2, "desc": 2, "radio": 0} and q["visits"] == 1
    assert q["last_seen"] == 1000.0 and [o["name"] for o in q["objects"]] == ["ball"]
    assert mem.add_samples("place-000000", [{"sig": sig(ROOM)}]) is None
    assert mem.add_samples(pid, [None, {"text": "nothing"}])["samples"]["desc"] == 2


# ------------------------------------------------ change check by the eye
def test_checked_objects_from_the_eyes_presence_answers():
    p = _place(("ball", 0.8, 0.0), ("chair", 1.0, 0.4), ("box", -1.0, 0.5), ("lamp", 0.5, -0.5))
    seen = {"Balls": True,                       # boxed now
            "chair": False,                      # asked, not there
            "box": None,                         # asked, no usable answer
            "red cube": True,                    # a box by its synonym, an adjective: the stored box
            "cup": True,                         # boxed, nothing of the kind stored
            "table": False}                      # asked, not there, never stored: nothing to say
    d = pm.checked_objects(p, seen)
    assert d["present"] == [{"name": "ball", "x": 0.8, "y": 0.0}, {"name": "box", "x": -1.0, "y": 0.5}]
    assert d["missing"] == [{"name": "chair", "x": 1.0, "y": 0.4}]
    assert d["unobservable"] == [{"name": "lamp", "x": 0.5, "y": -0.5, "why": "not asked"}]
    assert d["added"] == [{"name": "cup"}]
    # False only where the look could see the stored spot; a broken predicate claims nothing
    d = pm.checked_objects(p, {"chair": False, "ball": False}, visible=lambda x, y: y < 0.2)
    assert d["missing"] == [{"name": "ball", "x": 0.8, "y": 0.0}]
    assert {"name": "chair", "x": 1.0, "y": 0.4, "why": "out of view"} in d["unobservable"]

    def broken(x, y):
        raise RuntimeError
    assert pm.checked_objects(p, {"chair": False}, visible=broken)["missing"] == []
    # None / no key / garbage never counts as absence
    for s in ({"chair": None}, {}, None, {"chair": "no"}, {"chair": {"why": "the eye did not answer"}}):
        d = pm.checked_objects(p, s)
        assert d["missing"] == [] and d["added"] == [], s
        assert {"name": "chair", "x": 1.0, "y": 0.4, "why": "not asked" if not s or "chair" not in s
                else "no answer"} in d["unobservable"], s
    # two keys for one name: a seen beats a no, a no beats no answer
    assert pm.checked_objects(p, {"chair": False, "chairs": True})["present"][0]["name"] == "chair"
    assert pm.checked_objects(p, {"chairs": None, "chair": False})["missing"][0]["name"] == "chair"


def test_checked_objects_takes_the_cockpits_eye_answers_as_they_are():
    """The cockpit's look['eye'][name]: {seen, asked, x, y, confidence, bearing_deg, distance_m, why}."""
    p = _place(("ball", 0.75, 0.2))
    eye = {"ball": {"seen": False, "asked": True, "model": "lfm2.5-vl"},
           "box": {"seen": True, "asked": True, "x": 0.65, "y": 0.15, "confidence": 0.9, "bearing_deg": -13.0,
                   "distance_m": 0.67},
           "chair": {"seen": True, "asked": True, "x": None, "y": None, "confidence": 0.7,
                     "why": "boxed, not placed (beyond 2 m)"},
           "ramp": {"seen": None, "why": "out of view"}}
    d = pm.checked_objects(p, eye)
    assert d["missing"] == [{"name": "ball", "x": 0.75, "y": 0.2}]
    assert d["added"] == [{"name": "box", "x": 0.65, "y": 0.15, "confidence": 0.9},
                          {"name": "chair", "confidence": 0.7}]            # boxed, not placed: no position
    assert d["unobservable"] == [] and d["present"] == []
    assert pm.checked_objects({"objects": []}, {})["added"] == []


def test_the_eye_saying_no_to_what_the_description_names_is_never_missing():
    """Review 2026-09-25: the eye's box question said 'no ball' while the same look's words
    said 'a ball straight ahead': declared missing, kept by visit() (its desc_text names it),
    and declared missing again the next visit. A no against the look's own words is a conflict,
    not absence: unobservable, so two such visits declare nothing, and the memory agrees."""
    clk = Clock()
    mem = pm.PlaceMemory(clock=clk)
    pid = mem.enroll(scan_sig=sig(ROOM), desc_emb=onehot(0), desc_text="A ball straight ahead, 0.8 m away.",
                     objects=[{"name": "ball", "x": 0.8, "y": 0.0}])
    words = "A small room with pale walls. A ball straight ahead, 0.8 m away."
    for visit in range(2):
        checks = []
        for _look in range(2):                                   # the arrival + the second look
            d = pm.checked_objects(mem.get(pid), {"ball": {"seen": False, "asked": True}}, mentioned=words)
            assert d["missing"] == [] and d["unobservable"] == [
                {"name": "ball", "x": 0.8, "y": 0.0, "why": "the description names it"}], (visit, d)
            checks.append(d)
        c = pm.confirm_diff(*checks)
        assert c["missing"] == [] and c["pending"] == [], (visit, c)
        clk.t += 60
        mem.visit(pid, scan_sig=sig(ROOM), desc_emb=onehot(0), desc_text=words)
        assert [o["name"] for o in mem.get(pid)["objects"]] == ["ball"]          # the memory agrees
    # the same answer with mentioned as names, and without the words: missing as before
    assert pm.checked_objects(mem.get(pid), {"ball": False}, mentioned={"ball"})["missing"] == []
    assert pm.checked_objects(mem.get(pid), {"ball": False}, mentioned=["red balls"])["missing"] == []
    assert [o["name"] for o in pm.checked_objects(mem.get(pid), {"ball": False})["missing"]] == ["ball"]
    # a negated mention is no mention: 'no ball here' and the eye's no agree -> missing
    d = pm.checked_objects(mem.get(pid), {"ball": False}, mentioned="Pale walls, no ball on the floor.")
    assert [o["name"] for o in d["missing"]] == ["ball"]
    # words naming something else change nothing; out of view still wins over the words
    assert pm.checked_objects(mem.get(pid), {"ball": False}, mentioned="a box on the left")["missing"]
    d = pm.checked_objects(mem.get(pid), {"ball": False}, visible=lambda x, y: False, mentioned=words)
    assert d["unobservable"][0]["why"] == "out of view"
    # the eye's yes is never overruled by the words
    assert pm.checked_objects(mem.get(pid), {"ball": True}, mentioned="")["present"][0]["name"] == "ball"


def test_confirm_diff_on_two_presence_checks():
    """Two looks must agree: a thing boxed by both is added, a thing both said is gone is
    missing; what one look alone reports is pending. An unplaced 'added' pairs by name."""
    p = _place(("ball", 0.75, 0.2), ("lamp", -1.0, 0.0))
    one = pm.checked_objects(p, {"ball": False, "box": {"seen": True, "x": 0.65, "y": 0.15}, "lamp": False})
    two = pm.checked_objects(p, {"ball": False, "box": {"seen": True}, "lamp": True})
    c = pm.confirm_diff(one, two)
    assert [o["name"] for o in c["added"]] == ["box"] and [o["name"] for o in c["missing"]] == ["ball"]
    assert [(o["name"], o["change"]) for o in c["pending"]] == [("lamp", "missing")]
    assert c["moved"] == []
    # one look only (the second saw nothing to say): nothing is declared
    c = pm.confirm_diff(one, pm.checked_objects(p, {}))
    assert c["added"] == c["missing"] == [] and len(c["pending"]) == 3
    # positions on both sides still have to agree
    far = pm.checked_objects(p, {"box": {"seen": True, "x": -0.9, "y": -0.6}})
    assert pm.confirm_diff(one, far)["added"] == []
