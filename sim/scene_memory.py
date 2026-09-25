"""scene_memory — what the robot has seen, been told and run into, per world.

The owner asked for "memory" and "environmental / situational awareness".
This is the memory half (the awareness loop lives in sim/cockpit.py):

  observation  {t, pose {x, y, yaw}, kind, text, objects [...], world}
      kind: look  — a vision-model description of the eye frame
            find  — a find_object sighting (bearing + distance from the box geometry)
            scan  — a lidar summary worth keeping (an explicit scan, a new obstacle)
            guard — a guard fired (void latch, latched safe-stop, thermal, bus fault)
            said  — the robot reacted with a chord on its own
            user  — the operator said so ("remember that the charger is by the door")
  object       {name, x, y, confidence, source, t, n, size_m?} — the map-frame
      position of a named thing. Sightings of the same name within MERGE_M
      (0.25 m) are ONE object (confidence-weighted mean, so a later sighting
      moves it — unless it is less than half as confident, which only counts
      it); farther apart they are two (two boxes), at most MAX_PER_NAME each.
      where_is() answers with the newest one good enough to walk to (a newer
      vague one rides along as also_seen). Positions beyond WORLD_MAX_M (the
      floor plane) are refused.
  stale        an object older than STALE_S (10 min) or seen before the last
      reset / world load (new_epoch: the world respawns its bodies) may have
      moved: where_is / summary / the map say so, and go_back_to will not walk
      to one from before the reset. Places pinned at the robot's pose ('X is
      here', the spawn 'start') never go stale.

Positions are only as good as their source, and every object says which:
  find  0.4-1.0  the vision model's BOX projected onto the floor through the
                 eye's measured pose (cockpit_brains.pixel_to_floor), pushed
                 half the box's floor width beyond its near edge to estimate
                 the centre. The vision bench measured the near edge within
                 0.03 m (median) in CLEAN MuJoCo renders out to 1.4 m;
                 unmeasured on the real camera. A sighting farther than
                 cockpit_brains.FIND_TRUSTED_M (2 m) is kept at <= 0.3.
  user  0.9      "the charger is here" pins the robot's own pose; "... at
                 (1.0, 0.2)" pins those coordinates. Exact in the sim.
  look  0.2      a description that names an object with a direction and a
                 distance ("a ball on the left, about 0.5 m away"). Small VLMs
                 guess distances badly (0.30 m median error measured when they
                 type numbers), so these are hints for the map, never a goal:
                 go_back_to refuses anything below GO_MIN_CONF.

Pure Python, no MuJoCo, thread-safe (one RLock: the sim thread records
guards, the event loop records looks and finds, the bridge thread records
bus faults). Persistence is opt-in: SceneMemory(directory=...) keeps
<directory>/<world>.json (atomic replace, written off-thread, debounced);
directory=None keeps everything in RAM (the tests and headless sims).
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import threading
import time

VERSION = 1
KINDS = ("look", "find", "scan", "guard", "said", "user")
CAP = 500                  # observations kept per world
MERGE_M = 0.25             # same name within this = the same object
MAX_PER_NAME = 4           # instances of one name (two boxes, a ball that moved)
MAX_OBJECTS = 120
GO_MIN_CONF = 0.4          # go_back_to refuses a position vaguer than this (= FIND_MIN_CONF)
WEIGHT_CAP = 3.0           # a merged object's accumulated weight: new sightings still move it
VAGUE_RATIO = 0.5          # a sighting below this x the stored confidence only counts (no move, no relabel)
SAVE_DEBOUNCE_S = 0.5
RECENCY_S = 600.0          # recall's recency scale (10 min)
STALE_S = 600.0            # the map fades an object older than this
WORLD_MAX_M = 6.0          # |x|, |y| of any remembered position: the floor plane is 12 x 12 m
TEXT_MAX = 300
SPAWN_NAME = "start"       # new_epoch(spawn=...) pins this at the spawn pose ('go back to the start')
# how much an observation is worth keeping when the cap is reached (lowest, oldest go first)
VALUE = {"said": 0, "scan": 1, "look": 1, "guard": 3, "find": 3, "user": 5}
DEFAULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "memory")

_ARTICLES = {"the", "a", "an", "that", "this", "my", "your", "our", "some", "any"}
_STOP = _ARTICLES | {"is", "are", "was", "were", "be", "to", "of", "in", "on", "at", "by", "it",
                     "and", "or", "what", "where", "did", "you", "do", "see", "saw", "about",
                     "remember", "know", "please", "there", "here", "me", "for", "with", "now"}
_GENERIC = {"", "everything", "anything", "all", "stuff", "things", "around", "world", "situation"}
_WIPE = {"all", "everything", "it all", "all of it", "everything you know"}   # forget(): the ONLY wipe words
# words that never name an object: 'forget that' / 'forget those' / 'forget my stuff' name nothing
_PRONOUNS = {"it", "them", "those", "these", "they", "one", "ones", "thing", "things", "stuff", "him", "her"}
SYNONYMS = {"sphere": "ball", "orb": "ball", "cube": "box", "block": "box", "crate": "box",
            "carton": "box", "step": "stairs", "steps": "stairs", "staircase": "stairs",
            "stair": "stairs", "rocks": "rubble", "stones": "rubble", "pebbles": "rubble",
            "incline": "ramp", "slope": "ramp", "barrier": "wall"}
_KEEP_S = {"stairs", "glass", "grass", "boss", "class", "rubble", "canvas", "gas", "bus"}
# the nouns a look description may place on the map (world_builder.KINDS + household things)
LOOK_NOUNS = {"ball", "box", "wall", "ramp", "stairs", "rubble", "door", "charger", "chair",
              "table", "cup", "bottle", "bowl", "toy", "shoe", "cable", "basket"}
# (not "robot": the eye's descriptions say "in front of the robot" about itself)


# ------------------------------------------------------------------ names
def _singular(w):
    if w in _KEEP_S or len(w) <= 3:
        return w
    if w.endswith("ies") and len(w) > 4:
        return w[:-3] + "y"
    if w.endswith("es") and re.search(r"(x|s|sh|ch|z)es$", w):
        return w[:-2]
    if w.endswith("s") and not w.endswith("ss"):
        return w[:-1]
    return w


def norm_name(name):
    """'The Red Balls' -> 'red ball'; synonyms folded ('cube' -> 'box'). '' for nothing."""
    words = [w for w in re.findall(r"[a-z]+", str(name or "").lower()) if w not in _ARTICLES]
    words = [SYNONYMS.get(w, w) for w in words]
    words = [SYNONYMS.get(_singular(w), _singular(w)) for w in words]
    return " ".join(words[-3:])


def _head(name):
    parts = str(name).split()
    return parts[-1] if parts else ""


def same_thing(a, b):
    """Does name a refer to the same kind of thing as name b? Head nouns must
    match ('ball' ~ 'red ball'); two different adjectives do not ('red ball'
    vs 'blue ball')."""
    a, b = norm_name(a), norm_name(b)
    if not a or not b:
        return False
    if a == b:
        return True
    if _head(a) != _head(b):
        return False
    wa, wb = set(a.split()), set(b.split())
    return wa <= wb or wb <= wa


def forget_scope(name):
    """What a forget(name) means: ('all', None) for None or an explicit wipe
    word ('all', 'everything'); ('one', normalised name) for a real noun;
    ('unclear', None) for a pronoun / determiner / generic word ('that',
    'the', 'those', 'my stuff', '') — which must NEVER wipe the memory
    (review 2026-09-24: 'forget that' erased the operator's pins)."""
    if name is None:
        return "all", None
    raw = " ".join(re.findall(r"[a-z]+", str(name).lower()))
    if raw in _WIPE:
        return "all", None
    words = raw.split()
    if not words or all(w in _ARTICLES or w in _PRONOUNS or w in _GENERIC for w in words):
        return "unclear", None
    key = norm_name(raw)
    if not key or key in _GENERIC:
        return "unclear", None
    return "one", key


def _words(text):
    return {SYNONYMS.get(_singular(w), _singular(w)) for w in re.findall(r"[a-z]+", str(text).lower())} - _STOP


def fmt_age(s):
    s = max(0.0, float(s))
    if s < 10:
        return "just now"
    if s < 90:
        return f"{s:.0f} s ago"
    if s < 5400:
        return f"{s / 60:.0f} min ago"
    if s < 172800:
        return f"{s / 3600:.0f} h ago"
    return f"{s / 86400:.0f} d ago"


def direction_words(pose, x, y):
    """(distance m, 'ahead-left' ...) of map point (x, y) from the robot pose."""
    dx, dy = float(x) - float(pose["x"]), float(y) - float(pose["y"])
    d = math.hypot(dx, dy)
    rel = math.degrees(math.atan2(dy, dx)) - float(pose.get("yaw_deg", pose.get("yaw", 0.0)) or 0.0)
    rel = (rel + 180.0) % 360.0 - 180.0          # + = left of the nose
    names = ["ahead", "ahead-left", "left", "behind-left", "behind", "behind-right", "right", "ahead-right"]
    return d, names[int(((rel + 22.5) % 360.0) // 45.0)]


def _num(v):
    if isinstance(v, bool) or v is None:
        return None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _pose(p):
    """Any pose dict ({x, y, yaw_deg} or {x, y, yaw}) -> {x, y, yaw} rounded, or None."""
    if not isinstance(p, dict):
        return None
    x, y = _num(p.get("x")), _num(p.get("y"))
    yaw = _num(p.get("yaw", p.get("yaw_deg")))
    if x is None or y is None:
        return None
    return {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw or 0.0, 1)}


# ------------------------------------------------------ look descriptions
_DIST_RE = re.compile(r"(\d+(?:\.\d+)?)\s*(cm|centimet(?:er|re)s?|m\b|met(?:er|re)s?)")
_NEG_RE = re.compile(r"\b(no|not|without|nothing|none)\b")


def objects_from_description(text, pose, confidence=0.2):
    """A look description -> [{name, x, y, confidence, source 'look'}] for the
    objects it names WITH a direction and a distance in the same sentence.
    Left = -30 deg, right = +30 deg ('slightly' halves it), ahead = 0 (image
    convention, negative = LEFT); a number with a unit is the distance, else
    near/close = 0.5 m, far = 1.2 m; no distance word -> not placed. Negated
    mentions ('no ball') are skipped. Deliberately low confidence."""
    p = _pose(pose)
    if p is None or not text:
        return []
    out, seen = [], set()
    for sent in re.split(r"(?<!\d)\.(?!\d)|[;!?\n]+", str(text).lower()):     # not inside '0.5'
        if not sent:
            continue
        for m in re.finditer(r"[a-z]+", sent):
            noun = SYNONYMS.get(_singular(m.group(0)), _singular(m.group(0)))
            if noun not in LOOK_NOUNS or noun in seen:
                continue
            before = sent[max(0, m.start() - 24):m.start()]
            if _NEG_RE.search(before):
                continue
            if re.search(r"\bleft\b", sent) and not re.search(r"\bright\b", sent):
                bearing = -30.0
            elif re.search(r"\bright\b", sent) and not re.search(r"\bleft\b", sent):
                bearing = 30.0
            elif re.search(r"\b(ahead|in front|centre|center|straight|middle)\b", sent):
                bearing = 0.0
            else:
                continue
            if re.search(r"\bslightly\b", sent):
                bearing /= 2.0
            dm = _DIST_RE.search(sent)
            if dm:
                dist = float(dm.group(1)) * (0.01 if dm.group(2).startswith("c") else 1.0)
            elif re.search(r"\b(near|close|nearby|right in front)\b", sent):
                dist = 0.5
            elif re.search(r"\b(far|distant|background|distance)\b", sent):
                dist = 1.2
            else:
                continue
            if not 0.05 <= dist <= 4.0:
                continue
            th = math.radians(p["yaw"] - bearing)
            out.append({"name": noun, "x": round(p["x"] + dist * math.cos(th), 3),
                        "y": round(p["y"] + dist * math.sin(th), 3),
                        "confidence": float(confidence), "source": "look"})
            seen.add(noun)
    return out


_HERE_RE = re.compile(r"^(?:that\s+)?(?:the\s+|a\s+|an\s+|my\s+)?([a-z][a-z ]{0,40}?)\s+(?:is|are|sits?|lives?)\s+"
                      r"(?:right\s+)?(?:here|where\s+i\s+am|where\s+you\s+are|under\s+(?:me|you))\b")
_AT_RE = re.compile(r"^(?:that\s+)?(?:the\s+|a\s+|an\s+|my\s+)?([a-z][a-z ]{0,40}?)\s+(?:is|are)\s+at\s*\(?\s*"
                    r"(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)\s*\)?")
# 'this spot is home' / 'here as the dock' / 'this is the kitchen' / 'call this spot home' /
# 'mark here as dock': the robot's own pose, like 'X is here' (a name of at most 3 words)
_NAME3 = r"(?:the\s+|my\s+|a\s+|an\s+|our\s+)?([a-z]+(?:\s+[a-z]+){0,2})\s*$"
_SPOT_RE = re.compile(r"^(?:this\s+(?:spot|place|point)|here|this)\s+(?:is|as)\s+" + _NAME3)
_CALL_RE = re.compile(r"^(?:call|mark|name|label)\s+(?:this|here)(?:\s+(?:spot|place|point))?\s+(?:as\s+)?" + _NAME3)


def parse_note(note, pose):
    """A user note -> the object it pins, or None: 'the charger is here' /
    'this spot is home' / 'call this spot home' / 'mark here as the dock'
    (the robot's pose) | 'the charger is at (1.0, 0.2)' (map meters)."""
    t = str(note or "").strip().lower().rstrip(".!")
    m = _AT_RE.search(t)
    if m:
        name = norm_name(m.group(1))
        if name:
            return {"name": name, "x": float(m.group(2)), "y": float(m.group(3)),
                    "confidence": 0.9, "source": "user"}
    m = _HERE_RE.search(t) or _SPOT_RE.search(t) or _CALL_RE.search(t)
    p = _pose(pose)
    if m and p is not None:
        name = norm_name(m.group(1))
        if forget_scope(name)[0] == "one":           # 'this is it' names nothing
            return {"name": name, "x": p["x"], "y": p["y"], "confidence": 0.9, "source": "user",
                    "anchor": "robot"}
    return None


def _recent(e):
    return (e["t"], e.get("u", 0))


def _remove_quietly(p):
    try:
        os.remove(p)
    except OSError:
        pass


# ------------------------------------------------------------------ memory
class SceneMemory:
    """Observations + the object table for ONE world at a time (see the
    module docstring). Every public method is thread-safe and never raises
    on bad input: a bad observation is refused with ValueError from
    remember() only."""

    def __init__(self, world="flat", directory=None, cap=CAP, clock=time.time, log=None):
        self._lock = threading.RLock()
        self.clock = clock
        self.cap = int(cap)
        self.world = str(world)
        self.directory = directory
        self.obs = []
        self._objects = []
        self._seq = 0
        self._u = 0                         # update order: breaks ties between equal timestamps
        self.session_t0 = clock()           # when the current epoch began (new_epoch moves it)
        self._epoch = 0                     # objects carry the epoch they were last placed in: an older
        #                                     one (or one loaded from a file) is 'earlier_session'
        self._log = log or (lambda m: None)
        self._dirty = False
        self._timer = None
        self._pending = {}                  # path -> payload being written off-thread (load reads it first)
        self.loaded_from = None
        if directory:
            self.load()

    # -------------------------------------------------------------- files
    @staticmethod
    def filename(world):
        """<world>.json; a name that had to be changed to be a filename ('a/b')
        or cut (> 80 chars) gets a short hash so two worlds never share a file."""
        raw = str(world)
        s = re.sub(r"[^A-Za-z0-9_.-]+", "_", raw).strip("._") or "world"
        if re.search(r"[^A-Za-z0-9_. -]", raw) or len(s) > 80:
            s = s[:70] + "-" + hashlib.sha1(raw.encode()).hexdigest()[:8]
        return s + ".json"

    @property
    def path(self):
        return os.path.join(self.directory, self.filename(self.world)) if self.directory else None

    def set_directory(self, directory):
        """Turn persistence on (a directory) or off (None); loads this world's file."""
        with self._lock:
            self.flush()
            self.directory = directory
        if directory:
            self.load()

    def _parse(self, d):
        """A saved payload -> (observations, objects), every entry normalised
        the way remember() would have stored it (review 2026-09-24: a
        hand-edited object without 't' broke the whole state feed). Raises
        on a payload that is not the right shape."""
        if not isinstance(d, dict):
            raise ValueError("not a JSON object")
        saved = _num(d.get("saved"))
        saved = self.clock() if saved is None else saved
        obs = []
        for o in d.get("observations") or []:
            if not isinstance(o, dict) or o.get("kind") not in KINDS:
                continue
            t = _num(o.get("t"))
            objs = o.get("objects") if isinstance(o.get("objects"), list) else []
            obs.append({"t": round(saved if t is None else t, 2), "kind": o["kind"],
                        "text": str(o.get("text") or "")[:TEXT_MAX], "pose": _pose(o.get("pose")),
                        "objects": [c for c in (self._clean_object(x, o["kind"], 0.2) for x in objs) if c],
                        "world": self.world})
        objects = []
        for o in d.get("objects") or []:
            c = self._clean_object(o, "unknown", 0.2)       # no confidence saved: vague, never a goal
            if c is None:
                continue
            t = _num(o.get("t"))
            t = saved if t is None else t
            ft = _num(o.get("first_t"))
            n = _num(o.get("n"))
            w = _num(o.get("w"))
            c.update(t=round(t, 2), first_t=round(t if ft is None else min(ft, t), 2), epoch=-1,
                     n=int(n) if n is not None and 1 <= n < 1e9 else 1,
                     w=round(min(max(w, 0.05), WEIGHT_CAP), 3) if w is not None else c["confidence"])
            objects.append(c)
        objects = objects[-MAX_OBJECTS:]
        for i, c in enumerate(objects, 1):                   # file order = update order
            c["seq"], c["u"] = i, i
        return obs[-self.cap:], objects

    def load(self):
        """Replace the RAM contents with <dir>/<world>.json (empty when absent).
        A file that does not load is moved aside (*.corrupt-<time>), never
        lost. The file is read OUTSIDE the lock (the event loop's queries
        never wait on the disk)."""
        with self._lock:
            p = self.path
            world = self.world
            self.obs, self._objects, self.loaded_from = [], [], None
            pend = self._pending.get(p) if p else None
        if not p:
            return 0
        try:
            if pend is not None:                             # a write still in flight wins over the file
                d = json.loads(json.dumps(pend))
            elif os.path.exists(p):
                with open(p) as f:
                    d = json.load(f)
            else:
                return 0
            obs, objects = self._parse(d)
        except Exception as e:                               # noqa: BLE001 — a bad file is never fatal
            bad = f"{p}.corrupt-{int(self.clock())}"
            try:
                os.replace(p, bad)
            except OSError:
                pass
            self._log(f"memory: {p} did not load ({type(e).__name__}: {e}); moved to {bad}")
            return 0
        with self._lock:
            if self.world != world:                          # switched again meanwhile: not ours
                return 0
            for o in objects:                                # anything recorded while the file was read
                o["seq"] = o["u"] = 0
            self.obs = (obs + self.obs)[-self.cap:]
            self._objects = (objects + self._objects)[-MAX_OBJECTS:]
            for i, o in enumerate(self._objects, 1):
                o["seq"] = o["u"] = i
            self._seq = self._u = len(self._objects)
            self.loaded_from = p
            return len(obs)

    def _payload(self):
        return {"version": VERSION, "world": self.world, "saved": round(self.clock(), 1),
                "objects": [dict(o) for o in self._objects],
                "observations": [dict(o, objects=[dict(x) for x in o["objects"]]) for o in self.obs]}

    def _write(self, p, payload):
        """Atomic write (tmp + replace). Never raises."""
        try:
            os.makedirs(os.path.dirname(p), exist_ok=True)
            tmp = f"{p}.tmp{threading.get_ident()}"
            with open(tmp, "w") as f:
                json.dump(payload, f, indent=1)
            os.replace(tmp, p)
        except OSError as e:
            self._log(f"memory: could not save {p}: {e}")
            return None
        finally:
            with self._lock:
                if self._pending.get(p) is payload:
                    del self._pending[p]
        return p

    def _write_async(self, p, payload):
        with self._lock:
            self._pending[p] = payload
        threading.Thread(target=self._write, args=(p, payload), daemon=True, name="memory-save").start()

    def save(self):
        """Write now (atomic: tmp + replace). No-op without a directory."""
        with self._lock:
            p = self.path
            if not p:
                self._dirty = False
                return None
            payload = self._payload()
            self._dirty = False
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

    def _changed(self):
        """Debounced, off-thread save (the sim thread records guards: no file I/O there)."""
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

    def set_world(self, world, carry=False):
        """Switch worlds. carry=True (an EDIT of the current world: same
        objects, new name) keeps the RAM contents under the new name; else the
        old world is saved and the new one loaded (empty without a file).
        The sim thread calls this: the old world's save runs on its own
        thread, and the lock is never held across the disk (review 2026-09-24)."""
        world = str(world)
        with self._lock:
            if world == self.world:
                return
            t, self._timer = self._timer, None
            old_p = self.path
            if t is not None:
                t.cancel()
            payload = self._payload() if (self._dirty and old_p and not carry) else None
            self._dirty = False
            self.world = world
            for o in self.obs:
                o["world"] = world
            if carry:
                self._changed()
        if payload is not None:
            self._write_async(old_p, payload)
        if carry:
            # the carried memory continues under the new name; a superseded hashed
            # snapshot of an edited world ('custom-3fa91c0e.json') would only go stale
            if old_p and re.search(r"-[0-9a-f]{8}\.json$", old_p) and os.path.exists(old_p):
                threading.Thread(target=_remove_quietly, args=(old_p,), daemon=True).start()
        else:
            self.load()

    def new_epoch(self, why=None, pose=None, spawn=None, now=None):
        """The world's bodies are back at their spawn (a reset, a world load or
        edit): every object seen before now is 'earlier_session' (stale on the
        map; go_back_to will not walk to it without a fresh sighting). why:
        an observation to record; spawn: a pose {x, y, yaw?} to pin as
        SPAWN_NAME ('go back to the start'). No file I/O (the sim thread)."""
        now = self.clock() if now is None else float(now)
        with self._lock:
            self.session_t0 = now
            self._epoch += 1
        objs = []
        sp = _pose(spawn)
        if sp is not None:
            objs.append({"name": SPAWN_NAME, "x": sp["x"], "y": sp["y"], "confidence": 0.9,
                         "source": "spawn", "anchor": "robot"})
        if why or objs:
            return self.remember({"kind": "scan", "text": why or "spawn", "pose": pose or spawn,
                                  "objects": objs, "t": now})
        return None

    # --------------------------------------------------------- recording
    def remember(self, obs):
        """Store one observation (a dict: kind, text, pose, objects, t?) and
        merge its objects into the object table. Returns the stored record."""
        if not isinstance(obs, dict):
            raise ValueError("an observation is a dict")
        kind = obs.get("kind")
        if kind not in KINDS:
            raise ValueError(f"kind must be one of {', '.join(KINDS)}, got {kind!r}")
        now = _num(obs.get("t"))
        now = self.clock() if now is None else now
        rec = {"t": round(now, 2), "kind": kind, "text": str(obs.get("text") or "")[:TEXT_MAX],
               "pose": _pose(obs.get("pose")), "objects": [], "world": self.world}
        for o in obs.get("objects") or []:
            oo = self._clean_object(o, kind)
            if oo is not None:
                rec["objects"].append(oo)
        with self._lock:
            self.obs.append(rec)
            for oo in rec["objects"]:
                self._merge(oo, now)
            self._prune()
            self._changed()
        return rec

    @staticmethod
    def _clean_object(o, kind, default_conf=0.5):
        """One object -> the stored shape, or None (no name, not a finite
        position, or off the floor: |x|, |y| > WORLD_MAX_M)."""
        if not isinstance(o, dict):
            return None
        name = norm_name(o.get("name"))
        x, y = _num(o.get("x")), _num(o.get("y"))
        if not name or x is None or y is None or abs(x) > WORLD_MAX_M or abs(y) > WORLD_MAX_M:
            return None
        c = _num(o.get("confidence"))
        c = default_conf if c is None else max(0.0, min(1.0, c))
        out = {"name": name, "x": round(x, 3), "y": round(y, 3), "confidence": round(c, 2),
               "source": str(o.get("source") or kind)}
        s = _num(o.get("size_m"))
        if s is not None and 0 < s < 3:
            out["size_m"] = round(s, 3)
        if o.get("anchor") == "robot":           # pinned at the robot's own pose ('X is here')
            out["anchor"] = "robot"
        return out

    def _merge(self, o, now):
        near, best = None, MERGE_M
        for e in self._objects:
            if same_thing(e["name"], o["name"]):
                d = math.hypot(e["x"] - o["x"], e["y"] - o["y"])
                if d <= best:
                    near, best = e, d
        if near is None:
            self._seq += 1
            self._u += 1
            e = dict(o, t=round(now, 2), first_t=round(now, 2), n=1, w=o["confidence"], seq=self._seq, u=self._u,
                     epoch=self._epoch)
            self._objects.append(e)
            same = sorted((x for x in self._objects if same_thing(x["name"], o["name"])), key=_recent)
            for old in same[:-MAX_PER_NAME]:
                self._objects.remove(old)
            if len(self._objects) > MAX_OBJECTS:
                self._objects.sort(key=_recent)
                del self._objects[:len(self._objects) - MAX_OBJECTS]
            return
        near["n"] = int(near.get("n", 1)) + 1
        c_old, c_new = float(near["confidence"]), float(o["confidence"])
        if c_new < VAGUE_RATIO * c_old:
            # a much vaguer sighting of a well-located thing (a look near a find): counted,
            # but it neither moves it, relabels it nor makes it look fresh (review 2026-09-24:
            # a 0.2 look used to drag a 0.95 find and leave it labelled 'look' at 0.95)
            return
        w_old, w_new = min(float(near.get("w", c_old)), WEIGHT_CAP), max(c_new, 0.05)
        near["x"] = round((near["x"] * w_old + o["x"] * w_new) / (w_old + w_new), 3)
        near["y"] = round((near["y"] * w_old + o["y"] * w_new) / (w_old + w_new), 3)
        near["w"] = round(min(w_old + w_new, WEIGHT_CAP), 3)
        near["confidence"] = round((c_old * w_old + c_new * w_new) / (w_old + w_new), 2)   # weighted, not max
        near["t"] = round(now, 2)
        near["epoch"] = self._epoch
        self._u += 1
        near["u"] = self._u
        if c_new >= c_old:                       # the label follows the better source
            near["source"] = o["source"]
        if len(o["name"]) > len(near["name"]):          # 'ball' then 'orange ball': keep the specific one
            near["name"] = o["name"]
        if "size_m" in o:
            near["size_m"] = o["size_m"]
        if o.get("anchor"):
            near["anchor"] = o["anchor"]
        elif o["source"] != "user":
            near.pop("anchor", None)             # seen by the eye since: it is a thing, not a spot

    def _prune(self):
        extra = len(self.obs) - self.cap
        if extra <= 0:
            return
        order = sorted(range(len(self.obs)),
                       key=lambda i: (VALUE.get(self.obs[i]["kind"], 1) + (1 if self.obs[i]["objects"] else 0),
                                      self.obs[i]["t"]))
        drop = set(order[:extra])
        self.obs = [o for i, o in enumerate(self.obs) if i not in drop]

    # ------------------------------------------------------------ queries
    def _public(self, e, now):
        out = {k: e[k] for k in ("name", "x", "y", "confidence", "source") if k in e}
        out["age_s"] = round(max(0.0, now - e["t"]), 1)
        out["seen"] = int(e.get("n", 1))
        out["earlier_session"] = bool(e.get("epoch", -1) < self._epoch)
        for k in ("size_m", "anchor"):
            if k in e:
                out[k] = e[k]
        # stale: it may have moved — older than STALE_S, or seen before the last reset /
        # world load (the world respawns its bodies). A PLACE pinned at the robot's pose
        # ('X is here', the spawn 'start') does not move, so it never goes stale.
        out["stale"] = bool((out["age_s"] > STALE_S or out["earlier_session"]) and e.get("anchor") != "robot")
        return out

    def objects(self, name=None, now=None):
        """Every remembered object (or those matching name), newest first."""
        now = self.clock() if now is None else now
        with self._lock:
            es = [e for e in self._objects if name is None or same_thing(e["name"], name)]
            es.sort(key=_recent, reverse=True)
            return [self._public(e, now) for e in es]

    def where_is(self, name, now=None):
        """The best sighting of `name` -> {name, x, y, age_s, confidence,
        source, seen, stale, earlier_session, ...} or None. Best = the newest
        one good enough to walk to (confidence >= GO_MIN_CONF, not stale),
        else the newest confident one, else the newest; a newer vaguer
        sighting rides along as also_seen (review 2026-09-24: a 0.2 look
        used to hide a 0.95 find)."""
        if not norm_name(name):
            return None
        es = self.objects(name, now)
        if not es:
            return None
        good = [e for e in es if e["confidence"] >= GO_MIN_CONF]
        best = next((e for e in good if not e["stale"]), None) or (good[0] if good else es[0])
        if best is not es[0]:
            n = es[0]
            best = dict(best, also_seen={k: n[k] for k in ("name", "x", "y", "confidence", "source",
                                                           "age_s", "stale")})
        return best

    def notes_about(self, name, k=3):
        """User notes / observations that mention name (for 'not remembered with a position')."""
        return [r["text"] for r in self.recall(name, k=k) if r["text"]]

    def recall(self, query, k=5, pose=None, now=None):
        """The k most relevant observations for query: word match (query words
        in the text or the object names, head nouns and synonyms folded) x3,
        recency (exp(-age / 10 min)), closeness to the robot (exp(-d / 1 m))
        x0.5, and +0.3 for the operator's own notes. A specific query that
        matches nothing returns [] — not the latest random thing."""
        now = self.clock() if now is None else now
        q = _words(query)
        generic = not q or q <= _GENERIC
        p = _pose(pose)
        scored = []
        with self._lock:
            for o in self.obs:
                if generic:
                    match = 0.0
                else:
                    have = _words(o["text"]) | {w for ob in o["objects"] for w in _words(ob["name"])}
                    match = len(q & have) / len(q)
                    if match == 0.0:
                        continue
                age = max(0.0, now - o["t"])
                s = 3.0 * match + math.exp(-age / RECENCY_S)
                if p is not None:
                    pts = [(ob["x"], ob["y"]) for ob in o["objects"]] or (
                        [(o["pose"]["x"], o["pose"]["y"])] if o.get("pose") else [])
                    if pts:
                        d = min(math.hypot(x - p["x"], y - p["y"]) for x, y in pts)
                        s += 0.5 * math.exp(-d / 1.0)
                if o["kind"] == "user":
                    s += 0.3
                scored.append((s, o))
        scored.sort(key=lambda so: (so[0], so[1]["t"]), reverse=True)
        return [dict(o, age_s=round(now - o["t"], 1), score=round(s, 3)) for s, o in scored[:max(1, int(k))]]

    def forget(self, name="all"):
        """Forget one name (its objects and every observation that mentions
        it), or everything — ONLY for None or an explicit 'all' / 'everything'
        (forget_scope). A pronoun or determiner ('that', 'the', 'those')
        forgets nothing and returns 0. A wipe first keeps the old file as
        <world>.json.bak. Returns how many records went."""
        scope, key = forget_scope(name)
        if scope == "unclear":
            return 0
        backup = None
        with self._lock:
            if scope == "all":
                n = len(self._objects) + len(self.obs)
                if n and self.path:
                    backup = (self.path + ".bak", self._payload())
                self._objects, self.obs = [], []
                self._changed()
            else:
                n0 = len(self._objects) + len(self.obs)
                self._objects = [e for e in self._objects if not same_thing(e["name"], key)]
                head = _head(key)
                keep = []
                for o in self.obs:
                    o["objects"] = [ob for ob in o["objects"] if not same_thing(ob["name"], key)]
                    if head and head in _words(o["text"]):
                        continue
                    keep.append(o)
                self.obs = keep
                n = n0 - len(self._objects) - len(self.obs)
                if n:
                    self._changed()
        if backup is not None:
            threading.Thread(target=self._write, args=backup, daemon=True, name="memory-bak").start()
        return n

    def last(self, n=20, kind=None):
        with self._lock:
            rs = [o for o in self.obs if kind is None or o["kind"] == kind]
            return [dict(o) for o in rs[-max(0, int(n)):]][::-1]

    def to_map(self, now=None):
        """The objects for the cockpit map: [{name, x, y, confidence, source,
        age_s, seen, stale, earlier_session, size_m?, anchor?}], newest
        first (stale: see _public)."""
        return self.objects(now=now)

    def summary(self, pose=None, now=None, max_objects=4, short=False):
        """A natural-language line: the nearest few remembered objects with
        direction, age and source (STALE when they may have moved), plus the
        latest operator note. short=True is the situation line's form
        ('ball 0.7 m ahead-left (1 min, find)': no map coordinates — where_is
        has them — and not the spawn 'start' pin)."""
        now = self.clock() if now is None else now
        objs = self.objects(now=now)
        if short:
            objs = [o for o in objs if o.get("source") != "spawn"]
        p = _pose(pose)
        with self._lock:
            notes = [o for o in self.obs if o["kind"] == "user" and not o["objects"]]
        if not objs and not notes:
            return "nothing remembered in this world yet"
        parts = []
        if p is not None:
            objs.sort(key=lambda o: math.hypot(o["x"] - p["x"], o["y"] - p["y"]))
        for o in objs[:max_objects]:
            where = "" if short else f" at ({o['x']:.2f}, {o['y']:.2f})"
            if p is not None:
                d, side = direction_words({"x": p["x"], "y": p["y"], "yaw_deg": p["yaw"]}, o["x"], o["y"])
                where = f" {d:.1f} m {side}{where}"
            flags = (", vague" if o["confidence"] < GO_MIN_CONF else "") + (
                (", stale" if short else ", STALE: may have moved") if o["stale"] else "")
            age = fmt_age(o["age_s"])
            parts.append(f"{o['name']}{where} ({age.replace(' ago', '').replace('just now', 'now') if short else 'seen ' + age}, "
                         f"{o['source']}{flags})")
        s = "remembered: " + "; ".join(parts) if parts else "no object positions remembered"
        if len(objs) > max_objects:
            s += f"; +{len(objs) - max_objects} more"
        if notes:
            n = 40 if short else 80
            s += f"; operator note: \"{notes[-1]['text'][:n]}\""
        return s

    def stats(self):
        with self._lock:
            return {"world": self.world, "observations": len(self.obs), "objects": len(self._objects),
                    "cap": self.cap, "path": self.path, "loaded_from": self.loaded_from}
