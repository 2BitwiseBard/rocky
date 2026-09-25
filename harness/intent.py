#!/usr/bin/env python3
"""intent — talk to Pebble in plain text, no LLM required (session 8d).

The deterministic layer of the VISION_PLAN brain stack: a regex/keyword
intent parser that maps operator text onto the SAME tools the MCP
server and local_brain expose (say / gesture / goto / stop / scan_summary
/ status / look). Where local_brain needs a model server and a GPU, this
runs on anything — including, eventually, the Pi, as the fallback brain
when the fancy ones are offline. Same tool surface, same guards; the
parser can want whatever it wants, the cliff detector still wins.

    python3 -m harness.intent                     # sim backend REPL
    python3 -m harness.intent --backend mock      # no MuJoCo
    python3 -m harness.intent --once "wave hello"
    echo "pebble, go to 0.4, 0.2" | python3 -m harness.intent --stdin

Understood (case-insensitive; number WORDS 1-100 work too: "thirty cm"):
  hello / hi / hey                    -> say greeting + wave
  go to (0.4, 0.2) | walk to 0.4 0.2  -> goto METERS, map frame
  go forward|back|left|right 30 cm    -> relative goto in the ROBOT's frame
                                         (forward = where it faces; needs
                                         status first for pose + yaw).
                                         A bare number >= 3 has no unit we
                                         can guess ("forward 3": cm? m?) —
                                         it ASKS instead of moving (D052)
  turn left / turn right / turn around -> gesture turn_in_place (signed)
  sidestep [left|right]               -> gesture sidestep (signed)
  stop / halt / freeze / whoa         -> stop
  jazz hands / fist bump / wave / bow / sit / shake / beckon /
  look around                         -> gesture
  say <word>                          -> chord-speak (fuzzy-matched)
  find the ball / go to the box       -> find_object (look, turn, walk up to it; cockpit only)
  where is the ball / where's the box -> where_is (scene memory: no looking, no walking)
  go back to the ball / return to the box -> go_back_to (walks to the remembered spot)
  remember that the charger is by the door / remember the charger is here
                                      -> remember (a note; "X is here" pins the pose)
  what do you remember [about the box] / recall the ball -> recall
  forget the ball / forget everything -> forget        (memory: cockpit only)
  what do you see / look / take a look -> look (the eye + a vision model)
                                         where the backend has one, else
                                         scan_summary
  what's around / scan / obstacles    -> scan_summary (the lidar)
  where are you / status / how are you / battery       -> status
  anything else                       -> say confused + a hint

VOICE (D052): speech is not typing. Whisper hallucinates whole phrases
out of fan noise (" Thank you." at no_speech_prob 1e-8 on 1.5 s of pink
noise, measured on this box 2026-09-24), so a voice line that would MOVE
the robot needs the wake word ("pebble, forward 30 cm") unless the
operator confirms it; clean_transcript() drops the known junk first.
whisper.cpp's stream binary emits one line per utterance — pipe it in:
  ./stream -m models/ggml-base.en.bin -t 4 --step 0 --length 3000 \\
      | python3 -m harness.intent --stdin          # wake word required
"""
from __future__ import annotations
import argparse
import asyncio
import difflib
import inspect
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from harness.backend import CHORD_WORDS, GESTURES, SIGNED  # noqa: E402,F401

# ---------------------------------------------------------------- parsing
_NUM = r"[-+]?\d+(?:\.\d+)?"
# gesture phrasings -> canonical gesture names ("turn around" et al.)
_GESTURE_ALIASES = {g.replace("_", " "): g for g in GESTURES}
_GESTURE_ALIASES.update({"turn around": "turn_in_place",
                         "spin": "turn_in_place",
                         "high five": "fist_bump",
                         "sit down": "sit",
                         "shake hands": "shake"})
_DIRS = {"forward": (1, 0), "forwards": (1, 0), "ahead": (1, 0),
         "back": (-1, 0), "backward": (-1, 0), "backwards": (-1, 0),
         "left": (0, 1), "right": (0, -1)}
MOTION_TOOLS = ("goto", "gesture", "compose_gesture", "find_object", "go_back_to")   # stop is NEVER gated
MEMORY_TOOLS = ("remember", "where_is", "recall", "go_back_to", "forget")          # the cockpit's scene memory
WAKE_WORDS = ("pebble", "pebbles", "peble", "pebbly", "rocky", "rockie", "rocket")   # D052 voice: the fast whisper (base.en) mishears the name; accept its usual guesses
UNITLESS_MAX = 3.0        # a bare number below this is meters; at/above it we ask

_ONES = {w: i for i, w in enumerate(
    "zero one two three four five six seven eight nine ten eleven twelve "
    "thirteen fourteen fifteen sixteen seventeen eighteen nineteen".split())}
_TENS = {w: 10 * (i + 2) for i, w in enumerate(
    "twenty thirty forty fifty sixty seventy eighty ninety".split())}
_TENS["fourty"] = 40                       # ASR spells it both ways
_UNIT_WORDS = [(r"centimet(?:er|re)s?|cms?\b", "cm"),
               (r"millimet(?:er|re)s?|mms?\b", "mm"),
               (r"met(?:er|re)s?\b", "m")]


def _num_words(t: str) -> str:
    """'thirty centimeters' -> '30 cm', 'twenty-five' -> '25', 'a hundred'
    -> '100', 'half a meter' -> '0.5 m' (1-100 is all the robot's room needs)."""
    t = re.sub(r"\bhalf an? (met(?:er|re))\b", r"0.5 \1", t)
    t = re.sub(r"\b(?:a|one) hundred\b", "100", t)
    tens = "|".join(_TENS)
    ones = "|".join(sorted(_ONES, key=len, reverse=True))
    t = re.sub(rf"\b({tens})(?:[\s-]+({ones}))?\b",
               lambda m: str(_TENS[m.group(1)] + (_ONES[m.group(2)] if m.group(2) else 0)), t)
    t = re.sub(rf"\b({ones})\b", lambda m: str(_ONES[m.group(1)]), t)
    t = re.sub(r"\ban? (met(?:er|re)|centimet(?:er|re))\b", r"1 \1", t)
    for pat, unit in _UNIT_WORDS:
        t = re.sub(rf"(\d)\s*(?:{pat})", rf"\1 {unit}", t)
    return t


def _norm(text: str) -> str:
    text = text.lower().strip()
    return re.sub(r"[^\w\s.,()+-]", "", text)     # ASR punctuation etc.


def _fuzzy_word(w: str):
    m = difflib.get_close_matches(w, CHORD_WORDS, n=1, cutoff=0.6)
    return m[0] if m else None


def has_wake_word(text: str, extra=()) -> bool:
    """'pebble, forward 30 cm' / 'hey rocky stop' — the voice gate (D052).
    Fuzzy: whisper's misspellings of the name (edit distance 1 on a 5-6 letter
    word) count, so 'pebbel stop' still passes; 'people' does not (cutoff 0.8).
    extra: more accepted names (the cockpit's operator-set robot name, D054)."""
    low = text.lower()
    names = tuple(WAKE_WORDS) + tuple(n.strip().lower() for n in extra if n and n.strip())
    if re.search(rf"\b({'|'.join(re.escape(n) for n in names)})\b", low):
        return True
    canon = ("pebble", "rocky") + tuple(n for n in names[len(WAKE_WORDS):] if len(n) >= 4)
    return any(difflib.get_close_matches(w, canon, n=1, cutoff=0.8) for w in re.findall(r"[a-z]+", low)[:4])


def strip_wake_word(text: str) -> str:
    return re.sub(rf"^\W*(?:(?:hey|ok|okay)\s+)?(?:{'|'.join(WAKE_WORDS)})\b[\s,.:!-]*", "",
                  text.strip(), flags=re.I) or text.strip()


def moves(p) -> bool:
    """Does this plan move the robot (the thing a stray voice line must not do)?"""
    return p.get("relative") is not None or any(n in MOTION_TOOLS for n, _ in p["calls"])


def _p(calls, reply, relative=None, **kw):
    return dict(calls=calls, reply=reply, relative=relative, **kw)


_FIND_FILLER = {"please", "now", "for", "me", "pebble", "rocky", "and", "then", "again", "it", "there"}
_MEM_FILLER = _FIND_FILLER | {"is", "was", "are", "were", "you", "saw", "seen", "went", "at", "last",
                              "earlier", "before", "go", "went", "left", "put"}
_ARTICLE = r"(?:the|a|an|that|my|those|these)"
# words that name no object: 'forget that' / 'forget the' / 'forget my stuff' must ASK,
# never wipe (review 2026-09-24: they erased every note and pin)
_NOT_A_NAME = {"that", "this", "those", "these", "it", "them", "one", "ones", "stuff", "things", "thing",
               "the", "a", "an", "my", "our", "your", "some", "any"}
# 'go back to the start / the beginning / where you started' -> the spawn pin (scene_memory.SPAWN_NAME)
_START_WORDS = {"start", "beginning", "spawn", "starting point", "start point", "starting position"}


def _names_nothing(name):
    return not name or all(w in _NOT_A_NAME for w in name.split())


def _ask(reply):
    return _p([("say", {"word": "curious_question"})], reply, ask=True)


def _noun(phrase, filler=_MEM_FILLER):
    """'red ball was please' -> 'red ball' (the noun phrase ends at a filler word; last 2 words)."""
    words = []
    for w in (phrase or "").split():
        if w in filler:
            break
        words.append(w)
    return " ".join(words[-2:])


def _memory_plan(t, raw):
    """The scene-memory phrasings (cockpit only) -> a plan, or None.
    t: the normalised text; raw: the wake-word-stripped original (a note keeps its words)."""
    m = re.match(r"^\W*(?:please\s+|ok\s+|okay\s+)?remember\b(?:\s+that)?[\s,:]+(.+)$", raw.strip(), re.I)
    if m and not re.match(r"^(?:what|where|when|how|anything|everything)\b", m.group(1).strip(), re.I):
        note = m.group(1).strip().rstrip(".!")
        return _p([("remember", {"note": note})], f"remembering: {note}")
    m = re.match(r"^\W*(?:please\s+)?(?:call|mark|name|label)\s+(?:this|here)\b.+$", raw.strip(), re.I)
    if m:                                  # 'call this spot home' / 'mark here as the dock': a pin
        note = raw.strip().rstrip(".!")
        return _p([("remember", {"note": note})], f"remembering: {note}")
    m = re.match(r"^(?:(?:no|nah|oh|ok|okay|please)[\s,]+)*forget\s+(?:about\s+)?(?:" + _ARTICLE +
                 r"\s+)?([a-z][a-z ]*)", t)
    if m:
        name = m.group(1).strip()
        name = "all" if re.match(r"^(everything|all|it all)\b", name) else _noun(name)
        if _names_nothing(name):
            return _ask("forget what? name it ('forget the ball'), or say 'forget everything'.")
        return _p([("forget", {"name": name})], f"forgetting {'everything' if name == 'all' else 'the ' + name}.")
    # 'go back to the box' / 'return to the ball' / 'go back home' / 'go back to where you
    # were' — never a relative move (review 2026-09-24: 'go back home' walked 0.2 m backwards)
    m = re.search(r"\b(?:(?:go|walk|head|come|get|run|move)\s+back|return)\s+(?:to\b\s*(.*)|(home)\b)", t)
    if m:
        rest = (m.group(1) if m.group(1) is not None else m.group(2) or "").strip()
        xy = re.match(rf"^\(?\s*({_NUM})\s*[, ]\s*({_NUM})", rest)
        if xy:
            x, y = float(xy.group(1)), float(xy.group(2))
            return _p([("goto", {"x": x, "y": y})], f"heading to ({x:g}, {y:g}) m.")
        if re.match(r"^where\s+(?:you|i|we)\s+(?:started|began|spawned|came\s+from)\b", rest):
            name = "start"
        elif re.match(r"^(?:where\s+(?:you|i|we|it)\b|where\s*$)", rest) or not rest:
            return _ask("back to where? name a remembered place ('go back to the ball', 'go back to the "
                        "start'), or say how far ('go back 30 cm').")
        else:
            rest = re.sub(r"^where\s+", "", rest)
            rest = re.sub(r"^(?:the|a|an|that|my|our|those|these)\s+", "", rest)
            name = _noun(rest)
            if name in _START_WORDS:
                name = "start"
            elif name == "origin":
                return _p([("goto", {"x": 0.0, "y": 0.0})], "heading to the origin (0, 0) m.")
        if _names_nothing(name):
            return _ask("back to what? name a remembered object or place, e.g. 'go back to the ball'.")
        return _p([("go_back_to", {"name": name})], f"going back to the {name}.")
    m = re.search(r"\bwhere(?:s|\s+is|\s+are|\s+was|\s+were|\s+did\s+you\s+(?:see|leave|put|find)|"
                  r"\s+have\s+you\s+seen)\s+(?:" + _ARTICLE + r"\s+)?([a-z][a-z ]*)", t)
    if m:
        name = _noun(m.group(1))
        if name and name.split()[0] not in ("you", "i", "we", "it", "am", "are", "they"):
            return _p([("where_is", {"name": name})], f"the {name}:")
    m = re.search(r"\b(?:what\s+do\s+you\s+(?:remember|know|recall)|do\s+you\s+remember|recall)\b"
                  r"(?:\s+about)?\s*(?:" + _ARTICLE + r"\s+)?([a-z ]*)", t)
    if m:
        q = _noun(m.group(1))
        return _p([("recall", {"query": q})], f"recalling{' ' + q if q else ''}:")
    return None


def plan(text: str):
    """text -> dict(calls=[(tool, args)...], reply=str, relative=(dx,dy)|None).

    Pure function, deterministic, no I/O — this is what the tests pin.
    relative goto is returned as a delta; the executor adds the pose from
    status (the parser has no idea where the robot is, by design).
    A plan that needs the operator to be clearer carries ask=True and no motion."""
    raw = strip_wake_word(text)
    t = _norm(raw)
    if not t:
        return _p([], "")

    # stop first — it must win over anything else in the sentence
    if re.search(r"\b(stop|halt|freeze|whoa|abort)\b", t):
        return _p([("stop", {})], "stopping.")

    # scene memory: remember / forget / go back to / where is / recall (before "go ... back"
    # is read as a relative move and "where" as status)
    mp = _memory_plan(t, raw)
    if mp is not None:
        return mp

    # explicit chord-speak: "say discovery"
    m = re.search(r"\bsay\s+(\w+)", t)
    if m:
        w = _fuzzy_word(m.group(1))
        if w:
            return _p([("say", {"word": w})], f"chord: {w}")
        return _p([("say", {"word": "confused"})],
                  f"no chord word like {m.group(1)!r} (have: {', '.join(CHORD_WORDS)})")

    # absolute goto: "go to (0.4, 0.2)" / "walk to 0.4 0.2"
    m = re.search(rf"\b(?:go|walk|head|move)\s+to\s*\(?\s*({_NUM})\s*[, ]\s*"
                  rf"({_NUM})\s*\)?", t)
    if m:
        x, y = float(m.group(1)), float(m.group(2))
        return _p([("goto", {"x": x, "y": y})], f"heading to ({x:g}, {y:g}) m.")

    # the eye, moving: "find the ball" / "go to the box" / "fetch the red ball" -> find_object
    # (an article is required after go/walk/come to, so "go to 0.4 0.2" never lands here)
    m = re.search(r"\b(?:find|fetch|approach|search\s+for|go\s+get)\s+(?:the\s+|a\s+|an\s+|that\s+|my\s+)?"
                  r"([a-z][a-z ]*)|\b(?:go|walk|head|come|move)\s+(?:over\s+)?to\s+(?:the|a|an|that|my)\s+"
                  r"([a-z][a-z ]*)", t)
    if m:
        words = []
        for w in (m.group(1) or m.group(2)).split():      # the noun phrase ends at a filler word
            if w in _FIND_FILLER:
                break
            words.append(w)
        words = words[-2:]                                  # "big red ball" -> "red ball"
        if words:
            name = " ".join(words)
            return _p([("find_object", {"name": name})], f"looking for the {name}.")

    # the eye: "what do you see" / "look" / "take a look" (bare "look around" is the gesture)
    if re.search(r"what.*\bsee\b|\bcan you see\b|\btake a look\b|\blook\b(?!\s+around\b)", t) or \
            re.search(r"look around.*\bsee\b", t):
        return _p([("look", {})], "looking.")

    # turning is a gesture, not a sidestep (D052: "turn left" used to walk 20 cm left)
    m = re.search(r"\b(turn|spin|rotate|pivot)\b(?:\s+(?:to\s+)?(?:the\s+)?(left|right))?", t)
    if m and (m.group(2) or not re.search(r"\b(left|right)\b", t)):
        d = m.group(2)
        args = {"name": "turn_in_place"}
        if d:
            args["direction"] = d
        return _p([("gesture", args)], f"turning{' ' + d if d else ''} in place.")
    m = re.search(r"\bside\s*step\b(?:\s+(?:to\s+)?(?:the\s+)?(left|right))?", t)
    if m:
        args = {"name": "sidestep"}
        if m.group(1):
            args["direction"] = m.group(1)
        return _p([("gesture", args)], f"sidestep{' ' + m.group(1) if m.group(1) else ''}.")

    # relative move: "go forward 30 cm" / "back 0.2 m" (defaults 0.2 m). Number
    # words only here: elsewhere they are words ("high five" is a fist bump)
    tn = _num_words(t)
    m = re.search(rf"\b(?:go|walk|move|step)?\s*\b({'|'.join(_DIRS)})\b"
                  rf"\s*({_NUM})?\s*(cm|mm|m)?\b", tn)
    if m and (m.group(1) in _DIRS) and \
            re.search(r"\b(go|walk|move|step|forward|forwards|back|backward|"
                      r"backwards|ahead|left|right)\b", tn):
        d = float(m.group(2)) if m.group(2) else 0.2
        unit = m.group(3)
        if unit is None and m.group(2) and abs(d) >= UNITLESS_MAX:
            return _p([("say", {"word": "curious_question"})],
                      f"{m.group(1)} {d:g} what — cm or m? Say e.g. '{m.group(1)} {d:g} cm'.",
                      ask=True)
        d *= {"m": 1.0, "cm": 0.01, "mm": 0.001}[unit or "m"]
        ux, uy = _DIRS[m.group(1)]
        return _p([], f"{m.group(1)} {d:g} m.", relative=(ux * d, uy * d))

    # gestures (longest alias first so "look around" beats "around")
    for phrase in sorted(_GESTURE_ALIASES, key=len, reverse=True):
        if phrase in t:
            # "what's around you" is a scan, not the look_around gesture
            if phrase == "look around" and re.search(r"what|see|near", t):
                break
            g = _GESTURE_ALIASES[phrase]
            return _p([("gesture", {"name": g})], f"gesture: {g}")

    if re.search(r"\b(hi|hello|hey|howdy|good (morning|evening|afternoon))\b", t):
        return _p([("say", {"word": "greeting"}), ("gesture", {"name": "wave"})],
                  "hello! (chord + wave)")

    if re.search(r"what.*(around|near)|scan|obstacle|look around|lidar", t):
        return _p([("scan_summary", {})], "scanning.")

    if re.search(r"where are you|status|how are you|battery|health|pose", t):
        return _p([("status", {})], "status:")

    if re.search(r"\b(thanks|thank you|good (bot|robot)|nice)\b", t):
        return _p([("say", {"word": "acknowledge"})], "chord: acknowledge")

    return _p([("say", {"word": "confused"})],
              "didn't catch that — try: hello / go to (x, y) / forward 30 cm / "
              "turn left / stop / jazz hands / say discovery / what do you see / status")


# ------------------------------------------------------------------ voice
# Whisper's favourite inventions on silence / fan noise / a keyboard (the
# first one reproduced here on pink noise). Matched on the whole cleaned line.
HALLUCINATIONS = {
    "you", "thank you", "thanks", "thank you very much", "thanks for watching",
    "thank you for watching", "thanks for watching and see you next time",
    "please subscribe", "subscribe", "bye", "bye bye", "okay", "ok", "so", "uh", "um",
    "hmm", "mm", "oh", "the", "i", "blank_audio", "blank audio", "silence", "music",
    "applause", "laughter", "inaudible", "no speech", "sigh", "beep",
}
NO_SPEECH_MAX = 0.6       # drop a segment whisper itself thinks is not speech
LOGPROB_MIN = -1.0        # ... or that it barely believes


def clean_transcript(resp) -> dict:
    """whisper-server verbose_json (dict) or plain text -> {text, dropped}.
    Drops segments with no_speech_prob > 0.6 or avg_logprob < -1.0, then the
    whole line if it is a known hallucination or shorter than 3 characters."""
    dropped = []
    if isinstance(resp, dict):
        keep = []
        segs = resp.get("segments")
        if segs is None:
            segs = [{"text": resp.get("text", "")}]
        for s in segs:
            txt = str(s.get("text", "")).strip()
            nsp, alp = s.get("no_speech_prob"), s.get("avg_logprob")
            if nsp is not None and nsp > NO_SPEECH_MAX:
                dropped.append(f"{txt!r} (no_speech {nsp:.2f})")
            elif alp is not None and alp < LOGPROB_MIN:
                dropped.append(f"{txt!r} (avg_logprob {alp:.2f})")
            else:
                keep.append(txt)
        text = " ".join(k for k in keep if k)
    else:
        text = str(resp or "")
    text = re.sub(r"\s+", " ", text).strip()
    core = re.sub(r"[\[\]().,!?*\s]+", " ", text.lower()).strip()
    if core in HALLUCINATIONS or len(core.replace(" ", "")) < 3:
        if text:
            dropped.append(f"{text!r} (known hallucination / too short)")
        text = ""
    return {"text": text, "dropped": dropped}


# --------------------------------------------------------------- executing
def _accepts(fn, kw):
    try:
        ps = inspect.signature(fn).parameters
    except (TypeError, ValueError):
        return True
    return kw in ps or any(p.kind == p.VAR_KEYWORD for p in ps.values())


async def execute(backend, p, allow_motion=True):
    """Run a plan against a backend; returns list of (tool, result).
    allow_motion=False (an unconfirmed voice line) refuses every motion call
    with a result that says why — stop still goes through."""
    out = []
    if p["relative"] is not None:
        if not allow_motion:
            return [("goto", _VOICE_REFUSAL)]
        st = await backend.status()
        pose = st.get("pose", {}) if isinstance(st.get("pose"), dict) else {}
        px = float(st.get("x", pose.get("x", 0.0)) or 0.0)
        py = float(st.get("y", pose.get("y", 0.0)) or 0.0)
        dx, dy = p["relative"]
        # "forward" is where the robot faces (D052: after "turn right", "forward
        # 20 cm" walked map +x, 44 deg off its nose); backends without a yaw = 0
        yaw = math.radians(float(pose.get("yaw_deg", 0.0) or 0.0))
        dx, dy = dx * math.cos(yaw) - dy * math.sin(yaw), dx * math.sin(yaw) + dy * math.cos(yaw)
        p = dict(p, calls=[("goto", {"x": round(px + dx, 3),
                                     "y": round(py + dy, 3)})])
    for name, args in p["calls"]:
        if name in MOTION_TOOLS and not allow_motion:
            out.append((name, _VOICE_REFUSAL))
            continue
        if name == "forget" and not allow_motion and str(args.get("name", "")).lower() in ("all", "everything"):
            out.append((name, _VOICE_REFUSAL))    # a misheard line must not wipe the memory
            continue
        if name == "look" and not hasattr(backend, "look"):
            name = "scan_summary"                  # no eye here: the lidar is the honest answer
        if name == "find_object" and not hasattr(backend, "find_object"):
            out.append((name, {"ok": False, "error": "no eye here: find_object needs the cockpit "
                                                     "(./rocky.sh cockpit)"}))
            continue
        if name in MEMORY_TOOLS and not hasattr(backend, name):
            out.append((name, {"ok": False, "error": f"no scene memory here: {name} needs the cockpit "
                                                     "(./rocky.sh cockpit)"}))
            continue
        fn = getattr(backend, name)
        if name == "gesture" and "direction" in args and not _accepts(fn, "direction"):
            # a backend without signed gestures turns/steps LEFT only — say so, never fake right
            if args["direction"] == "right":
                out.append((name, {"ok": False, "error": f"this backend cannot {args['name']} right "
                                   "(its gesture tool has no direction)"}))
                continue
            args = {k: v for k, v in args.items() if k != "direction"}
        try:
            out.append((name, await fn(**args)))
        except Exception as e:                     # a tool error is a result, not a crash
            out.append((name, {"ok": False, "error": f"{type(e).__name__}: {e}"}))
    return out


_VOICE_REFUSAL = {"ok": False, "error": "voice_unconfirmed",
                  "hint": "a spoken motion command needs the wake word "
                          "('pebble, forward 30 cm') or a typed confirmation"}


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="sim", choices=["sim", "mock"])
    ap.add_argument("--once", default=None, help="one line, then exit")
    ap.add_argument("--stdin", action="store_true",
                    help="read lines from stdin (the whisper.cpp pipe): motion needs the wake word")
    ap.add_argument("--no-wake", action="store_true",
                    help="(--stdin) no wake word needed — for push-to-talk, where pressing the "
                         "key IS the confirmation (transcripts are still cleaned)")
    args = ap.parse_args()
    from harness.local_brain import make_backend
    backend = make_backend(args.backend)
    if not args.stdin and not args.once:
        print(f"intent REPL (backend={args.backend}) — plain text, no LLM. "
              f"'quit' exits.")
    voice = args.stdin and not args.no_wake          # motion needs the wake word
    clean = args.stdin                               # D052 V2: push-to-talk keeps the transcript cleaner

    async def handle(line):
        if clean:
            line = clean_transcript(line)["text"]
            if not line:
                return
        p = plan(line)
        ok = not voice or has_wake_word(line) or not moves(p)
        if p["reply"]:
            print(f"pebble> {p['reply']}" + ("" if ok else "  (needs the wake word — not moving)"))
        for name, res in await execute(backend, p, allow_motion=ok):
            print(f"  [tool] {name} -> {json.dumps(res)[:140]}")

    if args.once:
        await handle(args.once)
        return
    stream = sys.stdin if args.stdin else None
    while True:
        try:
            line = stream.readline() if stream else input("you> ")
        except EOFError:
            break
        if stream and not line:
            break
        line = line.strip()
        if line == "quit":
            break
        if line:
            await handle(line)


if __name__ == "__main__":
    asyncio.run(main())
