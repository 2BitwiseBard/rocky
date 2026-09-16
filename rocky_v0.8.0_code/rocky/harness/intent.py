#!/usr/bin/env python3
"""intent — talk to Pebble in plain text, no LLM required (session 8d).

The deterministic layer of the VISION_PLAN brain stack: a regex/keyword
intent parser that maps operator text onto the SAME six tools the MCP
server and local_brain expose (say / gesture / goto / stop / scan_summary
/ status). Where local_brain needs an Ollama daemon and a GPU, this runs
on anything — including, eventually, the Pi, as the fallback brain when
the fancy ones are offline. Same tool surface, same guards; the parser
can want whatever it wants, the cliff detector still wins.

    python3 -m harness.intent                     # sim backend REPL
    python3 -m harness.intent --backend mock      # no MuJoCo
    python3 -m harness.intent --once "wave hello"
    echo "go to 0.4, 0.2" | python3 -m harness.intent --stdin

Understood (case-insensitive):
  hello / hi / hey                    -> say greeting + wave
  go to (0.4, 0.2) | walk to 0.4 0.2  -> goto METERS, map frame
  go forward|back|left|right 30 cm    -> relative goto (+x = forward,
                                         +y = left; needs status first)
  stop / halt / freeze / whoa         -> stop
  jazz hands / fist bump / wave / bow / sit / shake / beckon /
  look around / turn in place|around / sidestep        -> gesture
  say <word>                          -> chord-speak (fuzzy-matched)
  what do you see / what's around / scan               -> scan_summary
  where are you / status / how are you / battery       -> status
  anything else                       -> say confused + a hint

VOICE HOOK (documented, not shipped): whisper.cpp's stream binary emits
one transcribed line per utterance on stdout; pipe it straight in —
  ./stream -m models/ggml-base.en.bin -t 4 --step 0 --length 3000 \
      | python3 -m harness.intent --stdin
On the Pi the same pipe runs against the hardware backend. Punctuation
and casing from ASR are already handled (we normalize both).
"""
from __future__ import annotations
import argparse
import asyncio
import difflib
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from harness.backend import CHORD_WORDS, GESTURES          # noqa: E402

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


def _norm(text: str) -> str:
    text = text.lower().strip()
    return re.sub(r"[^\w\s.,()+-]", "", text)     # ASR punctuation etc.


def _fuzzy_word(w: str):
    m = difflib.get_close_matches(w, CHORD_WORDS, n=1, cutoff=0.6)
    return m[0] if m else None


def plan(text: str):
    """text -> dict(calls=[(tool, args)...], reply=str, relative=(dx,dy)|None).

    Pure function, deterministic, no I/O — this is what the tests pin.
    relative goto is returned as a delta; the executor adds the pose from
    status (the parser has no idea where the robot is, by design)."""
    t = _norm(text)
    if not t:
        return dict(calls=[], reply="", relative=None)

    # stop first — it must win over anything else in the sentence
    if re.search(r"\b(stop|halt|freeze|whoa|abort)\b", t):
        return dict(calls=[("stop", {})], reply="stopping.", relative=None)

    # explicit chord-speak: "say discovery"
    m = re.search(r"\bsay\s+(\w+)", t)
    if m:
        w = _fuzzy_word(m.group(1))
        if w:
            return dict(calls=[("say", {"word": w})],
                        reply=f"chord: {w}", relative=None)
        return dict(calls=[("say", {"word": "confused"})],
                    reply=f"no chord word like {m.group(1)!r} "
                          f"(have: {', '.join(CHORD_WORDS)})", relative=None)

    # absolute goto: "go to (0.4, 0.2)" / "walk to 0.4 0.2"
    m = re.search(rf"\b(?:go|walk|head|move)\s+to\s*\(?\s*({_NUM})\s*[, ]\s*"
                  rf"({_NUM})\s*\)?", t)
    if m:
        x, y = float(m.group(1)), float(m.group(2))
        return dict(calls=[("goto", {"x": x, "y": y})],
                    reply=f"heading to ({x:g}, {y:g}) m.", relative=None)

    # relative move: "go forward 30 cm" / "back 0.2 m" (defaults 0.2 m)
    m = re.search(rf"\b(?:go|walk|move|step)?\s*\b({'|'.join(_DIRS)})\b"
                  rf"\s*({_NUM})?\s*(cm|mm|m)?\b", t)
    if m and (m.group(1) in _DIRS) and \
            re.search(r"\b(go|walk|move|step|forward|forwards|back|backward|"
                      r"backwards|ahead|left|right)\b", t):
        d = float(m.group(2)) if m.group(2) else 0.2
        unit = m.group(3) or ("m" if d < 3 else "cm")
        d *= {"m": 1.0, "cm": 0.01, "mm": 0.001}[unit]
        ux, uy = _DIRS[m.group(1)]
        return dict(calls=[], reply=f"{m.group(1)} {d:g} m.",
                    relative=(ux * d, uy * d))

    # gestures (longest alias first so "look around" beats "around")
    for phrase in sorted(_GESTURE_ALIASES, key=len, reverse=True):
        if phrase in t:
            # "what's around you" is a scan, not the look_around gesture
            if phrase == "look around" and re.search(r"what|see|near", t):
                break
            g = _GESTURE_ALIASES[phrase]
            return dict(calls=[("gesture", {"name": g})],
                        reply=f"gesture: {g}", relative=None)

    if re.search(r"\b(hi|hello|hey|howdy|good (morning|evening|afternoon))\b", t):
        return dict(calls=[("say", {"word": "greeting"}),
                           ("gesture", {"name": "wave"})],
                    reply="hello! (chord + wave)", relative=None)

    if re.search(r"what.*(see|around|near)|scan|obstacle|look around", t):
        return dict(calls=[("scan_summary", {})], reply="scanning.",
                    relative=None)

    if re.search(r"where are you|status|how are you|battery|health|pose", t):
        return dict(calls=[("status", {})], reply="status:", relative=None)

    if re.search(r"\b(thanks|thank you|good (bot|robot)|nice)\b", t):
        return dict(calls=[("say", {"word": "acknowledge"})],
                    reply="chord: acknowledge", relative=None)

    return dict(calls=[("say", {"word": "confused"})],
                reply="didn't catch that — try: hello / go to (x, y) / "
                      "forward 30 cm / stop / jazz hands / say discovery / "
                      "what do you see / status", relative=None)


# --------------------------------------------------------------- executing
async def execute(backend, p):
    """Run a plan against a backend; returns list of (tool, result)."""
    out = []
    if p["relative"] is not None:
        st = await backend.status()
        px = float(st.get("x", st.get("pose", {}).get("x", 0.0)) or 0.0)
        py = float(st.get("y", st.get("pose", {}).get("y", 0.0)) or 0.0)
        dx, dy = p["relative"]
        p = dict(p, calls=[("goto", {"x": round(px + dx, 3),
                                     "y": round(py + dy, 3)})])
    for name, args in p["calls"]:
        fn = getattr(backend, name)
        out.append((name, await fn(**args)))
    return out


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="sim", choices=["sim", "mock"])
    ap.add_argument("--once", default=None, help="one line, then exit")
    ap.add_argument("--stdin", action="store_true",
                    help="read lines from stdin (the whisper.cpp pipe)")
    args = ap.parse_args()
    from harness.local_brain import make_backend
    backend = make_backend(args.backend)
    if not args.stdin and not args.once:
        print(f"intent REPL (backend={args.backend}) — plain text, no LLM. "
              f"'quit' exits.")

    async def handle(line):
        p = plan(line)
        if p["reply"]:
            print(f"pebble> {p['reply']}")
        for name, res in await execute(backend, p):
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
