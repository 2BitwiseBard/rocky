"""Tests for the deterministic intent layer (session 8d).

Parser tests are pure (plan() is a pure function); executor tests run
against MockBackend — millisecond-fast, and the guard behavior (cliff
stop) comes with it for free.

    python3 -m pytest harness/test_intent.py -q
"""
import asyncio
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from harness.backend import MockBackend, GESTURES, CHORD_WORDS   # noqa: E402
from harness.intent import plan, execute                         # noqa: E402


def calls(text):
    return plan(text)["calls"]


# ------------------------------------------------------------- the parser
def test_greeting():
    assert calls("Hello there!") == [("say", {"word": "greeting"}),
                                     ("gesture", {"name": "wave"})]


def test_goto_absolute_variants():
    for s in ("go to (0.4, 0.2)", "walk to 0.4, 0.2", "head to 0.4 0.2",
              "GO TO (0.4,0.2)"):
        assert calls(s) == [("goto", {"x": 0.4, "y": 0.2})], s


def test_goto_negative():
    assert calls("go to (-0.3, 0.1)") == [("goto", {"x": -0.3, "y": 0.1})]


def test_relative_move_units():
    p = plan("go forward 30 cm")
    assert p["calls"] == [] and p["relative"] == (0.3, 0.0)
    p = plan("move left 0.2 m")
    assert p["relative"] == (0.0, 0.2)
    p = plan("step back 100 mm")
    dx, dy = p["relative"]
    assert abs(dx + 0.1) < 1e-9 and dy == 0.0


def test_stop_wins_over_everything():
    assert calls("stop") == [("stop", {})]
    assert calls("whoa whoa stop right there and wave") == [("stop", {})]


def test_gestures_and_aliases():
    assert calls("do jazz hands") == [("gesture", {"name": "jazz_hands"})]
    assert calls("turn around") == [("gesture", {"name": "turn_in_place"})]
    assert calls("give me a high five") == [("gesture", {"name": "fist_bump"})]
    assert calls("take a bow") == [("gesture", {"name": "bow"})]


def test_scan_vs_look_around_gesture():
    # bare "look around" = the gesture; asking WHAT is around = lidar scan
    assert calls("look around") == [("gesture", {"name": "look_around"})]
    assert calls("look around, what do you see?") == [("scan_summary", {})]
    assert calls("what's around you?") == [("scan_summary", {})]


def test_say_fuzzy():
    assert calls("say discovery") == [("say", {"word": "discovery"})]
    assert calls("say greetings") == [("say", {"word": "greeting"})]


def test_status():
    assert calls("where are you?") == [("status", {})]


def test_unknown_says_confused_with_hint():
    p = plan("flurble the wug")
    assert p["calls"] == [("say", {"word": "confused"})]
    assert "go to" in p["reply"]


def test_parser_only_emits_real_words_and_gestures():
    probes = ["hello", "thanks", "say discvery", "do jazz hands", "sit down",
              "flurble", "turn around", "shake hands"]
    for s in probes:
        for name, args in calls(s):
            if name == "say":
                assert args["word"] in CHORD_WORDS, (s, args)
            if name == "gesture":
                assert args["name"] in GESTURES, (s, args)


# ----------------------------------------------------------- the executor
def _run(coro):
    return asyncio.get_event_loop().run_until_complete(coro) \
        if sys.version_info < (3, 10) else asyncio.run(coro)


def test_execute_relative_uses_pose():
    b = MockBackend()
    b.x, b.y = -0.30, 0.05                # clear of the mock's void at +x
    out = _run(execute(b, plan("go forward 10 cm")))
    assert out[0][0] == "goto" and out[0][1]["ok"]
    st = _run(b.status())
    assert abs(st["pose"]["x"] + 0.20) < 0.02
    assert abs(st["pose"]["y"] - 0.05) < 0.02


def test_execute_guard_still_wins():
    # walk the mock straight at its void: the guard stops it, parser or no
    b = MockBackend()
    out = _run(execute(b, plan("go to (2.0, 0.0)")))
    res = out[0][1]
    assert res.get("stopped") == "cliff"
    st = _run(b.status())
    assert st["pose"]["x"] < 0.35                       # short of EDGE_X


def test_execute_full_greeting():
    b = MockBackend()
    out = _run(execute(b, plan("hi!")))
    assert [o[0] for o in out] == ["say", "gesture"]
    assert all(o[1]["ok"] for o in out)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"{len(fns)}/{len(fns)} intent tests green")
