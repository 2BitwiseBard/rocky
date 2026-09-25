"""Tests for the deterministic intent layer (session 8d).

Parser tests are pure (plan() is a pure function); executor tests run
against MockBackend — millisecond-fast. They are PLUMBING tests: the
mock's "cliff" is a scripted x-threshold, so a green run proves the parser
and executor pass vetoes through, not that the robot is safe (that is
test_harness.py's slow physics test and sim/run_cliff*.py).

    python3 -m pytest harness/test_intent.py -q
"""
import asyncio
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from harness.backend import MockBackend, GESTURES, CHORD_WORDS   # noqa: E402
from harness.intent import (plan, execute, clean_transcript,              # noqa: E402
                            has_wake_word, strip_wake_word, moves)


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


def test_scan_vs_look_vs_look_around_gesture():
    # bare "look around" = the gesture; what do you SEE = the eye (look);
    # what is AROUND / scan = the lidar (D052: "see" used to go to the lidar)
    assert calls("look around") == [("gesture", {"name": "look_around"})]
    assert calls("look around, what do you see?") == [("look", {})]
    assert calls("what do you see") == [("look", {})]
    assert calls("look") == [("look", {})]
    assert calls("take a look") == [("look", {})]
    assert calls("what's around you?") == [("scan_summary", {})]
    assert calls("scan") == [("scan_summary", {})]


def test_turn_left_right_is_a_signed_turn_not_a_sidestep():
    # D052: "turn left" used to parse as a relative goto 20 cm to the left
    p = plan("turn left")
    assert p["relative"] is None
    assert p["calls"] == [("gesture", {"name": "turn_in_place", "direction": "left"})]
    assert calls("Turn right.") == [("gesture", {"name": "turn_in_place", "direction": "right"})]
    assert calls("rotate to the right") == [("gesture", {"name": "turn_in_place",
                                                         "direction": "right"})]
    assert calls("turn around") == [("gesture", {"name": "turn_in_place"})]
    assert calls("sidestep right") == [("gesture", {"name": "sidestep", "direction": "right"})]
    # moving left is still a move, not a turn
    assert plan("go left 10 cm")["relative"] == (0.0, 0.1)


def test_number_words():
    assert plan("go forward thirty centimeters")["relative"] == (0.3, 0.0)
    dx, dy = plan("walk back twenty-five cm")["relative"]
    assert abs(dx + 0.25) < 1e-9 and dy == 0.0
    assert plan("move left fifteen centimetres")["relative"] == (0.0, 0.15)
    assert plan("forward half a meter")["relative"] == (0.5, 0.0)
    assert plan("forward a hundred millimeters")["relative"] == (0.1, 0.0)
    assert plan("forward one meter")["relative"] == (1.0, 0.0)


def test_bare_number_needs_a_unit_when_ambiguous():
    # "forward 3": 3 cm? 3 m? (3 m is a 70 s walk) — ask, never guess
    for s in ("go forward 3", "forward three", "back 30", "walk left 12"):
        p = plan(s)
        assert p["relative"] is None and p.get("ask"), s
        assert not moves(p), s
        assert "cm or m" in p["reply"], s
    assert plan("forward 3 cm")["relative"] == (0.03, 0.0)
    assert plan("forward 0.5")["relative"] == (0.5, 0.0)          # < 3 with no unit = meters


def test_wake_word():
    assert has_wake_word("Pebble, go forward 30 cm")
    assert has_wake_word("hey rocky stop")
    assert not has_wake_word("go forward 30 cm")
    assert strip_wake_word("Pebble, go forward 30 cm") == "go forward 30 cm"
    assert plan("pebble, turn left")["calls"] == [("gesture", {"name": "turn_in_place",
                                                              "direction": "left"})]
    assert calls("hey pebble") == [("say", {"word": "greeting"}), ("gesture", {"name": "wave"})]


def test_clean_transcript_drops_whisper_junk():
    # measured 2026-09-24: 1.5 s of pink noise -> " Thank you." at no_speech 1e-8
    noise = {"text": " Thank you.\n", "segments": [
        {"text": " Thank you.", "avg_logprob": -0.43, "no_speech_prob": 1.2e-8}]}
    assert clean_transcript(noise)["text"] == ""
    for junk in ("you", "[BLANK_AUDIO]", "(silence)", "  .  ", "ok", "Thanks for watching!"):
        assert clean_transcript(junk)["text"] == "", junk
    segs = {"segments": [{"text": " go forward", "avg_logprob": -0.2, "no_speech_prob": 0.01},
                         {"text": " 30 cm", "avg_logprob": -0.3, "no_speech_prob": 0.02},
                         {"text": " la la", "avg_logprob": -1.4, "no_speech_prob": 0.1},
                         {"text": " hmm", "avg_logprob": -0.1, "no_speech_prob": 0.9}]}
    c = clean_transcript(segs)
    assert c["text"] == "go forward 30 cm" and len(c["dropped"]) == 2
    assert clean_transcript("Pebble, stop.")["text"] == "Pebble, stop."


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


def test_execute_relative_is_in_the_robot_frame():
    class Yawed(MockBackend):
        async def status(self):
            st = await MockBackend.status(self)
            st["pose"]["yaw_deg"] = 90.0          # facing map +y
            return st
    b = Yawed()
    b.x, b.y = -0.30, 0.0
    out = _run(execute(b, plan("go forward 10 cm")))
    assert out[0][1]["ok"]
    assert abs(b.x + 0.30) < 0.02 and abs(b.y - 0.10) < 0.02


def test_execute_guard_still_wins():
    """PLUMBING: the mock's scripted void (an x threshold) comes back through
    plan -> execute as stopped='cliff'. Proves the veto is passed through,
    not that the real detector fires (that is the slow physics test)."""
    b = MockBackend()
    out = _run(execute(b, plan("go to (2.0, 0.0)")))
    res = out[0][1]
    assert res.get("stopped") == "cliff"
    st = _run(b.status())
    assert st["pose"]["x"] < 0.35                       # short of EDGE_X


def test_execute_look_falls_back_to_the_lidar_without_an_eye():
    b = MockBackend()                          # no look(): the honest answer is the lidar
    out = _run(execute(b, plan("what do you see?")))
    assert out[0][0] == "scan_summary" and out[0][1]["ok"]


def test_find_the_ball_is_find_object_and_moves():
    assert calls("find the ball") == [("find_object", {"name": "ball"})]
    assert calls("pebble, go to the box please") == [("find_object", {"name": "box"})]
    assert calls("find the big red ball") == [("find_object", {"name": "red ball"})]
    assert calls("walk to the door and wave") == [("find_object", {"name": "door"})]
    assert calls("go to 0.4 0.2") == [("goto", {"x": 0.4, "y": 0.2})]      # numbers stay a goto
    assert calls("look for the ball") == [("look", {})]                  # looking never walks
    assert moves(plan("find the ball"))                                  # voice needs the wake word
    assert calls("stop finding the ball") == [("stop", {})]


def test_execute_find_object_without_an_eye_says_so():
    b = MockBackend()                          # no find_object(): no eye, refused honestly
    out = _run(execute(b, plan("find the ball")))
    assert out[0][0] == "find_object" and out[0][1]["ok"] is False and "cockpit" in out[0][1]["error"]
    out = _run(execute(b, plan("find the ball"), allow_motion=False))
    assert out[0][1]["error"] == "voice_unconfirmed"


def test_execute_signed_gesture():
    b = MockBackend()
    out = _run(execute(b, plan("turn right")))
    assert out[0][1] == {"ok": True, "gesture": "turn_in_place", "direction": "right"}

    class NoDirection(MockBackend):            # e.g. SimBackend: gesture(name) only
        async def gesture(self, name: str) -> dict:
            return await MockBackend.gesture(self, name)
    nb = NoDirection()
    r = _run(execute(nb, plan("turn right")))[0][1]
    assert r["ok"] is False and "right" in r["error"]         # never fakes a right turn
    r = _run(execute(nb, plan("turn left")))[0][1]
    assert r["ok"] is True                                     # the canon turn IS left


def test_execute_unconfirmed_voice_does_not_move_but_stop_works():
    b = MockBackend()
    b.x, b.y = -0.30, 0.0
    for s in ("go forward 10 cm", "go to (-0.2, 0)", "do jazz hands"):
        out = _run(execute(b, plan(s), allow_motion=False))
        assert out[0][1]["error"] == "voice_unconfirmed", s
    assert b.x == -0.30 and not [e for e in b.events if e[0] == "gesture"]
    out = _run(execute(b, plan("stop"), allow_motion=False))
    assert out[0] == ("stop", {"ok": True, "mode": "safe_stop"})


def test_execute_tool_exception_is_a_result():
    class Broken(MockBackend):
        async def scan_summary(self):
            raise RuntimeError("lidar unplugged")
    out = _run(execute(Broken(), plan("scan")))
    assert out[0][1]["ok"] is False and "lidar unplugged" in out[0][1]["error"]


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


def test_wake_word_tolerates_whisper_misspellings():
    """D052 voice: base.en hears 'Pebbel' / 'Peble' for the name; 'people' must not pass."""
    from harness.intent import has_wake_word
    assert has_wake_word("Pebbel, stop") and has_wake_word("Peble bow") and has_wake_word("hey rocky sit")
    assert not has_wake_word("people walk forward") and not has_wake_word("walk forward thirty centimeters")


# ------------------------------------------------------------- scene memory (cockpit)
def test_memory_routes():
    assert calls("where is the ball") == [("where_is", {"name": "ball"})]
    assert calls("Where's the red ball?") == [("where_is", {"name": "red ball"})]
    assert calls("where did you see the box") == [("where_is", {"name": "box"})]
    assert calls("where are you") == [("status", {})]                # still status
    assert calls("go back to the box") == [("go_back_to", {"name": "box"})]
    assert calls("pebble, return to the ball please") == [("go_back_to", {"name": "ball"})]
    assert calls("remember that the charger is by the door") == [
        ("remember", {"note": "the charger is by the door"})]
    assert calls("Remember: the Charger is here.") == [("remember", {"note": "the Charger is here"})]
    assert calls("what do you remember") == [("recall", {"query": ""})]
    assert calls("what do you remember about the box") == [("recall", {"query": "box"})]
    assert calls("forget the ball") == [("forget", {"name": "ball"})]
    assert calls("forget everything") == [("forget", {"name": "all"})]


def test_memory_routes_do_not_steal_motion_or_stop():
    assert plan("go back 30 cm")["relative"] == (-0.3, 0.0)          # a relative move, not go_back_to
    assert calls("find the ball") == [("find_object", {"name": "ball"})]
    assert calls("remember to stop") == [("stop", {})]                # stop always wins
    assert moves(plan("go back to the box")) and not moves(plan("where is the box"))
    assert not moves(plan("remember that the ball is here"))


def test_memory_tools_on_a_backend_without_memory_say_so():
    res = asyncio.run(execute(MockBackend(), plan("where is the ball")))
    assert res[0][0] == "where_is" and res[0][1]["ok"] is False
    assert "no scene memory" in res[0][1]["error"]
    res = asyncio.run(execute(MockBackend(), plan("go back to the ball"), allow_motion=False))
    assert res[0][1]["error"] == "voice_unconfirmed"                  # gated before anything else


def test_go_back_somewhere_is_never_a_relative_move():
    """Review 2026-09-24: 'go back home' / 'go back to where you were' walked 0.2 m backwards."""
    for text in ("go back to where you were", "go back to where I was", "come back to me"):
        p = plan(text)
        assert p.get("ask") and not moves(p) and p["relative"] is None, text
    assert calls("go back home") == [("go_back_to", {"name": "home"})]
    assert calls("return to base") == [("go_back_to", {"name": "base"})]
    assert calls("go back to the start") == [("go_back_to", {"name": "start"})]
    assert calls("go back to where you started") == [("go_back_to", {"name": "start"})]
    assert calls("go back to the beginning") == [("go_back_to", {"name": "start"})]
    assert calls("go back to where the ball was") == [("go_back_to", {"name": "ball"})]
    assert calls("go back to (0.4, 0.2)") == [("goto", {"x": 0.4, "y": 0.2})]
    assert plan("go back 30 cm")["relative"] == (-0.3, 0.0) and plan("go back")["relative"] == (-0.2, 0.0)


def test_forget_a_pronoun_asks_and_never_wipes():
    for text in ("forget that", "no, forget that", "forget the", "forget those", "forget my stuff", "forget it"):
        p = plan(text)
        assert p.get("ask") and not any(n == "forget" for n, _ in p["calls"]), text
    assert calls("forget it all") == [("forget", {"name": "all"})]


def test_naming_this_spot_is_remembered():
    assert calls("call this spot home") == [("remember", {"note": "call this spot home"})]
    assert calls("pebble, mark here as the dock") == [("remember", {"note": "mark here as the dock"})]
    assert calls("remember this spot as home") == [("remember", {"note": "this spot as home"})]


def test_a_spoken_forget_everything_needs_the_wake_word():
    class Mem(MockBackend):
        async def forget(self, name):
            return {"ok": True, "forgot": name}
    res = asyncio.run(execute(Mem(), plan("forget everything"), allow_motion=False))
    assert res[0][1]["error"] == "voice_unconfirmed"
    res = asyncio.run(execute(Mem(), plan("forget the ball"), allow_motion=False))
    assert res[0][1]["ok"] is True
