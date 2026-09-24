"""CliffDetector: the D050 terrain-aware option (pure Python) and the
random course generator."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "perception"))
sys.path.insert(0, os.path.join(HERE, ".."))
from cliff import CliffDetector       # noqa: E402


def _run(det, below):
    fired = []
    t = 0.0
    for _ in range(20):                      # 0.2 s of a planted-commanded, contactless leg 0
        fired += det.update(t, [True] * 5, [False, True, True, True, True], probed_out=below)
        t += 0.01
    return fired


def test_void_needs_an_exhausted_probe_when_told():
    assert _run(CliffDetector(), None) == [0]                          # legacy: timing alone
    assert _run(CliffDetector(), [True, False, False, False, False]) == [0]   # probed out, no contact: void
    assert _run(CliffDetector(), [False] * 5) == []                    # still feeling for the floor


def test_random_course_is_seeded_and_clear_at_the_origin():
    from world_builder import random_course
    a, b = random_course(7, n=10), random_course(7, n=10)
    assert a == b
    assert len(a["objects"]) == 10
    for o in a["objects"]:
        assert (o["pos"][0] ** 2 + o["pos"][1] ** 2) ** 0.5 >= 0.45
    assert random_course(8, n=10) != a
