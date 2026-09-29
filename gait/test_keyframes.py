"""D063 (B76 fix 8): keyframe blends are minimum-jerk, and every duration the
module sizes from a speed budget accounts for the quintic's 1.875 x peak.

    python -m pytest gait/test_keyframes.py -q
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rocky_model as rm                                               # noqa: E402
import pebble_keyframes as pk                                          # noqa: E402
from pebble_gait import WaveGait                                       # noqa: E402

FS = 1000.0                                                            # 1 ms: fine enough for accelerations


@pytest.fixture(scope="module")
def g():
    return WaveGait()


def stream(kg, g, t0, t1):
    ts = np.arange(t0, t1 + 1e-9, 1.0 / FS)
    return ts, np.array([kg(g, t)[0] for t in ts])


def test_smooth_ease_is_the_minimum_jerk_quintic():
    u = np.linspace(0.0, 1.0, 10001)
    s = np.array([pk._ease(x, "smooth") for x in u])
    assert s[0] == 0.0 and s[-1] == 1.0 and s[5000] == pytest.approx(0.5)
    assert np.allclose(s, 10 * u**3 - 15 * u**4 + 6 * u**5)
    v = np.gradient(s, u)
    assert v.max() == pytest.approx(pk.SMOOTH_PEAK, abs=1e-3)       # the peak the budgets are sized with
    a = np.gradient(v, u)
    assert abs(a[1]) < 0.01 and abs(a[-2]) < 0.01                    # no acceleration step at either end
    assert [pk._ease(x, "linear") for x in (0.0, 0.3, 1.0)] == [0.0, 0.3, 1.0]


def test_no_acceleration_kick_at_a_key(g):
    """The fault: the smoothstep's acceleration jumps from 0 to 6 dq/T^2 at every
    key. 10 ms into a 1 s arm raise the quintic asks 0.58 dq/T^2."""
    spec = {"name": "raise", "end": "hold",
            "keyframes": [{"t": 0.0}, {"t": 1.0, "arm": {"0": [0, 60, -60]}}]}
    kg = pk.KeyframeGesture(spec)
    ts, Q = stream(kg, g, 0.0, 0.02)
    dq = abs(math.radians(60) - kg.frames[0].pose(g)[0][0, 1])        # the hip's travel (rad)
    acc = np.diff(Q[:, 0, 1], 2) * FS * FS                             # rad/s^2
    kick = 6.0 * dq                                                    # smoothstep at u = 0, T = 1 s
    assert abs(acc[:10]).max() < 0.12 * kick


def test_hold_move_and_tail_are_sized_for_the_quintic_peak(g):
    """Both code-sized moves reach 90 % of their class and no more: sized with
    1.5 (the smoothstep's ratio) the quintic would peak at 1.125 x 90 %."""
    free, loaded = rm.servo_speed("free"), rm.servo_speed("loaded")
    spec = {"name": "hold_then_tail", "keyframes": [
        {"t": 0.0},
        {"t": 2.0, "arm": {"0": [0, 70, -60]}, "ease": "hold"},
        {"t": 2.5, "arm": {"0": [0, 70, -60]}, "body": [0, 0, -20]}]}
    kg = pk.KeyframeGesture(spec)
    a, b = kg.frames[0], kg.frames[1]
    T = kg._hold_dur(0, g)
    dq = np.abs(b.pose(g)[0] - a.pose(g)[0])[0].max()
    assert T == pytest.approx(pk.SMOOTH_PEAK * dq / (pk.BUDGET_FRAC * free))
    ts, Q = stream(kg, g, 0.0, T)
    v = np.abs(np.diff(Q[:, 0], axis=0)).max() * FS
    assert 0.97 * pk.BUDGET_FRAC * free < v <= pk.BUDGET_FRAC * free * 1.001
    assert np.allclose(kg(g, 1.99)[0][0], np.deg2rad([0, 70, -60]))  # arrived early, holding
    last, tail = kg.frames[-2], kg.frames[-1]
    assert tail.nominal and kg.total > 2.5
    ts, Q = stream(kg, g, last.t, tail.t)
    V = np.abs(np.diff(Q, axis=0)) * FS
    assert V[:, 0].max() <= pk.BUDGET_FRAC * free * 1.001             # the arm leg: free class
    assert V[:, 1:].max() <= pk.BUDGET_FRAC * loaded * 1.001          # the planted legs: loaded class


def test_linear_and_hold_keep_their_meaning(g):
    spec = {"name": "lin", "end": "hold", "keyframes": [
        {"t": 0.0}, {"t": 1.0, "arm": {"0": [0, 60, -60]}, "ease": "linear"}]}
    kg = pk.KeyframeGesture(spec)
    q0, q1 = kg(g, 0.0)[0][0, 1], kg(g, 1.0)[0][0, 1]
    for t in (0.25, 0.5, 0.75):
        assert kg(g, t)[0][0, 1] == pytest.approx(q0 + t * (q1 - q0))
    hold = dict(spec, keyframes=[{"t": 0.0}, {"t": 3.0, "arm": {"0": [0, 60, -60]}, "ease": "hold"}])
    kh = pk.KeyframeGesture(hold)
    T = kh._hold_dur(0, g)
    assert T < 3.0 and kh(g, T)[0][0, 1] == pytest.approx(q1) and kh(g, 2.9)[0][0, 1] == pytest.approx(q1)
