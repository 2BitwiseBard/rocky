"""FALLEN/RIGHTED state-machine tests (session 8d, D042) — pure Python,
no MuJoCo, no torch: the supervisor is fed synthetic IMU signals and a
scripted righter, and must walk the NORMAL -> FALLEN -> RIGHTED -> NORMAL
path exactly, never emit NaN, and never fire on a transient tilt spike.

    python3 -m pytest gait/test_reflex_fallen.py -q
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from pebble_gait import WaveGait, N_LEGS                            # noqa: E402
from pebble_reflex import (ReflexSupervisor, NORMAL, FALLEN,        # noqa: E402
                           RIGHTED)

DT = 0.02


def make_sup(**kw):
    kw.setdefault("arm_after", 0.0)
    return ReflexSupervisor(WaveGait(), **kw)


def drive(sup, seq, righter_calls=None):
    """seq: list of (duration_s, tilt_deg, height). Returns state trace."""
    trace, t = [], 0.0
    for dur, tilt, h in seq:
        for _ in range(int(round(dur / DT))):
            q, st = sup.step(t, 0.0, 0.0, 0.0, 0.2,
                             tilt_deg=tilt, height=h)
            assert not np.isnan(q).any(), "NaN joint target"
            assert q.shape == (N_LEGS, 3)
            trace.append(st)
            t += DT
    return trace


def test_transient_spike_does_not_trip():
    sup = make_sup()
    trace = drive(sup, [(1.0, 5, 0.12), (0.4, 75, 0.05), (1.0, 5, 0.12)])
    assert FALLEN not in trace          # 0.4 s < fall_confirm_s
    assert sup.fall_count == 0


def test_full_fall_recover_cycle():
    calls = []

    def righter(t, dt):
        calls.append(t)
        return np.zeros((N_LEGS, 3))    # "policy" output

    sup = make_sup(righter=righter)
    trace = drive(sup, [
        (0.5, 5, 0.12),                 # walking upright
        (1.5, 90, 0.05),                # on its side, confirmed
        (1.0, 40, 0.07),                # righter working, not upright yet
        (1.0, 10, 0.12),                # handoff criterion met + held
        (2.0, 5, 0.12),                 # ramp + hold complete
    ])
    assert trace[0] == NORMAL
    assert FALLEN in trace and RIGHTED in trace
    assert trace[-1] == NORMAL
    i_f, i_r = trace.index(FALLEN), trace.index(RIGHTED)
    assert i_f < i_r < len(trace) - 1
    assert sup.fall_count == 1
    assert len(calls) > 0               # the righter actually drove


def test_no_tilt_feed_never_falls():
    sup = make_sup()
    t = 0.0
    for _ in range(100):                # tilt_deg=None -> branch disabled
        _, st = sup.step(t, 30.0, 0.0, 0.0, 0.2)
        assert st != FALLEN
        t += DT
    assert sup.fall_count == 0


def test_no_righter_holds_pose_until_deadline():
    sup = make_sup(fallen_max_s=1.0)    # no righter installed
    trace = drive(sup, [(1.5, 90, 0.05), (1.6, 90, 0.05)])
    assert FALLEN in trace and RIGHTED in trace   # deadline forced the ramp


def test_refall_during_ramp_reenters_fallen():
    sup = make_sup(righter=lambda t, dt: np.zeros((N_LEGS, 3)))
    drive(sup, [(1.5, 90, 0.05), (1.0, 10, 0.12)])     # -> RIGHTED
    assert sup.state == RIGHTED
    drive(sup, [(0.1, 80, 0.05)])                      # knocked over mid-ramp
    assert sup.state == FALLEN
    assert sup.fall_count == 2


def test_handoff_criterion_requires_hold():
    sup = make_sup(righter=lambda t, dt: np.zeros((N_LEGS, 3)))
    trace = drive(sup, [
        (1.5, 90, 0.05),
        (0.3, 10, 0.12),                # upright only 0.3 s < handoff_hold_s
        (0.3, 60, 0.05),                # rocks back
        (0.3, 10, 0.12),
    ])
    assert sup.state == FALLEN          # never held long enough
    assert RIGHTED not in trace


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"{len(fns)}/{len(fns)} FALLEN-branch tests green")
