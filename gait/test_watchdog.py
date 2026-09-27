"""The D023 stuck watchdog under the D052 servo budget: every retry stage can
still walk, every command it returns fits the gait's budget, and a stage switch
does not jump the gait phase.

    python -m pytest gait/test_watchdog.py -q
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pebble_feasibility as pf                                        # noqa: E402
from pebble_gait import WaveGait, STATION_DEG                          # noqa: E402
from pebble_watchdog import ProgressWatchdog, RetryPolicy              # noqa: E402


@pytest.fixture(scope="module")
def stages():
    return RetryPolicy.derive_stages(WaveGait())


def test_every_stage_has_a_speed_envelope(stages):
    g0 = WaveGait()
    assert [s["name"] for s in stages] == [r[0] for r in RetryPolicy.LADDER]
    assert stages[0]["hstep"] == g0.hstep and stages[0]["T"] == g0.T
    for s in stages:
        g = WaveGait(body_height=g0.h + s["dh"], cycle_time=s["T"], step_height=s["hstep"])
        v = g.vf_limit()
        assert v[2] > 0 and min(v) >= RetryPolicy.MIN_VF_MM_S, s
    assert stages[1]["hstep"] > stages[0]["hstep"] and stages[2]["hstep"] > stages[1]["hstep"]


def test_policy_commands_fit_the_budget_and_pass_the_checker():
    g = WaveGait()
    pol = RetryPolicy(g)
    for stage in range(len(pol.STAGES)):
        if stage:
            pol.on_stuck(10.0 * stage)
        cmd = pol.command(10.0 * stage + 0.1, 45.0, 0.0, 0.0)
        assert np.allclose(g.budget(*cmd), cmd)                  # already inside the envelope
        assert np.hypot(cmd[0], cmd[1]) > 1.0                    # and it still moves
        # the checker on the stage's gait (fresh phase: a shifted phase puts the
        # 1e-16 knife edge of two legs' swing flags on a sample instant)
        s = pol.STAGES[stage]
        fresh = WaveGait(body_height=pol.h0 + s["dh"], cycle_time=s["T"], step_height=s["hstep"])
        rep = pf.check_gait(fresh, cmd)
        assert rep.ok, (pol.stage_name, rep.lines[:2])


def test_stage_switch_keeps_the_gait_phase_and_the_stations():
    g = WaveGait()
    xy0 = g.p_nom[:, :2].copy()
    pol = RetryPolicy(g)
    t = 7.3
    before = g.foot_targets(t, 20.0, 0.0, 0.0)[0]
    pol.on_stuck(t)                                              # T 2.0 -> a longer cycle
    assert g.T > 2.0 - 1e-9 and g.hstep > pol.STAGES[0]["hstep"]
    after = g.foot_targets(t, 20.0, 0.0, 0.0)[0]
    # same phase: only the stride length (T) and body lift may move a foot, never a
    # jump from stance to mid-swing
    assert np.abs(after[:, :2] - before[:, :2]).max() < 25.0
    assert np.allclose(g.p_nom[:, :2], xy0)                      # stations untouched
    a = np.deg2rad(STATION_DEG)
    assert np.allclose(np.arctan2(g.p_nom[:, 1], g.p_nom[:, 0]), np.arctan2(np.sin(a), np.cos(a)))
    pol.maybe_relax(t + 100.0, True)
    assert pol.stage == 0 and g.T == pol.STAGES[0]["T"] and g.h == pol.h0


def test_watchdog_fires_on_a_stall_and_not_on_progress():
    dog = ProgressWatchdog(2.0)
    fired = [dog.update(0.02 * k, (0.9 * 40.0 * 0.02 * k, 0.0), 40.0) for k in range(600)]
    assert not any(fired)
    dog = ProgressWatchdog(2.0)
    fired = [dog.update(0.02 * k, (0.0, 0.0), 40.0) for k in range(600)]
    assert any(fired)
