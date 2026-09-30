"""The experiments' limits follow the gait envelope (B111), and the cliff
safe-stop's verdict can pass and fail without a march in place (B112).

    .venv/bin/python -m pytest sim/tests/test_experiment_limits.py -q
"""
import copy
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "experiments"))
import exp_paths  # noqa: E402,F401  (sys.path: sim/, gait/, perception/, audio/)
import scenes  # noqa: E402
from pebble_gait import WaveGait  # noqa: E402


# ------------------------------------------------------------------ B111
def test_the_rubble_walk_commands_the_distance_it_was_sized_for():
    import run_stuck
    ask = scenes.walk_ask(WaveGait())[0]
    assert run_stuck.V_X == pytest.approx(ask)
    # 25 s at the 45 mm/s ask: the commanded distance stays put when the envelope moves
    assert run_stuck.V_X * run_stuck.T_WALK == pytest.approx(25.0 * scenes.V_X)


def test_the_fairing_runs_run_stucks_trials():
    import run_fairing
    import run_stuck
    assert run_fairing.run_stuck is run_stuck      # the same trial, so the same T_WALK


def test_the_lidar_lap_is_inside_the_envelope_on_the_same_ring():
    import sim_lidar
    g = WaveGait()
    vx, wz, t_lap = sim_lidar.lap_command(g)
    assert g.budget(vx, 0.0, wz) == pytest.approx((vx, 0.0, wz))     # already fits
    assert vx < scenes.V_X                                          # the raw ask did not
    assert vx / wz / 1000.0 == pytest.approx(sim_lidar.LAP_R_M)     # the ring keeps its radius
    assert vx * t_lap == pytest.approx(scenes.V_X * sim_lidar.T_LAP_AT_ASK)   # the same arc


# ------------------------------------------------------------------ B112
_PASS = dict(
    fell=False, voids=[[6.3, 2]], stop_margin_mm=228.8,
    end=dict(feet_in_contact=5, foot_margin_mm=46.2),
    post_halt=dict(contact_breaks=0, mean_speed_mms=11.9, still_speed_mms=0.0,
                   max_tilt_deg=0.48),
    supervisor_states=["BRACE", "RECOVER", "NORMAL"],
)


def _failed(r, supervisor=True):
    import run_cliff_safestop as cs
    return sorted(k for k, (ok, _m) in cs.judge(r, supervisor).items() if not ok)


def _with(path, value):
    r = copy.deepcopy(_PASS)
    d = r
    for k in path[:-1]:
        d = d[k]
    d[path[-1]] = value
    return r


def test_the_safe_stop_verdict_passes_a_clean_stop():
    assert _failed(_PASS) == []
    assert _failed(_PASS, supervisor=False) == []               # the bare halt: no `delivered`


@pytest.mark.parametrize("path, value, check", [
    (("post_halt", "contact_breaks"), 20, "planted"),          # the pre-D063 march in place
    (("end", "feet_in_contact"), 4, "on_the_deck"),            # a foot left over the void
    (("end", "foot_margin_mm"), -11.5, "on_the_deck"),
    (("post_halt", "still_speed_mms"), 1.3, "still"),          # creeping after the stop
    (("supervisor_states",), ["RECOVER", "NORMAL"], "delivered"),    # never braced
    (("supervisor_states",), ["BRACE", "RECOVER"], "delivered"),     # never back to idle
    (("voids",), [], "detector"),
])
def test_each_check_can_fail_the_safe_stop(path, value, check):
    assert _failed(_with(path, value)) == [check]


def test_a_fall_or_an_unfinished_window_fails():
    assert "upright" in _failed(_with(("fell",), True))
    assert "upright" in _failed(_with(("stop_margin_mm",), None))


@pytest.mark.slow
def test_the_safe_stop_passes_in_physics_and_the_no_retreat_control_fails():
    import run_cliff_safestop as cs
    assert _failed(cs.run("safestop")) == []
    bad = _failed(cs.run("safestop", retreat_cycles=0.0))
    assert "on_the_deck" in bad                                 # halted with a foot over the void
