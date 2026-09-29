"""D063 (B76 fixes 1 and 2): the gait moves its joints without steps.

The swing is velocity-matched at lift-off and touchdown (swing_profile), the
lift fades in with the command (lift_scale), and a command change is slewed
(CommandSlew: a rate limit, then a lag) — except a stop, which lands at once.
Each fix is shown against the fault it removes: the pre-D063 sine swing must
FAIL the KINK check, an unslewed start must step a joint far more than the
slewed one does, and the bare rate limit (lag 0) must fail the ramps the lag
exists for.

    python -m pytest gait/test_gait_motion.py -q
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rocky_model as rm                                               # noqa: E402
import pebble_feasibility as pf                                        # noqa: E402
from pebble_gait import (WaveGait, ArmedGait, CommandSlew, N_LEGS,     # noqa: E402
                         swing_profile, swing_xy_peak)

DT = 0.02                     # the bus tick (50 Hz)


@pytest.fixture(scope="module")
def g():
    return WaveGait()


class _SineSwing(WaveGait):
    """The pre-D063 swing: smoothstep xy from rest, h sin(pi s) lift, full lift at any command."""
    _LIFT_CACHE = {}

    def lift_scale(self, vf_max):
        return 1.0

    def _leg_target(self, ph, pn, vf, lift):
        T_st = self.duty * self.T
        if ph < self.duty:
            return pn - vf * ((ph / self.duty - 0.5) * T_st), True
        s = (ph - self.duty) / (1 - self.duty)
        p_lift, p_land = pn - vf * (0.5 * T_st), pn + vf * (0.5 * T_st)
        p = p_lift + (p_land - p_lift) * s * s * (3 - 2 * s)
        p[2] += lift * np.sin(np.pi * s)
        return p, False


def _commands(g):
    return [g.budget(*c) for c in ((45, 0, 0), (0, 45, 0), (0, 0, 0.35), (45, 45, 0.3))]


# ------------------------------------------------------------------ (a) the soft swing
def test_swing_profile_matches_stance_at_both_ends():
    duty = 0.8
    k = (1 - duty) / duty
    e = 1e-6
    for s0, s1 in ((0.0, e), (1.0 - e, 1.0)):
        xy0, z0 = swing_profile(s0, duty)
        xy1, z1 = swing_profile(s1, duty)
        assert (xy1 - xy0) / e == pytest.approx(-k, abs=1e-4)          # the stance foot's own velocity
        assert abs(z1 - z0) / e < 1e-4                                   # lands and leaves at rest
    assert swing_profile(0.0, duty) == (0.0, 0.0) and swing_profile(1.0, duty) == (1.0, 0.0)
    assert swing_profile(0.5, duty)[1] == 1.0                            # the apex is the step height
    s = np.linspace(0, 1, 20001)
    assert np.gradient(swing_profile(s, duty)[0], s).max() == pytest.approx(swing_xy_peak(duty), abs=1e-3)


def test_joint_velocity_is_continuous_at_liftoff_and_touchdown(g):
    for cmd in _commands(g):
        r = pf.check_gait(g, cmd)
        assert r.ok, "\n".join(r.lines)
        assert r.kink_max < 0.05, (cmd, r.kink_max)                     # was 2.7-3.1 rad/s (below)


def test_the_checker_catches_the_old_sine_swing():
    old = _SineSwing()
    r = pf.check_gait(old, (30.0, 0.0, 0.0))
    assert "KINK" in r.fails and r.kink_max > 2.0, "\n".join(r.lines)   # HEAD measured 3.05 at 45 mm/s
    assert {k["where"] for k in r.kinks} == {"lift-off", "touchdown"}


def test_armed_gait_uses_the_soft_swing():
    ag = ArmedGait(arm_legs=(0,))

    def fn(gg, t):
        q, st, _f = gg.joint_targets(t, 15.0, 0.0, 0.0)
        return q, np.zeros(N_LEGS), st
    r = pf.check(fn, 2 * ag.T, g=ag, q_from=False, q_to=False, kind="gait")
    assert np.isfinite(r.kink_max) and r.kink_max < 0.05


# ------------------------------------------------------------------ lift fade-in
def test_zero_command_is_the_planted_stance(g):
    planted = pf.planted_q(g)
    for t in np.linspace(0.0, g.T, 17):
        q, _st, feet = g.joint_targets(t, 0.0, 0.0, 0.0)
        assert np.allclose(feet, g.p_nom, atol=1e-9)                    # it used to march in place
        assert np.allclose(q, planted, atol=1e-9)
    assert g.lift_scale(0.0) == 0.0 and g.lift_scale(g.LIFT_FULL_MM_S) == 1.0
    lifts = [g.lift_scale(v) for v in np.linspace(0, 2 * g.LIFT_FULL_MM_S, 41)]
    assert np.all(np.diff(lifts) >= 0)


def test_lift_ceiling_still_sees_an_unliftable_step():
    """lift_scale fades the lift out near standing; that must not hide a step
    height the servo cannot lift at any speed (the watchdog's ladder relies
    on the ceiling reading 0)."""
    assert WaveGait(step_height=60.0).vf_limit()[2] == 0.0


# ------------------------------------------------------------------ (b) the slewed command
def _ticks(g, target, t0, slew=None, v0=(0.0, 0.0, 0.0), secs=3.0):
    """Worst joint-target change (rad) per 50 Hz tick while the command goes
    v0 -> target; slew=None sends the target at once (the pre-D063 path)."""
    q_prev = g.joint_targets(t0, *v0)[0]
    worst = 0.0
    for k in range(1, int(secs / DT)):
        v = target if slew is None else slew.step(target, DT)
        q = g.joint_targets(t0 + k * DT, *v)[0]
        worst, q_prev = max(worst, float(np.abs(q - q_prev).max())), q
    return worst


def test_stand_to_walk_never_steps_a_joint_past_the_free_budget(g):
    bound = rm.servo_speed("free") * DT                                  # 4.6 deg per tick
    for target in _commands(g):
        for t0 in np.linspace(0.0, g.T, 5, endpoint=False):
            slewed = _ticks(g, target, t0, CommandSlew(g))
            assert slewed <= bound, (target, t0, np.rad2deg(slewed))
    unslewed = max(_ticks(g, g.budget(45, 0, 0), t0) for t0 in np.linspace(0.0, g.T, 10, endpoint=False))
    assert np.rad2deg(unslewed) > 15.0                                   # the snap B76 names (28.4 at HEAD)


def _env(g, deg):
    a = np.deg2rad(deg)
    return g.budget(100 * np.cos(a), 100 * np.sin(a), 0.0)


@pytest.mark.parametrize("case", ["start", "reverse", "walk_to_turn"])
def test_slewed_ramps_pass_the_checker(g, case):
    walk, turn = g.budget(45, 0, 0), g.budget(0, 0, 0.35)
    src, dst = {"start": ((0.0, 0.0, 0.0), walk), "reverse": (walk, g.budget(-45, 0, 0)),
                "walk_to_turn": (walk, turn)}[case]
    for t0 in (0.0, 0.5, 1.0, 1.5):
        r = pf.check_ramp(g, dst, cmd_from=src, t0=t0)
        assert r.ok and r.kink_max < 0.05, (case, t0, r.lines)


# ramps that end (or start) at the envelope just as a leg lifts off: the bare
# rate limit put them over the loaded 3.0 (measured 3.030 / 3.005 / 3.004)
_EDGE_RAMPS = {"start 18 deg": (None, 18, 0.54), "start 18 deg, later phase": (None, 18, 0.583),
               "reversal at 54 deg": (54, 234, 0.417)}


@pytest.mark.parametrize("case", sorted(_EDGE_RAMPS))
def test_the_lag_keeps_edge_ramps_under_the_loaded_budget(g, case):
    a, b, t0 = _EDGE_RAMPS[case]
    src, dst = (0.0, 0.0, 0.0) if a is None else _env(g, a), _env(g, b)
    bare = pf.check_ramp(g, dst, cmd_from=src, t0=t0, lag=0.0)
    assert "SPEED_LOADED" in bare.fails, "\n".join(bare.lines)          # the fault the lag removes
    r = pf.check_ramp(g, dst, cmd_from=src, t0=t0)
    assert r.ok, "\n".join(r.lines)
    assert r.peak()[0] < 2.99                                            # measured 2.977-2.981


def test_the_slewed_command_never_steps(g):
    """The command's rate changes smoothly all the way to rest: no step at the
    ramp's ends or where the lag's tail finishes (a 0.1 mm/s snap there put
    one 100 Hz sample 8 mm/s off and a ramp at 3.005 rad/s)."""
    dt = 0.01
    for src, dst in (((0.0, 0.0, 0.0), _env(g, 18)), (_env(g, 54), _env(g, 234)), (_env(g, 30), (0.0, 0.0, 0.0))):
        sl = CommandSlew(g)
        sl.hold(src)
        vs = [np.array(src)] + [np.array(sl.step(dst, dt)) for _ in range(400)]
        assert np.allclose(vs[-1], dst, atol=0.0)                         # it ends, exactly
        rate = [g._vf_max(*(b - a)) for a, b in zip(vs, vs[1:])]
        assert max(rate) <= g.CMD_ACCEL_MM_S2 * dt + 1e-9
        jerk = [g._vf_max(*(c - 2 * b + a)) for a, b, c in zip(vs, vs[1:], vs[2:])]
        assert max(jerk) < 0.03, max(jerk)                                # mm/s per tick^2; 0.0125 at the ramp's start


def test_one_slew_step_equals_many(g):
    """pebble_feasibility.ramp_fn judges a ramp with ONE step of t: step()
    must integrate the rate limit + lag exactly for a held target."""
    target = np.array(_env(g, 40))
    for t in (0.37, 1.0, 1.9, 3.3):
        one, many = CommandSlew(g), CommandSlew(g)
        one.step(target, t)
        n = int(round(t / 0.001))
        for _ in range(n):
            many.step(target, t / n)
        assert np.allclose(one.v, many.v, atol=1e-9) and np.allclose(one.u, many.u, atol=1e-9)


def test_slew_keeps_the_heading_and_the_envelope(g):
    sl = CommandSlew(g)
    target = np.array(g.budget(40.0, 25.0, 0.1))
    mc = g.max_command()["vf"]
    for _ in range(200):
        v = np.array(sl.step(target, DT))
        assert np.allclose(np.cross(v, target), 0.0, atol=1e-9)          # on the line to the target
        assert g._vf_max(*v) <= mc + 1e-9
    assert np.allclose(v, target)


# ------------------------------------------------------------------ (c) a stop is not slewed
def test_a_stop_lands_within_one_tick(g):
    sl = CommandSlew(g)
    walk = g.budget(45, 0, 0)
    for _ in range(200):
        sl.step(walk, DT)
    assert np.allclose(sl.v, walk)
    assert sl.stop() == (0.0, 0.0, 0.0)                                  # zero NOW
    assert sl.step((0.0, 0.0, 0.0), DT) == (0.0, 0.0, 0.0)
    q, _st, _f = g.joint_targets(1.234, *sl.v)
    assert np.allclose(q, pf.planted_q(g), atol=1e-9)                   # and its target is the stance
    sl.step(walk, DT, direct=True)                                       # the void retreat's replay
    assert np.allclose(sl.v, walk)
    assert sl.step((np.nan, 0.0, 0.0), DT) == (0.0, 0.0, 0.0)            # a bad command is a stop
    # a zero TARGET is not a stop: it slows down (the lag eases the first tick
    # in, so it moves less than the rate limit's accel x DT) ...
    sl.step(walk, DT, direct=True)
    v = sl.step((0.0, 0.0, 0.0), DT)
    assert 0.0 < walk[0] - v[0] < g.CMD_ACCEL_MM_S2 * DT
    # ... and its tail ENDS at exactly zero (idle checks read np.any(v))
    for k in range(1, 200):
        if sl.step((0.0, 0.0, 0.0), DT) == (0.0, 0.0, 0.0):
            break
    assert k * DT < walk[0] / g.CMD_ACCEL_MM_S2 + 6 * g.CMD_LAG_S          # 2.36 s from 34.2 mm/s
