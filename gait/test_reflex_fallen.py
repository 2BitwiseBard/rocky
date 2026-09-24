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
    assert sup.right_reason == "handoff"
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


def test_stalled_righter_ramps_before_deadline():
    """D048: no progress on the best tilt for stall_s -> ramp now (the
    planted ramp rights the robot from its back; the policy does not)."""
    sup = make_sup(righter=lambda t, dt: np.zeros((N_LEGS, 3)),
                   fallen_max_s=10.0, stall_s=2.0)
    trace = drive(sup, [(1.0, 170, 0.04),        # on its back: FALLEN at 1.0 s
                        (1.9, 170, 0.04),        # righter flails, tilt never moves
                        (0.4, 170, 0.04)])
    n_before = int(round((1.0 + 1.9) / DT))
    assert RIGHTED not in trace[:n_before]      # not before FALLEN + stall_s
    assert RIGHTED in trace                     # and well before the 10 s deadline
    assert sup.right_reason == "stall"


def test_improving_righter_is_not_stalled():
    sup = make_sup(righter=lambda t, dt: np.zeros((N_LEGS, 3)), stall_s=2.0)
    seq = [(1.5, 170, 0.04)] + [(1.5, 170 - 15 * k, 0.05) for k in range(1, 6)]
    trace = drive(sup, seq)                     # 15 deg better every 1.5 s
    assert RIGHTED not in trace                 # progress -> keep the righter
    assert sup.state == FALLEN


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"  ok  {fn.__name__}")
    print(f"{len(fns)}/{len(fns)} FALLEN-branch tests green")


def test_brace_holds_feet_when_airborne():
    """D048: with no foot in contact the tilt-vector seek has nothing to
    catch and its goal flips sign every step; BRACE must hold the foot
    targets until a foot lands, then resume the crouch."""
    sup = make_sup()
    sup.request_stop()
    t, st = 0.0, NORMAL
    for _ in range(int(1.0 / DT)):                      # stop request -> BRACE
        q, st = sup.step(t, 0.0, 0.0, 0.0, 0.2, contacts=[True] * 5,
                         gyro_vec=np.array([0.0, 0.0]))
        t += DT
        if st == "BRACE":
            break
    assert st == "BRACE"
    z0 = sup._last_feet[:, 2].copy()
    for _ in range(10):                                  # airborne: gyro spinning, zero contacts
        q, st = sup.step(t, 0.0, 0.0, 0.0, 6.0, contacts=[False] * 5,
                         gyro_vec=np.array([4.0, -3.0]))
        assert not np.isnan(q).any()
        t += DT
    assert st == "BRACE"
    assert np.allclose(sup._last_feet[:, 2], z0), "targets moved while airborne"
    for _ in range(10):                                  # feet back down: crouch resumes
        q, st = sup.step(t, 0.0, 0.0, 0.0, 0.2, contacts=[True] * 5,
                         gyro_vec=np.array([0.0, 0.0]))
        t += DT
    assert not np.allclose(sup._last_feet[:, 2], z0), "crouch did not resume"


def test_handoff_uses_the_joint_state_when_q_meas_is_fed():
    """D052 V2 (review): the robot and the cockpit pass q_meas, so the handoff is
    rocky_recover_env.handoff_ok (tilt + switches + the joints' kinematic height)
    — a robot kneeling on its shins passes the old world-z rule, and a mutation
    that disabled handoff_ok left the whole suite green."""
    import pebble_reflex
    assert pebble_reflex._handoff_fn() is not None, "handoff_ok must be importable from gait/"
    folded = np.tile(np.radians([0.0, 10.0, -150.0]), (N_LEGS, 1))   # deck on its shins, feet tucked
    con = np.ones(N_LEGS, bool)
    for q_meas, want_handoff in ((folded, False), (None, True)):
        sup = make_sup(stall_s=10.0, fallen_max_s=10.0)
        q_meas = sup._planted_q() if q_meas is None else q_meas
        sup._enter_fallen(0.0)
        t, st = 0.0, FALLEN
        while st == FALLEN and t < 3.0:
            _q, st = sup.step(t, 0.0, 0.0, 0.0, 0.1, contacts=con, tilt_deg=5.0, height=0.12,
                              q_meas=q_meas)
            t += DT
        if want_handoff:
            assert st == RIGHTED and sup.right_reason == "handoff"
            assert t >= sup.handoff_hold_s - 1e-9
        else:
            assert st == FALLEN and sup.right_reason != "handoff"   # height 0.12 m alone no longer passes
