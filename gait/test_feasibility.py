"""D052 motion feasibility — pure Python (MuJoCo only for one optional cross-check).

"I don't want to make a gesture the robot can't do, or have it do things
way quicker than possible." These tests hold every motion source to that:
the code gestures, the keyframe JSON on disk, the gait inside its budget —
and prove the checker catches the pre-D052 versions that broke it.

    python -m pytest gait/test_feasibility.py -q
"""
import json
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rocky_model as rm                                               # noqa: E402
import pebble_feasibility as pf                                        # noqa: E402
from pebble_gait import WaveGait, N_LEGS, leg_ik, body_to_leg, ik_ok   # noqa: E402
from pebble_gestures import (CANON, _planted_q, _lean_dir, _smooth,     # noqa: E402
                             BECKON_T, BECKON_ARM)
from pebble_gestures2 import GESTURES2, kind_of, stability_margin       # noqa: E402
from pebble_keyframes import (KeyframeGesture, GESTURE_DIR, save_keyframe_gesture,  # noqa: E402
                              load_keyframe_gestures, with_tail)
from pebble_manip_adjacent import AdjacentManip                         # noqa: E402
from pebble_reflex import ReflexSupervisor                              # noqa: E402
import pebble_pose_solver as ps                                         # noqa: E402


@pytest.fixture(scope="module")
def g():
    return WaveGait()


# ------------------------------------------------------------------ the library passes
@pytest.mark.parametrize("name", list(CANON) + list(GESTURES2))
def test_every_code_gesture_is_feasible(g, name):
    fn, total = CANON[name] if name in CANON else GESTURES2[name][:2]
    r = pf.check(fn, total, g=g, name=name, kind=kind_of(name))
    assert r.ok, "\n".join(r.lines)
    assert r.nan_count == 0 and not r.jumps
    assert np.all(r.vmax <= pf.speed_limits()["free"] + 1e-9)


def test_every_keyframe_json_on_disk_is_feasible(g):
    files = sorted(f for f in os.listdir(GESTURE_DIR) if f.endswith(".json"))
    assert files, "gait/gestures/ should hold at least the D052 example"
    for fn in files:
        with open(os.path.join(GESTURE_DIR, fn)) as f:
            spec = json.load(f)
        r = pf.check_spec(spec, g=g)
        assert r.ok, f"{fn}:\n" + "\n".join(r.lines)


def test_adjacent_manip_is_feasible_and_starts_planted(g):
    m = AdjacentManip(g)
    q0, _c, _n, planted = m.targets(0.0)
    assert np.allclose(q0, pf.planted_q(g), atol=1e-9)      # D052: flank feet no longer hang 30 mm up
    assert planted.all()
    from pebble_gestures import CLAW_MAX
    from pebble_manip_adjacent import T_TOTAL
    r = pf.check(lambda gg, t: (m.targets(t)[0], m.targets(t)[1] * CLAW_MAX), T_TOTAL, g=g, fs=50)
    assert r.ok, "\n".join(r.lines)


# ------------------------------------------------------------------ the gait + its budget
def _corners(g):
    out = []
    for ang in np.linspace(0, 2 * np.pi, 6, endpoint=False):
        for wz in (-3.0, 0.0, 3.0):
            out.append(g.budget(500 * np.cos(ang), 500 * np.sin(ang), wz))
    out += [g.budget(0, 0, 5.0), g.budget(0, 0, -5.0)]
    return out


def test_gait_passes_at_budgeted_command_box_corners(g):
    for cmd in _corners(g)[::2] + _corners(g)[-2:]:
        r = pf.check_gait(g, cmd, cycles=1.0)
        assert r.ok, f"{cmd}\n" + "\n".join(r.lines)


def test_default_walk_passes_and_budget_leaves_it_alone(g):
    assert g.budget(45.0, 0.0, 0.0) == (45.0, 0.0, 0.0)          # the everyday command is inside
    r = pf.check_gait(g, (45.0, 0.0, 0.0))
    assert r.ok, "\n".join(r.lines)
    assert r.support_min >= 4
    mc = g.max_command()
    assert 40.0 < mc["v"] < 50.0 and 0.2 < mc["wz"] < 0.3       # the envelope SHRANK (D052), on purpose


def test_budget_scales_uniformly(g):
    vx, vy, wz = g.budget(90.0, 30.0, 0.6)
    k = vx / 90.0
    assert k < 1.0 and vy == pytest.approx(30.0 * k) and wz == pytest.approx(0.6 * k)


def test_gait_defaults_come_from_params():
    d = rm.gait_defaults()
    g2 = WaveGait()
    assert (g2.h, g2.R0, g2.T, g2.duty, g2.hstep) == (d["body_height"], d["stance_radius"],
                                                     d["cycle_time"], d["duty"], d["step_height"])
    assert WaveGait(cycle_time=1.6).T == 1.6                     # kwargs still override


def test_duty_075_would_leave_the_stance_polygon():
    """Why D052 kept duty 0.8: in the wave order two ADJACENT legs overlap in swing."""
    r = pf.check_gait(WaveGait(duty=0.75), (45.0, 0.0, 0.0))
    assert "MARGIN" in r.fails and r.margin_min < -40


# ------------------------------------------------------------------ the pre-D052 motions fail
def _legacy_beckon(g, t, hz=0.9):
    """beckon as it was (0.9 Hz, 80 deg curl): 5.92 rad/s of knee."""
    t1 = BECKON_T["lean"]
    t2 = t1 + BECKON_T["raise_"]
    lean = _lean_dir((BECKON_ARM,)) * 13.0
    lean[2] = -10.0
    q_out = np.deg2rad([0.0, 55, -35])
    q_in = np.deg2rad([0.0, 75, -115])
    if t < t1:
        return _planted_q(g, _smooth(t / t1) * lean), np.zeros(N_LEGS)
    q = _planted_q(g, lean)
    if t < t2:
        a = _smooth((t - t1) / (t2 - t1))
        q[0] = (1 - a) * q[0] + a * q_out
        return q, np.zeros(N_LEGS)
    c = 0.5 * (1 - np.cos(2 * np.pi * hz * (t - t2)))
    c = c * c * (3 - 2 * c)
    q[0] = (1 - c) * q_out + c * q_in
    return q, np.zeros(N_LEGS)


def _legacy_fist_bump(g, t):
    """fist_bump as it was: body-frame waypoints IK'd past the hip/knee limits,
    the raise blending from p_nom instead of the leaned foot."""
    lean = _lean_dir((0,)) * 13.0
    lean[2] = -10.0
    carry, extend = np.array([0.0, 205.0, 45.0]), np.array([0.0, 235.0, 55.0])
    if t < 1.0:
        return _planted_q(g, _smooth(t) * lean), np.zeros(N_LEGS)
    q = _planted_q(g, lean)
    if t < 2.0:
        a = _smooth(t - 1.0)
        q[0] = leg_ik(body_to_leg(0, (1 - a) * g.p_nom[0] + a * carry))
    else:
        a = _smooth((t - 2.0) / 0.7)
        q[0] = leg_ik(body_to_leg(0, (1 - a) * carry + a * extend))
    return q, np.zeros(N_LEGS)


def test_pre_d052_beckon_fails_on_speed(g):
    r = pf.check(_legacy_beckon, 5.3, g=g, q_to=False)
    assert "SPEED_HARD" in r.fails and r.vmax[0, 2] > 5.5


def test_pre_d052_fist_bump_fails_on_limits_and_steps(g):
    r = pf.check(_legacy_fist_bump, 4.0, g=g, q_to=False)
    assert "LIMIT_HIP" in r.fails or "LIMIT_KNEE" in r.fails
    assert {"JUMP", "SPEED_HARD"} & set(r.fails)


def test_pre_d052_turn_in_place_fails_on_speed():
    old = WaveGait(cycle_time=1.6, step_height=32.0)

    def turn(gg, t):
        env = min(1.0, t / 0.6, (4.8 - t) / 0.6)
        return gg.joint_targets(t, 0.0, 0.0, 0.35 * env)[0], np.zeros(N_LEGS)
    r = pf.check(turn, 4.8, g=old, kind="gait", q_to=False)
    assert "SPEED_HARD" in r.fails and r.vmax[:, 0].max() > 5.0


def test_pre_d052_gait_fails_at_45(g):
    r = pf.check_gait(WaveGait(cycle_time=1.6, step_height=32.0), (45.0, 0.0, 0.0))
    assert not r.ok and r.vmax[:, 2].max() > 4.5                 # the 4.62 rad/s knee


# ------------------------------------------------------------------ keyframe specs
BAD = {"name": "bad", "keyframes": [
    {"t": 0.0},
    {"t": 0.5, "yaw": 25, "ease": "hold"},                        # coxa 53.6 deg
    {"t": 1.0, "arm": {"1": [0, 100, -160]}, "ease": "hold"}]}    # past hip AND knee


def test_bad_spec_fails_with_the_right_codes(g):
    r = pf.check_spec(BAD, g=g)
    assert not r.ok
    assert {"LIMIT_YAW", "LIMIT_HIP", "LIMIT_KNEE"} <= set(r.fails)
    assert {"SPEED_HARD", "SPEED_FREE"} & set(r.fails)
    worst_yaw = max(abs(v["value_deg"]) for v in r.violations if v["code"] == "LIMIT_YAW")
    assert worst_yaw == pytest.approx(53.6, abs=0.3)             # the studio's yaw-slider finding


def test_unreachable_reach_fails_with_a_clear_message(g):
    spec = {"name": "far", "keyframes": [{"t": 0.0}, {"t": 1.2, "reach": {"0": [0, 400, 50]}}]}
    kg = KeyframeGesture(spec)
    q, _ = kg(g, 1.2)
    assert np.isfinite(q).all()                                   # never a silent NaN
    r = kg.report(g)
    assert "REACH" in r.fails
    assert any("beyond the arm" in ln for ln in r.lines)


def test_reach_lands_the_hand_on_target(g):
    tgt = np.array([30.0, 290.0, 60.0])
    spec = {"name": "r", "keyframes": [{"t": 0.0}, {"t": 1.5, "reach": {"0": tgt.tolist()}}]}
    q, _ = KeyframeGesture(spec)(g, 1.5)
    assert np.linalg.norm(pf.feet_body(q)[0] - tgt) < 0.5


def test_hold_is_rate_limited_not_a_jump(g):
    spec = {"name": "h", "keyframes": [{"t": 0.0}, {"t": 2.0, "arm": {"0": [0, 70, -60]}, "ease": "hold"}]}
    kg = KeyframeGesture(spec)
    r = kg.report(g)
    assert r.ok, "\n".join(r.lines)
    assert not r.jumps
    assert np.allclose(kg(g, 1.9)[0][0], np.deg2rad([0, 70, -60]))   # arrived early, holding


def test_tail_is_appended_unless_end_hold(g):
    spec = {"name": "t", "keyframes": [{"t": 0.0}, {"t": 1.0, "body": [0, 0, -20]}]}
    assert len(with_tail(spec)["keyframes"]) == 3
    assert KeyframeGesture(spec).total > 1.0
    assert len(with_tail(dict(spec, end="hold"))["keyframes"]) == 2
    assert KeyframeGesture(dict(spec, end="hold")).total == pytest.approx(1.0)


def test_loop_must_close(g):
    spec = {"name": "l", "loop": True, "keyframes": [{"t": 0.0}, {"t": 1.0, "body": [8, 0, 0]}]}
    assert "LOOP_WRAP" in pf.check_spec(spec, g=g).fails
    closed = dict(spec, keyframes=spec["keyframes"] + [{"t": 2.0}])
    assert pf.check_spec(closed, g=g).ok


def test_save_refuses_infeasible_unless_forced(tmp_path):
    with pytest.raises(ValueError) as e:
        save_keyframe_gesture(BAD, str(tmp_path))
    assert "LIMIT_YAW" in str(e.value)
    p = save_keyframe_gesture(BAD, str(tmp_path), force=True)
    with open(p) as f:
        assert json.load(f)["checked"]["ok"] is False
    assert "bad" in load_keyframe_gestures(str(tmp_path))


def test_malformed_spec_is_a_value_error():
    with pytest.raises(ValueError):
        KeyframeGesture({"keyframes": [{"t": 1, "arm": {"7": [0, 0, 0]}}]})
    with pytest.raises(ValueError):
        KeyframeGesture({"keyframes": [{"t": 1, "body": [1, 2]}]})
    assert "SPEC" in pf.check_spec({"keyframes": [{"t": 1, "ease": "teleport"}]}).fails


# ------------------------------------------------------------------ whole-body IK + teach
def test_solve_reach_finds_a_pose_with_margin(g):
    r = ps.solve_reach(g, 0, (0.0, 300.0, 70.0), keep_margin_mm=25.0)
    assert r["ok"], r["note"]
    assert r["margin"] >= 25.0
    q = np.array(r["q"])
    assert pf.q_ok(q)
    hand = pf.feet_body(q)[0]
    target_in_body = ps._rotz(np.array([0.0, 300.0, 70.0]), -math.radians(r["yaw"])) - np.array(r["body"])
    assert np.linalg.norm(hand - target_in_body) < 0.5


def test_solve_reach_reports_the_impossible(g):
    r = ps.solve_reach(g, 0, (0.0, 450.0, 0.0))
    assert not r["ok"] and "out of reach" in r["note"]


def test_teach_by_demonstration_round_trip(g):
    fn, total = GESTURES2["bow"][:2]
    fs = 50.0
    ts = np.arange(0, total + 1e-9, 1 / fs)
    Q = np.array([fn(g, t)[0] for t in ts])
    spec = ps.keyframes_from_stream(Q, fs, tol_deg=2.0, name="taught_bow")
    assert len(spec["keyframes"]) < len(Q) / 10                  # RDP keeps few frames
    assert spec["taught"]["fit_residual_mm"] < 1.0
    kg = KeyframeGesture(spec)
    err = max(np.degrees(np.abs(kg(g, t)[0] - Q[k]).max()) for k, t in enumerate(ts))
    assert err < 2.5
    assert kg.report(g).ok


def test_pose_from_joint_state_recovers_body_pose(g):
    from pebble_gestures2 import posed
    q = posed(g, offset=(6.0, -4.0, -12.0), yaw=math.radians(7.0))[0]
    q[2] = np.deg2rad([0, 60, -60])                               # an arm up
    fr = ps.pose_from_joint_state(q, g)
    assert fr["body"] == pytest.approx([6.0, -4.0, -12.0], abs=0.2)
    assert fr["yaw"] == pytest.approx(7.0, abs=0.1)
    assert list(fr["arm"]) == ["2"]


# ------------------------------------------------------------------ margins, reflex, helpers
def test_stability_margin_is_com_aware(g):
    """Two ADJACENT arms up: the old body-origin proxy said positive, the CoM is outside."""
    from pebble_gestures2 import posed
    q = posed(g)[0]
    q[0] = q[1] = np.deg2rad([0, 60, -60])
    assert stability_margin(g, q, arm_legs=(0, 1)) < 0
    com_m, origin_m, _ = pf.margins(q, np.array([False, False, True, True, True]))
    assert com_m < origin_m


def test_righted_ramp_respects_the_loaded_budget(g):
    sup = ReflexSupervisor(g)
    assert sup.right_ramp_s == rm.reflex_defaults()["right_ramp_s"] == 1.0
    q_far = pf.planted_q(g).copy()
    q_far[:, 1] = np.deg2rad(90.0)                                # righter left every hip at +90
    T = sup.ramp_time(q_far)
    peak = 1.5 * np.max(np.abs(q_far - pf.planted_q(g))) / T
    assert peak <= rm.servo_speed("loaded") + 1e-9
    assert ReflexSupervisor(g, right_ramp_s=0.6, stall_s=2.0).right_ramp_s == 0.6   # kwargs win


def test_ik_ok(g):
    assert ik_ok(pf.planted_q(g)) and ik_ok(pf.planted_q(g), guard_deg=2.0)
    assert not ik_ok(np.deg2rad([0, 95, -60]))
    assert not ik_ok(np.array([np.nan, 0, -1]))


def test_com_model_matches_mujoco():
    mujoco = pytest.importorskip("mujoco")
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, "..", "sim", "pebble.xml"))
    d = mujoco.MjData(m)
    torso = m.body("torso").id
    jadr = [m.joint(f"{n}{i}").qposadr[0] for i in range(N_LEGS) for n in rm.LEG_JOINTS]
    assert m.body_subtreemass[torso] * 1000 == pytest.approx(pf.M_TOTAL, abs=0.05)
    rng = np.random.default_rng(3)
    for _ in range(10):
        q = np.column_stack([rng.uniform(-0.6, 0.6, 5), rng.uniform(-1.1, 1.5, 5),
                             rng.uniform(-2.5, -0.4, 5)])
        d.qpos[:] = m.qpos0
        d.qpos[0:3] = [0, 0, 1]
        d.qpos[jadr] = q.ravel()
        mujoco.mj_forward(m, d)
        assert np.allclose((d.subtree_com[torso] - [0, 0, 1]) * 1000, pf.com_body(q), atol=0.01)
