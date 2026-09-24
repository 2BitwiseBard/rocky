"""D052 playground guards — the always-on control loop, headless, no rendering.

Each test drives the real Playground (MuJoCo physics, reflex supervisor,
servo realism on) through one guard: gesture entry/exit blends and the
reflex monitor during a gesture, the void guard on plain walking and
teleop, the per-stance probe, the IMU gravity, the NaN guard and the trip
escalation latch. The righter is switched off (torch-free, deterministic)."""
import os
import sys
import warnings

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
for sub in ("sim", "gait", "perception"):
    sys.path.insert(0, os.path.join(ROOT, sub))

import mujoco                                                     # noqa: E402
import rocky_model as rm                                          # noqa: E402
from pebble_gait import WaveGait, N_LEGS                          # noqa: E402
from pebble_reflex import ReflexSupervisor, NORMAL, BRACE         # noqa: E402


def make(**kw):
    from playground import Playground
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        pg = Playground(**kw)
    pg.do("righter off")
    return pg


def run(pg, seconds, every=None):
    for _ in range(int(round(seconds / pg.DT))):
        pg.step()
        if every is not None:
            every(pg)


@pytest.fixture(scope="module")
def settled():
    """A planted robot 1 s after spawn (reused read-only where possible)."""
    pg = make()
    run(pg, 1.0)
    return pg


# ------------------------------------------------------------------ gestures
def test_gesture_refused_while_walking_and_braced():
    pg = make()
    run(pg, 1.0)
    assert pg.do("walk 30").startswith("walking")
    run(pg, 0.5)
    r = pg.do("gesture wave")
    assert "busy walking" in r and pg.gesture is None
    pg.do("stop")
    seen = set()
    for _ in range(int(2.0 / pg.DT)):
        pg.step()
        seen.add(pg.sup.state)
        if pg.sup.state == BRACE:
            r = pg.do("gesture wave")
            assert "BRACE" in r and pg.gesture is None
            break
    assert BRACE in seen


def test_gesture_blends_in_and_out_under_the_loaded_speed():
    """A far pose (hips +0.6 rad from the stance) held like the studio preview:
    no step of the joint targets exceeds the loaded budget, entry or exit."""
    pg = make()
    run(pg, 1.0)
    q_far = pg.sup._planted_q().copy()
    q_far[:, 1] += 0.6
    hold = (lambda g, t: (q_far.ravel(), np.zeros(N_LEGS)))
    prev = pg._q_cmd.copy()
    assert pg.start_gesture(hold, float("inf"), "far") is None
    pg.step()
    first = pg._q_cmd.copy()
    lim = rm.servo_speed("loaded") * pg.DT * 1.02
    assert np.abs(first - prev).max() <= lim           # the first target after the start
    steps = []

    def rec(p):
        steps.append(np.abs(p._q_cmd - rec.prev).max())
        rec.prev = p._q_cmd.copy()
    rec.prev = first
    run(pg, 1.5, rec)
    assert pg.gesture_phase == "run"
    assert np.abs(pg._q_cmd - q_far.ravel()).max() < 1e-3
    pg.end_gesture()                                    # = the studio's preview_off
    run(pg, 1.5, rec)
    assert pg.gesture_phase is None and not pg.gesture_busy
    assert max(steps) <= lim, max(steps) / pg.DT
    assert np.abs(pg._q_cmd - pg.sup._planted_q().ravel()).max() < 1e-3
    assert pg.sup.state == NORMAL


def test_fall_during_gesture_ends_in_fallen():
    pg = make()
    run(pg, 1.0)
    assert pg.do("gesture wave").startswith("wave")
    run(pg, 1.0)
    assert pg.gesture_phase == "run"
    pg.do("push 60 0 0.4")
    states = set()
    run(pg, 3.0, lambda p: states.add(p.sup.state))
    assert "FALLEN" in states
    assert pg.gesture is None and pg.gesture_phase is None
    assert any("aborted" in n[2] for n in pg.notes)


# ------------------------------------------------------------------ void guard
@pytest.mark.parametrize("how", ["walk", "teleop"])
def test_void_guard_stops_a_plain_walk_at_the_cliff(how):
    pg = make(cliff=True)
    run(pg, 1.0)
    if how == "walk":
        pg.do("walk 45")
    else:
        for _ in range(3):
            pg.teleop("up")
    assert pg.cmd_v[0] == pytest.approx(45.0)
    log = []
    run(pg, 14.0, lambda p: log.append((p.t, p.sup.state, p.probe_out.any(), p.cmd_eff.copy(),
                                        p.void is not None, p.last["height"])))
    t_out = next(t for t, _s, o, *_ in log if o)
    fired = next(t for t, _s, _o, _v, vd, _h in log if vd)
    assert fired - t_out < 0.5                          # the probe-out is acted on at once
    # from the step after the fire on, nothing is commanded toward the void
    u = np.radians(pg.void["world_bearing_deg"])
    u = np.array([np.cos(u), np.sin(u)])                # yaw stays ~0 here
    assert all(float(np.dot(v[:2], u)) <= 1.0 for t, _s, _o, v, *_ in log if t > fired + 1e-9)
    # and it settles: NORMAL, no command, within ~2 s of the first probe-out (measured 1.98 s), and stays
    late = [(s, v) for t, s, _o, v, *_ in log if t >= t_out + 2.2]
    assert late and all(s == NORMAL and not np.any(v) for s, v in late)
    assert pg.sup.trip_count <= 2 and pg.sup.fall_count == 0
    assert min(h for *_, h in log) > 0.16 + 0.08       # still on the platform
    # further nudges toward the void are refused; away is fine; `clear` releases
    assert pg.do("walk 45").startswith("blocked: void")
    assert pg.teleop("up").startswith("[teleop] blocked: void")
    assert pg.do("walk -30").startswith("walking")
    pg.do("stop")
    run(pg, 1.0)
    assert "void guard" in pg.do("clear")
    assert pg.void is None
    if how == "walk":
        # V2 (review finding: the first step AWAY after `clear` re-tripped — not
        # reproduced on this tree, pinned here so it stays that way)
        x0 = pg.data.xpos[pg.torso][0]
        assert pg.do("walk -30").startswith("walking")
        run(pg, 3.0)
        assert pg.void is None and pg.data.xpos[pg.torso][0] - x0 < -0.05


@pytest.mark.parametrize("approach_deg", [0, 10, 20, 30, 45])
def test_void_guard_holds_at_every_approach_angle(approach_deg):
    """D052 P2 (review V2): approached at 10-20 deg the robot walked off the
    cliff — the two leading legs (3 at -54, 4 at +18 deg) straddle the edge, one
    lands in the void and the other lifts at the same instant, and the three
    rear feet cannot hold the CoM. Before the fix: 10 deg fired and still fell,
    15 deg never fired, 20 deg fired late on the wrong leg after falling; 0 / 30 /
    45 deg stopped. Now the touchdown gate holds the gait on the late foot, the
    probe finds the void and the retreat plays the approach backwards."""
    pg = make(cliff=True)
    a = np.radians(approach_deg) / 2
    pg.data.qpos[3:7] = [np.cos(a), 0, 0, np.sin(a)]    # yaw the spawn: walk 45 = the approach
    mujoco.mj_forward(pg.model, pg.data)
    run(pg, 1.0)
    assert pg.do("walk 45").startswith("walking")
    log = []
    run(pg, 10.0, lambda p: log.append((p.t, p.sup.state, bool(p.probe_out.any()), p.cmd_eff.copy(),
                                        p.void is not None, float(p.data.xpos[p.torso][0]),
                                        float(p.data.xpos[p.torso][2]))))
    xs = [r[5] for r in log]
    assert max(xs) < 0.33, max(xs)                      # the edge is at x = 0.35
    assert pg.void is not None and pg.sup.fall_count == 0
    assert min(r[6] for r in log) > 0.16 + 0.08         # still standing on the platform
    t_out = next(r[0] for r in log if r[2])
    i_fire = next(k for k, r in enumerate(log) if r[4])
    assert log[i_fire][0] - t_out < 0.5
    assert xs[-1] < xs[i_fire] - 0.02                    # it backed off (the approach, played backwards)
    u = np.radians(pg.void["world_bearing_deg"] - np.degrees(pg.yaw()))
    u = np.array([np.cos(u), np.sin(u)])                # body frame (yaw is held through the stop)
    assert all(float(np.dot(r[3][:2], u)) <= 1.0 for r in log[i_fire + 1:])
    late = [(s, v) for t, s, _o, v, *_ in log if t >= t_out + 2.2]
    assert late and all(s == NORMAL and not np.any(v) for s, v in late)
    assert pg.do("walk 45").startswith("blocked: void")


_LIP = ("KNOWN GAP (review round 3, 2026-09-24): a leading foot lands on the edge's lip "
        "(its sphere centre 0-8 mm past it); the late-foot gate reacts 140 ms after the next "
        "touchdown and the rewind cannot put the lifted leg back before the tip. 7 of 310 "
        "approaches fall (1 deg grid, walk 15/25/35/45, -30..60 deg); the careful walk "
        "(gate.wait 1) falls in 9 of 310 at other angles")


def _approach_falls(speed, approach_deg, seconds=14.0):
    pg = make(cliff=True)
    a = np.radians(approach_deg) / 2
    pg.data.qpos[3:7] = [np.cos(a), 0, 0, np.sin(a)]
    mujoco.mj_forward(pg.model, pg.data)
    run(pg, 1.0)
    pg.do(f"walk {speed}")
    zmin = 9.0
    for _ in range(int(seconds / pg.DT)):
        pg.step()
        zmin = min(zmin, float(pg.data.xpos[pg.torso][2]))
        if pg.sup.fall_count or zmin < 0.24 or pg.void is not None:
            break
    return pg.sup.fall_count > 0 or zmin < 0.24 or pg.void is None


@pytest.mark.parametrize("speed,approach_deg", [
    (45, 13), (25, 15),
    pytest.param(45, 12.5, marks=pytest.mark.slow), pytest.param(45, 14, marks=pytest.mark.slow),
    pytest.param(35, 20, marks=pytest.mark.slow)])
@pytest.mark.xfail(strict=True, reason=_LIP)
def test_void_guard_lip_band_known_gap(speed, approach_deg):
    """The review's counter-examples to "the void guard stops at every approach
    angle": deterministic falls where the grid of the test above steps over
    them. Strict xfail: when a fix lands these must start passing."""
    assert not _approach_falls(speed, approach_deg)


def test_careful_walk_holds_at_every_touchdown_and_saves_a_lip_foothold():
    """`set gate.wait 1`: every touchdown waits for its switch. Measured case:
    walk 25 at 15 deg puts leg 4 on the edge's lip (sphere centre 2-5 mm past
    it) while leg 3 lands in the void — the late-foot gate alone lets leg 4
    unload for 140 ms and it slides off (the robot falls); the careful walk
    never lifts it."""
    pg = make(cliff=True)
    assert "careful walk" in pg.do("set gate.wait 1") and pg.gate_wait
    a = np.radians(15) / 2
    pg.data.qpos[3:7] = [np.cos(a), 0, 0, np.sin(a)]
    mujoco.mj_forward(pg.model, pg.data)
    run(pg, 1.0)
    pg.do("walk 25")
    xs = []
    run(pg, 14.0, lambda p: xs.append(float(p.data.xpos[p.torso][0])))
    assert pg.void is not None and pg.sup.fall_count == 0 and max(xs) < 0.33
    assert pg.gate_count >= 10                          # it held at (nearly) every step
    assert pg.guard_status()["gate_wait"] is True
    assert "late-foot gate only" in pg.do("set gate.wait 0") and not pg.gate_wait


def test_touchdown_gate_releases_on_a_small_step_down():
    """A foot that lands 12 mm low is late: the gate may hold the gait while it
    probes, but it finds the floor, the gait resumes and no void is declared."""
    model, drop = _step_down_model()
    pg = make(model=model, z0=drop)
    run(pg, 1.0)
    x0 = float(pg.data.xpos[pg.torso][1])
    pg.do("walk 0 30")                                  # sideways, off the step toward leg 0 (+y)
    run(pg, 8.0)
    assert pg.gate_count >= 1                           # measured: 6 holds in 8 s
    assert pg.void is None and pg._gate is None
    assert pg.sup.fall_count == 0
    assert float(pg.data.xpos[pg.torso][1]) - x0 > 0.12  # it kept walking


# ------------------------------------------------------------------ probe
def _step_down_model(drop_m=0.012):
    """pebble.xml on a 12 mm platform that leaves leg 0's foot (+y) over the floor."""
    with open(os.path.join(ROOT, "sim", "pebble.xml")) as f:
        xml = f.read()
    box = (f'<geom name="step" type="box" size="0.5 0.315 {drop_m / 2}" pos="0 -0.185 {drop_m / 2}" '
           f'friction="0.8 0.005 0.0001" rgba="0.5 0.5 0.6 1"/>')
    line = next(ln for ln in xml.splitlines() if 'name="floor"' in ln)
    return mujoco.MjModel.from_xml_string(xml.replace(line, line + "\n    " + box)), drop_m


def test_probe_seeks_preloads_and_holds_without_chatter():
    model, drop = _step_down_model()
    pg = make(model=model, z0=drop)
    hist = []
    run(pg, 2.0, lambda p: hist.append((p.probe_dz.copy(), list(p.probe_state))))
    dz0 = np.array([h[0][0] for h in hist])
    assert np.all(np.diff(dz0) >= -1e-9)                # monotone: never retracts while planted
    assert pg.probe_state[0] == "HOLD"
    assert drop * 1000 - 2 <= pg.probe_dz[0] <= drop * 1000 + 6   # found the floor, then preloaded
    assert pg.last["con"][0]                            # and the foot is really down
    from contacts import foot_forces
    assert foot_forces(pg.model, pg.data, pg.fids)[0] > 2.0     # carrying load, not grazing
    # walking: within every stance the offset only grows (or holds); it only
    # shrinks through SWING
    pg.do("walk 25")
    rows = []
    run(pg, 6.0, lambda p: rows.append((p.probe_dz.copy(), list(p.probe_state))))
    for i in range(N_LEGS):
        for (a, sa), (b, sb) in zip(rows, rows[1:]):
            if sa[i] != "SWING" and sb[i] != "SWING":
                assert b[i] >= a[i] - 1e-9, (i, a[i], b[i])


def test_flat_ground_walk_never_probes(settled):
    pg = make()
    run(pg, 1.0)
    pg.do("walk 45")
    peak = np.zeros(N_LEGS)

    def rec(p):
        np.maximum(peak, p.probe_dz, out=peak)
    run(pg, 4.0, rec)
    assert peak.max() < 1e-6, peak
    assert pg.gate_count == 0                           # and the touchdown gate never holds the gait


# ------------------------------------------------------------------ IMU, NaN, latch
def test_tilted_gravity_tilts_the_supervisors_gravity():
    pg = make()
    a = np.radians(10.0)
    pg.model.opt.gravity[:] = 9.81 * np.array([np.sin(a), 0.0, -np.cos(a)])
    pg.step()
    assert pg.last["tilt"] == pytest.approx(10.0, abs=0.3)
    assert pg.last["grav"][0] == pytest.approx(np.sin(a), abs=5e-3)


def test_kinematic_height_is_what_the_supervisor_gets(settled):
    pg = settled
    assert pg.last["kin_h"] == pytest.approx(pg.last["height"], abs=0.004)


def test_nan_residual_never_reaches_ctrl():
    pg = make()
    run(pg, 0.5)
    good = pg.data.ctrl[:15].copy()
    pg.residual = np.full(15, np.nan)
    run(pg, 0.3)
    assert np.isfinite(pg.data.ctrl).all()
    assert pg.nan_count > 0
    assert np.abs(pg.data.ctrl[:15] - good).max() < 0.05
    assert any(k == "nan" for _t, k, _m in pg.notes)


def test_three_trips_latch_a_safe_stop_until_clear():
    pg = make()
    run(pg, 1.0)
    pg.do("walk 45")
    pg.sup.gyro_trip = 0.3                    # every step of the walk now "trips"
    for _ in range(int(6.0 / pg.DT)):
        pg.step()
        if pg.sup.latched:
            break
    assert pg.sup.latched and len(pg.sup._trip_times) >= 3
    pg.sup.gyro_trip = 1.8
    run(pg, 2.0)
    assert not np.any(pg.cmd_v) and not np.any(pg.cmd_eff)
    assert pg.do("walk 30").startswith("blocked: latched")
    assert "latched safe-stop" in pg.do("clear")
    assert pg.do("walk 30").startswith("walking")
    run(pg, 0.5)
    assert pg.cmd_eff[0] > 0


def test_supervisor_latch_unit():
    """Pure supervisor: 3 gyro trips inside 5 s latch; velocity is then ignored."""
    sup = ReflexSupervisor(WaveGait(), arm_after=0.0)
    t, dt = 0.0, 0.02
    while not sup.latched and t < 10.0:
        spike = (int(t / dt) % 60) == 0              # a spike every 1.2 s
        _q, st = sup.step(t, 30.0, 0.0, 0.0, 3.0 if spike else 0.1)
        t += dt
    assert sup.latched and t < 5.0
    for _ in range(200):                             # velocity ignored: settles planted
        _q, st = sup.step(t, 30.0, 0.0, 0.0, 0.1)
        t += dt
    assert st == NORMAL
    tg = sup.t_gait
    sup.step(t, 30.0, 0.0, 0.0, 0.1)
    assert sup.t_gait == tg                           # the gait clock is not running
    assert sup.clear_latch() and not sup.latched


def test_budget_scales_and_explains():
    pg = make()
    mc = pg.gait.max_command()
    assert pg.V_MAX[0] == pytest.approx(mc["vx"])
    r = pg.do("walk 60 0 0.5")
    assert "scaled" in r
    pg.step()
    assert pg.budget_k < 1.0 and "budget" in pg.budget_note()
    assert np.linalg.norm(pg.cmd_eff[:2]) < 60.0


class FakeHW:
    """The bridge surface the Playground uses (mirror, allow_locomotion, the
    soft-entry dict, push_targets, real_pose, set_mirror, is_idle)."""

    def __init__(self, mirror="sim2real", allow_locomotion=False):
        self.mirror = mirror
        self.allow_locomotion = allow_locomotion
        self._blend = {}
        self.log = []                                # (mirror at push time, q)
        self.is_idle = lambda: True

    @property
    def pushed(self):
        return self.log[-1][1] if self.log else None

    def push_targets(self, q):
        self.log.append((self.mirror, np.array(q, float).ravel()))

    def real_pose(self):
        return np.zeros((5, 3)), np.zeros(5, bool)

    def set_mirror(self, mode):
        self.mirror = mode
        return mode


def test_hw_sim2real_holds_locomotion_and_the_probe():
    pg = make()
    run(pg, 0.5)
    hw = FakeHW()
    pg.hw = hw
    assert pg.do("walk 30").startswith("locomotion held")
    pg.step()
    assert hw.pushed is not None and np.isfinite(hw.pushed).all()
    with pg.lock:
        pg.cmd_v[:] = [30, 0, 0]                     # a direct writer (goto) is zeroed too
    pg.step()
    assert not np.any(pg.cmd_v) and not np.any(pg.cmd_eff)
    assert pg.guard_status()["probe_state"] == ["SWING"] * 5   # no probing of real feet


def test_gait_presets_and_check(tmp_path, monkeypatch):
    import playground
    monkeypatch.setattr(playground, "GAIT_DIR", str(tmp_path))
    pg = make()
    assert "default" in pg.do("gait list")
    pg.do("set gait.hstep 20")
    assert pg.do("gait save mine").startswith("saved")
    pg.do("gait default")
    assert pg.gait.hstep == rm.gait_defaults()["step_height"]
    pg.do("gait mine")
    assert pg.gait.hstep == 20.0
    out = pg.do("check")
    assert out.startswith("envelope") and "PASS" in out
    assert pg.do("check wave").split()[0] in ("PASS", "WARN", "FAIL")


def test_docstring_lists_the_new_commands():
    import playground
    doc = playground.__doc__
    for cmd in ("walk", "stop", "gesture", "say", "set", "show", "push", "record",
                "rl", "righter", "help", "gait NAME", "check", "clear", "probe"):
        assert cmd in doc, cmd


# ------------------------------------------------------------------ V2 (review of D052)
def test_idle_rule_is_anded_into_the_bridge():
    """The bridge's is_idle gets the Playground's rule ANDed in (the old
    assertion here was evaluated while standing, so it held either way)."""
    pg = make()
    run(pg, 0.5)
    hw = FakeHW(mirror="off", allow_locomotion=True)
    pg.hw = hw
    pg.step()
    assert hw.is_idle()
    assert pg.do("walk 30").startswith("walking")
    run(pg, 0.2)
    assert not pg.is_idle() and not hw.is_idle()
    pg.do("stop")
    run(pg, 2.5)
    assert pg.is_idle() and hw.is_idle()


def test_gaited_gestures_are_held_like_walking_in_sim2real():
    """Review: turn_in_place / sidestep (and D2's signed variants) run the wave
    gait directly, so zeroing cmd_v never held them."""
    import pebble_gestures2 as pg2
    pg = make()
    run(pg, 0.5)
    pg.hw = FakeHW()
    for name in ("turn_in_place", "sidestep"):
        assert pg.do(f"gesture {name}").startswith("locomotion held")
    fn, total = pg.gestures["turn_in_place"]
    assert "locomotion held" in pg.start_gesture(fn, total, "turn_in_place_right")

    def right(g, t):                                  # what cockpit_brains builds for "turn right"
        return pg2._gaited(g, t, pg2.TURN_TOTAL, (0.0, 0.0, -pg2.TURN_WZ))
    right.gaited = True
    assert "locomotion held" in pg.start_gesture(right, pg2.TURN_TOTAL, "whatever")
    with pg.lock:                                     # a direct writer of the request slot
        pg.gesture = (fn, total, pg.t)
    pg.step()
    assert pg._ges is None and pg.gesture is None
    assert any("refused: locomotion held" in m for _t, _k, m in pg.notes)
    pg.hw._blend = {0: {}}                            # the soft entry is still landing: nothing starts
    wfn, wt = pg.gestures["wave"]
    assert "soft entry" in pg.start_gesture(wfn, wt, "wave")
    pg.hw._blend = {}
    assert pg.start_gesture(wfn, wt, "wave") is None  # an arm gesture is fine once the entry landed
    pg.hw.allow_locomotion = True
    pg.end_gesture()
    run(pg, 1.0)
    assert pg.start_gesture(fn, total, "turn_in_place") is None


def test_a_sim_only_fall_never_reaches_the_real_legs():
    """Review repro: a shove in sim2real walked the sim through PLANT..FALLEN and
    the righter's routine went to the real legs open-loop (hip 124 deg)."""
    from shove import Shove
    pg = make()
    run(pg, 1.0)
    hw = FakeHW()
    pg.hw = hw
    assert pg.do("push 100 0 0.4").startswith("push refused")
    with pg.lock:                                     # a sim-only event anyway (a direct writer)
        pg.push = Shove(100.0, 0.0, dur=0.4, t0=pg.t)
    states = set()
    run(pg, 3.0, lambda p: states.add(p.sup.state))
    assert states - {"NORMAL"}                        # the sim did react
    assert hw.mirror == "off"
    live = np.array([q for m, q in hw.log if m == "sim2real"])
    assert np.abs(live - live[0]).max() < 0.1         # nothing but the standing pose was mirrored
    assert any("sim2real dropped" in m for _t, _k, m in pg.notes)


def test_a_respawn_in_sim2real_drops_the_mirror_first():
    pg = make()
    run(pg, 0.5)
    hw = FakeHW()
    pg.hw = hw
    pg.step()
    n = len(hw.log)
    pg.sup = ReflexSupervisor(pg.gait)                # what the cockpit's reset / set_world does
    pg.step()
    assert hw.mirror == "off" and all(m == "off" for m, _q in hw.log[n:])


def test_the_stream_to_the_bus_is_rate_clamped_at_the_hard_speed():
    """Review mutation: dropping the clip in _guard_target left the suite green."""
    pg = make()
    run(pg, 0.5)
    hw = FakeHW(mirror="off", allow_locomotion=True)
    pg.hw = hw
    pg.step()
    n0 = len(hw.log)
    jump = np.zeros(15)
    jump[1] = 1.0                                     # hip0 +1 rad in one tick
    pg.residual = jump
    lim = rm.servo_speed("hard") * pg.DT * 1.001
    steps = int(round(0.4 / pg.DT))
    qc = [pg._q_cmd.copy()]
    for _ in range(steps):
        pg.step()
        qc.append(pg._q_cmd.copy())
    pushed = np.array([q for _m, q in hw.log[n0 - 1:]])
    assert np.abs(np.diff(pushed, axis=0)).max() <= lim
    assert np.abs(np.diff(np.array(qc), axis=0)).max() <= lim
    ramp = int(np.ceil(1.0 / (rm.servo_speed("hard") * pg.DT)))
    planted = pg.sup._planted_q().ravel()
    assert abs(pushed[ramp // 2][1] - planted[1]) < 0.6                # still ramping half-way
    assert abs(pushed[ramp + 20][1] - (planted[1] + 1.0)) < 0.02      # arrived ~1/(4.7 DT) ticks later


def test_gait_values_are_validated_before_anything_changes():
    """Review repro: `set gait.T 0` was accepted, then every phase reader divided by zero."""
    pg = make()
    T0 = pg.gait.T
    for line in ("set gait.T 0", "set gait.T -1", "set gait.T nan", "set gait.duty 1.0",
                 "set gait.hstep -5", "set gait.h 500"):
        r = pg.do(line)
        assert "refused" in r, (line, r)
    assert pg.gait.T == T0 and np.isfinite(pg.gait.p_nom).all()
    run(pg, 0.1)
    assert pg.do("set gait.T 2.2").startswith("gait.T")
    pg.hw = FakeHW()
    assert "mirror off first" in pg.do("set gait.hstep 20")
    pg.hw = None
    assert pg.do("show")


# ------------------------------------------------------------------ review round 3 (D052 amendment)
def test_sim2real_is_refused_during_a_void_retreat_and_the_retreat_obeys_a_held_mirror():
    """Review repro: the void guard zeroes cmd_v and then retreats with the
    approach command, so is_idle / idle_reason (cmd_v only) let sim2real start
    mid-retreat and the retreat kept stepping while locomotion was held."""
    pg = make(cliff=True)
    run(pg, 1.0)
    pg.do("walk 45")
    for _ in range(int(12.0 / pg.DT)):
        pg.step()
        if pg._void_phase == "retreat":
            break
    assert pg._void_phase == "retreat" and not np.any(pg.cmd_v) and np.any(pg.cmd_eff)
    assert not pg.is_idle() and "void guard" in pg.motion_reason()
    # a mirror that holds locomotion (as sim2real would) ends the retreat where it is
    pg.hw = FakeHW(mirror="sim2real", allow_locomotion=False)
    run(pg, 0.1)
    assert pg._void_phase == "held" and not np.any(pg.cmd_eff)
    pg.hw = None


def test_a_gate_hold_ends_when_the_command_changes():
    """Review: the hold was released only when the command went to ZERO, so
    reversing away from an edge waited up to GATE_MAX_S (1.5 s)."""
    model, drop = _step_down_model()
    pg = make(model=model, z0=drop)
    run(pg, 1.0)
    pg.do("walk 0 30")
    for _ in range(int(8.0 / pg.DT)):
        pg.step()
        if pg._gate is not None:
            break
    assert pg._gate is not None
    assert pg.set_velocity(0.0, -30.0) is None
    pg.step()
    assert pg._gate is None


def test_gate_rewind_stays_inside_the_servo_budget():
    """Review: the 3x rewind asked for up to 17-18 rad/s during holds (57 % of
    hold steps past 4.7) and only the rate clamp held it. The rewind now runs
    at min(3, free / swing peak, loaded / stance peak) of the gait at that
    command: on the step-down every non-probing joint stays <= the free budget."""
    model, drop = _step_down_model()
    pg = make(model=model, z0=drop)
    assert 1.0 <= pg._rewind_rate(np.array([45.0, 0, 0])) < 1.3        # 1.12 at the envelope
    assert pg._rewind_rate(np.array([10.0, 0, 0])) > pg._rewind_rate(np.array([45.0, 0, 0]))
    run(pg, 1.0)
    orig, worst = pg._guard_target, [0.0]

    def wrap(target):
        if pg._gate is not None:
            raw = np.abs(np.asarray(target) - pg._q_cmd) / pg.DT
            m = np.ones(15, bool)
            m[3 * pg._gate["leg"]:3 * pg._gate["leg"] + 3] = False      # the probing leg lowers in 50 Hz steps
            worst[0] = max(worst[0], float(raw[m].max()))
        return orig(target)
    pg._guard_target = wrap
    pg.do("walk 0 30")
    run(pg, 8.0)
    assert pg.gate_count >= 1
    assert worst[0] <= rm.servo_speed("free") + 0.05, worst[0]


def test_playground_accounts_servo_heat_and_refuses_nothing_while_cool():
    """Review: the MJCF clips at PEAK torque, so the live sim now has the same
    thermal accounting as the RL envs (derate off): standing and walking stay
    cool; a joint past the budget is reported and noted."""
    pg = make()
    run(pg, 1.0)
    pg.do("walk 45")
    run(pg, 3.0)
    th = pg.guard_status()["thermal"]
    assert th["heat_max"] < 0.01 and th["tripped"] == []
    pg.thermal.heat[4] = 1.05 * pg.thermal.budget                     # leg 1 hip, as if held at stall
    run(pg, 0.05)
    th = pg.guard_status()["thermal"]
    assert th["tripped"] == ["leg 1 hip"] and th["hot"] == "leg 1 hip"
    assert any(k == "thermal" and "PAST its thermal budget" in m for _t, k, m in pg.notes)
    assert pg.model.actuator_forcerange[4, 1] == pytest.approx(rm.stall_nm())   # accounting only
