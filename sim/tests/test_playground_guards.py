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
from pebble_reflex import ReflexSupervisor, NORMAL, PLANT, BRACE  # noqa: E402
from playground import TIPPED_DEG                                 # noqa: E402  the tip after a fire


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
    # D063: 65 N (was 60). The tumble is chaotic: on the D063 model (+27.5 g of fork cheeks) 60 N
    # tips it to 129 deg and it rolls back upright; 62-70 N all land it upside down (164 deg).
    # 9q review: 62 N. With the belly the band is narrow and the keel rolls it back above it: on
    # deae868522cc 60-65 N land it at 164.8 deg (55-58 and 66-80 tip it to 150-155 and it rolls
    # back), on c3e82f13b671 (the shell relief, -0.2 g) 60-64 N; 62 is the middle of both
    pg.do("push 62 0 0.4")
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
    # walk asks 45 (the budget fits it); teleop caps each tap at the envelope (D063: 34.2 mm/s, was 45.5)
    assert pg.cmd_v[0] == pytest.approx(45.0 if how == "walk" else min(45.0, pg.V_MAX[0]))
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


@pytest.mark.parametrize("approach_deg", [0, 15, 20, 30, 45])
def test_void_guard_holds_at_every_approach_angle(approach_deg):
    """D052 P2 (review V2): approached at 10-20 deg the robot walked off the
    cliff — the two leading legs (3 at -54, 4 at +18 deg) straddle the edge, one
    lands in the void and the other lifts at the same instant, and the three
    rear feet cannot hold the CoM. Before the fix: 10 deg fired and still fell,
    15 deg never fired, 20 deg fired late on the wrong leg after falling; 0 / 30 /
    45 deg stopped. Now the touchdown gate holds the gait on the late foot, the
    probe finds the void and the retreat plays the approach backwards.
    D063: 10 deg now sits in the lip band (the soft-landing swing and the
    command slew moved it to 9-15 deg, see _LIP), so the grid's second point
    is 15; the approach is slower (34.2 mm/s after a 2.4 s ease-in: 45 deg
    fires at t 11.4 s), so it runs 15 s (was 10) to see the settle after it."""
    pg = make(cliff=True)
    a = np.radians(approach_deg) / 2
    pg.data.qpos[3:7] = [np.cos(a), 0, 0, np.sin(a)]    # yaw the spawn: walk 45 = the approach
    mujoco.mj_forward(pg.model, pg.data)
    run(pg, 1.0)
    assert pg.do("walk 45").startswith("walking")
    log = []
    run(pg, 15.0, lambda p: log.append((p.t, p.sup.state, bool(p.probe_out.any()), p.cmd_eff.copy(),
                                        p.void is not None, float(p.data.xpos[p.torso][0]),
                                        float(p.data.xpos[p.torso][2]), float(p.data.xpos[p.torso][1]))))
    xs = [r[5] for r in log]
    assert max(xs) < 0.33, max(xs)                      # the edge is at x = 0.35
    assert pg.void is not None and pg.sup.fall_count == 0
    assert min(r[6] for r in log) > 0.16 + 0.08         # still standing on the platform
    t_out = next(r[0] for r in log if r[2])
    i_fire = next(k for k, r in enumerate(log) if r[4])
    assert log[i_fire][0] - t_out < 0.5
    # it backed off (the approach, played backwards) — measured along the approach: D063's
    # slower back-off (0.6 cycles at 34.2 mm/s) measured 2026-09-29: 40 mm at 0 deg, 38 at 30,
    # and at 45 deg 33 mm along the approach but only 20.0 in x, on the old x-only bar
    head = np.array([np.cos(np.radians(approach_deg)), np.sin(np.radians(approach_deg))])
    back = float(np.dot([xs[-1] - xs[i_fire], log[-1][7] - log[i_fire][7]], head))
    assert back < -0.02, back
    u = np.radians(pg.void["world_bearing_deg"] - np.degrees(pg.yaw()))
    u = np.array([np.cos(u), np.sin(u)])                # body frame (yaw is held through the stop)
    assert all(float(np.dot(r[3][:2], u)) <= 1.0 for r in log[i_fire + 1:])
    late = [(s, v) for t, s, _o, v, *_ in log if t >= t_out + 2.2]
    assert late and all(s == NORMAL and not np.any(v) for s, v in late)
    assert pg.do("walk 45").startswith("blocked: void")


_LIP = ("KNOWN GAP (review round 3, 2026-09-24): a leading foot lands on the edge's lip "
        "(its sphere centre 0-8 mm past it); the late-foot gate reacts 140 ms after the next "
        "touchdown and the rewind cannot put the lifted leg back before the tip. 7 of 310 "
        "approaches fell (1 deg grid, walk 15/25/35/45, -30..60 deg); the careful walk "
        "(gate.wait 1) fell in 9 of 310 at other angles. D063 (2026-09-28, the soft-landing "
        "swing + the command slew, same grid, 364 approaches): 11 fell at 9-15 deg, 3 of them "
        "after the void fired; with the hold's faster seek (PROBE_HOLD_LEAD_MM, 2026-09-29) "
        "8 fall, none after a fire: 15 @ 14-15, 25 @ 9-10, 35 / 45 @ 10-11 (35 and 45 are "
        "the same run: both are fitted to 34.2 mm/s). Slew off, the D063 gait alone: 7, "
        "before and after. Careful walk (unchanged by the fix): 6, at 15 @ 15-16, "
        "25 @ 10-11, 35 / 45 @ 9. None of these fires the void guard. D064 (the belly in the sim): "
        "the five below now tip onto the keel tub at the edge and fire 1.1-1.3 s later at 20-22 deg; "
        "a tip, not a stop")


def _approach(speed, approach_deg, seconds=20.0):
    """Walk at the cliff: (fell, t_fire, t_hold, tilt_after) — t_hold is when the
    last touchdown-gate hold before the fire began, tilt_after the most tilt
    (deg) after the fire. It runs on 3 s after the fire: a robot that fires and
    then tips over while it backs off is a fall too (D063)."""
    pg = make(cliff=True)
    a = np.radians(approach_deg) / 2
    pg.data.qpos[3:7] = [np.cos(a), 0, 0, np.sin(a)]
    mujoco.mj_forward(pg.model, pg.data)
    run(pg, 1.0)
    pg.do(f"walk {speed}")
    zmin, t_fire, t_hold, n_hold, tilt = 9.0, None, None, pg.gate_count, 0.0
    for _ in range(int(seconds / pg.DT)):
        pg.step()
        zmin = min(zmin, float(pg.data.xpos[pg.torso][2]))
        if pg.gate_count != n_hold and t_fire is None:
            n_hold, t_hold = pg.gate_count, pg.t
        if t_fire is None and pg.void is not None:
            t_fire = pg.t
        if t_fire is not None:
            tilt = max(tilt, pg.last["tilt"])
        if pg.sup.fall_count or zmin < 0.24 or (t_fire is not None and pg.t > t_fire + 3.0):
            break
    return pg.sup.fall_count > 0 or zmin < 0.24, t_fire, t_hold, tilt



def _approach_falls(speed, approach_deg, seconds=20.0):
    """True when the approach falls, never fires the void guard, or fires it only after the
    robot tipped. D064: with the keel tub modelled, a robot that tips over the lip comes to
    rest on the tub across the edge (torso 0.243-0.245 m: the 0.24 floor no longer sees it,
    no FALLEN), and its probe then runs out: in all five cases below the tub met the
    platform 1.1-1.3 s before the void fired, at 20-22 deg. The tip came first, so that is
    the same gap, counted by its tilt."""
    fell, t_fire, _h, tilt = _approach(speed, approach_deg, seconds)
    return fell or t_fire is None or tilt > TIPPED_DEG


@pytest.mark.parametrize("speed,approach_deg", [
    (45, 10), (45, 11),
    pytest.param(15, 14, marks=pytest.mark.slow), pytest.param(25, 9, marks=pytest.mark.slow),
    pytest.param(25, 10, marks=pytest.mark.slow)])
@pytest.mark.xfail(strict=True, reason=_LIP)
def test_void_guard_lip_band_known_gap(speed, approach_deg):
    """The review's counter-examples to "the void guard stops at every approach
    angle": deterministic falls where the grid of the test above steps over
    them. Strict xfail: when a fix lands these must start passing. D063 moved
    the band (the D052 cases 45 @ 13, 25 @ 15, 45 @ 12.5 / 14, 35 @ 20 now stop);
    these are the re-measured falls with the hold's faster seek. None of them
    fires the void guard before the robot tips (D064: it now hangs on the keel tub
    at the edge and fires late, see _approach_falls)."""
    assert not _approach_falls(speed, approach_deg)


@pytest.mark.parametrize("speed,approach_deg", [
    (45, 12), pytest.param(25, 11, marks=pytest.mark.slow)])
def test_a_fired_void_guard_backs_off_without_tipping(speed, approach_deg):
    """D063: these fired the void guard and still fell while it backed off. The
    gate held the gait on leg 3 (in the void) while the leg it had put back
    down, 4, stood on the lip; at a 3.5 mm lead the 30 mm probe took ~0.75 s,
    leg 4 slid off first and the void fired 0.90 s into the hold with the
    robot already tipping. A held late foot now seeks at once with a 7 mm
    lead (PROBE_HOLD_LEAD_MM): measured, the void fires 0.56 s into the hold
    (both cases) and the tilt after it peaks at 1.6 deg (25 @ 11: 1.0).
    Forcing the fire saved both up to 0.70 s into the hold, not at 0.75."""
    fell, t_fire, t_hold, tilt = _approach(speed, approach_deg)
    assert t_fire is not None and not fell
    assert t_hold is not None and t_fire - t_hold < 0.65, (t_fire, t_hold)
    assert tilt < 3.0, tilt


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
    pg.sup.gyro_trip = 0.05                   # every step of the walk now "trips" (D063: the soft landing
    #                                           peaks the walk's gyro at 0.11 rad/s; 0.3 no longer trips it)
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
                "rl", "righter", "help", "gait NAME", "check", "clear", "probe",
                "level on|off", "level.tau_s"):
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
    pg.step()                                         # D063: the retreat replays what the gait RAN, not eased
    assert np.array_equal(pg.sup.slew.v, pg._retreat["v"]) and np.any(pg._retreat["v"])
    assert np.allclose(pg.cmd_eff, -pg._retreat["v"])
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
    env = np.array([pg.V_MAX[0], 0, 0])
    assert 1.0 <= pg._rewind_rate(env) < 1.5            # 1.35 at the D063 envelope (34.2 mm/s; 1.12 at D052's 45.5)
    assert pg._rewind_rate(np.array([10.0, 0, 0])) > pg._rewind_rate(env)
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


# ------------------------------------------------------------------ D063 command slew
_TICK = 0.02                                          # the 50 Hz tick the D063 numbers are per


def _sup_ticks(sup, t, n, cmd, gyro=0.1, q_prev=None, **kw):
    """n 20 ms ticks of the bare supervisor: (t, worst joint-target step deg/tick, states, last q)."""
    worst, states = 0.0, []
    for _ in range(n):
        t += _TICK
        q, st = sup.step(t, *cmd, gyro, **kw)
        if q_prev is not None:
            worst = max(worst, float(np.degrees(np.abs(q - q_prev).max())))
        q_prev = q.copy()
        states.append(st)
    return t, worst, states, q_prev


def test_supervisor_start_from_standing_eases_every_joint():
    """D063 (B76 fix 1): standing -> the envelope through the supervisor. Passed
    straight through, a start moved a joint target ~20 deg in one 20 ms tick;
    through the slew the worst is the gait's own ~3.4 deg (review: 3.38), at
    every start phase, for a walk, a strafe, a turn and a mix."""
    g = WaveGait()
    cmds = [g.budget(*c) for c in ((45, 0, 0), (0, 45, 0), (0, 0, 0.35), (-30, 30, 0.1))]
    worst = {True: 0.0, False: 0.0}
    for slew in (True, False):
        for ph in np.linspace(0.0, 1.0, 5, endpoint=False):
            for cmd in cmds:
                sup = ReflexSupervisor(WaveGait(), arm_after=0.0, slew=slew)
                sup.t_gait = ph * sup.g.T                 # the clock only runs while moving
                t, _w, _s, q0 = _sup_ticks(sup, 0.0, 5, (0.0, 0.0, 0.0))
                _t, w, states, _q = _sup_ticks(sup, t, 150, cmd, q_prev=q0)
                assert set(states) == {NORMAL}
                worst[slew] = max(worst[slew], w)
    assert worst[True] <= 4.0, worst                      # ~4 deg per 20 ms = 3.5 rad/s, under the hard 4.7
    assert worst[False] > 10.0, worst                     # the check sees the slew go missing


def test_supervisor_stop_reaches_brace_in_one_tick_with_the_slew():
    """D063: the slew never delays a stop. Walking at the envelope, request_stop()
    and a zero command land in the SAME tick: BRACE, command exactly zero. A stop
    that arrives in a gyro-trip PLANT (not queued there) zeroes the command too,
    so the gait does not come back at the old speed and ease down for ~2 s."""
    g = WaveGait()
    walk = g.budget(45.0, 0.0, 0.0)
    sup = ReflexSupervisor(WaveGait(), arm_after=0.0)
    assert sup.slew is not None                           # wired by default
    t, _w, _s, _q = _sup_ticks(sup, 0.0, 150, walk)       # 3 s: past the 2.36 s ramp
    assert np.allclose(sup.slew.v, walk)
    sup.request_stop()
    _t, _w, states, _q = _sup_ticks(sup, t, 1, (0.0, 0.0, 0.0))
    assert states == [BRACE] and not np.any(sup.slew.v) and not np.any(sup.slew.u)
    # a gyro trip while a swing foot is high -> PLANT (finishing the step at the slewed command)
    sup = ReflexSupervisor(WaveGait(), arm_after=0.0)
    t, _w, _s, _q = _sup_ticks(sup, 0.0, 150, walk)
    for _ in range(100):
        feet, stance = sup.g.foot_targets(sup.t_gait + _TICK, *sup.slew.v)
        if (feet[~stance, 2] > -sup.g.h + sup.plant_z_low + 0.5).any():     # near the 24 mm top
            break
        t, _w, _s, _q = _sup_ticks(sup, t, 1, walk)
    t, _w, states, _q = _sup_ticks(sup, t, 1, walk, gyro=5.0)
    assert states == [PLANT] and np.allclose(sup.slew.v, walk)
    sup.request_stop()                                    # the operator's stop, mid-PLANT
    assert not np.any(sup.slew.v)
    _t, _w, states, _q = _sup_ticks(sup, t, 1, (0.0, 0.0, 0.0))
    assert states == [BRACE] and not np.any(sup.slew.v)


def test_supervisor_gyro_trip_mid_ramp_keeps_the_slewed_command():
    """Review of the D063 wiring: a PLANT used to take the raw command, so a trip
    0.4 s into a start jumped the gait to full speed mid-shove (9.1 deg/tick)."""
    g = WaveGait()
    walk = g.budget(45.0, 0.0, 0.0)
    sup = ReflexSupervisor(WaveGait(), arm_after=0.0)
    t, _w, _s, _q = _sup_ticks(sup, 0.0, 20, walk)
    v0 = sup.slew.v.copy()
    assert 0.0 < v0[0] < 0.5 * walk[0]                    # mid-ramp
    _t, _w, states, _q = _sup_ticks(sup, t, 1, walk, gyro=5.0)
    assert states[0] in (PLANT, BRACE)
    assert sup.g._vf_max(*(sup.slew.v - v0)) <= sup.slew.accel * _TICK + 1e-9


def test_supervisor_resumes_from_righted_by_the_ramp():
    """D063 review: FALLEN / RIGHTED hold the command at zero, so after righting
    the gait starts from the planted stance by the ramp (review: 3.38 deg/tick)
    instead of jumping to mid-stride at the asked speed (16.75)."""
    g = WaveGait()
    walk = g.budget(45.0, 0.0, 0.0)
    sup = ReflexSupervisor(WaveGait(), arm_after=0.0)
    t, _w, _s, _q = _sup_ticks(sup, 0.0, 100, walk)
    sup._enter_fallen(t)
    seen, q_prev, worst = [], None, 0.0
    for _ in range(400):
        t += _TICK
        q, st = sup.step(t, *walk, 0.1, tilt_deg=5.0, height=0.12)     # upright: the handoff runs
        if st in ("FALLEN", "RIGHTED"):
            assert not np.any(sup.slew.v)
        elif q_prev is not None and seen and seen[-1] in ("RIGHTED", NORMAL):
            worst = max(worst, float(np.degrees(np.abs(q - q_prev).max())))
        if not seen or seen[-1] != st:
            seen.append(st)
        q_prev = q.copy()
    assert seen == ["FALLEN", "RIGHTED", NORMAL], seen
    assert np.allclose(sup.slew.v, walk)                  # it did walk again, eased in
    assert worst <= 4.0, worst


def test_playground_stop_is_never_eased():
    """`stop` with the slew wired: BRACE on the next physics step, command zero."""
    pg = make()
    run(pg, 1.0)
    pg.do("walk 45")
    run(pg, 3.0)
    assert pg.cmd_eff[0] == pytest.approx(pg.V_MAX[0])   # eased up to the envelope (2.36 s)
    pg.do("stop")
    pg.step()
    assert pg.sup.state == BRACE
    assert not np.any(pg.sup.slew.v) and not np.any(pg.cmd_eff)


def test_playground_zero_ask_eases_down_and_idle_waits_for_the_slew():
    """A zero ASK (not a stop) slows down on the slew; the robot is not idle for
    sim2real until the command has come to rest (D063)."""
    pg = make()
    run(pg, 1.0)
    pg.do("walk 30")
    run(pg, 3.0)
    assert pg.cmd_eff[0] == pytest.approx(30.0)
    assert pg.set_velocity(0.0) is None
    pg.step()
    assert pg.sup.state == NORMAL and 29.0 < pg.cmd_eff[0] < 30.0      # still walking, easing down
    assert not pg.is_idle() and pg.motion_reason() is not None
    t0, t_rest = pg.t, None
    for _ in range(int(4.0 / pg.DT)):
        pg.step()
        if t_rest is None and not np.any(pg.sup.slew.v):
            t_rest = pg.t - t0
    assert t_rest is not None and 1.5 < t_rest < 3.0, t_rest           # 30 / 25 s on the ramp + the lag's tail
    assert pg.is_idle() and pg.sup.trip_count == 0                      # eased to a stand, never braced


def test_a_gate_hold_freezes_the_running_command_and_outlives_the_ramp():
    """D063: the gate compares the TARGET, so the slew's ramp (up to 2.4 s after
    a command change) does not end a hold; while it holds, the command the gait
    runs is frozen (the planted feet do not slide under a ramp while a foot probes)."""
    model, drop = _step_down_model()
    pg = make(model=model, z0=drop)
    run(pg, 1.0)
    pg.do("walk 0 30")
    holds, cur = [], None
    for _ in range(int(8.0 / pg.DT)):
        pg.step()
        g = pg._gate
        if g is None:
            cur = None
            continue
        assert np.array_equal(pg.sup.slew.v, g["run"])            # frozen
        if cur is None or cur["g"] is not g:
            cur = dict(g=g, n=0, ramping=bool(np.abs(g["run"] - g["v"]).max() > 0.5))
            holds.append(cur)
        cur["n"] += 1
    assert holds
    assert any(h["ramping"] and h["n"] > 10 for h in holds), [(h["n"], h["ramping"]) for h in holds]


# ------------------------------------------------------------------ D065 body leveler + IMU spec
def _true_tilt_deg(pg):
    """The torso's tilt from the TRUE attitude vs gravity (not world z: wrong on a gravity
    slope, and not pg.last: that is the IMU's reading)."""
    from sim_imu import grav_body, tilt_from_grav
    return float(np.degrees(tilt_from_grav(grav_body(pg.model, pg.data, pg.torso))))


def _gravity_slope(pg, deg):
    """A slope the cheap way (the RL envs' stand-in): gravity tilted toward +x, so +x is downhill."""
    a = np.radians(deg)
    pg.model.opt.gravity[:] = 9.81 * np.array([np.sin(a), 0.0, -np.cos(a)])


def _stand_tilt(pg, seconds, tail=1.0):
    """Mean true tilt (deg) over the last `tail` s of a `seconds` stand."""
    tl = []
    run(pg, seconds, lambda p: tl.append(_true_tilt_deg(p)))
    return float(np.mean(tl[-int(round(tail / pg.DT)):]))


def _stone_pg(h_mm, foot, **kw):
    """A 40 x 40 mm stone of h_mm under `foot`'s nominal foothold; the planted stance
    spawns h_mm up (the other four feet settle onto the floor)."""
    import world_builder as wb
    p = WaveGait().p_nom[foot, :2] / 1000.0
    model, z0 = wb.build({"base": "flat", "objects": [
        {"kind": "box", "pos": [float(p[0]), float(p[1])], "size": [0.04, 0.04, h_mm / 1000.0]}]})
    return make(model=model, z0=z0 + h_mm / 1000.0, **kw)


def test_level_is_off_by_default_and_a_flat_walk_never_levels():
    """params level.enabled lands false. On flat ground with the ideal IMU a walk's
    filtered tilt stays inside the 0.5 deg deadband, so `level on` changes nothing:
    offsets exactly zero, joint targets bit-identical to level off."""
    from playground import level_default
    if os.environ.get("ROCKY_LEVEL") is None:                 # (P9 runs the suite with it on)
        assert rm.level_defaults()["enabled"] is False and level_default() is False
    ctrl = {}
    for on in (False, True):
        pg = make(level=on)
        assert pg.level_on is on and pg.sup.leveler.enabled is on
        run(pg, 1.0)
        pg.do("walk 45")
        rows = []
        run(pg, 3.0, lambda p: rows.append(p.data.ctrl[:15].copy()))
        assert np.max(np.abs(pg.sup.leveler.dz)) == 0.0 and pg.sup.leveler.level_slope() == 0.0
        ctrl[bool(on)] = np.array(rows)
    assert np.array_equal(ctrl[False], ctrl[True])


def test_level_default_follows_rocky_level_then_params(monkeypatch):
    """ROCKY_LEVEL=1 / 0 overrides params level.enabled for Playground(level=None) (the
    bench's P9 run); anything else falls back to params."""
    from playground import level_default
    for env, want in (("1", True), ("0", False), ("yes", rm.level_defaults()["enabled"])):
        monkeypatch.setenv("ROCKY_LEVEL", env)
        assert level_default() is want, env
    monkeypatch.setenv("ROCKY_LEVEL", "1")
    assert make().level_on is True and make(level=False).level_on is False
    monkeypatch.delenv("ROCKY_LEVEL")
    assert level_default() is bool(rm.level_defaults()["enabled"])


@pytest.mark.parametrize("deg,limit", [(5.0, 0.6), (8.0, 3.5)])
def test_level_stands_level_on_a_gravity_slope(deg, limit):
    """On a slope the bare stance takes the slope; the raise-only plane takes it out,
    fully to ~5 deg, partly past it (the 30 mm window saturates)."""
    off, on = make(level=False), make(level=True)
    _gravity_slope(off, deg)
    _gravity_slope(on, deg)
    t_off, t_on = _stand_tilt(off, 5.0), _stand_tilt(on, 5.0)
    assert t_off > deg - 0.3, t_off
    assert t_on <= limit, (deg, t_on)
    lv = on.sup.leveler
    assert lv.dz.min() >= 0.0 and lv.dz.max() <= lv.raise_mm
    assert int(np.argmax(lv.dz)) in (1, 2)                    # +x is downhill: the -x feet rise
    assert on.sup.state == NORMAL and on.sup.fall_count == 0
    if deg > 5.5:
        assert lv.saturated


def test_level_takes_a_stone_under_one_foot_out():
    """A 20 mm stone under foot 0 tilts the bare stance ~3.9 deg; level on stands it
    <= 1 deg by raising the stone foot (and its neighbours), never lowering one."""
    t_off = _stand_tilt(_stone_pg(20, 0, level=False), 5.0)
    on = _stone_pg(20, 0, level=True)
    t_on = _stand_tilt(on, 5.0)
    assert t_off > 3.0 and t_on <= 1.0, (t_off, t_on)
    dz = on.sup.leveler.dz
    assert int(np.argmax(dz)) == 0 and dz.min() >= 0.0
    assert on.sup.state == NORMAL and on.sup.fall_count == 0


def test_level_resets_on_a_gesture_and_a_respawn_keeps_its_settings():
    pg = make(level=True)
    _gravity_slope(pg, 5.0)
    run(pg, 3.0)
    lv = pg.sup.leveler
    assert lv.dz.max() > 5.0
    assert pg.do("set level.tau_s 1.2").startswith("level.tau_s = 1.2") and lv.tau_s == 1.2
    fn, total = pg.gestures["wave"]
    assert pg.start_gesture(fn, total, "wave") is None
    run(pg, 0.3)
    assert pg._ges is not None and not lv.P.any() and not lv.dz.any()     # the gesture owns the legs
    pg.end_gesture()
    for _ in range(int(5.0 / pg.DT)):
        if not pg.gesture_busy:
            break
        pg.step()
    assert not pg.gesture_busy
    run(pg, 3.0)
    assert lv.dz.max() > 5.0                                  # levels again once the gesture is out
    # a respawn (the cockpit's _respawn: a fresh supervisor from the factory) keeps the ask,
    # the `set level.*` overrides and the IMU spec
    pg.set_imu(grav_sigma=0.01, seed=3)
    imu = pg.imu
    pg.sup = pg._make_sup()
    pg.step()
    lv2 = pg.sup.leveler
    assert lv2 is not lv and lv2.enabled and lv2.tau_s == 1.2 and lv2.g is pg.gait
    assert pg.imu is not imu and pg.imu.grav_sigma == 0.01


def test_level_is_off_while_mirroring_sim2real_and_holds_its_start():
    """sim2real runs the leveler off (no offsets on the real feet), and refuses to start
    while it holds a foot up (the release would reach the real legs)."""
    pg = make(level=True)
    _gravity_slope(pg, 5.0)
    run(pg, 3.0)
    lv = pg.sup.leveler
    assert lv.dz.max() > 5.0
    assert not pg.is_idle() and "leveler holds a foot" in pg.level_held_reason()
    assert pg.do("level off").startswith("level off") and not lv.enabled
    run(pg, 3.0)                                              # slews back at 20 mm/s
    assert not lv.dz.any() and pg.is_idle() and pg.level_held_reason() is None
    pg.hw = FakeHW()
    assert "inactive while mirroring" in pg.do("level on") and not lv.enabled
    peak = []
    run(pg, 2.0, lambda p: peak.append(p.sup.leveler.dz.max()))
    assert pg.level_on and not lv.enabled and max(peak) == 0.0
    assert pg.guard_status()["level_active"] is False


def test_level_commands_and_guard_status():
    import json
    pg = make(level=False)
    assert pg.do("level").startswith("level off")
    assert pg.do("level maybe") == "level on|off"
    assert pg.do("level on").startswith("level on") and pg.sup.leveler.enabled
    assert "refused" in pg.do("set level.raise_mm 40")       # the 30 mm window the probe was proven with
    assert pg.do("set level.enabled 1") == "use `level on|off`"
    assert pg.do("set level.bogus 1").startswith("unknown param level.bogus")
    assert pg.do("set level.min_contacts 4").startswith("level.min_contacts = 4")
    assert pg.sup.leveler.min_contacts == 4 and pg.level_kw == {"min_contacts": 4}
    assert "level.swing_rate_mm_s" in pg.do("set") and "level.sink_max_mm" in pg.do("set")
    assert pg.do("set level.sink_max_mm 4").startswith("level.sink_max_mm = 4")     # B162: the sink cap
    assert pg.sup.leveler.sink_max_mm == 4.0 and "refused" in pg.do("set level.sink_max_mm 40")
    pg.level_kw.pop("sink_max_mm")
    assert pg.do("set level.rough_off_mm 6").startswith("level.rough_off_mm = 6")   # B162 review: the latch
    assert pg.sup.leveler.rough_off_mm == 6.0
    pg.level_kw.pop("rough_off_mm")
    pg.step()
    gs = pg.guard_status()
    json.dumps(gs)
    assert gs["level_on"] is True and gs["level_dz"] == [0.0] * N_LEGS
    assert {"level_active", "level_hold", "level_sat", "level_slope_deg", "level_tilt_deg"} <= set(gs)
    assert "level on" in pg.do("show")


def test_sim_imu_mount_error_is_a_constant_tilt_bias():
    from sim_imu import SimIMU, grav_body
    pg = make()
    run(pg, 0.5)
    d, true = pg.data, grav_body(pg.model, pg.data, pg.torso)
    assert np.array_equal(SimIMU(pg.model, pg.torso).read(d, pg.t)["grav"], true)   # the ideal default
    m = SimIMU(pg.model, pg.torso, mount_deg=1.5, mount_axis=(1, 0, 0))
    g = m.read(d)["grav"]
    ang = np.degrees(np.arctan2(np.linalg.norm(np.cross(g, true)), g @ true))
    assert ang == pytest.approx(1.5, abs=1e-6)                # (< 1.5 by g's tiny component along x)
    assert g[0] == pytest.approx(true[0], abs=1e-12)          # about body x: g_x is untouched
    assert np.array_equal(m.read(d)["grav"], g)               # constant: no noise to average it out
    from sim_imu import axis_rotation, gyro_body               # the gyro is seen through the same mount
    d.qvel[3:6] = [0.2, -0.3, 0.1]
    mujoco.mj_forward(pg.model, d)
    w_true = gyro_body(pg.model, d, pg.torso)
    assert np.linalg.norm(w_true) > 0.1
    assert np.allclose(m.measure(d)["gyro"] - m.bias, axis_rotation(1.5, (1, 0, 0)).T @ w_true, atol=1e-12)
    assert not np.allclose(m.measure(d)["gyro"] - m.bias, w_true, atol=1e-3)
    true = grav_body(pg.model, d, pg.torso)                   # (mj_forward refreshed the pose)
    m.reset(mount_deg=0.0)
    assert np.array_equal(m.read(d)["grav"], true)
    # a zero mount error leaves the noise stream exactly as it was
    kw = dict(grav_sigma=0.02, gyro_sigma=0.02, gyro_bias=0.01)
    a = SimIMU(pg.model, pg.torso, rng=np.random.default_rng(0), **kw)
    b = SimIMU(pg.model, pg.torso, rng=np.random.default_rng(0), mount_deg=0.0, **kw)
    for _ in range(3):
        ra, rb = a.measure(d), b.measure(d)
        assert np.array_equal(ra["grav"], rb["grav"]) and np.array_equal(ra["gyro"], rb["gyro"])
    with pytest.raises(ValueError):
        SimIMU(pg.model, pg.torso, mount_deg=1.0, mount_axis=(0, 0, 0))


def test_playground_imu_spec_latency_applies_and_survives_reset_guards():
    """step() reads the IMU with the sim time, so a spec's latency is applied (before
    D065 read(d) dropped it silently); the spec is rebuilt by reset_guards."""
    assert make().imu.latency_s == 0.0 and make().imu_spec == {}
    with pytest.raises(ValueError):
        make(imu={"grav_noise": 0.1})
    pg = make(imu={"latency_s": 0.02})
    run(pg, 0.5)
    assert pg.imu.latency_s == 0.02 and pg.imu._buf
    _gravity_slope(pg, 10.0)
    seen = []
    for _ in range(20):
        pg.step()
        seen.append(pg.last["tilt"])
    n_old = sum(1 for x in seen if x < 2.0)
    assert n_old == pytest.approx(round(0.02 / pg.DT), abs=1) and seen[-1] > 9.0, seen
    imu = pg.imu
    pg.reset_guards()
    assert pg.imu is not imu and pg.imu.latency_s == 0.02


# ---- D065 review fixes (9r) ------------------------------------------------------------
def test_level_holds_name_every_floor_feel_condition():
    """The holds that keep the void verdict honest (the body still while a foot feels for
    the floor): a gate hold, a void phase, a probe_out, a gesture, a commanded-stance foot
    still seeking after its settle window. Each one names itself."""
    from playground import SEEK
    pg = make(level=True)
    run(pg, 0.5)
    assert pg._level_holds(False) == set()
    assert pg._level_holds(True) == {"gesture"}
    pg._gate = {"leg": 0}
    assert pg._level_holds(False) == {"gate"}
    pg._gate = None
    pg._void_phase = "stop"
    assert pg._level_holds(False) == {"void"}
    pg._void_phase = None
    pg.probe_out[2] = True
    assert pg._level_holds(False) == {"probe_out"}
    pg.probe_out[:] = False
    st = pg.sup.last_stance
    i = int(np.flatnonzero(st)[0])
    pg.probe_state[i] = SEEK
    pg._probe_age[i] = pg._settle_ticks()
    assert pg._level_holds(False) == set()                    # inside its settle window: free
    pg._probe_age[i] = pg._settle_ticks() + 1
    assert pg._level_holds(False) == {"seek"}


def test_a_playground_hold_freezes_the_plane_and_the_copies(monkeypatch):
    """step() hands _level_holds to the supervisor: while it names a reason, the
    leveler's plane and every copy stand still (on a slope, mid-convergence)."""
    pg = make(level=True)
    _gravity_slope(pg, 6.0)
    run(pg, 1.0)
    lv = pg.sup.leveler
    assert lv.P.any() and lv.hold is None
    monkeypatch.setattr(pg, "_level_holds", lambda monitor: {"gate"})
    pg.step()
    P, C = lv.P.copy(), lv.copies.copy()
    run(pg, 0.5)
    assert lv.hold == "gate" and np.array_equal(lv.P, P) and np.array_equal(lv.copies, C)


@pytest.mark.parametrize("approach_deg", [15, 20])
def test_level_on_the_void_guard_still_holds_the_cliff(approach_deg):
    """Review 9r: with the floor-feel holds removed the leveler tilts the body while the
    leading foot feels the edge, and a15 / a20 at walk 45 tip (peak 20.5 / 18.7 deg)
    where the shipped stack stops. With the holds it fires and stops."""
    pg = make(cliff=True, level=True)
    a = np.radians(approach_deg) / 2
    pg.data.qpos[3:7] = [np.cos(a), 0, 0, np.sin(a)]
    mujoco.mj_forward(pg.model, pg.data)
    run(pg, 1.0)
    pg.do("walk 45")
    t_fire, peak = None, 0.0
    for _ in range(int(20.0 / pg.DT)):
        pg.step()
        peak = max(peak, _true_tilt_deg(pg))
        if t_fire is None and pg.void is not None:
            t_fire = pg.t
        if pg.sup.fall_count or (t_fire is not None and pg.t > t_fire + 3.0):
            break
    assert pg.sup.fall_count == 0 and t_fire is not None and peak <= TIPPED_DEG, (t_fire, peak)


def test_reset_guards_zeroes_a_levelled_leveler():
    pg = make(level=True)
    _gravity_slope(pg, 5.0)
    run(pg, 3.0)
    lv = pg.sup.leveler
    assert lv.dz.max() > 5.0
    pg.reset_guards()
    assert not lv.P.any() and not lv.copies.any() and lv.enabled


def test_level_derates_the_envelope_the_robot_runs_and_reports_it():
    """On a saturated 5 deg plane the command door runs the derated envelope (the bare
    34.2 mm/s asked runs at 32.5), and the readouts say so: envelope(), `check`."""
    pg = make(level=True)
    _gravity_slope(pg, 5.0)
    run(pg, 4.0)
    bare = pg.gait.max_command()["v"]
    assert pg.V_MAX[0] == bare                                # the ask's cap stays the bare one
    ev = pg.envelope()
    assert ev["level_slope_deg"] > 4.0 and ev["v"] < bare - 1.0
    pg.do("walk 45")
    run(pg, 3.0)
    v = pg.envelope()["v"]                                    # (the slew's lag still closing on it)
    assert v - 0.5 < pg.cmd_eff[0] <= v + 1e-6 and pg.cmd_eff[0] < bare - 1.0, (pg.cmd_eff, v)
    assert "derates it to" in pg.do("check")
    off = make(level=False)
    assert off.envelope()["v"] == bare and off.envelope()["level_slope_deg"] == 0.0


def test_a_gait_change_keeps_the_levelled_plane():
    """Review 9r: apply_gait reset the leveler on any h / R0 key, the same value too, so
    `gait default` on a levelled slope dropped the feet up to 30 mm in one step (4.7 rad/s
    at the guard's clamp, 4.8 deg of tilt). It keeps the plane now: no offset jumps."""
    pg = make(level=True)
    _gravity_slope(pg, 5.0)
    run(pg, 5.0)
    lv = pg.sup.leveler
    assert lv.dz.max() > 20.0
    for cmd in ("gait default", f"set gait.h {pg.gait.h:g}", f"set gait.h {pg.gait.h - 4:g}"):
        prev, worst = lv.dz.copy(), 0.0
        assert "refused" not in pg.do(cmd), cmd
        for _ in range(int(0.5 / pg.DT)):
            pg.step()
            worst = max(worst, float(np.max(np.abs(lv.dz - prev))))
            prev = lv.dz.copy()
        assert worst <= lv.rate_mm_s * pg.DT + 1e-6, (cmd, worst)
        assert lv.dz.max() > 20.0
