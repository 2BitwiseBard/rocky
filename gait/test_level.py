"""D065 body leveler: gait/pebble_level.py and its seams in the supervisor
(pebble_reflex), the gait (level_xy, budget(level_slope=)) and feasibility
(margins(normal=)). Pure kinematics: no MuJoCo."""
import hashlib
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rocky_model as rm                                                 # noqa: E402
import pebble_feasibility as pf                                          # noqa: E402
from pebble_gait import (WaveGait, ArmedGait, swing_profile, leg_ik, _leg_ik_rows, smooth01,   # noqa: E402
                         body_to_leg, ik_ok, SUPPORT_TOL_MM, R_BODY, N_LEGS)
from pebble_reflex import ReflexSupervisor, BRACE, FALLEN               # noqa: E402
from pebble_level import BodyLeveler                                     # noqa: E402

DT = 0.002                          # the sim's supervisor rate (500 Hz)
TICK = 1.0 / rm.bus_hz()            # the leveler's (50 Hz)
PROBE_MAX_MM = 30.0                 # sim/playground.py PROBE_MAX: the probe's deepest lowering


def grav_low(deg, toward_deg=0.0):
    """Body-frame unit gravity with the side at azimuth toward_deg LOW by deg
    (sim_imu's convention: +x low -> g_x > 0)."""
    a, b = np.radians(deg), np.radians(toward_deg)
    return np.array([np.sin(a) * np.cos(b), np.sin(a) * np.sin(b), -np.cos(a)])


def idle_pts(g):
    return np.ones(N_LEGS, bool), g.p_nom[:, :2].copy(), np.full(N_LEGS, np.nan), np.zeros(N_LEGS)


def run_idle(lv, secs, grav, t0=0.0, dt=TICK, **kw):
    """Tick a standing leveler (all feet planted) for secs; returns the end time."""
    st, xy, s, cl = idle_pts(lv.g)
    kw.setdefault("switches", np.ones(N_LEGS, bool))
    sw = kw.pop("switches")
    state = kw.pop("state", "NORMAL")
    t = t0
    for k in range(int(round(secs / dt))):
        t = t0 + k * dt
        lv.tick(t, grav, kw.get("gyro_xy", 0.0), state, st, sw, xy, s, cl, hold=kw.get("hold"))
    return t + dt


def fixed_plane(g, slope, psi_deg, raise_mm=30.0):
    """An enabled leveler holding the plane slope (mm/mm) rising toward psi_deg, every
    copy on it (hold it with hold=True to keep it there)."""
    lv = BodyLeveler(g, enabled=True, raise_mm=raise_mm)
    lv.P = slope * np.array([np.cos(np.radians(psi_deg)), np.sin(np.radians(psi_deg))])
    lv._window()
    assert np.hypot(*lv.P) == pytest.approx(slope, abs=1e-12), "the window clipped the test plane"
    lv.copies[:] = lv.target()
    return lv


# ------------------------------------------------------------------ params + construction
def test_level_defaults_come_from_params_with_their_types():
    d = rm.level_defaults()
    assert set(d) == {"enabled", "tau_s", "filter_s", "deadband_deg", "raise_mm", "rate_mm_s",
                      "swing_rate_mm_s", "tilt_max_deg", "min_contacts", "swing_blend"}
    assert d["enabled"] is False                      # lands OFF (the bench flips it)
    assert isinstance(d["min_contacts"], int) and isinstance(d["swing_blend"], tuple)
    lv = BodyLeveler(WaveGait())
    assert (lv.tau_s, lv.raise_mm, lv.swing_blend) == (d["tau_s"], d["raise_mm"], d["swing_blend"])
    assert lv.gyro_calm == rm.reflex_defaults()["gyro_calm"]    # reused, not copied
    assert BodyLeveler(WaveGait(), tau_s=1.2).tau_s == 1.2      # kwargs win
    assert lv.tick_s == pytest.approx(0.02)


def test_supervisor_refuses_a_leveler_on_another_gait():
    with pytest.raises(ValueError):
        ReflexSupervisor(WaveGait(), leveler=BodyLeveler(WaveGait()))


# ------------------------------------------------------------------ the law
def test_sign_the_low_side_comes_up():
    """sim_imu's convention: +x LOW reads g_x > 0; the -x feet rise (P_x < 0), so the
    -x side of the body comes down to meet the +x side. +y low: the -y feet rise."""
    g = WaveGait()
    x, y = g.p_nom[:, 0], g.p_nom[:, 1]
    lv = BodyLeveler(g, enabled=True)
    run_idle(lv, 1.0, grav_low(3.0, 0.0))
    assert lv.P[0] < 0 and abs(lv.P[1]) < 1e-12
    dz = lv.dz.copy()
    assert dz[x < 0].min() > dz[x > 0].max()
    lv = BodyLeveler(g, enabled=True)
    run_idle(lv, 1.0, grav_low(3.0, 90.0))
    assert lv.P[1] < 0
    assert lv.dz[y < 0].min() > lv.dz[y > 0].max() and lv.dz[0] == 0.0     # leg 0 (+y) the lowest


def test_deadband_flat_reads_exactly_zero():
    lv = BodyLeveler(WaveGait(), enabled=True)
    run_idle(lv, 3.0, grav_low(0.45, 30.0))           # inside the 0.5 deg radial deadband
    assert not lv.P.any() and not lv.dz.any() and not lv.copies.any()
    assert lv.hold is None and lv.level_slope() == 0.0
    run_idle(lv, 1.0, grav_low(0.7, 30.0), t0=3.0)    # outside it
    assert lv.P.any() and lv.dz.max() > 0.0


def test_window_raise_only_and_anti_windup():
    g = WaveGait()
    lv = BodyLeveler(g, enabled=True)
    run_idle(lv, 6.0, grav_low(10.0, 0.0))
    m = g.p_nom[:, :2] @ lv.P + lv.c
    assert m.max() - m.min() == pytest.approx(lv.raise_mm, abs=1e-9)    # saturated, never past it
    assert m.min() == pytest.approx(0.0, abs=1e-9)                        # raise-only: the lowest at 0
    assert lv.saturated and lv.dz.min() >= 0.0 and lv.dz.max() <= lv.raise_mm
    span_x = np.ptp(g.p_nom[:, 0])
    assert np.hypot(*lv.P) == pytest.approx(lv.raise_mm / span_x, abs=1e-12)         # 4.87 deg about x
    # no windup: the plane comes straight back when the tilt reverses
    p0 = lv.P[0]
    t = run_idle(lv, 0.2, grav_low(10.0, 180.0), t0=6.0)
    assert lv.P[0] > p0
    run_idle(lv, 6.0, grav_low(10.0, 180.0), t0=t)
    assert lv.P[0] == pytest.approx(-p0, abs=1e-9)


def test_rate_caps_per_leg_class():
    """The plane tilts at <= rate_mm_s at R0; a stance foot's offset moves <= rate_mm_s,
    an airborne swing foot's <= swing_rate_mm_s, a swing foot inside the band not at all."""
    g = WaveGait()
    lv = BodyLeveler(g, enabled=True)
    xy = g.p_nom[:, :2].copy()
    stance = np.array([True, True, False, False, True])
    s = np.array([np.nan, np.nan, 0.5, 0.1, np.nan])          # leg 2 airborne, leg 3 just lifted
    clear = np.array([0.0, 0.0, 24.0, 2.0, 0.0])
    P, Z = [], []
    for k in range(150):
        lv.tick(k * TICK, grav_low(20.0, 54.0), 0.0, "NORMAL", stance, np.ones(N_LEGS, bool),
                xy, s, clear)
        P.append(lv.P.copy())
        Z.append(lv.dz.copy())
    dP = np.hypot(*np.diff(np.array(P), axis=0).T)
    assert dP.max() <= lv.rate_mm_s * TICK / g.R0 + 1e-12
    dZ = np.abs(np.diff(np.array(Z), axis=0))
    assert dZ[:, stance].max() <= lv.rate_mm_s * TICK + 1e-9
    assert dZ[:, 2].max() <= lv.swing_rate_mm_s * TICK + 1e-9
    assert dZ[:, 2].max() > lv.rate_mm_s * TICK + 1e-6        # airborne moves faster than stance
    assert not dZ[:, 3].any()                                  # in the band: held


def test_tick_is_time_based_one_law_at_any_call_rate():
    """The sim calls at 500 Hz, the Pi at 50 Hz: the same plane."""
    a, b = BodyLeveler(WaveGait(), enabled=True), BodyLeveler(WaveGait(), enabled=True)
    run_idle(a, 2.0, grav_low(4.0, 70.0), dt=DT)
    run_idle(b, 2.0, grav_low(4.0, 70.0), dt=TICK)
    assert np.allclose(a.P, b.P, atol=1e-12) and np.allclose(a.dz, b.dz, atol=1e-9)
    assert a.P.any()


def test_holds_freeze_and_name_their_reason():
    g = WaveGait()
    st, xy, s, cl = idle_pts(g)
    lv = BodyLeveler(g, enabled=True)
    t = run_idle(lv, 1.0, grav_low(4.0, 0.0))
    p, cp = lv.P.copy(), lv.copies.copy()
    t = run_idle(lv, 0.5, grav_low(4.0, 0.0), t0=t, hold={"seek", "gate"})
    assert lv.hold == "gate,seek" and np.array_equal(lv.P, p) and np.array_equal(lv.copies, cp)
    t = run_idle(lv, 0.5, grav_low(4.0, 0.0), t0=t, hold=True)
    assert lv.hold == "hold" and np.array_equal(lv.P, p)
    for kw, why in ((dict(state="PLANT"), "plant"), (dict(gyro_xy=1.0), "gyro"),
                    (dict(switches=np.array([1, 1, 0, 0, 0], bool)), "contacts")):
        t = run_idle(lv, 0.3, grav_low(4.0, 0.0), t0=t, **kw)
        assert lv.hold == why and np.array_equal(lv.P, p)
    t = run_idle(lv, 0.3, grav_low(35.0, 0.0), t0=t)              # a fall, not a slope
    assert lv.hold == "tilt" and np.array_equal(lv.P, p)
    # PLANT holds the plane only: copies still settle onto it
    lv.copies[:] = 0.0
    t = run_idle(lv, 0.1, grav_low(4.0, 0.0), t0=t, state="PLANT")
    assert lv.copies.any() and np.array_equal(lv.P, p)
    # no switches fed: never integrates
    lv2 = BodyLeveler(g, enabled=True)
    for k in range(50):
        lv2.tick(k * TICK, grav_low(4.0), 0.0, "NORMAL", st, None, xy, s, cl)
    assert lv2.hold == "contacts" and not lv2.P.any()


def test_release_no_imu_and_reset():
    g = WaveGait()
    lv = BodyLeveler(g, enabled=True)
    t = run_idle(lv, 3.0, grav_low(4.0, 45.0))
    assert lv.active and lv.dz.max() > 5.0
    lv.release()
    t = run_idle(lv, 3.0, grav_low(4.0, 45.0), t0=t)
    assert not lv.P.any() and not lv.dz.any() and lv.settled          # exactly zero, at the caps
    assert lv.hold == "off" and not lv.active
    lv.engage()
    t = run_idle(lv, 3.0, grav_low(4.0, 45.0), t0=t)
    assert lv.dz.max() > 5.0
    t = run_idle(lv, 3.0, None, t0=t)                                  # no calibrated IMU: release
    assert lv.hold == "no imu" and not lv.dz.any() and not lv.active
    run_idle(lv, 1.0, grav_low(4.0, 45.0), t0=t)
    lv.reset()
    assert not lv.P.any() and not lv.copies.any() and not lv.e_f.any() and lv.hold == "reset"


def test_status_and_com_margin_along_gravity():
    g = WaveGait()
    lv = BodyLeveler(g, enabled=True)
    st, xy, s, cl = idle_pts(g)
    q = pf.planted_q(g)
    up = -grav_low(3.0, 20.0)
    lv.tick(0.0, -up, 0.0, "NORMAL", st, np.ones(N_LEGS, bool), xy, s, cl, q_meas=q)
    assert lv.com_margin_mm == pytest.approx(pf.margins(q, support=np.ones(N_LEGS, bool), normal=up)[0])
    for k in ("P", "offsets", "active", "hold", "saturated", "level_slope", "com_margin_mm"):
        assert k in lv.status()


def test_kinematic_plant_levels_a_slope():
    """Closed loop on a kinematic plant: the planted feet stand on the slope, so the body
    tilt is the slope minus the plane of its stance offsets (a 50 ms body lag). Inside the
    window it levels to the deadband without overshoot; past it the window saturates."""
    def run(slope_deg, dir_deg, cmd=(0.0, 0.0, 0.0), secs=8.0):
        g = WaveGait()
        lv = BodyLeveler(g, enabled=True)
        u = np.array([np.cos(np.radians(dir_deg)), np.sin(np.radians(dir_deg))])
        a = np.tan(np.radians(slope_deg)) * u
        beta, tg, along, dz = a.copy(), 0.0, [], []
        for k in range(int(secs / DT)):
            if any(cmd):
                tg += DT
                xy, s, cl = g.level_xy(tg, *cmd, lv.swing_blend)
                st = np.isnan(s)
            else:
                st, xy, s, cl = idle_pts(g)
            z = lv.dz_at(xy)
            fit = np.linalg.lstsq(np.column_stack([xy[st], np.ones(st.sum())]), z[st], rcond=None)[0]
            beta += (DT / 0.05) * ((a - fit[:2]) - beta)
            grav = np.array([-beta[0], -beta[1], -1.0]) / np.linalg.norm([beta[0], beta[1], 1.0])
            lv.tick(k * DT, grav, 0.0, "NORMAL", st, np.ones(N_LEGS, bool), xy, s, cl)
            along.append(np.degrees(np.arctan(beta @ u)))
            dz.append(z)
        return np.array(along), np.array(dz)
    for cmd in ((0.0, 0.0, 0.0), (32.0, 0.0, 0.0)):
        tilt, dz = run(3.0, 37.0, cmd)
        assert tilt[-1] <= 0.5 + 1e-6                                   # to the deadband
        assert tilt[int(3.0 / DT):].max() <= 0.6                        # within 3 s
        assert tilt.min() > 0.0                                         # never past level
        assert dz.min() >= 0.0 and dz.max() <= 30.0
    tilt, dz = run(8.0, 0.0)
    full = np.degrees(np.arctan(30.0 / np.ptp(WaveGait().p_nom[:, 0])))
    assert tilt[-1] == pytest.approx(8.0 - full, abs=0.3)               # partial: the window is full
    assert dz[-1].max() == pytest.approx(30.0, abs=0.5)


# ------------------------------------------------------------------ gait: level_xy, budget
def test_level_xy_is_the_foot_in_stance_and_lands_on_touchdown():
    g = WaveGait()
    cmd = (30.0, -8.0, 0.05)
    a, b = 0.30, 0.70
    feet, stance = g.foot_targets(0.37, *cmd)
    xy, s, clear = g.level_xy(0.37, *cmd)
    assert np.array_equal(np.isnan(s), stance)
    assert np.array_equal(xy[stance], feet[stance, :2])                  # stance: the foot itself
    assert np.allclose(clear[stance], 0.0)
    # continuity through lift-off and touchdown, and the blend window
    for i in range(N_LEGS):
        ph_lift = g.duty
        t_lift = ((ph_lift - g.phase_off[i]) % 1.0) * g.T
        before, after = g.level_xy(t_lift - 1e-7, *cmd)[0][i], g.level_xy(t_lift + 1e-7, *cmd)[0][i]
        assert np.allclose(before, after, atol=1e-3)
        t_land = ((1.0 - g.phase_off[i]) % 1.0) * g.T
        before, after = g.level_xy(t_land - 1e-7, *cmd)[0][i], g.level_xy(t_land + 1e-7, *cmd)[0][i]
        assert np.allclose(before, after, atol=1e-3)
        for sv in (0.1, a, b, 0.9):
            t = t_lift + sv * (1 - g.duty) * g.T
            pts = g.level_xy(t, *cmd)
            p_lift = g.level_xy(t_lift + 1e-9, *cmd)[0][i]
            p_land = g.level_xy(t_land - 1e-9, *cmd)[0][i]
            w = smooth01((sv - a) / (b - a))
            assert np.allclose(pts[0][i], p_lift + (p_land - p_lift) * w, atol=1e-4)
            assert pts[1][i] == pytest.approx(sv, abs=1e-6)
            lift = g.hstep * g.lift_scale(g._vf_max(*cmd))
            assert pts[2][i] == pytest.approx(lift * swing_profile(sv, g.duty)[1], abs=1e-4)
    with pytest.raises(NotImplementedError):
        ArmedGait().level_xy(0.0, 30.0, 0.0, 0.0)


def test_rows_ik_is_leg_ik():
    rng = np.random.default_rng(0)
    P = np.column_stack([rng.uniform(20, 140, 3000), rng.uniform(-60, 60, 3000),
                         rng.uniform(-170, -40, 3000)])
    A, B = np.array([leg_ik(p) for p in P]), _leg_ik_rows(P)
    fin = np.isfinite(A).all(axis=1)
    assert np.array_equal(fin, np.isfinite(B).all(axis=1)) and fin.any() and not fin.all()
    assert np.allclose(A[fin], B[fin], atol=1e-12, rtol=0)


def test_budget_level_slope_zero_is_the_bare_envelope():
    g = WaveGait()
    for cmd in ((45.0, 0.0, 0.0), (20.0, 30.0, 0.1), (0.0, 0.0, 0.4)):
        assert g.budget(*cmd, level_slope=0.0) == g.budget(*cmd)
    assert g.vf_limit(level_slope=0.0) == g.vf_limit()
    assert g.vf_limit()[2] == 34.2041015625                              # D063's lift ceiling
    lim = [min(g.vf_limit(level_slope=np.tan(np.radians(d)))) for d in (0.0, 2.0, 5.0, 8.0)]
    assert lim[0] > lim[1] > lim[2] > lim[3] > 0.9 * lim[0]
    assert lim[2] == pytest.approx(32.48, abs=0.01)
    assert WaveGait.level_slope_q(0.0) == 0.0 and WaveGait.level_slope_q(1e-4) == 0.0025
    assert WaveGait.level_slope_q(0.005) == 0.005 and WaveGait.level_slope_q(-0.0051) == 0.0075
    assert g.budget(45.0, 0.0, 0.0, level_slope=np.nan) == (0.0, 0.0, 0.0)   # unknown plane: stop
    assert g.max_command(level_slope=np.tan(np.radians(5)))["v"] == lim[2]


def _swing_peaks(g, u, speed, plane, w, z0, blend, n=400):
    """env_check's replica, re-sampled 2.5x finer than the bisection: (loaded, free) peak
    joint rates of ONE leg's swing — stride speed*u (LEG frame) over a foothold raised z0,
    a plane of slope `plane` rising toward w through it, the swing offset the plane at
    level_xy's blended point, the 15 mm band judged over the plane under the foot."""
    pn = np.array([g.R0 - R_BODY, 0.0, -g.h + z0])
    vf = speed * np.array([u[0], u[1], 0.0])
    T_st, T_sw = g.duty * g.T, (1 - g.duty) * g.T
    s = np.linspace(0.0, 1.0, n)
    xy, z = swing_profile(s, g.duty)
    p_lift, p_land = pn - vf * 0.5 * T_st, pn + vf * 0.5 * T_st
    P = p_lift + (p_land - p_lift) * xy[:, None]
    P[:, 2] += g.hstep * g.lift_scale(speed) * z
    pt = p_lift + (p_land - p_lift) * smooth01((s - blend[0]) / (blend[1] - blend[0]))[:, None]
    P[:, 2] += plane * ((pt[:, :2] - pn[:2]) @ w)                # the leveler's offset
    ground = pn[2] + plane * ((P[:, :2] - pn[:2]) @ w)           # the plane under the foot
    Q = _leg_ik_rows(P)
    assert np.isfinite(Q).all()
    V = np.abs(np.diff(Q, axis=0)).max(axis=1) / (T_sw / (n - 1))
    low = (P[:, 2] - ground) < SUPPORT_TOL_MM
    both = low[:-1] & low[1:]
    return (float(V[both].max()) if both.any() else 0.0), float(V[~both].max())


def test_envelope_every_speed_class_fits_on_a_leveled_plane():
    """At budget(level_slope=tan theta) every swing on that plane keeps the loaded class
    (inside 15 mm of the plane under the foot, pebble_feasibility.check's support_plane
    band) under 3.0 rad/s and the free class under 4.0, for 0 / 5 / 8 deg: any stride
    direction, the plane at any heading to it, the foothold raised 0..30 mm, any speed
    up to the envelope. And the derate is load-bearing: the bare 34.2 mm/s on 5 deg
    breaks the loaded class."""
    g = WaveGait()
    blend = rm.level_defaults()["swing_blend"]
    lo, fr = rm.servo_speed("loaded"), rm.servo_speed("free")
    worst = {}
    for deg in (0.0, 5.0, 8.0):
        plane = np.tan(np.radians(deg))
        v = min(g.vf_limit(level_slope=plane))
        wl = wf = 0.0
        for ang in np.radians(np.arange(0, 360, 15)):
            u = np.array([np.cos(ang), np.sin(ang)])
            for rel in np.radians(np.arange(0, 360, 45)):
                w = np.array([np.cos(ang + rel), np.sin(ang + rel)])
                for z0 in (0.0, 10.0, 30.0):
                    for frac in (0.5, 0.8, 1.0):
                        a, b = _swing_peaks(g, u, frac * v, plane, w, z0, blend)
                        wl, wf = max(wl, a), max(wf, b)
        worst[deg] = (v, wl, wf)
        assert wl <= lo and wf <= fr, (deg, v, wl, wf)
    assert worst[0.0][0] == g.vf_limit()[2]
    v0 = worst[0.0][0]
    bare = max(_swing_peaks(g, np.array([np.cos(a), np.sin(a)]), v0, np.tan(np.radians(5.0)),
                            sgn * np.array([np.cos(a), np.sin(a)]), 0.0, blend)[0]
               for a in np.radians(np.arange(0, 360, 15)) for sgn in (1.0, -1.0))
    assert bare > lo


@pytest.mark.slow
def test_envelope_through_the_supervisor():
    """The same through the real supervisor + leveler (a held 5 / 8 deg plane, the window
    widened so it is not clipped; raise-only clips a downhill landing, which only lowers
    the rates): joint targets at 400 Hz, each sample's loaded feet from pebble_feasibility.
    support_plane (with the CoM), as check() classifies them. Every class fits its budget
    at the derated speed, every target keeps the 2 deg joint guard."""
    lo, fr = rm.servo_speed("loaded"), rm.servo_speed("free")
    dt = 0.0025
    for deg in (5.0, 8.0):
        for head, rel in ((90.0, 180.0), (90.0, 0.0), (18.0, 180.0)):
            g = WaveGait()
            lv = fixed_plane(g, np.tan(np.radians(deg)), head + rel, raise_mm=60.0)
            sup = ReflexSupervisor(g, arm_after=1e9, slew=False, leveler=lv)
            cmd = g.budget(45 * np.cos(np.radians(head)), 45 * np.sin(np.radians(head)), 0.0,
                           level_slope=lv.level_slope())
            Q, L = [], []
            for k in range(int(g.T / dt) + 1):
                q, _ = sup.step(k * dt, *cmd, 0.0, contacts=np.ones(N_LEGS, bool),
                                grav=np.array([0.0, 0.0, -1.0]), level_hold=True)
                Q.append(q.copy())
                L.append(pf.support_plane(pf.feet_body(q), com=pf.com_body(q))[2])
                assert ik_ok(q, pf.GUARD_DEG)
            Q, L = np.array(Q), np.array(L)
            V = np.abs(np.diff(Q, axis=0)) / dt
            both = L[:-1] & L[1:]
            assert V[both].max() <= lo and V[~both].max() <= fr, (deg, head, rel)


def test_raised_and_probed_feet_stay_inside_the_joint_guard():
    """Every stance foot at the envelope on a full-window plane, raised by its offset,
    and also lowered from there by the probe's full PROBE_MAX, keeps ik_ok at 2 deg."""
    for psi in (0.0, 90.0, 200.0):
        g = WaveGait()
        span = np.ptp(g.p_nom[:, :2] @ np.array([np.cos(np.radians(psi)), np.sin(np.radians(psi))]))
        lv = fixed_plane(g, 30.0 / span - 1e-9, psi)
        cmd = g.budget(45.0 * np.cos(np.radians(psi + 30)), 45.0 * np.sin(np.radians(psi + 30)), 0.0,
                       level_slope=lv.level_slope())
        for t in np.linspace(0.0, g.T, 81):
            feet, stance = g.foot_targets(t, *cmd)
            dz = lv.dz_at(g.level_xy(t, *cmd)[0])
            for i in np.nonzero(stance)[0]:
                for extra in (0.0, -PROBE_MAX_MM):
                    p = feet[i] + np.array([0.0, 0.0, dz[i] + extra])
                    assert ik_ok(leg_ik(body_to_leg(i, p)), pf.GUARD_DEG), (psi, t, i, extra)


# ------------------------------------------------------------------ supervisor seams
def _trace(sup, n=3000, **kw):
    """A walk with a probed foot and two gyro trips (PLANT, BRACE, RECOVER) at 500 Hz."""
    Q, S = [], []
    for k in range(n):
        t = k * DT
        gxy = 2.5 if (2.0 <= t < 2.05 or 4.65 <= t < 4.7) else 0.1
        pdz = np.array([0.0, 3.0, 0.0, 0.0, 5.0]) if t >= 1.0 else None
        q, st = sup.step(t, 30.0, 4.0, 0.02, gxy, contacts=np.ones(N_LEGS, bool),
                         gyro_vec=np.array([0.3, -0.2]), tilt_deg=1.0, probe_dz=pdz, **kw)
        Q.append(np.asarray(q, float).reshape(N_LEGS, 3))
        S.append(st)
    digest = hashlib.sha256(np.round(np.array(Q), 9).tobytes() + "".join(S).encode()).hexdigest()[:16]
    return digest, sorted(set(S))


HEAD_TRACE = "a3ebd8f92a6a38f0"     # _trace of the D064 supervisor (6044c5e), before D065


def test_no_leveler_is_the_d064_supervisor_bit_for_bit():
    """leveler=None is HEAD's trace; so are an OFF leveler and an ON one on flat ground
    (inside the deadband its offsets are exactly zero)."""
    digest, states = _trace(ReflexSupervisor(WaveGait(), arm_after=0.0))
    assert states == ["BRACE", "NORMAL", "PLANT", "RECOVER"]
    assert digest == HEAD_TRACE
    g = WaveGait()
    assert _trace(ReflexSupervisor(g, arm_after=0.0, leveler=BodyLeveler(g)))[0] == HEAD_TRACE
    g = WaveGait()
    lv = BodyLeveler(g, enabled=True)
    assert _trace(ReflexSupervisor(g, arm_after=0.0, leveler=lv), grav=grav_low(0.4, 10.0))[0] == HEAD_TRACE
    assert lv.hold in (None, "brace", "plant", "recover") and not lv.P.any()


def test_an_idle_leveler_needs_no_points(monkeypatch):
    """Off and at zero, the supervisor never asks the gait for the plane's points (0.2 ms
    a call); a leveler still releasing, or on, does; dz_at(None) only when idle."""
    g = WaveGait()
    lv = BodyLeveler(g)
    assert lv.idle and not lv.dz_at(None).any()
    calls = []
    real = g.level_xy
    monkeypatch.setattr(g, "level_xy", lambda *a, **k: calls.append(1) or real(*a, **k))
    sup = ReflexSupervisor(g, arm_after=0.0, leveler=lv)
    for k in range(200):
        sup.step(k * DT, 30.0, 0.0, 0.0, 0.1, contacts=np.ones(N_LEGS, bool), grav=grav_low(0.2, 0.0))
    assert not calls and not sup.level_dz.any()
    lv.copies[0] = [0.0, 0.0, 1.0]                        # a copy off zero: not idle any more
    assert not lv.idle
    with pytest.raises(ValueError):
        lv.dz_at(None)
    sup.step(200 * DT, 30.0, 0.0, 0.0, 0.1, contacts=np.ones(N_LEGS, bool), grav=grav_low(0.2, 0.0))
    assert calls
    lv.reset()
    lv.engage()
    assert not lv.idle                                   # on: the law may move the plane any tick


def test_offsets_go_on_before_last_feet_raw_and_the_probe_after():
    g = WaveGait()
    lv = fixed_plane(g, np.tan(np.radians(4.0)), 30.0)
    sup = ReflexSupervisor(g, arm_after=1e9, leveler=lv)
    probe = np.array([0.0, 4.0, 0.0, 2.0, 0.0])
    for k in range(400):
        q, _ = sup.step(k * DT, 25.0, 0.0, 0.0, 0.0, contacts=np.ones(N_LEGS, bool),
                        grav=np.array([0.0, 0.0, -1.0]), probe_dz=probe, level_hold=True)
    feet, _st = g.foot_targets(sup.t_gait, *sup.slew.v)
    dz = lv.dz_at(g.level_xy(sup.t_gait, *sup.slew.v)[0])
    assert dz.max() > 5.0 and np.array_equal(sup.level_dz, dz)
    assert np.allclose(sup.last_feet_raw[:, 2], feet[:, 2] + dz)
    want = feet.copy()
    want[:, 2] += dz - probe
    assert np.allclose(q, [leg_ik(body_to_leg(i, want[i])) for i in range(N_LEGS)])


def test_swing_low_measures_each_foot_from_its_own_ground():
    g = WaveGait()
    sup = ReflexSupervisor(g, leveler=BodyLeveler(g, enabled=True))
    stance = np.array([True, True, False, True, True])
    feet = g.p_nom.copy()
    sup.level_dz = np.array([0.0, 0.0, 25.0, 0.0, 0.0])
    feet[2, 2] = -g.h + 25.0 + 10.0                       # 10 mm over its own (raised) ground
    assert sup._swing_low(feet, stance)
    sup.level_dz = np.zeros(N_LEGS)
    assert not sup._swing_low(feet, stance)               # 35 over the nominal one


def test_brace_crouches_from_the_leveled_plane():
    g = WaveGait()
    lv = fixed_plane(g, np.tan(np.radians(4.5)), 120.0)
    sup = ReflexSupervisor(g, arm_after=0.0, leveler=lv)
    con = np.ones(N_LEGS, bool)
    for k in range(200):
        sup.step(k * DT, 0.0, 0.0, 0.0, 0.0, contacts=con, grav=np.array([0, 0, -1.0]), level_hold=True)
    dz = sup.level_dz.copy()
    assert dz.max() > 10.0
    sup.request_stop()
    for k in range(200, 400):
        sup.step(k * DT, 0.0, 0.0, 0.0, 0.0, contacts=con, gyro_vec=np.zeros(2),
                 grav=np.array([0, 0, -1.0]), level_hold=True)
        if sup.state == BRACE and k * DT - sup._brace_since > 2 * sup.crouch_ramp_s:
            break
    assert sup.state == BRACE and np.array_equal(sup._brace_level, dz)
    assert lv.hold == "hold"
    assert np.allclose(sup._last_feet[:, 2], -(g.h + sup.crouch) + dz)   # every foot from its own plane


def test_fallen_and_a_gesture_reset_the_leveler():
    g = WaveGait()
    lv = BodyLeveler(g, enabled=True)
    sup = ReflexSupervisor(g, arm_after=0.0, leveler=lv)
    con = np.ones(N_LEGS, bool)
    t = 0.0
    for k in range(1500):
        t = k * DT
        sup.step(t, 0.0, 0.0, 0.0, 0.0, contacts=con, tilt_deg=4.0, grav=grav_low(4.0, 0.0))
    assert lv.P.any() and sup.level_dz.max() > 5.0
    sup.step(t + DT, 0.0, 0.0, 0.0, 0.0, contacts=con, tilt_deg=4.0, grav=grav_low(4.0), monitor=True)
    assert not lv.P.any() and not lv.copies.any() and not sup.level_dz.any()
    for k in range(1500):
        t += DT
        sup.step(t, 0.0, 0.0, 0.0, 0.0, contacts=con, tilt_deg=4.0, grav=grav_low(4.0, 0.0))
    assert lv.P.any()
    for k in range(int(1.5 / DT)):
        t += DT
        sup.step(t, 0.0, 0.0, 0.0, 0.0, contacts=con, tilt_deg=80.0, grav=grav_low(80.0, 0.0))
    assert sup.state == FALLEN and not lv.P.any() and not sup.level_dz.any()


# ------------------------------------------------------------------ feasibility
def test_margins_honours_a_normal_given_alone():
    """D065: margins(q, normal=n) used to drop n for the searched plane's unless support
    came with it; the default and the explicit paths are unchanged."""
    q = pf.planted_q(offset=(18.0, -9.0, 0.0))
    n0, _p, sup = pf.support_plane(pf.feet_body(q), com=pf.com_body(q))
    tilt = np.array([np.sin(np.radians(12.0)), 0.0, np.cos(np.radians(12.0))])
    base = pf.margins(q)
    assert pf.margins(q, support=None, normal=None)[:2] == base[:2]
    assert pf.margins(q, sup, n0)[:2] == base[:2] and np.array_equal(base[2], sup)
    alone = pf.margins(q, normal=tilt)
    assert alone[:2] == pf.margins(q, support=sup, normal=tilt)[:2] and np.array_equal(alone[2], sup)
    assert alone[0] != pytest.approx(base[0], abs=0.5)
