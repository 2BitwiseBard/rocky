"""Body leveler (D065): an integral law on the IMU tilt that levels the body by
RAISING feet — a plane of per-foot offsets under the wave gait.

Before it nothing levelled the body: the gait stands its feet on one body-frame
plane, so on a slope the body takes the slope, and a stone under one foot tilts
it. Pure numpy on the gait engine (no sim imports), written for the Pi's 50 Hz
loop as well as the sim's 500 Hz one; no Pi loop runs it yet (ros2's gait node
drives the bare WaveGait): when one runs the supervisor (B35 / B157) it must pass
grav=None until calibration.yaml has an imu: block (B159).
ReflexSupervisor(gait, leveler=BodyLeveler(gait)) opts in; without one the
supervisor is the D064 one bit for bit.

Frames and signs (pebble_gait's BODY frame, +z up, mm):
  grav   unit gravity in the body frame, [0, 0, -1] upright (sim_imu, the
         BNO085): g_x > 0 when the +x side is LOW. None = no calibrated IMU:
         the leveler releases to zero at its rate caps and stays there.
  e      the tilt error (u_x/u_z, u_y/u_z), u = -grav.
  plane  z(x, y) = P.(x, y) + c, + = foot HIGHER (the gestures' dz_leg sign,
         the OPPOSITE of the probe's probe_dz). +x low: e_x < 0, P_x < 0, so the
         -x feet rise and that side of the body comes down to level.

The law, one tick per 1 / bus_hz on a fixed schedule (the sim calls the
supervisor at 500 Hz, the Pi at 50 Hz with jitter: a call within half a call
period of the next slot ticks it, so a jittered 50 Hz loop gets one law tick per
call — a t - t_last >= tick_s rule ticked 62 % of them at 0.3 ms of jitter):
  1. e low-passed (filter_s), then a radial deadband (deadband_deg). A straight
     walk on flat ground with an ideal IMU stays inside it and reads exactly
     zero offsets; a transient past it (a turn, a stop's brace, a shove) leaves
     the plane it integrated, which the deadband then never unwinds (review 9r:
     up to 0.5 mm after a turn in place or a shove, 11.6 mm where it cancels
     the probe's 15 mm HOLD left by a turning stop, B163).
  2. P += e_db dt / tau_s, the step capped at rate_mm_s at stance_radius; only
     in NORMAL with >= min_contacts switches closed, |gyro_xy| < reflex.gyro_calm,
     the tilt <= tilt_max_deg and no hold.
  3. The window: if the plane spans more than raise_mm over the five nominal
     footholds (p_nom), P is scaled to fit (anti-windup), and c = -min there,
     so every foot is RAISED 0..raise_mm. Raise-only keeps the probe's 30 mm of
     lowering whole (the void verdict keeps its meaning) and the swing band at
     the bare gait's speed (an unraised foothold binds the envelope, a raised
     one has room); the body sinks instead, by c: up to 16.6 mm on a full
     window (13.4 for a slope along y, 16.6 at 54 deg). The 30 mm window over
     the 352 x 335 mm footprint levels fully to 4.87 deg for a slope along x
     and 5.12 deg along y; past that the leveling is partial.
  3b. The rough-ground sink cap (B162): sinking the body brings the belly down
     onto whatever is under it, and on rubble or stairs that is a stone or a
     step edge, not the plane the feet stand on. The probe feels it: walking,
     r = max - min of probe_dz over the loaded feet (~0 on flat ground and on a
     uniform slope, where the plane takes the slope; a step's or a stone's
     height on stairs and rubble), held as a peak decaying over rough_hold_s
     and frozen while the stance is fully planted (a stand on a stone keeps the
     full window; a stop on rubble keeps the cap). Past rough_r0_mm the plane
     is scaled down until c <= sink_limit(): the full window's sink (16.58 mm)
     falling to sink_max_mm at rough_r1_mm (a step at r0 when r1 <= r0). It only
     ever shrinks the plane, so a cap is partial leveling, never a lowered foot.
     No probe (probe off) reads r = 0.
  4. Per-leg copies: each leg keeps its own (P_x, P_y, c) that follows the
     plane, rate-limited at its own foot, on EVERY call (the plane moves on the
     ticks; the copies by rate x the call's dt, so a 500 Hz stream has no 50 Hz
     staircase): rate_mm_s in stance; swing_rate_mm_s for an airborne swing foot
     (swing progress inside swing_blend and >= the 15 mm band + BAND_MARGIN_MM
     over its ground at its REAL xy, after this call's move — the band check()'s
     support_plane judges); held for a swing foot inside it, where the swing already
     spends the loaded budget. (A global band gate would freeze the whole plane
     whenever any swing foot is low.)
Every call evaluates the copies at WaveGait.level_xy: a stance foot's own xy (it
stays on its plane as it sweeps), a swing foot's lift-off point moving to its
touchdown point over swing_blend (it lands on its plane); clipped to
[0, raise_mm] there.

Holds: the caller's (hold=: the playground's gate hold, void phase, probe_out,
a foot seeking past its settle window) and BRACE freeze the plane AND the
copies, so the body stays still while a foot feels for the floor; PLANT,
RECOVER, gyro, tilt and contacts freeze the plane only (the copies settle on
it). FALLEN, RIGHTED and a gesture reset it (ReflexSupervisor); release()
slews it to zero at the caps.

The envelope: the transfer keeps a swing foot inside the loaded band longer on
a tilted plane, so the speed envelope is derated by the plane's slope:
WaveGait.budget(..., level_slope=leveler.level_slope()). The ceilings are
bisected once per 0.0025 slope step; an enabled leveler primes every step up to
its window's steepest plane (prime()), so budget() in the control step is a
lookup (computed there it stalled a step 30-77 ms per new step).

    lv = BodyLeveler(gait)                        # params level:, enabled from there
    sup = ReflexSupervisor(gait, leveler=lv)      # it ticks lv inside step()
    sup.step(t, vx, vy, wz, gxy, contacts=sw, grav=imu_grav, level_hold=reasons)
    gait.budget(vx, vy, wz, level_slope=lv.level_slope())
"""
from __future__ import annotations

import numpy as np

import rocky_model as _rm
from pebble_gait import N_LEGS, SUPPORT_TOL_MM

NORMAL = "NORMAL"                     # = pebble_reflex.NORMAL (no import: the supervisor imports this side)


def _reason(hold):
    """A caller's hold as a reason string, None for no hold: a bool, a str, or a
    collection of reason strings (the playground assembles a set)."""
    if hold is None or hold is False:
        return None
    if hold is True:
        return "hold"
    if isinstance(hold, str):
        return hold or None
    names = sorted(str(h) for h in hold)
    return ",".join(names) if names else None


class BodyLeveler:
    DT_MAX_S = 0.1                    # a late tick integrates at most this: a stalled loop must not jump
    BAND_MARGIN_MM = 2.0              # an airborne foot clears the 15 mm band by this much more: check()'s
    #                                   support_plane fits a near-band swing foot into its plane, reading it
    #                                   ~1.5 mm lower than over the ground (review 9r, 5 deg converging walk)

    def __init__(self, gait, enabled=None, tau_s=None, filter_s=None, deadband_deg=None,
                 raise_mm=None, rate_mm_s=None, swing_rate_mm_s=None, tilt_max_deg=None,
                 min_contacts=None, swing_blend=None, gyro_calm=None, rough_r0_mm=None,
                 rough_r1_mm=None, sink_max_mm=None, rough_hold_s=None):
        # a None kwarg comes from params level: (rocky_model.level_defaults()); gyro_calm from
        # reflex: (the supervisor's calm threshold: the plane only moves while the body is calm)
        d = _rm.level_defaults()
        pick = lambda v, k: d[k] if v is None else v           # noqa: E731
        self.g = gait
        self.enabled = bool(pick(enabled, "enabled"))
        self.tau_s = float(pick(tau_s, "tau_s"))
        self.filter_s = float(pick(filter_s, "filter_s"))
        self.deadband_deg = float(pick(deadband_deg, "deadband_deg"))
        self.raise_mm = float(pick(raise_mm, "raise_mm"))
        self.rate_mm_s = float(pick(rate_mm_s, "rate_mm_s"))
        self.swing_rate_mm_s = float(pick(swing_rate_mm_s, "swing_rate_mm_s"))
        self.tilt_max_deg = float(pick(tilt_max_deg, "tilt_max_deg"))
        self.min_contacts = int(pick(min_contacts, "min_contacts"))
        self.swing_blend = tuple(float(x) for x in pick(swing_blend, "swing_blend"))
        self.gyro_calm = float(_rm.reflex_defaults()["gyro_calm"] if gyro_calm is None else gyro_calm)
        self.rough_r0_mm = float(pick(rough_r0_mm, "rough_r0_mm"))
        self.rough_r1_mm = float(pick(rough_r1_mm, "rough_r1_mm"))
        self.sink_max_mm = float(pick(sink_max_mm, "sink_max_mm"))
        self.rough_hold_s = float(pick(rough_hold_s, "rough_hold_s"))
        self.tick_s = 1.0 / _rm.bus_hz()
        self._fs_key = self._fs_ratio = None       # full_sink_mm()'s geometry, per p_nom
        self.reset()
        if self.enabled:
            self.prime()

    # ------------------------------------------------------------------ state
    def reset(self):
        """Zero everything now (FALLEN, RIGHTED, a gesture, a new gait): the ramps
        that follow all end on the un-leveled planted stance."""
        self.P = np.zeros(2)                       # plane slope, mm/mm (+ = foot higher)
        self.c = 0.0                               # plane height at the body origin, mm
        self.copies = np.zeros((N_LEGS, 3))        # per leg (P_x, P_y, c), rate-limited at its foot
        self.e_f = np.zeros(2)                     # filtered tilt error (tan)
        self.dz = np.zeros(N_LEGS)                 # the offsets of the last call, mm
        self.saturated = False                     # the window scaled P on the last tick
        self.rough_mm = 0.0                        # B162: the held roughness, mm (rough())
        self.sink_capped = False                   # B162: the rough-ground sink cap scaled P on the last tick
        self._probe_in = None                      # this call's probe_dz (mm, + = lowered), None = no probe
        self.hold = "reset"                        # why P did not integrate on the last tick (None: it did)
        self.imu_ok = False                        # the last tick had a usable grav
        self.com_margin_mm = None                  # CoM margin along -grav (enabled ticks with q_meas)
        self.ticks = 0                             # law ticks since the reset
        self._t_last = None                        # the last law tick's t
        self._t_next = None                        # the next tick's slot (the schedule)
        self._t_call = None                        # the last call's t (the copies move per call)
        self._call_dt = None                       # the caller's period, smoothed (the slot tolerance)

    def release(self):
        """Level off: the plane slews to zero at its rate cap and the feet follow at theirs."""
        self.enabled = False

    def engage(self):
        """Level on (primes the envelope's ceilings: cached, a lookup after the first)."""
        self.enabled = True
        self.prime()

    def max_slope(self):
        """|P| (mm/mm) of the steepest plane the window admits: raise_mm over the
        footprint's narrowest width (334.7 mm on the D064 stance, so 0.0896 at 30 mm).
        The copies interpolate between planes inside it, so no slope in use is steeper."""
        a = np.radians(np.arange(0.0, 180.0, 0.25))
        w = np.ptp(self.g.p_nom[:, :2] @ np.array([np.cos(a), np.sin(a)]), axis=0).min()
        return float(self.raise_mm / w)

    def prime(self):
        """Compute WaveGait.budget's leveled ceilings for every slope step up to
        max_slope() now, so budget(level_slope=) in the control step is a lookup
        (each step is a bisection over the swings: 30-77 ms, past a 20 ms tick).
        Returns how many were computed (0 once primed for this gait)."""
        return self.g.prime_level(self.max_slope() + self.g.LEVEL_SLOPE_STEP, self.swing_blend)

    @property
    def active(self):
        """Levelling: on, with a usable IMU on the last tick."""
        return bool(self.enabled and self.imu_ok)

    @property
    def idle(self):
        """Off and at zero: every offset is 0 wherever the feet are and a tick moves
        nothing that depends on them, so the supervisor passes no points (xy=None)
        and skips WaveGait.level_xy (about 0.2 ms, every supervisor call)."""
        return bool(not self.enabled and not self.P.any() and not self.copies.any())

    @property
    def settled(self):
        """Every leg's copy sits on the plane (and the plane on zero when off)."""
        return bool(np.all(self.copies == self.target()) and (self.enabled or not self.P.any()))

    def target(self):
        """(P_x, P_y, c): the plane the copies follow."""
        return np.array([self.P[0], self.P[1], self.c])

    def level_slope(self):
        """|slope| (mm/mm) of the steepest plane in use, the plane's or a copy's: what
        WaveGait.budget(level_slope=) derates the envelope by."""
        return float(max(np.hypot(self.P[0], self.P[1]),
                         np.hypot(self.copies[:, 0], self.copies[:, 1]).max()))

    def dz_at(self, xy):
        """(5,) offsets (mm, + = foot higher) of the legs' copies at their points xy (5, 2):
        what the supervisor adds to the feet every call. xy=None: an idle leveler's."""
        if xy is None:
            if self.copies.any():
                raise ValueError("dz_at(None) needs an idle leveler (every copy at zero)")
            return np.zeros(N_LEGS)
        xy = np.asarray(xy, float)
        z = self.copies[:, 0] * xy[:, 0] + self.copies[:, 1] * xy[:, 1] + self.copies[:, 2]
        return np.clip(z, 0.0, self.raise_mm)

    def plane_at(self, xy):
        """The plane itself at points xy (n, 2), clipped to the window (mm)."""
        xy = np.asarray(xy, float).reshape(-1, 2)
        return np.clip(xy @ self.P + self.c, 0.0, self.raise_mm)

    # ------------------------------------------------------------------ the law
    def tick(self, t, grav, gyro_xy, state, stance, switches, xy, s, clear, hold=None, q_meas=None,
             foot_xy=None, probe_dz=None):
        """Call every supervisor step; the law advances once per tick_s slot of t,
        the copies on every call. grav: body-frame unit gravity or None; gyro_xy:
        |roll/pitch rate| rad/s; state: the supervisor's; stance (5,) the commanded
        stance mask; switches (5,) the foot switches (None: never integrates); xy,
        s, clear, foot_xy: WaveGait.level_xy(foot=True)'s (the legs' plane points,
        swing progress NaN in stance, the swing foot's lift over its lift-off
        ground, the feet's real xy; foot_xy None judges the band by clear alone);
        hold: the caller's freeze (bool, reason or reasons); q_meas (5, 3):
        measured joints, for com_margin_mm; probe_dz (5,) the stance probe's
        lowering (mm, + = lowered; None = no probe: the sink cap reads no roughness).
        Returns the (5,) offsets (mm, + = foot higher) at xy (zeros when idle)."""
        xy = None if xy is None else np.asarray(xy, float)
        self._probe_in = None if probe_dz is None else np.asarray(probe_dz, float)
        if self._t_call is not None and t < self._t_call:
            self._t_last = self._t_next = None     # a new time base
        dt_call = 0.0 if self._t_call is None else min(float(t - self._t_call), self.DT_MAX_S)
        self._t_call = t
        if dt_call > 0.0:
            self._call_dt = dt_call if self._call_dt is None else self._call_dt + 0.1 * (dt_call - self._call_dt)
        # a call within half the caller's (smoothed) period, at most half a tick, of the slot
        # ticks it, and the schedule advances by tick_s: a jittered 50 Hz caller gets one tick
        # per call, a 500 Hz one every tenth (the slot to 1 ms); a loop a slot behind re-seeds
        tol = 0.5 * (self.tick_s if self._call_dt is None else min(self.tick_s, self._call_dt))
        if self._t_next is None or t >= self._t_next - tol - 1e-9:
            dt = 0.0 if self._t_last is None else min(float(t - self._t_last), self.DT_MAX_S)
            nxt = (t if self._t_next is None else self._t_next) + self.tick_s
            self._t_last, self._t_next = t, (nxt if nxt > t else t + self.tick_s)
            self.ticks += 1
            self._update(dt, grav, gyro_xy, state, stance, switches, xy, s, clear, hold, q_meas)
        if _reason(hold) is None:
            self._follow(stance, xy, s, clear, dt_call, foot_xy)
        self.dz = self.dz_at(xy)
        return self.dz.copy()

    def _update(self, dt, grav, gyro_xy, state, stance, switches, xy, s, clear, hold, q_meas):
        """One law tick: the filtered error, the plane's integral (or its hold), the window."""
        up, ok_att = None, False
        if grav is not None:
            g = np.asarray(grav, float).reshape(3)
            n = float(np.linalg.norm(g))
            if np.isfinite(n) and n > 1e-9:
                up = -g / n
                ok_att = bool(up[2] >= np.cos(np.deg2rad(self.tilt_max_deg)))
                if ok_att:
                    self.e_f = self.e_f + min(1.0, dt / self.filter_s) * (up[:2] / up[2] - self.e_f)
        self.imu_ok = up is not None
        frozen = _reason(hold)
        cap = self.rate_mm_s * dt / self.g.R0          # the plane's tilt rate: rate_mm_s at R0
        if not self.enabled or up is None:
            self.hold = "off" if not self.enabled else "no imu"
            k = float(np.hypot(self.P[0], self.P[1]))
            self.P = np.zeros(2) if k <= cap else self.P * (1.0 - cap / k)
        elif frozen is not None:
            self.hold = frozen
        elif state != NORMAL:
            self.hold = str(state).lower()
        elif not ok_att:
            self.hold = "tilt"
        elif not gyro_xy < self.gyro_calm:
            self.hold = "gyro"
        elif switches is None or int(np.count_nonzero(switches)) < self.min_contacts:
            self.hold = "contacts"
        else:
            self.hold = None
            n = float(np.hypot(self.e_f[0], self.e_f[1]))
            db = float(np.tan(np.deg2rad(self.deadband_deg)))
            if n > db:
                dP = self.e_f * ((n - db) / n) * (dt / self.tau_s)
                k = float(np.hypot(dP[0], dP[1]))
                if k > cap:
                    dP = dP * (cap / k)
                self.P = self.P + dP
        self._rough(dt, stance, switches)
        self._window()
        self.com_margin_mm = None
        if self.enabled and q_meas is not None and up is not None and switches is not None:
            sup = np.asarray(stance, bool) & np.asarray(switches, bool)
            if int(sup.sum()) >= 3:
                import pebble_feasibility as pf            # lazy: only an enabled leveler pays for it
                q = np.asarray(q_meas, float).reshape(N_LEGS, 3)
                self.com_margin_mm = float(pf.margins(q, support=sup, normal=up)[0])

    def _rough(self, dt, stance, switches):
        """B162: the roughness the probe feels while walking, held. r = max - min of
        probe_dz over the loaded feet (commanded stance and switch closed, >= 2 of
        them): ~0 on flat ground and on a uniform slope (the plane takes the slope),
        a step's or a stone's height on stairs and rubble. Held as a peak that decays
        toward the present reading over rough_hold_s; a fully planted stance (a stand,
        a stop) neither raises nor decays it, so a stand on a stone keeps the full
        window and a stop on rubble keeps the cap. No probe (S0, probe off): 0."""
        if bool(np.all(stance)):
            return
        p, r = self._probe_in, 0.0
        if p is not None and switches is not None:
            ld = np.asarray(stance, bool) & np.asarray(switches, bool)
            if int(ld.sum()) >= 2:
                r = float(np.ptp(p[ld]))
        if r >= self.rough_mm:
            self.rough_mm = r
        else:
            self.rough_mm += (r - self.rough_mm) * min(1.0, dt / self.rough_hold_s)

    def full_sink_mm(self):
        """The most a plane inside the window sinks the body (c, mm): raise_mm x the
        worst heading's -min / span over the nominal footholds (16.58 mm at 30 mm on
        the D064 stance; 15.0 for a slope along x, 13.42 along y)."""
        key = self.g.p_nom[:, :2].tobytes()
        if self._fs_key != key:
            a = np.radians(np.arange(0.0, 360.0, 0.25))
            pr = self.g.p_nom[:, :2] @ np.array([np.cos(a), np.sin(a)])
            self._fs_ratio = float((-pr.min(axis=0) / np.ptp(pr, axis=0)).max())
            self._fs_key = key
        return self.raise_mm * self._fs_ratio

    def sink_limit(self):
        """B162: the most the plane may sink the body (c, mm) at the held roughness:
        no cap up to rough_r0_mm, then falling linearly from the full window's sink
        (full_sink_mm) to sink_max_mm at rough_r1_mm (a step to sink_max_mm when
        rough_r1_mm <= rough_r0_mm). inf = no cap."""
        r = self.rough_mm
        if r <= self.rough_r0_mm:
            return np.inf
        if self.rough_r1_mm <= self.rough_r0_mm:
            return self.sink_max_mm
        f = min(1.0, (r - self.rough_r0_mm) / (self.rough_r1_mm - self.rough_r0_mm))
        full = self.full_sink_mm()
        return full + f * (self.sink_max_mm - full)

    def _window(self):
        """Anti-windup + raise-only: the plane spans <= raise_mm over the nominal
        footholds, its lowest one at 0; then the rough-ground sink cap (B162): a
        plane that sinks the body more than sink_limit() is scaled down to it (the
        cap only ever shrinks the plane)."""
        m = self.g.p_nom[:, :2] @ self.P
        span = float(m.max() - m.min())
        self.saturated = span >= self.raise_mm - 1e-9
        if span > self.raise_mm:
            self.P = self.P * (self.raise_mm / span)
            m = self.g.p_nom[:, :2] @ self.P
        self.c = 0.0 - float(m.min())
        lim = self.sink_limit()
        self.sink_capped = bool(self.c > lim)
        if self.sink_capped:
            k = max(0.0, lim) / self.c
            self.P = self.P * k
            self.c = self.c * k

    def _follow(self, stance, xy, s, clear, dt, foot_xy=None):
        """Each leg's copy toward the plane, its change at its own point <= its class's
        rate x dt. A swing foot is airborne when, after the most this call may move
        it, it still clears the 15 mm band over the higher of its copy's and the
        plane's ground under its REAL xy (check()'s support_plane band; the swing's
        own lift over its lift-off point overstates that on a tilted plane)."""
        T = self.target()
        if xy is None:                                 # idle: the plane and every copy at zero
            if T.any() or self.copies.any():
                raise ValueError("no points for a leveler that is not idle")
            return
        if dt <= 0.0:
            return
        a, b = self.swing_blend
        for i in range(N_LEGS):
            if stance[i]:
                r = self.rate_mm_s
            elif (a <= s[i] <= b and self._clear(i, xy, clear, foot_xy)
                  >= SUPPORT_TOL_MM + self.BAND_MARGIN_MM + self.swing_rate_mm_s * dt):
                r = self.swing_rate_mm_s               # airborne
            else:
                continue                               # a swing foot inside the band: held
            d = T - self.copies[i]
            dz = abs(d[0] * xy[i, 0] + d[1] * xy[i, 1] + d[2])
            lim = r * dt
            if dz <= lim:
                self.copies[i] = T
            else:
                self.copies[i] = self.copies[i] + d * (lim / dz)

    def _clear(self, i, xy, clear, foot_xy):
        """Swing foot i's height (mm) over the ground under its real xy: its lift plus
        its offset at its plane point, minus the highest offset under the foot of the
        plane and of every leg's copy (the support plane check() fits runs through the
        stance feet, each on its own copy while they converge; all clipped to the
        window, as dz_at applies them)."""
        if foot_xy is None:
            return float(clear[i])
        f = np.asarray(foot_xy, float)[i]
        c = self.copies[i]
        own = float(np.clip(c[0] * xy[i, 0] + c[1] * xy[i, 1] + c[2], 0.0, self.raise_mm))
        under = max(float(np.clip(self.copies[:, :2] @ f + self.copies[:, 2], 0.0, self.raise_mm).max()),
                    float(np.clip(self.P @ f + self.c, 0.0, self.raise_mm)))
        return float(clear[i]) + own - under

    # ------------------------------------------------------------------ report
    def status(self):
        """For the HUD / guard_status / the bench."""
        k = float(np.hypot(self.P[0], self.P[1]))
        return dict(enabled=self.enabled, active=self.active, hold=self.hold,
                    saturated=bool(self.saturated), settled=self.settled,
                    rough_mm=round(float(self.rough_mm), 2), sink_capped=bool(self.sink_capped),
                    P=[round(float(x), 6) for x in self.P], c_mm=round(float(self.c), 3),
                    slope_deg=round(float(np.degrees(np.arctan(k))), 3),
                    level_slope=round(self.level_slope(), 6),
                    offsets=[round(float(x), 3) for x in self.dz],
                    tilt_err_deg=round(float(np.degrees(np.arctan(np.hypot(*self.e_f)))), 3),
                    com_margin_mm=None if self.com_margin_mm is None else round(self.com_margin_mm, 1))
