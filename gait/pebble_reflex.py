"""Push-recovery reflex — gait-phase-aware crouch-and-brace (v2).

D017 found the blind gait survives >=1.3x bodyweight shoves with ZERO
reflexes; "stabilization is for grace, not survival." This module is the
grace — and the extra envelope. Pure Python on top of the gait engine (no
sim/ROS deps): the same supervisor later runs on the Pi with the BNO085
gyro + the SEA foot microswitches (D010) as inputs.

v1 (D022) froze every foot target and crouched the moment |gyro| tripped.
That RAISED the walking floor 33->40 N overall but opened a pocket: pushed
at +0.25T the reflex survived only 33 N where the bare gait took 48. The
2026-07-31 diagnosis (sim/diag_brace_phase.py): a shove arriving mid-swing
catches the polygon already broken — the swing leg is airborne and the
impulse BOUNCES lightly-loaded feet. Freezing then leaves a 2-leg support
line; worse, the global crouch keeps EXTENDING the loaded pivot-side legs,
actively driving the body over that line (tilt 51° vs 14.7° baseline). The
bare gait survives the same shove precisely because it keeps stepping —
feet re-plant themselves cycle after cycle.

v2 therefore knows the gait phase and the contact state:

NORMAL   — near-zero command -> all five feet PLANTED at p_nom (no marching
           in place); moving -> WaveGait targets. Watch |gyro_xy|.
  trip: swing foot LOW (<= plant_z_low above ground — just lifted or about
        to land) and >=3 contacts -> BRACE immediately (v1's win at +0.00T:
        the near-ground foot freezes into instant 5-leg support).
        swing foot HIGH mid-flight -> FINISH THE STEP first (PLANT state).
PLANT    — run the gait completely UNMODIFIED until the swing leg is back
           near the ground at its natural landing point, then freeze. Two
           sim lessons bought this: pinning + slamming the mid-air foot
           down "in place" pogo-ed the shove (48 N phase-0.25 trials fell
           that baseline survives), and freezing mid-flight leaves a 4-leg
           polygon during the shove peak. The bare gait IS the best-known
           controller while the polygon is broken — the reflex's only job
           here is to pick the moment to stop stepping. Exit to BRACE at
           swing-low + >=3 contacts (or plant_max_s deadline); exit to
           RECOVER if the gyro calms first (the stepping absorbed it).
BRACE    — freeze xy targets, ramp the crouch (crouch_ramp_s — a step-input
           crouch unloads feet mid-shove, v1 lesson #2), clock frozen.
           While braced, per-leg z is steered by the TILT VECTOR: the
           vertical rate rotation gives a body point p is (omega x p)_z —
           positive means that station is rising.
             airborne + rising  -> extend fast toward max reach (catch it)
             airborne + falling -> plant to the crouch plane
             in contact         -> crouch only while >=4 contacts; NEVER
                                   extend against a broken polygon (that is
                                   the pivot push-over failure)
           z commands clamp to per-leg reachability (IK NaN = sim death).
RECOVER  — after calm_time of quiet gyro, blend back into the (resumed)
           gait over blend_time. A new spike re-trips.
FALLEN   — session 8d (D042): braces don't save everything. When tilt
           exceeds fall_tilt for fall_confirm_s (a shove can spike tilt
           transiently — 60° held 1 s means it is ON THE GROUND, not
           wobbling), the supervisor abandons foot-space control entirely
           and hands the joints to a pluggable RIGHTER — the learned
           self-righting policy (sim: runs/recover*, hardware: same net on
           the Pi), commanding absolute joint targets. pebble_reflex stays
           torch-free: the righter is any callable(t, dt) -> q (15,) or
           (5,3) or None (None = hold pose; also the no-policy fallback).
           Exit: the HANDOFF criterion — tilt < handoff_tilt AND torso
           height > handoff_h, held handoff_hold_s (keep in sync with
           RecoverEnv v2 / eval_recover) -> RIGHTED.
RIGHTED  — joint-space smoothstep ramp from wherever the righter left the
           legs to the analytic planted stance (right_ramp_s = 1.0 s from
           params, stretched to 1.5 max|dq| / 3.0 rad/s when the legs are
           far away — D052), hold
           right_hold_s, then NORMAL with the gait clock intact. This is
           exactly eval_recover.py's hybrid handoff, made a state.
Arming   — reflex ignores the first arm_after seconds (startup transients).

D052 additions (all optional; a caller that passes none of them gets the
old supervisor exactly):
  monitor=True   — a GESTURE owns the joints. The supervisor still watches:
                   fall detection runs and FALLEN/RIGHTED take over as usual
                   (the caller must drop the gesture and use the returned q),
                   but in the other states the gait clock is frozen, gyro
                   trips are only counted (monitor_trips) and the q it returns
                   (the planted stance) is not meant to drive anything. Before
                   this, a playground gesture skipped sup.step entirely: no
                   fall detection for its whole length.
  probe_dz (mm)  — the caller's contact-probe offsets (playground stance
                   probe). Added below every foot target in NORMAL/PLANT/
                   RECOVER and captured into the brace, so a BRACE/PLANT
                   freezes from the probed z instead of snapping the probed
                   feet back up to the nominal plane mid-shove.
  q_meas + contacts — the handoff becomes rocky_recover_env.handoff_ok
                   (tilt, >= 3 switches, kinematic deck height from the joint
                   angles): the criterion the righter was trained to satisfy
                   and one the Pi can compute. Without q_meas the old
                   tilt + height rule stands.
  trip escalation — >= trip_escalate_n gyro trips within trip_escalate_s
                   (3 in 5 s) means the robot is teetering, not being
                   shoved: request_stop() and LATCH. While `latched` the
                   velocity command is ignored (treated as zero) until
                   clear_latch(). Measured motivation: at a cliff edge the
                   playground cycled NORMAL/PLANT/BRACE/RECOVER for 25 s.
                   ON by default (the Pi runs this class); an A/B harness that
                   must reproduce pre-D052 numbers passes trip_escalate_n=None.

Tuning: clean walking peaks |gyro_xy| ~1 rad/s in sim; trip defaults 1.8.
contact_aware=False reproduces v1 exactly (for A/B harnesses). Without a
contacts feed, PLANT falls back to the commanded swing state alone.
"""
from __future__ import annotations
import os
import sys

import numpy as np
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS, L1, L2, L3, Z_HIP
import rocky_model as _rm

_HANDOFF = None


def _handoff_fn():
    """rocky_recover_env.handoff_ok, imported on first use (D052): ONE criterion
    for the RL env, eval_recover and this supervisor. Lazy because it lives in
    sim/ (numpy-only at import on the Pi; mujoco and gymnasium are optional
    there). None when it cannot be imported -> the old tilt + height rule."""
    global _HANDOFF
    if _HANDOFF is None:
        try:
            sim = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim")
            if sim not in sys.path:
                sys.path.insert(0, sim)
            from rocky_recover_env import handoff_ok, HANDOFF_HOLD_S   # noqa: F401
            _HANDOFF = handoff_ok
        except Exception:                                   # noqa: BLE001 — robot without sim/
            _HANDOFF = False
    return _HANDOFF or None

NORMAL, PLANT, BRACE, RECOVER = "NORMAL", "PLANT", "BRACE", "RECOVER"
FALLEN, RIGHTED = "FALLEN", "RIGHTED"


def _z_reach_floor(i, p_body, margin=8.0):
    """Deepest reachable foot z (BODY frame) at this xy — commanding below
    it makes the IK go NaN, and one NaN in ctrl detonates MuJoCo."""
    pl = body_to_leg(i, p_body)
    dx = np.hypot(pl[0], pl[1]) - L1
    reach = L2 + L3 - 4.0
    if abs(dx) >= reach:
        return pl[2]
    return Z_HIP - float(np.sqrt(reach * reach - dx * dx)) + margin


class ReflexSupervisor:
    def __init__(self, gait: WaveGait, gyro_trip=None, gyro_calm=None,
                 crouch_mm=25.0, crouch_ramp_s=0.12, calm_time=0.30,
                 blend_time=0.35, max_brace_s=1.5, arm_after=0.8,
                 idle_eps=1.0, contact_aware=True, plant_z_low=22.0,
                 plant_max_s=0.45, seek_rate=180.0,
                 righter=None, fall_tilt_deg=60.0, fall_confirm_s=1.0,
                 handoff_tilt_deg=25.0, handoff_h=0.09, handoff_hold_s=0.5,
                 right_ramp_s=None, right_hold_s=0.5, fallen_max_s=None,
                 stall_s=None, stall_tilt_deg=None,
                 trip_escalate_n=3, trip_escalate_s=5.0):
        # D052: a None kwarg comes from params.yaml `reflex:` (rocky_model.
        # reflex_defaults()); anything passed wins, so every existing caller
        # (gait only, or gait + gyro_trip/stall_s/...) behaves as before —
        # except right_ramp_s, whose default moved 0.6 -> 1.0 on purpose.
        d = _rm.reflex_defaults()
        gyro_trip = d["gyro_trip"] if gyro_trip is None else gyro_trip
        gyro_calm = d["gyro_calm"] if gyro_calm is None else gyro_calm
        right_ramp_s = d["right_ramp_s"] if right_ramp_s is None else right_ramp_s
        fallen_max_s = d["fallen_max_s"] if fallen_max_s is None else fallen_max_s
        stall_s = d["stall_s"] if stall_s is None else stall_s
        stall_tilt_deg = d["stall_tilt_deg"] if stall_tilt_deg is None else stall_tilt_deg
        self.g = gait
        self.gyro_trip = gyro_trip
        self.gyro_calm = gyro_calm
        self.crouch = crouch_mm
        self.crouch_ramp_s = crouch_ramp_s
        self.calm_time = calm_time
        self.blend_time = blend_time
        self.max_brace_s = max_brace_s
        self.arm_after = arm_after
        self.idle_eps = idle_eps                 # mm/s: below this = standing
        self.contact_aware = contact_aware
        self.plant_z_low = plant_z_low           # mm: swing foot "near ground"
        self.plant_max_s = plant_max_s           # PLANT deadline -> BRACE
        self.seek_rate = seek_rate               # mm/s tilt-vector z steering
        self.state = NORMAL
        self.t_gait = 0.0                        # frozen during BRACE
        self._t0 = None
        self._last_t = None
        self._calm_since = None
        self._brace_since = None
        self._brace_feet = None
        self._brace_z0 = None
        self._z_now = None                       # rate-limited z state (BRACE)
        self._plant_since = None
        self._recover_t0 = None
        self._recover_from = None
        self._last_feet = None
        self._last_q = None
        self.trip_count = 0
        self.swing_at_brace = None               # diagnostics
        self._stop_req = False                   # session 6: safe-stop request
        self._stopping = False
        # session 8d (D042): fall detection + learned self-righting handoff
        self.righter = righter                   # callable(t, dt)->q or None
        self.fall_tilt = fall_tilt_deg
        self.fall_confirm_s = fall_confirm_s
        self.handoff_tilt = handoff_tilt_deg
        self.handoff_h = handoff_h               # meters (sim/IMU estimate)
        self.handoff_hold_s = handoff_hold_s
        self.right_ramp_s = right_ramp_s
        self.right_hold_s = right_hold_s
        self.fallen_max_s = fallen_max_s         # righter deadline -> retry ramp anyway
        # D048: stall rule. If the righter has not improved the best tilt by
        # stall_tilt_deg for stall_s, ramp now instead of flailing to the
        # deadline: the planted ramp rights the robot from its BACK (the
        # mode no checkpoint solves) 5/5 in the demo, the policy never does.
        self.stall_s = stall_s
        self.stall_tilt = stall_tilt_deg
        self._fallen_best = None
        self._fallen_best_t = None
        self.fall_count = 0
        self._fall_since = None
        self._fallen_t0 = None
        self._handoff_since = None
        self._right_t0 = None
        self._right_from = None
        self._right_dur = None                   # D052: set per ramp by ramp_time()
        self._q_planted = None                   # lazy: IK of p_nom
        # D052: trip escalation -> latched safe stop
        self.trip_escalate_n = trip_escalate_n
        self.trip_escalate_s = trip_escalate_s
        self._trip_times = []                    # t of recent GYRO trips (not stop requests)
        self.latched = False
        self.latch_reason = None
        self.monitor_trips = 0                   # gyro trips seen while a gesture owned the joints
        self._probe = None                       # (5,) mm, set per step by the caller
        self._brace_probe = np.zeros(N_LEGS)
        self.last_stance = None                  # commanded stance mask of the last step (probe uses it)
        self.last_feet_raw = None                # the gait's feet BEFORE the probe offset
        self.right_reason = None

    # ------------------------------------------------------------------
    def set_righter(self, fn):
        """Install the self-righting policy (callable(t, dt) -> q (15,) or
        (5,3) absolute joint targets, or None to hold pose)."""
        self.righter = fn

    def _planted_q(self):
        if self._q_planted is None:
            self._q_planted = np.array(
                [leg_ik(body_to_leg(i, self.g.p_nom[i])) for i in range(N_LEGS)])
        return self._q_planted

    def _enter_fallen(self, t):
        self.fall_count += 1
        self.state = FALLEN
        if hasattr(self.righter, "reset"):       # D048: a fresh plan per fall
            self.righter.reset()
        self._fallen_t0 = t
        self._fallen_best = None
        self._fallen_best_t = None
        self._handoff_since = None
        self._fall_since = None
        self._stop_req = False
        self._stopping = False
        self._calm_since = None

    def _fallen_step(self, t, dt, tilt_deg, height, contacts=None, q_meas=None):
        """FALLEN: righter drives; watch for the handoff criterion (handoff_ok
        from the joint state when q_meas + contacts are fed, D052)."""
        q = None
        if self.righter is not None:
            q = self.righter(t, dt)
        if q is None:
            q = self._last_q if self._last_q is not None else self._planted_q()
        else:
            q = np.asarray(q, float).reshape(N_LEGS, 3)
        hok = _handoff_fn() if (q_meas is not None and contacts is not None) else None
        if hok is not None and tilt_deg is not None:
            upright = bool(hok(tilt_deg, contacts, q_meas))
        else:
            upright = (tilt_deg is not None and tilt_deg < self.handoff_tilt and
                       (height is None or height > self.handoff_h))
        if upright:
            self._handoff_since = t if self._handoff_since is None else self._handoff_since
        else:
            self._handoff_since = None
        if tilt_deg is not None and (self._fallen_best is None or
                                     tilt_deg < self._fallen_best - self.stall_tilt):
            self._fallen_best, self._fallen_best_t = tilt_deg, t
        stalled = (self.stall_s is not None and self._fallen_best_t is not None and
                   (t - self._fallen_best_t) > self.stall_s)
        deadline = (t - self._fallen_t0) > self.fallen_max_s or stalled
        handed = (self._handoff_since is not None and
                  (t - self._handoff_since) >= self.handoff_hold_s)
        if handed or deadline:
            self.right_reason = "handoff" if handed else ("stall" if stalled else "deadline")
            self.state = RIGHTED
            self._right_t0 = t
            self._right_from = q.copy()
            self._right_dur = self.ramp_time(self._right_from)
        return q

    def ramp_time(self, q_from):
        """RIGHTED ramp duration (s): right_ramp_s, stretched so the smoothstep's
        peak (1.5 x mean speed) stays under the LOADED speed budget — the ramp
        lifts the body. D052: at 0.6 s a righter that left a hip at +90 asked
        for 5.4 rad/s, past the servo's no-load 4.7."""
        dq = np.nanmax(np.abs(np.asarray(q_from, float).reshape(N_LEGS, 3) - self._planted_q()))
        if not np.isfinite(dq):
            dq = 0.0
        return max(self.right_ramp_s, 1.5 * dq / _rm.servo_speed("loaded"))

    def _righted_step(self, t):
        """RIGHTED: smoothstep joint-space ramp to the planted stance."""
        dur = getattr(self, "_right_dur", None) or self.right_ramp_s
        a = min(1.0, (t - self._right_t0) / max(dur, 1e-3))
        a = a * a * (3 - 2 * a)
        q = (1 - a) * self._right_from + a * self._planted_q()
        if (t - self._right_t0) >= dur + self.right_hold_s:
            self.state = NORMAL
            self._calm_since = None
            self._handoff_since = None
        return q

    # ------------------------------------------------------------------
    def request_stop(self):
        """Session 6 (D025 pairing): route an EXTERNAL halt request — cliff
        detector, operator stop, watchdog — through the same PLANT→BRACE
        machinery a gyro trip uses. The gait finishes its current step,
        freezes into a full-contact crouch, then RECOVERs into the idle
        planted stance (caller is expected to command zero velocity).
        This replaces 'just zero the velocity', which leaves the wave gait
        marching in place — worst possible behavior at a cliff edge."""
        if self.state in (NORMAL, RECOVER):
            self._stop_req = True

    def clear_latch(self):
        """Operator acknowledgement of a latched safe stop (trip escalation):
        velocity commands are obeyed again and the trip window restarts."""
        was = self.latched
        self.latched = False
        self.latch_reason = None
        self._trip_times = []
        return was

    def _note_gyro_trip(self, t):
        """Count a GYRO trip (not a stop request); escalate on a cluster."""
        self._trip_times = [x for x in self._trip_times if t - x <= self.trip_escalate_s] + [t]
        if (self.trip_escalate_n and not self.latched
                and len(self._trip_times) >= self.trip_escalate_n):
            self.latched = True
            self.latch_reason = (f"{len(self._trip_times)} gyro trips in "
                                 f"{self.trip_escalate_s:.0f} s — teetering, not a shove")
            self._stopping = True                # PLANT must not calm-exit back into motion

    # ------------------------------------------------------------------
    def _gait_feet(self, vx, vy, wz):
        """(feet, commanded_stance_mask) — idle = five planted feet. The caller's
        probe offsets (D052) are applied here, so every state built on the gait
        (NORMAL, PLANT, RECOVER, the brace capture) sees the probed feet."""
        if abs(vx) + abs(vy) + abs(wz) * 100 < self.idle_eps:
            feet, stance = self.g.p_nom.copy(), np.ones(N_LEGS, dtype=bool)
        else:
            feet, stance = self.g.foot_targets(self.t_gait, vx, vy, wz)
        self.last_feet_raw = feet.copy()
        self.last_stance = stance.copy()
        if self._probe is not None:
            feet = feet.copy()
            feet[:, 2] -= self._probe
        return feet, stance

    def _swing_low(self, feet, stance):
        """True when every commanded-swing foot is near the ground."""
        gz = -self.g.h
        return all(feet[i, 2] <= gz + self.plant_z_low
                   for i in range(N_LEGS) if not stance[i])

    # ------------------------------------------------------------------
    def _enter_plant(self, t):
        self.state = PLANT
        self._plant_since = t
        self._calm_since = None

    # ------------------------------------------------------------------
    def _enter_brace(self, t, feet_now, contacts=None):
        self.trip_count += 1
        self._stopping = False                   # brace = stop delivered
        self.state = BRACE
        self._brace_since = t
        self._calm_since = None
        bf = feet_now.copy()
        self._brace_z0 = bf[:, 2].copy()
        # D052: feet_now already carries the probe offsets; the crouch plane
        # is per leg so a probed foot crouches from ITS ground, not the nominal one
        self._brace_probe = (np.zeros(N_LEGS) if self._probe is None
                             else np.asarray(self._probe, float).copy())
        bf[:, 2] = -(self.g.h + self.crouch) - self._brace_probe   # full-crouch reference
        self._brace_feet = bf
        self._z_now = self._brace_z0.copy()
        self.swing_at_brace = self._swing_legs()

    def _swing_legs(self):
        off = getattr(self.g, "phase_off", None)
        if off is None:
            return []
        ph = (self.t_gait / self.g.T + np.asarray(off)) % 1.0
        return [i for i in range(N_LEGS) if ph[i] >= self.g.duty]

    def _brace_targets(self, t, dt, contacts, gyro_vec):
        crouch_plane = -(self.g.h + self.crouch) - self._brace_probe    # per leg (D052 probe)
        feet = self._brace_feet.copy()
        if not self.contact_aware:
            # v1: one global crouch ramp from captured z
            a = min(1.0, (t - self._brace_since) / self.crouch_ramp_s)
            feet[:, 2] = (1 - a) * self._brace_z0 + a * crouch_plane
            return feet
        con = np.ones(N_LEGS, dtype=bool) if contacts is None \
            else np.asarray(contacts, dtype=bool)
        n_con = int(con.sum())
        if contacts is not None and n_con == 0:
            # D048: no foot on anything = airborne or on the back. The
            # tilt-vector seek below chatters (its goal flips with the
            # sign of omega x p every step) and there is nothing to
            # catch, so HOLD the last targets until a foot lands.
            feet[:, 2] = self._z_now
            return feet
        w = np.zeros(3) if gyro_vec is None else \
            np.array([gyro_vec[0], gyro_vec[1], 0.0])
        ramp_rate = self.crouch / max(self.crouch_ramp_s, 1e-3)
        for i in range(N_LEGS):
            p = self._brace_feet[i]
            rising = (w[0] * p[1] - w[1] * p[0])          # (omega x p)_z
            if con[i]:
                goal = crouch_plane[i] if n_con >= 4 else self._z_now[i]
                rate = ramp_rate
            elif rising > 0.15:
                goal = _z_reach_floor(i, p)               # catch the fall
                rate = self.seek_rate
            else:
                goal = crouch_plane[i]
                rate = self.seek_rate
            dz = np.clip(goal - self._z_now[i], -rate * dt, rate * dt)
            self._z_now[i] += dz
            self._z_now[i] = max(self._z_now[i], _z_reach_floor(i, p))
            feet[i, 2] = self._z_now[i]
        return feet

    # ------------------------------------------------------------------
    def _trip_entry(self, t, feet, stance, contacts):
        """Freeze now if the swing foot is near ground; else finish the step."""
        con_ok = True if contacts is None else \
            int(np.asarray(contacts, dtype=bool).sum()) >= 3
        if self.contact_aware and not (self._swing_low(feet, stance) and con_ok):
            self._enter_plant(t)
        else:
            self._enter_brace(t, feet, contacts)

    def step(self, t, vx, vy, wz, gyro_xy: float, contacts=None,
             gyro_vec=None, tilt_deg=None, height=None, monitor=False,
             probe_dz=None, q_meas=None):
        """Advance the supervisor; returns (q[5,3], state).

        t: monotonic time (s). gyro_xy: |body roll/pitch rate| rad/s (IMU).
        contacts: bool[5] foot switches (SEA microswitches), or None.
        gyro_vec: body-frame (gx, gy[, gz]) rad/s for the tilt-vector seek.
        tilt_deg: body tilt from vertical (deg) — IMU gravity vector; enables
        the FALLEN branch (without it the supervisor never declares a fall).
        height: torso height (m), sim truth or kinematic estimate; tightens
        the handoff criterion when available.
        D052 (all optional): monitor=True — a gesture owns the joints, only
        fall detection acts (see the module docstring); probe_dz (5,) mm —
        the caller's stance-probe offsets, added below the foot targets;
        q_meas (5,3) rad — measured joints, which (with contacts) switch the
        handoff to rocky_recover_env.handoff_ok. While `latched` the velocity
        command is ignored."""
        if self._t0 is None:
            self._t0 = t
        if self.latched:
            vx = vy = wz = 0.0
        self._probe = None if probe_dz is None else np.asarray(probe_dz, float)
        armed = (t - self._t0) >= self.arm_after
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        calm = gyro_xy < self.gyro_calm

        # --- fall detection (any state but FALLEN/RIGHTED, D042) ---------
        if tilt_deg is not None and self.state not in (FALLEN, RIGHTED):
            if armed and tilt_deg > self.fall_tilt:
                self._fall_since = t if self._fall_since is None else self._fall_since
                if (t - self._fall_since) >= self.fall_confirm_s:
                    self._enter_fallen(t)
            else:
                self._fall_since = None
        elif self.state == RIGHTED and tilt_deg is not None and \
                tilt_deg > self.fall_tilt:
            self._enter_fallen(t)                # ramp knocked it back over

        if self.state == FALLEN:
            q = self._fallen_step(t, dt, tilt_deg, height, contacts, q_meas)
            self._last_q = q.copy()
            self._last_feet = None
            return q, FALLEN if self.state == FALLEN else self.state
        if self.state == RIGHTED:
            q = self._righted_step(t)
            self._last_q = q.copy()
            self._last_feet = None
            return q, RIGHTED if self.state == RIGHTED else self.state

        if monitor:
            # D052: a gesture owns the joints. Fall detection above already
            # ran; here only bookkeeping. The gait clock stays frozen, a gyro
            # trip is counted but not acted on (a gesture that swings an arm
            # fast is not a shove), and a pending stop request is dropped
            # (the caller ends the gesture instead). The measured pose is
            # what FALLEN holds if it has no righter.
            if armed and gyro_xy > self.gyro_trip:
                self.monitor_trips += 1
            self._stop_req = False
            self._calm_since = None
            self.last_stance = None
            self._last_feet = None
            if q_meas is not None:
                self._last_q = np.asarray(q_meas, float).reshape(N_LEGS, 3).copy()
            return self._planted_q().copy(), self.state

        # the wave clock runs only while MOVING: idling must not advance the
        # phase, or the reflex's gait is phase-shifted vs a plain WaveGait
        # started at motion onset (this confounded the v1 phase probe —
        # supervisor runs pushed at "+0.25T" were really at a different
        # true phase than the baseline runs they were compared against)
        moving = abs(vx) + abs(vy) + abs(wz) * 100 >= self.idle_eps
        gdt = dt if moving else 0.0

        if self.state == NORMAL:
            self.t_gait += gdt
            feet, stance = self._gait_feet(vx, vy, wz)
            if (armed and gyro_xy > self.gyro_trip) or self._stop_req:
                self._stopping = self._stop_req
                self._stop_req = False
                if armed and gyro_xy > self.gyro_trip:
                    self._note_gyro_trip(t)
                self._trip_entry(t, feet, stance, contacts)
                if self.state == BRACE:
                    feet = self._brace_targets(t, dt, contacts, gyro_vec)

        elif self.state == PLANT:
            # finish the step: gait runs UNMODIFIED; we only watch for the
            # right moment to freeze
            self.t_gait += gdt
            feet, stance = self._gait_feet(vx, vy, wz)
            n_con = None if contacts is None else \
                int(np.asarray(contacts, dtype=bool).sum())
            con_ok = True if n_con is None else n_con >= 3
            if calm and self._calm_since is None:
                self._calm_since = t
            elif not calm:
                self._calm_since = None
            if (not self._stopping and self._calm_since is not None and
                    (t - self._calm_since) >= self.calm_time):
                # stepping absorbed the shove — but NOT an exit for a
                # requested stop (the gyro is calm the whole way there)
                self.state = RECOVER
                self._recover_t0 = t
                self._recover_from = feet.copy()
            elif (self._swing_low(feet, stance) and con_ok) or \
                    (t - self._plant_since) > self.plant_max_s:
                self._stopping = False        # brace reached: stop delivered
                self._enter_brace(t, feet, contacts)
                feet = self._brace_targets(t, dt, contacts, gyro_vec)

        elif self.state == BRACE:
            feet = self._brace_targets(t, dt, contacts, gyro_vec)
            if calm and self._calm_since is None:
                self._calm_since = t
            elif not calm:
                self._calm_since = None
            timed_out = (t - self._brace_since) > self.max_brace_s
            if (self._calm_since is not None and
                    (t - self._calm_since) >= self.calm_time) or timed_out:
                self.state = RECOVER
                self._recover_t0 = t
                self._recover_from = feet.copy()

        else:                                    # RECOVER
            self.t_gait += gdt
            a = min(1.0, (t - self._recover_t0) / self.blend_time)
            a = a * a * (3 - 2 * a)
            gait_feet, stance = self._gait_feet(vx, vy, wz)
            src = self._recover_from if self._recover_from is not None \
                else gait_feet
            feet = (1 - a) * src + a * gait_feet
            if (armed and gyro_xy > self.gyro_trip) or self._stop_req:
                self._stopping = self._stop_req      # re-shoved / stop request
                self._stop_req = False
                if armed and gyro_xy > self.gyro_trip:
                    self._note_gyro_trip(t)
                self._trip_entry(t, feet, stance, contacts)
                if self.state == BRACE:
                    feet = self._brace_targets(t, dt, contacts, gyro_vec)
            elif a >= 1.0:
                self.state = NORMAL

        self._last_feet = feet
        q = np.zeros((N_LEGS, 3))
        for i in range(N_LEGS):
            qi = leg_ik(body_to_leg(i, feet[i]))
            if np.isnan(qi).any():
                # unreachable: pull radially in + raise until the IK closes —
                # NEVER emit NaN
                p = feet[i].copy()
                for _ in range(8):
                    r = np.hypot(p[0], p[1])
                    p[:2] *= max(r - 20.0, 60.0) / max(r, 1e-9)
                    p[2] = min(p[2] + 8.0, -0.5 * self.g.h)
                    qi = leg_ik(body_to_leg(i, p))
                    if not np.isnan(qi).any():
                        break
                if np.isnan(qi).any():
                    qi = (self._last_q[i] if self._last_q is not None
                          else np.array([0.0, 0.3, -1.8]))
            q[i] = qi
        self._last_q = q.copy()
        return q, self.state


def body_gyro_xy(w_body: np.ndarray) -> float:
    """|roll/pitch rate| — on hardware: sqrt(gx^2+gy^2) from the BNO085."""
    return float(np.hypot(w_body[0], w_body[1]))
