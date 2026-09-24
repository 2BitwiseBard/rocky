"""Hardware bridge (D051, made safe in D052) — the real servo bus beside the sim, one leg at a time.

The cockpit's Hardware panel drives this. It opens the Feetech bus
(`/dev/ttyACM0`, or `mock` for the byte-level simulator the bench scripts
rehearse on), scans for servos, and mirrors between the sim and whatever
legs answered:

    mirror = "sim2real"   the sim's joint targets are streamed to every MIRROR
                          leg (all three servos answered the scan, none
                          tripped, not degraded) at the bus rate
    mirror = "real2sim"   the mirror legs' measured positions drive the sim's
                          joints — hold the real leg and the sim leg follows
    mirror = "off"        telemetry only

Legs that are not on the bus stay analytic in the sim, so a robot with one
leg built is a robot with one real leg and four simulated ones. Partial legs
(one servo of three on the bench — the runbook's normal state) and the hands
are NOT mirrored but ARE read, tabulated and monitored every tick.

D052 — "the bus gets a safe first move". Every rule below is a finding from
the D051 review, all confirmed on the mock:

  * Soft entry. sim2real is refused unless the sim is at a standstill
    (`is_idle()`, the cockpit passes: state NORMAL, cmd_v zero, no gesture)
    and streaming. Entering reads each leg's present pose, parks the goal
    THERE, drops TORQUE_LIMIT to 40 % and ACC to a gentle ramp, enables
    torque, and blends real -> sim with a smoothstep over
    max(BLEND_S, 1.5 * gap / (0.8 * entry speed)) at ENTRY_SPEED_CPS
    (never 0 = servo max), then releases to the configured torque limit /
    ACC / speed after max(ENTRY_S, blend + ENTRY_TAIL_S). A leg that re-joins the
    stream later (re-armed) gets the same entry. Before D052 the first tick
    streamed the sim pose at servo max speed and full torque.
    V2 (review): the parked goal is clamped half a margin inside the range
    apply_limits burns (soft +- MARGIN_DEG), so a leg resting outside it is
    moved in at entry speed and 40 % torque instead of being parked on a goal
    the servo refuses (angle-limit error -> the monitor limps the leg; the
    mock only clamps silently, so that case is VERIFY-ON-BENCH). A leg that
    LEFT the stream (degraded, cut, lost, torque off) is marked needs-entry:
    it is never streamed again until an entry re-joins it — before, a re-arm
    of a degraded leg snapped it to the sim pose at full torque. A leg whose
    entry was interrupted keeps its 40 % / ACC-10 limits only until the
    mirror drops or it is re-armed (both release them) — they used to stick.
  * Locomotion. `allow_locomotion` is False in sim2real (and during any
    blend) until real foot contacts are wired — the sim must not walk a real
    leg around the bench; the playground zeroes cmd_v when it says so.
  * Tolerant telemetry (bus.read_telemetry): one silent servo no longer blanks
    the frame. Per-id miss counters; a leg silent for DEGRADE_TICKS
    consecutive ticks leaves the mirror (event "degraded"); a servo missing
    from LOST_POLLS monitor polls is tripped as "lost" (event "lost").
  * Rates (V2): the stream runs at the bus rate (rocky_model.bus_hz(), 50 Hz
    — what ServoModel, the envs and the righter plan against; it was 25 Hz,
    so the real legs got every other target plus 0-40 ms of extra delay);
    telemetry is read every TEL_EVERY-th tick (25 Hz) — a full read of 20
    servos is the expensive half of a tick.
  * Faults act on LEGS: a fault bit or over-temperature seen in ANY telemetry
    frame (25 Hz, not only the 2 Hz monitor poll) limps the whole leg (all
    three servos) and drops it from the mirror (event "cut"). Nothing but an
    explicit `torque(True, ids)` naming the ids re-arms a trip — not a mirror
    change, not a rescan.
  * NaN guard: push_targets keeps the last good targets of any leg whose three
    values are not all finite (event "nan"); a frame never carries a
    non-finite value (the driver raises; the tick skips that leg and counts).
  * Rate limit at the bus: a goal that moves more than hard_speed * dt from
    the last sent goal is capped (default) or refused; hard speed from
    rocky_model.servo_speed('hard') = 4.7 rad/s (event "rate", <= 1/s).
  * Heartbeat: sim2real with no push_targets for STALE_S drops the mirror to
    off (event "stale"; torque stays on, the legs hold).
  * Port loss: LOST_ERRORS consecutive failed ticks -> mirror off, limp if the
    wire allows, event "lost", then a reopen attempt every REOPEN_S (event
    "reopen", followed by a limp: after a loss the servo state is unknown).
  * scan / set_id / center / set_dir / apply_limits refuse unless the mirror is
    off (scan holds the bus ~5 s and changes baud; set_dir flips a streamed
    joint). set_id refuses an id already present (two servos on one id =
    the runbook's "silence after ID write").
  * jog clamps in the GAIT frame through the calibration (q = (deg-off)/dir,
    soft limits, back), joint from rocky_model.id_to_joint, claws 0..55.
  * Calibration: set_dir re-derives a stored offset from the stored jig pose
    (off' = off + (d - d') * jig); center writes the jig it was given.
  * status() copies everything under the lock (the bridge thread inserts
    keys); the mock moves no faster than the datasheet 4.7 rad/s.

Locks: `bus_lock` (RLock) serialises every bus transaction; `lock` (RLock)
guards the shared state. Order is always bus_lock -> lock, never the reverse.

Calibration model (same as bench/calibrate_centers.py and PebbleRobot):
servo_deg = dir * q_deg + offset_deg, offset = (present - CENTER) - dir*jig,
stored in bench/calibration.yaml so the bench scripts and the cockpit agree.
"""
from __future__ import annotations
import glob
import math
import os
import re
import sys
import threading
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _sub in ("driver", "gait"):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
if os.path.join(ROOT, "bench") not in sys.path:     # appended: bench script names must not shadow anything
    sys.path.append(os.path.join(ROOT, "bench"))

from rocky_driver import (FeetechBus, Family, SerialTransport, PebbleRobot,     # noqa: E402
                          make_pebble_mock, load_bus_params, load_calibration)
from rocky_driver.registers import CENTER, COUNTS, SWEEP_DEG                    # noqa: E402
import rocky_model as rm                                                        # noqa: E402
import apply_limits as limits_mod                                               # noqa: E402

CAL_PATH = os.path.join(ROOT, "bench", "calibration.yaml")
JOINTS = ("yaw", "hip", "knee")
MIRRORS = ("off", "sim2real", "real2sim")
TICK_HZ = float(rm.bus_hz())   # D052 V2: the stream runs at the bus rate the sim models (50 Hz)
TEL_EVERY = 2                  # telemetry every 2nd tick (25 Hz): the read is the costly half
MONITOR_S = 0.5                # SafetyMonitor warnings + lost-poll cadence
# D052 entry: module-level so tests can shrink them; values are the decision.
BLEND_S = 1.5                  # minimum real -> sim blend on entering sim2real
ENTRY_S = 3.0                  # minimum low-torque / low-speed window
ENTRY_SPEED_CPS = 200          # ~17.6 deg/s — the pose_check.py speed
ENTRY_TORQUE_LIMIT = 400       # 40 % of stall — the pose_check.py torque
ENTRY_ACC = 10                 # x100 counts/s^2 (~88 deg/s^2): 0.2 s to entry speed
ENTRY_TAIL_S = 1.0             # limited torque this long after the blend lands
DEGRADE_TICKS = 10             # consecutive silent ticks before a leg leaves the mirror
LOST_POLLS = 3                 # consecutive silent monitor polls before "lost"
LOST_ERRORS = 25               # consecutive failed ticks before the port is "lost"
REOPEN_S = 2.0                 # port reopen cadence after a loss
STALE_S = 2.0                  # sim2real heartbeat: no push_targets this long -> off
EVENT_EVERY_S = 1.0            # repeated nan/rate events are throttled to this
RATE_MODES = ("cap", "refuse")

_KEY_RE = re.compile(r"^(?:leg(\d)_(yaw|hip|knee)|hand(\d)_claw)$")


class Refused(RuntimeError):
    """A bridge call that was declined for safety (the message says why)."""


def serial_ports():
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


class HardwareBridge:
    def __init__(self, port="mock", baud=1_000_000, on_event=None, is_idle=None):
        """port: '/dev/ttyACM0' or 'mock'. on_event(kind, msg) is called from any
        thread (the cockpit prefixes 'hw:'). is_idle() -> bool says the sim is at
        a standstill; sim2real is refused otherwise."""
        self.port = port
        self.baud = baud
        self.on_event = on_event or (lambda kind, msg: None)
        self.is_idle = is_idle or (lambda: True)
        bp = load_bus_params()
        fams = {i: Family.STS for leg in bp["leg_ids"] for i in leg}
        fams |= {i: Family.SCS for i in bp["hand_ids"]}
        self.mock = None
        if port == "mock":
            self.mock = make_pebble_mock()
            bus = FeetechBus(self.mock, fams, timeout_s=0.02)
        else:
            bus = FeetechBus(SerialTransport(port, baud), fams)
        self.bus = bus
        self.cal = load_calibration(CAL_PATH)
        self.robot = PebbleRobot(bus, bp, self.cal)
        mon = self.robot.monitor
        mon.on_event = self._monitor_event
        mon.cut = self._cut_for
        mon.lost_after = LOST_POLLS
        mon.ids = []
        self.leg_ids = self.robot.leg_ids
        self.hand_ids = list(self.robot.hand_ids)
        self.id_label = {sid: f"leg{leg} {JOINTS[j]}" for leg, ids in enumerate(self.leg_ids) for j, sid in enumerate(ids)}
        self.id_label.update({sid: f"hand{i}" for i, sid in enumerate(self.hand_ids)})
        self.bus_lock = threading.RLock()
        self.lock = threading.RLock()
        self.present: dict[int, dict] = {}
        self.tel: dict[int, dict] = {}
        self.torque_on: set[int] = set()
        self.mirror = "off"
        self.q_out = None                       # (5,3) rad last GOOD sim targets (NaN = never good)
        self.q_in = np.zeros((5, 3))            # (5,3) rad measured (real2sim)
        self.legs_present = np.zeros(5, bool)   # all three servos answered the scan
        self.degraded = np.zeros(5, bool)       # dropped from the mirror for silence
        self.speed_cps = 0                      # 0 = servo max; the sim2real stream can be slowed
        self.torque_limit = 1000                # released-to TORQUE_LIMIT (0.1 %)
        self.acc = 0                            # released-to ACC (0 = no ramp)
        self.rate_mode = "cap"
        self.locomotion_ok = False              # flip when real foot contacts are wired
        self.errors = 0
        self.last_error = ""
        self.nan_count = 0                      # pushes with a non-finite leg
        self.nan_skips = 0                      # leg writes skipped for a non-finite value
        self.rate_capped = 0
        self.rate_refused = 0
        self.port_ok = True
        self.alive = True
        self._missed: dict[int, int] = {}       # sid -> consecutive silent ticks
        self._seen: dict[int, float] = {}       # sid -> monotonic time of the last answer
        self._last_goal: dict[int, float] = {}  # sid -> last sent goal (servo deg)
        self._blend: dict[int, dict] = {}       # leg -> soft-entry state
        self._need_entry: set[int] = set()      # legs that left the stream: no writes until an entry (V2)
        self._limited: set[int] = set()         # legs still at the entry TORQUE_LIMIT / ACC (V2)
        self._tick_n = 0
        self._t_push = -1e9
        self._t_send = time.monotonic()
        self._t_mon = 0.0
        self._t_reopen = 0.0
        self._serial_errs = 0
        self._throttle: dict[str, list] = {}    # kind -> [t_last, suppressed]
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ events
    def _emit(self, kind, msg):
        try:
            self.on_event(kind, msg)
        except Exception:                       # noqa: BLE001 — a UI callback must not kill the bus
            pass

    def _emit_throttled(self, kind, msg):
        now = time.monotonic()
        st = self._throttle.setdefault(kind, [-1e9, 0])
        if now - st[0] >= EVENT_EVERY_S:
            extra = f" (+{st[1]} more in the last {EVENT_EVERY_S:.0f} s)" if st[1] else ""
            st[0], st[1] = now, 0
            self._emit(kind, msg + extra)
        else:
            st[1] += 1

    def _monitor_event(self, kind, tel):
        sid = tel.servo_id
        label = self.id_label.get(sid, "?")
        if kind == "lost":
            leg = self._leg_of(sid)
            if leg is not None:
                with self.lock:
                    self._need_entry.add(leg)
                    self._blend.pop(leg, None)
            self._emit("lost", f"id {sid} ({label}): no answer for {LOST_POLLS} monitor polls — tripped"
                               + (f"; leg{leg} out of the mirror" if leg is not None else "")
                               + f"; re-arm with torque on [{sid}]")
            return
        detail = (f"{tel.temp_c} C {tel.voltage_v:.1f} V load {tel.load_pct:.0f}%"
                  + (f" faults {','.join(tel.faults)}" if tel.faults else ""))
        if kind == "cut":
            leg = self._leg_of(sid)
            ids = self.leg_ids[leg] if leg is not None else [sid]
            why = self.robot.monitor.reason.get(sid, "")
            self._emit("cut", f"id {sid} ({label}): {why} ({detail}) -> torque off on {ids}"
                              + (f", leg{leg} out of the mirror" if leg is not None else "")
                              + f"; re-arm with torque on [{sid}]")
            return
        self._emit(kind, f"id {sid} ({label}): {detail}")

    # ------------------------------------------------------------ id helpers
    def _leg_of(self, sid):
        """Leg index for a LEG servo, None for hands / unknown ids."""
        try:
            leg, joint = rm.id_to_joint(sid)
        except KeyError:
            return None
        return None if joint == "claw" else leg

    def _joint_frame(self, sid):
        """sid -> (leg, joint, key, family, dir, offset_deg, (lo, hi) deg). KeyError if unmapped."""
        leg, joint = rm.id_to_joint(sid)
        if joint == "claw":
            key, fam = f"hand{leg}_claw", Family.SCS
            d = self.robot.dir.get(key, self.robot.claw_dir.get(key, self.robot.claw_dir.get(leg, 1)))
            lo, hi = rm.claw_limits("deg")
        else:
            key, fam = f"leg{leg}_{joint}", Family.STS
            d = self.robot.dir.get(key, 1)
            lo, hi = self.robot.soft_limits_deg[joint]
        d = 1 if int(d) >= 0 else -1
        off_deg = self.robot.offset.get(key, 0) * SWEEP_DEG[fam] / COUNTS[fam]
        return leg, joint, key, fam, d, off_deg, (float(lo), float(hi))

    def _leg_mask(self, write=True):
        """Legs that take part in the mirror. write=True (sim2real): present,
        not degraded, no tripped servo. write=False (real2sim): present, not
        degraded, no LOST servo (a cut leg is limp — reading it is the point)."""
        with self.lock:
            m = self.legs_present & ~self.degraded
            bad = set(self.robot.monitor.tripped if write else self.robot.monitor.lost)
        for leg, ids in enumerate(self.leg_ids):
            if any(i in bad for i in ids):
                m[leg] = False
        return m

    @property
    def mirror_legs(self):
        return self._leg_mask(write=self.mirror != "real2sim")

    @property
    def allow_locomotion(self):
        """May the sim walk (cmd_v != 0)? Not while streaming to real legs until
        real contacts are wired (locomotion_ok), and never during a soft entry."""
        return self.mirror != "sim2real" or (self.locomotion_ok and not self._blend)

    def present_leg_ids(self):
        return [sid for leg, ids in enumerate(self.leg_ids) if self.legs_present[leg] for sid in ids]

    def _need_port(self):
        if not self.port_ok:
            raise Refused(f"port {self.port} lost — reopening every {REOPEN_S:.0f} s")

    def _need_off(self, what):
        if self.mirror != "off":
            raise Refused(f"{what} needs mirror off (it is {self.mirror})")

    # ------------------------------------------------------------ scan / presence
    def scan(self, bauds=None, id_range=None):
        self._need_port()
        self._need_off("scan")
        kw = {"bauds": tuple(bauds)} if bauds else ({"bauds": (1_000_000,)} if self.mock else {})
        if id_range is not None:
            kw["id_range"] = id_range
        with self.bus_lock:
            found = self.bus.scan(**kw)
            for sid in found:                  # a scan tells us nothing about families; the map does
                if sid in self.id_label:
                    self.bus.families[sid] = Family.SCS if sid in self.hand_ids else Family.STS
            with self.lock:
                self.present = found
                self.legs_present = np.array([all(sid in found for sid in ids) for ids in self.leg_ids])
                self.degraded[:] = False
                self._missed = {sid: 0 for sid in found}
                self.robot.monitor.ids = sorted(found)
                self.robot.monitor.missed = {sid: 0 for sid in found}
                self.torque_on &= set(found)
                self.tel = {k: v for k, v in self.tel.items() if k in found}
        partial = [leg for leg, ids in enumerate(self.leg_ids)
                   if any(i in found for i in ids) and not all(i in found for i in ids)]
        self._emit("scan", f"{len(found)} servos: {sorted(found)}; legs present "
                           f"{[i for i in range(5) if self.legs_present[i]]}"
                           + (f"; partial legs {partial} (monitored, not mirrored)" if partial else ""))
        return found

    # ------------------------------------------------------------ loop
    def _loop(self):
        dt = 1.0 / TICK_HZ
        t_last = time.monotonic()
        while self.alive:
            now = time.monotonic()
            el = now - t_last
            t_last = now
            if self.mock is not None:
                with self.bus_lock:
                    self.mock.advance(el)       # the mock's clock is manual
            if not self.port_ok:
                if now - self._t_reopen >= REOPEN_S:
                    self._t_reopen = now
                    self._try_reopen()
            else:
                try:
                    self._tick(now)
                    self._serial_errs = 0
                except Exception as e:          # noqa: BLE001 — serial hiccups must not kill the loop
                    self.errors += 1
                    self.last_error = f"{type(e).__name__}: {e}"
                    self._serial_errs += 1
                    if self._serial_errs >= LOST_ERRORS:
                        self._port_lost()
            time.sleep(max(0.0, dt - (time.monotonic() - now)))

    def _tick(self, now):
        with self.lock:
            present = sorted(self.present)
            mode = self.mirror
            q_out = None if self.q_out is None else self.q_out.copy()
            t_push = self._t_push
        if not present:
            return
        if mode == "sim2real" and now - t_push > STALE_S:
            self._drop_mirror("stale", f"no sim targets for {now - t_push:.1f} s — mirror off, legs hold")
            mode = "off"
        if mode == "sim2real" and q_out is not None:
            self._stream(now, q_out)
        self._tick_n += 1
        poll = now - self._t_mon >= MONITOR_S
        if not poll and self._tick_n % max(1, int(TEL_EVERY)) != 0:
            return                                # V2: stream-only tick
        lost = set(self.robot.monitor.lost)
        read_ids = present if poll else [i for i in present if i not in lost]
        with self.bus_lock:
            tels, missing = self.bus.read_telemetry(read_ids)
            with self.lock:
                self.robot.monitor.ids = present
                self.robot.monitor.check(tels, missing if poll else None, warn=poll)
        if poll:
            self._t_mon = now
        self._account(now, tels, missing)

    def _stream(self, now, q_out):
        """One sim2real frame: blend, NaN guard, soft limits (q_to_deg), rate limit."""
        mask = self._leg_mask(write=True)
        with self.lock:
            blend = {k: dict(v) for k, v in self._blend.items()}
            need = set(self._need_entry)
            dt = min(max(now - self._t_send, 1e-3), 0.1)
        entry, run, release, sent = {}, {}, [], {}
        for leg in range(5):
            if not mask[leg] or (leg in need and leg not in blend):
                continue
            q = np.array(q_out[leg], float)
            try:
                degs = [self.robot.q_to_deg(leg, j, float(q[j])) for j in range(3)]
            except ValueError:                                   # NaN never reaches the bus (D052)
                with self.lock:
                    self.nan_skips += 1                          # (a leg with no good target yet lands here too)
                continue
            b = blend.get(leg)
            if b is not None:
                # blend in SERVO degrees from where the joint really is to the
                # soft-limited target: a start outside the soft range (a limp
                # knee resting at 0) walks in instead of jumping to the limit
                u = min(1.0, max(0.0, (now - b["t0"]) / b["T"]))
                s = u * u * (3.0 - 2.0 * u)                      # smoothstep: no velocity step at either end
                degs = [b["from"][j] + s * (degs[j] - b["from"][j]) for j in range(3)]
                if now - b["t0"] >= b["hold"]:
                    release.append(leg)
            if not all(math.isfinite(x) for x in degs):
                with self.lock:
                    self.nan_skips += 1
                continue
            for j, sid in enumerate(self.leg_ids[leg]):
                deg = self._rate_limit(sid, degs[j], dt)
                if deg is None:
                    continue
                (entry if (b is not None and leg not in release) else run)[sid] = deg
                sent[sid] = deg
        with self.bus_lock:
            if entry:
                self.bus.sync_positions(entry, ENTRY_SPEED_CPS)
            if run:
                self.bus.sync_positions(run, self.speed_cps)
            if release:
                ids = [sid for leg in release for sid in self.leg_ids[leg]]
                self._release_limits(ids)
        with self.lock:
            self._last_goal.update(sent)
            self._t_send = now
            for leg in release:
                self._blend.pop(leg, None)
                self._limited.discard(leg)
        if release:
            self._emit("entry", f"legs {release}: soft entry done — torque limit {self.torque_limit / 10:.0f} %, "
                                f"speed {'max' if not self.speed_cps else self.speed_cps}")

    def _rate_limit(self, sid, deg, dt):
        """Last line at the bus (D052): cap (or refuse) a goal step bigger than
        the servo's hard speed could cover in dt. Returns the deg to send or None."""
        with self.lock:
            last = self._last_goal.get(sid)
            if last is None:
                t = self.tel.get(sid)
                last = None if t is None or t.get("pos_deg") is None else t["pos_deg"]
        if last is None:
            return None                                  # nothing to measure a step against: wait a tick
        cap = math.degrees(rm.servo_speed("hard")) * dt
        step = deg - last
        if abs(step) <= cap:
            return deg
        with self.lock:
            if self.rate_mode == "refuse":
                self.rate_refused += 1
            else:
                self.rate_capped += 1
        self._emit_throttled("rate", f"id {sid} ({self.id_label.get(sid, '?')}): goal step {step:+.1f} deg "
                                     f"> {cap:.1f} deg in {dt * 1000:.0f} ms — "
                                     + ("refused" if self.rate_mode == "refuse" else "capped"))
        if self.rate_mode == "refuse":
            return None
        return last + math.copysign(cap, step)

    def _release_limits(self, ids):
        """bus_lock held: back to the configured torque limit / ACC."""
        self.bus.sync_write_reg("TORQUE_LIMIT", {sid: self.torque_limit for sid in ids})
        self.bus.sync_write_reg("ACC", {sid: self.acc for sid in ids})

    def _account(self, now, tels, missing):
        """Counters, degraded legs, q_in and the status table — every present id."""
        events = []
        with self.lock:
            q_in = self.q_in.copy()
            for sid in missing:
                self._missed[sid] = self._missed.get(sid, 0) + 1
            for sid, tel in tels.items():
                self._missed[sid] = 0
                self._seen[sid] = now
            for leg, ids in enumerate(self.leg_ids):
                if self.legs_present[leg] and not self.degraded[leg]:
                    silent = [i for i in ids if self._missed.get(i, 0) >= DEGRADE_TICKS]
                    if silent:
                        self.degraded[leg] = True
                        self._blend.pop(leg, None)
                        self._need_entry.add(leg)
                        events.append(("degraded", f"leg{leg}: id {silent} silent for {DEGRADE_TICKS} ticks — "
                                                   f"out of the mirror (servos hold their last goal); "
                                                   f"re-arm with torque on {ids} or rescan"))
            tripped = set(self.robot.monitor.tripped)
            reason = dict(self.robot.monitor.reason)
            for sid in sorted(self.present):
                tel = tels.get(sid)
                row = self.tel.get(sid) or dict(id=sid, label=self.id_label.get(sid, "?"),
                                                kind="hand" if sid in self.hand_ids else
                                                ("leg" if sid in self.id_label else "unmapped"),
                                                pos_deg=None, q_deg=None, load=None, volt=None, temp=None,
                                                current=None, moving=False, faults=[])
                if tel is not None:
                    q_deg = None
                    try:
                        leg, joint, key, fam, d, off, _ = self._joint_frame(sid)
                        q = (tel.position_deg - off) / d
                        q_deg = round(q, 2)
                        if joint != "claw":
                            q_in[leg][JOINTS.index(joint)] = math.radians(q)
                    except KeyError:
                        pass
                    row.update(pos_deg=round(tel.position_deg, 2), q_deg=q_deg, load=round(tel.load_pct, 1),
                               volt=round(tel.voltage_v, 2), temp=tel.temp_c,
                               current=None if tel.current_a is None else round(tel.current_a, 2),
                               moving=tel.moving, faults=list(tel.faults))
                seen = self._seen.get(sid)
                row.update(torque=sid in self.torque_on, missed=self._missed.get(sid, 0),
                           age_s=None if seen is None else round(now - seen, 2),
                           tripped=sid in tripped, reason=reason.get(sid))
                self.tel[sid] = row
            self.q_in = q_in
        for kind, msg in events:
            self._emit(kind, msg)

    # ------------------------------------------------------------ faults / loss
    def _cut_for(self, sid, tel):
        """SafetyMonitor.cut (bus_lock held): limp the WHOLE leg, not one servo of it."""
        leg = self._leg_of(sid)
        ids = list(self.leg_ids[leg]) if leg is not None else [sid]
        self.bus.torque_all(ids, False)
        with self.lock:
            self.torque_on.difference_update(ids)
            if leg is not None:
                self._blend.pop(leg, None)
                self._need_entry.add(leg)

    def _drop_mirror(self, kind, msg):
        with self.lock:
            prev = self.mirror
            self.mirror = "off"
            legs = sorted(set(self._blend) | self._limited)
            self._blend.clear()
            self._need_entry.clear()            # the next entry blends every mirror leg anyway
        if prev == "sim2real" and legs:
            try:
                with self.bus_lock:
                    self._release_limits([sid for leg in legs for sid in self.leg_ids[leg]])
                with self.lock:
                    self._limited.difference_update(legs)
            except Exception:                   # noqa: BLE001 — the drop matters more than the restore
                pass
        self._emit(kind, msg)

    def _port_lost(self):
        with self.lock:
            self.mirror = "off"
            self._blend.clear()
            self.port_ok = False
            ids = sorted(self.present)
        self._t_reopen = time.monotonic()
        ok = self._try_limp(ids)
        self._emit("lost", f"port {self.port}: {LOST_ERRORS} failed ticks ({self.last_error}) — mirror off, "
                           f"limp {'sent' if ok else 'FAILED (servos hold their last goal)'}; "
                           f"reopening every {REOPEN_S:.0f} s")

    def _try_limp(self, ids):
        if not ids:
            return True
        try:
            with self.bus_lock:
                self.bus.torque_all(ids, False)
            with self.lock:
                self.torque_on.difference_update(ids)
            return True
        except Exception:                       # noqa: BLE001
            return False

    def _open_transport(self):
        if self.mock is not None:
            return self.mock
        return SerialTransport(self.port, self.baud)

    def _try_reopen(self):
        try:
            t = self._open_transport()
        except Exception as e:                  # noqa: BLE001 — still unplugged
            self.last_error = f"reopen: {type(e).__name__}: {e}"
            return False
        with self.bus_lock:
            old = self.bus.t
            if old is not t:
                try:
                    old.close()
                except Exception:               # noqa: BLE001
                    pass
            self.bus.t = t
            self.bus._rxbuf.clear()
        self._serial_errs = 0
        self.port_ok = True
        ok = self._try_limp(sorted(self.present))
        self._emit("reopen", f"port {self.port} reopened — limp {'sent' if ok else 'failed'}; "
                             "rescan / re-arm before mirroring")
        return True

    # ------------------------------------------------------------ sim-side API (sim thread)
    def push_targets(self, q):
        """sim thread: the joint targets this step (5,3) rad. A leg whose three
        values are not all finite keeps its last good targets (event "nan")."""
        q = np.asarray(q, float).reshape(5, 3)
        good = np.isfinite(q).all(axis=1)
        with self.lock:
            if self.q_out is None:
                self.q_out = np.full((5, 3), np.nan)
            self.q_out[good] = q[good]
            self._t_push = time.monotonic()
            if not good.all():
                self.nan_count += 1
                self.errors += 1
        if not good.all():
            self._emit_throttled("nan", f"non-finite targets for legs {np.flatnonzero(~good).tolist()} — "
                                        "holding their last good targets")

    def real_pose(self):
        """sim thread: (q[5,3] rad measured, legs[5]) for real2sim."""
        mask = self._leg_mask(write=False)
        with self.lock:
            return self.q_in.copy(), mask

    # ------------------------------------------------------------ controls (any thread)
    def set_mirror(self, mode):
        if mode not in MIRRORS:
            raise ValueError(f"mirror must be one of {MIRRORS}")
        self._need_port()
        if mode == "sim2real":
            if self.mirror == "sim2real":
                return mode
            if not self.is_idle():
                raise Refused("sim2real needs the sim at a standstill (state NORMAL, no velocity "
                              "command, no gesture)")
            with self.lock:
                fresh = self.q_out is not None and time.monotonic() - self._t_push < STALE_S
            if not fresh:
                raise Refused("the sim is not streaming targets")
            mask = self._leg_mask(write=True)
            if not mask.any():
                raise Refused("no complete, untripped leg on the bus (scan; re-arm tripped ids)")
            legs = self._start_blend([leg for leg in range(5) if mask[leg]])
            if not legs:
                raise Refused("no mirror leg answered the entry read")
            with self.lock:
                self.mirror = mode
                T = max(b["T"] for b in self._blend.values())
            self._emit("mirror", f"sim2real on legs {legs}: soft entry {T:.1f} s at {ENTRY_SPEED_CPS} cps, "
                                 f"torque limit {ENTRY_TORQUE_LIMIT / 10:.0f} %; locomotion "
                                 f"{'allowed' if self.locomotion_ok else 'held (no real contacts)'}")
            return mode
        with self.lock:
            prev = self.mirror
        if prev == "sim2real":
            self._drop_mirror("mirror", mode)
            with self.lock:
                self.mirror = mode
        else:
            with self.lock:
                self.mirror = mode
            self._emit("mirror", mode)
        return mode

    def _start_blend(self, legs):
        """Park each leg's goal at its present pose, limited torque, then torque
        on; the stream blends from there. Returns the legs that answered."""
        now = time.monotonic()
        ids = [sid for leg in legs for sid in self.leg_ids[leg]]
        with self.bus_lock:
            tels, _ = self.bus.read_telemetry(ids, single_retries=1)
            ok_legs = [leg for leg in legs if all(sid in tels for sid in self.leg_ids[leg])]
            ok_ids = [sid for leg in ok_legs for sid in self.leg_ids[leg]]
            if not ok_ids:
                return []
            here = {sid: self._park_deg(sid, tels[sid].position_deg) for sid in ok_ids}
            self.bus.sync_write_reg("TORQUE_LIMIT", {sid: ENTRY_TORQUE_LIMIT for sid in ok_ids})
            self.bus.sync_write_reg("ACC", {sid: ENTRY_ACC for sid in ok_ids})
            self.bus.sync_positions(here, ENTRY_SPEED_CPS)
            self.bus.torque_all(ok_ids, True)
        v = 0.8 * ENTRY_SPEED_CPS * 360.0 / 4096.0          # deg/s the servo can surely follow
        with self.lock:
            self.torque_on.update(ok_ids)
            self._last_goal.update(here)
            self._limited.update(ok_legs)
            self._need_entry.difference_update(ok_legs)
            q_sim = None if self.q_out is None else self.q_out.copy()
            for leg in ok_legs:
                d_from = [here[self.leg_ids[leg][j]] for j in range(3)]
                gap = 0.0
                if q_sim is not None and np.isfinite(q_sim[leg]).all():
                    tgt = [self.robot.q_to_deg(leg, j, float(q_sim[leg][j])) for j in range(3)]
                    gap = max(abs(tgt[j] - d_from[j]) for j in range(3))
                # the sim is idle, so the target is (nearly) fixed: size the blend
                # so a 200 cps servo keeps up — smoothstep peaks at 1.5x its mean rate
                T = max(BLEND_S, 1.5 * gap / v)
                self._blend[leg] = dict(t0=now, T=T, hold=max(ENTRY_S, T + ENTRY_TAIL_S), gap=gap,
                                        **{"from": d_from})
        return ok_legs

    def _park_deg(self, sid, pos_deg):
        """V2: the goal to park a servo on at entry — its present position,
        clamped half a margin inside the range apply_limits burns (soft limits
        +- MARGIN_DEG, through the calibration). Unmapped ids: unchanged."""
        try:
            _leg, _joint, _key, _fam, d, off, (lo, hi) = self._joint_frame(sid)
        except KeyError:
            return pos_deg
        m = 0.5 * limits_mod.MARGIN_DEG
        q = min(hi + m, max(lo - m, (pos_deg - off) / d))
        return d * q + off

    def _hold_and_enable(self, ids):
        """Torque on WITHOUT a jump: goal := present at entry speed first (a
        Feetech servo enables toward its LAST goal, which may be anywhere).
        V2: a leg still at the entry limits (an interrupted entry) is released."""
        with self.bus_lock:
            tels, _ = self.bus.read_telemetry(ids, single_retries=1)
            ok = [i for i in ids if i in tels]
            if ok:
                here = {i: self._park_deg(i, tels[i].position_deg) for i in ok}
                self.bus.sync_positions(here, ENTRY_SPEED_CPS)
                self.bus.torque_all(ok, True)
            with self.lock:
                lim = [leg for leg in self._limited if any(i in ok for i in self.leg_ids[leg])]
            if lim:
                self._release_limits([sid for leg in lim for sid in self.leg_ids[leg]])
        with self.lock:
            self.torque_on.update(ok)
            self._limited.difference_update(lim)
            if ok:
                self._last_goal.update(here)
        return ok

    def torque(self, on: bool, ids=None):
        """Torque on/off. on=False, ids=None: every present servo. on=True,
        ids=None: every complete, untripped leg. on=True WITH ids: those ids,
        and this is the ONLY call that re-arms a tripped / lost servo and puts
        a degraded leg back in the mirror (D052)."""
        self._need_port()
        with self.lock:
            present = set(self.present)
        if not on:
            ids = [int(i) for i in ids] if ids is not None else sorted(present)
            ids = [i for i in ids if i in present]
            if not ids:
                return []
            with self.bus_lock:
                self.bus.torque_all(ids, False)
            with self.lock:
                self.torque_on.difference_update(ids)
                for i in ids:
                    leg = self._leg_of(i)
                    if leg is not None:
                        self._blend.pop(leg, None)
                        self._need_entry.add(leg)       # V2: back only through an entry
            return ids
        explicit = ids is not None
        if explicit:
            ids = [int(i) for i in ids if int(i) in present]
            with self.bus_lock, self.lock:
                rearmed = self.robot.monitor.rearm(ids)
                for leg, lids in enumerate(self.leg_ids):
                    if any(i in ids for i in lids):
                        self.degraded[leg] = False
                        for i in lids:
                            self._missed[i] = 0
            if rearmed:
                self._emit("rearm", f"ids {rearmed} re-armed")
        else:
            mask = self._leg_mask(write=True)
            ids = [sid for leg in range(5) if mask[leg] for sid in self.leg_ids[leg]]
        if not ids:
            return []
        if self.mirror == "sim2real":                    # a leg joining the stream gets a soft entry
            mask = self._leg_mask(write=True)
            with self.lock:
                on = set(self.torque_on)
                need = set(self._need_entry)
                blending = set(self._blend)
            # V2: an explicit re-arm always re-enters every mirror leg it touches
            # (a degraded / lost leg keeps its torque on, so "torque already on"
            # used to skip the entry and the next tick snapped it at full torque)
            join = [leg for leg in range(5) if mask[leg] and leg not in blending
                    and (any(i in ids for i in self.leg_ids[leg]) if explicit
                         else (leg in need or not all(i in on for i in self.leg_ids[leg])))]
            joined = [sid for leg in self._start_blend(join) for sid in self.leg_ids[leg]] if join else []
            rest = [i for i in ids if i not in joined]
            return joined + (self._hold_and_enable(rest) if rest else [])
        return self._hold_and_enable(ids)

    def limp(self):
        self._need_port()                    # a lost port already attempted its own limp
        with self.lock:
            self.mirror = "off"
            self._blend.clear()
            self._need_entry.clear()
            ids = sorted(self.present)
        if not ids:
            self._emit("limp", "nothing present")
            return []
        with self.bus_lock:
            self.bus.torque_all(ids, False)
        with self.lock:
            self.torque_on.difference_update(ids)
        self._emit("limp", f"torque off on {len(ids)} servos")
        return ids

    def jog(self, sid: int, deg: float, speed_cps: int = 200):
        """Move ONE servo to a center-relative servo angle (deg) — the bench nudge.
        The clamp happens in the GAIT frame through the calibration (D052):
        q = (deg - off)/dir, clip to the soft limits (claws 0..55), back to servo
        deg. Returns the servo deg actually commanded. Drops a sim2real stream."""
        self._need_port()
        sid = int(sid)
        try:
            leg, joint, key, fam, d, off, (lo, hi) = self._joint_frame(sid)
        except KeyError:
            raise Refused(f"id {sid} is not in the params bus map") from None
        with self.lock:
            if sid not in self.present:
                raise Refused(f"id {sid} not present (scan first)")
        if sid in self.robot.monitor.tripped:
            raise Refused(f"id {sid} is tripped ({self.robot.monitor.reason.get(sid)}); re-arm with torque on [{sid}]")
        deg = float(deg)
        if not math.isfinite(deg):
            raise ValueError(f"non-finite jog target {deg!r}")
        if self.mirror == "sim2real":                    # a jog takes the servo away from the sim stream
            self._drop_mirror("mirror", f"off (jog id {sid})")
        q = min(hi, max(lo, (deg - off) / d))
        out = d * q + off
        speed = int(speed_cps) if speed_cps and int(speed_cps) > 0 else ENTRY_SPEED_CPS   # a nudge is never servo-max
        with self.bus_lock:
            self.bus.set_position(sid, out, speed)       # goal first, then torque: no snap to an old goal
            if sid not in self.torque_on:
                self.bus.torque(sid, True)
        with self.lock:
            self.torque_on.add(sid)
            self._last_goal[sid] = out
        return out

    def set_id(self, old: int, new: int):
        self._need_port()
        self._need_off("set_id")
        old, new = int(old), int(new)
        with self.lock:
            present = set(self.present)
        if new in present:
            raise Refused(f"id {new} is already on the bus — two servos on one id go silent "
                          "(runbook: 'silence after ID write'); pick a free id")
        if old not in present:
            raise Refused(f"id {old} not present (scan first)")
        if not (1 <= new <= 253):
            raise ValueError(f"bad new id {new}")
        with self.bus_lock:
            self.bus.set_id(old, new)
            if new in self.id_label:
                self.bus.families[new] = Family.SCS if new in self.hand_ids else Family.STS
        with self.lock:
            self.present[new] = self.present.pop(old, {})
            self.tel.pop(old, None)
            self.torque_on.discard(old)
            self.legs_present = np.array([all(s in self.present for s in ids) for ids in self.leg_ids])
        self._emit("set_id", f"{old} -> {new}")
        return new

    # ------------------------------------------------------------ calibration
    def _save_cal(self):
        os.makedirs(os.path.dirname(CAL_PATH), exist_ok=True)
        with open(CAL_PATH, "w") as f:
            yaml.safe_dump(self.cal, f, sort_keys=True)
        self.robot.dir = {k: int(v) for k, v in self.cal.get("dir", {}).items()}
        self.robot.offset = {k: int(v) for k, v in self.cal.get("offset", {}).items()}
        self.robot.claw_dir = self.cal.get("claw_dir", {})

    @staticmethod
    def _parse_key(key):
        m = _KEY_RE.match(str(key))
        if not m:
            raise ValueError(f"calibration key must be leg<i>_<yaw|hip|knee> or hand<i>_claw, got {key!r}")
        if m.group(3) is not None:
            return int(m.group(3)), "claw", Family.SCS
        return int(m.group(1)), m.group(2), Family.STS

    def _jig_for(self, key, joint):
        meta = self.cal.get("meta", {}) or {}
        per = (meta.get("jig_deg") or {}).get(key)
        if per is not None:
            return float(per)
        return float(meta.get("knee_jig_deg", -90.0)) if joint == "knee" else 0.0

    def center(self, key: str, jig_deg: float | None = None):
        """Torque OFF that joint, read where it rests at the jig pose, store
        offset = (present - CENTER) - dir*jig and the jig itself."""
        self._need_port()
        self._need_off("center")
        leg, joint, fam = self._parse_key(key)
        sid = rm.joint_to_id(leg, joint)
        with self.lock:
            if sid not in self.present:
                raise Refused(f"{key} (id {sid}) not present")
        if jig_deg is None:
            jig_deg = -90.0 if joint == "knee" else 0.0
        jig_deg = float(jig_deg)
        with self.bus_lock:
            if fam is Family.STS:
                eo = self.bus.read_reg(sid, "POSITION_OFFSET")
                if eo:
                    raise Refused(f"{key}: EEPROM POSITION_OFFSET is {eo:+d} — a software offset on top "
                                  "would double-correct (D052: one place only)")
            self.bus.torque(sid, False)
            pos = self.bus.read_reg(sid, "PRESENT_POSITION")
        with self.lock:
            self.torque_on.discard(sid)
        d = int(self.cal.setdefault("dir", {}).get(key, 1))
        jig_counts = round(jig_deg * COUNTS[fam] / SWEEP_DEG[fam])
        offset = int((pos - CENTER[fam]) - d * jig_counts)
        self.cal.setdefault("offset", {})[key] = offset
        meta = self.cal.setdefault("meta", {})
        meta.setdefault("jig_deg", {})[key] = jig_deg
        if joint == "knee":
            meta["knee_jig_deg"] = jig_deg          # D052: the jig actually used, not a constant
        self._save_cal()
        self._emit("center", f"{key}: present {pos} at jig {jig_deg:+.1f} deg -> offset {offset:+d} counts")
        return dict(key=key, present=pos, jig_deg=jig_deg, offset=offset,
                    offset_deg=round(offset * SWEEP_DEG[fam] / COUNTS[fam], 2))

    def set_dir(self, key: str, d: int):
        """Set a joint's direction sign. A stored offset was derived with the old
        sign; it is re-derived from the stored jig (off' = off + (d - d') * jig)
        rather than silently left wrong (D052)."""
        self._need_off("set_dir")
        leg, joint, fam = self._parse_key(key)
        d_new = 1 if int(d) >= 0 else -1
        dirs = self.cal.setdefault("dir", {})
        d_old = 1 if int(dirs.get(key, 1)) >= 0 else -1
        dirs[key] = d_new
        note = ""
        offs = self.cal.setdefault("offset", {})
        if d_new != d_old and key in offs:
            jig = self._jig_for(key, joint)
            jig_counts = round(jig * COUNTS[fam] / SWEEP_DEG[fam])
            off_old = int(offs[key])
            offs[key] = int(off_old + (d_old - d_new) * jig_counts)
            note = (f"; offset re-derived {off_old:+d} -> {offs[key]:+d} counts from the stored jig "
                    f"{jig:+.1f} deg")
        self._save_cal()
        self._emit("dir", f"{key}: dir {d_old:+d} -> {d_new:+d}{note}")
        return d_new

    def apply_limits(self, ids=None, margin_deg: float = limits_mod.MARGIN_DEG, dry_run: bool = False):
        """Burn MIN/MAX_ANGLE_LIMIT from the params limits + calibration (see
        bench/apply_limits.py). ids=None -> every present mapped id. Returns
        {id: result}: dry_run -> the plan entries; else {ok, min, max, error}."""
        self._need_port()
        self._need_off("apply_limits")
        with self.lock:
            present = sorted(self.present)
        ids = [int(i) for i in (ids if ids is not None else present) if int(i) in self.id_label]
        ids = [i for i in ids if i in present]
        if not ids:
            raise Refused("no mapped servo present (scan first)")
        plan = limits_mod.plan(self.cal, ids, margin_deg)
        if dry_run:
            return plan
        lines = []
        with self.bus_lock:
            res = limits_mod.apply(self.bus, plan, log=lines.append)
        bad = [i for i, r in res.items() if not r["ok"]]
        self._emit("limits", f"{len(res) - len(bad)}/{len(res)} servos carry hardware limits "
                             f"(margin {margin_deg:.1f} deg)" + (f"; NOT applied {bad}" if bad else ""))
        return res

    # ------------------------------------------------------------ status
    def status(self):
        """A snapshot safe to JSON from any thread (copied under the lock)."""
        mask_w, mask_r = self._leg_mask(True), self._leg_mask(False)
        now = time.monotonic()
        with self.lock:
            mon = self.robot.monitor
            blend = {str(leg): dict(t=round(now - b["t0"], 2), blend_s=round(b["T"], 2),
                                    hold_s=round(b["hold"], 2), gap_deg=round(b["gap"], 1),
                                    frac=round(min(1.0, (now - b["t0"]) / b["T"]), 2))
                     for leg, b in self._blend.items()}
            return dict(
                port=self.port, mock=self.mock is not None, port_ok=self.port_ok, mirror=self.mirror,
                legs=[bool(x) for x in self.legs_present],
                mirror_legs=[bool(x) for x in (mask_r if self.mirror == "real2sim" else mask_w)],
                degraded=[bool(x) for x in self.degraded],
                present=sorted(self.present), n_present=len(self.present),
                tel=[dict(self.tel[i]) for i in sorted(self.tel)],
                torque=sorted(self.torque_on),
                tripped=sorted(mon.tripped), trip_reason={str(k): v for k, v in sorted(mon.reason.items())},
                lost=sorted(mon.lost),
                entry=blend or None, allow_locomotion=self.allow_locomotion, locomotion_ok=self.locomotion_ok,
                stale_s=None if self._t_push < 0 else round(now - self._t_push, 2),
                errors=self.errors, last_error=self.last_error, nan_count=self.nan_count,
                nan_skips=self.nan_skips, rate_mode=self.rate_mode, rate_capped=self.rate_capped,
                rate_refused=self.rate_refused,
                cal=dict(dir=dict(self.cal.get("dir", {}) or {}), offset=dict(self.cal.get("offset", {}) or {})),
                speed_cps=self.speed_cps, torque_limit=self.torque_limit, acc=self.acc,
                bus_stats=dict(self.bus.stats))

    def close(self):
        self.alive = False
        self.thread.join(timeout=1.0)
        if self.port_ok:
            try:
                self.limp()
            except Exception:                   # noqa: BLE001
                pass
        try:
            self.bus.t.close()
        except Exception:                       # noqa: BLE001
            pass
