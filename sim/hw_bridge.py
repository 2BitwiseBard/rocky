"""Hardware bridge (D051) — the real servo bus beside the sim, one leg at a time.

The cockpit's Hardware panel drives this. It opens the Feetech bus
(`/dev/ttyACM0`, or `mock` for the byte-level simulator the bench
scripts rehearse on), scans for servos, and mirrors between the sim and
whatever legs answered:

    mirror = "sim2real"   the sim's joint targets are streamed to every
                          PRESENT leg (all three of its servos answered
                          the scan) at the bus rate — the real leg does
                          what the sim leg does: gaits, gestures, reflexes
    mirror = "real2sim"   the present legs' measured positions drive the
                          sim's corresponding joints — hold the real leg
                          and the sim leg follows (calibration checks,
                          range checks, "does the IK frame match?")
    mirror = "off"        telemetry only

Legs that are not on the bus stay analytic in the sim, so a robot with
one leg built is a robot with one real leg and four simulated ones.

Safety: the driver's soft limits clamp every command (PebbleRobot.q_to_deg);
the SafetyMonitor polls at 2 Hz and torque-cuts on temperature or fault
flags; `limp()` drops torque everywhere and is what the cockpit's stop
does when the bridge is live. Only the bridge thread and API calls touch
the bus, and every bus call takes `bus_lock`.

Calibration: `center(key)` and `set_dir(key, ±1)` implement the same
model as bench/calibrate_centers.py (offset = present - center - dir*jig)
and write bench/calibration.yaml, so the bench scripts and the cockpit
agree on where zero is.
"""
from __future__ import annotations
import glob
import os
import sys
import threading
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "driver"))

from rocky_driver import (FeetechBus, Family, SerialTransport, PebbleRobot,     # noqa: E402
                          make_pebble_mock, load_bus_params, load_calibration, BusError)
from rocky_driver.registers import CENTER, COUNTS, SWEEP_DEG                    # noqa: E402

CAL_PATH = os.path.join(ROOT, "bench", "calibration.yaml")
JOINTS = ("yaw", "hip", "knee")
MIRRORS = ("off", "sim2real", "real2sim")
TICK_HZ = 25.0                 # mirror/telemetry loop
MONITOR_S = 0.5                # SafetyMonitor cadence


def serial_ports():
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


class HardwareBridge:
    def __init__(self, port="mock", baud=1_000_000, on_event=None):
        self.port = port
        self.on_event = on_event or (lambda kind, msg: None)
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
        self.robot.monitor.on_event = self._monitor_event
        self.leg_ids = self.robot.leg_ids
        self.id_label = {sid: f"leg{leg} {JOINTS[j]}" for leg, ids in enumerate(self.leg_ids) for j, sid in enumerate(ids)}
        self.id_label.update({sid: f"hand{i}" for i, sid in enumerate(self.robot.hand_ids)})
        self.bus_lock = threading.RLock()
        self.lock = threading.Lock()
        self.present: dict[int, dict] = {}
        self.tel: dict[int, dict] = {}
        self.torque_on: set[int] = set()
        self.mirror = "off"
        self.q_out = None                       # (5,3) rad from the sim (sim2real)
        self.q_in = np.zeros((5, 3))            # (5,3) rad measured (real2sim)
        self.legs_present = np.zeros(5, bool)
        self.speed_cps = 0                      # 0 = servo max; the sim2real stream can be slowed
        self.errors = 0
        self.last_error = ""
        self.alive = True
        self._t_mon = 0.0
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ events
    def _monitor_event(self, kind, tel):
        if kind == "cut":
            self.torque_on.discard(tel.servo_id)
        self.on_event(kind, f"id {tel.servo_id} ({self.id_label.get(tel.servo_id, '?')}): "
                            f"{tel.temp_c} C {tel.voltage_v:.1f} V load {tel.load_pct:.0f}%"
                            + (f" faults {','.join(tel.faults)}" if tel.faults else ""))

    # ------------------------------------------------------------ scan / presence
    def scan(self, bauds=None):
        with self.bus_lock:
            kw = {"bauds": tuple(bauds)} if bauds else ({"bauds": (1_000_000,)} if self.mock else {})
            found = self.bus.scan(**kw)
        with self.lock:
            self.present = found
            self.legs_present = np.array([all(sid in found for sid in ids) for ids in self.leg_ids])
        self.on_event("scan", f"{len(found)} servos: {sorted(found)}; legs present "
                              f"{[i for i in range(5) if self.legs_present[i]]}")
        return found

    def present_leg_ids(self):
        return [sid for leg, ids in enumerate(self.leg_ids) if self.legs_present[leg] for sid in ids]

    # ------------------------------------------------------------ mirror loop
    def _loop(self):
        dt = 1.0 / TICK_HZ
        t_last = time.monotonic()
        while self.alive:
            now = time.monotonic()
            el = now - t_last
            t_last = now
            if self.mock is not None:
                self.mock.advance(el)               # the mock's clock is manual
            try:
                self._tick(now)
            except BusError as e:
                self.errors += 1
                self.last_error = str(e)
            except Exception as e:                  # serial hiccups must not kill the loop
                self.errors += 1
                self.last_error = f"{type(e).__name__}: {e}"
            time.sleep(max(0.0, dt - (time.monotonic() - now)))

    def _tick(self, now):
        ids = self.present_leg_ids()
        if not ids:
            return
        with self.lock:
            mode = self.mirror
            q_out = None if self.q_out is None else self.q_out.copy()
        if mode == "sim2real" and q_out is not None:
            targets = {}
            for leg in range(5):
                if not self.legs_present[leg]:
                    continue
                for j in range(3):
                    targets[self.leg_ids[leg][j]] = self.robot.q_to_deg(leg, j, q_out[leg][j])
            with self.bus_lock:
                self.bus.sync_positions(targets, self.speed_cps)
        with self.bus_lock:
            if now - self._t_mon >= MONITOR_S:
                self._t_mon = now
                self.robot.monitor.ids = ids
                tels = self.robot.monitor.step()
            else:
                tels = self.bus.sync_telemetry(ids)
        q_in = self.q_in.copy()
        for sid, tel in tels.items():
            leg, j = divmod(sid - 1, 3)
            if 0 <= leg < 5:
                q_in[leg][j] = self.robot.deg_to_q(leg, j, tel.position_deg)
            self.tel[sid] = dict(id=sid, label=self.id_label.get(sid, "?"), pos_deg=round(tel.position_deg, 2),
                                 q_deg=round(float(np.degrees(q_in[leg][j])), 2) if 0 <= leg < 5 else None,
                                 load=round(tel.load_pct, 1), volt=round(tel.voltage_v, 2), temp=tel.temp_c,
                                 current=None if tel.current_a is None else round(tel.current_a, 2),
                                 moving=tel.moving, faults=tel.faults, torque=sid in self.torque_on)
        with self.lock:
            self.q_in = q_in

    # ------------------------------------------------------------ sim-side API (sim thread)
    def push_targets(self, q):
        """sim thread: the joint targets this step (5,3) rad."""
        with self.lock:
            self.q_out = np.asarray(q, float).reshape(5, 3).copy()

    def real_pose(self):
        """sim thread: (q[5,3] rad measured, legs_present[5]) for real2sim."""
        with self.lock:
            return self.q_in.copy(), self.legs_present.copy()

    # ------------------------------------------------------------ controls (any thread)
    def set_mirror(self, mode):
        if mode not in MIRRORS:
            raise ValueError(f"mirror must be one of {MIRRORS}")
        with self.lock:
            self.mirror = mode
        if mode == "sim2real":
            self.torque(True)
        self.on_event("mirror", mode)
        return mode

    def torque(self, on: bool, ids=None):
        ids = list(ids) if ids is not None else self.present_leg_ids()
        if not ids:
            return []
        with self.bus_lock:
            self.bus.torque_all(ids, on)
        if on:
            self.torque_on.update(ids)
            self.robot.monitor.tripped.difference_update(ids)
        else:
            self.torque_on.difference_update(ids)
        return ids

    def limp(self):
        with self.lock:
            self.mirror = "off"
        ids = self.torque(False)
        self.on_event("limp", f"torque off on {len(ids)} servos")
        return ids

    def jog(self, sid: int, deg: float, speed_cps: int = 200):
        """Move ONE servo to a center-relative angle (deg) — the bench nudge."""
        if sid not in self.present:
            raise BusError(f"id {sid} not present (scan first)")
        with self.lock:
            if self.mirror == "sim2real":           # a jog takes the servo away from the sim stream
                self.mirror = "off"
        with self.bus_lock:
            if sid not in self.torque_on:
                self.bus.torque(sid, True)
                self.torque_on.add(sid)
            lo, hi = -150.0, 150.0
            leg, j = divmod(sid - 1, 3)
            if 0 <= leg < 5:
                lo, hi = self.robot.soft_limits_deg[JOINTS[j]]
                d = self.robot.dir.get(f"leg{leg}_{JOINTS[j]}", 1)
                lo, hi = sorted((d * lo, d * hi))
            deg = float(np.clip(deg, lo, hi))
            self.bus.set_position(sid, deg, speed_cps)
        return deg

    def set_id(self, old: int, new: int):
        with self.bus_lock:
            self.bus.set_id(old, new)
        self.present[new] = self.present.pop(old, {})
        self.on_event("set_id", f"{old} -> {new}")
        return new

    # ------------------------------------------------------------ calibration
    def _save_cal(self):
        os.makedirs(os.path.dirname(CAL_PATH), exist_ok=True)
        with open(CAL_PATH, "w") as f:
            yaml.safe_dump(self.cal, f, sort_keys=True)
        self.robot.dir = {k: int(v) for k, v in self.cal.get("dir", {}).items()}
        self.robot.offset = {k: int(v) for k, v in self.cal.get("offset", {}).items()}

    def center(self, key: str, jig_deg: float | None = None):
        """Torque OFF that joint, read where it rests at the jig pose, store the offset."""
        leg, joint = key.replace("leg", "").split("_")
        leg, j = int(leg), JOINTS.index(joint)
        sid = self.leg_ids[leg][j]
        if sid not in self.present:
            raise BusError(f"{key} (id {sid}) not present")
        if jig_deg is None:
            jig_deg = -90.0 if joint == "knee" else 0.0
        with self.bus_lock:
            self.bus.torque(sid, False)
            self.torque_on.discard(sid)
            pos = self.bus.read_reg(sid, "PRESENT_POSITION")
        fam = Family.STS
        d = int(self.cal.setdefault("dir", {}).get(key, 1))
        jig_counts = round(jig_deg * COUNTS[fam] / SWEEP_DEG[fam])
        offset = int((pos - CENTER[fam]) - d * jig_counts)
        self.cal.setdefault("offset", {})[key] = offset
        self.cal.setdefault("meta", {})["knee_jig_deg"] = -90.0
        self._save_cal()
        self.on_event("center", f"{key}: present {pos} -> offset {offset:+d} counts")
        return dict(key=key, present=pos, offset=offset, offset_deg=round(offset * 360 / 4096, 2))

    def set_dir(self, key: str, d: int):
        self.cal.setdefault("dir", {})[key] = 1 if int(d) >= 0 else -1
        self._save_cal()
        return self.cal["dir"][key]

    # ------------------------------------------------------------ status
    def status(self):
        with self.lock:
            legs = [bool(x) for x in self.legs_present]
            mirror = self.mirror
        return dict(port=self.port, mock=self.mock is not None, mirror=mirror, legs=legs,
                    present=sorted(self.present), n_present=len(self.present),
                    tel=[self.tel[i] for i in sorted(self.tel)], torque=sorted(self.torque_on),
                    tripped=sorted(self.robot.monitor.tripped), errors=self.errors, last_error=self.last_error,
                    cal=dict(dir=self.cal.get("dir", {}), offset=self.cal.get("offset", {})),
                    speed_cps=self.speed_cps)

    def close(self):
        self.alive = False
        try:
            self.limp()
        except Exception:
            pass
        self.thread.join(timeout=1.0)
        try:
            self.bus.t.close()
        except Exception:
            pass
