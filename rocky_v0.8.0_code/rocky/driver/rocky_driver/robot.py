"""PebbleRobot — the ID map + calibration layer between gait-space and the bus.

Gait space (pebble_gait.py): q[5,3] radians, (yaw, hip, knee) per leg,
signs per the CAD frames. Servo space: counts per family. The bridge is
    counts = center + DIR * q * counts_per_rad + OFFSET
with DIR/OFFSET per joint from bench/calibration.yaml (written by
bench/calibrate_centers.py; defaults here let sim rehearsals run today).

Servo IDs (params.yaml `bus:` — the plan's IDs 1–15 legs + 16–20 hands):
    leg i (0..4): yaw = 3i+1, hip = 3i+2, knee = 3i+3
    hand i:       16 + i
"""
from __future__ import annotations
import math
import os
import yaml
from .bus import FeetechBus, SafetyMonitor, SafetyLimits, Telemetry
from .protocol import Family
from .registers import COUNTS, SWEEP_DEG

_HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.normpath(os.path.join(_HERE, "..", "..", "cad", "params.yaml"))

JOINT_NAMES = ("yaw", "hip", "knee")


def load_bus_params(path: str = PARAMS_PATH) -> dict:
    with open(path) as f:
        p = yaml.safe_load(f)
    if "bus" not in p:
        raise KeyError("params.yaml has no `bus:` section")
    return p["bus"]


class PebbleRobot:
    def __init__(self, bus: FeetechBus, bus_params: dict | None = None,
                 calibration: dict | None = None):
        bp = bus_params or load_bus_params()
        self.leg_ids: list[list[int]] = bp["leg_ids"]        # [5][3]
        self.hand_ids: list[int] = bp["hand_ids"]            # [5]
        fams = {i: Family.STS for leg in self.leg_ids for i in leg}
        fams |= {i: Family.SCS for i in self.hand_ids}
        bus.families |= fams
        self.bus = bus
        self.soft_limits_deg = bp.get("soft_limits_deg",
                                      {"yaw": [-40, 40], "hip": [-70, 90],
                                       "knee": [-150, -20], "claw": [0, 55]})
        cal = calibration or {}
        # per-joint direction (+1/-1) and offset (counts), keyed "leg{i}_{joint}"
        self.dir = {k: int(v) for k, v in cal.get("dir", {}).items()}
        self.offset = {k: int(v) for k, v in cal.get("offset", {}).items()}
        self.claw_dir = cal.get("claw_dir", {})
        self.monitor = SafetyMonitor(bus, self.all_ids(),
                                     SafetyLimits(**bp.get("safety", {})))

    # ------------------------------------------------------------ id utils
    def all_ids(self) -> list[int]:
        return [i for leg in self.leg_ids for i in leg] + list(self.hand_ids)

    def joint_key(self, leg: int, joint: int) -> str:
        return f"leg{leg}_{JOINT_NAMES[joint]}"

    # ------------------------------------------------------------ mapping
    def q_to_deg(self, leg: int, joint: int, q_rad: float) -> float:
        """Gait joint angle -> servo command in center-relative degrees."""
        key = self.joint_key(leg, joint)
        d = self.dir.get(key, 1)
        lo, hi = self.soft_limits_deg[JOINT_NAMES[joint]]
        q_deg = max(lo, min(hi, math.degrees(q_rad)))
        off_counts = self.offset.get(key, 0)
        off_deg = off_counts * SWEEP_DEG[Family.STS] / COUNTS[Family.STS]
        return d * q_deg + off_deg

    def deg_to_q(self, leg: int, joint: int, deg: float) -> float:
        key = self.joint_key(leg, joint)
        d = self.dir.get(key, 1)
        off_counts = self.offset.get(key, 0)
        off_deg = off_counts * SWEEP_DEG[Family.STS] / COUNTS[Family.STS]
        return math.radians((deg - off_deg) / d)

    # ------------------------------------------------------------ actions
    def enable(self, on: bool = True) -> None:
        self.bus.torque_all(self.all_ids(), on)

    def limp(self) -> None:
        """The safe pose exit: everything goes soft (sleep-pose prep)."""
        self.enable(False)

    def send_leg_targets(self, q_rad_5x3, speed_cps: int = 0) -> None:
        """q_rad_5x3: anything indexable [leg][joint] in radians (gait output)."""
        targets: dict[int, float] = {}
        for leg in range(5):
            for j in range(3):
                targets[self.leg_ids[leg][j]] = self.q_to_deg(leg, j, q_rad_5x3[leg][j])
        self.bus.sync_positions(targets, speed_cps)

    def send_claws(self, open_frac_5) -> None:
        """open_frac in 0..1 per hand -> 0..55 deg (D018 cap until hand v0.3)."""
        lo, hi = self.soft_limits_deg["claw"]
        targets = {}
        for i in range(5):
            off = self.offset.get(f"hand{i}_claw", 0) \
                * SWEEP_DEG[Family.SCS] / COUNTS[Family.SCS]
            d = self.dir.get(f"hand{i}_claw", 1)
            targets[self.hand_ids[i]] = \
                d * (lo + (hi - lo) * float(open_frac_5[i])) + off
        self.bus.sync_positions(targets)

    def read_joint_state(self) -> tuple[list[list[float]], dict[int, Telemetry]]:
        """Returns (q[5][3] radians, full telemetry map). One SYNC_READ for legs."""
        tels = self.bus.sync_telemetry(self.all_ids())
        q = [[0.0] * 3 for _ in range(5)]
        for leg in range(5):
            for j in range(3):
                tel = tels.get(self.leg_ids[leg][j])
                if tel:
                    q[leg][j] = self.deg_to_q(leg, j, tel.position_deg)
        return q, tels

    def health_step(self) -> dict[int, Telemetry]:
        """Run one SafetyMonitor poll (call at 1–5 Hz from your loop)."""
        return self.monitor.step()


def load_calibration(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}
