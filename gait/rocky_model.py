"""rocky_model — the ONE loader for what the robot is (D052).

Every number that says what Pebble's actuators and joints can do lives in
cad/params.yaml (`actuators:`, `joints:`, `gait:`, `reflex:`, `sensing:`,
`leg.foot`). Before D052 the servo identity was copied into 5+ files and
they disagreed (velocity 4.7 / 5.0 / 5.2 rad/s, mass 55 / 60 g, limits
hand-typed in the recover env). This module is how everything else reads
them: the MJCF + URDF generators, the servo realism layer, the driver's
soft-limit clamp, the gait, the reflexes, the RL envs, the studio.

Pure Python + PyYAML on purpose (no numpy, no mujoco): the bus driver
imports it, and so will anything that runs on the robot's computer.
Everything returned is a fresh copy — mutate freely, the cache is safe.

    import rocky_model as rm
    lo, hi = rm.joint_limits_rad()          # (yaw, hip, knee) tuples, rad
    rm.servo_speed("loaded")                # 3.0 rad/s
    WaveGait(**rm.gait_defaults())
    rm.stall_nm(), rm.continuous_nm()       # 2.94 (peak = sim forcerange), 1.911 N.m (thermal)
    rm.dc_speed(), rm.dc_speed(rm.continuous_nm())   # 4.7 / 3.05 rad/s (the DC line, no load)
    rm.thermal()                            # the RL thermal proxy's constants (budget 54)

D052 amendment (owner, 2026-09-24): a position servo's instantaneous torque
is its STALL torque; continuous_frac is a THERMAL (sustained) budget, not an
instantaneous cap. The MJCF forcerange / URDF effort are stall_nm(); the
sustained budget is enforced where time matters (rl_common.ThermalProxy,
pebble_feasibility THERMAL_LOAD), not by clipping every step.

D053 (2026-09-24, docs/ROBOT_AS_DATA.md step 1): the robot's SHAPE is data
too. `robot()` builds a frozen RobotSpec from params `robot:` (legs, the
per-leg joint chain, foot, tool, IK solver) and links it to the ranges in
`joints.pos_deg` and the ids in `bus:`; `validate()` refuses a spec the rest
of the code cannot honour. n_legs(), stations_deg(), actuator_order(),
id_table(), topology_hash() are how the generators and tests read it.
LEG_JOINTS / ALL_JOINTS are served from the spec (PEP 562, on first access),
so `import rocky_model` never validates anything and a half-edited `robot:`
block breaks only code that asks about topology — actuator(), leg_ids()
keep working.

    rm.n_legs(), rm.stations_deg()          # 5, (90.0, 162.0, 234.0, 306.0, 378.0)
    rm.actuator_order()                     # ['yaw0', 'hip0', 'knee0', ..., 'claw4'] = ctrl order
    rm.topology_hash()                      # changes iff a checkpoint can no longer load
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
from dataclasses import asdict, dataclass
from typing import NamedTuple

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.normpath(os.path.join(_HERE, "..", "cad", "params.yaml"))

# LEG_JOINTS / ALL_JOINTS: module __getattr__ below (D053) — ("yaw", "hip", "knee")
# and + ("claw",) for today's robot, read from the spec on first access.
SPAWN_CLEARANCE_MM = 1.0        # feet start this far above the floor, then settle

_CACHE: dict = {}
_ROBOT_CACHE: dict = {}         # path -> (RobotSpec, warnings)


# ------------------------------------------------------------------ raw params
def params(path: str | None = None) -> dict:
    """The parsed params.yaml (cached per path). A deep copy: callers may mutate it."""
    p = os.path.normpath(path or PARAMS_PATH)
    if p not in _CACHE:
        with open(p) as f:
            _CACHE[p] = yaml.safe_load(f)
    return copy.deepcopy(_CACHE[p])


def reload() -> None:
    """Drop the cache (tests, or a live session that just edited params.yaml)."""
    _CACHE.clear()
    _ROBOT_CACHE.clear()


def _p() -> dict:
    """Internal read-only view (no copy) — never hand this out."""
    params()
    return _CACHE[os.path.normpath(PARAMS_PATH)]


def params_rev() -> str:
    m = _p().get("meta", {})
    return f"{m.get('version', '?')}/{m.get('params_rev', '?')}"


# ------------------------------------------------------------------ actuators
def leg_servo() -> str:
    return _p()["leg"].get("servo", "st3215")


def claw_servo() -> str:
    return _p()["leg"].get("claw_servo", "scs0009")


def _servo_name(name: str | None) -> str:
    if name in (None, "leg", "yaw", "hip", "knee"):
        return leg_servo()
    if name in ("claw", "hand"):
        return claw_servo()
    return name


def actuator(name: str | None = None) -> dict:
    """The actuators.<name> block (default: the leg servo), normalised so every
    servo answers the same keys: stall_nm, no_load_rad_s, continuous_frac,
    counts, sweep_deg, latency_s, bus_hz, mass_g (+ whatever else it carries)."""
    key = _servo_name(name)
    a = dict(_p()["actuators"][key])
    stall = next((v for k, v in a.items() if k.startswith("stall_nm")), None)
    if stall is None:
        raise KeyError(f"actuators.{key} has no stall_nm*")
    a["stall_nm"] = float(stall)
    if "counts" not in a:                       # full-turn encoder (ST3215)
        a["counts"] = int(a["counts_per_rev"])
        a.setdefault("sweep_deg", 360.0)
    a["name"] = key
    return a


def stall_nm(name: str | None = None) -> float:
    """Stall torque (N.m) = the servo's PEAK (instantaneous) torque: the MJCF
    forcerange and the URDF effort since the D052 amendment. Not a sustained budget."""
    return actuator(name)["stall_nm"]


peak_nm = stall_nm                  # D052 amendment: the name says what the sim clips at


def continuous_nm(name: str | None = None) -> float:
    """Thermal (SUSTAINED) torque budget = continuous_frac x stall. What the servo
    can hold for minutes — not an instantaneous cap (D052 amendment: the sim's
    forcerange is stall; this is where the thermal proxy starts to heat)."""
    a = actuator(name)
    return a["stall_nm"] * float(a["continuous_frac"])


def continuous_frac(name: str | None = None) -> float:
    return float(actuator(name)["continuous_frac"])


def no_load_rad_s(name: str | None = None) -> float:
    return float(actuator(name)["no_load_rad_s"])


def damping_nms(name: str | None = None) -> float:
    """Slope of the DC motor torque-speed line, stall / no-load (N.m.s/rad):
    the joint damping that makes a saturated servo slow down under load."""
    a = actuator(name)
    return a["stall_nm"] / float(a["no_load_rad_s"])


def dc_speed(motor_nm: float | None = None, load_nm: float = 0.0, name: str | None = None) -> float:
    """Steady joint speed (rad/s) on the DC torque-speed line: the motor puts out
    `motor_nm` (default: stall, the MJCF forcerange), the joint carries `load_nm`,
    and the rest is eaten by the back-EMF damping — (motor - load) / damping.
    No load: 4.7 rad/s at stall (= no-load speed), 3.05 at the continuous torque
    (a thermally derated joint, and the 'loaded' speed budget). Stall motor with
    the continuous torque as load: 1.65."""
    m = stall_nm(name) if motor_nm is None else float(motor_nm)
    return max(0.0, (m - float(load_nm)) / damping_nms(name))


def thermal(name: str | None = None) -> dict:
    """The thermal proxy's constants (D052 amendment): heat integrates
    ((|tau_e|/stall)^2 - continuous_frac^2) dt (tau_e = the ELECTRICAL torque,
    force - damping x qvel: rl_common.motor_torque), clamped at 0; it trips at
    budget = (trip_frac^2 - continuous_frac^2) x trip_s load-fraction^2.s —
    i.e. `trip_s` seconds at `trip_frac` x stall (the driver mock's 70 C cut:
    ~3 min at 0.85). Servos without trip keys inherit the leg servo's."""
    a = actuator(name)
    leg = actuator()
    cf = float(a["continuous_frac"])
    tf = float(a.get("thermal_trip_frac", leg["thermal_trip_frac"]))
    ts = float(a.get("thermal_trip_s", leg["thermal_trip_s"]))
    return dict(stall_nm=a["stall_nm"], continuous_frac=cf, trip_frac=tf, trip_s=ts,
                budget=(tf * tf - cf * cf) * ts)


def counts_per_rad(name: str | None = None) -> float:
    a = actuator(name)
    return a["counts"] / math.radians(float(a["sweep_deg"]))


def bus_hz() -> float:
    return float(actuator()["bus_hz"])


def latency_s() -> float:
    return float(actuator()["latency_s"])


# ------------------------------------------------------------------ joints
def joint_limits_deg() -> dict:
    """{'yaw': (lo, hi), 'hip': ..., 'knee': ..., 'claw': ...} in degrees."""
    j = _p()["joints"]["pos_deg"]
    return {k: (float(j[k][0]), float(j[k][1])) for k in _all_joints()}


def joint_limits_rad() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(lo, hi) for (yaw, hip, knee), radians — np.tile(lo, 5) for all 15 leg joints."""
    d = joint_limits_deg()
    lo = tuple(math.radians(d[k][0]) for k in _leg_joints())
    hi = tuple(math.radians(d[k][1]) for k in _leg_joints())
    return lo, hi


def claw_limits(unit: str = "deg") -> tuple[float, float]:
    lo, hi = joint_limits_deg()["claw"]
    if unit.startswith("rad"):
        return math.radians(lo), math.radians(hi)
    return lo, hi


def servo_speed(kind: str = "loaded") -> float:
    """Joint speed budget (rad/s): 'loaded' (stance, carrying weight), 'free'
    (swinging leg) or 'hard' (= the leg servo's no-load speed; never exceed).

    'loaded' = 3.0 is dc_speed(continuous_nm()) = 1.911 / 0.626 = 3.05 rad/s
    rounded down: the fastest a joint turns while its motor stays inside the
    SUSTAINED (thermal) torque budget — an upper bound, a joint that also
    carries a load L gets (1.911 - L) / 0.626. 'free' = 4.0 is ~85 % of the
    4.7 no-load speed (dc_speed()), which the MJCF reaches since the D052
    amendment (forcerange = stall; it capped at 3.06 with the continuous clip)."""
    v = _p()["joints"]["vel_rad_s"]
    if kind not in v:
        raise KeyError(f"servo_speed kind must be one of {sorted(v)}")
    return float(v[kind])


# ------------------------------------------------------------------ gait / reflex / sensing
def gait_defaults() -> dict:
    """WaveGait kwargs: body_height, stance_radius, cycle_time, duty, step_height."""
    return {k: float(v) for k, v in _p()["gait"].items()}


def reflex_defaults() -> dict:
    """ReflexSupervisor kwargs seeded from today's values (gyro_trip, stall_s, ...)."""
    return {k: float(v) for k, v in _p()["reflex"].items()}


def foot_switch() -> tuple[float, float]:
    """(close_n, open_n): the foot switch's operating / release force (VERIFY)."""
    s = _p()["sensing"]["foot_switch"]
    return float(s["close_n"]), float(s["open_n"])


# ------------------------------------------------------------------ geometry
def foot_contact_radius_mm() -> float:
    """r_c = cone tip sphere + TPU crown: the sim foot sphere's radius."""
    f = _p()["leg"]["foot"]
    return float(f["tip_radius"]) + float(f["pad_crown"])


def shell_rim_z_m() -> float:
    """Carapace top rim above the deck plane (m): where sim/shove.py pushes."""
    return float(_p()["body"]["shell_rim_z"]) / 1000.0


def _stance_tibia_uz(body_height: float, stance_radius: float) -> float:
    """z-component of the tibia's unit vector at the nominal stance (knee-down IK)."""
    leg = _p()["leg"]
    L1, L2, L3 = leg["l1_coxa"], leg["l2_femur"], leg["l3_tibia"]
    zh = float(leg["hip_axis_z"])
    rb = float(_p()["body"]["circumradius"])
    dx = (stance_radius - rb) - L1
    dz = -body_height - zh
    c3 = (dx * dx + dz * dz - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if abs(c3) > 1.0:
        raise ValueError("nominal stance is out of reach")
    q3 = -math.acos(c3)
    q2 = math.atan2(dz, dx) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    return math.sin(q2 + q3)


def spawn_dz_mm(body_height: float | None = None, stance_radius: float | None = None,
                clearance_mm: float = SPAWN_CLEARANCE_MM) -> float:
    """Spawn the torso at z = (body_height + spawn_dz_mm()) / 1000 and the feet
    start `clearance_mm` above the floor. D052: the foot sphere's SURFACE is the
    IK foot point now, so this is ~1.4 mm (it was a hard-coded 14 = the old
    13 mm ball + 1). The extra ~0.4 mm: a tilted tibia puts the sphere's lowest
    point slightly below the IK point, r_c * (1 - |u_z|)."""
    g = gait_defaults()
    h = g["body_height"] if body_height is None else float(body_height)
    r0 = g["stance_radius"] if stance_radius is None else float(stance_radius)
    uz = _stance_tibia_uz(h, r0)
    return foot_contact_radius_mm() * (1.0 - abs(uz)) + clearance_mm


def spawn_z_m(body_height: float | None = None, platform_z_m: float = 0.0) -> float:
    """Torso spawn height (m) over a floor at platform_z_m."""
    h = gait_defaults()["body_height"] if body_height is None else float(body_height)
    return (h + spawn_dz_mm(h)) / 1000.0 + platform_z_m


def stance_torso_z_m(body_height: float | None = None) -> float:
    """Where the torso settles on a rigid floor with ideal servos (m) — no clearance.
    Real servos sag ~1-2 mm below this under the D052 forcerange (the joint
    damping and the kp=20 position error, not the clip: stance loads are ~0.1-0.2 x stall)."""
    h = gait_defaults()["body_height"] if body_height is None else float(body_height)
    return (h + spawn_dz_mm(h, clearance_mm=0.0)) / 1000.0


# ------------------------------------------------------------------ bus ids
def leg_ids() -> list[list[int]]:
    """[5][3] servo ids, (yaw, hip, knee) per leg."""
    return [list(map(int, row)) for row in _p()["bus"]["leg_ids"]]


def hand_ids() -> list[int]:
    return [int(i) for i in _p()["bus"]["hand_ids"]]


def id_to_joint(sid: int) -> tuple[int, str]:
    """Servo id -> (leg, joint name in yaw/hip/knee/claw), by inverting the
    params map (never divmod: the map is data, the bench may renumber)."""
    for leg, row in enumerate(leg_ids()):
        for j, i in enumerate(row):
            if i == sid:
                return leg, _leg_joints()[j]
    for leg, i in enumerate(hand_ids()):
        if i == sid:
            return leg, "claw"
    raise KeyError(f"servo id {sid} is not in params.yaml bus map")


def joint_to_id(leg: int, joint: str) -> int:
    if joint == "claw":
        return hand_ids()[leg]
    return leg_ids()[leg][_leg_joints().index(joint)]


# ------------------------------------------------------------------ robot description (D053)
# docs/ROBOT_AS_DATA.md. The robot's shape (how many legs, where, which joints
# in which order, which servo drives each, which id it answers to) as ONE
# frozen object built from params. Pure Python on purpose — the driver
# imports this module; the numpy kinematics come later (leg_kin, step 6).

class RobotSpecError(ValueError):
    """params `robot:` (or the bus map / joint ranges it links to) describes a
    robot the code cannot honour. The message says which key and what to do."""


@dataclass(frozen=True)
class JointSpec:
    name: str                    # 'yaw' / 'hip' / 'knee' / 'claw' / 'ankle' ...
    axis: tuple                  # unit vector in the joint's own (parent) frame
    offset_mm: tuple             # joint origin in the PARENT joint frame (mm); yaw: the LEG frame
    link: str                    # the body this joint moves ('coxa', 'femur', ...)
    servo: str                   # actuators.<key>
    range_deg: tuple             # joints.pos_deg[name]
    bus_id: int | None           # bus.leg_ids[leg][k] / bus.hand_ids[leg]
    role: str = "leg"            # 'leg' (IK + RL action) | 'tool' (claw: neither)


@dataclass(frozen=True)
class LegSpec:
    index: int
    station_deg: float           # unwrapped: leg 4 is 378, not 18 (pebble.xml euler)
    mount_mm: tuple              # yaw-axis origin in the BODY frame, z = deck plane
    joints: tuple                # JointSpec..., chain order base -> foot
    foot_offset_mm: tuple        # the IK foot point in the last joint's frame
    tool: JointSpec | None
    tool_bus_id: int | None
    ik: dict                     # {'solver', 'resolved', 'branch', ...} as declared + resolved

    @property
    def n_dof(self) -> int:
        return len(self.joints)

    @property
    def joint_names(self) -> tuple:
        return tuple(j.name for j in self.joints)


class IdRow(NamedTuple):
    """One servo on the bus: what the driver, hw_bridge and the mock need."""
    sid: int
    leg: int
    joint: str
    servo: str
    protocol: str                # 'sts' (ST3215, protocol 1) | 'scs' (SCS0009, protocol 0) | '?'
    counts: int
    sweep_deg: float
    limits_deg: tuple


@dataclass(frozen=True)
class RobotSpec:
    n_legs: int
    station_radius_mm: float
    legs: tuple                  # LegSpec...
    params_rev: str

    # --- names + order
    def stations_deg(self) -> tuple:
        return tuple(L.station_deg for L in self.legs)

    def uniform_dof(self) -> int:
        """The per-leg DOF when every leg has the same chain (today's (N, dof)
        consumers need that); RobotSpecError otherwise."""
        dofs = {L.n_dof for L in self.legs}
        names = {L.joint_names for L in self.legs}
        if len(dofs) != 1 or len(names) != 1:
            raise RobotSpecError(f"legs differ ({sorted(dofs)} DOF, chains {sorted(names)}): "
                                 "(N, dof) consumers need identical legs until ROBOT_AS_DATA step 10")
        return dofs.pop()

    def leg_joint_names(self, leg: int = 0) -> tuple:
        return self.legs[leg].joint_names

    def tool_names(self) -> tuple:
        """Distinct tool joint names in leg order (today: ('claw',))."""
        out = []
        for L in self.legs:
            if L.tool is not None and L.tool.name not in out:
                out.append(L.tool.name)
        return tuple(out)

    def servo_for(self, joint: str, leg: int = 0) -> str:
        L = self.legs[leg]
        for j in L.joints:
            if j.name == joint:
                return j.servo
        if L.tool is not None and L.tool.name == joint:
            return L.tool.servo
        raise KeyError(f"leg {leg} has no joint {joint!r} (chain {L.joint_names})")

    def actuator_order(self) -> list:
        """MJCF actuator names in ctrl order: every leg's chain, leg-major, then the
        tools — ['yaw0', 'hip0', 'knee0', ..., 'knee4', 'claw0', ..., 'claw4']. This
        is the contract behind today's ctrl[:15] / ctrl[15:20] slices."""
        out = [f"{j.name}{L.index}" for L in self.legs for j in L.joints]
        out += [f"{L.tool.name}{L.index}" for L in self.legs if L.tool is not None]
        return out

    def n_leg_actuators(self) -> int:
        return sum(L.n_dof for L in self.legs)

    # --- fingerprints
    def topology(self) -> dict:
        """What decides whether a checkpoint can load at all: leg count and
        stations, joint names / order / axes, the tool, the IK solver. NOT lengths,
        servos, masses or ids (those rotate the robot fingerprint instead)."""
        return {
            "n_legs": self.n_legs,
            "stations_deg": [round(a, 6) for a in self.stations_deg()],
            "legs": [{"joints": [[j.name, [round(v, 9) for v in j.axis]] for j in L.joints],
                      "tool": None if L.tool is None else [L.tool.name, [round(v, 9) for v in L.tool.axis]],
                      "ik": L.ik.get("resolved")} for L in self.legs],
        }

    def topology_hash(self) -> str:
        return _hash12(self.topology())

    def description_digest(self) -> str:
        """12-hex over EVERYTHING in the spec (lengths, servos, ranges, ids too)."""
        return _hash12(asdict(self))

    # --- views
    def id_rows(self, P: dict) -> list:
        acts = P["actuators"]
        rows = []
        for L in self.legs:
            for j in L.joints + ((L.tool,) if L.tool is not None else ()):
                a = _normalise_actuator(acts, j.servo)
                rows.append(IdRow(int(j.bus_id), L.index, j.name, j.servo, _protocol(j.servo, a),
                                  int(a["counts"]), float(a["sweep_deg"]), tuple(j.range_deg)))
        return sorted(rows, key=lambda r: r.sid)

    def to_json(self) -> dict:
        d = asdict(self)
        d["stations_deg"] = list(self.stations_deg())
        d["actuator_order"] = self.actuator_order()
        d["topology_hash"] = self.topology_hash()
        d["description_digest"] = self.description_digest()
        return json.loads(json.dumps(d))            # tuples -> lists: plain JSON

    def summary(self) -> str:
        L0 = self.legs[0]
        tool = f" + tool {L0.tool.name} ({L0.tool.servo})" if L0.tool is not None else ""
        chain = "/".join(f"{j.name}" for j in L0.joints)
        servos = sorted({j.servo for L in self.legs for j in L.joints})
        stations = ", ".join(f"{a:g}" for a in self.stations_deg())
        return (f"robot: {self.n_legs} legs at r {self.station_radius_mm:g} mm, stations ({stations}) deg; "
                f"leg chain {chain} ({L0.n_dof} DOF, {'/'.join(servos)}){tool}; "
                f"IK {L0.ik.get('resolved')}; {len(self.actuator_order())} actuators; "
                f"topology {self.topology_hash()}, description {self.description_digest()}")


_PROTOCOL_DEFAULT = {"st3215": "sts", "scs0009": "scs"}   # until actuators.<k>.protocol (step 2)
_SOLVERS = ("auto", "analytic", "numeric")


def _hash12(obj) -> str:
    blob = json.dumps(obj, sort_keys=True, separators=(",", ":"), default=list).encode()
    return hashlib.sha256(blob).hexdigest()[:12]


def _normalise_actuator(acts: dict, key: str) -> dict:
    """actuator()'s normalisation on an arbitrary params dict (counts / sweep_deg)."""
    a = dict(acts[key])
    if "counts" not in a:
        a["counts"] = int(a["counts_per_rev"])
        a.setdefault("sweep_deg", 360.0)
    return a


def _protocol(key: str, a: dict) -> str:
    return str(a.get("protocol", _PROTOCOL_DEFAULT.get(key, "?")))


def _default_robot_block(P: dict) -> dict:
    """What `robot:` says when it is absent: today's pentapod, from leg/body/bus.
    Must stay equal to the block in cad/params.yaml (gait/test_robot_spec.py (f))."""
    leg, body = P["leg"], P["body"]
    return {
        "legs": {"count": 5, "first_station_deg": 90, "station_radius_mm": body["circumradius"]},
        "leg": {
            "chain": [
                {"name": "yaw", "axis": [0, 0, 1], "offset_mm": [0, 0, 0], "link": "coxa"},
                {"name": "hip", "axis": [0, -1, 0], "offset_mm": [leg["l1_coxa"], 0, leg["hip_axis_z"]],
                 "link": "femur"},
                {"name": "knee", "axis": [0, -1, 0], "offset_mm": [leg["l2_femur"], 0, 0], "link": "tibia"},
            ],
            "foot": {"offset_mm": [leg["l3_tibia"], 0, 0]},
            "tool": {"name": "claw", "axis": [0, 0, 1], "servo": leg.get("claw_servo", "scs0009")},
            "servo": leg.get("servo", "st3215"),
            "ik": {"solver": "auto", "branch": "knee_down"},
        },
        "overrides": {},
    }


def _vec3(v, what: str, errors: list) -> tuple | None:
    try:
        t = tuple(float(x) for x in v)
    except (TypeError, ValueError):
        errors.append(f"{what} must be a list of 3 numbers, got {v!r}")
        return None
    if len(t) != 3:
        errors.append(f"{what} must have 3 numbers, got {len(t)}")
        return None
    return t


def _is_yaw_2r(chain: list, foot: tuple) -> bool:
    """The shape pebble_gait.leg_ik is exact for (docs/ROBOT_AS_DATA.md §3.2):
    3 joints; joint 0 about +z at the origin; joints 1, 2 about the same +-y axis;
    every offset and the foot in the y = 0 plane; the knee and the foot along +x
    of their parent. Chosen by SHAPE, never by joint name."""
    if len(chain) != 3 or any(c is None for c in chain) or foot is None:
        return False
    (a0, o0), (a1, o1), (a2, o2) = chain
    y = (0.0, 1.0, 0.0)
    return (a0 == (0.0, 0.0, 1.0) and o0 == (0.0, 0.0, 0.0)
            and a1 == a2 and a1 in (y, (0.0, -1.0, 0.0))
            and o1[1] == 0.0 and o1[0] >= 0.0
            and o2[1] == 0.0 and o2[2] == 0.0 and o2[0] > 0.0
            and foot[1] == 0.0 and foot[2] == 0.0 and foot[0] > 0.0)


def _resolve(P: dict):
    """params -> (fields for RobotSpec or None, errors, warnings). No exceptions:
    every problem is collected so one validate() run reports them all."""
    errors, warns = [], []
    R = P.get("robot")
    if R is None:
        R = _default_robot_block(P)
    if not isinstance(R, dict):
        return None, [f"robot: must be a mapping, got {type(R).__name__}"], warns
    legs_b = R.get("legs") or {}
    leg_b = R.get("leg") or {}
    acts = P.get("actuators") or {}
    ranges = (P.get("joints") or {}).get("pos_deg") or {}
    bus = P.get("bus") or {}

    # -- stations
    try:
        n = int(legs_b.get("count", 0))
    except (TypeError, ValueError):
        n = 0
    if n < 1:
        errors.append(f"robot.legs.count must be a positive integer, got {legs_b.get('count')!r}")
    explicit = legs_b.get("station_deg")
    if explicit is not None:
        stations = tuple(float(a) for a in explicit)
        if len(stations) != n:
            errors.append(f"robot.legs.station_deg lists {len(stations)} angles for count {n}")
        step = [stations[i + 1] - stations[i] for i in range(len(stations) - 1)]
        uniform = n > 0 and all(abs(d - 360.0 / n) < 1e-6 for d in step)
        if not uniform and not legs_b.get("allow_asymmetric", False):
            errors.append("robot.legs.station_deg is not evenly spaced CCW (360/count apart): the wave gait's "
                          "phase order, manip_adjacent's lean math and the CAD sectors assume symmetry — set "
                          "allow_asymmetric: true only after auditing them (ROBOT_AS_DATA §8)")
    else:
        first = float(legs_b.get("first_station_deg", 90.0))
        stations = tuple(first + i * 360.0 / n for i in range(n)) if n > 0 else ()
    radius = float(legs_b.get("station_radius_mm", (P.get("body") or {}).get("circumradius", 0.0)))

    # -- chain template
    default_servo = leg_b.get("servo", (P.get("leg") or {}).get("servo", "st3215"))
    chain_b = leg_b.get("chain") or []
    if not chain_b:
        errors.append("robot.leg.chain is empty: a leg needs at least one joint")
    chain, names = [], []
    for k, c in enumerate(chain_b):
        name = str(c.get("name", f"j{k}"))
        if name in names:
            errors.append(f"robot.leg.chain: joint name {name!r} appears twice")
        names.append(name)
        axis = _vec3(c.get("axis"), f"robot.leg.chain[{k}] ({name}).axis", errors)
        if axis is not None and abs(math.sqrt(sum(v * v for v in axis)) - 1.0) > 1e-9:
            errors.append(f"robot.leg.chain[{k}] ({name}).axis {list(axis)} is not a unit vector")
        off = _vec3(c.get("offset_mm", [0, 0, 0]), f"robot.leg.chain[{k}] ({name}).offset_mm", errors)
        servo = str(c.get("servo", default_servo))
        if servo not in acts:
            errors.append(f"robot.leg.chain[{k}] ({name}) servo {servo!r} is not in actuators: "
                          f"(known: {sorted(acts)}) — add an actuators.{servo} block first")
        if name not in ranges:
            errors.append(f"joint {name!r} has no joints.pos_deg entry: add joints.pos_deg.{name}: [lo, hi]")
        chain.append(dict(name=name, axis=axis, offset=off, link=str(c.get("link", name)), servo=servo))
    foot = _vec3((leg_b.get("foot") or {}).get("offset_mm"), "robot.leg.foot.offset_mm", errors)

    tool_b = leg_b.get("tool")
    tool = None
    if tool_b:
        tname = str(tool_b.get("name", "tool"))
        if tname in names:
            errors.append(f"robot.leg.tool name {tname!r} clashes with a chain joint")
        taxis = _vec3(tool_b.get("axis", [0, 0, 1]), "robot.leg.tool.axis", errors)
        toff = _vec3(tool_b.get("offset_mm", [0, 0, 0]), "robot.leg.tool.offset_mm", errors)
        tservo = str(tool_b.get("servo", (P.get("leg") or {}).get("claw_servo", "scs0009")))
        if tservo not in acts:
            errors.append(f"robot.leg.tool servo {tservo!r} is not in actuators: (known: {sorted(acts)})")
        if tname not in ranges:
            errors.append(f"tool {tname!r} has no joints.pos_deg entry: add joints.pos_deg.{tname}: [lo, hi]")
        tool = dict(name=tname, axis=taxis, offset=toff, servo=tservo)

    # -- IK
    ik = dict(leg_b.get("ik") or {"solver": "auto"})
    solver = str(ik.get("solver", "auto"))
    shape_ok = _is_yaw_2r([None if (c["axis"] is None or c["offset"] is None) else (c["axis"], c["offset"])
                           for c in chain], foot)
    chain_txt = "/".join(names) or "(empty)"
    if solver not in _SOLVERS:
        errors.append(f"robot.leg.ik.solver {solver!r} must be one of {list(_SOLVERS)}")
    elif solver == "analytic" and not shape_ok:
        errors.append(f"robot.leg.ik.solver 'analytic' is exact only for a yaw + 2 parallel-pitch chain; "
                      f"this chain is {chain_txt} ({len(chain)} DOF) — use solver: numeric"
                      + (" with a redundancy rule" if len(chain) > 3 else ""))
    resolved = "analytic" if (solver == "analytic" or (solver == "auto" and shape_ok)) else "numeric"
    if resolved == "numeric" and len(chain) > 3 and not ik.get("redundancy"):
        errors.append(f"robot.leg.ik: a {len(chain)}-DOF leg reaching a 3-D point has "
                      f"{len(chain) - 3} spare DOF — numeric IK needs a redundancy rule "
                      "(redundancy: foot_pitch or min_change), or it jumps between solutions")
    ik["solver"], ik["resolved"] = solver, resolved

    # -- overrides: nothing honours them yet
    if R.get("overrides"):
        errors.append("robot.overrides is not empty, but no consumer honours per-leg / per-joint servos yet "
                      "(ROBOT_AS_DATA step 10): refusing rather than silently ignoring it")

    # -- bus map
    leg_ids = bus.get("leg_ids") or []
    hand_ids = bus.get("hand_ids") or []
    if len(leg_ids) != n:
        errors.append(f"bus.leg_ids has {len(leg_ids)} rows for robot.legs.count {n}: one row per leg")
    for i, row in enumerate(leg_ids):
        if len(row) != len(chain):
            errors.append(f"bus.leg_ids[{i}] has {len(row)} ids for a {len(chain)}-joint chain ({chain_txt})")
    if tool is not None and len(hand_ids) != n:
        errors.append(f"bus.hand_ids has {len(hand_ids)} ids for {n} legs with a {tool['name']} tool")
    seen = {}
    slots = [(f"bus.leg_ids[{i}][{k}]", s) for i, row in enumerate(leg_ids) for k, s in enumerate(row)]
    slots += [(f"bus.hand_ids[{i}]", s) for i, s in enumerate(hand_ids)]
    for where, sid in slots:
        if sid in seen:
            errors.append(f"servo id {sid} is used twice ({seen[sid]} and {where}): every id must be unique")
        seen.setdefault(sid, where)

    # -- warnings
    duty = (P.get("gait") or {}).get("duty")
    if duty is not None and n > 0 and abs(float(duty) - (1.0 - 1.0 / n)) > 1e-3:
        warns.append(f"gait.duty {duty} != 1 - 1/count = {1 - 1 / n:.4f} (one leg in swing at a time); "
                     "fine for a tripod-style gait, check the support margin with pebble_feasibility")
    for s in sorted({c["servo"] for c in chain} | ({tool["servo"]} if tool else set())):
        if s not in acts:
            continue
        cad = acts[s].get("cad")
        if cad is None:
            warns.append(f"actuators.{s} has no `cad:` key (ROBOT_AS_DATA step 2): nothing checks that CAD "
                         "builds this servo's case")
        elif s in {c["servo"] for c in chain} and cad != "servo_st3215":
            warns.append(f"actuators.{s}.cad is {cad!r} but the leg CAD is still hard-keyed to servo_st3215 "
                         "(servo_st3215.py): the printed cups will not fit this servo")

    if errors:
        return None, errors, warns

    legs = []
    for i, a in enumerate(stations):
        ar = math.radians(a)
        joints = tuple(JointSpec(c["name"], c["axis"], c["offset"], c["link"], c["servo"],
                                 tuple(float(v) for v in ranges[c["name"]]), int(leg_ids[i][k]))
                       for k, c in enumerate(chain))
        tj = tid = None
        if tool is not None:
            tid = int(hand_ids[i])
            tj = JointSpec(tool["name"], tool["axis"], tool["offset"], f"{tool['name']}b", tool["servo"],
                           tuple(float(v) for v in ranges[tool["name"]]), tid, role="tool")
        legs.append(LegSpec(i, float(a), (radius * math.cos(ar), radius * math.sin(ar), 0.0),
                            joints, foot, tj, tid, dict(ik)))
    meta = P.get("meta") or {}
    fields = dict(n_legs=n, station_radius_mm=radius, legs=tuple(legs),
                  params_rev=f"{meta.get('version', '?')}/{meta.get('params_rev', '?')}")
    return fields, errors, warns


def validate(P: dict | None = None) -> list:
    """Check a params dict's robot description (default: the live params.yaml).
    Errors raise RobotSpecError (all of them, one per line); warnings are
    returned — `python gait/rocky_model.py` and `rocky.sh regen` print them.
    Never prints itself: stdio MCP servers import this module."""
    _, errors, warns = _resolve(params() if P is None else P)
    if errors:
        raise RobotSpecError("params robot description is invalid:\n  - " + "\n  - ".join(errors))
    return warns


def robot(path: str | None = None) -> RobotSpec:
    """The robot description (cached per params path, validated on first build).
    A deep copy: callers may mutate what they get (the ik dicts), the cache is safe."""
    p = os.path.normpath(path or PARAMS_PATH)
    if p not in _ROBOT_CACHE:
        fields, errors, warns = _resolve(params(p))
        if errors:
            raise RobotSpecError(f"{p}: robot description is invalid:\n  - " + "\n  - ".join(errors))
        _ROBOT_CACHE[p] = (RobotSpec(**fields), warns)
    return copy.deepcopy(_ROBOT_CACHE[p][0])


def robot_warnings(path: str | None = None) -> list:
    robot(path)
    return list(_ROBOT_CACHE[os.path.normpath(path or PARAMS_PATH)][1])


def _spec(spec: RobotSpec | None) -> RobotSpec:
    return robot() if spec is None else spec


def n_legs(spec: RobotSpec | None = None) -> int:
    return _spec(spec).n_legs


def stations_deg(spec: RobotSpec | None = None) -> tuple:
    """Leg station angles (deg, floats, NOT wrapped: 90, 162, 234, 306, 378)."""
    return _spec(spec).stations_deg()


def leg_joint_names(leg: int = 0, spec: RobotSpec | None = None) -> tuple:
    return _spec(spec).leg_joint_names(leg)


def n_leg_dof(leg: int = 0, spec: RobotSpec | None = None) -> int:
    return _spec(spec).legs[leg].n_dof


def uniform_dof(spec: RobotSpec | None = None) -> int:
    return _spec(spec).uniform_dof()


def servo_for(joint: str, leg: int = 0, spec: RobotSpec | None = None) -> str:
    return _spec(spec).servo_for(joint, leg)


def actuator_order(spec: RobotSpec | None = None) -> list:
    return _spec(spec).actuator_order()


def id_table(path: str | None = None) -> list:
    """Every servo on the bus, sorted by id: IdRow(sid, leg, joint, servo,
    protocol, counts, sweep_deg, limits_deg). The one table the driver, the
    bridge and the mock will build from (ROBOT_AS_DATA step 3/10)."""
    return robot(path).id_rows(params(path))


def topology(spec: RobotSpec | None = None) -> dict:
    return _spec(spec).topology()


def topology_hash(spec: RobotSpec | None = None) -> str:
    return _spec(spec).topology_hash()


def description_digest(spec: RobotSpec | None = None) -> str:
    return _spec(spec).description_digest()


def to_json(spec: RobotSpec | None = None) -> dict:
    return _spec(spec).to_json()


def robot_summary(spec: RobotSpec | None = None) -> str:
    return _spec(spec).summary()


def _leg_joints() -> tuple:
    s = robot()
    s.uniform_dof()
    return s.leg_joint_names(0)


def _all_joints() -> tuple:
    return _leg_joints() + robot().tool_names()


def __getattr__(name: str):
    """PEP 562: LEG_JOINTS / ALL_JOINTS come from the spec on first access, so a
    plain `import rocky_model` never validates the robot description."""
    if name == "LEG_JOINTS":
        return _leg_joints()
    if name == "ALL_JOINTS":
        return _all_joints()
    raise AttributeError(f"module 'rocky_model' has no attribute {name!r}")


def summary() -> str:
    a = actuator()
    lo, hi = joint_limits_deg()["hip"]
    return (f"{a['name']}: stall {a['stall_nm']:.2f} N.m (peak = sim clip), continuous "
            f"{continuous_nm():.2f} (thermal, budget {thermal()['budget']:.0f}), "
            f"no-load {a['no_load_rad_s']:.1f} rad/s, damping {damping_nms():.3f} N.m.s/rad; "
            f"speed budget loaded {servo_speed('loaded'):.1f} / free {servo_speed('free'):.1f} / "
            f"hard {servo_speed('hard'):.1f} rad/s; hip {lo:.0f}..{hi:.0f} deg; params {params_rev()}")


if __name__ == "__main__":
    import sys
    print(summary())
    print(f"foot r_c {foot_contact_radius_mm():.1f} mm, spawn_dz {spawn_dz_mm():.2f} mm, "
          f"stance torso z {stance_torso_z_m() * 1000:.2f} mm")
    try:
        print(robot_summary())
        for w in robot_warnings():
            print(f"  warning: {w}")
    except RobotSpecError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
