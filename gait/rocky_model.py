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
    rm.stall_nm(), rm.continuous_nm()       # 2.94, 1.911 N.m (leg servo)
"""
from __future__ import annotations

import copy
import math
import os

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.normpath(os.path.join(_HERE, "..", "cad", "params.yaml"))

LEG_JOINTS = ("yaw", "hip", "knee")
ALL_JOINTS = LEG_JOINTS + ("claw",)
SPAWN_CLEARANCE_MM = 1.0        # feet start this far above the floor, then settle

_CACHE: dict = {}


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
    """Stall torque (N.m). Use it for bodyweight quotes, NOT as a sustained budget."""
    return actuator(name)["stall_nm"]


def continuous_nm(name: str | None = None) -> float:
    """Thermal (sustained) torque budget = continuous_frac x stall — the sim's forcerange."""
    a = actuator(name)
    return a["stall_nm"] * float(a["continuous_frac"])


def no_load_rad_s(name: str | None = None) -> float:
    return float(actuator(name)["no_load_rad_s"])


def damping_nms(name: str | None = None) -> float:
    """Slope of the DC motor torque-speed line, stall / no-load (N.m.s/rad):
    the joint damping that makes a saturated servo slow down under load."""
    a = actuator(name)
    return a["stall_nm"] / float(a["no_load_rad_s"])


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
    return {k: (float(j[k][0]), float(j[k][1])) for k in ALL_JOINTS}


def joint_limits_rad() -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(lo, hi) for (yaw, hip, knee), radians — np.tile(lo, 5) for all 15 leg joints."""
    d = joint_limits_deg()
    lo = tuple(math.radians(d[k][0]) for k in LEG_JOINTS)
    hi = tuple(math.radians(d[k][1]) for k in LEG_JOINTS)
    return lo, hi


def claw_limits(unit: str = "deg") -> tuple[float, float]:
    lo, hi = joint_limits_deg()["claw"]
    if unit.startswith("rad"):
        return math.radians(lo), math.radians(hi)
    return lo, hi


def servo_speed(kind: str = "loaded") -> float:
    """Joint speed budget (rad/s): 'loaded' (stance, carrying weight), 'free'
    (swinging leg) or 'hard' (= the leg servo's no-load speed; never exceed)."""
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
    Real servos sag ~1-2 mm below this under the D052 forcerange."""
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
                return leg, LEG_JOINTS[j]
    for leg, i in enumerate(hand_ids()):
        if i == sid:
            return leg, "claw"
    raise KeyError(f"servo id {sid} is not in params.yaml bus map")


def joint_to_id(leg: int, joint: str) -> int:
    if joint == "claw":
        return hand_ids()[leg]
    return leg_ids()[leg][LEG_JOINTS.index(joint)]


def summary() -> str:
    a = actuator()
    lo, hi = joint_limits_deg()["hip"]
    return (f"{a['name']}: stall {a['stall_nm']:.2f} N.m, continuous {continuous_nm():.2f}, "
            f"no-load {a['no_load_rad_s']:.1f} rad/s, damping {damping_nms():.3f} N.m.s/rad; "
            f"speed budget loaded {servo_speed('loaded'):.1f} / free {servo_speed('free'):.1f} / "
            f"hard {servo_speed('hard'):.1f} rad/s; hip {lo:.0f}..{hi:.0f} deg; params {params_rev()}")


if __name__ == "__main__":
    print(summary())
    print(f"foot r_c {foot_contact_radius_mm():.1f} mm, spawn_dz {spawn_dz_mm():.2f} mm, "
          f"stance torso z {stance_torso_z_m() * 1000:.2f} mm")
