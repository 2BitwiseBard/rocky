#!/usr/bin/env python3
"""Generate pebble.urdf.xacro (+ expanded pebble.urdf) from cad/params.yaml.

Single source of truth (D004): the same params file drives CAD, the MJCF
(sim/build_mjcf.py), and this URDF. One leg macro instantiated 5x at 72 deg.
Joint conventions MATCH the gait engine + MJCF exactly:

  yaw   axis +Z, range +/-40 deg          (soft_limits_deg.yaw)
  hip   axis  0 -1 0, +q = femur up       (range -70..90)
  knee  axis  0 -1 0, 0 = straight, -down (range -150..-20)
  claw  axis +Z (visual gripper prong)    (range 0..55)

Masses/geometry mirror sim/build_mjcf.py's budget model; URDF inertials are
computed analytically for each primitive so MuJoCo/Gazebo/RViz all agree.
D052: limits, effort, velocity and damping come from gait/rocky_model.py
(params `actuators:` / `joints:`), the same loader the MJCF uses. effort =
the PEAK (stall) torque = the MJCF forcerange since the D052 amendment
(the continuous 0.65 x stall is a thermal budget, not an instantaneous
limit — a URDF effort is the latter), velocity = the no-load speed (the
hard ceiling); the foot is the 6.5 mm contact sphere whose surface is
the foot_fix frame (the IK foot point), and the claw prongs are carved out
of the tibia budget instead of added on top.

Run:  python3 generate_urdf.py        (writes urdf/pebble.urdf.xacro + .urdf)
Verify parity against the MJCF:  python3 ../../sim/check_urdf_parity.py
"""
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "gait"))
import rocky_model as rm                                               # noqa: E402

P = rm.params()

MM = 1e-3
L1 = P["leg"]["l1_coxa"] * MM
L2 = P["leg"]["l2_femur"] * MM
L3 = P["leg"]["l3_tibia"] * MM
RB = P["body"]["circumradius"] * MM
Z_HIP = P["leg"]["hip_axis_z"] * MM     # femur pivot height (params SSOT, D047)
LIM = rm.joint_limits_deg()             # joints.pos_deg (D052; bus.soft_limits_deg aliases it)
EFFORT = rm.stall_nm()                  # N*m: peak = the MJCF forcerange (D052 amendment; was continuous)
VEL = rm.servo_speed("hard")            # rad/s: no-load, 0.222 s/60 deg @ 12 V (was a stray 5.2)
DAMP = rm.damping_nms()                 # N*m*s/rad: stall / no-load, = the MJCF joint damping
CLAW_DAMP = rm.damping_nms("claw")      # the SCS0009's line (0.0242), = the MJCF claw damping
CLAW_EFFORT = rm.stall_nm("claw")       # SCS0009 peak, all VERIFY
CLAW_VEL = rm.no_load_rad_s("claw")
R_FOOT = rm.foot_contact_radius_mm() * MM   # contact sphere, centred R_FOOT behind the foot point
M_PRONG = 0.004                         # each visual claw prong (MJCF parity)

# mass budget: the CAD-derived sim/mass_budget.json (D039) — the same file
# sim/build_mjcf.py reads, so URDF and MJCF cannot drift apart again (they did:
# D046 updated the budget, this file still carried the session-2 hand budget)
_MB_PATH = os.path.join(REPO, "sim", "mass_budget.json")
if os.path.exists(_MB_PATH):
    import json
    with open(_MB_PATH) as _f:
        _mb = json.load(_f)
    M_TORSO, M_COXA, M_FEMUR, M_TIBIA = (_mb[k] / 1000.0 for k in ("torso", "coxa", "femur", "tibia"))
    MASS_SOURCE = "mass_budget.json"
else:                                    # session-2 hand budget, kept as the fallback
    M_TORSO, M_COXA, M_FEMUR, M_TIBIA = 1.35, 0.14, 0.03, 0.17
    MASS_SOURCE = "fallback constants"


def inertia_cylinder(m, r, h):
    ixx = m * (3 * r * r + h * h) / 12
    return ixx, ixx, m * r * r / 2


def inertia_box(m, x, y, z):
    return (m * (y * y + z * z) / 12, m * (x * x + z * z) / 12,
            m * (x * x + y * y) / 12)


def inertia_capsule_x(m, r, length):
    """Capsule along X approximated as a cylinder (fine at these masses)."""
    iyy = m * (3 * r * r + length * length) / 12
    return m * r * r / 2, iyy, iyy


def inertial(m, i, com=(0, 0, 0)):
    return (f'<inertial><origin xyz="{com[0]:.6f} {com[1]:.6f} {com[2]:.6f}"/>'
            f'<mass value="{m}"/>'
            f'<inertia ixx="{i[0]:.3e}" ixy="0" ixz="0" '
            f'iyy="{i[1]:.3e}" iyz="0" izz="{i[2]:.3e}"/></inertial>')


def deg(v):
    return f"{np.deg2rad(v):.6f}"


PURPLE = '<material name="pebble"><color rgba="0.55 0.42 0.75 1"/></material>'


def leg_macro() -> str:
    coxa_i = inertia_box(M_COXA, 0.060, 0.032, 0.060)
    femur_i = inertia_capsule_x(M_FEMUR, 0.012, L2)
    # D052: the tibia budget already holds the hand — prongs come OUT of the shin
    tib_m_link, tib_m_foot = M_TIBIA * 0.7 - 2 * M_PRONG, M_TIBIA * 0.3
    tibia_i = inertia_capsule_x(tib_m_link, 0.010, L3 * 0.92)
    foot_i = tuple([2 / 5 * tib_m_foot * R_FOOT ** 2] * 3)
    return f"""
  <xacro:macro name="pebble_leg" params="i angle">
    <link name="coxa${{i}}">
      {inertial(M_COXA, coxa_i, (0.020, 0, 0.030))}
      <visual><origin xyz="0.020 0 0.030"/>
        <geometry><box size="0.060 0.032 0.060"/></geometry>
        <material name="pebble"/></visual>
      <collision><origin xyz="0.020 0 0.030"/>
        <geometry><box size="0.060 0.032 0.060"/></geometry></collision>
    </link>
    <joint name="yaw${{i}}" type="revolute">
      <parent link="base_link"/><child link="coxa${{i}}"/>
      <origin xyz="${{{RB:.4f}*cos(angle)}} ${{{RB:.4f}*sin(angle)}} 0"
              rpy="0 0 ${{angle}}"/>
      <axis xyz="0 0 1"/>
      <limit lower="{deg(LIM['yaw'][0])}" upper="{deg(LIM['yaw'][1])}"
             effort="{EFFORT:.4f}" velocity="{VEL}"/>
      <dynamics damping="{DAMP:.4f}"/>
    </joint>

    <link name="femur${{i}}">
      {inertial(M_FEMUR, femur_i, (L2 / 2, 0, 0))}
      <visual><origin xyz="{L2 / 2:.4f} 0 0" rpy="0 {np.pi / 2:.6f} 0"/>
        <geometry><cylinder radius="0.012" length="{L2:.4f}"/></geometry>
        <material name="pebble"/></visual>
      <collision><origin xyz="{L2 / 2:.4f} 0 0" rpy="0 {np.pi / 2:.6f} 0"/>
        <geometry><cylinder radius="0.012" length="{L2:.4f}"/></geometry></collision>
    </link>
    <joint name="hip${{i}}" type="revolute">
      <parent link="coxa${{i}}"/><child link="femur${{i}}"/>
      <origin xyz="{L1:.4f} 0 {Z_HIP:.4f}"/>
      <axis xyz="0 -1 0"/>
      <limit lower="{deg(LIM['hip'][0])}" upper="{deg(LIM['hip'][1])}"
             effort="{EFFORT:.4f}" velocity="{VEL}"/>
      <dynamics damping="{DAMP:.4f}"/>
    </joint>

    <link name="tibia${{i}}">
      {inertial(tib_m_link, tibia_i, (L3 * 0.92 / 2, 0, 0))}
      <visual><origin xyz="{L3 * 0.92 / 2:.4f} 0 0" rpy="0 {np.pi / 2:.6f} 0"/>
        <geometry><cylinder radius="0.010" length="{L3 * 0.92:.4f}"/></geometry>
        <material name="pebble"/></visual>
      <collision><origin xyz="{L3 * 0.92 / 2:.4f} 0 0" rpy="0 {np.pi / 2:.6f} 0"/>
        <geometry><cylinder radius="0.010" length="{L3 * 0.92:.4f}"/></geometry></collision>
    </link>
    <joint name="knee${{i}}" type="revolute">
      <parent link="femur${{i}}"/><child link="tibia${{i}}"/>
      <origin xyz="{L2:.4f} 0 0"/>
      <axis xyz="0 -1 0"/>
      <limit lower="{deg(LIM['knee'][0])}" upper="{deg(LIM['knee'][1])}"
             effort="{EFFORT:.4f}" velocity="{VEL}"/>
      <dynamics damping="{DAMP:.4f}"/>
    </joint>

    <link name="foot${{i}}">
      {inertial(tib_m_foot + M_PRONG, foot_i, (-R_FOOT, 0, 0))}  <!-- + fixed claw prong (MJCF parity) -->
      <visual><origin xyz="{-R_FOOT:.4f} 0 0"/><geometry><sphere radius="{R_FOOT:.4f}"/></geometry>
        <material name="pebble"/></visual>
      <visual><origin xyz="0.012 0.0075 0" rpy="0 0 0.28"/>
        <geometry><cylinder radius="0.0035" length="0.028"/></geometry>
        <material name="pebble"/></visual>
      <collision><origin xyz="{-R_FOOT:.4f} 0 0"/><geometry><sphere radius="{R_FOOT:.4f}"/></geometry></collision>
    </link>
    <joint name="foot_fix${{i}}" type="fixed">
      <parent link="tibia${{i}}"/><child link="foot${{i}}"/>
      <origin xyz="{L3:.4f} 0 0"/>
    </joint>

    <link name="claw${{i}}_link">
      {inertial(M_PRONG, (1e-7, 1e-7, 1e-7))}
      <visual><origin xyz="0.012 -0.0075 0" rpy="0 0 -0.28"/>
        <geometry><cylinder radius="0.0035" length="0.028"/></geometry>
        <material name="pebble"/></visual>
    </link>
    <joint name="claw${{i}}" type="revolute">
      <parent link="foot${{i}}"/><child link="claw${{i}}_link"/>
      <origin xyz="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="{deg(LIM['claw'][0])}" upper="{deg(LIM['claw'][1])}"
             effort="{CLAW_EFFORT:.4f}" velocity="{CLAW_VEL}"/>
      <dynamics damping="{CLAW_DAMP:.4f}"/>
    </joint>
  </xacro:macro>
"""


def build_xacro() -> str:
    legs = "\n".join(                   # D053: stations from the spec (90 + 72 i, unwrapped)
        f'  <xacro:pebble_leg i="{i}" angle="${{radians({rm.stations_deg()[i]:g})}}"/>'
        for i in range(rm.n_legs()))
    return f"""<?xml version="1.0"?>
<!-- AUTO-GENERATED from cad/params.yaml by generate_urdf.py — edit THAT, not this -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="pebble">
  {PURPLE}

  <link name="base_link">
    __TORSO_INERTIAL__
    <visual><origin xyz="0 0 0.018"/>
      <geometry><cylinder radius="{RB:.4f}" length="0.024"/></geometry>
      <material name="pebble"/></visual>
    <visual><origin xyz="0 0 0.044"/>
      <geometry><cylinder radius="{RB * 0.7:.4f}" length="0.028"/></geometry>
      <material name="pebble"/></visual>
    <collision><origin xyz="0 0 0.018"/>
      <geometry><cylinder radius="{RB:.4f}" length="0.024"/></geometry></collision>
    <collision><origin xyz="0 0 0.044"/>
      <geometry><cylinder radius="{RB * 0.7:.4f}" length="0.028"/></geometry></collision>
  </link>
{leg_macro()}
{legs}

  <xacro:include filename="$(find rocky_description)/urdf/rocky.ros2_control.xacro"/>
</robot>
"""


def combine_torso_inertia():
    """Two stacked disks -> one inertial about a combined CoM (URDF wants one)."""
    m1, m2 = M_TORSO * 0.75, M_TORSO * 0.25
    z1, z2 = 0.018, 0.044
    zc = (m1 * z1 + m2 * z2) / (m1 + m2)
    i1 = inertia_cylinder(m1, RB, 0.024)
    i2 = inertia_cylinder(m2, RB * 0.7, 0.028)
    # parallel axis for ixx/iyy
    ixx = i1[0] + m1 * (z1 - zc) ** 2 + i2[0] + m2 * (z2 - zc) ** 2
    izz = i1[2] + i2[2]
    return (m1 + m2), (ixx, ixx, izz), (0, 0, zc)


def main():
    urdf_dir = os.path.join(HERE, "urdf")
    os.makedirs(urdf_dir, exist_ok=True)
    # base_link carries ONE inertial = both deck disks combined (parallel axis)
    m, i, com = combine_torso_inertia()
    xacro_txt = build_xacro().replace("__TORSO_INERTIAL__", inertial(m, i, com))
    xacro_path = os.path.join(urdf_dir, "pebble.urdf.xacro")
    with open(xacro_path, "w") as f:
        f.write(xacro_txt)
    print("wrote", xacro_path)

    # expanded URDF for tools without xacro (incl. our MuJoCo parity check):
    # drop the ros2_control include (plugin tags mean nothing to MuJoCo)
    plain = xacro_txt.replace(
        '  <xacro:include filename="$(find rocky_description)/urdf/'
        'rocky.ros2_control.xacro"/>\n', "")
    tmp = os.path.join(urdf_dir, "_plain.xacro")
    with open(tmp, "w") as f:
        f.write(plain)
    urdf_path = os.path.join(urdf_dir, "pebble.urdf")
    try:
        import xacro as xacro_mod
        doc = xacro_mod.process_file(tmp)
        with open(urdf_path, "w") as f:
            f.write(doc.toprettyxml(indent="  "))
        how = "xacro module"
    except ImportError:
        try:
            subprocess.run(["xacro", tmp, "-o", urdf_path], check=True)
            how = "xacro CLI"
        except (FileNotFoundError, subprocess.CalledProcessError):
            with open(urdf_path, "w") as f:      # no ROS on this machine: expand the one macro ourselves
                f.write(expand_plain(plain))
            how = "built-in expander (no xacro installed)"
    os.remove(tmp)
    print("wrote", urdf_path, f"({how}; masses: {MASS_SOURCE})")


def expand_plain(xacro_txt: str) -> str:
    """Minimal expander for THIS file: one macro, ${...} python expressions
    with i/angle bound, no other xacro features. Keeps the parity check
    runnable on a laptop without ROS."""
    import re
    m = re.search(r"<xacro:macro name=\"pebble_leg\" params=\"i angle\">(.*?)</xacro:macro>", xacro_txt, re.S)
    body = m.group(1)
    head = xacro_txt[:m.start()]
    tail = xacro_txt[m.end():]
    legs = []
    for inst in re.finditer(r'<xacro:pebble_leg i="(\d+)" angle="\$\{radians\(([\d.]+)\)\}"/>', tail):
        i, ang = int(inst.group(1)), np.radians(float(inst.group(2)))
        ns = {"i": i, "angle": ang, "cos": np.cos, "sin": np.sin, "radians": np.radians, "pi": np.pi}
        legs.append(re.sub(r"\$\{([^}]*)\}", lambda mm: str(eval(mm.group(1), {}, ns)), body))
    tail = re.sub(r'<xacro:pebble_leg [^>]*/>\n?', "", tail)
    out = head + "".join(legs) + tail
    return out.replace(' xmlns:xacro="http://www.ros.org/wiki/xacro"', "")


if __name__ == "__main__":
    sys.exit(main())
