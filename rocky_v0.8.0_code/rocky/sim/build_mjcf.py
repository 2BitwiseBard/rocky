"""Generate pebble.xml (MuJoCo MJCF) from the shared params.yaml.

Simplified dynamics model: capsule/box geoms, masses from the budget, position
actuators with ST3215 stall-torque clamps. Joint conventions MATCH the gait
engine exactly (femur/knee axes flipped so +q2 = up, q3 negative = knee down).
"""
import os, yaml
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "..", "cad", "params.yaml")) as f:
    P = yaml.safe_load(f)

MM = 1e-3
L1 = P["leg"]["l1_coxa"] * MM
L2 = P["leg"]["l2_femur"] * MM
L3 = P["leg"]["l3_tibia"] * MM
RB = P["body"]["circumradius"] * MM
Z_HIP = 58.0 * MM
STALL = 2.94                       # N*m @12V

# mass budget (kg). Session 8: derived from the CAD tree by mass_audit.py
# (sim/mass_budget.json — STL volumes x PLA x fill factor + params.mass_hw);
# the session-2 hand budget stays as the fallback so the file always builds.
_MB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mass_budget.json")
if os.path.exists(_MB):
    import json as _json
    _mb = _json.load(open(_MB))
    M_TORSO = _mb["torso"] / 1000.0
    M_COXA = _mb["coxa"] / 1000.0
    M_FEMUR = _mb["femur"] / 1000.0
    M_TIBIA = _mb["tibia"] / 1000.0
    MASS_SOURCE = "mass_budget.json (CAD-derived, session 8)"
else:
    M_TORSO = 1.35                     # deck + battery + electronics + carapace
    M_COXA = 0.14                      # fork + femur servo
    M_FEMUR = 0.03                     # link plate
    M_TIBIA = 0.17                     # knee servo + carrier + tube + SEA + hand
    MASS_SOURCE = "hand budget (session 2)"

def leg_xml(i):
    ang = 90 + 72 * i
    return f"""
      <body name="coxa{i}" pos="{RB*np.cos(np.deg2rad(ang)):.4f} {RB*np.sin(np.deg2rad(ang)):.4f} 0" euler="0 0 {ang}">
        <joint name="yaw{i}" type="hinge" axis="0 0 1" range="-40 40" damping="0.05" armature="0.005"/>
        <geom type="box" size="0.030 0.016 0.030" pos="0.020 0 0.030" mass="{M_COXA}" rgba="0.35 0.30 0.45 1"/>
        <body name="femur{i}" pos="{L1:.4f} 0 {Z_HIP:.4f}">
          <joint name="hip{i}" type="hinge" axis="0 -1 0" range="-70 90" damping="0.05" armature="0.005"/>
          <geom type="capsule" fromto="0 0 0 {L2:.4f} 0 0" size="0.012" mass="{M_FEMUR}" rgba="0.55 0.42 0.75 1"/>
          <body name="tibia{i}" pos="{L2:.4f} 0 0">
            <joint name="knee{i}" type="hinge" axis="0 -1 0" range="-150 -20" damping="0.05" armature="0.005"/>
            <geom type="capsule" fromto="0 0 0 {L3*0.92:.4f} 0 0" size="0.010" mass="{M_TIBIA*0.7:.3f}" rgba="0.55 0.42 0.75 1"/>
            <geom name="foot{i}" type="sphere" pos="{L3:.4f} 0 0" size="0.013" mass="{M_TIBIA*0.3:.3f}"
                  rgba="0.42 0.32 0.58 1" friction="1.2 0.01 0.001" condim="3"/>
            <geom type="capsule" fromto="{L3:.4f} 0.004 0 {L3+0.024:.4f} 0.011 0" size="0.0035"
                  mass="0.004" rgba="0.72 0.62 0.88 1" contype="0" conaffinity="0"/>
            <body name="clawb{i}" pos="{L3:.4f} 0 0">
              <joint name="claw{i}" type="hinge" axis="0 0 1" range="0 55" damping="0.01" armature="0.001"/>
              <geom type="capsule" fromto="0 -0.004 0 0.024 -0.011 0" size="0.0035"
                    mass="0.004" rgba="0.72 0.62 0.88 1" contype="0" conaffinity="0"/>
            </body>
          </body>
        </body>
      </body>"""

def actuators_xml():
    out = []
    for i in range(5):
        for j in ("yaw", "hip", "knee"):
            out.append(f'    <position name="{j}{i}" joint="{j}{i}" kp="20" kv="0.6" '
                       f'forcerange="-{STALL} {STALL}"/>')
    for i in range(5):
        out.append(f'    <position name="claw{i}" joint="claw{i}" kp="0.5" kv="0.02" '
                   f'forcerange="-0.2 0.2"/>')
    return "\n".join(out)

xml = f"""<mujoco model="pebble">
  <option timestep="0.002" integrator="implicitfast"/>
  <visual>
    <global offwidth="720" offheight="480"/>
    <headlight ambient="0.4 0.4 0.45" diffuse="0.7 0.7 0.7"/>
    <quality shadowsize="2048"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.16 0.14 0.20" rgb2="0.22 0.19 0.29" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="12 12" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0.5 -0.5 1.5" dir="-0.3 0.3 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="6 6 0.1" material="grid" friction="1.2 0.01 0.001"/>
    <body name="torso" pos="0 0 0.135">
      <freejoint/>
      <geom type="cylinder" size="{RB:.4f} 0.012" pos="0 0 0.018" mass="{M_TORSO*0.75:.3f}" rgba="0.62 0.50 0.82 1"/>
      <geom type="cylinder" size="{RB*0.7:.4f} 0.014" pos="0 0 0.044" mass="{M_TORSO*0.25:.3f}" rgba="0.55 0.42 0.75 1"/>
      {"".join(leg_xml(i) for i in range(5))}
    </body>
  </worldbody>
  <actuator>
{actuators_xml()}
  </actuator>
</mujoco>"""

path = os.path.join(HERE, "pebble.xml")
with open(path, "w") as f:
    f.write(xml)
print("wrote", path, "| masses:", MASS_SOURCE, f"torso {M_TORSO:.3f} coxa {M_COXA:.3f} femur {M_FEMUR:.3f} tibia {M_TIBIA:.3f} kg")
