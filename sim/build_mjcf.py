"""Generate pebble.xml (MuJoCo MJCF) from the shared params.yaml.

Simplified dynamics model: capsule/box geoms, masses from the budget, position
actuators clamped to the servo's CONTINUOUS torque. Joint conventions MATCH the
gait engine exactly (femur/knee axes flipped so +q2 = up, q3 negative = knee down).

D052 — the sim stops flattering the servo. Every actuator / joint number
comes from params.yaml through gait/rocky_model.py (no literals here):
  * leg joint damping = stall / no-load (0.626 N.m.s/rad): the DC motor's
    torque-speed line. With the actuator saturated, the net joint torque
    falls to zero at no-load speed (2.94 N.m clip) — or at 1.91 / 0.626 =
    3.05 rad/s with the continuous clip, which is exactly the loaded speed
    budget in `joints.vel_rad_s`. Owner measurement 2026-09-24 (scratch):
    damping alone, the wave gait still walks (242 of 273 mm in 6 s at
    45 mm/s, peak joint speed 4.2 vs 7.4 rad/s ideal) and turns 145 of
    157 deg at 0.5 rad/s; adding the 1.9 N.m forcerange, walk 240 mm, turn
    103 deg, walk+turn (45, 0, 0.35) yaws 74 of 103 deg. The turn envelope
    shrinks — the right answer, the old sim was lying about it.
  * leg forcerange = continuous_frac x stall (1.91 N.m). Stall stays
    available (rocky_model.stall_nm) for bodyweight quotes, not as a budget.
  * foot/floor friction 0.8 (was 1.2; TPU on tile is 0.5-0.8); feet are
    condim 4 with torsional friction so a pad twisting in place costs torque.
  * the 40 g claw double-count is gone: mass_audit already puts the hand
    (hub, cam, fingers, claw servo) into the tibia budget, so the two claw
    prong capsules are carved OUT of the tibia link, not added on top.
    Compiled total = the mass budget (2665 g at D047).
  * the foot sphere is the real contact radius (cone tip 3.5 + TPU crown 3.0
    = 6.5 mm) centred r_c BEHIND the l3 point, so its surface — not its
    centre — is the gait's IK foot point. Site `foot_tip{i}` marks that
    point. Stance deck height is now h (+~0.4 mm), not h + 13: spawn with
    rocky_model.spawn_dz_mm() (1.4 mm), not the old hard-coded 14.
"""
import os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
import rocky_model as rm                                               # noqa: E402

P = rm.params()

MM = 1e-3
L1 = P["leg"]["l1_coxa"] * MM
L2 = P["leg"]["l2_femur"] * MM
L3 = P["leg"]["l3_tibia"] * MM
RB = P["body"]["circumradius"] * MM
Z_HIP = P["leg"]["hip_axis_z"] * MM   # params SSOT (D047)

# ---- actuator physics (D052: all derived, see the module docstring)
STALL = rm.stall_nm()                  # N*m @12V — NOT used as the clamp any more
F_CONT = rm.continuous_nm()            # N*m: the leg actuators' forcerange
DAMP = rm.damping_nms()                # N*m*s/rad on the 15 leg joints
LIM = rm.joint_limits_deg()
# claws: SCS0009, every number VERIFY. Clip = its continuous budget too (was
# a literal 0.2 here vs 0.23 in the URDF); the prongs don't collide, so this
# changes nothing in the world. Claw damping stays a small literal: the hand
# v0.3 drive is not modelled yet.
CLAW_FR = rm.continuous_nm("claw")

# ---- foot contact (D052)
FOOT = P["leg"]["foot"]
R_FOOT = rm.foot_contact_radius_mm() * MM   # 6.5 mm contact radius
MU = float(FOOT["mu_slide"])
MU_T = float(FOOT["mu_torsion_m"])
FRICTION = f"{MU:g} {MU_T:g} 0.0001"   # sliding, torsional (m), rolling (m)

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

# the tibia budget INCLUDES the hand (mass_audit); split it without adding mass:
M_PRONG = 0.004                        # each visual claw prong (fixed one + the claw joint's)
M_FOOT = M_TIBIA * 0.3                 # tip: cone, pad, SEA slider
M_SHIN = M_TIBIA * 0.7 - 2 * M_PRONG   # D052: was 0.7 M + 2 prongs on top (+40 g robot-wide)


def leg_xml(i):
    ang = 90 + 72 * i
    rng = {k: f"{LIM[k][0]:g} {LIM[k][1]:g}" for k in LIM}
    return f"""
      <body name="coxa{i}" pos="{RB*np.cos(np.deg2rad(ang)):.4f} {RB*np.sin(np.deg2rad(ang)):.4f} 0" euler="0 0 {ang}">
        <joint name="yaw{i}" type="hinge" axis="0 0 1" range="{rng['yaw']}" damping="{DAMP:.4f}" armature="0.005"/>
        <geom type="box" size="0.030 0.016 0.030" pos="0.020 0 0.030" mass="{M_COXA}" rgba="0.35 0.30 0.45 1"/>
        <body name="femur{i}" pos="{L1:.4f} 0 {Z_HIP:.4f}">
          <joint name="hip{i}" type="hinge" axis="0 -1 0" range="{rng['hip']}" damping="{DAMP:.4f}" armature="0.005"/>
          <geom type="capsule" fromto="0 0 0 {L2:.4f} 0 0" size="0.012" mass="{M_FEMUR}" rgba="0.55 0.42 0.75 1"/>
          <body name="tibia{i}" pos="{L2:.4f} 0 0">
            <joint name="knee{i}" type="hinge" axis="0 -1 0" range="{rng['knee']}" damping="{DAMP:.4f}" armature="0.005"/>
            <geom type="capsule" fromto="0 0 0 {L3*0.92:.4f} 0 0" size="0.010" mass="{M_SHIN:.4f}" rgba="0.55 0.42 0.75 1"/>
            <geom name="foot{i}" type="sphere" pos="{L3 - R_FOOT:.4f} 0 0" size="{R_FOOT:.4f}" mass="{M_FOOT:.4f}"
                  rgba="0.42 0.32 0.58 1" friction="{FRICTION}" condim="4"/>
            <site name="foot_tip{i}" pos="{L3:.4f} 0 0" size="0.002" rgba="1 0.6 0.2 1"/>
            <geom type="capsule" fromto="{L3:.4f} 0.004 0 {L3+0.024:.4f} 0.011 0" size="0.0035"
                  mass="{M_PRONG}" rgba="0.72 0.62 0.88 1" contype="0" conaffinity="0"/>
            <body name="clawb{i}" pos="{L3:.4f} 0 0">
              <joint name="claw{i}" type="hinge" axis="0 0 1" range="{rng['claw']}" damping="0.01" armature="0.001"/>
              <geom type="capsule" fromto="0 -0.004 0 0.024 -0.011 0" size="0.0035"
                    mass="{M_PRONG}" rgba="0.72 0.62 0.88 1" contype="0" conaffinity="0"/>
            </body>
          </body>
        </body>
      </body>"""


def actuators_xml():
    out = []
    for i in range(5):
        for j in ("yaw", "hip", "knee"):
            out.append(f'    <position name="{j}{i}" joint="{j}{i}" kp="20" kv="0.6" '
                       f'forcerange="-{F_CONT:.4f} {F_CONT:.4f}"/>')
    for i in range(5):
        out.append(f'    <position name="claw{i}" joint="claw{i}" kp="0.5" kv="0.02" '
                   f'forcerange="-{CLAW_FR:.4f} {CLAW_FR:.4f}"/>')
    return "\n".join(out)


SPAWN_Z = rm.spawn_z_m()               # h + spawn_dz_mm (1.4 mm), was a literal 0.135


def build_xml() -> str:
    """The MJCF text (pure: tests compare it against the committed pebble.xml)."""
    return f"""<mujoco model="pebble">
  <!-- AUTO-GENERATED by sim/build_mjcf.py from cad/params.yaml (params {rm.params_rev()}) — edit THOSE -->
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
    <geom name="floor" type="plane" size="6 6 0.1" material="grid" friction="{FRICTION}"/>
    <body name="torso" pos="0 0 {SPAWN_Z:.4f}">
      <freejoint/>
      <geom type="cylinder" size="{RB:.4f} 0.012" pos="0 0 0.018" mass="{M_TORSO*0.75:.4f}" rgba="0.62 0.50 0.82 1"/>
      <geom type="cylinder" size="{RB*0.7:.4f} 0.014" pos="0 0 0.044" mass="{M_TORSO*0.25:.4f}" rgba="0.55 0.42 0.75 1"/>
      {"".join(leg_xml(i) for i in range(5))}
    </body>
  </worldbody>
  <actuator>
{actuators_xml()}
  </actuator>
</mujoco>"""


def main():
    path = os.path.join(HERE, "pebble.xml")
    with open(path, "w") as f:
        f.write(build_xml())
    print("wrote", path, "| masses:", MASS_SOURCE,
          f"torso {M_TORSO:.3f} coxa {M_COXA:.3f} femur {M_FEMUR:.3f} tibia {M_TIBIA:.3f} kg"
          f" | leg servo: forcerange {F_CONT:.3f} N.m, damping {DAMP:.3f} N.m.s/rad"
          f" | foot r {R_FOOT*1000:.1f} mm, mu {MU:g}")


if __name__ == "__main__":
    main()
