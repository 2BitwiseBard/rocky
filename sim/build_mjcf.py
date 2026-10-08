"""Generate pebble.xml (MuJoCo MJCF) from the shared params.yaml.

Simplified dynamics model: capsule/box geoms, masses from the budget, position
actuators clamped to the servo's PEAK (stall) torque — D052 amendment, see
below. Joint conventions MATCH the gait engine exactly (femur/knee axes
flipped so +q2 = up, q3 negative = knee down).

D052 — the sim stops flattering the servo. Every actuator / joint number
comes from params.yaml through gait/rocky_model.py (no literals here):
  * leg joint damping = stall / no-load (0.626 N.m.s/rad): the DC motor's
    torque-speed line. With the actuator saturated, the net joint torque
    falls to zero at no-load speed (2.94 N.m clip) — or at 1.91 / 0.626 =
    3.05 rad/s with the continuous clip, which is exactly the loaded speed
    budget in `joints.vel_rad_s` (rocky_model.dc_speed). Owner measurement 2026-09-24 (scratch):
    damping alone, the wave gait still walks (242 of 273 mm in 6 s at
    45 mm/s, peak joint speed 4.2 vs 7.4 rad/s ideal) and turns 145 of
    157 deg at 0.5 rad/s; adding the 1.9 N.m forcerange, walk 240 mm, turn
    103 deg, walk+turn (45, 0, 0.35) yaws 74 of 103 deg. The turn envelope
    shrinks — the right answer, the old sim was lying about it.
  * leg forcerange = STALL (2.94 N.m, peak) since the D052 amendment
    (owner, 2026-09-24): a position servo's instantaneous torque is its
    stall torque; continuous_frac (0.65 -> 1.91 N.m) is a THERMAL budget.
    With the continuous clip an unloaded joint capped at 1.911 / 0.626 =
    3.06 rad/s, below the servo's real 4.7 no-load speed and the 4.0
    free-leg budget. Now: an unloaded saturated joint reaches 2.94 / 0.626 =
    4.70 rad/s, one whose motor is held to the continuous torque 3.05
    (rocky_model.dc_speed). The sustained budget is enforced where time
    matters: rl_common.ThermalProxy (derates a hot joint's forcerange toward
    continuous) and pebble_feasibility's THERMAL_LOAD verdict (audit physics).
  * claws likewise: forcerange = SCS0009 stall (0.23 N.m, VERIFY), not 0.65 x.
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

D064 — the belly (pick 13, B97, B138). The torso had no belly: its lowest
surface was the cylinder at z +6 (124 mm over the ground), while option A's
keel tub hangs to z -55.4 (62.6 mm). Now:
  * the torso carries an explicit <inertial> (mass, CoM, full tensor) from
    sim/mass_budget.json (mass_audit's pose table: the 380 g pack and the hub
    under the deck put the CoM ~27 mm lower than the cylinders' 24.5). With an
    <inertial> MuJoCo takes no mass from the torso's geoms, so the two
    cylinders stay as the carapace's collision hull, mass 0. A budget with no
    CoM (pre-D064) falls back to the cylinders' masses, with a WARNING.
  * two massless boxes, belly_tub and belly_shelf (rocky_model.belly_boxes():
    the keel tub's outer box and the hub shelf's plate box), and three massless
    cylinders belly_shelf_post0..2 (rocky_model.belly_posts(): the shelf's Ø14
    post pads), right after the free joint, default contact (contype 1,
    conaffinity 1, condim 3) with the floor's friction. A leg folded under the
    keel now meets them (the righter's fall path did, B130), the ground meets the
    tub first, and pebble_feasibility's SELF_CONTACT names them. The nose chamfer
    (pick 10) is NOT in the box. The D064 integration replaced the shelf's one
    hull box (grown north to y 71 by the (26, 64) pad, where leg 0 touched it in
    20/20 recover1 falls and no real board is) by the plate box + the posts.
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
F_PEAK = rm.stall_nm()                 # N*m @12V: the leg actuators' forcerange (D052 amendment: peak)
F_CONT = rm.continuous_nm()            # N*m: thermal budget — NOT the clamp (rl_common.ThermalProxy)
DAMP = rm.damping_nms()                # N*m*s/rad on the 15 leg joints
LIM = rm.joint_limits_deg()
# claws: SCS0009, every number VERIFY. Clip = its stall (peak) torque like the
# legs; the prongs don't collide, so this changes nothing in the world. Claw
# damping stays a small literal: the hand v0.3 drive is not modelled yet.
CLAW_FR = rm.stall_nm("claw")
CLAW_DAMP = rm.damping_nms("claw")     # 0.23 / 9.5 = 0.0242: the claw's own DC line (review: was a literal
#                                        0.01, which let an unloaded claw run ~23 rad/s vs its 9.5 no-load)

# ---- foot contact (D052)
FOOT = P["leg"]["foot"]
R_FOOT = rm.foot_contact_radius_mm() * MM   # 6.5 mm contact radius
MU = float(FOOT["mu_slide"])
MU_T = float(FOOT["mu_torsion_m"])
FRICTION = f"{MU:g} {MU_T:g} 0.0001"   # sliding, torsional (m), rolling (m)

# mass budget (kg): derived from the CAD tree by mass_audit.py (D039:
# sim/mass_budget.json — STL volumes x material x fill factor + params mass_hw);
# the pre-D039 estimate (the budget D015 sized the servos on) stays as the
# fallback so the file always builds.
_MB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mass_budget.json")
if os.path.exists(_MB):
    import json as _json
    _mb = _json.load(open(_MB))
    M_TORSO = _mb["torso"] / 1000.0
    M_COXA = _mb["coxa"] / 1000.0
    M_FEMUR = _mb["femur"] / 1000.0
    M_TIBIA = _mb["tibia"] / 1000.0
    MASS_SOURCE = "mass_budget.json (CAD-derived, D039)"
else:
    M_TORSO = 1.35                     # deck + battery + electronics + carapace
    M_COXA = 0.14                      # fork + femur servo
    M_FEMUR = 0.03                     # link plate
    M_TIBIA = 0.17                     # knee servo + carrier + tube + SEA + hand
    MASS_SOURCE = "pre-D039 estimate (fallback)"

# the tibia budget INCLUDES the hand (mass_audit); split it without adding mass:
M_PRONG = 0.004                        # each visual claw prong (fixed one + the claw joint's)
M_FOOT = M_TIBIA * 0.3                 # tip: cone, pad, SEA slider
M_SHIN = M_TIBIA * 0.7 - 2 * M_PRONG   # D052: was 0.7 M + 2 prongs on top (+40 g robot-wide)

# D064: the torso's own mass distribution (mass_audit's pose table), None before D064
TORSO_INERTIAL = rm.torso_inertial() if os.path.exists(_MB) else None
BELLY_RGBA = {"belly_tub": "0.45 0.36 0.60 1", "belly_shelf": "0.50 0.44 0.66 1"}
BELLY_POST_RGBA = "0.50 0.44 0.66 1"   # the shelf's posts: the shelf's colour


def torso_xml() -> str:
    """The torso's mass and geoms: the explicit <inertial> + the cylinders at mass 0 (D064), or
    the pre-D064 cylinders carrying 0.75 / 0.25 of the mass; then the belly boxes."""
    lines = []
    if TORSO_INERTIAL is not None:
        ti = TORSO_INERTIAL
        pos = " ".join(f"{v * MM:.5f}" for v in ti["com_mm"])
        full = " ".join(f"{v:.6e}" for v in ti["inertia"])
        lines.append(f'<!-- the torso\'s mass, CoM and inertia: sim/mass_budget.json (mass_audit\'s pose table'
                     f'{", PROVISIONAL" if ti["provisional"] else ""}); the geoms below carry none -->')
        lines.append(f'<inertial pos="{pos}" mass="{ti["mass_g"] / 1000:.4f}" fullinertia="{full}"/>')
        m1 = m2 = "0"
    else:
        m1, m2 = f"{M_TORSO * 0.75:.4f}", f"{M_TORSO * 0.25:.4f}"
    for name, ((x0, x1), (y0, y1), (z0, z1)) in rm.belly_boxes().items():
        c = [(a + b) / 2 * MM for a, b in ((x0, x1), (y0, y1), (z0, z1))]
        h = [(b - a) / 2 * MM for a, b in ((x0, x1), (y0, y1), (z0, z1))]
        # 6 decimals (1 um): the shelf's params x (-56.2, 53.95) centre on -1.125 mm, which 5 would round
        lines.append(f'<geom name="{name}" type="box" size="{h[0]:.6f} {h[1]:.6f} {h[2]:.6f}" '
                     f'pos="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}" mass="0" friction="{FRICTION}" '
                     f'rgba="{BELLY_RGBA.get(name, "0.45 0.36 0.60 1")}"/>')
    for name, ((x, y), r, (z0, z1)) in rm.belly_posts().items():    # MuJoCo cylinders run along z
        lines.append(f'<geom name="{name}" type="cylinder" size="{r * MM:.6f} {(z1 - z0) / 2 * MM:.6f}" '
                     f'pos="{x * MM:.6f} {y * MM:.6f} {(z0 + z1) / 2 * MM:.6f}" mass="0" friction="{FRICTION}" '
                     f'rgba="{BELLY_POST_RGBA}"/>')
    lines.append(f'<geom type="cylinder" size="{RB:.4f} 0.012" pos="0 0 0.018" mass="{m1}" '
                 f'rgba="0.62 0.50 0.82 1"/>')
    lines.append(f'<geom type="cylinder" size="{RB*0.7:.4f} 0.014" pos="0 0 0.044" mass="{m2}" '
                 f'rgba="0.55 0.42 0.75 1"/>')
    return "\n".join("      " + ln for ln in lines)


def leg_xml(i):
    ang = rm.stations_deg()[i]             # D053: the spec's station (90 + 72 i, unwrapped: leg 4 is 378)
    rng = {k: f"{LIM[k][0]:g} {LIM[k][1]:g}" for k in LIM}
    return f"""
      <body name="coxa{i}" pos="{RB*np.cos(np.deg2rad(ang)):.4f} {RB*np.sin(np.deg2rad(ang)):.4f} 0" euler="0 0 {ang:g}">
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
              <joint name="claw{i}" type="hinge" axis="0 0 1" range="{rng['claw']}" damping="{CLAW_DAMP:.4f}" armature="0.001"/>
              <geom type="capsule" fromto="0 -0.004 0 0.024 -0.011 0" size="0.0035"
                    mass="{M_PRONG}" rgba="0.72 0.62 0.88 1" contype="0" conaffinity="0"/>
            </body>
          </body>
        </body>
      </body>"""


def actuators_xml():
    """One <position> per rm.actuator_order() entry (D053): leg joints leg-major,
    then the tools — the ctrl[:15] / ctrl[15:20] order every consumer relies on.
    The kp/kv literals stay here per kind until they move into params
    (docs/ROBOT_AS_DATA.md step 2)."""
    tools = set(rm.robot().tool_names())
    out = []
    for name in rm.actuator_order():
        if name.rstrip("0123456789") in tools:
            out.append(f'    <position name="{name}" joint="{name}" kp="0.5" kv="0.02" '
                       f'forcerange="-{CLAW_FR:.4f} {CLAW_FR:.4f}"/>')
        else:
            out.append(f'    <position name="{name}" joint="{name}" kp="20" kv="0.6" '
                       f'forcerange="-{F_PEAK:.4f} {F_PEAK:.4f}"/>')
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
{torso_xml()}
      {"".join(leg_xml(i) for i in range(rm.n_legs()))}
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
    if TORSO_INERTIAL is None:
        print("WARNING: mass_budget.json has no torso_com_mm / torso_inertia (pre-D064, or missing): the "
              "torso's mass sits on the two cylinders (CoM z 24.5) — run sim/mass_audit.py")
    elif TORSO_INERTIAL["provisional"]:
        print("WARNING: mass_budget.json is PROVISIONAL (mass_audit --allow-missing): re-run mass_audit "
              "on the built CAD tree, then this")
    ti = TORSO_INERTIAL
    if ti is not None:
        print(f"torso <inertial>: {ti['mass_g']:.1f} g at ({', '.join(f'{v:.2f}' for v in ti['com_mm'])}) mm | "
              "belly: " + ", ".join(f"{k} x {a[0]:g}..{a[1]:g} y {b[0]:g}..{b[1]:g} z {c[0]:g}..{c[1]:g}"
                                    for k, (a, b, c) in rm.belly_boxes().items())
              + ", posts " + ", ".join(f"Ø{2 * r:g} at ({x:g}, {y:g}) z {z0:g}..{z1:g}"
                                       for (x, y), r, (z0, z1) in rm.belly_posts().values()))
    print("wrote", path, "| masses:", MASS_SOURCE,
          f"torso {M_TORSO:.3f} coxa {M_COXA:.3f} femur {M_FEMUR:.3f} tibia {M_TIBIA:.3f} kg"
          f" | leg servo: forcerange {F_PEAK:.3f} N.m (peak; continuous {F_CONT:.3f} is thermal),"
          f" damping {DAMP:.3f} N.m.s/rad -> {rm.dc_speed():.2f} rad/s free,"
          f" {rm.dc_speed(F_CONT):.2f} at continuous"
          f" | foot r {R_FOOT*1000:.1f} mm, mu {MU:g}")


if __name__ == "__main__":
    main()
