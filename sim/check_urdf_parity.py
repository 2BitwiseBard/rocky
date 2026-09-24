#!/usr/bin/env python3
"""URDF <-> MJCF parity check: one robot, two model files, zero drift.

Loads ros2/rocky_description/urdf/pebble.urdf and sim/pebble.xml into MuJoCo,
then verifies:
  1. joint inventory: same 20 actuated joints, same axes, same ranges
  2. FK parity: for N random in-range configurations, every foot lands at the
     same body-frame position in both models AND matches the gait engine's
     closed-form leg_fk (the third witness)
  3. mass budget: total mass agrees
  4. actuator limits (D052 amendment): every URDF <limit effort> equals the
     MJCF actuator forcerange (both = the servo's PEAK / stall torque) and
     every URDF velocity equals the servo's no-load speed

Run after any params.yaml change:  python3 check_urdf_parity.py
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import leg_fk, leg_to_body, N_LEGS                # noqa: E402

URDF = os.path.join(HERE, "..", "ros2", "rocky_description", "urdf", "pebble.urdf")
MJCF = os.path.join(HERE, "pebble.xml")
N_SAMPLES = 60
TOL_MM = 0.5


def body_frame_foot(model, data, torso_name, i):
    """Foot point in torso frame. MJCF: site foot_tip{i} (D052: the foot
    sphere's CENTRE sits r_c behind the IK point, so the geom is no longer
    the foot point; the site is). URDF (MuJoCo import merges fixed links,
    foot{i} fuses into tibia{i}): the claw{i}_link body frame origin
    coincides with the foot point — use that."""
    t = model.body(torso_name).id
    R = data.xmat[t].reshape(3, 3)
    p0 = data.xpos[t]
    try:
        pw = data.site_xpos[model.site(f"foot_tip{i}").id]
    except KeyError:
        pw = data.xpos[model.body(f"claw{i}_link").id]
    return R.T @ (pw - p0)


def foot_contact_offset(model_u, model_m):
    """Where each model's foot contact sphere sits relative to the foot point:
    both must put it r_c behind it, with radius r_c (D052)."""
    gm = model_m.geom("foot0")
    # URDF: the fused tibia carries the foot collision sphere as its only sphere geom
    tib = model_u.body("tibia0").id
    sph = [g for g in range(model_u.ngeom) if model_u.geom_bodyid[g] == tib
           and model_u.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE]
    gu = model_u.geom(sph[0]) if sph else None
    return gm, gu


def set_joints(model, data, q):
    for i in range(N_LEGS):
        for name, val in (("yaw", q[i][0]), ("hip", q[i][1]), ("knee", q[i][2])):
            j = model.joint(f"{name}{i}")
            data.qpos[j.qposadr[0]] = val
    mujoco.mj_forward(model, data)


def main():
    m_u = mujoco.MjModel.from_xml_path(URDF)
    m_m = mujoco.MjModel.from_xml_path(MJCF)
    d_u, d_m = mujoco.MjData(m_u), mujoco.MjData(m_m)
    report = {"joints": {}, "fk": {}, "mass": {}}
    fails = []

    # -- 1. joint inventory ------------------------------------------------
    names = [f"{j}{i}" for i in range(N_LEGS) for j in ("yaw", "hip", "knee", "claw")]
    for n in names:
        ju, jm = m_u.joint(n), m_m.joint(n)
        ax_ok = np.allclose(ju.axis, jm.axis, atol=1e-9)
        rg_ok = np.allclose(ju.range, jm.range, atol=np.deg2rad(0.01))
        report["joints"][n] = dict(axis_ok=bool(ax_ok), range_ok=bool(rg_ok),
                                   range_deg=[round(float(x), 1) for x in
                                              np.rad2deg(ju.range)])
        if not (ax_ok and rg_ok):
            fails.append(f"joint {n}: axis_ok={ax_ok} range_ok={rg_ok} "
                         f"urdf={ju.axis}/{np.rad2deg(ju.range)} "
                         f"mjcf={jm.axis}/{np.rad2deg(jm.range)}")
    print(f"joint inventory: {len(names)} joints checked, "
          f"{sum(1 for f in fails if f.startswith('joint'))} mismatches")

    # -- 2. FK parity ------------------------------------------------------
    rng = np.random.default_rng(7)
    worst_um = worst_ua = 0.0
    for _ in range(N_SAMPLES):
        q = np.stack([
            rng.uniform([-0.6, -0.9, -2.4], [0.6, 1.2, -0.4]) for _ in range(N_LEGS)])
        set_joints(m_u, d_u, q)
        set_joints(m_m, d_m, q)
        for i in range(N_LEGS):
            # URDF root merges into MuJoCo's world -> torso frame is 'world'
            fu = body_frame_foot(m_u, d_u, "world", i) * 1000
            fm = body_frame_foot(m_m, d_m, "torso", i) * 1000
            fa = leg_to_body(i, leg_fk(q[i]))            # analytic, mm
            worst_um = max(worst_um, float(np.linalg.norm(fu - fm)))
            worst_ua = max(worst_ua, float(np.linalg.norm(fu - fa)))
    report["fk"] = dict(samples=N_SAMPLES, worst_urdf_vs_mjcf_mm=round(worst_um, 4),
                        worst_urdf_vs_analytic_mm=round(worst_ua, 4), tol_mm=TOL_MM)
    print(f"FK parity over {N_SAMPLES} random configs x {N_LEGS} legs:")
    print(f"  URDF vs MJCF     worst {worst_um:.4f} mm")
    print(f"  URDF vs analytic worst {worst_ua:.4f} mm")
    if worst_um > TOL_MM or worst_ua > TOL_MM:
        fails.append(f"FK mismatch: um={worst_um:.3f} ua={worst_ua:.3f} mm")

    # -- 2b. foot contact sphere (D052): same radius, same place ------------
    gm, gu = foot_contact_offset(m_u, m_m)
    if gu is None:
        fails.append("URDF tibia0 has no foot collision sphere")
    else:
        ok = (abs(gm.size[0] - gu.size[0]) < 1e-6
              and np.allclose(gm.pos, gu.pos, atol=1e-6))
        report["foot_sphere"] = dict(r_mm=round(float(gm.size[0]) * 1000, 3),
                                     mjcf_pos=[round(float(x), 5) for x in gm.pos],
                                     urdf_pos=[round(float(x), 5) for x in gu.pos], ok=bool(ok))
        print(f"foot sphere: r {gm.size[0]*1000:.2f} mm, mjcf {gm.pos}, urdf {gu.pos} "
              f"-> {'ok' if ok else 'MISMATCH'}")
        if not ok:
            fails.append("foot contact sphere differs between URDF and MJCF")

    # -- 3. mass -----------------------------------------------------------
    # MuJoCo's URDF import fuses the fixed base_link into `world`, dropping
    # its mass from body_mass — so sum the URDF's declared masses directly.
    import xml.etree.ElementTree as ET
    mass_u = sum(float(el.get("value"))
                 for el in ET.parse(URDF).getroot().iter("mass"))
    mass_m = float(m_m.body_mass.sum())
    report["mass"] = dict(urdf_kg=round(mass_u, 4), mjcf_kg=round(mass_m, 4))
    print(f"total mass: urdf {mass_u:.3f} kg vs mjcf {mass_m:.3f} kg")
    if abs(mass_u - mass_m) > 0.02:
        fails.append(f"mass mismatch {mass_u:.3f} vs {mass_m:.3f}")

    # -- 4. effort / velocity vs the MJCF actuators (D052 amendment) ------
    sys.path.insert(0, os.path.join(HERE, "..", "gait"))
    import rocky_model as rm
    worst_eff, worst_vel, worst_dc, n_lim = 0.0, 0.0, 0.0, 0
    for j in ET.parse(URDF).getroot().iter("joint"):
        name = j.get("name")
        kind = name.rstrip("0123456789")
        if kind not in ("yaw", "hip", "knee", "claw"):
            continue
        L = j.find("limit")
        fr = float(m_m.actuator_forcerange[m_m.actuator(name).id][1])
        worst_eff = max(worst_eff, abs(float(L.get("effort")) - fr))
        v_want = rm.no_load_rad_s("claw" if kind == "claw" else None)
        worst_vel = max(worst_vel, abs(float(L.get("velocity")) - v_want))
        # the MJCF's own DC line: an unloaded joint at full drive tops out at
        # forcerange / damping, which must be the no-load speed too (review: the
        # claws had a literal 0.01 damping -> 23 rad/s against a 9.5 no-load)
        dof = int(m_m.jnt_dofadr[m_m.joint(name).id])
        worst_dc = max(worst_dc, abs(fr / float(m_m.dof_damping[dof]) - v_want))
        n_lim += 1
    report["limits"] = dict(joints=n_lim, worst_effort_vs_forcerange_nm=round(worst_eff, 5),
                            worst_velocity_vs_no_load=round(worst_vel, 5),
                            worst_mjcf_dc_speed_vs_no_load=round(worst_dc, 4),
                            leg_effort_nm=rm.stall_nm(), leg_velocity=rm.no_load_rad_s())
    print(f"actuator limits: {n_lim} joints, URDF effort vs MJCF forcerange worst {worst_eff:.5f} N.m, "
          f"velocity vs no-load worst {worst_vel:.5f} rad/s, MJCF forcerange/damping vs no-load worst "
          f"{worst_dc:.4f} rad/s (leg peak {rm.stall_nm():g} N.m)")
    if n_lim != 20 or worst_eff > 1e-3 or worst_vel > 1e-6 or worst_dc > 0.05:
        fails.append(f"actuator limits: {n_lim} joints, effort off by {worst_eff:.4f} N.m, "
                     f"velocity off by {worst_vel:.4f} rad/s, MJCF DC speed off by {worst_dc:.3f} rad/s")

    report["pass"] = not fails
    with open(os.path.join(HERE, "urdf_parity.json"), "w") as f:
        json.dump(report, f, indent=1)
    print("wrote urdf_parity.json")
    if fails:
        print("\nPARITY FAILURES:")
        for f_ in fails:
            print(" -", f_)
        return 1
    print("PARITY OK — description and sim are the same robot")
    return 0


if __name__ == "__main__":
    sys.exit(main())
