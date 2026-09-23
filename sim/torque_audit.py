#!/usr/bin/env python3
"""Static stall-margin audit on the CAD-derived mass budget (session 8d).

D015 sized the servos against the SESSION-2 hand budget (1350 g torso,
140/30/170 g legs). Session 8's mass_audit (D039) replaced that with STL
volumes x material x fill — this re-runs the D015 stance cases against
the numbers the robot will actually weigh, plus the two cases D015 never
had: the self-righting push (the recover policy exists now) and a carried
payload on an untucked leg.

Method: exact statics through the same kinematics the gait uses — foot
force maps to joint torques by tau = J^T F with J the numerical Jacobian
of leg_fk at the case's actual joint angles; cantilevered (airborne) legs
get gravity torques from segment COMs at midpoints. No sim, no dynamics:
these are HOLDING torques, the thermal number that cooks a servo. Dynamic
peaks ride on top (bench day measures those, D017 re-baseline).

Servo envelopes (spec-sheet stall; running thermal comfort is the margin
we judge, not survival):
  ST3215  12 V: 30 kg*cm = 2.94 N*m     (what the frame is printed for)
  STS3250 12 V: 50 kg*cm = 4.90 N*m     (D015's sanctioned upgrade, same case)
Flags: OK < 35% of stall, WARM 35-50%, HOT > 50% (D015's language: 43-65%
was the thermal no-go; 22-33% walking was fine).

    python3 torque_audit.py          # table + out/torque_audit.json
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import (WaveGait, leg_ik, leg_fk, body_to_leg,      # noqa: E402
                         N_LEGS, L1, L2, L3)

G = 9.81
STALL = {"ST3215": 2.94, "STS3250": 4.90}          # N*m at 12 V
PAYLOAD_G = 100.0                                   # carried-pebble case


def load_masses():
    with open(os.path.join(HERE, "mass_budget.json")) as f:
        mb = json.load(f)
    m = {k: v / 1000.0 for k, v in mb.items() if not k.startswith("_")}
    m["robot"] = m["torso"] + N_LEGS * (m["coxa"] + m["femur"] +
                                        m["tibia"] + m.get("hand", 0.0))
    return m


def jac(q, eps=1e-6):
    """Numerical Jacobian of leg_fk (mm) wrt (yaw, hip, knee) — mm/rad."""
    q = np.asarray(q, float)
    J = np.zeros((3, 3))
    for j in range(3):
        dq = np.zeros(3)
        dq[j] = eps
        J[:, j] = (leg_fk(q + dq) - leg_fk(q - dq)) / (2 * eps)
    return J


def stance_torques(p_body, fz_n):
    """Joint torques (N*m) holding vertical foot force fz at body-frame
    foot point p_body (leg 0 geometry — radial symmetry makes it general)."""
    pl = body_to_leg(0, p_body)
    q = leg_ik(pl)
    assert not np.isnan(q).any(), f"unreachable stance {p_body}"
    J = jac(q) / 1000.0                    # m/rad
    F = np.array([0.0, 0.0, fz_n])         # ground pushes UP on the foot
    return J.T @ F                          # N*m per (yaw, hip, knee)


def cantilever_torques(m, extended=True, payload_kg=0.0):
    """Hip/knee gravity torque on an AIRBORNE leg. extended=True: femur and
    tibia horizontal (worst case, the untucked manipulation posture);
    else the gait's tucked swing (~55 deg knee fold)."""
    if extended:
        r_f, r_t, r_e = L2 / 2, L2 + L3 / 2, L2 + L3          # mm from hip
    else:
        r_f, r_t, r_e = L2 / 2 * 0.9, L2 * 0.9 + L3 * 0.25, L2 * 0.9 + L3 * 0.5
    tau_hip = G * (m["femur"] * r_f +
                   (m["tibia"] + m.get("hand", 0)) * r_t +
                   payload_kg * r_e) / 1000.0
    r_tk = (L3 / 2 if extended else L3 * 0.25)
    tau_knee = G * ((m["tibia"] + m.get("hand", 0)) * r_tk +
                    payload_kg * (L3 if extended else L3 * 0.5)) / 1000.0
    return tau_hip, tau_knee


def flag(pct):
    return "OK" if pct < 35 else ("WARM" if pct < 50 else "HOT")


def main():
    m = load_masses()
    g = WaveGait()
    W = m["robot"] * G
    p_nom = g.p_nom[0]
    print(f"robot {m['robot']*1000:.0f} g (torso {m['torso']*1000:.0f}, "
          f"leg {1000*(m['coxa']+m['femur']+m['tibia']+m.get('hand',0)):.0f} x5)"
          f"  |  W = {W:.2f} N")
    cases = []

    def add(name, tau_yaw, tau_hip, tau_knee, note=""):
        cases.append(dict(case=name, note=note,
                          tau=dict(yaw=round(abs(tau_yaw), 3),
                                   hip=round(abs(tau_hip), 3),
                                   knee=round(abs(tau_knee), 3))))

    # 1) 5-leg stand at nominal stance
    t = stance_torques(p_nom, W / 5)
    add("stand_5leg", *t, note="nominal stance, W/5 per foot")
    # 2) walking support: momentary 4-leg (wave duty 0.8)
    t = stance_torques(p_nom, W / 4)
    add("walk_4leg", *t, note="wave-gait stance share, W/4")
    # 3) worst walking transient / D015's 3-leg stance
    t = stance_torques(p_nom, W / 3)
    add("stance_3leg", *t, note="two legs raised, W/3 per planted foot")
    # 4) untucked manipulation: planted legs at W/3 while the raised leg
    #    cantilevers fully extended (both live at once on different servos)
    th, tk = cantilever_torques(m, extended=True)
    add("arm_untucked", 0.0, th, tk, note="airborne leg, femur+tibia horizontal")
    # 5) carry: untucked arm + payload in hand
    th, tk = cantilever_torques(m, extended=True, payload_kg=PAYLOAD_G / 1000)
    add("carry_100g", 0.0, th, tk, note=f"untucked + {PAYLOAD_G:.0f} g in hand")
    # 6) self-righting push: recovery presses ~half the robot up through
    #    ONE knee at deep fold (recover-policy strategy class, session 8)
    p_push = np.array([p_nom[0], p_nom[1] * 0.75, -60.0])
    t = stance_torques(p_push, W / 2)
    add("selfright_push", *t, note="crouched geometry, W/2 through one leg")

    print(f"\n{'case':>15s} {'joint':>5s} {'N*m':>6s}   "
          f"{'ST3215':>10s}   {'STS3250':>10s}")
    worst = {}
    for c in cases:
        for jname, tau in c["tau"].items():
            pcts = {s: 100 * tau / STALL[s] for s in STALL}
            c.setdefault("margin_pct", {})[jname] = {
                s: round(p, 1) for s, p in pcts.items()}
            f1, f2 = flag(pcts["ST3215"]), flag(pcts["STS3250"])
            print(f"{c['case']:>15s} {jname:>5s} {tau:6.2f}   "
                  f"{pcts['ST3215']:5.1f}% {f1:>4s}   "
                  f"{pcts['STS3250']:5.1f}% {f2:>4s}")
            key = (jname,)
            if tau > worst.get(key, (0, ""))[0]:
                worst[key] = (tau, c["case"])
    print("\nworst holding torque per joint:")
    verdict = {}
    for (jname,), (tau, case) in sorted(worst.items()):
        p1, p2 = 100 * tau / STALL["ST3215"], 100 * tau / STALL["STS3250"]
        verdict[jname] = dict(tau_nm=round(tau, 3), case=case,
                              st3215_pct=round(p1, 1), st3250_pct=round(p2, 1),
                              st3215=flag(p1), st3250=flag(p2))
        print(f"  {jname:>5s}: {tau:5.2f} N*m ({case})  "
              f"ST3215 {p1:.0f}% {flag(p1)}  STS3250 {p2:.0f}% {flag(p2)}")
    out = os.path.join(HERE, "out")
    os.makedirs(out, exist_ok=True)
    path = os.path.join(out, "torque_audit.json")
    with open(path, "w") as fh:
        json.dump(dict(masses_g={k: round(v * 1000, 1) for k, v in m.items()},
                       stall_nm=STALL, cases=cases, worst=verdict,
                       method="static J^T F on CAD-derived masses (D039); "
                              "holding torques, dynamics ride on top"),
                  fh, indent=1)
    print(f"\nwrote {path}")


if __name__ == "__main__":
    main()
