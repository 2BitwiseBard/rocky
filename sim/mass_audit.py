#!/usr/bin/env python3
"""Mass audit: CAD volumes -> link masses for the MuJoCo model (session 8).

The MJCF carried a hand-typed budget (M_TORSO 1.35, M_COXA 0.14, M_FEMUR
0.03, M_TIBIA 0.17 kg) since session 2. This derives every link from the
CURRENT cad/out STL volumes (x PLA density x the same fill factors
print_estimate.py uses) plus the non-printed hardware in params.yaml
`mass_hw` — and writes sim/mass_budget.json, which build_mjcf.py now reads.

Honesty: fill factors are +-30 % and mass_hw is all VERIFY. This is the
right SHAPE of the mass distribution, not the truth; bench day's kitchen
scale replaces every number here (NOTES_INBOX -> params -> regen).

  python3 mass_audit.py          # prints the table, writes mass_budget.json
"""
import json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "..", "cad")
sys.path.insert(0, CAD)
from common import params                                    # noqa: E402
from print_estimate import stl_volume_cm3, PLA, PLATES       # noqa: E402

P = params()
HW = P["mass_hw"]
# D052: servo masses come from the actuator identity (params `actuators:`);
# mass_hw.servo_* are YAML aliases of the same numbers, kept for old readers
SERVO_G = float(P["actuators"][P["leg"].get("servo", "st3215")]["mass_g"])
CLAW_SERVO_G = float(P["actuators"][P["leg"].get("claw_servo", "scs0009")]["mass_g"])
PETG = 1.27

FILL = {}
for plate in PLATES.values():
    for name, qty, ff, layer in plate:
        FILL[name] = ff
FILL.setdefault("shell_cap", 0.6)
FILL.setdefault("avionics_tray", 0.5)
FILL.setdefault("battery_sled", 0.5)
FILL.setdefault("belly_door", 0.5)
FILL.setdefault("busboard_bracket", 0.6)


def printed_g(name, material=PLA):
    v = stl_volume_cm3(os.path.join(CAD, "out", f"{name}.stl"))
    return v * material * FILL.get(name, 0.6)


def main():
    rows = {}
    # ---- torso: deck + carapace (PETG keepers) + battery + electronics + sled/tray/door
    torso = {
        "body_deck": printed_g("body_deck"),
        "shell_sector x5": 5 * printed_g("shell_sector", PETG),
        "shell_cap": printed_g("shell_cap", PETG),
        "battery_sled": printed_g("battery_sled"),
        "avionics_tray": printed_g("avionics_tray"),
        "belly_door": printed_g("belly_door"),
        "busboard_bracket": printed_g("busboard_bracket"),
        "battery": HW["battery_3s_5200"],
        "electronics_pod": HW["electronics_pod"],
        "yaw servos x5": 5 * SERVO_G,
        "coxa_yaw_base x5": 5 * printed_g("coxa_yaw_base"),
    }
    # ---- per leg links (the moving mass the gait carries)
    coxa = {                     # rotates about yaw: fork + femur servo
        "coxa_fork": printed_g("coxa_fork"),
        "femur servo": SERVO_G,
        "horn_coupler": printed_g("horn_coupler"),
        "fasteners": HW["fasteners_per_leg"] / 3,
    }
    femur = {
        "femur_link": printed_g("femur_link"),
        "femur_plate_b": printed_g("femur_plate_b", PETG),            # D047 idler-side plate
        "horn_coupler": printed_g("horn_coupler"),
        "fasteners": HW["fasteners_per_leg"] / 3,
    }
    tibia = {                    # swings about the knee: knee servo + carrier + shin + hand
        "tibia_knee_carrier": printed_g("tibia_knee_carrier"),
        "knee servo": SERVO_G,
        "tibia_tube": HW["tibia_tube"],
        "tibia_sea_outer": printed_g("tibia_sea_outer"),
        "tibia_sea_slider": printed_g("tibia_sea_slider"),
        "sea_spring_switch": HW["sea_spring_switch"],
        "hand_hub": printed_g("hand_hub"),
        "hand_cam": printed_g("hand_cam"),
        "hand_finger x3": 3 * printed_g("hand_finger"),
        "claw servo": CLAW_SERVO_G,
        "foot_pad_tpu": HW["foot_pad_tpu"],
        "fasteners": HW["fasteners_per_leg"] / 3,
    }
    budget_old = {"torso": 1350.0, "coxa": 140.0, "femur": 30.0, "tibia": 170.0}
    out = {}
    for name, d in (("torso", torso), ("coxa", coxa), ("femur", femur), ("tibia", tibia)):
        tot = sum(d.values())
        out[name] = round(tot, 1)
        print(f"\n{name.upper():6s}  {tot:7.1f} g   (old budget {budget_old[name]:.0f} g, "
              f"{100*(tot/budget_old[name]-1):+.0f} %)")
        for k, v in d.items():
            print(f"   {k:22s} {v:7.1f}")
    total = out["torso"] + 5 * (out["coxa"] + out["femur"] + out["tibia"])
    old_total = 1350 + 5 * (140 + 30 + 170)
    print(f"\nROBOT  {total:7.1f} g  (old budget {old_total:.0f} g, "
          f"{100*(total/old_total-1):+.0f} %)")
    # hand mass alone, for the sim's hand geoms
    hand = tibia["hand_hub"] + tibia["hand_cam"] + tibia["hand_finger x3"] + tibia["claw servo"]
    out["hand"] = round(hand, 1)
    out["_note"] = ("derived from cad/out STL volumes x PLA/PETG x print_estimate fill "
                    "factors + params.mass_hw (ALL VERIFY); grams")
    with open(os.path.join(HERE, "mass_budget.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("written: sim/mass_budget.json")


if __name__ == "__main__":
    main()
