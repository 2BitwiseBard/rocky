"""Per-batch filament + time estimator for the print plan.

Reads exported STLs, computes true part volumes, and estimates printed
grams and wall-clock time per BATCH of docs/PRINT_PLAN_2026-09-22.md
(batch 0 fit ladder, 1 coupons, 2 one leg, 3 the body, then the deferred
work). Estimates are honest approximations, not slicer truth:

  printed grams ≈ solid_volume × 1.24 g/cm³ (PLA) × fill_factor
    fill_factor models walls+infill: thin-walled parts ≈ 0.85–1.0 (they're
    nearly all perimeter), chunky parts at 3 walls/25 % ≈ 0.45–0.6.
    Factors are per-part judgment calls, kept in FILL (one per part).
  time ≈ grams × 3.2 min/g at 0.2 mm (a stock bed-slinger at conservative
    speeds; 0.3 mm layers ≈ ×0.7). Expect ±30 % either way.

FILL is also what sim/mass_audit.py weighs the printed parts with, so it is
a separate table from the batch list: re-planning the batches never moves a
sim mass. Changing a fill factor DOES (mass_budget.json -> pebble.xml).

Purpose: spool budgeting and realistic overnight scheduling — not gospel.
Slice the real thing for truth; if slicer numbers differ WILDLY from these,
something's wrong (wrong scale import, accidental supports-everywhere) —
that's the real value of a printed sanity number.
"""
import json
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
PLA = 1.24  # g/cm^3
MIN_PER_G_02 = 3.2   # 0.2 mm layers
MIN_PER_G_03 = 2.2   # 0.3 mm layers


def stl_volume_cm3(path):
    """Signed-tetrahedron volume of a (binary) STL, pure stdlib."""
    with open(path, "rb") as f:
        head = f.read(80)
        if head[:5] == b"solid" and b"facet" in open(path, "rb").read(400):
            # ASCII STL fallback (none of ours are, but be safe)
            import re
            txt = open(path).read()
            verts = re.findall(r"vertex\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)", txt)
            tris = [tuple(map(float, v)) for v in verts]
            tris = [tris[i:i + 3] for i in range(0, len(tris), 3)]
        else:
            n = struct.unpack("<I", f.read(4))[0]
            tris = []
            for _ in range(n):
                data = struct.unpack("<12fH", f.read(50))
                v = data[3:12]
                tris.append(((v[0], v[1], v[2]), (v[3], v[4], v[5]),
                             (v[6], v[7], v[8])))
    vol = 0.0
    for a, b, c in tris:
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1])
                - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    return abs(vol) / 1000.0


# per-part fill factor (thin/shelly parts high, chunky low). sim/mass_audit.py
# weighs the printed parts with THESE numbers — edit with that in mind.
FILL = {
    # calibration + interface coupons
    "fit_ladder": 0.55, "latch_housing": 0.85, "latch_rotor": 0.9,
    "thumb_knob_m3": 0.7, "dovetail_male_coupon": 0.7, "dovetail_shoe": 0.6,
    "coupler_recess_demo": 0.6, "port_coupon_deck": 0.55, "port_coupon_plate": 0.6,
    # hand + SEA + tools
    "hand_hub": 0.6, "hand_cam": 0.75, "hand_finger": 0.8,
    "tibia_sea_slider": 0.8, "tibia_sea_outer": 0.8,
    "tool_hook": 0.65, "tool_scoop": 0.55,
    # leg chain (D047)
    "coxa_yaw_base": 0.55, "coxa_fork": 0.55, "femur_link": 0.6,
    "femur_plate_b": 0.55, "tibia_knee_carrier": 0.55, "horn_coupler": 0.85,
    "servo_blank": 0.35, "blank_idler": 0.9,
    "coupon_cup": 0.7, "coupon_yaw_hub": 0.7, "coupon_hip_hub": 0.7, "coupon_idler": 0.7,
    # body
    "body_deck": 0.5, "avionics_tray": 0.5, "tray_rail": 0.6, "battery_sled": 0.5,
    "sled_rail": 0.6, "belly_door": 0.5, "busboard_bracket": 0.6,
    "shell_sector": 0.6, "shell_cap": 0.6,
    # bench
    "stand_base": 0.4, "stand_section": 0.4, "stand_crown": 0.4,
    "jig_base": 0.4, "jig_column": 0.4,
}

# batch -> [(stl, qty, layer mm)], in docs/PRINT_PLAN_2026-09-22.md order.
# foot_pad_tpu (TPU, optional) is left out: this estimator prices PLA.
_ONE_LEG = [("coxa_fork", 1), ("horn_coupler", 2), ("femur_link", 1), ("femur_plate_b", 1),
            ("tibia_knee_carrier", 1), ("tibia_sea_outer", 1), ("tibia_sea_slider", 1)]
PLATES = {
    "Batch 0 fit ladder": [("fit_ladder", 1, 0.2)],
    "Batch 1 coupons + blank": [
        ("servo_blank", 1, 0.2), ("blank_idler", 1, 0.2),
        ("coupon_cup", 1, 0.2), ("coupon_yaw_hub", 1, 0.2),
        ("coupon_hip_hub", 1, 0.2), ("coupon_idler", 1, 0.2),
        ("horn_coupler", 1, 0.2),
    ],
    "Batch 2 one leg": [("coxa_yaw_base", 1, 0.2)] + [(n, q, 0.2) for n, q in _ONE_LEG] + [
        ("servo_blank", 3, 0.2), ("blank_idler", 3, 0.2),
    ],
    "Batch 3 body": [
        ("body_deck", 1, 0.2), ("coxa_yaw_base", 4, 0.2),
        ("busboard_bracket", 1, 0.2), ("avionics_tray", 1, 0.2), ("tray_rail", 2, 0.2),
        ("battery_sled", 1, 0.2), ("sled_rail", 2, 0.2),
    ],
    "Deferred: four more legs": [(n, 4 * q, 0.2) for n, q in _ONE_LEG],
    "Deferred: bench jig": [("jig_base", 1, 0.3), ("jig_column", 1, 0.3)],
    "Deferred: stand": [
        ("stand_base", 1, 0.3), ("stand_section", 1, 0.3), ("stand_crown", 1, 0.3),
    ],
    "Deferred: carapace": [("shell_sector", 5, 0.25), ("shell_cap", 1, 0.25)],
}
NEAR_TERM = [k for k in PLATES if k.startswith("Batch")]


def main():
    report, grand_g, grand_min, near_g, near_min = {}, 0.0, 0.0, 0.0, 0.0
    for plate, parts in PLATES.items():
        rows, tot_g, tot_min = [], 0.0, 0.0
        for name, qty, layer in parts:
            p = os.path.join(OUT, f"{name}.stl")
            v = stl_volume_cm3(p)
            g = v * PLA * FILL[name] * qty
            mpg = MIN_PER_G_03 if layer >= 0.28 else MIN_PER_G_02
            mins = g * mpg
            rows.append({"part": name, "qty": qty, "solid_cm3": round(v, 1),
                         "est_g": round(g, 1), "est_min": round(mins)})
            tot_g += g
            tot_min += mins
        report[plate] = {"parts": rows, "total_g": round(tot_g),
                         "total_h": round(tot_min / 60, 1)}
        grand_g += tot_g
        grand_min += tot_min
        if plate in NEAR_TERM:
            near_g += tot_g
            near_min += tot_min
        print(f"{plate:28s} {tot_g:6.0f} g   ~{tot_min/60:4.1f} h")
    print(f"{'BATCHES 0-3':28s} {near_g:6.0f} g   ~{near_min/60:4.1f} h  "
          f"(one 1 kg spool {'COVERS it' if near_g < 900 else 'is NOT enough'})")
    print(f"{'WHOLE PLAN':28s} {grand_g:6.0f} g   ~{grand_min/60:4.1f} h")
    out = os.path.join(OUT, "print_estimate.json")
    json.dump(report, open(out, "w"), indent=1)
    print(f"written: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
