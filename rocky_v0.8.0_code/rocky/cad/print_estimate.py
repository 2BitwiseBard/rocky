"""Per-plate filament + time estimator for the print weekend (session 6).

Reads exported STLs, computes true part volumes, and estimates printed
grams and wall-clock time per WEEKEND PLATE (P1–P6, matching
docs/PRINT_WEEKEND_s6.html). Estimates are honest approximations, not
slicer truth:

  printed grams ≈ solid_volume × 1.24 g/cm³ (PLA) × fill_factor
    fill_factor models walls+infill: thin-walled parts ≈ 0.85–1.0 (they're
    nearly all perimeter), chunky parts at 3 walls/25 % ≈ 0.45–0.6.
    Factors below are per-part judgment calls, labeled.
  time ≈ grams × 3.2 min/g at 0.2 mm (old-Prusa single-perimeter-speed
    ballpark; 0.3 mm layers ≈ ×0.7). Expect ±30 % either way.

Purpose: spool budgeting (does the weekend fit the PLA on hand?) and
realistic overnight scheduling — not gospel. Slice the real thing for
truth; if slicer numbers differ WILDLY from these, something's wrong
(wrong scale import, accidental supports-everywhere) — that's the real
value of a printed sanity number.
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


# (stl, qty, fill_factor, layer) — factors: thin/shelly parts high, chunky low
PLATES = {
    "P1 fit ladder": [("fit_ladder", 1, 0.55, 0.2)],
    "P2 interface coupons": [
        ("latch_housing", 2, 0.85, 0.2), ("latch_rotor", 2, 0.9, 0.2),
        ("thumb_knob_m3", 2, 0.7, 0.2),
        ("dovetail_male_coupon", 1, 0.7, 0.2), ("dovetail_shoe", 1, 0.6, 0.2),
        ("horn_coupler", 1, 0.7, 0.2), ("coupler_recess_demo", 1, 0.6, 0.2),
        ("port_coupon_deck", 1, 0.55, 0.2), ("port_coupon_plate", 1, 0.6, 0.2),
    ],
    "P3 hand + I2": [
        ("hand_hub", 1, 0.6, 0.2), ("hand_cam", 1, 0.75, 0.2),
        ("hand_finger", 3, 0.8, 0.2), ("tibia_sea_slider", 1, 0.8, 0.2),
        ("tool_hook", 1, 0.65, 0.2), ("tool_scoop", 1, 0.55, 0.2),
    ],
    "P4 one leg (+3 blanks)": [
        ("coxa_yaw_base", 1, 0.55, 0.2), ("coxa_fork", 1, 0.55, 0.2),
        ("femur_link", 1, 0.6, 0.2), ("tibia_knee_carrier", 1, 0.5, 0.2),
        ("tibia_sea_outer", 1, 0.7, 0.2), ("horn_coupler", 3, 0.7, 0.2),
        ("servo_blank", 3, 0.35, 0.2),  # 2 walls / 10 % — deliberately airy
    ],
    "P5 body deck": [("body_deck", 1, 0.5, 0.2)],
    "P6 stand base+crown": [
        ("stand_base", 1, 0.4, 0.3), ("stand_crown", 1, 0.4, 0.3),
    ],
    "bonus: 2 shell sectors": [("shell_sector", 2, 0.6, 0.25)],
}


def main():
    report, grand_g, grand_min = {}, 0.0, 0.0
    for plate, parts in PLATES.items():
        rows, tot_g, tot_min = [], 0.0, 0.0
        for name, qty, ff, layer in parts:
            p = os.path.join(OUT, f"{name}.stl")
            v = stl_volume_cm3(p)
            g = v * PLA * ff * qty
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
        print(f"{plate:26s} {tot_g:6.0f} g   ~{tot_min/60:4.1f} h")
    print(f"{'WEEKEND TOTAL':26s} {grand_g:6.0f} g   ~{grand_min/60:4.1f} h  "
          f"(one 1 kg spool {'COVERS it' if grand_g < 900 else 'is NOT enough'})")
    out = os.path.join(OUT, "print_estimate.json")
    json.dump(report, open(out, "w"), indent=1)
    print(f"written: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
