"""Automated interference check: pairwise intersection volumes of posed solids.
Anything above ~1 mm^3 between parts that shouldn't touch = a real clash.
"""
from build123d import *
from leg_assembly import build

parts = build()
pairs = [
    ("coxa_yaw_base", "servo_yaw"),
    ("coxa_yaw_base", "coxa_fork"),
    ("coxa_yaw_base", "servo_femur"),
    ("coxa_fork", "servo_femur"),
    ("coxa_fork", "servo_yaw"),
    ("coxa_fork", "femur_link"),
    ("femur_link", "servo_femur"),   # expect ~0 (0.3 gap off the horn face)
]
print(f"{'pair':46s} intersection mm^3")
worst = 0.0
for a, b in pairs:
    inter = parts[a] & parts[b]
    v = inter.volume if inter is not None else 0.0
    worst = max(worst, v)
    flag = "  <-- CLASH" if v > 1.0 else ""
    print(f"{a+' x '+b:46s} {v:10.2f}{flag}")
# Sweep check: rotate everything that yaws (fork + femur servo + link) about Z
# and re-test against the fixed base across the +/-40 deg gait range.
print("\nYaw sweep vs fixed coxa_yaw_base:")
rotating = parts["coxa_fork"] + parts["servo_femur"] + parts["femur_link"]
for ang in (-40, -25, 25, 40):
    swept = Rot(0, 0, ang) * rotating
    inter = parts["coxa_yaw_base"] & swept
    v = inter.volume if inter is not None else 0.0
    worst = max(worst, v)
    flag = "  <-- CLASH" if v > 1.0 else ""
    print(f"  yaw {ang:+d} deg: {v:10.2f} mm^3{flag}")
print("RESULT:", "CLEAN" if worst <= 1.0 else "CLASHES PRESENT")
