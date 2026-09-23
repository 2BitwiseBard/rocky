"""Automated interference check: pairwise intersection volumes of posed solids.
Anything above ~1 mm^3 between parts that shouldn't touch = a real clash.
D046: the cap, straps and couplers joined the assembly.
"""
from build123d import *
from leg_assembly import build

parts = build()
pairs = [
    ("coxa_yaw_base", "servo_yaw"),
    ("coxa_yaw_base", "coxa_fork"),
    ("coxa_yaw_base", "servo_femur"),
    ("coxa_crown_cap", "coxa_fork"),
    ("coxa_crown_cap", "coxa_yaw_base"),
    ("coxa_crown_cap", "servo_femur"),
    ("coxa_fork", "servo_femur"),
    ("coxa_fork", "servo_yaw"),
    ("coxa_fork_strap", "servo_femur"),
    ("coxa_fork", "femur_link"),
    ("coupler_hip", "servo_femur"),      # disc rests on the horn top: ~0
    ("coupler_hip", "femur_link"),       # lobes in the recess: ~0
    ("coupler_hip", "coxa_fork"),
    ("coupler_knee", "servo_tibia"),
    ("coupler_knee", "femur_link"),
    ("coupler_knee", "tibia_knee_carrier"),
    ("femur_link", "servo_femur"),
    ("femur_link", "servo_tibia"),
    ("femur_link", "tibia_knee_carrier"),
    ("tibia_knee_carrier", "servo_tibia"),
    ("tibia_knee_strap", "servo_tibia"),
]
print(f"{'pair':46s} intersection mm^3")
worst = 0.0
for a, b in pairs:
    inter = parts[a] & parts[b]
    v = inter.volume if inter is not None else 0.0
    worst = max(worst, v)
    flag = "  <-- CLASH" if v > 1.0 else ""
    print(f"{a+' x '+b:46s} {v:10.2f}{flag}")
print("\nYaw sweep vs fixed coxa_yaw_base + cap:")
rotating = None
for k in ("coxa_fork", "coxa_fork_strap", "servo_femur", "coupler_hip", "femur_link",
          "coupler_knee", "servo_tibia", "tibia_knee_carrier", "tibia_knee_strap"):
    rotating = parts[k] if rotating is None else rotating + parts[k]
fixed = parts["coxa_yaw_base"] + parts["coxa_crown_cap"]
for ang in (-40, -25, 25, 40):
    swept = Rot(0, 0, ang) * rotating
    inter = fixed & swept
    v = inter.volume if inter is not None else 0.0
    worst = max(worst, v)
    flag = "  <-- CLASH" if v > 1.0 else ""
    print(f"  yaw {ang:+d} deg: {v:10.2f} mm^3{flag}")
print("RESULT:", "CLEAN" if worst <= 1.0 else "CLASHES PRESENT")
import sys; sys.exit(0 if worst <= 1.0 else 1)
