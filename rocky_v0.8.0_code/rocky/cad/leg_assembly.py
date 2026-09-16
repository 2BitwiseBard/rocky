"""Leg skeleton assembly, v0.1 — everything posed in the coxa frame (yaw axis = Z).

Exports one combined STL for visualization plus the posed sub-solids.
Neutral pose: femur horizontal (+X), tibia vertical (down).
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body
from part_coxa import (coxa_yaw_base, coxa_fork, femur_servo_placed,
                       Z_FEMUR_AXIS, L1, CASE_BASE_Y, HORN_TOP_Y)
from part_femur import femur_link, T as FEM_T

P = params()
S = P["servo_st3215"]
L2 = P["leg"]["l2_femur"]
L3 = P["leg"]["l3_tibia"]

def build():
    parts = {}
    parts["coxa_yaw_base"] = coxa_yaw_base()
    parts["coxa_fork"] = coxa_fork()
    parts["servo_yaw"] = servo_body(P)
    parts["servo_femur"] = femur_servo_placed()
    # femur link plate rides just outboard of the horn top plane (y = HORN_TOP_Y = -16)
    # link local: plate spans y 0..T, hub A at origin -> place so plate is at y -22.3..-16.3
    parts["femur_link"] = Pos(L1, HORN_TOP_Y - 0.3 - FEM_T, Z_FEMUR_AXIS) * femur_link()
    # tibia servo (body lives in the shin; its horn drives hub B), same orientation
    knee_x = L1 + L2
    parts["servo_tibia"] = Pos(knee_x, CASE_BASE_Y, Z_FEMUR_AXIS) * Rot(90, 0, 0) * servo_body(P)
    # tibia stand-in: carbon tube from knee downward
    parts["tibia_tube"] = Pos(knee_x, 0, Z_FEMUR_AXIS - L3/2) * \
        Cylinder(P["leg"]["tibia_tube_od"]/2, L3)
    # foot ball
    parts["foot"] = Pos(knee_x, 0, Z_FEMUR_AXIS - L3) * Sphere(9)
    return parts

def build_dryfit():
    """Session 6: the PRINT-WEEKEND dry-fit — real printed parts only, with
    servo BLANKS (part_servo_blank) in all three cradles instead of servo
    dummies, and no tube/SEA (carbon tube isn't in hand until Batch 0).
    This is exactly what sits on Tyler's bench Sunday night."""
    from part_servo_blank import servo_blank
    from part_tibia import tibia_knee_carrier, KNEE_X, Z_AXIS
    b = servo_blank()
    parts = {
        "coxa_yaw_base": coxa_yaw_base(),
        "coxa_fork": coxa_fork(),
        "blank_yaw": b,
        "blank_femur": Pos(L1, CASE_BASE_Y, Z_FEMUR_AXIS) * Rot(90, 0, 0) * b,
        "blank_knee": Pos(KNEE_X, CASE_BASE_Y, Z_AXIS) * Rot(90, 0, 0) * b,
        "femur_link": Pos(L1, HORN_TOP_Y - 0.3 - FEM_T, Z_FEMUR_AXIS) *
                      femur_link(),
        "tibia_knee_carrier": tibia_knee_carrier(),
    }
    return parts


if __name__ == "__main__":
    parts = build()
    combined = None
    for name, solid in parts.items():
        combined = solid if combined is None else combined + solid
    export(combined, "leg_skeleton_assembly", multi=True)

    # ---- session 6: posed dry-fit exports for the viewer's weekend mode ----
    df = build_dryfit()
    blanks = df["blank_yaw"] + df["blank_femur"] + df["blank_knee"]
    export(blanks, "dryfit_blanks_posed", multi=True)
    export(df["femur_link"], "femur_link_posed")
    # sanity: blanks must not intersect the printed structure they sit in
    struct = (df["coxa_yaw_base"] + df["coxa_fork"] +
              df["tibia_knee_carrier"] + df["femur_link"])
    inter = blanks & struct
    v = 0.0 if inter is None else inter.volume
    print(f"dry-fit: blanks x printed structure = {v:.2f} mm^3 "
          f"({'OK' if v < 1.0 else 'CLASH'})")
