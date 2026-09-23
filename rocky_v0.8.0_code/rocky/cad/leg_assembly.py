"""Leg skeleton assembly, v0.2 (D046) — everything posed in the coxa frame
(yaw axis = Z). Neutral pose: femur horizontal (+X), tibia vertical (down).

Exports one combined STL for visualization plus the posed sub-solids.
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body
from part_coxa import (coxa_yaw_base, coxa_fork, coxa_crown_cap, coxa_fork_strap,
                       femur_servo_placed, Z_FEMUR_AXIS, L1, CASE_BASE_Y, HORN_TOP_Y)
from part_femur import femur_link, hub_face_tf, T as FEM_T
from part_coupler import coupler_on_face, DISC_T
from part_tibia import tibia_knee_carrier, tibia_knee_strap, KNEE_X, Z_AXIS

P = params()
S = P["servo_st3215"]
L2 = P["leg"]["l2_femur"]
L3 = P["leg"]["l3_tibia"]

LINK_TF = Pos(L1, HORN_TOP_Y - DISC_T - FEM_T, Z_FEMUR_AXIS)   # link-local -> leg


def femur_link_placed():
    """Plate y -25..-19; its +Y face sits on the coupler discs (horn top -3)."""
    return LINK_TF * femur_link()


def coupler_placed(which="A"):
    """Hip (A) / knee (B) coupler: lobes in the link hub, disc on the horn."""
    return LINK_TF * coupler_on_face(hub_face_tf(which))


def build():
    parts = {}
    parts["coxa_yaw_base"] = coxa_yaw_base()
    parts["coxa_fork"] = coxa_fork()
    parts["coxa_crown_cap"] = coxa_crown_cap()
    parts["coxa_fork_strap"] = coxa_fork_strap()
    parts["servo_yaw"] = servo_body(P)
    parts["servo_femur"] = femur_servo_placed()
    parts["coupler_hip"] = coupler_placed("A")
    parts["femur_link"] = femur_link_placed()
    parts["coupler_knee"] = coupler_placed("B")
    parts["servo_tibia"] = Pos(KNEE_X, CASE_BASE_Y, Z_AXIS) * Rot(90, 0, 0) * servo_body(P)
    parts["tibia_knee_carrier"] = tibia_knee_carrier()
    parts["tibia_knee_strap"] = tibia_knee_strap()
    parts["tibia_tube"] = Pos(KNEE_X, 0, Z_FEMUR_AXIS - L3/2) * \
        Cylinder(P["leg"]["tibia_tube_od"]/2, L3)
    parts["foot"] = Pos(KNEE_X, 0, Z_FEMUR_AXIS - L3) * Sphere(9)
    return parts


def build_dryfit():
    """The bench dry-fit: printed parts only, servo BLANKS in all three
    cradles, couplers on the blank horns, cap + straps on. No tube/SEA."""
    from part_servo_blank import servo_blank
    b = servo_blank()
    parts = {
        "coxa_yaw_base": coxa_yaw_base(),
        "coxa_fork": coxa_fork(),
        "coxa_crown_cap": coxa_crown_cap(),
        "coxa_fork_strap": coxa_fork_strap(),
        "blank_yaw": b,
        "blank_femur": Pos(L1, CASE_BASE_Y, Z_FEMUR_AXIS) * Rot(90, 0, 0) * b,
        "blank_knee": Pos(KNEE_X, CASE_BASE_Y, Z_AXIS) * Rot(90, 0, 0) * b,
        "coupler_hip": coupler_placed("A"),
        "coupler_knee": coupler_placed("B"),
        "femur_link": femur_link_placed(),
        "tibia_knee_carrier": tibia_knee_carrier(),
        "tibia_knee_strap": tibia_knee_strap(),
    }
    return parts


if __name__ == "__main__":
    parts = build()
    combined = None
    for name, solid in parts.items():
        combined = solid if combined is None else combined + solid
    export(combined, "leg_skeleton_assembly", multi=True)
    df = build_dryfit()
    blanks = df["blank_yaw"] + df["blank_femur"] + df["blank_knee"]
    export(blanks, "dryfit_blanks_posed", multi=True)
    export(df["femur_link"], "femur_link_posed")
    export(df["coupler_hip"] + df["coupler_knee"], "dryfit_couplers_posed", multi=True)
    export(df["coxa_crown_cap"] + df["coxa_fork_strap"] + df["tibia_knee_strap"],
           "dryfit_cap_straps_posed", multi=True)
    struct = None
    for k, s in df.items():
        if not k.startswith("blank_"):
            struct = s if struct is None else struct + s
    inter = blanks & struct
    v = 0.0 if inter is None else inter.volume
    print(f"dry-fit: blanks x printed structure = {v:.2f} mm^3 "
          f"({'OK' if v < 1.0 else 'CLASH'})")
