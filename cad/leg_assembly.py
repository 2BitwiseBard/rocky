"""Leg skeleton assembly, v0.3 (D047) — everything posed in the coxa frame
(yaw axis = Z). Neutral pose: femur horizontal (+X), tibia vertical (down).

Exports one combined STL for visualization plus the posed sub-solids.
"""
from build123d import *
from common import params, export
from servo_st3215 import servo_body
from leg_frame import YAW_TF, HIP_TF, KNEE_TF, L1, L2, L3, KNEE_X, Z_HIP
from part_coxa import coxa_yaw_base, coxa_fork
from part_femur import femur_link, femur_plate_b, hub_face_tf, LINK_TF
from part_coupler import coupler_on_face
from part_tibia import tibia_knee_carrier

P = params()


def femur_link_placed():
    return LINK_TF * femur_link()


def femur_plate_b_placed():
    return LINK_TF * femur_plate_b()


def coupler_placed(which="A"):
    """Hip (A) / knee (B) coupler: lobes in the link hub, disc on the horn."""
    return LINK_TF * coupler_on_face(hub_face_tf(which))


def build():
    parts = {}
    parts["coxa_yaw_base"] = coxa_yaw_base()
    parts["coxa_fork"] = coxa_fork()
    parts["servo_yaw"] = YAW_TF * servo_body(P)
    parts["servo_femur"] = HIP_TF * servo_body(P)
    parts["coupler_hip"] = coupler_placed("A")
    parts["femur_link"] = femur_link_placed()
    parts["femur_plate_b"] = femur_plate_b_placed()
    parts["coupler_knee"] = coupler_placed("B")
    parts["servo_tibia"] = KNEE_TF * servo_body(P)
    parts["tibia_knee_carrier"] = tibia_knee_carrier()
    from part_tibia import BOSS_Z0, SOCKET_DEPTH
    tube_top = BOSS_Z0 + SOCKET_DEPTH
    parts["tibia_tube"] = Pos(KNEE_X, 0, tube_top - (tube_top - (Z_HIP - L3)) / 2) * \
        Cylinder(P["leg"]["tibia_tube_od"] / 2, tube_top - (Z_HIP - L3))
    parts["foot"] = Pos(KNEE_X, 0, Z_HIP - L3) * Sphere(9)
    return parts


def build_dryfit():
    """The bench dry-fit: printed parts only, servo BLANKS (+ glued idlers) in
    all three cradles, couplers on the blank horns, plate B on. No tube/SEA."""
    from part_servo_blank import blank_with_idler
    b = blank_with_idler()
    return {
        "coxa_yaw_base": coxa_yaw_base(),
        "coxa_fork": coxa_fork(),
        "blank_yaw": YAW_TF * b,
        "blank_femur": HIP_TF * b,
        "blank_knee": KNEE_TF * b,
        "coupler_hip": coupler_placed("A"),
        "coupler_knee": coupler_placed("B"),
        "femur_link": femur_link_placed(),
        "femur_plate_b": femur_plate_b_placed(),
        "tibia_knee_carrier": tibia_knee_carrier(),
    }


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
    export(df["femur_plate_b"], "femur_plate_b_posed")
    export(df["coupler_hip"] + df["coupler_knee"], "dryfit_couplers_posed", multi=True)
    struct = None
    for k, s in df.items():
        if not k.startswith("blank_"):
            struct = s if struct is None else struct + s
    inter = blanks & struct
    v = 0.0 if inter is None else inter.volume
    print(f"dry-fit: blanks x printed structure = {v:.2f} mm^3 ({'OK' if v < 1.0 else 'CLASH'})")
