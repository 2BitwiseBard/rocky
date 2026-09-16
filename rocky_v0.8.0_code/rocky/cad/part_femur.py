"""Femur link, v0.1 — twin-hub plate.

One hub bolts to the femur servo horn (at the coxa fork), the other bolts to the
tibia servo horn (the tibia servo body lives in the shin assembly). The link is a
flat plate printed on its side: layer lines run along the beam, perpendicular to
bending loads.

Local frame: hub A center at origin, hub B at (+L2, 0). Plate normal = +Y,
thickness 6, plate spans y in [0, 6].
"""
from build123d import *
from common import params, export
from servo_st3215 import horn_screw_positions

P = params()
S = P["servo_st3215"]
PR = P["print"]
L2 = P["leg"]["l2_femur"]
T = 6.0
HUB_D = 30.0
BEAM_H = 24.0

def femur_link():
    beam = Pos(L2/2, T/2, 0) * Rot(90, 0, 0) * Box(L2, BEAM_H, T)
    hubs = None
    for cx in (0.0, L2):
        h = Pos(cx, T/2, 0) * Rot(90, 0, 0) * Cylinder(HUB_D/2, T)
        hubs = h if hubs is None else hubs + h
    part = beam + hubs
    # hub holes: center pilot + 4x M2 on the horn BCD
    for cx in (0.0, L2):
        part -= Pos(cx, T/2, 0) * Rot(90, 0, 0) * Cylinder(6.4/2, T+2)
        for hx, hz in horn_screw_positions(P):
            part -= Pos(cx + hx, T/2, hz) * Rot(90, 0, 0) * \
                    Cylinder(PR["screw_m2_clear"]/2, T+2)
    # lightening slots
    for cx in (L2*0.32, L2*0.68):
        part -= Pos(cx, T/2, 0) * Rot(90, 0, 0) * Box(L2*0.2, BEAM_H*0.45, T+2)
    return part

if __name__ == "__main__":
    export(femur_link(), "femur_link")
