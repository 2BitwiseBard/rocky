"""Femur link, v0.2 (D046) — twin-hub plate that mates the horn COUPLERS.

v0.1 bolted each hub straight onto a servo horn with 4x M2 and nothing
else: no locating feature, no clamp, and on the printed blanks 3 mm of PLA
thread per screw. A 2 mm nudge in any direction met zero material. v0.2:
each hub carries the female castellation recess (part_coupler) on its +Y
face plus 2x M3 clearance for the clamp screws into the coupler's lobes.
The lobes locate the link radially and in rotation; the clamps hold it on.
The link never touches a horn.

Local frame: hub A center at origin, hub B at (+L2, 0). Plate normal = +Y,
thickness T, plate spans y in [0, T]; the +Y face (y = T) is the mating
face — it sits on the coupler disc, DISC_T outboard of the horn top.
Printed flat (recess face UP), layer lines along the beam.
"""
from build123d import *
from common import params, export
from part_coupler import (coupler_recess, coupler_on_face, DISC_T, RECESS_CLAMP_ANGS,
                          TAP_R_POS, POCKET_DEPTH, CLAMP_BORE_DEPTH)

P = params()
S = P["servo_st3215"]
PR = P["print"]
L2 = P["leg"]["l2_femur"]
T = 6.0
HUB_D = 30.0
BEAM_H = 24.0


def _flat():
    """Plate in XY, thickness along +Z (z 0..T); mating face = z = T."""
    beam = Pos(L2/2, 0, T/2) * Box(L2, BEAM_H, T)
    hubs = None
    for cx in (0.0, L2):
        h = Pos(cx, 0, T/2) * Cylinder(HUB_D/2, T)
        hubs = h if hubs is None else hubs + h
    part = beam + hubs
    for cx in (L2*0.32, L2*0.68):                      # lightening slots
        part -= Pos(cx, 0, T/2) * Box(L2*0.2, BEAM_H*0.45, T+2)
    for cx in (0.0, L2):                               # coupler recess per hub
        part = Pos(cx, 0, 0) * coupler_recess(Pos(-cx, 0, 0) * part, face_z=T, clamp_len=T)
    return part


def femur_link():
    return Rot(-90, 0, 0) * _flat()          # flat z -> world +y


def hub_face_tf(which="A"):
    """Transform whose z=0 plane is the hub's mating (+Y) face, +z toward the
    horn — feed to part_coupler.coupler_on_face. Link-local coords."""
    cx = 0.0 if which == "A" else L2
    return Pos(cx, T, 0) * Rot(-90, 0, 0)


if __name__ == "__main__":
    import sys
    link = femur_link()
    export(link, "femur_link")
    fails = []
    v = lambda x: 0.0 if x is None else x.volume
    for which in ("A", "B"):
        c = coupler_on_face(hub_face_tf(which))
        fit = v(c & link)
        cap_x = v((Pos(1.0, 0, 0) * c) & link)
        cap_z = v((Pos(0, 0, 1.0) * c) & link)
        # clamp bores coaxial: a Ø2.8 pin through both must touch neither
        cx = 0.0 if which == "A" else L2
        worst = 0.0
        for ang in RECESS_CLAMP_ANGS:
            # pin: 2 mm outside the outer face to 0.3 short of the bore end
            L = 2 + T + (CLAMP_BORE_DEPTH - (POCKET_DEPTH - 0.4)) - 0.3
            pin = Pos(cx, 0, 0) * Rot(-90, 0, 0) * Rot(0, 0, ang) * \
                Pos(TAP_R_POS, 0, -2 + L / 2) * Cylinder(PR["screw_m3_tap"]/2 - 0.05, L)
            worst = max(worst, v(pin & link), v(pin & c))
        ok = fit < 1 and cap_x > 1 and cap_z > 1 and worst < 1
        print(f"hub {which}: coupler fit {fit:.2f} mm^3, capture x {cap_x:.1f} / z {cap_z:.1f} mm^3, "
              f"clamp-bore coaxiality worst {worst:.2f} mm^3 -> {'OK' if ok else 'FAIL'}")
        if not ok: fails.append(which)
    bb = link.bounding_box()
    print(f"femur_link: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, "
          f"hub web under the recess {T - (DISC_T + 0.0):.1f} mm nominal")
    print(f"part_femur checks: {'ALL CLEAN' if not fails else 'FAILED'}")
    if fails: sys.exit(1)
