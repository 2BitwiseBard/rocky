"""Sacrificial horn coupler — the $0.30 part that dies so the $17 servo lives.

Master plan §3.4 design rule: every servo horn interfaces through a printed
replaceable coupler. Load path: horn --(4x M2)--> coupler --(castellation
shear lobes)--> driven part --(2x M3 clamp)--> retained. In a crash the
lobes shear; you print a new 1-gram coupler instead of buying a servo.

Parts:
  horn_coupler       : disc on the ST3215 horn, 4 shear lobes on top
  coupler_recess_demo: test coupon with the female pocket — print BOTH,
                       check the fit, file the result in NOTES_INBOX
The fork hub / femur plate adopt the female recess in v0.3 (post-caliper —
their horn faces get the recess + 2x M3 into the lobes' tap bores).
"""
from build123d import *
from common import params, export
from servo_st3215 import horn_screw_positions

P = params()
PR = P["print"]
S = P["servo_st3215"]
FIT = PR["clearance_fit"]

DISC_D = 26.0
DISC_T = 3.0
LOBE_H = 2.6
LOBE_R0, LOBE_R1 = 7.0, 12.4          # radial extent of shear lobes
LOBE_HALF_ANG = 28                    # deg: lobe angular half-width
TAP_R_POS = (LOBE_R0 + LOBE_R1) / 2   # M3 tap bores in two opposite lobes


def _lobe(ang_deg):
    seg = Pos(0, 0, DISC_T + LOBE_H / 2) * Cylinder(LOBE_R1, LOBE_H) \
        - Pos(0, 0, DISC_T + LOBE_H / 2) * Cylinder(LOBE_R0, LOBE_H + 2)
    hs1 = Rot(0, 0, ang_deg + LOBE_HALF_ANG) * Pos(0, -100, DISC_T + LOBE_H / 2) * \
        Box(200, 200, LOBE_H + 2)
    hs2 = Rot(0, 0, ang_deg - LOBE_HALF_ANG) * Pos(0, 100, DISC_T + LOBE_H / 2) * \
        Box(200, 200, LOBE_H + 2)
    return seg & hs1 & hs2


def horn_coupler():
    c = Pos(0, 0, DISC_T / 2) * Cylinder(DISC_D / 2, DISC_T)
    # M2 horn screws, countersunk-ish shallow cbore
    for hx, hy in horn_screw_positions(P):
        c -= Pos(hx, hy, DISC_T / 2) * Cylinder(PR["screw_m2_clear"] / 2, DISC_T + 2)
        c -= Pos(hx, hy, DISC_T - 0.6) * Cylinder(4.4 / 2, 1.4)
    # center pilot over the horn boss
    c -= Pos(0, 0, DISC_T / 2) * Cylinder(6.4 / 2, DISC_T + 2)
    for ang in (0, 90, 180, 270):
        c += _lobe(ang)
    # M3 thread-forming bores down two opposite lobes (clamps the driven part)
    for ang in (0, 180):
        c -= Rot(0, 0, ang) * Pos(TAP_R_POS, 0, DISC_T + LOBE_H / 2) * \
            Cylinder(PR["screw_m3_tap"] / 2, LOBE_H + 2)
    return c


def coupler_recess(solid, face_z, clamp_holes=True):
    """Cut the female castellation pocket into `solid` whose mating face is
    the horizontal plane z=face_z; the pocket descends LOBE_H (+0.4 slop)
    below that face. The coupler presents its lobes downward into it."""
    for ang in (0, 90, 180, 270):
        solid -= Pos(0, 0, face_z - (DISC_T + LOBE_H + 0.4)) * _lobe_inflated(ang)
    if clamp_holes:
        for ang in (0, 180):
            solid -= Rot(0, 0, ang) * Pos(TAP_R_POS, 0, face_z - 4) * \
                Cylinder(PR["screw_m3_clear"] / 2, 8)
    return solid


def _lobe_inflated(ang_deg):
    seg = Pos(0, 0, DISC_T + LOBE_H / 2 + 0.2) * Cylinder(LOBE_R1 + FIT, LOBE_H + 0.4) \
        - Pos(0, 0, DISC_T + LOBE_H / 2) * Cylinder(LOBE_R0 - FIT, LOBE_H + 3)
    a = LOBE_HALF_ANG + 1.5
    hs1 = Rot(0, 0, ang_deg + a) * Pos(0, -100, DISC_T + LOBE_H / 2) * \
        Box(200, 200, LOBE_H + 3)
    hs2 = Rot(0, 0, ang_deg - a) * Pos(0, 100, DISC_T + LOBE_H / 2) * \
        Box(200, 200, LOBE_H + 3)
    return seg & hs1 & hs2


def coupler_recess_demo():
    """Print-fit coupon: 34x34x8 pad, mating face on top (z=8)."""
    pad = Pos(0, 0, 4) * Box(34, 34, 8)
    pad = coupler_recess(pad, face_z=8.0)
    return pad


if __name__ == "__main__":
    c = horn_coupler()
    d = coupler_recess_demo()
    export(c, "horn_coupler")
    export(d, "coupler_recess_demo")
    # verification: flip the coupler onto the coupon face (disc up, lobes
    # down into the pocket) and check clearance by boolean intersection
    posed = Pos(0, 0, 11.0) * Rot(180, 0, 0) * c   # lobe tips at z=5.4..8
    inter = posed & d
    v = 0.0 if inter is None else inter.volume
    print(f"coupler x recess-coupon intersection: {v:.2f} mm^3 "
          f"({'FITS' if v < 1 else 'INTERFERES'})")
    # shear-area sanity: 4 lobes x area; PLA shear ~25 MPa vs ST3215 2.94 Nm
    import math
    area = math.pi * (LOBE_R1**2 - LOBE_R0**2) * (4 * 2 * LOBE_HALF_ANG / 360)
    t_shear = area * 1e-6 * 25e6 * TAP_R_POS * 1e-3     # very rough
    print(f"lobe shear capacity ~{t_shear:.1f} N*m vs servo stall 2.94 "
          f"(couplers should survive stall; they shear on IMPACT loads — "
          f"bench will tune lobe height/width)")
