"""Sacrificial horn coupler — the $0.30 part that dies so the $17 servo lives.

Master plan §3.4 design rule: every servo horn interfaces through a printed
replaceable coupler. Load path: horn --(4x M2)--> coupler --(castellation
shear lobes)--> driven part --(2x M3 clamp)--> retained. In a crash the
lobes shear; you print a new 1-gram coupler instead of buying a servo.

D046 (laptop session 3, 2026-09-17): the coupler is now ADOPTED by the
femur link (both hubs) — v0.8.0 had the female recess only in the demo
coupon, and the link bolted straight to the horn with nothing locating it.
Two fixes to the coupler itself found while adopting it:
  * the lobes sat at 0/90/180/270 — directly OVER the four M2 horn screws
    on the same angles, so an M2 head (Ø3.8) could not be dropped into its
    counterbore (0.8 mm under the lobe). Lobes now sit at 45/135/225/315,
    between the screws; the M2 driver columns are asserted clear below.
  * lobe half-angle 28 -> 22 deg so a Ø4.4 driver fits the 46 deg gaps.
    Shear area 161 mm² x 25 MPa ≈ 40 N·m at r 9.7 — still >> 2.94 N·m stall.
The yaw joint (fork hub) does NOT use a coupler: its clamp-screw heads would
sit under the crown cap and the yaw horn sees the smallest impact loads —
documented deviation, see part_coxa.

Parts:
  horn_coupler       : disc on the ST3215 horn, 4 shear lobes on top
  coupler_recess_demo: test coupon with the female pocket — print BOTH,
                       check the fit, file the result in NOTES_INBOX
"""
from build123d import *
from common import params, export
from servo_st3215 import horn_screw_positions

P = params()
PR = P["print"]
S = P["servo_st3215"]
FIT = PR["clearance_fit"]

DISC_D = 26.0
DISC_T = 3.0                          # horn top -> driven-part face
LOBE_H = 2.6
LOBE_R0, LOBE_R1 = 7.0, 12.4          # radial extent of shear lobes
LOBE_HALF_ANG = 22                    # deg: lobe angular half-width
LOBE_ANGS = (45, 135, 225, 315)       # between the M2 horn screws (0/90/180/270)
CLAMP_ANGS = (45, 225)                # M3 tap bores in two opposite lobes
RECESS_CLAMP_ANGS = (-45, -225)       # the same bores as the RECESS sees them: the
                                      # coupler is flipped (Rot 180 about X) onto the
                                      # driven face, which mirrors the angles
TAP_R_POS = (LOBE_R0 + LOBE_R1) / 2
M2_HEAD_D = 3.8
POCKET_DEPTH = LOBE_H + 0.4           # what coupler_recess removes below the face
CLAMP_BORE_DEPTH = DISC_T + LOBE_H - 0.8   # blind M3 bore: 4.8 mm from the lobe tip


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
    for ang in LOBE_ANGS:
        c += _lobe(ang)
    # M2 horn screws from the lobe side: clearance + shallow head counterbore
    for hx, hy in horn_screw_positions(P):
        c -= Pos(hx, hy, DISC_T / 2) * Cylinder(PR["screw_m2_clear"] / 2, DISC_T + 2)
        c -= Pos(hx, hy, DISC_T - 0.6) * Cylinder((M2_HEAD_D + 0.6) / 2, 1.4)
    # center pilot over the horn's centre screw head
    c -= Pos(0, 0, DISC_T / 2) * Cylinder(6.4 / 2, DISC_T + 2)
    # M3 thread-forming bores down two opposite lobes (clamps the driven
    # part): blind, from the lobe tip to 0.8 mm above the disc bottom =
    # 4.8 mm of thread; M3 x 8 through the link's 3 mm web bottoms 0.2 short
    for ang in CLAMP_ANGS:
        c -= Rot(0, 0, ang) * Pos(TAP_R_POS, 0, (0.8 + DISC_T + LOBE_H + 1) / 2) * \
            Cylinder(PR["screw_m3_tap"] / 2, DISC_T + LOBE_H + 1 - 0.8)
    return c


def _lobe_inflated(ang_deg):
    seg = Pos(0, 0, DISC_T + LOBE_H / 2 + 0.2) * Cylinder(LOBE_R1 + FIT, LOBE_H + 0.4) \
        - Pos(0, 0, DISC_T + LOBE_H / 2) * Cylinder(LOBE_R0 - FIT, LOBE_H + 3)
    a = LOBE_HALF_ANG + 1.5
    hs1 = Rot(0, 0, ang_deg + a) * Pos(0, -100, DISC_T + LOBE_H / 2) * \
        Box(200, 200, LOBE_H + 3)
    hs2 = Rot(0, 0, ang_deg - a) * Pos(0, 100, DISC_T + LOBE_H / 2) * \
        Box(200, 200, LOBE_H + 3)
    return seg & hs1 & hs2


def coupler_recess(solid, face_z, clamp_holes=True, clamp_len=8.0):
    """Cut the female castellation pocket into `solid` whose mating face is
    the horizontal plane z=face_z; the pocket descends POCKET_DEPTH below
    that face. The coupler presents its lobes downward into it. Clamp holes:
    M3 clearance, `clamp_len` deep from the face (through a plate)."""
    for ang in LOBE_ANGS:
        solid -= Pos(0, 0, face_z - (DISC_T + LOBE_H + 0.4)) * _lobe_inflated(ang)
    if clamp_holes:
        for ang in RECESS_CLAMP_ANGS:
            solid -= Rot(0, 0, ang) * Pos(TAP_R_POS, 0, face_z - clamp_len / 2 + 0.5) * \
                Cylinder(PR["screw_m3_clear"] / 2, clamp_len + 1)
    return solid


def coupler_recess_demo():
    """Print-fit coupon: 34x34x8 pad, mating face on top (z=8)."""
    pad = Pos(0, 0, 4) * Box(34, 34, 8)
    pad = coupler_recess(pad, face_z=8.0)
    return pad


def coupler_on_face(face_tf):
    """The coupler posed lobes-DOWN into a recess whose face is at `face_tf`
    (a transform whose local z=0 plane is the mating face, +z pointing
    OUT of the driven part, i.e. toward the horn). Disc bottom then sits at
    local z=DISC_T where the horn top is."""
    return face_tf * Pos(0, 0, DISC_T) * Rot(180, 0, 0) * horn_coupler()


if __name__ == "__main__":
    import math
    c = horn_coupler()
    d = coupler_recess_demo()
    export(c, "horn_coupler")
    export(d, "coupler_recess_demo")
    # fit: flip the coupler onto the coupon face (disc up, lobes down)
    posed = coupler_on_face(Pos(0, 0, 8.0))
    v = 0.0 if (posed & d) is None else (posed & d).volume
    print(f"coupler x recess-coupon intersection: {v:.2f} mm^3 "
          f"({'FITS' if v < 1 else 'INTERFERES'})")
    # D046: every M2 head must be droppable from the lobe side — driver
    # column Ø4.4 above the counterbore floor must be empty of lobe
    worst = 0.0
    for hx, hy in horn_screw_positions(P):
        col = Pos(hx, hy, DISC_T - 0.6 + 10) * Cylinder(2.2, 20)
        i = col & c
        worst = max(worst, 0.0 if i is None else i.volume)
    print(f"M2 driver access through the lobes: worst {worst:.2f} mm^3 "
          f"({'CLEAR' if worst < 1 else 'BLOCKED'})")
    # castellation must locate: nudge the posed coupler radially, expect contact
    nudged = Pos(1.0, 0, 0) * posed
    i = nudged & d
    vn = 0.0 if i is None else i.volume
    print(f"radial capture (1 mm nudge): {vn:.1f} mm^3 "
          f"({'LOCATES' if vn > 1 else 'LOOSE'})")
    area = math.pi * (LOBE_R1**2 - LOBE_R0**2) * (4 * 2 * LOBE_HALF_ANG / 360)
    t_shear = area * 1e-6 * 25e6 * TAP_R_POS * 1e-3
    print(f"lobe shear capacity ~{t_shear:.1f} N*m vs servo stall 2.94")
    # clamp bores coaxial through coupon + posed coupler (the mirrored angles)
    wc = 0.0
    for ang in RECESS_CLAMP_ANGS:
        # pin from 2 mm below the coupon (screw enters from its outer face,
        # z=0) up to 0.3 short of the blind bore's end inside the disc
        L = 2 + 8.0 + (CLAMP_BORE_DEPTH - (POCKET_DEPTH - 0.4)) - 0.3
        pin = Rot(0, 0, ang) * Pos(TAP_R_POS, 0, -2 + L / 2) * Cylinder(PR["screw_m3_tap"]/2 - 0.05, L)
        for part in (d, posed):
            i = pin & part
            wc = max(wc, 0.0 if i is None else i.volume)
    print(f"clamp-bore coaxiality (coupon + coupler): worst {wc:.2f} mm^3 "
          f"({'COAXIAL' if wc < 1 else 'MISALIGNED'})")
    ok = v < 1 and worst < 1 and vn > 1 and wc < 1
    print(f"part_coupler checks: {'ALL CLEAN' if ok else 'FAILED'}")
    if not ok:
        import sys; sys.exit(1)
