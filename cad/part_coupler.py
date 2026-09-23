"""Sacrificial horn coupler, v2 (D047) — the $0.30 part that dies so the $24
servo lives, re-cut for the REAL horn.

Master plan §3.4 design rule: every servo horn interfaces through a printed
replaceable coupler. Load path: horn --(4 screws)--> coupler --(castellation
shear lobes)--> driven part --(2x M3 clamp)--> retained. In a crash the lobes
shear; you print a new 1-gram coupler instead of buying a servo.

v2 vs D046: the STEP shows the horn's four holes on a 45° pattern, so the
lobes go BACK to 0/90/180/270 (between the screws) — D046 had moved them to
45° against a wrong hole pattern. The STEP is ambiguous about the hole
RADIUS (14–15.6 BCD depending on the probe) and the thread (RobotShop lists
M3 x 6 in the kit), so the screw holes are radial SLOTS with a counterbore
that seats an M3 head or an M2 head + washer: the coupler is the part you
reprint in an hour once one screw has been tried on the real horn.

The coupler is used at the hip and knee. The yaw joint rides the horn OD in
a pocket and bolts through the fork's hub directly (part_coxa) — the clamp
screws of a coupler would have to be driven from under the base plate.

Parts:
  horn_coupler       : disc on the horn, 4 shear lobes on top
  coupler_recess_demo: coupon with the female pocket — print both, check fit
"""
from build123d import *
from common import params, export
from servo_st3215 import horn_screw_positions, horn_slot_cutter, spec

P = params()
PR = P["print"]
S = spec(P)
FIT = PR["clearance_fit"]

DISC_D = 26.0
DISC_T = 3.0                          # horn top -> driven-part face
LOBE_H = 2.6
LOBE_R0, LOBE_R1 = 7.0, 12.4          # radial extent of shear lobes
LOBE_HALF_ANG = 20                    # deg: leaves a 50 deg gap centred on each 45 deg screw
LOBE_ANGS = (0, 90, 180, 270)         # between the horn screws (45/135/225/315)
CLAMP_ANGS = (0, 180)                 # M3 tap bores down two opposite lobes
RECESS_CLAMP_ANGS = (0, 180)          # the coupler is flipped (Rot 180 about X) onto the driven
                                      # face, which mirrors angles: 0/180 are their own mirror
TAP_R_POS = (LOBE_R0 + LOBE_R1) / 2   # 9.7
HEAD_CB_DEPTH = 1.4                   # counterbore from the lobe side
POCKET_DEPTH = LOBE_H + 0.4           # what coupler_recess removes below the face
CLAMP_BORE_DEPTH = DISC_T + LOBE_H - 0.8   # blind M3 bore: 4.8 mm from the lobe tip


def _lobe(ang_deg, r0=LOBE_R0, r1=LOBE_R1, half=LOBE_HALF_ANG, z0=DISC_T, h=LOBE_H):
    seg = Pos(0, 0, z0 + h / 2) * Cylinder(r1, h) - Pos(0, 0, z0 + h / 2) * Cylinder(r0, h + 2)
    hs1 = Rot(0, 0, ang_deg + half) * Pos(0, -100, z0 + h / 2) * Box(200, 200, h + 2)
    hs2 = Rot(0, 0, ang_deg - half) * Pos(0, 100, z0 + h / 2) * Box(200, 200, h + 2)
    return seg & hs1 & hs2


def horn_coupler():
    c = Pos(0, 0, DISC_T / 2) * Cylinder(DISC_D / 2, DISC_T)
    for ang in LOBE_ANGS:
        c += _lobe(ang)
    # horn screws from the lobe side: clearance SLOT through + head counterbore slot
    c -= horn_slot_cutter(-1, DISC_T + LOBE_H + 1, p=P)
    c -= horn_slot_cutter(DISC_T - HEAD_CB_DEPTH, DISC_T + LOBE_H + 1,
                          extra=S["horn_screw_head_d"] - S["horn_screw_clear_d"], p=P)
    # centre pilot over the horn's centre screw head
    c -= Pos(0, 0, DISC_T / 2) * Cylinder((S["horn_center_head_d"] + 1.0) / 2, DISC_T + 2)
    # M3 thread-forming bores down two opposite lobes (clamps the driven part):
    # blind, lobe tip to 0.8 mm above the disc bottom = 4.8 mm of thread;
    # an M3 x 8 through the driven part's 3 mm web bottoms 0.2 short
    for ang in CLAMP_ANGS:
        c -= Rot(0, 0, ang) * Pos(TAP_R_POS, 0, (0.8 + DISC_T + LOBE_H + 1) / 2) * \
            Cylinder(PR["screw_m3_tap"] / 2, DISC_T + LOBE_H + 1 - 0.8)
    return c


def _lobe_inflated(ang_deg):
    return _lobe(ang_deg, r0=LOBE_R0 - FIT, r1=LOBE_R1 + FIT, half=LOBE_HALF_ANG + 1.5,
                 z0=DISC_T - 0.2, h=LOBE_H + 0.4 + 2) & \
        Pos(0, 0, DISC_T + (LOBE_H + 0.4 + 4) / 2 - 0.2) * Cylinder(LOBE_R1 + FIT + 1, LOBE_H + 0.4 + 4)


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
    return coupler_recess(pad, face_z=8.0)


def coupler_on_face(face_tf):
    """The coupler posed lobes-DOWN into a recess whose face is at `face_tf`
    (a transform whose local z=0 plane is the mating face, +z pointing OUT of
    the driven part toward the horn). Disc bottom lands at local z = DISC_T."""
    return face_tf * Pos(0, 0, DISC_T) * Rot(180, 0, 0) * horn_coupler()


if __name__ == "__main__":
    import math, sys
    v = lambda x: 0.0 if x is None else x.volume
    c = horn_coupler()
    d = coupler_recess_demo()
    export(c, "horn_coupler")
    export(d, "coupler_recess_demo")
    fails = []
    posed = coupler_on_face(Pos(0, 0, 8.0))
    fit = v(posed & d)
    print(f"coupler x recess-coupon intersection: {fit:.2f} mm^3 ({'FITS' if fit < 1 else 'INTERFERES'})")
    if fit >= 1: fails.append("fit")
    # every horn screw head must be droppable from the lobe side at BOTH ends
    # of its slot: a Ø4.4 driver column above the counterbore floor is empty
    worst = 0.0
    from servo_st3215 import horn_screw_angles
    for ang in horn_screw_angles(P):
        for r in (S["horn_bcd"] / 2, S["horn_bcd"] / 2 + S["horn_bcd_slot"]):
            col = Rot(0, 0, ang) * Pos(r, 0, DISC_T - HEAD_CB_DEPTH + 10) * Cylinder(2.2, 20)
            worst = max(worst, v(col & c))
    print(f"horn-screw driver access through the lobes (both slot ends): worst {worst:.2f} mm^3 "
          f"({'CLEAR' if worst < 1 else 'BLOCKED'})")
    if worst >= 1: fails.append("driver access")
    nudged = Pos(1.0, 0, 0) * posed
    vn = v(nudged & d)
    print(f"radial capture (1 mm nudge): {vn:.1f} mm^3 ({'LOCATES' if vn > 1 else 'LOOSE'})")
    if vn <= 1: fails.append("capture")
    area = math.pi * (LOBE_R1**2 - LOBE_R0**2) * (4 * 2 * LOBE_HALF_ANG / 360)
    t_shear = area * 1e-6 * 25e6 * TAP_R_POS * 1e-3
    print(f"lobe shear capacity ~{t_shear:.1f} N*m vs servo stall 2.94")
    wc = 0.0
    for ang in RECESS_CLAMP_ANGS:
        L = 2 + 8.0 + (CLAMP_BORE_DEPTH - (POCKET_DEPTH - 0.4)) - 0.3
        pin = Rot(0, 0, ang) * Pos(TAP_R_POS, 0, -2 + L / 2) * Cylinder(PR["screw_m3_tap"] / 2 - 0.05, L)
        for part in (d, posed):
            wc = max(wc, v(pin & part))
    print(f"clamp-bore coaxiality (coupon + coupler): worst {wc:.2f} mm^3 ({'COAXIAL' if wc < 1 else 'MISALIGNED'})")
    if wc >= 1: fails.append("clamp coaxiality")
    # the slot must not break into a lobe: lobe volume unchanged by the cutter
    print(f"part_coupler checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails: sys.exit(1)
