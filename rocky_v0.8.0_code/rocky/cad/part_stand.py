"""Bench/maintenance stand — legs swing free, joints are printed I6.

The missing bench-day enabler: pose_check, calibrate_centers and the
torque-step all want the robot held with UNLOADED legs. Three parts, one
joint: every stack interface is the same spigot joint — the LOWER part's
top 16 mm carries five vertical I6 male bars on its pentagon facets, the
UPPER part's skirt drops over them with female dovetail channels (FIT
clearance). Gravity closes the joint; the trapezoids carry shear/moment.

  stand_base    — floor plate + pentagon tube stub, males on top
  stand_section — pentagon tube H=80: skirt below, males on top.
                  Print 0/1/2: deck-bottom heights 47 / 127 / 207 mm
                  (full leg reach below deck is 172 — two sections free
                  the whole envelope)
  stand_crown   — skirt below + top plate + THREE arms at the free webs
                  (az 54, 126, 270) ending in drop-in saddles that catch
                  the deck edge (apothem 80.9). Three, not five: the
                  belly bay blocks the az-342/198 webs, and STATIONS are
                  out entirely — the coxa-plate cantilever overhangs the
                  deck edge right where a saddle wall would rise.

Checks: aligned stack must slide (≈0 interference), a 36°-misaligned
stack must NOT (registration is real), saddle bite vs deck edge, arm
clearance angles vs bay corners.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import IF, dovetail_male

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
DV = IF["dovetail"]

APO = 40.0                       # pentagon tube apothem (facet radius)
WALL_T = 3.0
SEC_H = 80.0
ENG = 16.0                       # male bar length / joint engagement
FACETS = [18 + 72 * k for k in range(5)]
SKIRT_T = DV["depth"] + FIT + 2.6            # wall swallows crest + meat
ARM_AZ = [54, 126, 270]                      # free webs only


def _pent(r_apo, rot=54):
    return RegularPolygon(r_apo / np.cos(np.pi / 5), 5, rotation=rot)


def _tube(h, z0=0.0, apo=APO, wall=WALL_T):
    t = Pos(0, 0, z0) * extrude(_pent(apo), h)
    t -= Pos(0, 0, z0 - 1) * extrude(_pent(apo - wall), h + 2)
    return t


def _males(z_top):
    out = None
    for az in FACETS:
        m = Rot(0, 0, az) * Pos(APO - 1.2, 0, z_top - ENG / 2) * \
            Rot(0, 90, 0) * dovetail_male(length=ENG)
        out = m if out is None else out + m
    return out


def _male_neg(L):
    b = DV["base_w"] + 2 * FIT
    c = DV["crest_w"] + 2 * FIT
    d = DV["depth"] + FIT
    with BuildPart() as bp:
        with BuildSketch(Plane.YZ):
            with BuildLine():
                Polyline((-b / 2, -0.3), (b / 2, -0.3), (c / 2, d),
                         (-c / 2, d), (-b / 2, -0.3))
            make_face()
        extrude(amount=L / 2, both=True)
    return bp.part


def _skirt(z_seat=0.0):
    """Skirt from z_seat-18 to z_seat+3, hugging a lower tube (outer facet
    APO) with FIT, female channels swallowing its males."""
    sk = _tube(21.0, z0=z_seat - 18.0, apo=APO + FIT + SKIRT_T,
               wall=SKIRT_T)
    for az in FACETS:
        sk -= Rot(0, 0, az) * Pos(APO - 1.2, 0, z_seat - 9.0) * \
            Rot(0, 90, 0) * _male_neg(26.0)
    return sk


def stand_base():
    plate = extrude(_pent(52), 5) + Pos(0, 0, 0) * extrude(_pent(52), 5)
    plate = extrude(RegularPolygon(85, 5, rotation=54), 5)
    tube = _tube(30.0, z0=5.0)
    return plate + tube + _males(35.0)


def stand_section():
    # v0.1 (session 8): the skirt's inner facet is APO + FIT — it hugs the
    # LOWER tube, so it never touched THIS part's tube (0.3 mm gap all
    # round) and the section exported as two bodies (D036 class; the crown
    # was fine only because its top plate happens to bridge the gap). A
    # 3 mm seat ring z 0..3 now ties tube and skirt — it is also the hard
    # stop the lower part's male bars land against (their tops sit at z 0).
    seat = extrude(_pent(APO + FIT + SKIRT_T), 3.0) - \
        Pos(0, 0, -1.0) * extrude(_pent(APO - WALL_T), 5.0)
    return _tube(SEC_H, z0=0.0) + _skirt(0.0) + seat + _males(SEC_H)


def stand_crown():
    crown = _skirt(0.0)
    crown += Pos(0, 0, 3.0) * extrude(_pent(APO + FIT + SKIRT_T), 6)  # plate
    for az in ARM_AZ:
        arm = Pos((36 + 86) / 2, 0, 8.0) * Box(50, 14, 8)             # spar
        arm += Pos((70 + 86) / 2, 0, 10.0) * Box(16, 14, 4)           # floor
        arm += Pos(86.0, 0, 18.0) * Box(4, 14, 16)                    # wall
        crown += Rot(0, 0, az) * arm
    return crown


def _layout_checks():
    ok = True
    bay_h = np.rad2deg(np.arctan2(27.0, 87.5))       # bay corner az + margin
    for az in ARM_AZ:
        d0 = min(abs((az % 360) - 0), abs(az - 180), abs(az - 360))
        arm_h = np.rad2deg(np.arctan2(7.0, 70.0))
        clear = d0 - bay_h - arm_h
        ok &= clear > 2.0
        print(f"  arm az {az:3d}: {clear:5.1f} deg clear of belly bay "
              f"({'OK' if clear > 2 else 'CLASH'})")
        dmin = min(abs((az - (90 + 72 * k) + 180) % 360 - 180)
                   for k in range(5))
        ok &= dmin >= 35.9                            # web = 36 off stations
        print(f"    ...and {dmin:.0f} deg from the nearest coxa station")
    floor_in, wall_in = 70.0 - 8, 84.0
    bite = 80.9 - (70.0 - 8.0)
    print(f"  saddle: floor r {70 - 8:.0f}..{86:.0f}, wall at r 84..88; "
          f"deck edge 80.9 bears on {80.9 - 62:.0f} mm of floor, "
          f"{84 - 80.9:.1f} mm radial slack to the wall")
    return ok


if __name__ == "__main__":
    parts = dict(stand_base=stand_base(), stand_section=stand_section(),
                 stand_crown=stand_crown())
    for n, p in parts.items():
        export(p, n)
        bb = p.bounding_box()
        print(f"  {n}: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm,"
              f" {p.volume / 1000:.0f} cm^3")
    ok = _layout_checks()
    # stack joints: aligned must slide, 36-deg misaligned must NOT
    for upper_n, seat in (("stand_crown", 35.0), ("stand_section", 35.0)):
        posed = Pos(0, 0, seat) * parts[upper_n]
        v_ok = (posed & parts["stand_base"])
        v_ok = 0.0 if v_ok is None else v_ok.volume
        posed_bad = Pos(0, 0, seat) * Rot(0, 0, 36) * parts[upper_n]
        v_bad = (posed_bad & parts["stand_base"])
        v_bad = 0.0 if v_bad is None else v_bad.volume
        ok &= v_ok < 1.0 and v_bad > 50.0
        print(f"  {upper_n} on base: aligned {v_ok:.2f} mm^3 "
              f"({'SLIDES' if v_ok < 1 else 'BINDS'}), misaligned 36 deg "
              f"{v_bad:.0f} mm^3 ({'REGISTERS' if v_bad > 50 else 'no reg!'})")
    print(f"  deck-bottom heights: base+crown 47 mm | +1 section 127 | "
          f"+2 sections 207 (full leg reach below deck: 172)")
    assert ok
