"""Design-backlog small wins — cheap parts that make bench life better.

  belly_skid       : sacrificial under-deck skid rail (pair) — rubble scrapes
                     this, not the harness. Bolts to the deck M3 grid.
  trim_cup (+lid)  : ballast cup for CoM trimming: holds a stack of M3
                     washers at any deck grid point. Measured mass, no glue.
  imu_grommet      : TPU top-hat isolator for the BNO085 on the avionics
                     tray (4x). Kills gait-frequency vibration at the IMU.
  calib_gauge_hip  : bench-jig tower whose cradle holds the KNEE SERVO CASE
                     bottom at leg z 45.65 => femur exactly horizontal
                     (hip = 0) during center calibration.
  calib_gauge_knee : V-block that plumbs the tibia tube vertical at x = 140
                     => knee exactly -90. Registers in the jig base groove.
  whisker_shoe     : I6 dovetail shoe carrying two piano-wire whiskers
                     (contact feelers, Phase-3 toy — the shoe costs nothing).

All read frozen dims from params (D020); gauges assume the bench jig
(part_bench_jig: deck-proxy top at 130 over the base top at 6).
"""
from build123d import *
from common import params, export
from iface import IF, dovetail_female_shoe

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]

JIG_BASE_TOP = 6.0
LEG_Z0_IN_JIG = 130.0 + 4.0 + JIG_BASE_TOP     # leg-frame z=0 in jig coords


def belly_skid():
    L = 120
    sk = Pos(0, 0, -4) * Box(L, 9, 8)
    sk = fillet(sk.edges().group_by(Axis.Z)[0], 3.5)      # rounded scraping face
    for sx in (-40, 0, 40):
        sk -= Pos(sx, 0, -2) * Cylinder(PR["screw_m3_clear"] / 2, 14)
        sk -= Pos(sx, 0, -6.5) * Cylinder(6.4 / 2, 3.4)   # head recess (upside)
    return sk


def trim_cup():
    cup = Pos(0, 0, 5) * Cylinder(15 / 2, 10)
    cup -= Pos(0, 0, 6.2) * Cylinder(10.6 / 2, 10)        # washer stack bore
    cup -= Pos(0, 0, 5) * Cylinder(PR["screw_m3_clear"] / 2, 12)
    return cup


def trim_cup_lid():
    lid = Pos(0, 0, 1.1) * Cylinder(15 / 2, 2.2)
    lid += Pos(0, 0, -1.5) * Cylinder((10.6 - 2 * FIT) / 2, 3.2)
    lid -= Pos(0, 0, 0) * Cylinder(PR["screw_m3_clear"] / 2, 10)
    return lid


def imu_grommet():
    g = Pos(0, 0, 1.0) * Cylinder(8 / 2, 2.0)             # top flange
    g += Pos(0, 0, -1.6) * Cylinder(4.55 / 2, 3.4)        # shaft in Ø4.8 hole
    g += Pos(0, 0, -3.6) * Cylinder(8 / 2, 1.2)           # bottom flange (stretch in)
    g -= Pos(0, 0, -1) * Cylinder(2.7 / 2, 10)            # M2.5 through
    return g


def calib_gauge_hip():
    """Tower: base pad + shaft + top cradle at jig z 179.65 (leg 45.65)."""
    cradle_z = LEG_Z0_IN_JIG + 45.65 - 0.3                # -0.3: case FIT gap
    h = cradle_z - JIG_BASE_TOP
    base = Pos(0, 0, JIG_BASE_TOP + 3) * Box(60, 44, 6)
    shaft = Pos(0, 0, JIG_BASE_TOP + h / 2) * Box(26, 20, h)
    guss = Pos(0, 14, JIG_BASE_TOP + 30) * Box(20, 8, 54)
    guss += Pos(0, -14, JIG_BASE_TOP + 30) * Box(20, 8, 54)
    top = Pos(0, 0, cradle_z - 2) * Box(34, 30, 4)        # cradle shelf
    for sy in (1, -1):                                    # side cheeks
        top += Pos(0, sy * 14.2, cradle_z + 2) * Box(34, 2.4, 8)
    t = base + shaft + guss + top
    return t


def calib_gauge_knee():
    """V-block: vertical V-groove captures the Ø10 tube; locating rib rides
    the jig-base groove at x=140."""
    H = 46
    b = Pos(0, 0, JIG_BASE_TOP + H / 2) * Box(24, 30, H)
    # vertical V-groove on the -x face
    with BuildPart() as v:
        with BuildSketch(Plane.XY.offset(JIG_BASE_TOP)):
            with BuildLine():
                Polyline((-12 - 1, -8), (-12 + 7.5, 0), (-12 - 1, 8), (-12 - 1, -8))
            make_face()
        extrude(amount=H)
    b -= v.part
    base = Pos(0, 0, JIG_BASE_TOP + 2) * Box(36, 44, 4)
    rib = Pos(0, 0, JIG_BASE_TOP - 0.5) * Box(3.6, 28, 1.0)   # into jig groove
    return b + base + rib


def whisker_shoe():
    shoe = dovetail_female_shoe(body_h=12.0)
    for sy in (-4, 4):
        shoe -= Pos(0, sy, 12.6) * Rot(0, 70 * (1 if sy > 0 else -1), 0) * \
            Cylinder(1.35 / 2, 30)
    return shoe


if __name__ == "__main__":
    parts = dict(belly_skid=belly_skid(), trim_cup=trim_cup(),
                 trim_cup_lid=trim_cup_lid(), imu_grommet=imu_grommet(),
                 calib_gauge_hip=calib_gauge_hip(),
                 calib_gauge_knee=calib_gauge_knee(),
                 whisker_shoe=whisker_shoe())
    for n, p in parts.items():
        export(p, n)
        bb = p.bounding_box()
        big = max(bb.size.X, bb.size.Y, bb.size.Z)
        print(f"  {n}: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f}"
              f"{'  (print lying down)' if big > 200 else ''}")
    # gauge height sanity
    hip = parts["calib_gauge_hip"].bounding_box()
    print(f"hip gauge cradle top at jig z {hip.max.Z:.1f} "
          f"(target {LEG_Z0_IN_JIG + 45.65 - 0.3 + 6:.1f} incl. cheeks)")
