"""D047 joint COUPONS — one-hour prints of the real joint geometry, clipped
out of the production solids. Assemble on the bench BEFORE any full leg part
reprint; because they are boolean clips of the production parts, a coupon
that fits proves the production part fits.

  coupon_cup      : one servo cup (the same shape holds all three servos).
                    Slide a blank (or a real servo) in, drive the four rim
                    screws. Proves case fit, rim clearance, screw positions.
  coupon_yaw_hub  : the fork's lower hub — horn OD pocket + slotted holes.
                    Bolt it to a blank's horn (or the real horn: this is the
                    print that answers M2-or-M3 and the hole radius).
  coupon_hip_hub  : femur plate A hub A with the coupler recess + clamp holes
                    (+ horn_coupler, 2x M3 x 8).
  coupon_idler    : femur plate B hub A tower — the idler pocket with its
                    plug notch, on a blank_idler or the real idler.
"""
from build123d import *
from common import export
from part_coxa import coxa_fork
from part_femur import femur_link, femur_plate_b, YA0, YB0, YB1
from servo_mount import servo_cup


def coupon_cup():
    return servo_cup()


def coupon_yaw_hub():
    return coxa_fork() & Pos(0, 0, 5) * Box(30, 30, 10)


def coupon_hip_hub():
    return femur_link() & Pos(0, (YA0 + YA0 + 6) / 2, 0) * Box(34, 6.2, 34)


def coupon_idler():
    return femur_plate_b() & Pos(0, (YB0 + YB1) / 2, 0) * Box(30, YB1 - YB0 + 0.2, 40)


if __name__ == "__main__":
    for fn in (coupon_cup, coupon_yaw_hub, coupon_hip_hub, coupon_idler):
        export(fn(), fn.__name__)
    print("part_leg_coupons: 4 coupons exported (single-solid gate passed)")
