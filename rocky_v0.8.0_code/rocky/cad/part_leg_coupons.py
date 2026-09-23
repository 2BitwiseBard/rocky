"""D046 joint COUPONS — one-hour prints of the real joint geometry, both
mating parts clipped to the joint itself. Assemble on the bench BEFORE any
full leg part reprints. Because they are boolean clips of the production
solids, a coupon that fits proves the production part fits.

  J1 (yaw):  coupon_j1_hub  = fork hub + boss + floor stub (x -14..7)
             coupon_j1_post = base plate stub + crown post (x -45..-33)
             + coxa_crown_cap (full part) + 683ZZ + M3 x 10 axle + 2x M3 x 8
             + a servo_blank (or its horn zone: coupon_j2_horn) under the hub
  J2/J3:     coupon_j2_hub  = femur link hub A with recess + clamp holes
             coupon_j2_horn = servo blank horn zone with nut slots (z 29..38)
             + horn_coupler (full part) + 4x M2 x 8 + 4x M2 nuts + 2x M3 x 8
"""
from build123d import *
from common import export
from part_coxa import coxa_fork, coxa_yaw_base
from part_femur import femur_link
from part_servo_blank import servo_blank


def coupon_j1_hub():
    return coxa_fork() & Pos((-14 + 7) / 2, 0, 43) * Box(21, 30, 12)


def coupon_j1_post():
    return coxa_yaw_base() & Pos(-39, 0, 22) * Box(12, 24, 54)


def coupon_j2_hub():
    return femur_link() & Pos(0, 3, 0) * Box(32, 8, 32)


def coupon_j2_horn():
    return servo_blank() & Pos(0, 0, 33.5) * Box(22, 22, 9.2)


if __name__ == "__main__":
    for fn in (coupon_j1_hub, coupon_j1_post, coupon_j2_hub, coupon_j2_horn):
        export(fn(), fn.__name__)
    print("part_leg_coupons: 4 coupons exported (single-solid gate passed)")
