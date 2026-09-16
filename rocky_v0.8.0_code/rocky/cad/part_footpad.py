"""TPU foot pad — friction sock over the closed-hand cone tip.

The only missing filament (Batch 0's $25) finally gets a part. Slides over
the closed cone's last 12 mm; retention = TPU stretch (inner taper printed
0.25 mm undersized) + the tip sphere seating into the pad's bottom cup.
Three circumferential grip ribs; 3 mm crown under the tip for impact.

Print: TPU 95A, 0.2 layers, 2 walls, 25% gyroid, slow (25 mm/s), tip down
with a brim. Five needed + spares — they will be consumables, by design.
"""
import math
from build123d import *
from common import params, export
from part_hand import CONE_BASE_R, CONE_TIP_R, CONE_LEN, KNUCKLE_Z

P = params()

GRIP_LEN = 12.0            # how far up the cone the sock reaches
WALL = 1.8
CROWN = 3.0                # sole thickness under the tip sphere
STRETCH = 0.25             # radial undersize for grip
RIB_R = 0.7                # grip rib half-depth
TAPER = (CONE_BASE_R - CONE_TIP_R) / CONE_LEN    # radius growth per mm


def cone_r(up_from_tip):
    return CONE_TIP_R + TAPER * up_from_tip


def foot_pad():
    """Modeled tip-down: z=0 at the pad sole, +z up the cone axis."""
    h = GRIP_LEN + CROWN + CONE_TIP_R      # total height incl. tip sphere seat
    r_top_in = cone_r(GRIP_LEN) - STRETCH
    r_tip_in = CONE_TIP_R - STRETCH * 0.5
    # outer: cone shell following the inner taper + wall
    outer = Pos(0, 0, CROWN + (GRIP_LEN + CONE_TIP_R) / 2) * \
        Cone(r_tip_in + WALL + 1.2, r_top_in + WALL, GRIP_LEN + CONE_TIP_R)
    outer += Pos(0, 0, CROWN / 2 + 0.6) * Cylinder(r_tip_in + WALL + 1.4, CROWN + 1.2)
    outer = fillet(outer.edges().group_by(Axis.Z)[0], 1.6)
    # inner cavity: sphere seat + cone taper (subtract)
    sphere_c = CROWN + CONE_TIP_R
    cavity = Pos(0, 0, sphere_c) * Sphere(CONE_TIP_R + 0.05)
    cavity += Pos(0, 0, sphere_c + GRIP_LEN / 2) * \
        Cone(r_tip_in, r_top_in, GRIP_LEN)
    cavity += Pos(0, 0, sphere_c + GRIP_LEN + 4) * Cylinder(r_top_in + 0.01, 8)
    pad = outer - cavity
    # grip ribs: three tori proud of the outer surface
    for k in range(3):
        z = CROWN + 2.5 + k * 3.4
        r_here = r_tip_in + WALL + 1.2 + (z - CROWN) / (GRIP_LEN + CONE_TIP_R) * \
            ((r_top_in + WALL) - (r_tip_in + WALL + 1.2))
        pad += Pos(0, 0, z) * Torus(r_here + RIB_R * 0.4, RIB_R)
    return pad


if __name__ == "__main__":
    p = foot_pad()
    export(p, "foot_pad_tpu")
    bb = p.bounding_box()
    print(f"pad: Ø{bb.size.X:.1f} x {bb.size.Z:.1f} tall, "
          f"inner grip {GRIP_LEN} mm of cone, stretch {STRETCH} mm")
    # sanity: the pad must NOT reach the finger hinge zone (cone base)
    reach_z = KNUCKLE_Z + CONE_LEN - GRIP_LEN
    print(f"sock top sits {CONE_LEN - GRIP_LEN:.0f} mm below the knuckles "
          f"(hand-frame z {reach_z:.1f}) — clear of finger gaps: "
          f"{'OK' if CONE_LEN - GRIP_LEN > 25 else 'TOO LONG'}")
