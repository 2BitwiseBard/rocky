"""Leg-chain frame constants and the three servo poses (D047).

Leg-local frame: yaw axis = +Z at the origin, +X outboard, deck plane z = 0.
All three servos are posed from the servo_st3215 local frame (output axis +Z,
case hanging -X, z = 0 at the case BOTTOM = idler/cable face):

  yaw  : shaft DOWN. Horn faces the deck (the fork's lower hub bolts to it),
         idler + cable plugs face UP. YAW_TF = Pos(0,0,Z_YAW_TOP) * Rot(180,0,0)
         maps local z -> Z_YAW_TOP - z and local y -> -y.
  hip  : shaft along Y, horn toward -Y (coupler + femur plate A side), idler
         and plugs toward +Y (plate B side). HIP_TF = Pos(L1, YB, Z_HIP) * Rot(90,0,0)
         maps local (x, y, z) -> (L1 + x, YB - z, Z_HIP + y).
  knee : same as hip at x = KNEE_X; the knee servo's CASE belongs to the tibia.

Why shaft-down at the yaw (D047): the plugs then come out on top where the
cables can run inboard over the base, and the horn-side hub stack (6 mm) is
the only thing under the servo, which keeps the hip axis at 61 mm. Shaft-up
would have put the plugs under the servo (a 17 mm cavity in the base) and the
hip axis at ~75 mm.
"""
from build123d import *
from common import params
from servo_st3215 import z_levels, spec

P = params()
S = spec(P)
Z = z_levels(P)
PR = P["print"]
FIT = PR["clearance_fit"]
L1 = P["leg"]["l1_coxa"]                 # 45
L2 = P["leg"]["l2_femur"]                # 95
L3 = P["leg"]["l3_tibia"]                # 135
KNEE_X = L1 + L2                         # 140
Z_HIP = P["leg"]["hip_axis_z"]           # 61 (params SSOT; gait/sim/urdf read the same key)
YB = 22.0                                # hip/knee servo idler-face plane (leg y)
HALF_W = S["body_w"] / 2                 # 12.4

# yaw stage vertical stack (leg z)
HUB_Z0 = 1.3                             # fork lower hub bottom (1.3 above the base plate top)
HUB_T = 6.0
HUB_Z1 = HUB_Z0 + HUB_T                  # 7.3 = yaw horn TOP (horn faces down)
Z_YAW_TOP = HUB_Z1 + Z["horn1"]          # 40.4 = leg z of the yaw servo's local z=0 (idler face)
ZC_YAW = Z_YAW_TOP - S["body_h"] / 2     # 26.0 case mid-plane

YAW_TF = Pos(0, 0, Z_YAW_TOP) * Rot(180, 0, 0)
HIP_TF = Pos(L1, YB, Z_HIP) * Rot(90, 0, 0)
KNEE_TF = Pos(KNEE_X, YB, Z_HIP) * Rot(90, 0, 0)


def yaw_z(local_z):
    """leg z of a yaw-servo local z."""
    return Z_YAW_TOP - local_z


def hip_y(local_z):
    """leg y of a hip/knee-servo local z (horn side is -Y)."""
    return YB - local_z


# derived planes everybody shares
Z_YAW_IDLER_FACE = yaw_z(Z["idler1"])    # 43.7 (idler underside, facing up)
Z_YAW_RIM_TOP = yaw_z(Z["rim_bot"])      # 41.9 (bottom rim, facing up)
Y_HORN_TOP = hip_y(Z["horn1"])           # -11.1 hip/knee horn top plane
Y_IDLER_FACE = hip_y(Z["idler1"])        # 25.3 hip/knee idler face plane
Z_CUP_FLOOR_TOP = Z_HIP - HALF_W - FIT   # 48.3: hip/knee case bottom (leg -Z face) sits on this
