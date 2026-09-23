"""Servo retention features shared by every ST3215 cradle (D046).

v0.1 held each servo with zip ties through wall slots; a 2 mm nudge of any
blank out of any cradle met zero material. Two features replace that,
independent of the servo's (still unmeasured) case screw holes:

  retention_lips : short lips in FRONT of the case's horn-side corners
                   (the -Y face, y0) attached to the end walls — block the
                   pull toward the link. The case still drops in from +Z.
  servo_strap    : printed bar across the two end walls, 2x M3 thread-
                   forming into the wall tops, with a downstand that presses
                   the case top — blocks +Z. Prints flat (bar face down).

Frames: the caller's leg-local frame; the servo case spans x [cx0, cx1],
y [cy0, cy1] (cy0 = horn-side face), z [cz0, cz1]; walls are wall_t thick
just outside cx0/cx1 (FIT gap), tops at z_wall_top.
"""
from build123d import *
from common import params

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
LIP_T = 2.7          # lip thickness (y), stands 0.0..-2.7 off the case face
LIP_W = 4.0          # lip reach along x from the wall inner face (clears a Ø14 blank horn zone)
STRAP_T = 3.0
STRAP_W = 10.0


def retention_lips(cx0, cx1, cy0, z_lo_lo, z_lo_hi, z_hi, wall_t):
    """Two lips (one per end wall). z_lo_lo/z_lo_hi = where each wall's
    solid starts (-X wall may start high: the fork's collar)."""
    lips = None
    for x_in, z_lo in ((cx0 - FIT, z_lo_lo), (cx1 + FIT, z_lo_hi)):
        # from the wall's outer face through the wall to LIP_W inside
        if x_in < (cx0 + cx1) / 2:
            x_a, x_b = x_in - wall_t, x_in + LIP_W
        else:
            x_a, x_b = x_in - LIP_W, x_in + wall_t
        lip = Pos((x_a + x_b) / 2, cy0 - LIP_T / 2, (z_lo + z_hi) / 2) * \
            Box(x_b - x_a, LIP_T, z_hi - z_lo)
        lips = lip if lips is None else lips + lip
    return lips


def strap_tap_bores(solid, cx0, cx1, cy_c, z_wall_top, wall_t):
    """Cut the two M3 thread-forming bores into the wall tops."""
    for wx in (cx0 - FIT - wall_t / 2, cx1 + FIT + wall_t / 2):
        solid -= Pos(wx, cy_c, z_wall_top - 4) * Cylinder(PR["screw_m3_tap"] / 2, 8.2)
    return solid


def servo_strap(cx0, cx1, cy_c, cz1, z_wall_top, wall_t):
    """Bar over both wall tops + downstand onto the case top (FIT above it)."""
    x_a = cx0 - FIT - wall_t
    x_b = cx1 + FIT + wall_t
    bar = Pos((x_a + x_b) / 2, cy_c, z_wall_top + STRAP_T / 2) * \
        Box(x_b - x_a, STRAP_W, STRAP_T)
    d_lo, d_hi = cz1 + FIT, z_wall_top
    down = Pos((cx0 + cx1) / 2, cy_c, (d_lo + d_hi) / 2) * \
        Box(cx1 - cx0 - 2 * FIT, STRAP_W, d_hi - d_lo)
    strap = bar + down
    for wx in (cx0 - FIT - wall_t / 2, cx1 + FIT + wall_t / 2):
        strap -= Pos(wx, cy_c, z_wall_top + STRAP_T / 2) * \
            Cylinder(PR["screw_m3_clear"] / 2, STRAP_T + 2)
    return strap
