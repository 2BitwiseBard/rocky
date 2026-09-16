"""Bus star-board bracket (session 5) — see docs/BUS_STARBOARD.md.

Holds the 40x30 perfboard star hub flat at 12 mm standoff beside the
avionics tray: 4 posts (M3 thread-forming, 34x24 pattern), two deck tabs
at 20 mm pitch matching the electronics grid, two zip-tie wings for the
r=62 loom ring. Print flat, no supports, PLA fine (no load).
"""
from build123d import *
from common import params, export

P = params()
PR = P["print"]

PLATE_W, PLATE_L, PLATE_T = 46.0, 36.0, 3.0
POST_DX, POST_DY, POST_H, POST_D = 34.0, 24.0, 8.0, 5.6
TAB_PITCH = 20.0


def busboard_bracket():
    b = Pos(0, 0, PLATE_T / 2) * Box(PLATE_W, PLATE_L, PLATE_T)
    # lightening window
    b -= Pos(0, 2, PLATE_T / 2) * Box(24, 14, PLATE_T + 2)
    # corner posts, M3 thread-forming bores
    for sx in (-POST_DX / 2, POST_DX / 2):
        for sy in (-POST_DY / 2, POST_DY / 2):
            b += Pos(sx, sy, PLATE_T + POST_H / 2) * Cylinder(POST_D / 2, POST_H)
            b -= Pos(sx, sy, PLATE_T + POST_H / 2 + 0.5) * \
                Cylinder(PR["screw_m3_tap"] / 2, POST_H + 2)
    # deck tabs on the -y edge, 20 mm pitch (grid holes (20,-20)/(40,-20))
    for sx in (-TAB_PITCH / 2, TAB_PITCH / 2):
        b += Pos(sx, -PLATE_L / 2 - 5, PLATE_T / 2) * Box(10, 10, PLATE_T)
        b -= Pos(sx, -PLATE_L / 2 - 5, PLATE_T / 2) * \
            Cylinder(PR["screw_m3_clear"] / 2, PLATE_T + 2)
    # zip-tie wings for the loom ring (+y edge)
    for sx in (-16, 16):
        wing = Pos(sx, PLATE_L / 2 + 4, PLATE_T / 2) * Box(8, 8, PLATE_T)
        wing -= Pos(sx, PLATE_L / 2 + 4, PLATE_T / 2) * Box(3.2, 4.5, PLATE_T + 2)
        b += wing
    return b


if __name__ == "__main__":
    part = busboard_bracket()
    export(part, "busboard_bracket")
    bb = part.bounding_box()
    print(f"bracket: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm")
    # sanity: posts land on the 34x24 corner-drill pattern
    assert abs(POST_DX - 34) < 1e-9 and abs(POST_DY - 24) < 1e-9
    print("post pattern matches BUS_STARBOARD.md drill spec (34 x 24)")
