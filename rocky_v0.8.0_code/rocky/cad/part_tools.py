"""First real I2 tools — the tool-socket standard finally has tools.

The SEA slider stub (part_tibia, D020/I2) quarter-turn bayonets into the
hand hub; these parts carry the IDENTICAL female socket (same dims, same
cut recipe as hand_hub's boss — fit by construction), so any leg can swap
its hand for:

  tool_hook  — J-hook (drag a loop, pull a cable, hang the robot's own
               weight on a rail: loads land on the bore shoulder, lugs
               only retain — the I2 rule).
  tool_scoop — 42 x 36 spade with side walls, floor pitched 20 deg; for
               the regolith-cosplay sample scoop Rocky deserves.

Checks mirror part_tibia's: inserted & locked poses must be free, locked
+ pulled must interfere (retention proof).
"""
import math

import numpy as np
from build123d import *
from common import params, export

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
IF2 = P["interfaces"]["tool_socket"]
TUBE_OD = P["leg"]["tibia_tube_od"]


def i2_socket_boss(body_above=True):
    """The female I2 socket: boss z -14..0, bore shoulder at z +0.6 —
    byte-for-byte the hand-hub recipe (part_hand.py)."""
    boss = Pos(0, 0, -7) * Cylinder(16 / 2, 14)
    boss -= Pos(0, 0, -7) * Cylinder((TUBE_OD + FIT) / 2, 16)
    slot_w = IF2["lug_d"] + 2 * FIT
    circ_z = 0.6 - IF2["lug_z_from_face"]
    for base_ang in (0, 180):
        boss -= Rot(0, 0, base_ang) * \
            Pos(16 / 2 - 1.5, 0, -14 + IF2["entry_len"] / 2 - 0.5) * \
            Box(6, slot_w, IF2["entry_len"] + 1.0)
        for k in range(13):
            a = base_ang + IF2["twist_deg"] * k / 12
            w = slot_w - (IF2["detent_bump"]
                          if 60 < IF2["twist_deg"] * k / 12 < 78 else 0.0)
            boss -= Rot(0, 0, a) * Pos(16 / 2 - 1.5, 0, circ_z) * Box(6, w, w)
    if body_above:
        cap = Pos(0, 0, 1.8) * Cylinder(16 / 2, 3.6)     # bore-end shoulder
        cap -= Pos(0, 0, 0.1) * Cylinder((TUBE_OD + FIT) / 2, 1.1)
        boss += cap
    return boss


def tool_hook():
    t = i2_socket_boss()
    # shank: blade rising from the cap
    t += Pos(0, 0, 14) * Box(7, 10, 22)
    # J-hook: annulus in the XZ plane, mouth opening down-forward
    ring = Pos(9, 0, 30) * Rot(90, 0, 0) * Cylinder(15, 8)
    ring -= Pos(9, 0, 30) * Rot(90, 0, 0) * Cylinder(9, 10)
    ring -= Pos(9 + 8, 0, 30 - 16) * Box(24, 12, 24)      # open the mouth
    t += ring
    # rounded tip bead (snag-free)
    t += Pos(9 + 11.8, 0, 30 - 1.5) * Sphere(3.2)
    return t


def tool_scoop():
    t = i2_socket_boss()
    # spade: pitched floor + side walls + rear wall
    sp = Pos(0, 0, 0) * Box(42, 36, 2.6)
    for sy in (1, -1):
        sp += Pos(0, sy * (36 / 2 - 1.3), 4.5) * Box(42, 2.6, 9.4)
    sp += Pos(-42 / 2 + 1.3, 0, 4.5) * Box(2.6, 36, 9.4)
    # leading-edge chamfer (digs better)
    sp -= Pos(42 / 2, 0, -1.4) * Rot(0, -18, 0) * Box(8, 40, 3)
    spade_pose = Pos(14, 0, 18.6) * Rot(0, 20, 0)
    t += spade_pose * sp
    # neck (v0.2, session 8): v0.1's neck was Box z 9..17 — it floated 5.4 mm
    # above the socket cap (top z 3.6) AND 5 mm below the pitched floor; the
    # scoop shipped as THREE separate bodies (D036 class — the check suite
    # only ever exercised the bayonet). Now: a column rooted 0.6 mm into the
    # cap, trimmed by the floor's underside plane offset 1.0 mm INTO the
    # floor so the joint is a real fused overlap, not a tangent face.
    neck = Pos(0, 0, 16.5) * Box(12, 14, 27)                       # z 3..30
    neck -= spade_pose * Pos(0, 0, -0.3 + 50) * Box(200, 200, 100)  # above floor
    t += neck
    return t


if __name__ == "__main__":
    from part_tibia import tibia_sea_slider
    slider = tibia_sea_slider()
    for name, tool in (("tool_hook", tool_hook()),
                       ("tool_scoop", tool_scoop())):
        export(tool, name)
        ok = True
        for pose, rot, want_free in (("inserted", 0, True),
                                     ("locked", 90, True)):
            posed = Pos(0, 0, -23.4) * Rot(0, 0, rot) * slider
            inter = posed & tool
            v = 0.0 if inter is None else inter.volume
            good = (v < 1.0) == want_free
            ok &= good
            print(f"  {name} {pose}: {v:6.2f} mm^3 "
                  f"({'OK' if good else 'FAIL'})")
        pulled = Pos(0, 0, -23.4 - 3.0) * Rot(0, 0, 90) * slider
        inter = pulled & tool
        v = 0.0 if inter is None else inter.volume
        ok &= v > 3.0
        print(f"  {name} locked+pulled 3 mm: {v:6.2f} mm^3 "
              f"({'RETAINS' if v > 3 else 'FALLS OFF'})")
        bb = tool.bounding_box()
        print(f"  {name}: {bb.size.X:.0f} x {bb.size.Y:.0f} x "
              f"{bb.size.Z:.0f} mm, {tool.volume / 1000:.1f} cm^3")
        assert ok
