"""ST3215 servo dummy solid — the reference model every pocket subtracts from.

Frame convention: servo sits with its base on Z=0, body centered on X/Y of its
case footprint, OUTPUT SHAFT AXIS = +Z, located at the origin (0,0).
i.e., we place the *shaft axis* at (0,0) and the case hangs off -X from it.
This makes joint math trivial: every joint's rotation axis is the local Z at origin.
"""
from build123d import *
from common import params

def servo_body(p=None, clearance=0.0):
    """Return the servo as a solid. clearance>0 inflates it for pocket subtraction."""
    if p is None:
        p = params()
    s = p["servo_st3215"]
    c = clearance
    L, W, H = s["body_l"] + 2*c, s["body_w"] + 2*c, s["body_h"] + c
    # case: shaft axis at x=0; case front end is at +shaft_offset from axis
    front = s["shaft_offset"]
    case = Pos(front - L/2 + c, 0, H/2) * Box(L, W, H)
    boss = Pos(0, 0, s["body_h"] + s["boss_h"]/2) * Cylinder((s["boss_d"] + 2*c)/2, s["boss_h"])
    horn = Pos(0, 0, s["body_h"] + s["boss_h"] + s["horn_h"]/2) * \
           Cylinder((s["horn_d"] + 2*c)/2, s["horn_h"])
    return case + boss + horn

def horn_screw_positions(p=None):
    """XY positions of the 4x M2 horn screws (on the horn top plane)."""
    if p is None:
        p = params()
    s = p["servo_st3215"]
    r = s["horn_bcd"] / 2
    return [(r, 0), (-r, 0), (0, r), (0, -r)]

if __name__ == "__main__":
    from common import export
    export(servo_body(), "servo_st3215_dummy")
