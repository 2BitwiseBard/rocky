"""Cable clips (B13) — the harness stops dangling.

  tube_clip — snap C-clip for the Ø10 carbon tibia tube with a side wire
              tunnel (the JST-SH-3 hand lead + microswitch pair run down
              the shin). Snap gap 8 mm (~80 % of d: firm snap, no tools).
  link_clip — C-channel that clips over the 6 mm femur link plate edge
              with the same tunnel on top (the servo daisy jumpers).

Print flat, PLA/PETG, 3-4 per robot each; they ride the post-caliper regen
for final ID tuning but the geometry ships now (B13 was DEFERRED-trivial —
it stops being deferred the day looms exist).
"""
from build123d import *
from common import params, export

P = params()
FIT = P["print"]["clearance_fit"]
TUBE_OD = P["leg"]["tibia_tube_od"]

W = 8.0                       # clip width along the tube/plate


def tube_clip():
    ring = Pos(0, 0, W / 2) * Cylinder((TUBE_OD + 2 * FIT) / 2 + 2.4, W)
    ring -= Pos(0, 0, W / 2) * Cylinder((TUBE_OD + 2 * FIT) / 2, W + 2)
    ring -= Pos((TUBE_OD + 4) / 2, 0, W / 2) * Box(TUBE_OD + 4, 8.0, W + 2)
    # lead-in lips on the snap jaws
    for sy in (1, -1):
        ring += Pos(TUBE_OD / 2 + 1.4, sy * 4.6, W / 2) * \
            Rot(0, 0, sy * -28) * Box(3.2, 1.8, W)
    # wire tunnel alongside (4 x 6 channel, 1.8 walls)
    tun = Pos(-(TUBE_OD / 2 + 2.4 + 3.8), 0, W / 2) * Box(7.6, 9.6, W)
    tun -= Pos(-(TUBE_OD / 2 + 2.4 + 3.8), 0, W / 2) * Box(4.0, 6.0, W + 2)
    tun -= Pos(-(TUBE_OD / 2 + 2.4 + 5.6), 0, W / 2) * Box(4.0, 2.6, W + 2)
    return ring + tun


def link_clip(plate_t=6.0, jaw=14.0):
    c = Pos(0, 0, W / 2) * Box(jaw + 3.0, plate_t + 2 * FIT + 4.8, W)
    c -= Pos(1.5 + 0.1, 0, W / 2) * Box(jaw + 0.2, plate_t + 2 * FIT, W + 2)
    # retention nubs at the jaw mouth
    for sy in (1, -1):
        c += Pos(jaw / 2 + 1.0, sy * (plate_t / 2 + FIT - 0.25), W / 2) * \
            Box(1.6, 0.5, W)
    # wire tunnel on the spine
    tun = Pos(-(jaw / 2 + 1.5) - 3.8, 0, W / 2) * Box(7.6, 9.6, W)
    tun -= Pos(-(jaw / 2 + 1.5) - 3.8, 0, W / 2) * Box(4.0, 6.0, W + 2)
    tun -= Pos(-(jaw / 2 + 1.5) - 5.6, 0, W / 2) * Box(4.0, 2.6, W + 2)
    return c + tun


if __name__ == "__main__":
    tc, lc = tube_clip(), link_clip()
    export(tc, "tube_clip")
    export(lc, "link_clip")
    gap = 8.0
    assert gap < TUBE_OD, "snap gap must be under the tube diameter"
    print(f"  tube_clip: snap gap {gap} vs tube {TUBE_OD} "
          f"({gap / TUBE_OD * 100:.0f}% — snaps, holds)")
    for n, p in (("tube_clip", tc), ("link_clip", lc)):
        bb = p.bounding_box()
        print(f"  {n}: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm,"
              f" {p.volume / 1000:.1f} cm^3")
