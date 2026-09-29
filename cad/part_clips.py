"""Cable clips (B13) — the harness stops dangling.

  tube_clip — snap C-clip for the Ø10 carbon tibia tube with a side wire
              tunnel (the JST-SH-3 hand lead + microswitch pair run down
              the shin). Snap gap 8 mm (~80 % of d: firm snap, no tools).
  link_clip — C-channel that clips over the top edge of femur plate A (6 mm)
              with the same tunnel on top (the servo daisy jumpers). D063
              (B94): on the D062 femur only link x 54..80 is free (bridge
              walls and couplers elsewhere), and there plate A is 2.5 mm
              from the swinging knee carrier, so the inner jaw is 1.2 thin
              (1.0 mm running clearance) and the nubs hook under the
              lightening slot's top edge (link x 60..68, `link_clip_posed`).

Print flat, PLA/PETG, one per tibia tube and one per femur, 5 each + spares
(the tube clip needs 8 mm of bare tube, i.e. a tube 8 mm longer than its
two sockets, 18 in the knee carrier + 11 in the SEA outer: the shortest
tube, 29 mm, has none, B95); they ride the post-caliper regen for final ID
tuning but the geometry ships now (B13 was DEFERRED-trivial — it stops
being deferred the day looms exist).

Checks (run this file): the link clip posed on the femur against the link,
plate B and both couplers, held in y, snapping home and hooked in z, and
clear of the knee carrier + knee servo (+ its plug keep-out) over knee
-150..-20 even pushed across its jaw play.
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


LINK_JAW = 8.2                # spine face -> jaw mouth: the nubs' top lands 6.9 under the edge
                              # (jaw - 1.3), FIT under the 6.6 mm of plate A above its slot


def link_clip(plate_t=6.0, jaw=LINK_JAW, inner=1.2):
    """C-channel over plate A's edge, the plate entering from +x. B94: on the
    D062 femur it rides the free span between the bridge walls and hub B
    (the only stretch of edge nothing else uses), where plate A's mating face
    is 2.5 mm from the swinging knee carrier: the inner (+y) jaw is `inner`
    thick (the 2.4 + 0.3 fit jaw grazed the carrier, 3-16 mm^3 across the
    knee sweep; 1.2 leaves 1.0 mm), the outer jaw stays 2.4. `jaw` 8.2 puts
    the nubs FIT under the 6.6 mm of plate above the lightening slot, so they
    hook its edge (14 left the clip loose by 6 mm over the slot; 8.0 left
    0.1, and a slot edge printed 0.1 low would have kept them out)."""
    y0, y1 = -(plate_t / 2 + FIT + 2.4), plate_t / 2 + FIT + inner
    yc = (y0 + y1) / 2
    c = Pos(0, yc, W / 2) * Box(jaw + 3.0, y1 - y0, W)
    c -= Pos(1.5 + 0.1, 0, W / 2) * Box(jaw + 0.2, plate_t + 2 * FIT, W + 2)
    # retention nubs at the jaw mouth
    for sy in (1, -1):
        c += Pos(jaw / 2 + 1.0, sy * (plate_t / 2 + FIT - 0.25), W / 2) * \
            Box(1.6, 0.5, W)
    # wire tunnel on the spine, centred on the clip body so it stays inside the inner jaw's face
    tun = Pos(-(jaw / 2 + 1.5) - 3.8, yc, W / 2) * Box(7.6, 9.6, W)
    tun -= Pos(-(jaw / 2 + 1.5) - 3.8, yc, W / 2) * Box(4.0, 6.0, W + 2)
    tun -= Pos(-(jaw / 2 + 1.5) - 5.6, yc, W / 2) * Box(4.0, 2.6, W + 2)
    return c + tun


LINK_CLIP_X = 60.0            # link x of the clip's near face: x 60..68 sits over plate A's
                              # lightening slot (x 55.1..74.1), between the bridge walls (to 52)
                              # and hub B (from 80); its nubs cannot leave the slot along x


def link_clip_posed(x0=LINK_CLIP_X, jaw=LINK_JAW):
    """The clip on plate A's top edge (z = BEAM_H / 2 in the link frame),
    jaws straddling the plate, in the leg frame (femur horizontal)."""
    from part_femur import LINK_TF, YA0, YA1, BEAM_H
    # clip-local x runs down into the plate (Rot 90 about y: x -> -z); the spine's
    # inner face (x = 1.5 - jaw / 2) lands on the edge
    return LINK_TF * (Pos(x0, (YA0 + YA1) / 2, BEAM_H / 2 - (jaw / 2 - 1.5)) *
                      Rot(0, 90, 0) * link_clip(jaw=jaw))


if __name__ == "__main__":
    tc, lc = tube_clip(), link_clip()
    export(tc, "tube_clip")
    export(lc, "link_clip")
    bad = []
    gap = 8.0
    if gap >= TUBE_OD:
        bad.append("tube_clip snap gap is not under the tube diameter")
    print(f"  tube_clip: snap gap {gap} vs tube {TUBE_OD} "
          f"({gap / TUBE_OD * 100:.0f}% — snaps, holds)")
    for n, p in (("tube_clip", tc), ("link_clip", lc)):
        bb = p.bounding_box()
        print(f"  {n}: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm,"
              f" {p.volume / 1000:.1f} cm^3")
    # B94: the link clip posed on the D062 femur, against everything fixed to the femur and
    # everything that swings past it (knee carrier, knee servo, its plugs over joints.pos_deg.knee)
    from leg_assembly import femur_link_placed, femur_plate_b_placed, coupler_placed
    from part_tibia import tibia_knee_carrier, knee_servo_placed, Z_AXIS
    from leg_frame import KNEE_X, KNEE_TF
    from servo_st3215 import plug_envelope
    v = lambda s: 0.0 if s is None else s.volume

    def check(name, val, good, ok="OK", fail="FAIL"):
        print(f"  {name}: {val:.2f} mm^3 ({ok if good(val) else fail})")
        if not good(val):
            bad.append(name)

    clip, link = link_clip_posed(), femur_link_placed()
    fixed = link + femur_plate_b_placed() + coupler_placed("A") + coupler_placed("B")
    check("link_clip x femur (link, plate B, both couplers)", v(clip & fixed), lambda q: q < 1,
          "FITS", "CLASH")
    check("link_clip nudged 1 mm +/-y x plate A (the jaws straddle it)",
          min(v((Pos(0, d, 0) * clip) & link) for d in (1.0, -1.0)), lambda q: q > 1, "HELD", "LOOSE")
    check(f"link_clip lifted {FIT - 0.02:.2f} mm x plate A (the nubs pass under the slot edge "
          "with FIT to spare)", v((Pos(0, 0, FIT - 0.02) * clip) & link), lambda q: q < 0.01,
          "SNAPS HOME", "NUBS SHORT")
    check("link_clip lifted 1 mm x plate A (the nubs hook the slot edge)",
          v((Pos(0, 0, 1.0) * clip) & link), lambda q: q > 1, "HOOKED", "LIFTS OFF")
    # the plug keep-out swings too: the jumpers this clip carries end in those plugs
    swing = tibia_knee_carrier() + knee_servo_placed() + KNEE_TF * plug_envelope(P)
    push = Pos(0, FIT + 0.4, 0) * clip        # pushed across its jaw play toward the carrier, + 0.4
    k0, k1 = P["joints"]["pos_deg"]["knee"]   # -150, -20
    worst = worst_push = 0.0
    for knee in range(k0, k1 + 1, 5):
        s = Pos(KNEE_X, 0, Z_AXIS) * Rot(0, -(knee + 90), 0) * Pos(-KNEE_X, 0, -Z_AXIS) * swing
        worst, worst_push = max(worst, v(clip & s)), max(worst_push, v(push & s))
    check(f"link_clip x knee carrier + servo + plug keep-out over knee {k0}..{k1}", worst,
          lambda q: q < 1, "CLEAR", "CLASH")
    check(f"  ... with the clip pushed {FIT + 0.4:.1f} mm toward the carrier", worst_push,
          lambda q: q < 1, "CLEAR", "GRAZES")
    print(f"part_clips checks: {'CLEAN' if not bad else 'FAIL — ' + '; '.join(bad)}")
    raise SystemExit(1 if bad else 0)
