"""Three-finger transforming hand ("iris claw"), v0.1.

The signature Rocky feature: three fingers that are SEGMENTS OF A TAPERED CONE.
Closed, they seat together into a solid-looking cone = the walking foot (loads
pass through the closed shell, not the servo). Open, they spread ~65 deg into
the three-pronged hand from the movie still.

Hand frame: +Z points outward (down the shin toward the ground).
  z 0..HUB_H          hub cylinder (houses SCS0009, shaft on axis, pointing +Z)
  z HUB_H..+CAM       cam disc on the servo horn (3 drive slots - tune in v0.2)
  z KNUCKLE..TIP      three cone-segment fingers, hinged at 120 deg stations

v0.1 is geometry + poses: mechanism tuning (slot spirals, pin sizes, friction)
happens on the bench in Phase 1 - that's why it's a standalone print.
"""
from build123d import *
from common import params, export

P = params()
S9 = P["servo_scs0009"]
PR = P["print"]

HUB_D = 40.0
HUB_H = 18.0
CAM_D = 38.0                               # v0.2: sized for the real spiral (r 9->16.4)
CAM_T = 3.0
CAM_Z0 = HUB_H + 0.6
KNUCKLE_Z = CAM_Z0 + CAM_T + 5.4          # v0.2: raised 4.2 so lugs clear the cam disc
                                           # (v0.1 had cam/lug clash - caught by check below)
CONE_BASE_R = 17.0
CONE_TIP_R = 3.5
CONE_LEN = 52.0
WALL = 2.4
HINGE_R = 14.5                             # hinge pin circle radius
# v0.2.1: design open = 55 deg (was 65). The tilted drive pin's slot
# footprint grows ~1/cos(theta): at 65 deg the needed slot width breaches
# the cam rim AND the skirt clips the neighboring hinge lugs. 55 deg was
# already the commanded software cap (BUILD_LOG 07-28) — now the geometry
# and the software agree. Revisit at v0.3 with measured servos.
OPEN_DEG = 55.0

def scs0009_dummy(clearance=0.0):
    c = clearance
    L, W, H = S9["body_l"]+2*c, S9["body_w"]+2*c, S9["body_h"]+c
    body = Pos(0, 0, HUB_H - H/2 + 1) * Box(L, W, H)          # shaft end near hub top
    shaft = Pos(0, 0, HUB_H + 1.5) * Cylinder((4.6+2*c)/2, 5)
    return body + shaft

def hand_hub():
    hub = Pos(0, 0, HUB_H/2) * Cylinder(HUB_D/2, HUB_H)
    # servo pocket (drop-in from the back/-Z, shaft toward +Z)
    hub -= scs0009_dummy(clearance=PR["clearance_fit"])
    # back opening to insert servo + pass the tibia tube interface
    hub -= Pos(0, 0, 4) * Box(S9["body_l"]+2*PR["clearance_fit"], S9["body_w"]+2*PR["clearance_fit"], 8.2)
    # v0.2: pin-sweep relief pockets in the hub top face — the drive pin dips
    # below the cam plane into the hub as the finger opens (found by check)
    for i in range(3):
        hub -= Rot(0, 0, 120*i) * Pos(13.25, 0, 15.0) * Box(14.5, 5.6, 7.4)
    # tibia tube socket boss on the back (z<0). v0.2.1: tool socket I2 (D020)
    # — the SEA stub quarter-turn bayonets in instead of gluing: L-slots cut
    # THROUGH the boss wall (entry 6 axial, 90 deg twist, 0.6 detent; the
    # visible lug doubles as an engagement indicator). Compression still
    # lands on the bore-end shoulder, not the lugs.
    IF2 = P["interfaces"]["tool_socket"]
    boss = Pos(0, 0, -7) * Cylinder(16/2, 14)
    boss -= Pos(0, 0, -7) * Cylinder((P["leg"]["tibia_tube_od"] + PR["clearance_fit"])/2, 16)
    slot_w = IF2["lug_d"] + 2*PR["clearance_fit"]            # 3.1
    # twist channel centers on the SEATED lug height: stub tip bottoms on the
    # bore-end shoulder (z=+0.6), lug rides lug_z_from_face above the tip
    circ_z = 0.6 - IF2["lug_z_from_face"]                    # -8.4
    for base_ang in (0, 180):
        # axial entry: from the boss mouth (z -14) up entry_len
        boss -= Rot(0, 0, base_ang) * \
            Pos(16/2 - 1.5, 0, -14 + IF2["entry_len"]/2 - 0.5) * \
            Box(6, slot_w, IF2["entry_len"] + 1.0)
        # circumferential twist arc (90 deg), stepped cuts
        import math as _m
        for k in range(13):
            a = base_ang + IF2["twist_deg"] * k / 12
            # detent: slot narrows just before the end seat (last 12 deg)
            w = slot_w - (IF2["detent_bump"] if 60 < IF2["twist_deg"] * k / 12 < 78
                          else 0.0)
            boss -= Rot(0, 0, a) * Pos(16/2 - 1.5, 0, circ_z) * Box(6, w, w)
    hub += boss
    # hinge lugs: pairs flanking each knuckle at 120 deg stations.
    # v0.2.2 (session 7): the lugs were FLOATING — v0.2 raised KNUCKLE_Z 4.2
    # to clear the cam but never re-attached them to the hub (hub top z=18,
    # lug bottom z=22.3; the slicer showed six mid-air islands, and the
    # interference checks can't see a DISCONNECTION — new single-solid check
    # below). Fix: a knuckle collar — an annular wall outboard of the cam's
    # swept rim, rising from the hub top to the lug top, with each lug
    # extended radially outward to meet it. Loads go lug -> collar -> hub
    # (a proper shoulder path, D020), and the collar caps the pin bores so
    # hinge pins can't walk out; pins insert through the collar from outside.
    lug = None
    for i in range(3):
        for side in (-1, 1):
            # widened 7 -> 10.5 in x: reaches from the hinge circle into the
            # collar wall (radial overlap proven by the union probe below)
            blk = Pos(HINGE_R + 1.75, side*7.0, KNUCKLE_Z - 1.2) * Box(10.5, 3.2, 7)
            blk = Rot(0, 0, 120*i) * blk
            lug = blk if lug is None else lug + blk
    collar_id_r = CAM_D/2 + 0.45              # 19.45: running gap to the cam rim
    collar_od_r = 22.0
    collar_top = KNUCKLE_Z + 2.3              # flush with the lug tops (z=29.3)
    collar = Pos(0, 0, (HUB_H + collar_top)/2) * \
        Cylinder(collar_od_r, collar_top - HUB_H)
    collar -= Pos(0, 0, (HUB_H + collar_top)/2) * \
        Cylinder(collar_id_r, collar_top - HUB_H + 2)
    # swept windows: the finger tab's upper-outer corner arcs out to r~20.8
    # mid-open (inside the collar wall) — subtract the tab envelope posed
    # through the full opening sweep (+3 deg overtravel, like the cam slots)
    # at each station, fattened +0.5 x/z, +0.6 y. Windows span ~±12 deg of
    # azimuth; the ring stays connected between stations and the pin bores
    # exit at ~±49 deg, well clear of the windows.
    for i in range(3):
        for ang in list(range(0, int(OPEN_DEG) + 3, 4)) + [OPEN_DEG + 3]:
            env = Pos(HINGE_R + 0.5, 0, KNUCKLE_Z + 1.4) * Box(9.0, 7.6, 7.0)
            hinge = Pos(HINGE_R, 0, KNUCKLE_Z) * Rot(0, -ang, 0) * \
                Pos(-HINGE_R, 0, -KNUCKLE_Z)
            collar -= Rot(0, 0, 120*i) * (hinge * env)
    lug += collar
    # trim the lug corners that poke past the collar OD
    lug &= Pos(0, 0, (collar_top + 10)/2) * Cylinder(collar_od_r, collar_top + 30)
    hub += lug
    # pin holes along the tangent (Y at station 0) — drilled AFTER the union,
    # lengthened 24 -> 46 so they pass clean through the collar wall too
    # (pins insert from outside; the far collar wall retains them)
    for i in range(3):
        hole = Pos(HINGE_R, 0, KNUCKLE_Z) * Rot(90, 0, 0) * Cylinder(2.1/2, 46)
        hub -= Rot(0, 0, 120*i) * hole
    return hub

# ---- cam kinematics (v0.2, real spiral) ----------------------------------
# The drive pin hangs from the finger tab at (dx, dz) = (-5.5, -4.7) from the
# hinge line. As the finger opens by theta, the pin's radial distance from the
# hand axis is rho(theta) = HINGE_R + dx*cos(theta) + dz_eff*sin(theta) —
# verified numerically below against the actual posed transform.
# Coupling: linear, finger 0..OPEN_DEG over cam rotation 0..CAM_SWEEP.
CAM_SWEEP = 51.0        # v0.2.1: keeps the v0.2 spiral pitch (60/65 per deg)
PIN_DX, PIN_DZ = -5.5, -4.7

def pin_radius(theta_deg):
    import math
    t = math.radians(theta_deg)
    return HINGE_R + PIN_DX*math.cos(t) - PIN_DZ*math.sin(t)

def cam_disc():
    import math
    cam = Pos(0, 0, CAM_Z0 + CAM_T/2) * Cylinder(CAM_D/2, CAM_T)
    # center bore: press onto SCS0009 horn (VERIFY horn dims with calipers)
    cam -= Pos(0, 0, CAM_Z0 + CAM_T/2) * Cylinder(5.0/2, CAM_T+2)
    # three Archimedean spiral slots: radius pin_radius(theta) as the disc
    # rotates CAM_SWEEP deg; carved as overlapping round cuts along the path.
    # v0.2.1: slot coverage computed from the TRUE posed pin axis, sampled at
    # three depths through the cam slab. A tilted cylinder's slab footprint
    # is radially elongated by T*tan(theta) on top of the 1/cos(theta) width
    # growth — the old linear guess (1.7+0.75f) undersized past ~40 deg
    # (found by the dense pose sweep; the D014 bug class again). At full
    # open the slot end breaches the outer rim: the slots are OPEN-ENDED by
    # design now — full-open end stop is the servo soft limit, not the cam.
    for i in range(3):
        base = 120*i
        for k in range(29):
            f = k/26                          # ~3 deg overtravel past OPEN
            th_deg = min(OPEN_DEG*f, OPEN_DEG + 3)
            th = math.radians(th_deg)
            a = math.radians(base + CAM_SWEEP*f)
            # posed pin root: world x_P (radial), z_P (height) at station 0
            x_p = pin_radius(th_deg)
            z_p = KNUCKLE_Z + PIN_DX*math.sin(th) + PIN_DZ*math.cos(th)
            cut_r = 1.4/math.cos(th) + 0.35
            for z_s in (CAM_Z0 - 0.2, CAM_Z0 + CAM_T/2, CAM_Z0 + CAM_T + 0.2):
                r_s = x_p + (z_p - z_s)*math.tan(th)   # axis radial at depth z_s
                cam -= Pos(r_s*math.cos(a), r_s*math.sin(a), CAM_Z0 + CAM_T/2) * \
                       Cylinder(cut_r, CAM_T+2)
    return cam

def sector_solid(half_angle_deg, r=200, h=400, zc=0):
    """A true angular sector about +X: intersection of two half-spaces whose
    boundary planes contain the Z axis. (Cylinder arc_size semantics burned us
    in v0.1 - each 'wedge' was nearly a full ring. Verified by cross-sections.)"""
    hs_pos = Rot(0, 0,  half_angle_deg) * Pos(0, -h/2, zc) * Box(2*r, h, h)
    hs_neg = Rot(0, 0, -half_angle_deg) * Pos(0,  h/2, zc) * Box(2*r, h, h)
    return hs_pos & hs_neg

def finger():
    """One cone-segment finger, modeled CLOSED at station 0 (+X)."""
    zc = KNUCKLE_Z + CONE_LEN/2
    outer = Pos(0, 0, zc) * Cone(CONE_BASE_R, CONE_TIP_R, CONE_LEN)
    inner = Pos(0, 0, zc - 0.6) * Cone(CONE_BASE_R - WALL, max(CONE_TIP_R - 1.8, 0.8), CONE_LEN)
    shell = outer - inner
    # keep a TRUE 116-deg sector centered on +X (4 deg gap between segments)
    sect = sector_solid(58, zc=zc)
    seg = shell & sect
    # rounded tip cap on this segment's share of the tip
    cap = (Pos(0, 0, KNUCKLE_Z + CONE_LEN) * Sphere(CONE_TIP_R)) & sect
    seg += cap
    # knuckle tab reaching inward to the hinge circle + drive pin stub
    tab = Pos(HINGE_R + 0.5, 0, KNUCKLE_Z + 1.4) * Box(8, 6.4, 6)
    tab -= Pos(HINGE_R, 0, KNUCKLE_Z) * Rot(90, 0, 0) * Cylinder(2.1/2, 20)  # hinge pin hole
    # cam-drive pin: longer in v0.2 (9 mm) so it stays engaged in the slot
    # through the full open sweep (pin z travels ~7.7 mm as the finger rotates)
    # v0.2.1: arm RAISED 3 mm — at >45 deg open the old arm dipped into the
    # cam slab (dense sweep found it); pin extended up 3 mm to keep the joint
    pin_arm = Pos(HINGE_R - 2.5, 0, KNUCKLE_Z + 1.9) * Box(7, 3.4, 3)        # arm to pin root
    pin = Pos(HINGE_R + PIN_DX, 0, KNUCKLE_Z + PIN_DZ/2 - 3.0) * Cylinder(2.8/2, 12 + abs(PIN_DZ))
    part = seg + tab + pin_arm + pin
    # v0.2: notch the cone-skirt where the hub's hinge lugs live (found by check)
    for sy in (1, -1):
        part -= Pos(14.5, sy*7.0, KNUCKLE_Z + 2.6) * Box(13, 4.6, 8.2)
    # v0.2: swept-clearance scallops — subtract the cam disc + hub rim
    # envelopes, inverse-posed through the hinge rotation, so the skirt clears
    # them at every opening angle BY CONSTRUCTION (cut excludes the pin zone).
    # v0.2.1: the 4-angle sweep left a ~7 mm^3 graze BETWEEN sampled angles
    # (BUILD_LOG 07-28) — densified to 4-deg steps with a fattened envelope
    # (+0.35 radial, +0.5 axial margin); verified by the dense pose sweep in
    # __main__ (0..65 deg in 5-deg steps, worst pair < 0.5 mm^3).
    sweep_angles = list(range(6, int(OPEN_DEG), 4)) + [OPEN_DEG]
    for ang in sweep_angles:
        inv = Pos(HINGE_R, 0, KNUCKLE_Z) * Rot(0, ang, 0) * Pos(-HINGE_R, 0, -KNUCKLE_Z)
        ring_cam = (Pos(0, 0, CAM_Z0 + CAM_T/2) * Cylinder(20.05, CAM_T + 1.5)) - \
                   (Pos(0, 0, CAM_Z0 + CAM_T/2) * Cylinder(8.8, CAM_T + 2.5))
        ring_cam -= Pos(14, 0, CAM_Z0 + CAM_T/2) * Box(14, 8, CAM_T + 4)   # keep pin lane
        part -= inv * ring_cam
    return part

def finger_posed(open_deg=0.0, station=0):
    """Finger at a 120deg station, rotated open about its tangential hinge line."""
    f = finger()
    hinge = Pos(HINGE_R, 0, KNUCKLE_Z) * Rot(0, -open_deg, 0) * Pos(-HINGE_R, 0, -KNUCKLE_Z)
    return Rot(0, 0, 120*station) * (hinge * f)

def hand_assembly(open_deg=0.0):
    # the cam co-rotates with opening: -CAM_SWEEP * (open/OPEN_DEG) about z
    cam_rot = -CAM_SWEEP * (open_deg / OPEN_DEG)
    parts = {
        "hub": hand_hub(),
        "cam": Rot(0, 0, cam_rot) * cam_disc(),
        "servo": scs0009_dummy(),
    }
    for i in range(3):
        parts[f"finger_{i}"] = finger_posed(open_deg, i)
    return parts

if __name__ == "__main__":
    hub = hand_hub(); cam = cam_disc(); fin = finger()
    export(hub, "hand_hub")
    export(cam, "hand_cam")
    export(fin, "hand_finger")
    for label, deg in (("closed", 0.0), ("open", OPEN_DEG)):
        combined = None
        for s in hand_assembly(deg).values():
            combined = s if combined is None else combined + s
        export(combined, f"hand_assembly_{label}", multi=True)

    # ---- v0.2 verification ----
    print(f"pin radial travel: {pin_radius(0):.2f} -> {pin_radius(OPEN_DEG):.2f} mm "
          f"over {OPEN_DEG:.0f} deg finger / {CAM_SWEEP:.0f} deg cam")
    # v0.2.1 dense sweep: check EVERY 5 deg of opening (the 3-pose check
    # missed the mid-swing graze the first time — lesson kept)
    worst, worst_at = 0.0, None
    hub_only = hub
    for deg in [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 52.5, OPEN_DEG]:
        cam_rot = -CAM_SWEEP * (deg / OPEN_DEG)
        cam_p = Rot(0, 0, cam_rot) * cam
        f = finger_posed(deg, 0)          # station 0 suffices (120-deg symmetry)
        for other, oname in ((hub_only, "hub"), (cam_p, "cam")):
            inter = f & other
            v = 0.0 if inter is None else inter.volume
            if v > worst:
                worst, worst_at = v, (deg, oname)
            if v > 0.5:
                bb = inter.bounding_box()
                print(f"  CLASH @{deg} deg: finger x {oname} = {v:.2f} mm^3 "
                      f"bbox x {bb.min.X:.1f}..{bb.max.X:.1f} "
                      f"y {bb.min.Y:.1f}..{bb.max.Y:.1f} "
                      f"z {bb.min.Z:.1f}..{bb.max.Z:.1f}")
    inter = cam & hub
    v = 0.0 if inter is None else inter.volume
    ok = worst <= 0.5 and v <= 0.5
    print(f"cam x hub: {v:.2f} mm^3 | worst finger clash over 14 poses: "
          f"{worst:.2f} mm^3 at {worst_at} ({'CLEAN' if ok else 'CLASHES'})")

    # ---- v0.2.2 connectivity audit (new check class) ---------------------
    # Interference checks can only see OVERLAP; a part that drifts APART is
    # invisible to them — exactly how the v0.2 hinge lugs shipped floating
    # 4.3 mm above the hub for two releases (slicer islands, session 7).
    # Every single-print part must be ONE connected solid.
    conn_ok = True
    for name, part in (("hub", hub), ("cam", cam), ("finger", fin)):
        n = len(part.solids())
        one = n == 1
        conn_ok &= one
        print(f"connectivity: {name} = {n} solid(s) "
              f"({'OK' if one else 'FLOATING PARTS'})")
    # lug attachment is real, not just tangent: the collar/lug ring overlaps
    # the hub body by a finite volume (probe, not a render judgment)
    ring_probe = hand_hub() & Pos(0, 0, HUB_H + 0.5) * Cylinder(25, 1.0)
    ring_v = 0.0 if ring_probe is None else ring_probe.volume
    print(f"collar root section at hub top: {ring_v:.1f} mm^3 "
          f"({'OK' if ring_v > 100 else 'TOO THIN'})")
    conn_ok &= ring_v > 100

    ok = ok and conn_ok
    print(f"part_hand checks: {'ALL CLEAN' if ok else 'FAILED'}")
    if not ok:
        import sys
        sys.exit(1)
