"""Panel standard demo set (I3) + accessory dovetail coupons (I6) — D020.

  shell_sector_demo : one carapace shell sector blank (1/5 of the pentagon
                      perimeter, flat demo version) carrying the standard:
                      seating lip, ONE latch insert pocket, 2 magnet pockets,
                      and TWO male dovetail segments (I6) on its outer face.
  frame_coupon      : the mating frame, 6 thick like the deck: lip groove +
                      the deck's own latch strike + magnet pockets, laid out
                      to mate the demo (D063, B87) — print both, exercise the
                      standard before committing the real (curved) shells.
  dovetail_shoe     : female accessory shoe (I6): the undercut slot + a set
                      knob on a 45-deg spot face (B83).
  thumb_knob_m3     : printed knurled knob keyed onto an M3 hex head (ISO
                      4017, 5.5 A/F) — used by the leg port (I1: M3 x 16,
                      B82) and the dovetail shoe set screw (M3 x 12 or 16).

The latch cartridge itself (iface.latch_insert_housing / _rotor) is exported
by part_battery; this file checks it: the rotor goes into the housing, and on
a strike it drops in at OPEN, cams under the land at LOCKED and is worked from
below (latch_engagement, which part_deck runs on the real deck too).

The real shells stay D006 (cosmetic, bolt-on, sculpted later); this file
freezes how EVERY one of them will attach.
"""
import numpy as np
from build123d import *
from common import params, export
from iface import (IF, latch_insert_housing, latch_insert_rotor, latch_strike, latch_lug,
                   latch_housing_tf, latch_pocket, latch_d_profile,
                   LATCH_ENTRY_DEG, LATCH_LAND, LATCH_GAP, LATCH_REACH, LATCH_SLOT, LATCH_FLAT_R,
                   LATCH_KEYWAY_DEG, LATCH_SHAFT_D, dovetail_male, dovetail_female_shoe, set_knob_tf)

P = params()
PR = P["print"]
FIT = PR["clearance_fit"]
PL = IF["panel_latch"]
DV = IF["dovetail"]


# D063 (B87): the demo and the coupon are ONE mating pair, laid out from the latch.
# As first drawn they matched nowhere (latch-to-lip 26.5 vs strike-to-groove 16,
# magnets at +/-46 vs +/-20, a 3.0 groove on a 3.0 lip wall, and the coupon's second
# peg slot turned about the coupon's origin, 12 mm from its bore). The demo's inner
# face (z DEMO_T, the lip side) sits on the coupon's top: flipped by demo_on_coupon()
# its latch lands on the strike, its +y lip wall in the groove, its magnets on the
# coupon's.
DEMO_W, DEMO_H, DEMO_T = 116, 64, 2.4
LIP_W, LIP_H = 3.0, 2.4           # the seating lip ring, 4 in from the edge
DEMO_LATCH_Y = 10.5
LATCH_TO_LIP = DEMO_H / 2 - 4 - LIP_W / 2 - DEMO_LATCH_Y    # 16.0 to the lip wall's centre
MAG_X = 16.0                      # the magnet pairs, on the latch's line
MAG_D, MAG_T = PL["magnet_d"] + 0.25, PL["magnet_t"] + 0.2  # pocket
MAG_BOSS = DV["depth"]            # the demo's pockets were through its 2.4 plate: a boss under
                                  # each, down to the bed with the males' faces (a 1.8 one floated)
COUPON_T = 6.0                    # the deck's 6: the strike needs LATCH_REACH + 0.2


def shell_sector_demo():
    W, H, T = DEMO_W, DEMO_H, DEMO_T             # one sector blank (flat demo)
    p = Pos(0, 0, T / 2) * Box(W, H, T)
    # seating lip (loads live here)
    lip = Pos(0, 0, T + LIP_H / 2) * Box(W - 8, H - 8, LIP_H)
    lip -= Pos(0, 0, T + LIP_H / 2) * Box(W - 8 - 2 * LIP_W, H - 8 - 2 * LIP_W, LIP_H + 2)
    p += lip
    # two magnet pockets from the inner face, each over a boss on the outer face
    for sx in (-MAG_X, MAG_X):
        p += Pos(sx, DEMO_LATCH_Y, -MAG_BOSS / 2) * Cylinder(MAG_D / 2 + 2.0, MAG_BOSS)
        p -= Pos(sx, DEMO_LATCH_Y, T - MAG_T / 2 + 0.01) * Cylinder(MAG_D / 2, MAG_T)
    # one latch insert pocket (the housing's bottom flush with the inner face), the D of
    # the keyway index (B107) turned to the coupon's strike
    p -= demo_latch_tf() * latch_pocket(-2.0, T + 2.0)
    # two I6 male dovetail segments on the OUTER face (z=0 side), running x
    for sx in (-W / 4, W / 4):
        p += Pos(sx, H / 4, 0) * Rot(0, 180, 0) * Rot(0, 0, 90) * dovetail_male(undercut=True)
    # vent gill slots (backlog B9 preview — they cost nothing here)
    for k in range(4):
        p -= Pos(-W / 2 + 20 + k * 10, -H / 4, T / 2) * Rot(0, 0, 20) * \
            Box(3, 16, T + 2)
    return p


def frame_coupon():
    """The body-side frame a panel seats against, 6 thick like the deck: the deck's
    own latch strike (iface.latch_strike) at the origin, the lip groove (the lip
    wall + FIT each side) and two magnets, laid out to mate shell_sector_demo."""
    W, H, T = 60, 34, COUPON_T
    f = Pos(0, -6, T / 2) * Box(W, H, T)
    f -= Pos(0, -LATCH_TO_LIP, T - (LIP_H + FIT) / 2 + 0.05) * \
        Box(W + 2, LIP_W + 2 * FIT, LIP_H + FIT + 0.1)                    # lip groove
    f -= Pos(0, 0, T) * latch_strike(T)
    # magnet pockets (frame side: NORTH showing, i.e. away from the robot, INTERFACES I3)
    for sx in (-MAG_X, MAG_X):
        f -= Pos(sx, 0, T - MAG_T / 2 + 0.01) * Cylinder(MAG_D / 2, MAG_T)
    return f


def demo_on_coupon():
    """shell_sector_demo flipped onto frame_coupon: its latch on the strike."""
    return Pos(0, DEMO_LATCH_Y, COUPON_T + DEMO_T) * Rot(180, 0, 0)


def demo_latch_tf():
    """The latch frame in the demo's own coordinates (z 0 on its inner face, +z into the
    plate, +x the strike's +x once flipped onto the coupon): demo_on_coupon() times this
    is the coupon's latch frame, Pos(0, 0, COUPON_T)."""
    return Pos(0, DEMO_LATCH_Y, DEMO_T) * Rot(180, 0, 0)


def dovetail_male_coupon():
    """Standalone I6 male segment on a thin base — fit-test against the shoe
    without printing the whole 116 mm shell sector."""
    base = Pos(0, 0, 1.5) * Box(34, 22, 3)
    return base + Pos(0, 0, 3) * Rot(0, 0, 90) * dovetail_male(undercut=True)


# thumb_knob_m3 (B82): an ISO 4017 M3 head is 5.5 A/F x 2.0 tall. The floor
# stays at 6.5 so an M3 x 16 on the 4 mm coxa plate still takes 5.5 of the
# deck insert's 5.7 mm; the knob grows 8.0 -> 8.6 so the whole head sits in a
# 2.1 pocket (it stood 0.5 proud of a 1.5 pocket: the extrude ran past the top).
KNOB_H = 8.6
HEX_FLOOR = 6.5
HEX_AF = 5.6                     # 5.5 + 0.1: keys the head (2.0 mm^3 clash turned 15 deg)
HEX_HEAD = (5.5, 2.0)            # ISO 4017 M3: s, k
THUMBSCREW_L = 16.0              # BOM A-19


def thumb_knob_m3():
    """Knurled M3 knob: 5.6 A/F hex pocket (floor z 6.5, 2.1 deep) keys a hex head; 12 dia grip."""
    d = IF["leg_port"]["thumbscrew_head_d"]
    knob = Pos(0, 0, KNOB_H / 2) * Cylinder(d / 2, KNOB_H)
    for k in range(12):
        a = k * 30
        knob -= Rot(0, 0, a) * Pos(d / 2 + 0.4, 0, KNOB_H / 2) * Cylinder(1.1, KNOB_H + 1)
    knob -= Pos(0, 0, HEX_FLOOR) * extrude(RegularPolygon(HEX_AF / 2 / np.cos(np.pi / 6), 6),
                                           KNOB_H - HEX_FLOOR + 1)   # M3 hex head pocket
    knob -= Pos(0, 0, HEX_FLOOR / 2 - 1) * Cylinder(3.4 / 2, HEX_FLOOR + 2)  # shaft clear
    return knob


def _vol(s):
    return 0.0 if s is None else s.volume


def _annulus(xy, r0, r1, z0=-40.0, z1=40.0):
    return Pos(xy[0], xy[1], (z0 + z1) / 2) * (Cylinder(r1, z1 - z0) - Cylinder(r0, z1 - z0))


def hex_bolt(L, s=HEX_HEAD[0], k=HEX_HEAD[1]):
    """ISO 4017 M3 x L: head bearing face at z 0, a Ø3 (thread major) shank to z -L."""
    head = extrude(RegularPolygon(s / 2 / np.cos(np.pi / 6), 6), k)
    return head + Pos(0, 0, -L / 2) * Cylinder(1.5, L)


def thumbscrew_reach(L=THUMBSCREW_L):
    """B82: the knob + an M3 x L hex bolt on the port coupon pair (the same iface
    code as body_deck, coxa_yaw_base and the jig), every height measured from the
    solids: returns (engaged in the insert, tip below the deck underside,
    head-in-pocket clash, head proud of the knob, keyed clash at 15 deg, bolt x parts)."""
    from part_port_coupon import port_coupon_deck, port_coupon_plate
    xy = IF["leg_port"]["thumbscrew_xy"][0]
    plate = port_coupon_plate()
    deck = Pos(0, 0, -10) * port_coupon_deck()            # docked, as the coupon check
    plate_top = (plate & _annulus(xy, 2.5, 4.0)).bounding_box().max.Z
    ring = (deck & _annulus(xy, 2.6, 4.0)).bounding_box()  # solid deck round the insert
    deck_top, deck_bot = ring.max.Z, ring.min.Z
    insert_bot = (deck & _annulus(xy, 1.8, 2.2)).bounding_box().max.Z  # under the Ø4.6 pocket
    knob = Pos(xy[0], xy[1], plate_top) * thumb_knob_m3()
    seat = (knob & _annulus(xy, 1.9, 2.6)).bounding_box().max.Z        # the hex pocket floor
    bolt = Pos(xy[0], xy[1], seat) * hex_bolt(L)
    tip = seat - L
    engaged = deck_top - max(tip, insert_bot)
    proud = bolt.bounding_box().max.Z - knob.bounding_box().max.Z
    turned = Pos(xy[0], xy[1], seat) * Rot(0, 0, 15) * hex_bolt(L)
    return (engaged, deck_bot - tip, _vol(bolt & knob), proud, _vol(turned & knob),
            _vol(bolt & (plate + deck)))


def set_knob_reach(shoe, male):
    """B83: the I6 set knob on the shoe's spot face, measured against a seated male.
    Returns (knob x shoe, knob x male, knob's lowest z, the bolt length at which
    its tip first meets the male, the thread length in the shoe)."""
    tf = set_knob_tf()
    knob = tf * thumb_knob_m3()
    lo, hi = HEX_FLOOR, 40.0                      # bisect: the shortest bolt that touches
    for _ in range(16):
        mid = (lo + hi) / 2
        if _vol((tf * Pos(0, 0, HEX_FLOOR) * hex_bolt(mid)) & male) > 1e-3:
            hi = mid
        else:
            lo = mid
    # thread engagement: a Ø3 rod along the bore overlaps the Ø2.8 tapped wall
    rod = tf * Pos(0, 0, -10) * Cylinder(1.5, 20)
    thread = _vol(rod & shoe) / (np.pi * (1.5 ** 2 - (PR["screw_m3_tap"] / 2) ** 2))
    return (_vol(knob & shoe), _vol(knob & male), knob.bounding_box().min.Z, hi, thread)


def _lugs_col(phi, z0, z1):
    """Both lugs' footprints beyond the bore (no FIT), turned phi from the entry line."""
    x0, a = LATCH_SHAFT_D / 2 + FIT, LATCH_ENTRY_DEG + phi
    return Rot(0, 0, a) * latch_lug(0.0, z0, z1, x0=x0) + \
        Rot(0, 0, a + 180) * latch_lug(0.0, z0, z1, x0=x0)


def pocket_skin(xy, z_open, z_closed, d, t=0.5):
    """The 0.5 mm skin a magnet pocket (dia d) must be wrapped in: a sleeve over its
    depth + a floor disc beyond its closed end (z_closed above or below z_open)."""
    lo, hi = sorted((z_open, z_closed))
    sleeve = Pos(xy[0], xy[1], (lo + hi) / 2) * (Cylinder(d / 2 + t, hi - lo) - Cylinder(d / 2, hi - lo))
    fz = z_closed + (t / 2 if z_closed > z_open else -t / 2)
    return sleeve, Pos(xy[0], xy[1], fz) * Cylinder(d / 2 + t, t)


def walled(part, xy, z_open, z_closed, d):
    """(wall fraction, floor fraction) of a pocket's 0.5 mm skin that is material."""
    sleeve, floor = pocket_skin(xy, z_open, z_closed, d)
    return _vol(sleeve & part) / sleeve.volume, _vol(floor & part) / floor.volume


def latch_engagement(frame, tf):
    """I3 (B87): the cartridge on a frame's strike, every number measured. tf places
    the latch frame (z 0 = the seating plane, the strike's +x) in the frame's
    coordinates; the rotor turns phi from the entry line (OPEN 0, LOCKED 90)."""
    housing, rotor = latch_insert_housing(), latch_insert_rotor()

    def rot(phi, dz=0.0):
        return tf * Pos(0, 0, dz) * Rot(0, 0, LATCH_ENTRY_DEG + phi) * rotor

    def blade(phi, dz=0.0):          # a flat screwdriver, 0.9 x 5, its tip 0.1 off the slot floor
        z_tip = -LATCH_REACH + LATCH_SLOT[1] - 0.1 + dz
        return tf * Rot(0, 0, LATCH_ENTRY_DEG + phi) * Pos(0, 0, z_tip - 15) * Box(0.9, 5.0, 30)
    m = {"housing": _vol((tf * latch_housing_tf() * housing) & frame)}
    m["drop"] = max(_vol((rot(0, dz) + tf * Pos(0, 0, dz) * latch_housing_tf() * housing) & frame)
                    for dz in np.arange(0.0, 8.01, 0.5))
    m["turn"] = [(phi, _vol(rot(phi) & frame)) for phi in range(0, 91, 5)]
    m["contact_deg"] = next((phi for phi, v in m["turn"] if v > 0.01), None)
    m["bite"] = m["turn"][-1][1]
    m["stop_open"], m["stop_lock"] = _vol(rot(-8) & frame), _vol(rot(98) & frame) - m["bite"]
    lug_top = -(LATCH_LAND + LATCH_GAP)
    col = tf * _lugs_col(90, lug_top, 0.0)
    m["land"] = _vol(col & frame) / col.volume
    m["hold"] = _vol(rot(90, 0.2) & frame)            # a locked panel lifting 0.2
    m["blade"] = max(_vol(blade(phi) & (frame + rot(phi))) for phi in (0, 90))
    m["blade_bites"] = min(_vol(blade(phi, 0.3) & rot(phi)) for phi in (0, 90))  # the slot floor
    return m


def latch_verdict(m):
    """The latch rules: the cartridge seats and drops in at OPEN, turns free for the
    first 20 deg, bites 1-5 mm^3 at LOCKED under a full land, holds, stops at both
    ends, and a screwdriver reaches its slot from below."""
    bad = []
    if max(m["housing"], m["drop"], m["turn"][0][1]) > 0.01:
        bad.append("I3 cartridge does not drop into the strike at OPEN")
    if m["contact_deg"] is None or m["contact_deg"] < 20 or not 1.0 <= m["bite"] <= 5.0:
        bad.append("I3 cam does not bite (or jams) at LOCKED")
    if m["land"] < 0.95 or m["hold"] < 1.0:
        bad.append("I3 lugs are not under the land at LOCKED")
    if m["stop_open"] < 1.0 or m["stop_lock"] < 1.0:
        bad.append("I3 rotor turns past its stops")
    if m["blade"] > 0.01 or m["blade_bites"] < 0.01:
        bad.append("I3 operating slot not reachable from below")
    return bad


def latch_report(m):
    turn = ", ".join(f"{phi}:{v:.2f}" for phi, v in m["turn"] if phi % 15 == 0)
    return (f"housing x frame {m['housing']:.2f}, drop-in at OPEN {m['drop']:.2f} mm^3; "
            f"turn (deg:mm^3) {turn}; first contact {m['contact_deg']} deg, bite at LOCKED "
            f"{m['bite']:.2f} mm^3, land over the lugs {100 * m['land']:.0f} %, lifted 0.2 "
            f"{m['hold']:.1f}; past the stops -8 deg {m['stop_open']:.1f} / +8 {m['stop_lock']:.1f}; "
            f"a screwdriver from below: in the slot {m['blade']:.2f}, 0.3 up it meets the slot "
            f"floor {m['blade_bites']:.2f}")


def cartridge_assembly():
    """B87: the rotor drops into the housing through the keyways (sweep, lugs on the
    keyways), and turned 90 deg off them it cannot come back out (captive)."""
    housing, rotor = latch_insert_housing(), latch_insert_rotor()
    t = PL["housing_t"]
    sweep = max(_vol((Pos(0, 0, dz) * rotor) & housing)
                for dz in np.arange(0.0, t + LATCH_REACH + 1.01, 0.5))
    seated = max(_vol((Rot(0, 0, a) * rotor) & housing) for a in (0, 45, 90))
    captive = _vol((Pos(0, 0, LATCH_LAND + LATCH_GAP + 1.0) * Rot(0, 0, 90) * rotor) & housing)
    r = PL["housing_d"] / 2                     # its D outline (B107), 0.05 thick
    skin = latch_d_profile(r, LATCH_FLAT_R, 0.0, t) - latch_d_profile(r - 0.05, LATCH_FLAT_R - 0.05, -1.0, t + 1.0)
    return sweep, seated, captive, _vol(skin & housing) / skin.volume


def latch_seat(panel, tf, turn=10.0):
    """B107: the housing in a panel's D pocket (tf = the latch frame in the panel's
    coordinates). seated: on its index; turned: the least of +-turn deg and a half turn
    (180 deg about its axis); play: the largest turn (0.5 deg steps) it takes without
    meeting the panel. Upside down it seats as on its index: the housing is symmetric
    top to bottom, keyways and flat included, so that is the same part in the same place."""
    housing = latch_insert_housing()

    def at(d):
        return _vol((tf * Rot(0, 0, LATCH_ENTRY_DEG + LATCH_KEYWAY_DEG + d) * housing) & panel)
    m = {"seated": at(0.0), "turned": min(at(d) for d in (turn, -turn, 180.0))}
    m["play"] = max([d for d in np.arange(0.5, turn, 0.5) if max(at(d), at(-d)) < 0.01] or [0.0])
    return m


def latch_index(panel, tf, turn=10.0):
    """B107, the keyway index, every number measured. tf places the latch frame in the
    panel's coordinates. The housing seats on its index and not turned off it (+-turn
    deg or a half turn); with the housing on its index the rotor lifted 1 mm into it is
    captive at every angle a strike lets it turn (-8..98 deg from the entry line, 1 deg
    steps) and drops through only near LATCH_KEYWAY_DEG; returns how far past LOCKED
    (90) and back past OPEN (0) the first angle it passes lies."""
    housing, rotor = latch_insert_housing(), latch_insert_rotor()
    hx = latch_housing_tf() * housing
    m = latch_seat(panel, tf, turn)
    lift = LATCH_LAND + LATCH_GAP + 1.0              # the lug tops 1 mm up into the housing

    def held(phi):
        return _vol((Pos(0, 0, lift) * Rot(0, 0, LATCH_ENTRY_DEG + phi) * rotor) & hx)
    m["captive_min"] = min(held(phi) for phi in range(-8, 99))
    m["keyway_sweep"] = max(_vol((Pos(0, 0, dz) * Rot(0, 0, LATCH_ENTRY_DEG + LATCH_KEYWAY_DEG) * rotor) & hx)
                            for dz in np.arange(0.0, PL["housing_t"] + LATCH_REACH + 1.01, 0.5))
    free = [phi for phi in range(99, 172) if held(phi) < 0.01]
    m["margin_lock"] = (min(free) - 90) if free else None
    free = [phi for phi in range(-80, -7) if held(phi) < 0.01]
    m["margin_open"] = (0 - max(free)) if free else None
    return m


def index_verdict(m):
    bad = []
    if m["seated"] > 0.01 or m["turned"] < 1.0:
        bad.append("I3 housing not held to its keyway index")
    if m["captive_min"] < 1.0 or m["keyway_sweep"] > 0.01 or not m["margin_lock"] or not m["margin_open"]:
        bad.append("I3 keyways inside the latch's travel (or the rotor cannot pass them)")
    return bad


def index_report(m):
    return (f"housing on its index {m['seated']:.2f} mm^3 (play +-{m['play']:.1f} deg), turned "
            f"+-10 deg or a half turn >= {m['turned']:.1f}; rotor lifted 1 mm, -8..98 deg: >= {m['captive_min']:.1f} mm^3 "
            f"(captive); at the keyways ({LATCH_KEYWAY_DEG:.0f} deg) it drops through "
            f"({m['keyway_sweep']:.2f}); the first angle it passes is {m['margin_lock']} deg past "
            f"LOCKED, {m['margin_open']} back past OPEN")


if __name__ == "__main__":
    parts = dict(shell_sector_demo=shell_sector_demo(),
                 frame_coupon=frame_coupon(),
                 dovetail_shoe=dovetail_female_shoe(),
                 dovetail_male_coupon=dovetail_male_coupon(),
                 thumb_knob_m3=thumb_knob_m3())
    for n, p in parts.items():
        export(p, n)
    # I6 engagement: male segment must slide through the shoe slot
    male = dovetail_male(undercut=True)
    shoe = parts["dovetail_shoe"]
    posed = Pos(0, 0, 0.0) * male                 # both z=0-based, same axis
    inter = posed & shoe
    v = 0.0 if inter is None else inter.volume
    bad = []
    if v >= 1:
        bad.append("I6 dovetail binds")
    print(f"I6 dovetail male x shoe: {v:.2f} mm^3 ({'SLIDES' if v < 1 else 'BINDS'})")
    # I6 retention (B83): lifted 1 mm off the mount face the shoe must meet the
    # undercut (the old key let it off at 0 mm^3), and the set knob must reach
    v_lift = _vol((Pos(0, 0, 1.0) * shoe) & male)
    if v_lift < 1:
        bad.append("I6 shoe lifts off the male")
    k_shoe, k_male, k_low, l_meet, thread = set_knob_reach(shoe, male)
    if k_shoe > 0.01 or k_male > 0.01 or k_low < 1.0:
        bad.append("I6 set knob does not sit on its spot face")
    if l_meet > 12.0 or thread < 3.0:
        bad.append("I6 set screw misses the male or has too little thread")
    print(f"I6 shoe lifted 1.0: {v_lift:.2f} mm^3 ({'RETAINED' if v_lift >= 1 else 'LIFTS OFF'}); "
          f"set knob x shoe {k_shoe:.2f}, x male {k_male:.2f} mm^3, {k_low:.2f} above the mount face; "
          f"a hex bolt meets the male at {l_meet:.2f} mm (M3 x 12 reaches, need <= 12; "
          f"A-19's M3 x 16 has {THUMBSCREW_L - l_meet:.1f} of travel), {thread:.1f} mm of thread in the shoe")
    # I3 cartridge (B87): the rotor goes into the housing (the old closed track took
    # >= 17.8 mm^3 of housing at any of 36 angles) and, turned off the keyways, stays
    sweep, seated, captive, skin = cartridge_assembly()
    if sweep > 0.01 or seated > 0.01 or captive < 1.0 or skin < 0.99:
        bad.append("I3 cartridge does not assemble (or is not captive)")
    print(f"I3 rotor into the housing (keyway sweep): {sweep:.2f} mm^3, seated at 0/45/90 deg "
          f"{seated:.2f}, turned 90 and lifted {captive:.1f} mm^3 (captive), "
          f"Ø14 skin {100 * skin:.0f} % intact")
    # I3 mating pair: the demo flipped onto the coupon, the cartridge on its strike
    frame, demo = parts["frame_coupon"], demo_on_coupon() * parts["shell_sector_demo"]
    v_seat = _vol(demo & frame)
    lip_free = max(_vol((Pos(0, dy, 0) * demo) & frame) for dy in (-(FIT - 0.05), FIT - 0.05))
    lip_loc = min(_vol((Pos(0, dy, 0) * demo) & frame) for dy in (-(FIT + 0.3), FIT + 0.3))
    ctf = Pos(0, 0, COUPON_T)
    cart = ctf * latch_housing_tf() * latch_insert_housing() + ctf * Rot(0, 0, LATCH_ENTRY_DEG) * latch_insert_rotor()
    v_cart = _vol(cart & demo)
    mag = Cylinder(PL["magnet_d"] / 2, PL["magnet_t"])       # bottomed (the cuts sit 0.01 proud)
    mags = []
    for sx in (-MAG_X, MAG_X):
        d_mag = demo_on_coupon() * Pos(sx, DEMO_LATCH_Y, DEMO_T - MAG_T + 0.02 + PL["magnet_t"] / 2) * mag
        c_mag = Pos(sx, 0, COUPON_T - MAG_T + 0.02 + PL["magnet_t"] / 2) * mag
        off = np.hypot(d_mag.center().X - c_mag.center().X, d_mag.center().Y - c_mag.center().Y)
        mags.append((off, _vol(d_mag & (frame + demo)) + _vol(c_mag & (frame + demo)),
                     walled(parts["shell_sector_demo"], (sx, DEMO_LATCH_Y), DEMO_T, DEMO_T - MAG_T, MAG_D),
                     walled(frame, (sx, 0), COUPON_T, COUPON_T - MAG_T, MAG_D)))
    pair_ok = (v_seat < 0.01 and lip_free < 0.01 and lip_loc > 0.5 and v_cart < 0.01 and
               all(o < 0.01 and v < 0.01 and min(*dw, *cw) > 0.99 for o, v, dw, cw in mags))
    if not pair_ok:
        bad.append("I3 demo + frame coupon are not a mating pair")
    print(f"I3 demo on the frame coupon: {v_seat:.2f} mm^3; lip in the groove shifted "
          f"+/-{FIT - 0.05:.2f} {lip_free:.2f}, +/-{FIT + 0.3:.1f} {lip_loc:.1f} mm^3 (located, "
          f"with clearance); cartridge x demo {v_cart:.2f}; magnet pairs off-axis "
          f"{max(o for o, *_ in mags):.3f} mm, x parts {max(v for _, v, *_ in mags):.2f} mm^3, "
          f"pocket walls / floors >= {100 * min(min(*dw, *cw) for *_, dw, cw in mags):.0f} %")
    m = latch_engagement(frame, ctf)
    bad += latch_verdict(m)
    print(f"I3 latch on the frame coupon's strike: {latch_report(m)}")
    # I3 keyway index (B107): the demo's D pocket holds the housing one way, keyways
    # outside the travel (part_shell runs the same on the sectors)
    mi = latch_index(parts["shell_sector_demo"], demo_latch_tf())
    bad += index_verdict(mi)
    print(f"I3 keyway index in the demo: {index_report(mi)}")
    # I1 thumbscrew (B82): an M3 x 16 in the knob must bite the insert and stay in the deck
    eng, out, v_in, proud, v_key, v_parts = thumbscrew_reach()
    if eng < 5.0 or out > 0.0:
        bad.append("I1 thumbscrew misses the insert or exits the deck")
    if v_in > 0.01 or proud > 0.0 or v_key < 0.5 or v_parts > 0.01:
        bad.append("I1 hex head not seated and keyed in the knob")
    print(f"I1 M3 x {THUMBSCREW_L:.0f} hex in the knob: {eng:.2f} mm in the insert "
          f"(need >= 5), tip {-out:.2f} inside the deck underside; head x knob "
          f"{v_in:.2f} mm^3, {proud:+.2f} proud, turned 15 deg {v_key:.2f} mm^3 "
          f"(keyed), bolt x plate + deck {v_parts:.2f} mm^3")
    for n, p in parts.items():
        bb = p.bounding_box()
        print(f"  {n}: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f}")
    print(f"part_panel checks: {'CLEAN' if not bad else 'FAIL — ' + '; '.join(bad)}")
    raise SystemExit(1 if bad else 0)
