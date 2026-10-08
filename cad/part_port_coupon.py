"""Leg-port test coupons — prove I1 in 15 grams before printing 160.

Palm-sized patches carrying the COMPLETE leg-port interface (same iface.py
code the real deck and coxa_yaw_base use — nothing re-modeled). Since
2026-10-07 (body-layout decision 15, B117) the port is the "seats" design,
and the owner prints the deck patch with BOTH plate patches first:

  port_coupon_deck           : deck patch — the 2 seat posts (cone + vee), the
                               6.2 hook through-slot, 2 heat-set pockets, cable cutout
  port_coupon_plate_relief46 : coxa-plate patch, x -46..-11 like the real plate
                               (+ the extension to the hook's stem): the 2 seat
                               sockets, thumbscrew bores + head wells, the L.
                               Underside relieved outboard of x -46: the repo's
                               seat_relief_x0 (the judge's graft)
  port_coupon_plate_relief42 : the same, relieved outboard of x -42 (the
                               designer's value): the pads also cover x -46..-42
  port_coupon_plate_fit10    : relief46 with both sockets 0.1 bigger radially,
  port_coupon_plate_fit20    : and 0.2 (the seat-fit test, I1_DOCK_OPTIONS 6.2)

Every plate's fit is absolute (0 / 0.1 / 0.2 over the nominal post), not
params print.seat_fit, which is what they measure. They print like the real
coxa_yaw_base: L on the bed, the plate's underside on support with a blocker
in both sockets, so the sockets, the pads and the 0.2 relief come out of the
same print condition. (A seat row on the fit ladder could not do that, and the
deck coupon's second post, 34.6 from the cone, stood under it.)

The D020 plate patch ran inboard to x -57, 11 past the real plate's edge: at
the hooking tilt its inboard end dips into the deck, so it could never dock.

The physical test (I1_DOCK_OPTIONS 6.1; after the fit ladder's row D):
  1. hold a plate patch at 7-10 deg (outboard end up), bring the L in
     radially, pass it through the 6.2 slot without force (binds: print the
     6.6 slot, row D)
  2. lower it onto the cones: it must centre itself
  3. (if inserts on hand) melt 2x M3 inserts, run the two thumb_knob_m3 in,
     each keyed onto an M3 x 16 hex bolt (B82: part_panel checks the reach);
     finger-tight, nothing rocks and there is no x or y play (< 0.05 on a dial)
  4. a 0.2 feeler must not enter under the inboard pads
  5. compare the two plates (pads to x -46 vs -42); note slop / binding / rock
     in NOTES_INBOX
  6. the seat fit: dock relief46, fit10, fit20 in turn. The smallest that sits
     with no rock and no light under the pads (a 0.2 feeler stays out) is
     print.seat_fit (D032: NOTES_INBOX, then params, then regenerate). Rock
     or play > 0.15 on all three means the seats fail: fall back to "minimal".

If this works on the coupons, the deck and the coxa base print with
confidence. If it binds, fix params, not printed parts.
"""
import numpy as np
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from build123d import *
from common import params, export
from iface import (IF, PLATE_X0, leg_port_deck_features, leg_port_plate_features, seat_posts_symdiff,
                   leg_port_socket_walls)

P = params()
T_DECK = 6.0          # B27 OPEN: the deck thickness (params body.deck_t says 4, unread)
PLATE_X1 = -11.0      # the patch's outboard end: past the seats' sockets (x <= -28.8)
PLATE_W = 44.0        # as the real plate (part_coxa): its y +-22 edges set the sockets' mouth walls
COUPONS = {           # coupon name -> (seat_relief_x0, socket fit): absolute, not params
    "relief46": (-46.0, 0.0), "relief42": (-42.0, 0.0),
    "fit10": (-46.0, 0.1), "fit20": (-46.0, 0.2)}
assert COUPONS["relief46"][0] == IF["leg_port"]["seat_relief_x0"], \
    "port_coupon_plate_relief46 must carry the repo's seat_relief_x0: rename the coupons"


def port_coupon_deck():
    # patch spans the port feature zone (leg-local x -57..-11, y +/-27): the hook's foot
    # (x >= -52.5 docked) catches under it
    patch = Pos(-34, 0, T_DECK / 2) * Box(46, 54, T_DECK)
    patch = leg_port_deck_features(patch, top_z=T_DECK)
    return patch


def port_coupon_plate(relief_x0=None, fit=None):
    """The plate patch, x PLATE_X0..PLATE_X1 like the real plate there (relief_x0 / fit:
    None = params seat_relief_x0 / print.seat_fit, i.e. the coxa_yaw_base's own)."""
    patch = Pos((PLATE_X0 + PLATE_X1) / 2, 0, -2) * Box(PLATE_X1 - PLATE_X0, PLATE_W, 4)
    patch = leg_port_plate_features(patch, plate_top_z=0.0, plate_bot_z=-4.0, relief_x0=relief_x0, fit=fit)
    return patch


def _v(x):
    return 0.0 if x is None else x.volume


def _gap(a, b):
    return BRepExtrema_DistShapeShape(a.wrapped, b.wrapped).Value()


if __name__ == "__main__":
    d = port_coupon_deck()
    export(d, "port_coupon_deck")
    d_posed = Pos(0, 0, -10) * d          # deck top -> plate bottom plane
    # the patch's OWN posts (docked frame): the plates dock against them, and they must be
    # leg_port_seats() as drawn (with iface's posts a patch that lost its posts passed)
    sd, posts = seat_posts_symdiff(d_posed, -4.0)
    ok = sd < 0.01 and posts is not None
    print(f"port_coupon_deck carries its seat posts as drawn: symmetric difference {sd:.4f} mm^3 "
          f"({'OK' if ok else 'POSTS MISSING / WRONG'})")
    bad = int(not ok)                     # no posts: every plate's cone test below fails too
    slab = d_posed & (Pos(-34, 0, -7) * Box(60, 60, 6))     # the deck patch without its posts
    c30 = np.cos(np.deg2rad(IF["leg_port"]["seat_half_angle"]))
    for tag, (rx0, fit) in COUPONS.items():
        p = port_coupon_plate(rx0, fit)
        export(p, f"port_coupon_plate_{tag}")
        # docked-state boolean, exactly like the real deck check (part_deck, check_dock)
        v = _v(d_posed & p)
        # pressed 0.05 lower the deck meets only the pads (x < rx0): z is the pads' and the cones'
        pressed = (Pos(0, 0, -0.05) * p) & slab
        pads = _v(pressed)
        padx = pressed.bounding_box().max.X if pads > 0 else float("nan")
        # the cones: at fit 0 they bear (pressed 0.05 down the plate meets the posts: zero play,
        # nominal); at fit f the flanks stand f cos 30 off (radial play f, the pads carry z)
        bear = 0.0 if posts is None else _v((Pos(0, 0, -0.05) * p) & posts)
        gap = float("nan") if posts is None else _gap(p, posts)
        cones = posts is not None and (bear > 0.01 if fit == 0 else abs(gap - fit * c30) < 0.005 and bear < 1e-3)
        ok = v < 0.01 and pads > 0.01 and padx <= rx0 + 0.01 and cones
        bad += not ok
        print(f"coupon dock, {tag} (relieved outboard of x {rx0:.0f}, sockets +{fit:.1f}): {v:.3f} mm^3; "
              f"pressed 0.05 down it meets the deck at the pads {pads:.3f} mm^3 (x <= {padx:.2f}) and the "
              f"posts {bear:.3f} mm^3; cone flank gap {gap:.4f} (expect {fit * c30:.4f}) "
              f"({'OK' if ok else 'CLASH' if v >= 0.01 else 'SEATS / PADS DO NOT BEAR AS DRAWN'})")
        bb = p.bounding_box()
        wc, wv = leg_port_socket_walls(PLATE_W / 2, fit)
        print(f"  port_coupon_plate_{tag}: {bb.size.X:.1f} x {bb.size.Y:.0f} x {bb.size.Z:.1f} mm "
              f"(x {bb.min.X:.2f}..{bb.max.X:.2f}); socket mouth to the y edge: cone {wc:.2f}, vee {wv:.2f}"
              + (" (NOTE: under the 1.20 of fit 0; if this fit wins, I1_DOCK_OPTIONS 7: mouth 0.2 "
                 "or the seats at |y| 17.1, before any coxa_yaw_base)" if wv < 1.2 - 1e-6 else ""))
    bb = d.bounding_box()
    print(f"  port_coupon_deck: {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f} mm")
    if bad:                               # a run_all_checks module: fail loudly
        raise SystemExit(1)
