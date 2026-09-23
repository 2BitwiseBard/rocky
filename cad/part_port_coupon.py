"""Leg-port test coupon pair — prove I1 in 15 grams before printing 160.

Two palm-sized patches carrying the COMPLETE leg-port interface (same
iface.py code the real deck v0.3 + coxa v0.2 use — nothing re-modeled):

  port_coupon_deck  : deck patch — 2 dowel posts, hook through-slot,
                      2 heat-set pockets, cable cutout
  port_coupon_plate : coxa-plate patch — dowel bores, thumbscrew bores +
                      head wells, the inboard hook lip

The physical test (tonight, after the fit ladder):
  1. present the plate at ~15°, drop the lip through the slot
  2. slide inboard until the foot hooks under
  3. pivot flat — dowels must enter their bores WITHOUT force
  4. (if inserts on hand) melt 2x M3 inserts, run thumbscrews in
  5. tug every direction; note slop/binding in NOTES_INBOX

If this dance works on the coupons, deck v0.3 + coxa v0.2 print with
confidence. If it binds, session 5 fixes params, not printed parts.
"""
from build123d import *
from common import params, export
from iface import leg_port_deck_features, leg_port_plate_features, IF

P = params()
T_DECK = 6.0


def port_coupon_deck():
    # patch spans the port feature zone (leg-local x -56..-12, y +/-27)
    patch = Pos(-34, 0, T_DECK / 2) * Box(46, 54, T_DECK)
    patch = leg_port_deck_features(patch, top_z=T_DECK)
    return patch


def port_coupon_plate():
    patch = Pos(-34, 0, -2) * Box(46, 44, 4)
    patch = leg_port_plate_features(patch, plate_top_z=0.0, plate_bot_z=-4.0)
    return patch


if __name__ == "__main__":
    d = port_coupon_deck()
    p = port_coupon_plate()
    export(d, "port_coupon_deck")
    export(p, "port_coupon_plate")
    # docked-state boolean, exactly like the real deck check
    d_posed = Pos(0, 0, -10) * d          # deck top -> plate bottom plane
    inter = d_posed & p
    v = 0.0 if inter is None else inter.volume
    print(f"coupon dock (dowels in bores, lip in slot): {v:.2f} mm^3 "
          f"({'OK' if v < 1 else 'CLASH'})")
    for name, part in (("deck", d), ("plate", p)):
        bb = part.bounding_box()
        print(f"  port_coupon_{name}: {bb.size.X:.0f} x {bb.size.Y:.0f} x "
              f"{bb.size.Z:.0f} mm")
