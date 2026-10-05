# Independent check of one single-source claim: does each docked coxa base meet its carapace sector?
# Poses derived here from the repo modules only (not from the prep agents' keepouts.py).
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, "/home/bitwisebard/Development/rocky/cad")
from build123d import *
import part_coxa, part_shell, part_deck

v = lambda s: 0.0 if s is None else abs(s.volume)
base = part_coxa.coxa_yaw_base()                    # leg-local, plate z -4..0
bb = base.bounding_box()
print("base bbox x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f" % (bb.min.X, bb.max.X, bb.min.Y, bb.max.Y, bb.min.Z, bb.max.Z))
sector = part_shell.shell_sector()                  # as the module builds it
sb = sector.bounding_box()
print("sector bbox x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f" % (sb.min.X, sb.max.X, sb.min.Y, sb.max.Y, sb.min.Z, sb.max.Z))
print("STATIONS", part_deck.STATIONS, "R_STATION", part_deck.R_STATION)
deck = Pos(0, 0, -10) * part_deck.body_deck()       # deck model z 0..6 -> body z -10..-4
db = deck.bounding_box(); print("deck z %.2f..%.2f" % (db.min.Z, db.max.Z))
for k, ang in enumerate(part_deck.STATIONS):
    b = Rot(0, 0, ang) * Pos(part_deck.R_STATION, 0, 0) * base
    print("station %d (az %g): base x deck %.3f mm3" % (k, ang, v(b & deck)))
    # try every sector rotation: report which sector(s) this base meets
    for j, a2 in enumerate(part_deck.STATIONS):
        s = Rot(0, 0, a2) * sector
        inter = b & s
        x = v(inter)
        if x > 1e-6:
            ib = inter.bounding_box()
            print("   meets sector posed at az %g: %.3f mm3; overlap bbox x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f" % (
                a2, x, ib.min.X, ib.max.X, ib.min.Y, ib.max.Y, ib.min.Z, ib.max.Z))
    if k == 0:
        s0 = Rot(0, 0, ang) * sector
        print("   sector(az %g) x deck %.3f mm3" % (ang, v(s0 & deck)))
