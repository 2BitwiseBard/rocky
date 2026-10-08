#!/usr/bin/env python3
"""Mass audit: CAD volumes -> link masses and the torso's mass distribution (D039; D064).

The MJCF carried a hand-typed budget (M_TORSO 1.35, M_COXA 0.14, M_FEMUR
0.03, M_TIBIA 0.17 kg) since session 2. This derives every link from the
CURRENT cad/out STL volumes (x PLA density x print_estimate.FILL, the
per-part fill factors — NOT its batch list, so re-planning the print batches
never moves a mass) plus the non-printed hardware in params.yaml
`mass_hw` — and writes sim/mass_budget.json, which build_mjcf.py now reads.

D064 (pick 13, B97, B138): the torso is no longer a mass on two cylinders.
Every torso item gets a body-frame POSE (the table below: z = leg z, the deck
z -10..-4, +x east), so the budget carries the torso's CoM and its full
inertia tensor about it, and build_mjcf writes them as the torso's explicit
<inertial>. A printed part's centroid and own inertia come from its STL (the
solid at uniform density, scaled to its mass); a bought part is a box of its
bounding dims (a disc for the spread wiring). The torso of D063 was 1439.8 g
on two cylinders at (0, 0, 24.5): option A puts the 380 g pack and the hub
under the deck, so its CoM drops ~27 mm (z 24.5 -> -2.5).

electronics_pod (params 400 g, an estimate: sensors + compute + bus adapter +
wiring) is split BY LOCATION, every share VERIFY: compute and sensors on the
tray (Pi 5 + cooler on the 11 mm standoffs, the IMU under it, the camera and
the tray's leads), the lidar on the hatch cap (B12, not built), the five hub
boards where params hub_shelf packs them, and the rest (looms, 14 AWG, XT60s,
fuse, inserts, screws, magnets) spread under the deck. If params changes the
pod's mass the shares scale with it.

Honesty: fill factors are +-30 % and mass_hw is all VERIFY. This is the
right SHAPE of the mass distribution, not the truth; bench day's kitchen
scale replaces every number here (NOTES_INBOX -> params -> regen).

  python3 mass_audit.py                  # prints the tables, writes mass_budget.json
  python3 mass_audit.py --allow-missing  # DEV ONLY: a part with no STL yet (bay_tub, bay_lid,
                                         # bay_door, hub_shelf) gets the proposal's PROVISIONAL
                                         # mass and a params pose; the budget says "provisional": true
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "..", "cad")
sys.path.insert(0, CAD)
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from print_estimate import stl_volume_cm3, PLA, FILL         # noqa: E402  (stdlib only)
import rocky_model as rm                                      # noqa: E402  (no build123d: runs without the cad extra)

P = rm.params()
HW = P["mass_hw"]
IF = P["interfaces"]
BS, AT, HS = IF["battery_sled"], IF["avionics_tray"], IF["hub_shelf"]
# D052: servo masses come from the actuator identity (params `actuators:`);
# mass_hw.servo_* are YAML aliases of the same numbers, kept for old readers
SERVO_G = float(P["actuators"][P["leg"].get("servo", "st3215")]["mass_g"])
CLAW_SERVO_G = float(P["actuators"][P["leg"].get("claw_servo", "scs0009")]["mass_g"])
PETG = 1.27
OUT = os.path.join(CAD, "out")

# ---- frame constants the pose table needs that are part geometry, not params (each cited;
# re-read them if that part changes)
DECK_TOP_Z = rm.DECK_BOT_Z_MM + 6.0   # -4: the 6 deck (B27) from -10; part_deck models it 0..6
TRAY_GAP_DRAWN = 3.4                   # part_avionics: the rails' channel holds the tray plate 3.4
                                       # over the deck top as drawn; + tray_lift (params) since D064
TRAY_REST_DROP = 0.3                   # part_avionics.TRAY_Z = SEAT_Z - FIT: the tray RESTS with its
                                       # lugs on the channel floor (plate 5.1 over the deck top, its
                                       # gravity pose and its latched one), 0.3 under the centred 5.4
TONGUE_Y = float(AT["plate_l"]) / 2 + 8.0   # part_avionics.TONGUE_Y 43: the latch axis (tray frame);
                                            # tray_latch_boss is exported in the tongue's frame
TRAY_T = float(AT["plate_t"])
RAIL_POSE_X = float(AT["plate_w"]) / 2 + 5.3   # part_avionics.RAIL_POSE_X: rail block centre off
                                               # the tray centre (0.3 clearance + the 10 block / 2)
PI_BOARD_DX = 10.0                     # part_avionics: the Pi's centre off its hole pattern, USB end
SLED_RAIL_H = 2.10                     # part_bay.SLED_Z0 - floor: part_battery's 4.0 rail into the
                                       # sled's 1.9 groove, the rest lift part_bay measures (was 2.25)
SLED_T = 3.0                           # part_battery.SLED_T: the sled floor the pack sits on
YAW_HUB_Z1 = 1.3 + 6.0                 # cad/leg_frame: fork lower hub 1.3 over the plate, 6 thick
STATIONS = rm.stations_deg()
R_ST = float(P["body"]["circumradius"])

# electronics_pod split by location: grams at the params 400 g estimate (scaled with it).
# Every share is VERIFY (a guess from datasheets and a parts count, nothing weighed)
POD_REF_G = 400.0
POD_SHARE_G = {
    "pi5 + cooler + SD": 67.0,      # Pi 5 46 g + Active Cooler ~20 + microSD (BOM B-03 / B-04)
    "imu": 5.0,                     # BNO085 board 3 g + grommets, M2.5 x 10 + nuts (C-01, B4)
    "camera + tray leads": 23.0,    # Camera Module 3 Wide + cable, the tray's XT30, 5 V and bus leads
    "lidar": 45.0,                  # LDRobot D500, 45 g by its datasheet (C-02); hatch-cap mount B12
    "bus_adapter": 14.0,            # Waveshare Bus Servo Adapter (A), face-down on the shelf (pick 14)
    "buck_5v": 9.0,                 # Pololu D24V50F5 + its 1000 uF can
    "ubec_6v": 11.0,                # Hobbywing UBEC-3A, 11 g (BOM B-11)
    "node_12v": 10.0,               # 30.5 x 30.5 FPV PDB (pick 9)
    "star": 30.0,                   # 40 x 30 perfboard, 7 XH headers, the mated plugs at the board
    "wiring + small hardware": 186.0,   # the rest: 5 leg looms + XT30 pairs, 14 AWG, XT60s, fuse,
                                        # heat-set inserts, screws, magnets, latch cartridges
}
assert abs(sum(POD_SHARE_G.values()) - POD_REF_G) < 1e-9
# the hub boards' envelopes (plan w x l along the board's own x / y, stack height): params
# hub_shelf.boards places them; part_avionics.BOARDS and the hub_shelf notes size them
SHELF_BOARDS = {"bus_adapter": (42.0, 33.0, 15.1), "buck_5v": (27.8, 20.3, 10.0),
                "ubec_6v": (43.0, 17.0, 8.2), "node_12v": (36.0, 36.0, 12.6), "star": (40.0, 30.0, 22.4)}
WIRING_DISC = dict(r=80.0, t=10.0, z=-15.0)   # ESTIMATE: the looms and hardware as a disc in the
                                              # under-deck layer, centred on the body axis
LIDAR_BOX = (54.0, 46.3, 35.0)                 # D500 (STL-19P datasheet, VERIFY), seated ON the cap

# --allow-missing: the parts the body-layout round adds, until their STLs exist. Masses are
# the task's / proposal's estimates (BODY_LAYOUT_PROPOSAL s2: tub + door 56 g at fill 0.5;
# lid ~20, shelf ~25); the poses are the params boxes. PROVISIONAL: never final numbers.
PROVISIONAL_G = {"bay_tub": 52.0, "bay_door": 4.0, "bay_lid": 20.0, "hub_shelf": 25.0}
NEW_BODY_FRAME_PARTS = tuple(PROVISIONAL_G)    # the audit checks each centroid lands in its params box
# the material each prints in (cad/gen_print_pack.py PARTS): the bay parts are PETG, the shelf PLA
NEW_PART_RHO = {"bay_tub": PETG, "bay_door": PETG, "bay_lid": PETG, "hub_shelf": PLA}


def _ry(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# each new part's export frame -> the body frame (read from the module that exports it): bay_tub and
# bay_lid are exported in the body frame (part_bay: their print poses ARE it, the tub open top up,
# the lid's roof down), bay_door lying on its outer face (part_bay.DOOR_PRINT = Pos(0, 0, -bay_door_x)
# * Rot(0, -90, 0), so body = Ry(+90) p + (bay_door_x, 0, 0)), hub_shelf in the body frame
# (part_busboard: plate down, its print pose)
NEW_PART_POSE = {"bay_tub": (None, (0.0, 0.0, 0.0)), "bay_lid": (None, (0.0, 0.0, 0.0)),
                 "bay_door": (_ry(90.0), (float(BS["bay_door_x"]), 0.0, 0.0)),
                 "hub_shelf": (None, (0.0, 0.0, 0.0))}


def printed_g(name, material=PLA):
    v = stl_volume_cm3(os.path.join(OUT, f"{name}.stl"))
    return v * material * FILL.get(name, 0.6)


# ------------------------------------------------------------------ geometry helpers
_STL_DT = np.dtype([("n", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])


def stl_mass_props(path):
    """(volume mm^3, centroid (3,) mm, inertia tensor about the centroid per unit density
    (3, 3) mm^5) of a closed binary STL, by signed tetrahedra from the origin: the tetra
    (0, a, b, c) of volume v has covariance v/20 (aa' + bb' + cc' + ss'), s = a + b + c."""
    raw = open(path, "rb").read()
    n = int.from_bytes(raw[80:84], "little")
    tri = np.frombuffer(raw, _STL_DT, count=n, offset=84)["v"].astype(np.float64)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    v6 = np.einsum("ij,ij->i", a, np.cross(b, c))           # 6 x each signed tetra volume
    vol = v6.sum() / 6.0
    s = a + b + c
    cen = (v6[:, None] * s).sum(0) / 24.0 / vol
    w = v6 / 120.0
    C = (np.einsum("i,ij,ik->jk", w, a, a) + np.einsum("i,ij,ik->jk", w, b, b)
         + np.einsum("i,ij,ik->jk", w, c, c) + np.einsum("i,ij,ik->jk", w, s, s))
    C = C - vol * np.outer(cen, cen)                        # about the centroid
    sign = 1.0 if vol > 0 else -1.0                         # inside-out normals flip both
    vol, C = sign * vol, sign * C
    return vol, cen, np.trace(C) * np.eye(3) - C


def stl_bounds(path):
    raw = open(path, "rb").read()
    n = int.from_bytes(raw[80:84], "little")
    pts = np.frombuffer(raw, _STL_DT, count=n, offset=84)["v"].reshape(-1, 3)
    return pts.min(0).astype(float), pts.max(0).astype(float)


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tf(R=None, t=(0.0, 0.0, 0.0)):
    return (np.eye(3) if R is None else np.asarray(R, float), np.asarray(t, float))


def compose(a, b):
    """a after b: p -> Ra (Rb p + tb) + ta."""
    return a[0] @ b[0], a[0] @ b[1] + a[1]


def station_tf(i):
    """Leg i's frame -> body: Rot(z, station) * Pos(R_STATION, 0, 0) (cad/iface.station_tf)."""
    R = rz(STATIONS[i])
    return R, R @ np.array([R_ST, 0.0, 0.0])


def box_inertia(m, dims):
    a, b, c = dims
    return m / 12.0 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])


class Item:
    """One torso line: grams, body-frame CoM (mm), inertia about it (g mm^2, body axes)."""

    def __init__(self, name, g, com, inertia, src, note=""):
        self.name, self.g, self.src, self.note = name, float(g), src, note
        self.com = np.asarray(com, float)
        self.I = np.asarray(inertia, float)


def combine(parts, name, src, note=""):
    """parts: [(g, com, I_about_com)] -> one Item (parallel axis)."""
    m = sum(p[0] for p in parts)
    com = sum(p[0] * np.asarray(p[1], float) for p in parts) / m
    I = np.zeros((3, 3))
    for g, c, Ic in parts:
        d = np.asarray(c, float) - com
        I += Ic + g * (d @ d * np.eye(3) - np.outer(d, d))
    return Item(name, m, com, I, src, note)


def box_item(name, g, centre, dims, R=None, src="hw", note=""):
    R = np.eye(3) if R is None else R
    return Item(name, g, centre, R @ box_inertia(g, dims) @ R.T, src, note)


def box_span(name, g, xs, ys, zs, src, note=""):
    """A box given by its (lo, hi) spans, body frame."""
    return box_item(name, g, [(xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2, (zs[0] + zs[1]) / 2],
                    [xs[1] - xs[0], ys[1] - ys[0], zs[1] - zs[0]], src=src, note=note)


def plates(name, g, spans, src, note=""):
    """g spread over several boxes by volume (a thin-walled shell as its plates)."""
    vols = [(x[1] - x[0]) * (y[1] - y[0]) * (z[1] - z[0]) for x, y, z in spans]
    parts = []
    for (x, y, z), v in zip(spans, vols):
        b = box_span("", g * v / sum(vols), x, y, z, src)
        parts.append((b.g, b.com, b.I))
    return combine(parts, name, src, note)


def disc_item(name, g, centre, r, t, src="share", note=""):
    ixx = g * (3 * r * r + t * t) / 12.0
    return Item(name, g, centre, np.diag([ixx, ixx, g * r * r / 2.0]), src, note)


class MissingSTL(RuntimeError):
    pass


def stl_item(name, stem, rho, tfs, src="STL", note="", expect=None):
    """A printed part from its STL: mass = volume x rho x FILL (the old rule, per copy),
    centroid and inertia from the mesh at uniform density, one copy per transform.
    expect: a ((x0, x1), (y0, y1), (z0, z1)) box its body-frame centroid must fall in
    (the new parts are posed by NEW_PART_POSE: a wrong pose lands elsewhere)."""
    path = os.path.join(OUT, f"{stem}.stl")
    if not os.path.exists(path):
        raise MissingSTL(stem)
    g1 = stl_volume_cm3(path) * rho * FILL.get(stem, 0.6)
    vol, cen, Iu = stl_mass_props(path)
    dens = g1 / vol
    parts = [(g1, R @ cen + t, R @ (Iu * dens) @ R.T) for R, t in tfs]
    it = combine(parts, name, src, note)
    if expect is not None:
        grow = 15.0
        ok = all(lo - grow <= c <= hi + grow for c, (lo, hi) in zip(it.com, expect))
        if not ok:
            raise RuntimeError(
                f"{stem}.stl's centroid {np.round(it.com, 1).tolist()} is not inside its params box "
                f"{expect} (+-{grow:g}): it is not exported in the body frame. Export it posed (as "
                f"part_shell exports the carapace) or give mass_audit its pose")
    return it


# ------------------------------------------------------------------ the torso pose table
def torso_items(allow_missing=False):
    """Every torso item with its body-frame pose. Returns (items, provisional names)."""
    items, provisional = [], []
    pod = float(HW["electronics_pod"]) / POD_REF_G

    def share(key):
        return POD_SHARE_G[key] * pod

    # the deck: modelled z 0..6 (+ the seat posts), posed at the deck bottom (part_deck)
    items.append(stl_item("body_deck", "body_deck", PLA, [tf(t=(0, 0, rm.DECK_BOT_Z_MM))],
                          note="posed z -10..-4"))
    # the carapace: part_shell exports one sector at azimuth 0 and the cap in the body frame
    items.append(stl_item("shell_sector x5", "shell_sector", PETG, [tf(rz(72 * k)) for k in range(5)],
                          note="Rot(z, 72 k)"))
    items.append(stl_item("shell_cap", "shell_cap", PETG, [tf()], note="body frame"))
    cap_top = stl_bounds(os.path.join(OUT, "shell_cap.stl"))[1][2]

    # the avionics tray (I4, pick 7 / 11): tray_xy, turned tray_rot_deg, tray_lift over the drawn
    # 3.4, resting (part_avionics.TRAY_Z: the lugs on the channel floor, 0.3 under the centred pose)
    tx, ty = (float(v) for v in AT["tray_xy"])
    Rt = rz(float(AT["tray_rot_deg"]))
    tz0 = DECK_TOP_Z + TRAY_GAP_DRAWN + float(AT["tray_lift"]) - TRAY_REST_DROP
    tray = tf(Rt, (tx, ty, tz0))
    items.append(stl_item("avionics_tray", "avionics_tray", PLA, [tray],
                          note=f"({tx:g}, {ty:g}) rot {AT['tray_rot_deg']:g}, plate z {tz0:.1f}"))
    # its I3 boss, glued under the tongue (exported in part_avionics.tongue_tf()'s frame: on the
    # latch axis, z 0 its underside = the deck top with the tray resting)
    boss_h = tz0 - DECK_TOP_Z                              # part_avionics.BOSS_H = TRAY_Z
    by_ = sorted((Rt @ np.array([0.0, yy, 0.0]))[1] + ty for yy in (28.0, TONGUE_Y + 8.0))
    items.append(stl_item("tray_latch_boss", "tray_latch_boss", PLA,
                          [compose(tray, tf(t=(0.0, TONGUE_Y, -boss_h)))],
                          expect=((tx - 15.0, tx + 15.0), tuple(by_), (DECK_TOP_Z, tz0)),
                          note=f"under the tongue, z {DECK_TOP_Z:g}..{tz0:.1f}"))
    rails = [compose(tf(Rt, (tx, ty, DECK_TOP_Z)), tf(rz(a), rz(a) @ np.array([RAIL_POSE_X, 0, 0])))
             for a in (0.0, 180.0)]
    rails = [compose(r, tf(rz(90.0))) for r in rails]      # the block's length along the rails
    items.append(stl_item("tray_rail x2", "tray_rail", PLA, rails, note="on the deck top"))
    # what rides on the tray: the Pi (USB end params pi_usb_end, body frame) on its standoffs
    usb = np.array([1.0 if AT["pi_usb_end"] == "+x" else -1.0, 0.0, 0.0])
    sgn = 1.0 if (Rt.T @ usb)[0] > 0 else -1.0
    std = float(AT["pi_standoff_h"])
    items.append(box_item("pi5 + cooler + SD", share("pi5 + cooler + SD"),
                          Rt @ np.array([sgn * PI_BOARD_DX, 0.0, TRAY_T + std + 6.0]) + [tx, ty, tz0],
                          (85.0, 56.0, 12.0), Rt, "share", f"on the {std:g} mm standoffs, USB {AT['pi_usb_end']}"))
    items.append(box_item("imu", share("imu"), Rt @ np.array([0.0, 13.5, 9.0]) + [tx, ty, tz0],
                          (25.4, 22.86, 4.6), Rt, "share", "BNO085 on its grommets under the Pi"))
    items.append(box_item("camera + tray leads", share("camera + tray leads"),
                          Rt @ np.array([0.0, 0.0, TRAY_T + 5.0]) + [tx, ty, tz0], (80.0, 60.0, 10.0), Rt,
                          "share", "ESTIMATE: spread over the tray"))
    items.append(box_item("lidar", share("lidar"), (0.0, 0.0, cap_top + LIDAR_BOX[2] / 2), LIDAR_BOX,
                          src="share", note=f"on the cap top z {cap_top:.1f} (B12 mount not built)"))

    # the keel tub (I5, option A): the sled + pack in it, then the tub's own parts
    tub = rm.bay_tub_extent_mm()
    zin0 = float(BS["bay_top_z"]) - float(BS["bay_roof"]) - float(BS["bay_h"])
    bx, by = (float(v) for v in BS["bay_centre"])
    items.append(stl_item("battery_sled", "battery_sled", PLA, [tf(t=(bx, by, zin0 + SLED_RAIL_H))],
                          note=f"on the tub's rails, z {zin0 + SLED_RAIL_H:.2f}"))
    pack = (float(BS["pack_l"]), float(BS["pack_w"]), float(BS["pack_h"]))
    pz = zin0 + SLED_RAIL_H + SLED_T + pack[2] / 2
    items.append(box_item("battery", HW["battery_3s_5200"], (bx, by, pz), pack, src="hw",
                          note=f"pack centre z {pz:.2f}"))
    x0, x1 = tub["x"]
    y0, y1 = tub["y"]
    z0, z1 = tub["z"]
    wall, flo, roof, door = (float(BS[k]) for k in ("bay_wall", "bay_floor", "bay_roof", "door_t"))
    xd = x0 + door                                        # the interior's door face
    zr = z1 - roof                                        # the roof's underside
    fallback = {
        "bay_tub": lambda g: plates("bay_tub", g, [((xd, x1), (y0, y1), (z0, z0 + flo)),        # floor
                                                   ((xd, x1), (y0, y0 + wall), (z0 + flo, zr)),  # walls
                                                   ((xd, x1), (y1 - wall, y1), (z0 + flo, zr)),
                                                   ((x1 - wall, x1), (y0 + wall, y1 - wall), (z0 + flo, zr))],
                                    "PROVISIONAL", "U: floor, side + nose walls (no chamfer)"),
        "bay_lid": lambda g: box_span("bay_lid", g, (xd, x1), (y0, y1), (zr, z1), "PROVISIONAL",
                                      "the roof plate (bosses not modelled)"),
        "bay_door": lambda g: box_span("bay_door", g, (x0, xd), (y0, y1), (z0, z1), "PROVISIONAL",
                                       "the end door, -x"),
        "hub_shelf": lambda g: _shelf_fallback(g),
    }
    expect = {"bay_tub": (tub["x"], tub["y"], tub["z"]), "bay_lid": (tub["x"], tub["y"], tub["z"]),
              "bay_door": (tub["x"], tub["y"], tub["z"]),
              "hub_shelf": tuple(rm.hub_shelf_extent_mm()[k] for k in "xyz")}
    for stem in NEW_BODY_FRAME_PARTS:
        try:
            R, t = NEW_PART_POSE[stem]
            items.append(stl_item(stem, stem, NEW_PART_RHO[stem], [tf(R, t)], expect=expect[stem],
                                  note="body frame" if R is None else "print frame -> body (DOOR_PRINT)"))
        except MissingSTL:
            if not allow_missing:
                raise
            print(f"WARNING: cad/out/{stem}.stl does not exist: PROVISIONAL {PROVISIONAL_G[stem]:g} g "
                  f"at its params pose (--allow-missing; the budget is marked provisional)")
            items.append(fallback[stem](PROVISIONAL_G[stem]))
            provisional.append(stem)

    # the hub boards (pick 14), as params hub_shelf.boards packs them
    pt = float(HS["plate_top_z"])
    pb = pt - float(HS["plate_t"])
    for key, b in HS["boards"].items():
        w, l, h = SHELF_BOARDS[key]
        zc = pb - h / 2 if b["face"] == "down" else pt + h / 2
        R = rz(float(b["rot_deg"]))
        items.append(box_item(key, share(key), (float(b["xy"][0]), float(b["xy"][1]), zc), (w, l, h), R,
                              "share", f"face {b['face']}, z {zc - h / 2:.1f}..{zc + h / 2:.1f}"))
    items.append(disc_item("wiring + small hardware", share("wiring + small hardware"),
                           (0.0, 0.0, WIRING_DISC["z"]), WIRING_DISC["r"], WIRING_DISC["t"],
                           note=f"ESTIMATE: a disc r {WIRING_DISC['r']:g} under the deck"))

    # the five stations: the yaw servo (shaft down, case hanging -x, cad/leg_frame) + the base
    S = P["servo_st3215"]
    zc_yaw = YAW_HUB_Z1 + float(S["body_h"]) + float(S["out_boss_h"]) + float(S["horn_t"]) \
        - float(S["body_h"]) / 2                           # leg_frame.ZC_YAW, 26.0
    c_leg = np.array([float(S["shaft_offset"]) - float(S["body_l"]) / 2, 0.0, zc_yaw])
    dims = (float(S["body_l"]), float(S["body_w"]), float(S["body_h"]))
    parts = []
    for i in range(len(STATIONS)):
        R, t = station_tf(i)
        parts.append((SERVO_G, R @ c_leg + t, R @ box_inertia(SERVO_G, dims) @ R.T))
    items.append(combine(parts, "yaw servos x5", "hw", f"case box at leg ({c_leg[0]:.1f}, 0, {zc_yaw:.1f})"))
    items.append(stl_item("coxa_yaw_base x5", "coxa_yaw_base", PLA,
                          [station_tf(i) for i in range(len(STATIONS))], note="station_tf"))
    return items, provisional


def _shelf_fallback(g):
    """The shelf as its plate (params x / y, plate_t under plate_top_z) and three Ø post_d posts
    from the deck's underside to the plate: 22 + 3 of the provisional 25 g."""
    pt, t, d = float(HS["plate_top_z"]), float(HS["plate_t"]), float(HS["post_d"])
    plate = box_span("", g * 22.0 / 25.0, HS["x"], HS["y"], (pt - t, pt), "PROVISIONAL")
    parts = [(plate.g, plate.com, plate.I)]
    for x, y in HS["posts"]:
        p = box_span("", g * 1.0 / 25.0, (x - d / 2, x + d / 2), (y - d / 2, y + d / 2),
                     (pt, rm.DECK_BOT_Z_MM), "PROVISIONAL")
        parts.append((p.g, p.com, p.I))
    return combine(parts, "hub_shelf", "PROVISIONAL", "plate + three posts")


# ------------------------------------------------------------------ the legs (stance CoM only)
def leg_com_leg_frame(q, mc, mf, mt):
    """(CoM (3,) leg frame mm, g) of one leg at q, laid out as build_mjcf.leg_xml lays it
    (pebble_feasibility.leg_com_leg_frame is the same layout)."""
    leg = P["leg"]
    L1, L2, L3, zh = (float(leg[k]) for k in ("l1_coxa", "l2_femur", "l3_tibia", "hip_axis_z"))
    rc_ = rm.foot_contact_radius_mm()
    q1, q2, q3 = q
    c1, s1 = math.cos(q1), math.sin(q1)

    def along(sf, st):
        r = L1 + sf * math.cos(q2) + st * math.cos(q2 + q3)
        return np.array([r * c1, r * s1, zh + sf * math.sin(q2) + st * math.sin(q2 + q3)])
    prong = 4.0
    ps = ((mc, np.array([20.0 * c1, 20.0 * s1, 30.0])), (mf, along(L2 / 2, 0.0)),
          (0.7 * mt - 2 * prong, along(L2, 0.46 * L3)), (0.3 * mt, along(L2, L3 - rc_)),
          (2 * prong, along(L2, L3 + 12.0)))
    m = sum(p[0] for p in ps)
    return sum(p[0] * p[1] for p in ps) / m, m


def robot_com_at_stance(torso_g, torso_com, mc, mf, mt):
    from pebble_gait import WaveGait, leg_ik, body_to_leg, leg_to_body
    g = WaveGait(**rm.gait_defaults())
    acc, mass = torso_g * np.asarray(torso_com, float), torso_g
    for i in range(len(STATIONS)):
        q = leg_ik(body_to_leg(i, g.p_nom[i]))
        c, m = leg_com_leg_frame(q, mc, mf, mt)
        acc = acc + m * leg_to_body(i, c)
        mass += m
    return acc / mass


# ------------------------------------------------------------------ main
D063_TORSO = (1439.8, (0.0, 0.0, 24.5))     # the last cylinder budget (0.75 x 18 + 0.25 x 44)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--allow-missing", action="store_true",
                    help="DEV ONLY: PROVISIONAL masses for body-layout parts with no STL yet")
    args = ap.parse_args(argv)
    path = os.path.join(HERE, "mass_budget.json")
    prev = json.load(open(path)) if os.path.exists(path) else None
    try:
        titems, provisional = torso_items(allow_missing=args.allow_missing)
    except MissingSTL as e:
        print(f"ERROR: cad/out/{e}.stl does not exist. Build the CAD tree first (cd cad && python "
              f"run_all_checks.py), or pass --allow-missing for a PROVISIONAL dev budget", file=sys.stderr)
        return 2
    torso = combine([(it.g, it.com, it.I) for it in titems], "torso", "sum")

    # ---- per leg links (the moving mass the gait carries)
    coxa = {                     # rotates about yaw: fork + femur servo
        "coxa_fork": printed_g("coxa_fork"),
        "femur servo": SERVO_G,
        "horn_coupler": printed_g("horn_coupler"),
        "fasteners": HW["fasteners_per_leg"] / 3,
    }
    femur = {
        "femur_link": printed_g("femur_link"),
        "femur_plate_b": printed_g("femur_plate_b", PETG),            # D047 idler-side plate
        "horn_coupler": printed_g("horn_coupler"),
        "fasteners": HW["fasteners_per_leg"] / 3,
    }
    tibia = {                    # swings about the knee: knee servo + carrier + shin + hand
        "tibia_knee_carrier": printed_g("tibia_knee_carrier"),
        "knee servo": SERVO_G,
        "tibia_tube": HW["tibia_tube"],
        "tibia_sea_outer": printed_g("tibia_sea_outer"),
        "tibia_sea_slider": printed_g("tibia_sea_slider"),
        "sea_spring_switch": HW["sea_spring_switch"],
        "hand_hub": printed_g("hand_hub"),
        "hand_cam": printed_g("hand_cam"),
        "hand_finger x3": 3 * printed_g("hand_finger"),
        "claw servo": CLAW_SERVO_G,
        "foot_pad_tpu": HW["foot_pad_tpu"],
        "fasteners": HW["fasteners_per_leg"] / 3,
    }
    budget_old = {"torso": 1350.0, "coxa": 140.0, "femur": 30.0, "tibia": 170.0}
    out = {}

    # ---- the torso table: every item, its grams and its body-frame CoM
    out["torso"] = round(torso.g, 1)
    com = np.round(torso.com, 2)
    I_si = torso.I * 1e-9                                  # g mm^2 -> kg m^2
    full = [I_si[0, 0], I_si[1, 1], I_si[2, 2], I_si[0, 1], I_si[0, 2], I_si[1, 2]]
    out["torso_com_mm"] = [float(v) for v in com]
    out["torso_inertia"] = [round(float(v), 9) for v in full]
    print(f"\nTORSO  {torso.g:7.1f} g   (old budget {budget_old['torso']:.0f} g, "
          f"{100 * (torso.g / budget_old['torso'] - 1):+.0f} %)")
    print(f"   {'item':26s} {'g':>7s} {'x':>7s} {'y':>7s} {'z':>7s}  source")
    for it in titems:
        print(f"   {it.name:26s} {it.g:7.1f} {it.com[0]:7.1f} {it.com[1]:7.1f} {it.com[2]:7.1f}  "
              f"{it.src:11s} {it.note}")
    pm, pc = (prev["torso"], tuple(prev["torso_com_mm"])) if prev and "torso_com_mm" in prev else \
        ((prev or {}).get("torso", D063_TORSO[0]), D063_TORSO[1])
    print(f"   torso CoM ({com[0]:.2f}, {com[1]:.2f}, {com[2]:.2f}) mm; the previous budget "
          f"{pm:.1f} g at ({pc[0]:.1f}, {pc[1]:.1f}, {pc[2]:.1f}); D063 {D063_TORSO[0]} g at "
          f"(0, 0, {D063_TORSO[1][2]}) on the two cylinders")
    ev = np.linalg.eigvalsh(I_si)
    print("   inertia about the CoM, kg m^2 (ixx iyy izz ixy ixz iyz): "
          + " ".join(f"{v:.4e}" for v in full) + f"; principal {ev[0]:.4e} {ev[1]:.4e} {ev[2]:.4e}")

    for name, d in (("coxa", coxa), ("femur", femur), ("tibia", tibia)):
        tot = sum(d.values())
        out[name] = round(tot, 1)
        print(f"\n{name.upper():6s}  {tot:7.1f} g   (old budget {budget_old[name]:.0f} g, "
              f"{100 * (tot / budget_old[name] - 1):+.0f} %)")
        for k, v in d.items():
            print(f"   {k:22s} {v:7.1f}")
    total = out["torso"] + 5 * (out["coxa"] + out["femur"] + out["tibia"])
    old_total = 1350 + 5 * (140 + 30 + 170)
    print(f"\nROBOT  {total:7.1f} g  (old budget {old_total:.0f} g, "
          f"{100 * (total / old_total - 1):+.0f} %)")
    rc_new = robot_com_at_stance(out["torso"], com, out["coxa"], out["femur"], out["tibia"])
    rc_old = robot_com_at_stance(D063_TORSO[0], D063_TORSO[1], out["coxa"], out["femur"], out["tibia"])
    print(f"robot CoM at the planted stance (body frame, ground z {-rm.gait_defaults()['body_height']:.0f}): "
          f"({rc_new[0]:.2f}, {rc_new[1]:.2f}, {rc_new[2]:.2f}) mm; with the D063 torso "
          f"({rc_old[0]:.2f}, {rc_old[1]:.2f}, {rc_old[2]:.2f})")
    # hand mass alone, for the sim's hand geoms
    hand = tibia["hand_hub"] + tibia["hand_cam"] + tibia["hand_finger x3"] + tibia["claw servo"]
    out["hand"] = round(hand, 1)
    out["provisional"] = bool(provisional)
    if provisional:
        out["provisional_items"] = provisional
        print(f"\nWARNING: PROVISIONAL budget ({', '.join(provisional)} had no STL): not a number to "
              f"publish; re-run without --allow-missing once the CAD tree has built them")
    out["_note"] = ("derived from cad/out STL volumes x PLA/PETG x print_estimate fill "
                    "factors + params.mass_hw (ALL VERIFY); grams")
    out["_note_torso"] = ("D064: torso_com_mm = the torso's CoM (mm, body frame: z = leg z, deck "
                          "-10..-4); torso_inertia = ixx iyy izz ixy ixz iyz about it (kg m^2, body "
                          "axes, MuJoCo fullinertia order), from mass_audit's pose table")
    with open(path, "w") as f:
        json.dump(out, f, indent=1)
    print("written: sim/mass_budget.json" + (" (PROVISIONAL)" if provisional else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
