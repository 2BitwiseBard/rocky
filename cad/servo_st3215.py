"""ST3215 servo reference solid, v2 (D047, 2026-09-22) — measured from the real
servo's STEP model (cad/ref/STS3215_03a.step) instead of a guessed box.

Frame convention (unchanged): OUTPUT SHAFT AXIS = +Z at the origin, the case
hangs off -X, z = 0 is the case BOTTOM flat face. What changed vs v0.1:

  * case is 28.8 flat-to-flat (v0.1: 32) with a 1.5 mm raised RIM on both
    faces over the rear 29 mm; the four screw holes per face sit in that rim
    (this is how Feetech brackets and the SO-ARM100 hold the servo — D047's
    cups use them);
  * a 2.6 mm gearbox PLATEAU on the horn face (rel. x -35.2..-11.3, ±7.0);
  * the output axis is 10.2 from the front end (v0.1: 11.25);
  * horn: Ø9 flange, Ø20 × 2.5 disc, TOP at 33.1 (v0.1: 38.0), four holes on
    a 45° pattern (v0.1: 0/90/180/270) — the STEP is ambiguous about their
    radius, so every horn-bolted part uses a radial slot (horn_bcd_slot);
  * a REAR IDLER disc Ø19.9 × 2.1 on the bottom face (z -1.2..-3.3) — a
    two-sided joint is free;
  * connector housing + two 3-pin headers on the bottom face, the pins only
    3 mm behind the idler edge: plug_envelope() is the keep-out.

Helpers below are what every leg part uses; nothing else should hard-code a
servo dimension. Every number is "from STEP" and stays VERIFY until a real
servo has been measured (params.yaml servo_st3215).
"""
import math
from build123d import *
from common import params

_P = params()


def spec(p=None):
    return (p or _P)["servo_st3215"]


# ---- key planes (case-bottom frame) -------------------------------------
def z_levels(p=None):
    s = spec(p)
    z_top = s["body_h"]
    z_horn0 = z_top + s["out_boss_h"]
    z_horn1 = z_horn0 + s["horn_t"]
    return dict(
        bottom=0.0, top=z_top,
        rim_top=z_top + s["rim"]["h"], rim_bot=-s["rim"]["h"],
        plateau=z_top + s["plateau"]["h"],
        horn0=z_horn0, horn1=z_horn1, horn_head=z_horn1 + s["horn_center_head_h"],
        idler0=-s["idler_gap"], idler1=-(s["idler_gap"] + s["idler_t"]),
        idler_head=-(s["idler_gap"] + s["idler_t"] + s["idler_center_head_h"]),
        housing=-s["conn"]["housing_h"], pins=-s["conn"]["pins_h"],
        plug=-(s["conn"]["pins_h"] + s["conn"]["plug_h"]),
    )


def case_xspan(p=None):
    s = spec(p)
    return s["shaft_offset"] - s["body_l"], s["shaft_offset"]      # (-35.2, +10.2)


def horn_screw_positions(p=None):
    """XY of the four horn (and idler) screw holes at the NOMINAL 14 BCD, 45° pattern."""
    s = spec(p)
    r = s["horn_bcd"] / 2
    a0 = math.radians(s["horn_pattern_deg"])
    return [(r * math.cos(a0 + k * math.pi / 2), r * math.sin(a0 + k * math.pi / 2))
            for k in range(4)]


def horn_screw_angles(p=None):
    s = spec(p)
    return [s["horn_pattern_deg"] + 90 * k for k in range(4)]


def case_screw_positions(face="top", p=None):
    """XY of the rim screw holes on the horn face ('top') or idler face ('bot')."""
    s = spec(p)["case_screw"]
    xs = s["top_x"] if face == "top" else s["bot_x"]
    return [(x, sy * s["y"]) for x in xs for sy in (1, -1)]


# ---- solids --------------------------------------------------------------
def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def horn_slot_cutter(z0, z1, extra=0.0, p=None):
    """The four radial SLOTS a horn-bolted part carries: clearance width, from
    r = bcd/2 to r = bcd/2 + horn_bcd_slot, spanning z0..z1. extra widens
    (for a head counterbore use head_d)."""
    s = spec(p)
    w = s["horn_screw_clear_d"] + extra
    r0 = s["horn_bcd"] / 2
    L = s["horn_bcd_slot"]
    cut = None
    for ang in horn_screw_angles(p):
        one = Rot(0, 0, ang) * (Pos(r0, 0, (z0 + z1) / 2) * Cylinder(w / 2, z1 - z0)
                                + Pos(r0 + L, 0, (z0 + z1) / 2) * Cylinder(w / 2, z1 - z0)
                                + Pos(r0 + L / 2, 0, (z0 + z1) / 2) * Box(L, w, z1 - z0))
        cut = one if cut is None else cut + one
    return cut


def servo_body(p=None, clearance=0.0, detail=True):
    """The servo as one solid. clearance > 0 inflates it (pocket subtraction);
    detail=False drops rims/plateau/connectors (coarse hull for sweeps)."""
    s = spec(p)
    z = z_levels(p)
    c = clearance
    x0, x1 = case_xspan(p)
    w = s["body_w"] / 2
    body = _box(x0 - c, x1 + c, -w - c, w + c, -c, z["top"] + c)
    body += Pos(0, 0, (z["top"] + z["horn0"]) / 2) * Cylinder(s["out_boss_d"] / 2 + c, z["horn0"] - z["top"])
    body += Pos(0, 0, (z["horn0"] + z["horn1"]) / 2) * Cylinder(s["horn_d"] / 2 + c, s["horn_t"])
    body += Pos(0, 0, (z["horn1"] + z["horn_head"]) / 2) * \
        Cylinder(s["horn_center_head_d"] / 2 + c, s["horn_center_head_h"])
    body += Pos(0, 0, z["idler0"] / 2) * Cylinder(s["idler_boss_d"] / 2 + c, -z["idler0"])   # rear axle boss
    body += Pos(0, 0, (z["idler0"] + z["idler1"]) / 2) * Cylinder(s["idler_d"] / 2 + c, s["idler_t"])
    body += Pos(0, 0, (z["idler1"] + z["idler_head"]) / 2) * \
        Cylinder(s["idler_center_head_d"] / 2 + c, s["idler_center_head_h"])
    if detail:
        r = s["rim"]
        body += _box(r["x0"] - c, r["x1"] + c, -w - c, w + c, z["top"], z["rim_top"] + c)
        body += _box(r["x0"] - c, r["x1"] + c, -w - c, w + c, z["rim_bot"] - c, 0.0)
        pl = s["plateau"]
        body += _box(pl["x0"] - c, pl["x1"] + c, -pl["half_w"] - c, pl["half_w"] + c,
                     z["top"], z["plateau"] + c)
        cn = s["conn"]
        body += _box(cn["housing_x"][0] - c, cn["housing_x"][1] + c, -cn["housing_half_w"] - c,
                     cn["housing_half_w"] + c, z["housing"] - c, 0.0)
        for sy in (1, -1):
            ya, yb = sorted((sy * cn["pins_y"][0], sy * cn["pins_y"][1]))
            body += _box(cn["pins_x"][0] - c, cn["pins_x"][1] + c, ya - c, yb + c, z["pins"] - c, 0.0)
    return body


def plug_envelope(p=None):
    """Keep-out for the two mated cable plugs + their wire bend: a block over
    the pin headers standing plug_h off the pin tips. Any rigid part inside it
    means the servo cannot be plugged in."""
    s = spec(p)
    cn = s["conn"]
    z = z_levels(p)
    pad = cn["plug_pad"]
    return _box(cn["pins_x"][0] - pad, cn["pins_x"][1] + pad,
                -cn["pins_y"][1] - pad, cn["pins_y"][1] + pad, z["plug"], 0.0)


def horn_top_z(p=None):
    return z_levels(p)["horn1"]


if __name__ == "__main__":
    import sys, os
    from common import export
    b = servo_body()
    export(b, "servo_st3215_dummy")
    z = z_levels()
    print("z levels:", {k: round(v, 2) for k, v in z.items()})
    print("horn screws (nominal):", [(round(x, 2), round(y, 2)) for x, y in horn_screw_positions()])
    print("rim screws top:", case_screw_positions("top"), "bot:", case_screw_positions("bot"))
    fails = []
    ref_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ref", "STS3215_03a.step")
    if os.path.exists(ref_path):
        ref = import_step(ref_path)
        # STEP frame: axis at x = 12.5 of a case centred at 0, z = 0 at the case mid-plane
        ref = Pos(-12.5, 0, spec()["body_h"] / 2) * ref
        # compare a hair-inflated model so no face is coincident with the STEP's
        # (coincident faces make OCC booleans return garbage)
        bc = servo_body(clearance=0.02)
        common = bc & ref
        vc = 0.0 if common is None else common.volume
        ve, vm = bc.volume - vc, ref.volume - vc
        missing = ref - bc
        print(f"model vs STEP: model {b.volume/1000:.1f} cm^3, STEP {ref.volume/1000:.1f} cm^3, "
              f"model-outside-STEP {ve:.0f} mm^3, STEP-outside-model {vm:.0f} mm^3")
        # the model is a pocket-subtraction ENVELOPE: it may be fatter (chamfers,
        # hole fill, slot hedge) but no real feature may poke out of it. Report
        # the biggest missing blobs so a regression is visible.
        if missing is not None:
            for sol in sorted(missing.solids(), key=lambda q: -q.volume)[:4]:
                bb = sol.bounding_box()
                print(f"   STEP outside model: {sol.volume:6.1f} mm^3 at x {bb.min.X:.1f}..{bb.max.X:.1f} "
                      f"y {bb.min.Y:.1f}..{bb.max.Y:.1f} z {bb.min.Z:.1f}..{bb.max.Z:.1f}")
        if vm > 150: fails.append("STEP features outside the model")
        if ve > 3000: fails.append("model much fatter than the STEP")
    print(f"servo_st3215 checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails: sys.exit(1)
