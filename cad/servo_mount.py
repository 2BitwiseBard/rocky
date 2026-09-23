"""Servo retention, v2 (D047): the CUP — one shape holds all three servos.

v0.1 held each servo with zip ties; D046 replaced that with printed lips and
straps because the case's screw holes were unmeasured. The STEP settles it:
the ST3215 has a 1.5 mm raised rim on both faces over the rear 29 mm of the
case with two screw holes per face in it (the SO-ARM100 / Feetech bracket
mount). The cup is a rectangular sleeve around that rear part of the case:

  * back wall + two side walls (the case's ±y faces) take torque reaction;
  * a HORN-face strip (two rails beside the gearbox plateau) and an IDLER-
    face strip (rear bar + two rails beside the connector housing), each
    with the two rear rim-screw holes — the servo slides in from the front,
    four self-tappers hold it, nothing else is needed;
  * the whole cable side (housing, pins, plugs) stays open.

Built in the SERVO-LOCAL frame (servo_st3215) so every cradle poses it with
the same transform as its servo (leg_frame.YAW_TF / HIP_TF / KNEE_TF).
"""
from build123d import *
from common import params
from servo_st3215 import spec, z_levels, case_xspan, case_screw_positions

P = params()
S = spec(P)
Z = z_levels(P)
PR = P["print"]
FIT = PR["clearance_fit"]
T = 3.0                      # wall thickness
FRONT_X = -14.0              # cup front edge (rel. axis): > r 13 of any hub/plate riding the discs
PLATEAU_CLEAR = 0.6          # rails start this far outside the plateau half-width (+FIT)
HOUSING_CLEAR = 0.65         # idler-face rails start this far outside the housing half-width (+FIT)


def _box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cup_extents():
    x0, _ = case_xspan(P)
    w = S["body_w"] / 2
    return dict(
        x_back_out=x0 - FIT - T, x_back_in=x0 - FIT, x_front=FRONT_X,
        y_in=w + FIT, y_out=w + FIT + T,
        z_lo_in=Z["rim_bot"] - FIT, z_lo_out=Z["rim_bot"] - FIT - T,
        z_hi_in=Z["rim_top"] + FIT, z_hi_out=Z["rim_top"] + FIT + T,
        y_top_rail=S["plateau"]["half_w"] + FIT + PLATEAU_CLEAR,          # 7.9
        y_bot_rail=S["conn"]["housing_half_w"] + FIT + HOUSING_CLEAR,     # 10.1
        x_bot_bar=S["conn"]["housing_x"][0] - FIT - 0.15,                 # -30.3
    )


def servo_cup(front_x=None, back_half_w=None):
    """The cup in servo-local coords. back_half_w limits the back wall's
    width (the coxa base needs the I1 thumbscrew columns clear)."""
    e = cup_extents()
    xf = FRONT_X if front_x is None else front_x
    xb0, xb1 = e["x_back_out"], e["x_back_in"]
    yi, yo = e["y_in"], e["y_out"]
    zli, zlo, zhi, zho = e["z_lo_in"], e["z_lo_out"], e["z_hi_in"], e["z_hi_out"]
    bw = yo if back_half_w is None else back_half_w
    cup = _box(xb0, xb1, -bw, bw, zlo, zho)                             # back wall
    for sy in (1, -1):
        cup += _box(xb0, xf, min(sy * yi, sy * yo), max(sy * yi, sy * yo), zlo, zho)   # side walls
        yr = e["y_top_rail"]
        cup += _box(xb0, xf, min(sy * yr, sy * yo), max(sy * yr, sy * yo), zhi, zho)   # horn-face rails
        yr = e["y_bot_rail"]
        cup += _box(xb0, xf, min(sy * yr, sy * yo), max(sy * yr, sy * yo), zlo, zli)   # idler-face rails
    cup += _box(xb0, e["x_bot_bar"], -yo, yo, zlo, zli)                  # idler-face rear bar
    cs = S["case_screw"]
    for (x, y), (z0, z1) in [(pq, (zhi - 1, zho + 1)) for pq in case_screw_positions("top", P) if pq[0] < xf] + \
                            [(pq, (zlo - 1, zli + 1)) for pq in case_screw_positions("bot", P) if pq[0] < xf]:
        cup -= Pos(x, y, (z0 + z1) / 2) * Cylinder(cs["clear_d"] / 2, z1 - z0)
    return cup


def cup_screw_columns(face, length=25.0, d=4.4):
    """Driver columns for the rim screws, standing OFF the cup's outer face
    (top face: +z; bottom face: -z). For access checks against neighbours."""
    e = cup_extents()
    cols = []
    for x, y in case_screw_positions("top" if face == "top" else "bot", P):
        if x >= FRONT_X:
            continue
        if face == "top":
            cols.append(Pos(x, y, e["z_hi_out"] + length / 2) * Cylinder(d / 2, length))
        else:
            cols.append(Pos(x, y, e["z_lo_out"] - length / 2) * Cylinder(d / 2, length))
    return cols


if __name__ == "__main__":
    import sys
    from common import export
    from servo_st3215 import servo_body, plug_envelope
    v = lambda x: 0.0 if x is None else x.volume
    cup = servo_cup()
    export(cup, "servo_cup")
    fails = []
    servo = servo_body(P)
    def check(name, val, good, ok="OK", bad="FAIL"):
        okk = good(val)
        print(f"  {name}: {val:.2f} mm^3 ({ok if okk else bad})")
        if not okk: fails.append(name)
    check("cup x servo at rest", v(cup & servo), lambda q: q < 1, "CLEAR", "CLASH")
    check("cup x plug keep-out", v(cup & plug_envelope(P)), lambda q: q < 1, "CLEAR", "BLOCKS THE PLUGS")
    for name, d in (("+y", (0, 2, 0)), ("-y", (0, -2, 0)), ("+z (horn face)", (0, 0, 2)),
                    ("-z (idler face)", (0, 0, -2)), ("-x (rear)", (-2, 0, 0))):
        check(f"servo nudged {name} meets the cup", v((Pos(*d) * servo) & cup), lambda q: q > 1, "HELD", "FREE")
    worst = max(v((Pos(dx, 0, 0) * servo) & cup) for dx in (2, 5, 10, 20, 30))
    check("servo slide-in path (+x, the bolted direction)", worst, lambda q: q < 1, "OPEN", "BLOCKED")
    e = cup_extents()
    print(f"  cup: x {e['x_back_out']:.1f}..{e['x_front']:.1f}, y ±{e['y_out']:.1f}, z {e['z_lo_out']:.1f}..{e['z_hi_out']:.1f}")
    print(f"servo_mount checks: {'ALL CLEAN' if not fails else 'FAILED: ' + ', '.join(fails)}")
    if fails: sys.exit(1)
