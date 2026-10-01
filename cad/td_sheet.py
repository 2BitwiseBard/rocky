"""Run INSIDE FreeCAD (the console binary with its GUI on Qt's `minimal`
platform): one TechDraw A4 sheet for one STEP file (D061). Driven by
cad/gen_drawings.py; not a CAD-check module.

Env: TD_STEP (input), TD_OUT (output path without extension: writes .pdf +
.json), TD_TITLE, TD_SUBTITLE, TD_DATE, TD_WEIGHT.

Sheet: ISO A4 landscape frame + title block, first-angle Front / Top / Right
views + an isometric, overall extents dimensioned on the Front and Right
views, and a hole table read from the solid (concave cylindrical faces
that close a full circle, grouped by diameter and axis; a round boss is
convex and is not one). Beside it, when the part has any, a table of open
channels (B105): a concave cylinder that is exactly half a circle is either
a slot end or a pocket open to an edge, and the two cannot be told apart by
the arc. A slot end has a partner facing it (the slot's other end: same Ø,
parallel axis, offset toward its open side and open back toward it); a
channel has none, and is listed with its Ø and depth along its axis.

FreeCAD 1.1 traps, all hit while building this: the page exporters exist
only in TechDrawGui (so the GUI must be up; `offscreen` segfaults, `minimal`
without `enable_fonts` draws text as boxes); in GUI mode hidden-line removal
runs in worker threads, so views have no edges until the event loop has
turned; a timed processEvents loop after the page is shown never returns;
print() goes to the hidden report view; an uncaught error leaves the process
running forever, hence os._exit on every path.
"""
import json
import math
import os
import sys
import time
import traceback

import FreeCAD as App
import FreeCADGui as Gui

Gui.showMainWindow()
sys.stdout = sys.stderr

TPL = "/app/share/Mod/TechDraw/Templates/ISO/A4_Landscape_TD.svg"
SCALES = (5, 2, 1.5, 1, 0.75, 0.5, 0.25, 0.2, 0.1)


def features(shape):
    """(holes, channels) read from the SOLID, not the views. holes: {(Ø, axis): count};
    channels: {(Ø, depth, axis): count}.

    A cylindrical face whose normal points at its own axis is concave (a boss's points
    away); concave faces sharing an axis + diameter are one group. A group that closes
    the circle is a hole. A group that is exactly half a circle is a slot end when a
    partner faces it (same Ø, parallel axis, its axis offset toward this one's open
    side, and its own open side back toward this one) and an open channel when none
    does (B105: the coxa fork's Ø6.4 / Ø7 / Ø20.3 pockets, open to its mouth, are the
    same 180 deg as the ends of its Ø3.4 / Ø5.8 slots). Other partial arcs (fillets)
    are neither."""
    import Part
    groups = {}
    for f in shape.Faces:
        srf = f.Surface
        if not isinstance(srf, Part.Cylinder):
            continue
        u0, u1, v0, v1 = f.ParameterRange
        p = f.valueAt((u0 + u1) / 2, (v0 + v1) / 2)
        n = f.normalAt((u0 + u1) / 2, (v0 + v1) / 2)
        a = srf.Axis.normalize()
        radial = (p - srf.Center) - a * (p - srf.Center).dot(a)
        if n.dot(radial) >= 0:
            continue                                    # convex: a boss or an outer round
        a = a if (a.x, a.y, a.z) > (0, 0, 0) else -a  # one sign per axis direction
        c = srf.Center - a * srf.Center.dot(a)          # the axis's point nearest the origin
        key = (round(a.x, 3), round(a.y, 3), round(a.z, 3),
               round(c.x, 1), round(c.y, 1), round(c.z, 1), round(2 * srf.Radius, 2))
        g = groups.setdefault(key, {"ang": 0.0, "mid": App.Vector(), "lo": 1e9, "hi": -1e9,
                                    "a": a, "c": c})
        g["ang"] += abs(u1 - u0)
        for k in range(16):                             # the arc's mean radial direction
            q = f.valueAt(u0 + (u1 - u0) * (k + 0.5) / 16, (v0 + v1) / 2)
            r = (q - c) - a * (q - c).dot(a)
            g["mid"] += r.normalize() * (abs(u1 - u0) / 16)
        for v in (v0, v1):                              # its extent along the axis
            t = (f.valueAt((u0 + u1) / 2, v) - c).dot(a)
            g["lo"], g["hi"] = min(g["lo"], t), max(g["hi"], t)
    holes, halves = {}, []
    for key, g in groups.items():
        ax = "XYZ"[max(range(3), key=lambda k: abs(key[k]))]
        if g["ang"] >= 2 * math.pi - 1e-3:
            holes[(key[6], ax)] = holes.get((key[6], ax), 0) + 1
        elif abs(g["ang"] - math.pi) < 1e-3:
            halves.append((key, g, ax))
    channels = {}
    for key, g, ax in halves:
        side = -g["mid"].normalize()                    # the open side
        partnered = False
        for key2, g2, _ in halves:
            if key2 is key or key2[:3] != key[:3] or abs(key2[6] - key[6]) > 0.011:
                continue
            d = g2["c"] - g["c"]
            d = d - g["a"] * d.dot(g["a"])
            if d.Length < 1e-3:
                continue
            d.normalize()
            if d.dot(side) > 0.99 and d.dot(g2["mid"].normalize()) > 0.99:
                partnered = True
                break
        if not partnered:
            k = (key[6], round(g["hi"] - g["lo"], 2), ax)
            channels[k] = channels.get(k, 0) + 1
    return holes, channels


def main():
    import Part
    import TechDraw
    import TechDrawGui
    from fractions import Fraction
    from PySide import QtWidgets

    step, out = os.environ["TD_STEP"], os.environ["TD_OUT"]
    tpl_path = TPL if os.path.exists(TPL) else os.path.join(
        App.getResourceDir(), "Mod", "TechDraw", "Templates", "ISO", "A4_Landscape_TD.svg")

    def settle(views, timeout=60):
        t = time.time()
        while time.time() - t < timeout:
            QtWidgets.QApplication.processEvents()
            if all(v.getVisibleEdges() for v in views):
                return
            time.sleep(0.05)
        raise RuntimeError("hidden-line removal never finished")

    doc = App.newDocument("sheet")
    shape = Part.read(step)
    part = doc.addObject("Part::Feature", "Part")
    part.Shape = shape
    bb = shape.BoundBox
    W, D, H = bb.XLength, bb.YLength, bb.ZLength
    # A4 landscape, first angle: Right view LEFT of Front, Top view BELOW it; the title
    # block owns x > 150, y < 62. Fit Right + gap + Front across x 25..200 and
    # Front + gap + Top down y 195..65, then pin the Top view's bottom at y 67.
    s = next((c for c in SCALES if (D + H) * c <= 88 and D * c + 25 + W * c <= 172), SCALES[-1])
    GAP = 25

    page = doc.addObject("TechDraw::DrawPage", "Page")
    tpl = doc.addObject("TechDraw::DrawSVGTemplate", "Template")
    tpl.Template = tpl_path
    page.Template = tpl
    pg = doc.addObject("TechDraw::DrawProjGroup", "Views")
    page.addView(pg)
    pg.Source = [part]
    pg.ProjectionType = "First angle"
    pg.ScaleType = "Custom"
    pg.Scale = s
    for p in ("Front", "Top", "Right"):
        pg.addProjection(p)
    pg.Anchor.Direction = App.Vector(0, -1, 0)
    pg.Anchor.XDirection = App.Vector(1, 0, 0)
    pg.AutoDistribute = True
    pg.spacingX = pg.spacingY = GAP
    pg.X = 25 + D * s + GAP + W * s / 2
    pg.Y = 67 + D * s + GAP + H * s / 2
    iso = doc.addObject("TechDraw::DrawViewPart", "Iso")
    page.addView(iso)
    iso.Source = [part]
    iso.Direction = App.Vector(1, -1, 1)
    iso.XDirection = App.Vector(1, 1, 0)
    iso.ScaleType = "Custom"
    iso.Scale = s * 0.6
    iso.X, iso.Y = 245, 150
    doc.recompute()
    settle(list(pg.Views) + [iso])
    views = {v.Type: v for v in pg.Views}

    dims = []
    for view, direction, off in ((views["Front"], 0, H * s / 2 + 8), (views["Front"], 1, W * s / 2 + 8),
                                 (views["Right"], 0, H * s / 2 + 8)):
        d = TechDraw.makeExtentDim(view, [], direction)
        d.X, d.Y = (0, off) if direction == 0 else (off, 0)
        dims.append(d)

    holes, channels = features(shape)
    lines = ["HOLES (from the solid)"]
    table = []
    for (dia, ax), n in sorted(holes.items()):
        lines.append(f"  \u00d8{dia:g}  x{n}   along {ax}")
        table.append({"dia_mm": dia, "count": n, "axis": ax})
    if not table:
        lines.append("  none")
    note = doc.addObject("TechDraw::DrawViewAnnotation", "Holes")
    page.addView(note)
    note.Text = lines
    note.TextSize = 3.5
    note.X, note.Y = 50, 40
    chan = []
    if channels:                                        # B105: half circles open to an edge
        lines = ["OPEN CHANNELS"]
        for (dia, depth, ax), n in sorted(channels.items()):
            lines.append(f"  \u00d8{dia:g} x {depth:g} deep  x{n}   along {ax}")
            chan.append({"dia_mm": dia, "depth_mm": depth, "count": n, "axis": ax})
        cnote = doc.addObject("TechDraw::DrawViewAnnotation", "Channels")
        page.addView(cnote)
        cnote.Text = lines
        cnote.TextSize = 3.5
        cnote.X, cnote.Y = 108, 40          # clear of the holes and the title block

    te = tpl.EditableTexts
    fields = {"FC-Title": os.environ.get("TD_TITLE", "part"),
              "Subtitle": os.environ.get("TD_SUBTITLE", ""),
              "Drawing_number": os.environ.get("TD_TITLE", "part"),
              "FC-SC": "{0.numerator}:{0.denominator}".format(Fraction(s).limit_denominator(20)),
              "FC-SH": "1 / 1", "Designed_by_Name": "cad/gen_drawings.py",
              "FC-Date": os.environ.get("TD_DATE", ""), "Weight": os.environ.get("TD_WEIGHT", "")}
    for k, v in fields.items():
        if k in te:
            te[k] = v
    tpl.EditableTexts = te
    doc.recompute()
    summary = {"scale": s, "bbox_mm": [round(W, 2), round(D, 2), round(H, 2)],
               "extents_mm": [round(d.getRawValue(), 2) for d in dims], "holes": table,
               "channels": chan, "template_fields": sorted(te)}
    page.ViewObject.show()
    doc.recompute()
    for _ in range(20):
        QtWidgets.QApplication.processEvents()          # plain turns only: a timed loop here hangs
    TechDrawGui.exportPageAsPdf(page, out + ".pdf")
    with open(out + ".json", "w") as f:
        json.dump(summary, f, indent=1)


try:
    main()
    sys.stderr.flush()
    os._exit(0)
except Exception:
    traceback.print_exc()
    sys.stderr.flush()
    os._exit(1)
