"""Run INSIDE FreeCAD (the console binary with its GUI on Qt's `minimal`
platform): one TechDraw A4 sheet for one STEP file (D061). Driven by
cad/gen_drawings.py; not a CAD-check module.

Env: TD_STEP (input), TD_OUT (output path without extension: writes .pdf +
.json), TD_TITLE, TD_SUBTITLE, TD_DATE, TD_WEIGHT.

Sheet: ISO A4 landscape frame + title block, first-angle Front / Top / Right
views + an isometric, overall extents dimensioned on the Front and Right
views, and a hole table read from the solid (concave cylindrical faces
that close a full circle, grouped by diameter and axis; a slot end is half a
circle and is not a hole, a round boss is convex and is not one either).

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

    # holes from the SOLID, not the views: a cylindrical face whose normal points at its own
    # axis is a hole (a boss's points away); faces sharing an axis + diameter are one hole,
    # and only a hole whose faces close the circle counts (a slot end is half of one)
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
        groups[key] = groups.get(key, 0.0) + abs(u1 - u0)
    holes = {}
    for key, ang in groups.items():
        if ang >= 2 * math.pi - 1e-3:
            ax = "XYZ"[max(range(3), key=lambda k: abs(key[k]))]
            holes[(key[6], ax)] = holes.get((key[6], ax), 0) + 1
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
    note.X, note.Y = 60, 40

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
               "template_fields": sorted(te)}
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
