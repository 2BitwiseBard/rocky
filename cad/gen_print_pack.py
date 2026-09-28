"""Generate the Print-Prep Pack (cad/out/PRINT_PREP_PACK.pdf): the one document
to take to the printer.

    MPLBACKEND=Agg python3 gen_print_pack.py        (from cad/)

What is in it, in order:
  1. a cover: params revision, printer bed, filament settings, the batches
     (parts / grams / hours from out/print_estimate.json) and the strength
     verdicts (out/fem/fem_results.json);
  2. the print order: docs/PRINT_PLAN.md's batches as a checklist, with the
     batch 0 GO / NO-GO table and the batch 1 fit criteria;
  3. one section per batch in print order. Each section opens with a tick
     list of its parts; each part then gets a sheet: a three-view in its PRINT
     pose (check_printability.ORIENT, the pose the audit slices), material,
     pose, supports, walls / infill, the estimate, the printability audit's
     numbers, the notes, and for load-bearing leg parts the FEM verdict with
     its stress picture. A part printed again in a later batch is a line in
     that batch's list pointing back at its sheet;
  4. where cad/out/drawings/<part>.pdf exists (gen_drawings.py, FreeCAD
     TechDraw, D061), that A4 sheet follows its part's sheet, scaled onto a
     landscape letter page. A leg part without one says how to make it.

The PARTS table below is the print plan's per-part data (docs/PRINT_PLAN.md
is the authority: change it there first, then here). Sizes, overhang widths
and grams are NOT typed in here: they come from the STLs, printability.json
and print_estimate.json, so a geometry change cannot leave a stale number.

Deterministic, because the PDF is tracked (git-lfs) and `cad-check --derived`
must leave an unchanged tree clean: the views are Agg PNGs; reportlab runs in
invariant mode (no timestamp, fixed /ID); the drawing merge with pypdf writes
no dates, and its /ID is pypdf's checksum of the written structure, so equal
inputs give equal bytes. Reportlab cannot place a PDF page, hence pypdf: each
drawing lands on a landscape placeholder page reportlab left for it, so the
page numbers printed in the footer and the contents are the final ones.
"""
import functools
import io
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                            # noqa: E402
import numpy as np                                         # noqa: E402
import yaml                                                # noqa: E402
from matplotlib.collections import PolyCollection          # noqa: E402
from pypdf import PdfReader, PdfWriter, Transformation     # noqa: E402
from reportlab.lib import colors                           # noqa: E402
from reportlab.lib.pagesizes import landscape, letter      # noqa: E402
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet  # noqa: E402
from reportlab.lib.units import inch                       # noqa: E402
from reportlab.platypus import (BaseDocTemplate, CondPageBreak, Flowable, Frame,  # noqa: E402
                                Image, KeepTogether, NextPageTemplate, PageBreak,
                                PageTemplate, Paragraph, Spacer, Table, TableStyle)
from stl import mesh as stlmesh                            # noqa: E402

from check_printability import ORIENT                      # noqa: E402  the audit's print poses

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
DRAW = os.path.join(OUT, "drawings")
FEM = os.path.join(OUT, "fem")
PDF = os.path.join(OUT, "PRINT_PREP_PACK.pdf")
LEG_BATCH = "Batch 2 one leg"        # the parts gen_drawings.py draws (it reads the same key)

# ---------------------------------------------------------------------------
# Per-part print data (docs/PRINT_PLAN.md). material, pose, supports,
# walls / infill, notes. "(default)" marks walls the plan leaves to the slicer.
PLA_DEFAULT = "3 walls / 25% (default)"
PARTS = {
    # batch 0
    "fit_ladder": ("PLA", "flat, as exported", "none", "3 walls / 25%, 0.2 mm",
                   "B28, blocking. Elephant-foot compensation ~0.15 mm, sliced like the parts. "
                   "Rows A-E: see the GO / NO-GO table on the print-order page. Numbers go to "
                   "NOTES_INBOX.md, then params.yaml (print:), then regenerate (D032)."),
    # batch 1: D047 joint coupons + the blank
    "servo_blank": ("PLA", "bottom DOWN (rims on the bed, horn up)", "none", "2 walls / 15%",
                    "D047 v0.3 ST3215 stand-in, measured from the STEP. Glue a blank_idler into "
                    "the Ø6.2 pocket (CA, A-14)."),
    "blank_idler": ("PLA", "disc DOWN (stub up)", "none", "3 walls / 30%",
                    "Glues into servo_blank. The idler's centre screw head is omitted on purpose."),
    "coupon_cup": ("PLA", "back wall DOWN", "none", "3 walls / 30%",
                   "COUPON 1: the cup. Slide the blank in rear-first, drive 4 self-tappers into "
                   "the rim holes. The same cup holds all three servos."),
    "coupon_yaw_hub": ("PLA", "hub DOWN", "none", "4 walls / 30%",
                       "COUPON 2: the fork's yaw hub (Ø20.3 pocket on the horn, four slotted "
                       "holes, counterbores from below). On a real servo it answers M2-or-M3 and "
                       "the hole radius (B23)."),
    "coupon_hip_hub": ("PLA", "outer face DOWN", "none", "4 walls / 30%",
                       "COUPON 3: femur plate A's hub with horn_coupler: lobes into the recess, "
                       "2x M3 x 8 clamps from the outer face."),
    "coupon_idler": ("PLA", "outer face DOWN", "none", "3 walls / 30%",
                     "COUPON 4: plate B's tower. The pocket rides the idler (a blank_idler or the "
                     "real one); the notch faces the plugs."),
    "horn_coupler": ("PETG", "disc DOWN", "none", "4 walls / 40%",
                     "One per hip / knee horn. Slotted holes: M3 x 6 into the horn (A-09; M2 x 6 + "
                     "washer if the horn is M2, A-12)."),
    # interface coupons (with batch 1)
    "latch_housing": ("PLA", "as exported", "none", PLA_DEFAULT,
                      "I3 latch, with latch_rotor. As modelled the two halves cannot be joined "
                      "(the rotor's pegs have no way into the housing's track) and the rotor does "
                      "not reach the deck strike (B87): print only to look at the problem, fit none."),
    "latch_rotor": ("PLA", "as exported", "none", PLA_DEFAULT,
                    "Meant for latch_housing (I3); as modelled it cannot go in (B87)."),
    "thumb_knob_m3": ("PLA", "as exported", "none", PLA_DEFAULT,
                      "An M3 x 16 hex head (A-19) presses into the 5.6 A/F hex pocket and must not "
                      "turn (note if it needs persuasion); 0.5 mm of the head stands proud. A round "
                      "socket head spins in it. The same knob is the I1 port's thumbscrew: 10 for "
                      "the robot, 2 here (the port coupon's, then the bench leg's)."),
    "dovetail_male_coupon": ("PLA", "as exported", "none", PLA_DEFAULT,
                             "I6, with dovetail_shoe: the shoe slides down the 24 mm spec segment "
                             "(the knob lock waits for B83: its bore misses the dovetail)."),
    "dovetail_shoe": ("PLA", "as exported", "none", PLA_DEFAULT,
                      "I6 shoe for dovetail_male_coupon."),
    "shell_sector_demo": ("PLA", "as exported (the two I6 segments on the bed)",
                          "YES, under the plate (it starts above the bed on the I6 segments)",
                          PLA_DEFAULT,
                          "I3 panel standard, the panel half: lip, latch pocket, magnet pockets, "
                          "two I6 segments. Not a mating pair with frame_coupon: latch/strike, "
                          "lip/groove and magnets sit at different positions (B87). Feel the lip "
                          "seat and the magnet pull separately; no latch test until B87."),
    "frame_coupon": ("PLA", "as exported", "none", PLA_DEFAULT,
                     "I3 panel standard, the frame half: groove, latch strike, magnets. Its second "
                     "peg slot is misplaced (B87)."),
    "tool_hook": ("PLA", "as exported", "YES (the audit finds supported islands)", "3 walls / 25%",
                  "I2 tool socket: on a printed tibia_sea_slider the tool inserts free and "
                  "quarter-turns, and a tug must not pull it off."),
    "tool_scoop": ("PLA", "as exported", "YES (the audit finds supported islands)", "3 walls / 25%",
                   "I2 tool socket, as tool_hook."),
    # batch 2: one leg
    "coxa_yaw_base": ("PETG", "plate DOWN (the I1 hook lip on the bed)",
                      "YES under the whole plate: the hook lip holds it off the bed. The harness "
                      "channel's roof bridges 11 mm",
                      "5 walls / 40%",
                      "D047: I1 plate + servo cup (yaw servo shaft-down), D059 harness channel on "
                      "-Y. 4 rim self-tappers hold the servo (2 from above, 2 from under the plate)."),
    "coxa_fork": ("PETG", "lower hub DOWN, upright as assembled",
                  "YES in three places: under the upper plate (z ~41), the hip cup's lower wall "
                  "(z ~44) and its upper side wall over the hip servo (z ~72.5); widths: the "
                  "audit line",
                  "5 walls / 40%",
                  "D047: C-fork rides the yaw horn (below) and idler (above). 4x M3 x 6 from below "
                  "the hub into the horn (A-09). As drawn it cannot go onto the servo: the horn's "
                  "centre head and the idler catch the C's lips (B80). Clear all three supports "
                  "before the hip servo goes in."),
    "femur_link": ("PETG (CF-PLA later)", "plate A outer face DOWN",
                   "none: recesses up, bridge walls vertical", "6 walls / 40%",
                   "D047: plate A + bridge walls. 2x M3 x 8 clamps per coupler (A-10); "
                   "4x M3 x 12 for plate B (A-18)."),
    "femur_plate_b": ("PETG", "outer face DOWN", "none: pockets up", "5 walls / 40%",
                      "Idler-side plate; D062: 8 mm rails + a deck over the bridge (B74). Pockets "
                      "ride the hip + knee idlers; the notch is the cable-plug side. Fastens to "
                      "the bridge bosses with 4x M3 x 12 (A-18)."),
    "tibia_knee_carrier": ("PETG", "cup floor DOWN", "YES under the tube boss", "5 walls / 40%",
                           "D047: knee servo cup + tube clamp. The pinch-bolt slit prints as-is; "
                           "M3 x 10 pinch bolt (A-11)."),
    "tibia_sea_outer": ("PETG", "tube socket DOWN", "none", "4 walls / 35%",
                        "Check the slider glides in the bore before gluing the tube. Switch "
                        "pocket: KW10-class."),
    "tibia_sea_slider": ("PETG", "flange DOWN", "none", "4 walls / 35%",
                         "Keys slide in the keyways with a light wiggle: sand if tight."),
    # batch 3: the body
    "body_deck": ("PETG or PLA", "flat", "none", "4 walls / 30% gyroid",
                  "The biggest part: brim ON, dry filament, watch the first-layer corners. Print "
                  "it after B27 (deck_t 4 vs 6 unified), B28, B51 and the bay (B84), and after the "
                  "port coupons dock cleanly: each of them changes holes in the deck. Today's deck "
                  "has holes only for the star-board bracket."),
    "port_coupon_deck": ("PLA", "as exported", "none", PLA_DEFAULT,
                         "I1, with port_coupon_plate: tilt 15°, lip through the slot, slide "
                         "inboard to hook, pivot flat onto the dowels. Binds: note which step."),
    "port_coupon_plate": ("PLA", "as exported", "none (the audit reports a cantilever: check the "
                          "slicer preview)", PLA_DEFAULT,
                          "The leg half of the I1 port coupon."),
    "busboard_bracket": ("PLA or PETG", "as exported", "none", PLA_DEFAULT,
                         "The star board: print once the electronics exist (WIRING_HARNESS.md)."),
    "avionics_tray": ("PLA or PETG", "as exported", "none", PLA_DEFAULT,
                      "A bench carrier for the electronics once its Pi standoffs are 58 x 49 "
                      "(B89). As drawn, tray + rails fit nowhere between the legs (B51)."),
    "tray_rail": ("PLA or PETG", "as exported", "none", PLA_DEFAULT,
                  "The avionics tray's rails. Waits for B51: as drawn, tray + rails fit nowhere "
                  "between the legs. Do not print it for the robot yet."),
    "battery_sled": ("PETG", "as exported", "none (check the preview for the audit's overhang)",
                     PLA_DEFAULT,
                     "Waits for the bay (B84): nothing carries the sled yet. Then size it from the "
                     "pack you bought (B72)."),
    "sled_rail": ("PETG", "as exported", "none", PLA_DEFAULT,
                  "The battery sled's floor rails (the sled rides on top). Waits for the bay "
                  "(B84)."),
    # deferred: bench jig + calibration gauges
    "jig_base": ("PLA", "flat", "none", "3 walls / 20%, 0.3 mm", "Bench jig base."),
    "jig_column": ("PLA", "on its back, spine down", "YES", "3 walls / 20%, 0.3 mm",
                   "Bench jig column (the biggest volume in the jig)."),
    "calib_gauge_hip": ("PLA", "standing, as exported", "none (both lying poses leave an island)",
                        PLA_DEFAULT, "Used by the bench runbook's calibration."),
    "calib_gauge_knee": ("PLA", "as exported", "YES under the step near the bed", PLA_DEFAULT,
                         "Used by the bench runbook's calibration."),
    # deferred: the stand
    "stand_base": ("PLA", "as exported", "none", "2 walls / 15%, 0.3 mm",
                   "With stand_crown: the 47 mm deck cradle."),
    "stand_section": ("PLA", "as exported", "none", "2 walls / 15%, 0.3 mm",
                      "Stacks on the printed I6 spigots, +80 mm each; two free the leg's full "
                      "reach below the deck."),
    "stand_crown": ("PLA", "as exported", "YES from the bed inside the skirt (they pull out of "
                    "the open bottom)", "2 walls / 15%, 0.3 mm",
                    "The top plate spans the hollow skirt."),
    # deferred: the carapace
    "shell_sector": ("PLA or PETG", "as exported", "YES on the outside overhangs; the cavity's "
                     "45° terraces need nothing", "3 walls / 12% gyroid, 0.25 mm, brim",
                     "Five of them + shell_cap. The latch cartridges wait for B87: until then the "
                     "sectors sit on the seam tongues and the foot magnets; do not carry the robot "
                     "by the shell."),
    "shell_cap": ("PLA or PETG", "as exported", "none", "3 walls / 12% gyroid, 0.25 mm, brim",
                  "The carapace's hatch cap. The hatch magnets have no seat yet (B88): leave them "
                  "out; the cap sits on its seat by gravity."),
    # the hand (a later tool, B25)
    "hand_hub": ("PLA first / PETG final", "top face DOWN (boss up, collar ring on the bed)",
                 "YES (tree/organic) under the collar windows + servo-pocket ceiling",
                 "4 walls / 30%", "Ream the pin holes 2.0 mm."),
    "hand_cam": ("CF-PLA or PETG", "flat", "none", "4 walls / 40%",
                 "Slot profile is a v0.1 placeholder: expect to reprint after bench tuning."),
    "hand_finger": ("PLA first / PETG final", "on its wedge side face",
                    "YES under the knuckle tab only", "3 walls / 25%",
                    "Three identical. Light part: fast iterations."),
    "foot_pad_tpu": ("TPU", "as exported", "none", "slicer TPU profile",
                     "A sock over the closed hand's cone tip: waits for the hand."),
}

# The batches in print order. key = the print_estimate.json batch (its parts
# and quantities are used as-is), extra = parts the estimate leaves out,
# material = a batch-wide override (batch 1 is all PLA).
BATCHES = [
    dict(key="Batch 0 fit ladder", title="Batch 0: measure the printer (B28, blocking)",
         intro="One fit ladder answers every clearance in params.print. Nothing else prints "
               "until it is measured and filed."),
    dict(key="Batch 1 coupons + blank", title="Batch 1: four joint coupons + one blank (PLA)",
         material="PLA",
         intro="Before any full leg part. Each coupon is a boolean clip of the production solid "
               "(part_leg_coupons.py), so a coupon that fits proves the part it came from. The "
               "fit criteria are on the print-order page."),
    dict(key=None, title="Interface coupons (PLA, with batch 1; not in the estimate)",
         extra=[("latch_housing", 1), ("latch_rotor", 1), ("thumb_knob_m3", 2),
                ("dovetail_male_coupon", 1), ("dovetail_shoe", 1), ("shell_sector_demo", 1),
                ("frame_coupon", 1), ("tool_hook", 1), ("tool_scoop", 1)],
         intro="The standards the body, panels and tools attach by (D020, INTERFACES.md). None "
               "changed at D047: a set printed earlier still counts."),
    dict(key="Batch 2 one leg", title="Batch 2: one leg, after batch 1 passes",
         intro="PETG for the structure, PLA for the three blanks. Assemble in the order below; "
               "check_assembly.py asserts the final fits, not step 1's path, which is blocked as "
               "drawn (B80)."),
    dict(key="Batch 3 body", title="Batch 3: the body",
         extra=[("port_coupon_deck", 1), ("port_coupon_plate", 1), ("thumb_knob_m3", 8)],
         intro="The deck after B27, B28, B51 and the bay (B84), and after the port coupons dock "
               "cleanly; the star board once the electronics exist; the avionics tray only as a "
               "bench carrier (B89). tray_rail waits for B51, battery_sled and sled_rail for the "
               "bay (B84), though the estimate still prices them. 8 more thumb_knob_m3, 2 per "
               "coxa_yaw_base. The port coupons and the 8 knobs are not in the estimate."),
    dict(key="Deferred: four more legs", title="Deferred: four more legs (servos in hand)",
         intro="4 x batch 2 without the blanks and without coxa_yaw_base (batch 3 prints those)."),
    dict(key="Deferred: bench jig", title="Deferred: bench jig + calibration gauges",
         extra=[("calib_gauge_hip", 1), ("calib_gauge_knee", 1)],
         intro="Wait until servos are in hand. 0.3 mm layers for the jig. The gauges are not in "
               "the estimate."),
    dict(key="Deferred: stand", title="Deferred: the stand (print last)",
         intro="0.3 mm, 2 walls, 15%. stand_base + stand_crown alone are the deck cradle; each "
               "stand_section raises it 80 mm."),
    dict(key="Deferred: carapace", title="Deferred: the carapace (print last)",
         intro="0.25 mm, 3 walls, 12% gyroid, brim."),
    dict(key=None, title="The hand (a later tool, B25; not in the estimate)",
         extra=[("hand_hub", 1), ("hand_cam", 1), ("hand_finger", 3), ("foot_pad_tpu", 1)],
         intro="Printed after the leg batches. The tools (hook, scoop) are already coupons."),
]

FEM_KEY = {"femur_link": "femur", "femur_plate_b": "femur"}   # fem_check tests the pair as one
FEM_CASES = {"V": "vertical foot load", "R+": "radial foot load, outward",
             "R-": "radial foot load, inward", "L+": "lateral foot load", "L-": "lateral foot load",
             "T+": "stall torque on the horn", "T-": "stall torque on the horn"}
VERDICT_COLOUR = {"PASS": "#2e7d32", "WARN": "#b26a00", "FAIL": "#c62828"}


def _load_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


# ---------------------------------------------------------------------------
# three-view in the print pose

# (title, horizontal axis, vertical axis, +1 when the viewer sits on the + side
# of the dropped axis): top from +Z, front from -Y, side from +X (first-angle
# right view), so the nearer faces are the lighter ones and are drawn last
VIEWS = [("Top (XY, looking down on the bed)", 0, 1, +1), ("Front (XZ)", 0, 2, -1),
         ("Side (YZ)", 1, 2, +1)]


@functools.lru_cache(maxsize=None)        # the page-number passes reuse each picture
def three_view(name):
    """views_<name>.png: painter-shaded orthographic views of the STL turned
    into its print pose (the ORIENT transform check_printability slices), bed
    at z = 0."""
    v = stlmesh.Mesh.from_file(os.path.join(OUT, f"{name}.stl")).vectors.astype(float)
    T = ORIENT.get(name)
    if T is not None:
        T = np.asarray(T, float)
        v = v @ T[:3, :3].T + T[:3, 3]
    v = v - v.reshape(-1, 3).min(0)
    hi = v.reshape(-1, 3).max(0)
    fig, axes = plt.subplots(1, 3, figsize=(9.6, 3.1), dpi=110)
    for ax, (label, ix, iy, side) in zip(axes, VIEWS):
        iz = 3 - ix - iy
        near = side * v[:, :, iz].mean(axis=1)          # larger = nearer the viewer
        order = np.argsort(near, kind="stable")
        d = near[order]
        dn = (d - d.min()) / (np.ptp(d) + 1e-9)
        cols = np.outer(0.35 + 0.5 * dn, np.array([0.55, 0.44, 0.79]))
        ax.add_collection(PolyCollection(v[order][:, :, [ix, iy]],
                                         facecolors=np.clip(cols, 0, 1), edgecolors="none"))
        ax.set_xlim(-5, hi[ix] + 5)
        ax.set_ylim(-5, hi[iy] + 5)
        ax.set_aspect("equal")
        ax.set_title(f"{label}   {hi[ix]:.1f} x {hi[iy]:.1f} mm", fontsize=8)
        ax.tick_params(labelsize=6)
        ax.grid(alpha=0.25, lw=0.4)
        if iy == 2:
            ax.axhline(0, color="#444", lw=1.2)          # the bed
    fig.suptitle(f"{name} as printed   -   {hi[0]:.1f} x {hi[1]:.1f} x {hi[2]:.1f} mm "
                 f"(X x Y x height)", fontsize=10, fontweight="bold")
    plt.tight_layout(rect=(0, 0, 1, 0.94))
    p = os.path.join(OUT, f"views_{name}.png")
    plt.savefig(p)
    plt.close(fig)
    return p


# ---------------------------------------------------------------------------
# styles + small flowables

_ss = getSampleStyleSheet()
H1 = ParagraphStyle("h1", parent=_ss["Title"], fontSize=22, leading=26, spaceAfter=4)
H2 = ParagraphStyle("h2", parent=_ss["Heading1"], fontSize=15, leading=18, spaceBefore=0,
                    spaceAfter=4, textColor=colors.HexColor("#3d2f6b"))
H3 = ParagraphStyle("h3", parent=_ss["Heading3"], fontSize=10.5, leading=13, spaceBefore=6,
                    spaceAfter=2)
BODY = ParagraphStyle("body", parent=_ss["BodyText"], fontSize=9, leading=11.5)
SMALL = ParagraphStyle("small", parent=BODY, fontSize=8.2, leading=10.2)
CELL = ParagraphStyle("cell", parent=BODY, fontSize=7.6, leading=9.2)
HEAD = ParagraphStyle("head", parent=CELL, textColor=colors.white, fontName="Helvetica-Bold")
PURPLE = colors.HexColor("#5b4a8a")
LIGHT = colors.HexColor("#f2eefa")
GRID = colors.HexColor("#c9bfe0")


def table(rows, widths, header=True, zebra=True):
    """rows of strings / flowables -> a styled Table (strings become wrapping Paragraphs)."""
    cells = [[c if not isinstance(c, (str, int, float)) else
              Paragraph(str(c), HEAD if (header and i == 0) else CELL) for c in r]
             for i, r in enumerate(rows)]
    t = Table(cells, colWidths=widths, repeatRows=1 if header else 0)
    style = [("GRID", (0, 0), (-1, -1), 0.4, GRID), ("VALIGN", (0, 0), (-1, -1), "TOP"),
             ("TOPPADDING", (0, 0), (-1, -1), 2), ("BOTTOMPADDING", (0, 0), (-1, -1), 2)]
    if header:
        style.append(("BACKGROUND", (0, 0), (-1, 0), PURPLE))
    if zebra:
        style.append(("ROWBACKGROUNDS", (0, 1 if header else 0), (-1, -1), [colors.white, LIGHT]))
    t.setStyle(TableStyle(style))
    return t


class Mark(Flowable):
    """Zero-size flowable that records the page it lands on (for the contents,
    the 'sheet on p. N' references and the drawing placeholders)."""

    def __init__(self, key, pages):
        super().__init__()
        self.key, self.pages = key, pages
        self.width = self.height = 0

    def draw(self):
        self.pages.setdefault(self.key, self.canv.getPageNumber())


def _footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("Helvetica", 7)
    canvas.setFillColor(colors.HexColor("#777777"))
    w, _ = canvas._pagesize
    canvas.drawString(0.6 * inch, 0.35 * inch, "Pebble print-prep pack  -  generated by "
                      "cad/gen_print_pack.py; docs/PRINT_PLAN.md is the authority")
    canvas.drawRightString(w - 0.6 * inch, 0.35 * inch, f"page {canvas.getPageNumber()}")
    canvas.restoreState()


# ---------------------------------------------------------------------------
# page content

def cover(meta, est, fem):
    s = [Paragraph("Pebble: Print-Prep Pack", H1),
         Paragraph(f"Project ROCKY &middot; params rev <b>{meta['rev']}</b> "
                   f"(params {meta['version']}) &middot; reference bed {meta['bed']} mm", BODY),
         Spacer(1, 6),
         Paragraph("What to print, in what order, and how. Every STL is in cad/out/. Slice each "
                   "part in the pose on its sheet: that is the pose check_printability.py audits. "
                   "Dimensions are CAD-nominal mm; servo-interface numbers stay VERIFY until the "
                   "caliper session. The robot fingerprint is not printed here (it needs the "
                   "MuJoCo model): see sim/model_fingerprint.py.", BODY),
         Spacer(1, 8), Paragraph("Filament starting points", H3),
         table([["material", "nozzle / bed", "fan", "notes"],
                ["PETG", "240 °C / 85 °C", "30-50%", "0.2 mm layers, brim on structural parts, "
                 "dry filament"],
                ["PLA", "210 °C / 60 °C", "full", "coupons, blanks and anything expected to "
                 "change (D011)"],
                ["CF-PLA", "225 °C / 60 °C", "full", "HARDENED NOZZLE"]],
               [0.9 * inch, 1.3 * inch, 0.8 * inch, 4.3 * inch]),
         Spacer(1, 8), Paragraph("Batches (out/print_estimate.json, 0.2 mm, expect ±30%)", H3)]
    rows = [["batch", "parts (pieces)", "grams", "hours"]]
    tg = th = 0.0
    for b in BATCHES:
        if b["key"] is None or b["key"] not in est:
            continue
        e = est[b["key"]]
        n = sum(p["qty"] for p in e["parts"])
        rows.append([b["key"], f"{len(e['parts'])} ({n})", f"{e['total_g']}", f"{e['total_h']}"])
        tg += e["total_g"]
        th += e["total_h"]
    rows.append(["<b>whole plan</b>", "", f"<b>{tg:.0f}</b>", f"<b>{th:.1f}</b>"])
    s.append(table(rows, [3.3 * inch, 1.6 * inch, 1.2 * inch, 1.2 * inch]))
    s.append(Paragraph("Not in the estimate: the interface coupons, the port coupons, the "
                       "calibration gauges and the hand set.", SMALL))
    s += [Spacer(1, 8), Paragraph("Strength (out/fem/fem_results.json, D061)", H3)]
    if fem:
        rows = [["part", "verdict", "SF", "governing case", "held / loaded"]]
        for r in fem.get("results", []):
            c = VERDICT_COLOUR.get(r["verdict"], "#000000")
            rows.append([r["part"], f"<font color='{c}'><b>{r['verdict']}</b></font>",
                         f"{r['sf']:.2f}", f"{r['governing']}: {FEM_CASES.get(r['governing'], '')}",
                         f"{r.get('held', '')} / {r.get('loaded', '')}"])
        s.append(table(rows, [1.2 * inch, 0.6 * inch, 0.45 * inch, 1.5 * inch, 3.55 * inch]))
        s.append(Paragraph(f"Material {fem.get('material', '?').upper()}; pass at SF >= 2, warn "
                           "at >= 1.5. Servo-limited loads: a servo cannot push harder than its "
                           "stall. The allowables are VERIFY until your own pull coupons say "
                           "otherwise (B75). 'femur' is femur_link + femur_plate_b as one "
                           "part. Details: out/fem/FEM_REPORT.md.", SMALL))
    else:
        s.append(Paragraph("No FEM results on disk: run rocky.sh cad-check --fem.", SMALL))
    return s


def contents(pages):
    rows = [["section", "page"]]
    rows.append(["Print order + checklist", str(pages.get("sec:order", ""))])
    for b in BATCHES:
        rows.append([b["title"], str(pages.get("sec:" + b["title"], ""))])
    return [Paragraph("Contents", H3), table(rows, [6.3 * inch, 1.0 * inch])]


def print_order(fem):
    femur = next((r for r in (fem or {}).get("results", []) if r["part"] == "femur"), None)
    s = [Paragraph("Print order + checklist", H2),
         Paragraph("From docs/PRINT_PLAN.md. Print leg parts only from the current tree: every leg "
                   "part from before D047 was modelled around the wrong servo envelope. Log every "
                   "result in NOTES_INBOX.md: what stuck, what warped, measured hole sizes.",
                   BODY), Spacer(1, 4)]
    steps = [
        "Calibrate the printer (PID, first layer, flow) on the filament you will use.",
        "<b>Batch 0</b>: the fit ladder (PLA, 0.2 mm, 3 walls, 25%). Measure rows A-E, file them "
        "in NOTES_INBOX.md, then params.yaml (print:), regenerate. GO / NO-GO below.",
        "<b>Batch 1</b>: servo_blank + blank_idler, the four D047 coupons (cup, yaw hub, hip hub, "
        "idler) + 1 horn_coupler, all PLA. Also the interface coupons. Check the fit criteria "
        "below BEFORE any full leg part.",
        "<b>Batch 2</b>: one leg in PETG (+ 3 PLA blanks) after batch 1 passes. Assemble in the "
        "order on the batch 2 page.",
        "<b>Batch 3</b>: the deck (after B27, B28, B51 and the bay B84, and once the port "
        "coupons dock), 4 more coxa_yaw_base + 8 thumb_knob_m3, the star board once the "
        "electronics exist. The tray rails and the battery parts wait (B51, B84).",
        "<b>Deferred</b>, servos in hand: four more legs, the bench jig + calibration gauges. "
        "<b>Print last</b>: the stand and the carapace. The hand is a later tool (B25).",
    ]
    s.append(table([["[ ]", f"{i}. {t}"] for i, t in enumerate(steps, 1)],
                   [0.35 * inch, 6.95 * inch], header=False))
    if femur is not None and femur["verdict"] != "PASS":
        c = VERDICT_COLOUR.get(femur["verdict"], "#000000")
        s += [Spacer(1, 4), Paragraph(
            f"<font color='{c}'><b>Strength: the femur {femur['verdict']}S at SF "
            f"{femur['sf']:.2f}</b></font> (case {femur['governing']}, B74). Print it for fit, "
            "expect it to twist, and do not run a leg hard on it until a re-run passes.", BODY)]
    s += [Paragraph("Batch 0 GO / NO-GO (D032: before params are regenerated, bores only)", H3),
          table([["measurement", "result", "action"],
                 ["Row A (Ø4 bores)", "4.30 slides, 4.20 binds", "the printer matches params: GO"],
                 ["", "only 4.40 / 4.50 slides", "holes print small: slicer XY hole compensation "
                  "+0.05...+0.10 mm, GO, file the numbers"],
                 ["", "4.15 / 4.20 already slides", "true to CAD or roomy: GO and note it"],
                 ["Row D (slots)", "the lip passes 4.2", "GO"],
                 ["", "only 4.6 passes", "file it; regenerate with hook_slot_w 4.6 before the deck"],
                 ["Row E (Ø2 pins)", "snug at 2.10", "loose at 2.00: glue the pins. Binds at "
                  "2.20: ream the finger pin holes with a 2 mm drill"],
                 ["Port coupons", "dock without force", "binds at the dowels: note which step. "
                  "Slop after the pivot: note it, proceed (the thumbscrews close it)"]],
                [1.25 * inch, 1.75 * inch, 4.3 * inch]),
          Paragraph("Rows B (M3 tap 2.5-2.9) and C (heat-set 4.4-4.8) need screws and inserts from "
                    "the bench kit (bom/BOM.csv A-09 to A-13).", SMALL),
          Paragraph("Batch 1 fit criteria (write the verdicts in NOTES_INBOX.md)", H3),
          table([["[ ]", "<b>cup</b>: the blank slides in with finger pressure and doesn't rock; "
                  "the rim holes line up with a Ø2 pin pushed through."],
                 ["[ ]", "<b>yaw hub</b>: seats on the blank's horn with less than 0.3 mm wobble; "
                  "screws pass at both ends of the slots."],
                 ["[ ]", "<b>hip hub</b>: the coupler drops into the recess and the clamps draw it "
                  "flat; a 1 mm sideways push meets the lobes."],
                 ["[ ]", "<b>idler</b>: the tower drops over the glued idler without rocking; the "
                  "plug notch is clear."]],
                [0.35 * inch, 6.95 * inch], header=False)]
    return s


LEG_ASSEMBLY = [
    "Fork onto the yaw blank's horn, off the base: 4x M3 x 6 from below the hub (M2 x 6 + "
    "washer if the horn is M2). <b>Blocked as drawn (B80)</b>: the horn's centre head and the "
    "idler catch the C's lips by 1.0 + 1.5 mm, so do not force the fork on until B80 lands.",
    "Yaw blank + fork into the base cup from the front: 2 self-tappers from above into the "
    "idler-face rim holes, 2 from under the plate.",
    "Hip blank into the fork's cup from the front; 4 self-tappers.",
    "Knee blank into the carrier cup from the front; 4 self-tappers.",
    "Couplers onto the hip and knee horns: 4 screws each, through the counterbores.",
    "Plate A onto both couplers: 2x M3 x 8 clamps per hub from the outer face.",
    "Plate B over both idlers: 4x M3 x 12 into the bridge bosses.",
    "Tube into the carrier boss; M3 x 10 pinch bolt.",
]
LEG_HARDWARE = [
    ["A-08", "PA2.0 x 6 self-tappers", "12 (4 per cup, 3 cups; check the servo's own bag)"],
    ["A-09", "M3 x 6", "12 (4 per horn: yaw hub + two couplers); A-12 M2 x 6 + washers if M2"],
    ["A-10", "M3 x 8", "4 (coupler clamps)"],
    ["A-18", "M3 x 12", "4 (plate B)"],
    ["A-11", "M3 x 10", "1 (tube pinch bolt)"],
    ["A-19", "M3 x 16 hex head", "2 (the I1 thumbscrews, one pressed into each thumb_knob_m3)"],
    ["A-14", "CA glue", "the blank idlers"],
]


def batch_parts(b, est):
    """[(part, qty, est_g, est_min)] for a batch: the estimate's rows, then the extras."""
    rows = []
    if b["key"] is not None:
        for p in est.get(b["key"], {}).get("parts", []):
            rows.append((p["part"], p["qty"], p.get("est_g"), p.get("est_min")))
    for part, qty in b.get("extra", []):
        rows.append((part, qty, None, None))
    return rows


def part_info(part, b):
    mat, pose, sup, walls, notes = PARTS.get(
        part, ("?", "as exported", "?", PLA_DEFAULT, "Not in the pack's PARTS table: add it."))
    return b.get("material", mat), pose, sup, walls, notes


def audit_line(a):
    if not a:
        return "not audited (run check_printability.py)"
    bits = []
    n_isl = len(a.get("bed_support_islands", [])) + int(a.get("part_support_islands", 0))
    bits.append(f"{n_isl} supported island(s)" if n_isl else "no islands")
    if a.get("widest_overhang_step"):
        bits.append(f"widest overhang {a['widest_overhang_step']} mm at z {a['widest_overhang_z']}")
    if a.get("thin_soft_area"):
        bits.append(f"walls < 1.6 mm: {a['thin_soft_area']} mm² in a layer")
    if a.get("hard"):
        bits.append("<font color='#c62828'><b>HARD: " + "; ".join(a["hard"]) + "</b></font>")
    if not a.get("bed_ok", True):
        bits.append("<font color='#c62828'><b>does not fit the bed</b></font>")
    return "; ".join(bits)


def fem_block(r):
    c = VERDICT_COLOUR.get(r["verdict"], "#000000")
    txt = (f"<font color='{c}' size='11'><b>{r['verdict']}</b></font> &nbsp; SF <b>{r['sf']:.2f}</b>"
           f" &nbsp; governing case <b>{r['governing']}</b> "
           f"({FEM_CASES.get(r['governing'], '')})<br/>"
           f"held: {r.get('held', '')}<br/>loaded: {r.get('loaded', '')}")
    if r["part"] == "femur":
        txt += "<br/>femur_link + femur_plate_b analysed as one part."
    if r["verdict"] == "FAIL":
        txt += "<br/><b>Print it for fit; do not run a leg hard on it (B74).</b>"
    left = Paragraph(txt, CELL)
    png = os.path.join(FEM, f"{r['part']}.png")
    right = Image(png, width=3.3 * inch, height=1.65 * inch) if os.path.exists(png) else \
        Paragraph("(no stress picture)", CELL)
    t = Table([[left, right]], colWidths=[3.95 * inch, 3.35 * inch])
    t.setStyle(TableStyle([("BOX", (0, 0), (-1, -1), 0.8, colors.HexColor(c)),
                           ("VALIGN", (0, 0), (-1, -1), "MIDDLE")]))
    return t


def part_sheet(part, qty, g, mins, b, audit, fem_by_part, drawings, has_drawing, pages):
    mat, pose, sup, walls, notes = part_info(part, b)
    est = f"{g} g, {mins} min for {qty}" if g is not None else "not in the estimate"
    head = Table([[Paragraph(f"<font color='white'><b>{part}</b></font>", H3),
                   Paragraph(f"<font color='white'>qty <b>{qty}</b> &middot; {mat}</font>", BODY)]],
                 colWidths=[4.3 * inch, 3.0 * inch])
    head.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, -1), PURPLE),
                              ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                              ("ALIGN", (1, 0), (1, 0), "RIGHT")]))
    rows = [["print pose", pose], ["supports", sup], ["walls / infill", walls],
            ["estimate", est], ["audit", audit_line(audit.get(part))], ["notes", notes]]
    if has_drawing:
        d = drawings.get(part, {})
        holes = ", ".join(f"Ø{h['dia_mm']} x{h['count']} ({h['axis']})" for h in d.get("holes", []))
        ext = " x ".join(f"{e:g}" for e in d.get("extents_mm", []))
        rows.append(["drawing", f"TechDraw sheet on the next page (scale {d.get('scale', '?')}; "
                     f"extents {ext} mm; holes: {holes or 'none'})"])
    elif b["key"] == LEG_BATCH:
        rows.append(["drawing", "no drawing: run rocky.sh cad-drawings"])
    flow = [Mark("part:" + part, pages), head, Spacer(1, 2),
            Image(three_view(part), width=7.3 * inch, height=7.3 * inch * 3.1 / 9.6),
            table(rows, [1.1 * inch, 6.2 * inch], header=False)]
    r = fem_by_part.get(FEM_KEY.get(part, part))
    if r is not None:
        flow += [Spacer(1, 3), fem_block(r)]
    return KeepTogether(flow)


def batch_section(b, est, audit, fem_by_part, drawings, seen, pages, known):
    s = [Mark("sec:" + b["title"], pages),
         Paragraph(b["title"], H2), Paragraph(b["intro"], BODY), Spacer(1, 4)]
    parts = batch_parts(b, est)
    if b["key"] is not None and b["key"] not in est:
        s.append(Paragraph(f"<b>'{b['key']}' is missing from print_estimate.json</b>: run "
                           "print_estimate.py.", BODY))
    if b["key"] in est:
        e = est[b["key"]]
        s.append(Paragraph(f"Estimate: <b>{e['total_g']} g, {e['total_h']} h</b>.", BODY))
    rows = [["[ ]", "part", "qty", "material", "pose", "supports", "g", "min", "sheet"]]
    for part, qty, g, mins in parts:
        mat, pose, sup, _, _ = part_info(part, b)
        where = f"p. {known.get('part:' + part, '?')}" if part in seen else "below"
        rows.append(["[ ]", part, qty, mat, pose, sup, "" if g is None else g,
                     "" if mins is None else mins, where])
    s.append(table(rows, [w * inch for w in (0.3, 1.2, 0.35, 0.75, 1.45, 1.8, 0.45, 0.4, 0.6)]))
    if b["key"] == LEG_BATCH:
        s += [Paragraph("Assembly order (check_assembly.py asserts the final fits, not step 1's "
                        "path: B80)", H3),
              Paragraph("The full procedure, with pictures, is docs/ASSEMBLY_GUIDE.md.", SMALL),
              table([["[ ]", f"{i}. {t}"] for i, t in enumerate(LEG_ASSEMBLY, 1)],
                    [0.35 * inch, 6.95 * inch], header=False),
              Paragraph("To swap a hip or knee servo later: plate B off (4 screws), clamps out (4), "
                        "plate A off, then the cup's 4 rim screws. With real servos the leg drop "
                        "(XT30 + XH-5) runs through the base's harness channel.", SMALL),
              Paragraph("Hardware per leg (bom/BOM.csv; design counts, not a shopping list)", H3),
              table([["BOM", "item", "per leg"]] + LEG_HARDWARE,
                    [0.6 * inch, 1.8 * inch, 4.9 * inch])]
    new = [(p, q, g, m) for p, q, g, m in parts if p not in seen]
    if new:
        s.append(CondPageBreak(4.5 * inch))
    for part, qty, g, mins in new:
        seen.add(part)
        has_drawing = os.path.exists(os.path.join(DRAW, f"{part}.pdf"))
        s += [Spacer(1, 8),
              part_sheet(part, qty, g, mins, b, audit, fem_by_part, drawings, has_drawing, pages)]
        if has_drawing:     # a landscape placeholder page; merge_drawings() fills it
            s += [NextPageTemplate("landscape"), PageBreak(), Mark("draw:" + part, pages),
                  NextPageTemplate("portrait"), PageBreak()]
    return s


# ---------------------------------------------------------------------------
# build

def build_story(pages, known, meta, est, audit, fem, drawings):
    """Marks record into `pages`; the contents and 'p. N' references read
    `known`, the numbers the previous pass found."""
    fem_by_part = {r["part"]: r for r in (fem or {}).get("results", [])}
    s = cover(meta, est, fem) + [Spacer(1, 8)] + contents(known)
    s += [PageBreak(), Mark("sec:order", pages)] + print_order(fem)
    seen = set()
    for b in BATCHES:
        if not isinstance(s[-1], PageBreak):     # a drawing page already ended with one
            s.append(PageBreak())
        s += batch_section(b, est, audit, fem_by_part, drawings, seen, pages, known)
    return s


def render(meta, est, audit, fem, drawings, known):
    """One reportlab pass. `known` holds the page numbers from the previous
    pass (the contents and 'p. N' references need them); returns (bytes,
    pages found in this pass)."""
    buf = io.BytesIO()
    lm = 0.6 * inch
    doc = BaseDocTemplate(buf, pagesize=letter, invariant=1, title="Pebble print-prep pack",
                          author="cad/gen_print_pack.py", creator="gen_print_pack.py",
                          leftMargin=lm, rightMargin=lm, topMargin=0.5 * inch,
                          bottomMargin=0.55 * inch)
    pw, ph = letter
    lw, lh = landscape(letter)
    doc.addPageTemplates([
        PageTemplate("portrait", [Frame(lm, 0.55 * inch, pw - 2 * lm, ph - 1.05 * inch, id="p")],
                     onPage=_footer, pagesize=letter),
        PageTemplate("landscape", [Frame(0, 0, lw, lh, id="l")], onPage=_footer,
                     pagesize=landscape(letter)),
    ])
    found = {}
    doc.build(build_story(found, known, meta, est, audit, fem, drawings))
    return buf.getvalue(), found


def merge_drawings(pdf_bytes, pages):
    """Lay each TechDraw sheet (A4 landscape) onto its placeholder page, scaled
    to fit landscape letter and centred. Everything else passes through."""
    reader = PdfReader(io.BytesIO(pdf_bytes))
    writer = PdfWriter()
    placeholder = {n - 1: k.split(":", 1)[1] for k, n in pages.items() if k.startswith("draw:")}
    for i, page in enumerate(reader.pages):
        part = placeholder.get(i)
        if part is not None:
            sheet = PdfReader(os.path.join(DRAW, f"{part}.pdf")).pages[0]
            sw, sh = float(sheet.mediabox.width), float(sheet.mediabox.height)
            pw, ph = float(page.mediabox.width), float(page.mediabox.height)
            k = min((pw - 20) / sw, (ph - 36) / sh)
            tx, ty = (pw - sw * k) / 2, (ph - sh * k) / 2 + 6
            page.merge_transformed_page(sheet, Transformation().scale(k, k).translate(tx, ty))
        writer.add_page(page)
    writer.add_metadata({"/Title": "Pebble print-prep pack", "/Creator": "gen_print_pack.py"})
    writer.generate_file_identifiers()     # /ID = a checksum of the content, not random
    out = io.BytesIO()
    writer.write(out)                      # pypdf writes no dates
    return out.getvalue()


def main():
    with open(os.path.join(HERE, "params.yaml")) as f:
        P = yaml.safe_load(f)
    meta = {"rev": P.get("meta", {}).get("params_rev", "?"),
            "version": P.get("meta", {}).get("version", "?"),
            "bed": " x ".join(f"{v:g}" for v in P.get("print", {}).get("bed_mm", []))}
    est = {k: v for k, v in _load_json(os.path.join(OUT, "print_estimate.json"), {}).items()
           if isinstance(v, dict) and "parts" in v}
    audit = {a["part"]: a for a in _load_json(os.path.join(OUT, "printability.json"), [])}
    fem = _load_json(os.path.join(FEM, "fem_results.json"), None)
    drawings = _load_json(os.path.join(DRAW, "drawings.json"), {})

    # two passes (three at most): the first finds the page numbers, the next
    # prints them; stop when a pass reproduces the numbers it was given
    known = {}
    for _ in range(3):
        pdf, found = render(meta, est, audit, fem, drawings, known)
        if found == known:
            break
        known = found
    missing = [p for p, *_ in batch_parts(BATCHES[3], est)
               if not os.path.exists(os.path.join(DRAW, f"{p}.pdf"))]
    pdf = merge_drawings(pdf, found)
    with open(PDF, "wb") as f:
        f.write(pdf)
    n_draw = sum(1 for k in found if k.startswith("draw:"))
    n_pages = len(PdfReader(io.BytesIO(pdf)).pages)
    if missing:
        print("no drawing for:", ", ".join(missing), "(run rocky.sh cad-drawings)")
    print(f"PDF written: {PDF} ({n_pages} pages, {n_draw} drawings)")


if __name__ == "__main__":
    main()
