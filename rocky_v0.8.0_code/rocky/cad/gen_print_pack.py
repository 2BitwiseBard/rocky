"""Generate the Print-Prep Pack PDF: per-part 3-view reference sheets with
dimensions, slicer settings, and the printer-night checklist."""
import os
import numpy as np
from stl import mesh as stlmesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from reportlab.lib.pagesizes import letter
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Image,
                                Table, TableStyle, PageBreak)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.units import inch

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

# part, material, orientation, supports, walls/infill, qty, notes
PARTS = [
    ("hand_hub", "PLA first / PETG final", "hub top face DOWN (boss up)", "minimal (under lug ring)",
     "4 walls / 30%", 1, "Print FIRST - mechanism validation. Ream pin holes 2.0mm."),
    ("hand_cam", "CF-PLA or PETG", "flat", "none", "4 walls / 40%", 1,
     "Slot profile is v0.1 placeholder - expect to reprint after bench tuning."),
    ("hand_finger", "PLA first / PETG final", "on wedge side face", "under knuckle tab only",
     "3 walls / 25%", 3, "Three identical. Light part - fast iterations."),
    ("coxa_yaw_base", "PETG", "base plate DOWN", "under crown arm only",
     "5 walls / 40%", 1, "Bearing pocket: test-fit 683ZZ, ream gently. Print 1, verify, then more."),
    ("coxa_fork", "PETG", "hub DOWN", "none", "5 walls / 40%", 1,
     "Zip-slot bridges are small and print fine. Stub is fragile until installed."),
    ("coxa_crown_cap", "PETG", "bearing pocket UP (flipped)", "none", "5 walls / 40%", 1,
     "D046: bolts onto the post AFTER the fork is on. 683ZZ light press; M3 x 10 axle through it."),
    ("coxa_fork_strap", "PETG", "bar DOWN (flipped)", "none", "4 walls / 30%", 1,
     "D046: 2x M3 x 8 into the rail tops; replaces the zip ties."),
    ("femur_link", "CF-PLA (hardened nozzle)", "flat, recess face UP", "none", "6 walls / 40%", 1,
     "Layer lines along the beam. D046: hubs take the horn coupler (2x M3 x 8 clamps each)."),
    ("horn_coupler", "PETG", "disc DOWN", "none", "4 walls / 40%", 2,
     "D046: one per hip/knee horn. M2 x 8 + nut on a blank; M2 x 6 into a real horn."),
    ("tibia_knee_carrier", "PETG", "floor DOWN", "under tube boss",
     "5 walls / 40%", 1, "Pinch-bolt slit prints as-is; run an M3 tap or bolt through after."),
    ("tibia_knee_strap", "PETG", "bar DOWN (flipped)", "none", "4 walls / 30%", 1,
     "D046: 2x M3 x 8 into the carrier wall tops."),
    ("coupon_j1_hub", "PETG", "hub DOWN", "none", "4 walls / 30%", 1,
     "D046 J1 COUPON — print with coupon_j1_post + cap FIRST; assemble with a 683ZZ + M3 x 10."),
    ("coupon_j1_post", "PETG", "plate DOWN", "none", "4 walls / 30%", 1, "D046 J1 coupon, post half."),
    ("coupon_j2_hub", "PLA", "flat, recess UP", "none", "4 walls / 30%", 1,
     "D046 J2 COUPON — with coupon_j2_horn + horn_coupler: 4x M2 x 8 + nuts, 2x M3 x 8."),
    ("coupon_j2_horn", "PLA", "horn UP", "none", "2 walls / 20%", 1, "D046 J2 coupon, horn half (nut slots)."),
    ("tibia_sea_outer", "PETG", "tube socket DOWN", "none", "4 walls / 35%", 1,
     "Verify slider glides in bore before gluing tube. Switch pocket: KW10-class."),
    ("tibia_sea_slider", "PETG", "flange DOWN", "none", "4 walls / 35%", 1,
     "Keys must slide in keyways with light wiggle - sand if tight."),
    ("body_deck", "PETG or PLA", "flat", "none", "4 walls / 30% gyroid", 1,
     "Biggest part (190x181) - brim ON, dry filament, watch first layer corners."),
]

VIEWS = [("Top (XY)", 0, 1), ("Front (XZ)", 0, 2), ("Side (YZ)", 1, 2)]

def three_view(name):
    m = stlmesh.Mesh.from_file(os.path.join(OUT, f"{name}.stl"))
    v = m.vectors
    lo, hi = v.reshape(-1, 3).min(0), v.reshape(-1, 3).max(0)
    size = hi - lo
    fig, axes = plt.subplots(1, 3, figsize=(9.6, 3.1), dpi=110)
    for ax, (label, ix, iy) in zip(axes, VIEWS):
        tris2 = v[:, :, [ix, iy]]
        # simple painter shading by the dropped axis mean
        iz = 3 - ix - iy
        order = np.argsort(v[:, :, iz].mean(axis=1))
        depth = v[:, :, iz].mean(axis=1)[order]
        dn = (depth - depth.min()) / (np.ptp(depth) + 1e-9)
        cols = np.outer(0.35 + 0.5 * dn, np.array([0.55, 0.44, 0.79]))
        pc = PolyCollection(tris2[order], facecolors=np.clip(cols, 0, 1), edgecolors="none")
        ax.add_collection(pc)
        ax.set_xlim(lo[ix] - 5, hi[ix] + 5); ax.set_ylim(lo[iy] - 5, hi[iy] + 5)
        ax.set_aspect("equal"); ax.set_title(
            f"{label}   {size[ix]:.1f} x {size[iy]:.1f} mm", fontsize=8)
        ax.tick_params(labelsize=6); ax.grid(alpha=0.25, lw=0.4)
    fig.suptitle(f"{name}   —   bbox {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm",
                 fontsize=10, fontweight="bold")
    plt.tight_layout(rect=(0, 0, 1, 0.94))
    p = os.path.join(OUT, f"views_{name}.png")
    plt.savefig(p); plt.close(fig)
    return p

styles = getSampleStyleSheet()
h1 = styles["Title"]; h2 = styles["Heading2"]; body = styles["BodyText"]
small = ParagraphStyle("small", parent=body, fontSize=8.5, leading=11)

doc = SimpleDocTemplate(os.path.join(HERE, "..", "PRINT_PREP_PACK.pdf"),
                        pagesize=letter, topMargin=40, bottomMargin=36)
story = []
story.append(Paragraph("Pebble — Print-Prep Pack (v0.4)", h1))
story.append(Paragraph(
    "Project ROCKY · printer-night companion. All dimensions are CAD-nominal in mm; "
    "servo-interface dims remain VERIFY until the caliper session. Global PETG start: "
    "240&deg;C / bed 85&deg;C / fan 30-50% / 0.2 mm layers / brim on structural parts. "
    "PLA: 210/60, full fan. CF-PLA: 225/60, HARDENED NOZZLE.", body))
story.append(Spacer(1, 8))

story.append(Paragraph("Printer-night checklist (in order)", h2))
checklist = [
    "1. Reassemble hot end from spares; check thermistor/heater leads seated.",
    "2. PID tune (hot end + bed), then Live-Z first-layer calibration on the PETG you'll use.",
    "3. E-steps / flow sanity cube; belt twang check.",
    "4. Acceptance part: ONE hand_finger in PLA (20 min) - checks dimensions, overhangs, cooling.",
    "5. Print the hand set in PLA (hub, cam, 3x fingers) -> assemble with M2 pins, tune by hand.",
    "6. Print coxa_yaw_base + coxa_fork in PETG -> test-fit 683ZZ bearing + (when here) a servo.",
    "7. Log EVERYTHING in NOTES_INBOX.md: what stuck, what warped, measured hole sizes.",
]
for c in checklist:
    story.append(Paragraph(c, small))
story.append(Spacer(1, 6))

rows = [["Part", "Material", "Orientation", "Supports", "Walls/Infill", "Qty"]]
for name, mat, ori, sup, wi, qty, _ in PARTS:
    rows.append([name, mat, ori, sup, wi, str(qty)])
t = Table(rows, colWidths=[1.35*inch, 1.25*inch, 1.45*inch, 1.25*inch, 0.95*inch, 0.35*inch])
t.setStyle(TableStyle([
    ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#5b4a8a")),
    ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
    ("FONTSIZE", (0, 0), (-1, -1), 7.2),
    ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#c9bfe0")),
    ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f2eefa")]),
    ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
]))
story.append(t)
story.append(PageBreak())

for name, mat, ori, sup, wi, qty, note in PARTS:
    png = three_view(name)
    story.append(Image(png, width=7.0*inch, height=2.26*inch))
    story.append(Paragraph(f"<b>{name}</b> &nbsp;&nbsp; qty {qty} &nbsp;·&nbsp; {mat} "
                           f"&nbsp;·&nbsp; {ori} &nbsp;·&nbsp; supports: {sup} &nbsp;·&nbsp; {wi}", small))
    story.append(Paragraph(note, small))
    story.append(Spacer(1, 10))

doc.build(story)
print("PDF written:", os.path.join(HERE, "..", "PRINT_PREP_PACK.pdf"))
