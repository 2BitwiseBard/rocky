#!/usr/bin/env python3
"""Dimensioned A4 drawings of the printed parts, from their STEP files, by
FreeCAD TechDraw (D061). One sheet per part in cad/out/drawings/:
<part>.pdf, INDEX.md (scale, extents, hole table per part) and
drawings.json (the same + the STEP each sheet was drawn from).

    python3 gen_drawings.py              # the one-leg batch of the print plan
    python3 gen_drawings.py coxa_fork    # named parts (any cad/out/<name>.step)
    python3 gen_drawings.py --force      # redraw even when the STEP is unchanged

What a sheet carries: ISO frame + title block (part, scale, estimated
grams, params revision), first-angle Front / Top / Right + an isometric,
overall extents dimensioned, a hole table and, where the part has any, a
table of open channels (half-circle pockets open to an edge, B105). What it
does not: tolerances,
section views, every feature size. It is for checking a print with calipers
and for the print pack, not a manufacturing drawing.

Needs FreeCAD >= 1.1 (ROCKY_FREECADCMD, FreeCADCmd on PATH, or the Flatpak).
Each sheet runs FreeCAD's console binary with its GUI on Qt's `minimal`
platform (no window; the page exporters live in the GUI) in a private user
directory (cad/out/drawings/work/, git-ignored), so your own FreeCAD settings
and add-ons, the MCP add-on's port included, are never touched. ~7 s a part.
A sheet is redrawn only when its STEP's geometry or the drawer (td_sheet.py)
changed (drawings.json keeps a hash of each): TechDraw's hidden-line removal
runs in threads and emits its edges in a different order each run, so the
PDF bytes are never stable, and an unchanged part must leave git clean.
"""
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

import fem_tools as ft
from common import params, _step_body

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out", "drawings")
WORK = os.path.join(OUT, "work")
LEG_BATCH = "Batch 2 one leg"                    # print_estimate.json: the parts of one leg
TIMEOUT_S = 180


def _estimate():
    try:
        with open(os.path.join(HERE, "out", "print_estimate.json")) as f:
            est = json.load(f)
    except OSError:
        return {}, []
    grams = {p["part"]: p["est_g"] / p["qty"] for b in est.values() if isinstance(b, dict)
             for p in b.get("parts", [])}
    leg = [p["part"] for p in est.get(LEG_BATCH, {}).get("parts", [])]
    return grams, leg


def _cmd(base, env):
    """FreeCAD argv with env passed through (a Flatpak needs --env=)."""
    if base[:2] == ["flatpak", "run"]:
        return base[:2] + [f"--env={k}={v}" for k, v in env.items()] + base[2:]
    return base


def _step_sha(path):
    return hashlib.sha256(_step_body(path)).hexdigest()[:16]


def _sheet_rev():
    """Hash of the drawer: a change to what a sheet carries redraws every sheet."""
    with open(os.path.join(HERE, "td_sheet.py"), "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:16]


def sheet(fc, name, P, grams):
    step = os.path.join(HERE, "out", f"{name}.step")
    if not os.path.exists(step):
        raise FileNotFoundError(f"no {step}: run cad-check first")
    home = os.path.join(WORK, "fc_home")
    os.makedirs(home, exist_ok=True)
    out = os.path.join(OUT, name)
    g = grams.get(name)
    env = {"QT_QPA_PLATFORM": "minimal:enable_fonts", "FREECAD_USER_HOME": home,
           "TD_STEP": step, "TD_OUT": out, "TD_TITLE": name,
           "TD_SUBTITLE": f"rocky - mm - from cad/out/{name}.step",
           "TD_DATE": f"params {P['meta']['params_rev']}" if "meta" in P else "",
           "TD_WEIGHT": f"~{g:.0f} g" if g else ""}
    log = os.path.join(WORK, f"{name}.log")
    with open(log, "w") as f:
        r = subprocess.run(_cmd(fc, env) + [os.path.join(HERE, "td_sheet.py")],
                           env=dict(os.environ, **env), stdout=f, stderr=subprocess.STDOUT,
                           timeout=TIMEOUT_S)
    if r.returncode != 0 or not os.path.exists(out + ".pdf"):
        raise RuntimeError(f"FreeCAD failed on {name} (log: {log})")
    with open(out + ".json") as f:
        info = json.load(f)
    os.remove(out + ".json")
    info["step_sha"] = _step_sha(step)
    info["sheet_rev"] = _sheet_rev()
    return info


def main():
    want = [a for a in sys.argv[1:] if not a.startswith("--")]
    force = "--force" in sys.argv
    fc = ft.tool_cmd("FreeCADCmd", "ROCKY_FREECADCMD")
    if not fc:
        print("gen_drawings: SKIPPED (no FreeCAD: install it, or set ROCKY_FREECADCMD)")
        return 0
    P = params()
    grams, leg = _estimate()
    names = want or leg
    if not names:
        print("gen_drawings: no parts (print_estimate.json missing: run cad-check --derived)")
        return 1
    os.makedirs(WORK, exist_ok=True)
    man_path = os.path.join(OUT, "drawings.json")
    try:
        with open(man_path) as f:
            man = json.load(f)
    except OSError:
        man = {}
    t0, bad = time.time(), []
    for n in names:
        t = time.time()
        step = os.path.join(HERE, "out", f"{n}.step")
        if (not force and n in man and os.path.exists(step) and os.path.exists(os.path.join(OUT, f"{n}.pdf"))
                and man[n].get("step_sha") == _step_sha(step)
                and man[n].get("sheet_rev") == _sheet_rev()):
            print(f"SAME  {n:22s} (STEP and drawer unchanged)")
            continue
        try:
            man[n] = sheet(fc, n, P, grams)
            print(f"DRAWN {n:22s} scale {man[n]['scale']:<5} {time.time() - t:4.1f} s")
        except Exception as e:
            bad.append(n)
            print(f"FAIL  {n:22s} {e}")
    man = dict(sorted(man.items()))
    with open(man_path, "w") as f:
        json.dump(man, f, indent=1)
        f.write("\n")
    L = ["# Part drawings", "",
         "Generated by `cad/gen_drawings.py` (`rocky.sh cad-drawings`; FreeCAD TechDraw, D061) from "
         "`cad/out/*.step`. Do not edit by hand. One A4 sheet per part: first-angle Front / Top / "
         "Right + an isometric, overall extents, a hole table and the open channels (half-circle "
         "pockets open to an edge; a slot's ends are not listed). For calipers and the print "
         "pack, not for manufacture.", "",
         "| part | sheet | scale | extents W × H, depth (mm) | holes | open channels (Ø × depth) |",
         "|---|---|---|---|---|---|"]
    for n, info in man.items():
        ex = info["extents_mm"]
        holes = ", ".join(f"Ø{h['dia_mm']:g} ×{h['count']} ({h['axis']})" for h in info["holes"]) or "none"
        chan = ", ".join(f"Ø{c['dia_mm']:g} × {c['depth_mm']:g} ×{c['count']} ({c['axis']})"
                         for c in info.get("channels", [])) or "none"
        L.append(f"| {n} | [pdf]({n}.pdf) | {info['scale']:g} | {ex[0]:g} × {ex[1]:g}, {ex[2]:g} | {holes} "
                 f"| {chan} |")
    with open(os.path.join(OUT, "INDEX.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    shutil.rmtree(os.path.join(WORK, "fc_home"), ignore_errors=True)
    print(f"gen_drawings: {len(names) - len(bad)}/{len(names)} sheets current ({time.time() - t0:.0f} s)"
          + (f" — FAILING: {', '.join(bad)}" if bad else ""))
    return len(bad)


if __name__ == "__main__":
    sys.exit(main())
