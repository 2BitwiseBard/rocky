"""Run INSIDE FreeCAD (FreeCADCmd): wrap a fem_check CalculiX result in a
FreeCAD document with an Analysis, so the GUI shows it (D061).

    FreeCADCmd cad/fem_to_freecad.py          (reads FEM_FRD, writes FEM_FCSTD)

FreeCAD 1.1's .frd importer needs an Analysis to hang its result pipelines
on; opening a bare .frd raises inside setupPipeline. `rocky.sh cad-open
--fem PART` calls this, then opens the .FCStd.
"""
import os
import traceback

import FreeCAD as App

frd, out = os.environ["FEM_FRD"], os.environ["FEM_FCSTD"]
status = out + ".status"
try:
    import ObjectsFem
    from feminout import importCcxFrdResults
    doc = App.newDocument(os.path.splitext(os.path.basename(out))[0])
    analysis = ObjectsFem.makeAnalysis(doc, "Analysis")
    importCcxFrdResults.importFrd(frd, analysis)
    doc.recompute()
    doc.saveAs(out)
    with open(status, "w") as f:
        f.write("ok %d objects\n" % len(doc.Objects))
except Exception:
    with open(status, "w") as f:
        f.write(traceback.format_exc())
os._exit(0)
