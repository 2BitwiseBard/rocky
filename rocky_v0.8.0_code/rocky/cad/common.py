"""Shared helpers for Pebble CAD: parameter loading + export."""
import os, yaml
from build123d import export_step, export_stl

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)

def params():
    with open(os.path.join(HERE, "params.yaml")) as f:
        return yaml.safe_load(f)

def export(part, name, multi=False):
    """Write STEP + STL. D036 gate (session 8): a printable export must be
    exactly ONE connected solid — interference checks can't see a part that
    has drifted APART (the hand-hub lugs, the scoop neck, the shell feet,
    the fork collar, the sled tail, the jig plate and the stand seat all
    shipped floating while every boolean check said CLEAN). Assemblies and
    posed previews pass multi=True explicitly; nothing else may."""
    solids = part.solids()
    n = len(solids)
    if n != 1 and not multi:
        vols = sorted((s.volume for s in solids), reverse=True)
        raise RuntimeError(
            f"{name}: {n} disconnected solids (volumes mm^3: "
            f"{', '.join(f'{v:.0f}' for v in vols[:6])}) — a printable export "
            f"must be ONE solid (D036). Pass multi=True only for assemblies.")
    step = os.path.join(OUT, f"{name}.step")
    stl = os.path.join(OUT, f"{name}.stl")
    export_step(part, step)
    export_stl(part, stl)
    print(f"exported {name}: volume={part.volume/1000:.1f} cm^3, "
          f"solids={n}")
    return step, stl
