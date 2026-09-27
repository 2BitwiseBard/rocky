"""Where the experiments read and write. Every script in sim/experiments/
imports this first; importing it puts sim/, gait/, perception/, audio/ and the
repo root on sys.path, so a script runs from anywhere:

    MUJOCO_GL=egl .venv/bin/python sim/experiments/run_cliff.py

    SIM       sim/                               pebble.xml + the live modules (scenes, righter, shove, ...)
    RESULTS   sim/experiments/results/           result JSONs + figures: the current record
    PRE_D052  sim/experiments/results/pre_d052/  the 2026-09-22 record behind D017-D040 (history)
    OUT       sim/experiments/out/               videos + temporary mux files (git-ignored)

ROCKY_EXPERIMENTS_OUT=DIR sends both results and clips to DIR instead (a
scratch run that leaves the tracked record alone).
"""
import os
import sys

EXP = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(EXP)
ROOT = os.path.dirname(SIM)
RESULTS = os.path.join(EXP, "results")
PRE_D052 = os.path.join(RESULTS, "pre_d052")
OUT = os.path.join(EXP, "out")
MODEL_XML = os.path.join(SIM, "pebble.xml")

for _p in (ROOT, os.path.join(ROOT, "audio"), os.path.join(ROOT, "perception"),
           os.path.join(ROOT, "gait"), SIM):
    if _p not in sys.path:
        sys.path.insert(0, _p)
if EXP in sys.path:                      # keep this directory first: experiments import each other
    sys.path.remove(EXP)
sys.path.insert(0, EXP)

_SCRATCH = os.environ.get("ROCKY_EXPERIMENTS_OUT") or None


def result(name):
    """Path for a result JSON / figure (sim/experiments/results/, made on demand)."""
    d = _SCRATCH or RESULTS
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, name)


def out(name):
    """Path for a video or a temporary file (sim/experiments/out/, git-ignored)."""
    d = _SCRATCH or OUT
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, name)


def shown(path):
    """A path for printing: relative to the repo root when it is inside it (no
    machine-specific prefix in logs), else as given."""
    p = os.path.abspath(path)
    return os.path.relpath(p, ROOT) if p.startswith(ROOT + os.sep) else path


def prior(name):
    """An earlier result an experiment compares against: the current one when it
    has been re-run, else the pre-D052 record (None when neither exists)."""
    for d in (_SCRATCH, RESULTS, PRE_D052):
        if d and os.path.exists(os.path.join(d, name)):
            return os.path.join(d, name)
    return None
