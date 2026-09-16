"""Ramped TPU shin fairing — does it fix the >40 mm shin-catch? (backlog B1)

Hypothesis from run_stuck.py: at 45 mm rubble even the 52 mm retry step
fails, because boxes catch the thin (r10) shin ABOVE the foot — step height
can't fix a shin collision. A TPU ramp fairing fattens the lower shin into a
ski (r ~17 tapering up), deflecting obstacles under/around instead of
hooking them.

Sim model: add a second, larger capsule along the lower 55% of each tibia
(+8 g each, honest mass). Sweep rubble 30..45 mm x 4 seeds:
  A: bare shin (today's baseline, from stuck_results.json)
  B: fairing, no watchdog
  C: fairing + watchdog     <- the full Phase-3 terrain stack candidate

Usage: MUJOCO_GL=osmesa python3 run_fairing.py
"""
import json
import os
import re
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
import run_stuck                                                    # noqa: E402
from run_terrain import N_BOX, FIELD                                # noqa: E402

FAIRING_R = 0.017          # m — printed-TPU ski radius at the ankle
FAIRING_MASS = 0.008       # kg per leg


def patch_fairing(xml: str) -> str:
    """Insert the fairing capsule after each tibia capsule geom."""
    pat = re.compile(
        r'(<geom type="capsule" fromto="0 0 0 (0\.1242) 0 0" size="0\.010"[^/]*/>)')

    def repl(m):
        L = float(m.group(2))
        add = (f'<geom type="capsule" fromto="{L*0.45:.4f} 0 0 {L*1.02:.4f} 0 0" '
               f'size="{FAIRING_R}" mass="{FAIRING_MASS}" '
               f'friction="0.8 0.01 0.001" rgba="0.8 0.72 0.9 0.7"/>')
        return m.group(1) + add
    out, n = pat.subn(repl, xml)
    assert n == 5, f"fairing patch matched {n} tibias, expected 5"
    return out


def build_model_faired(amp_mm, seed):
    m = run_stuck.build_model.__wrapped__ if hasattr(run_stuck.build_model, "__wrapped__") \
        else None
    # simplest: replicate run_terrain.build_model but patch the base xml
    from run_terrain import build_model as _bm
    import run_terrain
    with open(os.path.join(HERE, "pebble.xml")) as f:
        xml = patch_fairing(f.read())
    boxes = []
    if amp_mm > 0:
        rng = np.random.default_rng(seed)
        for b in range(N_BOX):
            x = rng.uniform(FIELD[0], FIELD[1])
            y = rng.uniform(FIELD[2], FIELD[3])
            if np.hypot(x, y) < 0.26:
                continue
            sx, sy = rng.uniform(0.010, 0.020, 2)
            h = rng.uniform(0.5, 1.0) * amp_mm / 1000.0
            yaw = rng.uniform(0, 180)
            boxes.append(
                f'<geom type="box" size="{sx:.3f} {sy:.3f} {h/2:.4f}" '
                f'pos="{x:.3f} {y:.3f} {h/2:.4f}" euler="0 0 {yaw:.0f}" '
                f'friction="1.2 0.01 0.001" rgba="0.3 0.28 0.38 1"/>')
    xml = xml.replace("<worldbody>", "<worldbody>\n    " + "\n    ".join(boxes))
    return mujoco.MjModel.from_xml_string(xml)


def main():
    # monkey-patch run_stuck's model builder to inject the fairing
    run_stuck.build_model = build_model_faired
    results = []
    for amp in (30, 35, 40, 45):
        for seed in (0, 1, 2, 3):
            for dog in ((False, True) if amp >= 40 else (False,)):
                r = run_stuck.run_trial(amp, seed, dog)
                r["fairing"] = True
                results.append(r)
                print(f"amp {amp} seed {seed} fairing dog={int(dog)}: "
                      f"disp {r['disp_x']:4d}  crossed={int(r['crossed'])} "
                      f"t={r['t_cross']} tilt {r['tilt_max']:4.1f}")
    with open(os.path.join(HERE, "fairing_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    # summary vs the bare-shin stuck_results.json
    with open(os.path.join(HERE, "stuck_results.json")) as f:
        bare = json.load(f)
    print("\n=== crossing rate /4 fields: bare -> faired (no dog) "
          "-> faired+dog ===")
    for amp in (30, 35, 40, 45):
        b0 = sum(r["crossed"] for r in bare if r["amp"] == amp and not r["watchdog"])
        b1 = sum(r["crossed"] for r in bare if r["amp"] == amp and r["watchdog"])
        f0 = sum(r["crossed"] for r in results if r["amp"] == amp and not r["watchdog"])
        f1 = sum(r["crossed"] for r in results if r["amp"] == amp and r["watchdog"])
        fd = f"{f1}/4" if amp >= 40 else "  - "
        print(f"  {amp} mm: bare {b0}/4 (dog {b1}/4)  ->  faired {f0}/4  "
              f"(faired+dog {fd})")
    print("wrote fairing_results.json")


if __name__ == "__main__":
    main()
