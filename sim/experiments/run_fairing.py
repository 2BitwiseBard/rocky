"""Ramped TPU shin fairing — does it fix the >40 mm shin-catch? (backlog B1)

Hypothesis from run_stuck.py: at 45 mm rubble even the 52 mm retry step
fails, because boxes catch the thin (r10) shin ABOVE the foot — step height
can't fix a shin collision. A TPU ramp fairing fattens the lower shin into a
ski (r ~17 tapering up), deflecting obstacles under/around instead of
hooking them.

Sim model: add a second, larger capsule along the lower 55% of each tibia
(+8 g each, honest mass). Every trial is run_stuck.run_trial with its limits:
the walk asks scenes.walk_ask (the envelope) for run_stuck.T_WALK (the same
commanded distance as 25 s at 45 mm/s, B111). Sweep rubble 30..45 mm x 4 seeds:
  A: bare shin (run_stuck's result, stuck_results.json: re-run run_stuck first,
     else the pre-D052 record is used)
  B: fairing, no watchdog
  C: fairing + watchdog     <- the full Phase-3 terrain stack candidate

Usage: MUJOCO_GL=egl .venv/bin/python sim/experiments/run_fairing.py
"""
import json
import re


import exp_paths as X                   # sys.path (sim/, gait/, perception/, audio/) + where results / clips go
import run_stuck                                                    # noqa: E402
from scenes import build_model, load_xml                            # noqa: E402

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
    """The bare-shin rubble field (scenes.build_model: the same boxes for the
    same amp / seed) around the faired robot. (Before the sim/experiments move
    this was a copy that skipped the per-box colour draw, so from the second
    box on its fields were NOT the baseline's: the pre-D052 A/B compared
    different rubble.)"""
    return build_model(amp_mm, seed, xml=patch_fairing(load_xml()))


def main():
    # monkey-patch run_stuck's model builder to inject the fairing
    run_stuck.build_model = build_model_faired
    print(f"walk: {run_stuck.V_X:.1f} mm/s for {run_stuck.T_WALK:.1f} s (run_stuck's limits)")
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
    with open(X.result("fairing_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    # summary vs the bare-shin stuck_results.json
    bare_path = X.prior("stuck_results.json")
    with open(bare_path) as f:
        bare = json.load(f)
    print(f"bare-shin baseline: {X.shown(bare_path)}")
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
