"""Push envelope + phase probe for the GAIT-PHASE-AWARE brace (reflex v2).

What diag_brace_phase.py showed (2026-07-31): the D022 +0.25T pocket is a
real fall. The shove bounces feet; v1 freezes every target at brace entry,
so bounced feet never re-land, support collapses to a 2-leg line, robot
pivots over it. v2 (pebble_reflex.py): the commanded-swing leg plants on a
fast ramp, and any foot the microswitches report airborne ground-seeks
until it touches.

Modes measured back-to-back on identical conditions:
  base — no reflex (D017 harness)
  v1   — contact_aware=False, no contact feed (exactly the D022 reflex)
  v2   — phase-aware fast plant + contact-seeking (microswitch stand-in
         from foot contact forces, threshold 1.5 N as in run_odom.py)

Usage:
  python3 run_push_reflex_v2.py --envelope   # 12-dir walk envelope, v2
  python3 run_push_reflex_v2.py --probe      # 8-phase probe x {base,v1,v2}
  python3 run_push_reflex_v2.py --merge      # combine + figure + summary
(envelope and probe are separate so they can run on two cores.)
"""
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS       # noqa: E402
from pebble_reflex import ReflexSupervisor, body_gyro_xy            # noqa: E402
from run_push_reflex import make_data, gyro_xy_of, T_SETTLE, V_X    # noqa: E402

T_PUSH = 3.5
DUR = 0.15
T_AFTER = 4.0
DIRS = np.arange(0, 360, 30)
FORCES = [14, 22, 27, 33, 40, 48, 57, 67, 78, 90]
PROBE_FRACS = [0.0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875]
CONTACT_N = 1.5           # microswitch stand-in threshold (run_odom.py)
TRIP = 1.8                # from push_reflex_results.json calibration


def foot_contacts(model, data, fids):
    out = np.zeros(N_LEGS, dtype=bool)
    for c in range(data.ncon):
        con = data.contact[c]
        for i in range(N_LEGS):
            if fids[i] in (con.geom1, con.geom2):
                F = np.zeros(6)
                mujoco.mj_contactForce(model, data, c, F)
                if F[0] > CONTACT_N:
                    out[i] = True
    return out


def trial(model, walk, dir_deg, force_n, mode, push_t=T_PUSH, dur=DUR,
          check_progress=True):
    """mode in {'base','v1','v2'}. Returns (ok, tilt_max, skid_mm) where
    skid = displacement along the push direction while being pushed."""
    gait = WaveGait()
    data, q0 = make_data(model, gait)
    torso = model.body("torso").id
    fids = [model.geom(f"foot{i}").id for i in range(N_LEGS)]
    DT = model.opt.timestep
    if mode == "base":
        sup = None
    else:
        sup = ReflexSupervisor(gait, gyro_trip=TRIP, gyro_calm=TRIP / 2,
                               contact_aware=(mode == "v2"))
    fdir = np.array([np.cos(np.deg2rad(dir_deg)),
                     np.sin(np.deg2rad(dir_deg)), 0.0])
    f = force_n * fdir
    tilt_max, fell = 0.0, False
    pos_at_push = None
    pos_at_push_end = None
    total = push_t + dur + T_AFTER
    for k in range(int(total / DT)):
        t = k * DT
        gyro = gyro_xy_of(model, data, torso)
        if t < T_SETTLE or not walk:
            vx = 0.0
        else:
            vx = V_X * min((t - T_SETTLE) / 0.6, 1.0)
        if sup is not None:
            if mode == "v2":
                con = foot_contacts(model, data, fids)
                R = data.xmat[torso].reshape(3, 3)
                w_body = R.T @ data.cvel[torso][0:3]
                gvec = w_body[:2]
            else:
                con, gvec = None, None
            q, _state = sup.step(t, vx, 0.0, 0.0, gyro, contacts=con,
                                 gyro_vec=gvec)
            data.ctrl[:15] = q.flatten()
        else:
            if walk and t >= T_SETTLE:
                q, _, _ = gait.joint_targets(t - T_SETTLE, vx, 0.0, 0.0)
                data.ctrl[:15] = q.flatten()
            else:
                data.ctrl[:15] = q0
        if push_t <= t < push_t + dur:
            data.xfrc_applied[torso, :3] = f
            if pos_at_push is None:
                pos_at_push = data.xpos[torso].copy()
        else:
            data.xfrc_applied[torso, :3] = 0.0
            if pos_at_push is not None and pos_at_push_end is None:
                pos_at_push_end = data.xpos[torso].copy()
        mujoco.mj_step(model, data)
        zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
        tilt = np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1)))
        tilt_max = max(tilt_max, tilt)
        if tilt > 60 or data.xpos[torso][2] < 0.050:
            fell = True
            break
    zaxis = data.xmat[torso].reshape(3, 3)[:, 2]
    tilt_end = np.rad2deg(np.arccos(np.clip(zaxis[2], -1, 1)))
    ok = (not fell) and tilt_end < 25 and data.xpos[torso][2] > 0.070
    skid = 0.0
    if pos_at_push is not None:
        ref = pos_at_push_end if pos_at_push_end is not None \
            else data.xpos[torso]
        skid = float((ref - pos_at_push) @ fdir) * 1000
    if walk and ok and check_progress and pos_at_push is not None:
        prog = (data.xpos[torso] - pos_at_push)[0] * 1000
        frac_needed = 0.35 if mode == "base" else 0.15   # braced pause is on purpose
        ok = prog > frac_needed * V_X * (dur + T_AFTER)
    return ok, tilt_max, skid


def climb(model, walk, dir_deg, mode, push_t=T_PUSH):
    best = 0.0
    for F in FORCES:
        ok, _, _ = trial(model, walk, dir_deg, F, mode, push_t)
        if ok:
            best = F
        else:
            break
    return best


SUS_DUR = 1.2                      # someone leaning on it, not a tap
SUS_FORCES = [8, 12, 16, 20, 25, 31, 38, 46]


def do_sustained(model):
    """The reflex's actual hypothesis-job: a DISTURBANCE LONGER THAN A GAIT
    CYCLE. The aligned single-impulse envelope says the bare wave gait is
    the best 0.15 s-shove controller (mean 43.5 N vs v2 40.9) — freezing
    discards the stepping recovery. Sustained lean is a different regime:
    the gait gets herded downstream force-left-on, the brace digs in.
    Survival force + skid distance while pushed, per direction."""
    out = {}
    for d in (0, 90, 180, 270):
        row = {}
        for mode in ("base", "v2"):
            best, skid_at_best = 0.0, None
            for F in SUS_FORCES:
                ok, _, skid = trial(model, True, d, F, mode, dur=SUS_DUR,
                                    check_progress=False)
                if ok:
                    best, skid_at_best = F, round(skid)
                else:
                    break
            row[mode] = dict(force=best, skid_mm=skid_at_best)
        out[int(d)] = row
        print(f"  sustained dir {d:3d}: base {row['base']['force']:3.0f} N "
              f"(skid {row['base']['skid_mm']} mm)  v2 "
              f"{row['v2']['force']:3.0f} N (skid {row['v2']['skid_mm']} mm)",
              flush=True)
    with open(os.path.join(HERE, "push_v2_sustained.json"), "w") as fp:
        json.dump(out, fp, indent=1)
    print("wrote push_v2_sustained.json")


def do_envelope(model):
    """All three modes, PHASE-ALIGNED (the reflex gait clock now starts at
    motion onset — the D022-era runs had the supervisor 0.625 cycles offset
    from the baseline they were compared to, so old per-direction deltas
    were phase-confounded; these envelopes are the clean measurement)."""
    out = {m: {} for m in ("base", "v1", "v2")}
    for d in DIRS:
        for m in out:
            out[m][int(d)] = climb(model, True, d, m)
        print(f"  dir {d:3d}: base {out['base'][int(d)]:3.0f}  "
              f"v1 {out['v1'][int(d)]:3.0f}  v2 {out['v2'][int(d)]:3.0f} N",
              flush=True)
    with open(os.path.join(HERE, "push_v2_envelope.json"), "w") as fp:
        json.dump(out, fp, indent=1)
    print("wrote push_v2_envelope.json")


def do_probe(model):
    gait_T = WaveGait().T
    probe = {}
    for frac in PROBE_FRACS:
        pt = T_PUSH + frac * gait_T
        row = {}
        for mode in ("base", "v1", "v2"):
            row[mode] = climb(model, True, 180, mode, push_t=pt)
        print(f"  +{frac:.3f}T: base {row['base']:3.0f}  v1 {row['v1']:3.0f}  "
              f"v2 {row['v2']:3.0f} N", flush=True)
        probe[f"{frac:.3f}"] = row
    with open(os.path.join(HERE, "push_v2_probe.json"), "w") as fp:
        json.dump(probe, fp, indent=1)
    print("wrote push_v2_probe.json")


def do_merge():
    with open(os.path.join(HERE, "push_v2_envelope.json")) as fp:
        env = json.load(fp)
    with open(os.path.join(HERE, "push_v2_probe.json")) as fp:
        probe = json.load(fp)
    base, v1, env2 = env["base"], env["v1"], env["v2"]
    res = {
        "harness": "run_push_reflex_v2.py — D017 shove harness, all modes "
                   "phase-aligned (supervisor clock starts at motion onset)",
        "gyro_trip": TRIP,
        "walk_envelope": {"base": base, "v1": v1, "v2": env2},
        "phase_probe_dir180": probe,
    }
    for tag, e in (("base", base), ("v1", v1), ("v2", env2)):
        vals = list(e.values())
        res[f"walk_{tag}_min"] = min(vals)
        res[f"walk_{tag}_mean"] = round(float(np.mean(vals)), 1)
    pmin = {m: min(row[m] for row in probe.values()) for m in ("base", "v1", "v2")}
    res["probe_min"] = pmin
    with open(os.path.join(HERE, "push_reflex_v2_results.json"), "w") as fp:
        json.dump(res, fp, indent=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(12.5, 5.2))
    ax = fig.add_subplot(1, 2, 1, projection="polar")
    for tag, e, c in (("base", base, "#999999"), ("v1", v1, "#c49ade"),
                      ("v2", env2, "#7b3fa0")):
        th = np.deg2rad([int(k) for k in e] + [int(next(iter(e)))])
        r = list(e.values()) + [next(iter(e.values()))]
        ax.plot(th, r, "-o", ms=3.5, lw=1.8, color=c, label=f"walk {tag}")
    ax.set_title("Walking push envelope (N)", pad=18)
    ax.set_ylim(0, 95)
    ax.legend(loc="lower left", bbox_to_anchor=(-0.12, -0.12), fontsize=8)
    ax2 = fig.add_subplot(1, 2, 2)
    fr = [float(k) for k in probe]
    w = 0.028
    for j, (m, c) in enumerate((("base", "#999999"), ("v1", "#c49ade"),
                                ("v2", "#7b3fa0"))):
        ax2.bar(np.array(fr) + (j - 1) * w, [probe[k][m] for k in probe],
                width=w, color=c, label=m)
    ax2.set_xlabel("push timing (gait cycles after nominal)")
    ax2.set_ylabel("survivable force (N)")
    ax2.set_title("Phase probe, weakest walking dir (180°)")
    ax2.legend()
    ax2.grid(axis="y", alpha=0.3)
    fig.suptitle("Push recovery: gait-phase-aware + contact-seeking brace (v2)")
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, "fig_push_reflex_v2.png"), dpi=120)
    print(json.dumps({k: v for k, v in res.items()
                      if k.startswith(("walk_base_", "walk_v1_", "walk_v2_",
                                       "probe_min"))}, indent=1))
    print("wrote push_reflex_v2_results.json + fig_push_reflex_v2.png")


def main():
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "pebble.xml"))
    if "--envelope" in sys.argv:
        do_envelope(model)
    elif "--probe" in sys.argv:
        do_probe(model)
    elif "--sustained" in sys.argv:
        do_sustained(model)
    elif "--merge" in sys.argv:
        do_merge()
    else:
        do_envelope(model)
        do_probe(model)
        do_merge()


if __name__ == "__main__":
    main()
