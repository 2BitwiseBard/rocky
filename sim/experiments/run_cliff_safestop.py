"""The cliff safe-stop: VOID → retreat → PLANT→BRACE→RECOVER → planted idle.

D025 demoted the push reflex to a safe-stop primitive and said it "pairs
with the cliff detector"; D034 adopted `ReflexSupervisor.request_stop()` as
the halt after this harness measured it against the old path, a bare v = 0
command, which then left the wave gait MARCHING IN PLACE at the edge (24
contact breaks in the 4 s after the halt, feet lifting one at a time next to
a void).

Since D063 a zero command is the standing pose (the lift fades in over the
first 15 mm/s, WaveGait.lift_scale), so the bare halt no longer marches and
"fewer contact breaks than the bare halt" can no longer tell the two apart
(both are 0). B112: the safe-stop is judged on its own terms, in the run
where the supervisor owns the joints (judge()):

  detector    the VOID detector still fires while the supervisor owns ctrl
  upright     no fall (tilt > 50° or the torso below the platform)
  on the deck all five feet in contact at the end, each foot's far side short
              of the edge
  delivered   the stop passes through BRACE and ends back in NORMAL: the
              planted idle stance, the supervisor ready to walk again
  planted     no foot leaves the ground after the halt (the march-in-place
              signature, and any other lift)
  still       the torso moves < STILL_MM_S (1 mm/s) over the last second of
              the 4 s window: stopped, not creeping or sliding

The bare v = 0 halt runs as the reference (the same world, detector and
retreat) and is judged by the same checks, reported but not required.
`--retreat-cycles 0` is the negative control: the halt lands where the
void was felt (it writes out/cliff_safestop_results_retreat0.json, git-ignored,
not the record).

The void retreat reaches the gait unslewed (step(direct=True)), as in the
harness's goto (harness/sim_backend.py), so both halts back away alike;
before B112 this script slewed it, and on the D063 model the supervisor
path then stopped 43 mm nearer the edge (185.4 against 228.8 mm). Detector
inputs come from the SUPERVISOR's gait clock (it starts at motion onset,
like the bare gait's).

Usage: MUJOCO_GL=egl .venv/bin/python sim/experiments/run_cliff_safestop.py
           [--video] [--retreat-cycles N]
"""
import json
import sys

import mujoco
import numpy as np

import exp_paths as X                   # sys.path (sim/, gait/, perception/, audio/) + where results / clips go
from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS        # noqa: E402
import rocky_model as rm                                             # noqa: E402
from pebble_reflex import ReflexSupervisor, body_gyro_xy             # noqa: E402
from cliff import CliffDetector, CliffReaction                        # noqa: E402
from scenes import build_world, foot_contacts, walk_ask, EDGE_X, PLAT_H   # noqa: E402

T_TOTAL = 16.0
T_SETTLE = 1.0
T_AFTER_HALT = 4.0
T_STILL = 1.0            # s: the last part of the window the stillness check reads
STILL_MM_S = 1.0         # mm/s: mean torso speed that counts as stopped
RETREAT_CYCLES = 1.6     # CliffReaction's default (gait cycles backing away after a VOID)


def gyro_of(model, data, torso):
    R = data.xmat[torso].reshape(3, 3)
    w_body = R.T @ data.cvel[torso][0:3]
    return body_gyro_xy(w_body), w_body[:2]


def feet_state(model, data):
    """(n feet in contact, leading foot margin mm): the margin is the edge
    minus the far side of the foot nearest it (positive = on the deck)."""
    con = foot_contacts(model, data)
    far = [data.geom_xpos[g][0] + model.geom_size[g][0]
           for g in (model.geom(f"foot{i}").id for i in range(N_LEGS))]
    return int(con.sum()), round((EDGE_X - max(far)) * 1000, 1)


def run(mode, record=None, retreat_cycles=RETREAT_CYCLES):
    """mode: 'hardstop' (bare v = 0 halt) or 'safestop' (request_stop)."""
    model = build_world()
    gait = WaveGait()
    data = mujoco.MjData(model)
    q0 = np.array([leg_ik(body_to_leg(i, gait.p_nom[i]))
                   for i in range(N_LEGS)]).flatten()
    jadr = [model.joint(f"{n}{i}").qposadr[0]
            for i in range(5) for n in ("yaw", "hip", "knee")]
    data.qpos[0:3] = [0, 0, rm.spawn_z_m(gait.h, platform_z_m=PLAT_H)]    # D052: was h + 14 mm
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[jadr] = q0
    data.ctrl[:15] = q0
    mujoco.mj_forward(model, data)
    torso = model.body("torso").id
    DT = model.opt.timestep
    det = CliffDetector()
    react = CliffReaction(gait, retreat_cycles=retreat_cycles)
    sup = ReflexSupervisor(gait) if mode == "safestop" else None
    ask = walk_ask(gait)[0]       # D063: scenes.V_X fitted into the envelope (45 asks 34.2)

    fell = False
    max_edge_reach = -1.0
    halt_t = None                 # when v first commanded 0 / stop requested
    stop_x = None
    # post-halt metrics
    contact_breaks = 0
    prev_con = None
    speeds, tilts = [], []
    transitions = []              # (tw, state): the supervisor's state changes

    for k in range(int(T_TOTAL / DT)):
        t = k * DT
        tw = t - T_SETTLE
        if t < T_SETTLE:
            data.ctrl[:15] = q0
        else:
            vx0 = ask * min(tw / 0.6, 1.0)
            vx, vy, wz = react.command(tw, vx0, 0.0, 0.0)
            halted = react.retreat_until is not None and tw >= react.retreat_until
            if halted and halt_t is None:
                halt_t = tw
                if sup is not None:
                    sup.request_stop()
            if sup is not None:
                gxy, gvec = gyro_of(model, data, torso)
                con = foot_contacts(model, data)
                q, state = sup.step(tw, vx, vy, wz, gxy, contacts=con,
                                    gyro_vec=gvec,
                                    direct=react.retreat_until is not None)
                data.ctrl[:15] = q.flatten()
                if not transitions or transitions[-1][1] != state:
                    transitions.append((tw, state))
                # detector inputs from the SUPERVISOR's commanded stance
                if k % 10 == 0 and state == "NORMAL" and \
                        abs(vx) + abs(vy) > 1e-6:
                    ph = [(sup.t_gait / gait.T + gait.phase_off[i]) % 1.0
                          for i in range(5)]
                    settled = [ph[i] < gait.duty and
                               0.12 < ph[i] / gait.duty < 0.95
                               for i in range(5)]
                    if det.update(tw, settled, con):
                        react.on_void(tw)
            else:
                q, stance, _ = gait.joint_targets(tw, vx, vy, wz)
                data.ctrl[:15] = q.flatten()
                if k % 10 == 0:
                    con = foot_contacts(model, data)
                    ph = [(tw / gait.T + gait.phase_off[i]) % 1.0
                          for i in range(5)]
                    settled = [stance[i] and 0.12 < ph[i] / gait.duty < 0.95
                               for i in range(5)]
                    if det.update(tw, settled, con):
                        react.on_void(tw)
        mujoco.mj_step(model, data)
        if record is not None and k % record[0] == 0:
            record[1](data, t)
        x = data.xpos[torso][0]
        max_edge_reach = max(max_edge_reach, x - EDGE_X)
        z = data.xmat[torso].reshape(3, 3)[:, 2]
        tilt = np.rad2deg(np.arccos(np.clip(z[2], -1, 1)))
        if tilt > 50 or data.xpos[torso][2] < PLAT_H - 0.02:
            fell = True
            break
        # post-halt observation window
        if halt_t is not None and tw > halt_t + 0.05:
            if k % 10 == 0:
                con = foot_contacts(model, data)
                if prev_con is not None:
                    contact_breaks += int(np.sum(prev_con & ~con))
                prev_con = con
                speeds.append((tw - halt_t,
                               float(np.linalg.norm(data.cvel[torso][3:6]))))
                tilts.append(float(tilt))
            if tw > halt_t + T_AFTER_HALT:
                stop_x = x
                break

    n_feet, foot_margin = feet_state(model, data)
    late = [s for (ts, s) in speeds if ts >= T_AFTER_HALT - T_STILL]
    after = [s for (ts, s) in transitions if halt_t is not None and ts >= halt_t - 1e-9]
    return dict(
        mode=mode, retreat_cycles=retreat_cycles, fell=bool(fell),
        body_overhang_mm=round(max_edge_reach * 1000, 1),
        voids=det.events,
        halt_s=None if halt_t is None else round(halt_t, 2),
        stop_margin_mm=None if stop_x is None
        else round((EDGE_X - stop_x) * 1000, 1),
        end=dict(feet_in_contact=n_feet, foot_margin_mm=foot_margin),
        post_halt=dict(
            contact_breaks=contact_breaks,
            mean_speed_mms=None if not speeds
            else round(1000 * float(np.mean([s for _, s in speeds])), 1),
            still_speed_mms=None if not late
            else round(1000 * float(np.mean(late)), 2),
            max_tilt_deg=None if not tilts else round(max(tilts), 2),
        ),
        supervisor_states=after if sup is not None else None,
        transitions=[(round(ts, 2), s) for ts, s in transitions] if sup is not None else None,
    )


def judge(r, supervisor=True):
    """The safe-stop's checks on one run: {name: (ok, what was measured)}.
    supervisor=False drops `delivered` (the bare halt has no supervisor)."""
    ph, end = r["post_halt"], r["end"]
    states = r.get("supervisor_states") or []
    stopped = r["stop_margin_mm"] is not None     # the 4 s window after the halt completed
    checks = dict(
        detector=(bool(r["voids"]), f"{len(r['voids'])} VOID events"),
        upright=(not r["fell"] and stopped,
                 "fell" if r["fell"] else ("no halt" if not stopped else "upright")),
        on_the_deck=(end["feet_in_contact"] == N_LEGS and end["foot_margin_mm"] > 0,
                     f"{end['feet_in_contact']}/{N_LEGS} feet down, leading foot "
                     f"{end['foot_margin_mm']} mm short of the edge"),
        planted=(stopped and ph["contact_breaks"] == 0,
                 f"{ph['contact_breaks']} contact breaks after the halt"),
        still=(ph["still_speed_mms"] is not None and ph["still_speed_mms"] < STILL_MM_S,
               f"{ph['still_speed_mms']} mm/s over the last {T_STILL:.0f} s"),
    )
    if supervisor:
        checks["delivered"] = ("BRACE" in states and bool(states) and states[-1] == "NORMAL",
                               " → ".join(states) or "no state after the halt")
    return checks


def main():
    rc = RETREAT_CYCLES
    if "--retreat-cycles" in sys.argv:
        rc = float(sys.argv[sys.argv.index("--retreat-cycles") + 1])
    if "--video" in sys.argv:
        import imageio
        frames = []
        model = build_world()
        renderer = mujoco.Renderer(model, 480, 720)
        cam = mujoco.MjvCamera()
        cam.distance, cam.elevation, cam.azimuth = 0.9, -14, 135

        def grab(data, t):
            cam.lookat[:] = data.xpos[model.body("torso").id]
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
        spf = int(round(1 / (30 * model.opt.timestep)))
        run("safestop", record=(spf, grab), retreat_cycles=rc)
        out = X.out("pebble_cliff_safestop.mp4")
        imageio.mimsave(out, frames, fps=30, codec="libx264", quality=8)
        print("video:", out)
        return
    old = run("hardstop", retreat_cycles=rc)
    new = run("safestop", retreat_cycles=rc)
    verdicts = {}
    for r, sup in ((old, False), (new, True)):
        ph, end = r["post_halt"], r["end"]
        print(f"{r['mode']:9s} fell={r['fell']} overhang {r['body_overhang_mm']} mm, "
              f"stop margin {r['stop_margin_mm']} mm, leading foot "
              f"{end['foot_margin_mm']} mm, {end['feet_in_contact']}/5 feet down | "
              f"post-halt: {ph['contact_breaks']} contact breaks, mean speed "
              f"{ph['mean_speed_mms']} mm/s (last {T_STILL:.0f} s {ph['still_speed_mms']}), "
              f"max tilt {ph['max_tilt_deg']}°")
        checks = judge(r, supervisor=sup)
        verdicts[r["mode"]] = {k: dict(ok=bool(ok), measured=m) for k, (ok, m) in checks.items()}
        for name, (ok, m) in checks.items():
            print(f"    {'ok  ' if ok else 'FAIL'} {name:12s} {m}")
    ok = all(v["ok"] for v in verdicts["safestop"].values())
    # a control run leaves the record alone: it goes to the git-ignored out/
    path = (X.result("cliff_safestop_results.json") if rc == RETREAT_CYCLES
            else X.out(f"cliff_safestop_results_retreat{rc:g}.json"))
    with open(path, "w") as f:
        json.dump(dict(hardstop=old, safestop=new, verdicts=verdicts,
                       safestop_pass=ok, still_mm_s=STILL_MM_S), f, indent=1)
    failed = [k for k, v in verdicts["safestop"].items() if not v["ok"]]
    print("SAFE-STOP INTO CLIFF PATH " +
          ("PASS — the detector fires under the supervisor, and the stop ends "
           "planted, still and on the deck, through BRACE back to NORMAL"
           if ok else "FAIL — " + ", ".join(failed)))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
