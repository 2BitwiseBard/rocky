#!/usr/bin/env python3
"""Evaluate a self-righting checkpoint (session 8; D052 conditions + system mode).

    MUJOCO_GL=egl python eval_recover.py runs/recover1/latest.pt --episodes 20
    python eval_recover.py CKPT --randomize          # + a servo-random + DR + obs-noise column
    python eval_recover.py CKPT --servo nominal      # force the servo model on (legacy checkpoints)
    python eval_recover.py CKPT --supervisor         # the SYSTEM number (see below)
    python eval_recover.py CKPT --video out.mp4

Every checkpoint is replayed under its OWN contract (rl_common.checkpoint_contract):
obs version, action filter, rate limit. Pre-D052 checkpoints have no
env_config and replay as 'legacy obs' (obs v1, ideal servo, no filter,
5 rad/s — flagged 'exceeds servo': the ST3215 does 4.7 at no load).

Columns (conditions):
  nominal   the checkpoint's training servo made deterministic (off stays
            off; nominal/random -> the datasheet servo), no DR, no obs noise
  --servo X override the nominal column's servo (off | nominal | random)
  random    (--randomize) servo random + DR + obs noise: what training saw

Two numbers per column:
  stood (hybrid)   the policy rights the body; once the HANDOFF criterion has
                   held 0.5 s the analytic planted pose takes over (0.6 s ramp +
                   1.5 s hold, through the same servo path); stood = tilt < 15,
                   h > 0.11, >= 4 floor switches at the end. D052: the handoff
                   criterion is rocky_recover_env.handoff_ok (tilt + >= 3
                   switches + kinematic height) — --handoff legacy restores the
                   privileged tilt < 25 & h > 0.09 test.
  pure-RL success  the env's own success flag.
--supervisor instead runs each of the same seeds' falls through the REAL
stack for 14 s of raw MuJoCo: ReflexSupervisor (params reflex defaults:
stall 3 s, deadline 10 s, ramp 0.6 s) + PolicyRighter, fall detection
included. 'stood' there is the number to quote for the system; the hybrid
number above is policy-to-handoff only.
"""
import argparse
import os
import sys
import warnings

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "gait"))
import rl_common as rc                                     # noqa: E402
from rocky_recover_env import RecoverEnv, HANDOFF_HOLD_S, handoff_ok, CTRL_DT   # noqa: E402

SUPERVISOR_S = 14.0            # >= 13 s: 1 s fall confirm + 10 s deadline + ramp/hold + margin


def load_full(ckpt):
    """(policy fn, checkpoint dict, contract). The policy is the actor MEAN."""
    import torch
    from train_ppo import Agent, RunningMeanStd
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    contract = rc.checkpoint_contract(ck, env_hint="recover")
    obs_dim, act_dim = int(ck.get("obs_dim", contract["obs_dim"])), int(ck.get("act_dim", 15))
    agent = Agent(obs_dim, act_dim)
    agent.load_state_dict(ck["model"])
    agent.eval()
    rms = RunningMeanStd((obs_dim,))
    rms.load_state_dict(ck["obs_rms"])

    def policy(obs):
        on = (np.asarray(obs, np.float64) - rms.mean) / np.sqrt(rms.var + 1e-8)
        with torch.no_grad():
            return agent.actor(torch.as_tensor(np.clip(on, -10, 10), dtype=torch.float32)
                               .unsqueeze(0)).squeeze(0).numpy()
    return policy, ck, contract


def load(ckpt):
    """(policy, global_step) — the pre-D052 API, kept for callers."""
    policy, ck, _ = load_full(ckpt)
    return policy, ck.get("global_step", 0)


def ckpt_args(ckpt):
    """The trainer's argument dict as saved in the checkpoint ({} if absent)."""
    import torch
    ck = torch.load(ckpt, map_location="cpu", weights_only=False)
    return dict(ck.get("args") or {})


def env_for(contract, condition="nominal", servo=None, reward=None, seed=0):
    """A RecoverEnv that replays `contract` under an eval condition."""
    trained = contract.get("servo", "off")
    if condition == "random":
        servo_mode, randomize = "random", True
    else:
        servo_mode = servo or ("off" if trained == "off" else "nominal")
        randomize = False
    rate = contract.get("rate_limit_rad_s", rc.LEGACY_RATE_LIMIT_RAD_S)
    return RecoverEnv(seed=seed, reward=reward or contract.get("reward", "v1"),
                      rate_limit_rad_s=rate, servo=servo_mode, randomize=randomize,
                      ema_alpha=contract.get("ema_alpha", 1.0),
                      obs_version=contract["obs_version"])


def _planted_q0():
    from pebble_gait import WaveGait, leg_ik, body_to_leg, N_LEGS
    import rocky_model as rm
    g = WaveGait(**rm.gait_defaults())
    return np.array([leg_ik(body_to_leg(i, g.p_nom[i])) for i in range(N_LEGS)]).flatten()


def _stood(env):
    _g, _w, tilt, h = env._state()
    feet = int(env.true_feet().sum())
    return bool(np.degrees(tilt) < 15 and h > 0.11 and feet >= 4), float(np.degrees(tilt)), h, feet


def run(env, policy, seed, record=None, handoff=True, criterion="hw"):
    """Hybrid evaluation (see module doc). criterion: 'hw' = handoff_ok,
    'legacy' = tilt < 25 & h > 0.09 (privileged)."""
    obs, _ = env.reset(seed=seed)
    mode = env.last_mode
    done, ret, frames = False, 0.0, []
    best_tilt, ok_hold, stood, t_handoff = 180.0, 0.0, False, None
    q0 = _planted_q0()
    n = 0
    info = {}
    while not done:
        a = policy(obs)
        obs, r, term, trunc, info = env.step(a)
        n += 1
        ret += r
        best_tilt = min(best_tilt, info["tilt_deg"])
        done = term or trunc
        if record is not None:
            frames.append(env.render())
        if criterion == "legacy":
            ok = info["tilt_deg"] < 25 and info["height"] > 0.09
        else:
            ok = handoff_ok(info["tilt_deg"], env.true_feet(), env.true_q())
        ok_hold = ok_hold + CTRL_DT if (handoff and ok) else 0.0
        if handoff and ok_hold >= HANDOFF_HOLD_S - 1e-9:
            t_handoff = n * CTRL_DT
            q_start = env.true_q()                      # as before D052: ramp from where the joints ARE
            for k in range(105):                        # 0.6 s ramp + 1.5 s hold
                a_ = min(1.0, k / 30)
                env.hold_step((1 - a_) * q_start + a_ * q0)
                if record is not None:
                    frames.append(env.render())
            stood, tilt, h, feet = _stood(env)
            info = dict(info, tilt_deg=tilt, height=h, feet=feet)
            break
    return dict(seed=seed, mode=mode, success=bool(info.get("success", False)), stood=stood,
                t_handoff=t_handoff, ret=ret, end_tilt=info["tilt_deg"], end_h=info["height"],
                best_tilt=best_tilt, frames=frames)


def run_supervisor(env, policy, contract, seed, seconds=SUPERVISOR_S):
    """The system: the fallen pose from env.reset(seed), then raw MuJoCo under
    ReflexSupervisor + PolicyRighter (fall detection, stall rule, deadline,
    ramp) through the env's servo path. Zero velocity command throughout.
    policy=None: no righter installed (the supervisor holds pose in FALLEN
    until the stall rule / deadline ramps) — the analytic-only baseline the
    policy has to beat. t_stood = first time the stood test passed."""
    import mujoco
    import rocky_model as rm
    from pebble_gait import WaveGait
    from pebble_reflex import ReflexSupervisor, FALLEN, RIGHTED, body_gyro_xy
    from righter import PolicyRighter, foot_contacts
    from sim_imu import grav_body, gyro_body, tilt_from_grav
    env.reset(seed=seed)
    mode = env.last_mode
    m, d, torso = env.model, env.data, env.torso
    fids = env._foot_gid
    righter = None
    if policy is not None:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            righter = PolicyRighter(None, m, d, torso, fids, policy=policy, contract=contract)
    sup = ReflexSupervisor(WaveGait(**rm.gait_defaults()), righter=righter, **rm.reflex_defaults())
    DT = m.opt.timestep
    jadr = env._jadr
    prev, t_fallen, t_righted, reason, best, t_stood = None, None, None, None, 180.0, None
    every = int(round(CTRL_DT / DT))
    for k in range(int(seconds / DT)):
        t = k * DT
        if t_stood is None and k % every == 0 and _stood(env)[0]:
            t_stood = t
        g = grav_body(m, d, torso)
        w = gyro_body(m, d, torso)
        tilt = float(np.degrees(tilt_from_grav(g)))
        best = min(best, tilt)
        # D052 V2: q_meas as well — the supervisor then hands off on handoff_ok (tilt +
        # switches + the joints' kinematic height), the criterion the robot and the
        # cockpit use; without it this eval measured the old world-z handoff
        q, state = sup.step(t, 0.0, 0.0, 0.0, body_gyro_xy(w), contacts=foot_contacts(m, d, fids),
                            gyro_vec=w[:2], tilt_deg=tilt, height=float(d.xpos[torso][2]),
                            q_meas=d.qpos[jadr].reshape(5, 3))
        if state == FALLEN and prev != FALLEN and t_fallen is None:
            t_fallen = t
        if state == RIGHTED and prev == FALLEN and t_righted is None:
            t_righted, reason = t, sup.right_reason
        prev = state
        tgt = env.servo.filter(np.asarray(q, float).flatten(), DT, force=d.actuator_force[:15])
        d.ctrl[:15] = tgt + env._q_offset
        d.ctrl[15:20] = 0.0
        mujoco.mj_step(m, d)
    env._feet_state[:] = False
    env.true_feet()
    stood, tilt, h, feet = _stood(env)
    return dict(seed=seed, mode=mode, stood=stood, fallen=t_fallen is not None, reason=reason,
                t_stood=None if t_stood is None else round(t_stood, 2),
                t_right=None if t_righted is None or t_fallen is None else round(t_righted - t_fallen, 2),
                end_tilt=tilt, end_h=h, best_tilt=best, state=prev)


def header(ckpt, ck, contract, model):
    fp = rc.check_fingerprint(contract, model, who=ckpt)
    flags = ", ".join(contract["flags"]) or "D052 contract"
    return (f"checkpoint {ckpt} @ {ck.get('global_step', 0):,} steps | env {contract['env']} "
            f"obs v{contract['obs_version']} ({contract['obs_dim']}) | reward {contract.get('reward')} | "
            f"rate limit {contract.get('rate_limit_rad_s')} rad/s | ema {contract.get('ema_alpha')} | "
            f"trained servo {contract.get('servo')} | robot fingerprint {fp} | [{flags}]")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("ckpt")
    p.add_argument("--episodes", type=int, default=20)
    p.add_argument("--video", type=str, default="")
    p.add_argument("--reward", type=str, default=None, choices=["v1", "v2", "v3"],
                   help="env success criterion for the pure-RL column "
                        "(stood-after-handoff is criterion-agnostic)")
    p.add_argument("--servo", type=str, default=None, choices=list(rc.SERVO_MODES),
                   help="override the nominal column's servo model")
    p.add_argument("--randomize", action="store_true",
                   help="add a servo-random + DR + obs-noise column")
    p.add_argument("--handoff", type=str, default="hw", choices=["hw", "legacy"],
                   help="hw = handoff_ok (tilt + switches + kinematic height); "
                        "legacy = privileged tilt < 25 & h > 0.09")
    p.add_argument("--supervisor", action="store_true",
                   help="run the falls through ReflexSupervisor + PolicyRighter (system number)")
    p.add_argument("--no-baselines", action="store_true")
    args = p.parse_args()
    policy, ck, contract = load_full(args.ckpt)
    conds = [("nominal", args.servo)] + ([("random", None)] if args.randomize else [])
    first = True
    for cond, servo in conds:
        env = env_for(contract, cond, servo=servo, reward=args.reward)
        if first:
            print(header(args.ckpt, ck, contract, env.model))
            first = False
        tag = f"{cond} (servo {env.servo_mode}{', DR + obs noise' if env.randomize else ''})"
        if args.supervisor:
            runs = [("policy", policy)] + ([] if args.no_baselines else [("no-righter", None)])
            for name, pol in runs:
                rs = [run_supervisor(env, pol, contract, seed=s) for s in range(args.episodes)]
                reasons = [r["reason"] for r in rs if r["reason"]]
                ts = [r["t_stood"] for r in rs if r["t_stood"] is not None]
                print(f"[{tag}] SUPERVISOR {SUPERVISOR_S:.0f} s, {name:10s}: stood {sum(r['stood'] for r in rs)}/{len(rs)}  "
                      f"FALLEN declared {sum(r['fallen'] for r in rs)}/{len(rs)}  "
                      f"exits: handoff {reasons.count('handoff')} stall {reasons.count('stall')} "
                      f"deadline {reasons.count('deadline')}  "
                      f"t_stood median {np.median(ts) if ts else float('nan'):.2f} s")
                print("    " + "  ".join(f"{m_}: {sum(r['stood'] for r in rs if r['mode'] == m_)}/"
                                         f"{sum(r['mode'] == m_ for r in rs)}" for m_ in ("back", "side", "tumble")))
            continue
        pols = [("policy", policy)]
        if not args.no_baselines and cond == "nominal":
            pols += [("hold-pose", lambda o: env._a_f),
                     ("random", lambda o: env.rng.uniform(-1, 1, 15))]
        results = {}
        for name, pol in pols:
            rs = [run(env, pol, seed=s, criterion=args.handoff) for s in range(args.episodes)]
            results[name] = rs
            succ = sum(r["success"] for r in rs)
            stood = sum(r["stood"] for r in rs)
            print(f"[{tag}] {name:10s} stood-after-handoff {stood}/{len(rs)}  pure-RL success {succ}/{len(rs)}  "
                  f"mean return {np.mean([r['ret'] for r in rs]):6.1f}  "
                  f"end tilt median {np.median([r['end_tilt'] for r in rs]):5.1f}°  "
                  f"best tilt median {np.median([r['best_tilt'] for r in rs]):5.1f}°  "
                  f"end height median {np.median([r['end_h'] for r in rs])*1000:5.1f} mm")
        rs = results["policy"]
        for mode in ("back", "side", "tumble"):
            sub = [r for r in rs if r["mode"] == mode]
            if sub:
                print(f"    {mode:7s}: {sum(r['stood'] for r in sub)}/{len(sub)} stood, "
                      f"end tilt median {np.median([r['end_tilt'] for r in sub]):5.1f}°")
        if args.video and cond == "nominal":
            import imageio
            picks = []
            for want in (True, False):
                for r in rs:
                    if r["stood"] == want:
                        picks.append(r["seed"])
                        break
            frames = []
            for s in picks:
                frames += run(env, policy, seed=s, record=True, criterion=args.handoff)["frames"]
            if frames:
                imageio.mimsave(args.video, frames[::2], fps=25, codec="libx264", quality=7)
                print("wrote", args.video, f"({len(frames)//2} frames, seeds {picks})")


if __name__ == "__main__":
    main()
