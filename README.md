# PROJECT ROCKY — Pebble

A 1:3-scale pentapod walking robot: CAD that prints, a MuJoCo sim that
walks, gestures and gets itself back up, and a tool harness that lets a
person or a language model drive it by text, voice or MCP.

The physical build is not assembled yet. Everything green below is green
in simulation or in CAD checks, which the project treats as a different
claim from "it works on hardware".

## Layout

| Path | What it is |
|---|---|
| `rocky_v0.8.0_code/rocky/` | The live tree. Everything below is relative to it. |
| `rocky_v0.8.0_assets_cad/` | CAD export drop. `cad/out` in the live tree is a symlink into here. |
| `rocky_v0.8.0_assets_av_s8d/` | Audio and video drop. |
| `patches/` | Drop-vs-local record: the 2026-09-08 harness diff plus the pristine files it applies to. |
| `rocky.sh` | Launcher for this laptop. Not part of any drop. |

Inside the live tree: `cad/` parametric parts and the check suite ·
`sim/` MuJoCo worlds, PPO training and evaluation · `gait/` the wave gait
and the reflex supervisor · `driver/` servo bus · `harness/` the tool
layer, MCP server, intent parser and local-LLM brain · `perception/`
lidar and vision · `bench/` hardware bring-up · `bom/` parts ·
`docs/` the design record.

## Driving it

`rocky.sh` wraps the common entry points:

```
./rocky.sh play        # MuJoCo viewer + WASD teleop
./rocky.sh chat        # Claude Code over MCP, viewer and speakers on
./rocky.sh brain       # a local model as the robot's brain
./rocky.sh voice       # push-to-talk through whisper-server into intent.py
./rocky.sh test        # harness, gait and driver suites
./rocky.sh cad-check   # the full CAD check tree
```

## Ground rules

These are the ones that have actually cost something when broken.

- **`params.yaml` is the single source of truth** for dimensions. Parts
  are regenerated from it, never edited downstream.
- **Checks green before delivery.** `cad/run_all_checks.py` is 21/21 and
  takes about 103 seconds on this machine.
- **Sim honesty.** A negative result is a result and gets written down.
  Three separate recovery retrains scored worse than the policy they
  were meant to beat, and that is recorded rather than buried.
- **The SCS0009 servo never sees 12 V.**

## Where the record lives

- `BUILD_LOG.md` — engineering notebook, newest entry first.
- `docs/decisions.md` — numbered decisions, the D-numbers cited elsewhere.
- `NOTES_INBOX.md` — raw dump of measurements and results, filed later.
- `docs/NEXT_SESSION.md` — what the next session should pick up.
- `docs/RL_TOUR.md` — the reinforcement-learning ladder and what each
  rung was worth.

## Current state (2026-09-08)

CAD is print-clean and waiting on caliper measurements before the final
regen. The sim robot walks, gestures, self-rights after a shove, patrols
with lidar, and answers to text, a local model, or Claude over MCP. The
servo order has not been placed, so there is no bench day yet.

The shipped self-righting policy is `recover1`, which stands up 10 times
in 20 on this machine. Two later attempts to improve it scored 7 and 3.
