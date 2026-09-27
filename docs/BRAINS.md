# Brains

Which models drive Pebble in the cockpit, how to add one and how to
measure it. The model ids here are the **reference setup's** (D055): one
laptop with a 16 GB GPU running llama-swap on a pinned llama.cpp build.
Any OpenAI-compatible server works
(`ROCKY_LLM_BASE_URL`), and every id below is a default that `rocky.env`
overrides (`rocky.env.example` lists the knobs). The full 2026-09-24
research round behind the picks, with 22 candidates and their install
recipes, is archived in [archive/BRAIN_MODELS_2026-09-24.md](archive/BRAIN_MODELS_2026-09-24.md).

## Roles

| role | what it does | cockpit mode | reference default, then fallbacks | set with |
|---|---|---|---|---|
| multimodal | sees the eye frame with every message and calls tools; one model, one load | Multimodal | `qwen3.5-9b` → `qwen3.5-4b` → `gemma-4-12b` | `ROCKY_MULTIMODAL_MODEL`, `ROCKY_FALLBACK_MULTIMODAL` |
| brain | text + tools, beside a separate eye | Local model | `tool-model` (a llama-swap selector for `qwen3.6-35b-a3b`) → `qwen3.6-35b-a3b` → `qwen3.8-27b-iq4` | `ROCKY_BRAIN_MODEL`, `ROCKY_FALLBACK_BRAIN` |
| vision | describes the eye for `look` and `find_object`, in every mode | — | `vision-model` (a selector for `lfm2.5-vl`) → `lfm2.5-vl` → `gemma-4-12b` → `gemma-4-26b-a4b` | `ROCKY_VISION_MODEL`, `ROCKY_FALLBACK_VISION` |
| claude | the cloud brain | Claude | `claude-sonnet-5` (needs the `cockpit` extra and `ANTHROPIC_API_KEY`) | `ROCKY_CLAUDE_MODEL` |
| embedding | place recognition's description vectors (D057) | — | `embedding` (Qwen3-Embedding-0.6B, CPU) | `ROCKY_EMBED_MODEL` |

A pick in the cockpit's Talk tab is saved (`ROCKY_COCKPIT_CONF`) and wins
over the defaults. Around every model: the **stop gate** executes a stop
line before any model sees it (D055; `ROCKY_STOP_FIRST=0` turns it off,
which only the bench does); a **per-family prompt note** for ids starting
`gemma` or `qwen3.5` (D055a; `ROCKY_FAMILY_NOTES=0` off); thinking off
unless the id is in `ROCKY_THINKING_MODELS`; `ROCKY_QUARANTINED` ids are
never used; `ROCKY_CORESIDENT` names the brain + vision pair that fits the
GPU together, and the UI warns about any other pair (every look would swap
models). Box coordinates are read x-first except for the ids in
`BOX_Y_FIRST` (`sim/cockpit_brains.py`: `gemma-4-26b`, `gemma-4-31b`).

## Results on the reference machine (2026-09-25)

Measured by `brain_bench` on the obstacle-course world at 2× physics, in
the multimodal role: 20 spoken-style commands, 6 describe turns, and with
`--vision` the vision bench's box and floor cases. The bench cockpit runs
with the stop gate off, so *stop missed* shows what a model does with a
bare "stop" on its own. The 9B row is after the turn-discipline rules were
added to the prompt (before them: 16/20, p95 latency 16 s, describe 3/6).
The `+ family note` rows are D055a. `lfm2.5-vl` is listed to show it is not
a brain.

| model id | files | measured | VRAM (MiB) | first answer (s) | t/s | commands ok /20 | stop missed | unsafe | latency per command (s) | describe | vision (`--vision`) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `qwen3.5-9b` | Qwen3.5-9B-UD-Q4_K_XL (6.0 GB) + mmproj-Qwen3.5-9B-F16 (0.9 GB) | 2026-09-25, brain_bench (multimodal) | 6,997 at 16k context | 6.9 (cold load + first answer) | 62.8 decode (llama-server timings) | 17/20 | 0 | 1 | 3.6 median / 7.3 p95 wall; first action 0.9 / 3.2 | 5/6 (side 4/5) | seen 12/12, false+ 1/4, bearing err 0.9°, dist err 0.07 m, 1.40 s, floor 2/4 |
| `qwen3.5-9b` + family note (B41) | Qwen3.5-9B-UD-Q4_K_XL (6.0 GB) + mmproj-Qwen3.5-9B-F16 (0.9 GB) | 2026-09-25, brain_bench (multimodal) | 7,003 at 16k context | 29.1 (cold load + first answer); 26.1 without its 3.0 s of look, gesture, gesture | 62.0 decode (llama-server timings) | 19/20 | 0 | 1 | 2.9 median / 5.6 p95 wall; first action 0.9 / 1.9 | 5/6 (side 4/5) | not run |
| `lfm2.5-vl` | LiquidAI_LFM2.5-VL-3B-Q8_0 (2.9 GB) + mmproj-LiquidAI_LFM2.5-VL-3B-f16 (0.9 GB) | 2026-09-25, brain_bench (multimodal) | 4,591 at 64k context | 7.5 (cold load + first answer) | 127.8 decode (llama-server timings) | 5/20 | 2 | 2 | 0.2 median / 8.0 p95 wall; first action 0.6 / 1.1 | 4/6 (side 3/5) | seen 12/12, false+ 0/4, bearing err 0.9°, dist err 0.03 m, 0.42 s, floor 2/4 |
| `qwen3.5-4b` | Qwen3.5-4B-Q8_0 (4.5 GB) + mmproj-Qwen3.5-4B-F16 (0.7 GB) | 2026-09-25, brain_bench (multimodal) | 5,841 at 16k context | 10.1 (cold load + first answer) | 72.3 decode (llama-server timings) | 16/20 | 0 | 3 | 3.6 median / 18.8 p95 wall; first action 0.9 / 6.2 | 4/6 (side 3/5) | seen 12/12, false+ 0/4, bearing err 1.1°, dist err 0.12 m, 0.83 s, floor 3/4 |
| `gemma-4-12b` | gemma-4-12b-it-UD-Q6_K_XL (10.7 GB) + mmproj-gemma-4-12b-it-F16 (0.2 GB) | 2026-09-25, brain_bench (multimodal) | 11,325 at 16k context | 15.2 (cold load + first answer) | 33.1 decode (llama-server timings) | 17/20 | 1 | 0 | 4.7 median / 9.7 p95 wall; first action 1.4 / 2.4 | 5/6 (side 4/5) | seen 12/12, false+ 1/4, bearing err 0.7°, dist err 0.04 m, 1.71 s, floor 3/4 — re-measured after the box-order fix (its boxes are x-first; read y-first the first run got 21.9°) |
| `gemma-4-26b-a4b` | gemma-4-26B-A4B-it-UD-Q5_K_XL (21.3 GB) + mmproj-gemma-4-26B-A4B-F16 (1.2 GB) | 2026-09-25, brain_bench (multimodal) | 14,857 at 64k context | 41.9 (cold load + first answer) | 38.7 decode (llama-server timings) | 15/20 | 2 | 1 | 6.2 median / 32.1 p95 wall; first action 2.0 / 4.8 | 4/6 (side 3/5) | seen 12/12, false+ 0/4, bearing err 0.6°, dist err 0.07 m, 2.79 s, floor 3/4 |
| `gemma-4-12b` + family note (B41) | gemma-4-12b-it-UD-Q6_K_XL (10.7 GB) + mmproj-gemma-4-12b-it-F16 (0.2 GB) | 2026-09-25, brain_bench (multimodal) | 11,325 at 16k context | 24.9 (cold load + first answer) | 33.8 decode (llama-server timings) | 20/20 | 0 | 0 | 4.4 median / 13.7 p95 wall; first action 1.4 / 5.6 | 4/6 (side 3/5) | not run |
| `gemma-4-12b` + family note + thinking ON (B41) | gemma-4-12b-it-UD-Q6_K_XL (10.7 GB) + mmproj-gemma-4-12b-it-F16 (0.2 GB) | 2026-09-25, brain_bench (multimodal) | 11,325 at 16k context | 25.2 (cold load + first answer) | 33.3 decode (llama-server timings) | 16/20 | 0 | 0 | 13.2 median / 27.7 p95 wall; first action 4.4 / 20.8 | 2/6 (side 1/5) | not run |
| `qwen3.5-9b-q6k` | Qwen3.5-9B-UD-Q6_K_XL (8.8 GB) + mmproj-Qwen3.5-9B-F16 (0.9 GB) | 2026-09-25, brain_bench (multimodal) | 9,173 at 16k context | 13.7 (cold load + first answer) | 43.4 decode (llama-server timings) | 17/20 | 0 | 1 | 3.8 median / 7.1 p95 wall; first action 1.1 / 2.3 | 3/6 (side 2/5) | seen 12/12, false+ 0/4, bearing err 1.7°, dist err 0.07 m, 1.54 s, floor 3/4 |

**Verdict** (D055, D055a):
- **`qwen3.5-9b` (UD-Q4_K_XL) is the default all-in-one brain.** 17/20
  (19/20 with its family note), no missed stop, 0.9 s to the first action,
  7.0 GB, 63 t/s, sub-degree boxes. Nothing beat it on any axis that
  matters for driving. (It read "thirty centimeters" as x = 30 until the
  prompt said distances are metres, D054.)
- **`qwen3.5-4b` (Q8_0) is the small brain.** 16/20, no missed stop, the
  same 0.9 s first action, 5.8 GB, 72 t/s. Its misses were extra motions
  after the asked one, the class the one-motion rule and the `move` tool
  target. The robot-side candidate.
- **`gemma-4-12b` (UD-Q6_K_XL) is the accuracy option.** With its family
  note 20/20, no missed stop, no unasked motion; boxes 0.7°, 4 cm (read
  x-first); but 1.4 s to act and 11.3 GB. Without the note it answered a
  bare "stop" with a chord. One click away in the Talk tab.
- **Thinking on hurts**: the 12B with thinking drops to 16/20 (three lines
  answered in text with no tool), 13 s median latency, describe 2/6.
- **`qwen3.5-9b-q6k` gave nothing for its cost**: 17/20 like the Q4, 2.2 GB
  more VRAM, a third slower, describe no better.
- **`gemma-4-26b-a4b` is not a robot brain** (42 s cold, 32 s p95, 15/20,
  both stops missed), but its boxes are excellent (0.6°): a vision fallback.
- **`lfm2.5-vl` is the eye only** (5/20 as a brain, tool text in its
  replies, both stops missed; 0.9° boxes at 0.4 s).

n = 20 lines per run: a one-line difference is noise, three is not. Every
vision number is a clean MuJoCo render, a best case for a real camera.

## Adding and measuring a model

Two commands, from the repo root, once the download is complete:

```bash
./rocky.sh brain-install ~/Downloads/<file>.gguf --dry-run   # the plan; touches nothing
./rocky.sh brain-install ~/Downloads/<file>.gguf             # do it
./rocky.sh brain-bench --models <id> --vision                # <id> is the one brain-install printed
```

Without the launcher: `.venv/bin/python sim/brain_install.py …` and
`.venv/bin/python sim/brain_bench.py …`. The paths come from `rocky.env`:
`ROCKY_DOWNLOADS_DIR` (default `~/Downloads`; with no FILE, every finished
`*.gguf` there), `ROCKY_MODELS_DIR` (default `~/models`),
`ROCKY_SWAP_CONFIG` (default `~/.config/llama-swap/config.yaml`) and
`ROCKY_SWAP_MANIFEST` (the restore manifest beside it); each has a flag
(`--downloads`, `--models-dir`, `--config`, `--manifest`).

**brain-install** reads the GGUF header, derives the id from family + size
(Qwen3.5-4B → `qwen3.5-4b`; a second quant of a configured id gets a
suffix, `qwen3.5-9b-q6k`; `--id` overrides), moves the file to
`<models-dir>/<id>/` (copy, verify, then delete the download; `--copy`
keeps it), finds or fetches its vision projector (a hard link
when another id already holds the same one; `--no-fetch` never downloads),
adds a manifest entry, backs up the config and inserts an **UNMEASURED**
stanza (16k context, ttl 1800, the family's template rules). It never loads
a model and never edits `groups:`, `selectors:` or `macros:`. It expects a
llama-swap config that has `${binary}` and `${common_flags}` macros and a
comment block marked `RETRIEVAL STACK` (the stanza goes right before it);
any other setup adds the model by hand. It then restarts llama-swap, which
unloads **every** loaded model, the live cockpit's included
(`--no-restart` leaves that to you). A partial or still-growing download is
skipped; `--uninstall ID [--delete-files]` removes one it installed.
Conventions and safety rules: the `sim/brain_install.py` docstring.

**brain-bench** needs llama-swap too (its `/running` and unload routes). It
starts its own cockpit on :8792, never the live one, and for each model in
turn unloads every GPU model (the one under test included, so its load is
cold), waits `--idle` s (default 20) of true idle, sets the role
(`--mode multimodal`, or `--mode local` for the text brain), times the
cold first answer, reads the VRAM it added, measures decode speed, then
scores `--commands` lines (default 20), `--describe` turns (default 6) and
with `--vision` the box and floor cases, and unloads it (unless
`--keep-loaded`). **Leave the live cockpit alone while it runs**
([COCKPIT_GUIDE](COCKPIT_GUIDE.md), "Adding and measuring a model"). It
writes `sim/out/brain_bench.json` and prints a table plus a Markdown row in
the 12 columns above: paste the row into the results table.

With another OpenAI-compatible server neither tool applies: add the model
to that server, pick it in the Talk tab, and judge it against the rows
here.

## Reading a row

The column glossary is in [COCKPIT_GUIDE](COCKPIT_GUIDE.md) ("Reading a
bench row"). What each role needs:

| role | required | then compare |
|---|---|---|
| multimodal | stop missed 0, unsafe 0 | commands ok near 20; first-action median near the 9B's 0.9 s; describe on the right side |
| brain (`--mode local`) | stop missed 0, unsafe 0 | the same command numbers; no picture is needed |
| vision | — | seen rate, median bearing and distance error against `lfm2.5-vl`'s 12/12, 0.9° and 0.03 m at 0.4 s |

Compare decision speed with the **first action** column, never the latency
column: latency is the wall time of the whole chat turn and includes the
motion (a gesture runs to its end, a goto until the robot arrives), so a
model that gets "wave hello" exactly right still shows several seconds
there. The hand-timed tool-call bars are 0.2–0.5 s per command for
`lfm2.5-vl` and 0.2–1.5 s for `qwen3.5-9b`. **first answer** is what a swap
costs (after switching, a fallback, or the 30-minute ttl). **t/s** matters
for descriptions and chat; tool calls are short.

## Fitting the GPU (reference machine)

- The multimodal role runs **alone**: the 9B (7.0 GB), the 12B (11.3 GB)
  and the 35B driver (10.4 GB at its tuned offload) cannot be loaded
  together, and llama-swap evicts one to load another. Its VRAM only
  decides what else can run at the same time.
- The reference `resident` group, the text brain + eye pair
  (`ROCKY_CORESIDENT=qwen3.6-35b-a3b,lfm2.5-vl`), is measured at
  14,928 of 16,384 MiB (10,372 + 4,556). llama-swap does not check memory
  for a group, so a third member turns a swap into an out-of-memory crash at
  spawn: **a new id never goes in that group.**
- A brain small enough to sit beside the eye (its VRAM + 4,556 MiB, with
  headroom under 16,384) could share a new group with a copy of the eye's
  stanza; that is a decision after measuring, never the installer's.
- The installed `qwen3.5-4b` is a Q8_0: its weights and projector alone are
  4,916 MiB before any KV cache, above the eye's 4,556 MiB slot, so it
  cannot replace `lfm2.5-vl` in the pair. A UD-Q4_K_XL 4B might; measure it
  first.
- `brain-install` warns when weights + projector exceed
  `ROCKY_GPU_VRAM_GB` − 2 GB (16 on the reference machine).
- A vision model outside the resident pair swaps with the brain on every
  look (seconds to minutes each time).
