# Small brain models for Pebble — research 2026-09-24

*Written by an Opus research workflow (3 sweeps, 22 candidates, each adversarially verified against the Hugging Face page and the pinned llama.cpp build 36b10154). Nothing here was downloaded or measured on this machine when written; see the 'could not verify' section. Kept as the record behind the model choice.*

## Status 2026-09-25

The research below picked Qwen3.5-9B. It was downloaded and measured by hand on 2026-09-24 and is now the default multimodal role. On 2026-09-25 three more files finished downloading into `~/Downloads`: `Qwen3.5-4B-Q8_0.gguf` (4,482,403,488 B), `gemma-4-12b-it-UD-Q6_K_XL.gguf` (10,685,012,800 B) and `Qwen3.5-9B-UD-Q6_K_XL.gguf` (8,756,929,760 B). A brain-install dry run on copies of the config and manifest planned the ids `qwen3.5-4b`, `gemma-4-12b` and `qwen3.5-9b-q6k`. The 4B's and the 12B's mmproj files were already in their folders under `/mnt/models`; the 9B Q6 file gets a hard link of the 9B's mmproj. The installer and the bench below are how each one gets into llama-swap and gets its row. A pending row stays "not measured yet" until the bench has run on this machine.

### Measured

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

Rows measured by `brain_bench` 2026-09-25 on the obstacle-course world at 2x physics, multimodal role, 20 commands + 6 describe turns (+ the vision bench's detect and floor cases). The 9B row is the run after the turn-discipline rules were added to the brain prompt (before them: 16/20, p95 latency 16 s, describe 3/6). The 3B eye is listed only to show it is not a brain: in the multimodal role its tool calls leak into the reply and both stop lines were missed — keep it in the vision role.

Notes on the table:

- **The 9B row was measured by hand, not by the bench.** The commands, stop, unsafe and describe columns stay empty until `brain-bench --models qwen3.5-9b` runs. Its 57.7 t/s includes prefill; the bench's `decode_tps` comes from llama-server's own timings, so the two are not directly comparable.
- **The find numbers from the same day are not the 9B's.** 3/3 finds, 0.7° median bearing error and 3 cm distance error were measured with lfm2.5-vl and gemma-4-26b-a4b as the vision role (`docs/SIM_GUIDE.md`, the vision bench table). The 9B's `--vision` row has not been measured.
- **Known quirk (9B):** it read "thirty centimeters" as x = 30 until the system prompt stated that distances are metres (D054).
- **The second 9B file** is UD-Q6_K_XL, not a duplicate, so it gets its own id, `qwen3.5-9b-q6k`, and its own row. (A UD-Q4_K_XL file of the same size would have been a duplicate of `/mnt/models/qwen3.5-9b/Qwen3.5-9B-UD-Q4_K_XL.gguf`: brain-install reports "duplicate of …" and leaves such a download where it is, for you to delete. A Q8_0 would have become `qwen3.5-9b-q8`.)
- **The downloaded quants are bigger than the research's.** Sections B2 and C1 below estimate UD-Q4_K_XL files. The 12B came as UD-Q6_K_XL (10.7 GB of weights against B2's 7.4 GB) and the 4B as Q8_0 (4.5 GB against C1's 2.9 GB), so expect their VRAM well above those estimates.
- **Comparison bars:** lfm2.5-vl (the resident eye, 3B) makes tool calls in 0.2–0.5 s per command warm; the 9B took 0.2–1.5 s. Both are the model's tool-call time, measured by hand. Compare them with the bench's **first action** median and p95, not with its latency column: latency is the wall time of the whole `/api/chat` turn and includes the motion (a gesture runs to its end and a goto until the robot arrives), so a model that gets "wave hello" or "walk forward thirty centimeters" exactly right still shows several seconds there. A new model has to beat or match these bars on first action to be worth a role.

### Verdict 2026-09-25

Same harness, same prompt (with the turn-discipline rules), all in the multimodal role, one model on the GPU at a time.
The bench cockpit runs with `ROCKY_STOP_FIRST=0`, so the *stop missed* column shows what each model does with a bare
"stop" on its own; in normal operation the stop gate (`sim/cockpit_brains.py`, 2026-09-25) executes every stop line before
any model sees it, so a miss here is a mark against the model's tool eagerness, not a live hazard.

- **`qwen3.5-9b` (UD-Q4_K_XL) stays the default all-in-one brain.** 17/20, no missed stop, 0.9 s to the first action,
  7.0 GB, 63 t/s, sub-degree boxes. Nothing beat it on any axis that matters for driving.
- **`qwen3.5-4b` (Q8_0) is the small brain.** 16/20, no missed stop, the same 0.9 s first action, 5.8 GB, 72 t/s. Its
  misses were extra motions after the asked one, the class the one-motion rule and the `move` tool target. It cannot sit
  beside the 35B driver (5.8 + 10.4 GB), so it is a "robot-side" candidate, not a co-resident one.
- **`qwen3.5-9b-q6k` gave nothing for its cost**: 17/20 like the Q4, 2.2 GB more VRAM, a third slower, describe no better.
  Delete the file or keep it as a quality reserve; the fleet does not need it.
- **`gemma-4-12b` (UD-Q6_K_XL) is a fine eye and an adequate brain, not the pick.** Its boxes are as good as anyone's
  (0.7°, 4 cm) once read x-first, describe 5/6, 17/20 commands, but it answered a bare "stop" with a chord (the 26B did the
  same, twice), it is the slowest to act (1.4 s) and the biggest (11.3 GB). Keep it installed as the Gemma option; try it
  with thinking on and a Gemma-style prompt before judging the family (backlog B41).
- **`gemma-4-26b-a4b` is not a robot brain**: 42 s cold, 32 s p95, 15/20, both stops missed. Its boxes are excellent
  (0.6°), so it stays what it was, a vision fallback at the desk.
- **`lfm2.5-vl` stays the eye only** (5/20 as a brain, tool text in its replies, both stops missed; 0.9° boxes at 0.4 s).

Roles after this round: multimodal `qwen3.5-9b` → `qwen3.5-4b` → `gemma-4-12b`; vision `lfm2.5-vl` → `gemma-4-12b` →
`gemma-4-26b-a4b`; brain (text, beside the eye) `qwen3.6-35b-a3b` → `qwen3.8-27b-iq4` as before.

**B41 addendum (same evening): explicit numbered rules lift both small families; thinking hurts.** A per-family
prompt note (`sim/cockpit_brains.py` `FAMILY_NOTES`, on by default for ids starting `gemma` and `qwen3.5`; `ROCKY_FAMILY_NOTES=0`
turns it off) that says, in order: a stop line = call stop and nothing else; pick the ONE tool that does what was asked and call
it before writing; one sentence after the result; chords are for greetings and feelings. Measured with the same harness
(stop gate off in the bench so the model's own stop shows): `gemma-4-12b` 17/20 → **20/20**, 0 missed stops, 0 unasked
motion, first action 1.4 s; `qwen3.5-9b` 17/20 → **19/20**, 0 missed, median latency 3.6 → 2.9 s, p95 7.3 → 5.6 s, describe
5/6. Thinking ON for the 12B (`ROCKY_THINKING_MODELS=gemma-4-12b`, `enable_thinking` in its template): 16/20, three lines
answered in text with no tool, median latency 13 s, describe 2/6 — off stays the rule. Roles are unchanged: the 9B keeps the
default on latency and VRAM (0.9 s to act, 7 GB vs 1.4 s, 11.3 GB); Gemma 12B is the accuracy option for an operator who
prefers it, one click away in the Talk tab. n = 20 lines per run: a one-line difference is noise, three is not.

### Adding and measuring a model

Two commands, run from the repo root once the browser says the download is complete (a browser partial is named `Unconfirmed NNN.crdownload`; the installer skips it, and any file whose size is still changing):

```bash
./rocky.sh brain-install ~/Downloads/<file>.gguf          # add --dry-run first to see the plan
./rocky.sh brain-bench --models <id> --vision              # <id> is the one brain-install printed
```

The same tools without the launcher: `.venv/bin/python sim/brain_install.py …` and `.venv/bin/python sim/brain_bench.py …`. With no FILE, brain-install takes every finished `*.gguf` in `~/Downloads`. A GGUF whose architecture has no brain template (the flux image model in `~/Downloads`, an MTP draft head) is reported as skipped and left where it is.

**What brain-install does.** It reads the GGUF header (family `qwen35`, `gemma4` or `lfm2`, base name, size) and derives the id: Qwen3.5-4B → `qwen3.5-4b`, gemma-4-12b-it → `gemma-4-12b` (`--id` overrides). It copies the file to `/mnt/models/<id>/`, checks the size and the header there, then deletes the copy in Downloads (`--copy` keeps it). It never overwrites a file. For a vision family it uses, in this order: the mmproj already in that folder; a generic `mmproj-F16.gguf` from Downloads that names the same base model; a hard link of the one another id's folder already holds for the same base model (same bytes, no extra disk; this is how `qwen3.5-9b-q6k` gets the 9B's); or else it fetches `mmproj-F16.gguf` from the model's Hugging Face repo (`--no-fetch` stops that). The file ends up as `mmproj-<basename>-F16.gguf`. It adds a `models.manifest.json` entry, backs up `config.yaml` to `config.yaml.bak-YYYYMMDD-<id>`, and inserts a stanza before the RETRIEVAL STACK block. The stanza's name says "vision (mmproj)" so the cockpit offers it as a vision model; its description starts with "UNMEASURED"; it uses 16k context, ttl 1800, Qwen3.5 with thinking off, Gemma 4 with the 140–280 image-token rule. It never edits `groups:`, `selectors:` or `macros:`, and it never loads the model.

**brain-install restarts llama-swap.** Every loaded model unloads: the resident pair, anything Open WebUI is using, the live cockpit's brain. The next request to each starts cold. `--no-restart` installs now and leaves the restart to you (`systemctl --user restart llama-swap`); the new id is listed only after a restart.

**What brain-bench measures.** It starts its own cockpit on :8792 in the obstacle course and refuses the live one on :8765. For each model it unloads every loaded GPU model, the one under test included, so its load is cold (the CPU embedding and reranker models stay), and waits `--idle` seconds (default 20) of true idle, as the GPU rules require. Then it:

1. makes the model the role being tested (`--mode multimodal` for the multimodal role, `--mode local` for the text brain);
2. times the cold first answer and checks that this model answered, not a fallback (a fallback answer skips the model, with the reason);
3. reads `nvidia-smi` for the VRAM the model added;
4. asks llama-swap directly for about 150 tokens to get the decode speed;
5. sends 20 spoken-style commands (`--commands`; stop, wave, walk forward thirty centimeters, turn left ninety degrees, find the ball, how are you feeling, …), each followed by a stop, and scores the tool picked, its arguments, raw tool text leaking into the reply, missed stops and unasked motion;
6. places the ball left, right, ahead or nowhere and asks "what do you see, and is it on your left or right?" (`--describe`, default 6 turns);
7. with `--vision`, runs the vision bench's box detection and floor questions;
8. unloads the model (unless `--keep-loaded`). The models it unloaded earlier load again on their next request, cold.

**Leave the live cockpit alone while brain-bench runs.** The bench never sends anything to :8765, but both cockpits share the GPU. Do not type or talk to the live cockpit (turn hands-free off), and do not ask it to look or find. Any request there that reaches a model loads that model (the live cockpit's multimodal role is `qwen3.5-9b`, which runs alone) and evicts the one being measured; the bench's next line then loads it back. That repeated swapping is the pattern behind the 2026-09-01 GPU lockout, and it also spoils the VRAM and latency numbers.

It writes `sim/out/brain_bench.json` and prints a table plus a Markdown block. The block starts with the table's header line and separator line, which the table above already has: paste only the model's row (the lines after the separator) over its pending row.

**What each number means for choosing a role:**

- **Multimodal role** (the model sees the eye with every message and acts): `stop_missed` 0 and `unsafe` 0 are required; after that, `cmd_ok` near 20, a first-action median near the 9B's 0.2–1.5 s, and a correct describe side. It runs alone, so its VRAM only decides what else can run at the same time (ComfyUI, a game).
- **Brain role** (Local model mode; bench it with `--mode local`): the same command numbers, and no picture is needed. A brain small enough to sit beside lfm2.5-vl (its VRAM plus lfm2.5-vl's 4,556 MiB, with headroom under 16,384) could share a new group with an eye copy, as section A below describes. That is your decision after measuring, never the `resident` group.
- **Vision role** (look and find in every mode): the `--vision` columns. Compare its seen rate, median bearing error and median distance error with lfm2.5-vl's 12/12, 0.7° and 3 cm at 0.4 s. A vision model outside the resident pair swaps with the brain on every look.
- **`first_answer_s`** is what a swap costs: after switching, after a fallback, or after the 30-minute ttl unloaded the model.
- **`decode_tps`** matters for descriptions and chat. Tool calls are short, so for voice commands the first-action numbers count more.
- **Latency and first action.** The latency cell's first pair (median / p95, "wall") is the wall time of each `/api/chat` turn, motion included. The "first action" pair (`1st act s` in the printed table, median only) is the time from sending the line to the first thing the sim shows: a stop, a chord, a gesture or a walk starting, a look. It is polled ten times a second, so it can read up to 0.1 s late, and it counts only the lines that changed something in the sim. First action is the model's decision time and the number to compare with the 0.2–0.5 s and 0.2–1.5 s bars.

**The co-residency rule.** The 9B, the 12B and the 35B driver cannot be loaded together, and llama-swap evicts one to load another. New ids never go in the `resident` group: that group is measured at 14,928 of 16,384 MiB, and llama-swap does not check memory, so a third member turns a swap into an out-of-memory crash at spawn. Section C1's estimate for a UD-Q4_K_XL 4B (4.2–4.7 GB) raised the question of replacing lfm2.5-vl in `resident`. That idea applies to a UD-Q4_K_XL 4B only, not to the Q8_0 downloaded on 2026-09-25: its weights and mmproj alone are 4,916 MiB before any KV cache or compute buffer, above lfm2.5-vl's measured 4,556 MiB slot (section B's note says the same). It would need a UD-Q4_K_XL download, its measured VRAM and your decision; the installer never does it.

---

# Rocky brain: small-model recommendation

**Short version:** get **Qwen3.5-9B** first. It covers both of your wishes. Run it without its vision file and it is a 9B text brain that fits beside lfm2.5-vl entirely in VRAM (about 11.5 GB of 16). Add its vision file (mmproj) and it becomes the all-in-one brain on its own (about 8.3 GB). It has the best published tool-calling scores of anything checked. It also uses the same tool format and parser as your current brain (qwen3.6-35b-a3b), and the rocky brain code already sends `enable_thinking:false`. If it gets tools wrong in testing, try **Gemma 4 12B** next for all-in-one, or **Granite 4.1 8B** next for text-only.

All 22 candidates in the verified table exist on Hugging Face and are supported by your pinned llama.cpp build (36b10154). None came back "unknown", so there are no support-risk models to list separately. Every VRAM figure below is calculated from each model's config and file sizes. **None has been measured on this laptop.**

## ⚠ Rules that apply to every pick

- **Do not add any of these to the `resident` group.** That group is measured at 14,928 of 16,384 MiB (qwen3.6-35b-a3b 10,372 + lfm2.5-vl 4,556). The config's own comment warns: *"group logic does NOT check VRAM … A third member turns a graceful swap into a CUDA OOM at spawn."* Every candidate here either needs its own group or runs alone. So choosing one means the 35B driver gets unloaded during robot sessions.
- **lfm2.5-vl is already in `resident`.** I did not check whether llama-swap v249 lets one model belong to two groups. The safe route is a second model ID (for example `lfm2.5-vl-eye`) that points at the same GGUF and flags, and putting that ID in the new group.
- **Rename every generic `mmproj-F16.gguf` when you download it** (the commands below do this), and add each new file to `models.manifest.json`.
- **Bench etiquette:** load models through llama-swap only (`test-llm.sh load …`), and leave at least 15–30 s idle between spawns.
- **Speed estimates are probably too high.** They come from memory bandwidth (about 512 GB/s on the laptop 3080 Ti, divided by the weight size). But lfm2.5-vl, a 3B model at Q8, measures only 31 t/s on this machine, far below its bandwidth ceiling. Small models here appear to be held back by per-token overhead, so treat every t/s figure below as an upper bound.
- **KV cache** figures assume q8_0 (about 1.06 bytes per element), as `common_flags` sets.

---

## (A) A 7–14B text brain that fits beside lfm2.5-vl on the GPU (16k context)

### A1. Qwen3.5-9B, text-only (no mmproj): recommended

- **VRAM:** weights 5.29 GiB (Q4_K_M, 5.68 GB file).
  - KV cache: only 8 of its 32 layers use attention, at 8 × 4 × 256 × 2 × 1.06 ≈ 17 KB/token, so 16k ≈ 0.28 GB.
  - Recurrent state (the other 24 layers) ≈ 0.05 GB. Logits/compute buffer ≈ 0.5 GB. CUDA context ≈ 0.3 GB.
  - **Total ≈ 6.5–7.0 GB. With lfm2.5-vl: ≈ 11.5–12 GB of 16 GB. Fits, with about 4 GB spare.**
  - Because the KV cache is so small, 32k context would add only about 0.27 GB.
- **Speed:** ceiling about 95 t/s. Estimate 50–75 t/s (unmeasured). The MTP variant of the file might speed it up; the pinned build supports MTP on qwen35 (optional, test later).
- **Tool-calling confidence: high (on paper).**
  - Published scores: BFCL-V4 66.1, TAU2 79.1.
  - Tools use the Qwen3-Coder XML format. Your pinned build routes it to `common_chat_params_init_qwen3_coder` (chat.cpp:3596), the same parser your current brain uses.
  - Caveat: those scores are probably thinking-mode scores. With thinking off, accuracy is untested.
- **Install:**
  ```bash
  hf download unsloth/Qwen3.5-9B-GGUF Qwen3.5-9B-Q4_K_M.gguf --local-dir /mnt/models
  ```
  ```yaml
    "qwen3.5-9b-text":
      name: "Qwen3.5-9B (Q4_K_M) — text brain, fully on GPU"
      description: "5.7 GB, est ~6.5-7 GB at 16k (8/32 layers carry KV). UNMEASURED."
      ttl: 1800
      cmd: |
        ${binary}
        -m ${models_dir}/Qwen3.5-9B-Q4_K_M.gguf
        -c 16384
        -ngl 99
        ${common_flags}
        --chat-template-kwargs '{"enable_thinking":false}'
        --temp 0.7 --top-p 0.8 --top-k 20 --presence-penalty 1.5
  # groups: (NOT "resident")
    "robot":
      swap: false
      exclusive: true
      members: ["qwen3.5-9b-text", "lfm2.5-vl-eye"]   # lfm2.5-vl-eye = copy of the lfm2.5-vl stanza
  ```

### A2. Granite 4.1 8B: the fastest answers, with no thinking to manage

- **VRAM:** weights 4.98 GiB (5.35 GB file).
  - KV cache: 2 × 40 × 8 × 128 × 1.06 ≈ 85 KiB/token, so 16k = 1.33 GiB.
  - Compute buffer ≈ 0.4 GiB, CUDA context ≈ 0.3 GiB.
  - **Total ≈ 7.0 GiB. With lfm2.5-vl: ≈ 11.5 GiB. Fits.** At 32k it grows to about 8.3 GiB.
- **Speed:** ceiling about 95 t/s. Estimate 60–75 t/s (unmeasured).
- **Tool-calling confidence: medium-high.**
  - BFCL v3 68.27.
  - Tools are JSON inside `<tool_call>` tags. The pinned build ships `ibm-granite-granite-4.1.jinja` and tests parsing it (test-chat.cpp:3375).
  - It is not a reasoning model, so there is no thinking phase to switch off.
  - Red flag: the model card still contains *"<todo>Need to test the examples (especially the tool calling…)</todo>"*.
  - Chat personality is flat, which matters little here because Rocky replies in chord-speak anyway.
- **Install:**
  ```bash
  hf download unsloth/granite-4.1-8b-GGUF granite-4.1-8b-Q4_K_M.gguf --local-dir /mnt/models
  ```
  ```yaml
    "granite-4.1-8b":
      name: "Granite-4.1-8B (Q4_K_M) — dense text brain, no thinking"
      description: "5.35 GB, est ~7.0 GiB at 16k q8_0 KV. UNMEASURED."
      cmd: |
        ${binary}
        -m ${models_dir}/granite-4.1-8b-Q4_K_M.gguf
        -c 16384
        -ngl 99
        ${common_flags}
  # group: "robot" with members ["granite-4.1-8b", "lfm2.5-vl-eye"]
  ```

### A3. Qwen3-14B: the proven older control (tight fit)

- **VRAM:** weights 8.38 GiB (9.0 GB file).
  - KV cache: 2 × 40 × 8 × 128 × 1.06 → 1.33 GiB at 16k.
  - Compute buffer ≈ 0.4 GiB, CUDA context ≈ 0.3 GiB.
  - **Total ≈ 10.4 GiB. With lfm2.5-vl: ≈ 14.9 GiB, which leaves about 1 GiB.** That is the same tightness as today's `resident` pair. It works at 16k only; **32k would OOM**.
  - Qwen3-8B (5.03 GB file) is the roomier version: about 6.5 GiB, or about 11 GiB with the eye.
- **Speed:** ceiling about 57 t/s. Estimate 40–50 t/s. No MTP.
- **Tool-calling confidence: medium-high.** It uses Hermes-style `<tool_call>{json}` tags, which is the most widely used small-model tool format. It is from April 2025, so Qwen3.5-9B supersedes it. Keep it as an A/B control.
  - Correction to the candidate notes: llama.cpp uses the template embedded in the GGUF. The repo's separate `template` file is for Ollama and is never read.
- **Install:**
  ```bash
  hf download unsloth/Qwen3-14B-GGUF Qwen3-14B-Q4_K_M.gguf --local-dir /mnt/models
  ```
  ```yaml
    "qwen3-14b":
      name: "Qwen3 14B (Q4_K_M) — resident text brain (16k MAX beside lfm)"
      ttl: 1800
      cmd: |
        ${binary}
        -m ${models_dir}/Qwen3-14B-Q4_K_M.gguf
        -c 16384
        -ngl 99
        ${common_flags}
        --temp 0.7 --top-p 0.8 --top-k 20
  # per request: chat_template_kwargs {"enable_thinking": false}
  ```

**Not picked for A: LFM2.5-8B-A1B.** It would be the fastest here (only 1.5B parameters active per token, about 5.6–5.9 GB, about 10.3 GB beside lfm2.5-vl). But it always thinks before answering, and its template has no `enable_thinking` switch. The only lever is `--reasoning-budget 0`, which is untested on this model. For one-line commands, the thinking overhead may cancel the speed advantage.

---

## (B) All-in-one brain: vision + chat + tools in one 4–12B model

### B1. Qwen3.5-9B with vision (UD-Q4_K_XL + mmproj): recommended

- **VRAM:** weights 5.97 GB + mmproj 0.92 GB + KV 0.27 GB (16k) + recurrent state 0.05 GB + logits 0.5 GB + vision encoder buffer ≈ 0.4 GB + CUDA context 0.3 GB.
  - **Total ≈ 8.1–8.4 GB alone.** lfm2.5-vl is no longer needed, which leaves about 8 GB free. It does **not** fit beside qwen3.6-35b-a3b (≈ 18.8 GB).
- **Speed:** estimate 50–65 t/s decode (unmeasured). A 320×240 eye frame is about 70–80 image tokens, which is cheap to process.
- **Tool-calling confidence: high (on paper).** Same parser and scores as A1. Published vision scores: MMMU 78.4, RealWorldQA 80.3, CountBench 97.2.
- **Grounding risk:** the pinned `clip.cpp:1634` warns that Qwen-VL needs at least 1024 image tokens for grounding. At about 80 tokens, questions like "where is X?" may be weak. Test `look` and `scan` both with and without `--image-min-tokens 256`–`1024`; more image tokens add prefill time.
- **Install:**
  ```bash
  hf download unsloth/Qwen3.5-9B-GGUF Qwen3.5-9B-UD-Q4_K_XL.gguf mmproj-F16.gguf --local-dir /mnt/models \
    && mv /mnt/models/mmproj-F16.gguf /mnt/models/mmproj-Qwen3.5-9B-F16.gguf
  # optional later: unsloth/Qwen3.5-9B-MTP-GGUF (+0.19 GB) → add --spec-type draft-mtp --spec-draft-n-max 2 --spec-draft-p-min 0.75
  ```
  ```yaml
    "qwen3.5-9b":
      name: "Qwen3.5-9B (UD-Q4_K_XL) + vision — all-in-one robot brain"
      description: "6.9 GB incl. mmproj, est ~8.3 GB at 16k. UNMEASURED. Runs ALONE (no group with qwen3.6)."
      ttl: 1800
      cmd: |
        ${binary}
        -m ${models_dir}/Qwen3.5-9B-UD-Q4_K_XL.gguf
        --mmproj ${models_dir}/mmproj-Qwen3.5-9B-F16.gguf
        -c 16384
        -ngl 99
        ${common_flags}
        --chat-template-kwargs '{"enable_thinking":false}'
        --temp 0.7 --top-p 0.8 --top-k 20 --presence-penalty 1.5
  ```

### B2. Gemma 4 12B Unified: the newest option, with the cheapest image handling

- **VRAM:** weights 7.37 GB + mmproj 0.18 GB.
  - KV cache: 40 sliding-window layers ≈ 0.27 GB, plus 8 global layers × 16k ≈ 0.14 GB.
  - Compute ≈ 0.8 GB (vocabulary of 262k), CUDA context ≈ 0.3 GB.
  - **Total ≈ 9.0 GB alone** (9.5 GB is a safe upper bound; about 10 GB if you add the MTP drafter). It would fit beside lfm2.5-vl with about 2 GB spare, but as an all-in-one it doesn't need the eye.
- **Speed:** estimate 45–55 t/s (unmeasured). The MTP drafter (465 MB, `gemma4-assistant` architecture, supported at the pin) might help, but whether it works together with `--mmproj` is untested.
- **Tool-calling confidence: medium-high.**
  - Tau2 69.0, about the same as the gemma-4-26b-a4b already in your fleet.
  - The pinned build has a dedicated `PEG_GEMMA4` parser. The GGUF's template carries the official "OpenAI Chat Completions" marker, so it takes the current path, not the outdated-template workaround.
  - Thinking defaults to off.
  - The parser tests use the 31B model's template, not the 12B one.
- **Vision caveats:**
  - It has no separate vision encoder, so the 12B model itself processes every image.
  - The pinned `clip.cpp` has a comment saying it *"performs quite poor with small images"*, so set a floor of `--image-min-tokens 140`.
  - Keep `--image-max-tokens` at or below 280, and never above 512 without raising `-b/-ub`. That is the same rule noted in your gemma-4-26b-a4b entry.
- **Install:**
  ```bash
  hf download unsloth/gemma-4-12b-it-GGUF gemma-4-12b-it-UD-Q4_K_XL.gguf mmproj-F16.gguf --local-dir /mnt/models \
    && mv /mnt/models/mmproj-F16.gguf /mnt/models/mmproj-gemma-4-12b-it-F16.gguf
  ```
  ```yaml
    "gemma-4-12b":
      name: "Gemma 4 12B Unified (UD-Q4_K_XL) — vision + tools, fully on GPU"
      # Encoder-free vision; image decoded non-causally in ONE ubatch → image-max-tokens ≤ 512 unless -b/-ub raised.
      description: "7.4 GB + 175 MB mmproj, est ~9 GB at 16k. UNMEASURED."
      cmd: |
        ${binary}
        -m ${models_dir}/gemma-4-12b-it-UD-Q4_K_XL.gguf
        --mmproj ${models_dir}/mmproj-gemma-4-12b-it-F16.gguf
        -c 16384
        -ngl 99
        --image-min-tokens 140 --image-max-tokens 280
        ${common_flags}
  # per request: chat_template_kwargs {"enable_thinking": false}
  ```

### B3. Ministral 3 8B Instruct 2512: direct answers, no thinking at all

- **VRAM:** weights 4.93 GiB + mmproj 0.80 GiB.
  - KV cache: 34 × 2 × 8 × 128 × 1.06 → 1.13 GiB at 16k.
  - Compute buffer + CUDA context ≈ 0.7–1.0 GiB.
  - **Total ≈ 7.5–8.4 GB alone.**
- **Speed:** estimate 55–75 t/s (unmeasured). About 100–115 image tokens per eye frame.
- **Tool-calling confidence: medium, and unbenchmarked.**
  - The card claims native function calling but publishes no BFCL or Tau2 score.
  - The pinned build routes it to a dedicated `common_chat_params_init_ministral_3` parser, which constrains tool arguments with a JSON grammar.
  - I checked `rocky/sim/cockpit_brains.py:965-1007` and the current brain's message layout doesn't break this template's rules.
  - Mistral recommends temperature at or below 0.15. That suits tool calls but makes chat flat, so pass a higher temperature per request for chat turns.
- **Fallback if Qwen3.5's thinking handling misbehaves:** Qwen3-VL-8B-Instruct. It is instruct-only (no thinking), widely used, uses Hermes JSON tools, and needs about 8 GB.
- **Install:**
  ```bash
  hf download unsloth/Ministral-3-8B-Instruct-2512-GGUF Ministral-3-8B-Instruct-2512-UD-Q4_K_XL.gguf --local-dir /mnt/models
  hf download unsloth/Ministral-3-8B-Instruct-2512-GGUF mmproj-F16.gguf --local-dir /mnt/models/.stage-ministral3 \
    && mv /mnt/models/.stage-ministral3/mmproj-F16.gguf /mnt/models/mmproj-Ministral-3-8B-Instruct-2512-F16.gguf \
    && rm -rf /mnt/models/.stage-ministral3
  ```
  ```yaml
    "ministral-3-8b":
      name: "Ministral 3 8B Instruct 2512 (UD-Q4_K_XL) — all-in-one VLM + tools"
      description: "5.3 GB + 0.86 GB mmproj, est ~7.5-8.4 GB at 16k. Non-reasoning. UNMEASURED."
      cmd: |
        ${binary}
        -m ${models_dir}/Ministral-3-8B-Instruct-2512-UD-Q4_K_XL.gguf
        --mmproj ${models_dir}/mmproj-Ministral-3-8B-Instruct-2512-F16.gguf
        -c 16384
        -ngl 99
        ${common_flags}
        --temp 0.15
  ```

**A middle option that keeps the 35B driver:** Qwen3.5-4B (UD-Q4_K_XL + mmproj) is estimated at about 4.2–4.7 GB, roughly the size of lfm2.5-vl's slot in `resident` (4.45 GiB). It could *replace* lfm2.5-vl there as a far stronger eye and fast-tool model. Measure it with `nvidia-smi` before swapping it into the group. Its Q8_0 version will **not** fit there, which disproves the candidate notes' claim that it would.

---

## (C) A tiny brain that could later run on the robot's own computer

I assumed a Raspberry Pi 5 or Jetson-class board, since the robot's computer isn't specified. Every Pi speed below is an estimate or a vendor figure, not a measurement.

### C1. Qwen3.5-4B (vision + tools)

- **Size:** UD-Q4_K_XL 2.91 GB + mmproj 0.67 GB.
- **GPU estimate on the laptop:** weights 2.71 GiB + mmproj 0.63 + KV 0.27 at 16k + recurrent state 0.05 + compute 0.5–0.8 ≈ **4.2–4.7 GB**. Beside lfm2.5-vl that is about 9 GB.
- **On a Pi 5:** perhaps 4–6 t/s (estimate).
- **Tools:** Qwen3-Coder XML format, same parser as your current brain. BFCL-V4 50.3, TAU2 79.9 (both thinking-mode scores). Multi-argument calls like `compose_gesture` are the risk.
- **Install:**
  ```bash
  hf download unsloth/Qwen3.5-4B-GGUF Qwen3.5-4B-UD-Q4_K_XL.gguf mmproj-F16.gguf --local-dir /mnt/models \
    && mv /mnt/models/mmproj-F16.gguf /mnt/models/mmproj-Qwen3.5-4B-F16.gguf
  ```
  ```yaml
    "qwen3.5-4b":
      name: "Qwen3.5-4B (UD-Q4_K_XL) — small all-in-one VLM brain"
      description: "3.6 GB weights+mmproj, est ~4.5 GB at 16k. Not in 'resident' until measured ≤ ~4.4 GiB."
      cmd: |
        ${binary}
        -m ${models_dir}/Qwen3.5-4B-UD-Q4_K_XL.gguf
        --mmproj ${models_dir}/mmproj-Qwen3.5-4B-F16.gguf
        -c 16384
        -ngl 99
        ${common_flags}
        --chat-template-kwargs '{"enable_thinking":false}'
        --temp 0.7 --top-p 0.8 --top-k 20 --presence-penalty 1.5
  ```

### C2. LFM2.5-1.2B-Instruct (text-only, Pi-class)

- **Size:** 0.73 GB (Q4_K_M). A QAD-Q4_0 file (0.70 GB) is the ARM-friendly quant.
- **VRAM:** 16 layers, only 6 use attention, so KV is about 0.10 GB at 16k. **Total ≈ 1.2–1.3 GB.**
- **Speed:** vendor figure 239 t/s on an AMD CPU. On a Pi 5, about 10–15 t/s (extrapolated).
- **Tools:** Pythonic calls between `<|tool_call_start|>` and `<|tool_call_end|>`, the same LFM2.5 parser as your lfm2.5-vl, whose tool calling you already verified. BFCL v3 49 is mediocre, so keep the tool list short with temperature 0.1. The parser does not accept dotted names or true/false/null inside arguments. Weak at casual chat.
- **Install:**
  ```bash
  hf download LiquidAI/LFM2.5-1.2B-Instruct-GGUF LFM2.5-1.2B-Instruct-Q4_K_M.gguf LFM2.5-1.2B-Instruct-QAD-Q4_0.gguf --local-dir /mnt/models
  ```
  ```yaml
    "lfm2.5-1.2b":
      name: "LFM2.5-1.2B-Instruct (Q4_K_M) — tiny text brain (Pi prototype)"
      cmd: |
        ${binary}
        -m ${models_dir}/LFM2.5-1.2B-Instruct-Q4_K_M.gguf
        -c 16384
        -ngl 99
        ${common_flags}
        --temp 0.1 --top-k 50 --repeat-penalty 1.05
  ```

### C3. LFM2.5-VL-1.6B (the smallest all-in-one; smaller sibling of your eye model)

- **Size:** 0.73 GB (Q4_K_M) + mmproj 0.58 GB. **Use Q8_0 (1.25 GB)** at this size.
- **VRAM:** about 1.6–2.2 GB. The 320×240 frame fits in one 512 px tile.
- **Tools:** same parser as C2. The model card says tool calling is **for text-only input**, so a turn that has both an image and tools is unverified. Plan on a two-step loop: describe the image, then act.
- **Install:**
  ```bash
  hf download LiquidAI/LFM2.5-VL-1.6B-GGUF LFM2.5-VL-1.6B-Q8_0.gguf mmproj-LFM2.5-VL-1.6b-Q8_0.gguf --local-dir /mnt/models
  ```
  ```yaml
    "lfm2.5-vl-1.6b":
      name: "LFM2.5-VL-1.6B (Q8_0) — tiniest all-in-one vision"
      cmd: |
        ${binary}
        -m ${models_dir}/LFM2.5-VL-1.6B-Q8_0.gguf
        --mmproj ${models_dir}/mmproj-LFM2.5-VL-1.6b-Q8_0.gguf
        -c 16384
        -ngl 99
        ${common_flags}
  ```

**Other small options (verified):**
- **No thinking at all:** Granite 4.1 3B (BFCL v3 60.8, about 3.5 GB) and Qwen3-4B-Instruct-2507 (BFCL v3 61.9, about 4.3 GB, never thinks).
- **Always thinks:** LFM2.5-2.6B (BFCL v4 56.9). Its template forces `<think>` and ignores `enable_thinking`.
- **Loop risk:** Qwen3.5-2B. Its card warns it can get stuck in thinking loops.

---

## Suggested order of work

1. **Download Qwen3.5-9B**: UD-Q4_K_XL + mmproj ≈ 6.9 GB; the Q4_K_M file is another 5.7 GB if you also want A1's text-only entry. /mnt/models has 146–153 GB free.
2. **Add two entries:** `qwen3.5-9b` (vision) and, if you want the pair option, `qwen3.5-9b-text` in a new `robot` group with an `lfm2.5-vl-eye` copy.
3. **Acceptance test before switching the brain:**
   - 20 spoken-style commands against the real tool schema (say, gesture, goto, stop, look, scan, compose_gesture) with thinking off.
   - 10 eye-JPEG describe turns.
   - Record `nvidia-smi` MiB, t/s and seconds per command, and compare with lfm2.5-vl's 0.2–0.5 s per command.
   - Check specifically that `stop` is never missed, and that tool calls come back as parsed `tool_calls`, not raw text.
4. **If it falls short:** Gemma 4 12B for all-in-one, then Granite 4.1 8B (with the lfm eye) for the text-brain route.

## Ideas for testing the vision

- Rocky already has `look`, `scan`, the cockpit world editor and recordings. Place known objects in the world (a coloured block, a cliff edge, stairs) and ask *"what's in front of you? how many X? is it left or right?"*.
  - Grading is automatic because the sim knows where everything really is.
  - The weak point at about 80 image tokens is left/right and "where" questions, so sweep `--image-min-tokens`.
- **Look-then-act chains:** *"go to the red block"* → look → goto. This tests the all-in-one case directly: an image and tools in the same turn.
- **Stop safety:** show frames with a cliff or rubble ahead and check that the model's own description agrees with the feet-sensing veto that fires. Score how often a `stopped='cliff'` veto is reported correctly rather than retried.
- **Head-to-head:** run the same recorded frames through lfm2.5-vl and the new model with an identical prompt, then compare accuracy and latency. Replaying a recording makes it repeatable.

## What I could not verify

- **Nothing was downloaded, spawned or benchmarked.** Every VRAM, t/s and latency figure is calculated or estimated. The speed estimates are likely optimistic (see the lfm2.5-vl 31 t/s note at the top).
- **Tool accuracy with thinking off** is untested for every model. The published BFCL and Tau2 scores are vendor numbers, mostly measured with thinking on.
- **Rocky's own tool schema** has not been run against any candidate. That includes whether `compose_gesture`'s multi-argument calls parse.
- **Parser tests use sibling templates:** the Gemma 4 tests use the 31B template, the Ministral tests use the 14B-Reasoning template, and the Qwen3.5 tests use the 4B template.
- **Group membership:** whether llama-swap v249 lets one model sit in two groups is unchecked, hence the `lfm2.5-vl-eye` duplicate.
- **Untested combinations:** MTP speedups (Qwen3.5-9B-MTP, the Gemma drafters), MTP together with `--mmproj` on Gemma, and `--reasoning-budget 0` on the always-thinking LFM models.
- **Image quality at 320×240** is unknown for all candidates, in particular the Qwen-VL grounding warning and the Gemma small-image warning in `clip.cpp`.
- **Not confirmed:** Unsloth's "tool-calling template fix" claim for Qwen3.5, the "strongest in class" wording, the claim that Qwen3.5-9B beats Qwen3-VL-8B, and all Pi 5 speed figures.
