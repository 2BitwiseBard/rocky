# Small brain models for Pebble — research 2026-09-24

*Written by an Opus research workflow (3 sweeps, 22 candidates, each adversarially verified against the Hugging Face page and the pinned llama.cpp build 36b10154). Nothing here was downloaded or measured on this machine when written; see the 'could not verify' section. Kept as the record behind the model choice.*

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