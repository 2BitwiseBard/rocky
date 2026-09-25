"""sim/brain_install.py: a downloaded GGUF lands where the brains expect it, and nothing else moves.

Everything runs on synthetic GGUF headers in tmp_path (a few hundred bytes
each: magic, v3, KV strings, one skipped array, optionally a real tensor table
+ data section) with a copy of a minimal llama-swap config and manifest.
Always --no-restart; --no-fetch except where a fetch must FAIL (HF_BIN points
at a file that does not exist): no test restarts llama-swap, runs `hf
download`, or loads a model. Two tests read the REAL config/manifest, but only
as copies in tmp_path, and one reads real GGUF headers under /mnt/models
(read-only; skipped where those files are absent).
"""
import hashlib
import json
import os
import shutil
import struct
import subprocess
import sys
import threading
import time
import types

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim"))

import brain_install as bi                                               # noqa: E402

TODAY = time.strftime("%Y%m%d")
REAL_CONFIG = os.path.expanduser("~/.config/llama-swap/config.yaml")
REAL_MANIFEST = os.path.expanduser("~/.config/llama-swap/models.manifest.json")


# ------------------------------------------------------------------ synthetic GGUF
def _s(x):
    b = x.encode()
    return struct.pack("<Q", len(b)) + b


# ggml type -> (elements per block, bytes per block): the two this file writes, from ggml-common.h
_TEST_TYPES = {0: (1, 4), 8: (32, 34)}          # F32, Q8_0


def write_gguf(path, meta, pad=64, tensors=()):
    """A GGUF v3 header: the given KVs (str / int / bool), a string array and
    a u32 array (both must be skipped by the reader), then — for `tensors`, a
    list of (name, dims, ggml type) — their infos, the 32-aligned data section
    (each tensor padded to 32 as llama.cpp writes it), then `pad` bytes."""
    kv, n = b"", 0
    for k, v in meta.items():
        kv += _s(k)
        if isinstance(v, bool):
            kv += struct.pack("<I?", 7, v)
        elif isinstance(v, int):
            kv += struct.pack("<II", 4, v)
        else:
            kv += struct.pack("<I", 8) + _s(str(v))
        n += 1
    kv += _s("tokenizer.ggml.tokens") + struct.pack("<IIQ", 9, 8, 3) + _s("a") + _s("bb") + _s("ccc")
    kv += _s("x.rope.sections") + struct.pack("<IIQ", 9, 4, 4) + struct.pack("<4I", 1, 2, 3, 4)
    n += 2
    infos, data, off = b"", b"", 0
    for i, (name, dims, ttype) in enumerate(tensors):
        blck, tsize = _TEST_TYPES[ttype]
        nel = 1
        for d in dims:
            nel *= d
        nbytes = nel // blck * tsize
        infos += _s(name) + struct.pack("<I", len(dims)) + struct.pack(f"<{len(dims)}Q", *dims) + \
            struct.pack("<IQ", ttype, off)
        data += b"\x01" * nbytes + (b"\0" * (-nbytes % 32) if i < len(tensors) - 1 else b"")
        off += nbytes + (-nbytes % 32)
    head = b"GGUF" + struct.pack("<IQQ", 3, len(tensors), n) + kv + infos
    if tensors:
        head += b"\0" * (-len(head) % 32)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(head + data + b"\0" * pad)
    return path


TENSORS = [("blk.0.w", [7, 3], 0), ("blk.0.q", [64, 2], 8)]      # 84 + 136 bytes of data


def sha1(path):
    with open(path, "rb") as f:
        return hashlib.sha1(f.read()).hexdigest()


def qwen(base="Qwen3.5-4B", size="4B"):
    return {"general.architecture": "qwen35", "general.type": "model", "general.name": base,
            "general.basename": base, "general.size_label": size, "general.file_type": 15}


def qwen_mm(base="Qwen3.5-4B"):
    return {"general.architecture": "clip", "general.type": "mmproj", "general.name": base,
            "general.basename": base, "general.size_label": "334M",
            "general.finetune": base.split("-")[-1].lower(),
            "clip.has_vision_encoder": True, "clip.projector_type": "qwen3vl_merger"}


def gemma(base="Gemma-4-12B-It", size="12B"):
    return {"general.architecture": "gemma4", "general.type": "model", "general.name": base,
            "general.basename": base, "general.size_label": size, "general.finetune": "it"}


def gemma_mm(base="Gemma-4-12B-It"):
    return {"general.architecture": "clip", "general.type": "mmproj", "general.name": base,
            "general.basename": base, "general.finetune": "12b-it", "general.size_label": "52M",
            "clip.has_vision_encoder": True, "clip.vision.projector_type": "gemma4uv"}


CONFIG = """\
# test config — the same skeleton as the real one
healthCheckTimeout: 300
apiKeys:
  - ${{env.LOCAL_AI_KEY}}

macros:
  binary: /opt/llama/llama-server
  models_dir: {models_dir}
  common_flags: |
    --host 127.0.0.1 --port ${{PORT}}
    -np 1

selectors:
  "vision-model":
    strategy: warm
    targets: ["qwen3.5-9b"]

models:
  "qwen3.5-9b":
    name: "Qwen3.5-9B (UD-Q4_K_XL) + vision"
    description: "6.0 GB + 0.9 GB mmproj. VISION (mmproj) + tools. Runs alone."
    ttl: 1800
    cmd: |
      ${{binary}}
      -m ${{models_dir}}/qwen3.5-9b/Qwen3.5-9B-UD-Q4_K_XL.gguf
      --mmproj ${{models_dir}}/qwen3.5-9b/mmproj-Qwen3.5-9B-F16.gguf
      -c 16384
      ${{common_flags}}

  # ===================================================================
  # RETRIEVAL STACK (CPU, always warm — ttl 0 disables idle unload)
  # ===================================================================
  "embedding":
    name: "Qwen3-Embedding-0.6B (CPU)"
    ttl: 0
    cmd: |
      ${{binary}}
      -m ${{models_dir}}/Qwen3-Embedding-0.6B-Q8_0.gguf
      --embedding

groups:
  # ⚠ a third member turns a graceful swap into a CUDA OOM at spawn
  "resident":
    swap: false
    exclusive: true
    members:
      - "qwen3.5-9b"
      - "embedding"
"""


class Env:
    def __init__(self, tmp):
        self.tmp = tmp
        self.downloads = os.path.join(tmp, "Downloads")
        self.models = os.path.join(tmp, "models")
        self.config = os.path.join(tmp, "config.yaml")
        self.manifest = os.path.join(tmp, "models.manifest.json")
        os.makedirs(self.downloads)
        self.model("qwen3.5-9b/Qwen3.5-9B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-9B", "9B"), 300)    # configured
        self.model("qwen3.5-9b/mmproj-Qwen3.5-9B-F16.gguf", qwen_mm("Qwen3.5-9B"), 90)
        with open(self.config, "w") as f:
            f.write(CONFIG.format(models_dir=self.models))
        with open(self.manifest, "w") as f:
            json.dump({"_comment": "test", "models_dir": self.models, "models": [
                {"filename": "qwen3.5-9b/Qwen3.5-9B-UD-Q4_K_XL.gguf", "size_bytes": 1,
                 "hf_repo": "unsloth/Qwen3.5-9B-GGUF", "hf_file": "Qwen3.5-9B-UD-Q4_K_XL.gguf", "notes": "x"},
                {"filename": "qwen3.5-9b/mmproj-Qwen3.5-9B-F16.gguf", "size_bytes": 1,
                 "hf_repo": "unsloth/Qwen3.5-9B-GGUF", "hf_file": "mmproj-F16.gguf", "notes": "x"}]},
                      f, indent=2)

    def dl(self, name, meta, pad=64, tensors=()):
        return write_gguf(os.path.join(self.downloads, name), meta, pad, tensors)

    def model(self, rel, meta, pad=64, tensors=()):
        return write_gguf(os.path.join(self.models, rel), meta, pad, tensors)

    def args(self, *extra, fetch=False):
        a = ["--downloads", self.downloads, "--models-dir", self.models, "--config", self.config,
             "--manifest", self.manifest, "--settle", "0", "--no-restart", "--json"]
        return a + ([] if fetch else ["--no-fetch"]) + list(extra)

    def text(self):
        with open(self.config) as f:
            return f.read()

    def mani(self):
        with open(self.manifest) as f:
            return json.load(f)


@pytest.fixture
def env(tmp_path, monkeypatch):
    monkeypatch.setattr(bi, "HF_BIN", str(tmp_path / "no-such-hf"))        # belt and braces: never download
    monkeypatch.setattr(bi.Installer, "_restart", lambda self, ids: pytest.fail("restart attempted"))
    return Env(str(tmp_path))


def run(env, capsys, *extra, fetch=False):
    code = bi.main(env.args(*extra, fetch=fetch))
    out = capsys.readouterr().out
    summary = json.loads(out.strip().splitlines()[-1])
    return code, out, summary


def by_name(summary):
    return {it["name"]: it for it in summary["items"]}


def tree(root):
    """{relpath: (size, mtime_ns, sha1)} for every file under root."""
    out = {}
    for d, _, files in os.walk(root):
        for fn in files:
            p = os.path.join(d, fn)
            st = os.stat(p)
            with open(p, "rb") as f:
                digest = hashlib.sha1(f.read()).hexdigest()
            out[os.path.relpath(p, root)] = (st.st_size, st.st_mtime_ns, digest)
    return out


# ------------------------------------------------------------------ the reader + helpers
def test_reader_reads_strings_skips_arrays_and_refuses_junk(tmp_path):
    p = write_gguf(str(tmp_path / "a.gguf"), gemma())
    h = bi.read_gguf_header(p)
    assert h["version"] == 3 and h["n_tensors"] == 0
    assert h["meta"]["general.architecture"] == "gemma4"
    assert h["meta"]["general.size_label"] == "12B"
    assert "tokenizer.ggml.tokens" not in h["meta"]                      # arrays are skipped, not kept
    info = bi.header_info(p)
    assert (info["family"], info["key"], info["is_mmproj"]) == ("gemma4", "gemma-4-12b", False)
    raw = open(p, "rb").read()
    cut = tmp_path / "cut.gguf"
    cut.write_bytes(raw[:60])
    with pytest.raises(bi.GGUFError):
        bi.read_gguf_header(str(cut))
    junk = tmp_path / "junk.gguf"
    junk.write_bytes(b"PK\x03\x04" + b"\0" * 64)
    with pytest.raises(bi.GGUFError):
        bi.read_gguf_header(str(junk))


def test_names_quants_and_keys():
    assert bi.quant_of("Qwen3.5-4B-UD-Q4_K_XL.gguf") == "UD-Q4_K_XL"
    assert bi.quant_of("Qwen3.5-4B-Q8_0.gguf") == "Q8_0"
    assert bi.quant_of("Qwen3.8-27B-UD-Q4_K_XL-v3.gguf") == "UD-Q4_K_XL"
    assert bi.file_base("gemma-4-12b-it-UD-Q4_K_XL.gguf") == "gemma-4-12b-it"
    assert bi.file_base("Qwen3.5-9B-UD-Q6_K_XL.gguf") == "Qwen3.5-9B"
    assert bi.clean_name("/x/Qwen3.5-4B-Q8_0 (1).gguf") == "Qwen3.5-4B-Q8_0.gguf"
    assert [bi.quant_suffix(q) for q in ("UD-Q6_K_XL", "Q8_0", "IQ4_XS", "Q4_K_M", "BF16", "UD-IQ1_S")] == \
        ["q6k", "q8", "iq4", "q4k", "bf16", "iq1"]
    assert bi.quant_suffix("UD-Q4_K_XL", long=True) == "q4kxl"
    # the id comes from the header; an mmproj's size_label is the projector's own size and is ignored
    assert bi.base_key(qwen("Qwen3.5-4B", "4B"), "x.gguf") == "qwen3.5-4b"
    assert bi.base_key(gemma("gemma-4", "12B"), "x.gguf") == "gemma-4-12b"
    assert bi.base_key(gemma("Gemma-4-26B-A4B-It", "26B-A4B"), "x.gguf") == "gemma-4-26b-a4b"
    assert bi.base_key(gemma_mm(), "mmproj-F16.gguf", is_mmproj=True) == "gemma-4-12b"
    lfm_mm = {"general.architecture": "clip", "general.basename": "LFM2.5-VL", "general.finetune": "3b",
              "general.size_label": "426M"}
    assert bi.base_key(lfm_mm, "mmproj-F16.gguf", is_mmproj=True) == "lfm2.5-vl-3b"
    assert bi.family_of({"general.architecture": "gemma4-assistant"}, "gemma-4-423m") == "unknown"


REAL_HEADERS = {
    "/mnt/models/qwen3.5-9b/Qwen3.5-9B-UD-Q4_K_XL.gguf": ("qwen35", "model", "qwen3.5-9b"),
    "/mnt/models/qwen3.5-9b/mmproj-Qwen3.5-9B-F16.gguf": ("clip", "mmproj", "qwen3.5-9b"),
    "/mnt/models/gemma-4-26B-A4B-it-UD-Q5_K_XL.gguf": ("gemma4", "model", "gemma-4-26b-a4b"),
    "/mnt/models/mmproj-gemma-4-26B-A4B-F16.gguf": ("clip", "mmproj", "gemma-4-26b-a4b"),
    "/mnt/models/LiquidAI_LFM2.5-VL-3B-Q8_0.gguf": ("lfm2", "model", "lfm2.5-vl-3b"),
    "/mnt/models/gemma-4-12b/mmproj-gemma-4-12b-it-F16.gguf": ("clip", "mmproj", "gemma-4-12b"),
    "/mnt/models/qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf": ("clip", "mmproj", "qwen3.5-4b"),
}


@pytest.mark.parametrize("path", sorted(REAL_HEADERS))
def test_real_headers_on_this_machine(path):
    """Read-only: the header of the real files (only where they exist)."""
    if not os.path.isfile(path):
        pytest.skip(f"{path} not on this machine")
    arch, gtype, key = REAL_HEADERS[path]
    i = bi.header_info(path)
    assert (i["arch"], i["type"], i["key"]) == (arch, gtype, key)


# ------------------------------------------------------------------ installs
def test_three_downloads_get_the_expected_ids_and_projectors(env, capsys):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))        # projector in the folder
    env.dl("mmproj-F16.gguf", gemma_mm(), pad=77)                                     # projector in Downloads
    env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 200)
    env.dl("gemma-4-12b-it-UD-Q4_K_XL.gguf", gemma(), 210)
    env.dl("Qwen3.5-9B-UD-Q6_K_XL.gguf", qwen("Qwen3.5-9B", "9B"), 220)       # 9b is configured: suffix
    code, out, s = run(env, capsys)
    assert code == 0, out
    items = by_name(s)
    assert items["Qwen3.5-4B-UD-Q4_K_XL.gguf"]["id"] == "qwen3.5-4b"
    assert items["gemma-4-12b-it-UD-Q4_K_XL.gguf"]["id"] == "gemma-4-12b"
    assert items["Qwen3.5-9B-UD-Q6_K_XL.gguf"]["id"] == "qwen3.5-9b-q6k"
    assert all(items[n]["status"] == "done" for n in items), items
    m = env.models
    for rel in ("qwen3.5-4b/Qwen3.5-4B-UD-Q4_K_XL.gguf", "gemma-4-12b/gemma-4-12b-it-UD-Q4_K_XL.gguf",
                "qwen3.5-9b-q6k/Qwen3.5-9B-UD-Q6_K_XL.gguf", "gemma-4-12b/mmproj-gemma-4-12b-it-F16.gguf",
                "qwen3.5-9b-q6k/mmproj-Qwen3.5-9B-F16.gguf"):
        assert os.path.isfile(os.path.join(m, rel)), rel
        bi.read_gguf_header(os.path.join(m, rel))
    assert os.listdir(env.downloads) == []                              # moved, projector included
    assert items["gemma-4-12b-it-UD-Q4_K_XL.gguf"]["mmproj"]["how"] == "downloads"
    assert items["Qwen3.5-4B-UD-Q4_K_XL.gguf"]["mmproj"]["how"] == "in-folder"
    assert items["Qwen3.5-9B-UD-Q6_K_XL.gguf"]["mmproj"]["how"] == "sibling"
    # the sibling projector is the SAME file (hard link), not a second copy
    assert os.stat(os.path.join(m, "qwen3.5-9b-q6k/mmproj-Qwen3.5-9B-F16.gguf")).st_ino == \
        os.stat(os.path.join(m, "qwen3.5-9b/mmproj-Qwen3.5-9B-F16.gguf")).st_ino
    cfg = yaml.safe_load(env.text())
    for mid in ("qwen3.5-4b", "gemma-4-12b", "qwen3.5-9b-q6k"):
        spec = cfg["models"][mid]
        assert "vision (mmproj)" in spec["name"] and spec["description"].startswith("UNMEASURED")
        assert spec["ttl"] == 1800 and "-c 16384" in spec["cmd"] and "-ngl 99" in spec["cmd"]
        assert "${common_flags}" in spec["cmd"] and "--mmproj ${models_dir}/" + mid + "/" in spec["cmd"]
    assert "--chat-template-kwargs '{\"enable_thinking\":false}'" in cfg["models"]["qwen3.5-4b"]["cmd"]
    assert "--presence-penalty 1.5" in cfg["models"]["qwen3.5-9b-q6k"]["cmd"]
    assert "--image-min-tokens 140 --image-max-tokens 280" in cfg["models"]["gemma-4-12b"]["cmd"]
    assert "enable_thinking" not in cfg["models"]["gemma-4-12b"]["cmd"]
    for mid in ("qwen3.5-4b", "gemma-4-12b", "qwen3.5-9b-q6k"):
        assert os.path.isfile(f"{env.config}.bak-{TODAY}-{mid}")


def test_stanza_lands_before_retrieval_and_protected_sections_are_untouched(env, capsys):
    before = env.text()
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    code, out, _ = run(env, capsys)
    assert code == 0, out
    after = env.text()
    i_new, i_old = after.index('"qwen3.5-4b":'), after.index('"qwen3.5-9b":')
    block = "  # " + "=" * 67 + "\n  # RETRIEVAL STACK"
    i_block = after.index(block)
    assert i_old < i_new < i_block
    assert after[:after.index("  # ---- added")] == before[:before.index("  # ===")]    # nothing above moved
    assert after[i_block:] == before[before.index(block):]
    assert after[after.index("\ngroups:"):] == before[before.index("\ngroups:"):]       # groups byte-for-byte
    old, new = yaml.safe_load(before), yaml.safe_load(after)
    for k in ("groups", "selectors", "macros", "apiKeys"):
        assert old[k] == new[k]
    assert set(new["models"]) - set(old["models"]) == {"qwen3.5-4b"}


def test_manifest_entries_are_appended_once(env, capsys):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 123)
    code, out, _ = run(env, capsys)
    assert code == 0, out
    names = [e["filename"] for e in env.mani()["models"]]
    assert names.count("qwen3.5-4b/Qwen3.5-4B-UD-Q4_K_XL.gguf") == 1
    assert names.count("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf") == 1
    entry = next(e for e in env.mani()["models"] if e["filename"] == "qwen3.5-4b/Qwen3.5-4B-UD-Q4_K_XL.gguf")
    assert entry["hf_repo"] == "unsloth/Qwen3.5-4B-GGUF" and entry["hf_file"] == "Qwen3.5-4B-UD-Q4_K_XL.gguf"
    assert entry["size_bytes"] == os.path.getsize(os.path.join(env.models, entry["filename"]))
    mani_bytes, cfg_bytes = open(env.manifest, "rb").read(), env.text()
    # a second run: nothing in Downloads, and the installed file named in place — both change nothing
    code, out, s = run(env, capsys)
    assert code == 0 and s["items"] == []
    code, out, s = run(env, capsys, os.path.join(env.models, "qwen3.5-4b/Qwen3.5-4B-UD-Q4_K_XL.gguf"))
    assert code == 0, out
    assert s["items"][0]["status"] == "done" and s["items"][0]["config"] == "present"
    assert "already in the config" in out
    assert open(env.manifest, "rb").read() == mani_bytes and env.text() == cfg_bytes


def test_duplicate_is_reported_and_left_alone(env, capsys):
    installed = os.path.join(env.models, "qwen3.5-9b", "Qwen3.5-9B-UD-Q4_K_XL.gguf")
    src = os.path.join(env.downloads, "Qwen3.5-9B-UD-Q4_K_XL.gguf")
    shutil.copyfile(installed, src)
    before_cfg, before_mani = env.text(), open(env.manifest, "rb").read()
    code, out, s = run(env, capsys)
    assert code == 0, out
    it = by_name(s)["Qwen3.5-9B-UD-Q4_K_XL.gguf"]
    assert it["status"] == "duplicate"
    assert f"duplicate of {installed}" in out
    assert os.path.isfile(src)                                          # never deleted
    assert env.text() == before_cfg and open(env.manifest, "rb").read() == before_mani


def test_partial_files_are_skipped(env, capsys):
    part = os.path.join(env.downloads, "Unconfirmed 370048.crdownload")
    write_gguf(part, qwen("Qwen3.5-4B", "4B"))
    growing = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    stop = threading.Event()

    def grow():
        with open(growing, "ab") as f:
            while not stop.is_set():
                f.write(b"\0" * 1024)
                f.flush()
                time.sleep(0.02)
    t = threading.Thread(target=grow, daemon=True)
    t.start()
    try:
        code = bi.main([a if a != "0" else "0.4" for a in env.args()])          # --settle 0.4
    finally:
        stop.set()
        t.join()
    out = capsys.readouterr().out
    s = json.loads(out.strip().splitlines()[-1])
    items = by_name(s)
    assert code == 0, out
    assert items["Unconfirmed 370048.crdownload"]["status"] == "partial"
    assert items["Qwen3.5-4B-UD-Q4_K_XL.gguf"]["status"] == "partial"
    assert "still growing" in out and "partial download" in out
    assert sorted(os.listdir(env.downloads)) == ["Qwen3.5-4B-UD-Q4_K_XL.gguf",
                                                 "Unconfirmed 370048.crdownload"]
    assert not os.path.exists(os.path.join(env.models, "qwen3.5-4b"))
    # naming a partial file on the command line is a failure, not a quiet skip
    code, out, _ = run(env, capsys, part)
    assert code == 1


def test_dry_run_touches_nothing_and_shows_the_plan(env, capsys):
    env.model("qwen3.5-4b/mmproj-F16.gguf", qwen_mm("Qwen3.5-4B"))            # generic name: to be renamed
    env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    env.dl("gemma-4-12b-it-UD-Q4_K_XL.gguf", gemma())                                # no projector anywhere
    snap = tree(env.tmp)
    code, out, s = run(env, capsys, "--dry-run", fetch=True)
    assert tree(env.tmp) == snap                                         # not one byte, not one mtime
    assert code == 0, out
    assert "DRY RUN" in out and s["dry_run"] is True
    assert '|   "qwen3.5-4b":' in out and '|   "gemma-4-12b":' in out
    renamed = os.path.join(env.models, "qwen3.5-4b", "mmproj-Qwen3.5-4B-F16.gguf")
    assert f"rename mmproj-F16.gguf -> {renamed}" in out
    assert "download unsloth/gemma-4-12b-it-GGUF mmproj-F16.gguf --local-dir " + \
        os.path.join(env.models, "gemma-4-12b") in out
    assert "manifest  + " in out and "would run: systemctl --user restart llama-swap" in out


def test_generic_projector_in_the_folder_is_renamed(env, capsys):
    env.model("qwen3.5-4b/mmproj-F16.gguf", qwen_mm("Qwen3.5-4B"))
    env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    code, out, _ = run(env, capsys)
    assert code == 0, out
    assert sorted(os.listdir(os.path.join(env.models, "qwen3.5-4b"))) == \
        ["Qwen3.5-4B-UD-Q4_K_XL.gguf", "mmproj-Qwen3.5-4B-F16.gguf"]
    assert "--mmproj ${models_dir}/qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf" in env.text()


def test_copy_keeps_the_source(env, capsys):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    src = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    data = open(src, "rb").read()
    code, out, _ = run(env, capsys, "--copy")
    assert code == 0, out
    assert open(src, "rb").read() == data
    assert open(os.path.join(env.models, "qwen3.5-4b", "Qwen3.5-4B-UD-Q4_K_XL.gguf"), "rb").read() == data


def test_vision_model_without_projector_moves_nothing(env, capsys):
    before, mani = env.text(), open(env.manifest, "rb").read()
    src = env.dl("gemma-4-12b-it-UD-Q4_K_XL.gguf", gemma())
    digest = sha1(src)
    code, out, s = run(env, capsys)
    assert code == 1
    assert env.text() == before and open(env.manifest, "rb").read() == mani   # no stanza for a blind model
    assert sha1(src) == digest                                          # left where it is, not moved
    assert not os.path.exists(os.path.join(env.models, "gemma-4-12b", "gemma-4-12b-it-UD-Q4_K_XL.gguf"))
    assert "hf download unsloth/gemma-4-12b-it-GGUF mmproj-F16.gguf" in out.replace(bi.HF_BIN, "hf")
    assert "(NOT done)" in out and by_name(s)["gemma-4-12b-it-UD-Q4_K_XL.gguf"]["config"] == "blocked"
    # the projector arrives later (in the model's folder): re-running the same command does it all
    env.model("gemma-4-12b/mmproj-gemma-4-12b-it-F16.gguf", gemma_mm())
    code, out, _ = run(env, capsys)
    assert code == 0, out
    assert '"gemma-4-12b":' in env.text() and not os.path.exists(src)


def test_copy_mode_rerun_after_the_projector_arrives_in_downloads(env, capsys):
    """--copy, no projector: nothing moves; the projector lands in Downloads; the
    same command then installs AND configures (it used to say 'duplicate', OK)."""
    src = env.dl("gemma-4-12b-it-UD-Q4_K_XL.gguf", gemma())
    code, out, _ = run(env, capsys, "--copy")
    assert code == 1 and not os.path.exists(os.path.join(env.models, "gemma-4-12b"))
    env.dl("mmproj-F16.gguf", gemma_mm())
    code, out, s = run(env, capsys, "--copy")
    assert code == 0, out
    assert by_name(s)["gemma-4-12b-it-UD-Q4_K_XL.gguf"]["status"] == "done"
    assert '"gemma-4-12b":' in env.text() and os.path.isfile(src)                      # --copy keeps it
    assert os.path.isfile(os.path.join(env.models, "gemma-4-12b", "mmproj-gemma-4-12b-it-F16.gguf"))


def test_unknown_architectures_are_left_alone(env, capsys):
    env.dl("flux-2-klein-9b-Q6_K.gguf", {"general.architecture": "flux", "general.file_type": 18})
    env.dl("mtp-gemma-4-12b-it.gguf", {"general.architecture": "gemma4-assistant",
                                       "general.basename": "gemma-4", "general.size_label": "423M"})
    before = env.text()
    code, out, s = run(env, capsys)
    assert code == 0, out
    assert {it["status"] for it in s["items"]} == {"skipped"}
    assert "MTP draft head" in out
    assert len(os.listdir(env.downloads)) == 2 and env.text() == before
    code, _, _ = run(env, capsys, os.path.join(env.downloads, "flux-2-klein-9b-Q6_K.gguf"))
    assert code == 1                                                    # named explicitly: a failure


def test_explicit_id_that_is_taken_fails_before_moving(env, capsys):
    src = env.dl("Qwen3.5-9B-UD-Q6_K_XL.gguf", qwen("Qwen3.5-9B", "9B"))
    code, out, _ = run(env, capsys, "--id", "qwen3.5-9b")
    assert code == 1 and "already configured" in out
    assert os.path.isfile(src)


def test_naming_an_installed_projector_is_a_quiet_no_op(env, capsys):
    mm = env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    snap = tree(env.tmp)
    code, out, s = run(env, capsys, mm)
    assert code == 0, out
    assert s["items"][0]["status"] == "installed" and s["items"][0]["is_mmproj"]
    assert "duplicate" not in out and tree(env.tmp) == snap


def _fake_restart(monkeypatch, tmp_path, listed_after):
    """systemctl and llama-swap replaced by fakes: the real service is never touched."""
    calls = []
    real_run = subprocess.run

    def fake_run(cmd, *a, **k):
        if cmd[:1] == ["systemctl"]:
            calls.append(cmd)
            return subprocess.CompletedProcess(cmd, 0, "", "")
        return real_run(cmd, *a, **k)
    monkeypatch.setattr(bi.subprocess, "run", fake_run)
    fake = types.SimpleNamespace(running_models=lambda: {"qwen3.6-35b-a3b"},
                                 listed_models=lambda: set(listed_after) if calls else set())
    monkeypatch.setitem(sys.modules, "vision_bench", fake)
    monkeypatch.setattr(bi, "HF_BIN", str(tmp_path / "no-such-hf"))
    monkeypatch.setattr(bi, "RESTART_WAIT_S", 1.5)
    return calls


@pytest.mark.parametrize("listed_ok", [True, False])
def test_restart_waits_for_the_new_id(tmp_path, monkeypatch, capsys, listed_ok):
    e = Env(str(tmp_path))
    calls = _fake_restart(monkeypatch, tmp_path, {"qwen3.5-4b", "embedding"} if listed_ok else {"embedding"})
    e.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    e.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    args = [a for a in e.args() if a != "--no-restart"]
    code = bi.main(args)
    out = capsys.readouterr().out
    assert calls == [["systemctl", "--user", "restart", "llama-swap"]]
    s = json.loads(out.strip().splitlines()[-1])
    assert s["restart"]["unloaded"] == ["qwen3.6-35b-a3b"]
    if listed_ok:
        assert code == 0 and "llama-swap restarted" in out and s["restart"]["listed"] == ["qwen3.5-4b"]
    else:
        assert code == 1 and "did not list qwen3.5-4b" in out


# ------------------------------------------------------------------ the REAL config (copies only)
def _real_copies(env):
    if not (os.path.isfile(REAL_CONFIG) and os.path.isfile(REAL_MANIFEST)):
        pytest.skip("no real llama-swap config on this machine")
    shutil.copyfile(REAL_CONFIG, env.config)
    shutil.copyfile(REAL_MANIFEST, env.manifest)


def _expected_gemma_id():
    """gemma-4-12b until the owner installs one; then the quant-suffixed id; a
    test id if even that is taken (the real config changes under this test)."""
    cfg = yaml.safe_load(open(REAL_CONFIG))
    taken = bi.config_models(cfg, (cfg.get("macros") or {}).get("models_dir", ""))
    for mid in ("gemma-4-12b", "gemma-4-12b-q4k"):
        if mid not in taken:
            return mid, ()
    return "gemma-4-12b-selftest", ("--id", "gemma-4-12b-selftest")


def test_dry_run_against_a_copy_of_the_real_config(env, capsys):
    _real_copies(env)
    expected, extra = _expected_gemma_id()
    env.model(f"{expected}/mmproj-gemma-4-12b-it-F16.gguf", gemma_mm())
    env.dl("gemma-4-12b-it-UD-Q4_K_XL.gguf", gemma())
    code, out, s = run(env, capsys, "--dry-run", *extra)
    assert code == 0, out
    it = by_name(s)["gemma-4-12b-it-UD-Q4_K_XL.gguf"]
    assert it["id"] == expected and it["config"] == "add"
    assert "--image-min-tokens 140" in it["stanza"] and "--image-min-tokens 140" in out
    assert "vision (mmproj)" in it["stanza"]


def test_real_config_copy_takes_the_stanza_and_keeps_its_groups(env, capsys):
    _real_copies(env)
    expected, extra = _expected_gemma_id()
    before = env.text()
    env.model(f"{expected}/mmproj-gemma-4-12b-it-F16.gguf", gemma_mm())
    env.dl("gemma-4-12b-it-UD-Q4_K_XL.gguf", gemma())
    code, out, _ = run(env, capsys, *extra)
    assert code == 0, out
    after = env.text()
    assert after[after.index("\ngroups:"):] == before[before.index("\ngroups:"):]
    assert after.index(f'"{expected}":') < after.index(bi.MARKER) < after.index('"embedding":')
    old, new = yaml.safe_load(before), yaml.safe_load(after)
    assert all(old[k] == new[k] for k in ("groups", "selectors", "macros"))
    assert set(new["models"]) - set(old["models"]) == {expected}
    assert new["groups"]["resident"]["members"] == old["groups"]["resident"]["members"]


def test_cli_entrypoint_dry_run_json(env):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    r = subprocess.run([sys.executable, os.path.join(ROOT, "sim", "brain_install.py"), "--dry-run",
                        *env.args()], capture_output=True, text=True, timeout=60,
                       env={**os.environ, "ROCKY_HF_BIN": os.path.join(env.tmp, "no-such-hf")})
    assert r.returncode == 0, r.stdout + r.stderr
    s = json.loads(r.stdout.strip().splitlines()[-1])
    assert s["ok"] and s["dry_run"] and s["items"][0]["id"] == "qwen3.5-4b"
    assert os.path.isfile(os.path.join(env.downloads, "Qwen3.5-4B-UD-Q4_K_XL.gguf"))


# ------------------------------------------------------------------ completeness (review 2026-09-25)
def test_reader_computes_the_tensor_extent_and_catches_truncation(tmp_path):
    p = write_gguf(str(tmp_path / "t.gguf"), qwen(), pad=0, tensors=TENSORS)
    h = bi.check_complete(p)
    assert h["exact"] and h["data_end"] == os.path.getsize(p)             # the file ends at the last tensor
    assert h["data_end"] - h["data_start"] == 84 + 12 + 136               # F32 7x3, pad to 32, Q8_0 64x2
    raw = open(p, "rb").read()
    cut = tmp_path / "cut.gguf"
    cut.write_bytes(raw[:-1])
    bi.read_gguf_header(str(cut))                                         # the KV header alone still parses
    with pytest.raises(bi.GGUFIncomplete, match="incomplete"):
        bi.check_complete(str(cut))
    head = tmp_path / "head.gguf"                                         # cut inside the tensor table
    head.write_bytes(raw[:h["data_start"] - 40])
    with pytest.raises(bi.GGUFIncomplete):
        bi.check_complete(str(head))
    # a tensor type newer than the table: only a lower bound, never a false alarm on a whole file
    u = write_gguf(str(tmp_path / "u.gguf"), qwen(), pad=0, tensors=TENSORS)
    blob = bytearray(open(u, "rb").read())
    i = blob.index(b"blk.0.q") + len(b"blk.0.q") + 4 + 16                # past name, n_dims, 2 dims
    blob[i:i + 4] = struct.pack("<I", 999)
    open(u, "wb").write(bytes(blob))
    h = bi.check_complete(u)
    assert h["exact"] is False and h["data_end"] <= os.path.getsize(u)


def test_truncated_download_under_its_final_name_is_partial(env, capsys):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    src = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), pad=0, tensors=TENSORS)
    with open(src, "r+b") as f:                                           # a stalled wget / curl -o
        f.truncate(os.path.getsize(src) - 100)
    digest, cfg = sha1(src), env.text()
    code, out, s = run(env, capsys)
    assert code == 0, out                                                 # not named: a partial, not a failure
    it = by_name(s)["Qwen3.5-4B-UD-Q4_K_XL.gguf"]
    assert it["status"] == "partial" and "incomplete" in out
    assert sha1(src) == digest and env.text() == cfg
    assert not os.path.exists(os.path.join(env.models, "qwen3.5-4b", "Qwen3.5-4B-UD-Q4_K_XL.gguf"))
    code, out, _ = run(env, capsys, src)                                  # named: a failure
    assert code == 1 and sha1(src) == digest


def test_browser_placeholders_aria2_and_junk_do_not_fail_the_run(env, capsys):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    ff = os.path.join(env.downloads, "Qwen3.5-4B-UD-Q4_K_XL.gguf")
    open(ff, "wb").close()                                                # Firefox: 0-byte X.gguf ...
    write_gguf(ff + ".part", qwen("Qwen3.5-4B", "4B"))                    # ... and the data in X.gguf.part
    ar = env.dl("gemma-4-12b-it-UD-Q4_K_XL.gguf", gemma())                # aria2c: final name + control file
    open(ar + ".aria2", "wb").write(b"\0" * 16)
    junk = os.path.join(env.downloads, "not-really.gguf")
    open(junk, "wb").write(b"<html>rate limited</html>" + b"\0" * 64)
    code, out, s = run(env, capsys)
    assert code == 0 and s["ok"], out
    items = by_name(s)
    assert items["Qwen3.5-4B-UD-Q4_K_XL.gguf"]["status"] == "partial"
    assert items["Qwen3.5-4B-UD-Q4_K_XL.gguf.part"]["status"] == "partial"
    assert items["gemma-4-12b-it-UD-Q4_K_XL.gguf"]["status"] == "partial"
    assert items["not-really.gguf"]["status"] == "skipped" and items["not-really.gguf"]["error"] is None
    assert os.listdir(os.path.join(env.models, "qwen3.5-4b")) == ["mmproj-Qwen3.5-4B-F16.gguf"]
    assert not os.path.exists(os.path.join(env.models, "gemma-4-12b"))
    code, _, _ = run(env, capsys, junk)                                   # named: a failure
    assert code == 1


def test_split_gguf_is_refused_not_half_installed(env, capsys):
    one = env.dl("Qwen3.5-4B-UD-Q4_K_XL-00001-of-00002.gguf",
                 {**qwen("Qwen3.5-4B", "4B"), "split.no": 0, "split.count": 2})
    env.dl("Qwen3.5-4B-UD-Q4_K_XL-00002-of-00002.gguf", {"split.no": 1, "split.count": 2})
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    before = env.text()
    code, out, s = run(env, capsys)
    assert code == 0, out
    assert {it["status"] for it in s["items"]} == {"skipped"} and "split GGUF (shard 1 of 2)" in out
    assert len(os.listdir(env.downloads)) == 2 and env.text() == before
    code, out, _ = run(env, capsys, one)
    assert code == 1 and "split GGUF" in out and os.path.isfile(one)


# ------------------------------------------------------------------ nothing moves before it can finish
def test_dry_run_catches_a_config_the_stanza_cannot_go_into(env, capsys):
    with open(env.config) as f:
        text = f.read().replace("RETRIEVAL STACK", "RETRIEVAL THINGS")
    with open(env.config, "w") as f:
        f.write(text)
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    src = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    digest, mani = sha1(src), open(env.manifest, "rb").read()
    code, out, s = run(env, capsys, "--dry-run")
    assert code == 1 and "RETRIEVAL STACK" in out and "result    FAILED" in out
    assert by_name(s)["Qwen3.5-4B-UD-Q4_K_XL.gguf"]["config"] == "invalid"
    code, out, _ = run(env, capsys)                                       # the real run moves nothing
    assert code == 1
    assert sha1(src) == digest and open(env.manifest, "rb").read() == mani and env.text() == text
    assert not os.path.exists(os.path.join(env.models, "qwen3.5-4b", "Qwen3.5-4B-UD-Q4_K_XL.gguf"))


def test_a_bad_id_is_refused(env, capsys):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    src = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    snap = tree(env.tmp)
    for bad in ("../escape", 'a"b', "x: y"):
        code, out, _ = run(env, capsys, "--id", bad)
        assert code == 1 and "--id" in out, (bad, out)
    assert tree(env.tmp) == snap and os.path.isfile(src)


def test_a_failed_fetch_moves_nothing_and_a_rerun_finishes(env, capsys):
    src = env.dl("gemma-4-12b-it-UD-Q4_K_XL.gguf", gemma())
    digest, cfg, mani = sha1(src), env.text(), open(env.manifest, "rb").read()
    code, out, s = run(env, capsys, fetch=True)                           # HF_BIN does not exist
    assert code == 1 and "nothing moved" in out
    assert sha1(src) == digest and env.text() == cfg and open(env.manifest, "rb").read() == mani
    assert not os.path.exists(os.path.join(env.models, "gemma-4-12b", "gemma-4-12b-it-UD-Q4_K_XL.gguf"))
    env.dl("mmproj-F16.gguf", gemma_mm())                                 # the owner fetched it by hand
    code, out, _ = run(env, capsys, fetch=True)
    assert code == 0, out
    assert '"gemma-4-12b":' in env.text() and os.listdir(env.downloads) == []


def test_failure_after_the_copy_keeps_the_source_and_the_same_command_finishes(env, capsys, monkeypatch):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    src = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 150)
    dest = os.path.join(env.models, "qwen3.5-4b", "Qwen3.5-4B-UD-Q4_K_XL.gguf")
    digest, cfg = sha1(src), env.text()
    real = bi.Installer._write_manifest
    boom = {"n": 1}

    def flaky(self, it):
        if boom["n"]:
            boom["n"] -= 1
            raise OSError(28, "No space left on device")
        return real(self, it)
    monkeypatch.setattr(bi.Installer, "_write_manifest", flaky)
    code, out, s = run(env, capsys)
    assert code == 1
    assert sha1(src) == digest and sha1(dest) == digest                   # both: the source was NOT unlinked
    assert env.text() == cfg and "NOT configured" in out
    assert f"finish with:  ./rocky.sh brain-install {dest}" in out
    # the same command again: the installed copy is a duplicate with no stanza -> finished in place
    code, out, s = run(env, capsys)
    assert code == 0, out
    it = by_name(s)["Qwen3.5-4B-UD-Q4_K_XL.gguf"]
    assert (it["status"], it["mode"], it["dest"]) == ("done", "in-place", dest)
    assert "installed but NOT configured" in out and '"qwen3.5-4b":' in env.text()
    assert sha1(src) == digest                                            # a duplicate: never deleted
    names = [e["filename"] for e in env.mani()["models"]]
    assert names.count("qwen3.5-4b/Qwen3.5-4B-UD-Q4_K_XL.gguf") == 1
    # and once configured it is a plain duplicate
    code, out, s = run(env, capsys)
    assert code == 0 and by_name(s)["Qwen3.5-4B-UD-Q4_K_XL.gguf"]["status"] == "duplicate"


# ------------------------------------------------------------------ the copy's guards, one by one
def test_existing_destination_with_a_different_size_is_refused_before_anything(env, capsys):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    dest = env.model("qwen3.5-4b/Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 500)
    src = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 200)
    d_src, d_dest, cfg, mani = sha1(src), sha1(dest), env.text(), open(env.manifest, "rb").read()
    code, out, _ = run(env, capsys, "--dry-run")                          # caught at PLAN time
    assert code == 1 and "already exists with a different size" in out
    code, out, _ = run(env, capsys)
    assert code == 1 and "already exists with a different size" in out
    assert sha1(src) == d_src and sha1(dest) == d_dest
    assert env.text() == cfg and open(env.manifest, "rb").read() == mani


def _no_part_files(root):
    return not [f for _, _, fs in os.walk(root) for f in fs if f.endswith(".brain-install.part")]


def test_short_copy_leaves_the_source_and_no_destination(env, capsys, monkeypatch):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    src = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 200)
    digest, cfg, mani = sha1(src), env.text(), open(env.manifest, "rb").read()

    def short(a, b, *k, **kw):                                            # a copy that lost its tail
        data = open(a, "rb").read()
        open(b, "wb").write(data[:-10])
    monkeypatch.setattr(bi.shutil, "copyfile", short)
    code, out, _ = run(env, capsys)
    monkeypatch.undo()
    assert code == 1 and "expected" in out and "nothing moved" in out
    assert sha1(src) == digest and env.text() == cfg and open(env.manifest, "rb").read() == mani
    assert not os.path.exists(os.path.join(env.models, "qwen3.5-4b", "Qwen3.5-4B-UD-Q4_K_XL.gguf"))
    assert _no_part_files(env.models)


def test_corrupt_copy_of_the_right_size_is_refused(env, capsys, monkeypatch):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    src = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 200)
    digest = sha1(src)

    def garbled(a, b, *k, **kw):                                          # right size, wrong bytes
        data = open(a, "rb").read()
        open(b, "wb").write(b"XXXX" + data[4:])
    monkeypatch.setattr(bi.shutil, "copyfile", garbled)
    code, out, _ = run(env, capsys)
    monkeypatch.undo()
    assert code == 1 and "not a GGUF file" in out
    assert sha1(src) == digest and _no_part_files(env.models)
    assert not os.path.exists(os.path.join(env.models, "qwen3.5-4b", "Qwen3.5-4B-UD-Q4_K_XL.gguf"))


def test_copy_verified_refuses_an_existing_destination_without_copying(tmp_path, monkeypatch):
    src = write_gguf(str(tmp_path / "a.gguf"), qwen(), 100)
    dest = write_gguf(str(tmp_path / "b.gguf"), qwen(), 50)
    before, calls = sha1(dest), []
    monkeypatch.setattr(bi.shutil, "copyfile", lambda *a, **k: calls.append(a))
    with pytest.raises(RuntimeError, match="already exists"):
        bi.copy_verified(src, dest, os.path.getsize(src))
    assert calls == [] and sha1(dest) == before and _no_part_files(str(tmp_path))


def test_source_is_unlinked_only_after_the_copy_and_its_folder_are_fsynced(env, capsys, monkeypatch):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    src = env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 200)
    dest = os.path.join(env.models, "qwen3.5-4b", "Qwen3.5-4B-UD-Q4_K_XL.gguf")
    events, real_fsync, real_unlink = [], bi.fsync_path, os.unlink

    def fsync(p):
        events.append(("fsync", os.path.abspath(p)))
        real_fsync(p)

    def unlink(p, *a, **k):
        events.append(("unlink", os.path.abspath(p)))
        real_unlink(p, *a, **k)
    monkeypatch.setattr(bi, "fsync_path", fsync)
    monkeypatch.setattr(bi.os, "unlink", unlink)
    code = bi.main(env.args())
    monkeypatch.undo()
    out = capsys.readouterr().out
    assert code == 0, out
    assert not os.path.exists(src) and os.path.isfile(dest)
    gone = events.index(("unlink", src))
    assert events.index(("fsync", dest + ".brain-install.part")) < gone
    assert events.index(("fsync", os.path.dirname(dest))) < gone
    assert events.index(("fsync", os.path.dirname(env.config))) < gone      # the config is written first


# ------------------------------------------------------------------ the launcher
def test_launcher_resolves_a_relative_file_from_the_callers_directory(env):
    if not os.access(os.path.join(ROOT, ".venv", "bin", "python"), os.X_OK):
        pytest.skip("no venv: rocky.sh refuses to run")
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"))
    snap = tree(env.tmp)
    child_env = {k: v for k, v in os.environ.items() if k != "ROCKY_REPO"}
    child_env["ROCKY_HF_BIN"] = os.path.join(env.tmp, "no-such-hf")
    r = subprocess.run(["bash", os.path.join(ROOT, "rocky.sh"), "brain-install", "--dry-run",
                        "Qwen3.5-4B-UD-Q4_K_XL.gguf", *env.args()], cwd=env.downloads,
                       capture_output=True, text=True, timeout=60, env=child_env)
    assert r.returncode == 0, r.stdout + r.stderr
    s = json.loads(r.stdout.strip().splitlines()[-1])
    assert s["items"][0]["source"] == os.path.join(env.downloads, "Qwen3.5-4B-UD-Q4_K_XL.gguf")
    assert s["items"][0]["id"] == "qwen3.5-4b" and tree(env.tmp) == snap


# ------------------------------------------------------------------ --uninstall (2026-09-25)
def _install_three(env, capsys):
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    env.dl("mmproj-F16.gguf", gemma_mm(), pad=77)
    env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 200)
    env.dl("gemma-4-12b-it-UD-Q4_K_XL.gguf", gemma(), 210)
    env.dl("Qwen3.5-9B-UD-Q6_K_XL.gguf", qwen("Qwen3.5-9B", "9B"), 220)
    code, out, _ = run(env, capsys)
    assert code == 0, out


def test_uninstall_cuts_exactly_one_stanza_and_its_manifest_entries(env, capsys):
    _install_three(env, capsys)
    before = env.text()
    cfg_before = yaml.safe_load(before)
    code, out, s = run(env, capsys, "--uninstall", "qwen3.5-9b-q6k")
    assert code == 0, out
    after = env.text(); cfg = yaml.safe_load(after)
    assert "qwen3.5-9b-q6k" not in cfg["models"] and set(cfg_before["models"]) - set(cfg["models"]) == {"qwen3.5-9b-q6k"}
    for other in cfg["models"]:
        assert cfg["models"][other] == cfg_before["models"][other]
    assert cfg["groups"] == cfg_before["groups"] and bi.MARKER in after
    assert "Qwen3.5 9B, UNMEASURED" not in after            # the stanza's own comment header went with it
    assert after.count("\n\n\n") == 0                      # no double blank line left behind
    assert not any(e["filename"].startswith("qwen3.5-9b-q6k/") for e in env.mani()["models"])
    assert any(e["filename"].startswith("qwen3.5-9b/") for e in env.mani()["models"])   # the sibling stays
    assert os.path.isdir(os.path.join(env.models, "qwen3.5-9b-q6k"))                     # files kept by default
    assert s["plan"]["manifest"] == ["qwen3.5-9b-q6k/Qwen3.5-9B-UD-Q6_K_XL.gguf",
                                     "qwen3.5-9b-q6k/mmproj-Qwen3.5-9B-F16.gguf"]
    assert os.path.isfile(s["config_backup"]) and os.path.isfile(s["manifest_backup"])
    # the earlier stanzas are byte-identical text
    for mid in ("qwen3.5-4b", "gemma-4-12b"):
        a, b = bi.stanza_span(before, mid), bi.stanza_span(after, mid)
        assert "".join(before.splitlines(True)[a[0]:a[1]]) == "".join(after.splitlines(True)[b[0]:b[1]])


def test_uninstall_delete_files_removes_the_folder_but_not_the_hard_linked_sibling(env, capsys):
    _install_three(env, capsys)
    sib = os.path.join(env.models, "qwen3.5-9b/mmproj-Qwen3.5-9B-F16.gguf")
    code, out, s = run(env, capsys, "--uninstall", "qwen3.5-9b-q6k", "--delete-files")
    assert code == 0, out
    assert not os.path.exists(os.path.join(env.models, "qwen3.5-9b-q6k"))
    assert os.path.isfile(sib) and bi.read_gguf_header(sib)
    assert s["deleted"] == ["Qwen3.5-9B-UD-Q6_K_XL.gguf", "mmproj-Qwen3.5-9B-F16.gguf"]


def test_uninstall_refuses_unknown_files_in_the_folder_and_unknown_ids(env, capsys):
    _install_three(env, capsys)
    with open(os.path.join(env.models, "qwen3.5-4b", "notes.bin"), "wb") as f:
        f.write(b"x")
    code, out, s = run(env, capsys, "--uninstall", "qwen3.5-4b", "--delete-files")
    assert code == 1 and not s["ok"] and "notes.bin" in s["error"]
    assert "qwen3.5-4b" in yaml.safe_load(env.text())["models"]         # nothing happened
    code, out, s = run(env, capsys, "--uninstall", "nope")
    assert code == 1 and "nothing to uninstall" in s["error"]


def test_uninstall_dry_run_touches_nothing(env, capsys):
    _install_three(env, capsys)
    snap = (env.text(), env.mani(), tree(env.models))
    code, out, s = run(env, capsys, "--uninstall", "gemma-4-12b", "--delete-files", "--dry-run")
    assert code == 0 and s["dry_run"] and "DRY RUN" in out
    assert (env.text(), env.mani(), tree(env.models)) == snap


def test_uninstall_restart_waits_for_the_id_to_go(tmp_path, monkeypatch, capsys):
    env = Env(str(tmp_path))
    monkeypatch.setattr(bi, "HF_BIN", str(tmp_path / "no-such-hf"))
    env.dl("Qwen3.5-4B-UD-Q4_K_XL.gguf", qwen("Qwen3.5-4B", "4B"), 200)
    env.model("qwen3.5-4b/mmproj-Qwen3.5-4B-F16.gguf", qwen_mm("Qwen3.5-4B"))
    assert bi.main(env.args()) == 0
    capsys.readouterr()
    calls = []
    monkeypatch.setattr(bi.subprocess, "run", lambda cmd, *a, **k: (calls.append(cmd), types.SimpleNamespace(returncode=0, stderr=""))[1])
    fake_vb = types.SimpleNamespace(listed_models=lambda: {"qwen3.5-9b"}, running_models=lambda: [])
    monkeypatch.setitem(sys.modules, "vision_bench", fake_vb)
    args = [a for a in env.args() if a != "--no-restart"] + ["--uninstall", "qwen3.5-4b"]
    code = bi.main(args)
    out = capsys.readouterr().out
    s = json.loads(out.strip().splitlines()[-1])
    assert code == 0 and calls == [["systemctl", "--user", "restart", "llama-swap"]]
    assert "gone from /v1/models" in s["restart"]
