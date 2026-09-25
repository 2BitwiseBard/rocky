"""brain_install — put a downloaded GGUF where the rocky brains expect it, and tell llama-swap.

    .venv/bin/python sim/brain_install.py --dry-run                 # the plan for ~/Downloads/*.gguf
    .venv/bin/python sim/brain_install.py                           # do it (+ restart llama-swap)
    .venv/bin/python sim/brain_install.py ~/Downloads/Qwen3.5-4B-UD-Q4_K_XL.gguf --no-restart
    .venv/bin/python sim/brain_install.py /mnt/models/qwen3.5-4b/Qwen3.5-4B-UD-Q4_K_XL.gguf   # finish one
    rocky.sh brain-install --dry-run                                # the same, from the launcher

The owner downloads brain candidates with a browser (Qwen3.5-4B / 9B, Gemma 4
12B, ...) and every one needs the same five chores done by hand: move it off
the OS disk, find or fetch its vision projector, record it in the restore
manifest, write a llama-swap stanza that follows the family's hard-won rules,
and restart llama-swap. This does exactly those, in that order, and nothing
else: it never loads a model (benching is sim/brain_bench.py's job) and it
never touches the groups:/selectors:/macros: sections.

CONVENTIONS (set 2026-09-24 with qwen3.5-9b)
  * one folder per model id: <models-dir>/<id>/<original filename>, plus
    <models-dir>/<id>/mmproj-<base>-F16.gguf for a vision model, where <base>
    is the model filename without its quant (Qwen3.5-4B-UD-Q4_K_XL.gguf ->
    Qwen3.5-4B, gemma-4-12b-it-UD-Q4_K_XL.gguf -> gemma-4-12b-it). Hugging Face
    ships the projector as a generic mmproj-F16.gguf, so it is always renamed.
  * the id is family + size, lowercase, from the GGUF header (general.basename
    + general.size_label): Qwen3.5-4B -> qwen3.5-4b, Gemma-4-12B-It ->
    gemma-4-12b, LFM2.5-VL 3B -> lfm2.5-vl-3b. --id overrides. If the id is
    already configured for a DIFFERENT file, the quant is appended
    (UD-Q6_K_XL -> qwen3.5-9b-q6k, Q8_0 -> -q8, IQ4_XS -> -iq4).
  * families with a template: qwen35 (Qwen3.5: thinking off at the template,
    Qwen's non-thinking sampling), gemma4 (Gemma 4: image tokens 140..280,
    because Gemma decodes an image non-causally in ONE ubatch and more than
    512 without raising -b/-ub aborts the server), lfm2 (plain). Qwen3.5 and
    Gemma 4 are always vision; LFM2 only when the name says VL. Anything else
    (an MTP draft head, a diffusion GGUF, a qwen35moe that needs a measured
    -ncmoe) is reported and left where it is.
  * every stanza is UNMEASURED (the description says so, with the file sizes),
    ttl 1800, -c 16384, -ngl 99, ${common_flags}, and goes right before the
    RETRIEVAL STACK comment block. A vision stanza's name says "vision
    (mmproj)": the cockpit decides a model can see from that text.

SAFETY
  * a file is used only when complete: *.crdownload / *.part are partial; so
    is X.gguf with an X.gguf.part / .crdownload / .aria2 beside it or with 0
    bytes (a browser's placeholder), any file whose size changes over a
    --settle (2 s) recheck, and any GGUF whose own tensor table ends past the
    end of the file (a stalled or truncated download under its final name);
  * a split GGUF (-00001-of-0000N, split.count > 1) is refused: install the
    shards by hand, together;
  * the same filename with the same size already under <models-dir> is a
    duplicate: reported, never moved, never deleted (that is the owner's
    call). If no stanza runs that installed copy (an earlier run stopped half
    way), the install is finished in place from it;
  * nothing moves until the install can complete: the plan finds the
    projector first and applies every stanza to an in-memory copy of the
    config (yaml.safe_load; groups/selectors/macros unchanged; exactly one
    model more), so a config problem fails its file in --dry-run too;
  * install = the projector first (a fetch is what fails most often), then
    shutil.copyfile to a temp name beside the destination (Downloads and
    /mnt/models are different drives), byte-size check, fsync, header +
    tensor-extent re-check, link into place (never overwrites), fsync of the
    folder, then the manifest and the config; the source is unlinked (unless
    --copy) only after all of that worked. A failure after the copy keeps the
    source and prints the command that finishes the install;
  * the mmproj comes from, in order: the model's folder, --downloads (a
    generic mmproj whose header names the same base model), another folder
    under <models-dir> that already holds it (hard link: same bytes, no disk),
    or `hf download <repo> mmproj-F16.gguf` (unless --no-fetch). A vision
    model with no projector is left where it is (no move, no stanza) and the
    run fails, so the cockpit never meets a "vision" model that cannot see;
  * config: a dated backup first (<config>.bak-YYYYMMDD-<id>), the edit is
    validated in memory again before it is written and re-checked after;
  * restart (unless --no-restart, and only when the config changed):
    `systemctl --user restart llama-swap` — it unloads EVERY model — then GET
    /v1/models (bearer LOCAL_AI_KEY) until the new ids are listed, up to 60 s;
  * --dry-run prints the whole plan (moves, stanza text, manifest entries) and
    touches nothing. No prompts ever; exit 1 on any failure, 0 otherwise.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shlex
import shutil
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, field

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

HOME = os.path.expanduser("~")
DEFAULT_DOWNLOADS = os.path.join(HOME, "Downloads")
DEFAULT_MODELS_DIR = "/mnt/models"
DEFAULT_CONFIG = os.path.join(HOME, ".config", "llama-swap", "config.yaml")
DEFAULT_MANIFEST = os.path.join(HOME, ".config", "llama-swap", "models.manifest.json")
HF_BIN = os.environ.get("ROCKY_HF_BIN", os.path.join(HOME, ".local", "bin", "hf"))
SETTLE_S = 2.0                   # a file whose size changes over this recheck is still downloading
RESTART_WAIT_S = 60.0
FREE_MARGIN_B = 512 * 1024**2    # leave this much on the models drive after a copy
VRAM_WARN_B = 14.0e9             # weights + projector above this cannot sit on the 16 GB card at -ngl 99
PARTIAL_EXT = (".crdownload", ".part", ".partial", ".download")
PARTIAL_SIBLING = PARTIAL_EXT + (".aria2",)   # X.gguf + X.gguf.part (Firefox) / X.gguf.aria2 (aria2c)
MARKER = "RETRIEVAL STACK"
PROTECTED = ("groups", "selectors", "macros")
TEMPLATE_FAMILIES = ("qwen35", "gemma4", "lfm2")
FAMILY_TITLE = {"qwen35": "Qwen3.5", "gemma4": "Gemma 4", "lfm2": "LFM2"}


# ------------------------------------------------------------------ GGUF header (pure Python)
class GGUFError(ValueError):
    pass


class GGUFIncomplete(GGUFError):
    """The file ends inside its header or before its tensor data does: a partial download."""


_SCALAR = {0: ("<B", 1), 1: ("<b", 1), 2: ("<H", 2), 3: ("<h", 2), 4: ("<I", 4), 5: ("<i", 4),
           6: ("<f", 4), 7: ("<?", 1), 10: ("<Q", 8), 11: ("<q", 8), 12: ("<d", 8)}
T_STRING, T_ARRAY = 8, 9
MAX_KEY, MAX_STR, MAX_COUNT, MAX_KV = 1 << 16, 1 << 26, 1 << 28, 1 << 20
MAX_DIMS = 4                     # GGML_MAX_DIMS
# ggml tensor type -> (elements per block, bytes per block), from the static_asserts in
# ggml/src/ggml-common.h at the pinned llama.cpp 36b10154 (4/5 and 31-33/36-38 are retired ids)
_GGML_TYPES = {0: (1, 4), 1: (1, 2), 2: (32, 18), 3: (32, 20), 6: (32, 22), 7: (32, 24), 8: (32, 34),
               9: (32, 36), 10: (256, 84), 11: (256, 110), 12: (256, 144), 13: (256, 176), 14: (256, 210),
               15: (256, 292), 16: (256, 66), 17: (256, 74), 18: (256, 98), 19: (256, 50), 20: (32, 18),
               21: (256, 110), 22: (256, 82), 23: (256, 136), 24: (1, 1), 25: (1, 2), 26: (1, 4), 27: (1, 8),
               28: (1, 8), 29: (256, 56), 30: (1, 2), 34: (256, 54), 35: (256, 66), 39: (32, 17),
               40: (64, 36), 41: (128, 18), 42: (64, 18)}


def _take(f, n):
    b = f.read(n)
    if len(b) != n:
        raise GGUFIncomplete("truncated header")
    return b


def _u(f, fmt, n):
    return struct.unpack(fmt, _take(f, n))[0]


def _string(f, limit=MAX_STR):
    n = _u(f, "<Q", 8)
    if n > limit:
        raise GGUFError(f"a {n}-byte string: corrupt header")
    return _take(f, n).decode("utf-8", "replace")


def _value(f, vtype):
    """One GGUF value. Arrays are skipped (tokenizer vocabularies are
    megabytes and nothing here needs them): the return is None."""
    if vtype in _SCALAR:
        fmt, n = _SCALAR[vtype]
        return _u(f, fmt, n)
    if vtype == T_STRING:
        return _string(f)
    if vtype == T_ARRAY:
        etype, count = _u(f, "<I", 4), _u(f, "<Q", 8)
        if count > MAX_COUNT:
            raise GGUFError(f"an array of {count} items: corrupt header")
        if etype in _SCALAR:
            f.seek(_SCALAR[etype][1] * count, 1)
        elif etype == T_STRING:
            for _ in range(count):
                n = _u(f, "<Q", 8)
                if n > MAX_STR:
                    raise GGUFError(f"a {n}-byte string: corrupt header")
                f.seek(n, 1)
        elif etype == T_ARRAY:
            for _ in range(count):
                _value(f, T_ARRAY)
        else:
            raise GGUFError(f"unknown array item type {etype}")
        return None
    raise GGUFError(f"unknown value type {vtype}")


def read_gguf_header(path, tensors=False):
    """{'version', 'n_tensors', 'n_kv', 'meta', 'size'}: the KV metadata
    (strings and numbers; arrays skipped). Stops before the tensor infos unless
    tensors=True, which reads them too and adds 'data_start' / 'data_end'
    (where the tensor data begins and ends per the file's own table: aligned
    start + max(offset + bytes)) and 'exact' (False when a tensor type is not in
    _GGML_TYPES: data_end is then only a lower bound). Raises GGUFIncomplete
    when the file ends inside the header or the tensor table, GGUFError on
    anything else that is not a GGUF v2/v3."""
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        magic = f.read(4)
        if magic != b"GGUF":
            raise GGUFError(f"not a GGUF file (magic {magic!r})")
        version = _u(f, "<I", 4)
        if version not in (2, 3):
            raise GGUFError(f"GGUF version {version} (only 2 and 3 are read)")
        n_tensors, n_kv = _u(f, "<Q", 8), _u(f, "<Q", 8)
        if n_kv > MAX_KV or n_tensors > MAX_COUNT:
            raise GGUFError(f"{n_kv} keys / {n_tensors} tensors: corrupt header")
        meta = {}
        for _ in range(n_kv):
            key = _string(f, MAX_KEY)
            val = _value(f, _u(f, "<I", 4))
            if val is not None:
                meta[key] = val
        if f.tell() > size:
            raise GGUFIncomplete("truncated header")
        out = {"version": version, "n_tensors": n_tensors, "n_kv": n_kv, "meta": meta, "size": size}
        if not tensors:
            return out
        align = meta.get("general.alignment", 32)
        if not isinstance(align, int) or isinstance(align, bool) or align <= 0 or align & (align - 1):
            raise GGUFError(f"general.alignment {align!r}: corrupt header")
        end, exact = 0, True
        for _ in range(n_tensors):
            _string(f, MAX_KEY)
            n_dims = _u(f, "<I", 4)
            if n_dims > MAX_DIMS:
                raise GGUFError(f"a {n_dims}-dimensional tensor: corrupt header")
            n = 1
            for _ in range(n_dims):
                n *= _u(f, "<Q", 8)
            ttype, offset = _u(f, "<I", 4), _u(f, "<Q", 8)
            if ttype in _GGML_TYPES:
                blck, tsize = _GGML_TYPES[ttype]
                end = max(end, offset + -(-n // blck) * tsize)
            else:                                      # a type newer than this table: a lower bound
                exact = False
                end = max(end, offset + (1 if n else 0))
        if f.tell() > size:
            raise GGUFIncomplete("truncated tensor table")
        start = -(-f.tell() // align) * align
        out.update(data_start=start, data_end=start + end, exact=exact)
    return out


def check_complete(path):
    """read_gguf_header(tensors=True), raising GGUFIncomplete unless the file
    holds all of its tensor data (llama.cpp's loader refuses such a file too)."""
    h = read_gguf_header(path, tensors=True)
    if h["size"] < h["data_end"]:
        raise GGUFIncomplete(f"incomplete: the file has {h['size']:,} bytes but its tensor data ends at "
                             f"{h['data_end']:,} (a stalled or truncated download?)")
    return h


# ------------------------------------------------------------------ names, families, ids
_QUANT_RE = re.compile(r"(?:UD-)?(?:I?Q\d(?:_[A-Z0-9]+)*|BF16|F16|F32|MXFP4(?:_MOE)?)(?=$|[-_.])", re.I)
_SIZE_RE = re.compile(r"(?<![a-z0-9.])(\d+(?:\.\d+)?[bm](?:-a\d+(?:\.\d+)?[bm])?)(?![a-z0-9])", re.I)
_LABEL_RE = re.compile(r"^\d+(?:\.\d+)?[BM](?:-A\d+(?:\.\d+)?[BM])?$", re.I)
_GENERIC_MMPROJ_RE = re.compile(r"^mmproj-(BF16|F16|F32|Q8_0)(?: \(\d+\))?\.gguf$", re.I)
_SPLIT_RE = re.compile(r"-(\d{5})-of-(\d{5})\.gguf$", re.I)
_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$")      # a folder name and a YAML key
_BROWSER_DUP_RE = re.compile(r" \(\d+\)(?=\.gguf$)", re.I)
# file_type -> the plain llama.cpp name, only for files whose name carries no quant
_FILE_TYPE = {0: "F32", 1: "F16", 2: "Q4_0", 3: "Q4_1", 7: "Q8_0", 8: "Q5_0", 9: "Q5_1", 10: "Q2_K",
              11: "Q3_K_S", 12: "Q3_K_M", 13: "Q3_K_L", 14: "Q4_K_S", 15: "Q4_K_M", 16: "Q5_K_S",
              17: "Q5_K_M", 18: "Q6_K", 30: "IQ4_XS", 32: "BF16", 38: "MXFP4"}


def clean_name(name):
    """The browser's 'X (1).gguf' for a second download of X.gguf -> X.gguf."""
    return _BROWSER_DUP_RE.sub("", os.path.basename(name))


def quant_of(filename, meta=None):
    """'Qwen3.5-4B-UD-Q4_K_XL.gguf' -> 'UD-Q4_K_XL' (the last quant token in
    the name), else the header's general.file_type, else ''."""
    stem = re.sub(r"\.gguf$", "", clean_name(filename), flags=re.I)
    found = list(_QUANT_RE.finditer(stem))
    if found:
        return found[-1].group(0)
    return _FILE_TYPE.get((meta or {}).get("general.file_type"), "")


def file_base(filename):
    """The filename without its quant: 'gemma-4-12b-it-UD-Q4_K_XL.gguf' ->
    'gemma-4-12b-it'. Names the mmproj and the default Hugging Face repo."""
    stem = re.sub(r"\.gguf$", "", clean_name(filename), flags=re.I)
    found = list(_QUANT_RE.finditer(stem))
    if found:
        stem = stem[:found[-1].start()]
    return stem.rstrip("-_.") or stem


def quant_suffix(quant, long=False):
    """'UD-Q6_K_XL' -> 'q6k', 'Q8_0' -> 'q8', 'IQ4_XS' -> 'iq4', 'BF16' ->
    'bf16'; long=True keeps every letter ('q6kxl') for a second collision."""
    q = re.sub(r"^UD-", "", quant or "", flags=re.I).lower()
    if long:
        return re.sub(r"[^a-z0-9]", "", q)
    m = re.match(r"^(i?q)(\d)(_k)?", q)
    if m:
        return f"{m.group(1)}{m.group(2)}{'k' if m.group(3) else ''}"
    return re.sub(r"[^a-z0-9]", "", q)


def _slug(s):
    s = re.sub(r"[\s_/]+", "-", str(s).strip().lower())
    return re.sub(r"-{2,}", "-", re.sub(r"[^a-z0-9.\-]", "", s)).strip("-.")


def size_of(meta, filename, is_mmproj):
    """The model's parameter size, lowercase ('4b', '12b', '26b-a4b'). An
    mmproj's size_label is the PROJECTOR's size, so it is never used there."""
    label = str(meta.get("general.size_label") or "")
    if not is_mmproj and _LABEL_RE.match(label):
        return label.lower()
    for src in (meta.get("general.basename"), meta.get("general.name"),
                meta.get("general.finetune"), file_base(filename)):
        m = _SIZE_RE.search(str(src or ""))
        if m:
            return m.group(1).lower()
    return ""


def base_key(meta, filename, is_mmproj=False):
    """Family + size, lowercase: the model id before any collision suffix,
    and how a projector is matched to its model (both headers name the base
    model: Qwen3.5-9B / Qwen3.5-9B, Gemma-4-12B-It / Gemma-4-12B-It)."""
    raw = meta.get("general.basename") or meta.get("general.name") or file_base(filename)
    b = re.sub(r"-(it|instruct|chat)$", "", _slug(raw))
    size = size_of(meta, filename, is_mmproj)
    if size and not re.search(rf"(^|-){re.escape(size)}($|-)", b):
        b = f"{b}-{size}"
    return b


def family_of(meta, key):
    """qwen35 / gemma4 / lfm2 / unknown. Exact architecture match for a model
    (so gemma4-assistant, an MTP draft head, is NOT gemma4); the base-model
    name for a projector (its architecture is always 'clip')."""
    arch = str(meta.get("general.architecture") or "")
    if arch in TEMPLATE_FAMILIES:
        return arch
    if arch == "clip":
        for fam, pat in (("qwen35", r"^qwen3\.5"), ("gemma4", r"^gemma-?4"), ("lfm2", r"^lfm2")):
            if re.match(pat, key):
                return fam
    return "unknown"


def is_vision_family(family, key):
    if family in ("qwen35", "gemma4"):
        return True
    return family == "lfm2" and bool(re.search(r"(^|-)vl($|-)", key))


def header_info(path, check=False):
    """Everything the installer needs from one GGUF header. check=True also
    reads the tensor table and raises GGUFIncomplete for a truncated file."""
    h = check_complete(path) if check else read_gguf_header(path)
    m = h["meta"]
    arch = str(m.get("general.architecture") or "")
    gtype = str(m.get("general.type") or ("mmproj" if arch == "clip" else "model"))
    is_mmproj = gtype == "mmproj" or arch == "clip"
    name = os.path.basename(path)
    key = base_key(m, name, is_mmproj)
    return {
        "arch": arch, "type": gtype, "is_mmproj": is_mmproj,
        "name": m.get("general.name"), "basename": m.get("general.basename"),
        "size_label": m.get("general.size_label"), "finetune": m.get("general.finetune"),
        "file_type": m.get("general.file_type"),
        "projector": m.get("clip.projector_type") or m.get("clip.vision.projector_type"),
        "audio_projector": m.get("clip.audio.projector_type"),
        "context_length": m.get(f"{arch}.context_length"), "block_count": m.get(f"{arch}.block_count"),
        "n_tensors": h["n_tensors"], "gguf_version": h["version"],
        "split_no": m.get("split.no"), "split_count": m.get("split.count"),
        "key": key, "family": family_of(m, key), "quant": "" if is_mmproj else quant_of(name, m),
    }


def split_of(name, info):
    """'shard 1 of 3' for a split GGUF (every shard carries split.no / split.count;
    only the first carries the model's metadata), else ''."""
    n = (info or {}).get("split_count")
    if isinstance(n, int) and not isinstance(n, bool) and n > 1:
        return f"shard {int((info or {}).get('split_no') or 0) + 1} of {n}"
    m = _SPLIT_RE.search(name)
    if m and int(m.group(2)) > 1:
        return f"shard {int(m.group(1))} of {int(m.group(2))}"
    return ""


# ------------------------------------------------------------------ small helpers
def gb(n):
    return f"{n / 1e9:.1f} GB"


def tilde(p):
    p = str(p)
    return "~" + p[len(HOME):] if p == HOME or p.startswith(HOME + os.sep) else p


def is_partial_name(name):
    return name.lower().endswith(PARTIAL_EXT)


def partial_marker(path):
    """Why X.gguf is still a download in progress although its name is final:
    a sibling X.gguf.part / .crdownload / .aria2 (Firefox keeps a 0-byte X.gguf
    beside the .part; aria2c writes under the final name + a control file), or
    0 bytes. '' when neither."""
    for ext in PARTIAL_SIBLING:
        if os.path.exists(path + ext):
            return f"{os.path.basename(path + ext)} is beside it"
    try:
        if os.path.getsize(path) == 0:
            return "0 bytes (a download placeholder)"
    except OSError:
        pass
    return ""


def fsync_path(path):
    """fsync a file or a folder (the source is unlinked only after the copy is on
    disk: Downloads and /mnt/models are different drives with no shared journal)."""
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def settle(paths, seconds):
    """{path: size} of the paths whose size did NOT change over `seconds`
    (missing files count as changed)."""
    def sizes():
        out = {}
        for p in paths:
            try:
                out[p] = os.path.getsize(p)
            except OSError:
                out[p] = None
        return out
    first = sizes()
    if seconds > 0 and paths:
        time.sleep(seconds)
    second = sizes()
    return {p: s for p, s in second.items() if s is not None and first.get(p) == s}


def under(path, root):
    try:
        return os.path.commonpath([os.path.realpath(path), os.path.realpath(root)]) == os.path.realpath(root)
    except ValueError:
        return False


def model_files(models_dir):
    """Every *.gguf at the top of <models-dir> and one folder down (the
    convention), skipping dot-folders and ComfyUI's store."""
    out = []
    for pat in ("*.gguf", "*/*.gguf"):
        for p in glob.glob(os.path.join(glob.escape(models_dir), pat)):
            rel = os.path.relpath(p, models_dir)
            if rel.split(os.sep)[0].startswith(".") or rel.startswith("comfyui" + os.sep):
                continue
            out.append(p)
    return sorted(out)


def config_models(parsed, models_dir_macro):
    """{id: -m filename basename (or '(alias of X)' / '(selector)')} from a parsed config."""
    out = {}
    for mid, spec in ((parsed or {}).get("models") or {}).items():
        cmd = str((spec or {}).get("cmd") or "")
        m = re.search(r"(?:^|\s)-m\s+(\S+)", cmd)
        path = m.group(1).replace("${models_dir}", models_dir_macro or "") if m else ""
        out[str(mid)] = os.path.basename(path) if path else "(no -m)"
        for alias in (spec or {}).get("aliases") or []:
            out.setdefault(str(alias), f"(alias of {mid})")
    for sid in ((parsed or {}).get("selectors") or {}):
        out.setdefault(str(sid), "(selector)")
    return out


def yq(s):
    """A YAML double-quoted scalar (JSON's escaping is valid YAML)."""
    return json.dumps(str(s), ensure_ascii=False)


# ------------------------------------------------------------------ the plan
@dataclass
class Item:
    source: str
    name: str                          # clean filename (browser ' (1)' suffix dropped)
    status: str = "pending"            # install | in-place | duplicate | partial | skipped | failed | done
    info: dict | None = None
    size: int | None = None
    id: str | None = None
    dest: str | None = None
    mmproj: dict | None = None         # {how, src, dest, name, size, cmd, info}
    manifest: list = field(default_factory=list)
    stanza: str | None = None
    config: str = "none"               # add | added | present | blocked | invalid | none
    notes: list = field(default_factory=list)
    error: str | None = None
    explicit: bool = False             # named on the command line (a skip is then a failure)
    listed: bool = True                # in this run's report (a Downloads projector in FILE mode is not)
    mode: str = "move"                 # move | copy | in-place
    copied: bool = False               # the model's copy is at dest (a later step may still fail)

    def fail(self, why):
        self.status, self.error = "failed", why
        return self

    def summary(self):
        i = self.info or {}
        return {"source": self.source, "name": self.name, "status": self.status, "id": self.id,
                "dest": self.dest, "mode": self.mode, "size_bytes": self.size,
                "family": i.get("family"), "arch": i.get("arch"),
                "gguf_type": i.get("type"), "basename": i.get("basename"), "size_label": i.get("size_label"),
                "quant": i.get("quant"), "is_mmproj": i.get("is_mmproj"),
                "mmproj": ({k: v for k, v in self.mmproj.items() if k != "info"} if self.mmproj else None),
                "manifest_added": [e["filename"] for e in self.manifest], "config": self.config,
                "stanza": self.stanza, "notes": self.notes, "error": self.error}


class Installer:
    def __init__(self, args):
        self.a = args
        self.models_dir = os.path.abspath(os.path.expanduser(args.models_dir))
        self.downloads = os.path.abspath(os.path.expanduser(args.downloads))
        self.config = os.path.abspath(os.path.expanduser(args.config))
        self.manifest_path = os.path.abspath(os.path.expanduser(args.manifest))
        self.date = time.strftime("%Y-%m-%d")
        self.items: list[Item] = []
        self.pool: list[Item] = []             # projectors available to claim (downloads + mmproj FILEs)
        self.taken: dict[str, str] = {}        # id -> filename (config + this run's plan)
        self.config_changed = False
        self.backups: list[str] = []
        self.restart: dict = {"done": False}

    # ---------------- load
    def load(self):
        with open(self.config) as f:
            self.config_text = f.read()
        self.parsed = yaml.safe_load(self.config_text) or {}
        macros = self.parsed.get("macros") or {}
        self.macro_dir = str(macros.get("models_dir") or "")
        self.use_macro = bool(self.macro_dir) and \
            os.path.realpath(self.macro_dir) == os.path.realpath(self.models_dir)
        self.taken = config_models(self.parsed, self.macro_dir)
        self.configured_paths = set()          # realpath of every stanza's -m
        for spec in (self.parsed.get("models") or {}).values():
            m = re.search(r"(?:^|\s)-m\s+(\S+)", str((spec or {}).get("cmd") or ""))
            if m:
                self.configured_paths.add(os.path.realpath(m.group(1).replace("${models_dir}", self.macro_dir)))
        with open(self.manifest_path) as f:
            raw = f.read()
        self.manifest = json.loads(raw)
        self.manifest_nl = raw.endswith("\n")
        self.manifest.setdefault("models", [])

    # ---------------- discover
    def discover(self):
        files = [os.path.abspath(os.path.expanduser(p)) for p in self.a.files]
        cands, partial = [], []
        if files:
            for p in files:
                if not os.path.isfile(p):
                    self.items.append(Item(p, os.path.basename(p), explicit=True).fail(f"no such file: {p}"))
                elif is_partial_name(p):
                    partial.append((p, True, "partial download"))
                elif partial_marker(p):
                    partial.append((p, True, f"still downloading: {partial_marker(p)}"))
                else:
                    cands.append((p, True))
        elif os.path.isdir(self.downloads):
            for e in sorted(os.scandir(self.downloads), key=lambda e: e.name.lower()):
                if not e.is_file():
                    continue
                if is_partial_name(e.name):
                    partial.append((e.path, False, "partial download"))
                elif e.name.lower().endswith(".gguf"):
                    why = partial_marker(e.path)
                    if why:
                        partial.append((e.path, False, f"still downloading: {why}"))
                    else:
                        cands.append((e.path, False))
        else:
            self.items.append(Item(self.downloads, "", explicit=True).fail("no downloads folder"))
        # projectors in Downloads are claimable even when FILEs were named
        extra = []
        if files and os.path.isdir(self.downloads):
            extra = [p for p in glob.glob(os.path.join(glob.escape(self.downloads), "*.gguf"))
                     if p not in {c[0] for c in cands} and not partial_marker(p)]
        installed_mm = [p for p in model_files(self.models_dir)
                        if os.path.basename(p).lower().startswith("mmproj")]
        stable = settle([c[0] for c in cands] + extra + installed_mm, self.a.settle)
        self.stable_installed = {p for p in installed_mm if p in stable}
        for p, explicit, why in partial:
            self.items.append(self._partial(p, explicit, why))
        for p, explicit, listed in [(c, e, True) for c, e in cands] + [(x, False, False) for x in extra]:
            if p not in stable:
                if listed:
                    self.items.append(self._partial(p, explicit, "still growing (size changed over the "
                                                                 f"{self.a.settle:g} s recheck)"))
                continue
            it = Item(p, clean_name(p), size=stable[p], explicit=explicit, listed=listed)
            try:
                it.info = header_info(p, check=True)   # the tensor table must end inside the file
            except GGUFIncomplete as e:
                if listed:
                    self.items.append(self._partial(p, explicit, str(e)))
                continue
            except (GGUFError, OSError) as e:
                if listed:
                    if explicit:
                        it.fail(f"unreadable GGUF header: {e}")
                    else:                              # not ours to judge: note it, do not fail the run
                        it.status = "skipped"
                        it.notes.append(f"unreadable GGUF header ({e}) — not something this installs; "
                                        "left where it is")
                    self.items.append(it)
                continue
            if it.info["is_mmproj"]:
                it.status = "pool"                     # claimable by a model in this run
                self.pool.append(it)
            if listed:
                self.items.append(it)

    def _partial(self, p, explicit, why):
        it = Item(p, os.path.basename(p), status="partial", explicit=explicit)
        try:
            it.size = os.path.getsize(p)
            i = header_info(p)
            it.info = i
            what = f"GGUF {i['arch']} {i.get('basename') or i.get('name') or ''}".rstrip()
        except (GGUFError, OSError):
            what = "no readable GGUF header yet"
        it.notes.append(f"{why} ({gb(it.size or 0)} so far; {what}) — skipped, re-run when it is complete")
        if explicit:
            it.error = "partial file"
        return it

    # ---------------- plan one model
    def plan_model(self, it):
        i = it.info
        split = split_of(it.name, i)
        if split:
            why = (f"a split GGUF ({split}): brain-install moves single-file models only — merge it with "
                   "llama-gguf-split --merge, or put every shard in one folder by hand (-m names the first)")
            if it.explicit:
                return it.fail(why)
            it.status = "skipped"
            it.notes.append(why + " — left where it is")
            return it
        if i["family"] not in TEMPLATE_FAMILIES:
            why = (f"architecture '{i['arch']}' has no brain template ({' / '.join(TEMPLATE_FAMILIES)})"
                   + (" — an MTP draft head, not a model" if i["arch"].endswith("-assistant") else "")
                   + (" — a MoE needs a measured -ncmoe, write its stanza by hand"
                      if i["arch"].endswith("moe") else ""))
            if it.explicit:
                return it.fail(why)
            it.status = "skipped"
            it.notes.append(why + " — left where it is")
            return it
        target = it.source
        if not under(it.source, self.models_dir):
            dup = self._duplicate(it.name, it.size)
            if dup and os.path.realpath(dup) in self.configured_paths:
                it.status = "duplicate"
                it.notes.append(f"duplicate of {dup} (same name, same size) — nothing done; "
                                f"the download is yours to delete")
                return it
            if dup:                                    # an earlier run copied it, then stopped
                it.notes.append(f"duplicate of {dup} (same name, same size), which is installed but NOT "
                                "configured: finishing that install in place; the download is left alone "
                                "(yours to delete)")
                target = dup
        # id
        if under(target, self.models_dir):
            # finishing an install: the id is --id, else the folder's name (the convention), else a
            # configured id that already runs this file (the old flat layout), else the derived one
            # (quant-suffixed when another file holds it, exactly as for a fresh install)
            rel = os.path.relpath(os.path.realpath(target), os.path.realpath(self.models_dir))
            folder = rel.split(os.sep)[0] if os.sep in rel else None
            by_config = sorted(k for k, v in self.taken.items() if v == it.name)
            if self.a.id:
                it.id = self._pick_id(it)
            elif folder and self.taken.get(folder) in (None, it.name):
                it.id = folder
            elif by_config:
                it.id = by_config[0]
            else:
                it.id = self._pick_id(it)
            if it.id is None:
                return it
            it.dest, it.mode, it.status = target, "in-place", "in-place"
        else:
            it.id = self._pick_id(it)
            if it.id is None:
                return it
            it.dest = os.path.join(self.models_dir, it.id, it.name)
            it.mode = "copy" if self.a.copy else "move"
            if os.path.exists(it.dest):
                return it.fail(f"{it.dest} already exists with a different size "
                               f"({os.path.getsize(it.dest)} vs {it.size} bytes) — a re-upload? "
                               "move one aside or pass --id")
            free = shutil.disk_usage(self.models_dir).free
            if free < it.size + FREE_MARGIN_B:
                return it.fail(f"{gb(free)} free on {self.models_dir}, {gb(it.size)} needed")
            it.status = "install"
        present = self.taken.get(it.id) == it.name or it.id in self._config_ids()
        self.taken.setdefault(it.id, it.name)
        # projector: only for a stanza this run writes (a configured id already names its own)
        vision = is_vision_family(i["family"], i["key"])
        if vision and not present:
            it.mmproj = self._find_mmproj(it)
        # manifest
        repo = self.a.hf_repo or f"unsloth/{file_base(it.name)}-GGUF"
        rel_model = os.path.relpath(it.dest, self.models_dir)
        guessed = "" if self.a.hf_repo else " hf_repo inferred from the filename."
        self._want_manifest(it, rel_model, it.size, repo, it.name,
                            f"ROCKY brain candidate {it.id} ({i.get('basename') or file_base(it.name)} "
                            f"{i['quant']}), installed {self.date} by rocky sim/brain_install.py; UNMEASURED."
                            + guessed)
        mm = it.mmproj
        if mm and mm["how"] != "missing":
            rel_mm = os.path.relpath(mm["dest"], self.models_dir)
            src_entry = self._manifest_entry(os.path.relpath(mm["src"], self.models_dir)) \
                if mm["how"] == "sibling" else None
            src_entry = src_entry or {}
            note = (f"mmproj for {i.get('basename') or file_base(it.name)} (renamed from the repo's generic "
                    f"{mm['hf_file']})")
            if mm["how"] == "sibling":
                note += f"; hard link of {os.path.relpath(mm['src'], self.models_dir)}"
            self._want_manifest(it, rel_mm, mm.get("size"), src_entry.get("hf_repo", repo),
                                src_entry.get("hf_file", mm["hf_file"]), note)
        # config
        if present:
            it.config = "present"
            it.notes.append(f"'{it.id}' is already in the config — config step skipped")
        elif vision and (not mm or mm["how"] == "missing"):
            it.config = "blocked"
            self._plan_fail(it, f"no projector for {i['key']}: nothing moved, no stanza (a 'vision' model "
                                "that cannot see would fool the cockpit) — get the projector (see the note), "
                                "then re-run")
        else:
            it.config = "add"
            it.stanza = self.stanza(it)
        return it

    def _config_ids(self):
        return {str(k) for k in (self.parsed.get("models") or {})}

    def _plan_fail(self, it, why):
        """Fail an item at plan time: nothing of it runs, its planned manifest
        entries go, and a Downloads projector it claimed is free again."""
        it.fail(why)
        it.manifest = []
        mm = it.mmproj
        if mm and mm.get("how") == "downloads":
            for p in self.pool:
                if p.source == mm["src"] and p.status == "claimed":
                    p.status, p.id = "pool", None
        return it

    def _duplicate(self, name, size):
        for p in model_files(self.models_dir):
            if os.path.basename(p) == name and os.path.getsize(p) == size:
                return p
        return None

    def _pick_id(self, it):
        if self.a.id:
            if not _ID_RE.match(self.a.id):
                it.fail(f"--id {self.a.id!r}: letters, digits, '.', '-' and '_' only (it names a folder "
                        "under the models dir and a key in the config)")
                return None
            other = self.taken.get(self.a.id)
            if other and other != it.name:
                it.fail(f"--id {self.a.id} is already configured for {other}; choose another --id")
                return None
            return self.a.id
        base = it.info["key"]
        for cand in (base, f"{base}-{quant_suffix(it.info['quant'])}",
                     f"{base}-{quant_suffix(it.info['quant'], long=True)}"):
            if cand.endswith("-"):
                continue
            other = self.taken.get(cand)
            if other is None or other == it.name:
                if cand != base:
                    it.notes.append(f"'{base}' is configured for {self.taken[base]}: "
                                    f"this quant becomes '{cand}'")
                return cand
        it.fail(f"'{base}' and its quant-suffixed ids are all taken by other files; pass --id")
        return None

    # ---------------- projector
    def _find_mmproj(self, it):
        # the model's own folder: <models-dir>/<id> for an install, wherever it sits for an in-place finish
        key, folder = it.info["key"], os.path.dirname(it.dest)
        base = file_base(it.name)
        # 1. already in the model's folder
        for p in sorted(glob.glob(os.path.join(glob.escape(folder), "mmproj-*.gguf"))):
            if p not in self.stable_installed:
                it.notes.append(f"{p} is still growing — not used")
                continue
            try:
                mi = header_info(p, check=True)
            except (GGUFError, OSError) as e:
                it.notes.append(f"{p}: {'incomplete' if isinstance(e, GGUFIncomplete) else 'unreadable'} "
                                f"({e}) — not used")
                continue
            if not mi["is_mmproj"] or mi["key"] != key:
                it.notes.append(f"{p} is a projector for '{mi['key']}', not '{key}' — not used")
                continue
            m = _GENERIC_MMPROJ_RE.match(os.path.basename(p))
            prec = m.group(1).upper() if m else None
            dest = os.path.join(folder, f"mmproj-{base}-{prec}.gguf") if m else p
            return {"how": "in-folder" if not m else "rename", "src": p, "dest": dest,
                    "name": os.path.basename(dest),
                    "size": os.path.getsize(p), "hf_file": f"mmproj-{prec or self._prec(p)}.gguf",
                    "info": mi, "projector": mi["projector"]}
        # 2. a generic projector in Downloads (or named on the command line)
        for mm in self.pool:
            if mm.status == "pool" and mm.info["key"] == key:
                m = _GENERIC_MMPROJ_RE.match(mm.name)
                prec = m.group(1).upper() if m else self._prec(mm.name)
                mm.status, mm.id = "claimed", it.id
                dest = os.path.join(folder, f"mmproj-{base}-{prec}.gguf")
                return {"how": "downloads", "src": mm.source, "dest": dest, "name": os.path.basename(dest),
                        "size": mm.size, "hf_file": f"mmproj-{prec}.gguf", "info": mm.info,
                        "projector": mm.info["projector"]}
        # 3. the same projector already installed for another id: hard-link it in
        for p in sorted(self.stable_installed):
            if os.path.dirname(p) == folder:
                continue
            try:
                mi = header_info(p, check=True)
            except (GGUFError, OSError):
                continue
            if mi["is_mmproj"] and mi["key"] == key:
                prec = self._prec(p)
                dest = os.path.join(folder, f"mmproj-{base}-{prec}.gguf")
                return {"how": "sibling", "src": p, "dest": dest, "name": os.path.basename(dest),
                        "size": os.path.getsize(p), "hf_file": f"mmproj-{prec}.gguf", "info": mi,
                        "projector": mi["projector"]}
        # 4. fetch it
        repo = self.a.hf_repo or f"unsloth/{base}-GGUF"
        cmd = [HF_BIN, "download", repo, "mmproj-F16.gguf", "--local-dir", folder]
        dest = os.path.join(folder, f"mmproj-{base}-F16.gguf")
        if self.a.no_fetch:
            it.notes.append("no projector found (the model's folder, Downloads, other folders) and --no-fetch: "
                            f"get it with  {' '.join(cmd)}  then re-run")
            return {"how": "missing", "src": None, "dest": dest, "name": os.path.basename(dest), "size": None,
                    "hf_file": "mmproj-F16.gguf", "cmd": cmd}
        return {"how": "fetch", "src": os.path.join(folder, "mmproj-F16.gguf"), "dest": dest,
                "name": os.path.basename(dest), "size": None, "hf_file": "mmproj-F16.gguf", "cmd": cmd}

    @staticmethod
    def _prec(path):
        m = re.search(r"(BF16|F16|F32|Q8_0)(?:\.gguf)?$", os.path.basename(path), re.I)
        return m.group(1).upper() if m else "F16"

    # ---------------- manifest
    def _manifest_entry(self, filename):
        for e in self.manifest.get("models", []):
            if e.get("filename") == filename:
                return e
        return None

    def _want_manifest(self, it, filename, size, repo, hf_file, notes):
        planned = {e["filename"] for x in self.items for e in x.manifest}
        if self._manifest_entry(filename) or filename in planned:
            return
        it.manifest.append({"filename": filename, "size_bytes": size, "hf_repo": repo, "hf_file": hf_file,
                            "notes": notes})

    # ---------------- stanza
    def stanza(self, it):
        i, mm, fam = it.info, it.mmproj, it.info["family"]

        def path(abs_path):
            rel = os.path.relpath(abs_path, self.models_dir)
            return f"${{models_dir}}/{rel}" if self.use_macro else abs_path

        vision = bool(mm and mm["how"] != "missing")
        size_m = it.size or 0
        size_p = (mm or {}).get("size") or 0
        folder, base = os.path.dirname(it.dest), file_base(it.name).lower()
        mtp = sorted(os.path.basename(p) for p in glob.glob(os.path.join(glob.escape(folder), "mtp-*.gguf"))
                     if base in os.path.basename(p).lower())
        title = f"{file_base(it.name)} ({i['quant'] or '?'})"
        c = [f"  # ---- added {self.date} by rocky sim/brain_install.py — {FAMILY_TITLE[fam]} "
             f"{i.get('size_label') or ''}, UNMEASURED until a bench says otherwise.",
             "  # One folder per model id under the models dir (rocky convention 2026-09-24). Runs ALONE:",
             "  # never add it to the resident group (group logic does not check VRAM -> CUDA OOM at spawn)."]
        if fam == "qwen35":
            c.append("  # Thinking OFF at the template level (robot turns are short tool calls); Qwen's")
            c.append("  # recommended non-thinking sampling.")
        if fam == "gemma4":
            c.append("  # Gemma decodes an image NON-CAUSALLY, so the whole image must land in one ubatch:")
            c.append("  # --image-max-tokens above 512 WITHOUT raising -b/-ub aborts the server (GGML_ASSERT")
            c.append("  # in llama-context). 140..280 fits the default n_ubatch 512.")
        if mtp:
            c.append(f"  # MTP draft on disk ({', '.join(mtp)}) — not enabled here; try it via the lab slot.")
        if size_m + size_p > VRAM_WARN_B:
            c.append(f"  # WARNING: {gb(size_m + size_p)} of weights at -ngl 99 will not fit the 16 GB")
            c.append("  # card: expect a CUDA OOM at spawn; drop -ngl 99 (auto-fit) or pick a smaller quant.")
            it.notes.append(f"WARNING: {gb(size_m + size_p)} of weights at -ngl 99 will not fit in 16 GB")
        if vision:
            name = f"{title} — vision (mmproj), rocky brain candidate"
            mm_size = f"{gb(size_p)} mmproj" if size_p else "mmproj (size once fetched)"
            desc = (f"UNMEASURED — {gb(size_m)} model + {mm_size}, fully on GPU (-ngl 99), 16k ctx. "
                    "Vision (mmproj) + chat; tool calls untested. Runs alone.")
        else:
            name = f"{title} — rocky brain candidate"
            desc = (f"UNMEASURED — {gb(size_m)} model, fully on GPU (-ngl 99), 16k ctx. Chat; tool calls "
                    "untested. Runs alone.")
        lines = c + [f"  {yq(it.id)}:", f"    name: {yq(name)}", f"    description: {yq(desc)}",
                     "    ttl: 1800", "    cmd: |", "      ${binary}", f"      -m {path(it.dest)}"]
        if vision:
            lines.append(f"      --mmproj {path(mm['dest'])}")
        lines += ["      -c 16384", "      -ngl 99", "      ${common_flags}"]
        if fam == "qwen35":
            lines.append("      --chat-template-kwargs '{\"enable_thinking\":false}'")
            lines.append("      --temp 0.7 --top-p 0.8 --top-k 20 --presence-penalty 1.5")
        if fam == "gemma4" and vision:
            lines.append("      --image-min-tokens 140 --image-max-tokens 280")
        return "\n".join(lines) + "\n"

    # ---------------- plan everything
    def plan(self):
        models = [it for it in self.items if it.status == "pending"]
        if (self.a.id or self.a.hf_repo) and len(models) > 1:
            for it in models:
                it.fail("--id / --hf-repo name ONE model, and this run has "
                        f"{len(models)}: {', '.join(m.name for m in models)}")
            return
        for it in models:
            self.plan_model(it)
        self._check_config_edits()
        # projectors nobody claimed (only the ones this run lists: in FILE mode a Downloads
        # projector is merely available, and silence about it is right)
        for mm in self.pool:
            if mm.status != "pool" or not mm.listed:
                continue
            if under(mm.source, self.models_dir):
                mm.status = "installed"
                mm.notes.append(f"an installed projector for '{mm.info['key']}' ({mm.info['projector']}) — "
                                "nothing to do; a model of that base installed to this folder uses it")
                continue
            dup = next((p for p in sorted(self.stable_installed)
                        if os.path.getsize(p) == mm.size and self._key_of(p) == mm.info["key"]), None)
            if dup:
                mm.status = "duplicate"
                mm.notes.append(f"duplicate of {dup} (same projector, same size) — nothing done; "
                                "the download is yours to delete")
            else:
                mm.status = "skipped"
                mm.notes.append(f"a projector for '{mm.info['key']}' that no model in this run needs — left "
                                "where it is (name its model on the command line to pair them)")
            if mm.explicit and mm.status == "skipped":
                mm.fail(mm.notes[-1])

    def _check_config_edits(self):
        """Every planned stanza, applied in order to an in-memory copy of the config
        and checked exactly like the real write. A bad edit (no RETRIEVAL STACK
        block, an id that breaks the YAML, ...) fails its item here, so --dry-run
        shows it and a real run moves nothing for it."""
        text, parsed = self.config_text, self.parsed
        for it in self.items:
            if it.status == "failed" or it.config != "add":
                continue
            try:
                new_text = insert_stanza(text, it.stanza)
                check_config(parsed, new_text, it.id)
                new_parsed = yaml.safe_load(new_text)
            except Exception as e:                     # RuntimeError from the checks, yaml.YAMLError
                it.config = "invalid"
                self._plan_fail(it, f"the config edit does not check out ({e}) — nothing moved, nothing "
                                    "written")
                continue
            text, parsed = new_text, new_parsed

    @staticmethod
    def _key_of(p):
        try:
            return header_info(p)["key"]
        except (GGUFError, OSError):
            return None

    # ---------------- execute
    def execute(self):
        for it in self.items:
            if it.status not in ("install", "in-place"):
                continue
            try:
                self._do(it)
            except Exception as e:                     # one bad file never stops the others
                it.fail(f"{type(e).__name__}: {e}")
                it.notes.append(self._after_failure(it))
        if self.config_changed and not self.a.no_restart:
            self._restart([it.id for it in self.items if it.config == "added"])

    def _do(self, it):
        unlink = []                                    # sources go only after every step below worked
        mm = it.mmproj
        if mm and mm["how"] != "missing":
            src = self._do_mmproj(it, mm)              # first: a fetch is the step most likely to fail
            if src:
                unlink.append(src)
        if it.status == "install":
            os.makedirs(os.path.dirname(it.dest), exist_ok=True)
            t0 = time.monotonic()
            print(f"  copying {tilde(it.source)} ({gb(it.size)}) -> {tilde(it.dest)} ...", flush=True)
            copy_verified(it.source, it.dest, it.size)
            it.copied = True
            it.notes.append(f"copied + verified in {time.monotonic() - t0:.0f} s")
            if not self.a.copy:
                unlink.append(it.source)
        if it.manifest:
            self._write_manifest(it)
        if it.config == "add":
            self._write_config(it)
        for p in unlink:
            os.unlink(p)
            it.notes.append(f"removed {tilde(p)}")
        it.status = "done"

    def _after_failure(self, it):
        """What state a failed install left, and the command that finishes it."""
        if it.config == "added":
            return "the model is installed and configured; only the step in the ERROR failed"
        if it.copied or (it.mode == "in-place" and it.dest and os.path.isfile(it.dest)):
            kept = "" if it.mode == "in-place" else "; the download is kept (it now counts as a duplicate)"
            return (f"the model is at {it.dest} but NOT configured{kept} — fix the ERROR, then finish with:  "
                    f"{self._finish_cmd(it.dest)}")
        return "nothing moved: the download is where it was — fix the ERROR and re-run"

    def _finish_cmd(self, path):
        cmd = ["./rocky.sh", "brain-install", path]
        for flag, val, default in (("--models-dir", self.models_dir, DEFAULT_MODELS_DIR),
                                   ("--config", self.config, DEFAULT_CONFIG),
                                   ("--manifest", self.manifest_path, DEFAULT_MANIFEST)):
            if os.path.abspath(val) != os.path.abspath(default):
                cmd += [flag, val]
        return " ".join(shlex.quote(c) for c in cmd)

    def _do_mmproj(self, it, mm):
        """Put the projector in place. Returns a source to unlink once the whole
        install has worked (a Downloads projector without --copy), else None."""
        os.makedirs(os.path.dirname(mm["dest"]), exist_ok=True)
        if mm["how"] == "in-folder":
            return None
        if os.path.exists(mm["dest"]):
            raise RuntimeError(f"{mm['dest']} already exists — refusing to overwrite it")
        unlink = None
        if mm["how"] == "rename":
            move_within(mm["src"], mm["dest"])
        elif mm["how"] == "downloads":
            copy_verified(mm["src"], mm["dest"], mm["size"])
            unlink = None if self.a.copy else mm["src"]
        elif mm["how"] == "sibling":
            try:
                os.link(mm["src"], mm["dest"])
            except OSError:
                copy_verified(mm["src"], mm["dest"], mm["size"])
        elif mm["how"] == "fetch":
            if os.path.exists(mm["src"]):
                raise RuntimeError(f"{mm['src']} is in the way of the download — move it aside")
            print(f"  fetching: {' '.join(mm['cmd'])}", flush=True)
            r = subprocess.run(mm["cmd"], capture_output=True, text=True, timeout=3600)
            if r.returncode != 0 or not os.path.isfile(mm["src"]):
                why = (r.stderr or r.stdout).strip()[-400:]
                raise RuntimeError(f"hf download failed ({r.returncode}): {why}")
            mi = header_info(mm["src"], check=True)
            if not mi["is_mmproj"] or mi["key"] != it.info["key"]:
                raise RuntimeError(f"the fetched {mm['src']} is a projector for '{mi['key']}', not "
                                   f"'{it.info['key']}' — left there, no stanza written")
            move_within(mm["src"], mm["dest"])
            mm["size"] = os.path.getsize(mm["dest"])
            rel = os.path.relpath(mm["dest"], self.models_dir)
            for e in it.manifest:
                if e["filename"] == rel:
                    e["size_bytes"] = mm["size"]
            if it.stanza:                              # the stanza quoted the size as unknown
                it.stanza = self.stanza(it)
        header_info(mm["dest"], check=True)            # whole, where it landed
        return unlink

    def _write_manifest(self, it):
        with open(self.manifest_path) as f:            # re-read: another item may have written it
            raw = f.read()
        data = json.loads(raw)
        have = {e.get("filename") for e in data.get("models", [])}
        new = [e for e in it.manifest if e["filename"] not in have]
        if not new:
            return
        bak = f"{self.manifest_path}.bak-{time.strftime('%Y%m%d')}-{it.id}"
        if not os.path.exists(bak):
            shutil.copy2(self.manifest_path, bak)
        data.setdefault("models", []).extend(new)
        atomic_write(self.manifest_path, json.dumps(data, indent=2) + ("\n" if raw.endswith("\n") else ""))
        self.manifest = data

    def _write_config(self, it):
        with open(self.config) as f:
            old_text = f.read()
        old = yaml.safe_load(old_text) or {}
        if it.id in (old.get("models") or {}):
            it.config = "present"
            it.notes.append(f"'{it.id}' appeared in the config meanwhile — config step skipped")
            return
        new_text = insert_stanza(old_text, it.stanza)
        check_config(old, new_text, it.id)             # in memory, before anything is written
        bak = unique_path(f"{self.config}.bak-{time.strftime('%Y%m%d')}-{it.id}")
        shutil.copy2(self.config, bak)
        self.backups.append(bak)
        atomic_write(self.config, new_text)
        try:
            with open(self.config) as f:
                check_config(old, f.read(), it.id)
        except Exception as e:
            raise RuntimeError(f"the written config does not check out ({e}) — restore it with: cp {bak} "
                               f"{self.config}") from e
        it.config = "added"
        it.notes.append(f"config backup: {bak}")
        self.config_changed = True
        self.parsed = yaml.safe_load(new_text)

    def _restart(self, ids):
        why = ""
        try:                                           # lazy: only a real restart needs httpx + the key
            from vision_bench import listed_models, running_models
        except Exception as e:                         # the config is written: restart anyway, unverified
            why = f"{type(e).__name__}: {e}"
            listed_models = running_models = None
        before = sorted(m for m in running_models() if m) if running_models else []
        loaded = f": {', '.join(before)}" if before else ""
        print(f"restarting llama-swap (unloads every loaded model{loaded}) ...", flush=True)
        r = subprocess.run(["systemctl", "--user", "restart", "llama-swap"], capture_output=True, text=True,
                           timeout=120)
        self.restart = {"done": r.returncode == 0, "unloaded": before, "waited_s": 0.0, "listed": []}
        if r.returncode != 0:
            self.restart["error"] = f"systemctl failed ({r.returncode}): {r.stderr.strip()[-300:]}"
            return
        if listed_models is None:
            self.restart["error"] = (f"restarted, but the /v1/models check could not run ({why}): "
                                     "look for the new ids in the llama-swap UI")
            return
        t0 = time.monotonic()
        while time.monotonic() - t0 < RESTART_WAIT_S:
            listed = listed_models()
            if set(ids) <= listed:
                self.restart.update(waited_s=round(time.monotonic() - t0, 1), listed=sorted(ids))
                return
            time.sleep(1.0)
        self.restart.update(waited_s=RESTART_WAIT_S,
                            error=f"llama-swap did not list {', '.join(ids)} within {RESTART_WAIT_S:.0f} s "
                                  "(is LOCAL_AI_KEY set? journalctl --user -u llama-swap)")

    # ---------------- report
    def report(self, dry):
        print(f"brain-install {self.date}" + ("  — DRY RUN: nothing below has been done" if dry else ""))
        print(f"  downloads {tilde(self.downloads)} · models {self.models_dir} "
              f"({gb(shutil.disk_usage(self.models_dir).free)} free)")
        print(f"  config {tilde(self.config)} · manifest {tilde(self.manifest_path)}")
        if not self.items:
            print("  nothing to do: no *.gguf here")
        mark = {"install": "+", "in-place": "+", "done": "+", "duplicate": "=", "installed": "=",
                "partial": "~", "skipped": "-", "failed": "!", "claimed": "·"}
        for it in self.items:
            if it.status == "claimed":
                continue
            head = f"{mark.get(it.status, '?')} {it.name}"
            if it.id and it.status in ("install", "in-place", "done", "failed"):
                head += f"  -> {it.id}"
            print(head + f"  [{it.status}]")
            i = it.info or {}
            if i and it.status not in ("partial",):
                who = i.get("basename") or i.get("name") or "-"
                print(f"    header    {i['arch']} · {i['type']} · {who} · "
                      f"{i.get('size_label') or '-'} · {i.get('quant') or '-'} · {gb(it.size or 0)}"
                      + (f" · projector {i['projector']}" if i.get("projector") else ""))
            if it.status in ("install", "in-place", "done") or (it.status == "failed" and it.dest):
                if it.mode == "in-place":
                    print(f"    in place  {it.dest}")
                else:
                    undone = "  (NOT done)" if it.status == "failed" and not it.copied else ""
                    print(f"    {it.mode:<9} {tilde(it.source)} -> {it.dest}{undone}")
                mm = it.mmproj
                if mm:
                    print(f"    mmproj    {self._mmproj_line(mm)}")
                for e in it.manifest:
                    print(f"    manifest  + {json.dumps(e, ensure_ascii=False)}")
                if not it.manifest:
                    print("    manifest  (entries already present)")
                if it.config in ("add", "added"):
                    bak = f"{tilde(self.config)}.bak-{time.strftime('%Y%m%d')}-{it.id}"
                    if it.config == "added":
                        head = f"written (backup {bak}); this stanza, right before the {MARKER} block:"
                    elif it.status == "failed":
                        head = "NOT written (the install stopped first); the stanza would be:"
                    else:
                        head = f"back up to {bak}, then this stanza right before the {MARKER} block:"
                    print(f"    config    {head}")
                    for line in (it.stanza or "").splitlines():
                        print(f"      | {line}")
                elif it.config == "blocked":
                    print("    config    NOT written (a vision model with no projector)")
                elif it.config == "invalid":
                    print("    config    NOT written: the edit does not check out (see ERROR)")
            for n in it.notes:
                print(f"    note      {n}")
            if it.error:
                print(f"    ERROR     {it.error}")
        adds = [it.id for it in self.items if it.config in ("add", "added")]
        if adds and dry:
            print("restart   would run: systemctl --user restart llama-swap (it unloads EVERY loaded model), "
                  f"then wait up to {RESTART_WAIT_S:.0f} s for /v1/models to list {', '.join(adds)}")
        elif adds and self.a.no_restart:
            print("restart   not done (--no-restart): the new ids appear after  "
                  "systemctl --user restart llama-swap")
        elif self.restart.get("done") and not self.restart.get("error"):
            gone = ", ".join(self.restart["unloaded"]) or "nothing"
            print(f"restart   llama-swap restarted (unloaded: {gone}); "
                  f"lists {', '.join(self.restart['listed'])} after {self.restart['waited_s']} s")
        if self.restart.get("error"):
            print(f"restart   ERROR {self.restart['error']}")
        print("result    " + ("OK" if self.ok() else "FAILED — see ERROR lines above"))

    def _mmproj_line(self, mm):
        how = mm["how"]
        if how == "missing":
            return f"NONE — no stanza will be written; get it with  {' '.join(mm['cmd'])}"
        if how == "fetch":
            return f"fetch:  {' '.join(mm['cmd'])}  then rename -> {mm['dest']}"
        line = {"in-folder": f"in the folder: {mm['dest']}",
                "rename": f"rename {os.path.basename(mm['src'])} -> {mm['dest']}",
                "downloads": f"{'copy' if self.a.copy else 'move'} {tilde(mm['src'])} -> {mm['dest']}",
                "sibling": f"hard link {mm['src']} -> {mm['dest']} (same bytes, no extra disk)"}[how]
        return line + (f"  ({mm.get('projector')}, {gb(mm['size'])})" if mm.get("size") else "")

    def ok(self):
        bad = any(it.status == "failed" or it.error for it in self.items)
        return not bad and not self.restart.get("error")

    def summary(self, dry):
        return {"ok": self.ok(), "dry_run": dry, "date": self.date, "downloads": self.downloads,
                "models_dir": self.models_dir, "config": self.config, "manifest": self.manifest_path,
                "items": [it.summary() for it in self.items if it.status != "claimed"],
                "config_changed": self.config_changed, "config_backups": self.backups,
                "restart": self.restart}


# ------------------------------------------------------------------ file + config primitives
def copy_verified(src, dest, size):
    """Copy across drives to a temp name beside `dest`, check the byte size and
    the header, then link it into place — never over an existing file."""
    if os.path.exists(dest):
        raise RuntimeError(f"{dest} already exists — refusing to overwrite it")
    tmp = dest + ".brain-install.part"
    try:
        shutil.copyfile(src, tmp)
        got = os.path.getsize(tmp)
        if got != size:
            raise RuntimeError(f"copied {got} bytes, expected {size}")
        fsync_path(tmp)                                # on disk before the caller may unlink the source
        check_complete(tmp)                            # header + tensor extent, where it landed
        move_within(tmp, dest)                         # (fsyncs the folder)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def move_within(src, dest):
    """Rename on one filesystem without ever replacing `dest`: a hard link
    (fails if dest exists) then unlink; plain rename where links are refused.
    The folder is fsynced after, so the new name survives a crash."""
    try:
        os.link(src, dest)
    except FileExistsError:
        raise RuntimeError(f"{dest} already exists — refusing to overwrite it") from None
    except OSError:
        if os.path.exists(dest):
            raise RuntimeError(f"{dest} already exists — refusing to overwrite it") from None
        os.rename(src, dest)
    else:
        os.unlink(src)
    fsync_path(os.path.dirname(os.path.abspath(dest)))


def atomic_write(path, text):
    tmp = f"{path}.brain-install.tmp"
    with open(tmp, "w") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    if os.path.exists(path):
        shutil.copymode(path, tmp)                     # the config keeps its permissions
    os.replace(tmp, path)
    fsync_path(os.path.dirname(os.path.abspath(path)))


def unique_path(p):
    if not os.path.exists(p):
        return p
    n = 2
    while os.path.exists(f"{p}.{n}"):
        n += 1
    return f"{p}.{n}"


def insert_stanza(text, stanza):
    """The stanza, then a blank line, immediately before the comment block that
    holds the RETRIEVAL STACK marker (the block's first comment line)."""
    lines = text.splitlines(keepends=True)
    idx = next((i for i, ln in enumerate(lines) if MARKER in ln and ln.lstrip().startswith("#")), None)
    if idx is None:
        raise RuntimeError(f"no '{MARKER}' comment block in the config — "
                           "refusing to guess where the stanza goes")
    start = idx
    while start > 0 and lines[start - 1].lstrip().startswith("#"):
        start -= 1
    block = stanza.rstrip("\n") + "\n\n"
    if start > 0 and lines[start - 1].strip():
        block = "\n" + block
    return "".join(lines[:start]) + block + "".join(lines[start:])


def check_config(old, new_text, mid):
    """The edited config parses, holds exactly one model more (mid, with a -m),
    and groups / selectors / macros are unchanged."""
    new = yaml.safe_load(new_text)
    if not isinstance(new, dict) or not isinstance(new.get("models"), dict):
        raise RuntimeError("the edited config has no models: mapping")
    old_ids, new_ids = set((old.get("models") or {})), set(new["models"])
    if new_ids - old_ids != {mid} or old_ids - new_ids:
        raise RuntimeError(f"model ids changed beyond '{mid}': "
                           f"+{sorted(new_ids - old_ids)} -{sorted(old_ids - new_ids)}")
    if "-m " not in str((new["models"][mid] or {}).get("cmd", "")):
        raise RuntimeError(f"'{mid}' has no -m in its cmd")
    for k in PROTECTED:
        if old.get(k) != new.get(k):
            raise RuntimeError(f"the {k}: section changed")


# ------------------------------------------------------------------ CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("files", nargs="*", metavar="FILE", help="GGUFs to install (default: every complete "
                    "*.gguf in --downloads); a path already under --models-dir finishes that install")
    ap.add_argument("--downloads", default=DEFAULT_DOWNLOADS)
    ap.add_argument("--models-dir", default=DEFAULT_MODELS_DIR)
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--manifest", default=DEFAULT_MANIFEST)
    ap.add_argument("--id", help="the llama-swap model id (one model per run)")
    ap.add_argument("--hf-repo", help="Hugging Face repo for the manifest + mmproj fetch "
                    "(default unsloth/<base>-GGUF; one model per run)")
    ap.add_argument("--copy", action="store_true", help="keep the source file")
    ap.add_argument("--no-fetch", action="store_true", help="never run `hf download` for a missing mmproj")
    ap.add_argument("--no-restart", action="store_true", help="edit the config but do not restart llama-swap")
    ap.add_argument("--dry-run", action="store_true", help="print the plan, touch nothing")
    ap.add_argument("--json", action="store_true", help="end with a one-line JSON summary")
    ap.add_argument("--settle", type=float, default=SETTLE_S,
                    help=f"seconds a file's size must hold still to count as complete (default {SETTLE_S:g})")
    args = ap.parse_args(argv)

    ins = Installer(args)
    try:
        if not os.path.isdir(ins.models_dir):
            raise OSError(f"no models dir at {ins.models_dir}")
        ins.load()
    except (OSError, ValueError, yaml.YAMLError) as e:
        print(f"brain-install: {e}", file=sys.stderr)
        if args.json:
            print(json.dumps({"ok": False, "error": str(e)}))
        return 1
    ins.discover()
    ins.plan()
    if not args.dry_run:
        ins.execute()
    ins.report(args.dry_run)
    if args.json:
        print(json.dumps(ins.summary(args.dry_run), ensure_ascii=False))
    return 0 if ins.ok() else 1


if __name__ == "__main__":
    sys.exit(main())
