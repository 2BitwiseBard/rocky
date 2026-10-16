"""model_admin — the cockpit's Models panel: what llama-swap serves, and the chores around it (2026-10-15).

Before this, the cockpit could only pick a model by name. Everything else was a terminal job:
loading / unloading one (curl), installing a download (`rocky.sh brain-install`), benching one
(`rocky.sh brain-bench`), changing its context or idle TTL (editing ~/.config/llama-swap/config.yaml
by hand), deleting one, and freeing an idle ComfyUI that held the GPU (the 2026-10-15 incident:
gpu_check.py). This module is the server side of all of those. The cockpit's routes stay thin.

WHAT IT READS
  * the llama-swap config (ROCKY_SWAP_CONFIG) — every model's files, -c, ttl, image tokens,
    groups / selectors / startup preload it belongs to;
  * llama-swap's /running (what is loaded or starting) — the catalog comes from the Brains;
  * the restore manifest (ROCKY_SWAP_MANIFEST) — each file's Hugging Face repo;
  * bench results: sim/out/brain_bench/*.json (what the panel's bench writes) and the older
    sim/out/brain_bench.json — the newest run per model is shown on its card;
  * disk: the models drive (ROCKY_MODELS_DIR, with its .trash) and the downloads folder.

WHAT IT CHANGES, AND THE RULES
  * load / unload one model, unload all — llama-swap's own API (POST /api/models/unload/<id>);
  * edit: ttl, -c (context), --image-min/max-tokens, name, description of ONE stanza. Text-level
    edits inside that stanza only, so the config's comments survive; the result must parse and
    differ from the old one in exactly the fields asked for (anything else refuses the edit). A
    dated backup first. Image tokens over 512 need -b and -ub at least that big (Gemma decodes an
    image in one ubatch; past 512 without them the server aborts — workstation CLAUDE.md);
  * delete: the stanza out (brain_install.remove_stanza, the same cut the CLI uninstall makes),
    its manifest entries out, and its files MOVED to <models dir>/.trash/<stamp>-<id>/ (same
    drive: a rename, undoable; RESTORE.txt says how). A file another stanza also uses stays.
    Refused while the model is in a group, a selector, the startup preload, or one of this
    cockpit's roles. "Empty trash" is the step that frees the disk;
  * install: sim/brain_install.py (plan with --dry-run first, then the run) — its own rules;
  * download: a GGUF from huggingface.co straight to the downloads folder (.part while it runs,
    resumable, size + sha256 checked against the repo's listing); split GGUFs are refused (the
    installer refuses them too). Needs room on the downloads drive AND on the models drive (the
    install copies it there);
  * bench: sim/brain_bench.py with its own cockpit; the live cockpit's model modes pause while it
    runs (it unloads models, and a chat would swap the model under test out);
  * restart llama-swap — every config change needs it; it unloads EVERY model (Open WebUI, pi and
    the digests share it), so the panel always says so first.

WHO MAY: AdminGate. Reading, load / unload, warm, cancel and bench are open to every page that
reaches the cockpit. The rest ("risky": download, install, edit, delete, empty trash, restart
llama-swap) is open to the desktop itself (a direct loopback request: no proxy headers, a
loopback Host) and, from anywhere else (the phone via `rocky.sh tailnet`), only with
ROCKY_ADMIN_PIN from rocky.env; 5 wrong PINs lock it for 10 minutes. No PIN set: desktop only.
"""
import copy
import hashlib
import hmac
import ipaddress
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from urllib.parse import quote

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import brain_install as bi       # noqa: E402  (stanza_span / remove_stanza / atomic_write / unique_path / tilde)
import gpu_check                 # noqa: E402

GiB = 1024 ** 3
HF = "https://huggingface.co"
REPO_RE = re.compile(r"^[A-Za-z0-9][\w.-]{0,95}/[\w.-]{1,96}$")
SPLIT_RE = re.compile(r"-(\d{5})-of-(\d{5})\.gguf$", re.I)
FILE_FLAGS = ("-m", "--model", "--mmproj", "-md", "--model-draft", "--lora")
SERVICE_FLAGS = ("--embedding", "--embeddings", "--rerank", "--reranking")
EDIT_LIMITS = {"ttl": (0, 86400), "ctx": (2048, 262144),
               "image_min_tokens": (16, 4096), "image_max_tokens": (16, 4096)}
EDIT_TEXT = ("name", "description")
ROLE_LABEL = {"model": "brain", "vision_model": "eye (vision model)", "multimodal_model": "multimodal model"}
CMD_FLAG = {"ctx": r"-c|--ctx-size", "image_min_tokens": r"--image-min-tokens",
            "image_max_tokens": r"--image-max-tokens"}
IMAGE_UBATCH_FREE = 512          # past this many image tokens, -b and -ub must cover it
DOWNLOAD_MARGIN_B = 1 * GiB
INSTALL_MARGIN_B = 2 * GiB
PIN_MAX_FAILS, PIN_WINDOW_S, PIN_LOCK_S = 5, 600.0, 600.0
COMFYUI_URL = os.environ.get("ROCKY_COMFYUI_URL", "http://127.0.0.1:8188").rstrip("/")


def _env_path(name, default):
    p = os.path.expanduser(os.path.expandvars(os.environ.get(name) or default))
    return p if os.path.isabs(p) else os.path.join(REPO, p)


def paths():
    """Read per call: tests (and a rocky.env edit + restart) point these elsewhere."""
    return dict(config=_env_path("ROCKY_SWAP_CONFIG", bi.DEFAULT_CONFIG),
                manifest=_env_path("ROCKY_SWAP_MANIFEST", bi.DEFAULT_MANIFEST),
                models_dir=_env_path("ROCKY_MODELS_DIR", bi.DEFAULT_MODELS_DIR),
                downloads=_env_path("ROCKY_DOWNLOADS_DIR", bi.DEFAULT_DOWNLOADS),
                bench_dir=_env_path("ROCKY_BENCH_DIR", os.path.join(HERE, "out", "brain_bench")),
                sources=_env_path("ROCKY_HF_SOURCES", os.path.join(HERE, "out", "hf_sources.json")),
                legacy_bench=os.path.join(HERE, "out", "brain_bench.json"))


def load_sources(pths=None):
    """{downloaded filename: the Hugging Face repo it came from} — kept on disk, so the installer
    still knows the repo (manifest + which projector goes with which model) after a cockpit restart."""
    try:
        with open((pths or paths())["sources"]) as f:
            d = json.load(f)
        return {str(k): str(v) for k, v in d.items()} if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def save_source(name, repo, pths=None):
    p = (pths or paths())["sources"]
    d = load_sources(pths)
    d[name] = repo
    os.makedirs(os.path.dirname(p), exist_ok=True)
    bi.atomic_write(p, json.dumps(d, indent=1, sort_keys=True) + "\n")


class AdminError(Exception):
    """A refusal the operator should read (the route answers 4xx with it)."""

    def __init__(self, msg, status=400):
        super().__init__(msg)
        self.status = status


# ------------------------------------------------------------------ the config, read
def _expand(s, macros):
    def sub(m):
        v = macros.get(m.group(1))
        return str(v).strip() if isinstance(v, (str, int, float)) else m.group(0)
    for _ in range(3):                              # macros may name macros
        s2 = re.sub(r"\$\{([\w-]+)\}", sub, s)
        if s2 == s:
            break
        s = s2
    return s


def cmd_tokens(cmd, macros):
    try:
        return shlex.split(_expand(str(cmd or ""), macros))
    except ValueError:
        return _expand(str(cmd or ""), macros).split()


def flag_value(tokens, flags):
    """The value after the LAST of these flags (llama-server's own rule), or None."""
    want = set(flags.split("|")) if isinstance(flags, str) else set(flags)
    val = None
    for i, t in enumerate(tokens[:-1]):
        if t in want:
            val = tokens[i + 1]
    return val


def _int(v):
    try:
        return int(str(v))
    except (TypeError, ValueError):
        return None


def model_files(tokens):
    """Every file the command loads, split GGUF shards included (llama.cpp finds shards 2..N by name)."""
    out = []
    for i, t in enumerate(tokens[:-1]):
        if t in FILE_FLAGS:
            p = tokens[i + 1]
            m = SPLIT_RE.search(p)
            if m:
                n = int(m.group(2))
                out += [p[:m.start()] + f"-{k:05d}-of-{n:05d}.gguf" for k in range(1, n + 1)]
            else:
                out.append(p)
    seen, uniq = set(), []
    for p in out:
        if p not in seen:
            seen.add(p)
            uniq.append(p)
    return uniq


def memberships(cfg):
    """{model id: [why it may not be deleted, ...]} from groups / selectors / the startup preload."""
    out = {}
    for g, spec in (cfg.get("groups") or {}).items():
        for m in (spec or {}).get("members") or []:
            out.setdefault(str(m), []).append(f"group '{g}'")
    for s, spec in (cfg.get("selectors") or {}).items():
        for m in (spec or {}).get("targets") or []:
            out.setdefault(str(m), []).append(f"selector '{s}'")
    pre = ((cfg.get("hooks") or {}).get("on_startup") or {}).get("preload") or []
    for m in pre:
        out.setdefault(str(m), []).append("the startup preload (hooks)")
    return out


def read_config(path=None):
    path = path or paths()["config"]
    with open(path) as f:
        text = f.read()
    cfg = yaml.safe_load(text) or {}
    if not isinstance(cfg.get("models"), dict):
        raise AdminError(f"{bi.tilde(path)} has no models: mapping")
    return text, cfg


# ------------------------------------------------------------------ bench results
def bench_scores(pths=None):
    """{model id: newest bench summary} from the panel's bench folder + the older single file."""
    pths = pths or paths()
    files = []
    if os.path.isdir(pths["bench_dir"]):
        files += [os.path.join(pths["bench_dir"], f) for f in os.listdir(pths["bench_dir"]) if f.endswith(".json")]
    if os.path.isfile(pths["legacy_bench"]):
        files.append(pths["legacy_bench"])
    out = {}
    for fp in files:
        try:
            with open(fp) as f:
                d = json.load(f)
        except (OSError, ValueError):
            continue
        if not isinstance(d, dict) or d.get("interrupted"):      # a cancelled run is no score
            continue
        for mid, m in (d.get("models") or {}).items():
            if not isinstance(m, dict) or not m.get("summary"):
                continue
            s = m["summary"]
            row = dict(date=str(d.get("date", "")), mode=m.get("mode") or d.get("mode"),
                       commands=s.get("cmd_ok_str"), stop_missed=s.get("stop_missed"),
                       unsafe=s.get("unsafe"), vram_mib=m.get("vram_mib"), tps=m.get("decode_tps"),
                       first_answer_s=m.get("first_answer_s"), latency_med_s=s.get("latency_med_s"),
                       describe=(m.get("describe_summary") or {}).get("ok_str"), file=bi.tilde(fp))
            if mid not in out or row["date"] > out[mid]["date"]:
                out[mid] = row
    return out


# ------------------------------------------------------------------ disk
_CAPS = {}


def file_caps(path):
    """What a model's own GGUF header says (cached per file + size + mtime): arch, a chat template,
    tool calls in it, a thinking switch, experts. None when the header cannot be read."""
    try:
        st = os.stat(path)
    except OSError:
        return None
    key = (path, st.st_size, st.st_mtime_ns)
    if key not in _CAPS:
        try:
            m = bi.read_gguf_header(path)["meta"]
            arch = str(m.get("general.architecture") or "")
            f = bi.template_facts(m, arch)
            _CAPS[key] = dict(arch=arch, chat_template=f["chat_template"], tools=f["tools"],
                              thinking=f["thinking_switch"], experts=f["experts"],
                              trained_ctx=m.get(f"{arch}.context_length"),
                              family=bi.family_of(m, ""))
        except (bi.GGUFError, OSError):
            _CAPS[key] = None
    return _CAPS[key]


def disk(path):
    p = path
    while p and not os.path.exists(p):
        p = os.path.dirname(p)
    try:
        u = shutil.disk_usage(p or "/")
        return dict(path=bi.tilde(path), free=u.free, total=u.total, dev=os.stat(p or "/").st_dev)
    except OSError:
        return dict(path=bi.tilde(path), free=None, total=None, dev=None)


def tree_bytes(path):
    seen, n = set(), 0
    for d, _, fns in os.walk(path):
        for fn in fns:
            try:
                st = os.lstat(os.path.join(d, fn))
            except OSError:
                continue
            if (st.st_dev, st.st_ino) not in seen:
                seen.add((st.st_dev, st.st_ino))
                n += st.st_size
    return n


# ------------------------------------------------------------------ the inventory
def inventory(running=None, catalog=None, roles=None, pths=None):
    """Every model in the config as a card. running: llama-swap /running rows; catalog: the Brains'
    classified /v1/models (vision + quarantine); roles: {role key: model id} this cockpit uses."""
    pths = pths or paths()
    text, cfg = read_config(pths["config"])
    macros = cfg.get("macros") or {}
    members = memberships(cfg)
    state = {str(r.get("model")): str(r.get("state")) for r in (running or [])}
    cat = {m["id"]: m for m in (catalog or [])}
    scores = bench_scores(pths)
    by_role = {}
    for k, v in (roles or {}).items():
        if v:
            by_role.setdefault(str(v), []).append(k)
    # who references which file (by real path): a file two stanzas share is never trashed
    refs = {}
    toks = {}
    for mid, spec in cfg["models"].items():
        toks[mid] = cmd_tokens((spec or {}).get("cmd"), macros)
        for p in model_files(toks[mid]):
            refs.setdefault(os.path.realpath(p), set()).add(str(mid))
    seen_inodes = set()
    cards = []
    for mid, spec in cfg["models"].items():
        mid, spec = str(mid), spec or {}
        t = toks[mid]
        files, disk_b = [], 0
        for p in model_files(t):
            try:
                st = os.stat(p)
                ok, size, links = True, st.st_size, st.st_nlink
                key = (st.st_dev, st.st_ino)
                if key not in seen_inodes:
                    seen_inodes.add(key)
                    disk_b += size
            except OSError:
                ok, size, links = False, 0, 0
            files.append(dict(path=bi.tilde(p), bytes=size, exists=ok,
                              shared=len(refs.get(os.path.realpath(p), ())) > 1 or links > 1))
        c = cat.get(mid, {})
        blob = " ".join([mid, str(spec.get("name", "")), str(spec.get("description", ""))])
        kind = ("lab" if mid == "lab" else "service" if any(f in t for f in SERVICE_FLAGS) else "chat")
        blockers = list(members.get(mid, []))
        blockers += [f"this cockpit's {ROLE_LABEL.get(r, r)}" for r in by_role.get(mid, [])]
        if kind == "lab":
            blockers.append("the Model Lab slot")
        cuttable = bi.stanza_span(text, mid) is not None
        if not cuttable:
            blockers.append("its stanza is not in the standard '  \"id\":' form — edit the config by hand")
        img = (_int(flag_value(t, CMD_FLAG["image_min_tokens"])), _int(flag_value(t, CMD_FLAG["image_max_tokens"])))
        weights = next((p for p in model_files(t) if not os.path.basename(p).lower().startswith(("mmproj", "mtp-"))),
                       None)
        cards.append(dict(
            id=mid, name=str(spec.get("name") or mid), description=str(spec.get("description") or ""),
            kind=kind, state=state.get(mid, "cold"),
            loaded=state.get(mid) == "ready" or bool(c.get("loaded")),
            vision=bool(c.get("vision")) if c else bool(re.search(r"mmproj|vision|\bVL\b", blob, re.I)),
            quarantined=bool(c.get("quarantined")) or "QUARANTIN" in blob.upper(),
            ttl=_int(spec.get("ttl")), ctx=_int(flag_value(t, CMD_FLAG["ctx"])),
            image_tokens=list(img) if any(v is not None for v in img) else None,
            batch=[_int(flag_value(t, "-b|--batch-size")), _int(flag_value(t, "-ub|--ubatch-size"))],
            files=files, disk_bytes=disk_b, missing=[f["path"] for f in files if not f["exists"]],
            aliases=len(spec.get("aliases") or []), member_of=members.get(mid, []),
            roles=by_role.get(mid, []), bench=scores.get(mid),
            caps=file_caps(weights) if kind == "chat" and weights else None,
            editable=cuttable, deletable=not blockers, delete_blockers=blockers))
    trash = os.path.join(pths["models_dir"], ".trash")
    dm, dd = disk(pths["models_dir"]), disk(pths["downloads"])
    dm["trash_bytes"] = tree_bytes(trash) if os.path.isdir(trash) else 0
    return dict(models=cards, disk=dict(models=dm, downloads=dd, same_drive=dm["dev"] == dd["dev"]),
                config=bi.tilde(pths["config"]))


# ------------------------------------------------------------------ edit one stanza
def _stanza_lines(text, mid):
    span = bi.stanza_span(text, mid)
    if span is None:
        raise AdminError(f"no stanza \"{mid}\": in the config (or not in the standard form) — edit it by hand")
    lines = text.splitlines(keepends=True)
    key = f"  {bi.yq(mid)}:"
    head = next(i for i in range(span[0], span[1]) if lines[i].rstrip("\n") == key)
    return lines, head + 1, span[1]


def _check_edit(old_cfg, new_text, mid, expect):
    new = yaml.safe_load(new_text)
    if not isinstance(new, dict) or not isinstance(new.get("models"), dict):
        raise AdminError("the edited config would not parse as a llama-swap config — nothing written")
    a, b = copy.deepcopy(old_cfg), copy.deepcopy(new)
    if b["models"].get(mid) != expect:
        raise AdminError(f"the edit would change more of {mid} than asked — nothing written (edit it by hand)")
    a["models"].pop(mid, None)
    b["models"].pop(mid, None)
    if a != b:
        raise AdminError("the edit would touch other parts of the config — nothing written")


def edit_stanza(text, cfg, mid, changes):
    """(new_text, [what changed], [warnings]) for ttl / ctx / image tokens / name / description."""
    if mid not in cfg["models"]:
        raise AdminError(f"no model '{mid}' in the config", 404)
    unknown = set(changes) - set(EDIT_LIMITS) - set(EDIT_TEXT)
    if unknown:
        raise AdminError(f"can't edit {', '.join(sorted(unknown))} here (editable: "
                         f"{', '.join(list(EDIT_LIMITS) + list(EDIT_TEXT))})")
    spec = cfg["models"][mid] or {}
    macros = cfg.get("macros") or {}
    t = cmd_tokens(spec.get("cmd"), macros)
    vals, done, warn = {}, [], []
    for k, v in changes.items():
        if k in EDIT_LIMITS:
            n = _int(v)
            lo, hi = EDIT_LIMITS[k]
            if n is None or not lo <= n <= hi:
                raise AdminError(f"{k} must be a whole number {lo}..{hi} (got {v!r})")
            vals[k] = n
        else:
            s = " ".join(str(v or "").split())
            if not s or len(s) > 400:
                raise AdminError(f"{k} must be 1..400 characters on one line")
            vals[k] = s
    imin = vals.get("image_min_tokens", _int(flag_value(t, CMD_FLAG["image_min_tokens"])))
    imax = vals.get("image_max_tokens", _int(flag_value(t, CMD_FLAG["image_max_tokens"])))
    if imin is not None and imax is not None and imin > imax:
        raise AdminError(f"image min tokens ({imin}) must not exceed max ({imax})")
    if "image_max_tokens" in vals and imax > IMAGE_UBATCH_FREE:
        b, ub = _int(flag_value(t, "-b|--batch-size")), _int(flag_value(t, "-ub|--ubatch-size"))
        if not (b and ub and b >= imax and ub >= imax):
            raise AdminError(f"image max tokens over {IMAGE_UBATCH_FREE} needs -b and -ub of at least {imax} in "
                             "its command (the image decodes in one ubatch; without them llama-server aborts)")
    lines, lo_i, hi_i = _stanza_lines(text, mid)
    expect = copy.deepcopy(spec)
    for k, v in vals.items():
        if k == "ttl" or k in EDIT_TEXT:
            yk = k
            pat = re.compile(rf"^    {yk}:[ \t]*(.*)$")
            hit = [i for i in range(lo_i, hi_i) if pat.match(lines[i].rstrip("\n"))]
            new_line = f"    {yk}: {v if k == 'ttl' else bi.yq(v)}\n"
            if hit:
                old_v = pat.match(lines[hit[0]].rstrip("\n")).group(1).strip()
                if old_v in ("|", ">", "|-", ">-", "|+", ">+"):
                    raise AdminError(f"{mid}'s {yk} is a multi-line block — edit it by hand")
                lines[hit[0]] = new_line
            elif k == "ttl":
                after = [i for i in range(lo_i, hi_i) if re.match(r"^    (name|description):", lines[i])]
                at = (after[-1] + 1) if after else lo_i
                lines.insert(at, new_line)
                hi_i += 1
            else:
                lines.insert(lo_i, new_line)
                hi_i += 1
            old = spec.get(yk)
            expect[yk] = v
            done.append(f"{yk}: {old!r} → {v!r}")
        else:
            flag = CMD_FLAG[k]
            pat = re.compile(rf"(?<!\S)({flag})([ \t]+)(\d+)(?!\S)")
            hit = [i for i in range(lo_i, hi_i) if lines[i].startswith("      ") and pat.search(lines[i])]
            if not hit:
                raise AdminError(f"{mid}'s command has no {flag.split('|')[0]} to change"
                                 + (" (llama-server then uses the model's own context)" if k == "ctx" else ""))
            if len(hit) > 1 or len(pat.findall(lines[hit[0]])) > 1:
                raise AdminError(f"{mid}'s command sets {flag.split('|')[0]} more than once — edit it by hand")
            old = int(pat.search(lines[hit[0]]).group(3))
            lines[hit[0]] = pat.sub(lambda m: f"{m.group(1)}{m.group(2)}{v}", lines[hit[0]])
            expect["cmd"] = pat.sub(lambda m: f"{m.group(1)}{m.group(2)}{v}", str(expect.get("cmd") or ""))
            done.append(f"{flag.split('|')[0]}: {old} → {v}")
            if k == "ctx" and v > old:
                warn.append(f"more context = more VRAM: {mid} at {v} is unmeasured — warm it and watch the GPU line")
    new_text = "".join(lines)
    _check_edit(cfg, new_text, mid, expect)
    return new_text, done, warn


# ------------------------------------------------------------------ delete (to the trash)
def delete_plan(text, cfg, mid, roles=None, pths=None):
    pths = pths or paths()
    if mid not in cfg["models"]:
        raise AdminError(f"no model '{mid}' in the config", 404)
    inv = {c["id"]: c for c in inventory(roles=roles, pths=pths)["models"]}
    card = inv[mid]
    if card["delete_blockers"]:
        raise AdminError(f"{mid} can't be deleted while it is in " + "; ".join(card["delete_blockers"])
                         + " — take it out of there first", 409)
    macros = cfg.get("macros") or {}
    mine = model_files(cmd_tokens(cfg["models"][mid].get("cmd"), macros))
    others = {}
    for oid, spec in cfg["models"].items():
        if str(oid) != mid:
            for p in model_files(cmd_tokens((spec or {}).get("cmd"), macros)):
                others.setdefault(os.path.realpath(p), str(oid))
    mdir = os.path.realpath(pths["models_dir"])
    move, keep = [], []
    for p in mine:
        rp = os.path.realpath(p)
        if rp in others:
            keep.append(f"{bi.tilde(p)} (also used by {others[rp]})")
        elif not os.path.exists(p):
            keep.append(f"{bi.tilde(p)} (already missing)")
        elif not rp.startswith(mdir + os.sep):
            keep.append(f"{bi.tilde(p)} (outside {bi.tilde(mdir)} — delete it by hand)")
        else:
            move.append(rp)
    folder = os.path.join(mdir, mid)
    with open(pths["manifest"]) as f:
        mani = json.load(f)
    rel = {os.path.relpath(p, mdir) for p in move}
    drop = [e.get("filename") for e in mani.get("models", [])
            if e.get("filename") in rel or str(e.get("filename", "")).startswith(mid + "/")]
    new_text = bi.remove_stanza(text, mid)
    try:
        bi.check_config_removed(cfg, new_text, mid)
    except RuntimeError as e:
        raise AdminError(str(e)) from None
    freed = sum(os.stat(p).st_size for p in move if os.stat(p).st_nlink == 1)
    return dict(id=mid, move=move, keep=keep, folder=folder if os.path.isdir(folder) else None,
                manifest=drop, new_text=new_text, frees_after_empty=freed)


def delete_apply(plan, pths=None):
    pths = pths or paths()
    mdir = os.path.realpath(pths["models_dir"])
    stamp = time.strftime("%Y%m%d-%H%M%S")
    trash = os.path.join(mdir, ".trash", f"{stamp}-{plan['id']}")
    os.makedirs(trash, exist_ok=True)
    cfg_path = pths["config"]
    bak = bi.unique_path(f"{cfg_path}.bak-{time.strftime('%Y%m%d')}-cockpit-delete-{plan['id']}")
    shutil.copy2(cfg_path, bak)
    bi.atomic_write(cfg_path, plan["new_text"])
    moved = []
    for p in plan["move"]:
        dest = os.path.join(trash, os.path.relpath(p, mdir))
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        os.rename(p, dest)                            # same drive: instant, and undoable
        moved.append((p, dest))
    folder = plan.get("folder")
    if folder and os.path.isdir(folder):
        left = [os.path.join(d, f) for d, _, fs in os.walk(folder) for f in fs]
        if all(os.path.relpath(x, folder).startswith(".cache") for x in left):
            shutil.move(folder, os.path.join(trash, "folder-" + plan["id"]))
    mbak = None
    if plan["manifest"]:
        with open(pths["manifest"]) as f:
            raw = f.read()
        mani = json.loads(raw)
        mbak = bi.unique_path(f"{pths['manifest']}.bak-{time.strftime('%Y%m%d')}-cockpit-delete-{plan['id']}")
        shutil.copy2(pths["manifest"], mbak)
        mani["models"] = [e for e in mani.get("models", []) if e.get("filename") not in set(plan["manifest"])]
        bi.atomic_write(pths["manifest"], json.dumps(mani, indent=2) + ("\n" if raw.endswith("\n") else ""))
    with open(os.path.join(trash, "RESTORE.txt"), "w") as f:
        f.write(f"Deleted from the rocky cockpit {time.strftime('%Y-%m-%d %H:%M')}: {plan['id']}\n\n"
                f"To undo: move these back, restore the config (and manifest) backup, restart llama-swap.\n")
        for src, dest in moved:
            f.write(f"  mv {shlex.quote(dest)} {shlex.quote(src)}\n")
        f.write(f"  cp {shlex.quote(bak)} {shlex.quote(cfg_path)}\n")
        if mbak:
            f.write(f"  cp {shlex.quote(mbak)} {shlex.quote(pths['manifest'])}\n")
        f.write("  systemctl --user restart llama-swap\n")
    return dict(trash=bi.tilde(trash), moved=len(moved), config_backup=bi.tilde(bak),
                manifest_backup=bi.tilde(mbak) if mbak else None)


def empty_trash(pths=None):
    pths = pths or paths()
    trash = os.path.join(os.path.realpath(pths["models_dir"]), ".trash")
    if not os.path.isdir(trash):
        return dict(freed=0, removed=0)
    n, freed = 0, tree_bytes(trash)
    for e in os.listdir(trash):
        p = os.path.join(trash, e)
        if os.path.isdir(p) and not os.path.islink(p):
            shutil.rmtree(p)
        else:
            os.unlink(p)
        n += 1
    return dict(freed=freed, removed=n)


# ------------------------------------------------------------------ Hugging Face
def _hf_headers():
    tok = os.environ.get("HF_TOKEN", "").strip()
    if not tok:
        try:
            with open(os.path.expanduser("~/.cache/huggingface/token")) as f:
                tok = f.read().strip()
        except OSError:
            tok = ""
    return {"Authorization": f"Bearer {tok}"} if tok else {}


def check_repo(repo):
    repo = str(repo or "").strip().strip("/")
    if repo.startswith(HF + "/"):
        repo = repo[len(HF) + 1:].split("/tree/")[0].split("/blob/")[0]
    if not REPO_RE.match(repo) or ".." in repo:
        raise AdminError(f"not a Hugging Face repo id: {repo!r} (want owner/name, e.g. unsloth/Qwen3.5-4B-GGUF)")
    return repo


def hf_search(q, limit=12, get=None):
    import httpx
    q = " ".join(str(q or "").split())[:80]
    if len(q) < 2:
        raise AdminError("type at least two characters to search")
    get = get or (lambda url, **kw: httpx.get(url, timeout=10.0, headers=_hf_headers(), **kw))
    r = get(f"{HF}/api/models", params={"search": q, "filter": "gguf", "sort": "downloads",
                                        "limit": int(limit)})
    r.raise_for_status()
    return [dict(id=e.get("id"), downloads=e.get("downloads"), likes=e.get("likes"),
                 vision=e.get("pipeline_tag") == "image-text-to-text", gated=bool(e.get("gated")))
            for e in r.json() if e.get("id")]


QUANT_RE = re.compile(r"(?<![A-Za-z0-9])((?:UD-)?(?:IQ\d\w*|Q\d\w*|BF16|F16|F32|MXFP4))", re.I)


def hf_files(repo, get=None):
    """The repo's GGUFs: [{path, size, sha256, mmproj, split, quant}], biggest-first per kind."""
    import httpx
    repo = check_repo(repo)
    get = get or (lambda url, **kw: httpx.get(url, timeout=15.0, headers=_hf_headers(), **kw))
    r = get(f"{HF}/api/models/{repo}/tree/main", params={"recursive": "true"})
    if r.status_code in (401, 403):
        raise AdminError(f"{repo} is gated or private: accept its terms on huggingface.co and put a token in "
                         "HF_TOKEN (or ~/.cache/huggingface/token)", 403)
    if r.status_code == 404:
        raise AdminError(f"no repo {repo} on huggingface.co", 404)
    r.raise_for_status()
    out = []
    for e in r.json():
        p = str(e.get("path", ""))
        if e.get("type") != "file" or not p.lower().endswith(".gguf"):
            continue
        name = os.path.basename(p)
        q = QUANT_RE.search(name.rsplit(".", 1)[0])
        out.append(dict(path=p, size=int(e.get("size") or 0), sha256=(e.get("lfs") or {}).get("oid"),
                        mmproj=name.lower().startswith("mmproj"), split=bool(SPLIT_RE.search(name)),
                        quant=q.group(1) if q else None))
    out.sort(key=lambda f: (f["mmproj"], f["size"]))
    return dict(repo=repo, url=f"{HF}/{repo}", files=out)


def download_room(size, pths=None):
    """None, or why there is no room: the downloads drive takes the file, the models drive takes
    the install's copy of it (one drive: both)."""
    pths = pths or paths()
    dd, dm = disk(pths["downloads"]), disk(pths["models_dir"])
    if dd["free"] is None or dm["free"] is None:
        return None
    need_d = size + DOWNLOAD_MARGIN_B + (size + INSTALL_MARGIN_B if dd["dev"] == dm["dev"] else 0)
    if dd["free"] < need_d:
        return (f"not enough room on {dd['path']}: {dd['free'] / GiB:.1f} GiB free, this needs "
                f"{need_d / GiB:.1f} GiB")
    if dd["dev"] != dm["dev"] and dm["free"] < size + INSTALL_MARGIN_B:
        return (f"not enough room on the models drive {dm['path']} for the install: {dm['free'] / GiB:.1f} GiB "
                f"free, it needs {(size + INSTALL_MARGIN_B) / GiB:.1f} GiB — delete a model and empty the trash")
    return None


def download_file(job, repo, path, size, sha256, pths=None, stream=None):
    """Into the downloads folder as <name>.part (resumed with a Range when one is there), then the size
    and the repo's sha256 are checked and it is renamed to <name>. Never overwrites a different file."""
    import httpx
    pths = pths or paths()
    os.makedirs(pths["downloads"], exist_ok=True)
    name = os.path.basename(path)
    dest = os.path.join(pths["downloads"], name)
    part = dest + ".part"
    if os.path.exists(dest):
        if os.path.getsize(dest) == size:
            job.say(f"{name} is already in {bi.tilde(pths['downloads'])} — skipped")
            return dest
        raise AdminError(f"{bi.tilde(dest)} exists with a different size — move it away first", 409)
    have = os.path.getsize(part) if os.path.exists(part) else 0
    if have > size:
        os.unlink(part)
        have = 0
    url = f"{HF}/{repo}/resolve/main/{quote(path)}"
    job.progress = dict(file=name, done=have, total=size)
    hdrs = dict(_hf_headers())
    if have:
        hdrs["Range"] = f"bytes={have}-"
        job.say(f"resuming {name} at {have / GiB:.2f} GiB")
    stream = stream or (lambda: httpx.stream("GET", url, headers=hdrs, follow_redirects=True,
                                             timeout=httpx.Timeout(30.0, read=60.0)))
    with stream() as r:
        if r.status_code in (401, 403):
            raise AdminError(f"{repo} refused the download (gated? set HF_TOKEN)", 403)
        if have and r.status_code != 206:
            have = 0                                     # the server ignored the Range: start over
            job.progress["done"] = 0
        r.raise_for_status()
        with open(part, "ab" if have else "wb") as f:
            last = time.monotonic()
            for chunk in r.iter_bytes(1 << 20):
                if job.cancelled:
                    raise AdminError("cancelled (the .part stays: downloading it again resumes)")
                f.write(chunk)
                job.progress["done"] += len(chunk)
                if time.monotonic() - last > 5:
                    last = time.monotonic()
                    job.say(f"{name}: {job.progress['done'] / GiB:.2f} / {size / GiB:.2f} GiB")
    got = os.path.getsize(part)
    if got != size:
        raise AdminError(f"{name}: got {got} bytes, the repo lists {size} — run the download again to resume")
    if sha256:
        job.say(f"{name}: checking sha256 …")
        job.progress = dict(file=name, done=0, total=size, verifying=True)
        h = hashlib.sha256()
        with open(part, "rb") as f:
            for b in iter(lambda: f.read(8 << 20), b""):
                if job.cancelled:
                    raise AdminError("cancelled while checking (the .part stays)")
                h.update(b)
                job.progress["done"] += len(b)
        if h.hexdigest() != sha256:
            os.unlink(part)
            raise AdminError(f"{name}: sha256 does not match the repo's — the file was deleted; download it again")
    if os.path.exists(dest):
        raise AdminError(f"{bi.tilde(dest)} appeared while downloading — left the .part beside it", 409)
    os.rename(part, dest)
    job.say(f"{name}: done ({size / GiB:.2f} GiB, {'sha256 ok' if sha256 else 'size ok'})")
    return dest


def downloaded_ggufs(pths=None):
    """Complete GGUFs in the downloads folder (what Install offers): no .part / .crdownload beside."""
    pths = pths or paths()
    d = pths["downloads"]
    if not os.path.isdir(d):
        return []
    names = set(os.listdir(d))
    out = []
    for n in sorted(names):
        if not n.lower().endswith(".gguf"):
            continue
        if any(n + s in names for s in (".part", ".crdownload", ".aria2")):
            continue
        p = os.path.join(d, n)
        try:
            size = os.path.getsize(p)
        except OSError:
            continue
        if size:
            out.append(dict(file=n, bytes=size, mmproj=n.lower().startswith("mmproj"), split=bool(SPLIT_RE.search(n))))
    src = load_sources(pths)
    for f in out:
        f["repo"] = src.get(f["file"])
    for f in out:                                       # after every repo is known
        f["with"] = projector_for(f, out) if not f["mmproj"] else None
    return out


def projector_for(f, files):
    """The one projector downloaded from the same repo as model file f (installed with it), or None."""
    if not f.get("repo"):
        return None
    mm = [x["file"] for x in files if x["mmproj"] and x.get("repo") == f["repo"]]
    return mm[0] if len(mm) == 1 else None


# ------------------------------------------------------------------ jobs
class Job:
    def __init__(self, jid, kind, title):
        self.id, self.kind, self.title = jid, kind, title
        self.state, self.t0, self.t1 = "running", time.time(), None
        self.log, self.progress, self.result, self.error = deque(maxlen=300), None, None, None
        self.cancelled, self.proc = False, None

    def say(self, line):
        self.log.append(str(line).rstrip()[:400])

    def view(self, tail=40):
        return dict(id=self.id, kind=self.kind, title=self.title, state=self.state,
                    started=self.t0, elapsed_s=round((self.t1 or time.time()) - self.t0, 1),
                    log=list(self.log)[-tail:], progress=self.progress, result=self.result, error=self.error)


class Jobs:
    """Background work for the panel, one per lane: 'swap' (install / bench / a llama-swap restart:
    each would break the other) and 'download'. Plain threads; a subprocess job is killed by
    process group on cancel."""
    LANES = {"install": "swap", "bench": "swap", "restart": "swap", "download": "download"}

    def __init__(self, on_change=None, keep=20):
        self._jobs, self._n, self._lock, self.on_change, self.keep = [], 0, threading.Lock(), on_change, keep

    def busy(self, kind):
        lane = self.LANES.get(kind, kind)
        return next((j for j in self._jobs if j.state == "running" and self.LANES.get(j.kind, j.kind) == lane), None)

    def _new(self, kind, title):
        with self._lock:
            b = self.busy(kind)
            if b:
                raise AdminError(f"'{b.title}' is still running — wait for it or cancel it", 409)
            self._n += 1
            j = Job(f"j{self._n}", kind, title)
            self._jobs.append(j)
            self._jobs = [x for x in self._jobs if x.state == "running"] + \
                [x for x in self._jobs if x.state != "running"][-self.keep:]
        return j

    def _finish(self, j, ok, error=None):
        j.t1 = time.time()
        j.state = "cancelled" if j.cancelled else ("done" if ok else "failed")
        j.error = None if ok else (error or "failed")
        if self.on_change:
            try:
                self.on_change(j)
            except Exception:                              # noqa: BLE001
                pass

    def run_fn(self, kind, title, fn):
        j = self._new(kind, title)

        def go():
            try:
                j.result = fn(j)
                self._finish(j, True)
            except AdminError as e:
                j.say(str(e))
                self._finish(j, False, str(e))
            except Exception as e:                         # noqa: BLE001
                j.say(f"{type(e).__name__}: {e}")
                self._finish(j, False, f"{type(e).__name__}: {e}")
        threading.Thread(target=go, daemon=True, name=f"job-{j.id}").start()
        return j

    def run_proc(self, kind, title, argv, env=None, cwd=None, parse=None):
        """argv in its own process group; stdout+stderr into the log; parse(job, lines) -> result."""
        def fn(j):
            j.say("$ " + " ".join(shlex.quote(a) for a in argv))
            if j.cancelled:
                raise AdminError("cancelled before it started")
            j.proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                      env=env, cwd=cwd, start_new_session=True, bufsize=1)
            if j.cancelled:                                # a cancel that came while it was starting
                self._kill(j)
            lines = []
            for line in j.proc.stdout:
                lines.append(line.rstrip("\n"))
                j.say(line)
            rc = j.proc.wait()
            res = parse(j, lines) if parse else None
            if rc != 0 and not j.cancelled:
                raise AdminError(f"exited with code {rc}" + (f": {lines[-1][:200]}" if lines else ""))
            return res
        return self.run_fn(kind, title, fn)

    def cancel(self, jid):
        j = next((x for x in self._jobs if x.id == jid), None)
        if j is None:
            raise AdminError(f"no job {jid}", 404)
        if j.state != "running":
            return j.view()
        j.cancelled = True
        self._kill(j)
        return j.view()

    KILL_GRACE_S = 20.0

    @classmethod
    def _kill(cls, j):
        """SIGTERM to the job's own process first (brain_bench unwinds on it: its cockpit, the roles it
        changed); the whole process group gets SIGKILL only if it is still there KILL_GRACE_S later."""
        p = j.proc
        if not p or p.poll() is not None:
            return
        try:
            os.kill(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            return

        def hard():
            try:
                p.wait(timeout=cls.KILL_GRACE_S)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
        threading.Thread(target=hard, daemon=True).start()

    def view(self):
        """Running jobs first, then the newest finished."""
        js = sorted(self._jobs, key=lambda j: (j.state != "running", -j.t0))
        return [j.view() for j in js]


# ------------------------------------------------------------------ who may
def _hdr(headers, k):
    return (headers or {}).get(k) or (headers or {}).get(k.title())


class AdminGate:
    def __init__(self, pin=None, clock=time.monotonic):
        p = str(os.environ.get("ROCKY_ADMIN_PIN", "") if pin is None else pin).strip()
        self.pin = p if len(p) >= 4 else ""              # a 1-3 character PIN is no lock: treated as unset
        self.clock, self.fails, self.locked_until = clock, deque(), 0.0

    @staticmethod
    def direct_local(client_host, headers):
        """The desktop itself: a loopback peer, a loopback Host, and no proxy in between (tailscale serve
        connects from 127.0.0.1 too, but keeps the *.ts.net Host and adds X-Forwarded-For)."""
        try:
            if not ipaddress.ip_address(str(client_host or "").strip("[]")).is_loopback:
                return False
        except ValueError:
            return False
        if any(_hdr(headers, k) for k in ("x-forwarded-for", "forwarded", "x-forwarded-host",
                                           "tailscale-user-login", "x-real-ip")):
            return False
        host = str(_hdr(headers, "host") or "").lower()
        name = host[1:].partition("]")[0] if host.startswith("[") else host.rsplit(":", 1)[0] if host.count(":") == 1 else host
        if name == "localhost":
            return True
        try:
            return ipaddress.ip_address(name).is_loopback
        except ValueError:
            return False

    def info(self, client_host, headers):
        local = self.direct_local(client_host, headers)
        return dict(local=local, pin_set=bool(self.pin), needs_pin=not local and bool(self.pin),
                    allowed=local or bool(self.pin), locked=self.clock() < self.locked_until)

    def check(self, client_host, headers, pin):
        """None (go ahead) or raise AdminError 403."""
        if self.direct_local(client_host, headers):
            return None
        if not self.pin:
            raise AdminError("this is a desktop-only action: set ROCKY_ADMIN_PIN in rocky.env (and restart the "
                             "cockpit) to allow it from the phone", 403)
        now = self.clock()
        if now < self.locked_until:
            raise AdminError(f"too many wrong PINs — locked for {self.locked_until - now:.0f} s more", 403)
        if hmac.compare_digest(str(pin or "").strip().encode(), self.pin.encode()):
            return None
        self.fails.append(now)
        while self.fails and now - self.fails[0] > PIN_WINDOW_S:
            self.fails.popleft()
        if len(self.fails) >= PIN_MAX_FAILS:
            self.locked_until, self.fails = now + PIN_LOCK_S, deque()
            raise AdminError(f"wrong PIN {PIN_MAX_FAILS} times — locked for {PIN_LOCK_S / 60:.0f} minutes", 403)
        raise AdminError("wrong PIN" if pin else "this needs the admin PIN (ROCKY_ADMIN_PIN)", 403)


# ------------------------------------------------------------------ ComfyUI
def free_comfyui(get=None, post=None, url=COMFYUI_URL):
    """ComfyUI's /free (unload models + free memory), only when its queue is empty."""
    import httpx
    get = get or (lambda u: httpx.get(u, timeout=3.0))
    post = post or (lambda u, j: httpx.post(u, json=j, timeout=10.0))
    before = gpu_check.status(refresh=True)
    try:
        q = get(f"{url}/queue").json()
    except Exception as e:                                 # noqa: BLE001
        raise AdminError(f"ComfyUI does not answer on {url} ({type(e).__name__}) — close it by hand", 502) from None
    busy = len(q.get("queue_running") or []) + len(q.get("queue_pending") or [])
    if busy:
        raise AdminError(f"ComfyUI is working ({busy} job{'s' if busy > 1 else ''} queued) — not freeing it "
                         "under a running job", 409)
    r = post(f"{url}/free", {"unload_models": True, "free_memory": True})
    if getattr(r, "status_code", 200) >= 400:
        raise AdminError(f"ComfyUI /free answered HTTP {r.status_code}", 502)
    time.sleep(1.5)
    after = gpu_check.status(refresh=True)
    freed = (before["used_mib"] - after["used_mib"]) if before and after else None
    return dict(ok=True, freed_mib=freed, gpu=after)


# ------------------------------------------------------------------ the panel
class ModelAdmin:
    """What the cockpit's /api/admin/* routes call. brains: the cockpit's Brains (catalog, roles,
    the llama-swap base URL + key, pause during a bench)."""

    def __init__(self, brains, gate=None, jobs=None, py=None, log=None):
        self.brains, self.gate = brains, gate or AdminGate()
        self.jobs = jobs or Jobs(on_change=self._job_done)
        self.py = py or sys.executable
        self.log = log or (lambda m: None)

    # ---- llama-swap
    def _root(self):
        b = self.brains.base
        return b[:-3] if b.endswith("/v1") else b

    def _http(self, method, path, timeout=10.0, **kw):
        import httpx
        return httpx.request(method, self._root() + path, timeout=timeout,
                             headers={"Authorization": f"Bearer {self.brains.key}"}, **kw)

    def running(self):
        try:
            r = self._http("GET", "/running", timeout=4.0)
            r.raise_for_status()
            return r.json().get("running") or [], True
        except Exception:                                   # noqa: BLE001
            return [], False

    def payload(self, client_host=None, headers=None):
        run, up = self.running()
        cat, _ok = self.brains.catalog(refresh=True)
        roles = {k: self.brains.state.get(k) for k in ("model", "vision_model", "multimodal_model")}
        inv = inventory(run, cat, roles)
        try:
            ver = self._http("GET", "/api/version", timeout=3.0).json().get("version") if up else None
        except Exception:                                   # noqa: BLE001
            ver = None
        inv.update(swap=dict(up=up, version=ver), jobs=self.jobs.view(),
                   downloads=downloaded_ggufs(), gate=self.gate.info(client_host, headers),
                   gpu=gpu_check.status(), paused=getattr(self.brains, "paused", None),
                   service=bool(os.environ.get("INVOCATION_ID")))
        return inv

    def unload(self, mid=None):
        if mid:
            r = self._http("POST", f"/api/models/unload/{mid}", timeout=30.0)
        else:
            r = self._http("POST", "/api/models/unload", timeout=60.0)
        if r.status_code >= 400:
            raise AdminError(f"llama-swap: HTTP {r.status_code} {r.text[:120]}", 502)
        self.brains.catalog(refresh=True)
        gpu_check.status(refresh=True)
        return dict(ok=True, unloaded=mid or "all")

    def restart_swap(self):
        """systemctl --user restart llama-swap, then wait for /v1/models (60 s). Never under an install or
        a bench (the 'swap' lane): the restart would break them."""
        b = self.jobs.busy("restart")
        if b:
            raise AdminError(f"'{b.title}' is running — a llama-swap restart now would break it", 409)
        r = subprocess.run(["systemctl", "--user", "restart", "llama-swap"], capture_output=True, text=True,
                           timeout=120)
        if r.returncode != 0:
            raise AdminError(f"systemctl restart llama-swap failed: {r.stderr.strip()[-200:]}", 502)
        t0 = time.monotonic()
        while time.monotonic() - t0 < bi.RESTART_WAIT_S:
            try:
                if self._http("GET", "/v1/models", timeout=3.0).status_code == 200:
                    self.brains.catalog(refresh=True)
                    return f"llama-swap restarted ({time.monotonic() - t0:.1f} s)"
            except Exception:                               # noqa: BLE001
                pass
            time.sleep(1.0)
        raise AdminError(f"llama-swap did not answer within {bi.RESTART_WAIT_S:.0f} s of the restart — "
                         "journalctl --user -u llama-swap", 502)

    # ---- edit / delete
    def edit(self, mid, changes, apply=False, restart=False):
        text, cfg = read_config()
        new_text, done, warn = edit_stanza(text, cfg, mid, changes or {})
        out = dict(id=mid, changes=done, warnings=warn, applied=False)
        if not apply:
            return out
        p = paths()["config"]
        bak = bi.unique_path(f"{p}.bak-{time.strftime('%Y%m%d')}-cockpit-edit-{mid}")
        shutil.copy2(p, bak)
        bi.atomic_write(p, new_text)
        with open(p) as f:
            _check_edit(cfg, f.read(), mid, yaml.safe_load(new_text)["models"][mid])
        out.update(applied=True, config_backup=bi.tilde(bak))
        self.log(f"models: edited {mid}: {'; '.join(done)} (backup {bi.tilde(bak)})")
        if restart:
            out["restart"] = self.restart_swap()
        else:
            out["restart"] = "not restarted — llama-swap uses the change after its next restart"
        return out

    def _roles(self):
        return {k: self.brains.state.get(k) for k in ("model", "vision_model", "multimodal_model")}

    def delete(self, mid, apply=False, restart=True):
        text, cfg = read_config()
        plan = delete_plan(text, cfg, mid, roles=self._roles())
        view = dict(id=mid, move=[bi.tilde(p) for p in plan["move"]], keep=plan["keep"],
                    manifest=plan["manifest"], frees_after_empty=plan["frees_after_empty"], applied=False)
        if not apply:
            return view
        b = self.jobs.busy("restart")
        if b:
            raise AdminError(f"'{b.title}' is running — wait for it first", 409)
        try:
            self.unload(mid)
        except Exception:                                   # noqa: BLE001 — not loaded / swap down: still delete
            pass
        view.update(delete_apply(plan), applied=True)
        self.log(f"models: deleted {mid} → {view['trash']}")
        view["restart"] = self.restart_swap() if restart else "not restarted"
        return view

    def empty_trash(self):
        r = empty_trash()
        self.log(f"models: emptied the trash ({r['freed'] / GiB:.1f} GiB)")
        return r

    # ---- Hugging Face
    def hf_search(self, q):
        return hf_search(q)

    def hf_files(self, repo):
        out = hf_files(repo)
        d = paths()["downloads"]
        for f in out["files"]:
            f["have"] = os.path.exists(os.path.join(d, os.path.basename(f["path"])))
            f["partial"] = os.path.exists(os.path.join(d, os.path.basename(f["path"]) + ".part"))
        return out

    def download(self, repo, files):
        listing = {f["path"]: f for f in hf_files(repo)["files"]}
        repo = check_repo(repo)
        want = [listing.get(str(p)) for p in (files or [])]
        if not want or None in want:
            raise AdminError("pick one or more GGUF files from the repo's list")
        if any(f["split"] for f in want):
            raise AdminError("split GGUFs (-00001-of-0000N) are not installable from here — install the shards "
                             "by hand (the installer refuses them too)")
        room = download_room(sum(f["size"] for f in want))
        if room:
            raise AdminError(room, 507)

        def fn(job):
            done = []
            for f in want:
                p = download_file(job, repo, f["path"], f["size"], f["sha256"])
                save_source(os.path.basename(p), repo)
                done.append(os.path.basename(p))
            return dict(files=done, repo=repo)
        names = ", ".join(os.path.basename(f["path"]) for f in want)
        return self.jobs.run_fn("download", f"download {names}", fn).view()

    # ---- install
    def _install_argv(self, file, dry):
        d = paths()["downloads"]
        name = os.path.basename(str(file or ""))
        if not name.lower().endswith(".gguf") or name != str(file):
            raise AdminError("pick a GGUF from the downloads list")
        p = os.path.join(d, name)
        if not os.path.isfile(p):
            raise AdminError(f"{name} is not in {bi.tilde(d)}", 404)
        files = downloaded_ggufs()
        f = next((x for x in files if x["file"] == name), None)
        if f and f["mmproj"]:
            raise AdminError("that is a vision projector: install its model, and the projector goes with it")
        argv = [self.py, os.path.join(HERE, "brain_install.py"), p]
        if f and f.get("with"):                         # its projector, from the same repo: install them together
            argv.append(os.path.join(d, f["with"]))
        argv.append("--json")
        if f and f.get("repo"):
            argv += ["--hf-repo", f["repo"]]
        return argv + (["--dry-run"] if dry else [])

    def install_plan(self, file):
        argv = self._install_argv(file, dry=True)
        r = subprocess.run(argv, capture_output=True, text=True, timeout=180, cwd=REPO)
        lines = (r.stdout + ("\n" + r.stderr if r.stderr.strip() else "")).strip().splitlines()
        summary = _last_json(lines)
        return dict(ok=r.returncode == 0, plan=[ln for ln in lines if not ln.startswith("{")][-60:],
                    summary=summary)

    def install(self, file):
        argv = self._install_argv(file, dry=False)
        return self.jobs.run_proc("install", f"install {os.path.basename(file)}", argv, cwd=REPO,
                                  parse=lambda j, lines: _last_json(lines)).view()

    # ---- bench
    def bench(self, mid, mode=None):
        cat, _ = self.brains.catalog(refresh=True)
        c = next((m for m in (cat or []) if m["id"] == mid), None)
        if c is None or c.get("selector"):
            raise AdminError(f"'{mid}' is not a chat model llama-swap serves", 404)
        if c.get("quarantined"):
            raise AdminError(f"{mid} is quarantined — never benched (it faults the GPU)", 409)
        blocked = gpu_check.blocked()
        if blocked:
            raise AdminError(blocked, 409)
        mode = mode or ("multimodal" if c.get("vision") else "local")
        out = os.path.join(paths()["bench_dir"], f"{mid}-{time.strftime('%Y%m%d-%H%M')}.json")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        env = dict(os.environ, MUJOCO_GL="egl")
        argv = [self.py, os.path.join(HERE, "brain_bench.py"), "--models", mid, "--mode", mode, "--out", out]
        self.brains.paused = f"the brain bench is running on {mid} (Models panel) — talk mode still works"
        self.log(f"models: bench {mid} ({mode}) started; local/multimodal chat paused")
        return self.jobs.run_proc("bench", f"bench {mid} ({mode})", argv, env=env, cwd=REPO).view()

    def _job_done(self, j):
        if j.kind == "bench":
            self.brains.paused = None
            self.log(f"models: bench finished ({j.state}); chat resumed")
        elif j.kind in ("install", "restart"):
            try:
                self.brains.catalog(refresh=True)
            except Exception:                               # noqa: BLE001
                pass
        if j.kind != "download" or j.state != "done":
            self.log(f"models: {j.title} — {j.state}" + (f": {j.error}" if j.error else ""))

    def free_comfyui(self):
        return free_comfyui()


def _last_json(lines):
    for ln in reversed(lines or []):
        s = ln.strip()
        if s.startswith("{") and s.endswith("}"):
            try:
                return json.loads(s)
            except ValueError:
                continue
    return None
