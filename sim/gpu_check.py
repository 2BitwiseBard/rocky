"""gpu_check — who holds the GPU, for the cockpit's model calls (2026-10-15).

Why: on 2026-10-15 every model the cockpit tried failed to start ("upstream
command exited prematurely" from llama-swap): an idle ComfyUI held 13.3 of
16 GB. The fallback chains then walked five models (gemma-4-12b, qwen3.5-9b,
qwen3.5-4b, qwen3.6-35b-a3b, qwen3.8-27b-iq4), each a failed spawn, before
the regex brain answered with "HTTP 500" notes that named no cause. Rapid
failed spawns are also the bench churn the workstation's GPU notes warn about.

status() reads nvidia-smi (best effort: None without it, on another vendor's
GPU, or on any error) and splits the memory into what the model server can
reclaim (its own llama-server processes: llama-swap swaps them out) and what
it cannot (every other process). blocked() says whether a model load cannot
fit even after the server unloads its own. Pure stdlib; cached 2 s.
"""
import os
import shutil
import subprocess
import time

MIN_FREE_MIB = int(os.environ.get("ROCKY_GPU_MIN_FREE_MIB", "4096"))   # the smallest useful model (lfm2.5-vl 4.6 G)
CACHE_S = 2.0
MODEL_SERVER_NAMES = ("llama-server", "llama-swap", "whisper-server")
# how to free a known holder (shown in the reply; a hint, never run)
HINTS = {"comfyui": "ComfyUI: its Unload Models button, or curl -X POST http://127.0.0.1:8188/free "
                    "-d '{\"unload_models\":true,\"free_memory\":true}'",
         "unrealeditor": "close the Unreal editor",
         "blender": "close Blender"}

_cache = (0.0, None)


def _proc_label(pid):
    """A short human name for a pid: the script for python (ComfyUI/main.py), else the exe name."""
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            argv = [a.decode(errors="replace") for a in f.read().split(b"\0") if a]
    except OSError:
        return f"pid {pid}"
    if not argv:
        return f"pid {pid}"
    exe = os.path.basename(argv[0])
    if exe.startswith("python") and len(argv) > 1:
        script = next((a for a in argv[1:] if not a.startswith("-")), argv[1])
        parts = script.rstrip("/").split("/")
        return "/".join(parts[-2:]) if len(parts) > 1 else parts[0]
    return exe


def _smi(args):
    out = subprocess.run(["nvidia-smi", *args, "--format=csv,noheader,nounits"],
                         capture_output=True, text=True, timeout=3.0, check=True).stdout
    return [[c.strip() for c in line.split(",")] for line in out.strip().splitlines() if line.strip()]


def status(refresh=False):
    """{total_mib, used_mib, free_mib, reclaimable_mib, holders: [{pid, name, mib, model_server}]}
    for GPU 0, or None (no nvidia-smi / any error)."""
    global _cache
    t, st = _cache
    if not refresh and time.monotonic() - t < CACHE_S:
        return st
    st = None
    if shutil.which("nvidia-smi"):
        try:
            total, used = (int(float(x)) for x in _smi(["--query-gpu=memory.total,memory.used"])[0][:2])
            holders = []
            for row in _smi(["--query-compute-apps=pid,used_memory"]):
                try:
                    pid, mib = int(row[0]), int(float(row[1]))
                except (ValueError, IndexError):
                    continue
                name = _proc_label(pid)
                holders.append(dict(pid=pid, name=name, mib=mib,
                                    model_server=any(n in name for n in MODEL_SERVER_NAMES)))
            holders.sort(key=lambda h: -h["mib"])
            st = dict(total_mib=total, used_mib=used, free_mib=max(0, total - used), holders=holders,
                      reclaimable_mib=sum(h["mib"] for h in holders if h["model_server"]))
        except (subprocess.SubprocessError, OSError, ValueError, IndexError):
            st = None
    _cache = (time.monotonic(), st)
    return st


def hint(name):
    low = str(name).lower()
    return next((h for k, h in HINTS.items() if k in low), None)


def blocked(st=None, min_free_mib=MIN_FREE_MIB):
    """None when a model can load (or we cannot tell); else a one-line reason naming the
    biggest holders that are not the model server, and how to free a known one."""
    st = status() if st is None else st
    if not st:
        return None
    room = st["free_mib"] + st["reclaimable_mib"]
    if room >= min_free_mib:
        return None
    others = [h for h in st["holders"] if not h["model_server"]][:2]
    who = ", ".join(f"{h['name']} (pid {h['pid']}) holds {h['mib'] / 1024:.1f} GB" for h in others) \
        or f"{st['used_mib'] / 1024:.1f} GB in use"
    tip = next((hint(h["name"]) for h in others if hint(h["name"])), None)
    return (f"the GPU is full — {who}; {room / 1024:.1f} of {st['total_mib'] / 1024:.0f} GB free for models"
            + (f". Free it ({tip}) and send again" if tip else ". Free it and send again"))
