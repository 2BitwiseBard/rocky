"""sim/model_admin.py: the Models panel's server side, on a synthetic llama-swap config in tmp_path.

Nothing here talks to llama-swap, Hugging Face, ComfyUI or systemd: the config, manifest, model
files, downloads folder and bench results are files under tmp_path, and the network parts get
fake get / stream callables. No model loads, nothing restarts.
"""
import contextlib
import hashlib
import json
import os
import sys
import time
import types

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim"))

import model_admin as ma                                                 # noqa: E402

CONFIG = """\
# a llama-swap config in the reference setup's shape (comments must survive every edit)
hooks:
  on_startup:
    preload:
      - "embedding"

macros:
  binary: /bin/llama-server
  models_dir: {models}
  common_flags: |
    --host 127.0.0.1 --port ${{PORT}}
    -np 1

selectors:
  "tool-model":
    strategy: pin
    targets: ["big"]

models:
  # the everyday driver
  "big":
    name: "Big MoE"
    description: "the driver"
    cmd: |
      ${{binary}}
      -m ${{models_dir}}/Big-IQ4_XS.gguf
      -c 131072
      ${{common_flags}}

  # ---- added by brain_install
  "small-vl":
    name: "Small VL — vision (mmproj)"
    description: "UNMEASURED"
    ttl: 1800
    cmd: |
      ${{binary}}
      -m ${{models_dir}}/small-vl/Small-Q4_K_M.gguf
      --mmproj ${{models_dir}}/small-vl/mmproj-Small-F16.gguf
      -c 16384
      ${{common_flags}}
      --image-min-tokens 140 --image-max-tokens 280

  "small-q8":
    name: "Small VL Q8 — vision (mmproj)"
    description: "an A/B"
    ttl: 1800
    cmd: |
      ${{binary}}
      -m ${{models_dir}}/small-q8/Small-Q8_0.gguf
      --mmproj ${{models_dir}}/small-vl/mmproj-Small-F16.gguf
      -c 16384
      ${{common_flags}}

  "embedding":
    name: "Embedding (CPU)"
    ttl: 0
    cmd: |
      ${{binary}}
      -m ${{models_dir}}/Embed.gguf
      --embedding
      -c 8192

groups:
  "resident":
    swap: false
    members:
      - "big"
"""


@pytest.fixture
def env(tmp_path, monkeypatch):
    models, dl, bench = tmp_path / "models", tmp_path / "Downloads", tmp_path / "bench"
    for d in (models / "small-vl", models / "small-q8", dl, bench):
        d.mkdir(parents=True, exist_ok=True)
    files = {"Big-IQ4_XS.gguf": 4000, "small-vl/Small-Q4_K_M.gguf": 3000, "small-vl/mmproj-Small-F16.gguf": 500,
             "small-q8/Small-Q8_0.gguf": 5000, "Embed.gguf": 300}
    for rel, n in files.items():
        (models / rel).write_bytes(b"G" * n)
    cfg = tmp_path / "config.yaml"
    cfg.write_text(CONFIG.format(models=models))
    mani = tmp_path / "models.manifest.json"
    mani.write_text(json.dumps({"models": [{"filename": rel, "hf_repo": "x/y"} for rel in files]}, indent=2) + "\n")
    monkeypatch.setenv("ROCKY_SWAP_CONFIG", str(cfg))
    monkeypatch.setenv("ROCKY_SWAP_MANIFEST", str(mani))
    monkeypatch.setenv("ROCKY_MODELS_DIR", str(models))
    monkeypatch.setenv("ROCKY_DOWNLOADS_DIR", str(dl))
    monkeypatch.setenv("ROCKY_BENCH_DIR", str(bench))
    monkeypatch.setenv("ROCKY_HF_SOURCES", str(tmp_path / "hf_sources.json"))
    pths = ma.paths()
    pths["legacy_bench"] = str(tmp_path / "no-legacy.json")
    monkeypatch.setattr(ma, "paths", lambda: dict(pths))
    return types.SimpleNamespace(tmp=tmp_path, models=models, dl=dl, bench=bench, cfg=cfg, mani=mani, pths=pths)


def cards(env, **kw):
    return {c["id"]: c for c in ma.inventory(pths=env.pths, **kw)["models"]}


# ------------------------------------------------------------------ inventory
def test_inventory_reads_files_flags_and_what_holds_a_model(env):
    c = cards(env, running=[{"model": "small-vl", "state": "ready"}], roles={"multimodal_model": "small-vl"})
    assert c["big"]["ctx"] == 131072 and c["big"]["ttl"] is None and c["big"]["disk_bytes"] == 4000
    assert c["small-vl"]["image_tokens"] == [140, 280] and c["small-vl"]["loaded"] and c["small-vl"]["vision"]
    assert c["embedding"]["kind"] == "service"
    # the shared mmproj is counted once across the fleet, and marked shared on both cards
    assert c["small-vl"]["disk_bytes"] == 3500 and c["small-q8"]["disk_bytes"] == 5000
    assert all(f["shared"] for f in c["small-q8"]["files"] if "mmproj" in f["path"])
    # what may not be deleted, and why
    assert not c["big"]["deletable"] and "group 'resident'" in c["big"]["delete_blockers"]
    assert "selector 'tool-model'" in c["big"]["delete_blockers"]
    assert not c["embedding"]["deletable"] and any("preload" in b for b in c["embedding"]["delete_blockers"])
    assert not c["small-vl"]["deletable"] and "this cockpit's multimodal model" in c["small-vl"]["delete_blockers"]
    assert c["small-q8"]["deletable"]


def test_a_missing_file_is_named(env):
    os.unlink(env.models / "Big-IQ4_XS.gguf")
    assert cards(env)["big"]["missing"] == [str(env.models / "Big-IQ4_XS.gguf")]


def test_the_newest_bench_per_model_is_on_its_card(env):
    def run(date, ok):
        return {"date": date, "mode": "multimodal", "models": {"small-vl": {
            "mode": "multimodal", "vram_mib": 5841, "decode_tps": 72.3, "first_answer_s": 10.1,
            "summary": {"cmd_ok_str": ok, "stop_missed": 0, "unsafe": 1, "latency_med_s": 3.6}}}}
    (env.bench / "a.json").write_text(json.dumps(run("2026-09-25 12:00", "16/20")))
    (env.bench / "b.json").write_text(json.dumps(run("2026-10-15 09:00", "19/20")))
    (env.bench / "junk.json").write_text("{not json")
    cut = run("2026-10-16 09:00", "3/20")
    cut["interrupted"] = "SIGTERM during small-vl"
    (env.bench / "c.json").write_text(json.dumps(cut))                 # a cancelled run is not a score
    b = cards(env)["small-vl"]["bench"]
    assert b["commands"] == "19/20" and b["vram_mib"] == 5841 and b["date"].startswith("2026-10-15")


def test_split_shards_are_all_files_of_the_model():
    t = ["-m", "/m/Flash-UD-IQ1_S-00001-of-00003.gguf", "--mmproj", "/m/mm.gguf"]
    assert ma.model_files(t) == [f"/m/Flash-UD-IQ1_S-0000{k}-of-00003.gguf" for k in (1, 2, 3)] + ["/m/mm.gguf"]


# ------------------------------------------------------------------ edit
def _edit(env, mid, changes):
    text, cfg = ma.read_config(str(env.cfg))
    return text, ma.edit_stanza(text, cfg, mid, changes)


def test_an_edit_changes_exactly_the_fields_asked_and_keeps_the_comments(env):
    text, (new, done, warn) = _edit(env, "small-vl", {"ttl": 600, "ctx": 32768, "description": 'say "hi"'})
    old_cfg, new_cfg = yaml.safe_load(text), yaml.safe_load(new)
    s = new_cfg["models"]["small-vl"]
    assert s["ttl"] == 600 and "-c 32768" in s["cmd"] and s["description"] == 'say "hi"'
    assert "# the everyday driver" in new and "# ---- added by brain_install" in new
    for k in ("big", "small-q8", "embedding"):
        assert new_cfg["models"][k] == old_cfg["models"][k]
    assert any("unmeasured" in w for w in warn) and len(done) == 3


def test_a_ttl_is_added_where_there_was_none(env):
    _, (new, done, _w) = _edit(env, "big", {"ttl": 900})
    assert yaml.safe_load(new)["models"]["big"]["ttl"] == 900 and "None → 900" in done[0]


def test_edits_that_would_break_a_model_are_refused(env):
    with pytest.raises(ma.AdminError, match="-b and -ub"):
        _edit(env, "small-vl", {"image_max_tokens": 600})          # the server would abort on an image
    with pytest.raises(ma.AdminError, match="must not exceed"):
        _edit(env, "small-vl", {"image_min_tokens": 300})
    with pytest.raises(ma.AdminError, match="no --image-max-tokens"):
        _edit(env, "small-q8", {"image_max_tokens": 280})
    with pytest.raises(ma.AdminError, match="whole number"):
        _edit(env, "big", {"ctx": 10})
    with pytest.raises(ma.AdminError, match="can't edit"):
        _edit(env, "big", {"cmd": "rm -rf /"})
    with pytest.raises(ma.AdminError, match="no model"):
        _edit(env, "nope", {"ttl": 1})


def test_edit_apply_writes_a_backup(env, monkeypatch):
    adm = ma.ModelAdmin(types.SimpleNamespace(state={}, base="http://x/v1", key="k"))
    before = env.cfg.read_text()
    r = adm.edit("small-vl", {"ttl": 60}, apply=True, restart=False)
    assert r["applied"] and "next restart" in r["restart"]
    assert yaml.safe_load(env.cfg.read_text())["models"]["small-vl"]["ttl"] == 60
    bak = [p for p in os.listdir(env.tmp) if p.startswith("config.yaml.bak-")]
    assert len(bak) == 1 and (env.tmp / bak[0]).read_text() == before


# ------------------------------------------------------------------ delete
def test_delete_moves_only_its_own_files_to_the_trash(env):
    text, cfg = ma.read_config(str(env.cfg))
    plan = ma.delete_plan(text, cfg, "small-q8", pths=env.pths)
    assert plan["move"] == [os.path.realpath(env.models / "small-q8/Small-Q8_0.gguf")]
    assert any("also used by small-vl" in k for k in plan["keep"])          # the shared mmproj stays
    assert plan["manifest"] == ["small-q8/Small-Q8_0.gguf"]
    r = ma.delete_apply(plan, pths=env.pths)
    new = yaml.safe_load(env.cfg.read_text())
    assert "small-q8" not in new["models"] and "small-vl" in new["models"]
    assert not (env.models / "small-q8/Small-Q8_0.gguf").exists()
    assert (env.models / "small-vl/mmproj-Small-F16.gguf").exists()
    trash = env.models / ".trash"
    (entry,) = os.listdir(trash)
    assert (trash / entry / "small-q8/Small-Q8_0.gguf").exists()
    restore = (trash / entry / "RESTORE.txt").read_text()
    assert "mv " in restore and "systemctl --user restart llama-swap" in restore
    mani = json.loads(env.mani.read_text())
    assert "small-q8/Small-Q8_0.gguf" not in [e["filename"] for e in mani["models"]]
    assert r["moved"] == 1
    # the trash shows on the inventory and empties for good
    assert ma.inventory(pths=env.pths)["disk"]["models"]["trash_bytes"] >= 5000
    e = ma.empty_trash(pths=env.pths)
    assert e["freed"] >= 5000 and not os.listdir(trash)


def test_delete_is_refused_for_a_model_something_depends_on(env):
    text, cfg = ma.read_config(str(env.cfg))
    with pytest.raises(ma.AdminError, match="group 'resident'") as e:
        ma.delete_plan(text, cfg, "big", pths=env.pths)
    assert e.value.status == 409
    with pytest.raises(ma.AdminError, match="this cockpit's brain"):
        ma.delete_plan(text, cfg, "small-q8", roles={"model": "small-q8"}, pths=env.pths)


# ------------------------------------------------------------------ the gate
TAILNET = {"host": "ai-hub.tail54f481.ts.net:9445", "x-forwarded-for": "100.64.1.2",
           "tailscale-user-login": "someone@example.com"}


def test_the_desktop_is_open_and_a_proxied_request_needs_the_pin():
    g = ma.AdminGate(pin="2468")
    assert g.check("127.0.0.1", {"host": "127.0.0.1:8765"}, None) is None
    assert g.check("::1", {"host": "localhost:8765"}, None) is None
    # tailscale serve connects from loopback too: its headers and Host give it away
    with pytest.raises(ma.AdminError, match="needs the admin PIN"):
        g.check("127.0.0.1", TAILNET, None)
    with pytest.raises(ma.AdminError):
        g.check("127.0.0.1", {"host": "127.0.0.1:8765", "x-forwarded-for": "100.64.1.2"}, None)
    with pytest.raises(ma.AdminError):
        g.check("100.64.1.2", {"host": "127.0.0.1:8765"}, None)
    assert g.check("127.0.0.1", TAILNET, "2468") is None
    assert g.info("127.0.0.1", TAILNET) == dict(local=False, pin_set=True, needs_pin=True, allowed=True, locked=False)


def test_no_pin_means_desktop_only_and_a_short_pin_counts_as_none():
    for pin in ("", "12"):
        g = ma.AdminGate(pin=pin)
        with pytest.raises(ma.AdminError, match="desktop-only"):
            g.check("127.0.0.1", TAILNET, pin)


def test_wrong_pins_lock_the_gate():
    now = [1000.0]
    g = ma.AdminGate(pin="2468", clock=lambda: now[0])
    for _ in range(ma.PIN_MAX_FAILS - 1):
        with pytest.raises(ma.AdminError, match="wrong PIN"):
            g.check("127.0.0.1", TAILNET, "0000")
    with pytest.raises(ma.AdminError, match="locked"):
        g.check("127.0.0.1", TAILNET, "0000")
    with pytest.raises(ma.AdminError, match="locked"):
        g.check("127.0.0.1", TAILNET, "2468")                         # even the right one, while locked
    now[0] += ma.PIN_LOCK_S + 1
    assert g.check("127.0.0.1", TAILNET, "2468") is None


# ------------------------------------------------------------------ Hugging Face
class _Resp:
    def __init__(self, data=None, status=200, chunks=()):
        self._data, self.status_code, self._chunks = data, status, chunks

    def json(self):
        return self._data

    def raise_for_status(self):
        if self.status_code >= 400:
            raise RuntimeError(self.status_code)

    def iter_bytes(self, _n):
        yield from self._chunks


def test_repo_ids_are_checked():
    assert ma.check_repo("https://huggingface.co/unsloth/Qwen3.5-4B-GGUF/tree/main") == "unsloth/Qwen3.5-4B-GGUF"
    for bad in ("", "noslash", "a/b/c", "../etc", "a/..", "a b/c"):
        with pytest.raises(ma.AdminError):
            ma.check_repo(bad)


def test_hf_files_lists_ggufs_with_quant_and_flags():
    tree = [{"type": "file", "path": "README.md", "size": 10},
            {"type": "file", "path": "Q-4B-UD-Q4_K_XL.gguf", "size": 2_900_000_000, "lfs": {"oid": "ab" * 32}},
            {"type": "file", "path": "mmproj-F16.gguf", "size": 670_000_000, "lfs": {"oid": "cd" * 32}},
            {"type": "file", "path": "big/Q-4B-BF16-00001-of-00002.gguf", "size": 5_000_000_000}]
    out = ma.hf_files("unsloth/Q-4B-GGUF", get=lambda url, **kw: _Resp(tree))
    by = {f["path"]: f for f in out["files"]}
    assert set(by) == {"Q-4B-UD-Q4_K_XL.gguf", "mmproj-F16.gguf", "big/Q-4B-BF16-00001-of-00002.gguf"}
    assert by["Q-4B-UD-Q4_K_XL.gguf"]["quant"] == "UD-Q4_K_XL" and by["mmproj-F16.gguf"]["mmproj"]
    assert by["big/Q-4B-BF16-00001-of-00002.gguf"]["split"]
    with pytest.raises(ma.AdminError, match="gated"):
        ma.hf_files("a/b", get=lambda url, **kw: _Resp(status=401))


def _job():
    return ma.Job("j1", "download", "t")


def test_a_download_is_checked_and_renamed_into_place(env):
    data = b"GGUF" * 1000
    sha = hashlib.sha256(data).hexdigest()
    stream = lambda: contextlib.nullcontext(_Resp(status=200, chunks=[data[:1500], data[1500:]]))   # noqa: E731
    p = ma.download_file(_job(), "a/b", "sub/M-Q4_K_M.gguf", len(data), sha, pths=env.pths, stream=stream)
    assert p == str(env.dl / "M-Q4_K_M.gguf") and (env.dl / "M-Q4_K_M.gguf").read_bytes() == data
    assert not (env.dl / "M-Q4_K_M.gguf.part").exists()
    assert [f["file"] for f in ma.downloaded_ggufs(pths=env.pths)] == ["M-Q4_K_M.gguf"]


def test_a_bad_hash_deletes_the_part_and_a_short_one_keeps_it(env):
    data = b"x" * 2000
    stream = lambda: contextlib.nullcontext(_Resp(chunks=[data]))   # noqa: E731
    with pytest.raises(ma.AdminError, match="sha256"):
        ma.download_file(_job(), "a/b", "M.gguf", len(data), "0" * 64, pths=env.pths, stream=stream)
    assert not (env.dl / "M.gguf.part").exists() and not (env.dl / "M.gguf").exists()
    with pytest.raises(ma.AdminError, match="resume"):
        ma.download_file(_job(), "a/b", "M.gguf", len(data) + 10, None, pths=env.pths, stream=stream)
    assert (env.dl / "M.gguf.part").exists()
    assert ma.downloaded_ggufs(pths=env.pths) == []            # a .part beside it: not offered for install


def test_a_cancelled_download_keeps_its_part(env):
    j = _job()

    def chunks():
        yield b"a" * 100
        j.cancelled = True
        yield b"b" * 100
    stream = lambda: contextlib.nullcontext(types.SimpleNamespace(status_code=200, raise_for_status=lambda: None,   # noqa: E731
                                                                  iter_bytes=lambda n: chunks()))
    with pytest.raises(ma.AdminError, match="cancelled"):
        ma.download_file(j, "a/b", "M.gguf", 10_000, None, pths=env.pths, stream=stream)
    assert (env.dl / "M.gguf.part").stat().st_size == 100


def test_room_is_checked_on_both_drives(env, monkeypatch):
    big = {"free": 10 * ma.GiB, "total": 100 * ma.GiB}

    def disk(path):
        return dict(path=path, dev=1 if "Downloads" in str(path) else 2,
                    **(big if "Downloads" in str(path) else {"free": 3 * ma.GiB, "total": 100 * ma.GiB}))
    monkeypatch.setattr(ma, "disk", disk)
    assert ma.download_room(int(0.5 * ma.GiB), pths=env.pths) is None
    assert "models drive" in ma.download_room(2 * ma.GiB, pths=env.pths)
    assert "not enough room on" in ma.download_room(20 * ma.GiB, pths=env.pths)


# ------------------------------------------------------------------ jobs
def test_a_cancel_while_it_starts_still_kills_it():
    jobs = ma.Jobs()
    j = jobs.run_proc("bench", "sleepy", [sys.executable, "-c", "import time; time.sleep(30)"])
    jobs.cancel(j.id)                                       # at once: before or just after the Popen
    t0 = time.monotonic()
    while j.state == "running" and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    assert j.state == "cancelled" and (j.proc is None or j.proc.poll() is not None)


def test_jobs_run_one_per_lane_and_cancel_kills_the_process():
    jobs = ma.Jobs()
    j = jobs.run_proc("bench", "sleepy", [sys.executable, "-c", "import time; print('hi', flush=True); time.sleep(30)"])
    with pytest.raises(ma.AdminError, match="still running"):
        jobs.run_proc("install", "other", [sys.executable, "-c", "pass"])      # same lane: refused
    t0 = time.monotonic()
    while "hi" not in jobs.view()[0]["log"] and time.monotonic() - t0 < 10:      # the line itself, not the $ echo
        time.sleep(0.05)
    jobs.cancel(j.id)
    t0 = time.monotonic()
    while jobs.view()[0]["state"] == "running" and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    assert jobs.view()[0]["state"] == "cancelled"
    d = jobs.run_fn("download", "dl", lambda job: {"ok": 1})                  # its own lane
    t0 = time.monotonic()
    while d.state == "running" and time.monotonic() - t0 < 5:
        time.sleep(0.02)
    assert d.state == "done" and d.result == {"ok": 1}


# ------------------------------------------------------------------ ComfyUI
def test_comfyui_is_freed_only_when_idle(monkeypatch):
    monkeypatch.setattr(ma.gpu_check, "status", lambda refresh=False: {"used_mib": 1000})
    monkeypatch.setattr(ma.time, "sleep", lambda s: None)
    posted = []
    busy = lambda u: _Resp({"queue_running": [[1]], "queue_pending": []})   # noqa: E731
    with pytest.raises(ma.AdminError, match="working"):
        ma.free_comfyui(get=busy, post=lambda u, j: posted.append(u))
    assert not posted
    idle = lambda u: _Resp({"queue_running": [], "queue_pending": []})      # noqa: E731
    r = ma.free_comfyui(get=idle, post=lambda u, j: posted.append((u, j)) or _Resp({}))
    assert r["ok"] and posted == [("http://127.0.0.1:8188/free", {"unload_models": True, "free_memory": True})] \
        or posted[0][0].endswith("/free")


def test_a_model_and_its_projector_from_one_repo_install_together(env, monkeypatch):
    for n in ("Mistral-Q4_K_M.gguf", "mmproj-mistral-f16.gguf", "Other-Q8_0.gguf", "mmproj-other-f16.gguf"):
        (env.dl / n).write_bytes(b"G" * 10)
    ma.save_source("Mistral-Q4_K_M.gguf", "bartowski/Mistral-GGUF", env.pths)
    ma.save_source("mmproj-mistral-f16.gguf", "bartowski/Mistral-GGUF", env.pths)
    files = {f["file"]: f for f in ma.downloaded_ggufs(env.pths)}
    assert files["Mistral-Q4_K_M.gguf"]["with"] == "mmproj-mistral-f16.gguf"
    assert files["Other-Q8_0.gguf"]["with"] is None and files["Other-Q8_0.gguf"]["repo"] is None
    admin = ma.ModelAdmin(types.SimpleNamespace(), py="py")
    monkeypatch.setattr(ma, "paths", lambda: dict(env.pths))
    argv = admin._install_argv("Mistral-Q4_K_M.gguf", dry=True)
    assert argv[2:] == [str(env.dl / "Mistral-Q4_K_M.gguf"), str(env.dl / "mmproj-mistral-f16.gguf"), "--json",
                        "--hf-repo", "bartowski/Mistral-GGUF", "--dry-run"]
    assert admin._install_argv("Other-Q8_0.gguf", dry=False)[2:] == [str(env.dl / "Other-Q8_0.gguf"), "--json"]
    with pytest.raises(ma.AdminError, match="projector"):
        admin._install_argv("mmproj-other-f16.gguf", dry=True)


def test_a_card_says_what_its_file_can_do(env, monkeypatch):
    import brain_install as bi
    meta = {"general.architecture": "llama", "tokenizer.chat_template": "{{ messages }}", "llama.context_length": 8192}
    monkeypatch.setattr(bi, "read_gguf_header", lambda p: {"meta": meta})
    ma._CAPS.clear()
    c = cards(env)
    assert c["big"]["caps"] == dict(arch="llama", chat_template=True, tools=False, thinking=False, experts=0,
                                    trained_ctx=8192, family="generic")
    assert c["embedding"]["caps"] is None                             # a service: not read
    ma._CAPS.clear()
