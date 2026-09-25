"""D052 cockpit brains — roles, fallbacks, transcripts, voice gate, gesture
composing. No model server, no MuJoCo: a FakeSim carries the tool surface
and a scripted `complete` stands in for llama-swap. These are PLUMBING
tests of the brain layer (what reaches the robot and what the operator is
told); the physics behind each tool is tested elsewhere."""
import asyncio
import json
import os
import sys
import threading
import time
from collections import deque
from types import SimpleNamespace

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
for sub in ("sim", "gait"):
    sys.path.insert(0, os.path.join(ROOT, sub))
sys.path.insert(0, ROOT)

import cockpit_brains as cb                                            # noqa: E402

# the live llama-swap list on this box (2026-09-24), trimmed to the fields that matter
LLAMA_SWAP = [
    {"id": "embedding", "name": "Qwen3-Embedding-0.6B", "description": "639 MB."},
    {"id": "gemma-4-26b-a4b", "name": "Gemma 4 26B-A4B MoE (UD-Q5_K_XL) — vision",
     "description": "21 GB, partial CPU offload. 64k ctx. Multimodal."},
    {"id": "qwen3.5-9b", "name": "Qwen3.5-9B MoE (UD-Q5_K_XL) — vision",
     "description": "21 GB, partial CPU offload. 64k ctx. Multimodal."},
    {"id": "gemma-4-31b", "name": "Gemma 4 31B (UD-Q4_K_XL) — dense Gemma",
     "description": "19 GB, partial CPU offload. 64k ctx. Text-only."},
    {"id": "glm-flash-reap", "name": "GLM-4.7-Flash REAP-23B-A3B — ⚠ QUARANTINED (faults ~29%/spawn)",
     "description": "CUDA-faults."},
    {"id": "gpt-oss-20b", "name": "gpt-oss-20b (MXFP4) — fast resident agent model",
     "description": "12 GB, fully on GPU."},
    {"id": "lfm2.5-vl", "name": "LFM2.5-VL-3B (Q8_0) — tiny fast vision",
     "description": "3 GB, fully on GPU.", "status": {"value": "loaded"}},
    {"id": "qwen3.6-35b-a3b", "name": "Qwen 3.6 35B-A3B MoE — everyday driver",
     "description": "17 GB / 3B active.", "status": {"value": "loaded"}},
    {"id": "qwen3.8-27b", "name": "Qwen 3.8 27B — dense VLM, max local quality",
     "description": "16 GB, partial CPU offload. VISION (mmproj). 96k ctx."},
    {"id": "qwen3.8-27b-iq4", "name": "Qwen 3.8 27B (UD-IQ4_XS v3) — speed variant",
     "description": "13 GB, all on GPU. 64k ctx, q4_0 KV. Text-only."},
    {"id": "tool-model", "name": "Tool-calling model", "description": "Indirection for MCP/agent traffic.",
     "meta": {"llamaswap": {"type": "selector", "strategy": "pin", "targets": ["qwen3.6-35b-a3b"]}}},
    {"id": "vision-model", "name": "Vision model", "description": "Prefers whichever VLM is warm.",
     "meta": {"llamaswap": {"type": "selector", "strategy": "warm",
                            "targets": ["lfm2.5-vl", "qwen3.8-27b"]}}},
]
CAT = cb.classify_models(LLAMA_SWAP)


class FakeSim:
    def __init__(self):
        self.gesture_names = ["wave", "turn_in_place", "sidestep"]
        self.lexicon = ["greeting", "confused", "curious_question", "acknowledge"]
        self.gestures = {n: (lambda g, t: None, 1.0) for n in self.gesture_names}
        self.frames = {"eye": (1, b"\xff\xd8fake")}
        self.events, self.logs, self.cmds, self.started = deque(), [], [], []
        self.gesture_busy = False
        self.mode, self.speed = "idle", 1.0
        self.x = 0.0
        self.stops = 0
        self.model = None
        self.gait = None

    def log(self, m):
        self.logs.append(m)

    def note_cmd(self, line):
        self.cmds.append(line)

    async def call(self, fn):
        return fn()

    def start_gesture(self, fn, total, name=None):
        self.started.append((name, fn, total))
        return None

    def pose(self):
        return {"x": self.x, "y": 0.0, "yaw_deg": 0.0}

    async def tool_say(self, word):
        return {"ok": True, "word": word}

    async def tool_gesture(self, name):
        self.started.append((name, None, 1.0))
        return {"ok": True, "gesture": name}

    async def tool_goto(self, x, y):
        tx, _ty, err = cb.validate_goto(x, y)
        if err:
            return {"ok": False, "error": err}
        self.x = tx
        return {"ok": True, "stopped": "arrived", "pose": self.pose()}

    async def tool_stop(self):
        self.stops += 1
        return {"ok": True, "mode": "safe_stop"}

    async def tool_status(self):
        return {"ok": True, "pose": self.pose(), "mode": "idle"}

    async def tool_scan_summary(self):
        raise RuntimeError("lidar unplugged")


class Script:
    """A scripted model: per model id a list of replies (dicts, or exceptions to raise)."""

    def __init__(self, **per_model):
        self.per = {k.replace("_", "-").replace("qwen3-", "qwen3."): list(v) for k, v in per_model.items()}
        self.calls = []

    def __call__(self, model, messages, tools, max_tokens):
        self.calls.append((model, json.loads(json.dumps(messages)), tools))
        q = self.per.get(model)
        if not q:
            raise RuntimeError(f"HTTP 404: no such model {model}")
        r = q.pop(0) if len(q) > 1 else q[0]
        if isinstance(r, Exception):
            raise r
        return r


def call(name, args, cid=None):
    return {"id": cid, "name": name, "arguments": args if isinstance(args, str) else json.dumps(args)}


def answer(text):
    return {"content": text, "tool_calls": []}


def brains(script=None, **kw):
    return cb.Brains(FakeSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None,
                     complete=script, catalog=CAT, **kw)


def run(coro):
    return asyncio.run(coro)


def transcript_ok(messages):
    """Every assistant tool_call has exactly one tool result after it."""
    pending = set()
    for m in messages:
        if m["role"] == "assistant":
            assert not pending, f"tool calls without results: {pending}"
            pending = {tc["id"] for tc in m.get("tool_calls") or []}
        elif m["role"] == "tool":
            pending.discard(m["tool_call_id"])
        elif m["role"] == "user":
            assert not pending, f"tool calls without results: {pending}"
    return not pending


# ------------------------------------------------------------------ catalog
def test_vision_detection_uses_llama_swap_metadata():
    by = {m["id"]: m for m in CAT}
    assert "embedding" not in by
    assert by["gemma-4-26b-a4b"]["vision"] and by["qwen3.8-27b"]["vision"] and by["lfm2.5-vl"]["vision"]
    assert not by["gemma-4-31b"]["vision"]           # the old 'gemma' substring rule said yes: 500
    assert not by["qwen3.8-27b-iq4"]["vision"] and not by["qwen3.6-35b-a3b"]["vision"]
    assert by["vision-model"]["vision"] and by["vision-model"]["selector"]
    assert by["vision-model"]["loaded"]              # a target (lfm2.5-vl) is loaded
    assert not by["tool-model"]["vision"] and by["tool-model"]["loaded"]
    assert by["glm-flash-reap"]["quarantined"] and by["gpt-oss-20b"]["quarantined"]


def test_default_chains_are_the_co_resident_pair_then_fallbacks():
    b = brains()
    cmap = {m["id"]: m for m in CAT}
    assert b.chain("brain", cat=cmap)[0] == ["tool-model", "qwen3.8-27b-iq4"]    # 3.6 = tool-model's pin
    assert b.chain("vision", cat=cmap)[0] == ["vision-model", "gemma-4-26b-a4b"]
    chain, notes = b.chain("vision", first="gemma-4-31b", cat=cmap)
    assert "gemma-4-31b" not in chain and any("text-only" in n for n in notes)
    chain, notes = b.chain("brain", first="gpt-oss-20b", cat=cmap)
    assert "gpt-oss-20b" not in chain and any("quarantined" in n for n in notes)


def test_set_roles_refuses_quarantined_and_text_only_and_persists(tmp_path):
    conf = tmp_path / "cockpit.json"
    b = cb.Brains(FakeSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=str(conf),
                  catalog=CAT)
    r = b.set_roles({"brain": "gpt-oss-20b", "vision": "gemma-4-31b", "multimodal": "qwen3.8-27b",
                     "mode": "multimodal"})
    assert not r["ok"] and len(r["errors"]) == 2
    assert r["roles"]["brain"] == "tool-model" and r["roles"]["vision"] == "vision-model"
    assert r["roles"]["multimodal"] == "qwen3.8-27b" and r["brain"]["mode"] == "multimodal"
    assert json.loads(conf.read_text())["roles"]["multimodal_model"] == "qwen3.8-27b"
    b2 = cb.Brains(FakeSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=str(conf),
                   catalog=CAT)
    assert b2.roles()["multimodal"] == "qwen3.8-27b"
    assert b.set_roles({"mode": "telepathy"})["ok"] is False


# -------------------------------------------------------------------- tools
def test_tool_never_raises():
    b = brains()
    r = run(b.tool("scan_summary"))
    assert r["ok"] is False and "lidar unplugged" in r["error"]
    r = run(b.tool("goto", {"x": "here", "y": 0}))
    assert r["ok"] is False and "meters" in r["error"]
    assert run(b.tool("goto", {"x": float("nan"), "y": 0}))["ok"] is False
    assert run(b.tool("goto", {"x": 0.1}))["ok"] is False                 # missing y
    assert run(b.tool("teleport", {}))["ok"] is False
    assert run(b.tool("say", "greeting"))["ok"] is False                  # not an object


def test_goto_validation():
    assert cb.validate_goto("0.5", 0) == (0.5, 0.0, None)
    for bad in (("here", 0), (float("nan"), 0), (0, float("inf")), (True, 0), (None, 0)):
        assert cb.validate_goto(*bad)[2], bad
    assert cb.goto_range_error(3.5, 0.0, {"x": 0.0, "y": 0.0})
    assert cb.goto_range_error(2.5, 0.0, {"x": 0.0, "y": 0.0}) is None


def test_signed_turn_right_runs_the_mirrored_gait():
    b = brains()
    r = run(b.tool("gesture", {"name": "turn_in_place", "direction": "right"}))
    assert r["ok"] and r["direction"] == "right"
    name, fn, total = b.sim.started[-1]
    assert name == "turn_in_place_right" and total > 3
    import pebble_gestures2 as pg2
    from pebble_gait import WaveGait
    g = WaveGait()
    ql, _ = pg2.turn_in_place(g, 2.0)
    qr, _ = fn(g, 2.0)
    assert abs(qr[:, 0] - ql[:, 0]).max() > 0.02             # the coxa yaws the other way
    r = run(b.tool("gesture", {"name": "wave", "direction": "right"}))
    assert r["ok"] is False
    r = run(b.tool("gesture", {"name": "turn_in_place", "direction": "up"}))
    assert r["ok"] is False
    r = run(b.tool("gesture", {"name": "turn_in_place", "direction": "left"}))
    assert r["ok"] and r["direction"] == "left" and b.sim.started[-1][0] == "turn_in_place"


def test_tools_are_built_with_live_enums():
    b = brains()
    b.sim.gesture_names.append("point_there")
    tools = {t["function"]["name"]: t["function"] for t in b.tools_for()}
    assert "point_there" in tools["gesture"]["parameters"]["properties"]["name"]["enum"]
    assert tools["say"]["parameters"]["properties"]["word"]["enum"] == b.sim.lexicon
    for out in ("arrived", "cliff", "stuck", "blocked", "timeout", "user", "preempted", "FELL"):
        assert out in tools["goto"]["description"]
    assert {"look", "list_gestures", "compose_gesture", "check_gesture", "save_gesture"} <= set(tools)
    assert "point_there" in b.system_for()



# ------------------------------------------------------------ D056: the tool registry
# EXTRA_TOOLS and GATED as they were on 2026-09-25, BEFORE harness/capabilities.py existed
# (the registry task's snapshot: cockpit_brains.TOOLS after `look`, and GATED as a set).
# The sha256 is over EXTRA_TOOLS' canonical JSON (sort_keys, compact): no file needed.
EXTRA_BEFORE = ("compose_gesture", "check_gesture", "save_gesture", "find_object", "remember",
                "where_is", "recall", "go_back_to", "forget")
EXTRA_BEFORE_SHA = "5c53a90ca8083e8679fd70ca2170cf0c67a786e7ffb49eff43710e342333d4e4"
GATED_BEFORE = frozenset({"goto", "gesture", "compose_gesture", "find_object", "go_back_to", "turn", "move"})
REGISTRY_SNAPSHOTS = os.environ.get(
    "ROCKY_D056_SNAPSHOTS",
    "/tmp/claude-1000/-home-bitwisebard-Development-rocky/"
    "617aee55-a110-4e95-989e-f8423ee0b4fa/scratchpad/d056")


def _canon_sha(obj):
    import hashlib
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False).encode("utf-8")).hexdigest()


def test_extra_tools_and_gated_are_registry_views_equal_to_before():
    import harness.capabilities as C
    from harness.intent import MOTION_TOOLS
    assert isinstance(cb.EXTRA_TOOLS, list)
    assert tuple(t["function"]["name"] for t in cb.EXTRA_TOOLS) == EXTRA_BEFORE
    assert _canon_sha(cb.EXTRA_TOOLS) == EXTRA_BEFORE_SHA
    assert cb.GATED == GATED_BEFORE == C.GATED_NAMES
    assert set(MOTION_TOOLS) <= cb.GATED and "stop" not in cb.GATED     # talk mode's motion stays gated
    assert cb.MEMORY_TOOLS == C.MEMORY_NAMES == ("remember", "where_is", "recall", "go_back_to", "forget")
    for name in ("COMPOSE_DOC", "FIND_DOC", "REMEMBER_DOC", "WHERE_IS_DOC", "RECALL_DOC", "GO_BACK_DOC",
                 "FORGET_DOC", "FIND_MAX_STEPS", "FIND_MAX_STEPS_CAP"):
        assert getattr(cb, name) == getattr(C, name), name          # still importable from here
    # the cockpit's model list (build_tools' own + EXTRA_TOOLS) is the registry's, every capability on
    b = brains()
    want = C.to_openai_tools(C.build(b.sim.gesture_names, b.sim.lexicon, list(cb.SIGNED),
                                     has_eye=True, has_memory=True, is_cockpit=True))
    assert json.dumps(b.tools_for(look=True)) == json.dumps(want)


def test_extra_tools_and_gated_equal_the_pre_registry_snapshot():
    path = os.path.join(REGISTRY_SNAPSHOTS, "snapshot_openai_tools_variants.json")
    if not os.path.exists(path):
        pytest.skip(f"no D056 snapshot at {path} (the hashes above still pin it)")
    with open(path, encoding="utf-8") as f:
        before = json.load(f)["cockpit_brains.TOOLS (build_tools(look=True, extra=EXTRA_TOOLS))"]
    assert json.dumps(cb.TOOLS) == json.dumps(before)                                  # key order too
    assert json.dumps(cb.EXTRA_TOOLS) == json.dumps(
        [t for t in before if t["function"]["name"] in EXTRA_BEFORE])
    with open(os.path.join(REGISTRY_SNAPSHOTS, "snapshot_sets.json"), encoding="utf-8") as f:
        sets = json.load(f)
    assert cb.GATED == frozenset(sets["GATED_raw"]) == frozenset(sets["GATED"])
    assert list(cb.TOOL_NAMES) == sets["cockpit_dispatch (CockpitSim.tool -> Brains.tool accepts TOOL_NAMES)"]
    assert list(cb.NOTED) == sets["NOTED"] and list(cb.MEMORY_TOOLS) == sets["MEMORY_TOOLS"]


def test_every_registry_tool_this_cockpit_has_is_dispatchable():
    """The registry check (a test, not a startup assertion): every registry tool whose
    `requires` this cockpit has is in Brains.tool's dispatch set with a route, and every
    name it dispatches is a registry tool."""
    b = brains()
    assert cb.cockpit_flags(b.sim) == {"has_eye": True, "has_memory": False, "is_cockpit": True}
    assert cb.registry_problems(b) == []
    assert cb.registry_problems(b, {"has_eye": True, "has_memory": True, "is_cockpit": True}) == []
    b.sim.memory = SimpleNamespace()                        # a scene memory: the memory tools are required
    assert cb.cockpit_flags(b.sim)["has_memory"] is True and cb.registry_problems(b) == []
    # the routes are the callables the executor always called
    assert b._route("gesture") == b._gesture and b._route("list_gestures") == b.list_gestures
    assert b._route("move") == b.move and b._route("turn") == b.turn and b._route("look") == b.look
    assert b._route("remember") == b.mem_remember and b._route("go_back_to") == b.mem_go_back_to
    assert b._route("say") == b.sim.tool_say and b._route("goto") == b.sim.tool_goto


def test_the_registry_check_catches_a_drift(monkeypatch):
    b = brains()
    monkeypatch.setattr(cb, "OWN_ROUTES", {k: v for k, v in cb.OWN_ROUTES.items() if k != "move"})
    assert any(p.startswith("move: Brains.tool accepts it but has no route") for p in cb.registry_problems(b))
    monkeypatch.undo()
    monkeypatch.setattr(cb, "TOOL_NAMES", tuple(n for n in cb.TOOL_NAMES if n != "stop") + ("dance",))
    probs = cb.registry_problems(b)
    assert "stop: a registry tool this cockpit has, but Brains.tool refuses it" in probs
    assert "dance: Brains.tool runs it, but the tool registry does not know it" in probs


def test_a_sim_without_the_tool_is_an_error_result_as_before(monkeypatch):
    b = brains()
    monkeypatch.delattr(FakeSim, "tool_status")
    r = run(b.tool("status"))
    assert r["ok"] is False and r["error"].startswith("status failed: AttributeError")
    assert run(b.tool("list_gestures"))["ok"] is True       # the one plain (not awaited) route


# --------------------------------------------------------------------- chat
def test_failing_tool_and_bad_arguments_keep_the_transcript_valid():
    s = Script(tool_model=[
        {"content": "", "tool_calls": [call("goto", '{"x": NaN, "y": 0}', "a"),
                                       call("scan_summary", {}, "b"),
                                       call("goto", "not json", "c")]},
        answer("I could not move: the arguments were bad and the lidar failed."),
        answer("second turn fine")])
    b = brains(s)
    r = run(b.chat("walk somewhere", mode="local"))
    assert r["model"] == "tool-model" and len(r["trace"]) == 3
    assert all(t["result"]["ok"] is False for t in r["trace"])
    r2 = run(b.chat("again", mode="local"))
    assert r2["reply"] == "second turn fine"
    for _model, msgs, _tools in s.calls:
        assert transcript_ok(msgs)
    # the older turn is squeezed to user + final sentence: no tool traffic survives
    last = s.calls[-1][1]
    assert [m["role"] for m in last] == ["system", "user", "assistant", "user"]


def test_fallback_chain_says_who_answered():
    s = Script(tool_model=[TimeoutError("read timed out")],
               qwen3_8_27b_iq4=[answer("done by the fallback")])
    b = brains(s)
    r = run(b.chat("hello", mode="local"))
    assert r["model"] == "qwen3.8-27b-iq4"
    assert "tool-model failed" in r["reply"] and "answered by qwen3.8-27b-iq4" in r["reply"]


def test_every_model_down_falls_back_to_the_regex_brain_once():
    b = brains(Script())                                        # every model 404s
    r = run(b.chat("turn left", mode="local"))
    assert r["model"] == "talk" and "regex brain" in r["reply"]
    assert b.sim.started[-1][0] == "turn_in_place"
    # a model that ran a tool and THEN died must not have the regex brain act again
    s = Script(tool_model=[{"content": "", "tool_calls": [call("gesture", {"name": "wave"}, "g")]},
                           RuntimeError("HTTP 500")])
    b = brains(s)
    run(b.chat("do jazz hands", mode="local"))
    assert [x[0] for x in b.sim.started] == ["wave"]


def test_hop_cap_gives_up_with_a_sentence():
    s = Script(tool_model=[{"content": "", "tool_calls": [call("status", {}, "s")]}])
    b = brains(s)
    r = run(b.chat("where are you", mode="local"))
    assert r["gave_up"] and "gave up after 6" in r["reply"] and len(r["trace"]) == 6


def test_history_is_bounded():
    s = Script(tool_model=[answer("ok")])
    b = brains(s)
    for i in range(15):
        run(b.chat(f"message {i}", mode="local"))
    msgs = s.calls[-1][1]
    assert len(msgs) == 1 + 2 * 12 + 1
    assert msgs[1]["content"] == "message 2" and msgs[-1]["content"] == "message 14"


def test_concurrent_messages_do_not_interleave():
    active, peak = [0], [0]
    lock = threading.Lock()

    def slow(model, messages, tools, max_tokens):
        with lock:
            active[0] += 1
            peak[0] = max(peak[0], active[0])
        time.sleep(0.05)
        with lock:
            active[0] -= 1
        return answer("ok")
    b = brains(slow)

    async def both():
        await asyncio.gather(b.chat("typed", mode="local"), b.chat("spoken", mode="local", source="voice"))
    run(both())
    assert peak[0] == 1 and len(b.hist["local"]) == 2


def test_multimodal_sends_the_eye_and_strips_it_for_a_text_fallback():
    s = Script(qwen3_5_9b=[RuntimeError("HTTP 503")], tool_model=[answer("text brain here")])   # D055 chain
    b = brains(s)
    r = run(b.chat("what do you see?", mode="multimodal"))
    first = s.calls[0][1][-1]["content"]
    assert s.calls[0][0] == "qwen3.5-9b"
    assert isinstance(first, list) and first[1]["type"] == "image_url"
    fallback_user = s.calls[1][1][-1]["content"]
    assert isinstance(fallback_user, str) and "call look" in fallback_user
    assert r["model"] == "tool-model"


def test_look_skips_empty_descriptions_and_strips_thinking():
    s = Script(vision_model=[answer("<think>budget gone</think>")],
               gemma_4_26b_a4b=[answer("A red box half a meter ahead; open floor to the left.")])
    b = brains(s)
    r = run(b.look())
    assert r["ok"] and r["model"] == "gemma-4-26b-a4b"
    assert any("empty" in f for f in r["fallback"])          # the vision-model's empty answer
    b = brains(Script(vision_model=[answer("")], gemma_4_26b_a4b=[answer("  ")]))
    r = run(b.look())
    # two empty answers plus the unlisted 12B's skip note (D055 vision chain)
    assert r["ok"] is False and sum("empty" in x for x in r["tried"]) == 2 and len(r["tried"]) == 3
    b = brains(Script())
    b.sim.frames["eye"] = (0, b"")
    assert run(b.look())["ok"] is False


# -------------------------------------------------------------------- voice
def test_voice_gate():
    b = brains()
    v = b.voice_result({"segments": [{"text": " Thank you.", "no_speech_prob": 1e-8,
                                      "avg_logprob": -0.43}]}, mode="talk")
    assert v == {"ok": True, "text": "", "wake": False, "motion_blocked": False,
                 "dropped": v["dropped"]} and v["dropped"]
    v = b.voice_result("go forward 10 cm", mode="talk")
    assert v["motion_blocked"] and not v["wake"]
    # the legacy UI auto-sends the transcript with no source: still treated as voice
    r = run(b.chat("go forward 10 cm", mode="talk"))
    assert r["voice"] and r["motion_blocked"] and b.sim.x == 0.0
    r = run(b.chat("go forward 10 cm", mode="talk", source="voice", trusted=True))
    assert r["motion_allowed"] and abs(b.sim.x - 0.1) < 1e-9
    v = b.voice_result("pebble, go forward 10 cm", mode="talk")
    assert v["wake"] and not v["motion_blocked"]
    r = run(b.chat("pebble, go forward 10 cm", mode="talk", source="voice"))
    assert r["motion_allowed"] and abs(b.sim.x - 0.2) < 1e-9
    assert not b.voice_result("say greeting", mode="talk")["motion_blocked"]
    assert b.voice_result("say greeting", mode="local")["motion_blocked"]   # an LLM may move on anything
    r = run(b.chat("stop", mode="talk", source="voice"))                     # stop is never gated
    assert r["trace"][0]["result"]["ok"]


def test_voice_gate_holds_for_llm_tool_calls():
    s = Script(tool_model=[{"content": "", "tool_calls": [call("goto", {"x": 0.3, "y": 0}, "g")]},
                           answer("I was told not to move.")])
    b = brains(s)
    r = run(b.chat("walk to the box", mode="local", source="voice"))
    assert r["trace"][0]["result"]["error"] == "voice_unconfirmed" and b.sim.x == 0.0
    assert r["motion_blocked"] and "wake word" in r["reply"]


# ---------------------------------------------------------- gesture composing
WAVE = [{"t": 0}, {"t": 1.0, "body": [0, -15, 0]},
        {"t": 1.8, "body": [0, -15, 0], "arm": {"0": [0, 70, -60]}},
        {"t": 2.6, "body": [0, -15, 0], "arm": {"0": [25, 70, -60]}},
        {"t": 3.4, "body": [0, -15, 0]}, {"t": 4.2}]


def test_compose_previews_a_feasible_gesture_and_saves_on_request(tmp_path, monkeypatch):
    b = brains()
    r = run(b.tool("compose_gesture", {"name": "hello_leg", "description": "wave leg 0",
                                       "keyframes": WAVE}))
    assert r["ok"] and r["previewed"] and r["note"] == "previewed, say save to keep it"
    assert r["report"][0].startswith("PASS")
    assert b.sim.started[-1][0] == "hello_leg"
    import pebble_keyframes as pk
    saved = {}

    def fake_save(spec, *a, **k):
        saved["spec"] = spec
        return str(tmp_path / f"{spec['name']}.json")
    monkeypatch.setattr(pk, "save_keyframe_gesture", fake_save)
    r = run(b.tool("save_gesture", {}))
    assert r["ok"] and saved["spec"]["name"] == "hello_leg"
    r = run(b.tool("save_gesture", {"name": "wave"}))
    assert r["ok"] is False and "built-in" in r["error"]


def test_compose_refuses_the_infeasible_without_moving():
    b = brains()
    bad = [{"t": 0}, {"t": 0.2, "arm": {"0": [0, 88, -25]}, "body": [0, 0, -40]}, {"t": 0.4}]
    r = run(b.tool("compose_gesture", {"name": "flail", "keyframes": bad}))
    assert r["ok"] is False and not r.get("previewed") and r["report"][0].startswith("FAIL")
    assert not b.sim.started
    r = run(b.tool("compose_gesture", {"name": "x", "keyframes": "not json"}))
    assert r["ok"] is False
    r = run(b.tool("check_gesture", {"spec": {"name": "w", "keyframes": WAVE}}))
    assert r["ok"] and r["feasible"] and not b.sim.started


def test_compose_is_motion_for_the_voice_gate():
    b = brains()
    r = run(b.tool("compose_gesture", {"name": "w", "keyframes": WAVE}, voice=True))
    assert r["error"] == "voice_unconfirmed" and not b.sim.started


@pytest.mark.parametrize("mode", ["talk", "local"])
def test_unknown_mode_and_empty_text(mode):
    b = brains(Script(tool_model=[answer("x")]))
    assert run(b.chat("", mode=mode))["reply"] == ""
    assert "unknown mode" in run(b.chat("hi", mode="psychic"))["reply"]


def test_voice_result_trusted_skips_the_wake_word(monkeypatch):
    """Hands-free sends trusted=1: the transcript may move the robot without 'pebble'."""
    import asyncio
    import cockpit_brains as cb
    b = cb.Brains.__new__(cb.Brains)
    b.state = {"mode": "talk"}; b._voice_last = (0.0, ""); b._log = lambda *a, **k: None
    data = {"text": "walk forward thirty centimeters", "segments": [{"text": "walk forward thirty centimeters", "no_speech_prob": 0.1, "avg_logprob": -0.3}]}
    r = b.voice_result(data, "talk")
    assert r["text"] and r["motion_blocked"] is True                      # no wake word: blocked
    monkeypatch.setattr(cb, "WHISPER_URL", "http://127.0.0.1:1")           # unreachable: transcribe must fail early on size, not on the network
    out = asyncio.run(b.transcribe(b"x" * 10, mode="talk", trusted=True))
    assert out["ok"] is False and "too short" in out["error"]


# ------------------------------------------------------------ stop first (2026-09-25)
STOP_LINES = ["stop", "stop right now", "STOP!", "halt", "freeze", "Pebble, stop."]
NOT_STOPS = ["don't stop", "Don\u2019t stop walking", "do not stop", "non-stop", "stop sign ahead?",
             "is that a stop sign?", "go to the bus stop"]
STOP_RESULT = {"ok": True, "mode": "safe_stop"}


class LoggedSim(FakeSim):
    """FakeSim with the cockpit's cmd_log (what _TurnStops reads) and a goto that
    walks until a stop arrives (tool_stop ends it with stopped='user', as the
    cockpit's does); once stopped, a goto 'arrives' at once and moves x."""

    def __init__(self):
        super().__init__()
        self.cmd_log = []
        self.walking = None

    def note_cmd(self, line):
        super().note_cmd(line)
        self.cmd_log.append((0.0, line))

    async def tool_goto(self, x, y):
        if self.stops == 0:
            self.walking = asyncio.Event()
            await asyncio.wait_for(self.walking.wait(), 5.0)
            return {"ok": True, "stopped": "user", "pose": self.pose()}
        self.x = float(x)
        return {"ok": True, "stopped": "arrived", "pose": self.pose()}

    async def tool_stop(self):
        if self.walking is not None:
            self.walking.set()
        return await super().tool_stop()


class FakeAnthropic:
    """Stands in for the anthropic module: records every client and every
    messages.create; replies are scripted (SimpleNamespace content blocks)."""

    def __init__(self, replies=()):
        self.created, self.calls, self.replies = [], [], list(replies)
        outer = self

        class Anthropic:
            def __init__(self, **kw):
                outer.created.append(kw)
                self.messages = SimpleNamespace(create=outer._create)
        self.Anthropic = Anthropic

    def _create(self, **kw):
        self.calls.append(kw)
        return self.replies.pop(0)


def _claude_ready(monkeypatch, replies=()):
    fake = FakeAnthropic(replies)
    monkeypatch.setitem(sys.modules, "anthropic", fake)
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test-key")
    return fake


def _text(t):
    return SimpleNamespace(content=[SimpleNamespace(type="text", text=t)])


def _uses(*calls):
    return SimpleNamespace(content=[SimpleNamespace(type="tool_use", id=f"u{i}", name=n, input=a)
                                    for i, (n, a) in enumerate(calls)])


@pytest.fixture(autouse=True)
def _stop_first_default(monkeypatch):
    monkeypatch.delenv(cb.STOP_FIRST_ENV, raising=False)      # the shell must not switch it off


@pytest.mark.parametrize("line", STOP_LINES + ["whoa", "abort", "hey rocky stop", "don't go, stop!",
                                               "stop, there's a stop sign", "it won't stop!", "no stop!",
                                               "remember to stop", "come to a full stop"])
def test_stop_line_is_intents_stop(line):
    assert cb.stop_line(line) and cb.intent_stop(line)


@pytest.mark.parametrize("line", NOT_STOPS + ["dont stop", "pebble, don't stop", "a pit stop"])
def test_stop_line_skips_uses_that_are_not_an_order(line):
    assert cb.intent_stop(line)          # intent (talk mode) still reads these as a stop ...
    assert not cb.stop_line(line)        # ... the model modes hand them to the model


@pytest.mark.parametrize("line", ["hello", "stopping", "nonstop party", "walk forward 30 cm", "stand still", ""])
def test_stop_line_never_invents_a_stop(line):
    assert not cb.intent_stop(line) and not cb.stop_line(line)


@pytest.mark.parametrize("mode", ["local", "multimodal", "claude"])
@pytest.mark.parametrize("line", STOP_LINES)
def test_stop_first_runs_the_stop_and_never_asks_a_model(line, mode, monkeypatch):
    fake = _claude_ready(monkeypatch, [_text("should never be asked")])
    s = Script(tool_model=[answer("should never be asked")], gemma_4_26b_a4b=[answer("nor this one")])
    b = brains(s)
    r = run(b.chat(line, mode=mode, source="typed"))
    assert b.sim.stops == 1 and b.sim.cmds == ["tool stop {}"]          # the stop tool, noted like any stop call
    assert r["trace"] == [{"tool": "stop", "args": {}, "result": STOP_RESULT}]
    assert r["reply"] == cb.STOP_FIRST_REPLY == "stopped (safe-stop)."
    assert r["stop_first"] is True and r["model"] is None and r["mode"] == mode
    assert s.calls == [] and fake.created == [] and fake.calls == []   # no model client was touched
    assert b.hist[mode][-1]["reply"] == cb.STOP_FIRST_REPLY           # the next turn knows it stopped


def test_stop_first_needs_no_claude_key(monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    b = brains()
    r = run(b.chat("stop", mode="claude"))
    assert b.sim.stops == 1 and r["reply"] == cb.STOP_FIRST_REPLY


@pytest.mark.parametrize("mode", ["local", "multimodal"])
def test_spoken_stop_without_the_wake_word_still_stops(mode):
    s = Script(tool_model=[answer("model")], gemma_4_26b_a4b=[answer("model")])
    b = brains(s)
    v = b.voice_result("stop", mode=mode)
    assert not v["wake"] and not v["motion_blocked"]                  # the page sends it at once, no Enter
    assert b.voice_result("walk forward 30 cm", mode=mode)["motion_blocked"]   # the gate still holds motion
    assert b.voice_result("don't stop", mode=mode)["motion_blocked"]           # not a stop: a model may move on it
    r = run(b.chat("stop", mode=mode, source="voice", trusted=False))
    assert r["voice"] and not r["motion_allowed"] and not r["motion_blocked"]
    assert b.sim.stops == 1 and "stopped" in r["reply"] and s.calls == []
    r = run(b.chat("halt", mode=mode))                                 # the legacy UI: no source, the last transcript
    assert b.sim.stops == 2 and s.calls == []


@pytest.mark.parametrize("mode", ["local", "multimodal"])
@pytest.mark.parametrize("line", NOT_STOPS)
def test_not_a_stop_reaches_the_model(line, mode):
    s = Script(tool_model=[answer("carrying on")], qwen3_5_9b=[answer("carrying on")])   # D055 multimodal chain
    b = brains(s)
    r = run(b.chat(line, mode=mode, source="typed"))
    assert len(s.calls) == 1 and r["reply"].endswith("carrying on") and "stop_first" not in r
    assert b.sim.stops == 0 and r["trace"] == []


def test_not_a_stop_reaches_claude(monkeypatch):
    fake = _claude_ready(monkeypatch, [_text("carrying on")])
    b = brains()
    r = run(b.chat("don't stop", mode="claude", source="typed"))
    assert len(fake.calls) == 1 and r["reply"] == "carrying on" and b.sim.stops == 0


def test_talk_mode_keeps_intents_stop():
    b = brains()
    r = run(b.chat("don't stop", mode="talk"))                        # talk: every intent stop word stops
    assert r["reply"] == "stopping." and r["trace"][0]["tool"] == "stop" and b.sim.stops == 1
    assert "stop_first" not in r


def test_a_stop_does_not_wait_for_a_running_model_turn():
    gate = threading.Event()

    def thinking(model, messages, tools, max_tokens):
        assert gate.wait(5.0), "the stop never overtook the turn"
        return answer("done thinking")
    b = brains(thinking)

    async def scenario():
        turn = asyncio.create_task(b.chat("walk around the room", mode="local"))
        await asyncio.sleep(0.05)                                      # the turn holds the lock; its model thinks
        r = await asyncio.wait_for(b.chat("stop", mode="local"), 2.0)
        overtook = b.sim.stops == 1 and not turn.done()
        gate.set()
        return r, overtook, await turn
    r, overtook, first = run(scenario())
    assert overtook and r["reply"] == cb.STOP_FIRST_REPLY and first["reply"] == "done thinking"


def test_a_talk_stop_does_not_wait_for_a_running_talk_turn():
    b = brains()

    async def scenario():
        release = asyncio.Event()

        async def slow_goto(x, y):
            await asyncio.wait_for(release.wait(), 5.0)
            return {"ok": True, "stopped": "user", "pose": b.sim.pose()}
        b.sim.tool_goto = slow_goto
        walk = asyncio.create_task(b.chat("go to 0.4 0.2", mode="talk"))
        await asyncio.sleep(0.05)
        r = await asyncio.wait_for(b.chat("stop", mode="talk"), 2.0)
        overtook = b.sim.stops == 1 and not walk.done()
        release.set()
        return r, overtook, await walk
    r, overtook, walk = run(scenario())
    assert overtook and r["reply"] == "stopping." and r["trace"][0]["result"] == STOP_RESULT
    assert "stopped: user" in walk["reply"]


def _stop_mid_walk(b, mode, text):
    """Send `text` in `mode`; once its goto is walking, the operator says stop (stop first)."""
    async def scenario():
        turn = asyncio.create_task(b.chat(text, mode=mode))
        for _ in range(200):
            if b.sim.walking is not None:
                break
            await asyncio.sleep(0.01)
        stop = await asyncio.wait_for(b.chat("stop", mode=mode), 2.0)
        return stop, await asyncio.wait_for(turn, 5.0)
    return run(scenario())


def test_motion_after_an_operator_stop_is_refused_for_the_rest_of_the_turn():
    s = Script(tool_model=[
        {"content": "", "tool_calls": [call("goto", {"x": 0.3, "y": 0}, "a")]},
        {"content": "", "tool_calls": [call("goto", {"x": 0.5, "y": 0}, "b")]},
        {"content": "", "tool_calls": [call("gesture", {"name": "wave"}, "c"), call("say", {"word": "acknowledge"}, "d")]},
        answer("The operator stopped me.")])
    b = cb.Brains(LoggedSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None,
                  complete=s, catalog=CAT)
    stop, turn = _stop_mid_walk(b, "local", "walk to the door")
    assert stop["reply"] == cb.STOP_FIRST_REPLY
    res = [(t["tool"], t["result"].get("stopped") or t["result"].get("error") or t["result"]["ok"])
           for t in turn["trace"]]
    assert res == [("goto", "user"), ("goto", "operator_stopped"), ("gesture", "operator_stopped"),
                   ("say", True)]                                     # a chord is not motion
    assert b.sim.x == 0.0 and b.sim.started == [] and turn["reply"] == "The operator stopped me."
    for _model, msgs, _tools in s.calls:
        assert transcript_ok(msgs)


def test_motion_after_an_operator_stop_is_refused_in_claude_too(monkeypatch):
    fake = _claude_ready(monkeypatch, [_uses(("goto", {"x": 0.3, "y": 0})), _uses(("goto", {"x": 0.5, "y": 0})),
                                       _text("stopped, as asked")])
    b = cb.Brains(LoggedSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None, catalog=CAT)
    stop, turn = _stop_mid_walk(b, "claude", "walk to the door")
    assert stop["reply"] == cb.STOP_FIRST_REPLY and len(fake.calls) == 3
    assert [t["result"].get("stopped") or t["result"].get("error") for t in turn["trace"]] == \
        ["user", "operator_stopped"]
    assert b.sim.x == 0.0


def test_the_turns_own_stop_is_not_an_operator_stop():
    s = Script(tool_model=[{"content": "", "tool_calls": [call("stop", {}, "a")]},
                           {"content": "", "tool_calls": [call("gesture", {"name": "wave"}, "b")]},
                           answer("stopped, then waved")])
    b = cb.Brains(LoggedSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None,
                  complete=s, catalog=CAT)
    r = run(b.chat("hold still, then wave", mode="local"))            # no stop word: the model decides
    assert [t["tool"] for t in r["trace"]] == ["stop", "gesture"] and r["trace"][1]["result"]["ok"]
    assert [x[0] for x in b.sim.started] == ["wave"]


def test_a_failed_chain_does_not_run_the_line_after_an_operator_stop():
    holder = {}

    def down(model, messages, tools, max_tokens):
        holder["b"].sim.cmd_log.append((0.0, "teleop  "))              # the STOP key while the chain fails
        raise RuntimeError("HTTP 503")
    b = cb.Brains(LoggedSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None,
                  complete=down, catalog=CAT)
    holder["b"] = b
    r = run(b.chat("turn left", mode="local"))
    assert b.sim.started == [] and r["trace"] == [] and "stopped meanwhile" in r["reply"]
    assert r["model"] is None


def test_stop_first_can_be_switched_off_for_the_bench(monkeypatch):
    s = Script(tool_model=[{"content": "", "tool_calls": [call("stop", {}, "a")]}, answer("Stopped.")])
    b = brains(s, stop_first=False)
    r = run(b.chat("stop", mode="local"))
    assert r["model"] == "tool-model" and "stop_first" not in r and len(s.calls) == 2
    assert b.voice_result("stop", mode="local")["motion_blocked"]      # as before: a model may move on it
    monkeypatch.setenv(cb.STOP_FIRST_ENV, "0")
    assert brains().stop_first is False
    monkeypatch.setenv(cb.STOP_FIRST_ENV, "1")
    assert brains().stop_first is True


def test_a_stop_that_fails_says_so():
    b = brains()

    async def dead():
        raise RuntimeError("the sim thread has stopped")
    b.sim.tool_stop = dead
    r = run(b.chat("stop", mode="local"))
    assert r["trace"][0]["result"]["ok"] is False
    assert r["reply"].startswith("stop FAILED:") and "red STOP button" in r["reply"]


# ------------------------------------------------------------------ move (relative, robot frame)
class PoseSim(FakeSim):
    """FakeSim with a full pose (x, y, yaw) and a goto that records its targets;
    `goto_result` (a dict) replaces the arrival: a guard's refusal or veto."""

    def __init__(self, x=0.0, y=0.0, yaw_deg=0.0):
        super().__init__()
        self.x, self.y, self.yaw = x, y, yaw_deg
        self.gotos, self.goto_result = [], None

    def pose(self):
        return {"x": self.x, "y": self.y, "yaw_deg": self.yaw}

    async def tool_goto(self, x, y):
        tx, ty, err = cb.validate_goto(x, y)
        if err:
            return {"ok": False, "error": err}
        self.gotos.append((tx, ty))
        if self.goto_result is not None:
            return dict(self.goto_result, pose=self.pose())
        self.x, self.y = tx, ty
        return {"ok": True, "stopped": "arrived", "pose": self.pose()}


def pose_brains(script=None, **pose):
    return cb.Brains(PoseSim(**pose), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None,
                     complete=script, catalog=CAT)


def test_move_is_in_the_schema_with_robot_frame_parameters():
    tools = {t["function"]["name"]: t["function"] for t in brains().tools_for()}
    mv = tools["move"]
    assert mv["parameters"]["properties"]["forward_m"]["type"] == "number"
    assert mv["parameters"]["properties"]["left_m"]["type"] == "number"
    assert mv["parameters"]["required"] == ["forward_m"]
    assert "move(forward_m=0.3)" in mv["description"] and "move(forward_m=-0.2)" in mv["description"]
    assert "MAP coordinates" in mv["description"]
    for out in ("arrived", "cliff", "stuck", "blocked", "timeout", "user", "preempted", "FELL"):
        assert out in mv["description"], out
    assert "use move instead" in tools["goto"]["description"]
    assert {t["function"]["name"] for t in cb.TOOLS} >= {"move", "goto"}


def test_the_prompt_asks_for_move_not_trigonometry():
    sysmsg = brains().system_for()
    assert "cos(yaw" not in sysmsg and "call status first" not in sysmsg
    assert "Relative moves use move" in sysmsg and "remembered positions" in sysmsg
    assert "move(forward_m=0.3)" in cb.UNITS_NOTE


def test_move_is_a_gated_motion_tool_and_not_noted_itself():
    assert "move" in cb.GATED and "move" in cb.TOOL_NAMES and "move" not in cb.NOTED
    b = pose_brains()
    r = run(b.tool("move", {"forward_m": 0.3}, voice=True))
    assert r["error"] == "voice_unconfirmed" and b.sim.gotos == []
    r = run(b.tool("move", {"forward_m": 0.3}))
    assert r["stopped"] == "arrived" and b.sim.gotos == [(0.3, 0.0)]
    assert b.sim.cmds == ['tool goto {"x": 0.3, "y": 0.0}']        # a replay repeats the goto


@pytest.mark.parametrize("yaw, fwd, left, want", [
    (0.0, 0.3, 0.0, (1.3, 2.0)), (0.0, -0.2, 0.0, (0.8, 2.0)), (0.0, 0.0, 0.25, (1.0, 2.25)),
    (90.0, 0.3, 0.0, (1.0, 2.3)), (90.0, 0.0, 0.2, (0.8, 2.0)), (90.0, -0.2, 0.0, (1.0, 1.8)),
    (-90.0, 0.3, 0.0, (1.0, 1.7)), (-90.0, 0.0, 0.2, (1.2, 2.0)), (-90.0, 0.3, -0.1, (0.9, 1.7)),
    (180.0, 0.5, 0.0, (0.5, 2.0))])
def test_move_target_maps_the_robot_frame_to_the_map(yaw, fwd, left, want):
    tx, ty = cb.move_target({"x": 1.0, "y": 2.0, "yaw_deg": yaw}, fwd, left)
    assert (tx, ty) == pytest.approx(want, abs=1e-3)


def test_move_target_without_a_heading_faces_map_x():
    assert cb.move_target({"x": 0.1, "y": 0.2}, 0.3) == (0.4, 0.2)


@pytest.mark.parametrize("yaw, want", [(0.0, (0.3, 0.0)), (90.0, (0.0, 0.3)), (-90.0, (0.0, -0.3))])
def test_move_runs_the_goto_from_the_pose_at_call_time(yaw, want):
    b = pose_brains(yaw_deg=yaw)
    r = run(b.tool("move", {"forward_m": 0.3}))
    assert b.sim.gotos == [pytest.approx(want, abs=1e-3)]
    assert r["ok"] and r["stopped"] == "arrived"
    assert r["move"]["forward_m"] == 0.3 and r["move"]["left_m"] == 0.0
    assert r["move"]["from"] == {"x": 0.0, "y": 0.0, "yaw_deg": yaw}
    assert (r["move"]["target"]["x"], r["move"]["target"]["y"]) == pytest.approx(want, abs=1e-3)
    run(b.tool("move", {"forward_m": 0.2, "left_m": 0.1}))           # the NEW pose: moves chain
    assert len(b.sim.gotos) == 2


@pytest.mark.parametrize("res", [
    {"ok": False, "stopped": "blocked", "detail": "blocked: void at 12 deg (latched by the void guard)"},
    {"ok": False, "stopped": "blocked", "detail": "a gesture is running — stop first"},
    {"ok": False, "stopped": "cliff"}, {"ok": False, "stopped": "stuck"},
    {"ok": False, "stopped": "user"}, {"ok": False, "stopped": "FELL"},
    {"ok": False, "error": "target is 3.20 m away; goto is capped at 3 m"}])
def test_move_passes_the_goto_guards_through_unchanged(res):
    b = pose_brains()
    b.sim.goto_result = res
    r = run(b.tool("move", {"forward_m": 0.3}))
    for k, v in res.items():
        assert r[k] == v, k
    assert r["move"]["target"] == {"x": 0.3, "y": 0.0}


@pytest.mark.parametrize("args, frag", [
    ({"forward_m": 30}, "METERS"), ({"forward_m": -20}, "at most 1.5 m"),
    ({"forward_m": 0.2, "left_m": 1.6}, "at most"), ({"forward_m": 1.2, "left_m": 1.2}, "1.70 m"),
    ({"forward_m": 0}, "needs a distance"), ({"forward_m": 0.0, "left_m": 0.0}, "turn_in_place"),
    ({"forward_m": "here"}, "numbers"), ({"forward_m": float("nan")}, "finite"),
    ({"forward_m": True}, "boolean"), ({"left_m": 0.3}, "bad arguments"),
    ({"forward_m": 0.3, "x": 1}, "bad arguments")])
def test_move_refuses_bad_or_out_of_envelope_arguments_before_walking(args, frag):
    b = pose_brains()
    r = run(b.tool("move", args))
    assert r["ok"] is False and frag in r["error"], r
    assert b.sim.gotos == [] and b.sim.cmds == []


def test_move_within_the_envelope_is_accepted():
    assert cb.validate_move(1.5, 0) == (1.5, 0.0, None)
    assert cb.validate_move("0.3", None) == (0.3, 0.0, None)
    assert cb.validate_move(-0.2) == (-0.2, 0.0, None)
    assert cb.validate_move(1.51, 0)[2]


def test_move_without_a_pose_is_refused():
    b = cb.Brains(SimpleNamespace(note_cmd=lambda line: None, log=lambda m: None, events=deque()),
                  base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None, catalog=CAT)
    r = run(b.tool("move", {"forward_m": 0.3}))
    assert r == {"ok": False, "error": "no pose: cannot aim a move"}


def test_a_model_turn_that_calls_move_walks_where_it_said():
    s = Script(tool_model=[{"content": "", "tool_calls": [call("move", {"forward_m": 0.3}, "m")]},
                           answer("Walked 30 cm forward.")])
    b = pose_brains(s, x=1.0, y=1.0, yaw_deg=90.0)
    r = run(b.chat("walk forward thirty centimeters", mode="local"))
    assert [t["tool"] for t in r["trace"]] == ["move"] and r["trace"][0]["args"] == {"forward_m": 0.3}
    assert r["trace"][0]["result"]["stopped"] == "arrived"
    assert b.sim.gotos == [pytest.approx((1.0, 1.3), abs=1e-3)]
    tool_msg = next(m for m in s.calls[1][1] if m["role"] == "tool")
    assert json.loads(tool_msg["content"])["move"]["target"] == {"x": 1.0, "y": 1.3}


def test_move_after_an_operator_stop_is_refused_mid_turn():
    holder = {}

    def script(model, msgs, tools, mt):
        if not any(m["role"] == "tool" for m in msgs):
            holder["b"].sim.cmd_log.append((0.0, "stop"))                # the console `stop`, mid-turn
            return {"content": "", "tool_calls": [call("move", {"forward_m": 0.3}, "m")]}
        return answer("I was stopped.")

    class LoggedSim(PoseSim):
        def __init__(self):
            super().__init__()
            self.cmd_log = deque()

    b = cb.Brains(LoggedSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None,
                  complete=script, catalog=CAT)
    holder["b"] = b
    r = run(b.chat("back up a bit", mode="local"))
    assert r["trace"][0]["result"]["error"] == "operator_stopped" and b.sim.gotos == []


def test_surface_move_goes_through_the_tool():
    b = pose_brains(yaw_deg=180.0)
    r = run(b.surface().move(0.2))
    assert r["stopped"] == "arrived" and b.sim.gotos == [pytest.approx((-0.2, 0.0), abs=1e-3)]
    r = run(b.surface(voice=True).move(0.2))
    assert r["error"] == "voice_unconfirmed"


# ------------------------------------------- review fixes (2026-09-25, second pass)
def _logged(script=None, sim=None):
    return cb.Brains(sim or LoggedSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None,
                     complete=script, catalog=CAT)


def _cmd_order(sim):
    return [line.split(" {")[0] for _t, line in sim.cmd_log]


def _queued_behind_a_walk(b, mode, walk, queued, stop="stop"):
    """`walk` is walking; `queued` is sent and waits for the lock; then the operator stops."""
    async def scenario():
        a = asyncio.create_task(b.chat(walk, mode=mode))
        for _ in range(200):
            if b.sim.walking is not None:
                break
            await asyncio.sleep(0.01)
        q = asyncio.create_task(b.chat(queued, mode=mode))
        await asyncio.sleep(0.05)                                      # B has arrived: it waits on the lock
        assert not q.done()
        s = await asyncio.wait_for(b.chat(stop, mode=mode), 2.0)
        return s, await asyncio.wait_for(a, 5.0), await asyncio.wait_for(q, 5.0)
    return run(scenario())


def test_a_line_queued_before_a_stop_does_not_move_after_it():
    """Review repro: 'walk to the door' walking, 'wave' waiting on the lock, then 'stop':
    the wave used to run after the stop (cmd order goto, stop, gesture)."""
    s = Script(tool_model=[{"content": "", "tool_calls": [call("goto", {"x": 0.3, "y": 0}, "a")]},
                           answer("Walking to the door."),
                           {"content": "", "tool_calls": [call("gesture", {"name": "wave"}, "b")]},
                           answer("I was stopped, so I did not wave.")])
    b = _logged(s)
    stop, _walk, wave = _queued_behind_a_walk(b, "local", "walk to the door", "wave")
    assert stop["reply"] == cb.STOP_FIRST_REPLY
    assert [(t["tool"], t["result"].get("error")) for t in wave["trace"]] == [("gesture", "operator_stopped")]
    assert b.sim.started == [] and _cmd_order(b.sim) == ["tool goto", "tool stop"]
    assert wave["reply"] == "I was stopped, so I did not wave."
    tool_msg = next(m for m in s.calls[-1][1] if m["role"] == "tool")
    assert "after sending this message" in json.loads(tool_msg["content"])["hint"]


def test_a_talk_line_queued_before_a_stop_does_not_move_after_it():
    b = _logged()
    stop, walk, turn = _queued_behind_a_walk(b, "talk", "go to 0.4 0.2", "turn left")
    assert stop["reply"] == "stopping." and "stopped: user" in walk["reply"]
    assert turn["trace"][-1]["result"]["error"] == "operator_stopped"
    assert cb.TALK_STOPPED_NOTE in turn["reply"] and b.sim.started == []
    assert _cmd_order(b.sim) == ["tool goto", "tool stop"]


def test_a_talk_move_whose_status_read_a_stop_overtakes_does_not_walk():
    """Review repro: 'go forward 30 cm' was still reading the pose when 'stop'
    overtook it; the goto then ran (x ended at 0.3)."""
    b = _logged()
    release = asyncio.Event()
    real_status = b.sim.tool_status

    async def slow_status():
        await asyncio.wait_for(release.wait(), 5.0)
        return await real_status()
    b.sim.tool_status = slow_status

    async def scenario():
        move = asyncio.create_task(b.chat("go forward 30 cm", mode="talk"))
        await asyncio.sleep(0.05)
        s = await asyncio.wait_for(b.chat("stop", mode="talk"), 2.0)
        release.set()
        return s, await asyncio.wait_for(move, 5.0)
    stop, move = run(scenario())
    assert stop["reply"] == "stopping." and b.sim.x == 0.0
    assert move["trace"][-1]["tool"] == "goto" and move["trace"][-1]["result"]["error"] == "operator_stopped"
    assert _cmd_order(b.sim) == ["tool stop"]


def test_another_turns_own_stop_does_not_refuse_a_queued_line():
    """A model's stop is not the operator's: the line queued behind that turn still moves."""
    gate = threading.Event()
    n = [0]

    def script(model, messages, tools, max_tokens):
        n[0] += 1
        if n[0] == 1:
            assert gate.wait(5.0)
            return {"content": "", "tool_calls": [call("stop", {}, "a")]}
        if n[0] == 3:
            return {"content": "", "tool_calls": [call("gesture", {"name": "wave"}, "b")]}
        return answer("done")
    b = _logged(script)

    async def scenario():
        a = asyncio.create_task(b.chat("hold still", mode="local"))
        await asyncio.sleep(0.05)
        q = asyncio.create_task(b.chat("wave", mode="local"))
        await asyncio.sleep(0.05)
        gate.set()
        return await asyncio.wait_for(a, 5.0), await asyncio.wait_for(q, 5.0)
    first, wave = run(scenario())
    assert [t["tool"] for t in first["trace"]] == ["stop"] and wave["trace"][0]["result"]["ok"]
    assert [x[0] for x in b.sim.started] == ["wave"] and b._model_stops == 1


@pytest.mark.parametrize("line, is_stop", [
    ("stop", True), ("Stop", True), ("STOP", True), ("stop now", True), ("  stop  ", True),
    ("teleop  ", True), ('tool stop {}', True),
    ("stopwatch", False), ("teleop w", False), ('tool goto {"x": 0.1}', False), ("walk 0.1", False)])
def test_is_stop_cmd_reads_the_console_as_playground_does(line, is_stop):
    assert cb._is_stop_cmd(line) is is_stop


@pytest.mark.parametrize("console", ["Stop", "STOP", "stop now"])
def test_a_console_stop_from_a_phone_keyboard_refuses_the_turns_motion(console):
    holder = {}

    def script(model, msgs, tools, mt):
        if not any(m["role"] == "tool" for m in msgs):
            holder["b"].sim.cmd_log.append((0.0, console))            # /api/cmd writes the raw line
            return {"content": "", "tool_calls": [call("gesture", {"name": "wave"}, "g")]}
        return answer("I was stopped.")
    b = _logged(script)
    holder["b"] = b
    r = run(b.chat("wave", mode="local"))
    assert r["trace"][0]["result"]["error"] == "operator_stopped" and b.sim.started == []


def test_a_stop_that_overtakes_a_turn_is_remembered_after_it():
    s = Script(tool_model=[{"content": "", "tool_calls": [call("goto", {"x": 0.3, "y": 0}, "a")]},
                           answer("Walking to the door."), answer("You stopped me at the door.")])
    b = _logged(s)
    _stop, _walk = _stop_mid_walk(b, "local", "walk to the door")
    assert [h["user"] for h in b.hist["local"]] == ["walk to the door", "stop"]
    assert [h["reply"] for h in b.hist["local"]] == ["Walking to the door.", cb.STOP_FIRST_REPLY]
    assert b.hist["local"][0]["tools"] == ["goto"] and "pending" not in b.hist["local"][0]
    run(b.chat("what happened?", mode="local"))                        # the next turn sees them in order
    users = [m["content"] for m in s.calls[-1][1] if m["role"] == "user"]
    assert users[:2] == ["walk to the door", "stop"]


def test_a_running_turn_does_not_see_its_own_pending_slot():
    seen = []

    def script(model, messages, tools, max_tokens):
        seen.append([m["content"] for m in messages if m["role"] == "user"])
        return answer("ok")
    b = brains(script)
    run(b.chat("first", mode="local"))
    run(b.chat("second", mode="local"))
    assert seen == [["first"], ["first", "second"]]


def test_a_turn_that_raises_leaves_no_history_slot(monkeypatch):
    b = brains(Script(tool_model=[answer("ok")]))

    async def boom(*a, **k):
        raise RuntimeError("the sim thread died")
    monkeypatch.setattr(b, "_llm", boom)
    with pytest.raises(RuntimeError):
        run(b.chat("walk", mode="local"))
    assert not b.hist.get("local")


@pytest.mark.parametrize("line, want", [
    ("stop", None), ("stop right now", None), ("Pebble, stop.", None), ("come to a full stop", None),
    ("it won't stop!", None), ("can you stop?", None), ("stop turning", None), ("I said stop", None),
    ("why did you stop?", "ask"), ("what does the stop button do?", "ask"), ("who told you to stop?", "ask"),
    ("remember that the stop button is red", "ask"), ("hey pebble, why did you stop", "ask"),
    ("walk forward 30 cm and then stop", "note"), ("find the ball and stop in front of it", "note"),
    ("stop at the door", "note"), ("walk to the box, stop there", "note")])
def test_stop_follow_up_is_pure(line, want):
    assert cb.stop_line(line)
    assert cb.stop_follow_up(line) == want
    assert cb.stop_follow_up("Buddy, stop", names=("Buddy",)) is None


@pytest.mark.parametrize("mode", ["local", "multimodal"])
def test_a_question_with_stop_in_it_stops_and_then_gets_an_answer(mode):
    s = Script(tool_model=[{"content": "", "tool_calls": [call("move", {"forward_m": 0.1}, "m")]},
                           answer("The void guard stopped me at the edge.")],
               gemma_4_26b_a4b=[{"content": "", "tool_calls": [call("move", {"forward_m": 0.1}, "m")]},
                                answer("The void guard stopped me at the edge.")])
    b = cb.Brains(PoseSim(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None,
                  complete=s, catalog=CAT)
    b.sim.cmd_log = []
    b.sim.note_cmd = lambda line: b.sim.cmd_log.append((0.0, line))
    r = run(b.chat("why did you stop?", mode=mode, source="typed"))
    assert b.sim.stops == 1 and b.sim.gotos == []                      # it stopped; the model's move did not run
    assert [(t["tool"], t["result"].get("error")) for t in r["trace"]] == [("stop", None), ("move", "operator_stopped")]
    assert r["reply"].startswith(cb.STOP_FIRST_REPLY + " ")            # (multimodal: + the chain's note)
    assert r["reply"].endswith("The void guard stopped me at the edge.")
    assert r["stop_first"] is True and r["model"] is not None
    sent = s.calls[0][1][-1]["content"]
    sent = sent if isinstance(sent, str) else sent[0]["text"]
    assert sent.startswith("why did you stop?") and cb.STOP_ASK_NOTE.strip() in sent
    assert [(h["user"], h["reply"]) for h in b.hist[mode]] == [("why did you stop?", r["reply"])]
    assert "[The cockpit" not in b.hist[mode][0]["user"]


def test_a_question_with_stop_in_it_reaches_claude_after_the_stop(monkeypatch):
    fake = _claude_ready(monkeypatch, [_text("I stopped because you asked.")])
    b = brains()
    r = run(b.chat("why did you stop?", mode="claude"))
    assert b.sim.stops == 1 and len(fake.calls) == 1
    assert r["reply"] == f"{cb.STOP_FIRST_REPLY} I stopped because you asked."
    first_user = fake.calls[0]["messages"][0]                           # (the list grows after the call)
    assert first_user["role"] == "user" and cb.STOP_ASK_NOTE.strip() in first_user["content"]
    assert b.hist["claude"][-1]["user"] == "why did you stop?"


def test_without_a_claude_key_a_question_with_stop_in_it_only_stops(monkeypatch):
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    b = brains()
    r = run(b.chat("why did you stop?", mode="claude"))
    assert b.sim.stops == 1 and r["reply"] == cb.STOP_FIRST_REPLY + cb.STOP_NO_CLAUDE_NOTE
    assert b.hist["claude"][-1]["reply"] == r["reply"]


@pytest.mark.parametrize("line", ["walk forward 30 cm and then stop", "find the ball and stop in front of it"])
def test_a_command_with_stop_in_it_only_stops_and_says_so(line):
    s = Script(tool_model=[answer("should never be asked")])
    b = brains(s)
    r = run(b.chat(line, mode="local"))
    assert b.sim.stops == 1 and s.calls == [] and b.sim.x == 0.0
    assert r["reply"] == cb.STOP_FIRST_REPLY + cb.STOP_REST_NOTE
    assert b.hist["local"][-1]["reply"] == r["reply"]


def test_a_failed_follow_up_chain_says_only_the_stop_ran():
    def down(model, messages, tools, max_tokens):
        raise RuntimeError("HTTP 503")
    b = brains(down)
    r = run(b.chat("why did you stop?", mode="local"))
    assert b.sim.stops == 1 and r["reply"].startswith(cb.STOP_FIRST_REPLY)
    assert "only the stop ran" in r["reply"] and b.sim.started == []


class _PoseReadStop(PoseSim):
    """A stop lands while the pose is read (it takes one sim tick in the cockpit)."""

    def __init__(self):
        super().__init__()
        self.cmd_log, self.hit = [], False

    def note_cmd(self, line):
        super().note_cmd(line)
        self.cmd_log.append((0.0, line))

    async def call(self, fn):
        if fn == self.pose and not self.hit:
            self.hit = True
            self.cmd_log.append((0.0, "teleop  "))                     # the STOP key, in that tick
        return fn()


def test_a_stop_during_moves_pose_read_is_not_lost():
    """Review repro: timeline ['stop', 'goto start'], trace [('move', 'arrived')]."""
    b = cb.Brains(_PoseReadStop(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None, catalog=CAT)
    r = run(b.tool("move", {"forward_m": 0.3}))
    assert r["error"] == "operator_stopped" and b.sim.gotos == []
    r = run(b.tool("move", {"forward_m": 0.3}))                        # the next move (no new stop) walks
    assert r["stopped"] == "arrived" and b.sim.gotos == [(0.3, 0.0)]


def test_a_stop_during_turns_pose_read_is_not_lost():
    b = cb.Brains(_PoseReadStop(), base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None, catalog=CAT)
    r = run(b.tool("turn", {"deg": 30}))
    assert r["error"] == "operator_stopped" and b.sim.started == []


_FAR = {"method": "bbox", "seen": True, "bearing_deg": 0.0, "distance_m": 1.0, "confidence": 0.9,
        "what": "ball", "parsed": True}
_UNSURE = dict(_FAR, confidence=0.2)


def _find_brains(detect, memory=None):
    sim = _PoseReadStop()
    sim.hit = True                                                     # no stop in the pose reads
    if memory is not None:
        sim.memory = memory
    b = cb.Brains(sim, base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None, catalog=CAT)
    b.frame_wait_s = 0.0

    async def fake_detect(name, model=None, method="bbox"):
        return {"ok": True, "model": "eye", "detection": dict(detect(sim)), "raw": "", "latency_s": 0.1}
    b.detect = fake_detect
    return b


@pytest.mark.parametrize("det", [_FAR, _UNSURE])
def test_a_stop_while_find_looks_ends_it_before_it_walks_or_turns(det):
    def detect(sim):
        sim.cmd_log.append((0.0, "stop"))                              # the console stop, while the eye looks
        return det
    b = _find_brains(detect)
    r = run(b.find_object("ball"))
    assert r["stopped"] == "user" and r["looks"] == 1
    assert b.sim.gotos == [] and b.sim.started == []


def test_a_world_change_while_find_looks_ends_it_and_remembers_nothing():
    import scene_memory as sm
    mem = sm.SceneMemory("flat")

    def detect(sim):
        mem.set_world("room")                                          # the cockpit's set_world ...
        mem.new_epoch(why="world 'room' loaded")                       # ... and its _memory_epoch
        return _FAR
    b = _find_brains(detect, mem)
    r = run(b.find_object("ball"))
    assert r["stopped"] == "preempted" and r["stale_scene"] is True and r["ok"] is False
    assert b.sim.gotos == [] and b.last_find is None
    assert mem.where_is("ball") is None and not any("ball" in o["text"] for o in mem.obs)


def test_find_in_an_unchanged_scene_still_remembers():
    import scene_memory as sm
    mem = sm.SceneMemory("flat")
    b = _find_brains(lambda sim: dict(_FAR, distance_m=0.2), mem)
    r = run(b.find_object("ball"))
    assert r["found"] and "stale_scene" not in r and b.last_find["found"]
    assert mem.where_is("ball") is not None


BALL_AHEAD = "An orange ball about 1 meter ahead on the floor."


def _look_brains(memory, before_answer=None):
    sim = FakeSim()
    sim.memory = memory
    b = brains()

    async def vision(prompt, model=None, max_tokens=None):
        if before_answer is not None:
            before_answer()
        return {"ok": True, "model": "eye", "text": BALL_AHEAD, "latency_s": 0.1}
    b.sim = sim
    b._vision = vision
    return b


def test_a_look_in_flight_across_a_world_change_is_not_remembered():
    """Review repro: 'room memory objects: [('ball', 'look'), ...]' — the old
    world's ball landed in the new world's scene memory."""
    import scene_memory as sm
    mem = sm.SceneMemory("flat")

    def world_change():
        mem.set_world("room")
        mem.new_epoch(why="world 'room' loaded")
    b = _look_brains(mem, world_change)
    r = run(b.look())
    assert r["ok"] and r["description"] == BALL_AHEAD and r["stale_scene"] is True
    assert "placed" not in r and b.last_look is None
    assert mem.world == "room" and mem.where_is("ball") is None


def test_talk_says_when_its_look_described_the_old_scene():
    import scene_memory as sm
    mem = sm.SceneMemory("flat")
    b = _look_brains(mem, lambda: mem.new_epoch(why="reset"))
    r = run(b.chat("what do you see", mode="talk"))
    assert r["trace"][-1]["tool"] == "look" and BALL_AHEAD in r["reply"]
    assert "while the eye looked" in r["reply"] and b.last_look is None


def test_a_look_in_flight_across_a_reset_is_not_remembered():
    import scene_memory as sm
    mem = sm.SceneMemory("flat")
    b = _look_brains(mem, lambda: mem.new_epoch(why="reset"))
    r = run(b.look())
    assert r["stale_scene"] is True and b.last_look is None and mem.where_is("ball") is None


def test_a_look_in_an_unchanged_scene_is_remembered():
    import scene_memory as sm
    mem = sm.SceneMemory("flat")
    b = _look_brains(mem)
    r = run(b.look())
    assert "stale_scene" not in r and r["placed"] == ["ball"] and b.last_look["text"] == BALL_AHEAD
    assert mem.where_is("ball") is not None


# ------------------------------------------------------------------ B41: per-family prompt notes + thinking
def test_family_note_and_thinking_switch(monkeypatch):
    monkeypatch.delenv("ROCKY_FAMILY_NOTES", raising=False)
    assert "call stop" in cb.family_note("gemma-4-12b") and cb.family_note("gemma-4-26b-a4b")
    assert "call stop" in cb.family_note("qwen3.5-9b") and "Gemma" not in cb.family_note("qwen3.5-9b")
    assert cb.family_note("qwen3.6-35b-a3b") == "" and cb.family_note(None) == ""
    monkeypatch.setenv("ROCKY_FAMILY_NOTES", "0")
    assert cb.family_note("gemma-4-12b") == ""
    monkeypatch.delenv("ROCKY_THINKING_MODELS", raising=False)
    assert not cb.thinking_for("gemma-4-12b")
    monkeypatch.setenv("ROCKY_THINKING_MODELS", "gemma-4-12b, qwen3.5-4b")
    assert cb.thinking_for("gemma-4-12b") and cb.thinking_for("qwen3.5-4b") and not cb.thinking_for("qwen3.5-9b")


def test_system_prompt_carries_the_family_note_per_model_in_the_chain(monkeypatch):
    monkeypatch.delenv("ROCKY_FAMILY_NOTES", raising=False)
    s = Script(gemma_4_26b_a4b=[RuntimeError("HTTP 503")], qwen3_5_9b=[answer("fine")])
    b = brains(s)
    b.state["multimodal_model"] = "gemma-4-26b-a4b"
    r = run(b.chat("how are you", mode="multimodal", source="typed"))
    assert r["model"] == "qwen3.5-9b"
    sys_by_model = {m: msgs[0]["content"] for m, msgs, *_ in s.calls}
    assert "Gemma:" in sys_by_model["gemma-4-26b-a4b"] and "Gemma:" not in sys_by_model["qwen3.5-9b"]
    assert b.system_for(model="tool-model") == b.system_for()
