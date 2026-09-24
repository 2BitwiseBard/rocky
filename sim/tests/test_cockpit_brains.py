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
    s = Script(gemma_4_26b_a4b=[RuntimeError("HTTP 503")], tool_model=[answer("text brain here")])
    b = brains(s)
    r = run(b.chat("what do you see?", mode="multimodal"))
    first = s.calls[0][1][-1]["content"]
    assert s.calls[0][0] == "gemma-4-26b-a4b"
    assert isinstance(first, list) and first[1]["type"] == "image_url"
    fallback_user = s.calls[1][1][-1]["content"]
    assert isinstance(fallback_user, str) and "call look" in fallback_user
    assert r["model"] == "tool-model"


def test_look_skips_empty_descriptions_and_strips_thinking():
    s = Script(vision_model=[answer("<think>budget gone</think>")],
               gemma_4_26b_a4b=[answer("A red box half a meter ahead; open floor to the left.")])
    b = brains(s)
    r = run(b.look())
    assert r["ok"] and r["model"] == "gemma-4-26b-a4b" and "empty" in r["fallback"][0]
    b = brains(Script(vision_model=[answer("")], gemma_4_26b_a4b=[answer("  ")]))
    r = run(b.look())
    assert r["ok"] is False and len(r["tried"]) == 2
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
