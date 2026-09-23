#!/usr/bin/env python3
"""local_brain — a LOCAL LLM drives sim-Pebble through the harness tools.

The VISION_PLAN brain rehearsal, fully offline: an Ollama model gets the
same six tools the MCP server exposes (say / gesture / goto / stop /
scan_summary / status), you type to it, it decides which tools to call,
and the SimBackend runs them on real MuJoCo physics with the reflex
guards live. Swap the model, keep the robot — that's the architecture.

    # terminal 1 (once): install + pull a tool-calling model
    ollama pull qwen2.5:7b            # or llama3.1:8b, qwen3, etc.

    # terminal 2, from the repo root:
    python3 -m harness.local_brain                       # sim backend, qwen2.5:7b
    python3 -m harness.local_brain --model llama3.1:8b
    python3 -m harness.local_brain --backend mock        # no MuJoCo needed
    python3 -m harness.local_brain --mock-llm            # scripted fake LLM
                                                         # (tests the loop, no Ollama)

Talk to it: "say hi and do jazz hands", "walk to (0.4, 0)", "what's
around you?", "stop!". `quit` exits. Guard supremacy is untouched: a
goto that meets the cliff detector still comes back stopped:"cliff"
no matter what the model wanted.

Requires: `pip install ollama` and a running Ollama daemon (except
--mock-llm). Tool-calling quality varies by model; qwen2.5 7B+ and
llama3.1 8B+ behave well.

OpenAI-compatible servers (2026-09-08, laptop addition): llama-swap,
llama.cpp's llama-server, vLLM, LM Studio… anything speaking
/v1/chat/completions with tools. `pip install openai`, then

    python3 -m harness.local_brain --base-url http://127.0.0.1:8080/v1 \
        --model qwen3.6-35b-a3b            # ROCKY_LLM_API_KEY if it needs one

(or ROCKY_LLM_BASE_URL / ROCKY_LLM_MODEL in the environment). Same tool
loop, same guards; thinking is switched off for snappy tool calls unless
you pass --think.
"""
from __future__ import annotations
import argparse
import asyncio
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

SYSTEM = """You are the brain of Pebble, a five-legged rock robot (a subscale
prototype of ROCKY). You act ONLY by calling tools. You cannot speak text to
the human directly through the robot: the robot's only voice is chord-speak
words via the say tool. Keep tool use purposeful: one gesture at a time, stop
before gesturing if walking. Positions are METERS in the map frame;
the robot starts at (0, 0) — it walks ~0.045 m/s, so keep targets within a
meter or two. After acting, reply with ONE short sentence
summarizing what you did (this prints to the operator console, not the robot).
Available chord-speak words include: greeting, acknowledge, curious_question,
amaze, found_it, thinking, confused, determined, error, alarm_help, sleepy,
discovery, low_battery, startup."""

TOOLS = [
    {"type": "function", "function": {
        "name": "say",
        "description": "Speak one chord-speak word through the speakers.",
        "parameters": {"type": "object", "properties": {
            "word": {"type": "string", "description": "lexicon word, e.g. greeting"}},
            "required": ["word"]}}},
    {"type": "function", "function": {
        "name": "gesture",
        "description": "Perform a gesture: jazz_hands, fist_bump, beckon, wave, "
                       "bow, look_around, shake, sit, turn_in_place, sidestep.",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string"}}, "required": ["name"]}}},
    {"type": "function", "function": {
        "name": "goto",
        "description": "Walk to (x, y) in METERS, map frame. Blocks until arrival "
                       "or a guard stop (cliff!). Returns how it ended.",
        "parameters": {"type": "object", "properties": {
            "x": {"type": "number"}, "y": {"type": "number"}},
            "required": ["x", "y"]}}},
    {"type": "function", "function": {
        "name": "stop",
        "description": "Safe-stop immediately (PLANT->BRACE).",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "scan_summary",
        "description": "What the lidar sees: nearest obstacles by direction.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "status",
        "description": "Robot status: pose, mode, health.",
        "parameters": {"type": "object", "properties": {}}}},
]


def make_backend(kind):
    if kind == "sim":
        from harness.sim_backend import SimBackend
        return SimBackend()
    from harness.backend import MockBackend
    return MockBackend()


async def call_tool(backend, name, args):
    fn = getattr(backend, name, None)
    if fn is None:
        return {"ok": False, "error": f"no such tool {name}"}
    try:
        return await fn(**args)
    except TypeError as e:
        return {"ok": False, "error": f"bad arguments for {name}: {e}"}


class MockLLM:
    """Scripted stand-in so the tool loop is testable without Ollama:
    greets, looks, walks a bit, reports. One canned turn per user input."""

    def __init__(self):
        self.turn = 0

    def chat(self, model, messages, tools):
        self.turn += 1
        script = [
            [("say", {"word": "greeting"}), ("gesture", {"name": "wave"})],
            [("scan_summary", {}), ("status", {})],
            [("goto", {"x": -0.30, "y": 0.10})],   # meters; away from the cliff world's void (+x)
        ]
        calls = script[(self.turn - 1) % len(script)]
        return {"message": {
            "content": f"(mock turn {self.turn})",
            "tool_calls": [{"function": {"name": n, "arguments": a}}
                           for n, a in calls]}}


class OpenAIChat:
    """Any OpenAI-compatible /v1/chat/completions server (llama-swap,
    llama-server, vLLM, LM Studio…) behind the same .chat(model, messages,
    tools) surface as the ollama client, so the tool loop below does not
    care which brain it got. The loop keeps an ollama-shaped transcript;
    this converts it on the way out and hands back an ollama-shaped reply."""

    def __init__(self, base_url, api_key, think=False):
        from openai import OpenAI
        self.client = OpenAI(base_url=base_url, api_key=api_key or "none")
        # Qwen3/GLM-style templates: thinking off = one short hop per tool
        self.extra = None if think else {
            "chat_template_kwargs": {"enable_thinking": False}}

    @staticmethod
    def _to_openai(messages):
        out = []
        for m in messages:
            if m["role"] == "assistant":
                mm = {"role": "assistant", "content": m.get("content") or ""}
                tcs = [{"id": tc.get("id") or f"call_{i}", "type": "function",
                        "function": {
                            "name": tc["function"]["name"],
                            "arguments": json.dumps(tc["function"]["arguments"])}}
                       for i, tc in enumerate(m.get("tool_calls") or [])]
                if tcs:
                    mm["tool_calls"] = tcs
                out.append(mm)
            elif m["role"] == "tool":
                out.append({"role": "tool",
                            "tool_call_id": m.get("tool_call_id") or "call_0",
                            "content": m["content"]})
            else:
                out.append({"role": m["role"], "content": m["content"]})
        return out

    def chat(self, model, messages, tools):
        r = self.client.chat.completions.create(
            model=model, messages=self._to_openai(messages), tools=tools,
            extra_body=self.extra)
        msg = r.choices[0].message
        calls = []
        for tc in msg.tool_calls or []:
            try:
                targs = json.loads(tc.function.arguments or "{}")
            except json.JSONDecodeError:
                targs = {}
            calls.append({"id": tc.id, "function": {"name": tc.function.name,
                                                    "arguments": targs}})
        return {"message": {"content": msg.content or "", "tool_calls": calls}}


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.environ.get("ROCKY_LLM_MODEL",
                                                      "qwen2.5:7b"))
    ap.add_argument("--backend", default="sim", choices=["sim", "mock"])
    ap.add_argument("--mock-llm", action="store_true",
                    help="scripted fake LLM — tests the loop without Ollama")
    ap.add_argument("--once", default=None,
                    help="one prompt, then exit (for scripting/tests)")
    ap.add_argument("--base-url", default=os.environ.get("ROCKY_LLM_BASE_URL"),
                    help="OpenAI-compatible endpoint instead of Ollama, e.g. "
                         "http://127.0.0.1:8080/v1 (llama-swap/llama-server/"
                         "vLLM/LM Studio); key via ROCKY_LLM_API_KEY")
    ap.add_argument("--think", action="store_true",
                    help="(--base-url) leave the model's thinking mode on")
    args = ap.parse_args()

    if args.mock_llm:
        client = MockLLM()
    elif args.base_url:
        try:
            client = OpenAIChat(args.base_url, os.environ.get("ROCKY_LLM_API_KEY"),
                                think=args.think)
        except ImportError:
            sys.exit("pip install openai for --base-url, or use --mock-llm")
    else:
        try:
            import ollama
            client = ollama
        except ImportError:
            sys.exit("pip install ollama (and run the Ollama daemon), or use --mock-llm")

    backend = make_backend(args.backend)
    messages = [{"role": "system", "content": SYSTEM}]
    print(f"local brain: model={'MOCK' if args.mock_llm else args.model} "
          f"backend={args.backend}. Talk to Pebble ('quit' exits).")

    while True:
        try:
            user = args.once if args.once else input("you> ")
        except EOFError:
            break
        if not user or user.strip() == "quit":
            break
        messages.append({"role": "user", "content": user})
        # tool loop: let the model call tools until it stops asking to
        for _hop in range(6):
            resp = client.chat(model=args.model, messages=messages, tools=TOOLS)
            msg = resp["message"] if isinstance(resp, dict) else resp.message
            content = msg.get("content") if isinstance(msg, dict) else msg.content
            tool_calls = (msg.get("tool_calls") if isinstance(msg, dict)
                          else getattr(msg, "tool_calls", None)) or []
            messages.append({"role": "assistant", "content": content or "",
                             "tool_calls": tool_calls})
            if not tool_calls:
                if content:
                    print(f"pebble-brain> {content}")
                break
            for tc in tool_calls:
                f = tc["function"] if isinstance(tc, dict) else tc.function
                name = f["name"] if isinstance(f, dict) else f.name
                raw = f["arguments"] if isinstance(f, dict) else f.arguments
                targs = raw if isinstance(raw, dict) else json.loads(raw or "{}")
                tc_id = tc.get("id") if isinstance(tc, dict) else getattr(tc, "id", None)
                print(f"  [tool] {name}({json.dumps(targs)})")
                result = await call_tool(backend, name, targs)
                print(f"         -> {json.dumps(result)[:160]}")
                messages.append({"role": "tool", "name": name,
                                 "tool_call_id": tc_id,
                                 "content": json.dumps(result)})
        if args.once:
            break
    print("bye")


if __name__ == "__main__":
    asyncio.run(main())
