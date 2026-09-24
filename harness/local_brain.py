#!/usr/bin/env python3
"""local_brain — a LOCAL LLM drives sim-Pebble through the harness tools.

The VISION_PLAN brain rehearsal, fully offline: an Ollama model gets the
same tools the MCP server exposes (say / gesture / goto / stop /
scan_summary / status / list_gestures), you type to it, it decides which tools to call,
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

D052 reliability rules (shared with sim/cockpit_brains.py): every tool call
gets a tool result even when it fails (an orphaned tool_call 400s every
later turn); a tool exception is a result, not a crash; NaN arguments are
refused; at most MAX_HOPS tool rounds per turn, then a plain "gave up";
the transcript keeps the last MAX_TURNS turns; model calls time out at
90 s (the SDK default of 600 s hung the cockpit through a model swap).
"""
from __future__ import annotations
import argparse
import asyncio
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

# Everything a goto can come back with (sim/cockpit.py _goto_post + harness
# backends). The model is told all of them so a veto is read as a result,
# not retried: D052 saw a local model re-issue the same goto after "stuck".
GOTO_OUTCOMES = ("arrived", "cliff", "stuck", "blocked", "timeout", "user",
                 "preempted", "FELL")
GOTO_DOC = ("Walk to (x, y) in METERS, map frame (the robot starts at (0, 0) "
            "facing +x; +y is its left). Blocks until it ends and returns "
            "stopped= arrived | cliff (the void reflex vetoed it: do NOT retry "
            "toward it) | stuck (no progress: something is in the way — look or "
            "scan_summary, then pick a different target; never re-send the same "
            "one) | blocked (a guard refused to start: read detail) | timeout "
            "(too far: ~0.045 m/s, 40 s cap, keep targets within ~1.5 m) | user "
            "(stop was called) | preempted (a newer goto took over) | FELL. "
            "A veto is a NORMAL result: report it.")


def build_tools(gestures=None, lexicon=None, signed=("turn_in_place", "sidestep"),
                look=False, extra=()):
    """The OpenAI tool list with the CURRENT gesture names and chord words as
    enums (D052: a free-text 'name' let models invent gestures like 'dance'
    and burn a hop on the refusal). Built per request by the cockpit."""
    gestures = list(gestures or [])
    lexicon = list(lexicon or [])
    word = {"type": "string", "description": "a chord-speak word"}
    if lexicon:
        word["enum"] = lexicon
    gname = {"type": "string", "description": "gesture name"}
    if gestures:
        gname["enum"] = gestures
    tools = [
        {"type": "function", "function": {
            "name": "say",
            "description": "Speak one chord-speak word through the speakers (the robot's "
                           "only voice — it never speaks human words).",
            "parameters": {"type": "object", "properties": {"word": word},
                           "required": ["word"]}}},
        {"type": "function", "function": {
            "name": "gesture",
            "description": "Perform one gesture and wait for it to finish. Refused with "
                           "error='busy' while walking: stop first. direction left|right "
                           f"applies to {' and '.join(signed)} only (turn_in_place turns "
                           "on the spot for ~5 s; left = counter-clockwise).",
            "parameters": {"type": "object", "properties": {
                "name": gname,
                "direction": {"type": "string", "enum": ["left", "right"]}},
                "required": ["name"]}}},
        {"type": "function", "function": {
            "name": "goto", "description": GOTO_DOC,
            "parameters": {"type": "object", "properties": {
                "x": {"type": "number"}, "y": {"type": "number"}},
                "required": ["x", "y"]}}},
        {"type": "function", "function": {
            "name": "stop",
            "description": "Safe-stop immediately (PLANT->BRACE). Always accepted.",
            "parameters": {"type": "object", "properties": {}}}},
        {"type": "function", "function": {
            "name": "scan_summary",
            "description": "What the lidar sees: nearest obstacles by direction (walls "
                           "and objects yes; holes in the floor no).",
            "parameters": {"type": "object", "properties": {}}}},
        {"type": "function", "function": {
            "name": "status",
            "description": "Robot status: pose (x, y m; yaw deg), mode, reflex state.",
            "parameters": {"type": "object", "properties": {}}}},
        {"type": "function", "function": {
            "name": "list_gestures",
            "description": "The gestures this robot knows right now (saved ones included).",
            "parameters": {"type": "object", "properties": {}}}},
    ]
    if look:
        tools.append({"type": "function", "function": {
            "name": "look",
            "description": "Look through the robot's eye camera: a vision model describes "
                           "what is in front of the robot (obstacles, open floor, objects).",
            "parameters": {"type": "object", "properties": {}}}})
    return tools + list(extra)


def build_system(gestures=None, lexicon=None, extra=""):
    """The brain's standing orders, with the live lexicon and gesture list."""
    return (
        "You are the brain of Pebble, a five-legged rock robot (a 1:3 prototype of "
        "ROCKY). You act ONLY by calling tools. The robot's only voice is chord-speak "
        "words via `say`; your text reply goes to the operator's console.\n"
        "Rules:\n"
        "- Do what the operator asked and nothing more. One motion at a time: call a "
        "gesture or a goto, read its result, then decide the next step. Never call a "
        "motion tool twice in one reply.\n"
        "- Positions are METERS in the map frame; the robot starts at (0, 0) facing +x, "
        "+y is its left. For a relative move ('forward 30 cm') call status first: forward is the "
        "robot heading yaw_deg, so x += d*cos(yaw), y += d*sin(yaw). "
        "It walks ~0.045 m/s: keep targets within ~1.5 m.\n"
        "- goto ends as arrived | cliff | stuck | blocked | timeout | user | preempted | "
        "FELL. cliff/stuck/blocked are vetoes: report them; do not retry the same target.\n"
        "- 'turn left/right' = gesture turn_in_place with direction; a sidestep is gesture "
        "sidestep with direction.\n"
        "- If a tool returns ok=false, say so plainly in your reply.\n"
        f"Chord-speak words: {', '.join(lexicon or []) or '(see say)'}.\n"
        f"Gestures: {', '.join(gestures or []) or '(call list_gestures)'}.\n"
        + (extra + "\n" if extra else "")
        + "After acting, reply with ONE short sentence saying what happened.")


MAX_HOPS = 6              # tool rounds per operator turn before we give up and say so
MAX_TURNS = 12            # operator turns kept in the transcript (older: user + reply only)
CALL_TIMEOUT_S = 90.0     # the SDK default is 600 s: a model swap hung the UI for minutes


def parse_args_json(raw):
    """Tool-call arguments -> (dict, None) or (None, error). NaN/Infinity are
    refused (json.loads accepts them; goto(NaN, 0) poisoned cmd_v)."""
    if isinstance(raw, dict):
        return raw, None
    def _no_const(c):
        raise ValueError(f"non-finite number {c}")
    try:
        args = json.loads(raw or "{}", parse_constant=_no_const)
    except ValueError as e:
        return None, f"arguments are not valid JSON: {e}"
    if not isinstance(args, dict):
        return None, "arguments must be a JSON object"
    return args, None


def trim_history(messages, max_turns=MAX_TURNS):
    """system + the last max_turns operator turns. Older turns keep the user
    line and the assistant's final sentence only: their tool calls and tool
    results go (an orphaned tool_call with no result 400s every later turn)."""
    sys_msgs = [m for m in messages[:1] if m.get("role") == "system"]
    body = messages[len(sys_msgs):]
    starts = [i for i, m in enumerate(body) if m.get("role") == "user"]
    if len(starts) <= max_turns:
        return messages
    keep_from = starts[-max_turns]
    old, recent = body[:keep_from], body[keep_from:]
    squeezed = []
    for m in old:
        if m.get("role") == "user":
            squeezed.append({"role": "user", "content": m.get("content") if isinstance(
                m.get("content"), str) else "(earlier message)"})
        elif m.get("role") == "assistant" and not m.get("tool_calls") and m.get("content"):
            squeezed.append({"role": "assistant", "content": m["content"]})
    return sys_msgs + squeezed[-2 * max_turns:] + recent


from harness.backend import GESTURES as _G, CHORD_WORDS as _W   # noqa: E402
SYSTEM = build_system(_G, _W)
TOOLS = build_tools(_G, _W)


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
    except Exception as e:                      # a tool failure is a RESULT the model reads
        return {"ok": False, "error": f"{name} failed: {type(e).__name__}: {e}"}


class MockLLM:
    """Scripted stand-in so the tool loop is testable without Ollama:
    greets, looks, walks a bit, reports. One canned turn per user input."""

    def __init__(self):
        self.turn = 0

    def chat(self, model, messages, tools):
        if messages and messages[-1].get("role") == "tool":     # results are in: summarize
            return {"message": {"content": f"(mock turn {self.turn}: done)", "tool_calls": []}}
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
        self.client = OpenAI(base_url=base_url, api_key=api_key or "none",
                             timeout=CALL_TIMEOUT_S, max_retries=0)
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
            targs, err = parse_args_json(tc.function.arguments)
            calls.append({"id": tc.id, "function": {"name": tc.function.name,
                                                    "arguments": targs if err is None else {},
                                                    "error": err}})
        return {"message": {"content": msg.content or "", "tool_calls": calls}}


async def run_turn(client, model, messages, backend, tools=None, log=print):
    """One operator turn (the user message is already appended): let the model
    call tools until it answers, at most MAX_HOPS rounds. Every tool_call gets
    a tool result — even a malformed or failing one — so the transcript stays
    valid for the next turn. Returns the reply text (never empty)."""
    tools = TOOLS if tools is None else tools
    for _hop in range(MAX_HOPS):
        try:
            resp = await asyncio.wait_for(asyncio.to_thread(
                client.chat, model=model, messages=messages, tools=tools), CALL_TIMEOUT_S + 5)
        except Exception as e:
            return f"(model {model} failed: {type(e).__name__}: {e})"
        msg = resp["message"] if isinstance(resp, dict) else resp.message
        content = msg.get("content") if isinstance(msg, dict) else msg.content
        tool_calls = (msg.get("tool_calls") if isinstance(msg, dict)
                      else getattr(msg, "tool_calls", None)) or []
        messages.append({"role": "assistant", "content": content or "",
                         "tool_calls": tool_calls})
        if not tool_calls:
            return (content or "").strip() or "(the model answered with no text)"
        for i, tc in enumerate(tool_calls):
            f = tc["function"] if isinstance(tc, dict) else tc.function
            name = f["name"] if isinstance(f, dict) else f.name
            raw = f["arguments"] if isinstance(f, dict) else f.arguments
            err = f.get("error") if isinstance(f, dict) else None
            targs, perr = parse_args_json(raw)
            err = err or perr
            tc_id = (tc.get("id") if isinstance(tc, dict) else getattr(tc, "id", None)) or f"call_{i}"
            if err:
                result = {"ok": False, "error": f"bad arguments for {name}: {err}"}
            else:
                log(f"  [tool] {name}({json.dumps(targs)})")
                result = await call_tool(backend, name, targs)
            log(f"         -> {json.dumps(result)[:160]}")
            messages.append({"role": "tool", "name": name, "tool_call_id": tc_id,
                             "content": json.dumps(result)[:4000]})
    return f"(gave up after {MAX_HOPS} tool rounds without a final answer)"


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
        messages[:] = trim_history(messages)
        reply = await run_turn(client, args.model, messages, backend)
        print(f"pebble-brain> {reply}")
        if args.once:
            break
    print("bye")


if __name__ == "__main__":
    asyncio.run(main())
