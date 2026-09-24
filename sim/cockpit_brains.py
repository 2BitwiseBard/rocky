"""cockpit_brains (D052) — the cockpit's brains, out of cockpit.py.

The owner asked for "multiple options for AI models — one for vision, or
one just multimodal, a small or big one" and for commands that are
followed reliably. This module holds all of it; sim/cockpit.py keeps thin
wrappers (chat / tool / tool_look / llm_models / the /api/chat, /api/brain,
/api/models, /api/look, /api/voice routes).

ROLES (state keys in brackets, persisted to ~/.config/rocky/cockpit.json):
  brain      [model]            text + tools   default tool-model -> qwen3.6-35b-a3b
  vision     [vision_model]     describes the eye   vision-model -> lfm2.5-vl
  multimodal [multimodal_model] ONE model sees the eye frame AND calls tools
                                (brain mode 'multimodal'; off by default)
                                                     gemma-4-26b-a4b
  claude     [claude_model]     brain mode 'claude' (needs ANTHROPIC_API_KEY)
  stt        [stt_model]        whisper-server :8082
Why that default pair: on this 16 GB card only qwen3.6-35b-a3b (10.4 G at
-ncmoe 26) and lfm2.5-vl (4.6 G) co-reside in llama-swap's `resident`
group; any other brain + vision pair pays a model swap on EVERY look.

MODES: talk (regex, harness/intent.py — no model) | local (brain + look
tool) | multimodal (the eye JPEG rides in the user turn) | claude.

FALLBACK: brain qwen3.6-35b-a3b -> qwen3.8-27b-iq4 -> talk; vision
lfm2.5-vl -> gemma-4-26b-a4b; multimodal -> gemma -> the text brain chain
(images stripped). A chain advances on timeout / HTTP error / empty answer
and the reply SAYS which model answered and why the earlier ones did not.
Quarantined models (gpt-oss-20b, glm-flash-reap: GPU faults) are never
selected, whatever the UI sends.

RELIABILITY (the D052 findings this fixes):
  * a tool that raises is a RESULT {"ok": false, "error": ...}, never a 500;
  * every tool_call gets a tool result, even malformed/failing ones (an
    orphaned tool_call 400'd every later turn); NaN arguments are refused;
  * per-mode asyncio.Lock: a typed and a spoken message no longer interleave
    one transcript;
  * the transcript is system + the last 12 turns; older turns keep the user
    line and the final sentence only (no tool traffic, no images);
  * at most 6 tool rounds, then an explicit "gave up";
  * every model call: SDK timeout 90 s, max_retries 0, and asyncio.wait_for
    (the SDK default is 600 s — a model swap froze the chat panel);
  * thinking off (chat_template_kwargs enable_thinking=false) for the brain
    and for look: qwen3.8-27b defaults to xhigh reasoning and spent the whole
    160-token look budget thinking -> an empty description (now ok=false and
    the chain moves on);
  * vision detection from llama-swap's own metadata (name/description say
    VISION (mmproj) / Multimodal / Text-only), not from the id: the old
    'gemma' substring rule offered gemma-4-31b (text-only: an image 500s)
    and missed qwen3.8-27b.

VOICE: whisper verbose_json, per-segment no_speech_prob / avg_logprob
filter + a hallucination list (harness.intent.clean_transcript); a spoken
line may not MOVE the robot without the wake word ("pebble, ...") unless
the operator confirms it (trusted=true, i.e. pressed Enter on it).
"""
from __future__ import annotations

import asyncio
import base64
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (ROOT, os.path.join(ROOT, "gait")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from harness.intent import (plan as intent_plan, execute as intent_execute,        # noqa: E402
                            has_wake_word, moves, clean_transcript, MOTION_TOOLS)
from harness.local_brain import (build_tools, build_system, parse_args_json,          # noqa: E402
                                 MAX_HOPS, MAX_TURNS, CALL_TIMEOUT_S)
from harness.backend import SIGNED                                                    # noqa: E402

DEFAULT_BASE = "http://127.0.0.1:8080/v1"
WHISPER_URL = os.environ.get("ROCKY_WHISPER_URL", "http://127.0.0.1:8082")
WHISPER_PROMPT = ("Pebble, walk forward thirty centimeters. Pebble, stop. Wave. Bow. Sit. Shake. Turn left. Turn right. What do you see? Say hello.")   # vocabulary hint for the small model
MIN_VOICE_BYTES = 1200      # a webm/opus container with no audio is ~200-900 bytes
MIN_VOICE_S = 0.4           # shorter than a word: the hold was lost, not the speech
CONF_PATH = os.environ.get("ROCKY_COCKPIT_CONF", os.path.expanduser("~/.config/rocky/cockpit.json"))
MODES = ("talk", "local", "multimodal", "claude")
QUARANTINED = frozenset({"gpt-oss-20b", "glm-flash-reap"})   # GPU faults (workstation CLAUDE.md)
SKIP_PREFIXES = ("embedding", "reranker", "lab")              # not chat models
ROLE_KEYS = {"brain": "model", "vision": "vision_model", "multimodal": "multimodal_model",
             "claude": "claude_model", "stt": "stt_model"}
ROLE_DEFAULTS = {"model": "tool-model", "vision_model": "vision-model",
                 "multimodal_model": "gemma-4-26b-a4b", "claude_model": "claude-sonnet-5",
                 "stt_model": "whisper"}
FALLBACK = {"brain": ["qwen3.6-35b-a3b", "qwen3.8-27b-iq4"],
            "vision": ["lfm2.5-vl", "gemma-4-26b-a4b"],
            "multimodal": ["gemma-4-26b-a4b"]}
GOTO_MAX_M = 3.0          # a goto target farther than this from the robot is an argument error
LOOK_MAX_TOKENS = 160
REPLY_MAX_TOKENS = 512
CATALOG_TTL_S = 20.0      # llama-swap's model list (loaded/cold changes as models swap)
VOICE_RECENT_S = 60.0     # a chat text equal to the last transcript within this is voice
MAX_TEXT = 2000
RESULT_CHARS = 4000       # a tool result is cut to this in the transcript

_VISION_RE = re.compile(r"vision|multimodal|mmproj|\bvlm?\b", re.I)
_TEXT_ONLY_RE = re.compile(r"text[\s-]*only", re.I)
_THINK_RE = re.compile(r"<think>.*?(</think>|$)", re.S)

VISION_PROMPT = ("You are the eye of a small five-legged robot walking on a floor. Describe "
                 "what is in front of it in two short sentences: obstacles or objects, roughly "
                 "how far (near = within a few body lengths), and where the open floor is. "
                 "If the view is mostly floor, say so.")
LOOK_NOTE = ("You also have `look`: the robot's eye camera described by a vision model. Use it "
             "when asked what you see, before walking toward something, or after a goto came "
             "back stuck or blocked.")
MULTIMODAL_NOTE = ("The operator's message may carry the robot's CURRENT eye-camera image: answer "
                   "'what do you see' from it directly; call look only for a fresh view after "
                   "the robot has moved.")
COMPOSE_NOTE = ("To create a new gesture from a description, call compose_gesture with keyframes; "
                "it is checked against the real servo limits and previewed once. Nothing is kept "
                "until the operator says save (then call save_gesture). If the check fails, fix "
                "the FAIL lines and compose again.")

COMPOSE_DOC = (
    "Create a NEW gesture from keyframes (the owner describes it; you write the frames). It is "
    "checked against the robot's real limits (joint range, servo speed, balance, self-contact); "
    "when feasible it is previewed once on the robot and kept as a draft until save_gesture. "
    "Frame fields (all optional except t; missing = standing): t seconds from start (first frame "
    "{\"t\": 0} = standing); body [dx, dy, dz] mm body shift, feet planted (z < 0 crouches; "
    "+-30 mm is a lot); yaw deg body twist (+-15); dz [5] mm extra crouch per leg; arm "
    "{\"leg\": [yaw, hip, knee] deg} lifts that leg as an arm (yaw -40..40, hip -70..90 (up), "
    "knee -150..-20); claw [5] 0..1 open; say a chord word cue; ease smooth|linear|hold. Legs: "
    "0 = left (+y), then counter-clockwise: 1 rear-left, 2 rear-right, 3 front-right, 4 "
    "front-left. Keep >= 4 feet down (raise ONE leg; shift the body away from it first, "
    "body [0, -15, 0] for leg 0), give each move >= 0.6 s. Example wave with leg 0: "
    "[{\"t\":0},{\"t\":1.0,\"body\":[0,-15,0]},{\"t\":1.8,\"body\":[0,-15,0],\"arm\":{\"0\":[0,70,-60]}},"
    "{\"t\":2.6,\"body\":[0,-15,0],\"arm\":{\"0\":[25,70,-60]}},{\"t\":3.4,\"body\":[0,-15,0],"
    "\"arm\":{\"0\":[-25,70,-60]}},{\"t\":4.2,\"body\":[0,-15,0],\"arm\":{\"0\":[0,70,-60]}},"
    "{\"t\":5.0,\"body\":[0,-15,0]},{\"t\":5.8}]")
_FRAME_SCHEMA = {"type": "object", "properties": {
    "t": {"type": "number"},
    "body": {"type": "array", "items": {"type": "number"}},
    "yaw": {"type": "number"},
    "dz": {"type": "array", "items": {"type": "number"}},
    "arm": {"type": "object"},
    "claw": {"type": "array", "items": {"type": "number"}},
    "say": {"type": "string"},
    "ease": {"type": "string", "enum": ["smooth", "linear", "hold"]}},
    "required": ["t"]}
EXTRA_TOOLS = [
    {"type": "function", "function": {
        "name": "compose_gesture", "description": COMPOSE_DOC,
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "short snake_case name"},
            "description": {"type": "string", "description": "what the gesture should look like"},
            "keyframes": {"type": "array", "items": _FRAME_SCHEMA},
            "loop": {"type": "boolean"}},
            "required": ["name", "keyframes"]}}},
    {"type": "function", "function": {
        "name": "check_gesture",
        "description": "Check a keyframe gesture spec {name, keyframes, loop?, end?} against the "
                       "robot's limits WITHOUT moving; returns the report lines.",
        "parameters": {"type": "object", "properties": {"spec": {"type": "object"}},
                       "required": ["spec"]}}},
    {"type": "function", "function": {
        "name": "save_gesture",
        "description": "Save the last composed (previewed) gesture so it can be played by name. "
                       "Only when the operator asked to keep it.",
        "parameters": {"type": "object", "properties": {
            "name": {"type": "string", "description": "optional new name"},
            "overwrite": {"type": "boolean"}}}}},
]
TOOL_NAMES = ("say", "gesture", "goto", "stop", "scan_summary", "status", "look",
              "list_gestures", "compose_gesture", "check_gesture", "save_gesture")
NOTED = ("goto", "gesture", "say", "stop", "compose_gesture")     # recordings replay these

# the static defaults (the cockpit builds both per request with the live lists)
TOOLS = build_tools(look=True, extra=EXTRA_TOOLS)
SYSTEM = build_system(extra=LOOK_NOTE)


class ModelFailure(Exception):
    """A model call that did not produce an answer (timeout, HTTP error, empty)."""


# ------------------------------------------------------------------ pure helpers
def local_ai_key():
    for k in ("ROCKY_LLM_API_KEY", "LOCAL_AI_KEY"):
        if os.environ.get(k):
            return os.environ[k]
    conf = os.path.expanduser("~/.config/environment.d/local-ai.conf")
    if os.path.exists(conf):
        for line in open(conf):
            if line.startswith("LOCAL_AI_KEY="):
                return line.split("=", 1)[1].strip().strip('"').strip("'")
    return "none"


def classify_models(entries):
    """llama-swap /v1/models `data` -> [{id, name, vision, loaded, selector,
    targets, strategy, quarantined}] for chat models. vision iff the id, name
    or description says vision / multimodal / mmproj / VL and not text-only;
    a selector inherits vision and loaded from its targets."""
    out = []
    for e in entries or []:
        mid = str(e.get("id", ""))
        if not mid or mid.startswith(SKIP_PREFIXES):
            continue
        meta = (e.get("meta") or {}).get("llamaswap") or {}
        text = " ".join([mid, str(e.get("name") or ""), str(e.get("description") or "")])
        out.append(dict(
            id=mid, name=str(e.get("name") or mid),
            vision=bool(_VISION_RE.search(text)) and not _TEXT_ONLY_RE.search(text),
            loaded=(e.get("status") or {}).get("value") == "loaded",
            selector=meta.get("type") == "selector",
            targets=list(meta.get("targets") or []), strategy=meta.get("strategy"),
            quarantined=mid in QUARANTINED or "QUARANTIN" in text.upper()))
    by = {m["id"]: m for m in out}
    for m in out:
        tg = [by[t] for t in m["targets"] if t in by]
        if m["selector"] and tg:
            m["vision"] = any(t["vision"] for t in tg)
            m["loaded"] = any(t["loaded"] for t in tg)
            m["quarantined"] = all(t["quarantined"] for t in tg)
    return out


def validate_goto(x, y):
    """(tx, ty, None) or (None, None, error): numbers in meters, finite.
    goto(x='here') used to raise in the sim thread and 500 /api/chat;
    goto(NaN, 0) got through json.loads and poisoned cmd_v."""
    if isinstance(x, bool) or isinstance(y, bool):
        return None, None, "goto needs x and y in meters (numbers), got a boolean"
    try:
        tx, ty = float(x), float(y)
    except (TypeError, ValueError):
        return None, None, f"goto needs x and y in meters (numbers), got x={x!r}, y={y!r}"
    if not (math.isfinite(tx) and math.isfinite(ty)):
        return None, None, f"goto needs finite numbers, got x={x!r}, y={y!r}"
    return tx, ty, None


def goto_range_error(tx, ty, pose, max_m=GOTO_MAX_M):
    d = math.hypot(tx - float(pose["x"]), ty - float(pose["y"]))
    if d > max_m:
        return (f"target is {d:.2f} m away; goto is capped at {max_m:.0f} m (and ~1.5 m is what "
                f"fits in its 40 s) — pick a closer waypoint")
    return None


def _short_err(e):
    code = getattr(e, "status_code", None)
    msg = str(getattr(e, "message", "") or e).replace("\n", " ")
    return (f"HTTP {code}: " if code else f"{type(e).__name__}: ") + msg[:200]


def _summ(res):
    if not isinstance(res, dict):
        return str(res)[:40]
    if res.get("stopped"):
        return str(res["stopped"])
    if res.get("ok") is False:
        return "refused: " + str(res.get("error", ""))[:60]
    return "ok"


def summarize_trace(trace):
    return "; ".join(f"{t['tool']} -> {_summ(t['result'])}" for t in trace[-4:]) or "nothing ran"


def _strip_images(msgs):
    """Replace image parts with a note (for a text-only fallback model)."""
    for m in msgs:
        c = m.get("content")
        if isinstance(c, list):
            txt = " ".join(p.get("text", "") for p in c if p.get("type") == "text")
            m["content"] = txt + " [the eye image was dropped: this model is text-only; call look]"
    return msgs


class _Surface:
    """The harness backend contract (say/gesture/goto/...) routed through
    Brains.tool, so harness.intent.execute gets the same guards, argument
    checks and voice gate as the LLM brains."""

    def __init__(self, brains, voice=False):
        self._b, self._voice = brains, voice

    async def _t(self, tool, /, **args):          # positional-only: gesture's arg is `name`
        return await self._b.tool(tool, args, voice=self._voice)

    async def say(self, word):
        return await self._t("say", word=word)

    async def gesture(self, name, direction=None):
        return await self._t("gesture", name=name, **({"direction": direction} if direction else {}))

    async def goto(self, x, y):
        return await self._t("goto", x=x, y=y)

    async def stop(self):
        return await self._t("stop")

    async def scan_summary(self):
        return await self._t("scan_summary")

    async def status(self):
        return await self._t("status")

    async def look(self):
        return await self._t("look")

    async def list_gestures(self):
        return await self._t("list_gestures")


# ------------------------------------------------------------------ the brains
class Brains:
    """Roles, histories, locks and fallbacks for one cockpit sim.

    `sim` needs: gesture_names, lexicon, gestures {name: (fn, total)},
    frames['eye'] = (n, jpeg), log(str), events (deque), note_cmd(str),
    async call(fn), tool_<say|stop|scan_summary|status|goto|gesture>, and for
    previews start_gesture / gesture_busy / reload_library (Playground, D052).
    `complete` (tests) replaces the OpenAI call: fn(model, messages, tools,
    max_tokens) -> {"content": str, "tool_calls": [{"id", "name", "arguments"}]}."""

    def __init__(self, sim, base_url=None, api_key=None, conf_path=CONF_PATH, complete=None,
                 timeout_s=CALL_TIMEOUT_S, catalog=None):
        self.sim = sim
        self.base = (base_url or os.environ.get("ROCKY_LLM_BASE_URL") or DEFAULT_BASE).rstrip("/")
        self.key = api_key or local_ai_key()
        self.timeout_s = float(timeout_s)
        self.conf_path = conf_path
        self.state = dict(mode="talk", **ROLE_DEFAULTS)       # sim.brain IS this dict
        self.hist = {}                                         # mode -> [{user, reply, tools, t}]
        self._locks = {}
        self._complete_fn = complete
        self._client = None
        self._cat = (time.monotonic(), catalog, True) if catalog is not None else (0.0, None, False)
        self._probe = {}                                       # id -> vision bool from /props
        self._voice_last = (0.0, "")
        self.composed = None                                   # the last compose_gesture draft
        self._load_conf()

    # ---------------------------------------------------------------- config
    def _load_conf(self):
        if not self.conf_path:
            return
        try:
            with open(self.conf_path) as f:
                saved = json.load(f).get("roles", {})
        except (OSError, ValueError, AttributeError):
            return
        for k, v in saved.items():
            if k in ROLE_DEFAULTS and isinstance(v, str) and v and v not in QUARANTINED:
                self.state[k] = v

    def _save_conf(self):
        if not self.conf_path:
            return
        try:
            os.makedirs(os.path.dirname(self.conf_path), exist_ok=True)
            with open(self.conf_path, "w") as f:
                json.dump({"roles": {k: self.state[k] for k in ROLE_DEFAULTS}}, f, indent=1)
        except OSError as e:
            self._log(f"brains: could not save {self.conf_path}: {e}")

    def roles(self):
        return {r: self.state[k] for r, k in ROLE_KEYS.items()}

    def _log(self, msg):
        try:
            self.sim.log(msg)
        except Exception:
            pass

    # --------------------------------------------------------------- catalog
    def catalog(self, refresh=False):
        """(models list or None when llama-swap is unreachable, reachable bool).
        Cached CATALOG_TTL_S; blocking — call it off the event loop."""
        t, cat, ok = self._cat
        if not refresh and cat is not None and time.monotonic() - t < CATALOG_TTL_S:
            return cat, ok
        try:
            import httpx
            r = httpx.get(f"{self.base}/models", headers={"Authorization": f"Bearer {self.key}"},
                          timeout=4.0)
            r.raise_for_status()
            cat, ok = classify_models(r.json().get("data", [])), True
        except Exception:
            cat, ok = (self._cat[1], False)                     # keep the last good list
        self._cat = (time.monotonic(), cat, ok)
        return cat, ok

    async def _acatalog(self):
        t, cat, _ = self._cat
        if cat is None or time.monotonic() - t >= CATALOG_TTL_S:
            await asyncio.to_thread(self.catalog, True)
        return {m["id"]: m for m in (self._cat[1] or [])} if self._cat[1] is not None else None

    def _probe_vision(self, entry):
        """Lazy truth from llama-server's /props (modalities.vision), only for a
        model that is ALREADY loaded — probing a cold one would load it."""
        mid = entry["id"]
        if mid in self._probe or entry["selector"] or not entry["loaded"]:
            return self._probe.get(mid)
        root = self.base[:-3] if self.base.endswith("/v1") else self.base
        try:
            import httpx
            r = httpx.get(f"{root}/upstream/{mid}/props",
                          headers={"Authorization": f"Bearer {self.key}"}, timeout=3.0)
            mod = r.json().get("modalities")
            self._probe[mid] = bool(mod.get("vision")) if isinstance(mod, dict) else None
        except Exception:
            self._probe[mid] = None
        return self._probe[mid]

    def chain(self, role, first=None, cat=None):
        """([model ids to try in order], [notes on what was skipped and why])."""
        key = ROLE_KEYS[role]
        cands = [first or self.state.get(key)] + FALLBACK.get(role, [])
        out, notes, seen = [], [], set()
        for c in cands:
            if not c or c in out or c in seen:
                continue
            if c in QUARANTINED:
                notes.append(f"{c} is quarantined (GPU faults) — never used")
                continue
            if cat is not None:
                e = cat.get(c)
                if e is None:
                    notes.append(f"{c} is not in llama-swap")
                    continue
                if e["quarantined"]:
                    notes.append(f"{c} is quarantined — skipped")
                    continue
                if role in ("vision", "multimodal"):
                    probed = self._probe.get(c)
                    if not e["vision"] or probed is False:
                        notes.append(f"{c} is text-only — skipped")
                        continue
                if e["selector"] and e["targets"]:
                    seen.add(e["targets"][0])                   # don't retry the same weights
            out.append(c)
        return out, notes

    def models_payload(self):
        """GET /api/models (blocking: run in a thread)."""
        cat, ok = self.catalog(refresh=True)
        cat = cat or []
        for e in cat:
            if e["loaded"] and e["vision"]:
                p = self._probe_vision(e)
                if p is False:
                    e["vision"] = False
        usable = [m for m in cat if not m["quarantined"]]
        cmap = {m["id"]: m for m in cat} if ok else None
        chains = {r: self.chain(r, cat=cmap)[0] for r in ("brain", "vision", "multimodal")}
        chains["brain"] = chains["brain"] + ["talk"]
        return {
            "models": [m["id"] for m in usable],                           # legacy UI: ids
            "vision_models": [m["id"] for m in usable if m["vision"]],     # legacy UI: ids
            "catalog": cat, "reachable": ok, "roles": self.roles(), "fallbacks": chains,
            "modes": list(MODES), "brain": self.state, "claude": self.claude_available(),
            "base_url": self.base, "stt": {"url": WHISPER_URL, "model": self.state["stt_model"]},
            "quarantined": sorted(QUARANTINED),
            "draft": None if self.composed is None else {        # the gesture studio can load it
                k: self.composed.get(k) for k in ("name", "spec", "ok", "report", "saved")}}

    def set_roles(self, body):
        """POST /api/brain (blocking: run in a thread). Keys: mode, and any role
        by name (brain|vision|multimodal|claude|stt) or state key (model,
        vision_model, ...), or roles={...}. Refuses quarantined models, text-only
        models for vision/multimodal, and unknown modes — saying why."""
        body = dict(body or {})
        items = dict(body.get("roles") or {}) if isinstance(body.get("roles"), dict) else {}
        items.update({k: v for k, v in body.items() if k != "roles"})
        errors, changed = [], False
        cat, ok = self.catalog()
        cmap = {m["id"]: m for m in cat} if (cat is not None and ok) else None
        if "mode" in items:
            if items["mode"] in MODES:
                self.state["mode"] = items["mode"]
            else:
                errors.append(f"unknown mode {items['mode']!r} (have {', '.join(MODES)})")
        for k, v in items.items():
            key = ROLE_KEYS.get(k, k if k in ROLE_DEFAULTS else None)
            if key is None or v is None:
                continue
            v = str(v).strip()
            err = self._role_error(key, v, cmap)
            if err:
                errors.append(err)
            elif self.state[key] != v:
                self.state[key] = v
                changed = True
        if changed:
            self._save_conf()
        return {"ok": not errors, "errors": errors, "brain": self.state, "roles": self.roles()}

    def _role_error(self, key, v, cmap):
        if not v:
            return f"{key}: empty model id"
        if v in QUARANTINED:
            return f"{v} is quarantined (GPU faults) — not selectable"
        if key == "claude_model":
            return None if v.startswith("claude") else f"{v} is not a Claude model id"
        if key == "stt_model":
            return None
        if cmap is not None:
            e = cmap.get(v)
            if e is None:
                return f"{v} is not in llama-swap's model list"
            if e["quarantined"]:
                return f"{v} is quarantined — not selectable"
            if key in ("vision_model", "multimodal_model") and not e["vision"]:
                return f"{v} is text-only (an image returns an error) — not a {key.split('_')[0]} model"
        return None

    def claude_available(self):
        try:
            import anthropic                                        # noqa: F401
        except ImportError:
            return False
        return bool(os.environ.get("ANTHROPIC_API_KEY"))

    def clear(self, mode=None):
        if mode in (None, "all"):
            self.hist.clear()
        else:
            self.hist.pop(mode, None)

    # ------------------------------------------------------------ model calls
    def _openai(self):
        if self._client is None:
            from openai import OpenAI
            self._client = OpenAI(base_url=self.base, api_key=self.key,
                                  timeout=self.timeout_s, max_retries=0)
        return self._client

    def _complete(self, model, messages, tools, max_tokens):
        if self._complete_fn is not None:
            return self._complete_fn(model, messages, tools, max_tokens)
        kw = dict(model=model, messages=messages, max_tokens=max_tokens,
                  extra_body={"chat_template_kwargs": {"enable_thinking": False}})
        if tools:
            kw["tools"] = tools
        r = self._openai().chat.completions.create(**kw)
        msg = r.choices[0].message
        return {"content": msg.content or "",
                "tool_calls": [{"id": tc.id, "name": tc.function.name,
                                "arguments": tc.function.arguments}
                               for tc in (msg.tool_calls or [])]}

    async def _acall(self, model, messages, tools, max_tokens):
        try:
            r = await asyncio.wait_for(
                asyncio.to_thread(self._complete, model, messages, tools, max_tokens),
                self.timeout_s + 5.0)
        except asyncio.TimeoutError:
            raise ModelFailure(f"timed out after {self.timeout_s:.0f} s") from None
        except ModelFailure:
            raise
        except Exception as e:
            raise ModelFailure(_short_err(e)) from None
        r = dict(r or {})
        r["content"] = _THINK_RE.sub("", r.get("content") or "").strip()
        r["tool_calls"] = list(r.get("tool_calls") or [])
        return r

    # ------------------------------------------------------------------ tools
    def tools_for(self, look=True):
        s = self.sim
        return build_tools(getattr(s, "gesture_names", None), getattr(s, "lexicon", None),
                           signed=SIGNED, look=look, extra=EXTRA_TOOLS)

    def system_for(self, multimodal=False):
        s = self.sim
        extra = LOOK_NOTE + " " + COMPOSE_NOTE + (" " + MULTIMODAL_NOTE if multimodal else "")
        return build_system(getattr(s, "gesture_names", None), getattr(s, "lexicon", None), extra)

    def surface(self, voice=False):
        return _Surface(self, voice)

    async def tool(self, name, args=None, voice=False, look_model=None):
        """Run one tool for any brain or the MCP proxy. Never raises: a bad
        argument or a failing tool comes back as {"ok": False, "error": ...}."""
        if name not in TOOL_NAMES:
            return {"ok": False, "error": f"no such tool {name!r}", "hint": ", ".join(TOOL_NAMES)}
        if args is None:
            args = {}
        if not isinstance(args, dict):
            return {"ok": False, "error": f"arguments for {name} must be an object"}
        if voice and name in MOTION_TOOLS:
            return {"ok": False, "error": "voice_unconfirmed",
                    "hint": "a spoken motion command needs the wake word ('pebble, ...') "
                            "or the operator's confirmation"}
        if name in NOTED:
            try:
                self.sim.note_cmd(f"tool {name} {json.dumps(args)}")
            except Exception:
                pass
        try:
            if name == "look":
                return await self.look(args.get("model") or look_model)
            if name == "gesture":
                return await self._gesture(**args)
            if name == "list_gestures":
                return self.list_gestures(**args)
            if name == "compose_gesture":
                return await self.compose_gesture(**args)
            if name == "check_gesture":
                return await self.check_gesture(**args)
            if name == "save_gesture":
                return await self.save_gesture(**args)
            return await getattr(self.sim, "tool_" + name)(**args)
        except TypeError as e:
            return {"ok": False, "error": f"bad arguments for {name}: {e}"}
        except Exception as e:
            self._log(f"tool {name} failed: {type(e).__name__}: {e}")
            return {"ok": False, "error": f"{name} failed: {type(e).__name__}: {e}"}

    def list_gestures(self):
        g = getattr(self.sim, "gestures", {}) or {}
        names = list(getattr(self.sim, "gesture_names", None) or sorted(g))
        kf = sorted(k for k, v in g.items() if hasattr(v[0], "spec"))
        return {"ok": True, "gestures": names, "keyframe": kf, "signed": list(SIGNED),
                "draft": None if self.composed is None else self.composed["name"]}

    async def _gesture(self, name, direction=None):
        if direction not in ("", None, "left", "right"):
            return {"ok": False, "error": f"direction must be left or right, got {direction!r}"}
        if direction and name not in SIGNED:
            return {"ok": False, "error": f"direction applies to {', '.join(SIGNED)} only"}
        if direction != "right":
            r = await self.sim.tool_gesture(name)
            if isinstance(r, dict) and direction == "left" and r.get("ok"):
                r["direction"] = "left"
            return r
        # the right-handed mirror: same gait, negated command (the gait turns both ways;
        # the registry only carries the left-handed canon)
        import pebble_gestures2 as pg2
        if name == "turn_in_place":
            total, cmd = pg2.TURN_TOTAL, (0.0, 0.0, -pg2.TURN_WZ)
        else:
            total, cmd = pg2.SIDE_TOTAL, (0.0, -pg2.SIDE_VY, 0.0)

        def fn(g, t, total=total, cmd=cmd):
            return pg2._gaited(g, t, total, cmd)
        fn.gaited = True                         # D052 V2: it walks — the sim2real locomotion hold applies
        r = await self._run_fn(fn, total, f"{name}_right")
        if r.get("ok"):
            r.update(gesture=name, direction="right")
        return r

    async def _run_fn(self, fn, total, label):
        """Start a gesture function through the Playground's blend (C, D052) and
        wait until its exit blend is done. Refusals come back as busy."""
        sim = self.sim
        why = await sim.call(lambda: sim.start_gesture(fn, total, name=label))
        if why:
            return {"ok": False, "error": "busy", "hint": str(why)}
        sim.mode = "gesturing"
        try:
            sim.events.append(("gesture", label))
        except Exception:
            pass
        t_end = time.monotonic() + total / max(getattr(sim, "speed", 1.0), 0.05) + 4.0
        await asyncio.sleep(0.1)
        while getattr(sim, "gesture_busy", False) and time.monotonic() < t_end:
            await asyncio.sleep(0.05)
        sim.mode = "idle"
        pose = sim.pose() if hasattr(sim, "pose") else None
        return {"ok": True, "gesture": label, "duration_s": round(float(total), 1), "pose": pose}

    # ---------------------------------------------------------- gesture studio
    def _spec(self, name=None, keyframes=None, loop=False, end="stand", description=None, spec=None):
        if spec is not None:
            if isinstance(spec, str):
                spec, err = parse_args_json(spec)
                if err:
                    raise ValueError(f"spec: {err}")
            if not isinstance(spec, dict):
                raise ValueError("spec must be an object {name, keyframes}")
            return dict(spec)
        if isinstance(keyframes, str):
            try:
                keyframes = json.loads(keyframes, parse_constant=_no_const)
            except ValueError as e:
                raise ValueError(f"keyframes are not valid JSON: {e}") from None
        if isinstance(keyframes, dict) and "keyframes" in keyframes:
            keyframes = keyframes["keyframes"]
        if not isinstance(keyframes, list) or not keyframes:
            raise ValueError("keyframes must be a non-empty list of frames")
        out = {"name": str(name or "composed"), "loop": bool(loop), "end": end or "stand",
               "keyframes": keyframes}
        if description:
            out["description"] = str(description)[:300]
        return out

    async def _check(self, spec):
        import pebble_feasibility as pf
        model = getattr(self.sim, "model", None)
        g = getattr(self.sim, "gait", None)
        rep = await asyncio.to_thread(pf.check_spec, spec, g, 100.0, model)
        return rep

    async def check_gesture(self, spec=None, name=None, **kw):
        if spec is None and name is not None:
            if self.composed is not None and name == self.composed["name"]:
                spec = self.composed["spec"]
            else:
                kg = (getattr(self.sim, "gestures", {}) or {}).get(name, (None,))[0]
                if not hasattr(kg, "spec"):
                    return {"ok": False, "error": f"{name!r} is not a keyframe gesture or the draft"}
                spec = kg.spec
        try:
            spec = self._spec(spec=spec, **kw) if spec is not None else self._spec(**kw)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        rep = await self._check(spec)
        return {"ok": bool(rep.ok), "feasible": bool(rep.ok), "name": spec.get("name"),
                "report": list(rep.lines), "peak_rad_s": round(float(rep.peak()[0]), 2)}

    async def compose_gesture(self, name, keyframes, description=None, loop=False, end="stand"):
        """Check a described gesture; if feasible, preview it once in the sim and
        keep it as the draft (save_gesture keeps it). Infeasible: nothing moves."""
        try:
            spec = self._spec(name=name, keyframes=keyframes, loop=loop, end=end,
                              description=description)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        rep = await self._check(spec)
        self.composed = {"name": spec["name"], "spec": spec, "ok": bool(rep.ok),
                         "report": list(rep.lines)}
        if not rep.ok:
            return {"ok": False, "feasible": False, "name": spec["name"], "report": list(rep.lines),
                    "hint": "not previewed: fix the FAIL lines and compose again"}
        from pebble_keyframes import KeyframeGesture
        kg = KeyframeGesture(spec)
        if kg.loop:
            return {"ok": True, "feasible": True, "name": spec["name"], "report": list(rep.lines),
                    "previewed": False, "note": "a looping gesture is not auto-previewed; "
                                                "say save to keep it, then play it by name"}
        run = await self._run_fn(kg, kg.total, spec["name"])
        if not run.get("ok"):
            return {"ok": False, "feasible": True, "name": spec["name"], "report": list(rep.lines),
                    "previewed": False, "error": run.get("error"), "hint": run.get("hint")}
        return {"ok": True, "feasible": True, "name": spec["name"], "report": list(rep.lines),
                "previewed": True, "duration_s": round(kg.total, 1),
                "note": "previewed, say save to keep it"}

    async def save_gesture(self, name=None, overwrite=False):
        if self.composed is None:
            return {"ok": False, "error": "nothing composed yet — compose_gesture first"}
        from pebble_keyframes import save_keyframe_gesture, _safe_name
        spec = dict(self.composed["spec"])
        try:
            spec["name"] = _safe_name(str(name or spec["name"]))
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        g = getattr(self.sim, "gestures", {}) or {}
        if spec["name"] in g and not hasattr(g[spec["name"]][0], "spec"):
            return {"ok": False, "error": f"{spec['name']!r} is a built-in gesture — pick another name"}
        if spec["name"] in g and not overwrite:
            return {"ok": False, "error": f"{spec['name']!r} exists — pass overwrite=true or a new name"}
        try:
            path = await asyncio.to_thread(save_keyframe_gesture, spec)
        except (ValueError, KeyError) as e:
            return {"ok": False, "error": str(e)}
        if hasattr(self.sim, "reload_library"):
            self.sim.reload_library()
        self._log(f"gesture saved: {os.path.relpath(path, ROOT)}")
        self.composed = dict(self.composed, name=spec["name"], saved=path)
        return {"ok": True, "name": spec["name"], "path": os.path.relpath(path, ROOT)}

    # ------------------------------------------------------------------ look
    def _eye_b64(self):
        frames = getattr(self.sim, "frames", {}) or {}
        _n, jpg = frames.get("eye", (0, b""))
        return base64.b64encode(jpg).decode() if jpg else None

    async def look(self, model=None):
        b64 = self._eye_b64()
        if b64 is None:
            return {"ok": False, "error": "no eye frame yet (renderer off?)"}
        cat = await self._acatalog()
        chain, tried = self.chain("vision", first=model, cat=cat)
        for m in chain:
            msgs = [{"role": "user", "content": [
                {"type": "text", "text": VISION_PROMPT},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{b64}"}}]}]
            try:
                r = await self._acall(m, msgs, None, LOOK_MAX_TOKENS)
            except ModelFailure as e:
                tried.append(f"{m}: {e}")
                continue
            text = r["content"]
            if not text:
                tried.append(f"{m}: empty description")
                continue
            try:
                self.sim.events.append(("look", text[:60]))
            except Exception:
                pass
            self._log(f"look ({m}): {text}")
            out = {"ok": True, "model": m, "description": text}
            if tried:
                out["fallback"] = tried
            return out
        return {"ok": False, "error": "no vision model described the frame" +
                (": " + "; ".join(tried) if tried else ""), "tried": tried}

    # ------------------------------------------------------------------ voice
    async def transcribe(self, raw, mode=None, trusted=False):
        """Browser audio blob -> ffmpeg 16 kHz mono -> whisper verbose_json ->
        cleaned text. Returns {ok, text, motion_blocked, wake, dropped}."""
        # A phone that lost the hold (long-press callout, a scroll, a synthetic
        # mouse event) sends a container with no audio in it; say so with the
        # size instead of a misleading "nothing heard" (D052 phone test).
        if not raw or len(raw) < MIN_VOICE_BYTES:
            return {"ok": False, "error": f"recording too short ({len(raw) if raw else 0} bytes) — "
                                          "hold the button while you speak, or tap once to start and "
                                          "tap again to stop"}
        if not shutil.which("ffmpeg"):
            return {"ok": False, "error": "ffmpeg not installed"}
        with tempfile.TemporaryDirectory() as td:
            src, wav = os.path.join(td, "in.webm"), os.path.join(td, "in.wav")
            with open(src, "wb") as f:
                f.write(raw)
            await asyncio.to_thread(subprocess.run, ["ffmpeg", "-y", "-loglevel", "error", "-i", src,
                                                     "-ar", "16000", "-ac", "1", wav], check=False)
            if not os.path.exists(wav):
                return {"ok": False, "error": "could not decode the audio"}
            audio_s = max(os.path.getsize(wav) - 44, 0) / 32000.0        # 16 kHz mono int16
            if audio_s < MIN_VOICE_S:
                self._log(f"voice: {audio_s:.2f} s of audio in {len(raw)} bytes — too short")
                return {"ok": False, "audio_s": round(audio_s, 2),
                        "error": f"recording too short ({audio_s:.1f} s) — hold the button while you speak, "
                                 "or tap once to start and tap again to stop"}
            import httpx
            try:
                async with httpx.AsyncClient(timeout=60) as c:
                    with open(wav, "rb") as fw:
                        r = await c.post(f"{WHISPER_URL}/v1/audio/transcriptions",
                                         files={"file": ("in.wav", fw, "audio/wav")},
                                         data={"response_format": "verbose_json", "temperature": "0.0",
                                               "language": "en", "prompt": WHISPER_PROMPT})
                if r.status_code != 200:
                    return {"ok": False, "audio_s": round(audio_s, 2),
                            "error": f"whisper-server HTTP {r.status_code}: {r.text[:160]}"}
                try:
                    data = r.json()
                except ValueError:
                    return {"ok": False, "audio_s": round(audio_s, 2),
                            "error": f"whisper-server answered non-JSON: {r.text[:160]}"}
            except Exception as e:
                return {"ok": False, "error": f"whisper-server: {e}"}
        out = self.voice_result(data, mode)
        out["audio_s"] = round(audio_s, 2)
        if trusted and out.get("text"):            # hands-free: the operator opted out of the wake word
            out["motion_blocked"] = False
            out["trusted"] = True
        if not out.get("text"):
            self._log(f"voice: {audio_s:.1f} s of audio, whisper heard nothing"
                      + (f" (dropped: {'; '.join(out['dropped'])})" if out.get("dropped") else ""))
        return out

    def voice_result(self, data, mode=None):
        """whisper output -> the /api/voice JSON (pure apart from remembering
        the transcript for the chat's voice check)."""
        c = clean_transcript(data)
        text = c["text"]
        mode = mode or self.state["mode"]
        wake = has_wake_word(text)
        # talk mode knows exactly what moves; an LLM brain may turn any sentence into a goto
        motion = bool(text) and (mode != "talk" or moves(intent_plan(text)))
        self._voice_last = (time.monotonic(), text)
        if text:
            self._log(f"voice: {text}")
        return {"ok": True, "text": text, "wake": wake, "motion_blocked": motion and not wake,
                "dropped": c["dropped"]}

    def _recent_voice(self, text):
        t, last = self._voice_last
        return bool(last) and text.strip() == last.strip() and time.monotonic() - t < VOICE_RECENT_S

    # ------------------------------------------------------------------- chat
    def _lock(self, mode):
        if mode not in self._locks:
            self._locks[mode] = asyncio.Lock()
        return self._locks[mode]

    def _accept_model(self, mode, model):
        """A per-message model from the (legacy) UI: honoured when it fits the mode."""
        if not model or mode == "talk":
            return ""
        key = {"local": "model", "multimodal": "multimodal_model", "claude": "claude_model"}[mode]
        cat = self._cat[1]
        cmap = {m["id"]: m for m in cat} if cat is not None else None
        err = self._role_error(key, str(model), cmap)
        if err:
            return "" if mode in ("claude", "multimodal") else f"({err}; kept {self.state[key]})"
        self.state[key] = str(model)
        return ""

    async def chat(self, text, mode=None, model=None, source=None, trusted=False):
        """One operator message -> {reply, trace, mode, model?, fallback?, voice,
        motion_allowed, motion_blocked}. source 'voice' | 'typed' | None (None:
        voice iff the text equals the last transcript — the legacy UI auto-sent it)."""
        text = str(text or "").strip()[:MAX_TEXT]
        mode = mode or self.state["mode"]
        if mode not in MODES:
            return {"reply": f"unknown mode {mode!r} (have {', '.join(MODES)})", "trace": [], "mode": mode}
        self.state["mode"] = mode
        voice = source == "voice" or (source is None and self._recent_voice(text))
        allow = bool(trusted) or not voice or has_wake_word(text)
        if not text:
            return {"reply": "", "trace": [], "mode": mode}
        note = self._accept_model(mode, model)
        async with self._lock(mode):
            if mode == "talk":
                r = await self._talk(text, allow)
            elif mode == "claude":
                r = await self._claude(text, allow)
            else:
                r = await self._llm(text, mode, allow)
        if note:
            r["reply"] = f"{note} {r['reply']}"
        blocked = any(isinstance(t.get("result"), dict) and t["result"].get("error") == "voice_unconfirmed"
                      for t in r["trace"])
        r.update(voice=voice, motion_allowed=allow, motion_blocked=blocked)
        if blocked:
            r["reply"] += (" (heard by voice without the wake word, so I did not move — say "
                           "'pebble, …' or press Enter to confirm)")
        return r

    async def _talk(self, text, allow):
        p = intent_plan(text)
        results = await intent_execute(self.surface(voice=not allow), p, allow_motion=allow)
        if p["relative"] is not None:                 # execute() turned the delta into a goto
            argl = [{"dx_m": p["relative"][0], "dy_m": p["relative"][1]}]
        else:
            argl = [a for _, a in p["calls"]]
        trace = [{"tool": n, "args": a, "result": r}
                 for (n, r), a in zip(results, argl + [{}] * len(results))]
        reply = p["reply"]
        if results:
            name, res = results[-1]
            if name == "look" and isinstance(res, dict):
                reply = (f"eye ({res.get('model')}): {res['description']}" if res.get("ok")
                         else f"look failed: {res.get('error')}")
            elif name in ("status", "scan_summary"):
                reply += " " + json.dumps(res, default=str)[:400]
            elif isinstance(res, dict) and res.get("ok") is False and res.get("error") != "voice_unconfirmed":
                reply += f" — {name} refused: {res.get('error')}" + (
                    f" ({res['hint']})" if res.get("hint") else "")
            elif isinstance(res, dict) and res.get("stopped") and res.get("stopped") != "arrived":
                reply += f" — stopped: {res['stopped']}"
        return {"reply": reply, "trace": trace, "mode": "talk"}

    def _messages(self, mode, text, image_b64=None):
        msgs = [{"role": "system", "content": self.system_for(multimodal=mode == "multimodal")}]
        for turn in self.hist.get(mode, [])[-MAX_TURNS:]:
            msgs.append({"role": "user", "content": turn["user"]})
            msgs.append({"role": "assistant", "content": turn["reply"]})
        if image_b64:
            msgs.append({"role": "user", "content": [
                {"type": "text", "text": text},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}}]})
        else:
            msgs.append({"role": "user", "content": text})
        return msgs

    def _remember(self, mode, text, reply, trace):
        h = self.hist.setdefault(mode, [])
        h.append({"user": text, "reply": reply or "(no reply)", "tools": [t["tool"] for t in trace],
                  "t": round(time.time(), 1)})
        del h[:-MAX_TURNS]

    async def _tool_loop(self, model, msgs, tools, trace, allow, look_model=None):
        """(reply, gave_up). Raises ModelFailure only from a model call — by
        then every earlier tool_call already has its result in msgs."""
        for hop in range(MAX_HOPS):
            r = await self._acall(model, msgs, tools, REPLY_MAX_TOKENS)
            calls = r["tool_calls"]
            if not calls:
                msgs.append({"role": "assistant", "content": r["content"]})
                return r["content"] or (f"done: {summarize_trace(trace)}" if trace
                                        else "(the model answered with no text)"), False
            parsed = []
            for i, tc in enumerate(calls):
                args, err = parse_args_json(tc.get("arguments"))
                parsed.append((tc.get("id") or f"call_{hop}_{i}", str(tc.get("name")), args, err))
            msgs.append({"role": "assistant", "content": r["content"], "tool_calls": [
                {"id": cid, "type": "function",
                 "function": {"name": n, "arguments": json.dumps(a if a is not None else {})}}
                for cid, n, a, _e in parsed]})
            for cid, n, a, err in parsed:
                res = ({"ok": False, "error": f"bad arguments for {n}: {err}"} if err
                       else await self.tool(n, a, voice=not allow, look_model=look_model))
                trace.append({"tool": n, "args": a or {}, "result": res})
                msgs.append({"role": "tool", "tool_call_id": cid,
                             "content": json.dumps(res, default=str)[:RESULT_CHARS]})
        return (f"gave up after {MAX_HOPS} tool rounds without a final answer "
                f"(last: {summarize_trace(trace)})"), True

    async def _llm(self, text, mode, allow):
        trace, notes = [], []
        cat = await self._acatalog()
        stages = []
        image = None
        if mode == "multimodal":
            image = self._eye_b64()
            if image is None:
                notes.append("no eye frame yet — sent text only")
            ch, sk = self.chain("multimodal", cat=cat)
            notes += sk
            stages.append(("multimodal", ch))
        ch, sk = self.chain("brain", cat=cat)
        if mode == "local":
            notes += sk
        stages.append(("brain", ch))
        msgs = self._messages(mode, text, image)
        for stage, chain in stages:
            tools = self.tools_for(look=True)
            for mdl in chain:
                try:
                    reply, gave_up = await self._tool_loop(
                        mdl, msgs, tools, trace, allow,
                        look_model=mdl if stage == "multimodal" else None)
                except ModelFailure as e:
                    notes.append(f"{mdl} failed ({e})")
                    self._log(f"brain {mdl} failed: {e}")
                    continue
                if notes:
                    reply = f"[{'; '.join(notes)} → answered by {mdl}] {reply}"
                self._remember(mode, text, reply, trace)
                return {"reply": reply, "trace": trace, "mode": mode, "model": mdl,
                        "fallback": notes, "gave_up": gave_up}
            if stage == "multimodal":
                _strip_images(msgs)
                notes.append("multimodal chain exhausted — text brain + look")
        # nothing answered: the regex brain, unless tools already ran (never act twice)
        if trace:
            reply = (f"[no model finished the turn: {'; '.join(notes)}] what ran: "
                     f"{summarize_trace(trace)}")
            r = {"reply": reply, "trace": trace, "mode": mode, "model": None}
        else:
            r = await self._talk(text, allow)
            why = "; ".join(notes) or "none available"
            r["reply"] = f"[no model answered: {why} → regex brain] {r['reply']}"
            r["mode"], r["model"] = mode, "talk"
        r["fallback"] = notes
        self._remember(mode, text, r["reply"], trace)
        return r

    async def _claude(self, text, allow):
        if not self.claude_available():
            return {"reply": "Claude in the cockpit needs `pip install anthropic` and ANTHROPIC_API_KEY. "
                             "Without a key, run `./rocky.sh chat` in a terminal: with the cockpit up, "
                             "Claude Code drives THIS sim over MCP and you watch it here.",
                    "trace": [], "mode": "claude"}
        import anthropic
        client = anthropic.Anthropic(timeout=self.timeout_s, max_retries=0)
        model = self.state["claude_model"]
        tools = [{"name": t["function"]["name"], "description": t["function"]["description"],
                  "input_schema": t["function"]["parameters"]} for t in self.tools_for(look=True)]
        system = self.system_for()
        msgs = []
        for turn in self.hist.get("claude", [])[-MAX_TURNS:]:
            msgs += [{"role": "user", "content": turn["user"]},
                     {"role": "assistant", "content": turn["reply"]}]
        msgs.append({"role": "user", "content": text})
        trace, content, gave_up = [], "", True
        for _hop in range(MAX_HOPS):
            try:
                r = await asyncio.wait_for(asyncio.to_thread(lambda: client.messages.create(
                    model=model, max_tokens=REPLY_MAX_TOKENS, system=system, tools=tools,
                    messages=msgs)), self.timeout_s + 5.0)
            except Exception as e:
                err = "timed out" if isinstance(e, asyncio.TimeoutError) else _short_err(e)
                content = f"[Claude {model} failed: {err}] " + (
                    f"what ran: {summarize_trace(trace)}" if trace else "nothing was done")
                gave_up = False
                break
            msgs.append({"role": "assistant", "content": r.content})
            uses = [b for b in r.content if b.type == "tool_use"]
            content = " ".join(b.text for b in r.content if b.type == "text").strip()
            if not uses:
                gave_up = False
                content = content or (f"done: {summarize_trace(trace)}" if trace else "(no text)")
                break
            results = []
            for u in uses:                                  # every tool_use gets its tool_result
                args = dict(u.input) if isinstance(u.input, dict) else {}
                res = await self.tool(u.name, args, voice=not allow)
                trace.append({"tool": u.name, "args": args, "result": res})
                results.append({"type": "tool_result", "tool_use_id": u.id,
                                "content": json.dumps(res, default=str)[:RESULT_CHARS]})
            msgs.append({"role": "user", "content": results})
        if gave_up:
            content = (f"gave up after {MAX_HOPS} tool rounds without a final answer "
                       f"(last: {summarize_trace(trace)})")
        self._remember("claude", text, content, trace)
        return {"reply": content, "trace": trace, "mode": "claude", "model": model, "gave_up": gave_up}


def _no_const(c):
    raise ValueError(f"non-finite number {c}")
