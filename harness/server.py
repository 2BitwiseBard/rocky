"""Rocky-MCP tool server v0 (docs/MCP_CONTRACT_v0.md) — FastMCP over stdio.

The model proposes, the reflex layer disposes: every tool result is an
ordinary value (vetoes included — a goto stopped by the cliff reflex
returns {"stopped": "cliff"}, it does not raise). The backend owns all
robot state; kill the brain any time and the robot is safe.

Run against the mock (default) or the MuJoCo sim:

    python3 -m harness.server                 # mock backend
    ROCKY_BACKEND=sim python3 -m harness.server   # MuJoCo cliff world
    ROCKY_BACKEND=auto python3 -m harness.server  # the cockpit whenever one answers, else the sim
                                                  # (re-resolved per call, logged to stderr — D052)

Wire into a client (e.g. Claude Code .mcp.json):
    {"rocky": {"command": "python3", "args": ["-m", "harness.server"],
               "cwd": "<repo>/rocky"}}

Tools: say, gesture, move, goto, stop, scan_summary, status, list_gestures, and
look / find_object where an eye exists and remember / where_is / recall /
go_back_to / forget where the scene memory exists (a cockpit backend). map_query / patrol / dock are
not stubbed — absent tool > lying tool. D052: the gesture and say docs are
built from the backend's LIVE lists (saved keyframe gestures and custom chord
words included); gesture takes direction left|right for turn_in_place /
sidestep. move (2026-09-25) is a relative move in the robot's frame
(forward_m, left_m): the server reads the pose from the backend's status and
calls the backend's own goto (harness.local_brain.move_via), so every backend
gets it with goto's guards and results, unchanged; a cockpit backend runs it
itself (/api/tool/move: its stop check between the pose read and the goto).
move and goto refuse a boolean for a distance, as the cockpit does (pydantic's
lax mode made move(forward_m=true) a 1 m walk).

D056 — the tool list is generated, and it is live. Every tool comes from the
one registry, harness/capabilities.py: build_server registers
to_mcp_specs(build(<the backend's lists>, <its capabilities()>)) — names,
texts, input schemas and annotations exactly as before D056 — and each one
forwards to backend.<backend_method>(...), positionally in the schema's order
as before (a cockpit backend without that method: POST /api/tool/<name>).
What the registry cannot say stays here as per-tool adapters keyed by name:
ADAPTERS (gesture's direction fallback for a backend without one, move
through move_via, list_gestures' canon answer) and ARG_TYPES (the Meters that
refuse a boolean). LiveTools keeps the backend's last capabilities snapshot
(a cockpit: GET /api/capabilities; before D056 the gesture + chord list
routes): a tools/list whose snapshot is older than LIST_TTL_S (2 s) re-reads
it and re-registers the tools whose text or schema changed (remove_tool +
add_tool), so a gesture saved in the cockpit's studio is in gesture's text on
the next list, no restart. Once a client has made a request, a watcher polls
it every POLL_S (3 s) and sends notifications/tools/list_changed when the list
moved (the server then declares tools.listChanged); both are best effort — a
failure is logged to stderr, never fatal, and a fetch that fails keeps the
last list. Backends with nothing to follow (the mock, the in-process sim) stay
static, exactly as before. ROCKY_MCP_LIVE=0 turns the refresh and the watcher
off: the lists are then read once, at start, as before D056.
"""
from __future__ import annotations

import inspect
import os
import sys
import time
from contextlib import asynccontextmanager
from typing import Annotated, Any

import anyio
import anyio.to_thread
from mcp.server.fastmcp import FastMCP
from mcp.server.lowlevel.server import NotificationOptions
from mcp.types import ToolAnnotations
from pydantic import BeforeValidator

from harness import capabilities as C
from harness.backend import MockBackend, GESTURES, CHORD_WORDS, SIGNED, derived_capabilities
from harness.intent import _accepts
from harness.local_brain import move_via

LIST_TTL_S = 2.0          # a tools/list re-reads the backend's capabilities when its copy is older
POLL_S = 3.0              # the list-changed watcher's poll interval
LIVE_ENV = "ROCKY_MCP_LIVE"   # "0" = no refresh, no watcher: the lists are read once at start

_ORDER = {name: i for i, name in enumerate(C.TOOL_NAMES)}     # tools/list keeps the registry's order


def _log(msg):
    print(f"[rocky-mcp] {msg}", file=sys.stderr, flush=True)     # stdout is the MCP stream


def _no_bool(v):
    """MCP arguments go through pydantic's lax mode, where true is 1.0: move(forward_m=true)
    walked a metre and goto(true, 0) went to (1, 0), while the cockpit refuses a boolean
    (validate_move / validate_goto). Refused here the same way; the schema stays 'number'."""
    if isinstance(v, bool):
        raise ValueError("meters as a number, got a boolean")     # noqa: TRY004 — pydantic reports a ValueError
    return v


Meters = Annotated[float, BeforeValidator(_no_bool)]
# left_m: null reads as 0 (the default), as the cockpit's validate_move reads it
LeftMeters = Annotated[float, BeforeValidator(lambda v: 0.0 if v is None else _no_bool(v))]


# ---------------------------------------------------------------- per-tool adapters
async def _gesture(be, name, direction=""):
    """direction left|right where the backend takes one; the canon's gesture turns / steps
    left, so 'left' still runs on a backend without direction, 'right' is refused."""
    if not direction:
        return await be.gesture(name)
    if _accepts(be.gesture, "direction"):
        return await be.gesture(name, direction=direction)
    if direction == "left":
        return await be.gesture(name)          # the canon turns/steps left
    return {"ok": False, "error": f"this backend cannot {name} {direction}"}


async def _move(be, forward_m, left_m=0.0):
    """No harness backend has a move method: status + goto on any backend (a cockpit's
    own /api/tool/move), see harness.local_brain.move_via."""
    return await move_via(be, forward_m, left_m)


async def _list_gestures(be):
    if hasattr(be, "list_gestures"):
        return await be.list_gestures()
    return {"ok": True, "gestures": list(GESTURES), "signed": list(SIGNED)}


ADAPTERS = {"gesture": _gesture, "move": _move, "list_gestures": _list_gestures}
# what an adapter needs from the backend (() = it answers by itself)
ADAPTER_NEEDS = {"gesture": ("gesture",), "move": ("status", "goto"), "list_gestures": ()}
# argument types the registry's JSON types do not carry (the published schema is the same)
ARG_TYPES = {("move", "forward_m"): Meters, ("move", "left_m"): LeftMeters,
             ("goto", "x"): Meters, ("goto", "y"): Meters}
_PY_TYPES = {"string": str, "number": float, "integer": int, "boolean": bool, "object": dict, "array": list}


def backend_capabilities(be) -> set:
    """The capability flags the backend says it has (its capabilities()), else what its
    methods show (harness.backend.derived_capabilities)."""
    fn = getattr(be, "capabilities", None)
    if callable(fn):
        try:
            return set(fn())
        except Exception as e:                    # noqa: BLE001 — a flag query must not kill the server
            _log(f"{type(be).__name__}.capabilities() failed ({e}); using its methods")
    return derived_capabilities(be)


def runnable(be, spec) -> bool:
    """True when the backend can run this tool: what the tool's adapter needs, else the
    backend method, else a cockpit's /api/tool/<name> (it runs any registry tool)."""
    name = spec["name"]
    if name in ADAPTER_NEEDS:
        return all(callable(getattr(be, m, None)) for m in ADAPTER_NEEDS[name])
    return callable(getattr(be, spec["backend_method"], None)) or callable(getattr(be, "_tool", None))


def tool_function(be, spec):
    """The async function FastMCP registers for one to_mcp_specs entry: its signature is
    the spec's input schema (so FastMCP publishes that schema and validates by it), its
    body the tool's adapter, else backend.<backend_method>(<args in schema order>), else a
    cockpit's /api/tool/<name>."""
    name = spec["name"]
    props = (spec.get("input_schema") or {}).get("properties") or {}
    argnames = list(props)
    method = spec["backend_method"]
    adapter = ADAPTERS.get(name)
    if adapter is not None:
        async def run(**kw):
            return await adapter(be, **kw)
    else:
        async def run(**kw):
            fn = getattr(be, method, None)
            if callable(fn):
                return await fn(*[kw[a] for a in argnames])      # positional, as before D056
            remote = getattr(be, "_tool", None)
            if callable(remote):
                return await remote(name, **kw)                   # a cockpit runs any registry tool
            return {"ok": False, "error": f"this backend cannot {name}"}
    params = []
    for arg, p in props.items():
        ty = ARG_TYPES.get((name, arg)) or _PY_TYPES.get(p.get("type"), Any)
        default = p["default"] if "default" in p else inspect.Parameter.empty
        params.append(inspect.Parameter(arg, inspect.Parameter.KEYWORD_ONLY, default=default, annotation=ty))
    run.__name__ = run.__qualname__ = name          # FastMCP names the argument model '<name>Arguments'
    run.__doc__ = spec["description"]
    run.__signature__ = inspect.Signature(params, return_annotation=dict)
    run.__annotations__ = {**{p.name: p.annotation for p in params}, "return": dict}
    return run


def _strs(v):
    return [x for x in v if isinstance(x, str)] if isinstance(v, list) else None


def _envelope_of(snap):
    """The snapshot's envelope when the texts can use it (goto_reach_m and move_max_m numbers)."""
    env = snap.get("envelope") if isinstance(snap, dict) else None
    if isinstance(env, dict) and all(isinstance(env.get(k), (int, float)) and not isinstance(env.get(k), bool)
                                     for k in ("goto_reach_m", "move_max_m")):
        return dict(env)
    return None


def _key(spec):
    return (spec["description"], C.canonical_json(spec["input_schema"]),
            C.canonical_json(spec["annotations"]))


def _env_enabled() -> bool:
    return os.environ.get(LIVE_ENV, "1").strip().lower() not in ("0", "false", "no", "off")


# ---------------------------------------------------------------- the live list
class LiveTools:
    """The MCP tool list as a function of the backend's capabilities (D056).

    fetch()      the backend's snapshot (fetch_capabilities(), else live_lists()), or None
    apply(snap)  rebuild the registry snapshot and re-register the tools that changed
    refresh()    fetch + apply when the copy is older than ttl_s (every tools/list)
    poll_once()  one watcher tick: refresh, and notify the session when the list moved
    """

    def __init__(self, backend, *, ttl_s=LIST_TTL_S, poll_s=POLL_S, enabled=None):
        self.be = backend
        self.mcp = None
        self.ttl_s, self.poll_s = ttl_s, poll_s
        self.enabled = _env_enabled() if enabled is None else bool(enabled)
        self.gestures, self.lexicon, self.signed = list(GESTURES), list(CHORD_WORDS), list(SIGNED)
        self.envelope = self.robot = None           # None = the registry's defaults (this machine's params)
        self.caps = None                            # the last snapshot (harness.capabilities.build)
        self.version = None
        self.registered = {}                        # tool name -> (description, schema, annotations)
        self.fetched_at = 0.0                       # time.monotonic() of the last fetch
        self.source_ok = None                       # did the last fetch answer? (None: never fetched)
        self.fetches = 0
        self.notified = 0                           # list_changed notifications sent
        self.session = None                         # the ServerSession of the last request
        self.watching = False
        self._tg = None                             # the lifespan's task group (the watcher runs in it)
        self._notify_failed = False

    # -------------------------------------------------------------- the source
    @property
    def has_source(self) -> bool:
        return (callable(getattr(self.be, "fetch_capabilities", None))
                or callable(getattr(self.be, "live_lists", None)))

    @property
    def live(self) -> bool:
        """Refresh and watch? Only with something to follow (a cockpit, live_lists) and not disabled."""
        return self.enabled and self.has_source

    def fetch(self):
        """Sync (a thread from async code). The backend's snapshot dict, or None."""
        self.fetches += 1
        try:
            f = getattr(self.be, "fetch_capabilities", None)
            if callable(f):
                snap = f()
                return snap if isinstance(snap, dict) else None
            f = getattr(self.be, "live_lists", None)
            if callable(f):
                g, w = f()
                return None if g is None and w is None else {"gestures": g, "lexicon": w}
        except Exception as e:                    # noqa: BLE001 — a source must never kill the server
            _log(f"capabilities fetch failed ({type(e).__name__}: {e})")
        return None

    # -------------------------------------------------------------- registration
    def attach(self, mcp):
        """Read the lists once and register every tool (build_server). A registry that
        cannot be registered raises here: a server that cannot list its tools must not start."""
        self.mcp = mcp
        mcp.live = self
        snap = self.fetch() if self.has_source else None
        self.fetched_at = time.monotonic()
        self.source_ok = isinstance(snap, dict) if self.has_source else None
        self.apply(snap)
        if self.live:                              # declare tools.listChanged: this list can move
            low = mcp._mcp_server
            make = low.create_initialization_options

            def init_options(notification_options=None, experimental_capabilities=None):
                return make(notification_options or NotificationOptions(tools_changed=True),
                            experimental_capabilities)
            low.create_initialization_options = init_options
        return self

    def apply(self, snap) -> bool:
        """Take the snapshot's lists (a field it lacks keeps the last value; an empty list is
        the canon, as before D056), rebuild, re-register what changed. True when the
        published tool list changed. Synchronous: no request sees half a list."""
        if isinstance(snap, dict):
            g, w, s = _strs(snap.get("gestures")), _strs(snap.get("lexicon")), _strs(snap.get("signed"))
            if g is not None:
                self.gestures = g or list(GESTURES)
            if w is not None:
                self.lexicon = w or list(CHORD_WORDS)
            if s:
                self.signed = s
            env = _envelope_of(snap)
            if env is not None:
                self.envelope = env
            if isinstance(snap.get("robot"), dict):
                self.robot = dict(snap["robot"])
        flags = backend_capabilities(self.be)
        caps = C.build(self.gestures, self.lexicon, self.signed, has_eye="eye" in flags,
                       has_memory="memory" in flags, is_cockpit="cockpit" in flags,
                       envelope=self.envelope, robot=self.robot)
        if caps["version"] == self.version:
            return False
        old = self.version
        changed = self._sync([s for s in C.to_mcp_specs(caps) if runnable(self.be, s)])
        self.caps, self.version = caps, caps["version"]
        if old is not None and changed:
            _log(f"tool list moved (capabilities {old} -> {self.version}): {', '.join(changed)}")
        return bool(changed)

    def _sync(self, specs) -> list:
        want = {s["name"]: s for s in specs}
        changed = []
        for name in list(self.registered):
            if name not in want:
                self.mcp.remove_tool(name)
                del self.registered[name]
                changed.append(f"-{name}")
        for name, spec in want.items():
            key = _key(spec)
            if self.registered.get(name) == key:
                continue
            if name in self.registered:
                self.mcp.remove_tool(name)
                del self.registered[name]
            ann = ToolAnnotations(**spec["annotations"]) if spec["annotations"] else None
            self.mcp.add_tool(tool_function(self.be, spec), name=name, description=spec["description"],
                              annotations=ann)
            self.registered[name] = key
            changed.append(name)
        return changed

    # -------------------------------------------------------------- refresh + watch
    async def refresh(self, force=False) -> bool:
        """Re-read the capabilities when the copy is older than ttl_s (force: now). True when
        the tool list changed. Never raises: a failed fetch keeps the last list."""
        if not self.live:
            return False
        now = time.monotonic()
        if not force and now - self.fetched_at < self.ttl_s:
            return False
        self.fetched_at = now
        snap = await anyio.to_thread.run_sync(self.fetch)
        ok = isinstance(snap, dict)
        if ok != self.source_ok:
            _log("capabilities answer again" if ok else
                 f"no capabilities from {type(self.be).__name__}: keeping the last tool list")
        self.source_ok = ok
        if not ok:
            return False
        try:
            return self.apply(snap)
        except Exception as e:                    # noqa: BLE001 — keep serving the last list
            _log(f"tool list not rebuilt ({type(e).__name__}: {e}); keeping the last list")
            return False

    def on_request(self):
        """Every request: remember its session; start the watcher once (in the lifespan's
        task group, so it ends with the server run)."""
        if not self.live or self.mcp is None:
            return
        try:
            session = self.mcp.get_context().request_context.session
        except (LookupError, ValueError, AttributeError):
            return                                 # not inside a request (a direct list_tools call)
        self.session = session
        if not self.watching and self._tg is not None:
            self.watching = True
            self._tg.start_soon(self.watch)

    async def poll_once(self) -> bool:
        """One watcher tick: refresh now; when the list moved, tell the client
        (notifications/tools/list_changed). True when a notification went out. Never raises."""
        try:
            changed = await self.refresh(force=True)
        except Exception as e:                    # noqa: BLE001
            _log(f"capabilities poll failed ({type(e).__name__}: {e})")
            return False
        session = self.session
        if not changed or session is None:
            return False
        try:
            await session.send_tool_list_changed()
        except Exception as e:                    # noqa: BLE001 — best effort: the next tools/list is right
            if not self._notify_failed:
                _log(f"tools/list_changed not sent ({type(e).__name__}: {e})")
            self._notify_failed = True
            return False
        self._notify_failed = False
        self.notified += 1
        return True

    async def watch(self):
        try:
            while True:
                await anyio.sleep(self.poll_s)
                try:
                    await self.poll_once()
                except Exception as e:            # noqa: BLE001 — the watcher never takes the server down
                    _log(f"capabilities watcher: {type(e).__name__}: {e}")
        finally:
            self.watching = False

    @asynccontextmanager
    async def lifespan(self, _app):
        """FastMCP lifespan: a task group for the watcher that lives exactly as long as the
        server run; yields the same empty context FastMCP's default lifespan does."""
        async with anyio.create_task_group() as tg:
            self._tg = tg
            try:
                yield {}
            finally:
                self._tg = None
                self.session = None
                tg.cancel_scope.cancel()


class RockyMCP(FastMCP):
    """FastMCP whose tool list follows the robot: tools/list refreshes it (LiveTools)."""
    live: LiveTools | None = None

    async def list_tools(self):
        live = self.live
        if live is not None:
            live.on_request()
            await live.refresh()
        tools = await super().list_tools()
        return sorted(tools, key=lambda t: _ORDER.get(t.name, len(_ORDER)))

    async def call_tool(self, name, arguments):
        if self.live is not None:
            self.live.on_request()
        return await super().call_tool(name, arguments)


def build_server(backend=None) -> RockyMCP:
    be = backend or MockBackend()
    live = LiveTools(be)
    mcp = RockyMCP("rocky", instructions=C.MCP_INSTRUCTIONS, lifespan=live.lifespan)
    live.attach(mcp)
    return mcp


def _pick_backend():
    kind = os.environ.get("ROCKY_BACKEND", "mock")
    if kind == "auto":
        from harness.cockpit_backend import AutoBackend, DEFAULT_URL
        be = AutoBackend()
        print(f"[rocky-mcp] ROCKY_BACKEND=auto: the cockpit at {DEFAULT_URL} whenever it "
              "answers, else the in-process sim — checked per call", file=sys.stderr, flush=True)
        return be
    if kind == "cockpit":
        from harness.cockpit_backend import CockpitBackend, cockpit_alive, DEFAULT_URL
        if not cockpit_alive():
            raise SystemExit(f"ROCKY_BACKEND=cockpit but nothing answers at {DEFAULT_URL} "
                             "(start ./rocky.sh cockpit first)")
        be = CockpitBackend()
    elif kind == "sim":
        from harness.sim_backend import SimBackend
        be = SimBackend()
    else:
        be = MockBackend()
    print(f"[rocky-mcp] backend: {type(be).__name__} (ROCKY_BACKEND={kind})",
          file=sys.stderr, flush=True)
    return be


if __name__ == "__main__":
    import logging
    # D056: the capabilities watcher GETs every POLL_S; httpx's INFO line per request would bury
    # the [rocky-mcp] lines on stderr (backend switches, 'tool list moved', stop not confirmed)
    logging.getLogger("httpx").setLevel(logging.WARNING)
    build_server(_pick_backend()).run()          # stdio transport
