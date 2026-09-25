"""CockpitBackend — the MCP contract proxied to a running cockpit
(sim/cockpit.py) over HTTP, so `./rocky.sh chat` (Claude Code over MCP),
local_brain and intent all drive the ONE sim the browser is showing.

    ROCKY_BACKEND=cockpit  ROCKY_COCKPIT_URL=http://127.0.0.1:8765
    ROCKY_BACKEND=auto     -> AutoBackend: the cockpit whenever one answers,
                              else the in-process sim — re-resolved per call

Adds the `look` and `find_object` tools (the robot's eye through a vision
model) and the scene-memory tools (`remember`, `where_is`, `recall`,
`go_back_to`, `forget` — sim/scene_memory.py), which only exist where a
cockpit is running — absent tool > lying tool.

D052: AutoBackend used to be resolved ONCE when the MCP server started, so
a cockpit started after `./rocky.sh chat` was never used (the chat drove an
invisible in-process sim) and nothing said so. Now each call checks (cached
AUTO_TTL_S) and every switch is logged loudly to stderr.

D052 V2 — `stop` never falls back silently. The per-call re-probe made a busy
cockpit (a 1 s console `check` holds the sim thread, and /api/state waits on
it) look dead, and a stop then went to the in-process sim and answered ok
while the cockpit robot kept walking. Now: the probe is /api/ping (answered
by the event loop, not the sim thread; /api/state for an older cockpit), a
5xx other than 503 counts as alive-but-degraded, and once a cockpit has been
seen, stop goes to IT (with a retry) — the fallback gets it only in
addition, never instead. Every motion result names the backend it ran on.
"""
from __future__ import annotations
import asyncio
import os
import sys
import time

import httpx

DEFAULT_URL = os.environ.get("ROCKY_COCKPIT_URL", "http://127.0.0.1:8765")
AUTO_TTL_S = 3.0
FIND_TIMEOUT_S = 400.0     # find_object: up to 16 looks, each maybe a goto (~10 s) or a turn
GO_BACK_TIMEOUT_S = 200.0  # go_back_to: up to 4 goto legs of <= 40 s each


def cockpit_alive(url=DEFAULT_URL, timeout=2.0):
    """True when a cockpit answers at url. /api/ping first (event loop only);
    404 = an older cockpit -> /api/state. 503 = its sim thread is dead (not
    alive); any other 5xx = alive but degraded (it still takes a stop)."""
    for route in ("/api/ping", "/api/state"):
        try:
            code = httpx.get(f"{url}{route}", timeout=timeout).status_code
        except Exception:
            return False
        if code == 404 and route == "/api/ping":
            continue
        return code == 200 or (code >= 500 and code != 503)
    return False


def _log(msg):
    print(f"[rocky-mcp] {msg}", file=sys.stderr, flush=True)     # stdout is the MCP stream


class CockpitBackend:
    def __init__(self, url=DEFAULT_URL):
        self.url = url.rstrip("/")
        self.client = httpx.AsyncClient(timeout=httpx.Timeout(120.0, connect=5.0))

    async def _tool(self, tool, /, **args):
        # positional-only: gesture's own argument is called `name` (D052 — with
        # `_tool(self, name, **args)` every proxied gesture died in a TypeError)
        try:
            r = await self.client.post(f"{self.url}/api/tool/{tool}", json=args)
            return r.json()
        except httpx.ConnectError as e:              # nothing listens: no cockpit process (D052 V2)
            return {"ok": False, "error": f"cockpit unreachable: {e}", "gone": True}
        except Exception as e:
            return {"ok": False, "error": f"cockpit unreachable: {e}"}

    def live_lists(self):
        """(gesture names, chord words) from the running cockpit, for the MCP
        tool docs; None where the cockpit does not answer."""
        g = w = None
        try:
            g = httpx.get(f"{self.url}/api/gesture/list", timeout=2.0).json().get("all")
            w = httpx.get(f"{self.url}/api/chord/list", timeout=2.0).json().get("lexicon")
        except Exception:
            pass
        return g, w

    async def say(self, word: str) -> dict:
        return await self._tool("say", word=word)

    async def gesture(self, name: str, direction: str | None = None) -> dict:
        if direction:
            return await self._tool("gesture", name=name, direction=direction)
        return await self._tool("gesture", name=name)

    async def goto(self, x: float, y: float) -> dict:
        return await self._tool("goto", x=x, y=y)

    async def stop(self) -> dict:
        return await self._tool("stop")

    async def scan_summary(self) -> dict:
        return await self._tool("scan_summary")

    async def status(self) -> dict:
        return await self._tool("status")

    async def look(self) -> dict:
        return await self._tool("look")

    async def find_object(self, name: str, max_steps: int | None = None) -> dict:
        """Several looks + gotos inside the cockpit: allow FIND_TIMEOUT_S, not 120 s."""
        args = {"name": name, **({"max_steps": max_steps} if max_steps is not None else {})}
        try:
            r = await self.client.post(f"{self.url}/api/tool/find_object", json=args,
                                       timeout=httpx.Timeout(FIND_TIMEOUT_S, connect=5.0))
            return r.json()
        except httpx.ConnectError as e:
            return {"ok": False, "error": f"cockpit unreachable: {e}", "gone": True}
        except Exception as e:
            return {"ok": False, "error": f"cockpit unreachable: {e} (find_object may still be "
                                          "running in the cockpit — call stop to end it)"}

    async def list_gestures(self) -> dict:
        return await self._tool("list_gestures")

    # ---- scene memory (the cockpit's; sim/scene_memory.py)
    async def remember(self, note: str) -> dict:
        return await self._tool("remember", note=note)

    async def where_is(self, name: str) -> dict:
        return await self._tool("where_is", name=name)

    async def recall(self, query: str = "", k: int = 5) -> dict:
        return await self._tool("recall", query=query, k=k)

    async def forget(self, name: str) -> dict:
        return await self._tool("forget", name=name)

    async def go_back_to(self, name: str) -> dict:
        """Up to GO_BACK_MAX_LEGS gotos inside the cockpit: allow GO_BACK_TIMEOUT_S."""
        try:
            r = await self.client.post(f"{self.url}/api/tool/go_back_to", json={"name": name},
                                       timeout=httpx.Timeout(GO_BACK_TIMEOUT_S, connect=5.0))
            return r.json()
        except httpx.ConnectError as e:
            return {"ok": False, "error": f"cockpit unreachable: {e}", "gone": True}
        except Exception as e:
            return {"ok": False, "error": f"cockpit unreachable: {e} (go_back_to may still be "
                                          "walking in the cockpit — call stop to end it)"}


_NO_MEMORY = {"ok": False, "error": "no scene memory here: it lives in the cockpit "
                                    "(./rocky.sh cockpit); the in-process sim has none"}


class AutoBackend:
    """ROCKY_BACKEND=auto, re-resolved per call: the cockpit whenever one
    answers at `url`, else an in-process backend built on first need
    (make_fallback, default SimBackend). Switching is logged to stderr."""

    def __init__(self, url=DEFAULT_URL, make_fallback=None, ttl=AUTO_TTL_S, alive=cockpit_alive):
        self.url = url
        self.cockpit = CockpitBackend(url)
        self._make_fallback = make_fallback
        self._fallback = None
        self._alive = alive
        self._ttl = ttl
        self._seen = (0.0, None)          # (monotonic, alive)
        self.current = None               # "cockpit" | "sim"
        self.cockpit_seen = False         # a cockpit answered at least once (stop then goes to it)

    def _fb(self):
        if self._fallback is None:
            if self._make_fallback is None:
                from harness.sim_backend import SimBackend
                self._fallback = SimBackend()
            else:
                self._fallback = self._make_fallback()
        return self._fallback

    async def pick(self):
        t, alive = self._seen
        if alive is None or time.monotonic() - t > self._ttl:
            alive = await asyncio.to_thread(self._alive, self.url)
            self._seen = (time.monotonic(), alive)
        kind = "cockpit" if alive else "sim"
        if alive:
            self.cockpit_seen = True
        if kind != self.current:
            _log(f"backend -> {kind}" + (f" at {self.url}" if alive else
                                         " (in-process MuJoCo: no cockpit answers at " + self.url + ")"))
            self.current = kind
        return self.cockpit if alive else self._fb()

    def live_lists(self):
        if self._alive(self.url):
            return self.cockpit.live_lists()
        return None, None

    def _tag(self, r):
        """Every motion result says which robot it moved (D052 V2)."""
        return dict(r, backend=self.current) if isinstance(r, dict) else r

    async def say(self, word: str) -> dict:
        return self._tag(await (await self.pick()).say(word))

    async def gesture(self, name: str, direction: str | None = None) -> dict:
        be = await self.pick()
        from harness.intent import _accepts
        if direction and not _accepts(be.gesture, "direction"):
            if direction == "right":
                return self._tag({"ok": False, "error": f"the in-process sim cannot {name} right "
                                                        "(start the cockpit for signed gestures)"})
            return self._tag(await be.gesture(name))
        return self._tag(await (be.gesture(name, direction=direction) if direction else be.gesture(name)))

    async def goto(self, x: float, y: float) -> dict:
        return self._tag(await (await self.pick()).goto(x, y))

    async def stop(self, retries: int = 2) -> dict:
        """Once a cockpit has been seen, the stop goes to IT, whatever the last
        probe said (a busy sim thread only delays /api/tool/stop — it queues
        and runs), retried; the in-process fallback, if one was ever built, is
        stopped as well — in addition, never instead. A stop the cockpit did
        not confirm says so (ok False) rather than claiming success — unless
        nothing listens at the URL any more (connection refused: that process
        and its robot are gone) and the fallback confirmed its own stop."""
        if not self.cockpit_seen:
            be = await self.pick()
            if not self.cockpit_seen:                   # still no cockpit: the fallback is the robot
                return self._tag(await be.stop())
        r = None
        for i in range(max(1, retries + 1)):
            r = await self.cockpit.stop()
            if isinstance(r, dict) and r.get("ok"):
                break
            await asyncio.sleep(0.2 * (i + 1))
        also = await self._fallback.stop() if self._fallback is not None else None
        ok = isinstance(r, dict) and bool(r.get("ok"))
        if (not ok and isinstance(r, dict) and r.get("gone") and isinstance(also, dict)
                and also.get("ok")):
            return dict(also, backend="sim", cockpit="gone (connection refused)")
        if not ok and isinstance(r, dict) and r.get("gone") and self._fallback is None:
            return {"ok": True, "mode": "gone", "backend": "cockpit",
                    "note": "nothing listens at the cockpit URL any more (its process and robot are gone) "
                            "and no in-process sim was ever started — nothing is moving"}
        out = dict(r) if isinstance(r, dict) else {"ok": False, "error": str(r)}
        out.pop("gone", None)
        out["backend"] = "cockpit"
        if not ok:
            out["ok"] = False
            out["error"] = (out.get("error") or "the cockpit did not confirm the stop") + \
                " — the cockpit robot may still be moving; press stop in the cockpit"
            _log(f"STOP NOT CONFIRMED by the cockpit at {self.url}: {out.get('error')}")
        if also is not None:
            out["fallback_stop"] = also
        return out

    async def scan_summary(self) -> dict:
        return await (await self.pick()).scan_summary()

    async def status(self) -> dict:
        r = await (await self.pick()).status()
        if isinstance(r, dict):
            r = dict(r, backend=self.current)
        return r

    async def look(self) -> dict:
        be = await self.pick()
        if not hasattr(be, "look"):
            return {"ok": False, "error": "no eye here: look needs the cockpit "
                                          "(./rocky.sh cockpit); the in-process sim has none"}
        return await be.look()

    async def find_object(self, name: str, max_steps: int | None = None) -> dict:
        be = await self.pick()
        if not hasattr(be, "find_object"):
            return {"ok": False, "error": "no eye here: find_object needs the cockpit "
                                          "(./rocky.sh cockpit); the in-process sim has none"}
        return self._tag(await be.find_object(name, max_steps))

    async def _mem(self, tool, /, **args):
        be = await self.pick()
        if not hasattr(be, tool):
            return dict(_NO_MEMORY)
        return await getattr(be, tool)(**args)

    async def remember(self, note: str) -> dict:
        return await self._mem("remember", note=note)

    async def where_is(self, name: str) -> dict:
        return await self._mem("where_is", name=name)

    async def recall(self, query: str = "", k: int = 5) -> dict:
        return await self._mem("recall", query=query, k=k)

    async def forget(self, name: str) -> dict:
        return await self._mem("forget", name=name)

    async def go_back_to(self, name: str) -> dict:
        return self._tag(await self._mem("go_back_to", name=name))     # it moves: say which robot

    async def list_gestures(self) -> dict:
        be = await self.pick()
        if hasattr(be, "list_gestures"):
            return await be.list_gestures()
        from harness.backend import GESTURES, SIGNED
        return {"ok": True, "gestures": list(GESTURES), "signed": list(SIGNED)}
