"""CockpitBackend — the MCP contract proxied to a running cockpit
(sim/cockpit.py) over HTTP, so `./rocky.sh chat` (Claude Code over MCP),
local_brain and intent all drive the ONE sim the browser is showing.

    ROCKY_BACKEND=cockpit  ROCKY_COCKPIT_URL=http://127.0.0.1:8765
    ROCKY_BACKEND=auto     -> cockpit if one answers, else the in-process sim

Adds the `look` tool (the robot's eye through a vision model), which only
exists where a cockpit is running — absent tool > lying tool.
"""
from __future__ import annotations
import os

import httpx

DEFAULT_URL = os.environ.get("ROCKY_COCKPIT_URL", "http://127.0.0.1:8765")


def cockpit_alive(url=DEFAULT_URL, timeout=1.0):
    try:
        return httpx.get(f"{url}/api/state", timeout=timeout).status_code == 200
    except Exception:
        return False


class CockpitBackend:
    def __init__(self, url=DEFAULT_URL):
        self.url = url.rstrip("/")
        self.client = httpx.AsyncClient(timeout=httpx.Timeout(120.0, connect=5.0))

    async def _tool(self, name, **args):
        try:
            r = await self.client.post(f"{self.url}/api/tool/{name}", json=args)
            return r.json()
        except Exception as e:
            return {"ok": False, "error": f"cockpit unreachable: {e}"}

    async def say(self, word: str) -> dict:
        return await self._tool("say", word=word)

    async def gesture(self, name: str) -> dict:
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
