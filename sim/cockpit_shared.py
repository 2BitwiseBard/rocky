"""Shared UI state for the cockpit page: one phone drives, another screen follows.

The cockpit's camera view is already global (/api/camera moves the one chase
camera every page streams), so the only thing two browsers need to agree on
is which panel is showing. A device in *lead* mode POSTs its tab / panel here
whenever it changes them; a device in *follow* mode listens on
/api/ui/events and opens the same tab and panel. The robot's name for the
wake word (wake_name) is kept here too: a screen with no name of its own
adopts it, and wake_name(sim) is there for the voice endpoint (which does not
read it yet: harness/intent.py's has_wake_word knows pebble / rocky only).

Deliberately tiny and separate from CockpitSim.snapshot(): an in-memory
dict, a sequence number, its own 4 Hz SSE. Nothing persists across a
restart; an empty state means followers do nothing.

    from cockpit_shared import routes as shared_routes, wake_name
    routes += shared_routes(sim)          # inside cockpit.make_app, before Starlette(...)
    name = wake_name(sim)                 # 'Pebble', or None when nobody set one

Routes:
    GET  /api/ui         -> {ok, state}
    POST /api/ui         {panel?, tab?, view?, remote?, leader_id?, wake_name?}
                         -> {ok, state, errors}   (bad fields are reported, not stored)
    GET  /api/ui/events  -> text/event-stream, a state message whenever seq changes
                            (checked 4x a second), a comment every 10 s otherwise;
                            ends when the client goes away or the sim dies
    GET  /api/guide      -> docs/COCKPIT_GUIDE.md as text/markdown
"""
from __future__ import annotations

import asyncio
import json
import os
import re
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GUIDE_PATH = os.path.join(HERE, "..", "docs", "COCKPIT_GUIDE.md")

TABS = ("drive", "talk", "make", "world", "robot")
VIEWS = ("follow", "wide", "top", "low")
DEFAULT_WAKE = "Pebble"
SSE_HZ = 4.0
KEEPALIVE_S = 10.0

_SLUG = re.compile(r"^[a-z0-9][a-z0-9-]{0,47}$")
_ID = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
_NAME = re.compile(r"^[A-Za-z]{3,24}$")          # one word: the page matches transcripts word by word


def _check(key, v):
    """(value, None) or (None, error) for one POSTed field."""
    if key == "tab":
        return (v, None) if v in TABS else (None, f"tab must be one of {', '.join(TABS)}")
    if key == "panel":
        return (v, None) if isinstance(v, str) and _SLUG.match(v) else (None, "panel must be a slug (a-z, 0-9, -)")
    if key == "view":
        return (v, None) if v in VIEWS else (None, f"view must be one of {', '.join(VIEWS)}")
    if key == "remote":
        return (v, None) if isinstance(v, bool) else (None, "remote must be true or false")
    if key == "leader_id":
        return (v, None) if isinstance(v, str) and _ID.match(v) else (None, "leader_id must be 1-40 of A-Z a-z 0-9 _ -")
    if key == "wake_name":
        if isinstance(v, str) and _NAME.match(v.strip()):
            return v.strip(), None
        return None, "wake_name must be one word of 3-24 letters"
    return None, f"unknown field {key!r}"


class SharedUI:
    """The whole state: a dict, a sequence number and a lock (the SSE and
    the POSTs run on the event loop; the voice endpoint may read from a
    worker thread)."""

    FIELDS = ("panel", "tab", "view", "remote", "leader_id", "wake_name")

    def __init__(self):
        self._lock = threading.Lock()
        self._state = {}
        self.seq = 0
        self.updated = 0.0

    def get(self):
        with self._lock:
            return dict(self._state, seq=self.seq, updated=round(self.updated, 3))

    def update(self, body):
        """Merge the known, valid fields of body; return (state, errors)."""
        errors = []
        if not isinstance(body, dict):
            return self.get(), ["body must be a JSON object"]
        clean = {}
        for k, v in body.items():
            if k not in self.FIELDS:
                errors.append(f"unknown field {k!r}")
                continue
            val, err = _check(k, v)
            if err:
                errors.append(err)
            else:
                clean[k] = val
        if clean:
            with self._lock:
                if any(self._state.get(k) != v for k, v in clean.items()):
                    self._state.update(clean)
                    self.seq += 1
                    self.updated = time.time()
        return self.get(), errors

    def wake_name(self):
        with self._lock:
            return self._state.get("wake_name")


def shared_state(sim):
    """The SharedUI attached to this sim (created on first use)."""
    st = getattr(sim, "ui_shared", None)
    if st is None:
        st = SharedUI()
        try:
            sim.ui_shared = st
        except AttributeError:          # a sim that refuses attributes: keep it module-local
            pass
    return st


def wake_name(sim, default=None):
    """The robot's name as the operator set it on the page, or default."""
    st = getattr(sim, "ui_shared", None)
    return (st.wake_name() if st is not None else None) or default


async def event_stream(state, is_disconnected, alive=lambda: True, hz=SSE_HZ, keepalive_s=KEEPALIVE_S):
    """SSE body: the state now, then again whenever seq moves. Stops when
    the client disconnects or alive() turns false. is_disconnected is an
    async callable (Starlette's request.is_disconnected)."""
    last = None
    quiet_since = time.monotonic()
    period = 1.0 / max(hz, 0.1)
    while alive():
        if await is_disconnected():
            return
        s = state.get()
        if s["seq"] != last:
            last = s["seq"]
            quiet_since = time.monotonic()
            yield f"data: {json.dumps(s)}\n\n"
        elif time.monotonic() - quiet_since > keepalive_s:
            quiet_since = time.monotonic()
            yield ": keepalive\n\n"           # a write to a dead socket is how a vanished phone is noticed
        await asyncio.sleep(period)


def routes(sim, guide_path=None):
    """Starlette Routes for the shared UI state and the guide; mount them in
    cockpit.make_app (the RequestGuard there applies: POST /api/ui needs
    application/json and a same-origin Origin, like every other POST)."""
    from starlette.responses import JSONResponse, Response, StreamingResponse
    from starlette.routing import Route

    state = shared_state(sim)
    gpath = guide_path or GUIDE_PATH

    def alive():
        return getattr(sim, "alive", True) is not False

    async def ui_get(_request):
        return JSONResponse({"ok": True, "state": state.get()})

    async def ui_post(request):
        try:
            body = await request.json()
        except Exception:                                  # noqa: BLE001 — bad JSON is the caller's error
            return JSONResponse({"ok": False, "error": "body must be JSON"}, status_code=400)
        s, errors = state.update(body)
        return JSONResponse({"ok": not errors, "state": s, "errors": errors})

    async def ui_events(request):
        return StreamingResponse(event_stream(state, request.is_disconnected, alive),
                                 media_type="text/event-stream",
                                 headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    async def guide(_request):
        try:
            with open(gpath, encoding="utf-8") as f:
                text = f.read()
        except OSError as e:
            return Response(f"# Guide missing\n\n{gpath}: {e.strerror}\n", status_code=404,
                            media_type="text/markdown; charset=utf-8")
        return Response(text, media_type="text/markdown; charset=utf-8",
                        headers={"Cache-Control": "no-cache"})

    return [
        Route("/api/ui", ui_get), Route("/api/ui", ui_post, methods=["POST"]),
        Route("/api/ui/events", ui_events), Route("/api/guide", guide),
    ]
