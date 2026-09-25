"""cockpit_shared: the shared UI state (lead / follow screens, the wake name)
and the guide route. Pure Starlette, no sim: a stand-in object carries the
state the way CockpitSim will once cockpit.make_app mounts routes(sim).

Pinned: POST merges only valid known fields and reports the rest; seq moves
only on a real change; the SSE generator sends the state, then only changes,
and stops when the client goes away or the sim dies; /api/guide serves the
Markdown and every section the page links to exists in it."""
import asyncio
import json
import os
import re
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
sys.path.insert(0, os.path.join(ROOT, "sim"))

pytest.importorskip("starlette")
from starlette.applications import Starlette                   # noqa: E402
from starlette.testclient import TestClient                     # noqa: E402

import cockpit_shared as cs                                     # noqa: E402


class FakeSim:
    alive = True


def _client(sim=None, guide_path=None):
    sim = sim or FakeSim()
    return sim, TestClient(Starlette(routes=cs.routes(sim, guide_path=guide_path)))


def test_empty_state_then_merge():
    sim, c = _client()
    r = c.get("/api/ui").json()
    assert r["ok"] and r["state"]["seq"] == 0 and "tab" not in r["state"]
    r = c.post("/api/ui", json={"tab": "talk", "panel": "p-brain", "leader_id": "dabc123", "remote": True}).json()
    assert r["ok"] and r["errors"] == []
    st = r["state"]
    assert (st["tab"], st["panel"], st["leader_id"], st["remote"], st["seq"]) == ("talk", "p-brain", "dabc123", True, 1)
    # a partial update keeps the other fields (the wake-name post carries no leader_id)
    st = c.post("/api/ui", json={"wake_name": "Pebble"}).json()["state"]
    assert st["leader_id"] == "dabc123" and st["tab"] == "talk" and st["wake_name"] == "Pebble" and st["seq"] == 2
    assert cs.wake_name(sim) == "Pebble"


def test_seq_moves_only_on_change():
    _, c = _client()
    s1 = c.post("/api/ui", json={"tab": "make"}).json()["state"]["seq"]
    s2 = c.post("/api/ui", json={"tab": "make"}).json()["state"]["seq"]
    s3 = c.post("/api/ui", json={"tab": "world"}).json()["state"]["seq"]
    assert s1 == s2 == 1 and s3 == 2


@pytest.mark.parametrize("body,field", [
    ({"tab": "kitchen"}, "tab"),
    ({"panel": "<script>"}, "panel"),
    ({"panel": "P-Brain"}, "panel"),
    ({"view": "sideways"}, "view"),
    ({"remote": "yes"}, "remote"),
    ({"leader_id": "a b"}, "leader_id"),
    ({"leader_id": "x" * 41}, "leader_id"),
    ({"wake_name": "R2-D2"}, "wake_name"),
    ({"wake_name": "<b>"}, "wake_name"),
    ({"wake_name": "x" * 25}, "wake_name"),
    ({"wake_name": "Bo"}, "wake_name"),                # too short: every 'bow' / 'box' would pass
    ({"wake_name": "Mr Pebble"}, "wake_name"),         # one word only
    ({"wake_name": "O'Neil"}, "wake_name"),
    ({"colour": "red"}, "colour"),
])
def test_bad_fields_are_reported_not_stored(body, field):
    _, c = _client()
    r = c.post("/api/ui", json=body).json()
    assert r["ok"] is False and any(field in e for e in r["errors"]), r
    assert field not in r["state"] and r["state"]["seq"] == 0


def test_not_json_and_not_object():
    _, c = _client()
    r = c.post("/api/ui", content=b"{nope", headers={"Content-Type": "application/json"})
    assert r.status_code == 400 and r.json()["ok"] is False
    r = c.post("/api/ui", json=[1, 2]).json()
    assert r["ok"] is False and r["state"]["seq"] == 0


def test_wake_name_default_and_trim():
    sim = FakeSim()
    assert cs.wake_name(sim, "Pebble") == "Pebble"           # nobody set one yet
    st = cs.shared_state(sim)
    st.update({"wake_name": "  Rocky  "})
    assert cs.wake_name(sim) == "Rocky"
    assert cs.shared_state(sim) is st                        # one state per sim


def test_sse_sends_state_then_stops_on_disconnect():
    st = cs.SharedUI()
    st.update({"tab": "robot", "leader_id": "d1"})
    calls = {"n": 0}

    async def gone_after_three():
        calls["n"] += 1
        return calls["n"] > 3

    async def go():
        return [c async for c in cs.event_stream(st, gone_after_three, hz=200)]
    chunks = asyncio.run(go())
    assert len(chunks) == 1 and chunks[0].startswith("data: ")      # the state once, no repeats
    assert json.loads(chunks[0][6:])["tab"] == "robot"
    assert calls["n"] == 4                                           # it asked, and it stopped


def test_sse_sends_changes_and_stops_when_sim_dies():
    st = cs.SharedUI()
    alive = {"v": True}
    ticks = {"n": 0}

    async def never_gone():
        ticks["n"] += 1
        if ticks["n"] == 3:
            st.update({"tab": "world"})              # a leader changes tab mid-stream
        if ticks["n"] == 6:
            alive["v"] = False                       # the sim thread stops
        return False

    async def go():
        return [c async for c in cs.event_stream(st, never_gone, alive=lambda: alive["v"], hz=200)]
    chunks = asyncio.run(go())
    data = [json.loads(c[6:]) for c in chunks if c.startswith("data: ")]
    assert [d["seq"] for d in data] == [0, 1] and data[1]["tab"] == "world"


def test_sse_keepalive_when_quiet():
    st = cs.SharedUI()
    n = {"v": 0}

    async def gone_late():
        n["v"] += 1
        return n["v"] > 30

    async def go():
        return [c async for c in cs.event_stream(st, gone_late, hz=500, keepalive_s=0.01)]
    chunks = asyncio.run(go())
    assert chunks[0].startswith("data: ") and any(c.startswith(": keepalive") for c in chunks[1:])


def test_events_route_is_an_event_stream():
    """The route itself (the generator is covered above): a dead sim ends the
    stream at once, so the TestClient read terminates."""
    sim = FakeSim()
    sim.alive = False
    _, c = _client(sim)
    r = c.get("/api/ui/events")
    assert r.status_code == 200 and r.headers["content-type"].startswith("text/event-stream")
    assert r.text == ""


def test_guide_route_serves_markdown(tmp_path):
    _, c = _client()
    r = c.get("/api/guide")
    assert r.status_code == 200 and r.headers["content-type"].startswith("text/markdown")
    assert r.text.startswith("# Pebble cockpit guide")
    _, c2 = _client(guide_path=str(tmp_path / "missing.md"))
    r = c2.get("/api/guide")
    assert r.status_code == 404 and "Guide missing" in r.text


def _slug(t):
    """The page's slug(): lower case, backticks / asterisks dropped, runs of
    anything else become one '-'."""
    t = re.sub(r"[`*]", "", t.lower())
    return re.sub(r"[^a-z0-9]+", "-", t).strip("-")


def test_guide_has_every_section_the_page_links_to():
    guide = open(cs.GUIDE_PATH, encoding="utf-8").read()
    heads = {_slug(m) for m in re.findall(r"^#{1,4}\s+(.*)$", guide, re.M)}
    for need in ("first-five-minutes", "driving", "talking", "the-wake-word", "making-gestures",
                 "the-feasibility-verdict", "sounds", "worlds-and-guards", "rl-panel",
                 "hardware-bring-up", "phone-and-remote", "troubleshooting"):
        assert need in heads, need
    page = open(os.path.join(ROOT, "sim", "cockpit_ui.html"), encoding="utf-8").read()
    linked = set(re.findall(r'data-guide="([^"]+)"', page)) | set(re.findall(r"showGuide\('([a-z0-9-]+)'\)", page))
    assert linked and linked <= heads, linked - heads
    anchors = set(re.findall(r"\]\(#([^)]+)\)", guide))                # links inside the guide
    assert {_slug(a) for a in anchors} <= heads


def test_guide_names_the_tailnet_address_and_every_verdict_code():
    guide = open(cs.GUIDE_PATH, encoding="utf-8").read()
    assert "https://ai-hub.tail54f481.ts.net:9445" in guide
    sys.path.insert(0, os.path.join(ROOT, "gait"))
    pf = pytest.importorskip("pebble_feasibility")
    for code in pf.FAIL_CODES + pf.WARN_CODES:
        assert f"`{code}`" in guide, code
