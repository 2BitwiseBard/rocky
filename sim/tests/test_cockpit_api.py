"""D052 cockpit HTTP API — the server the browser (and the MCP proxy) talks
to, driven through Starlette's TestClient against a real headless CockpitSim
(MuJoCo physics running in its own thread, rendering off, righter off).

What is pinned here: the heartbeat and the fatal path (a sim thread that
dies fails every waiting request instead of hanging it), the request guard
(same-origin JSON POSTs only, Host allow-list, no non-loopback bind without
--unsafe-lan), the gesture studio's feasibility gate (check / save / play
refuse a FAIL), the REACH solver and TEACH recorder endpoints, the hardware
panel's sim2real idle rule on the mock bus and the model panel's
fingerprint. The goto reactive layer (lidar detour -> 'blocked') is a slow
test: it walks the robot into a wall."""
import json
import os
import signal
import subprocess
import sys
import threading
import time
import warnings

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
for sub in ("sim", "gait", "perception"):
    sys.path.insert(0, os.path.join(ROOT, sub))

pytest.importorskip("starlette")
from starlette.testclient import TestClient                     # noqa: E402

GOOD = {"name": "t_good", "keyframes": [{"t": 0.8, "body": [0, 0, -10]}, {"t": 1.6, "body": [0, 0, 0]}]}
# an arm snapped up in 0.1 s: ~29 rad/s at the hip, six times the servo's no-load speed
BAD = {"name": "t_bad_d052", "keyframes": [{"t": 0.1, "arm": {"0": [0, 80, -40]}}]}


def _make_sim(world="flat"):
    import cockpit
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        sim = cockpit.CockpitSim(world)
    sim.do("righter off")                 # torch-free, deterministic
    sim.render_every = 10 ** 9            # no GL
    sim.brains.conf_path = None           # never touch ~/.config/rocky/cockpit.json
    return sim


def _start(sim):
    threading.Thread(target=sim.run_forever, daemon=True).start()
    import cockpit
    return TestClient(cockpit.make_app(sim, extra_hosts=("testserver",)))


def _wait(pred, timeout=15.0, dt=0.05):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if pred():
            return True
        time.sleep(dt)
    return False


@pytest.fixture(scope="module")
def cockpit_client():
    sim = _make_sim()
    sim.speed = 4.0
    client = _start(sim)
    yield sim, client
    try:
        sim.hw_disconnect()
    finally:
        sim.alive = False


def _idle(client):
    return _wait(lambda: client.get("/api/state").json().get("idle"), timeout=20.0, dt=0.1)


# ------------------------------------------------------------------ state + heartbeat
def test_state_has_heartbeat_servo_and_no_hw(cockpit_client):
    sim, c = cockpit_client
    s = c.get("/api/state").json()
    hb = s["heartbeat"]
    assert hb["alive"] is True and hb["fatal"] is None
    assert hb["step_age_s"] < 2.0 and hb["loop_age_s"] < 2.0
    assert s["servo"]["on"] is True                  # D052: servo realism is the default
    assert s["hw"] is None
    assert "budget_note" in s["guards"] and s["guards"]["void"] is None
    assert s["fingerprint"] == sim.fingerprint


def test_model_panel_has_fingerprint_and_limits(cockpit_client):
    import rocky_model as rm
    from model_fingerprint import robot_fingerprint
    sim, c = cockpit_client
    m = c.get("/api/model").json()
    assert m["fingerprint"] == robot_fingerprint(sim.model)
    assert m["params_rev"] == rm.params_rev()
    assert m["limits_deg"]["knee"] == list(rm.joint_limits_deg()["knee"])
    assert m["speeds"]["loaded"] == rm.servo_speed("loaded")
    assert m["envelope"]["v"] > 0 and m["studio"]["yaw_deg"] > 0
    assert len(m["leg_ids"]) == 5


# ------------------------------------------------------------------ gesture studio gate
def test_gesture_check_good_and_bad(cockpit_client):
    _, c = cockpit_client
    g = c.post("/api/gesture/check", json=GOOD).json()
    assert g["feasible"] is True and g["verdict"].startswith("PASS")
    b = c.post("/api/gesture/check", json={"spec": BAD}).json()
    assert b["feasible"] is False and b["verdict"].startswith("FAIL")
    assert any(f.startswith("SPEED") for f in b["fails"])
    assert b["summary"]["peak_rad_s"] > 4.7 and b["summary"]["peak_joint"] == "hip"
    m = c.post("/api/gesture/check", json={"keyframes": "nope"}).json()   # malformed: a verdict, not a 500
    assert m["feasible"] is False


def test_gesture_save_and_play_refuse_a_fail(cockpit_client, tmp_path, monkeypatch):
    import cockpit
    monkeypatch.setattr(cockpit, "GESTURE_DIR", str(tmp_path))   # a regression must not write into gait/gestures/
    _, c = cockpit_client
    path = os.path.join(str(tmp_path), BAD["name"] + ".json")
    assert not os.path.exists(path)
    r = c.post("/api/gesture/save", json=BAD).json()
    assert r["ok"] is False and "not feasible" in r["error"] and r["check"]["feasible"] is False
    assert not os.path.exists(path)
    r = c.post("/api/gesture/play", json=BAD).json()
    assert r["ok"] is False and "not feasible" in r["error"]
    r = c.post("/api/gesture/save", json=dict(GOOD, name="wave")).json()    # never over a code gesture
    assert r["ok"] is False and "built-in" in r["error"]
    assert os.listdir(str(tmp_path)) == []


def test_reach_solver_endpoint(cockpit_client):
    _, c = cockpit_client
    r = c.post("/api/gesture/solve", json={"leg": 0, "target": [0, 250, 60]}).json()
    assert r["ok"] is True and "0" in r["frame"]["arm"]
    assert r["margin"] >= 25.0 - 1e-6
    bad = c.post("/api/gesture/solve", json={"leg": 7, "target": [0, 1, 2]}).json()
    assert bad["ok"] is False


def test_teach_records_a_pose_stream_into_keyframes(cockpit_client):
    sim, c = cockpit_client
    assert _idle(c)
    assert c.post("/api/gesture/teach", json={"action": "start"}).json()["ok"]
    pose = {"keyframes": [{"t": 0, "body": [0, 0, -20]}]}
    assert c.post("/api/gesture/preview", json={"spec": pose, "t": 0}).json()["ok"]
    assert _wait(lambda: sim.teach is not None and len(sim.teach["qs"]) >= 30, timeout=20.0)
    r = c.post("/api/gesture/teach", json={"action": "stop", "name": "taught_test"}).json()
    c.post("/api/gesture/preview", json={"off": True})
    assert r["ok"] is True, r
    assert r["meta"]["samples"] >= 30 and len(r["spec"]["keyframes"]) >= 1
    assert "taught" in r["spec"] and r["check"]["verdict"]


# ------------------------------------------------------------------ tools + guards
def test_goto_with_a_bad_argument_is_an_answer_not_a_500(cockpit_client):
    _, c = cockpit_client
    r = c.post("/api/tool/goto", json={"x": "here", "y": 0})
    assert r.status_code == 200 and r.json()["ok"] is False


def test_request_guard_origin_content_type_and_host(cockpit_client):
    _, c = cockpit_client
    ok = {"line": "show"}
    assert c.post("/api/cmd", json=ok).status_code == 200
    assert c.post("/api/cmd", json=ok, headers={"Origin": "http://testserver"}).status_code == 200
    assert c.post("/api/cmd", json=ok, headers={"Origin": "http://evil.example"}).status_code == 403
    assert c.post("/api/cmd", json=ok, headers={"Origin": "null"}).status_code == 403
    assert c.post("/api/cmd", content="line=walk 45",
                  headers={"Content-Type": "application/x-www-form-urlencoded"}).status_code == 415
    assert c.post("/api/cmd", content='{"line": "walk 45"}', headers={"Content-Type": "text/plain"}).status_code == 415
    # tailscale serve keeps the public Host: same-origin through the proxy passes
    ts = {"Host": "pebble-box.tail54f481.ts.net:9445", "Origin": "https://pebble-box.tail54f481.ts.net:9445"}
    assert c.post("/api/gesture/check", json=GOOD, headers=ts).status_code == 200
    # DNS rebinding: evil.example resolving to 127.0.0.1 sends Host = Origin = evil.example
    reb = {"Host": "evil.example:8765", "Origin": "http://evil.example:8765"}
    assert c.post("/api/cmd", json=ok, headers=reb).status_code == 403
    assert c.get("/api/state", headers={"Host": "evil.example:8765"}).status_code == 403


def test_request_refusal_rules():
    from cockpit import request_refusal, is_loopback
    j = {"content-type": "application/json"}
    assert request_refusal("POST", "/api/cmd", {"host": "127.0.0.1:8765", **j}) is None
    assert request_refusal("POST", "/api/cmd", {"host": "localhost:8765", "origin": "http://localhost:8765", **j}) is None
    assert request_refusal("POST", "/api/cmd", {"host": "localhost:8765", "origin": "http://localhost:8766", **j})[0] == 403
    assert request_refusal("POST", "/api/cmd", {"host": "x.ts.net", "origin": "https://x.ts.net", **j}) is None
    assert request_refusal("POST", "/api/voice", {"host": "127.0.0.1", "content-type": "multipart/form-data; b=1"}) is None
    assert request_refusal("POST", "/api/voice", {"host": "127.0.0.1", **j})[0] == 415
    assert request_refusal("GET", "/api/state", {"host": "[::1]:8765"}) is None
    assert request_refusal("GET", "/api/state", {"host": "evil.example"})[0] == 403
    assert request_refusal("GET", "/api/state", {"host": "evil.example"}, check_host=False) is None
    assert is_loopback("127.0.0.1") and is_loopback("::1") and is_loopback("localhost")
    assert not is_loopback("0.0.0.0") and not is_loopback("192.168.1.5")


def test_non_loopback_host_needs_unsafe_lan(monkeypatch):
    import cockpit
    import uvicorn

    class Sentinel(Exception):
        pass

    def boom(*_a, **_k):
        raise Sentinel("the guard let a non-loopback bind through")
    # if the guard regresses, fail HERE instead of serving an unauthenticated cockpit on 0.0.0.0
    monkeypatch.setattr(cockpit, "CockpitSim", boom)
    monkeypatch.setattr(uvicorn, "Server", boom)
    monkeypatch.setattr(uvicorn, "run", boom)
    with pytest.raises(SystemExit) as e:
        cockpit.main(["--host", "0.0.0.0", "--port", "8799"])
    assert e.value.code == 2


def test_walk_refusal_reaches_the_overlay(cockpit_client):
    sim, c = cockpit_client
    assert _idle(c)
    sim.do("walk 30")
    try:
        sim.sup.latched = True                       # as 3 gyro trips in 5 s would
        sim.sup.latch_reason = "test latch"
        c.post("/api/cmd", json={"line": "stop"})
        r = c.post("/api/cmd", json={"line": "walk 30"}).json()
        assert r["reply"].startswith("blocked: latched")
        assert "latched" in (c.get("/api/state").json()["refusal"] or "")
    finally:
        sim.sup.clear_latch()
        c.post("/api/cmd", json={"line": "stop"})


# ------------------------------------------------------------------ hardware (mock bus)
def test_hw_mock_sim2real_needs_idle(cockpit_client):
    sim, c = cockpit_client
    assert c.post("/api/hw", json={"action": "connect", "port": "mock"}).json()["ok"]
    try:
        s = c.post("/api/hw", json={"action": "scan"}).json()
        assert s["ok"] and all(s["status"]["legs"])
        assert s["status"]["speed_cps"] == 200       # the stream starts gentle
        assert _idle(c)
        assert c.post("/api/cmd", json={"line": "walk 30"}).json()["reply"].startswith("walking")
        r = c.post("/api/hw", json={"action": "mirror", "mode": "sim2real"}).json()
        assert r["ok"] is False and "walking" in r["error"]
        c.post("/api/cmd", json={"line": "stop"})
        assert _idle(c)
        r = c.post("/api/hw", json={"action": "mirror", "mode": "sim2real"}).json()
        assert r["ok"] is False and "1x" in r["error"]          # V2: the fixture runs the sim at 4x
        assert c.post("/api/speed", json={"speed": 1.0}).json()["speed"] == 1.0
        r = c.post("/api/hw", json={"action": "mirror", "mode": "sim2real"}).json()
        assert r["ok"] is True and r["mirror"] == "sim2real", r
        hw = c.get("/api/state").json()["hw"]
        assert hw["mirror"] == "sim2real" and hw["allow_locomotion"] is False
        r = c.post("/api/cmd", json={"line": "walk 30"}).json()
        assert r["reply"].startswith("locomotion held")
        # V2: nothing sim-only reaches the real legs while mirroring
        assert c.post("/api/speed", json={"speed": 4.0}).status_code == 400
        assert c.post("/api/shove", json={"fx": 60, "fy": 0}).status_code == 409
        assert _wait(lambda: c.get("/api/hw").json()["status"]["entry"] is None, timeout=10.0)
        r = c.post("/api/tool/gesture", json={"name": "turn_in_place"}).json()
        assert r["ok"] is False and "locomotion held" in r["hint"], r
        plan = c.post("/api/hw", json={"action": "limits", "dry_run": True}).json()
        assert plan["ok"] is False and "mirror off" in plan["error"]
        assert c.post("/api/hw", json={"action": "mirror", "mode": "off"}).json()["ok"]
        plan = c.post("/api/hw", json={"action": "limits", "dry_run": True}).json()
        assert plan["ok"] and plan["dry_run"] and len(plan["result"]) == 20
    finally:
        c.post("/api/hw", json={"action": "disconnect"})
        c.post("/api/cmd", json={"line": "stop"})
        c.post("/api/speed", json={"speed": 4.0})


def test_bad_numbers_never_wedge_the_cockpit(cockpit_client):
    """V2 review repros: `set gait.T 0` made /api/state 500 for good (the sim kept
    running blind); speed NaN unthrottled the loop until restart; a 1e9 N shove
    blew MuJoCo up."""
    sim, c = cockpit_client
    assert c.get("/api/ping").json()["ok"] is True
    T0 = sim.gait.T
    r = c.post("/api/cmd", json={"line": "set gait.T 0"}).json()
    assert "refused" in r["reply"] and sim.gait.T == T0
    assert c.get("/api/state").status_code == 200
    r = c.post("/api/speed", content=b'{"speed": NaN}', headers={"content-type": "application/json"})
    assert r.status_code == 400 and sim.speed == 4.0
    r = c.post("/api/shove", content=b'{"fx": NaN, "fy": 0}', headers={"content-type": "application/json"})
    assert r.status_code == 400
    from playground import shove_args
    assert shove_args(1e9, 0.0, 0.01) == (200.0, 0.0, 0.05)          # clamped, not refused
    assert shove_args(-30.0, 40.0, 0.4) == (-30.0, 40.0, 0.4)
    sim.gait.T = 0.0                                   # a value that slipped past validation anyway
    try:
        s = c.get("/api/state")
        assert s.status_code == 200
    finally:
        sim.gait.T = T0
    c.post("/api/cmd", json={"line": "stop"})


# ------------------------------------------------------------------ hygiene (review items, 2026-09-25)
EYE = "a red chair two metres ahead"


def _plant_eye(sim):
    """What a look and a find_object leave on the brains (no model call)."""
    now = time.time()
    sim.brains.last_look = {"text": EYE, "model": "fake-vl", "t": now, "pose": dict(x=0.0, y=0.0, yaw_deg=0.0)}
    sim.brains.last_find = {"name": "ball", "found": True, "detail": "0.40 m ahead", "t": now - 5.0,
                            "model": "fake-vl", "looks": 1}


def _situation(c):
    """A situation composed NOW (in the sim thread), as a chat turn gets it."""
    r = c.post("/api/awareness", json={"refresh": True}).json()
    assert r["ok"], r
    return r["situation"]["text"]


def test_a_world_change_or_reset_forgets_the_eye_context(cockpit_client):
    """Review item: last_look / last_find outlived a world swap, so the next
    chat turn's situation line described a world that no longer existed."""
    sim, c = cockpit_client
    b = sim.brains
    try:
        _plant_eye(sim)
        assert EYE in _situation(c)                                  # the eye clause quotes the last look
        assert EYE in (c.get("/api/state").json()["situation"]["text"] or "")
        assert c.post("/api/world", json={"preset": "room"}).json()["ok"]           # another world
        assert b.last_look is None and b.last_find is None
        assert EYE not in (c.get("/api/state").json()["situation"]["text"] or "")    # the stored line too
        text = _situation(c)
        assert EYE not in text and "eye: no description yet" in text and "'room'" in text
        for how in ("reset", "reload", "edit"):                      # same world: the scene still changed
            _plant_eye(sim)
            assert EYE in _situation(c), how
            if how == "reset":
                assert c.post("/api/reset", json={}).json()["reply"].startswith("reset")
            elif how == "reload":
                assert c.post("/api/world", json={"preset": "room"}).json()["ok"]
            else:
                assert c.post("/api/world/add", json={"object": {"kind": "box", "pos": [1.6, 0.8],
                                                                  "size": [0.1, 0.1, 0.05]}}).json()["ok"]
            assert b.last_look is None and b.last_find is None, how
            assert "eye: no description yet" in _situation(c), how
    finally:
        b.last_look = b.last_find = None
        assert c.post("/api/world", json={"preset": "flat"}).json()["ok"]            # as the fixture had it


OLD_FRAME = b"OLD-WORLD-FRAME"


def _fake_vision(monkeypatch, b):
    """A vision chain of one fake model that echoes the eye frame's bytes (no
    catalog, no network): a look describes exactly the frame it was sent."""
    import base64

    async def acatalog():
        return {}

    async def acall(model, msgs, tools, max_tokens):
        url = msgs[0]["content"][1]["image_url"]["url"]
        return {"content": "I see " + base64.b64decode(url.split(",", 1)[1]).decode(errors="replace")}

    monkeypatch.setattr(b, "_acatalog", acatalog)
    monkeypatch.setattr(b, "chain", lambda role, first=None, cat=None: (["fake-vl"], []))
    monkeypatch.setattr(b, "_acall", acall)


def test_a_world_change_or_reset_drops_the_old_eye_frame(cockpit_client, monkeypatch):
    """Review item: _forget_scene dropped the eye's description but kept its
    FRAME, which only a render replaces — and nothing renders while paused,
    with the cameras off, or for up to 1/15 s on the cadence — so the next
    look described the world that was gone. The frame now goes too."""
    sim, c = cockpit_client
    b = sim.brains
    _fake_vision(monkeypatch, b)
    try:
        for how, paused in (("world", True), ("reset", False), ("edit", False)):
            c.post("/api/speed", json={"paused": paused})
            sim.frames["eye"] = (7, OLD_FRAME)
            r = c.post("/api/look", json={}).json()
            assert r["ok"] and r["description"] == "I see OLD-WORLD-FRAME", (how, r)   # the fake is wired
            b.last_look = None
            if how == "world":
                assert c.post("/api/world", json={"preset": "room"}).json()["ok"]
            elif how == "reset":
                assert c.post("/api/reset", json={}).json()["reply"].startswith("reset")
            else:
                assert c.post("/api/world/add", json={"object": {"kind": "box", "pos": [1.6, 0.8],
                                                                  "size": [0.1, 0.1, 0.05]}}).json()["ok"]
            n, jpg = sim.frames["eye"]
            assert jpg == b"" and n > 7, how                           # empty, counter still monotonic
            r = c.post("/api/look", json={}).json()
            assert r["ok"] is False and "no eye frame yet" in r["error"], (how, r)
            assert b.last_look is None, how
            f = c.post("/api/look", json={"find": "ball"}).json()      # find_object's detector too
            assert f["ok"] is False and "no eye frame yet" in f["error"], (how, f)
            assert c.get("/frame/eye.jpg").content == b"", how
            assert "eye: no description yet" in _situation(c), how
    finally:
        c.post("/api/speed", json={"paused": False})
        b.last_look = b.last_find = None
        assert c.post("/api/world", json={"preset": "flat"}).json()["ok"]            # as the fixture had it


def test_live_cameras_render_the_new_scene_at_once_even_paused(cockpit_client, monkeypatch):
    """When the cameras work, a world load / reset does not leave the eye
    empty until the cadence (or forever, paused): a render is due on the next
    loop turn. A GL-free sim (no renderer ever built) never renders for it."""
    sim, c = cockpit_client
    renders = []

    def fake_render():                                   # sim thread; stands in for the GL renderer
        renders.append(sim.world_name)
        for cam in ("chase", "eye"):
            n = sim.frames[cam][0] + 1
            sim.frames[cam] = (n, f"NEW:{sim.world_name}".encode())

    monkeypatch.setattr(sim, "_render", fake_render)
    try:
        # GL-free (the fixture's state): the frame is dropped and nothing renders
        assert sim._cams_live is False
        assert c.post("/api/world", json={"preset": "room"}).json()["ok"]
        assert sim.frames["eye"][1] == b"" and renders == [] and sim._render_due is False
        # live cameras, paused: the new world is on both cameras without a single step
        monkeypatch.setattr(sim, "_cams_live", True)
        c.post("/api/speed", json={"paused": True})
        assert c.get("/api/state").status_code == 200    # a sim-thread round trip: it is parked now
        sim.frames["eye"] = (sim.frames["eye"][0], OLD_FRAME)
        k0 = sim._k
        assert c.post("/api/world", json={"preset": "obstacle course"}).json()["ok"]
        assert _wait(lambda: sim.frames["eye"][1] == b"NEW:obstacle course", timeout=5.0), sim.frames["eye"]
        assert sim.frames["chase"][1] == b"NEW:obstacle course"
        assert sim._k == k0 and renders == ["obstacle course"]      # paused: rendered once, no physics step
        time.sleep(0.2)
        assert renders == ["obstacle course"]                       # once, not every paused turn
        # running: the reset's render comes on the next step, not on the (here: never) cadence
        c.post("/api/speed", json={"paused": False})
        assert c.post("/api/reset", json={}).json()["reply"].startswith("reset")
        assert _wait(lambda: len(renders) == 2, timeout=5.0), renders
        assert sim.frames["eye"][1] == b"NEW:obstacle course"
    finally:
        c.post("/api/speed", json={"paused": False})
        monkeypatch.setattr(sim, "_cams_live", False)
        sim._render_due = False
        assert c.post("/api/world", json={"preset": "flat"}).json()["ok"]            # as the fixture had it
        sim.frames["eye"] = (sim.frames["eye"][0] + 1, b"")
        sim.frames["chase"] = (sim.frames["chase"][0] + 1, b"")


def test_event_seq_counts_every_event(cockpit_client, monkeypatch):
    """Review item: a client could not tell a new event from a 12-event window
    that looks the same (brain_bench's documented blind spot)."""
    import cockpit
    sim, c = cockpit_client
    s0 = c.get("/api/state").json()
    assert isinstance(s0["event_seq"], int) and s0["event_seq"] >= len(s0["events"])
    probe = ("say", "event_seq_probe")
    sim.events.append(probe)                          # as the bridge's thread or the brains would
    s1 = c.get("/api/state").json()
    sim.events.append(probe)
    s2 = c.get("/api/state").json()
    assert s0["event_seq"] < s1["event_seq"] < s2["event_seq"]
    assert s2["event_seq"] - s0["event_seq"] >= 2
    assert list(probe) in s1["events"] and s2["events"][-2:] == [list(probe)] * 2
    # the SSE feed is the same snapshot: its first message carries it (a SimDead
    # on the second call ends the stream, so the TestClient read terminates)
    real, n = sim.call, {"k": 0}

    async def once(fn):
        n["k"] += 1
        if n["k"] > 1:
            raise cockpit.SimDead("test: end the stream")
        return await real(fn)
    monkeypatch.setattr(sim, "call", once)
    r = c.get("/api/events")
    first = json.loads(r.text.split("\n\n")[0][len("data: "):])
    assert first["alive"] is True and first["event_seq"] >= s2["event_seq"]


def test_event_log_sees_a_new_event_the_window_hides():
    import copy
    from cockpit import EventLog
    ev = EventLog(maxlen=3)
    for _ in range(5):
        ev.append(("say", "yes"))
    seq1, w1 = ev.tail(12)
    ev.append(("say", "yes"))
    seq2, w2 = ev.tail(12)
    assert w1 == w2 == [("say", "yes")] * 3 and (seq1, seq2) == (5, 6)
    ev.extend([("a", 1), ("b", 2)])
    assert ev.seq == 8 and list(ev) == [("say", "yes"), ("a", 1), ("b", 2)] and ev.tail(0) == (8, [])
    cp = copy.copy(ev)                                # deque's copy calls type(d)(d, maxlen)
    assert isinstance(cp, EventLog) and list(cp) == list(ev) and cp.maxlen == 3
    big = EventLog(maxlen=10)                         # appends from several threads: none lost
    ts = [threading.Thread(target=lambda: [big.append(i) for i in range(2000)]) for _ in range(4)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()
    assert big.seq == 8000 and len(big) == 10


# ------------------------------------------------------------------ D056: capabilities
CAPS_KEYS = {"format", "version", "capabilities", "gestures", "signed", "lexicon", "envelope", "robot",
             "tools"}


def _caps(c):
    return c.get("/api/capabilities").json()


def _new_events(sim, seq0):
    """The events appended since sim.events.seq was seq0 (all of them, not the 12-window)."""
    seq, ev = sim.events.tail(sim.events.seq)
    return ev[len(ev) - (seq - seq0):] if seq > seq0 else []


def _caps_events(sim, seq0):
    """Capabilities entries on the events feed since seq0: there must never be any (the
    status tool hands every model that feed's last five entries; D056 review)."""
    return [e for e in _new_events(sim, seq0) if e[0] == "capabilities"]


def _caps_state(c):
    """(caps_version, caps_seq) as the state feed (/api/state, /api/events) carries them."""
    s = c.get("/api/state").json()
    return s["caps_version"], s["caps_seq"]


def test_recognition_on_offers_the_place_tools_and_off_is_the_d056_list_again(cockpit_client):
    """D057 review: with place recognition off, no model is offered a place tool (they could
    only answer 'recognition is off'); on, the four come after the D056 tools, the version
    moves once each way and the snapshot off is the one before, byte for byte."""
    import cockpit_brains as cb
    import harness.capabilities as C
    sim, c = cockpit_client
    a = _caps(c)
    v0, n0 = _caps_state(c)
    assert v0 == a["version"] and not set(cb.PLACE_TOOLS) & {t["name"] for t in a["tools"]}
    names0 = [t["function"]["name"] for t in sim.brains.tools_for(look=True)]
    assert not set(cb.PLACE_TOOLS) & set(names0)
    assert "Place recognition is on" not in sim.brains.system_for()
    try:
        assert c.post("/api/awareness", json={"recognize": True}).json()["awareness"]["recognize"] is True
        on = _caps(c)
        assert on["capabilities"] == ["cockpit", "eye", "memory", "places"] and on["version"] != a["version"]
        assert [t["name"] for t in on["tools"]] == list(C.TOOL_NAMES)
        assert _caps_state(c) == (on["version"], n0 + 1)
        assert C.to_openai_tools(on) == sim.brains.tools_for(look=True)
        assert [t["function"]["name"] for t in sim.brains.tools_for(look=True)] == names0 + list(cb.PLACE_TOOLS)
        assert "Place recognition is on" in sim.brains.system_for()
        assert cb.registry_problems(sim.brains, dict(cb.cockpit_flags(sim), has_places=True)) == []
    finally:
        c.post("/api/awareness", json={"recognize": False})
    assert _caps(c) == a and _caps_state(c) == (a["version"], n0 + 2)
    assert [t["function"]["name"] for t in sim.brains.tools_for(look=True)] == names0


def test_capabilities_endpoint_shape_and_a_stable_version(cockpit_client):
    import cockpit
    import cockpit_brains as cb
    import harness.capabilities as C
    from harness.backend import SIGNED
    sim, c = cockpit_client
    a, b = _caps(c), _caps(c)
    assert a == b and set(a) == CAPS_KEYS and a["format"] == C.FORMAT
    assert len(a["version"]) == 12 and a["version"] == C.snapshot_version(a)
    assert a["capabilities"] == ["cockpit", "eye", "memory"]
    assert a["gestures"] == sim.gesture_names and a["lexicon"] == sim.lexicon and a["signed"] == list(SIGNED)
    tools = {t["name"]: t for t in a["tools"]}
    # recognition off (the default): the D056 set exactly — TOOL_NAMES, pinned in test_cockpit_brains
    # (D057's place tools need the registry's `places` capability: recognition on, see below)
    assert list(tools) == [n for n in C.TOOL_NAMES if n not in cb.PLACE_TOOLS] and set(tools) == set(cb.TOOL_NAMES)
    assert {n for n, t in tools.items() if t["gated"]} == set(cb.GATED)
    assert tools["gesture"]["parameters"]["properties"]["name"]["enum"] == sim.gesture_names
    assert tools["say"]["parameters"]["properties"]["word"]["enum"] == sim.lexicon
    assert tools["turn"]["surfaces"] == ["internal"]
    # what its models are offered is the same registry view
    assert C.to_openai_tools(a) == sim.brains.tools_for(look=True)
    env, mc = a["envelope"], sim.gait.max_command()
    assert env["source"].startswith("the cockpit's live gait")
    assert env["speed_m_s"] == pytest.approx(mc["v"] / 1000.0, abs=1e-4)
    assert env["turn_rad_s"] == pytest.approx(mc["wz"], abs=1e-3)
    assert env["step_height_mm"] == pytest.approx(sim.gait.hstep, abs=0.05)
    assert (env["goto_reach_m"], env["goto_timeout_s"]) == (C.GOTO_REACH_M, cockpit.GOTO_CAP_S)
    assert a["robot"]["legs"] == 5 and a["robot"]["joint_names"] == ["yaw", "hip", "knee"]
    assert sim.capabilities() == a                             # the method the route serves (a copy)
    assert cb.registry_problems(sim.brains) == []              # the real cockpit routes every tool


def test_capabilities_version_follows_a_gesture_save_and_delete(cockpit_client, tmp_path, monkeypatch):
    import cockpit
    import pebble_keyframes as pk
    import playground
    sim, c = cockpit_client
    real_load = pk.load_keyframe_gestures
    # the save and the delete go to tmp_path; the library reads it beside gait/gestures/ (never written)
    monkeypatch.setattr(cockpit, "GESTURE_DIR", str(tmp_path))
    monkeypatch.setattr(playground, "load_keyframe_gestures",
                        lambda: {**real_load(), **real_load(str(tmp_path))})
    monkeypatch.setattr(cockpit, "delete_keyframe_gesture",
                        lambda name: pk.delete_keyframe_gesture(name, str(tmp_path)))
    name = "t_caps_d056"
    try:
        v0 = _caps(c)["version"]
        seq0 = sim.events.seq
        assert _caps_state(c)[0] == v0
        n0 = _caps_state(c)[1]
        r = c.post("/api/gesture/save", json=dict(GOOD, name=name)).json()
        assert r["ok"] is True and name in r["all"], r
        a = _caps(c)
        assert a["version"] != v0 and name in a["gestures"]
        gesture = next(t for t in a["tools"] if t["name"] == "gesture")
        assert name in gesture["parameters"]["properties"]["name"]["enum"]
        # the state feed (and so /api/events) carries the new version, counted once
        assert _caps_state(c) == (a["version"], n0 + 1)
        assert _caps(c)["version"] == a["version"]                                      # stable again
        r = c.post("/api/gesture/delete", json={"name": name}).json()
        assert r["ok"] is True and name not in r["all"]
        b = _caps(c)
        assert b["version"] == v0 and name not in b["gestures"]
        assert _caps_state(c) == (v0, n0 + 2)
        assert _caps_events(sim, seq0) == []                  # never an entry on the events feed
    finally:
        monkeypatch.undo()
        sim.reload_library()
    assert not os.path.exists(os.path.join(ROOT, "gait", "gestures", name + ".json"))


def test_capabilities_version_follows_a_chord_word_save_and_delete(cockpit_client, tmp_path, monkeypatch):
    import cockpit
    import playground
    sim, c = cockpit_client
    spec_dir, wav_dir = str(tmp_path / "spec"), str(tmp_path / "wav")
    # custom words go to tmp_path, never audio/: the lexicon reads the canon dir + this one
    monkeypatch.setattr(cockpit, "CHORD_SPEC_DIR", spec_dir)
    monkeypatch.setattr(cockpit, "CHORD_CUSTOM_DIR", wav_dir)
    monkeypatch.setattr(playground, "CHORD_CUSTOM_DIR", wav_dir)
    word = "t_caps_word"
    try:
        sim.reload_library()
        v0 = _caps(c)["version"]
        seq0 = sim.events.seq
        n0 = _caps_state(c)[1]
        r = c.post("/api/chord/save", json={"name": word, "spec": {
            "root": 120, "syllables": [{"dur": 0.1, "ratios": [1.0, 1.5]}]}}).json()
        assert r["ok"] is True and word in r["lexicon"], r
        a = _caps(c)
        assert a["version"] != v0 and word in a["lexicon"]
        say = next(t for t in a["tools"] if t["name"] == "say")
        assert word in say["parameters"]["properties"]["word"]["enum"]
        assert _caps_state(c) == (a["version"], n0 + 1)
        r = c.post("/api/chord/delete", json={"name": word}).json()
        assert r["ok"] is True and word not in r["lexicon"]
        assert _caps(c)["version"] == v0 and _caps_state(c) == (v0, n0 + 2)
        assert _caps_events(sim, seq0) == []
    finally:
        monkeypatch.undo()
        sim.reload_library()
    for d in (cockpit.CHORD_SPEC_DIR, cockpit.CHORD_CUSTOM_DIR):
        assert not any(f.startswith(word + ".") for f in (os.listdir(d) if os.path.isdir(d) else []))


def test_capabilities_follow_the_live_gait(cockpit_client):
    sim, c = cockpit_client
    a = _caps(c)
    t0 = float(sim.gait.T)
    seq0 = sim.events.seq
    n0 = _caps_state(c)[1]
    try:
        r = c.post("/api/cmd", json={"line": f"set gait.T {t0 * 1.1:.4f}"}).json()
        assert "refused" not in str(r.get("reply")), r
        b = _caps(c)
        assert b["version"] != a["version"] and b["envelope"]["speed_m_s"] != a["envelope"]["speed_m_s"]
        assert _caps_state(c) == (b["version"], n0 + 1)
        # the console says so (the operator's trace); the events feed does not
        line = f"capabilities: {a['version']} -> {b['version']}"
        assert any(line in str(e[1]) for e in c.get("/api/state").json()["console"])
        assert _caps_events(sim, seq0) == []
    finally:
        c.post("/api/cmd", json={"line": f"set gait.T {t0!r}"})
    assert float(sim.gait.T) == t0 and _caps(c)["version"] == a["version"]


def test_a_capabilities_change_leaves_the_status_tool_alone(cockpit_client):
    """D056 review: the status tool returns the events feed's last five entries to every
    model (local brains, Claude mode, MCP through /api/tool/status). A gait tweak or a
    gesture / chord-word save is a new capabilities version on the STATE feed, never an
    entry on the events feed, so status answers as it did before D056 (the review saw
    ["capabilities", <hash>] push real events out of last_events)."""
    sim, c = cockpit_client
    assert _idle(c)
    assert c.post("/api/tool/say", json={"word": "yes"}).json().get("ok") is True
    assert c.post("/api/tool/stop", json={}).json().get("ok") is True
    assert _idle(c)
    before = c.post("/api/tool/status", json={}).json()["last_events"]
    seq0, (v0, n0) = sim.events.seq, _caps_state(c)
    t0 = float(sim.gait.T)
    try:
        for t in (t0 + 0.1, t0):                              # a new version, then the old one again
            r = c.post("/api/cmd", json={"line": f"set gait.T {t!r}"}).json()
            assert "refused" not in str(r.get("reply")), r
        after = c.post("/api/tool/status", json={}).json()["last_events"]
    finally:
        c.post("/api/cmd", json={"line": f"set gait.T {t0!r}"})
    assert _caps_state(c) == (v0, n0 + 2)                     # both changes happened and were counted
    new = _new_events(sim, seq0)
    assert [e for e in new if e[0] == "capabilities"] == []
    assert [e for e in after if e[0] == "capabilities"] == []
    assert after == before or new, (before, after)     # unchanged unless the robot itself acted


def test_start_cockpit_installs_the_parent_death_hook(monkeypatch, tmp_path):
    """Review item: a SIGKILLed bench left its cockpit running, holding the port."""
    import vision_bench as vb
    seen = {}

    class FakeProc:
        returncode = None

        def poll(self):
            return None

        def terminate(self):
            pass

    def fake_popen(argv, **kw):
        seen.update(kw, argv=argv)
        return FakeProc()
    monkeypatch.setattr(vb.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(vb.Cockpit, "alive", lambda self: True)
    monkeypatch.setattr(vb.tempfile, "gettempdir", lambda: str(tmp_path))     # the log file lands here
    try:
        proc, ck = vb.start_cockpit(8799, world="flat")
    finally:
        if seen.get("stdout"):
            seen["stdout"].close()
    assert isinstance(proc, FakeProc) and ck.url == "http://127.0.0.1:8799"
    assert seen["argv"][-4:] == ["--port", "8799", "--world", "flat"] and seen["cwd"] == vb.ROOT
    if sys.platform.startswith("linux"):
        assert callable(seen["preexec_fn"])
    else:
        assert seen["preexec_fn"] is None


LINUX = pytest.mark.skipif(not sys.platform.startswith("linux"), reason="PR_SET_PDEATHSIG is Linux-only")
GET_PDEATHSIG = ("import ctypes; v = ctypes.c_int(-1); "
                 "ctypes.CDLL(None).prctl(2, ctypes.byref(v), 0, 0, 0); print(v.value)")   # PR_GET_PDEATHSIG


@LINUX
def test_parent_death_hook_arms_the_child():
    """Run in a CHILD (calling the hook here would arm the pytest process itself)."""
    import vision_bench as vb
    hook = vb.parent_death_hook()
    assert hook is not None
    r = subprocess.run([sys.executable, "-c", GET_PDEATHSIG], preexec_fn=hook,
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0 and r.stdout.strip() == str(int(signal.SIGTERM)), r
    plain = subprocess.run([sys.executable, "-c", GET_PDEATHSIG], capture_output=True, text=True, timeout=30)
    assert plain.stdout.strip() == "0"                # without the hook: no death signal


def _gone(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(")", 1)[1].split()[0] == "Z"      # a zombie waiting for its reaper
    except OSError:
        return True


@LINUX
def test_a_sigkilled_parent_takes_its_child_along():
    """The point of the hook: SIGKILL the 'bench' (no finally runs) and its
    'cockpit' child gets SIGTERM from the kernel instead of living on."""
    sim_dir = os.path.join(ROOT, "sim")
    code = (f"import subprocess, sys, time; sys.path.insert(0, {sim_dir!r}); import vision_bench as vb; "
            "p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'], "
            "preexec_fn=vb.parent_death_hook()); print(p.pid, flush=True); time.sleep(60)")
    # MUJOCO_GL=disable: the middle process imports vision_bench -> mujoco, and the pytest process
    # carries MUJOCO_GL=egl (cockpit.py sets it at import) which has no libEGL on the CI runner
    mid = subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True,
                           env=dict(os.environ, MUJOCO_GL="disable"))
    child = None
    try:
        child = int(mid.stdout.readline())
        assert not _gone(child)
        mid.kill()
        mid.wait(timeout=10)
        assert _wait(lambda: _gone(child), timeout=10.0), "the child outlived its SIGKILLed parent"
    finally:
        mid.kill()
        mid.stdout.close()
        if child is not None and not _gone(child):
            os.kill(child, signal.SIGKILL)


# ------------------------------------------------------------------ the fatal path
def test_a_step_exception_fails_requests_instead_of_hanging():
    import cockpit
    sim = _make_sim()
    c = _start(sim)
    assert c.get("/api/state").status_code == 200

    def boom():
        raise FloatingPointError("injected")
    sim.step = boom
    assert _wait(lambda: not sim.alive, timeout=5.0)
    assert "injected" in sim.fatal
    r = c.get("/api/state")
    assert r.status_code == 503 and r.json()["alive"] is False and "injected" in r.json()["fatal"]
    t0 = time.monotonic()
    r = c.post("/api/cmd", json={"line": "show"})        # would hang forever before D052
    assert r.status_code == 503 and time.monotonic() - t0 < 3.0
    assert isinstance(cockpit.SimDead("x"), RuntimeError)


# ------------------------------------------------------------------ goto reactive layer (slow)
@pytest.mark.slow
def test_goto_into_a_wall_ends_blocked_with_bearing_and_range():
    import cockpit
    sim = _make_sim()
    sim.set_world({"base": "flat", "objects": [{"kind": "wall", "pos": [0.6, 0.0], "len_m": 0.8,
                                                "yaw_deg": 90}]}, "wall")
    sim.do("righter off")
    sim.speed = 8.0
    c = _start(sim)
    try:
        r = c.post("/api/tool/goto", json={"x": 1.2, "y": 0.0}).json()
        assert r["stopped"] == "blocked", r
        assert r["obstacle"]["range_m"] < cockpit.GOTO_CLEAR_M
        assert abs(r["obstacle"]["bearing_deg"]) <= cockpit.GOTO_CONE_DEG + 1
        assert r["obstacle"]["detours"] == cockpit.GOTO_DETOURS
        assert r["pose"]["x"] < 0.6 - 0.2                 # it never walked into the wall
    finally:
        sim.alive = False


def test_residual_walker_gets_its_trained_contract():
    """A pre-D052 walker (obs v1, T 1.6 s / step 32 mm) is fed the legacy obs and
    its own gait — and the note says that gait has no envelope on the D052 servo."""
    pytest.importorskip("torch")
    import cockpit
    runs = [r for r in ("robust_fwd2", "cmd_sample3")
            if os.path.exists(os.path.join(cockpit.HERE, "runs", r, "latest.pt"))]
    if not runs:
        pytest.skip("no gait checkpoint in sim/runs/")
    sim = _make_sim()
    note = sim.set_walk(runs[0])
    assert "obs v1" in note and sim.walk["obs"].version == 1
    assert sim.gait.T == pytest.approx(1.6) and "NO speed envelope" in note
    for _ in range(60):
        sim.step()
    assert np.isfinite(sim.walk["res"]).all() and np.any(sim.walk["res"])
    sim.set_walk("off")
    import rocky_model as rm
    assert sim.walk is None and sim.gait.T == pytest.approx(rm.gait_defaults()["cycle_time"])


# ------------------------------------------------------------------ review round 3 (D052 amendment)
def test_editing_a_preset_world_keeps_the_pose_and_heading():
    """Review repro: /api/world/add always renamed the world 'custom' and
    set_world decided 'same world' by name, so the FIRST edit of any preset
    teleported the robot to the origin and reset its yaw."""
    import mujoco
    sim = _make_sim()
    a = np.radians(34.0) / 2
    sim.data.qpos[0:2] = [0.06, 0.014]
    sim.data.qpos[3:7] = [np.cos(a), 0, 0, np.sin(a)]
    mujoco.mj_forward(sim.model, sim.data)
    c = _start(sim)
    try:
        r = c.post("/api/world/add", json={"object": {"kind": "box", "pos": [1.4, 0.0],
                                                      "size": [0.1, 0.1, 0.05]}}).json()
        assert r["ok"] and sim.world_name == "custom"
        p = sim.data.xpos[sim.torso]
        assert np.hypot(p[0] - 0.06, p[1] - 0.014) < 0.01, p
        assert abs(np.degrees(sim.yaw()) - 34.0) < 1.5, np.degrees(sim.yaw())
        # a preset switch still starts at the origin facing +x
        assert c.post("/api/world", json={"preset": "flat"}).json()["ok"]
        p = sim.data.xpos[sim.torso]
        assert np.hypot(p[0], p[1]) < 0.01 and abs(np.degrees(sim.yaw())) < 1.0
    finally:
        sim.alive = False


def test_sim2real_refusal_covers_a_void_retreat_and_a_hot_servo():
    """Review repro: during the void retreat idle_reason said None (cmd_v was
    zeroed) and sim2real started while the retreat still walked. And a joint
    past its thermal budget in the sim now refuses sim2real too."""
    sim = _make_sim("cliff")
    sim.speed = 1.0
    for _ in range(int(1.0 / sim.DT)):
        sim.step()
    sim.do("walk 45")
    for _ in range(int(12.0 / sim.DT)):
        sim.step()
        if sim._void_phase == "retreat":
            break
    assert sim._void_phase == "retreat"
    assert "void guard" in (sim.idle_reason() or "") and "void guard" in (sim.sim2real_refusal() or "")
    assert not sim.is_idle()
    sim2 = _make_sim()
    sim2.speed = 1.0
    for _ in range(int(0.5 / sim2.DT)):
        sim2.step()
    assert sim2.sim2real_refusal() is None
    sim2.thermal.heat[7] = 1.1 * sim2.thermal.budget
    assert "past the thermal budget" in sim2.sim2real_refusal() and "leg 2 hip" in sim2.sim2real_refusal()
