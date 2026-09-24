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
import os
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
