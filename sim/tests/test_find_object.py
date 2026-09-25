"""find_object (the eye-driven 'go to the ball') — the pure parts and the
step policy, with a fake eye and a fake goto: no model server, no MuJoCo
(except the one HTTP test, which builds a real headless CockpitSim and
points its brains at a vision model that never answers).

What is pinned: parse_detection's tolerance of what small VLMs emit (and
that garbage is 'not seen', never a guess); the camera geometry that turns a
bounding box into bearing + distance; the image-bearing -> map-frame math
(negative bearing = LEFT = +y in the body frame); the policy (confident +
far -> goto toward the bearing, unsure or unseen -> a scan turn, near ->
stop, a vetoed goto -> stop and report); and that the HTTP route answers
ok False with the reason when no vision model answers."""
import asyncio
import math
import os
import sys
import threading
import warnings
from collections import deque

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
for sub in ("sim", "gait", "perception"):
    sys.path.insert(0, os.path.join(ROOT, sub))
sys.path.insert(0, ROOT)

import cockpit_brains as cb                                            # noqa: E402


# ------------------------------------------------------------------ the parser
def test_parse_good_json():
    d = cb.parse_detection('{"seen": true, "bearing_deg": -20, "distance_m": 0.8, '
                           '"confidence": 0.9, "what": "an orange ball"}')
    assert d["parsed"] and d["seen"] and d["method"] == "estimate"
    assert d["bearing_deg"] == -20.0 and d["distance_m"] == 0.8 and d["confidence"] == 0.9
    assert d["what"] == "an orange ball"


def test_parse_json_with_prose_fences_and_python_literals():
    d = cb.parse_detection("Sure! Here is what I see:\n```json\n{'seen': True, 'bearing_deg': '15', "
                           "'distance_m': '0.5 m', 'confidence': 85,}\n```\nHope that helps.")
    assert d["parsed"] and d["seen"]
    assert d["bearing_deg"] == 15.0 and d["distance_m"] == 0.5
    assert d["confidence"] == 0.85                        # a percent confidence is rescaled


def test_parse_garbage_is_not_seen_never_a_guess():
    for junk in ("", None, "I think there is a ball on the left, fairly close.", "{not json at all}",
                 '{"seen": true, "bearing_deg": NaN, "distance_m": 1}', "[1, 2]"):
        d = cb.parse_detection(junk)
        assert d["seen"] is False and d["bearing_deg"] is None and d["distance_m"] is None, junk


def test_parse_seen_without_a_bearing_is_not_actionable():
    d = cb.parse_detection('{"seen": true, "distance_m": 0.6, "confidence": 0.9}')
    assert d["parsed"] and d["seen"] is False and "no bearing" in d["what"]


def test_parse_clamps_and_defaults():
    d = cb.parse_detection('{"seen": true, "bearing_deg": -170, "distance_m": -3}')
    assert d["bearing_deg"] == -cb.BEARING_LIMIT_DEG and d["distance_m"] is None
    assert d["confidence"] == cb.FIND_DEFAULT_CONF         # seen, no confidence field
    assert cb.parse_detection('{"seen": false}')["confidence"] == 0.0


def test_parse_bbox_uses_the_camera_geometry_not_the_typed_numbers():
    # LFM2.5-VL's real answer for the ball at map (0.7, 0.2), robot at the origin
    # (eye at (0.10, 0)): true bearing -18.4 deg (left), near edge 0.58 m, centre 0.63 m
    d = cb.parse_detection('{"seen": true, "bbox": [275, 443, 375, 563], "confidence": 0.9, '
                           '"bearing_deg": 0, "distance_m": 0.2, "what": "a yellow sphere"}')
    assert d["method"] == "bbox" and d["bbox"] == [0.275, 0.443, 0.375, 0.563]
    assert abs(d["bearing_deg"] - (-18.4)) < 2.0
    assert 0.55 < d["distance_m"] < 0.66
    # a bare box list (the model's grounding format) and bbox_2d are boxes too
    assert cb.parse_detection("[0.29, 0.45, 0.37, 0.55]")["seen"] is True
    d2 = cb.parse_detection('[{"image_id": 0, "bbox_2d": [663, 429, 805, 521], "label": "box"}]')
    assert d2["seen"] and d2["method"] == "bbox" and d2["bearing_deg"] > 15 and d2["what"] == "box"
    # an impossible box is no box
    assert cb.parse_detection('{"seen": true, "bbox": [500, 500, 500, 700]}')["seen"] is False


def test_gemma_boxes_are_y_first():
    # gemma-4-26b-a4b's real answer for the ball at image bearing -25 deg, 0.45 m (near edge)
    raw = '{"seen": true, "bbox": [557, 202, 687, 323], "confidence": 0.9}'
    assert cb.box_order("gemma-4-26b-a4b") == "yx" and cb.box_order("lfm2.5-vl") == "xy"
    d = cb.parse_detection(raw, order=cb.box_order("gemma-4-26b-a4b"))
    assert abs(d["bearing_deg"] - (-25.0)) < 3.0 and abs(d["distance_m"] - 0.45) < 0.08
    wrong = cb.parse_detection(raw)                          # read x-first: nowhere near
    assert wrong["distance_m"] is None or abs(wrong["distance_m"] - 0.45) > 1.0


def test_pixel_to_floor_matches_the_measured_eye():
    near, b0 = cb.pixel_to_floor(0.5, 1.0)                 # bottom edge, centre
    mid, _ = cb.pixel_to_floor(0.5, 0.5)                   # image centre
    assert abs(near - 0.178) < 0.005 and abs(mid - 0.793) < 0.005 and b0 == 0.0
    assert cb.pixel_to_floor(0.5, 0.2) is None              # above the horizon: no floor
    _d, left = cb.pixel_to_floor(0.1, 0.8)
    _d, right = cb.pixel_to_floor(0.9, 0.8)
    assert left < 0 < right and abs(left + right) < 1e-9   # image left = negative bearing


# ------------------------------------------------------------ bearing -> map frame
@pytest.mark.parametrize("pose,bearing,dist,expect", [
    ({"x": 0, "y": 0, "yaw_deg": 0}, 0, 0.4, (0.4, 0.0)),            # straight ahead = +x
    ({"x": 0, "y": 0, "yaw_deg": 0}, -90, 0.4, (0.0, 0.4)),          # negative bearing = LEFT = +y
    ({"x": 0, "y": 0, "yaw_deg": 0}, 30, 0.2, (0.2 * math.cos(math.radians(-30)),
                                               0.2 * math.sin(math.radians(-30)))),
    ({"x": 1.0, "y": -0.5, "yaw_deg": 90}, 0, 0.3, (1.0, -0.2)),     # facing +y
    ({"x": 1.0, "y": -0.5, "yaw_deg": 90}, 90, 0.3, (1.3, -0.5)),    # right of +y is +x
])
def test_bearing_to_map(pose, bearing, dist, expect):
    x, y = cb.bearing_to_map(pose, bearing, dist)
    assert abs(x - expect[0]) < 1e-9 and abs(y - expect[1]) < 1e-9


# ------------------------------------------------------------------ the policy
def _det(seen=True, bearing=0.0, dist=1.0, conf=0.9):
    return {"seen": seen, "bearing_deg": bearing, "distance_m": dist, "confidence": conf}


def test_policy_pure():
    assert cb.find_policy(_det(dist=1.0)) == ("goto", cb.FIND_STEP_MAX_M)
    assert cb.find_policy(_det(dist=0.45)) == ("goto", 0.2)
    assert cb.find_policy(_det(dist=0.30)) == ("goto", cb.FIND_STEP_MIN_M)   # never rams it
    assert cb.find_policy(_det(dist=0.25)) == ("stop", 0.0)
    assert cb.find_policy(_det(dist=None)) == ("goto", cb.FIND_STEP_BLIND_M)
    assert cb.find_policy(_det(conf=0.39)) == ("scan", 0.0)                  # never walks on it
    assert cb.find_policy(_det(seen=False, conf=1.0)) == ("scan", 0.0)


class FakeEyeSim:
    """Pose + a scripted eye: detections are fed through a fake `complete`
    as JSON text, gotos move the pose to the target (or return a veto)."""

    def __init__(self, goto_result=None):
        self.frames = {"eye": (1, b"\xff\xd8fake")}
        self.events, self.logs, self.cmd_log = deque(), [], []
        self.gesture_busy = False
        self.mode, self.speed = "idle", 1.0
        self.p = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
        self.gotos, self.turns = [], []
        self.goto_result = goto_result

    def log(self, m):
        self.logs.append(m)

    def note_cmd(self, line):
        self.cmd_log.append((0.0, line))

    async def call(self, fn):
        return fn()

    def pose(self):
        return dict(self.p)

    def start_gesture(self, fn, total, name=None):       # a scan turn: turn the pose ~30 deg
        self.turns.append(name)
        self.p["yaw_deg"] += 30.0 if "left" in name else -30.0
        return None

    async def tool_goto(self, x, y):
        self.gotos.append((x, y))
        if self.goto_result:
            return dict(self.goto_result, pose=self.pose())
        self.p.update(x=x, y=y)
        return {"ok": True, "stopped": "arrived", "pose": self.pose()}

    async def tool_stop(self):
        return {"ok": True}


def _brains(sim, answers):
    """answers: list of model texts (the last repeats), or a callable."""
    q = [] if callable(answers) else list(answers)

    def complete(model, messages, tools, max_tokens):
        if callable(answers):
            return answers(model, messages)
        return {"content": q.pop(0) if len(q) > 1 else q[0], "tool_calls": []}
    b = cb.Brains(sim, conf_path=None, complete=complete, catalog=[])
    b._cat = (float("inf"), None, False)          # no catalog: chains are not filtered
    b.frame_wait_s = 0.0
    return b


def run(coro):
    return asyncio.run(coro)


SEEN_FAR = '{"seen": true, "bearing_deg": -30, "distance_m": 1.0, "confidence": 0.9, "what": "ball"}'
SEEN_NEAR = '{"seen": true, "bearing_deg": 5, "distance_m": 0.2, "confidence": 0.9, "what": "ball"}'
UNSURE = '{"seen": true, "bearing_deg": 10, "distance_m": 0.8, "confidence": 0.2, "what": "maybe"}'
NOTHING = '{"seen": false, "what": "floor"}'


def test_seen_and_far_walks_toward_the_bearing_then_stops_near():
    sim = FakeEyeSim()
    r = run(_brains(sim, [SEEN_FAR, SEEN_NEAR]).find_object("ball"))
    assert r["ok"] and r["found"] and r["looks"] == 2
    assert len(sim.gotos) == 1 and not sim.turns
    x, y = sim.gotos[0]                                       # 0.4 m at -30 deg = ahead-LEFT (+y)
    assert abs(x - 0.4 * math.cos(math.radians(30))) < 1e-3 and abs(y - 0.4 * math.sin(math.radians(30))) < 1e-3
    assert any(line.startswith("tool goto") for _t, line in sim.cmd_log)   # noted: replays repeat it
    assert "found the ball" in r["detail"] and any("find ball" in m for m in sim.logs)
    assert any(k == "find" for k, _ in sim.events)


def test_low_confidence_scans_never_walks():
    sim = FakeEyeSim()
    r = run(_brains(sim, [UNSURE, UNSURE, SEEN_NEAR]).find_object("ball"))
    assert r["found"] and sim.gotos == [] and len(sim.turns) == 2
    assert all("left" in t for t in sim.turns) and r["turned_deg"] == 60.0


def test_not_seen_scans_at_most_one_full_circle():
    sim = FakeEyeSim()
    r = run(_brains(sim, [NOTHING]).find_object("ball", max_steps=16))
    assert r["ok"] and not r["found"] and sim.gotos == []
    assert len(sim.turns) == 11 and "full circle" in r["detail"]   # 11 x 30 = 330 deg + the FOV


def test_steps_exhausted_is_not_found_with_the_last_sighting():
    sim = FakeEyeSim()
    r = run(_brains(sim, [SEEN_FAR]).find_object("ball", max_steps=3))
    assert r["ok"] and not r["found"] and r["looks"] == 3
    assert len(sim.gotos) == 2                                # no move after the last look
    assert r["last_seen"]["bearing_deg"] == -30.0 and "not reached" in r["detail"]


@pytest.mark.parametrize("veto", ["cliff", "blocked", "stuck"])
def test_a_vetoed_goto_ends_it_and_is_reported(veto):
    sim = FakeEyeSim(goto_result={"ok": False, "stopped": veto, "detail": f"{veto} detail"})
    r = run(_brains(sim, [SEEN_FAR]).find_object("ball"))
    assert r["ok"] is False and r["found"] is False and r["stopped"] == veto
    assert len(sim.gotos) == 1 and veto in r["detail"]


def test_operator_stop_ends_it():
    sim = FakeEyeSim()

    def eye(model, messages):
        sim.note_cmd("teleop  ")                              # the STOP key, pressed mid-run
        return {"content": UNSURE, "tool_calls": []}
    r = run(_brains(sim, eye).find_object("ball"))
    assert r["ok"] is False and r["stopped"] == "user" and r["looks"] == 1


def test_bad_arguments_and_voice_gate():
    b = _brains(FakeEyeSim(), [SEEN_NEAR])
    assert run(b.find_object(""))["ok"] is False
    assert run(b.find_object("ball", max_steps="lots"))["ok"] is False
    assert run(b.find_object("ball", method="psychic"))["ok"] is False
    r = run(b.tool("find_object", {"name": "ball"}, voice=True))
    assert r["error"] == "voice_unconfirmed"
    assert run(b.tool("turn", {"deg": 30}, voice=True))["error"] == "voice_unconfirmed"
    assert run(b.tool("turn", {"deg": 400}))["ok"] is False
    assert "find_object" in {t["function"]["name"] for t in b.tools_for()}
    assert "turn" not in {t["function"]["name"] for t in b.tools_for()}   # internal only


def test_no_vision_model_is_an_error_not_a_hang():
    def dead(model, messages, tools, max_tokens):
        raise RuntimeError("HTTP 503: nobody home")
    sim = FakeEyeSim()
    b = cb.Brains(sim, conf_path=None, complete=dead, catalog=[])
    b._cat = (float("inf"), None, False)
    r = run(b.find_object("ball"))
    assert r["ok"] is False and "no vision model answered" in r["error"] and sim.gotos == []


# ------------------------------------------------------------------ over HTTP
def test_http_find_object_without_a_vision_model_says_so():
    pytest.importorskip("starlette")
    from starlette.testclient import TestClient
    import cockpit
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        sim = cockpit.CockpitSim("flat")
    sim.do("righter off")
    sim.render_every = 10 ** 9                  # no GL: the eye frame is planted below
    b = sim.brains
    b.conf_path = None
    b.base = "http://127.0.0.1:9/v1"            # nothing listens: the catalog is unreachable
    b._complete_fn = lambda *a: (_ for _ in ()).throw(RuntimeError("HTTP 503: no vision model"))
    sim.frames["eye"] = (1, b"\xff\xd8fake")
    threading.Thread(target=sim.run_forever, daemon=True).start()
    try:
        c = TestClient(cockpit.make_app(sim, extra_hosts=("testserver",)))
        r = c.post("/api/tool/find_object", json={"name": "ball"})
        assert r.status_code == 200
        j = r.json()
        assert j["ok"] is False and j["found"] is False
        assert "no vision model answered" in j["error"] and "lfm2.5-vl" in j["error"]
        r = c.post("/api/look", json={"find": "ball"})
        assert r.status_code == 200 and r.json()["ok"] is False
    finally:
        sim.alive = False
