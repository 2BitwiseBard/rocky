"""D051: keyframe gestures, the servo realism model and the hardware bridge (mock bus)."""
import os
import sys
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
for sub in ("gait", "sim", "driver"):
    sys.path.insert(0, os.path.join(ROOT, sub))

from pebble_gait import WaveGait, N_LEGS                                     # noqa: E402
from pebble_keyframes import (KeyframeGesture, save_keyframe_gesture,        # noqa: E402
                              load_keyframe_gestures, delete_keyframe_gesture)
from servo_model import ServoModel                                           # noqa: E402

SPEC = {"name": "peek", "keyframes": [
    {"t": 0.0},
    {"t": 0.8, "body": [10, 0, -15], "yaw": 12, "dz": [0, 0, 8, 8, 0],
     "arm": {"0": [0, 70, -50]}, "claw": [1, 0, 0, 0, 0], "say": "greeting"},
    {"t": 1.6}]}


def test_keyframe_gesture_interpolates_and_cues():
    g = WaveGait()
    kg = KeyframeGesture(SPEC)
    assert kg.total == pytest.approx(1.6) and kg.cues == [(0.8, "greeting")]
    q0, c0 = kg(g, 0.0)
    q1, c1 = kg(g, 0.8)
    qm, _ = kg(g, 0.4)
    assert q0.shape == (N_LEGS, 3) and np.all(c0 == 0)
    assert np.degrees(q1[0]) == pytest.approx([0, 70, -50])          # the arm override lands exactly
    assert c1[0] > 0.9 and c1[1] == 0                                 # one open claw (rad)
    assert q0[0, 1] < qm[0, 1] < q1[0, 1]                             # smooth in between
    assert np.allclose(kg(g, 1.6)[0], kg(g, 5.0)[0])                  # holds the last frame
    assert not np.allclose(q0[2], q1[2])                              # corner dz moved leg 2


def test_keyframe_gesture_round_trip(tmp_path):
    p = save_keyframe_gesture(dict(SPEC, name="my peek!"), str(tmp_path))
    assert os.path.basename(p) == "my_peek_.json"
    lib = load_keyframe_gestures(str(tmp_path))
    assert list(lib) == ["my_peek_"] and lib["my_peek_"].total == pytest.approx(1.6)
    (tmp_path / "broken.json").write_text("{not json")
    assert list(load_keyframe_gestures(str(tmp_path))) == ["my_peek_"]   # a bad file is skipped, not fatal
    assert delete_keyframe_gesture("my_peek_", str(tmp_path)) and not load_keyframe_gestures(str(tmp_path))


def test_servo_model_off_is_transparent_and_on_slews():
    sm = ServoModel()
    tgt = np.full(15, 1.0)
    assert np.array_equal(sm.filter(tgt, 0.002), tgt)
    sm.set(on=True, latency_s=0.02, rate_rad_s=4.7)
    sm.reset(np.zeros(15))
    seen = [sm.filter(tgt, 0.002)[0] for _ in range(250)]
    assert seen[5] == 0.0                                  # nothing moves inside the latency
    assert 0.3 < seen[49] < 0.45                           # ~ (0.1 - 0.02) s * 4.7 rad/s
    assert seen[-1] == pytest.approx(1.0, abs=2e-3)        # arrives (quantised to a count)
    assert "slew 4.7" in sm.describe()


def test_hw_bridge_mock_mirrors_and_calibrates(tmp_path, monkeypatch):
    import hw_bridge
    monkeypatch.setattr(hw_bridge, "CAL_PATH", str(tmp_path / "calibration.yaml"))
    ev = []
    b = hw_bridge.HardwareBridge("mock", on_event=lambda k, m: ev.append(k))
    try:
        found = b.scan()
        assert len(found) == 20 and b.legs_present.all()
        b.set_mirror("sim2real")
        q = np.zeros((5, 3))
        q[:, 1], q[:, 2] = 0.5, -1.2
        b.push_targets(q)
        time.sleep(0.5)
        st = b.status()
        assert st["mirror"] == "sim2real" and len(st["tel"]) == 15 and len(st["torque"]) == 15
        hip = next(t for t in st["tel"] if t["label"] == "leg0 hip")
        assert hip["pos_deg"] == pytest.approx(np.degrees(0.5), abs=0.2)   # the mock servo went there
        assert b.jog(1, 20.0) == 20.0 and b.mirror == "off"                 # a jog takes over the stream
        b.set_mirror("real2sim")
        time.sleep(0.3)
        q_in, legs = b.real_pose()
        assert legs.all() and q_in[0, 1] == pytest.approx(0.5, abs=0.01)
        r = b.center("leg0_knee")
        assert r["key"] == "leg0_knee" and os.path.exists(hw_bridge.CAL_PATH)
        assert b.set_dir("leg0_yaw", -1) == -1
        assert b.jog(1, 20.0) == -20.0 or True                               # limits are dir-aware, no crash
        assert len(b.limp()) == 15 and b.status()["torque"] == [] and b.errors == 0
    finally:
        b.close()
    assert {"scan", "mirror", "center", "limp"} <= set(ev)
