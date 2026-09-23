"""shove.py: the D048 shove model's arithmetic and wrench (needs mujoco)."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
mujoco = pytest.importorskip("mujoco")
from shove import Shove, impulse_ns, profile, bodyweights, LEVER_Z    # noqa: E402

XML = os.path.join(HERE, "..", "pebble.xml")


def test_profile_and_impulse():
    assert profile(-0.01, 0.4) == 0.0 and profile(0.4, 0.4) == 0.0
    assert profile(0.2, 0.4) == pytest.approx(1.0)
    assert profile(0.1, 0.4, "rect") == 1.0
    # half-sine mean is 2/pi of the peak; rectangular is the peak
    assert impulse_ns(60, 0.4) == pytest.approx(60 * 0.4 * 2 / np.pi)
    assert impulse_ns(60, 0.4, "rect") == pytest.approx(24.0)
    # the old fallen-demo strike, for the record: 30 N s
    assert impulse_ns(120, 0.25, "rect") == pytest.approx(30.0)


@pytest.mark.skipif(not os.path.exists(XML), reason="run sim/build_mjcf.py first")
def test_wrench_is_force_at_com_plus_rim_torque():
    model = mujoco.MjModel.from_xml_path(XML)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    torso = model.body("torso").id
    sh = Shove(30.0, 0.0, dur=0.4, t0=1.0)
    assert sh.apply(model, data, torso, 0.5) is False          # not yet: wrench zero
    assert np.all(data.xfrc_applied[torso] == 0)
    assert sh.apply(model, data, torso, 1.2) is True           # peak of the half-sine
    F, tau = data.xfrc_applied[torso, :3], data.xfrc_applied[torso, 3:]
    assert F == pytest.approx([30.0, 0.0, 0.0])
    # +x push above the CoM tips the top toward +x: a rotation about +y
    # (r x F with r = (0, 0, lever) and F = (30, 0, 0) is (0, 30 lever, 0))
    lever = (data.xpos[torso][2] + LEVER_Z) - data.xipos[torso][2]
    assert tau[1] == pytest.approx(30.0 * lever, rel=1e-6)
    assert tau[0] == pytest.approx(0.0, abs=1e-9) and tau[2] == pytest.approx(0.0, abs=1e-9)
    assert lever > 0.03                                        # the rim is above the CoM
    assert sh.apply(model, data, torso, 1.5) is False          # over, and zeroed again
    assert np.all(data.xfrc_applied[torso] == 0)
    assert 0.8 < bodyweights(25.0, model) < 1.1               # 2.67 kg robot


def test_describe_mentions_units():
    model = mujoco.MjModel.from_xml_path(XML) if os.path.exists(XML) else None
    if model is None:
        pytest.skip("no pebble.xml")
    s = Shove(40.0, 0.0).describe(model)
    assert "N·s" in s and "BW" in s and "halfsine" in s
