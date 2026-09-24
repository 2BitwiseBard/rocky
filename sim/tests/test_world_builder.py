"""world_builder.py: every preset builds, objects/terrain/conditions land in
the model, the eye camera exists (needs mujoco + sim/pebble.xml)."""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
mujoco = pytest.importorskip("mujoco")
pytestmark = pytest.mark.skipif(not os.path.exists(os.path.join(HERE, "..", "pebble.xml")),
                                reason="run sim/build_mjcf.py first")
from world_builder import build, PRESETS, KINDS, world_xml    # noqa: E402


def test_presets_build_with_an_eye():
    for name, spec in PRESETS.items():
        m, z = build(spec)
        assert m.ncam == 1 and m.camera("eye").id >= 0, name
        assert m.nbody >= 22, name
        assert z == (0.16 if spec.get("base") == "cliff" else 0.0), name


def test_every_object_kind_builds_and_is_lidar_visible():
    spec = {"base": "flat", "objects": [{"kind": k, "pos": [0.6 + 0.3 * i, 0.0]} for i, k in enumerate(KINDS)]}
    m, _ = build(spec)
    base, _ = build({"base": "flat"})
    assert m.ngeom > base.ngeom + len(KINDS)          # rubble/stairs add several each
    extra = [g for g in range(m.ngeom) if m.geom_group[g] == 3]
    assert len(extra) == m.ngeom - base.ngeom          # every added geom is group 3
    assert m.nbody == base.nbody + 1                   # the ball is a free body


def test_terrain_heightfield_and_conditions():
    m, _ = build({"base": "flat", "friction": 0.5, "gravity_tilt_deg": 8.0,
                  "terrain": {"kind": "rough", "amp_m": 0.03, "size_m": 1.0, "cell_m": 0.05, "seed": 3}})
    assert m.nhfield == 1
    d = m.hfield_data
    assert d.min() >= 0.0 and d.max() <= 1.0 and d.std() > 0.05
    assert m.geom_friction[m.geom("floor").id, 0] == pytest.approx(0.5)
    g = m.opt.gravity
    assert np.hypot(g[0], g[1]) == pytest.approx(9.81 * np.sin(np.radians(8)), rel=1e-3)
    assert g[2] < 0


def test_unknown_kind_is_refused():
    with pytest.raises(ValueError):
        world_xml({"objects": [{"kind": "dragon"}]})
