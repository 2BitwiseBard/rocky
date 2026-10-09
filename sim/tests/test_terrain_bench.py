"""terrain_bench (D065): the seed tiers, the row table and its worlds, the IMU modes and
stacks (applied through the Playground's hooks, refused honestly without them), the pass
rules on synthetic runs, the provenance stamps, PRESETS untouched, and a short smoke of one
row under S0 / S1 / S2 (ideal: S2 on a flat walk is bit-identical to S1). Needs mujoco."""
import json
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "gait"))
mujoco = pytest.importorskip("mujoco")
pytestmark = pytest.mark.skipif(not os.path.exists(os.path.join(HERE, "..", "pebble.xml")),
                                reason="run sim/build_mjcf.py first")
import world_builder as wb                                            # noqa: E402

_PRESETS_BEFORE = json.dumps(wb.PRESETS, sort_keys=True)
import terrain_bench as tb                                            # noqa: E402
import rocky_model as rm                                              # noqa: E402
import rl_common as rc                                                # noqa: E402


# ------------------------------------------------------------------ seed tiers
def test_seed_tiers_are_pinned_and_disjoint():
    assert tb.BENCH_SEEDS == (0, 1, 2, 3, 4)
    assert (tb.HELDOUT_SEEDS.start, tb.HELDOUT_SEEDS.stop) == (100, 200)
    assert tb.TRAIN_SEED_MIN == 1000
    assert [tb.tier_of(s) for s in (0, 4, 5, 99, 100, 199, 200, 999, 1000, 10 ** 6)] == \
        ["bench", "bench", None, None, "heldout", "heldout", None, None, "train", "train"]


def test_rows_score_only_the_bench_or_heldout_tier():
    for r in tb.rows():
        assert tb.tier_of(r["seed"]) == "bench", r["id"]
    held = [r for r in tb.rows(tier="heldout") if r["world"] == "W4"]
    assert held and all(tb.tier_of(r["seed"]) == "heldout" for r in held)
    with pytest.raises(ValueError):
        tb.rows(tier="train")


# ------------------------------------------------------------------ the row table
KEYS = {"id", "world", "kind", "build", "seconds", "z0", "yaw_deg", "walk", "phase", "seed", "slope_deg", "drop"}


def test_row_table_schema_and_counts():
    full, quick = tb.rows(), tb.rows(quick=True)
    ids = [r["id"] for r in full]
    assert len(ids) == len(set(ids)) == 87
    assert {r["world"] for r in full} == {f"W{k}" for k in range(1, 9)}
    for r in full:
        assert KEYS <= set(r), r["id"]
        assert r["kind"] in ("walk", "stand", "shove", "cliff", "motion", "run_sim"), r["id"]
        assert r["id"].startswith(r["world"] + "."), r["id"]
    assert {r["id"] for r in quick} <= set(ids)
    # --quick keeps one phase and one rubble seed, never a cliff approach
    assert {r["id"] for r in full if r["kind"] == "cliff"} == {r["id"] for r in quick if r["kind"] == "cliff"}
    assert len([r for r in full if r["kind"] == "cliff" and r["build"].get("cliff")]) == \
        len(tb.CLIFF_DEG) * len(tb.CLIFF_SPEEDS)
    assert sum(r["kind"] == "run_sim" for r in full) == 1
    # review 9r: flat motions beyond the straight walk (P1), a raised leading foot at an edge (P6)
    w1 = {r["id"] for r in full if r["world"] == "W1"}
    assert {"W1.flat.turn", "W1.flat.walkturn", "W1.flat.shove.x", "W1.flat.shove.y"} <= w1
    assert tb.row_by_id("W1.flat.walkturn")["cmd"] == [tb.WALK_ASK, 0.0, 0.2]
    sc = {r["build"]["wb"]["gravity_tilt_dir_deg"] for r in full if r["id"].startswith("W8.slopecliff5")}
    assert sc == {0.0, 90.0, 180.0}
    assert tb.row_by_id("W3.stone20.f0.stand")["z0"] == pytest.approx(0.02)
    with pytest.raises(KeyError):
        tb.row_by_id("W9.nowhere")


def test_every_bench_world_builds():
    seen = set()
    for r in tb.rows():
        b = r["build"]
        if "wb" not in b:
            continue
        key = json.dumps(b["wb"], sort_keys=True)
        if key in seen:
            continue
        seen.add(key)
        model, z = wb.build(b["wb"])
        assert model.nbody >= 22, r["id"]
        assert z == (0.16 if b["wb"].get("base") == "cliff" else 0.0), r["id"]
    assert len(seen) >= 15


def test_slope_worlds_tilt_gravity_and_ramps_carry_their_angle():
    for deg, d in ((5, 0), (8, 90), (10, 180)):
        m, _ = wb.build(tb._slope(deg, d))
        g = m.opt.gravity / np.linalg.norm(m.opt.gravity)
        assert np.degrees(np.arccos(-g[2])) == pytest.approx(deg, abs=1e-3)
        assert np.degrees(np.arctan2(g[1], g[0])) % 360 == pytest.approx(d % 360, abs=1e-3)
    for r in tb.rows():
        if r["world"] == "W5":
            o = r["build"]["wb"]["objects"][0]
            assert np.degrees(np.arctan2(o["rise_m"], o["len_m"])) == pytest.approx(r["slope_deg"], abs=1e-3)


def test_the_stone_sits_under_its_foot():
    from pebble_gait import WaveGait
    p_nom = WaveGait().p_nom
    for foot in (0, 3):
        o = tb._stone_spec(20, foot)["objects"][0]
        assert o["kind"] == "box" and o["size"] == [0.04, 0.04, 0.02]
        assert np.allclose(o["pos"], p_nom[foot, :2] / 1000.0, atol=1e-4)


def test_presets_untouched():
    """test_place_bench pins world_builder.PRESETS: the bench's worlds live in the bench."""
    assert json.dumps(wb.PRESETS, sort_keys=True) == _PRESETS_BEFORE
    assert not {r["id"] for r in tb.rows()} & set(wb.PRESETS)


# ------------------------------------------------------------------ IMU modes and stacks
def test_imu_specs():
    assert tb.imu_spec("ideal", 3) is None
    n = tb.imu_spec("noisy", 3)
    assert n["seed"] == 3 and n["latency_s"] == 0.02
    for k in ("grav_sigma", "gyro_sigma", "gyro_bias"):
        assert n[k] == rc.OBS_NOISE[k]
    m = tb.imu_spec("mount1.5", 0)
    assert m["mount_deg"] == 1.5 and m["mount_axis"] == [1.0, 0.0, 0.0]
    with pytest.raises(ValueError):
        tb.imu_spec("perfect", 0)


class _NoHooks:
    """A Playground without the D065 hooks: no set_imu, no leveler, no `level` command."""
    def __init__(self):
        self.sup = object()

    def do(self, line):
        return f"unknown command {line.split()[0]!r}"


def test_a_missing_hook_is_unavailable_never_ideal():
    pg = _NoHooks()
    assert tb._apply_imu(pg, "ideal", 0) is None
    for mode in ("noisy", "mount1.5"):
        assert "IMU-spec hook" in tb._apply_imu(pg, mode, 0)
    assert "not wired" in tb._apply_stack(pg, "S2")
    assert "B160" in tb._apply_stack(pg, "S3")
    r = tb.run_one(tb.row_by_id("W1.flat.stand"), "S3", "ideal")
    assert r["status"] == "unavailable"


def test_the_noisy_and_mount_modes_land_on_the_playground_imu():
    want = {"noisy": ("grav_sigma", rc.OBS_NOISE["grav_sigma"]), "mount1.5": ("mount_deg", 1.5)}
    for mode, (attr, val) in want.items():
        pg = tb._playground(tb.row_by_id("W1.flat.stand"))
        assert tb._apply_imu(pg, mode, 0) is None, mode
        assert getattr(pg.imu, attr) == val, mode
        pg.reset_guards()                                  # a respawn / world change keeps the spec
        assert getattr(pg.imu, attr) == val, mode


def test_plan_runs_s0_ideal_only_unless_asked_and_run_sim_once():
    rs = tb.rows(quick=True)
    tasks = tb.plan(rs, ["S0", "S1"], ["ideal", "noisy"])
    assert not [t for t in tasks if t[1] == "S0" and t[2] != "ideal"]
    assert [t for t in tasks if t[1] == "S1" and t[2] == "noisy"]
    assert sum(t[0]["kind"] == "run_sim" for t in tasks) == 1
    assert [t for t in tb.plan(rs, ["S0"], ["noisy"], imu_explicit=True) if t[1] == "S0" and t[2] == "noisy"]


# ------------------------------------------------------------------ smoke
RUN_KEYS = {"tilt_rms_deg", "tilt_p95_deg", "tilt_peak_deg", "tilt_end_deg", "roll_rms_deg", "pitch_rms_deg",
            "height_err_mean_mm", "height_err_rms_mm", "progress_mm", "creep_mm", "fell", "tipped",
            "gate_holds", "probe_out_edges", "void", "false_void", "min_clear_mm", "belly_contact_s",
            "load_rms_max", "load_peak", "heat_max", "thermal_trip", "level_sat_pct", "level_max_mm", "digest"}


def test_smoke_one_walk_under_s0_s1_s2():
    row = tb.row_by_id("W1.flat.walk.p0")
    runs = {s: tb.run_one(row, s, "ideal", seconds=2.0) for s in ("S0", "S1", "S2")}
    for s, r in runs.items():
        assert r["status"] == "ok", (s, r.get("why"))
        assert RUN_KEYS <= set(r), s
        assert r["seconds"] == pytest.approx(2.0)
        assert not r["fell"] and r["tilt_peak_deg"] < 1.0 and r["min_clear_mm"] > 40.0
        assert r["progress_mm"] > 0 and r["creep_mm"] is None
    # an ideal IMU on flat ground reads inside the deadband: S2 is S1, bit for bit (P1)
    assert runs["S2"]["digest"] == runs["S1"]["digest"] and runs["S2"]["level_max_mm"] == 0.0
    assert runs["S2"]["level_status"]["enabled"] and not runs["S1"]["level_status"]["enabled"]


def test_the_meter_calls_a_void_with_ground_in_the_probes_reach_false():
    """A void verdict on a drop world is false when ground lies within PROBE_MAX of the
    foot's un-probed target (review 9r: measured from the lowered foot, ground 56-60 mm
    below the target counted); a void on a world with no drop is false whatever is under
    it. A planted stance lifted 10 mm: the floor is ~10 mm under every foot."""
    row = dict(tb.row_by_id("W1.flat.walk.p0"))
    pg = tb._playground(row)
    for _ in range(int(round(0.5 / pg.DT))):
        pg.step()
    pg.data.qpos[2] += 0.010
    mujoco.mj_forward(pg.model, pg.data)
    for drop in (True, False):
        m = tb.Meter(pg, dict(row, drop=drop))
        pg.void = {"leg": 0}
        m.physics(0.0)
        m.sample(tb.TICK_S)
        r = m.result()
        pg.void = None
        assert r["void"] and r["void_ground_nom_mm"] is not None and 5.0 < r["void_ground_nom_mm"] < 15.0, r
        assert r["false_void"] and not r["void_nohit"]
    m = tb.Meter(pg, dict(row, drop=True))
    m.physics(0.0)
    m.sample(tb.TICK_S)
    assert not m.result()["false_void"]                       # no void, nothing to call


def test_a_stone_row_reports_the_stone_foot_load():
    r = tb.run_one(tb.row_by_id("W3.stone20.f0.stand"), "S1", "ideal", seconds=2.5)
    assert r["status"] == "ok" and r["stone_foot_n"] is not None and 0.0 <= r["stone_foot_closed_pct"] <= 100.0


def test_a_stand_measures_the_true_tilt_on_a_gravity_slope():
    r = tb.run_one(tb.row_by_id("W2.slope8.d0.stand"), "S1", "ideal", seconds=0.5)
    assert r["status"] == "ok" and r["tilt_end_deg"] == pytest.approx(8.06, abs=0.1)
    assert r["creep_mm"] is not None and r["progress_mm"] is None


# ------------------------------------------------------------------ the rules
def _run(row, stack, imu="ideal", **kw):
    r = dict(row=row, stack=stack, imu=imu, status="ok", kind=kw.pop("kind", "walk"), tilt_rms_deg=1.0,
             tilt_end_deg=1.0, progress_mm=500.0, level_max_mm=0.0, digest="d", fell=False, fired=True,
             cliff_ok=True, false_void=False, min_clear_mm=50.0, belly_contact_s=0.0, load_rms_max=0.1,
             thermal_trip=False)
    r.update(kw)
    return r


def test_evaluate_passes_a_clean_s2_and_fails_each_rule():
    base = [_run("W1.flat.walk.p0", "S1"), _run("W1.flat.walk.p0", "S2"),
            _run("W3.stone20.f0.stand", "S1", kind="stand", tilt_end_deg=3.9, progress_mm=None),
            _run("W3.stone20.f0.stand", "S2", kind="stand", tilt_end_deg=0.4, progress_mm=None),
            _run("W2.slope8.d0.stand", "S1", kind="stand", tilt_end_deg=8.1, progress_mm=None),
            _run("W2.slope8.d0.stand", "S2", kind="stand", tilt_end_deg=3.2, progress_mm=None),
            _run("W8.cliff.a0.v25", "S1", kind="cliff"), _run("W8.cliff.a0.v25", "S2", kind="cliff")]
    v = tb.evaluate(base)
    assert v["verdict"] == "pass", v
    assert v["rules"]["P9"]["status"] == "external"

    def broken(row, stack, **kw):
        runs = [dict(r) for r in base]
        next(r for r in runs if r["row"] == row and r["stack"] == stack).update(kw)
        return tb.evaluate(runs)["rules"]
    assert broken("W1.flat.walk.p0", "S2", digest="e")["P1"]["status"] == "fail"
    assert broken("W1.flat.walk.p0", "S2", level_max_mm=0.6)["P1"]["status"] == "fail"
    assert broken("W3.stone20.f0.stand", "S2", tilt_end_deg=1.2)["P2"]["status"] == "fail"
    assert broken("W2.slope8.d0.stand", "S2", tilt_end_deg=3.6)["P3"]["status"] == "fail"
    assert broken("W1.flat.walk.p0", "S2", progress_mm=470.0)["P4"]["status"] == "fail"
    assert broken("W1.flat.walk.p0", "S2", fell=True)["P5"]["status"] == "fail"
    assert broken("W8.cliff.a0.v25", "S2", fired=False)["P6"]["status"] == "fail"
    assert broken("W1.flat.walk.p0", "S2", false_void=True, void_class="false")["P6"]["status"] == "fail"
    assert broken("W1.flat.walk.p0", "S2", min_clear_mm=20.0)["P7"]["status"] == "fail"
    assert broken("W1.flat.walk.p0", "S2", load_rms_max=0.7)["P8"]["status"] == "fail"
    assert broken("W1.flat.walk.p0", "S2", tilt_rms_deg=1.2)["R1"]["status"] == "fail"
    # S1 failing its own cliff hold is not S2's regression; a noisy-mode P1 needs no bit identity
    assert broken("W8.cliff.a0.v25", "S1", cliff_ok=False)["P6"]["status"] == "pass"
    noisy = base + [_run("W1.flat.walk.p0", "S1", "noisy"), _run("W1.flat.walk.p0", "S2", "noisy", digest="x")]
    assert tb.evaluate(noisy)["rules"]["P1"]["status"] == "pass"
    # without S2 runs nothing can pass; mount1.5 is reported, never gated
    assert tb.evaluate([r for r in base if r["stack"] == "S1"])["verdict"] == "unavailable"
    m = [dict(r, imu="mount1.5") for r in base]
    assert tb.evaluate(m)["verdict"] == "unavailable"


def test_the_b162_restatements_keep_the_d065_wording_visible():
    """B162 restated P1 (the flat-motion rows), P4 (the derate; rubble by family) and P6 (a
    void the probe cannot reach is reported, not gated); evaluate() still returns each under
    its D065 wording, and a restated rule still fails on what it claims to measure."""
    base = [_run("W1.flat.walk.p0", "S1"), _run("W1.flat.walk.p0", "S2"),
            _run("W1.flat.turn", "S1", kind="motion", progress_mm=None),
            _run("W1.flat.turn", "S2", kind="motion", progress_mm=None, level_max_mm=0.6, digest="t"),
            # the walk + turn: the same walk until `stop`, then the leveler answers the probe's HOLD (B163)
            _run("W1.flat.walkturn", "S1", kind="motion", progress_mm=124.3, progress_stop_mm=126.6, tilt_rms_deg=1.17),
            _run("W1.flat.walkturn", "S2", kind="motion", progress_mm=127.9, progress_stop_mm=126.6, tilt_rms_deg=0.80,
                 level_max_mm=14.7, digest="w"),
            _run("W2.slope5.d0.walk.p0", "S1", progress_mm=690.0),
            _run("W2.slope5.d0.walk.p0", "S2", progress_mm=633.0, derate_pct=94.97),
            _run("W6.stairs15.walk.p0", "S1", tipped=True), _run("W6.stairs15.walk.p0", "S2")]
    base += [_run(f"W4.rubble20.s{k}.walk", st, progress_mm=p)
             for k, (p1, p2) in enumerate(((400.0, 300.0), (300.0, 420.0))) for st, p in (("S1", p1), ("S2", p2))]
    v = tb.evaluate(base)
    r, d = v["rules"], v["d065"]
    # P1: a turn the leveler acts on is gated on tilt and progress, its offset reported; D065 failed it
    assert r["P1"]["status"] == "pass" and d["P1"]["status"] == "fail"
    assert any("W1.flat.turn" in x for x in r["P1"]["flat_motion"])
    # P4: 633 / (690 x 0.9497) = 96.6 % passes; the D065 line (95 % of 690) fails; rubble by family
    assert r["P4"]["status"] == "pass" and d["P4"]["status"] == "fail", (r["P4"], d["P4"])
    assert r["P4"]["rubble_family"]["ideal"]["family_pct"] == pytest.approx(102.9, abs=0.1)
    assert r["P4"]["rubble_family"]["ideal"]["rows_under_95"] == ["W4.rubble20.s0.walk"]
    assert r["P4"]["stairs"]["ideal"]["s1_tips"] == 1 and r["P4"]["stairs"]["ideal"]["s2_tips"] == 0

    def broken(row, stack, **kw):
        runs = [dict(x) for x in base]
        next(x for x in runs if x["row"] == row and x["stack"] == stack).update(kw)
        return tb.evaluate(runs)
    assert broken("W1.flat.turn", "S2", tilt_rms_deg=1.1)["rules"]["P1"]["status"] == "fail"
    assert broken("W1.flat.walkturn", "S2", progress_stop_mm=120.0)["rules"]["P1"]["status"] == "fail"
    assert any("progress at stop 126.6 -> 126.6 mm, at the end 124.3 -> 127.9" in x for x in r["P1"]["flat_motion"])
    assert broken("W2.slope5.d0.walk.p0", "S2", progress_mm=600.0)["rules"]["P4"]["status"] == "fail"
    assert broken("W4.rubble20.s1.walk", "S2", progress_mm=330.0)["rules"]["P4"]["status"] == "fail"
    # P6: a probe-reach void is a safe stop, reported; within the probe's reach it fails
    pr = broken("W1.flat.walk.p0", "S2", false_void=True, void_class="probe_reach")
    assert pr["rules"]["P6"]["status"] == "pass" and pr["d065"]["P6"]["status"] == "fail"
    assert pr["rules"]["P6"]["probe_reach"]
    assert broken("W1.flat.walk.p0", "S2", false_void=True, void_class="false")["rules"]["P6"]["status"] == "fail"


def test_the_meter_classes_a_void_by_the_probes_reach():
    """void_class: 'false' within PROBE_MAX of the un-probed target, past it 'drop' on a drop
    world and 'probe_reach' on one without (B161), none without a void."""
    row = dict(tb.row_by_id("W1.flat.walk.p0"))
    pg = tb._playground(row)
    m = tb.Meter(pg, dict(row, drop=False))
    assert m._void_class(False) is None
    for nom, drop, want in ((10.0, False, "false"), (10.0, True, "false"), (35.0, False, "probe_reach"),
                            (35.0, True, "drop")):
        m = tb.Meter(pg, dict(row, drop=drop))
        m.void_ground_mm, m.void_ground_nom_mm = 5.0, nom
        assert m._void_class(True) == want, (nom, drop)
    m.void_ground_mm, m.void_ground_nom_mm = None, None              # no ground found under the foot
    assert m._void_class(True) == "drop"


def test_pct_s1():
    runs = [_run("W1.flat.walk.p0", "S1"), _run("W1.flat.walk.p0", "S2", progress_mm=480.0)]
    tb.add_ratios(runs)
    assert runs[1]["pct_s1"] == 96.0 and runs[0]["pct_s1"] == 100.0


# ------------------------------------------------------------------ provenance and CLI
def test_provenance_stamps():
    from model_fingerprint import robot_fingerprint
    p = tb.provenance()
    model = mujoco.MjModel.from_xml_path(os.path.join(HERE, "..", "pebble.xml"))
    assert p["fingerprint"] == robot_fingerprint(model)
    assert p["params_rev"] == rm.params_rev() and p["level"] == rm.params()["level"]
    assert p["decision"] == "D065" and set(p["imu_modes"]) == {"ideal", "noisy", "mount1.5"}
    assert p["gated_imu"] == ["ideal", "noisy"] and p["seed_tiers"]["train_min"] == 1000
    assert p["head"] and len(p["head"]) == 40 and p["date"].endswith("+00:00")
    assert p["code_diff_sha256"] == tb.code_diff_sha256()          # None on a clean tree, else the code's diff
    assert p["code_diff_sha256"] is None or len(p["code_diff_sha256"]) == 64
    json.dumps(p)


def test_cli_list_and_bad_args(capsys):
    assert tb.main(["--list"]) == 0
    assert "87 rows" in capsys.readouterr().out
    for bad in (["--stack", "S9"], ["--imu", "perfect"]):
        with pytest.raises(SystemExit):
            tb.main(bad)
