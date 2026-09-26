"""D056: harness/capabilities.py regenerates today's tool definitions exactly.

Three layers of proof that the registry changes nothing a model sees:
  1. the snapshots taken BEFORE the registry existed (JSON files; the test skips
     when they are not on this machine — ROCKY_D056_SNAPSHOTS points at them);
  2. GOLDEN: the sha256 of those same snapshots (canonical JSON), pinned here, so
     the proof survives without the files (CI, another machine, a cleared /tmp);
  3. the live builders (local_brain.build_tools, cockpit_brains, server.build_server)
     — while they still carry their own copies, any drift between the two fails.
If a tool text or schema changes ON PURPOSE, update GOLDEN in the same commit and say
so; the snapshot files are a one-off record of 2026-09-25.

D057 ADDED four tools (where_am_i, name_place, places, forget_place: place recognition,
appended after `turn`, offered to the local brains only, not MCP) behind a new capability
flag, `places` (the cockpit's recognition switched on). Adding is allowed, changing is
not: every list built WITHOUT has_places is the D056 one, byte for byte (the D056 hashes,
no filtering), the lists with it are the D056 tools + the four appended (GOLDEN_D057).

F3 made the same four MCP tools too (mcp_args + the `mcp` surface; nothing else changed): the
MCP server offers them over a cockpit that reports `places` (recognition on). Every MCP list
built without has_places is still the D056 one (GOLDEN["mcp_rep"], GOLDEN["mcp_mock"]); with it,
the D056 rows + the four (GOLDEN_D057["mcp_rep"]).

    MUJOCO_GL=egl .venv/bin/python -m pytest harness/test_capabilities.py -q
"""
import ast
import asyncio
import hashlib
import json
import os
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import harness.capabilities as C                                   # noqa: E402
from harness.backend import CHORD_WORDS, GESTURES, SIGNED, MockBackend   # noqa: E402

SNAP_DIR = os.environ.get(
    "ROCKY_D056_SNAPSHOTS",
    "/tmp/claude-1000/-home-bitwisebard-Development-rocky/"
    "617aee55-a110-4e95-989e-f8423ee0b4fa/scratchpad/d056")

# the representative call the snapshots were taken with
REP_GESTURES = ['wave', 'sit', 'turn_in_place', 'sidestep', 'look_around', 'bow', 'shake', 'point_there']
REP_LEXICON = ['greeting', 'yes', 'no', 'acknowledge', 'found_it', 'thinking', 'error']

# sha256 of the canonical JSON (sort_keys, compact) of the 2026-09-25 snapshots
GOLDEN = {
    # build_tools(REP_GESTURES, REP_LEXICON, look=True, extra=cockpit_brains.EXTRA_TOOLS)
    "openai_rep": "469a6144f7308046c82e4007d205fe0b7caa5d9b47d6656bb758c4e451833fd8",
    # local_brain.TOOLS = build_tools(GESTURES, CHORD_WORDS)
    "openai_local_brain_TOOLS": "7efe2065850cf00fe3cf5ae52b47f95cba80c79e36a89848be3aaee7a60629a1",
    # cockpit_brains.TOOLS = build_tools(look=True, extra=EXTRA_TOOLS)
    "openai_cockpit_brains_TOOLS": "f932d346fe16bbc9274d9a1fd1501b0837c96aa797c1eed2972d44eb0660c4eb",
    # build_server(<every capability, live lists REP_*>).list_tools(): {name, description,
    # inputSchema, annotations} rows sorted by name
    "mcp_rep": "3753dc32f7c8ded1a99db7bb642c2c55f6963dbbdf0ba4d18b570be34cb19b1e",
    # build_server(MockBackend()).list_tools(), same rows
    "mcp_mock": "eec37bf895676b5a52c0046917424bb65d497232f4622471702a9608966d0aaf",
}
# D057: the tools added after the D056 snapshots (registry order), and the sha256 of the full lists
ADDED_D057 = ("where_am_i", "name_place", "places", "forget_place")
GOLDEN_D057 = {       # has_places=True (recognition on); regenerated 2026-09-25 for name_place's `rename`
    "openai_rep": "0f9b0f0daaa92e85b72416b17a9ca553dbfdf39914202b6f551d2eb54a09a505",
    "openai_cockpit_brains_TOOLS": "82039933acaad11e465d3f56d14e05bd000378eb6af862629925b021ab6e46fd",
    # F3: to_mcp_specs(rep_caps(has_places=True)) = build_server over a cockpit reporting `places`,
    # {name, description, inputSchema, annotations} rows sorted by name (the D056 rows + the four)
    "mcp_rep": "bb4b0fd415d5f4ed676d89f17dab1af8020725137eaa32e36815183b50e9166d",
}
# the no-cockpit snapshot's version at D056 (docs/TOOLS.md at HEAD): with has_places off the
# snapshot is still exactly that one (the D057 tools add nothing unless recognition is on)
D056_FALLBACK_VERSION = "99615678f55b"
# today's sets, as recorded in snapshot_sets.json
GATED_TODAY = frozenset({"compose_gesture", "find_object", "gesture", "go_back_to", "goto", "move", "turn"})
DISPATCH_TODAY = frozenset({"say", "gesture", "move", "goto", "stop", "scan_summary", "status", "look",
                            "list_gestures", "compose_gesture", "check_gesture", "save_gesture",
                            "find_object", "turn", "remember", "where_is", "recall", "go_back_to",
                            "forget"})


def _sha(obj):
    return hashlib.sha256(C.canonical_json(obj).encode("utf-8")).hexdigest()


def _pre_d057(tools):
    """An OpenAI tool list without the D057 additions (what the D056 snapshots saw)."""
    return [t for t in tools if t["function"]["name"] not in ADDED_D057]


def _added_d057(tools):
    return [t for t in tools if t["function"]["name"] in ADDED_D057]


def _snap(name):
    path = os.path.join(SNAP_DIR, name)
    if not os.path.exists(path):
        pytest.skip(f"no D056 snapshot at {path} (the GOLDEN hashes still pin it)")
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def rep_caps(**kw):
    args = dict(has_eye=True, has_memory=True, is_cockpit=True)
    args.update(kw)
    return C.build(REP_GESTURES, REP_LEXICON, list(SIGNED), **args)


def _rows_from_specs(specs):
    return sorted(({"name": s["name"], "description": s["description"], "inputSchema": s["input_schema"],
                    "annotations": s["annotations"]} for s in specs), key=lambda r: r["name"])


def _rows_from_mcp(tools):
    return sorted(({"name": t["name"], "description": t["description"], "inputSchema": t["inputSchema"],
                    "annotations": t.get("annotations", {})} for t in tools), key=lambda r: r["name"])


class FullBackend(MockBackend):
    """Every capability the MCP server looks for, with the representative live lists."""

    def live_lists(self):
        return list(REP_GESTURES), list(REP_LEXICON)

    async def look(self):
        return {"ok": True}

    async def find_object(self, name, max_steps=None):
        return {"ok": True}

    async def remember(self, note):
        return {"ok": True}

    async def where_is(self, name):
        return {"ok": True}

    async def recall(self, query="", k=5):
        return {"ok": True}

    async def go_back_to(self, name):
        return {"ok": True}

    async def forget(self, name):
        return {"ok": True}


def _server_rows(backend):
    from harness.server import build_server
    srv = build_server(backend)
    return [t.model_dump(mode="json", exclude_none=True, by_alias=True)
            for t in asyncio.run(srv.list_tools())]


# ---------------------------------------------------------------- OpenAI (local brains)
def test_openai_tools_equal_the_snapshot_byte_for_byte():
    snap = _snap("snapshot_openai_tools.json")
    call = snap["call"]
    caps = C.build(call["gestures"], call["lexicon"], call["signed"],
                   has_eye=True, has_memory=True, is_cockpit=True)
    mine = C.to_openai_tools(caps)                                # recognition off: no filtering needed
    assert C.canonical_json(mine) == C.canonical_json(snap["tools"])
    assert json.dumps(mine) == json.dumps(snap["tools"])          # same key order too
    # D057 (recognition on): the additions come after every D056 tool, in registry order
    full = C.to_openai_tools(C.build(call["gestures"], call["lexicon"], call["signed"],
                                     has_eye=True, has_memory=True, is_cockpit=True, has_places=True))
    assert full == mine + _added_d057(full)
    assert [t["function"]["name"] for t in _added_d057(full)] == list(ADDED_D057)


def test_openai_variants_equal_the_snapshot():
    v = _snap("snapshot_openai_tools_variants.json")
    by = {k.split(" ")[0]: val for k, val in v.items()}
    assert C.to_openai_tools(C.build(GESTURES, CHORD_WORDS)) == by["local_brain.TOOLS"]     # no D057 tool there
    assert C.to_openai_tools(C.build(None, None, has_eye=True, has_memory=True,
                                     is_cockpit=True)) == by["cockpit_brains.TOOLS"]


def test_openai_tools_match_the_golden_hashes():
    # recognition off (no `places`): the D056 lists, unchanged and UNFILTERED (the pins taken
    # before the registry existed) — "off = exactly as before" for every model's tool list
    assert _sha(C.to_openai_tools(rep_caps())) == GOLDEN["openai_rep"]
    assert _sha(C.to_openai_tools(C.build(GESTURES, CHORD_WORDS))) == GOLDEN["openai_local_brain_TOOLS"]
    assert _sha(C.to_openai_tools(C.build(None, None, has_eye=True, has_memory=True, is_cockpit=True))) \
        == GOLDEN["openai_cockpit_brains_TOOLS"]
    # D057, recognition on: the D056 tools (still unchanged) + the four place tools, as added on purpose
    full_rep = C.to_openai_tools(rep_caps(has_places=True))
    full_cb = C.to_openai_tools(C.build(None, None, has_eye=True, has_memory=True, is_cockpit=True,
                                        has_places=True))
    assert _sha(_pre_d057(full_rep)) == GOLDEN["openai_rep"]
    assert _sha(_pre_d057(full_cb)) == GOLDEN["openai_cockpit_brains_TOOLS"]
    assert _sha(full_rep) == GOLDEN_D057["openai_rep"]
    assert _sha(full_cb) == GOLDEN_D057["openai_cockpit_brains_TOOLS"]


def test_openai_tools_equal_todays_builders():
    import harness.local_brain as lb
    import sim.cockpit_brains as cb
    live = lb.build_tools(REP_GESTURES, REP_LEXICON, signed=SIGNED, look=True, extra=cb.EXTRA_TOOLS)
    assert json.dumps(C.to_openai_tools(rep_caps())) == json.dumps(live)
    live_on = lb.build_tools(REP_GESTURES, REP_LEXICON, signed=SIGNED, look=True,
                             extra=cb.EXTRA_TOOLS + cb.PLACE_TOOL_DEFS)
    assert json.dumps(C.to_openai_tools(rep_caps(has_places=True))) == json.dumps(live_on)
    assert C.to_openai_tools(C.build(lb._G, lb._W)) == lb.TOOLS
    # cockpit_brains.TOOLS is the static D056 list; with recognition on the cockpit's models get
    # it + PLACE_TOOL_DEFS (tools_for)
    assert C.to_openai_tools(C.build(None, None, has_eye=True, has_memory=True, is_cockpit=True)) == cb.TOOLS
    full = C.to_openai_tools(C.build(None, None, has_eye=True, has_memory=True, is_cockpit=True, has_places=True))
    assert full == cb.TOOLS + cb.PLACE_TOOL_DEFS


def test_anthropic_tools_are_claude_modes_conversion():
    oa = C.to_openai_tools(rep_caps())
    assert C.to_anthropic_tools(rep_caps()) == [
        {"name": t["function"]["name"], "description": t["function"]["description"],
         "input_schema": t["function"]["parameters"]} for t in oa]


def test_live_enums_are_filled_and_dropped_when_empty():
    caps = rep_caps()
    fn = {t["function"]["name"]: t["function"] for t in C.to_openai_tools(caps)}
    assert fn["say"]["parameters"]["properties"]["word"]["enum"] == REP_LEXICON
    assert fn["gesture"]["parameters"]["properties"]["name"]["enum"] == REP_GESTURES
    empty = {t["function"]["name"]: t["function"] for t in C.to_openai_tools(C.build())}
    assert "enum" not in empty["say"]["parameters"]["properties"]["word"]
    assert "enum" not in empty["gesture"]["parameters"]["properties"]["name"]
    assert C.LIVE not in json.dumps(C.to_openai_tools(caps))
    # generators hand out copies: mutating one never reaches the snapshot
    fn["say"]["parameters"]["properties"]["word"]["enum"].append("dance")
    assert "dance" not in json.dumps(caps)


# ---------------------------------------------------------------- MCP
def test_mcp_specs_equal_the_snapshot():
    snap = _snap("snapshot_mcp_tools.json")
    mine = {s["name"]: s for s in C.to_mcp_specs(rep_caps())}
    theirs = {t["name"]: t for t in snap["tools"]}
    assert set(mine) == set(theirs)
    for name, t in theirs.items():
        assert mine[name]["description"] == t["description"], name
        assert mine[name]["input_schema"] == t["inputSchema"], name
        assert mine[name]["annotations"] == t.get("annotations", {}), name
    assert snap["instructions"] == C.MCP_INSTRUCTIONS
    mock = _snap("snapshot_mcp_tools_mock.json")
    assert _rows_from_specs(C.to_mcp_specs(C.build(GESTURES, CHORD_WORDS))) == _rows_from_mcp(mock["tools"])


def test_mcp_specs_match_the_golden_hashes():
    assert _sha(_rows_from_specs(C.to_mcp_specs(rep_caps()))) == GOLDEN["mcp_rep"]
    assert _sha(_rows_from_specs(C.to_mcp_specs(C.build(GESTURES, CHORD_WORDS)))) == GOLDEN["mcp_mock"]


def test_mcp_specs_equal_todays_server():
    full = _server_rows(FullBackend())
    assert [s["name"] for s in C.to_mcp_specs(rep_caps())] == [t["name"] for t in full]   # order too
    assert _rows_from_specs(C.to_mcp_specs(rep_caps())) == _rows_from_mcp(full)
    flags = C.backend_flags(MockBackend())
    assert flags == {"has_eye": False, "has_memory": False, "is_cockpit": False}
    mock = _server_rows(MockBackend())
    assert _rows_from_specs(C.to_mcp_specs(C.build(GESTURES, CHORD_WORDS, **flags))) == _rows_from_mcp(mock)
    from harness.server import build_server
    assert build_server(MockBackend()).instructions == C.MCP_INSTRUCTIONS


def test_backend_flags_for_the_proxies():
    from harness.cockpit_backend import AutoBackend, CockpitBackend
    url = "http://127.0.0.1:9"                       # never contacted: flags read attributes only
    assert C.backend_flags(CockpitBackend(url)) == {"has_eye": True, "has_memory": True, "is_cockpit": True}
    auto = C.backend_flags(AutoBackend(url, alive=lambda u: False))
    assert auto["has_eye"] and auto["has_memory"]
    # the MCP list on the proxies is every MCP tool in the registry but (F3) the place tools,
    # which need the cockpit to report its recognition on (the `places` flag)
    names = [s["name"] for s in C.to_mcp_specs(C.build(GESTURES, CHORD_WORDS, **auto))]
    assert names == [t.name for t in C.REGISTRY if "mcp" in t.surfaces and "places" not in t.requires]


# ---------------------------------------------------------------- sets
def test_gated_names_equal_today():
    assert C.gated_names(rep_caps()) == GATED_TODAY == C.GATED_NAMES
    import sim.cockpit_brains as cb
    assert C.gated_names(rep_caps()) == frozenset(cb.GATED)


def test_registry_is_the_cockpit_dispatch_set():
    assert set(C.TOOL_NAMES) == DISPATCH_TODAY | set(ADDED_D057)
    assert C.TOOL_NAMES[-len(ADDED_D057):] == ADDED_D057          # appended: the D056 order is kept
    import sim.cockpit_brains as cb
    assert set(C.TOOL_NAMES) == set(cb.dispatch_names())
    assert set(cb.TOOL_NAMES) == DISPATCH_TODAY and cb.PLACE_TOOLS == ADDED_D057
    assert set(C.MEMORY_NAMES) == set(cb.MEMORY_TOOLS)
    # the tools a model is never offered (recognition on: every model-facing tool): exactly the
    # executor-only ones
    offered = {t["function"]["name"] for t in C.to_openai_tools(rep_caps(has_places=True))}
    internal = {t.name for t in C.REGISTRY if t.surfaces == ("internal",)}
    assert set(C.TOOL_NAMES) - offered == internal == {"turn"}


def test_sets_equal_the_snapshot():
    snap = _snap("snapshot_sets.json")
    assert C.gated_names(rep_caps()) == frozenset(snap["GATED"]) == frozenset(snap["GATED_raw"])
    assert set(C.TOOL_NAMES) - set(ADDED_D057) == set(snap["cockpit_dispatch"])
    offered = {t["function"]["name"] for t in C.to_openai_tools(rep_caps())}
    assert set(snap["cockpit_dispatch"]) - offered == set(snap["cockpit_not_offered_to_models"])
    assert set(snap["MEMORY_TOOLS"]) == set(C.MEMORY_NAMES)


def test_cockpit_sim_tool_methods_are_registry_tools():
    """Brains.tool falls through to getattr(sim, 'tool_' + name): every such method is a tool."""
    tree = ast.parse(open(os.path.join(ROOT, "sim", "cockpit.py"), encoding="utf-8").read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "CockpitSim")
    methods = {f.name[len("tool_"):] for f in cls.body
               if isinstance(f, (ast.FunctionDef, ast.AsyncFunctionDef)) and f.name.startswith("tool_")}
    assert methods and methods <= set(C.TOOL_NAMES)


# ---------------------------------------------------------------- the texts' sources
def _module_constants(path, names):
    tree = ast.parse(open(os.path.join(ROOT, path), encoding="utf-8").read())
    out = {}
    for n in tree.body:
        if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name) \
                and n.targets[0].id in names:
            out[n.targets[0].id] = ast.literal_eval(n.value)
    return out


def test_constants_and_texts_equal_their_sources():
    import harness.local_brain as lb
    import sim.cockpit_brains as cb
    import scene_memory                              # on sys.path once cockpit_brains is imported
    assert (C.GOTO_REACH_M, C.MOVE_MAX_M) == (lb.GOTO_REACH_M, lb.MOVE_MAX_M)
    assert C.GOTO_DOC == lb.GOTO_DOC and C.MOVE_DOC == lb.MOVE_DOC
    for name in ("COMPOSE_DOC", "FIND_DOC", "REMEMBER_DOC", "WHERE_IS_DOC", "RECALL_DOC", "GO_BACK_DOC",
                 "FORGET_DOC", "FIND_MAX_STEPS", "FIND_MAX_STEPS_CAP"):
        assert getattr(C, name) == getattr(cb, name), name
    assert C.SPAWN_NAME == scene_memory.SPAWN_NAME
    ck = _module_constants("sim/cockpit.py", {"GOTO_CAP_S", "V_GOTO"})
    assert (C.GOTO_CAP_S, C.GOTO_SPEED_MM_S) == (ck["GOTO_CAP_S"], ck["V_GOTO"])


def test_registry_specs_are_well_formed():
    seen = set()
    for t in C.REGISTRY:
        assert t.name not in seen, t.name
        seen.add(t.name)
        assert t.kind in C.KINDS
        assert t.backend_method == t.name                          # D056: the same name everywhere
        assert t.parameters.get("type") == "object"
        assert ("mcp" in t.surfaces) == (t.mcp_input_schema() is not None)
        assert set(t.annotations) <= {"readOnlyHint", "idempotentHint", "destructiveHint", "openWorldHint"}
        for live in (v.get(C.LIVE) for v in t.parameters.get("properties", {}).values()):
            assert live in (None,) + C.LIVE_LISTS
    assert "stop" not in C.GATED_NAMES
    with pytest.raises(ValueError):
        C.ToolSpec("x", "dance", "d", {"type": "object", "properties": {}})


# ---------------------------------------------------------------- the snapshot + version
def test_version_is_stable_and_changes_with_the_robot():
    a, b = rep_caps(), rep_caps()
    assert a == b and a["version"] == b["version"] == C.snapshot_version(a)
    assert len(a["version"]) == 12 and int(a["version"], 16) >= 0
    more = C.build(REP_GESTURES + ["dance"], REP_LEXICON, list(SIGNED),
                   has_eye=True, has_memory=True, is_cockpit=True)
    assert more["version"] != a["version"]
    assert "dance" in more["gestures"]
    assert rep_caps(has_eye=False)["version"] != a["version"]
    assert C.build(REP_GESTURES, REP_LEXICON + ["amaze"], has_eye=True, has_memory=True,
                   is_cockpit=True)["version"] != a["version"]
    env = dict(a["envelope"], goto_reach_m=1.0)
    moved = rep_caps(envelope=env)
    assert moved["version"] != a["version"]
    goto = next(t for t in moved["tools"] if t["name"] == "goto")
    assert "within ~1 m" in goto["description"] and "within ~1.5 m" not in goto["description"]
    json.dumps(a)                                                     # a plain JSON dict


def test_snapshot_shape_envelope_and_robot():
    caps = C.fallback_caps()
    assert set(caps) == {"format", "version", "capabilities", "gestures", "signed", "lexicon",
                         "envelope", "robot", "tools"}
    assert caps["capabilities"] == ["cockpit", "eye", "memory", "places"]
    assert [t["name"] for t in caps["tools"]] == list(C.TOOL_NAMES)
    # D057 off: the D056 snapshot exactly (its capabilities, its tools, its version)
    off = C.fallback_caps(has_places=False)
    assert off["capabilities"] == ["cockpit", "eye", "memory"]
    assert [t["name"] for t in off["tools"]] == [n for n in C.TOOL_NAMES if n not in ADDED_D057]
    assert off["version"] == D056_FALLBACK_VERSION
    env = caps["envelope"]
    assert env["goto_reach_m"] == 1.5 and env["goto_timeout_s"] == 40.0
    assert env["source"].startswith("gait/"), env                   # the gait is importable here
    assert env["speed_m_s"] == pytest.approx(0.0455, abs=5e-4)
    assert env["turn_rad_s"] == pytest.approx(0.246, abs=2e-3)
    assert env["step_height_mm"] == 24.0
    rb = caps["robot"]
    assert rb["legs"] == 5 and rb["joints_per_leg"] == 3 and rb["joint_names"] == ["yaw", "hip", "knee"]
    # capability flags gate the tool set
    bare = C.build(GESTURES, CHORD_WORDS)
    assert [t["name"] for t in bare["tools"]] == ["say", "gesture", "move", "goto", "stop", "scan_summary",
                                                   "status", "list_gestures"]


# ---------------------------------------------------------------- docs/TOOLS.md + CLI
def test_markdown_is_deterministic_and_complete():
    caps = C.fallback_caps()
    md = C.to_markdown(caps)
    assert md == C.to_markdown(C.fallback_caps())
    for t in C.REGISTRY:
        assert f"### `{t.name}`" in md, t.name
    assert caps["version"] in md
    for g in GESTURES:
        assert f"`{g}`" in md
    assert "Description (MCP):" in md and "Description (local brains):" in md
    assert md.endswith("\n") and "\t" not in md
    # the executor is CHECKED against the registry, not generated from it (a new tool still
    # needs its handler): the page must not claim otherwise, and the check it names must exist
    assert "executor (`/api/tool/<name>`) is checked against it" in md
    assert "executor (`/api/tool/<name>`) are generated" not in md
    with open(os.path.join(ROOT, "sim", "cockpit_brains.py"), encoding="utf-8") as f:
        assert "\ndef registry_problems(" in f.read()
    # every envelope row carries its meaning; step_height_mm reads as the lift, not a stride
    for k in caps["envelope"]:
        if k != "source":
            assert f"| `{k}` | {caps['envelope'][k]} | {C._cell(C.ENVELOPE_FIELDS[k])} |" in md, k
    assert "LIFTS" in C.ENVELOPE_FIELDS["step_height_mm"] and "not the stride" in C.ENVELOPE_FIELDS["step_height_mm"]


def test_every_envelope_field_has_a_meaning():
    """A field published without its meaning is how step_height_mm (a lift height) came to read as a
    step length: the gait's envelope, the fallback and the glossary name the same fields."""
    fields = set(C.ENVELOPE_FIELDS)
    assert set(C.default_envelope()) - {"source"} == fields
    assert set(C.FALLBACK_ENVELOPE) - {"source"} == fields
    assert all(v.strip() for v in C.ENVELOPE_FIELDS.values())
    # the glossary lives in the markdown only: the snapshot (and so its version) still carries
    # plain numbers, so /api/capabilities and the MCP server see no change
    env = C.fallback_caps()["envelope"]
    assert all(isinstance(v, (int, float)) and not isinstance(v, bool)
               for k, v in env.items() if k != "source"), env


def _cli(*args):
    env = dict(os.environ, MUJOCO_GL=os.environ.get("MUJOCO_GL", "egl"))
    return subprocess.run([sys.executable, "-m", "harness.capabilities", *args], cwd=ROOT, env=env,
                          capture_output=True, text=True, timeout=120)


def test_cli_check_round_trips(tmp_path):
    out = tmp_path / "TOOLS.md"
    r = _cli("--md", "--out", str(out))
    assert r.returncode == 0, r.stderr
    assert out.read_text(encoding="utf-8") == C.to_markdown(C.fallback_caps())
    assert _cli("--md", "--check", str(out)).returncode == 0
    out.write_text(out.read_text(encoding="utf-8") + "hand edit\n", encoding="utf-8")
    r = _cli("--md", "--check", str(out))
    assert r.returncode == 1 and "stale" in r.stderr
    assert _cli("--md", "--check", str(tmp_path / "missing.md")).returncode == 1
    r = _cli("--md")
    assert r.returncode == 0 and r.stdout == C.to_markdown(C.fallback_caps())
    r = _cli("--json")
    assert r.returncode == 0
    assert json.loads(r.stdout)["version"] == C.fallback_caps()["version"]


def test_envelope_and_robot_fall_back_when_the_gait_is_missing(monkeypatch):
    with_gait = C.to_openai_tools(C.fallback_caps())
    mcp_with_gait = C.to_mcp_specs(C.fallback_caps())

    def gone(mod):
        raise ImportError(f"no {mod}")
    monkeypatch.setattr(C, "_gait_import", gone)
    C._envelope_cached.cache_clear()
    C._robot_cached.cache_clear()
    try:
        env, rb = C.default_envelope(), C.default_robot()
        assert env == C.FALLBACK_ENVELOPE and env["source"].startswith("fallback")
        assert rb == C.FALLBACK_ROBOT
        caps = C.fallback_caps()
        # the texts do not depend on where the envelope came from: same tool lists either way
        assert C.to_openai_tools(caps) == with_gait
        assert C.to_mcp_specs(caps) == mcp_with_gait
    finally:
        C._envelope_cached.cache_clear()
        C._robot_cached.cache_clear()
    monkeypatch.undo()
    assert C.default_envelope()["source"].startswith("gait/")


# ---------------------------------------------------------------- D057: the place tools
def test_place_tools_are_cockpit_tools_offered_only_while_recognition_is_on():
    specs = {n: C.BY_NAME[n] for n in ADDED_D057}
    assert {n: s.kind for n, s in specs.items()} == {"where_am_i": "query", "name_place": "memory",
                                                    "places": "query", "forget_place": "memory"}
    for n, s in specs.items():
        # the place memory lives in the cockpit, and the tools exist only while recognition is on
        assert s.requires == frozenset({"cockpit", "places"}), n
        assert s.surfaces == ("openai", "mcp") and s.mcp_args is not None, n   # F3: MCP tools too
        assert not s.gated, n                                     # none of them moves the robot
        assert n not in C.MEMORY_NAMES, n                         # not the scene-memory group (mem_<name>)
    assert specs["forget_place"].annotations == {"readOnlyHint": False, "destructiveHint": True}
    assert specs["where_am_i"].annotations == specs["places"].annotations == {"readOnlyHint": True}
    # name_place is NOT idempotent: new=true on two visits stores two places (review 2026-09-25)
    assert specs["name_place"].annotations == {"readOnlyHint": False}
    np_ = specs["name_place"].parameters
    assert np_["required"] == ["name"] and np_["properties"]["new"]["type"] == "boolean"
    assert np_["properties"]["rename"]["type"] == "boolean"
    # where they appear: a cockpit list with recognition on (F3: the MCP list too), nothing without `places`
    assert set(ADDED_D057) <= set(C.tool_names(C.fallback_caps(), "openai"))
    assert set(ADDED_D057) <= set(C.tool_names(C.fallback_caps(), "mcp"))
    assert not set(ADDED_D057) & set(C.tool_names(C.build(GESTURES, CHORD_WORDS, has_eye=True, has_memory=True)))
    assert not set(ADDED_D057) & set(C.tool_names(C.build(None, None, has_eye=True, has_memory=True,
                                                          is_cockpit=True)))       # recognition off
    assert not set(ADDED_D057) & set(C.tool_names(C.build(None, None, has_places=True)))   # no cockpit
    assert C.gated_names(C.fallback_caps()) == GATED_TODAY
    md = C.to_markdown(C.fallback_caps())
    for n in ADDED_D057:
        assert f"### `{n}`" in md


def test_the_cockpit_executor_routes_the_place_tools(monkeypatch):
    import sim.cockpit_brains as cb
    assert [t["function"]["name"] for t in cb.PLACE_TOOL_DEFS] == list(ADDED_D057)
    assert not {t["function"]["name"] for t in cb.EXTRA_TOOLS} & set(ADDED_D057)   # EXTRA_TOOLS is as it was
    assert cb.dispatch_names() == tuple(cb.TOOL_NAMES) + ADDED_D057
    assert all(cb.OWN_ROUTES[n] == n for n in ADDED_D057)            # Brains.<name> runs each
    # the executor runs them whatever the flag (/api/tool answers 'recognition is off');
    # registry_problems checks their routes once `places` is among the flags
    b = cb.Brains(type("S", (), {"frames": {"eye": (0, b"")}, "memory": object()})(),
                  base_url="http://127.0.0.1:1/v1", api_key="x", conf_path=None)
    flags = {"has_eye": True, "has_memory": True, "is_cockpit": True, "has_places": True}

    def place_problems():                  # (this bare sim has no tool_<name> methods: those are not asked)
        return [p for p in cb.registry_problems(b, flags) if p.split(":")[0] in ADDED_D057]
    assert place_problems() == []
    monkeypatch.setattr(cb, "OWN_ROUTES", {k: v for k, v in cb.OWN_ROUTES.items() if k != "where_am_i"})
    assert [p.split(" (")[0] for p in place_problems()] == ["where_am_i: Brains.tool accepts it but has no route"]
    monkeypatch.undo()
    monkeypatch.setattr(cb, "PLACE_TOOLS", tuple(n for n in cb.PLACE_TOOLS if n != "places"))
    assert place_problems() == ["places: a registry tool this cockpit has, but Brains.tool refuses it"]
    monkeypatch.undo()
    no_places = dict(flags, has_places=False)
    monkeypatch.setattr(cb, "PLACE_TOOLS", ())
    assert not [p for p in cb.registry_problems(b, no_places) if p.split(":")[0] in ADDED_D057]


# ---------------------------------------------------------------- F3: the place tools on MCP
def test_place_tools_mcp_schemas_follow_the_cockpit_executor():
    """The MCP arguments are Brains.where_am_i() / name_place(name, new=False, rename=False) /
    places() / forget_place(name): the same names, defaults and required set as the
    registry's OpenAI schema (the executor's contract), as FastMCP publishes them."""
    specs = {s["name"]: s for s in C.to_mcp_specs(C.fallback_caps())}
    empty = lambda n: {"properties": {}, "title": f"{n}Arguments", "type": "object"}    # noqa: E731
    assert specs["where_am_i"]["input_schema"] == empty("where_am_i")
    assert specs["places"]["input_schema"] == empty("places")
    assert specs["name_place"]["input_schema"] == {
        "properties": {"name": {"title": "Name", "type": "string"},
                       "new": {"default": False, "title": "New", "type": "boolean"},
                       "rename": {"default": False, "title": "Rename", "type": "boolean"}},
        "required": ["name"], "title": "name_placeArguments", "type": "object"}
    assert specs["forget_place"]["input_schema"] == {
        "properties": {"name": {"title": "Name", "type": "string"}},
        "required": ["name"], "title": "forget_placeArguments", "type": "object"}
    for n in ADDED_D057:
        spec, s = C.BY_NAME[n], specs[n]
        # the same argument names, in the same order, and the same required set as the local brains'
        assert list(s["input_schema"]["properties"]) == list(spec.parameters["properties"]), n
        assert s["input_schema"].get("required", []) == spec.parameters.get("required", []), n
        for arg, p in s["input_schema"]["properties"].items():
            assert p["type"] == spec.parameters["properties"][arg]["type"], (n, arg)
        # one text for both surfaces (no MCP-only description), the same annotations, never gated
        assert s["description"] == next(t for t in C.fallback_caps()["tools"] if t["name"] == n)["description"]
        assert s["annotations"] == spec.annotations and not s["gated"], n
        assert s["backend_method"] == n and s["requires"] == ["cockpit", "places"], n


def test_mcp_specs_with_places_are_the_d056_list_plus_the_four():
    off, on = C.to_mcp_specs(rep_caps()), C.to_mcp_specs(rep_caps(has_places=True))
    assert [s["name"] for s in on] == [s["name"] for s in off] + list(ADDED_D057)     # appended, in order
    assert on[:len(off)] == off                                  # every D056 MCP tool unchanged
    assert _sha(_rows_from_specs(off)) == GOLDEN["mcp_rep"]
    assert _sha(_rows_from_specs(on[:len(off)])) == GOLDEN["mcp_rep"]
    assert _sha(_rows_from_specs(on)) == GOLDEN_D057["mcp_rep"]
    # the mock's list, and every list without a cockpit, has none of them (places alone is not enough)
    assert _sha(_rows_from_specs(C.to_mcp_specs(C.build(GESTURES, CHORD_WORDS, has_places=True)))) \
        == GOLDEN["mcp_mock"]
    # F3 changed no OpenAI list (the local brains' texts and schemas, recognition on or off)
    assert _sha(C.to_openai_tools(rep_caps(has_places=True))) == GOLDEN_D057["openai_rep"]
    assert C.fallback_caps(has_places=False)["version"] == D056_FALLBACK_VERSION
