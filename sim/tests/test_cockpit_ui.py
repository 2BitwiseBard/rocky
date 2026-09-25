"""Static checks on the cockpit page (sim/cockpit_ui.html): it is one file of
CSS + HTML + JS served straight from disk, so a typo ships the moment it is
saved. No browser here (the phone / desktop pass is done by hand with Playwright).

Pinned: no duplicate ids; every $('#id') / getElementById the script names
exists; the script parses (node --check when node is installed) and its
brackets and backticks balance outside strings and comments; no hard-coded
http(s) host (the page must work on 127.0.0.1:8765 and on the tailnet);
the five tabs, the floating STOP, the wake-word card and the remote toggle
exist; every panel has a '?' into the guide; the Memory panel, the map's
go back / forget menu and the phone's situation ticker exist."""
import os
import re
import shutil
import subprocess
from html.parser import HTMLParser

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
UI = os.path.join(HERE, "..", "cockpit_ui.html")
TABS = ("drive", "talk", "make", "world", "robot")


@pytest.fixture(scope="module")
def page():
    return open(UI, encoding="utf-8").read()


@pytest.fixture(scope="module")
def script(page):
    m = re.findall(r"<script>(.*?)</script>", page, re.S)
    assert len(m) == 1, "one inline script expected"
    return m[0]


class _Ids(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids, self.tags = [], []

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        self.tags.append((tag, a))
        if "id" in a:
            self.ids.append(a["id"])


@pytest.fixture(scope="module")
def parsed(page):
    p = _Ids()
    p.feed(page.split("<script>")[0])
    return p


def test_no_duplicate_ids(parsed):
    seen, dups = set(), set()
    for i in parsed.ids:
        (dups if i in seen else seen).add(i)
    assert not dups, dups


def test_every_id_the_script_names_exists(parsed, script):
    ids = set(parsed.ids)
    named = set(re.findall(r"\$\('#([A-Za-z0-9_-]+)'\)", script)) | set(re.findall(r"getElementById\('([A-Za-z0-9_-]+)'\)", script))
    assert named, "the regex found nothing: the script changed style"
    assert named <= ids, sorted(named - ids)


def _code_only(js):
    """The script with comments, string / template text and regex literals
    removed, keeping the code inside template ${ } expressions — what is left
    is the bracket structure."""
    out, i, n = [], 0, len(js)
    stack = []                        # open template literals: brace depth inside each ${ }
    prev = "("                        # last significant code char (decides / = regex or divide)
    while i < n:
        c = js[i]
        if stack and stack[-1] == -1:                 # inside template text
            if c == "\\":
                i += 2
                continue
            if c == "`":
                stack.pop()
                out.append("`")
                prev = "`"
                i += 1
                continue
            if js.startswith("${", i):
                stack[-1] = 0                          # now in an expression at depth 0
                out.append("{")
                prev = "{"
                i += 2
                continue
            i += 1
            continue
        if js.startswith("//", i):
            j = js.find("\n", i)
            i = n if j < 0 else j
            continue
        if js.startswith("/*", i):
            i = js.index("*/", i) + 2
            continue
        if c in "'\"":
            j = i + 1
            while js[j] != c:
                j += 2 if js[j] == "\\" else 1
            i = j + 1
            prev = "a"
            continue
        if c == "`":
            out.append("`")
            stack.append(-1)
            i += 1
            continue
        if c == "/" and prev in "(,=:[!&|?{};+-*%<>~^" + "\n":
            j, cls = i + 1, False                      # a regex literal
            while cls or js[j] != "/":
                if js[j] == "\\":
                    j += 1
                elif js[j] == "[":
                    cls = True
                elif js[j] == "]":
                    cls = False
                j += 1
            i = j + 1
            while i < n and js[i].isalpha():
                i += 1
            prev = "a"
            continue
        if stack and stack[-1] >= 0:                  # inside a ${ } expression
            if c == "{":
                stack[-1] += 1
            elif c == "}":
                if stack[-1] == 0:
                    stack[-1] = -1                     # back to template text
                    out.append("}")
                    prev = "}"
                    i += 1
                    continue
                stack[-1] -= 1
        out.append(c)
        if not c.isspace():
            prev = c
        i += 1
    assert not stack, "unterminated template literal"
    return "".join(out)


def test_script_brackets_balance(script):
    s = _code_only(script)
    assert s.count("`") % 2 == 0, "odd number of backticks"
    for a, b in ("{}", "()", "[]"):
        assert s.count(a) == s.count(b), f"{a}{b}: {s.count(a)} vs {s.count(b)}"


def test_script_parses_with_node(script, tmp_path):
    node = shutil.which("node")
    if not node:
        pytest.skip("node not installed")
    f = tmp_path / "ui.mjs"
    f.write_text(script, encoding="utf-8")
    r = subprocess.run([node, "--check", str(f)], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr


def test_no_hard_coded_hosts(page):
    hits = re.findall(r"(?:https?|wss?)://[^\s'\"`<)]+", page)
    assert not hits, hits


def test_tabs_stop_wake_remote(parsed):
    ids = set(parsed.ids)
    for t in TABS:
        assert f"g-{t}" in ids
        bars = [a for tag, a in parsed.tags if tag == "button" and a.get("data-tab") == t]
        assert len(bars) == 2, (t, "one in the phone tab bar, one across the sidebar")
    for need in ("tabbar", "side-tabs", "pin", "fab-stop", "remote", "follow", "status-line",
                 "p-wake", "wake-name", "wake-gate", "guide", "q-wave", "q-hello", "q-shove", "q-reset",
                 "chase", "eye", "log", "mic", "handsfree"):
        assert need in ids, need


def test_memory_panel_ticker_and_map_menu(parsed, page, script):
    """Memory & awareness: the panel sits in the World group with its guide
    link, the situation ticker in the Drive group (a phone-only 44 px tap
    target), the map has its inline go back / forget menu, and the script
    talks to the memory, awareness and tool endpoints the server mounts."""
    ids = set(parsed.ids)
    for need in ("p-memory", "map-wrap", "map-menu", "mem-situation", "mem-find", "mem-find-go",
                 "mem-note", "mem-note-go", "mem-msg", "mem-objects", "mem-stats", "mem-obs", "mem-clear",
                 "mem-sit-now", "aw-interval", "aw-intervalv", "aw-curious", "aw-reactions", "aw-msg",
                 "sit-ticker", "sit-text", "sit-age", "map"):
        assert need in ids, need
    body = page.split("<script>")[0]
    world = body.split('id="g-world"')[1].split('id="g-robot"')[0]
    drive = body.split('id="g-drive"')[1].split('id="g-talk"')[0]
    assert 'id="p-memory"' in world and 'id="map-menu"' in world
    assert 'id="sit-ticker"' in drive
    assert re.search(r'<details id="p-memory"><summary>.*?data-guide="memory-and-awareness"', body)
    iv = next(a for tag, a in parsed.tags if a.get("id") == "aw-interval")
    assert (iv.get("type"), iv.get("min"), iv.get("max")) == ("range", "0", "60")
    css = page.split("</style>")[0]
    phone = css.split("@media (max-width: 900px)")[-1]
    tick = re.search(r"\n\s*#sit-ticker \{([^}]*)\}", phone).group(1).replace(" ", "")
    assert "display:flex" in tick and "min-height:44px" in tick
    assert re.search(r"\n\s*#sit-ticker \{ display:none; \}", css.split("@media (max-width: 900px)")[0])
    for url in ("/api/memory", "/api/awareness", "/api/tool/go_back_to", "/api/tool/find_object"):
        assert url in script, url
    for fn in ("drawMemory(g)", "renderMemory(s)", "memHit(", "memMenu(", "memPinAt(", "'contextmenu'"):
        assert fn in script, fn
    assert "confirm(" in script.split("async function memForget")[1].split("\n}")[0]
    assert "confirm(" not in script.split("async function memGoBack")[1].split("\n}")[0]


def test_every_panel_has_a_guide_link(page):
    for m in re.finditer(r'<details id="(p-[a-z]+)"[^>]*><summary>(.*?)</summary>', page, re.S):
        pid, summ = m.groups()
        if pid == "p-commands":                     # the command reference inside Help
            continue
        assert 'class="qhelp"' in summ and "data-guide=" in summ, pid


def test_streams_start_without_src(parsed):
    """The page decides which streams to open (remote mode, a phone on another
    tab, the eye hidden); an src in the markup would open both before it can."""
    imgs = {a["id"]: a for tag, a in parsed.tags if tag == "img" and a.get("id") in ("chase", "eye")}
    assert set(imgs) == {"chase", "eye"} and not any("src" in a for a in imgs.values())


def test_phone_media_query_and_tap_targets(page):
    css = page.split("</style>")[0]
    assert "@media (max-width: 900px)" in css
    phone = css.split("@media (max-width: 900px)")[-1]
    assert "min-height:44px" in phone.replace(" ", "")


def test_phone_stop_clearance_is_on_the_scroll_container(page):
    """The floating STOP sits over the bottom of the scroll area: the clearance
    has to be on <main> (what scrolls), or the last element under it (the
    console's run button on Drive) can never be tapped."""
    phone = page.split("</style>")[0].split("@media (max-width: 900px)")[-1]
    main_rule = re.search(r"\n\s*main, main\.nosidebar \{([^}]*)\}", phone).group(1)
    assert "padding-bottom:96px" in main_rule.replace(" ", "")
    side_rule = re.search(r"\n\s*#side \{([^}]*)\}", phone).group(1)
    assert "padding-bottom" not in side_rule


def test_hands_free_does_not_disable_the_mic(script):
    """disabled dims the button to 40 %, and with it the wake-word glow."""
    assert "micBtn.disabled" not in script


# ---- the wake-word match, run in node (the page is the only place a custom name is enforced)
WAKE_CASES = [
    # (name, transcript, expected class)
    ("Pebble", "pebble", "only"),
    ("Pebble", "hey pebble", "only"),
    ("Pebble", "Pebble, walk forward 30 cm", "contains"),
    ("Pebble", "ok peble walk forward", "contains"),          # one slip of whisper's
    ("Pebble", "walk to the pebbles", "none"),                # the name late in the sentence
    ("Pebble", "people walk forward", "none"),                # two letters off
    ("Pebble", "pebblestone walk", "none"),                   # no prefix rule
    ("Rocky", "the rocket is over there, walk forward", "none"),
    ("Rocky", "rocky stop", "contains"),
    ("Max", "max walk forward", "contains"),
    ("Max", "go to max speed", "none"),                       # a short name must come first
    ("Max", "mix walk forward", "none"),                      # no slips on a short name
    ("Bo", "bo walk", "none"),                                # too short to be a name at all
    ("Al", "also walk forward 30 cm", "none"),
    ("Mr Pebble", "mr pebble walk", "none"),                  # names are one word
    ("Pebble", "", "none"),
    ("Pebble", "um uh", "none"),
]


def _wake_block(script):
    m = re.search(r"// wake-match:begin[^\n]*\n(.*?)// wake-match:end", script, re.S)
    assert m, "the wake-match markers are gone"
    return m.group(1)


def test_wake_match_in_node(script, tmp_path):
    node = shutil.which("node")
    if not node:
        pytest.skip("node not installed")
    import json
    f = tmp_path / "wake.js"
    f.write_text(_wake_block(script) + "\nconst cases = " + json.dumps(WAKE_CASES) + ";\n"
                 "console.log(JSON.stringify(cases.map(([n, t]) => wakeClassFor(t, n))));\n", encoding="utf-8")
    r = subprocess.run([node, str(f)], capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    got = json.loads(r.stdout)
    bad = [(c, g) for c, g in zip(WAKE_CASES, got) if c[2] != g]
    assert not bad, bad


def test_name_rules_agree_with_the_server(script):
    """The page and cockpit_shared accept the same names."""
    import sys
    sys.path.insert(0, os.path.join(HERE, ".."))
    import cockpit_shared as cs
    js = re.search(r"const NAME_RE = /(.*?)/;", script).group(1)
    assert re.compile(js).pattern == cs._NAME.pattern
