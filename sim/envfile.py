"""envfile — this machine's settings from <repo>/rocky.env, for Python started without rocky.sh.

rocky.sh sources rocky.env (git-ignored; the tracked template rocky.env.example lists every
ROCKY_* variable the code reads, with its default). A script started directly — a cockpit
under systemd-run, a bench, brain_install, pytest — reads the same file here: load() applies
it with setdefault, so a variable already in the environment wins, as it does in rocky.sh.
sim/cockpit_brains.py and sim/brain_install.py call load() when they are imported.

The file is plain KEY=value lines: blank lines and # comments are skipped, a leading
`export ` is allowed, one pair of quotes around the value is stripped (single quotes: taken
literally), $NAME / ${NAME} are expanded (an unset name is empty) and so is a leading ~ on an
unquoted value. Nothing else of the shell is: keep rocky.env to that subset.
ROCKY_ENV_FILE names another file (/dev/null: none).

    python sim/envfile.py              # what rocky.env sets, and what the environment overrides
    python sim/envfile.py --llm-key    # the LLM server's key (rocky.sh brain uses it)
"""
from __future__ import annotations

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PATH = os.path.join(ROOT, "rocky.env")
_LINE = re.compile(r"^\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)=(.*)$")
_VAR = re.compile(r"\$(?:\{([A-Za-z_][A-Za-z0-9_]*)\}|([A-Za-z_][A-Za-z0-9_]*))")


def path():
    """The rocky.env this process reads: ROCKY_ENV_FILE, else <repo>/rocky.env."""
    return os.environ.get("ROCKY_ENV_FILE") or DEFAULT_PATH


def parse(text, env=None):
    """KEY=value text -> {key: value}. Pure: $NAME expands against the keys above it, then
    env (default os.environ); malformed lines are skipped."""
    env = os.environ if env is None else env
    out = {}
    for raw in str(text).splitlines():
        m = _LINE.match(raw)
        if not m:
            continue
        key, val = m.group(1), m.group(2).strip()
        quote = val[:1] if val[:1] in ("'", '"') else ""
        if quote:
            end = val.find(quote, 1)
            if end < 0:
                continue                                    # an unclosed quote: not a value
            val = val[1:end]
        else:
            val = re.split(r"\s+#", val, maxsplit=1)[0].strip()   # a trailing comment
        if quote != "'":
            val = _VAR.sub(lambda v: out.get(v.group(1) or v.group(2),
                                             env.get(v.group(1) or v.group(2), "")), val)
            if not quote and val.startswith("~"):
                val = os.path.expanduser(val)
        out[key] = val
    return out


def read(file=None):
    """{key: value} of rocky.env ({} when there is none or it cannot be read)."""
    try:
        with open(file or path(), encoding="utf-8") as f:
            return parse(f.read())
    except OSError:
        return {}


def load(file=None):
    """Apply rocky.env to os.environ, never overwriting a variable that is already set.
    Returns {key: value} of what it set (idempotent: a second call sets nothing)."""
    applied = {}
    for k, v in read(file).items():
        if k not in os.environ:
            os.environ[k] = applied[k] = v
    return applied


def env_list(name, default=()):
    """A comma-separated variable as a tuple (items stripped, empties dropped). Unset -> default;
    set but empty -> () (so ROCKY_QUARANTINED= means none)."""
    v = os.environ.get(name)
    if v is None:
        return tuple(default)
    return tuple(x.strip() for x in v.split(",") if x.strip())


def llm_key():
    """The LLM server's bearer key, '' when there is none: ROCKY_LLM_API_KEY, else the key in
    the file ROCKY_LLM_KEY_FILE names — its ROCKY_LLM_API_KEY= line, else its first *_KEY=
    line (a systemd environment.d file works as is), else its first line as a bare key."""
    if os.environ.get("ROCKY_LLM_API_KEY"):
        return os.environ["ROCKY_LLM_API_KEY"]
    f = os.path.expanduser(os.environ.get("ROCKY_LLM_KEY_FILE") or "")
    if not f:
        return ""
    try:
        with open(f, encoding="utf-8") as fh:
            text = fh.read()
    except OSError:
        return ""
    vals = parse(text, env={})
    if vals.get("ROCKY_LLM_API_KEY"):
        return vals["ROCKY_LLM_API_KEY"]
    for k, v in vals.items():
        if k.endswith("_KEY") and v:
            return v
    lines = [ln.strip() for ln in text.splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
    return lines[0] if lines and not vals and "=" not in lines[0] else ""


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["--llm-key"]:
        load()
        print(llm_key())
        return 0
    if argv:
        print("usage: python sim/envfile.py [--llm-key]", file=sys.stderr)
        return 2
    vals = read()
    print(f"{path()}: " + (f"{len(vals)} setting(s)" if vals else "none (the code's defaults apply)"))
    for k, v in vals.items():
        shown = "<set>" if "KEY" in k and "FILE" not in k else v
        over = "   (the environment overrides it)" if k in os.environ and os.environ[k] != v else ""
        print(f"  {k}={shown}{over}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
