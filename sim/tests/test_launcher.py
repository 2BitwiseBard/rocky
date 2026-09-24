"""rocky.sh stays honest about itself (D052).

The launcher's help used to print a fixed line range (it hid the RL/CAD half as
commands were added) and `train-walk` passed an --env the trainer never had, so
every run died in argparse. These are cheap text checks, no venv needed: CI runs
them on a bare checkout.
"""
import os
import re
import shutil
import subprocess

import pytest

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
LAUNCHER = os.path.join(REPO, "rocky.sh")
HELP_ALIASES = {"-h", "--help"}          # spellings of `help`, not commands of their own

pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")


def _text():
    with open(LAUNCHER) as f:
        return f.read()


def _header_commands(text):
    """Every `rocky.sh X` that opens a line of the leading comment block."""
    cmds = set()
    for line in text.splitlines()[1:]:
        if not line.startswith("#"):
            break
        m = re.match(r"#\s+rocky\.sh\s+([a-z][a-z-]*)", line)
        if m:
            cmds.add(m.group(1))
    return cmds


def _case_labels(text):
    """The labels of the top-level `case "$cmd" in` (two-space indent, `*)` excluded)."""
    body = text[text.index('case "$cmd" in'):]
    labels = set()
    for m in re.finditer(r"^  ([a-z|-][a-z|-]*)\)", body, flags=re.M):
        labels.update(m.group(1).split("|"))
    return labels - HELP_ALIASES


def _env():
    env = dict(os.environ)
    env.pop("ROCKY_REPO", None)          # the script must find itself, not an inherited path
    return env


def test_bash_syntax():
    r = subprocess.run(["bash", "-n", LAUNCHER], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_header_documents_every_command_and_no_ghosts():
    text = _text()
    header, labels = _header_commands(text), _case_labels(text)
    assert header, "no `rocky.sh X` lines found in the header"
    assert not header - labels, f"documented but no case label: {sorted(header - labels)}"
    assert not labels - header, f"case label but undocumented: {sorted(labels - header)}"


def test_help_lists_every_command():
    r = subprocess.run(["bash", LAUNCHER, "help"], capture_output=True, text=True,
                       env=_env(), timeout=30)
    assert r.returncode == 0, r.stderr
    for label in _case_labels(_text()):
        assert re.search(rf"rocky\.sh {re.escape(label)}\b", r.stdout), f"help omits {label}"


def test_train_envs_are_trainer_choices():
    """Every `train_ppo.py --env X` the launcher passes is one argparse accepts."""
    with open(os.path.join(REPO, "sim", "train_ppo.py")) as f:
        src = f.read()
    m = re.search(r'"--env".*?choices=\[([^\]]*)\]', src, flags=re.S)
    assert m, "train_ppo.py --env choices not found"
    choices = set(re.findall(r'"([^"]+)"', m.group(1)))
    passed = set(re.findall(r"train_ppo\.py --env (\S+)", _text()))
    assert passed, "rocky.sh passes no --env to train_ppo.py"
    assert passed <= choices, f"rocky.sh passes {sorted(passed - choices)}, trainer takes {sorted(choices)}"
