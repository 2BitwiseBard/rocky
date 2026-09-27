"""Repo-wide pytest setup: every suite runs on the REFERENCE setup, as CI does.

sim/envfile.py applies <repo>/rocky.env to any Python that imports the cockpit's brains,
pytest included, so one machine's rocky.env (other model ids, a quarantine or co-resident
list, a memory or gesture directory) would change what the tests see. This points
ROCKY_ENV_FILE at /dev/null for the session unless the caller set ROCKY_ENV_FILE already:

    ROCKY_ENV_FILE=rocky.env .venv/bin/python -m pytest ...   # test against this machine's settings

It runs before any test module is imported, and the subprocesses the tests start (a bench's
own cockpit, the MCP server) inherit it. `rocky.sh test` does the same.
"""
import os

os.environ.setdefault("ROCKY_ENV_FILE", os.devnull)
