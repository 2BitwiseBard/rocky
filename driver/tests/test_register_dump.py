"""B79 (D063): bench/register_dump.py archives STS 0..87 (the factory block with
the hidden 85/86 acceleration ramp) and SCS 0..83, falls back to the old 0..73
when a servo refuses the long read, and --diff still reads a pre-D063 dump."""
import csv
import os
import subprocess
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.normpath(os.path.join(HERE, "..", "..", "bench"))
sys.path.append(BENCH)                                                  # appended: shadows nothing
import register_dump as rd                                               # noqa: E402
from rocky_driver import BusError, FeetechBus, Family, make_pebble_mock  # noqa: E402
from rocky_driver.registers import MAPS                                  # noqa: E402


def run_dump(out, *extra):
    r = subprocess.run([sys.executable, os.path.join(BENCH, "register_dump.py"), "--mock",
                        "--out", str(out), *extra], capture_output=True, text=True, timeout=120, cwd=BENCH)
    assert r.returncode == 0, r.stdout + r.stderr
    return r.stdout


def read_csv(path):
    with open(path) as f:
        return list(csv.reader(f))


def test_every_mapped_register_is_inside_its_family_span():
    for fam, m in MAPS.items():
        top = max(r.addr + r.nbytes for r in m.values())
        assert top <= rd.SPAN[fam], (fam, top)
    assert rd.SPAN[Family.STS] == 88 and rd.SPAN[Family.SCS] == 84


def test_mock_dump_reads_the_factory_block_and_diffs_a_legacy_dump(tmp_path):
    out = run_dump(tmp_path, "--ids", "1-17")
    assert "MAX_ACC(85) 0" in out and "fw 3.10" in out
    (path,) = tmp_path.glob("dump_*.csv")
    rows = read_csv(path)
    head, body = rows[0], rows[1:]
    assert len(body) == 88 and [r[0] for r in body] == [str(a) for a in range(88)]
    assert body[85][1] == "MAX_ACC" and body[86][1] == "ACC_MULTIPLIER" and body[83][2] == "ACC_2"
    c1, c16 = head.index("id1"), head.index("id16")
    assert all(body[a][c1] != "" for a in range(88))                    # STS: 0..87
    assert body[83][c16] != "" and all(body[a][c16] == "" for a in range(84, 88))   # SCS: 0..83
    legacy = tmp_path / "old" / "dump_legacy.csv"                        # what a pre-D063 run wrote
    legacy.parent.mkdir()
    old = [r[:] for r in rows[:75]]
    old[1 + 7][c1] = str(int(old[1 + 7][c1]) + 1)                        # id 1's return delay differed
    with open(legacy, "w", newline="") as f:
        csv.writer(f).writerows(old)
    out = run_dump(tmp_path / "new", "--ids", "1-17", "--diff", str(legacy))
    assert "id 1 addr  7:" in out and "1 changed bytes" in out


def test_a_refused_long_read_falls_back_to_the_legacy_span():
    bus = FeetechBus(make_pebble_mock(), {1: Family.STS})
    real = bus.read_raw

    def short_table(sid, addr, n, retries=None):
        if n > rd.LEGACY_SPAN:
            raise BusError("short read")
        return real(sid, addr, n, retries)
    bus.read_raw = short_table
    raw, note = rd.read_span(bus, 1)
    assert len(raw) == rd.LEGACY_SPAN and "0..87 refused" in note
    assert "MAX_ACC(85) -" in rd.summary(1, Family.STS, raw)
    assert rd.read_span(FeetechBus(make_pebble_mock(), {1: Family.STS}), 1)[1] == ""


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
