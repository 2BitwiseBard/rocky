"""Subtotals of bom/BOM.csv by phase and status, and a check that every
line total is qty x unit. Stdlib only:  python3 bom/totals.py

core     = 'to buy' + 'wait until one leg walks'
optional = 'optional'
have     = workshop basics, bought only if missing
'choice' and 'alternative' rows are listed, never summed.
"""
import csv
import pathlib
import sys
from collections import defaultdict

CSV = pathlib.Path(__file__).with_name("BOM.csv")
CORE = ("to buy", "wait until one leg walks")
UNSUMMED = ("choice", "alternative")


def main():
    rows = list(csv.DictReader(CSV.open(encoding="utf-8")))
    bad = [r["id"] for r in rows
           if abs(float(r["qty"]) * float(r["unit_usd_est"]) - float(r["line_usd_est"])) > 0.005]
    sums = defaultdict(float)
    for r in rows:
        sums[(r["phase"], r["status"])] += float(r["line_usd_est"])
    phases = sorted({r["phase"] for r in rows})
    cols = ("to buy", "wait until one leg walks", "optional", "have")
    print(f"{'phase':<18}" + "".join(f"{c:>10}" for c in ("to buy", "wait", "optional", "have")))
    for p in phases:
        print(f"{p:<18}" + "".join(f"{sums[(p, c)]:>10.2f}" for c in cols))
    tot = lambda sts, ph=None: sum(v for (p, s), v in sums.items()
                                   if s in sts and (ph is None or p.startswith(ph)))
    print(f"\nbench kit (A, to buy)          {tot(('to buy',), 'A'):9.2f}")
    print(f"core robot (A-C, incl. wait)   {tot(CORE):9.2f}")
    print(f"optional                       {tot(('optional',)):9.2f}")
    print(f"workshop basics if missing     {tot(('have',)):9.2f}")
    for r in rows:
        if r["status"] in UNSUMMED:
            print(f"  not summed: {r['id']} {r['status']:<11} "
                  f"{float(r['line_usd_est']):8.2f}  {r['item'][:60]}")
    if bad:
        print("line_usd_est != qty x unit_usd_est:", ", ".join(bad))
        sys.exit(1)


if __name__ == "__main__":
    main()
