"""Load the viewer in headless Chromium; screenshot every mode; capture console.

A dev tool (not a run_all_checks module): needs `pip install playwright` and
`playwright install chromium`. Targets the version-agnostic pebble_viewer.html
and discovers the mode buttons dynamically instead of hard-coding indices.

    python3 test_viewer.py                 # screenshots to <tmp>/pebble_viewer_shots
    python3 test_viewer.py --out DIR       # ... or to DIR (keep them out of cad/out)
"""
import argparse
import os
import re
import tempfile

from playwright.sync_api import sync_playwright

HERE = os.path.dirname(os.path.abspath(__file__))
url = "file://" + os.path.join(HERE, "pebble_viewer.html")

ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("--out", default=os.path.join(tempfile.gettempdir(), "pebble_viewer_shots"),
                help="screenshot directory (default: a temp dir, never the tracked cad/out)")
out_dir = ap.parse_args().out
os.makedirs(out_dir, exist_ok=True)

with sync_playwright() as p:
    browser = p.chromium.launch()
    page = browser.new_page(viewport={"width": 1280, "height": 800})
    msgs = []
    page.on("console", lambda m: msgs.append(f"[{m.type}] {m.text}"))
    page.on("pageerror", lambda e: msgs.append(f"[PAGEERROR] {e}"))
    page.goto(url)
    page.wait_for_timeout(3000)
    btns = page.locator("#modeBtns .btn")
    n = btns.count()
    labels = [btns.nth(i).inner_text() for i in range(n)]
    print("modes:", labels)
    for i in range(n):
        btns.nth(i).click()
        page.wait_for_timeout(700)
        # one flat file name per mode: 'Hand open/closed' used to make a directory
        slug = re.sub(r"[^a-z0-9]+", "_", labels[i].lower().replace("&", "and")).strip("_")
        page.screenshot(path=os.path.join(out_dir, f"viewer_{slug}.png"))
    browser.close()

errors = [m for m in msgs if "error" in m.lower()]
print("console messages:", len(msgs), "| errors:", len(errors))
for m in msgs:
    print(" ", m)
assert not errors, "viewer console errors"
print(f"screenshots saved to {out_dir}/viewer_*.png")
