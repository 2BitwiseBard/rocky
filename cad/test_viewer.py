"""Load the viewer in headless Chromium; screenshot every mode; capture console.

Session 6: targets the version-agnostic pebble_viewer.html and discovers the
mode buttons dynamically instead of hard-coding indices.
"""
from playwright.sync_api import sync_playwright
import os

HERE = os.path.dirname(os.path.abspath(__file__))
url = "file://" + os.path.join(HERE, "pebble_viewer.html")

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
        slug = labels[i].lower().replace(" ", "_").replace(":", "").replace("&", "and")
        page.screenshot(path=os.path.join(HERE, "out", f"viewer_{slug}.png"))
    browser.close()

errors = [m for m in msgs if "error" in m.lower()]
print("console messages:", len(msgs), "| errors:", len(errors))
for m in msgs:
    print(" ", m)
assert not errors, "viewer console errors"
print("screenshots saved to out/viewer_*.png")
