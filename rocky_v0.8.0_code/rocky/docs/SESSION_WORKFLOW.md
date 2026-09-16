# How we work: the organization system

Three layers, each with one job. Nothing lives in two places.

## 1. This repo — source of truth for artifacts
CAD scripts, gait code, BOM, logs, media. Recommended: `git init`, push to a
private GitHub repo tonight. Every session's outputs land here (Claude hands
you a refreshed zip, or — better — commits patches you apply).

## 2. Claude "Robotics" Project — cross-session memory
What Claude reads at the start of any future session, on any device:
- **Project ROCKY — Master Plan** (phases, budget, physics)
- **Pebble CAD — status log** (versions, frame conventions, defect history, TODO)

When plans change, Claude rewrites those docs in-session. If a chat ever feels
like it "forgot" something, point it at the status log — that's the memory.

## 3. The notebook: `BUILD_LOG.md` + `NOTES_INBOX.md`
- **BUILD_LOG.md** — dated entries, newest first. Claude writes one at the end
  of each working session (template at the bottom of the file). Failures are
  first-class content.
- **NOTES_INBOX.md** — your scratch inbox. Any time, from any device: paste
  caliper measurements, print results, part arrivals, ideas, photos'
  descriptions. Next session, Claude files the contents into params.yaml /
  BOM / BUILD_LOG / decisions and empties the inbox. Zero organizing required
  from you.

## Session ritual
**Start:** tell Claude "pick up ROCKY" → it reads the Project docs + you paste
or sync the current repo state (or just the NOTES_INBOX) → it states where we
are and proposes the day's goal.
**End:** Claude updates BUILD_LOG + status log + zips changed artifacts.

## Physical-world habits (the cheap ones that pay off)
- A cheap paper pad next to the printer for mid-print scribbles → photograph or
  retype into NOTES_INBOX later. Don't organize it; the inbox is for mess.
- Label every printed part revision with a Sharpie (e.g. "coxa v0.3") the
  moment it comes off the bed. Old revisions into a "graveyard" box, not the
  trash — comparing failures is diagnosis.
- One parts bin per Batch (0, 1, 2…) as orders arrive; the BOM's Status column
  ("to buy / ordered / here / installed") is the inventory truth.
