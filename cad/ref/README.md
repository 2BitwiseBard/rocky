# Reference geometry (not printed)

`STS3215_03a.step` — Feetech STS3215 servo with its disc horn and rear idler
wheel, from TheRobotStudio/SO-ARM100 (`STEP/SO100/STS3215_03a.step`, last
changed upstream in commit `d04fa975fb79c9870df5824cce7c7390d52cac85`; the
copy here is byte-identical to that blob, git `133997c1`). Licensed under the
Apache License 2.0: the full text is `LICENSE-Apache-2.0.txt` next to it (the
upstream repository ships no NOTICE file). Unmodified.

This is the solid `servo_st3215.py` is derived from; its self-check (a
`run_all_checks` module, so CI runs it) compares the model against it and
fails if any STEP feature pokes out of the model. Every number taken from it
is marked `from STEP` in `params.yaml` and stays VERIFY until a real servo has
been measured with calipers.

`rpi5_envelope.json` — the Raspberry Pi 5's part envelope (B127), derived
from the official STEP: `RaspberryPi5-step.zip` from
datasheets.raspberrypi.com/rpi5/ (sha256 `6841637b…f06f`), its
`rpi-5b_no_graphics.step` dated 2026-05-27 (sha256 `78164070…fb44`, 2689
solids). Licensed MIT, Copyright (c) 2026 Raspberry Pi Ltd: the zip's own
`LICENSE.txt` is `LICENSE-MIT-RaspberryPi.txt` next to it, unmodified (its
disclaimer: guidance only, measure the real board). The STEP itself is not
here: 77.6 MB (its four connector shells alone are 31 MB as STEP), and the
checks need only boxes. The JSON is every part above the board as a box (the
USB and Ethernet stacks in 1 mm z slabs, so their real tops are kept), one
layer box for everything ≤ 3.88 over the board, and the underside, in the
STEP's frame. `part_avionics.py --pi5-from-step <the .step>` regenerates it
from the file above (about 1.5 min); `part_avionics`' carapace, USB-plug and
leg-clearance checks pose it on the tray.
