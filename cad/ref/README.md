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
