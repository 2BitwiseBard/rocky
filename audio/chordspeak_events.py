#!/usr/bin/env python3
"""Chord-speak event engine — the robot narrates its own runtime.

Maps runtime events to vocabulary words with priorities, per-word
cooldowns and no-overlap scheduling, then renders a narration track for a
timeline (sim video soundtrack today; the MAX98357A + 40 mm speaker on the
Batch-3 sheet tomorrow — same engine, play instead of render).

    from chordspeak_events import Narrator
    n = Narrator()
    n.event(2.31, "stuck")        # -> confused
    n.event(4.80, "retry_escalate")   # -> determined
    track = n.render(total_s=30.0)

Design rules:
  * one mouth: words never overlap; a word starts at max(event time, end
    of current word + min_gap). If it would start later than max_latency
    after its event, it is dropped (stale narration is worse than none).
  * priorities: an URGENT word (alarm/error) cuts the current word short
    (12 ms fade) and jumps the queue.
  * per-word cooldown stops stutter (a watchdog that fires 3x in 4 s says
    "confused" once, then thinks).
"""
from __future__ import annotations
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from chordspeak2 import SR, render_word, vocabulary, write_wav   # noqa: E402

# event -> (word, priority, cooldown_s)   priority 2 = urgent (preempts)
EVENT_MAP = {
    "boot":            ("startup", 1, 10.0),
    "walk_start":      ("acknowledge", 0, 3.0),
    "stuck":           ("confused", 1, 6.0),
    "retry":           ("thinking", 0, 5.0),
    "retry_escalate":  ("determined", 1, 5.0),
    "recovered":       ("acknowledge", 0, 3.0),
    "crossing":        ("found_it", 1, 8.0),
    "goal":            ("amaze", 1, 30.0),
    "cliff":           ("alarm_help", 2, 4.0),
    "brace":           ("error", 2, 4.0),
    "resume":          ("acknowledge", 0, 3.0),
    "discovery":       ("discovery", 1, 8.0),
    "low_battery":     ("low_battery", 1, 20.0),
    "question":        ("curious_question", 0, 6.0),
    "greet":           ("greeting", 1, 15.0),
    "sleep":           ("sleepy", 1, 30.0),
}


class Narrator:
    def __init__(self, min_gap=0.25, max_latency=2.5):
        self.min_gap = min_gap
        self.max_latency = max_latency
        self.events = []                      # (t, event_name)
        self._V = vocabulary()
        self._cache = {}

    def event(self, t, name):
        if name not in EVENT_MAP:
            raise KeyError(f"unknown event '{name}' — add it to EVENT_MAP")
        self.events.append((float(t), name))

    def _wav(self, wname):
        if wname not in self._cache:
            self._cache[wname] = render_word(wname, self._V)
        return self._cache[wname]

    def schedule(self):
        """Resolve events -> [(t_start, word, cut_at or None)]. Pure logic,
        unit-testable without audio."""
        out = []
        busy_until = 0.0
        last_said = {}                        # word -> t last spoken
        for t, name in sorted(self.events):
            wname, prio, cooldown = EVENT_MAP[name]
            if t - last_said.get(wname, -1e9) < cooldown:
                continue
            dur = len(self._wav(wname)) / SR
            if prio >= 2 and out and busy_until > t:
                # urgent: cut the current word at the event time
                pt, pw, _ = out[-1]
                out[-1] = (pt, pw, max(t - pt, 0.05))
                start = max(t, pt + 0.06)
            else:
                start = max(t, busy_until + self.min_gap)
                if start - t > self.max_latency:
                    continue                  # stale — drop it
            out.append((start, wname, None))
            busy_until = start + dur
            last_said[wname] = start
        return out

    def render(self, total_s, gain=0.9):
        track = np.zeros(int(total_s * SR))
        for t0, wname, cut in self.schedule():
            buf = self._wav(wname)
            if cut is not None:
                nc = int(cut * SR)
                fade = max(int(0.012 * SR), 2)
                buf = buf[:nc].copy()
                if len(buf) > fade:
                    buf[-fade:] *= np.linspace(1, 0, fade)
            i0 = int(t0 * SR)
            if i0 >= len(track):
                continue
            seg = buf[:len(track) - i0]
            track[i0:i0 + len(seg)] += seg
        peak = np.abs(track).max()
        if peak > 1e-9:
            track = track / max(peak, 1.0) * gain
        return track


def _selftest():
    n = Narrator()
    n.event(1.0, "walk_start")
    n.event(3.0, "stuck")
    n.event(3.4, "stuck")          # cooldown -> dropped
    n.event(5.0, "retry_escalate")
    n.event(5.1, "brace")          # urgent -> cuts 'determined'
    n.event(9.0, "recovered")
    n.event(12.0, "goal")
    sched = n.schedule()
    for t, w, cut in sched:
        print(f"  {t:5.2f}s  {w:12s}" + (f" (cut {cut:.2f}s)" if cut else ""))
    words = [w for _, w, _ in sched]
    assert words.count("confused") == 1, "cooldown failed"
    assert "error" in words, "urgent event lost"
    cuts = [c for _, w, c in sched if w == "determined"]
    assert cuts and cuts[0] is not None, "urgent did not preempt"
    track = n.render(total_s=16.0)
    assert np.abs(track).max() <= 0.91
    print(f"self-test PASS ({len(sched)} scheduled, "
          f"{np.abs(track).max():.2f} peak)")


if __name__ == "__main__":
    _selftest()
