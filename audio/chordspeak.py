#!/usr/bin/env python3
"""Chord-speak v0 — Rocky talks in chords. Actual audio, today.

The voice: additive synthesis (warm organ-bell hybrid — fundamental + five
harmonics with independent decay), two slightly-detuned voices per note
(chorus), ADSR per note, gentle tanh saturation, and a small Schroeder
reverb so it sounds like a creature in a room, not a sine in a void.

The vocabulary maps robot states to short musical phrases with consistent
grammar: UP = good/asking, DOWN = bad/ending, MAJOR = certain,
MINOR/DIMINISHED = trouble, TRITONE = does-not-compute, LYDIAN = wonder.
Phase-4 hardware: MAX98357A + 40 mm speaker (on the Batch-3 sheet);
until then these WAVs play from any laptop.

    python3 chordspeak.py            # renders samples/*.wav + demo_reel.wav
"""
import os
import wave

import numpy as np

SR = 44100
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "samples")

# ------------------------------------------------------------------ synth
HARMONICS = [(1, 1.00, 1.6), (2, 0.42, 2.2), (3, 0.24, 2.8),
             (4, 0.12, 3.4), (5, 0.07, 4.2), (6, 0.045, 5.0)]


def note(freq, dur, amp=0.5, attack=0.012, release=0.12, bend=0.0,
         vibrato=(0.0, 5.0)):
    """One warm note. bend: +/- semitones over the duration.
    vibrato: (depth_semitones, rate_hz)."""
    n = int(dur * SR)
    t = np.arange(n) / SR
    sweep = freq * 2 ** ((bend * t / dur) / 12)
    vib = 2 ** ((vibrato[0] * np.sin(2 * np.pi * vibrato[1] * t)) / 12)
    f_inst = sweep * vib
    phase = 2 * np.pi * np.cumsum(f_inst) / SR
    sig = np.zeros(n)
    for mult, a, decay in HARMONICS:
        env_h = np.exp(-decay * t / max(dur, 0.2))
        for det in (-0.0012, 0.0012):                    # chorus pair
            sig += a * env_h * np.sin(phase * mult * (1 + det))
    # ADSR-ish: attack + sustain + release
    env = np.ones(n)
    na, nr = max(int(attack * SR), 2), max(int(release * SR), 2)
    env[:na] = np.linspace(0, 1, na)
    env[-nr:] *= np.linspace(1, 0, nr)
    return amp * env * sig / len(HARMONICS)


def render(events, total=None, pad=0.25):
    """events: list of (t_start, freqs, dur, amp, kwargs). Mix + polish."""
    end = max(t + d for t, _, d, _, _ in events) + pad
    total = total or end
    buf = np.zeros(int(total * SR))
    for t0, freqs, dur, amp, kw in events:
        for f in np.atleast_1d(freqs):
            s = note(f, dur, amp=amp / max(len(np.atleast_1d(freqs)) ** 0.5, 1),
                     **kw)
            i0 = int(t0 * SR)
            buf[i0:i0 + len(s)] += s[:len(buf) - i0]
    buf = np.tanh(1.3 * buf) * 0.82                      # glue + headroom
    return reverb(buf)


def reverb(x, mix=0.16):
    """Tiny Schroeder: 3 combs + 1 allpass."""
    y = np.zeros_like(x)
    for d_ms, g in ((29.7, 0.72), (37.1, 0.68), (44.1, 0.63)):
        d = int(d_ms / 1000 * SR)
        c = np.copy(x)
        for i in range(d, len(x)):
            c[i] += g * c[i - d]
        y += c
    y /= 3
    d = int(0.005 * SR)
    g = 0.5
    ap = np.copy(y)
    for i in range(d, len(y)):
        ap[i] = -g * y[i] + y[i - d] + g * ap[i - d]
    out = (1 - mix) * x + mix * ap
    return out / max(np.abs(out).max(), 1e-9) * 0.9


def hz(name):
    """'C4' -> frequency. Supports #/b."""
    names = {"C": 0, "D": 2, "E": 4, "F": 5, "G": 7, "A": 9, "B": 11}
    k = names[name[0]]
    rest = name[1:]
    if rest.startswith("#"):
        k += 1
        rest = rest[1:]
    elif rest.startswith("b"):
        k -= 1
        rest = rest[1:]
    octave = int(rest)
    midi = 12 * (octave + 1) + k
    return 440.0 * 2 ** ((midi - 69) / 12)


def N(*names):
    return [hz(x) for x in names]


# ------------------------------------------------------------------ words
def vocabulary():
    V = {}
    q = dict()

    # acknowledge — quick rising major arpeggio into a settled chord. "Heard you."
    V["acknowledge"] = [
        (0.00, N("C4"), 0.16, 0.5, q), (0.10, N("E4"), 0.16, 0.5, q),
        (0.20, N("G4"), 0.16, 0.5, q),
        (0.32, N("C4", "E4", "G4"), 0.55, 0.75, q)]

    # error — diminished stack, two heavy low pulses. "That went wrong."
    V["error"] = [
        (0.00, N("B3", "D4", "F4"), 0.30, 0.8, q),
        (0.42, N("B2", "F3"), 0.55, 0.85, dict(bend=-1.0))]

    # found_it — V7 -> I cadence, bright. "There it is."
    V["found_it"] = [
        (0.00, N("G4", "B4", "D5", "F5"), 0.34, 0.6, q),
        (0.38, N("C5", "E5", "G5"), 0.7, 0.8, q),
        (0.52, N("C6"), 0.5, 0.35, q)]

    # thinking — soft slow 4th oscillation, loopable. "Processing…"
    V["thinking"] = [
        (0.00, N("A3"), 0.42, 0.30, q), (0.45, N("D4"), 0.42, 0.30, q),
        (0.90, N("A3"), 0.42, 0.30, q), (1.35, N("D4"), 0.42, 0.30, q),
        (1.80, N("A3"), 0.5, 0.24, q)]

    # amaze — the canon word: lydian sweep up + sparkle on top. "AMAZE!"
    lyd = ["C4", "D4", "E4", "F#4", "G4", "A4", "B4", "C5", "D5", "E5"]
    V["amaze"] = [(0.05 * i, N(p), 0.22, 0.42, q) for i, p in enumerate(lyd)]
    V["amaze"] += [
        (0.58, N("C5", "E5", "G5", "B5"), 0.85, 0.72, q),
        (0.80, N("F#6"), 0.32, 0.28, dict(vibrato=(0.3, 6.5))),
        (0.98, N("D6"), 0.42, 0.30, q)]

    # discovery — whole-tone rise resolving into major-add9. "Something NEW."
    V["discovery"] = [
        (0.00, N("C4"), 0.16, 0.4, q), (0.13, N("D4"), 0.16, 0.42, q),
        (0.26, N("E4"), 0.16, 0.44, q), (0.39, N("F#4"), 0.16, 0.46, q),
        (0.52, N("G#4"), 0.2, 0.48, q),
        (0.74, N("A4", "C#5", "E5", "B5"), 0.9, 0.7, q)]

    # low_battery — descending minor thirds, slowing, flatting. "Getting sleepy-hungry."
    V["low_battery"] = [
        (0.00, N("A4"), 0.3, 0.55, q), (0.36, N("F4"), 0.34, 0.5, q),
        (0.78, N("D4"), 0.4, 0.45, dict(bend=-0.4)),
        (1.30, N("A3"), 0.7, 0.4, dict(bend=-0.8))]

    # confused — tritone wobble with vibrato. "Does not compute."
    V["confused"] = [
        (0.00, N("C4"), 0.3, 0.5, dict(vibrato=(0.5, 7))),
        (0.32, N("F#4"), 0.3, 0.5, dict(vibrato=(0.5, 7))),
        (0.64, N("C4"), 0.3, 0.5, dict(vibrato=(0.5, 7))),
        (0.96, N("F#4", "C5"), 0.5, 0.5, dict(vibrato=(0.7, 6)))]

    # startup — dawn chord: low root swell, open fifth, then full major. "I am."
    V["startup"] = [
        (0.00, N("C3"), 1.4, 0.55, dict(attack=0.5)),
        (0.50, N("G3"), 1.0, 0.5, dict(attack=0.3)),
        (0.95, N("C4", "E4", "G4"), 1.1, 0.7, dict(attack=0.15)),
        (1.55, N("C5"), 0.7, 0.35, q)]

    # sleepy — major-7 lullaby drifting down, ritardando. "Powering down."
    V["sleepy"] = [
        (0.00, N("B4"), 0.35, 0.42, q), (0.40, N("G4"), 0.4, 0.4, q),
        (0.88, N("E4"), 0.5, 0.36, q),
        (1.48, N("C4", "E4", "G4", "B4"), 1.2, 0.4,
         dict(attack=0.2, bend=-0.3))]

    # alarm_help — urgent minor-2nd cluster pulses. "HELP. HELP."
    puls = N("E5", "F5")
    V["alarm_help"] = [(0.28 * i, puls, 0.16, 0.8, q) for i in range(4)]
    V["alarm_help"] += [(1.15, N("E5", "F5", "B4"), 0.5, 0.85, q)]

    # yes / no — one-syllable words
    V["yes"] = [(0.00, N("C4"), 0.14, 0.5, q), (0.12, N("E4"), 0.3, 0.6, q)]
    V["no"] = [(0.00, N("E4"), 0.16, 0.6, q),
               (0.16, N("Eb4"), 0.4, 0.55, dict(bend=-0.3))]

    # curious_question — rises and does NOT resolve. "…what's that?"
    V["curious_question"] = [
        (0.00, N("C4"), 0.18, 0.45, q), (0.16, N("E4"), 0.18, 0.47, q),
        (0.32, N("G4"), 0.18, 0.5, q),
        (0.50, N("Bb4"), 0.55, 0.55, dict(bend=+0.5))]

    # greeting — pentatonic flourish. "Hi, Tyler."
    V["greeting"] = [
        (0.00, N("G4"), 0.14, 0.45, q), (0.11, N("A4"), 0.14, 0.45, q),
        (0.22, N("C5"), 0.14, 0.5, q), (0.33, N("D5"), 0.16, 0.5, q),
        (0.47, N("G5", "D5", "C5"), 0.6, 0.6, q)]
    return V


def write_wav(path, buf):
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes((np.clip(buf, -1, 1) * 32767).astype(np.int16).tobytes())


def main():
    os.makedirs(OUT, exist_ok=True)
    V = vocabulary()
    reel = []
    for name, events in V.items():
        buf = render(events)
        write_wav(os.path.join(OUT, f"{name}.wav"), buf)
        print(f"  {name:18s} {len(buf) / SR:4.2f} s  rms {np.sqrt((buf**2).mean()):.3f}")
        reel.append(buf)
        reel.append(np.zeros(int(0.45 * SR)))
    write_wav(os.path.join(OUT, "demo_reel.wav"), np.concatenate(reel))
    print(f"wrote {len(V)} words + demo_reel.wav -> {OUT}/")


if __name__ == "__main__":
    main()
