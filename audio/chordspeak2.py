#!/usr/bin/env python3
"""Chord-speak v0.2 — the Eridian voice. Speech made of chords, not music.

v0 review (Tyler, session 5): "still sounds musical... too nice." Right —
v0 was built from 12-TET scale runs, major/minor cadences, organ-bell
chorus and cathedral reverb: a pleasant instrument. Rocky (Project Hail
Mary) doesn't play an instrument; he TALKS, and each word happens to be a
chord. v0.2 rebuilds the voice around that:

  * SYLLABLES, not phrases: every unit is a simultaneous chord stack with
    a speech envelope (fast attack, spoken decay, slight downward pitch
    tail). No arpeggios, no scale runs, no V–I cadences anywhere.
  * JUST-INTONATION ratio stacks on LOW roots (66–200 Hz) with septimal
    and undecimal intervals (7/6, 7/4, 11/8, 15/14...) — structured, so
    grammar survives, but off the piano grid, so it stops reading as
    Western music. Certainty = ratio simplicity: {1, 3/2, 2} is a
    confident word; {1, 11/8, 7/4} detuned & beating is a confused one.
  * ONE THROAT: every word passes the same fixed formant resonances
    (420 / 1150 / 2500 Hz), with a breath-noise layer shaped by the same
    filters and a half-root sub for chest — a creature's voice box, not
    an oscillator bank.
  * ROUGHNESS is an emotion axis: ~31 Hz amplitude growl scaled up for
    alarm/error/determination, absent in calm words.
  * PROSODY carries the grammar v0 established: rising final syllable =
    good/asking, falling = bad/ending, wobble = does-not-compute,
    lengthened final = question. Dry, close space (small reverb 8%).

Renders samples_v2/*.wav + demo_reel_v2.wav:  python3 chordspeak2.py
Event-driven narration: chordspeak_events.py. v0 kept in chordspeak.py
for A/B (audition_v2.html plays both).
"""
import os
import wave

import numpy as np
from scipy.signal import lfilter

SR = 44100
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "samples_v2")

# the throat: fixed formant resonances shared by every word
FORMANTS = [(420, 4.0, 0.34), (1150, 5.0, 0.30), (2500, 6.0, 0.14)]
DRY_MIX = 0.50


def _bandpass_coefs(f0, Q):
    w0 = 2 * np.pi * f0 / SR
    alpha = np.sin(w0) / (2 * Q)
    b = np.array([alpha, 0.0, -alpha])
    a = np.array([1 + alpha, -2 * np.cos(w0), 1 - alpha])
    return b / a[0], a / a[0]


_FORMANT_BA = [(_bandpass_coefs(f, q), g) for f, q, g in FORMANTS]


def throat(x):
    """The fixed vocal tract: dry + formant resonances."""
    y = DRY_MIX * x
    for (b, a), g in _FORMANT_BA:
        y = y + g * lfilter(b, a, x)
    return y


def syllable(root_hz, ratios, dur, amp=0.6, rough=0.0, breath=0.10,
             bend_cents=-14.0, attack=0.014, tail=0.30, detune_cents=0.0,
             wobble=0.0, tilt=1.0, seed=0):
    """One chord-syllable.

    ratios: JI multipliers of root sounding SIMULTANEOUSLY.
    bend_cents: pitch glide over the syllable (negative = spoken fall).
    rough: 0..~0.5 growl AM depth.  breath: aspiration noise level.
    detune_cents: random per-note detune (beating = uncertainty).
    wobble: slow 5 Hz pitch LFO depth in cents (confusion).
    tilt: partial brightness multiplier (>1 brighter)."""
    rng = np.random.default_rng(seed * 7919 + int(root_hz))
    n = int(dur * SR)
    t = np.arange(n) / SR
    # envelope: fast attack, spoken decay to `tail` of peak
    na = max(int(attack * SR), 8)
    env = np.ones(n)
    env[:na] = np.linspace(0, 1, na) ** 1.5
    decay = np.exp(np.log(max(tail, 0.05)) * t / max(dur, 1e-3))
    env *= decay
    nr = max(int(0.03 * SR), 8)
    env[-nr:] *= np.linspace(1, 0, nr)

    bend = 2 ** ((bend_cents * t / max(dur, 1e-3)) / 1200)
    wob = 2 ** ((wobble * np.sin(2 * np.pi * 5.1 * t + rng.uniform(0, 6))) / 1200) \
        if wobble else 1.0

    sig = np.zeros(n)
    part_amp = np.array([1.0, 0.46, 0.24, 0.11]) * \
        np.array([1.0, tilt, tilt ** 2, tilt ** 3])
    for j, ratio in enumerate(ratios):
        det = 2 ** (rng.normal(0, detune_cents) / 1200) if detune_cents else 1.0
        f = root_hz * ratio * det
        for p, pa in enumerate(part_amp, start=1):
            # slight inharmonic stretch: metallic-vocal, not organ
            fp = f * p * (1 + 0.0017 * p)
            if fp > SR * 0.42:
                continue
            phase = 2 * np.pi * np.cumsum(fp * bend * wob) / SR \
                + rng.uniform(0, 2 * np.pi)
            sig += pa * np.sin(phase)
    sig /= max(len(ratios), 1)
    # chest: half-root sub
    phase = 2 * np.pi * np.cumsum(0.5 * root_hz * np.ones(n) * bend) / SR
    sig += 0.22 * np.sin(phase)
    # growl AM
    if rough:
        sig *= 1 + rough * 0.55 * np.sin(2 * np.pi * 31.0 * t
                                         + rng.uniform(0, 6))
    # aspiration: band-limited breath, not full-band hiss (a raw white
    # layer put the spectral centroid at 7 kHz — a whisper, not a chest)
    if breath:
        noise = rng.normal(0, 1, n)
        a1 = 1 - np.exp(-2 * np.pi * 1400 / SR)      # low-pass ~1.4 kHz
        noise = lfilter([a1], [1, -(1 - a1)], noise)
        a2 = 1 - np.exp(-2 * np.pi * 180 / SR)       # high-pass ~180 Hz
        noise = noise - lfilter([a2], [1, -(1 - a2)], noise)
        sig += breath * 2.2 * noise
    return amp * env * sig


def word(syllables, root_hz=110.0, pad=0.16):
    """syllables: list of dicts: dur, gap (before), ratios, root_mul, and
    any syllable() kwargs. Renders through the shared throat + small room."""
    parts, t_cursor = [], 0.0
    for i, s in enumerate(syllables):
        t_cursor += s.get("gap", 0.05 if i else 0.0)
        buf = syllable(root_hz * s.get("root_mul", 1.0), s["ratios"],
                       s["dur"],
                       **{k: v for k, v in s.items()
                          if k not in ("gap", "ratios", "root_mul", "dur")},
                       seed=i)
        parts.append((t_cursor, buf))
        t_cursor += s["dur"]
    n = int((t_cursor + pad) * SR)
    mix = np.zeros(n)
    for t0, buf in parts:
        i0 = int(t0 * SR)
        mix[i0:i0 + len(buf)] += buf[:n - i0]
    mix = throat(mix)
    mix = np.tanh(1.25 * mix) * 0.85
    return small_room(mix)


def small_room(x, mix=0.08):
    y = np.zeros_like(x)
    for d_ms, g in ((17.3, 0.58), (23.9, 0.52)):
        d = int(d_ms / 1000 * SR)
        c = np.copy(x)
        for i in range(d, len(x)):
            c[i] += g * c[i - d]
        y += c
    y /= 2
    out = (1 - mix) * x + mix * y
    return out / max(np.abs(out).max(), 1e-9) * 0.88


# ------------------------------------------------------------- vocabulary
# grammar: ratio simplicity = certainty; final-syllable contour = valence;
# roughness + rate = urgency; lengthened rising final = question.
def S(dur, ratios, **kw):
    d = dict(dur=dur, ratios=ratios)
    d.update(kw)
    return d


def vocabulary():
    V = {}

    # "Mm-hm." two clipped grunts, second slightly up. Heard you.
    V["acknowledge"] = (128, [
        S(0.11, [1, 3 / 2], amp=0.55, bend_cents=-8, breath=0.08),
        S(0.16, [1, 3 / 2, 2], gap=0.06, amp=0.62, bend_cents=+10)])

    # one syllable, clean fifth+octave, rising tail. Yes.
    V["yes"] = (140, [S(0.20, [1, 3 / 2, 2], amp=0.62, bend_cents=+16)])

    # clean start, then the root DROPS a septimal step and roughens. No.
    V["no"] = (140, [
        S(0.12, [1, 3 / 2], amp=0.6, bend_cents=-4),
        S(0.30, [1, 7 / 6, 3 / 2], gap=0.04, root_mul=6 / 7, amp=0.62,
          rough=0.18, bend_cents=-22)])

    # wandering roots, 11/8+7/4 stacks, detuned & wobbling. Does-not-compute.
    V["confused"] = (118, [
        S(0.16, [1, 11 / 8, 7 / 4], detune_cents=7, bend_cents=-6),
        S(0.16, [1, 11 / 8, 7 / 4], gap=0.07, root_mul=11 / 10,
          detune_cents=7, bend_cents=+4),
        S(0.30, [1, 11 / 8, 7 / 4, 2], gap=0.08, root_mul=10 / 11,
          detune_cents=9, wobble=28, bend_cents=-10, breath=0.14)])

    # soft slow two-beat mutter, loopable. Processing...
    V["thinking"] = (96, [
        S(0.30, [1, 4 / 3], amp=0.34, breath=0.12, bend_cents=-6, tail=0.5),
        S(0.30, [1, 4 / 3], gap=0.14, root_mul=9 / 8, amp=0.30,
          breath=0.12, bend_cents=-6, tail=0.5),
        S(0.34, [1, 4 / 3], gap=0.14, amp=0.32, breath=0.12, bend_cents=-8,
          tail=0.5)])

    # three hard accents, crescendo, growl rising. Going to DO it.
    V["determined"] = (104, [
        S(0.13, [1, 3 / 2, 7 / 4], amp=0.5, rough=0.18, attack=0.006,
          bend_cents=-4),
        S(0.13, [1, 3 / 2, 7 / 4], gap=0.06, amp=0.6, rough=0.24,
          attack=0.006, bend_cents=-4),
        S(0.26, [1, 3 / 2, 7 / 4, 2], gap=0.06, root_mul=9 / 8, amp=0.72,
          rough=0.30, attack=0.006, bend_cents=+6)])

    # two syllables, second jumps a fifth and brightens. There it is.
    V["found_it"] = (124, [
        S(0.14, [1, 7 / 4], amp=0.55, bend_cents=-6),
        S(0.34, [1, 5 / 4, 3 / 2, 2], gap=0.05, root_mul=3 / 2, amp=0.7,
          tilt=1.15, bend_cents=+8)])

    # swelling stacks, the 45/32 shimmer on top (the canon color). AMAZE.
    V["amaze"] = (98, [
        S(0.22, [1, 3 / 2], amp=0.5, attack=0.03, bend_cents=+4),
        S(0.26, [1, 5 / 4, 3 / 2, 2], gap=0.05, root_mul=5 / 4, amp=0.62,
          attack=0.03, tilt=1.15, bend_cents=+6),
        S(0.62, [1, 5 / 4, 45 / 32, 3 / 2, 15 / 8, 2], gap=0.06,
          root_mul=3 / 2, amp=0.78, attack=0.04, tilt=1.28, bend_cents=+10,
          tail=0.45)])

    # septimal cluster, two heavy strikes, second lands lower. That broke.
    V["error"] = (92, [
        S(0.20, [1, 7 / 6, 7 / 5], amp=0.72, rough=0.26, attack=0.005,
          bend_cents=-10),
        S(0.36, [1, 7 / 6, 3 / 2], gap=0.09, root_mul=6 / 7, amp=0.75,
          rough=0.32, attack=0.005, bend_cents=-26, breath=0.13)])

    # four fast cluster pulses + one longer, higher, rougher. HELP HELP.
    V["alarm_help"] = (196, [
        S(0.10, [1, 15 / 14, 7 / 6], amp=0.8, rough=0.4, attack=0.004,
          bend_cents=0, breath=0.1)] + [
        S(0.10, [1, 15 / 14, 7 / 6], gap=0.075, amp=0.8, rough=0.4,
          attack=0.004, bend_cents=0, breath=0.1) for _ in range(3)] + [
        S(0.30, [1, 15 / 14, 7 / 6, 3 / 2], gap=0.09, root_mul=9 / 8,
          amp=0.85, rough=0.45, attack=0.004, bend_cents=+6, breath=0.12)])

    # sinking septimal steps, slowing, breath rising. Sleepy-hungry.
    V["low_battery"] = (118, [
        S(0.22, [1, 3 / 2], amp=0.5, bend_cents=-8),
        S(0.26, [1, 7 / 5], gap=0.09, root_mul=6 / 7, amp=0.46,
          bend_cents=-12, breath=0.14),
        S(0.46, [1, 7 / 6], gap=0.13, root_mul=(6 / 7) ** 2, amp=0.42,
          bend_cents=-30, breath=0.2, tail=0.5)])

    # long low swell opening into a clean confident strike. I am.
    V["startup"] = (66, [
        S(1.10, [1, 2, 3], amp=0.55, attack=0.55, bend_cents=+6,
          breath=0.16, tail=0.7),
        S(0.42, [1, 3 / 2, 2], gap=0.06, root_mul=2, amp=0.68,
          attack=0.02, bend_cents=+4)])

    # two long soft syllables drifting down, breathy. Powering down.
    V["sleepy"] = (104, [
        S(0.5, [1, 3 / 2, 15 / 8], amp=0.42, attack=0.08, bend_cents=-10,
          breath=0.18, tail=0.5),
        S(0.8, [1, 3 / 2], gap=0.16, root_mul=6 / 7, amp=0.36, attack=0.1,
          bend_cents=-24, breath=0.24, tail=0.6)])

    # rising final with undecimal color, lengthened, unresolved. What is?
    V["curious_question"] = (132, [
        S(0.13, [1, 3 / 2], amp=0.5, bend_cents=+4),
        S(0.42, [1, 11 / 8, 2], gap=0.05, root_mul=9 / 8, amp=0.6,
          bend_cents=+34, tail=0.55)])

    # two warm syllables, up-inflected, clean ratios. Hello Tyler.
    V["greeting"] = (124, [
        S(0.15, [1, 3 / 2], amp=0.55, bend_cents=+4),
        S(0.15, [1, 5 / 4, 3 / 2], gap=0.05, root_mul=5 / 4, amp=0.6),
        S(0.30, [1, 3 / 2, 2], gap=0.05, root_mul=4 / 3, amp=0.66,
          bend_cents=+12)])

    # quickening rising steps into a septimal-bright landing. Something NEW.
    V["discovery"] = (110, [
        S(0.14, [1, 3 / 2], amp=0.5),
        S(0.14, [1, 3 / 2], gap=0.05, root_mul=9 / 8, amp=0.55),
        S(0.14, [1, 3 / 2, 7 / 4], gap=0.045, root_mul=7 / 6, amp=0.6),
        S(0.4, [1, 3 / 2, 7 / 4, 9 / 4], gap=0.05, root_mul=4 / 3, amp=0.7,
          tilt=1.12, bend_cents=+8)])

    return V


def render_word(name, V=None):
    V = V or vocabulary()
    root, syls = V[name]
    return word(syls, root_hz=root)


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
    for name in V:
        buf = render_word(name, V)
        write_wav(os.path.join(OUT, f"{name}.wav"), buf)
        print(f"  {name:18s} {len(buf) / SR:4.2f} s  "
              f"rms {np.sqrt((buf ** 2).mean()):.3f}")
        reel.append(buf)
        reel.append(np.zeros(int(0.5 * SR)))
    write_wav(os.path.join(OUT, "demo_reel_v2.wav"), np.concatenate(reel))
    print(f"wrote {len(V)} words + demo_reel_v2.wav -> {OUT}/")


if __name__ == "__main__":
    main()
