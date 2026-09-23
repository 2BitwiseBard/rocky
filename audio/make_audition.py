#!/usr/bin/env python3
"""Build audition_v2.html — self-contained A/B page: chord-speak v0 vs v0.2.

Every wav is embedded as a base64 data URI (D007: chat-preview sandbox
blocks external fetches; self-contained or it doesn't ship). Run after
chordspeak.py and chordspeak2.py have rendered their sample dirs.

    python3 make_audition.py   ->  audition_v2.html
"""
import base64
import os

HERE = os.path.dirname(os.path.abspath(__file__))
V0 = os.path.join(HERE, "samples")
V2 = os.path.join(HERE, "samples_v2")

NOTES = {
    "acknowledge": "two clipped grunts, second up — “mm-hm”",
    "yes": "clean 1:3/2:2, rising tail",
    "no": "root drops a septimal step and roughens",
    "confused": "11/8 + 7/4 stacks, detuned, wobbling — beats you can hear",
    "thinking": "soft 1:4/3 mutter, loopable",
    "determined": "NEW — three hard accents, growl crescendo (retry escalation)",
    "found_it": "second syllable jumps a fifth and brightens",
    "amaze": "swelling stacks, 45/32 shimmer on top — the canon word",
    "error": "septimal cluster, two strikes, second lands lower + rougher",
    "alarm_help": "fast 15/14 cluster pulses — urgent, preempts other words",
    "low_battery": "sinking 6/7 root steps, slowing, breath rising",
    "startup": "long low swell into one confident strike — “I am”",
    "sleepy": "long breathy syllables drifting down",
    "curious_question": "lengthened rising final with 11/8 color — unresolved",
    "greeting": "warm clean ratios, up-inflected",
    "discovery": "quickening rising steps, septimal-bright landing",
}


def b64(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode()


def main():
    words = [w[:-4] for w in sorted(os.listdir(V2))
             if w.endswith(".wav") and not w.startswith("demo")]
    rows = []
    for w in words:
        v2 = b64(os.path.join(V2, f"{w}.wav"))
        v0p = os.path.join(V0, f"{w}.wav")
        v0btn = (f'<button class="b v0" data-src="data:audio/wav;base64,'
                 f'{b64(v0p)}">v0</button>') if os.path.exists(v0p) \
            else '<span class="none">—</span>'
        rows.append(f"""
  <div class="row">
    <div class="w">{w.replace('_', ' ')}</div>
    <div class="btns">{v0btn}
      <button class="b v2" data-src="data:audio/wav;base64,{v2}">v0.2</button>
    </div>
    <div class="note">{NOTES.get(w, '')}</div>
  </div>""")
    reels = f"""
  <div class="row reel">
    <div class="w">full reel</div>
    <div class="btns">
      <button class="b v0" data-src="data:audio/wav;base64,{b64(os.path.join(V0, 'demo_reel.wav'))}">v0 reel</button>
      <button class="b v2" data-src="data:audio/wav;base64,{b64(os.path.join(V2, 'demo_reel_v2.wav'))}">v0.2 reel</button>
    </div>
    <div class="note">all words back to back</div>
  </div>"""

    html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>ROCKY chord-speak — v0 vs v0.2 audition</title>
<style>
  body {{ background:#17121f; color:#e8e2f2; font-family: system-ui, sans-serif;
         max-width: 860px; margin: 2rem auto; padding: 0 1rem; }}
  h1 {{ font-size: 1.4rem; color:#c9b6e8; }}
  .sub {{ color:#9a8fb0; margin-bottom:1.4rem; line-height:1.45 }}
  .row {{ display:flex; align-items:center; gap:1rem; padding:.5rem .8rem;
         border-bottom:1px solid #2a2338; }}
  .row:hover {{ background:#1f1930 }}
  .w {{ width:11.5rem; font-weight:600; color:#d9c9f5 }}
  .btns {{ display:flex; gap:.5rem; width:9.5rem }}
  .b {{ background:#3a2a5a; color:#e8e2f2; border:1px solid #5a4394;
       border-radius:6px; padding:.35rem .8rem; cursor:pointer; font-size:.9rem }}
  .b.v2 {{ background:#5a3a1a; border-color:#a06a2a }}
  .b:hover {{ filter:brightness(1.3) }}
  .b.playing {{ outline:2px solid #e0a030 }}
  .none {{ color:#554a66; padding:.35rem .9rem }}
  .note {{ color:#9a8fb0; font-size:.85rem; flex:1 }}
  .reel {{ border-top:2px solid #5a4394; margin-top:.6rem }}
  #status {{ margin-top:1rem; color:#e0a030; min-height:1.2em }}
</style></head><body>
<h1>ROCKY chord-speak — v0 “organ” vs v0.2 “Eridian”</h1>
<div class="sub">v0.2 design: chord-<i>syllables</i> with speech envelopes
(no scale runs, no cadences), just-intonation stacks with septimal/undecimal
intervals on low roots (66–200&nbsp;Hz), one fixed formant “throat”,
band-limited breath, growl&nbsp;=&nbsp;urgency, beating detune&nbsp;=&nbsp;
uncertainty. Purple&nbsp;=&nbsp;v0, amber&nbsp;=&nbsp;v0.2.</div>
{''.join(rows)}
{reels}
<div id="status"></div>
<script>
  let cur = null;
  const status = document.getElementById('status');
  document.querySelectorAll('button.b').forEach(btn => {{
    btn.addEventListener('click', () => {{
      if (cur) {{ cur.pause(); document.querySelectorAll('.playing')
                  .forEach(b => b.classList.remove('playing')); }}
      cur = new Audio(btn.dataset.src);
      btn.classList.add('playing');
      status.textContent = 'playing: ' + btn.closest('.row')
        .querySelector('.w').textContent + ' (' + btn.textContent + ')';
      cur.addEventListener('ended', () => {{
        btn.classList.remove('playing'); status.textContent = ''; }});
      cur.play();
    }});
  }});
</script>
</body></html>"""
    out = os.path.join(HERE, "audition_v2.html")
    with open(out, "w") as f:
        f.write(html)
    print(f"wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB, "
          f"{len(words)} words)")


if __name__ == "__main__":
    main()
