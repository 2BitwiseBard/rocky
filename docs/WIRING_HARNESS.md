# Wiring harness plan v1 — Pebble (implements D016 + I1/I4/I5)

*The one-page truth for how electrons move. Master plan §4's diagram is
superseded by this doc + the D016 fix: hand servos NEVER see 12 V.*

```
                 ┌────────────────────────────────────────────────┐
 3S LiPo 5200    │ battery sled (I5)                              │
   XT60 ─────────┤ blind-mate dock ── bus bar ──┬── XT30 spare ×2 │
                 └──────────────┬───────────────┴────────────────-┘
                                │ 12 AWG
                    15 A blade fuse ── rocker switch (E-stop reachable)
                                │
              ┌─────────────────┼──────────────────────────┐
              │ 12 V rail       │                          │
              │                 │                          │
        ┌─────┴─────┐   ┌───────┴────────┐         ┌───────┴───────┐
        │ 5-leg star │   │ avionics tray  │         │ 6 V BEC (D016)│
        │ (below)    │   │ XT30 → 5V/5A   │         │ ≥3 A, 12→6 V  │
        │            │   │ buck → Pi 5    │         │ hand-servo    │
        └────────────┘   │ (I4 bulkhead)  │         │ power rail    │
                         └────────────────┘         └───────┬───────┘
                                                            │ to the 5 leg
                                                            │ drops (below)
```

## Per-leg drop (×5, terminates at the I1 leg-port connectors)

| conductor | gauge | connector | carries |
|---|---|---|---|
| 12 V + GND | **20 AWG silicone** | **XT30** | 3× ST3215 (peak ~2.7 A/servo stall, ~2–3 A/leg walking; 20 AWG silicone is good for ~10 A short runs) |
| TTL data | 26 AWG | JST-XH-5 pin 1 | shared bus daisy (see topology) |
| GND (signal) | 26 AWG | JST-XH-5 pin 2 | bus reference |
| 6 V (hand) | 24 AWG | JST-XH-5 pin 3 | 1× SCS0009 (≤1 A stall) — from the BEC rail, **never 12 V** |
| spare A | 26 AWG | JST-XH-5 pin 4 | SEA microswitch signal (D010) — switch closes to GND |
| spare B | 26 AWG | JST-XH-5 pin 5 | reserved (future foot sensor / leg LED) |

In-leg routing: XT30+XH mate at the deck cutout → loom up the coxa cable
slot → femur servo pigtail (servo-to-servo daisy jumpers stay the stock
Feetech 3-pin) → knee → down the tube channel → SEA/hand: JST-SH-3 through
the I2 tool socket (6 V, GND, data) + microswitch pair back up.

## Bus topology (one logical TTL bus, D003)

Star-of-daisies: the Waveshare adapter's bus header feeds a small printed
**star board** (5-position JST-XH breakout, lives beside the tray bulkhead);
each leg drop is its own daisy chain (yaw→hip→knee→hand). A wiring fault in
one leg unplugs at the port without lobotomizing the other four (master
plan §7 mitigation). Total stubs stay short (<400 mm) — fine at 1 Mbps.

## Power budget sanity

| load | nominal | peak |
|---|---|---|
| 15× ST3215 walking | ~4–6 A @ 12 V | 8–10 A (stance transitions) |
| stall worst-case (protection-limited) | — | 15 A fuse is the ceiling, by design |
| Pi 5 + camera | 1.2 A @ 5 V | 2.5 A (buck: 5 A headroom ✓) |
| 5× SCS0009 | 0.2 A @ 6 V | ~1.5 A all-stall (BEC ≥3 A ✓) |

Voltage sag: 3S under 10 A ≈ 0.3–0.5 V dip — the driver's SafetyMonitor
volt_min (9.9 V) already accounts for it; log brownouts in NOTES_INBOX.

## Build order (when Batch 2 lands)

1. Bench: dock + fuse + switch + one XT30 drop → one leg on the jig.
2. Loom lengths measured ON the robot (deck v0.3 printed), not guessed.
3. Heat-shrink color code: red 12 V, yellow 6 V, black GND, white data,
   blue sensor. Every drop labeled with its leg number at BOTH ends.
4. Continuity + polarity test EVERY drop before a servo ever plugs in
   (the D016 rule exists because one backwards moment cooks a hand).

## Connector shopping list (addendum lines — see Batch-3 sheet)

- XT30 pairs ×8 (5 drops + dock taps + spares) — genuine Amass
- XT60 pairs ×3 (sled + dock + spare) — Amass
- JST-XH 2.54 kit: 5-pin housings ×10 + crimp terminals (or pre-crimped)
- JST-SH 1.0 3-pin pigtails ×8 (I2 tool sockets + spares) — pre-crimped only
  (SH crimping by hand is misery)
- 6 V BEC ≥3 A (hobbywing-style adjustable, set 6.0 V) ×1 + spare
- 20 AWG silicone wire (red/black), 26 AWG (white/black/blue), 24 AWG yellow
- M3 thumbscrew stock: knurled-head M3×10 ×6 (or print `thumb_knob_m3` over
  SHCS — zero cost, try first)
- heat-shrink assortment, blade fuses 15 A ×5, fuse holder ×2
