# Shopping list — 2026-09-22 (supersedes `docs/pebble_order_sheet_v2.html` for what to buy)

Prices are estimates in USD from vendor listings seen on 2026-09-22 or
from the 2026-09-01 sheet, marked (est). Nothing here has been ordered.
Two parts: **A** gets one leg moving on the bench; **B** completes the
robot. Buy A now; B can wait for the leg to walk on the bench.

## A. Bench kit — one leg (≈ $300–370)

| item | qty | est. | where | why |
|---|---|---|---|---|
| Waveshare / Feetech **ST3215 12 V 30 kg·cm** (the 12 V variant: listings mix it with the 7.4 V 19.5 kg·cm) | 4 | $24–30 ea | Seeed (US stock), Waveshare, AliExpress ($24.57) | leg 0 (yaw, hip, knee) + 1 spare. Comes with the disc horn, the rear idler wheel and its screws — keep the bag, it answers the M2/M3 question. |
| Waveshare **Servo Driver with ESP32** | 1 | $15.99 | Waveshare, Amazon | bus master with a web UI: the day the servos arrive you can set IDs and move them with no code. DC 6–12 V jack in; stays as the hardware watchdog later. |
| Waveshare **Bus Servo Adapter (A)** | 1 | $4.99 | Waveshare | USB-C ↔ bus for the laptop driver (`bench/` scripts). Either board works for the bench; both are cheap. |
| Bench supply: 12 V 10 A with current limit (30 V/10 A lab supply) or a Mean Well LRS-150-12 + inline XT60 wattmeter | 1 | $60–85 / $20 + $12 | Amazon | the runbook's 1.5 A first-contact limit needs an adjustable limit; a fixed supply + wattmeter is the budget path |
| Feetech 3-pin servo cables, 100–300 mm | 10 | $8 | Waveshare / AliExpress | daisy chain + bench leads |
| 5.5 × 2.1 DC barrel plug pigtails | 2 | $5 | Amazon | into the driver boards |
| **M2 × 6 self-tapping** (PA2.0) — wait for the servo kit's screws before buying 100 | 40 | $6 | AliExpress / Amazon | 4 per cup, 3 cups per leg |
| **M3 × 6 pan** | 20 | $5 | any | yaw hub → horn |
| **M3 × 8 pan** | 30 | $5 | any | coupler clamps, plate B |
| **M3 × 10 pan** | 10 | $4 | any | tube pinch bolt, thumbscrews |
| M2 × 6 pan + M2 washers | 10 + 10 | $4 | any | only if the horn is M2-tapped |
| M3 heat-set inserts Ø4.6 × 5.7 (100) + iron tip | 1 | $17–30 | ruthex / Amazon | deck thumbscrew pockets (I1) and anywhere a screw is opened often |
| Blue threadlocker | 1 | $6 | any | horn screws |
| Carbon tube Ø10 × 8, 500 mm (batch-01 item) | 1 | $12 | Amazon | tibia |
| KW10 microswitch, spring Ø12 × 25, TPU (batch-01 items) | — | ~$20 | — | SEA foot |
| Digital calipers | have | — | — | the fit ladder and the servo |

Not needed for the bench: the 683ZZ bearings (D047 removed the yaw bearing),
M2 nuts (the blank v0.3 has pilots, not nut slots).

## B. Full robot (adds ≈ $900–1,050)

| item | qty | est. | note |
|---|---|---|---|
| ST3215 12 V (to 16 total) | 12 | $290–360 | ask Seeed for the 10+ price ($22.99) |
| SCS0009 (hand / claw) | 6 | $54 | 4.8–6 V only — never on the 12 V rail (D016) |
| Raspberry Pi 5 8 GB + active cooler + 64 GB A2 card | 1 | $125 + $5 + $12 | compute; the ESP32 board stays as bus master/watchdog |
| 3S 5200 mAh 30C+ LiPo, XT60 | 1 | $35–40 | the sled is sized for 138 × 46 × 25 — VERIFY the pack |
| Balance charger (ISDT / B6 class) + LiPo bag + **balance-lead voltage alarm** | 1 each | $45 + $8 + $5 | the alarm was missing from every sheet |
| 5 V / 5 A buck (Pololu D24V50F5 class) + 470 µF / 1000 µF caps | 1 | $18 | Pi rail into the GPIO header, not USB-C |
| 6 V UBEC 3 A (hand servos) | 2 | $18 | D016 |
| Blade fuse holder + 15 A fuses; **XT60 loop-key** arming plug or a DC-rated 20 A rocker | 1 + 5, 1 | $8, $5 | |
| XT60 pairs ×5, XT30 pairs ×10 | — | $10 | |
| Per-leg 12 V splitter (XT30 → 3 leads) | 5 | $10 | **do not** daisy-chain leg power through the servo connectors (2 A pins) |
| Silicone wire 14 AWG (1 m), 20 AWG (3 m), 24–26 AWG assortment | — | $20 | |
| JST-XH 2.54 kit + crimper (Engineer PA-09), JST-SH 3-pin pre-crimped ×10 | — | $13 + $40 + $13 | |
| Heat-shrink, perfboard, right-angle XH header | — | $15 | |
| BNO085 IMU (Adafruit 4754) | 1 | $25 | run it in UART-RVC or SPI, not I²C on a Pi |
| Lidar: LD19 / D500 ($90) or a used LDS02RR pull ($17–30) | 1 | $17–90 | |
| Pi Camera Module 3 Wide + 500 mm cable | 1 | $40 | |
| KW10 ×10, US-100 ×5 (trig/echo mode), TCA9548A, ReSpeaker, BME688, PIR ×3, MAX98357A + speaker | — | ~$110 | phase 3/4 |

## Decisions folded in

- **All-ST3215.** The STS3250 knee (D015 upgrade path) is not required by
  the torque audit, shares the case but not the 12.6 V rating, pulls 4.2 A
  through 2 A pins, and lists at ~$88 now. Buy one only as a measured sample.
- **ESP32 driver board is core.** $16 for deterministic bus timing and a
  hardware watchdog; also the fastest possible first servo motion.
- **Bench kit first, Pi later.** The laptop drives the bench; the Pi is a
  full-robot item.
