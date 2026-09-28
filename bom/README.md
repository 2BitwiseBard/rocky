# Bill of materials

`BOM.csv` is the one list of what Pebble needs. GitHub renders it as a table,
any spreadsheet opens it, and a price change is a one-line diff. Prices are
USD estimates checked on **2026-09-28**, before tax and shipping. Check them
again at checkout, and change them in `BOM.csv` only: no other file repeats
a price. `python3 bom/totals.py` prints the subtotals below and checks that
every line total is qty × unit.

## Reading the CSV

| column | meaning |
|---|---|
| `id` | phase letter + number (`A-01`). Backlog items are `B<n>` with no dash, decisions `D<nnn>` |
| `phase` | **A** bench kit (one leg moving on the bench) · **B** full robot (five legs, power, compute) · **C** senses · **D** voice/brain · **X** optional additions and tools |
| `spec_notes` | what to check before ordering and why the part is there |
| `qty`, `unit_usd_est`, `line_usd_est` | line = qty × unit; a pack (e.g. "(100)") is qty 1 |
| `source` | vendor + URL where the price was seen |
| `price_checked` | the date the listed price was seen, or `est` for a typical-range estimate |
| `status` | `to buy` · `wait until one leg walks` (buy later, on purpose) · `optional` · `have` (workshop basics: buy only if missing) · `choice` (buy only if a bench measurement says so) · `alternative` (replaces another row; never summed) |
| `design_ref` | the `cad/params.yaml` key, CAD part, decision (`docs/decisions.md`) or backlog item (`docs/DESIGN_BACKLOG.md`) that sets the part |

## Totals (2026-09-28)

| | to buy | wait | total |
|---|---:|---:|---:|
| A bench kit | $276 | | **$276** |
| B full robot | $644 | $153 (Pi 5 4 GB + cooler + card) | $797 |
| C senses | $134 | | $134 |
| **Core robot (A-C)** | | | **$1,207** |
| D voice/brain (optional) | | | $21 |
| X optional, near term (power monitor, downward ToF, kill relay, Wi-Fi adapter, logic analyzer, second pack, pins, whisker wire, filament dryer) | | | $216 |
| X optional, later (sensor pods, depth camera) | | | $214 |
| Workshop basics, if missing (calipers, meter, scales, iron, crimper, PETG, PLA) | | | $212 |

Each row carries the cheapest reliable source seen. Buying the bench kit
all on Amazon costs about $60 more, mostly the servos and the two Waveshare
boards; the rows' `source` notes carry the Amazon links where they exist.

The brains (speech, vision, tool-calling models) run on a separate computer
with a GPU over Wi-Fi; that computer is not in this list.

## Buy first: phase A

One leg on the bench: four ST3215 (three joints + a spare), the Bus Servo
Adapter (A), the ESP32 Servo Driver (its web UI sets IDs and moves a servo
with no code), an adjustable current-limited bench supply, the leg's
fasteners and inserts, the carbon tube, the SEA springs and the foot
switches. Everything in B waits until that leg passes
`bench/BENCH_RUNBOOK.md`; the Raspberry Pi waits until one leg walks
(the laptop drives the bench through the adapter).

## Safety

- **First contact is current-limited.** Set the supply to 12.0 V with a
  ~1.5 A limit and check it with a meter before any servo sees power.
- **ST3215: the 12 V / 30 kg·cm variant only.** The 7.4 V one looks the same.
- **SCS0009 hand servos never touch 12 V.** They get their own 6 V UBEC;
  measure 6.0 V at the plug before a claw goes on. Data and ground are shared,
  V+ is not.
- **Soft-case pack only**, at most 138 × 44 × 25 mm (D062: the sled narrowed to fit its bay). Hard-case 3S 5200 packs
  are ~37 mm tall and do not fit the 30 mm bay.
- **LiPo:** voltage alarm on the balance lead whenever the robot runs; charge
  in a LiPo bag, never unattended; store at storage charge.
- **Fuse, then loop key**, first thing after the pack. Each leg's 12 V fans
  out at the coxa: servo connector pins are rated ~2 A.
- **ESP32 driver input is 6-12 V.** Run it from the bench supply at 12.0 V,
  not from a full 3S pack (12.6 V).

## Things not to buy

- **A servo tester.** Feetech bus servos do not take PWM; the ESP32 Servo
  Driver is the bus-servo tester.
- **Generic 25T disc horns.** The couplers are cut for the stock ST3215 horn's
  45° hole pattern; the spare servo's bag is the spare horn.
- **Stainless washers for the magnet strikes.** 304 is barely magnetic: the
  deck strikes are zinc-plated steel DIN125 (B-20).
- **PETG-CF for the legs.** Stiffer, but weaker across layers and more
  brittle, it needs a hardened nozzle, and the FEM check assumes PETG.
- **KW11 / KW12 microswitches.** ~20 mm long; the foot pocket takes a KW10.
- **683ZZ bearings, a USB-UART dongle, STS3250 knees, a pan turret.** Older
  plans had them; the current design (D047 and later) does not use them.
