# Archive

Superseded plans, early logs and full-length records, kept word for word so every
measurement and decision stays readable in the working tree. Nothing here is maintained:
where an archived file and a live doc disagree, the live doc (and `docs/decisions.md`)
wins. Archived text is verbatim except that personal identifiers are genericized (the
owner's first name reads "the owner"; machine paths read as `ROCKY_*` settings; the
reference laptop's GPU is named by its size) and personal details (owned gear, store
locations) read `[personal note removed]`.

| File | What it is | Dates | Why archived | Superseded by |
|---|---|---|---|---|
| [ROCKY_MASTER_PLAN_v1.1.md](ROCKY_MASTER_PLAN_v1.1.md) | the founding plan: phases, torque math, electrical sketch, parts tree, risks | July 2026 (v1.1) | the origin, overtaken by decisions; kept whole because code cites §3.4 and §7. Bannered with a "superseded since v1.1" table and the D016 correction: the SCS0009 hands are **not** on the 12 V rail | [README.md](../../README.md), [decisions.md](../decisions.md), [bom/BOM.csv](../../bom/BOM.csv), [PRINT_PLAN.md](../PRINT_PLAN.md) |
| [BUILD_LOG_2026-07-27_to_2026-09-01.md](BUILD_LOG_2026-07-27_to_2026-09-01.md) | build-log sessions 1a–8d | 2026-07-27 → 2026-09-01 | the design era before D047 (the leg rebuilt around the measured servo); its parts, order sheets and several docs no longer exist | [BUILD_LOG.md](../../BUILD_LOG.md) (from session 8e; its last table indexes these sessions) |
| [decisions_D036-D057_full.md](decisions_D036-D057_full.md) | decisions D036–D057 (with D052a, D055a) at full length: measurements, A/Bs, what was tried and rejected | 2026-08-30 → 2026-09-25 | the rows had grown into session logs; the live log keeps each as a short ADR row | [decisions.md](../decisions.md) (condensed rows, AMENDED markers, D058+) |
| [BRAIN_MODELS_2026-09-24.md](BRAIN_MODELS_2026-09-24.md) | the brain-model research round and the 2026-09-25 bench status (22 candidates, five brains measured) | 2026-09-24 → 2026-09-25 | a dated report; the current picks and how-to are a short guide | [BRAINS.md](../BRAINS.md) |
| [prep-2026-10-01/](prep-2026-10-01/) | the two read-only measurement rounds ahead of the body-layout build (BUILD_LOG 9p): `PREP_REPORT_1.md` (the keel tub in the sim, decision 14, keep-outs), `PREP_REPORT_2.md` (the I1 docking fault and its three fixes, decisions 13/14 re-checked, lanes, Pi USB, option B, connectors), `I1_DOCK_OPTIONS.md` (the judge's I1 comparison), `base_shell_check.py` (the orchestrator's own coxa-base × carapace check). Every number names the script that produced it; the scripts (about 1,000 files) are not in the repo: they sit in the owner's Claude project folder (`prep-2026-10-01/`, see OWNER_TODO) and are listed for promotion in `PREP_REPORT_1.md` §6. Paths of the form `S/…`, `S2/…`, `prep/…` refer to that folder | 2026-10-01 → 2026-10-02 | dated measurement records; their conclusions are backlog rows B117–B138 and the corrections block in the proposal | [DESIGN_BACKLOG.md](../DESIGN_BACKLOG.md) B117–B138, [BODY_LAYOUT_PROPOSAL.md](../BODY_LAYOUT_PROPOSAL.md) ("Re-measured 2026-10-01/02") |

## Old names in archived files

Archived files keep the paths of their day. Where one names a file that has since moved or
gone, this is where its content lives now:

| Old name | Now |
|---|---|
| `plan/ROCKY_MASTER_PLAN.md` ("the master plan") | [ROCKY_MASTER_PLAN_v1.1.md](ROCKY_MASTER_PLAN_v1.1.md) |
| `docs/PRINT_PLAN_2026-09-22.md`, `docs/PRINTER_NIGHT.md`, `docs/PRINT_NIGHT_s8.html`, `docs/PRINT_WEEKEND_s6.html` | [docs/PRINT_PLAN.md](../PRINT_PLAN.md): the fit-ladder GO/NO-GO table, the interface-coupon fit criteria, and the fixture and carapace print settings. The night-by-night print queues themselves are only in the git history |
| `PRINT_PREP_PACK.pdf` (repo root) | `cad/out/PRINT_PREP_PACK.pdf` (`cad/gen_print_pack.py`) |
| `bom/ROCKY_BOM.xlsx`, `bom/SHOPPING_LIST_2026-09-22.md`, the order sheets and addenda (`bom/ORDER_SHEET_batch01.html`, `bom/ORDER_ADDENDUM_batch3.html`, `bom/ADDENDUM_session5.html`, `docs/pebble_order_sheet_v2.html`) | [bom/BOM.csv](../../bom/BOM.csv) + [bom/README.md](../../bom/README.md) (D058) |
| `docs/BUS_STARBOARD.md` | the star-board section of [docs/WIRING_HARNESS.md](../WIRING_HARNESS.md) |
| `docs/BRAIN_MODELS_2026-09-24.md` | [BRAIN_MODELS_2026-09-24.md](BRAIN_MODELS_2026-09-24.md) (here) and [docs/BRAINS.md](../BRAINS.md) |
| `docs/VISION_PLAN.md`, `docs/SENSING_PLAN.md` | [docs/PERCEPTION_PLAN.md](../PERCEPTION_PLAN.md) |
| `docs/RL_TOUR.md` | [docs/RL_GUIDE.md](../RL_GUIDE.md) |
| `docs/MCP_CONTRACT_v0.md`, `docs/PLAYGROUND.md` | [docs/SIM_GUIDE.md](../SIM_GUIDE.md) (the MCP invariants); the tool list is [docs/TOOLS.md](../TOOLS.md), generated |
| `docs/LAPTOP_SETUP.md` | the Setup section of [README.md](../../README.md) |
| `docs/NEXT_SESSION.md`, `docs/SESSION_WORKFLOW.md` | retired 2026-09-22; git is the record |
| `sim/run_*.py` experiments, `sim/diag_brace_phase.py` | `sim/experiments/` (`sim/run_sim.py` stays) |
| `sim/*_results.json`, `sim/fig_*.png`, `sim/out/patrol_report.json`, `sim/out/reflex_fallen.json`, `sim/plot_results.py` | `sim/experiments/results/pre_d052/` |
| `audio/chordspeak.py` (chord-speak v0), `audio/make_audition.py`, `sim/ppo_smoke.py` | removed 2026-09-26 (D060); git keeps them |
| `sim/runs/recover2`, `recover3_capped`, `recover3_scratch`, `recover5_v3` | removed 2026-09-26 (negative results, recorded in [docs/RL_GUIDE.md](../RL_GUIDE.md)); git keeps them |
