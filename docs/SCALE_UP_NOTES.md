# Scale-up notes — what survives the jump from Pebble to full Rocky

*Written in session 6 (2026-08-07), updated 2026-09-26. The owner's stated
ambition: if Pebble lands, full-scale Rocky gets real fabrication (CNC, SLS,
sent-out drawings). These notes capture, while the subscale design is still
fluid, which of our decisions are scale-portable and what "professional
quality" means for the handoff — so we keep banking the right assets now
instead of retrofitting discipline later.*

## What already scales (keep investing here)

- **params.yaml as SSOT + code-CAD.** This is exactly how you brief a fab
  house: change scale factors and regenerate STEP files with real
  tolerances. A hand-modeled tree would have to be redrawn; ours re-runs.
  Every part already exports STEP (Xometry/JLC/Protolabs quote directly
  from it).
- **The check suite (`run_all_checks.py`, 28 modules incl. the joint suite and
  printability).** Boolean
  interference, keep-outs, containment audits, engagement proofs — at full
  scale a failed fit costs a $200 machined part and two weeks, not 40 g of
  PLA. The habit of "no part ships without its own checks" IS the
  professional-quality bar; CI-for-geometry is rarer in hobby robotics
  than it should be.
- **The six frozen interfaces (I1–I6).** Interface discipline is what lets
  subsystems upgrade independently at scale (QDD actuators at Phase 5 swap
  behind the leg port the way tools swap behind I2). The *dimensions*
  rescale; the *contract* (loads through dowels/lips/shoulders, never
  latches — D020) carries verbatim.
- **The fit-ladder methodology.** Full-scale version = a machining test
  coupon on the chosen process (SLS shrinkage, CNC bore class) before
  committing the real order. Same protocol, different machine.
- **Decision log + build log.** A fab partner (or a future collaborator)
  can be onboarded from decisions.md in an afternoon. Keep filing.

## What does NOT scale (known, deliberate subscale shortcuts)

- **Self-tapping screws into printed and servo-rim holes, thread-forming M3
  into PETG** (the D047 servo cups, plate B, the tube pinch bolt): at Rocky
  scale these become machined bosses + threaded inserts or steel hardware.
- **PLA/PETG structural sections**: cube-square law is unkind — Rocky's
  femur is a CNC 7075 or SLS-nylon+carbon part with actual FEA, not a
  6 mm printed plate. The *geometry* (twin-hub link, lightening pattern)
  is a valid starting shape; the sections are not.
- **Printed springs-and-detents** (I2 detent bump, latch cam): fine at
  hand loads; full scale wants steel detents/cam followers.
- **Printed couplers between the horn and the links** (D047): the
  sacrificial-part idea carries, but at scale the part becomes a machined
  hub.

## Handoff hygiene to start now (cheap) 

- When a dimension gets a measured value (caliper sessions), record the
  MEASURED value and the FUNCTION (slide/press/tap) in params comments —
  a fab house needs the intent (fit class), not our printer's compensation.
- Keep VERIFY flags honest: at quote time, every VERIFY still open is a
  question the machinist will bill to answer.
- STEP is the exchange format; STLs are for our printer only.
- Full-scale gate (the founding plan's Phase 5,
  [archive/ROCKY_MASTER_PLAN_v1.1.md](archive/ROCKY_MASTER_PLAN_v1.1.md)) triggers a tolerance re-derivation
  pass: FDM clearances (0.30 fit / 0.15 press) do not transfer to CNC
  (ISO 286-style fits) or SLS (process shrink + minimum wall 1.0–1.5).
  That pass regenerates from params.yaml — which is why params stays SSOT.
