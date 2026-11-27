# Successor evidence acceptance plan — not execution authorization

D2 governs criteria; D4 governs applicable raw evidence, native clocks and frames; D5 governs equivalence; E7 separates data acceptance, test acceptance and qualification. F00 remains frozen.

Before execution, resolve entry gates and approve a per-requirement card with test/configuration/attempt IDs, source version, exact criterion, measurement interval, clock map, independent truth, uncertainty, abort/stop plan and evidence recorder admission. No unresolved [P]/[C] value becomes a pass threshold by implementation default.

For each immutable attempt, inventory raw files with bytes/SHA-256/producer/acquisition time and their external location; retain failures, aborts and missing channels. Require independent recorder supervision (REC-01), OS exit statuses surviving executor failure, zero-loss evidence where D2 requires it and explicit completeness decisions. Hash the manifest and bind it to the delivery notice. A summary, socket, action success or upload is insufficient.

Independently recompute every criterion with source/model/configuration and native clock/frame provenance. Keep data rejection separate from behavioural FAIL/INDETERMINATE. Cross-simulator differences are reports, not CFG-SIL-01 equivalence. HIL covers target execution, not real aerodynamics. REAL claims apply only to the aircraft and conditions actually flown. Qualified successor closure requires independent review and PD decision.
