# Longer day comparison stops on missing supported thermal events

15 November2026. The [frozen first600frame protocol](long-day-strong-controls-protocol-2026-11-15.md)
requires all599increments of `_2021-08-13-16-31-10` with a thermal contribution
and forbids dropping pairs or introducing fallback. Capture stops at25→26:
there is no supported saved native thermal first-write for that interval.
No full-span accuracy or runtime comparison is qualified or scored.

## Input-only resolution

A whole-span audit of the unchanged saved inputs finds599/599supported RGB
intervals and595/599supported thermal intervals. Thermal lacks a supported native
first-write at25→26,27→28,28→29 and32→33. Independent direct event-map inspection
confirms exactly those four unsupported intervals. Source/capture/config hashes
still match the original frozen input pins. This describes saved native estimator
support, not proof of absent camera images or physical thermal sensor failure.

The driver indexed the thermal event map before a full-span support preflight and
raised `KeyError` atpair25. That implementation/preflight mistake is preserved,
not described as a solver failure. The25completed rows, original source, freeze
and log remain at `results/long-day-strong-controls/run-001/`. There is no prediction
lock or reference score; the successful prefix is not substituted for the frozen
population. `input-support-audit.json` records the599pair inventory and independent
event-support cross-check. No new solves or reference scores followed the stop.

## Decision

The all-thermal full-coverage comparison is unexecuted beyond its partial attempt.
Four unavailable estimator intervals prevent it from satisfying the declared
contract. Do not infer an accuracy regression, inability of thermal to help, or
scientific closure of longer-day fusion from this engineering stop.

The earlier [short day gain](thermal-strong-rgb-findings-2026-11-15.md) and
[longer-night regression](fixed-rotation-continuation-findings-2026-11-15.md) remain
unchanged. No interval is removed, no tolerance changed and no RGB-only fallback
is silently inserted. A practical full-coverage study would first need an explicit
missing-modality contract, such as a separately declared fallback policy, and its
own pre-prediction freeze. It must count all599outputs and thermal usage separately.
That is a possible next decision, not an executed or automatically selected study.

Whole-span, per-arm input availability must be checked before future freezes and
capture launches. This prevents unnecessary partial runs while retaining missing
support as evidence. No repeated inference, weight tuning, new data, physical timing
correction, model change or protected-data access occurs in this resolution.

The evidence packet preserves the failure and inventory outside Git. No whole-run
numerical gain, matched-cost claim or successful native reproduction is asserted
for this partial capture. No Box upload is performed.
