# Fixed camera priority and selection headroom protocol

16 November2026, America/New_York. Post-result development control: OutdoorNight
RGB-first/thermal/gyro outcomes and prior SmokeRoom/FlashLight outcomes have
already been examined. This is not blind transfer, a new selector or a new
paper reading/reproduction. No model or threshold will be chosen from this replay.

## Why this control

The [gyro diagnostic](outdoornight-gyro-findings-2026-11-16.md) shows fixed thermal
outperforming RGB-first on moving intervals. Before adding a quality model, test
whether a fixed priority reversal explains much of that gap and its compute cost.
Earlier EXP078health/uncertainty, EXP087flow closure and EXP112geometry selection
already have mixed/negative fixed-camera controls; do not rename those routes.

Primary descriptive endpoint: thermal-first gyro agreement within1° on all242
frozen moving anchors, missing outputs counted as no agreement. Secondary:5°,
outputs, exact calls/projected seconds and label-assisted headroom at both bounds.
Retain all971anchors,964common-gyro anchors and fixed thirds; no new cohort fit.

## Status-only priority and cost

For thermal-first, request thermal iff input-qualified; retain its`ok`proposal.
Otherwise request RGB iff input-qualified and retain iff`ok`; otherwise abstain.
Reuse pure `smoke_rgb_first_fallback.choose` by exchanging camera names, without
reading gyro disagreement or any score. Controls are fixed RGB, fixed thermal
and unchanged RGB-first. Both orders have the same status union by construction.
Choices retain sensor, own source/target stamps and paths. Unknown translation
scale remains; do not compose poses or infer full-VO recovery.

Lock all status-only choices before joining the frozen gyro disagreement rows.
Cost counts supported requests, including failures. Projected time sums original
synchronized pair seconds for the requested calls. No actual lazy execution,
startup/residency/sensor-energy or equal-accuracy saving claim. No extra inference.

After choices lock, compute the per-endpoint hindsight either-agrees count,
best-fixed-camera count and their difference on each declared cohort. Count both
supported cases where one camera agrees and the other disagrees. Oracle choices
use gyro labels and are only a ceiling, never a deployable policy or achieved gain.

## Multi-record limits without inventing missing raw evidence

Use first learned moving-cohort tables from the completed SmokeRoom and FlashLight
findings. Record N, RGB/thermal outputs, output union and each camera's1°/5° counts.
Do not rerun their absent raw predictions or claim a new paired benchmark. Their
targets are supplied-reference rotation error; OutdoorNight uses nominal gyro
agreement. Report them separately, never pooled into one accuracy statistic.

Let q=union_outputs-thermal_outputs be the number of RGB-only outputs, and b=
rgb_outputs-q be both-supported outputs. Thermal-first agreements equal thermal
agreements plus RGB agreements among those q RGB-only outputs. Thus the tight
summary-compatible interval is

`thermal_agreements + max(0, rgb_agreements-b)` through
`thermal_agreements + min(rgb_agreements,q)`.

Verify the bound exhaustively on small finite populations. An upper bound below
the established RGB control rejects universal thermal priority for that target
without inventing the unknown paired outcomes. Bounds are not actual replay scores.

## Audit and stop

Pin original predictions/decisions, gyro targets/errors/summary/audit, count-source
findings, protocol and consumed source. Independently reconstruct routing, calls,
times, endpoint counts, both-supported disagreements and oracle ceilings; reject
semantic decision/cost/oracle/bound corruption in CPU controls. Preserve all
positive/null/negative cases. Stop after this baseline/headroom audit; no fitted
gate, error-selected window or model adoption follows automatically. New quality
selection requires a separate recording-level evaluation and fixed-camera controls.
