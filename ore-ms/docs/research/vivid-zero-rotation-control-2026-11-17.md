# ViViD++ zero-rotation control

This diagnostic reuses all 972 rows from the
[rotation agreement audit](vivid-nominal-rotation-agreement-2026-11-17.md).
On each interval with an existing proposal and two bracketed nominal motion
signals, compare the proposal with identity rotation on identical camera
endpoints. The two identity disagreements are the reference rotation
magnitudes. Missing rows remain explicit. The 5° result requires agreement
with both signals. No fitting, model calls or routing changes were made.

| Recording | Learned camera | Paired proposals | Proposal within 5° of both | Identity within 5° of both |
|---|---|---:|---:|---:|
| Bright | RGB | 79 | 46 | 56 |
| Bright | Thermal | 83 | 48 | 61 |
| Varying | RGB | 50 | 23 | 29 |
| Varying | Thermal | 62 | 25 | 34 |
| Dark | RGB | 53 | 25 | 34 |
| Dark | Thermal | 65 | 30 | 40 |
| Total | Both camera arms | 392 | 197 | 254 |

The learned estimates added zero passes and lost 57 relative to identity
on this supported population. On the 121 learned proposals where both
reference rotation magnitudes exceed 5°, none agree with both within 5°.
Errors can still decrease without reaching the endpoint. In dark RGB, the
estimate reduced disagreement to both references on 8/53 paired proposals,
increased both on 38, and had mixed directions or ties on seven. SIFT RGB
has one added 5° pass across the bright and varying records, with 13 losses
and no dark proposals.

The 39 RGB-first fallback additions yield 18 dual-reference 5° passes with
the selected thermal proposal and 22 under identity on those same endpoints.
The thermal proposals add no passes over identity and lose four. This
addresses the value of the available proposals within the nominal comparison.
It does not support a zero-rotation deployment policy.

As a separate direction sanity check, inverting all proposals lowers combined
learned dual-reference passes from 197 to 174. This check keeps the declared
source-to-target convention and does not select a direction from outcomes.
The two reference signals retain the earlier timing, frame and integration
limitations. Neither nominal result qualifies 1° pose accuracy. No translation
scoring or raw-output recovery claims follow.

Verified hashes and all per-anchor comparisons are saved in
`/root/ore-ms-artifacts/vivid-plusplus-intake/`. No additional data download,
inference or Box storage was required.
