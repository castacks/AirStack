# ViViD++ nominal rotation agreement

This post-hoc diagnostic follows the
[three-recording support comparison](vivid-three-condition-support-2026-11-17.md).
It asks whether dark RGB proposals and added fallback proposals agree with
both recorded motion signals. It reuses all 972 locked prediction rows across
the two frontends, two cameras, and 243 anchors. No new matching or routing
was performed.

For each camera, evaluate the proposal on its own image header endpoints.
Use the published signed offsets and hand-eye transforms, interpolate the
released mocap quaternions, and integrate the external gyro through those
same endpoints. Express both rotations in the source-to-target camera
convention used by the proposal. Verified agreement metrics against the
earlier frame-invariant preflight and preserved missing inputs, missing
estimates and unbracketed intervals. Original prediction hashes still match.

## Learned proposals at the 5° diagnostic endpoint

| Recording | Camera | Proposals | Within 5° of both | More than 5° from both | Mixed | Unbracketed proposal |
|---|---|---:|---:|---:|---:|---:|
| Bright | RGB | 79 | 46 | 29 | 4 | 0 |
| Bright | Thermal | 83 | 48 | 30 | 5 | 0 |
| Varying | RGB | 51 | 23 | 25 | 2 | 1 |
| Varying | Thermal | 62 | 25 | 30 | 7 | 0 |
| Dark | RGB | 53 | 25 | 25 | 3 | 0 |
| Dark | Thermal | 65 | 30 | 31 | 4 | 0 |

For dark RGB, 25 of 53 proposals differ from both nominal signals by more
than 5°. On those 25, the median discrepancy is 11.44° against mocap and
10.27° against gyro. Among 42 dark RGB anchors where both references indicate
at least 2° motion, 36 have a proposal. Of those, 10 agree with both within
5°, 23 differ by more than 5° from both, and three are mixed.

| Recording | Extra thermal proposals under RGB-first | Within 5° of both | More than 5° from both |
|---|---:|---:|---:|
| Bright | 13 | 7 | 6 |
| Varying | 13 | 4 | 9 |
| Dark | 13 | 7 | 6 |

RGB-first's 39 extra proposals contain both nominally agreeing and
disagreeing outputs. Do not interpret availability alone as recovery or
choose a priority from these rows. Gyro and mocap share uncertainty from
timestamp and frame binding and differ by several degrees on these
half-second intervals. Agreement is descriptive. Independent 1° pose
accuracy and metric translation scoring remain unqualified.

The protocol, script, row ledger, summary and verified hashes are saved in
`/root/ore-ms-artifacts/vivid-plusplus-intake/`. The earlier
[reference preflight](vivid-handheld-payload-intake-2026-11-17.md) and
[learned support findings](vivid-varying-learned-support-2026-11-17.md)
retain the calibration, runtime and checkpoint context. No additional
download, inference, or Box storage was required.
