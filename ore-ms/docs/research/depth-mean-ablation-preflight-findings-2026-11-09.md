# Depth-mean ablation: bounded preflight

9 November 2026, America/New_York. Branch `ore-ms-orb-development`; freeze `ecc4c930e`.

**Native replay passes; the four-arm experiment has not run.** Exact source/target depth maps for the selected packet are absent locally and from the two inventoried depth-reference backups. No mean-changing arm, covariance-control arm, training run or accuracy comparison is substituted for the missing inputs.

## Executed work

Selected the first 20 consecutive pairs (source groups 0–20) in each RGB/thermal stream of the existing day/night/rain captures. All 120 native slots are present. Native configuration is static-motion initialization, analytic disparity `TwoFrame_PGO`, CPU optimization and mapping disabled.

Used the unchanged native graph definitions, graph construction and solver loop from the earlier solver-repair experiment in the exact recovered runtime (`44ff9a1e6fa…`, Torch2.7.1+cu128/PyPose0.9.5). The original arm is unchanged: raw-residual Huber/FastTriggs, covariance-weighted linear solve, PINV/TrustRegion and original StopOnPlateau schedule. Stored parent poses supply initialization. Source factors and numerical observation covariance are checked unchanged.

| Recording | Native pair solves | Exact native parameter-array matches | Maximum translation / rotation discrepancy |
|---|---:|---:|---:|
| Day | 40 | 40 | 0 m / 0 deg |
| Night | 40 | 40 | 0 m / 0 deg |
| Rain | 40 | 40 | 0 m / 0 deg |

The declared parity tolerance was 0.001 m / 0.01 deg, inherited from EXP083. Exact parameter-array equality exceeds that requirement. Parity establishes faithful replay, not optimization correctness, convergence or physical accuracy. Fifty-seven runs use all ten scheduler steps; native convergence telemetry remains unavailable. Source-depth/ray residuals are retained as diagnostics; they do not certify a projected-depth intervention or physical correspondence.

No new frontend calls or runtime benchmarking. These 120 solves reproduce existing native outputs; they do not constitute new method gains.

## Depth-input gate

The packet requires 21 source/target depth maps per camera, 42 per recording, **126 distinct own-camera maps** total:

| Current native recording | Required maps | Exact maps in inspected backups |
|---|---:|---:|
| `_2021-08-13-16-31-10` | 42 | 0 |
| `_2021-08-13-22-16-02` | 42 | 0 |
| `_2021-08-06-16-19-00` | 42 | 0 |

The bounded backup lookup inspected manifests from EXP147 native-depth-proxy and EXP151 cross-recording-stereo-depth. Their label members belong to `_2021-08-13-21-18-04` and `_2021-08-06-11-23-45`, sampled every100frames. They differ in recording identity and cannot provide consecutive source/target labels for this packet. A filename such as `000000.png` is insufficient without its sequence and camera identity. Locally saved `first-depth.npz` files are model predictions, not supplied depth labels.

Only the small backup manifests were fetched. The 334MB/235MB scientific archives and the 1.61GB ORB runtime archive were not downloaded, because their listed labels cannot satisfy this gate. No source archive, additional recording or reference pixel data was acquired. This inventory is scoped to those two relevant backup packets; it is not a claim that the required maps do not exist elsewhere.

## Verification

Two focused tests pass: exact sequence/camera/stem matching rejects wrong-record and nonconsecutive maps; parity comparison handles translation/rotation discrepancies and quaternion sign equivalence. Independent standard-library audit checks all 32 frozen source/input hashes, all 120 source-event/initialization identities, exact native parameter-array equality and the 126 missing map identities. It imports no producer. All original inputs remain unchanged.

The [frozen preflight](../../experiments/depth-mean-ablation-preflight-2026-11-09.json) and [outcome](../../experiments/depth-mean-ablation-preflight-outcome-2026-11-09.json) retain complete case identities, source hashes, required map paths and the audit. Replay results/logs and inventoried manifests are in `results/depth-mean-ablation/preflight-001` and the companion Box packet.

## Decision and exact continuation

Stop this bounded execution pass at input qualification. Keep the faithful replay as reusable infrastructure; do not repeat its 120 solves merely to rediscover this input gap. No conclusion about the depth-mean hypothesis follows from an unexecuted intervention.

Continuation requires targeted recovery of the listed `rgb/depth` and `thr/depth` maps for source IDs `000000`–`000020` in the three exact sequences, from a verified existing backup or the corresponding supplied source products. Then qualify native pixel/depth semantics and shared support, freeze the four-arm packet/covariance control, and execute it. Do not use another recording's depth, interpolate the sparse old labels, fit a pose scale, or launch another confidence model as a substitute.

This is a specific input dependency, not a decision to end RGB–thermal research. Current accuracy results and the RGB baseline remain unchanged.
