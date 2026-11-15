# Sensor transfer: available evidence and qualified claims

14 November 2026 UTC, `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus preserved working changes.
The finite intake selected by the [contribution decision](contribution-synthesis-decision-2026-11-14.md)
is complete. No new inference, training, pose scoring, clock fitting or downloads.

## Decision

The three exposed MS² prefixes support nominal provided-geometry development and
have actual image/depth/reference files. They do not constitute three different
thermal sensors. A physical cross-sensor accuracy or timing-transfer experiment
is not qualified by the inspected bundle. TD3 supplies a distinct historical
rig/data route, with useful calibration and owner evidence, but its original
recording directories are not restored at the checked paths. Its reference and
checkpoint-training qualifications also remain relevant.

This is a claim-specific readiness result. Existing provided-reference results
remain valid under their stated assumptions. Synthetic and self-supervised
learning do not require independently measured dense flow for every training
image, and are not ruled out by this inventory. Neither new data acquisition
nor another clock search is selected as an automatic next step.

## Current files: fixed populations only

| MS² recording | Existing exposed stems | Thermal-left files | Single-scan thermal depth maps | Supplied RGB pose files | First two stereo captures |
|---|---|---:|---:|---:|---:|
| Day `_2021-08-06-11-23-45` | 000000–000019 |20|20|20|4 views|
| Night `_2021-08-13-21-18-04` | 000000–000019 |20|20|20|4 views|
| Rain `_2021-08-06-16-19-00` | 000000–000019 |20|20|20|4 views|

All 192 file bindings match their prior SHA256 values: 60 left images, 60 depth
maps, 60 opaque pose files and 12 first-pair stereo views. These are 192 paths
but 186 distinct byte hashes: six left views are copies in the stereo cache.
PNG headers confirm all 132 image/depth paths are 640×256, 16-bit grayscale.
No pixels are decoded and no reference matrices are newly parsed.

The historical manifests declare 80 RGB/thermal stereo image paths per prefix.
None of these original paths is live under either the current ore-ms root or
recovered persistent-window ore-ms root. The relocated files above are live.
Thus the available first-pair stereo input is not a restored 20-frame stereo
sequence. This path check does not claim that other files/backups do not exist.

Nine already-exposed TD3 recordings remain in the exposure ledger:
`1_salon_mac_cov_day`, `0_salon_so_day`, `night_thermal_so_salon2`,
`night_mac_salon_warehousefig8`, `night_so_waypoint_skip2`,
`night_macthermal_salon_redcourse_waypointskip1`, `night_teleop_so_salon_fig8`,
`night_thermal_so_salon`, `night_thermal_so_salon3`, all under `20251027`.
For each, neither `data/tartandrive3/20251027/<name>` under the working root nor
the November13 backup-tree root exists. This is a bounded 18-path check, not
archive inspection or a global availability census. The old staging note's
“restored locally” statement is historical and is not current path readiness.

## Sensor and preprocessing provenance

| Property | MS²: all three prefixes | TD3: nine historical recordings | Evidence class |
|---|---|---|---|
| Camera identity | Paper TableII specifies FLIR A65C | Earlier thesis review describes BOSON+640 context; exact per-record hardware association not established here | Author-specified dataset hardware / contextual hardware, respectively |
| Sensor grid | Published 640×512 | Historical raw thermal arrays 640×512 | Author specification / prior local measurements |
| Available/prepared grid | Current PNG headers 640×256; native MACT preparation keeps this grid; prior U²Flow path resizes to 320×128 | Historical MSA replay uses 640×512 maps; native TD3 loader consumes provided uint8 images and camera-info dimensions | Current header/source evidence / historical replay evidence |
| Export processing | Author rectification and crop precede the uint16 files | Raw-first MSA fisheye rectification is separate from original supplied processing | Author description / pinned local replay source |
| Intensity preparation | Explicit percentile clipping/scaling to uint8, CLAHE, bilateral filtering, channel replication and `/255`; U²Flow area resize follows enhancement | Historical raw-first MSA remap precedes the producer's `firestereo_v2` and enhancement; TD3 loader itself does not normalize raw uint16 | Inspected local source; not proof of checkpoint training-recipe equivalence |
| Exposure duration | Not qualified for these captures | Not qualified for these captures | Unknown |
| Response constant | Not qualified for these captures | Not qualified for these captures | Unknown |
| Row period / trigger delay | Not qualified for these captures | Not qualified for these captures | Unknown |
| Rectified pixel → physical row/time | Not qualified in selected payload | MSA spatial map construction exists; physical readout/time relation unqualified | Spatial processing is not timing calibration |

MS²'s hardware and crop facts come from the targeted
[extended paper III-A/TableII and III-D/E](https://arxiv.org/html/2503.22060v1).
Its [setup page](https://sites.google.com/view/multi-spectral-stereo-dataset/setup)
also documents preprocessing. The uint16 files preserve sensor-value encoding,
not an untouched physical sensor grid. Consequently, assigning a sensor row time
directly from the current cropped/rectified image row is an additional modeling
assumption. A physical timing model requires the spatial mapping and acquisition
time convention; neither can be supplied by the intensity dtype alone.

The virtual 8ms response, 27.8µs row period and 0.5ms delay from our earlier
adapter remain **borrowed source-inspired parameters**, not measured values for
either MS² or TD3. No fit or reclassification is performed here. No calibrated
physical noise/coverage model is established.

## Timestamp, reference and alignment contract

MS² manifests retain all four integer-nanosecond camera timestamp streams for
these prefixes. This intake copies the 240 metadata entries without recomputing
offsets or fitting a clock. The official loader sorts left/right filenames and
uses matching indices. The local loader preserves source IDs/times and labels
their physical event semantics unverified. Authors describe synchronization
hardware; prior exported-time disagreements do not prove unsynchronized exposures.
The [previous clock review](ms2-clock-evidence-findings-2026-11-04.md) and
[strict-interface decision](shared-motion-input-contract-findings-2026-11-12.md)
remain authoritative; the latter is not a universal dataset admission rule.

The [official format](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format)
identifies navigation-derived per-camera poses and single-scan projected depth.
The inherited local convention is camera optical pose relative to the sequence's
first camera. Using RGB poses for thermal requires the nominal supplied rig
conjugation and same-ID pairing; no per-segment alignment, fitted scale or
reference-derived pose initialization is introduced. Navigation and LiDAR supply
separate sensing sources, while projection can share calibration/timing errors.
Opaque pose hash agreement proves identity, not physical thermal accuracy.
The local MS² adapter emits stereo frames, not a newly qualified synchronized
IMU packet with noise/extrinsic/timing provenance.

TD3's [owner-reply record](source-replies-2026-10-22.md) supersedes older blanket
uncertainty: MSA applicability has owner support, and GNSS origin is owner-identified.
Exact GNSS axes, lever arm, timing/receiver quality and RTK membership are not
fully specified. Super Odometry independence is tentatively owner-supported,
not verified from every input graph. The recorded SLAM proxy remains a separate
reference. Do not call GNSS origin wholly unknown or silently promote antenna
motion to calibrated camera-frame truth. The [calibration history](../calibration.md)
and existing GNSS experiments remain available; none is rerun here.

## Recording splits and what can be evaluated

| Intended claim | Readiness and limit |
|---|---|
| More real-image nominal matching diagnostics on MS² | Actual 60-frame mono inputs exist; the previous geometry diagnostic is already complete, so a renamed repeat adds no evidence |
| First-pair native stereo execution | Inputs exist for all three conditions; runtime was previously qualified; no new smoke is needed |
| Recording-grouped development | The three prefixes can define declared development folds, with no shared recording; all are already exposed, so no fresh-test claim |
| Cross-condition transfer | A distinct evaluation question, not cross-sensor transfer; checkpoint training overlap remains unknown |
| Nominal TD3–MS² comparison | Historical results exist and replay is a restoration option; dataset, optics, scene, preparation and reference differences confound attribution to sensor identity |
| Physical sensor-timing transfer / uncertainty coverage | Not qualified by current response/readout, physical clock or noise/reference evidence |
| Synthetic/self-supervised method research | Available in principle, subject to a distinct hypothesis and suitable evaluation; missing dense physical flow does not close T2 |
| New protected test | No reserve payload opened; no test assignment or exposure-ledger status changed |

MACT_Offroad checkpoint training membership remains unknown, including TD3
membership. Splitting our own recordings cannot establish network-training-held-out
status. Previous development training/evaluation on these MS² records is explicitly
recorded in the November10 exposure ledger. New noise draws, frame subsets or
condition names do not restore recording-level holdout status.

## Stop and continuation

Close this inventory with the available nominal-development route preserved and
the physical-transfer claim unqualified. Do not extend the synthetic severity
series, restore a large archive, rerun a successful first-pair baseline or repeat
clock fitting to manufacture readiness. The next research decision should select
a distinct mechanism and endpoint from the broad original-paper map, or explicitly
choose a nominal comparison with the limitations above. This intake does not
select a new superior algorithm and does not close supervision, representation,
persistent estimation, multispectral learning or active-perception gaps.

Artifacts: [inventory](../../results/sensor-transfer-intake/inventory.json),
[923-check verification](../../results/sensor-transfer-intake/verification.json)
and [bounded inventory procedure](../../results/sensor-transfer-intake/inventory-procedure.py).
They bind 24 reused local sources/records, three recovered manifests and the
procedure itself. Existing numerical audits are reused rather than rerun. No
external paper text, credentials or notebook content is included in this packet.
