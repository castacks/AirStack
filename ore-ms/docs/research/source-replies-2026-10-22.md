# Recording-owner replies: MSA applicability, GNSS origin and training provenance

Received from the user on 2026-10-22 (America/New_York), branch `ore_ms`, base `51fd115be`. The user pasted replies attributed to Geethika Hemkumar (displayed 11:18 PM) and Matthew Sivaprakasam (11:02 PM). Original Slack permalinks, message calendar dates and display timezone were not supplied or independently retrieved. These are attributed owner statements, not newly inspected source code. No messages were sent.

## What changes

| Item | Supplied reply | Updated working status |
|---|---|---|
| MSA applicability | Geethika says the MSA calibrations should apply to the recordings being tested | Applicability now has owner support in addition to the fixed image checks and raw-first replay. Continue using MSA as the working thermal calibration baseline. This reply does not separately define the GNSS/camera/body transform. |
| GNSS generation | Matthew says the topic comes directly from the GNSS system, without processing on the lab side | Treat the exported stream as an owner-identified, separately generated GNSS reference candidate. Do not continue describing its sensor origin as wholly unknown. Onboard receiver filtering/aiding and timestamp details are not specified by this reply. |
| Super Odometry independence | Geethika says she thinks NovAtel odometry is not used as a Super Odometry input | This supports proceeding with a provisional independent-source translation comparison. The wording is tentative; it does not prove the exact SO input graph or statistical independence, including possible shared IMU inputs. |
| Coordinates | Matthew says UTM, with an approximate X/Y flip | Coordinate family identified; exact axis permutation/signs, zone and orientation convention remain unresolved. Do not guess a swap, reflect quaternions or fit an alignment to VO errors. |
| GNSS accuracy | Matthew reports some jitter; some runs have RTK, with membership unknown | Usable contextual reference, not certified centimetre ground truth. Check recording-specific quality/RTK and interval behavior; longer displacements may be more informative than tiny near-static increments. |
| Thermal checkpoint training | Geethika does not know the recordings used for MACT_Offroad.pth and identifies Yifei Liu as the likely source | Training/finetuning split remains unknown. Previously unused-by-us recordings are not established training-held-out data. Ask about 20251027 membership and the exact checkpoint/split/recipe before generalization claims. |

These replies supersede the blanket “MSA October applicability/NovAtel origin unconfirmed” status in older audit notes, while preserving their historical evidence. They do not invalidate previous measurements or convert the SLAM proxy into ground truth. No reference, calibration, axis transform, weighting or VO settings were changed in this update.

## Answer to Geethika's calibration question

Yes: [EXP-023](../../experiments/EXP-023-msa-preprocessing-vo.md) compared raw-first MSA preprocessing against the dataset calibration on the same fixed windows, unchanged checkpoint/settings and the same recorded-SLAM proxy. Day translation RMSE fell from 1.5171/0.7027/2.8966 m to 0.0763/0.1600/0.1792 m: about 95%/77%/94% improvement. Night was mixed: early translation improved, middle changed little, and late near-static discrepancy remained. Rotation did not improve uniformly.

This is evidence that MSA works better in those day-window proxy comparisons, not a blanket independent pose-accuracy claim. The experiment changes the combined lens/stereo calibration and rectification, rather than isolating intrinsics from extrinsics. Later thermal operational completion does not by itself establish physical metric accuracy.

## Available fields and remaining data

The inspected converter configuration binds `novatel_gps_odom` to `/odometry/filtered_odom` as `OdomRBState`. Its 13-field schema is position3, quaternion4, linear velocity3 and angular velocity3. The inspected converter copies these fields and header/frame IDs, omitting pose/twist covariance and receiver solution quality. This schema inspection does not independently identify the exact converter/receiver implementation used for every recording.

The latest source export retains `map -> gps_frame`. A filename search of the complete central-directory inventories for the three separately supplied archives finds NovAtel odometry/IMU and GQ7-related TF names, but no explicitly named RTK/fix/receiver-covariance logs. This is a bounded name-based inventory, not proof those fields were absent from the original bags or hidden in other metadata. `gq7_gnss_*` TF frame names do not establish that the exported NovAtel stream had RTK.

**If available, the next additional reference data needed are original odometry covariance and NovAtel solution/fix/RTK-quality messages for the selected intervals, with their timestamps.** Another image archive alone cannot supply this information. No bulk date download is requested. An owner explanation of which runs had RTK is useful even if quality messages were not exported.

## Immediate next comparison

Proceed with separately labelled GNSS translation checks on existing saved windows; no new VO inference is necessary. Begin with interval horizontal displacement magnitudes using original timestamps and float64 coordinates, retaining SLAM and GNSS as separate references. Such magnitudes are invariant to a signed horizontal axis permutation, but still depend on timing, GNSS noise and the physical antenna/camera origins. Do not relabel antenna displacement as camera error or adopt a full pose/heading reference yet. Keep near-static intervals descriptive rather than treating their small GNSS increments as exact motion truth.

Use a documented coordinate conversion, receiver origin/lever arm and timing/quality checks before vector/heading/camera-frame error comparisons. Avoid fitting the coordinate convention, scale, latency or lever arm to the same VO results being evaluated. No numerical GNSS evaluation or reference replacement is performed by this documentation update.

## Focused follow-ups — drafts, not sent

To Geethika:

> Thanks, yes—the MSA calibration worked substantially better on the day windows: translation RMSE against the same SLAM reference dropped by about 77–95%. Night results were mixed, so I’m checking those against GNSS as well. I’ll follow up with Yifei about whether any 20251027 recordings were used to train or finetune MACT_Offroad.pth.

To Matthew:

> Thanks—that helps. Do you know the exact XYZ/UTM axis conversion for /odometry/filtered_odom, and whether night_thermal_so_salon2 or night_mac_salon_warehousefig8 had RTK or saved fix-quality logs?

To Yifei:

> We’re evaluating MACT_Offroad.pth on 20251027 recordings, including night_thermal_so_salon2 and night_mac_salon_warehousefig8. Were any recordings from that date used for training or finetuning? Is there a recording/split list and the preprocessing/calibration recipe for this checkpoint?

The origin represented by gps_frame and any antenna/IMU lever-arm compensation remain questions for the frame/reference audit, even after the axis convention is clarified.

## Preservation

The original user-pasted wording, bounded archive-name inventory, inspected local schema/frame/config sources and SHA256 registry are saved under `results/source-replies-2026-10-22/`. Existing frozen scientific inputs are verified unchanged; there is no new inference or scientific test in this source/documentation update. Incremental source and evidence are preserved at Box `ore-ms/backups/2026-10-22-source-replies`, with prior research/model/data snapshots remaining prerequisites.
