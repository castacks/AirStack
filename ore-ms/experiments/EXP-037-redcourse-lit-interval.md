# EXP-037: Sustained tracking and pairing around the redcourse illuminated interval

Recorded: 2026-10-22T23:19:27.352136-04:00. Branch `ore_ms`, starting commit `51f87a539` plus frozen follow-up source. **The brief illuminated interval has sparse RGB tracks and only6/1800 initial grid points persist through its original20 frames. A preceding20-frame block retains74/1800, but no continuous paired window passes the inherited timing gates. This is exploratory image readiness, not a VO accuracy or impossibility result.**

## Fixed expansion and source scope

The user authorized following up EXP-036's redcourse candidate. Before new pixels were inspected, freeze its original RGB-left20 acquisitions4173–4192, plus100 unique acquisitions before and100 after: indices4073–4292,220 frames over21.900676s. Exact camera interval:2025-10-27 23:09:54.151407–23:10:16.052083 EDT; original23:10:04.151911–23:10:06.051962 EDT. The capture is `20251027/night_macthermal_salon_redcourse_waypointskip1`. This selection is informed by EXP-036 outcomes and is exploratory, not an independent test set.

Retrieve nearest RGB-right, thermal-left and thermal-right members using full preserved timelines, without clock/axis/scale fitting. ZIP range acquisition verifies874 unique image members and four original timestamp files via CRC/SHA256; the full25.5GB archive is not downloaded/hashed. No learned inference, camera calibration, intensity recipe, pose/weight/default change. All1088 frozen previous files remain unchanged.

For tracking, independently deduplicate each selected stream's actual acquisition timestamps. There are220 RGB-left,220 RGB-right and217 distinct thermal-left images; nearest cross association reuses three thermal-left and three thermal-right frames. Repeated selections are not presented as fresh frames or temporally tracked duplicates. One20-position thermal block contains19 distinct acquisitions, so its20-frame survival is unavailable. Corresponding before/original/after nearest-thermal groups overlap at a boundary and are not independent disjoint thermal clips.

## Adjacent support and unreseeded survival

| Stream | Group | Unique frames | Median adjacent support | Texture-restricted support |
|---|---|---:|---:|---:|
| rgb_left | before | 100 | 5.78% | 1.22% |
| rgb_left | original | 20 | 10.56% | 4.33% |
| rgb_left | after | 100 | 4.22% | 0.28% |
| rgb_right | before | 100 | 6.33% | 1.50% |
| rgb_right | original | 20 | 11.11% | 5.11% |
| rgb_right | after | 100 | 4.44% | 0.33% |
| thermal_left | before | 98 | 70.14% | 45.54% |
| thermal_left | original | 20 | 77.48% | 48.81% |
| thermal_left | after | 100 | 69.25% | 46.53% |

Native grayscale LK inherits EXP-027/035 settings:32px border,16px grid,21x21 window,3 pyramid levels,30 iterations/epsilon0.01,minEig1e-4, finite/status/inbounds gates and<=1px forward/backward check. Adjacent support reinitializes a fixed source grid each pair. Tight<=0.5px and texture std>=5 sensitivities remain saved. RGB1024x544/1800 grid points and thermal640x512/1008 have different views/resolutions; these percentages do not calibrate relative physical reliability.

Separately, initialize one fixed grid per20-frame block and advect only surviving identities, retaining status failures and never reseeding. Report positions after5/10/20 images (4/9/19 transitions). Initial texture>=5 is a sensitivity population, not a learned selector. No acceptance percentage was fitted.

| RGB-left block | Original indices | Survive5 frames | Survive10 frames | Survive20 frames | Initial-textured survivors at last frame |
|---|---|---:|---:|---:|---:|
| before-0 | 4073–4092 | 17 | 10 | 1 /1800 | 0 |
| before-1 | 4093–4112 | 19 | 14 | 3 /1800 | 0 |
| before-2 | 4113–4132 | 63 | 30 | 1 /1800 | 0 |
| before-3 | 4133–4152 | 19 | 3 | 0 /1800 | 0 |
| before-4 | 4153–4172 | 143 | 108 | 74 /1800 | 43 |
| original | 4173–4192 | 202 | 63 | 6 /1800 | 0 |
| after-0 | 4193–4212 | 100 | 60 | 16 /1800 | 3 |
| after-1 | 4213–4232 | 37 | 22 | 11 /1800 | 0 |
| after-2 | 4233–4252 | 30 | 14 | 5 /1800 | 0 |
| after-3 | 4253–4272 | 26 | 3 | 1 /1800 | 0 |
| after-4 | 4273–4292 | 41 | 24 | 7 /1800 | 0 |

The originally illuminated window retains6/1800(0.33%) RGB-left identities through20 frames, with zero initially-textured survivors. RGB-right retains4/1800(0.22%), including one initially-textured survivor. The best exploratory neighboring block4153–4172 retains74 RGB-left tracks(4.11%),43 initially textured, and78 RGB-right tracks(4.33%),49 initially textured. This short precursor is a more persistent local image candidate than the original block; it does not establish sustained whole-window VO support or metrically useful stereo observations. Low survival can reflect occlusion, motion blur, changing illumination and LK limits; no single failure cause is inferred.

The original thermal block retains308/1008 tracks(30.56%); other fully20-frame thermal blocks retain~13.6–27.8%. This is algorithmic persistence under different scene/view conditions, not an accuracy comparison. No learned frontend/optimizer was exercised.

Fixed first/middle/last context previews for before/original/after were inspected for RGB-left. Visible content consists largely of briefly illuminated vegetation, with blur; no outcome-selected images replace the fixed previews. Native and explicitly labelled CLAHE display panels are saved; enhancement does not enter measured inputs. The separately identified strongest block is reported transparently as an outcome-selected candidate.

## Timing and pairing

The220 RGB stereo pairs have identical recorded times. Nearest thermal-minus-RGB offsets have median−40.819ms,range−41.091 to+59.447ms;41/220 are within30ms and217/220 within50ms. Thermal stereo exceeds the inherited2ms gate on136/220 nearest selections. Under joint RGB stereo<=2ms,thermal stereo<=2ms,cross-modal<=30ms, only16/220 rows qualify. With the separately labelled50ms cross-modal sensitivity,84 qualify; thermal stereo2ms remains fixed. Neither selection has even two consecutive unique qualifying frames under the inherited150ms continuity requirement; hence no20-frame continuous paired window.

These are recorded timestamp gates, not proof that the physical cameras are unusably synchronized. Some thermal gaps exceed150ms only by95–96ns, and some stereo differences are near gate boundaries; preserve exact decimal-derived times and do not describe numerical boundary excursions as a newly discovered large data dropout. The major joint eligibility limitation includes systematic~41ms cross-modal separation and thermal stereo offsets, not RGB temporal discontinuity: RGB gaps remain~100ms with none>150ms. Any relaxed stereo/synchronization policy needs separate justification and evaluation rather than silent threshold tuning. No corrected timestamps, synthetic images, interpolation or substituted one-to-one matches are adopted.

## Verification and replay

2,984,570 independent computational checks pass over1,006,128 adjacent point observations, plus unreseeded track histories: source CRC/SHA and raw timestamp associations, nearest matching against complete prior inventories, per-point masks/counts, cumulative identities/no revival, survival summaries, independent joint-run calculation and historical synthetic identity/translation/blank controls. All190 arrays from the original19 RGB-left pairs reproduce EXP-036 exactly, including endpoints, masks and texture. Earlier scientific files remain unchanged. This establishes calculation/source consistency, not ground-truth motion or camera-pose accuracy.

Runtime: Python3.12.3,NumPy2.2.6,OpenCV4.10.0; CPU,OpenCV single thread/seed0. Source acquisition entry point: `Scripts/Setup/td3_lit_interval.py --root <ore-ms> --output <new-run> --config <private-config>`. Analysis,validation,documentation,exact commands and hash registries are preserved under `results/redcourse-lit-interval/staging/`; raw source,plans,adjacent/persistent arrays,context sheets and summaries under `run-001/`. Analysis uses the same two historical LK helper functions, without importing unrelated runtime dependencies.

## Decision

This expansion does not yield a continuous paired RGB–thermal case under current timing limits. Keep the brief4153–4172 precursor as an exploratory RGB candidate, with its74 surviving grid tracks, while acknowledging broader sparse support and pairing limitations. Do not launch a full learned VO/fusion comparison as if these timing and availability questions were solved. If continuing on this source, first define a justified temporal pairing policy with explicit dropped/reused-frame treatment; a public synchronized dataset remains an alternative for the numerical cross-modal characterization. No more upload is needed to reproduce this result.

Recovery target: Box `backups/2026-10-22-exp-037`, containing selected source images/timestamps,plan,raw diagnostics,source/report/notebook,validation and incremental Git bundle. The EXP-036 recovery archive supplies the prior ignored data and190-array controls required by the frozen hash checks, in addition to the inherited code bundles; no model checkpoints or GPU image are prerequisites. Private credentials,packages,range caches and whole source archives are excluded.
