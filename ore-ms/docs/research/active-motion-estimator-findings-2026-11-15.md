# Matched motion histories: small, mixed pose effects at a fixed deadline

Started14 November2026UTC; completed15 November2026UTC. Branch
`ore-ms-orb-development`, baseline `a79658f6341194b0954852226f200313ccdf5408`
plus source-pinned working changes. All18native frontend batches and36pose fits
complete; all36fits converge. No training, online action selection or robot run.

## Result and contribution

Changing the speed history along the same path changes the memory images and
estimated pose, while the sharp images and estimates repeat exactly. Effects
across the three prescribed actions are small and texture-dependent. Native
reselection translation-error ranges are0.015–0.079mm; fixed-query ranges are
0.002–0.042mm. The common sharp baseline errors are1.668–5.344mm. Less recent
motion does not consistently improve both translation and rotation.

This supplies a controlled connection to MACThermal's active-perception opening:
action timing can affect acquired evidence even at equal progress and deadline.
It does not establish useful scheduling-policy headroom, a new planner or a
physical sensor-transfer benefit. No statistical/practical advancement threshold
was prespecified; the small absolute changes and mixed endpoints are reported
rather than assigned a retrospective pass/fail threshold.

## Fixed controls and image qualification

Use the [qualified motion contract](active-perception-motion-decision-2026-11-14.md):
three monotone C2 schedules beta−2/0/+2 along the same4cm straight path over0.2s,
fixed orientation, zero endpoint velocity/acceleration, and identical endpoint
poses. The reversed nonzero schedules have equal full-horizon squared-speed
integrals and equal peak speed/acceleration. For a fronto-parallel4m plane and
pure lateral translation, their whole-horizon squared image-velocity integrals
also tie. The beta0 control is retained separately.

Three exposed thermal textures are prepared once by the existing recipe and
used as static radiance proxies. This is not a real radiance-to-camera-to-CLAHE
pipeline. Borrowed geometry is640×256, focal387.78695px, baseline0.303899514m;
no resize/distortion or real-camera calibration is added.

Sharp and exponential-memory arms use source time0 and target time0.2s, with
one virtual response tau8ms. The camera is stationary before0. Memory integrates
ages0..0.2s and includes the exact stationary prehistory contribution with weight
exp(−25); there is no eight-tau tail truncation. No rolling readout or trigger
delay is applied. This full-history treatment differs explicitly from the older
four-arm timing adapter. It does not change the older result or label the virtual
profile as measured MS²/TD3 physics.

All nine cases produce72individual views. Pre-inference64/128Gauss convergence
passes on11,045,952supported intensity values: maximum0.00136950greycode,
RMS0.0000131214code, versus fixed0.5/0.05bounds. Choose128nodes as specified;
no higher-grid or profile rescue. Independent bilinear interpolation with
adaptive time integrals split at spatial interpolation knots checks1,728pixels;
maximum discrepancy0.000075439code. The complete image audit passes2,309checks,
including pose geometry, quantization, common support and exact sharp/source nulls.

Opposite memory schedules change only424/970/1067of306,832supported target stereo
values for day/night/rain respectively; every change is one greycode. Supported
RMS image changes are0.03717/0.05623/0.05897code. Those are explicit small input
changes, not physical noise estimates. The scalar recent-box and exponential
history controls in the motion contract both distinguish the schedules. No
recent-box neural/image arm was executed here; neither scalar control by itself
establishes which motion should give a better learned pose.

## Estimator and query controls

Use unchanged native MACT architecture/checkpoint, native selection/covariance
call bodies and the existing identity-initialized optical frozen-weight IRLS
pose harness. Strict state loading requires no migration; checkpoint state
stays unchanged. This is not full historical MAC-VO, VIO, PAMPC, FIF or Wu2022
execution. Each batch predicts source stereo, target stereo and temporal-left
flow/covariance:18batches/54pair predictions total.

One raw renderer support mask intersects all schedules/arms/stereo/time. It is
eroded31×31and intersected with the native32pixel crop. Source queries and
predicted target coordinates must be in that mask, with minimum10points. This
oracle synthetic-boundary eligibility is a research control, not a deployable
frontend policy. Images are unmasked.

Primary queries are regenerated independently per image arm. The secondary
control starts with the sharp accepted queries and intersects their admission
across all six schedule/arm packets per texture. It uses exactly the same source
IDs for all six fits; there is no replenishment. All sharp accepted queries
survive this intersection:146day,129night,125rain. Primary memory counts are
137/139/139day,133/132/134night and123/122/122rain in beta order.

All18dense outputs/36fit receipts lock before the scorer reads true pose values.
Sharp dense outputs, fitted pose and inverse-information outputs repeat exactly
across schedules in each query mode. The18capture null checks pass before
scoring. Sharp pose errors remain nonzero and are retained.

## All memory-action outcomes

Translation error, millimetres. Each shared sharp value applies to all three
schedules and both query modes.

| Texture | Shared sharp | Native beta−2 | Native beta0 | Native beta+2 | Fixed IDs beta−2 | Fixed IDs beta0 | Fixed IDs beta+2 |
|---|---:|---:|---:|---:|---:|---:|---:|
| day |5.343584|6.124127|6.194663|6.115585|5.656037|5.627735|5.614306|
| night |2.426730|2.070965|2.041809|2.053717|2.343407|2.344349|2.345669|
| rain |1.668346|1.468591|1.478601|1.483602|1.680599|1.695374|1.705920|

Rotation error, degrees:

| Texture | Shared sharp | Native beta−2 | Native beta0 | Native beta+2 | Fixed IDs beta−2 | Fixed IDs beta0 | Fixed IDs beta+2 |
|---|---:|---:|---:|---:|---:|---:|---:|
| day |0.07414114|0.08517243|0.08629135|0.08514774|0.07920001|0.07875235|0.07854086|
| night |0.03387984|0.03204182|0.03145396|0.03117483|0.03463358|0.03448318|0.03443446|
| rain |0.00690746|0.00844299|0.00829880|0.00814078|0.00929355|0.00954253|0.00965837|

![All memory-action errors and shared sharp controls](../../results/active-motion-assessment/scoring-001/pose-errors.png)

Preserve positive cases. Native night beta0 memory improves translation
2.426730→2.041809mm and rotation0.03387984→0.03145396deg compared with sharp.
Day memory worsens both endpoints for every schedule. Rain's native memory
translation improves but rotation worsens; under fixed IDs, both errors worsen
slightly relative to sharp. Thus a single aggregate “memory helps/hurts” label
would hide the controls and endpoint tradeoffs.

Within the memory arm, moving from beta−2 to+2 reduces scalar recent-motion lag.
With fixed IDs it improves both errors for day, increases translation while
reducing rotation for night, and worsens both for rain. Native reselection changes
some of these comparisons. This shows the action ranking is not supplied by
that scalar surrogate on these cases. It does not prove existing full-horizon
blur-aware planners make the same decisions or that a new scheduling head is
needed. Endpoint means, inferred depths and covariance still vary jointly.

Median matching-covariance readouts and inverse-information proxies for every
fit are retained in the CSV. They are separate targets; there is no calibrated
noise ensemble, integrated-pixel flow truth or coverage claim. No weight change,
covariance correction or solver choice is selected from these results.

## Verification, limits and stopping decision

Ten inherited pre-inference mask/packet/analytic-pose fixtures pass. The separate
estimator audit passes955checks: source/image/output locks, input batching,
independently reconstructed native queries and geometric support, packet means/
covariances, six-way common IDs, fit stationarity/information inverse, true-pose
errors and exact sharp nulls. All36attempts are scored; no method source, input,
solver, seed or tolerance amendment is made during rendering/inference/auditing.

Only three exposed textures, one plane/motion axis/virtual response and one
learned checkpoint are tested. Kinematic bounds do not establish robot dynamics
or collision feasibility. All schedules are prescribed; no causal policy,
continuous trajectory estimator, navigation task or deployment is tested.
Known scene/model previews and exact poses are research tools, not an online
planner's available measurements. Small action spreads are descriptive, not
independent-trial significance or real-world accuracy guarantees.

Close this exact profile without increasing speed, response time, schedule
asymmetry, query budget or solver complexity. The result adds controlled
acquisition-timing evidence to T5 while leaving the broad active-perception,
representation, supervision and persistent-estimation directions open. No new
planning algorithm or useful general scheduling rule is established.

## Artifacts

Image freeze:`experiments/active-motion-image-freeze-2026-11-14.json`.
Estimator design/freeze:`experiments/active-motion-estimator-design-2026-11-14.json`
and `experiments/active-motion-estimator-freeze-2026-11-15.json`.
Source:`tools/active_motion_assessment.py`, `tools/audit_active_motion_assessment.py`,
`tools/summarize_active_motion.py`. Raw image/dense/packet/fit outputs, locks,
score CSV/figure, image contrasts, logs and audit receipts are under
`results/active-motion-assessment/`. Local notebook content is excluded from backup.
