# RGB–Thermal Visual Odometry: Progress and Direction

**Current next work:** [published thermal baseline reproduction intake](thermal-sfm-reproduction-2026-11-12.md), pending author model/processed-ViViD link. Source is pinned; no published result reproduced yet. Keep the [living paper library](references/README.md) updated for every paper used.

**Latest: [native raw thermal refinement](radiometric-native-findings-2026-11-12.md) is complete.** All three recording gates fail: translation mean/RMSE worsen, thermal coverage is unchanged, RGB remains better. The exact configuration is closed without retuning. All1,200frames,7,839parity arrays and1,332independent audit checks pass. The feasibility and hook milestones below are historical, not pending accuracy evidence.

**12 November 2026 · accuracy and literature-informed next work**

**Continuation: [raw thermal tracking feasibility](radiometric-tracking-preflight-2026-11-12.md)**
uses original MAC endpoints and native uint16 images. All 1,755 fixed first-pair
attempts complete, 16 new/64 total tests and a 15,008-check cost audit pass.
This is mechanics only: no motion or correspondence-accuracy gain is established.
The next native experiment requires a sparse integration hook and explicit
covariance/rejection controls; the closed rotation routes remain closed.

**Latest completed accuracy experiment: the [100-frame unweighted test](unweighted-longer-findings-2026-11-12.md)
closes the remaining route.** The candidate is frozen thermal replacement flow
plus unweighted NEC, tested against fresh nativeRGB and thermal controls.
On the added80intervals per recording, day loses to RGB, night loses to both,
and rain slightly improves RGB mean but worsens RMSE and loses coverage.
All three frozen gates fail: **CLOSE_UNWEIGHTED_ROUTE_NO_RETUNING**.
All600native frames and297candidate solves complete;40tests and independent
saved-output/strict-cost audits pass. These are exposed first100-frame prefixes;
there is no full-motion integration, training or generalization claim.

**Prior transfer: the [unchanged day/night assay](bearing-rotation-transfer-findings-2026-11-12.md)
fails its overall transfer gate.** Day primary rotation mean/RMSE improve
34.33%/34.35%against native thermal, but night worsens104.57%/147.53%.
Primary, NEC and native retain19/19eligible intervals on each recording.
The fixed covariance configuration is closed for promotion, with no retuning
or substitution of a secondary winner. All228attempts are retained;39tests and
the independent saved-output audit pass. Both are exposed stereo pilot prefixes,
not the later whole-record cohort or held-out data. The replacement-flow NEC
control is a useful secondary lead, not a promoted method.

**Prior rain result: the [frozen bearing-rotation assay](bearing-rotation-assay-findings-2026-11-12.md)
passes its rain development gate.** On18matched intervals, original-flow PNEC
rotation mean/RMSE are0.05658/0.05952degrees, versus NEC0.07670/0.08009 and
native thermal0.07527/0.09538. That is24.83%/37.60%below native thermal, with
unchanged18/19nominal eligible coverage. All114solves converge under the declared
checks;31tests and an independent cost/mapping/metric audit pass. The exact-eight
initializer correction preceded execution and is recorded. This is rotation-only
development evidence; prior RGB remains better at0.03128/0.04529degrees. No full
motion, fusion or covariance-calibration gain is established.

The objective remains better RGB–thermal VO. We have reproducible baselines and
useful within-camera uncertainty ranking, but no validated adaptive improvement
over RGB. The preceding stereo-mean and temporal-correspondence changes produced
mixed results and missed their predeclared rules. Their outcomes remain closed:

On the same exposed 20-frame rain prefix, scored on 18 common motion intervals:

| Method | Translation mean / RMSE, m | Rotation mean / RMSE, deg |
|---|---:|---:|
| Original thermal | 0.9600 / 1.0490 | 0.07527 / 0.09538 |
| Fitted stereo residual | 0.9308 / 1.0305 | 0.06763 / 0.08510 |
| Pretrained temporal-flow replacement | 0.8967 / 0.9722 | 0.08732 / 0.09927 |
| Original RGB | 0.6700 / 0.8674 | 0.03128 / 0.04529 |

The stereo residual learns, reducing training loss 17.65%, but rain depth AbsRel
worsens 15.47%. The temporal matcher improves translation mean/RMSE 6.59%/7.32%
while worsening rotation 16.01%/4.08%. Thermal coverage is unchanged. RGB remains
better on the mean/RMSE endpoints. These are interval errors, not trajectory ATE.
See [stereo-fit findings](final-residual-fit-findings-2026-11-10.md) and
[temporal-matcher findings](thermal-matcher-pilot-findings-2026-11-12.md).

**Decision:** retain the original models and close those exact configurations
without parameter searches. This does not close thermal correspondence or
uncertainty learning. Reusing the old flow covariance with a new matcher was
an explicit experimental control, not calibrated uncertainty for that matcher.

**Earlier recommendation (now executed and closed below):** test the rotation objective on fixed correspondences
before another model fit. [PNEC](https://arxiv.org/abs/2204.02256) offers a
bearing-space uncertainty model that avoids stereo feature depths;
[DNLS](https://arxiv.org/abs/2305.09527) connects correspondence uncertainty
learning directly to pose optimization. Our proposal is a controlled NEC/PNEC
rotation comparison, with identical points and covariance controls, followed by
existing day/night development prefixes only under a fixed execution protocol.
It is a hypothesis, not a demonstrated explanation or completed experiment.

The [12-source research decision](research-direction-literature-2026-11-12.md)
also considers thermal-specific tracking, multi-spectral self-supervision and
joint depth/motion optimization. Self-TIO/TP-TIO motivate a tracking-refinement
alternative. DROID and covariance propagation are already part of our prior
coverage. Missing dense labels limits direct matching evaluation; published
self-supervised methods mean it does not eliminate method development.

The fixed-covariance comparison and final longer unweighted comparison are now
complete. Both promotion routes are closed under their declared rules. Preserve
the positive small-window findings and negative longer/RGB-controlled findings
together. No additional fitting, threshold/window search or full-motion
integration follows. A future paper-informed direction needs a materially
distinct hypothesis and recording-wise accuracy/coverage evidence. The
equation-level prototype uses the existing numerical runtime; the official
solver is not claimed reproduced.

All current scores use previously exposed development data, supplied references
and nominal frame/time assumptions. No fresh-record generalization, independent
physical accuracy or runtime claim follows. The latest update adds longer
rotation scores and audits, fresh unchanged nativeRGB/thermal controls and
frozen matcher observations. No training, default change or new full-VO
estimator was introduced.
