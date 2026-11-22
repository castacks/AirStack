# IRS thermal frame withholding by pre-update innovation

Withholding the thermal frames ranked worst by innovation improved position
RMSE on Difficult during dimming and early recovery. On Medium, dimming
error increased slightly versus both controls. The experiment fails the
two-recording advancement rule.

## What this adds to prior experiments

The [visible-update ablation](irs-rtvi-flagged-visible-update-ablation-2026-11-18.md)
withheld three brightness-flagged visible updates at handoff. The
[thermal cadence experiment](irs-thermal-cadence-medium-transfer-2026-11-19.md)
regularly halved thermal rate after initialization. The
[thermal innovation study](irs-thermal-admission-innovation-2026-11-22.md)
ranked existing outputs without changing filter inputs. Here the
recorded thermal innovation ranking changes which images the estimator
actually receives, with a fixed-thinning control using the same count.
The previous work and the literature behind the innovation score are
linked above.

## Protocol and checks

Use native continuously running thermal and IMU on IRS Medium and
Difficult. Freeze two lists of 20 omitted images per recording before
modified replay. Within each block of ten of the 200 thermal images in
the established dimming window, the innovation list skips the largest
prior innovation among the eight interior images. Fixed thinning skips
the sixth image in every block. All other thermal inputs and every IMU
message are retained, with no consecutive omissions. The residual list
uses whole-block lookahead over a previous full-rate run. It stays
fixed as the modified replay's state changes. This tests the effect of
retrospectively selecting inputs to withhold.

Independent readback verified timing, delivery order and serialized
payload equality for every retained message. Both variants publish
exactly 20 fewer poses, at the declared removed timestamps, and
reproduce every native pre-intervention pose byte for byte. Use one
native pre-dimming yaw and translation fit for all arms per recording
on 199 Medium and 200 Difficult reference samples. Position scores are
on common reference epochs with a 100 ms interpolation cap.
Advancement requires lower dimming RMSE than both controls and no early
recovery error increase versus full rate in both recordings, with an
output gap at most 150 ms.

## Results

| Recording | Arm | Dimming RMSE | Early recovery | Late recovery | Maximum output gap |
| --- | --- | ---: | ---: | ---: | ---: |
| Medium | Full thermal | 0.102 m | 0.119 m | 0.215 m | 50.11 ms |
| Medium | Fixed thinning | 0.105 m | 0.131 m | 0.122 m | 100.01 ms |
| Medium | Innovation withholding | 0.106 m | 0.117 m | 0.146 m | 100.11 ms |
| Difficult | Full thermal | 0.186 m | 0.261 m | 0.220 m | 50.22 ms |
| Difficult | Fixed thinning | 0.184 m | 0.342 m | 0.341 m | 100.23 ms |
| Difficult | Innovation withholding | 0.114 m | 0.240 m | 0.235 m | 100.04 ms |

Innovation withholding improves Difficult dimming RMSE by 38% against
fixed thinning and by 39% against full rate on the common reference grid.
Its early recovery error remains above the historical 0.15 m recovery
target and its late recovery is worse than full rate. Medium's late
recovery is better than full rate and worse than fixed thinning.

Full, fixed and innovation dimming reference coverage is 200, 188 and
190 timestamps on Medium and 200, 182 and 184 on Difficult. The common
cohorts are 180 Medium and 168 Difficult. All arms have full support on
the 40 early recovery epochs and on the late recovery epochs. The
dimming RMSEs describe the common supported epochs, so error inside the
excluded gaps is unmeasured. Neither replay changes the visible camera
or tests an online handoff. Close this fixed withholding schedule under
the declared two-recording gate.

All 18 RMSE entries reproduce independently from the shared-grid error
ledger. Protocol, raw-message audit, omit lists, bags, logs, captures,
and scores are under
`/root/ore-ms-artifacts/irs-dual-preflight/admission-innovation/withholding/`.

## Difficult half-window attribution

The earlier [recovery error-origin diagnostic](irs-rtvi-early-recovery-error-origin-2026-11-19.md)
decomposes the error on saved trajectories after visible-to-thermal handoff.
This follow-up uses two additional thermal-only executions to isolate the
effect of omission history. Split Difficult's frozen intervention at
1614701825 s. Cross the first and second halves of the existing fixed and
innovation omit lists. Each hybrid still omits 20 images and retains all IMU
messages. The two hybrids are fully determined by the prior lists and midpoint.

Raw-message readback verifies all 37,384 retained messages per hybrid.
Each hybrid reproduces all 888 poses before the midpoint byte for byte
against the parent sharing its first half. Score all five arms on the
same reference support, retaining 78 and 90 dimming epochs in the first
and second halves and all 40 and 159 early and late recovery epochs.
Use the unchanged native alignment. The maximum gap in each sparse arm
is between 100.01 and 100.23 ms. All 20 scores reproduce from the error ledger.

| First-half schedule | Second-half schedule | First-half RMSE | Second-half RMSE | Early recovery | Late recovery |
| --- | --- | ---: | ---: | ---: | ---: |
| Full rate | Full rate | 0.106 m | 0.234 m | 0.261 m | 0.220 m |
| Fixed | Fixed | 0.070 m | 0.243 m | 0.342 m | 0.341 m |
| Innovation | Fixed | 0.087 m | 0.131 m | 0.176 m | 0.168 m |
| Fixed | Innovation | 0.070 m | 0.210 m | 0.232 m | 0.176 m |
| Innovation | Innovation | 0.087 m | 0.133 m | 0.240 m | 0.235 m |

Replacing the first-half fixed list with innovation increases error
during that half. During the second half it lowers RMSE by 0.111 m
when the second-half list is fixed and by 0.077 m when the second-half
list is innovation. Replacing the second-half list alone gives a
0.033 m reduction after fixed in the first half, and about a 0.002 m
increase after innovation in the first half. The dimming advantage
therefore depends primarily on which omissions occurred earlier and
was carried forward by estimator history under identical later inputs.
The different early and late recovery outcomes show that the schedule
effects depend on the estimator's starting state. This attribution
does not identify individual causal frames or qualify a hybrid policy.

The frozen protocol, lists, fixtures, logs, and checks are under
`/root/ore-ms-artifacts/irs-dual-preflight/admission-innovation/withholding-halves/`.

## Single-update effects over the next two seconds

Test whether a high pre-update residual predicts when omitting an
individual thermal measurement improves later position. This follows
the half-window result by changing one input per replay rather than a
sequence of omissions. Use the same photometric innovation from the
[ROVIO-based ranking study](irs-thermal-admission-innovation-2026-11-22.md).

Before replay, freeze three one-second selection bins starting 0, 2 and
4 s into dimming on each recording. Choose the largest-residual frame
in each bin and the nearest frame in that bin's lowest residual third.
Use chronological ties. The six pairs have frame separations of 0.10
to 0.30 s. Matching is by recording and local time. Motion and immediate
tracking state can still differ between the two candidate frames.

Run twelve thermal-only replays. Each omits the specified image and
retains every other thermal and IMU message. Independent readback
checks 37,540 retained Medium messages and 37,403 Difficult messages per
arm, including serialized content and delivery order. Each replay
matches every native output before its omission, and that frame's
timestamp is the only missing output. Compare each skip arm to the
native keep arm on identical epochs in the fixed two-second horizon
after that intervention. Use the same native alignment and 100 ms
reference scoring cap as the previous experiment. High and low frames
have two-second windows starting at their respective candidate times.

| Recording | Selection bin start | High frame time | Low frame time | High skip minus keep RMSE | Low skip minus keep RMSE |
| --- | ---: | ---: | ---: | ---: | ---: |
| Medium | 0 s | 0.128 s | 0.428 s | −2.05 mm | −1.87 mm |
| Medium | 2 s | 2.678 s | 2.478 s | −0.51 mm | +0.01 mm |
| Medium | 4 s | 4.078 s | 4.178 s | +0.33 mm | −7.39 mm |
| Difficult | 0 s | 0.775 s | 0.925 s | −2.21 mm | +5.74 mm |
| Difficult | 2 s | 2.425 s | 2.325 s | −9.15 mm | −12.75 mm |
| Difficult | 4 s | 4.175 s | 4.075 s | −1.28 mm | −2.22 mm |

Negative means withholding the frame lowered two-second position RMSE.
Frame times are relative to the dimming boundary. All horizons contain
40 reference epochs. Both Medium 4 s variants retain 39 common epochs
and all other variants retain 40. Maximum gaps span 99.98–100.01 ms.
The arithmetic audit verifies the 478 paired error samples and all six
high-versus-low decisions.

Five of six high-residual omissions reduce error, but only three beat
their low-residual controls. The frozen requirement is at least two
such wins on each recording. Medium yields two and Difficult one.
Difficult's largest improvement is a low-residual omission. Close the
tested residual as a standalone selector of beneficial omissions on
this horizon. These effects start from full-rate estimator history and
do not determine which omissions mattered in the multi-omission replay.

Protocol, selected stamps, fixtures, readback, captures and audited scores
are under
`/root/ore-ms-artifacts/irs-dual-preflight/admission-innovation/single-update/`.

## The same thermal update under two estimator histories

The previous comparison paired nearby high- and low-residual frames.
This test holds the candidate measurement fixed and changes the
estimator's input history. On Medium and Difficult, replay the first
ten omissions from each existing schedule, fixed and innovation-ranked.
Keep all images from the midpoint onward, with one additional omitted
image in each skip branch. Select the largest full-rate innovation in
each of [5, 6) and [7, 8) s before running any branch. Both histories
use the same nominated image. This gives four candidate comparisons
and twelve unique replays, with the keep traces reused.

Each keep/skip pair matches every pose before the candidate byte for
byte. The candidate timestamp is its only output difference in coverage.
All retained thermal and IMU messages pass byte and delivery-order
readback. Score all four arms per candidate on a common two-second
Vicon–IMU grid, with the original pre-dimming alignment and 100 ms
interpolation cap. A 1 mm threshold was declared to distinguish small
sign changes from a reversal with a larger effect in both histories.

| Recording and candidate time | Common epochs | Fixed-history keep → skip RMSE | Innovation-history keep → skip RMSE | Effect change across histories |
| --- | ---: | ---: | ---: | ---: |
| Medium 5.128 s | 40 | 86.16 → 86.51 mm | 104.81 → 104.71 mm | −0.45 mm |
| Medium 7.928 s | 39 | 148.70 → 148.72 mm | 145.52 → 145.53 mm | −0.01 mm |
| Difficult 5.775 s | 39 | 209.52 → 195.88 mm | 157.71 → 141.39 mm | −2.68 mm |
| Difficult 7.925 s | 40 | 282.26 → 272.30 mm | 183.72 → 178.84 mm | +5.08 mm |

The last column is the innovation-history skip-minus-keep difference
minus the corresponding fixed-history difference. The same frame
changes sign once, on Medium at 5.128 s. Withholding worsens RMSE by
0.36 mm under fixed history and improves it by 0.10 mm under
innovation history. Neither side exceeds 1 mm. Both Difficult
omissions help in both histories, with benefits differing by 2.68
and 5.08 mm. The tested histories change the size of the omission
effect. These four candidates do not establish a substantial sign
reversal.

Logging-only replays reproduce every keep trajectory exactly and
show that the same image has different pre-update residuals and
track counts after different histories. On Difficult at 7.925 s,
the residual falls from 23.80 to 19.71 in native score units and
accepted track count falls from 10 to 7. Withholding helps in both
cases, by 9.96 and 4.88 mm. The pose and tracking telemetry indicate
different histories without isolating whether landmarks, inertial
bias or another internal state drives the changed effect.

The arithmetic audit reproduces all 16 RMSE entries and four
interaction comparisons. Protocol, raw-message checks, logs, scores
and telemetry are under
`/root/ore-ms-artifacts/irs-dual-preflight/admission-innovation/state-history/`.

### Where the history effect enters position error

Following the earlier [recovery error-origin diagnostic](irs-rtvi-early-recovery-error-origin-2026-11-19.md),
decompose the paired skip-minus-keep mean squared position error.
Use the latest common reference epoch at or before the last camera
output before the candidate. It is about 97.5 ms before each candidate
and can be interpolated solely from earlier outputs in every arm.
Within either history, keep and skip have identical position error
at that anchor. Retain the original four-arm scoring cohorts and
alignment. The error change is the change in squared displacement
error from the anchor plus twice the inner product of the anchor
error and the change in displacement error. The anchor's squared
norm cancels in the keep/skip subtraction.

| Recording and candidate | History | Total skip minus keep MSE | From displacement error | From interaction with existing position error |
| --- | --- | ---: | ---: | ---: |
| Medium 5.128 s | Fixed | +61.5 | −29.7 | +91.1 |
| Medium 5.128 s | Innovation | −20.3 | −25.9 | +5.6 |
| Medium 7.928 s | Fixed | +6.7 | −414.5 | +421.2 |
| Medium 7.928 s | Innovation | +3.3 | −1,255.5 | +1,258.8 |
| Difficult 5.775 s | Fixed | −5,530.8 | −2,090.7 | −3,440.1 |
| Difficult 5.775 s | Innovation | −4,881.8 | −554.5 | −4,327.2 |
| Difficult 7.925 s | Fixed | −5,522.9 | −377.4 | −5,145.5 |
| Difficult 7.925 s | Innovation | −1,769.3 | −884.4 | −884.9 |

Values are in mm². Negative changes favor skipping. On Medium at
5.128 s, skipping improves displacement error from the anchor under
both histories. The small reversal in absolute pose error comes from
how the changed displacement aligns with the position error already
present. On Difficult at 7.925 s, the cross-history difference in the
omission's MSE effect is +3,753.5 mm². The interaction term contributes
+4,260.5 mm² while the displacement term offsets −507.0 mm². At
Difficult 5.775 s, the displacement and interaction terms both
contribute and partially cancel. These contributions are conditional
on the anchor and the fixed two-second scoring horizon.

The signed-vector calculation verifies every previously saved
squared error and reconstructs all eight branch differences. No
source frames or estimator states were modified. The decomposition
does not determine which internal filter variable caused the
change. It shows that judging a measurement by absolute trajectory
error can depend strongly on the direction of an existing offset,
even when the incremental displacement comparison agrees across
histories. Protocol, scorer and verified results are under
`/root/ore-ms-artifacts/irs-dual-preflight/admission-innovation/state-history/error-attribution/`.

### Local displacement as the single-frame scoring target

Revisit the twelve original high- and low-residual omissions using
the pre-frame anchor from the history analysis. This tests whether
the residual is a stronger selector when the scoring target is
change in displacement error rather than absolute position error.
Use the same candidates, alignment, two-second intervals and all
478 common reference epochs. Anchor each keep/skip pair at its
latest supported reference epoch before the candidate, requiring
the anchor interpolation to use only pre-candidate camera outputs.
The pose offset common to the keep and skip branches is removed
from each motion error vector. Reuse the original pair decision
and the two-wins-per-recording requirement.

| Recording and selection bin | High omission, absolute | High omission, local | Low omission, absolute | Low omission, local |
| --- | ---: | ---: | ---: | ---: |
| Medium 0 s | −2.05 | +1.84 | −1.87 | −3.11 |
| Medium 2 s | −0.51 | −1.52 | +0.01 | +0.03 |
| Medium 4 s | +0.33 | +3.98 | −7.39 | +2.74 |
| Difficult 0 s | −2.21 | −2.64 | +5.74 | +4.60 |
| Difficult 2 s | −9.15 | −17.54 | −12.75 | −2.43 |
| Difficult 4 s | −1.28 | −0.55 | −2.22 | −1.50 |

All values are skip-minus-keep RMSE in mm. The pair wins move from
two on Medium and one on Difficult under absolute scoring to one
on Medium and two on Difficult under local scoring. Difficult's
2 s pair reverses in favor of the high-residual omission. Medium's
0 s pair reverses in favor of the low-residual omission. The same
three-of-six total still fails the two-recording gate. The
standalone residual does not become a reliable single-update
selector by changing to a displacement endpoint. This was a
retrospective change of scoring target on the same replay outputs.

The independent arithmetic check verifies the reference keys,
24 local RMSE entries and six pair decisions. The calculation
also reproduces all saved absolute errors. Protocol, per-epoch
errors and scores are under
`/root/ore-ms-artifacts/irs-dual-preflight/admission-innovation/single-update/anchored/`.
