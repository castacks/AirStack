# Results: RGB–thermal odometry under visual degradation

This chapter characterizes when thermal input improves odometry accuracy
or availability, and measures the cost of retaining that benefit where
a direct comparison is available. The comparisons span sustained darkness,
controlled visible degradation and recorded field conditions. Table entries
identify the reference and scoring population for each experiment.
[Results table](rgb-thermal-thesis-results-2026-11-20.md).

## Thermal lowers error during controlled visual degradation

The fixed-camera comparison establishes the importance of condition. On
IRS RTVI Medium, position RMSE was 0.089 m with visible input and 0.248 m
with thermal. On Fast Dark, the same comparison gave 4.302 m and 0.282 m,
respectively. Both use common Vicon–IMU timestamps and an early unit-scale
alignment. The change in ranking across recordings motivates conditional
sensor use. Illumination and motion both differ across the two recordings,
so their individual contributions cannot be estimated from this comparison.
[Medium](irs-rtvi-bright-medium-metric-2026-11-18.md),
[Fast Dark](irs-rtvi-fast-dark-common-support-2026-11-18.md).

A controlled intervention isolates the effect of dimming the visible stream
within each recording. During ten seconds of programmed dimming, an
image-triggered switch to thermal reduced position RMSE from 0.657 to
0.105 m on Medium and from 1.015 to 0.113 m on Difficult. In the first
two seconds after light returned, the triggered arm improved on dimmed
visible alone, but its errors of 0.183 and 0.254 m exceeded the 0.15 m
accuracy criterion on both recordings. The corrected sensor-time gaps
stayed below 51 ms. The experiment demonstrates an accuracy benefit during
dimming and shows that regular output can coexist with early recovery error.
[Triggered handoff](irs-rtvi-causal-dimming-2026-11-18.md).

## Field performance depends on the estimator and the comparison

In recorded SmokeRoom obscuration, learned RGB-first pairwise matching with
thermal fallback filled 15 missing RGB proposals on 236 moving anchors.
Eleven additions were within 5° of the supplied relative rotation, raising
overall success from 76.7% to 81.4%. Cached matching cost was projected
to rise 6.6% on the moving subset and 42.7% across the entire recording.
For the FlashLight transfer, learned RGB produced estimates on all 192
moving anchors, leaving no gaps for the same fallback. These are
relative-motion results against the published references.
[SmokeRoom pairwise comparison](smoke-rgb-first-findings-2026-11-16.md),
[FlashLight transfer](flashlight-transfer-findings-2026-11-16.md).

The stateful SmokeRoom comparison reached a different outcome. The
scheduled thermal switch preserved full scoring coverage and corrected
gaps near 50 ms. During obscuration its position RMSE was 4.881 m,
compared with 1.614 m for continuous RGB and 1.033 m for an RGB track
reset with the same schedule. In the later window the thermal arm
improved on continuous RGB, while the matched RGB reset retained the
lowest error. The comparison uses the supplied fused trajectory on an
exposed recording. Its result illustrates why pairwise availability
and publication cadence must be evaluated alongside stateful accuracy.
[SmokeRoom stateful comparison](smokeroom-vins-scheduled-handoff-2026-11-19.md).

## Lower thermal cadence saves updates and lengthens corrected gaps

After full-rate initialization, retaining every other thermal frame
reduced post-prefix camera inputs from 1,231 to 616 on Fast Dark and
from 1,241 to 621 on Medium. On the common retained-time cohort in each
recording, position RMSE changed from 0.282 to 0.171 m and from 0.248 to
0.168 m, respectively. The largest corrected gaps increased from
66.68 to 116.68 ms and from 50.24 to 100.23 ms. Single serial loader runs
also took less wall-clock time at the reduced cadence, including setup
and I/O. Immediate 10 Hz, a five-second full-rate prefix and 2 × 2
spatial pooling each failed their frozen qualification on Fast Dark.
The supported configuration is therefore specific to the tested
initialization and temporal sampling policy.
[Fast Dark cadence](irs-thermal-input-budget-intake-2026-11-19.md),
[Medium cadence](irs-thermal-cadence-medium-transfer-2026-11-19.md),
[startup](irs-thermal-cadence-startup-2026-11-19.md),
[short prefix](irs-thermal-cadence-five-second-startup-2026-11-20.md),
[spatial pooling](irs-thermal-spatial-pooling-fast-dark-2026-11-20.md).

## Thermal update quality depends on estimator history

The pre-update thermal photometric residual ranked incremental motion
accuracy better than accepted track count during dimming on both IRS
recordings. At 90% coverage, after matching broad reference-motion
categories, displacement RMSE fell from 5.95 to 5.77 mm on Medium and
from 6.56 to 5.57 mm on Difficult. The matched comparison used offline
rankings of already computed thermal poses. Early recovery on Difficult
showed a tradeoff between rotation and displacement. A fixed-window
ranking establishes association with output quality. An admission
decision also changes the filter's later input history.
[Thermal update ranking](irs-thermal-admission-innovation-2026-11-22.md).

To measure that effect, a replay omitted 20 images chosen from a prior
full-rate thermal trace. A fixed-thinning control omitted the same
number. On Difficult, position RMSE over the common dimming reference
epochs was 0.114 m with residual-based withholding, 0.184 m with fixed
thinning and 0.186 m with full-rate thermal. On Medium the same
comparison was 0.106, 0.105 and 0.102 m. The omissions roughly doubled
the maximum corrected output gap from 50 to 100 ms. Difficult's early
recovery improved from 0.261 to 0.240 m relative to full rate and still
exceeded the 0.15 m target. The omits were preselected from a prior trace
with within-block lookahead. No online controller was evaluated.
[Thermal withholding replay](irs-thermal-innovation-withholding-2026-11-22.md).

Crossing the omission lists by half isolated a carryover effect on
Difficult. With identical fixed thinning during the second half, using
residual-based withholding in the first half lowered second-half RMSE
from 0.243 to 0.131 m, even though first-half RMSE increased from 0.070
to 0.087 m. In a separate twelve-replay test, five of six high-residual
single-frame omissions improved two-second position error. Only three
beat the nearby low-residual omission, including one of three on Difficult.
Difficult's largest single-frame benefit came from a low-residual
omission. These results motivate evaluating decisions on downstream
trajectory error. They leave a reliable online admission rule open.
[Half-window and single-update replays](irs-thermal-innovation-withholding-2026-11-22.md#difficult-half-window-attribution).

## A qualified reference still requires a valid estimator baseline

On CAVERS `loc_handheld_1`, median adjacent-frame RGB track retention
fell from 82.3% to 7.4% during the selected one-second interval and
returned to 69.5% in the next two seconds. The event was selected
after the image sweep and represents moving headlight illumination
inside a cave. OptiTrack and IMU bracketed every camera arrival in
the five-second context. All matched replay arms published at every
expected camera epoch, while the sensor-only RGB+IMU estimate
failed the pre-event translation control.

With the constructor gravity, the first-sample and fixed accelerometer
mean initializers gave 0.316 m and 1.283 m pre-event translation RMSE.
A single OptiTrack-derived tilt at startup reduced that error to
0.040 m. None of 6,011 candidate IMU windows met the frozen
stationarity rule. These results document an initialization
dependency and close this recording for the tested sensor-only
recovery protocol. The image-space tracking loss and partial return
remain measurable on the released recording.
[CAVERS intake and controls](cavers-headlight-transition-intake-2026-11-22.md).

## Conclusion and limits

Thermal input improved accuracy in the tested sustained-darkness and
controlled-dimming comparisons and added usable relative-rotation proposals
in recorded smoke. The stateful smoke comparison and controlled recovery
windows establish limits on those gains. Thermal availability alone does
not establish an accuracy benefit, and nominal sensor-time continuity
does not establish receiver-side delivery. Independent metric validation
during natural field recovery remains open. PARK contributes image-space
field evidence. The route crosses a shaded passage into daylight, but
its thermal calibration conflict and reference-to-body frame binding
prevent metric scoring. M2DGR Door and Brno offer complementary public
reference material. The Door bags were inaccessible during intake and
the Leica exports contain position only. Brno has paired camera
timestamps and GNSS positions, with fix quality, full attitude and the
location of any degradation interval still unresolved. Establish a
qualified RGB+IMU control on the clear lead-in before comparing
candidate sensor policies through a new transition.
[PARK geometry](diter-plusplus-park-geometry-2026-11-19.md),
[natural transition intake](natural-transition-reference-intake-2026-11-22.md),
[baseline and recording gates](natural-recovery-baseline-gate-2026-11-23.md),
[paced delivery](irs-rtvi-paced-imu-bridge-delivery-2026-11-18.md).
