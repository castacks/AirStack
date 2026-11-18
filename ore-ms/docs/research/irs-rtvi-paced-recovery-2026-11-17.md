# IRS RTVI causal camera fallback at recorded cadence

The [native-state handoff](irs-rtvi-single-slot-handoff-2026-11-17.md) met
its scheduled replay criteria. This experiment runs that filter over the
first 46 s of easy and fast dark at 1× recorded cadence. The replay source
withholds visual messages from 40 through 42 s. The selector receives the
remaining messages and their arrival times. It selects thermal when a
thermal frame arrives at least 125 ms after the last visual arrival and
returns on the third consecutive restored visual frame. The schedule is
available to the fault injector alone. Uninterrupted visual and visual
blackout runs serve as controls. The same compiled binary, estimator
settings, and IMU stream are used across policies.

Monotonic wall-clock timestamps mark delivery, dispatch to the estimator,
and publication. The largest wall-clock gap spans intervals that intersect
the outage through 0.5 s after visual input returns. Sensor-stamp gaps are
retained separately. The scorer matches emitted poses to selected images
and checks unchanged no-switch outputs against the native visual runs
(917 shared rows in easy and 886 in fast dark). The published Vicon–IMU
position errors use one yaw and translation fit in the 10 seconds before
the outage. Scale remains fixed.

## Results

| Recording and arm | Largest wall publication gap | Position RMSE during outage | Position RMSE over 2 s after visual input returns |
|---|---:|---:|---:|
| Easy, uninterrupted visual | 0.053 s | 0.045 m | 0.041 m |
| Easy, visual blackout | 1.998 s | No odometry | 0.059 m |
| Easy, causal fallback | 0.099 s | 0.062 m | 0.038 m |
| Fast dark, uninterrupted visual | 0.052 s | 0.344 m | 0.284 m |
| Fast dark, visual blackout | 1.996 s | No odometry | 0.451 m |
| Fast dark, causal fallback | 0.100 s | 0.236 m | 0.277 m |

Both causal runs publish 40 thermal poses. Of those and nearby visual
outputs, 38 poses associate inside the outage window and 39 associate
inside the return window in each run. First thermal pose publication
takes 0.250 s on easy and 0.252 s on fast dark from the last visual input.
The selector dispatches the first thermal image after approximately 0.152 s.
Buffered publication of earlier poses makes the longest publication gap
shorter than that first thermal pose delay. Visual pose publication returns
0.203 s and 0.201 s after the first restored visual frame. P95 dispatch to
publication is 0.106 s on easy and 0.105 s on fast dark, including filter
buffering.

Easy meets both frozen bounds of 0.20 s publication gap and 0.15 m nominal
position RMSE. Fast dark meets the publication bound and misses the
position bound. In the 10 s before its outage, all three arms already have
0.189 m position RMSE and one median tracked feature, compared with ten
tracked features in easy. Every pre-outage visual interarrival
interval is below 0.067 s in both recordings, so the input timeout does
not detect the deteriorated fast-dark visual estimate.

A retrospective check of the existing native thermal run on fast dark
gives 0.047 m RMSE during the outage interval and 0.069 m afterward under
the same pre-window fitting convention. It starts on thermal and uses
the authors' thermal preset. This provides a fixed-thermal comparison
under the adverse condition. It cannot isolate the effects of inherited
visual error, cold feature initialization and shared visual filter settings.

The timed result covers a laboratory replay with controlled frame loss.
Hardware, ROS 2 transport timing and image degradation with continuously
arriving frames still need their own qualification. The fast-dark outcome
closes the input-timeout policy as a general pose-recovery mechanism at the
tested bound. A later quality-sensitive trigger needs to be evaluated
before it is allowed to switch a running estimator.

## Retrospective tracking-health check

A separate audit checked whether the median native visual tracked-feature
count over the preceding 250 ms ranks error in the next one-second
displacement. It uses the three existing visual trajectories, a yaw fit on
the initial 10 s, nonoverlapping reference-grid intervals, the existing
0.15 m error threshold, and no threshold selection for feature count.
The published feature status can include freshly initialized features.
Feature timestamps were used as a proxy for observable history.

| Recording | High-error intervals / eligible intervals | AUROC for low feature count |
|---|---:|---:|
| Easy | 1 / 78 | 0.55 |
| Dark | 118 / 119 | 0.52 |
| Fast dark | 31 / 68 | 0.64 |

Two fast-dark grid intervals lacked eligible associations. Easy and dark
each have only one example in the minority class, so their AUROCs are
weak estimates. The fast-dark result shows limited separation. These checks
leave the native feature count unqualified as the sole switching trigger.
The frozen audit and interval ledger are under
`/root/ore-ms-artifacts/irs-rtvi-intake/stateful/tracking-health/`.

The first offline score invocation used a NumPy option missing in the
legacy runtime. A later audit corrected the largest wall gap to be scoped
using wall rather than pose time. The easy decision is the same after
correction. Original logs, preliminary scorers, corrected scoring and
checksums are under `/root/ore-ms-artifacts/irs-rtvi-intake/stateful/paced/`
and `/root/ore-ms-artifacts/irs-rtvi-intake/stateful/paced-fast-dark/`.
No new bags, derived bag copies or Box writes were required.
