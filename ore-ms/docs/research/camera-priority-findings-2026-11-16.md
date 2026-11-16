# Fixed thermal priority helps OutdoorNight, but cannot be a universal rule

16 November 2026, America/New_York. Completed the
[fixed-priority/headroom protocol](camera-priority-protocol-2026-11-16.md).
This is an exposed-development control after the earlier gyro outcomes were seen,
not blind transfer, a fitted quality model or a new method. No new inference,
training, threshold/window/axis/clock/bias change or model adoption.

## OutdoorNight: actual locked-output replay

Thermal-first requests thermal when supported and keeps an `ok` proposal;
otherwise it requests RGB when supported. This uses the same pure preference
helper with camera names exchanged. All 971 anchors and both frontends remain;
1,942 new status-only choices lock before error joins. Both orders have exactly
the same output-status union. The candidate changes which accepted camera is kept,
not the available proposal population or estimator behavior.

On the same 242 both-camera gyro-supported, >=2-degree moving anchors:

| Learned policy | Outputs | Within 1° of gyro | Within 5° | Calls | Projected pair seconds |
|---|---:|---:|---:|---:|---:|
| Fixed RGB | 141 | 49 | 81 | 242 | 66.837 |
| Fixed thermal | 235 | 158 | 212 | 242 | 67.622 |
| RGB-first | 242 | 125 | 180 | 343 | 94.971 |
| Thermal-first | 242 | 164 | 218 | 249 | 69.570 |

Thermal-first closes much of the RGB-first deficit here. The seven RGB requests
after thermal failure add six agreeing outputs at each bound beyond fixed thermal,
with one added output exceeding 5°. Its projected moving-subset processing is
26.75% lower than RGB-first, but this is a cached request projection, not measured
live latency or established equal-pose-accuracy computation savings.

| All 971 anchors, learned | Outputs | Calls | Projected pair seconds |
|---|---:|---:|---:|
| RGB-first | 840 | 1,389 | 383.278 |
| Thermal-first | 840 | 1,145 | 321.477 |

Across the full recording, calls fall 17.57% and projected seconds 16.12% relative
to RGB-first. On the 964 common-gyro anchors, thermal-first agrees 713/811 times
within 1°/5°, versus RGB-first 646/773 and fixed thermal 670/768. All seven
non-common anchors remain in full output/call accounting. Fixed thirds use the
original index partition, with eligible gyro populations 318/324/322.

SIFT also improves with reversed priority: moving RGB-first/thermal-first
agreements are 72/92 within 1° and 136/154 within 5°, with identical 181 outputs.
The strict SIFT result merely matches fixed thermal's 92, not an increment over
the strongest fixed camera. Its full-record calls change 1,668→1,554 and projected
seconds 54.397→46.155. No sensor must win every interval or frontend.

## How much quality-selection opportunity remains?

The hindsight oracle counts an anchor if either available camera agrees at the
specified bound. It reads gyro disagreement labels and is not deployable selection.

| Learned moving endpoint | Best fixed | Thermal-first | Oracle either-agrees | Oracle beyond thermal-first |
|---|---:|---:|---:|---:|
| Within 1° /242 | 158 | 164 | 188 | 24 |
| Within 5° /242 | 212 | 218 | 228 | 10 |

Among 134 both-supported moving anchors, at 1° there are 63 RGB-disagrees/
thermal-agrees cases and 24 of the reverse; at 5°, 48 versus 10. Priority reversal
benefits from the imbalance, but cannot identify the reverse cases. Both cameras
disagree on 28 both-supported anchors at 1° and 11 at 5°. Including single-camera
cases, no available proposal agrees on 54/14 moving anchors at the two bounds;
a selector cannot repair those underlying proposals.

Across all 964 common-gyro anchors, remaining oracle headroom beyond thermal-first
is 59 strict or 10 coarse agreements. Do not describe this as achieved improvement
or evidence that a fitted cue can recover it. At 1°, unknown gyro bias/noise and
nominal camera/exposure binding remain particularly relevant limitations.

## Historical cross-record limits: bounds, not new paired replays

Local [FlashLight](flashlight-transfer-findings-2026-11-16.md) and
[SmokeRoom](smoke-rgb-first-findings-2026-11-16.md) moving-cohort tables provide
enough counts to bound thermal-first without inventing unavailable paired rows.
Those targets are supplied-reference rotation errors; OutdoorNight uses nominal
gyro agreement. They are separate results, with no pooled accuracy statistic.

| Historical moving population | Thermal-first possible within 1° | Observed RGB-first within 1° | Thermal-first possible within 5° | Observed RGB-first within 5° |
|---|---:|---:|---:|---:|
| FlashLight, 192 anchors | 65–81 | 176 | 127–134 | 185 |
| SmokeRoom, 236 anchors | 116–121 | 155 | 190–195 | 192 |

Even the most favorable compatible thermal-first pairing loses to RGB-first on
FlashLight at both bounds and SmokeRoom at 1°. SmokeRoom's coarse ordering is
undetermined from these counts: the interval includes values below and above 192.
Thus a universal thermal-priority replacement is contradicted by existing evidence;
this does not establish an adaptive selector or new raw transfer result.

The bounds are tight: thermal-first keeps all thermal agreements and can add only
RGB-correct cases among RGB-only outputs. In FlashLight there are 24 RGB-only
outputs; in SmokeRoom only five. The exact formula and its required counts are
recorded in the protocol. No historical inference or choice replay was performed.

## Inventory review, verification and decision

EXP078 already characterized uncertainty, retained fraction, coverage and prior
support; EXP087 forward/backward closure and EXP112 geometry/history also retained
mixed or failing fixed-camera comparisons. This control does not repackage them
as a new cue. It establishes a stronger simple OutdoorNight baseline before any
new quality-selection study.

Ten CPU tests pass, including exhaustive enumeration of all 6,561 four-case
availability/agreement assignments, swapped-priority/unsupported truth tables,
failed-call costs, oracle semantics and independent-audit rejection of four
corruptions (choice, cost, oracle, documentary bound). The independent auditor
passes 19,672 checks, reconstructing full populations, original RGB-first parity,
sensor/time pointers, calls/times, every gyro cohort, conflict/oracle counts and
historical bounds via independent feasible allocations and source-table checks.

Every prior prediction, gyro target, disagreement and original choice remains
unchanged. Source/findings/protocol/input pins precede choices; labels are joined
only after the new choice lock. Exact old checkpoints/runtime remain inherited
prerequisites but are not executed. This is nominal angular agreement on one
exposed recording plus separate documentary bounds, not physical pose/translation
truth, blind generalization or a stateful VO/energy/latency benchmark.

Evidence/PNG/PDF: `results/camera-priority-controls/`. A member-readback verified
incremental packet is under
`/root/ore-ms-artifacts/camera-priority-controls-2026-11-16/`, with explicit pinned
parent matching/gyro prerequisites. No Box upload, author outreach or Git publication.

Stop this priority comparison without adoption or a heuristic rescue. A next
quality-selection study needs recording-level data and fixed RGB, fixed thermal,
both priority orders and unchanged error/cost endpoints. Historical summary bounds
are sufficient to reject universal priority, but not to train or validate a new
gate. Restore/qualify actual paired multi-record evidence before such a study;
retain OutdoorNight as exposed development and preserve its gyro-only limitation.
